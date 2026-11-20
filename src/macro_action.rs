//! Macro actions: one action record (`grasp`, `place`) driven by an async orchestrator
//! task outside the supervisor lock and the control tick.
//!
//! The orchestrator runs a sequence of sub-motions (Cartesian paths, gripper motions).
//! Each one is installed under the write lock only while the action is still running
//! ([`SupervisorInner::install_macro_path`], [`SupervisorInner::start_macro_gripper`]) and
//! carries the action id, so stop / E-stop / lease loss / supersede end the action between
//! or during sub-motions without leaving a half-issued motion. The supervisor keeps one
//! [`MacroRun`] (the running macro) keyed by action id; `reconcile_action` treats any action
//! with a matching run the same way, whatever its kind.
//!
//! [`SupervisorInner::install_macro_path`]: crate::supervisor::SupervisorInner::install_macro_path
//! [`SupervisorInner::start_macro_gripper`]: crate::supervisor::SupervisorInner::start_macro_gripper
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::api::rest::ApiError;
use crate::kinematics::Pose;
use crate::motion::{self, GoalSpec, GripperCommand, GripperRequest, Speed};
use crate::supervisor::{ArmError, ArmSupervisor};

/// Poll period of the orchestrator while a sub-motion runs.
pub const POLL: Duration = Duration::from_millis(20);

/// How an orchestrator phase ended early.
#[derive(Debug, Clone, PartialEq)]
pub enum Stop {
    /// The action already ended (stop, E-stop, lease, supersede): exit without touching it.
    Cancelled,
    Fail(String, String),
}

pub fn fail(code: &str, message: impl Into<String>) -> Stop { Stop::Fail(code.into(), message.into()) }

/// Failure carrying the API code of an [`ArmError`] (IK_UNREACHABLE, WORKSPACE_VIOLATION, ...).
pub fn arm_fail(e: ArmError) -> Stop {
    let api = ApiError::from(e);
    Stop::Fail(api.code.unwrap_or_else(|| api.error.clone()), api.message)
}

/// Result of a finished sub-motion.
#[derive(Debug, Clone, PartialEq)]
pub struct SubDone {
    pub settled: Option<bool>,
    pub holding: Option<bool>,
    pub gripper_gap_rad: Option<f32>,
    pub at: Instant,
}

#[derive(Debug, Clone, PartialEq)]
pub enum SubState {
    Idle,
    Running,
    Done(SubDone),
    Failed { code: String, message: String },
}

/// Supervisor-side record of the running macro action.
#[derive(Debug, Clone, PartialEq)]
pub struct MacroRun {
    pub id: u64,
    pub epoch: u64,
    pub sub: SubState,
    /// `/motion/stop` arrived while no sub-motion was running.
    pub stop_requested: bool,
    pub phase: Option<String>,
    pub phase_start: Option<Instant>,
}

/// Same-side final approach of a macro sub-motion ([`MacroCtx::move_arm_with`]).
#[derive(Debug, Clone)]
pub struct FinalApproach {
    /// `grasp_final` | `place_final`
    pub use_: &'static str,
    pub request: crate::backlash::Request,
    /// From-above and table rules of the descent.
    pub rules: crate::backlash::DescentRules,
}

/// Orchestrator handle of one macro action: alive checks, phase reports, sub-motions.
pub struct MacroCtx {
    pub sup: Arc<ArmSupervisor>,
    pub id: u64,
    /// Whole-action deadline.
    pub deadline: Instant,
    /// Upper bound for one sub-motion wait.
    pub phase_timeout_s: f64,
    /// Failure code on a deadline / phase timeout (GRASP_TIMEOUT, PLACE_TIMEOUT).
    pub timeout_code: &'static str,
    /// Message for the whole-action deadline.
    pub timeout_message: String,
}

impl MacroCtx {
    pub async fn alive(&self) -> bool { self.sup.inner().read().await.macro_alive(self.id) }

    /// Cancelled when the action ended, a timeout failure past the deadline.
    pub async fn check(&self) -> Result<(), Stop> {
        if !self.alive().await { return Err(Stop::Cancelled); }
        if Instant::now() > self.deadline { return Err(fail(self.timeout_code, self.timeout_message.clone())); }
        Ok(())
    }

    /// Report `phase` in `action.phase` (after a [`check`](Self::check)).
    pub async fn phase(&self, phase: &str, progress: f32) -> Result<(), Stop> {
        self.check().await?;
        self.sup.inner().write().await.set_action_phase(self.id, phase, progress, None, None);
        Ok(())
    }

    /// FK of the commanded J1..J6.
    pub async fn pose(&self) -> Pose {
        let inner = self.sup.inner().read().await;
        let arm = &inner.config.arm_model;
        arm.fk_f32(&inner.commanded_positions).unwrap_or_else(|| arm.fk(&[0.0; crate::chain::MAX_CHAIN_JOINTS]))
    }

    /// Wait for the installed sub-motion; stop it on a timeout.
    pub async fn wait_sub(&self, phase: &str, limit_s: f64) -> Result<SubDone, Stop> {
        let until = Instant::now() + Duration::from_secs_f64(limit_s.min(self.phase_timeout_s));
        loop {
            tokio::time::sleep(POLL).await;
            {
                // poll with the read lock; the write lock only to stop on a timeout
                let inner = self.sup.inner().read().await;
                if !inner.macro_alive(self.id) { return Err(Stop::Cancelled); }
                match inner.macro_sub(self.id) {
                    Some(SubState::Done(d)) => return Ok(d),
                    Some(SubState::Failed { code, message }) => return Err(Stop::Fail(code, message)),
                    _ => {}
                }
            }
            let now = Instant::now();
            if now > until || now > self.deadline {
                let mut inner = self.sup.inner().write().await;
                if !inner.macro_alive(self.id) { return Err(Stop::Cancelled); }
                if let Some(SubState::Done(d)) = inner.macro_sub(self.id) { return Ok(d); }
                if inner.active_trajectory.as_ref().is_some_and(|a| a.action_id == Some(self.id)) { inner.stop_motion(); }
                return Err(fail(self.timeout_code, format!("Phase {phase} did not finish in time")));
            }
        }
    }

    /// Plan (outside the lock) a Cartesian path from the current commanded state, install it
    /// as a sub-motion and wait for it.
    pub async fn move_arm(&self, phase: &str, spec: GoalSpec, speed: Speed) -> Result<SubDone, Stop> {
        self.move_arm_with(phase, spec, speed, None).await
    }

    /// [`move_arm`](Self::move_arm) ending with the same-side approach of `approach`
    /// (src/backlash.rs): the path goes to the pre-target, the short segment (phase
    /// `final_approach`) to the goal; the outcome is in the action record.
    pub async fn move_arm_with(&self, phase: &str, spec: GoalSpec, speed: Speed, approach: Option<FinalApproach>) -> Result<SubDone, Stop> {
        self.move_arm_staged(phase, spec, speed, approach).await.map_err(|(stop, _)| stop)
    }

    /// [`move_arm_with`](Self::move_arm_with) telling a refusal before any motion (`false`:
    /// planning or install refused, the arm did not move) from a failure after the sub-motion
    /// was installed (`true`).
    pub async fn move_arm_staged(&self, phase: &str, spec: GoalSpec, speed: Speed, approach: Option<FinalApproach>) -> Result<SubDone, (Stop, bool)> {
        let before = |s: Stop| (s, false);
        let (input, plan) = {
            let inner = self.sup.inner().read().await;
            if !inner.macro_alive(self.id) { return Err(before(Stop::Cancelled)); }
            let plan = approach.as_ref().map(|a| inner.approach_plan(a.use_, a.request));
            (inner.plan_input().map_err(arm_fail).map_err(before)?, plan)
        };
        let (goal, yaw_free) = motion::resolve_goal_ex(&input, &spec);
        let (input, planned) = tokio::task::spawn_blocking(move || {
            let p = motion::plan_cartesian_ex(&input, goal, speed, yaw_free).map(|direct| {
                let pre = match &plan {
                    Some(Ok(plan)) => crate::backlash::plan_cartesian_pre(&input, &direct, plan,
                        approach.as_ref().map(|a| &a.rules), speed, yaw_free),
                    Some(Err(Some(r))) => crate::backlash::Pre::Direct((*r).to_string()),
                    _ => crate::backlash::Pre::Off,
                };
                (direct, pre, plan.and_then(Result::ok))
            });
            (input, p)
        }).await
            .map_err(|e| before(fail("INTERNAL", format!("planner task failed: {e}"))))?;
        let (path, pre, plan) = match planned {
            Ok(p) => p,
            Err(e) => {
                if let motion::MotionError::WorkspaceViolation { ref detail, .. } = e {
                    self.sup.inner().write().await.set_action_workspace_violation(self.id, (**detail).clone());
                }
                return Err(before(arm_fail(e.into())));
            }
        };
        let limit = {
            let mut inner = self.sup.inner().write().await;
            if !inner.macro_alive(self.id) { return Err(before(Stop::Cancelled)); }
            match inner.install_macro_path_same_side(self.id, &input, path, pre, plan.as_ref()) {
                Ok(l) => l,
                Err(e) => {
                    if let Some(detail) = e.workspace_detail() {
                        inner.set_action_workspace_violation(self.id, detail);
                    }
                    return Err(before(arm_fail(e)));
                }
            }
        };
        let t0 = Instant::now();
        let sub_res = self.wait_sub(phase, limit * 2.0 + 5.0).await;
        let ms = t0.elapsed().as_millis() as u32;
        {
            let mut inner = self.sup.inner().write().await;
            let arm_model = inner.config.arm_model.clone();
            let meas_opt = arm_model.fk_f32(&inner.measured_positions);
            let cmd_opt = arm_model.fk_f32(&inner.commanded_positions);
            if let (Some(meas), Some(cmd)) = (meas_opt, cmd_opt) {
                let track_mm = (crate::kinematics::norm(&crate::kinematics::sub(&meas.p, &cmd.p)) * 1000.0) as f32;
                let track_deg = crate::kinematics::angle_between(&meas.r, &cmd.r).to_degrees() as f32;
                let goal_mm = (crate::kinematics::norm(&crate::kinematics::sub(&meas.p, &goal.p)) * 1000.0) as f32;
                let goal_deg = crate::kinematics::angle_between(&meas.r, &goal.r).to_degrees() as f32;
                let contact = inner.last_action.as_ref().filter(|a| a.id == self.id).and_then(|a| a.descend_contact);
                if let Some(a) = inner.last_action.as_mut().filter(|a| a.id == self.id) {
                    if a.motions.len() < 32 {
                        a.motions.push(motion::MacroMotionDiag {
                            phase: phase.to_string(),
                            ms,
                            track_mm,
                            track_deg,
                            goal_mm,
                            goal_deg,
                            contact,
                        });
                    }
                }
            }
        }
        sub_res.map_err(|s| (s, true))
    }

    /// Run a gripper open / close sub-motion and wait for it.
    pub async fn gripper(&self, phase: &str, command: GripperCommand) -> Result<SubDone, Stop> {
        let request = GripperRequest { command, fraction: None, grasp: command == GripperCommand::Close, speed: 1.0, timeout_s: Some(8.0), tag: None };
        let limit = {
            let mut inner = self.sup.inner().write().await;
            if !inner.macro_alive(self.id) { return Err(Stop::Cancelled); }
            inner.start_macro_gripper(self.id, request).map_err(arm_fail)?
        };
        self.wait_sub(phase, limit + 10.0).await
    }

    /// Run a gripper move sub-motion with a fraction in [0, 1] and wait for it.
    pub async fn gripper_move(&self, phase: &str, fraction: f64) -> Result<SubDone, Stop> {
        let request = GripperRequest {
            command: GripperCommand::Move,
            fraction: Some(fraction),
            grasp: false,
            speed: 1.0,
            timeout_s: Some(8.0),
            tag: None,
        };
        let limit = {
            let mut inner = self.sup.inner().write().await;
            if !inner.macro_alive(self.id) { return Err(Stop::Cancelled); }
            inner.start_macro_gripper(self.id, request).map_err(arm_fail)?
        };
        self.wait_sub(phase, limit + 10.0).await
    }

    /// [`phase`](Self::phase) followed by [`gripper`](Self::gripper).
    pub async fn gripper_phase(&self, phase: &str, progress: f32, command: GripperCommand) -> Result<SubDone, Stop> {
        self.phase(phase, progress).await?;
        self.gripper(phase, command).await
    }
}
