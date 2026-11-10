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
use crate::kinematics::{self, Pose};
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
    pub async fn pose(&self) -> Pose { kinematics::fk(&kinematics::q6(&self.sup.inner().read().await.commanded_positions)) }

    /// Wait for the installed sub-motion; stop it on a timeout.
    pub async fn wait_sub(&self, phase: &str, limit_s: f64) -> Result<SubDone, Stop> {
        let until = Instant::now() + Duration::from_secs_f64(limit_s.min(self.phase_timeout_s));
        loop {
            tokio::time::sleep(POLL).await;
            let mut inner = self.sup.inner().write().await;
            if !inner.macro_alive(self.id) { return Err(Stop::Cancelled); }
            match inner.macro_sub(self.id) {
                Some(SubState::Done(d)) => return Ok(d),
                Some(SubState::Failed { code, message }) => return Err(Stop::Fail(code, message)),
                _ => {}
            }
            let now = Instant::now();
            if now > until || now > self.deadline {
                inner.stop_motion();
                return Err(fail(self.timeout_code, format!("Phase {phase} did not finish in time")));
            }
        }
    }

    /// Plan (outside the lock) a Cartesian path from the current commanded state, install it
    /// as a sub-motion and wait for it.
    pub async fn move_arm(&self, phase: &str, spec: GoalSpec, speed: Speed) -> Result<SubDone, Stop> {
        let input = {
            let inner = self.sup.inner().read().await;
            if !inner.macro_alive(self.id) { return Err(Stop::Cancelled); }
            inner.plan_input().map_err(arm_fail)?
        };
        let goal = motion::resolve_goal(&input, &spec);
        let (input, planned) = tokio::task::spawn_blocking(move || { let p = motion::plan_cartesian(&input, goal, speed); (input, p) }).await
            .map_err(|e| fail("INTERNAL", format!("planner task failed: {e}")))?;
        let path = planned.map_err(|e| arm_fail(e.into()))?;
        let limit = {
            let mut inner = self.sup.inner().write().await;
            if !inner.macro_alive(self.id) { return Err(Stop::Cancelled); }
            inner.install_macro_path(self.id, &input, path).map_err(arm_fail)?
        };
        self.wait_sub(phase, limit * 2.0 + 5.0).await
    }

    /// Run a gripper open / close sub-motion and wait for it.
    pub async fn gripper(&self, phase: &str, command: GripperCommand) -> Result<SubDone, Stop> {
        let request = GripperRequest { command, fraction: None, grasp: command == GripperCommand::Close, speed: 1.0, timeout_s: 8.0, tag: None };
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
