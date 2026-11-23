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
//! # Child primitives (armd 1.6 RT core, track E)
//!
//! Every installed sub-motion is a *child* of the macro: [`MacroRun::children`] counts the
//! physically installed ones and [`MacroRun::child`] is the RT record of the current one
//! ([`ChildRecord`]: sequence number, deadline, timeout result, contact guard). The worker
//! holds a [`ChildTicket`] (parent action id + child sequence) and only accepts the result of
//! that child; a cancelled or replaced parent reads as [`Stop::Cancelled`].
//!
//! The control tick (`reconcile_action`) decides what must be decided before further
//! movement, without the worker taking the write lock: it ends the child at its deadline
//! (`Failed` with the macro's timeout code, the motion stopped) and, with a [`ContactGuard`]
//! stops a descent at the measured position on contact ([`ContactWatch`]).
//!
//! The broker's child primitives (`rt_core::ops::Primitive::{CartesianPath, Gripper,
//! JointQuinticLimited, Replay}`, each with a [`ChildSpec`]) install through
//! `SupervisorInner::commit_macro_child` with the same record; a macro that ends leaves a
//! `rt_core::MacroTerminal` (outcome with its code, child count) in the core.
//!
//! [`SupervisorInner::install_macro_path`]: crate::supervisor::SupervisorInner::install_macro_path
//! [`SupervisorInner::start_macro_gripper`]: crate::supervisor::SupervisorInner::start_macro_gripper
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::api::rest::ApiError;
use crate::kinematics::Pose;
use crate::motion::{self, GoalFrame, GoalSpec, GripperCommand, GripperRequest, Speed};
use crate::supervisor::{ArmError, ArmSupervisor, SupervisorInner};
pub use crate::rt_core::broker::ActionId;
/// The child identity and contact guard of the RT contract (`rt_core::ops`).
pub use crate::rt_core::ops::{ChildSpec, ChildTicket, ContactGuard};

/// Poll period of the orchestrator while a sub-motion runs.
pub const POLL: Duration = Duration::from_millis(20);
/// The worker ends an overdue child itself only when no control tick did within this.
pub const OVERDUE_GRACE: Duration = Duration::from_millis(500);

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
    /// State of the current child ([`Self::child`]).
    pub sub: SubState,
    /// `/motion/stop` arrived while no sub-motion was running.
    pub stop_requested: bool,
    pub phase: Option<String>,
    pub phase_start: Option<Instant>,
    /// Children physically installed so far.
    pub children: u32,
    /// RT record of the current (last installed) child.
    pub child: ChildRecord,
    /// Contact guard of the next installed child ([`ContactGuard`]).
    pub next_guard: Option<ContactGuard>,
}

impl MacroRun {
    /// A run without children.
    pub fn new(id: u64, epoch: u64) -> Self {
        Self { id, epoch, sub: SubState::Idle, stop_requested: false, phase: None, phase_start: None,
            children: 0, child: ChildRecord::default(), next_guard: None }
    }

    /// A child was installed: the next sequence number, the pending guard, state Running.
    pub fn begin_child(&mut self) {
        self.children += 1;
        self.child = ChildRecord { seq: self.child.seq.wrapping_add(1), guard: self.next_guard.take(), ..ChildRecord::default() };
        self.sub = SubState::Running;
    }

    /// The ticket of the current child (None before the first).
    pub fn ticket(&self) -> Option<ChildTicket> {
        (self.children > 0).then_some(ChildTicket { parent: ActionId(self.id), child_seq: self.child.seq })
    }
}

/// RT record of a macro's current child.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ChildRecord {
    /// 1 for the first child of the run.
    pub seq: u32,
    /// The tick fails the child (code and message of `timeout`) when it still runs after this.
    pub deadline: Option<Instant>,
    pub timeout: Option<(String, String)>,
    pub guard: Option<ContactGuard>,
    pub watch: ContactWatch,
}

/// What the contact guard ([`ContactGuard`]) of a child saw, evaluated by the control tick
/// while the child's motion runs: peak |residual| of J2 / J3 (grasp diagnostics), peak
/// J1..J(arm) tracking error, and where a residual above its bound ended the child.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ContactWatch {
    pub peak_residual_nm: Option<[f32; 2]>,
    pub peak_tracking_rad: Option<f32>,
    /// Base-frame TCP height (m) where the contact stop ended the child.
    pub contact_tcp_z_m: Option<f64>,
    /// The joint whose residual exceeded its bound.
    pub contact_joint: Option<usize>,
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

    /// FK of the commanded J1..J6 in `end_link` frame.
    pub async fn pose(&self) -> Pose {
        let inner = self.sup.inner().read().await;
        let arm = &inner.config.arm_model;
        arm.fk_f32(&inner.commanded_positions).unwrap_or_else(|| arm.fk(&[0.0; crate::chain::MAX_CHAIN_JOINTS]))
    }

    /// FK of the commanded J1..J6 in the active TCP frame.
    pub async fn tcp_pose(&self) -> Pose {
        let inner = self.sup.inner().read().await;
        inner.config.tcp_pose_f32(&inner.commanded_positions).unwrap_or_else(|| {
            inner.config.end_to_tcp(&inner.config.arm_model.fk(&[0.0; crate::chain::MAX_CHAIN_JOINTS]))
        })
    }

    /// RT deadline of a child that may run `limit_s` (at most `phase_timeout_s`, at most the
    /// whole-action deadline).
    fn child_deadline(&self, limit_s: f64) -> Instant {
        let s = limit_s.min(self.phase_timeout_s);
        let s = if s.is_finite() { s.clamp(0.0, 1e6) } else { 1e6 };
        (Instant::now() + Duration::from_secs_f64(s)).min(self.deadline)
    }

    /// Under the install guard, right after a child of this macro was installed: arms its RT
    /// deadline (`limit_s`, see [`child_deadline`](Self::child_deadline)) and returns its ticket.
    /// None when no child of this macro is installed.
    pub fn arm_child(&self, inner: &mut SupervisorInner, phase: &str, limit_s: f64) -> Option<ChildTicket> {
        inner.arm_macro_child(self.id, self.child_deadline(limit_s), self.timeout_code, format!("Phase {phase} did not finish in time"))
    }

    /// Wait for the installed sub-motion for at most `limit_s` (arms its RT deadline first; an
    /// install site that holds the guard uses [`arm_child`](Self::arm_child) and
    /// [`wait_child`](Self::wait_child) instead).
    pub async fn wait_sub(&self, phase: &str, limit_s: f64) -> Result<SubDone, Stop> {
        let ticket = {
            let mut inner = self.sup.inner().write().await;
            if !inner.macro_alive(self.id) { return Err(Stop::Cancelled); }
            self.arm_child(&mut inner, phase, limit_s)
        };
        let Some(ticket) = ticket else { return Err(Stop::Cancelled); };
        self.wait_child(ticket).await
    }

    /// Wait for child `ticket` with the read lock only: its result, [`Stop::Cancelled`] when
    /// the parent ended or was replaced. The control tick ends an overdue child (its timeout
    /// failure); only when no tick did so for [`OVERDUE_GRACE`] does the worker end it.
    pub async fn wait_child(&self, ticket: ChildTicket) -> Result<SubDone, Stop> {
        loop {
            tokio::time::sleep(POLL).await;
            let overdue = {
                let inner = self.sup.inner().read().await;
                match inner.macro_child_state(ticket) {
                    None => return Err(Stop::Cancelled),
                    Some(SubState::Done(d)) => return Ok(d),
                    Some(SubState::Failed { code, message }) => return Err(Stop::Fail(code, message)),
                    Some(_) => inner.macro_child_overdue(ticket, Instant::now(), OVERDUE_GRACE),
                }
            };
            if overdue { self.sup.inner().write().await.expire_macro_child(Instant::now()); }
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
        self.move_arm_staged_in(GoalFrame::EndLink, phase, spec, speed, approach).await
    }

    /// [`move_arm_with`](Self::move_arm_with) in an explicit [`GoalFrame`] (`EndLink` or `Tcp`).
    pub async fn move_arm_with_in(&self, frame: GoalFrame, phase: &str, spec: GoalSpec, speed: Speed, approach: Option<FinalApproach>) -> Result<SubDone, Stop> {
        self.move_arm_staged_in(frame, phase, spec, speed, approach).await.map_err(|(stop, _)| stop)
    }

    /// [`move_arm`](Self::move_arm) in an explicit [`GoalFrame`] (`EndLink` or `Tcp`).
    pub async fn move_arm_in(&self, frame: GoalFrame, phase: &str, spec: GoalSpec, speed: Speed) -> Result<SubDone, Stop> {
        self.move_arm_with_in(frame, phase, spec, speed, None).await
    }

    /// [`move_arm_staged`](Self::move_arm_staged) in an explicit [`GoalFrame`].
    pub async fn move_arm_staged_in(&self, frame: GoalFrame, phase: &str, spec: GoalSpec, speed: Speed, approach: Option<FinalApproach>) -> Result<SubDone, (Stop, bool)> {
        self.move_arm_staged_in_with_table(frame, phase, spec, speed, approach, None).await
    }

    /// [`move_arm_staged_in`](Self::move_arm_staged_in) with an optional table-clearance bound
    /// enforced on the actually installed joint path (including direct fallbacks).
    pub async fn move_arm_staged_in_with_table(
        &self,
        frame: GoalFrame,
        phase: &str,
        spec: GoalSpec,
        speed: Speed,
        approach: Option<FinalApproach>,
        table_clearance: Option<(Vec<crate::kinematics::Vec3>, f64)>,
    ) -> Result<SubDone, (Stop, bool)> {
        let before = |s: Stop| (s, false);
        let (input, plan) = {
            let inner = self.sup.inner().read().await;
            if !inner.macro_alive(self.id) { return Err(before(Stop::Cancelled)); }
            let plan = approach.as_ref().map(|a| inner.approach_plan(a.use_, a.request));
            (inner.plan_input_in(frame).map_err(arm_fail).map_err(before)?, plan)
        };
        let effective_rules: Option<crate::backlash::DescentRules> = match (&approach, &table_clearance) {
            (Some(a), Some(t)) => {
                let mut r = a.rules.clone();
                if r.table.is_none() { r.table = Some(t.clone()); }
                Some(r)
            }
            (Some(a), None) => Some(a.rules.clone()),
            (None, Some(t)) => Some(crate::backlash::DescentRules { table: Some(t.clone()), ..Default::default() }),
            (None, None) => None,
        };
        let (goal, opts) = motion::resolve_goal_opts(&input, &spec);
        let rules_for_pre = effective_rules.clone();
        let (input, planned) = tokio::task::spawn_blocking(move || {
            let p = motion::plan_cartesian_opts(&input, goal, speed, opts).map(|direct| {
                let pre = match &plan {
                    Some(Ok(plan)) => crate::backlash::plan_cartesian_pre(&input, &direct, plan,
                        rules_for_pre.as_ref(), speed, opts),
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
        let ticket = {
            let mut inner = self.sup.inner().write().await;
            if !inner.macro_alive(self.id) { return Err(before(Stop::Cancelled)); }
            match inner.install_macro_path_same_side_with_rules(self.id, &input, path, pre, plan.as_ref(), effective_rules.as_ref()) {
                Ok(l) => self.arm_child(&mut inner, phase, l * 2.0 + 5.0),
                Err(e) => {
                    if let Some(detail) = e.workspace_detail() {
                        inner.set_action_workspace_violation(self.id, detail);
                    }
                    return Err(before(arm_fail(e)));
                }
            }
        };
        let t0 = Instant::now();
        let sub_res = match ticket { Some(t) => self.wait_child(t).await, None => Err(Stop::Cancelled) };
        let ms = t0.elapsed().as_millis() as u32;
        {
            let mut inner = self.sup.inner().write().await;
            let meas_opt = inner.config.arm_model.fk_f32(&inner.measured_positions).map(|p| motion::end_to_frame(&p, &input));
            let cmd_opt = inner.config.arm_model.fk_f32(&inner.commanded_positions).map(|p| motion::end_to_frame(&p, &input));
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

    /// Reconcile a planned gripper sub-motion with the remaining macro budget before installation,
    /// arm its RT child deadline under the same write lock, and wait on its `ChildTicket`:
    /// pos-force closes arm up to `(timeout_s + 10 s).min(self.deadline)` (not clamped to
    /// `phase_timeout_s`), while pos-vel and opening moves use `arm_child` (`phase_timeout_s`).
    async fn run_gripper(&self, phase: &str, request: GripperRequest) -> Result<SubDone, Stop> {
        let ticket = {
            let mut inner = self.sup.inner().write().await;
            if !inner.macro_alive(self.id) { return Err(Stop::Cancelled); }
            let now = Instant::now();
            if now >= self.deadline {
                return Err(fail(self.timeout_code, self.timeout_message.clone()));
            }
            let remaining_s = self.deadline.duration_since(now).as_secs_f64();
            let (duration, timeout_s, is_pos_force_close) = inner
                .start_macro_gripper_within(self.id, request, Some(remaining_s), true)
                .map_err(|e| match e {
                    ArmError::Motion(crate::motion::MotionError::InvalidCommand(msg))
                        if msg.contains("remaining macro budget") =>
                    {
                        fail(self.timeout_code, format!("Phase {phase}: {msg}"))
                    }
                    other => arm_fail(other),
                })?;
            if is_pos_force_close {
                let s = if timeout_s.is_finite() { (timeout_s + 10.0).clamp(0.0, 1e6) } else { 1e6 };
                let deadline = (now + Duration::from_secs_f64(s)).min(self.deadline);
                inner.arm_macro_child(self.id, deadline, self.timeout_code, format!("Phase {phase} did not finish in time"))
            } else {
                self.arm_child(&mut inner, phase, duration + 10.0)
            }
        };
        self.wait_child(ticket.ok_or(Stop::Cancelled)?).await
    }

    /// Run a gripper open / close sub-motion and wait for it.
    pub async fn gripper(&self, phase: &str, command: GripperCommand) -> Result<SubDone, Stop> {
        let request = GripperRequest { command, fraction: None, grasp: command == GripperCommand::Close, speed: 1.0, timeout_s: Some(8.0), tag: None };
        self.run_gripper(phase, request).await
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
        self.run_gripper(phase, request).await
    }

    /// [`phase`](Self::phase) followed by [`gripper`](Self::gripper).
    pub async fn gripper_phase(&self, phase: &str, progress: f32, command: GripperCommand) -> Result<SubDone, Stop> {
        self.phase(phase, progress).await?;
        self.gripper(phase, command).await
    }
}
