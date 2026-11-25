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
use crate::supervisor::{AdmissionStamp, ArmError, ArmSupervisor, SupervisorInner};
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
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SubDone {
    pub settled: Option<bool>,
    pub holding: Option<bool>,
    pub gripper_gap_rad: Option<f32>,
    pub at: Instant,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SubState {
    Idle,
    Running,
    Done(SubDone),
    Failed { code: crate::safety::BoundedText<48>, message: crate::motion::StepErrorText },
}

/// Supervisor-side record of the running macro action.
#[derive(Debug, Clone, PartialEq)]
pub struct MacroRun {
    pub id: u64,
    pub epoch: u64,
    /// Admission stamp captured ONCE when the parent macro was admitted; checked on every child.
    pub stamp: AdmissionStamp,
    /// State of the current child ([`Self::child`]).
    pub sub: SubState,
    /// `/motion/stop` arrived while no sub-motion was running.
    pub stop_requested: bool,
    /// True once the owner has entered its terminal finishing transition (explicit finish,
    /// parent deadline expiry, or stop) while awaiting bus acceptance of the replacement hold;
    /// rejects any duplicate `FinishMacro` or new `InstallChild` with `PARENT_REPLACED`.
    pub finishing: bool,
    /// True after a child motion was quiesced on a driver-backed RT owner until a subsequent
    /// tick's replacement hold output is accepted on the bus.
    pub hold_pending_bus_accept: bool,
    pub phase: Option<crate::model::HolderLabel>,
    pub phase_start: Option<Instant>,
    /// Optional owner-enforced whole-macro deadline and failure code.
    pub deadline: Option<Instant>,
    pub timeout_code: crate::safety::BoundedText<48>,
    /// Children physically installed so far.
    pub children: u32,
    /// RT record of the current (last installed) child.
    pub child: ChildRecord,
    /// Contact guard of the next installed child ([`ContactGuard`]).
    pub next_guard: Option<ContactGuard>,
    /// Phase B adaptive macro invariant: J7 `PosForceIntent::Target` permission is strictly
    /// restricted to active `GripperMotion` children; `BeginMacro`, arm-only children, and
    /// inter-child gaps force `j7_admitted_stamp = None` (PosForce `Hold`).
    pub j7_intent_only: bool,
}

impl MacroRun {
    /// A run without children.
    pub fn new(id: u64, epoch: u64, stamp: AdmissionStamp) -> Self {
        Self { id, epoch, stamp, sub: SubState::Idle, stop_requested: false, finishing: false,
            hold_pending_bus_accept: false, phase: None, phase_start: None,
            deadline: None, timeout_code: crate::safety::BoundedText::empty(),
            children: 0, child: ChildRecord::default(), next_guard: None,
            j7_intent_only: false }
    }

    /// Sets an owner-enforced whole-macro deadline and timeout failure code.
    pub fn with_deadline(mut self, deadline: Option<Instant>, timeout_code: &str) -> Self {
        self.deadline = deadline;
        self.timeout_code = crate::safety::BoundedText::new(timeout_code);
        self
    }

    /// Next strictly monotonic child sequence number (never wraps to 0).
    pub fn next_child_seq(&self) -> Option<u32> {
        self.child.seq.checked_add(1)
    }

    /// A child was installed: the next sequence number, the pending guard, state Running.
    pub fn begin_child(&mut self) {
        let next = self.child.seq.checked_add(1).expect("macro child sequence exhausted");
        self.children = self.children.saturating_add(1);
        self.child = ChildRecord { seq: next, guard: self.next_guard.take(), ..ChildRecord::default() };
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
    pub timeout: Option<(crate::safety::BoundedText<48>, crate::motion::StepErrorText)>,
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
    /// Parent macro admission stamp passed into every child motion start (never re-sampled).
    pub stamp: AdmissionStamp,
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
    /// Whether this supervisor is currently owned by the synchronous RT executor (`OWNER_EXECUTOR`),
    /// in which case all reads use published [`StateHub`] snapshots and all child/finish actions
    /// go through the RT broker without ever taking `SupervisorLock`.
    #[inline]
    pub fn is_executor(&self) -> bool {
        self.sup.owner_mode() == crate::supervisor::OWNER_EXECUTOR
    }

    /// Derives a fresh single-use child [`AdmissionStamp`] preserving the parent macro's
    /// admission-captured authority without re-sampling live owner state.
    #[inline]
    pub fn next_child_stamp(&self) -> AdmissionStamp {
        self.stamp.for_macro_child()
    }

    /// Captures a coherent planning snapshot (`StateHub` on `OWNER_EXECUTOR`, brief read lock on
    /// `OWNER_LEGACY`) carrying a fresh child `AdmissionStamp`.
    pub async fn snapshot(
        &self,
        deadline: Instant,
    ) -> Result<crate::rt_core::requests::actions::ActionSnapshot, Stop> {
        match crate::rt_core::requests::actions::ActionSnapshot::capture(
            &self.sup,
            self.next_child_stamp(),
            deadline,
        )
        .await
        {
            Ok(s) => Ok(s),
            Err(e) => {
                if !self.alive().await {
                    Err(Stop::Cancelled)
                } else {
                    Err(arm_fail(e))
                }
            }
        }
    }

    pub async fn alive(&self) -> bool {
        if self.is_executor() {
            self.sup.hub().latest().is_some_and(|s| {
                let m = s.rt.owner.macro_state;
                m.active_id == self.id && m.alive
            })
        } else {
            self.sup.inner().read().await.macro_alive(self.id)
        }
    }

    /// Map a child motion-install error under the owner write lock: admission-stamp refusal
    /// (`STOPPED`, `LEASE_LOST`, `ESTOP_ACTIVE`, `EPOCH_MISMATCH`) cancels the parent macro
    /// (preserving `cancelled` state and exact reason code) rather than failing it.
    pub fn map_install_err(&self, inner: &mut SupervisorInner, e: ArmError) -> Stop {
        use crate::safety::SafetyError;
        if let ArmError::Rejected { ref code, .. } = e {
            if code == "PARENT_REPLACED" {
                return Stop::Cancelled;
            }
            if code == "DEADLINE_EXPIRED" {
                return fail(self.timeout_code, self.timeout_message.clone());
            }
        }
        if matches!(
            e,
            ArmError::Safety(
                SafetyError::Stopped
                    | SafetyError::LeaseLost
                    | SafetyError::EStopActive
                    | SafetyError::EpochMismatch { .. }
            )
        ) {
            let api = ApiError::from(e);
            let code = api.code.unwrap_or_else(|| api.error.clone());
            inner.finish_macro(self.id, "cancelled", Some(&code), Some(api.message), None, None);
            return Stop::Cancelled;
        }
        arm_fail(e)
    }

    /// Cancelled when the action ended, a timeout failure past the deadline.
    pub async fn check(&self) -> Result<(), Stop> {
        if !self.alive().await { return Err(Stop::Cancelled); }
        if Instant::now() > self.deadline { return Err(fail(self.timeout_code, self.timeout_message.clone())); }
        Ok(())
    }

    /// Report `phase` in `action.phase` (after a [`check`](Self::check)).
    pub async fn phase(&self, phase: &str, progress: f32) -> Result<(), Stop> {
        self.check().await?;
        if !self.is_executor() {
            self.sup.inner().write().await.set_action_phase(self.id, phase, progress, None, None);
        }
        Ok(())
    }

    /// FK of the commanded J1..J6 in `end_link` frame (0 lock on `OWNER_EXECUTOR`).
    pub async fn pose(&self) -> Pose {
        if self.is_executor() {
            if let (Some(snap), Some(env)) = (self.sup.hub().latest(), self.sup.hub().prep_env()) {
                let arm = &env.config.arm_model;
                return arm
                    .fk_f32(&snap.rt.commanded_positions)
                    .unwrap_or_else(|| arm.fk(&[0.0; crate::chain::MAX_CHAIN_JOINTS]));
            }
            return Pose::new([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], [0.0; 3]);
        }
        let inner = self.sup.inner().read().await;
        let arm = &inner.config.arm_model;
        arm.fk_f32(&inner.commanded_positions).unwrap_or_else(|| arm.fk(&[0.0; crate::chain::MAX_CHAIN_JOINTS]))
    }

    /// FK of the commanded J1..J6 in the active TCP frame (0 lock on `OWNER_EXECUTOR`).
    pub async fn tcp_pose(&self) -> Pose {
        if self.is_executor() {
            if let (Some(snap), Some(env)) = (self.sup.hub().latest(), self.sup.hub().prep_env()) {
                return env
                    .config
                    .tcp_pose_f32(&snap.rt.commanded_positions)
                    .unwrap_or_else(|| {
                        env.config
                            .end_to_tcp(&env.config.arm_model.fk(&[0.0; crate::chain::MAX_CHAIN_JOINTS]))
                    });
            }
            return Pose::new([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], [0.0; 3]);
        }
        let inner = self.sup.inner().read().await;
        inner.config.tcp_pose_f32(&inner.commanded_positions).unwrap_or_else(|| {
            inner.config.end_to_tcp(&inner.config.arm_model.fk(&[0.0; crate::chain::MAX_CHAIN_JOINTS]))
        })
    }

    /// Measured joint positions, measured joint velocities, and arm joint indices
    /// (0 lock on `OWNER_EXECUTOR`, short read lock on `OWNER_LEGACY`).
    pub async fn measured_with_vel(&self) -> (Vec<f32>, Vec<f32>, Vec<usize>) {
        if self.is_executor() {
            if let (Some(snap), Some(env)) = (self.sup.hub().latest(), self.sup.hub().prep_env()) {
                return (
                    snap.rt.positions.clone(),
                    snap.rt.velocities.clone(),
                    env.config.arm_joint_indices(),
                );
            }
            return (Vec::new(), Vec::new(), Vec::new());
        }
        let inner = self.sup.inner().read().await;
        (
            inner.measured_positions.clone(),
            inner.measured_velocities.clone(),
            inner.config.arm_joint_indices(),
        )
    }

    /// Measured positions, arm-joint feedback instants, arm-joint feedback sequence counters,
    /// and arm-joint receive-queue bounds from one coherent sample (0 lock on `OWNER_EXECUTOR`).
    pub async fn arm_receive_progress(
        &self,
    ) -> (Vec<f32>, Vec<Option<Instant>>, Vec<u64>, Vec<Option<Instant>>) {
        if self.is_executor() {
            if let (Some(snap), Some(env)) = (self.sup.hub().latest(), self.sup.hub().prep_env()) {
                let arm = env.config.arm_joint_indices();
                let instant = |i: usize| snap.rt.feedback_instants.get(i).copied().flatten();
                let instants = arm.iter().map(|&i| instant(i)).collect();
                let seqs = arm
                    .iter()
                    .map(|&i| snap.rt.feedback_seqs.get(i).copied().unwrap_or(0))
                    .collect();
                let bounds = arm
                    .iter()
                    .map(|&i| match snap.rt.feedback_queue_bounds.get(i) {
                        Some(b) => *b,
                        None => instant(i),
                    })
                    .collect();
                return (snap.rt.positions.clone(), instants, seqs, bounds);
            }
            return (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        }
        let inner = self.sup.inner().read().await;
        let arm = inner.config.arm_joint_indices();
        let raw_instants = inner.core.feedback().instants();
        let raw_seqs = inner.core.feedback().seqs();
        let instant = |i: usize| raw_instants.get(i).copied().flatten();
        let instants = arm.iter().map(|&i| instant(i)).collect();
        let seqs = arm
            .iter()
            .map(|&i| raw_seqs.get(i).copied().unwrap_or(0))
            .collect();
        let bounds = arm
            .iter()
            .map(|&i| match inner.joint_feedback_queue_bounds.get(i) {
                Some(b) => *b,
                None => instant(i),
            })
            .collect();
        (inner.measured_positions.clone(), instants, seqs, bounds)
    }

    /// Read the [`ContactWatch`] recorded for `ticket` (0 lock on `OWNER_EXECUTOR`).
    pub async fn child_watch(&self, ticket: ChildTicket) -> ContactWatch {
        if self.is_executor() {
            return self
                .sup
                .hub()
                .latest()
                .map(|s| s.rt.owner.macro_state)
                .filter(|m| m.active_id == ticket.parent.0 && m.child_seq == ticket.child_seq)
                .map(|m| m.child_watch)
                .unwrap_or_default();
        }
        self.sup
            .inner()
            .read()
            .await
            .macro_child_watch(ticket)
            .unwrap_or_default()
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

    /// Wait for child `ticket`:
    /// - On `OWNER_EXECUTOR`: polls published `StateHub` snapshots with zero `SupervisorLock`.
    /// - On `OWNER_LEGACY`: polls under read lock and expires overdue children only after
    ///   [`OVERDUE_GRACE`].
    pub async fn wait_child(&self, ticket: ChildTicket) -> Result<SubDone, Stop> {
        if self.is_executor() {
            loop {
                tokio::time::sleep(POLL).await;
                let Some(snap) = self.sup.hub().latest() else {
                    if Instant::now() > self.deadline {
                        return Err(fail(self.timeout_code, self.timeout_message.clone()));
                    }
                    continue;
                };
                let m = snap.rt.owner.macro_state;
                if m.active_id != ticket.parent.0 || !m.alive {
                    return Err(Stop::Cancelled);
                }
                if m.child_seq < ticket.child_seq {
                    if Instant::now() > self.deadline {
                        return Err(fail(self.timeout_code, self.timeout_message.clone()));
                    }
                    continue;
                }
                if m.child_seq > ticket.child_seq {
                    return Err(Stop::Cancelled);
                }
                match m.child_sub {
                    SubState::Done(d) => return Ok(d),
                    SubState::Failed { code, message } => {
                        return Err(Stop::Fail(
                            code.as_str().to_owned(),
                            message.as_str().to_owned(),
                        ));
                    }
                    SubState::Running | SubState::Idle => {}
                }
            }
        }
        loop {
            tokio::time::sleep(POLL).await;
            let overdue = {
                let inner = self.sup.inner().read().await;
                match inner.macro_child_state(ticket) {
                    None => return Err(Stop::Cancelled),
                    Some(SubState::Done(d)) => return Ok(d),
                    Some(SubState::Failed { code, message }) => return Err(Stop::Fail(code.as_str().to_owned(), message.as_str().to_owned())),
                    Some(_) => inner.macro_child_overdue(ticket, Instant::now(), OVERDUE_GRACE),
                }
            };
            if overdue { self.sup.inner().write().await.expire_macro_child(Instant::now()); }
        }
    }

    /// Submits an A7 `FinishMacro` command through the RT executor's shared bounded FIFO
    /// reply router (`ActionReplyRouter`) without touching `SupervisorLock`, without head-of-line
    /// blocking behind unrelated replies, and without stealing external traffic.
    async fn submit_action_executor(
        &self,
        stamp: &crate::rt_core::requests::actions::ActionStamp,
        deadline: Instant,
        make_submit: impl Fn() -> crate::rt_core::requests::actions::ActionSubmit<'static, crate::rt_core::ops::PlanSlot>,
        ignore_parent_dead: bool,
    ) -> Result<ActionId, Stop> {
        use crate::rt_core::broker::{AdmitError, ClientId, Reply};
        use tokio::sync::oneshot::error::TryRecvError;
        let Some(handle) = self.sup.executor_handle() else {
            return Err(Stop::Cancelled);
        };
        let client = ClientId(stamp.admission.connection_id);
        let auth = stamp.broker_authority();
        let (req_id, mut rx) = loop {
            let sub = make_submit();
            match handle.submit_internal_action(client, 0, auth, deadline, sub) {
                Ok(pair) => break pair,
                Err(AdmitError::Disconnected) => return Err(Stop::Cancelled),
                Err(AdmitError::PlansFull | AdmitError::CommandsFull | AdmitError::RepliesFull) => {
                    handle.pump_broker_replies();
                    if !ignore_parent_dead && !self.alive().await {
                        return Err(Stop::Cancelled);
                    }
                    if Instant::now() > deadline {
                        return Err(fail(self.timeout_code, self.timeout_message.clone()));
                    }
                    tokio::time::sleep(Duration::from_millis(2)).await;
                }
            }
        };
        let wait_limit = deadline + Duration::from_secs(2);
        loop {
            handle.pump_broker_replies();
            match rx.try_recv() {
                Ok(reply) => {
                    return match reply {
                        Reply::Ack { action, .. } => Ok(action.unwrap_or(ActionId(self.id))),
                        Reply::Refused { code, .. } => match code {
                            "STOPPED" | "LEASE_LOST" | "ESTOP_ACTIVE" | "EPOCH_MISMATCH"
                            | "PARENT_REPLACED" => Err(Stop::Cancelled),
                            "DEADLINE_EXPIRED" => {
                                Err(fail(self.timeout_code, self.timeout_message.clone()))
                            }
                            other => Err(fail(other, format!("RT action refused: {other}"))),
                        },
                        Reply::Terminal { .. } => Ok(ActionId(self.id)),
                        Reply::Lease { .. } => Err(fail("INTERNAL", "unexpected Lease reply for RT action")),
                    };
                }
                Err(TryRecvError::Closed) => return Err(Stop::Cancelled),
                Err(TryRecvError::Empty) => {}
            }
            if !ignore_parent_dead && !self.alive().await {
                handle.cancel_internal_waiter(req_id);
                return Err(Stop::Cancelled);
            }
            if Instant::now() > wait_limit {
                handle.cancel_internal_waiter(req_id);
                if let Ok(reply) = rx.try_recv() {
                    return match reply {
                        Reply::Ack { action, .. } => Ok(action.unwrap_or(ActionId(self.id))),
                        Reply::Refused { code, .. } => match code {
                            "STOPPED" | "LEASE_LOST" | "ESTOP_ACTIVE" | "EPOCH_MISMATCH"
                            | "PARENT_REPLACED" => Err(Stop::Cancelled),
                            "DEADLINE_EXPIRED" => {
                                Err(fail(self.timeout_code, self.timeout_message.clone()))
                            }
                            other => Err(fail(other, format!("RT action refused: {other}"))),
                        },
                        Reply::Terminal { .. } => Ok(ActionId(self.id)),
                        Reply::Lease { .. } => Err(fail("INTERNAL", "unexpected Lease reply for RT action")),
                    };
                }
                return Err(fail(self.timeout_code, self.timeout_message.clone()));
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
    }

    /// Finishes the parent macro through the single owner-side [`install_finish_macro`] path
    /// (Legacy write lock or RT Executor `ActionSubmit::FinishMacro`, 0 lock on Executor).
    pub async fn finish(
        &self,
        state: crate::rt_core::requests::actions::FinishState,
        code: Option<&str>,
        message: Option<String>,
        holding: Option<bool>,
        gripper_gap_rad: Option<f32>,
        held: crate::rt_core::requests::actions::HeldUpdate,
    ) {
        let _ = self
            .finish_fenced(state, code, message, holding, gripper_gap_rad, held, None)
            .await;
    }

    /// Revision-fenced variant of [`Self::finish`]: when `revisions` is `Some(rev)` and
    /// `state == FinishState::Done`, binds ALL five captured revisions atomically on the owner
    /// commit before updating `held` (both Legacy and RT Executor). Returns `Err(code)` if a
    /// revision changed before commit so the caller can immediately terminate the parent cleanly
    /// with `FinishState::Failed` and `HeldUpdate::Clear`.
    #[allow(clippy::too_many_arguments)]
    pub async fn finish_fenced(
        &self,
        state: crate::rt_core::requests::actions::FinishState,
        code: Option<&str>,
        message: Option<String>,
        holding: Option<bool>,
        gripper_gap_rad: Option<f32>,
        held: crate::rt_core::requests::actions::HeldUpdate,
        revisions: Option<crate::grasp_select::AdaptiveRevisions>,
    ) -> Result<(), &'static str> {
        use crate::rt_core::requests::actions::{
            install_finish_macro, ActionStamp, FinishMacroSpec, FinishState, OwnerCheckCtx,
            DEFAULT_FINISH_DEADLINE,
        };
        let now = Instant::now();
        let finish_deadline = now + DEFAULT_FINISH_DEADLINE;
        let fence = revisions.filter(|_| state == FinishState::Done);
        if self.is_executor() {
            let Some(handle) = self.sup.executor_handle() else {
                return Ok(());
            };
            let boot = handle.boot();
            let arm = self
                .sup
                .hub()
                .latest()
                .map_or(crate::rt_core::ids::ArmId::LOCAL, |s| s.rt.owner.arm);
            let mut spec = match state {
                FinishState::Done => FinishMacroSpec::done(ActionId(self.id), boot),
                FinishState::Failed => FinishMacroSpec::failed(
                    ActionId(self.id),
                    boot,
                    code.unwrap_or("FAILED"),
                    message.as_deref().unwrap_or(""),
                ),
                FinishState::Cancelled => {
                    FinishMacroSpec::cancelled(ActionId(self.id), boot, code, message.as_deref())
                }
            };
            spec.holding = holding;
            spec.gripper_gap_rad = gripper_gap_rad;
            spec.held = held;
            let mut stamp = ActionStamp {
                arm,
                boot,
                admission: self.stamp,
                config_revision: 0,
                calibration_revision: None,
                poses_revision: None,
                sequences_revision: None,
                grasp_setup_revision: None,
                gripper_calib_revision: None,
                power_budget_revision: None,
                parent_generation: self.id,
                deadline: finish_deadline,
            };
            if let Some(rev) = fence {
                spec.check_workspace_revision = true;
                spec.calibration_revision = Some(rev.calibration_revision);
                spec.grasp_setup_revision = Some(rev.grasp_setup_revision);
                spec.gripper_calib_revision = Some(rev.gripper_calib_revision);
                spec.power_budget_revision = Some(rev.power_budget_revision);
                stamp.config_revision = rev.workspace_revision;
                stamp.calibration_revision = Some(rev.calibration_revision);
                stamp.grasp_setup_revision = Some(rev.grasp_setup_revision);
                stamp.gripper_calib_revision = Some(rev.gripper_calib_revision);
                stamp.power_budget_revision = Some(rev.power_budget_revision);
            }
            match self
                .submit_action_executor(
                    &stamp,
                    finish_deadline,
                    || crate::rt_core::requests::actions::ActionSubmit::FinishMacro(spec),
                    true,
                )
                .await
            {
                Ok(_) => return Ok(()),
                Err(Stop::Cancelled) => return Ok(()),
                Err(Stop::Fail(c, _)) if fence.is_some() => {
                    let code: &'static str = crate::rt_core::ops::TERMINAL_CODES
                        .iter()
                        .copied()
                        .find(|&k| k == c.as_str())
                        .unwrap_or(match c.as_str() {
                            "CONFIG_BUSY" => "CONFIG_BUSY",
                            "FEEDBACK_STALE" => "FEEDBACK_STALE",
                            "MOTION_BUSY" => "MOTION_BUSY",
                            "DEADLINE_EXPIRED" => "DEADLINE_EXPIRED",
                            "CALIBRATION_CHANGED" => "CALIBRATION_CHANGED",
                            _ => "PLAN_STALE",
                        });
                    return Err(code);
                }
                Err(_) => return Ok(()),
            }
        }
        let mut inner = self.sup.inner().write().await;
        inner.set_command("macro:finish");
        inner.set_action_id(self.id);
        if !inner.macro_alive(self.id) {
            return Ok(());
        }
        let boot = inner.boot_id;
        let mut spec = match state {
            FinishState::Done => FinishMacroSpec::done(ActionId(self.id), boot),
            FinishState::Failed => FinishMacroSpec::failed(
                ActionId(self.id),
                boot,
                code.unwrap_or("FAILED"),
                message.as_deref().unwrap_or(""),
            ),
            FinishState::Cancelled => {
                FinishMacroSpec::cancelled(ActionId(self.id), boot, code, message.as_deref())
            }
        };
        spec.holding = holding;
        spec.gripper_gap_rad = gripper_gap_rad;
        spec.held = held;
        let ctx = OwnerCheckCtx::for_legacy(&inner);
        let mut stamp = ActionStamp {
            arm: inner.core.id(),
            boot,
            admission: self.stamp,
            config_revision: inner.workspace_revision,
            calibration_revision: None,
            poses_revision: None,
            sequences_revision: None,
            grasp_setup_revision: None,
            gripper_calib_revision: None,
            power_budget_revision: None,
            parent_generation: self.id,
            deadline: finish_deadline,
        };
        if let Some(rev) = fence {
            spec.check_workspace_revision = true;
            spec.calibration_revision = Some(rev.calibration_revision);
            spec.grasp_setup_revision = Some(rev.grasp_setup_revision);
            spec.gripper_calib_revision = Some(rev.gripper_calib_revision);
            spec.power_budget_revision = Some(rev.power_budget_revision);
            stamp.config_revision = rev.workspace_revision;
            stamp.calibration_revision = Some(rev.calibration_revision);
            stamp.grasp_setup_revision = Some(rev.grasp_setup_revision);
            stamp.gripper_calib_revision = Some(rev.gripper_calib_revision);
            stamp.power_budget_revision = Some(rev.power_budget_revision);
        }
        match install_finish_macro(&mut inner, &stamp, &ctx, &spec, message) {
            Ok(_) => Ok(()),
            Err(r) if matches!(r.code, "PARENT_REPLACED" | "STOPPED" | "LEASE_LOST" | "ESTOP_ACTIVE" | "EPOCH_MISMATCH") => Ok(()),
            Err(r) if fence.is_some() => Err(r.code),
            Err(_) => Ok(()),
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
        self.move_arm_staged_in_with_table_guarded(
            frame,
            phase,
            spec,
            speed,
            approach,
            table_clearance,
            None,
        )
        .await
        .map(|(done, _)| done)
    }

    /// [`move_arm_staged_in_with_table`](Self::move_arm_staged_in_with_table) with an optional
    /// [`ContactGuard`] folded atomically into `ChildSpec.contact` at `InstallChild` time
    /// (zero pre-arm leak on refusal). Returns `(SubDone, ContactWatch)`.
    #[allow(clippy::too_many_arguments)]
    pub async fn move_arm_staged_in_with_table_guarded(
        &self,
        frame: GoalFrame,
        phase: &str,
        spec: GoalSpec,
        speed: Speed,
        approach: Option<FinalApproach>,
        table_clearance: Option<(Vec<crate::kinematics::Vec3>, f64)>,
        contact: Option<ContactGuard>,
    ) -> Result<(SubDone, ContactWatch), (Stop, bool)> {
        use crate::rt_core::requests::actions as req;
        let before = |s: Stop| (s, false);
        if !self.alive().await {
            return Err(before(Stop::Cancelled));
        }
        let snap = self.snapshot(self.deadline).await.map_err(before)?;
        let input = snap.plan_input_in(frame).map_err(arm_fail).map_err(before)?;
        let plan = approach.as_ref().map(|a| snap.env.approach_plan(a.use_, a.request));
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
        let hard_limits = snap.env.config.limits.clone();
        let commissioning_required = snap.env.config.active_tool.commissioning_required;
        let torque_or_power_hold = snap.owner.torque_or_power_hold;
        let (input, resolved) = tokio::task::spawn_blocking(move || {
            let r: Result<req::ResolvedCartesianLegs, ArmError> =
                motion::plan_cartesian_opts(&input, goal, speed, opts)
                    .map_err(ArmError::from)
                    .and_then(|direct| {
                        let pre = match &plan {
                            Some(Ok(plan)) => crate::backlash::plan_cartesian_pre(
                                &input,
                                &direct,
                                plan,
                                rules_for_pre.as_ref(),
                                speed,
                                opts,
                            ),
                            Some(Err(Some(r))) => crate::backlash::Pre::Direct((*r).to_string()),
                            _ => crate::backlash::Pre::Off,
                        };
                        let plan_ok = plan.and_then(Result::ok);
                        req::resolve_cartesian_legs_with_rules(
                            &input,
                            &hard_limits,
                            direct,
                            pre,
                            plan_ok.as_ref(),
                            rules_for_pre.as_ref(),
                            commissioning_required,
                            torque_or_power_hold,
                        )
                    });
            (input, r)
        }).await
            .map_err(|e| before(fail("INTERNAL", format!("planner task failed: {e}"))))?;
        let resolved = match resolved {
            Ok(p) => p,
            Err(e) => {
                if !self.is_executor() {
                    if let Some(detail) = e.workspace_detail() {
                        self.sup.inner().write().await.set_action_workspace_violation(self.id, detail);
                    }
                }
                return Err(before(arm_fail(e)));
            }
        };
        let mut child_stamp = snap.stamp.for_child(ActionId(self.id), self.deadline);
        child_stamp.poses_revision = None;
        child_stamp.sequences_revision = None;
        child_stamp.grasp_setup_revision = None;
        child_stamp.gripper_calib_revision = None;
        child_stamp.calibration_revision = None;
        child_stamp.power_budget_revision = Some(snap.env.power_budget_revision);

        let ticket = if self.is_executor() {
            let m = snap.owner.macro_state;
            let next_seq = (m.active_id == self.id && m.alive)
                .then(|| m.child_seq.checked_add(1))
                .flatten()
                .ok_or_else(|| before(Stop::Cancelled))?;
            let approach_then = resolved.approach_leg.as_ref().map(|l| l.staged);
            let leg2_s = approach_then.map_or(0.0, |l| l.duration_s);
            let timeout_s =
                ((resolved.path.duration + leg2_s + 0.5) * 2.0 + 5.0).min(self.phase_timeout_s);
            let child_spec = ChildSpec {
                ticket: ChildTicket {
                    parent: ActionId(self.id),
                    child_seq: next_seq,
                },
                timeout_s,
                timeout_code: self.timeout_code,
                contact,
            };
            let slot = req::prepare_child_cartesian_slot_with_approach(
                &snap.commanded_positions,
                &resolved.path,
                input.power.as_ref().map(|_| resolved.path.power_scale),
                approach_then,
                child_spec,
            )
            .map_err(|c| before(fail(c, format!("failed to stage Cartesian child: {c}"))))?;
            self.submit_child_slot_executor(&child_stamp, slot, true, None)
                .await
                .map_err(before)?;
            child_spec.ticket
        } else {
            let mut inner = self.sup.inner().write().await;
            inner.set_command("macro_child:cartesian");
            inner.set_action_id(self.id);
            let Some(next_seq) = inner
                .macro_run
                .as_ref()
                .filter(|m| m.id == self.id && inner.macro_alive(self.id))
                .and_then(|m| m.next_child_seq())
            else {
                return Err(before(Stop::Cancelled));
            };
            let child_spec = ChildSpec {
                ticket: ChildTicket {
                    parent: ActionId(self.id),
                    child_seq: next_seq,
                },
                timeout_s: self.phase_timeout_s,
                timeout_code: self.timeout_code,
                contact,
            };
            let owner_ctx = req::OwnerCheckCtx::for_legacy(&inner);
            let installed = match req::install_macro_child(
                &mut inner,
                &child_stamp,
                &owner_ctx,
                true,
                req::MacroChildPayload::LegacyCartesian {
                    spec: child_spec,
                    start: snap.commanded_positions.clone(),
                    input: input.clone(),
                    resolved,
                    lock_j7_hold: false,
                },
            ) {
                Ok(v) => v,
                Err(e) => {
                    if let Some(detail) = e.workspace_detail() {
                        inner.set_action_workspace_violation(self.id, detail);
                    }
                    return Err(before(self.map_install_err(&mut inner, e)));
                }
            };
            installed.ticket
        };
        let t0 = Instant::now();
        let sub_res = self.wait_child(ticket).await;
        let ms = t0.elapsed().as_millis() as u32;
        let watch = self.child_watch(ticket).await;
        if !self.is_executor() {
            let mut inner = self.sup.inner().write().await;
            let meas_opt = inner.config.arm_model.fk_f32(&inner.measured_positions).map(|p| motion::end_to_frame(&p, &input));
            let cmd_opt = inner.config.arm_model.fk_f32(&inner.commanded_positions).map(|p| motion::end_to_frame(&p, &input));
            if let (Some(meas), Some(cmd)) = (meas_opt, cmd_opt) {
                let track_mm = (crate::kinematics::norm(&crate::kinematics::sub(&meas.p, &cmd.p)) * 1000.0) as f32;
                let track_deg = crate::kinematics::angle_between(&meas.r, &cmd.r).to_degrees() as f32;
                let goal_mm = (crate::kinematics::norm(&crate::kinematics::sub(&meas.p, &goal.p)) * 1000.0) as f32;
                let goal_deg = crate::kinematics::angle_between(&meas.r, &goal.r).to_degrees() as f32;
                let contact_flag = inner.last_action.as_ref().filter(|a| a.id == self.id).and_then(|a| a.descend_contact);
                if let Some(a) = inner.last_action.as_mut().filter(|a| a.id == self.id) {
                    if a.motions.len() < 32 {
                        a.motions.push(motion::MacroMotionDiag {
                            phase: phase.to_string(),
                            ms,
                            track_mm,
                            track_deg,
                            goal_mm,
                            goal_deg,
                            contact: contact_flag,
                        });
                    }
                }
            }
        }
        sub_res.map(|done| (done, watch)).map_err(|s| (s, true))
    }

    /// Install an already-checked adaptive arm [`motion::JointPath`] (Cartesian TCP line or
    /// swept-joint fallback) verbatim with `lock_j7_hold: true` and ALL FIVE captured revisions
    /// fenced on both Legacy and RT Executor owners (0 `SupervisorLock` on Executor).
    #[allow(clippy::too_many_arguments)]
    pub async fn install_adaptive_path_child(
        &self,
        phase: &str,
        snap: &crate::rt_core::requests::actions::ActionSnapshot,
        input: &motion::PlanInput,
        goal: crate::kinematics::Pose,
        path: motion::JointPath,
        revisions: crate::grasp_select::AdaptiveRevisions,
        contact: Option<ContactGuard>,
    ) -> Result<(SubDone, ContactWatch), (Stop, bool)> {
        use crate::rt_core::requests::actions as req;
        let before = |s: Stop| (s, false);
        if !self.alive().await {
            return Err(before(Stop::Cancelled));
        }
        let mut child_stamp = snap.stamp.for_child(ActionId(self.id), self.deadline);
        child_stamp.config_revision = revisions.workspace_revision;
        child_stamp.calibration_revision = Some(revisions.calibration_revision);
        child_stamp.poses_revision = None;
        child_stamp.sequences_revision = None;
        child_stamp.grasp_setup_revision = Some(revisions.grasp_setup_revision);
        child_stamp.gripper_calib_revision = Some(revisions.gripper_calib_revision);
        child_stamp.power_budget_revision = Some(revisions.power_budget_revision);

        let ticket = if self.is_executor() {
            let m = snap.owner.macro_state;
            let next_seq = (m.active_id == self.id && m.alive)
                .then(|| m.child_seq.checked_add(1))
                .flatten()
                .ok_or_else(|| before(Stop::Cancelled))?;
            let timeout_s = ((path.duration + 0.5) * 2.0 + 5.0).min(self.phase_timeout_s);
            let child_spec = ChildSpec {
                ticket: ChildTicket {
                    parent: ActionId(self.id),
                    child_seq: next_seq,
                },
                timeout_s,
                timeout_code: self.timeout_code,
                contact,
            };
            let slot = req::prepare_child_cartesian_slot_with_options(
                &snap.commanded_positions,
                &path,
                input.power.as_ref().map(|_| path.power_scale),
                None,
                true,
                child_spec,
            )
            .map_err(|c| before(fail(c, format!("[{phase}] failed to stage adaptive path child: {c}"))))?;
            self.submit_child_slot_executor(
                &child_stamp,
                slot,
                true,
                Some(revisions.calibration_revision),
            )
            .await
            .map_err(|s| before(tag_phase_stop(phase, s)))?;
            child_spec.ticket
        } else {
            let mut inner = self.sup.inner().write().await;
            let Some(next_seq) = inner
                .macro_run
                .as_ref()
                .filter(|m| m.id == self.id && inner.macro_alive(self.id))
                .and_then(|m| m.next_child_seq())
            else {
                return Err(before(Stop::Cancelled));
            };
            let child_spec = ChildSpec {
                ticket: ChildTicket {
                    parent: ActionId(self.id),
                    child_seq: next_seq,
                },
                timeout_s: self.phase_timeout_s,
                timeout_code: self.timeout_code,
                contact,
            };
            let expected_duration_s = path.duration;
            let owner_ctx = req::OwnerCheckCtx::for_legacy(&inner);
            let installed = match req::install_macro_child(
                &mut inner,
                &child_stamp,
                &owner_ctx,
                true,
                req::MacroChildPayload::LegacyCartesian {
                    spec: child_spec,
                    start: snap.commanded_positions.clone(),
                    input: input.clone(),
                    resolved: req::ResolvedCartesianLegs {
                        path,
                        approach_leg: None,
                        outcome: None,
                        expected_duration_s,
                    },
                    lock_j7_hold: true,
                },
            ) {
                Ok(v) => v,
                Err(e) => {
                    if let Some(detail) = e.workspace_detail() {
                        inner.set_action_workspace_violation(self.id, detail);
                    }
                    return Err(before(tag_phase_stop(phase, self.map_install_err(&mut inner, e))));
                }
            };
            installed.ticket
        };
        let t0 = Instant::now();
        let sub_res = self.wait_child(ticket).await;
        let ms = t0.elapsed().as_millis() as u32;
        let watch = self.child_watch(ticket).await;
        if !self.is_executor() {
            let mut inner = self.sup.inner().write().await;
            let meas_opt = inner.config.arm_model.fk_f32(&inner.measured_positions).map(|p| motion::end_to_frame(&p, input));
            let cmd_opt = inner.config.arm_model.fk_f32(&inner.commanded_positions).map(|p| motion::end_to_frame(&p, input));
            if let (Some(meas), Some(cmd)) = (meas_opt, cmd_opt) {
                let track_mm = (crate::kinematics::norm(&crate::kinematics::sub(&meas.p, &cmd.p)) * 1000.0) as f32;
                let track_deg = crate::kinematics::angle_between(&meas.r, &cmd.r).to_degrees() as f32;
                let goal_mm = (crate::kinematics::norm(&crate::kinematics::sub(&meas.p, &goal.p)) * 1000.0) as f32;
                let goal_deg = crate::kinematics::angle_between(&meas.r, &goal.r).to_degrees() as f32;
                let contact_flag = inner.last_action.as_ref().filter(|a| a.id == self.id).and_then(|a| a.descend_contact);
                if let Some(a) = inner.last_action.as_mut().filter(|a| a.id == self.id) {
                    if a.motions.len() < 32 {
                        a.motions.push(motion::MacroMotionDiag {
                            phase: phase.to_string(),
                            ms,
                            track_mm,
                            track_deg,
                            goal_mm,
                            goal_deg,
                            contact: contact_flag,
                        });
                    }
                }
            }
        }
        sub_res
            .map(|done| (done, watch))
            .map_err(|s| (tag_phase_stop(phase, s), true))
    }

    /// Install a joint-space quintic macro child through the single `install_macro_child` /
    /// `InstallChild` path (shared by `sequences` pose steps and `measure_multiview` views) and
    /// wait for completion. `Err((stop, false))` = refused before motion; `Err((stop, true))` =
    /// failed after installation.
    pub async fn install_joint_child(
        &self,
        phase: &str,
        target: Vec<f32>,
        limits: Vec<crate::model::JointLimit>,
        check_workspace_rev: bool,
        workspace_rev_override: Option<u64>,
        calibration_rev: Option<u64>,
    ) -> Result<SubDone, (Stop, bool)> {
        use crate::rt_core::requests::actions as req;
        let before = |s: Stop| (s, false);
        if !self.alive().await {
            return Err(before(Stop::Cancelled));
        }
        let snap = self.snapshot(self.deadline).await.map_err(before)?;
        let mut child_stamp = snap.stamp.for_child(ActionId(self.id), self.deadline);
        if let Some(ws) = workspace_rev_override {
            child_stamp.config_revision = ws;
        }
        child_stamp.calibration_revision = calibration_rev;
        child_stamp.poses_revision = None;
        child_stamp.sequences_revision = None;
        child_stamp.grasp_setup_revision = None;
        child_stamp.gripper_calib_revision = None;
        child_stamp.power_budget_revision = Some(snap.env.power_budget_revision);

        let ticket = if self.is_executor() {
            let m = snap.owner.macro_state;
            let next_seq = (m.active_id == self.id && m.alive)
                .then(|| m.child_seq.checked_add(1))
                .flatten()
                .ok_or_else(|| before(Stop::Cancelled))?;
            let zeros = vec![0.0f32; snap.env.config.dof];
            let traj = crate::trajectory::TrajectoryPlanner::plan(
                &snap.commanded_positions,
                &snap.commanded_velocities,
                &target,
                &zeros,
                None,
                &limits,
            )
            .map_err(|e| before(arm_fail(e.into())))?;
            let (traj, power_scale) = req::fit_quintic_power_off_lock(
                traj,
                &snap.commanded_positions,
                &snap.commanded_velocities,
                &target,
                &limits,
                snap.env.power_budget.as_ref(),
                &snap.env.config.arm_model,
            )
            .map_err(|e| before(arm_fail(e)))?;
            let max_vel: Vec<f32> = limits.iter().map(|l| l.max_velocity).collect();
            let timeout_s = ((traj.duration + motion::SETTLE_MAX_S) * 2.0 + 5.0).min(self.phase_timeout_s);
            let child_spec = ChildSpec {
                ticket: ChildTicket {
                    parent: ActionId(self.id),
                    child_seq: next_seq,
                },
                timeout_s,
                timeout_code: self.timeout_code,
                contact: None,
            };
            let slot = req::prepare_child_joint_slot(
                &snap.commanded_positions,
                &target,
                &max_vel,
                traj.duration,
                power_scale,
                child_spec,
            )
            .map_err(|c| before(fail(c, format!("failed to stage joint child for {phase}: {c}"))))?;
            self.submit_child_slot_executor(
                &child_stamp,
                slot,
                check_workspace_rev,
                calibration_rev,
            )
            .await
            .map_err(before)?;
            child_spec.ticket
        } else {
            let rev_ok = (!check_workspace_rev
                || snap.env.workspace_revision == child_stamp.config_revision)
                && calibration_rev.is_none_or(|c| c == snap.env.calibration_revision);
            let (snap, target, limits, prepared_res) = if rev_ok {
                tokio::task::spawn_blocking(move || {
                    let r = req::build_prepared_quintic_off_lock(&snap, &target, &limits);
                    (snap, target, limits, Some(r))
                })
                .await
                .map_err(|e| before(fail("INTERNAL", format!("planner task failed: {e}"))))?
            } else {
                (snap, target, limits, None)
            };
            let prepared = match prepared_res {
                Some(Ok(p)) => Some(p),
                Some(Err(e)) => {
                    let mut inner = self.sup.inner().write().await;
                    inner.set_command("macro_child:joint");
                    inner.set_action_id(self.id);
                    if let Some(detail) = e.workspace_detail() {
                        inner.set_action_workspace_violation(self.id, detail);
                    }
                    return Err(before(self.map_install_err(&mut inner, e)));
                }
                None => None,
            };
            let mut inner = self.sup.inner().write().await;
            inner.set_command("macro_child:joint");
            inner.set_action_id(self.id);
            let Some(next_seq) = inner
                .macro_run
                .as_ref()
                .filter(|m| m.id == self.id && inner.macro_alive(self.id))
                .and_then(|m| m.next_child_seq())
            else {
                return Err(before(Stop::Cancelled));
            };
            let child_spec = ChildSpec {
                ticket: ChildTicket {
                    parent: ActionId(self.id),
                    child_seq: next_seq,
                },
                timeout_s: self.phase_timeout_s,
                timeout_code: self.timeout_code,
                contact: None,
            };
            let owner_ctx = req::OwnerCheckCtx::for_legacy(&inner);
            match req::install_macro_child(
                &mut inner,
                &child_stamp,
                &owner_ctx,
                check_workspace_rev,
                req::MacroChildPayload::LegacyJoint {
                    spec: child_spec,
                    start: snap.commanded_positions,
                    target,
                    limits,
                    prepared,
                },
            ) {
                Ok(v) => v.ticket,
                Err(e) => {
                    if let Some(detail) = e.workspace_detail() {
                        inner.set_action_workspace_violation(self.id, detail);
                    }
                    return Err(before(self.map_install_err(&mut inner, e)));
                }
            }
        };
        self.wait_child(ticket).await.map_err(|s| (s, true))
    }

    /// Install a prepared replay macro child through the single `install_macro_child` /
    /// `InstallChild` path and wait for completion.
    pub async fn install_replay_child(
        &self,
        phase: &str,
        plan: crate::replay::ReplayPlan,
        workspace_rev_override: Option<u64>,
        calibration_rev: Option<u64>,
    ) -> Result<SubDone, Stop> {
        use crate::rt_core::requests::actions as req;
        if !self.alive().await {
            return Err(Stop::Cancelled);
        }
        let snap = self.snapshot(self.deadline).await?;
        let mut child_stamp = snap.stamp.for_child(ActionId(self.id), self.deadline);
        if let Some(ws) = workspace_rev_override {
            child_stamp.config_revision = ws;
        }
        child_stamp.calibration_revision = calibration_rev;
        child_stamp.poses_revision = None;
        child_stamp.sequences_revision = None;
        child_stamp.grasp_setup_revision = None;
        child_stamp.gripper_calib_revision = None;
        child_stamp.power_budget_revision = Some(snap.env.power_budget_revision);
        let tool_digest = snap.env.tool_digest_hash;

        let ticket = if self.is_executor() {
            let m = snap.owner.macro_state;
            let next_seq = (m.active_id == self.id && m.alive)
                .then(|| m.child_seq.checked_add(1))
                .flatten()
                .ok_or(Stop::Cancelled)?;
            let timeout_s = ((plan.duration + 1.0) * 2.0 + 5.0).min(self.phase_timeout_s);
            let child_spec = ChildSpec {
                ticket: ChildTicket {
                    parent: ActionId(self.id),
                    child_seq: next_seq,
                },
                timeout_s,
                timeout_code: self.timeout_code,
                contact: None,
            };
            // Always stage `plan`'s original planned start and pre-swept approach (never
            // `snap.commanded_positions` from a post-planning snapshot) so commit refuses
            // `PLAN_STALE` if the pose moved while planning.
            let slot = req::prepare_child_replay_slot(&plan, tool_digest, child_spec)
                .map_err(|c| fail(c, format!("failed to stage replay child for {phase}: {c}")))?;
            self.submit_child_slot_executor(&child_stamp, slot, true, calibration_rev)
                .await?;
            child_spec.ticket
        } else {
            let mut inner = self.sup.inner().write().await;
            inner.set_command("macro_child:replay");
            inner.set_action_id(self.id);
            let Some(next_seq) = inner
                .macro_run
                .as_ref()
                .filter(|m| m.id == self.id && inner.macro_alive(self.id))
                .and_then(|m| m.next_child_seq())
            else {
                return Err(Stop::Cancelled);
            };
            let child_spec = ChildSpec {
                ticket: ChildTicket {
                    parent: ActionId(self.id),
                    child_seq: next_seq,
                },
                timeout_s: self.phase_timeout_s,
                timeout_code: self.timeout_code,
                contact: None,
            };
            let owner_ctx = req::OwnerCheckCtx::for_legacy(&inner);
            let planned_start = plan.input.start.clone();
            match req::install_macro_child(
                &mut inner,
                &child_stamp,
                &owner_ctx,
                true,
                req::MacroChildPayload::LegacyReplay {
                    spec: child_spec,
                    start: planned_start,
                    plan,
                    tool_digest,
                },
            ) {
                Ok(v) => v.ticket,
                Err(e) => return Err(self.map_install_err(&mut inner, e)),
            }
        };
        self.wait_child(ticket).await
    }

    /// Reconcile a planned gripper sub-motion with the remaining macro budget before installation,
    /// arm its RT child deadline under the same owner install call, and wait on its `ChildTicket`:
    /// pos-force closes arm up to `(timeout_s + 10 s).min(self.deadline)` (not clamped to
    /// `phase_timeout_s`), while pos-vel and opening moves use `phase_timeout_s`.
    async fn run_gripper_planned(
        &self,
        phase: &str,
        command: GripperCommand,
        fraction: Option<f64>,
        grasp: bool,
    ) -> Result<SubDone, Stop> {
        use crate::rt_core::requests::actions as req;
        if !self.alive().await {
            return Err(Stop::Cancelled);
        }
        let now = Instant::now();
        if now >= self.deadline {
            return Err(fail(self.timeout_code, self.timeout_message.clone()));
        }
        let snap = self.snapshot(self.deadline).await?;
        let closes = match command {
            GripperCommand::Close | GripperCommand::Seek => true,
            GripperCommand::Open => false,
            GripperCommand::Move => match (
                snap.env.config.gripper_policy.as_ref(),
                snap.env.config.gripper_index(),
            ) {
                (Some(p), Some(g)) => {
                    let target =
                        p.closed_rad + fraction.unwrap_or(0.0) as f32 * (p.open_rad - p.closed_rad);
                    let dir = if p.closed_rad >= p.open_rad { 1.0 } else { -1.0 };
                    (target - snap.commanded_positions.get(g).copied().unwrap_or(target)) * dir
                        > 1e-3
                }
                _ => true,
            },
        };
        let close_speed = snap
            .env
            .config
            .gripper_policy
            .as_ref()
            .map_or(f64::from(crate::model::DEFAULT_CLOSE_SPEED_RAD_S), |p| p.close_speed());
        let (speed, timeout_s) = if closes { (close_speed, None) } else { (1.0, Some(8.0)) };
        let request = GripperRequest { command, fraction, grasp, speed, timeout_s, tag: None };
        let remaining_s = self.deadline.saturating_duration_since(Instant::now()).as_secs_f64();
        let mut child_stamp = snap.stamp.for_child(ActionId(self.id), self.deadline);
        child_stamp.poses_revision = None;
        child_stamp.sequences_revision = None;
        child_stamp.grasp_setup_revision = None;
        child_stamp.gripper_calib_revision = Some(snap.env.gripper_calib_revision);
        child_stamp.calibration_revision = None;
        child_stamp.power_budget_revision = None;

        let ticket = if self.is_executor() {
            let m = snap.owner.macro_state;
            let next_seq = (m.active_id == self.id && m.alive)
                .then(|| m.child_seq.checked_add(1))
                .flatten()
                .ok_or(Stop::Cancelled)?;
            let child_spec = ChildSpec {
                ticket: ChildTicket {
                    parent: ActionId(self.id),
                    child_seq: next_seq,
                },
                timeout_s: self.phase_timeout_s,
                timeout_code: self.timeout_code,
                contact: None,
            };
            let slot = req::prepare_child_gripper_slot(
                &snap.commanded_positions,
                command,
                fraction,
                grasp,
                speed,
                timeout_s,
                Some(remaining_s),
                true,
                child_spec,
            )
            .map_err(|c| fail(c, format!("failed to stage gripper child for {phase}: {c}")))?;
            self.submit_child_slot_executor(&child_stamp, slot, false, None)
                .await?;
            child_spec.ticket
        } else {
            let mut inner = self.sup.inner().write().await;
            inner.set_command("macro_child:gripper");
            inner.set_action_id(self.id);
            let Some(next_seq) = inner
                .macro_run
                .as_ref()
                .filter(|m| m.id == self.id && inner.macro_alive(self.id))
                .and_then(|m| m.next_child_seq())
            else {
                return Err(Stop::Cancelled);
            };
            let child_spec = ChildSpec {
                ticket: ChildTicket {
                    parent: ActionId(self.id),
                    child_seq: next_seq,
                },
                timeout_s: self.phase_timeout_s,
                timeout_code: self.timeout_code,
                contact: None,
            };
            let owner_ctx = req::OwnerCheckCtx::for_legacy(&inner);
            match req::install_macro_child(
                &mut inner,
                &child_stamp,
                &owner_ctx,
                false,
                req::MacroChildPayload::LegacyGripper {
                    spec: child_spec,
                    start: snap.commanded_positions,
                    request,
                    learn: crate::motion::LearnMode::Opening,
                    max_budget_s: Some(remaining_s),
                    extend_pos_force_close_timeout: true,
                },
            ) {
                Ok(v) => v.ticket,
                Err(ArmError::Motion(crate::motion::MotionError::InvalidCommand(msg)))
                    if msg.contains("remaining macro budget") =>
                {
                    return Err(fail(self.timeout_code, format!("Phase {phase}: {msg}")));
                }
                Err(other) => return Err(self.map_install_err(&mut inner, other)),
            }
        };
        self.wait_child(ticket).await
    }

    /// Helper to submit a staged child `PlanSlot` to `OWNER_EXECUTOR` via the shared bounded FIFO
    /// reply router (`ActionReplyRouter`) without leaking heap memory or blocking unrelated replies.
    async fn submit_child_slot_executor(
        &self,
        stamp: &crate::rt_core::requests::actions::ActionStamp,
        slot: crate::rt_core::ops::PlanSlot,
        check_workspace_revision: bool,
        calibration_revision: Option<u64>,
    ) -> Result<ActionId, Stop> {
        use crate::rt_core::broker::{AdmitError, ClientId, Reply};
        use crate::rt_core::requests::actions::ActionSubmit;
        use tokio::sync::oneshot::error::TryRecvError;
        let Some(handle) = self.sup.executor_handle() else {
            return Err(Stop::Cancelled);
        };
        let client = ClientId(stamp.admission.connection_id);
        let auth = stamp.broker_authority();
        let deadline = stamp.deadline;
        let boot = stamp.boot;
        let (req_id, mut rx) = loop {
            let write = |dst: &mut crate::rt_core::ops::PlanSlot| {
                *dst = slot.clone();
            };
            let res = handle.submit_internal_action(
                client,
                0,
                auth,
                deadline,
                ActionSubmit::InstallChild {
                    write: &write,
                    prepared_boot: boot,
                    check_workspace_revision,
                    calibration_revision,
                    grasp_setup_revision: stamp.grasp_setup_revision,
                    gripper_calib_revision: stamp.gripper_calib_revision,
                    power_budget_revision: stamp.power_budget_revision,
                },
            );
            match res {
                Ok(pair) => break pair,
                Err(AdmitError::Disconnected) => return Err(Stop::Cancelled),
                Err(AdmitError::PlansFull | AdmitError::CommandsFull | AdmitError::RepliesFull) => {
                    handle.pump_broker_replies();
                    if !self.alive().await {
                        return Err(Stop::Cancelled);
                    }
                    if Instant::now() > deadline {
                        return Err(fail(self.timeout_code, self.timeout_message.clone()));
                    }
                    tokio::time::sleep(Duration::from_millis(2)).await;
                }
            }
        };
        let wait_limit = deadline + Duration::from_secs(2);
        loop {
            handle.pump_broker_replies();
            match rx.try_recv() {
                Ok(reply) => {
                    return match reply {
                        Reply::Ack { action, .. } => Ok(action.unwrap_or(ActionId(self.id))),
                        Reply::Refused { code, .. } => match code {
                            "STOPPED" | "LEASE_LOST" | "ESTOP_ACTIVE" | "EPOCH_MISMATCH"
                            | "PARENT_REPLACED" => Err(Stop::Cancelled),
                            "DEADLINE_EXPIRED" => {
                                Err(fail(self.timeout_code, self.timeout_message.clone()))
                            }
                            other => Err(fail(other, format!("RT child refused: {other}"))),
                        },
                        Reply::Terminal { .. } => Ok(ActionId(self.id)),
                        Reply::Lease { .. } => Err(fail("INTERNAL", "unexpected Lease reply for RT child")),
                    };
                }
                Err(TryRecvError::Closed) => return Err(Stop::Cancelled),
                Err(TryRecvError::Empty) => {}
            }
            if !self.alive().await {
                handle.cancel_internal_waiter(req_id);
                return Err(Stop::Cancelled);
            }
            if Instant::now() > wait_limit {
                handle.cancel_internal_waiter(req_id);
                if let Ok(reply) = rx.try_recv() {
                    return match reply {
                        Reply::Ack { action, .. } => Ok(action.unwrap_or(ActionId(self.id))),
                        Reply::Refused { code, .. } => match code {
                            "STOPPED" | "LEASE_LOST" | "ESTOP_ACTIVE" | "EPOCH_MISMATCH"
                            | "PARENT_REPLACED" => Err(Stop::Cancelled),
                            "DEADLINE_EXPIRED" => {
                                Err(fail(self.timeout_code, self.timeout_message.clone()))
                            }
                            other => Err(fail(other, format!("RT child refused: {other}"))),
                        },
                        Reply::Terminal { .. } => Ok(ActionId(self.id)),
                        Reply::Lease { .. } => Err(fail("INTERNAL", "unexpected Lease reply for RT child")),
                    };
                }
                return Err(fail(self.timeout_code, self.timeout_message.clone()));
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
    }

    /// Run a gripper open / close sub-motion and wait for it. Opens at 1 rad/s within 8 s; a
    /// close runs at `min(gripper_policy.close_speed_rad_s, damiao_gripper.close_speed_rad_s when
    /// pos_force)` (wire-quantized in `pos_force`) with the derived deadline.
    pub async fn gripper(&self, phase: &str, command: GripperCommand) -> Result<SubDone, Stop> {
        self.run_gripper_planned(phase, command, None, command == GripperCommand::Close).await
    }

    /// Run a gripper move sub-motion with a fraction in [0, 1] and wait for it: at the unified
    /// closing speed with the derived deadline when closing, else 1 rad/s within 8 s.
    pub async fn gripper_move(&self, phase: &str, fraction: f64) -> Result<SubDone, Stop> {
        self.run_gripper_planned(phase, GripperCommand::Move, Some(fraction), false).await
    }

    /// [`phase`](Self::phase) followed by [`gripper`](Self::gripper).
    pub async fn gripper_phase(&self, phase: &str, progress: f32, command: GripperCommand) -> Result<SubDone, Stop> {
        self.phase(phase, progress).await?;
        self.gripper(phase, command).await
    }

    /// Revision-fenced gripper child for adaptive Phase B execution: checks ALL FIVE captured
    /// revisions before staging AND binds ALL FIVE revisions atomically on `InstallChild`
    /// (both Legacy and RT Executor, strictly J7 intents through `GripperMotion`).
    pub async fn run_gripper_fenced(
        &self,
        phase: &str,
        progress: f32,
        command: GripperCommand,
        fraction: Option<f64>,
        grasp: bool,
        revisions: crate::grasp_select::AdaptiveRevisions,
    ) -> Result<SubDone, Stop> {
        use crate::rt_core::requests::actions as req;
        self.phase(phase, progress).await?;
        if !self.alive().await {
            return Err(Stop::Cancelled);
        }
        let now = Instant::now();
        if now >= self.deadline {
            return Err(fail(self.timeout_code, self.timeout_message.clone()));
        }
        let snap = self.snapshot(self.deadline).await?;
        if snap.env.calibration_revision != revisions.calibration_revision {
            return Err(fail("CALIBRATION_CHANGED", format!("[{phase}] calibration changed during adaptive grasp")));
        }
        if snap.env.workspace_revision != revisions.workspace_revision
            || snap.env.grasp_setup_revision != revisions.grasp_setup_revision
            || snap.env.gripper_calib_revision != revisions.gripper_calib_revision
            || snap.env.power_budget_revision != revisions.power_budget_revision
        {
            return Err(fail("PLAN_STALE", format!("[{phase}] configuration changed during adaptive grasp")));
        }
        let closes = match command {
            GripperCommand::Close | GripperCommand::Seek => true,
            GripperCommand::Open => false,
            GripperCommand::Move => match (
                snap.env.config.gripper_policy.as_ref(),
                snap.env.config.gripper_index(),
            ) {
                (Some(p), Some(g)) => {
                    let target =
                        p.closed_rad + fraction.unwrap_or(0.0) as f32 * (p.open_rad - p.closed_rad);
                    let dir = if p.closed_rad >= p.open_rad { 1.0 } else { -1.0 };
                    (target - snap.commanded_positions.get(g).copied().unwrap_or(target)) * dir
                        > 1e-3
                }
                _ => true,
            },
        };
        let close_speed = snap
            .env
            .config
            .gripper_policy
            .as_ref()
            .map_or(f64::from(crate::model::DEFAULT_CLOSE_SPEED_RAD_S), |p| p.close_speed());
        let (speed, timeout_s) = if closes { (close_speed, None) } else { (1.0, Some(8.0)) };
        let request = GripperRequest { command, fraction, grasp, speed, timeout_s, tag: None };
        let remaining_s = self.deadline.saturating_duration_since(Instant::now()).as_secs_f64();
        let mut child_stamp = snap.stamp.for_child(ActionId(self.id), self.deadline);
        child_stamp.config_revision = revisions.workspace_revision;
        child_stamp.calibration_revision = Some(revisions.calibration_revision);
        child_stamp.poses_revision = None;
        child_stamp.sequences_revision = None;
        child_stamp.grasp_setup_revision = Some(revisions.grasp_setup_revision);
        child_stamp.gripper_calib_revision = Some(revisions.gripper_calib_revision);
        child_stamp.power_budget_revision = Some(revisions.power_budget_revision);

        let ticket = if self.is_executor() {
            let m = snap.owner.macro_state;
            let next_seq = (m.active_id == self.id && m.alive)
                .then(|| m.child_seq.checked_add(1))
                .flatten()
                .ok_or(Stop::Cancelled)?;
            let child_spec = ChildSpec {
                ticket: ChildTicket {
                    parent: ActionId(self.id),
                    child_seq: next_seq,
                },
                timeout_s: self.phase_timeout_s,
                timeout_code: self.timeout_code,
                contact: None,
            };
            let slot = req::prepare_child_gripper_slot(
                &snap.commanded_positions,
                command,
                fraction,
                grasp,
                speed,
                timeout_s,
                Some(remaining_s),
                true,
                child_spec,
            )
            .map_err(|c| fail(c, format!("[{phase}] failed to stage gripper child: {c}")))?;
            self.submit_child_slot_executor(
                &child_stamp,
                slot,
                true,
                Some(revisions.calibration_revision),
            )
            .await
            .map_err(|s| tag_phase_stop(phase, s))?;
            child_spec.ticket
        } else {
            let mut inner = self.sup.inner().write().await;
            let Some(next_seq) = inner
                .macro_run
                .as_ref()
                .filter(|m| m.id == self.id && inner.macro_alive(self.id))
                .and_then(|m| m.next_child_seq())
            else {
                return Err(Stop::Cancelled);
            };
            let child_spec = ChildSpec {
                ticket: ChildTicket {
                    parent: ActionId(self.id),
                    child_seq: next_seq,
                },
                timeout_s: self.phase_timeout_s,
                timeout_code: self.timeout_code,
                contact: None,
            };
            let owner_ctx = req::OwnerCheckCtx::for_legacy(&inner);
            match req::install_macro_child(
                &mut inner,
                &child_stamp,
                &owner_ctx,
                true,
                req::MacroChildPayload::LegacyGripper {
                    spec: child_spec,
                    start: snap.commanded_positions,
                    request,
                    learn: crate::motion::LearnMode::Opening,
                    max_budget_s: Some(remaining_s),
                    extend_pos_force_close_timeout: true,
                },
            ) {
                Ok(v) => v.ticket,
                Err(ArmError::Motion(crate::motion::MotionError::InvalidCommand(msg)))
                    if msg.contains("remaining macro budget") =>
                {
                    return Err(fail(self.timeout_code, format!("[{phase}] {msg}")));
                }
                Err(other) => return Err(tag_phase_stop(phase, self.map_install_err(&mut inner, other))),
            }
        };
        self.wait_child(ticket).await.map_err(|s| tag_phase_stop(phase, s))
    }
}

fn tag_phase_stop(phase: &str, stop: Stop) -> Stop {
    match stop {
        Stop::Fail(code, msg) => {
            let prefix = format!("[{phase}]");
            if msg.starts_with(&prefix) {
                Stop::Fail(code, msg)
            } else {
                Stop::Fail(code, format!("{prefix} {msg}"))
            }
        }
        other => other,
    }
}

/// Production broker-to-worker `BeginMacro` handoff on `OWNER_EXECUTOR` (P-F1):
/// atomically reserves bounded reply capacity and binds the terminal destination via
/// [`RtExecutorHandle::submit_external_action`], carrying the caller's original `AdmissionStamp`
/// (including `admission_request_id`) and exact absolute `spec.macro_deadline`.
///
/// Returns `(accepted ActionId, broker RequestId, AckCommitGuard)` on `Ack`. For WebSocket
/// callers (`require_ack_commit == true`), the caller MUST call `guard.confirm_enqueued()`
/// strictly AFTER enqueuing `command_ack` onto the socket's outbound control queue.
pub(crate) async fn begin_macro_executor_with_barrier(
    sup: &ArmSupervisor,
    stamp: &crate::rt_core::requests::actions::ActionStamp,
    spec: crate::rt_core::requests::actions::BeginMacroSpec,
    submit_deadline: Instant,
    policy: crate::rt_core::reply_router::ExternalDestinationPolicy,
    require_ack_commit: bool,
    wire_tracker: Option<Arc<crate::rt_core::reply_router::WsWireAckTracker>>,
) -> Result<(u64, crate::rt_core::broker::RequestId, crate::rt_core::reply_router::AckCommitGuard), ArmError> {
    use crate::rt_core::broker::{AdmitError, ClientId, Reply};
    use crate::rt_core::reply_router::ExternalSubmitError;
    use crate::rt_core::requests::actions::{ActionRefusal, ActionSubmit};
    let Some(handle) = sup.executor_handle() else {
        return Err(ArmError::Rejected {
            status: 503,
            code: "EXECUTOR_UNAVAILABLE".into(),
            message: "RT executor is not active".into(),
        });
    };
    let client = ClientId(stamp.admission.connection_id);
    let auth = stamp.broker_authority();
    let (req_id, mut rx, ack_guard) = loop {
        match handle.submit_external_action(
            client,
            0,
            auth,
            submit_deadline,
            ActionSubmit::BeginMacro(spec),
            policy,
            require_ack_commit,
            wire_tracker.clone(),
        ) {
            Ok(triple) => break triple,
            Err(ExternalSubmitError::NoDestination) => {
                return Err(ArmError::Rejected {
                    status: 503,
                    code: "EXECUTOR_ROUTING_PENDING".into(),
                    message: "RT executor has no reply destination bound for this connection".into(),
                });
            }
            Err(ExternalSubmitError::Stopped) => {
                return Err(ArmError::Rejected {
                    status: 503,
                    code: "EXECUTOR_STOPPED".into(),
                    message: "RT executor stopped before admitting BeginMacro".into(),
                });
            }
            Err(ExternalSubmitError::NotAnExternalAction) => {
                return Err(ArmError::Internal("BeginMacro rejected as non-external action".into()));
            }
            Err(ExternalSubmitError::Admit(AdmitError::Disconnected)) => {
                return Err(ActionRefusal::new("LEASE_LOST").into_arm_error_with_epoch(auth.epoch));
            }
            Err(ExternalSubmitError::Admit(AdmitError::RepliesFull)) => {
                handle.pump_broker_replies();
                if Instant::now() > submit_deadline {
                    return Err(ActionRefusal::new("TERMINALS_BUSY")
                        .into_arm_error_with_epoch(auth.epoch));
                }
                tokio::time::sleep(Duration::from_millis(2)).await;
            }
            Err(ExternalSubmitError::Admit(AdmitError::PlansFull | AdmitError::CommandsFull)) => {
                handle.pump_broker_replies();
                if Instant::now() > submit_deadline {
                    return Err(ActionRefusal::new("DEADLINE_EXPIRED")
                        .into_arm_error_with_epoch(auth.epoch));
                }
                tokio::time::sleep(Duration::from_millis(2)).await;
            }
        }
    };
    let wait_limit = submit_deadline + Duration::from_secs(2);
    loop {
        handle.pump_broker_replies();
        let remaining = wait_limit.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            // Dropping `ack_guard` unconfirmed abandons the terminal and recovers credit cleanly.
            drop(ack_guard);
            return Err(ActionRefusal::new("DEADLINE_EXPIRED").into_arm_error_with_epoch(auth.epoch));
        }
        let step = remaining.min(Duration::from_millis(2));
        match tokio::time::timeout(step, &mut rx).await {
            Ok(Ok(Ok(Reply::Ack { action: Some(id), .. }))) => return Ok((id.0, req_id, ack_guard)),
            Ok(Ok(Ok(Reply::Ack { action: None, .. }))) => {
                drop(ack_guard);
                return Err(ArmError::Internal("BeginMacro ACK omitted ActionId".into()));
            }
            Ok(Ok(Ok(Reply::Refused { code, .. }))) => {
                drop(ack_guard);
                let epoch = sup
                    .hub()
                    .latest()
                    .map_or(auth.epoch, |s| s.rt.owner.authority.epoch);
                return Err(ActionRefusal::new(code).into_arm_error_with_epoch(epoch));
            }
            Ok(Ok(Ok(Reply::Terminal { .. }))) => {
                drop(ack_guard);
                return Err(ArmError::Internal("unexpected Terminal before BeginMacro ACK".into()));
            }
            Ok(Ok(Ok(Reply::Lease { .. }))) => {
                drop(ack_guard);
                return Err(ArmError::Internal("unexpected Lease reply for BeginMacro".into()));
            }
            Ok(Ok(Err(_))) | Ok(Err(_)) => {
                drop(ack_guard);
                return Err(ArmError::Rejected {
                    status: 503,
                    code: "EXECUTOR_STOPPED".into(),
                    message: "RT executor stopped before acknowledging BeginMacro".into(),
                });
            }
            Err(_) => {
                // Step timeout: loop to pump broker replies (for tests without `armd-dispatch`).
            }
        }
    }
}

/// Production broker-to-worker `BeginMacro` handoff on `OWNER_EXECUTOR` (A7 / P-F1):
/// submits `ActionSubmit::BeginMacro(spec)` through the per-connection reply router (or attached
/// global fallback sink for direct non-WS callers/tests), returning `(accepted ActionId,
/// broker RequestId)` on `Ack`.
pub async fn begin_macro_executor(
    sup: &ArmSupervisor,
    stamp: &crate::rt_core::requests::actions::ActionStamp,
    spec: crate::rt_core::requests::actions::BeginMacroSpec,
    submit_deadline: Instant,
) -> Result<(u64, crate::rt_core::broker::RequestId), ArmError> {
    let (id, req_id, guard) = begin_macro_executor_with_barrier(
        sup,
        stamp,
        spec,
        submit_deadline,
        crate::rt_core::reply_router::ExternalDestinationPolicy::ConnectionOrGlobal,
        false,
        None,
    )
    .await?;
    let _ = guard.confirm_enqueued();
    Ok((id, req_id))
}
