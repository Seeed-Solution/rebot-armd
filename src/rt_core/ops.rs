//! Narrow operations on one arm's core and the deterministic release-stepping fixture
//! (armd 1.6 RT core, step 2 foundation).
//!
//! Workers never get mutable access to the core: they receive a [`PlanningSnapshot`] and
//! submit through the broker; the RT side applies one of these operations per item:
//!
//! | Operation | Used for |
//! |---|---|
//! | [`CoreOps::planning_snapshot`] | the state a worker plans from (stamped, owned copy) |
//! | [`CoreOps::commit_primitive`] | install one admitted primitive (track C, E children) |
//! | [`CoreOps::consume_target`] | apply one latest target (track D) |
//! | [`ConfigAuthority`] (`reserve` / `activate` / `release`) | versioned configuration (track B) |
//! | [`CoreOps::poll_hardware`] | the bus owner's progress and confirmations (track A) |
//! | [`CoreOps::advance`] | one release of the control tick (integrator) |
//!
//! The production adapter (`impl CoreOps for SupervisorInner`) delegates each operation to
//! the supervisor's existing behaviour, under the legacy write guard that still owns the
//! single core. Only it can make a [`Committed`] proof, which [`super::broker::Taken::ack`] requires:
//! a stub or test double ([`StubCore`]) can refuse but never acknowledge success, and has no
//! way to move hardware.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use super::broker::{
    ActionId, Authority, BootId, Broker, Capacities, CommandKind, Descriptor, ImmediateOp, Installed, Outcome, RequestId, RtIntake,
};
use super::config_iface::{Activated, ConfigAuthority, ConfigDomain, ConfigProposal, Durability, Reservation, Reserved};
use super::ids::ArmId;
use super::mailbox::{target_mailbox, TargetFrame, TargetReader, TargetWriter, MAX_DOF};
use super::urgent::StopRequests;
use crate::driver::EnableState;
use crate::supervisor::{ArmError, SupervisorInner};

/// Proof that an authorized core operation committed the command of one request; required
/// to acknowledge success ([`super::broker::Taken::ack`] consumes it and checks the request).
/// Only this module's production adapter constructs one. Neither `Clone` nor `Copy`: one
/// commit acknowledges one descriptor, once.
///
/// ```compile_fail
/// fn copy<T: Copy>() {}
/// copy::<armd::rt_core::ops::Committed>();
/// ```
///
/// ```compile_fail
/// fn clone<T: Clone>() {}
/// clone::<armd::rt_core::ops::Committed>();
/// ```
#[derive(Debug, PartialEq, Eq)]
pub struct Committed {
    request: RequestId,
    action: Option<ActionId>,
}

impl Committed {
    fn authorized(d: &Descriptor, action: Option<ActionId>) -> Self { Self { request: d.stamp.request, action } }

    /// The request whose command was committed.
    pub fn request(&self) -> RequestId { self.request }

    /// The committed action (None for an immediate command).
    pub fn action(&self) -> Option<ActionId> { self.action }

    #[cfg(test)]
    pub(crate) fn for_tests(request: RequestId, action: Option<ActionId>) -> Self { Self { request, action } }
}

/// A primitive staged in a plan slot. Fixed-size, so a slot is pre-touched at construction;
/// tracks add variants (Cartesian path, same-side approach, replay) through the integrator.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum Primitive {
    #[default]
    None,
    /// Joint-space quintic from `start[..dof]` to `q[..dof]` in `duration_s`
    /// (`SupervisorInner::jog_action`). `start` is the commanded position the plan was
    /// prepared from, every joint including the gripper: the commit refuses it (`PLAN_STALE`)
    /// when any commanded joint differs from it by `>= START_TOLERANCE`.
    JointQuintic { dof: u8, start: [f32; MAX_DOF], q: [f32; MAX_DOF], duration_s: f64 },
}

/// A prepared start differs from the commanded position when a joint (gripper included)
/// moved by at least this much (rad; `SupervisorInner::check_plan_fresh`).
pub const START_TOLERANCE: f32 = 1e-4;

impl Primitive {
    /// A joint quintic prepared from `start` (the planning snapshot's commanded positions).
    /// `INVALID_COMMAND` when `q` is empty or longer than [`MAX_DOF`], or `start` has
    /// another length: nothing is truncated.
    pub fn joint_quintic(start: &[f32], q: &[f32], duration_s: f64) -> Result<Self, &'static str> {
        let n = q.len();
        if n == 0 || n > MAX_DOF || start.len() != n { return Err("INVALID_COMMAND"); }
        let (mut a, mut s0) = ([0.0; MAX_DOF], [0.0; MAX_DOF]);
        a[..n].copy_from_slice(q);
        s0[..n].copy_from_slice(start);
        Ok(Self::JointQuintic { dof: n as u8, start: s0, q: a, duration_s })
    }
}

/// Codes an action can end with ([`Outcome`] carries them as `&'static str`).
pub const TERMINAL_CODES: &[&str] = &[
    // stop / ownership / epoch
    "STOPPED", "CANCELLED", "SUPERSEDED", "LEASE_LOST", "EPOCH_MISMATCH", "PLAN_STALE", "CALIBRATION_CHANGED", "CONFIG_BUSY",
    "DEPLOYMENT_CHANGED", "TOOL_MISMATCH", "DATA_STALE", "NOT_ARMED", "MOTION_BUSY", "INTERNAL", "UNSUPPORTED",
    // faults and holds (FaultCode, SCREAMING_SNAKE_CASE)
    "ESTOP", "ESTOP_ACTIVE", "FAULT", "FAULT_ACTIVE", "BUS_LOSS", "THERMAL_TRIP", "JOINT_LIMIT", "WATCHDOG", "WATCHDOG_TIMEOUT",
    "FOLLOWING_ERROR", "CONTROL_DEADLINE", "INVALID_TELEMETRY", "DRIVER_WRITE_FAILED", "HARDWARE_FAULT", "ENABLE_FAILED",
    "SERVO_NO_BRAKE", "HANDOVER_UNSETTLED", "GRAVITY_COMP_FAILED", "COLLISION", "COLLISION_HOLD", "BLOCKED_HOLD", "WORKSPACE_HOLD",
    "TORQUE_LIMIT", "TRACKING_STALL", "SAFETY_ERROR", "SAFETY_REJECTED", "SOFT_LIMIT_EXCEEDED", "FEEDBACK_UNAVAILABLE",
    "FEEDBACK_UNCONFIRMED", "INVALID_FEEDBACK", "UNLOAD_FAILED",
    // motion and action failures
    "IK_UNREACHABLE", "IK_BRANCH_FLIP", "WORKSPACE_VIOLATION", "INVALID_COMMAND", "INVALID_TARGET", "JUMP_LIMIT", "STEP_LIMIT",
    "SERVO_INFEASIBLE", "SERVO_TARGET_DISCARDED", "STALE_INPUT", "MOTION_TIMEOUT", "GRIPPER_TIMEOUT", "GRASP_TIMEOUT",
    "PLACE_TIMEOUT", "TARGET_TIMEOUT", "SEQUENCE_TIMEOUT", "MAX_SECONDS", "NOT_HOLDING", "TARGET_LOST", "TARGET_DISPLACED",
    "DROPPED", "OBJECT_HELD", "OBJECT_TOO_TALL", "TOO_WIDE", "CENTRING_FAILED", "DETECTION_UNAVAILABLE", "TASK_UNSUPPORTED",
    "OBSERVATION_UNAVAILABLE", "PERSIST_FAILED", "POLICY_NOT_ACTIVE",
];

/// The static form of an action's code (None: it named none or an unlisted one).
fn terminal_code(code: Option<&str>) -> Option<&'static str> {
    let code = code?;
    TERMINAL_CODES.iter().find(|k| **k == code).copied()
}

/// How an ended action translates to an [`Outcome`] (state `running`: None).
pub fn outcome_of(state: &str, code: Option<&str>) -> Option<Outcome> {
    Some(match (state, terminal_code(code)) {
        ("running", _) => return None,
        ("done", _) => Outcome::Completed,
        ("cancelled", None | Some("STOPPED" | "CANCELLED")) => Outcome::Stopped,
        ("cancelled", Some("SUPERSEDED")) => Outcome::Superseded,
        ("cancelled", Some(c)) => Outcome::Cancelled(c),
        (_, c) => Outcome::Failed(c.unwrap_or("FAILED")),
    })
}

/// The prepared start of a primitive matches `commanded` (every joint, gripper included).
fn start_matches(start: &[f32], commanded: &[f32]) -> bool {
    start.len() == commanded.len() && start.iter().zip(commanded).all(|(s, c)| (s - c).abs() < START_TOLERANCE)
}

/// The state a worker plans from: an owned copy, stamped with the publication sequence and
/// the authority it is valid under.
#[derive(Debug, Clone)]
pub struct PlanningSnapshot {
    pub arm: ArmId,
    /// `seq` of the last publication when the snapshot was taken.
    pub published_seq: u64,
    pub authority: Authority,
    pub measured: Vec<f32>,
    pub commanded: Vec<f32>,
    pub feedback_instants: Vec<Option<Instant>>,
    pub feedback_seqs: Vec<u64>,
    pub motion_active: bool,
    pub taken_at: Instant,
}

/// What the RT knows when it commits: its identity, the time and the urgent-stop generation.
#[derive(Debug, Clone, Copy)]
pub struct CommitCtx {
    pub arm: ArmId,
    pub boot: BootId,
    pub now: Instant,
    pub stop_generation: u64,
}

/// Receive frames the bus owner services per release at most (design: 64 frames or 5 ms).
pub const RX_FRAME_BUDGET: u16 = 64;
/// Time the bus owner services per release at most.
pub const RX_TIME_BUDGET: Duration = Duration::from_millis(5);

/// What one [`CoreOps::poll_hardware`] may spend: it returns by `deadline` (the earlier of
/// the release deadline and the service budget) and services at most `max_rx_frames`.
/// `now` is the executor's clock (virtual in [`ReleaseFixture`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ServiceBudget {
    pub now: Instant,
    pub deadline: Instant,
    pub max_rx_frames: u16,
}

impl ServiceBudget {
    /// The per-release budget at `now` of a release ending at `release_deadline`.
    pub fn for_release(now: Instant, release_deadline: Instant) -> Self {
        Self { now, deadline: (now + RX_TIME_BUDGET).min(release_deadline), max_rx_frames: RX_FRAME_BUDGET }
    }
}

/// One hardware poll: feedback validity and enable state, and what the poll serviced (no
/// bus traffic today: `rx_frames` 0).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct HardwarePoll {
    /// No driver (mock follower).
    pub driverless: bool,
    pub feedback_valid: bool,
    pub enable_state: Option<EnableState>,
    pub oldest_feedback_age: Option<Duration>,
    /// Receive frames serviced in this poll (<= `ServiceBudget::max_rx_frames`).
    pub rx_frames: u16,
    /// The poll stopped at its budget (deadline or frame bound) with work left.
    pub budget_exhausted: bool,
}

/// The narrow operations of one arm's core.
pub trait CoreOps {
    /// Current authority at `ctx.now` and `ctx.stop_generation` (the urgent-stop generation
    /// is not core state): the lease generation of a lease valid at `ctx.now` (0: none), the
    /// generation of the parent that may install children (0: none).
    fn authority(&self, ctx: &CommitCtx) -> Authority;
    fn planning_snapshot(&self, ctx: &CommitCtx) -> PlanningSnapshot;
    /// Installs `primitive` for `d` after rechecking its stamp; refusal leaves active motion unchanged.
    fn commit_primitive(&mut self, d: &Descriptor, primitive: &Primitive, ctx: &CommitCtx) -> Result<Committed, &'static str>;
    fn commit_immediate(&mut self, d: &Descriptor, op: ImmediateOp, ctx: &CommitCtx) -> Result<Committed, &'static str>;
    /// Applies one target after rechecking its authority.
    fn consume_target(&mut self, frame: &TargetFrame, ctx: &CommitCtx) -> Result<(), &'static str>;
    /// An authorized stop was requested: stop active motion.
    fn apply_stop(&mut self);
    /// How `action` ended, None while it runs.
    fn action_outcome(&self, action: ActionId) -> Option<Outcome>;
    /// The bus owner's progress and confirmations within `budget` (exclusive: the bus
    /// owner's state machine advances here).
    fn poll_hardware(&mut self, budget: &ServiceBudget) -> HardwarePoll;
    /// One release of the control tick (`generation`: the publication's lock generation).
    fn advance(&mut self, dt: f64, generation: u64);
}

/// The protocol code of a delegated error, as a static string (unknown codes: `REJECTED`).
fn static_code(e: ArmError) -> &'static str {
    const KNOWN: &[&str] = &[
        "MOTION_BUSY", "NOT_ARMED", "ESTOP_ACTIVE", "FAULT_ACTIVE", "EPOCH_MISMATCH", "INVALID_COMMAND", "IK_UNREACHABLE",
        "WORKSPACE_VIOLATION", "JOINT_LIMIT", "PLAN_STALE", "LEASE_LOST", "NO_LEASE", "COLLISION_HOLD", "COMMISSIONING_REQUIRED",
        "INVALID_VALUE", "UNSUPPORTED", "INTERNAL",
    ];
    if let ArmError::Motion(m) = &e { return m.code(); }
    let code = crate::api::rest::ApiError::from(e).code.unwrap_or_default();
    KNOWN.iter().find(|k| **k == code.as_str()).copied().unwrap_or("REJECTED")
}

impl SupervisorInner {
    /// The lease valid at `now` (its generation names its holder).
    fn lease_valid_at(&self, now: Instant) -> Option<&crate::supervisor::ActiveLease> {
        self.active_lease.as_ref().filter(|l| now.saturating_duration_since(l.acquired_at) < self.lease_timeout)
    }

    /// The stamp checks every commit shares: identity, deadline, the current authority, and
    /// a lease when leases are enforced.
    fn check_commit(&self, d: &Descriptor, ctx: &CommitCtx) -> Result<(), &'static str> {
        if let Some(code) = d.stamp.check(ctx.arm, ctx.boot, ctx.now, &self.authority(ctx)) { return Err(code); }
        if self.lease_enabled && d.stamp.authority.lease_generation == 0 { return Err("NO_LEASE"); }
        Ok(())
    }
}

impl CoreOps for SupervisorInner {
    fn authority(&self, ctx: &CommitCtx) -> Authority {
        Authority {
            lease_generation: self.lease_valid_at(ctx.now).map_or(0, |l| l.generation),
            epoch: self.safety.epoch_id(),
            stop_generation: ctx.stop_generation,
            config_revision: self.workspace_revision,
            // the running action is the parent its children are installed for
            parent_generation: self.last_action.as_ref().filter(|a| a.is_running()).map_or(0, |a| a.id),
        }
    }

    fn planning_snapshot(&self, ctx: &CommitCtx) -> PlanningSnapshot {
        PlanningSnapshot {
            arm: self.core.id(),
            published_seq: self.core.published_seq(),
            authority: self.authority(ctx),
            measured: self.measured_positions.clone(),
            commanded: self.commanded_positions.clone(),
            feedback_instants: self.core.feedback().instants().to_vec(),
            feedback_seqs: self.core.feedback().seqs().to_vec(),
            motion_active: self.active_trajectory.is_some(),
            taken_at: ctx.now,
        }
    }

    fn commit_primitive(&mut self, d: &Descriptor, primitive: &Primitive, ctx: &CommitCtx) -> Result<Committed, &'static str> {
        self.check_commit(d, ctx)?;
        if self.core.motion_inhibited_by_config() { return Err("CONFIG_BUSY"); }
        match primitive {
            Primitive::None => Err("INVALID_COMMAND"),
            Primitive::JointQuintic { dof, start, q, duration_s } => {
                // the fields are public: a hand-built primitive may name more joints than it holds
                let n = usize::from(*dof);
                if n == 0 || n > MAX_DOF { return Err("INVALID_COMMAND"); }
                if !start_matches(&start[..n], &self.commanded_positions) { return Err("PLAN_STALE"); }
                let target = q[..n].to_vec();
                self.jog_action(target, Some(*duration_s), Some(d.stamp.authority.epoch), "joint_move")
                    .map(|(_, id, _)| Committed::authorized(d, Some(ActionId(id))))
                    .map_err(static_code)
            }
        }
    }

    fn commit_immediate(&mut self, d: &Descriptor, op: ImmediateOp, ctx: &CommitCtx) -> Result<Committed, &'static str> {
        self.check_commit(d, ctx)?;
        let r = match op {
            // renews the lease the command was admitted under, if it is still valid at the
            // commit instant (a queued renewal cannot resurrect an expired lease)
            ImmediateOp::Heartbeat => {
                let generation = d.stamp.authority.lease_generation;
                match self.active_lease.as_mut().filter(|l| l.generation == generation && generation != 0) {
                    Some(l) => {
                        l.acquired_at = ctx.now;
                        self.last_heartbeat = ctx.now;
                        Ok(())
                    }
                    None => return Err(if generation == 0 { "NO_LEASE" } else { "LEASE_LOST" }),
                }
            }
            ImmediateOp::TorqueOn => self.set_torque(true).map(drop).map_err(ArmError::Safety),
            ImmediateOp::TorqueOff => self.set_torque(false).map(drop).map_err(ArmError::Safety),
        };
        r.map(|_| Committed::authorized(d, None)).map_err(static_code)
    }

    fn consume_target(&mut self, frame: &TargetFrame, ctx: &CommitCtx) -> Result<(), &'static str> {
        if usize::from(frame.dof) > MAX_DOF { return Err("INVALID_COMMAND"); }
        if let Some(code) = frame.authority.admits(&self.authority(ctx)) { return Err(code); }
        self.servo(frame.positions().to_vec(), frame.authority.epoch).map_err(|e| static_code(ArmError::Safety(e)))
    }

    fn apply_stop(&mut self) { self.stop_motion(); }

    fn action_outcome(&self, action: ActionId) -> Option<Outcome> {
        let Some(a) = self.last_action.as_ref().filter(|a| a.id == action.0) else { return Some(Outcome::Superseded); };
        outcome_of(&a.state, a.code.as_deref())
    }

    fn poll_hardware(&mut self, budget: &ServiceBudget) -> HardwarePoll {
        // no bus servicing is delegated yet (the tick reads feedback): only the budget is reported
        let budget_exhausted = budget.now > budget.deadline;
        let Some(driver) = self.driver.as_ref() else { return HardwarePoll { driverless: true, budget_exhausted, ..HardwarePoll::default() }; };
        let d = driver.lock().unwrap_or_else(|e| e.into_inner());
        let ages = d.feedback_ages();
        HardwarePoll {
            driverless: false,
            feedback_valid: d.is_feedback_valid(),
            enable_state: Some(d.enable_state()),
            oldest_feedback_age: ages.iter().copied().try_fold(Duration::ZERO, |m, a| a.map(|a| m.max(a))),
            rx_frames: 0,
            budget_exhausted,
        }
    }

    fn advance(&mut self, dt: f64, generation: u64) {
        drop(self.tick_publish(dt, generation, false));
    }
}

/// The supervisor's configuration revisions; reservations are held in the core. Activation
/// stays on the existing commit path (`NOT_WIRED` here) until track B moves it.
impl ConfigAuthority for SupervisorInner {
    fn revision(&self, domain: ConfigDomain) -> u64 {
        match domain {
            ConfigDomain::Workspace => self.workspace_revision,
            ConfigDomain::Keepout => self.keepout.revision,
            ConfigDomain::Collision => self.collision.revision,
            ConfigDomain::Calibration => self.calibration_revision,
            ConfigDomain::GripperCalibration => self.gripper_calib_revision,
            ConfigDomain::GraspSetup => self.grasp_setup_revision,
            ConfigDomain::ToolCommissioning => self.tool_commissioning_revision,
            ConfigDomain::Poses => self.poses.revision,
            ConfigDomain::Sequences => self.sequences.revision,
        }
    }

    fn reserve(&mut self, p: &ConfigProposal, now: Instant) -> Reserved {
        let current = self.revision(p.domain);
        if now > p.stamp.deadline { return Reserved::Refused { code: "DEADLINE_EXPIRED", current }; }
        if p.expected_revision != current { return Reserved::Refused { code: "REVISION_MISMATCH", current }; }
        let r = Reservation { domain: p.domain, from: current, candidate: p.candidate, inhibits_motion: p.domain.affects_motion() };
        if !self.core.hold_config(r) { return Reserved::Refused { code: "CONFIG_BUSY", current }; }
        Reserved::Granted(r)
    }

    fn activate(&mut self, _r: Reservation, _durability: Durability) -> Activated { Activated::Refused { code: "NOT_WIRED" } }

    fn release(&mut self, r: Reservation) { self.core.release_config(&r); }
}

/// A core double for contract tests: refuses every commit and target (`NOT_WIRED`), moves
/// nothing, and cannot make a [`Committed`] proof.
#[derive(Debug, Default)]
pub struct StubCore {
    pub commits_seen: u64,
    pub targets_seen: u64,
    pub releases: u64,
    pub polls: u64,
    /// The budget of the last poll.
    pub last_budget: Option<ServiceBudget>,
}

impl CoreOps for StubCore {
    fn authority(&self, ctx: &CommitCtx) -> Authority { Authority { stop_generation: ctx.stop_generation, ..Authority::default() } }
    fn planning_snapshot(&self, ctx: &CommitCtx) -> PlanningSnapshot {
        PlanningSnapshot { arm: ArmId::LOCAL, published_seq: 0, authority: self.authority(ctx), measured: Vec::new(), commanded: Vec::new(),
            feedback_instants: Vec::new(), feedback_seqs: Vec::new(), motion_active: false, taken_at: ctx.now }
    }
    fn commit_primitive(&mut self, _: &Descriptor, _: &Primitive, _: &CommitCtx) -> Result<Committed, &'static str> { self.commits_seen += 1; Err("NOT_WIRED") }
    fn commit_immediate(&mut self, _: &Descriptor, _: ImmediateOp, _: &CommitCtx) -> Result<Committed, &'static str> { self.commits_seen += 1; Err("NOT_WIRED") }
    fn consume_target(&mut self, _: &TargetFrame, _: &CommitCtx) -> Result<(), &'static str> { self.targets_seen += 1; Err("NOT_WIRED") }
    fn apply_stop(&mut self) {}
    fn action_outcome(&self, _: ActionId) -> Option<Outcome> { Some(Outcome::Failed("NOT_WIRED")) }
    fn poll_hardware(&mut self, budget: &ServiceBudget) -> HardwarePoll {
        self.polls += 1;
        self.last_budget = Some(*budget);
        HardwarePoll { driverless: true, ..HardwarePoll::default() }
    }
    fn advance(&mut self, _: f64, _: u64) { self.releases += 1; }
}

/// What one release did.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ReleaseReport {
    pub release: u64,
    /// An urgent stop was applied in this release (at its start or at a checkpoint).
    pub stopped: bool,
    /// The ordinary command taken, with its commit sequence or refusal code.
    pub command: Option<(RequestId, Result<u64, &'static str>)>,
    /// The target consumed, with its sequence or refusal code.
    pub target: Option<(u64, Result<(), &'static str>)>,
    /// The active action that ended in this release.
    pub ended: Option<(ActionId, Outcome)>,
    /// The hardware poll of this release.
    pub poll: HardwarePoll,
}

/// The fixture's clock: virtual monotonic time, advanced by the fixture (one `dt` per
/// release) or by a test ([`Self::advance`]); clones share it. Never reads the wall clock
/// after construction.
#[derive(Debug, Clone)]
pub struct VirtualClock {
    base: Instant,
    offset_ns: Arc<AtomicU64>,
}

impl VirtualClock {
    /// A clock starting at `base`.
    pub fn starting_at(base: Instant) -> Self { Self { base, offset_ns: Arc::new(AtomicU64::new(0)) } }
    pub fn base(&self) -> Instant { self.base }
    pub fn now(&self) -> Instant { self.base + Duration::from_nanos(self.offset_ns.load(Ordering::SeqCst)) }
    pub fn advance(&self, d: Duration) { self.offset_ns.fetch_add(d.as_nanos() as u64, Ordering::SeqCst); }
}

/// Advances one core one release at a time, in the executor's order: urgent stop, hardware
/// poll, at most one ordinary command, the latest target, the control tick, terminal
/// detection. Tests and tracks drive it deterministically: no thread, and virtual time
/// ([`VirtualClock`]): release `k` (from 1) starts at `clock.base() + (k - 1) * dt` plus what
/// a test advanced.
pub struct ReleaseFixture<O: CoreOps> {
    pub core: O,
    intake: RtIntake<Primitive>,
    targets: TargetReader,
    stop: StopRequests,
    active: Option<Installed>,
    releases: u64,
    clock: VirtualClock,
    hook: Option<Box<dyn FnMut(Checkpoint)>>,
    pub dt: f64,
}

/// The points of a release where the fixture re-reads the stop generation and the time
/// (and applies a stop that arrived since): a stop or a deadline can never be overtaken by
/// the command, target or output that follows it in the same release.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Checkpoint {
    /// Before the ordinary command commits.
    BeforeCommit,
    /// Before the latest target is consumed.
    BeforeTarget,
    /// Before the control tick computes and sends its output.
    BeforeOutput,
}

impl<O: CoreOps> ReleaseFixture<O> {
    /// The fixture, the broker feeding it, the target writer and the stop handle.
    pub fn new(core: O, boot: BootId, caps: Capacities) -> (Self, Broker<Primitive>, TargetWriter, StopRequests) {
        let (broker, intake) = super::broker::channel(ArmId::LOCAL, boot, caps, Primitive::default);
        let (tw, tr) = target_mailbox();
        let stop = StopRequests::new();
        let clock = VirtualClock::starting_at(Instant::now());
        (Self { core, intake, targets: tr, stop: stop.clone(), active: None, releases: 0, clock, hook: None, dt: 0.02 }, broker, tw, stop)
    }

    /// The fixture's virtual clock (clone it to advance time from a test).
    pub fn clock(&self) -> &VirtualClock { &self.clock }
    /// Virtual now.
    pub fn now(&self) -> Instant { self.clock.now() }

    pub fn releases(&self) -> u64 { self.releases }
    pub fn active(&self) -> Option<ActionId> { self.active.as_ref().map(Installed::action) }
    pub fn reply_overflow(&self) -> u64 { self.intake.reply_overflow() }

    pub fn ctx(&self, now: Instant) -> CommitCtx {
        CommitCtx { arm: self.intake.arm(), boot: self.intake.boot(), now, stop_generation: self.stop.current() }
    }

    /// Runs `hook` at every [`Checkpoint`] just before the fixture samples the stop
    /// generation and the time there (tests: a stop or a clock step arriving mid-release).
    pub fn on_checkpoint(&mut self, hook: impl FnMut(Checkpoint) + 'static) { self.hook = Some(Box::new(hook)); }

    /// Applies a stop requested since the last one applied; true when it did.
    fn apply_pending_stop(&mut self) -> bool {
        let generation = self.stop.current();
        if generation == self.stop.applied() { return false; }
        self.core.apply_stop();
        self.stop.mark_applied(generation);
        true
    }

    /// `at`: re-reads the stop generation (applying a new stop first) and the time.
    fn checkpoint(&mut self, at: Checkpoint, report: &mut ReleaseReport) -> CommitCtx {
        if let Some(hook) = self.hook.as_mut() { hook(at); }
        if self.apply_pending_stop() { report.stopped = true; }
        self.ctx(self.clock.now())
    }

    /// One release.
    pub fn step(&mut self) -> ReleaseReport {
        self.releases += 1;
        let mut report = ReleaseReport { release: self.releases, ..ReleaseReport::default() };
        let started = self.clock.now();
        let release_deadline = started + Duration::from_secs_f64(self.dt);
        // 1. urgent stop, independent of the command ring's capacity
        if self.apply_pending_stop() { report.stopped = true; }
        // 2. the bus owner, within the release's service budget
        report.poll = self.core.poll_hardware(&ServiceBudget::for_release(self.clock.now(), release_deadline));
        // 3. one ordinary command (the intake enforces one per release)
        self.intake.begin_release(self.releases);
        let ctx = self.checkpoint(Checkpoint::BeforeCommit, &mut report);
        if let Some(mut t) = self.intake.take() {
            let d = *t.descriptor();
            let result = match d.kind {
                CommandKind::Install { .. } => match t.plan().copied() {
                    None => Err("PLAN_RETIRED"),
                    Some(p) => self.core.commit_primitive(&d, &p, &ctx),
                },
                CommandKind::Immediate(op) => self.core.commit_immediate(&d, op, &ctx),
            };
            report.command = Some((d.stamp.request, match result {
                Ok(c) => {
                    if d.kind.starts_action() && c.action().is_some() && c.request() == d.stamp.request {
                        if let Some(old) = self.active.take() { t.terminal_before(old, Outcome::Superseded); }
                    }
                    t.ack(c).map(|acked| {
                        if let Some(i) = acked.installed { self.active = Some(i); }
                        acked.commit_seq
                    })
                }
                Err(code) => {
                    t.refuse(code);
                    Err(code)
                }
            }));
        }
        // 4. the latest target
        if let Some(f) = self.targets.take() {
            let ctx = self.checkpoint(Checkpoint::BeforeTarget, &mut report);
            report.target = Some((f.seq, self.core.consume_target(&f, &ctx)));
        }
        // 5. the control tick (its output gate also holds for a stop arriving during it:
        // `EstopGate::with_stops`)
        self.checkpoint(Checkpoint::BeforeOutput, &mut report);
        self.core.advance(self.dt, self.releases);
        // 6. terminal of the active action
        if let Some(action) = self.active() {
            if let Some(outcome) = self.core.action_outcome(action) {
                let installed = self.active.take().expect("active above");
                self.intake.terminal(installed, outcome);
                report.ended = Some((action, outcome));
            }
        }
        // the next release starts one period later
        self.clock.advance(release_deadline.saturating_duration_since(self.clock.now()));
        report
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_oversized_or_mismatched_primitive_is_refused_not_truncated() {
        let q9 = [0.1f32; MAX_DOF + 1];
        assert_eq!(Primitive::joint_quintic(&q9, &q9, 1.0), Err("INVALID_COMMAND"));
        assert_eq!(Primitive::joint_quintic(&[0.0; 3], &[0.1; 2], 1.0), Err("INVALID_COMMAND"));
        assert_eq!(Primitive::joint_quintic(&[], &[], 1.0), Err("INVALID_COMMAND"));
        let Ok(Primitive::JointQuintic { dof, q, .. }) = Primitive::joint_quintic(&[0.0; MAX_DOF], &[0.2; MAX_DOF], 1.0) else { panic!() };
        assert_eq!((dof, q), (MAX_DOF as u8, [0.2; MAX_DOF]));
    }

    #[test]
    fn terminal_outcomes_keep_their_cause() {
        assert_eq!(outcome_of("running", None), None);
        assert_eq!(outcome_of("done", None), Some(Outcome::Completed));
        assert_eq!(outcome_of("cancelled", Some("STOPPED")), Some(Outcome::Stopped));
        assert_eq!(outcome_of("cancelled", None), Some(Outcome::Stopped));
        assert_eq!(outcome_of("cancelled", Some("SUPERSEDED")), Some(Outcome::Superseded));
        assert_eq!(outcome_of("cancelled", Some("LEASE_LOST")), Some(Outcome::Cancelled("LEASE_LOST")));
        assert_eq!(outcome_of("cancelled", Some("COLLISION_HOLD")), Some(Outcome::Cancelled("COLLISION_HOLD")));
        assert_eq!(outcome_of("failed", Some("TRACKING_STALL")), Some(Outcome::Failed("TRACKING_STALL")));
        assert_eq!(outcome_of("failed", Some("FOLLOWING_ERROR")), Some(Outcome::Failed("FOLLOWING_ERROR")));
        assert_eq!(outcome_of("failed", Some("SOMETHING_NEW")), Some(Outcome::Failed("FAILED")));
        assert_eq!(outcome_of("failed", None), Some(Outcome::Failed("FAILED")));
        // every fault code is listed
        for c in crate::safety::FaultCode::ALL {
            let name = serde_json::to_value(c).unwrap();
            let name = name.as_str().unwrap();
            if name != "OTHER" { assert!(TERMINAL_CODES.contains(&name), "{name}"); }
        }
    }
}
