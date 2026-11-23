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
//! | [`ConfigAuthority`] (`reserve` / `persisted` / `activate` / `release` / reconcile) | versioned configuration (track B) |
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
    TransitionOp,
};
use super::config_iface::{Activated, ConfigAuthority, ConfigDomain, ConfigProposal, Durability, Released, Reservation, Reserved};
use super::ids::ArmId;
use super::mailbox::{target_mailbox, Refusal, TargetFrame, TargetOutcome, TargetReader, TargetWriter, MAX_DOF};
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
// a plan slot holds the largest variant inline and is pre-touched: no boxing, by design
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum Primitive {
    #[default]
    None,
    /// Joint-space quintic from `start[..dof]` to `q[..dof]` in `duration_s`
    /// (`SupervisorInner::jog_action`). `start` is the commanded position the plan was
    /// prepared from, every joint including the gripper: the commit refuses it (`PLAN_STALE`)
    /// when any commanded joint differs from it by `>= START_TOLERANCE`.
    JointQuintic { dof: u8, start: [f32; MAX_DOF], q: [f32; MAX_DOF], duration_s: f64 },
    /// Macro children (track E; the foundation adapter refuses them with `UNSUPPORTED`
    /// until C/E wire them). Each names its parent ([`ChildSpec`]); its deadline is the
    /// descriptor's.
    ///
    /// Straight-line Cartesian path through `waypoints[..n]` (base frame `[x, y, z, roll,
    /// pitch, yaw]`, m / rad) at `speed_mps`.
    CartesianPath { child: Option<ChildSpec>, dof: u8, start: [f32; MAX_DOF], n: u8, waypoints: [[f32; 6]; MAX_WAYPOINTS], speed_mps: f32 },
    /// Gripper to `fraction` (0 closed .. 1 open) at `speed`; `grasp`: stop on contact.
    Gripper { child: Option<ChildSpec>, dof: u8, start: [f32; MAX_DOF], fraction: f32, speed: f32, grasp: bool },
    /// Joint quintic with per-joint velocity limits and a settle time after it.
    JointQuinticLimited { child: Option<ChildSpec>, dof: u8, start: [f32; MAX_DOF], q: [f32; MAX_DOF], duration_s: f64, max_velocity: [f32; MAX_DOF], settle_s: f64 },
    /// Replay of a recorded episode.
    Replay { child: Option<ChildSpec>, dof: u8, start: [f32; MAX_DOF], episode: EpisodeRef, speed: f32, gripper: bool },
    /// A joint-sample path the worker prepared into the slot's [`PreparedPath`] (Cartesian
    /// path: IK solved; replay: episode read and resampled), installed by
    /// [`CoreOps::commit_prepared`] without IK or file I/O. `start` is the commanded position
    /// it was prepared from (checked as for `JointQuintic`); `child` names a macro parent.
    Prepared { child: Option<ChildSpec>, dof: u8, start: [f32; MAX_DOF], kind: PathKind, duration_s: f64, power_scale: Option<f64> },
}

/// What a [`Primitive::Prepared`] path is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PathKind {
    /// Samples at path parameters `0..=1` (`PreparedPath::params`, first sample = start),
    /// run over `duration_s` with the quintic time law of a Cartesian path.
    Cartesian,
    /// Recorded samples every `sample_dt` s of recorded time (`duration_s` recorded), played
    /// at `speed` after an approach quintic from the commanded position to the first sample
    /// (planned at the commit: no IK). `gripper` false keeps the gripper's command.
    /// `tool_digest`: [`tool_digest_hash`] of the active tool the worker checked the
    /// episode's tool provenance against (`TOOL_MISMATCH` at the commit when it changed).
    Replay { episode: EpisodeRef, speed: f32, gripper: bool, sample_dt: f64, tool_digest: u64 },
}

/// FNV-1a hash of a tool configuration digest (no allocation): the prepared replay names the
/// tool its provenance was checked against.
pub fn tool_digest_hash(digest: &str) -> u64 {
    digest.bytes().fold(0xcbf2_9ce4_8422_2325u64, |h, b| (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3))
}

/// Joint samples one plan slot's [`PreparedPath`] holds (allocated when the slot is built;
/// a longer path or episode is refused `PLAN_TOO_LARGE` when it is prepared).
pub const MAX_PATH_SAMPLES: usize = 2048;

/// The joint samples of a [`Primitive::Prepared`], in a buffer allocated once (plan slot
/// construction) and refilled in place: preparing never reallocates it.
#[derive(Debug, Clone, PartialEq)]
pub struct PreparedPath {
    samples: Vec<[f32; MAX_DOF]>,
    params: Vec<f64>,
}

impl PreparedPath {
    pub fn with_capacity(n: usize) -> Self { Self { samples: Vec::with_capacity(n), params: Vec::with_capacity(n) } }
    pub fn capacity(&self) -> usize { self.samples.capacity().min(self.params.capacity()) }
    pub fn clear(&mut self) { self.samples.clear(); self.params.clear(); }
    pub fn len(&self) -> usize { self.samples.len() }
    pub fn is_empty(&self) -> bool { self.samples.is_empty() }
    pub fn samples(&self) -> &[[f32; MAX_DOF]] { &self.samples }
    pub fn params(&self) -> &[f64] { &self.params }

    /// Appends sample `q` at path parameter `s` (a replay passes its sample index).
    /// `PLAN_TOO_LARGE` when the buffer is full (it never grows), `INVALID_COMMAND` for more
    /// than [`MAX_DOF`] joints or a non-finite value.
    pub fn push(&mut self, q: &[f32], s: f64) -> Result<(), &'static str> {
        if q.len() > MAX_DOF || !s.is_finite() || q.iter().any(|v| !v.is_finite()) { return Err("INVALID_COMMAND"); }
        if self.samples.len() >= self.capacity() { return Err("PLAN_TOO_LARGE"); }
        let mut a = [0.0; MAX_DOF];
        a[..q.len()].copy_from_slice(q);
        self.samples.push(a);
        self.params.push(s);
        Ok(())
    }

    /// `other`'s samples into this buffer (`PLAN_TOO_LARGE` when they do not fit; nothing
    /// is reallocated).
    pub fn copy_from(&mut self, other: &PreparedPath) -> Result<(), &'static str> {
        if other.len() > self.capacity() { return Err("PLAN_TOO_LARGE"); }
        self.clear();
        self.samples.extend_from_slice(&other.samples);
        self.params.extend_from_slice(&other.params);
        Ok(())
    }
}

/// One plan slot: the primitive and the prepared path a [`Primitive::Prepared`] reads.
#[derive(Debug, Clone, PartialEq)]
pub struct PlanSlot {
    pub primitive: Primitive,
    pub path: PreparedPath,
}

impl Default for PlanSlot {
    /// A slot with a [`MAX_PATH_SAMPLES`] path buffer (pre-touched by the allocation).
    fn default() -> Self { Self::with_path_capacity(MAX_PATH_SAMPLES) }
}

impl PlanSlot {
    pub fn with_path_capacity(n: usize) -> Self { Self { primitive: Primitive::None, path: PreparedPath::with_capacity(n) } }

    /// Stages a primitive that reads no path (the path is cleared).
    pub fn set(&mut self, p: Primitive) {
        self.primitive = p;
        self.path.clear();
    }

    /// Stages a slot the worker prepared (`PLAN_TOO_LARGE`: nothing staged; the slot is empty).
    pub fn copy_from(&mut self, other: &PlanSlot) -> Result<(), &'static str> {
        self.primitive = Primitive::None;
        self.path.copy_from(&other.path)?;
        self.primitive = other.primitive;
        Ok(())
    }
}

/// Waypoints a [`Primitive::CartesianPath`] carries at most.
pub const MAX_WAYPOINTS: usize = 16;

/// A macro child's identity: its parent action and its sequence within that parent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChildTicket {
    pub parent: ActionId,
    pub child_seq: u32,
}

/// A child primitive's ticket and the guards it runs under.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChildSpec {
    pub ticket: ChildTicket,
    /// The child fails with `timeout_code` after `timeout_s` (e.g. `GRIPPER_TIMEOUT`).
    pub timeout_s: f64,
    pub timeout_code: &'static str,
    pub contact: Option<ContactGuard>,
}

/// Stop the child when a joint's residual torque exceeds its bound (N·m; 0: unguarded).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContactGuard {
    pub max_residual_nm: [f32; MAX_DOF],
    /// The child's terminal code on contact (e.g. `COLLISION`, or `Completed` for a descent
    /// that is meant to touch: `None`).
    pub stop_code: Option<&'static str>,
}

/// An episode id, inline (ULID: 26 bytes; up to 32).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EpisodeRef {
    len: u8,
    id: [u8; 32],
}

impl EpisodeRef {
    /// None when `id` is longer than 32 bytes.
    pub fn new(id: &str) -> Option<Self> {
        if id.len() > 32 { return None; }
        let mut b = [0u8; 32];
        b[..id.len()].copy_from_slice(id.as_bytes());
        Some(Self { len: id.len() as u8, id: b })
    }
    pub fn as_str(&self) -> &str { std::str::from_utf8(&self.id[..usize::from(self.len)]).unwrap_or_default() }
}

/// A prepared start differs from the commanded position when a joint (gripper included)
/// moved by at least this much (rad): the threshold of `SupervisorInner::check_plan_fresh`
/// ([`crate::motion::PLAN_START_TOL_RAD`]).
pub const START_TOLERANCE: f32 = crate::motion::PLAN_START_TOL_RAD as f32;

impl Primitive {
    /// The child ticket of a macro child (None: a plain primitive).
    pub fn child(&self) -> Option<&ChildSpec> {
        match self {
            Self::None | Self::JointQuintic { .. } => None,
            Self::CartesianPath { child, .. } | Self::Gripper { child, .. } | Self::JointQuinticLimited { child, .. } | Self::Replay { child, .. }
            | Self::Prepared { child, .. } => child.as_ref(),
        }
    }

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
    "FEEDBACK_UNCONFIRMED", "FEEDBACK_STALE", "INVALID_FEEDBACK", "UNLOAD_FAILED",
    // motion and action failures
    "IK_UNREACHABLE", "IK_BRANCH_FLIP", "WORKSPACE_VIOLATION", "INVALID_COMMAND", "INVALID_TARGET", "JUMP_LIMIT", "STEP_LIMIT",
    "SERVO_INFEASIBLE", "SERVO_TARGET_DISCARDED", "STALE_INPUT", "MOTION_TIMEOUT", "GRIPPER_TIMEOUT", "GRASP_TIMEOUT",
    "PLACE_TIMEOUT", "TARGET_TIMEOUT", "SEQUENCE_TIMEOUT", "MAX_SECONDS", "NOT_HOLDING", "TARGET_LOST", "TARGET_DISPLACED",
    "DROPPED", "OBJECT_HELD", "OBJECT_TOO_TALL", "TOO_WIDE", "CENTRING_FAILED", "DETECTION_UNAVAILABLE", "TASK_UNSUPPORTED",
    "OBSERVATION_UNAVAILABLE", "PERSIST_FAILED", "POLICY_NOT_ACTIVE", "POWER_LIMIT",
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
    /// Per-joint receive-queue bounds of the latest feedback (multiview receive progress);
    /// empty when the driver reports none.
    pub feedback_queue_bounds: Vec<Option<Instant>>,
    /// Estimated residual joint torques (N·m, contact), None without an estimate.
    pub residual_torques: Option<[f32; 6]>,
    pub motion_active: bool,
    /// A configuration hold inhibits motion (`ArmCore::motion_inhibited_by_config`): refuse
    /// motion at admission; the commit refuses it with `CONFIG_BUSY`.
    pub config_inhibited: bool,
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
///
/// A receive pass the budget cuts leaves the unread replies queued and the joints' feedback
/// older. With the MIT deadline fence armed (gravity compensation, MIT tracking), feedback
/// that ages past the fence (45 ms) trips the whole group and discards cached output: this
/// is deliberate (stale feedback never drives MIT output), so the budget has to drain a
/// release's traffic. Expected per release (inference, not measured): one reply per command
/// frame and motor, plus status polls, i.e. well under 64 frames for 7 motors at 50 Hz. The
/// sizing is checked in the device qualification (worst RX age at load), not here.
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

/// Progress of the driver transition in flight (`HardwarePoll::torque`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TorqueProgress {
    /// Still waiting for confirmation: service it again at `wake` (the executor wakes then,
    /// capped at the next release).
    Pending { wake: Instant },
    /// Confirmed: the transition's terminal is `Completed`.
    Done,
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
    /// The driver transition in flight (None: none): progress, or its failure code (the
    /// transition's terminal is `Failed(code)`).
    pub torque: Option<Result<TorqueProgress, &'static str>>,
}

impl HardwarePoll {
    /// When the bus owner wants to be serviced before the next release (a pending transition).
    pub fn wake(&self) -> Option<Instant> {
        match self.torque { Some(Ok(TorqueProgress::Pending { wake })) => Some(wake), _ => None }
    }

    /// The executor's extra service within the release ending at `release`: the pending
    /// transition's wake when it comes before the release (None: serve it at the release).
    pub fn wake_before(&self, release: Instant) -> Option<Instant> { self.wake().filter(|w| *w < release) }
}

/// Extra hardware services the executor runs within one release at most (a transition's
/// retry cadence is 25 ms and its confirmation service 2 ms: a 20 ms release needs a few).
pub const MAX_SERVICE_WAKES: u32 = 8;

impl HardwarePoll {
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
    /// Installs a [`Primitive::Prepared`] with its slot's `path` after rechecking its stamp
    /// (as `commit_primitive`); no IK and no file I/O. Default: `UNSUPPORTED`.
    fn commit_prepared(&mut self, _d: &Descriptor, _primitive: &Primitive, _path: &PreparedPath, _ctx: &CommitCtx) -> Result<Committed, &'static str> {
        Err("UNSUPPORTED")
    }
    /// Starts a driver transition after rechecking the stamp; its progress and outcome come
    /// from [`Self::poll_hardware`] (`HardwarePoll::torque`). The committed action id names
    /// the transition (the adapter uses the request id's value).
    fn commit_transition(&mut self, d: &Descriptor, op: TransitionOp, ctx: &CommitCtx) -> Result<Committed, &'static str>;
    /// Applies one target after rechecking it ([`TargetFrame::check`] against the core's
    /// [`super::mailbox::TargetAuthority`], then the configuration hold).
    ///
    /// No allocation: the frame is `Copy` with fixed-size arrays; an implementation applies
    /// `frame.positions()` (a slice of the frame) into the core's preallocated target state.
    /// (The foundation adapter still delegates to `servo(Vec)`, which allocates: track D.)
    fn consume_target(&mut self, frame: &TargetFrame, ctx: &CommitCtx) -> Result<(), Refusal>;
    /// Applies any pending urgent E-stop request; returns true when an E-stop was applied.
    fn apply_estop(&mut self) -> bool { false }
    /// An authorized stop was requested: stop active motion.
    fn apply_stop(&mut self);
    /// How `action` ended, None while it runs.
    fn action_outcome(&self, action: ActionId) -> Option<Outcome>;
    /// Child primitives the ended macro `action` ran (its terminal record,
    /// `ArmCore::last_macro`); 0 for a plain primitive.
    fn action_children(&self, _action: ActionId) -> u32 { 0 }
    /// The bus owner's progress and confirmations within `budget` (exclusive: the bus
    /// owner's state machine advances here).
    fn poll_hardware(&mut self, budget: &ServiceBudget) -> HardwarePoll;
    /// Sub-release wake for a pending driver transition only (without running a full release
    /// feedback pass). Default delegates to [`Self::poll_hardware`].
    fn poll_transition_wake(&mut self, budget: &ServiceBudget) -> HardwarePoll {
        self.poll_hardware(budget)
    }
    /// One release of the control tick (`generation`: the publication's lock generation).
    fn advance(&mut self, dt: f64, generation: u64);
    /// Advances control output for `dt` seconds and returns the observation timestamp for
    /// [`Self::publish`]. Default delegates to [`Self::advance`].
    fn advance_output(&mut self, dt: f64, generation: u64) -> f64 {
        self.advance(dt, generation);
        0.0
    }
    /// Publishes the state computed by [`Self::advance_output`] at `timestamp` with `generation`.
    fn publish(&mut self, _timestamp: f64, _generation: u64) {}
    /// Current local monotonic observation timestamp (seconds), used when publishing after
    /// a sub-release transition wake before delivering its terminal.
    fn now_timestamp(&self) -> f64 { 0.0 }
}

/// The protocol code of a delegated error, as a static string (unknown codes: `REJECTED`).
fn static_code(e: ArmError) -> &'static str {
    const KNOWN: &[&str] = &[
        "MOTION_BUSY", "NOT_ARMED", "ESTOP_ACTIVE", "FAULT_ACTIVE", "EPOCH_MISMATCH", "INVALID_COMMAND", "IK_UNREACHABLE",
        "WORKSPACE_VIOLATION", "JOINT_LIMIT", "PLAN_STALE", "LEASE_LOST", "NO_LEASE", "COLLISION_HOLD", "COMMISSIONING_REQUIRED",
        "INVALID_VALUE", "UNSUPPORTED", "INTERNAL", "FEEDBACK_STALE", "CONFIG_BUSY", "ARM_NOT_AT_RESTING_POSE", "TOOL_MISMATCH",
        "MOTION_FORBIDDEN_IN_FAULT_STATE",
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

impl SupervisorInner {
    /// The checks every motion commit shares: identity, deadline, stop, epoch, a lease valid
    /// now, configuration revision (`check_commit`); the active revision, reservations and
    /// feedback (track C); a child's parent is the one its stamp was admitted under.
    fn check_motion_commit(&self, d: &Descriptor, primitive: &Primitive, ctx: &CommitCtx) -> Result<(), &'static str> {
        self.check_commit(d, ctx)?;
        if let Some(code) = self.motion_commit_blocker(&d.stamp.authority) { return Err(code); }
        if let Some(c) = primitive.child() {
            if c.ticket.parent.0 != d.stamp.authority.parent_generation || c.ticket.parent.0 == 0 { return Err("PARENT_REPLACED"); }
        }
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
            feedback_queue_bounds: self.joint_feedback_queue_bounds.clone(),
            residual_torques: self.residual_torques,
            motion_active: self.active_trajectory.is_some(),
            config_inhibited: self.core.motion_inhibited_by_config(),
            taken_at: ctx.now,
        }
    }

    fn commit_primitive(&mut self, d: &Descriptor, primitive: &Primitive, ctx: &CommitCtx) -> Result<Committed, &'static str> {
        self.check_motion_commit(d, primitive, ctx)?;
        match primitive {
            // a prepared path is installed with its slot's samples (`commit_prepared`)
            Primitive::None | Primitive::Prepared { .. } => Err("INVALID_COMMAND"),
            // macro children (track E): installed for the running macro named by the ticket;
            // the committed action is the parent (its terminal carries the child count)
            Primitive::CartesianPath { .. } | Primitive::Gripper { .. } | Primitive::JointQuinticLimited { .. } | Primitive::Replay { .. } => {
                if primitive.child().is_none() { return Err("UNSUPPORTED"); }
                self.commit_macro_child(primitive, ctx.now).map(|parent| Committed::authorized(d, Some(parent)))
            }
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

    fn commit_prepared(&mut self, d: &Descriptor, primitive: &Primitive, path: &PreparedPath, ctx: &CommitCtx) -> Result<Committed, &'static str> {
        self.check_motion_commit(d, primitive, ctx)?;
        let Primitive::Prepared { child, dof, start, kind, duration_s, power_scale } = *primitive else { return Err("INVALID_COMMAND"); };
        let n = usize::from(dof);
        if n == 0 || n > MAX_DOF { return Err("INVALID_COMMAND"); }
        let epoch = d.stamp.authority.epoch;
        let committed = |r: Result<u64, ArmError>| r.map(|id| Committed::authorized(d, Some(ActionId(id)))).map_err(static_code);
        match kind {
            PathKind::Cartesian => {
                if !start_matches(&start[..n], &self.commanded_positions) { return Err("PLAN_STALE"); }
                let joint_path = self.prepared_joint_path(n, path, duration_s, power_scale)?;
                match child {
                    None => committed(self.install_prepared_path(epoch, joint_path, power_scale.is_some()).map(|(id, _)| id)),
                    Some(spec) => self.commit_child_install(spec, &start[..n], ctx.now, move |this, id| {
                        this.install_macro_prepared_path(id, joint_path, power_scale.is_some()).map(drop)
                    }).map(|parent| Committed::authorized(d, Some(parent))),
                }
            }
            PathKind::Replay { .. } => {
                if !start_matches(&start[..n], &self.commanded_positions) { return Err("PLAN_STALE"); }
                match child {
                    None => {
                        let prepared = self.prepared_replay_motion(n, path, kind, duration_s, power_scale, epoch).map_err(static_code)?;
                        committed(self.install_prepared_replay(prepared).map(|(id, _)| id))
                    }
                    Some(spec) => self.commit_child_install(spec, &start[..n], ctx.now, move |this, id| {
                        let prepared = this.prepared_replay_motion(n, path, kind, duration_s, power_scale, epoch)?;
                        this.install_macro_prepared_replay(id, prepared).map(drop)
                    }).map(|parent| Committed::authorized(d, Some(parent))),
                }
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
            ImmediateOp::TorqueOn => {
                if self.driver.is_some() {
                    return Err("UNSUPPORTED");
                }
                self.set_torque(true).map(drop).map_err(ArmError::Safety)
            }
            ImmediateOp::TorqueOff => {
                if self.driver.is_some() {
                    return Err("UNSUPPORTED");
                }
                self.set_torque(false).map(drop).map_err(ArmError::Safety)
            }
        };
        r.map(|_| Committed::authorized(d, None)).map_err(static_code)
    }

    fn commit_transition(&mut self, d: &Descriptor, op: TransitionOp, ctx: &CommitCtx) -> Result<Committed, &'static str> {
        self.check_commit(d, ctx)?;
        if self.core.transition_pending() || self.torque_transition_pending() { return Err("TRANSITION_BUSY"); }
        // starts the bus owner's handshake (ArmDriver::begin_torque); a refusal before any
        // frame refuses the commit, a later failure is the transition's terminal
        let progress = self.begin_torque_transition(matches!(op, TransitionOp::TorqueOn)).map_err(|e| static_code(ArmError::Safety(e)))?;
        // the next service is the next poll (the begin's wake is on the driver's clock)
        self.core.begin_transition(progress.map(|p| match p { TorqueProgress::Pending { .. } => TorqueProgress::Pending { wake: ctx.now }, done => done }));
        Ok(Committed::authorized(d, Some(ActionId(d.stamp.request.0))))
    }

    fn consume_target(&mut self, frame: &TargetFrame, ctx: &CommitCtx) -> Result<(), Refusal> {
        // track D: identity, deadline, authority, stream, policy generation, input / camera
        // ages, config hold, then the servo / policy step checks (supervisor::streams::targets)
        self.consume_target_frame(frame, ctx)
    }

    fn apply_estop(&mut self) -> bool { self.consume_pending_estop() }

    // ends client motion; a running watchdog park belongs to the lease watchdog and continues
    fn apply_stop(&mut self) { self.authorized_stop(); }

    fn action_outcome(&self, action: ActionId) -> Option<Outcome> {
        // a macro's terminal record outlives its action record (finish_action, a newer action)
        if let Some(m) = self.core.last_macro().filter(|m| m.action == action) { return Some(m.outcome); }
        let Some(a) = self.last_action.as_ref().filter(|a| a.id == action.0) else { return Some(Outcome::Superseded); };
        outcome_of(&a.state, a.code.as_deref())
    }

    fn action_children(&self, action: ActionId) -> u32 { self.core.last_macro().filter(|m| m.action == action).map_or(0, |m| m.children) }

    fn poll_hardware(&mut self, budget: &ServiceBudget) -> HardwarePoll {
        // One budgeted bus service per release: first serve any torque transition in flight,
        // then apply feedback from the transition's RX cache (when this call polled the bus)
        // or run a single budgeted receive pass.
        let trans = self.service_torque_transition(budget);
        let transition_rx = if let Some(tp) = trans {
            self.core.begin_transition(tp.progress);
            tp.rx
        } else {
            self.service_background_disable(budget)
        };
        let rx = self.poll_hardware_rx(Some(budget), transition_rx);
        let rx_frames = rx.rx_frames;
        let rx_exhausted = rx.budget_exhausted;
        self.pending_rx_poll = rx;
        // If a background disable (non-broker, e.g. failed enable or thermal trip) was pending
        // and Step 2's fresh feedback just confirmed every motor disabled, clear `disable_pending`
        // immediately so Step 2b maintenance (`ClearFault`/`RequestShutdown`) and Step 3 commits
        // observe the confirmed torque-off on this release rather than waiting until Step 5.
        if self.disable_pending && self.torque_transition.is_none() {
            let confirmed = self.driver.as_ref().is_none_or(|d| {
                d.lock().is_ok_and(|drv| drv.is_feedback_valid() && drv.enable_state() == crate::driver::EnableState::Disabled)
            });
            if confirmed {
                self.is_torqued = false;
                self.pending_rearm = false;
                self.disable_pending = false;
            }
        }
        let budget_exhausted = budget.now > budget.deadline || rx_exhausted;
        let Some(driver) = self.driver.as_ref() else {
            return HardwarePoll { driverless: true, rx_frames, budget_exhausted, torque: self.core.poll_transition(), ..HardwarePoll::default() };
        };
        let d = driver.lock().unwrap_or_else(|e| e.into_inner());
        d.feedback_ages_into(&mut self.scratch.ages);
        let oldest = self.scratch.ages.iter().copied().try_fold(Duration::ZERO, |m, a| a.map(|a| m.max(a)));
        HardwarePoll {
            driverless: false,
            feedback_valid: d.is_feedback_valid(),
            enable_state: Some(d.enable_state()),
            oldest_feedback_age: oldest,
            rx_frames,
            budget_exhausted,
            torque: self.core.poll_transition(),
        }
    }

    fn poll_transition_wake(&mut self, budget: &ServiceBudget) -> HardwarePoll {
        let trans = self.service_torque_transition(budget);
        let (rx_frames, trans_exhausted) = match trans {
            Some(tp) => {
                self.core.begin_transition(tp.progress);
                tp.rx.unwrap_or((0, false))
            }
            None => (0, false),
        };
        let budget_exhausted = budget.now > budget.deadline || trans_exhausted;
        let Some(driver) = self.driver.as_ref() else {
            return HardwarePoll { driverless: true, rx_frames, budget_exhausted, torque: self.core.poll_transition(), ..HardwarePoll::default() };
        };
        let d = driver.lock().unwrap_or_else(|e| e.into_inner());
        d.feedback_ages_into(&mut self.scratch.ages);
        let oldest = self.scratch.ages.iter().copied().try_fold(Duration::ZERO, |m, a| a.map(|a| m.max(a)));
        HardwarePoll {
            driverless: false,
            feedback_valid: d.is_feedback_valid(),
            enable_state: Some(d.enable_state()),
            oldest_feedback_age: oldest,
            rx_frames,
            budget_exhausted,
            torque: self.core.poll_transition(),
        }
    }

    fn advance(&mut self, dt: f64, generation: u64) {
        drop(self.tick_publish(dt, generation, false));
    }

    fn advance_output(&mut self, dt: f64, _generation: u64) -> f64 {
        self.tick_control(dt)
    }

    fn publish(&mut self, timestamp: f64, generation: u64) {
        drop(self.publish_at(timestamp, generation, false));
    }

    fn now_timestamp(&self) -> f64 {
        self.now_s()
    }
}

/// The supervisor's configuration revisions; holds live in the core.
///
/// Protocol at the existing commit boundary (one supervisor write guard, track B): the worker
/// keeps the candidate (its payload, keyed by the reservation's `CandidateId`) and stages its
/// file off the lock; under the lock it rechecks authorization, lease and epoch, then
/// [`ConfigAuthority::reserve`]s the domain at the revision the candidate was built from,
/// renames the file into place and reports [`ConfigAuthority::persisted`], applies the
/// candidate (`SupervisorInner::activate_config_candidate`, which moves the domain to
/// `from + 1`) and [`ConfigAuthority::activate`]s the reservation, which confirms the new
/// revision and ends the hold. A failure before the rename releases the reservation; a
/// failure after it leaves the hold `Reconciling`.
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

    fn reserve(&mut self, p: &ConfigProposal, ctx: &CommitCtx) -> Reserved {
        let current = self.revision(p.domain);
        if let Some(code) = p.check(ctx.arm, ctx.boot, ctx.now, &self.authority(ctx)) { return Reserved::Refused { code, current }; }
        if p.expected_revision != current { return Reserved::Refused { code: "REVISION_MISMATCH", current }; }
        match self.core.config_mut().grant(p, current) {
            Some(r) => Reserved::Granted(r),
            None => Reserved::Refused { code: "CONFIG_BUSY", current },
        }
    }

    fn persisted(&mut self, r: &Reservation, durability: Durability) -> Result<(), &'static str> { self.core.config_mut().persisted(r, durability) }

    /// Confirms a reservation whose candidate was applied: the domain must be at `from + 1`.
    /// The durability is the one reported by [`ConfigAuthority::persisted`] (`Volatile`
    /// without it). Refusals: `NOT_HELD` (another grant or none), `RECONCILING`,
    /// `NOT_APPLIED` (still at `from`), `REVISION_MISMATCH` (moved elsewhere); before
    /// persistence the reservation comes back, after it the hold is `Reconciling`.
    fn activate(&mut self, r: Reservation) -> Activated {
        let Some(held) = self.core.config_held(r.domain()).filter(|h| h.grant == Some(r.grant())) else {
            return Activated::Refused { code: "NOT_HELD", reservation: r };
        };
        let durability = match held.state {
            super::config_iface::HoldState::Reserved => Durability::Volatile,
            super::config_iface::HoldState::PersistedAwaitingActivation { durability } => durability,
            super::config_iface::HoldState::Reconciling => return self.core.config_mut().activation_failed(r, "RECONCILING"),
        };
        let revision = self.revision(r.domain());
        if revision != r.from() + 1 {
            let code = if revision == r.from() { "NOT_APPLIED" } else { "REVISION_MISMATCH" };
            return self.core.config_mut().activation_failed(r, code);
        }
        self.core.config_mut().activated(&r);
        Activated::Active { revision, durability }
    }

    fn release(&mut self, r: Reservation) -> Released { self.core.config_mut().release(r) }

    fn begin_reconcile(&mut self, domain: ConfigDomain) -> bool {
        let active = self.revision(domain);
        self.core.config_mut().begin_reconcile(domain, active)
    }

    fn finish_reconcile(&mut self, domain: ConfigDomain) -> bool { self.core.config_mut().finish_reconcile(domain) }
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
            feedback_instants: Vec::new(), feedback_seqs: Vec::new(), feedback_queue_bounds: Vec::new(), residual_torques: None, motion_active: false, config_inhibited: false, taken_at: ctx.now }
    }
    fn commit_primitive(&mut self, _: &Descriptor, _: &Primitive, _: &CommitCtx) -> Result<Committed, &'static str> { self.commits_seen += 1; Err("NOT_WIRED") }
    fn commit_immediate(&mut self, _: &Descriptor, _: ImmediateOp, _: &CommitCtx) -> Result<Committed, &'static str> { self.commits_seen += 1; Err("NOT_WIRED") }
    fn commit_transition(&mut self, _: &Descriptor, _: TransitionOp, _: &CommitCtx) -> Result<Committed, &'static str> { self.commits_seen += 1; Err("NOT_WIRED") }
    fn consume_target(&mut self, _: &TargetFrame, _: &CommitCtx) -> Result<(), Refusal> { self.targets_seen += 1; Err(Refusal::code("NOT_WIRED")) }
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
    /// The full refusal of that target (also reported to the stream's writer).
    pub target_refusal: Option<Refusal>,
    /// The active action that ended in this release.
    pub ended: Option<(ActionId, Outcome)>,
    /// The hardware poll of this release.
    pub poll: HardwarePoll,
    /// The driver transition that ended in this release.
    pub transition_ended: Option<(ActionId, Outcome)>,
    /// Extra hardware services at a pending transition's wake within this release
    /// (`HardwarePoll::wake_before`, at most [`MAX_SERVICE_WAKES`]); `poll` is the last one.
    pub wakes: u32,
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

/// Synchronous single-arm release engine shared by [`ReleaseFixture`] (virtual time)
/// and the production RT executor (real monotonic time).
///
/// Exact per-release order:
/// 1. Urgent requests (E-stop, then authorized stop).
/// 2. Single budgeted hardware poll (completing any pending transition, deferring terminal delivery until after Step 6 publish).
/// 3. Urgent recheck (`BeforeCommit`), then commit at most one ordinary descriptor.
/// 4. Urgent recheck (`BeforeTarget`), then consume at most one latest target.
/// 5. Urgent recheck (`BeforeOutput`), then advance control output (`dt`).
/// 6. Publish observation and RT inputs.
/// 7. Deliver terminals (completed transition terminal, then active-action terminal).
pub struct ReleaseEngine {
    intake: RtIntake<PlanSlot>,
    targets: TargetReader,
    stop: StopRequests,
    active: Option<Installed>,
    /// The driver transition in flight.
    transition: Option<Installed>,
    releases: u64,
    hook: Option<Box<dyn FnMut(Checkpoint) + Send>>,
}

impl ReleaseEngine {
    /// Creates an engine with its broker, target writer, and authorized-stop handle.
    pub fn new(boot: BootId, caps: Capacities) -> (Self, Broker<PlanSlot>, TargetWriter, StopRequests) {
        let (broker, intake) = super::broker::channel(ArmId::LOCAL, boot, caps, PlanSlot::default);
        let (tw, tr) = target_mailbox();
        let stop = StopRequests::new();
        (
            Self {
                intake,
                targets: tr,
                stop: stop.clone(),
                active: None,
                transition: None,
                releases: 0,
                hook: None,
            },
            broker,
            tw,
            stop,
        )
    }

    pub fn releases(&self) -> u64 { self.releases }
    pub fn active(&self) -> Option<ActionId> { self.active.as_ref().map(Installed::action) }
    pub fn transition(&self) -> Option<ActionId> { self.transition.as_ref().map(Installed::action) }
    pub fn reply_overflow(&self) -> u64 { self.intake.reply_overflow() }
    pub fn stop_requests(&self) -> &StopRequests { &self.stop }

    pub fn ctx(&self, now: Instant) -> CommitCtx {
        CommitCtx { arm: self.intake.arm(), boot: self.intake.boot(), now, stop_generation: self.stop.current() }
    }

    pub fn on_checkpoint(&mut self, hook: impl FnMut(Checkpoint) + Send + 'static) { self.hook = Some(Box::new(hook)); }

    /// Applies any pending urgent E-stop and/or authorized-stop requests; true when either applied.
    pub fn apply_pending_urgent<O: CoreOps>(&mut self, core: &mut O) -> bool {
        let mut applied = false;
        if core.apply_estop() {
            applied = true;
        }
        let generation = self.stop.current();
        if generation != self.stop.applied() {
            core.apply_stop();
            self.stop.mark_applied(generation);
            applied = true;
        }
        applied
    }

    fn checkpoint<O: CoreOps>(
        &mut self,
        core: &mut O,
        at: Checkpoint,
        now_fn: &mut impl FnMut() -> Instant,
        report: &mut ReleaseReport,
    ) -> CommitCtx {
        if let Some(hook) = self.hook.as_mut() { hook(at); }
        if self.apply_pending_urgent(core) { report.stopped = true; }
        self.ctx(now_fn())
    }

    fn take_finished_transition(&mut self, report: &mut ReleaseReport) -> Option<(Installed, Outcome)> {
        let ended = match report.poll.torque {
            Some(Ok(TorqueProgress::Done)) => Some(Outcome::Completed),
            Some(Err(code)) => Some(Outcome::Failed(code)),
            _ => None,
        };
        if let Some(outcome) = ended {
            if let Some(t) = self.transition.take() {
                report.transition_ended = Some((t.action(), outcome));
                return Some((t, outcome));
            }
        }
        None
    }

    /// Runs one full release on `core`.
    pub fn step<O: CoreOps>(
        &mut self,
        core: &mut O,
        now_fn: impl FnMut() -> Instant,
        dt: f64,
        release_deadline: Instant,
        publish_generation: u64,
    ) -> ReleaseReport {
        self.step_with_pre_commit(core, now_fn, dt, release_deadline, publish_generation, |_, _| false)
    }

    /// Runs one full release on `core`, invoking `pre_commit(core, ctx)` strictly AFTER Step 2's
    /// hardware poll and strictly BEFORE Step 3 (`BeforeCommit`), Step 4 (`BeforeTarget`), Step 5
    /// (`BeforeOutput` / `advance_output`), Step 6 (`publish`), and Step 7 (`terminals`).
    ///
    /// When `pre_commit` returns `true` (an accepted `CommitShutdown`), Steps 3–5 are skipped so
    /// no ordinary command, target, or actuator output runs before the owner thread stops.
    pub fn step_with_pre_commit<O: CoreOps>(
        &mut self,
        core: &mut O,
        mut now_fn: impl FnMut() -> Instant,
        dt: f64,
        release_deadline: Instant,
        publish_generation: u64,
        mut pre_commit: impl FnMut(&mut O, &CommitCtx) -> bool,
    ) -> ReleaseReport {
        self.releases += 1;
        let mut report = ReleaseReport { release: self.releases, ..ReleaseReport::default() };
        // 1. Urgent requests (E-stop + authorized stop), independent of command ring capacity
        if self.apply_pending_urgent(core) { report.stopped = true; }
        // 2. Single budgeted hardware poll (frees `self.transition`, defers terminal delivery until after Step 6 publish).
        // NOTE: `DamiaoArmDriver::receive_feedback` still has a legacy stale-joint RX-poll
        // recovery TX path (`src/driver.rs:4701`) outside Step 5's `BeforeOutput` checkpoint;
        // that path is addressed on branch `feat/rt-tx-budget`, which merges after stage 3 step 1.
        let budget = ServiceBudget::for_release(now_fn(), release_deadline);
        report.poll = core.poll_hardware(&budget);
        let finished_transition = self.take_finished_transition(&mut report);
        // 2b. Owner persistence and bounded maintenance run strictly after Step 2 feedback intake
        // and strictly before Step 3–6 commit, output, and publication.
        let pre_ctx = self.ctx(now_fn());
        let stopping = pre_commit(core, &pre_ctx);
        if stopping {
            if let Some((t, outcome)) = finished_transition {
                let ts = core.now_timestamp();
                core.publish(ts, publish_generation);
                self.intake.terminal(t, outcome);
            }
            return report;
        }
        // 3. Urgent recheck + at most one ordinary command
        self.intake.begin_release(self.releases);
        let ctx = self.checkpoint(core, Checkpoint::BeforeCommit, &mut now_fn, &mut report);
        let mut superseded_and_ack: Option<(Installed, super::broker::Reply)> = None;
        if let Some(t) = self.intake.take() {
            let d = *t.descriptor();
            let result = match d.kind {
                CommandKind::Install { .. } => match t.plan() {
                    None => Err("PLAN_RETIRED"),
                    Some(slot) if matches!(slot.primitive, Primitive::Prepared { .. }) => core.commit_prepared(&d, &slot.primitive, &slot.path, &ctx),
                    Some(slot) => core.commit_primitive(&d, &slot.primitive, &ctx),
                },
                CommandKind::Immediate(op) => core.commit_immediate(&d, op, &ctx),
                CommandKind::Transition(op) => core.commit_transition(&d, op, &ctx),
            };
            report.command = Some((d.stamp.request, match result {
                Ok(c) => {
                    let install = matches!(d.kind, CommandKind::Install { .. });
                    let supersedes = install && c.action().is_some() && c.request() == d.stamp.request && self.active.is_some();
                    if supersedes {
                        t.ack_deferred(c).map(|(acked, ack_reply)| {
                            let old = self.active.take().expect("checked active.is_some()");
                            superseded_and_ack = Some((old, ack_reply));
                            if let Some(i) = acked.installed {
                                self.active = Some(i);
                            }
                            acked.commit_seq
                        })
                    } else {
                        t.ack(c).map(|acked| {
                            match acked.installed {
                                Some(i) if install => self.active = Some(i),
                                Some(i) => self.transition = Some(i),
                                None => {}
                            }
                            acked.commit_seq
                        })
                    }
                }
                Err(code) => {
                    t.refuse(code);
                    Err(code)
                }
            }));
        }
        // 4. Unconditional urgent recheck + consume at most one latest target
        let ctx = self.checkpoint(core, Checkpoint::BeforeTarget, &mut now_fn, &mut report);
        if let Some(f) = self.targets.take() {
            let result = core.consume_target(&f, &ctx);
            self.targets.report(TargetOutcome { stream: f.stream, seq: f.seq, result });
            report.target_refusal = result.err();
            report.target = Some((f.seq, result.map_err(|r| r.code)));
        }
        // 5. Urgent recheck + advance control output
        let _ = self.checkpoint(core, Checkpoint::BeforeOutput, &mut now_fn, &mut report);
        let timestamp = core.advance_output(dt, publish_generation);
        // 6. Publish
        core.publish(timestamp, publish_generation);
        // 7. Deliver terminals (transition terminal, superseded terminal + deferred ACK, then active-action terminal) strictly AFTER publish
        if let Some((t, outcome)) = finished_transition {
            self.intake.terminal(t, outcome);
        }
        if let Some((old, ack_reply)) = superseded_and_ack {
            self.intake.terminal(old, Outcome::Superseded);
            self.intake.emit_reply(ack_reply);
        }
        if let Some(action) = self.active() {
            if let Some(outcome) = core.action_outcome(action) {
                let installed = self.active.take().expect("active above");
                let children = core.action_children(action);
                self.intake.terminal_with_children(installed, outcome, children);
                report.ended = Some((action, outcome));
            }
        }
        report
    }

    /// Runs one sub-release transition wake on `core` (no full feedback poll, no output step).
    /// When the transition completes on this wake, publishes the updated state at `publish_generation`
    /// BEFORE delivering the terminal reply and returns `true`.
    pub fn service_wake<O: CoreOps>(
        &mut self,
        core: &mut O,
        now: Instant,
        release_deadline: Instant,
        publish_generation: u64,
        report: &mut ReleaseReport,
    ) -> bool {
        if self.apply_pending_urgent(core) { report.stopped = true; }
        let budget = ServiceBudget::for_release(now, release_deadline);
        report.poll = core.poll_transition_wake(&budget);
        if let Some((t, outcome)) = self.take_finished_transition(report) {
            let ts = core.now_timestamp();
            core.publish(ts, publish_generation);
            self.intake.terminal(t, outcome);
            return true;
        }
        false
    }
}

/// Advances one core one release at a time, in the executor's order: urgent stop, hardware
/// poll, at most one ordinary command, the latest target, the control tick, terminal
/// detection, then extra hardware services at a pending transition's wake before the next
/// release (`HardwarePoll::wake_before`, at most [`MAX_SERVICE_WAKES`]). Tests and tracks drive it deterministically: no thread, and virtual time
/// ([`VirtualClock`]): release `k` (from 1) starts at `clock.base() + (k - 1) * dt` plus what
/// a test advanced.
pub struct ReleaseFixture<O: CoreOps> {
    pub core: O,
    engine: ReleaseEngine,
    clock: VirtualClock,
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
    pub fn new(core: O, boot: BootId, caps: Capacities) -> (Self, Broker<PlanSlot>, TargetWriter, StopRequests) {
        let (engine, broker, tw, stop) = ReleaseEngine::new(boot, caps);
        let clock = VirtualClock::starting_at(Instant::now());
        (Self { core, engine, clock, dt: 0.02 }, broker, tw, stop)
    }

    /// The fixture's virtual clock (clone it to advance time from a test).
    pub fn clock(&self) -> &VirtualClock { &self.clock }
    /// Virtual now.
    pub fn now(&self) -> Instant { self.clock.now() }

    pub fn releases(&self) -> u64 { self.engine.releases() }
    pub fn active(&self) -> Option<ActionId> { self.engine.active() }
    /// The driver transition in flight.
    pub fn transition(&self) -> Option<ActionId> { self.engine.transition() }
    pub fn reply_overflow(&self) -> u64 { self.engine.reply_overflow() }

    pub fn ctx(&self, now: Instant) -> CommitCtx { self.engine.ctx(now) }

    /// Runs `hook` at every [`Checkpoint`] just before the fixture samples the stop
    /// generation and the time there (tests: a stop or a clock step arriving mid-release).
    pub fn on_checkpoint(&mut self, hook: impl FnMut(Checkpoint) + Send + 'static) { self.engine.on_checkpoint(hook); }

    /// One release.
    pub fn step(&mut self) -> ReleaseReport {
        let started = self.clock.now();
        let release_deadline = started + Duration::from_secs_f64(self.dt);
        let next_gen = self.engine.releases() + 1;
        let clock = &self.clock;
        let mut report = self.engine.step(&mut self.core, || clock.now(), self.dt, release_deadline, next_gen);
        // Extra sub-release hardware services at a pending transition's wake before the next release
        // (a transition committed in this release is first served at the next one).
        let mut next = report.poll.wake_before(release_deadline);
        while let Some(wake) = next {
            if report.wakes == MAX_SERVICE_WAKES { break; }
            report.wakes += 1;
            self.clock.advance(wake.saturating_duration_since(self.clock.now()));
            self.engine.service_wake(&mut self.core, self.clock.now(), release_deadline, next_gen, &mut report);
            next = report.poll.wake_before(release_deadline);
        }
        // The next release starts one period later
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
    fn the_commit_start_check_uses_the_plan_start_threshold() {
        assert_eq!(START_TOLERANCE, crate::motion::PLAN_START_TOL_RAD as f32);
        let t = START_TOLERANCE;
        assert!(start_matches(&[0.0, 0.0], &[t * 0.5, 0.0]));
        assert!(!start_matches(&[0.0, 0.0], &[0.0, t]));
    }

    #[test]
    fn delegated_feedback_and_config_codes_keep_their_name() {
        for code in ["FEEDBACK_STALE", "CONFIG_BUSY"] {
            assert_eq!(static_code(ArmError::detailed(409, code, "x", serde_json::json!({}))), code);
        }
        assert_eq!(static_code(ArmError::detailed(409, "SOMETHING_NEW", "x", serde_json::json!({}))), "REJECTED");
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
        assert_eq!(outcome_of("failed", Some("FEEDBACK_STALE")), Some(Outcome::Failed("FEEDBACK_STALE")));
        // every fault code is listed
        for c in crate::safety::FaultCode::ALL {
            let name = serde_json::to_value(c).unwrap();
            let name = name.as_str().unwrap();
            if name != "OTHER" { assert!(TERMINAL_CODES.contains(&name), "{name}"); }
        }
    }
}
