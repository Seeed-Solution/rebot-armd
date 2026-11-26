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

use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::time::{Duration, Instant};

use super::broker::{
    ActionId, Authority, BootId, Broker, Capacities, CommandKind, Descriptor, ImmediateOp, Installed, Outcome, PlanSlotRef, RequestId, RtIntake,
    TransitionOp,
};
use super::budget::{ReleaseBudget, DEFAULT_TX_ATTEMPT_BUDGET};
use super::config_iface::{Activated, ConfigAuthority, ConfigDomain, ConfigProposal, Durability, Released, Reservation, Reserved};
use super::ids::ArmId;
use super::mailbox::{target_mailbox, Refusal, TargetFrame, TargetOutcome, TargetReader, TargetWriter, MAX_DOF};
use super::urgent::{StopRequests, UrgentStop, UrgentStopIntake, UrgentStopOutcome};
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
    pub(super) fn authorized(d: &Descriptor, action: Option<ActionId>) -> Self { Self { request: d.stamp.request, action } }

    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn for_action(d: &Descriptor, action: Option<ActionId>) -> Self {
        Self { request: d.stamp.request, action }
    }

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
    /// Prepared action-family command or macro child staged for [`super::requests::actions`] (A6 + A7).
    #[cfg(not(target_arch = "wasm32"))]
    Action(super::requests::actions::StagedAction),
}

/// What a [`Primitive::Prepared`] path is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PathKind {
    /// Samples at path parameters `0..=1` (`PreparedPath::params`, first sample = start),
    /// run over `duration_s` with the quintic time law of a Cartesian path.
    Cartesian,
    /// Recorded samples every `sample_dt` s of recorded time (`duration_s` recorded), played
    /// at `speed` after a pre-planned approach quintic (`approach_duration_s`, optional
    /// `approach_power_scale`) from the commanded position to the first sample (planned,
    /// power-fitted, and workspace-swept off-thread at prepare time; install verifies start
    /// position only). `gripper` false keeps the gripper's command.
    /// `tool_digest`: [`tool_digest_hash`] of the active tool the worker checked the
    /// episode's tool provenance against (`TOOL_MISMATCH` at the commit when it changed).
    Replay {
        episode: EpisodeRef,
        speed: f32,
        gripper: bool,
        sample_dt: f64,
        approach_duration_s: f64,
        approach_power_scale: Option<f64>,
        tool_digest: u64,
    },
}

/// FNV-1a hash of a tool configuration digest (no allocation): the prepared replay names the
/// tool its provenance was checked against.
pub fn tool_digest_hash(digest: &str) -> u64 {
    digest.bytes().fold(0xcbf2_9ce4_8422_2325u64, |h, b| (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3))
}

/// Joint samples one plan slot's [`PreparedPath`] holds (allocated and write-touched when
/// the slot is built; a longer path or episode is refused `PLAN_TOO_LARGE` when prepared).
pub const MAX_PATH_SAMPLES: usize = 2048;

/// Payload bytes reserved by one default [`PlanSlot`]'s [`PreparedPath`] (`2048 * (8 * 4 + 8) = 81,920`).
pub const PLAN_SLOT_PAYLOAD_BYTES: usize = MAX_PATH_SAMPLES * (std::mem::size_of::<[f32; MAX_DOF]>() + std::mem::size_of::<f64>());

/// Total payload bytes reserved across [`super::broker::PLAN_SLOTS`] default plan slots (`8 * 81,920 = 655,360`).
pub const ALL_PLAN_SLOTS_PAYLOAD_BYTES: usize = super::broker::PLAN_SLOTS * PLAN_SLOT_PAYLOAD_BYTES;

/// Write-touches every element of `v`'s allocated spare capacity via volatile stores so the
/// OS faults physical pages at construction rather than leaving lazy COW zero pages.
fn pre_fault_spare<T: Copy>(v: &mut Vec<T>, val: T) {
    for slot in v.spare_capacity_mut() {
        // SAFETY: `slot` points to valid, aligned, exclusively-owned allocated memory in `v`.
        unsafe { std::ptr::write_volatile(slot.as_mut_ptr(), val) };
    }
}

/// The joint samples of a [`Primitive::Prepared`], in a buffer allocated and pre-faulted
/// once (plan slot construction) and refilled in place: preparing never reallocates it.
#[derive(Debug, Clone, PartialEq)]
pub struct PreparedPath {
    samples: Vec<[f32; MAX_DOF]>,
    params: Vec<f64>,
}

impl PreparedPath {
    pub fn with_capacity(n: usize) -> Self {
        let mut samples = Vec::with_capacity(n);
        pre_fault_spare(&mut samples, [0.0; MAX_DOF]);
        let mut params = Vec::with_capacity(n);
        pre_fault_spare(&mut params, 0.0);
        Self { samples, params }
    }
    pub fn capacity(&self) -> usize { self.samples.capacity().min(self.params.capacity()) }
    /// Payload bytes reserved by `samples` and `params` backing buffers.
    pub fn payload_bytes(&self) -> usize {
        self.samples.capacity() * std::mem::size_of::<[f32; MAX_DOF]>() + self.params.capacity() * std::mem::size_of::<f64>()
    }
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

/// One plan slot: the primitive, the prepared path a [`Primitive::Prepared`] reads, optional
/// `Copy` prepared motion plan (`PlanSlot::motion`), and (for prepared replays) the exact
/// off-thread fitted & workspace-swept approach quintic.
#[derive(Debug, Clone, PartialEq)]
pub struct PlanSlot {
    pub primitive: Primitive,
    pub path: PreparedPath,
    pub motion: Option<super::requests::motion::PreparedMotionSlot>,
    #[cfg(not(target_arch = "wasm32"))]
    pub replay_approach: Option<crate::replay::PreparedReplayApproach>,
}

impl Default for PlanSlot {
    /// A slot with a [`MAX_PATH_SAMPLES`] path buffer (pre-touched by the allocation).
    fn default() -> Self { Self::with_path_capacity(MAX_PATH_SAMPLES) }
}

impl PlanSlot {
    pub fn with_path_capacity(n: usize) -> Self {
        Self {
            primitive: Primitive::None,
            path: PreparedPath::with_capacity(n),
            motion: None,
            #[cfg(not(target_arch = "wasm32"))]
            replay_approach: None,
        }
    }

    /// Payload bytes reserved by this slot's [`PreparedPath`] buffers.
    pub fn payload_bytes(&self) -> usize { self.path.payload_bytes() }

    /// Stages a primitive that reads no path (the path, prepared motion slot, and replay approach are cleared).
    pub fn set(&mut self, p: Primitive) {
        self.primitive = p;
        self.path.clear();
        self.motion = None;
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.replay_approach = None;
        }
    }

    /// Stages a slot the worker prepared (`PLAN_TOO_LARGE`: nothing staged; the slot is empty).
    pub fn copy_from(&mut self, other: &PlanSlot) -> Result<(), &'static str> {
        self.primitive = Primitive::None;
        self.motion = None;
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.replay_approach = None;
        }
        self.path.copy_from(&other.path)?;
        self.primitive = other.primitive;
        self.motion = other.motion;
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.replay_approach = other.replay_approach;
        }
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
            #[cfg(not(target_arch = "wasm32"))]
            Self::Action(a) => a.child(),
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
    "STOPPED", "CANCELLED", "SUPERSEDED", "LEASE_LOST", "DISCONNECTED", "EPOCH_MISMATCH", "PLAN_STALE", "CALIBRATION_CHANGED", "CONFIG_BUSY",
    "DEPLOYMENT_CHANGED", "TOOL_MISMATCH", "DATA_STALE", "NOT_ARMED", "MOTION_BUSY", "TRANSITION_BUSY", "HANDOVER_SETTLING", "DISABLE_PENDING",
    "INTERNAL", "UNSUPPORTED", "OPERATOR_REQUIRED", "DEADLINE_EXPIRED", "WRONG_TARGET", "INTERNAL_FAULT", "SAFE_HOLD",
    // faults and holds (FaultCode, SCREAMING_SNAKE_CASE)
    "ESTOP", "ESTOP_ACTIVE", "FAULT", "FAULT_ACTIVE", "BUS_LOSS", "THERMAL_TRIP", "JOINT_LIMIT", "WATCHDOG", "WATCHDOG_TIMEOUT",
    "FOLLOWING_ERROR", "CONTROL_DEADLINE", "INVALID_TELEMETRY", "DRIVER_WRITE_FAILED", "CAN_TX_STALLED", "J7_FEEDBACK_STALE", "HARDWARE_FAULT", "ENABLE_FAILED",
    "SERVO_NO_BRAKE", "HANDOVER_UNSETTLED", "GRAVITY_COMP_FAILED", "COLLISION", "COLLISION_HOLD", "BLOCKED_HOLD", "WORKSPACE_HOLD",
    "TORQUE_LIMIT", "TRACKING_STALL", "SAFETY_ERROR", "SAFETY_REJECTED", "SOFT_LIMIT_EXCEEDED", "FEEDBACK_UNAVAILABLE",
    "FEEDBACK_UNCONFIRMED", "FEEDBACK_STALE", "INVALID_FEEDBACK", "UNLOAD_FAILED", "TORQUE_OFF_WOULD_DROP", "ARM_NOT_AT_RESTING_POSE", "DRIVER_BUSY",
    // motion and action failures
    "IK_UNREACHABLE", "IK_BRANCH_FLIP", "WORKSPACE_VIOLATION", "INVALID_COMMAND", "INVALID_TARGET", "JUMP_LIMIT", "STEP_LIMIT",
    "SERVO_INFEASIBLE", "SERVO_TARGET_DISCARDED", "STALE_INPUT", "MOTION_TIMEOUT", "GRIPPER_TIMEOUT", "GRASP_TIMEOUT",
    "PLACE_TIMEOUT", "TARGET_TIMEOUT", "SEQUENCE_TIMEOUT", "HOVER_TIMEOUT", "MEASURE_TIMEOUT", "MAX_SECONDS", "NOT_HOLDING", "TARGET_LOST", "TARGET_DISPLACED",
    "DROPPED", "OBJECT_HELD", "OBJECT_TOO_TALL", "TOO_WIDE", "CENTRING_FAILED", "DETECTION_UNAVAILABLE", "TASK_UNSUPPORTED",
    "OBSERVATION_UNAVAILABLE", "PERSIST_FAILED", "INVALID_CALIBRATION", "POLICY_REVISION_MISMATCH", "POLICY_NOT_ACTIVE", "POWER_LIMIT",
    "LIFT_TORQUE", "INSUFFICIENT_EVIDENCE", "BUDGET_EXHAUSTED",
    "VISION_STALE", "INFERENCE_ERROR", "POLICY_REVOKED", "POLICY_FAILED", "MODEL_CHANGED", "HANDSHAKE_EXPIRED",
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

    /// Derives a tightened [`ServiceBudget`] from a shared [`ReleaseBudget`] ledger.
    pub fn from_release_budget(
        ledger: &ReleaseBudget,
        now: Instant,
        max_duration: Duration,
        max_rx: u16,
    ) -> Self {
        ledger.service_budget(now, max_duration, max_rx)
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
    /// Sub-release wake for an internal park transition (`ParkRearm` or `Park` disable);
    /// separate from `torque` so broker `ReleaseEngine::transition` stays isolated.
    pub park_wake: Option<Instant>,
    /// True when a park `ParkRearm` or `Park` disable transition finished on this poll,
    /// requiring a Step-6 / sub-release publication.
    pub park_state_changed: bool,
    /// Earliest watchdog-required sub-release wake for already-enabled arm joints while a
    /// transition is in flight (`None` when no transition is active or no motor is energized).
    /// Admitted up to the release's urgent cutoff (never deferred by optional-work margins).
    pub urgent_wake: Option<Instant>,
}

impl HardwarePoll {
    /// When the bus owner wants to be serviced before the next release (a pending broker or park transition).
    pub fn wake(&self) -> Option<Instant> {
        let torque_wake = match self.torque { Some(Ok(TorqueProgress::Pending { wake })) => Some(wake), _ => None };
        match (torque_wake, self.park_wake) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        }
    }

    /// The executor's extra service within the release ending at `release`: the pending
    /// transition's wake when it comes before the release (None: serve it at the release).
    pub fn wake_before(&self, release: Instant) -> Option<Instant> { self.wake().filter(|w| *w < release) }

    /// Next sub-release wake admitted for Window 2 as `(wake_at, is_urgent)`:
    /// - Optional transition polls (`wake()`) are deferred to the next release past `optional_cutoff`.
    /// - Watchdog-required holds (`urgent_wake`) remain eligible up to `urgent_cutoff`.
    pub fn admitted_wake_before(
        &self,
        optional_cutoff: Instant,
        urgent_cutoff: Instant,
    ) -> Option<(Instant, bool)> {
        let opt = self.wake().filter(|w| *w < optional_cutoff);
        let urg = self.urgent_wake.filter(|w| *w < urgent_cutoff);
        match (opt, urg) {
            (Some(a), Some(b)) => {
                if b <= a {
                    Some((b, true))
                } else {
                    Some((a, false))
                }
            }
            (Some(a), None) => Some((a, false)),
            (None, Some(b)) => Some((b, true)),
            (None, None) => None,
        }
    }
}

/// Maximum sub-release transition/urgent wakes the executor runs within one release.
/// Bounded to 1 so a single release cannot stack multiple wake sleep+poll cycles past its period.
pub const MAX_SERVICE_WAKES: u32 = 1;

/// Default safety margin reserved before `release_deadline` so any sub-release wake
/// (sleep + poll + publish + shadow flush) completes by `period - margin`
/// (`>= 4 ms` on 50 Hz / 20 ms SG2002 releases, configurable via `ExecutorConfig::sub_release_margin`
/// or `ARMD_SUB_RELEASE_MARGIN_MS`).
pub const DEFAULT_SUB_RELEASE_MARGIN: Duration = Duration::from_millis(4);

/// Estimated worst-case sub-release transition poll duration on SG2002 (RX drain + transition TX +
/// publish + shadow flush). Combined with [`DEFAULT_SUB_RELEASE_MARGIN`] (4 ms), this reserves
/// 9 ms ahead of `release_deadline` on 20 ms releases (`t + 11 ms` start cutoff -> `<= 16 ms` end).
pub const SUB_RELEASE_POLL_RESERVE: Duration = Duration::from_millis(5);

/// Tail reserve kept between the Window-2 hardware poll deadline and `wake_deadline`
/// (`period - margin`) on 50 Hz (`>= 20 ms`) releases to cover observation publish, reply flush,
/// shadow flush, and diagnostic ring push/seal (measured ~0.35 ms on SG2002).
pub const SUB_RELEASE_TAIL_RESERVE: Duration = Duration::from_millis(1);

/// Minimum positive headroom before `wake_deadline` required to flush non-urgent torque-shadow
/// steps during Window 2 so a delayed poll cannot spill non-urgent shadow work past `wake_deadline`.
pub const SUB_RELEASE_SHADOW_MARGIN: Duration = Duration::from_micros(500);

/// Single-authority sub-release wake budget shared by the RT executor and [`ReleaseFixture`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SubReleaseBudget {
    /// Target release completion deadline (`release_deadline - effective_margin`).
    pub wake_deadline: Instant,
    /// Deadline passed into the Window-2 hardware transition poll (`wake_deadline - tail_reserve`)
    /// so driver TX/RX finishes with enough tail headroom to complete publish/shadow/seal by `wake_deadline`.
    pub poll_deadline: Instant,
    /// Admission cutoff for starting/sleeping to a CAN transition or watchdog-hold wake:
    /// `wake_deadline - poll_reserve`, so `wake_sleep + wake_poll + tail` completes by `wake_deadline`.
    pub transition_start_cutoff: Instant,
    /// Post-sleep cutoff for a transition or watchdog-hold wake (allows up to 1 ms OS timer slack
    /// past `transition_start_cutoff` on 20 ms releases while still guaranteeing headroom before `poll_deadline`).
    pub transition_post_sleep_cutoff: Instant,
    /// Cutoff for immediate in-memory urgent Stop / E-Stop / Disconnect wakes (no sleep, no CAN poll):
    /// always anchored to `release_deadline - publish_margin` (independent of `sub_release_margin`).
    pub urgent_stop_cutoff: Instant,
}

impl SubReleaseBudget {
    /// Default (and minimum safe) safety margin for `period`: 4 ms on 50 Hz (`>= 20 ms`) SG2002
    /// releases, scaled proportionally (`period / 5`) for faster test periods so short-period mock
    /// tests keep consistent 20% tail headroom.
    pub fn default_margin_for_period(period: Duration) -> Duration {
        DEFAULT_SUB_RELEASE_MARGIN.min(period / 5)
    }

    /// Computes all Window-2 sub-release cutoffs for one release from its period and configured
    /// `sub_release_margin`. Enforces the minimum safe floor (`default_margin_for_period(period)`,
    /// i.e. `>= 4 ms` on 50 Hz / 20 ms releases) so neither env nor direct config can bypass it.
    pub fn for_release(
        release_start: Instant,
        release_deadline: Instant,
        period: Duration,
        sub_release_margin: Duration,
    ) -> Self {
        let min_margin = Self::default_margin_for_period(period);
        let effective_margin = sub_release_margin.max(min_margin).min(period);
        let wake_deadline = release_deadline
            .checked_sub(effective_margin)
            .unwrap_or(release_start);
        let tail_reserve = if period >= Duration::from_millis(20) {
            SUB_RELEASE_TAIL_RESERVE
        } else {
            Duration::ZERO
        };
        let poll_deadline = wake_deadline
            .checked_sub(tail_reserve)
            .unwrap_or(release_start);
        let poll_reserve = if period >= Duration::from_millis(20) {
            SUB_RELEASE_POLL_RESERVE
        } else {
            (period / 2)
                .saturating_sub(effective_margin)
                .min(SUB_RELEASE_POLL_RESERVE)
        };
        let transition_start_cutoff = wake_deadline
            .checked_sub(poll_reserve)
            .unwrap_or(release_start);
        let post_sleep_reserve = if period >= Duration::from_millis(20) {
            poll_reserve.saturating_sub(Duration::from_millis(1))
        } else {
            Duration::ZERO
        };
        let transition_post_sleep_cutoff = wake_deadline
            .checked_sub(post_sleep_reserve)
            .unwrap_or(release_start);
        // Immediate in-memory urgent Stop / E-Stop / Disconnect intake is NEVER disabled by larger
        // `sub_release_margin` overrides (e.g. margin = 20 ms), while still reserving the minimum
        // safe completion margin (`min_margin`, 4 ms at 50 Hz) + `SUB_RELEASE_SHADOW_MARGIN`
        // (0.5 ms at 50 Hz -> cutoff at `t + 15.5 ms`, vs 2 ms `DAMIAO_TX_RELEASE_PUBLISH_MARGIN`
        // for shorter periods) so even an urgent in-memory wake (typical service < 0.2 ms, 0 CAN
        // I/O) completes strictly before `release_deadline - min_margin` (<= 16 ms at 50 Hz).
        let publish_margin = crate::driver::DAMIAO_TX_RELEASE_PUBLISH_MARGIN.min(period / 2);
        let urgent_headroom = if period >= Duration::from_millis(20) {
            SUB_RELEASE_SHADOW_MARGIN
        } else {
            Duration::ZERO
        };
        let urgent_margin = min_margin
            .max(publish_margin)
            .saturating_add(urgent_headroom)
            .min(period / 2);
        let urgent_stop_cutoff = release_deadline
            .checked_sub(urgent_margin)
            .unwrap_or(release_start);
        Self {
            wake_deadline,
            poll_deadline,
            transition_start_cutoff,
            transition_post_sleep_cutoff,
            urgent_stop_cutoff,
        }
    }

    /// Single-authority Window-2 wake admission shared by the RT executor and [`ReleaseFixture`].
    pub fn admit_next_wake(
        &self,
        now: Instant,
        has_pending_urgent: bool,
        poll: &HardwarePoll,
    ) -> Option<(Instant, SubReleaseWakeKind)> {
        if has_pending_urgent && now < self.urgent_stop_cutoff {
            Some((now, SubReleaseWakeKind::UrgentStop))
        } else {
            poll.admitted_wake_before(self.transition_start_cutoff, self.transition_start_cutoff)
                .map(|(w, _)| (w, SubReleaseWakeKind::Transition))
        }
    }

    /// Single-authority post-sleep gate shared by the RT executor and [`ReleaseFixture`].
    /// Returns `Some(allow_transition_poll)` if the wake should execute at `post_now`, or `None`
    /// if the cutoff has already passed.
    pub fn post_sleep_allow_transition_poll(
        &self,
        post_now: Instant,
        urgent_stop_now: bool,
    ) -> Option<bool> {
        if urgent_stop_now {
            (post_now < self.urgent_stop_cutoff).then_some(false)
        } else {
            (post_now < self.transition_post_sleep_cutoff).then_some(true)
        }
    }
}

/// Parses an optional `ARMD_SUB_RELEASE_MARGIN_MS` override (in milliseconds), falling back to
/// [`SubReleaseBudget::default_margin_for_period`] when unset or empty and failing closed when
/// below the minimum safe margin for `period` (`>= 4 ms` at 50 Hz / 20 ms).
pub fn parse_sub_release_margin_ms(
    raw: Option<&str>,
    period: Duration,
) -> Result<Duration, String> {
    let min_margin = SubReleaseBudget::default_margin_for_period(period);
    match raw.map(str::trim).filter(|s| !s.is_empty()) {
        None => Ok(min_margin),
        Some(s) => {
            let ms = s.parse::<u64>().map_err(|_| {
                format!("ARMD_SUB_RELEASE_MARGIN_MS must be a non-negative integer in ms, got {s:?}")
            })?;
            let parsed = Duration::from_millis(ms);
            if parsed < min_margin {
                return Err(format!(
                    "ARMD_SUB_RELEASE_MARGIN_MS must be >= {:.3} ms for period {:.3} ms, got {ms} ms",
                    min_margin.as_secs_f64() * 1e3,
                    period.as_secs_f64() * 1e3,
                ));
            }
            Ok(parsed)
        }
    }
}

/// Kind of admitted Window-2 sub-release wake.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SubReleaseWakeKind {
    /// Driver transition or watchdog-hold poll (may sleep and run bounded CAN RX/TX).
    Transition,
    /// Immediate in-memory urgent Stop / E-Stop / Disconnect intake (no sleep).
    UrgentStop,
}

/// The narrow operations of one arm's core.
/// Zero-allocation RT commit refusal carrying a static error code and an optional per-request
/// diagnostic attached directly to the request's [`Reply::Refused`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CommitRefusal {
    pub code: &'static str,
    pub detail: Option<crate::motion::PlanningLimitViolationRt>,
}

impl CommitRefusal {
    #[inline]
    pub const fn code(code: &'static str) -> Self {
        Self { code, detail: None }
    }

    #[inline]
    pub const fn planning_limit(v: crate::motion::PlanningLimitViolationRt) -> Self {
        Self { code: "IK_UNREACHABLE", detail: Some(v) }
    }
}

impl From<&'static str> for CommitRefusal {
    #[inline]
    fn from(code: &'static str) -> Self {
        Self::code(code)
    }
}

impl From<crate::motion::PlanningLimitViolationRt> for CommitRefusal {
    #[inline]
    fn from(v: crate::motion::PlanningLimitViolationRt) -> Self {
        Self::planning_limit(v)
    }
}

pub trait CoreOps {
    /// Current authority at `ctx.now` and `ctx.stop_generation` (the urgent-stop generation
    /// is not core state): the lease generation of a lease valid at `ctx.now` (0: none), the
    /// generation of the parent that may install children (0: none).
    fn authority(&self, ctx: &CommitCtx) -> Authority;
    fn planning_snapshot(&self, ctx: &CommitCtx) -> PlanningSnapshot;
    /// Installs `primitive` for `d` after rechecking its stamp; refusal leaves active motion unchanged.
    fn commit_primitive(&mut self, d: &Descriptor, primitive: &Primitive, ctx: &CommitCtx) -> Result<Committed, &'static str>;
    /// Installs an off-lock prepared jog (`PreparedMotionSlot`) for `d` with zero RT planning.
    fn commit_install_slot(&mut self, _d: &Descriptor, _slot: &super::requests::motion::PreparedMotionSlot, _ctx: &CommitCtx) -> Result<Committed, &'static str> {
        Err("UNSUPPORTED")
    }
    /// Installs an off-lock prepared `home` motion (`PreparedMotionSlot`) for `d`.
    fn commit_home(&mut self, _d: &Descriptor, _slot: &super::requests::motion::PreparedMotionSlot, _ctx: &CommitCtx) -> Result<Committed, &'static str> {
        Err("UNSUPPORTED")
    }
    /// Installs an off-lock prepared `park` motion (or `holding_after_clear` rearm) for `d`.
    fn commit_park(&mut self, _d: &Descriptor, _slot: &super::requests::motion::PreparedMotionSlot, _ctx: &CommitCtx) -> Result<Committed, &'static str> {
        Err("UNSUPPORTED")
    }
    /// Installs an off-lock prepared continuation trajectory onto an existing deferred `park` action.
    fn commit_park_continuation(&mut self, _d: &Descriptor, _slot: &super::requests::motion::PreparedMotionSlot, _ctx: &CommitCtx) -> Result<Committed, &'static str> {
        Err("UNSUPPORTED")
    }
    fn commit_immediate(&mut self, d: &Descriptor, op: ImmediateOp, ctx: &CommitCtx) -> Result<Committed, &'static str>;
    /// Executes one lease lifecycle request (`LeaseAcquire`, `LeasePreempt`, `LeaseRelease`, `LeaseDisconnect`).
    fn commit_lease(
        &mut self,
        _d: &Descriptor,
        _req: super::requests::lease::LeaseRequest,
        _ctx: &CommitCtx,
    ) -> Result<super::requests::lease::LeaseOutcome, &'static str> {
        Err("NOT_WIRED")
    }
    /// Applies one priority connection lifecycle command (`Open` / `Disconnect`) at the owner.
    fn handle_conn_lifecycle(
        &mut self,
        cmd: super::requests::lease::LifecycleCmd,
        _now: Instant,
    ) -> super::requests::lease::LifecycleReply {
        use super::requests::lease::{LeaseOutcome, LifecycleCmd, LifecycleReply};
        match cmd {
            LifecycleCmd::Open { seq, conn_id } => LifecycleReply::Opened {
                seq,
                conn_id,
                result: Ok(()),
            },
            LifecycleCmd::Disconnect { seq, connection } => LifecycleReply::Disconnected {
                seq,
                conn_id: connection.id(),
                outcome: LeaseOutcome::Disconnected {
                    released: false,
                    displaced_generation: 0,
                },
            },
        }
    }
    /// Installs a [`Primitive::Prepared`] with its slot's `path` after rechecking its stamp
    /// (as `commit_primitive`); no IK and no file I/O. Default: `UNSUPPORTED`.
    fn commit_prepared(&mut self, _d: &Descriptor, _primitive: &Primitive, _path: &PreparedPath, _ctx: &CommitCtx) -> Result<Committed, &'static str> {
        Err("UNSUPPORTED")
    }
    /// Installs a [`Primitive::Prepared`] using a pinned [`PlanSlotRef`] so implementations
    /// can borrow the immutable slot samples until terminal retirement without vector copies.
    /// Default delegates to [`Self::commit_prepared`].
    fn commit_prepared_slot(&mut self, d: &Descriptor, slot: PlanSlotRef<PlanSlot>, ctx: &CommitCtx) -> Result<Committed, &'static str> {
        self.commit_prepared(d, &slot.primitive, &slot.path, ctx)
    }
    /// [`Self::commit_prepared_slot`] preserving per-request [`CommitRefusal`] details for `Reply::Refused`.
    fn commit_prepared_slot_with_detail(&mut self, d: &Descriptor, slot: PlanSlotRef<PlanSlot>, ctx: &CommitCtx) -> Result<Committed, CommitRefusal> {
        self.commit_prepared_slot(d, slot, ctx).map_err(CommitRefusal::code)
    }
    /// Starts a driver transition after rechecking the stamp; its progress and outcome come
    /// from [`Self::poll_hardware`] (`HardwarePoll::torque`). The committed action id names
    /// the transition (the adapter uses the request id's value).
    fn commit_transition(&mut self, d: &Descriptor, op: TransitionOp, ctx: &CommitCtx) -> Result<Committed, &'static str>;
    /// Release-bounded [`Self::commit_transition`] carrying the absolute control `release_deadline`
    /// so initial handshake TX frames are capped at `release_deadline - margin`.
    fn commit_transition_for_release(
        &mut self,
        d: &Descriptor,
        op: TransitionOp,
        ctx: &CommitCtx,
        _release_deadline: Instant,
    ) -> Result<Committed, &'static str> {
        self.commit_transition(d, op, ctx)
    }
    /// Release-bounded [`Self::commit_transition`] with a live owner clock callback for
    /// conservative driver-clock-first cutoff translation.
    fn commit_transition_for_release_with_clock(
        &mut self,
        d: &Descriptor,
        op: TransitionOp,
        ctx: &CommitCtx,
        release_deadline: Instant,
        _now_fn: &mut dyn FnMut() -> Instant,
    ) -> Result<Committed, &'static str> {
        self.commit_transition_for_release(d, op, ctx, release_deadline)
    }
    /// Per-release budgeted TX attempt cap used to initialize [`ReleaseEngine`]'s [`ReleaseBudget`].
    fn release_tx_budget_limit(&self) -> u16 {
        DEFAULT_TX_ATTEMPT_BUDGET
    }
    /// Ledger-threaded [`Self::commit_transition`]: draws deadline and TX attempts from `budget`.
    fn commit_transition_with_budget(
        &mut self,
        d: &Descriptor,
        op: TransitionOp,
        ctx: &CommitCtx,
        budget: &mut ReleaseBudget,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> Result<Committed, &'static str> {
        if budget.deadline_exceeded(now_fn()) {
            return Err("BUDGET_EXPIRED");
        }
        self.commit_transition_for_release_with_clock(d, op, ctx, budget.deadline(), now_fn)
    }
    /// Applies one target after rechecking it ([`TargetFrame::check`] against the core's
    /// [`super::mailbox::TargetAuthority`], then the configuration hold).
    ///
    /// No allocation: the frame is `Copy` with fixed-size arrays; an implementation applies
    /// `frame.positions()` (a slice of the frame) into the core's preallocated target state.
    /// (The foundation adapter still delegates to `servo(Vec)`, which allocates: track D.)
    fn consume_target(&mut self, frame: &TargetFrame, ctx: &CommitCtx) -> Result<(), Refusal>;
    /// Applies one target together with its atomic [`super::mailbox::PolicyValidityBounds`].
    fn consume_target_with_validity(
        &mut self,
        frame: &TargetFrame,
        _validity: super::mailbox::PolicyValidityBounds,
        ctx: &CommitCtx,
    ) -> Result<(), Refusal> {
        self.consume_target(frame, ctx)
    }
    /// Brakes a live policy run in place when a queued chunked slot is refused because RT
    /// already refused an earlier slot (`expected_refusal_gen` mismatch; zero alloc).
    fn on_policy_refusal_cascade(&mut self, _frame: &TargetFrame) {}
    /// Applies any pending urgent E-stop request; returns true when an E-stop was applied.
    fn apply_estop(&mut self) -> bool { false }
    /// Synchronizes any non-blocking socket-close lease fence (lock-free, zero allocations)
    /// before command commit, target intake, or bus output.
    fn sync_disconnect_fence(&mut self, _now: Instant) -> bool { false }
    /// True when displaced lease generations are queued for off-thread `close_servo` dispatch.
    fn has_pending_servo_closes(&self) -> bool { false }
    /// Drains displaced lease generations queued since the last drain (zero allocs) so the
    /// release engine can dispatch off-thread `close_servo(old_generation)` strictly after publish.
    fn drain_servo_closes(&mut self, _each: &mut dyn FnMut(u64)) {}
    /// True when terminated policy action IDs await post-publication atomic close watermark flush.
    fn has_pending_policy_close(&self) -> bool { false }
    /// True when urgent E-stop, stop, or disconnect-fence work is awaiting service on the owner.
    fn has_pending_urgent(&self) -> bool { false }
    /// Advances the durable post-publication policy close watermark (zero allocs, lossless).
    fn flush_policy_close_watermark(&mut self) -> u64 { 0 }
    /// An authorized stop was requested: stop active motion.
    fn apply_stop(&mut self);
    /// Applies an authorized stop at `generation` and synchronizes the owner's stop counter.
    fn apply_stop_at(&mut self, _generation: u64) {
        self.apply_stop();
    }
    /// Consumes a failed/aborted transition outcome recorded after Step 2 (e.g. an in-flight enable
    /// aborted by `LeasePreempt`, `LeaseRelease`, `LeaseDisconnect`, Step 5 lease expiry, or
    /// thermal supersede) so Step 6 publishes `torque_transition_pending == false` and Step 7
    /// delivers the terminal strictly after publication and strictly before lifecycle/grant replies.
    fn take_aborted_transition(&mut self) -> Option<&'static str> {
        None
    }
    /// Authorizes and applies one bounded urgent [`UrgentStop`] request at the owner.
    /// Only an authorized stop advances `stops` (`StopRequests`), inhibits output, and
    /// applies stop semantics before marking `stops` applied.
    fn handle_urgent_stop(
        &mut self,
        _req: &UrgentStop,
        _stops: &StopRequests,
        _now: Instant,
    ) -> UrgentStopOutcome {
        UrgentStopOutcome::ExecutorStopped
    }
    /// How `action` ended, None while it runs.
    fn action_outcome(&self, action: ActionId) -> Option<Outcome>;
    /// Child primitives the ended macro `action` ran (its terminal record,
    /// `ArmCore::last_macro`); 0 for a plain primitive.
    fn action_children(&self, _action: ActionId) -> u32 { 0 }
    /// The bus owner's progress and confirmations within `budget` (exclusive: the bus
    /// owner's state machine advances here).
    fn poll_hardware(&mut self, budget: &ServiceBudget) -> HardwarePoll;
    /// Release-bounded [`Self::poll_hardware`] carrying the absolute control `release_deadline`
    /// so any refresh/recovery TX started during Step 2 is capped at `release_deadline - margin`.
    fn poll_hardware_for_release(&mut self, budget: &ServiceBudget, _release_deadline: Instant) -> HardwarePoll {
        self.poll_hardware(budget)
    }
    /// Release-bounded [`Self::poll_hardware`] with a live owner clock callback for conservative
    /// driver-clock-first cutoff translation.
    fn poll_hardware_for_release_with_clock(
        &mut self,
        budget: &ServiceBudget,
        release_deadline: Instant,
        _now_fn: &mut dyn FnMut() -> Instant,
    ) -> HardwarePoll {
        self.poll_hardware_for_release(budget, release_deadline)
    }
    /// Ledger-threaded [`Self::poll_hardware`]: draws RX frames, TX attempts, and deadline from
    /// `budget` and charges consumption back into `budget`.
    fn poll_hardware_with_budget(
        &mut self,
        budget: &mut ReleaseBudget,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> HardwarePoll {
        let now = now_fn();
        let sb = budget.poll_service_budget(now);
        let mut poll = self.poll_hardware_for_release_with_clock(&sb, budget.deadline(), now_fn);
        budget.charge_rx(poll.rx_frames, poll.budget_exhausted);
        poll.budget_exhausted |= budget.rx_exhausted();
        poll
    }
    /// Sub-release wake for a pending driver transition only (without running a full release
    /// feedback pass). Default delegates to [`Self::poll_hardware`].
    fn poll_transition_wake(&mut self, budget: &ServiceBudget) -> HardwarePoll {
        self.poll_hardware(budget)
    }
    /// Sub-release wake with a live owner clock callback.
    fn poll_transition_wake_with_clock(
        &mut self,
        budget: &ServiceBudget,
        _now_fn: &mut dyn FnMut() -> Instant,
    ) -> HardwarePoll {
        self.poll_transition_wake(budget)
    }
    /// Release-bounded [`Self::poll_transition_wake`] carrying the absolute control
    /// `release_deadline` so sub-release transition TX obeys `release_deadline - margin`.
    fn poll_transition_wake_for_release(
        &mut self,
        budget: &ServiceBudget,
        _release_deadline: Instant,
    ) -> HardwarePoll {
        self.poll_transition_wake(budget)
    }
    /// Release-bounded [`Self::poll_transition_wake`] with a live owner clock callback.
    fn poll_transition_wake_for_release_with_clock(
        &mut self,
        budget: &ServiceBudget,
        release_deadline: Instant,
        _now_fn: &mut dyn FnMut() -> Instant,
    ) -> HardwarePoll {
        self.poll_transition_wake_for_release(budget, release_deadline)
    }
    /// Ledger-threaded [`Self::poll_transition_wake`]: draws remaining RX frames, TX attempts, and
    /// tightened deadline from `budget`; returns pending immediately on exhaustion without polling.
    fn poll_transition_wake_with_budget(
        &mut self,
        budget: &mut ReleaseBudget,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> HardwarePoll {
        let now = now_fn();
        if !budget.can_rx(now) {
            budget.mark_rx_exhausted();
            return HardwarePoll {
                budget_exhausted: true,
                ..HardwarePoll::default()
            };
        }
        let sb = budget.poll_service_budget(now);
        let mut poll = self.poll_transition_wake_for_release_with_clock(&sb, budget.deadline(), now_fn);
        budget.charge_rx(poll.rx_frames, poll.budget_exhausted);
        poll.budget_exhausted |= budget.rx_exhausted();
        poll
    }
    /// One release of the control tick (`generation`: the publication's lock generation).
    fn advance(&mut self, dt: f64, generation: u64);
    /// Advances control output for `dt` seconds and returns the observation timestamp for
    /// [`Self::publish`]. Default delegates to [`Self::advance`].
    fn advance_output(&mut self, dt: f64, generation: u64) -> f64 {
        self.advance(dt, generation);
        0.0
    }
    /// Release-bounded [`Self::advance_output`] carrying the current release clock `now` and
    /// absolute `release_deadline` so TX start-cutoffs never run past `release_deadline - margin`.
    fn advance_output_with_deadline(
        &mut self,
        dt: f64,
        generation: u64,
        _now: Instant,
        _release_deadline: Instant,
    ) -> f64 {
        self.advance_output(dt, generation)
    }
    /// Release-bounded [`Self::advance_output`] carrying a live monotonic clock sampler `now_fn`
    /// and absolute `release_deadline` so pre-step and transmit-point deadline checks take fresh
    /// clock reads rather than reusing `Checkpoint::BeforeOutput`.
    fn advance_output_with_clock(
        &mut self,
        dt: f64,
        generation: u64,
        now_fn: &mut dyn FnMut() -> Instant,
        release_deadline: Instant,
    ) -> f64 {
        let now = now_fn();
        self.advance_output_with_deadline(dt, generation, now, release_deadline)
    }
    /// Ledger-threaded [`Self::advance_output`]: draws TX attempts and deadline from `budget`.
    fn advance_output_with_budget(
        &mut self,
        dt: f64,
        generation: u64,
        budget: &mut ReleaseBudget,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> f64 {
        self.advance_output_with_clock(dt, generation, now_fn, budget.deadline())
    }
    /// Refreshes `HardwarePoll::urgent_wake` strictly AFTER Step 5 output so control/hold frames
    /// transmitted in Step 5 supersede any stale Step-2 watchdog wake before Window 2 admission.
    /// Translates driver-clock instants onto `owner_now` by elapsed duration so mismatched test
    /// clocks cannot manufacture spurious past-due urgent wakes.
    fn post_output_urgent_wake(
        &self,
        transition_active: bool,
        owner_now: Instant,
        step2_urgent: Option<Instant>,
    ) -> Option<Instant> {
        let _ = owner_now;
        if transition_active { step2_urgent } else { None }
    }
    /// Publishes the state computed by [`Self::advance_output`] at `timestamp` with `generation`.
    fn publish(&mut self, _timestamp: f64, _generation: u64) {}
    /// Current local monotonic observation timestamp (seconds), used when publishing after
    /// a sub-release transition wake before delivering its terminal.
    fn now_timestamp(&self) -> f64 { 0.0 }
    /// Zero-alloc RT commit refusal carrying a static code plus an optional fixed-size
    /// per-request planning-limit diagnostic delivered directly in `Reply::Refused`.
    #[cfg(not(target_arch = "wasm32"))]
    fn commit_action(
        &mut self,
        _d: &Descriptor,
        _cmd: super::requests::actions::ActionCommand,
        _slot: Option<PlanSlotRef<PlanSlot>>,
        _ctx: &CommitCtx,
    ) -> Result<Committed, CommitRefusal> {
        Err(CommitRefusal::code("UNSUPPORTED"))
    }
    /// Commits one policy lifecycle broker command (`ReserveLoad`, `BeginPolicy`, `EndPolicy`).
    #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
    fn commit_policy(
        &mut self,
        _d: &Descriptor,
        _cmd: super::requests::policy::PolicyCommand,
        _ctx: &CommitCtx,
    ) -> Result<Committed, &'static str> {
        Err("UNSUPPORTED")
    }
    /// Publishes the post-completion request watermark strictly AFTER `commit_policy` (or refusal)
    /// finishes reading cancellation slots and updating active load/run tokens.
    #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
    fn finish_policy_command(&mut self, _req_id: u64) {}
    /// True while a macro parent's current child still owns trajectory, settle, or deferred TX.
    fn macro_child_running(&self, _parent: ActionId) -> bool { false }
    /// Attaches the broker's lock-free closed-connection bitset for RT commit checks.
    #[cfg(not(target_arch = "wasm32"))]
    fn attach_closed_bitset(&mut self, _bitset: std::sync::Arc<super::requests::actions::ClosedBitset>) {}
    /// Synchronizes the core's published [`BootId`] with the owning executor / fixture.
    fn set_boot_id(&mut self, _boot: BootId) {}
}

/// Diagnostic fields recorded on the RT thread when an authorized stop arrives during a
/// watchdog park; formatted and logged off-thread via [`StopDiagSlot::drain_and_log`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StopDiagRecord {
    pub count: u64,
    pub arm: ArmId,
    pub park_elapsed_us: u64,
    pub park_budget_us: u64,
}

/// Preallocated, lock-free diagnostic slot for `authorized_stop` on the RT thread.
///
/// The RT path only performs plain atomic stores (zero allocations, zero formatting, zero
/// tracing); normal-priority periodic paths ([`super::publish::StateHub::latest`], driven by
/// the dispatcher loop) drain and format via [`Self::drain_and_log_once`].
#[derive(Debug)]
pub struct StopDiagSlot {
    count: AtomicU64,
    drained: AtomicU64,
    arm: AtomicU32,
    park_elapsed_us: AtomicU64,
    park_budget_us: AtomicU64,
}

impl StopDiagSlot {
    pub const fn new() -> Self {
        Self {
            count: AtomicU64::new(0),
            drained: AtomicU64::new(0),
            arm: AtomicU32::new(0),
            park_elapsed_us: AtomicU64::new(0),
            park_budget_us: AtomicU64::new(0),
        }
    }

    /// Records an authorized stop ignored because a watchdog park is running (RT thread:
    /// atomic stores only).
    pub fn record_park_ignored(&self, arm: ArmId, elapsed_s: f64, budget_s: f64) {
        let to_us = |s: f64| if s.is_finite() && s >= 0.0 { (s * 1_000_000.0).min(u64::MAX as f64) as u64 } else { 0 };
        self.arm.store(u32::from(arm.0), Ordering::Relaxed);
        self.park_elapsed_us.store(to_us(elapsed_s), Ordering::Relaxed);
        self.park_budget_us.store(to_us(budget_s), Ordering::Relaxed);
        self.count.fetch_add(1, Ordering::Release);
    }

    /// Total park-ignored stop events recorded so far.
    pub fn count(&self) -> u64 { self.count.load(Ordering::Acquire) }

    /// Returns newly recorded events since `*seen` and advances `*seen` (off-thread).
    pub fn poll(&self, seen: &mut u64) -> Option<StopDiagRecord> {
        let now = self.count.load(Ordering::Acquire);
        if now == *seen { return None; }
        let delta = now.wrapping_sub(*seen);
        *seen = now;
        Some(StopDiagRecord {
            count: delta,
            arm: ArmId(self.arm.load(Ordering::Relaxed) as u16),
            park_elapsed_us: self.park_elapsed_us.load(Ordering::Relaxed),
            park_budget_us: self.park_budget_us.load(Ordering::Relaxed),
        })
    }

    fn emit_log(rec: &StopDiagRecord) {
        tracing::info!(
            "Stop during a watchdog park (arm {}, elapsed {:.2} s / budget {:.2} s, {} event(s)): the park continues (only a new lease takes it over)",
            rec.arm.0,
            rec.park_elapsed_us as f64 / 1_000_000.0,
            rec.park_budget_us as f64 / 1_000_000.0,
            rec.count,
        );
    }

    /// Off-thread drain with caller-managed cursor: formats and emits `tracing::info!` for
    /// newly recorded events since `*seen`.
    pub fn drain_and_log(&self, seen: &mut u64) -> Option<StopDiagRecord> {
        let rec = self.poll(seen)?;
        Self::emit_log(&rec);
        Some(rec)
    }

    /// Off-thread shared drain called by normal-priority periodic paths (e.g.
    /// `StateHub::refresh` driven by the supervisor dispatcher loop): claims unlogged events
    /// via CAS on `self.drained` so each recorded stop event is emitted once.
    pub fn drain_and_log_once(&self) -> Option<StopDiagRecord> {
        loop {
            let prev = self.drained.load(Ordering::Acquire);
            let now = self.count.load(Ordering::Acquire);
            if now == prev { return None; }
            if self.drained.compare_exchange(prev, now, Ordering::AcqRel, Ordering::Acquire).is_ok() {
                let rec = StopDiagRecord {
                    count: now.wrapping_sub(prev),
                    arm: ArmId(self.arm.load(Ordering::Relaxed) as u16),
                    park_elapsed_us: self.park_elapsed_us.load(Ordering::Relaxed),
                    park_budget_us: self.park_budget_us.load(Ordering::Relaxed),
                };
                Self::emit_log(&rec);
                return Some(rec);
            }
        }
    }
}

impl Default for StopDiagSlot {
    fn default() -> Self { Self::new() }
}

static STOP_DIAG: StopDiagSlot = StopDiagSlot::new();

/// Preallocated diagnostic slot written by [`SupervisorInner::authorized_stop`] until step 1
/// moves it onto `ArmCore` / `ExecutorStats`.
pub fn stop_diag() -> &'static StopDiagSlot { &STOP_DIAG }

/// Lock-free fixed-size diagnostic slot for RT gripper force-cap warnings (0 allocs, 0 tracing
/// calls on `armd-control`; drained and emitted off-thread in `StateHub::refresh`).
pub struct GripperForceDiagSlot {
    unavail_count: AtomicU64,
    unavail_drained: AtomicU64,
    unavail_bus_fresh: std::sync::atomic::AtomicBool,
    unavail_has_torque: std::sync::atomic::AtomicBool,
    cap_trip_count: AtomicU64,
    cap_trip_drained: AtomicU64,
    cap_tau_milli_nm: AtomicU32,
    cap_limit_milli_nm: AtomicU32,
    cap_pos_milli_rad: std::sync::atomic::AtomicI32,
    cap_trips: AtomicU32,
    stream_stop_count: AtomicU64,
    stream_stop_drained: AtomicU64,
    stream_stop_max_trips: AtomicU32,
}

impl GripperForceDiagSlot {
    pub const fn new() -> Self {
        Self {
            unavail_count: AtomicU64::new(0),
            unavail_drained: AtomicU64::new(0),
            unavail_bus_fresh: std::sync::atomic::AtomicBool::new(false),
            unavail_has_torque: std::sync::atomic::AtomicBool::new(false),
            cap_trip_count: AtomicU64::new(0),
            cap_trip_drained: AtomicU64::new(0),
            cap_tau_milli_nm: AtomicU32::new(0),
            cap_limit_milli_nm: AtomicU32::new(0),
            cap_pos_milli_rad: std::sync::atomic::AtomicI32::new(0),
            cap_trips: AtomicU32::new(0),
            stream_stop_count: AtomicU64::new(0),
            stream_stop_drained: AtomicU64::new(0),
            stream_stop_max_trips: AtomicU32::new(0),
        }
    }

    pub fn record_feedback_unavailable(&self, bus_fresh: bool, has_torque: bool) {
        self.unavail_bus_fresh.store(bus_fresh, Ordering::Relaxed);
        self.unavail_has_torque.store(has_torque, Ordering::Relaxed);
        self.unavail_count.fetch_add(1, Ordering::Release);
    }

    pub fn record_cap_trip(&self, tau_abs: f32, limit_nm: f32, pos_rad: f32, trips: u32) {
        let to_u32 = |v: f32| if v.is_finite() && v >= 0.0 { (v * 1000.0).min(u32::MAX as f32) as u32 } else { 0 };
        let to_i32 = |v: f32| if v.is_finite() { (v * 1000.0).clamp(i32::MIN as f32, i32::MAX as f32) as i32 } else { 0 };
        self.cap_tau_milli_nm.store(to_u32(tau_abs), Ordering::Relaxed);
        self.cap_limit_milli_nm.store(to_u32(limit_nm), Ordering::Relaxed);
        self.cap_pos_milli_rad.store(to_i32(pos_rad), Ordering::Relaxed);
        self.cap_trips.store(trips, Ordering::Relaxed);
        self.cap_trip_count.fetch_add(1, Ordering::Release);
    }

    pub fn record_stream_stopped(&self, max_trips: u32) {
        self.stream_stop_max_trips.store(max_trips, Ordering::Relaxed);
        self.stream_stop_count.fetch_add(1, Ordering::Release);
    }

    #[cfg(test)]
    pub fn counts(&self) -> (u64, u64, u64) {
        (
            self.unavail_count.load(Ordering::Acquire),
            self.cap_trip_count.load(Ordering::Acquire),
            self.stream_stop_count.load(Ordering::Acquire),
        )
    }

    pub fn drain_and_log_once(&self) {
        loop {
            let prev = self.unavail_drained.load(Ordering::Acquire);
            let now = self.unavail_count.load(Ordering::Acquire);
            if now == prev { break; }
            if self.unavail_drained.compare_exchange(prev, now, Ordering::AcqRel, Ordering::Acquire).is_ok() {
                tracing::warn!(
                    "Gripper close stopped ({} event(s)): feedback_fresh={} torque_feedback={} while max_close_torque_nm is active; holding",
                    now.wrapping_sub(prev),
                    self.unavail_bus_fresh.load(Ordering::Relaxed),
                    self.unavail_has_torque.load(Ordering::Relaxed),
                );
                break;
            }
        }
        loop {
            let prev = self.cap_trip_drained.load(Ordering::Acquire);
            let now = self.cap_trip_count.load(Ordering::Acquire);
            if now == prev { break; }
            if self.cap_trip_drained.compare_exchange(prev, now, Ordering::AcqRel, Ordering::Acquire).is_ok() {
                tracing::warn!(
                    "Gripper torque cap ({} event(s)): |tau| {:.2} N·m above {:.2} N·m at {:.3} rad (trip {} on this contact); holding",
                    now.wrapping_sub(prev),
                    self.cap_tau_milli_nm.load(Ordering::Relaxed) as f32 / 1000.0,
                    self.cap_limit_milli_nm.load(Ordering::Relaxed) as f32 / 1000.0,
                    self.cap_pos_milli_rad.load(Ordering::Relaxed) as f32 / 1000.0,
                    self.cap_trips.load(Ordering::Relaxed),
                );
                break;
            }
        }
        loop {
            let prev = self.stream_stop_drained.load(Ordering::Acquire);
            let now = self.stream_stop_count.load(Ordering::Acquire);
            if now == prev { break; }
            if self.stream_stop_drained.compare_exchange(prev, now, Ordering::AcqRel, Ordering::Acquire).is_ok() {
                tracing::warn!(
                    "Gripper torque cap tripped {} times on one contact under a stream ({} event(s)); stopping the stream",
                    self.stream_stop_max_trips.load(Ordering::Relaxed),
                    now.wrapping_sub(prev),
                );
                break;
            }
        }
    }
}

impl Default for GripperForceDiagSlot {
    fn default() -> Self { Self::new() }
}

static GRIPPER_FORCE_DIAG: GripperForceDiagSlot = GripperForceDiagSlot::new();

pub(crate) fn gripper_force_diag() -> &'static GripperForceDiagSlot { &GRIPPER_FORCE_DIAG }

/// The protocol code of a delegated error, as a static string without allocating
/// (unknown codes: `REJECTED`).
pub(crate) fn static_code(e: ArmError) -> &'static str {
    use crate::safety::SafetyError as S;
    const KNOWN: &[&str] = &[
        "MOTION_BUSY", "NOT_ARMED", "ESTOP_ACTIVE", "FAULT_ACTIVE", "EPOCH_MISMATCH", "INVALID_COMMAND", "IK_UNREACHABLE",
        "WORKSPACE_VIOLATION", "JOINT_LIMIT", "PLAN_STALE", "LEASE_LOST", "NO_LEASE", "DISCONNECTED", "COLLISION_HOLD", "COMMISSIONING_REQUIRED",
        "INVALID_VALUE", "UNSUPPORTED", "INTERNAL", "FEEDBACK_STALE", "J7_FEEDBACK_STALE", "FEEDBACK_UNAVAILABLE", "CONFIG_BUSY", "ARM_NOT_AT_RESTING_POSE", "TOOL_MISMATCH",
        "MOTION_FORBIDDEN_IN_FAULT_STATE", "TORQUE_OFF_WOULD_DROP", "STOPPED", "TERMINALS_BUSY", "STAMP_REUSED", "HANDOVER_SETTLING", "DISABLE_PENDING", "TRANSITION_BUSY",
        "WRONG_TARGET", "DEADLINE_EXPIRED",
    ];
    match &e {
        ArmError::Motion(m) => m.code(),
        ArmError::Safety(s) => match s {
            S::EStopActive => "ESTOP_ACTIVE",
            S::FaultActive(_) => "FAULT_ACTIVE",
            S::NotArmed(_) => "NOT_ARMED",
            S::EpochMismatch { .. } => "EPOCH_MISMATCH",
            S::CollisionHold(_) => "COLLISION_HOLD",
            S::CommissioningRequired(_) => "COMMISSIONING_REQUIRED",
            S::MotionBusy => "MOTION_BUSY",
            S::HandoverSettling => "HANDOVER_SETTLING",
            S::FeedbackUnavailable => "FEEDBACK_UNAVAILABLE",
            S::WorkspaceViolation { .. } => "WORKSPACE_VIOLATION",
            S::ArmNotAtRestingPose { .. } => "ARM_NOT_AT_RESTING_POSE",
            S::MotionForbiddenInFaultState(_) => "MOTION_FORBIDDEN_IN_FAULT_STATE",
            S::TorqueOffWouldDrop { .. } => "TORQUE_OFF_WOULD_DROP",
            S::Stopped => "STOPPED",
            S::LeaseLost => "LEASE_LOST",
            S::TerminalsBusy => "TERMINALS_BUSY",
            S::StampAlreadyConsumed => "STAMP_REUSED",
            _ => "REJECTED",
        },
        ArmError::Unsupported(_) => "UNSUPPORTED",
        ArmError::WorkspaceRejected { .. } => "WORKSPACE_VIOLATION",
        ArmError::RejectedStatic { code, .. } => code,
        ArmError::Rejected { code, .. } | ArmError::Detailed { code, .. } => {
            KNOWN.iter().find(|k| **k == code.as_str()).copied().unwrap_or("REJECTED")
        }
        ArmError::Trajectory(_) | ArmError::Recording(_) | ArmError::PolicyRevisionMismatch { .. } | ArmError::Internal(_) => "REJECTED",
    }
}

impl SupervisorInner {
    /// The lease valid at `now` (unexpired, unfenced, and owning connection not closed).
    pub(crate) fn lease_valid_at(&self, now: Instant) -> Option<&crate::supervisor::ActiveLease> {
        let fenced = self.effective_fenced_lease_generation();
        if self.active_lease_conn_id != 0 && self.is_connection_closed(self.active_lease_conn_id) {
            return None;
        }
        self.active_lease.as_ref().filter(|l| {
            l.generation > fenced
                && now.saturating_duration_since(l.acquired_at) < self.lease_timeout
        })
    }

    /// The stamp checks every commit shares: registered live connection, identity, deadline,
    /// current authority, and binding `d.stamp.client` to the active lease holder when leases are
    /// enforced (M2).
    fn check_commit(&self, d: &Descriptor, ctx: &CommitCtx) -> Result<(), &'static str> {
        self.check_client_connection_live(d.stamp.client.0)?;
        if let Some(code) = d.stamp.check(ctx.arm, ctx.boot, ctx.now, &self.authority(ctx)) { return Err(code); }
        self.check_descriptor_lease_owner(d.stamp.client.0, d.stamp.authority.lease_generation)?;
        Ok(())
    }

    pub(crate) fn check_commit_for_motion(&self, d: &Descriptor, ctx: &CommitCtx) -> Result<(), &'static str> {
        self.check_commit(d, ctx)
    }

    /// Stop-direction variant of [`Self::check_commit`]: exempt from `stop_generation` checks
    /// so an authorized Stop never blocks a queued torque disable.
    fn check_commit_stop_direction(&self, d: &Descriptor, ctx: &CommitCtx) -> Result<(), &'static str> {
        self.check_client_connection_live(d.stamp.client.0)?;
        if let Some(code) = d.stamp.check_stop_direction(ctx.arm, ctx.boot, ctx.now, &self.authority(ctx)) { return Err(code); }
        self.check_descriptor_lease_owner(d.stamp.client.0, d.stamp.authority.lease_generation)?;
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
        self.sync_stop_generation(ctx.stop_generation);
        let stamp = crate::supervisor::AdmissionStamp::from_broker(
            d.stamp.authority.stop_generation,
            d.stamp.authority.epoch,
            d.stamp.authority.lease_generation,
            d.stamp.request.0,
        );
        self.preflight_admission(stamp).map_err(|e| static_code(ArmError::Safety(e)))?;
        match primitive {
            // a prepared path is installed with its slot's samples (`commit_prepared`)
            Primitive::None | Primitive::Prepared { .. } => Err("INVALID_COMMAND"),
            #[cfg(not(target_arch = "wasm32"))]
            Primitive::Action(_) => Err("INVALID_COMMAND"),
            // macro children (track E): installed for the running macro named by the ticket;
            // the committed action is the parent (its terminal carries the child count)
            Primitive::CartesianPath { .. } | Primitive::Gripper { .. } | Primitive::JointQuinticLimited { .. } | Primitive::Replay { .. } => {
                if primitive.child().is_none() {
                    return Err("UNSUPPORTED");
                }
                let admitted = self.check_admission(stamp).map_err(|e| static_code(ArmError::Safety(e)))?;
                SupervisorInner::commit_macro_child(admitted, primitive, ctx.now).map(|parent| Committed::authorized(d, Some(parent)))
            }
            Primitive::JointQuintic { .. } => {
                // Joint quintic plans must be prepared off armd-control and staged as a
                // `PreparedMotionSlot` (`PlanSlot::motion` via `req::stage_jog`); never plan on RT.
                Err("INVALID_COMMAND")
            }
        }
    }

    fn commit_install_slot(&mut self, d: &Descriptor, slot: &super::requests::motion::PreparedMotionSlot, ctx: &CommitCtx) -> Result<Committed, &'static str> {
        super::requests::motion::commit_install_slot(self, d, slot, ctx)
    }

    fn commit_home(&mut self, d: &Descriptor, slot: &super::requests::motion::PreparedMotionSlot, ctx: &CommitCtx) -> Result<Committed, &'static str> {
        super::requests::motion::commit_home(self, d, slot, ctx)
    }

    fn commit_park(&mut self, d: &Descriptor, slot: &super::requests::motion::PreparedMotionSlot, ctx: &CommitCtx) -> Result<Committed, &'static str> {
        super::requests::motion::commit_park(self, d, slot, ctx)
    }

    fn commit_park_continuation(&mut self, d: &Descriptor, slot: &super::requests::motion::PreparedMotionSlot, ctx: &CommitCtx) -> Result<Committed, &'static str> {
        super::requests::motion::commit_park_continuation(self, d, slot, ctx)
    }

    fn commit_prepared(&mut self, d: &Descriptor, primitive: &Primitive, path: &PreparedPath, ctx: &CommitCtx) -> Result<Committed, &'static str> {
        self.check_motion_commit(d, primitive, ctx)?;
        self.sync_stop_generation(ctx.stop_generation);
        let stamp = crate::supervisor::AdmissionStamp::from_broker(
            d.stamp.authority.stop_generation,
            d.stamp.authority.epoch,
            d.stamp.authority.lease_generation,
            d.stamp.request.0,
        );
        self.preflight_admission(stamp).map_err(|e| static_code(ArmError::Safety(e)))?;
        let Primitive::Prepared { child, dof, start, kind, duration_s, power_scale } = *primitive else {
            return Err("INVALID_COMMAND");
        };
        let n = usize::from(dof);
        if n == 0 || n > MAX_DOF {
            return Err("INVALID_COMMAND");
        }
        let epoch = d.stamp.authority.epoch;
        let committed = |r: Result<u64, ArmError>| r.map(|id| Committed::authorized(d, Some(ActionId(id)))).map_err(static_code);
        match kind {
            PathKind::Cartesian => {
                if !start_matches(&start[..n], &self.commanded_positions) {
                    return Err("PLAN_STALE");
                }
                #[cfg(not(target_arch = "wasm32"))]
                super::requests::actions::check_cartesian_path_planning_rt(
                    self,
                    n,
                    &self.commanded_positions,
                    path.samples(),
                )
                .map_err(|r| r.code)?;
                let joint_path = self.prepared_joint_path(n, path, duration_s, power_scale)?;
                match child {
                    None => {
                        let admitted = self.check_admission(stamp).map_err(|e| static_code(ArmError::Safety(e)))?;
                        committed(SupervisorInner::install_prepared_path(admitted, epoch, joint_path, power_scale.is_some()).map(|(id, _)| id))
                    }
                    Some(spec) => self.commit_child_install(spec, &start[..n], ctx.now, move |this, id| {
                        let admitted = this.check_admission(stamp).map_err(ArmError::from)?;
                        SupervisorInner::install_macro_prepared_path(admitted, id, joint_path, power_scale.is_some()).map(drop)
                    }).map(|parent| Committed::authorized(d, Some(parent))),
                }
            }
            PathKind::Replay { .. } => {
                if !start_matches(&start[..n], &self.commanded_positions) {
                    return Err("PLAN_STALE");
                }
                match child {
                    None => {
                        let _ = self.validate_prepared_replay(n, path, kind, duration_s, epoch)?;
                        let prepared = self.prepared_replay_motion(n, path, kind, duration_s, power_scale, epoch).map_err(static_code)?;
                        let admitted = self.check_admission(stamp).map_err(|e| static_code(ArmError::Safety(e)))?;
                        committed(SupervisorInner::install_prepared_replay(admitted, prepared).map(|(id, _)| id))
                    }
                    Some(spec) => {
                        self.commit_child_checks(&spec, &start[..n])?;
                        let _ = self.validate_prepared_replay(n, path, kind, duration_s, epoch)?;
                        self.commit_child_install(spec, &start[..n], ctx.now, move |this, id| {
                            let prepared = this.prepared_replay_motion(n, path, kind, duration_s, power_scale, epoch)?;
                            let admitted = this.check_admission(stamp).map_err(ArmError::from)?;
                            SupervisorInner::install_macro_prepared_replay(admitted, id, prepared).map(drop)
                        }).map(|parent| Committed::authorized(d, Some(parent)))
                    }
                }
            }
        }
    }

    fn commit_prepared_slot(&mut self, d: &Descriptor, slot: PlanSlotRef<PlanSlot>, ctx: &CommitCtx) -> Result<Committed, &'static str> {
        self.commit_prepared_slot_with_detail(d, slot, ctx).map_err(|r| r.code)
    }

    fn commit_prepared_slot_with_detail(&mut self, d: &Descriptor, slot: PlanSlotRef<PlanSlot>, ctx: &CommitCtx) -> Result<Committed, CommitRefusal> {
        let primitive = slot.primitive;
        self.check_motion_commit(d, &primitive, ctx)?;
        self.sync_stop_generation(ctx.stop_generation);
        let stamp = crate::supervisor::AdmissionStamp::from_broker(
            d.stamp.authority.stop_generation,
            d.stamp.authority.epoch,
            d.stamp.authority.lease_generation,
            d.stamp.request.0,
        );
        self.preflight_admission(stamp)
            .map_err(|e| CommitRefusal::code(static_code(ArmError::Safety(e))))?;
        let Primitive::Prepared { child, dof, start, kind, duration_s, power_scale } = primitive else {
            return Err(CommitRefusal::code("INVALID_COMMAND"));
        };
        let n = usize::from(dof);
        if n == 0 || n > MAX_DOF {
            return Err(CommitRefusal::code("INVALID_COMMAND"));
        }
        let epoch = d.stamp.authority.epoch;
        let committed = |r: Result<u64, ArmError>| {
            r.map(|id| Committed::authorized(d, Some(ActionId(id))))
                .map_err(|e| CommitRefusal::code(static_code(e)))
        };
        match kind {
            PathKind::Cartesian => {
                if !start_matches(&start[..n], &self.commanded_positions) {
                    return Err(CommitRefusal::code("PLAN_STALE"));
                }
                #[cfg(not(target_arch = "wasm32"))]
                super::requests::actions::check_cartesian_path_planning_rt(
                    self,
                    n,
                    &self.commanded_positions,
                    slot.path.samples(),
                )?;
                match child {
                    None => {
                        self.validate_prepared_joint_path(n, &slot.path, duration_s)?;
                        self.check_prepared_fresh_static(epoch)?;
                        let admitted = self
                            .check_admission(stamp)
                            .map_err(|e| CommitRefusal::code(static_code(ArmError::Safety(e))))?;
                        SupervisorInner::install_prepared_path_slot(admitted, epoch, n, slot, duration_s, power_scale)
                            .map(|(id, _)| Committed::authorized(d, Some(ActionId(id))))
                            .map_err(CommitRefusal::code)
                    }
                    Some(spec) => self
                        .commit_child_install(spec, &start[..n], ctx.now, move |this, id| {
                            let admitted = this.check_admission(stamp).map_err(ArmError::from)?;
                            SupervisorInner::install_macro_prepared_path_slot(admitted, id, n, slot, duration_s, power_scale).map(drop)
                        })
                        .map(|parent| Committed::authorized(d, Some(parent)))
                        .map_err(CommitRefusal::code),
                }
            }
            PathKind::Replay { approach_duration_s, approach_power_scale, .. } => {
                if !start_matches(&start[..n], &self.commanded_positions) {
                    return Err(CommitRefusal::code("PLAN_STALE"));
                }
                match child {
                    None => {
                        let checked = self.validate_prepared_replay(n, &slot.path, kind, duration_s, epoch)?;
                        let prepared = self.prepared_replay_from_slot_preplanned(
                            n,
                            &start[..n],
                            slot,
                            checked,
                            duration_s,
                            power_scale,
                            approach_duration_s,
                            approach_power_scale,
                        )?;
                        let admitted = self
                            .check_admission(stamp)
                            .map_err(|e| CommitRefusal::code(static_code(ArmError::Safety(e))))?;
                        committed(SupervisorInner::install_prepared_replay(admitted, prepared).map(|(id, _)| id))
                    }
                    Some(spec) => {
                        self.commit_child_checks(&spec, &start[..n])?;
                        let checked = self.validate_prepared_replay(n, &slot.path, kind, duration_s, epoch)?;
                        let prepared = self.prepared_replay_from_slot_preplanned(
                            n,
                            &start[..n],
                            slot,
                            checked,
                            duration_s,
                            power_scale,
                            approach_duration_s,
                            approach_power_scale,
                        )?;
                        self.commit_child_install(spec, &start[..n], ctx.now, move |this, id| {
                            let admitted = this.check_admission(stamp).map_err(ArmError::from)?;
                            SupervisorInner::install_macro_prepared_replay(admitted, id, prepared).map(drop)
                        })
                        .map(|parent| Committed::authorized(d, Some(parent)))
                        .map_err(CommitRefusal::code)
                    }
                }
            }
        }
    }

    fn commit_immediate(&mut self, d: &Descriptor, op: ImmediateOp, ctx: &CommitCtx) -> Result<Committed, &'static str> {
        if matches!(op, ImmediateOp::Heartbeat) {
            return super::requests::lease::install_broker_heartbeat(self, d, ctx)
                .map(|()| Committed::authorized(d, None));
        }
        match op {
            ImmediateOp::TorqueOff => self.check_commit_stop_direction(d, ctx)?,
            ImmediateOp::TorqueOn => self.check_commit(d, ctx)?,
            ImmediateOp::Heartbeat => unreachable!(),
        }
        match op {
            ImmediateOp::Heartbeat => unreachable!(),
            ImmediateOp::TorqueOn => {
                self.sync_stop_generation(ctx.stop_generation);
                let stamp = crate::supervisor::AdmissionStamp::from_broker_conn(
                    d.stamp.authority.stop_generation,
                    d.stamp.authority.epoch,
                    d.stamp.authority.lease_generation,
                    d.stamp.client.0,
                    d.stamp.request.0,
                );
                let admitted = self
                    .check_admission(stamp)
                    .map_err(|e| static_code(ArmError::Safety(e)))?;
                SupervisorInner::enable_torque_immediate_rt(admitted, ctx.now)?;
            }
            // Stop-direction action: exempt from stop-generation checks and Admitted.
            ImmediateOp::TorqueOff => {
                self.disable_torque_immediate_rt(ctx.now)?;
            }
        }
        Ok(Committed::authorized(d, None))
    }

    fn commit_lease(
        &mut self,
        d: &Descriptor,
        req: super::requests::lease::LeaseRequest,
        ctx: &CommitCtx,
    ) -> Result<super::requests::lease::LeaseOutcome, &'static str> {
        super::requests::lease::commit_lease_descriptor(self, d, req, ctx)
    }

    fn handle_conn_lifecycle(
        &mut self,
        cmd: super::requests::lease::LifecycleCmd,
        now: Instant,
    ) -> super::requests::lease::LifecycleReply {
        super::requests::lease::apply_conn_lifecycle(self, cmd, now)
    }

    fn commit_transition(&mut self, d: &Descriptor, op: TransitionOp, ctx: &CommitCtx) -> Result<Committed, &'static str> {
        match op {
            TransitionOp::TorqueOff => self.check_commit_stop_direction(d, ctx)?,
            TransitionOp::TorqueOn => self.check_commit(d, ctx)?,
        }
        if self.core.transition_pending() || self.torque_transition_pending() { return Err("TRANSITION_BUSY"); }
        // starts the bus owner's handshake (ArmDriver::begin_torque); a refusal before any
        // frame refuses the commit, a later failure is the transition's terminal
        let progress = match op {
            TransitionOp::TorqueOn => {
                self.sync_stop_generation(ctx.stop_generation);
                let stamp = crate::supervisor::AdmissionStamp::from_broker_conn(
                    d.stamp.authority.stop_generation,
                    d.stamp.authority.epoch,
                    d.stamp.authority.lease_generation,
                    d.stamp.client.0,
                    d.stamp.request.0,
                );
                let admitted = self
                    .check_admission(stamp)
                    .map_err(|e| static_code(ArmError::Safety(e)))?;
                super::requests::lease::install_torque_on_transition(admitted)?
            }
            // Stop-direction action: exempt from stop-generation checks and Admitted.
            TransitionOp::TorqueOff => super::requests::lease::install_torque_off_transition(self)?,
        };
        // the next service is the next poll (the begin's wake is on the driver's clock)
        self.core.begin_transition(progress.map(|p| match p { TorqueProgress::Pending { .. } => TorqueProgress::Pending { wake: ctx.now }, done => done }));
        Ok(Committed::authorized(d, Some(ActionId(d.stamp.request.0))))
    }

    fn commit_transition_for_release(
        &mut self,
        d: &Descriptor,
        op: TransitionOp,
        ctx: &CommitCtx,
        release_deadline: Instant,
    ) -> Result<Committed, &'static str> {
        let entry_wall = Instant::now();
        let mut live_now = || ctx.now + Instant::now().saturating_duration_since(entry_wall);
        self.commit_transition_for_release_with_clock(d, op, ctx, release_deadline, &mut live_now)
    }

    fn release_tx_budget_limit(&self) -> u16 {
        SupervisorInner::release_tx_budget_limit(self)
    }

    fn commit_transition_with_budget(
        &mut self,
        d: &Descriptor,
        op: TransitionOp,
        ctx: &CommitCtx,
        budget: &mut ReleaseBudget,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> Result<Committed, &'static str> {
        let before = self.sync_driver_budget_before(budget);
        let res = self.commit_transition_for_release_with_clock(d, op, ctx, budget.deadline(), now_fn);
        self.charge_driver_budget_after(budget, before, 0, false, 0);
        res
    }

    fn commit_transition_for_release_with_clock(
        &mut self,
        d: &Descriptor,
        op: TransitionOp,
        ctx: &CommitCtx,
        release_deadline: Instant,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> Result<Committed, &'static str> {
        match self.install_driver_tx_release_cap_with_clock(release_deadline, now_fn) {
            crate::supervisor::DriverTxCapOutcome::Expired => Err("BUDGET_EXPIRED"),
            outcome => {
                let res = self.commit_transition(d, op, ctx);
                if outcome == crate::supervisor::DriverTxCapOutcome::Installed {
                    self.clear_driver_tx_release_cap();
                }
                res
            }
        }
    }

    fn consume_target(&mut self, frame: &TargetFrame, ctx: &CommitCtx) -> Result<(), Refusal> {
        if self.internal_fault_lane.is_fenced() {
            return Err(Refusal::code("INTERNAL_FAULT"));
        }
        // track D: identity, deadline, authority, stream, policy generation, input / camera
        // ages, config hold, then the servo / policy step checks (supervisor::streams::targets)
        self.consume_target_frame(frame, ctx)
    }

    fn consume_target_with_validity(
        &mut self,
        frame: &TargetFrame,
        validity: super::mailbox::PolicyValidityBounds,
        ctx: &CommitCtx,
    ) -> Result<(), Refusal> {
        if self.internal_fault_lane.is_fenced() {
            return Err(Refusal::code("INTERNAL_FAULT"));
        }
        self.consume_target_frame_with_validity(frame, validity, ctx)
    }

    fn on_policy_refusal_cascade(&mut self, frame: &TargetFrame) {
        self.policy_target_refused("POLICY_REVOKED", frame);
    }

    fn apply_estop(&mut self) -> bool { self.consume_pending_estop() }

    fn sync_disconnect_fence(&mut self, now: Instant) -> bool {
        self.sync_nonblocking_disconnect_fence(now)
    }

    fn has_pending_servo_closes(&self) -> bool {
        self.pending_servo_close_len > 0
    }

    fn drain_servo_closes(&mut self, each: &mut dyn FnMut(u64)) {
        self.drain_pending_servo_closes(each);
    }

    fn has_pending_policy_close(&self) -> bool {
        #[cfg(feature = "policy-runtime")]
        {
            self.has_pending_policy_close()
        }
        #[cfg(not(feature = "policy-runtime"))]
        {
            false
        }
    }

    fn has_pending_urgent(&self) -> bool {
        self.urgent_stop_unapplied() || self.disconnect_fence_blocks_output()
    }

    fn flush_policy_close_watermark(&mut self) -> u64 {
        #[cfg(feature = "policy-runtime")]
        {
            self.flush_pending_policy_close_watermark()
        }
        #[cfg(not(feature = "policy-runtime"))]
        {
            0
        }
    }

    // ends client motion; a running watchdog park belongs to the lease watchdog and continues
    fn apply_stop(&mut self) { self.authorized_stop(); }

    fn apply_stop_at(&mut self, generation: u64) {
        self.authorized_stop();
        self.sync_stop_generation(generation);
    }

    fn take_aborted_transition(&mut self) -> Option<&'static str> {
        if self.torque_transition.is_none() {
            self.core.take_aborted_transition()
        } else {
            None
        }
    }

    fn handle_urgent_stop(
        &mut self,
        req: &UrgentStop,
        stops: &StopRequests,
        now: Instant,
    ) -> UrgentStopOutcome {
        let epoch = self.safety.epoch_id();
        if req.lease_generation == 0
            || !self.validate_lease_at(req.connection.as_str(), req.lease_generation, now)
        {
            return UrgentStopOutcome::LeaseLost { epoch };
        }
        if req.epoch != Some(epoch) {
            return UrgentStopOutcome::EpochMismatch { epoch };
        }
        if !req.request_id_valid {
            return UrgentStopOutcome::InvalidRequestId;
        }
        if let Some(message) = req.body_error {
            return UrgentStopOutcome::InvalidBody { message, epoch };
        }
        let stop_generation = stops.request();
        let applied_to_motion = self.authorized_stop();
        self.sync_stop_generation(stop_generation);
        stops.mark_applied(stop_generation);
        UrgentStopOutcome::Applied {
            stop_generation,
            applied_to_motion,
            epoch: self.safety.epoch_id(),
            safety_state: self.safety.state_arc(),
            is_torqued: self.is_torqued,
            control_mode_gravity_comp: self.gravity_comp_active(),
        }
    }

    fn action_outcome(&self, action: ActionId) -> Option<Outcome> {
        // a macro's terminal record outlives its action record (finish_action, a newer action)
        if let Some(m) = self.core.last_macro().filter(|m| m.action == action) { return Some(m.outcome); }
        let Some(a) = self.last_action.as_ref().filter(|a| a.id == action.0) else { return Some(Outcome::Superseded); };
        outcome_of(&a.state, a.code.as_deref())
    }

    fn action_children(&self, action: ActionId) -> u32 { self.core.last_macro().filter(|m| m.action == action).map_or(0, |m| m.children) }

    fn poll_hardware(&mut self, budget: &ServiceBudget) -> HardwarePoll {
        self.poll_hardware_with_release_deadline(budget, None)
    }

    fn poll_hardware_for_release(&mut self, budget: &ServiceBudget, release_deadline: Instant) -> HardwarePoll {
        self.poll_hardware_with_release_deadline(budget, Some(release_deadline))
    }

    fn poll_hardware_for_release_with_clock(
        &mut self,
        budget: &ServiceBudget,
        release_deadline: Instant,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> HardwarePoll {
        self.poll_hardware_with_release_deadline_with_clock(budget, Some(release_deadline), now_fn)
    }

    fn poll_hardware_with_budget(
        &mut self,
        budget: &mut ReleaseBudget,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> HardwarePoll {
        self.poll_hardware_with_budget_and_clock(budget, now_fn)
    }

    fn poll_transition_wake_with_budget(
        &mut self,
        budget: &mut ReleaseBudget,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> HardwarePoll {
        let now = now_fn();
        if budget.deadline_exceeded(now) {
            return HardwarePoll {
                driverless: self.driver.is_none(),
                budget_exhausted: true,
                torque: self.core.poll_transition(),
                ..HardwarePoll::default()
            };
        }
        // RX exhaustion skips reading (`sb.max_rx_frames == 0`), but MUST NOT suppress
        // TX-capable transition work (inline watchdog holds and protective disable retries).
        let rx_was_zero = budget.rx_remaining() == 0;
        if rx_was_zero {
            budget.mark_rx_exhausted();
        }
        match self.install_driver_tx_release_cap_with_clock(budget.deadline(), now_fn) {
            crate::supervisor::DriverTxCapOutcome::Expired => HardwarePoll {
                driverless: self.driver.is_none(),
                budget_exhausted: true,
                torque: self.core.poll_transition(),
                ..HardwarePoll::default()
            },
            outcome => {
                if let Some(d) = self.driver.as_mut() {
                    let _ = d.try_with_mut(|drv| drv.begin_wake_tx_pass());
                }
                let before = self.sync_driver_budget_before(budget);
                let mut sb = budget.poll_service_budget(now);
                if before.is_none() {
                    sb.max_rx_frames = 0;
                }
                self.arm_conservative_cap_if_contended(before);
                let mut poll = self.poll_transition_wake_with_clock(&sb, now_fn);
                let (cons_rx, cons_tx) = self.take_conservative_step_deltas();
                self.charge_driver_budget_after(
                    budget,
                    before,
                    poll.rx_frames.max(cons_rx),
                    poll.budget_exhausted || rx_was_zero,
                    cons_tx,
                );
                poll.budget_exhausted |= budget.rx_exhausted();
                if outcome == crate::supervisor::DriverTxCapOutcome::Installed {
                    self.clear_driver_tx_release_cap();
                }
                poll
            }
        }
    }

    fn poll_transition_wake_for_release(
        &mut self,
        budget: &ServiceBudget,
        release_deadline: Instant,
    ) -> HardwarePoll {
        let entry_wall = Instant::now();
        let mut live_now = || budget.now + Instant::now().saturating_duration_since(entry_wall);
        self.poll_transition_wake_for_release_with_clock(budget, release_deadline, &mut live_now)
    }

    fn poll_transition_wake_for_release_with_clock(
        &mut self,
        budget: &ServiceBudget,
        release_deadline: Instant,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> HardwarePoll {
        match self.install_driver_tx_release_cap_with_clock(release_deadline, now_fn) {
            crate::supervisor::DriverTxCapOutcome::Expired => HardwarePoll {
                driverless: self.driver.is_none(),
                budget_exhausted: true,
                torque: self.core.poll_transition(),
                ..HardwarePoll::default()
            },
            outcome => {
                // Window 2 sub-release wake: open a fresh TX pass bound by `release_deadline`
                // without clearing this release's coalescing ledger (`release_ctrl_tx_mask`,
                // `refresh_scheduler`, `release_recovery_tx_count`), and mark that Step 5 normal
                // output will NOT run again in this release.
                if let Some(d) = self.driver.as_mut() {
                    let _ = d.try_with_mut(|drv| drv.begin_wake_tx_pass());
                }
                let poll = self.poll_transition_wake_with_clock(budget, now_fn);
                if outcome == crate::supervisor::DriverTxCapOutcome::Installed {
                    self.clear_driver_tx_release_cap();
                }
                poll
            }
        }
    }

    fn poll_transition_wake(&mut self, budget: &ServiceBudget) -> HardwarePoll {
        let entry_wall = Instant::now();
        let mut live_now = || budget.now + Instant::now().saturating_duration_since(entry_wall);
        self.poll_transition_wake_with_clock(budget, &mut live_now)
    }

    fn poll_transition_wake_with_clock(
        &mut self,
        budget: &ServiceBudget,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> HardwarePoll {
        self.last_clock_now = Some(budget.now);
        #[cfg(not(target_arch = "wasm32"))]
        self.torque_shadow.enqueue_at(
            budget.now,
            crate::torque::shadow::ShadowInputKind::TransitionWake {
                live_intent: self.shadow_live_intent(),
                live_is_torqued: self.is_torqued,
                live_stopped: self.safety.state().is_stopped(),
            },
        );
        let trans = self.service_torque_transition_with_clock(budget, now_fn);
        let mut park_wake = None;
        let mut park_state_changed = false;
        let (rx_frames, trans_exhausted) = match trans {
            Some(tp) => {
                if tp.broker_owned {
                    self.core.begin_transition(tp.progress);
                } else {
                    match tp.progress {
                        Ok(TorqueProgress::Pending { wake }) => park_wake = Some(wake),
                        Ok(TorqueProgress::Done) | Err(_) => park_state_changed = true,
                    }
                }
                tp.rx.unwrap_or((0, false))
            }
            None => self
                .service_background_disable_with_clock(budget, now_fn)
                .unwrap_or((0, false)),
        };
        let budget_exhausted = budget.now > budget.deadline || trans_exhausted;
        let Some(driver) = self.driver.as_ref() else {
            return HardwarePoll {
                driverless: true,
                rx_frames,
                budget_exhausted,
                torque: self.core.poll_transition(),
                park_wake,
                park_state_changed,
                ..HardwarePoll::default()
            };
        };
        let sample = |d: &dyn crate::driver::ArmDriver, ages: &mut Vec<Option<Duration>>| {
            d.feedback_ages_into(ages);
            let oldest = ages.iter().copied().try_fold(Duration::ZERO, |m, a| a.map(|a| m.max(a)));
            let wd_rem = d
                .next_watchdog_hold_wake()
                .map(|w| w.saturating_duration_since(d.clock_now()));
            (d.is_feedback_valid(), d.enable_state(), oldest, wd_rem)
        };
        let (feedback_valid, enable_state, oldest, next_wd_rem) = match driver.try_with_ref(|d| sample(d, &mut self.scratch.ages)) {
            Ok(v) => v,
            Err(crate::supervisor_lock::DriverTryError::WouldBlock) => {
                (false, crate::driver::EnableState::Unknown, None, None)
            }
            Err(crate::supervisor_lock::DriverTryError::Poisoned) => {
                driver.with_ref_recover(|d| sample(d, &mut self.scratch.ages))
            }
        };
        let torque_poll = self.core.poll_transition();
        let urgent_wake = if torque_poll.is_some() || park_wake.is_some() {
            let owner_now = now_fn();
            next_wd_rem.map(|rem| owner_now + rem)
        } else {
            None
        };
        HardwarePoll {
            driverless: false,
            feedback_valid,
            enable_state: Some(enable_state),
            oldest_feedback_age: oldest,
            rx_frames,
            budget_exhausted,
            torque: torque_poll,
            park_wake,
            park_state_changed,
            urgent_wake,
        }
    }

    fn advance(&mut self, dt: f64, generation: u64) {
        drop(self.tick_publish(dt, generation, false));
    }

    fn advance_output(&mut self, dt: f64, _generation: u64) -> f64 {
        self.tick_control(dt)
    }

    fn advance_output_with_deadline(
        &mut self,
        dt: f64,
        _generation: u64,
        now: Instant,
        release_deadline: Instant,
    ) -> f64 {
        self.tick_control_with_deadline(dt, now, release_deadline)
    }

    fn advance_output_with_clock(
        &mut self,
        dt: f64,
        _generation: u64,
        now_fn: &mut dyn FnMut() -> Instant,
        release_deadline: Instant,
    ) -> f64 {
        self.tick_control_with_clock(dt, now_fn, release_deadline)
    }

    fn advance_output_with_budget(
        &mut self,
        dt: f64,
        _generation: u64,
        budget: &mut ReleaseBudget,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> f64 {
        let before = self.sync_driver_budget_before(budget);
        let ts = self.tick_control_with_clock(dt, now_fn, budget.deadline());
        self.charge_driver_budget_after(budget, before, 0, false, 0);
        ts
    }

    fn post_output_urgent_wake(
        &self,
        transition_active: bool,
        owner_now: Instant,
        _step2_urgent: Option<Instant>,
    ) -> Option<Instant> {
        if !transition_active {
            return None;
        }
        let driver = self.driver.as_ref()?;
        driver
            .try_with_ref_recover(|d| {
                d.next_watchdog_hold_wake()
                    .map(|w| owner_now + w.saturating_duration_since(d.clock_now()))
            })
            .flatten()
    }

    fn publish(&mut self, timestamp: f64, generation: u64) {
        drop(self.publish_at(timestamp, generation, false));
    }

    fn now_timestamp(&self) -> f64 {
        self.now_s()
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn commit_action(
        &mut self,
        d: &Descriptor,
        cmd: super::requests::actions::ActionCommand,
        slot: Option<PlanSlotRef<PlanSlot>>,
        ctx: &CommitCtx,
    ) -> Result<Committed, CommitRefusal> {
        super::requests::actions::commit_action(self, d, cmd, slot, ctx)
    }

    #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
    fn commit_policy(
        &mut self,
        d: &Descriptor,
        cmd: super::requests::policy::PolicyCommand,
        ctx: &CommitCtx,
    ) -> Result<Committed, &'static str> {
        super::requests::policy::commit_policy(self, d, cmd, ctx)
    }

    #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
    fn finish_policy_command(&mut self, req_id: u64) {
        self.policy_cancel_slots.mark_policy_req_completed(req_id);
    }

    fn macro_child_running(&self, parent: ActionId) -> bool {
        SupervisorInner::macro_child_running(self, parent.0)
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn attach_closed_bitset(&mut self, bitset: std::sync::Arc<super::requests::actions::ClosedBitset>) {
        SupervisorInner::attach_closed_bitset(self, bitset);
    }
    fn set_boot_id(&mut self, boot: BootId) {
        self.boot_id = boot;
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
        if p.stamp.boot.is_zero() || ctx.boot.is_zero() {
            return Reserved::Refused { code: "WRONG_TARGET", current };
        }
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
/// after construction. Backed by [`crate::transport::MockClock`] so a test can drive both the
/// executor and a mock CAN transport from ONE shared clock.
#[derive(Debug, Clone)]
pub struct VirtualClock {
    clock: crate::transport::MockClock,
}

impl VirtualClock {
    /// A clock starting at `base`.
    pub fn starting_at(base: Instant) -> Self {
        Self { clock: crate::transport::MockClock::starting_at(base) }
    }
    /// Wraps an existing [`crate::transport::MockClock`] so executor and driver share ONE clock.
    pub fn from_mock_clock(clock: crate::transport::MockClock) -> Self {
        Self { clock }
    }
    /// Returns a clone of the shared [`crate::transport::MockClock`].
    pub fn as_mock_clock(&self) -> crate::transport::MockClock {
        self.clock.clone()
    }
    pub fn base(&self) -> Instant { self.clock.base() }
    pub fn now(&self) -> Instant { self.clock.now() }
    pub fn advance(&self, d: Duration) { self.clock.advance(d); }
}

impl crate::transport::DriverClock for VirtualClock {
    fn clock_id(&self) -> u64 { self.clock.clock_id() }
    fn now(&self) -> Instant { self.clock.now() }
    fn sleep(&self, d: Duration) { self.clock.sleep(d); }
    fn yield_now(&self) { self.clock.yield_now(); }
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
enum ActiveSlot {
    Standalone(Installed),
    #[cfg(not(target_arch = "wasm32"))]
    Macro {
        parent: Installed,
        child_plan: Option<super::broker::PlanHandle>,
    },
}

impl ActiveSlot {
    fn action(&self) -> ActionId {
        match self {
            Self::Standalone(i) => i.action(),
            #[cfg(not(target_arch = "wasm32"))]
            Self::Macro { parent, .. } => parent.action(),
        }
    }

    fn into_parts(self) -> (Installed, Option<super::broker::PlanHandle>) {
        match self {
            Self::Standalone(i) => (i, None),
            #[cfg(not(target_arch = "wasm32"))]
            Self::Macro { parent, child_plan } => (parent, child_plan),
        }
    }
}

pub struct ReleaseEngine {
    intake: RtIntake<PlanSlot>,
    targets: TargetReader,
    stop: StopRequests,
    urgent_stops: Option<UrgentStopIntake>,
    urgent_replies_flushed: bool,
    conn_lifecycle: Option<super::requests::lease::ConnLifecycleIntake>,
    lifecycle_replies_flushed: bool,
    eviction_tx: Option<rtrb::Producer<(super::requests::lease::ConnectionHandle, u64)>>,
    servo_close_tx: Option<rtrb::Producer<u64>>,
    servo_closes_flushed: bool,
    active: Option<ActiveSlot>,
    /// The driver transition in flight.
    transition: Option<Installed>,
    releases: u64,
    /// Single per-release mutable resource ledger shared across Step 2 poll, Step 2b sessions,
    /// Step 3 transition commit, Step 5 output, and Window 2 sub-release wakes.
    release_budget: Option<ReleaseBudget>,
    /// Per-release RX frame allowance cap (`RX_FRAME_BUDGET` by default; tests may tighten).
    rx_budget_limit: u16,
    /// Optional per-release TX attempt allowance cap override (`None` queries `core.release_tx_budget_limit()`).
    tx_budget_limit: Option<u16>,
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
                urgent_stops: None,
                urgent_replies_flushed: false,
                conn_lifecycle: None,
                lifecycle_replies_flushed: false,
                eviction_tx: None,
                servo_close_tx: None,
                servo_closes_flushed: false,
                active: None,
                transition: None,
                releases: 0,
                release_budget: None,
                rx_budget_limit: RX_FRAME_BUDGET,
                tx_budget_limit: None,
                hook: None,
            },
            broker,
            tw,
            stop,
        )
    }

    /// Configures per-release RX frame and optional TX attempt ledger caps.
    pub fn set_release_budget_limits(&mut self, rx_limit: u16, tx_limit: Option<u16>) {
        self.rx_budget_limit = rx_limit;
        self.tx_budget_limit = tx_limit;
    }

    /// Snapshot of the current / most recent release's [`ReleaseBudget`] ledger.
    pub fn last_release_budget(&self) -> Option<ReleaseBudget> {
        self.release_budget
    }

    /// Attaches the dedicated bounded urgent-stop intake drained at Step 1, every
    /// checkpoint (`BeforeCommit`, `BeforeTarget`, `BeforeOutput`), and sub-release wakes.
    pub fn attach_urgent_stops(&mut self, intake: UrgentStopIntake) {
        self.urgent_stops = Some(intake);
    }

    /// Attaches the dedicated priority connection lifecycle intake (`Open` / `Disconnect`),
    /// drained at Step 1, every checkpoint, and sub-release wakes, with replies flushed after Step 6 publish.
    pub fn attach_conn_lifecycle(&mut self, intake: super::requests::lease::ConnLifecycleIntake) {
        self.conn_lifecycle = Some(intake);
    }

    /// Attaches the bounded post-publication lease eviction notification ring (0 RT allocs) so
    /// committed preemptions deliver `lease_lost(reason: "preempted")` to the evictee even if the
    /// preempting requester disconnects before receiving its own `Reply::Lease`.
    pub fn attach_eviction_tx(
        &mut self,
        tx: rtrb::Producer<(super::requests::lease::ConnectionHandle, u64)>,
    ) {
        self.eviction_tx = Some(tx);
    }

    /// Attaches the bounded post-publication servo stream close ring (0 RT allocs) so every
    /// lease revocation (release, preemption, expiry, disconnect) dispatches `close_servo(old_gen)`
    /// off-thread on `armd-dispatch`.
    pub fn attach_servo_close_tx(&mut self, tx: rtrb::Producer<u64>) {
        self.servo_close_tx = Some(tx);
    }

    /// Flushes any displaced lease generations recorded on `core` into `servo_close_tx` strictly
    /// AFTER publication (0 RT allocs).
    pub fn flush_servo_closes<O: CoreOps>(&mut self, core: &mut O) -> bool {
        let mut pushed = false;
        if let Some(tx) = self.servo_close_tx.as_mut() {
            core.drain_servo_closes(&mut |gen| {
                let _ = tx.push(gen);
                pushed = true;
            });
        } else {
            core.drain_servo_closes(&mut |_| {});
        }
        if pushed {
            self.servo_closes_flushed = true;
        }
        pushed
    }

    pub fn take_servo_closes_flushed(&mut self) -> bool {
        std::mem::take(&mut self.servo_closes_flushed)
    }

    pub fn has_pending_lifecycle_replies(&self) -> bool {
        self.conn_lifecycle
            .as_ref()
            .is_some_and(super::requests::lease::ConnLifecycleIntake::has_pending_replies)
    }

    pub fn flush_lifecycle_replies(&mut self) -> bool {
        self.conn_lifecycle
            .as_mut()
            .is_some_and(super::requests::lease::ConnLifecycleIntake::flush_replies)
    }

    pub fn take_lifecycle_replies_flushed(&mut self) -> bool {
        std::mem::take(&mut self.lifecycle_replies_flushed)
    }

    pub fn drain_conn_lifecycle_stopped(&mut self, flush_existing: bool) -> bool {
        self.conn_lifecycle
            .as_mut()
            .is_some_and(|intake| intake.drain_stopped(flush_existing))
    }

    pub fn has_pending_urgent_stop_replies(&self) -> bool {
        self.urgent_stops
            .as_ref()
            .is_some_and(UrgentStopIntake::has_pending_replies)
    }

    pub fn has_applied_urgent_stop_pending(&self) -> bool {
        self.urgent_stops
            .as_ref()
            .is_some_and(UrgentStopIntake::has_applied_pending)
    }

    /// True when NEW incoming urgent E-stop, stop, or connection-lifecycle requests require
    /// immediate service without waiting for optional-work margins. Intentionally excludes
    /// already-staged outbound replies waiting on a full reply ring so blocked consumers cannot
    /// exhaust the Window 2 wake quota.
    pub fn has_pending_urgent<O: CoreOps>(&self, core: &O) -> bool {
        self.stop.current() != self.stop.applied()
            || self.urgent_stops.as_ref().is_some_and(UrgentStopIntake::has_pending_requests)
            || self.conn_lifecycle.as_ref().is_some_and(super::requests::lease::ConnLifecycleIntake::has_pending_requests)
            || core.has_pending_urgent()
    }

    pub fn flush_urgent_stop_replies(&mut self) -> bool {
        self.urgent_stops
            .as_mut()
            .is_some_and(UrgentStopIntake::flush_replies)
    }

    /// Returns true if urgent-stop replies were flushed since the last call (used by the
    /// executor to wake `StateHub` even on refusal-only wakes or `CommitShutdown`).
    pub fn take_urgent_replies_flushed(&mut self) -> bool {
        std::mem::take(&mut self.urgent_replies_flushed)
    }

    pub fn drain_urgent_stops_stopped(&mut self, flush_existing: bool) -> bool {
        self.urgent_stops
            .as_mut()
            .is_some_and(|intake| intake.drain_stopped(flush_existing))
    }

    pub fn boot(&self) -> BootId { self.intake.boot() }
    pub fn releases(&self) -> u64 { self.releases }
    pub fn active(&self) -> Option<ActionId> { self.active.as_ref().map(ActiveSlot::action) }
    pub fn transition(&self) -> Option<ActionId> { self.transition.as_ref().map(Installed::action) }
    pub fn reply_overflow(&self) -> u64 { self.intake.reply_overflow() }
    pub fn stop_requests(&self) -> &StopRequests { &self.stop }

    pub fn ctx(&self, now: Instant) -> CommitCtx {
        CommitCtx { arm: self.intake.arm(), boot: self.intake.boot(), now, stop_generation: self.stop.current() }
    }

    pub fn on_checkpoint(&mut self, hook: impl FnMut(Checkpoint) + Send + 'static) { self.hook = Some(Box::new(hook)); }

    /// Applies an authorized stop requested since the last one applied; true when it did.
    pub fn apply_pending_stop<O: CoreOps>(&mut self, core: &mut O) -> bool {
        let generation = self.stop.current();
        if generation == self.stop.applied() { return false; }
        core.apply_stop_at(generation);
        self.stop.mark_applied(generation);
        true
    }

    /// Applies any pending urgent E-stop, priority connection lifecycle (`Open`/`Disconnect`),
    /// bounded urgent `Stop` handoffs (authorized against the owner's lease at `now`), and/or
    /// direct `StopRequests`; true when any urgent stop applied.
    pub fn apply_pending_urgent_at<O: CoreOps>(&mut self, core: &mut O, now: Instant) -> bool {
        self.apply_pending_urgent_detailed(core, now).0
    }

    /// Like [`Self::apply_pending_urgent_at`], returning `(stop_applied, any_urgent_consumed)` where
    /// `any_urgent_consumed` also covers connection lifecycle (`Open`/`Disconnect`) and disconnect-fence
    /// transitions so Window-2 wakes suppress CAN polling whenever any urgent intake was consumed.
    fn apply_pending_urgent_detailed<O: CoreOps>(&mut self, core: &mut O, now: Instant) -> (bool, bool) {
        let estop = core.apply_estop();
        let lifecycle = self.drain_conn_lifecycle_at(core, now);
        let fence = core.sync_disconnect_fence(now);
        let urgent_handoff = if let Some(intake) = self.urgent_stops.as_mut() {
            let stops = &self.stop;
            intake.drain_requests(|req| core.handle_urgent_stop(req, stops, now))
        } else {
            false
        };
        let stop = self.apply_pending_stop(core);
        let stopped = estop || urgent_handoff || stop;
        let consumed_any = stopped || lifecycle || fence;
        (stopped, consumed_any)
    }

    /// Drains pending priority connection lifecycle commands (`Open` / `Disconnect`) at `now`.
    fn drain_conn_lifecycle_at<O: CoreOps>(&mut self, core: &mut O, now: Instant) -> bool {
        if let Some(intake) = self.conn_lifecycle.as_mut() {
            intake.drain_requests(|cmd| core.handle_conn_lifecycle(cmd, now))
        } else {
            false
        }
    }

    /// Applies any pending urgent E-stop and/or authorized-stop requests at `Instant::now()`.
    pub fn apply_pending_urgent<O: CoreOps>(&mut self, core: &mut O) -> bool {
        self.apply_pending_urgent_at(core, Instant::now())
    }

    fn checkpoint<O: CoreOps>(
        &mut self,
        core: &mut O,
        at: Checkpoint,
        now_fn: &mut impl FnMut() -> Instant,
        report: &mut ReleaseReport,
    ) -> CommitCtx {
        if let Some(hook) = self.hook.as_mut() { hook(at); }
        let now = now_fn();
        if self.apply_pending_urgent_at(core, now) { report.stopped = true; }
        self.ctx(now)
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

    /// Consumes a mid-release aborted transition (`Err(code)`) at the common pre-publish boundary
    /// so Step 6 publishes `torque_transition_pending == false` while Step 7 emits the terminal
    /// strictly AFTER publication and strictly BEFORE lifecycle or replacement-lease replies.
    fn take_aborted_transition<O: CoreOps>(
        &mut self,
        core: &mut O,
        report: &mut ReleaseReport,
    ) -> Option<(Installed, Outcome)> {
        if self.transition.is_some() {
            if let Some(code) = core.take_aborted_transition() {
                return self.transition.take().map(|t| {
                    let outcome = Outcome::Failed(code);
                    report.transition_ended = Some((t.action(), outcome));
                    (t, outcome)
                });
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
        now_fn: impl FnMut() -> Instant,
        dt: f64,
        release_deadline: Instant,
        publish_generation: u64,
        mut pre_commit: impl FnMut(&mut O, &CommitCtx) -> bool,
    ) -> ReleaseReport {
        self.step_with_pre_commit_budget(
            core,
            now_fn,
            dt,
            release_deadline,
            publish_generation,
            |core, ctx, _budget| pre_commit(core, ctx),
        )
    }

    /// Runs one full release on `core`, owning ONE mutable [`ReleaseBudget`] ledger threaded
    /// through Step 2 hardware poll, Step 2b `pre_commit` (maintenance + sessions), Step 3
    /// transition commit, Step 5 control output, and retained for Window 2 sub-release wakes.
    pub fn step_with_pre_commit_budget<O: CoreOps>(
        &mut self,
        core: &mut O,
        mut now_fn: impl FnMut() -> Instant,
        dt: f64,
        release_deadline: Instant,
        publish_generation: u64,
        mut pre_commit: impl FnMut(&mut O, &CommitCtx, &mut ReleaseBudget) -> bool,
    ) -> ReleaseReport {
        self.releases += 1;
        let mut report = ReleaseReport { release: self.releases, ..ReleaseReport::default() };
        let release_started = now_fn();
        let core_tx_limit = core.release_tx_budget_limit();
        let tx_limit = self
            .tx_budget_limit
            .map_or(core_tx_limit, |lim| lim.min(core_tx_limit));
        let mut ledger = ReleaseBudget::new(
            self.releases,
            release_started,
            release_deadline,
            self.rx_budget_limit,
            tx_limit,
        );
        // 1. Urgent requests (E-stop + urgent Stop handoff + authorized stop), independent of command ring capacity
        if self.apply_pending_urgent_at(core, now_fn()) { report.stopped = true; }
        // 2. Single budgeted hardware poll drawing from `ledger` (frees `self.transition`, defers
        //    terminal delivery until after Step 6 publish).
        report.poll = core.poll_hardware_with_budget(&mut ledger, &mut now_fn);
        let finished_transition = self.take_finished_transition(&mut report);
        // 2b. Owner persistence and bounded maintenance/sessions run strictly after Step 2 feedback
        // intake and strictly before Step 3–6 commit, output, and publication, sharing `ledger`.
        let pre_ctx = self.ctx(now_fn());
        let stopping = pre_commit(core, &pre_ctx, &mut ledger);
        if stopping {
            self.release_budget = Some(ledger);
            let ended_transition = finished_transition
                .or_else(|| self.take_aborted_transition(core, &mut report));
            // Single publication boundary: publish any Step 1-2b state changes (such as an E-stop
            // or finished/aborted transition) strictly BEFORE delivering terminals, urgent stop
            // replies, lifecycle replies, maintenance replies, or E-stop acknowledgements.
            let ts = core.now_timestamp();
            core.publish(ts, publish_generation);
            if let Some((t, outcome)) = ended_transition {
                self.intake.terminal(t, outcome);
            }
            if self.flush_urgent_stop_replies() {
                self.urgent_replies_flushed = true;
            }
            if self.flush_lifecycle_replies() {
                self.lifecycle_replies_flushed = true;
            }
            let _ = self.flush_servo_closes(core);
        if core.flush_policy_close_watermark() != 0 {
            self.servo_closes_flushed = true;
        }
            return report;
        }
        // 3. Urgent recheck + at most one ordinary command (replies deferred until strictly AFTER Step 6 publish)
        self.intake.begin_release(self.releases);
        let ctx = self.checkpoint(core, Checkpoint::BeforeCommit, &mut now_fn, &mut report);
        let mut superseded_and_ack: Option<(Installed, Outcome, u32, super::broker::Reply)> = None;
        let mut deferred_step3_reply: Option<super::broker::Reply> = None;
        let mut deferred_eviction: Option<(super::requests::lease::ConnectionHandle, u64)> = None;
        let mut retire_after_take: Option<super::broker::PlanHandle> = None;
        let mut transition_installed_this_release = false;
        if let Some(t) = self.intake.take() {
            let d = *t.descriptor();
            match d.kind {
                CommandKind::Lease(req) => {
                    let result = core.commit_lease(&d, req, &ctx);
                    report.command = Some((d.stamp.request, match result {
                        Ok(outcome) => {
                            if let super::requests::lease::LeaseOutcome::Granted {
                                evicted_connection: Some(ev_conn),
                                evicted_generation: ev_gen,
                                ..
                            } = outcome
                            {
                                deferred_eviction = Some((ev_conn, ev_gen));
                            }
                            let (commit_seq, reply) = t.ack_lease_deferred(outcome);
                            deferred_step3_reply = Some(reply);
                            Ok(commit_seq)
                        }
                        Err(code) => {
                            deferred_step3_reply = Some(t.refuse_deferred(code));
                            Err(code)
                        }
                    }));
                }
                _ => {
                    #[cfg(not(target_arch = "wasm32"))]
                    let is_begin_macro = matches!(d.kind, CommandKind::Action(super::requests::actions::ActionCommand::BeginMacro(_)));
                    #[cfg(target_arch = "wasm32")]
                    let is_begin_macro = false;
                    #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
                    let is_action_cmd = matches!(d.kind, CommandKind::Action(_) | CommandKind::Policy(_));
                    #[cfg(all(not(target_arch = "wasm32"), not(feature = "policy-runtime")))]
                    let is_action_cmd = matches!(d.kind, CommandKind::Action(_));
                    #[cfg(target_arch = "wasm32")]
                    let is_action_cmd = false;
                    #[cfg(not(target_arch = "wasm32"))]
                    let is_child_cmd = matches!(d.kind, CommandKind::Action(super::requests::actions::ActionCommand::InstallChild { .. }));
                    #[cfg(target_arch = "wasm32")]
                    let is_child_cmd = false;
                    let starts_top_action = match d.kind {
                        CommandKind::Install { .. }
                        | CommandKind::InstallHome { .. }
                        | CommandKind::InstallPark { .. } => true,
                        #[cfg(not(target_arch = "wasm32"))]
                        CommandKind::Action(a) => a.starts_action(),
                        #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
                        CommandKind::Policy(p) => p.starts_action(),
                        CommandKind::ContinuePark { .. }
                        | CommandKind::Immediate(_)
                        | CommandKind::Transition(_)
                        | CommandKind::Lease(_) => false,
                    };
                    // Provisional read-only capture before Step 3 mutates `core`: if a Step-1 Stop
                    // already ended the prior active action/macro, preserve its exact outcome & children
                    // ONLY if the new command commits (refusal leaves `self.active` untouched).
                    let prior_ended = if starts_top_action {
                        self.active.as_ref().map(ActiveSlot::action).and_then(|old_id| {
                            core.action_outcome(old_id).map(|o| (o, core.action_children(old_id)))
                        })
                    } else {
                        None
                    };
                    let result: Result<Committed, CommitRefusal> = match d.kind {
                        CommandKind::Install { .. } => {
                            #[cfg(not(target_arch = "wasm32"))]
                            let mixed_a7_child = matches!(self.active, Some(ActiveSlot::Macro { .. }))
                                && t.plan().is_some_and(|p| p.primitive.child().is_some());
                            #[cfg(target_arch = "wasm32")]
                            let mixed_a7_child = false;
                            if mixed_a7_child {
                                Err(CommitRefusal::code("UNSUPPORTED"))
                            } else {
                                match t.plan() {
                                    None => Err(CommitRefusal::code("PLAN_RETIRED")),
                                    Some(slot) if slot.motion.is_some() => {
                                        let m = slot.motion.expect("checked is_some");
                                        core.commit_install_slot(&d, &m, &ctx).map_err(CommitRefusal::code)
                                    }
                                    Some(slot) if matches!(slot.primitive, Primitive::Prepared { .. }) => match t.plan_ref() {
                                        Some(slot_ref) => core.commit_prepared_slot_with_detail(&d, slot_ref, &ctx),
                                        None => Err(CommitRefusal::code("PLAN_RETIRED")),
                                    },
                                    Some(slot) => core.commit_primitive(&d, &slot.primitive, &ctx).map_err(CommitRefusal::code),
                                }
                            }
                        }
                        CommandKind::InstallHome { .. } => match t.plan().and_then(|s| s.motion) {
                            Some(m) => core.commit_home(&d, &m, &ctx).map_err(CommitRefusal::code),
                            None => Err(CommitRefusal::code("PLAN_RETIRED")),
                        },
                        CommandKind::InstallPark { .. } => match t.plan().and_then(|s| s.motion) {
                            Some(m) => core.commit_park(&d, &m, &ctx).map_err(CommitRefusal::code),
                            None => Err(CommitRefusal::code("PLAN_RETIRED")),
                        },
                        CommandKind::ContinuePark { .. } => match t.plan().and_then(|s| s.motion) {
                            Some(m) => core.commit_park_continuation(&d, &m, &ctx).map_err(CommitRefusal::code),
                            None => Err(CommitRefusal::code("PLAN_RETIRED")),
                        },
                        CommandKind::Immediate(op) => core.commit_immediate(&d, op, &ctx).map_err(CommitRefusal::code),
                        CommandKind::Transition(op) => {
                            core.commit_transition_with_budget(
                                &d,
                                op,
                                &ctx,
                                &mut ledger,
                                &mut now_fn,
                            )
                            .map_err(CommitRefusal::code)
                        }
                        #[cfg(not(target_arch = "wasm32"))]
                        CommandKind::Action(cmd) => {
                            let slot_res = if cmd.plan().is_some() {
                                t.plan_ref().map(Some).ok_or(CommitRefusal::code("PLAN_RETIRED"))
                            } else {
                                Ok(None)
                            };
                            match slot_res {
                                Ok(slot_ref) => core.commit_action(&d, cmd, slot_ref, &ctx),
                                Err(err) => Err(err),
                            }
                        }
                        #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
                        CommandKind::Policy(cmd) => {
                            let r = core.commit_policy(&d, cmd, &ctx).map_err(CommitRefusal::code);
                            core.finish_policy_command(d.stamp.request.0);
                            r
                        }
                        CommandKind::Lease(_) => unreachable!(),
                    };
                    report.command = Some((d.stamp.request, match result {
                        Ok(c) => {
                            let install = d.kind.is_install();
                            let supersedes = starts_top_action && c.action().is_some() && c.request() == d.stamp.request && self.active.is_some();
                            if supersedes {
                                t.ack_deferred(c).map(|(acked, ack_reply)| {
                                    let old_slot = self.active.take().expect("checked active.is_some()");
                                    let (old_installed, old_child_plan) = old_slot.into_parts();
                                    retire_after_take = old_child_plan;
                                    let (old_outcome, old_children) = prior_ended
                                        .unwrap_or_else(|| (Outcome::Superseded, core.action_children(old_installed.action())));
                                    superseded_and_ack = Some((old_installed, old_outcome, old_children, ack_reply));
                                    if let Some(i) = acked.installed {
                                        #[cfg(not(target_arch = "wasm32"))]
                                        {
                                            self.active = Some(if is_begin_macro {
                                                ActiveSlot::Macro { parent: i, child_plan: None }
                                            } else {
                                                ActiveSlot::Standalone(i)
                                            });
                                        }
                                        #[cfg(target_arch = "wasm32")]
                                        {
                                            let _ = is_begin_macro;
                                            self.active = Some(ActiveSlot::Standalone(i));
                                        }
                                    }
                                    acked.commit_seq
                                })
                            } else if is_action_cmd {
                                t.ack_deferred(c).map(|(acked, ack_reply)| {
                                    deferred_step3_reply = Some(ack_reply);
                                    if is_child_cmd {
                                        #[cfg(not(target_arch = "wasm32"))]
                                        if let Some(new_cp) = d.kind.plan() {
                                            if let Some(ActiveSlot::Macro { child_plan, .. }) = self.active.as_mut() {
                                                retire_after_take = child_plan.replace(new_cp);
                                            } else {
                                                retire_after_take = Some(new_cp);
                                            }
                                        }
                                    } else if let Some(i) = acked.installed {
                                        #[cfg(not(target_arch = "wasm32"))]
                                        {
                                            self.active = Some(if is_begin_macro {
                                                ActiveSlot::Macro { parent: i, child_plan: None }
                                            } else {
                                                ActiveSlot::Standalone(i)
                                            });
                                        }
                                        #[cfg(target_arch = "wasm32")]
                                        {
                                            self.active = Some(ActiveSlot::Standalone(i));
                                        }
                                    }
                                    acked.commit_seq
                                })
                            } else if install {
                                t.ack(c).map(|acked| {
                                    if let Some(i) = acked.installed {
                                        self.active = Some(ActiveSlot::Standalone(i));
                                    }
                                    acked.commit_seq
                                })
                            } else {
                                t.ack_deferred(c).map(|(acked, ack_reply)| {
                                    if let Some(i) = acked.installed {
                                        self.transition = Some(i);
                                        transition_installed_this_release = true;
                                    }
                                    deferred_step3_reply = Some(ack_reply);
                                    acked.commit_seq
                                })
                            }
                        }
                        Err(refusal) => {
                            deferred_step3_reply = Some(t.refuse_deferred_with(refusal.code, refusal.detail));
                            Err(refusal.code)
                        }
                    }));
                }
            }
        }
        if let Some(cp) = retire_after_take {
            self.intake.retire_plan(cp);
        }
        // 4. Unconditional urgent recheck + consume at most one latest target
        //    Defer reporting `TargetOutcome` until Step 7 (strictly AFTER Step 6 `core.publish`)
        //    so off-thread command waiters always observe the post-motion published snapshot.
        let ctx = self.checkpoint(core, Checkpoint::BeforeTarget, &mut now_fn, &mut report);
        let mut deferred_target_outcome: Option<TargetOutcome> = None;
        if let Some((f, validity)) = self.targets.take_with_validity() {
            let cur_refusal_gen = self.targets.policy_refusal_gen();
            let cascaded = f.policy_generation != 0
                && validity
                    .expected_refusal_gen
                    .is_some_and(|exp| exp != cur_refusal_gen);
            let result = if cascaded {
                core.on_policy_refusal_cascade(&f);
                Err(Refusal::code("POLICY_REVOKED"))
            } else {
                let res = core.consume_target_with_validity(&f, validity, &ctx);
                if f.policy_generation != 0 && res.is_err() {
                    self.targets.note_policy_refusal();
                }
                res
            };
            deferred_target_outcome = Some(TargetOutcome { stream: f.stream, seq: f.seq, result });
            report.target_refusal = result.err();
            report.target = Some((f.seq, result.map_err(|r| r.code)));
        }
        // 5. Urgent recheck + advance control output (passes live `now_fn` so pre-step and
        //    transmit-point macro deadline checks always take fresh clock reads).
        let _out_ctx = self.checkpoint(core, Checkpoint::BeforeOutput, &mut now_fn, &mut report);
        let timestamp = core.advance_output_with_budget(dt, publish_generation, &mut ledger, &mut now_fn);
        self.release_budget = Some(ledger);
        // Consume any transition aborted during Steps 2b-5 (including one installed in Step 3 and
        // fenced by Disconnect/expiry in Steps 4-5) at the single pre-publish boundary so Step 6
        // publishes `torque_transition_pending == false`, while Step 7 delivers replies strictly
        // AFTER publication (and emits a same-release transition's own broker ACK before its
        // abort Terminal, both strictly BEFORE lifecycle replies).
        let aborted_transition = if finished_transition.is_none() {
            self.take_aborted_transition(core, &mut report)
        } else {
            None
        };
        // Refresh `report.poll.urgent_wake` strictly AFTER Step 5 output (and transition abort
        // settlement) so control/hold frames transmitted in Step 5 supersede any stale Step-2
        // watchdog wake before Window 2 admission.
        let transition_active = (self.transition.is_some()
            && finished_transition.is_none()
            && aborted_transition.is_none())
            || report.poll.park_wake.is_some();
        let post_out_now = now_fn();
        report.poll.urgent_wake =
            core.post_output_urgent_wake(transition_active, post_out_now, report.poll.urgent_wake);
        // 6. Publish
        core.publish(timestamp, publish_generation);
        // 7. Deliver aborted/finished transition terminals, urgent stop replies, lifecycle replies,
        //    Step 3 replies, target outcomes, and active-action terminals strictly AFTER publish.
        if transition_installed_this_release {
            if let Some((t, outcome)) = aborted_transition {
                if let Some(ack_reply) = deferred_step3_reply.take() {
                    self.intake.emit_reply(ack_reply);
                }
                self.intake.terminal(t, outcome);
            } else if let Some((t, outcome)) = finished_transition {
                self.intake.terminal(t, outcome);
            }
        } else if let Some((t, outcome)) = finished_transition.or(aborted_transition) {
            self.intake.terminal(t, outcome);
        }
        if self.flush_urgent_stop_replies() {
            self.urgent_replies_flushed = true;
        }
        if self.flush_lifecycle_replies() {
            self.lifecycle_replies_flushed = true;
        }
        if let Some((old, outcome, children, ack_reply)) = superseded_and_ack {
            self.intake.terminal_with_children(old, outcome, children);
            self.intake.emit_reply(ack_reply);
        }
        if let Some(reply) = deferred_step3_reply {
            self.intake.emit_reply(reply);
        }
        if let Some(outcome) = deferred_target_outcome {
            self.targets.report(outcome);
        }
        if let Some(eviction) = deferred_eviction {
            if let Some(tx) = self.eviction_tx.as_mut() {
                let _ = tx.push(eviction);
            }
        }
        let _ = self.flush_servo_closes(core);
        if core.flush_policy_close_watermark() != 0 {
            self.servo_closes_flushed = true;
        }
        #[cfg(not(target_arch = "wasm32"))]
        if let Some(ActiveSlot::Macro { parent, child_plan }) = self.active.as_mut() {
            if child_plan.is_some() && !core.macro_child_running(parent.action()) {
                if let Some(cp) = child_plan.take() {
                    self.intake.retire_plan(cp);
                }
            }
        }
        if let Some(action) = self.active() {
            if let Some(outcome) = core.action_outcome(action) {
                let slot = self.active.take().expect("active above");
                let (installed, child_plan) = slot.into_parts();
                if let Some(cp) = child_plan {
                    self.intake.retire_plan(cp);
                }
                let children = core.action_children(action);
                self.intake.terminal_with_children(installed, outcome, children);
                report.ended = Some((action, outcome));
            }
        }
        report
    }

    /// Runs one sub-release transition wake on `core` (no full feedback poll, no output step).
    /// When the transition completes, a lifecycle command mutated state, or an urgent stop applies
    /// on this wake, publishes the updated state at `publish_generation` BEFORE delivering
    /// replies/terminals and returns `true`.
    pub fn service_wake<O: CoreOps>(
        &mut self,
        core: &mut O,
        now: Instant,
        release_deadline: Instant,
        publish_generation: u64,
        report: &mut ReleaseReport,
    ) -> bool {
        let entry_wall = Instant::now();
        let mut live_now = || now + Instant::now().saturating_duration_since(entry_wall);
        self.service_wake_with_clock(core, &mut live_now, release_deadline, publish_generation, report)
    }

    pub fn service_wake_with_clock<O: CoreOps>(
        &mut self,
        core: &mut O,
        now_fn: impl FnMut() -> Instant,
        release_deadline: Instant,
        publish_generation: u64,
        report: &mut ReleaseReport,
    ) -> bool {
        self.service_wake_with_clock_opts(core, now_fn, release_deadline, publish_generation, true, report)
    }

    /// Runs one sub-release wake. When `allow_transition_poll` is `false` (urgent Stop / E-Stop /
    /// Disconnect wake outside the CAN transition poll budget), applies urgent stops/disconnects,
    /// publishes, and flushes replies strictly in-memory without invoking hardware transition polling.
    pub fn service_wake_with_clock_opts<O: CoreOps>(
        &mut self,
        core: &mut O,
        mut now_fn: impl FnMut() -> Instant,
        release_deadline: Instant,
        publish_generation: u64,
        allow_transition_poll: bool,
        report: &mut ReleaseReport,
    ) -> bool {
        let now = now_fn();
        let (urgent_stopped, urgent_consumed) = self.apply_pending_urgent_detailed(core, now);
        let lifecycle_pending = self.has_pending_lifecycle_replies();
        if urgent_stopped { report.stopped = true; }
        if allow_transition_poll && !urgent_consumed && !lifecycle_pending {
            let core_tx_limit = core.release_tx_budget_limit();
            let tx_limit = self
                .tx_budget_limit
                .map_or(core_tx_limit, |lim| lim.min(core_tx_limit));
            let ledger = self.release_budget.get_or_insert_with(|| {
                ReleaseBudget::new(self.releases, now, release_deadline, self.rx_budget_limit, tx_limit)
            });
            // Sub-budgets in Window 2 may only tighten the release deadline, never widen it.
            ledger.tighten_deadline(release_deadline);
            report.poll = core.poll_transition_wake_with_budget(ledger, &mut now_fn);
        }
        let finished = self
            .take_finished_transition(report)
            .or_else(|| self.take_aborted_transition(core, report));
        let active_ended = self.active().and_then(|a| core.action_outcome(a).map(|o| (a, o)));
        if urgent_stopped
            || urgent_consumed
            || lifecycle_pending
            || core.has_pending_servo_closes()
            || core.has_pending_policy_close()
            || finished.is_some()
            || active_ended.is_some()
            || report.poll.park_state_changed
        {
            let ts = core.now_timestamp();
            core.publish(ts, publish_generation);
            if let Some((t, outcome)) = finished {
                self.intake.terminal(t, outcome);
            }
            if let Some((action, outcome)) = active_ended {
                let old_slot = self.active.take().expect("active above");
                let (installed, child_plan) = old_slot.into_parts();
                if let Some(cp) = child_plan {
                    self.intake.retire_plan(cp);
                }
                let children = core.action_children(action);
                self.intake.terminal_with_children(installed, outcome, children);
                report.ended = Some((action, outcome));
            }
            if self.flush_urgent_stop_replies() {
                self.urgent_replies_flushed = true;
            }
            if self.flush_lifecycle_replies() {
                self.lifecycle_replies_flushed = true;
            }
            let _ = self.flush_servo_closes(core);
            if core.flush_policy_close_watermark() != 0 {
                self.servo_closes_flushed = true;
            }
            return true;
        }
        if self.has_pending_urgent_stop_replies() && self.flush_urgent_stop_replies() {
            self.urgent_replies_flushed = true;
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
    /// Optional explicit Window-2 sub-release safety margin override (`None` uses
    /// [`SubReleaseBudget::default_margin_for_period`]).
    pub sub_release_margin: Option<Duration>,
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
        Self::new_with_clock(core, boot, caps, VirtualClock::starting_at(Instant::now()))
    }

    /// [`Self::new`] driven by a caller-supplied shared [`VirtualClock`].
    pub fn new_with_clock(
        mut core: O,
        boot: BootId,
        caps: Capacities,
        clock: VirtualClock,
    ) -> (Self, Broker<PlanSlot>, TargetWriter, StopRequests) {
        core.set_boot_id(boot);
        let (engine, broker, tw, stop) = ReleaseEngine::new(boot, caps);
        #[cfg(not(target_arch = "wasm32"))]
        core.attach_closed_bitset(broker.closed_bitset());
        core.set_boot_id(boot);
        (Self { core, engine, clock, dt: 0.02, sub_release_margin: None }, broker, tw, stop)
    }

    /// Configures an explicit Window-2 sub-release safety margin (`period - margin`).
    pub fn with_sub_release_margin(mut self, margin: Duration) -> Self {
        self.sub_release_margin = Some(margin);
        self
    }

    /// Configures per-release RX frame and optional TX attempt ledger caps on the underlying
    /// [`ReleaseEngine`].
    pub fn with_release_budget_limits(mut self, rx_limit: u16, tx_limit: Option<u16>) -> Self {
        self.engine.set_release_budget_limits(rx_limit, tx_limit);
        self
    }

    /// Snapshot of the most recent release's [`ReleaseBudget`] ledger.
    pub fn last_release_budget(&self) -> Option<ReleaseBudget> {
        self.engine.last_release_budget()
    }

    /// [`Self::new`] with a freshly generated non-zero executor [`BootId`].
    pub fn new_fresh(core: O, caps: Capacities) -> (Self, Broker<PlanSlot>, TargetWriter, StopRequests) {
        Self::new(core, BootId::fresh(), caps)
    }

    /// The executor's boot identity.
    pub fn boot(&self) -> BootId { self.engine.boot() }

    /// The executor's live urgent-stop handle.
    pub fn stops(&self) -> &StopRequests { self.engine.stop_requests() }

    /// Attaches a bounded urgent-stop intake for deterministic fixture tests.
    pub fn attach_urgent_stops(&mut self, intake: UrgentStopIntake) {
        self.engine.attach_urgent_stops(intake);
    }

    /// Attaches a priority connection lifecycle intake for deterministic fixture tests.
    pub fn attach_conn_lifecycle(&mut self, intake: super::requests::lease::ConnLifecycleIntake) {
        self.engine.attach_conn_lifecycle(intake);
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
        let clock = self.clock.clone();
        self.step_with_clock(move || clock.now())
    }

    /// One release using an explicit live clock sampler `now_fn` for all in-release timestamps
    /// (`BeforeCommit`, `BeforeTarget`, `BeforeOutput`, pre-step, transmit, and post-output),
    /// while release scheduling/deadline remains anchored to `self.clock`.
    pub fn step_with_clock(&mut self, mut now_fn: impl FnMut() -> Instant) -> ReleaseReport {
        let fixture_started = self.clock.now();
        let period = Duration::from_secs_f64(self.dt);
        let fixture_deadline = fixture_started + period;
        // Anchor release admission, cutoffs, sleep, and polling to one consistent owner timeline
        // (`now_fn()`) so tests driving a separate `MockClock` never compare wakes across clocks.
        let owner_started = now_fn();
        let owner_deadline = owner_started + period;
        let next_gen = self.engine.releases() + 1;
        let mut report = self.engine.step(&mut self.core, &mut now_fn, self.dt, owner_deadline, next_gen);
        // Extra sub-release hardware services in Window 2 (bounded to MAX_SERVICE_WAKES = 1):
        // - Transition and watchdog-hold polls run only if sleep + poll + tail can complete before
        //   `owner_deadline - margin` (`wake_budget.transition_start_cutoff`, bounded by
        //   `wake_budget.poll_deadline`); otherwise deferred to the start of the next release.
        // - Immediate in-memory urgent Stop / E-Stop / Disconnect wakes remain eligible up to
        //   `wake_budget.urgent_stop_cutoff` without entering hardware CAN polling.
        let margin = self
            .sub_release_margin
            .unwrap_or_else(|| SubReleaseBudget::default_margin_for_period(period));
        let wake_budget = SubReleaseBudget::for_release(owner_started, owner_deadline, period, margin);
        if let Some((wake, kind)) = wake_budget.admit_next_wake(
            now_fn(),
            self.engine.has_pending_urgent(&self.core),
            &report.poll,
        ) {
            if report.wakes < MAX_SERVICE_WAKES {
                let now = now_fn();
                let urgent_stop_pending = kind == SubReleaseWakeKind::UrgentStop
                    || self.engine.has_pending_urgent(&self.core);
                let exec_cutoff = if urgent_stop_pending {
                    wake_budget.urgent_stop_cutoff
                } else {
                    wake_budget.transition_start_cutoff
                };
                let target_wake = if urgent_stop_pending { now } else { wake };
                if now < exec_cutoff && target_wake <= exec_cutoff {
                    let target = target_wake.min(wake_budget.wake_deadline);
                    if target > now {
                        self.clock.advance(target.saturating_duration_since(now));
                    }
                    let post_now = now_fn();
                    let urgent_stop_now = urgent_stop_pending
                        || self.engine.has_pending_urgent(&self.core);
                    if let Some(allow_transition_poll) =
                        wake_budget.post_sleep_allow_transition_poll(post_now, urgent_stop_now)
                    {
                        report.wakes += 1;
                        self.engine.service_wake_with_clock_opts(
                            &mut self.core,
                            &mut now_fn,
                            wake_budget.poll_deadline,
                            next_gen,
                            allow_transition_poll,
                            &mut report,
                        );
                    }
                }
            }
        }
        // Sample `now_fn` strictly AFTER Window 2 wakes and BEFORE advancing to `fixture_deadline`
        // so callers timing full release duration observe all Window 2 sleep + poll work.
        let _ = now_fn();
        // The next release starts one period later
        self.clock.advance(fixture_deadline.saturating_duration_since(self.clock.now()));
        report
    }

    /// Test helper: explicitly run one Window-2 sub-release transition wake at an arbitrary
    /// deterministic clock instant (used to reproduce sub-release repeat-disable wake timing).
    #[doc(hidden)]
    pub fn service_wake_at_clock(
        &mut self,
        now_fn: impl FnMut() -> Instant,
        release_deadline: Instant,
        report: &mut ReleaseReport,
    ) -> bool {
        let next_gen = self.engine.releases();
        self.engine
            .service_wake_with_clock(&mut self.core, now_fn, release_deadline, next_gen, report)
    }
}

impl<O: CoreOps + ConfigAuthority> ReleaseFixture<O> {
    /// Reserves a configuration proposal against the fixture's real boot identity, virtual
    /// clock, and live urgent-stop generation (applying any pending stop first).
    pub fn reserve_config(&mut self, proposal: &ConfigProposal) -> Reserved {
        self.engine.apply_pending_stop(&mut self.core);
        let ctx = self.ctx(self.clock.now());
        self.core.reserve(proposal, &ctx)
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

    #[test]
    fn eight_plan_slots_reserve_655360_payload_bytes() {
        assert_eq!(PLAN_SLOT_PAYLOAD_BYTES, 81_920);
        assert_eq!(ALL_PLAN_SLOTS_PAYLOAD_BYTES, 655_360);
        let (mut w, mut r) = super::super::broker::plan_slots(super::super::broker::PLAN_SLOTS, PlanSlot::default);
        let mut handles = Vec::new();
        let mut total_bytes = 0usize;
        for _ in 0..super::super::broker::PLAN_SLOTS {
            let h = w.stage(|s| { s.path.clear(); }).expect("slot");
            let slot = r.get(h).expect("live");
            assert_eq!(slot.path.capacity(), MAX_PATH_SAMPLES);
            assert_eq!(slot.payload_bytes(), PLAN_SLOT_PAYLOAD_BYTES);
            total_bytes += slot.payload_bytes();
            handles.push(h);
        }
        assert_eq!(total_bytes, 655_360);
        assert!(w.stage(|_| {}).is_none());
        for h in handles { assert!(r.retire(h)); }
    }

    #[test]
    fn stop_diag_slot_records_coalesces_and_drains() {
        let slot = StopDiagSlot::new();
        let mut seen = 0;
        assert_eq!(slot.poll(&mut seen), None);

        slot.record_park_ignored(ArmId(3), 1.25, 12.5);
        slot.record_park_ignored(ArmId(3), 2.5, 12.5);
        assert_eq!(slot.count(), 2);

        let rec = slot.drain_and_log(&mut seen).expect("coalesced record");
        assert_eq!(
            rec,
            StopDiagRecord {
                count: 2,
                arm: ArmId(3),
                park_elapsed_us: 2_500_000,
                park_budget_us: 12_500_000,
            }
        );
        assert_eq!(seen, 2);
        assert_eq!(slot.poll(&mut seen), None);

        slot.record_park_ignored(ArmId::LOCAL, f64::NAN, -1.0);
        let san = slot.poll(&mut seen).expect("sanitized record");
        assert_eq!((san.count, san.park_elapsed_us, san.park_budget_us), (1, 0, 0));
    }
}
