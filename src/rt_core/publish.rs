//! State publication: the control tick is the one producer of state samples; readers
//! (REST `GET /observation`, the WS observation stream, the black box, measurement polls)
//! get them through [`StateHub`] without the supervisor lock.
//!
//! # What a published sample means
//!
//! A [`Snapshot`] is the state the control tick computed at the end of one tick, under the
//! supervisor write lock, after that tick's feedback was read and its output was written:
//!
//! - `obs` is exactly what `SupervisorInner::current_observation` returned at that moment;
//!   `obs.timestamp` is its local monotonic sample time ([`super::ids::MonotonicS`]).
//! - `meta.seq` counts publications of this process (1, 2, ...; no gaps). The tick
//!   publishes once per tick, so it is the RT tick counter: a reader that sees it stop
//!   advancing knows the control tick stopped (`observation_cache.published` exposes it).
//! - `meta.generation` is the supervisor lock generation of the tick's write guard
//!   (`SupervisorLock::generation`): every write guard taken later has a higher one, so a
//!   sample answers for the state while the current generation still equals it, and a
//!   reader that saw generation `g` before a request gets read-your-writes from any sample
//!   with `generation >= g`.
//! - `meta.valid_until` is the local deadline after which time-derived fields would flip
//!   without a write (lease expiry, previous-action window, 250 ms feedback freshness behind
//!   `feedback_valid` / `enable_state`; `SupervisorInner::observation_valid_until`), capped at
//!   two control periods.
//! - `rt` holds the per-joint feedback the tick also used (positions, velocities, receive
//!   instants and per-joint feedback sequence numbers), sized for this arm's profile.
//!
//! The tick never waits for a reader: publication is one atomic swap of a preallocated
//! slot ([`super::triple`]). Readers share [`StateHub`]'s mutex with each other only.

use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

use crate::macro_action::{ContactWatch, SubState};
use crate::model::{ArmObservation, HolderLabel, ObservationSchema};

use super::broker::{Authority, BootId};
use super::ids::ArmId;
use super::mailbox::MAX_DOF;
use super::triple::{triple_buffer, Consumer, Producer};

/// Fixed-size, `Copy` owner state published alongside every observation on [`StateHub`].
///
/// Zero heap allocations and zero locks on the RT control thread (`armd-control`); readers
/// (REST schema/lease checks, WS upgrade/ACK/reject decoration, off-thread request preparation)
/// project variable-length fields off-thread from this coherent snapshot.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OwnerStateSnapshot {
    pub arm: ArmId,
    pub boot: BootId,
    pub control_frequency_hz: u32,
    pub lease_enabled: bool,
    /// Active unfenced lease generation at `sampled_at`, or `0` when no valid lease is held.
    pub lease_generation: u64,
    /// Bounded internal connection handle of the active lease holder (`None` when unheld/expired).
    pub lease_owner: Option<HolderLabel>,
    /// Numeric WS connection ID (`>= 1`) of the active lease holder (`0` for non-WS / none).
    pub lease_owner_conn_id: u64,
    /// Display label of the active lease holder (`Some("")` preserved for empty legacy labels).
    pub lease_label: Option<HolderLabel>,
    /// Monotonic expiry instant of the active lease (`acquired_at + lease_timeout`).
    pub lease_expires_at: Option<Instant>,
    /// Internal connection handle of the lease most recently evicted by forced preemption.
    pub last_evicted_owner: Option<HolderLabel>,
    /// Generation of the lease most recently evicted by forced preemption (`0` if none).
    pub last_evicted_generation: u64,
    /// Highest lease generation fenced by release, expiry, preemption, or disconnect.
    pub fenced_lease_generation: u64,
    /// Timestamp of the last forced preemption (for off-thread cooldown visibility).
    pub last_preemption: Option<Instant>,
    pub epoch: u64,
    pub stop_generation: u64,
    pub config_revision: u64,
    pub parent_generation: u64,
    /// Revision pairing this published sample with [`StateHub`]'s off-thread schema cache.
    pub schema_revision: u64,
    pub is_torqued: bool,
    pub disable_pending: bool,
    pub torque_transition_pending: bool,
    pub motion_active: bool,
    pub handover_settling: bool,
    pub calibration_busy: bool,
    pub config_inhibited: bool,
    pub e_stop_latched: bool,
    pub operator_auth_enforced: bool,
    pub gravity_comp_active: bool,
    pub safety_stopped: bool,
    pub safety_recoverable: bool,
    pub dof: u8,
    pub commanded_positions: [f32; MAX_DOF],
    pub residual_torques: Option<[f32; 6]>,
    pub has_zero_reference: bool,
    pub zero_convention: crate::joint_zero::ZeroConvention,
}

impl Default for OwnerStateSnapshot {
    fn default() -> Self {
        Self {
            arm: ArmId::LOCAL,
            boot: BootId(0),
            control_frequency_hz: 50,
            lease_enabled: true,
            lease_generation: 0,
            lease_owner: None,
            lease_owner_conn_id: 0,
            lease_label: None,
            lease_expires_at: None,
            last_evicted_owner: None,
            last_evicted_generation: 0,
            fenced_lease_generation: 0,
            last_preemption: None,
            epoch: 0,
            stop_generation: 0,
            config_revision: 0,
            parent_generation: 0,
            schema_revision: 0,
            is_torqued: false,
            disable_pending: false,
            torque_transition_pending: false,
            motion_active: false,
            handover_settling: false,
            calibration_busy: false,
            config_inhibited: false,
            e_stop_latched: false,
            operator_auth_enforced: false,
            gravity_comp_active: false,
            safety_stopped: false,
            safety_recoverable: true,
            dof: 0,
            commanded_positions: [0.0; MAX_DOF],
            residual_torques: None,
            has_zero_reference: false,
            zero_convention: crate::joint_zero::ZeroConvention::default(),
        }
    }
}

impl OwnerStateSnapshot {
    /// True iff this snapshot reflects an active, unfenced lease not yet expired at `now`.
    #[inline]
    pub fn is_lease_active_at(&self, now: Instant) -> bool {
        self.lease_generation != 0
            && self.lease_generation > self.fenced_lease_generation
            && self.lease_expires_at.is_some_and(|exp| now < exp)
    }

    /// Why `connection` no longer holds `generation` according to this snapshot.
    #[inline]
    pub fn lease_loss_reason(&self, connection: &str, generation: u64) -> &'static str {
        match self.last_evicted_owner {
            Some(owner)
                if owner.as_str() == connection && self.last_evicted_generation == generation =>
            {
                "preempted"
            }
            _ => "expired",
        }
    }

    /// Authority tuple projected from this snapshot at `now`.
    #[inline]
    pub fn authority_at(&self, now: Instant) -> Authority {
        Authority {
            lease_generation: if self.is_lease_active_at(now) {
                self.lease_generation
            } else {
                0
            },
            epoch: self.epoch,
            stop_generation: self.stop_generation,
            config_revision: self.config_revision,
            parent_generation: self.parent_generation,
        }
    }
}

/// Publication stamp of a sample (see the module docs).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SampleMeta {
    pub seq: u64,
    pub generation: u64,
    pub sampled_at: Instant,
    pub valid_until: Instant,
}

/// Scalar motion/authority metadata published alongside [`RtSample`] (Copy, 0 alloc on RT).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RtMotionMeta {
    pub boot_id: super::broker::BootId,
    pub clock_now: Option<Instant>,
    pub workspace_revision: u64,
    pub motion_config_revision: u64,
    pub stop_generation: u64,
    pub lease_generation: u64,
    pub config_inhibited: bool,
    pub calibration_busy: bool,
    pub holding_after_clear: bool,
    pub watchdog_park_pending_gen: u64,
    pub watchdog_park_pending_attempt: u8,
}

impl Default for RtMotionMeta {
    fn default() -> Self {
        Self {
            boot_id: super::broker::BootId(0),
            clock_now: None,
            workspace_revision: 0,
            motion_config_revision: 0,
            stop_generation: 0,
            lease_generation: 0,
            config_inhibited: false,
            calibration_busy: false,
            holding_after_clear: false,
            watchdog_park_pending_gen: 0,
            watchdog_park_pending_attempt: 0,
        }
    }
}

/// Zero-alloc `Copy` snapshot of live held-object state published per tick.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PublishedHeld {
    pub label: HolderLabel,
    pub spill_safe: bool,
}

/// Zero-alloc `Copy` snapshot of the running macro parent and current child status.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PublishedMacroState {
    pub active_id: u64,
    pub alive: bool,
    pub children: u32,
    pub child_seq: u32,
    pub child_running: bool,
    pub child_sub: SubState,
    pub child_watch: ContactWatch,
}

impl Default for PublishedMacroState {
    fn default() -> Self {
        Self {
            active_id: 0,
            alive: false,
            children: 0,
            child_seq: 0,
            child_running: false,
            child_sub: SubState::Idle,
            child_watch: ContactWatch::default(),
        }
    }
}

/// Zero-alloc `Copy` owner metadata published every tick alongside `RtSample`, enabling
/// off-thread prepared-action and macro-orchestrator reads without acquiring `SupervisorLock`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PublishedOwnerMeta {
    pub arm: ArmId,
    pub boot: BootId,
    pub authority: Authority,
    /// Monotonic control-release sequence counter of the active owner (0 before first tick).
    pub control_release_seq: u64,
    /// Monotonic generation bumped on every motion lifecycle transition (trajectory, servo,
    /// gravity comp, stop, E-stop, clear-fault).
    pub motion_generation: u64,
    pub calibration_revision: u64,
    pub collision_revision: u64,
    pub poses_revision: u64,
    pub sequences_revision: u64,
    pub grasp_setup_revision: u64,
    pub gripper_calib_revision: u64,
    pub power_budget_revision: u64,
    pub tool_commissioning_revision: u64,
    pub tool_digest_hash: u64,
    pub motion_active: bool,
    pub config_inhibited: bool,
    pub calibration_busy: bool,
    pub feedback_valid: bool,
    pub is_torqued: bool,
    pub pending_rearm: bool,
    pub pending_normal_unload: bool,
    pub collision_hold: bool,
    pub handover_settling: bool,
    pub gravity_comp_active: bool,
    pub policy_busy: bool,
    pub zero_check_persisting: bool,
    pub torque_or_power_hold: bool,
    pub held: Option<PublishedHeld>,
    pub macro_state: PublishedMacroState,
    /// Coherent snapshot of the active or latest terminal PR0C `ModeSwitch` session (`Copy`, 0 alloc).
    #[cfg(not(target_arch = "wasm32"))]
    pub mode_switch: Option<crate::motor_mode::ModeSwitchSnapshot>,
    /// Bounded fixed ring of recent terminal `ModeSwitch` snapshots retained by `session_id` so
    /// owner-side slot retirement never loses a terminal result when a newer session starts.
    #[cfg(not(target_arch = "wasm32"))]
    pub recent_mode_switch_terminals: [Option<crate::motor_mode::ModeSwitchSnapshot>; 4],
}

impl Default for PublishedOwnerMeta {
    fn default() -> Self {
        Self {
            arm: ArmId::LOCAL,
            boot: BootId(0),
            authority: Authority::default(),
            control_release_seq: 0,
            motion_generation: 0,
            calibration_revision: 0,
            collision_revision: 0,
            poses_revision: 0,
            sequences_revision: 0,
            grasp_setup_revision: 0,
            gripper_calib_revision: 0,
            power_budget_revision: 0,
            tool_commissioning_revision: 0,
            tool_digest_hash: 0,
            motion_active: false,
            config_inhibited: false,
            calibration_busy: false,
            feedback_valid: false,
            is_torqued: false,
            pending_rearm: false,
            pending_normal_unload: false,
            collision_hold: false,
            handover_settling: false,
            gravity_comp_active: false,
            policy_busy: false,
            zero_check_persisting: false,
            torque_or_power_hold: false,
            held: None,
            macro_state: PublishedMacroState::default(),
            #[cfg(not(target_arch = "wasm32"))]
            mode_switch: None,
            #[cfg(not(target_arch = "wasm32"))]
            recent_mode_switch_terminals: [None; 4],
        }
    }
}

/// Coherent revision vector captured atomically from a single RT [`Snapshot`] slot.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ServiceRevisions {
    pub workspace: u64,
    pub motion_config: u64,
    pub calibration: u64,
    pub collision: u64,
    pub poses: u64,
    pub sequences: u64,
    pub grasp_setup: u64,
    pub gripper_calib: u64,
    pub power_budget: u64,
    pub tool_commissioning: u64,
    pub tool_digest_hash: u64,
}

impl ServiceRevisions {
    pub fn from_snapshot(snap: &Snapshot) -> Self {
        let o = &snap.rt.owner;
        Self {
            workspace: o.authority.config_revision,
            motion_config: snap.rt.motion_config_revision,
            calibration: o.calibration_revision,
            collision: o.collision_revision,
            poses: o.poses_revision,
            sequences: o.sequences_revision,
            grasp_setup: o.grasp_setup_revision,
            gripper_calib: o.gripper_calib_revision,
            power_budget: o.power_budget_revision,
            tool_commissioning: o.tool_commissioning_revision,
            tool_digest_hash: o.tool_digest_hash,
        }
    }
}

/// Off-RT arm identity and static/joint-zero descriptors paired with RT [`Snapshot`] by
/// `(arm, boot, tool_digest_hash, calibration_revision)`.
///
/// Never allocated, mutated, or locked on `armd-control`. Live RT-changing fields such as
/// `zero_convention` and `has_zero_reference` are taken exclusively from `Snapshot::owner`.
#[derive(Debug, Clone, PartialEq)]
pub struct ServiceIdentity {
    pub arm: ArmId,
    pub boot: BootId,
    pub profile_id: Option<String>,
    pub follower: Option<String>,
    pub dof: usize,
    pub arm_dof: usize,
    pub control_frequency_hz: u32,
    pub joint_names: Vec<String>,
    pub tool_id: String,
    pub tool_config_digest: String,
    pub tool_digest_hash: u64,
    pub tool_is_stock: bool,
    pub joint_zero_offsets: Vec<f32>,
    /// Exact `calibration_revision` corresponding to `joint_zero_offsets`.
    pub calibration_revision: u64,
}

impl ServiceIdentity {
    pub fn from_inner(inner: &crate::supervisor::SupervisorInner) -> Self {
        Self {
            arm: inner.core.id(),
            boot: inner.boot_id,
            profile_id: inner.config.profile_id.clone(),
            follower: inner.follower.clone(),
            dof: inner.config.dof,
            arm_dof: inner.config.arm_dof(),
            control_frequency_hz: inner.config.control_frequency_hz.max(1),
            joint_names: inner.config.limits.iter().map(|l| l.name.clone()).collect(),
            tool_id: inner.config.active_tool.id.clone(),
            tool_config_digest: inner.config.active_tool.config_digest.clone(),
            tool_digest_hash: super::ops::tool_digest_hash(&inner.config.active_tool.config_digest),
            tool_is_stock: inner.config.active_tool.is_stock,
            joint_zero_offsets: inner.joint_zero_offsets.clone(),
            calibration_revision: inner.calibration_revision,
        }
    }

    pub fn from_published_config(
        arm: ArmId,
        boot: BootId,
        cfg: &super::requests::config::PublishedConfig,
    ) -> Self {
        Self {
            arm,
            boot,
            profile_id: cfg.config.profile_id.clone(),
            follower: cfg.follower.clone(),
            dof: cfg.config.dof,
            arm_dof: cfg.config.arm_dof(),
            control_frequency_hz: cfg.config.control_frequency_hz.max(1),
            joint_names: cfg.config.limits.iter().map(|l| l.name.clone()).collect(),
            tool_id: cfg.config.active_tool.id.clone(),
            tool_config_digest: cfg.config.active_tool.config_digest.clone(),
            tool_digest_hash: super::ops::tool_digest_hash(&cfg.config.active_tool.config_digest),
            tool_is_stock: cfg.config.active_tool.is_stock,
            joint_zero_offsets: cfg.joint_zero_offsets.clone(),
            calibration_revision: cfg.calibration_revision,
        }
    }

    /// True iff this off-RT identity matches the sampled RT [`PublishedOwnerMeta`] on every
    /// authenticating key (`arm`, `boot`, `tool_digest_hash`, `calibration_revision`).
    #[inline]
    pub fn matches_owner_meta(&self, owner: &PublishedOwnerMeta) -> bool {
        self.arm == owner.arm
            && self.boot == owner.boot
            && self.tool_digest_hash == owner.tool_digest_hash
            && self.calibration_revision == owner.calibration_revision
    }
}

/// Caller requirements for [`StateHub::read_services`] (fail-closed on any unmet predicate).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReadRequirements {
    /// Minimum `SampleMeta::generation` (used to bind reads past an executor handoff boundary).
    pub min_generation: Option<u64>,
    /// Minimum `SampleMeta::seq` (StateHub publication sequence).
    pub min_publish_seq: Option<u64>,
    /// Minimum `PublishedOwnerMeta::control_release_seq`.
    pub min_release_seq: Option<u64>,
    /// Require `now <= snap.meta.valid_until`.
    pub require_within_valid_until: bool,
    /// Maximum allowed age `now - snap.meta.sampled_at`.
    pub max_snapshot_age: Option<Duration>,
    /// Require `PublishedOwnerMeta::feedback_valid == true`.
    pub require_feedback_valid: bool,
    /// Require every arm joint (`0..arm_dof`) to carry a receive instant with
    /// `now - rx_instant <= max_feedback_age`.
    pub max_feedback_age: Option<Duration>,
    /// Require command/mode inactivity (no active trajectory, servo target, gravity comp,
    /// active/loading policy, pending unload, torque transition, or calibration busy).
    pub require_motion_idle: bool,
    /// Require every arm joint's measured speed `|v_i| <= max_arm_joint_speed` (rad/s).
    pub max_arm_joint_speed: Option<f32>,
    /// Require and attach an off-RT [`ActionPrepEnv`](super::requests::actions::ActionPrepEnv)
    /// authenticated against the sampled RT `PublishedOwnerMeta` (workspace, calibration, poses,
    /// sequences, grasp_setup, gripper_calib, power_budget, tool_digest_hash).
    pub require_prep_env: bool,
}

impl Default for ReadRequirements {
    fn default() -> Self {
        Self {
            min_generation: None,
            min_publish_seq: None,
            min_release_seq: None,
            require_within_valid_until: true,
            max_snapshot_age: None,
            require_feedback_valid: false,
            max_feedback_age: None,
            require_motion_idle: false,
            max_arm_joint_speed: None,
            require_prep_env: false,
        }
    }
}

impl ReadRequirements {
    /// Relaxed coherent read: accepts any published sample with coherent identity (even if
    /// `valid_until` elapsed), suitable for post-run metadata / sandbox identity reads.
    pub const fn identity() -> Self {
        Self {
            min_generation: None,
            min_publish_seq: None,
            min_release_seq: None,
            require_within_valid_until: false,
            max_snapshot_age: None,
            require_feedback_valid: false,
            max_feedback_age: None,
            require_motion_idle: false,
            max_arm_joint_speed: None,
            require_prep_env: false,
        }
    }

    /// Requires a live, valid-until snapshot with confirmed fresh feedback within `max_age`.
    pub const fn fresh_feedback(max_age: Duration) -> Self {
        Self {
            min_generation: None,
            min_publish_seq: None,
            min_release_seq: None,
            require_within_valid_until: true,
            max_snapshot_age: Some(max_age),
            require_feedback_valid: true,
            max_feedback_age: Some(max_age),
            require_motion_idle: false,
            max_arm_joint_speed: None,
            require_prep_env: false,
        }
    }

    /// Requirements for stationary measurement / calibration hover capture.
    pub const fn measure(max_feedback_age: Duration, max_joint_speed: f32) -> Self {
        Self {
            min_generation: None,
            min_publish_seq: None,
            min_release_seq: None,
            require_within_valid_until: true,
            max_snapshot_age: Some(max_feedback_age),
            require_feedback_valid: true,
            max_feedback_age: Some(max_feedback_age),
            require_motion_idle: true,
            max_arm_joint_speed: Some(max_joint_speed),
            require_prep_env: true,
        }
    }
}

/// Fail-closed error returned by [`StateHub::read_services`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ServiceReadError {
    NoSnapshot,
    IdentityUnavailable,
    PrepEnvUnavailable,
    GenerationNotReached { required: u64, actual: u64 },
    SequenceNotReached { required: u64, actual: u64 },
    ReleaseSequenceNotReached { required: u64, actual: u64 },
    SnapshotExpired,
    SnapshotStale { age_ms: u64, max_ms: u64 },
    IdentityMismatch,
    PrepEnvMismatch,
    FeedbackInvalid,
    FeedbackStale { joint: usize },
    MotionActive,
    JointMoving { joint: usize },
}

impl std::fmt::Display for ServiceReadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoSnapshot => write!(f, "no published control snapshot is available"),
            Self::IdentityUnavailable => write!(f, "service identity metadata is not initialized"),
            Self::PrepEnvUnavailable => write!(f, "planning environment metadata is unavailable"),
            Self::GenerationNotReached { required, actual } => {
                write!(f, "snapshot generation {actual} is older than required {required}")
            }
            Self::SequenceNotReached { required, actual } => {
                write!(f, "snapshot publish sequence {actual} is older than required {required}")
            }
            Self::ReleaseSequenceNotReached { required, actual } => {
                write!(f, "control release sequence {actual} is older than required {required}")
            }
            Self::SnapshotExpired => write!(f, "published snapshot exceeded its valid_until horizon"),
            Self::SnapshotStale { age_ms, max_ms } => {
                write!(f, "published snapshot age {age_ms} ms exceeds {max_ms} ms limit")
            }
            Self::IdentityMismatch => {
                write!(f, "off-RT service identity revisions do not match published RT snapshot")
            }
            Self::PrepEnvMismatch => {
                write!(f, "off-RT planning environment revisions do not match published RT snapshot")
            }
            Self::FeedbackInvalid => write!(f, "hardware feedback is not currently valid"),
            Self::FeedbackStale { joint } => {
                write!(f, "joint {} feedback is missing or stale", joint + 1)
            }
            Self::MotionActive => write!(f, "arm motion or transition is active"),
            Self::JointMoving { joint } => {
                write!(f, "joint {} measured speed exceeds standstill limit", joint + 1)
            }
        }
    }
}

impl std::error::Error for ServiceReadError {}

impl From<ServiceReadError> for crate::supervisor::ArmError {
    fn from(err: ServiceReadError) -> Self {
        match err {
            ServiceReadError::MotionActive | ServiceReadError::JointMoving { .. } => {
                crate::supervisor::ArmError::Rejected {
                    status: 409,
                    code: "ARM_MOVING".into(),
                    message: err.to_string(),
                }
            }
            ServiceReadError::IdentityMismatch | ServiceReadError::PrepEnvMismatch => {
                crate::supervisor::ArmError::Rejected {
                    status: 409,
                    code: "SNAPSHOT_STALE".into(),
                    message: err.to_string(),
                }
            }
            ServiceReadError::FeedbackInvalid | ServiceReadError::FeedbackStale { .. } => {
                crate::supervisor::ArmError::Rejected {
                    status: 503,
                    code: "FEEDBACK_STALE".into(),
                    message: err.to_string(),
                }
            }
            _ => crate::supervisor::ArmError::Rejected {
                status: 503,
                code: "OBSERVATION_UNAVAILABLE".into(),
                message: err.to_string(),
            },
        }
    }
}

/// Coherent off-RT service view built from a single RT [`Snapshot`] slot and authenticated
/// off-RT identity/planning metadata.
///
/// Field provenance:
/// - All live state, revisions, `boot`, `arm`, `release_seq`, `motion_generation`,
///   `applied_stop_generation`, `zero_convention`, and `has_zero_reference` come exclusively
///   from the sampled triple-buffer `snapshot`.
/// - `identity` carries off-RT profile/follower/tool/joint descriptors authenticated against
///   `snapshot.rt.owner` by `(arm, boot, tool_digest_hash, calibration_revision)`.
/// - `prep_env` is present only when requested via `ReadRequirements::require_prep_env` and
///   authenticated by `ActionPrepEnv::matches_owner_meta(&snapshot.rt.owner)`.
#[derive(Debug, Clone)]
pub struct ServiceSnapshot {
    pub boot: BootId,
    pub arm: ArmId,
    /// Monotonic `StateHub` publication sequence (`snapshot.meta.seq`).
    pub publish_seq: u64,
    /// Control-release sequence captured in the same RT slot (`snapshot.rt.owner.control_release_seq`).
    pub release_seq: u64,
    pub captured_at: Instant,
    pub valid_until: Instant,
    pub feedback_valid: bool,
    /// Oldest arm-joint feedback age at `captured_at` (`None` if `arm_dof == 0` or any arm joint
    /// had no receive instant).
    pub feedback_age_at_capture: Option<Duration>,
    pub motion_generation: u64,
    /// True when an active trajectory or servo target is present.
    pub motion_active: bool,
    /// True when no trajectory, servo target, gravity comp, policy run/load, pending unload,
    /// torque transition, or calibration transaction is active.
    pub motion_idle: bool,
    /// Applied stop generation captured in the same RT slot (`snapshot.rt.stop_generation`).
    pub applied_stop_generation: u64,
    pub epoch_id: u64,
    pub lease_generation: u64,
    pub zero_convention: crate::joint_zero::ZeroConvention,
    pub has_zero_reference: bool,
    pub revisions: ServiceRevisions,
    pub identity: Arc<ServiceIdentity>,
    pub snapshot: Arc<Snapshot>,
    pub prep_env: Option<Arc<super::requests::actions::ActionPrepEnv>>,
}

impl ServiceSnapshot {
    /// Worst-case arm-joint feedback age at `now` (`None` if `arm_dof == 0` or any arm joint
    /// lacks a receive instant).
    pub fn feedback_age_at(&self, now: Instant) -> Option<Duration> {
        let arm_dof = self.identity.arm_dof;
        if arm_dof == 0 || self.snapshot.rt.feedback_instants.len() < arm_dof {
            return None;
        }
        let mut oldest = Duration::ZERO;
        for slot in &self.snapshot.rt.feedback_instants[..arm_dof] {
            let rx = (*slot)?;
            oldest = oldest.max(now.saturating_duration_since(rx));
        }
        Some(oldest)
    }

    /// Maximum absolute measured velocity across arm joints `0..arm_dof` (rad/s), or `f32::INFINITY`
    /// if any arm joint velocity is non-finite.
    pub fn max_arm_joint_speed(&self) -> f32 {
        let arm_dof = self.identity.arm_dof;
        let mut max_v = 0.0f32;
        for &v in self.snapshot.rt.velocities.iter().take(arm_dof) {
            if !v.is_finite() {
                return f32::INFINITY;
            }
            max_v = max_v.max(v.abs());
        }
        max_v
    }
}

/// Per-joint feedback and owner metadata of a sample; every vector has the profile's dof.
#[derive(Debug, Clone, PartialEq)]
pub struct RtSample {
    pub positions: Vec<f32>,
    pub velocities: Vec<f32>,
    /// Commanded joint positions (profile dof) at the end of this tick.
    pub commanded_positions: Vec<f32>,
    /// Commanded joint velocities (profile dof) at the end of this tick.
    pub commanded_velocities: Vec<f32>,
    /// Hardware receive instant of each joint's latest feedback.
    pub feedback_instants: Vec<Option<Instant>>,
    /// Per-joint count of distinct feedback samples.
    pub feedback_seqs: Vec<u64>,
    /// Per-joint driver RX-queue upper bound timestamps (for multiview receive-progress fence).
    pub feedback_queue_bounds: Vec<Option<Instant>>,
    /// Live owner authority, domain revisions, flags, held state, and macro child state.
    pub owner: PublishedOwnerMeta,
    pub boot_id: super::broker::BootId,
    pub clock_now: Option<Instant>,
    pub workspace_revision: u64,
    pub motion_config_revision: u64,
    pub stop_generation: u64,
    pub lease_generation: u64,
    pub config_inhibited: bool,
    pub calibration_busy: bool,
    pub holding_after_clear: bool,
    pub watchdog_park_pending_gen: u64,
    pub watchdog_park_pending_attempt: u8,
}

impl Default for RtSample {
    fn default() -> Self { Self::with_dof(0) }
}

impl RtSample {
    fn with_dof(dof: usize) -> Self {
        Self {
            positions: Vec::with_capacity(dof),
            velocities: Vec::with_capacity(dof),
            commanded_positions: Vec::with_capacity(dof),
            commanded_velocities: Vec::with_capacity(dof),
            feedback_instants: Vec::with_capacity(dof),
            feedback_seqs: Vec::with_capacity(dof),
            feedback_queue_bounds: Vec::with_capacity(dof),
            owner: PublishedOwnerMeta::default(),
            boot_id: super::broker::BootId(0),
            clock_now: None,
            workspace_revision: 0,
            motion_config_revision: 0,
            stop_generation: 0,
            lease_generation: 0,
            config_inhibited: false,
            calibration_busy: false,
            holding_after_clear: false,
            watchdog_park_pending_gen: 0,
            watchdog_park_pending_attempt: 0,
        }
    }

    /// `SupervisorInner::arm_feedback_snapshot` of this sample: all positions, and the
    /// velocities, receive instants and feedback sequence numbers of the joints `arm`.
    pub fn arm_feedback(&self, arm: &[usize]) -> (Vec<f32>, Vec<f32>, Vec<Option<Instant>>, Vec<u64>) {
        (
            self.positions.clone(),
            arm.iter().map(|&i| self.velocities.get(i).copied().unwrap_or(0.0)).collect(),
            arm.iter().map(|&i| self.feedback_instants.get(i).copied().flatten()).collect(),
            arm.iter().map(|&i| self.feedback_seqs.get(i).copied().unwrap_or(0)).collect(),
        )
    }
}

/// Copies `src` into `dst` without allocating while `dst` has the capacity.
fn copy_into<T: Copy>(dst: &mut Vec<T>, src: &[T]) {
    dst.clear();
    dst.extend_from_slice(src);
}

/// Preallocated per-slot spares reused across `None <-> Some(...)` action message and policy
/// observation transitions so RT publication never allocates or drops heap storage.
#[derive(Debug, Default)]
pub struct ObservationSpares {
    pub action_msg: String,
    pub prev_action_msg: String,
    #[cfg(feature = "policy-runtime")]
    pub policy_obs: Option<crate::model::PolicyObservation>,
    #[cfg(feature = "policy-runtime")]
    pub policy_shadow_vec: Vec<f32>,
}

impl ObservationSpares {
    pub fn preallocated() -> Self {
        Self {
            action_msg: String::with_capacity(256),
            prev_action_msg: String::with_capacity(256),
            #[cfg(feature = "policy-runtime")]
            policy_obs: Some(crate::model::PolicyObservation::preallocated_rt()),
            #[cfg(feature = "policy-runtime")]
            policy_shadow_vec: Vec::with_capacity(super::mailbox::MAX_DOF),
        }
    }
}

/// One preallocated slot of the triple buffer.
struct Slot {
    meta: Option<SampleMeta>,
    obs: Option<ArmObservation>,
    owner: OwnerStateSnapshot,
    /// Preallocated string spares reused across state/fault/action transitions (0 alloc on RT).
    spares: ObservationSpares,
    rt: RtSample,
}

/// A sample as readers get it.
#[derive(Debug, Clone)]
pub struct Snapshot {
    pub meta: SampleMeta,
    pub obs: ArmObservation,
    pub owner: OwnerStateSnapshot,
    pub rt: RtSample,
    /// Off-RT policy diagnostic overlay revision reflected in `obs` (`0` before first update).
    pub policy_diag_seq: u64,
}

/// Off-RT policy diagnostic overlay update emitted by the executor policy runner (never touched
/// on `armd-control`). Carries Shadow step/target telemetry and Live/Shadow timing `PolicyStats`.
#[cfg(feature = "policy-runtime")]
#[derive(Debug, Clone)]
pub struct PolicyDiagUpdate {
    pub action_id: u64,
    pub is_shadow: bool,
    pub steps: u64,
    pub stats: crate::motion::PolicyStats,
    pub last_input_age_ms: Option<u64>,
    pub last_vision_age_ms: Option<u64>,
    pub vision_frame_id: Option<u64>,
    pub last_shadow_target: Option<Vec<f32>>,
}

#[cfg(feature = "policy-runtime")]
#[derive(Debug, Clone)]
struct PolicyDiagEntry {
    action_id: u64,
    is_shadow: bool,
    steps: u64,
    stats: crate::motion::PolicyStats,
    last_input_age_ms: Option<u64>,
    last_vision_age_ms: Option<u64>,
    vision_frame_id: Option<u64>,
    last_shadow_target: Option<Vec<f32>>,
    sealed: bool,
}

#[cfg(feature = "policy-runtime")]
const POLICY_DIAG_CAPACITY: usize = 64;

/// Capacity of the retire ring.
pub const RETIRE_CAPACITY: usize = 8;

/// Observations the tick displaced from unconsumed slots, handed to the readers' side to
/// be freed there: a single-producer (the tick) single-consumer (the holder of the hub
/// mutex) ring of preallocated `Option` slots, so neither side allocates or waits.
struct RetireRing {
    /// Heap-allocated once at channel creation so constructing `StateHub` does not place
    /// `RETIRE_CAPACITY` full `ArmObservation` values on the caller's stack.
    slots: Box<[UnsafeCell<Option<ArmObservation>>]>,
    head: AtomicU64,
    tail: AtomicU64,
}

// SAFETY: `slots` has fixed length `RETIRE_CAPACITY`; slot `i % CAP` is written by the producer
// only while `tail <= i < tail + CAP` (Acquire load of `tail`) and taken by the consumer only
// while `i < head` (Acquire load of `head`); the consumer side runs under the hub mutex, so it is single.
unsafe impl Sync for RetireRing {}

impl RetireRing {
    fn new() -> Self {
        let mut slots = Vec::with_capacity(RETIRE_CAPACITY);
        for _ in 0..RETIRE_CAPACITY {
            slots.push(UnsafeCell::new(None));
        }
        Self { slots: slots.into_boxed_slice(), head: AtomicU64::new(0), tail: AtomicU64::new(0) }
    }

    /// Producer: hands `obs` over (None), or gives it back when the ring is full.
    fn push(&self, head: &mut u64, obs: ArmObservation) -> Option<ArmObservation> {
        if *head - self.tail.load(Ordering::Acquire) >= RETIRE_CAPACITY as u64 { return Some(obs); }
        // SAFETY: see the Sync impl; the replaced value is None (the consumer took it).
        let old = unsafe { (*self.slots[(*head % RETIRE_CAPACITY as u64) as usize].get()).replace(obs) };
        debug_assert!(old.is_none());
        *head += 1;
        self.head.store(*head, Ordering::Release);
        None
    }

    /// Consumer (hub mutex held): frees what the producer handed over.
    fn free_all(&self) {
        let head = self.head.load(Ordering::Acquire);
        let mut tail = self.tail.load(Ordering::Relaxed);
        while tail < head {
            // SAFETY: tail < head: the producer finished this slot.
            drop(unsafe { (*self.slots[(tail % RETIRE_CAPACITY as u64) as usize].get()).take() });
            tail += 1;
        }
        self.tail.store(tail, Ordering::Release);
    }
}

/// The tick's side: owned by `ArmCore`, used under the supervisor write lock.
pub struct StatePublisher {
    producer: Producer<Slot>,
    seq: u64,
    retire_head: u64,
    hub: Arc<StateHub>,
}

/// The per-joint inputs of [`StatePublisher::publish`].
pub struct RtInputs<'a> {
    pub positions: &'a [f32],
    pub velocities: &'a [f32],
    pub commanded_positions: &'a [f32],
    pub commanded_velocities: &'a [f32],
    pub feedback_instants: &'a [Option<Instant>],
    pub feedback_seqs: &'a [u64],
    pub feedback_queue_bounds: &'a [Option<Instant>],
    pub owner: PublishedOwnerMeta,
    pub meta: RtMotionMeta,
}

impl StatePublisher {
    /// Publishes `obs` with its stamp. Copies the per-joint inputs into the slot's
    /// preallocated vectors and swaps one atomic: no allocation, no deallocation, no wait
    /// (while the dof does not grow beyond the profile's). Steady state: the tick moved the
    /// slot's observation out ([`Self::take_back_observation`]), filled it in place and hands
    /// it back here, so nothing is displaced. An observation still in the slot (a caller that
    /// built a new one) goes to the retire ring and is freed by the readers' side; only
    /// when that ring is full (no reader for [`RETIRE_CAPACITY`] such publications) it is
    /// returned, the caller drops it after releasing the lock, and `StateHub::tick_frees`
    /// counts it. Readers copy the slot's observation and never take it, so a reader never
    /// empties the tick's buffer and the tick never waits for one.
    pub fn publish(&mut self, generation: u64, sampled_at: Instant, valid_until: Instant, obs: ArmObservation, rt: RtInputs<'_>) -> Option<ArmObservation> {
        self.publish_with_owner(generation, sampled_at, valid_until, obs, OwnerStateSnapshot::default(), rt)
    }

    pub fn publish_with_owner(
        &mut self,
        generation: u64,
        sampled_at: Instant,
        valid_until: Instant,
        obs: ArmObservation,
        owner: OwnerStateSnapshot,
        rt: RtInputs<'_>,
    ) -> Option<ArmObservation> {
        self.seq += 1;
        let seq = self.seq;
        let slot = self.producer.back_mut();
        slot.meta = Some(SampleMeta { seq, generation, sampled_at, valid_until });
        slot.owner = owner;
        let displaced = slot.obs.replace(obs).and_then(|old| self.hub.retire.push(&mut self.retire_head, old));
        if displaced.is_some() { self.hub.tick_frees.fetch_add(1, Ordering::Relaxed); }
        copy_into(&mut slot.rt.positions, rt.positions);
        copy_into(&mut slot.rt.velocities, rt.velocities);
        copy_into(&mut slot.rt.commanded_positions, rt.commanded_positions);
        copy_into(&mut slot.rt.commanded_velocities, rt.commanded_velocities);
        copy_into(&mut slot.rt.feedback_instants, rt.feedback_instants);
        copy_into(&mut slot.rt.feedback_seqs, rt.feedback_seqs);
        copy_into(&mut slot.rt.feedback_queue_bounds, rt.feedback_queue_bounds);
        slot.rt.owner = rt.owner;
        slot.rt.boot_id = rt.meta.boot_id;
        slot.rt.clock_now = rt.meta.clock_now;
        slot.rt.workspace_revision = rt.meta.workspace_revision;
        slot.rt.motion_config_revision = rt.meta.motion_config_revision;
        slot.rt.stop_generation = rt.meta.stop_generation;
        slot.rt.lease_generation = rt.meta.lease_generation;
        slot.rt.config_inhibited = rt.meta.config_inhibited;
        slot.rt.calibration_busy = rt.meta.calibration_busy;
        slot.rt.holding_after_clear = rt.meta.holding_after_clear;
        slot.rt.watchdog_park_pending_gen = rt.meta.watchdog_park_pending_gen;
        slot.rt.watchdog_park_pending_attempt = rt.meta.watchdog_park_pending_attempt;
        let _ = self.producer.publish();
        self.hub.published.store(seq, Ordering::Release);
        self.hub.wake();
        displaced
    }

    pub fn hub(&self) -> &Arc<StateHub> { &self.hub }

    /// Exact sequence number the next [`Self::publish_with_owner`] will store in `hub.published`.
    #[inline]
    pub fn next_seq(&self) -> u64 { self.seq + 1 }

    /// Moves the observation out of the slot the next [`Self::publish`] writes (see
    /// `ArmCore::take_observation_buffer`).
    pub fn take_back_observation(&mut self) -> Option<ArmObservation> { self.producer.back_mut().obs.take() }

    /// Pre-populates empty triple-buffer observation slots off the release loop without blocking
    /// on a contended reader. Leaves `meta` and `published_seq` untouched.
    ///
    /// - If `hub.state` is uncontended (`try_lock`), warms all 3 slots directly.
    /// - If a reader holds `hub.state`, warms the producer's writable slot(s) without waiting
    ///   (both `back` and `middle` when nothing has been published yet via `swap_unfresh`) and
    ///   stores a template in `hub.prewarm_template` so `StateHub::refresh` warms `front`
    ///   off-thread before rotating it back to the producer.
    pub fn try_prewarm_observations(&mut self, mut make: impl FnMut() -> ArmObservation) {
        {
            let mut g = self
                .hub
                .prewarm_template
                .write()
                .unwrap_or_else(|e| e.into_inner());
            *g = Some(Box::new(make()));
        }
        match self.hub.state.try_lock() {
            Ok(mut st) => {
                self.producer.for_each_slot_mut(&mut st.consumer, |slot| {
                    if slot.obs.is_none() {
                        slot.obs = Some(make());
                    }
                });
            }
            Err(std::sync::TryLockError::Poisoned(p)) => {
                let mut st = p.into_inner();
                self.producer.for_each_slot_mut(&mut st.consumer, |slot| {
                    if slot.obs.is_none() {
                        slot.obs = Some(make());
                    }
                });
            }
            Err(std::sync::TryLockError::WouldBlock) => {
                let back = self.producer.back_mut();
                if back.obs.is_none() {
                    back.obs = Some(make());
                }
                if self.producer.swap_unfresh() {
                    let other = self.producer.back_mut();
                    if other.obs.is_none() {
                        other.obs = Some(make());
                    }
                }
            }
        }
    }

    /// Installs privately staged pre-warmed `back` and `middle` observation buffers into the
    /// producer's own slots without locking `hub.state` or allocating on `armd-control`.
    /// Returns any unused buffers so the caller can retire them off-RT (zero RT frees).
    pub fn install_prewarmed_producer_slots(
        &mut self,
        back_obs: ArmObservation,
        mid_obs: ArmObservation,
    ) -> (Option<ArmObservation>, Option<ArmObservation>) {
        let unused_back = {
            let back = self.producer.back_mut();
            if back.obs.is_none() {
                back.obs = Some(back_obs);
                None
            } else {
                Some(back_obs)
            }
        };
        let unused_mid = if self.producer.swap_unfresh() {
            let other = self.producer.back_mut();
            if other.obs.is_none() {
                other.obs = Some(mid_obs);
                None
            } else {
                Some(mid_obs)
            }
        } else {
            Some(mid_obs)
        };
        (unused_back, unused_mid)
    }

    /// Moves the preallocated string spares out of the back slot (0 alloc).
    pub fn take_back_spares(&mut self) -> ObservationSpares {
        std::mem::take(&mut self.producer.back_mut().spares)
    }

    /// Returns the string spares to the back slot before publishing (0 alloc, 0 drop).
    pub fn restore_back_spares(&mut self, mut spares: ObservationSpares) {
        let slot = self.producer.back_mut();
        if spares.action_msg.capacity() > slot.spares.action_msg.capacity() {
            slot.spares.action_msg = std::mem::take(&mut spares.action_msg);
        }
        if spares.prev_action_msg.capacity() > slot.spares.prev_action_msg.capacity() {
            slot.spares.prev_action_msg = std::mem::take(&mut spares.prev_action_msg);
        }
        #[cfg(feature = "policy-runtime")]
        {
            if spares.policy_obs.is_some() && slot.spares.policy_obs.is_none() {
                slot.spares.policy_obs = spares.policy_obs.take();
            }
            if spares.policy_shadow_vec.capacity() > slot.spares.policy_shadow_vec.capacity() {
                slot.spares.policy_shadow_vec = std::mem::take(&mut spares.policy_shadow_vec);
            }
        }
    }

    /// Puts `obs` into the back slot when it is empty (else back to the retire ring; never
    /// freed here when that ring has room). `ArmCore::publish`'s fallback for a displaced
    /// observation, which its encapsulation makes unreachable.
    pub(super) fn restore_back_observation(&mut self, obs: ArmObservation) {
        let slot = self.producer.back_mut();
        if slot.obs.is_none() { slot.obs = Some(obs); return; }
        if let Some(back) = self.hub.retire.push(&mut self.retire_head, obs) {
            self.hub.tick_frees.fetch_add(1, Ordering::Relaxed);
            drop(back);
        }
    }
}

struct HubState {
    consumer: Consumer<Slot>,
    latest: Option<Arc<Snapshot>>,
    last_policy_diag_seq: u64,
}

/// Result of [`StateHub::try_latest`].
pub enum Poll {
    Ready(Option<Arc<Snapshot>>),
    Busy,
}

/// The readers' side (the dispatcher): the newest sample, and the wake-up of the dispatcher
/// thread that forwards samples to subscribers.
pub struct StateHub {
    state: Mutex<HubState>,
    /// `seq` of the newest publication (0: none yet).
    published: AtomicU64,
    dispatcher: OnceLock<std::thread::Thread>,
    closed: AtomicBool,
    /// The recording ring, drained by the dispatcher (set by the first recording).
    record: OnceLock<Arc<super::record_ring::RecordRing>>,
    retire: RetireRing,
    /// Displaced observations the tick had to free itself (retire ring full).
    tick_frees: AtomicU64,
    /// Pre-sized observation template recorded atomically at executor startup activation so a
    /// reader warms its own `front` slot off-thread before handing it back to the RT producer.
    prewarm_template: std::sync::RwLock<Option<Box<ArmObservation>>>,
    /// Immutable motion planning config snapshot updated off-tick for zero-lock workers.
    motion_config: std::sync::RwLock<Option<Arc<super::requests::motion::MotionConfigSnapshot>>>,
    /// Off-thread schema cache paired with [`OwnerStateSnapshot::schema_revision`]. Never locked
    /// on the RT control thread.
    schema_cache: Mutex<Option<(u64, Arc<ObservationSchema>)>>,
    /// Revision-tagged slow-changing planning environment published off-thread (never touched
    /// on `armd-control`; read by off-thread action preparation and macro orchestrators).
    prep_env: Mutex<Option<Arc<super::requests::actions::ActionPrepEnv>>>,
    /// Off-RT arm identity and joint-zero metadata paired with RT `PublishedOwnerMeta` by
    /// `(arm, boot, tool_digest_hash, calibration_revision)`. Never locked on `armd-control`.
    service_identity: std::sync::RwLock<Option<Arc<ServiceIdentity>>>,
    /// Bounded off-RT sidecar store of `measure_multiview` measurement payloads keyed by action
    /// ID (never touched on `armd-control`; bounded to 16 entries). Attached in [`Self::refresh`]
    /// to `obs.action` and `obs.previous_action` before any snapshot or terminal is delivered.
    action_measurements: Mutex<std::collections::HashMap<u64, serde_json::Value>>,
    /// Pre-publication policy diagnostic seal watermark shared with `SupervisorInner` (never
    /// locked on `armd-control`).
    #[cfg(feature = "policy-runtime")]
    policy_diag_sealed: OnceLock<Arc<AtomicU64>>,
    /// Bounded off-RT sidecar store of runner-emitted Shadow diagnostics and `PolicyStats` keyed
    /// by `action_id` (cap 64; never touched on `armd-control`). Lock order: `state -> policy_diag`.
    #[cfg(feature = "policy-runtime")]
    policy_diag: Mutex<std::collections::VecDeque<PolicyDiagEntry>>,
    /// Monotonic revision bumped on every accepted [`Self::update_policy_diag`] call.
    #[cfg(feature = "policy-runtime")]
    policy_diag_seq: AtomicU64,
    /// Number of off-RT readers currently waiting to acquire `self.state` inside `latest()`.
    latest_waiters: AtomicUsize,
}

/// A publisher for an arm with `dof` joints and its hub.
pub fn state_channel(dof: usize) -> (StatePublisher, Arc<StateHub>) {
    let slot = || Slot {
        meta: None,
        obs: None,
        owner: OwnerStateSnapshot::default(),
        spares: ObservationSpares::preallocated(),
        rt: RtSample::with_dof(dof),
    };
    let (producer, consumer) = triple_buffer(slot(), slot(), slot());
    let hub = Arc::new(StateHub {
        state: Mutex::new(HubState { consumer, latest: None, last_policy_diag_seq: 0 }),
        published: AtomicU64::new(0),
        dispatcher: OnceLock::new(),
        closed: AtomicBool::new(false),
        record: OnceLock::new(),
        retire: RetireRing::new(),
        tick_frees: AtomicU64::new(0),
        prewarm_template: std::sync::RwLock::new(None),
        motion_config: std::sync::RwLock::new(None),
        schema_cache: Mutex::new(None),
        prep_env: Mutex::new(None),
        service_identity: std::sync::RwLock::new(None),
        action_measurements: Mutex::new(std::collections::HashMap::new()),
        #[cfg(feature = "policy-runtime")]
        policy_diag_sealed: OnceLock::new(),
        #[cfg(feature = "policy-runtime")]
        policy_diag: Mutex::new(std::collections::VecDeque::with_capacity(POLICY_DIAG_CAPACITY)),
        #[cfg(feature = "policy-runtime")]
        policy_diag_seq: AtomicU64::new(0),
        latest_waiters: AtomicUsize::new(0),
    });
    (StatePublisher { producer, seq: 0, retire_head: 0, hub: hub.clone() }, hub)
}

impl StateHub {
    /// The newest published sample (None before the first tick). Takes the hub mutex,
    /// which only readers and the dispatcher share; the first reader after a publication
    /// copies the observation out of the slot (the copy is allocated and freed on the reader
    /// side; the slot keeps its memory, which the tick fills again).
    pub fn latest(&self) -> Option<Arc<Snapshot>> {
        struct WaiterGuard<'a>(&'a AtomicUsize);
        impl Drop for WaiterGuard<'_> {
            fn drop(&mut self) {
                self.0.fetch_sub(1, Ordering::Relaxed);
            }
        }
        self.latest_waiters.fetch_add(1, Ordering::Release);
        let _waiter = WaiterGuard(&self.latest_waiters);
        let mut st = self.state.lock().unwrap_or_else(|e| e.into_inner());
        drop(_waiter);
        self.refresh(&mut st)
    }

    /// Number of threads currently waiting on the hub mutex inside [`Self::latest`] (tests).
    #[doc(hidden)]
    pub fn latest_waiters(&self) -> usize {
        self.latest_waiters.load(Ordering::Acquire)
    }

    /// Displaced observations the tick freed itself because no reader drained the retire ring.
    pub fn tick_frees(&self) -> u64 { self.tick_frees.load(Ordering::Relaxed) }

    /// Frees the observations the tick retired and drains any deferred RT stop diagnostics
    /// (hub mutex held on the normal-priority dispatcher / reader path, never the RT tick).
    fn refresh(&self, st: &mut HubState) -> Option<Arc<Snapshot>> {
        self.retire.free_all();
        super::ops::stop_diag().drain_and_log_once();
        super::ops::gripper_force_diag().drain_and_log_once();
        // Acquire-check `has_fresh()` BEFORE inspecting `prewarm_template`: in executor mode
        // `try_prewarm_observations` sets `prewarm_template` (Release) before any release
        // calls `Producer::publish` (AcqRel), so observing `FRESH == 1` guarantees
        // `prewarm_template` is visible and prevents rotating an empty `front` slot into
        // `middle` for the RT producer.
        let has_fresh = st.consumer.has_fresh();
        let diag_seq = self.policy_diag_seq();
        let diag_changed = diag_seq != st.last_policy_diag_seq;
        let mut refreshed = false;
        if has_fresh {
            if st.consumer.front().obs.is_none() {
                let tmpl_guard = self
                    .prewarm_template
                    .read()
                    .unwrap_or_else(|e| e.into_inner());
                if let Some(tmpl) = tmpl_guard.as_ref() {
                    let mut obs = (**tmpl).clone();
                    obs.ensure_rt_safety_text_spare();
                    let dof = obs.positions.len();
                    obs.gravity_torques.reserve(dof);
                    obs.motor_errors.reserve(dof);
                    st.consumer.front_mut().obs = Some(obs);
                }
            }
            refreshed = st.consumer.refresh();
        }
        if refreshed || (diag_changed && st.consumer.front().obs.is_some()) {
            let slot = st.consumer.front_mut();
            // copied, not moved: the slot keeps its memory for the tick to fill again (the
            // tick neither allocates nor frees observations)
            if let (Some(meta), Some(obs)) = (slot.meta, slot.obs.as_ref()) {
                let mut obs = obs.clone();
                self.attach_action_measurement(&mut obs);
                self.attach_policy_diag(&mut obs);
                st.last_policy_diag_seq = diag_seq;
                let snap = Snapshot {
                    meta,
                    obs,
                    owner: slot.owner,
                    rt: slot.rt.clone(),
                    policy_diag_seq: diag_seq,
                };
                st.latest = Some(Arc::new(snap));
            }
        }
        st.latest.clone()
    }

    /// Holds the readers' mutex (tests: a preempted reader).
    #[doc(hidden)]
    pub fn reader_guard(&self) -> impl Drop + '_ { self.state.lock().unwrap_or_else(|e| e.into_inner()) }

    /// [`latest`](Self::latest) without waiting: `Busy` while another reader holds the hub.
    pub fn try_latest(&self) -> Poll {
        match self.state.try_lock() {
            Ok(mut st) => Poll::Ready(self.refresh(&mut st)),
            Err(std::sync::TryLockError::Poisoned(p)) => Poll::Ready(self.refresh(&mut p.into_inner())),
            Err(std::sync::TryLockError::WouldBlock) => Poll::Busy,
        }
    }

    /// `seq` of the newest publication, without the hub mutex (0: none yet).
    pub fn published_seq(&self) -> u64 { self.published.load(Ordering::Acquire) }

    /// Monotonic sequence of off-RT policy diagnostic overlay updates (`0` before first update).
    pub fn policy_diag_seq(&self) -> u64 {
        #[cfg(feature = "policy-runtime")]
        {
            self.policy_diag_seq.load(Ordering::Acquire)
        }
        #[cfg(not(feature = "policy-runtime"))]
        {
            0
        }
    }

    /// Binds the RT pre-publication policy diagnostic seal watermark (`AtomicU64`).
    #[cfg(feature = "policy-runtime")]
    pub fn attach_policy_diag_seal(&self, seal: Arc<AtomicU64>) {
        let _ = self.policy_diag_sealed.set(seal);
    }

    /// Updates the off-RT policy diagnostic overlay for `update.action_id` and wakes `armd-dispatch`.
    /// Returns `false` (no-op) once RT seals `action_id` or a terminal snapshot freezes the entry.
    #[cfg(feature = "policy-runtime")]
    pub fn update_policy_diag(&self, update: PolicyDiagUpdate) -> bool {
        if update.action_id == 0 {
            return false;
        }
        {
            let mut entries = self.policy_diag.lock().unwrap_or_else(|e| e.into_inner());
            let sealed = self
                .policy_diag_sealed
                .get()
                .map_or(0, |a| a.load(Ordering::Acquire));
            if update.action_id <= sealed {
                return false;
            }
            if let Some(existing) = entries.iter_mut().find(|e| e.action_id == update.action_id) {
                if existing.sealed {
                    return false;
                }
                existing.is_shadow = update.is_shadow;
                existing.steps = existing.steps.max(update.steps);
                existing.stats = update.stats;
                existing.last_input_age_ms = update.last_input_age_ms;
                existing.last_vision_age_ms = update.last_vision_age_ms;
                existing.vision_frame_id = update.vision_frame_id;
                existing.last_shadow_target = update.last_shadow_target;
            } else {
                if entries.len() >= POLICY_DIAG_CAPACITY {
                    entries.pop_front();
                }
                entries.push_back(PolicyDiagEntry {
                    action_id: update.action_id,
                    is_shadow: update.is_shadow,
                    steps: update.steps,
                    stats: update.stats,
                    last_input_age_ms: update.last_input_age_ms,
                    last_vision_age_ms: update.last_vision_age_ms,
                    vision_frame_id: update.vision_frame_id,
                    last_shadow_target: update.last_shadow_target,
                    sealed: false,
                });
            }
            self.policy_diag_seq.fetch_add(1, Ordering::Release);
        }
        self.wake();
        true
    }

    /// Freezes sealed/inactive policy diagnostic entries and enriches `obs` in place (called in
    /// [`Self::refresh`] and in `MaintClientInner::pop_all_replies` before delivering `Observation`).
    #[cfg(feature = "policy-runtime")]
    pub fn attach_policy_diag(&self, obs: &mut ArmObservation) {
        let mut entries = self.policy_diag.lock().unwrap_or_else(|e| e.into_inner());
        let sealed = self
            .policy_diag_sealed
            .get()
            .map_or(0, |a| a.load(Ordering::Acquire));
        if sealed > 0 {
            for e in entries.iter_mut() {
                if e.action_id <= sealed {
                    e.sealed = true;
                }
            }
        }
        if let Some(ref p) = obs.policy {
            if !p.active {
                if let Some(e) = entries.iter_mut().find(|e| e.action_id == p.action_id) {
                    e.sealed = true;
                }
            }
        }
        if entries.is_empty() {
            return;
        }
        if let Some(a) = obs
            .action
            .as_mut()
            .filter(|a| a.kind.as_str() == "policy" && a.policy_stats.is_none())
        {
            if let Some(e) = entries.iter().find(|e| e.action_id == a.id) {
                a.policy_stats = Some(e.stats);
            }
        }
        if let Some(a) = obs
            .previous_action
            .as_mut()
            .filter(|a| a.kind.as_str() == "policy" && a.policy_stats.is_none())
        {
            if let Some(e) = entries.iter().find(|e| e.action_id == a.id) {
                a.policy_stats = Some(e.stats);
            }
        }
        if let Some(p) = obs.policy.as_mut() {
            if let Some(e) = entries.iter().find(|e| e.action_id == p.action_id) {
                if e.is_shadow || p.mode == "shadow" {
                    p.steps = p.steps.max(e.steps);
                    if e.last_input_age_ms.is_some() {
                        p.last_input_age_ms = e.last_input_age_ms;
                    }
                    if e.last_vision_age_ms.is_some() {
                        p.last_vision_age_ms = e.last_vision_age_ms;
                    }
                    if e.vision_frame_id.is_some() {
                        p.vision_frame_id = e.vision_frame_id;
                    }
                    if e.last_shadow_target.is_some() {
                        p.last_shadow_target = e.last_shadow_target.clone();
                    }
                }
            }
        }
    }

    #[cfg(not(feature = "policy-runtime"))]
    pub fn attach_policy_diag(&self, _obs: &mut ArmObservation) {}

    /// Registers the dispatcher thread woken by every publication. Only the first call counts.
    pub fn set_dispatcher(&self, thread: std::thread::Thread) -> bool { self.dispatcher.set(thread).is_ok() }

    pub fn has_dispatcher(&self) -> bool { self.dispatcher.get().is_some() }

    /// Wakes the dispatcher thread: an atomic swap and, when it sleeps, one futex wake.
    pub fn wake(&self) {
        if let Some(t) = self.dispatcher.get() { t.unpark(); }
    }

    /// Registers the recording ring the dispatcher drains (the first call counts).
    pub fn set_record_ring(&self, ring: Arc<super::record_ring::RecordRing>) { let _ = self.record.set(ring); }

    pub fn record_ring(&self) -> Option<&Arc<super::record_ring::RecordRing>> { self.record.get() }

    /// Ends the dispatcher thread (the supervisor is dropped).
    pub fn close(&self) {
        self.closed.store(true, Ordering::Release);
        self.wake();
    }

    pub fn is_closed(&self) -> bool { self.closed.load(Ordering::Acquire) }

    /// Atomically installs the pre-warmed observation template and warms the hub's `front` slot
    /// during startup activation off `armd-control` (never called before startup commit wins).
    pub fn install_prewarm_template_and_front(
        &self,
        tmpl: Box<ArmObservation>,
        front_obs: ArmObservation,
    ) {
        {
            let mut g = self
                .prewarm_template
                .write()
                .unwrap_or_else(|e| e.into_inner());
            *g = Some(tmpl);
        }
        if let Ok(mut st) = self.state.try_lock() {
            if st.consumer.front().obs.is_none() {
                st.consumer.front_mut().obs = Some(front_obs);
            }
        }
    }

    /// Returns true if a pre-warmed observation template has been installed on the hub.
    #[doc(hidden)]
    pub fn has_prewarm_template(&self) -> bool {
        self.prewarm_template
            .read()
            .unwrap_or_else(|e| e.into_inner())
            .is_some()
    }

    /// Publishes an immutable motion planning config snapshot off the RT tick.
    pub fn set_motion_config(&self, cfg: Arc<super::requests::motion::MotionConfigSnapshot>) {
        *self.motion_config.write().unwrap_or_else(|e| e.into_inner()) = Some(cfg);
    }

    /// Reads the latest immutable motion planning config snapshot without touching the owner lock.
    pub fn motion_config(&self) -> Option<Arc<super::requests::motion::MotionConfigSnapshot>> {
        self.motion_config.read().unwrap_or_else(|e| e.into_inner()).clone()
    }

    /// Records the off-thread [`ObservationSchema`] corresponding to `revision`.
    pub fn set_schema_cache(&self, revision: u64, schema: Arc<ObservationSchema>) {
        let mut g = self.schema_cache.lock().unwrap_or_else(|e| e.into_inner());
        if g.as_ref().is_none_or(|(rev, _)| revision >= *rev) {
            *g = Some((revision, schema));
        }
    }

    /// Returns the cached [`ObservationSchema`] only when its revision matches `revision`.
    pub fn cached_schema(&self, revision: u64) -> Option<Arc<ObservationSchema>> {
        if revision == 0 {
            return None;
        }
        let g = self.schema_cache.lock().unwrap_or_else(|e| e.into_inner());
        g.as_ref()
            .filter(|(rev, _)| *rev == revision)
            .map(|(_, schema)| Arc::clone(schema))
    }

    /// Returns the newest cached [`ObservationSchema`] regardless of revision (in-process fallback).
    pub fn latest_cached_schema(&self) -> Option<Arc<ObservationSchema>> {
        let g = self.schema_cache.lock().unwrap_or_else(|e| e.into_inner());
        g.as_ref().map(|(_, schema)| Arc::clone(schema))
    }

    /// Publishes an updated off-RT revision-tagged planning environment (never called on
    /// `armd-control`).
    pub fn update_prep_env(&self, env: Arc<super::requests::actions::ActionPrepEnv>) {
        *self.prep_env.lock().unwrap_or_else(|e| e.into_inner()) = Some(env);
    }

    /// Reads the latest published off-RT revision-tagged planning environment.
    pub fn prep_env(&self) -> Option<Arc<super::requests::actions::ActionPrepEnv>> {
        self.prep_env.lock().unwrap_or_else(|e| e.into_inner()).clone()
    }

    /// Publishes an updated off-RT [`ServiceIdentity`] (never called on `armd-control`).
    pub fn set_service_identity(&self, identity: Arc<ServiceIdentity>) {
        *self
            .service_identity
            .write()
            .unwrap_or_else(|e| e.into_inner()) = Some(identity);
    }

    /// Reads the latest published off-RT [`ServiceIdentity`].
    pub fn service_identity(&self) -> Option<Arc<ServiceIdentity>> {
        self.service_identity
            .read()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
    }

    /// Updates off-RT [`ServiceIdentity`] from a [`PublishedConfig`](super::requests::config::PublishedConfig),
    /// preserving the active owner's `(arm, boot)` identity. Never called on `armd-control`.
    pub fn update_service_identity_from_config(
        &self,
        cfg: &super::requests::config::PublishedConfig,
    ) {
        let mut g = self
            .service_identity
            .write()
            .unwrap_or_else(|e| e.into_inner());
        let (arm, boot) = g
            .as_ref()
            .map(|cur| (cur.arm, cur.boot))
            .unwrap_or((ArmId::LOCAL, BootId(0)));
        *g = Some(Arc::new(ServiceIdentity::from_published_config(arm, boot, cfg)));
    }

    /// Reads a coherent [`ServiceSnapshot`] from the latest triple-buffer sample and off-RT
    /// identity/planning metadata, failing closed if any requirement in `req` or any revision
    /// authentication check fails. Never acquires `SupervisorLock` and never blocks `armd-control`.
    pub fn read_services(
        &self,
        req: ReadRequirements,
    ) -> Result<Arc<ServiceSnapshot>, ServiceReadError> {
        let snap = self.latest().ok_or(ServiceReadError::NoSnapshot)?;
        let identity = self
            .service_identity()
            .ok_or(ServiceReadError::IdentityUnavailable)?;
        let prep_env = if req.require_prep_env {
            Some(self.prep_env().ok_or(ServiceReadError::PrepEnvUnavailable)?)
        } else {
            None
        };
        Self::build_service_snapshot_from(snap, identity, prep_env, req, Instant::now())
    }

    /// Single-authority validator and assembler for [`ServiceSnapshot`] shared by
    /// [`Self::read_services`] and the legacy no-ticker read fallback.
    pub fn build_service_snapshot_from(
        snap: Arc<Snapshot>,
        identity: Arc<ServiceIdentity>,
        prep_env: Option<Arc<super::requests::actions::ActionPrepEnv>>,
        req: ReadRequirements,
        now: Instant,
    ) -> Result<Arc<ServiceSnapshot>, ServiceReadError> {
        if let Some(min_gen) = req.min_generation {
            if snap.meta.generation < min_gen {
                return Err(ServiceReadError::GenerationNotReached {
                    required: min_gen,
                    actual: snap.meta.generation,
                });
            }
        }
        if let Some(min_seq) = req.min_publish_seq {
            if snap.meta.seq < min_seq {
                return Err(ServiceReadError::SequenceNotReached {
                    required: min_seq,
                    actual: snap.meta.seq,
                });
            }
        }
        if let Some(min_rel) = req.min_release_seq {
            if snap.rt.owner.control_release_seq < min_rel {
                return Err(ServiceReadError::ReleaseSequenceNotReached {
                    required: min_rel,
                    actual: snap.rt.owner.control_release_seq,
                });
            }
        }
        if req.require_within_valid_until && now > snap.meta.valid_until {
            return Err(ServiceReadError::SnapshotExpired);
        }
        if let Some(max_age) = req.max_snapshot_age {
            let age = now.saturating_duration_since(snap.meta.sampled_at);
            if age > max_age {
                return Err(ServiceReadError::SnapshotStale {
                    age_ms: age.as_millis() as u64,
                    max_ms: max_age.as_millis() as u64,
                });
            }
        }
        if !identity.matches_owner_meta(&snap.rt.owner) {
            return Err(ServiceReadError::IdentityMismatch);
        }
        let prep_env = if req.require_prep_env {
            let env = prep_env.ok_or(ServiceReadError::PrepEnvUnavailable)?;
            if !env.matches_owner_meta(&snap.rt.owner) {
                return Err(ServiceReadError::PrepEnvMismatch);
            }
            Some(env)
        } else {
            None
        };
        if req.require_feedback_valid && !snap.rt.owner.feedback_valid {
            return Err(ServiceReadError::FeedbackInvalid);
        }
        let arm_dof = identity.arm_dof;
        let mut feedback_age_at_capture = if arm_dof > 0 && snap.rt.feedback_instants.len() >= arm_dof {
            Some(Duration::ZERO)
        } else {
            None
        };
        if let Some(ref mut oldest_at_capture) = feedback_age_at_capture {
            for slot in &snap.rt.feedback_instants[..arm_dof] {
                match *slot {
                    Some(rx) => {
                        *oldest_at_capture =
                            (*oldest_at_capture).max(snap.meta.sampled_at.saturating_duration_since(rx));
                    }
                    None => {
                        feedback_age_at_capture = None;
                        break;
                    }
                }
            }
        }
        if let Some(max_fb_age) = req.max_feedback_age {
            if arm_dof == 0 || snap.rt.feedback_instants.len() < arm_dof {
                return Err(ServiceReadError::FeedbackStale { joint: 0 });
            }
            for (joint, slot) in snap.rt.feedback_instants[..arm_dof].iter().enumerate() {
                let Some(rx) = *slot else {
                    return Err(ServiceReadError::FeedbackStale { joint });
                };
                if now.saturating_duration_since(rx) > max_fb_age {
                    return Err(ServiceReadError::FeedbackStale { joint });
                }
            }
        }
        let motion_active = snap.rt.owner.motion_active;
        let motion_idle = !motion_active
            && !snap.rt.owner.gravity_comp_active
            && !snap.rt.owner.policy_busy
            && !snap.rt.owner.pending_normal_unload
            && !snap.rt.owner.handover_settling
            && !snap.owner.torque_transition_pending
            && !snap.rt.owner.calibration_busy;
        if req.require_motion_idle && !motion_idle {
            return Err(ServiceReadError::MotionActive);
        }
        if let Some(max_speed) = req.max_arm_joint_speed {
            if !max_speed.is_finite() || max_speed < 0.0 || arm_dof == 0 || snap.rt.velocities.len() < arm_dof {
                return Err(ServiceReadError::JointMoving { joint: 0 });
            }
            for (joint, &v) in snap.rt.velocities[..arm_dof].iter().enumerate() {
                if !v.is_finite() || v.abs() > max_speed {
                    return Err(ServiceReadError::JointMoving { joint });
                }
            }
        }
        let revisions = ServiceRevisions::from_snapshot(&snap);
        Ok(Arc::new(ServiceSnapshot {
            boot: snap.rt.owner.boot,
            arm: snap.rt.owner.arm,
            publish_seq: snap.meta.seq,
            release_seq: snap.rt.owner.control_release_seq,
            captured_at: snap.meta.sampled_at,
            valid_until: snap.meta.valid_until,
            feedback_valid: snap.rt.owner.feedback_valid,
            feedback_age_at_capture,
            motion_generation: snap.rt.owner.motion_generation,
            motion_active,
            motion_idle,
            applied_stop_generation: snap.rt.stop_generation,
            epoch_id: snap.rt.owner.authority.epoch,
            lease_generation: snap.rt.owner.authority.lease_generation,
            zero_convention: snap.owner.zero_convention,
            has_zero_reference: snap.owner.has_zero_reference,
            revisions,
            identity,
            snapshot: snap,
            prep_env,
        }))
    }

    /// Records an off-RT measurement payload for action `id` (bounded to `TERMINAL_FIFO_CAPACITY`
    /// entries so sidecars survive across the full terminal reservation window) and patches
    /// the cached latest snapshot if it already carries `id` without a measurement. Never called
    /// on `armd-control`; lock order is strictly `state -> action_measurements` (never nested in
    /// reverse).
    pub fn record_action_measurement(&self, id: u64, value: serde_json::Value) {
        {
            let mut map = self
                .action_measurements
                .lock()
                .unwrap_or_else(|e| e.into_inner());
            map.insert(id, value.clone());
            while map.len() > crate::supervisor::TERMINAL_FIFO_CAPACITY {
                if let Some(&oldest) = map.keys().min() {
                    map.remove(&oldest);
                } else {
                    break;
                }
            }
        }
        let mut st = self.state.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(latest) = st.latest.as_ref() {
            let needs_patch = latest
                .obs
                .action
                .as_ref()
                .is_some_and(|a| a.id == id && a.measurement.is_none())
                || latest
                    .obs
                    .previous_action
                    .as_ref()
                    .is_some_and(|a| a.id == id && a.measurement.is_none());
            if needs_patch {
                let mut snap = (**latest).clone();
                if let Some(a) = snap
                    .obs
                    .action
                    .as_mut()
                    .filter(|a| a.id == id && a.measurement.is_none())
                {
                    a.measurement = Some(value.clone());
                }
                if let Some(a) = snap
                    .obs
                    .previous_action
                    .as_mut()
                    .filter(|a| a.id == id && a.measurement.is_none())
                {
                    a.measurement = Some(value);
                }
                st.latest = Some(Arc::new(snap));
            }
        }
    }

    /// Reads back the off-RT sidecar measurement payload recorded for action `id`, if present.
    pub fn action_measurement(&self, id: u64) -> Option<serde_json::Value> {
        self.action_measurements
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(&id)
            .cloned()
    }

    /// Populates `action.measurement` from the off-RT sidecar store when missing.
    pub fn attach_action_measurement_to_action(&self, action: &mut crate::motion::ActionStatus) {
        if action.measurement.is_some() {
            return;
        }
        let map = self
            .action_measurements
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        if let Some(v) = map.get(&action.id) {
            action.measurement = Some(v.clone());
        }
    }

    /// Populates `obs.action.measurement` and `obs.previous_action.measurement` from the off-RT
    /// sidecar store when missing.
    pub fn attach_action_measurement(&self, obs: &mut ArmObservation) {
        let need_cur = obs.action.as_ref().is_some_and(|a| a.measurement.is_none());
        let need_prev = obs
            .previous_action
            .as_ref()
            .is_some_and(|a| a.measurement.is_none());
        if !need_cur && !need_prev {
            return;
        }
        let map = self
            .action_measurements
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        if map.is_empty() {
            return;
        }
        if let Some(a) = obs.action.as_mut().filter(|a| a.measurement.is_none()) {
            if let Some(v) = map.get(&a.id) {
                a.measurement = Some(v.clone());
            }
        }
        if let Some(a) = obs
            .previous_action
            .as_mut()
            .filter(|a| a.measurement.is_none())
        {
            if let Some(v) = map.get(&a.id) {
                a.measurement = Some(v.clone());
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn obs(ts: f64) -> ArmObservation {
        let mut o = crate::supervisor::SupervisorInner::new(crate::ArmConfig::rebot_dm()).current_observation();
        o.timestamp = ts;
        o
    }

    fn inputs<'a>(p: &'a [f32], s: &'a [u64], i: &'a [Option<Instant>]) -> RtInputs<'a> {
        RtInputs {
            positions: p,
            velocities: p,
            commanded_positions: p,
            commanded_velocities: p,
            feedback_instants: i,
            feedback_seqs: s,
            feedback_queue_bounds: &[],
            owner: PublishedOwnerMeta::default(),
            meta: RtMotionMeta::default(),
        }
    }

    #[test]
    fn readers_get_the_newest_sample_with_its_stamp() {
        let (mut publisher, hub) = state_channel(7);
        assert!(hub.latest().is_none());
        let now = Instant::now();
        let p = [0.5f32; 7];
        let s = [3u64; 7];
        let i = [Some(now); 7];
        assert!(publisher.publish(10, now, now + Duration::from_millis(40), obs(1.0), inputs(&p, &s, &i)).is_none());
        assert!(publisher.publish(12, now, now + Duration::from_millis(40), obs(2.0), inputs(&p, &s, &i)).is_none());
        let snap = hub.latest().unwrap();
        assert_eq!((snap.meta.seq, snap.meta.generation, snap.obs.timestamp), (2, 12, 2.0));
        assert_eq!(snap.rt.positions, p);
        assert_eq!(snap.rt.feedback_seqs, s);
        assert_eq!(hub.published_seq(), 2);
        // nothing new: the same Arc
        assert!(Arc::ptr_eq(&snap, &hub.latest().unwrap()));
        // slots rotate: the unconsumed seq-1 observation goes to the retire ring, not back to
        // the tick; the next reader frees it
        assert!(publisher.publish(13, now, now, obs(3.0), inputs(&p, &s, &i)).is_none());
        assert!(publisher.publish(14, now, now, obs(4.0), inputs(&p, &s, &i)).is_none());
        assert_eq!(hub.retire.head.load(Ordering::Relaxed), 1);
        assert_eq!(hub.latest().unwrap().obs.timestamp, 4.0);
        assert_eq!(hub.retire.tail.load(Ordering::Relaxed), 1);
        assert_eq!(hub.tick_frees(), 0);
    }

    #[test]
    fn without_readers_the_retire_ring_fills_and_the_tick_frees_are_counted() {
        let (mut publisher, hub) = state_channel(7);
        let now = Instant::now();
        let (p, s, i) = ([0.0f32; 7], [0u64; 7], [None; 7]);
        let mut back = 0;
        // the first two publications fill empty slots; each later one displaces one
        for k in 0..(2 + RETIRE_CAPACITY + 3) {
            if publisher.publish(k as u64, now, now, obs(k as f64), inputs(&p, &s, &i)).is_some() { back += 1; }
        }
        assert_eq!((back, hub.tick_frees()), (3, 3));
        hub.latest();
        assert!(publisher.publish(99, now, now, obs(99.0), inputs(&p, &s, &i)).is_none(), "room again after a reader");
        assert_eq!(hub.latest().unwrap().obs.timestamp, 99.0);
    }

    #[test]
    fn arm_feedback_matches_the_supervisor_view() {
        let (mut publisher, hub) = state_channel(3);
        let now = Instant::now();
        publisher.publish(1, now, now, obs(0.0), RtInputs {
            positions: &[1.0, 2.0, 3.0], velocities: &[0.1, 0.2, 0.3],
            commanded_positions: &[1.0, 2.0, 3.0], commanded_velocities: &[0.0, 0.0, 0.0],
            feedback_instants: &[Some(now), None, Some(now)], feedback_seqs: &[5, 6, 7],
            feedback_queue_bounds: &[], owner: PublishedOwnerMeta::default(),
            meta: RtMotionMeta::default(),
        });
        let (p, v, i, s) = hub.latest().unwrap().rt.arm_feedback(&[0, 2]);
        assert_eq!((p, v, i, s), (vec![1.0, 2.0, 3.0], vec![0.1, 0.3], vec![Some(now), Some(now)], vec![5, 7]));
    }

    #[test]
    fn startup_reader_holding_lock_before_prewarm_never_rotates_empty_front_to_producer() {
        let (mut publisher, hub) = state_channel(7);
        let now = Instant::now();
        let (p, s, i) = ([0.0f32; 7], [1u64; 7], [Some(now); 7]);

        // Reader acquires `hub.state` BEFORE `try_prewarm_observations` runs and pauses.
        let (entered_tx, entered_rx) = std::sync::mpsc::sync_channel::<()>(1);
        let (resume_tx, resume_rx) = std::sync::mpsc::sync_channel::<()>(1);
        let hub_r = Arc::clone(&hub);
        let reader = std::thread::spawn(move || {
            let mut st = hub_r.state.lock().unwrap();
            assert!(st.consumer.front().obs.is_none());
            assert!(!hub_r.has_prewarm_template());
            entered_tx.send(()).unwrap();
            // Pause while producer runs `try_prewarm_observations` (hits WouldBlock) + publishes!
            resume_rx.recv().unwrap();
            let snap = hub_r.refresh(&mut st).expect("published snapshot visible after resume");
            assert_eq!(snap.meta.seq, 2);
            assert!(st.consumer.front().obs.is_some());
        });

        entered_rx.recv().unwrap();
        // Contended startup prewarm warms back & middle without blocking and records template.
        publisher.try_prewarm_observations(|| obs(0.0));
        assert!(hub.has_prewarm_template());
        // Publish twice while reader still holds `hub.state`.
        for seq in 1..=2u64 {
            let mut o = publisher.take_back_observation().expect("back slot prewarmed");
            o.timestamp = seq as f64;
            assert!(publisher.publish(seq, now, now + Duration::from_millis(40), o, inputs(&p, &s, &i)).is_none());
        }
        // Resume reader: `refresh()` must warm `front` BEFORE rotating it into `middle`.
        resume_tx.send(()).unwrap();
        reader.join().unwrap();

        // Subsequent publications rotate that consumer slot back into the producer: NEVER empty!
        for seq in 3..=6u64 {
            let mut o = publisher
                .take_back_observation()
                .expect("rotated consumer front slot must be prewarmed off-thread");
            o.timestamp = seq as f64;
            assert!(publisher.publish(seq, now, now + Duration::from_millis(40), o, inputs(&p, &s, &i)).is_none());
        }
        assert_eq!(hub.tick_frees(), 0);
    }

    #[test]
    fn read_services_enforces_coherence_freshness_motion_and_live_zero_convention() {
        let (mut publisher, hub) = state_channel(7);
        assert_eq!(hub.read_services(ReadRequirements::identity()).unwrap_err(), ServiceReadError::NoSnapshot);

        let now = Instant::now();
        let p = [0.0f32; 7];
        let v_still = [0.001f32; 7];
        let v_moving = [0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0];
        let s = [1u64; 7];
        let fresh_i = [Some(now); 7];
        let boot = BootId(42);
        let tool_hash = super::super::ops::tool_digest_hash("sha256:tool-a");

        let mut owner_meta = PublishedOwnerMeta {
            arm: ArmId::LOCAL,
            boot,
            control_release_seq: 10,
            motion_generation: 3,
            calibration_revision: 5,
            collision_revision: 2,
            poses_revision: 4,
            sequences_revision: 6,
            grasp_setup_revision: 7,
            gripper_calib_revision: 8,
            power_budget_revision: 9,
            tool_commissioning_revision: 11,
            tool_digest_hash: tool_hash,
            feedback_valid: true,
            ..PublishedOwnerMeta::default()
        };
        owner_meta.authority.config_revision = 12;
        owner_meta.authority.epoch = 2;
        owner_meta.authority.lease_generation = 1;

        let mut owner_state = OwnerStateSnapshot::default();
        owner_state.boot = boot;
        owner_state.has_zero_reference = true;
        owner_state.zero_convention.status = "legacy_j6";
        owner_state.zero_convention.j6_rad = Some(1.57);

        let mut rt_meta = RtMotionMeta::default();
        rt_meta.boot_id = boot;
        rt_meta.motion_config_revision = 13;
        rt_meta.stop_generation = 4;

        let rt = RtInputs {
            positions: &p,
            velocities: &v_still,
            commanded_positions: &p,
            commanded_velocities: &p,
            feedback_instants: &fresh_i,
            feedback_seqs: &s,
            feedback_queue_bounds: &[],
            owner: owner_meta,
            meta: rt_meta,
        };
        publisher.publish_with_owner(20, now, now + Duration::from_secs(5), obs(1.0), owner_state, rt);

        // Missing identity fails closed.
        assert_eq!(
            hub.read_services(ReadRequirements::identity()).unwrap_err(),
            ServiceReadError::IdentityUnavailable
        );

        let base_identity = ServiceIdentity {
            arm: ArmId::LOCAL,
            boot,
            profile_id: Some("rebot-dm".into()),
            follower: Some("mock".into()),
            dof: 7,
            arm_dof: 6,
            control_frequency_hz: 50,
            joint_names: (1..=7).map(|i| format!("j{i}")).collect(),
            tool_id: "stock".into(),
            tool_config_digest: "sha256:tool-a".into(),
            tool_digest_hash: tool_hash,
            tool_is_stock: true,
            joint_zero_offsets: vec![0.0; 7],
            calibration_revision: 5,
        };

        // Mismatched boot / calibration_revision / tool_digest_hash all fail closed.
        let mut wrong_boot = base_identity.clone();
        wrong_boot.boot = BootId(99);
        hub.set_service_identity(Arc::new(wrong_boot));
        assert_eq!(
            hub.read_services(ReadRequirements::identity()).unwrap_err(),
            ServiceReadError::IdentityMismatch
        );

        let mut wrong_cal = base_identity.clone();
        wrong_cal.calibration_revision = 4;
        hub.set_service_identity(Arc::new(wrong_cal));
        assert_eq!(
            hub.read_services(ReadRequirements::identity()).unwrap_err(),
            ServiceReadError::IdentityMismatch
        );

        let mut wrong_tool = base_identity.clone();
        wrong_tool.tool_digest_hash = super::super::ops::tool_digest_hash("sha256:tool-b");
        hub.set_service_identity(Arc::new(wrong_tool));
        assert_eq!(
            hub.read_services(ReadRequirements::identity()).unwrap_err(),
            ServiceReadError::IdentityMismatch
        );

        // Matching identity succeeds and pulls live zero_convention + revisions from the RT slot.
        hub.set_service_identity(Arc::new(base_identity.clone()));
        let svc = hub
            .read_services(ReadRequirements::fresh_feedback(Duration::from_secs(1)))
            .expect("coherent read_services");
        assert_eq!(svc.boot, boot);
        assert_eq!(svc.publish_seq, 1);
        assert_eq!(svc.release_seq, 10);
        assert_eq!(svc.motion_generation, 3);
        assert_eq!(svc.applied_stop_generation, 4);
        assert_eq!(svc.epoch_id, 2);
        assert_eq!(svc.lease_generation, 1);
        assert!(svc.motion_idle);
        assert_eq!(svc.zero_convention.status, "legacy_j6");
        assert!(svc.has_zero_reference);
        assert_eq!(svc.revisions.workspace, 12);
        assert_eq!(svc.revisions.motion_config, 13);
        assert_eq!(svc.revisions.calibration, 5);
        assert_eq!(svc.revisions.collision, 2);
        assert_eq!(svc.revisions.tool_commissioning, 11);
        assert_eq!(svc.revisions.tool_digest_hash, tool_hash);

        // Generation / release sequence bounds fail closed.
        let mut req_gen = ReadRequirements::default();
        req_gen.min_generation = Some(21);
        assert_eq!(
            hub.read_services(req_gen).unwrap_err(),
            ServiceReadError::GenerationNotReached { required: 21, actual: 20 }
        );
        let mut req_rel = ReadRequirements::default();
        req_rel.min_release_seq = Some(11);
        assert_eq!(
            hub.read_services(req_rel).unwrap_err(),
            ServiceReadError::ReleaseSequenceNotReached { required: 11, actual: 10 }
        );

        // Missing prep_env when required fails closed.
        let mut req_prep = ReadRequirements::default();
        req_prep.require_prep_env = true;
        assert_eq!(hub.read_services(req_prep).unwrap_err(), ServiceReadError::PrepEnvUnavailable);

        // Mismatched prep_env fails closed; matching prep_env succeeds.
        let inner_cfg = crate::supervisor::SupervisorInner::new(crate::ArmConfig::rebot_dm());
        let mut env = super::super::requests::actions::ActionPrepEnv::from_inner(&inner_cfg);
        hub.update_prep_env(Arc::new(env.clone()));
        assert_eq!(hub.read_services(req_prep).unwrap_err(), ServiceReadError::PrepEnvMismatch);
        env.workspace_revision = 12;
        env.calibration_revision = 5;
        env.poses_revision = 4;
        env.sequences_revision = 6;
        env.grasp_setup_revision = 7;
        env.gripper_calib_revision = 8;
        env.power_budget_revision = 9;
        env.tool_digest_hash = tool_hash;
        hub.update_prep_env(Arc::new(env));
        assert!(hub.read_services(req_prep).unwrap().prep_env.is_some());

        // Missing joint timestamp fails closed under max_feedback_age.
        let mut missing_j2 = fresh_i;
        missing_j2[2] = None;
        let rt_missing = RtInputs {
            positions: &p,
            velocities: &v_still,
            commanded_positions: &p,
            commanded_velocities: &p,
            feedback_instants: &missing_j2,
            feedback_seqs: &s,
            feedback_queue_bounds: &[],
            owner: owner_meta,
            meta: rt_meta,
        };
        publisher.publish_with_owner(21, now, now + Duration::from_secs(5), obs(2.0), owner_state, rt_missing);
        assert_eq!(
            hub.read_services(ReadRequirements::fresh_feedback(Duration::from_secs(1))).unwrap_err(),
            ServiceReadError::FeedbackStale { joint: 2 }
        );

        // Gravity comp blocks motion_idle; moving joint blocks max_arm_joint_speed.
        let mut gc_owner = owner_meta;
        gc_owner.gravity_comp_active = true;
        let rt_gc = RtInputs {
            positions: &p,
            velocities: &v_still,
            commanded_positions: &p,
            commanded_velocities: &p,
            feedback_instants: &fresh_i,
            feedback_seqs: &s,
            feedback_queue_bounds: &[],
            owner: gc_owner,
            meta: rt_meta,
        };
        publisher.publish_with_owner(22, now, now + Duration::from_secs(5), obs(3.0), owner_state, rt_gc);
        assert_eq!(
            hub.read_services(ReadRequirements::measure(Duration::from_secs(1), 0.02)).unwrap_err(),
            ServiceReadError::MotionActive
        );

        let rt_moving = RtInputs {
            positions: &p,
            velocities: &v_moving,
            commanded_positions: &p,
            commanded_velocities: &p,
            feedback_instants: &fresh_i,
            feedback_seqs: &s,
            feedback_queue_bounds: &[],
            owner: owner_meta,
            meta: rt_meta,
        };
        publisher.publish_with_owner(23, now, now + Duration::from_secs(5), obs(4.0), owner_state, rt_moving);
        assert_eq!(
            hub.read_services(ReadRequirements::measure(Duration::from_secs(1), 0.02)).unwrap_err(),
            ServiceReadError::JointMoving { joint: 1 }
        );
        // Non-finite speed limit fails closed; handover_settling blocks motion_idle.
        let mut req_nan = ReadRequirements::default();
        req_nan.max_arm_joint_speed = Some(f32::NAN);
        assert_eq!(
            hub.read_services(req_nan).unwrap_err(),
            ServiceReadError::JointMoving { joint: 0 }
        );
        let mut hs_owner = owner_meta;
        hs_owner.handover_settling = true;
        let rt_hs = RtInputs {
            positions: &p,
            velocities: &v_still,
            commanded_positions: &p,
            commanded_velocities: &p,
            feedback_instants: &fresh_i,
            feedback_seqs: &s,
            feedback_queue_bounds: &[],
            owner: hs_owner,
            meta: rt_meta,
        };
        publisher.publish_with_owner(24, now, now + Duration::from_secs(5), obs(4.5), owner_state, rt_hs);
        assert_eq!(
            hub.read_services(ReadRequirements::measure(Duration::from_secs(1), 0.02)).unwrap_err(),
            ServiceReadError::MotionActive
        );

        // Expired valid_until fails closed when require_within_valid_until is true, allowed by identity().
        let past = now - Duration::from_secs(1);
        let rt_expired = RtInputs {
            positions: &p,
            velocities: &v_still,
            commanded_positions: &p,
            commanded_velocities: &p,
            feedback_instants: &fresh_i,
            feedback_seqs: &s,
            feedback_queue_bounds: &[],
            owner: owner_meta,
            meta: rt_meta,
        };
        publisher.publish_with_owner(25, past, past, obs(5.0), owner_state, rt_expired);
        assert_eq!(
            hub.read_services(ReadRequirements::default()).unwrap_err(),
            ServiceReadError::SnapshotExpired
        );
        assert_eq!(hub.read_services(ReadRequirements::identity()).unwrap().publish_seq, 6);
    }

    #[tokio::test]
    async fn supervisor_read_services_works_on_legacy_and_executor_and_tracks_motion_generation() {
        let sup = Arc::new(crate::supervisor::ArmSupervisor::new(crate::ArmConfig::rebot_dm()));

        // Legacy without control ticker falls back cleanly under one read guard.
        let legacy_svc = sup
            .read_services(ReadRequirements::identity())
            .await
            .expect("legacy no-ticker read_services");
        assert_eq!(legacy_svc.identity.dof, 7);
        assert_eq!(legacy_svc.identity.arm_dof, 6);
        assert!(legacy_svc.motion_idle);

        // Start RT executor and wait for its first post-handoff published sample.
        let handle = sup
            .start_executor(super::super::executor::ExecutorConfig::for_tests(
                Duration::from_millis(20),
            ))
            .await
            .expect("start executor");
        let mut exec_svc = None;
        for _ in 0..50 {
            if let Ok(s) = sup.read_services(ReadRequirements::default()).await {
                exec_svc = Some(s);
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        let s1 = exec_svc.expect("executor must publish coherent ServiceSnapshot past handoff");
        assert!(s1.publish_seq >= 1);
        assert!(s1.release_seq >= 1);
        assert_eq!(s1.identity.boot, s1.boot);
        let gen_before = s1.motion_generation;

        // Triggering E-stop bumps epoch and motion_generation on the next release.
        let _ = sup.trigger_e_stop().await;
        let mut s2 = None;
        for _ in 0..50 {
            if let Ok(s) = sup.read_services(ReadRequirements::default()).await {
                if s.motion_generation > gen_before {
                    s2 = Some(s);
                    break;
                }
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        let s2 = s2.expect("motion_generation must advance after E-stop");
        assert!(s2.motion_generation > gen_before);
        assert!(s2.epoch_id > s1.epoch_id);
        assert!(s2.release_seq > s1.release_seq);

        handle.stop_and_join();
    }
}
