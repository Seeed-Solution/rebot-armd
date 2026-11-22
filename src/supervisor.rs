use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::{broadcast, RwLock};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::driver::{ArmDriver, JointTarget, EnableState, DriverCapabilities};
use crate::model::{ArmConfig, ArmObservation, MotionLimit, MotionLimitReason, ObservationSchema, RestingPosePolicy};
use crate::recording::{EpisodeRecorder, RecordFrame, RecordingError};
use crate::safety::{FaultCode, FaultReason, SafetyError, SafetyManager, SafetyState, StopPolicy};
use crate::trajectory::{MultiJointQuinticTrajectory, TrajectoryError, TrajectoryPlanner};
use crate::macro_action::{MacroRun, SubDone, SubState};
use crate::motion::{
    ActionStatus, ActiveTrajectory, GripperCommand, GripperMotion, GripperRequest, JointPath, MotionError, PathTrajectory,
    PlanInput, Reference, Settle, StepCtx, StepOut, StepStatus,
};

#[derive(Debug, Error)]
pub enum ArmError {
    #[error("Safety error: {0}")]
    Safety(#[from] SafetyError),

    #[error("Trajectory error: {0}")]
    Trajectory(#[from] TrajectoryError),

    #[error("Recording error: {0}")]
    Recording(#[from] RecordingError),

    #[error("Unsupported operation: {0}")]
    Unsupported(String),

    #[error("Motion error: {0}")]
    Motion(#[from] MotionError),

    #[error("Internal error: {0}")]
    Internal(String),

    /// Request refused with a specific code (gravity compensation mode changes).
    #[error("{code}: {message}")]
    Rejected { status: u16, code: String, message: String },

    /// [`Rejected`](Self::Rejected) with extra reply fields (`owner`, `sequences`, `step`, `reason`).
    #[error("{code}: {message}")]
    Detailed { status: u16, code: String, message: String, details: Box<serde_json::Value> },

    /// Workspace rejection preserving structured violation detail (e.g. replay).
    #[error("WORKSPACE_VIOLATION: {message}")]
    WorkspaceRejected {
        message: String,
        detail: Box<crate::kinematics::WorkspaceViolationDetail>,
    },

    /// Config write expected_revision does not match the current revision.
    #[error("Policy revision mismatch: expected {expected}, current {current}")]
    PolicyRevisionMismatch { expected: u64, current: u64 },
}

impl ArmError {
    pub fn detailed(status: u16, code: &str, message: impl Into<String>, details: serde_json::Value) -> Self {
        Self::Detailed { status, code: code.into(), message: message.into(), details: Box::new(details) }
    }

    pub fn workspace_detail(&self) -> Option<crate::kinematics::WorkspaceViolationDetail> {
        match self {
            Self::Safety(e) => e.workspace_detail(),
            Self::Motion(MotionError::WorkspaceViolation { detail, .. }) => Some((**detail).clone()),
            Self::WorkspaceRejected { detail, .. } => Some((**detail).clone()),
            _ => None,
        }
    }
}

impl From<crate::recording::StoreError> for ArmError {
    fn from(e: crate::recording::StoreError) -> Self {
        let (status, code) = e.status_code();
        ArmError::Rejected { status, code: code.into(), message: e.to_string() }
    }
}

#[derive(Debug, thiserror::Error, PartialEq, Eq, Clone)]
pub enum PreemptError {
    #[error("Preemption cooldown active: {0}ms remaining")]
    Cooldown(u64),
}

/// J7 stall latch: |τ7| at or above this (N·m) with the jaw still, for 6 ticks.
/// Tuned on hardware as 0.80 when τ7 was decoded with TMAX 18; the J7 motor's TMAX
/// is 10, so the same physical torque is 0.80 × 10 / 18.
pub const GRIPPER_STALL_LATCH_NM: f32 = 0.444;
/// J7 stall release: |τ7| below this (N·m) for 6 ticks (formerly 0.40 at TMAX 18;
/// 0.40 × 10 / 18 = 0.2222). Raw torque 2093 decodes to exactly that value and was read
/// as 0.39999962 < 0.40 before, so the bound sits just above it.
pub const GRIPPER_STALL_RELEASE_NM: f32 = 0.2223;
/// Gripper torque cap: consecutive ticks with |τ| above `gripper_policy.max_close_torque_nm`
/// (fresh feedback, jaw driven closed) that latch the stall regardless of the jaw speed.
pub const GRIPPER_FORCE_CAP_TICKS: u32 = 2;
/// A torque-cap latch under a servo stream or live policy releases only when the stream's
/// gripper target is at least this much more open than the latch hold (rad). Low torque does
/// not release it there: on a rigid object the hold at the measured jaw reads ~0 N·m, and a
/// release would let the stream squeeze again (a cycle every ~19 ticks in the sim rig).
pub const GRIPPER_FORCE_RELEASE_MARGIN_RAD: f32 = 0.05;
/// Stream contact regulator: the contact ends when the jaw closes this far past the contact
/// point with |τ| below half of `contact_torque_nm` (the object gave way or slipped).
pub const GRIPPER_STREAM_CONTACT_LOST_RAD: f32 = 0.1;

/// Contact regulator state of a servo stream / live policy on the gripper
/// (`gripper_policy.stream_contact_speed_rad_s`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GripperStreamContact {
    /// Measured jaw at the contact (rad).
    pub at_rad: f32,
    /// Most-closed J7 command allowed (rad), applied by `gripper_lead_cap`.
    pub limit_rad: f32,
}

/// Feedback gap that latches BusLoss while torqued. The Damiao motors run with
/// CAN TIMEOUT=0 and keep holding their last POS_VEL target through a gap, so a
/// short interruption is not a hazard by itself. 500 ms tolerates the ~260 ms
/// CAN gaps observed on the reBot rig (2026-11-06, coinciding with 24 V sags);
/// a real power loss drops the arm regardless of this bound.
pub const BUS_LOSS_TIMEOUT: Duration = Duration::from_millis(500);
/// A control tick arriving later than this after the previous one (or a non-finite /
/// non-positive step) latches Fault `CONTROL_DEADLINE` while torqued and operational.
/// Same bound as the hardware feedback freshness limit of the drivers.
pub const CONTROL_DEADLINE: Duration = Duration::from_millis(250);
/// After a takeover (`acquire_lease` with `force`) another connection must wait this long
/// before it can take the lease over again.
pub const PREEMPT_COOLDOWN: Duration = Duration::from_secs(3);
/// A ThermalTrip can be cleared only once every joint reports a temperature at or below
/// `max_temperature` minus this margin (°C).
pub const THERMAL_CLEAR_MARGIN_C: f32 = 5.0;
/// Lower bound for the automatic park started by an expired lease (trajectory + settle).
/// The actual budget is `max(WATCHDOG_PARK_TIMEOUT, deceleration + planned duration +
/// UNLOAD_SETTLE_HOLD + WATCHDOG_PARK_MARGIN)`, counted in control-loop time.
pub const WATCHDOG_PARK_TIMEOUT: Duration = Duration::from_secs(10);
/// Tracking lag and slack added to the watchdog park budget.
pub const WATCHDOG_PARK_MARGIN: Duration = Duration::from_secs(3);
/// Extra time granted once when the watchdog park budget runs out with the arm at the
/// resting pose (position, support torque, enable and feedback all pass) and only the
/// stationary check pending. A blocked or unsupported arm gets no grace.
pub const WATCHDOG_PARK_SETTLE_GRACE: Duration = Duration::from_secs(5);
/// Time the arm must stay at the resting pose before an unload cuts torque.
pub const UNLOAD_SETTLE_HOLD: Duration = Duration::from_millis(800);
/// Park with an object in the gripper: at the resting pose J7 opens this far past the contact
/// point (rad, about a third of the B601-DM stroke) to drop it, then closes.
pub const PARK_RELEASE_OPEN_RAD: f32 = 2.0;
/// J7 speed of the park release (rad/s), as macro gripper moves.
pub const PARK_RELEASE_SPEED: f64 = 1.0;

/// Gripper release at the end of a park that carried an object.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParkRelease { Opening, Closing }
/// Stationary evidence over the settle hold window, per joint: max - min measured
/// position (rad). 4 mrad in 0.8 s bounds the average speed at 0.005 rad/s.
pub const UNLOAD_STILL_POSITION_SPAN_RAD: f32 = 0.004;
/// Per-sample |velocity| bound during the settle hold, as a multiple of the joint's
/// resting-pose velocity limit (J1-J3: 0.02 rad/s = 4 LSB at VMAX 10). The window mean
/// must stay within the limit itself.
pub const UNLOAD_STILL_VELOCITY_PEAK_FACTOR: f32 = 4.0;
/// The mean velocity over this most recent part of the settle window must also stay within
/// the resting-pose limit, so motion starting late in the window blocks the release
/// before it accumulates the position span.
pub const UNLOAD_STILL_RECENT: Duration = Duration::from_millis(300);
/// A settle pending longer than the hold plus this logs its blocker, at most once per second.
const UNLOAD_SETTLE_REPORT_AFTER_S: f64 = 1.0;

/// Settle phase of a NormalUnload after its trajectory finished.
#[derive(Debug, Clone, Default)]
pub struct UnloadSettle {
    /// Time in the settle phase (s, control-loop time).
    pub pending_s: f64,
    /// Time from the first to the latest sample of the current run of samples that met
    /// the static resting-pose conditions (s).
    pub held_s: f64,
    /// `pending_s` of the first sample of that run.
    pub held_since_s: Option<f64>,
    /// (time, positions, velocities) over the last [`UNLOAD_SETTLE_HOLD`] while held.
    pub samples: std::collections::VecDeque<(f64, Vec<f32>, Vec<f32>)>,
    /// `pending_s` of the last blocker report.
    pub last_report_s: Option<f64>,
}

/// Servo streaming: a lead-capped joint whose measured position moves less than this
/// (rad) over `following_error.persistence_ms` is blocked.
pub const SERVO_BLOCKED_PROGRESS_RAD: f32 = 0.01;

/// Most FK evaluations the keep-out sweep check may add for one servo or braking step;
/// a step that needs more is held like a step into a box.
pub const MAX_KEEPOUT_EVALS_PER_TICK: usize = 64;

/// Most FK evaluations the keep-out sweep check may add for one joint-space trajectory;
/// a path that needs more is WORKSPACE_VIOLATION.
pub const MAX_KEEPOUT_EVALS_PER_PLAN: usize = 4000;

/// `observation.motion_limit` is cleared this long (s) after the limit last applied.
pub const MOTION_LIMIT_CLEAR_S: f64 = 0.3;

/// Park started by the lease watchdog: elapsed control-loop time against its budget.
#[derive(Debug, Clone, PartialEq)]
pub struct WatchdogPark {
    pub elapsed_s: f64,
    pub budget_s: f64,
    /// True once [`WATCHDOG_PARK_SETTLE_GRACE`] was added to the budget.
    pub grace_granted: bool,
}

/// Active WebSocket client lease record
#[derive(Debug, Clone, PartialEq)]
pub struct ActiveLease {
    pub client_id: String,
    pub generation: u64,
    /// The `client_id` label the holder sent with `acquire_lease` (e.g. "rebot-mcp"),
    /// published in the observation as `lease_holder`; empty when none was given.
    pub label: String,
    pub acquired_at: Instant,
    pub last_valid_motion_instant: Option<Instant>,
}

/// An object currently held in the gripper jaws.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HeldObject {
    pub label: String,
    pub spill_safe: bool,
}

/// Action completion captured during trajectory stepping and published only after
/// that tick's output passes tracking validation and is written to the driver.
struct DeferredActionDone {
    action_id: Option<u64>,
    macro_sub: bool,
    settled: Option<bool>,
    holding: Option<bool>,
    gripper_gap_rad: Option<f32>,
    clear_releasing: bool,
}

/// Internal mutable state of the robotic arm supervisor
pub struct SupervisorInner {
    pub config: ArmConfig,
    pub safety: SafetyManager,
    pub held: Option<HeldObject>,
    /// A gripper Open is running: `held` is cleared when it completes, not when it starts, so an
    /// interrupted release still counts as holding.
    pub releasing: bool,
    pub measured_positions: Vec<f32>,
    pub measured_velocities: Vec<f32>,
    pub commanded_positions: Vec<f32>,
    pub commanded_velocities: Vec<f32>,
    pub last_commanded_velocities: Vec<f32>,
    pub commanded_accelerations: Vec<f32>,
    pub following_error_since: Option<Instant>,
    pub disable_pending: bool,
    pub last_disable_attempt: Option<Instant>,
    pub output_suspended: bool,
    pub feedback_numeric_valid: bool,
    pub torques: Vec<f32>,
    pub loads: Vec<f32>,
    pub temperatures: Vec<f32>,
    pub target_positions: Vec<f32>,
    pub servo_target: Option<Vec<f32>>,
    /// True while a servo/policy stream owns output, including its braking tail after
    /// `servo_target` is cleared by stop, timeout, or dirty-jump discard.
    pub servo_stream_active: bool,
    pub servo_settled_ticks: Vec<u32>,
    /// Per joint, while the servo lead cap holds it: when the current run started and
    /// the measured position then (blocked detection, [`SERVO_BLOCKED_PROGRESS_RAD`]).
    pub servo_lead_stall: Vec<Option<(Instant, f32)>>,
    /// Published as `observation.motor_errors`; a change is logged once per joint.
    pub motor_errors: Vec<Option<crate::model::MotorError>>,
    /// Published as `observation.motion_limit`.
    pub motion_limit: Option<MotionLimit>,
    /// Structured details of the active `MotionLimitReason::Workspace` hold/rejection.
    pub workspace_violation: Option<crate::kinematics::WorkspaceViolationDetail>,
    /// Observation-clock time (s) `motion_limit` last applied.
    pub motion_limit_seen_s: f64,
    /// Holding after a blocked servo stream (keeps `motion_limit` = blocked).
    pub servo_blocked_hold: bool,
    /// Persistent monotonic escape baseline across a servo stream & its braking phase, tagged with `workspace_revision`.
    pub servo_ws_min_depths: Option<(u64, crate::kinematics::SlotBuf)>,
    pub is_torqued: bool,
    pub pending_rearm: bool,
    pub active_trajectory: Option<ActiveTrajectory>,
    /// Most recent action record (published as `observation.action`).
    pub last_action: Option<ActionStatus>,
    /// Record replaced by a newer action and the timestamp until which it is published.
    pub previous_action: Option<(ActionStatus, f64)>,
    pub next_action_id: u64,
    /// Lease generation when the running action started (lease loss detection).
    pub action_lease_generation: Option<u64>,
    /// Settle phase of a finished joint_move / home quintic: (action id, settle state).
    pub quintic_settle: Option<(u64, Settle)>,
    /// Running macro action (grasp, place); its sub-motions carry the action id.
    pub macro_run: Option<crate::macro_action::MacroRun>,
    /// Active streaming policy run (`src/policy_runtime/runner.rs`).
    #[cfg(feature = "policy-runtime")]
    pub policy_run: Option<crate::policy_runtime::PolicyRun>,
    /// True while a policy pack is being loaded/prepared outside the supervisor lock.
    #[cfg(feature = "policy-runtime")]
    pub policy_loading: bool,
    /// Most recent policy observation telemetry (retained after policy run ends).
    #[cfg(feature = "policy-runtime")]
    pub last_policy_obs: Option<crate::model::PolicyObservation>,
    /// Device qualification ledger (`src/policy_runtime/qualification.rs`); None = no records
    /// are written and production releases are refused.
    #[cfg(feature = "policy-runtime")]
    pub policy_ledger_path: Option<std::path::PathBuf>,
    /// Test hook run once by the WS action handler between planning and install.
    #[doc(hidden)]
    pub before_install_hook: Option<Box<dyn FnOnce(&mut SupervisorInner) + Send + Sync>>,
    pub last_heartbeat: Instant,
    pub last_motion_frame: Instant,
    pub motion_frame_timeout: Duration,
    pub lease_timeout: Duration,
    pub lease_enabled: bool,
    pub active_lease: Option<ActiveLease>,
    pub lease_generation_counter: u64,
    pub gripper_stalled: bool,
    pub gripper_high_torque_ticks: u32,
    pub gripper_low_torque_ticks: u32,
    pub gripper_stall_sign: f32,
    pub gripper_prev_hw_overload: bool,
    /// The stall latch was set (or refreshed) by the gripper torque cap.
    pub gripper_force_capped: bool,
    /// Consecutive ticks above the gripper torque cap.
    pub gripper_force_ticks: u32,
    /// Gripper torque cap trips since start (`observation.gripper.force_limit_trips`).
    pub gripper_force_trips: u64,
    /// J7 freeze during a whole-arm reference (joint quintic, path, replay): (hold, closing sign).
    /// Set once J7 stalls or an object is held; the reference may not drive J7 further in the
    /// stall direction until it ends or moves J7 the other way.
    pub gripper_motion_hold: Option<(f32, f32)>,
    /// Gripper target of the running servo stream or live policy (last accepted frame); None
    /// when no stream drives the gripper (no frame for `motion_frame_timeout`, an action or a
    /// whole-arm reference runs, torque changes).
    pub gripper_stream_target: Option<f32>,
    /// Contact regulator of the stream (`gripper_policy.stream_contact_speed_rad_s`).
    pub gripper_stream_contact: Option<GripperStreamContact>,
    /// A park carried an object (J7 frozen short of closed): release it at the resting pose.
    pub park_gripper_contact: bool,
    pub park_release: Option<ParkRelease>,
    pub pending_normal_unload: bool,
    pub unload_settle: Option<UnloadSettle>,
    /// Set while a park started by the lease watchdog is running.
    pub watchdog_park: Option<WatchdogPark>,
    /// Set when the last park was refused because its path enters a keep-out box.
    pub park_rejection: Option<SafetyError>,
    /// Bumped on every change of the workspace rule inputs (keep-out boxes, camera mount);
    /// a plan made outside the lock must match it at install (else PLAN_STALE).
    pub workspace_revision: u64,
    /// Camera calibration in effect (`calibration.json`, `crate::calibration`); None = the
    /// profile's camera mount.
    pub calibration: Option<crate::calibration::CalibrationFile>,
    /// The profile's `workspace.floor_z_m` (None without a workspace). `config.workspace.floor_z_m`
    /// holds the effective floor ([`crate::calibration::effective_floor`]), recomputed by
    /// [`refresh_effective_floor`](Self::refresh_effective_floor) whenever its inputs change.
    pub profile_floor_z_m: Option<f32>,
    /// Source of the effective floor: `profile`, `calibration_camera` or `calibration_hover`.
    pub floor_source: &'static str,
    /// Calibration hover jobs (`crate::calib_hover`), newest last, at most
    /// [`crate::calib_hover::KEEP_JOBS`].
    pub hover_jobs: Vec<crate::calib_hover::HoverJob>,
    /// A calibration apply / restore transaction is running: motion commands are refused
    /// (MOTION_BUSY) and other configuration writes too (CONFIG_BUSY).
    pub calibration_busy: bool,
    /// A motor zero runs (`crate::motor_zero`, with `calibration_busy`): torque on and
    /// clear_fault are refused too.
    pub motor_zero_busy: bool,
    /// `calibration.json` was rejected at startup (message; cleared by a successful apply / restore).
    pub calibration_load_error: Option<String>,
    /// Joint zero offsets in effect (motor - joint, rad, one per joint; calibration.json
    /// `joint_zero`), applied by the driver boundary ([`crate::joint_zero::ZeroOffsetDriver`]).
    pub joint_zero_offsets: Vec<f32>,
    /// Zero check against the profile's `zero_reference` (`zero_convention`).
    pub zero_convention: crate::joint_zero::ZeroConvention,
    /// Run the zero check on the next tick with fresh feedback and torque off (startup, after
    /// torque went off, after a zero).
    pub zero_check_pending: bool,
    zero_check_prev_torqued: bool,
    /// The profile's camera mount and grasp setup before any file was loaded (restore of a
    /// backup without calibration.json / grasp_setup.json).
    pub profile_camera_mount: crate::model::CameraMount,
    pub profile_grasp: Option<crate::grasp::GraspSetup>,
    /// Duration of the most recently planned NormalUnload trajectory (s).
    pub unload_planned_duration: Option<f64>,
    pub recorder: EpisodeRecorder,
    /// Incremented when the joint zero or the kinematic correction changes: a running sequence
    /// fails CALIBRATION_CHANGED before its next motion step.
    pub calibration_revision: u64,
    /// Stored sequences (`crate::sequences`).
    pub sequences: crate::sequences::SequenceStore,
    /// The running recording and its owner (`crate::teach`).
    pub recording: Option<crate::teach::ActiveRecording>,
    /// Length cap of a recording (s); [`crate::recording::store::MAX_EPISODE_S`] (tests lower it).
    pub max_episode_s: f64,
    /// Episode store (`--episodes-dir`, `src/recording/store.rs`).
    pub episodes: crate::recording::EpisodeStore,
    pub start_time: Instant,
    pub driver: Option<std::sync::Mutex<Box<dyn ArmDriver>>>,
    pub gravity_compensated: bool,
    pub gravity_torques: Vec<f32>,
    pub gravity_scale: f32,
    pub gravity_torques_ramped: Vec<f32>,
    pub last_can_feedback_instant: Option<Instant>,
    pub bus_loss_instant: Option<Instant>,
    pub e_stop_latched: Arc<AtomicBool>,
    pub requested_estop_count: Arc<AtomicU64>,
    pub applied_estop_count: u64,
    pub last_preemption: Option<Instant>,
    /// Connection and generation most recently evicted by a forced acquire.
    pub last_evicted: Option<(String, u64)>,
    pub handover_settling_since: Option<Instant>,
    /// Running gravity-compensation session (`src/gravity_comp.rs`).
    pub gravity_comp: Option<crate::gravity_comp::GravityCompRun>,
    pub gravity_comp_last_exit: Option<crate::model::GravityCompExit>,
    /// Keep-out source and file (`src/keepout.rs`).
    pub keepout: crate::keepout::Keepout,
    /// Named poses (`src/poses.rs`).
    pub poses: crate::poses::PoseStore,
    /// Measured minus model gravity torque, J1..J6 (`observation.residual_torques`).
    pub residual_torques: Option<[f32; 6]>,
    /// Collision guard state (`src/collision.rs`).
    pub collision: crate::collision::CollisionGuard,
    /// Torque-saturation guard state (`src/torque_guard.rs`).
    pub torque_guard: crate::torque_guard::TorqueGuard,
    /// Joint tracking output history (`src/tracking.rs`).
    pub tracking: crate::tracking::TrackingRuntime,
    /// Supply-current estimate (`src/power.rs`).
    pub power: crate::power::PowerMonitor,
    /// `--follower` model name armd was started with (`safety/describe` `device.follower`).
    pub follower: Option<String>,
    /// Base gripper policy from profile (unshifted).
    pub base_gripper_policy: Option<crate::model::GripperPolicy>,
    /// Persisted gripper calibration offset (rad).
    pub gripper_calib_offset: Option<f32>,
    /// Persisted nominal jaw opening curve.
    pub gripper_calib_jaw_opening: Option<crate::model::JawOpening>,
    /// File path for gripper calibration (`gripper_calib.json`).
    pub gripper_calib_path: Option<PathBuf>,
    /// Set to true once an active zero check action has completed.
    pub gripper_zero_checked: bool,
    /// Set to true if startup loading encountered an invalid calibration file.
    pub gripper_calib_invalid: bool,
    /// The gripper calibration was taught with an older gripper zero (set by a zero of the
    /// gripper joint, persisted in `gripper_calib.json`; a new calibration clears it).
    pub gripper_calib_stale: Option<crate::calibration::CalibrationStale>,
    /// Last measured zero check offset (rad).
    pub last_zero_check_offset: Option<f32>,
    /// Mock follower stall offset (rad); None = stall at closed_rad.
    pub mock_gripper_stall_offset: Option<f32>,
    /// The mock follower set torques[6] for a simulated stall (cleared when the stall ends).
    pub mock_gripper_stall_torque: bool,
    /// Simulation only (no driver): an object in the jaw at this J7 position, blocking every
    /// reference (not only gripper moves); `true` once touched. It falls out when the jaw
    /// opens 0.5 rad past it after contact.
    pub mock_gripper_object: Option<(f32, bool)>,
    /// Active driver limit configuration error; prevents clearing faults and enabling torque until resolved.
    pub driver_config_error: Option<String>,
    /// Running zero check parameters (id, store_offset, threshold_rad, base_closed_rad).
    pub zero_check_params: Option<ZeroCheckParams>,
    /// Monotonic revision for gripper calibration (starts at 1).
    pub gripper_calib_revision: u64,
    /// Monotonic revision for grasp setup (starts at 1).
    pub grasp_setup_revision: u64,
    /// When true (inside ArmSupervisor::tick), zero_check(store_offset=true) defers disk persist
    /// and memory commit until after the lock is released.
    pub defer_zero_check_persist: bool,
    /// Action id whose zero_check(store_offset=true) disk persist is currently in flight.
    pub zero_check_persisting: Option<u64>,
    /// Pending disk persist + commit payload produced by finish_action during ArmSupervisor::tick.
    pub pending_zero_check_persist: Option<PendingZeroCheckPersist>,
    /// Whether operator auth is enforced (exposed in GET /api/v1/safety/describe).
    pub operator_auth_enforced: bool,
}

/// Complete validated gripper calibration snapshot ready for infallible commit after disk rename.
#[derive(Debug, Clone)]
pub struct GripperCalibrationCandidate {
    config: ArmConfig,
    base_gripper_policy: Option<crate::model::GripperPolicy>,
    gripper_calib_offset: Option<f32>,
    gripper_calib_jaw_opening: Option<crate::model::JawOpening>,
    gripper_calib_invalid: bool,
    driver_config_error: Option<String>,
}

/// Parameters of a running gripper zero check action.
#[derive(Clone)]
pub struct ZeroCheckParams {
    pub action_id: u64,
    pub store_offset: bool,
    pub threshold_rad: f32,
    pub base_closed_rad: f32,
    pub operator_auth: Option<(crate::auth::OperatorAuth, Option<String>)>,
}

/// Deferred atomic persist-before-commit job for `gripper_zero_check(store_offset=true)`.
#[derive(Clone)]
pub struct PendingZeroCheckPersist {
    pub action_id: u64,
    pub offset_rad: f32,
    pub jaw_opening: Option<crate::model::JawOpening>,
    pub base_revision: u64,
    pub lease_owner: Option<(String, u64)>,
    pub operator_auth: Option<(crate::auth::OperatorAuth, Option<String>)>,
    pub write: crate::persist::PendingWrite,
}

impl SupervisorInner {
    pub fn new(mut config: ArmConfig) -> Self {
        config.validate().expect("Invalid ArmConfig: invariant violation");
        let dof = config.dof;
        let collision_policy = config.collision.unwrap_or_default();
        // power planning: build the dynamics chain now (the built-in model parses the embedded
        // URDF once), not on the first planned motion
        if config.power_budget.as_ref().is_some_and(|b| b.enabled && b.planning.enabled) { let _ = config.arm_model.dynamics_chain(); }
        let positions = config.home_positions.clone();
        let target_positions = positions.clone();
        let lease_timeout = Duration::from_millis(config.lease_timeout_ms);
        let motion_frame_timeout = Duration::from_millis(200);
        let now = Instant::now();

        let candidate_urdf_paths = [
            "assets/reBot-DevArm_gripper.urdf",
            "/usr/share/rebot-teleop/assets/reBot-DevArm_gripper.urdf",
            "/usr/share/rebot-teleop/reBot-DevArm_gripper.urdf",
        ];
        let mut gravity_compensated = false;
        if let Some(chain) = config.arm_model.chain() {
            // Profile URDF: read and (with urdf_sha256) verified by ArmConfig::load_arm_model.
            tracing::info!("Arm model from URDF '{}' ({} -> {}, {} joints, {:.3} kg moving)",
                chain.robot, chain.base_link, chain.tip_link, chain.dof(), chain.moving_mass());
            gravity_compensated = chain.moving_mass() > 0.0;
        }
        for path in candidate_urdf_paths.iter().filter(|_| config.arm_model.chain().is_none()) {
            if std::path::Path::new(path).exists() {
                match crate::gravity::verify_urdf_sha256(path) {
                    Ok(()) => {
                        tracing::info!("URDF model verified ({}); output still requires an explicit model profile and compatible driver.", path);
                        gravity_compensated = true;
                        break;
                    }
                    Err(e) => {
                        tracing::warn!("URDF verification failed for {} ({:?})", path, e);
                    }
                }
            }
        }
        if !gravity_compensated {
            tracing::warn!("Running in degraded mode without gravity compensation.");
        }

        let base_gripper_policy = config.gripper_policy.clone();
        let profile_camera_mount = config.camera_mount.clone();
        let profile_floor_z_m = config.workspace.as_ref().map(|w| w.floor_z_m);
        let profile_grasp = config.grasp.clone();
        let mock_gripper_stall_offset = std::env::var("ARMD_MOCK_GRIPPER_STALL_OFFSET").ok().and_then(|s| s.parse::<f32>().ok());

        Self {
            config,
            safety: SafetyManager::new(),
            held: None,
            releasing: false,
            measured_positions: positions.clone(),
            measured_velocities: vec![0.0; dof],
            commanded_positions: positions,
            commanded_velocities: vec![0.0; dof],
            last_commanded_velocities: vec![0.0; dof],
            commanded_accelerations: vec![0.0; dof],
            following_error_since: None,
            disable_pending: false,
            last_disable_attempt: None,
            output_suspended: false,
            feedback_numeric_valid: true,
            torques: vec![0.0; dof],
            loads: vec![0.0; dof],
            temperatures: vec![25.0; dof],
            target_positions,
            servo_target: None,
            servo_stream_active: false,
            servo_settled_ticks: vec![0; dof],
            servo_lead_stall: vec![None; dof],
            motor_errors: Vec::new(),
            motion_limit: None,
            workspace_violation: None,
            motion_limit_seen_s: 0.0,
            servo_blocked_hold: false,
            servo_ws_min_depths: None,
            is_torqued: false,
            pending_rearm: false,
            active_trajectory: None,
            last_action: None,
            previous_action: None,
            next_action_id: 0,
            action_lease_generation: None,
            macro_run: None,
            #[cfg(feature = "policy-runtime")]
            policy_run: None,
            #[cfg(feature = "policy-runtime")]
            policy_loading: false,
            #[cfg(feature = "policy-runtime")]
            last_policy_obs: None,
            #[cfg(feature = "policy-runtime")]
            policy_ledger_path: crate::policy_runtime::qualification::ledger_path_from_env(),
            quintic_settle: None,
            before_install_hook: None,
            last_heartbeat: now,
            last_motion_frame: now,
            motion_frame_timeout,
            lease_timeout,
            lease_enabled: true,
            active_lease: None,
            lease_generation_counter: 0,
            gripper_stalled: false,
            gripper_high_torque_ticks: 0,
            gripper_low_torque_ticks: 0,
            gripper_stall_sign: 0.0,
            gripper_prev_hw_overload: false,
            gripper_force_capped: false,
            gripper_force_ticks: 0,
            gripper_force_trips: 0,
            gripper_motion_hold: None,
            gripper_stream_target: None,
            gripper_stream_contact: None,
            park_gripper_contact: false,
            park_release: None,
            pending_normal_unload: false,
            unload_settle: None,
            watchdog_park: None,
            park_rejection: None,
            workspace_revision: 0,
            calibration: None,
            profile_floor_z_m,
            floor_source: "profile",
            hover_jobs: Vec::new(),
            calibration_busy: false,
            motor_zero_busy: false,
            calibration_load_error: None,
            joint_zero_offsets: vec![0.0; dof],
            zero_convention: Default::default(),
            zero_check_pending: true,
            zero_check_prev_torqued: false,
            profile_camera_mount,
            profile_grasp,
            unload_planned_duration: None,
            recorder: EpisodeRecorder::new(),
            calibration_revision: 0,
            sequences: Default::default(),
            recording: None,
            max_episode_s: crate::recording::store::MAX_EPISODE_S,
            episodes: crate::recording::EpisodeStore::new(Some(PathBuf::from(crate::recording::store::FALLBACK_EPISODES_DIR))),
            start_time: now,
            driver: None,
            gravity_compensated,
            gravity_torques: Vec::new(),
            gravity_scale: 0.5,
            gravity_torques_ramped: vec![0.0; dof],
            last_can_feedback_instant: None,
            bus_loss_instant: None,
            e_stop_latched: Arc::new(AtomicBool::new(false)),
            requested_estop_count: Arc::new(AtomicU64::new(0)),
            applied_estop_count: 0,
            last_preemption: None,
            last_evicted: None,
            handover_settling_since: None,
            gravity_comp: None,
            gravity_comp_last_exit: None,
            keepout: crate::keepout::Keepout::default(),
            poses: crate::poses::PoseStore::default(),
            residual_torques: None,
            collision: crate::collision::CollisionGuard::from_policy(&collision_policy),
            torque_guard: crate::torque_guard::TorqueGuard::default(),
            tracking: crate::tracking::TrackingRuntime::default(),
            power: crate::power::PowerMonitor::default(),
            follower: None,
            base_gripper_policy,
            gripper_calib_offset: None,
            gripper_calib_jaw_opening: None,
            gripper_calib_path: None,
            gripper_zero_checked: false,
            gripper_calib_invalid: false,
            gripper_calib_stale: None,
            last_zero_check_offset: None,
            mock_gripper_stall_offset,
            mock_gripper_stall_torque: false,
            mock_gripper_object: None,
            driver_config_error: None,
            zero_check_params: None,
            gripper_calib_revision: 1,
            grasp_setup_revision: 1,
            defer_zero_check_persist: false,
            zero_check_persisting: None,
            pending_zero_check_persist: None,
            operator_auth_enforced: false,
        }
    }

    /// Observation schema: profile capabilities plus the parts that depend on the driver.
    pub fn schema(&self) -> ObservationSchema {
        let mut schema = ObservationSchema::from_config(&self.config);
        schema.capabilities.gravity_comp = self.gravity_comp_support().is_ok();
        let torque_feedback = self.driver.as_ref().is_some_and(|d| d.lock().is_ok_and(|drv| drv.capabilities().torque_feedback));
        schema.capabilities.collision_guard &= torque_feedback;
        schema.capabilities.torque_guard &= torque_feedback;
        schema.capabilities.power_budget &= torque_feedback;
        schema.capabilities.auto_yaw = self.config.arm_dof() == 5;
        #[cfg(feature = "policy-runtime")]
        if let Some(ref mut pr) = schema.capabilities.policy_runtime {
            pr.modes = if self.policy_live_allowed() {
                vec!["shadow".to_string(), "live_sim".to_string()]
            } else {
                vec!["shadow".to_string(), "live_gated".to_string()]
            };
        }
        if schema.feetech_joints.is_none() {
            schema.feetech_joints = self.config.feetech.as_ref().map(|f| f.joints.clone());
        }
        schema.capabilities.calibration_tools = Some(self.calibration_tools_with(&schema.capabilities));
        if let Some(t) = schema.capabilities.teach.as_mut() { t.episodes = self.episodes.enabled(); }
        schema.joint_zero = self.calibration.as_ref().and_then(|c| c.joint_zero.as_ref()).and_then(|z| serde_json::to_value(z).ok());
        schema.zero_convention = self.config.zero_reference.as_ref().and_then(|_| serde_json::to_value(&self.zero_convention).ok());
        schema
    }

    /// `capabilities.calibration_tools`: the profile's camera / kinematics / gripper support
    /// and the driver's motor zero support.
    pub fn calibration_tools(&self) -> crate::model::CalibrationTools {
        self.calibration_tools_with(&crate::model::Capabilities::from_config(&self.config))
    }

    fn calibration_tools_with(&self, caps: &crate::model::Capabilities) -> crate::model::CalibrationTools {
        use crate::model::{CalibrationToolInfo, GripperCalibrationTool, MotorZeroTool};
        let no_grasp = "this profile has no grasp setup (the camera calibration needs one)";
        let kin_reason = if self.config.arm_model.chain().is_some() {
            "this profile uses a URDF chain; the kinematic correction applies to the built-in model only"
        } else { no_grasp };
        let mut support = self.motor_zero_support();
        let (motor, software) = (support.supported, self.config.zero_reference.is_some());
        let methods: Vec<String> = [(software, "software"), (motor, "motor")].into_iter().filter(|(ok, _)| *ok).map(|(_, m)| m.to_string()).collect();
        if software && !motor {
            support = crate::model::MotorZeroSupport { supported: true, notes: support.reason.take().map(|r| vec![format!("motor zero: {r}")]).unwrap_or_default(), ..Default::default() };
        }
        let any = motor || software;
        let motor_zero = MotorZeroTool {
            joints: if any { (1..=self.config.dof).collect() } else { Vec::new() },
            requires: if any { ["lease", "torque_off", "no_action", "no_fault"].map(String::from).to_vec() } else { Vec::new() },
            max_offset_deg: crate::model::MOTOR_ZERO_MAX_OFFSET_DEG,
            verify_mrad: crate::model::MOTOR_ZERO_VERIFY_MRAD,
            invalidates: if any { ["kinematics", "camera", "poses", "gripper_calibration"].map(String::from).to_vec() } else { Vec::new() },
            zero_pose: self.config.zero_pose.clone(),
            motor, software, methods,
            support,
        };
        let policy = self.config.gripper_policy.as_ref().filter(|_| self.config.gripper_index().is_some());
        let gripper = GripperCalibrationTool {
            supported: policy.is_some(),
            zero_check: policy.is_some_and(|p| p.zero_check_supported),
            calibration: policy.is_some(),
            reason: policy.is_none().then(|| "this profile has no gripper joint with a gripper policy".to_string()),
        };
        crate::model::CalibrationTools {
            camera: CalibrationToolInfo::new(caps.calibration_apply, Vec::new(), no_grasp),
            kinematics: CalibrationToolInfo::new(caps.calibration_kinematics, vec![2, 3, 4, 5], kin_reason),
            motor_zero,
            gripper,
        }
    }

    /// The driver's motor zero support; the mock follower (no driver) zeroes its readings.
    pub fn motor_zero_support(&self) -> crate::model::MotorZeroSupport {
        match self.driver.as_ref() {
            Some(d) => d.lock().map_or_else(|_| crate::model::MotorZeroSupport::unsupported("driver unavailable"), |drv| drv.motor_zero_support()),
            None => crate::model::MotorZeroSupport::set_current("process", true,
                vec!["mock follower: the readings of the zeroed joints become 0 (the simulated arm has no encoder offset)".into()]),
        }
    }

    /// Install joint zero offsets (motor - joint, rad, one per joint) at the driver boundary.
    /// The current readings, commanded and target positions move by old - new, so nothing jumps;
    /// the mock follower (no driver) moves its simulated readings the same way. Needs torque
    /// off; the caller checks the rest.
    pub fn set_joint_zero_offsets(&mut self, offsets: &[f32]) -> Result<(), String> {
        if self.is_torqued || self.disable_pending { return Err("torque is on".into()); }
        if offsets.len() != self.config.dof || !offsets.iter().all(|o| o.is_finite()) {
            return Err(format!("joint zero needs {} finite offsets", self.config.dof));
        }
        let old = std::mem::replace(&mut self.joint_zero_offsets, offsets.to_vec());
        self.tracking.invalidate();
        if old != offsets { self.calibration_revision += 1; self.refresh_effective_floor(); }
        if let Some(d) = self.driver.as_ref() {
            let mut drv = d.lock().map_err(|_| "driver unavailable".to_string())?;
            drv.set_joint_zero_offsets(offsets);
            let pos: Vec<_> = (0..self.config.dof).map(|i| drv.position_limits(i)).collect();
            drop(drv);
            if pos.iter().any(Option::is_some) { self.reapply_driver_limits(); }
        }
        for (i, (o, n)) in old.iter().zip(offsets).enumerate() {
            let d = o - n;
            if d == 0.0 { continue; }
            for v in [&mut self.measured_positions, &mut self.commanded_positions, &mut self.target_positions] {
                if let Some(p) = v.get_mut(i) { *p += d; }
            }
        }
        if old.len() > 5 && offsets.len() > 5 && old[5] != offsets[5] {
            self.zero_convention = Default::default();
        }
        self.zero_check_pending = true;
        Ok(())
    }

    /// The reference pose in joint angles (`home_positions` derived from `zero_reference`).
    pub fn zero_reference_joints(&self) -> Option<Vec<f32>> {
        self.config.zero_reference.as_ref().map(|_| self.config.home_positions.clone())
    }

    /// The zero check (`zero_convention`) at startup and each time torque goes off: with torque
    /// off and fresh feedback, an arm in the rest pose with J6 about +-90 deg from the reference is
    /// reported as `legacy_j6`. Never moves the arm. A `legacy_j6` verdict stays until a zero of J6.
    fn step_zero_convention(&mut self) {
        if self.zero_check_prev_torqued && !self.is_torqued { self.zero_check_pending = true; }
        self.zero_check_prev_torqued = self.is_torqued;
        if !self.zero_check_pending || self.is_torqued || self.calibration_busy { return; }
        let Some(rest) = self.zero_reference_joints() else { self.zero_check_pending = false; return; };
        if let Some(d) = self.driver.as_ref() {
            if !d.lock().is_ok_and(|drv| drv.is_feedback_valid()) { return; }
        }
        let tol = self.resting_policy().map(|p| p.position_tolerances).unwrap_or_else(|| vec![0.08; 6]);
        self.zero_check_pending = false;
        let Some(v) = crate::joint_zero::check_convention(&self.measured_positions, &rest, &tol) else { return; };
        if self.zero_convention.status == "legacy_j6" && v.status != "legacy_j6" { return; }
        if v.status == "legacy_j6" && self.zero_convention.status != "legacy_j6" {
            tracing::warn!("Joint zero check: {}", v.message.as_deref().unwrap_or(""));
        }
        self.zero_convention = v;
    }

    /// Make the current position of `joint` (0-based) its zero: the driver's
    /// [`ArmDriver::set_zero_position`], or for the mock follower the joint's reading (and
    /// commanded / target value) becomes 0. Needs torque off; the caller checks the rest.
    pub fn set_motor_zero(&mut self, joint: usize) -> Result<(), String> {
        if self.is_torqued || self.disable_pending { return Err("torque is on".into()); }
        if joint >= self.config.dof { return Err(format!("joint {} does not exist", joint + 1)); }
        // a new J6 zero ends a legacy_j6 verdict; the next check decides again
        if joint == 5 { self.zero_convention = Default::default(); }
        self.zero_check_pending = true;
        match self.driver.as_ref() {
            Some(d) => d.lock().map_err(|_| "driver unavailable".to_string())?.set_zero_position(joint).map_err(|e| e.to_string()),
            None => {
                // the simulated motor reads 0: the joint reads 0 - its joint zero offset
                let r = -self.joint_zero_offsets.get(joint).copied().unwrap_or(0.0);
                for v in [&mut self.measured_positions, &mut self.commanded_positions, &mut self.target_positions] {
                    if let Some(p) = v.get_mut(joint) { *p = r; }
                }
                Ok(())
            }
        }
    }

    pub(crate) fn estop_pending(&self) -> bool {
        self.e_stop_latched.load(Ordering::SeqCst) || self.requested_estop_count.load(Ordering::SeqCst) != self.applied_estop_count
    }

    fn reset_gripper_stall(&mut self) {
        self.gripper_stalled = false;
        self.gripper_stall_sign = 0.0;
        self.gripper_prev_hw_overload = false;
        self.gripper_high_torque_ticks = 0;
        self.gripper_low_torque_ticks = 0;
        self.gripper_force_capped = false;
        self.gripper_force_ticks = 0;
        self.gripper_stream_target = None;
        self.gripper_stream_contact = None;
    }

    pub fn with_driver(mut self, mut driver: Box<dyn ArmDriver>) -> Self {
        driver.set_gripper_index(self.config.gripper_index());
        if driver.capabilities().load_feedback {
            self.loads = vec![f32::NAN; self.config.dof];
        }
        let driver: Box<dyn ArmDriver> = Box::new(crate::joint_zero::ZeroOffsetDriver::new(driver, &self.joint_zero_offsets));
        self.apply_driver_limits(driver.as_ref());
        self.driver = Some(std::sync::Mutex::new(driver));
        self
    }

    pub fn sync_driver_feedback(
        &mut self,
        states: &[crate::model::JointState],
        loads: Option<&[f32]>,
    ) {
        Self::apply_feedback(
            &mut self.measured_positions,
            &mut self.measured_velocities,
            &mut self.torques,
            &mut self.temperatures,
            &mut self.loads,
            self.config.dof,
            states,
            loads,
        );
    }

    fn apply_feedback(
        measured_positions: &mut [f32],
        measured_velocities: &mut [f32],
        torques: &mut [f32],
        temperatures: &mut [f32],
        loads_vec: &mut Vec<f32>,
        dof: usize,
        states: &[crate::model::JointState],
        loads: Option<&[f32]>,
    ) {
        for (i, state) in states.iter().enumerate().take(dof) {
            measured_positions[i] = state.position;
            measured_velocities[i] = state.velocity;
            torques[i] = state.torque;
            temperatures[i] = state.temperature;
        }
        if let Some(loads) = loads {
            if loads.len() == dof && loads.iter().all(|x| x.is_finite()) {
                *loads_vec = loads.to_vec();
            }
        }
    }

    pub fn apply_driver_limits(&mut self, driver: &dyn ArmDriver) {
        let vel: Vec<_> = (0..self.config.dof).map(|i| driver.max_command_velocity(i)).collect();
        let pos: Vec<_> = (0..self.config.dof).map(|i| driver.position_limits(i)).collect();
        self.apply_extracted_driver_limits(&vel, &pos);
    }

    pub fn apply_extracted_driver_limits(
        &mut self,
        vel_limits: &[Option<f32>],
        pos_limits: &[Option<(f32, f32)>],
    ) {
        self.apply_extracted_driver_limits_inner(vel_limits, pos_limits, true);
    }

    pub fn apply_extracted_driver_limits_inner(
        &mut self,
        vel_limits: &[Option<f32>],
        pos_limits: &[Option<(f32, f32)>],
        trigger_fault: bool,
    ) {
        self.driver_config_error = None;
        let mut candidate = self.config.clone();

        for (i, limit) in candidate.limits.iter_mut().enumerate() {
            if let Some(Some(max)) = vel_limits.get(i) {
                if max.is_finite() && *max > 0.0 {
                    limit.max_velocity = limit.max_velocity.min(*max);
                }
            }
            if let Some(Some((d_min, d_max))) = pos_limits.get(i).copied() {
                if !d_min.is_finite() || !d_max.is_finite() || d_min >= d_max {
                    let reason = format!(
                        "Driver position limits [{:.4}, {:.4}] for joint '{}' are invalid",
                        d_min, d_max, limit.name
                    );
                    self.driver_config_error = Some(reason.clone());
                    if trigger_fault {
                        self.safety.trigger_fault_coded(FaultCode::JointLimit, FaultReason::Other(reason));
                    }
                    return;
                }
                let new_min = limit.min_position.max(d_min);
                let new_max = limit.max_position.min(d_max);
                if !new_min.is_finite() || !new_max.is_finite() || new_max - new_min < 1e-2 {
                    let reason = format!(
                        "Driver position limits [{:.4}, {:.4}] for joint '{}' conflict with profile limits [{:.4}, {:.4}] (intersected range < 0.01)",
                        d_min, d_max, limit.name, limit.min_position, limit.max_position
                    );
                    self.driver_config_error = Some(reason.clone());
                    if trigger_fault {
                        self.safety.trigger_fault_coded(FaultCode::JointLimit, FaultReason::Other(reason));
                    }
                    return;
                }
                if let Some(pos) = candidate.resting_pose.as_ref().and_then(|r| r.positions.as_ref()).and_then(|p| p.get(i).copied()) {
                    if pos < new_min || pos > new_max {
                        let reason = format!(
                            "Driver position limits [{:.4}, {:.4}] for joint '{}' conflict with resting pose position {:.4}",
                            d_min, d_max, limit.name, pos
                        );
                        self.driver_config_error = Some(reason.clone());
                        if trigger_fault {
                            self.safety.trigger_fault_coded(FaultCode::JointLimit, FaultReason::Other(reason));
                        }
                        return;
                    }
                }
                if (new_min - limit.min_position).abs() > 1e-4 || (new_max - limit.max_position).abs() > 1e-4 {
                    tracing::info!(
                        "Intersected joint '{}' position limits with driver range: profile=[{:.4}, {:.4}], hw=[{:.4}, {:.4}] -> [{:.4}, {:.4}]",
                        limit.name, limit.min_position, limit.max_position, d_min, d_max, new_min, new_max
                    );
                }
                limit.min_position = new_min;
                limit.max_position = new_max;
            }
        }

        if candidate.validate().is_err()
            && self.base_gripper_policy.is_some()
            && (self.gripper_calib_offset.is_some() || self.gripper_calib_jaw_opening.is_some())
        {
            let mut base_test = candidate.clone();
            base_test.gripper_policy = self.base_gripper_policy.clone();
            if base_test.validate().is_ok() {
                tracing::warn!("Driver limits invalidated outdated calibrated gripper policy; reverting to base uncalibrated policy");
                self.gripper_calib_invalid = true;
                self.gripper_calib_offset = None;
                self.gripper_calib_jaw_opening = None;
                self.config.gripper_policy = self.base_gripper_policy.clone();
                candidate = base_test;
            }
        }

        if let Err(err) = candidate.validate() {
            let reason = format!("Driver limits violated config invariants: {err}");
            self.driver_config_error = Some(reason.clone());
            if trigger_fault {
                self.safety.trigger_fault_coded(FaultCode::JointLimit, FaultReason::Other(reason));
            }
            return;
        }

        self.config = candidate;
    }

    pub fn reapply_driver_limits(&mut self) -> bool {
        let limits = if let Some(ref d) = self.driver {
            if let Ok(drv) = d.lock() {
                let vel: Vec<_> = (0..self.config.dof).map(|i| drv.max_command_velocity(i)).collect();
                let pos: Vec<_> = (0..self.config.dof).map(|i| drv.position_limits(i)).collect();
                Some((vel, pos))
            } else {
                None
            }
        } else {
            None
        };
        if let Some((vel_limits, pos_limits)) = limits {
            self.apply_extracted_driver_limits_inner(&vel_limits, &pos_limits, false);
            self.driver_config_error.is_none()
        } else {
            false
        }
    }

    /// Joint limits used for planning and servo stepping: `config.limits` with
    /// `max_velocity` capped at `config.planned_max_velocity` for every joint
    /// except the 7-DOF gripper (index 6).
    /// `speed_cap.joint_velocity_scale` then scales the result (same joints).
    pub fn planning_limits(&self) -> Vec<crate::model::JointLimit> {
        let mut limits = self.config.limits.clone();
        let scale = self.config.speed_cap.and_then(|c| c.joint_velocity_scale).unwrap_or(1.0);
        let gripper_idx = self.config.gripper_index();
        for (i, limit) in limits.iter_mut().enumerate() {
            if Some(i) != gripper_idx {
                if let Some(cap) = self.config.planned_max_velocity {
                    limit.max_velocity = limit.max_velocity.min(cap);
                }
                limit.max_velocity *= scale;
            }
        }
        limits
    }

    /// Joint limits used for servo stepping: `config.limits` with `max_velocity` capped
    /// at `config.servo_max_velocity` per joint, except the 7-DOF gripper (index 6).
    /// When a live policy run has a `verified_release_gate`, each joint's `max_velocity`
    /// is further scaled by `gate.max_speed_scale().clamp(0.1, 1.0)`.
    pub fn servo_limits(&self) -> Vec<crate::model::JointLimit> {
        #[cfg(feature = "policy-runtime")]
        {
            let mut limits = self.config.servo_limits();
            if let Some(ref run) = self.policy_run {
                if run.mode == crate::policy_runtime::PolicyMode::Live {
                    if let Some(ref gate) = run.verified_release_gate {
                        let scale = (gate.max_speed_scale() as f32).clamp(0.1, 1.0);
                        for limit in &mut limits {
                            limit.max_velocity *= scale;
                        }
                    }
                }
            }
            limits
        }
        #[cfg(not(feature = "policy-runtime"))]
        {
            self.config.servo_limits()
        }
    }

    /// Store the motor error codes, logging each per-joint change once.
    fn update_motor_errors(&mut self, errors: Vec<Option<crate::model::MotorError>>) {
        if errors == self.motor_errors { return; }
        for (i, e) in errors.iter().enumerate() {
            let prev = self.motor_errors.get(i).cloned().flatten();
            match (prev, e) {
                (p, Some(e)) if p.as_ref() != Some(e) =>
                    tracing::warn!("Joint {} motor reports error code 0x{:X} ({})", i + 1, e.code, e.name),
                (Some(p), None) => tracing::info!("Joint {} motor error 0x{:X} ({}) cleared", i + 1, p.code, p.name),
                _ => {}
            }
        }
        self.motor_errors = errors;
    }

    /// Report that `reason` shapes the motion now. The same reason keeps its start time;
    /// a fresh limit of higher priority is not replaced by a lower one.
    pub fn note_motion_limit(&mut self, reason: MotionLimitReason, joint: Option<usize>) {
        let now = self.now_s();
        if let Some(cur) = self.motion_limit.as_mut() {
            if cur.reason == reason {
                cur.joint = joint;
                self.motion_limit_seen_s = now;
                return;
            }
            if now - self.motion_limit_seen_s < MOTION_LIMIT_CLEAR_S && cur.reason.priority() > reason.priority() {
                return;
            }
        }
        if reason != MotionLimitReason::Workspace {
            self.workspace_violation = None;
        }
        self.motion_limit = Some(MotionLimit { reason, joint, since_ms: (now * 1000.0) as u64 });
        self.motion_limit_seen_s = now;
    }

    /// End of tick: keep `blocked` while the hold lasts, clear a limit not seen for
    /// [`MOTION_LIMIT_CLEAR_S`].
    fn expire_motion_limit(&mut self) {
        if let Some(h) = self.torque_guard.hold { self.note_motion_limit(MotionLimitReason::Torque, Some(h.joint)); }
        if self.gripper_stalled && self.gripper_force_capped {
            self.note_motion_limit(MotionLimitReason::GripperForce, self.config.gripper_index());
        }
        if self.power.gov.hold.is_some() {
            let j = self.motion_limit.as_ref().filter(|l| l.reason == MotionLimitReason::Power).and_then(|l| l.joint);
            self.note_motion_limit(MotionLimitReason::Power, j);
        }
        if self.servo_blocked_hold {
            #[cfg(feature = "policy-runtime")]
            let policy_active = self.policy_run.is_some();
            #[cfg(not(feature = "policy-runtime"))]
            let policy_active = false;
            let resumed = (self.servo_target.is_some() && !policy_active)
                || self.active_trajectory.is_some()
                || self.macro_run.is_some()
                || self.gravity_comp.is_some()
                || !self.is_torqued
                || self.safety.state().is_stopped();
            if resumed {
                self.clear_blocked_hold();
            } else {
                self.motion_limit_seen_s = self.now_s();
            }
        }
        if self.motion_limit.is_some() && self.now_s() - self.motion_limit_seen_s > MOTION_LIMIT_CLEAR_S {
            self.motion_limit = None;
            self.workspace_violation = None;
        }
    }

    /// The control loop dropped or could not follow a live policy target: end the owning
    /// policy action as failed with `reason` (no-op without a live policy run).
    #[allow(unused_variables)]
    fn fail_live_policy_from_control(&mut self, reason: &str, message: String) {
        #[cfg(feature = "policy-runtime")]
        if self.policy_run.as_ref().is_some_and(|r| r.mode == crate::policy_runtime::PolicyMode::Live) {
            tracing::warn!("Policy run ended by the control loop: {reason}: {message}");
            self.terminate_policy_run(reason, "failed", Some(reason), Some(message));
        }
    }

    fn clear_blocked_hold(&mut self) {
        self.servo_blocked_hold = false;
        if self.motion_limit.as_ref().is_some_and(|l| l.reason == MotionLimitReason::Blocked) {
            self.motion_limit = None;
        }
    }

    /// Servo streaming found joint `joint` blocked: end the stream and hold the measured
    /// pose with torque on, without a Fault. The gripper keeps its command so that a held
    /// object is not released. A tracking error that persists in this hold latches the
    /// usual tracking Fault (the stream is no longer active).
    fn enter_servo_blocked_hold(&mut self, joint: usize, why: &str) {
        tracing::warn!(
            "Joint {} blocked during servo streaming ({why}; cmd {:.3} meas {:.3} rad): stream stopped, holding position",
            joint + 1, self.commanded_positions[joint], self.measured_positions[joint]);
        #[cfg(feature = "policy-runtime")]
        self.terminate_policy_run(
            "BLOCKED_HOLD",
            "cancelled",
            Some("BLOCKED_HOLD"),
            Some(format!("Joint {} blocked during policy streaming: {why}", joint + 1)),
        );
        self.servo_hold(MotionLimitReason::Blocked, Some(joint));
    }

    /// The servo / braking step to `next` violates the workspace: same recoverable hold.
    fn enter_servo_keepout_hold(&mut self, e: &SafetyError) {
        tracing::warn!("Servo step violates workspace ({e}): stream stopped, holding position");
        #[cfg(feature = "policy-runtime")]
        self.terminate_policy_run(
            "WORKSPACE_HOLD",
            "cancelled",
            Some("WORKSPACE_HOLD"),
            Some(format!("Policy step violated workspace: {e}")),
        );
        self.servo_hold(MotionLimitReason::Workspace, None);
        if self.motion_limit.as_ref().is_some_and(|m| m.reason == MotionLimitReason::Workspace) {
            self.workspace_violation = e.workspace_detail();
        }
    }

    /// Joint-space step from the commanded position to `next` (linear in joint space)
    /// against the full workspace (floor, outer box, reach, keep-out boxes, link capsules),
    /// using the persistent per-constraint monotonic escape rule and up to [`MAX_KEEPOUT_EVALS_PER_TICK`]
    /// added FK evaluations.
    pub fn servo_step_keepout(&mut self, next: &[f32]) -> Result<(), SafetyError> {
        self.servo_step_workspace(next, "servo_step")
    }

    /// Full workspace sweep check for a servo streaming or braking tick.
    pub fn servo_step_workspace(&mut self, next: &[f32], phase: &str) -> Result<(), SafetyError> {
        let Some((ws, cam)) = self.workspace_policy() else { return Ok(()); };
        let eval = crate::chain::WorkspaceEvaluator::new(&self.config.arm_model, ws, cam);
        let mut c0 = eval.alloc_buf();
        eval.eval_into(&self.commanded_positions, &mut c0, phase)
            .map_err(SafetyError::from_workspace)?;
        let mut cand_min_d = match &self.servo_ws_min_depths {
            Some((rev, min_d)) if *rev == self.workspace_revision => min_d.clone(),
            _ => eval.init_min_depths(&c0),
        };
        let mut budget = MAX_KEEPOUT_EVALS_PER_TICK;
        eval.sweep_linear_step(
            &self.commanded_positions,
            next,
            &c0,
            &mut cand_min_d,
            &mut budget,
            MAX_KEEPOUT_EVALS_PER_TICK,
            phase,
            phase,
        )
        .map_err(|(_, f)| SafetyError::from_workspace(f))?;
        self.servo_ws_min_depths = Some((self.workspace_revision, cand_min_d));
        Ok(())
    }

    /// Conservative keep-out check of the joint path `at(t)` over `knots` (ascending).
    ///
    /// Every check point (end_link origin, `check_points_end_m`, camera offset) moves along
    /// an interval `[t0, t1]` at most `L = sum_j travel_j * reach_j`, with `travel` an upper
    /// bound on each joint's travel and `reach_j` the largest distance from joint `j`'s axis to
    /// a check point ([`ArmModel::joint_reach`](crate::chain::ArmModel::joint_reach)). A
    /// point at Euclidean distances `d0`, `d1` from a box at the interval ends cannot enter
    /// it when `d0 + d1 > L`. An interval where that fails for some point and box is split at
    /// its midpoint; one with `L` at most [`WORKSPACE_EPS`](crate::kinematics::WORKSPACE_EPS)
    /// is accepted (the depth rule's tolerance). A sample with a point deeper than the
    /// tolerance in a box is WORKSPACE_VIOLATION; so is needing more than `budget` added
    /// evaluations. Boxes a point is already inside at `at(knots[0])` are left to the
    /// sample rule (a move out of a box is allowed). Returns the evaluated samples in order.
    pub fn keepout_sweep(
        &self,
        ws: &crate::model::WorkspacePolicy,
        cam: &crate::model::CameraMount,
        at: &dyn Fn(f64) -> Vec<f32>,
        travel: &dyn Fn(f64, f64) -> Vec<f64>,
        knots: &[f64],
        budget: usize,
    ) -> Result<Vec<Vec<f32>>, SafetyError> {
        keepout_sweep_with(&self.config.arm_model, ws, cam, at, travel, knots, budget)
    }

    fn servo_hold(&mut self, reason: MotionLimitReason, joint: Option<usize>) {
        self.hold_measured_pose();
        self.note_motion_limit(reason, joint);
        self.servo_blocked_hold = true;
    }

    /// The command `hold_measured_pose` holds: the measured pose clamped to the limits
    /// (the current command when a measurement is not finite), the gripper keeping its command.
    pub(crate) fn measured_hold_pose(&self) -> Vec<f32> {
        let mut hold = self.commanded_positions.clone();
        if self.measured_positions.iter().all(|q| q.is_finite()) {
            for (i, (c, m)) in hold.iter_mut().zip(&self.measured_positions).enumerate() {
                let l = &self.config.limits[i];
                *c = m.clamp(l.min_position, l.max_position);
            }
        }
        if let Some(g) = self.config.gripper_index() {
            if let (Some(c), Some(&cmd)) = (hold.get_mut(g), self.commanded_positions.get(g)) {
                *c = cmd;
            }
        }
        hold
    }

    /// End every motion and hold the measured pose (clamped to the limits) with torque on;
    /// the gripper keeps its command so that a held object stays held.
    pub(crate) fn hold_measured_pose(&mut self) {
        self.stop_motion();
        self.servo_stream_active = false;
        self.tracking.invalidate();
        self.commanded_positions = self.measured_hold_pose();
        self.target_positions = self.commanded_positions.clone();
        self.commanded_velocities.fill(0.0);
        self.commanded_accelerations.fill(0.0);
        self.following_error_since = None;
        self.servo_lead_stall.fill(None);
        self.safety.mark_motion_finished();
    }

    /// Workspace policy and camera mount when the Cartesian check applies
    /// (workspace and kinematics configured, 7-DOF arm).
    pub fn workspace_policy(&self) -> Option<(&crate::model::WorkspacePolicy, &crate::model::CameraMount)> {
        match (&self.config.workspace, self.config.kinematics) {
            (Some(ws), Some(_)) if self.config.dof >= self.config.arm_dof() => Some((ws, &self.config.camera_mount)),
            _ => None,
        }
    }

    /// Reject a move whose samples leave the workspace further than `start` already is.
    pub fn check_workspace_path<'a>(
        &self,
        start: &[f32],
        samples: impl IntoIterator<Item = &'a [f32]>,
    ) -> Result<(), SafetyError> {
        let Some((ws, cam)) = self.workspace_policy() else { return Ok(()); };
        self.check_path_against(ws, cam, start, samples)
    }

    /// [`check_workspace_path`](Self::check_workspace_path) against the keep-out boxes only
    /// (no floor / box / reach): the part of the rule park may not bypass.
    pub fn check_keepout_path<'a>(
        &self,
        start: &[f32],
        samples: impl IntoIterator<Item = &'a [f32]>,
    ) -> Result<(), SafetyError> {
        let Some((ws, cam)) = self.workspace_policy() else { return Ok(()); };
        if ws.keepout_boxes.is_empty() { return Ok(()); }
        let only = ws.keepout_only();
        self.check_path_against(&only, cam, start, samples)
    }

    fn check_path_against<'a>(
        &self,
        ws: &crate::model::WorkspacePolicy,
        cam: &crate::model::CameraMount,
        start: &[f32],
        samples: impl IntoIterator<Item = &'a [f32]>,
    ) -> Result<(), SafetyError> {
        check_path_with(&self.config.arm_model, ws, cam, start, samples)
    }

    /// The planner's 21 verification points of a trajectory.
    pub fn trajectory_samples(traj: &MultiJointQuinticTrajectory) -> Vec<Vec<f32>> {
        Self::uniform_samples(traj, 20)
    }

    fn uniform_samples(traj: &MultiJointQuinticTrajectory, n: usize) -> Vec<Vec<f32>> {
        (0..=n).map(|k| traj.sample(traj.duration * k as f64 / n as f64).positions).collect()
    }

    /// Workspace verification points of a joint-space trajectory: the 21 planner samples
    /// verified against the full workspace (floor, outer box, reach, keep-out boxes,
    /// link capsules) via the conservative sweep check
    /// ([`keepout_sweep`](Self::keepout_sweep), at most [`MAX_KEEPOUT_EVALS_PER_PLAN`] added
    /// FK evaluations; a path that needs more, or that violates, is WORKSPACE_VIOLATION).
    pub fn workspace_samples(&self, traj: &MultiJointQuinticTrajectory) -> Result<Vec<Vec<f32>>, SafetyError> {
        self.workspace_samples_with_budget(traj, MAX_KEEPOUT_EVALS_PER_PLAN)
    }

    /// [`workspace_samples`](Self::workspace_samples) with `budget` added FK evaluations.
    pub fn workspace_samples_with_budget(&self, traj: &MultiJointQuinticTrajectory, budget: usize) -> Result<Vec<Vec<f32>>, SafetyError> {
        let Some((ws, cam)) = self.workspace_policy()
            else { return Ok(Self::trajectory_samples(traj)); };
        let knots: Vec<f64> = (0..=20).map(|k| traj.duration * k as f64 / 20.0).collect();
        let at = |t: f64| traj.sample(t).positions;
        let travel = |t0: f64, t1: f64| traj.travel_bound(t0, t1);
        self.keepout_sweep(ws, cam, &at, &travel, &knots, budget)
    }

    /// Keepout-only sweep samples for `park` (exempt from floor / outer box / reach, still enforces keep-out boxes).
    pub fn keepout_samples(&self, traj: &MultiJointQuinticTrajectory) -> Result<Vec<Vec<f32>>, SafetyError> {
        let Some((ws, cam)) = self.workspace_policy().filter(|(ws, _)| !ws.keepout_boxes.is_empty())
            else { return Ok(Self::trajectory_samples(traj)); };
        let only = ws.keepout_only();
        let knots: Vec<f64> = (0..=20).map(|k| traj.duration * k as f64 / 20.0).collect();
        let at = |t: f64| traj.sample(t).positions;
        let travel = |t0: f64, t1: f64| traj.travel_bound(t0, t1);
        self.keepout_sweep(&only, cam, &at, &travel, &knots, MAX_KEEPOUT_EVALS_PER_PLAN)
    }

    fn gravity_output_enabled(&self) -> bool {
        self.gravity_compensated && self.config.gravity_model.is_some() && self.config.dof >= self.config.arm_dof() && self.driver.as_ref().is_none_or(|d|
            d.lock().is_ok_and(|drv| drv.capabilities().torque_feedforward))
    }

    fn states_valid(
        states: &[crate::model::JointState],
        caps: DriverCapabilities,
        dof: usize,
        loads: Option<&[f32]>,
    ) -> bool {
        states.len() == dof
            && states.iter().all(|s| {
                s.position.is_finite()
                    && (!caps.velocity_feedback || s.velocity.is_finite())
                    && (!caps.torque_feedback || s.torque.is_finite())
                    && (!caps.temperature_feedback || s.temperature.is_finite())
            })
            && (!caps.load_feedback
                || loads.is_some_and(|l| l.len() == dof && l.iter().all(|x| x.is_finite())))
    }

    pub fn hardware_feedback_fresh(&self, max_age: Duration) -> bool {
        if !self.feedback_numeric_valid { return false; }
        self.driver.as_ref().is_none_or(|d| d.lock().is_ok_and(|drv| {
            // Ages alone decide: drivers' is_feedback_valid() hard-codes 250 ms,
            // which would silently cap callers asking for a longer bound.
            let ages = drv.feedback_ages();
            ages.len() == self.config.dof
                && ages.iter().all(|a| a.is_some_and(|age| age <= max_age))
        }))
    }

    /// Explicit user release uses the same bounded deceleration as a stream timeout.
    pub fn stop_motion(&mut self) {
        #[cfg(feature = "policy-runtime")]
        self.terminate_policy_run("STOPPED", "cancelled", Some("STOPPED"), Some("Motion stopped".into()));
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        self.servo_ws_min_depths = None;
        self.active_trajectory = None;
        self.pending_normal_unload = false;
        self.unload_settle = None;
        self.zero_check_persisting = None;
        self.target_positions = self.commanded_positions.clone();
    }

    /// A successful transport write is not confirmation that torque was removed.
    fn request_disable(&mut self) -> Result<(), SafetyError> {
        self.disable_pending = true;
        self.tracking.invalidate();
        self.tracking.clear_recovery_suspension();
        self.last_disable_attempt = Some(Instant::now());
        if let Some(ref d) = self.driver {
            let mut drv = d.lock().map_err(|_| SafetyError::FeedbackUnavailable)?;
            drv.set_torque(false).map_err(|e| SafetyError::ClearFaultRejected(format!("Torque disable transmission failed: {e}")))?;
            let poll_start = Instant::now();
            while poll_start.elapsed() < Duration::from_millis(25) {
                let _ = drv.read_joint_states();
                if drv.is_feedback_valid() && drv.enable_state() == EnableState::Disabled {
                    break;
                }
                std::thread::sleep(Duration::from_millis(2));
            }
            if !drv.is_feedback_valid() || drv.enable_state() != EnableState::Disabled {
                return Err(SafetyError::ClearFaultRejected("Torque disable has not been confirmed by fresh hardware feedback".into()));
            }
        }
        self.is_torqued = false;
        self.pending_rearm = false;
        self.disable_pending = false;
        Ok(())
    }

    pub fn current_observation(&self) -> ArmObservation {
        let timestamp = self.start_time.elapsed().as_secs_f64();
        let tcp = if self.config.kinematics.is_some() {
            self.config.arm_model.fk_f32(&self.measured_positions).map(|pose| {
                let rpy = crate::kinematics::r_to_rpy(&pose.r);
                let quat = crate::kinematics::r_to_quat(&pose.r);
                crate::model::TcpState {
                    xyz_m: [pose.p[0] as f32, pose.p[1] as f32, pose.p[2] as f32],
                    rpy_rad: [rpy[0] as f32, rpy[1] as f32, rpy[2] as f32],
                    quat_wxyz: [quat[0] as f32, quat[1] as f32, quat[2] as f32, quat[3] as f32],
                }
            })
        } else {
            None
        };
        let gripper = if let (Some(idx), Some(policy)) = (self.config.gripper_index(), self.config.gripper_policy.as_ref()) {
            if let Some(&rad) = self.measured_positions.get(idx) {
                let span = policy.open_rad - policy.closed_rad;
                let fraction = if span.abs() < 1e-6 { 0.0 } else { ((rad - policy.closed_rad) / span).clamp(0.0, 1.0) };
                let opening_m = policy.jaw_opening.as_ref().map(|jo| jo.opening_m(rad));
                Some(crate::model::GripperObs {
                    joint: idx,
                    rad,
                    fraction,
                    opening_m,
                    stalled: self.gripper_stalled,
                    force_limited: self.gripper_stalled && self.gripper_force_capped,
                    force_limit_trips: self.gripper_force_trips,
                })
            } else {
                None
            }
        } else {
            None
        };
        let driver_capabilities = self.driver.as_ref().and_then(|d| d.lock().ok().map(|drv| drv.capabilities())).unwrap_or_default();
        ArmObservation {
            timestamp,
            positions: self.measured_positions.clone(),
            velocities: self.measured_velocities.clone(),
            torques: self.torques.clone(),
            temperatures: self.temperatures.clone(),
            is_torqued: self.is_torqued,
            safety_state: self.safety.state().clone(),
            epoch_id: self.safety.epoch_id(),
            gravity_compensated: self.gravity_output_enabled(),
            gravity_torques: self.gravity_torques.clone(),
            driver_capabilities,
            enable_state: self.driver.as_ref().map_or(if self.is_torqued { EnableState::Enabled } else { EnableState::Disabled }, |d| d.lock().map_or(EnableState::Unknown, |drv| drv.enable_state())),
            feedback_age_ms: self.driver.as_ref().and_then(|d| d.lock().ok().map(|drv| drv.feedback_ages().iter().map(|a| a.map(|v| v.as_millis().min(u64::MAX as u128) as u64)).collect())).unwrap_or_else(|| vec![Some(0); self.config.dof]),
            disable_pending: self.disable_pending,
            feedback_valid: self.hardware_feedback_fresh(Duration::from_millis(250)),
            commanded_positions: self.commanded_positions.clone(),
            action: self.last_action.clone(),
            previous_action: self.previous_action.as_ref().filter(|(_, until)| timestamp < *until).map(|(a, _)| a.clone()),
            control_mode: if self.gravity_comp.is_some() { crate::model::ControlMode::GravityComp } else { crate::model::ControlMode::Position },
            gravity_comp: self.gravity_comp_observation(),
            driver_range_check: self.driver.as_ref().and_then(|d| d.lock().ok().and_then(|drv| drv.range_check())),
            lease_holder: self.lease_holder(),
            motion_limit: self.motion_limit.clone(),
            workspace_violation: self.workspace_violation.clone(),
            motor_errors: self.motor_errors.clone(),
            fault: self.safety.fault(),
            residual_torques: self.residual_torques,
            collision: self.collision_observation(),
            power: self.power_observation(),
            gripper_zero: self.gripper_zero_observation(),
            tcp,
            gripper,
            loads: (driver_capabilities.load_feedback && self.loads.len() == self.config.dof).then(|| self.loads.clone()),
            #[cfg(feature = "policy-runtime")]
            policy: self.policy_observation(),
            #[cfg(not(feature = "policy-runtime"))]
            policy: None,
        }
    }

    pub fn is_at_safe_resting_pose(&self) -> bool {
        self.resting_pose_violation().is_none()
    }

    /// The resting-pose conditions that fail on the current sample, or `None` when the arm
    /// may be released. Velocity is judged on this one sample.
    fn resting_pose_violation(&self) -> Option<String> {
        self.resting_pose_static_violation().or_else(|| {
            let policy = self.resting_policy()?;
            let moving: Vec<String> = self.measured_velocities.iter().enumerate()
                .filter(|(i, v)| !v.is_finite() || v.abs() > policy.max_velocity_of(*i))
                .map(|(i, v)| format!("joint {} moving at {v:.4} rad/s (limit {})", i + 1, policy.max_velocity_of(i)))
                .collect();
            (!moving.is_empty()).then(|| moving.join("; "))
        })
    }

    fn resting_policy(&self) -> Option<RestingPosePolicy> {
        match &self.config.resting_pose {
            Some(policy) => Some(policy.clone()),
            None if self.driver.is_none() => Some(RestingPosePolicy::rebot_dm()),
            None => None,
        }
    }

    /// Every resting-pose condition except stationarity: feedback, enable state, position,
    /// support torque. Lists every failing joint with its value.
    pub fn resting_pose_static_violation(&self) -> Option<String> {
        let dof = self.config.dof;
        if self.measured_positions.len() != dof || self.measured_velocities.len() != dof {
            return Some("feedback length does not match dof".into());
        }
        let Some(policy) = self.resting_policy() else { return Some("no resting pose configured".into()); };
        if let Some(ref d) = self.driver {
            let Ok(drv) = d.lock() else { return Some("driver lock poisoned".into()); };
            let caps = drv.capabilities();
            if !caps.velocity_feedback || !caps.enable_feedback {
                return Some("driver lacks velocity/enable feedback".into());
            }
            if !policy.support_torque_limits.is_empty() && !caps.torque_feedback {
                return Some("driver lacks torque feedback for support torque limits".into());
            }
            if !policy.support_load_limits.is_empty() && !caps.load_feedback {
                return Some("driver lacks load feedback for support load limits".into());
            }
            let enable = drv.enable_state();
            if enable != EnableState::Enabled && !drv.has_known_partial_enable() { return Some(format!("enable state {enable:?}")); }
            if self.safety.state().is_stopped() { return Some(format!("safety state {:?}", self.safety.state())); }
            drop(drv);
            if !self.hardware_feedback_fresh(Duration::from_millis(policy.feedback_max_age_ms)) {
                return Some(format!("feedback older than {} ms", policy.feedback_max_age_ms));
            }
        }
        let mut failures = Vec::new();
        let mut total = 0.0;
        let rest = self.config.resting_positions();
        for (i, tolerance) in policy.position_tolerances.iter().enumerate().take(dof) {
            let dev = (self.measured_positions[i] - rest[i]).abs();
            if !dev.is_finite() || dev > *tolerance {
                failures.push(format!("joint {} off home by {dev:.4} rad (limit {tolerance})", i + 1));
            }
            total += dev;
        }
        if failures.is_empty() && total > policy.max_total_deviation {
            failures.push(format!("total deviation {total:.4} rad (limit {})", policy.max_total_deviation));
        }
        if self.driver.is_some() {
            for &(i, max_tau) in &policy.support_torque_limits {
                if i >= dof { return Some(format!("support torque index {i} out of range")); }
                let tau = self.torques.get(i).copied().unwrap_or(f32::NAN);
                if !tau.is_finite() || tau.abs() > max_tau {
                    failures.push(format!("joint {} torque {tau:.2} Nm (limit {max_tau})", i + 1));
                }
            }
            for &(i, max_load) in &policy.support_load_limits {
                if i >= dof { return Some(format!("support load index {i} out of range")); }
                let load = self.loads.get(i).copied().unwrap_or(f32::NAN);
                if !load.is_finite() || load.abs() > max_load {
                    failures.push(format!("joint {} load {load:.2} (limit {max_load})", i + 1));
                }
            }
        }
        (!failures.is_empty()).then(|| failures.join("; "))
    }

    /// Resting-pose position tolerances and support torque/load limits without requiring
    /// operational state (!is_stopped()) or enabled driver. Used to guard disabling
    /// supporting joints during fault clearing.
    pub fn resting_pose_support_violation_with_driver(&self, drv: Option<&dyn ArmDriver>) -> Option<String> {
        let dof = self.config.dof;
        if self.measured_positions.len() != dof || self.measured_velocities.len() != dof {
            return Some("feedback length does not match dof".into());
        }
        let Some(policy) = self.resting_policy() else {
            return Some("no resting pose configured".into());
        };
        if self.driver.is_some() {
            let Some(driver) = drv else {
                return Some("driver feedback unavailable".into());
            };
            let caps = driver.capabilities();
            if !caps.velocity_feedback {
                return Some("driver lacks velocity feedback".into());
            }
            if !policy.support_torque_limits.is_empty() && !caps.torque_feedback {
                return Some("driver lacks torque feedback for support torque limits".into());
            }
            if !policy.support_load_limits.is_empty() && !caps.load_feedback {
                return Some("driver lacks load feedback for support load limits".into());
            }
            let max_age = Duration::from_millis(policy.feedback_max_age_ms);
            let ages = driver.feedback_ages();
            if !self.feedback_numeric_valid
                || ages.len() != dof
                || !ages.iter().all(|a| a.is_some_and(|age| age <= max_age))
            {
                return Some(format!("feedback older than {} ms", policy.feedback_max_age_ms));
            }
        }
        let mut failures = Vec::new();
        for (i, &v) in self.measured_velocities.iter().enumerate().take(dof) {
            let limit = policy.max_velocity_of(i);
            if !v.is_finite() || v.abs() > limit {
                failures.push(format!("joint {} moving at {v:.4} rad/s (limit {limit})", i + 1));
            }
        }
        let mut total = 0.0;
        let rest = self.config.resting_positions();
        for (i, tolerance) in policy.position_tolerances.iter().enumerate().take(dof) {
            let dev = (self.measured_positions[i] - rest[i]).abs();
            if !dev.is_finite() || dev > *tolerance {
                failures.push(format!("joint {} off home by {dev:.4} rad (limit {tolerance})", i + 1));
            }
            total += dev;
        }
        if failures.is_empty() && total > policy.max_total_deviation {
            failures.push(format!("total deviation {total:.4} rad (limit {})", policy.max_total_deviation));
        }
        if self.driver.is_some() {
            for &(i, max_tau) in &policy.support_torque_limits {
                if i >= dof {
                    return Some(format!("support torque index {i} out of range"));
                }
                let tau = self.torques.get(i).copied().unwrap_or(f32::NAN);
                if !tau.is_finite() || tau.abs() > max_tau {
                    failures.push(format!("joint {} torque {tau:.2} Nm (limit {max_tau})", i + 1));
                }
            }
            for &(i, max_load) in &policy.support_load_limits {
                if i >= dof {
                    return Some(format!("support load index {i} out of range"));
                }
                let load = self.loads.get(i).copied().unwrap_or(f32::NAN);
                if !load.is_finite() || load.abs() > max_load {
                    failures.push(format!("joint {} load {load:.2} (limit {max_load})", i + 1));
                }
            }
        }
        (!failures.is_empty()).then(|| failures.join("; "))
    }

    /// Stationary check over the settle hold window: per joint, the position span, the
    /// mean velocity over the window and over its last [`UNLOAD_STILL_RECENT`], and every
    /// velocity sample. A single quantized velocity sample above
    /// the resting-pose limit is noise when the position does not move and the mean holds.
    fn unload_still_violation(&self, settle: &UnloadSettle) -> Option<String> {
        let policy = self.resting_policy()?;
        let hold = UNLOAD_SETTLE_HOLD.as_secs_f64();
        if settle.held_s + 1e-6 < hold || settle.samples.is_empty() {
            return Some(format!("held {:.2} s of {hold:.2} s", settle.held_s));
        }
        let n = settle.samples.len() as f32;
        let recent_from = settle.pending_s - UNLOAD_STILL_RECENT.as_secs_f64() - 1e-6;
        let mut failures = Vec::new();
        for i in 0..self.config.dof {
            let (mut lo, mut hi, mut sum, mut peak) = (f32::INFINITY, f32::NEG_INFINITY, 0.0f32, 0.0f32);
            let (mut recent_sum, mut recent_n) = (0.0f32, 0u32);
            for (t, q, v) in &settle.samples {
                let (q, v) = (q.get(i).copied().unwrap_or(f32::NAN), v.get(i).copied().unwrap_or(f32::NAN));
                if !q.is_finite() || !v.is_finite() { return Some(format!("joint {} non-finite feedback", i + 1)); }
                lo = lo.min(q); hi = hi.max(q); sum += v; peak = peak.max(v.abs());
                if *t >= recent_from { recent_sum += v; recent_n += 1; }
            }
            let limit = policy.max_velocity_of(i);
            let (span, mean) = (hi - lo, sum / n);
            let recent = recent_sum / recent_n.max(1) as f32;
            if recent.abs() > limit {
                failures.push(format!("joint {} mean velocity over the last {} ms {recent:.4} rad/s (limit {limit})",
                    i + 1, UNLOAD_STILL_RECENT.as_millis()));
            }
            if span > UNLOAD_STILL_POSITION_SPAN_RAD {
                failures.push(format!("joint {} moved {span:.4} rad in {hold:.1} s (limit {UNLOAD_STILL_POSITION_SPAN_RAD})", i + 1));
            }
            if mean.abs() > limit {
                failures.push(format!("joint {} mean velocity {mean:.4} rad/s (limit {limit})", i + 1));
            }
            if peak > limit * UNLOAD_STILL_VELOCITY_PEAK_FACTOR {
                failures.push(format!("joint {} velocity peak {peak:.4} rad/s (limit {})", i + 1, limit * UNLOAD_STILL_VELOCITY_PEAK_FACTOR));
            }
        }
        (!failures.is_empty()).then(|| failures.join("; "))
    }

    pub fn set_torque(&mut self, enable: bool) -> Result<SafetyState, SafetyError> {
        // Torque changes and re-arming are refused in gravity compensation: exit first.
        self.gravity_comp_servo_busy()?;
        if enable {
            if let Some(ref err) = self.driver_config_error {
                return Err(SafetyError::MotionForbiddenInFaultState(format!("Driver limit configuration error: {err}")));
            }
            if self.handover_settling_since.is_some() {
                return Err(SafetyError::HandoverSettling);
            }
            if self.disable_pending { return Err(SafetyError::ClearFaultRejected("Torque disable is still pending".into())); }
            // Atomic e-stop check
            if self.estop_pending() {
                return Err(SafetyError::EStopActive);
            }

            // Strict pre-check: If stopped (E-Stop or Fault), reject IMMEDIATELY before touching driver!
            if self.safety.state().is_stopped() {
                if let SafetyState::Fault(ref reason) = self.safety.state() {
                    return Err(SafetyError::FaultActive(reason.clone()));
                }
                return Err(SafetyError::EStopActive);
            }

            // Enable hardware torque so bus devices wake up and publish state, then verify physical feedback
            let any_driver_enabled = self.driver.as_ref().is_some_and(|d| d.lock().is_ok_and(|drv| drv.has_any_joint_enabled()));
            let was_torqued = self.is_torqued || any_driver_enabled;
            let prev_gripper_cmd = if was_torqued {
                self.config.gripper_index().and_then(|i| self.commanded_positions.get(i).copied())
            } else {
                None
            };

            if let Some(ref d) = self.driver {
                // Secondary double-check latch right before hardware actuation
                if self.estop_pending() {
                    return Err(SafetyError::EStopActive);
                }
                let mut driver = d.lock().map_err(|_| SafetyError::FeedbackUnavailable)?;
                if self.estop_pending() {
                    return Err(SafetyError::EStopActive);
                }
                if !driver.is_feedback_valid() {
                    if let Ok(states) = driver.read_joint_states() {
                        let loads = driver.joint_loads();
                        if !states.is_empty() && driver.is_feedback_valid() && SupervisorInner::states_valid(&states, driver.capabilities(), self.config.dof, loads.as_deref()) {
                            Self::apply_feedback(
                                &mut self.measured_positions,
                                &mut self.measured_velocities,
                                &mut self.torques,
                                &mut self.temperatures,
                                &mut self.loads,
                                self.config.dof,
                                &states,
                                loads.as_deref(),
                            );
                            for (i, s) in states.iter().enumerate().take(self.config.dof) {
                                if s.position.is_finite() {
                                    self.commanded_positions[i] = s.position;
                                    self.target_positions[i] = s.position;
                                }
                            }
                        }
                    }
                }
                if let Err(e) = driver.set_torque(true) {
                    if !was_torqued {
                        let sent = driver.set_torque(false).is_ok();
                        let _ = driver.read_joint_states();
                        let confirmed_off = sent && driver.is_feedback_valid() && driver.enable_state() == EnableState::Disabled;
                        self.is_torqued = !confirmed_off;
                        self.disable_pending = !confirmed_off;
                        self.last_disable_attempt = Some(Instant::now());
                    }
                    let err_msg = format!("Hardware torque enable failed: {}", e);
                    self.safety.trigger_fault_coded(FaultCode::EnableFailed, FaultReason::Other(err_msg.clone()));
                    return Err(SafetyError::ClearFaultRejected(err_msg));
                }
                let states = match driver.read_joint_states() {
                    Ok(s) => s,
                    Err(_) => {
                        if !was_torqued {
                            let sent = driver.set_torque(false).is_ok();
                            let _ = driver.read_joint_states();
                            let confirmed_off = sent && driver.is_feedback_valid() && driver.enable_state() == EnableState::Disabled;
                            self.is_torqued = !confirmed_off;
                            self.disable_pending = !confirmed_off;
                            self.last_disable_attempt = Some(Instant::now());
                        }
                        return Err(SafetyError::FeedbackUnavailable);
                    }
                };
                let loads = driver.joint_loads();
                if !driver.is_feedback_valid() || !Self::states_valid(&states, driver.capabilities(), self.config.dof, loads.as_deref())
                    || (driver.capabilities().enable_feedback && driver.enable_state() != EnableState::Enabled) {
                    if !was_torqued {
                        let sent = driver.set_torque(false).is_ok();
                        let _ = driver.read_joint_states();
                        let confirmed_off = sent && driver.is_feedback_valid() && driver.enable_state() == EnableState::Disabled;
                        self.is_torqued = !confirmed_off;
                        self.disable_pending = !confirmed_off;
                        self.last_disable_attempt = Some(Instant::now());
                    }
                    return Err(SafetyError::FeedbackUnavailable);
                }
                self.feedback_numeric_valid = true;
                self.last_can_feedback_instant = driver.feedback_ages().into_iter().collect::<Option<Vec<_>>>()
                    .and_then(|ages| ages.into_iter().max()).and_then(|age| Instant::now().checked_sub(age));
                Self::apply_feedback(
                    &mut self.measured_positions,
                    &mut self.measured_velocities,
                    &mut self.torques,
                    &mut self.temperatures,
                    &mut self.loads,
                    self.config.dof,
                    &states,
                    loads.as_deref(),
                );
            }

            // Initialize commanded_positions to verified measured_positions to prevent sudden jumps
            self.commanded_positions = self.measured_positions.clone();
            self.target_positions = self.measured_positions.clone();
            // Preserve gripper holding position when re-arming an already torqued arm so grasped objects are not released
            if let Some(g_cmd) = prev_gripper_cmd {
                if let Some(grip_idx) = self.config.gripper_index() {
                    self.commanded_positions[grip_idx] = g_cmd;
                    self.target_positions[grip_idx] = g_cmd;
                }
            }
            self.commanded_velocities = vec![0.0; self.config.dof];
            self.commanded_accelerations.fill(0.0);
            if self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled) {
                let arm_indices = self.config.arm_joint_indices();
                let has_vel_fb = self.driver.as_ref().is_none_or(|d| {
                    d.lock().is_ok_and(|drv| drv.capabilities().velocity_feedback)
                });
                let arm_stationary = !has_vel_fb
                    || arm_indices.iter().all(|&j| {
                        self.measured_velocities
                            .get(j)
                            .is_some_and(|v| v.is_finite() && v.abs() <= 0.05)
                    });
                let aligned_with_issued = !self
                    .tracking
                    .has_residual_lead(&self.commanded_positions, &arm_indices);
                if was_torqued && self.tracking.last_issued_positions.is_some() {
                    if arm_stationary && aligned_with_issued {
                        self.tracking.seed(&self.commanded_positions);
                    } else {
                        // Already torqued and moving or holding active lead: preserve real
                        // issued history so the next tick ramps smoothly instead of jumping.
                        self.tracking.ws_min_depths = None;
                    }
                } else if arm_stationary {
                    self.tracking.seed(&self.commanded_positions);
                } else {
                    self.tracking.invalidate();
                }
            } else {
                self.tracking.invalidate();
            }
            if self.driver.is_none() { self.measured_velocities.fill(0.0); }
            self.active_trajectory = None;
            self.pending_normal_unload = false;
            self.unload_settle = None;
            self.watchdog_park = None;
            self.servo_target = None;
            self.servo_stream_active = false;
            self.servo_ws_min_depths = None;

            let state = self.safety.set_torque(true)?;
            self.is_torqued = true;
            self.pending_rearm = false;
            self.disable_pending = false;
            let now = Instant::now();
            self.last_heartbeat = now;
            self.last_motion_frame = now;
            Ok(state)
        } else {
            self.torque_off(false)
        }
    }

    /// Torque off at the resting pose. `stillness_verified` is set by the unload settle,
    /// which judged stationarity over its hold window; otherwise the current velocity
    /// sample must be within the resting-pose limit.
    fn torque_off(&mut self, stillness_verified: bool) -> Result<SafetyState, SafetyError> {
        if self.safety.state().is_stopped() || self.e_stop_latched.load(Ordering::SeqCst) {
            return Err(SafetyError::MotionForbiddenInFaultState(
                "Cannot disable torque in EStop or Fault. Call clear_fault first.".into(),
            ));
        }

        let violation = if stillness_verified { self.resting_pose_static_violation() } else { self.resting_pose_violation() };
        if let Some(reason) = violation {
            tracing::warn!("Torque off refused, not at resting pose: {reason}");
            return Err(SafetyError::ArmNotAtRestingPose {
                current: self.measured_positions.clone(),
                expected: self.config.resting_positions().to_vec(),
            });
        }

        self.stop_motion();
        self.commanded_velocities.fill(0.0);
        if let Err(error) = self.request_disable() {
            self.safety.trigger_fault_coded(FaultCode::DriverWriteFailed, FaultReason::Other(error.to_string()));
            return Err(error);
        }
        let state = self.safety.set_torque(false)?;
        self.reset_gripper_stall();
        if self.driver.is_none() { self.measured_velocities.fill(0.0); }
        Ok(state)
    }

    /// Automatic unload requires a configured resting pose and observable
    /// velocity, supporting torque and enable state. This is a capability check,
    /// not a claim that the arm is currently resting or ready to disable.
    pub fn supports_automatic_unload(&self) -> bool {
        self.driver.as_ref().is_none_or(|driver| {
            self.config.resting_pose.as_ref().is_some_and(|policy| {
                driver.lock().is_ok_and(|driver| {
                    let capabilities = driver.capabilities();
                    capabilities.velocity_feedback
                        && capabilities.enable_feedback
                        && (capabilities.torque_feedback || policy.support_torque_limits.is_empty())
                        && (capabilities.load_feedback || policy.support_load_limits.is_empty())
                })
            })
        })
    }

    pub fn execute_stop(&mut self, policy: StopPolicy) {
        self.apply_stop_policy(policy);
    }

    pub fn apply_stop_policy(&mut self, policy: StopPolicy) {
        #[cfg(feature = "policy-runtime")]
        match policy {
            StopPolicy::NormalUnload => {
                self.terminate_policy_run("STOPPED", "cancelled", Some("STOPPED"), Some("Arm unloading".into()));
            }
            StopPolicy::EStopLatch => {
                self.terminate_policy_run("ESTOP", "failed", Some("ESTOP_ACTIVE"), Some("E-stop latched".into()));
            }
            StopPolicy::ThermalTrip => {
                self.terminate_policy_run("THERMAL_TRIP", "failed", Some("THERMAL_TRIP"), Some("Thermal trip".into()));
            }
            StopPolicy::BusLossFallback => {
                self.terminate_policy_run("BUS_LOSS", "failed", Some("BUS_LOSS"), Some("CAN feedback lost".into()));
            }
        }
        if policy == StopPolicy::NormalUnload && !self.supports_automatic_unload() {
            tracing::warn!("NormalUnload unsupported: resting pose or confirmed velocity/torque/enable feedback unavailable; no unload motion started");
            return;
        }
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        self.servo_ws_min_depths = None;
        self.servo_stream_active = false;
        // M1: keep output history across NormalUnload so park ramps any residual lead out smoothly
        // instead of dropping active lead in a single tick.
        if policy != StopPolicy::NormalUnload {
            self.tracking.invalidate();
        }
        match policy {
            StopPolicy::NormalUnload => {
                if self.handover_settling_since.is_some() {
                    tracing::warn!("NormalUnload rejected while handover settling; maintaining holding torque");
                    return;
                }
                // If in EStop or Fault: strictly forbid planning motion, maintain holding torque if torqued!
                if self.safety.state().is_stopped() || self.e_stop_latched.load(Ordering::SeqCst) {
                    tracing::warn!("NormalUnload rejected in EStop or Fault state; maintaining holding torque");
                    if self.is_torqued {
                        self.pending_normal_unload = false;
                        self.unload_settle = None;
                        self.active_trajectory = None;
                        self.servo_target = None;
                    }
                    return;
                }

                // Return to home position cleanly, then disable torque.
                // Allow unload from Operational or Idle.
                let can_unload = self.is_torqued && !self.pending_rearm && (
                    self.safety.state().is_operational()
                    || matches!(self.safety.state(), SafetyState::Idle)
                );
                if can_unload {
                    let home = self.config.resting_positions().to_vec();
                    let clamped_start: Vec<f32> = self
                        .commanded_positions
                        .iter()
                        .enumerate()
                        .map(|(i, &p)| {
                            if i < self.config.limits.len() {
                                let lim = &self.config.limits[i];
                                p.clamp(lim.min_position, lim.max_position)
                            } else {
                                p
                            }
                        })
                        .collect();
                    if let Ok(traj) = TrajectoryPlanner::plan(
                        &clamped_start,
                        &self.commanded_velocities,
                        &home,
                        &vec![0.0; self.config.dof],
                        Some(1.5),
                        &self.planning_limits(),
                    ) {
                        // power budget: slowed to fit, never refused
                        let traj = match self.power_fit_quintic(traj.clone(), &clamped_start, &self.commanded_velocities, &home, &self.planning_limits(), false) {
                            Ok((t, _)) => t,
                            Err(e) => { tracing::warn!("Park power planning failed ({e}); parking as planned"); traj }
                        };
                        // Park is exempt from the floor / box / reach limits (it must remain
                        // possible from a violating pose) but not from the keep-out boxes.
                        let samples = match self.keepout_samples(&traj) {
                            Ok(s) => s,
                            Err(e) => {
                                let e = e.with_workspace_phase("park");
                                tracing::warn!("Park path cannot be verified against the keep-out boxes ({e}); not parking");
                                self.park_rejection = Some(e);
                                self.pending_normal_unload = false;
                                self.unload_settle = None;
                                self.active_trajectory = None;
                                return;
                            }
                        };
                        if let Err(e) = self.check_keepout_path(&clamped_start, samples.iter().map(Vec::as_slice)) {
                            let e = e.with_workspace_phase("park");
                            tracing::warn!("Park path enters a keep-out box ({e}); not parking");
                            self.park_rejection = Some(e);
                            self.pending_normal_unload = false;
                            self.unload_settle = None;
                            self.active_trajectory = None;
                            return;
                        }
                        if let Err(e) = self.check_workspace_path(&clamped_start, samples.iter().map(Vec::as_slice)) {
                            tracing::warn!("Park path leaves the workspace ({e}); parking anyway");
                        }
                        let _ = self.safety.set_torque(true);
                        let _ = self.safety.mark_motion_started();
                        self.unload_planned_duration = Some(traj.duration);
                        self.active_trajectory = Some(ActiveTrajectory::quintic(traj, None));
                        self.target_positions = home;
                        self.pending_normal_unload = true;
                        self.unload_settle = None;
                        self.park_gripper_contact = false;
                        self.park_release = None;
                        let now = Instant::now();
                        self.last_heartbeat = now;
                        self.last_motion_frame = now;
                        return;
                    }
                }
                // If the arm is torqued, maintain holding torque if motion cannot be planned or arm is not operational!
                // NEVER cut power under gravity when motion planning fails or arm is in stopped/idle state!
                if self.is_torqued {
                    tracing::warn!("NormalUnload cannot plan motion or arm not operational; maintaining holding torque to prevent dropping arm");
                    self.pending_normal_unload = false;
                    self.unload_settle = None;
                    self.active_trajectory = None;
                    self.servo_target = None;
                    return;
                }
                // If not torqued, ensure state is clean
                self.pending_normal_unload = false;
                self.unload_settle = None;
                self.is_torqued = false;
                self.tracking.invalidate();
                self.active_trajectory = None;
                self.reset_gripper_stall();
                self.commanded_velocities = vec![0.0; self.config.dof];
                if self.driver.is_none() {
                    self.measured_velocities = vec![0.0; self.config.dof];
                }
                let _ = self.safety.set_torque(false);
                if let Some(ref d) = self.driver {
                    if let Ok(mut driver) = d.lock() {
                        let _ = driver.set_torque(false);
                    }
                }
            }
            StopPolicy::EStopLatch => {
                self.e_stop_latched.store(true, Ordering::SeqCst);
                self.exit_gravity_comp("ESTOP", "E-stop");
                // Immediate rigid lock at current position, latch E-Stop, advance epoch
                // Maintain active restorative torque hold: do NOT cut power or set is_torqued=false
                self.pending_normal_unload = false;
                self.unload_settle = None;
                self.active_trajectory = None;
                self.servo_target = None;
                let hold_pos = if self.measured_positions.iter().all(|p| p.is_finite())
                    && self.measured_positions.len() == self.config.dof
                {
                    self.measured_positions.clone()
                } else {
                    self.commanded_positions.clone()
                };
                self.target_positions = hold_pos.clone();
                self.commanded_positions = hold_pos;
                self.commanded_velocities = vec![0.0; self.config.dof];
                if self.driver.is_none() {
                    self.measured_velocities = vec![0.0; self.config.dof];
                }
                self.safety.trigger_e_stop();
            }
            StopPolicy::ThermalTrip => {
                self.stop_motion();
                self.commanded_velocities.fill(0.0);
                self.commanded_accelerations.fill(0.0);
                let (joint_idx, temperature) = self.temperatures.iter().copied().enumerate()
                    .filter(|(_, t)| t.is_finite()).max_by(|a,b| a.1.total_cmp(&b.1)).unwrap_or((0, 0.0));
                if !matches!(self.safety.state(), SafetyState::Fault(FaultReason::ThermalTrip { .. })) {
                    self.safety.trigger_fault(FaultReason::ThermalTrip { joint_idx, temperature, limit: self.config.max_temperature });
                }
                if let Err(e) = self.request_disable() { tracing::error!("Thermal shutdown not confirmed: {e}"); }
            }
            StopPolicy::BusLossFallback => {
                self.exit_gravity_comp("BUS_LOSS", "CAN feedback lost");
                // Bus loss / disconnect: hold position locked, latch fault, advance epoch
                // Maintain holding torque: DO NOT disable hardware torque to prevent gravitational collapse!
                self.pending_normal_unload = false;
                self.unload_settle = None;
                self.active_trajectory = None;
                self.servo_target = None;
                let hold_pos = if self.measured_positions.iter().all(|p| p.is_finite())
                    && self.measured_positions.len() == self.config.dof
                {
                    self.measured_positions.clone()
                } else {
                    self.commanded_positions.clone()
                };
                self.target_positions = hold_pos.clone();
                self.commanded_positions = hold_pos;
                self.commanded_velocities = vec![0.0; self.config.dof];
                if self.driver.is_none() {
                    self.measured_velocities = vec![0.0; self.config.dof];
                }
                self.bus_loss_instant = Some(Instant::now());
                self.safety.trigger_fault(FaultReason::BusLoss);
            }
        }
    }

    /// In Fault / EStop with torque on: fresh feedback from every motor reporting Disabled
    /// (Damiao: all fresh 0xCC / feedback states 0) means the motors lost torque, e.g. a
    /// motor power reset during a bus loss (real arm, 2026-11-12: all seven motors came
    /// back Disabled after BusLoss and armd kept `is_torqued: true`). armd then records
    /// torque off (`is_torqued` false, re-arm needs the full enable handshake) and keeps the
    /// Fault until `clear_fault`. Not applied during gravity compensation (its exit path
    /// owns the motor modes) or while a disable is pending.
    fn sync_torque_state_after_stop(&mut self) {
        if !self.is_torqued || self.disable_pending || self.gravity_comp.is_some() || !self.safety.state().is_stopped() {
            return;
        }
        let Some(d) = self.driver.as_ref() else { return; };
        let off = d.lock().is_ok_and(|drv| drv.capabilities().enable_feedback && drv.is_feedback_valid()
            && drv.enable_state() == EnableState::Disabled);
        if !off || !self.hardware_feedback_fresh(Duration::from_millis(250)) { return; }
        if let Ok(mut drv) = d.lock() { drv.set_torque_flag(false); }
        tracing::warn!("Motors report Disabled with fresh feedback in {:?}: torque is off (is_torqued = false); the fault stays latched until clear_fault",
            self.safety.state());
        self.is_torqued = false;
        self.pending_rearm = true;
        self.output_suspended = false;
        self.gripper_stalled = false;
        self.gripper_stall_sign = 0.0;
        self.gripper_prev_hw_overload = false;
        self.stop_motion();
        self.servo_stream_active = false;
        self.tracking.invalidate();
        if self.measured_positions.iter().all(|q| q.is_finite()) {
            self.commanded_positions = self.measured_positions.clone();
            self.target_positions = self.measured_positions.clone();
        }
        self.commanded_velocities.fill(0.0);
        self.commanded_accelerations.fill(0.0);
    }

    pub(crate) fn consume_pending_estop(&mut self) -> bool {
        let req = self.requested_estop_count.load(Ordering::SeqCst);
        if req > self.applied_estop_count {
            self.applied_estop_count = req;
            self.e_stop_latched.store(true, Ordering::SeqCst);
            if !matches!(self.safety.state(), SafetyState::Fault(_)) {
                self.apply_stop_policy(StopPolicy::EStopLatch);
            }
            true
        } else {
            false
        }
    }

    pub(crate) fn check_concurrent_estop(inner: &mut SupervisorInner) -> Result<(), SafetyError> {
        if inner.consume_pending_estop() {
            return Err(SafetyError::EStopActive);
        }
        Ok(())
    }

    pub fn clear_fault(&mut self) -> Result<SafetyState, SafetyError> {
        Self::check_concurrent_estop(self)?;
        if self.driver_config_error.is_some() {
            self.reapply_driver_limits();
        }
        if let Some(ref err) = self.driver_config_error {
            return Err(SafetyError::ClearFaultRejected(format!("Driver limit configuration error: {err}")));
        }
        if !self.safety.state().is_stopped() { return Ok(self.safety.state().clone()); }
        if self.gravity_comp_blocks_clear() {
            return Err(SafetyError::ClearFaultRejected("Gravity compensation: the switch back to POS_VEL is not confirmed yet".into()));
        }
        if self.disable_pending { return Err(SafetyError::ClearFaultRejected("Torque disable is still unconfirmed".into())); }
        let thermal = matches!(self.safety.state(), SafetyState::Fault(FaultReason::ThermalTrip { .. }));
        if let Some(ref d) = self.driver {
            let mut drv = d.lock().map_err(|_| SafetyError::FeedbackUnavailable)?;
            let disabling = drv.clear_fault_disables_joints();
            if (self.is_torqued || drv.has_any_joint_enabled() || drv.enable_state() != EnableState::Disabled)
                && disabling.iter().any(|&j| j < self.config.arm_dof())
            {
                let states = drv.read_joint_states().map_err(|e| SafetyError::ClearFaultRejected(e.to_string()))?;
                let ages = drv.feedback_ages();
                let loads = drv.joint_loads();
                let max_age = self.resting_policy().map(|p| Duration::from_millis(p.feedback_max_age_ms)).unwrap_or(Duration::from_millis(250));
                if !drv.is_feedback_valid()
                    || ages.len() != self.config.dof
                    || !ages.iter().all(|a| a.is_some_and(|t| t <= max_age))
                    || !Self::states_valid(&states, drv.capabilities(), self.config.dof, loads.as_deref())
                {
                    return Err(SafetyError::ClearFaultRejected("Fresh feedback is required before checking resting pose support".into()));
                }
                self.feedback_numeric_valid = true;
                self.last_can_feedback_instant = ages.into_iter().collect::<Option<Vec<_>>>()
                    .and_then(|a| a.into_iter().max()).and_then(|age| Instant::now().checked_sub(age));
                Self::apply_feedback(
                    &mut self.measured_positions,
                    &mut self.measured_velocities,
                    &mut self.torques,
                    &mut self.temperatures,
                    &mut self.loads,
                    self.config.dof,
                    &states,
                    loads.as_deref(),
                );
                if let Some(reason) = self.resting_pose_support_violation_with_driver(Some(drv.as_ref())) {
                    return Err(SafetyError::ClearFaultRejected(format!(
                        "Cannot disable faulted supporting arm joint away from resting pose ({reason}); support the arm at rest first"
                    )));
                }
            }
            // Clearing a latched motor error requires a later feedback observation, not a cleared cache.
            if drv.check_hardware_status().is_err() {
                drv.clear_fault().map_err(|e| SafetyError::ClearFaultRejected(e.to_string()))?;
                return Err(SafetyError::ClearFaultRejected("Hardware reset sent; wait for fresh fault-free feedback before retrying".into()));
            }
            let states = drv.read_joint_states().map_err(|e| SafetyError::ClearFaultRejected(e.to_string()))?;
            let ages = drv.feedback_ages();
            let loads = drv.joint_loads();
            if !drv.is_feedback_valid() || ages.len() != self.config.dof
                || ages.iter().any(|a| a.is_none_or(|t| t > Duration::from_millis(250)))
                || !Self::states_valid(&states, drv.capabilities(), self.config.dof, loads.as_deref()) {
                return Err(SafetyError::ClearFaultRejected("Fresh feedback is required before resetting a fault".into()));
            }
            drv.check_hardware_status().map_err(|e| SafetyError::ClearFaultRejected(e.to_string()))?;
            if thermal && (!drv.capabilities().temperature_feedback || states.iter().any(|state|
                !state.temperature.is_finite() || state.temperature > self.config.max_temperature - THERMAL_CLEAR_MARGIN_C)) {
                return Err(SafetyError::ClearFaultRejected("Temperature must be confirmed below the reset threshold".into()));
            }
            let mut state = drv.enable_state();
            let mut partial = drv.has_known_partial_enable();
            let mut any_enabled = drv.has_any_joint_enabled();
            if state == EnableState::Unknown && !partial {
                let _ = drv.refresh_enable_state();
                state = drv.enable_state();
                partial = drv.has_known_partial_enable();
                any_enabled = drv.has_any_joint_enabled();
            }
            if state == EnableState::Unknown && !partial {
                return Err(SafetyError::ClearFaultRejected("Hardware enable state is unknown; reset remains latched".into()));
            }
            self.feedback_numeric_valid = true;
            self.last_can_feedback_instant = ages.into_iter().collect::<Option<Vec<_>>>()
                .and_then(|a| a.into_iter().max()).and_then(|age| Instant::now().checked_sub(age));
            Self::apply_feedback(
                &mut self.measured_positions,
                &mut self.measured_velocities,
                &mut self.torques,
                &mut self.temperatures,
                &mut self.loads,
                self.config.dof,
                &states,
                loads.as_deref(),
            );
            self.is_torqued = state == EnableState::Enabled || partial || any_enabled;
        } else if thermal && self.temperatures.iter().any(|t| !t.is_finite() || *t > self.config.max_temperature - THERMAL_CLEAR_MARGIN_C) {
            return Err(SafetyError::ClearFaultRejected("Temperature is above the reset threshold".into()));
        }
        self.pending_rearm = true;
        self.stop_motion();
        self.servo_stream_active = false;
        self.tracking.invalidate();
        self.tracking.clear_recovery_suspension();
        // Reset at the measured position; never retain a residual old trajectory target.
        self.commanded_positions = self.measured_positions.clone();
        self.target_positions = self.measured_positions.clone();
        self.commanded_velocities.fill(0.0);
        self.last_commanded_velocities.fill(0.0);
        self.commanded_accelerations.fill(0.0);
        self.following_error_since = None;
        self.bus_loss_instant = None;
        self.reset_gripper_stall();
        Self::check_concurrent_estop(self)?;
        let was_estop = *self.safety.state() == SafetyState::EStop;
        let state = self.safety.clear_fault()?;
        self.handover_settling_since = None;
        if !was_estop && self.e_stop_latched.load(Ordering::SeqCst) {
            self.apply_stop_policy(StopPolicy::EStopLatch);
            self.last_heartbeat = Instant::now();
            self.last_motion_frame = Instant::now();
            return Ok(self.safety.state().clone());
        }
        self.e_stop_latched.store(false, Ordering::SeqCst);
        Self::check_concurrent_estop(self)?;
        self.last_heartbeat = Instant::now();
        self.last_motion_frame = Instant::now();
        Ok(state)
    }

    pub fn shutdown_ready(&self) -> bool {
        if self.disable_pending { return false; }
        match &self.driver {
            None => !self.is_torqued,
            Some(driver) => !self.is_torqued
                && driver.lock().is_ok_and(|drv| {
                    if !drv.capabilities().enable_feedback {
                        true
                    } else {
                        drv.enable_state() == EnableState::Disabled
                    }
                }),
        }
    }

    pub fn is_fully_settled_hold(&self) -> bool {
        if self.gravity_comp.is_some() { return false; }
        // Safe condition 1: Confirmed disabled (torque off, Idle, not disable pending)
        if !self.is_torqued && matches!(self.safety.state(), SafetyState::Idle) && !self.disable_pending {
            return true;
        }

        let (has_velocity_capability, is_valid) = match &self.driver {
            None => (true, true),
            Some(driver) => driver.lock().map_or((false, false), |drv| (drv.capabilities().velocity_feedback, drv.is_feedback_valid())),
        };

        if !is_valid {
            return false;
        }

        let velocity_settled = if has_velocity_capability {
            self.measured_velocities.len() == self.config.dof
                && self.measured_velocities.iter().all(|v| v.is_finite() && v.abs() <= 0.05)
        } else {
            true
        };

        let tracking_settled = !self.config.joint_tracking.as_ref().is_some_and(|jt| {
            jt.enabled
                && self
                    .tracking
                    .has_residual_lead(&self.commanded_positions, &self.config.arm_joint_indices())
        });

        (self.safety.state() == &SafetyState::Armed || self.safety.state() == &SafetyState::Idle)
            && self.active_trajectory.is_none()
            && self.servo_target.is_none()
            && self.commanded_velocities.len() == self.config.dof
            && self.commanded_velocities.iter().all(|v| *v == 0.0)
            && self.hardware_feedback_fresh(Duration::from_millis(100))
            && velocity_settled
            && tracking_settled
    }

    pub fn is_fully_settled_armed_hold(&self) -> bool {
        self.is_fully_settled_hold()
    }

    pub fn preempt_lease(&mut self, conn: &str, label: &str) -> Result<(u64, Option<(String, u64)>), PreemptError> {
        let now = Instant::now();
        if let Some(ref active) = self.active_lease {
            let is_expired = active.acquired_at.elapsed() >= self.lease_timeout;
            if active.client_id == conn && !is_expired {
                return Ok((active.generation, None));
            }

            if active.client_id != conn {
                if let Some(last) = self.last_preemption {
                    let elapsed = last.elapsed();
                    if elapsed < PREEMPT_COOLDOWN {
                        let remaining_ms = (PREEMPT_COOLDOWN.saturating_sub(elapsed).as_millis() as u64).max(1);
                        return Err(PreemptError::Cooldown(remaining_ms));
                    }
                }
            }
        }

        self.cancel_watchdog_park_for_new_lease(conn);

        #[cfg(feature = "policy-runtime")]
        self.terminate_policy_run("LEASE_LOST", "cancelled", Some("LEASE_LOST"), Some("Control lease preempted".into()));

        // Four-category state categorization (strictly fail-closed, no unwrap/panic, driver lock poison returns false)
        let confirmed_hardware_disabled = match &self.driver {
            None => !self.is_torqued && !self.disable_pending && self.safety.state() == &SafetyState::Idle,
            Some(driver) => !self.is_torqued
                && !self.disable_pending
                && self.safety.state() == &SafetyState::Idle
                && driver.lock().is_ok_and(|drv| {
                    if !drv.capabilities().enable_feedback {
                        true
                    } else {
                        drv.enable_state() == EnableState::Disabled
                    }
                }),
        };

        if confirmed_hardware_disabled
            || self.safety.state().is_stopped()
            || self.is_fully_settled_armed_hold()
        {
            self.handover_settling_since = None;
        } else {
            self.stop_motion();
            self.handover_settling_since = Some(now);
        }

        let evicted = self.active_lease.as_ref().filter(|a| a.client_id != conn).map(|a| (a.client_id.clone(), a.generation));

        self.lease_generation_counter += 1;
        let gen = self.lease_generation_counter;
        self.active_lease = Some(ActiveLease {
            client_id: conn.to_string(),
            generation: gen,
            label: label.to_string(),
            acquired_at: now,
            last_valid_motion_instant: None,
        });
        self.last_heartbeat = now;
        if evicted.is_some() {
            self.last_preemption = Some(now);
            self.last_evicted = evicted.clone();
        }

        tracing::info!("Preempted lease for connection {} (label {}), generation {}, evicted: {:?}", conn, label, gen, evicted);
        Ok((gen, evicted))
    }

    pub fn acquire_lease(&mut self, client_id: &str) -> Result<u64, (String, u64)> {
        let now = Instant::now();
        if let Some(ref active) = self.active_lease {
            let is_expired = active.acquired_at.elapsed() >= self.lease_timeout;
            if !is_expired {
                if active.client_id == client_id { return Ok(active.generation); }
                return Err((active.client_id.clone(), active.generation));
            }
        }
        // A new controller never inherits an expired controller's queued motion,
        // nor a watchdog park started after the previous lease was released.
        self.cancel_watchdog_park_for_new_lease(client_id);
        #[cfg(feature = "policy-runtime")]
        self.terminate_policy_run("LEASE_LOST", "cancelled", Some("LEASE_LOST"), Some("Control lease acquired by new controller".into()));
        if self.active_lease.is_some() { self.stop_motion(); }
        self.lease_generation_counter += 1;
        let gen = self.lease_generation_counter;
        self.active_lease = Some(ActiveLease {
            client_id: client_id.to_string(),
            generation: gen,
            label: String::new(),
            acquired_at: now,
            last_valid_motion_instant: None,
        });
        Ok(gen)
    }

    /// A new lease owner takes over from a running watchdog park: the park's
    /// motion is cancelled like any other queued motion and the arm brakes to a hold.
    fn cancel_watchdog_park_for_new_lease(&mut self, client_id: &str) {
        if self.watchdog_park.take().is_some() && self.pending_normal_unload {
            tracing::info!("Watchdog park cancelled: new lease acquired by {client_id}; holding position");
            self.stop_motion();
        }
    }

    /// Budget for a watchdog park, in control-loop time: deceleration of the physical arm
    /// from its measured velocity (max over joints of |v| / planning acceleration), the
    /// planned park duration (which already starts from the commanded velocity), the
    /// settle hold and [`WATCHDOG_PARK_MARGIN`]; never less than [`WATCHDOG_PARK_TIMEOUT`].
    pub fn watchdog_park_timeout(&self) -> Duration {
        let decel = self.measured_velocities.iter().zip(self.planning_limits())
            .filter(|(v, l)| v.is_finite() && l.max_acceleration > 1e-6)
            .map(|(v, l)| f64::from(v.abs() / l.max_acceleration))
            .fold(0.0, f64::max);
        let planned = self.unload_planned_duration
            .filter(|d| d.is_finite() && *d >= 0.0)
            .map_or(Duration::ZERO, |d| Duration::from_secs_f64((decel + d).min(3600.0)) + UNLOAD_SETTLE_HOLD + WATCHDOG_PARK_MARGIN);
        planned.max(WATCHDOG_PARK_TIMEOUT)
    }

    /// Advance a running watchdog park by `dt`. When the budget runs out with the arm at the
    /// resting pose and only the stationary check pending, the budget is extended once by
    /// [`WATCHDOG_PARK_SETTLE_GRACE`]; otherwise torque is held and WatchdogTimeout latched.
    fn supervise_watchdog_park(&mut self, dt: f64) {
        let Some(mut park) = self.watchdog_park.take() else { return; };
        if !self.pending_normal_unload {
            tracing::warn!("Watchdog park interrupted (safety state {:?})", self.safety.state());
            if self.is_torqued && !self.safety.state().is_stopped() {
                self.latch_watchdog_fault();
            }
            return;
        }
        // budget in reference time: the power governor may slow the park
        if dt.is_finite() && dt > 0.0 { park.elapsed_s += dt * self.motion_time_scale(); }
        if park.elapsed_s < park.budget_s {
            self.watchdog_park = Some(park);
            return;
        }
        let blocker = self.unload_settle_blocker().unwrap_or_else(|| "settle pending".into());
        let at_pose = self.active_trajectory.is_none() && self.resting_pose_static_violation().is_none();
        if at_pose && !park.grace_granted {
            tracing::warn!("Watchdog park at the resting pose after {:.2} s but not settled ({blocker}); waiting up to {:?} more",
                park.elapsed_s, WATCHDOG_PARK_SETTLE_GRACE);
            park.budget_s += WATCHDOG_PARK_SETTLE_GRACE.as_secs_f64();
            park.grace_granted = true;
            self.watchdog_park = Some(park);
            return;
        }
        tracing::warn!("Watchdog park did not finish within {:.2} s ({blocker}); holding torque and latching WatchdogTimeout", park.budget_s);
        self.latch_watchdog_fault();
    }

    /// Conditions for parking automatically when the lease watchdog expires.
    /// Returns the reason the park is not allowed, or `None` when it is.
    fn watchdog_park_blocker(&self) -> Option<String> {
        let Some(policy) = &self.config.resting_pose else { return Some("no resting pose configured".into()); };
        if !self.supports_automatic_unload() { return Some("driver lacks velocity/torque/enable feedback".into()); }
        if self.e_stop_latched.load(Ordering::SeqCst) { return Some("E-stop latched".into()); }
        if self.collision.hold.is_some() { return Some("collision hold".into()); }
        if let Some(h) = self.torque_guard.hold {
            let rest = self.config.resting_positions().get(h.joint).copied().unwrap_or(h.hold_q);
            if (rest - h.anchor_q) * h.sign > crate::torque_guard::TORQUE_PUSH_IN_RAD {
                return Some(format!("torque hold on joint {}: the resting pose lies in the push direction", h.joint + 1));
            }
        }
        if !self.safety.state().is_operational() { return Some(format!("safety state {:?}", self.safety.state())); }
        if self.pending_rearm || self.disable_pending || self.output_suspended || self.handover_settling_since.is_some() {
            return Some("re-arm, disable, output suspension or handover pending".into());
        }
        if !self.hardware_feedback_fresh(Duration::from_millis(policy.feedback_max_age_ms)) {
            return Some(format!("feedback older than {} ms", policy.feedback_max_age_ms));
        }
        None
    }

    /// Previous watchdog behaviour: rigid hold at the measured (or commanded) position
    /// and latch Fault(WatchdogTimeout). Torque stays on.
    pub(crate) fn latch_watchdog_fault(&mut self) {
        #[cfg(feature = "policy-runtime")]
        self.terminate_policy_run("WATCHDOG_TIMEOUT", "failed", Some("WATCHDOG_TIMEOUT"), Some("Lease watchdog expired".into()));
        let mut hold_pos = if self.measured_positions.iter().all(|p| p.is_finite())
            && self.measured_positions.len() == self.config.dof
        {
            self.measured_positions.clone()
        } else {
            self.commanded_positions.clone()
        };
        // a torque hold keeps its backed-off command (src/torque_guard.rs)
        if let Some(h) = self.torque_guard.hold {
            if let (Some(p), Some(c)) = (hold_pos.get_mut(h.joint), self.commanded_positions.get(h.joint)) { *p = *c; }
        }
        self.watchdog_park = None;
        self.pending_normal_unload = false;
        self.unload_settle = None;
        self.active_trajectory = None;
        self.servo_target = None; self.servo_settled_ticks.fill(0);
        self.servo_ws_min_depths = None;
        self.servo_stream_active = false;
        self.tracking.invalidate();
        self.target_positions = hold_pos.clone();
        self.commanded_positions = hold_pos;
        self.commanded_velocities = vec![0.0; self.config.dof];
        self.safety.trigger_fault(FaultReason::WatchdogTimeout);
    }

    /// Drops the lease if `client_id`/`generation` own it. Returns true only when the
    /// lease was still valid: callers stop motion on true. An expired lease is dropped
    /// without that, because its expiry may already have started a watchdog park that a
    /// client without a valid lease must not cancel.
    pub fn release_lease(&mut self, client_id: &str, generation: u64) -> bool {
        if let Some(ref active) = self.active_lease {
            if active.client_id == client_id && active.generation == generation {
                let valid = active.acquired_at.elapsed() < self.lease_timeout;
                self.active_lease = None;
                #[cfg(feature = "policy-runtime")]
                self.terminate_policy_run("LEASE_LOST", "cancelled", Some("LEASE_LOST"), Some("Control lease released".into()));
                return valid;
            }
        }
        false
    }

    /// Records the label a lease holder sent with `acquire_lease` (shown to other clients).
    pub fn set_lease_label(&mut self, client_id: &str, generation: u64, label: &str) {
        if let Some(active) = self.active_lease.as_mut().filter(|a| a.client_id == client_id && a.generation == generation) {
            active.label = label.to_string();
        }
    }

    /// Label of the valid lease holder (empty string when it sent none); None when no lease is held.
    pub fn lease_holder(&self) -> Option<String> {
        self.active_lease.as_ref().filter(|a| a.acquired_at.elapsed() < self.lease_timeout).map(|a| a.label.clone())
    }

    /// Why `client_id` no longer holds `generation`: "preempted" when a forced acquire
    /// evicted exactly that lease, otherwise "expired" (heartbeats stopped arriving).
    pub fn lease_loss_reason(&self, client_id: &str, generation: u64) -> &'static str {
        match &self.last_evicted {
            Some((conn, gen)) if conn == client_id && *gen == generation => "preempted",
            _ => "expired",
        }
    }

    pub fn is_lease_active(&self) -> bool {
        if let Some(ref active) = self.active_lease {
            active.acquired_at.elapsed() < self.lease_timeout
        } else {
            false
        }
    }

    pub fn validate_servo_lease(&mut self, client_id: &str, generation: u64) -> bool {
        if let Some(ref mut active) = self.active_lease {
            let not_expired = active.acquired_at.elapsed() < self.lease_timeout;
            if not_expired && active.generation == generation && !client_id.is_empty() && active.client_id == client_id {
                active.last_valid_motion_instant = Some(Instant::now());
                true
            } else {
                false
            }
        } else {
            false
        }
    }

    pub fn heartbeat(&mut self, client_id: &str, epoch_id: u64) -> Result<(), SafetyError> {
        if epoch_id != self.safety.epoch_id() {
            return Err(SafetyError::EpochMismatch {
                expected: self.safety.epoch_id(),
                received: epoch_id,
            });
        }
        if let Some(ref mut active) = self.active_lease {
            if active.client_id == client_id && active.acquired_at.elapsed() < self.lease_timeout {
                active.acquired_at = Instant::now();
                self.last_heartbeat = Instant::now();
            } else {
                tracing::debug!(
                    "Heartbeat ignored from non-lease owner '{}' (lease owner is '{}')",
                    client_id, active.client_id
                );
            }
        }
        // NOTE: When there is no active lease, do NOT refresh last_heartbeat.
        // Watchdog can only be refreshed by the authenticated active lease owner.
        Ok(())
    }

    fn validate_and_filter_servo_targets(
        &mut self,
        targets: &mut [f32],
        epoch_id: u64,
    ) -> Result<(), SafetyError> {
        self.gravity_comp_servo_busy()?;
        self.collision_hold_check()?;
        if self.handover_settling_since.is_some() {
            return Err(SafetyError::HandoverSettling);
        }

        if self.estop_pending() {
            return Err(SafetyError::EStopActive);
        }

        if epoch_id != self.safety.epoch_id() {
            return Err(SafetyError::EpochMismatch {
                expected: self.safety.epoch_id(),
                received: epoch_id,
            });
        }

        if self.pending_rearm || self.pending_normal_unload {
            return Err(SafetyError::NotArmed(self.safety.state().clone()));
        }
        if self.active_trajectory.is_some() || self.macro_run.is_some() {
            return Err(SafetyError::MotionBusy);
        }

        // 1. Validate motion command against epoch, safety state, and soft limits
        if let Err(e) = self.safety.validate_motion_command(Some(epoch_id), targets, &self.config.limits) {
            if let SafetyError::SoftLimitExceeded { joint, .. } = &e {
                let idx = self.config.limits.iter().position(|l| &l.name == joint);
                self.note_motion_limit(MotionLimitReason::JointLimit, idx);
            }
            return Err(e);
        }

        // 1b. Cartesian workspace: the target may not leave the box further than now.
        if let Err(e) = self.check_workspace_path(&self.commanded_positions, [&*targets]) {
            let e = e.with_workspace_phase("servo_target");
            self.workspace_violation = e.workspace_detail();
            self.note_motion_limit(MotionLimitReason::Workspace, None);
            return Err(e);
        }
        self.torque_hold_filter(targets);
        self.power_hold_filter(targets);
        Ok(())
    }

    /// Servo / policy intake of the gripper target: records it as the stream's gripper target and
    /// releases the stall latch on a target away from the stall. A torque-cap latch needs the
    /// target [`GRIPPER_FORCE_RELEASE_MARGIN_RAD`] more open than the hold.
    fn release_gripper_stall_on_reverse(&mut self, targets: &[f32]) {
        if let Some(g_idx) = self.config.gripper_index() {
            self.gripper_stream_target = targets.get(g_idx).copied().filter(|t| t.is_finite());
            if self.gripper_stalled && targets.len() > g_idx && self.commanded_positions.len() > g_idx {
                let delta = targets[g_idx] - self.commanded_positions[g_idx];
                let tau_g = self.torques.get(g_idx).copied().unwrap_or(0.0);
                let reversed = if self.gripper_force_capped {
                    delta * self.gripper_stall_sign <= -GRIPPER_FORCE_RELEASE_MARGIN_RAD
                } else {
                    (delta * tau_g < -1e-4) || (delta * self.gripper_stall_sign < -1e-4)
                };
                if reversed {
                    self.gripper_stalled = false;
                    self.gripper_high_torque_ticks = 0;
                    self.gripper_low_torque_ticks = 0;
                }
            }
        }
    }

    pub fn servo(&mut self, mut targets: Vec<f32>, epoch_id: u64) -> Result<(), SafetyError> {
        #[cfg(feature = "policy-runtime")]
        if self.policy_run.is_some() || self.policy_loading {
            return Err(SafetyError::MotionBusy);
        }
        self.validate_and_filter_servo_targets(&mut targets, epoch_id)?;
        if self.servo_blocked_hold { self.clear_blocked_hold(); }

        // Only renew lease and motion frame timestamp AFTER command validation succeeds
        self.last_heartbeat = Instant::now();
        self.last_motion_frame = Instant::now();
        if let Some(ref mut l) = self.active_lease {
            l.last_valid_motion_instant = Some(Instant::now());
        }

        // Gripper stall release check on reverse command
        self.release_gripper_stall_on_reverse(&targets);

        let _ = self.safety.mark_motion_started();

        // 2. Store servo target for tick(dt) to step with actual dt and velocity limits
        self.target_positions = targets.clone();
        self.servo_target = Some(targets);
        self.servo_stream_active = true;
        self.servo_settled_ticks.fill(0);

        Ok(())
    }

    pub fn jog(
        &mut self,
        target_positions: Vec<f32>,
        duration_secs: Option<f64>,
        epoch_id: Option<u64>,
    ) -> Result<u64, ArmError> {
        self.jog_action(target_positions, duration_secs, epoch_id, "joint_move").map(|(epoch, _, _)| epoch)
    }

    /// Jog that records an action of `kind` (`joint_move` or `home`).
    /// Returns (epoch, action id, planned duration).
    pub fn jog_action(
        &mut self,
        target_positions: Vec<f32>,
        duration_secs: Option<f64>,
        epoch_id: Option<u64>,
        kind: &str,
    ) -> Result<(u64, u64, f64), ArmError> {
        let limits = self.planning_limits();
        self.start_quintic(target_positions, duration_secs, epoch_id, kind, None, &limits)
    }

    /// Joint-space quintic to `target_positions` planned with `limits`, recorded as an
    /// action of `kind`. Returns (epoch, action id, planned duration).
    pub(crate) fn start_quintic(
        &mut self,
        target_positions: Vec<f32>,
        duration_secs: Option<f64>,
        epoch_id: Option<u64>,
        kind: &str,
        tag: Option<String>,
        limits: &[crate::model::JointLimit],
    ) -> Result<(u64, u64, f64), ArmError> {
        let mut target_positions = target_positions;
        self.gravity_comp_motion_busy()?;
        #[cfg(feature = "policy-runtime")]
        if self.policy_loading {
            return Err(MotionError::MotionBusy("a policy is loading".into()).into());
        }
        #[cfg(feature = "policy-runtime")]
        if self.policy_run.is_some() {
            return Err(MotionError::MotionBusy("a policy run is active".into()).into());
        }
        self.collision_hold_check()?;
        if self.handover_settling_since.is_some() {
            return Err(ArmError::Safety(SafetyError::HandoverSettling));
        }

        if self.e_stop_latched.load(Ordering::SeqCst) {
            return Err(ArmError::Safety(SafetyError::EStopActive));
        }

        if self.pending_rearm || self.pending_normal_unload {
            return Err(ArmError::Safety(SafetyError::NotArmed(self.safety.state().clone())));
        }
        self.pending_normal_unload = false;

        // 1. Validate command against single controller authority, safety state, and soft limits
        self.safety.validate_motion_command(
            epoch_id,
            &target_positions,
            &self.config.limits,
        )?;
        self.torque_hold_filter(&mut target_positions);
        self.power_hold_filter(&mut target_positions);

        // Gripper stall release check on reverse command
        if let Some(g_idx) = self.config.gripper_index() {
            if self.gripper_stalled && target_positions.len() > g_idx && self.commanded_positions.len() > g_idx {
                let delta = target_positions[g_idx] - self.commanded_positions[g_idx];
                let tau_g = self.torques.get(g_idx).copied().unwrap_or(0.0);
                if (delta * tau_g < -1e-4) || (delta * self.gripper_stall_sign < -1e-4) {
                    self.gripper_stalled = false;
                    self.gripper_high_torque_ticks = 0;
                    self.gripper_low_torque_ticks = 0;
                }
            }
        }

        // 2. Plan quintic polynomial trajectory from commanded state to target
        let traj = TrajectoryPlanner::plan(
            &self.commanded_positions,
            &self.commanded_velocities,
            &target_positions,
            &vec![0.0; self.config.dof],
            duration_secs,
            limits,
        )?;
        // 2.1 Power budget: slow the move to fit (or refuse with POWER_BUDGET); the workspace
        // check below runs on the trajectory that will be executed
        let (traj, power_scale) = self.power_fit_quintic(traj, &self.commanded_positions, &self.commanded_velocities, &target_positions, limits, true)?;
        let samples = self.workspace_samples(&traj)?;
        self.check_workspace_path(&self.commanded_positions, samples.iter().map(Vec::as_slice))?;

        // 3. Only renew lease AFTER command validation and planning succeed
        self.last_heartbeat = Instant::now();

        self.servo_target = None;
        self.servo_ws_min_depths = None;
        self.target_positions = target_positions;
        self.safety.mark_motion_started()?;
        let duration = traj.duration;
        let id = self.begin_action(kind, tag);
        if let Some(s) = power_scale { self.note_action_power_scale(id, s); }
        self.active_trajectory = Some(ActiveTrajectory::quintic(traj, Some(id)).with_motion_limits(limits.to_vec()));

        Ok((self.safety.epoch_id(), id, duration))
    }

    // ---- actions ----

    fn now_s(&self) -> f64 { self.start_time.elapsed().as_secs_f64() }

    /// Start a new action record; a running one is cancelled as SUPERSEDED.
    pub fn begin_action(&mut self, kind: &str, tag: Option<String>) -> u64 {
        let now = self.now_s();
        // any new action ends a running macro (its orchestrator sees the record cancelled)
        self.macro_run = None;
        #[cfg(feature = "policy-runtime")]
        if let Some(mut run) = self.policy_run.take() {
            run.stop_reason = Some("SUPERSEDED".to_string());
            if run.mode == crate::policy_runtime::PolicyMode::Live {
                self.servo_target = None;
                self.servo_settled_ticks.fill(0);
                self.servo_ws_min_depths = None;
            }
            self.last_policy_obs = Some(run.to_observation(false));
        }
        if let Some(mut old) = self.last_action.take() {
            if old.is_running() {
                old.state = "cancelled".into();
                old.code = Some("SUPERSEDED".into());
                old.message = Some(format!("Replaced by a new {kind} action"));
                old.finished_at = Some(now);
            }
            self.previous_action = Some((old, now + 2.0));
        }
        self.next_action_id += 1;
        let id = self.next_action_id;
        self.last_action = Some(ActionStatus {
            id, kind: kind.into(), tag, state: "running".into(), code: None, message: None, progress: 0.0,
            started_at: now, finished_at: None, holding: None, gripper_gap_rad: None, settled: None,
            task: None, phase: None, iteration: None, error_px: None, final_offset_px: None, acquire_height_m: None,
            episode: None, t_s: None, power_scale: None,
            sequence_id: None, step_index: None, step_type: None, repeat_index: None, repeats: None, failed_step: None, episodes: Vec::new(),
            align: None, align_turn_rad: None, angle_spread_deg: None, orientation: None, grasp_width_m: None, height_source: None, height_reason: None, object_height_m: None, close_height_m: None, probe_dz_m: None, depth_anchor: None, object_depth_m: None, object_length_m: None, jaw_clear: None, border: None, border_shift_px: None, probe_mode: None, lateral_baseline_m: None, lateral_baseline_err_mm: None, height_estimates: Vec::new(), yaw_estimates: Vec::new(), width_estimates: Vec::new(),
            recentre: None, recentre_iterations: None, descend_peak_residual_nm: None, descend_peak_tracking_rad: None,
            descend_contact: None, descend_contact_height_m: None, post_lift: None,
            pre_open: None, pre_open_m: None, held_width_m: None,
            stop_rad: None, offset_rad: None, pass: None,
            approach: None, approach_reason: None, approach_signs: None, approach_flipped: None,
            phase_ms: std::collections::BTreeMap::new(), motions: Vec::new(),
            constraint_id: None, geometry_id: None, violation_phase: None, reason: None,
            durability_confirmed: None, durability_warning: None, policy_stats: None, measurement: None,
        });
        self.action_lease_generation = self.active_lease.as_ref().map(|l| l.generation);
        self.quintic_settle = None;
        id
    }

    pub fn finish_action(&mut self, id: u64, state: &str, code: Option<&str>, message: Option<String>,
                     settled: Option<bool>, holding: Option<bool>, gap: Option<f32>) {
        let now = self.now_s();
        let mut store_offset_req: Option<(f32, Option<(crate::auth::OperatorAuth, Option<String>)>)> = None;
        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id && a.is_running()) {
            a.state = state.into();
            a.code = code.map(Into::into);
            a.message = message;
            a.finished_at = Some(now);
            if state != "failed" || code != Some("WORKSPACE_VIOLATION") {
                a.constraint_id = None;
                a.geometry_id = None;
                a.violation_phase = None;
                a.reason = None;
            }
            if state == "done" { a.progress = 1.0; }
            if settled.is_some() { a.settled = settled; }
            if holding.is_some() { a.holding = holding; }
            if gap.is_some() { a.gripper_gap_rad = gap; }
            if a.kind == "gripper_zero_check" {
                if let Some(params) = self.zero_check_params.take().filter(|p| p.action_id == id) {
                    if state == "done" {
                        let stop_rad = self.config.gripper_index().and_then(|idx| self.measured_positions.get(idx)).copied().unwrap_or(0.0);
                        let offset_rad = stop_rad - params.base_closed_rad;
                        let pass = offset_rad.abs() <= params.threshold_rad;
                        a.stop_rad = Some(stop_rad);
                        a.offset_rad = Some(offset_rad);
                        a.pass = Some(pass);
                        self.gripper_zero_checked = true;
                        self.last_zero_check_offset = Some(offset_rad);
                        if params.store_offset {
                            if offset_rad.abs() <= 0.5 {
                                store_offset_req = Some((offset_rad, params.operator_auth));
                            } else {
                                tracing::warn!("Gripper zero check offset {offset_rad:.4} exceeds 0.5 rad; storing refused");
                            }
                        }
                    }
                }
            }
            tracing::info!("Action {} ({}) {}{}", id, a.kind, state, code.map(|c| format!(" {c}")).unwrap_or_default());
        }
        if let Some((offset_rad, operator_auth)) = store_offset_req {
            let candidate_jaw = self.gripper_calib_jaw_opening.clone();
            match self.build_gripper_calibration_candidate(Some(offset_rad), candidate_jaw.clone()) {
                Err(e) => {
                    tracing::warn!("Failed to apply gripper calibration after zero_check: {e}");
                    if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
                        a.state = "failed".into();
                        a.code = Some("INVALID_CALIBRATION".into());
                        a.message = Some(format!("Gripper calibration rejected: {e}"));
                    }
                }
                Ok((cand, _)) => {
                    let path = self.gripper_calib_path.clone().unwrap_or_else(crate::gripper_calib::path_from_env);
                    let file = crate::gripper_calib::GripperCalibFile {
                        offset_rad: Some(offset_rad),
                        jaw_opening: candidate_jaw.clone(),
                        ts: Some(std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs())),
                        stale: self.gripper_calib_stale.clone(),
                    };
                    let text = serde_json::to_string_pretty(&file).unwrap_or_default();
                    if self.defer_zero_check_persist {
                        // Hold action as running until atomic persist-before-commit completes outside the lock.
                        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
                            a.state = "running".into();
                            a.finished_at = None;
                        }
                        self.zero_check_persisting = Some(id);
                        self.pending_zero_check_persist = Some(PendingZeroCheckPersist {
                            action_id: id,
                            offset_rad,
                            jaw_opening: candidate_jaw,
                            base_revision: self.gripper_calib_revision,
                            lease_owner: self.active_lease.as_ref().map(|l| (l.client_id.clone(), l.generation)),
                            operator_auth,
                            write: crate::persist::PendingWrite::new(path, text),
                        });
                    } else {
                        let auth_ok = operator_auth.as_ref().is_none_or(|(a, tok)| !a.is_enforced() || a.is_operator(tok.as_deref()));
                        if !auth_ok {
                            if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
                                a.state = "failed".into();
                                a.code = Some("OPERATOR_REQUIRED".into());
                                a.message = Some("Operator session expired or locked before gripper calibration persisted".into());
                            }
                        } else {
                            match crate::persist::StagedWrite::stage(path, text) {
                                Err(e) => {
                                    tracing::warn!("Failed to stage gripper calibration after zero_check: {e}");
                                    if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
                                        a.state = "failed".into();
                                        a.code = Some("PERSIST_FAILED".into());
                                        a.message = Some(format!("Failed to persist gripper calibration: {e}"));
                                    }
                                }
                                Ok(staged) => match staged.rename_only() {
                                    Err(e) => {
                                        tracing::warn!("Failed to publish gripper calibration after zero_check: {e}");
                                        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
                                            a.state = "failed".into();
                                            a.code = Some("PERSIST_FAILED".into());
                                            a.message = Some(format!("Failed to persist gripper calibration: {e}"));
                                        }
                                    }
                                    Ok(dir_sync) => {
                                        self.commit_gripper_calibration_candidate(cand);
                                        self.gripper_calib_revision += 1;
                                        let outcome = dir_sync.sync_parent_dir();
                                        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
                                            match outcome {
                                                crate::persist::PublishOutcome::Published => {
                                                    a.durability_confirmed = Some(true);
                                                }
                                                crate::persist::PublishOutcome::PublishedDurabilityUnconfirmed(w) => {
                                                    a.durability_confirmed = Some(false);
                                                    a.durability_warning = Some(w);
                                                }
                                            }
                                        }
                                    }
                                },
                            }
                        }
                    }
                }
            }
        }
        if holding == Some(false) { self.held = None; }
        // a standalone close that detected an object counts as holding until a release completes
        if holding == Some(true) && self.held.is_none() {
            self.held = Some(HeldObject { label: String::new(), spill_safe: false });
        }
        if self.quintic_settle.as_ref().is_some_and(|(sid, _)| *sid == id) { self.quintic_settle = None; }
        if self.macro_run.as_ref().is_some_and(|m| m.id == id) { self.macro_run = None; }
    }

    /// Start a park (NormalUnload) and record it as a `park` action. Returns its id when
    /// the park started.
    /// A park whose path enters a keep-out box is `WORKSPACE_VIOLATION`.
    /// After a fault or E-stop clear the motors may still hold the arm: safety Idle, torque on,
    /// re-arm pending (clear_fault keeps the hold rather than drop an arm that is not at rest).
    pub fn holding_after_clear(&self) -> bool {
        self.is_torqued && self.pending_rearm && !self.disable_pending
            && matches!(self.safety.state(), SafetyState::Idle)
            && !self.e_stop_latched.load(Ordering::SeqCst)
    }

    pub fn unload_action(&mut self) -> Result<Option<u64>, ArmError> {
        self.park_rejection = None;
        // A park from the hold after a clear re-arms through the verified enable path first (fresh
        // feedback, commanded = measured); without it NormalUnload keeps holding and parks nothing.
        if self.holding_after_clear() {
            self.set_torque(true)?;
        }
        self.execute_stop(StopPolicy::NormalUnload);
        if let Some(e) = self.park_rejection.take() { return Err(e.into()); }
        if !self.pending_normal_unload { return Ok(None); }
        let id = self.begin_action("park", None);
        if let Some(active) = self.active_trajectory.as_mut() { active.action_id = Some(id); }
        Ok(Some(id))
    }

    /// Preconditions shared by Cartesian and gripper actions.
    pub(crate) fn action_preconditions(&self) -> Result<(), ArmError> {
        if self.handover_settling_since.is_some() { return Err(SafetyError::HandoverSettling.into()); }
        if self.e_stop_latched.load(Ordering::SeqCst) { return Err(SafetyError::EStopActive.into()); }
        match self.safety.state() {
            SafetyState::EStop => return Err(SafetyError::EStopActive.into()),
            SafetyState::Fault(r) => return Err(SafetyError::FaultActive(r.clone()).into()),
            s if !s.is_operational() => return Err(SafetyError::NotArmed(s.clone()).into()),
            _ => {}
        }
        if self.pending_rearm || self.pending_normal_unload {
            return Err(SafetyError::NotArmed(self.safety.state().clone()).into());
        }
        self.collision_hold_check()?;
        if let Some(reason) = self.busy_reason() { return Err(MotionError::MotionBusy(reason).into()); }
        Ok(())
    }

    /// Why the workspace rule may not change now: a motion, macro, stream or park is
    /// running (each was checked against the current rule).
    pub fn workspace_change_blocker(&self) -> Option<String> {
        if self.macro_run.is_some() { return Some("a macro action is running".into()); }
        if self.pending_normal_unload { return Some("a park is running".into()); }
        self.busy_reason()
    }

    fn busy_reason(&self) -> Option<String> {
        if self.calibration_busy { return Some("a calibration apply is running".into()); }
        if self.zero_check_persisting.is_some() { return Some("gripper calibration is being persisted".into()); }
        #[cfg(feature = "policy-runtime")]
        if self.policy_loading { return Some("a policy is loading".into()); }
        #[cfg(feature = "policy-runtime")]
        if self.policy_run.is_some() { return Some("a policy run is active".into()); }
        if self.active_trajectory.is_some() { return Some("a trajectory is running".into()); }
        if self.servo_target.is_some() { return Some("servo streaming is active".into()); }
        if self.commanded_velocities.iter().any(|v| v.abs() >= 1e-3) { return Some("the arm is still moving".into()); }
        None
    }

    /// Snapshot for planning a Cartesian action outside the lock.
    pub fn plan_input(&self) -> Result<PlanInput, ArmError> {
        if self.config.kinematics.is_none() || !self.config.has_cartesian_with_gripper() {
            return Err(MotionError::Unsupported("Cartesian actions need a kinematics model and an arm with gripper".into()).into());
        }
        self.action_preconditions()?;
        self.gravity_comp_motion_busy()?;
        let arm_dof = self.config.arm_dof();
        let gripper_idx = self.config.gripper_index().unwrap_or(arm_dof);
        Ok(PlanInput {
            q0: crate::kinematics::q6(&self.commanded_positions[..arm_dof.min(self.commanded_positions.len())]),
            j7: self.commanded_positions.get(gripper_idx).copied().unwrap_or(0.0),
            limits: self.planning_limits(),
            speed_cap: self.config.speed_cap,
            ws: self.workspace_policy().map(|(ws, _)| ws.clone()),
            cam: self.config.camera_mount.clone(),
            arm: self.config.arm_model.clone(),
            epoch: self.safety.epoch_id(),
            workspace_revision: self.workspace_revision,
            power: self.power_budget().cloned(),
        })
    }

    /// Install a planned path if nothing changed since `input` was taken.
    /// Returns (action id, expected duration).
    pub fn install_path(&mut self, input: &PlanInput, path: JointPath, kind: &str, tag: Option<String>) -> Result<(u64, f64), ArmError> {
        self.check_plan_fresh(input)?;
        let duration = self.start_path_motion(input, &path)?;
        let id = self.begin_action(kind, tag);
        if input.power.is_some() { self.note_action_power_scale(id, path.power_scale); }
        self.active_trajectory = Some(ActiveTrajectory::new(Some(id), Reference::Path(PathTrajectory::new(path))));
        Ok((id, duration + 0.5))
    }

    /// Joint-space goal seeds besides the current joints ([`crate::motion::goal_seeds`]): the
    /// profile home and the resting pose (when it differs), arm joints only.
    pub fn goal_seed_poses(&self) -> Vec<[f64; 6]> {
        let arm_dof = self.config.arm_dof();
        let q6 = |q: &[f32]| crate::kinematics::q6(&q[..arm_dof.min(q.len())]);
        let mut seeds = vec![q6(&self.config.home_positions)];
        let rest = q6(self.config.resting_positions());
        if rest != seeds[0] { seeds.push(rest); }
        seeds
    }

    /// `move_to` / `move_relative` with `path: "joint"`: the joint-space quintic of
    /// `move_to_pose` from the commanded joints to the arm joints `q` (gripper kept), with the
    /// arm joints' planning velocity times `speed_scale`. The quintic is checked like every joint
    /// move (swept workspace, keep-out boxes, link capsules, power budget, torque and collision
    /// holds). `same_side`: end with the same-side approach of `move_to_precise`.
    /// Returns (action id, expected duration).
    #[allow(clippy::too_many_arguments)]
    pub fn install_joint_goal(&mut self, input: &PlanInput, q: &[f64; 6], speed_scale: f64, kind: &str, tag: Option<String>,
        same_side: Option<crate::backlash::Request>) -> Result<(u64, f64), ArmError> {
        self.check_plan_fresh(input)?;
        let target = self.joint_goal_target(q);
        let limits = self.joint_goal_limits(speed_scale);
        let (_, id, duration) = match same_side {
            Some(request) => self.start_quintic_same_side(target, Some(input.epoch), kind, tag, &limits, "move_to_precise", request)?,
            None => self.start_quintic(target, None, Some(input.epoch), kind, tag, &limits)?,
        };
        Ok((id, duration))
    }

    /// Whether the joint-space quintic to the arm joints `q` would pass the checks of
    /// [`start_quintic`](Self::start_quintic) that depend on the path (soft limits, swept
    /// workspace); nothing starts. Used for the `path: "joint"` hint of a failed straight line.
    pub fn joint_goal_feasible(&self, q: &[f64; 6]) -> bool {
        let target = self.joint_goal_target(q);
        let limits = self.planning_limits();
        if target.iter().zip(&self.config.limits).any(|(t, l)| !t.is_finite() || *t < l.min_position || *t > l.max_position) {
            return false;
        }
        let Ok(traj) = crate::trajectory::TrajectoryPlanner::plan(
            &self.commanded_positions, &vec![0.0; self.config.dof], &target, &vec![0.0; self.config.dof], None, &limits) else { return false; };
        match self.workspace_samples(&traj) {
            Ok(samples) => self.check_workspace_path(&self.commanded_positions, samples.iter().map(Vec::as_slice)).is_ok(),
            Err(_) => false,
        }
    }

    /// The commanded joints with the arm joints replaced by `q` (gripper and extra joints kept).
    pub(crate) fn joint_goal_target(&self, q: &[f64; 6]) -> Vec<f32> {
        let arm_dof = self.config.arm_dof();
        let gripper_idx = self.config.gripper_index();
        let mut target = self.commanded_positions.clone();
        for (i, t) in target.iter_mut().enumerate().take(arm_dof.min(6)) {
            if Some(i) != gripper_idx { *t = q[i] as f32; }
        }
        target
    }

    /// `planning_limits()` with the non-gripper velocities times `speed_scale` (as `move_to_pose`).
    pub(crate) fn joint_goal_limits(&self, speed_scale: f64) -> Vec<crate::model::JointLimit> {
        let gripper_idx = self.config.gripper_index();
        let mut limits = self.planning_limits();
        for (i, l) in limits.iter_mut().enumerate() {
            if Some(i) != gripper_idx { l.max_velocity *= speed_scale as f32; }
        }
        limits
    }

    /// PLAN_STALE checks shared by every installed plan.
    pub(crate) fn check_plan_fresh(&self, input: &PlanInput) -> Result<(), ArmError> {
        if self.safety.epoch_id() != input.epoch {
            return Err(MotionError::PlanStale("epoch changed while planning".into()).into());
        }
        if self.workspace_revision != input.workspace_revision {
            return Err(MotionError::PlanStale("workspace or keep-out boxes changed while planning".into()).into());
        }
        match self.action_preconditions() {
            Err(ArmError::Motion(MotionError::MotionBusy(reason))) => {
                return Err(MotionError::PlanStale(format!("arm no longer idle ({reason})")).into());
            }
            other => other?,
        }
        let arm_dof = input.arm.arm_dof();
        let gripper_idx = self.config.gripper_index().unwrap_or(arm_dof);
        let moved = (0..arm_dof).any(|i| (f64::from(self.commanded_positions[i]) - input.q0[i]).abs() >= 1e-4)
            || (self.commanded_positions.get(gripper_idx).copied().unwrap_or(0.0) - input.j7).abs() >= 1e-4;
        if moved { return Err(MotionError::PlanStale("commanded positions changed while planning".into()).into()); }
        Ok(())
    }

    fn start_path_motion(&mut self, input: &PlanInput, path: &JointPath) -> Result<f64, ArmError> {
        let last = path.last_slice().to_vec();
        self.safety.validate_motion_command(Some(input.epoch), &last, &self.config.limits)?;
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        self.servo_ws_min_depths = None;
        self.target_positions = last;
        self.safety.mark_motion_started()?;
        self.last_heartbeat = Instant::now();
        Ok(path.duration)
    }

    // ---- grasp ----

    /// Runtime grasp setup (`config.grasp`, with the camera tilt taken from `camera_mount`).
    /// None when the profile cannot grasp (no kinematics, gripper policy or grasp section).
    pub fn grasp_setup(&self) -> Option<crate::grasp::GraspSetup> {
        if !self.config.has_cartesian_with_gripper() || self.config.kinematics.is_none() || self.config.gripper_policy.is_none() { return None; }
        let mut s = self.config.grasp.clone()?;
        s.camera_tilt_deg = f64::from(self.config.camera_mount.tilt_deg);
        Some(s)
    }

    /// The camera mount is calibrated (`calibration.json` with `q_end_cam` in effect).
    pub fn camera_calibrated(&self) -> bool {
        self.calibration.as_ref().is_some_and(|c| c.camera_mount.q_end_cam.is_some())
    }

    /// Install the calibration in effect (load / apply / revert / restore / zero invalidation)
    /// and recompute the effective floor.
    pub fn set_calibration(&mut self, c: Option<crate::calibration::CalibrationFile>) {
        self.calibration = c;
        self.refresh_effective_floor();
    }

    /// Recompute `config.workspace.floor_z_m` = max(profile floor, valid calibrated table
    /// height - 3 mm) (src/calibration.rs `effective_floor`). Every floor check (end points,
    /// link capsules, swept paths, servo steps) reads that field. A change bumps the workspace
    /// revision (plans made with the old floor fail PLAN_STALE).
    pub fn refresh_effective_floor(&mut self) {
        let Some(profile) = self.profile_floor_z_m else { return; };
        let (_, fp) = crate::calibration::live_fingerprints(self);
        let (floor, source, _) = crate::calibration::effective_floor(profile, self.calibration.as_ref(), &fp);
        self.floor_source = source;
        let Some(ws) = self.config.workspace.as_mut() else { return; };
        if ws.floor_z_m != floor {
            tracing::info!("Effective floor {:.4} m (was {:.4} m, profile {profile:.4} m, source {source})", floor, ws.floor_z_m);
            ws.floor_z_m = floor;
            self.workspace_revision += 1;
            self.servo_ws_min_depths = None;
        }
    }

    /// Install a camera mount (calibration apply / restore / startup): bumps the workspace
    /// revision when it changes and keeps the grasp setup's tilt in sync with the mount's.
    pub fn set_camera_mount(&mut self, mount: crate::model::CameraMount) {
        if self.config.camera_mount != mount {
            self.workspace_revision += 1;
            self.servo_ws_min_depths = None;
        }
        self.config.camera_mount = mount;
        let tilt = f64::from(self.config.camera_mount.tilt_deg);
        if let Some(g) = self.config.grasp.as_mut() { g.camera_tilt_deg = tilt; }
        self.refresh_effective_floor();
    }

    /// Install the kinematic correction of the built-in model (calibration apply / restore /
    /// startup; None = uncorrected): the live swap of `config.arm_model`, which every FK / IK /
    /// workspace / keep-out / grasp path reads under this lock. Bumps the workspace revision when
    /// it changes (plans made with the old model fail PLAN_STALE) and recomputes the stored
    /// poses' TCPs. Err (nothing changed): a URDF chain profile or values outside the bounds.
    pub fn set_kinematics_correction(&mut self, corr: Option<&crate::kinematics::Correction>) -> Result<(), String> {
        let model = self.config.arm_model.with_correction(corr)?;
        if model.correction() == self.config.arm_model.correction() { return Ok(()); }
        self.config.arm_model = model;
        // the servo escape baseline was measured with the old model
        self.workspace_revision += 1;
        self.servo_ws_min_depths = None;
        self.calibration_revision += 1;
        self.refresh_pose_tcps();
        self.refresh_effective_floor();
        Ok(())
    }

    /// Replace the runtime setup; `camera_tilt_deg` also sets `camera_mount.tilt_deg`.
    pub fn set_grasp_setup(&mut self, setup: crate::grasp::GraspSetup) {
        if self.config.camera_mount.tilt_deg != setup.camera_tilt_deg as f32 {
            self.workspace_revision += 1;
            self.servo_ws_min_depths = None;
        }
        self.config.camera_mount.tilt_deg = setup.camera_tilt_deg as f32;
        self.config.grasp = Some(setup);
    }

    // ---- macro actions (crate::macro_action) ----

    /// Begin a macro action (`grasp`, `place`) after its first motion was planned from
    /// `input`. Its orchestrator drives it from now on. Returns its id.
    pub fn begin_macro(&mut self, kind: &str, input: &PlanInput, tag: Option<String>) -> Result<u64, ArmError> {
        self.check_plan_fresh(input)?;
        let id = self.begin_action(kind, tag);
        self.macro_run = Some(MacroRun { id, epoch: input.epoch, sub: SubState::Idle, stop_requested: false, phase: None, phase_start: None });
        self.set_action_phase(id, "start", 0.0, None, None);
        Ok(id)
    }

    /// Begin a sequence run (parent macro action of kind `sequence`, no Cartesian plan needed).
    pub fn begin_sequence_action(&mut self, tag: Option<String>, epoch: u64) -> Result<u64, ArmError> {
        self.begin_orchestrated_action("sequence", tag, epoch)
    }

    /// Begin a parent action of `kind` whose orchestrator runs outside the lock (sequence,
    /// calib_hover): epoch, preconditions, macro run record.
    pub fn begin_orchestrated_action(&mut self, kind: &str, tag: Option<String>, epoch: u64) -> Result<u64, ArmError> {
        if epoch != self.safety.epoch_id() {
            return Err(SafetyError::EpochMismatch { expected: self.safety.epoch_id(), received: epoch }.into());
        }
        self.action_preconditions()?;
        self.gravity_comp_motion_busy()?;
        let id = self.begin_action(kind, tag);
        self.macro_run = Some(MacroRun { id, epoch, sub: SubState::Idle, stop_requested: false, phase: None, phase_start: None });
        self.set_action_phase(id, "start", 0.0, None, None);
        Ok(id)
    }

    /// The macro action `id` is still the running action.
    pub fn macro_alive(&self, id: u64) -> bool {
        self.macro_run.as_ref().is_some_and(|m| m.id == id && !m.stop_requested)
            && self.last_action.as_ref().is_some_and(|a| a.id == id && a.is_running())
    }

    pub fn macro_sub(&self, id: u64) -> Option<SubState> {
        self.macro_run.as_ref().filter(|m| m.id == id).map(|m| m.sub.clone())
    }

    /// Set `phase` / `progress` (and centring stats) of the running action `id`.
    pub fn set_action_phase(&mut self, id: u64, phase: &str, progress: f32, iteration: Option<u32>, error_px: Option<f32>) {
        if let Some(m) = self.macro_run.as_mut().filter(|m| m.id == id) {
            let now = Instant::now();
            if m.phase.as_deref() != Some(phase) {
                if let (Some(prev_phase), Some(phase_start)) = (m.phase.take(), m.phase_start.take()) {
                    let elapsed_ms = now.duration_since(phase_start).as_millis() as u32;
                    if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
                        *a.phase_ms.entry(prev_phase).or_insert(0) += elapsed_ms;
                    }
                }
                m.phase = Some(phase.to_string());
                m.phase_start = Some(now);
            }
        }
        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id && a.is_running()) {
            a.phase = Some(phase.into());
            a.progress = progress;
            // centring stats stay in the record after the phase (result of the closed loop)
            if iteration.is_some() { a.iteration = iteration; }
            if error_px.is_some() { a.error_px = error_px; }
        }
    }

    /// Grasp camera-down re-check result ([`ActionStatus::final_offset_px`]).
    pub fn set_action_final_offset(&mut self, id: u64, offset_px: [f32; 2]) {
        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id && a.is_running()) {
            a.final_offset_px = Some(offset_px);
        }
    }

    /// Grasp camera-down acquisition height ([`ActionStatus::acquire_height_m`]).
    pub fn set_action_acquire_height(&mut self, id: u64, height_m: f32) {
        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id && a.is_running()) {
            a.acquire_height_m = Some(height_m);
        }
    }

    /// Change diagnostic fields of the running action `id` (grasp status fields).
    pub fn update_action(&mut self, id: u64, f: impl FnOnce(&mut ActionStatus)) {
        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id && a.is_running()) { f(a); }
    }

    /// Record structured workspace violation details on running action `id`.
    pub fn set_action_workspace_violation(&mut self, id: u64, detail: crate::kinematics::WorkspaceViolationDetail) {
        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id && a.is_running()) {
            a.constraint_id = detail.constraint_id;
            a.geometry_id = detail.geometry_id;
            a.violation_phase = detail.violation_phase;
            a.reason = detail.reason;
        }
    }

    /// Stop the running sub-motion of macro action `id` where the arm is (J1..J6 commanded to
    /// the measured positions, the gripper keeps its command) and report it done to the
    /// orchestrator. False when no sub-motion of `id` is running.
    pub fn end_macro_motion_at_measured(&mut self, id: u64) -> bool {
        if !self.macro_alive(id) || !self.active_trajectory.as_ref().is_some_and(|a| a.action_id == Some(id)) { return false; }
        self.stop_motion();
        let n = self.measured_positions.len().min(self.commanded_positions.len()).min(6);
        if self.measured_positions[..n].iter().all(|q| q.is_finite()) {
            for i in 0..n {
                let l = &self.config.limits[i];
                self.commanded_positions[i] = self.measured_positions[i].clamp(l.min_position, l.max_position);
            }
        }
        self.commanded_velocities.fill(0.0);
        self.commanded_accelerations.fill(0.0);
        self.target_positions = self.commanded_positions.clone();
        self.safety.mark_motion_finished();
        if let Some(m) = self.macro_run.as_mut() {
            m.sub = SubState::Done(SubDone { settled: None, holding: None, gripper_gap_rad: None, at: Instant::now() });
        }
        true
    }

    fn macro_sub_ready(&self, id: u64) -> Result<u64, ArmError> {
        let run = self.macro_run.as_ref().filter(|m| m.id == id && self.macro_alive(id))
            .ok_or_else(|| ArmError::Internal("the action is no longer running".into()))?;
        if run.sub == SubState::Running {
            return Err(MotionError::MotionBusy("a sub-motion of the action is running".into()).into());
        }
        Ok(run.epoch)
    }

    /// [`macro_sub_ready`](Self::macro_sub_ready) for sub-motions installed by other modules.
    pub(crate) fn macro_sub_ready_pub(&self, id: u64) -> Result<u64, ArmError> {
        let epoch = self.macro_sub_ready(id)?;
        if epoch != self.safety.epoch_id() { return Err(MotionError::PlanStale("epoch changed during the action".into()).into()); }
        Ok(epoch)
    }

    pub(crate) fn set_macro_sub_running(&mut self, id: u64) {
        if let Some(m) = self.macro_run.as_mut().filter(|m| m.id == id) { m.sub = SubState::Running; }
    }

    /// Install a planned sub-motion of macro action `id`. Returns its planned duration.
    pub fn install_macro_path(&mut self, id: u64, input: &PlanInput, path: JointPath) -> Result<f64, ArmError> {
        let epoch = self.macro_sub_ready(id)?;
        if input.epoch != epoch { return Err(MotionError::PlanStale("epoch changed during the action".into()).into()); }
        self.check_plan_fresh(input)?;
        let duration = self.start_path_motion(input, &path)?;
        if input.power.is_some() { self.note_action_power_scale(id, path.power_scale); }
        self.active_trajectory = Some(ActiveTrajectory::new(Some(id), Reference::Path(PathTrajectory::new(path))));
        if let Some(m) = self.macro_run.as_mut() { m.sub = SubState::Running; }
        Ok(duration + 0.5)
    }

    /// Start a gripper sub-motion of macro action `id`. Returns its expected duration.
    pub fn start_macro_gripper(&mut self, id: u64, request: GripperRequest) -> Result<f64, ArmError> {
        let epoch = self.macro_sub_ready(id)?;
        let gripper_idx = self.config.gripper_index().ok_or_else(|| {
            MotionError::Unsupported("Gripper actions need a profile with a gripper joint".into())
        })?;
        let Some(policy) = self.config.gripper_policy.clone() else {
            return Err(MotionError::Unsupported("Gripper actions need a profile with a gripper policy".into()).into());
        };
        if epoch != self.safety.epoch_id() { return Err(SafetyError::EpochMismatch { expected: self.safety.epoch_id(), received: epoch }.into()); }
        self.action_preconditions()?;
        self.releasing = matches!(request.command, GripperCommand::Open);
        let motion = GripperMotion::new(request, policy, self.config.limits[gripper_idx].clone(), self.commanded_positions[gripper_idx], gripper_idx);
        let duration = motion.expected_duration();
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        self.servo_ws_min_depths = None;
        let mut target = self.commanded_positions.clone();
        target[gripper_idx] = motion.target;
        self.target_positions = target;
        self.safety.mark_motion_started()?;
        self.last_heartbeat = Instant::now();
        self.active_trajectory = Some(ActiveTrajectory::new(Some(id), Reference::Gripper(Box::new(motion))));
        if let Some(m) = self.macro_run.as_mut() { m.sub = SubState::Running; }
        Ok(duration)
    }

    /// Start a joint-space quintic sub-motion of macro action `id` (sequence pose step) to
    /// `target`, planned with `limits` from the commanded state; it counts as done once settled.
    /// Returns its planned duration.
    pub fn install_macro_quintic(&mut self, id: u64, target: Vec<f32>, limits: &[crate::model::JointLimit]) -> Result<f64, ArmError> {
        let epoch = self.macro_sub_ready(id)?;
        if epoch != self.safety.epoch_id() { return Err(MotionError::PlanStale("epoch changed during the action".into()).into()); }
        self.action_preconditions()?;
        self.gravity_comp_motion_busy()?;
        let mut target = target;
        self.safety.validate_motion_command(Some(epoch), &target, &self.config.limits)?;
        self.torque_hold_filter(&mut target);
        self.power_hold_filter(&mut target);
        let traj = TrajectoryPlanner::plan(&self.commanded_positions, &self.commanded_velocities, &target,
            &vec![0.0; self.config.dof], None, limits)?;
        let (traj, power_scale) = self.power_fit_quintic(traj, &self.commanded_positions, &self.commanded_velocities, &target, limits, true)?;
        let samples = self.workspace_samples(&traj)?;
        self.check_workspace_path(&self.commanded_positions, samples.iter().map(Vec::as_slice))?;
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        self.target_positions = target;
        self.safety.mark_motion_started()?;
        self.last_heartbeat = Instant::now();
        if let Some(s) = power_scale { self.note_action_power_scale(id, s); }
        let duration = traj.duration;
        self.quintic_settle = None;
        self.active_trajectory = Some(ActiveTrajectory::quintic(traj, Some(id)).with_motion_limits(limits.to_vec()));
        if let Some(m) = self.macro_run.as_mut() { m.sub = SubState::Running; }
        Ok(duration + crate::motion::SETTLE_MAX_S)
    }

    /// End macro action `id` from its orchestrator (no-op when it already ended).
    pub fn finish_macro(&mut self, id: u64, state: &str, code: Option<&str>, message: Option<String>, holding: Option<bool>, gap: Option<f32>) {
        if !self.macro_alive(id) { return; }
        if self.active_trajectory.as_ref().is_some_and(|a| a.action_id == Some(id)) { self.stop_motion(); }
        if state == "done" { self.set_action_phase(id, "done", 1.0, None, None); }
        if let Some(m) = self.macro_run.as_mut().filter(|m| m.id == id) {
            if let (Some(prev_phase), Some(phase_start)) = (m.phase.take(), m.phase_start.take()) {
                let ms = Instant::now().duration_since(phase_start).as_millis() as u32;
                if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
                    *a.phase_ms.entry(prev_phase).or_insert(0) += ms;
                }
            }
        }
        self.finish_action(id, state, code, message, None, holding, gap);
    }

    /// `/motion/stop`: ends the running macro action now, under the caller's lock, so its
    /// orchestrator cannot install another sub-motion before the next tick.
    pub fn request_macro_stop(&mut self) {
        let Some(mut m) = self.macro_run.take() else { return; };
        m.stop_requested = true;
        let id = m.id;
        if self.active_trajectory.as_ref().is_some_and(|a| a.action_id == Some(id)) { self.stop_motion(); }
        if let (Some(prev_phase), Some(phase_start)) = (m.phase.take(), m.phase_start.take()) {
            let ms = Instant::now().duration_since(phase_start).as_millis() as u32;
            if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
                *a.phase_ms.entry(prev_phase).or_insert(0) += ms;
            }
        }
        self.finish_action(id, "cancelled", Some("STOPPED"), Some("Motion stopped".into()), None, None, None);
    }

    /// Start a gripper action. Returns (action id, expected duration).
    pub fn start_gripper(&mut self, request: GripperRequest, epoch: u64) -> Result<(u64, f64), ArmError> {
        let gripper_idx = self.config.gripper_index().ok_or_else(|| {
            MotionError::Unsupported("Gripper actions need a profile with a gripper joint".into())
        })?;
        let Some(policy) = self.config.gripper_policy.clone() else {
            return Err(MotionError::Unsupported("Gripper actions need a profile with a gripper policy".into()).into());
        };
        if epoch != self.safety.epoch_id() {
            return Err(SafetyError::EpochMismatch { expected: self.safety.epoch_id(), received: epoch }.into());
        }
        self.action_preconditions()?;
        self.releasing = matches!(request.command, GripperCommand::Open);
        let motion = GripperMotion::new(request, policy, self.config.limits[gripper_idx].clone(), self.commanded_positions[gripper_idx], gripper_idx);
        let duration = motion.expected_duration();
        let tag = motion.request.tag.clone();
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        self.servo_ws_min_depths = None;
        let mut target = self.commanded_positions.clone();
        target[gripper_idx] = motion.target;
        self.target_positions = target;
        self.safety.mark_motion_started()?;
        self.last_heartbeat = Instant::now();
        let id = self.begin_action("gripper", tag);
        self.active_trajectory = Some(ActiveTrajectory::new(Some(id), Reference::Gripper(Box::new(motion))));
        Ok((id, duration))
    }

    /// Start a gripper zero check action. Returns (action id, expected duration).
    pub fn start_gripper_zero_check(&mut self, body: &serde_json::Value, epoch: u64) -> Result<(u64, f64), ArmError> {
        self.start_gripper_zero_check_with_auth(body, epoch, None)
    }

    pub fn start_gripper_zero_check_with_auth(
        &mut self,
        body: &serde_json::Value,
        epoch: u64,
        operator_auth: Option<(crate::auth::OperatorAuth, Option<String>)>,
    ) -> Result<(u64, f64), ArmError> {
        let gripper_idx = self.config.gripper_index().ok_or_else(|| {
            MotionError::Unsupported("Gripper zero check needs a profile with a gripper joint".into())
        })?;
        let Some(policy) = self.config.gripper_policy.clone() else {
            return Err(MotionError::Unsupported("Gripper zero check needs a profile with a gripper policy".into()).into());
        };
        if !policy.zero_check_supported {
            return Err(MotionError::Unsupported("Gripper zero check is not supported for this gripper policy".into()).into());
        }
        if epoch != self.safety.epoch_id() {
            return Err(SafetyError::EpochMismatch { expected: self.safety.epoch_id(), received: epoch }.into());
        }
        self.action_preconditions()?;
        if !self.is_torqued {
            return Err(SafetyError::NotArmed(self.safety.state().clone()).into());
        }
        if self.held.is_some() {
            return Err(ArmError::Rejected {
                status: 409,
                code: "OBJECT_HELD".into(),
                message: "Gripper holds an object (or its release did not finish); open the gripper fully before the zero check".into(),
            });
        }
        let store_offset = body.get("store_offset").and_then(serde_json::Value::as_bool).unwrap_or(false);
        let threshold_rad = body.get("threshold_rad").and_then(serde_json::Value::as_f64).map(|v| v as f32).unwrap_or(0.10);
        if !threshold_rad.is_finite() || threshold_rad <= 0.0 {
            return Err(MotionError::InvalidCommand("threshold_rad must be positive and finite".into()).into());
        }

        let request = GripperRequest {
            command: GripperCommand::Seek,
            fraction: None,
            grasp: false,
            speed: 0.3,
            // derived from the travel at 0.3 rad/s (fully open -> closed takes about 20 s)
            timeout_s: None,
            tag: None,
        };
        let motion = GripperMotion::new(request, policy, self.config.limits[gripper_idx].clone(), self.commanded_positions[gripper_idx], gripper_idx);
        let duration = motion.expected_duration();
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        self.servo_ws_min_depths = None;
        let mut target = self.commanded_positions.clone();
        target[gripper_idx] = motion.target;
        self.target_positions = target;
        self.safety.mark_motion_started()?;
        self.last_heartbeat = Instant::now();
        let id = self.begin_action("gripper_zero_check", None);
        self.active_trajectory = Some(ActiveTrajectory::new(Some(id), Reference::Gripper(Box::new(motion))));
        let base_closed_rad = self.base_gripper_policy.as_ref().map_or(0.0, |p| p.closed_rad);
        self.gripper_stalled = false;
        self.gripper_high_torque_ticks = 0;
        self.gripper_low_torque_ticks = 0;
        self.zero_check_params = Some(ZeroCheckParams {
            action_id: id,
            store_offset,
            threshold_rad,
            base_closed_rad,
            operator_auth,
        });
        Ok((id, duration))
    }

    /// Apply gripper calibration: offset shifts closed_rad, open_rad and jaw points.
    pub fn apply_gripper_calibration(
        &mut self,
        offset_rad: Option<f32>,
        jaw_opening: Option<crate::model::JawOpening>,
    ) -> Result<(), String> {
        if self.base_gripper_policy.is_none() {
            self.base_gripper_policy = self.config.gripper_policy.clone();
        }
        let Some(base) = self.base_gripper_policy.clone() else {
            return Err("No base gripper policy available".into());
        };
        let mut policy = base.clone();
        let offset = offset_rad.unwrap_or(0.0);
        if !offset.is_finite() {
            return Err("offset_rad must be finite".into());
        }
        let closed_rad = base.closed_rad + offset;
        let mut open_rad = base.open_rad + offset;
        if !closed_rad.is_finite() || !open_rad.is_finite() {
            return Err("Computed closed_rad or open_rad is not finite".into());
        }
        let lim = self.config.gripper_index().and_then(|idx| self.config.limits.get(idx));
        // The measured stop must lie inside the limits; the open end may be shifted past one
        // (the base open_rad sits close to it), so it is held at the limit.
        if let Some(lim) = lim {
            if closed_rad < lim.min_position || closed_rad > lim.max_position {
                return Err(format!(
                    "Calibrated closed_rad {closed_rad:.4} exceeds joint limits [{}, {}]",
                    lim.min_position, lim.max_position
                ));
            }
            open_rad = open_rad.clamp(lim.min_position, lim.max_position);
        }
        if (closed_rad - open_rad).abs() < 0.1 {
            return Err(format!(
                "Base gripper policy span |closed_rad - open_rad| < 0.1 (closed: {closed_rad}, open: {open_rad})"
            ));
        }
        policy.closed_rad = closed_rad;
        policy.open_rad = open_rad;

        let candidate_jo = jaw_opening.clone()
            .or_else(|| self.gripper_calib_jaw_opening.clone())
            .or_else(|| base.jaw_opening.clone());
        // The table is checked against the limits before the offset shift: the shift can move
        // its open end past a limit, like open_rad, and those points are never reached.
        let mut jo = candidate_jo.unwrap_or_else(crate::model::JawOpening::sim_default);
        jo.validate(lim).map_err(|e| format!("Invalid jaw_opening calibration against joint limits: {e}"))?;
        for pt in &mut jo.points {
            pt[0] += offset;
        }
        jo.validate(None).map_err(|e| format!("Invalid jaw_opening calibration: {e}"))?;
        policy.jaw_opening = Some(jo);

        self.config.gripper_policy = Some(policy);
        self.gripper_calib_offset = offset_rad;
        if jaw_opening.is_some() {
            self.gripper_calib_jaw_opening = jaw_opening;
        }
        self.gripper_calib_invalid = false;
        Ok(())
    }

    pub fn gripper_zero_observation(&self) -> Option<crate::model::GripperZeroObservation> {
        let idx = self.config.gripper_index()?;
        self.config.gripper_policy.as_ref()?;
        let j7 = self.measured_positions.get(idx).copied().unwrap_or(0.0);
        let offset_rad = self.gripper_calib_offset.or(self.last_zero_check_offset);
        let state = if self.gripper_calib_invalid {
            "calib_invalid".to_string()
        } else if self.gripper_zero_checked {
            "checked".to_string()
        } else {
            let policy = self.config.gripper_policy.as_ref().unwrap();
            let min_j7 = policy.closed_rad.min(policy.open_rad) - 0.3;
            let max_j7 = policy.closed_rad.max(policy.open_rad) + 0.3;
            if j7 < min_j7 || j7 > max_j7 {
                "out_of_range".to_string()
            } else {
                "ok".to_string()
            }
        };
        Some(crate::model::GripperZeroObservation {
            state,
            j7,
            offset_rad,
        })
    }

    /// Validate a candidate gripper calibration (including hardware driver limits) without
    /// leaving `self` mutated. Returns the resulting effective GripperPolicy on success.
    /// Build and validate a complete gripper calibration candidate (including driver limit
    /// re-application) without leaving `SupervisorInner` mutated.
    pub fn build_gripper_calibration_candidate(
        &mut self,
        offset_rad: Option<f32>,
        jaw_opening: Option<crate::model::JawOpening>,
    ) -> Result<(GripperCalibrationCandidate, crate::model::GripperPolicy), String> {
        let prev_config = self.config.clone();
        let prev_base = self.base_gripper_policy.clone();
        let prev_offset = self.gripper_calib_offset;
        let prev_jaw = self.gripper_calib_jaw_opening.clone();
        let prev_invalid = self.gripper_calib_invalid;
        let prev_err = self.driver_config_error.clone();

        let res = (|| {
            self.apply_gripper_calibration(offset_rad, jaw_opening)?;
            if self.driver.is_some() || self.driver_config_error.is_some() {
                let ok = self.reapply_driver_limits();
                if !ok || self.gripper_calib_invalid || self.driver_config_error.is_some() {
                    return Err(self.driver_config_error.clone().unwrap_or_else(|| {
                        "Gripper calibration rejected by driver hardware limits".to_string()
                    }));
                }
            }
            let policy = self.config.gripper_policy.clone().ok_or_else(|| {
                "Gripper calibration requires a gripper policy".to_string()
            })?;
            let cand = GripperCalibrationCandidate {
                config: self.config.clone(),
                base_gripper_policy: self.base_gripper_policy.clone(),
                gripper_calib_offset: self.gripper_calib_offset,
                gripper_calib_jaw_opening: self.gripper_calib_jaw_opening.clone(),
                gripper_calib_invalid: self.gripper_calib_invalid,
                driver_config_error: self.driver_config_error.clone(),
            };
            Ok((cand, policy))
        })();

        self.config = prev_config;
        self.base_gripper_policy = prev_base;
        self.gripper_calib_offset = prev_offset;
        self.gripper_calib_jaw_opening = prev_jaw;
        self.gripper_calib_invalid = prev_invalid;
        self.driver_config_error = prev_err;

        res
    }

    /// Infallibly commit a pre-validated [`GripperCalibrationCandidate`] after disk rename.
    pub fn commit_gripper_calibration_candidate(&mut self, cand: GripperCalibrationCandidate) {
        self.config = cand.config;
        self.base_gripper_policy = cand.base_gripper_policy;
        self.gripper_calib_offset = cand.gripper_calib_offset;
        self.gripper_calib_jaw_opening = cand.gripper_calib_jaw_opening;
        self.gripper_calib_invalid = cand.gripper_calib_invalid;
        self.driver_config_error = cand.driver_config_error;
    }

    pub fn validate_gripper_calibration_with_driver(
        &mut self,
        offset_rad: Option<f32>,
        jaw_opening: Option<crate::model::JawOpening>,
    ) -> Result<crate::model::GripperPolicy, String> {
        self.build_gripper_calibration_candidate(offset_rad, jaw_opening).map(|(_, p)| p)
    }

    pub fn gripper_calibration_response(&self) -> crate::gripper_calib::GripperCalibrationResponse {
        let policy = self.config.gripper_policy.clone();
        let calibrated = policy.as_ref().is_some_and(|p| p.is_calibrated());
        let jaw_opening = policy.as_ref().and_then(|p| p.jaw_opening.clone());
        crate::gripper_calib::GripperCalibrationResponse {
            offset_rad: self.gripper_calib_offset,
            jaw_opening,
            calibrated,
            policy,
            ts: None,
            stale: self.gripper_calib_stale.clone(),
            revision: self.gripper_calib_revision,
        }
    }

    pub fn set_mock_gripper_stall_offset(&mut self, offset: Option<f32>) {
        self.mock_gripper_stall_offset = offset;
    }

    /// Single place that ends a running action whose reference went away (spec 2.3).
    fn reconcile_action(&mut self, dt: f64, tick_output_accepted: bool) {
        let Some(action) = self.last_action.as_ref().filter(|a| a.is_running()) else { return; };
        let (id, kind) = (action.id, action.kind.clone());
        let is_macro = self.macro_run.as_ref().is_some_and(|m| m.id == id);
        #[cfg(feature = "policy-runtime")]
        let is_policy = self.policy_run.as_ref().is_some_and(|p| p.action_id == id);
        #[cfg(not(feature = "policy-runtime"))]
        let is_policy = false;
        let carried = self.active_trajectory.as_ref().is_some_and(|a| a.action_id == Some(id));
        let lease_now = self.active_lease.as_ref().map(|l| l.generation);
        let cause: Option<(&str, &str, String)> = match self.safety.state() {
            SafetyState::EStop => Some(("failed", "ESTOP_ACTIVE", "E-stop latched".into())),
            SafetyState::Fault(FaultReason::WatchdogTimeout) => Some(("failed", "WATCHDOG", "Lease watchdog expired".into())),
            SafetyState::Fault(r) => Some(("failed", "FAULT", match r { FaultReason::Other(m) => m.clone(), r => format!("{r:?}") })),
            _ if kind != "park" && self.watchdog_park.is_some() => Some(("failed", "WATCHDOG", "Lease watchdog expired; parking".into())),
            _ if lease_now != self.action_lease_generation => Some(("cancelled", "LEASE_LOST", "Control lease lost or released".into())),
            _ => None,
        };
        if let Some((state, code, message)) = cause {
            // a macro sub-motion or a replay must not outlive its action
            if (is_macro || kind == "replay") && carried { self.stop_motion(); }
            if self.zero_check_persisting == Some(id) { self.zero_check_persisting = None; }
            #[cfg(feature = "policy-runtime")]
            if is_policy {
                let reason = match code {
                    "ESTOP_ACTIVE" => "ESTOP",
                    "WATCHDOG" => "WATCHDOG_TIMEOUT",
                    "LEASE_LOST" => "LEASE_LOST",
                    _ => "FAULT",
                };
                self.terminate_policy_run(reason, state, Some(code), Some(message));
                return;
            }
            self.finish_action(id, state, Some(code), Some(message), None, None, None);
            return;
        }
        if is_policy {
            return;
        }
        if let Some(run) = self.macro_run.as_ref().filter(|m| m.id == id) {
            // between sub-motions the orchestrator owns the action; it ends here only when
            // something else took the arm or a stop arrived
            if carried || self.zero_check_persisting == Some(id) { return; }
            let superseded = self.active_trajectory.is_some() || self.servo_target.is_some();
            // a joint-space sub-motion (sequence pose step) settles before it counts as done
            if !superseded && !run.stop_requested && run.sub == SubState::Running && self.quintic_settle.as_ref().is_some_and(|(sid, _)| *sid == id) {
                if !tick_output_accepted { return; }
                let (cmd, meas, vel) = (&self.commanded_positions, &self.measured_positions, &self.measured_velocities);
                if let Some(settled) = self.quintic_settle.as_mut().and_then(|(_, settle)| settle.step(dt, cmd, meas, vel)) {
                    self.quintic_settle = None;
                    if let Some(m) = self.macro_run.as_mut() {
                        m.sub = SubState::Done(SubDone { settled: Some(settled), holding: None, gripper_gap_rad: None, at: Instant::now() });
                    }
                }
                return;
            }
            let ended: Option<(&str, &str, String)> = if superseded {
                Some(("cancelled", "SUPERSEDED", "Replaced by new motion".into()))
            } else if run.sub == SubState::Running || run.stop_requested {
                Some(("cancelled", "STOPPED", "Motion stopped".into()))
            } else { None };
            if let Some((state, code, message)) = ended { self.finish_action(id, state, Some(code), Some(message), None, None, None); }
            return;
        }
        let superseded = (self.active_trajectory.is_some() && !carried) || self.servo_target.is_some();
        if kind == "park" {
            if self.pending_normal_unload {
                let progress = self.active_trajectory.as_ref().and_then(|a| a.as_quintic()).map_or(1.0, |t| (t.elapsed / t.duration).min(1.0) as f32);
                if let Some(a) = self.last_action.as_mut() { a.progress = progress; }
            } else if !self.is_torqued && matches!(self.safety.state(), SafetyState::Idle) {
                self.finish_action(id, "done", None, None, None, None, None);
            } else if superseded {
                self.finish_action(id, "cancelled", Some("SUPERSEDED"), Some("Replaced by new motion".into()), None, None, None);
            } else {
                self.finish_action(id, "failed", Some("UNLOAD_FAILED"), Some("Park ended with torque still on".into()), None, None, None);
            }
            return;
        }
        if carried || self.zero_check_persisting == Some(id) { return; }
        if self.quintic_settle.as_ref().is_some_and(|(sid, _)| *sid == id) {
            if superseded {
                self.finish_action(id, "cancelled", Some("SUPERSEDED"), Some("Replaced by new motion".into()), None, None, None);
                return;
            }
            if !tick_output_accepted { return; }
            let (cmd, meas, vel) = (&self.commanded_positions, &self.measured_positions, &self.measured_velocities);
            let result = self.quintic_settle.as_mut().and_then(|(_, settle)| settle.step(dt, cmd, meas, vel));
            if let Some(settled) = result {
                self.finish_action(id, "done", None, None, Some(settled), None, None);
            }
            return;
        }
        if superseded {
            self.finish_action(id, "cancelled", Some("SUPERSEDED"), Some("Replaced by new motion".into()), None, None, None);
        } else {
            self.finish_action(id, "cancelled", Some("STOPPED"), Some("Motion stopped".into()), None, None, None);
        }
    }

    /// Apply one executor step of a Path / Gripper / Replay reference.
    /// Returns a deferred completion payload on `StepStatus::Done` so `"done"` is only
    /// published after that tick's output has been validated and written to the driver.
    fn apply_action_step(&mut self, action_id: Option<u64>, out: StepOut) -> Option<DeferredActionDone> {
        self.commanded_positions = out.positions;
        self.commanded_velocities = out.velocities;
        if out.clear_gripper_stall {
            self.gripper_stalled = false;
            self.gripper_high_torque_ticks = 0;
            self.gripper_low_torque_ticks = 0;
        }
        let macro_sub = self.macro_run.as_ref().is_some_and(|m| Some(m.id) == action_id);
        if let (Some(id), Some(a)) = (action_id, self.last_action.as_mut()) {
            if a.id == id && a.is_running() && !macro_sub { a.progress = out.progress; }
        }
        match out.status {
            StepStatus::Running => None,
            StepStatus::Done { settled, holding, gripper_gap_rad } => {
                let clear_releasing = self.releasing
                    && matches!(self.active_trajectory.as_ref().map(|a| &a.reference), Some(Reference::Gripper(_)));
                self.active_trajectory = None;
                self.commanded_velocities.fill(0.0);
                self.target_positions = self.commanded_positions.clone();
                Some(DeferredActionDone {
                    action_id,
                    macro_sub,
                    settled,
                    holding,
                    gripper_gap_rad,
                    clear_releasing,
                })
            }
            StepStatus::Failed { code, message } => {
                if code == "TRACKING_STALL" {
                    self.stop_motion();
                    self.tracking.invalidate();
                    if self.measured_positions.iter().all(|q| q.is_finite()) {
                        self.commanded_positions = self.measured_positions.clone();
                    }
                    self.commanded_accelerations.fill(0.0);
                } else {
                    self.active_trajectory = None;
                }
                self.commanded_velocities.fill(0.0);
                self.target_positions = self.commanded_positions.clone();
                if !self.pending_normal_unload { self.safety.mark_motion_finished(); }
                if macro_sub {
                    if let Some(m) = self.macro_run.as_mut() { m.sub = SubState::Failed { code: code.into(), message }; }
                } else if let Some(id) = action_id { self.finish_action(id, "failed", Some(code), Some(message), None, None, None); }
                None
            }
        }
    }

    /// Semi-implicit reference step: q_next = q + v_next * dt. All constraint
    /// arithmetic is f64; storage/CAN values remain f32 (and incur rounding).
    /// This bounds acceleration, not jerk. An infeasible initial state must
    /// take the fault/hold path rather than silently overriding acceleration.
    pub fn bounded_servo_step(
        q: f32,
        target: f32,
        v_prev: f32,
        dt: f64,
        limit: &crate::model::JointLimit,
    ) -> Option<(f32, f32, bool)> {
        Self::bounded_servo_step_with_jerk(q, target, v_prev, dt, limit, 0.0, None)
    }

    pub fn bounded_servo_step_with_jerk(
        q: f32, target: f32, v_prev: f32, dt: f64,
        limit: &crate::model::JointLimit, previous_acceleration: f32, max_jerk: Option<f32>,
    ) -> Option<(f32, f32, bool)> {
        const POS_EPS: f64 = 1.0e-3;

        if !q.is_finite() || !target.is_finite() || !v_prev.is_finite()
            || !dt.is_finite() || dt <= 0.0
            || !limit.max_velocity.is_finite() || limit.max_velocity <= 0.0
            || !limit.max_acceleration.is_finite() || limit.max_acceleration <= 0.0
            || !limit.min_position.is_finite() || !limit.max_position.is_finite()
            || limit.min_position >= limit.max_position
        {
            return None;
        }

        let lo = f64::from(limit.min_position);
        let hi = f64::from(limit.max_position);
        let q64 = f64::from(q);

        if q64 < lo - POS_EPS || q64 > hi + POS_EPS {
            return None;
        }

        let q_projected = q64.clamp(lo, hi);
        let h = dt;
        let vmax = f64::from(limit.max_velocity).min(2.0);
        let amax = f64::from(limit.max_acceleration);
        let dv = amax * h;
        let v_prev64 = f64::from(v_prev);

        if (q_projected <= lo && v_prev64 < -dv) || (q_projected >= hi && v_prev64 > dv) {
            return None;
        }

        // Rationalized positive solution of u*h + u^2/(2*a) <= distance.
        // The continuous stopping-distance term also covers variable dt.
        let safe_speed = |distance: f64| -> f64 {
            if distance <= 0.0 { return 0.0; }
            (2.0 * amax * distance
                / ((dv * dv + 2.0 * amax * distance).sqrt() + dv)).min(vmax)
        };

        let clamped_target = f64::from(target).clamp(lo, hi);
        let target_error = clamped_target - q_projected;
        let desired = if target_error.abs() <= 1.0e-5 {
            0.0
        } else {
            target_error.signum() * safe_speed(target_error.abs())
        };

        let lower = (v_prev64 - dv)
            .max(-safe_speed(q_projected - lo))
            .max(-vmax);
        let upper = (v_prev64 + dv)
            .min(safe_speed(hi - q_projected))
            .min(vmax);

        // Never collapse an empty interval to a midpoint: that would violate
        // at least one of the acceleration or stopping-distance constraints.
        if lower > upper { return None; }

        let preferred = if let Some(jerk) = max_jerk {
            if !jerk.is_finite() || jerk <= 0.0 || !previous_acceleration.is_finite() { return None; }
            let acceleration = ((desired - v_prev64) / h).clamp(-amax, amax);
            let previous = f64::from(previous_acceleration);
            let da = f64::from(jerk) * h;
            v_prev64 + acceleration.clamp(previous - da, previous + da) * h
        } else { desired };
        // Do not let acceleration history carry a stationary target past its braking
        // envelope. This also removes the small limit cycle that a pure jerk filter
        // creates near arrival. Changed targets can make this envelope infeasible;
        // the final projection still gives acceleration and joint limits priority.
        let preferred = if desired >= 0.0 && v_prev64 >= 0.0 {
            preferred.clamp(0.0, desired)
        } else if desired <= 0.0 && v_prev64 <= 0.0 {
            preferred.clamp(desired, 0.0)
        } else { preferred };
        // Safety bounds override the comfort preference when braking space is short.
        let v_next64 = preferred.clamp(lower, upper);
        let q_next64 = (q_projected + v_next64 * h).clamp(lo, hi);
        let v_final64 = if h > 0.0 {
            (q_next64 - q_projected) / h
        } else {
            0.0
        };

        let error = clamped_target - q_next64;
        let reached = error.abs() <= 1.0e-4 && v_final64.abs() <= 1.5e-2;

        Some((q_next64 as f32, v_final64 as f32, reached))
    }

    /// Write one rejection-fallback step (`tracking_fallback_compute`) and keep the braking
    /// ramp going while the output is not at rest. When no step is feasible,
    /// latch a fault and send no frame this tick; the command stays on the last issued
    /// output so the following fault-hold frames repeat what the motors already hold.
    /// Returns a driver write error.
    fn apply_tracking_fallback_step(
        &mut self,
        pending: crate::tracking::PendingFallback,
        gravity_output_enabled: bool,
    ) -> Option<String> {
        let crate::tracking::PendingFallback { step, ramp, prev_confirmed, other_depths } = pending;
        match step {
            Ok(fb) => {
                self.commanded_positions = fb.positions.clone();
                self.target_positions = fb.positions.clone();
                self.commanded_velocities = fb.velocities.clone();
                self.commanded_accelerations.fill(0.0);
                let targets: Vec<JointTarget> = fb
                    .positions
                    .iter()
                    .zip(fb.velocities.iter())
                    .enumerate()
                    .map(|(i, (&p, &v))| JointTarget {
                        position: p,
                        velocity: Some(v),
                        torque: if gravity_output_enabled {
                            self.gravity_torques_ramped.get(i).copied()
                        } else {
                            Some(0.0)
                        },
                        kp: None,
                        kd: None,
                    })
                    .collect();
                let write_res = self
                    .driver
                    .as_ref()
                    .and_then(|d| d.lock().ok())
                    .map(|mut driver| driver.write_joint_targets(&targets));
                match write_res {
                    Some(Ok(())) => {
                        let done = fb.done;
                        if done {
                            self.commanded_velocities.fill(0.0);
                        }
                        // Commit the step's escape depths to the workspace it was checked
                        // against; the other workspace's depths stay as they were.
                        let (full, park) = if ramp.keepout_only {
                            (other_depths, fb.ws_min_depths)
                        } else {
                            (fb.ws_min_depths, other_depths)
                        };
                        self.tracking.record_issued_with_ws_depths(fb.positions, fb.velocities, full, park);
                        self.tracking.fallback_ramp = (!done).then_some(ramp);
                        None
                    }
                    Some(Err(e)) => {
                        self.tracking.invalidate();
                        Some(e.to_string())
                    }
                    None => {
                        self.tracking.invalidate();
                        None
                    }
                }
            }
            Err(why) => {
                tracing::error!("Joint tracking fallback cannot brake within the limits ({why}): no frame sent");
                if let Some(pq) = prev_confirmed.as_deref().filter(|p| p.len() == self.commanded_positions.len()) {
                    let gripper_idx = self.config.gripper_index();
                    for (i, (c, &p)) in self.commanded_positions.iter_mut().zip(pq).enumerate() {
                        if Some(i) != gripper_idx {
                            *c = p;
                        }
                    }
                }
                self.target_positions = self.commanded_positions.clone();
                self.commanded_velocities.fill(0.0);
                self.commanded_accelerations.fill(0.0);
                self.tracking.invalidate();
                if !self.safety.state().is_stopped() {
                    self.safety.trigger_fault_coded(
                        FaultCode::ServoNoBrake,
                        FaultReason::Other(format!("Joint tracking fallback cannot brake within the limits: {why}")),
                    );
                }
                None
            }
        }
    }

    /// An already infeasible state cannot satisfy both acceleration and joint
    /// limits. Latch a fault and retain holding torque; normal smoothing does
    /// not apply to this exceptional safety interruption.
    fn latch_servo_fault(&mut self, joint_idx: usize) {
        self.fail_live_policy_from_control("SERVO_INFEASIBLE",
            format!("Joint {} has an infeasible servo braking state", joint_idx + 1));
        let limit = &self.config.limits[joint_idx];
        let reason = FaultReason::Other(format!(
            "Joint '{}' has an invalid or infeasible servo braking state (q={}, v={}, limits=[{}, {}], vmax={}, amax={})",
            limit.name, self.commanded_positions[joint_idx], self.commanded_velocities[joint_idx],
            limit.min_position, limit.max_position, limit.max_velocity, limit.max_acceleration,
        ));
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        self.servo_ws_min_depths = None;
        self.servo_stream_active = false;
        self.tracking.invalidate();
        self.active_trajectory = None;
        self.commanded_velocities.fill(0.0);
        self.target_positions = self.commanded_positions.clone();
        self.safety.trigger_fault_coded(FaultCode::ServoNoBrake, reason);
    }

    /// Gripper lead cap (`gripper_policy.max_lead_rad`): with torque on and fresh feedback the
    /// gripper command stays within the bound ahead of the measured position in the closing
    /// direction, whatever wrote it (macro
    /// gripper moves, servo streams, whole-arm references, gravity compensation), so a blocked
    /// jaw cannot build a large position error.
    fn gripper_lead_cap(&mut self) {
        let (Some(g), Some(lead)) = (self.config.gripper_index(), self.config.gripper_policy.as_ref().and_then(|p| p.max_lead_rad)) else { return; };
        if self.driver.is_none() || !self.is_torqued || !self.hardware_feedback_fresh(Duration::from_millis(100)) { return; }
        let (Some(&meas), Some(&cmd)) = (self.measured_positions.get(g), self.commanded_positions.get(g)) else { return; };
        if !meas.is_finite() || !cmd.is_finite() { return; }
        // closing direction only: an open ends on the open stop, nothing to crush there
        let dir = self.config.gripper_policy.as_ref().map_or(1.0, |p| if p.closed_rad >= p.open_rad { 1.0 } else { -1.0 });
        // stream contact regulator: the closing command stops at its limit
        let cmd = match self.gripper_stream_contact.filter(|c| self.gripper_stream_target.is_some() && c.limit_rad.is_finite()) {
            Some(c) if (cmd - c.limit_rad) * dir > 1e-6 => {
                let l = &self.config.limits[g];
                self.commanded_positions[g] = c.limit_rad.clamp(l.min_position, l.max_position);
                // the stepper restarts from rest at the limit (an open frame reverses at once)
                if let Some(v) = self.commanded_velocities.get_mut(g) { *v = 0.0; }
                self.note_motion_limit(MotionLimitReason::GripperForce, Some(g));
                self.commanded_positions[g]
            }
            _ => cmd,
        };
        if (cmd - meas) * dir > lead {
            let capped = meas + dir * lead;
            let l = &self.config.limits[g];
            self.commanded_positions[g] = capped.clamp(l.min_position, l.max_position);
        }
    }

    /// J7 stall freeze for whole-arm references. A joint quintic (park, home, presets), path or
    /// replay writes all seven joints each tick; without this a J7 target past an object in the
    /// jaw is driven at full motor torque (the stall latch alone is overwritten next tick, and
    /// re-arms after the torque drops, squeezing in steps). Once J7 is stalled, or an object is
    /// held, J7 stays at the hold point while the reference pushes further in the stall
    /// direction; a reference moving J7 the other way ends the freeze.
    fn gripper_hold_filter(&mut self, positions: &mut [f32], velocities: &mut [f32]) {
        let Some(g_idx) = self.config.gripper_index() else { return; };
        if positions.len() <= g_idx || self.commanded_positions.len() <= g_idx { return; }
        if self.gripper_motion_hold.is_none() {
            let close_sign = self.config.gripper_policy.as_ref()
                .map_or(1.0, |p| if p.closed_rad >= p.open_rad { 1.0 } else { -1.0 });
            if self.gripper_stalled {
                self.gripper_motion_hold = Some((self.commanded_positions[g_idx], self.gripper_stall_sign));
            } else if self.held.is_some() {
                self.gripper_motion_hold = Some((self.commanded_positions[g_idx], close_sign));
            }
        }
        let Some((hold, sign)) = self.gripper_motion_hold else { return; };
        let push = (positions[g_idx] - hold) * sign;
        if push > 0.0 {
            positions[g_idx] = hold;
            if velocities.len() > g_idx { velocities[g_idx] = 0.0; }
        } else if push < -1e-3 {
            self.gripper_motion_hold = None;
            return;
        }
        if self.pending_normal_unload {
            // a hold at the jaw's own closed stop is not an object
            let gap = self.config.gripper_policy.as_ref().map_or(f32::INFINITY, |p| (hold - p.closed_rad).abs() - p.holding_gap_rad);
            if gap >= 0.0 && !self.park_gripper_contact {
                tracing::info!("Park: gripper held at {hold:.3} rad (object in the jaw); releasing it at the resting pose");
                self.park_gripper_contact = true;
            }
        }
    }

    /// Release step of a park that carried an object, run once the park trajectory ends:
    /// open [`PARK_RELEASE_OPEN_RAD`] past the contact, then close (stall-protected gripper
    /// moves), then the unload settle. Returns true while the release owns the tick.
    fn step_park_release(&mut self) -> bool {
        let next = match self.park_release {
            None if self.park_gripper_contact => ParkRelease::Opening,
            Some(ParkRelease::Opening) => ParkRelease::Closing,
            Some(ParkRelease::Closing) => {
                self.park_release = None;
                self.park_gripper_contact = false;
                return false;
            }
            None => return false,
        };
        let Some(gripper_idx) = self.config.gripper_index() else {
            self.park_gripper_contact = false;
            return false;
        };
        let Some(policy) = self.config.gripper_policy.clone() else {
            self.park_gripper_contact = false;
            return false;
        };
        let (command, fraction) = match next {
            ParkRelease::Opening => {
                let open_sign = if policy.open_rad >= policy.closed_rad { 1.0 } else { -1.0 };
                let target = self.commanded_positions[gripper_idx] + open_sign * PARK_RELEASE_OPEN_RAD;
                let span = policy.open_rad - policy.closed_rad;
                let f = if span.abs() > 1e-6 { ((target - policy.closed_rad) / span).clamp(0.0, 1.0) } else { 1.0 };
                (GripperCommand::Move, Some(f64::from(f)))
            }
            ParkRelease::Closing => (GripperCommand::Close, None),
        };
        let request = GripperRequest { command, fraction, grasp: false, speed: PARK_RELEASE_SPEED, timeout_s: None, tag: None };
        // the stall latch of the carry would block the opening move's first ticks otherwise
        self.gripper_stalled = false;
        self.gripper_high_torque_ticks = 0;
        self.gripper_low_torque_ticks = 0;
        self.gripper_motion_hold = None;
        if next == ParkRelease::Opening { self.releasing = true; }
        let motion = GripperMotion::new(request, policy, self.config.limits[gripper_idx].clone(), self.commanded_positions[gripper_idx], gripper_idx);
        let duration = motion.expected_duration();
        tracing::info!("Park release: {} joint {} {:.3} -> {:.3} rad", if next == ParkRelease::Opening { "opening" } else { "closing" },
            gripper_idx + 1, self.commanded_positions[gripper_idx], motion.target);
        if let Some(park) = self.watchdog_park.as_mut() { park.budget_s += motion.timeout_s.max(duration); }
        self.target_positions[gripper_idx] = motion.target;
        self.active_trajectory = Some(ActiveTrajectory::new(None, Reference::Gripper(Box::new(motion))));
        self.park_release = Some(next);
        true
    }

    /// One control tick of the unload settle: the static resting-pose conditions must hold
    /// continuously for [`UNLOAD_SETTLE_HOLD`] and the arm must be stationary over that
    /// window ([`Self::unload_still_violation`]); then torque is cut. A settle pending past
    /// the hold logs which condition blocks it, at most once per second.
    fn step_unload_settle(&mut self, dt: f64) {
        let mut settle = self.unload_settle.take().unwrap_or_default();
        settle.pending_s += dt;
        let static_violation = self.resting_pose_static_violation();
        if static_violation.is_none() {
            let since = *settle.held_since_s.get_or_insert(settle.pending_s);
            settle.held_s = settle.pending_s - since;
            settle.samples.push_back((settle.pending_s, self.measured_positions.clone(), self.measured_velocities.clone()));
            let oldest = settle.pending_s - UNLOAD_SETTLE_HOLD.as_secs_f64() - 1e-6;
            while settle.samples.front().is_some_and(|(t, _, _)| *t < oldest) { settle.samples.pop_front(); }
        } else {
            settle.held_s = 0.0;
            settle.held_since_s = None;
            settle.samples.clear();
        }
        let blocker = static_violation.or_else(|| self.unload_still_violation(&settle));
        let Some(blocker) = blocker else {
            self.pending_normal_unload = false;
            tracing::info!("Arm settled cleanly at home position for {} ms after {:.2} s; safely cutting torque to Standby/Idle.",
                UNLOAD_SETTLE_HOLD.as_millis(), settle.pending_s);
            let watchdog_park = self.watchdog_park.take().is_some();
            match self.torque_off(true) {
                Ok(_) if watchdog_park => tracing::info!("Watchdog park complete: torque off at resting pose"),
                Ok(_) => {}
                Err(e) => {
                    tracing::warn!("Failed to cut torque after normal unload settle: {}", e);
                    self.safety.mark_motion_finished();
                    if watchdog_park && self.is_torqued && !self.safety.state().is_stopped() {
                        tracing::warn!("Watchdog park failed at torque off; holding torque and latching WatchdogTimeout");
                        self.latch_watchdog_fault();
                    }
                }
            }
            return;
        };
        let report_after = UNLOAD_SETTLE_HOLD.as_secs_f64() + UNLOAD_SETTLE_REPORT_AFTER_S;
        if settle.pending_s >= report_after && settle.last_report_s.is_none_or(|t| settle.pending_s - t >= 1.0) {
            tracing::warn!("Unload settle pending for {:.1} s: {blocker}", settle.pending_s);
            settle.last_report_s = Some(settle.pending_s);
        }
        // Not yet stably at rest: hold the home target. Gripper keeps its command: driving it to the
        // home value would squeeze anything still in the jaw at full motor torque.
        let mut home = self.config.resting_positions().to_vec();
        if let Some(g_idx) = self.config.gripper_index() {
            if home.len() > g_idx && self.commanded_positions.len() > g_idx { home[g_idx] = self.commanded_positions[g_idx]; }
        }
        self.commanded_positions = home.clone();
        self.target_positions = home;
        self.commanded_velocities = vec![0.0; self.config.dof];
        self.unload_settle = Some(settle);
    }

    /// Blocker of a pending unload settle, for watchdog reporting.
    fn unload_settle_blocker(&self) -> Option<String> {
        if self.active_trajectory.is_some() { return Some("park trajectory still running".into()); }
        self.resting_pose_static_violation()
            .or_else(|| self.unload_settle.as_ref().and_then(|s| self.unload_still_violation(s)))
    }

    pub fn tick(&mut self, dt: f64) -> ArmObservation {
        self.consume_pending_estop();
        // Immediate atomic e-stop latch check before any processing
        if self.e_stop_latched.load(Ordering::SeqCst) && !self.safety.state().is_stopped() {
            self.apply_stop_policy(StopPolicy::EStopLatch);
        }

        // Polling a cached sample never refreshes its hardware receive timestamp.
        let mut invalid_feedback = false;
        let mut motor_errors = None;
        let mut driver_torque_lost = false;
        let mut recovered_joints = Vec::new();
        let mut recovery_ages: Option<Vec<Option<Duration>>> = None;
        if let Some(ref d) = self.driver {
            self.feedback_numeric_valid = false;
            if let Ok(mut driver) = d.lock() {
                if let Ok(states) = driver.read_joint_states() {
                    recovered_joints = driver.take_recovery_writes();
                    let loads = driver.joint_loads();
                    self.feedback_numeric_valid = Self::states_valid(&states, driver.capabilities(), self.config.dof, loads.as_deref());
                    invalid_feedback = !self.feedback_numeric_valid;
                    let ages = driver.feedback_ages();
                    if !recovered_joints.is_empty() || self.tracking.recovery_suspended {
                        recovery_ages = Some(ages.clone());
                    }
                    if self.feedback_numeric_valid && ages.len() == self.config.dof && driver.is_feedback_valid() {
                        self.last_can_feedback_instant = ages.iter().copied().collect::<Option<Vec<_>>>()
                            .and_then(|ages| ages.into_iter().max()).and_then(|age| Instant::now().checked_sub(age));
                    } else { self.last_can_feedback_instant = None; }
                    if self.feedback_numeric_valid {
                        Self::apply_feedback(
                            &mut self.measured_positions,
                            &mut self.measured_velocities,
                            &mut self.torques,
                            &mut self.temperatures,
                            &mut self.loads,
                            self.config.dof,
                            &states,
                            loads.as_deref(),
                        );
                    }
                }
                if let Some(status) = driver.motor_status() {
                    motor_errors = Some(status.into_iter().map(|code| {
                        driver.decode_motor_error(code).map(|name| crate::model::MotorError { code, name })
                    }).collect());
                }
                if self.is_torqued && !self.disable_pending && self.gravity_comp.is_none() && self.safety.state().is_operational()
                    && driver.capabilities().enable_feedback && driver.is_feedback_valid() && driver.enable_state() != EnableState::Enabled
                    && !(self.pending_rearm && driver.has_known_partial_enable()) {
                    driver_torque_lost = true;
                }
            }
        }
        if let Some(errors) = motor_errors {
            self.update_motor_errors(errors);
        }
        // B1: autonomous driver recovery writes (e.g. Damiao 45 ms POS_VEL hold) invalidate
        // tracking output history and suspend compensated writes until fresh post-recovery
        // feedback confirms a clean baseline across every joint.
        if self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled) {
            if !recovered_joints.is_empty() {
                tracing::warn!(
                    "Driver emitted autonomous recovery writes for joints {:?}; suspending tracking compensation until fresh baseline",
                    recovered_joints.iter().map(|i| i + 1).collect::<Vec<_>>()
                );
                self.tracking.suspend_for_recovery(&recovered_joints, Instant::now());
                self.stop_motion();
                self.servo_stream_active = false;
            } else if self.tracking.recovery_suspended && self.feedback_numeric_valid {
                const RECOVERY_FRESH_BOUND: Duration = Duration::from_millis(45);
                let all_fresh_post_recovery = recovery_ages.as_ref().is_some_and(|ages| {
                    ages.len() == self.config.dof
                        && ages.iter().all(|a| a.is_some_and(|age| age < RECOVERY_FRESH_BOUND))
                        && self.tracking.recovery_joints.iter().all(|&(j, since)| {
                            ages.get(j)
                                .copied()
                                .flatten()
                                .is_some_and(|age| age <= since.elapsed())
                        })
                }) && self.measured_positions.len() == self.config.dof
                    && self.measured_positions.iter().all(|q| q.is_finite());
                if all_fresh_post_recovery {
                    let gripper_idx = self.config.gripper_index();
                    let recovered: Vec<usize> = self
                        .tracking
                        .recovery_joints
                        .iter()
                        .map(|&(j, _)| j)
                        .collect();
                    let mut recovered_updates = Vec::with_capacity(recovered.len());
                    for &j in &recovered {
                        if j < self.config.dof && Some(j) != gripper_idx {
                            let l = &self.config.limits[j];
                            let clamped = self.measured_positions[j].clamp(l.min_position, l.max_position);
                            self.commanded_positions[j] = clamped;
                            self.target_positions[j] = clamped;
                            recovered_updates.push((j, clamped));
                        }
                    }
                    self.commanded_velocities.fill(0.0);
                    self.commanded_accelerations.fill(0.0);
                    self.tracking.resume_from_recovery(&recovered_updates);
                }
            }
        }
        if invalid_feedback && self.is_torqued && !self.safety.state().is_stopped() {
            self.stop_motion();
            self.tracking.invalidate();
            self.commanded_velocities.fill(0.0);
            self.safety.trigger_fault_coded(FaultCode::InvalidTelemetry, FaultReason::Other("Invalid hardware telemetry".into()));
        } else if driver_torque_lost && self.is_torqued && !self.safety.state().is_stopped() {
            self.stop_motion();
            self.tracking.invalidate();
            if self.measured_positions.iter().all(|p| p.is_finite()) {
                self.commanded_positions = self.measured_positions.clone();
                self.target_positions = self.measured_positions.clone();
            }
            self.commanded_velocities.fill(0.0);
            self.commanded_accelerations.fill(0.0);
            self.safety.trigger_fault_coded(
                FaultCode::HardwareFault,
                FaultReason::Other("Unexpected motor torque loss detected while operational".into()),
            );
        }

        // A single late tick only shortens integration (dt is clamped below); the motors
        // keep holding their last POS_VEL target meanwhile. Fault on a real stall, using
        // CONTROL_DEADLINE (the same 250 ms bound as the hardware feedback deadline).
        if (!dt.is_finite() || dt <= 0.0 || dt > CONTROL_DEADLINE.as_secs_f64())
            && self.is_torqued && self.safety.state().is_operational() {
            self.stop_motion();
            self.tracking.invalidate();
            if self.measured_positions.iter().all(|p| p.is_finite()) {
                self.commanded_positions = self.measured_positions.clone();
                self.target_positions = self.measured_positions.clone();
            }
            self.commanded_velocities.fill(0.0);
            self.commanded_accelerations.fill(0.0);
            self.safety.trigger_fault_coded(FaultCode::ControlDeadline, FaultReason::Other("Control loop deadline exceeded".into()));
        }
        self.step_zero_convention();
        let dt = if dt.is_finite() && dt > 0.0 { dt.min(0.080) } else { 0.020 };
        let before_velocities = self.commanded_velocities.clone();

        // 0.1 Gripper stall protection state machine
        if self.active_trajectory.is_some() || self.macro_run.is_some() || self.pending_normal_unload
            || self.gravity_comp.is_some() || self.last_motion_frame.elapsed() > self.motion_frame_timeout {
            self.gripper_stream_target = None;
        }
        if let Some(g_idx) = self.config.gripper_index() {
            let (is_bus_fresh, has_torque_feedback, has_load_feedback) = if let Some(ref d) = self.driver {
                if let Ok(driver) = d.lock() {
                    let fresh = self.last_can_feedback_instant.is_some_and(|t| t.elapsed() < Duration::from_millis(50)) && driver.is_feedback_valid();
                    let caps = driver.capabilities();
                    (fresh, caps.torque_feedback, caps.load_feedback)
                } else {
                    (false, false, false)
                }
            } else {
                (true, true, false) // In simulation without a physical driver
            };

            let tau_g = self.torques.get(g_idx).copied().unwrap_or(0.0);
            let load_g = self.loads.get(g_idx).copied().unwrap_or(0.0);
            let p_cmd = self.commanded_positions.get(g_idx).copied().unwrap_or(0.0);
            let p_meas = self.measured_positions.get(g_idx).copied().unwrap_or(0.0);
            let v_g = self.measured_velocities.get(g_idx).copied().unwrap_or(0.0);
            let gap = p_cmd - p_meas;

            let gripper_hw_overload = is_bus_fresh
                && self
                    .motor_errors
                    .get(g_idx)
                    .and_then(|e| e.as_ref())
                    .is_some_and(|e| (e.code & 0x20) != 0);

            let hw_overload_rising = is_bus_fresh && gripper_hw_overload && !self.gripper_prev_hw_overload;
            if is_bus_fresh {
                self.gripper_prev_hw_overload = gripper_hw_overload;
            }

            if !gripper_hw_overload && !self.gripper_stalled {
                self.gripper_stall_sign = 0.0;
            }

            let desired_g = self.servo_target.as_ref().and_then(|st| st.get(g_idx).copied())
                .or_else(|| self.target_positions.get(g_idx).copied())
                .unwrap_or(p_cmd);
            let close_dir = self.config.gripper_policy.as_ref()
                .map(|p| if p.closed_rad < p.open_rad { -1.0_f32 } else { 1.0_f32 })
                .unwrap_or(1.0_f32);
            let pushing_into_stall = self.gripper_stall_sign != 0.0
                && ((desired_g - p_cmd) * self.gripper_stall_sign > 1e-4
                    || (desired_g - p_meas) * self.gripper_stall_sign > 1e-2);
            let escaping_known_stall = self.gripper_stall_sign != 0.0
                && (desired_g - p_cmd) * self.gripper_stall_sign < -1e-4;
            let should_latch_hw_overload = if hw_overload_rising {
                !escaping_known_stall
            } else {
                self.gripper_stall_sign == 0.0 || pushing_into_stall
            };

            // Stream contact regulator (gripper_policy.stream_contact_speed_rad_s): from the contact
            // the closing command creeps at the contact speed while |τ| < contact_torque_nm and
            // its lead over the jaw is scaled by contact_torque_nm / |τ| above it, so the grip
            // settles near contact_torque_nm whatever the stiffness. The impact sample itself is
            // stiffness x the POS_VEL lag at contact: no feedback precedes it.
            let regulator = self.config.gripper_policy.as_ref()
                .and_then(|p| Some((p.contact_torque_nm?, p.stream_contact_speed_rad_s?)))
                .filter(|_| has_torque_feedback && self.is_torqued);
            match (regulator, self.gripper_stream_target) {
                (Some((target_nm, speed)), Some(stream_g)) if is_bus_fresh && p_meas.is_finite() && p_cmd.is_finite() => {
                    let tau = tau_g.abs();
                    self.gripper_stream_contact = match self.gripper_stream_contact {
                        None => {
                            let pushing = (p_cmd - p_meas) * close_dir > 1e-3 || (stream_g - p_meas) * close_dir > 1e-3;
                            let still = tau >= GRIPPER_STALL_LATCH_NM && v_g.abs() < 0.10;
                            (pushing && (tau >= target_nm || still)).then(|| {
                                let lead = ((p_cmd - p_meas) * close_dir).max(0.0);
                                let scale = if tau > target_nm { target_nm / tau } else { 1.0 };
                                GripperStreamContact { at_rad: p_meas, limit_rad: p_meas + close_dir * lead * scale }
                            })
                        }
                        // the jaw opened past the contact point, or closed on with little torque
                        Some(c) if (p_meas - c.at_rad) * close_dir <= -GRIPPER_FORCE_RELEASE_MARGIN_RAD => None,
                        Some(c) if tau < 0.5 * target_nm && (p_meas - c.at_rad) * close_dir > GRIPPER_STREAM_CONTACT_LOST_RAD => None,
                        Some(mut c) => {
                            if tau >= target_nm {
                                let lead = ((c.limit_rad - p_meas) * close_dir).max(0.0);
                                c.limit_rad = p_meas + close_dir * lead * (target_nm / tau);
                            } else {
                                // at most one step ahead of what was written last tick
                                let base = if (c.limit_rad - p_cmd) * close_dir > 0.0 { p_cmd } else { c.limit_rad };
                                c.limit_rad = base + close_dir * speed * dt as f32;
                            }
                            Some(c)
                        }
                    };
                }
                (Some(_), Some(_)) => {} // late feedback: keep the limit (gripper_lead_cap needs fresh feedback)
                _ => self.gripper_stream_contact = None,
            }

            // Torque cap: |τ| above max_close_torque_nm while the jaw is driven closed latches
            // the stall whatever the jaw speed (a deforming object or a flexing finger keeps
            // the jaw moving, so the stall rule below never sees it still; 2026-11-20 a finger
            // broke at 5 N·m). Every mode reads gripper_stalled: macro gripper moves, servo
            // streams (teleop, policies) and whole-arm references (gripper_hold_filter).
            if !self.gripper_stalled { self.gripper_force_capped = false; }
            let v_cmd = self.commanded_velocities.get(g_idx).copied().unwrap_or(0.0);
            let closing = (p_cmd - p_meas) * close_dir > 1e-3 || (desired_g - p_meas) * close_dir > 1e-3 || v_cmd * close_dir > 1e-3;
            let force_cap = self.config.gripper_policy.as_ref().and_then(|p| p.max_close_torque_nm);
            let over_cap = is_bus_fresh && has_torque_feedback && closing && force_cap.is_some_and(|cap| tau_g.abs() > cap);
            self.gripper_force_ticks = if over_cap { self.gripper_force_ticks.saturating_add(1) } else { 0 };

            if !is_bus_fresh {
                self.gripper_high_torque_ticks = 0;
                self.gripper_low_torque_ticks = 0;
            } else if self.gripper_force_ticks >= GRIPPER_FORCE_CAP_TICKS {
                // latch, or refresh a latch that still presses, at the measured position
                if !(self.gripper_stalled && self.gripper_force_capped) {
                    self.gripper_force_trips += 1;
                    tracing::warn!("Gripper torque cap: |tau| {:.2} N·m above {:.2} N·m at {p_meas:.3} rad; holding",
                        tau_g.abs(), force_cap.unwrap_or(0.0));
                }
                self.gripper_stalled = true;
                self.gripper_force_capped = true;
                self.gripper_force_ticks = 0;
                self.gripper_stall_sign = close_dir;
                self.gripper_high_torque_ticks = 0;
                self.gripper_low_torque_ticks = 0;
                if p_meas.is_finite() {
                    if g_idx < self.commanded_positions.len() { self.commanded_positions[g_idx] = p_meas; }
                    if g_idx < self.commanded_velocities.len() { self.commanded_velocities[g_idx] = 0.0; }
                    if g_idx < self.target_positions.len() { self.target_positions[g_idx] = p_meas; }
                    if let Some(ref mut st) = self.servo_target {
                        if st.len() > g_idx { st[g_idx] = p_meas; }
                    }
                    if let Some(h) = self.gripper_motion_hold.as_mut() { *h = (p_meas, close_dir); }
                }
                self.note_motion_limit(MotionLimitReason::GripperForce, Some(g_idx));
            } else if !self.gripper_stalled && gripper_hw_overload && should_latch_hw_overload {
                self.gripper_stalled = true;
                let dir = if gap.abs() > 1e-3 {
                    gap
                } else if (desired_g - p_meas).abs() > 1e-3 {
                    desired_g - p_meas
                } else if has_torque_feedback && tau_g.abs() > 1e-3 {
                    tau_g
                } else if has_load_feedback && load_g.abs() > 1e-3 {
                    load_g
                } else if self.gripper_stall_sign != 0.0 {
                    self.gripper_stall_sign
                } else {
                    close_dir
                };
                self.gripper_stall_sign = if dir >= 0.0 { 1.0 } else { -1.0 };
                self.gripper_low_torque_ticks = 0;
                self.gripper_high_torque_ticks = 0;
                if let Some(&hold) = self.measured_positions.get(g_idx) {
                    if g_idx < self.commanded_positions.len() {
                        self.commanded_positions[g_idx] = hold;
                    }
                    if g_idx < self.target_positions.len() {
                        self.target_positions[g_idx] = hold;
                    }
                    if let Some(ref mut st) = self.servo_target {
                        if st.len() > g_idx {
                            st[g_idx] = hold;
                        }
                    }
                }
            } else if !self.gripper_stalled && !gripper_hw_overload {
                let is_stalled = if has_torque_feedback {
                    // the stream contact regulator holds the grip instead of the stall hold at
                    // the measured jaw (which reads ~0 N·m on a rigid object and re-squeezes)
                    self.gripper_stream_contact.is_none()
                        && tau_g.abs() >= GRIPPER_STALL_LATCH_NM && v_g.abs() < 0.10
                } else {
                    let stall_load_thresh = self.config.gripper_policy.as_ref()
                        .filter(|p| p.holding_mode == crate::model::HoldingMode::LoadAndGap)
                        .and_then(|p| p.holding_load_fraction)
                        .map(|frac| (frac * 1.25).clamp(0.18, 0.35))
                        .unwrap_or(0.35);
                    let load_condition = if has_load_feedback { load_g.abs() > stall_load_thresh } else { true };
                    gap.abs() > 0.08 && v_g.abs() < 0.05 && load_condition
                };
                let required_ticks = if has_torque_feedback { 6 } else { 10 };

                if is_stalled {
                    self.gripper_high_torque_ticks += 1;
                    if self.gripper_high_torque_ticks >= required_ticks {
                        self.gripper_stalled = true;
                        let dir = if gap.abs() > 1e-3 { gap } else { tau_g };
                        self.gripper_stall_sign = if dir >= 0.0 { 1.0 } else { -1.0 };
                        self.gripper_low_torque_ticks = 0;
                        if let Some(&hold) = self.measured_positions.get(g_idx) {
                            if g_idx < self.commanded_positions.len() { self.commanded_positions[g_idx] = hold; }
                            if g_idx < self.target_positions.len() { self.target_positions[g_idx] = hold; }
                            if let Some(ref mut st) = self.servo_target {
                                if st.len() > g_idx {
                                    st[g_idx] = hold;
                                }
                            }
                        }
                    }
                } else {
                    self.gripper_high_torque_ticks = 0;
                }
            } else if self.gripper_stalled && self.gripper_force_capped && self.gripper_stream_target.is_some() {
                // torque-cap latch under a stream: held until the stream opens (intake) or ends
                self.gripper_high_torque_ticks = 0;
                self.gripper_low_torque_ticks = 0;
            } else if self.gripper_stalled {
                self.gripper_high_torque_ticks = 0;
                let is_released = if has_torque_feedback {
                    tau_g.abs() < GRIPPER_STALL_RELEASE_NM
                } else {
                    gap * self.gripper_stall_sign < -0.005
                };
                if is_released {
                    self.gripper_low_torque_ticks += 1;
                    if self.gripper_low_torque_ticks >= 6 {
                        self.gripper_stalled = false;
                        self.gripper_low_torque_ticks = 0;
                    }
                } else {
                    self.gripper_low_torque_ticks = 0;
                }
            } else {
                self.gripper_high_torque_ticks = 0;
                self.gripper_low_torque_ticks = 0;
            }
        }

        // 0.2 Compute gravity torques if gravity compensation is enabled, driver feedback is valid, and measured positions available
        let driver_feedback_ok = self.hardware_feedback_fresh(Duration::from_millis(250));

        // The model torques are computed whenever the profile has a gravity model (for the
        // residuals); they drive the output only when gravity output is enabled.
        let arm_dof = self.config.arm_dof();
        let model_gravity = if self.config.gravity_model.is_some() && self.config.dof >= arm_dof
            && driver_feedback_ok && self.measured_positions.len() >= arm_dof {
            match self.config.arm_model.gravity_torques(&self.measured_positions[..arm_dof]) {
                Ok(tau) if tau.iter().all(|t| t.is_finite()) => Some(tau),
                Ok(_) => {
                    tracing::warn!("Gravity torque contains non-finite values, zeroing out");
                    None
                }
                Err(e) => {
                    tracing::warn!("Gravity torque computation error: {:?}", e);
                    None
                }
            }
        } else { None };
        match model_gravity.filter(|_| self.gravity_output_enabled()) {
            Some(tau) => { self.gravity_torques.clear(); self.gravity_torques.extend_from_slice(&tau[..arm_dof]); }
            None => self.gravity_torques.clear(),
        }
        let torque_feedback = self.driver.as_ref().is_some_and(|d| d.lock().is_ok_and(|drv| drv.capabilities().torque_feedback));
        self.collision.torque_feedback = torque_feedback;
        // Residuals and the collision guard are per joint of a 6-joint arm.
        self.residual_torques = model_gravity.filter(|_| arm_dof == 6 && torque_feedback && self.feedback_numeric_valid && self.torques.len() >= 6)
            .map(|tau| std::array::from_fn(|i| self.torques[i] - tau[i]))
            .filter(|r: &[f32; 6]| r.iter().all(|v| v.is_finite()));

        // Apply Slew-Rate Limiter (15.0 N·m/s) and safety limits to gravity torques
        if self.gravity_torques_ramped.len() != self.config.dof {
            self.gravity_torques_ramped = vec![0.0; self.config.dof];
        }
        let dt_sec = (dt as f32).clamp(0.001, 0.1);
        let max_dtau = 15.0 * dt_sec; // Slew rate: 15.0 N·m/s (0.30 N·m/tick at 50Hz)
        for i in 0..self.config.dof {
            let target_tau = if self.gravity_output_enabled() && driver_feedback_ok && i < self.gravity_torques.len() {
                let raw = self.gravity_torques[i];
                let raw_finite = if raw.is_finite() { raw } else { 0.0 };
                let scaled = raw_finite * self.gravity_scale;
                let limit = if i < 3 { 12.0 } else { 6.0 };
                scaled.clamp(-limit, limit)
            } else {
                0.0
            };
            let diff = target_tau - self.gravity_torques_ramped[i];
            let ramped = self.gravity_torques_ramped[i] + diff.clamp(-max_dtau, max_dtau);
            self.gravity_torques_ramped[i] = if ramped.is_finite() { ramped } else { 0.0 };
        }

        // Bus loss: no fresh feedback from every joint within BUS_LOSS_TIMEOUT.
        if self.is_torqued
            && !self.safety.state().is_stopped()
            && self.driver.is_some()
            && !self.hardware_feedback_fresh(BUS_LOSS_TIMEOUT)
        {
            self.execute_stop(StopPolicy::BusLossFallback);
        }
        self.sync_torque_state_after_stop();

        // Check hardware motor faults / error flags from feedback
        if self.is_torqued && !self.safety.state().is_stopped() {
            if let Some(ref d) = self.driver {
                if let Ok(driver) = d.lock() {
                    if let Err(e) = driver.check_hardware_status() {
                        tracing::error!("Hardware fault detected on driver: {}", e);
                        let hold_pos = if self.measured_positions.iter().all(|p| p.is_finite())
                            && self.measured_positions.len() == self.config.dof
                        {
                            self.measured_positions.clone()
                        } else {
                            self.commanded_positions.clone()
                        };
                        self.pending_normal_unload = false;
                        self.unload_settle = None;
                        self.active_trajectory = None;
                        self.servo_target = None; self.servo_settled_ticks.fill(0);
                        self.servo_ws_min_depths = None;
                        self.tracking.invalidate();
                        self.target_positions = hold_pos.clone();
                        self.commanded_positions = hold_pos;
                        self.commanded_velocities = vec![0.0; self.config.dof];
                        self.safety.trigger_fault_coded(FaultCode::HardwareFault, FaultReason::Other(e.to_string()));
                    }
                }
            }
        }

        // 0.3 Gravity compensation: exits (to a hold) on any safety event before the
        // watchdog below may start a park, which then runs from position mode.
        self.gravity_comp_tick(dt);

        // 1. Check lease watchdog (skip during autonomous stow/unload)
        if self.lease_enabled
            && self.gravity_comp.is_none()
            && self.is_torqued
            && self.safety.state().is_operational()
            && !self.pending_normal_unload
            && self.last_heartbeat.elapsed() > self.lease_timeout
        {
            #[cfg(feature = "policy-runtime")]
            self.terminate_policy_run("WATCHDOG_TIMEOUT", "failed", Some("WATCHDOG_TIMEOUT"), Some("Lease watchdog expired".into()));
            // Watchdog expired with torque on. When the arm is healthy and has a resting
            // pose, run the normal unload (park at home, settle, torque off). Otherwise
            // rigid lock at the current position and latch the fault, keeping torque on.
            match self.watchdog_park_blocker() {
                None => {
                    tracing::warn!("Watchdog: lease expired while torqued, parking");
                    self.apply_stop_policy(StopPolicy::NormalUnload);
                    if self.pending_normal_unload {
                        let budget = self.watchdog_park_timeout();
                        tracing::info!("Watchdog park budget {:.2} s (planned {:.2} s)", budget.as_secs_f64(), self.unload_planned_duration.unwrap_or(0.0));
                        self.watchdog_park = Some(WatchdogPark { elapsed_s: 0.0, budget_s: budget.as_secs_f64(), grace_granted: false });
                    } else {
                        tracing::warn!("Watchdog park could not be planned; holding torque and latching WatchdogTimeout");
                        self.latch_watchdog_fault();
                    }
                }
                Some(reason) => {
                    tracing::warn!("Watchdog: lease expired while torqued, not parking ({reason}); holding torque and latching WatchdogTimeout");
                    self.latch_watchdog_fault();
                }
            }
        }

        // Watchdog park supervision: a park that stops without completing, or runs past
        // its budget, falls back to the hold + Fault(WatchdogTimeout) behaviour.
        self.supervise_watchdog_park(dt);

        // Thermal protection remains active while stopped or shutdown is unconfirmed.
        let temperature_available = self.driver.as_ref().is_none_or(|d| d.lock().is_ok_and(|drv| drv.capabilities().temperature_feedback));
        if (self.is_torqued || self.disable_pending) && temperature_available
            && self.temperatures.iter().any(|t| t.is_finite() && *t > self.config.max_temperature)
                && !matches!(self.safety.state(), SafetyState::Fault(FaultReason::ThermalTrip { .. })) {
                self.apply_stop_policy(StopPolicy::ThermalTrip);
            }
        if self.disable_pending {
            let confirmed = self.driver.as_ref().is_none_or(|d| d.lock().is_ok_and(|drv|
                drv.is_feedback_valid() && drv.enable_state() == EnableState::Disabled));
            if confirmed {
                self.is_torqued = false;
                self.pending_rearm = false;
                self.disable_pending = false;
            } else if self.last_disable_attempt.is_none_or(|t| t.elapsed() >= Duration::from_millis(250)) {
                if let Err(e) = self.request_disable() { tracing::warn!("Torque shutdown remains unconfirmed: {e}"); }
            }
        }
        // Detect sustained physical tracking error, never compare against stale samples.
        if self.driver.is_some() && self.is_torqued && self.safety.state().is_operational()
            && self.hardware_feedback_fresh(Duration::from_millis(100)) {
            let threshold = self.config.following_error.max_error_rad;
            // The 7-DOF gripper is excluded: Damiao POS_VEL trails its command by about
            // v / 2.2 rad (0.9 rad at 2 rad/s), and gripping an object leaves a lasting
            // error by design. Its own stall detection covers it.
            let gripper_idx = self.config.gripper_index();
            let outside = self.commanded_positions.iter().zip(&self.measured_positions).enumerate()
                .find(|(i, (q, measured))| Some(*i) != gripper_idx
                    && (!measured.is_finite() || (*q - *measured).abs() > threshold))
                .map(|(i, (q, measured))| (i, *q, *measured));
            if let Some((joint, cmd, meas)) = outside {
                let since = self.following_error_since.get_or_insert_with(Instant::now);
                if since.elapsed() >= Duration::from_millis(self.config.following_error.persistence_ms)
                    && self.servo_target.is_some() && self.measured_positions.iter().all(|q| q.is_finite()) {
                    // Streaming toward a target the joint cannot follow: a recoverable hold.
                    self.enter_servo_blocked_hold(joint, "tracking error above the bound");
                } else if since.elapsed() >= Duration::from_millis(self.config.following_error.persistence_ms) {
                    self.stop_motion();
                    self.tracking.invalidate();
                    self.commanded_positions = self.measured_positions.clone();
                    self.target_positions = self.measured_positions.clone();
                    self.commanded_velocities.fill(0.0);
                    self.commanded_accelerations.fill(0.0);
                    self.safety.trigger_fault_coded(FaultCode::FollowingError, FaultReason::Other(format!(
                        "Persistent joint tracking error: joint {} cmd {cmd:.3} meas {meas:.3} rad", joint + 1)));
                }
            } else { self.following_error_since = None; }
        } else { self.following_error_since = None; }

        // Use one integration interval and never integrate a reference twice in a tick.
        let servo_dt = dt.clamp(0.001, 0.1);

        // Collision guard: a trip stops the reference before it is stepped this tick.
        self.collision_tick(servo_dt);
        // Torque-saturation guard: same placement (src/torque_guard.rs).
        self.torque_guard_tick(servo_dt);
        // Supply-current estimate (src/power.rs).
        self.power_tick(servo_dt);
        let mut reference_stepped = false;

        // 3. Step trajectory if active, or step servo target
        if self.active_trajectory.is_none() && !self.pending_normal_unload { self.gripper_motion_hold = None; }
        if !self.pending_normal_unload { self.park_gripper_contact = false; self.park_release = None; }
        if self.pending_normal_unload {
            self.last_heartbeat = Instant::now();
        }
        // Action: the gripper reference (its own stall handling); Arm / Replay / Quintic move
        // J7 as part of a whole-arm reference and pass the J7 stall freeze.
        enum Stepped { Quintic(crate::trajectory::TrajectoryPoint, bool, f32), Action(StepOut), Arm(StepOut), Replay(StepOut, &'static str, f64) }
        let time_scale = self.motion_time_scale();
        let (torque_feedback, load_feedback) = if let Some(ref d) = self.driver {
            if let Ok(driver) = d.lock() {
                let caps = driver.capabilities();
                (caps.torque_feedback, caps.load_feedback)
            } else {
                (false, false)
            }
        } else {
            (true, false) // Default in simulation / without physical driver
        };
        let feedback_fresh = if self.driver.is_some() {
            driver_feedback_ok
        } else {
            true
        };
        let mut approach_started: Option<u64> = None;
        let stepped = if let Some(active) = self.active_trajectory.as_mut() {
            let ctx = StepCtx {
                dt: servo_dt,
                time_scale,
                measured: &self.measured_positions,
                measured_velocities: &self.measured_velocities,
                commanded: &self.commanded_positions,
                commanded_velocities: &self.commanded_velocities,
                torques: &self.torques,
                loads: &self.loads,
                feedback_fresh,
                torque_feedback,
                load_feedback,
                gripper_stalled: self.gripper_stalled,
                gripper_stall_sign: self.gripper_stall_sign,
                gripper_force_capped: self.gripper_stalled && self.gripper_force_capped,
            };
            let out = match &mut active.reference {
                Reference::Quintic(traj) => {
                    // power governor: the reference advances at dt x scale (velocities scaled with it)
                    let mut point = traj.step(dt * time_scale);
                    if time_scale < 1.0 { for v in point.velocities.iter_mut() { *v *= time_scale as f32; } }
                    Stepped::Quintic(point, traj.is_finished(), (traj.elapsed / traj.duration).min(1.0) as f32)
                }
                Reference::Path(path) => Stepped::Arm(path.step(&ctx)),
                Reference::Gripper(grip) => Stepped::Action(grip.step(&ctx)),
                Reference::Replay(replay) => {
                    let out = replay.step(&ctx);
                    Stepped::Replay(out, replay.phase_name(), replay.t_s())
                }
            };
            // same-side approach: progress over both legs; the first leg's end starts the short
            // segment (src/backlash.rs)
            let out = match (active.span.is_some(), out) {
                (true, Stepped::Quintic(p, f, g)) => Stepped::Quintic(p, f, active.overall_progress(g)),
                (true, Stepped::Arm(mut o)) => { o.progress = active.overall_progress(o.progress); Stepped::Arm(o) }
                (_, o) => o,
            };
            if active.first_leg_ended() && active.start_then() { approach_started = active.action_id; }
            Some((active.action_id, out))
        } else { None };
        if let Some(id) = approach_started {
            // macro actions (grasp, place) already have a phase `approach`: theirs is `final_approach`
            if self.macro_run.as_ref().is_some_and(|m| m.id == id) {
                let progress = self.last_action.as_ref().map_or(0.0, |a| a.progress);
                self.set_action_phase(id, "final_approach", progress, None, None);
            } else if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id && a.is_running()) {
                a.phase = Some("approach".into());
            }
        }
        let mut deferred_done: Option<DeferredActionDone> = None;
        match stepped {
            None => {}
            Some((action_id, Stepped::Action(out))) => {
                reference_stepped = true;
                self.servo_stream_active = false;
                deferred_done = self.apply_action_step(action_id, out);
            }
            Some((action_id, Stepped::Arm(mut out))) => {
                reference_stepped = true;
                self.servo_stream_active = false;
                self.gripper_hold_filter(&mut out.positions, &mut out.velocities);
                deferred_done = self.apply_action_step(action_id, out);
            }
            Some((action_id, Stepped::Replay(mut out, phase, t_s))) => {
                reference_stepped = true;
                self.servo_stream_active = false;
                self.gripper_hold_filter(&mut out.positions, &mut out.velocities);
                if let (Some(id), Some(a)) = (action_id, self.last_action.as_mut()) {
                    if a.id == id && a.is_running() {
                        a.phase = Some(phase.into());
                        a.t_s = Some(t_s as f32);
                    }
                }
                deferred_done = self.apply_action_step(action_id, out);
            }
            Some((action_id, Stepped::Quintic(mut point, finished, progress))) => {
                reference_stepped = true;
                self.servo_stream_active = false;
                self.gripper_hold_filter(&mut point.positions, &mut point.velocities);
                self.commanded_positions = point.positions;
                self.commanded_velocities = point.velocities;
                let macro_sub = self.macro_run.as_ref().is_some_and(|m| Some(m.id) == action_id);
                if let (Some(id), Some(a)) = (action_id, self.last_action.as_mut()) {
                    if a.id == id && a.is_running() && !macro_sub { a.progress = progress; }
                }
                if finished && approach_started.is_none() {
                    self.active_trajectory = None;
                    self.commanded_velocities = vec![0.0; self.config.dof];
                    if !self.pending_normal_unload {
                        self.safety.mark_motion_finished();
                        if let Some(id) = action_id { self.quintic_settle = Some((id, Settle::default())); }
                    }
                }
            }
        }

        // 3.1 Handle pending normal unload settling phase
        if self.pending_normal_unload && self.active_trajectory.is_none() {
            if !self.step_park_release() { self.step_unload_settle(servo_dt); }
        } else if let Some(ref target) = self.servo_target.clone() {
            // Check continuous motion frame timeout:
            // If > 200ms without a new pose frame, clear servo_target to begin controlled smooth deceleration
            if self.last_motion_frame.elapsed() > self.motion_frame_timeout {
                self.servo_target = None;
                self.servo_settled_ticks.fill(0);
                self.fail_live_policy_from_control("TARGET_TIMEOUT",
                    format!("No policy target for {} ms; the control loop dropped the target", self.motion_frame_timeout.as_millis()));
            } else {
                // Check if any target joint has instantaneous jump > 0.35 rad relative to commanded_positions
                // The 7-DOF gripper is exempt: its far targets are expected (POS_VEL needs a
                // lead to reach speed) and the per-joint servo step below still bounds its
                // velocity and acceleration.
                let gripper_idx = self.config.gripper_index();
                let has_dirty_jump = target.iter().enumerate().any(|(i, &tgt)| {
                    if Some(i) == gripper_idx {
                        false
                    } else if i < self.commanded_positions.len() {
                        let jump = (tgt - self.commanded_positions[i]).abs();
                        jump > 0.35
                    } else {
                        false
                    }
                });

                if has_dirty_jump {
                    tracing::warn!(
                        "Discarding dirty servo target frame: instantaneous jump exceeds 0.35 rad relative to commanded_positions"
                    );
                    self.servo_target = None;
                    self.servo_settled_ticks.fill(0);
                    self.fail_live_policy_from_control("SERVO_TARGET_DISCARDED",
                        "The control loop discarded the policy target: it leads the commanded position by more than 0.35 rad".into());
                } else {
                    reference_stepped = true;
                    let mut reached_flags = vec![false; self.config.dof];
                    let mut next_positions = self.commanded_positions.clone();
                    let mut next_velocities = self.commanded_velocities.clone();
                    let mut infeasible = None;
                    // Speed cap: joint limits capped by servo_max_velocity. Lead cap: in the direction
                    // of travel the stepper target stays within servo_max_lead_rad of the measured
                    // position (torque on, fresh feedback), so the tracking error stays below the bound
                    // whatever the client sends.
                    let mut step_limits = self.servo_limits();
                    // power governor: scale the velocity cap, never below what the joint can brake
                    // to in this tick (the stepper stays feasible); acceleration unchanged
                    if time_scale < 1.0 {
                        for (i, l) in step_limits.iter_mut().enumerate() {
                            if Some(i) == gripper_idx { continue; }
                            let brake = self.commanded_velocities.get(i).map_or(0.0, |v| v.abs()) - 0.9 * l.max_acceleration * servo_dt as f32;
                            l.max_velocity = (l.max_velocity * time_scale as f32).max(brake).max(1e-3);
                        }
                    }
                    let lead = self.config.servo_max_lead_rad;
                    let lead_gate = self.driver.is_some() && self.is_torqued
                        && self.hardware_feedback_fresh(Duration::from_millis(100));
                    let mut lead_capped = vec![false; self.config.dof];
                    let (mut lead_hit, mut speed_hit, mut limit_hit) = (None::<(usize, f32)>, None, None);
                    for (i, &tgt) in target.iter().enumerate() {
                        if i < self.config.dof {
                            let limit = &step_limits[i];
                            let clamped_tgt = tgt.clamp(limit.min_position, limit.max_position);
                            let mut actual_tgt = if Some(i) == gripper_idx && self.gripper_stalled {
                                self.commanded_positions.get(i).copied().unwrap_or(clamped_tgt)
                            } else {
                                clamped_tgt
                            };
                            let meas = self.measured_positions[i];
                            if lead_gate && Some(i) != gripper_idx && meas.is_finite() {
                                let q = next_positions[i];
                                if actual_tgt > q && actual_tgt > meas + lead {
                                    actual_tgt = meas + lead;
                                    lead_capped[i] = true;
                                } else if actual_tgt < q && actual_tgt < meas - lead {
                                    actual_tgt = meas - lead;
                                    lead_capped[i] = true;
                                }
                            }
                            let step = Self::bounded_servo_step_with_jerk(
                                next_positions[i],
                                actual_tgt,
                                next_velocities[i],
                                servo_dt,
                                limit,
                                self.commanded_accelerations[i],
                                self.config.max_jerk,
                            );
                            let Some((q_next, v_next, reached)) = step else {
                                infeasible = Some(i);
                                break;
                            };
                            // The lead cap shapes this tick only when the unclamped target would
                            // step differently (otherwise the speed cap or the limits bind).
                            let lead_shaping = lead_capped[i] && Self::bounded_servo_step_with_jerk(
                                next_positions[i], clamped_tgt, next_velocities[i], servo_dt, limit,
                                self.commanded_accelerations[i], self.config.max_jerk,
                            ).is_none_or(|(_, v_raw, _)| (v_raw - v_next).abs() > 1.0e-4);
                            next_positions[i] = q_next;
                            next_velocities[i] = v_next;
                            // a lead-capped joint has not reached the client's target
                            reached_flags[i] = reached && !lead_capped[i];
                            let raw = &self.config.limits[i];
                            let excess = (clamped_tgt - actual_tgt).abs();
                            if lead_shaping && lead_hit.is_none_or(|(_, e)| excess > e) {
                                lead_hit = Some((i, excess));
                            } else if !lead_shaping && speed_hit.is_none()
                                && limit.max_velocity < raw.max_velocity
                                && v_next.abs() >= limit.max_velocity - 1.0e-3 {
                                speed_hit = Some(i);
                            }
                            if Some(i) != gripper_idx && limit_hit.is_none()
                                && ((clamped_tgt >= raw.max_position - 1.0e-3 && q_next >= raw.max_position - 5.0e-3)
                                    || (clamped_tgt <= raw.min_position + 1.0e-3 && q_next <= raw.min_position + 5.0e-3)) {
                                limit_hit = Some(i);
                            }
                        }
                    }
                    if let Some(i) = limit_hit { self.note_motion_limit(MotionLimitReason::JointLimit, Some(i)); }
                    if let Some((i, _)) = lead_hit { self.note_motion_limit(MotionLimitReason::Lead, Some(i)); }
                    else if let Some(i) = speed_hit { self.note_motion_limit(MotionLimitReason::Speed, Some(i)); }
                    // Lead-capped without measured progress for the persistence time: blocked.
                    let persistence = Duration::from_millis(self.config.following_error.persistence_ms);
                    let mut blocked = None;
                    for (i, &capped) in lead_capped.iter().enumerate() {
                        let meas = self.measured_positions[i];
                        self.servo_lead_stall[i] = match (capped, self.servo_lead_stall[i]) {
                            (false, _) => None,
                            (true, Some((t0, m0))) if (meas - m0).abs() <= SERVO_BLOCKED_PROGRESS_RAD => {
                                if t0.elapsed() >= persistence && blocked.is_none() { blocked = Some(i); }
                                Some((t0, m0))
                            }
                            (true, _) => Some((Instant::now(), meas)),
                        };
                    }
                    if let Some(i) = infeasible {
                        self.latch_servo_fault(i);
                    } else if let Some(i) = blocked {
                        self.enter_servo_blocked_hold(i, "no progress at the lead cap");
                    } else if let Err(e) = self.servo_step_workspace(&next_positions, "servo_step") {
                        self.enter_servo_keepout_hold(&e);
                    } else {
                        self.commanded_positions = next_positions;
                        self.commanded_velocities = next_velocities;
                        self.target_positions = self.commanded_positions.clone();

                        // Update hysteresis counter for each axis: if reached, increment; if not reached, reset to 0
                        for i in 0..self.config.dof {
                            if reached_flags[i] {
                                self.servo_settled_ticks[i] = self.servo_settled_ticks[i].saturating_add(1);
                            } else {
                                self.servo_settled_ticks[i] = 0;
                            }
                        }

                        // If all axes have servo_settled_ticks[i] >= 2, mark motion finished
                        if self.servo_settled_ticks.iter().all(|&ticks| ticks >= 2) {
                            self.servo_target = None;
                            self.safety.mark_motion_finished();
                            if self.commanded_velocities.iter().all(|&v| v == 0.0) {
                                self.servo_ws_min_depths = None;
                                let keep_stream = self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled)
                                    && self.tracking.has_residual_lead(
                                        &self.commanded_positions,
                                        &self.config.arm_joint_indices(),
                                    );
                                if !keep_stream {
                                    self.servo_stream_active = false;
                                }
                            }
                        }
                    }
                }
            }
        }

        if !reference_stepped
            && self.servo_target.is_none()
            && self.active_trajectory.is_none()
            && !self.pending_normal_unload
            && self.gravity_comp.is_none()
            && !self.tracking.recovery_suspended
            && self.tracking.fallback_ramp.is_none()
            && !self.safety.state().is_stopped()
        {
            // A stale/dropped target requests zero velocity. The same feasible
            // interval governs braking, with no small-velocity truncation.
            let mut next_positions = self.commanded_positions.clone();
            let mut next_velocities = self.commanded_velocities.clone();
            let mut infeasible = None;
            let step_limits = self.config.limits.clone(); // braking a stream: same limits as streaming
            for i in 0..self.config.dof {
                let step = Self::bounded_servo_step_with_jerk(
                    self.commanded_positions[i],
                    self.commanded_positions[i],
                    self.commanded_velocities[i],
                    servo_dt,
                    &step_limits[i],
                    self.commanded_accelerations[i],
                    self.config.max_jerk,
                );
                let Some((q_next, v_next, _)) = step else {
                    infeasible = Some(i);
                    break;
                };
                next_positions[i] = q_next;
                next_velocities[i] = v_next;
            }
            let braking = self.commanded_velocities.iter().any(|&v| v != 0.0);
            if let Some(i) = infeasible {
                self.latch_servo_fault(i);
            } else if let Some(e) = braking.then(|| self.servo_step_workspace(&next_positions, "braking").err()).flatten() {
                self.enter_servo_keepout_hold(&e);
            } else {
                self.commanded_positions = next_positions;
                self.commanded_velocities = next_velocities;
                self.target_positions = self.commanded_positions.clone();
                if self.commanded_velocities.iter().all(|&v| v == 0.0) {
                    self.safety.mark_motion_finished();
                    self.servo_ws_min_depths = None;
                    let keep_stream = self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled)
                        && (braking
                            || self.tracking.has_residual_lead(
                                &self.commanded_positions,
                                &self.config.arm_joint_indices(),
                            ));
                    if !keep_stream {
                        self.servo_stream_active = false;
                    }
                }
            }
        }

        self.gripper_lead_cap();
        for i in 0..self.config.dof {
            self.commanded_accelerations[i] = if self.safety.state().is_stopped() { 0.0 }
                else { (self.commanded_velocities[i] - before_velocities[i]) / servo_dt as f32 };
        }
        self.last_commanded_velocities = self.commanded_velocities.clone();

        let feedback_fresh = self.hardware_feedback_fresh(BUS_LOSS_TIMEOUT);
        if self.driver.is_some() && self.is_torqued {
            if !feedback_fresh {
                self.output_suspended = true;
                self.tracking.invalidate();
            } else if self.output_suspended {
                self.stop_motion();
                self.tracking.invalidate();
                self.commanded_positions = self.measured_positions.clone();
                self.target_positions = self.measured_positions.clone();
                self.commanded_velocities.fill(0.0);
                self.commanded_accelerations.fill(0.0);
                self.output_suspended = false;
                if !self.safety.state().is_stopped() {
                    self.safety.trigger_fault(FaultReason::BusLoss);
                }
            }
        }
        let gravity_output_enabled = self.gravity_output_enabled();
        // Hold only while torque may be active and fresh valid feedback permits output.
        // Pending disable and invalid feedback suppress ordinary commands; reconnect resynchronizes first.
        let mut output_error = None;
        let mut tick_output_accepted = false;
        if self.is_torqued && !self.disable_pending && self.gravity_comp.is_some() {
            // Gravity compensation (any phase): MIT targets for J1-J6. Sent without the
            // feedback-freshness gate so that an exit hold reaches the motors even when
            // feedback is late; a floating phase never outlives fresh feedback (it exits).
            if self.driver.is_none() {
                tick_output_accepted = true;
            } else if let (Some(targets), Some(d)) = (self.gravity_comp_targets(), self.driver.as_ref()) {
                if let Ok(mut driver) = d.lock() {
                    if driver.enable_state() != EnableState::Disabled {
                        match driver.write_joint_targets(&targets) {
                            Ok(()) => tick_output_accepted = true,
                            Err(e) => output_error = Some(e.to_string()),
                        }
                    }
                }
            }
        } else if self.is_torqued && !self.disable_pending {
            let send_to_bus = feedback_fresh && self.driver.as_ref().is_none_or(|d|
                d.lock().is_ok_and(|drv| drv.enable_state() != EnableState::Disabled));

            if send_to_bus {
                let tracking_active = self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled);
                let arm_indices = self.config.arm_joint_indices();
                let has_residual = tracking_active
                    && self.tracking.has_residual_lead(&self.commanded_positions, &arm_indices);
                let holding_residual = !reference_stepped
                    && self.active_trajectory.is_none()
                    && self.servo_target.is_none()
                    && self.commanded_velocities.iter().all(|&v| v.abs() <= 1e-6);
                let ramp_out_handover = tracking_active
                    && self.driver.is_some()
                    && !self.safety.state().is_stopped()
                    && (self.pending_normal_unload
                        || self.handover_settling_since.is_some()
                        || self.torque_guard.hold.is_some()
                        || holding_residual)
                    && has_residual;
                let tracking_owned = tracking_active
                    && self.driver.is_some()
                    && !self.safety.state().is_stopped()
                    && !self.pending_normal_unload;

                if tracking_active
                    && self.tracking.fallback_ramp.is_some()
                    && (self.active_trajectory.is_some() || self.servo_target.is_some() || self.pending_normal_unload)
                {
                    // A new motion or park took over from the issued (braking) state.
                    self.tracking.fallback_ramp = None;
                }
                if tracking_active && self.tracking.recovery_suspended {
                    // Driver emitted an autonomous recovery write; suspend supervisor target
                    // writes until fresh post-recovery feedback confirms every joint's baseline.
                } else if tracking_active && self.driver.is_some() && self.tracking.fallback_ramp.is_some() {
                    // Rejection-fallback braking ramp, also while the rejection fault is latched.
                    let ramp = self.tracking.fallback_ramp.clone().expect("checked above");
                    let active_limits = self.active_tracking_limits(servo_dt, time_scale);
                    let mut tick_sweep_budget = MAX_KEEPOUT_EVALS_PER_TICK;
                    let pending = self.tracking_fallback_compute(ramp, servo_dt, &active_limits, &mut tick_sweep_budget);
                    output_error = self.apply_tracking_fallback_step(pending, gravity_output_enabled);
                    self.tracking.last_tick_sweep_evals =
                        MAX_KEEPOUT_EVALS_PER_TICK.saturating_sub(tick_sweep_budget);
                } else if ramp_out_handover || tracking_owned {
                    let jt = self.config.joint_tracking.as_ref().unwrap();
                    let active_limits = self.active_tracking_limits(servo_dt, time_scale);
                    let mut tick_sweep_budget = MAX_KEEPOUT_EVALS_PER_TICK;
                    let eval_res = if ramp_out_handover {
                        self.compute_lead_ramp_out_with_budget(
                            servo_dt,
                            &active_limits,
                            self.pending_normal_unload,
                            &mut tick_sweep_budget,
                        )
                    } else if self.handover_settling_since.is_some() {
                        // Moving lease handover with lead already ramped out: validate nominal
                        // handover hold/braking without adding new lead.
                        let mut zero_jt = jt.clone();
                        for j in &mut zero_jt.joints {
                            j.strategy = crate::tracking::TrackingStrategy::Position;
                        }
                        self.compute_position_lead_output_with_limits_and_budget(
                            servo_dt,
                            &zero_jt,
                            &active_limits,
                            &mut tick_sweep_budget,
                        )
                    } else {
                        self.compute_position_lead_output_with_limits_and_budget(
                            servo_dt,
                            jt,
                            &active_limits,
                            &mut tick_sweep_budget,
                        )
                    };
                    match eval_res {
                        Ok(out) => {
                            let targets: Vec<JointTarget> = out
                                .positions
                                .iter()
                                .zip(self.commanded_velocities.iter())
                                .enumerate()
                                .map(|(i, (&p, &v))| JointTarget {
                                    position: p,
                                    velocity: Some(v),
                                    torque: if gravity_output_enabled {
                                        self.gravity_torques_ramped.get(i).copied()
                                    } else {
                                        Some(0.0)
                                    },
                                    kp: None,
                                    kd: None,
                                })
                                .collect();
                            let write_res = self
                                .driver
                                .as_ref()
                                .and_then(|d| d.lock().ok())
                                .map(|mut driver| driver.write_joint_targets(&targets));
                            match write_res {
                                Some(Ok(())) => {
                                    self.tracking.record_issued_with_ws_depths(
                                        out.positions,
                                        out.velocities,
                                        out.ws_min_depths,
                                        out.ws_keepout_min_depths,
                                    );
                                    tick_output_accepted = true;
                                }
                                Some(Err(e)) => {
                                    self.tracking.invalidate();
                                    output_error = Some(e.to_string());
                                }
                                None => self.tracking.invalidate(),
                            }
                        }
                        Err(reject) => {
                            let was_streaming = self.servo_target.is_some() || self.servo_stream_active;
                            // Rejection fallback: evaluated from the intact history and park
                            // state BEFORE the hold below invalidates them. It brakes the issued
                            // output toward the measured hold within the active limits (continued
                            // on later ticks by `fallback_ramp`), or latches a fault without
                            // sending a frame when no step is feasible.
                            let ramp = crate::tracking::FallbackRamp {
                                hold: self.measured_hold_pose(),
                                keepout_only: self.pending_normal_unload,
                            };
                            let pending =
                                self.tracking_fallback_compute(ramp, servo_dt, &active_limits, &mut tick_sweep_budget);
                            match reject {
                                crate::tracking::TrackingReject::Workspace(ws_err) if was_streaming => {
                                    self.enter_servo_keepout_hold(&ws_err);
                                }
                                crate::tracking::TrackingReject::Workspace(ws_err) => {
                                    tracing::warn!("Joint tracking output violated workspace ({ws_err}): terminating motion");
                                    self.hold_measured_pose();
                                    self.macro_run = None;
                                    self.safety.trigger_fault_coded(
                                        FaultCode::Other,
                                        FaultReason::Other(format!("Joint tracking output rejected: {ws_err}")),
                                    );
                                }
                                crate::tracking::TrackingReject::JointLimit(msg) => {
                                    tracing::warn!("Joint tracking output exceeded limits ({msg}): terminating motion");
                                    self.hold_measured_pose();
                                    self.macro_run = None;
                                    self.safety.trigger_fault_coded(
                                        FaultCode::JointLimit,
                                        FaultReason::Other(format!("Joint tracking output rejected: {msg}")),
                                    );
                                }
                            }
                            output_error = self.apply_tracking_fallback_step(pending, gravity_output_enabled);
                        }
                    }
                    self.tracking.last_tick_sweep_evals =
                        MAX_KEEPOUT_EVALS_PER_TICK.saturating_sub(tick_sweep_budget);
                } else {
                    if tracking_active {
                        self.tracking.invalidate();
                    }
                    if let Some(ref d) = self.driver {
                        if let Ok(mut driver) = d.lock() {
                            let targets: Vec<JointTarget> = self
                                .commanded_positions
                                .iter()
                                .zip(self.commanded_velocities.iter())
                                .enumerate()
                                .map(|(i, (&p, &v))| JointTarget {
                                    position: p,
                                    velocity: Some(v),
                                    torque: if gravity_output_enabled {
                                        self.gravity_torques_ramped.get(i).copied()
                                    } else {
                                        Some(0.0)
                                    },
                                    kp: None,
                                    kd: None,
                                })
                                .collect();
                            match driver.write_joint_targets(&targets) {
                                Ok(()) => tick_output_accepted = true,
                                Err(e) => output_error = Some(e.to_string()),
                            }
                        }
                    } else {
                        tick_output_accepted = true;
                    }
                }
            } else {
                self.tracking.invalidate();
            }
        } else {
            self.tracking.invalidate();
        }

        if let Some(error) = output_error {
            tick_output_accepted = false;
            self.stop_motion();
            self.tracking.invalidate();
            self.commanded_velocities.fill(0.0);
            if self.measured_positions.iter().all(|q| q.is_finite()) {
                self.commanded_positions = self.measured_positions.clone();
                self.target_positions = self.measured_positions.clone();
            }
            if !self.safety.state().is_stopped() {
                self.safety.trigger_fault_coded(FaultCode::DriverWriteFailed, FaultReason::Other(format!("Control output failed: {error}")));
            }
            tracing::error!("Control output failed: {error}");
        }

        if self.servo_stream_active
            && self.servo_target.is_none()
            && (self.safety.state().is_stopped()
                || (self.commanded_velocities.iter().all(|&v| v == 0.0)
                    && !self.tracking.has_residual_lead(
                        &self.commanded_positions,
                        &self.config.arm_joint_indices(),
                    )))
        {
            self.servo_stream_active = false;
        }

        let tick_output_accepted = tick_output_accepted && !self.safety.state().is_stopped();
        if let Some(done) = deferred_done {
            if tick_output_accepted {
                if done.clear_releasing {
                    self.releasing = false;
                    self.held = None;
                }
                if !self.pending_normal_unload {
                    self.safety.mark_motion_finished();
                }
                if done.macro_sub {
                    if let Some(m) = self.macro_run.as_mut() {
                        m.sub = SubState::Done(SubDone {
                            settled: done.settled,
                            holding: done.holding,
                            gripper_gap_rad: done.gripper_gap_rad,
                            at: Instant::now(),
                        });
                    }
                } else if let Some(id) = done.action_id {
                    self.finish_action(
                        id,
                        "done",
                        None,
                        None,
                        done.settled,
                        done.holding,
                        done.gripper_gap_rad,
                    );
                }
            }
        }

        // If in simulation (no hardware driver attached), mirror commanded into measured
        if self.driver.is_none() {
            self.measured_positions = self.commanded_positions.clone();
            self.measured_velocities = self.commanded_velocities.clone();
            if let Some(g_idx) = self.config.gripper_index() {
                let gripper_moving = matches!(&self.active_trajectory, Some(ActiveTrajectory { reference: Reference::Gripper(_), .. }));
                // the stall torque lasts only while the jaw pushes on the stop; a stale value would
                // end the next gripper motion at once as a stall
                let mut stalled = false;
                if gripper_moving {
                    if let Some(policy) = self.config.gripper_policy.as_ref() {
                        let base_closed = self.base_gripper_policy.as_ref().map_or(policy.closed_rad, |p| p.closed_rad);
                        let stall_offset = self.mock_gripper_stall_offset.unwrap_or(0.0);
                        let stop_rad = base_closed + stall_offset;
                        let dir = if policy.closed_rad >= policy.open_rad { 1.0 } else { -1.0 };
                        let push = (self.commanded_positions[g_idx] - stop_rad) * dir;
                        if push > 1e-4 {
                            self.measured_positions[g_idx] = stop_rad;
                            self.measured_velocities[g_idx] = 0.0;
                            self.torques[g_idx] = dir * (GRIPPER_STALL_LATCH_NM * 1.5 + push * 2.0);
                            stalled = true;
                        }
                    }
                }
                if let (Some((obj, touched)), Some(policy)) = (self.mock_gripper_object, self.config.gripper_policy.as_ref()) {
                    let dir = if policy.closed_rad >= policy.open_rad { 1.0 } else { -1.0 };
                    let push = (self.commanded_positions[g_idx] - obj) * dir;
                    if push > 1e-4 {
                        // squeeze torque rises with the command past the contact (POS_VEL stiffness)
                        self.measured_positions[g_idx] = obj;
                        self.measured_velocities[g_idx] = 0.0;
                        self.torques[g_idx] = dir * (GRIPPER_STALL_LATCH_NM * 1.5 + push * 2.0);
                        stalled = true;
                        self.mock_gripper_object = Some((obj, true));
                    } else if touched && push < -0.5 {
                        self.mock_gripper_object = None;
                    }
                }
                // clear only the torque this mock stall set (tests may inject torques themselves)
                if stalled {
                    self.mock_gripper_stall_torque = true;
                } else if self.mock_gripper_stall_torque {
                    self.torques[g_idx] = 0.0;
                    self.mock_gripper_stall_torque = false;
                }
            }
        }

        if let Some(settling_since) = self.handover_settling_since {
            if self.safety.state().is_stopped() {
                // If arm entered Stopped (Fault, e.g. ThermalTrip, BusLoss, or EStop), terminate
                // handover settling immediately and preserve the existing fault reason!
                self.handover_settling_since = None;
            } else if self.is_fully_settled_armed_hold() {
                self.handover_settling_since = None;
            } else if settling_since.elapsed() > Duration::from_millis(1500) {
                self.stop_motion();
                self.commanded_velocities = vec![0.0; self.config.dof];
                self.safety.trigger_fault_coded(FaultCode::HandoverUnsettled, FaultReason::Other("Handover settling timeout".into()));
                self.handover_settling_since = None;
            }
        }

        self.reconcile_action(servo_dt, tick_output_accepted);
        self.expire_motion_limit();
        let obs = self.current_observation();

        // 5. Record frame if episode recording is active
        if self.recording_frame_room() {
            self.recorder.append_frame(RecordFrame {
                timestamp: obs.timestamp,
                observation_state: self.measured_positions.clone(),
                action: self.commanded_positions.clone(),
            });
        }

        obs
    }
}

#[cfg(feature = "policy-runtime")]
impl SupervisorInner {
    /// Phase 1/3 real-arm gate: `Live` policy mode without release gate is only allowed for explicit mock or sim drivers.
    pub fn policy_live_allowed(&self) -> bool {
        matches!(
            (self.driver.is_some(), self.follower.as_deref()),
            (false, None) | (false, Some("mock")) | (_, Some("sim"))
        )
    }

    /// Deterministic SHA-256 digest of the active camera calibration state, or
    /// `EMPTY_CALIBRATION_DIGEST` when uncalibrated.
    pub fn active_calibration_digest(&self) -> String {
        match &self.calibration {
            None => crate::policy_runtime::EMPTY_CALIBRATION_DIGEST.to_string(),
            Some(cal) => {
                let bytes = serde_json::to_vec(cal).unwrap_or_default();
                crate::gravity::compute_sha256(&bytes)
            }
        }
    }

    pub fn driver_feedback_confirmed(&self) -> bool {
        match &self.driver {
            None => false,
            Some(d) => d.lock().is_ok_and(|drv| drv.is_feedback_valid()),
        }
    }

    pub fn policy_run_active(&self) -> bool {
        self.policy_run.is_some()
    }

    pub fn is_policy_run_active(&self, action_id: u64) -> bool {
        self.policy_run
            .as_ref()
            .is_some_and(|r| r.action_id == action_id)
    }

    pub fn active_policy_release_gate(
        &self,
        action_id: u64,
    ) -> Option<&crate::policy_runtime::VerifiedReleaseGate> {
        self.policy_run
            .as_ref()
            .filter(|r| {
                r.action_id == action_id && r.mode == crate::policy_runtime::PolicyMode::Live
            })
            .and_then(|r| r.verified_release_gate.as_ref())
    }

    pub fn policy_observation(&self) -> Option<crate::model::PolicyObservation> {
        if let Some(ref run) = self.policy_run {
            Some(run.to_observation(true))
        } else {
            self.last_policy_obs.clone()
        }
    }

    pub fn policy_input_age_ms(&self) -> Option<u64> {
        self.policy_input_age_range_ms().map(|(_, max_ms)| max_ms)
    }

    pub fn policy_input_age_range_ms(&self) -> Option<(u64, u64)> {
        match &self.driver {
            None => Some((0, 0)),
            Some(d) => d.lock().ok().and_then(|drv| {
                let ages = drv.feedback_ages();
                if ages.len() != self.config.dof {
                    return None;
                }
                let durs = ages.into_iter().collect::<Option<Vec<_>>>()?;
                let mut min_ms = u64::MAX;
                let mut max_ms = 0u64;
                for dur in durs {
                    let ms = dur.as_millis().min(u64::MAX as u128) as u64;
                    min_ms = min_ms.min(ms);
                    max_ms = max_ms.max(ms);
                }
                if min_ms > max_ms {
                    return None;
                }
                Some((min_ms, max_ms))
            }),
        }
    }

    /// Preconditions checked under lock before loading a policy pack outside the lock.
    pub fn check_policy_start_preconditions(
        &self,
        mode: crate::policy_runtime::PolicyMode,
        epoch_id: u64,
    ) -> Result<(), ArmError> {
        self.check_policy_start_preconditions_with(mode, false, false, epoch_id)
    }

    /// Preconditions checked under lock with explicit `sim_only` and operator presence flags.
    pub fn check_policy_start_preconditions_with(
        &self,
        mode: crate::policy_runtime::PolicyMode,
        sim_only: bool,
        i_am_at_the_arm: bool,
        epoch_id: u64,
    ) -> Result<(), ArmError> {
        use crate::policy_runtime::PolicyMode;

        if self.estop_pending() {
            return Err(SafetyError::EStopActive.into());
        }
        if epoch_id != self.safety.epoch_id() {
            return Err(SafetyError::EpochMismatch {
                expected: self.safety.epoch_id(),
                received: epoch_id,
            }
            .into());
        }
        if self.handover_settling_since.is_some() {
            return Err(SafetyError::HandoverSettling.into());
        }
        if sim_only && !self.policy_live_allowed() {
            return Err(ArmError::Rejected {
                status: 403,
                code: "POLICY_SIM_ONLY".into(),
                message: "live_sim mode is not permitted on real or unknown hardware drivers".into(),
            });
        }
        match self.safety.state() {
            SafetyState::EStop => return Err(SafetyError::EStopActive.into()),
            SafetyState::Fault(r) => return Err(SafetyError::FaultActive(r.clone()).into()),
            s if mode == PolicyMode::Live && !s.is_operational() => {
                return Err(SafetyError::NotArmed(s.clone()).into());
            }
            _ => {}
        }
        if mode == PolicyMode::Live {
            if !self.policy_live_allowed() && !i_am_at_the_arm {
                return Err(ArmError::Rejected {
                    status: 403,
                    code: "OPERATOR_CONFIRMATION_REQUIRED".into(),
                    message: "Real-arm live policy execution requires explicit i_am_at_the_arm confirmation".into(),
                });
            }
            self.gravity_comp_servo_busy().map_err(ArmError::Safety)?;
            self.collision_hold_check().map_err(ArmError::Safety)?;
            if self.pending_rearm || self.pending_normal_unload || !self.is_torqued {
                return Err(SafetyError::NotArmed(self.safety.state().clone()).into());
            }
            if self.servo_blocked_hold {
                return Err(ArmError::Rejected {
                    status: 409,
                    code: "BLOCKED_HOLD".into(),
                    message: "Cannot start live policy while servo blocked hold is active".into(),
                });
            }
            if self.commanded_velocities.iter().any(|v| v.abs() >= 1e-3) {
                return Err(ArmError::Safety(SafetyError::MotionBusy));
            }
        }
        if self.calibration_busy
            || self.zero_check_persisting.is_some()
            || self.gravity_comp.is_some()
            || self.pending_normal_unload
            || self.active_trajectory.is_some()
            || self.macro_run.is_some()
            || self.policy_loading
            || self.policy_run.is_some()
            || self.servo_target.is_some()
        {
            return Err(ArmError::Safety(SafetyError::MotionBusy));
        }
        Ok(())
    }

    /// Install an exclusive streaming `PolicyRun` after lease, epoch, driver gate, and mutual exclusion checks.
    pub fn begin_policy_run(
        &mut self,
        req: crate::policy_runtime::PolicyRunStart,
    ) -> Result<u64, ArmError> {
        use crate::policy_runtime::{PolicyMode, PolicyRun, DEFAULT_MAX_FIRST_STEP_RAD};

        self.check_policy_start_preconditions_with(
            req.mode,
            req.sim_only,
            req.i_am_at_the_arm,
            req.epoch_id,
        )?;
        if req.mode == PolicyMode::Live && !self.policy_live_allowed() {
            if !req.i_am_at_the_arm {
                return Err(ArmError::Rejected {
                    status: 403,
                    code: "OPERATOR_CONFIRMATION_REQUIRED".into(),
                    message: "Real-arm live policy execution requires explicit i_am_at_the_arm confirmation".into(),
                });
            }
            let Some(ref gate) = req.verified_release_gate else {
                return Err(ArmError::Rejected {
                    status: 403,
                    code: "RELEASE_GATE_REQUIRED".into(),
                    message: "Real-arm live policy execution requires a verified rebot-release-gate/v1 bound to the policy artifact digest".into(),
                });
            };
            if req.artifact_digest.is_empty()
                || !gate.policy_sha256().eq_ignore_ascii_case(&req.artifact_digest)
            {
                return Err(ArmError::Rejected {
                    status: 403,
                    code: "RELEASE_GATE_REQUIRED".into(),
                    message: "Real-arm live policy execution requires a verified rebot-release-gate/v1 bound to the policy artifact digest".into(),
                });
            }
            let active_calib = self.active_calibration_digest();
            if !gate.calibration_digest().eq_ignore_ascii_case(&active_calib) {
                return Err(crate::policy_runtime::PolicyError::ReleaseGate(format!(
                    "calibration_digest mismatch: gate declares {}, active calibration is {active_calib}",
                    gate.calibration_digest()
                ))
                .into());
            }
            if !self.feedback_numeric_valid
                || !self.driver_feedback_confirmed()
                || self.policy_input_age_range_ms().is_none()
            {
                return Err(ArmError::Rejected {
                    status: 409,
                    code: "FEEDBACK_UNCONFIRMED".into(),
                    message: "Real-arm live policy execution requires confirmed, valid hardware feedback".into(),
                });
            }
            if gate.authorization_tier() == crate::policy_runtime::AuthorizationTier::ProductionRelease {
                use crate::policy_runtime::qualification;
                let Some(path) = self.policy_ledger_path.as_deref() else {
                    return Err(ArmError::Rejected {
                        status: 403,
                        code: "QUALIFICATION_REQUIRED".into(),
                        message: "production_release needs the device qualification ledger, and none is configured".into(),
                    });
                };
                let records = qualification::load_records(path)?;
                qualification::check_production_qualified(&records, gate).map_err(|e| ArmError::Rejected {
                    status: 403,
                    code: "QUALIFICATION_REQUIRED".into(),
                    message: e.to_string(),
                })?;
            }
        }
        if !self.validate_servo_lease(&req.connection_id, req.lease_generation) {
            return Err(ArmError::Rejected {
                status: 409,
                code: "LEASE_LOST".into(),
                message: "Connection does not own an active lease".into(),
            });
        }

        let max_first_step_rad = req
            .max_first_step_rad
            .filter(|v| v.is_finite() && *v > 0.0)
            .map_or(DEFAULT_MAX_FIRST_STEP_RAD, |v| {
                v.min(DEFAULT_MAX_FIRST_STEP_RAD)
            });

        let action_id = self.begin_action("policy", req.tag);
        let run = PolicyRun {
            action_id,
            epoch_id: req.epoch_id,
            connection_id: req.connection_id,
            lease_generation: req.lease_generation,
            policy_id: req.policy_id,
            artifact_digest: req.artifact_digest,
            backend: req.backend,
            mode: req.mode,
            steps: 0,
            last_input_age_ms: None,
            last_vision_age_ms: None,
            vision_frame_id: None,
            last_shadow_target: None,
            last_live_target: None,
            max_first_step_rad,
            max_step_rad: req.max_step_rad,
            i_am_at_the_arm: req.i_am_at_the_arm,
            verified_release_gate: req.verified_release_gate,
            stop_reason: None,
            max_input_age_ms: crate::policy_runtime::max_policy_input_age_ms(crate::policy_runtime::MIN_LIVE_POLICY_FPS),
            step_stats: crate::policy_runtime::PolicyStepStats::default(),
        };
        self.last_policy_obs = Some(run.to_observation(true));
        self.set_action_policy_stats(action_id, run.step_stats.report());
        self.policy_run = Some(run);
        Ok(action_id)
    }

    /// Submit one policy inference target frame.
    ///
    /// Enforces strict identity/epoch/lease checks, never renews `last_heartbeat`, never clears
    /// `servo_blocked_hold`, enforces first-step and per-step jump caps in `Live` mode, and
    /// shares `validate_and_filter_servo_targets` with `servo()`.
    pub fn policy_step(
        &mut self,
        action_id: u64,
        targets: Vec<f32>,
        input_age_ms: Option<u64>,
    ) -> Result<(), ArmError> {
        self.policy_step_with_vision(action_id, targets, input_age_ms, None, None)
    }

    pub fn policy_step_with_vision(
        &mut self,
        action_id: u64,
        mut targets: Vec<f32>,
        input_age_ms: Option<u64>,
        vision_age_ms: Option<u64>,
        vision_frame_id: Option<u64>,
    ) -> Result<(), ArmError> {
        use crate::policy_runtime::PolicyMode;

        let (epoch_id, conn_id, lease_gen, mode, max_first, max_step, prev_live, max_input_age_ms) = {
            let Some(active) = self.policy_run.as_ref() else {
                return Err(ArmError::Rejected {
                    status: 409,
                    code: "POLICY_NOT_ACTIVE".into(),
                    message: "No active policy run".into(),
                });
            };
            if active.action_id != action_id {
                return Err(ArmError::Rejected {
                    status: 409,
                    code: "POLICY_NOT_ACTIVE".into(),
                    message: "Stale policy action_id".into(),
                });
            }
            (
                active.epoch_id,
                active.connection_id.clone(),
                active.lease_generation,
                active.mode,
                active.max_first_step_rad,
                active.max_step_rad,
                active.last_live_target.clone(),
                active.max_input_age_ms,
            )
        };

        if self.estop_pending() || matches!(self.safety.state(), SafetyState::EStop) {
            self.terminate_policy_run("ESTOP", "failed", Some("ESTOP_ACTIVE"), Some("E-stop latched".into()));
            return Err(SafetyError::EStopActive.into());
        }
        if epoch_id != self.safety.epoch_id() {
            let current = self.safety.epoch_id();
            self.terminate_policy_run(
                "STOPPED",
                "cancelled",
                Some("EPOCH_MISMATCH"),
                Some("Epoch changed during policy run".into()),
            );
            return Err(SafetyError::EpochMismatch {
                expected: current,
                received: epoch_id,
            }
            .into());
        }
        if let SafetyState::Fault(r) = self.safety.state().clone() {
            let reason = if matches!(r, FaultReason::WatchdogTimeout) {
                "WATCHDOG_TIMEOUT"
            } else {
                "FAULT"
            };
            self.terminate_policy_run(
                reason,
                "failed",
                Some(reason),
                Some(format!("Fault active: {r:?}")),
            );
            return Err(SafetyError::FaultActive(r).into());
        }

        if !self.validate_servo_lease(&conn_id, lease_gen) {
            self.terminate_policy_run(
                "LEASE_LOST",
                "cancelled",
                Some("LEASE_LOST"),
                Some("Control lease lost or expired".into()),
            );
            return Err(ArmError::Rejected {
                status: 409,
                code: "LEASE_LOST".into(),
                message: "Control lease lost or expired".into(),
            });
        }

        if self.servo_blocked_hold {
            self.terminate_policy_run(
                "BLOCKED_HOLD",
                "cancelled",
                Some("BLOCKED_HOLD"),
                Some("Servo blocked hold active".into()),
            );
            return Err(ArmError::Rejected {
                status: 409,
                code: "BLOCKED_HOLD".into(),
                message: "Servo blocked hold active".into(),
            });
        }

        if targets.len() != self.config.dof || targets.iter().any(|v| !v.is_finite()) {
            self.terminate_policy_run(
                "INVALID_TARGET",
                "failed",
                Some("INVALID_TARGET"),
                Some("Policy produced non-finite or wrong-dimension target".into()),
            );
            return Err(SafetyError::InvalidValue.into());
        }

        if mode == PolicyMode::Shadow {
            if let Some(run) = self.policy_run.as_mut().filter(|r| r.action_id == action_id) {
                run.steps += 1;
                run.last_input_age_ms = input_age_ms;
                run.last_vision_age_ms = vision_age_ms;
                run.vision_frame_id = vision_frame_id;
                run.last_shadow_target = Some(targets);
                self.last_policy_obs = Some(run.to_observation(true));
            }
            return Ok(());
        }

        // Live mode checks: release-gate calibration binding, real-arm gate, stale joint input
        // (max_policy_input_age_ms), jump limits, and shared servo validation.
        if let Some(gate) = self
            .policy_run
            .as_ref()
            .filter(|r| r.action_id == action_id)
            .and_then(|r| r.verified_release_gate.as_ref())
        {
            let active_calib = self.active_calibration_digest();
            if !gate.calibration_digest().eq_ignore_ascii_case(&active_calib) {
                let msg = format!(
                    "calibration_digest mismatch: gate declares {}, active deployment is {active_calib}",
                    gate.calibration_digest()
                );
                self.terminate_policy_run(
                    "DEPLOYMENT_CHANGED",
                    "failed",
                    Some("DEPLOYMENT_CHANGED"),
                    Some(msg.clone()),
                );
                return Err(ArmError::Rejected {
                    status: 409,
                    code: "DEPLOYMENT_CHANGED".into(),
                    message: msg,
                });
            }
        }
        if !self.policy_live_allowed() {
            let (has_confirm, has_gate) = self.policy_run.as_ref().map_or((false, false), |run| {
                (
                    run.i_am_at_the_arm,
                    run.verified_release_gate.as_ref().is_some_and(|g| {
                        !run.artifact_digest.is_empty()
                            && g.policy_sha256().eq_ignore_ascii_case(&run.artifact_digest)
                    }),
                )
            });
            if !has_confirm {
                self.terminate_policy_run(
                    "OPERATOR_CONFIRMATION_REQUIRED",
                    "failed",
                    Some("OPERATOR_CONFIRMATION_REQUIRED"),
                    Some("Real-arm live policy execution requires operator confirmation".into()),
                );
                return Err(ArmError::Rejected {
                    status: 403,
                    code: "OPERATOR_CONFIRMATION_REQUIRED".into(),
                    message: "Real-arm live policy execution requires operator confirmation".into(),
                });
            }
            if !has_gate {
                self.terminate_policy_run(
                    "RELEASE_GATE_REQUIRED",
                    "failed",
                    Some("RELEASE_GATE_REQUIRED"),
                    Some("Real-arm live policy execution requires a verified release gate".into()),
                );
                return Err(ArmError::Rejected {
                    status: 403,
                    code: "RELEASE_GATE_REQUIRED".into(),
                    message: "Real-arm live policy execution requires a verified release gate".into(),
                });
            }
            if !self.feedback_numeric_valid
                || !self.driver_feedback_confirmed()
                || self.policy_input_age_range_ms().is_none()
            {
                self.terminate_policy_run(
                    "FEEDBACK_UNCONFIRMED",
                    "failed",
                    Some("FEEDBACK_UNCONFIRMED"),
                    Some("Hardware feedback is unconfirmed during live policy step".into()),
                );
                return Err(ArmError::Rejected {
                    status: 409,
                    code: "FEEDBACK_UNCONFIRMED".into(),
                    message: "Hardware feedback is unconfirmed during live policy step".into(),
                });
            }
        }

        // Capture-to-commit age of the joint positions this target was computed from: a
        // target from an old pose (slow inference while the arm moved) never reaches the servo.
        let stale = match input_age_ms {
            Some(age) => (age > max_input_age_ms).then(|| format!("{age} ms > {max_input_age_ms} ms")),
            None if !self.policy_live_allowed() => Some("input age unknown".to_string()),
            None => None,
        };
        if let Some(why) = stale {
            let msg = format!("Live policy target computed from stale joint positions ({why})");
            self.terminate_policy_run("STALE_INPUT", "failed", Some("STALE_INPUT"), Some(msg.clone()));
            return Err(ArmError::Rejected { status: 409, code: "STALE_INPUT".into(), message: msg });
        }

        if prev_live.is_none() {
            if self.measured_positions.len() != self.config.dof
                || self.measured_positions.iter().any(|m| !m.is_finite())
            {
                self.terminate_policy_run(
                    "INVALID_FEEDBACK",
                    "failed",
                    Some("INVALID_FEEDBACK"),
                    Some("Measured joint positions are invalid on first live policy step".into()),
                );
                return Err(SafetyError::FeedbackUnavailable.into());
            }
            for i in 0..self.config.dof {
                let delta = (targets[i] - self.measured_positions[i]).abs();
                if delta > max_first {
                    let msg = format!(
                        "First live policy step on joint {} exceeds max_first_step_rad ({delta:.4} > {max_first:.4})",
                        i + 1
                    );
                    self.terminate_policy_run("JUMP_LIMIT", "failed", Some("JUMP_LIMIT"), Some(msg.clone()));
                    return Err(ArmError::Rejected {
                        status: 422,
                        code: "JUMP_LIMIT".into(),
                        message: msg,
                    });
                }
            }
        } else if let (Some(prev), Some(step_cap)) = (prev_live.as_ref(), max_step) {
            for i in 0..self.config.dof {
                let delta = (targets[i] - prev[i]).abs();
                if delta > step_cap {
                    let msg = format!(
                        "Live policy step on joint {} exceeds max_step_rad ({delta:.4} > {step_cap:.4})",
                        i + 1
                    );
                    self.terminate_policy_run("STEP_LIMIT", "failed", Some("STEP_LIMIT"), Some(msg.clone()));
                    return Err(ArmError::Rejected {
                        status: 422,
                        code: "STEP_LIMIT".into(),
                        message: msg,
                    });
                }
            }
        }

        if let Err(e) = self.validate_and_filter_servo_targets(&mut targets, epoch_id) {
            let (reason, state, code) = match &e {
                SafetyError::EStopActive => ("ESTOP", "failed", "ESTOP_ACTIVE"),
                SafetyError::FaultActive(_) => ("FAULT", "failed", "FAULT_ACTIVE"),
                SafetyError::CollisionHold(_) => ("COLLISION_HOLD", "cancelled", "COLLISION_HOLD"),
                SafetyError::SoftLimitExceeded { .. } => ("SOFT_LIMIT_EXCEEDED", "failed", "SOFT_LIMIT_EXCEEDED"),
                SafetyError::WorkspaceViolation { .. } => ("WORKSPACE_VIOLATION", "failed", "WORKSPACE_VIOLATION"),
                SafetyError::NotArmed(_) => ("NOT_ARMED", "failed", "NOT_ARMED"),
                _ => ("SAFETY_REJECTED", "failed", "SAFETY_ERROR"),
            };
            self.terminate_policy_run(reason, state, Some(code), Some(e.to_string()));
            return Err(ArmError::Safety(e));
        }

        // CRITICAL 1: NEVER touch self.last_heartbeat here!
        // CRITICAL 2: NEVER clear self.servo_blocked_hold here!
        self.last_motion_frame = Instant::now();
        self.release_gripper_stall_on_reverse(&targets);
        let _ = self.safety.mark_motion_started();

        self.target_positions = targets.clone();
        self.servo_target = Some(targets.clone());
        self.servo_stream_active = true;
        self.servo_settled_ticks.fill(0);

        if let Some(run) = self.policy_run.as_mut().filter(|r| r.action_id == action_id) {
            run.steps += 1;
            run.last_input_age_ms = input_age_ms;
            run.last_vision_age_ms = vision_age_ms;
            run.vision_frame_id = vision_frame_id;
            run.last_shadow_target = Some(targets.clone());
            run.last_live_target = Some(targets);
            self.last_policy_obs = Some(run.to_observation(true));
        }

        Ok(())
    }

    /// Vision hold of a live run: drop its servo target so the control loop brakes now
    /// (instead of tracking the last target until the motion-frame timeout). The next target
    /// is checked against the measured pose with the first-step limit.
    pub fn hold_policy_run(&mut self, action_id: u64) {
        let Some(run) = self.policy_run.as_mut().filter(|r| r.action_id == action_id && r.mode == crate::policy_runtime::PolicyMode::Live) else { return; };
        run.last_live_target = None;
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        self.servo_ws_min_depths = None;
        self.target_positions = self.commanded_positions.clone();
    }

    /// Set the run's [`crate::policy_runtime::max_policy_input_age_ms`] bound.
    pub fn set_policy_max_input_age(&mut self, action_id: u64, max_ms: u64) {
        if let Some(run) = self.policy_run.as_mut().filter(|r| r.action_id == action_id) {
            run.max_input_age_ms = max_ms;
        }
    }

    /// Set the run's period (`policy_stats.period_us`).
    pub fn set_policy_period_us(&mut self, action_id: u64, period_us: u64) {
        let Some(run) = self.policy_run.as_mut().filter(|r| r.action_id == action_id) else { return; };
        run.step_stats.set_period_us(period_us);
        let stats = run.step_stats.report();
        self.set_action_policy_stats(action_id, stats);
    }

    /// Record one inference of the run (see [`crate::policy_runtime::PolicyStepStats::record`])
    /// and publish `policy_stats` in the action status.
    pub fn record_policy_step_timing(&mut self, action_id: u64, infer_us: u64, step_us: u64) {
        let Some(run) = self.policy_run.as_mut().filter(|r| r.action_id == action_id) else { return; };
        run.step_stats.record(infer_us, step_us);
        let stats = run.step_stats.report();
        self.set_action_policy_stats(action_id, stats);
    }

    fn set_action_policy_stats(&mut self, action_id: u64, stats: crate::motion::PolicyStats) {
        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == action_id) {
            a.policy_stats = Some(stats);
        }
    }

    pub fn stop_policy_run(&mut self, action_id: Option<u64>, reason: &str) -> bool {
        let matches = match (self.policy_run.as_ref(), action_id) {
            (Some(_), None) => true,
            (Some(r), Some(id)) => r.action_id == id,
            (None, _) => false,
        };
        if !matches {
            return false;
        }
        self.terminate_policy_run(
            reason,
            "cancelled",
            Some(reason),
            Some(format!("Policy stopped: {reason}")),
        );
        true
    }

    pub fn finish_policy_run(&mut self, action_id: u64, reason: &str) -> bool {
        if !self.is_policy_run_active(action_id) {
            return false;
        }
        // a fault or E-stop that the runner has not seen yet is not a normal completion
        if let SafetyState::Fault(r) = self.safety.state().clone() {
            self.terminate_policy_run("FAULT", "failed", Some("FAULT"), Some(format!("Fault active: {r:?}")));
            return true;
        }
        if self.estop_pending() || matches!(self.safety.state(), SafetyState::EStop) {
            self.terminate_policy_run("ESTOP", "failed", Some("ESTOP_ACTIVE"), Some("E-stop latched".into()));
            return true;
        }
        self.terminate_policy_run(reason, "done", None, None);
        true
    }

    pub fn fail_policy_run(
        &mut self,
        action_id: u64,
        reason: &str,
        message: impl Into<String>,
    ) -> bool {
        if !self.is_policy_run_active(action_id) {
            return false;
        }
        self.terminate_policy_run(reason, "failed", Some(reason), Some(message.into()));
        true
    }

    /// Append the outcome of a supervised_first_run live run on real hardware to the device
    /// qualification ledger. Passed = ran to its deadline (`MAX_SECONDS`, action done).
    fn record_policy_qualification(&self, run: &crate::policy_runtime::PolicyRun, stop_reason: &str, action_state: &str) {
        use crate::policy_runtime::{qualification, AuthorizationTier, PolicyMode};
        if run.mode != PolicyMode::Live || self.policy_live_allowed() { return; }
        let Some(gate) = run.verified_release_gate.as_ref().filter(|g| g.authorization_tier() == AuthorizationTier::SupervisedFirstRun) else { return; };
        let Some(path) = self.policy_ledger_path.as_deref() else {
            tracing::warn!("No policy qualification ledger configured; supervised run {} not recorded", run.action_id);
            return;
        };
        let passed = stop_reason == "MAX_SECONDS" && action_state == "done";
        let record = qualification::QualificationRecord::for_run(gate, run.action_id, run.steps, stop_reason, passed)
            .with_policy_stats(run.step_stats.report());
        if let Err(e) = qualification::append_record(path, &record) {
            tracing::warn!("Cannot append policy qualification record to {}: {e}", path.display());
        }
    }

    fn terminate_policy_run(
        &mut self,
        stop_reason: &str,
        action_state: &str,
        action_code: Option<&str>,
        action_message: Option<String>,
    ) {
        let Some(mut run) = self.policy_run.take() else {
            return;
        };
        run.stop_reason = Some(stop_reason.to_string());
        self.set_action_policy_stats(run.action_id, run.step_stats.report());
        self.record_policy_qualification(&run, stop_reason, action_state);
        if run.mode == crate::policy_runtime::PolicyMode::Live {
            self.servo_target = None;
            self.servo_settled_ticks.fill(0);
            self.servo_ws_min_depths = None;
            self.target_positions = self.commanded_positions.clone();
        }
        self.last_policy_obs = Some(run.to_observation(false));
        self.finish_action(
            run.action_id,
            action_state,
            action_code,
            action_message,
            None,
            None,
            None,
        );
    }
}

/// [`SupervisorInner::keepout_sweep`] with an explicit arm model (planning outside the lock).
pub fn keepout_sweep_with(
    model: &crate::chain::ArmModel,
    ws: &crate::model::WorkspacePolicy,
    cam: &crate::model::CameraMount,
    at: &dyn Fn(f64) -> Vec<f32>,
    travel: &dyn Fn(f64, f64) -> Vec<f64>,
    knots: &[f64],
    budget: usize,
) -> Result<Vec<Vec<f32>>, SafetyError> {
    crate::chain::WorkspaceEvaluator::new(model, ws, cam)
        .sweep_knots(at, travel, knots, budget, "sweep")
        .map_err(SafetyError::from_workspace)
}

/// Workspace rule of [`SupervisorInner::check_workspace_path`] with an explicit arm model:
/// reject a sample that worsens or newly intrudes on any `(geometry_id, constraint_id)`
/// relative to `start` (per-constraint monotonic escape rule).
pub fn check_path_with<'a>(
    model: &crate::chain::ArmModel,
    ws: &crate::model::WorkspacePolicy,
    cam: &crate::model::CameraMount,
    start: &[f32],
    samples: impl IntoIterator<Item = &'a [f32]>,
) -> Result<(), SafetyError> {
    let eval = crate::chain::WorkspaceEvaluator::new(model, ws, cam);
    let mut c0 = eval.alloc_buf();
    eval.eval_into(start, &mut c0, "plan").map_err(SafetyError::from_workspace)?;
    let mut min_depths = eval.init_min_depths(&c0);
    let mut clr = eval.alloc_buf();
    for q in samples {
        eval.eval_into(q, &mut clr, "plan").map_err(SafetyError::from_workspace)?;
        eval.check_and_advance(&clr, &mut min_depths, "plan").map_err(SafetyError::from_workspace)?;
    }
    Ok(())
}

/// Thread-safe Arm Supervisor managing the robotic arm daemon
pub struct ArmSupervisor {
    inner: Arc<RwLock<SupervisorInner>>,
    telemetry_tx: broadcast::Sender<ArmObservation>,
    pub e_stop_latched: Arc<AtomicBool>,
    pub requested_estop_count: Arc<AtomicU64>,
    tick_stats: std::sync::Mutex<crate::diagnostics::tick::TickStats>,
    config_write_mutex: Arc<tokio::sync::Mutex<()>>,
}

impl ArmSupervisor {
    pub fn new(config: ArmConfig) -> Self {
        let (tx, _) = broadcast::channel(128);
        let e_stop_latched = Arc::new(AtomicBool::new(false));
        let requested_estop_count = Arc::new(AtomicU64::new(0));
        let mut inner = SupervisorInner::new(config);
        inner.e_stop_latched = e_stop_latched.clone();
        inner.requested_estop_count = requested_estop_count.clone();
        Self {
            inner: Arc::new(RwLock::new(inner)),
            telemetry_tx: tx,
            e_stop_latched,
            requested_estop_count,
            tick_stats: std::sync::Mutex::new(crate::diagnostics::tick::TickStats::default()),
            config_write_mutex: Arc::new(tokio::sync::Mutex::new(())),
        }
    }

    pub fn config_write_lock(&self) -> Arc<tokio::sync::Mutex<()>> {
        Arc::clone(&self.config_write_mutex)
    }

    pub async fn set_operator_auth_enforced(&self, enforced: bool) {
        self.inner.write().await.operator_auth_enforced = enforced;
    }

    pub fn subscribe_telemetry(&self) -> broadcast::Receiver<ArmObservation> {
        self.telemetry_tx.subscribe()
    }

    #[doc(hidden)]
    pub fn inner(&self) -> &Arc<RwLock<SupervisorInner>> {
        &self.inner
    }

    pub async fn get_observation(&self) -> ArmObservation {
        let inner = self.inner.read().await;
        inner.current_observation()
    }

    pub async fn get_schema(&self) -> ObservationSchema {
        self.inner.read().await.schema()
    }

    /// Records the `--follower` model name for `GET /api/v1/safety/describe`.
    pub async fn set_follower(&self, name: &str) {
        self.inner.write().await.follower = Some(name.to_string());
    }

    pub async fn get_config(&self) -> ArmConfig {
        let inner = self.inner.read().await;
        inner.config.clone()
    }

    pub async fn set_driver(&self, mut driver: Box<dyn ArmDriver>) {
        driver.set_abort_flag(self.e_stop_latched.clone());
        let mut inner = self.inner.write().await;
        driver.set_gripper_index(inner.config.gripper_index());
        let mut drv: Box<dyn ArmDriver> = Box::new(crate::joint_zero::ZeroOffsetDriver::new(driver, &inner.joint_zero_offsets));
        let caps = drv.capabilities();
        if caps.load_feedback {
            inner.loads = vec![f32::NAN; inner.config.dof];
        }
        if let Ok(states) = drv.read_joint_states() {
            let loads = drv.joint_loads();
            if !states.is_empty() && drv.is_feedback_valid() && SupervisorInner::states_valid(&states, caps, inner.config.dof, loads.as_deref()) {
                inner.sync_driver_feedback(&states, loads.as_deref());
                for (i, s) in states.iter().enumerate().take(inner.config.dof) {
                    if s.position.is_finite() {
                        inner.commanded_positions[i] = s.position;
                        inner.target_positions[i] = s.position;
                    }
                }
            }
        }
        inner.apply_driver_limits(drv.as_ref());
        inner.driver = Some(std::sync::Mutex::new(drv));
    }

    pub fn with_driver(self, mut driver: Box<dyn ArmDriver>) -> Self {
        driver.set_abort_flag(self.e_stop_latched.clone());
        if let Ok(mut inner) = self.inner.try_write() {
            driver.set_gripper_index(inner.config.gripper_index());
            if driver.capabilities().load_feedback {
                inner.loads = vec![f32::NAN; inner.config.dof];
            }
            let driver: Box<dyn ArmDriver> = Box::new(crate::joint_zero::ZeroOffsetDriver::new(driver, &inner.joint_zero_offsets));
            inner.apply_driver_limits(driver.as_ref());
            inner.driver = Some(std::sync::Mutex::new(driver));
        }
        self
    }

    pub async fn set_torque(&self, enable: bool) -> Result<SafetyState, ArmError> {
        if enable && self.e_stop_latched.load(Ordering::SeqCst) {
            return Err(ArmError::Safety(SafetyError::EStopActive));
        }
        let mut inner = self.inner.clone().write_owned().await;
        // Post-lock double-check: verify atomic e-stop latch again after acquiring write lock
        if enable && inner.estop_pending() {
            return Err(ArmError::Safety(SafetyError::EStopActive));
        }
        tokio::task::spawn_blocking(move || inner.set_torque(enable))
            .await
            .map_err(|e| ArmError::Internal(format!("set_torque blocking task failed: {e}")))?
            .map_err(ArmError::Safety)
    }

    pub async fn heartbeat(&self, client_id: &str, epoch_id: u64) -> Result<(), SafetyError> {
        let mut inner = self.inner.write().await;
        inner.heartbeat(client_id, epoch_id)
    }

    pub async fn acquire_lease(&self, client_id: &str) -> Result<u64, (String, u64)> {
        let mut inner = self.inner.write().await;
        inner.acquire_lease(client_id)
    }

    pub async fn preempt_lease(&self, conn: &str, label: &str) -> Result<(u64, Option<(String, u64)>), PreemptError> {
        let mut inner = self.inner.write().await;
        inner.preempt_lease(conn, label)
    }

    pub async fn is_handover_settling(&self) -> bool {
        let inner = self.inner.read().await;
        inner.handover_settling_since.is_some()
    }

    pub async fn release_lease(&self, client_id: &str, generation: u64) -> bool {
        let mut inner = self.inner.write().await;
        inner.release_lease(client_id, generation)
    }

    pub async fn is_lease_active(&self) -> bool {
        let inner = self.inner.read().await;
        inner.is_lease_active()
    }

    pub async fn validate_servo_lease(&self, client_id: &str, generation: u64) -> bool {
        let mut inner = self.inner.write().await;
        inner.validate_servo_lease(client_id, generation)
    }

    pub async fn set_torques(&self, torques: Vec<f32>) {
        let mut inner = self.inner.write().await;
        inner.torques = torques;
    }

    pub async fn is_gripper_stalled(&self) -> bool {
        let inner = self.inner.read().await;
        inner.gripper_stalled
    }

    pub async fn is_gravity_compensated(&self) -> bool {
        let inner = self.inner.read().await;
        inner.gravity_output_enabled()
    }

    pub async fn get_gravity_torques(&self) -> Vec<f32> {
        let inner = self.inner.read().await;
        inner.gravity_torques.clone()
    }

    pub async fn set_gravity_compensated(&self, enabled: bool) {
        let mut inner = self.inner.write().await;
        inner.gravity_compensated = enabled;
    }

    pub async fn get_gravity_scale(&self) -> f32 {
        let inner = self.inner.read().await;
        inner.gravity_scale
    }

    pub async fn set_gravity_scale(&self, scale: f32) {
        let mut inner = self.inner.write().await;
        inner.gravity_scale = scale.clamp(0.0, 1.0);
    }

    pub async fn get_gravity_torques_ramped(&self) -> Vec<f32> {
        let inner = self.inner.read().await;
        inner.gravity_torques_ramped.clone()
    }

    pub async fn servo(&self, targets: Vec<f32>, epoch_id: u64) -> Result<(), ArmError> {
        if self.e_stop_latched.load(Ordering::SeqCst) {
            return Err(ArmError::Safety(SafetyError::EStopActive));
        }
        let mut inner = self.inner.write().await;
        if inner.estop_pending() {
            return Err(ArmError::Safety(SafetyError::EStopActive));
        }
        inner.servo(targets, epoch_id)?;
        Ok(())
    }

    pub async fn execute_stop(&self, policy: StopPolicy) {
        let mut inner = self.inner.write().await;
        inner.execute_stop(policy);
    }

    pub async fn set_lease_enabled(&self, enabled: bool) {
        let mut inner = self.inner.write().await;
        inner.lease_enabled = enabled;
    }

    pub async fn set_temperatures(&self, temps: Vec<f32>) {
        let mut inner = self.inner.write().await;
        inner.temperatures = temps;
    }

    pub async fn jog(
        &self,
        target_positions: Vec<f32>,
        duration_secs: Option<f64>,
        epoch_id: Option<u64>,
    ) -> Result<u64, ArmError> {
        let mut inner = self.inner.write().await;
        inner.jog(target_positions, duration_secs, epoch_id)
    }

    pub async fn home(
        &self,
        duration_secs: Option<f64>,
        epoch_id: Option<u64>,
    ) -> Result<u64, ArmError> {
        let mut inner = self.inner.write().await;
        let home_pos = inner.config.home_positions.clone();
        inner.jog_action(home_pos, duration_secs, epoch_id, "home").map(|(epoch, _, _)| epoch)
    }

    pub async fn stop_motion(&self) { self.inner.write().await.stop_motion(); }

    pub async fn shutdown_ready(&self) -> bool { self.inner.read().await.shutdown_ready() }

    pub async fn is_at_safe_resting_pose(&self) -> bool {
        let inner = self.inner.read().await;
        inner.is_at_safe_resting_pose()
    }

    pub async fn trigger_e_stop(&self) -> (SafetyState, u64) {
        self.requested_estop_count.fetch_add(1, Ordering::SeqCst);
        self.e_stop_latched.store(true, Ordering::SeqCst);
        let inner = Arc::clone(&self.inner);
        if let Ok(handle) = tokio::runtime::Handle::try_current() {
            let jh = handle.spawn(async move {
                let mut guard = inner.write().await;
                guard.consume_pending_estop();
                (guard.safety.state().clone(), guard.safety.epoch_id())
            });
            match jh.await {
                Ok(res) => res,
                Err(_) => {
                    let guard = self.inner.read().await;
                    (guard.safety.state().clone(), guard.safety.epoch_id())
                }
            }
        } else {
            let mut guard = inner.write().await;
            guard.consume_pending_estop();
            (guard.safety.state().clone(), guard.safety.epoch_id())
        }
    }

    pub async fn clear_fault(&self) -> Result<SafetyState, ArmError> {
        let mut inner = self.inner.clone().write_owned().await;
        let res = tokio::task::spawn_blocking(move || inner.clear_fault())
            .await
            .map_err(|e| ArmError::Internal(format!("clear_fault blocking task failed: {e}")))?;
        res.map_err(ArmError::Safety)
    }

    pub async fn apply_stop_policy(&self, policy: StopPolicy) {
        let mut inner = self.inner.write().await;
        inner.apply_stop_policy(policy);
    }

    pub async fn is_unloading(&self) -> bool {
        let inner = self.inner.read().await;
        inner.pending_normal_unload || inner.active_trajectory.is_some() || inner.unload_settle.is_some()
    }

    pub async fn has_active_trajectory(&self) -> bool {
        let inner = self.inner.read().await;
        inner.active_trajectory.is_some()
    }

    /// Execute a control cycle step
    pub async fn tick(&self, dt: f64) -> ArmObservation {
        let started = Instant::now();
        let (obs, lock_held, control_hz, pending_zc) = {
            let mut inner = self.inner.write().await;
            let locked = Instant::now();
            inner.defer_zero_check_persist = true;
            let obs = inner.tick(dt);
            inner.defer_zero_check_persist = false;
            let pending_zc = inner.pending_zero_check_persist.take();
            (obs, locked.elapsed(), inner.config.control_frequency_hz, pending_zc)
        };
        if let Some(job) = pending_zc {
            let sup_inner = Arc::clone(&self.inner);
            let cw_lock = Arc::clone(&self.config_write_mutex);
            tokio::spawn(async move {
                let _cw = cw_lock.lock().await;
                let path = job.write.path.clone();
                let text = job.write.text.clone();
                let staged_res = tokio::task::spawn_blocking(move || {
                    crate::persist::StagedWrite::stage(path, text)
                }).await;
                let staged = match staged_res {
                    Ok(Ok(s)) => s,
                    Ok(Err(e)) => {
                        tracing::warn!("Failed to stage gripper calibration after zero_check: {e}");
                        let mut inner = sup_inner.write().await;
                        if inner.zero_check_persisting == Some(job.action_id) {
                            inner.zero_check_persisting = None;
                        }
                        let now = inner.now_s();
                        if let Some(a) = inner.last_action.as_mut().filter(|a| a.id == job.action_id && a.is_running()) {
                            a.state = "failed".into();
                            a.code = Some("PERSIST_FAILED".into());
                            a.message = Some(format!("Failed to persist gripper calibration: {e}"));
                            a.finished_at = Some(now);
                        }
                        return;
                    }
                    Err(e) => {
                        tracing::warn!("Gripper calibration stage task failed: {e}");
                        let mut inner = sup_inner.write().await;
                        if inner.zero_check_persisting == Some(job.action_id) {
                            inner.zero_check_persisting = None;
                        }
                        let now = inner.now_s();
                        if let Some(a) = inner.last_action.as_mut().filter(|a| a.id == job.action_id && a.is_running()) {
                            a.state = "failed".into();
                            a.code = Some("PERSIST_FAILED".into());
                            a.message = Some(format!("Failed to persist gripper calibration: {e}"));
                            a.finished_at = Some(now);
                        }
                        return;
                    }
                };

                let dir_sync = {
                    let mut inner = sup_inner.write().await;
                    let still_persisting = inner.zero_check_persisting == Some(job.action_id);
                    let action_running = inner.last_action.as_ref().is_some_and(|a| a.id == job.action_id && a.is_running());
                    let auth_ok = job.operator_auth.as_ref().is_none_or(|(a, tok)| !a.is_enforced() || a.is_operator(tok.as_deref()));
                    let lease_ok = match &job.lease_owner {
                        None => true,
                        Some((c, g)) => inner.active_lease.as_ref().is_some_and(|l| {
                            l.acquired_at.elapsed() < inner.lease_timeout && l.client_id == *c && l.generation == *g
                        }),
                    };
                    let rev_ok = inner.gripper_calib_revision == job.base_revision;

                    if !still_persisting || !action_running || !lease_ok {
                        staged.abort();
                        if inner.zero_check_persisting == Some(job.action_id) {
                            inner.zero_check_persisting = None;
                        }
                        if action_running {
                            let now = inner.now_s();
                            if let Some(a) = inner.last_action.as_mut().filter(|a| a.id == job.action_id && a.is_running()) {
                                a.state = "cancelled".into();
                                a.code = Some(if !lease_ok { "LEASE_LOST" } else { "CANCELLED" }.into());
                                a.message = Some(if !lease_ok {
                                    "Control lease lost or expired before gripper calibration persisted".into()
                                } else {
                                    "Gripper zero check cancelled before calibration persisted".into()
                                });
                                a.finished_at = Some(now);
                            }
                        }
                        return;
                    }

                    if !auth_ok {
                        staged.abort();
                        if inner.zero_check_persisting == Some(job.action_id) {
                            inner.zero_check_persisting = None;
                        }
                        let now = inner.now_s();
                        if let Some(a) = inner.last_action.as_mut().filter(|a| a.id == job.action_id && a.is_running()) {
                            a.state = "failed".into();
                            a.code = Some("OPERATOR_REQUIRED".into());
                            a.message = Some("Operator session expired or locked before gripper calibration persisted".into());
                            a.finished_at = Some(now);
                        }
                        return;
                    }

                    if !rev_ok {
                        staged.abort();
                        if inner.zero_check_persisting == Some(job.action_id) {
                            inner.zero_check_persisting = None;
                        }
                        let now = inner.now_s();
                        if let Some(a) = inner.last_action.as_mut().filter(|a| a.id == job.action_id && a.is_running()) {
                            a.state = "failed".into();
                            a.code = Some("POLICY_REVISION_MISMATCH".into());
                            a.message = Some("Gripper calibration revision changed concurrently".into());
                            a.finished_at = Some(now);
                        }
                        return;
                    }

                    // Build complete validated candidate BEFORE atomic rename so memory commit is infallible.
                    let cand = match inner.build_gripper_calibration_candidate(Some(job.offset_rad), job.jaw_opening.clone()) {
                        Ok((cand, _)) => cand,
                        Err(e) => {
                            staged.abort();
                            if inner.zero_check_persisting == Some(job.action_id) {
                                inner.zero_check_persisting = None;
                            }
                            let now = inner.now_s();
                            if let Some(a) = inner.last_action.as_mut().filter(|a| a.id == job.action_id && a.is_running()) {
                                a.state = "failed".into();
                                a.code = Some("INVALID_CALIBRATION".into());
                                a.message = Some(format!("Gripper calibration rejected: {e}"));
                                a.finished_at = Some(now);
                            }
                            return;
                        }
                    };

                    // Re-verify operator session & lease (not expired) immediately before atomic rename
                    // because OperatorAuth or lease timeout can expire during driver limit validation.
                    let auth_still_ok = job.operator_auth.as_ref().is_none_or(|(a, tok)| !a.is_enforced() || a.is_operator(tok.as_deref()));
                    let lease_still_ok = match &job.lease_owner {
                        None => true,
                        Some((c, g)) => inner.active_lease.as_ref().is_some_and(|l| {
                            l.acquired_at.elapsed() < inner.lease_timeout && l.client_id == *c && l.generation == *g
                        }),
                    };
                    if !lease_still_ok {
                        staged.abort();
                        if inner.zero_check_persisting == Some(job.action_id) {
                            inner.zero_check_persisting = None;
                        }
                        let now = inner.now_s();
                        if let Some(a) = inner.last_action.as_mut().filter(|a| a.id == job.action_id && a.is_running()) {
                            a.state = "cancelled".into();
                            a.code = Some("LEASE_LOST".into());
                            a.message = Some("Control lease lost or expired before gripper calibration persisted".into());
                            a.finished_at = Some(now);
                        }
                        return;
                    }
                    if !auth_still_ok {
                        staged.abort();
                        if inner.zero_check_persisting == Some(job.action_id) {
                            inner.zero_check_persisting = None;
                        }
                        let now = inner.now_s();
                        if let Some(a) = inner.last_action.as_mut().filter(|a| a.id == job.action_id && a.is_running()) {
                            a.state = "failed".into();
                            a.code = Some("OPERATOR_REQUIRED".into());
                            a.message = Some("Operator session expired or locked before gripper calibration persisted".into());
                            a.finished_at = Some(now);
                        }
                        return;
                    }

                    let dir_sync = match staged.rename_only() {
                        Ok(ds) => ds,
                        Err(e) => {
                            tracing::warn!("Failed to publish gripper calibration after zero_check: {e}");
                            if inner.zero_check_persisting == Some(job.action_id) {
                                inner.zero_check_persisting = None;
                            }
                            let now = inner.now_s();
                            if let Some(a) = inner.last_action.as_mut().filter(|a| a.id == job.action_id && a.is_running()) {
                                a.state = "failed".into();
                                a.code = Some("PERSIST_FAILED".into());
                                a.message = Some(format!("Failed to persist gripper calibration: {e}"));
                                a.finished_at = Some(now);
                            }
                            return;
                        }
                    };

                    // Infallible commit after atomic rename; keep zero_check_persisting set while parent dir fsync runs outside lock.
                    inner.commit_gripper_calibration_candidate(cand);
                    inner.gripper_calib_revision += 1;
                    dir_sync
                };

                let outcome = tokio::task::spawn_blocking(move || dir_sync.sync_parent_dir())
                    .await
                    .unwrap_or_else(|e| crate::persist::PublishOutcome::PublishedDurabilityUnconfirmed(e.to_string()));
                let mut inner = sup_inner.write().await;
                if inner.zero_check_persisting == Some(job.action_id) {
                    inner.zero_check_persisting = None;
                }
                let now = inner.now_s();
                if let Some(a) = inner.last_action.as_mut().filter(|a| a.id == job.action_id) {
                    if a.is_running() {
                        a.state = "done".into();
                        a.progress = 1.0;
                        a.finished_at = Some(now);
                    }
                    match outcome {
                        crate::persist::PublishOutcome::Published => {
                            a.durability_confirmed = Some(true);
                        }
                        crate::persist::PublishOutcome::PublishedDurabilityUnconfirmed(w) => {
                            a.durability_confirmed = Some(false);
                            a.durability_warning = Some(w);
                        }
                    }
                }
            });
        }
        // Broadcast observation to any active WebSocket listeners
        let _ = self.telemetry_tx.send(obs.clone());
        let late = !dt.is_finite() || dt > 2.0 / f64::from(control_hz.max(1));
        self.tick_stats.lock().unwrap_or_else(|e| e.into_inner()).record(started.elapsed(), lock_held, late);
        obs
    }

    /// Timing of the last ticks (`GET /api/v1/diagnostics/tick`).
    pub fn tick_stats(&self) -> crate::diagnostics::tick::TickSnapshot {
        self.tick_stats.lock().unwrap_or_else(|e| e.into_inner()).snapshot()
    }

    pub async fn set_mock_gripper_stall_offset(&self, offset: Option<f32>) {
        self.inner.write().await.mock_gripper_stall_offset = offset;
    }

    pub async fn gripper_calibration_response(&self) -> crate::gripper_calib::GripperCalibrationResponse {
        self.inner.read().await.gripper_calibration_response()
    }
}

#[cfg(test)]
#[path = "supervisor_servo_order_tests.rs"]
mod servo_order_tests;
