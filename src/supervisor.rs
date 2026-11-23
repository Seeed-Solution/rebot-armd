use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::broadcast;
use crate::supervisor_lock::SupervisorLock;
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::rt_core::scratch::{copy_into, fill_n};
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

// supervisor leaf modules (armd 1.6 RT-core foundation)
mod driver_transitions;
mod config;
mod motion;
mod streams;
mod actions;

/// Broker side of the servo, teleop and policy latest-target path (track D); the WS loop
/// and the policy runner submit through it once they route through the mailbox.
pub use streams::targets;

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

/// What [`SupervisorInner::tick_publish`] hands back.
pub struct TickPublished {
    /// A copy of the published observation (asked for with `keep`).
    pub kept: Option<ArmObservation>,
    /// End of the tick body (the tick phase mark), None without phase timing.
    pub body_end: Option<Instant>,
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
    pub(crate) sim_time_s: f64,
    pub(crate) last_still_since_s: Option<f64>,
    pub(crate) still_anchor_positions: Vec<f32>,
    pub(crate) still_window_ever_moved: bool,
    pub(crate) table_still_first_rx_instants: Vec<Instant>,
    pub(crate) table_still_last_rx_instants: Vec<Instant>,
    pub(crate) table_still_anchor_positions: Vec<f32>,
    pub torques: Vec<f32>,
    pub loads: Vec<f32>,
    pub temperatures: Vec<f32>,
    pub target_positions: Vec<f32>,
    pub servo_target: Option<Vec<f32>>,
    /// The buffer `servo_target` reuses while it is None (`release_servo_target`), so a
    /// stop and the next target neither free nor allocate it.
    pub(crate) servo_target_spare: Vec<f32>,
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
    /// Why the last arm refused MIT tracking (`src/mit.rs` qualification gate); None = passed or
    /// MIT not requested.
    pub mit_gate_error: Option<String>,
    /// MIT tracking output owner and history (`src/mit.rs`).
    pub mit: crate::mit::MitRuntime,
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
    /// RT-owned state of this arm (`crate::rt_core`): per-joint feedback receive instants and
    /// sequence counters, the state publisher and the recording ring.
    pub core: crate::rt_core::ArmCore,
    /// Per joint, the receive-queue bound of its last sample ([`ArmDriver::feedback_queue_bounds`]);
    /// empty when the driver has no receive queue (the receive instant bounds the sample then).
    pub joint_feedback_queue_bounds: Vec<Option<Instant>>,
    /// Preallocated buffers of the tick (no allocation in a steady-state tick).
    pub(crate) scratch: crate::rt_core::scratch::TickScratch,
    /// Vectors of the tick's workspace evaluators (`servo_step_workspace`).
    pub(crate) ws_eval_bufs: crate::chain::EvalBufs,
    pub bus_loss_instant: Option<Instant>,
    pub e_stop_latched: Arc<AtomicBool>,
    pub requested_estop_count: Arc<AtomicU64>,
    pub applied_estop_count: u64,
    /// `applied_estop_count` for the bus driver's E-stop gate (`crate::rt_core::urgent`).
    pub estop_applied: Arc<AtomicU64>,
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
    /// True when startup `grasp::load_into` fell back to tool defaults (missing/rejected
    /// `grasp_setup.json`) and should adopt shared camera fields from `calibration.json`.
    pub grasp_setup_needs_camera_sync: bool,
    /// When true (inside ArmSupervisor::tick), zero_check(store_offset=true) defers disk persist
    /// and memory commit until after the lock is released.
    pub defer_zero_check_persist: bool,
    /// Joint quintics installed from a [`PreparedQuintic`] (planned without the lock).
    #[doc(hidden)]
    pub prepared_quintics_used: u64,
    /// Phase marks of the running tick ([`Self::tick_phase`]); µs per phase.
    tick_phase_mark: Option<Instant>,
    tick_phases_us: [u32; 5],
    /// Action id whose zero_check(store_offset=true) disk persist is currently in flight.
    pub zero_check_persisting: Option<u64>,
    /// Pending disk persist + commit payload produced by finish_action during ArmSupervisor::tick.
    pub pending_zero_check_persist: Option<PendingZeroCheckPersist>,
    /// Whether operator auth is enforced (exposed in GET /api/v1/safety/describe).
    pub operator_auth_enforced: bool,
    /// Path to persisted tool commissioning confirmations (`tool_commissioning.json`).
    pub tool_commissioning_path: Option<PathBuf>,
    /// Timestamp when the active tool's commissioning was confirmed (if any).
    pub tool_commissioning_confirmed_at: Option<String>,
    /// Monotonic revision for tool commissioning state.
    pub tool_commissioning_revision: u64,
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
            sim_time_s: 0.0,
            last_still_since_s: None,
            still_anchor_positions: Vec::new(),
            still_window_ever_moved: false,
            table_still_first_rx_instants: Vec::new(),
            table_still_last_rx_instants: Vec::new(),
            table_still_anchor_positions: Vec::new(),
            torques: vec![0.0; dof],
            loads: vec![0.0; dof],
            temperatures: vec![25.0; dof],
            target_positions,
            servo_target: None,
            servo_target_spare: Vec::with_capacity(crate::rt_core::mailbox::MAX_DOF.max(dof)),
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
            mit_gate_error: None,
            mit: Default::default(),
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
            core: crate::rt_core::ArmCore::new(dof),
            joint_feedback_queue_bounds: Vec::new(),
            scratch: crate::rt_core::scratch::TickScratch::with_dof(dof),
            ws_eval_bufs: Default::default(),
            bus_loss_instant: None,
            e_stop_latched: Arc::new(AtomicBool::new(false)),
            requested_estop_count: Arc::new(AtomicU64::new(0)),
            applied_estop_count: 0,
            estop_applied: Arc::new(AtomicU64::new(0)),
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
            grasp_setup_needs_camera_sync: false,
            defer_zero_check_persist: false,
            prepared_quintics_used: 0,
            tick_phase_mark: None,
            tick_phases_us: [0; 5],
            zero_check_persisting: None,
            pending_zero_check_persist: None,
            operator_auth_enforced: false,
            tool_commissioning_path: None,
            tool_commissioning_confirmed_at: None,
            tool_commissioning_revision: 1,
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

    /// The E-stop gate of this arm's bus driver: a request not yet applied by the tick.
    pub fn estop_gate(&self) -> crate::rt_core::urgent::EstopGate {
        crate::rt_core::urgent::EstopGate::new(self.requested_estop_count.clone(), self.estop_applied.clone())
    }

    /// An E-stop request the tick has not applied yet (not the latch: hold output while
    /// latched still reaches the motors).
    pub(crate) fn estop_request_unapplied(&self) -> bool {
        self.requested_estop_count.load(Ordering::SeqCst) != self.applied_estop_count
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
        keepout_sweep_with_tcp_tol(&self.config.arm_model, ws, cam, self.config.tcp_offset, crate::kinematics::WORKSPACE_EPS, at, travel, knots, budget)
    }

    pub fn keepout_sweep_with_tol(
        &self,
        ws: &crate::model::WorkspacePolicy,
        cam: &crate::model::CameraMount,
        escape_tol: f64,
        at: &dyn Fn(f64) -> Vec<f32>,
        travel: &dyn Fn(f64, f64) -> Vec<f64>,
        knots: &[f64],
        budget: usize,
    ) -> Result<Vec<Vec<f32>>, SafetyError> {
        keepout_sweep_with_tcp_tol(&self.config.arm_model, ws, cam, self.config.tcp_offset, escape_tol, at, travel, knots, budget)
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

    /// [`check_workspace_path`](Self::check_workspace_path) with explicit escape jitter tolerance.
    pub fn check_workspace_path_with_tol<'a>(
        &self,
        start: &[f32],
        samples: impl IntoIterator<Item = &'a [f32]>,
        escape_tol: f64,
    ) -> Result<(), SafetyError> {
        let Some((ws, cam)) = self.workspace_policy() else { return Ok(()); };
        check_path_with_tcp_tol(&self.config.arm_model, ws, cam, self.config.tcp_offset, escape_tol, start, samples)
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
        check_path_with_tcp_tol(&self.config.arm_model, ws, cam, self.config.tcp_offset, crate::kinematics::WORKSPACE_EPS, start, samples)
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
        self.workspace_samples_with_budget_tol(traj, budget, crate::kinematics::WORKSPACE_EPS)
    }

    /// [`workspace_samples`](Self::workspace_samples) with explicit escape jitter tolerance.
    pub fn workspace_samples_with_tol(&self, traj: &MultiJointQuinticTrajectory, escape_tol: f64) -> Result<Vec<Vec<f32>>, SafetyError> {
        self.workspace_samples_with_budget_tol(traj, MAX_KEEPOUT_EVALS_PER_PLAN, escape_tol)
    }

    fn workspace_samples_with_budget_tol(&self, traj: &MultiJointQuinticTrajectory, budget: usize, escape_tol: f64) -> Result<Vec<Vec<f32>>, SafetyError> {
        let Some((ws, cam)) = self.workspace_policy()
            else { return Ok(Self::trajectory_samples(traj)); };
        let knots: Vec<f64> = (0..=20).map(|k| traj.duration * k as f64 / 20.0).collect();
        let at = |t: f64| traj.sample(t).positions;
        let travel = |t0: f64, t1: f64| traj.travel_bound(t0, t1);
        self.keepout_sweep_with_tol(ws, cam, escape_tol, &at, &travel, &knots, budget)
    }

    /// Keepout-only sweep samples.
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
        self.gravity_compensated
            && self.config.dynamics_qualified
            && self.config.gravity_model.is_some()
            && self.config.dof >= self.config.arm_dof()
            && self.driver.as_ref().is_none_or(|d|
                d.lock().is_ok_and(|drv| drv.capabilities().torque_feedforward))
    }

    /// Adds the time since the previous mark to phase `i` of the running tick (no-op
    /// outside `ArmSupervisor::tick`).
    fn tick_phase(&mut self, i: usize) {
        if let Some(mark) = self.tick_phase_mark {
            let now = Instant::now();
            self.tick_phases_us[i] = self.tick_phases_us[i].saturating_add(crate::diagnostics::tick::us(now - mark));
            self.tick_phase_mark = Some(now);
        }
    }

    /// When `obs` (computed by [`Self::current_observation`]) was sampled and the absolute
    /// local deadline until which it stays correct with no write: the first clock-driven
    /// flip of the lease holder (`acquired_at + lease_timeout`), the previous-action window
    /// (`start_time + until`) or the 250 ms feedback freshness behind `feedback_valid` /
    /// `enable_state`, capped at `max_age` after the sample. Both come from the clocks the
    /// observation was computed from, so a later publication does not extend them.
    pub fn observation_valid_until(&self, obs: &ArmObservation, max_age: Duration) -> (Instant, Instant) {
        const FEEDBACK_FRESH: Duration = Duration::from_millis(250);
        let sampled = self.start_time + Duration::from_secs_f64(obs.timestamp.max(0.0));
        let mut until = sampled + max_age;
        if let Some(lease) = self.active_lease.as_ref() {
            until = until.min(lease.acquired_at + self.lease_timeout);
        }
        if let Some((_, end)) = self.previous_action.as_ref() {
            if obs.timestamp < *end { until = until.min(self.start_time + Duration::from_secs_f64(*end)); }
        }
        // ages are whole ms, truncated and read after `timestamp`: one more ms of margin
        if let Some(oldest) = obs.feedback_age_ms.iter().flatten().max() {
            let age = Duration::from_millis(oldest.saturating_add(1));
            if age <= FEEDBACK_FRESH { until = until.min(sampled + (FEEDBACK_FRESH - age)); }
        }
        (sampled, until.max(sampled))
    }

    /// The control loop's tick under the write guard of lock generation `generation`:
    /// [`Self::tick_control`], the observation filled in place into the publication's back
    /// slot ([`Self::fill_observation`]), and its publication (before the guard drops: no
    /// other write can come in between). No allocation in a steady-state tick; `keep` copies
    /// the observation for the caller (tests).
    pub fn tick_publish(&mut self, dt: f64, generation: u64, keep: bool) -> TickPublished {
        let timestamp = self.tick_control(dt);
        let mut obs = self.core.take_observation_buffer();
        self.fill_observation(timestamp, &mut obs);
        self.tick_phase(4);
        let body_end = self.tick_phase_mark.take();
        let control_hz = self.config.control_frequency_hz;
        let (sampled_at, valid_until) = self.observation_valid_until(&obs, Duration::from_secs_f64(2.0 / f64::from(control_hz.max(1))));
        let kept = keep.then(|| (*obs).clone());
        let SupervisorInner { core, measured_positions, measured_velocities, .. } = self;
        core.publish(obs, generation, sampled_at, valid_until, measured_positions, measured_velocities);
        TickPublished { kept, body_end }
    }

    pub fn current_observation(&self) -> ArmObservation {
        let mut obs = ArmObservation::blank();
        self.fill_observation(self.start_time.elapsed().as_secs_f64(), &mut obs);
        obs
    }

    /// [`Self::current_observation`] at `timestamp` written into `out`, reusing its memory: the
    /// control tick fills its published sample this way, without allocating while no value
    /// changes its shape (armd 1.6 RT core). Every field is overwritten.
    pub fn fill_observation(&self, timestamp: f64, out: &mut ArmObservation) {
        use crate::rt_core::scratch::{fill_n, set_opt, set_opt_eq};
        let tcp = if self.config.kinematics.is_some() {
            self.config.tcp_pose_f32(&self.measured_positions).map(|pose| {
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
        out.timestamp = timestamp;
        out.positions.clone_from(&self.measured_positions);
        out.velocities.clone_from(&self.measured_velocities);
        out.torques.clone_from(&self.torques);
        out.temperatures.clone_from(&self.temperatures);
        out.is_torqued = self.is_torqued;
        // `fault` is a function of the safety state and its code: rebuilt (a String) only
        // when either changed
        let state = self.safety.state();
        if out.safety_state != *state || out.fault.as_ref().map(|f| f.code) != self.safety.fault_code() {
            out.fault = self.safety.fault();
        }
        if out.safety_state != *state { out.safety_state = state.clone(); }
        out.epoch_id = self.safety.epoch_id();
        out.gravity_compensated = self.gravity_output_enabled();
        out.gravity_torques.clone_from(&self.gravity_torques);
        out.driver_capabilities = driver_capabilities;
        out.enable_state = self.driver.as_ref().map_or(if self.is_torqued { EnableState::Enabled } else { EnableState::Disabled }, |d| d.lock().map_or(EnableState::Unknown, |drv| drv.enable_state()));
        match self.driver.as_ref().map(|d| d.lock()) {
            Some(Ok(drv)) => drv.feedback_age_ms_into(&mut out.feedback_age_ms),
            _ => fill_n(&mut out.feedback_age_ms, Some(0), self.config.dof),
        }
        out.disable_pending = self.disable_pending;
        out.feedback_valid = self.hardware_feedback_fresh(Duration::from_millis(250));
        out.commanded_positions.clone_from(&self.commanded_positions);
        out.action.clone_from(&self.last_action);
        set_opt(&mut out.previous_action, self.previous_action.as_ref().filter(|(_, until)| timestamp < *until).map(|(a, _)| a));
        out.control_mode = if self.gravity_comp.is_some() { crate::model::ControlMode::GravityComp } else { crate::model::ControlMode::Position };
        self.gravity_comp_observation_into(&mut out.gravity_comp);
        match self.driver.as_ref().map(|d| d.lock()) {
            Some(Ok(drv)) => drv.with_range_check(&mut |rc| set_opt_eq(&mut out.driver_range_check, rc)),
            _ => out.driver_range_check = None,
        }
        // inline label: setting or clearing it never allocates or frees on the tick
        let holder = self.lease_holder_str();
        if out.lease_holder.as_deref() != holder { out.lease_holder = holder.map(crate::model::HolderLabel::new); }
        out.motion_limit.clone_from(&self.motion_limit);
        set_opt_eq(&mut out.workspace_violation, self.workspace_violation.as_ref());
        if out.motor_errors != self.motor_errors { out.motor_errors.clone_from(&self.motor_errors); }
        out.residual_torques = self.residual_torques;
        out.collision = self.collision_observation();
        self.power_observation_into(&mut out.power);
        self.gripper_zero_observation_into(&mut out.gripper_zero);
        out.tcp = tcp;
        out.gripper = gripper;
        if driver_capabilities.load_feedback && self.loads.len() == self.config.dof {
            set_opt(&mut out.loads, Some(&self.loads));
        } else {
            out.loads = None;
        }
        #[cfg(feature = "policy-runtime")]
        self.policy_observation_into(&mut out.policy);
        #[cfg(not(feature = "policy-runtime"))]
        { out.policy = None; }
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
        if let Some(msg) = self.measured_workspace_violation("park_release") {
            failures.push(msg);
        }
        (!failures.is_empty()).then(|| failures.join("; "))
    }

    /// Evaluate the active workspace policy (including `tcp_offset`, keep-outs, floor,
    /// max reach, and link capsules) at `self.measured_positions`.
    fn measured_workspace_violation(&self, phase: &str) -> Option<String> {
        let (ws, cam) = self.workspace_policy()?;
        let eval = crate::chain::WorkspaceEvaluator::new_with_tcp(
            &self.config.arm_model,
            ws,
            cam,
            self.config.tcp_offset,
        );
        let mut buf = eval.alloc_buf();
        match eval.eval_into(&self.measured_positions, &mut buf, phase) {
            Err(f) => Some(format!(
                "measured pose workspace evaluation failed ({}): {}",
                f.limit,
                f.detail.reason.as_deref().unwrap_or("invalid joint positions")
            )),
            Ok(()) => {
                if let Some((slot, &clr)) = buf
                    .as_slice()
                    .iter()
                    .enumerate()
                    .find(|(_, clr)| !clr.is_finite() || **clr < -crate::kinematics::WORKSPACE_EPS)
                {
                    let f = eval.describe_failure(slot, clr, -clr, 0.0, phase);
                    let detail = f.detail.reason.as_deref().unwrap_or(f.limit.as_str());
                    let geom = f.detail.geometry_id.as_deref().unwrap_or("end_link");
                    Some(format!(
                        "measured pose violates workspace constraint '{}' (point {} [{}]: {detail}, value {:.4} m vs bound {:.4} m)",
                        f.limit, f.point, geom, f.value, f.bound
                    ))
                } else {
                    None
                }
            }
        }
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
        if let Some(msg) = self.measured_workspace_violation("clear_fault") {
            failures.push(msg);
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

    pub(crate) fn consume_pending_estop(&mut self) -> bool {
        let req = self.requested_estop_count.load(Ordering::SeqCst);
        if req > self.applied_estop_count {
            self.applied_estop_count = req;
            self.estop_applied.store(req, Ordering::SeqCst);
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

    // ---- actions ----

    fn now_s(&self) -> f64 { self.start_time.elapsed().as_secs_f64() }

    // ---- grasp ----

    // ---- macro actions (crate::macro_action) ----

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

    /// One control tick and its observation ([`Self::tick_control`], then
    /// [`Self::current_observation`] at the tick's timestamp).
    pub fn tick(&mut self, dt: f64) -> ArmObservation {
        let timestamp = self.tick_control(dt);
        let mut obs = ArmObservation::blank();
        self.fill_observation(timestamp, &mut obs);
        obs
    }

    /// One control tick without building the observation; returns the tick's observation
    /// timestamp (the recording sample's). The control loop fills its published sample with
    /// [`Self::fill_observation`] at this timestamp.
    pub fn tick_control(&mut self, dt: f64) -> f64 {
        self.consume_pending_estop();
        // Immediate atomic e-stop latch check before any processing
        if self.e_stop_latched.load(Ordering::SeqCst) && !self.safety.state().is_stopped() {
            self.apply_stop_policy(StopPolicy::EStopLatch);
        }

        // Polling a cached sample never refreshes its hardware receive timestamp.
        // The driver is read into the tick's preallocated buffers (`self.scratch`).
        let mut invalid_feedback = false;
        let mut new_motor_errors = None;
        let mut driver_torque_lost = false;
        let mut recovery_ages = false;
        let mut hw_rx = false;
        let mut reset_stillness_on_read_err = false;
        let mut sc = std::mem::take(&mut self.scratch);
        sc.recovered.clear();
        if let Some(ref d) = self.driver {
            self.feedback_numeric_valid = false;
            match d.lock() {
                Ok(mut driver) => {
                    match driver.read_joint_states_into(&mut sc.states) {
                        Ok(()) => {
                            driver.take_recovery_writes_into(&mut sc.recovered);
                            let has_loads = driver.joint_loads_into(&mut sc.loads);
                            self.feedback_numeric_valid = Self::states_valid(&sc.states, driver.capabilities(), self.config.dof, has_loads.then_some(sc.loads.as_slice()));
                            invalid_feedback = !self.feedback_numeric_valid;
                            driver.feedback_ages_into(&mut sc.ages);
                            if !sc.recovered.is_empty() || self.tracking.recovery_suspended {
                                recovery_ages = true;
                            }
                            let driver_valid = driver.is_feedback_valid();
                            let all_fresh_age = sc.ages.len() == self.config.dof
                                && sc.ages.iter().all(|a| a.is_some_and(|age| age <= Duration::from_millis(250)));
                            if self.feedback_numeric_valid && sc.ages.len() == self.config.dof && driver_valid {
                                // the oldest joint's sample; None when a joint has none
                                let oldest = sc.ages.iter().try_fold(Duration::ZERO, |m, a| a.map(|a| m.max(a)));
                                self.last_can_feedback_instant = oldest.filter(|_| !sc.ages.is_empty())
                                    .and_then(|age| Instant::now().checked_sub(age));
                            } else {
                                self.last_can_feedback_instant = None;
                            }
                            if self.feedback_numeric_valid {
                                self.core.poll_feedback(&mut self.joint_feedback_queue_bounds, self.config.dof, driver.as_ref(), &mut sc.fb);
                                Self::apply_feedback(
                                    &mut self.measured_positions,
                                    &mut self.measured_velocities,
                                    &mut self.torques,
                                    &mut self.temperatures,
                                    &mut self.loads,
                                    self.config.dof,
                                    &sc.states,
                                    has_loads.then_some(sc.loads.as_slice()),
                                );
                            }
                            if self.feedback_numeric_valid && all_fresh_age && driver_valid {
                                driver.feedback_receive_instants_into(&mut sc.rx);
                                if sc.rx.len() == self.config.dof && sc.rx.iter().all(Option::is_some) {
                                    sc.rx_vals.clear();
                                    sc.rx_vals.extend(sc.rx.iter().map(|t| t.unwrap()));
                                    hw_rx = true;
                                }
                            }
                        }
                        Err(_) => {
                            self.last_can_feedback_instant = None;
                            reset_stillness_on_read_err = true;
                        }
                    }
                    if driver.motor_status_into(&mut sc.status) {
                        // decoded (a String per code) only when a code changed
                        let changed = sc.status.len() != self.motor_errors.len()
                            || sc.status.iter().zip(&self.motor_errors).any(|(code, e)| {
                                let decoded = e.as_ref().map(|e| e.code);
                                decoded != Some(*code) && !(decoded.is_none() && driver.decode_motor_error(*code).is_none())
                            });
                        if changed {
                            new_motor_errors = Some(sc.status.iter().map(|&code| {
                                driver.decode_motor_error(code).map(|name| crate::model::MotorError { code, name })
                            }).collect());
                        }
                    }
                    if self.is_torqued && !self.disable_pending && self.gravity_comp.is_none() && self.safety.state().is_operational()
                        && driver.capabilities().enable_feedback && driver.is_feedback_valid() && driver.enable_state() != EnableState::Enabled
                        && !(self.pending_rearm && driver.has_known_partial_enable()) {
                        driver_torque_lost = true;
                    }
                }
                Err(_) => {
                    self.last_can_feedback_instant = None;
                    reset_stillness_on_read_err = true;
                }
            }
        } else {
            // Driverless simulation mode: each control tick produces a fresh joint sample.
            self.core.simulate_feedback(self.config.dof, Instant::now());
            self.joint_feedback_queue_bounds.clear();
        }
        if let Some(errors) = new_motor_errors {
            self.update_motor_errors(errors);
        }
        if reset_stillness_on_read_err {
            self.reset_table_stillness();
        }
        self.tick_phase(0);
        // B1: autonomous driver recovery writes (e.g. Damiao 45 ms POS_VEL hold) invalidate
        // tracking output history and suspend compensated writes until fresh post-recovery
        // feedback confirms a clean baseline across every joint.
        if self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled) {
            if !sc.recovered.is_empty() {
                tracing::warn!(
                    "Driver emitted autonomous recovery writes for joints {:?}; suspending tracking compensation until fresh baseline",
                    sc.recovered.iter().map(|i| i + 1).collect::<Vec<_>>()
                );
                self.tracking.suspend_for_recovery(&sc.recovered, Instant::now());
                self.stop_motion();
                self.servo_stream_active = false;
            } else if self.tracking.recovery_suspended && self.feedback_numeric_valid {
                const RECOVERY_FRESH_BOUND: Duration = Duration::from_millis(45);
                let all_fresh_post_recovery = recovery_ages && Some(&sc.ages).is_some_and(|ages| {
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
        let dt_raw = dt;
        let dt = if dt.is_finite() && dt > 0.0 { dt.min(0.080) } else { 0.020 };
        self.sim_time_s += dt;
        if !dt_raw.is_finite() || dt_raw <= 0.0 || dt_raw > 0.25 {
            self.reset_table_stillness();
        }
        let is_still_sample = self.feedback_numeric_valid
            && (self.driver.is_none() || (self.hardware_feedback_fresh(Duration::from_millis(250)) && hw_rx))
            && self.active_trajectory.is_none()
            && self.servo_target.is_none()
            && self.commanded_velocities.iter().all(|v| v.abs() <= 1e-3)
            && self.measured_velocities.len() == self.config.dof
            && self.measured_velocities.iter().all(|v| v.is_finite() && v.abs() <= 0.02);
        if !is_still_sample {
            self.last_still_since_s = None;
            self.still_anchor_positions.clear();
            self.still_window_ever_moved = true;
            self.reset_table_stillness();
        } else {
            if self.still_anchor_positions.len() != self.measured_positions.len()
                || self.measured_positions.iter().zip(&self.still_anchor_positions)
                    .any(|(m, a)| !m.is_finite() || (m - a).abs() > UNLOAD_STILL_POSITION_SPAN_RAD)
            {
                if !self.still_anchor_positions.is_empty() {
                    self.still_window_ever_moved = true;
                }
                copy_into(&mut self.still_anchor_positions, &self.measured_positions);
                self.last_still_since_s = Some(self.sim_time_s);
            }
            if hw_rx {
                let rx_vals = &sc.rx_vals;
                let drifted = self.table_still_anchor_positions.len() != self.measured_positions.len()
                    || self.table_still_first_rx_instants.len() != self.config.dof
                    || self.table_still_last_rx_instants.len() != self.config.dof
                    || self.measured_positions.iter().zip(&self.table_still_anchor_positions)
                        .any(|(m, a)| !m.is_finite() || (m - a).abs() > UNLOAD_STILL_POSITION_SPAN_RAD);
                if drifted {
                    copy_into(&mut self.table_still_anchor_positions, &self.measured_positions);
                    copy_into(&mut self.table_still_first_rx_instants, rx_vals);
                    copy_into(&mut self.table_still_last_rx_instants, rx_vals);
                } else {
                    for (last, rx) in self.table_still_last_rx_instants.iter_mut().zip(rx_vals.iter().copied()) {
                        if rx > *last {
                            *last = rx;
                        }
                    }
                }
            }
        }
        copy_into(&mut sc.before_velocities, &self.commanded_velocities);
        // the scratch goes back for the helpers below; `before_velocities` stays in it
        self.scratch = sc;

        // 0.1 Gripper stall protection state machine
        if self.active_trajectory.is_some() || self.macro_run.is_some() || self.pending_normal_unload
            || self.gravity_comp.is_some() || self.last_motion_frame.elapsed() > self.motion_frame_timeout {
            self.gripper_stream_target = None;
        }
        if let Some(g_idx) = self.config.gripper_index() {
            let (is_bus_fresh, has_torque_feedback, has_load_feedback) = if let Some(ref d) = self.driver {
                if let Ok(driver) = d.lock() {
                    let fresh = self.feedback_numeric_valid
                        && self.last_can_feedback_instant.is_some_and(|t| t.elapsed() < Duration::from_millis(50))
                        && driver.is_feedback_valid();
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

            if force_cap.is_some() && closing && self.is_torqued && (!is_bus_fresh || !has_torque_feedback) {
                if !(self.gripper_stalled && self.gripper_force_capped) {
                    tracing::warn!(
                        "Gripper close stopped: feedback_fresh={is_bus_fresh} torque_feedback={has_torque_feedback} while max_close_torque_nm is active; holding"
                    );
                }
                self.gripper_stalled = true;
                self.gripper_force_capped = true;
                self.gripper_force_ticks = 0;
                self.gripper_stall_sign = close_dir;
                self.gripper_high_torque_ticks = 0;
                self.gripper_low_torque_ticks = 0;
                self.gripper_stream_target = None;
                let hold_q = if p_meas.is_finite() {
                    if let Some(lim) = self.config.limits.get(g_idx) {
                        p_meas.clamp(lim.min_position, lim.max_position)
                    } else {
                        p_meas
                    }
                } else {
                    p_cmd
                };
                if g_idx < self.commanded_positions.len() { self.commanded_positions[g_idx] = hold_q; }
                if g_idx < self.commanded_velocities.len() { self.commanded_velocities[g_idx] = 0.0; }
                if g_idx < self.target_positions.len() { self.target_positions[g_idx] = hold_q; }
                if let Some(ref mut st) = self.servo_target {
                    if st.len() > g_idx { st[g_idx] = hold_q; }
                }
                self.gripper_motion_hold = Some((hold_q, close_dir));
                self.note_motion_limit(MotionLimitReason::GripperForce, Some(g_idx));
            } else if !is_bus_fresh {
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
                        // release_servo_target inline (the driver guard borrows self)
                        if let Some(v) = self.servo_target.take() {
                            if v.capacity() >= self.servo_target_spare.capacity() { self.servo_target_spare = v; }
                        }
                        self.servo_settled_ticks.fill(0);
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
        // Quintic: the sample is in `point_buf` (the scratch's, no allocation)
        enum Stepped { Quintic(bool, f32), Action(StepOut), Arm(StepOut), Replay(StepOut, &'static str, f64) }
        let mut point_buf = std::mem::take(&mut self.scratch.point);
        // the scratch's step output: Path / Gripper / Replay steps write into its vectors
        let mut step_buf = std::mem::take(&mut self.scratch.step_out);
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
        // A planned motion's caps outlive its reference: kept from the tick it finishes
        // until it has settled (cleared after the output below).
        let tracking_enabled = self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled);
        let caps_before_step = if tracking_enabled {
            self.active_trajectory.as_ref().and_then(|a| a.motion_limits.clone())
        } else {
            None
        };
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
                    traj.step_into(dt * time_scale, &mut point_buf);
                    if time_scale < 1.0 { for v in point_buf.velocities.iter_mut() { *v *= time_scale as f32; } }
                    Stepped::Quintic(traj.is_finished(), (traj.elapsed / traj.duration).min(1.0) as f32)
                }
                Reference::Path(path) => { path.step_into(&ctx, &mut step_buf); Stepped::Arm(std::mem::take(&mut step_buf)) }
                Reference::Gripper(grip) => { grip.step_into(&ctx, &mut step_buf); Stepped::Action(std::mem::take(&mut step_buf)) }
                Reference::Replay(replay) => {
                    replay.step_into(&ctx, &mut step_buf);
                    Stepped::Replay(std::mem::take(&mut step_buf), replay.phase_name(), replay.t_s())
                }
            };
            // same-side approach: progress over both legs; the first leg's end starts the short
            // segment (src/backlash.rs)
            let out = match (active.span.is_some(), out) {
                (true, Stepped::Quintic(f, g)) => Stepped::Quintic(f, active.overall_progress(g)),
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
            Some((action_id, Stepped::Quintic(finished, progress))) => {
                reference_stepped = true;
                self.servo_stream_active = false;
                self.gripper_hold_filter(&mut point_buf.positions, &mut point_buf.velocities);
                // swap: the previous command's memory goes back to the scratch
                std::mem::swap(&mut self.commanded_positions, &mut point_buf.positions);
                std::mem::swap(&mut self.commanded_velocities, &mut point_buf.velocities);
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

        self.scratch.point = point_buf;
        // not stepped (no Path / Gripper / Replay reference): the buffer goes back unused
        if self.scratch.step_out.positions.capacity() < step_buf.positions.capacity() { self.scratch.step_out = step_buf; }
        if let Some(caps) = caps_before_step {
            if self.active_trajectory.is_none() {
                self.tracking.retained_motion_limits = Some(caps);
            }
        }

        // 3.1 Handle pending normal unload settling phase
        if self.pending_normal_unload && self.active_trajectory.is_none() {
            if !self.step_park_release() { self.step_unload_settle(servo_dt); }
        } else if self.servo_target.is_some() {
            // the target, copied into the scratch (the stream may replace it meanwhile)
            let mut target_buf = std::mem::take(&mut self.scratch.servo_target);
            copy_into(&mut target_buf, self.servo_target.as_deref().unwrap_or(&[]));
            let target = &target_buf;
            // Check continuous motion frame timeout:
            // If > 200ms without a new pose frame, clear servo_target to begin controlled smooth deceleration
            if self.last_motion_frame.elapsed() > self.motion_frame_timeout {
                self.release_servo_target();
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
                    self.release_servo_target();
                    self.servo_settled_ticks.fill(0);
                    self.fail_live_policy_from_control("SERVO_TARGET_DISCARDED",
                        "The control loop discarded the policy target: it leads the commanded position by more than 0.35 rad".into());
                } else {
                    reference_stepped = true;
                    let mut reached_flags = std::mem::take(&mut self.scratch.reached);
                    fill_n(&mut reached_flags, false, self.config.dof);
                    let mut next_positions = std::mem::take(&mut self.scratch.next_positions);
                    let mut next_velocities = std::mem::take(&mut self.scratch.next_velocities);
                    copy_into(&mut next_positions, &self.commanded_positions);
                    copy_into(&mut next_velocities, &self.commanded_velocities);
                    let mut infeasible = None;
                    // Speed cap: joint limits capped by servo_max_velocity. Lead cap: in the direction
                    // of travel the stepper target stays within servo_max_lead_rad of the measured
                    // position (torque on, fresh feedback), so the tracking error stays below the bound
                    // whatever the client sends.
                    let mut step_limits = std::mem::take(&mut self.scratch.step_limits);
                    self.servo_limits_into(&mut step_limits);
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
                    let mut lead_capped = std::mem::take(&mut self.scratch.lead_capped);
                    fill_n(&mut lead_capped, false, self.config.dof);
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
                        // swap: the previous command's memory goes back to the scratch
                        std::mem::swap(&mut self.commanded_positions, &mut next_positions);
                        std::mem::swap(&mut self.commanded_velocities, &mut next_velocities);
                        copy_into(&mut self.target_positions, &self.commanded_positions);

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
                            self.release_servo_target();
                            self.safety.mark_motion_finished();
                            if self.commanded_velocities.iter().all(|&v| v == 0.0) {
                                self.servo_ws_min_depths = None;
                                let keep_stream = self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled)
                                    && self.tracking.has_residual_lead(
                                        &self.commanded_positions,
                                        &self.config.arm_joints(),
                                    );
                                if !keep_stream {
                                    self.servo_stream_active = false;
                                }
                            }
                        }
                    }
                    self.scratch.reached = reached_flags;
                    self.scratch.next_positions = next_positions;
                    self.scratch.next_velocities = next_velocities;
                    self.scratch.step_limits = step_limits;
                    self.scratch.lead_capped = lead_capped;
                }
            }
            self.scratch.servo_target = target_buf;
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
            let mut next_positions = std::mem::take(&mut self.scratch.next_positions);
            let mut next_velocities = std::mem::take(&mut self.scratch.next_velocities);
            copy_into(&mut next_positions, &self.commanded_positions);
            copy_into(&mut next_velocities, &self.commanded_velocities);
            let mut infeasible = None;
            // braking a stream: same limits as streaming (`config.limits`)
            for i in 0..self.config.dof {
                let step = Self::bounded_servo_step_with_jerk(
                    self.commanded_positions[i],
                    self.commanded_positions[i],
                    self.commanded_velocities[i],
                    servo_dt,
                    &self.config.limits[i],
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
                // swap: the previous command's memory goes back to the scratch
                std::mem::swap(&mut self.commanded_positions, &mut next_positions);
                std::mem::swap(&mut self.commanded_velocities, &mut next_velocities);
                copy_into(&mut self.target_positions, &self.commanded_positions);
                if self.commanded_velocities.iter().all(|&v| v == 0.0) {
                    self.safety.mark_motion_finished();
                    self.servo_ws_min_depths = None;
                    let keep_stream = self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled)
                        && (braking
                            || self.tracking.has_residual_lead(
                                &self.commanded_positions,
                                &self.config.arm_joints(),
                            ));
                    if !keep_stream {
                        self.servo_stream_active = false;
                    }
                }
            }
            self.scratch.next_positions = next_positions;
            self.scratch.next_velocities = next_velocities;
        }

        self.gripper_lead_cap();
        for i in 0..self.config.dof {
            self.commanded_accelerations[i] = if self.safety.state().is_stopped() { 0.0 }
                else { (self.commanded_velocities[i] - self.scratch.before_velocities[i]) / servo_dt as f32 };
        }
        copy_into(&mut self.last_commanded_velocities, &self.commanded_velocities);

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
        // MIT tracking (Phase 2): deadline supervision, park hand back at rest, handover at rest.
        // No-ops while MIT is not requested.
        self.mit_supervise();
        if self.mit.revoke_pending {
            // a refused rearm revoked MIT: hand back once the brake reached rest
            let at_rest = self.commanded_velocities.iter().all(|v| *v == 0.0);
            if !self.mit_active() || self.safety.state().is_stopped()
                || (at_rest && self.mit_hand_back("authorization revoked").is_ok()) {
                self.mit.revoke_pending = false;
            }
        }
        if self.mit.park_after_hand_back {
            // a park waits for rest and the confirmed hand back, then starts
            let at_rest = self.commanded_velocities.iter().all(|v| *v == 0.0);
            if !self.mit_active() || self.safety.state().is_stopped() {
                self.mit.park_after_hand_back = false;
                if let Some(id) = self.mit.deferred_park_action.take() {
                    self.finish_action(id, "failed", Some("UNLOAD_FAILED"), Some("Park cancelled: MIT tracking lost or a fault".into()), None, None, None);
                }
            } else if at_rest && self.mit_hand_back("park").is_ok() {
                self.mit.park_after_hand_back = false;
                self.apply_stop_policy(StopPolicy::NormalUnload);
                if let Some(id) = self.mit.deferred_park_action.take() {
                    if self.pending_normal_unload {
                        if let Some(active) = self.active_trajectory.as_mut() { active.action_id = Some(id); }
                    } else {
                        self.finish_action(id, "failed", Some("UNLOAD_FAILED"), Some("Park could not start after the MIT hand back".into()), None, None, None);
                    }
                }
            }
        }
        if self.pending_normal_unload && self.mit_active() {
            let _ = self.mit_hand_back("park");
        }
        self.mit_maybe_activate();
        let gravity_output_enabled = self.gravity_output_enabled();
        self.tick_phase(1);
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
                let arm_indices = self.config.arm_joints();
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
                // Unknown history: plain nominal output until the arm is confirmed at rest.
                let tracking_owned = tracking_active
                    && self.driver.is_some()
                    && !self.safety.state().is_stopped()
                    && !self.pending_normal_unload
                    && (self.tracking.history_state != crate::tracking::TrackingHistoryState::Unknown
                        || self.tracking_stationary_evidence());

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
                    // The rejected motion's caps (`FallbackRamp::limits`) stay in force, tightened
                    // by the current power governor limit; the ramp keeps its original caps.
                    let ramp = self.tracking.fallback_ramp.clone().expect("checked above");
                    let governed = self.active_tracking_limits(servo_dt, time_scale);
                    let mut tick_ramp = ramp.clone();
                    for (l, g) in tick_ramp.limits.iter_mut().zip(&governed) {
                        l.max_velocity = l.max_velocity.min(g.max_velocity);
                        l.max_acceleration = l.max_acceleration.min(g.max_acceleration);
                    }
                    let mut tick_sweep_budget = MAX_KEEPOUT_EVALS_PER_TICK;
                    let mut pending = self.tracking_fallback_compute(tick_ramp, servo_dt, &mut tick_sweep_budget);
                    pending.ramp = ramp;
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
                            let write_res = self.write_output(targets, servo_dt);
                            match write_res {
                                Some(Ok(())) => {
                                    let nominal = self.tracking.nominal_entry(&self.commanded_positions, servo_dt);
                                    self.tracking.record_issued_with_ws_depths(
                                        out.positions,
                                        out.velocities,
                                        out.ws_min_depths,
                                        out.ws_keepout_min_depths,
                                    );
                                    self.tracking.last_nominal = Some(nominal);
                                    if out.withdrawn {
                                        self.tracking.withdrawal_ticks += 1;
                                    }
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
                                limits: active_limits.clone(),
                            };
                            let pending = self.tracking_fallback_compute(ramp, servo_dt, &mut tick_sweep_budget);
                            match reject {
                                crate::tracking::TrackingReject::Workspace(ws_err) if was_streaming => {
                                    self.enter_servo_keepout_hold(&ws_err);
                                }
                                crate::tracking::TrackingReject::Workspace(ws_err) => {
                                    tracing::warn!("Joint tracking output violated workspace ({ws_err}): terminating motion");
                                    self.hold_measured_pose();
                                    self.drop_macro_run("failed", Some("WORKSPACE_VIOLATION"));
                                    self.safety.trigger_fault_coded(
                                        FaultCode::Other,
                                        FaultReason::Other(format!("Joint tracking output rejected: {ws_err}")),
                                    );
                                }
                                crate::tracking::TrackingReject::JointLimit(msg) => {
                                    tracing::warn!("Joint tracking output exceeded limits ({msg}): terminating motion");
                                    self.hold_measured_pose();
                                    self.drop_macro_run("failed", Some("JOINT_LIMIT"));
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
                    if self.driver.is_some() {
                        // built in the tick's scratch (no allocation)
                        let mut targets = std::mem::take(&mut self.scratch.targets);
                        targets.clear();
                        targets.extend(self
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
                            }));
                        self.scratch.targets = targets;
                        match self.write_output_scratch(servo_dt) {
                            Some(Ok(())) => tick_output_accepted = true,
                            Some(Err(e)) => output_error = Some(e),
                            None => {}
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

        if self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled) {
            self.note_tracking_position_sample();
            let settled = self.active_trajectory.is_none()
                && self.quintic_settle.is_none()
                && self.tracking.fallback_ramp.is_none()
                && !self.tracking.has_residual_lead(&self.commanded_positions, &self.config.arm_joints());
            if settled {
                self.tracking.retained_motion_limits = None;
            }
        }

        self.tick_phase(2);
        // An E-stop requested during this tick: its output did not reach the bus (or the batch
        // stopped at the next frame, `ArmDriver::set_estop_gate`); apply it now instead of one
        // period later. A batch the gate stopped is not a driver failure.
        if self.estop_request_unapplied() {
            tick_output_accepted = false;
            self.consume_pending_estop();
            if output_error.as_deref().is_some_and(|e| e.contains(crate::driver::ESTOP_INHIBITED)) { output_error = None; }
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
                        &self.config.arm_joints(),
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
            copy_into(&mut self.measured_positions, &self.commanded_positions);
            copy_into(&mut self.measured_velocities, &self.commanded_velocities);
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

        // Tracking off: settle completion does not wait for an accepted output (e4f72b6).
        self.reconcile_action(servo_dt, tick_output_accepted || !tracking_enabled);
        self.expire_motion_limit();
        self.tick_phase(3);
        let timestamp = self.start_time.elapsed().as_secs_f64();

        // 5. Record frame if episode recording is active: a teach session appends to the
        // preallocated ring (no allocation, `crate::rt_core::record_ring`); a recorder started
        // directly (no session) keeps its frames itself
        if self.recording_frame_room() {
            if self.core.recording_active() {
                let seq = self.core.published_seq() + 1;
                self.core.record_sample(seq, timestamp, &self.measured_positions, &self.commanded_positions);
            } else {
                self.recorder.append_frame(RecordFrame {
                    timestamp,
                    observation_state: self.measured_positions.clone(),
                    action: self.commanded_positions.clone(),
                });
            }
        }

        timestamp
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
    keepout_sweep_with_tcp_tol(model, ws, cam, [0.0; 3], crate::kinematics::WORKSPACE_EPS, at, travel, knots, budget)
}

#[allow(clippy::too_many_arguments)]
/// The commanded joints `start` with the arm joints replaced by `q` (gripper and extra joints kept).
pub(crate) fn joint_goal_target_from(start: &[f32], q: &[f64; 6], arm_dof: usize, gripper_idx: Option<usize>) -> Vec<f32> {
    let mut target = start.to_vec();
    for (i, t) in target.iter_mut().enumerate().take(arm_dof.min(6)) {
        if Some(i) != gripper_idx { *t = q[i] as f32; }
    }
    target
}

/// Supervisor state a `path: "joint"` quintic is planned from ([`SupervisorInner::joint_goal_snapshot`]).
#[derive(Debug, Clone)]
pub struct JointGoalSnapshot {
    pub start: Vec<f32>,
    pub v0: Vec<f32>,
    pub limits: Vec<crate::model::JointLimit>,
    pub arm_dof: usize,
    pub gripper_index: Option<usize>,
}

/// A joint quintic planned, power-fitted and workspace-checked without the supervisor lock
/// ([`prepare_quintic`]), with the inputs it was made from.
#[derive(Debug, Clone)]
pub struct PreparedQuintic {
    start: Vec<f32>,
    v0: Vec<f32>,
    target: Vec<f32>,
    limits: Vec<crate::model::JointLimit>,
    workspace_revision: u64,
    tcp_offset: [f64; 3],
    power: Option<crate::power::PowerBudget>,
    checked_workspace: bool,
    traj: MultiJointQuinticTrajectory,
    power_scale: Option<f64>,
}

/// The work of [`SupervisorInner::start_quintic`] that needs no lock, for the joints `q`:
/// quintic, power fit and swept-workspace check, from `input` and `snap`. None when any of
/// them fails: the install then repeats them under the lock and returns their error.
pub fn prepare_quintic(input: &PlanInput, snap: &JointGoalSnapshot, q: &[f64; 6]) -> Option<PreparedQuintic> {
    let target = joint_goal_target_from(&snap.start, q, snap.arm_dof, snap.gripper_index);
    let zeros = vec![0.0; target.len()];
    let traj = TrajectoryPlanner::plan(&snap.start, &snap.v0, &target, &zeros, None, &snap.limits).ok()?;
    let (traj, power_scale) = crate::power::power_fit_quintic_with(
        input.power.as_ref(), &input.arm, input.arm.arm_dof(), traj, &snap.start, &snap.v0, &target, &snap.limits, true).ok()?;
    if let Some(ws) = input.ws.as_ref() {
        let knots: Vec<f64> = (0..=20).map(|k| traj.duration * k as f64 / 20.0).collect();
        let at = |t: f64| traj.sample(t).positions;
        let travel = |t0: f64, t1: f64| traj.travel_bound(t0, t1);
        let eps = crate::kinematics::WORKSPACE_EPS;
        let samples = keepout_sweep_with_tcp_tol(&input.arm, ws, &input.cam, input.tcp_offset, eps, &at, &travel, &knots, MAX_KEEPOUT_EVALS_PER_PLAN).ok()?;
        check_path_with_tcp_tol(&input.arm, ws, &input.cam, input.tcp_offset, eps, &snap.start, samples.iter().map(Vec::as_slice)).ok()?;
    }
    Some(PreparedQuintic {
        start: snap.start.clone(),
        v0: snap.v0.clone(),
        target,
        limits: snap.limits.clone(),
        workspace_revision: input.workspace_revision,
        tcp_offset: input.tcp_offset,
        power: input.power.clone(),
        checked_workspace: input.ws.is_some(),
        traj,
        power_scale,
    })
}

pub fn keepout_sweep_with_tcp_tol(
    model: &crate::chain::ArmModel,
    ws: &crate::model::WorkspacePolicy,
    cam: &crate::model::CameraMount,
    tcp_offset: [f64; 3],
    escape_tol: f64,
    at: &dyn Fn(f64) -> Vec<f32>,
    travel: &dyn Fn(f64, f64) -> Vec<f64>,
    knots: &[f64],
    budget: usize,
) -> Result<Vec<Vec<f32>>, SafetyError> {
    crate::chain::WorkspaceEvaluator::new_with_tcp(model, ws, cam, tcp_offset)
        .with_escape_tol(escape_tol)
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
    check_path_with_tcp_tol(model, ws, cam, [0.0; 3], crate::kinematics::WORKSPACE_EPS, start, samples)
}

pub fn check_path_with_tcp_tol<'a>(
    model: &crate::chain::ArmModel,
    ws: &crate::model::WorkspacePolicy,
    cam: &crate::model::CameraMount,
    tcp_offset: [f64; 3],
    escape_tol: f64,
    start: &[f32],
    samples: impl IntoIterator<Item = &'a [f32]>,
) -> Result<(), SafetyError> {
    let eval = crate::chain::WorkspaceEvaluator::new_with_tcp(model, ws, cam, tcp_offset)
        .with_escape_tol(escape_tol);
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
    inner: Arc<SupervisorLock<SupervisorInner>>,
    telemetry_tx: broadcast::Sender<ArmObservation>,
    pub e_stop_latched: Arc<AtomicBool>,
    pub requested_estop_count: Arc<AtomicU64>,
    tick_stats: std::sync::Mutex<crate::diagnostics::tick::TickStats>,
    /// Wait for `tick_stats` of the previous tick (µs), recorded with the next tick.
    tick_stats_wait_us: AtomicU64,
    control_sched: std::sync::Mutex<Option<crate::diagnostics::tick::ControlThreadSched>>,
    /// Readers' side of the tick's state publication (`crate::rt_core::publish`).
    hub: Arc<crate::rt_core::publish::StateHub>,
    /// Serializes the start of the dispatcher thread.
    dispatcher_start: std::sync::Mutex<()>,
    /// Control period of the last tick (µs): bounds how long a reader waits for a sample.
    control_period_us: AtomicU64,
    obs_cache_hits: AtomicU64,
    obs_cache_misses: AtomicU64,
    obs_waits: AtomicU64,
    obs_unavailable: AtomicU64,
    config_write_mutex: Arc<tokio::sync::Mutex<()>>,
}

/// How often a reader that waits for the next sample looks again.
const OBS_POLL: Duration = Duration::from_millis(1);

/// How long [`ArmSupervisor::observation`] waits for the read lock when no sample answers.
pub const OBS_LOCK_WAIT: Duration = Duration::from_millis(500);

/// 503 `OBSERVATION_UNAVAILABLE`: neither a fresh sample nor the read lock in time.
fn observation_unavailable() -> ArmError {
    ArmError::Rejected { status: 503, code: "OBSERVATION_UNAVAILABLE".into(),
        message: "No current observation: the control tick has not published and the supervisor lock stayed busy".into() }
}

/// A published sample, or the read guard when no sample answers (no control loop running).
enum Fresh<'a> {
    Sample(Arc<crate::rt_core::publish::Snapshot>),
    Locked(crate::supervisor_lock::SupReadGuard<'a, SupervisorInner>),
}

impl Drop for ArmSupervisor {
    fn drop(&mut self) { self.hub.close(); }
}

impl ArmSupervisor {
    pub fn new(config: ArmConfig) -> Self {
        let (tx, _) = broadcast::channel(128);
        let e_stop_latched = Arc::new(AtomicBool::new(false));
        let requested_estop_count = Arc::new(AtomicU64::new(0));
        let mut inner = SupervisorInner::new(config);
        inner.e_stop_latched = e_stop_latched.clone();
        inner.requested_estop_count = requested_estop_count.clone();
        let hub = inner.core.hub().clone();
        let control_period_us = (1e6 / f64::from(inner.config.control_frequency_hz.max(1))) as u64;
        Self {
            inner: Arc::new(SupervisorLock::new(inner)),
            telemetry_tx: tx,
            e_stop_latched,
            requested_estop_count,
            tick_stats: std::sync::Mutex::new(crate::diagnostics::tick::TickStats::default()),
            tick_stats_wait_us: AtomicU64::new(0),
            control_sched: std::sync::Mutex::new(None),
            hub,
            dispatcher_start: std::sync::Mutex::new(()),
            control_period_us: AtomicU64::new(control_period_us),
            obs_cache_hits: AtomicU64::new(0),
            obs_cache_misses: AtomicU64::new(0),
            obs_waits: AtomicU64::new(0),
            obs_unavailable: AtomicU64::new(0),
            config_write_mutex: Arc::new(tokio::sync::Mutex::new(())),
        }
    }

    pub fn config_write_lock(&self) -> Arc<tokio::sync::Mutex<()>> {
        Arc::clone(&self.config_write_mutex)
    }

    pub async fn set_operator_auth_enforced(&self, enforced: bool) {
        self.inner.write().await.operator_auth_enforced = enforced;
    }

    /// Every tick's observation, forwarded by the dispatcher thread (started here when it
    /// does not run yet; an error when it cannot start); a lagging receiver skips
    /// observations, a lagging dispatcher forwards only the newest.
    pub fn subscribe_telemetry(&self) -> Result<broadcast::Receiver<ArmObservation>, ArmError> {
        self.ensure_dispatcher()?;
        Ok(self.telemetry_tx.subscribe())
    }

    /// Starts the dispatcher thread (armd does at startup, before the control loop): it
    /// frees the observations the tick retired, forwards telemetry and drains the
    /// recording ring. Idempotent.
    pub fn start_dispatcher(&self) -> Result<(), ArmError> { self.ensure_dispatcher() }

    /// Starts the dispatcher thread once: a normal-priority thread woken by every
    /// publication that forwards samples to [`Self::subscribe_telemetry`] receivers and
    /// drains the recording ring.
    pub(crate) fn ensure_dispatcher(&self) -> Result<(), ArmError> {
        if self.hub.has_dispatcher() { return Ok(()); }
        let _start = self.dispatcher_start.lock().unwrap_or_else(|e| e.into_inner());
        if self.hub.has_dispatcher() { return Ok(()); }
        let (hub, tx) = (Arc::clone(&self.hub), self.telemetry_tx.clone());
        let handle = std::thread::Builder::new().name("armd-dispatch".into()).spawn(move || dispatch_loop(&hub, &tx))
            .map_err(|e| ArmError::Internal(format!("cannot start the observation dispatcher thread: {e}")))?;
        self.hub.set_dispatcher(handle.thread().clone());
        Ok(())
    }

    #[doc(hidden)]
    pub fn inner(&self) -> &Arc<SupervisorLock<SupervisorInner>> {
        &self.inner
    }

    /// The current observation, without the supervisor lock while the control loop runs
    /// (`GET /api/v1/observation`).
    ///
    /// Served from the tick's publication (`crate::rt_core::publish`): a sample answers when
    /// it was taken after every write that completed before this call (its lock generation is
    /// at least the one seen here) and its validity deadline has not passed (lease expiry,
    /// previous-action window, feedback freshness, two control periods). Otherwise, while a
    /// sample younger than two control periods shows the control loop running, the call waits
    /// for the next tick's sample (at most two periods). Without a running loop the
    /// observation is computed under the read lock, waiting for it at most
    /// [`OBS_LOCK_WAIT`]; when neither gives fresh state in time the call fails with 503
    /// `OBSERVATION_UNAVAILABLE` instead of waiting further. The hub is only polled
    /// (never waited for), and not at all while the read guard is held.
    pub async fn observation(&self) -> Result<ArmObservation, ArmError> {
        match self.fresh(true).await? {
            Fresh::Sample(s) => Ok(s.obs.clone()),
            Fresh::Locked(inner) => Ok(inner.current_observation()),
        }
    }

    /// [`Self::observation`] for in-process callers and tests: when it is unavailable, waits
    /// for the read lock without a bound (the 1.5 behaviour).
    pub async fn get_observation(&self) -> ArmObservation {
        match self.observation().await {
            Ok(obs) => obs,
            Err(_) => self.inner.read().await.current_observation(),
        }
    }

    fn wait_bound(&self) -> Duration { Duration::from_micros(2 * self.control_period_us.load(Ordering::Relaxed)) }

    fn answers(s: &crate::rt_core::publish::Snapshot, want: u64, now: Instant) -> bool {
        s.meta.generation >= want && now <= s.meta.valid_until
    }

    async fn fresh(&self, count: bool) -> Result<Fresh<'_>, ArmError> {
        use crate::rt_core::publish::Poll;
        let want = self.inner.generation();
        let started = Instant::now();
        let bound = self.wait_bound();
        let mut waited = false;
        loop {
            let poll = self.hub.try_latest();
            // read after the refresh: reclamation or preemption inside it must not let an
            // expired sample through
            let now = Instant::now();
            match poll {
                Poll::Ready(None) => break,
                Poll::Ready(Some(s)) => {
                    if Self::answers(&s, want, now) {
                        if count {
                            self.obs_cache_hits.fetch_add(1, Ordering::Relaxed);
                            if waited { self.obs_waits.fetch_add(1, Ordering::Relaxed); }
                        }
                        return Ok(Fresh::Sample(s));
                    }
                    if now.saturating_duration_since(s.meta.sampled_at) > bound { break; }
                }
                // another reader holds the hub: look again
                Poll::Busy => {}
            }
            if now.duration_since(started) >= bound { break; }
            waited = true;
            tokio::time::sleep(OBS_POLL).await;
        }
        let seen_seq = self.hub.published_seq();
        // one deadline for every wait for the read lock below (not one per attempt)
        let lock_deadline = tokio::time::Instant::now() + OBS_LOCK_WAIT;
        // a stalled tick (or writer) holds the lock: one last non-blocking look, then 503
        let unavailable = || {
            if let Poll::Ready(Some(s)) = self.hub.try_latest() {
                if Self::answers(&s, want, Instant::now()) {
                    if count { self.obs_cache_hits.fetch_add(1, Ordering::Relaxed); }
                    return Ok(Fresh::Sample(s));
                }
            }
            if count { self.obs_unavailable.fetch_add(1, Ordering::Relaxed); }
            Err(observation_unavailable())
        };
        let Ok(inner) = tokio::time::timeout_at(lock_deadline, self.inner.read()).await else { return unavailable(); };
        // a tick may have published while this call waited for the lock. Only the atomic
        // sequence is read under the guard: refreshing the hub (reclamation, the snapshot's
        // allocation) happens after the guard is dropped
        if self.hub.published_seq() != seen_seq {
            drop(inner);
            if let Poll::Ready(Some(s)) = self.hub.try_latest() {
                if Self::answers(&s, want, Instant::now()) {
                    if count { self.obs_cache_hits.fetch_add(1, Ordering::Relaxed); }
                    return Ok(Fresh::Sample(s));
                }
            }
            let Ok(inner) = tokio::time::timeout_at(lock_deadline, self.inner.read()).await else { return unavailable(); };
            if count { self.obs_cache_misses.fetch_add(1, Ordering::Relaxed); }
            return Ok(Fresh::Locked(inner));
        }
        if count { self.obs_cache_misses.fetch_add(1, Ordering::Relaxed); }
        Ok(Fresh::Locked(inner))
    }

    /// The newest sample while the control loop runs (one younger than two control
    /// periods), for feedback reads: the per-joint feedback fields are written by the tick
    /// (and by torque and fault handshakes, whose feedback the next sample carries).
    fn running_sample(&self) -> Option<Arc<crate::rt_core::publish::Snapshot>> {
        self.hub.latest().filter(|s| s.meta.sampled_at.elapsed() <= self.wait_bound())
    }

    /// [`SupervisorInner::arm_feedback_snapshot`] from the newest sample while the control
    /// loop runs, else under the read lock. `arm`: the profile's arm joint indices
    /// (`config.arm_joint_indices()`).
    pub async fn arm_feedback_snapshot(&self, arm: &[usize]) -> (Vec<f32>, Vec<f32>, Vec<Option<Instant>>, Vec<u64>) {
        match self.running_sample() {
            Some(s) => s.rt.arm_feedback(arm),
            None => self.inner.read().await.arm_feedback_snapshot(),
        }
    }

    /// The measured joint positions from the newest sample while the control loop runs,
    /// else under the read lock.
    pub async fn measured_positions(&self) -> Vec<f32> {
        match self.running_sample() {
            Some(s) => s.rt.positions.clone(),
            None => self.inner.read().await.measured_positions.clone(),
        }
    }

    /// Scheduling of the control thread, reported by it after its real-time request.
    pub fn set_control_thread_sched(&self, sched: crate::diagnostics::tick::ControlThreadSched) {
        *self.control_sched.lock().unwrap_or_else(|e| e.into_inner()) = Some(sched);
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
        driver.set_estop_gate(inner.estop_gate());
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
            driver.set_estop_gate(inner.estop_gate());
            driver.set_gripper_index(inner.config.gripper_index());
            driver.set_strict_mode_handover(inner.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled));
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
        self.tick_shared(dt, true).await.expect("tick keeps a copy when asked")
    }

    /// [`Self::tick`] for the control loop, which does not use the observation: saves a copy.
    pub async fn control_tick(&self, dt: f64) {
        self.tick_shared(dt, false).await;
    }

    /// One tick; publishes its observation (`crate::rt_core::publish`) and returns a copy
    /// when `keep`.
    async fn tick_shared(&self, dt: f64, keep: bool) -> Option<ArmObservation> {
        use crate::diagnostics::tick::{us, UsageMark};
        let started = Instant::now();
        let usage_start = UsageMark::now();
        let (obs, wait, lock_held, control_hz, pending_zc, phases, log, locked, usage_locked, pre, publish, allocs) = {
            let (mut inner, generation) = self.inner.write_tick().await;
            let locked = Instant::now();
            let usage_locked = UsageMark::now();
            let _ = crate::diagnostics::alloc::take();
            // No other guard exists now: the log is complete up to the guard waited for.
            let log = self.inner.take_log();
            inner.defer_zero_check_persist = true;
            inner.tick_phases_us = [0; 5];
            let body_start = Instant::now();
            inner.tick_phase_mark = Some(body_start);
            let TickPublished { kept, body_end } = inner.tick_publish(dt, generation, keep);
            let body_end = body_end.unwrap_or(body_start);
            let phases = inner.tick_phases_us;
            inner.defer_zero_check_persist = false;
            let pending_zc = inner.pending_zero_check_persist.take();
            let control_hz = inner.config.control_frequency_hz;
            let lock_end = Instant::now();
            let allocs = crate::diagnostics::alloc::take();
            (kept, locked - started, lock_end - locked, control_hz, pending_zc, phases, log, locked, usage_locked,
                body_start - locked, lock_end - body_end, allocs)
        };
        let released = Instant::now();
        let usage_released = UsageMark::now();
        self.control_period_us.store((1e6 / f64::from(control_hz.max(1))) as u64, Ordering::Relaxed);
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
        let late = !dt.is_finite() || dt > 2.0 / f64::from(control_hz.max(1));
        let done = Instant::now();
        let log = log.unwrap_or_default();
        let holder = |r: crate::supervisor_lock::HoldRecord| {
            let signed = |a: Instant, b: Instant| if a >= b { us(a - b) as i64 } else { -(us(b - a) as i64) };
            crate::diagnostics::tick::Holder {
                kind: r.kind.as_str(),
                location: r.location,
                hold_us: us(r.hold()),
                acquire_us: us(r.acquire()),
                acquired_before_wait_us: signed(started, r.acquired),
                released_before_lock_us: signed(locked, r.released),
            }
        };
        let sample = crate::diagnostics::tick::TickSample {
            tick_us: us(done - started),
            wait_us: us(wait),
            lock_us: us(lock_held),
            post_us: us(done - released),
            period_us: if dt.is_finite() && dt >= 0.0 { us(Duration::from_secs_f64(dt.min(4.0e3))) } else { 0 },
            reader_hold_us: us(log.reader_hold),
            writer_hold_us: us(log.writer_hold),
            reader_acquire_us: us(log.reader_acquire),
            writer_acquire_us: us(log.writer_acquire),
            stats_wait_us: self.tick_stats_wait_us.load(Ordering::Relaxed).min(u64::from(u32::MAX)) as u32,
            phases: crate::diagnostics::tick::LockPhases {
                pre_us: us(pre),
                rx_us: phases[0], control_us: phases[1], tx_us: phases[2], finish_us: phases[3], obs_us: phases[4],
                publish_us: us(publish),
            },
            usage: crate::diagnostics::tick::ThreadUsage {
                lock_alloc_calls: allocs.calls.min(u64::from(u32::MAX)) as u32,
                lock_alloc_us: (allocs.total_ns / 1000).min(u64::from(u32::MAX)) as u32,
                lock_alloc_max_us: (allocs.max_ns / 1000).min(u64::from(u32::MAX)) as u32,
                ..crate::diagnostics::tick::thread_usage(&usage_start, &usage_locked, &usage_released)
            },
            waited_for: log.last_release.map(holder),
            longest_hold: log.longest.map(holder),
            late,
        };
        let mut stats = self.tick_stats.lock().unwrap_or_else(|e| e.into_inner());
        self.tick_stats_wait_us.store(done.elapsed().as_micros().min(u128::from(u64::MAX)) as u64, Ordering::Relaxed);
        stats.record(&sample);
        drop(stats);
        obs
    }

    /// The raw samples of the last ticks, oldest first (tests, diagnostics).
    pub fn tick_samples(&self) -> Vec<crate::diagnostics::tick::TickSample> {
        self.tick_stats.lock().unwrap_or_else(|e| e.into_inner()).window().samples().to_vec()
    }

    /// Timing of the last ticks (`GET /api/v1/diagnostics/tick`). The ring is copied under
    /// the stats mutex and sorted after it is released, so this route does not delay a tick.
    pub fn tick_stats(&self) -> crate::diagnostics::tick::TickSnapshot {
        let window = self.tick_stats.lock().unwrap_or_else(|e| e.into_inner()).window();
        let mut snap = window.snapshot();
        snap.control_thread = self.control_sched.lock().unwrap_or_else(|e| e.into_inner()).clone();
        snap.observation_cache = crate::diagnostics::tick::ObservationCacheStats {
            hits: self.obs_cache_hits.load(Ordering::Relaxed),
            misses: self.obs_cache_misses.load(Ordering::Relaxed),
            publish_skipped: 0,
            waited: self.obs_waits.load(Ordering::Relaxed),
            unavailable: self.obs_unavailable.load(Ordering::Relaxed),
            published: self.hub.published_seq(),
            tick_frees: self.hub.tick_frees(),
        };
        snap
    }

    pub async fn set_mock_gripper_stall_offset(&self, offset: Option<f32>) {
        self.inner.write().await.mock_gripper_stall_offset = offset;
    }

    pub async fn gripper_calibration_response(&self) -> crate::gripper_calib::GripperCalibrationResponse {
        self.inner.read().await.gripper_calibration_response()
    }
}

/// The dispatcher thread: forwards each new sample to the telemetry receivers (one clone
/// per sample, off the control thread) until the supervisor is dropped.
fn dispatch_loop(hub: &crate::rt_core::publish::StateHub, tx: &broadcast::Sender<ArmObservation>) {
    #[cfg(target_os = "linux")]
    {
        // a normal-priority thread even when started from a real-time one
        // SAFETY: sched_param is plain old data (musl has extra fields); the call only
        // touches the calling thread.
        unsafe {
            let mut param: libc::sched_param = std::mem::zeroed();
            param.sched_priority = 0;
            libc::pthread_setschedparam(libc::pthread_self(), libc::SCHED_OTHER, &param);
        }
    }
    let mut last = 0;
    while !hub.is_closed() {
        if let Some(ring) = hub.record_ring() { ring.drain(); }
        if hub.published_seq() != last {
            if let Some(s) = hub.latest().filter(|s| s.meta.seq > last) {
                last = s.meta.seq;
                if tx.receiver_count() > 0 { let _ = tx.send(s.obs.clone()); }
            }
        }
        std::thread::park_timeout(Duration::from_millis(50));
    }
}

#[cfg(test)]
#[path = "supervisor_servo_order_tests.rs"]
mod servo_order_tests;

#[cfg(test)]
mod observation_snapshot_tests {
    use super::*;

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_reader_holding_the_hub_does_not_delay_the_tick() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.tick(0.02).await;
        let (held_tx, held_rx) = std::sync::mpsc::channel();
        let (release_tx, release_rx) = std::sync::mpsc::channel::<()>();
        let holder = {
            let sup = sup.clone();
            // a preempted reader: holds the readers' mutex until released
            std::thread::spawn(move || {
                let _g = sup.hub.reader_guard();
                held_tx.send(()).unwrap();
                release_rx.recv().unwrap();
            })
        };
        held_rx.recv().unwrap();
        let t = Instant::now();
        let mut last = 0.0;
        for _ in 0..3 {
            last = tokio::time::timeout(Duration::from_millis(500), sup.tick(0.02)).await.expect("tick must not wait for readers").timestamp;
        }
        assert!(t.elapsed() < Duration::from_millis(100), "{:?}", t.elapsed());
        release_tx.send(()).unwrap();
        holder.join().unwrap();
        // nothing was skipped: the newest sample is the last tick's
        let served = sup.get_observation().await;
        assert_eq!(served.timestamp, last);
        let c = sup.tick_stats().observation_cache;
        assert_eq!((c.hits, c.misses, c.publish_skipped, c.published), (1, 0, 0, 4));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_reader_that_waited_for_the_lock_serves_the_sample_published_meanwhile() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let mut writer = sup.inner().write().await;
        let reader = {
            let sup = sup.clone();
            tokio::spawn(async move { sup.get_observation().await })
        };
        tokio::time::sleep(Duration::from_millis(10)).await;
        // what a tick does under its guard
        let mut published = writer.current_observation();
        published.timestamp = 123.0;
        let generation = sup.inner().generation();
        let SupervisorInner { core, measured_positions, measured_velocities, .. } = &mut *writer;
        let _ = core.publish_displacing(generation, Instant::now(), Instant::now() + Duration::from_secs(1), published, measured_positions, measured_velocities);
        drop(writer);
        assert_eq!(reader.await.unwrap().timestamp, 123.0);
        let c = sup.tick_stats().observation_cache;
        assert_eq!((c.hits, c.misses), (1, 0));
    }

    /// Read-your-writes: a write after the tick is answered by the next tick's sample, not
    /// by the older one and not under the lock.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_write_waits_for_the_next_sample_while_the_loop_runs() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.tick(0.02).await;
        sup.set_temperatures(vec![55.0; 7]).await;
        let ticker = {
            let sup = sup.clone();
            tokio::spawn(async move {
                tokio::time::sleep(Duration::from_millis(5)).await;
                sup.tick(0.02).await
            })
        };
        let obs = sup.get_observation().await;
        let ticked = ticker.await.unwrap();
        assert_eq!(obs.temperatures, vec![55.0; 7]);
        assert_eq!(obs.timestamp, ticked.timestamp);
        let c = sup.tick_stats().observation_cache;
        assert_eq!((c.hits, c.misses, c.waited), (1, 0, 1));
    }

    /// A tick that stalls inside its write guard: GET /observation answers 503 within its
    /// bounds instead of waiting for the lock.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_stalled_tick_gives_observation_unavailable_in_bounded_time() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.tick(0.02).await;
        let stalled = sup.inner().write().await; // the tick never releases its guard
        let t = Instant::now();
        let r = tokio::time::timeout(Duration::from_secs(3), sup.observation()).await.expect("must not hang");
        let e = r.expect_err("no fresh state");
        assert!(matches!(&e, ArmError::Rejected { status: 503, code, .. } if code == "OBSERVATION_UNAVAILABLE"), "{e:?}");
        assert!(t.elapsed() < OBS_LOCK_WAIT + Duration::from_millis(300), "{:?}", t.elapsed());
        assert_eq!(sup.tick_stats().observation_cache.unavailable, 1);
        drop(stalled);
        assert!(sup.observation().await.is_ok());
    }

    /// The second lock wait (a sample was published while the first one waited, but it did
    /// not answer) shares the first wait's deadline, and a sample published by the writer
    /// that keeps it from the lock answers at that deadline instead of a 503.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn the_second_lock_wait_shares_one_deadline_and_ends_with_a_cache_check() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.tick(0.02).await;
        let mut w1 = sup.inner().write().await;
        let t = Instant::now();
        let reader = { let sup = sup.clone(); tokio::spawn(async move { sup.observation().await }) };
        tokio::time::sleep(Duration::from_millis(20)).await;
        // a second writer queues behind the reader (tokio grants in queue order)
        let w2 = {
            let sup = sup.clone();
            tokio::spawn(async move {
                let mut w = sup.inner().write().await;
                let generation = sup.inner().generation();
                let mut obs = w.current_observation();
                obs.timestamp = 456.0;
                let SupervisorInner { core, measured_positions, measured_velocities, .. } = &mut *w;
                let now = Instant::now();
                let _ = core.publish_displacing(generation, now, now + Duration::from_secs(5), obs, measured_positions, measured_velocities);
                // keeps the reader from the lock past its deadline
                tokio::time::sleep(OBS_LOCK_WAIT + Duration::from_millis(300)).await;
                drop(w);
            })
        };
        tokio::time::sleep(Duration::from_millis(280)).await;
        // a sample published while the reader waits, too old to answer: the reader retries
        let generation = sup.inner().generation();
        let mut stale = w1.current_observation();
        stale.timestamp = 123.0;
        let SupervisorInner { core, measured_positions, measured_velocities, .. } = &mut *w1;
        let past = Instant::now() - Duration::from_millis(1);
        let _ = core.publish_displacing(generation, past, past, stale, measured_positions, measured_velocities);
        drop(w1);
        let got = tokio::time::timeout(Duration::from_secs(3), reader).await.expect("must not hang").unwrap();
        let elapsed = t.elapsed();
        assert_eq!(got.expect("the final cache check answers").timestamp, 456.0);
        assert!(elapsed < OBS_LOCK_WAIT + Duration::from_millis(150), "the budget restarted: {elapsed:?}");
        w2.await.unwrap();
    }

    /// A preempted reader holding the hub mutex: a request after a write neither waits for
    /// it nor holds the read guard while waiting for it, so ticks keep running.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_reader_stuck_on_the_hub_does_not_block_the_fallback_or_the_tick() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.tick(0.02).await;
        sup.set_temperatures(vec![50.0; 7]).await;
        let (held_tx, held_rx) = std::sync::mpsc::channel();
        let (release_tx, release_rx) = std::sync::mpsc::channel::<()>();
        let holder = {
            let sup = sup.clone();
            std::thread::spawn(move || {
                let _g = sup.hub.reader_guard();
                held_tx.send(()).unwrap();
                release_rx.recv().unwrap();
            })
        };
        held_rx.recv().unwrap();
        let obs = tokio::time::timeout(Duration::from_secs(2), sup.observation()).await.expect("bounded").unwrap();
        assert_eq!(obs.temperatures, vec![50.0; 7]);
        tokio::time::timeout(Duration::from_millis(500), sup.tick(0.02)).await.expect("tick runs");
        release_tx.send(()).unwrap();
        holder.join().unwrap();
    }

    /// Recording boundaries: the saved frames are exactly the ticks between start and stop,
    /// with the values the 1.5 in-tick append recorded (the tick's observation timestamp,
    /// measured and commanded positions), also across ring drains.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn recording_holds_exactly_the_ticks_between_start_and_stop() {
        let tmp = tempfile::tempdir().unwrap();
        let sup = ArmSupervisor::new(ArmConfig::rebot_dm());
        sup.set_episodes_dir(Some(tmp.path().to_path_buf())).await;
        sup.tick(0.02).await;
        sup.start_recording(None, None).await.unwrap();
        let mut expected = Vec::new();
        for i in 0..300 {
            // move the commanded state so every frame differs
            sup.inner().write().await.commanded_positions[0] = i as f32 * 1e-3;
            let obs = sup.tick(0.02).await;
            expected.push(RecordFrame { timestamp: obs.timestamp, observation_state: obs.positions.clone(), action: obs.commanded_positions.clone() });
            if i % 100 == 0 { tokio::time::sleep(Duration::from_millis(5)).await; }
        }
        assert_eq!(sup.get_recording_state().await.1, 300);
        let (tx, rx) = std::sync::mpsc::channel();
        sup.save_recording_with(move |_, frames| { tx.send(frames.to_vec()).unwrap(); Ok(()) }).await.unwrap();
        // a tick after the stop fence records nothing
        sup.tick(0.02).await;
        let frames = rx.recv().unwrap();
        assert_eq!(frames.len(), expected.len());
        assert_eq!(frames, expected);
        assert_eq!(sup.inner().read().await.core.record_dropped(), 0);
    }

    /// The stop path takes the stop fence under the writer and collects the frames after
    /// releasing it: a dispatcher preempted inside the ring's consumer mutex delays the save,
    /// not the tick.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_stalled_ring_consumer_does_not_block_the_tick_during_a_stop() {
        let tmp = tempfile::tempdir().unwrap();
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.set_episodes_dir(Some(tmp.path().to_path_buf())).await;
        sup.start_recording(None, None).await.unwrap();
        for _ in 0..5 { sup.tick(0.02).await; }
        let ring = sup.inner().read().await.core.record_ring().unwrap().clone();
        let (held_tx, held_rx) = std::sync::mpsc::channel();
        let (release_tx, release_rx) = std::sync::mpsc::channel::<()>();
        let holder = {
            let ring = ring.clone();
            std::thread::spawn(move || {
                let _g = ring.consumer_guard();
                held_tx.send(()).unwrap();
                release_rx.recv().unwrap();
            })
        };
        held_rx.recv().unwrap();
        let saver = { let sup = sup.clone(); tokio::spawn(async move { sup.save_recording().await }) };
        tokio::time::sleep(Duration::from_millis(30)).await;
        assert!(!saver.is_finished(), "the save waits for the consumer");
        tokio::time::timeout(Duration::from_millis(500), sup.tick(0.02)).await.expect("the tick must not wait for the stop");
        release_tx.send(()).unwrap();
        holder.join().unwrap();
        saver.await.unwrap().unwrap();
        let st = sup.episode_store().await;
        assert_eq!(st.list_episodes(None, Some(1), None, &[])[0].sidecar.frames, 5);
    }

    /// A save before the first recorded tick is refused and the session keeps recording
    /// into the ring (not into the recorder).
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_save_before_the_first_tick_keeps_the_session() {
        let tmp = tempfile::tempdir().unwrap();
        let sup = ArmSupervisor::new(ArmConfig::rebot_dm());
        sup.set_episodes_dir(Some(tmp.path().to_path_buf())).await;
        sup.start_recording(None, None).await.unwrap();
        assert!(sup.save_recording().await.is_err());
        {
            let inner = sup.inner().read().await;
            assert!(inner.core.recording_active() && inner.recording.is_some());
        }
        for _ in 0..3 { sup.tick(0.02).await; }
        {
            let inner = sup.inner().read().await;
            assert_eq!((inner.recorder.frame_count(), inner.core.record_pushed()), (0, 3));
        }
        sup.save_recording().await.unwrap();
    }

    /// A full ring drops samples without waiting and the session reports them.
    #[test]
    fn recording_ring_overflow_is_explicit() {
        let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
        inner.recorder.start(Some("r".into()), Some(std::env::temp_dir().join("rt1_overflow.parquet"))).unwrap();
        inner.core.begin_recording();
        let n = crate::rt_core::record_ring::RECORD_RING_CAPACITY + 44;
        for _ in 0..n { inner.tick(0.02); }
        assert_eq!(inner.recording_frames(), crate::rt_core::record_ring::RECORD_RING_CAPACITY);
        let done = inner.core.finish_recording();
        assert_eq!((done.frames.len(), done.dropped), (crate::rt_core::record_ring::RECORD_RING_CAPACITY, 44));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn snapshot_expires_before_the_lease_or_feedback_freshness_flips() {
        let sup = ArmSupervisor::new(ArmConfig::rebot_dm());
        {
            let mut inner = sup.inner().write().await;
            let timeout = inner.lease_timeout;
            inner.active_lease = Some(ActiveLease {
                client_id: "c".into(),
                generation: 1,
                label: "x".into(),
                acquired_at: Instant::now() - timeout + Duration::from_millis(5),
                last_valid_motion_instant: None,
            });
        }
        let obs = sup.tick(0.02).await;
        assert_eq!(obs.lease_holder.as_deref(), Some("x"));
        tokio::time::sleep(Duration::from_millis(15)).await;
        // within two periods of the tick, but the lease has expired: recomputed
        assert_eq!(sup.get_observation().await.lease_holder, None);
        assert_eq!(sup.tick_stats().observation_cache.misses, 1);

        let mut inner = sup.inner().write().await;
        inner.active_lease = None;
        let mut obs = inner.current_observation();
        obs.feedback_age_ms = vec![Some(10), Some(240), None];
        let cap = Duration::from_secs(1);
        let (at, until) = inner.observation_valid_until(&obs, cap);
        assert_eq!(until - at, Duration::from_millis(9));
        assert_eq!(at, inner.start_time + Duration::from_secs_f64(obs.timestamp));
        obs.feedback_age_ms = vec![Some(300)];
        let (at, until) = inner.observation_valid_until(&obs, cap);
        assert_eq!(until - at, cap);
        // the deadline comes from the sample, not from when it is evaluated
        std::thread::sleep(Duration::from_millis(5));
        assert_eq!(inner.observation_valid_until(&obs, cap), (at, until));
    }
}
