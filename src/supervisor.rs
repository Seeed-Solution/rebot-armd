use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicU8, AtomicUsize, Ordering};
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
pub(crate) mod config;
mod motion;
pub(crate) mod streams;
mod actions;
pub(crate) mod terminal_capture;
mod torque_off;
mod workspace;
mod admission;
mod state_init;
mod terminal_plumbing;
mod types;
mod driver_shadow;
mod legacy_loop;
mod mode_switch;
mod observation;
mod ownership;
mod resting_thermal;
mod shutdown_park;

/// Broker side of the servo, teleop and policy latest-target path (track D); the WS loop
/// and the policy runner submit through it once they route through the mailbox.
pub use streams::targets;
pub use driver_transitions::{
    run_with_panic_deadline_observer, EnableLifecycle, PanicDisableOutcome, TorqueIntent,
    STOPPED_HOLD_CONTENTION_ESCALATE_SKIPS,
    PANIC_DISABLE_TIMEOUT,
};
pub(crate) use driver_transitions::{
    ClearFaultCommit, ClearFaultLegacyInnerError, ClearFaultLegacyWithAuthError,
    ClearFaultWithAuthError, ParkRearmStart,
};
pub(crate) use driver_transitions::{
    bounded_panic_disable_driver_until, panic_disable_deadlines,
    run_with_panic_deadline_observer_until,
};
pub use torque_off::{StoppedTorqueOffRefusal, STOPPED_TORQUE_OFF_FEEDBACK_MAX_AGE};
pub use workspace::{
    check_path_with, check_path_with_tcp_tol, keepout_sweep_with, keepout_sweep_with_tcp_tol,
    prepare_quintic, JointGoalSnapshot, PreparedQuintic,
};
pub(crate) use workspace::joint_goal_target_from;
pub use admission::{
    next_admission_request_id, AdmissionStamp, Admitted, ArmError, MotionStartKind,
    MotionStartOp, PreemptError, MAX_LIVE_CONNECTIONS,
};
pub(crate) use admission::{
    AdmittedProof, ClosedConnections, ConsumedStamps, LiveConnections, LiveDisconnectFenceView,
};
#[cfg(test)]
use admission::{BROKER_REQUEST_ID_BIT, RT_INTERNAL_REQUEST_ID_BIT};
pub use terminal_plumbing::{TerminalSubscription, TERMINAL_FIFO_CAPACITY};
pub(crate) use terminal_plumbing::TerminalBroadcaster;
pub use types::{
    ActiveLease, GripperBaselineCmd, GripperCalibrationCandidate, GripperCalibrationRun,
    GripperStreamContact, HeldObject, ParkContinuation, ParkContinuationPhase, ParkRelease,
    PendingZeroCheckPersist, TickPublished, UnloadSettle, UnloadSettleSample, WatchdogPark,
    WatchdogParkPending, ZeroCheckParams, ZeroCheckPersistContext, ZeroCheckStreamEvent,
    BUS_LOSS_TIMEOUT, CONTROL_DEADLINE, GRIPPER_BASELINE_PENDING_MAX, GRIPPER_END_LEAD_RAD,
    GRIPPER_FORCE_CAP_TICKS, GRIPPER_FORCE_RELEASE_MARGIN_RAD,
    GRIPPER_STALL_BASELINE_RAISE_MAX_NM, GRIPPER_STALL_LATCH_NM, GRIPPER_STALL_MIN_LEAD_RAD,
    GRIPPER_STALL_PROGRESS_EPS, GRIPPER_STALL_RELEASE_NM, GRIPPER_STALL_WINDOW_MAX_S,
    GRIPPER_STREAM_CAP_HOLD_FRACTION, GRIPPER_STREAM_CAP_RAMP_RAD_S,
    GRIPPER_STREAM_CONTACT_LOST_RAD, GRIPPER_STREAM_TARGET_RAISE_MAX_NM,
    JOINT_LIMIT_CLEAR_MARGIN_RAD, MAX_KEEPOUT_EVALS_PER_PLAN, MAX_KEEPOUT_EVALS_PER_TICK,
    MOTION_LIMIT_CLEAR_S, PARK_RELEASE_OPEN_RAD, PARK_RELEASE_SPEED, POWER_LOSS_SYNC_WINDOW_MS,
    PREEMPT_COOLDOWN, SERVO_BLOCKED_PROGRESS_RAD, THERMAL_CLEAR_MARGIN_C,
    THERMAL_FEEDBACK_FRESHNESS_LIMIT, UNLOAD_SETTLE_HOLD, UNLOAD_STILL_POSITION_SPAN_RAD,
    UNLOAD_STILL_RECENT, UNLOAD_STILL_VELOCITY_PEAK_FACTOR, WATCHDOG_PARK_MARGIN,
    WATCHDOG_PARK_SETTLE_GRACE, WATCHDOG_PARK_TIMEOUT,
};
pub(crate) use types::{
    translate_driver_cutoff_conservative, DeferredActionDone, DeferredCompletionTarget,
    DriverTxCapOutcome, EffectiveTxCutoff, J7PosForceGate, PendingRxPoll, RtRestingCheck,
    TickOutput,
};
use types::{sync_action_obs, UNLOAD_SETTLE_REPORT_AFTER_S};

pub struct SupervisorInner {
    pub config: ArmConfig,
    /// Optional startup-resolved multi-arm cell context (off by default, never touched on RT).
    pub(crate) cell: Option<Arc<crate::cell::CellContext>>,
    pub safety: SafetyManager,
    pub held: Option<HeldObject>,
    /// Preallocated reusable `HeldObject::label` buffer on RT (0 alloc / 0 free).
    pub(crate) held_label_spare: String,
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
    /// The torque transition the RT started and serves from `poll_hardware` (None: none).
    pub(crate) torque_transition: Option<driver_transitions::TorqueTransition>,
    /// Monotonic token generator for legacy WS torque transitions.
    pub(crate) legacy_torque_seq: u64,
    /// Preallocated per-request completion slots for legacy WS torque transitions (0 alloc/free on control release).
    pub(crate) legacy_torque_slots: [Arc<driver_transitions::LegacyTorqueSlot>; driver_transitions::LEGACY_TORQUE_SLOTS],
    /// Immutable driver capability cache captured on [`Self::install_driver`] so capability checks
    /// never lock `self.driver` or mistake mutex contention for missing capability.
    pub(crate) driver_has_load_feedback: bool,
    /// True for the duration of a legacy control release servicing an in-flight non-broker torque
    /// transition so every driver lock acquisition (`lock_driver_for_tick`) stays non-blocking.
    pub(crate) nonblocking_driver_release: bool,
    /// Set to `true` when a legacy WS torque completion slot publishes a terminal outcome under
    /// the write guard; `ArmSupervisor` reads & clears it before dropping the guard and calls
    /// `legacy_torque_notify.notify_waiters()` strictly OFF-LOCK (zero mutexes on control release).
    pub(crate) legacy_torque_settled_this_tick: bool,
    /// Lock-free mirror of [`Self::has_detached_torque_transition`] so off-lock headless waiters
    /// schedule bounded detached cleanup after any settlement (Stop, lease loss, or timeout).
    pub(crate) detached_torque_pending_atomic: Arc<AtomicBool>,
    /// When a contended clean Stop during legacy enable installs `DetachedCancel` provisionally
    /// as `FailedHold`, preserves the prior `TorqueIntent` so `step_enable_cancel` restores
    /// `Disarmed` if the driver later confirms zero frames dispatched and zero joints enabled.
    pub(crate) detached_cancel_restore_intent: Option<driver_transitions::TorqueIntent>,
    /// Static torque direction (`"on"` / `"off"`) set by [`Self::service_torque_transition`] ONLY
    /// on ticks that serviced a legacy WS torque handshake step (not Park / BrokerTransition),
    /// consumed by `tick_shared` for `/api/v1/torque` diagnostics attribution.
    pub(crate) last_tick_torque_ws_direction: Option<&'static str>,
    /// Persistent enable-lifecycle state owned by the single enable transition reducer.
    pub(crate) enable_lifecycle: driver_transitions::EnableLifecycle,
    /// Owner-owned torque decision state (`On` by default on hardware until an accepted disable
    /// or `DisableProtective` sets `Off`; never inferred from feedback telemetry).
    pub torque_intent: driver_transitions::TorqueIntent,
    /// Explicit owner-bound ProcessExit authorization flag (DRV-F1 B7).
    pub(crate) process_exit_authorized: bool,
    /// Monotonic control-release sequence counter pushed to the driver once per release so
    /// rest-pose K=3 ticks count control releases, never individual RX receive passes.
    pub(crate) control_release_seq: u64,
    /// Lock-free mirror of [`Self::control_release_seq`] so off-lock legacy WS waiters can detect
    /// whether a live control loop is ticking without contending for `SupervisorLock`.
    pub(crate) control_release_seq_atomic: Arc<AtomicU64>,
    /// Inline observation-only `TorqueController` shadow runtime (zero alloc, zero lock on RT).
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) torque_shadow: crate::torque::shadow::ShadowRuntime,
    /// Stable live operation identifier of the currently active enable attempt (for shadow HandshakeResult join).
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) shadow_enable_op_id: u64,
    /// Inline per-joint measured-position hold state (zero alloc on RT).
    pub(crate) measured_hold: driver_transitions::MeasuredHoldTracker,
    /// Inline owner-held driver snapshot refreshed whenever the owner already holds the driver
    /// lock so stopped-hold capture and MIT stopped-hold preparation never re-lock `self.driver`
    /// or allocate (Rule R1).
    pub(crate) owner_driver_snapshot: driver_transitions::OwnerDriverSnapshot,
    /// Bounded retry state for a non-broker background disable whose initial disable attempt
    /// failed on transport TX (routes every retry strictly through the driver's single
    /// `begin_disable_torque_budgeted(kind, 0)` disable choke point with the original `DisableKind`
    /// request timestamp and rest-pose gate within the 1 s lifetime).
    pub(crate) background_disable_retry: Option<driver_transitions::BackgroundDisableRetry>,
    pub last_disable_attempt: Option<Instant>,
    /// Active background/transition disable intent carried into driver sends and periodic retries.
    pub(crate) active_disable_kind: Option<crate::driver::DisableKind>,
    /// When the active disable originated from a client-initiated park (`!watchdog`), carries its
    /// captured `expected_stop_gen` so every retry preserves the Stop fence and never sends a
    /// motor its first disable write after Stop.
    pub(crate) active_disable_client_park: Option<Option<u64>>,
    pub(crate) last_stopped_torque_off_refusal: Option<StoppedTorqueOffRefusal>,
    /// Rate-limit timestamp for non-motion telemetry warnings when hardware reports `enable_state != Enabled`.
    pub(crate) last_reported_torque_loss_warn: Option<Instant>,
    /// Consecutive causal samples of reported unexpected torque loss initiated during active motion (R12 M2).
    pub(crate) reported_torque_loss_count: u8,
    /// Control release sequence of the last counted active-motion reported torque-loss sample (R12 M2).
    pub(crate) reported_torque_loss_last_release: u64,
    /// Driver causal sample sequence of the last counted active-motion reported torque-loss sample (R12 M2).
    pub(crate) reported_torque_loss_last_sample_seq: Option<u64>,
    /// True after an active-motion reported torque loss stopped motion and is tracking persistence to K=3 (R12 M2).
    pub(crate) reported_torque_loss_from_motion: bool,
    /// Consecutive RT disable dispatch/poll ticks skipped due to `driver.try_lock()` contention (M4).
    pub(crate) disable_lock_contended_skips: u32,
    /// Latched once the consecutive RT disable lock contention threshold (`N = 25`) escalates.
    pub(crate) disable_lock_contended_escalated: bool,
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
    /// Reusable preallocated [`crate::kinematics::WorkspaceViolationDetail`] moved into/out of
    /// `workspace_violation` on RT so workspace refusals and clears perform 0 heap ops.
    pub(crate) workspace_violation_spare: Option<crate::kinematics::WorkspaceViolationDetail>,
    /// Observation-clock time (s) `motion_limit` last applied.
    pub motion_limit_seen_s: f64,
    /// Holding after a blocked servo stream (keeps `motion_limit` = blocked).
    pub servo_blocked_hold: bool,
    /// Persistent monotonic escape baseline across a servo stream & its braking phase, tagged with `workspace_revision`.
    pub servo_ws_min_depths: Option<(u64, crate::kinematics::SlotBuf)>,
    /// Persistent velocity ceiling (rad/s) for the gripper's current hold target, set when a
    /// `GripperMotion` starts/steps and retained across completion and post-completion holds
    /// until replaced by a new gripper motion, an explicit non-gripper J7 move, or torque-off.
    pub(crate) gripper_hold_vel_ceiling: Option<f32>,
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
    /// Non-blocking Step-2 cache of `driver.is_feedback_valid()` (0 driver locks in Step 3/4).
    pub(crate) driver_feedback_confirmed_cached: bool,
    /// Inline owner-side reservation held during off-thread policy pack load/warmup on executor.
    #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
    pub(crate) policy_load_reservation: Option<crate::rt_core::requests::policy::PolicyLoadReservation>,
    /// Shared exclusive-lifetime lock-free cancellation slots for executor policy start/run.
    #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
    pub(crate) policy_cancel_slots: Arc<crate::rt_core::requests::policy::PolicyCancelSlots>,
    /// Lock-free single-producer ring publishing immutable RT policy terminal records off-thread
    /// (0 allocs, 0 frees, 0 locks on `armd-control`).
    #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
    pub(crate) policy_terminal_tx: Option<crate::rt_core::requests::policy::PolicyTerminalTx>,
    /// Most recent policy observation telemetry (retained after policy run ends).
    #[cfg(feature = "policy-runtime")]
    pub last_policy_obs: Option<crate::model::PolicyObservation>,
    /// Preallocated `PolicyObservation` and target-vector spares recycled on `armd-control`
    /// so starting, stepping, holding, superseding, and ending policy runs allocate and free 0 bytes.
    #[cfg(feature = "policy-runtime")]
    pub(crate) rt_policy_auth: crate::policy_runtime::PolicyAuthorization,
    #[cfg(feature = "policy-runtime")]
    pub(crate) policy_run_spare: Option<crate::policy_runtime::PolicyRun>,
    #[cfg(feature = "policy-runtime")]
    pub(crate) policy_live_vec_spare: Vec<f32>,
    #[cfg(feature = "policy-runtime")]
    pub(crate) policy_shadow_vec_spare: Vec<f32>,
    #[cfg(feature = "policy-runtime")]
    pub(crate) last_policy_obs_spare: Option<crate::model::PolicyObservation>,
    #[cfg(feature = "policy-runtime")]
    pub(crate) last_policy_obs_vec_spare: Vec<f32>,
    /// Highest terminated policy action ID awaiting post-publication atomic close watermark flush.
    #[cfg(feature = "policy-runtime")]
    pub(crate) pending_policy_close_max: u64,
    /// Monotonic watermark advanced strictly BEFORE Step 6 publish on policy termination so
    /// off-thread `StateHub` seals diagnostics before the first terminal publication.
    #[cfg(feature = "policy-runtime")]
    pub(crate) atomic_diag_sealed_policy_action_id: Arc<AtomicU64>,
    /// Monotonic watermark advanced strictly AFTER Step 6 publish (Step 7 / wake) so
    /// `ServoTargetDispatcher` closes terminated policy streams losslessly.
    #[cfg(feature = "policy-runtime")]
    pub(crate) atomic_closed_policy_action_id: Arc<AtomicU64>,
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
    /// Numeric WS connection ID (`>= 1`) owning `active_lease`, or `0` for non-WS / none.
    pub(crate) active_lease_conn_id: u64,
    /// Highest lease generation fenced by release, expiry, preemption, or disconnect.
    pub(crate) fenced_lease_generation: u64,
    /// Fixed zero-alloc queue of displaced lease generations awaiting off-thread `close_servo`
    /// dispatch on `armd-dispatch` (populated by `fence_displaced_lease_generation`).
    pub(crate) pending_servo_closes: [u64; 32],
    pub(crate) pending_servo_close_len: usize,
    pub lease_generation_counter: u64,
    /// Boot identity of the active owner (process boot in legacy, `ExecutorConfig::boot` in RT).
    pub(crate) boot_id: crate::rt_core::broker::BootId,
    /// Monotonic revision paired with `StateHub`'s off-thread `ObservationSchema` cache.
    pub(crate) schema_revision: u64,
    pub gripper_stalled: bool,
    pub gripper_high_torque_ticks: u32,
    pub gripper_low_torque_ticks: u32,
    pub gripper_stall_window: crate::motion::GripperProgressWindow,
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
    /// Finished unloaded gripper moves waiting to be learned off the control tick
    /// (`crate::gripper_calib::spawn_baseline_persister`); at most GRIPPER_BASELINE_PENDING_MAX.
    pub gripper_baseline_pending: Vec<crate::gripper_baseline::MoveRecord>,
    /// Running `gripper_calibrate` action: its strokes' records, learned only when every stroke
    /// passed (crate::gripper_calib).
    pub gripper_calibration: Option<GripperCalibrationRun>,
    /// Bumped by every baseline change made under the lock (reset, learned result stored); a
    /// learning pass computed from an older generation is dropped.
    pub gripper_baseline_generation: u64,
    /// Gripper target of the running servo stream or live policy (last accepted frame); None
    /// when no stream drives the gripper (no frame for `motion_frame_timeout`, an action or a
    /// whole-arm reference runs, torque changes).
    pub gripper_stream_target: Option<f32>,
    /// Contact regulator of the stream (`gripper_policy.stream_contact_speed_rad_s`).
    pub gripper_stream_contact: Option<GripperStreamContact>,
    /// Shared net-progress / moving-floor contact detector for the stream contact regulator.
    pub gripper_stream_contact_detector: crate::motion::GripperContactDetector,
    /// Lease generation the gripper stream target was set under (None without a lease).
    pub gripper_stream_lease: Option<u64>,
    /// A gripper action reference was active at the start of the previous tick ...
    pub gripper_ref_prev: bool,
    /// ... and it reported done in that tick ...
    pub gripper_ref_done: bool,
    /// ... with `holding: true` (a grasp squeeze hold on an object is kept).
    pub gripper_ref_holding: bool,
    /// Grip ramp of the current torque-cap latch (stiffness estimate, trips on this contact);
    /// None while no latch holds.
    pub gripper_cap_ramp: Option<crate::motion::CapRamp>,
    /// The previous tick was over the torque cap (a new over-cap episode counts a trip).
    pub gripper_over_cap_prev: bool,
    /// A park carried an object (J7 frozen short of closed): release it at the resting pose.
    pub park_gripper_contact: bool,
    pub park_release: Option<ParkRelease>,
    pub pending_normal_unload: bool,
    pub unload_settle: Option<UnloadSettle>,
    /// Set while a park started by the lease watchdog is running.
    pub watchdog_park: Option<WatchdogPark>,
    /// Inline reason the lease watchdog held position instead of starting a park (0 heap ops).
    pub watchdog_park_blocked_reason: Option<crate::model::HolderLabel>,
    /// Set when the last park was refused because its path enters a keep-out box.
    pub park_rejection: Option<SafetyError>,
    /// Bumped on every change of the workspace rule inputs (keep-out boxes, camera mount);
    /// a plan made outside the lock must match it at install (else PLAN_STALE).
    pub workspace_revision: u64,
    /// Camera calibration in effect (`calibration.json`, `crate::calibration`); None = the
    /// profile's camera mount.
    pub calibration: Option<crate::calibration::CalibrationFile>,
    /// SHA-256 of `calibration` (`active_calibration_digest`), recomputed by `set_calibration`
    /// so a live policy step compares it without serializing and hashing.
    #[cfg(feature = "policy-runtime")]
    pub(crate) calibration_digest: String,
    /// Why the last arm refused MIT tracking (`src/mit.rs` qualification gate); None = passed or
    /// MIT not requested.
    pub mit_gate_error: Option<String>,
    /// Admission-bound one-shot MIT qualification proof verified off-RT against fresh register
    /// readback + `mit::check_gate`, consumed atomically at `enable_prepare_rt_precheck`.
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) mit_gate_proof: Option<crate::rt_core::requests::config::MitGateProof>,
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
    /// Active executor calibration transaction token bound to `calibration_busy` / `motor_zero_busy`
    /// so stale cleanup can never clear a newer transaction's busy flags.
    pub(crate) calibration_txn_id: Option<u64>,
    /// Single preallocated tick-driven hardware commissioning state machine (C-F1 part 2).
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) commissioning: crate::mit::CommissioningMachine,
    /// Release-side session lifecycle driver (`ExclusiveOutput` + singleton `Recorder`).
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) sessions: crate::rt_core::session::SessionDriver,
    /// Inline RT recorder stream producer and frozen close fence (0 alloc, 0 lock on `armd-control`).
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) rt_recorder: crate::rt_core::recorder::RtRecorder,
    /// Preallocated tick-driven PR0C `ModeSwitch` session runner (0 RT allocs).
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) mode_switch: ModeSwitchRunner,
    /// `control_release_seq` on which `service_mode_switch_release` last stepped a live session
    /// (keeps `mode_switch_owns_bus` latched for the remainder of the terminal release so ordinary
    /// output and the J7 tick epilogue never emit duplicate frames after ModeSwitch finishes).
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) mode_switch_ran_release_seq: u64,
    /// Owner-recorded causal authority counters (owner generation, applied general-stop seq,
    /// applied E-stop seq, and qualifying fault-stop epoch). Updated only on the owner thread
    /// strictly after the corresponding stop effect or ownership transfer completes.
    pub(crate) authority: crate::rt_core::authority::OwnerAuthorityState,
    /// Shared per-release resource ledger captured at the end of the legacy control tick so
    /// post-tick commissioning draws from the exact same `ReleaseBudget` as RX/transitions/output.
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) last_legacy_release_budget: Option<crate::rt_core::budget::ReleaseBudget>,
    /// Set when `sync_driver_budget_before` returned `None` under contention so any subsequent
    /// driver borrow in the same step installs the conservative `(rx=0, tx=0)` cap and accounts
    /// exact incremental `(rx, tx)` deltas inside that borrow.
    pub(crate) pending_conservative_tx_cap: bool,
    pub(crate) conservative_step_rx_delta: u16,
    pub(crate) conservative_step_tx_delta: u16,
    /// True when startup found an incomplete or unreadable pending calibration commit marker.
    pub(crate) calibration_quarantined: bool,
    /// Cached raw driver position limits (updated off-RT / on bounded driver-limit commits) so
    /// RT gripper calibration validation never acquires the driver mutex on `armd-control`.
    pub(crate) cached_driver_pos_limits: [Option<(f32, f32)>; crate::rt_core::mailbox::MAX_DOF],
    /// `calibration.json` was rejected at startup (message; cleared by a successful apply / restore).
    pub calibration_load_error: Option<String>,
    /// Joint zero offsets in effect (motor - joint, rad, one per joint; calibration.json
    /// `joint_zero`), applied by the driver boundary ([`crate::joint_zero::ZeroOffsetDriver`]).
    pub joint_zero_offsets: Vec<f32>,
    /// Zero check against the profile's `zero_reference` (`zero_convention`).
    pub zero_convention: crate::joint_zero::ZeroConvention,
    /// Pending `legacy_j6` warning deferred from `armd-control` to off-thread log formatting.
    pub(crate) pending_legacy_j6_log: Option<crate::joint_zero::ZeroConvention>,
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
    pub driver: Option<crate::supervisor_lock::DriverSlot>,
    /// Shared counter of forbidden `DriverLock` acquisitions attempted on `armd-control`.
    pub(crate) driver_lock_attempts: Arc<AtomicU64>,
    pub gravity_compensated: bool,
    pub gravity_torques: Vec<f32>,
    pub gravity_scale: f32,
    pub gravity_torques_ramped: Vec<f32>,
    pub last_can_feedback_instant: Option<Instant>,
    /// Per-motor last valid RX timestamp (driver clock), preserved across stale periods and
    /// `J7FeedbackStale` faults so `BusLoss` can classify synchronized power/CAN interruptions.
    pub(crate) last_valid_motor_rx: [Option<Instant>; crate::model::MAX_MOTION_DOF],
    /// Deferred `ArmDriver::notify_bus_loss` if the driver slot could not be locked during BusLoss.
    pub(crate) pending_driver_bus_loss_notify: bool,
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
    /// Single authoritative atomic state packing `(generation << 1) | latched` for E-stop.
    pub estop_state: Arc<crate::rt_core::urgent::EstopState>,
    pub e_stop_latched: crate::rt_core::urgent::EstopLatchView,
    pub requested_estop_count: crate::rt_core::urgent::EstopCounterView,
    pub applied_estop_count: u64,
    /// Deterministic test hook fired strictly after `safety.clear_with_check` commits and
    /// immediately before `estop_state.try_clear_if_generation` attempts to clear bit 0.
    #[doc(hidden)]
    pub before_estop_unlatch_hook: Option<Box<dyn FnOnce() + Send + Sync>>,
    /// `applied_estop_count` for the bus driver's E-stop gate (`crate::rt_core::urgent`).
    pub estop_applied: Arc<AtomicU64>,
    /// Monotonic stop generation incremented under the owner lock whenever a Stop applies.
    /// Shared lock-free with [`ArmSupervisor`] so WS/REST admission can capture [`AdmissionStamp`].
    pub stop_gen: Arc<AtomicU64>,
    /// Lock-free mirror of the active lease generation (`0` when no lease is held).
    pub lease_gen_mirror: Arc<AtomicU64>,
    /// Lock-free closed-connection bitset shared with [`ArmSupervisor`] so socket close fences
    /// immediately across both legacy and RT owners without locking.
    pub(crate) shared_closed_bitset: Arc<crate::rt_core::requests::actions::ClosedBitset>,
    /// Lock-free highest lease generation fenced on immediate socket close (`fetch_max`).
    pub(crate) atomic_fenced_lease_gen: Arc<AtomicU64>,
    /// Lock-free count of queued socket-close fences awaiting owner drain (fail-closed hold when > 0).
    pub(crate) pending_disconnect_hold: Arc<AtomicU64>,
    pub last_preemption: Option<Instant>,
    /// Connection and generation most recently evicted by a forced acquire.
    pub last_evicted: Option<(String, u64)>,
    pub handover_settling_since: Option<Instant>,
    /// Running gravity-compensation session (`src/gravity_comp.rs`).
    pub gravity_comp: Option<crate::gravity_comp::GravityCompRun>,
    /// Bounded queue of retired gravity-compensation runs waiting for off-thread drop (0 RT free).
    pub(crate) retired_gravity_comp: std::collections::VecDeque<crate::gravity_comp::GravityCompRun>,
    /// Bounded queue of retired gravity-comp policies waiting for off-thread drop (0 RT free).
    pub(crate) retired_gravity_comp_policies: std::collections::VecDeque<crate::model::GravityCompPolicy>,
    /// Staged hardware gravity-comp entry payload awaiting PR0C `ModeSwitch` confirmation (0 RT alloc).
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) pending_gravity_enter: Option<crate::gravity_comp::PendingGravityCompEnter>,
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
    pub(crate) static_load_cache: crate::power::StaticLoadBaselineCache,
    pub(crate) static_load_tracking_cache: crate::power::StaticLoadBaselineCache,
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
    /// Learned free-motion torque of the active tool's jaw (`crate::gripper_baseline`); None
    /// until the first gripper action creates it (or a file loaded it).
    pub gripper_baseline: Option<crate::gripper_baseline::FreeTorqueBaseline>,
    /// File path for the baseline (`gripper_baseline.json`); None: not persisted.
    pub gripper_baseline_path: Option<PathBuf>,
    /// The baseline changed since it was last written.
    pub gripper_baseline_dirty: bool,
    /// Last grasp contact verdict (`observation.gripper.baseline.last_contact`).
    pub gripper_last_contact: Option<crate::gripper_baseline::ContactInfo>,
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
    /// Previous `driver_config_error` string retired on the RT thread for off-thread deallocation.
    pub(crate) retired_driver_config_error: Option<String>,
    /// Non-recoverable configuration persistence fault latched after bounded reconciliation fails.
    pub(crate) config_reconcile_fault: Option<crate::safety::BoundedText<256>>,
    /// Required continuous duration below `max_temperature - THERMAL_CLEAR_MARGIN_C` before
    /// `ThermalTrip` may clear (default `Duration::ZERO`).
    pub thermal_cooldown_duration: Duration,
    /// Consecutive fresh-sample thermal cooldown tracker (zero heap allocations; restarts
    /// whenever any joint's gap between consecutive fresh samples exceeds
    /// [`THERMAL_FEEDBACK_FRESHNESS_LIMIT`]).
    pub(crate) thermal_cooldown: ThermalCooldown,
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
    /// Live lock-free operator grant captured for an in-flight executor `zero_check(store_offset=true)`
    /// persistence transaction so PR C owner gates verify operator revocation on `armd-control` with 0 alloc.
    pub(crate) zero_check_persisting_grant: Option<(u64, crate::auth::OperatorGrant)>,
    /// Pending disk persist + commit payload produced by finish_action during ArmSupervisor::tick.
    pub pending_zero_check_persist: Option<PendingZeroCheckPersist>,
    /// Preallocated lock-free RT->off-RT stream producer for executor `gripper_zero_check`
    /// persistence handoff and context discard (uses `rt_core/stream`, 0 RT heap allocs/frees).
    pub(crate) zero_check_stream_tx: Option<crate::rt_core::stream::StreamProducer<ZeroCheckStreamEvent>>,
    /// Pending `Copy` persistence handoff queued by `finish_action` and flushed at release tail
    /// (stamped with live `stop_gen`) onto `zero_check_stream_tx`.
    pub(crate) pending_zero_check_stream_persist: Option<ZeroCheckStreamEvent>,
    /// True when `discard_zero_check_ctx_id_rt` pushed a `Discard` onto `zero_check_stream_tx`
    /// so `armd-control` wakes `armd-persist` immediately.
    pub(crate) zero_check_stream_pushed: bool,
    /// Inline fixed-capacity fallback queue of staged `persist_ctx_id`s to discard off-thread via
    /// `OffloadReclaim::DiscardZeroCheckCtx` whenever `zero_check_stream_tx` is full or closed.
    pub(crate) retired_zero_check_ctx_ids: [u64; 16],
    pub(crate) retired_zero_check_ctx_id_len: usize,
    /// Pending Legacy supervised_first_run qualification ledger write deferred off the control
    /// release lock (0 alloc, 0 file I/O in tick; B4).
    #[cfg(feature = "policy-runtime")]
    pub(crate) pending_policy_qual_persist: Option<(
        crate::policy_runtime::PolicyRun,
        crate::model::HolderLabel,
        crate::model::HolderLabel,
    )>,
    /// Retired `ZeroCheckPersistContext` payloads waiting for off-thread drop (never dropped on RT).
    pub(crate) retired_zero_check_ctx: std::collections::VecDeque<Box<ZeroCheckPersistContext>>,
    /// Whether operator auth is enforced (exposed in GET /api/v1/safety/describe).
    pub operator_auth_enforced: bool,
    /// Path to persisted tool commissioning confirmations (`tool_commissioning.json`).
    pub tool_commissioning_path: Option<PathBuf>,
    /// Timestamp when the active tool's commissioning was confirmed (if any).
    pub tool_commissioning_confirmed_at: Option<String>,
    /// Monotonic revision for tool commissioning state.
    pub tool_commissioning_revision: u64,
    /// Single-pass Step-2 hardware poll state consumed once by `tick_control` in the same release.
    pub(crate) pending_rx_poll: PendingRxPoll,
    /// Active per-tick TX cutoff installed for the current control output pass (`None` outside tick).
    pub(crate) active_tx_cutoff: Option<EffectiveTxCutoff>,
    /// Authorized-stop generations wired into `estop_gate()` when owned by an RT executor.
    pub(crate) stop_requests: Option<crate::rt_core::urgent::StopRequests>,
    /// True on the empty shell left inside `ArmSupervisor::inner` after executor ownership transfer.
    pub(crate) detached: bool,
    /// True when deferred zero-check persistence can accept a job (false if the dedicated
    /// persistence worker channel is saturated or disconnected).
    pub persist_capacity_available: bool,
    /// True once ownership has been transferred to the synchronous RT executor thread
    /// (`armd-control`); internal driver disable paths stay non-blocking and budgeted.
    pub rt_executor_owned: bool,
    /// Why the daemon is running under its current effective control-loop owner (0 alloc Copy).
    pub owner_reason: crate::model::OwnerReason,
    /// Irreversibly set by [`Self::commit_shutdown`] under the owner's authority once final
    /// shutdown readiness is confirmed; permanently blocks `clear_fault`, torque enable, and
    /// actuator motion admission while keeping hardware disable paths permitted.
    pub shutdown_committed: bool,
    /// True after a shutdown request was refused because hardware `enable_state()` was `Unknown`,
    /// so a repeated shutdown signal issues a bounded disable request instead of only re-checking.
    pub(crate) shutdown_refused_unknown: bool,
    /// Lock-free owner-side internal-fault lane shared with the process panic hook and API/WS
    /// ingress (DRV-F1 / FU-PANIC).
    pub internal_fault_lane: Arc<crate::panic_coord::InternalFaultLane>,
    /// True while a non-control-panic stop-direction park-to-rest is actively progressing on this owner.
    pub(crate) internal_fault_park_active: bool,
    /// Preallocated bounded journal of immutable E-stop acknowledgements recorded at the exact
    /// moment [`Self::consume_pending_estop`] applies each urgent request (cap 64; never grows).
    pub(crate) pending_estop_acks: std::collections::VecDeque<crate::rt_core::executor::EstopAck>,
    /// Highest E-stop request generation dropped if `pending_estop_acks` ever reached capacity 64.
    pub(crate) estop_ack_overflow_gen: u64,
    /// Preallocated queue of heap-owning motion payloads retired on `armd-control` for off-thread drop.
    pub(crate) pending_retired_stops: std::collections::VecDeque<crate::rt_core::urgent::RetiredStopPayloads>,
    /// Coalesced sorted inclusive intervals `(lo, hi)` of closed WS connection IDs (`>= 1`),
    /// set under the owner lock during teardown and checked on every motion/output install.
    pub(crate) closed_connections: ClosedConnections,
    /// Fixed-capacity table of currently registered live WS connection IDs (`MAX_LIVE_CONNECTIONS`).
    /// Zero heap allocations; rejects unknown or closed non-zero connection IDs fail-closed.
    pub(crate) live_connections: LiveConnections,
    /// Preallocated string spares recycled across active lease and eviction transitions (0 alloc/free on RT).
    pub(crate) lease_client_spare: String,
    pub(crate) lease_label_spare: String,
    pub(crate) evicted_conn_spare: String,
    /// Bounded, preallocated single-use tracker for consumed global admission request IDs.
    consumed_stamps: ConsumedStamps,
    /// Bounded, preallocated single-use tracker for consumed RT broker `RequestId` values.
    consumed_broker_stamps: ConsumedStamps,
    /// Separate tracker for RT-internal IDs (`RT_INTERNAL_REQUEST_ID_BIT`, one per executor
    /// servo/jog frame). They share the counter with ordinary IDs, so in one tracker the two
    /// namespaces fragment each other; at the interval cap the fail-closed fold then moved the
    /// watermark past every ordinary ID (all ordinary commands refused with STAMP_REUSED).
    consumed_internal_stamps: ConsumedStamps,
    /// Shared per-connection bounded action-terminal FIFO broadcaster (wired by `ArmSupervisor::new`;
    /// `None` on standalone `SupervisorInner::new`, never locked on `armd-control`).
    terminals: Option<Arc<TerminalBroadcaster>>,
    /// When true (inside `ArmSupervisor::tick_shared` write guard), terminal action transitions
    /// stage into `staged_terminals` and flush via the lock-free SPSC ring instead of touching
    /// `TerminalBroadcaster`'s mutex under the tick guard.
    pub(crate) defer_terminal_capture: bool,
    /// Lock-free SPSC producer for edge-triggered fixed-size terminal records (`&mut self` only).
    terminal_capture_tx: Option<terminal_capture::TerminalCaptureTx>,
    /// Shared off-thread consumer + atomic accounting for terminal captures (`rx` never locked on RT).
    terminal_capture_channel: Option<Arc<terminal_capture::TerminalCaptureChannel>>,
    /// Inline fixed-size staging buffer for terminal transitions within one release before publish.
    staged_terminals: terminal_capture::StagedTerminalBuf,
    /// Fixed-size dedupe tracker keyed by `(action_id, terminal_seq)` preventing dwell re-capture.
    terminal_edge_tracker: terminal_capture::TerminalEdgeTracker,
    /// Accumulated wall time (µs) spent in `capture_edge_terminals` during the latest release.
    last_phase_terminal_us: u32,
    /// Wall time (µs) spent strictly in `StatePublisher::publish_with_rt_meta` during the latest
    /// release (disjoint from `last_phase_terminal_us`).
    last_publish_only_us: u32,
    /// Context-switch baseline sampled right before the FIRST terminal capture of the release
    /// (whether early supersession/materialization or `capture_edge_terminals`), consumed by
    /// `publish_at` / RT Executor so `mid_nvcsw` attributes stalls in every capture path.
    first_terminal_csw_mark: Option<crate::diagnostics::tick::CswMark>,
    /// Preserved `mid_csw_start` from the latest `publish_at` so RT Executor (`CoreOps::publish`
    /// drops `TickPublished`) can still compute `mid_nvcsw`/`mid_nivcsw` across capture+publish.
    last_mid_csw_start: Option<crate::diagnostics::tick::CswMark>,
    /// Dedicated per-tick cached driver state read exclusively by terminal capture (isolated
    /// from hardware park/torque-off interlock fields `last_poll_*`).
    pub(crate) capture_driver_cache: terminal_capture::CaptureDriverCache,
    /// Prebuilt template observation used only by off-tick terminal drain when `hub` has not published.
    template_obs_for_drain: Option<Arc<ArmObservation>>,
    /// Doc-hidden deterministic test seam invoked at entry (`true`) and exit (`false`) of
    /// `capture_edge_terminals` (`None` in production, 0 alloc).
    #[doc(hidden)]
    pub terminal_capture_test_hook: Option<Arc<dyn Fn(&SupervisorInner, bool) + Send + Sync>>,
    /// Monotonic count of deferred safety log events dropped at the RT->off-thread SPSC ring.
    pub(crate) safety_log_ring_dropped: u64,
    /// Deterministic test hook invoked inside `publish_at` strictly before `core.publish(...)`.
    pub(crate) before_publish_hook: Option<Arc<dyn Fn() + Send + Sync>>,
    /// Per-joint consecutive non-stopped ticks without a completed motion target TX (preallocated
    /// once at `SupervisorInner::new`; mutated strictly in place -> 0 RT allocs/frees).
    pub(crate) tx_joint_no_progress_ticks: Vec<u32>,
    /// Driver-clock instant of the first missed tick in each joint's current no-progress run
    /// (preallocated once at `SupervisorInner::new`; mutated strictly in place -> 0 RT allocs/frees).
    pub(crate) tx_joint_no_progress_since: Vec<Option<Instant>>,
    /// Per-`(joint, slot)` consecutive stopped-hold ticks in which a due hold frame failed to
    /// reach the transport (queued, expired, or hard TX error; preallocated once at `new` -> 0 RT
    /// allocs/frees).
    pub(crate) hold_tx_no_progress_ticks: Vec<u32>,
    /// Driver-clock instant of the first missed stopped-hold tick per `(joint, slot)`
    /// (preallocated once at `new`; mutated strictly in place -> 0 RT allocs/frees).
    pub(crate) hold_tx_no_progress_since: Vec<Option<Instant>>,
    /// True once `CAN_TX_STALLED` has been installed for the current stopped/fault session
    /// (prevents repeated fault re-latching; cleared in `clear_fault_finish`).
    pub(crate) hold_tx_stalled_latched: bool,
    /// Last driver-clock instant when a stopped-hold backlog stall diagnostic was logged (<= 1/s).
    /// Kept separate from `last_hold_tx_error_log` so backlog diagnostics never suppress `"Stopped hold output failed"`.
    pub(crate) last_hold_tx_stall_log: Option<Instant>,
    /// Last driver-clock instant when `"Stopped hold output failed"` was logged (<= 1/s throttle).
    pub(crate) last_hold_tx_error_log: Option<Instant>,
    /// Last driver-clock instant accepted by the active stopped-hold error sink (used on RT
    /// Executor to roll back `last_hold_tx_error_log` if `safety_log_tx` rejects an event).
    pub(crate) last_hold_tx_error_delivered: Option<Instant>,
    /// Exact count of `"Stopped hold output failed"` events suppressed by the 1/s rate limit or
    /// retained after deferred-log ring saturation; emitted on the next allowed log window.
    pub(crate) hold_tx_error_suppressed: u64,
    /// Last rejected `(partial, message, logged_at)` awaiting immediate retry once `safety_log_tx`
    /// has capacity (zero RT heap allocation).
    pub(crate) pending_hold_tx_error_retry:
        Option<(bool, crate::safety::BoundedText<160>, Instant)>,
    /// Driver-clock anchor `(pf_fresh_gen, since)` when continuous J7 position-force staleness
    /// (`> 100 ms`) began for the current feedback generation; cleared on fresh J7 feedback,
    /// feedback-generation advance, or `clear_fault_finish_bounded`.
    pub(crate) j7_stale_since: Option<(u64, Instant)>,
    /// Lock-free handle to the attached driver's clock (when provided by the driver) so RT
    /// deferred-log forwarding always timestamps acceptance on the exact driver clock.
    pub(crate) driver_clock: Option<Arc<dyn crate::transport::DriverClock>>,
    /// Action / macro-sub / park-release completion awaiting bus acceptance (`Complete`) across
    /// one or more transient `Deferred` ticks.
    pub(crate) pending_deferred_done: Option<DeferredActionDone>,
    /// Scheduled POS_VEL handback for gravity compensation, armed strictly post-output after
    /// `exit_hold_ms` (and stationary settle when tracking is enabled) so the mode switch runs
    /// on the next release's dedicated TX pass without bypassing the current release's cutoff.
    pub(crate) gravity_comp_handback_ready: bool,
    /// Preallocated terminal action message buffers reused on RT (0 alloc / 0 free across
    /// consecutive RT-started failures occupying both `last_action` and `previous_action`).
    pub(crate) action_msg_spares: [Option<String>; 4],
    /// Retired `ActiveTrajectory` instances queued for off-thread drop on RT executor (0 free on RT).
    pub(crate) retired_trajectories: std::collections::VecDeque<ActiveTrajectory>,
    /// Retired `MacroRun` instances queued for off-thread drop on RT executor (0 free on RT).
    pub(crate) retired_macro_runs: std::collections::VecDeque<crate::macro_action::MacroRun>,
    /// Retired `PolicyRun` instances queued for off-thread drop on RT executor (0 free on RT).
    #[cfg(feature = "policy-runtime")]
    pub(crate) retired_policy_runs: std::collections::VecDeque<crate::policy_runtime::PolicyRun>,
    /// Preallocated reusable `UnloadSettle` ring returned on stop/finish (0 alloc / 0 free on RT).
    pub(crate) unload_settle_spare: Option<UnloadSettle>,
    /// Preallocated reusable `Box<GripperMotion>` for park object-release on RT (0 alloc / 0 free).
    pub(crate) park_release_spare: Option<Box<GripperMotion>>,
    /// Monotonic revision bumped on every change to motion planning inputs (limits, workspace,
    /// keep-out boxes, camera mount, kinematics, commissioning, gripper calibration, power).
    pub motion_config_revision: u64,
    /// Monotonic generation bumped on every motion lifecycle transition (trajectory install/clear,
    /// servo stream start/stop, gravity comp enter/exit, stop, E-stop, clear-fault).
    pub(crate) motion_generation: u64,
    /// Last motion-state fingerprint observed at publication time (catches direct field transitions
    /// without allocating on RT).
    last_published_motion_fp: u64,
    /// Static hardware driver capabilities cached once in [`Self::with_driver`] (0 driver lock on RT).
    pub(crate) driver_caps: DriverCapabilities,
    /// Step-2 / RX-poll cached hardware enable & freshness snapshot (0 extra driver lock on RT settle).
    pub(crate) last_poll_enable_state: EnableState,
    pub(crate) last_poll_partial_enable: bool,
    pub(crate) last_poll_oldest_age: Option<Duration>,
    pub(crate) last_poll_feedback_valid: bool,
    /// Active off-owner deferred park continuation (MIT handback or `holding_after_clear` rearm).
    pub(crate) park_continuation: Option<ParkContinuation>,
    /// Off-RT waiter notification for legacy WS park continuations (never touched on `rt_executor_owned`).
    pub(crate) park_continuation_notify: Arc<tokio::sync::Notify>,
    /// Bounded planning deadline applied to off-lock legacy WS and `ArmSupervisor::{jog,home}` plans.
    pub(crate) motion_plan_deadline: Duration,
    /// Last executor/release clock instant observed on `tick_control_with_deadline` or transition wake,
    /// mirrored into `RtSample.clock_now` so off-RT snapshots inherit the owner's exact clock origin.
    pub(crate) last_clock_now: Option<Instant>,
    /// Pending off-thread watchdog park preparation (set on lease expiry instead of planning on tick).
    pub(crate) watchdog_park_pending: Option<WatchdogParkPending>,
    /// Monotonic generation counter for off-thread watchdog park requests.
    pub(crate) watchdog_park_seq: u64,
    /// Set by `ArmSupervisor::tick_shared` so legacy watchdog expiry never plans under `SupervisorLock`.
    pub(crate) defer_watchdog_park_plan: bool,
    /// Set by `ArmSupervisor::tick_shared` so legacy commissioning runs in separate `<= 2 ms`
    /// write guards strictly AFTER tick output & publish, never inflating the control-tick hold.
    pub(crate) defer_legacy_commissioning: bool,
    /// Current control tick's J7 `PosForce` intent set by arm/motion/hold paths and consumed
    /// strictly once by the tick epilogue's single J7 emitter (`emit_j7_tick_epilogue`).
    pub(crate) j7_intent: Option<crate::driver::PosForceIntent>,
    /// Admission `(stop_gen, estop_epoch)` captured directly from the A1 [`AdmissionStamp`] of
    /// the command that produced the active J7 target. Never re-stamped at output time and never
    /// stamped with `max(local, requested)`; cleared on stop, E-stop, and torque-off.
    pub(crate) j7_admitted_stamp: Option<crate::driver::PosForceIntentStamp>,
    /// Single-sample Step 2 snapshot of J7 PosForce freshness and freeze position for the current
    /// RT release (`Some` from Step 2 `poll_hardware_with_release_deadline` until Step 5
    /// `tick_control_inner` exit), eliminating TOCTOU and extra driver locks during Step 4 intake.
    pub(crate) rt_j7_pf_gate: Option<J7PosForceGate>,
    /// Optional deterministic test seam invoked right after `mit.rs` checks
    /// `urgent_stop_unapplied()` and before J7 intent stamping (`None` in production, 0 alloc).
    #[doc(hidden)]
    pub post_stop_check_hook: Option<Arc<dyn Fn() + Send + Sync>>,
    /// Preallocated reusable `Box<GripperMotion>` shell for zero-alloc / zero-free RT gripper actions.
    pub(crate) gripper_motion_spare: Option<Box<GripperMotion>>,
    /// Preallocated reusable `MoveSamples` buffers for zero-alloc RT `LearnMode::Opening` moves.
    pub(crate) move_samples_spares: Vec<crate::gripper_baseline::MoveSamples>,
    /// Cached scalar `driver.capabilities().torque_feedback` (true without driver) so RT gripper
    /// commits check close-torque capability without acquiring `driver.lock()`.
    pub(crate) driver_has_torque_feedback: bool,
    /// Cached scalar pos-force gripper close speed limit (rad/s) for lock-free RT gripper commits.
    pub(crate) driver_gripper_close_speed_limit: Option<f32>,
    /// Cached scalar pos-force gripper ceiling torque estimate (N·m) for lock-free RT gripper commits.
    pub(crate) driver_gripper_ceiling_torque_estimate: Option<f32>,
    /// Monotonic revision for `config.power_budget` (starts at 1; checked by prepared actions).
    pub power_budget_revision: u64,
    /// Lock-free exact closed-connection bitset shared with the RT `Broker` (checked on RT commits).
    pub(crate) rt_closed_connections: Option<Arc<crate::rt_core::requests::actions::ClosedBitset>>,
}

impl SupervisorInner {

    /// Captures a single consistent `(stale, freeze_pos)` snapshot for J7 `PosForce` gating.
    /// Reuses Step 2's cached [`Self::rt_j7_pf_gate`] during an RT release (zero driver lock
    /// attempts on Step 4); outside an RT release, reads both fields under a single `try_lock()`.
    pub(crate) fn sample_pos_force_j7_gate(&self) -> J7PosForceGate {
        if let Some(gate) = self.rt_j7_pf_gate {
            return gate;
        }
        if self.check_pos_force_gripper_fresh_static().is_ok() {
            return J7PosForceGate::default();
        }
        J7PosForceGate {
            stale: true,
            freeze_pos: self.pos_force_j7_freeze_position(),
        }
    }

    /// Refuse new gripper targets while active `PosForce` J7 feedback is stale (`> 100 ms`).
    /// Returns [`ArmError::RejectedStatic`] so callers on `armd-control` allocate zero memory.
    pub(crate) fn check_pos_force_gripper_fresh(&self) -> Result<(), ArmError> {
        if let Err(code) = self.check_pos_force_gripper_fresh_static() {
            return Err(ArmError::RejectedStatic {
                status: 409,
                code,
                message: "Gripper (J7) position-force feedback is stale (> 100 ms); new gripper targets are refused until fresh feedback returns",
            });
        }
        Ok(())
    }

    /// Zero-alloc failure of a running or deferred-done gripper action (or `gripper=true` replay)
    /// when active `PosForce` J7 feedback is stale (`> 100 ms`). Snaps `commanded_positions[joint]`
    /// and `target_positions[joint]` to the last fresh J7 position and ends the action with code
    /// `"J7_FEEDBACK_STALE"`.
    fn fail_running_gripper_for_j7_stale_rt(&mut self, joint: usize, deferred: Option<DeferredActionDone>) {
        let freeze_pos = self.pos_force_j7_freeze_position();
        let active_gripper_id = if self.active_reference_drives_gripper() {
            let old = self.active_trajectory.take();
            let id = old.as_ref().and_then(|a| a.action_id);
            if let Some(traj) = old {
                self.retire_active_trajectory(traj);
            }
            Some(id)
        } else {
            None
        };
        let deferred_gripper = deferred.filter(|d| d.is_gripper).or_else(|| {
            self.pending_deferred_done
                .as_ref()
                .is_some_and(|d| d.is_gripper)
                .then(|| self.pending_deferred_done.take())
                .flatten()
        });
        if active_gripper_id.is_none() && deferred_gripper.is_none() {
            return;
        }
        if let Some(fp) = freeze_pos {
            if joint < self.commanded_positions.len() {
                self.commanded_positions[joint] = fp;
            }
            if joint < self.target_positions.len() {
                self.target_positions[joint] = fp;
            }
        }
        if joint < self.commanded_velocities.len() {
            self.commanded_velocities[joint] = 0.0;
        }
        if let Some(id_opt) = active_gripper_id {
            if id_opt.is_none() && self.park_release == Some(ParkRelease::Opening) {
                self.park_release = None;
                self.park_gripper_contact = false;
                self.releasing = false;
            }
            if !self.pending_normal_unload {
                self.safety.mark_motion_finished();
            }
            let macro_sub = self.macro_run.as_ref().is_some_and(|m| Some(m.id) == id_opt);
            if macro_sub {
                if let Some(m) = self.macro_run.as_mut() {
                    m.sub = SubState::Failed {
                        code: crate::safety::BoundedText::new("J7_FEEDBACK_STALE"),
                        message: crate::motion::StepErrorText::new(
                            "Gripper (J7) position-force feedback became stale (> 100 ms)",
                        ),
                    };
                }
            } else if let Some(id) = id_opt {
                let mut msg = self.take_action_msg_buf(id, 160);
                msg.clear();
                msg.push_str("Gripper (J7) position-force feedback became stale (> 100 ms)");
                self.finish_action(id, "failed", Some("J7_FEEDBACK_STALE"), Some(msg), None, None, None);
            }
        }
        if let Some(done) = deferred_gripper {
            if !self.pending_normal_unload {
                self.safety.mark_motion_finished();
            }
            match done.target {
                DeferredCompletionTarget::MacroSub { action_id, ticket } => {
                    if let Some(m) = self.macro_run.as_mut().filter(|m| {
                        m.id == action_id && m.ticket() == Some(ticket) && !m.stop_requested && m.sub == SubState::Running
                    }) {
                        m.sub = SubState::Failed {
                            code: crate::safety::BoundedText::new("J7_FEEDBACK_STALE"),
                            message: crate::motion::StepErrorText::new(
                                "Gripper (J7) position-force feedback became stale (> 100 ms)",
                            ),
                        };
                    }
                }
                DeferredCompletionTarget::Action(id) => {
                    let mut msg = self.take_action_msg_buf(id, 160);
                    msg.clear();
                    msg.push_str("Gripper (J7) position-force feedback became stale (> 100 ms)");
                    self.finish_action(id, "failed", Some("J7_FEEDBACK_STALE"), Some(msg), None, None, None);
                }
                DeferredCompletionTarget::ParkRelease(ParkRelease::Opening) => {
                    self.park_release = None;
                    self.park_gripper_contact = false;
                    self.releasing = false;
                }
                DeferredCompletionTarget::ParkRelease(ParkRelease::Closing)
                | DeferredCompletionTarget::MacroFinish { .. }
                | DeferredCompletionTarget::Anonymous => {}
            }
        }
    }

    /// Maximum consecutive non-stopped ticks any joint has gone without a completed target TX.
    fn max_tx_joint_no_progress_ticks(&self) -> u32 {
        let n = self.config.dof.min(self.tx_joint_no_progress_ticks.len());
        self.tx_joint_no_progress_ticks[..n].iter().copied().max().unwrap_or(0)
    }

    /// Maximum elapsed driver-clock duration any joint has gone without a completed target TX.
    fn max_tx_joint_no_progress_age(&self, now: Instant) -> Option<Duration> {
        let n = self.config.dof.min(self.tx_joint_no_progress_since.len());
        self.tx_joint_no_progress_since[..n]
            .iter()
            .flatten()
            .map(|since| now.saturating_duration_since(*since))
            .max()
    }

    /// Zero-allocation motion stall latch (`CAN_TX_STALLED`) routed through `SafetyManager::latch`.
    fn latch_can_tx_stalled_motion(&mut self, detail: crate::safety::TxStallDetail, now: Instant) {
        self.reset_tx_joint_progress();
        if let Some(d) = self.driver.as_mut() {
            let _ = d.try_with_mut(|drv| {
                drv.tx_mut().discard_pending_tx();
                drv.tx_mut().clear_tx_probe();
            });
        }
        self.stop_motion();
        self.tracking.invalidate();
        self.commanded_velocities.fill(0.0);
        if self.measured_positions.iter().all(|q| q.is_finite()) {
            copy_into(&mut self.commanded_positions, &self.measured_positions);
            copy_into(&mut self.target_positions, &self.measured_positions);
        }
        if !self.safety.state().is_stopped() {
            self.safety.latch(
                crate::safety::FaultKind::CanTxStalled,
                crate::safety::FaultDetail::CanTxStalled(Some(detail)),
            );
            self.hold_tx_stalled_latched = true;
            self.last_hold_tx_stall_log = Some(now);
        }
    }

    /// Per-motor (`(joint, slot)`) stopped-hold lack-of-progress accounting across ALL hold
    /// outcomes (`Complete`, `Deferred`, `Expired`, and hard `Error`):
    /// - Counts ticks in which a due motor's stopped-hold frame failed to reach the transport,
    ///   whether it remained queued, expired (`pending == 0`), or failed with a hard TX error.
    /// - Each motor's streak changes ONLY on that motor's own outcome: one motor's refusal
    ///   (e.g. stale feedback on J1 removing J1 from `due_mask`) never clears another motor's
    ///   no-progress streak.
    /// - When `reset_when_empty` is `true` and `due_mask == 0` (idle / disabled / nothing due),
    ///   resets progress counters and never latches `CAN_TX_STALLED`; when `reset_when_empty` is
    ///   `false` (e.g. an error pass where `due_mask == 0`), existing streaks are left intact.
    fn account_stopped_hold_progress(
        &mut self,
        now: Instant,
        max_age: Duration,
        due_mask: u64,
        sent_mask: u64,
        queued: usize,
        dropped: usize,
        reset_when_empty: bool,
    ) {
        self.reset_tx_joint_progress();
        if due_mask == 0 {
            if reset_when_empty {
                self.reset_hold_tx_joint_progress();
            }
            return;
        }
        let slots = self
            .config
            .dof
            .saturating_mul(2)
            .min(self.hold_tx_no_progress_ticks.len())
            .min(self.hold_tx_no_progress_since.len())
            .min(64);
        let mut max_slot_ticks = 0u32;
        let mut max_slot_age = Duration::ZERO;
        for s in 0..slots {
            let bit = 1u64 << s;
            if (due_mask & bit) == 0 || (sent_mask & bit) != 0 {
                self.hold_tx_no_progress_ticks[s] = 0;
                self.hold_tx_no_progress_since[s] = None;
            } else {
                let ticks = self.hold_tx_no_progress_ticks[s].saturating_add(1);
                self.hold_tx_no_progress_ticks[s] = ticks;
                let since = *self.hold_tx_no_progress_since[s].get_or_insert(now);
                let age = now.saturating_duration_since(since);
                if ticks > max_slot_ticks {
                    max_slot_ticks = ticks;
                }
                if age > max_slot_age {
                    max_slot_age = age;
                }
            }
        }
        if max_slot_ticks == 0 || max_slot_age < max_age {
            return;
        }
        let mut escalated_now = false;
        if self.can_escalate_to_can_tx_stalled() {
            let detail = crate::safety::TxStallDetail {
                queued,
                dropped,
                age_ms: max_slot_age.as_millis().min(u128::from(u64::MAX)) as u64,
                limit_ms: max_age.as_millis().min(u128::from(u64::MAX)) as u64,
                consecutive_ticks: max_slot_ticks,
                is_hold: true,
            };
            self.safety.latch(
                crate::safety::FaultKind::CanTxStalled,
                crate::safety::FaultDetail::CanTxStalled(Some(detail)),
            );
            self.hold_tx_stalled_latched = true;
            // Single log event on the transition (emitted or deferred by `SafetyManager::latch`).
            self.last_hold_tx_stall_log = Some(now);
            escalated_now = true;
        }
        if !escalated_now && !self.safety.deferred_logging() && self.should_log_hold_tx_stall(now) {
            let age_ms = max_slot_age.as_millis().min(u128::from(u64::MAX)) as u64;
            let limit_ms = max_age.as_millis().min(u128::from(u64::MAX)) as u64;
            if dropped > 0 {
                tracing::error!(
                    dropped,
                    queued,
                    age_ms,
                    limit_ms,
                    consecutive_ticks = max_slot_ticks,
                    "Stopped hold CAN TX expired without reaching transport"
                );
            } else {
                tracing::error!(
                    queued,
                    age_ms,
                    limit_ms,
                    consecutive_ticks = max_slot_ticks,
                    "Stopped hold CAN TX stalled without reaching transport within age limit"
                );
            }
        }
    }

    /// Current owner `(stop_gen, estop_epoch, lease_gen)` stamp read in the tick epilogue (and
    /// used for non-target `Hold`/`Stop` intents). NEVER used to stamp a `PosForceIntent::Target`:
    /// every `Target` carries the A1 [`AdmissionStamp`] captured in `self.j7_admitted_stamp`.
    #[inline]
    pub(crate) fn current_j7_intent_stamp(&self) -> crate::driver::PosForceIntentStamp {
        let lease_gen = if self.pending_disconnect_hold.load(Ordering::SeqCst) > 0 {
            0
        } else {
            let cur = self.lease_gen_mirror.load(Ordering::SeqCst);
            let fenced = self.atomic_fenced_lease_gen.load(Ordering::SeqCst);
            if cur > 0 && cur > fenced { cur } else { 0 }
        };
        crate::driver::PosForceIntentStamp {
            stop_gen: self.current_stop_gen(),
            estop_epoch: self.safety.epoch_id(),
            lease_gen,
        }
    }

    /// Single owner-side J7 `PosForce` (`0x307`) emitter invoked strictly ONCE per control tick
    /// via [`Self::emit_j7_tick_epilogue`] in `tick_control_inner` (shared by BOTH Legacy and RT
    /// Executor owners).
    ///
    /// Runs AFTER whatever arm-write path ran (whether that path succeeded, refused, tripped the
    /// MIT fence, hit `BuildRefused`, had an invalid caller target, or returned early) and merges
    /// J7's progress masks and freshness status into `tick_output` without heap allocation.
    fn emit_and_merge_pos_force_j7(
        drv: &mut dyn ArmDriver,
        j7_is_hold: bool,
        intent: crate::driver::PosForceIntent,
        current_stamp: crate::driver::PosForceIntentStamp,
        cutoff: Option<EffectiveTxCutoff>,
        tick_output: &mut TickOutput,
        allow_output_accepted: &mut bool,
    ) -> Option<(usize, crate::driver::JointHoldStatus)> {
        use crate::driver::{ArmDriverError, JointHoldStatus, PosForceEmitOutcome};
        // Reported `drv.enable_state()` is telemetry only (R10): never suppress J7 hold or
        // freeze frames based on feedback-claimed enable state.
        // Single effective TX cutoff with the stop/hold/freeze exception:
        // - Fresh J7 `Target` motion respects the active action/macro deadline (`drv_action_cutoff`)
        //   identically to arm motion frames (zero post-deadline target frames on the wire).
        // - Stop / Hold / Freeze-class J7 frames (`j7_is_hold`, explicit `Hold`/`Stop`, stale
        //   admission stamp degraded to `Hold`, or stale J7 feedback requiring a FREEZE frame)
        //   restore the unclipped control-release deadline (`drv_release_cap`) so an action/macro
        //   deadline ends motion and NEVER suppresses a stop, hold, or freeze.
        let j7_effective_hold_or_freeze = j7_is_hold
            || !matches!(intent.kind, crate::driver::PosForceIntentKind::Target { .. })
            || intent.stamp.is_stale_against(current_stamp)
            || !drv.pos_force().pos_force_feedback_fresh();
        let action_cap = cutoff.and_then(|c| c.drv_action_cutoff);
        if j7_effective_hold_or_freeze {
            if let Some(c) = cutoff {
                if c.drv_action_cutoff.is_some() {
                    drv.set_tx_release_deadline(Some(c.drv_release_cap));
                }
            }
        } else if let Some(cap) = action_cap {
            let pre_now = drv.clock_now();
            if matches!(tick_output, TickOutput::ActionDeadlineRefused { .. }) || pre_now >= cap {
                drv.tx_mut().discard_pending_tx();
                *allow_output_accepted = false;
                if !matches!(tick_output, TickOutput::ActionDeadlineRefused { .. }) {
                    *tick_output = TickOutput::ActionDeadlineRefused {
                        drv_now: pre_now,
                        pf_status: crate::driver::PosForceTxStatus::Unobserved,
                    };
                }
                return None;
            }
        }
        let now = drv.clock_now();
        let max_age = drv.tx().tx_frame_max_age();
        let emit_res = drv.pos_force_mut().emit_pos_force_j7(intent, current_stamp);
        if !j7_effective_hold_or_freeze
            && matches!(
                emit_res,
                Ok(PosForceEmitOutcome::Deferred { .. }
                    | PosForceEmitOutcome::SentHoldInsteadOfTarget { .. })
                    | Err(ArmDriverError::Transport(
                        crate::transport::TransportError::Timeout
                    ))
            )
            && action_cap.is_some_and(|cap| drv.clock_now() >= cap)
        {
            let refused_now = drv.clock_now();
            let pf_status = match emit_res {
                Ok(
                    PosForceEmitOutcome::Deferred { pf_status, .. }
                    | PosForceEmitOutcome::SentHoldInsteadOfTarget { pf_status, .. },
                ) => pf_status,
                _ => drv.pos_force_mut().take_last_pos_force_tx_status(),
            };
            drv.tx_mut().discard_pending_tx();
            *allow_output_accepted = false;
            *tick_output = TickOutput::ActionDeadlineRefused {
                drv_now: refused_now,
                pf_status,
            };
            return None;
        }
        let mut hold_status_update: Option<(usize, JointHoldStatus)> = None;
        match emit_res {
            Ok(PosForceEmitOutcome::NotApplicable) => {}
            Ok(PosForceEmitOutcome::AwaitingFirstFresh { joint, pf_status }) => {
                hold_status_update = Some((joint, JointHoldStatus::UnheldNoFreshFeedback));
                match tick_output {
                    TickOutput::None => {
                        *tick_output = TickOutput::Complete {
                            is_hold: j7_is_hold,
                            now,
                            max_age,
                            due_mask: 0,
                            sent_mask: 0,
                            pf_status,
                        };
                    }
                    TickOutput::ActionDeadlineRefused { pf_status: dst_pf, .. }
                    | TickOutput::Complete { pf_status: dst_pf, .. }
                    | TickOutput::Deferred { pf_status: dst_pf, .. }
                    | TickOutput::Expired { pf_status: dst_pf, .. }
                    | TickOutput::Error { pf_status: dst_pf, .. } => {
                        *dst_pf = pf_status;
                    }
                    TickOutput::EstopInhibited { .. } | TickOutput::LeaseFenced => {}
                }
            }
            Ok(PosForceEmitOutcome::Sent { joint, pf_status, .. }) => {
                let status = if matches!(pf_status, crate::driver::PosForceTxStatus::Stale { .. }) {
                    JointHoldStatus::HeldStaleTarget
                } else {
                    JointHoldStatus::Held
                };
                hold_status_update = Some((joint, status));
                let pf_bit_hold = if joint.saturating_mul(2) < 64 {
                    1u64 << (joint * 2)
                } else {
                    0
                };
                let pf_bit_motion = if joint < 64 { 1u64 << joint } else { 0 };
                match tick_output {
                    TickOutput::None => {
                        *tick_output = TickOutput::Complete {
                            is_hold: j7_is_hold,
                            now,
                            max_age,
                            due_mask: if j7_is_hold { pf_bit_hold } else { 0 },
                            sent_mask: if j7_is_hold { pf_bit_hold } else { 0 },
                            pf_status,
                        };
                    }
                    TickOutput::ActionDeadlineRefused { pf_status: dst_pf, .. } => {
                        *dst_pf = pf_status;
                    }
                    TickOutput::Complete { is_hold, due_mask, sent_mask, pf_status: dst_pf, .. } => {
                        if *is_hold {
                            *due_mask |= pf_bit_hold;
                            *sent_mask |= pf_bit_hold;
                        }
                        *dst_pf = pf_status;
                    }
                    TickOutput::Deferred { is_hold, due_mask, sent_mask, pf_status: dst_pf, .. } => {
                        if *is_hold {
                            *due_mask |= pf_bit_hold;
                            *sent_mask |= pf_bit_hold;
                        } else {
                            *sent_mask |= pf_bit_motion;
                        }
                        *dst_pf = pf_status;
                    }
                    TickOutput::Expired { is_hold, due_mask, sent_mask, pf_status: dst_pf, .. } => {
                        if *is_hold {
                            *due_mask |= pf_bit_hold;
                            *sent_mask |= pf_bit_hold;
                        }
                        *dst_pf = pf_status;
                    }
                    TickOutput::Error { is_hold, sent_any, due_mask, sent_mask, pf_status: dst_pf, .. } => {
                        if *is_hold {
                            *sent_any = true;
                            *due_mask |= pf_bit_hold;
                            *sent_mask |= pf_bit_hold;
                        }
                        *dst_pf = pf_status;
                    }
                    TickOutput::EstopInhibited { .. } | TickOutput::LeaseFenced => {}
                }
            }
            Ok(PosForceEmitOutcome::SentHoldInsteadOfTarget { joint, pf_status, .. }) => {
                // Stationary continuity Hold reached the wire (keeping J7 held), while the
                // requested J7 motion Target was NOT transmitted: never count the requested
                // output as accepted; defer it to the next release.
                let status = if matches!(pf_status, crate::driver::PosForceTxStatus::Stale { .. }) {
                    JointHoldStatus::HeldStaleTarget
                } else {
                    JointHoldStatus::Held
                };
                hold_status_update = Some((joint, status));
                *allow_output_accepted = false;
                let pf_bit_hold = if joint.saturating_mul(2) < 64 {
                    1u64 << (joint * 2)
                } else {
                    0
                };
                match std::mem::replace(tick_output, TickOutput::None) {
                    TickOutput::None => {
                        *tick_output = TickOutput::Deferred {
                            pending: 1,
                            now,
                            max_age,
                            oldest_age: drv.tx().oldest_pending_tx_age(),
                            due_mask: if j7_is_hold { pf_bit_hold } else { 0 },
                            is_hold: j7_is_hold,
                            sent_mask: if j7_is_hold { pf_bit_hold } else { 0 },
                            pf_status,
                        };
                    }
                    TickOutput::ActionDeadlineRefused { drv_now, .. } => {
                        *tick_output = TickOutput::ActionDeadlineRefused { drv_now, pf_status };
                    }
                    TickOutput::Complete { is_hold, now, max_age, due_mask, sent_mask, .. } => {
                        *tick_output = TickOutput::Deferred {
                            pending: 1,
                            now,
                            max_age,
                            oldest_age: drv.tx().oldest_pending_tx_age(),
                            due_mask: if is_hold { due_mask | pf_bit_hold } else { 0 },
                            is_hold,
                            sent_mask: if is_hold {
                                sent_mask | pf_bit_hold
                            } else {
                                drv.tx().last_target_tx_sent_mask()
                            },
                            pf_status,
                        };
                    }
                    TickOutput::Deferred {
                        pending,
                        now,
                        max_age,
                        oldest_age,
                        due_mask,
                        is_hold,
                        sent_mask,
                        ..
                    } => {
                        *tick_output = TickOutput::Deferred {
                            pending: pending.saturating_add(1),
                            now,
                            max_age,
                            oldest_age: oldest_age.or_else(|| drv.tx().oldest_pending_tx_age()),
                            due_mask: if is_hold { due_mask | pf_bit_hold } else { due_mask },
                            is_hold,
                            sent_mask: if is_hold { sent_mask | pf_bit_hold } else { sent_mask },
                            pf_status,
                        };
                    }
                    TickOutput::Expired {
                        dropped,
                        pending,
                        now,
                        max_age,
                        oldest_age,
                        due_mask,
                        sent_mask,
                        is_hold,
                        ..
                    } => {
                        *tick_output = TickOutput::Expired {
                            dropped,
                            pending: pending.saturating_add(1),
                            now,
                            max_age,
                            oldest_age,
                            due_mask: if is_hold { due_mask | pf_bit_hold } else { due_mask },
                            sent_mask: if is_hold { sent_mask | pf_bit_hold } else { sent_mask },
                            is_hold,
                            pf_status,
                        };
                    }
                    TickOutput::Error {
                        message,
                        now,
                        max_age,
                        is_hold,
                        sent_any,
                        due_mask,
                        sent_mask,
                        ..
                    } => {
                        *tick_output = TickOutput::Error {
                            message,
                            now,
                            max_age,
                            is_hold,
                            sent_any: sent_any || is_hold,
                            due_mask: if is_hold { due_mask | pf_bit_hold } else { due_mask },
                            sent_mask: if is_hold { sent_mask | pf_bit_hold } else { sent_mask },
                            pf_status,
                        };
                    }
                    TickOutput::EstopInhibited { is_hold } => {
                        *tick_output = TickOutput::EstopInhibited { is_hold };
                    }
                    TickOutput::LeaseFenced => {
                        *tick_output = TickOutput::LeaseFenced;
                    }
                }
            }
            Ok(PosForceEmitOutcome::Deferred { joint, pf_status }) => {
                hold_status_update = Some((joint, JointHoldStatus::Queued));
                let pf_bit_hold = if joint.saturating_mul(2) < 64 {
                    1u64 << (joint * 2)
                } else {
                    0
                };
                match std::mem::replace(tick_output, TickOutput::None) {
                    TickOutput::None => {
                        *tick_output = TickOutput::Deferred {
                            pending: 1,
                            now,
                            max_age,
                            oldest_age: drv.tx().oldest_pending_tx_age(),
                            due_mask: if j7_is_hold { pf_bit_hold } else { 0 },
                            is_hold: j7_is_hold,
                            sent_mask: 0,
                            pf_status,
                        };
                    }
                    TickOutput::ActionDeadlineRefused { drv_now, .. } => {
                        *tick_output = TickOutput::ActionDeadlineRefused { drv_now, pf_status };
                    }
                    TickOutput::Complete { is_hold, now, max_age, due_mask, sent_mask, .. } => {
                        *allow_output_accepted = false;
                        *tick_output = TickOutput::Deferred {
                            pending: 1,
                            now,
                            max_age,
                            oldest_age: drv.tx().oldest_pending_tx_age(),
                            due_mask: if is_hold { due_mask | pf_bit_hold } else { 0 },
                            is_hold,
                            sent_mask: if is_hold { sent_mask } else { drv.tx().last_target_tx_sent_mask() },
                            pf_status,
                        };
                    }
                    TickOutput::Deferred {
                        pending,
                        now,
                        max_age,
                        oldest_age,
                        due_mask,
                        is_hold,
                        sent_mask,
                        ..
                    } => {
                        *tick_output = TickOutput::Deferred {
                            pending: pending.saturating_add(1),
                            now,
                            max_age,
                            oldest_age: oldest_age.or_else(|| drv.tx().oldest_pending_tx_age()),
                            due_mask: if is_hold { due_mask | pf_bit_hold } else { due_mask },
                            is_hold,
                            sent_mask,
                            pf_status,
                        };
                    }
                    TickOutput::Expired {
                        dropped,
                        pending,
                        now,
                        max_age,
                        oldest_age,
                        due_mask,
                        sent_mask,
                        is_hold,
                        ..
                    } => {
                        *tick_output = TickOutput::Expired {
                            dropped,
                            pending: pending.saturating_add(1),
                            now,
                            max_age,
                            oldest_age,
                            due_mask: if is_hold { due_mask | pf_bit_hold } else { due_mask },
                            sent_mask,
                            is_hold,
                            pf_status,
                        };
                    }
                    TickOutput::Error {
                        message,
                        now,
                        max_age,
                        is_hold,
                        sent_any,
                        due_mask,
                        sent_mask,
                        ..
                    } => {
                        *tick_output = TickOutput::Error {
                            message,
                            now,
                            max_age,
                            is_hold,
                            sent_any,
                            due_mask: if is_hold { due_mask | pf_bit_hold } else { due_mask },
                            sent_mask,
                            pf_status,
                        };
                    }
                    TickOutput::EstopInhibited { is_hold } => {
                        *tick_output = TickOutput::EstopInhibited { is_hold };
                    }
                    TickOutput::LeaseFenced => {
                        *tick_output = TickOutput::LeaseFenced;
                    }
                }
            }
            Err(ArmDriverError::EstopInhibited) => {
                if matches!(tick_output, TickOutput::None) {
                    *tick_output = TickOutput::EstopInhibited { is_hold: j7_is_hold };
                }
            }
            Err(ArmDriverError::LeaseFenced) => {
                if matches!(tick_output, TickOutput::None) {
                    *tick_output = TickOutput::LeaseFenced;
                }
            }
            Err(e) => {
                if let Some(c) = drv.pos_force().gripper_pos_force_info() {
                    hold_status_update = Some((c.joint, JointHoldStatus::UnheldTxFailed));
                }
                let pf_bit_hold = drv
                    .pos_force().gripper_pos_force_info()
                    .map_or(0, |c| if c.joint.saturating_mul(2) < 64 { 1u64 << (c.joint * 2) } else { 0 });
                let pf_status = drv.pos_force_mut().take_last_pos_force_tx_status();
                match tick_output {
                    TickOutput::ActionDeadlineRefused { pf_status: dst_pf, .. } => {
                        *dst_pf = pf_status;
                    }
                    TickOutput::Error { is_hold, due_mask, pf_status: dst_pf, .. }
                    | TickOutput::Expired { is_hold, due_mask, pf_status: dst_pf, .. } => {
                        if *is_hold {
                            *due_mask |= pf_bit_hold;
                        }
                        *dst_pf = pf_status;
                    }
                    TickOutput::Complete { is_hold, due_mask, sent_mask, .. }
                    | TickOutput::Deferred { is_hold, due_mask, sent_mask, .. } => {
                        let was_hold = *is_hold;
                        let prev_due = *due_mask;
                        let prev_sent = *sent_mask;
                        *allow_output_accepted = false;
                        let mut buf = crate::mit::OutputErrorText::empty();
                        e.write_bounded(&mut buf);
                        *tick_output = TickOutput::Error {
                            message: buf,
                            now,
                            max_age,
                            is_hold: was_hold || j7_is_hold,
                            sent_any: was_hold && prev_sent != 0,
                            due_mask: if was_hold || j7_is_hold { prev_due | pf_bit_hold } else { 0 },
                            sent_mask: if was_hold || j7_is_hold { prev_sent } else { 0 },
                            pf_status,
                        };
                    }
                    TickOutput::None => {
                        *allow_output_accepted = false;
                        let mut buf = crate::mit::OutputErrorText::empty();
                        e.write_bounded(&mut buf);
                        *tick_output = TickOutput::Error {
                            message: buf,
                            now,
                            max_age,
                            is_hold: j7_is_hold,
                            sent_any: false,
                            due_mask: if j7_is_hold { pf_bit_hold } else { 0 },
                            sent_mask: 0,
                            pf_status,
                        };
                    }
                    TickOutput::EstopInhibited { .. } | TickOutput::LeaseFenced => {}
                }
            }
        }
        hold_status_update
    }

    /// Tick-epilogue wrapper invoking [`Self::emit_and_merge_pos_force_j7`] directly on an
    /// exclusive RT driver (zero locks) or via non-blocking `try_lock()` on a legacy driver.
    fn emit_j7_tick_epilogue(
        &mut self,
        j7_is_hold: bool,
        intent: crate::driver::PosForceIntent,
        current_stamp: crate::driver::PosForceIntentStamp,
        tick_output: &mut TickOutput,
        allow_output_accepted: &mut bool,
    ) {
        let cutoff = self.active_tx_cutoff;
        let Some(d) = self.driver.as_mut() else {
            return;
        };
        #[cfg(not(target_arch = "wasm32"))]
        let shadow_rt = &mut self.torque_shadow;
        let Some(hold_update) = d.try_with_mut_recover(|drv| {
            let out = Self::emit_and_merge_pos_force_j7(
                drv,
                j7_is_hold,
                intent,
                current_stamp,
                cutoff,
                tick_output,
                allow_output_accepted,
            );
            #[cfg(not(target_arch = "wasm32"))]
            shadow_rt.drain_from_driver(drv);
            out
        }) else {
            return;
        };
        if j7_is_hold {
            if let Some((joint, status)) = hold_update {
                if joint < self.measured_hold.joint_status.len() {
                    self.measured_hold.joint_status[joint] = status;
                }
            }
        }
    }

    /// Single shared constructor converting any bounded driver motion or stopped-hold write
    /// result into a zero-allocation [`TickOutput`] carrying attempt-scoped J7 freshness status.
    pub(crate) fn tick_output_from_driver_write(
        drv: &mut dyn ArmDriver,
        is_hold: bool,
        write_res: Result<crate::driver::TxBatchOutcome, crate::driver::ArmDriverError>,
    ) -> TickOutput {
        use crate::driver::TxBatchOutcome;
        let now = drv.clock_now();
        let max_age = drv.tx().tx_frame_max_age();
        let hold_sent_any = is_hold && drv.tx().last_hold_tx_sent_any();
        let due_mask = if is_hold { drv.tx().last_hold_tx_due_mask() } else { 0 };
        let hold_sent_mask = if is_hold { drv.tx().last_hold_tx_sent_mask() } else { 0 };
        let pf_status = drv.pos_force_mut().take_last_pos_force_tx_status();
        match write_res {
            Ok(TxBatchOutcome::Complete) => TickOutput::Complete {
                is_hold,
                now,
                max_age,
                due_mask,
                sent_mask: hold_sent_mask,
                pf_status,
            },
            Ok(TxBatchOutcome::Deferred { pending }) => TickOutput::Deferred {
                pending,
                now,
                max_age,
                oldest_age: drv.tx().oldest_pending_tx_age(),
                due_mask,
                is_hold,
                sent_mask: if is_hold { hold_sent_mask } else { drv.tx().last_target_tx_sent_mask() },
                pf_status,
            },
            Ok(TxBatchOutcome::Expired { dropped, pending }) => TickOutput::Expired {
                dropped,
                pending,
                now,
                max_age,
                oldest_age: drv.tx().last_tx_expired_age(),
                due_mask,
                sent_mask: hold_sent_mask,
                is_hold,
                pf_status,
            },
            Err(crate::driver::ArmDriverError::EstopInhibited) => {
                TickOutput::EstopInhibited { is_hold }
            }
            Err(crate::driver::ArmDriverError::LeaseFenced) => TickOutput::LeaseFenced,
            Err(e) => {
                let mut buf = crate::mit::OutputErrorText::empty();
                e.write_bounded(&mut buf);
                TickOutput::Error {
                    message: buf,
                    now,
                    max_age,
                    is_hold,
                    sent_any: hold_sent_any,
                    due_mask,
                    sent_mask: hold_sent_mask,
                    pf_status,
                }
            }
        }
    }

    /// Zero-allocation / zero-free motion halt for an RT fault latch on `skip_joint` (e.g. J7
    /// pos-force staleness): retires any active trajectory off-thread, clears servo/park/stream
    /// motion state WITHOUT calling allocating `stop_motion()`, snaps healthy joints to measured
    /// positions, and snaps `skip_joint` to its last fresh freeze position (when available).
    fn halt_motion_for_fault_rt(&mut self, skip_joint: usize) {
        let freeze_pos = self.pos_force_j7_freeze_position();
        self.mit.park_after_hand_back = false;
        self.mit.deferred_park_action = None;
        self.release_servo_target();
        self.servo_settled_ticks.fill(0);
        self.clear_servo_ws_min_depths_rt();
        self.servo_stream_active = false;
        if let Some(old_traj) = self.active_trajectory.take() {
            self.retire_active_trajectory(old_traj);
        }
        self.pending_deferred_done = None;
        self.pending_normal_unload = false;
        self.clear_unload_settle();
        self.zero_check_persisting = None;
        self.gripper_stream_target = None;
        self.gripper_stream_contact = None;
        self.gripper_stream_lease = None;
        self.tracking.invalidate();
        self.commanded_velocities.fill(0.0);
        self.commanded_accelerations.fill(0.0);
        let dof = self.config.dof.min(self.commanded_positions.len()).min(self.measured_positions.len());
        for i in 0..dof {
            if i == skip_joint {
                if let Some(fp) = freeze_pos {
                    self.commanded_positions[i] = fp;
                }
            } else if self.measured_positions[i].is_finite() {
                self.commanded_positions[i] = self.measured_positions[i];
            }
        }
        copy_into(&mut self.target_positions, &self.commanded_positions);
    }

    /// Zero-allocation refusal for an in-flight planned motion (Cartesian path, replay, or
    /// quintic) whose next discrete tick increment fails static-load admission or restriction (M5).
    /// Keeps `commanded_positions` at the last safe reference, zeroes velocity, and fails the
    /// active action/macro with `STATIC_LOAD_LIMIT`.
    pub(crate) fn abort_planned_motion_static_load_rt(&mut self, action_id: Option<u64>) {
        if let Some(id) = action_id {
            if self
                .last_action
                .as_ref()
                .is_some_and(|a| a.id == id && a.is_running())
            {
                let mut msg = self.take_action_msg_buf(id, 192);
                msg.clear();
                let _ = core::fmt::Write::write_str(&mut msg, crate::power::STATIC_LOAD_LIMIT_MSG);
                self.finish_action(
                    id,
                    "failed",
                    Some("STATIC_LOAD_LIMIT"),
                    Some(msg),
                    None,
                    None,
                    None,
                );
            }
            if self.macro_run.as_ref().is_some_and(|m| m.id == id) {
                self.drop_macro_run("failed", Some("STATIC_LOAD_LIMIT"));
            }
        }
        self.replace_active_trajectory(None);
        self.pending_normal_unload = false;
        self.clear_unload_settle();
        self.quintic_settle = None;
        self.commanded_velocities.fill(0.0);
        copy_into(&mut self.target_positions, &self.commanded_positions);
        self.safety.mark_motion_finished();
        self.note_motion_limit(MotionLimitReason::Power, None);
    }

    /// Classify whether a `BusLoss` matches a synchronized multi-motor feedback drop (all
    /// previously reporting motors stale, with their last valid RX timestamps clustered within
    /// [`POWER_LOSS_SYNC_WINDOW_MS`]). Zero heap allocations, O(dof) timestamp comparisons.
    pub(crate) fn classify_bus_loss_detail(&self) -> Option<crate::safety::BusLossDetail> {
        let dof = self.config.dof.min(crate::model::MAX_MOTION_DOF);
        let mut reported = 0u8;
        let mut min_ts: Option<Instant> = None;
        let mut max_ts: Option<Instant> = None;
        for i in 0..dof {
            if let Some(ts) = self.last_valid_motor_rx[i] {
                reported = reported.saturating_add(1);
                min_ts = Some(min_ts.map_or(ts, |m| m.min(ts)));
                max_ts = Some(max_ts.map_or(ts, |m| m.max(ts)));
            }
        }
        if reported < 2 {
            return None;
        }
        let (min_ts, max_ts) = (min_ts?, max_ts?);
        let now = self.cached_driver_clock_now();
        // Every previously reporting motor must have gone stale (older than the 250 ms feedback
        // validity window), not merely experienced a single-axis silence or immediate read error
        // while other motors are still answering fresh frames.
        if now.saturating_duration_since(max_ts) < Duration::from_millis(250) {
            return None;
        }
        let spread = max_ts.saturating_duration_since(min_ts);
        if spread > Duration::from_millis(POWER_LOSS_SYNC_WINDOW_MS) {
            return None;
        }
        let last_power_a = self
            .power
            .last_valid_bus_a
            .or(self.power.last_valid_fast_a)
            .filter(|a| a.is_finite());
        Some(crate::safety::BusLossDetail {
            synchronized: true,
            reporting_motors: reported,
            spread_ms: spread.as_millis().min(u128::from(u64::MAX)) as u64,
            last_power_a,
        })
    }

    /// Notify the active driver that `BusLoss` latched so the next enable re-verifies `CTRL_MODE`
    /// and `TIMEOUT` causally across all motors. Non-blocking: defers via
    /// `pending_driver_bus_loss_notify` if the driver slot is currently contended.
    pub(crate) fn notify_driver_bus_loss(&mut self) {
        let notified = self
            .driver
            .as_mut()
            .and_then(|d| d.try_with_mut(|drv| drv.bus_recovery_mut().notify_bus_loss()).ok())
            .is_some();
        if !notified && self.driver.is_some() {
            self.pending_driver_bus_loss_notify = true;
        }
    }

    /// Single owner-side J7 position-force staleness handler shared by BOTH motion and stopped-hold
    /// paths:
    /// - `Fresh`: resets the staleness timer.
    /// - `Unobserved`: leaves state unchanged.
    /// - `Stale { joint, age, fresh_gen }`: immediately terminates any running gripper action with
    ///   `"J7_FEEDBACK_STALE"` (J7 gets the `vel_raw = 0` freeze frame at the last fresh measured
    ///   position plus a `0xCC` refresh while J1–J6 continue), and advances the continuous
    ///   staleness duration on the driver clock; at or above `damiao_gripper.feedback_stale_fault_ms`
    ///   (default 250 ms, 200..=450 ms), halts motion zero-alloc and latches recoverable
    ///   [`FaultKind::J7FeedbackStale`] once per fault episode.
    fn handle_j7_feedback_staleness(
        &mut self,
        pf_status: crate::driver::PosForceTxStatus,
        now: Instant,
    ) {
        use crate::driver::PosForceTxStatus;
        match pf_status {
            PosForceTxStatus::Unobserved => {}
            PosForceTxStatus::Fresh => {
                self.j7_stale_since = None;
            }
            PosForceTxStatus::Stale { joint, age, fresh_gen } => {
                self.fail_running_gripper_for_j7_stale_rt(joint, None);
                let since = match self.j7_stale_since {
                    Some((gen, t)) if gen == fresh_gen => t,
                    _ => {
                        self.j7_stale_since = Some((fresh_gen, now));
                        now
                    }
                };
                let elapsed = now.saturating_duration_since(since);
                let effective_age = age.map_or(elapsed, |a| a.max(elapsed));
                let limit_ms = self.config.resolved_j7_stale_fault_ms();
                if effective_age >= Duration::from_millis(limit_ms)
                    && !self.safety.has_latched_fault_kind(crate::safety::FaultKind::J7FeedbackStale)
                {
                    self.halt_motion_for_fault_rt(joint);
                    let detail = crate::safety::J7StaleDetail {
                        joint: joint.min(u8::MAX as usize) as u8,
                        age_ms: Some(effective_age.as_millis().min(u128::from(u64::MAX)) as u64),
                        limit_ms,
                    };
                    self.safety.latch(
                        crate::safety::FaultKind::J7FeedbackStale,
                        crate::safety::FaultDetail::J7FeedbackStale(Some(detail)),
                    );
                }
            }
        }
    }

    /// Classify one release's (or immediate write's) bounded TX outcome. Returns `true` iff the
    /// submitted batch completed (`TickOutput::Complete`).
    /// - `count_release_streak`: `true` ONLY for periodic control releases (`tick_control`),
    ///   `false` for off-tick immediate writes (`gravity_comp_write_now`) so off-tick calls
    ///   never double-count against `tx_stall_ticks`.
    /// - `has_pending_motion`: when `output` is `TickOutput::None` during a periodic release,
    ///   preserves an active deferral streak ONLY while a genuinely pending motion batch remains
    ///   queued in the driver; otherwise resets the streak cleanly.
    pub(crate) fn classify_tick_output(
        &mut self,
        output: TickOutput,
        count_release_streak: bool,
        has_pending_motion: bool,
    ) -> bool {
        match output {
            TickOutput::None | TickOutput::LeaseFenced => {
                self.reset_hold_tx_joint_progress();
                if !self.is_torqued
                    || self.torque_intent == TorqueIntent::Off
                    || self.shutdown_committed
                {
                    self.reset_tx_joint_progress();
                    self.j7_stale_since = None;
                } else if count_release_streak && !has_pending_motion {
                    self.reset_tx_joint_progress();
                }
                false
            }
            TickOutput::EstopInhibited { is_hold } => {
                self.reset_tx_joint_progress();
                self.reset_hold_tx_joint_progress();
                if is_hold || self.safety.state().is_stopped() {
                    self.tracking.invalidate();
                    self.commanded_velocities.fill(0.0);
                    if self.measured_positions.iter().all(|q| q.is_finite()) {
                        copy_into(&mut self.commanded_positions, &self.measured_positions);
                        copy_into(&mut self.target_positions, &self.measured_positions);
                    }
                } else {
                    self.stop_motion();
                    self.tracking.invalidate();
                    self.commanded_velocities.fill(0.0);
                    if self.measured_positions.iter().all(|q| q.is_finite()) {
                        copy_into(&mut self.commanded_positions, &self.measured_positions);
                        copy_into(&mut self.target_positions, &self.measured_positions);
                    }
                    if !self.safety.has_latched_fault_code(FaultCode::DriverWriteFailed) {
                        self.safety.latch(
                            crate::safety::FaultKind::Other,
                            crate::safety::FaultDetail::formatted(FaultCode::DriverWriteFailed, |buf| {
                                buf.push_str("Control output failed: ");
                                buf.push_str(crate::driver::ESTOP_INHIBITED);
                            }),
                        );
                    }
                    if !self.safety.deferred_logging() {
                        tracing::error!("Control output failed: {}", crate::driver::ESTOP_INHIBITED);
                    }
                }
                false
            }
            TickOutput::ActionDeadlineRefused { drv_now, pf_status } => {
                // TX was refused by the active action/macro deadline, not a CAN bus stall:
                // clear any bus-stall streak and leave `batch_completed = false` so the tick's
                // post-output reconciliation terminates the expired action cleanly, while still
                // accounting any J7 staleness observed by a stop/hold/freeze epilogue frame.
                self.reset_tx_joint_progress();
                self.handle_j7_feedback_staleness(pf_status, drv_now);
                false
            }
            TickOutput::Complete { is_hold, now, max_age, due_mask, sent_mask, pf_status } => {
                self.reset_tx_joint_progress();
                if is_hold || self.safety.state().is_stopped() {
                    self.account_stopped_hold_progress(now, max_age, due_mask, sent_mask, 0, 0, true);
                } else {
                    self.reset_hold_tx_joint_progress();
                }
                self.handle_j7_feedback_staleness(pf_status, now);
                true
            }
            TickOutput::Deferred {
                pending,
                now,
                max_age,
                oldest_age,
                due_mask,
                is_hold,
                sent_mask,
                pf_status,
            } => {
                // Classify using the pass's original motion vs stopped-hold context BEFORE
                // `handle_j7_feedback_staleness` so a same-release J7 threshold crossing can never
                // mask a motion TX stall; both conditions record through `SafetyManager::latch`.
                let treat_as_hold = is_hold || self.safety.state().is_stopped();
                if treat_as_hold {
                    self.account_stopped_hold_progress(now, max_age, due_mask, sent_mask, pending, 0, true);
                    self.handle_j7_feedback_staleness(pf_status, now);
                    return false;
                }
                self.reset_hold_tx_joint_progress();

                if count_release_streak {
                    // Track per-motor lack of progress: joints that completed a target TX in this pass
                    // reset their streak, while unsent joints advance their own streak & age. Healthy
                    // partial passes where carry-over frames drain every 1-2 ticks never false-fault.
                    let n = self
                        .config
                        .dof
                        .min(self.tx_joint_no_progress_ticks.len())
                        .min(self.tx_joint_no_progress_since.len())
                        .min(64);
                    let mut max_joint_ticks = 0u32;
                    let mut max_joint_age = Duration::ZERO;
                    for j in 0..n {
                        if (sent_mask & (1u64 << j)) != 0 {
                            self.tx_joint_no_progress_ticks[j] = 0;
                            self.tx_joint_no_progress_since[j] = None;
                        } else {
                            let ticks = self.tx_joint_no_progress_ticks[j].saturating_add(1);
                            self.tx_joint_no_progress_ticks[j] = ticks;
                            let since = *self.tx_joint_no_progress_since[j].get_or_insert(now);
                            let age = now.saturating_duration_since(since);
                            if ticks > max_joint_ticks {
                                max_joint_ticks = ticks;
                            }
                            if age > max_joint_age {
                                max_joint_age = age;
                            }
                        }
                    }
                    let observed_age = oldest_age.map_or(max_joint_age, |a| a.max(max_joint_age));
                    let max_ticks = self.config.resolved_tx_stall_ticks();
                    if max_joint_ticks >= max_ticks || observed_age >= max_age {
                        let detail = crate::safety::TxStallDetail {
                            queued: pending,
                            dropped: 0,
                            age_ms: observed_age.as_millis().min(u128::from(u64::MAX)) as u64,
                            limit_ms: max_age.as_millis().min(u128::from(u64::MAX)) as u64,
                            consecutive_ticks: max_joint_ticks,
                            is_hold: false,
                        };
                        self.latch_can_tx_stalled_motion(detail, now);
                    }
                }
                self.handle_j7_feedback_staleness(pf_status, now);
                false
            }
            TickOutput::Expired {
                dropped,
                pending,
                now,
                max_age,
                oldest_age,
                due_mask,
                sent_mask,
                is_hold,
                pf_status,
            } => {
                // Classify using the pass's original motion vs stopped-hold context BEFORE
                // `handle_j7_feedback_staleness` so a same-release J7 threshold crossing can never
                // mask a non-recoverable motion `CAN_TX_STALLED` latch.
                let treat_as_hold = is_hold || self.safety.state().is_stopped();
                if treat_as_hold {
                    self.account_stopped_hold_progress(now, max_age, due_mask, sent_mask, pending, dropped, true);
                    self.handle_j7_feedback_staleness(pf_status, now);
                    return false;
                }
                self.reset_hold_tx_joint_progress();

                let age = oldest_age
                    .or_else(|| self.max_tx_joint_no_progress_age(now))
                    .unwrap_or(max_age)
                    .max(max_age);
                let detail = crate::safety::TxStallDetail {
                    queued: pending,
                    dropped,
                    age_ms: age.as_millis().min(u128::from(u64::MAX)) as u64,
                    limit_ms: max_age.as_millis().min(u128::from(u64::MAX)) as u64,
                    consecutive_ticks: self.max_tx_joint_no_progress_ticks().saturating_add(1),
                    is_hold: false,
                };
                self.latch_can_tx_stalled_motion(detail, now);
                self.handle_j7_feedback_staleness(pf_status, now);
                false
            }
            TickOutput::Error { message, now, max_age, is_hold, sent_any, due_mask, sent_mask, pf_status } => {
                // Classify using the pass's original motion vs stopped-hold context BEFORE
                // `handle_j7_feedback_staleness` so recoverable J7 staleness can never mask a
                // non-recoverable motion `DRIVER_WRITE_FAILED` latch.
                let treat_as_hold = is_hold || self.safety.state().is_stopped();
                if treat_as_hold {
                    self.reset_tx_joint_progress();
                    self.tracking.invalidate();
                    self.commanded_velocities.fill(0.0);
                    if self.measured_positions.iter().all(|q| q.is_finite()) {
                        copy_into(&mut self.commanded_positions, &self.measured_positions);
                        copy_into(&mut self.target_positions, &self.measured_positions);
                    }
                    // Rate-limit `"Stopped hold output failed"` (<= 1/s with exact suppressed
                    // counter) AND account per-motor lack of progress toward `CAN_TX_STALLED`.
                    self.log_stopped_hold_output_failed_throttled(now, sent_any, message.as_str());
                    self.account_stopped_hold_progress(now, max_age, due_mask, sent_mask, 0, 0, false);
                } else {
                    self.reset_tx_joint_progress();
                    self.reset_hold_tx_joint_progress();
                    self.stop_motion();
                    self.tracking.invalidate();
                    self.commanded_velocities.fill(0.0);
                    if self.measured_positions.iter().all(|q| q.is_finite()) {
                        copy_into(&mut self.commanded_positions, &self.measured_positions);
                        copy_into(&mut self.target_positions, &self.measured_positions);
                    }
                    if !self.safety.has_latched_fault_code(FaultCode::DriverWriteFailed) {
                        self.safety.latch(
                            crate::safety::FaultKind::Other,
                            crate::safety::FaultDetail::formatted(FaultCode::DriverWriteFailed, |buf| {
                                buf.push_str("Control output failed: ");
                                buf.push_str(message.as_str());
                            }),
                        );
                    }
                    if !self.safety.deferred_logging() {
                        tracing::error!("Control output failed: {message}");
                    }
                }
                self.handle_j7_feedback_staleness(pf_status, now);
                false
            }
        }
    }

    /// Snapshot of owner-side live disconnect fence & E-stop state borrowed across a driver closure.
    pub(crate) fn live_disconnect_fence_view(&self) -> LiveDisconnectFenceView<'_> {
        LiveDisconnectFenceView::new(
            &self.e_stop_latched,
            &self.pending_disconnect_hold,
            &self.atomic_fenced_lease_gen,
            self.fenced_lease_generation,
            self.active_lease_conn_id,
            self.active_lease.as_ref().map(|a| a.generation),
            &self.closed_connections,
            self.rt_closed_connections.as_deref(),
            &self.shared_closed_bitset,
        )
    }

    /// Shared gravity-comp target write helper used by both the periodic tick path and the
    /// immediate write path. Samples the live E-stop latch and live disconnect fence AFTER
    /// acquiring the driver, reading `enable_state()`, AND staging `output_lease_gen` (final
    /// normal-write boundary), switching to `write_estop_hold_bounded` (with lease gen `0`) if
    /// E-stop latched or suppressing non-hold output on disconnect, and returns `(output, effective_hold)`
    /// so callers align J7 intent and output acceptance with the actual write path.
    pub(crate) fn write_gravity_comp_targets_checked(
        drv: &mut dyn crate::driver::ArmDriver,
        targets: &[crate::driver::JointTarget],
        is_hold: bool,
        output_lease_gen: u64,
        fence: &LiveDisconnectFenceView<'_>,
    ) -> Option<(TickOutput, bool)> {
        if drv.enable_state() == EnableState::Disabled {
            return None;
        }
        let mut effective_hold = is_hold || fence.estop_latched();
        let write_res = if effective_hold {
            drv.tx_mut().set_output_lease_gen(0);
            drv.write_estop_hold_bounded(targets)
        } else {
            drv.tx_mut().set_output_lease_gen(output_lease_gen);
            if fence.estop_latched() {
                drv.tx_mut().set_output_lease_gen(0);
                effective_hold = true;
                drv.write_estop_hold_bounded(targets)
            } else if fence.blocks_non_hold() {
                drv.tx_mut().set_output_lease_gen(0);
                return None;
            } else {
                let r = drv.write_joint_targets_bounded(targets);
                drv.tx_mut().set_output_lease_gen(0);
                r
            }
        };
        Some((
            Self::tick_output_from_driver_write(drv, effective_hold, write_res),
            effective_hold,
        ))
    }

    /// Runs `f(&SupervisorInner, &mut dyn ArmDriver)` with a single implementation across both
    /// `DriverSlot::Legacy` (holding the `DriverLock` continuously across `f`) and
    /// `DriverSlot::Exclusive` (direct mutable access with panic-safe slot restoration).
    pub(crate) fn with_driver_and_owner_ref<R>(
        &mut self,
        f: impl FnOnce(&SupervisorInner, &mut dyn crate::driver::ArmDriver) -> R,
    ) -> Option<R> {
        if let Some(lock) = self.driver.as_ref().and_then(|d| d.as_legacy_lock()) {
            let mut drv = lock.lock().ok()?;
            return Some(f(self, &mut **drv));
        }
        let mut slot = self.driver.take()?;
        let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            slot.as_exclusive_mut().map(|drv| f(self, drv))
        }));
        self.driver = Some(slot);
        match res {
            Ok(out) => out,
            Err(payload) => std::panic::resume_unwind(payload),
        }
    }

    /// Publication margin reserved at the end of a control release when bounding per-tick CAN TX
    /// (`DAMIAO_TX_RELEASE_PUBLISH_MARGIN`, capped to `period / 2` for sub-4 ms control periods).
    pub(crate) fn tx_release_publish_margin(&self) -> Duration {
        let period = Duration::from_secs_f64(1.0 / f64::from(self.config.control_frequency_hz.max(1)));
        crate::driver::DAMIAO_TX_RELEASE_PUBLISH_MARGIN.min(period / 2)
    }

    /// Installs `driver.set_tx_release_deadline(Some(cap))` using conservative driver-clock-first,
    /// live-owner-clock-second translation, refusing expired cutoffs BEFORE staging any deadline.
    pub(crate) fn install_driver_tx_release_cap_with_clock(
        &mut self,
        release_deadline: Instant,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> DriverTxCapOutcome {
        let margin = self.tx_release_publish_margin();
        let Some(owner_cap) = release_deadline.checked_sub(margin) else {
            return DriverTxCapOutcome::Expired;
        };
        let Some(d) = self.driver.as_mut() else {
            return DriverTxCapOutcome::NotInstalled;
        };
        d.try_with_mut(|driver| {
            match translate_driver_cutoff_conservative(driver, owner_cap, now_fn) {
                Ok((_, _, cap)) => {
                    driver.set_tx_release_deadline(Some(cap));
                    DriverTxCapOutcome::Installed
                }
                Err(_) => DriverTxCapOutcome::Expired,
            }
        })
        .unwrap_or(DriverTxCapOutcome::NotInstalled)
    }

    /// Clears any active per-release TX deadline cap on the attached driver.
    pub(crate) fn clear_driver_tx_release_cap(&mut self) {
        if let Some(d) = self.driver.as_mut() {
            let _ = d.try_with_mut(|driver| driver.set_tx_release_deadline(None));
        }
    }

    /// One control tick and its observation ([`Self::tick_control`], then
    /// [`Self::current_observation`] at the tick's timestamp).
    pub fn tick(&mut self, dt: f64) -> ArmObservation {
        let timestamp = self.tick_control(dt);
        let mut obs = ArmObservation::blank();
        self.fill_observation(timestamp, &mut obs);
        obs
    }

    /// [`Self::tick`] bounded by an explicit control release deadline (`now`, `release_deadline`)
    /// so TX start-cutoffs never run past `release_deadline - publish_margin`.
    pub fn tick_with_release_deadline(
        &mut self,
        dt: f64,
        now: Instant,
        release_deadline: Instant,
    ) -> ArmObservation {
        let timestamp = self.tick_control_with_deadline(dt, now, release_deadline);
        let mut obs = ArmObservation::blank();
        self.fill_observation(timestamp, &mut obs);
        obs
    }

    /// Effective per-release budgeted TX attempt cap used to initialize [`crate::rt_core::budget::ReleaseBudget`].
    pub(crate) fn release_tx_budget_limit(&self) -> u16 {
        let cfg_max = self
            .config
            .damiao_tx_budget()
            .max_frames
            .min(u16::MAX as usize) as u16;
        self.driver
            .as_ref()
            .and_then(|d| {
                d.try_with_ref_recover(|drv| drv.tx().tx_budget_max_frames())
                    .flatten()
            })
            .unwrap_or(cfg_max)
    }

    /// Snapshots the driver's current `(rx_frames, tx_attempts)` consumed in this control release
    /// and monotonically tightens the driver's remaining RX/TX caps from `budget`.
    ///
    /// Returns `Some((0, 0))` when no driver is attached, `Some(before)` when the snapshot and
    /// cap tightening succeeded, and `None` if the driver lock was contended/unavailable (callers
    /// must then run under the most conservative cap and `charge_driver_budget_after` must not
    /// post-charge cumulative driver counters).
    #[inline]
    pub(crate) fn sync_driver_budget_before(
        &mut self,
        budget: &crate::rt_core::budget::ReleaseBudget,
    ) -> Option<(u16, u16)> {
        let Some(d) = self.driver.as_mut() else {
            return Some((0, 0));
        };
        let rx_rem = budget.rx_remaining();
        let tx_rem = budget.tx_remaining();
        let apply = |drv: &mut dyn ArmDriver| {
            let before = drv.tx().release_budget_consumed();
            drv.tx_mut().sync_release_budget_limits(rx_rem, tx_rem);
            before
        };
        d.try_with_mut(apply).ok()
    }

    /// Charges the driver's newly consumed `(rx_frames, tx_attempts)` since `before` into `budget`.
    ///
    /// When `before` is `None` (pre-snapshot failed under contention), cumulative driver counters
    /// are NEVER read or charged so earlier-in-release usage cannot be double-charged; only the
    /// explicit incremental `(reported_rx, rx_exhausted)` and `reported_tx` from this step apply.
    #[inline]
    pub(crate) fn charge_driver_budget_after(
        &self,
        budget: &mut crate::rt_core::budget::ReleaseBudget,
        before: Option<(u16, u16)>,
        reported_rx: u16,
        rx_exhausted: bool,
        reported_tx: u16,
    ) {
        let (rx_delta, tx_delta) = match before {
            Some(before) => {
                let after = self
                    .driver
                    .as_ref()
                    .and_then(|d| d.try_with_ref_recover(|drv| drv.tx().release_budget_consumed()))
                    .unwrap_or(before);
                (
                    after.0.saturating_sub(before.0).max(reported_rx),
                    after.1.saturating_sub(before.1).max(reported_tx),
                )
            }
            None => (reported_rx, reported_tx),
        };
        if rx_delta > 0 || rx_exhausted {
            budget.charge_rx(rx_delta, rx_exhausted);
        }
        if tx_delta > 0 {
            budget.charge_tx(tx_delta);
        }
    }

    /// Arms conservative in-borrow `(rx=0, tx=0)` cap installation and incremental accounting when
    /// a pre-snapshot failed under contention (`before.is_none()`).
    #[inline]
    pub(crate) fn arm_conservative_cap_if_contended(&mut self, before: Option<(u16, u16)>) {
        self.pending_conservative_tx_cap = before.is_none();
        self.conservative_step_rx_delta = 0;
        self.conservative_step_tx_delta = 0;
    }

    /// Disarms conservative in-borrow capping and returns the incremental `(rx, tx)` deltas
    /// accumulated during any driver borrow that ran after a contended pre-snapshot.
    #[inline]
    pub(crate) fn take_conservative_step_deltas(&mut self) -> (u16, u16) {
        self.pending_conservative_tx_cap = false;
        let out = (
            self.conservative_step_rx_delta,
            self.conservative_step_tx_delta,
        );
        self.conservative_step_rx_delta = 0;
        self.conservative_step_tx_delta = 0;
        out
    }

    /// Shared Step-2 hardware poll implementation for [`crate::rt_core::ops::CoreOps::poll_hardware`]
    /// and [`crate::rt_core::ops::CoreOps::poll_hardware_for_release`].
    pub(crate) fn poll_hardware_with_release_deadline(
        &mut self,
        budget: &crate::rt_core::ops::ServiceBudget,
        release_deadline: Option<Instant>,
    ) -> crate::rt_core::ops::HardwarePoll {
        let entry_wall = Instant::now();
        let mut live_now = || budget.now + Instant::now().saturating_duration_since(entry_wall);
        self.poll_hardware_with_release_deadline_with_clock(budget, release_deadline, &mut live_now)
    }

    pub(crate) fn poll_hardware_with_release_deadline_with_clock(
        &mut self,
        budget: &crate::rt_core::ops::ServiceBudget,
        release_deadline: Option<Instant>,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> crate::rt_core::ops::HardwarePoll {
        let mut ledger = crate::rt_core::budget::ReleaseBudget::new(
            self.control_release_seq.wrapping_add(1).max(1),
            budget.now,
            release_deadline.unwrap_or(budget.deadline),
            budget.max_rx_frames,
            self.release_tx_budget_limit(),
        );
        self.poll_hardware_with_budget_inner(&mut ledger, Some(budget.deadline), release_deadline.is_some(), now_fn)
    }

    /// Ledger-threaded Step-2 hardware poll owned by [`crate::rt_core::ops::ReleaseEngine`].
    pub(crate) fn poll_hardware_with_budget_and_clock(
        &mut self,
        budget: &mut crate::rt_core::budget::ReleaseBudget,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> crate::rt_core::ops::HardwarePoll {
        self.poll_hardware_with_budget_inner(budget, None, true, now_fn)
    }

    fn poll_hardware_with_budget_inner(
        &mut self,
        budget: &mut crate::rt_core::budget::ReleaseBudget,
        rx_deadline_override: Option<Instant>,
        install_tx_cap: bool,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> crate::rt_core::ops::HardwarePoll {
        self.control_release_seq = self.control_release_seq.wrapping_add(1).max(1);
        self.control_release_seq_atomic
            .store(self.control_release_seq, Ordering::Release);
        // Notify the driver of the new control release BEFORE transition polling so Step 2
        // transition polls, Step 2 RX recovery/refresh, Step 3 commits, and Step 5 output all
        // share a single per-release TX budget and per-motor control-frame coalescing mask.
        if let Some(d) = self.driver.as_mut() {
            let seq = self.control_release_seq;
            let _ = d.try_with_mut(|drv| drv.begin_control_release(seq));
        }
        #[cfg(not(target_arch = "wasm32"))]
        self.torque_shadow
            .enqueue_at(
                now_fn(),
                crate::torque::shadow::ShadowInputKind::Release {
                    release_seq: self.control_release_seq,
                    live_intent: self.shadow_live_intent(),
                    live_is_torqued: self.is_torqued,
                    live_stopped: self.safety.state().is_stopped(),
                },
            );
        let was_thermal_active = self.thermal_protection_active();
        // Install the release publish-margin TX cap BEFORE enable-transition polling, background
        // disable polling, and RX feedback intake so every TX in Step 2 obeys the same cutoff.
        // If the release cutoff has already expired, stage ZERO driver deadline and skip budgeted
        // Step-2 TX/poll work cleanly (`budget_exhausted = true`).
        let cap_outcome = if install_tx_cap {
            self.install_driver_tx_release_cap_with_clock(budget.deadline(), now_fn)
        } else {
            DriverTxCapOutcome::NotInstalled
        };
        let applied_step2_tx_cap = cap_outcome == DriverTxCapOutcome::Installed;
        let release_cap_expired = cap_outcome == DriverTxCapOutcome::Expired;
        let mut park_wake = None;
        let mut park_state_changed = false;
        let make_sb = |b: &crate::rt_core::budget::ReleaseBudget, now: Instant| {
            let mut sb = b.poll_service_budget(now);
            if let Some(dl) = rx_deadline_override {
                sb.deadline = sb.deadline.min(dl);
            }
            sb
        };
        let step_now = now_fn();
        let (rx_frames, rx_exhausted) = if release_cap_expired {
            // Release TX cutoff already expired: skip budgeted Step-2 TX transitions without
            // staging an expired driver deadline, and still read RX feedback so `bus_feedback_fresh`
            // remains accurate.
            let before_rx = self.sync_driver_budget_before(budget);
            let mut sb = make_sb(budget, step_now);
            if before_rx.is_none() {
                sb.max_rx_frames = 0;
            }
            let mut rx = self.poll_hardware_rx_with_clock(Some(&sb), None, None, now_fn);
            rx.budget_exhausted = true;
            self.charge_driver_budget_after(budget, before_rx, rx.rx_frames, true, 0);
            rx.rx_frames = budget.rx_used();
            let rx_frames = rx.rx_frames;
            self.pending_rx_poll = rx;
            (rx_frames, true)
        } else {
            let mut sb_t = make_sb(budget, step_now);
            let rx_zero_before_t = sb_t.max_rx_frames == 0;
            if rx_zero_before_t {
                budget.mark_rx_exhausted();
            }
            let transition_rx = if sb_t.now > sb_t.deadline {
                if self.torque_transition.is_some() || self.disable_pending {
                    Some((0, true))
                } else {
                    None
                }
            } else {
                // Zero remaining RX skips reading inside the driver (`max_rx_frames == 0`),
                // but MUST still run TX-capable transition work (inline watchdog holds and
                // protective disable retries).
                let before_t = self.sync_driver_budget_before(budget);
                if before_t.is_none() {
                    sb_t.max_rx_frames = 0;
                }
                self.arm_conservative_cap_if_contended(before_t);
                let trans = self.service_torque_transition_with_clock(&sb_t, now_fn);
                let t_rx = if let Some(tp) = trans {
                    if tp.broker_owned {
                        self.core.begin_transition(tp.progress);
                    } else {
                        match tp.progress {
                            Ok(crate::rt_core::ops::TorqueProgress::Pending { wake }) => park_wake = Some(wake),
                            Ok(crate::rt_core::ops::TorqueProgress::Done) | Err(_) => park_state_changed = true,
                        }
                    }
                    tp.rx
                } else {
                    self.service_background_disable_with_clock(&sb_t, now_fn)
                };
                let (cons_rx, cons_tx) = self.take_conservative_step_deltas();
                if let Some((tf, tex)) = t_rx {
                    self.charge_driver_budget_after(
                        budget,
                        before_t,
                        tf.max(cons_rx),
                        tex || rx_zero_before_t,
                        cons_tx,
                    );
                } else {
                    self.charge_driver_budget_after(
                        budget,
                        before_t,
                        cons_rx,
                        rx_zero_before_t,
                        cons_tx,
                    );
                }
                if rx_zero_before_t && (self.torque_transition.is_some() || self.disable_pending || t_rx.is_some()) {
                    Some((t_rx.map_or(0, |(f, _)| f), true))
                } else {
                    t_rx
                }
            };
            let before_rx = self.sync_driver_budget_before(budget);
            let mut sb_rx = make_sb(budget, now_fn());
            if before_rx.is_none() {
                sb_rx.max_rx_frames = 0;
            }
            // `sb_rx.max_rx_frames` already reflects `budget.rx_remaining()` after Phase 2a,
            // so pass `(0, exhausted)` to avoid double-subtracting transition RX frames.
            let rx_marker = transition_rx.map(|(_, ex)| (0u16, ex));
            let mut rx = self.poll_hardware_rx_with_clock(Some(&sb_rx), rx_marker, None, now_fn);
            if applied_step2_tx_cap {
                self.clear_driver_tx_release_cap();
            }
            self.charge_driver_budget_after(budget, before_rx, rx.rx_frames, rx.budget_exhausted, 0);
            rx.rx_frames = budget.rx_used();
            rx.budget_exhausted |= budget.rx_exhausted();
            let rx_frames = rx.rx_frames;
            let rx_exhausted = rx.budget_exhausted;
            self.pending_rx_poll = rx;
            (rx_frames, rx_exhausted)
        };
        // Run thermal protection BEFORE consuming background disable confirmation and before
        // Step 2b maintenance (`ClearFault`/`RequestShutdown`) or Step 3 command admission so an
        // over-temperature sample arriving on the same poll as confirmed disable or enabling
        // cannot suppress `ThermalTrip`.
        self.check_thermal_protection_inner(was_thermal_active, true);
        let _ = self.confirm_background_disable_if_ready();
        let budget_exhausted =
            release_cap_expired || budget.deadline_exceeded(step_now) || rx_exhausted || budget.rx_exhausted();
        let Some(driver) = self.driver.as_ref() else {
            self.rt_j7_pf_gate = None;
            return crate::rt_core::ops::HardwarePoll {
                driverless: true,
                rx_frames,
                budget_exhausted,
                torque: self.core.poll_transition(),
                park_wake,
                park_state_changed,
                ..crate::rt_core::ops::HardwarePoll::default()
            };
        };
        let is_pf = self
            .config
            .damiao_gripper
            .as_ref()
            .is_some_and(|g| g.mode == crate::model::DamiaoGripperMode::PosForce);
        let (poll, pf_gate) = driver
            .try_with_ref_recover(|d| {
                d.feedback_ages_into(&mut self.scratch.ages);
                let oldest = self
                    .scratch
                    .ages
                    .iter()
                    .copied()
                    .try_fold(Duration::ZERO, |m, a| a.map(|a| m.max(a)));
                let gate = if is_pf {
                    let stale = !d.pos_force().pos_force_feedback_fresh();
                    let freeze_pos = self.config.gripper_index().and_then(|g| {
                        let raw = d
                            .pos_force().pos_force_last_fresh_position()
                            .or_else(|| self.measured_positions.get(g).copied().filter(|p| p.is_finite()))
                            .or_else(|| self.commanded_positions.get(g).copied().filter(|p| p.is_finite()))?;
                        Some(
                            self.config
                                .limits
                                .get(g)
                                .map_or(raw, |l| raw.clamp(l.min_position, l.max_position)),
                        )
                    });
                    J7PosForceGate { stale, freeze_pos }
                } else {
                    J7PosForceGate::default()
                };
                let torque_poll = self.core.poll_transition();
                let urgent_wake = if torque_poll.is_some() || park_wake.is_some() {
                    let owner_now = now_fn();
                    let drv_now = d.clock_now();
                    d.tx().next_watchdog_hold_wake()
                        .map(|w| owner_now + w.saturating_duration_since(drv_now))
                } else {
                    None
                };
                let poll = crate::rt_core::ops::HardwarePoll {
                    driverless: false,
                    feedback_valid: d.is_feedback_valid(),
                    enable_state: Some(d.enable_state()),
                    oldest_feedback_age: oldest,
                    rx_frames,
                    budget_exhausted,
                    torque: torque_poll,
                    park_wake,
                    park_state_changed,
                    urgent_wake,
                };
                (poll, gate)
            })
            .unwrap_or_else(|| {
                let gate = if is_pf {
                    let freeze_pos = self.config.gripper_index().and_then(|g| {
                        let raw = self
                            .measured_positions
                            .get(g)
                            .copied()
                            .filter(|p| p.is_finite())
                            .or_else(|| self.commanded_positions.get(g).copied().filter(|p| p.is_finite()))?;
                        Some(
                            self.config
                                .limits
                                .get(g)
                                .map_or(raw, |l| raw.clamp(l.min_position, l.max_position)),
                        )
                    });
                    J7PosForceGate { stale: true, freeze_pos }
                } else {
                    J7PosForceGate::default()
                };
                let poll = crate::rt_core::ops::HardwarePoll {
                    driverless: false,
                    feedback_valid: false,
                    enable_state: Some(EnableState::Unknown),
                    oldest_feedback_age: None,
                    rx_frames,
                    budget_exhausted,
                    torque: self.core.poll_transition(),
                    park_wake,
                    park_state_changed,
                    urgent_wake: None,
                };
                (poll, gate)
            });
        self.rt_j7_pf_gate = Some(pf_gate);
        poll
    }

    /// Single-pass hardware receive + feedback intake.
    ///
    /// Called at Step 2 (`CoreOps::poll_hardware`) with `budget: Some(...)` so Step 3 commits
    /// and Step 4 targets see fresh feedback and Step 5 `tick_control` does not re-read the bus.
    /// Legacy callers invoking `tick_control` directly without a prior `poll_hardware` call this
    /// with `budget: None`.
    #[cfg(test)]
    pub(crate) fn poll_hardware_rx(
        &mut self,
        budget: Option<&crate::rt_core::ops::ServiceBudget>,
        transition_rx: Option<(u16, bool)>,
        release_deadline: Option<Instant>,
    ) -> PendingRxPoll {
        let entry_wall = Instant::now();
        let base = budget.map_or(entry_wall, |b| b.now);
        let mut live_now = || base + Instant::now().saturating_duration_since(entry_wall);
        self.poll_hardware_rx_with_clock(budget, transition_rx, release_deadline, &mut live_now)
    }

    #[inline(never)]
    pub(crate) fn poll_hardware_rx_with_clock(
        &mut self,
        budget: Option<&crate::rt_core::ops::ServiceBudget>,
        transition_rx: Option<(u16, bool)>,
        release_deadline: Option<Instant>,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> PendingRxPoll {
        let mut out = PendingRxPoll { active: true, ..PendingRxPoll::default() };
        let mut new_motor_errors = None;
        let mut sc = std::mem::take(&mut self.scratch);
        sc.recovered.clear();
        let margin = self.tx_release_publish_margin();
        let use_try = self.rt_executor_owned || self.nonblocking_driver_release || budget.is_some();
        if let Some(d) = self.driver.as_mut() {
            let run_poll = |driver: &mut dyn ArmDriver| {
                self.feedback_numeric_valid = false;
                if self.control_release_seq > 0 {
                    driver.begin_control_release(self.control_release_seq);
                }
                let applied_step2_tx_cap =
                    if let (Some(_), Some(rd)) = (budget, release_deadline) {
                        match rd.checked_sub(margin) {
                            Some(owner_cap) => {
                                match translate_driver_cutoff_conservative(driver, owner_cap, now_fn) {
                                    Ok((_, _, cap)) => {
                                        driver.set_tx_release_deadline(Some(cap));
                                        true
                                    }
                                    Err(_) => false,
                                }
                            }
                            None => false,
                        }
                    } else {
                        false
                    };
                let read_res = match (budget, transition_rx) {
                    (Some(b), Some((frames, exhausted))) => {
                        if driver.read_cached_joint_states_into(&mut sc.states).is_some() {
                            if !exhausted
                                && self.enable_lifecycle
                                    == driver_transitions::EnableLifecycle::FailedHold
                            {
                                driver
                                    .bus_recovery_mut().service_pending_refresh()
                                    .map(|()| (frames, exhausted))
                            } else {
                                Ok((frames, exhausted))
                            }
                        } else {
                            let remaining = b.max_rx_frames.saturating_sub(frames);
                            if remaining == 0 {
                                Ok((frames, true))
                            } else {
                                match translate_driver_cutoff_conservative(driver, b.deadline, now_fn) {
                                    Ok((_, _, drv_deadline)) => driver
                                        .read_joint_states_budgeted(&mut sc.states, drv_deadline, remaining)
                                        .map(|(f2, ex2)| (frames.saturating_add(f2), exhausted || ex2)),
                                    Err(_) => {
                                        let _ = driver.read_cached_joint_states_into(&mut sc.states);
                                        Ok((frames, true))
                                    }
                                }
                            }
                        }
                    }
                    (Some(b), None) => {
                        if b.max_rx_frames == 0 {
                            let _ = driver.read_cached_joint_states_into(&mut sc.states);
                            Ok((0, true))
                        } else {
                            match translate_driver_cutoff_conservative(driver, b.deadline, now_fn) {
                                Ok((_, _, drv_deadline)) => {
                                    driver.read_joint_states_budgeted(&mut sc.states, drv_deadline, b.max_rx_frames)
                                }
                                Err(_) => {
                                    let _ = driver.read_cached_joint_states_into(&mut sc.states);
                                    Ok((0, true))
                                }
                            }
                        }
                    }
                    (None, _) => driver.read_joint_states_into(&mut sc.states).map(|()| (0, false)),
                };
                if applied_step2_tx_cap {
                    driver.set_tx_release_deadline(None);
                }
                match read_res {
                    Ok((frames, exhausted)) => {
                        out.rx_frames = frames;
                        out.budget_exhausted = exhausted;
                        driver.joint_mode_mut().take_recovery_writes_into(&mut sc.recovered);
                        let has_loads = driver.joint_loads_into(&mut sc.loads);
                        self.feedback_numeric_valid = Self::states_valid(
                            &sc.states,
                            driver.capabilities(),
                            self.config.dof,
                            has_loads.then_some(sc.loads.as_slice()),
                        );
                        out.invalid_feedback = !self.feedback_numeric_valid;
                        let poll_before = self.driver_clock.as_ref().map_or_else(Instant::now, |c| c.now());
                        driver.feedback_ages_into(&mut sc.ages);
                        if !sc.recovered.is_empty() || self.tracking.recovery_suspended {
                            out.recovery_ages = true;
                        }
                        let driver_valid = driver.is_feedback_valid();
                        self.driver_feedback_confirmed_cached = driver_valid;
                        let all_ages = sc.ages.len() == self.config.dof && sc.ages.iter().all(Option::is_some);
                        let caps = driver.capabilities();
                        let enable_state = driver.enable_state();
                        self.driver_caps = caps;
                        self.last_poll_enable_state = enable_state;
                        self.last_poll_partial_enable = driver.has_known_partial_enable();
                        self.last_poll_feedback_valid = driver_valid;
                        self.last_poll_oldest_age = if all_ages {
                            sc.ages.iter().flatten().copied().max()
                        } else {
                            None
                        };
                        let mut cap_cache = terminal_capture::CaptureDriverCache {
                            enable_state,
                            driver_caps: caps,
                            feedback_valid: driver_valid && self.feedback_numeric_valid,
                            ..terminal_capture::CaptureDriverCache::default()
                        };
                        if self.feedback_numeric_valid {
                            cap_cache.copy_ages_from(&sc.ages, self.config.dof, poll_before);
                        }
                        self.capture_driver_cache = cap_cache;
                        let all_fresh_age = sc.ages.len() == self.config.dof
                            && sc.ages.iter().all(|a| a.is_some_and(|age| age <= Duration::from_millis(250)));
                        driver.feedback_receive_instants_into(&mut sc.rx);
                        let rx_dof = self.config.dof.min(crate::model::MAX_MOTION_DOF);
                        for i in 0..rx_dof {
                            let st_ok = sc
                                .states
                                .get(i)
                                .is_some_and(|st| crate::driver::joint_state_valid(st, caps))
                                && (!caps.load_feedback
                                    || (has_loads && sc.loads.get(i).is_some_and(|l| l.is_finite())));
                            if !st_ok {
                                continue;
                            }
                            let ts = sc
                                .rx
                                .get(i)
                                .copied()
                                .flatten()
                                .or_else(|| {
                                    sc.rx
                                        .is_empty()
                                        .then(|| {
                                            sc.ages
                                                .get(i)
                                                .copied()
                                                .flatten()
                                                .filter(|&age| age <= BUS_LOSS_TIMEOUT)
                                                .and_then(|age| poll_before.checked_sub(age))
                                        })
                                        .flatten()
                                });
                            if let Some(ts) = ts {
                                self.last_valid_motor_rx[i] = Some(match self.last_valid_motor_rx[i] {
                                    Some(prev) => prev.max(ts),
                                    None => ts,
                                });
                            }
                        }
                        if self.feedback_numeric_valid && sc.ages.len() == self.config.dof && driver_valid {
                            let oldest = sc.ages.iter().try_fold(Duration::ZERO, |m, a| a.map(|a| m.max(a)));
                            self.last_can_feedback_instant = oldest
                                .filter(|_| !sc.ages.is_empty())
                                .and_then(|age| Instant::now().checked_sub(age));
                        } else {
                            self.last_can_feedback_instant = None;
                        }
                        if self.feedback_numeric_valid {
                            self.core.poll_feedback(
                                &mut self.joint_feedback_queue_bounds,
                                self.config.dof,
                                driver,
                                &mut sc.fb,
                            );
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
                            if sc.rx.len() == self.config.dof && sc.rx.iter().all(Option::is_some) {
                                sc.rx_vals.clear();
                                sc.rx_vals.extend(sc.rx.iter().map(|t| t.unwrap()));
                                out.hw_rx = true;
                            }
                            let threshold = self.config.max_temperature - THERMAL_CLEAR_MARGIN_C;
                            let thermal_below = caps.temperature_feedback
                                && !self.temperatures.is_empty()
                                && self.temperatures.iter().all(|t| t.is_finite() && *t <= threshold);
                            self.thermal_cooldown = Self::evaluate_thermal_cooldown_fields(
                                self.config.dof,
                                self.thermal_cooldown,
                                driver,
                                &sc.ages,
                                thermal_below,
                                &mut sc.rx,
                            );
                        } else {
                            // Stale or invalid temperature feedback resets thermal cooldown.
                            self.thermal_cooldown.reset();
                        }
                    }
                    Err(_) => {
                        self.driver_feedback_confirmed_cached = false;
                        self.last_can_feedback_instant = None;
                        self.last_poll_feedback_valid = false;
                        self.last_poll_oldest_age = None;
                        self.capture_driver_cache.enable_state = driver.enable_state();
                        self.capture_driver_cache.feedback_valid = false;
                        self.capture_driver_cache.clear_ages();
                        self.thermal_cooldown.reset();
                        out.reset_stillness_on_read_err = true;
                    }
                }
                out.bus_feedback_fresh = self.feedback_numeric_valid
                    && driver.feedback_all_within(self.config.dof, BUS_LOSS_TIMEOUT);
                if driver.motor_status_into(&mut sc.status) {
                    let rt_owned = self.rt_executor_owned;
                    let changed = sc.status.len() != self.motor_errors.len()
                        || sc.status.iter().zip(&self.motor_errors).any(|(&code, e)| {
                            let decoded = e.as_ref().map(|e| e.code);
                            let has_name = driver.decode_motor_error_static(code).is_some()
                                || (!rt_owned && driver.decode_motor_error(code).is_some());
                            decoded != Some(code) && !(decoded.is_none() && !has_name)
                        });
                    if changed {
                        if rt_owned {
                            // Zero-alloc in-place update on `armd-control` using pre-reserved
                            // `self.motor_errors` capacity and static fault names.
                            self.motor_errors.resize_with(sc.status.len(), || None);
                            for (slot, &code) in self.motor_errors.iter_mut().zip(sc.status.iter()) {
                                *slot = driver.decode_motor_error_static(code).map(|name| {
                                    crate::model::MotorError {
                                        code,
                                        name: std::borrow::Cow::Borrowed(name),
                                    }
                                });
                            }
                        } else {
                            new_motor_errors = Some(
                                sc.status
                                    .iter()
                                    .map(|&code| {
                                        driver
                                            .decode_motor_error_static(code)
                                            .map(std::borrow::Cow::Borrowed)
                                            .or_else(|| {
                                                driver
                                                    .decode_motor_error(code)
                                                    .map(std::borrow::Cow::Owned)
                                            })
                                            .map(|name| crate::model::MotorError { code, name })
                                    })
                                    .collect(),
                            );
                        }
                    }
                }
                if self.pending_driver_bus_loss_notify {
                    driver.bus_recovery_mut().notify_bus_loss();
                    self.pending_driver_bus_loss_notify = false;
                }
                if self.is_torqued
                    && !self.disable_pending
                    && self.gravity_comp.is_none()
                    && self.safety.state().is_operational()
                    && driver.capabilities().enable_feedback
                    && driver.is_feedback_valid()
                {
                    if driver.enable_state() == EnableState::Enabled {
                        out.driver_enable_confirmed_ok = true;
                    } else if !(self.pending_rearm && driver.has_known_partial_enable()) {
                        out.driver_torque_lost = true;
                        out.driver_torque_loss_sample_seq =
                            driver.reported_torque_loss_sample_seq();
                    }
                }
                // R1/B2: Sync MIT ownership loss before capturing hold-mode verification so
                // MIT-configured joints in verified `PosVel` fall back to position-mode holds
                // while joints still in `Mit` mode enter L2's 200 ms readback -> fault bound.
                driver.mit_mut().set_mit_ownership_lost(self.mit.owner == crate::mit::MitOwner::Lost);
                // R1: Refresh the owner-held driver snapshot while already holding the driver
                // so stopped-hold capture and MIT stopped-hold preparation never lock
                // `self.driver` or allocate.
                self.owner_driver_snapshot =
                    driver_transitions::OwnerDriverSnapshot::capture_from_driver(
                        &*driver,
                        self.config.dof,
                        self.feedback_numeric_valid,
                        &self.measured_positions,
                        &mut sc.ages,
                        &mut sc.fb.bounds,
                    );
                let torqued_or_stopped = self.is_torqued
                    || (self.torque_intent == TorqueIntent::On
                        && (self.safety.state().is_stopped()
                            || matches!(
                                self.enable_lifecycle,
                                EnableLifecycle::FailedHold | EnableLifecycle::Armed
                            )));
                Self::enforce_unverified_mode_readback_parts(
                    &mut self.measured_hold,
                    &mut self.safety,
                    &self.owner_driver_snapshot,
                    self.torque_transition.is_some(),
                    torqued_or_stopped,
                    self.config.dof,
                    driver,
                );
                #[cfg(not(target_arch = "wasm32"))]
                self.torque_shadow.drain_from_driver(driver);
            };
            let poll_res = if use_try {
                d.try_with_mut(run_poll)
            } else {
                d.with_mut(run_poll)
                    .ok_or(crate::supervisor_lock::DriverTryError::Poisoned)
            };
            match poll_res {
                Ok(()) => {}
                Err(crate::supervisor_lock::DriverTryError::WouldBlock) => {
                    // M4: Never block on a contended legacy driver mutex during Step-2 RX poll.
                    self.driver_feedback_confirmed_cached = false;
                    self.capture_driver_cache.feedback_valid = false;
                    self.capture_driver_cache.clear_ages();
                }
                Err(crate::supervisor_lock::DriverTryError::Poisoned) => {
                    self.feedback_numeric_valid = false;
                    self.driver_feedback_confirmed_cached = false;
                    self.last_can_feedback_instant = None;
                    self.capture_driver_cache.enable_state = EnableState::Unknown;
                    self.capture_driver_cache.feedback_valid = false;
                    self.capture_driver_cache.clear_ages();
                    self.thermal_cooldown.reset();
                    self.owner_driver_snapshot.invalidate();
                    out.reset_stillness_on_read_err = true;
                }
            }
        } else {
            // Driverless simulation mode: each control tick produces a fresh joint sample.
            self.driver_feedback_confirmed_cached = false;
            let sim_now = Instant::now();
            self.core.simulate_feedback(self.config.dof, sim_now);
            #[cfg(not(target_arch = "wasm32"))]
            if self.torque_shadow.is_enabled() {
                let n = self.config.dof.min(crate::torque::MAX_CHANNELS);
                for ch in 0..n {
                    let pos = self.measured_positions.get(ch).copied().unwrap_or(0.0);
                    let vel = self.measured_velocities.get(ch).copied();
                    let temp = self.temperatures.get(ch).copied();
                    self.torque_shadow.enqueue_at(
                        sim_now,
                        crate::torque::shadow::ShadowInputKind::DriverObs(
                            crate::torque::shadow::ShadowDriverObs::CausalSample {
                                channel: ch as u8,
                                sample_id: self.control_release_seq.saturating_mul(64).saturating_add(ch as u64 + 1),
                                stamp: crate::torque::shadow::RxStampKind::Valid {
                                    rx_at: sim_now,
                                    drift_ns: 0,
                                    live_post_settle: true,
                                },
                                decode_now: sim_now,
                                position_rad: pos,
                                velocity_rad_s: vel,
                                temperature_c: temp,
                                reported_enabled: Some(self.is_torqued),
                                correlated_after_write_done: None,
                                poll_issued: None,
                                provenance: crate::torque::shadow::EvidenceProvenance::MockSynthetic,
                            },
                        ),
                    );
                }
            }
            self.joint_feedback_queue_bounds.clear();
            out.bus_feedback_fresh = true;
        }
        self.scratch = sc;
        if let Some(errors) = new_motor_errors {
            self.update_motor_errors(errors);
        }
        if out.reset_stillness_on_read_err {
            self.reset_table_stillness();
        }
        self.tick_phase(0);
        out
    }

    /// One control tick without building the observation; returns the tick's observation
    /// timestamp (the recording sample's). The control loop fills its published sample with
    /// [`Self::fill_observation`] at this timestamp.
    pub fn tick_control(&mut self, dt: f64) -> f64 {
        let mut sys_now = Instant::now;
        self.tick_control_inner(dt, None, &mut sys_now)
    }

    /// [`Self::tick_control`] bounded by an explicit control release deadline (`now`,
    /// `release_deadline`) so TX start-cutoffs never run past `release_deadline - publish_margin`.
    pub fn tick_control_with_deadline(&mut self, dt: f64, now: Instant, release_deadline: Instant) -> f64 {
        let mut fixed_now = move || now;
        self.tick_control_inner(dt, Some(release_deadline), &mut fixed_now)
    }

    /// [`Self::tick_control`] bounded by `release_deadline` with a live monotonic clock sampler
    /// `now_fn` so pre-step and transmit-point macro deadline checks always take fresh clock reads.
    pub fn tick_control_with_clock(
        &mut self,
        dt: f64,
        now_fn: &mut dyn FnMut() -> Instant,
        release_deadline: Instant,
    ) -> f64 {
        self.tick_control_inner(dt, Some(release_deadline), now_fn)
    }

    /// Legacy [`Self::tick_control`] with an injected clock sampler (for deterministic tests of
    /// mid-tick deadline checks without release-deadline capping).
    #[doc(hidden)]
    pub fn tick_control_legacy_with_clock(
        &mut self,
        dt: f64,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> f64 {
        self.tick_control_inner(dt, None, now_fn)
    }

    /// Zero-lock immutable borrow in `DriverSlot::Exclusive`; non-blocking `try_with_ref` in
    /// `DriverSlot::Legacy` to check `supports_rt_executor()` without invoking it during
    /// `install_driver` (preserves handover-preflight stall test semantics).
    #[inline]
    pub(crate) fn driver_supports_rt_executor_nonblocking(&self) -> bool {
        self.driver.as_ref().is_some_and(|d| {
            d.try_with_ref(|drv| drv.supports_rt_executor().is_ok())
                .unwrap_or(false)
        })
    }

    /// Zero-lock immutable borrow in `DriverSlot::Exclusive`; non-blocking `try_read` when
    /// `rt_executor_owned` or `nonblocking_driver_release` in `DriverSlot::Legacy`; blocking
    /// `read` on ordinary legacy releases.
    #[inline]
    pub(crate) fn read_driver_for_tick(
        &self,
    ) -> Option<crate::supervisor_lock::DriverReadGuard<'_>> {
        let d = self.driver.as_ref()?;
        if self.rt_executor_owned || self.nonblocking_driver_release {
            d.try_read()
        } else {
            d.read()
        }
    }

    /// Zero-lock mutable borrow in `DriverSlot::Exclusive`; non-blocking `try_write` when
    /// `rt_executor_owned` or `nonblocking_driver_release` in `DriverSlot::Legacy`; blocking
    /// `write` on ordinary legacy releases.
    #[inline]
    pub(crate) fn write_driver_for_tick(
        &mut self,
    ) -> Option<crate::supervisor_lock::DriverWriteGuard<'_>> {
        let d = self.driver.as_mut()?;
        if self.rt_executor_owned || self.nonblocking_driver_release {
            d.try_write()
        } else {
            d.write()
        }
    }

    /// Computes the single per-tick TX cutoff ONCE in the owner:
    ///   `effective_cutoff = min(control-release deadline, active action/macro parent & child deadlines)`
    /// translated into the driver clock domain using conservative driver-clock-first,
    /// live-owner-clock-second sampling. Rejects expired release cutoffs BEFORE staging any driver
    /// deadline. Safety/stopped holds are never clipped by an action deadline.
    #[inline(never)]
    fn install_effective_tx_cutoff(
        &mut self,
        release_deadline: Option<Instant>,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> (DriverTxCapOutcome, Option<EffectiveTxCutoff>) {
        if self.driver.is_none() {
            return (DriverTxCapOutcome::NotInstalled, None);
        }
        let margin = self.tx_release_publish_margin();
        let period = Duration::from_secs_f64(1.0 / f64::from(self.config.control_frequency_hz.max(1)));
        let is_stopped = self.safety.state().is_stopped()
            || self.enable_lifecycle == driver_transitions::EnableLifecycle::FailedHold;
        #[allow(unused_mut)]
        let mut fail_closed_action_dl = false;
        let owner_dl = if is_stopped {
            None
        } else {
            #[allow(unused_mut)]
            let mut dl = self.active_action_deadline();
            #[cfg(feature = "policy-runtime")]
            if self.watchdog_park.is_none() && !self.pending_normal_unload {
                if let Some(run) = self
                    .policy_run
                    .as_ref()
                    .filter(|r| r.mode == crate::policy_runtime::PolicyMode::Live)
                {
                    if let Some(run_dl) = run.deadline {
                        dl = Some(dl.map_or(run_dl, |cur| cur.min(run_dl)));
                    }
                    if self.lease_enabled {
                        // Include absolute lease expiry (fail-closed to immediate expiry if lease
                        // is missing or mismatched) so a policy target can never transmit after expiry.
                        if let Some(l) = self
                            .active_lease
                            .as_ref()
                            .filter(|l| l.generation == run.lease_generation)
                        {
                            let lease_exp = l.acquired_at + self.lease_timeout;
                            dl = Some(dl.map_or(lease_exp, |cur| cur.min(lease_exp)));
                        } else {
                            fail_closed_action_dl = true;
                        }
                    }
                }
            }
            dl
        };
        let use_try = self.rt_executor_owned || self.nonblocking_driver_release;
        let Some(d) = self.driver.as_mut() else {
            return (DriverTxCapOutcome::NotInstalled, None);
        };
        let apply = |drv: &mut dyn ArmDriver| {
            // Sample driver clock FIRST, then live owner clock inside the driver boundary, and
            // refuse expired release cutoffs BEFORE staging any driver deadline.
            let (drv_now, owner_now, drv_release_cap) = match release_deadline {
                Some(rd) => {
                    let Some(owner_cap) = rd.checked_sub(margin) else {
                        return (DriverTxCapOutcome::Expired, None);
                    };
                    match translate_driver_cutoff_conservative(drv, owner_cap, now_fn) {
                        Ok(translated) => translated,
                        Err(_) => return (DriverTxCapOutcome::Expired, None),
                    }
                }
                None => {
                    let drv_now = drv.clock_now();
                    let owner_now = now_fn();
                    let span = period.saturating_sub(margin);
                    (drv_now, owner_now, drv_now + span)
                }
            };
            let drv_action_cutoff = if fail_closed_action_dl {
                Some(drv_now)
            } else {
                owner_dl.map(|dl| drv_now + dl.saturating_duration_since(owner_now))
            };
            // When the action cutoff is already expired (`drv_action_cutoff == Some(drv_now)`),
            // stage ONLY the live `drv_release_cap` on the driver (preserving the hold/J7-freeze
            // exception without staging an expired deadline), while motion output checks
            // `drv_action_cutoff <= drv_anchor` and returns `ActionDeadlineRefused` with 0 TX.
            let drv_effective = match drv_action_cutoff {
                Some(ac) if ac > drv_now => drv_release_cap.min(ac),
                _ => drv_release_cap,
            };
            drv.set_tx_release_deadline(Some(drv_effective));
            (
                DriverTxCapOutcome::Installed,
                Some(EffectiveTxCutoff {
                    drv_release_cap,
                    drv_action_cutoff,
                    owner_anchor: owner_now,
                    drv_anchor: drv_now,
                }),
            )
        };
        if use_try {
            d.try_with_mut(apply)
                .unwrap_or((DriverTxCapOutcome::NotInstalled, None))
        } else {
            d.with_mut(apply)
                .unwrap_or((DriverTxCapOutcome::NotInstalled, None))
        }
    }

    /// True when legacy ticked hardware commissioning may run on this release (disarmed,
    /// no active torque transition or pending disable, not detached/shutting down, legacy driver).
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn legacy_commissioning_eligible(&self) -> bool {
        !self.detached
            && !self.shutdown_committed
            && !self.is_torqued
            && !self.disable_pending
            && self.torque_transition.is_none()
            && self.commissioning.is_running()
            && self
                .driver
                .as_ref()
                .is_some_and(|d| d.as_legacy_lock().is_some())
    }

    #[inline(never)]
    fn tick_control_inner(
        &mut self,
        dt: f64,
        release_deadline: Option<Instant>,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> f64 {
        if release_deadline.is_some() {
            self.last_clock_now = Some(now_fn());
        }
        // Legacy callers without a preceding Step-2 `poll_hardware_for_release` run `poll_hardware_rx`
        // below; bound any stale-joint recovery TX during RX intake to the control-release cap
        // using conservative driver-first, live-owner-second translation.
        // Shared legacy transition step budget: <= 1 ms leaves >= 1 ms margin for the rest of the
        // control tick so every write-guard hold stays strictly <= 2 ms even under sustained RX load.
        const LEGACY_TORQUE_STEP_BUDGET: Duration = Duration::from_millis(1);
        let has_legacy_transition =
            !self.pending_rx_poll.active && self.has_non_broker_torque_transition();
        let has_legacy_enable_transition = has_legacy_transition
            && matches!(
                self.torque_transition,
                Some(driver_transitions::TorqueTransition::Enable(_))
            );
        self.nonblocking_driver_release = has_legacy_transition;
        let legacy_step_start = has_legacy_transition.then(&mut *now_fn);
        let period_tick =
            Duration::from_secs_f64(1.0 / f64::from(self.config.control_frequency_hz.max(1)));
        let margin_tick = self.tx_release_publish_margin();
        let legacy_rx_deadline = legacy_step_start.map(|t| {
            if has_legacy_enable_transition {
                t + LEGACY_TORQUE_STEP_BUDGET
            } else if let Some(rd) = release_deadline {
                rd.checked_sub(margin_tick).unwrap_or(t)
            } else {
                t + period_tick.saturating_sub(margin_tick)
            }
        });
        let legacy_tx_deadline = legacy_step_start.map(|t| {
            if let Some(rd) = release_deadline {
                rd.checked_sub(margin_tick).unwrap_or(t)
            } else {
                t + period_tick.saturating_sub(margin_tick)
            }
        });
        let (applied_rx_tx_cap, rx_tx_cap_expired) = if !self.pending_rx_poll.active && self.driver.is_some() {
            let margin = self.tx_release_publish_margin();
            let period = Duration::from_secs_f64(1.0 / f64::from(self.config.control_frequency_hz.max(1)));
            if let Some(mut drv) = self.write_driver_for_tick() {
                match release_deadline {
                    Some(rd) => match rd.checked_sub(margin) {
                        Some(owner_cap) => {
                            match translate_driver_cutoff_conservative(drv.as_ref(), owner_cap, now_fn) {
                                Ok((drv_now, _, mut cap)) => {
                                    if let (Some(step_start), Some(step_dl)) =
                                        (legacy_step_start, legacy_tx_deadline)
                                    {
                                        let step_cap =
                                            drv_now + step_dl.saturating_duration_since(step_start);
                                        if step_cap < cap {
                                            cap = step_cap;
                                        }
                                    }
                                    drv.set_tx_release_deadline(Some(cap));
                                    (true, false)
                                }
                                Err(_) => (false, true),
                            }
                        }
                        None => (false, true),
                    },
                    None => {
                        let drv_now = drv.clock_now();
                        let _ = now_fn();
                        let mut cap = drv_now + period.saturating_sub(margin);
                        if let (Some(step_start), Some(step_dl)) =
                            (legacy_step_start, legacy_tx_deadline)
                        {
                            let step_cap = drv_now + step_dl.saturating_duration_since(step_start);
                            if step_cap < cap {
                                cap = step_cap;
                            }
                        }
                        drv.set_tx_release_deadline(Some(cap));
                        (true, false)
                    }
                }
            } else {
                (false, false)
            }
        } else {
            (false, false)
        };
        self.consume_pending_estop();
        self.service_internal_fault_lane();
        // Immediate atomic e-stop latch check before any processing (only latches once when not yet recorded;
        // every new E-stop request arrives via `consume_pending_estop` above and advances the epoch).
        if self.e_stop_latched.load(Ordering::SeqCst)
            && !self.safety.has_latched_fault_kind(crate::safety::FaultKind::Estop)
        {
            self.apply_stop_policy(StopPolicy::EStopLatch);
        }

        // Reuse Step 2's single budgeted hardware poll when present; legacy callers without
        // a preceding `poll_hardware` advance the control-release counter and run a single
        // `ReleaseBudget` ledger for parity across transition poll, RX/recovery, and ModeSwitch.
        let was_step2_poll = self.pending_rx_poll.active;
        #[cfg(not(target_arch = "wasm32"))]
        let mut legacy_ledger: Option<crate::rt_core::budget::ReleaseBudget> = None;
        let rx = if was_step2_poll {
            std::mem::take(&mut self.pending_rx_poll)
        } else {
            self.control_release_seq = self.control_release_seq.wrapping_add(1).max(1);
            self.control_release_seq_atomic
                .store(self.control_release_seq, Ordering::Release);
            let rel_seq = self.control_release_seq;
            if let Some(mut drv) = self.write_driver_for_tick() {
                drv.begin_control_release(rel_seq);
            }
            let leg_now = legacy_step_start.unwrap_or_else(&mut *now_fn);
            let leg_rd = release_deadline.unwrap_or(leg_now + period_tick);
            let mut leg_budget = crate::rt_core::budget::ReleaseBudget::new(
                rel_seq,
                leg_now,
                leg_rd,
                crate::rt_core::ops::RX_FRAME_BUDGET,
                self.release_tx_budget_limit(),
            );
            #[cfg(not(target_arch = "wasm32"))]
            self.torque_shadow
                .enqueue_at(
                    now_fn(),
                    crate::torque::shadow::ShadowInputKind::Release {
                        release_seq: self.control_release_seq,
                        live_intent: self.shadow_live_intent(),
                        live_is_torqued: self.is_torqued,
                        live_stopped: self.safety.state().is_stopped(),
                    },
                );
            let rx_out = if rx_tx_cap_expired {
                // Cutoff already expired: read cached feedback ONLY so unbudgeted RX can never
                // emit Damiao recovery TX past the release cutoff (DRV-F1 B9).
                leg_budget.tighten_rx_remaining(0);
                let expired_now = now_fn();
                let expired_budget = leg_budget.service_budget(expired_now, Duration::ZERO, 0);
                self.poll_hardware_rx_with_clock(
                    Some(&expired_budget),
                    None,
                    release_deadline,
                    now_fn,
                )
            } else if self.has_non_broker_torque_transition() {
                let now = legacy_step_start.unwrap_or_else(&mut *now_fn);
                let deadline = legacy_rx_deadline.unwrap_or_else(|| now + LEGACY_TORQUE_STEP_BUDGET);
                let before_t = self.sync_driver_budget_before(&leg_budget);
                let budget = crate::rt_core::ops::ServiceBudget {
                    now,
                    deadline: deadline.min(leg_budget.deadline()),
                    max_rx_frames: if before_t.is_some() {
                        leg_budget.rx_remaining()
                    } else {
                        0
                    },
                };
                self.arm_conservative_cap_if_contended(before_t);
                let trans = self.service_torque_transition_with_clock(&budget, now_fn);
                let (cons_rx, cons_tx) = self.take_conservative_step_deltas();
                let transition_rx = trans.and_then(|tp| tp.rx);
                if let Some((tf, tex)) = transition_rx {
                    self.charge_driver_budget_after(
                        &mut leg_budget,
                        before_t,
                        tf.max(cons_rx),
                        tex,
                        cons_tx,
                    );
                } else {
                    self.charge_driver_budget_after(&mut leg_budget, before_t, cons_rx, false, cons_tx);
                }
                if self.driver_supports_rt_executor_nonblocking() {
                    let after = now_fn();
                    let before_rx = self.sync_driver_budget_before(&leg_budget);
                    let rem_budget = crate::rt_core::ops::ServiceBudget {
                        now: after,
                        deadline: deadline.min(leg_budget.deadline()),
                        max_rx_frames: if before_rx.is_some() {
                            leg_budget.rx_remaining()
                        } else {
                            0
                        },
                    };
                    let rx_marker = transition_rx.map(|(_, ex)| (0u16, ex));
                    let mut r = self.poll_hardware_rx_with_clock(
                        Some(&rem_budget),
                        rx_marker,
                        release_deadline,
                        now_fn,
                    );
                    self.charge_driver_budget_after(&mut leg_budget, before_rx, r.rx_frames, r.budget_exhausted, 0);
                    r.rx_frames = leg_budget.rx_used();
                    r
                } else {
                    let before_rx = self.sync_driver_budget_before(&leg_budget);
                    let r = if before_rx.is_some() {
                        self.poll_hardware_rx_with_clock(None, None, None, now_fn)
                    } else {
                        let conservative = leg_budget.service_budget(now_fn(), Duration::ZERO, 0);
                        self.poll_hardware_rx_with_clock(Some(&conservative), None, None, now_fn)
                    };
                    self.charge_driver_budget_after(&mut leg_budget, before_rx, r.rx_frames, r.budget_exhausted, 0);
                    r
                }
            } else {
                let before_rx = self.sync_driver_budget_before(&leg_budget);
                let r = if before_rx.is_some() {
                    self.poll_hardware_rx_with_clock(None, None, None, now_fn)
                } else {
                    let conservative = leg_budget.service_budget(now_fn(), Duration::ZERO, 0);
                    self.poll_hardware_rx_with_clock(Some(&conservative), None, None, now_fn)
                };
                self.charge_driver_budget_after(&mut leg_budget, before_rx, r.rx_frames, r.budget_exhausted, 0);
                r
            };
            #[cfg(not(target_arch = "wasm32"))]
            {
                legacy_ledger = Some(leg_budget);
            }
            rx_out
        };
        #[cfg(not(target_arch = "wasm32"))]
        if !was_step2_poll
            && !self.rt_executor_owned
            && !self.detached
            && !self.shutdown_committed
            && !self.internal_fault_lane.is_fenced()
            && self
                .sessions
                .slot(crate::rt_core::session::SessionSlot::ExclusiveOutput)
                .is_some_and(|s| {
                    s.kind == crate::rt_core::session::SessionKind::ModeSwitch && s.is_live()
                })
        {
            let ms_now = now_fn();
            let ms_rd = release_deadline.unwrap_or(ms_now + period_tick);
            let ms_ctx = crate::rt_core::ops::CommitCtx {
                arm: self.core.id(),
                boot: crate::rt_core::broker::BootId::process(),
                now: ms_now,
                stop_generation: self.current_stop_gen(),
            };
            if let Some(ref mut leg_budget) = legacy_ledger {
                self.service_mode_switch_release_with_budget(&ms_ctx, leg_budget, now_fn);
            } else {
                self.service_mode_switch_release_with_clock(&ms_ctx, ms_rd, now_fn);
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        let legacy_output_before = if let Some(ref leg_budget) = legacy_ledger {
            let b = self.sync_driver_budget_before(leg_budget);
            self.arm_conservative_cap_if_contended(b);
            b
        } else {
            None
        };
        let invalid_feedback = rx.invalid_feedback;
        let driver_torque_lost = rx.driver_torque_lost;
        let driver_enable_confirmed_ok = rx.driver_enable_confirmed_ok;
        let driver_torque_loss_sample_seq = rx.driver_torque_loss_sample_seq;
        let bus_feedback_fresh = rx.bus_feedback_fresh;
        let recovery_ages = rx.recovery_ages;
        let hw_rx = rx.hw_rx;
        let mut sc = std::mem::take(&mut self.scratch);
        // B1: autonomous driver recovery writes (e.g. Damiao 45 ms POS_VEL hold) invalidate
        // tracking output history and suspend compensated writes until fresh post-recovery
        // feedback confirms a clean baseline across every joint.
        if self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled) {
            if !sc.recovered.is_empty() {
                if !self.safety.deferred_logging() {
                    tracing::warn!(
                        "Driver emitted autonomous recovery writes for joints {:?}; suspending tracking compensation until fresh baseline",
                        sc.recovered.iter().map(|i| i + 1).collect::<Vec<_>>()
                    );
                }
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
                    let mut recovered_updates = [(0usize, 0.0f32); crate::model::MAX_MOTION_DOF];
                    let mut rec_len = 0usize;
                    for &(j, _) in &self.tracking.recovery_joints {
                        if j < self.config.dof
                            && Some(j) != gripper_idx
                            && rec_len < recovered_updates.len()
                        {
                            let l = &self.config.limits[j];
                            let clamped = self.measured_positions[j].clamp(l.min_position, l.max_position);
                            self.commanded_positions[j] = clamped;
                            self.target_positions[j] = clamped;
                            recovered_updates[rec_len] = (j, clamped);
                            rec_len += 1;
                        }
                    }
                    self.commanded_velocities.fill(0.0);
                    self.commanded_accelerations.fill(0.0);
                    self.tracking.resume_from_recovery(&recovered_updates[..rec_len]);
                }
            }
        }
        if invalid_feedback && self.is_torqued {
            self.stop_motion();
            self.tracking.invalidate();
            self.commanded_velocities.fill(0.0);
            if !self.safety.has_latched_fault_code(FaultCode::InvalidTelemetry) {
                self.safety.latch(
                    crate::safety::FaultKind::Other,
                    crate::safety::FaultDetail::Static(crate::safety::StaticFault::InvalidTelemetry),
                );
            }
        }
        if driver_torque_lost
            && self.is_torqued
            && !self.disable_pending
            && self.mit.owner != crate::mit::MitOwner::Lost
        {
            let policy_running = {
                #[cfg(feature = "policy-runtime")]
                {
                    self.policy_run.is_some()
                }
                #[cfg(not(feature = "policy-runtime"))]
                {
                    false
                }
            };
            let in_active_motion = self.active_trajectory.is_some()
                || self.servo_target.is_some()
                || self.servo_stream_active
                || self.pending_normal_unload
                || self.quintic_settle.is_some()
                || self.gripper_stream_target.is_some()
                || self.mit.owner == crate::mit::MitOwner::Active
                || policy_running;
            if in_active_motion {
                // R12 M2: During active motion, a report of unexpected torque loss may stop
                // motion immediately (safety) while keeping `torque_intent == On` so stationary
                // holds continue. Do not latch a fault on the report alone: require persistence
                // across K=3 causal samples below.
                self.stop_motion();
                self.servo_stream_active = false;
                self.quintic_settle = None;
                self.tracking.invalidate();
                if self.measured_positions.iter().all(|p| p.is_finite()) {
                    copy_into(&mut self.commanded_positions, &self.measured_positions);
                    copy_into(&mut self.target_positions, &self.measured_positions);
                }
                self.commanded_velocities.fill(0.0);
                self.commanded_accelerations.fill(0.0);
                self.reported_torque_loss_from_motion = true;
            }
            if self.reported_torque_loss_from_motion {
                let release_seq = self.control_release_seq;
                let distinct_release =
                    release_seq != 0 && release_seq != self.reported_torque_loss_last_release;
                let fresh_causal_sample = match driver_torque_loss_sample_seq {
                    Some(seq) => seq != 0 && Some(seq) != self.reported_torque_loss_last_sample_seq,
                    None => hw_rx,
                };
                if distinct_release && fresh_causal_sample {
                    self.reported_torque_loss_last_release = release_seq;
                    if let Some(seq) = driver_torque_loss_sample_seq {
                        self.reported_torque_loss_last_sample_seq = Some(seq);
                    }
                    self.reported_torque_loss_count =
                        self.reported_torque_loss_count.saturating_add(1);
                    if self.reported_torque_loss_count >= crate::driver::EVIDENCE_WINDOW_SAMPLES {
                        self.safety.latch(
                            crate::safety::FaultKind::Other,
                            crate::safety::FaultDetail::Static(
                                crate::safety::StaticFault::UnexpectedTorqueLoss,
                            ),
                        );
                    }
                }
            } else {
                // R11 M1 / R12 M2: When holding / not actively moving, reported enable state is
                // telemetry + warning only — never latch `UnexpectedTorqueLoss` or stop/drop holds.
                let now = Instant::now();
                let should_warn = self
                    .last_reported_torque_loss_warn
                    .is_none_or(|prev| now.saturating_duration_since(prev) >= Duration::from_secs(1));
                if should_warn {
                    self.last_reported_torque_loss_warn = Some(now);
                    if !self.safety.deferred_logging() {
                        tracing::warn!(
                            "Motor feedback reports disabled/unknown while holding (telemetry only; holds remain active)"
                        );
                    }
                }
            }
        } else if driver_enable_confirmed_ok
            || !self.is_torqued
            || self.disable_pending
            || self.torque_intent == TorqueIntent::Off
        {
            self.reset_reported_torque_loss();
        }

        // A single late tick only shortens integration (dt is clamped below); the motors
        // keep holding their last POS_VEL target meanwhile. Fault on a real stall, using
        // CONTROL_DEADLINE (the same 250 ms bound as the hardware feedback deadline).
        if (!dt.is_finite() || dt <= 0.0 || dt > CONTROL_DEADLINE.as_secs_f64())
            && self.is_torqued
            && !matches!(self.safety.state(), SafetyState::Booting | SafetyState::Idle)
        {
            self.stop_motion();
            self.tracking.invalidate();
            if self.measured_positions.iter().all(|p| p.is_finite()) {
                copy_into(&mut self.commanded_positions, &self.measured_positions);
                copy_into(&mut self.target_positions, &self.measured_positions);
            }
            self.commanded_velocities.fill(0.0);
            self.commanded_accelerations.fill(0.0);
            self.safety.latch(
                crate::safety::FaultKind::Other,
                crate::safety::FaultDetail::Static(crate::safety::StaticFault::ControlDeadline),
            );
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

        // 0.0 Apply any non-blocking socket-close lease fence, pending disconnect hold, and
        // lease-expiry fence BEFORE gripper, motion, or bus output runs.
        self.sync_nonblocking_disconnect_fence(now_fn());

        // 0.0b Output-boundary policy deadline & authorization recheck BEFORE the gripper
        // force/contact block reads `gripper_stream_target` (so an expired or revoked policy never
        // advances J7 via CapRamp or emits post-deadline motion).
        #[cfg(feature = "policy-runtime")]
        self.enforce_policy_deadline_and_auth_rt(now_fn());

        // 0.1 Gripper stall protection state machine
        if self.active_trajectory.is_some() || self.macro_run.is_some() || self.pending_normal_unload
            || self.gravity_comp.is_some() || self.last_motion_frame.elapsed() > self.motion_frame_timeout {
            // stream end: no further advance, the last command stays (it may hold a grip)
            self.gripper_stream_target = None;
        }
        // E-stop, fault, torque off or the stream's lease gone (lost, released, preempted)
        let stream_lease_gone = self.lease_enabled
            && self.active_lease.as_ref().map(|l| l.generation) != self.gripper_stream_lease;
        if (self.gripper_stream_target.is_some() || self.gripper_stream_contact.is_some())
            && (!self.safety.state().is_operational() || !self.is_torqued || stream_lease_gone) {
            self.end_gripper_stream();
        }
        if let Some(g_idx) = self.config.gripper_index() {
            let (is_bus_fresh, has_torque_feedback, has_load_feedback, pos_force_info) = if self.driver.is_some() {
                if let Some(driver) = self.read_driver_for_tick() {
                    let fresh = self.feedback_numeric_valid
                        && self.last_can_feedback_instant.is_some_and(|t| t.elapsed() < Duration::from_millis(50))
                        && driver.is_feedback_valid();
                    let caps = driver.capabilities();
                    let pf = driver.pos_force().gripper_pos_force_info().filter(|info| info.joint == g_idx);
                    (fresh, caps.torque_feedback, caps.load_feedback, pf)
                } else {
                    (false, false, false, None)
                }
            } else {
                (true, true, false, None) // In simulation without a physical driver
            };

            let tau_g = self.torques.get(g_idx).copied().unwrap_or(0.0);
            let load_g = self.loads.get(g_idx).copied().unwrap_or(0.0);
            let p_cmd = self.commanded_positions.get(g_idx).copied().unwrap_or(0.0);
            let p_meas = self.measured_positions.get(g_idx).copied().unwrap_or(0.0);
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

            // Learned free-motion torque (crate::gripper_baseline) in the direction the jaw is
            // driven: the stall threshold and the stream contact threshold are at least that + margin.
            // Only closing is raised, and by at most GRIPPER_STALL_BASELINE_RAISE_MAX_NM.
            let driven = if (desired_g - p_meas).abs() > 1e-3 { desired_g - p_meas } else { gap };
            let driven_closing = driven * close_dir > 0.0;
            let learned_close_nm = self.gripper_baseline_threshold(p_meas, true, crate::gripper_baseline::SLOW_BAND_MAX_RAD_S * 0.5);
            let stall_nm = match learned_close_nm.filter(|_| driven_closing) {
                Some(t) => t.clamp(GRIPPER_STALL_LATCH_NM, GRIPPER_STALL_LATCH_NM + GRIPPER_STALL_BASELINE_RAISE_MAX_NM),
                None => GRIPPER_STALL_LATCH_NM,
            };
            // Stream contact regulator (gripper_policy.stream_contact_speed_rad_s): uses the same
            // shared `GripperContactDetector` (net-progress watermark window, moving torque floors,
            // sustained tick counts; no single-sample or instantaneous-velocity shortcuts). From
            // contact the closing command creeps at the contact speed while |τ| < target_nm and
            // its lead over the jaw is scaled by target_nm / |τ| above it, so the grip settles
            // near target_nm whatever the stiffness.
            // A learned baseline moves the contact detection to baseline + margin; the regulated
            // grip torque stays contact_torque_nm, raised by at most GRIPPER_STREAM_TARGET_RAISE_MAX_NM.
            let v_cmd = self.commanded_velocities.get(g_idx).copied().unwrap_or(0.0);
            let force_cap_nm = self.config.gripper_policy.as_ref().and_then(|p| p.max_close_torque_nm);
            let contact_torque_nm = self.config.gripper_policy.as_ref().and_then(|p| p.contact_torque_nm);
            let stream_detect_nm = crate::gripper_baseline::closing_contact_threshold(self.gripper_baseline.as_ref(), p_meas,
                crate::gripper_baseline::SLOW_BAND_MAX_RAD_S * 0.5,
                contact_torque_nm, force_cap_nm).map(|t| t.0);
            let regulator = self.config.gripper_policy.as_ref()
                .and_then(|p| Some((p.contact_torque_nm?, p.stream_contact_speed_rad_s?)))
                .map(|(c, speed)| {
                    let d = stream_detect_nm.unwrap_or(c);
                    (d.clamp(c, c + GRIPPER_STREAM_TARGET_RAISE_MAX_NM), speed)
                })
                .filter(|_| has_torque_feedback && self.is_torqued);
            let ceiling_est_nm = pos_force_info
                .and_then(|info| info.ceiling_torque_estimate_nm)
                .filter(|v| v.is_finite() && *v > 0.0);
            let stream_window_speed = {
                let v = f64::from(v_cmd.abs());
                if v > 1e-4 {
                    Some(v)
                } else {
                    pos_force_info
                        .map(|info| f64::from(info.close_speed_rad_s))
                        .filter(|v| v.is_finite() && *v > 0.0)
                }
            };
            match (regulator, self.gripper_stream_target) {
                (Some((target_nm, speed)), Some(stream_g)) if is_bus_fresh && p_meas.is_finite() && p_cmd.is_finite() => {
                    let tau = tau_g.abs();
                    self.gripper_stream_contact = match self.gripper_stream_contact {
                        None => {
                            let pushing = (p_cmd - p_meas) * close_dir > 1e-3 || (stream_g - p_meas) * close_dir > 1e-3;
                            if !pushing {
                                self.gripper_stream_contact_detector.reset();
                                None
                            } else {
                                let eval = self.gripper_stream_contact_detector.step(&crate::motion::GripperContactInput {
                                    baseline: self.gripper_baseline.as_ref(),
                                    contact_torque_nm,
                                    max_close_torque_nm: force_cap_nm,
                                    ceiling_torque_estimate_nm: ceiling_est_nm,
                                    stopped_stall_nm: Some(stall_nm),
                                    measured_pos: p_meas,
                                    dir: close_dir,
                                    baseline_speed: crate::gripper_baseline::SLOW_BAND_MAX_RAD_S * 0.5,
                                    window_speed: stream_window_speed,
                                    tau_abs: tau,
                                    dt,
                                    torque_feedback: has_torque_feedback,
                                    active: pushing,
                                });
                                eval.triggered.then(|| {
                                    self.gripper_stream_contact_detector.reset();
                                    let lead = ((p_cmd - p_meas) * close_dir).max(0.0);
                                    let scale = if tau > target_nm { target_nm / tau } else { 1.0 };
                                    GripperStreamContact { at_rad: p_meas, limit_rad: p_meas + close_dir * lead * scale }
                                })
                            }
                        }
                        // the jaw opened past the contact point, or closed on with little torque
                        Some(c) if (p_meas - c.at_rad) * close_dir <= -GRIPPER_FORCE_RELEASE_MARGIN_RAD => {
                            self.gripper_stream_contact_detector.reset();
                            None
                        }
                        Some(c) if tau < 0.5 * target_nm && (p_meas - c.at_rad) * close_dir > GRIPPER_STREAM_CONTACT_LOST_RAD => {
                            self.gripper_stream_contact_detector.reset();
                            None
                        }
                        Some(mut c) => {
                            self.gripper_stream_contact_detector.reset();
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
                (Some(_), Some(_)) => {
                    // late feedback: keep an engaged limit (gripper_lead_cap needs fresh feedback),
                    // but do not carry unlatched contact ticks across stale gaps.
                    if self.gripper_stream_contact.is_none() {
                        self.gripper_stream_contact_detector.reset();
                    }
                }
                _ => {
                    self.gripper_stream_contact = None;
                    self.gripper_stream_contact_detector.reset();
                }
            }

            // Torque cap: |τ| above max_close_torque_nm while the jaw is driven closed latches
            // the stall whatever the jaw speed (a deforming object or a flexing finger keeps
            // the jaw moving, so the stall rule below never sees it still; 2026-11-20 a finger
            // broke at 5 N·m). Every mode reads gripper_stalled: macro gripper moves, servo
            // streams (teleop, policies) and whole-arm references (gripper_hold_filter).
            if !self.gripper_stalled { self.gripper_force_capped = false; }
            if !(self.gripper_stalled && self.gripper_force_capped) { self.gripper_cap_ramp = None; }
            let closing = (p_cmd - p_meas) * close_dir > 1e-3 || (desired_g - p_meas) * close_dir > 1e-3 || v_cmd * close_dir > 1e-3;
            let opening = !closing
                && ((p_cmd - p_meas) * close_dir < -1e-3 || (desired_g - p_meas) * close_dir < -1e-3 || v_cmd * close_dir < -1e-3);
            let force_cap = self.config.gripper_policy.as_ref().and_then(|p| p.max_close_torque_nm);
            // closing: |τ| over the cap; opening: the torque pushing open over the cap (signed, so
            // releasing a squeezed object, whose torque still pushes closed, does not trip it)
            let over_cap = is_bus_fresh && has_torque_feedback && force_cap.is_some_and(|cap|
                (closing && tau_g.abs() > cap) || (opening && -tau_g * close_dir > cap));
            let cap_dir = if opening { -close_dir } else { close_dir };
            self.gripper_force_ticks = if over_cap { self.gripper_force_ticks.saturating_add(1) } else { 0 };
            let new_over_cap = over_cap && !self.gripper_over_cap_prev;
            self.gripper_over_cap_prev = over_cap;

            if force_cap.is_some() && closing && self.is_torqued && (!is_bus_fresh || !has_torque_feedback) {
                if !(self.gripper_stalled && self.gripper_force_capped) {
                    if self.rt_executor_owned {
                        crate::rt_core::ops::gripper_force_diag()
                            .record_feedback_unavailable(is_bus_fresh, has_torque_feedback);
                    } else {
                        tracing::warn!(
                            "Gripper close stopped: feedback_fresh={is_bus_fresh} torque_feedback={has_torque_feedback} while max_close_torque_nm is active; holding"
                        );
                    }
                }
                self.gripper_stalled = true;
                self.gripper_force_capped = true;
                self.gripper_force_ticks = 0;
                self.gripper_stall_sign = close_dir;
                self.gripper_high_torque_ticks = 0;
                self.gripper_low_torque_ticks = 0;
                self.gripper_stall_window.reset();
                self.gripper_stream_target = None;
                self.gripper_stream_contact = None;
                self.gripper_stream_contact_detector.reset();
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
                self.gripper_stall_window.reset();
            } else if self.gripper_force_ticks >= GRIPPER_FORCE_CAP_TICKS {
                // latch, or refresh a latch that still presses, at the measured position; every
                // new over-cap episode counts a trip (also one that refreshes a latch)
                if new_over_cap {
                    self.gripper_force_trips += 1;
                    let lead = ((p_cmd - p_meas) * cap_dir).max(0.0);
                    match self.gripper_cap_ramp.as_mut().filter(|_| self.gripper_stalled && self.gripper_force_capped) {
                        Some(r) => r.trip(tau_g.abs(), lead),
                        None => self.gripper_cap_ramp = Some(crate::motion::CapRamp::from_trip(tau_g.abs(), lead)),
                    }
                    let trips = self.gripper_cap_ramp.map_or(1, |r| r.trips);
                    if self.rt_executor_owned {
                        crate::rt_core::ops::gripper_force_diag().record_cap_trip(
                            tau_g.abs(),
                            force_cap.unwrap_or(0.0),
                            p_meas,
                            trips,
                        );
                    } else {
                        tracing::warn!("Gripper torque cap: |tau| {:.2} N·m above {:.2} N·m at {p_meas:.3} rad (trip {trips} on this contact); holding",
                            tau_g.abs(), force_cap.unwrap_or(0.0));
                    }
                }
                self.gripper_stalled = true;
                self.gripper_force_capped = true;
                self.gripper_force_ticks = 0;
                self.gripper_stall_sign = cap_dir;
                self.gripper_high_torque_ticks = 0;
                self.gripper_low_torque_ticks = 0;
                self.gripper_stall_window.reset();
                self.gripper_stream_contact_detector.reset();
                if p_meas.is_finite() {
                    if g_idx < self.commanded_positions.len() { self.commanded_positions[g_idx] = p_meas; }
                    if g_idx < self.commanded_velocities.len() { self.commanded_velocities[g_idx] = 0.0; }
                    if g_idx < self.target_positions.len() { self.target_positions[g_idx] = p_meas; }
                    if let Some(ref mut st) = self.servo_target {
                        if st.len() > g_idx { st[g_idx] = p_meas; }
                    }
                    if let Some(h) = self.gripper_motion_hold.as_mut() { *h = (p_meas, cap_dir); }
                }
                self.note_motion_limit(MotionLimitReason::GripperForce, Some(g_idx));
                // a stream that keeps tripping the cap on one contact is stopped
                if new_over_cap && self.gripper_stream_target.is_some()
                    && self.gripper_cap_ramp.is_some_and(|r| r.exhausted()) {
                    if self.rt_executor_owned {
                        crate::rt_core::ops::gripper_force_diag()
                            .record_stream_stopped(crate::motion::CAP_RAMP_MAX_TRIPS);
                    } else {
                        tracing::warn!("Gripper torque cap tripped {} times on one contact under a stream; stopping the stream", crate::motion::CAP_RAMP_MAX_TRIPS);
                    }
                    self.stop_motion();
                }
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
                self.gripper_stall_window.reset();
                self.gripper_stream_contact_detector.reset();
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
                let active_gripper_motion = self.active_trajectory.as_ref().and_then(|a| match &a.reference {
                    crate::motion::Reference::Gripper(gm) if gm.gripper_index == g_idx => Some(gm),
                    _ => None,
                });
                // When driven open, only torque pushing in the opening direction (`-close_dir`)
                // counts as stall effort — residual closing preload from a closed stop or squeeze
                // (`tau_g * close_dir > 0`) must not falsely latch an opening stall while the
                // command unwinds (same signed opening rule as `over_cap` above). Closing and idle
                // keep unsigned `|tau_g| >= stall_nm`.
                let opening_drive = active_gripper_motion
                    .is_some_and(|gm| (gm.target - gm.start) * close_dir < -1e-6)
                    || (active_gripper_motion.is_none() && opening);
                let high_effort = if has_torque_feedback {
                    // the stream contact regulator holds the grip instead of the stall hold at
                    // the measured jaw (which reads ~0 N·m on a rigid object and re-squeezes);
                    // the threshold is raised to the learned free-motion torque + margin where
                    // the jaw needs more than GRIPPER_STALL_LATCH_NM to move
                    // during a calibration stroke the command must also lead the jaw: a free jaw
                    // settling onto its target reads its friction torque at low speed with the
                    // command next to it, and every stall fails the calibration
                    let torque_high = if opening_drive {
                        -tau_g * close_dir >= stall_nm
                    } else {
                        tau_g.abs() >= stall_nm
                    };
                    self.gripper_stream_contact.is_none()
                        && torque_high
                        && (self.gripper_calibration.is_none() || gap.abs() > GRIPPER_STALL_MIN_LEAD_RAD)
                } else {
                    let stall_load_thresh = self.config.gripper_policy.as_ref()
                        .filter(|p| p.holding_mode == crate::model::HoldingMode::LoadAndGap)
                        .and_then(|p| p.holding_load_fraction)
                        .map(|frac| (frac * 1.25).clamp(0.18, 0.35))
                        .unwrap_or(0.35);
                    let load_condition = if has_load_feedback { load_g.abs() > stall_load_thresh } else { true };
                    gap.abs() > 0.08 && load_condition
                };
                let active_gripper_speed = active_gripper_motion
                    .map(|gm| gm.planned_speed)
                    .or(stream_window_speed)
                    .filter(|v| v.is_finite() && *v > 0.0);
                let travel_dir = active_gripper_motion
                    .and_then(|gm| {
                        let d = gm.target - gm.start;
                        (d.abs() > 1e-6).then_some(d.signum())
                    })
                    .or_else(|| (gap.abs() > 1e-3).then_some(gap.signum()))
                    .or_else(|| ((desired_g - p_meas).abs() > 1e-3).then_some((desired_g - p_meas).signum()))
                    .or_else(|| (v_cmd.abs() > 1e-4).then_some(v_cmd.signum()))
                    .or_else(|| (tau_g.abs() > 1e-3).then_some(tau_g.signum()))
                    .unwrap_or(close_dir);
                let base_window_s: f64 = if has_torque_feedback { 0.10 } else { 0.18 };
                let stall_window_s = crate::motion::gripper_progress_window_s(active_gripper_speed, base_window_s);

                if self.gripper_stall_window.step_sustained(
                    high_effort,
                    p_meas,
                    travel_dir,
                    dt,
                    stall_window_s,
                    &mut self.gripper_high_torque_ticks,
                    crate::motion::GRIPPER_CONTACT_TICKS,
                ) {
                    self.gripper_stalled = true;
                    self.gripper_stall_window.reset();
                    self.gripper_stream_contact_detector.reset();
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
            } else if self.gripper_stalled && self.gripper_force_capped && self.gripper_stream_target.is_some() {
                // torque-cap latch under a stream: held until the stream opens (intake) or ends.
                // While the stream still asks to close, the grip ramps back in from the measured
                // jaw (crate::motion::CapRamp, at most GRIPPER_STREAM_CAP_RAMP_RAD_S) until |τ| reaches
                // GRIPPER_STREAM_CAP_HOLD_FRACTION of the cap, then holds; a sample over the cap
                // latches at the measured jaw again (branch above).
                self.gripper_high_torque_ticks = 0;
                self.gripper_low_torque_ticks = 0;
                self.gripper_stall_window.reset();
                let stream_g = self.gripper_stream_target.unwrap_or(p_cmd);
                let want = (stream_g - p_cmd) * close_dir;
                let hold_nm = force_cap.map(|c| GRIPPER_STREAM_CAP_HOLD_FRACTION * c);
                if let Some(hold_nm) = hold_nm.filter(|_| has_torque_feedback && self.is_torqued && want > 1e-4 && p_cmd.is_finite()
                    && self.safety.state().is_operational()) {
                    let ramp = self.gripper_cap_ramp.get_or_insert_with(crate::motion::CapRamp::unknown);
                    let step = ramp.step(tau_g.abs(), hold_nm, (GRIPPER_STREAM_CAP_RAMP_RAD_S * dt as f32).min(want));
                    if step > 0.0 {
                        let mut next = p_cmd + close_dir * step;
                        if let Some(lead) = self.config.gripper_policy.as_ref().and_then(|p| p.max_lead_rad) {
                            if p_meas.is_finite() && (next - p_meas) * close_dir > lead { next = p_meas + close_dir * lead; }
                        }
                        if let Some(lim) = self.config.limits.get(g_idx) { next = next.clamp(lim.min_position, lim.max_position); }
                        if g_idx < self.commanded_positions.len() { self.commanded_positions[g_idx] = next; }
                        if g_idx < self.commanded_velocities.len() { self.commanded_velocities[g_idx] = 0.0; }
                        if g_idx < self.target_positions.len() { self.target_positions[g_idx] = next; }
                        if let Some(h) = self.gripper_motion_hold.as_mut() { h.0 = next; }
                    }
                }
            } else if self.gripper_stalled {
                self.gripper_high_torque_ticks = 0;
                self.gripper_stall_window.reset();
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
                self.gripper_stall_window.reset();
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
        let torque_feedback = self.driver.is_some() && self.driver_has_torque_feedback;
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

        // Bus loss: no fresh feedback from every joint within BUS_LOSS_TIMEOUT as observed
        // on this release's RX poll (avoids falsely latching BusLoss when an in-release
        // Step-3 `send_enable` resets post-enable feedback timestamps before aborting on E-stop).
        if self.is_torqued
            && self.driver.is_some()
            && !(self.feedback_numeric_valid && bus_feedback_fresh)
        {
            if !self.safety.has_latched_fault_kind(crate::safety::FaultKind::BusLoss) {
                self.execute_stop_in_tick(StopPolicy::BusLossFallback);
            } else if !self.safety.has_synchronized_bus_loss() {
                if let Some(detail) = self.classify_bus_loss_detail() {
                    self.safety.enrich_bus_loss_detail(detail);
                    self.notify_driver_bus_loss();
                }
            }
        }

        // Check hardware motor faults / error flags from feedback (0 heap allocs on RT).
        if self.is_torqued {
            let hw_fault_status = self.read_driver_for_tick().and_then(|driver| {
                if driver.has_hardware_fault() {
                    let hw_err = if !self.safety.has_latched_fault_code(FaultCode::HardwareFault)
                        || !self.safety.has_latched_fault_kind(crate::safety::FaultKind::Other)
                    {
                        driver.check_hardware_status().err()
                    } else {
                        None
                    };
                    Some(hw_err)
                } else {
                    None
                }
            });
            if let Some(hw_err) = hw_fault_status {
                if !self.safety.has_latched_fault_kind(crate::safety::FaultKind::Other) {
                    if let Some(ref e) = hw_err {
                        tracing::error!("Hardware fault detected on driver: {}", e);
                    }
                }
                if self.measured_positions.iter().all(|p| p.is_finite())
                    && self.measured_positions.len() == self.config.dof
                {
                    copy_into(&mut self.commanded_positions, &self.measured_positions);
                }
                self.pending_normal_unload = false;
                self.unload_settle = None;
                self.active_trajectory = None;
                self.release_servo_target();
                self.servo_settled_ticks.fill(0);
                if let Some((_, old)) = self.servo_ws_min_depths.take() {
                    if old.is_heap() && self.ws_eval_bufs.min_depths_spare.is_none() {
                        self.ws_eval_bufs.min_depths_spare = Some(old);
                    } else if old.is_heap()
                        && self.rt_executor_owned
                        && self.pending_retired_stops.len() < self.pending_retired_stops.capacity()
                    {
                        self.pending_retired_stops.push_back(crate::rt_core::urgent::RetiredStopPayloads {
                            servo_ws_min_depths: Some((0, old)),
                            ..Default::default()
                        });
                    }
                }
                self.tracking.invalidate();
                copy_into(&mut self.target_positions, &self.commanded_positions);
                self.commanded_velocities.fill(0.0);
                if !self.safety.has_latched_fault_code(FaultCode::HardwareFault) {
                    self.safety.latch(
                        crate::safety::FaultKind::Other,
                        crate::safety::FaultDetail::formatted(FaultCode::HardwareFault, |buf| {
                            if let Some(ref e) = hw_err {
                                e.write_bounded(buf);
                            } else {
                                buf.push_str("Hardware motor fault");
                            }
                        }),
                    );
                }
            }
        }

        // 0.3 Gravity compensation: exits (to a hold) on any safety event before the
        // watchdog below may start a park, which then runs from position mode.
        self.gravity_comp_tick(dt);

        // 1. Check lease watchdog (skip during autonomous stow/unload or un-armed Idle/Booting)
        if self.lease_enabled
            && self.gravity_comp.is_none()
            && self.is_torqued
            && !matches!(self.safety.state(), SafetyState::Booting | SafetyState::Idle)
            && !self.pending_normal_unload
            && self.watchdog_park_pending.is_none()
            && !self.safety.has_latched_fault_kind(crate::safety::FaultKind::WatchdogTimeout)
            && self.last_heartbeat.elapsed() > self.lease_timeout
        {
            #[cfg(feature = "policy-runtime")]
            if self.policy_run.is_some() {
                self.terminate_policy_run_static("WATCHDOG_TIMEOUT", "failed", Some("WATCHDOG_TIMEOUT"), Some("Lease watchdog expired"));
            }
            // Watchdog expired with torque on. When the arm is healthy and has a resting
            // pose, run the normal unload (park at home, settle, torque off). Otherwise
            // rigid lock at the current position and latch the fault, keeping torque on.
            match self.watchdog_park_blocker_bounded() {
                None => {
                    self.watchdog_park_blocked_reason = None;
                    if self.rt_executor_owned || self.defer_watchdog_park_plan {
                        // Never plan on `armd-control` or under `SupervisorLock`: hold at the
                        // measured position meanwhile and hand off planning to the off-thread
                        // worker (`install_watchdog_park`, zero alloc/free on RT).
                        if !self.rt_executor_owned {
                            tracing::warn!("Watchdog: lease expired while torqued, holding at measured pose for off-thread park plan");
                        }
                        self.release_servo_target();
                        self.servo_settled_ticks.fill(0);
                        self.retire_servo_ws_min_depths();
                        self.servo_stream_active = false;
                        self.retire_motion_payloads();
                        self.pending_deferred_done = None;
                        self.end_gripper_stream();
                        let waiting_mit_handback = if self.mit_active() {
                            self.mit.park_after_hand_back = true;
                            self.mit.deferred_park_off_owner = true;
                            true
                        } else {
                            self.commanded_velocities.fill(0.0);
                            if self.measured_positions.len() == self.config.dof
                                && self.measured_positions.iter().all(|p| p.is_finite())
                            {
                                copy_into(&mut self.commanded_positions, &self.measured_positions);
                            }
                            copy_into(&mut self.target_positions, &self.commanded_positions);
                            false
                        };
                        self.watchdog_park_seq = self.watchdog_park_seq.wrapping_add(1).max(1);
                        let generation = self.watchdog_park_seq;
                        self.watchdog_park_pending = Some(WatchdogParkPending {
                            generation,
                            attempt: 0,
                            epoch: self.safety.epoch_id(),
                            workspace_revision: self.workspace_revision,
                            motion_config_revision: self.motion_config_revision,
                            waiting_mit_handback,
                        });
                        let budget = self.watchdog_park_timeout();
                        self.watchdog_park = Some(WatchdogPark { elapsed_s: 0.0, budget_s: budget.as_secs_f64(), grace_granted: false });
                    } else {
                        tracing::warn!("Watchdog: lease expired while torqued, parking");
                        self.apply_stop_policy(StopPolicy::NormalUnload);
                        if self.pending_normal_unload {
                            let budget = self.watchdog_park_timeout();
                            tracing::info!("Watchdog park budget {:.2} s (planned {:.2} s)", budget.as_secs_f64(), self.unload_planned_duration.unwrap_or(0.0));
                            self.watchdog_park = Some(WatchdogPark { elapsed_s: 0.0, budget_s: budget.as_secs_f64(), grace_granted: false });
                        } else {
                            let reason = crate::safety::BoundedText::new("watchdog park could not be planned");
                            self.watchdog_park_blocked_reason = Some(crate::model::HolderLabel::new(reason.as_str()));
                            self.safety.log_watchdog_park_blocked(reason);
                            self.latch_watchdog_fault();
                        }
                    }
                }
                Some(reason) => {
                    self.watchdog_park_blocked_reason = Some(crate::model::HolderLabel::new(reason.as_str()));
                    if !self.safety.has_latched_fault_kind(crate::safety::FaultKind::WatchdogTimeout) {
                        self.safety.log_watchdog_park_blocked(reason);
                    }
                    self.latch_watchdog_fault();
                }
            }
        }

        // Watchdog park supervision: a park that stops without completing, or runs past
        // its budget, falls back to the hold + Fault(WatchdogTimeout) behaviour.
        self.supervise_watchdog_park(dt);

        // Thermal protection remains active while stopped or shutdown is unconfirmed.
        self.check_thermal_protection(false);
        // (a torque transition in flight confirms its own disable: `service_torque_transition`;
        // under RT executor ownership, Step 2 `service_background_disable` is the sole retry and
        // exhaustion state machine, so keep the 250 ms periodic fallback strictly legacy-only).
        // Retries pass the original `DisableKind` (`started_at`) back into the driver transmit
        // choke point, which enforces Rule A′ and the single 1 s total request lifetime at send time.
        if !self.rt_executor_owned
            && !(self.last_tick_torque_ws_direction.is_some()
                && !self.is_protective_disable_active())
            && !self.service_legacy_pending_disable()
            && !self.confirm_background_disable_if_ready()
            && self
                .last_disable_attempt
                .is_none_or(|t| t.elapsed() >= Duration::from_millis(250))
        {
            if let Some(expected_stop_gen) = self.active_disable_client_park {
                if self.client_park_stop_invalidated(expected_stop_gen) {
                    self.clear_active_disable_retry_state();
                }
            } else {
                let kind = self.active_disable_kind.unwrap_or_else(|| {
                    crate::driver::DisableKind::DisableRequested {
                        started_at: self.driver_clock_now(),
                    }
                });
                if let Err(e) = self.request_disable_bounded(kind) {
                    if !e.refused_by_chokepoint && !self.safety.deferred_logging() {
                        tracing::warn!("Torque shutdown remains unconfirmed: {}", e.message.as_str());
                    }
                }
            }
        }
        // Detect sustained physical tracking error, never compare against stale samples.
        if self.driver.is_some()
            && self.is_torqued
            && !self.disable_pending
            && !matches!(self.safety.state(), SafetyState::Booting | SafetyState::Idle)
            && self.hardware_feedback_fresh(Duration::from_millis(100))
        {
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
                    copy_into(&mut self.commanded_positions, &self.measured_positions);
                    copy_into(&mut self.target_positions, &self.measured_positions);
                    self.commanded_velocities.fill(0.0);
                    self.commanded_accelerations.fill(0.0);
                    self.safety.latch(
                        crate::safety::FaultKind::Other,
                        crate::safety::FaultDetail::formatted(FaultCode::FollowingError, |buf| {
                            let _ = core::fmt::write(
                                buf,
                                format_args!(
                                    "Persistent joint tracking error: joint {} cmd {cmd:.3} meas {meas:.3} rad",
                                    joint + 1
                                ),
                            );
                        }),
                    );
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
        // Pre-step deadline check ONLY to avoid advancing the reference when a macro parent or
        // child deadline has already elapsed before Step 3.
        if self.macro_run.is_some() {
            let pre_step_now = now_fn();
            if !self.enforce_macro_deadlines(pre_step_now) {
                self.step_macro_contact();
            }
        }
        // Compute effective_cutoff = min(control-release deadline, active action/macro parent &
        // child deadlines) ONCE for this tick and pass it as THE deadline to the driver TX choke
        // point. If a deadline elapses anywhere during Step 3, tracking, or the driver lock/TX
        // critical section, the TX choke point refuses writes past the cutoff and the next tick holds.
        let (output_tx_cap_outcome, tx_cutoff) = self.install_effective_tx_cutoff(release_deadline, now_fn);
        let applied_output_tx_cap = output_tx_cap_outcome == DriverTxCapOutcome::Installed;
        let output_tx_cap_expired = output_tx_cap_outcome == DriverTxCapOutcome::Expired;
        self.active_tx_cutoff = tx_cutoff;
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
        let (torque_feedback, load_feedback) = if self.driver.is_some() {
            (self.driver_has_torque_feedback, self.driver_has_load_feedback)
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
        let vel_caps_before_step = if tracking_enabled {
            self.active_trajectory.as_ref().and_then(|a| a.motion_vel_caps)
        } else {
            None
        };
        let gripper_idx_for_ceiling = self.config.gripper_index();
        // a gripper action that ended since the last tick without reporting done (lease loss,
        // fault, cancel), or one still installed while the arm is stopped: hold the measured jaw;
        // one that reported done short of its command (stopped on something) and is not holding
        // an object: the same, so the leading command does not keep pressing
        let grip_active = self.gripper_reference_active();
        if self.macro_run.as_ref().is_some_and(|m| m.j7_intent_only) && !grip_active {
            self.j7_admitted_stamp = None;
        }
        let lead = self.config.gripper_index()
            .and_then(|g| Some((self.commanded_positions.get(g)? - self.measured_positions.get(g)?).abs()))
            .filter(|l| l.is_finite()).unwrap_or(0.0);
        let ended = self.gripper_ref_prev && !grip_active;
        if ((ended && !self.gripper_ref_done)
            || (ended && !self.gripper_ref_holding && lead > GRIPPER_END_LEAD_RAD)
            || (grip_active && !self.safety.state().is_operational()))
            && (!self.e_stop_latched.load(Ordering::SeqCst) || feedback_fresh)
        {
            self.hold_gripper_at_measured();
        }
        self.gripper_ref_prev = grip_active;
        self.gripper_ref_done = false;
        self.gripper_ref_holding = false;
        let mut grip_learned: Option<(Option<u64>, crate::gripper_baseline::MoveRecord)> = None;
        let cap_k = self.gripper_cap_ramp.map(|r| r.k_est);
        let mut grip_contact: Option<crate::gripper_baseline::ContactInfo> = None;
        let j7_pf_stale_before_step = self.is_pos_force_j7_stale();
        let j7_pf_freeze_pos = if j7_pf_stale_before_step {
            self.pos_force_j7_freeze_position()
        } else {
            None
        };
        let last_action_progress = self.last_action.as_ref().map_or(0.0, |a| a.progress);
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
                Reference::Gripper(grip) => {
                    if j7_pf_stale_before_step {
                        let g = grip.gripper_index;
                        step_buf.positions.clear();
                        step_buf.positions.extend_from_slice(&self.commanded_positions);
                        step_buf.velocities.clear();
                        step_buf.velocities.resize(self.commanded_velocities.len(), 0.0);
                        if let Some(fp) = j7_pf_freeze_pos {
                            if g < step_buf.positions.len() {
                                step_buf.positions[g] = fp;
                            }
                        }
                        step_buf.progress = last_action_progress;
                        step_buf.clear_gripper_stall = false;
                        step_buf.status = StepStatus::Failed {
                            code: "J7_FEEDBACK_STALE",
                            message: crate::motion::StepErrorText::new(
                                "Gripper (J7) position-force feedback became stale (> 100 ms)",
                            ),
                        };
                        Stepped::Action(std::mem::take(&mut step_buf))
                    } else {
                        grip.set_cap_stiffness(cap_k);
                        grip.step_into(&ctx, &mut step_buf);
                        grip_learned = grip.take_learning().map(|r| (active.action_id, r));
                        if let Some(c) = grip.last_contact { grip_contact = Some(c); }
                        Stepped::Action(std::mem::take(&mut step_buf))
                    }
                }
                Reference::Replay(replay) => {
                    if let Some(g) = replay.gripper_idx.filter(|_| j7_pf_stale_before_step && replay.gripper) {
                        step_buf.positions.clear();
                        step_buf.positions.extend_from_slice(&self.commanded_positions);
                        step_buf.velocities.clear();
                        step_buf.velocities.resize(self.commanded_velocities.len(), 0.0);
                        if let Some(fp) = j7_pf_freeze_pos {
                            if g < step_buf.positions.len() {
                                step_buf.positions[g] = fp;
                            }
                        }
                        step_buf.progress = last_action_progress;
                        step_buf.clear_gripper_stall = false;
                        step_buf.status = StepStatus::Failed {
                            code: "J7_FEEDBACK_STALE",
                            message: crate::motion::StepErrorText::new(
                                "Gripper (J7) position-force feedback became stale (> 100 ms)",
                            ),
                        };
                        Stepped::Replay(std::mem::take(&mut step_buf), replay.phase_name(), replay.t_s())
                    } else {
                        replay.step_into(&ctx, &mut step_buf);
                        Stepped::Replay(std::mem::take(&mut step_buf), replay.phase_name(), replay.t_s())
                    }
                }
            };
            // Keep the persistent gripper hold ceiling synchronized with `GripperMotion` (including
            // contact speed reduction) so completion and subsequent hold ticks preserve it.
            if let Reference::Gripper(gm) = &active.reference {
                if gripper_idx_for_ceiling == Some(gm.gripper_index) {
                    self.gripper_hold_vel_ceiling = Some(gm.wire_velocity_ceiling());
                }
            }
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
        if let Some(c) = grip_contact { self.gripper_last_contact = Some(c); }
        if let Some((action_id, rec)) = grip_learned {
            if let Some(cal) = self.gripper_calibration.as_mut().filter(|c| Some(c.id) == action_id) {
                // a calibration stroke: checked and learned by its orchestrator (capacity reserved)
                if cal.records.len() < cal.records.capacity() { cal.records.push(rec); }
            } else if self.gripper_baseline_pending.len() < self.gripper_baseline_pending.capacity().min(GRIPPER_BASELINE_PENDING_MAX) {
                // learned off the tick (gripper_calib::spawn_baseline_persister); capacity is reserved
                self.gripper_baseline_pending.push(rec);
            } else if self.rt_executor_owned {
                self.recycle_move_samples_rt(rec.samples);
            }
        }
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
                if let StepStatus::Done { holding, .. } = &out.status {
                    self.gripper_ref_done = true;
                    self.gripper_ref_holding = *holding == Some(true);
                }
                deferred_done = self.apply_action_step(action_id, out);
            }
            Some((action_id, Stepped::Arm(mut out))) => {
                reference_stepped = true;
                self.servo_stream_active = false;
                self.gripper_hold_filter(&mut out.positions, &mut out.velocities);
                if self.check_static_load_admission_commit(&out.positions).is_err() {
                    self.scratch.step_out.positions = out.positions;
                    self.scratch.step_out.velocities = out.velocities;
                    self.abort_planned_motion_static_load_rt(action_id);
                } else {
                    deferred_done = self.apply_action_step(action_id, out);
                }
            }
            Some((action_id, Stepped::Replay(mut out, phase, t_s))) => {
                reference_stepped = true;
                self.servo_stream_active = false;
                self.gripper_hold_filter(&mut out.positions, &mut out.velocities);
                if self.check_static_load_admission_commit(&out.positions).is_err() {
                    self.scratch.step_out.positions = out.positions;
                    self.scratch.step_out.velocities = out.velocities;
                    self.abort_planned_motion_static_load_rt(action_id);
                } else {
                    if let (Some(id), Some(a)) = (action_id, self.last_action.as_mut()) {
                        if a.id == id && a.is_running() {
                            a.phase = Some(phase.into());
                            a.t_s = Some(t_s as f32);
                        }
                    }
                    deferred_done = self.apply_action_step(action_id, out);
                }
            }
            Some((action_id, Stepped::Quintic(finished, progress))) => {
                reference_stepped = true;
                self.servo_stream_active = false;
                self.gripper_hold_filter(&mut point_buf.positions, &mut point_buf.velocities);
                if self.check_static_load_admission_commit(&point_buf.positions).is_err() {
                    self.abort_planned_motion_static_load_rt(action_id);
                } else {
                    // swap: the previous command's memory goes back to the scratch
                    std::mem::swap(&mut self.commanded_positions, &mut point_buf.positions);
                    std::mem::swap(&mut self.commanded_velocities, &mut point_buf.velocities);
                    let macro_sub = self.macro_run.as_ref().is_some_and(|m| Some(m.id) == action_id);
                    if let (Some(id), Some(a)) = (action_id, self.last_action.as_mut()) {
                        if a.id == id && a.is_running() && !macro_sub { a.progress = progress; }
                    }
                    if finished && approach_started.is_none() {
                        self.replace_active_trajectory(None);
                        self.commanded_velocities.fill(0.0);
                        if !self.pending_normal_unload {
                            self.safety.mark_motion_finished();
                            if let Some(id) = action_id { self.quintic_settle = Some((id, Settle::default())); }
                        }
                    }
                }
            }
        }

        self.scratch.point = point_buf;
        if self.macro_run.as_ref().is_some_and(|m| m.j7_intent_only) && !self.gripper_reference_active() {
            self.j7_admitted_stamp = None;
        }
        // not stepped (no Path / Gripper / Replay reference): the buffer goes back unused
        if self.scratch.step_out.positions.capacity() < step_buf.positions.capacity() { self.scratch.step_out = step_buf; }
        if let Some(caps) = caps_before_step {
            if self.active_trajectory.is_none() {
                self.tracking.retained_motion_limits = Some(caps);
            }
        }
        if let Some(vcaps) = vel_caps_before_step {
            if self.active_trajectory.is_none() {
                self.tracking.retained_motion_vel_caps = Some(vcaps);
            }
        }

        // Step-5 owner output-boundary revocation recheck (Linearization 2): if `PolicyAuthorization`
        // was bumped after Step 4 consumption or mid-tick before servo stepping, brake immediately
        // so zero policy-derived motion is emitted.
        #[cfg(feature = "policy-runtime")]
        self.recheck_live_policy_authorization_rt();

        // 3.1 Servo streaming (park release / unload settle runs AFTER output acceptance below
        //     so park never advances or cuts torque on a deferred or rejected release).
        if !(self.pending_normal_unload && self.active_trajectory.is_none()) && self.servo_target.is_some() {
            if j7_pf_stale_before_step {
                if let Some(g) = self.config.gripper_index() {
                    if let Some(fp) = j7_pf_freeze_pos {
                        if let Some(st) = self.servo_target.as_mut().filter(|s| s.len() > g) {
                            st[g] = fp;
                        }
                        if g < self.target_positions.len() {
                            self.target_positions[g] = fp;
                        }
                    }
                    self.gripper_stream_target = None;
                }
            }
            // the target, copied into the scratch (the stream may replace it meanwhile)
            let mut target_buf = std::mem::take(&mut self.scratch.servo_target);
            copy_into(&mut target_buf, self.servo_target.as_deref().unwrap_or(&[]));
            let target = &target_buf;
            // Check continuous motion frame timeout:
            // If > 200ms without a new pose frame, clear servo_target to begin controlled smooth deceleration
            if self.last_motion_frame.elapsed() > self.motion_frame_timeout {
                self.release_servo_target();
                self.servo_settled_ticks.fill(0);
                if self.rt_executor_owned {
                    self.fail_live_policy_from_control_static(
                        "TARGET_TIMEOUT",
                        "No policy target within motion frame timeout; the control loop dropped the target",
                    );
                } else {
                    self.fail_live_policy_from_control("TARGET_TIMEOUT",
                        format!("No policy target for {} ms; the control loop dropped the target", self.motion_frame_timeout.as_millis()));
                }
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
                    if !self.rt_executor_owned {
                        tracing::warn!(
                            "Discarding dirty servo target frame: instantaneous jump exceeds 0.35 rad relative to commanded_positions"
                        );
                    }
                    self.release_servo_target();
                    self.servo_settled_ticks.fill(0);
                    if self.rt_executor_owned {
                        self.fail_live_policy_from_control_static(
                            "SERVO_TARGET_DISCARDED",
                            "The control loop discarded the policy target: it leads the commanded position by more than 0.35 rad",
                        );
                    } else {
                        self.fail_live_policy_from_control("SERVO_TARGET_DISCARDED",
                            "The control loop discarded the policy target: it leads the commanded position by more than 0.35 rad".into());
                    }
                } else {
                    reference_stepped = true;
                    if let Some(g) = gripper_idx {
                        if target.get(g).is_some_and(|&tgt| (tgt - self.commanded_positions[g]).abs() > 1e-4) {
                            self.gripper_hold_vel_ceiling = None;
                        }
                    }
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
                    } else {
                        match self.servo_step_workspace_rt(&next_positions, "servo_step") {
                            Err(f) => self.enter_servo_keepout_hold_rt(f),
                            Ok(pending_ws) => {
                                if self.check_static_load_admission_commit(&next_positions).is_err() {
                                    self.enter_servo_static_load_hold_rt();
                                } else {
                                    if let Some(n) = pending_ws {
                                        self.commit_servo_ws_min_depths_rt(n);
                                    }
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
                                            self.clear_servo_ws_min_depths_rt();
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
            } else {
                let ws_res = if braking {
                    self.servo_step_workspace_rt(&next_positions, "braking")
                } else {
                    Ok(None)
                };
                match ws_res {
                    Err(f) => self.enter_servo_keepout_hold_rt(f),
                    Ok(pending_ws) => {
                        if braking && self.check_static_load_admission_commit(&next_positions).is_err() {
                            self.enter_servo_static_load_hold_rt();
                        } else {
                            if let Some(n) = pending_ws {
                                self.commit_servo_ws_min_depths_rt(n);
                            }
                            // swap: the previous command's memory goes back to the scratch
                            std::mem::swap(&mut self.commanded_positions, &mut next_positions);
                            std::mem::swap(&mut self.commanded_velocities, &mut next_velocities);
                            copy_into(&mut self.target_positions, &self.commanded_positions);
                            if self.commanded_velocities.iter().all(|&v| v == 0.0) {
                                self.safety.mark_motion_finished();
                                self.clear_servo_ws_min_depths_rt();
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

        let feedback_fresh = self.feedback_numeric_valid && bus_feedback_fresh;
        if self.driver.is_some() && self.is_torqued {
            if !feedback_fresh {
                self.output_suspended = true;
                self.tracking.invalidate();
            } else if self.output_suspended {
                self.stop_motion();
                self.tracking.invalidate();
                copy_into(&mut self.commanded_positions, &self.measured_positions);
                copy_into(&mut self.target_positions, &self.measured_positions);
                self.commanded_velocities.fill(0.0);
                self.commanded_accelerations.fill(0.0);
                self.output_suspended = false;
                let fault_detail = match self.classify_bus_loss_detail() {
                    Some(d) => crate::safety::FaultDetail::BusLossAnnotated(d),
                    None => crate::safety::FaultDetail::BusLoss,
                };
                self.safety.latch(crate::safety::FaultKind::BusLoss, fault_detail);
                self.notify_driver_bus_loss();
            }
        }
        // MIT tracking (Phase 2): deadline supervision, park hand back at rest, handover at rest.
        // No-ops while MIT is not requested.
        self.mit_supervise();
        if self.mit.revoke_pending {
            // a refused rearm revoked MIT: hand back once the brake reached rest
            let at_rest = self.commanded_velocities.iter().all(|v| *v == 0.0);
            if !self.mit_active() || self.safety.state().is_stopped() {
                self.mit.revoke_pending = false;
            } else if at_rest {
                #[cfg(not(target_arch = "wasm32"))]
                let handback_done = if self.driver.is_some()
                    && (self.rt_executor_owned || self.driver_supports_nonblocking_mode_switch())
                {
                    if !self.mode_switch.is_running() {
                        let now = now_fn();
                        let _ = self.mit_hand_back_start_rt(
                            MitHandBackReason::AuthorizationRevoked,
                            self.safety.epoch_id(),
                            now + Duration::from_secs(2),
                            crate::rt_core::broker::ClientId(0),
                            None,
                            &mut None,
                            now,
                        );
                    }
                    false
                } else {
                    self.mit_hand_back("authorization revoked").is_ok()
                };
                #[cfg(target_arch = "wasm32")]
                let handback_done = self.mit_hand_back("authorization revoked").is_ok();
                if handback_done {
                    self.mit.revoke_pending = false;
                }
            }
        }
        if self.mit.park_after_hand_back {
            // a park waits for rest and the confirmed hand back, then starts (or wakes off-owner continuation)
            let at_rest = self.commanded_velocities.iter().all(|v| *v == 0.0);
            if !self.mit_active() || self.safety.state().is_stopped() {
                self.mit.park_after_hand_back = false;
                self.mit.deferred_park_off_owner = false;
                if let Some(c) = self.park_continuation.take() {
                    let mut msg = self.take_action_msg_buf(c.action_id, 96);
                    msg.clear();
                    msg.push_str("Park cancelled: MIT tracking lost or a fault");
                    self.finish_action(c.action_id, "failed", Some("UNLOAD_FAILED"), Some(msg), None, None, None);
                    self.notify_park_continuation();
                }
                if self.watchdog_park_pending.take().is_some() {
                    self.watchdog_park = None;
                    if self.is_torqued && !self.safety.state().is_stopped() {
                        self.safety.log_watchdog_park_blocked(crate::safety::BoundedText::new("MIT tracking lost during watchdog park"));
                        self.latch_watchdog_fault();
                    }
                }
                if let Some(id) = self.mit.deferred_park_action.take() {
                    let mut msg = self.take_action_msg_buf(id, 96);
                    msg.clear();
                    msg.push_str("Park cancelled: MIT tracking lost or a fault");
                    self.finish_action(id, "failed", Some("UNLOAD_FAILED"), Some(msg), None, None, None);
                }
            } else if at_rest {
                #[cfg(not(target_arch = "wasm32"))]
                if self.driver.is_some()
                    && (self.rt_executor_owned || self.driver_supports_nonblocking_mode_switch())
                {
                    if !self.mode_switch.is_running() {
                        let now = now_fn();
                        let _ = self.mit_hand_back_start_rt(
                            MitHandBackReason::Park,
                            self.safety.epoch_id(),
                            now + Duration::from_secs(2),
                            crate::rt_core::broker::ClientId(0),
                            None,
                            &mut None,
                            now,
                        );
                    }
                } else if self.mit_hand_back("park").is_ok() {
                    self.mit.park_after_hand_back = false;
                    if std::mem::take(&mut self.mit.deferred_park_off_owner) {
                        if let Some(c) = self.park_continuation.as_mut() {
                            c.phase = ParkContinuationPhase::ReadyForPlan;
                        }
                        if let Some(w) = self.watchdog_park_pending.as_mut() {
                            w.waiting_mit_handback = false;
                        }
                        self.notify_park_continuation();
                    } else {
                        self.apply_stop_policy(StopPolicy::NormalUnload);
                        if let Some(id) = self.mit.deferred_park_action.take() {
                            if self.pending_normal_unload {
                                if let Some(active) = self.active_trajectory.as_mut() { active.action_id = Some(id); }
                            } else {
                                let mut msg = self.take_action_msg_buf(id, 96);
                                msg.clear();
                                msg.push_str("Park could not start after the MIT hand back");
                                self.finish_action(id, "failed", Some("UNLOAD_FAILED"), Some(msg), None, None, None);
                            }
                        }
                    }
                }
                #[cfg(target_arch = "wasm32")]
                if self.mit_hand_back("park").is_ok() {
                    self.mit.park_after_hand_back = false;
                    if std::mem::take(&mut self.mit.deferred_park_off_owner) {
                        if let Some(c) = self.park_continuation.as_mut() {
                            c.phase = ParkContinuationPhase::ReadyForPlan;
                        }
                        if let Some(w) = self.watchdog_park_pending.as_mut() {
                            w.waiting_mit_handback = false;
                        }
                        self.notify_park_continuation();
                    } else {
                        self.apply_stop_policy(StopPolicy::NormalUnload);
                        if let Some(id) = self.mit.deferred_park_action.take() {
                            if self.pending_normal_unload {
                                if let Some(active) = self.active_trajectory.as_mut() { active.action_id = Some(id); }
                            } else {
                                let mut msg = self.take_action_msg_buf(id, 96);
                                msg.clear();
                                msg.push_str("Park could not start after the MIT hand back");
                                self.finish_action(id, "failed", Some("UNLOAD_FAILED"), Some(msg), None, None, None);
                            }
                        }
                    }
                }
            }
        }
        if self.pending_normal_unload && self.mit_active() {
            #[cfg(not(target_arch = "wasm32"))]
            if self.driver.is_some()
                && (self.rt_executor_owned || self.driver_supports_nonblocking_mode_switch())
            {
                if !self.mode_switch.is_running() {
                    let now = now_fn();
                    let _ = self.mit_hand_back_start_rt(
                        MitHandBackReason::Park,
                        self.safety.epoch_id(),
                        now + Duration::from_secs(2),
                        crate::rt_core::broker::ClientId(0),
                        None,
                        &mut None,
                        now,
                    );
                }
            } else {
                let _ = self.mit_hand_back("park");
            }
            #[cfg(target_arch = "wasm32")]
            {
                let _ = self.mit_hand_back("park");
            }
        }
        self.mit_maybe_activate(
            #[cfg(not(target_arch = "wasm32"))]
            now_fn(),
        );
        let gravity_output_enabled = self.gravity_output_enabled();
        self.tick_phase(1);
        // Hold only while torque may be active and fresh valid feedback permits output.
        // Pending disable and invalid feedback suppress ordinary commands; reconnect resynchronizes first.
        let mut tick_output = TickOutput::None;
        let mut allow_output_accepted = false;
        let mut driverless_accepted = false;
        let mut gravity_exiting_or_failed = false;
        let mut gravity_driverless_ok = false;
        let mut gravity_handback_ran = false;
        self.j7_intent = None;
        // An enable transition owns the bus exclusively. A pending disable suppresses ordinary
        // motion, while stationary holds continue until a disable is accepted (`torque_intent == Off`,
        // i.e. rest gate passed and the first disable frame has been dispatched).
        let transition_owns_bus = self.torque_transition.is_some();
        #[cfg(not(target_arch = "wasm32"))]
        let mode_switch_ran_this_release = self.control_release_seq != 0
            && self.mode_switch_ran_release_seq == self.control_release_seq;
        #[cfg(target_arch = "wasm32")]
        let mode_switch_ran_this_release = false;
        #[cfg(not(target_arch = "wasm32"))]
        let safe_hold_inline_fallback = self.sessions.is_safe_hold_requested()
            && !mode_switch_ran_this_release;
        #[cfg(target_arch = "wasm32")]
        let safe_hold_inline_fallback = false;
        #[cfg(not(target_arch = "wasm32"))]
        let mode_switch_owns_bus = mode_switch_ran_this_release
            || self.sessions.is_safe_hold_requested();
        #[cfg(target_arch = "wasm32")]
        let mode_switch_owns_bus = false;
        let enable_transition_owns_bus = matches!(
            self.torque_transition,
            Some(driver_transitions::TorqueTransition::Enable(_))
        ) || mode_switch_owns_bus;
        if !self.shutdown_committed && self.is_torqued && !self.disable_pending && !transition_owns_bus && !mode_switch_owns_bus && self.gravity_comp.is_some() {
            let gc_stamp = self.current_j7_intent_stamp();
            self.j7_intent = Some(if self.safety.state().is_stopped() {
                crate::driver::PosForceIntent::stop(gc_stamp)
            } else {
                crate::driver::PosForceIntent::hold(gc_stamp)
            });
            // If a prior release completed `exit_hold_ms` (and stationary settle when tracking is
            // enabled), execute POS_VEL handback on this dedicated transition release using the
            // active release's TX pass (`set_joint_modes_in_tick`) without first spending 7 frames
            // on stiff MIT holds.
            if self.run_scheduled_gravity_comp_handback(
                #[cfg(not(target_arch = "wasm32"))]
                now_fn(),
            ) {
                gravity_handback_ran = true;
                if self.driver.is_none() {
                    driverless_accepted = true;
                }
            } else {
                // Gravity compensation (any phase): MIT targets for J1-J6. Sent without the
                // feedback-freshness gate so that an exit hold reaches the motors even when
                // feedback is late; a floating phase never outlives fresh feedback (it exits).
                gravity_exiting_or_failed = self.gravity_comp.as_ref().is_some_and(|r| {
                    matches!(
                        r.phase,
                        crate::gravity_comp::GravityCompPhase::Exiting { .. }
                            | crate::gravity_comp::GravityCompPhase::ExitFailed { .. }
                    )
                });
                if self.driver.is_none() {
                    driverless_accepted = true;
                    gravity_driverless_ok = true;
                } else if let Some(targets) = self.gravity_comp_targets_array() {
                    if self.torque_intent == TorqueIntent::On {
                        let is_hold = self.safety.state().is_stopped();
                        let output_lease_gen = if is_hold || gravity_exiting_or_failed {
                            0
                        } else {
                            self.current_active_lease_gen()
                        };
                        let fence = LiveDisconnectFenceView::new(
                            &self.e_stop_latched,
                            &self.pending_disconnect_hold,
                            &self.atomic_fenced_lease_gen,
                            self.fenced_lease_generation,
                            self.active_lease_conn_id,
                            self.active_lease.as_ref().map(|a| a.generation),
                            &self.closed_connections,
                            self.rt_closed_connections.as_deref(),
                            &self.shared_closed_bitset,
                        );
                        let rt_owned = self.rt_executor_owned;
                        #[cfg(not(target_arch = "wasm32"))]
                        let shadow_rt = &mut self.torque_shadow;
                        if let Some(d) = self.driver.as_mut() {
                            let write_fn = |driver: &mut dyn ArmDriver| {
                                let res = Self::write_gravity_comp_targets_checked(
                                    driver,
                                    &targets,
                                    is_hold,
                                    output_lease_gen,
                                    &fence,
                                );
                                #[cfg(not(target_arch = "wasm32"))]
                                shadow_rt.drain_from_driver(driver);
                                res
                            };
                            let written = if rt_owned {
                                d.try_with_mut(write_fn).ok().flatten()
                            } else {
                                d.with_mut(write_fn).flatten()
                            };
                            if let Some((out, _effective_hold)) = written {
                                tick_output = out;
                                if matches!(tick_output, TickOutput::Complete { is_hold: false, .. }) {
                                    allow_output_accepted = true;
                                }
                            }
                        }
                    }
                }
            }
        } else if !self.shutdown_committed && !enable_transition_owns_bus && output_tx_cap_expired {
            // Release TX cutoff already expired before output started: transmit 0 frames and
            // enqueue unfenced motion targets into the driver's carry-over queue without staging
            // an already-expired driver deadline.
            let is_stopped_or_failed_hold = self.safety.state().is_stopped()
                || self.enable_lifecycle == driver_transitions::EnableLifecycle::FailedHold;
            let send_to_bus = !self.disable_pending
                && !transition_owns_bus
                && self.is_torqued
                && self.torque_intent == TorqueIntent::On
                && feedback_fresh
                && self.hardware_feedback_fresh(BUS_LOSS_TIMEOUT);
            if send_to_bus && !is_stopped_or_failed_hold {
                let mut sc = std::mem::take(&mut self.scratch);
                sc.targets.clear();
                let dof = self.config.dof;
                for i in 0..dof {
                    let pos = self.commanded_positions.get(i).copied().unwrap_or(0.0);
                    let vel = self.commanded_velocities.get(i).copied();
                    sc.targets.push(JointTarget {
                        position: pos,
                        velocity: vel,
                        kp: None,
                        kd: None,
                        torque: None,
                    });
                }
                let lease_gen = self.current_active_lease_gen();
                if let Some(mut drv) = self.write_driver_for_tick() {
                    drv.tx_mut().set_output_lease_gen(lease_gen);
                    drv.defer_motion_targets_past_cutoff(&sc.targets);
                }
                self.scratch = sc;
            }
        } else if !self.shutdown_committed && !enable_transition_owns_bus && !output_tx_cap_expired {
            let tracking_active = self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled);
            let is_stopped_or_failed_hold = self.safety.state().is_stopped()
                || self.enable_lifecycle == driver_transitions::EnableLifecycle::FailedHold;
            let pending_disable_hold =
                (self.disable_pending || transition_owns_bus) && self.torque_intent == TorqueIntent::On;
            let send_to_bus = !self.disable_pending
                && !transition_owns_bus
                && self.is_torqued
                && self.torque_intent == TorqueIntent::On
                && feedback_fresh
                && self.hardware_feedback_fresh(BUS_LOSS_TIMEOUT);
            let has_fallback_ramp = send_to_bus
                && tracking_active
                && self.driver.is_some()
                && self.enable_lifecycle == driver_transitions::EnableLifecycle::Armed
                && !matches!(self.safety.state(), SafetyState::EStop)
                && (self.tracking.fallback_ramp.is_some() || self.tracking.holding_fallback_pose);

            if (is_stopped_or_failed_hold && !has_fallback_ramp) || pending_disable_hold {
                self.tracking.invalidate();
                if self.stopped_hold_eligible() {
                    if self.driver.is_some() {
                        tick_output = self.service_stopped_measured_hold(servo_dt, gravity_output_enabled);
                    }
                } else {
                    self.measured_hold.clear();
                }
            } else if send_to_bus
                && tx_cutoff.is_some_and(|c| c.drv_action_cutoff.is_some_and(|ac| ac <= c.drv_anchor))
            {
                let c = tx_cutoff.expect("checked above");
                self.tracking.invalidate();
                tick_output = TickOutput::ActionDeadlineRefused {
                    drv_now: c.drv_anchor,
                    pf_status: crate::driver::PosForceTxStatus::Unobserved,
                };
            } else if send_to_bus {
                if !is_stopped_or_failed_hold {
                    self.measured_hold.clear();
                }
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
                    if let Some(old) = self.tracking.fallback_ramp.take() {
                        self.tracking.recycle_fallback_ramp(old);
                    }
                }
                if tracking_active && self.tracking.recovery_suspended {
                    // Driver emitted an autonomous recovery write; suspend supervisor target
                    // writes until fresh post-recovery feedback confirms every joint's baseline.
                } else if tracking_active && self.driver.is_some() && self.tracking.fallback_ramp.is_some() {
                    // Rejection-fallback braking ramp, also while the rejection fault is latched.
                    // The rejected motion's caps (`FallbackRamp::limits`) stay in force, tightened
                    // by the current power governor limit; the ramp keeps its original caps (0 alloc).
                    let mut ramp = self.tracking.fallback_ramp.take().expect("checked above");
                    let mut governed = std::mem::take(&mut self.scratch.fallback_limits);
                    self.active_tracking_limits_into(servo_dt, time_scale, &mut governed);
                    let mut orig_va = [(0.0_f32, 0.0_f32); crate::model::MAX_MOTION_DOF];
                    let n_lim = ramp.limits.len().min(crate::model::MAX_MOTION_DOF);
                    for (slot, l) in orig_va[..n_lim].iter_mut().zip(&ramp.limits[..n_lim]) {
                        *slot = (l.max_velocity, l.max_acceleration);
                    }
                    for (l, g) in ramp.limits.iter_mut().zip(&governed) {
                        l.max_velocity = l.max_velocity.min(g.max_velocity);
                        l.max_acceleration = l.max_acceleration.min(g.max_acceleration);
                    }
                    self.scratch.fallback_limits = governed;
                    let mut tick_sweep_budget = MAX_KEEPOUT_EVALS_PER_TICK;
                    let mut pending = self.tracking_fallback_compute_inline(ramp, servo_dt, &mut tick_sweep_budget);
                    for (l, &(v, a)) in pending.ramp.limits[..n_lim].iter_mut().zip(&orig_va[..n_lim]) {
                        l.max_velocity = v;
                        l.max_acceleration = a;
                    }
                    tick_output = self.apply_tracking_fallback_step_inline(pending, gravity_output_enabled);
                    self.tracking.last_tick_sweep_evals =
                        MAX_KEEPOUT_EVALS_PER_TICK.saturating_sub(tick_sweep_budget);
                } else if ramp_out_handover || tracking_owned {
                    let mut active_limits = std::mem::take(&mut self.scratch.fallback_limits);
                    self.active_tracking_limits_into(servo_dt, time_scale, &mut active_limits);
                    let mut tick_sweep_budget = MAX_KEEPOUT_EVALS_PER_TICK;
                    let use_inline = !ramp_out_handover
                        && self.handover_settling_since.is_none()
                        && self.config.dof <= crate::model::MAX_MOTION_DOF;
                    let eval_res = if use_inline {
                        self.compute_position_lead_output_inline(
                            servo_dt,
                            false,
                            &active_limits,
                            &mut tick_sweep_budget,
                        )
                    } else if ramp_out_handover && self.config.dof <= crate::model::MAX_MOTION_DOF {
                        self.compute_lead_ramp_out_inline(
                            servo_dt,
                            &active_limits,
                            self.pending_normal_unload,
                            &mut tick_sweep_budget,
                        )
                    } else if ramp_out_handover {
                        self.compute_lead_ramp_out_with_budget(
                            servo_dt,
                            &active_limits,
                            self.pending_normal_unload,
                            &mut tick_sweep_budget,
                        )
                        .map(|o| {
                            let dof = o.positions.len().min(crate::model::MAX_MOTION_DOF);
                            let mut positions = [0.0_f32; crate::model::MAX_MOTION_DOF];
                            let mut velocities = [0.0_f32; crate::model::MAX_MOTION_DOF];
                            positions[..dof].copy_from_slice(&o.positions[..dof]);
                            velocities[..dof].copy_from_slice(&o.velocities[..dof]);
                            let ws_min_depths_rev = o.ws_min_depths.as_ref().map(|(rev, sb)| {
                                let s = sb.as_slice();
                                if self.ws_eval_bufs.tracking_staged_ws.len() < s.len() {
                                    self.ws_eval_bufs.tracking_staged_ws.resize(s.len(), 0.0);
                                }
                                self.ws_eval_bufs.tracking_staged_ws[..s.len()].copy_from_slice(s);
                                (*rev, s.len())
                            });
                            let ws_keepout_min_depths_rev = o.ws_keepout_min_depths.as_ref().map(|(rev, sb)| {
                                let s = sb.as_slice();
                                if self.ws_eval_bufs.tracking_staged_ws.len() < s.len() {
                                    self.ws_eval_bufs.tracking_staged_ws.resize(s.len(), 0.0);
                                }
                                self.ws_eval_bufs.tracking_staged_ws[..s.len()].copy_from_slice(s);
                                (*rev, s.len())
                            });
                            crate::tracking::TrackingOutputInline {
                                positions,
                                velocities,
                                dof,
                                multiplier: o.multiplier,
                                withdrawn: o.withdrawn,
                                sweep_evals_used: o.sweep_evals_used,
                                ws_min_depths_rev,
                                ws_keepout_min_depths_rev,
                            }
                        })
                        .map_err(crate::tracking::TrackingRejectRt::from_tracking_reject)
                    } else {
                        self.compute_position_lead_output_inline(
                            servo_dt,
                            true,
                            &active_limits,
                            &mut tick_sweep_budget,
                        )
                    };
                    match eval_res {
                        Ok(out) => {
                            self.scratch.fallback_limits = active_limits;
                            if self.check_static_load_tracking_output(&out.positions[..out.dof], true).is_err() {
                                // Tracking lead or ramp-out output would worsen static load above
                                // cap or under active static overload restriction: do not emit
                                // `out.positions`; halt worsening motion and emit a safe
                                // hold pose. Also cancel any deferred completion captured earlier
                                // on this tick (e.g. `StepStatus::Done` when `active_trajectory`
                                // was already retired) or pending from a prior tick so a refused
                                // release can never report an action as completed.
                                let deferred_this = deferred_done.take();
                                let deferred_prev = self.pending_deferred_done.take();
                                let had_deferred_done = deferred_this.is_some() || deferred_prev.is_some();
                                let extract_id = |d: DeferredActionDone| match d.target {
                                    DeferredCompletionTarget::Action(id)
                                    | DeferredCompletionTarget::MacroSub { action_id: id, .. }
                                    | DeferredCompletionTarget::MacroFinish { action_id: id, .. } => Some(id),
                                    DeferredCompletionTarget::ParkRelease(_)
                                    | DeferredCompletionTarget::Anonymous => None,
                                };
                                let deferred_action_id = deferred_this
                                    .and_then(extract_id)
                                    .or_else(|| deferred_prev.and_then(extract_id));
                                let quintic_settle_id = self.quintic_settle.as_ref().map(|(id, _)| *id);
                                if let Some(action_id) = self
                                    .active_trajectory
                                    .as_ref()
                                    .and_then(|a| a.action_id)
                                    .or(deferred_action_id)
                                    .or(quintic_settle_id)
                                {
                                    self.abort_planned_motion_static_load_rt(Some(action_id));
                                } else if self.active_trajectory.is_some()
                                    || had_deferred_done
                                    || self.pending_normal_unload
                                {
                                    self.abort_planned_motion_static_load_rt(None);
                                }
                                if self.servo_target.is_some() || self.servo_stream_active {
                                    self.enter_servo_static_load_hold_rt();
                                }
                                // If nominal `commanded_positions` is worse than the last issued
                                // wire pose, stay on the last issued arm pose; otherwise keep the
                                // safe nominal pose.
                                let nominal_worse = self
                                    .tracking
                                    .last_issued_positions
                                    .as_ref()
                                    .is_some_and(|v| v.len() == self.commanded_positions.len())
                                    && {
                                        let mut nom_buf = [0.0f32; crate::model::MAX_MOTION_DOF];
                                        let n = self.commanded_positions.len().min(nom_buf.len());
                                        nom_buf[..n].copy_from_slice(&self.commanded_positions[..n]);
                                        self.check_static_load_tracking_output(&nom_buf[..n], false).is_err()
                                    };
                                if nominal_worse {
                                    if let Some(prev) = self.tracking.last_issued_positions.as_deref() {
                                        let gripper_idx = self.config.gripper_index();
                                        for (i, (c, &p)) in self
                                            .commanded_positions
                                            .iter_mut()
                                            .zip(prev.iter())
                                            .enumerate()
                                        {
                                            if Some(i) != gripper_idx {
                                                *c = p;
                                            }
                                        }
                                    }
                                }
                                copy_into(&mut self.target_positions, &self.commanded_positions);
                                self.commanded_velocities.fill(0.0);
                                self.commanded_accelerations.fill(0.0);
                                self.tracking.invalidate();
                                self.note_motion_limit(MotionLimitReason::Power, None);
                                let mut targets = std::mem::take(&mut self.scratch.targets);
                                targets.clear();
                                targets.extend(
                                    self.commanded_positions
                                        .iter()
                                        .enumerate()
                                        .map(|(i, &p)| JointTarget {
                                            position: p,
                                            velocity: Some(0.0),
                                            torque: if gravity_output_enabled {
                                                self.gravity_torques_ramped.get(i).copied()
                                            } else {
                                                Some(0.0)
                                            },
                                            kp: None,
                                            kd: None,
                                        }),
                                );
                                self.scratch.targets = targets;
                                tick_output = self.write_output_scratch(servo_dt);
                            } else {
                                let mut targets = std::mem::take(&mut self.scratch.targets);
                                targets.clear();
                                targets.extend(
                                    out.positions[..out.dof]
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
                                        }),
                                );
                                self.scratch.targets = targets;
                                tick_output = self.write_output_scratch(servo_dt);
                                match &tick_output {
                                    TickOutput::Complete { .. } => {
                                        let ws_slice = out
                                            .ws_min_depths_rev
                                            .map(|(rev, n)| (rev, &self.ws_eval_bufs.tracking_staged_ws[..n]));
                                        let keepout_slice = out
                                            .ws_keepout_min_depths_rev
                                            .map(|(rev, n)| (rev, &self.ws_eval_bufs.tracking_staged_ws[..n]));
                                        self.tracking.record_issued_and_nominal_inline(
                                            &out.positions[..out.dof],
                                            &out.velocities[..out.dof],
                                            &self.commanded_positions,
                                            servo_dt,
                                            ws_slice,
                                            keepout_slice,
                                        );
                                        if out.withdrawn {
                                            self.tracking.withdrawal_ticks += 1;
                                        }
                                        allow_output_accepted = true;
                                    }
                                    TickOutput::Deferred { .. }
                                    | TickOutput::Expired { .. }
                                    | TickOutput::Error { .. }
                                    | TickOutput::EstopInhibited { .. }
                                    | TickOutput::LeaseFenced
                                    | TickOutput::ActionDeadlineRefused { .. }
                                    | TickOutput::None => {
                                        self.tracking.invalidate();
                                    }
                                }
                            }
                        }
                        Err(reject) => {
                            let was_streaming = self.servo_target.is_some() || self.servo_stream_active;
                            // Rejection fallback: evaluated from the intact history and park
                            // state BEFORE the hold below invalidates them. Reuses preallocated
                            // `TrackingRuntime` spare buffers (0 RT allocs) and returns
                            // `active_limits` directly to `self.scratch.fallback_limits`.
                            let mut hold_buf = [0.0f32; crate::model::MAX_MOTION_DOF];
                            let hold_len = self.measured_hold_pose_into(&mut hold_buf);
                            let ramp = self.tracking.build_fallback_ramp(
                                &hold_buf[..hold_len],
                                self.pending_normal_unload,
                                &active_limits,
                            );
                            self.scratch.fallback_limits = active_limits;
                            let pending = self.tracking_fallback_compute_inline(ramp, servo_dt, &mut tick_sweep_budget);
                            match reject {
                                crate::tracking::TrackingRejectRt::Workspace(f) if was_streaming => {
                                    self.enter_servo_keepout_hold_rt(f);
                                }
                                crate::tracking::TrackingRejectRt::Workspace(f) => {
                                    if !self.safety.deferred_logging() {
                                        tracing::warn!(
                                            "Joint tracking output violated workspace ({} at {}): terminating motion",
                                            f.code(),
                                            f.phase
                                        );
                                    }
                                    self.hold_measured_pose();
                                    self.drop_macro_run("failed", Some("WORKSPACE_VIOLATION"));
                                    self.safety.latch(
                                        crate::safety::FaultKind::Other,
                                        crate::safety::FaultDetail::formatted(FaultCode::Other, |buf| {
                                            let _ = core::fmt::write(
                                                buf,
                                                format_args!(
                                                    "Joint tracking output rejected: {} ({})",
                                                    f.code(),
                                                    f.phase
                                                ),
                                            );
                                        }),
                                    );
                                }
                                crate::tracking::TrackingRejectRt::JointLimit(msg) => {
                                    if !self.safety.deferred_logging() {
                                        tracing::warn!(
                                            "Joint tracking output exceeded limits ({}): terminating motion",
                                            msg.as_str()
                                        );
                                    }
                                    self.hold_measured_pose();
                                    self.drop_macro_run("failed", Some("JOINT_LIMIT"));
                                    self.safety.latch(
                                        crate::safety::FaultKind::JointLimit,
                                        crate::safety::FaultDetail::formatted(FaultCode::JointLimit, |buf| {
                                            let _ = core::fmt::write(
                                                buf,
                                                format_args!("Joint tracking output rejected: {}", msg.as_str()),
                                            );
                                        }),
                                    );
                                }
                            }
                            tick_output = self.apply_tracking_fallback_step_inline(pending, gravity_output_enabled);
                        }
                    }
                    self.tracking.last_tick_sweep_evals =
                        MAX_KEEPOUT_EVALS_PER_TICK.saturating_sub(tick_sweep_budget);
                } else {
                    if tracking_active {
                        let keep_fallback = self.tracking.holding_fallback_pose
                            && self.safety.state().is_stopped();
                        self.tracking.invalidate();
                        self.tracking.holding_fallback_pose = keep_fallback;
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
                        tick_output = self.write_output_scratch(servo_dt);
                        if matches!(tick_output, TickOutput::Complete { .. }) {
                            allow_output_accepted = true;
                        }
                    } else {
                        driverless_accepted = true;
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
                self.tracking.retained_motion_vel_caps = None;
            }
        }

        self.tick_phase(2);
        // An E-stop or authorized stop requested during this tick: its output did not reach
        // the bus (or the batch stopped at the next frame, `ArmDriver::set_estop_gate`); apply
        // it now instead of one period later. A batch the gate stopped is not a driver failure.
        let mut stop_inhibited_this_tick = matches!(tick_output, TickOutput::EstopInhibited { .. });
        if self.urgent_stop_unapplied() {
            stop_inhibited_this_tick = true;
            allow_output_accepted = false;
            driverless_accepted = false;
            if self.estop_request_unapplied() {
                self.consume_pending_estop();
            }
            if let Some(stops) = self.stop_requests.clone() {
                let generation = stops.current();
                if generation != stops.applied() {
                    self.authorized_stop();
                    self.sync_stop_generation(generation);
                    stops.mark_applied(generation);
                }
            }
            match &tick_output {
                TickOutput::EstopInhibited { .. }
                | TickOutput::LeaseFenced
                | TickOutput::ActionDeadlineRefused { .. } => {
                    tick_output = TickOutput::None;
                }
                TickOutput::Complete { is_hold: false, .. }
                | TickOutput::Deferred { is_hold: false, .. }
                | TickOutput::Expired { is_hold: false, .. } => {
                    tick_output = TickOutput::None;
                }
                _ => {}
            }
        }
        // A disconnect / lease fence arriving between `mit.rs` output check and the TX / J7
        // epilogue: apply the non-blocking disconnect fence now (stopping client motion and
        // revoking target authority) and degrade any pending J7 `Target` intent to stationary
        // `Hold` so the arm holds cleanly with ZERO old-generation motion on CAN.
        let mut disconnect_fenced_mid_tick = false;
        if self.disconnect_fence_blocks_output() || matches!(tick_output, TickOutput::LeaseFenced) {
            disconnect_fenced_mid_tick = true;
            allow_output_accepted = false;
            driverless_accepted = false;
            self.sync_nonblocking_disconnect_fence(now_fn());
            if matches!(
                tick_output,
                TickOutput::LeaseFenced
                    | TickOutput::Complete { is_hold: false, .. }
                    | TickOutput::Deferred { is_hold: false, .. }
                    | TickOutput::Expired { is_hold: false, .. }
            ) {
                tick_output = TickOutput::None;
            }
        }
        // R4 Blocker 3: when Legacy is driven manually via `tick()` / `control_tick()` (without
        // the threaded `legacy_safe_hold_handoff` loop) after a session requested SAFE-HOLD,
        // execute bounded measured-position holds for all verified-mode axes + causal CTRL_MODE
        // readback probes for uncertain axes directly in the release (0 heap alloc, 0 CMD_DISABLE).
        #[cfg(not(target_arch = "wasm32"))]
        if safe_hold_inline_fallback
            && !self.shutdown_committed
            && !self.detached
            && self.torque_intent == TorqueIntent::On
        {
            let dof = self.config.dof.min(crate::rt_core::mailbox::MAX_DOF);
            let mut hold_targets: [crate::driver::JointTarget; crate::rt_core::mailbox::MAX_DOF] =
                std::array::from_fn(|_| crate::driver::JointTarget::position(0.0));
            let mut hold_mask = 0u64;
            let feedback_ok = self.commit_feedback_valid();
            for j in 0..dof {
                let pos = self
                    .measured_hold
                    .captured_positions
                    .get(j)
                    .copied()
                    .flatten()
                    .filter(|p| p.is_finite())
                    .or_else(|| {
                        feedback_ok
                            .then(|| self.measured_positions.get(j).copied())
                            .flatten()
                            .filter(|p| p.is_finite())
                    });
                if let Some(p) = pos {
                    hold_targets[j] = crate::driver::JointTarget::position(p);
                    if j < 64 {
                        hold_mask |= 1u64 << j;
                    }
                }
            }
            let release_cap = self.active_tx_cutoff.map(|c| c.drv_release_cap);
            let use_try = self.rt_executor_owned || self.nonblocking_driver_release;
            let shadow_rt = &mut self.torque_shadow;
            if let Some(d) = self.driver.as_mut() {
                let run_inline_sh = |drv: &mut dyn ArmDriver| {
                    if let Some(cap) = release_cap {
                        drv.set_tx_release_deadline(Some(cap));
                    }
                    drv.pos_force_mut().set_pending_pos_force_hold_hint(true);
                    let mut unverified_mask = 0u64;
                    if hold_mask != 0 {
                        if let Ok(report) =
                            drv.write_measured_hold_masked(hold_mask, &hold_targets[..dof])
                        {
                            for j in 0..dof.min(report.dof) {
                                if matches!(
                                    report.joint_status[j],
                                    crate::driver::JointHoldStatus::UnheldNoVerifiedMode
                                ) && j < 64
                                {
                                    unverified_mask |= 1u64 << j;
                                }
                            }
                        }
                    }
                    if unverified_mask != 0 {
                        let _ = drv.joint_mode_mut().request_mode_readback_budgeted(unverified_mask);
                    }
                    shadow_rt.drain_from_driver(drv);
                };
                if use_try {
                    let _ = d.try_with_mut(run_inline_sh);
                } else {
                    let _ = d.with_mut(run_inline_sh);
                }
            }
        }
        // Tick epilogue (BOTH Legacy and RT Executor owners): invoke the ONE J7 PosForce emitter
        // exactly once per control tick AFTER whatever arm-write path ran (whether arm output
        // succeeded, deferred, expired, errored, hit action deadline cutoff, tripped the MIT fence
        // before/mid-batch, hit stale precheck / BuildRefused / invalid caller J7 target, or was
        // skipped by send_to_bus). Fresh `Target` intents respect `active_tx_cutoff`'s action/macro
        // deadline; stop/hold/freeze-class frames restore `drv_release_cap` so a deadline never
        // suppresses a stop, hold, or freeze.
        let is_stopped_or_failed_hold_epilogue = self.safety.state().is_stopped()
            || self.enable_lifecycle == driver_transitions::EnableLifecycle::FailedHold
            || safe_hold_inline_fallback;
        let pending_disable_hold_epilogue =
            (self.disable_pending || transition_owns_bus) && self.torque_intent == TorqueIntent::On;
        let j7_eligible = self.torque_intent == TorqueIntent::On
            && (self.is_torqued
                || ((is_stopped_or_failed_hold_epilogue || pending_disable_hold_epilogue)
                    && self.stopped_hold_eligible()));
        if !self.shutdown_committed
            && !output_tx_cap_expired
            && j7_eligible
            && !(enable_transition_owns_bus && !safe_hold_inline_fallback)
            && !self.urgent_stop_unapplied()
        {
            let current_stamp = self.current_j7_intent_stamp();
            let j7_is_hold = is_stopped_or_failed_hold_epilogue
                || pending_disable_hold_epilogue
                || self.e_stop_latched.load(Ordering::SeqCst);
            let j7_intent = if j7_is_hold {
                crate::driver::PosForceIntent::stop(current_stamp)
            } else if disconnect_fenced_mid_tick
                || self.disconnect_fence_blocks_output()
                || matches!(tick_output, TickOutput::Error { .. } | TickOutput::Expired { .. })
            {
                crate::driver::PosForceIntent::hold(current_stamp)
            } else {
                self.j7_intent
                    .unwrap_or_else(|| crate::driver::PosForceIntent::hold(current_stamp))
            };
            self.emit_j7_tick_epilogue(
                j7_is_hold,
                j7_intent,
                current_stamp,
                &mut tick_output,
                &mut allow_output_accepted,
            );
        }
        self.j7_intent = None;
        // Coalesced end-of-release watchdog fallback: normal control/hold output has now run;
        // emit an urgent stationary hold ONLY for owner-authorized already-enabled arm joints whose
        // normal output in this release was absent, deferred, or failed and whose watchdog deadline
        // needs it. Stopped / E-stop / FailedHold / pending-disable states strictly consume the
        // owner's `measured_hold` joint mask & captured targets (never overriding
        // `UnheldNoFreshFeedback` or `DisableProtective`).
        // Restores `drv_release_cap` so an expired action deadline never suppresses watchdog safety.
        let is_stopped_or_failed_hold_fb = self.safety.state().is_stopped()
            || self.enable_lifecycle == driver_transitions::EnableLifecycle::FailedHold;
        let pending_disable_hold_fb =
            (self.disable_pending || transition_owns_bus) && self.torque_intent == TorqueIntent::On;
        if !self.shutdown_committed
            && !enable_transition_owns_bus
            && !stop_inhibited_this_tick
            && !disconnect_fenced_mid_tick
            && !self.urgent_stop_unapplied()
            && !output_tx_cap_expired
            && !matches!(tick_output, TickOutput::ActionDeadlineRefused { .. })
            && !(self.mit.owner == crate::mit::MitOwner::Active
                && matches!(tick_output, TickOutput::Error { .. }))
            && self.stopped_hold_eligible()
            && (self.is_torqued || is_stopped_or_failed_hold_fb || pending_disable_hold_fb)
        {
            let dof = self.config.dof.min(crate::rt_core::mailbox::MAX_DOF);
            let use_owner_hold_targets = is_stopped_or_failed_hold_fb || pending_disable_hold_fb;
            let mut owner_hold_targets = [None; crate::rt_core::mailbox::MAX_DOF];
            let authorized_mask = if use_owner_hold_targets {
                let mask = self.measured_hold.owner_hold_mask;
                for j in 0..dof {
                    if (mask & (1u64 << j)) != 0 {
                        let pos = self.measured_hold.owner_hold_targets[j];
                        if pos.is_finite() {
                            owner_hold_targets[j] = Some(pos);
                        }
                    }
                }
                mask
            } else if dof >= 64 {
                u64::MAX
            } else {
                (1u64 << dof) - 1
            };
            let release_cap = self.active_tx_cutoff.map(|c| c.drv_release_cap);
            let use_try = self.rt_executor_owned || self.nonblocking_driver_release;
            #[cfg(not(target_arch = "wasm32"))]
            let shadow_rt = &mut self.torque_shadow;
            if let Some(d) = self.driver.as_mut() {
                let target_slice = use_owner_hold_targets.then_some(&owner_hold_targets[..dof]);
                let run_fb = |drv: &mut dyn ArmDriver| {
                    if let Some(cap) = release_cap {
                        drv.set_tx_release_deadline(Some(cap));
                    }
                    drv.service_release_watchdog_fallback(authorized_mask, target_slice);
                    #[cfg(not(target_arch = "wasm32"))]
                    shadow_rt.drain_from_driver(drv);
                };
                if use_try {
                    let _ = d.try_with_mut(run_fb);
                } else {
                    let _ = d.with_mut(run_fb);
                }
            }
        }
        let action_deadline_refused_at = match tick_output {
            TickOutput::ActionDeadlineRefused { drv_now, .. } => Some(drv_now),
            _ => None,
        };
        let has_pending_motion = matches!(tick_output, TickOutput::None)
            && self.max_tx_joint_no_progress_ticks() > 0
            && self
                .read_driver_for_tick()
                .is_some_and(|drv| drv.tx().pending_motion_tx_len() > 0);
        let j7_stale = tick_output.j7_stale();
        let batch_completed = self.classify_tick_output(tick_output, true, has_pending_motion);

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

        let arm_output_accepted =
            (driverless_accepted || (allow_output_accepted && batch_completed))
                && !self.safety.state().is_stopped();
        let tick_output_accepted = arm_output_accepted && !j7_stale;

        if !gravity_handback_ran {
            let gravity_stiff_hold_completed =
                gravity_exiting_or_failed && (gravity_driverless_ok || batch_completed);
            self.step_gravity_comp_exit(servo_dt, gravity_stiff_hold_completed);
        }

        let mut tick_now = now_fn();
        if let (Some(drv_refused_now), Some(c)) = (action_deadline_refused_at, self.active_tx_cutoff) {
            let translated = c.owner_anchor + drv_refused_now.saturating_duration_since(c.drv_anchor);
            if translated > tick_now {
                tick_now = translated;
            }
        }
        if let Some(done) = deferred_done {
            if !self.safety.state().is_stopped() {
                if j7_stale && done.is_gripper {
                    let g = self.config.gripper_index().unwrap_or(self.config.dof.saturating_sub(1));
                    self.fail_running_gripper_for_j7_stale_rt(g, Some(done));
                } else {
                    self.pending_deferred_done = Some(done);
                }
            }
        }
        if self.pending_deferred_done.is_some() {
            let current_epoch = self.safety.epoch_id();
            let current_lease = self.active_lease.as_ref().map(|l| l.generation);
            let done_still_valid = self.pending_deferred_done.as_ref().is_some_and(|done| {
                !self.safety.state().is_stopped()
                    && done.epoch_id == current_epoch
                    && done.lease_generation == current_lease
                    && self.active_trajectory.is_none()
                    && self.servo_target.is_none()
                    && match done.target {
                        DeferredCompletionTarget::Action(id) => {
                            self.last_action.as_ref().is_some_and(|a| a.id == id && a.is_running())
                        }
                        DeferredCompletionTarget::MacroSub { action_id, ticket } => {
                            self.macro_run.as_ref().is_some_and(|m| {
                                m.id == action_id
                                    && m.ticket() == Some(ticket)
                                    && !m.stop_requested
                                    && m.sub == SubState::Running
                            })
                        }
                        DeferredCompletionTarget::MacroFinish { action_id, .. } => {
                            self.macro_run.as_ref().is_some_and(|m| m.id == action_id)
                                && self.last_action.as_ref().is_some_and(|a| a.id == action_id && a.is_running())
                        }
                        DeferredCompletionTarget::ParkRelease(phase) => {
                            self.pending_normal_unload && self.park_release == Some(phase)
                        }
                        DeferredCompletionTarget::Anonymous => {
                            !self.pending_normal_unload
                                && !self.last_action.as_ref().is_some_and(|a| a.is_running())
                        }
                    }
            });
            let deadline_expired_for_sub = matches!(
                self.pending_deferred_done.as_ref().map(|d| d.target),
                Some(DeferredCompletionTarget::MacroSub { action_id, .. })
                    if self.macro_deadline_elapsed(action_id, tick_now)
            );
            if !done_still_valid {
                self.pending_deferred_done = None;
            } else if tick_output_accepted && !deadline_expired_for_sub {
                if let Some(done) = self.pending_deferred_done.take() {
                    if done.clear_releasing {
                        self.releasing = false;
                        self.clear_held_rt();
                    }
                    if !self.pending_normal_unload {
                        self.safety.mark_motion_finished();
                    }
                    match done.target {
                        DeferredCompletionTarget::MacroSub { action_id, ticket } => {
                            if let Some(m) = self.macro_run.as_mut().filter(|m| {
                                m.id == action_id
                                    && m.ticket() == Some(ticket)
                                    && !m.stop_requested
                                    && m.sub == SubState::Running
                            }) {
                                m.sub = SubState::Done(SubDone {
                                    settled: done.settled,
                                    holding: done.holding,
                                    gripper_gap_rad: done.gripper_gap_rad,
                                    at: tick_now,
                                });
                            }
                        }
                        DeferredCompletionTarget::MacroFinish {
                            action_id,
                            state,
                            code,
                            message,
                            clear_held,
                        } => {
                            if clear_held {
                                self.clear_held_rt();
                            }
                            let msg = if !message.is_empty() {
                                if self.rt_executor_owned {
                                    let mut buf = self.take_action_msg_buf(action_id, message.as_str().len().max(160));
                                    buf.clear();
                                    buf.push_str(message.as_str());
                                    Some(buf)
                                } else {
                                    Some(message.as_str().to_owned())
                                }
                            } else {
                                None
                            };
                            self.finish_macro_inner(
                                action_id,
                                state.as_str(),
                                code.as_deref(),
                                msg,
                                done.holding,
                                done.gripper_gap_rad,
                            );
                        }
                        DeferredCompletionTarget::Action(id) => {
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
                        DeferredCompletionTarget::ParkRelease(_)
                        | DeferredCompletionTarget::Anonymous => {}
                    }
                }
            }
        }

        // Clear macro replacement-hold pending flag only once an output release is accepted on the bus.
        if tick_output_accepted {
            if let Some(m) = self.macro_run.as_mut() {
                if !m.finishing {
                    m.hold_pending_bus_accept = false;
                }
            }
        }

        // Park release and unload settle advance strictly after output acceptance so neither
        // gripper release progression nor resting-pose settle/torque-off can advance across a
        // deferred or rejected release.
        if self.pending_normal_unload
            && self.active_trajectory.is_none()
            && self.pending_deferred_done.is_none()
            && !self.safety.state().is_stopped()
        {
            if tick_output_accepted {
                if !self.step_park_release() {
                    self.step_unload_settle(servo_dt);
                }
            } else if let Some(settle) = self.unload_settle.as_mut() {
                settle.pending_s += servo_dt;
                settle.held_s = 0.0;
                settle.held_since_s = None;
                settle.samples.clear();
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
                self.commanded_velocities.fill(0.0);
                self.safety.latch(
                    crate::safety::FaultKind::Other,
                    crate::safety::FaultDetail::Static(crate::safety::StaticFault::HandoverSettlingTimeout),
                );
                self.handover_settling_since = None;
            }
        }

        // Settle and action completion require an accepted output on both tracking-on and
        // tracking-off paths so deferred releases never finalize an action early.
        self.reconcile_action(servo_dt, tick_output_accepted, tick_now);
        self.expire_motion_limit();
        self.tick_phase(3);
        let timestamp = self.start_time.elapsed().as_secs_f64();

        // 5. Record frame if episode recording is active:
        //    - Executor owner: streams `Copy` sample over `rt_core/stream` (0 alloc, 0 lock).
        //    - Legacy owner: appends to `record_ring` or direct `recorder` buffer.
        if self.rt_executor_owned {
            #[cfg(not(target_arch = "wasm32"))]
            if self
                .sessions
                .slot(crate::rt_core::session::SessionSlot::Recorder)
                .is_some_and(crate::rt_core::session::ActiveSession::is_live)
            {
                let _ = self.rt_recorder.try_push_sample(
                    self.control_release_seq,
                    timestamp,
                    &self.measured_positions,
                    &self.commanded_positions,
                );
            }
        } else if self.recording_frame_room() {
            if self.core.recording_active() {
                let seq = self.core.published_seq() + 1;
                if self.core.record_sample(seq, timestamp, &self.measured_positions, &self.commanded_positions) {
                    let count = self.recording_frames();
                    if let Some(r) = &self.recording {
                        r.note_frame_count(count);
                    }
                }
            } else {
                self.recorder.append_frame(RecordFrame {
                    timestamp,
                    observation_state: self.measured_positions.clone(),
                    action: self.commanded_positions.clone(),
                });
                let count = self.recording_frames();
                if let Some(r) = &self.recording {
                    r.note_frame_count(count);
                }
            }
        }

        #[cfg(not(target_arch = "wasm32"))]
        if let Some(ref mut leg_budget) = legacy_ledger {
            let (cons_rx, cons_tx) = self.take_conservative_step_deltas();
            self.charge_driver_budget_after(
                leg_budget,
                legacy_output_before,
                cons_rx,
                false,
                cons_tx,
            );
        }

        // Legacy ticked hardware commissioning for direct `tick_control*` callers
        // (`ArmSupervisor::tick_shared` sets `defer_legacy_commissioning = true` and services
        // commissioning in separate <= 2 ms write guards strictly AFTER tick output & publish).
        // Runs strictly AFTER the tick's own output using the remaining release budget derived
        // from `release_deadline - margin_tick` (matching TQ-LOCK TX deadline separation),
        // bounded to `COMM_MAX_TX_PER_RELEASE` frames per release.
        #[cfg(not(target_arch = "wasm32"))]
        if !was_step2_poll
            && !self.defer_legacy_commissioning
            && self.legacy_commissioning_eligible()
        {
            let comm_now = now_fn();
            let comm_cutoff = match release_deadline {
                Some(rd) => rd.checked_sub(margin_tick).unwrap_or(comm_now),
                None => comm_now + period_tick.saturating_sub(margin_tick),
            };
            let comm_ctx = crate::rt_core::ops::CommitCtx {
                arm: self.core.id(),
                boot: crate::rt_core::broker::BootId::process(),
                now: comm_now,
                stop_generation: self.current_stop_gen(),
            };
            if let Some(ref mut leg_budget) = legacy_ledger {
                let _ = self.service_commissioning_step_with_budget(
                    &comm_ctx,
                    leg_budget,
                    comm_cutoff,
                    crate::mit::COMM_MAX_TX_PER_RELEASE,
                    None,
                    true,
                    now_fn,
                );
            } else {
                let _ = self.service_commissioning_step_with_cutoff(
                    &comm_ctx,
                    comm_cutoff,
                    crate::mit::COMM_MAX_TX_PER_RELEASE,
                    None,
                    true,
                    now_fn,
                );
            }
        }

        self.active_tx_cutoff = None;
        self.rt_j7_pf_gate = None;
        if applied_rx_tx_cap || applied_output_tx_cap {
            if let Some(mut drv) = self.write_driver_for_tick() {
                drv.set_tx_release_deadline(None);
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        if !was_step2_poll {
            self.last_legacy_release_budget = legacy_ledger;
        }
        #[cfg(not(target_arch = "wasm32"))]
        if !self.rt_executor_owned {
            self.flush_torque_shadow();
        }
        self.nonblocking_driver_release = false;

        timestamp
    }
}

/// Lock-free ticket returned synchronously by [`ArmSupervisor::begin_e_stop_checked`] after
/// latching `e_stop_latched` and advancing `requested_estop_count` via atomics (zero mutexes,
/// zero waiter scans).
pub(crate) struct CheckedEstopTicket {
    req: u64,
    mode: u8,
}

/// Thread-safe Arm Supervisor managing the robotic arm daemon
pub struct ArmSupervisor {
    inner: Arc<SupervisorLock<SupervisorInner>>,
    telemetry_tx: broadcast::Sender<ArmObservation>,
    terminals: Arc<TerminalBroadcaster>,
    terminal_capture_channel: Arc<terminal_capture::TerminalCaptureChannel>,
    /// Single authoritative atomic state packing `(generation << 1) | latched` for E-stop.
    pub estop_state: Arc<crate::rt_core::urgent::EstopState>,
    pub e_stop_latched: crate::rt_core::urgent::EstopLatchView,
    pub requested_estop_count: crate::rt_core::urgent::EstopCounterView,
    stop_gen: Arc<AtomicU64>,
    estop_epoch: Arc<AtomicU64>,
    lease_gen_mirror: Arc<AtomicU64>,
    closed_bitset: Arc<crate::rt_core::requests::actions::ClosedBitset>,
    atomic_fenced_lease_gen: Arc<AtomicU64>,
    pending_disconnect_hold: Arc<AtomicU64>,
    /// Shared persistent `PolicyAuthorization` for executor-owned policy runs (0 RT alloc/free).
    #[cfg(feature = "policy-runtime")]
    rt_policy_auth: crate::policy_runtime::PolicyAuthorization,
    /// Shared exclusive-lifetime lock-free policy cancellation slots.
    #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
    policy_cancel_slots: Arc<crate::rt_core::requests::policy::PolicyCancelSlots>,
    /// Off-thread coordinator for immutable RT policy terminal records and qualification ledger fences.
    #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
    policy_qualification: Arc<crate::rt_core::requests::policy::PolicyQualificationCoordinator>,
    /// Bounded permit pool capping concurrent blocking policy loads/warmups and inferences.
    #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
    policy_backend_permits: Arc<tokio::sync::Semaphore>,
    /// Leaf queue of socket-close disconnects deferred when the owner write lock is contended
    /// or ownership is transitioning (`OWNER_TRANSFERRING`); NEVER locked on `armd-control`.
    pending_disconnects: std::sync::Mutex<std::collections::VecDeque<crate::rt_core::requests::lease::ConnectionHandle>>,
    tick_stats: Arc<crate::diagnostics::tick::LockFreeTickStats>,
    control_sched: Arc<std::sync::Mutex<Option<crate::diagnostics::tick::ControlThreadSched>>>,
    /// Readers' side of the tick's state publication (`crate::rt_core::publish`).
    hub: Arc<crate::rt_core::publish::StateHub>,
    /// Serializes the start of the dispatcher thread.
    dispatcher_start: std::sync::Mutex<()>,
    /// Control period of the last tick (µs): bounds how long a reader waits for a sample.
    control_period_us: Arc<AtomicU64>,
    /// Configured nominal control loop frequency (Hz, immutable after startup).
    control_frequency_hz: u32,
    obs_cache_hits: AtomicU64,
    obs_cache_misses: AtomicU64,
    obs_waits: AtomicU64,
    obs_unavailable: AtomicU64,
    config_write_mutex: Arc<tokio::sync::Mutex<()>>,
    /// Off-RT published configuration snapshot for REST readers and off-thread prepare (never touched on RT).
    published_config: Arc<crate::rt_core::requests::config::PublishedConfigHub>,
    /// Test-only hooks for deterministic config-commit failure injection across both owners.
    config_commit_hooks: std::sync::Mutex<crate::rt_core::requests::config::ConfigCommitTestHooks>,
    /// Monotonic mirror of committed `SequenceStore::revision` (Release-updated on activate
    /// BEFORE unpinning reserved replay episodes so concurrent episode deletes are linearizable).
    sequences_revision: Arc<AtomicU64>,
    /// Single-owner slot (`Legacy` -> `Transferring` -> `Executor` | `Failed`).
    /// Lock order: always `self.inner` before `self.owner`.
    owner: std::sync::Mutex<OwnerSlot>,
    owner_mode: Arc<AtomicU8>,
    /// Lock-free mirror of why the current effective owner was selected (`OwnerReason as u8`).
    owner_reason: Arc<AtomicU8>,
    /// `self.inner.generation()` captured at the moment ownership transfers to the RT executor.
    /// In `OWNER_EXECUTOR` mode `fresh()` requires `s.meta.generation >= executor_base_gen + 1`
    /// so rejected legacy lock attempts on the detached shell cannot skew observation freshness.
    executor_base_gen: AtomicU64,
    /// Always-present lock-serialized E-stop waiter registry (never touched by `armd-control`).
    estop_reg: Arc<std::sync::Mutex<crate::rt_core::executor::EstopRegistry>>,
    /// Off-thread consumer sinks installed when ownership transfers to the RT executor.
    sinks: Arc<std::sync::Mutex<Option<Arc<crate::rt_core::executor::OffThreadSinks>>>>,
    /// Template observation used by `armd-dispatch` to carry drained action terminals if no
    /// control tick has published to `hub` yet.
    template_obs: Box<ArmObservation>,
    /// Shared periodic-loop timing stats attached in Legacy mode (`run_periodic`).
    legacy_loop_stats: std::sync::Mutex<Option<Arc<crate::rt_core::executor::ExecutorStats>>>,
    /// Off-thread bounded WS connection slot pool (`MAX_LIVE_CONNECTIONS = 64`), enforcing the
    /// fail-closed cap before allocating a monotonic connection ID and before touching any owner.
    ws_slots: Arc<WsConnectionSlotPool>,
    /// Off-thread `TorqueController` shadow comparator/diagnostics hub (never locked on RT).
    #[cfg(not(target_arch = "wasm32"))]
    torque_shadow: Arc<crate::torque::shadow::TorqueShadowHub>,
    /// Unique identifier for this daemon/supervisor instance (changes across restarts).
    instance_id: String,
    /// Guards off-lock legacy watchdog park planning against concurrent duplicate spawns.
    watchdog_park_planning: Arc<AtomicBool>,
    /// Off-RT sidecar store of `calib_hover` jobs when `OWNER_EXECUTOR` is active (never touched
    /// by `armd-control`; bounded to `calib_hover::KEEP_JOBS`).
    executor_hover_jobs: std::sync::Mutex<Vec<crate::calib_hover::HoverJob>>,
    /// Off-RT coordinator for executor-owned teach recording sessions (PR0B `SessionSlot::Recorder`
    /// + `rt_core/stream`; never locked on `armd-control`).
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) executor_recorder: Arc<crate::teach::ExecutorRecorder>,
    /// Test-only override for legacy WS `/api/v1/torque` request deadline in ms (0 = default).
    legacy_torque_deadline_override_ms: AtomicU64,
    /// Off-lock wakeup broadcast for legacy WS `/api/v1/torque` waiters, notified strictly AFTER
    /// releasing `SupervisorLock::write` so control releases never touch Tokio's Notify mutex.
    legacy_torque_notify: Arc<tokio::sync::Notify>,
    /// Guards the single bounded headless cleanup task that drains a contended `DetachedCancel` or
    /// expired `Detached` disable when no live control loop is ticking.
    headless_detached_drain_active: Arc<AtomicBool>,
    /// Lock-free mirror of `SupervisorInner::has_detached_torque_transition()`.
    detached_torque_pending_atomic: Arc<AtomicBool>,
    /// Shared counter of forbidden `DriverLock` attempts on `armd-control`.
    driver_lock_attempts: Arc<AtomicU64>,
    /// Startup-resolved multi-arm cell context mirrored off the supervisor lock for read-only
    /// `GET /api/v1/cell/status` without touching RT or control-loop locks (off by default).
    cell: std::sync::RwLock<Option<Arc<crate::cell::CellContext>>>,
    /// Process-wide / supervisor-wide lock-free internal-fault lane (DRV-F1 / FU-PANIC).
    internal_fault_lane: Arc<crate::panic_coord::InternalFaultLane>,
    /// Shared SAFE-HOLD control block for the legacy control loop.
    legacy_safe_hold: Arc<crate::rt_core::safe_hold::SafeHoldControl>,
    /// Authoritative active bus-writer lifecycle endpoint (Legacy, Executor, or DetachedSafeHold)
    /// so daemon shutdown and deliberate process-exit always reach the actual writer (B4).
    active_writer: std::sync::Mutex<WriterEndpoint>,
    /// Supervisor-wide sticky flag set when deliberate `ProcessExit` protective disable is
    /// requested so concurrent ownership handback never strands the driver without disable (B3).
    process_exit_requested: AtomicBool,
    /// Test-only fault-injection flag forcing `ArmSupervisor::drop`'s SAFE-HOLD thread spawn to
    /// fail so inline ownership retention is deterministically verified.
    #[cfg(not(target_arch = "wasm32"))]
    fail_drop_safe_hold_spawn: AtomicBool,
}

#[derive(Clone)]
enum LegacyPanicDisableStep {
    Outcome(PanicDisableOutcome),
    Detached,
}

/// Authoritative endpoint for the single active hardware bus writer across startup, handback,
/// detached SAFE-HOLD, and daemon process exit (B4).
#[derive(Clone)]
pub enum WriterEndpoint {
    Legacy {
        control_stop: Option<Arc<AtomicBool>>,
        safe_hold: Arc<crate::rt_core::safe_hold::SafeHoldControl>,
    },
    /// Ownership transfer is in progress (e.g. `start_executor` extracting `SupervisorInner`
    /// into `armd-control`); `ProcessExit` must wait for the authoritative endpoint rather than
    /// selecting `Legacy` or reading the detached shell.
    Transferring,
    Executor(crate::rt_core::executor::RtExecutorHandle),
    DetachedSafeHold {
        safe_hold: Arc<crate::rt_core::safe_hold::SafeHoldControl>,
        control_stop: Arc<AtomicBool>,
    },
}

static NEXT_WS_CONN_ID: AtomicU64 = AtomicU64::new(1);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum WsSlotPhase {
    Admitted,
    Open,
    Closing,
}

struct WsSlotEntry {
    phase: WsSlotPhase,
    opening: bool,
    #[cfg(not(target_arch = "wasm32"))]
    reply_sink: Option<Arc<crate::rt_core::reply_router::ConnectionReplySink>>,
}

struct WsConnectionSlotPool {
    entries: std::sync::Mutex<std::collections::HashMap<u64, WsSlotEntry>>,
    notify: tokio::sync::Notify,
}

impl WsConnectionSlotPool {
    fn new() -> Self {
        Self {
            entries: std::sync::Mutex::new(std::collections::HashMap::with_capacity(
                MAX_LIVE_CONNECTIONS,
            )),
            notify: tokio::sync::Notify::new(),
        }
    }

    fn try_claim(&self) -> Result<crate::rt_core::requests::lease::ConnectionHandle, ArmError> {
        let mut g = self.entries.lock().unwrap_or_else(|e| e.into_inner());
        if g.len() >= MAX_LIVE_CONNECTIONS {
            return Err(ArmError::Rejected {
                status: 503,
                code: "TOO_MANY_CONNECTIONS".into(),
                message: "Maximum live WebSocket connections reached".into(),
            });
        }
        let id = NEXT_WS_CONN_ID.fetch_add(1, Ordering::Relaxed);
        let handle = crate::rt_core::requests::lease::ConnectionHandle::from_ws_id(id)
            .expect("monotonic WS conn_id >= 1");
        g.insert(
            id,
            WsSlotEntry {
                phase: WsSlotPhase::Admitted,
                opening: false,
                #[cfg(not(target_arch = "wasm32"))]
                reply_sink: None,
            },
        );
        Ok(handle)
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn attach_reply_sink(
        &self,
        conn_id: u64,
        sink: Arc<crate::rt_core::reply_router::ConnectionReplySink>,
    ) -> bool {
        let mut g = self.entries.lock().unwrap_or_else(|e| e.into_inner());
        let Some(entry) = g.get_mut(&conn_id) else {
            return false;
        };
        if entry.phase == WsSlotPhase::Closing {
            return false;
        }
        entry.reply_sink = Some(sink);
        true
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn reply_sink(
        &self,
        conn_id: u64,
    ) -> Option<Arc<crate::rt_core::reply_router::ConnectionReplySink>> {
        let g = self.entries.lock().unwrap_or_else(|e| e.into_inner());
        let entry = g.get(&conn_id)?;
        if entry.phase == WsSlotPhase::Closing {
            return None;
        }
        entry.reply_sink.clone()
    }

    /// Joins transferred `live_ids` against non-closing `WsSlotEntry` reply sinks for atomic
    /// executor handover seeding (omits unregistered claimed slots and already-closing slots).
    #[cfg(not(target_arch = "wasm32"))]
    fn sinks_for_live_ids(
        &self,
        live_ids: &[u64],
    ) -> Vec<(u64, Arc<crate::rt_core::reply_router::ConnectionReplySink>)> {
        let g = self.entries.lock().unwrap_or_else(|e| e.into_inner());
        let mut out = Vec::with_capacity(live_ids.len());
        for &id in live_ids {
            if let Some(entry) = g.get(&id) {
                if entry.phase != WsSlotPhase::Closing {
                    if let Some(sink) = entry.reply_sink.as_ref() {
                        out.push((id, Arc::clone(sink)));
                    }
                }
            }
        }
        out
    }

    fn begin_open(&self, conn_id: u64) -> bool {
        let mut g = self.entries.lock().unwrap_or_else(|e| e.into_inner());
        let Some(entry) = g.get_mut(&conn_id) else {
            return false;
        };
        if entry.phase != WsSlotPhase::Admitted || entry.opening {
            return false;
        }
        entry.opening = true;
        true
    }

    fn finish_open(&self, conn_id: u64, succeeded: bool) {
        {
            let mut g = self.entries.lock().unwrap_or_else(|e| e.into_inner());
            if let Some(entry) = g.get_mut(&conn_id) {
                entry.opening = false;
                if succeeded && entry.phase == WsSlotPhase::Admitted {
                    entry.phase = WsSlotPhase::Open;
                }
            }
        }
        self.notify.notify_waiters();
    }

    fn mark_closing(&self, conn_id: u64) -> Option<bool> {
        let mut g = self.entries.lock().unwrap_or_else(|e| e.into_inner());
        let entry = g.get_mut(&conn_id)?;
        entry.phase = WsSlotPhase::Closing;
        Some(entry.opening)
    }

    fn is_opening(&self, conn_id: u64) -> bool {
        self.entries
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(&conn_id)
            .is_some_and(|e| e.opening)
    }

    fn remove(&self, conn_id: u64) {
        {
            let mut g = self.entries.lock().unwrap_or_else(|e| e.into_inner());
            g.remove(&conn_id);
        }
        self.notify.notify_waiters();
    }

    fn len(&self) -> usize {
        self.entries.lock().unwrap_or_else(|e| e.into_inner()).len()
    }
}

impl ArmSupervisor {

    /// Owner-aware storage of a `measure_multiview` measurement payload for action `id`.
    /// On `OWNER_EXECUTOR`, stores in `self.hub` (never touching `SupervisorLock` or `armd-control`)
    /// so `StateHub::refresh` attaches `action.measurement` into every published observation and
    /// delivered terminal identically to the Legacy owner.
    pub async fn record_action_measurement(&self, id: u64, value: serde_json::Value) {
        if self.owner_mode() == OWNER_EXECUTOR {
            self.hub.record_action_measurement(id, value);
        } else {
            self.inner()
                .write()
                .await
                .update_action(id, |a| a.measurement = Some(value));
        }
    }

    /// Owner-aware readback of the measurement payload recorded for action `id`.
    /// Never touches `SupervisorLock` when `OWNER_EXECUTOR` is active.
    pub async fn action_measurement(&self, id: u64) -> Option<serde_json::Value> {
        if self.owner_mode() == OWNER_EXECUTOR {
            self.hub.action_measurement(id)
        } else {
            let inner = self.inner().read().await;
            inner
                .last_action
                .as_ref()
                .filter(|a| a.id == id)
                .and_then(|a| a.measurement.clone())
                .or_else(|| {
                    inner
                        .previous_action
                        .as_ref()
                        .filter(|(a, _)| a.id == id)
                        .and_then(|(a, _)| a.measurement.clone())
                })
        }
    }

    /// Owner-aware storage of a `calib_hover` job record (never touches `SupervisorLock` in
    /// `OWNER_EXECUTOR` mode, and re-checks detachment if a handoff lands while waiting on the
    /// legacy write lock).
    pub async fn put_hover_job(&self, job: crate::calib_hover::HoverJob) {
        if self.owner_mode() == OWNER_EXECUTOR {
            self.put_executor_hover_job(job);
            return;
        }
        let mut inner = self.inner().write().await;
        if inner.detached {
            drop(inner);
            if self.owner_mode() == OWNER_EXECUTOR {
                self.put_executor_hover_job(job);
            }
            return;
        }
        inner.hover_jobs.retain(|j| j.id != job.id);
        inner.hover_jobs.push(job);
        let n = inner.hover_jobs.len();
        if n > crate::calib_hover::KEEP_JOBS {
            inner.hover_jobs.drain(..n - crate::calib_hover::KEEP_JOBS);
        }
    }

    fn put_executor_hover_job(&self, job: crate::calib_hover::HoverJob) {
        let mut jobs = self
            .executor_hover_jobs
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        jobs.retain(|j| j.id != job.id);
        jobs.push(job);
        let n = jobs.len();
        if n > crate::calib_hover::KEEP_JOBS {
            jobs.drain(..n - crate::calib_hover::KEEP_JOBS);
        }
    }

    /// Owner-aware snapshot of stored `calib_hover` jobs (never touches `SupervisorLock` in
    /// `OWNER_EXECUTOR` mode, and re-checks detachment if a handoff lands while waiting on the
    /// legacy read lock).
    pub async fn hover_jobs_snapshot(&self) -> Vec<crate::calib_hover::HoverJob> {
        if self.owner_mode() == OWNER_EXECUTOR {
            return self
                .executor_hover_jobs
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .clone();
        }
        let inner = self.inner().read().await;
        if inner.detached {
            drop(inner);
            if self.owner_mode() == OWNER_EXECUTOR {
                return self
                    .executor_hover_jobs
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .clone();
            }
            return Vec::new();
        }
        inner.hover_jobs.clone()
    }

    /// Current live `SequenceStore::revision` mirror used to linearize episode deletes against
    /// concurrent `Sequences` save commits without blocking on `config_write_mutex`.
    pub fn sequences_revision(&self) -> u64 {
        self.sequences_revision.load(Ordering::Acquire)
    }

    pub(crate) fn set_sequences_revision(&self, rev: u64) {
        self.sequences_revision.store(rev, Ordering::Release);
    }

    /// Checks on the single live owner (`Legacy` or `Executor`) whether episode `id` is free of
    /// saved, running, or reserved sequence references (`409 IN_USE` otherwise).
    pub async fn check_episode_delete_allowed(&self, id: &str) -> Result<(), ArmError> {
        use crate::rt_core::requests::config::{ConfigReply, ConfigRequest};
        if self.owner_mode() == OWNER_LEGACY {
            let store = {
                let inner = self.inner.read().await;
                inner.episode_delete_allowed(id)?;
                inner.episodes.clone()
            };
            if store.is_pinned(id) {
                return Err(ArmError::detailed(
                    409,
                    "IN_USE",
                    format!("Episode '{id}' is reserved by an in-progress sequence save"),
                    serde_json::json!({"sequences": ["pending_save"]}),
                ));
            }
            return Ok(());
        }
        match self
            .dispatch_config_request_owned_reliable(ConfigRequest::CheckEpisodeDeleteAllowed(
                crate::model::HolderLabel::new(id),
            ))
            .await
        {
            Ok(ConfigReply::EpisodeDeleteAllowed(true)) => Ok(()),
            Ok(ConfigReply::EpisodeDeleteAllowed(false)) => {
                let mut users = self
                    .published_config()
                    .await
                    .map(|p| p.sequences.using_episode(id))
                    .unwrap_or_default();
                if users.is_empty() {
                    users.push("pending_save".to_string());
                }
                Err(ArmError::detailed(
                    409,
                    "IN_USE",
                    format!("Episode '{id}' is used by {} sequence(s)", users.len()),
                    serde_json::json!({"sequences": users}),
                ))
            }
            Err(e) => Err(e.to_arm_error()),
            _ => Err(ArmError::Internal("unexpected CheckEpisodeDeleteAllowed reply".into())),
        }
    }

    /// Linearizably verifies episode `id` is unreferenced (saved, running, or reserved) on the
    /// live owner, removes it from the `EpisodeStore` catalog on a blocking thread (off the Tokio
    /// worker), and returns its RAII deletion claim guard (`Some(guard)`) if removed, `None` if
    /// absent, or `Err(409 IN_USE)` if referenced. Dropping the guard on async cancellation marks
    /// the claim retryable so the episode ID is never stranded.
    pub async fn take_episode_for_delete_claim(
        &self,
        id: &str,
    ) -> Result<Option<crate::recording::store::EpisodeDeleteClaimGuard>, ArmError> {
        use crate::recording::store::TakeEpisodeOutcome;
        loop {
            let rev0 = self.sequences_revision();
            self.check_episode_delete_allowed(id).await?;
            let store = self.try_episode_store().await?;
            let id_owned = id.to_string();
            let seq_rev = Arc::clone(&self.sequences_revision);
            let outcome = tokio::task::spawn_blocking(move || {
                store.take_if_unpinned_and_rev_matches(&id_owned, &seq_rev, rev0)
            })
            .await
            .map_err(|e| ArmError::Internal(format!("episode delete take failed: {e}")))?;
            match outcome {
                TakeEpisodeOutcome::Removed(guard) => return Ok(Some(guard)),
                TakeEpisodeOutcome::NotFound => return Ok(None),
                TakeEpisodeOutcome::Pinned => {
                    let mut users = self
                        .published_config()
                        .await
                        .map(|p| p.sequences.using_episode(id))
                        .unwrap_or_default();
                    if users.is_empty() {
                        users.push("pending_save".to_string());
                    }
                    return Err(ArmError::detailed(
                        409,
                        "IN_USE",
                        format!("Episode '{id}' is used by {} sequence(s)", users.len()),
                        serde_json::json!({"sequences": users}),
                    ));
                }
                TakeEpisodeOutcome::SequenceRevisionChanged => continue,
            }
        }
    }

    /// Linearizably verifies episode `id` is unreferenced (saved, running, or reserved) on the
    /// live owner and removes it from the `EpisodeStore` catalog. Returns `Ok(true)` if removed,
    /// `Ok(false)` if absent, or `Err(409 IN_USE)` if referenced or pinned by a sequence save.
    pub async fn take_episode_for_delete(&self, id: &str) -> Result<bool, ArmError> {
        self.take_episode_for_delete_claim(id)
            .await
            .map(|opt| opt.is_some())
    }

    /// Linearizably verifies every episode in `dataset` is unreferenced (saved, running, or
    /// reserved) and no recording/import pins `dataset`, then atomically removes all of them from
    /// the `EpisodeStore` catalog while returning their deletion claim tokens and a
    /// [`DatasetDeleteGuard`] blocking new pins.
    pub async fn take_dataset_episodes_for_delete(
        &self,
        dataset: &str,
    ) -> Result<(Vec<(String, u64)>, crate::recording::store::DatasetDeleteGuard), ArmError> {
        use crate::recording::store::TakeDatasetOutcome;
        loop {
            let rev0 = self.sequences_revision();
            let store = self.try_episode_store().await?;
            let ids = store.episodes_of(dataset);
            for id in &ids {
                self.check_episode_delete_allowed(id).await?;
            }
            match store.take_dataset_if_unpinned_and_rev_matches(
                dataset,
                &ids,
                &self.sequences_revision,
                rev0,
            ) {
                TakeDatasetOutcome::Removed(taken, guard) => return Ok((taken, guard)),
                TakeDatasetOutcome::DatasetRecordingBusy => {
                    return Err(ArmError::detailed(
                        409,
                        "RECORDER_BUSY",
                        "a recording into this dataset is running",
                        serde_json::json!({}),
                    ));
                }
                TakeDatasetOutcome::Pinned(pinned_id) => {
                    let mut users = self
                        .published_config()
                        .await
                        .map(|p| p.sequences.using_episode(&pinned_id))
                        .unwrap_or_default();
                    if users.is_empty() {
                        users.push("pending_save".to_string());
                    }
                    return Err(ArmError::detailed(
                        409,
                        "IN_USE",
                        format!("Episode '{pinned_id}' is used by {} sequence(s)", users.len()),
                        serde_json::json!({"sequences": users}),
                    ));
                }
                TakeDatasetOutcome::SequenceRevisionChanged => continue,
            }
        }
    }

    /// Current monotonic stop generation across both `Legacy` and `Executor` owners (lock-free).
    pub fn current_stop_gen(&self) -> u64 {
        let legacy = self.stop_gen.load(Ordering::Acquire);
        self.executor_handle()
            .map_or(legacy, |h| legacy.max(h.stop_requests().current()))
    }

    pub async fn set_operator_auth_enforced(&self, enforced: bool) {
        if self.owner_mode() == OWNER_LEGACY {
            let mut inner = self.inner.write().await;
            if self.check_legacy_under_guard(&inner).is_ok() {
                inner.operator_auth_enforced = enforced;
                let snap = crate::rt_core::requests::config::PublishedConfig::from_inner(&inner);
                let gen = self.inner.generation();
                drop(inner);
                self.published_config.publish(snap, gen);
                return;
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        if self.owner_mode() == OWNER_EXECUTOR {
            let _ = self
                .dispatch_config_request_reliable(|| {
                    crate::rt_core::requests::config::ConfigRequest::SetOperatorAuthEnforced(
                        enforced,
                    )
                })
                .await;
        }
        self.published_config
            .update(|cfg| cfg.operator_auth_enforced = enforced);
    }

    /// Snapshot of shadow `TorqueController` diagnostics (lock-free / off-RT; never touches RT owner).
    #[cfg(not(target_arch = "wasm32"))]
    pub fn torque_shadow_diagnostics(&self) -> crate::torque::shadow::TorqueShadowDiagnostics {
        self.torque_shadow.snapshot()
    }

    /// Off-thread bounded torque back-off diagnostics read from [`StateHub`] without taking
    /// `SupervisorLock`.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn torque_backoff_diagnostics(&self) -> crate::torque_guard::TorqueBackoffDiagnostics {
        if let Some(snap) = self.hub().latest() {
            return snap.rt.torque_backoff.to_diagnostics();
        }
        self.inner()
            .try_read()
            .map(|g| g.torque_guard.backoff_history.to_diagnostics())
            .unwrap_or_else(|_| crate::torque_guard::TorqueBackoffHistory::default().to_diagnostics())
    }

    /// Off-thread `/api/v1/diagnostics/power` payload that reads live published RT telemetry and
    /// `torque_backoff` history from [`StateHub`] when executor-owned (never reading the detached
    /// Legacy shell), and reads `SupervisorInner::power_diagnostics()` under Legacy ownership.
    #[cfg(not(target_arch = "wasm32"))]
    pub async fn power_diagnostics(&self) -> serde_json::Value {
        if self.owner_mode() == OWNER_EXECUTOR {
            let budget = self
                .published_config_hub()
                .latest()
                .and_then(|c| c.config.power_budget.clone())
                .filter(|b| b.enabled);
            let Some(budget) = budget else {
                return serde_json::json!({"enabled": false});
            };
            let snap = self.hub().latest();
            let obs_power = snap.as_ref().and_then(|s| s.obs.power.clone());
            let power_diag = snap
                .as_ref()
                .map(|s| s.rt.power_diag)
                .unwrap_or_default();
            let torque_backoff = snap
                .as_ref()
                .map(|s| s.rt.torque_backoff.to_diagnostics())
                .unwrap_or_else(|| {
                    crate::torque_guard::TorqueBackoffHistory::default().to_diagnostics()
                });
            return power_diag.to_json(&budget, obs_power, torque_backoff);
        }
        self.inner().read().await.power_diagnostics()
    }

    /// Shared handle to the off-thread shadow comparator/diagnostics hub.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn torque_shadow_hub(&self) -> &Arc<crate::torque::shadow::TorqueShadowHub> {
        &self.torque_shadow
    }

    /// Lock-free atomic overflow/summary handle for primary `bb.jsonl` piggyback summaries.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn torque_shadow_overflow_atomics(
        &self,
    ) -> Arc<crate::torque::shadow::ShadowOverflowAtomics> {
        self.torque_shadow.overflow_atomics().clone()
    }

    /// Claims one off-thread WS connection slot (`<= MAX_LIVE_CONNECTIONS`) and allocates a
    /// strictly monotonic connection ID without touching the RT executor. Rejected upgrades
    /// never increment the connection ID counter (preventing permanent holes in `ClosedConnections`)
    /// and the budget applies uniformly to every retained socket, including degraded E-stop sockets.
    pub fn try_claim_ws_slot(
        &self,
    ) -> Result<crate::rt_core::requests::lease::ConnectionHandle, ArmError> {
        self.ws_slots.try_claim()
    }

    /// Attaches a per-connection reply sink to an already-claimed WS slot BEFORE owner
    /// registration begins, so both live executor registration and Legacy->Executor handover
    /// seed the exact same sink boundary.
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn attach_ws_reply_sink(
        &self,
        conn_id: u64,
        sink: Arc<crate::rt_core::reply_router::ConnectionReplySink>,
    ) -> bool {
        self.ws_slots.attach_reply_sink(conn_id, sink)
    }

    /// Number of currently admitted WS connection slots (including closing slots awaiting owner
    /// disconnect confirmation).
    pub fn live_ws_slot_count(&self) -> usize {
        self.ws_slots.len()
    }

    /// Registers an already-claimed off-thread WS slot with the active owner (legacy or RT executor).
    /// Returns `Err(DISCONNECTED)` without touching the owner if teardown already marked the slot closing.
    pub async fn register_ws_connection_for_slot(&self, conn_id: u64) -> Result<(), ArmError> {
        if !self.ws_slots.begin_open(conn_id) {
            return Err(ArmError::Rejected {
                status: 409,
                code: "DISCONNECTED".into(),
                message: "Connection slot is already closing".into(),
            });
        }
        struct OpenGuard<'a> {
            pool: &'a WsConnectionSlotPool,
            conn_id: u64,
            succeeded: bool,
        }
        impl Drop for OpenGuard<'_> {
            fn drop(&mut self) {
                self.pool.finish_open(self.conn_id, self.succeeded);
            }
        }
        let mut guard = OpenGuard {
            pool: &self.ws_slots,
            conn_id,
            succeeded: false,
        };
        let res = self.register_ws_connection(conn_id).await;
        if res.is_ok() {
            guard.succeeded = true;
        }
        res
    }

    /// Completes owner-side disconnect teardown for an admitted WS connection and releases its
    /// off-thread slot ONLY AFTER owner cleanup finishes (or the executor has permanently stopped).
    pub async fn finish_ws_connection(
        &self,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
    ) {
        let Some(was_opening) = self.ws_slots.mark_closing(conn.id()) else {
            // Slot already removed (idempotent).
            return;
        };
        if was_opening {
            // Wait briefly for an in-flight `register_ws_connection_for_slot` to drop/finish so
            // `Open` and `Disconnect` stay linearized without hanging if the executor is stalled.
            let deadline = tokio::time::Instant::now() + Duration::from_millis(250);
            while self.ws_slots.is_opening(conn.id()) {
                let notified = self.ws_slots.notify.notified();
                if !self.ws_slots.is_opening(conn.id()) {
                    break;
                }
                if tokio::time::timeout_at(deadline, notified).await.is_err() {
                    break;
                }
            }
        }
        let _ = self.disconnect_ws_connection(conn).await;
        self.ws_slots.remove(conn.id());
    }

    /// Durable cancellation-safe Drop fallback for externally aborted WS socket tasks:
    /// spawns async `finish_ws_connection` on the Tokio runtime when available so contended
    /// legacy write locks, in-flight `OWNER_TRANSFERRING`, or pending `Open`s never strand a slot.
    pub fn finish_ws_connection_detached(
        self: &Arc<Self>,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
    ) {
        if let Ok(rt) = tokio::runtime::Handle::try_current() {
            let sup = Arc::clone(self);
            rt.spawn(async move {
                sup.finish_ws_connection(conn).await;
            });
            return;
        }
        self.disconnect_ws_connection_detached(conn);
        self.ws_slots.remove(conn.id());
    }

    /// Registers a live WS connection ID (`>= 1`) through the active owner (fail-closed at
    /// [`MAX_LIVE_CONNECTIONS`]). If ownership transfer (`OWNER_TRANSFERRING`) is momentarily
    /// in flight, waits up to a bounded 200 ms window for handoff to settle before registering.
    pub async fn register_ws_connection(&self, conn_id: u64) -> Result<(), ArmError> {
        use crate::rt_core::requests::lease::ConnLifecycleError;
        let deadline = Instant::now() + Duration::from_millis(200);
        let mode = loop {
            let m = self.owner_mode();
            if m != OWNER_TRANSFERRING || Instant::now() >= deadline {
                break m;
            }
            tokio::time::sleep(Duration::from_millis(1)).await;
        };
        match mode {
            OWNER_LEGACY => {
                let mut inner = self.inner.write().await;
                if self.check_legacy_under_guard(&inner).is_err() {
                    drop(inner);
                    if self.owner_mode() == OWNER_EXECUTOR {
                        if let Some(handle) = self.executor_handle() {
                            let open_res = if let Some(sink) = self.ws_slots.reply_sink(conn_id) {
                                handle.open_connection_with_reply_sink_async(conn_id, sink).await
                            } else {
                                handle.open_connection_async(conn_id).await
                            };
                            return match open_res {
                                Ok(Ok(())) => Ok(()),
                                Ok(Err(code)) => {
                                    let status = if code == "TOO_MANY_CONNECTIONS" { 503 } else { 409 };
                                    Err(ArmError::Rejected {
                                        status,
                                        code: code.into(),
                                        message: format!("Connection registration rejected: {code}"),
                                    })
                                }
                                Err(ConnLifecycleError::TooManyConnections) => Err(ArmError::Rejected {
                                    status: 503,
                                    code: "TOO_MANY_CONNECTIONS".into(),
                                    message: "Maximum live WebSocket connections reached".into(),
                                }),
                                Err(ConnLifecycleError::AlreadyClosed | ConnLifecycleError::InvalidConnection) => {
                                    Err(ArmError::Rejected {
                                        status: 409,
                                        code: "DISCONNECTED".into(),
                                        message: "Connection identity is closed or invalid".into(),
                                    })
                                }
                                Err(ConnLifecycleError::Stopped | ConnLifecycleError::ReentrantCallback) => {
                                    Err(executor_unavailable())
                                }
                            };
                        }
                    }
                    return Err(executor_unavailable());
                }
                inner.register_live_connection(conn_id).map_err(|code| {
                    let status = if code == "TOO_MANY_CONNECTIONS" { 503 } else { 409 };
                    ArmError::Rejected {
                        status,
                        code: code.into(),
                        message: format!("Connection registration rejected: {code}"),
                    }
                })
            }
            OWNER_EXECUTOR => {
                let Some(handle) = self.executor_handle() else {
                    return Err(executor_unavailable());
                };
                let open_res = if let Some(sink) = self.ws_slots.reply_sink(conn_id) {
                    handle.open_connection_with_reply_sink_async(conn_id, sink).await
                } else {
                    handle.open_connection_async(conn_id).await
                };
                match open_res {
                    Ok(Ok(())) => Ok(()),
                    Ok(Err(code)) => {
                        let status = if code == "TOO_MANY_CONNECTIONS" { 503 } else { 409 };
                        Err(ArmError::Rejected {
                            status,
                            code: code.into(),
                            message: format!("Connection registration rejected: {code}"),
                        })
                    }
                    Err(ConnLifecycleError::TooManyConnections) => Err(ArmError::Rejected {
                        status: 503,
                        code: "TOO_MANY_CONNECTIONS".into(),
                        message: "Maximum live WebSocket connections reached".into(),
                    }),
                    Err(ConnLifecycleError::AlreadyClosed | ConnLifecycleError::InvalidConnection) => {
                        Err(ArmError::Rejected {
                            status: 409,
                            code: "DISCONNECTED".into(),
                            message: "Connection identity is closed or invalid".into(),
                        })
                    }
                    Err(ConnLifecycleError::Stopped | ConnLifecycleError::ReentrantCallback) => {
                        Err(executor_unavailable())
                    }
                }
            }
            _ => Err(executor_unavailable()),
        }
    }

    /// True when one or more socket-close disconnect fences are queued awaiting owner drain
    /// (fail-closed output hold active).
    pub fn has_pending_disconnect_hold(&self) -> bool {
        self.pending_disconnect_hold.load(Ordering::SeqCst) > 0
    }

    /// True if `conn_id` (`>= 1`) has been marked closed in the lock-free immediate disconnect fence.
    pub fn is_connection_fenced_closed(&self, conn_id: u64) -> bool {
        self.closed_bitset.is_closed(conn_id)
    }

    /// Highest lease generation fenced via the lock-free immediate socket-close fence.
    pub fn atomic_fenced_lease_gen(&self) -> u64 {
        self.atomic_fenced_lease_gen.load(Ordering::SeqCst)
    }

    fn enqueue_pending_disconnect(&self, conn: crate::rt_core::requests::lease::ConnectionHandle) {
        let mut q = self
            .pending_disconnects
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        if !q.iter().any(|c| c.id() == conn.id()) {
            q.push_back(conn);
        }
        self.pending_disconnect_hold
            .store(q.len() as u64, Ordering::SeqCst);
    }

    fn drain_pending_disconnects_under_guard(
        &self,
        inner: &mut SupervisorInner,
        target_conn_id: Option<u64>,
    ) -> Option<crate::rt_core::requests::lease::LeaseOutcome> {
        use crate::rt_core::requests::lease::{install_lease_disconnect, prepare_lease_disconnect};
        let mut target_outcome = None;
        loop {
            let next = {
                let q = self
                    .pending_disconnects
                    .lock()
                    .unwrap_or_else(|e| e.into_inner());
                q.front().copied()
            };
            let Some(conn) = next else {
                break;
            };
            let prep = prepare_lease_disconnect(conn);
            let outcome = install_lease_disconnect(inner, &prep.request, Instant::now());
            if target_conn_id == Some(conn.id()) {
                target_outcome = Some(outcome);
            }
            let mut q = self
                .pending_disconnects
                .lock()
                .unwrap_or_else(|e| e.into_inner());
            if q.front().is_some_and(|c| c.id() == conn.id()) {
                q.pop_front();
            }
            self.pending_disconnect_hold
                .store(q.len() as u64, Ordering::SeqCst);
        }
        target_outcome
    }

    fn forward_pending_disconnects_to_executor(
        &self,
        handle: &crate::rt_core::executor::RtExecutorHandle,
    ) {
        loop {
            let next = {
                let q = self
                    .pending_disconnects
                    .lock()
                    .unwrap_or_else(|e| e.into_inner());
                q.front().copied()
            };
            let Some(conn) = next else {
                break;
            };
            handle.enqueue_disconnect_detached(conn);
            let mut q = self
                .pending_disconnects
                .lock()
                .unwrap_or_else(|e| e.into_inner());
            if q.front().is_some_and(|c| c.id() == conn.id()) {
                q.pop_front();
            }
            self.pending_disconnect_hold
                .store(q.len() as u64, Ordering::SeqCst);
        }
    }

    fn try_drain_pending_disconnects_legacy(&self) {
        if self.pending_disconnect_hold.load(Ordering::Acquire) == 0 {
            return;
        }
        if let Ok(mut inner) = self.inner.try_write() {
            if !inner.detached && self.owner_mode() == OWNER_LEGACY {
                let _ = self.drain_pending_disconnects_under_guard(&mut inner, None);
            }
        }
    }

    /// Single non-blocking socket-close fence entry point (M1): marks the connection closed in the
    /// lock-free bitset, monotonically fences `local_gen` if non-zero, and dispatches a
    /// non-blocking owner-side fence BEFORE any task abort, join, or cleanup runs.
    ///
    /// - Under `OWNER_EXECUTOR`: immediately disconnects the broker client and pushes priority
    ///   `LifecycleCmd::Disconnect` onto the RT priority intake without awaiting.
    /// - Under `OWNER_LEGACY`: attempts `try_write()` to run `install_lease_disconnect` inline;
    ///   when the write lock is contended, queues the fence and activates fail-closed hold.
    /// - Under `OWNER_TRANSFERRING` / `OWNER_FAILED`: queues the fence to the winning owner and
    ///   activates fail-closed hold so teardown never returns unfenced and RT never inherits a
    ///   stale connection or lease.
    pub fn fence_ws_connection_immediate(
        &self,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
        local_gen: u64,
    ) {
        let _ = self.fence_ws_connection_immediate_outcome(conn, local_gen);
    }

    fn fence_ws_connection_immediate_outcome(
        &self,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
        local_gen: u64,
    ) -> Option<crate::rt_core::requests::lease::LeaseOutcome> {
        use crate::rt_core::requests::lease::{install_lease_disconnect, prepare_lease_disconnect};
        if local_gen > 0 {
            self.atomic_fenced_lease_gen
                .fetch_max(local_gen, Ordering::SeqCst);
            let _ = self.lease_gen_mirror.compare_exchange(
                local_gen,
                0,
                Ordering::SeqCst,
                Ordering::SeqCst,
            );
        }
        if conn.id() != 0 {
            self.closed_bitset.mark_closed(conn.id());
        }
        match self.owner_mode() {
            OWNER_EXECUTOR => {
                if let Some(handle) = self.executor_handle() {
                    if local_gen > 0 {
                        // Non-blocking: `atomic_fenced_lease_gen` is already set above, so if
                        // `servo_dispatcher`'s mutex is currently held, never block teardown; the
                        // owner and dispatcher sync the authoritative fence asynchronously.
                        let _ = handle.try_close_servo(local_gen);
                    }
                    handle.enqueue_disconnect_detached(conn);
                } else {
                    self.enqueue_pending_disconnect(conn);
                }
                None
            }
            OWNER_LEGACY => {
                if let Ok(mut inner) = self.inner.try_write() {
                    if !inner.detached && self.owner_mode() == OWNER_LEGACY {
                        let queued_out =
                            self.drain_pending_disconnects_under_guard(&mut inner, Some(conn.id()));
                        let prep = prepare_lease_disconnect(conn);
                        let direct_out =
                            install_lease_disconnect(&mut inner, &prep.request, Instant::now());
                        return Some(queued_out.unwrap_or(direct_out));
                    }
                }
                self.enqueue_pending_disconnect(conn);
                if self.owner_mode() == OWNER_EXECUTOR {
                    if let Some(handle) = self.executor_handle() {
                        self.forward_pending_disconnects_to_executor(&handle);
                    }
                }
                None
            }
            _ => {
                self.enqueue_pending_disconnect(conn);
                match self.owner_mode() {
                    OWNER_EXECUTOR => {
                        if let Some(handle) = self.executor_handle() {
                            self.forward_pending_disconnects_to_executor(&handle);
                        }
                    }
                    OWNER_LEGACY => {
                        self.try_drain_pending_disconnects_legacy();
                    }
                    _ => {}
                }
                None
            }
        }
    }

    /// Unified owner-routed WS connection disconnect (same `prepare_lease_disconnect` +
    /// `install_lease_disconnect` split on both legacy and executor; priority lossless channel
    /// on executor, never dropped when the ordinary broker ring is full).
    ///
    /// Runs the non-blocking immediate fence FIRST, then waits up to 500 ms for `OWNER_TRANSFERRING`
    /// or a contended legacy write lock to settle. If the lock or transfer is still held past the
    /// bound, the connection remains fenced in `closed_bitset`, queued in `pending_disconnects`,
    /// and held fail-closed (`pending_disconnect_hold > 0`) until the winning owner drains it.
    pub async fn disconnect_ws_connection(
        &self,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
    ) -> crate::rt_core::requests::lease::LeaseOutcome {
        use crate::rt_core::requests::lease::{
            install_lease_disconnect, prepare_lease_disconnect, LeaseOutcome,
        };
        if let Some(outcome) = self.fence_ws_connection_immediate_outcome(conn, 0) {
            return outcome;
        }
        let prep = prepare_lease_disconnect(conn);
        let settle_deadline = Instant::now() + Duration::from_millis(500);
        let mode = loop {
            let m = self.owner_mode();
            if m != OWNER_TRANSFERRING || Instant::now() >= settle_deadline {
                break m;
            }
            tokio::time::sleep(Duration::from_millis(1)).await;
        };
        match mode {
            OWNER_LEGACY => {
                let rem = settle_deadline
                    .saturating_duration_since(Instant::now())
                    .max(Duration::from_millis(50));
                match tokio::time::timeout(rem, self.inner.write()).await {
                    Ok(mut inner) => {
                        if self.check_legacy_under_guard(&inner).is_ok() {
                            let queued_out = self
                                .drain_pending_disconnects_under_guard(&mut inner, Some(conn.id()));
                            let direct_out =
                                install_lease_disconnect(&mut inner, &prep.request, Instant::now());
                            return queued_out.unwrap_or(direct_out);
                        }
                        drop(inner);
                        if let Some(handle) = self.executor_handle() {
                            self.forward_pending_disconnects_to_executor(&handle);
                            if let Ok(outcome) = handle.disconnect_connection_async(conn).await {
                                return outcome;
                            }
                        }
                        LeaseOutcome::Disconnected {
                            released: false,
                            displaced_generation: 0,
                        }
                    }
                    Err(_) => {
                        // Write lock held past bound: keep queued & fail-closed (hold); never unfenced.
                        self.enqueue_pending_disconnect(conn);
                        if self.owner_mode() == OWNER_EXECUTOR {
                            if let Some(handle) = self.executor_handle() {
                                self.forward_pending_disconnects_to_executor(&handle);
                            }
                        }
                        LeaseOutcome::Disconnected {
                            released: false,
                            displaced_generation: 0,
                        }
                    }
                }
            }
            OWNER_EXECUTOR => {
                if let Some(handle) = self.executor_handle() {
                    self.forward_pending_disconnects_to_executor(&handle);
                    if let Ok(outcome) = handle.disconnect_connection_async(conn).await {
                        if let LeaseOutcome::Disconnected {
                            displaced_generation,
                            ..
                        } = outcome
                        {
                            if displaced_generation > 0 {
                                handle.close_servo(displaced_generation);
                            }
                        }
                        return outcome;
                    }
                }
                LeaseOutcome::Disconnected {
                    released: false,
                    displaced_generation: 0,
                }
            }
            _ => {
                // Still `OWNER_TRANSFERRING` past bound (or `OWNER_FAILED`): ensure queued & held
                // fail-closed so the winning owner drains it before any RT handoff or bus output.
                self.enqueue_pending_disconnect(conn);
                if self.owner_mode() == OWNER_EXECUTOR {
                    if let Some(handle) = self.executor_handle() {
                        self.forward_pending_disconnects_to_executor(&handle);
                    }
                }
                LeaseOutcome::Disconnected {
                    released: false,
                    displaced_generation: 0,
                }
            }
        }
    }

    /// Drop-time non-blocking disconnect safety net for aborted WS handler tasks.
    pub fn disconnect_ws_connection_detached(
        &self,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
    ) {
        self.fence_ws_connection_immediate(conn, 0);
    }

    fn map_lease_install_err(code: &'static str) -> ArmError {
        let status = match code {
            "OWNER_TRANSFERRED" | "DISCONNECTED" | "LEASE_LOST" | "NO_LEASE" | "STAMP_REUSED" => 409,
            "DEADLINE_EXPIRED" => 408,
            "MOTION_BUSY" => 503,
            _ => 400,
        };
        ArmError::Rejected {
            status,
            code: code.into(),
            message: format!("Lease operation refused: {code}"),
        }
    }

    fn map_broker_dispatch_err(
        err: crate::rt_core::executor::BrokerDispatchError,
    ) -> ArmError {
        use crate::rt_core::executor::BrokerDispatchError;
        match err {
            BrokerDispatchError::Admit(e) => {
                let code = e.code();
                let status = if code == "DISCONNECTED" { 409 } else { 503 };
                ArmError::Rejected {
                    status,
                    code: code.into(),
                    message: format!("Broker admission refused: {code}"),
                }
            }
            BrokerDispatchError::Stopped => executor_unavailable(),
        }
    }

    /// Unified owner-routed WS `acquire_lease` (`force == false`): off-lock preparation from
    /// published snapshot + single owner-side [`install_lease_acquire`].
    pub async fn acquire_lease_ws(
        &self,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
        label: &str,
        client_seq: u64,
    ) -> Result<crate::rt_core::requests::lease::LeaseOutcome, ArmError> {
        use crate::rt_core::broker::Reply;
        use crate::rt_core::requests::lease::{
            install_lease_acquire, prepare_lease_acquire, LeaseInstallCtx, LeasePrepareError,
            LeaseRequest, DEFAULT_LEASE_REQUEST_DEADLINE,
        };
        let snap = self.fresh_snapshot(None).await?;
        let now = Instant::now();
        let deadline = now + DEFAULT_LEASE_REQUEST_DEADLINE;
        let prep = prepare_lease_acquire(&snap.owner, conn, label, client_seq, now, deadline)
            .map_err(|e| match e {
                LeasePrepareError::InvalidClientId => ArmError::Rejected {
                    status: 400,
                    code: "INVALID_CLIENT_ID".into(),
                    message: "client_id must be 1..=128 bytes".into(),
                },
                LeasePrepareError::InvalidConnection => ArmError::Rejected {
                    status: 400,
                    code: "INVALID_CONNECTION".into(),
                    message: "Invalid connection handle".into(),
                },
                LeasePrepareError::SnapshotUnavailable => observation_unavailable(),
            })?;

        match self.owner_mode() {
            OWNER_LEGACY => {
                let mut inner = self.inner.write().await;
                inner.set_command("acquire_lease");
                self.check_legacy_under_guard(&inner)?;
                let ctx = LeaseInstallCtx {
                    expected_arm: inner.core.id(),
                    expected_boot: inner.boot_id,
                    stamp_arm: prep.arm,
                    stamp_boot: prep.boot,
                    now: Instant::now(),
                    request_id: prep.request_id,
                    client: prep.client,
                    deadline: prep.deadline,
                    enforce_single_use: true,
                };
                let out = install_lease_acquire(&mut inner, &prep.request, &ctx)
                    .map_err(Self::map_lease_install_err)?;
                Ok(out)
            }
            OWNER_EXECUTOR => {
                let Some(handle) = self.executor_handle() else {
                    return Err(executor_unavailable());
                };
                match handle
                    .submit_lease_async(
                        prep.client,
                        prep.client_seq,
                        prep.authority,
                        prep.deadline,
                        LeaseRequest::Acquire(prep.request),
                    )
                    .await
                    .map_err(Self::map_broker_dispatch_err)?
                {
                    Reply::Lease { outcome, .. } => Ok(outcome),
                    Reply::Refused { code, .. } => Err(Self::map_lease_install_err(code)),
                    _ => Err(ArmError::Internal("Unexpected broker reply for LeaseAcquire".into())),
                }
            }
            _ => Err(executor_unavailable()),
        }
    }

    /// Unified owner-routed WS `acquire_lease` (`force == true`): off-lock preparation from
    /// published snapshot + single owner-side [`install_lease_preempt`] with live [`OperatorGrant`].
    pub async fn preempt_lease_ws(
        &self,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
        label: &str,
        operator: crate::auth::OperatorGrant,
        client_seq: u64,
    ) -> Result<crate::rt_core::requests::lease::LeaseOutcome, ArmError> {
        use crate::rt_core::broker::Reply;
        use crate::rt_core::requests::lease::{
            install_lease_preempt, prepare_lease_preempt, LeaseInstallCtx, LeasePrepareError,
            LeaseRequest, DEFAULT_LEASE_REQUEST_DEADLINE,
        };
        let snap = self.fresh_snapshot(None).await?;
        let now = Instant::now();
        let deadline = now + DEFAULT_LEASE_REQUEST_DEADLINE;
        let prep =
            prepare_lease_preempt(&snap.owner, conn, label, operator, client_seq, now, deadline)
                .map_err(|e| match e {
                    LeasePrepareError::InvalidClientId => ArmError::Rejected {
                        status: 400,
                        code: "INVALID_CLIENT_ID".into(),
                        message: "client_id must be 1..=128 bytes".into(),
                    },
                    LeasePrepareError::InvalidConnection => ArmError::Rejected {
                        status: 400,
                        code: "INVALID_CONNECTION".into(),
                        message: "Invalid connection handle".into(),
                    },
                    LeasePrepareError::SnapshotUnavailable => observation_unavailable(),
                })?;

        match self.owner_mode() {
            OWNER_LEGACY => {
                let mut inner = self.inner.write().await;
                inner.set_command("acquire_lease");
                self.check_legacy_under_guard(&inner)?;
                let ctx = LeaseInstallCtx {
                    expected_arm: inner.core.id(),
                    expected_boot: inner.boot_id,
                    stamp_arm: prep.arm,
                    stamp_boot: prep.boot,
                    now: Instant::now(),
                    request_id: prep.request_id,
                    client: prep.client,
                    deadline: prep.deadline,
                    enforce_single_use: true,
                };
                let out = install_lease_preempt(&mut inner, &prep.request, &ctx)
                    .map_err(Self::map_lease_install_err)?;
                Ok(out)
            }
            OWNER_EXECUTOR => {
                let Some(handle) = self.executor_handle() else {
                    return Err(executor_unavailable());
                };
                match handle
                    .submit_lease_async(
                        prep.client,
                        prep.client_seq,
                        prep.authority,
                        prep.deadline,
                        LeaseRequest::Preempt(prep.request),
                    )
                    .await
                    .map_err(Self::map_broker_dispatch_err)?
                {
                    Reply::Lease { outcome, .. } => {
                        if let crate::rt_core::requests::lease::LeaseOutcome::Granted {
                            evicted_generation,
                            ..
                        } = outcome
                        {
                            if evicted_generation > 0 {
                                handle.close_servo(evicted_generation);
                            }
                        }
                        Ok(outcome)
                    }
                    Reply::Refused { code, .. } => Err(Self::map_lease_install_err(code)),
                    _ => Err(ArmError::Internal("Unexpected broker reply for LeasePreempt".into())),
                }
            }
            _ => Err(executor_unavailable()),
        }
    }

    /// Unified owner-routed WS `release_lease`: off-lock preparation from published snapshot +
    /// single owner-side [`install_lease_release`].
    pub async fn release_lease_ws(
        &self,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
        generation: u64,
    ) -> Result<crate::rt_core::requests::lease::LeaseOutcome, ArmError> {
        use crate::rt_core::broker::Reply;
        use crate::rt_core::requests::lease::{
            install_lease_release, prepare_lease_release, LeaseInstallCtx, LeaseRequest,
            DEFAULT_LEASE_REQUEST_DEADLINE,
        };
        let snap = self.fresh_snapshot(None).await?;
        let now = Instant::now();
        let deadline = now + DEFAULT_LEASE_REQUEST_DEADLINE;
        let prep = prepare_lease_release(&snap.owner, conn, generation, now, deadline)
            .map_err(|_| observation_unavailable())?;

        match self.owner_mode() {
            OWNER_LEGACY => {
                let mut inner = self.inner.write().await;
                inner.set_command("release_lease");
                self.check_legacy_under_guard(&inner)?;
                let ctx = LeaseInstallCtx {
                    expected_arm: inner.core.id(),
                    expected_boot: inner.boot_id,
                    stamp_arm: prep.arm,
                    stamp_boot: prep.boot,
                    now: Instant::now(),
                    request_id: prep.request_id,
                    client: prep.client,
                    deadline: prep.deadline,
                    enforce_single_use: true,
                };
                let out = install_lease_release(&mut inner, &prep.request, &ctx)
                    .map_err(Self::map_lease_install_err)?;
                Ok(out)
            }
            OWNER_EXECUTOR => {
                let Some(handle) = self.executor_handle() else {
                    return Err(executor_unavailable());
                };
                match handle
                    .submit_lease_async(
                        prep.client,
                        generation,
                        prep.authority,
                        prep.deadline,
                        LeaseRequest::Release(prep.request),
                    )
                    .await
                    .map_err(Self::map_broker_dispatch_err)?
                {
                    Reply::Lease { outcome, .. } => {
                        if let crate::rt_core::requests::lease::LeaseOutcome::Released {
                            displaced_generation,
                            ..
                        } = outcome
                        {
                            if displaced_generation > 0 {
                                handle.close_servo(displaced_generation);
                            }
                        }
                        Ok(outcome)
                    }
                    Reply::Refused { code, .. } => Err(Self::map_lease_install_err(code)),
                    _ => Err(ArmError::Internal("Unexpected broker reply for LeaseRelease".into())),
                }
            }
            _ => Err(executor_unavailable()),
        }
    }

    /// Unified owner-routed WS `heartbeat`: off-lock preparation from published snapshot +
    /// `ImmediateOp::Heartbeat` in executor mode or `install_legacy_heartbeat` in legacy mode.
    #[allow(clippy::too_many_arguments)]
    pub async fn heartbeat_ws(
        &self,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
        local_generation: u64,
        supplied_generation: u64,
        local_label: Option<&str>,
        supplied_client_id: Option<Option<&str>>,
        supplied_epoch: Option<u64>,
    ) -> Result<crate::rt_core::requests::lease::HeartbeatInstallOutcome, ArmError> {
        use crate::rt_core::broker::{ImmediateOp, Reply};
        use crate::rt_core::requests::lease::{
            install_legacy_heartbeat, prepare_heartbeat, HeartbeatInstallOutcome,
            PreparedHeartbeat, DEFAULT_LEASE_REQUEST_DEADLINE,
        };
        let mode = self.owner_mode();
        if mode == OWNER_TRANSFERRING
            || mode == OWNER_FAILED
            || (mode == OWNER_EXECUTOR && self.executor_handle().is_none_or(|h| h.is_exited()))
        {
            return Err(observation_unavailable());
        }
        if local_generation == 0 {
            return Ok(HeartbeatInstallOutcome::LeaseLost {
                generation: 0,
                reason: None,
            });
        }
        let snap = self.fresh_snapshot(None).await?;
        let now = Instant::now();
        let deadline = now + DEFAULT_LEASE_REQUEST_DEADLINE;
        match prepare_heartbeat(
            &snap.owner,
            conn,
            local_generation,
            supplied_generation,
            local_label,
            supplied_client_id,
            supplied_epoch,
            now,
            deadline,
        ) {
            PreparedHeartbeat::LeaseLost { generation, reason } => {
                Ok(HeartbeatInstallOutcome::LeaseLost { generation, reason })
            }
            PreparedHeartbeat::Submit {
                connection,
                local_generation,
                supplied_epoch,
                mut authority,
                client,
                deadline,
                ..
            } => match self.owner_mode() {
                OWNER_LEGACY => {
                    let mut inner = self.inner.write().await;
                    inner.set_command("heartbeat");
                    self.check_legacy_under_guard(&inner)?;
                    install_legacy_heartbeat(
                        &mut inner,
                        connection,
                        local_generation,
                        supplied_epoch,
                        Instant::now(),
                    )
                    .map_err(Self::map_lease_install_err)
                }
                OWNER_EXECUTOR => {
                    let Some(handle) = self.executor_handle() else {
                        return Err(executor_unavailable());
                    };
                    // Bounded retry (at most 1) only when `stop_generation` or `config_revision`
                    // moved between snapshot preparation and commit: an ordinary Stop or config
                    // change must not cause a spurious `lease_lost` on heartbeat.
                    for attempt in 0..2 {
                        let reply = handle
                            .submit_immediate_async(
                                client,
                                0,
                                authority,
                                deadline,
                                ImmediateOp::Heartbeat,
                            )
                            .await
                            .map_err(Self::map_broker_dispatch_err)?;
                        match reply {
                            Reply::Ack { .. } => return Ok(HeartbeatInstallOutcome::Renewed),
                            Reply::Refused { code: "STOPPED" | "PLAN_STALE", .. } if attempt == 0 => {
                                let fresh = self.fresh_snapshot(None).await?;
                                let mut next_auth = fresh.owner.authority_at(Instant::now());
                                next_auth.lease_generation = local_generation;
                                next_auth.epoch = supplied_epoch.unwrap_or(u64::MAX);
                                authority = next_auth;
                            }
                            Reply::Refused { code: "EPOCH_MISMATCH", .. } => {
                                let latest = self.fresh_snapshot(None).await?;
                                return Ok(HeartbeatInstallOutcome::EpochSync {
                                    epoch: latest.obs.epoch_id,
                                    safety_state: latest.obs.safety_state.clone(),
                                });
                            }
                            Reply::Refused {
                                code: "LEASE_LOST" | "NO_LEASE" | "DISCONNECTED",
                                ..
                            } => {
                                let reason = self
                                    .hub
                                    .latest()
                                    .map_or("expired", |s| {
                                        s.owner.lease_loss_reason(conn.as_str(), local_generation)
                                    });
                                return Ok(HeartbeatInstallOutcome::LeaseLost {
                                    generation: local_generation,
                                    reason: Some(reason),
                                });
                            }
                            Reply::Refused { code, .. } => {
                                return Err(Self::map_lease_install_err(code));
                            }
                            _ => {
                                return Err(ArmError::Internal(
                                    "Unexpected broker reply for Heartbeat".into(),
                                ));
                            }
                        }
                    }
                    Err(ArmError::Rejected {
                        status: 409,
                        code: "STOPPED".into(),
                        message: "Heartbeat superseded by concurrent stop".into(),
                    })
                }
                _ => Err(executor_unavailable()),
            },
        }
    }

    /// Unified owner-routed WS `/api/v1/torque` and `/api/v1/arm` across legacy and RT executor.
    ///
    /// Legacy runs off-lock snapshot + short write lock using `install_torque_on_legacy` (consuming
    /// A1's `Admitted<'_>`) or `install_torque_off_legacy` + `StateHub` publication.
    /// Executor prepares off-thread from `StateHub`, submits `TransitionOp::{TorqueOn, TorqueOff}`
    /// via `BrokerDispatcher::submit_action`, and awaits BOTH `Ack` AND confirmed `Terminal` before
    /// returning `Accepted` (never acknowledging on early `Ack` before hardware confirmation).
    #[allow(clippy::too_many_arguments)]
    pub async fn torque_ws(
        &self,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
        local_generation: u64,
        supplied_epoch: Option<u64>,
        request_id_valid: bool,
        parsed_request_id: u64,
        enable: Result<bool, &'static str>,
        auth_policy: crate::rt_core::requests::lease::OwnerAuthPolicy,
        operator: crate::auth::OperatorGrant,
        stamp: AdmissionStamp,
    ) -> Result<crate::rt_core::requests::lease::TorqueWsOutcome, ArmError> {
        let override_ms = self.legacy_torque_deadline_override_ms.load(Ordering::Acquire);
        let request_deadline = if override_ms > 0 {
            Duration::from_millis(override_ms)
        } else {
            crate::rt_core::requests::lease::DEFAULT_LEASE_REQUEST_DEADLINE
        };
        self.torque_ws_with_deadline(
            conn,
            local_generation,
            supplied_epoch,
            request_id_valid,
            parsed_request_id,
            enable,
            auth_policy,
            operator,
            stamp,
            request_deadline,
        )
        .await
    }

    #[doc(hidden)]
    #[allow(clippy::too_many_arguments)]
    pub async fn torque_ws_with_deadline(
        &self,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
        local_generation: u64,
        supplied_epoch: Option<u64>,
        request_id_valid: bool,
        parsed_request_id: u64,
        enable: Result<bool, &'static str>,
        auth_policy: crate::rt_core::requests::lease::OwnerAuthPolicy,
        operator: crate::auth::OperatorGrant,
        stamp: AdmissionStamp,
        request_deadline: Duration,
    ) -> Result<crate::rt_core::requests::lease::TorqueWsOutcome, ArmError> {
        use crate::rt_core::broker::{Outcome, Reply, TransitionOp};
        use crate::rt_core::requests::lease::{
            check_torque_owner_preflight, map_torque_code_to_api_error, prepare_torque,
            prepare_torque_ws, OwnerAuthPolicy, TorquePreflightError, TorqueWsOutcome,
        };
        let map_preflight_err = |err: TorquePreflightError| -> TorqueWsOutcome {
            match err {
                TorquePreflightError::LeaseLost { epoch_id } => {
                    TorqueWsOutcome::LeaseLost { epoch_id }
                }
                TorquePreflightError::EpochMismatch { epoch_id } => {
                    TorqueWsOutcome::EpochMismatch { epoch_id }
                }
                TorquePreflightError::InvalidRequestId => TorqueWsOutcome::InvalidRequestId,
                TorquePreflightError::OperatorRequired {
                    operation,
                    epoch_id,
                } => TorqueWsOutcome::Rejected {
                    error: crate::api::rest::ApiError::operator_required(operation),
                    epoch_id,
                },
                TorquePreflightError::MotionBusy { epoch_id } => TorqueWsOutcome::Rejected {
                    error: crate::api::rest::ApiError(
                        axum::http::StatusCode::CONFLICT,
                        "a calibration apply, restore or motor zero is running".into(),
                        "MOTION_BUSY".into(),
                    ),
                    epoch_id,
                },
                TorquePreflightError::InvalidBody { message, epoch_id } => {
                    TorqueWsOutcome::InvalidBody { message, epoch_id }
                }
            }
        };
        // Off-lock typed preparation shared by both legacy and RT executor owners.
        let snap = self.fresh_snapshot(None).await?;
        let now = Instant::now();
        let deadline = now + request_deadline;
        let prep_ws = prepare_torque_ws(
            &snap.owner,
            conn,
            local_generation,
            supplied_epoch,
            request_id_valid,
            parsed_request_id,
            enable,
            auth_policy,
            operator,
            stamp,
            now,
            deadline,
        )
        .map_err(|_| observation_unavailable())?;
        match self.owner_mode() {
            OWNER_LEGACY => {
                // Drop the pre-admission snapshot immediately so the off-lock wait never pins a
                // `StateHub` publication slot during the legacy torque transition.
                drop(snap);
                use crate::supervisor::driver_transitions::{
                    LegacyTorqueFailure, LegacyTorqueOutcome, LegacyTorqueSlot, LegacyTorqueStart,
                };
                let map_legacy_failure = |failure: LegacyTorqueFailure,
                                          epoch_id: u64,
                                          safety_state: &SafetyState,
                                          joint_names: &[String]|
                 -> TorqueWsOutcome {
                    match failure {
                        LegacyTorqueFailure::TorqueOffWouldDrop(refusal) => {
                            let err = ArmError::Safety(
                                refusal.into_safety_error(joint_names),
                            );
                            TorqueWsOutcome::Rejected {
                                error: err.into(),
                                epoch_id,
                            }
                        }
                        LegacyTorqueFailure::EnableFailed {
                            detail,
                            feedback_unavailable,
                            estop_interrupted,
                        } => {
                            // Preserve exact legacy WS wire mapping (`SafetyError::ClearFaultRejected`
                            // -> `CLEAR_FAULT_REJECTED` with `safety_state: "Fault"`), while Safety
                            // still latches `FaultCode::EnableFailed` internally.
                            let err = if feedback_unavailable {
                                ArmError::Safety(SafetyError::FeedbackUnavailable)
                            } else if estop_interrupted {
                                ArmError::Safety(SafetyError::EStopActive)
                            } else {
                                let msg = if detail.as_str().is_empty() {
                                    "Hardware torque enable failed".to_string()
                                } else {
                                    detail.as_str().to_string()
                                };
                                ArmError::Safety(SafetyError::ClearFaultRejected(msg))
                            };
                            TorqueWsOutcome::Rejected {
                                error: err.into(),
                                epoch_id,
                            }
                        }
                        LegacyTorqueFailure::DisableFailed { detail } => {
                            // Preserve exact legacy WS wire mapping (`SafetyError::ClearFaultRejected`
                            // -> `CLEAR_FAULT_REJECTED` with `safety_state: "Fault"`), while Safety
                            // still latches `FaultCode::DriverWriteFailed` internally.
                            let msg = if detail.as_str().is_empty() {
                                "Torque disable transmission failed".to_string()
                            } else {
                                detail.as_str().to_string()
                            };
                            TorqueWsOutcome::Rejected {
                                error: ArmError::Safety(SafetyError::ClearFaultRejected(msg)).into(),
                                epoch_id,
                            }
                        }
                        LegacyTorqueFailure::Stopped => TorqueWsOutcome::Rejected {
                            error: ArmError::Safety(SafetyError::Stopped).into(),
                            epoch_id,
                        },
                        LegacyTorqueFailure::LeaseLost => TorqueWsOutcome::LeaseLost { epoch_id },
                        LegacyTorqueFailure::EpochMismatch => {
                            TorqueWsOutcome::EpochMismatch { epoch_id }
                        }
                        LegacyTorqueFailure::FeedbackUnavailable => TorqueWsOutcome::Rejected {
                            error: ArmError::Safety(SafetyError::FeedbackUnavailable).into(),
                            epoch_id,
                        },
                        LegacyTorqueFailure::EStopActive => TorqueWsOutcome::Rejected {
                            error: ArmError::Safety(SafetyError::EStopActive).into(),
                            epoch_id,
                        },
                        LegacyTorqueFailure::StaticCode(code) => TorqueWsOutcome::Rejected {
                            error: map_torque_code_to_api_error(code, Some(safety_state)),
                            epoch_id,
                        },
                    }
                };

                #[cfg(not(target_arch = "wasm32"))]
                if prep_ws.enable == Some(true)
                    && self
                        .published_config()
                        .await?
                        .driver_supports_nonblocking_mode_switch
                {
                    let needs_handback = {
                        let mut inner = self.inner.write().await;
                        let io_scope = crate::diagnostics::tick::HoldIoScope::enter();
                        inner.set_command("/api/v1/torque");
                        inner.set_direction("on");
                        if let Err(e) = self.check_legacy_under_guard(&inner) {
                            inner.set_io_timing(io_scope.finish());
                            return Err(e);
                        }
                        if let Err(err) = check_torque_owner_preflight(
                            &mut inner,
                            prep_ws.connection,
                            prep_ws.local_generation,
                            prep_ws.supplied_epoch,
                            prep_ws.request_id_valid,
                            prep_ws.body_error,
                            prep_ws.auth_policy,
                            prep_ws.operator,
                            Instant::now(),
                        ) {
                            inner.set_io_timing(io_scope.finish());
                            return Ok(map_preflight_err(err));
                        }
                        if Instant::now() >= deadline {
                            let epoch_id = inner.safety.epoch_id();
                            let safety_state = inner.safety.state().clone();
                            inner.set_io_timing(io_scope.finish());
                            return Ok(map_legacy_failure(
                                LegacyTorqueFailure::StaticCode("DEADLINE_EXPIRED"),
                                epoch_id,
                                &safety_state,
                                &[],
                            ));
                        }
                        let active =
                            inner.driver_supports_nonblocking_mode_switch() && inner.mit_active();
                        inner.set_io_timing(io_scope.finish());
                        active
                    };
                    if needs_handback {
                        let lease_holder = (prep_ws.local_generation != 0).then(|| {
                            (
                                prep_ws.connection.to_holder_label(),
                                prep_ws.local_generation,
                            )
                        });
                        let mut qual_stamp = prep_ws.stamp;
                        qual_stamp.request_id = next_admission_request_id();
                        qual_stamp.connection_id = prep_ws.connection.id();
                        qual_stamp.lease_gen = prep_ws.local_generation;
                        if let Some(ep) = prep_ws.supplied_epoch {
                            qual_stamp.estop_epoch = ep;
                        }
                        if let Err(err) = crate::rt_core::requests::config::run_executor_mit_hand_back(
                            self,
                            crate::supervisor::MitHandBackReason::Rearm,
                            qual_stamp,
                            lease_holder,
                            crate::auth::OperatorRequirement::NotRequired,
                            crate::rt_core::broker::ClientId(prep_ws.connection.id()),
                            Duration::from_secs(2),
                        )
                        .await
                        {
                            let epoch_id = self.inner.read().await.safety.epoch_id();
                            return Ok(TorqueWsOutcome::Rejected {
                                error: err.into(),
                                epoch_id,
                            });
                        }
                    }
                }

                let (enabled, start_res, pre_epoch, joint_names, tick_seq_counter, mut last_tick_seq) = {
                    let mut inner = self.inner.write().await;
                    let io_scope = crate::diagnostics::tick::HoldIoScope::enter();
                    inner.set_command("/api/v1/torque");
                    if let Ok(en) = enable {
                        inner.set_direction(if en { "on" } else { "off" });
                    }
                    if let Err(e) = self.check_legacy_under_guard(&inner) {
                        inner.set_io_timing(io_scope.finish());
                        return Err(e);
                    }
                    if let Err(err) = check_torque_owner_preflight(
                        &mut inner,
                        prep_ws.connection,
                        prep_ws.local_generation,
                        prep_ws.supplied_epoch,
                        prep_ws.request_id_valid,
                        prep_ws.body_error,
                        prep_ws.auth_policy,
                        prep_ws.operator,
                        Instant::now(),
                    ) {
                        inner.set_io_timing(io_scope.finish());
                        return Ok(map_preflight_err(err));
                    }
                    let enabled = prep_ws.enable.expect("checked by check_torque_owner_preflight");
                    if Instant::now() >= deadline {
                        let epoch_id = inner.safety.epoch_id();
                        let safety_state = inner.safety.state().clone();
                        inner.set_io_timing(io_scope.finish());
                        return Ok(map_legacy_failure(
                            LegacyTorqueFailure::StaticCode("DEADLINE_EXPIRED"),
                            epoch_id,
                            &safety_state,
                            &[],
                        ));
                    }
                    let joint_names = if !enabled {
                        inner.config.joint_names.clone()
                    } else {
                        Vec::new()
                    };
                    let tick_seq_counter = Arc::clone(&inner.control_release_seq_atomic);
                    let last_tick_seq = tick_seq_counter.load(Ordering::Acquire);
                    let res = if enabled {
                        if inner.driver.is_some() && !inner.driver_supports_rt_executor_nonblocking() {
                            // Serial & RobStride drivers stay on the untouched synchronous
                            // `enable_torque` path (full bus read, calibration, range check, and
                            // measured-goal pre-seed with blocking driver lock).
                            inner
                                .check_admission(prep_ws.stamp)
                                .and_then(SupervisorInner::enable_torque)
                                .map(|_| LegacyTorqueStart::ImmediateAccepted {
                                    epoch_id: inner.safety.epoch_id(),
                                    safety_state: inner.safety.state().clone(),
                                    is_torqued: inner.is_torqued,
                                    control_mode_gravity_comp: inner.gravity_comp_active(),
                                })
                                .map_err(ArmError::from)
                        } else {
                            inner
                                .check_admission(prep_ws.stamp)
                                .and_then(|a| {
                                    SupervisorInner::begin_enable_torque_transition_legacy(a, deadline)
                                })
                                .map_err(ArmError::from)
                        }
                    } else {
                        inner
                            .begin_disable_torque_transition_legacy(deadline)
                            .map_err(ArmError::from)
                    };
                    let epoch_id = inner.safety.epoch_id();
                    inner.set_io_timing(io_scope.finish());
                    (enabled, res, epoch_id, joint_names, tick_seq_counter, last_tick_seq)
                };

                let (token, slot_idx, slot) = match start_res {
                    Ok(LegacyTorqueStart::ImmediateAccepted {
                        epoch_id,
                        safety_state,
                        is_torqued,
                        control_mode_gravity_comp,
                    }) => {
                        return Ok(TorqueWsOutcome::Accepted {
                            epoch_id,
                            safety_state,
                            is_torqued,
                            control_mode_gravity_comp,
                        });
                    }
                    Ok(LegacyTorqueStart::ImmediateRejected {
                        failure,
                        epoch_id,
                        safety_state,
                    }) => {
                        return Ok(map_legacy_failure(
                            failure,
                            epoch_id,
                            &safety_state,
                            &joint_names,
                        ));
                    }
                    Ok(LegacyTorqueStart::Pending {
                        token,
                        slot_idx,
                        slot,
                    }) => (token, slot_idx, slot),
                    Err(err) => {
                        return Ok(TorqueWsOutcome::Rejected {
                            error: err.into(),
                            epoch_id: pre_epoch,
                        });
                    }
                };

                struct LegacySlotGuard {
                    slot: Arc<LegacyTorqueSlot>,
                    token: u64,
                    consumed: bool,
                }
                impl Drop for LegacySlotGuard {
                    fn drop(&mut self) {
                        if !self.consumed {
                            self.slot.abandon(self.token);
                        }
                    }
                }
                let mut slot_guard = LegacySlotGuard {
                    slot: Arc::clone(&slot),
                    token,
                    consumed: false,
                };

                // Bounded off-lock waiter polling + headless fallback:
                // - Normal live control loop (50 Hz): `tick_shared` publishes wait-free to the
                //   atomic slot and calls `self.legacy_torque_notify.notify_waiters()` strictly
                //   OFF-LOCK after dropping `SupWriteGuard`; the 5 ms poll cap catches any
                //   non-tick settlement (Stop / E-stop / lease fence / handoff race) cleanly.
                // - Headless mode (no ticking control loop): fallback is throttled to >= 25 ms AND
                //   re-verifies `inner.control_release_seq == last_tick_seq` under the write guard
                //   so a live control tick that ran while queued keeps single progress ownership.
                const WAIT_POLL_INTERVAL: Duration = Duration::from_millis(5);
                // Wait > one 50 Hz period (25 ms) before assuming headless mode (no live ticker);
                // once confirmed headless, step every 5 ms so multi-phase Damiao POS_FORCE
                // handshakes complete well inside phase/freshness bounds while still keeping every
                // write guard <= 500 us and yielding immediately if a live ticker resumes.
                const FALLBACK_IDLE_DETECT: Duration = Duration::from_millis(25);
                const FALLBACK_HEADLESS_INTERVAL: Duration = Duration::from_millis(5);
                const FALLBACK_STEP_BUDGET: Duration = Duration::from_micros(500);
                let mut last_fallback_step = Instant::now();
                let mut headless_active = false;

                let outcome = loop {
                    if let Some(done) = slot.take_settled(token) {
                        slot_guard.consumed = true;
                        break done;
                    }
                    let now_wait = Instant::now();
                    if now_wait >= deadline {
                        let (done, needs_headless_drain, settled_any, cur_seq) = {
                            let mut inner = self.inner.write().await;
                            let io_scope = crate::diagnostics::tick::HoldIoScope::enter();
                            inner.set_command("/api/v1/torque");
                            inner.set_direction(if enabled { "on" } else { "off" });
                            let out = if inner.detached {
                                slot.take_settled(token)
                                    .unwrap_or(LegacyTorqueOutcome::OwnerTransferred)
                            } else {
                                inner.timeout_legacy_torque_transition(token, slot_idx, enabled)
                            };
                            let needs_drain =
                                !inner.detached && inner.has_detached_torque_transition();
                            let settled_any =
                                std::mem::replace(&mut inner.legacy_torque_settled_this_tick, false);
                            let seq = inner.control_release_seq;
                            inner.set_io_timing(io_scope.finish());
                            (out, needs_drain, settled_any, seq)
                        };
                        if settled_any {
                            self.legacy_torque_notify.notify_waiters();
                        }
                        if needs_headless_drain {
                            self.spawn_headless_detached_drain_if_needed(cur_seq);
                        }
                        slot_guard.consumed = true;
                        break done;
                    }
                    let notified = self.legacy_torque_notify.notified();
                    if let Some(done) = slot.take_settled(token) {
                        slot_guard.consumed = true;
                        break done;
                    }
                    let rem = deadline.saturating_duration_since(now_wait);
                    let wait_for = rem.min(WAIT_POLL_INTERVAL);
                    let _ = tokio::time::timeout(wait_for, notified).await;
                    if let Some(done) = slot.take_settled(token) {
                        slot_guard.consumed = true;
                        break done;
                    }
                    let cur_tick_seq = tick_seq_counter.load(Ordering::Acquire);
                    let step_interval = if headless_active {
                        FALLBACK_HEADLESS_INTERVAL
                    } else {
                        FALLBACK_IDLE_DETECT
                    };
                    if cur_tick_seq != last_tick_seq {
                        last_tick_seq = cur_tick_seq;
                        last_fallback_step = Instant::now();
                        headless_active = false;
                    } else if last_fallback_step.elapsed() >= step_interval
                        && Instant::now() < deadline
                    {
                        last_fallback_step = Instant::now();
                        let (expired_out, needs_headless_drain, settled_any, observed_seq, did_headless_step) = {
                            let mut inner = self.inner.write().await;
                            let io_scope = crate::diagnostics::tick::HoldIoScope::enter();
                            inner.set_command("/api/v1/torque");
                            inner.set_direction(if enabled { "on" } else { "off" });
                            let step_now = Instant::now();
                            let mut stepped = false;
                            let out = if step_now >= deadline {
                                Some(if inner.detached {
                                    slot.take_settled(token)
                                        .unwrap_or(LegacyTorqueOutcome::OwnerTransferred)
                                } else {
                                    inner.timeout_legacy_torque_transition(
                                        token, slot_idx, enabled,
                                    )
                                })
                            } else {
                                // Re-verify under the write guard that no live control tick ran
                                // while we waited to acquire `SupervisorLock::write`, then advance
                                // `control_release_seq` so Damiao's per-release TX budget refreshes
                                // cleanly on each headless fallback step.
                                if !inner.detached
                                    && inner.control_release_seq == last_tick_seq
                                    && inner.has_legacy_torque_transition_token(token)
                                {
                                    inner.control_release_seq =
                                        inner.control_release_seq.wrapping_add(1).max(1);
                                    inner
                                        .control_release_seq_atomic
                                        .store(inner.control_release_seq, Ordering::Release);
                                    let budget = crate::rt_core::ops::ServiceBudget {
                                        now: step_now,
                                        deadline: step_now + FALLBACK_STEP_BUDGET,
                                        max_rx_frames: crate::rt_core::ops::RX_FRAME_BUDGET,
                                    };
                                    #[cfg(not(target_arch = "wasm32"))]
                                    {
                                        let live_intent = inner.shadow_live_intent();
                                        let live_is_torqued = inner.is_torqued;
                                        let live_stopped = inner.safety.state().is_stopped();
                                        inner.torque_shadow.enqueue_at(
                                            step_now,
                                            crate::torque::shadow::ShadowInputKind::TransitionWake {
                                                live_intent,
                                                live_is_torqued,
                                                live_stopped,
                                            },
                                        );
                                    }
                                    let _ = inner.service_torque_transition(&budget);
                                    inner.flush_torque_shadow();
                                    stepped = true;
                                }
                                None
                            };
                            let needs_drain =
                                !inner.detached && inner.has_detached_torque_transition();
                            let settled_any =
                                std::mem::replace(&mut inner.legacy_torque_settled_this_tick, false);
                            let seq = inner.control_release_seq;
                            inner.set_io_timing(io_scope.finish());
                            (out, needs_drain, settled_any, seq, stepped)
                        };
                        if settled_any {
                            self.legacy_torque_notify.notify_waiters();
                        }
                        if needs_headless_drain {
                            self.spawn_headless_detached_drain_if_needed(observed_seq);
                        }
                        if let Some(done) = expired_out {
                            slot_guard.consumed = true;
                            break done;
                        }
                        headless_active = did_headless_step;
                        last_tick_seq = observed_seq;
                    }
                };
                // Schedule bounded headless cleanup if ANY settlement path (contended Stop,
                // E-stop, lease loss, or timeout) left a detached transition pending (R5 M4):
                // if a live ticker is running, the drain task exits on its first wake once
                // `control_release_seq` advances.
                if self.detached_torque_pending_atomic.load(Ordering::Acquire) {
                    let cur_seq = tick_seq_counter.load(Ordering::Acquire);
                    self.spawn_headless_detached_drain_if_needed(cur_seq);
                }

                match outcome {
                    LegacyTorqueOutcome::Accepted {
                        epoch_id,
                        safety_state,
                        is_torqued,
                        control_mode_gravity_comp,
                    } => Ok(TorqueWsOutcome::Accepted {
                        epoch_id,
                        safety_state: (*safety_state).clone(),
                        is_torqued,
                        control_mode_gravity_comp,
                    }),
                    LegacyTorqueOutcome::Failed {
                        failure,
                        epoch_id,
                        safety_state,
                    } => Ok(map_legacy_failure(
                        failure,
                        epoch_id,
                        &safety_state,
                        &joint_names,
                    )),
                    LegacyTorqueOutcome::OwnerTransferred => {
                        Err(ArmError::Safety(SafetyError::OwnerTransferred))
                    }
                }
            }
            OWNER_EXECUTOR => {
                let Some(handle) = self.executor_handle() else {
                    return Err(executor_unavailable());
                };
                // Malformed requests or rows carrying an owner auth policy evaluate LIVE owner
                // authority through the single shared `check_torque_owner_preflight` (never stale
                // snapshot-only checks), preserving exact legacy/executor precedence parity.
                if prep_ws.broker.is_none() || prep_ws.auth_policy != OwnerAuthPolicy::None {
                    let preflight = handle
                        .lease_torque_preflight_async(
                            prep_ws.connection,
                            prep_ws.local_generation,
                            prep_ws.supplied_epoch,
                            prep_ws.request_id_valid,
                            prep_ws.body_error,
                            prep_ws.auth_policy,
                            prep_ws.operator,
                        )
                        .await
                        .map_err(ArmError::from)?;
                    if let Err(err) = preflight {
                        return Ok(map_preflight_err(err));
                    }
                }
                let enabled = prep_ws.enable.expect("checked above");
                let op = if enabled {
                    TransitionOp::TorqueOn
                } else {
                    TransitionOp::TorqueOff
                };
                let mut prep = prep_ws.broker.expect("checked above");

                // MIT-configured profiles on OWNER_EXECUTOR: run off-RT qualification preflight
                // (sched::mit_refusal + optional rearm handback + ReadIdentityForEnable +
                // mit::check_gate + admission-bound MitGateProof) BEFORE submitting TorqueOn.
                if enabled && crate::mit::mit_joint_mask(&self.published_config().await?.config) != 0 {
                    let lease_holder = (prep_ws.local_generation != 0).then(|| {
                        (
                            prep_ws.connection.to_holder_label(),
                            prep_ws.local_generation,
                        )
                    });
                    let mut qual_stamp = prep.stamp;
                    qual_stamp.request_id = prep.client_seq;
                    qual_stamp.connection_id = prep.client.0;
                    qual_stamp.lease_gen = prep.authority.lease_generation;
                    qual_stamp.estop_epoch = prep.authority.epoch;
                    qual_stamp.stop_gen = prep.authority.stop_generation;
                    if let Err(err) = crate::mit::run_mit_qualification_for_enable_executor(
                        self,
                        qual_stamp,
                        lease_holder,
                        prep_ws.operator,
                        prep.client,
                    )
                    .await
                    {
                        let epoch_id = self
                            .hub
                            .latest()
                            .map_or(snap.obs.epoch_id, |s| s.obs.epoch_id);
                        return Ok(TorqueWsOutcome::Rejected {
                            error: ArmError::Safety(err).into(),
                            epoch_id,
                        });
                    }
                }

                for attempt in 0..2 {
                    let reply = handle
                        .submit_transition_async(
                            prep.client,
                            prep.client_seq,
                            prep.authority,
                            prep.deadline,
                            op,
                        )
                        .await
                        .map_err(Self::map_broker_dispatch_err)?;
                    match reply {
                        Reply::Refused {
                            code: "PLAN_STALE",
                            ..
                        } if attempt == 0 => {
                            let fresh = self.fresh_snapshot(None).await?;
                            prep = prepare_torque(
                                &fresh.owner,
                                prep_ws.connection,
                                prep_ws.local_generation,
                                prep_ws.supplied_epoch,
                                enabled,
                                prep_ws.parsed_request_id,
                                prep_ws.stamp,
                                Instant::now(),
                                deadline,
                            )
                            .map_err(|_| observation_unavailable())?;
                        }
                        Reply::Refused {
                            code: "LEASE_LOST" | "NO_LEASE" | "DISCONNECTED",
                            ..
                        } => {
                            let epoch_id = self
                                .hub
                                .latest()
                                .map_or(snap.obs.epoch_id, |s| s.obs.epoch_id);
                            return Ok(TorqueWsOutcome::LeaseLost { epoch_id });
                        }
                        Reply::Refused {
                            code: "EPOCH_MISMATCH",
                            ..
                        } => {
                            let epoch_id = self
                                .hub
                                .latest()
                                .map_or(snap.obs.epoch_id, |s| s.obs.epoch_id);
                            return Ok(TorqueWsOutcome::EpochMismatch { epoch_id });
                        }
                        Reply::Refused { code, .. } => {
                            let post = self.hub.latest();
                            let epoch_id = post
                                .as_ref()
                                .map_or(snap.obs.epoch_id, |s| s.obs.epoch_id);
                            let st = post.as_ref().map(|s| &s.obs.safety_state);
                            return Ok(TorqueWsOutcome::Rejected {
                                error: map_torque_code_to_api_error(code, st),
                                epoch_id,
                            });
                        }
                        Reply::Terminal {
                            outcome: Outcome::Completed,
                            ..
                        } => {
                            let post = self.hub.latest().unwrap_or(snap);
                            return Ok(TorqueWsOutcome::Accepted {
                                epoch_id: post.obs.epoch_id,
                                safety_state: post.obs.safety_state.clone(),
                                is_torqued: post.obs.is_torqued,
                                control_mode_gravity_comp: post.owner.gravity_comp_active,
                            });
                        }
                        Reply::Terminal { outcome, .. } => {
                            let code = match outcome {
                                Outcome::Failed(c) | Outcome::Cancelled(c) => c,
                                Outcome::Stopped => "STOPPED",
                                Outcome::Superseded => "SUPERSEDED",
                                Outcome::Completed => unreachable!(),
                            };
                            let post = self.hub.latest();
                            let epoch_id = post
                                .as_ref()
                                .map_or(snap.obs.epoch_id, |s| s.obs.epoch_id);
                            if matches!(code, "LEASE_LOST" | "NO_LEASE" | "DISCONNECTED") {
                                return Ok(TorqueWsOutcome::LeaseLost { epoch_id });
                            }
                            if code == "EPOCH_MISMATCH" {
                                return Ok(TorqueWsOutcome::EpochMismatch { epoch_id });
                            }
                            let st = post.as_ref().map(|s| &s.obs.safety_state);
                            return Ok(TorqueWsOutcome::Rejected {
                                error: map_torque_code_to_api_error(code, st),
                                epoch_id,
                            });
                        }
                        _ => {
                            return Err(ArmError::Internal(
                                "Unexpected broker reply for torque transition".into(),
                            ));
                        }
                    }
                }
                Err(ArmError::Rejected {
                    status: 409,
                    code: "PLAN_STALE".into(),
                    message: "Torque transition superseded by concurrent config change".into(),
                })
            }
            _ => Err(executor_unavailable()),
        }
    }

    /// Unified owner-routed WS `/api/v1/safety/clear_fault` across legacy and RT executor owners.
    ///
    /// Both owners share the exact same authority, pending-E-stop, live [`OperatorGrant`] policy
    /// (supplied by the WS dispatch table), and `motor_zero_busy` precedence in
    /// `requests::lease::install_lease_clear_fault_*`.
    #[allow(clippy::too_many_arguments)]
    pub async fn clear_fault_ws(
        &self,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
        local_generation: u64,
        supplied_epoch: Option<u64>,
        request_id_valid: bool,
        body_error: Option<&'static str>,
        mit_recovery: bool,
        auth_policy: crate::rt_core::requests::lease::OwnerAuthPolicy,
        operator: crate::auth::OperatorGrant,
    ) -> Result<
        Result<crate::rt_core::requests::lease::LeaseClearFaultOutcome, (ArmError, u64)>,
        ArmError,
    > {
        use crate::rt_core::requests::lease::install_lease_clear_fault_legacy;
        match self.owner_mode() {
            OWNER_LEGACY => {
                let mut inner = self.inner.write().await;
                inner.set_command("/api/v1/safety/clear_fault");
                self.check_legacy_under_guard(&inner)?;
                let res = install_lease_clear_fault_legacy(
                    &mut inner,
                    conn,
                    local_generation,
                    supplied_epoch,
                    request_id_valid,
                    body_error,
                    mit_recovery,
                    auth_policy,
                    operator,
                    Instant::now(),
                );
                Ok(res)
            }
            OWNER_EXECUTOR => {
                let Some(handle) = self.executor_handle() else {
                    return Err(executor_unavailable());
                };
                let recovery_started_at = mit_recovery.then(Instant::now);
                let mut final_attempt = false;
                loop {
                    let out = handle
                        .lease_clear_fault_async(
                            conn,
                            local_generation,
                            supplied_epoch,
                            request_id_valid,
                            body_error,
                            mit_recovery,
                            recovery_started_at,
                            auth_policy,
                            operator,
                        )
                        .await
                        .map_err(ArmError::from)?;
                    let incomplete = matches!(
                        out,
                        crate::rt_core::requests::lease::LeaseClearFaultOutcome::ClearFailed {
                            err: crate::safety::ClearFaultError::Rejected(
                                crate::safety::ClearRefusal::MitRecoveryIncomplete
                            ),
                            ..
                        }
                    );
                    if !incomplete || final_attempt {
                        return Ok(Ok(out));
                    }
                    let Some(started) = recovery_started_at else {
                        return Ok(Ok(out));
                    };
                    if Instant::now().saturating_duration_since(started) >= Duration::from_secs(1) {
                        // One final RT step after the 1 s Rule A′ lifetime expires so the driver
                        // observes `DisableRequestExpired` and resets recovery state cleanly.
                        final_attempt = true;
                    } else {
                        tokio::time::sleep(Duration::from_millis(5)).await;
                    }
                }
            }
            _ => Err(executor_unavailable()),
        }
    }

    /// Fast-path lock-free check whether E-stop is currently latched.
    pub(crate) fn is_estop_latched(&self) -> bool {
        self.e_stop_latched.load(Ordering::SeqCst)
    }

    /// Fast-path latest published snapshot for high-rate executor WS servo intake: returns the
    /// current `StateHub` snapshot without waiting once the executor has published at least once,
    /// falling back to `fresh_snapshot(None)` only before the first post-handoff publish.
    pub(crate) async fn latest_executor_snapshot(
        &self,
    ) -> Result<Arc<crate::rt_core::publish::Snapshot>, ArmError> {
        if self.owner_mode() != OWNER_EXECUTOR {
            return Err(executor_unavailable());
        }
        let want = self.executor_base_gen.load(Ordering::Acquire).saturating_add(1);
        let now = Instant::now();
        if let Some(s) = self.hub.latest() {
            if Self::answers(&s, want, now) {
                return Ok(s);
            }
        }
        self.fresh_snapshot(None).await
    }

    fn executor_lease_active(&self) -> bool {
        if self.owner_mode() != OWNER_EXECUTOR {
            return false;
        }
        let Some(handle) = self.executor_handle() else {
            return false;
        };
        if handle.is_exited() {
            return false;
        }
        let want = self.executor_base_gen.load(Ordering::Acquire).saturating_add(1);
        let now = Instant::now();
        self.hub
            .latest()
            .is_some_and(|s| Self::answers(&s, want, now) && s.owner.is_lease_active_at(now))
    }

    pub async fn is_lease_active(&self) -> bool {
        if self.owner_mode() == OWNER_EXECUTOR {
            return self.executor_lease_active();
        }
        let inner = self.inner.read().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            return inner.is_lease_active();
        }
        drop(inner);
        self.executor_lease_active()
    }

    pub async fn validate_servo_lease(&self, client_id: &str, generation: u64) -> bool {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_err() {
            return false;
        }
        inner.validate_servo_lease(client_id, generation)
    }

    pub async fn set_torques(&self, torques: Vec<f32>) {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            inner.torques = torques;
        }
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
        if self.check_legacy_under_guard(&inner).is_ok() {
            inner.gravity_compensated = enabled;
        }
    }

    pub async fn get_gravity_scale(&self) -> f32 {
        let inner = self.inner.read().await;
        inner.gravity_scale
    }

    pub async fn set_gravity_scale(&self, scale: f32) {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            inner.gravity_scale = scale.clamp(0.0, 1.0);
        }
    }

    pub async fn get_gravity_torques_ramped(&self) -> Vec<f32> {
        let inner = self.inner.read().await;
        inner.gravity_torques_ramped.clone()
    }

    pub async fn servo(&self, stamp: AdmissionStamp, targets: Vec<f32>, epoch_id: u64) -> Result<(), ArmError> {
        if self.e_stop_latched.load(Ordering::SeqCst) {
            return Err(ArmError::Safety(SafetyError::EStopActive));
        }
        let mut inner = self.inner.write().await;
        self.check_legacy_under_guard(&inner)?;
        if inner.estop_pending() {
            return Err(ArmError::Safety(SafetyError::EStopActive));
        }
        let admitted = inner.check_admission(stamp)?;
        SupervisorInner::servo(admitted, targets, epoch_id)?;
        Ok(())
    }

    pub async fn execute_stop(&self, policy: StopPolicy) {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            inner.execute_stop(policy);
        }
    }

    pub async fn set_lease_enabled(&self, enabled: bool) {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            inner.lease_enabled = enabled;
        }
    }

    pub async fn set_temperatures(&self, temps: Vec<f32>) {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            inner.temperatures = temps;
        }
    }

    async fn run_prepared_motion(
        &self,
        stamp: AdmissionStamp,
        request: crate::rt_core::requests::motion::MotionRequest,
        epoch_id: Option<u64>,
    ) -> Result<u64, ArmError> {
        use crate::rt_core::requests::motion as req;
        const MAX_RETRIES: usize = 5;
        let request_at = Instant::now();
        let cached_cfg = self.get_fresh_legacy_motion_config().await;
        let (mut snapshot, cache_hit) = {
            let inner = self.inner.read().await;
            self.check_legacy_under_guard(&inner)?;
            req::preflight_motion_request_at_with_config(
                &inner,
                stamp,
                &request,
                epoch_id,
                request_at,
                cached_cfg.as_ref(),
            )?
        };
        drop(cached_cfg);
        if !cache_hit {
            self.hub.set_legacy_motion_config_if_newer(Arc::clone(&snapshot.config));
        }
        for attempt in 0..MAX_RETRIES {
            let (snap_plan, req_plan) = (snapshot.clone(), request.clone());
            let prepared = tokio::task::spawn_blocking(move || req::prepare_motion(&snap_plan, &req_plan))
                .await
                .map_err(|e| ArmError::Internal(format!("motion planner failed: {e}")))?;
            let mut inner = self.inner.write().await;
            inner.set_command(match request.kind {
                req::MotionKind::Jog => "/api/v1/motion/jog",
                req::MotionKind::Home => "/api/v1/motion/home",
                req::MotionKind::Park => "/api/v1/motion/unload",
            });
            self.check_legacy_under_guard(&inner)?;
            if let Some(hook) = inner.before_install_hook.take() { hook(&mut inner); }
            let plan = prepared?;
            let res = match request.kind {
                req::MotionKind::Jog => req::install_jog(&mut inner, stamp, &plan.slot, plan.heap_traj, epoch_id, false),
                req::MotionKind::Home => req::install_home(&mut inner, stamp, &plan.slot, plan.heap_traj, epoch_id, false),
                req::MotionKind::Park => req::install_park(&mut inner, stamp, &plan.slot, plan.heap_traj, epoch_id, false),
            };
            match res {
                Ok(installed) => {
                    let aid = match installed {
                        req::MotionInstalled::Started { action_id, .. }
                        | req::MotionInstalled::DeferredContinuation { action_id, .. } => action_id,
                    };
                    inner.set_action_id(aid);
                    return Ok(installed.epoch());
                }
                Err(req::MotionInstallError::StartDrifted) if attempt + 1 < MAX_RETRIES => {
                    if Instant::now() > snapshot.authority.deadline {
                        return Err(req::MotionInstallError::DeadlineExpired.into_arm_error(&inner, request.action_kind, epoch_id));
                    }
                    let Some(next) = req::MotionPlanSnapshot::refresh_for_drift_retry(&inner, &snapshot) else {
                        return Err(req::MotionInstallError::PlanStale.into_arm_error(&inner, request.action_kind, epoch_id));
                    };
                    snapshot = next;
                }
                Err(e) => return Err(e.into_arm_error(&inner, request.action_kind, epoch_id)),
            }
        }
        let inner = self.inner.read().await;
        Err(req::MotionInstallError::PlanStale.into_arm_error(&inner, request.action_kind, epoch_id))
    }

    pub async fn jog(
        &self,
        stamp: AdmissionStamp,
        target_positions: Vec<f32>,
        duration_secs: Option<f64>,
        epoch_id: Option<u64>,
    ) -> Result<u64, ArmError> {
        let req = crate::rt_core::requests::motion::MotionRequest::jog(target_positions, duration_secs, None);
        self.run_prepared_motion(stamp, req, epoch_id).await
    }

    pub async fn home(
        &self,
        stamp: AdmissionStamp,
        duration_secs: Option<f64>,
        epoch_id: Option<u64>,
    ) -> Result<u64, ArmError> {
        let req = crate::rt_core::requests::motion::MotionRequest::home(duration_secs);
        self.run_prepared_motion(stamp, req, epoch_id).await
    }

    pub async fn stop_motion(&self) {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            inner.stop_motion();
        }
    }

    pub async fn shutdown_ready(&self) -> bool {
        match self.owner_mode() {
            OWNER_EXECUTOR => match self.executor_handle() {
                Some(h) => h.shutdown_ready_async().await.unwrap_or(false),
                None => false,
            },
            OWNER_LEGACY => {
                let inner = self.inner.read().await;
                if self.check_legacy_under_guard(&inner).is_ok() {
                    return inner.shutdown_ready();
                }
                drop(inner);
                match self.executor_handle() {
                    Some(h) => h.shutdown_ready_async().await.unwrap_or(false),
                    None => false,
                }
            }
            _ => false,
        }
    }

    /// Authoritative shutdown gate routed through whichever single owner holds `ArmCore`.
    /// Bounded last-chance hardware disable for the legacy control-thread panic handler.
    /// Latches E-stop immediately and attempts one disable through the legacy driver within
    /// `timeout` without ever blocking indefinitely (even if another task holds `inner` or a
    /// blocking transport syscall stalls). `timeout` is the hard upper bound, with a small
    /// internal cooperative reserve so clean `Unconfirmed` exhaustion finishes logging, lock
    /// release, and channel handoff before `rx.recv_timeout` triggers `TimedOut`.
    /// Explicit test-only authorization hook for legacy `ProcessExit` protective disable tests.
    #[doc(hidden)]
    pub fn authorize_process_exit_for_test(&self) {
        self.process_exit_requested.store(true, Ordering::Release);
    }

    /// Bounded legacy `DisableProtective { ProcessExit }` helper. Requires explicit owner-bound
    /// `ProcessExit` authorization (`emergency_process_exit_disable_until`, `shutdown_committed`,
    /// or `authorize_process_exit_for_test`) so ad-hoc callers cannot issue a protective cut
    /// outside process exit (DRV-F1 B7). Never returns `NoDriver` from a detached shell.
    pub fn attempt_legacy_panic_disable(&self, timeout: Duration) -> PanicDisableOutcome {
        match self.attempt_legacy_panic_disable_step(timeout) {
            LegacyPanicDisableStep::Outcome(outcome) => outcome,
            LegacyPanicDisableStep::Detached => PanicDisableOutcome::Failed(
                "legacy supervisor shell is detached; active writer owns driver".into(),
            ),
        }
    }

    fn attempt_legacy_panic_disable_step(&self, timeout: Duration) -> LegacyPanicDisableStep {
        let exit_authorized = self.process_exit_requested.load(Ordering::Acquire)
            || self
                .inner
                .try_read()
                .is_ok_and(|g| !g.detached && g.shutdown_committed);
        if !exit_authorized {
            return LegacyPanicDisableStep::Outcome(PanicDisableOutcome::Failed(
                "legacy protective disable requires explicit ProcessExit authorization".into(),
            ));
        }
        if self.owner_mode.load(Ordering::Acquire) == OWNER_TRANSFERRING {
            return LegacyPanicDisableStep::Detached;
        }
        self.estop_state.request_and_latch();
        let owner_mode = Arc::clone(&self.owner_mode);
        #[cfg(not(target_arch = "wasm32"))]
        type PrePanicShadowSnap = Option<(crate::torque::TorqueIntent, bool, bool, Instant)>;
        #[cfg(target_arch = "wasm32")]
        type PrePanicShadowSnap = ();

        let run_inline = |inner: Arc<SupervisorLock<SupervisorInner>>,
                          owner_mode: Arc<AtomicU8>,
                          coop_deadline: Instant,
                          hard_deadline: Instant,
                          coop_budget: Duration|
         -> (LegacyPanicDisableStep, PrePanicShadowSnap) {
            std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                let mut guard = loop {
                    if owner_mode.load(Ordering::Acquire) == OWNER_TRANSFERRING {
                        return (LegacyPanicDisableStep::Detached, Default::default());
                    }
                    if let Ok(g) = inner.try_write() {
                        break g;
                    }
                    let rem = hard_deadline.saturating_duration_since(Instant::now());
                    if rem.is_zero() {
                        return (
                            LegacyPanicDisableStep::Outcome(PanicDisableOutcome::TimedOut),
                            Default::default(),
                        );
                    }
                    std::thread::sleep(rem.min(Duration::from_millis(2)));
                };
                if guard.detached || owner_mode.load(Ordering::Acquire) == OWNER_TRANSFERRING {
                    // Ownership moved or is actively transferring out of the Legacy shell;
                    // NEVER dispatch Legacy disable or falsely report `NoDriver`.
                    return (LegacyPanicDisableStep::Detached, Default::default());
                }
                if Instant::now() >= hard_deadline {
                    return (
                        LegacyPanicDisableStep::Outcome(PanicDisableOutcome::TimedOut),
                        Default::default(),
                    );
                }
                #[cfg(not(target_arch = "wasm32"))]
                let pre_snap = Some((
                    guard.shadow_live_intent(),
                    guard.is_torqued,
                    guard.safety.state().is_stopped(),
                    guard.cached_driver_clock_now(),
                ));
                #[cfg(target_arch = "wasm32")]
                let pre_snap = ();
                let outcome = guard.attempt_panic_disable_until(
                    coop_deadline,
                    hard_deadline,
                    coop_budget,
                );
                (LegacyPanicDisableStep::Outcome(outcome), pre_snap)
            }))
            .unwrap_or_else(|_| {
                (
                    LegacyPanicDisableStep::Outcome(PanicDisableOutcome::Failed(
                        "panic during legacy panic-disable".into(),
                    )),
                    Default::default(),
                )
            })
        };

        let origin = Instant::now();
        let (coop_deadline, hard_deadline) = driver_transitions::panic_disable_deadlines(origin, timeout);
        let coop_budget = coop_deadline.saturating_duration_since(origin);
        #[cfg(not(target_arch = "wasm32"))]
        {
            let inner = Arc::clone(&self.inner);
            let owner_mode_thread = Arc::clone(&owner_mode);
            let shadow_hub = Arc::clone(&self.torque_shadow);
            let (tx, rx) = std::sync::mpsc::sync_channel(1);
            match std::thread::Builder::new()
                .name("armd-panic-disable".into())
                .spawn(move || {
                    let (step, pre_snap, done_at) = {
                        let (res, pre_snap) = run_inline(
                            inner.clone(),
                            owner_mode_thread,
                            coop_deadline,
                            hard_deadline,
                            coop_budget,
                        );
                        (res, pre_snap, Instant::now())
                    };
                    // Seal and deliver the live outcome FIRST before touching shadow recording.
                    let _ = tx.send((step.clone(), done_at));
                    if let LegacyPanicDisableStep::Outcome(outcome) = step {
                        match inner.try_write() {
                            Ok(mut g) if !g.detached => {
                                if let Some((pre_intent, pre_torqued, pre_stopped, pre_ts)) = pre_snap {
                                    g.record_post_panic_disable_shadow_at(
                                        true,
                                        &outcome,
                                        pre_intent,
                                        pre_torqued,
                                        pre_stopped,
                                        pre_ts,
                                    );
                                } else {
                                    g.record_post_panic_disable_shadow(true, &outcome);
                                }
                            }
                            _ => {
                                // Lock contended or detached after fatal panic-disable: latch
                                // session staging-loss cutoff lock-free on the hub instead of
                                // silently dropping post-panic shadow state.
                                shadow_hub.record_contended_staging_loss(1);
                            }
                        }
                    }
                }) {
                Ok(_) => {
                    let wait = hard_deadline.saturating_duration_since(Instant::now());
                    match rx.recv_timeout(wait) {
                        Ok((step, completed_at)) if completed_at < hard_deadline => step,
                        Ok(_) | Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                            LegacyPanicDisableStep::Outcome(PanicDisableOutcome::TimedOut)
                        }
                        Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {
                            LegacyPanicDisableStep::Outcome(PanicDisableOutcome::Failed(
                                "panic-disable helper thread disconnected".into(),
                            ))
                        }
                    }
                }
                Err(e) => {
                    // Never fall back to an unbounded inline driver call when the bounded
                    // helper thread cannot be spawned; only confirm NoDriver when NOT detached.
                    if self.owner_mode.load(Ordering::Acquire) == OWNER_TRANSFERRING {
                        return LegacyPanicDisableStep::Detached;
                    }
                    if let Ok(guard) = self.inner.try_write() {
                        if guard.detached
                            || self.owner_mode.load(Ordering::Acquire) == OWNER_TRANSFERRING
                        {
                            return LegacyPanicDisableStep::Detached;
                        }
                        if guard.driver.is_none() {
                            return LegacyPanicDisableStep::Outcome(PanicDisableOutcome::NoDriver);
                        }
                    }
                    LegacyPanicDisableStep::Outcome(PanicDisableOutcome::Failed(format!(
                        "failed to spawn bounded panic-disable helper thread: {e}"
                    )))
                }
            }
        }
        #[cfg(target_arch = "wasm32")]
        {
            run_inline(
                Arc::clone(&self.inner),
                owner_mode,
                coop_deadline,
                hard_deadline,
                coop_budget,
            )
            .0
        }
    }

    pub async fn request_shutdown(&self) -> Result<bool, ArmError> {
        let deadline = Instant::now() + PANIC_DISABLE_TIMEOUT + Duration::from_millis(100);
        self.request_shutdown_until(deadline).await
    }

    /// Bounded shutdown admission carrying ONE absolute `deadline` from daemon entry (B3).
    pub async fn request_shutdown_until(&self, deadline: Instant) -> Result<bool, ArmError> {
        match self.owner_mode() {
            OWNER_EXECUTOR => match self.executor_handle() {
                Some(h) => h.request_shutdown_until(deadline).await.map_err(Into::into),
                None => Err(executor_unavailable()),
            },
            OWNER_LEGACY => {
                let rem = deadline.saturating_duration_since(Instant::now());
                if rem.is_zero() {
                    return Err(crate::rt_core::executor::OwnerError::TimedOut.into());
                }
                let mut inner = match tokio::time::timeout(rem, self.inner.clone().write_owned()).await {
                    Ok(g) => g,
                    Err(_) => return Err(crate::rt_core::executor::OwnerError::TimedOut.into()),
                };
                if self.check_legacy_under_guard(&inner).is_ok() {
                    let rem_after_lock = deadline.saturating_duration_since(Instant::now());
                    if rem_after_lock.is_zero() {
                        return Err(crate::rt_core::executor::OwnerError::TimedOut.into());
                    }
                    let task = tokio::task::spawn_blocking(move || inner.request_shutdown_until(deadline));
                    return match tokio::time::timeout(rem_after_lock, task).await {
                        Ok(Ok(ok)) => Ok(ok),
                        Ok(Err(e)) => Err(ArmError::Internal(format!(
                            "request_shutdown blocking task failed: {e}"
                        ))),
                        Err(_) => Err(crate::rt_core::executor::OwnerError::TimedOut.into()),
                    };
                }
                drop(inner);
                match self.executor_handle() {
                    Some(h) => h.request_shutdown_until(deadline).await.map_err(Into::into),
                    None => Err(executor_unavailable()),
                }
            }
            _ => {
                if self.has_detached_safe_hold() {
                    // A detached SAFE-HOLD writer holds an energized/unknown driver; ordinary
                    // shutdown admission is false until deliberate process exit disables it.
                    Ok(false)
                } else {
                    Err(executor_unavailable())
                }
            }
        }
    }

    /// Deliberate process-exit protective disable routed through the authoritative active writer
    /// endpoint (`Executor`, `Legacy`, or `DetachedSafeHold`) under ONE absolute `deadline` (B3/B4/B11).
    /// Waits across `WriterEndpoint::Transferring` and re-resolves if the Legacy shell was detached
    /// during extraction or an Executor handback completed concurrently.
    pub async fn emergency_process_exit_disable_until(
        self: &Arc<Self>,
        deadline: Instant,
    ) -> PanicDisableOutcome {
        self.process_exit_requested.store(true, Ordering::Release);
        self.estop_state.latch();
        self.internal_fault_lane.latch_control_safe_hold_fence();
        loop {
            let rem = deadline.saturating_duration_since(Instant::now());
            if rem.is_zero() {
                return PanicDisableOutcome::TimedOut;
            }
            match self.active_writer_endpoint() {
                WriterEndpoint::Transferring => {
                    // Ownership extraction/transfer is in progress; wait for the authoritative
                    // endpoint (`Executor`, `DetachedSafeHold`, or rolled-back `Legacy`) rather
                    // than selecting Legacy or touching the detached shell.
                    tokio::time::sleep(rem.min(Duration::from_millis(2))).await;
                    continue;
                }
                WriterEndpoint::DetachedSafeHold {
                    safe_hold,
                    control_stop,
                } => {
                    control_stop.store(true, Ordering::Release);
                    safe_hold.request_process_exit_disable(deadline);
                    return tokio::task::spawn_blocking(move || {
                        safe_hold.wait_process_exit_until(deadline)
                    })
                    .await
                    .unwrap_or_else(|je| PanicDisableOutcome::Failed(je.to_string()));
                }
                WriterEndpoint::Executor(handle) => {
                    let h_em = handle.clone();
                    let outcome = tokio::task::spawn_blocking(move || {
                        h_em.emergency_disable_and_join_until(deadline)
                    })
                    .await
                    .unwrap_or_else(|je| PanicDisableOutcome::Failed(je.to_string()));
                    // If the executor completed a concurrent handback to Legacy right as ProcessExit
                    // arrived, re-resolve the authoritative endpoint and disable via Legacy within
                    // the SAME `deadline` (DRV-F1 B3).
                    if handle.was_handed_back() {
                        continue;
                    }
                    return outcome;
                }
                WriterEndpoint::Legacy {
                    control_stop,
                    safe_hold,
                } => {
                    if let Some(stop) = control_stop {
                        self.signal_legacy_stop(&stop);
                    }
                    if safe_hold.is_active() {
                        safe_hold.request_process_exit_disable(deadline);
                        return tokio::task::spawn_blocking(move || {
                            safe_hold.wait_process_exit_until(deadline)
                        })
                        .await
                        .unwrap_or_else(|je| PanicDisableOutcome::Failed(je.to_string()));
                    }
                    let sup = Arc::clone(self);
                    let step = tokio::task::spawn_blocking(move || {
                        let rem = deadline.saturating_duration_since(Instant::now());
                        sup.attempt_legacy_panic_disable_step(rem)
                    })
                    .await
                    .map_err(|je| PanicDisableOutcome::Failed(je.to_string()));
                    match step {
                        Ok(LegacyPanicDisableStep::Outcome(outcome)) => return outcome,
                        Ok(LegacyPanicDisableStep::Detached) => {
                            // Shell was detached by a concurrent extraction into `Executor` or
                            // `DetachedSafeHold`: re-resolve the authoritative writer endpoint.
                            tokio::time::sleep(Duration::from_millis(1)).await;
                            continue;
                        }
                        Err(outcome) => return outcome,
                    }
                }
            }
        }
    }

    pub async fn is_at_safe_resting_pose(&self) -> bool {
        let inner = self.inner.read().await;
        inner.is_at_safe_resting_pose()
    }

    async fn await_estop_waiter(
        &self,
        mut rx: tokio::sync::oneshot::Receiver<Result<(SafetyState, u64), ArmError>>,
    ) -> Result<(SafetyState, u64), ArmError> {
        loop {
            match tokio::time::timeout(Duration::from_millis(2), &mut rx).await {
                Ok(Ok(res)) => return res,
                Ok(Err(_)) => {
                    return Err(ArmError::Rejected {
                        status: 503,
                        code: "EXECUTOR_STOPPED".into(),
                        message: "RT executor stopped before acknowledging E-stop".into(),
                    });
                }
                Err(_) => {
                    self.estop_reg.lock().unwrap_or_else(|e| e.into_inner()).drain();
                }
            }
        }
    }

    pub(crate) fn flush_legacy_estop_acks_into(
        estop_reg: &std::sync::Mutex<crate::rt_core::executor::EstopRegistry>,
        inner: &mut SupervisorInner,
    ) {
        if inner.pending_estop_acks.is_empty() && inner.estop_ack_overflow_gen == 0 {
            return;
        }
        let mut reg = estop_reg.lock().unwrap_or_else(|e| e.into_inner());
        while let Some(ack) = inner.pending_estop_acks.pop_front() {
            reg.record_ack(ack);
        }
        if inner.estop_ack_overflow_gen > 0 {
            let ov = std::mem::replace(&mut inner.estop_ack_overflow_gen, 0);
            reg.record_overflow(ov);
        }
    }

    /// Test-only hook: holds the ACK-drain `estop_reg` mutex so tests can prove the WS reader
    /// E-stop path latches and increments `requested_estop_count` purely via lock-free atomics.
    #[doc(hidden)]
    pub fn test_lock_estop_registry(&self) -> impl Drop + '_ {
        self.estop_reg.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Test-only accessor for the monotonic lock-free E-stop request counter.
    #[doc(hidden)]
    pub fn test_requested_estop_count(&self) -> u64 {
        self.requested_estop_count.load(Ordering::SeqCst)
    }

    /// Synchronously latches the physical E-stop flag and increments the requested generation
    /// using ONLY lock-free atomics (never takes `estop_reg` mutex or scans waiters on the
    /// reader path). Call [`Self::finish_e_stop_checked`] off-reader to register/resolve and
    /// obtain the applied `(SafetyState, epoch_id)`.
    pub(crate) fn begin_e_stop_checked(&self) -> CheckedEstopTicket {
        crate::rt_core::executor::assert_not_in_rt_release("ArmSupervisor::begin_e_stop_checked");
        let req = self.estop_state.request_and_latch();
        let mode = self.owner_mode.load(Ordering::Acquire);
        CheckedEstopTicket { req, mode }
    }

    pub(crate) async fn finish_e_stop_checked(
        &self,
        ticket: CheckedEstopTicket,
    ) -> Result<(SafetyState, u64), ArmError> {
        let CheckedEstopTicket { req, mode } = ticket;
        // Off-reader waiter registration / early-ACK resolution (never runs on WS reader).
        let wait = self
            .estop_reg
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .register_or_resolve(req);
        let mut rx = match wait {
            crate::rt_core::executor::EstopWait::Resolved(res) => return res,
            crate::rt_core::executor::EstopWait::Pending(rx) => rx,
        };
        match mode {
            OWNER_EXECUTOR => {
                self.estop_reg.lock().unwrap_or_else(|e| e.into_inner()).drain();
                self.await_estop_waiter(rx).await
            }
            OWNER_LEGACY | OWNER_TRANSFERRING => {
                if let Ok(res) = rx.try_recv() {
                    return res;
                }
                let inner = Arc::clone(&self.inner);
                let mut guard = inner.write().await;
                if let Ok(res) = rx.try_recv() {
                    drop(guard);
                    return res;
                }
                if !guard.detached {
                    guard.consume_pending_estop();
                    Self::flush_legacy_estop_acks_into(&self.estop_reg, &mut guard);
                    let fallback = (guard.safety.state().clone(), guard.safety.epoch_id());
                    drop(guard);
                    return rx.try_recv().unwrap_or(Ok(fallback));
                }
                drop(guard);
                // Ownership changed while waiting on `inner.write()`: if a concurrent legacy
                // call or the newly attached executor already applied `req`, return that pinned result.
                if let Ok(res) = rx.try_recv() {
                    return res;
                }
                match self.owner_mode() {
                    OWNER_EXECUTOR => {
                        self.estop_reg.lock().unwrap_or_else(|e| e.into_inner()).drain();
                        self.await_estop_waiter(rx).await
                    }
                    _ => {
                        self.estop_reg.lock().unwrap_or_else(|e| e.into_inner()).cancel_waiter(
                            req,
                            503,
                            "EXECUTOR_UNAVAILABLE",
                            "RT executor ownership unavailable while applying E-stop",
                        );
                        if let Ok(res) = rx.try_recv() {
                            return res;
                        }
                        Err(executor_unavailable())
                    }
                }
            }
            _ => {
                self.estop_reg.lock().unwrap_or_else(|e| e.into_inner()).cancel_waiter(
                    req,
                    503,
                    "EXECUTOR_UNAVAILABLE",
                    "RT executor ownership is unavailable",
                );
                Err(executor_unavailable())
            }
        }
    }

    /// Lock-free physical stop latch + exact per-request pinned acknowledgement across both
    /// Legacy and Executor ownership (including concurrent Legacy -> Executor handoff).
    pub async fn trigger_e_stop_checked(&self) -> Result<(SafetyState, u64), ArmError> {
        crate::rt_core::executor::assert_not_in_rt_release("ArmSupervisor::trigger_e_stop_checked");
        let ticket = self.begin_e_stop_checked();
        self.finish_e_stop_checked(ticket).await
    }

    pub async fn trigger_e_stop(&self) -> (SafetyState, u64) {
        crate::rt_core::executor::assert_not_in_rt_release("ArmSupervisor::trigger_e_stop");
        self.trigger_e_stop_checked()
            .await
            .unwrap_or((SafetyState::EStop, 0))
    }

    /// Non-blocking urgent-stop enqueue in `OWNER_EXECUTOR` mode so a WS receive loop can
    /// await the reply in `tokio::select!` without blocking subsequent `estop` frames on
    /// the same socket. Returns `Ok(None)` in `OWNER_LEGACY`.
    pub(crate) fn try_enqueue_executor_urgent_stop(
        &self,
        stop: crate::rt_core::urgent::UrgentStop,
    ) -> Result<
        Option<(
            Arc<crate::rt_core::urgent::UrgentStopClient>,
            tokio::sync::oneshot::Receiver<
                Result<crate::rt_core::urgent::UrgentStopOutcome, crate::rt_core::urgent::UrgentStopError>,
            >,
        )>,
        ArmError,
    > {
        crate::rt_core::executor::assert_not_in_rt_release("ArmSupervisor::try_enqueue_executor_urgent_stop");
        match self.owner_mode() {
            OWNER_LEGACY => Ok(None),
            OWNER_EXECUTOR => {
                let h = self.executor_handle().ok_or_else(executor_unavailable)?;
                let client = h.urgent_stop_client();
                let rx = client.enqueue_async(stop).map_err(ArmError::from)?;
                Ok(Some((client, rx)))
            }
            _ => Err(executor_unavailable()),
        }
    }

    /// Authorized-stop handoff routed through whichever single owner holds `ArmCore`
    /// (including concurrent Legacy -> Executor handoff).
    /// - In `OWNER_EXECUTOR`: forwards through the bounded urgent `Stop` channel without
    ///   touching the legacy supervisor lock; only an authorized stop advances `StopRequests`.
    /// - In `OWNER_LEGACY`: validates under `inner.write()` and preserves exact legacy
    ///   `/api/v1/motion/stop` semantics (gravity-comp exit, `stop_motion`, `request_macro_stop`).
    pub async fn urgent_stop(
        &self,
        stop: crate::rt_core::urgent::UrgentStop,
    ) -> Result<crate::rt_core::urgent::UrgentStopOutcome, ArmError> {
        self.urgent_stop_with_legacy_hook(stop, |_, _| {}).await
    }

    /// Like [`Self::urgent_stop`], running `on_legacy_applied(epoch, stop_gen)` strictly inside
    /// the legacy `inner.write()` critical section alongside the stop mutation.
    pub(crate) async fn urgent_stop_with_legacy_hook(
        &self,
        stop: crate::rt_core::urgent::UrgentStop,
        on_legacy_applied: impl FnOnce(u64, u64),
    ) -> Result<crate::rt_core::urgent::UrgentStopOutcome, ArmError> {
        use crate::rt_core::urgent::UrgentStopOutcome;
        crate::rt_core::executor::assert_not_in_rt_release("ArmSupervisor::urgent_stop");
        match self.owner_mode() {
            OWNER_EXECUTOR => {
                let h = self.executor_handle().ok_or_else(executor_unavailable)?;
                h.urgent_stop_async(stop).await.map_err(ArmError::from)
            }
            OWNER_LEGACY => {
                let mut guard = self.inner.write().await;
                if !guard.detached {
                    guard.consume_pending_estop();
                    Self::flush_legacy_estop_acks_into(&self.estop_reg, &mut guard);
                    let epoch = guard.safety.epoch_id();
                    if stop.lease_generation == 0
                        || !guard.validate_lease_at(stop.connection.as_str(), stop.lease_generation, Instant::now())
                    {
                        return Ok(UrgentStopOutcome::LeaseLost { epoch });
                    }
                    if stop.epoch != Some(epoch) {
                        return Ok(UrgentStopOutcome::EpochMismatch { epoch });
                    }
                    if !stop.request_id_valid {
                        return Ok(UrgentStopOutcome::InvalidRequestId);
                    }
                    if let Some(message) = stop.body_error {
                        return Ok(UrgentStopOutcome::InvalidBody { message, epoch });
                    }
                    let applied_to_motion = guard.authorized_stop();
                    let stop_gen = if applied_to_motion {
                        guard.current_stop_gen()
                    } else {
                        guard.advance_stop_generation()
                    };
                    let applied_epoch = guard.safety.epoch_id();
                    on_legacy_applied(applied_epoch, stop_gen);
                    return Ok(UrgentStopOutcome::Applied {
                        stop_generation: stop_gen,
                        applied_to_motion,
                        epoch: applied_epoch,
                        safety_state: guard.safety.state_arc(),
                        is_torqued: guard.is_torqued,
                        control_mode_gravity_comp: guard.gravity_comp_active(),
                    });
                }
                drop(guard);
                if self.owner_mode() == OWNER_EXECUTOR {
                    let h = self.executor_handle().ok_or_else(executor_unavailable)?;
                    h.urgent_stop_async(stop).await.map_err(ArmError::from)
                } else {
                    Err(executor_unavailable())
                }
            }
            _ => Err(executor_unavailable()),
        }
    }

    pub async fn clear_fault(&self) -> Result<SafetyState, ArmError> {
        self.clear_fault_for_epoch(None).await
    }

    pub async fn clear_fault_for_epoch(&self, expected_epoch: Option<u64>) -> Result<SafetyState, ArmError> {
        match self.owner_mode() {
            OWNER_EXECUTOR => {
                let h = self.executor_handle().ok_or_else(executor_unavailable)?;
                let res = h.clear_fault_for_epoch_async(expected_epoch).await.map_err(ArmError::from)?;
                res.map_err(ArmError::Safety)
            }
            OWNER_LEGACY => {
                let mut inner = self.inner.clone().write_owned().await;
                if self.check_legacy_under_guard(&inner).is_err() {
                    drop(inner);
                    if self.owner_mode() == OWNER_EXECUTOR {
                        let h = self.executor_handle().ok_or_else(executor_unavailable)?;
                        let res = h.clear_fault_for_epoch_async(expected_epoch).await.map_err(ArmError::from)?;
                        return res.map_err(ArmError::Safety);
                    }
                    return Err(executor_unavailable());
                }
                let estop_reg = Arc::clone(&self.estop_reg);
                let res = tokio::task::spawn_blocking(move || {
                    let r = inner.clear_fault_for_epoch(expected_epoch);
                    inner.safety.drain_retired_state_arcs_off_thread();
                    ArmSupervisor::flush_legacy_estop_acks_into(&estop_reg, &mut inner);
                    r
                })
                    .await
                    .map_err(|e| ArmError::Internal(format!("clear_fault blocking task failed: {e}")))?;
                res.map_err(ArmError::Safety)
            }
            _ => Err(executor_unavailable()),
        }
    }

    pub async fn apply_stop_policy(&self, policy: StopPolicy) {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            inner.apply_stop_policy(policy);
        }
    }

    pub async fn is_unloading(&self) -> bool {
        let inner = self.inner.read().await;
        inner.pending_normal_unload || inner.active_trajectory.is_some() || inner.unload_settle.is_some()
    }

    pub async fn has_active_trajectory(&self) -> bool {
        let inner = self.inner.read().await;
        inner.active_trajectory.is_some()
    }

    /// Exposes the lock-free tick ring for deterministic paused-reader contention tests.
    #[doc(hidden)]
    pub fn tick_stats_ring_for_test(&self) -> &Arc<crate::diagnostics::tick::LockFreeTickStats> {
        &self.tick_stats
    }

    /// The raw samples of the last ticks, oldest first (tests, diagnostics).
    pub fn tick_samples(&self) -> Vec<crate::diagnostics::tick::TickSample> {
        self.drain_diag();
        self.tick_stats.window().samples().to_vec()
    }

    /// Requests a coherent control-thread baseline marker (`POST /api/v1/diagnostics/tick/mark`)
    /// on the currently active periodic owner. Returns 503 if no control loop is actively running.
    pub fn mark_tick_baseline(&self) -> Result<u64, ArmError> {
        match self.owner_mode() {
            OWNER_EXECUTOR => {
                let Some(h) = self.executor_handle() else {
                    return Err(executor_unavailable());
                };
                if h.is_exited() || !h.stats().is_loop_active() {
                    return Err(ArmError::Rejected {
                        status: 503,
                        code: "EXECUTOR_STOPPED".into(),
                        message: "RT executor control loop is not active".into(),
                    });
                }
                Ok(h.stats().request_marker())
            }
            OWNER_LEGACY => {
                let stats = self
                    .legacy_loop_stats
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .clone();
                let Some(s) = stats else {
                    return Err(ArmError::Rejected {
                        status: 503,
                        code: "CONTROL_LOOP_INACTIVE".into(),
                        message: "Legacy periodic control loop is not active".into(),
                    });
                };
                if !s.is_loop_active() {
                    return Err(ArmError::Rejected {
                        status: 503,
                        code: "CONTROL_LOOP_INACTIVE".into(),
                        message: "Legacy periodic control loop is not active".into(),
                    });
                }
                Ok(s.request_marker())
            }
            _ => Err(executor_unavailable()),
        }
    }

    /// Timing of the last ticks (`GET /api/v1/diagnostics/tick`). Reads the lock-free ring
    /// without ever blocking the control thread.
    pub fn tick_stats(&self) -> crate::diagnostics::tick::TickSnapshot {
        const MAX_ALIGN_ATTEMPTS: usize = 4;
        let executor_stats = self.executor_handle().map(|h| Arc::clone(h.stats()));
        let is_executor = executor_stats.is_some();
        let active_stats = if let Some(s) = executor_stats {
            Some(s)
        } else {
            self.legacy_loop_stats
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .clone()
        };

        let mut snap = if let Some(ref s) = active_stats {
            // Bounded alignment loop: read monotonic snapshot FIRST (Acquire on `stats_seq`),
            // drain off-thread diagnostics (guaranteed to include all samples & amendments
            // published before `mono1`), copy the lock-free window (fast memcpy, no sort),
            // and re-read monotonic snapshot to confirm no release crossed the capture window.
            let mut chosen: Option<(
                crate::diagnostics::tick::TickWindow,
                crate::rt_core::executor::MonotonicStatsSnapshot,
            )> = None;
            let mut last_attempt = None;
            for _ in 0..MAX_ALIGN_ATTEMPTS {
                let mono1 = s.snapshot_monotonic();
                self.drain_diag();
                let win = self.tick_stats.window();
                let mono2 = s.snapshot_monotonic();
                let aligned = mono1.coherent
                    && mono2.coherent
                    && !win.is_lapped()
                    && mono1.releases_started == mono2.releases_started
                    && mono1.releases_total == mono2.releases_total
                    && mono1.skipped_releases == mono2.skipped_releases
                    && mono1.loop_active == mono2.loop_active;
                if aligned {
                    chosen = Some((win, mono2));
                    break;
                }
                last_attempt = Some((win, mono2));
            }
            let (win, mono, align_lapped) = match chosen {
                Some((win, mono)) => (win, mono, false),
                None => {
                    let (win, mono) = last_attempt.expect("MAX_ALIGN_ATTEMPTS > 0");
                    (win, mono, true)
                }
            };
            let mut snap = win.snapshot();
            snap.stats_reader_retries = self.tick_stats.reader_retries();
            snap.stats_writer_drops = self
                .tick_stats
                .writer_contention_drops()
                .saturating_add(s.amend_missed.load(Ordering::Relaxed));
            if !mono.coherent || align_lapped {
                snap.stats_snapshot_lapped = true;
            }
            snap.loop_active = mono.loop_active;
            snap.releases_started = mono.releases_started;
            snap.releases_total = mono.releases_total;
            snap.in_flight_release_us = mono.in_flight_release_us;
            snap.release_us_max_ever = mono.release_us_max_ever;
            snap.releases_over_10ms = mono.releases_over_10ms;
            snap.releases_over_15ms = mono.releases_over_15ms;
            snap.releases_over_20ms = mono.releases_over_20ms;
            snap.wake_late_max_ever = mono.wake_late_max_ever;
            snap.skipped_releases = mono.skipped_releases;
            snap.diag_dropped = mono.diag_dropped;
            snap.wake_lateness_over_1ms = mono.wake_lateness_over_1ms;
            snap.wake_lateness_over_5ms = mono.wake_lateness_over_5ms;
            snap.wake_lateness_over_10ms = mono.wake_lateness_over_10ms;
            snap.since_marker = mono.since_marker;
            snap
        } else {
            self.drain_diag();
            self.tick_stats.snapshot()
        };
        snap.instance_id = self.instance_id.clone();
        let ident = init_process_identity();
        snap.boot_id = ident.boot_id.clone();
        snap.process_start_unix_ns = ident.process_start_unix_ns;
        snap.owner_mode = self.owner_mode_name();
        snap.effective_owner = self.effective_owner().as_str();
        snap.owner_reason = self.owner_reason().as_str();
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
        snap.terminal_capture = self.terminal_capture_channel.stats();
        snap.safety_log_dropped = self
            .sinks
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .map_or_else(
                || self.inner.try_read().map_or(0, |g| g.safety.log_dropped().saturating_add(g.safety_log_ring_dropped)),
                |s| s.safety_log_dropped(),
            );
        snap.log_writer_dropped = crate::diagnostics::logwriter::total_dropped();
        if is_executor {
            snap.control_thread_lock_attempts = Some(
                self.inner
                    .control_thread_attempts()
                    .saturating_add(self.driver_lock_attempts.load(Ordering::Acquire))
                    .saturating_add(self.terminal_capture_channel.control_thread_lock_attempts()),
            );
        }
        if let Some(ref s) = active_stats {
            if s.alloc_timing_enabled.load(Ordering::Acquire) {
                snap.alloc_timing_enabled = Some(true);
                if is_executor {
                    snap.control_thread_alloc_ops = Some(s.total_allocs().total());
                }
            }
            if is_executor {
                snap.executor_wakes = Some(s.completed_wakes.load(Ordering::Acquire));
            }
        }
        snap
    }

    /// Thread-CPU-time percentiles of lock-held tick execution over the current tick window
    /// (`CLOCK_THREAD_CPUTIME_ID` on Linux, excluding off-CPU OS preemption under host load).
    #[doc(hidden)]
    pub fn tick_lock_cpu_us(&self) -> crate::diagnostics::tick::Percentiles {
        self.drain_diag();
        self.tick_stats.window().lock_cpu_us()
    }

    pub async fn set_mock_gripper_stall_offset(&self, offset: Option<f32>) {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            inner.mock_gripper_stall_offset = offset;
        }
    }

    pub async fn gripper_calibration_response(&self) -> crate::gripper_calib::GripperCalibrationResponse {
        self.inner.read().await.gripper_calibration_response()
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
        sup.save_recording_with(move |path, frames| {
            tx.send(frames.to_vec()).unwrap();
            crate::recording::write_episode_parquet(path, frames, None, &[])
        })
        .await
        .unwrap();
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

        // An already-expired lease record does not collapse `valid_until` to `sampled` forever.
        {
            let inner = sup.inner().read().await;
            let mut obs = inner.current_observation();
            assert_eq!(obs.lease_holder, None);
            obs.feedback_age_ms.clear();
            let cap = Duration::from_secs(1);
            let (at, until) = inner.observation_valid_until(&obs, cap);
            assert_eq!(until - at, cap);
        }

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

    #[tokio::test]
    async fn legacy_urgent_stop_advances_policy_gen_inside_write_lock_only_on_apply() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let conn = "connection:policy-race";
        let (gen, ep) = {
            let mut inner = sup.inner().write().await;
            let g = inner.acquire_lease(conn).unwrap();
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).unwrap();
            SupervisorInner::enable_torque(admitted).unwrap();
            (g, inner.safety.epoch_id())
        };
        let policy_stop_gen = Arc::new(AtomicU64::new(0));
        let mk = |lg, e| crate::rt_core::urgent::UrgentStop {
            connection: crate::model::HolderLabel::new(conn),
            lease_generation: lg,
            epoch: Some(e),
            request_id: 1,
            request_id_valid: true,
            body_error: None,
        };
        // Refused stop (wrong epoch / wrong lease) must NOT advance stop_gen or invoke the hook.
        let r_refused = sup.urgent_stop_with_legacy_hook(mk(gen, ep + 1), |_, _| { policy_stop_gen.fetch_add(1, Ordering::SeqCst); }).await.unwrap();
        assert!(matches!(r_refused, crate::rt_core::urgent::UrgentStopOutcome::EpochMismatch { .. }));
        assert_eq!(policy_stop_gen.load(Ordering::SeqCst), 0);
        assert_eq!(sup.stop_generation(), 0);

        // Authorized stop must advance stop_gen BEFORE releasing `inner.write()`, so a
        // concurrent motion-start installer queued on `inner.write()` observes the bump immediately.
        let entered = Arc::new(tokio::sync::Notify::new());
        let release = Arc::new(tokio::sync::Notify::new());
        let (s1, p1, e1, r1) = (sup.clone(), policy_stop_gen.clone(), entered.clone(), release.clone());
        let stop_task = tokio::spawn(async move {
            s1.urgent_stop_with_legacy_hook(mk(gen, ep), |_, stop_gen| {
                p1.store(stop_gen, Ordering::SeqCst);
                e1.notify_one();
                // Yield briefly while still holding `inner.write()` so the installer task queues on the lock.
                std::thread::sleep(Duration::from_millis(15));
                r1.notify_one();
            }).await.unwrap()
        });
        entered.notified().await;
        // Now acquire `inner.write()` just like policy-load completion does: as soon as the lock
        // is granted, `stop_gen` MUST already be 1 (zero race window after Stop applies).
        let guard = sup.inner().write().await;
        assert_eq!(policy_stop_gen.load(Ordering::SeqCst), 1);
        assert_eq!(guard.current_stop_gen(), 1);
        drop(guard);
        let outcome = stop_task.await.unwrap();
        assert!(matches!(outcome, crate::rt_core::urgent::UrgentStopOutcome::Applied { stop_generation: 1, .. }));
    }

    #[test]
    fn terminal_broadcaster_never_skips_or_pops_on_full_fifo_or_failed_enqueue() {
        let tb = TerminalBroadcaster::default();
        tb.set_capacity(2);
        let accept = Arc::new(AtomicBool::new(false));
        let delivered = Arc::new(std::sync::Mutex::new(Vec::<u64>::new()));
        let accept_c = Arc::clone(&accept);
        let delivered_c = Arc::clone(&delivered);
        let sub_id = tb.subscribe(Box::new(move |obs| {
            if !accept_c.load(Ordering::SeqCst) {
                return false;
            }
            if let Some(a) = obs.action.as_ref() {
                delivered_c.lock().unwrap().push(a.id);
            }
            true
        }));
        let mut factory = SupervisorInner::new(ArmConfig::rebot_dm());
        let mut mk_done = || {
            let id = factory.begin_action("jog", None);
            factory.finish_action(id, "done", None, None, None, None, None);
            factory.last_action.clone().unwrap()
        };
        let a1 = mk_done();
        let a2 = mk_done();
        let a3 = mk_done();
        assert!(tb.can_start_action(false, false));
        assert!(tb.record_finished(&a1));
        assert!(tb.record_finished(&a2));
        // FIFO is now at capacity (2/2): NEW starts must be refused (`TERMINALS_BUSY`).
        assert!(!tb.can_start_action(false, false));
        assert!(!tb.can_start_action(false, true));
        // An already-running action finishing while FIFO is full MUST NOT be skipped/dropped.
        assert!(tb.record_finished(&a3));
        assert_eq!(tb.pending_total(), 3);
        assert!(!tb.can_start_action(false, false));

        // Sink rejects (outbound queue full): drain_all MUST NOT pop action 1 and MUST NOT remove
        // the subscriber.
        let base_obs = factory.current_observation();
        let _ = tb.drain_all(&base_obs, 0);
        assert_eq!(tb.pending_total(), 3, "failed enqueue must not pop terminal or drop subscriber");
        assert!(delivered.lock().unwrap().is_empty());

        // Once sink accepts again, all 3 terminals pop and deliver in exact FIFO order.
        accept.store(true, Ordering::SeqCst);
        let _ = tb.drain_all(&base_obs, 0);
        assert_eq!(tb.pending_total(), 0);
        assert_eq!(*delivered.lock().unwrap(), vec![a1.id, a2.id, a3.id]);
        assert!(tb.can_start_action(false, false));
        tb.unsubscribe(sub_id);
    }

    #[test]
    fn parse_linux_process_start_unix_ns_handles_valid_weird_comm_and_errors() {
        // 19 fields after `)` before field 22 (`starttime`), e.g. state='S', ppid..=num_threads(18) -> 19 tokens, index 19 is starttime=250.
        let proc_stat = "cpu 1 2 3\nbtime 1700000000\nprocesses 42\n";
        let self_stat = "1234 (armd (rt) v1.6) S 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 250 999 1000";
        // 1700000000 s + 250 / 100 s = 1700000002.5 s = 1700000002_500_000_000 ns
        assert_eq!(
            parse_linux_process_start_unix_ns(proc_stat, self_stat, 100),
            Some(1_700_000_002_500_000_000)
        );
        // Invalid clk_tck <= 0 -> None
        assert_eq!(parse_linux_process_start_unix_ns(proc_stat, self_stat, 0), None);
        assert_eq!(parse_linux_process_start_unix_ns(proc_stat, self_stat, -1), None);
        // Missing btime -> None
        assert_eq!(parse_linux_process_start_unix_ns("cpu 1 2 3\n", self_stat, 100), None);
        // Malformed / truncated /proc/self/stat -> None
        assert_eq!(parse_linux_process_start_unix_ns(proc_stat, "1234 (armd) S 1 2", 100), None);
        // Overflow -> None
        let overflow_stat = format!("btime {}\n", u64::MAX);
        assert_eq!(parse_linux_process_start_unix_ns(&overflow_stat, self_stat, 100), None);

        #[cfg(target_os = "linux")]
        {
            let id1 = init_process_identity();
            let id2 = init_process_identity();
            assert_eq!(id1, id2);
            assert!(id1.boot_id.as_deref().is_some_and(|s| !s.is_empty()));
            assert!(id1.process_start_unix_ns.is_some_and(|ns| ns > 0));
        }
    }

    #[test]
    fn drv_lock_release_helpers_positive_control_legacy_panics_exclusive_succeeds() {
        use crate::driver::{
            ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget,
            PosForceEmitOutcome, PosForceIntent, PosForceIntentKind, PosForceIntentStamp,
        };
        use crate::model::JointState;
        use crate::rt_core::executor::{ControlThreadGuard, ReleaseGuard};
        use crate::rt_core::ops::ServiceBudget;
        use crate::supervisor_lock::DriverSlot;
        use std::sync::atomic::AtomicUsize;

        struct ObservableReleaseDriver {
            dof: usize,
            rx_pos: f32,
            writes: Arc<AtomicUsize>,
            last_j0_cmd: Arc<std::sync::atomic::AtomicU32>,
            j7_epilogue_calls: Arc<AtomicUsize>,
        }

        impl ArmDriver for ObservableReleaseDriver {
            fn capabilities(&self) -> DriverCapabilities {
                DriverCapabilities {
                    enable_feedback: true,
                    velocity_feedback: true,
                    torque_feedback: true,
                    temperature_feedback: true,
                    ..DriverCapabilities::default()
                }
            }
            fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
                let mut v = vec![JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 30.0 }; self.dof];
                v[0].position = self.rx_pos;
                Ok(v)
            }
            fn read_joint_states_budgeted(
                &mut self,
                out: &mut Vec<JointState>,
                _deadline: Instant,
                _max_rx_frames: u16,
            ) -> Result<(u16, bool), ArmDriverError> {
                out.clear();
                out.resize(self.dof, JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 30.0 });
                out[0].position = self.rx_pos;
                Ok((self.dof as u16, false))
            }
            fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
                self.writes.fetch_add(1, Ordering::SeqCst);
                if let Some(t0) = targets.first() {
                    self.last_j0_cmd.store(t0.position.to_bits(), Ordering::SeqCst);
                }
                Ok(())
            }
            fn set_torque(&mut self, _enable: bool) -> Result<(), ArmDriverError> {
                Ok(())
            }
            fn supports_rt_executor(&self) -> Result<(), &'static str> {
                Ok(())
            }
            fn is_feedback_valid(&self) -> bool { true }
            fn is_enabled(&self) -> bool { true }
            fn enable_state(&self) -> EnableState { EnableState::Enabled }
            fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::from_millis(1)); self.dof] }
            fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
                out.clear();
                out.resize(self.dof, Some(Duration::from_millis(1)));
            }
            fn feedback_instants(&self) -> Vec<Option<Instant>> { vec![Some(Instant::now()); self.dof] }
            fn feedback_seqs(&self) -> Vec<u64> { vec![1; self.dof] }
            crate::ext_accessors!(pos_force);
        }
        impl crate::driver::PosForceExt for ObservableReleaseDriver {
            fn emit_pos_force_j7(
                &mut self,
                _intent: PosForceIntent,
                _current_stamp: PosForceIntentStamp,
            ) -> Result<PosForceEmitOutcome, ArmDriverError> {
                self.j7_epilogue_calls.fetch_add(1, Ordering::SeqCst);
                Ok(PosForceEmitOutcome::NotApplicable)
            }
        }

        let make_inner = |exclusive: bool| {
            let writes = Arc::new(AtomicUsize::new(0));
            let last_j0_cmd = Arc::new(std::sync::atomic::AtomicU32::new(0));
            let j7_epilogue_calls = Arc::new(AtomicUsize::new(0));
            let drv = ObservableReleaseDriver {
                dof: 7,
                rx_pos: 0.42,
                writes: Arc::clone(&writes),
                last_j0_cmd: Arc::clone(&last_j0_cmd),
                j7_epilogue_calls: Arc::clone(&j7_epilogue_calls),
            };
            let counter = Arc::new(AtomicU64::new(0));
            let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
            inner.lease_enabled = false;
            inner.is_torqued = true;
            inner.rt_executor_owned = true;
            inner.safety.set_torque(true).unwrap();
            inner.driver_lock_attempts = Arc::clone(&counter);
            inner.driver = Some(if exclusive {
                DriverSlot::exclusive(Box::new(drv))
            } else {
                DriverSlot::legacy_with_counter(Box::new(drv), Arc::clone(&counter))
            });
            (inner, counter, writes, last_j0_cmd, j7_epilogue_calls)
        };

        // 1. Positive control: Legacy DriverSlot panics + increments `driver_lock_attempts` on
        //    EVERY one of the 4 release-path helpers under ControlThreadGuard + ReleaseGuard.
        {
            let (mut inner, counter, writes, _last_j0, j7_calls) = make_inner(false);
            let mut obs = inner.current_observation();
            let _cg = ControlThreadGuard::enter();
            let _rg = ReleaseGuard::enter();
            let now = Instant::now();
            let budget = ServiceBudget::for_release(now, now + Duration::from_millis(5));

            assert!(std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                let _ = inner.poll_hardware_rx(Some(&budget), None, Some(now + Duration::from_millis(5)));
            })).is_err());
            assert_eq!(counter.load(Ordering::Acquire), 1, "poll_hardware_rx must count +1");

            let mut targets = vec![JointTarget::position(0.15); 7];
            assert!(std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                let _ = inner.write_output_slice(&mut targets, 0.02, None);
            })).is_err());
            assert_eq!(counter.load(Ordering::Acquire), 2, "write_output_slice must count +1");
            assert_eq!(writes.load(Ordering::SeqCst), 0);

            let stamp = PosForceIntentStamp { stop_gen: 1, estop_epoch: 0, lease_gen: 0 };
            let mut tick_out = TickOutput::None;
            let mut allow_acc = true;
            assert!(std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                inner.emit_j7_tick_epilogue(
                    false,
                    PosForceIntent { kind: PosForceIntentKind::Hold, stamp },
                    stamp,
                    &mut tick_out,
                    &mut allow_acc,
                );
            })).is_err());
            assert_eq!(counter.load(Ordering::Acquire), 3, "emit_j7_tick_epilogue must count +1");
            assert_eq!(j7_calls.load(Ordering::SeqCst), 0);

            assert!(std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                inner.fill_observation(1.0, &mut obs);
            })).is_err());
            assert_eq!(counter.load(Ordering::Acquire), 4, "fill_observation must count +1");
        }

        // 2. Exclusive DriverSlot succeeds across ALL 4 release-path helpers under
        //    ControlThreadGuard + ReleaseGuard with 0 attempts AND observable side effects.
        {
            let (mut inner, counter, writes, last_j0, j7_calls) = make_inner(true);
            let _cg = ControlThreadGuard::enter();
            let _rg = ReleaseGuard::enter();
            let now = Instant::now();
            let budget = ServiceBudget::for_release(now, now + Duration::from_millis(5));

            let rx = inner.poll_hardware_rx(Some(&budget), None, Some(now + Duration::from_millis(5)));
            assert!(rx.active);
            assert!((inner.measured_positions[0] - 0.42).abs() < 1e-5, "poll_hardware_rx must update measured_positions");

            let mut targets = vec![JointTarget::position(0.15); 7];
            let out = inner.write_output_slice(&mut targets, 0.02, None);
            assert!(matches!(out, TickOutput::Complete { .. }), "write_output_slice must complete: {out:?}");
            assert_eq!(writes.load(Ordering::SeqCst), 1);
            assert!((f32::from_bits(last_j0.load(Ordering::SeqCst)) - 0.15).abs() < 1e-5);

            let stamp = PosForceIntentStamp { stop_gen: 1, estop_epoch: 0, lease_gen: 0 };
            let mut tick_out = TickOutput::None;
            let mut allow_acc = true;
            inner.emit_j7_tick_epilogue(
                false,
                PosForceIntent { kind: PosForceIntentKind::Hold, stamp },
                stamp,
                &mut tick_out,
                &mut allow_acc,
            );
            assert_eq!(j7_calls.load(Ordering::SeqCst), 1, "emit_j7_tick_epilogue must invoke driver");

            let mut obs = inner.current_observation();
            inner.fill_observation(1.0, &mut obs);
            assert!(obs.driver_capabilities.enable_feedback);
            assert!((obs.positions[0] - 0.42).abs() < 1e-5);
            assert_eq!(counter.load(Ordering::Acquire), 0, "Exclusive slot must record 0 lock attempts");
        }
    }

    #[test]
    fn drv_lock_major6_tx_cutoff_samples_owner_clock_at_driver_boundary() {
        use crate::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
        use crate::model::JointState;
        use crate::supervisor_lock::DriverSlot;
        use crate::transport::MockClock;

        struct DeadlineProbeDriver {
            dof: usize,
            clock: MockClock,
            installed_deadlines: Arc<std::sync::Mutex<Vec<Option<Instant>>>>,
        }
        impl ArmDriver for DeadlineProbeDriver {
            fn capabilities(&self) -> DriverCapabilities { DriverCapabilities::default() }
            fn clock_now(&self) -> Instant { self.clock.now() }
            fn set_tx_release_deadline(&mut self, deadline: Option<Instant>) {
                self.installed_deadlines.lock().unwrap().push(deadline);
            }
            fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
                Ok(vec![JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 30.0 }; self.dof])
            }
            fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
            fn set_torque(&mut self, _enable: bool) -> Result<(), ArmDriverError> { Ok(()) }
            fn is_feedback_valid(&self) -> bool { true }
            fn is_enabled(&self) -> bool { true }
            fn enable_state(&self) -> EnableState { EnableState::Enabled }
            fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::from_millis(1)); self.dof] }
            fn feedback_instants(&self) -> Vec<Option<Instant>> { vec![Some(self.clock.now()); self.dof] }
            fn feedback_seqs(&self) -> Vec<u64> { vec![1; self.dof] }
        }

        let clock = MockClock::new();
        let deadlines = Arc::new(std::sync::Mutex::new(Vec::new()));
        let drv = DeadlineProbeDriver {
            dof: 7,
            clock: clock.clone(),
            installed_deadlines: Arc::clone(&deadlines),
        };
        let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
        inner.lease_enabled = false;
        inner.is_torqued = true;
        inner.safety.set_torque(true).unwrap();
        inner.driver = Some(DriverSlot::exclusive(Box::new(drv)));

        let t0 = Instant::now();
        let rd = t0 + Duration::from_millis(10);
        // Simulate clock advancing past `rd` right at the driver-boundary sample.
        let mut calls = 0usize;
        let mut now_fn = || {
            calls += 1;
            if calls == 1 { t0 } else { rd + Duration::from_millis(5) }
        };
        let _ = inner.tick_control_inner(0.02, Some(rd), &mut now_fn);
        let captured = deadlines.lock().unwrap().clone();
        // DRV-F1 conservative driver-first cutoff translation: since call #2 (`owner_now`) is
        // already past `rd - margin`, expired cutoffs are rejected BEFORE staging any driver
        // deadline (zero `Some(_)` deadlines staged).
        assert!(
            captured.iter().all(Option::is_none),
            "expired RX/output cutoffs must reject before staging any driver deadline, got {captured:?}"
        );

        // Also verify `install_effective_tx_cutoff` directly when `owner_now` is past `rd`:
        // must return `DriverTxCapOutcome::Expired` with zero staged deadlines.
        deadlines.lock().unwrap().clear();
        let mut expired_now = || rd + Duration::from_millis(10);
        let (outcome, cutoff) = inner.install_effective_tx_cutoff(Some(rd), &mut expired_now);
        assert_eq!(outcome, DriverTxCapOutcome::Expired);
        assert!(cutoff.is_none());
        assert!(deadlines.lock().unwrap().is_empty());

        // When `owner_now` is still strictly before `rd - margin`, a live deadline IS staged.
        let mut live_now = || t0;
        let (live_outcome, live_cutoff) = inner.install_effective_tx_cutoff(Some(rd), &mut live_now);
        assert_eq!(live_outcome, DriverTxCapOutcome::Installed);
        let live_cutoff = live_cutoff.expect("live cutoff returned");
        assert!(live_cutoff.drv_release_cap > clock.now());
        assert_eq!(live_cutoff.owner_anchor, t0);
    }

    #[test]
    fn drv_lock_major7_gravity_comp_rechecks_disconnect_fence_after_enable_state() {
        use crate::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
        use crate::gravity_comp::{GravityCompPhase, GravityCompRun};
        use crate::model::{GravityCompPolicy, JointState};
        use crate::supervisor_lock::DriverSlot;

        #[derive(Clone, Copy)]
        enum InjectPoint {
            EnableState,
            SetOutputLeaseGen,
            None,
        }

        struct DisconnectInjectDriver {
            dof: usize,
            inject_at: InjectPoint,
            pending_disconnect_hold: Arc<AtomicU64>,
            writes: Arc<AtomicUsize>,
            hold_writes: Arc<AtomicUsize>,
            last_lease_gen: Arc<AtomicU64>,
        }
        impl ArmDriver for DisconnectInjectDriver {
            fn capabilities(&self) -> DriverCapabilities { DriverCapabilities::default() }
            fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
                Ok(vec![JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 30.0 }; self.dof])
            }
            fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> {
                self.writes.fetch_add(1, Ordering::SeqCst);
                Ok(())
            }
            fn write_estop_hold_bounded(
                &mut self,
                targets: &[JointTarget],
            ) -> Result<crate::driver::TxBatchOutcome, ArmDriverError> {
                self.hold_writes.fetch_add(1, Ordering::SeqCst);
                self.write_joint_targets_bounded(targets)
            }
            fn set_torque(&mut self, _enable: bool) -> Result<(), ArmDriverError> { Ok(()) }
            fn is_feedback_valid(&self) -> bool { true }
            fn is_enabled(&self) -> bool { true }
            fn enable_state(&self) -> EnableState {
                if matches!(self.inject_at, InjectPoint::EnableState) {
                    self.pending_disconnect_hold.fetch_add(1, Ordering::SeqCst);
                }
                EnableState::Enabled
            }
            fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::from_millis(1)); self.dof] }
            fn feedback_instants(&self) -> Vec<Option<Instant>> { vec![Some(Instant::now()); self.dof] }
            fn feedback_seqs(&self) -> Vec<u64> { vec![1; self.dof] }
            crate::ext_accessors!(tx);
        }
        impl crate::driver::BoundedTxExt for DisconnectInjectDriver {
            fn set_output_lease_gen(&mut self, gen: u64) {
                self.last_lease_gen.store(gen, Ordering::SeqCst);
                if matches!(self.inject_at, InjectPoint::SetOutputLeaseGen) && gen != 0 {
                    self.pending_disconnect_hold.fetch_add(1, Ordering::SeqCst);
                }
            }
        }

        let make_run = |phase: GravityCompPhase, lease_generation: Option<u64>| GravityCompRun {
            phase,
            policy: GravityCompPolicy::rebot_dm(),
            elapsed_s: 1.0,
            scale: 1.0,
            damping_kd: [0.0; 6],
            entry_pose: [0.0; 6],
            entry_torque: [0.0; 6],
            tau_ff: [0.0; 6],
            kp: [0.0; 6],
            kd: [0.0; 6],
            position: [0.0; 6],
            overspeed_count: 0,
            prev_measured: Some([0.0; 6]),
            lease_generation,
            workspace_status: crate::gravity_comp::GravityCompWorkspaceStatus::Inside,
            workspace_cache_key: None,
        };

        // (a) Periodic output suppresses non-hold write when disconnect arrives in `enable_state()`
        //     OR in `set_output_lease_gen()`, and resets staged `output_lease_gen` to 0.
        for inject_at in [InjectPoint::EnableState, InjectPoint::SetOutputLeaseGen] {
            let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
            inner.lease_enabled = true;
            let gen = inner.acquire_lease("c1").unwrap();
            assert!(gen > 0);
            inner.is_torqued = true;
            inner.torque_intent = TorqueIntent::On;
            inner.safety.set_torque(true).unwrap();
            let writes = Arc::new(AtomicUsize::new(0));
            let hold_writes = Arc::new(AtomicUsize::new(0));
            let last_lease_gen = Arc::new(AtomicU64::new(999));
            let drv = DisconnectInjectDriver {
                dof: 7,
                inject_at,
                pending_disconnect_hold: Arc::clone(&inner.pending_disconnect_hold),
                writes: Arc::clone(&writes),
                hold_writes: Arc::clone(&hold_writes),
                last_lease_gen: Arc::clone(&last_lease_gen),
            };
            inner.driver = Some(DriverSlot::exclusive(Box::new(drv)));
            inner.gravity_comp = Some(make_run(GravityCompPhase::Active, Some(gen)));

            let _ = inner.tick_control(0.02);
            assert_eq!(
                writes.load(Ordering::SeqCst),
                0,
                "periodic non-hold write must be suppressed"
            );
            assert!(
                inner.pending_disconnect_hold.load(Ordering::SeqCst) >= 1,
                "disconnect injection must have fired"
            );
            assert_eq!(
                last_lease_gen.load(Ordering::SeqCst),
                0,
                "staged output_lease_gen must be cleared to 0 on suppression"
            );
        }

        // (b) Immediate gravity-comp output suppresses non-hold writes on live disconnect in BOTH
        //     Active and Exiting (output_lease_gen == 0) phases, while preserving E-stop hold!
        for exiting in [false, true] {
            let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
            inner.lease_enabled = false;
            inner.is_torqued = true;
            inner.torque_intent = TorqueIntent::On;
            inner.safety.set_torque(true).unwrap();
            inner.pending_disconnect_hold.store(1, Ordering::SeqCst);
            let writes = Arc::new(AtomicUsize::new(0));
            let hold_writes = Arc::new(AtomicUsize::new(0));
            let last_lease_gen = Arc::new(AtomicU64::new(999));
            let drv = DisconnectInjectDriver {
                dof: 7,
                inject_at: InjectPoint::None,
                pending_disconnect_hold: Arc::clone(&inner.pending_disconnect_hold),
                writes: Arc::clone(&writes),
                hold_writes: Arc::clone(&hold_writes),
                last_lease_gen: Arc::clone(&last_lease_gen),
            };
            inner.driver = Some(DriverSlot::exclusive(Box::new(drv)));
            inner.gravity_comp = Some(make_run(
                if exiting {
                    GravityCompPhase::Exiting {
                        hold: [0.0; 6],
                        elapsed_s: 0.0,
                    }
                } else {
                    GravityCompPhase::Active
                },
                None,
            ));

            let out_non_hold = inner.write_gravity_comp_immediate_inner(None, false, false);
            assert!(matches!(out_non_hold, TickOutput::None), "immediate non-hold write must be suppressed (exiting={exiting})");
            assert_eq!(writes.load(Ordering::SeqCst), 0);
            assert_eq!(last_lease_gen.load(Ordering::SeqCst), 0);

            let out_hold = inner.write_gravity_comp_immediate_inner(None, true, false);
            assert!(matches!(out_hold, TickOutput::Complete { .. }), "immediate E-stop hold must still transmit");
            assert_eq!(hold_writes.load(Ordering::SeqCst), 1);
        }

        // (c) Live E-stop latching during `enable_state()` OR during `set_output_lease_gen()` on
        //     BOTH immediate AND periodic gravity-comp switches to `write_estop_hold_bounded`,
        //     clears staged lease gen to 0, and sets immediate J7 intent to `Stop`.
        for (inject_in_setter, immediate) in [(false, true), (true, true), (true, false)] {
            struct EstopInjectDriver {
                dof: usize,
                inject_in_setter: bool,
                estop_state: Arc<crate::rt_core::urgent::EstopState>,
                writes: Arc<AtomicUsize>,
                hold_writes: Arc<AtomicUsize>,
                last_lease_gen: Arc<AtomicU64>,
            }
            impl ArmDriver for EstopInjectDriver {
                fn capabilities(&self) -> DriverCapabilities { DriverCapabilities::default() }
                fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
                    Ok(vec![JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 30.0 }; self.dof])
                }
                fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> {
                    self.writes.fetch_add(1, Ordering::SeqCst);
                    Ok(())
                }
                fn write_estop_hold_bounded(
                    &mut self,
                    _targets: &[JointTarget],
                ) -> Result<crate::driver::TxBatchOutcome, ArmDriverError> {
                    self.hold_writes.fetch_add(1, Ordering::SeqCst);
                    Ok(crate::driver::TxBatchOutcome::Complete)
                }
                fn set_torque(&mut self, _enable: bool) -> Result<(), ArmDriverError> { Ok(()) }
                fn is_feedback_valid(&self) -> bool { true }
                fn is_enabled(&self) -> bool { true }
                fn enable_state(&self) -> EnableState {
                    if !self.inject_in_setter {
                        self.estop_state.request_and_latch();
                    }
                    EnableState::Enabled
                }
                fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::from_millis(1)); self.dof] }
                fn feedback_instants(&self) -> Vec<Option<Instant>> { vec![Some(Instant::now()); self.dof] }
                fn feedback_seqs(&self) -> Vec<u64> { vec![1; self.dof] }
                crate::ext_accessors!(tx);
            }
            impl crate::driver::BoundedTxExt for EstopInjectDriver {
                fn set_output_lease_gen(&mut self, gen: u64) {
                    self.last_lease_gen.store(gen, Ordering::SeqCst);
                    if self.inject_in_setter && gen != 0 {
                        self.estop_state.request_and_latch();
                    }
                }
            }

            let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
            inner.lease_enabled = true;
            let gen = inner.acquire_lease("c1").unwrap();
            inner.is_torqued = true;
            inner.torque_intent = TorqueIntent::On;
            inner.safety.set_torque(true).unwrap();
            let writes = Arc::new(AtomicUsize::new(0));
            let hold_writes = Arc::new(AtomicUsize::new(0));
            let last_lease_gen = Arc::new(AtomicU64::new(999));
            let drv = EstopInjectDriver {
                dof: 7,
                inject_in_setter,
                estop_state: Arc::clone(&inner.estop_state),
                writes: Arc::clone(&writes),
                hold_writes: Arc::clone(&hold_writes),
                last_lease_gen: Arc::clone(&last_lease_gen),
            };
            inner.driver = Some(DriverSlot::exclusive(Box::new(drv)));
            inner.gravity_comp = Some(make_run(GravityCompPhase::Active, Some(gen)));

            if immediate {
                let out = inner.write_gravity_comp_immediate_inner(None, false, false);
                assert!(
                    matches!(out, TickOutput::Complete { is_hold: true, .. }),
                    "immediate write must switch to hold on live E-stop (inject_in_setter={inject_in_setter}): {out:?}"
                );
                assert_eq!(writes.load(Ordering::SeqCst), 0);
                assert_eq!(hold_writes.load(Ordering::SeqCst), 1);
                assert_eq!(last_lease_gen.load(Ordering::SeqCst), 0);
                assert!(matches!(
                    inner.j7_intent.map(|i| i.kind),
                    Some(crate::driver::PosForceIntentKind::Stop)
                ));
            } else {
                let _ = inner.tick_control(0.02);
                assert_eq!(writes.load(Ordering::SeqCst), 0);
                assert!(hold_writes.load(Ordering::SeqCst) >= 1);
                assert_eq!(last_lease_gen.load(Ordering::SeqCst), 0);
            }
        }
    }

    #[test]
    fn drv_lock_budgeted_clear_fault_clock_expiry_under_guard_refuses_without_tx() {
        use crate::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
        use crate::model::JointState;
        use crate::safety::{ClearFaultError, ClearRefusal, FaultCode, FaultReason};
        use crate::supervisor_lock::DriverSlot;
        use crate::transport::MockClock;

        struct ExpiryProbeDriver {
            dof: usize,
            clock: MockClock,
            hw_fault: bool,
            tx_probe_pending: bool,
            deadlines_staged: Arc<AtomicUsize>,
            tx_calls: Arc<AtomicUsize>,
        }
        impl ArmDriver for ExpiryProbeDriver {
            fn capabilities(&self) -> DriverCapabilities { DriverCapabilities::default() }
            fn clock_now(&self) -> Instant { self.clock.now() }
            fn set_tx_release_deadline(&mut self, deadline: Option<Instant>) {
                if deadline.is_some() {
                    self.deadlines_staged.fetch_add(1, Ordering::SeqCst);
                }
            }
            fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
                Ok(vec![JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 30.0 }; self.dof])
            }
            fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
            fn set_torque(&mut self, _enable: bool) -> Result<(), ArmDriverError> { Ok(()) }
            fn clear_fault_budgeted(&mut self, _deadline: Instant) -> Result<(), ArmDriverError> {
                self.tx_calls.fetch_add(1, Ordering::SeqCst);
                Ok(())
            }
            fn has_hardware_fault(&self) -> bool { self.hw_fault }
            fn is_feedback_valid(&self) -> bool { true }
            fn is_enabled(&self) -> bool { false }
            fn enable_state(&self) -> EnableState { EnableState::Disabled }
            fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::from_millis(1)); self.dof] }
            fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
                out.clear();
                out.resize(self.dof, Some(Duration::from_millis(1)));
            }
            fn feedback_instants(&self) -> Vec<Option<Instant>> { vec![Some(self.clock.now()); self.dof] }
            fn feedback_seqs(&self) -> Vec<u64> { vec![1; self.dof] }
            crate::ext_accessors!(tx);
        }
        impl crate::driver::BoundedTxExt for ExpiryProbeDriver {
            fn probe_tx_ready_budgeted(&mut self, _deadline: Instant) -> Result<bool, ArmDriverError> {
                self.tx_calls.fetch_add(1, Ordering::SeqCst);
                Ok(true)
            }
            fn tx_probe_pending(&self) -> bool { false }
            fn tx_probe_confirmed(&self) -> bool { !self.tx_probe_pending }
        }

        // Test BOTH budgeted TX probe and budgeted hardware reset when owner clock reaches
        // `tx_deadline` right after `drv.clock_now()` under the guard: must return `BudgetExpired`
        // with 0 staged deadlines, 0 TX calls, and preserved `scratch.ages` capacity.
        for (test_reset, overshoot_ms) in [(false, 0u64), (false, 5u64), (true, 0u64), (true, 5u64)] {
            let clock = MockClock::new();
            let staged = Arc::new(AtomicUsize::new(0));
            let tx_calls = Arc::new(AtomicUsize::new(0));
            let drv = ExpiryProbeDriver {
                dof: 7,
                clock: clock.clone(),
                hw_fault: test_reset,
                tx_probe_pending: !test_reset,
                deadlines_staged: Arc::clone(&staged),
                tx_calls: Arc::clone(&tx_calls),
            };
            let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
            inner.lease_enabled = false;
            inner.driver = Some(DriverSlot::exclusive(Box::new(drv)));
            inner.scratch.ages = Vec::with_capacity(16);
            let orig_cap = inner.scratch.ages.capacity();
            if test_reset {
                let _ = inner
                    .safety
                    .trigger_fault_coded(FaultCode::Other, FaultReason::Other("hw fault".into()));
            } else {
                let _ = inner
                    .safety
                    .trigger_fault_coded(FaultCode::CanTxStalled, FaultReason::CanTxStalled);
            }

            let t0 = Instant::now();
            let release_deadline = t0 + Duration::from_millis(20);
            let tx_deadline = release_deadline - inner.tx_release_publish_margin();
            // First `now_fn()` call (pre-guard) is before `tx_deadline`; second `now_fn()` call
            // (inside guard after `drv.clock_now()`) is AT or PAST `tx_deadline`!
            let mut calls = 0usize;
            let mut live_now = || {
                calls += 1;
                if calls == 1 {
                    t0
                } else {
                    tx_deadline + Duration::from_millis(overshoot_ms)
                }
            };
            let res = inner.clear_fault_executor_bounded_for_release_with_clock(
                t0,
                release_deadline,
                None,
                &mut live_now,
            );
            assert!(
                matches!(res, Err(ClearFaultError::Rejected(ClearRefusal::BudgetExpired))),
                "expected BudgetExpired (test_reset={test_reset}, overshoot_ms={overshoot_ms}), got {res:?}"
            );
            assert_eq!(staged.load(Ordering::SeqCst), 0, "must not stage TX deadline after expiry");
            assert_eq!(tx_calls.load(Ordering::SeqCst), 0, "must not transmit after expiry");
            assert_eq!(inner.scratch.ages.capacity(), orig_cap, "scratch.ages must be restored on BudgetExpired");
        }
    }

    #[test]
    fn drv_lock_major8_park_disable_stop_gen_max_cancels_cleanly_on_estop_inhibited() {
        use crate::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget, TorqueProgress};
        use crate::model::JointState;
        use crate::rt_core::urgent::StopRequests;
        use crate::supervisor_lock::DriverSlot;

        struct StopGenLeadParkDriver {
            dof: usize,
            owner_stop_gen: Arc<AtomicU64>,
        }
        impl ArmDriver for StopGenLeadParkDriver {
            fn capabilities(&self) -> DriverCapabilities { DriverCapabilities::default() }
            fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
                Ok(vec![JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 30.0 }; self.dof])
            }
            fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
            fn set_torque(&mut self, _enable: bool) -> Result<(), ArmDriverError> { Ok(()) }
            fn begin_client_park_disable(
                &mut self,
                _max_rx_frames: Option<u16>,
                _expected_stop_gen: Option<u64>,
            ) -> Result<TorqueProgress, ArmDriverError> {
                // Advance owner-local `stop_gen` ahead of `StopRequests::current()` and fail
                // with `EstopInhibited` (modeling a local stop before any disable frame hit the wire).
                self.owner_stop_gen.fetch_add(1, Ordering::SeqCst);
                Err(ArmDriverError::EstopInhibited)
            }
            fn is_feedback_valid(&self) -> bool { true }
            fn is_enabled(&self) -> bool { true }
            fn enable_state(&self) -> EnableState { EnableState::Enabled }
            fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::from_millis(1)); self.dof] }
            fn feedback_instants(&self) -> Vec<Option<Instant>> { vec![Some(Instant::now()); self.dof] }
            fn feedback_seqs(&self) -> Vec<u64> { vec![1; self.dof] }
        }

        let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
        inner.lease_enabled = false;
        inner.is_torqued = true;
        inner.safety.set_torque(true).unwrap();
        inner.commanded_velocities.fill(0.0);
        inner.attach_stop_requests(StopRequests::new());
        let drv = StopGenLeadParkDriver {
            dof: 7,
            owner_stop_gen: Arc::clone(&inner.stop_gen),
        };
        inner.driver = Some(DriverSlot::exclusive(Box::new(drv)));

        inner.begin_park_disable_transition(false, None);
        assert!(
            inner.torque_transition.is_none(),
            "client park stopped via owner-local stop_gen must cancel cleanly without torque_transition"
        );
        assert!(
            !inner.safety.state().is_stopped(),
            "clean park stop-gate cancellation must not latch E-stop/stopped fault"
        );
    }
}

#[cfg(test)]
mod stamp_namespace_tests {
    use super::*;

    /// 2026-11-27 real arm (rc7, executor): after cockpit jogging, grasp/Home were refused with
    /// STAMP_REUSED. Interleaved RT-internal (servo frame) and ordinary IDs from the shared
    /// counter fragmented one tracker; the capacity fold moved the watermark past every
    /// ordinary ID. Each namespace now has its own tracker.
    #[test]
    fn jog_frames_do_not_consume_future_ordinary_ids() {
        let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
        let mut ordinary_used = Vec::new();
        for i in 0..4000u32 {
            let internal = next_admission_request_id() | RT_INTERNAL_REQUEST_ID_BIT;
            assert!(!inner.is_stamp_consumed(internal));
            inner.record_consumed_stamp(internal);
            // an ordinary ID allocated between frames (most are never consumed here)
            let ordinary = next_admission_request_id();
            if i % 7 == 0 {
                inner.record_consumed_stamp(ordinary);
                ordinary_used.push(ordinary);
            }
        }
        let fresh = next_admission_request_id();
        assert!(!inner.is_stamp_consumed(fresh), "a fresh ordinary ID must stay admissible after jogging");
        let fresh_internal = next_admission_request_id() | RT_INTERNAL_REQUEST_ID_BIT;
        assert!(!inner.is_stamp_consumed(fresh_internal));
        // replay protection is kept in both namespaces
        assert!(ordinary_used.iter().all(|&id| inner.is_stamp_consumed(id)));
        inner.record_consumed_stamp(fresh_internal);
        assert!(inner.is_stamp_consumed(fresh_internal));
        inner.record_consumed_stamp(fresh);
        assert!(inner.is_stamp_consumed(fresh));
    }
}
