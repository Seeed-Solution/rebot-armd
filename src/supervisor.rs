use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicU8, AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::broadcast;
use crate::supervisor_lock::SupervisorLock;
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::rt_core::scratch::{copy_into, copy_limits, fill_n};
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
mod streams;
mod actions;

/// Broker side of the servo, teleop and policy latest-target path (track D); the WS loop
/// and the policy runner submit through it once they route through the mailbox.
pub use streams::targets;
pub use driver_transitions::{run_with_panic_deadline_observer, PanicDisableOutcome, PANIC_DISABLE_TIMEOUT};
pub(crate) use driver_transitions::ClearFaultCommit;
pub(crate) use driver_transitions::{panic_disable_deadlines, run_with_panic_deadline_observer_until};

/// Global monotonic allocator for single-use [`AdmissionStamp::request_id`] values.
static NEXT_ADMISSION_REQUEST_ID: AtomicU64 = AtomicU64::new(1);

/// High bit reserving an isolated single-use namespace for RT broker `RequestId` values so
/// per-broker monotonic IDs never collide with global [`next_admission_request_id`] IDs.
const BROKER_REQUEST_ID_BIT: u64 = 1u64 << 63;

/// Allocates a fresh non-zero, single-use admission request ID (lock-free, zero-alloc).
#[inline]
pub fn next_admission_request_id() -> u64 {
    let id = NEXT_ADMISSION_REQUEST_ID.fetch_add(1, Ordering::Relaxed);
    assert!(
        id != 0 && (id & BROKER_REQUEST_ID_BIT) == 0,
        "admission request_id overflow"
    );
    id
}

/// Generation snapshot captured ONCE at request admission (lock-free in the WS reader
/// before queuing, or at REST/caller entry) and required by every supervisor/owner function
/// that starts motion or output.
///
/// Although `AdmissionStamp` is [`Copy`] so callers can carry it across off-lock planning,
/// each stamp carries a unique non-zero [`request_id`](Self::request_id) consumed once by
/// [`SupervisorInner::check_admission`] under the owner lock; any second use is refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdmissionStamp {
    pub stop_gen: u64,
    pub estop_epoch: u64,
    pub lease_gen: u64,
    /// Monotonic WS connection ID (`>= 1`) captured at WS admission, or `0` for non-WS callers.
    /// Checked against the owner-lock closed-connection set so a stale stamp from a closed
    /// connection is always refused.
    pub connection_id: u64,
    /// Globally unique non-zero request ID consumed once by [`SupervisorInner::check_admission`].
    /// A zero or already-consumed `request_id` is refused fail-closed.
    pub request_id: u64,
}

impl AdmissionStamp {
    /// Derives a fresh single-use child [`AdmissionStamp`] for a macro sub-motion while
    /// preserving the parent's admission-captured `stop_gen`, `estop_epoch`, `lease_gen`, and
    /// `connection_id` verbatim (never re-sampling live owner authority).
    pub fn for_macro_child(self) -> Self {
        Self {
            request_id: if self.request_id == 0 {
                0
            } else {
                next_admission_request_id()
            },
            ..self
        }
    }

    /// Constructs a single-use [`AdmissionStamp`] for an RT broker request using its broker
    /// `RequestId` in an isolated namespace (zero-alloc; `0` stays `0` and is refused fail-closed).
    pub(crate) fn from_broker(
        stop_gen: u64,
        estop_epoch: u64,
        lease_gen: u64,
        broker_request_id: u64,
    ) -> Self {
        Self {
            stop_gen,
            estop_epoch,
            lease_gen,
            connection_id: 0,
            request_id: if broker_request_id == 0 {
                0
            } else {
                broker_request_id | BROKER_REQUEST_ID_BIT
            },
        }
    }

    /// Constructs a fresh single-use [`AdmissionStamp`] for an RT-internal latest-target or
    /// stream-handback step (zero-alloc atomic counter).
    pub(crate) fn for_rt_internal(stop_gen: u64, estop_epoch: u64, lease_gen: u64) -> Self {
        Self {
            stop_gen,
            estop_epoch,
            lease_gen,
            connection_id: 0,
            request_id: next_admission_request_id(),
        }
    }
}

/// Opaque, single-use admission proof token created ONLY by [`SupervisorInner::check_admission`]
/// under the owner lock and borrowing the owner lock guard for `'g`.
///
/// It cannot be constructed externally, cloned, copied, sent across threads/`.await` points,
/// stored past the guard lifetime `'g`, or held after dropping the owner lock guard, and every
/// function that starts motion or output consumes it by value:
///
/// ```compile_fail
/// use armd::{AdmissionStamp, Admitted};
/// let _forged = Admitted(AdmissionStamp { stop_gen: 0, estop_epoch: 1, lease_gen: 0, connection_id: 0, request_id: 1 });
/// ```
///
/// ```compile_fail
/// fn assert_copy<T: Copy>() {}
/// assert_copy::<armd::Admitted<'_>>();
/// ```
///
/// ```compile_fail
/// fn assert_clone<T: Clone>() {}
/// assert_clone::<armd::Admitted<'_>>();
/// ```
///
/// ```compile_fail
/// fn assert_send<T: Send>() {}
/// assert_send::<armd::Admitted<'_>>();
/// ```
///
/// Holding [`Admitted`] after dropping the owner lock guard does not compile:
///
/// ```compile_fail,E0505
/// use armd::{ArmSupervisor, SupervisorInner};
/// async fn cannot_drop_guard_while_holding_admitted(sup: &ArmSupervisor) {
///     let stamp = sup.capture_admission_stamp();
///     let mut guard = sup.inner().write().await;
///     let admitted = guard.check_admission(stamp).unwrap();
///     drop(guard);
///     let _ = SupervisorInner::enable_torque(admitted);
/// }
/// ```
///
/// ```compile_fail,E0597
/// use armd::{ArmSupervisor, SupervisorInner};
/// async fn cannot_return_admitted_past_guard_scope(sup: &ArmSupervisor) {
///     let stamp = sup.capture_admission_stamp();
///     let admitted = {
///         let mut guard = sup.inner().write().await;
///         guard.check_admission(stamp).unwrap()
///     };
///     let _ = SupervisorInner::enable_torque(admitted);
/// }
/// ```
///
/// Mutating the owner state while holding [`Admitted`] or storing it in `'static` does not compile:
///
/// ```compile_fail,E0499
/// use armd::{ArmConfig, SupervisorInner};
/// let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
/// let stamp = inner.current_admission_stamp();
/// let admitted = inner.check_admission(stamp).unwrap();
/// let _ = inner.authorized_stop();
/// let _ = SupervisorInner::enable_torque(admitted);
/// ```
///
/// ```compile_fail
/// fn assert_static<T: 'static>() {}
/// fn reject_static<'a>() { assert_static::<armd::Admitted<'a>>(); }
/// ```
///
/// Calling a motion or output start without an [`Admitted`] token does not compile:
///
/// ```compile_fail,E0061
/// use armd::SupervisorInner;
/// let _ = SupervisorInner::jog(vec![0.0; 7], None, None);
/// ```
///
/// ```compile_fail,E0061
/// use armd::SupervisorInner;
/// let _ = SupervisorInner::enable_torque();
/// ```
///
/// ```compile_fail,E0061
/// use armd::motion::{GripperCommand, GripperRequest};
/// use armd::SupervisorInner;
/// let req = GripperRequest { command: GripperCommand::Open, fraction: None, grasp: false, speed: 1.0, timeout_s: None, tag: None };
/// let _ = SupervisorInner::start_gripper_with_baseline(req, 1, armd::supervisor::GripperBaselineCmd::None);
/// ```
///
/// ```compile_fail,E0061
/// use armd::motion::{GripperCommand, GripperRequest, LearnMode};
/// use armd::SupervisorInner;
/// let req = GripperRequest { command: GripperCommand::Open, fraction: None, grasp: false, speed: 1.0, timeout_s: None, tag: None };
/// let _ = SupervisorInner::start_macro_gripper_learning(1, req, LearnMode::Calibration);
/// ```
///
/// ```compile_fail,E0061
/// use std::sync::Arc;
/// use armd::{ArmConfig, ArmSupervisor};
/// let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
/// let _ = armd::gripper_calib::begin_calibration(&sup, 1, None);
/// ```
///
/// ```compile_fail,E0061
/// use std::sync::Arc;
/// use armd::{ArmConfig, ArmSupervisor};
/// let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
/// let _ = armd::gripper_calib::start_calibration(&sup);
/// ```
#[must_use = "Admitted token must be consumed by a motion/output start"]
pub struct Admitted<'g> {
    inner: &'g mut SupervisorInner,
    stamp: AdmissionStamp,
    _not_send_sync: std::marker::PhantomData<*mut ()>,
}

impl std::fmt::Debug for Admitted<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Admitted").field("stamp", &self.stamp).finish()
    }
}

/// Crate-private, non-`Copy`, non-`Clone` witness proving a live [`Admitted`] token was
/// verified and consumed inside `armd`. Only [`Admitted::consume`] can construct it.
pub(crate) struct AdmittedProof {
    stamp: AdmissionStamp,
}

impl AdmittedProof {
    #[inline]
    pub(crate) fn stamp(&self) -> AdmissionStamp {
        self.stamp
    }
}

impl<'g> Admitted<'g> {
    /// The validated stamp captured at request admission.
    #[inline]
    pub fn stamp(&self) -> AdmissionStamp {
        self.stamp
    }

    /// Re-verifies this token against live owner state (including lock-free atomic E-stop
    /// and urgent-stop counters) without consuming it.
    pub(crate) fn verify_live(&self) -> Result<AdmissionStamp, SafetyError> {
        self.inner.verify_admission_stamp(self.stamp)?;
        Ok(self.stamp)
    }

    /// Consumes this single-use token by value, re-verifies its stamp against live owner
    /// state, and yields `&'g mut SupervisorInner` plus a crate-private [`AdmittedProof`].
    pub(crate) fn consume(self) -> Result<(&'g mut SupervisorInner, AdmittedProof), SafetyError> {
        let stamp = self.stamp;
        let inner = self.inner;
        inner.verify_admission_stamp(stamp)?;
        // Record the A1 admission `(stop_gen, estop_epoch)` directly from the admitted command's
        // stamp so J7 `Target` intents carry their admission stamp and are never re-stamped later
        // (and never stamped with `max(local, requested)`).
        inner.j7_admitted_stamp = Some(crate::driver::PosForceIntentStamp {
            stop_gen: stamp.stop_gen,
            estop_epoch: stamp.estop_epoch,
        });
        Ok((inner, AdmittedProof { stamp }))
    }
}

/// Complete catalog of motion/output start entry points gated by [`AdmissionStamp`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MotionStartKind {
    Jog,
    Home,
    Park,
    Preset,
    MoveTo,
    MoveRelative,
    MoveToPose,
    Place,
    Grasp,
    Replay,
    Sequence,
    CalibHover,
    MeasureMultiview,
    MacroStart,
    MacroChildPath,
    MacroChildQuintic,
    MacroChildGripper,
    MacroChildReplay,
    Servo,
    PolicyShadow,
    PolicyLive,
    GravityComp,
    Gripper,
    GripperCalibrate,
    GripperZeroCheck,
    TorqueEnable,
}

impl MotionStartKind {
    pub const ALL: &'static [MotionStartKind] = &[
        Self::Jog,
        Self::Home,
        Self::Park,
        Self::Preset,
        Self::MoveTo,
        Self::MoveRelative,
        Self::MoveToPose,
        Self::Place,
        Self::Grasp,
        Self::Replay,
        Self::Sequence,
        Self::CalibHover,
        Self::MeasureMultiview,
        Self::MacroStart,
        Self::MacroChildPath,
        Self::MacroChildQuintic,
        Self::MacroChildGripper,
        Self::MacroChildReplay,
        Self::Servo,
        Self::PolicyShadow,
        Self::PolicyLive,
        Self::GravityComp,
        Self::Gripper,
        Self::GripperCalibrate,
        Self::GripperZeroCheck,
        Self::TorqueEnable,
    ];
}

/// Single sealed execution contract for [`SupervisorInner::start_motion`]: an operation can only
/// run when handed a single-use [`Admitted`] token produced by [`SupervisorInner::check_admission`]
/// under the owner lock.
///
/// ```compile_fail,E0061
/// use armd::SupervisorInner;
/// // Calling start_motion without an Admitted token fails to compile:
/// let _ = SupervisorInner::start_motion(());
/// ```
pub trait MotionStartOp {
    type Output;
    fn kind(&self) -> MotionStartKind;
    fn execute(self, admitted: Admitted<'_>) -> Result<Self::Output, ArmError>;
}

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

    /// Zero-allocation static refusal for RT paths (`armd-control` never allocates `String`).
    #[error("{code}: {message}")]
    RejectedStatic { status: u16, code: &'static str, message: &'static str },

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
    #[error("OWNER_TRANSFERRED: control ownership transferred to RT executor; shell lease calls are refused")]
    OwnerTransferred,
}

/// J7 stall latch: |τ7| at or above this (N·m) with the jaw making no progress over the
/// stall window. Tuned on hardware as 0.80 when τ7 was decoded with TMAX 18; the J7 motor's
/// TMAX is 10, so the same physical torque is 0.80 × 10 / 18.
pub const GRIPPER_STALL_LATCH_NM: f32 = 0.444;
/// Minimum measured net position progress (rad) along the commanded direction proving the jaw
/// is advancing under load (~2.5x the 0.0008 rad / 2-LSB stall jitter measured on B601-DM J7,
/// well below the 0.0057 rad median 50 Hz closing tick).
pub const GRIPPER_STALL_PROGRESS_EPS: f32 = 0.002;
/// Hard upper bound (s) on the stall-detection window so a real jam fails within <= 0.5 s after
/// progress stops, independent of the action timeout.
pub const GRIPPER_STALL_WINDOW_MAX_S: f64 = 0.50;
/// J7 stall release: |τ7| below this (N·m) for 6 ticks (formerly 0.40 at TMAX 18;
/// 0.40 × 10 / 18 = 0.2222). Raw torque 2093 decodes to exactly that value and was read
/// as 0.39999962 < 0.40 before, so the bound sits just above it.
pub const GRIPPER_STALL_RELEASE_NM: f32 = 0.2223;
/// A gripper action that ends with its J7 command more than this far from the measured jaw
/// (rad) and no object held has J7 set to the measured jaw on the next tick.
pub const GRIPPER_END_LEAD_RAD: f32 = 0.02;
/// During a `gripper_calibrate` stroke the torque stall latch needs the J7 command at least this
/// far from the measured jaw (rad): a blocked jaw falls behind its command, a free jaw settling
/// onto its target does not (it reads 0.5-0.7 N·m of friction at low speed near open on the
/// B601-DM new finger).
pub const GRIPPER_STALL_MIN_LEAD_RAD: f32 = 0.05;
/// Gripper torque cap: consecutive ticks with |τ| above `gripper_policy.max_close_torque_nm`
/// (fresh feedback, jaw driven closed; or the torque pushing open above it while driven open)
/// that latch the stall regardless of the jaw speed. One: the first sample over the cap latches
/// (at two, one impact sample of a stiff contact passed before the latch).
pub const GRIPPER_FORCE_CAP_TICKS: u32 = 1;
/// After a torque-cap latch under a servo stream or live policy that still closes, the grip
/// ramps back in ([`crate::motion::CapRamp`]: steps sized from the contact stiffness), at most
/// this fast (rad/s) ...
pub const GRIPPER_STREAM_CAP_RAMP_RAD_S: f32 = 0.25;
/// ... until |τ| reaches this fraction of `max_close_torque_nm`, then holds.
pub const GRIPPER_STREAM_CAP_HOLD_FRACTION: f32 = 0.6;
/// The learned free-motion torque may raise the closing stall threshold above
/// GRIPPER_STALL_LATCH_NM by at most this (N·m); opening keeps GRIPPER_STALL_LATCH_NM.
pub const GRIPPER_STALL_BASELINE_RAISE_MAX_NM: f32 = 0.15;
/// Finished gripper moves queued for learning; further ones are dropped until the queue is
/// processed (no allocation on the control tick).
pub const GRIPPER_BASELINE_PENDING_MAX: usize = 4;

/// Records of the strokes of a running `gripper_calibrate` action (id), in stroke order.
#[derive(Debug, Clone)]
pub struct GripperCalibrationRun {
    pub id: u64,
    pub records: Vec<crate::gripper_baseline::MoveRecord>,
}

/// What a gripper action does with the learned free-motion baseline (tests and tools; the API
/// calibrates with the `gripper_calibrate` action).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GripperBaselineCmd {
    /// Ordinary action: only opening moves are learned.
    #[default]
    None,
    /// `"calibrate"`: a calibration stroke with nothing in the jaws; non-grasp closes are learned too.
    Calibrate,
    /// `"reset"`: forget the baseline once the action is admitted, then as `Calibrate`.
    Reset,
}
/// The learned free-motion torque may raise the stream regulation target above
/// `contact_torque_nm` by at most this (N·m); it raises the stream contact detection fully.
pub const GRIPPER_STREAM_TARGET_RAISE_MAX_NM: f32 = 0.10;
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
/// Maximum allowed gap between consecutive fresh temperature samples before thermal cooldown
/// dwell restarts from the next fresh sample.
pub const THERMAL_FEEDBACK_FRESHNESS_LIMIT: Duration = Duration::from_millis(250);
/// Outward tolerance (rad) when checking whether measured joint positions are back within
/// configured limits before clearing [`crate::safety::FaultKind::JointLimit`].
pub const JOINT_LIMIT_CLEAR_MARGIN_RAD: f32 = 0.01;
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

/// One stationary-settle sample stored inline without heap vectors (`MAX_DOF = 8`),
/// so clearing the settle window on a deferred output gap never allocates or frees on RT.
#[derive(Debug, Clone, Copy)]
pub struct UnloadSettleSample {
    pub t: f64,
    pub dof: usize,
    pub positions: [f32; crate::rt_core::mailbox::MAX_DOF],
    pub velocities: [f32; crate::rt_core::mailbox::MAX_DOF],
}

impl UnloadSettleSample {
    pub fn from_slices(t: f64, q: &[f32], v: &[f32]) -> Self {
        let dof = q.len().min(v.len()).min(crate::rt_core::mailbox::MAX_DOF);
        let mut positions = [f32::NAN; crate::rt_core::mailbox::MAX_DOF];
        let mut velocities = [f32::NAN; crate::rt_core::mailbox::MAX_DOF];
        positions[..dof].copy_from_slice(&q[..dof]);
        velocities[..dof].copy_from_slice(&v[..dof]);
        Self { t, dof, positions, velocities }
    }
}

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
    /// Preallocated inline sample ring over the last [`UNLOAD_SETTLE_HOLD`] while held.
    pub samples: std::collections::VecDeque<UnloadSettleSample>,
    /// `pending_s` of the last blocker report.
    pub last_report_s: Option<f64>,
}

impl UnloadSettle {
    pub fn with_capacity(cap: usize) -> Self {
        Self {
            samples: std::collections::VecDeque::with_capacity(cap),
            ..Self::default()
        }
    }
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

/// Owner identity bound to a [`DeferredActionDone`] so a later `Complete` never commits a
/// stale or superseded action, macro child, or internal park gripper step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum DeferredCompletionTarget {
    Action(u64),
    MacroSub {
        action_id: u64,
        ticket: crate::macro_action::ChildTicket,
    },
    ParkRelease(ParkRelease),
    Anonymous,
}

/// Action completion captured during trajectory stepping and published only after
/// that tick's output passes tracking validation and is accepted (`Complete`) by the driver.
pub(crate) struct DeferredActionDone {
    pub(crate) target: DeferredCompletionTarget,
    pub(crate) epoch_id: u64,
    pub(crate) lease_generation: Option<u64>,
    pub(crate) settled: Option<bool>,
    pub(crate) holding: Option<bool>,
    pub(crate) gripper_gap_rad: Option<f32>,
    pub(crate) clear_releasing: bool,
    pub(crate) is_gripper: bool,
}

/// Zero-allocation classification of a bounded driver output write in one release,
/// explicitly bound to whether the submitted batch was a stopped hold (`is_hold`).
#[derive(Debug)]
#[allow(clippy::large_enum_variant)] // inline OutputErrorText avoids RT heap allocation
pub(crate) enum TickOutput {
    None,
    /// Driver refused target/hold because an E-stop or authorized stop is pending/latched.
    /// Zero-allocation typed refusal (avoids formatting or matching `ESTOP_INHIBITED`).
    EstopInhibited { is_hold: bool },
    Complete {
        is_hold: bool,
        now: Instant,
        max_age: Duration,
        due_mask: u64,
        sent_mask: u64,
        pf_status: crate::driver::PosForceTxStatus,
    },
    Deferred {
        pending: usize,
        now: Instant,
        max_age: Duration,
        oldest_age: Option<Duration>,
        due_mask: u64,
        is_hold: bool,
        sent_mask: u64,
        pf_status: crate::driver::PosForceTxStatus,
    },
    Expired {
        dropped: usize,
        pending: usize,
        now: Instant,
        max_age: Duration,
        oldest_age: Option<Duration>,
        due_mask: u64,
        sent_mask: u64,
        is_hold: bool,
        pf_status: crate::driver::PosForceTxStatus,
    },
    Error {
        message: crate::mit::OutputErrorText,
        now: Instant,
        max_age: Duration,
        is_hold: bool,
        sent_any: bool,
        due_mask: u64,
        sent_mask: u64,
        pf_status: crate::driver::PosForceTxStatus,
    },
}

impl TickOutput {
    /// True when the active J7 position-force gripper was stale (> 100 ms) during this pass
    /// (receiving a freeze frame instead of any new target); prevents J7-dependent action
    /// completions from committing on that tick.
    #[inline]
    pub(crate) fn j7_stale(&self) -> bool {
        matches!(
            self,
            Self::Complete { pf_status: crate::driver::PosForceTxStatus::Stale { .. }, .. }
                | Self::Deferred { pf_status: crate::driver::PosForceTxStatus::Stale { .. }, .. }
                | Self::Expired { pf_status: crate::driver::PosForceTxStatus::Stale { .. }, .. }
                | Self::Error { pf_status: crate::driver::PosForceTxStatus::Stale { .. }, .. }
        )
    }
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
    /// The torque transition the RT started and serves from `poll_hardware` (None: none).
    pub(crate) torque_transition: Option<driver_transitions::TorqueTransition>,
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
    /// Monotonic stop generation incremented under the owner lock whenever a Stop applies.
    /// Shared lock-free with [`ArmSupervisor`] so WS/REST admission can capture [`AdmissionStamp`].
    pub stop_gen: Arc<AtomicU64>,
    /// Lock-free mirror of the active lease generation (`0` when no lease is held).
    pub lease_gen_mirror: Arc<AtomicU64>,
    pub last_preemption: Option<Instant>,
    /// Connection and generation most recently evicted by a forced acquire.
    pub last_evicted: Option<(String, u64)>,
    pub handover_settling_since: Option<Instant>,
    /// Running gravity-compensation session (`src/gravity_comp.rs`).
    pub gravity_comp: Option<crate::gravity_comp::GravityCompRun>,
    /// Retired gravity-compensation session waiting for off-thread drop (never dropped on RT).
    pub(crate) retired_gravity_comp: Option<crate::gravity_comp::GravityCompRun>,
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
    /// Pending disk persist + commit payload produced by finish_action during ArmSupervisor::tick.
    pub pending_zero_check_persist: Option<PendingZeroCheckPersist>,
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
    /// Irreversibly set by [`Self::commit_shutdown`] under the owner's authority once final
    /// shutdown readiness is confirmed; permanently blocks `clear_fault`, torque enable, and
    /// actuator motion admission while keeping hardware disable paths permitted.
    pub shutdown_committed: bool,
    /// True after a shutdown request was refused because hardware `enable_state()` was `Unknown`,
    /// so a repeated shutdown signal issues a bounded disable request instead of only re-checking.
    pub(crate) shutdown_refused_unknown: bool,
    /// Preallocated bounded journal of immutable E-stop acknowledgements recorded at the exact
    /// moment [`Self::consume_pending_estop`] applies each urgent request (cap 64; never grows).
    pub(crate) pending_estop_acks: std::collections::VecDeque<crate::rt_core::executor::EstopAck>,
    /// Highest E-stop request generation dropped if `pending_estop_acks` ever reached capacity 64.
    pub(crate) estop_ack_overflow_gen: u64,
    /// Preallocated queue of heap-owning motion payloads retired on `armd-control` for off-thread drop.
    pub(crate) pending_retired_stops: std::collections::VecDeque<crate::rt_core::urgent::RetiredStopPayloads>,
    /// Coalesced sorted inclusive intervals `(lo, hi)` of closed WS connection IDs (`>= 1`),
    /// set under the owner lock during teardown and checked on every motion/output install.
    closed_connections: ClosedConnections,
    /// Bounded, preallocated single-use tracker for consumed global admission request IDs.
    consumed_stamps: ConsumedStamps,
    /// Bounded, preallocated single-use tracker for consumed RT broker `RequestId` values.
    consumed_broker_stamps: ConsumedStamps,
    /// Shared per-connection bounded action-terminal FIFO broadcaster (wired by `ArmSupervisor::new`;
    /// `None` on standalone `SupervisorInner::new`, never locked on `armd-control`).
    terminals: Option<Arc<TerminalBroadcaster>>,
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
    /// Current control tick's J7 `PosForce` intent set by arm/motion/hold paths and consumed
    /// strictly once by the tick epilogue's single J7 emitter (`emit_j7_tick_epilogue`).
    pub(crate) j7_intent: Option<crate::driver::PosForceIntent>,
    /// Admission `(stop_gen, estop_epoch)` captured directly from the A1 [`AdmissionStamp`] of
    /// the command that produced the active J7 target. Never re-stamped at output time and never
    /// stamped with `max(local, requested)`; cleared on stop, E-stop, and torque-off.
    pub(crate) j7_admitted_stamp: Option<crate::driver::PosForceIntentStamp>,
    /// Optional deterministic test seam invoked right after `mit.rs` checks
    /// `urgent_stop_unapplied()` and before J7 intent stamping (`None` in production, 0 alloc).
    #[doc(hidden)]
    pub post_stop_check_hook: Option<Arc<dyn Fn() + Send + Sync>>,
}

/// Copy `src` into `dst` (for `ArmObservation::{action, previous_action}`), routing
/// `Option<String>` message transitions (`None <-> Some`) through `spare` so `armd-control`
/// never allocates or drops heap strings on action start, failure, or `previous_action` expiry.
fn sync_action_obs(
    dst: &mut Option<ActionStatus>,
    src: Option<&ActionStatus>,
    spare: Option<&mut String>,
) {
    match (src, dst.as_mut()) {
        (Some(s), Some(d)) => {
            if let Some(sp) = spare {
                match (s.message.is_some(), d.message.is_some()) {
                    (true, false) if sp.capacity() > 0 => {
                        d.message = Some(std::mem::take(sp));
                    }
                    (false, true) => {
                        if let Some(old) = d.message.take() {
                            if old.capacity() > sp.capacity() {
                                *sp = old;
                            }
                        }
                    }
                    _ => {}
                }
            }
            d.clone_from(s);
        }
        (Some(s), None) => {
            if let (Some(src_msg), Some(sp)) = (s.message.as_deref(), spare) {
                if sp.capacity() >= src_msg.len() {
                    let mut blank = ActionStatus::blank();
                    blank.message = Some(std::mem::take(sp));
                    blank.clone_from(s);
                    *dst = Some(blank);
                    return;
                }
            }
            *dst = Some(s.clone());
        }
        (None, Some(d)) => {
            if let Some(sp) = spare {
                if let Some(old) = d.message.take() {
                    if old.capacity() > sp.capacity() {
                        *sp = old;
                    }
                }
            }
            *dst = None;
        }
        (None, None) => {}
    }
}

/// Zero-allocation tracker of continuous thermal cooldown dwell across consecutive fresh
/// temperature samples. Dwell is the span between consecutive fresh cool samples; any unobserved
/// gap longer than [`THERMAL_FEEDBACK_FRESHNESS_LIMIT`] between consecutive fresh samples restarts
/// the dwell from the next fresh sample.
#[derive(Debug, Clone, Copy, Default)]
pub(crate) struct ThermalCooldown {
    first_sample_at: Option<Instant>,
    last_oldest_sample_at: Option<Instant>,
    last_joint_samples: [Option<Instant>; crate::rt_core::mailbox::MAX_DOF],
    joint_count: usize,
}

impl ThermalCooldown {
    pub(crate) fn reset(&mut self) {
        self.first_sample_at = None;
        self.last_oldest_sample_at = None;
        self.last_joint_samples.fill(None);
        self.joint_count = 0;
    }

    pub(crate) fn elapsed_ms(&self) -> u64 {
        match (self.first_sample_at, self.last_oldest_sample_at) {
            (Some(first), Some(last)) => last
                .saturating_duration_since(first)
                .as_millis()
                .min(u128::from(u64::MAX)) as u64,
            _ => 0,
        }
    }

    pub(crate) fn record_fresh_cool_samples(&mut self, samples: &[Instant]) {
        let (Some(oldest), Some(newest)) = (samples.iter().copied().min(), samples.iter().copied().max()) else {
            self.reset();
            return;
        };
        let track_joints = samples.len() <= crate::rt_core::mailbox::MAX_DOF;
        match (self.first_sample_at, self.last_oldest_sample_at) {
            (Some(first), Some(prev_oldest)) => {
                let oldest_gapped =
                    oldest.saturating_duration_since(prev_oldest) > THERMAL_FEEDBACK_FRESHNESS_LIMIT;
                let joint_gapped = track_joints
                    && self.joint_count == samples.len()
                    && samples.iter().enumerate().any(|(i, &t)| {
                        self.last_joint_samples[i].is_none_or(|prev| {
                            t.saturating_duration_since(prev) > THERMAL_FEEDBACK_FRESHNESS_LIMIT
                        })
                    });
                let gapped = oldest_gapped || joint_gapped;
                if gapped {
                    // Continuity across all joints is only proven from the newest sample on: a joint
                    // that resumed late must not credit dwell from an older joint's timestamp.
                    self.first_sample_at = Some(newest);
                    self.last_oldest_sample_at = Some(oldest);
                } else {
                    self.first_sample_at = Some(first);
                    self.last_oldest_sample_at = Some(prev_oldest.max(oldest));
                }
                if track_joints {
                    if gapped || self.joint_count != samples.len() {
                        self.last_joint_samples.fill(None);
                        for (slot, &t) in self.last_joint_samples.iter_mut().zip(samples.iter()) {
                            *slot = Some(t);
                        }
                    } else {
                        for (slot, &t) in self.last_joint_samples.iter_mut().zip(samples.iter()) {
                            *slot = Some(slot.map_or(t, |prev| prev.max(t)));
                        }
                    }
                    self.joint_count = samples.len();
                } else {
                    self.last_joint_samples.fill(None);
                    self.joint_count = 0;
                }
            }
            _ => {
                self.first_sample_at = Some(oldest);
                self.last_oldest_sample_at = Some(oldest);
                self.last_joint_samples.fill(None);
                if track_joints {
                    for (slot, &t) in self.last_joint_samples.iter_mut().zip(samples.iter()) {
                        *slot = Some(t);
                    }
                    self.joint_count = samples.len();
                } else {
                    self.joint_count = 0;
                }
            }
        }
    }

    pub(crate) fn record_fresh_cool_at(&mut self, sample_at: Instant) {
        self.record_fresh_cool_samples(std::slice::from_ref(&sample_at));
    }
}

#[cfg(test)]
mod thermal_cooldown_tests {
    use super::{ThermalCooldown, THERMAL_FEEDBACK_FRESHNESS_LIMIT};
    use std::time::{Duration, Instant};

    /// A joint that resumes after a stale gap restarts the dwell from the newest sample: dwell is
    /// never credited from an older joint's timestamp (2026-11-24 review: J1 resumes at 300 ms,
    /// J2's last sample was at 240 ms, both sample at 330 ms -> dwell 30 ms, not 90 ms).
    #[test]
    fn staggered_gap_restarts_dwell_from_newest_sample() {
        let t0 = Instant::now();
        let ms = |v: u64| t0 + Duration::from_millis(v);
        let mut c = ThermalCooldown::default();
        c.record_fresh_cool_samples(&[ms(0), ms(0)]);
        c.record_fresh_cool_samples(&[ms(40), ms(240)]);
        assert!(Duration::from_millis(300 - 40) > THERMAL_FEEDBACK_FRESHNESS_LIMIT);
        c.record_fresh_cool_samples(&[ms(300), ms(240)]); // J1 gap 260 ms > 250 ms
        assert_eq!(c.elapsed_ms(), 0);
        c.record_fresh_cool_samples(&[ms(330), ms(330)]);
        assert_eq!(c.elapsed_ms(), 30);
    }
}

/// Summary of the release's Step-2 hardware poll passed into Step-5 `tick_control`
/// without allocating or re-reading the bus.
#[derive(Debug, Default)]
pub(crate) struct PendingRxPoll {
    pub active: bool,
    pub invalid_feedback: bool,
    pub driver_torque_lost: bool,
    pub bus_feedback_fresh: bool,
    pub recovery_ages: bool,
    pub hw_rx: bool,
    pub reset_stillness_on_read_err: bool,
    pub rx_frames: u16,
    pub budget_exhausted: bool,
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

/// Pre-captured heap context for `gripper_zero_check(store_offset=true)` persistence,
/// allocated once at action start so `finish_action` on `armd-control` performs zero heap
/// allocations and zero frees.
#[derive(Clone)]
pub struct ZeroCheckPersistContext {
    pub path: PathBuf,
    pub jaw_opening: Option<crate::model::JawOpening>,
    pub stale: Option<crate::calibration::CalibrationStale>,
    pub tool_id: String,
    pub tool_config_digest: String,
    pub active_tool_is_stock: bool,
    pub base_gripper_policy: Option<crate::model::GripperPolicy>,
    pub gripper_limit: Option<crate::model::JointLimit>,
    pub operator_auth: Option<(crate::auth::OperatorAuth, Option<String>)>,
    /// Pre-allocated scratch buffer reused on saturation failure without allocating on RT.
    pub sat_msg_buf: String,
}

impl ZeroCheckPersistContext {
    /// Captures all heap-backed calibration persistence inputs off the RT completion path.
    pub fn from_inner(
        inner: &SupervisorInner,
        operator_auth: Option<(crate::auth::OperatorAuth, Option<String>)>,
    ) -> Self {
        let path = inner
            .gripper_calib_path
            .clone()
            .unwrap_or_else(crate::gripper_calib::path_from_env);
        let gripper_limit = inner
            .config
            .gripper_index()
            .and_then(|idx| inner.config.limits.get(idx))
            .cloned();
        Self {
            path,
            jaw_opening: inner.gripper_calib_jaw_opening.clone(),
            stale: inner.gripper_calib_stale.clone(),
            tool_id: inner.config.active_tool.id.clone(),
            tool_config_digest: inner.config.active_tool.config_digest.clone(),
            active_tool_is_stock: inner.config.active_tool.is_stock,
            base_gripper_policy: inner
                .base_gripper_policy
                .clone()
                .or_else(|| inner.config.gripper_policy.clone()),
            gripper_limit,
            operator_auth,
            sat_msg_buf: String::with_capacity(64),
        }
    }

    /// Serializes `GripperCalibFile` off the control thread into a [`crate::persist::PendingWrite`].
    pub fn serialize_write(&self, offset_rad: f32) -> crate::persist::PendingWrite {
        let file = crate::gripper_calib::GripperCalibFile {
            offset_rad: Some(offset_rad),
            jaw_opening: self.jaw_opening.clone(),
            ts: Some(
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map_or(0, |d| d.as_secs()),
            ),
            stale: self.stale.clone(),
            tool_id: Some(self.tool_id.clone()),
            tool_config_digest: Some(self.tool_config_digest.clone()),
        };
        let text = serde_json::to_string_pretty(&file).unwrap_or_default();
        crate::persist::PendingWrite::new(self.path.clone(), text)
    }
}

/// Parameters of a running gripper zero check action.
#[derive(Clone)]
pub struct ZeroCheckParams {
    pub action_id: u64,
    pub store_offset: bool,
    pub threshold_rad: f32,
    pub base_closed_rad: f32,
    pub persist_ctx: Option<Box<ZeroCheckPersistContext>>,
}

/// Deferred atomic persist-before-commit job for `gripper_zero_check(store_offset=true)`.
#[derive(Clone)]
pub struct PendingZeroCheckPersist {
    pub action_id: u64,
    pub offset_rad: f32,
    pub base_revision: u64,
    pub lease_owner: Option<(crate::model::HolderLabel, u64)>,
    pub estop_epoch: u64,
    pub stop_gen: u64,
    pub deadline: Instant,
    pub ctx: Box<ZeroCheckPersistContext>,
}

/// Coalesced sorted inclusive intervals `(lo, hi)` of closed WS connection IDs (`>= 1`).
/// Bounded by `open_connections + 1` with zero eviction and O(log K) zero-alloc lookup.
#[derive(Debug, Default)]
pub(crate) struct ClosedConnections {
    intervals: Vec<(u64, u64)>,
}

impl ClosedConnections {
    pub(crate) fn mark_closed(&mut self, id: u64) {
        if id == 0 {
            return;
        }
        let idx = self
            .intervals
            .partition_point(|&(_, end)| end.saturating_add(1) < id);
        if idx < self.intervals.len() && self.intervals[idx].0 <= id.saturating_add(1) {
            let (lo, hi) = self.intervals[idx];
            let new_lo = lo.min(id);
            let mut new_hi = hi.max(id);
            if idx + 1 < self.intervals.len()
                && self.intervals[idx + 1].0 <= new_hi.saturating_add(1)
            {
                new_hi = new_hi.max(self.intervals[idx + 1].1);
                self.intervals.remove(idx + 1);
            }
            self.intervals[idx] = (new_lo, new_hi);
        } else {
            self.intervals.insert(idx, (id, id));
        }
    }

    pub(crate) fn is_closed(&self, id: u64) -> bool {
        if id == 0 {
            return false;
        }
        let idx = self.intervals.partition_point(|&(_, end)| end < id);
        idx < self.intervals.len() && self.intervals[idx].0 <= id
    }
}

/// Preallocated capacity for [`ConsumedStamps`] disjoint intervals above `watermark`.
const CONSUMED_STAMPS_INTERVAL_CAP: usize = 256;

/// Bounded, zero-allocation-in-steady-state set of consumed single-use admission request IDs
/// (`>= 1`), stored as a monotonic `watermark` (every `id <= watermark` is treated as consumed,
/// fail-closed) plus up to [`CONSUMED_STAMPS_INTERVAL_CAP`] sorted coalesced inclusive
/// intervals `(lo, hi)` above `watermark`.
#[derive(Debug)]
pub(crate) struct ConsumedStamps {
    watermark: u64,
    intervals: Vec<(u64, u64)>,
}

impl ConsumedStamps {
    pub(crate) fn new() -> Self {
        Self {
            watermark: 0,
            intervals: Vec::with_capacity(CONSUMED_STAMPS_INTERVAL_CAP),
        }
    }

    pub(crate) fn contains(&self, id: u64) -> bool {
        if id == 0 || id <= self.watermark {
            return true;
        }
        let idx = self.intervals.partition_point(|&(_, end)| end < id);
        idx < self.intervals.len() && self.intervals[idx].0 <= id
    }

    pub(crate) fn record(&mut self, id: u64) {
        if id == 0 || id <= self.watermark {
            return;
        }
        if id == self.watermark.saturating_add(1) {
            self.watermark = id;
            self.fold_contiguous_into_watermark();
            return;
        }
        let mut idx = self
            .intervals
            .partition_point(|&(_, end)| end.saturating_add(1) < id);
        if idx < self.intervals.len() && self.intervals[idx].0 <= id.saturating_add(1) {
            let (lo, hi) = self.intervals[idx];
            let new_lo = lo.min(id);
            let mut new_hi = hi.max(id);
            if idx + 1 < self.intervals.len()
                && self.intervals[idx + 1].0 <= new_hi.saturating_add(1)
            {
                new_hi = new_hi.max(self.intervals[idx + 1].1);
                self.intervals.remove(idx + 1);
            }
            self.intervals[idx] = (new_lo, new_hi);
            self.fold_contiguous_into_watermark();
            return;
        }
        // Capacity full: fold oldest range into `watermark` fail-closed (zero allocation,
        // never forgets any previously consumed ID).
        if self.intervals.len() >= CONSUMED_STAMPS_INTERVAL_CAP {
            if idx == 0 {
                self.watermark = self.watermark.max(id);
                self.fold_contiguous_into_watermark();
                return;
            }
            let (_, oldest_hi) = self.intervals.remove(0);
            self.watermark = self.watermark.max(oldest_hi);
            self.fold_contiguous_into_watermark();
            // Re-locate insertion point after folding into watermark.
            if id <= self.watermark {
                return;
            }
            if id == self.watermark.saturating_add(1) {
                self.watermark = id;
                self.fold_contiguous_into_watermark();
                return;
            }
            idx = self
                .intervals
                .partition_point(|&(_, end)| end.saturating_add(1) < id);
        }
        self.intervals.insert(idx, (id, id));
        self.fold_contiguous_into_watermark();
    }

    fn fold_contiguous_into_watermark(&mut self) {
        while let Some(&(lo, hi)) = self.intervals.first() {
            if lo <= self.watermark.saturating_add(1) {
                self.watermark = self.watermark.max(hi);
                self.intervals.remove(0);
            } else {
                break;
            }
        }
    }
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
        let unload_settle_cap = ((UNLOAD_SETTLE_HOLD.as_secs_f64() * f64::from(config.control_frequency_hz.max(1)))
            .ceil() as usize
            + 16)
            .max(64);

        let mut inner = Self {
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
            torque_transition: None,
            last_disable_attempt: None,
            output_suspended: false,
            feedback_numeric_valid: true,
            sim_time_s: 0.0,
            last_still_since_s: None,
            still_anchor_positions: Vec::with_capacity(dof),
            still_window_ever_moved: false,
            table_still_first_rx_instants: Vec::with_capacity(dof),
            table_still_last_rx_instants: Vec::with_capacity(dof),
            table_still_anchor_positions: Vec::with_capacity(dof),
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
            gripper_hold_vel_ceiling: None,
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
            gripper_stall_window: crate::motion::GripperProgressWindow::default(),
            gripper_stall_sign: 0.0,
            gripper_prev_hw_overload: false,
            gripper_force_capped: false,
            gripper_force_ticks: 0,
            gripper_force_trips: 0,
            gripper_motion_hold: None,
            gripper_stream_target: None,
            gripper_stream_contact: None,
            gripper_stream_contact_detector: crate::motion::GripperContactDetector::default(),
            gripper_stream_lease: None,
            gripper_ref_prev: false,
            gripper_ref_done: false,
            gripper_ref_holding: false,
            gripper_cap_ramp: None,
            gripper_over_cap_prev: false,
            park_gripper_contact: false,
            park_release: None,
            pending_normal_unload: false,
            unload_settle: None,
            watchdog_park: None,
            watchdog_park_blocked_reason: None,
            park_rejection: None,
            workspace_revision: 0,
            calibration: None,
            #[cfg(feature = "policy-runtime")]
            calibration_digest: crate::policy_runtime::EMPTY_CALIBRATION_DIGEST.to_string(),
            mit_gate_error: None,
            mit: Default::default(),
            profile_floor_z_m,
            floor_source: "profile",
            hover_jobs: Vec::new(),
            calibration_busy: false,
            motor_zero_busy: false,
            cached_driver_pos_limits: [None; crate::rt_core::mailbox::MAX_DOF],
            calibration_load_error: None,
            joint_zero_offsets: vec![0.0; dof],
            zero_convention: Default::default(),
            pending_legacy_j6_log: None,
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
            joint_feedback_queue_bounds: Vec::with_capacity(dof),
            scratch: crate::rt_core::scratch::TickScratch::with_dof(dof),
            ws_eval_bufs: Default::default(),
            bus_loss_instant: None,
            e_stop_latched: Arc::new(AtomicBool::new(false)),
            requested_estop_count: Arc::new(AtomicU64::new(0)),
            applied_estop_count: 0,
            estop_applied: Arc::new(AtomicU64::new(0)),
            stop_gen: Arc::new(AtomicU64::new(0)),
            lease_gen_mirror: Arc::new(AtomicU64::new(0)),
            last_preemption: None,
            last_evicted: None,
            handover_settling_since: None,
            gravity_comp: None,
            retired_gravity_comp: None,
            gravity_comp_last_exit: None,
            keepout: crate::keepout::Keepout::default(),
            poses: crate::poses::PoseStore::default(),
            residual_torques: None,
            collision: crate::collision::CollisionGuard::from_policy(&collision_policy),
            torque_guard: crate::torque_guard::TorqueGuard::default(),
            tracking: crate::tracking::TrackingRuntime::with_capacity(dof),
            power: crate::power::PowerMonitor::default(),
            follower: None,
            base_gripper_policy,
            gripper_calib_offset: None,
            gripper_calib_jaw_opening: None,
            gripper_calib_path: None,
            gripper_baseline: None,
            gripper_baseline_path: None,
            gripper_baseline_dirty: false,
            gripper_baseline_pending: Vec::with_capacity(GRIPPER_BASELINE_PENDING_MAX),
            gripper_baseline_generation: 0,
            gripper_calibration: None,
            gripper_last_contact: None,
            gripper_zero_checked: false,
            gripper_calib_invalid: false,
            gripper_calib_stale: None,
            last_zero_check_offset: None,
            mock_gripper_stall_offset,
            mock_gripper_stall_torque: false,
            mock_gripper_object: None,
            driver_config_error: None,
            retired_driver_config_error: None,
            config_reconcile_fault: None,
            thermal_cooldown_duration: Duration::ZERO,
            thermal_cooldown: ThermalCooldown::default(),
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
            retired_zero_check_ctx: std::collections::VecDeque::with_capacity(4),
            operator_auth_enforced: false,
            tool_commissioning_path: None,
            tool_commissioning_confirmed_at: None,
            tool_commissioning_revision: 1,
            pending_rx_poll: PendingRxPoll::default(),
            stop_requests: None,
            detached: false,
            persist_capacity_available: true,
            rt_executor_owned: false,
            shutdown_committed: false,
            shutdown_refused_unknown: false,
            pending_estop_acks: std::collections::VecDeque::with_capacity(
                crate::rt_core::executor::ESTOP_ACK_CAPACITY,
            ),
            estop_ack_overflow_gen: 0,
            pending_retired_stops: std::collections::VecDeque::with_capacity(
                crate::rt_core::urgent::URGENT_STOP_CAPACITY,
            ),
            closed_connections: ClosedConnections::default(),
            consumed_stamps: ConsumedStamps::new(),
            consumed_broker_stamps: ConsumedStamps::new(),
            terminals: None,
            safety_log_ring_dropped: 0,
            before_publish_hook: None,
            tx_joint_no_progress_ticks: vec![0; dof],
            tx_joint_no_progress_since: vec![None; dof],
            hold_tx_no_progress_ticks: vec![0; dof.saturating_mul(2)],
            hold_tx_no_progress_since: vec![None; dof.saturating_mul(2)],
            hold_tx_stalled_latched: false,
            last_hold_tx_stall_log: None,
            last_hold_tx_error_log: None,
            last_hold_tx_error_delivered: None,
            hold_tx_error_suppressed: 0,
            pending_hold_tx_error_retry: None,
            j7_stale_since: None,
            driver_clock: None,
            pending_deferred_done: None,
            gravity_comp_handback_ready: false,
            action_msg_spares: [
                Some(String::with_capacity(256)),
                Some(String::with_capacity(256)),
                None,
                None,
            ],
            retired_trajectories: std::collections::VecDeque::with_capacity(8),
            retired_macro_runs: std::collections::VecDeque::with_capacity(8),
            #[cfg(feature = "policy-runtime")]
            retired_policy_runs: std::collections::VecDeque::with_capacity(8),
            unload_settle_spare: Some(UnloadSettle::with_capacity(unload_settle_cap)),
            j7_intent: None,
            j7_admitted_stamp: None,
            post_stop_check_hook: None,
        };
        inner.prepare_workspace_eval_bufs();
        inner
    }

    /// Pre-sizes `ws_eval_bufs` (including overflow slot and capsule buffers for workspaces
    /// exceeding 256 slots) off the RT thread so RT checks allocate zero heap memory.
    pub fn prepare_workspace_eval_bufs(&mut self) {
        let mut bufs = std::mem::take(&mut self.ws_eval_bufs);
        if let Some((ws, cam)) = self.workspace_policy() {
            bufs.prepare_for(&self.config.arm_model, ws, cam, self.config.tcp_offset);
        }
        self.ws_eval_bufs = bufs;
    }

    /// Minimal empty shell left inside `ArmSupervisor::inner` once ownership transfers to
    /// the synchronous RT executor; all legacy lock paths reject it when `detached == true`.
    pub(crate) fn detached_shell(
        config: &ArmConfig,
        e_stop_latched: Arc<AtomicBool>,
        requested_estop_count: Arc<AtomicU64>,
        stop_gen: Arc<AtomicU64>,
        lease_gen_mirror: Arc<AtomicU64>,
    ) -> Self {
        let mut shell = Self::new(config.clone());
        shell.e_stop_latched = e_stop_latched;
        shell.requested_estop_count = requested_estop_count;
        shell.stop_gen = stop_gen;
        shell.lease_gen_mirror = lease_gen_mirror;
        shell.detached = true;
        shell
    }

    /// Current owner-serialized stop generation.
    pub fn current_stop_gen(&self) -> u64 {
        let local = self.stop_gen.load(Ordering::SeqCst);
        match self.stop_requests.as_ref() {
            Some(stops) => local.max(stops.current()),
            None => local,
        }
    }

    /// Current valid (unexpired) active lease generation, or `0` when no valid lease is held.
    pub fn current_active_lease_gen(&self) -> u64 {
        self.active_lease
            .as_ref()
            .filter(|a| a.acquired_at.elapsed() < self.lease_timeout)
            .map_or(0, |a| a.generation)
    }

    /// Capture an [`AdmissionStamp`] from the current owner state (for tests and in-process
    /// request entry before mutating/stopping; never called inside motion-start functions).
    pub fn current_admission_stamp(&self) -> AdmissionStamp {
        AdmissionStamp {
            stop_gen: self.current_stop_gen(),
            estop_epoch: self.safety.epoch_id(),
            lease_gen: self.current_active_lease_gen(),
            connection_id: 0,
            request_id: next_admission_request_id(),
        }
    }

    /// Marks WS connection `connection_id` (`>= 1`) closed under the owner lock so no
    /// resuming async install or stale [`Admitted`] token from that connection can start motion.
    pub fn mark_connection_closed(&mut self, connection_id: u64) {
        self.closed_connections.mark_closed(connection_id);
    }

    /// Returns true if WS connection `connection_id` (`>= 1`) has been marked closed under the owner lock.
    pub fn is_connection_closed(&self, connection_id: u64) -> bool {
        self.closed_connections.is_closed(connection_id)
    }

    /// Ensures every subscribed connection's bounded terminal FIFO has room for any action
    /// superseded by this start plus the new action's eventual terminal (never drops a terminal).
    pub(crate) fn check_terminal_fifo_for_new_action(&self) -> Result<(), SafetyError> {
        let Some(terminals) = self.terminals.as_ref() else {
            return Ok(());
        };
        let has_running = self.last_action.as_ref().is_some_and(|a| a.is_running());
        if !terminals.can_start_action(has_running, self.rt_executor_owned) {
            return Err(SafetyError::TerminalsBusy);
        }
        Ok(())
    }

    /// Records a finished action (`!is_running()`) into every subscribed connection's bounded
    /// terminal FIFO (deduped by action ID) and wakes `armd-dispatch` immediately. Skipped on
    /// `armd-control` (`rt_executor_owned == true`) so RT ticks stay lock-free/zero-alloc.
    pub(crate) fn record_finished_action(&self, action: &ActionStatus) {
        if self.rt_executor_owned || action.is_running() {
            return;
        }
        if let Some(terminals) = self.terminals.as_ref() {
            if terminals.record_finished(action) {
                self.core.hub().wake();
            }
        }
    }

    /// Convenience helper: if `self.last_action` matches `id` and is terminal, record it.
    pub(crate) fn record_finished_action_by_id(&self, id: u64) {
        if self.rt_executor_owned {
            return;
        }
        if let Some(a) = self.last_action.as_ref().filter(|a| a.id == id && !a.is_running()) {
            self.record_finished_action(a);
        }
    }

    /// Patches `failed_step` on a queued terminal action if still awaiting dispatcher drain.
    pub(crate) fn update_recorded_terminal_failed_step(&self, id: u64, step: Option<usize>) {
        if self.rt_executor_owned {
            return;
        }
        if let Some(terminals) = self.terminals.as_ref() {
            terminals.update_failed_step(id, step);
        }
    }

    /// Advance the stop generation under the owner lock whenever a Stop applies.
    pub fn advance_stop_generation(&mut self) -> u64 {
        self.reset_hold_tx_joint_progress();
        self.j7_admitted_stamp = None;
        self.stop_gen.fetch_add(1, Ordering::SeqCst) + 1
    }

    /// Monotonically synchronize `stop_gen` with an RT executor's `CommitCtx::stop_generation`.
    pub(crate) fn sync_stop_generation(&mut self, gen: u64) {
        let _ = self.stop_gen.fetch_max(gen, Ordering::SeqCst);
    }

    /// Keep `lease_gen_mirror` synchronized with `active_lease` under the owner lock.
    pub(crate) fn sync_lease_gen_mirror(&self) {
        self.lease_gen_mirror.store(self.current_active_lease_gen(), Ordering::SeqCst);
    }

    fn verify_admission_stamp(&self, stamp: AdmissionStamp) -> Result<(), SafetyError> {
        if stamp.connection_id != 0 && self.is_connection_closed(stamp.connection_id) {
            return Err(SafetyError::LeaseLost);
        }
        if stamp.stop_gen != self.current_stop_gen() {
            return Err(SafetyError::Stopped);
        }
        let active_gen = self.current_active_lease_gen();
        if stamp.lease_gen != active_gen {
            return Err(SafetyError::LeaseLost);
        }
        let current_epoch = self.safety.epoch_id();
        if stamp.estop_epoch != current_epoch {
            if self.e_stop_latched.load(Ordering::SeqCst)
                || matches!(self.safety.state(), SafetyState::EStop)
            {
                return Err(SafetyError::EStopActive);
            }
            return Err(SafetyError::EpochMismatch {
                expected: current_epoch,
                received: stamp.estop_epoch,
            });
        }
        if self.e_stop_latched.load(Ordering::SeqCst)
            || matches!(self.safety.state(), SafetyState::EStop)
        {
            return Err(SafetyError::EStopActive);
        }
        Ok(())
    }

    pub(crate) fn is_stamp_consumed(&self, request_id: u64) -> bool {
        if request_id == 0 {
            return true;
        }
        if (request_id & BROKER_REQUEST_ID_BIT) != 0 {
            self.consumed_broker_stamps
                .contains(request_id & !BROKER_REQUEST_ID_BIT)
        } else {
            self.consumed_stamps.contains(request_id)
        }
    }

    pub(crate) fn record_consumed_stamp(&mut self, request_id: u64) {
        if request_id == 0 {
            return;
        }
        if (request_id & BROKER_REQUEST_ID_BIT) != 0 {
            self.consumed_broker_stamps
                .record(request_id & !BROKER_REQUEST_ID_BIT);
        } else {
            self.consumed_stamps.record(request_id);
        }
    }

    /// Read-only preflight admission check used before off-lock planning or non-installing
    /// validation: verifies authority and confirms `stamp.request_id` is non-zero and not yet
    /// consumed, WITHOUT consuming it.
    pub fn preflight_admission(&self, stamp: AdmissionStamp) -> Result<(), SafetyError> {
        if self.is_stamp_consumed(stamp.request_id) {
            return Err(SafetyError::StampAlreadyConsumed);
        }
        self.verify_admission_stamp(stamp)
    }

    /// Single owner-lock motion/output admission gate: the SOLE constructor of [`Admitted`].
    /// Compares the caller's admission-captured [`AdmissionStamp`] against current owner state
    /// without re-sampling, consumes its single-use [`AdmissionStamp::request_id`] under the
    /// owner lock, and returns an [`Admitted<'g>`] token borrowing the owner guard.
    pub fn check_admission<'g>(
        &'g mut self,
        stamp: AdmissionStamp,
    ) -> Result<Admitted<'g>, SafetyError> {
        if self.is_stamp_consumed(stamp.request_id) {
            return Err(SafetyError::StampAlreadyConsumed);
        }
        self.verify_admission_stamp(stamp)?;
        self.record_consumed_stamp(stamp.request_id);
        Ok(Admitted {
            inner: self,
            stamp,
            _not_send_sync: std::marker::PhantomData,
        })
    }

    /// Unified motion-start wrapper: requires an [`Admitted`] token by value under the owner lock.
    pub fn start_motion<Op: MotionStartOp>(admitted: Admitted<'_>, op: Op) -> Result<Op::Output, ArmError> {
        admitted.verify_live()?;
        op.execute(admitted)
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
        schema.zero_convention = self.config.zero_reference.as_ref().and_then(|_| serde_json::to_value(self.zero_convention).ok());
        schema
    }

    /// The E-stop gate of this arm's bus driver: a request not yet applied by the tick
    /// (plus authorized-stop generations when attached to an RT executor).
    pub fn estop_gate(&self) -> crate::rt_core::urgent::EstopGate {
        let gate = crate::rt_core::urgent::EstopGate::new(self.requested_estop_count.clone(), self.estop_applied.clone());
        match self.stop_requests.as_ref() {
            Some(stops) => gate.with_stops(stops.clone()),
            None => gate,
        }
    }

    /// Attaches the executor's authorized-stop generation tracker and updates the bus
    /// driver's `EstopGate` so mid-batch writes also stop on authorized stop requests.
    pub fn attach_stop_requests(&mut self, stops: crate::rt_core::urgent::StopRequests) {
        self.stop_requests = Some(stops);
        let gate = self.estop_gate();
        if let Some(ref d) = self.driver {
            if let Ok(mut drv) = d.lock() {
                drv.set_estop_gate(gate);
            }
        }
    }

    /// An E-stop request the tick has not applied yet (not the latch: hold output while
    /// latched still reaches the motors).
    pub(crate) fn estop_request_unapplied(&self) -> bool {
        self.requested_estop_count.load(Ordering::SeqCst) != self.applied_estop_count
    }

    /// True when either an E-stop request or an attached authorized-stop request has not
    /// yet been applied by the control tick / release engine.
    pub(crate) fn urgent_stop_unapplied(&self) -> bool {
        self.estop_request_unapplied()
            || self.stop_requests.as_ref().is_some_and(crate::rt_core::urgent::StopRequests::pending)
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
        self.gripper_stall_window.reset();
        self.gripper_force_capped = false;
        self.gripper_force_ticks = 0;
        self.gripper_stream_target = None;
        self.gripper_stream_contact = None;
        self.gripper_stream_contact_detector.reset();
        self.gripper_stream_lease = None;
        self.gripper_cap_ramp = None;
        self.gripper_over_cap_prev = false;
    }

    /// End the gripper state of a servo stream or live policy (target, contact regulator) on
    /// stop, E-stop, fault and lease loss: nothing may advance J7 for that stream afterwards.
    /// While that state was active and torque is on, J7 holds the measured jaw.
    pub fn end_gripper_stream(&mut self) {
        let active = self.gripper_stream_target.is_some() || self.gripper_stream_contact.is_some();
        self.gripper_stream_target = None;
        self.gripper_stream_contact = None;
        self.gripper_stream_contact_detector.reset();
        self.gripper_stream_lease = None;
        if !active || !self.is_torqued { return; }
        let Some(g) = self.config.gripper_index() else { return };
        let Some(&m) = self.measured_positions.get(g) else { return };
        if !m.is_finite() { return; }
        let m = self.config.limits.get(g).map_or(m, |l| m.clamp(l.min_position, l.max_position));
        if g < self.commanded_positions.len() { self.commanded_positions[g] = m; }
        if g < self.commanded_velocities.len() { self.commanded_velocities[g] = 0.0; }
        if g < self.target_positions.len() { self.target_positions[g] = m; }
        if let Some(st) = self.servo_target.as_mut() { if st.len() > g { st[g] = m; } }
        if let Some(h) = self.gripper_motion_hold.as_mut() { h.0 = m; }
    }

    /// A gripper action (or macro gripper step) ends without finishing (stop, E-stop, fault,
    /// lease loss, cancel): J7 command and target go to the measured jaw (clamped to the limits),
    /// so a command that led the jaw does not keep compressing after the stop.
    pub fn hold_gripper_at_measured(&mut self) {
        if !self.is_torqued { return; }
        let Some(g) = self.config.gripper_index() else { return };
        let Some(&m) = self.measured_positions.get(g) else { return };
        if !m.is_finite() { return; }
        let m = self.config.limits.get(g).map_or(m, |l| m.clamp(l.min_position, l.max_position));
        if g < self.commanded_positions.len() { self.commanded_positions[g] = m; }
        if g < self.commanded_velocities.len() { self.commanded_velocities[g] = 0.0; }
        if g < self.target_positions.len() { self.target_positions[g] = m; }
        if let Some(st) = self.servo_target.as_mut() { if st.len() > g { st[g] = m; } }
        if let Some(h) = self.gripper_motion_hold.as_mut() { h.0 = m; }
    }

    fn gripper_reference_active(&self) -> bool {
        matches!(self.active_trajectory.as_ref().map(|a| &a.reference), Some(Reference::Gripper(_)))
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
            let expires = lease.acquired_at + self.lease_timeout;
            if expires > sampled {
                until = until.min(expires);
            }
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

    pub(crate) fn begin_tick_phases(&mut self, start: Instant) {
        self.tick_phases_us = [0; 5];
        self.tick_phase_mark = Some(start);
    }

    pub(crate) fn take_tick_phases(&mut self) -> ([u32; 5], Option<Instant>) {
        (self.tick_phases_us, self.tick_phase_mark.take())
    }

    /// Publishes an observation filled in place at `timestamp` with lock/release generation
    /// `generation` without re-running `tick_control`.
    pub fn publish_at(&mut self, timestamp: f64, generation: u64, keep: bool) -> TickPublished {
        let mut obs = self.core.take_observation_buffer();
        {
            let (obs_ref, spares) = obs.split_mut();
            self.fill_observation_with_spare(timestamp, obs_ref, Some(spares));
        }
        self.tick_phase(4);
        let body_end = self.tick_phase_mark.take();
        let control_hz = self.config.control_frequency_hz;
        let (sampled_at, valid_until) = self.observation_valid_until(&obs, Duration::from_secs_f64(2.0 / f64::from(control_hz.max(1))));
        let kept = keep.then(|| (*obs).clone());
        if let Some(ref hook) = self.before_publish_hook {
            hook();
        }
        let SupervisorInner { core, measured_positions, measured_velocities, .. } = self;
        core.publish(obs, generation, sampled_at, valid_until, measured_positions, measured_velocities);
        TickPublished { kept, body_end }
    }

    /// The control loop's tick under the write guard of lock generation `generation`:
    /// [`Self::tick_control`], the observation filled in place into the publication's back
    /// slot ([`Self::fill_observation`]), and its publication (before the guard drops: no
    /// other write can come in between). No allocation in a steady-state tick; `keep` copies
    /// the observation for the caller (tests).
    pub fn tick_publish(&mut self, dt: f64, generation: u64, keep: bool) -> TickPublished {
        self.tick_publish_with_deadline(dt, generation, keep, None)
    }

    pub(crate) fn tick_publish_with_deadline(
        &mut self,
        dt: f64,
        generation: u64,
        keep: bool,
        release_bound: Option<(Instant, Instant)>,
    ) -> TickPublished {
        let timestamp = self.tick_control_inner(dt, release_bound);
        self.publish_at(timestamp, generation, keep)
    }

    pub fn current_observation(&self) -> ArmObservation {
        let mut obs = ArmObservation::blank();
        self.fill_observation(self.start_time.elapsed().as_secs_f64(), &mut obs);
        obs
    }

    /// Pre-warms RT-owned scratch/history buffers, static fallback policies, and empty
    /// triple-buffer observation slots for the RT executor before the first release runs
    /// (nonblocking under reader contention; leaves `published_seq` unchanged).
    pub(crate) fn prewarm_observation_slots(&mut self) {
        let _ = Self::default_rebot_dm_resting_pose();
        let dof = self.config.dof;
        let arm_dof = self.config.arm_dof();
        self.still_anchor_positions.reserve(dof);
        self.table_still_anchor_positions.reserve(dof);
        self.table_still_first_rx_instants.reserve(dof);
        self.table_still_last_rx_instants.reserve(dof);
        self.joint_feedback_queue_bounds.reserve(dof);
        self.motor_errors.reserve(dof);
        self.gravity_torques.reserve(arm_dof);
        let tmpl = self.current_observation();
        self.core.try_prewarm_observations(|| {
            let mut obs = tmpl.clone();
            obs.ensure_rt_safety_text_spare();
            obs.gravity_torques.reserve(arm_dof);
            obs.motor_errors.reserve(dof);
            obs
        });
    }

    /// [`Self::current_observation`] at `timestamp` written into `out`, reusing its memory: the
    /// control tick fills its published sample this way, without allocating while no value
    /// changes its shape (armd 1.6 RT core). Every field is overwritten.
    pub fn fill_observation(&self, timestamp: f64, out: &mut ArmObservation) {
        self.fill_observation_with_spare(timestamp, out, None);
    }

    fn fill_observation_with_spare(
        &self,
        timestamp: f64,
        out: &mut ArmObservation,
        spares: Option<&mut crate::rt_core::publish::ObservationSpares>,
    ) {
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
                    baseline: Some(self.gripper_baseline_observation()),
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
        // `fault` reflects the most severe recorded kind and worst-of recoverability
        // (`FaultInfo` is stack-allocated `Copy`, 0 heap ops).
        let state = self.safety.state();
        if state.is_stopped() || self.safety.has_latched_fault() || out.fault.is_some() {
            out.fault = self.safety.fault();
        }
        out.set_safety_state(state);
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
        let (action_spare, prev_action_spare) = match spares {
            Some(s) => (Some(&mut s.action_msg), Some(&mut s.prev_action_msg)),
            None => (None, None),
        };
        sync_action_obs(&mut out.action, self.last_action.as_ref(), action_spare);
        let prev_src = self
            .previous_action
            .as_ref()
            .filter(|(_, until)| timestamp < *until)
            .map(|(a, _)| a);
        sync_action_obs(&mut out.previous_action, prev_src, prev_action_spare);
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
        out.watchdog_park_blocked_reason = self.watchdog_park_blocked_reason;
        out.safety_log_dropped = self.safety.log_dropped().saturating_add(self.safety_log_ring_dropped);
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

    fn default_rebot_dm_resting_pose() -> &'static RestingPosePolicy {
        static DEFAULT_REBOT_DM: std::sync::OnceLock<RestingPosePolicy> = std::sync::OnceLock::new();
        DEFAULT_REBOT_DM.get_or_init(RestingPosePolicy::rebot_dm)
    }

    fn resting_policy(&self) -> Option<&RestingPosePolicy> {
        match &self.config.resting_pose {
            Some(policy) => Some(policy),
            None if self.driver.is_none() => Some(Self::default_rebot_dm_resting_pose()),
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
        if failures.is_empty() {
            None
        } else if failures.len() > crate::safety::MAX_RESTING_SUPPORT_ITEMS {
            let overflowed = failures.len() - crate::safety::MAX_RESTING_SUPPORT_ITEMS;
            let mut joined = failures[..crate::safety::MAX_RESTING_SUPPORT_ITEMS].join("; ");
            let _ = core::fmt::write(&mut joined, format_args!(" (+{overflowed} more)"));
            Some(joined)
        } else {
            Some(failures.join("; "))
        }
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
            for sample in &settle.samples {
                let (q, v) = if i < sample.dof {
                    (sample.positions[i], sample.velocities[i])
                } else {
                    (f32::NAN, f32::NAN)
                };
                if !q.is_finite() || !v.is_finite() { return Some(format!("joint {} non-finite feedback", i + 1)); }
                lo = lo.min(q); hi = hi.max(q); sum += v; peak = peak.max(v.abs());
                if sample.t >= recent_from { recent_sum += v; recent_n += 1; }
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
            let first_gen = self.applied_estop_count + 1;
            self.reset_hold_tx_joint_progress();
            self.e_stop_latched.store(true, Ordering::SeqCst);
            if matches!(self.torque_transition, Some(driver_transitions::TorqueTransition::Enable { .. })) {
                if let Some(ref d) = self.driver {
                    if let Ok(mut drv) = d.lock() {
                        drv.abort_enable();
                    }
                }
            }
            self.apply_stop_policy(StopPolicy::EStopLatch);
            self.applied_estop_count = req;
            self.estop_applied.store(req, Ordering::SeqCst);
            if self.pending_estop_acks.len() < crate::rt_core::executor::ESTOP_ACK_CAPACITY {
                self.pending_estop_acks.push_back(crate::rt_core::executor::EstopAck {
                    first_gen,
                    last_gen: req,
                    state: self.safety.state_arc(),
                    epoch: self.safety.epoch_id(),
                });
            } else {
                self.estop_ack_overflow_gen = self.estop_ack_overflow_gen.max(req);
            }
            true
        } else {
            false
        }
    }

    /// Pure, allocation-free evaluation of the next [`ThermalCooldown`] state from fresh driver
    /// feedback timestamps without mutating `self`. Any gap between consecutive fresh samples
    /// exceeding [`THERMAL_FEEDBACK_FRESHNESS_LIMIT`] restarts the dwell from the new sample.
    pub(crate) fn evaluate_thermal_cooldown_with_driver(
        &self,
        drv: &dyn ArmDriver,
        ages: &[Option<Duration>],
        thermal_below: bool,
        rx_scratch: &mut Vec<Option<Instant>>,
    ) -> ThermalCooldown {
        if !thermal_below {
            return ThermalCooldown::default();
        }
        let dof = self.config.dof;
        if dof == 0 {
            return ThermalCooldown::default();
        }
        let mut next = self.thermal_cooldown;
        drv.feedback_receive_instants_into(rx_scratch);
        if rx_scratch.len() != dof || rx_scratch.iter().any(Option::is_none) {
            drv.feedback_instants_into(rx_scratch);
        }
        if rx_scratch.len() == dof && rx_scratch.iter().all(Option::is_some) {
            if dof <= crate::rt_core::mailbox::MAX_DOF {
                let mut buf = [rx_scratch[0].unwrap(); crate::rt_core::mailbox::MAX_DOF];
                for (dst, src) in buf.iter_mut().take(dof).zip(rx_scratch.iter()) {
                    *dst = src.unwrap();
                }
                next.record_fresh_cool_samples(&buf[..dof]);
            } else if let Some(oldest) = rx_scratch.iter().filter_map(|t| *t).min() {
                next.record_fresh_cool_at(oldest);
            } else {
                next.reset();
            }
            return next;
        }
        if ages.len() == dof && ages.iter().all(Option::is_some) {
            let drv_now = drv.clock_now();
            if dof <= crate::rt_core::mailbox::MAX_DOF {
                let mut buf = [drv_now; crate::rt_core::mailbox::MAX_DOF];
                for (dst, age) in buf.iter_mut().take(dof).zip(ages.iter()) {
                    let a = age.unwrap();
                    *dst = drv_now.checked_sub(a).unwrap_or(drv_now);
                }
                next.record_fresh_cool_samples(&buf[..dof]);
            } else if let Some(oldest_age) = ages.iter().copied().try_fold(Duration::ZERO, |m, a| a.map(|t| m.max(t))) {
                let oldest = drv_now.checked_sub(oldest_age).unwrap_or(drv_now);
                next.record_fresh_cool_at(oldest);
            } else {
                next.reset();
            }
            return next;
        }
        ThermalCooldown::default()
    }

    /// Evaluates continuous thermal cooldown tracking from driverless simulation temperatures at
    /// `now` without mutating `self.thermal_cooldown`.
    pub(crate) fn evaluate_thermal_cooldown_driverless(&self, now: Instant) -> ThermalCooldown {
        let mut next = self.thermal_cooldown;
        let threshold = self.config.max_temperature - THERMAL_CLEAR_MARGIN_C;
        if !self.temperatures.is_empty()
            && self.temperatures.iter().all(|t| t.is_finite() && *t <= threshold)
        {
            next.record_fresh_cool_at(now);
        } else {
            next.reset();
        }
        next
    }

    /// Updates continuous thermal cooldown tracking from driverless simulation temperatures at `now`.
    pub(crate) fn update_thermal_cooldown_driverless(&mut self, now: Instant) {
        self.thermal_cooldown = self.evaluate_thermal_cooldown_driverless(now);
    }

    /// Returns the first measured joint violating configured limits plus
    /// [`JOINT_LIMIT_CLEAR_MARGIN_RAD`] (zero heap allocations), or `None` when every joint is
    /// finite and within clear bounds.
    pub(crate) fn joint_limit_clear_violation(&self) -> Option<crate::safety::JointLimitClearViolation> {
        use crate::safety::{BoundedText, JointLimitClearViolation as V};
        let dof = self.config.dof;
        if self.measured_positions.len() != dof || self.config.limits.len() != dof {
            return Some(V::FeedbackIncomplete {
                measured_len: self.measured_positions.len(),
                dof,
            });
        }
        for (i, (&q, l)) in self.measured_positions.iter().zip(&self.config.limits).enumerate() {
            if !q.is_finite() {
                return Some(V::NonFinite {
                    joint_idx: i as u16,
                    joint_name: BoundedText::new(&l.name),
                    value: q,
                });
            }
            if q < l.min_position - JOINT_LIMIT_CLEAR_MARGIN_RAD
                || q > l.max_position + JOINT_LIMIT_CLEAR_MARGIN_RAD
            {
                return Some(V::OutOfBounds {
                    joint_idx: i as u16,
                    joint_name: BoundedText::new(&l.name),
                    value: q,
                    min: l.min_position,
                    max: l.max_position,
                    margin: JOINT_LIMIT_CLEAR_MARGIN_RAD,
                });
            }
        }
        None
    }

    /// True when every measured joint position is finite and within configured limits plus
    /// [`JOINT_LIMIT_CLEAR_MARGIN_RAD`].
    #[allow(dead_code)]
    pub(crate) fn joints_within_clear_limits(&self) -> bool {
        self.joint_limit_clear_violation().is_none()
    }

    /// Thermal protection remains active while torqued or while torque disable is unconfirmed.
    /// `was_active` preserves thermal protection when a torque transition confirmed disable on
    /// the exact same Step 2 poll that delivered an over-temperature sample.
    ///
    /// Thermal cooldown accrues ONLY across consecutive fresh temperature samples; stale,
    /// unavailable, or gapped (> [`THERMAL_FEEDBACK_FRESHNESS_LIMIT`]) feedback resets or
    /// restarts the cooldown dwell.
    pub(crate) fn check_thermal_protection(&mut self, was_active: bool) {
        let temperature_available = match self.driver.as_ref() {
            None => {
                self.update_thermal_cooldown_driverless(Instant::now());
                true
            }
            Some(d) => {
                let mut scratch_ages = std::mem::take(&mut self.scratch.ages);
                let mut scratch_rx = std::mem::take(&mut self.scratch.rx);
                let (avail, next_cd) = match d.lock() {
                    Ok(drv) => {
                        let avail = drv.capabilities().temperature_feedback;
                        drv.feedback_ages_into(&mut scratch_ages);
                        let fresh = avail
                            && self.feedback_numeric_valid
                            && drv.is_feedback_valid()
                            && scratch_ages.len() == self.config.dof
                            && scratch_ages
                                .iter()
                                .all(|a| a.is_some_and(|t| t <= THERMAL_FEEDBACK_FRESHNESS_LIMIT));
                        let threshold = self.config.max_temperature - THERMAL_CLEAR_MARGIN_C;
                        let thermal_below = fresh
                            && !self.temperatures.is_empty()
                            && self.temperatures.iter().all(|t| t.is_finite() && *t <= threshold);
                        let cd = self.evaluate_thermal_cooldown_with_driver(
                            drv.as_ref(),
                            &scratch_ages,
                            thermal_below,
                            &mut scratch_rx,
                        );
                        (avail, cd)
                    }
                    Err(_) => (false, ThermalCooldown::default()),
                };
                self.scratch.ages = scratch_ages;
                self.scratch.rx = scratch_rx;
                self.thermal_cooldown = next_cd;
                avail
            }
        };
        if (self.is_torqued || self.disable_pending || was_active)
            && temperature_available
            && !self.safety.has_latched_fault_kind(crate::safety::FaultKind::ThermalTrip)
            && self
                .temperatures
                .iter()
                .any(|t| t.is_finite() && *t > self.config.max_temperature)
        {
            self.apply_stop_policy(StopPolicy::ThermalTrip);
        }
    }

    /// Consumes confirmation of a non-broker background disable once fresh valid hardware
    /// feedback reports every motor disabled. Returns `true` when no background disable
    /// remains pending.
    pub(crate) fn confirm_background_disable_if_ready(&mut self) -> bool {
        if !self.disable_pending || self.torque_transition.is_some() {
            return true;
        }
        let confirmed = self.driver.as_ref().is_none_or(|d| {
            d.lock()
                .is_ok_and(|drv| drv.is_feedback_valid() && drv.enable_state() == EnableState::Disabled)
        });
        if confirmed {
            self.disable_confirmed();
        }
        confirmed
    }

    pub fn shutdown_ready(&self) -> bool {
        if self.disable_pending || self.gravity_comp.is_some() || self.torque_transition_pending() {
            return false;
        }
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

    fn driver_enable_state_unknown(&self) -> bool {
        self.driver.as_ref().is_some_and(|d| {
            d.lock().is_ok_and(|drv| {
                drv.capabilities().enable_feedback && drv.enable_state() == EnableState::Unknown
            })
        })
    }

    /// Authoritative shutdown gate shared by legacy lock and RT executor ownership paths.
    /// Returns `true` when safe to stop the control loop and exit; returns `false` after
    /// latching stop when hardware disable is not yet confirmed. When shutdown was refused
    /// because the hardware enable state is `Unknown`, a repeated shutdown signal issues a
    /// bounded disable request rather than only re-checking.
    pub fn request_shutdown(&mut self) -> bool {
        if self.driver.is_none() || self.shutdown_ready() {
            self.shutdown_refused_unknown = false;
            tracing::info!("shutdown: latching EStop so the control loop stops before exit (in memory only, not persisted; the next start is Idle)");
            self.e_stop_latched.store(true, Ordering::SeqCst);
            return true;
        }
        if !self.safety.state().is_stopped() {
            self.execute_stop(StopPolicy::EStopLatch);
        }
        let unknown = self.driver_enable_state_unknown();
        if unknown && self.shutdown_refused_unknown && (!self.is_torqued || self.disable_pending) {
            if self.rt_executor_owned {
                if self.torque_transition.is_none() {
                    let _ = self.request_disable();
                }
            } else {
                let outcome = self.attempt_panic_disable(PANIC_DISABLE_TIMEOUT);
                outcome.log("Repeated shutdown disable while state Unknown", PANIC_DISABLE_TIMEOUT);
            }
            if self.shutdown_ready() {
                self.shutdown_refused_unknown = false;
                tracing::info!("shutdown: latching EStop so the control loop stops before exit (in memory only, not persisted; the next start is Idle)");
                self.e_stop_latched.store(true, Ordering::SeqCst);
                return true;
            }
        }
        self.shutdown_refused_unknown = unknown;
        false
    }

    /// Final shutdown admission executed under the owner's exclusive authority.
    /// Returns `true` and irreversibly sets `shutdown_committed` + `e_stop_latched` when
    /// shutdown is safe so no concurrent `clear_fault`, torque enable, or motion command can
    /// re-energize the arm before process exit; hardware disable paths remain permitted.
    pub fn commit_shutdown(&mut self) -> bool {
        let ready = self.driver.is_none()
            || (self.shutdown_ready()
                && !self.is_torqued
                && !self.disable_pending
                && !self.torque_transition_pending());
        if !ready {
            return false;
        }
        self.shutdown_committed = true;
        self.e_stop_latched.store(true, Ordering::SeqCst);
        self.stop_motion();
        true
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

    pub(crate) fn now_s(&self) -> f64 { self.start_time.elapsed().as_secs_f64() }

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
    ///
    /// Exempt from `Admitted` (Invariant 1a): system-initiated stop-direction motion
    /// completing a `NormalUnload` park so the gripper releases safely before torque-off.
    fn step_park_release(&mut self) -> bool {
        if self.is_pos_force_j7_stale() {
            return false;
        }
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
        let close_cap = self.gripper_close_speed_limit();
        let ceiling_est = self.gripper_ceiling_torque_estimate();
        let motion = match GripperMotion::try_with_close_speed_limit(
            request, policy, self.config.limits[gripper_idx].clone(),
            self.commanded_positions[gripper_idx], gripper_idx, close_cap,
        ) {
            Ok(m) => m.with_ceiling_torque_estimate(ceiling_est),
            Err(e) => {
                tracing::error!("Park release skipped: {e}");
                return false;
            }
        };
        // the stall latch of the carry would block the opening move's first ticks otherwise
        self.gripper_stalled = false;
        self.gripper_high_torque_ticks = 0;
        self.gripper_low_torque_ticks = 0;
        self.gripper_stall_window.reset();
        self.gripper_motion_hold = None;
        if next == ParkRelease::Opening { self.releasing = true; }
        let duration = motion.expected_duration();
        tracing::info!("Park release: {} joint {} {:.3} -> {:.3} rad", if next == ParkRelease::Opening { "opening" } else { "closing" },
            gripper_idx + 1, self.commanded_positions[gripper_idx], motion.target);
        if let Some(park) = self.watchdog_park.as_mut() { park.budget_s += motion.timeout_s.max(duration); }
        self.gripper_hold_vel_ceiling = Some(motion.wire_velocity_ceiling());
        self.target_positions[gripper_idx] = motion.target;
        if self.watchdog_park.is_some() {
            // Lease-watchdog park owns gripper release independent of client stops; stamp from
            // local owner state (never `max(local, requested)`).
            self.j7_admitted_stamp = Some(crate::driver::PosForceIntentStamp {
                stop_gen: self.stop_gen.load(Ordering::SeqCst),
                estop_epoch: self.safety.epoch_id(),
            });
        }
        self.active_trajectory = Some(ActiveTrajectory::new(None, Reference::Gripper(Box::new(motion))));
        self.park_release = Some(next);
        true
    }

    /// Queue a replaced `ActiveTrajectory` for off-thread drop when owned by the RT executor
    /// (prevents dropping heap-backed `motion_limits` / path data on `armd-control`).
    pub(crate) fn retire_active_trajectory(&mut self, traj: ActiveTrajectory) {
        if self.rt_executor_owned
            && self.retired_trajectories.len() < self.retired_trajectories.capacity()
        {
            self.retired_trajectories.push_back(traj);
        }
    }

    /// Pop one retired `ActiveTrajectory` for off-thread drop (0 alloc, 0 drop).
    pub fn pop_retired_trajectory(&mut self) -> Option<ActiveTrajectory> {
        self.retired_trajectories.pop_front()
    }

    /// True when the supervisor has guaranteed off-thread retirement capacity for a new `MacroRun`
    /// so retiring it on `armd-control` can never fall back to freeing heap strings on the RT
    /// thread. Always `true` in Legacy mode (off-RT).
    pub fn can_admit_macro_run(&self) -> bool {
        !self.rt_executor_owned
            || self.retired_macro_runs.len() + usize::from(self.macro_run.is_some())
                < self.retired_macro_runs.capacity()
    }

    /// Queue a replaced `MacroRun` for off-thread drop when owned by the RT executor
    /// (prevents dropping heap-backed `phase` / `SubState::Failed` strings on `armd-control`).
    pub(crate) fn retire_macro_run(&mut self, run: crate::macro_action::MacroRun) {
        if self.rt_executor_owned
            && self.retired_macro_runs.len() < self.retired_macro_runs.capacity()
        {
            self.retired_macro_runs.push_back(run);
        }
    }

    /// Number of retired `MacroRun` instances currently queued for off-thread drop.
    pub fn retired_macro_runs_len(&self) -> usize {
        self.retired_macro_runs.len()
    }

    /// Pop one retired `MacroRun` for off-thread drop (0 alloc, 0 drop).
    pub fn pop_retired_macro_run(&mut self) -> Option<crate::macro_action::MacroRun> {
        self.retired_macro_runs.pop_front()
    }

    /// True when the supervisor has guaranteed off-thread retirement capacity for a new
    /// `PolicyRun` so ending it on `armd-control` never frees heap fields on the RT thread.
    #[cfg(feature = "policy-runtime")]
    pub fn can_admit_policy_run(&self) -> bool {
        !self.rt_executor_owned
            || self.retired_policy_runs.len() + usize::from(self.policy_run.is_some())
                < self.retired_policy_runs.capacity()
    }

    /// Queue a finished `PolicyRun` for off-thread drop when owned by the RT executor
    /// (prevents dropping heap strings/vectors on `armd-control`).
    #[cfg(feature = "policy-runtime")]
    pub(crate) fn retire_policy_run(&mut self, run: crate::policy_runtime::PolicyRun) {
        if self.rt_executor_owned
            && self.retired_policy_runs.len() < self.retired_policy_runs.capacity()
        {
            self.retired_policy_runs.push_back(run);
        }
    }

    /// Number of retired `PolicyRun` instances currently queued for off-thread drop.
    #[cfg(feature = "policy-runtime")]
    pub fn retired_policy_runs_len(&self) -> usize {
        self.retired_policy_runs.len()
    }

    /// Pop one retired `PolicyRun` for off-thread drop (0 alloc, 0 drop).
    #[cfg(feature = "policy-runtime")]
    pub fn pop_retired_policy_run(&mut self) -> Option<crate::policy_runtime::PolicyRun> {
        self.retired_policy_runs.pop_front()
    }

    /// Current exact count of suppressed `"Stopped hold output failed"` log events pending the
    /// next 1/s log window (or deferred-ring retry).
    pub fn hold_tx_error_suppressed(&self) -> u64 {
        self.hold_tx_error_suppressed
    }

    /// Per-`(joint, slot)` (`index = joint * 2 + slot`) consecutive stopped-hold ticks without
    /// reaching the transport.
    pub fn hold_tx_no_progress_ticks(&self) -> &[u32] {
        &self.hold_tx_no_progress_ticks
    }

    /// Return an `UnloadSettle` buffer to `unload_settle_spare` (resetting its logical state
    /// without freeing its preallocated sample ring; any overflow is retired off-thread on RT).
    pub(crate) fn clear_unload_settle(&mut self) {
        if let Some(mut settle) = self.unload_settle.take() {
            settle.pending_s = 0.0;
            settle.held_s = 0.0;
            settle.held_since_s = None;
            settle.samples.clear();
            settle.last_report_s = None;
            if self.unload_settle_spare.is_none() {
                self.unload_settle_spare = Some(settle);
            } else {
                self.push_retired_stop(crate::rt_core::urgent::RetiredStopPayloads {
                    unload_settle: Some(settle),
                    ..Default::default()
                });
            }
        }
    }

    /// One control tick of the unload settle: the static resting-pose conditions must hold
    /// continuously for [`UNLOAD_SETTLE_HOLD`] and the arm must be stationary over that
    /// window ([`Self::unload_still_violation`]); then torque is cut. A settle pending past
    /// the hold logs which condition blocks it, at most once per second.
    fn step_unload_settle(&mut self, dt: f64) {
        let cap = ((UNLOAD_SETTLE_HOLD.as_secs_f64() * f64::from(self.config.control_frequency_hz.max(1)))
            .ceil() as usize
            + 16)
            .max(64);
        let mut settle = self
            .unload_settle
            .take()
            .or_else(|| self.unload_settle_spare.take())
            .unwrap_or_else(|| UnloadSettle::with_capacity(cap));
        settle.pending_s += dt;
        let static_violation = self.resting_pose_static_violation();
        if static_violation.is_none() {
            let since = *settle.held_since_s.get_or_insert(settle.pending_s);
            settle.held_s = settle.pending_s - since;
            let oldest = settle.pending_s - UNLOAD_SETTLE_HOLD.as_secs_f64() - 1e-6;
            while settle.samples.front().is_some_and(|s| s.t < oldest) { settle.samples.pop_front(); }
            if settle.samples.capacity() > 0 && settle.samples.len() == settle.samples.capacity() {
                settle.samples.pop_front();
            }
            settle.samples.push_back(UnloadSettleSample::from_slices(
                settle.pending_s,
                &self.measured_positions,
                &self.measured_velocities,
            ));
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
        // Not yet stably at rest: hold the home target in place (0 alloc). Gripper keeps its
        // command: driving it to the home value would squeeze anything still in the jaw.
        let g_idx = self.config.gripper_index();
        let saved_gripper = g_idx.and_then(|g| self.commanded_positions.get(g).copied());
        copy_into(&mut self.commanded_positions, self.config.resting_positions());
        if let (Some(g), Some(pos)) = (g_idx, saved_gripper) {
            if let Some(slot) = self.commanded_positions.get_mut(g) {
                *slot = pos;
            }
        }
        copy_into(&mut self.target_positions, &self.commanded_positions);
        self.commanded_velocities.fill(0.0);
        self.unload_settle = Some(settle);
    }

    /// Blocker of a pending unload settle, for watchdog reporting.
    fn unload_settle_blocker(&self) -> Option<String> {
        if self.active_trajectory.is_some() { return Some("park trajectory still running".into()); }
        self.resting_pose_static_violation()
            .or_else(|| self.unload_settle.as_ref().and_then(|s| self.unload_still_violation(s)))
    }

    /// Whether stopped-hold TX stalling is allowed to install `CAN_TX_STALLED`.
    /// Never overwrites `EStop`, `ThermalTrip`, `MitOwner::Lost`, an already-latched
    /// `CanTxStalled`, or a saturated off-thread reclamation queue.
    fn can_escalate_to_can_tx_stalled(&self) -> bool {
        if self.hold_tx_stalled_latched || self.e_stop_latched.load(Ordering::SeqCst) {
            return false;
        }
        if self.safety.has_latched_fault_kind(crate::safety::FaultKind::Estop)
            || self.safety.has_latched_fault_kind(crate::safety::FaultKind::ThermalTrip)
            || self.safety.has_latched_fault_kind(crate::safety::FaultKind::CanTxStalled)
        {
            return false;
        }
        if matches!(
            self.safety.state(),
            SafetyState::EStop
                | SafetyState::Fault(FaultReason::ThermalTrip { .. } | FaultReason::CanTxStalled)
        ) {
            return false;
        }
        if self.mit.owner == crate::mit::MitOwner::Lost {
            return false;
        }
        self.safety.can_accept_fault_transition()
    }

    /// Attach a driver, bind tick-epilogue ownership of the single J7 `PosForce` emitter, and
    /// cache its lock-free clock handle (if provided).
    pub(crate) fn install_driver(&mut self, mut driver: Box<dyn ArmDriver>) {
        driver.set_external_pos_force_emitter(true);
        self.driver_clock = driver.driver_clock();
        self.driver = Some(std::sync::Mutex::new(driver));
    }

    /// Lock-free read of the driver clock for RT off-thread forwarding timestamps, falling back
    /// to a non-blocking driver try-lock or `fallback` when no lock-free driver clock is attached.
    pub(crate) fn driver_clock_now_or(&self, fallback: Instant) -> Instant {
        self.driver_clock
            .as_ref()
            .map(|c| c.now())
            .or_else(|| {
                self.driver
                    .as_ref()
                    .and_then(|d| d.try_lock().ok())
                    .map(|d| d.clock_now())
            })
            .unwrap_or(fallback)
    }

    /// Consumes any pending `"Stopped hold output failed"` aggregate retained due to a full
    /// `safety_log_tx` ring so RT executor teardown can deliver it off-thread without losing count.
    pub(crate) fn take_pending_hold_tx_error_for_shutdown(
        &mut self,
    ) -> Option<crate::safety::SafetyLogEvent> {
        if self.hold_tx_error_suppressed == 0 {
            return None;
        }
        let (partial, message, logged_at) = self.pending_hold_tx_error_retry.take()?;
        let suppressed = self.hold_tx_error_suppressed - 1;
        self.hold_tx_error_suppressed = 0;
        Some(crate::safety::SafetyLogEvent::StoppedHoldOutputFailed {
            partial,
            suppressed,
            message,
            logged_at,
        })
    }

    /// Rate-limit stopped-hold backlog stall diagnostics to at most once per second on the driver
    /// clock (independent of `"Stopped hold output failed"` throttling).
    fn should_log_hold_tx_stall(&mut self, now: Instant) -> bool {
        let should = self
            .last_hold_tx_stall_log
            .is_none_or(|last| now.saturating_duration_since(last) >= Duration::from_secs(1));
        if should {
            self.last_hold_tx_stall_log = Some(now);
        }
        should
    }

    /// Rate-limit `"Stopped hold output failed"` logs to at most 1/s on the driver clock across
    /// both Legacy (direct tracing) and RT Executor (zero-alloc deferred safety log) paths,
    /// maintaining an exact suppressed-event counter even under inline or downstream ring
    /// saturation (rolling back the throttle window if downstream `safety_log_tx` rejects).
    pub(crate) fn log_stopped_hold_output_failed_throttled(&mut self, now: Instant, partial: bool, message: &str) {
        if self
            .last_hold_tx_error_log
            .is_some_and(|last| now.saturating_duration_since(last) < Duration::from_secs(1))
        {
            self.hold_tx_error_suppressed = self.hold_tx_error_suppressed.saturating_add(1);
            return;
        }
        let mut msg = crate::safety::BoundedText::<160>::empty();
        msg.push_str(message);
        if self
            .safety
            .log_stopped_hold_output_failed(partial, self.hold_tx_error_suppressed, msg, now)
        {
            self.hold_tx_error_suppressed = 0;
            self.pending_hold_tx_error_retry = None;
            self.last_hold_tx_error_log = Some(now);
            if !self.safety.deferred_logging() {
                self.last_hold_tx_error_delivered = Some(now);
            }
        } else {
            // Deferred safety-log ring is full of unrelated events: retain this occurrence so no
            // count is lost, and leave `last_hold_tx_error_log` unchanged for immediate retry.
            self.hold_tx_error_suppressed = self.hold_tx_error_suppressed.saturating_add(1);
            self.pending_hold_tx_error_retry = Some((partial, msg, now));
        }
    }

    /// Reset all per-joint non-stopped TX lack-of-progress counters and timestamps.
    pub(crate) fn reset_tx_joint_progress(&mut self) {
        self.tx_joint_no_progress_ticks.fill(0);
        self.tx_joint_no_progress_since.fill(None);
    }

    /// Reset all per-`(joint, slot)` stopped-hold TX lack-of-progress counters and timestamps.
    pub(crate) fn reset_hold_tx_joint_progress(&mut self) {
        self.hold_tx_no_progress_ticks.fill(0);
        self.hold_tx_no_progress_since.fill(None);
    }

    /// True when an active position-force gripper (`damiao_gripper.mode == PosForce`) currently
    /// has stale (`> 100 ms`) or missing feedback. Uses non-blocking `try_lock()` so callers on
    /// `armd-control` never block on a mutex (falling back to `self.j7_stale_since.is_some()` on
    /// contention).
    pub(crate) fn is_pos_force_j7_stale(&self) -> bool {
        if !self.config.damiao_gripper.as_ref().is_some_and(|g| g.mode == crate::model::DamiaoGripperMode::PosForce) {
            return false;
        }
        let Some(d) = self.driver.as_ref() else {
            return false;
        };
        match d.try_lock() {
            Ok(drv) => !drv.pos_force_feedback_fresh(),
            Err(std::sync::TryLockError::Poisoned(p)) => !p.into_inner().pos_force_feedback_fresh(),
            Err(std::sync::TryLockError::WouldBlock) => self.j7_stale_since.is_some(),
        }
    }

    /// Last provably fresh (`<= 100 ms`) J7 position reported by the driver (in user coordinates),
    /// falling back to finite `measured_positions[g]` or `commanded_positions[g]`. Uses
    /// non-blocking `try_lock()` so callers on `armd-control` never block on a mutex.
    pub(crate) fn pos_force_j7_freeze_position(&self) -> Option<f32> {
        let g = self.config.gripper_index()?;
        let raw = self
            .driver
            .as_ref()
            .and_then(|d| match d.try_lock() {
                Ok(drv) => drv.pos_force_last_fresh_position(),
                Err(std::sync::TryLockError::Poisoned(p)) => p.into_inner().pos_force_last_fresh_position(),
                Err(std::sync::TryLockError::WouldBlock) => None,
            })
            .or_else(|| self.measured_positions.get(g).copied().filter(|p| p.is_finite()))
            .or_else(|| self.commanded_positions.get(g).copied().filter(|p| p.is_finite()))?;
        Some(self.config.limits.get(g).map_or(raw, |l| raw.clamp(l.min_position, l.max_position)))
    }

    /// True when the active trajectory drives the gripper (`Reference::Gripper`, or
    /// `Reference::Replay` with `gripper == true` and a configured gripper joint).
    pub(crate) fn active_reference_drives_gripper(&self) -> bool {
        match self.active_trajectory.as_ref().map(|a| &a.reference) {
            Some(Reference::Gripper(_)) => true,
            Some(Reference::Replay(r)) => r.gripper && r.gripper_idx.is_some(),
            _ => false,
        }
    }

    /// Zero-allocation admission check refusing new gripper targets while active `PosForce` J7
    /// feedback is stale (`> 100 ms`). Single source of truth wrapped by
    /// [`Self::check_pos_force_gripper_fresh`].
    pub(crate) fn check_pos_force_gripper_fresh_static(&self) -> Result<(), &'static str> {
        if self.is_pos_force_j7_stale() {
            return Err("J7_FEEDBACK_STALE");
        }
        Ok(())
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
        if let Some(ref d) = self.driver {
            if let Ok(mut drv) = d.try_lock() {
                drv.discard_pending_tx();
                drv.clear_tx_probe();
            }
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

    /// Current owner `(stop_gen, estop_epoch)` stamp read in the tick epilogue (and used for
    /// non-target `Hold`/`Stop` intents). NEVER used to stamp a `PosForceIntent::Target`: every
    /// `Target` carries the A1 [`AdmissionStamp`] captured in `self.j7_admitted_stamp`.
    #[inline]
    pub(crate) fn current_j7_intent_stamp(&self) -> crate::driver::PosForceIntentStamp {
        crate::driver::PosForceIntentStamp {
            stop_gen: self.current_stop_gen(),
            estop_epoch: self.safety.epoch_id(),
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
        tick_output: &mut TickOutput,
        allow_output_accepted: &mut bool,
    ) {
        use crate::driver::{ArmDriverError, PosForceEmitOutcome};
        if drv.enable_state() == EnableState::Disabled {
            return;
        }
        let now = drv.clock_now();
        let max_age = drv.tx_frame_max_age();
        match drv.emit_pos_force_j7(intent, current_stamp) {
            Ok(PosForceEmitOutcome::NotApplicable) => {}
            Ok(PosForceEmitOutcome::AwaitingFirstFresh { pf_status, .. }) => {
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
                    TickOutput::Complete { pf_status: dst_pf, .. }
                    | TickOutput::Deferred { pf_status: dst_pf, .. }
                    | TickOutput::Expired { pf_status: dst_pf, .. }
                    | TickOutput::Error { pf_status: dst_pf, .. } => {
                        *dst_pf = pf_status;
                    }
                    TickOutput::EstopInhibited { .. } => {}
                }
            }
            Ok(PosForceEmitOutcome::Sent { joint, pf_status, .. }) => {
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
                    TickOutput::EstopInhibited { .. } => {}
                }
            }
            Ok(PosForceEmitOutcome::Deferred { joint, pf_status }) => {
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
                            oldest_age: drv.oldest_pending_tx_age(),
                            due_mask: if j7_is_hold { pf_bit_hold } else { 0 },
                            is_hold: j7_is_hold,
                            sent_mask: 0,
                            pf_status,
                        };
                    }
                    TickOutput::Complete { is_hold, now, max_age, due_mask, sent_mask, .. } => {
                        *allow_output_accepted = false;
                        *tick_output = TickOutput::Deferred {
                            pending: 1,
                            now,
                            max_age,
                            oldest_age: drv.oldest_pending_tx_age(),
                            due_mask: if is_hold { due_mask | pf_bit_hold } else { 0 },
                            is_hold,
                            sent_mask: if is_hold { sent_mask } else { drv.last_target_tx_sent_mask() },
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
                            oldest_age: oldest_age.or_else(|| drv.oldest_pending_tx_age()),
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
                }
            }
            Err(ArmDriverError::EstopInhibited) => {
                if matches!(tick_output, TickOutput::None) {
                    *tick_output = TickOutput::EstopInhibited { is_hold: j7_is_hold };
                }
            }
            Err(e) => {
                let pf_bit_hold = drv
                    .gripper_pos_force_info()
                    .map_or(0, |c| if c.joint.saturating_mul(2) < 64 { 1u64 << (c.joint * 2) } else { 0 });
                let pf_status = drv.take_last_pos_force_tx_status();
                match tick_output {
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
                    TickOutput::EstopInhibited { .. } => {}
                }
            }
        }
    }

    /// Tick-epilogue wrapper invoking [`Self::emit_and_merge_pos_force_j7`] under a non-blocking
    /// `try_lock()` on the driver mutex (zero blocking locks on `armd-control`).
    fn emit_j7_tick_epilogue(
        &mut self,
        j7_is_hold: bool,
        intent: crate::driver::PosForceIntent,
        current_stamp: crate::driver::PosForceIntentStamp,
        tick_output: &mut TickOutput,
        allow_output_accepted: &mut bool,
    ) {
        let Some(ref d) = self.driver else {
            return;
        };
        let mut drv = match d.try_lock() {
            Ok(g) => g,
            Err(std::sync::TryLockError::Poisoned(p)) => p.into_inner(),
            Err(std::sync::TryLockError::WouldBlock) => return,
        };
        Self::emit_and_merge_pos_force_j7(
            &mut **drv,
            j7_is_hold,
            intent,
            current_stamp,
            tick_output,
            allow_output_accepted,
        );
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
        let max_age = drv.tx_frame_max_age();
        let hold_sent_any = is_hold && drv.last_hold_tx_sent_any();
        let due_mask = if is_hold { drv.last_hold_tx_due_mask() } else { 0 };
        let hold_sent_mask = if is_hold { drv.last_hold_tx_sent_mask() } else { 0 };
        let pf_status = drv.take_last_pos_force_tx_status();
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
                oldest_age: drv.oldest_pending_tx_age(),
                due_mask,
                is_hold,
                sent_mask: if is_hold { hold_sent_mask } else { drv.last_target_tx_sent_mask() },
                pf_status,
            },
            Ok(TxBatchOutcome::Expired { dropped, pending }) => TickOutput::Expired {
                dropped,
                pending,
                now,
                max_age,
                oldest_age: drv.last_tx_expired_age(),
                due_mask,
                sent_mask: hold_sent_mask,
                is_hold,
                pf_status,
            },
            Err(crate::driver::ArmDriverError::EstopInhibited) => {
                TickOutput::EstopInhibited { is_hold }
            }
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
        self.servo_ws_min_depths = None;
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
            TickOutput::None => {
                self.reset_hold_tx_joint_progress();
                if !self.is_torqued || self.disable_pending || self.shutdown_committed {
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

    /// Publication margin reserved at the end of a control release when bounding per-tick CAN TX
    /// (`DAMIAO_TX_RELEASE_PUBLISH_MARGIN`, capped to `period / 2` for sub-4 ms control periods).
    pub(crate) fn tx_release_publish_margin(&self) -> Duration {
        let period = Duration::from_secs_f64(1.0 / f64::from(self.config.control_frequency_hz.max(1)));
        crate::driver::DAMIAO_TX_RELEASE_PUBLISH_MARGIN.min(period / 2)
    }

    /// Translates an executor/caller `(now, release_deadline)` pair into the absolute driver-clock
    /// TX start-cutoff `drv_now + (release_deadline - now - margin)`.
    pub(crate) fn driver_tx_release_cap(
        &self,
        drv_now: Instant,
        now: Instant,
        release_deadline: Instant,
    ) -> Instant {
        let margin = self.tx_release_publish_margin();
        let remaining = release_deadline
            .saturating_duration_since(now)
            .saturating_sub(margin);
        drv_now + remaining
    }

    /// Installs `driver.set_tx_release_deadline(Some(cap))` for `(now, release_deadline)` and
    /// returns `true` when a driver cap was installed.
    pub(crate) fn install_driver_tx_release_cap(
        &self,
        now: Instant,
        release_deadline: Instant,
    ) -> bool {
        let Some(ref d) = self.driver else {
            return false;
        };
        let Ok(mut driver) = d.lock() else {
            return false;
        };
        let drv_now = driver.clock_now();
        let cap = self.driver_tx_release_cap(drv_now, now, release_deadline);
        driver.set_tx_release_deadline(Some(cap));
        true
    }

    /// Clears any active per-release TX deadline cap on the attached driver.
    pub(crate) fn clear_driver_tx_release_cap(&self) {
        if let Some(ref d) = self.driver {
            if let Ok(mut driver) = d.lock() {
                driver.set_tx_release_deadline(None);
            }
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

    /// Shared Step-2 hardware poll implementation for [`crate::rt_core::ops::CoreOps::poll_hardware`]
    /// and [`crate::rt_core::ops::CoreOps::poll_hardware_for_release`].
    pub(crate) fn poll_hardware_with_release_deadline(
        &mut self,
        budget: &crate::rt_core::ops::ServiceBudget,
        release_deadline: Option<Instant>,
    ) -> crate::rt_core::ops::HardwarePoll {
        let was_torqued_or_disabling = self.is_torqued || self.disable_pending;
        // Install the release publish-margin TX cap BEFORE enable-transition polling, background
        // disable polling, and RX feedback intake so every TX in Step 2 obeys the same cutoff.
        let applied_step2_tx_cap = release_deadline
            .is_some_and(|rd| self.install_driver_tx_release_cap(budget.now, rd));
        let trans = self.service_torque_transition(budget);
        let transition_rx = if let Some(tp) = trans {
            self.core.begin_transition(tp.progress);
            tp.rx
        } else {
            self.service_background_disable(budget)
        };
        let rx = self.poll_hardware_rx(Some(budget), transition_rx, None);
        if applied_step2_tx_cap {
            self.clear_driver_tx_release_cap();
        }
        let rx_frames = rx.rx_frames;
        let rx_exhausted = rx.budget_exhausted;
        self.pending_rx_poll = rx;
        // Run thermal protection BEFORE consuming background disable confirmation and before
        // Step 2b maintenance (`ClearFault`/`RequestShutdown`) or Step 3 command admission so an
        // over-temperature sample arriving on the same poll as confirmed disable cannot suppress
        // `ThermalTrip`.
        self.check_thermal_protection(was_torqued_or_disabling);
        let _ = self.confirm_background_disable_if_ready();
        let budget_exhausted = budget.now > budget.deadline || rx_exhausted;
        let Some(driver) = self.driver.as_ref() else {
            return crate::rt_core::ops::HardwarePoll {
                driverless: true,
                rx_frames,
                budget_exhausted,
                torque: self.core.poll_transition(),
                ..crate::rt_core::ops::HardwarePoll::default()
            };
        };
        let d = driver.lock().unwrap_or_else(|e| e.into_inner());
        d.feedback_ages_into(&mut self.scratch.ages);
        let oldest = self.scratch.ages.iter().copied().try_fold(Duration::ZERO, |m, a| a.map(|a| m.max(a)));
        crate::rt_core::ops::HardwarePoll {
            driverless: false,
            feedback_valid: d.is_feedback_valid(),
            enable_state: Some(d.enable_state()),
            oldest_feedback_age: oldest,
            rx_frames,
            budget_exhausted,
            torque: self.core.poll_transition(),
        }
    }

    /// Single-pass hardware receive + feedback intake.
    ///
    /// Called at Step 2 (`CoreOps::poll_hardware`) with `budget: Some(...)` so Step 3 commits
    /// and Step 4 targets see fresh feedback and Step 5 `tick_control` does not re-read the bus.
    /// Legacy callers invoking `tick_control` directly without a prior `poll_hardware` call this
    /// with `budget: None`.
    pub(crate) fn poll_hardware_rx(
        &mut self,
        budget: Option<&crate::rt_core::ops::ServiceBudget>,
        transition_rx: Option<(u16, bool)>,
        release_deadline: Option<Instant>,
    ) -> PendingRxPoll {
        let mut out = PendingRxPoll { active: true, ..PendingRxPoll::default() };
        let mut new_motor_errors = None;
        let mut sc = std::mem::take(&mut self.scratch);
        sc.recovered.clear();
        if let Some(ref d) = self.driver {
            self.feedback_numeric_valid = false;
            match d.lock() {
                Ok(mut driver) => {
                    let applied_step2_tx_cap = if let (Some(b), Some(rd)) = (budget, release_deadline) {
                        let drv_now = driver.clock_now();
                        let cap = self.driver_tx_release_cap(drv_now, b.now, rd);
                        driver.set_tx_release_deadline(Some(cap));
                        true
                    } else {
                        false
                    };
                    let read_res = match (budget, transition_rx) {
                        (Some(b), Some((frames, exhausted))) => {
                            if driver.read_cached_joint_states_into(&mut sc.states).is_some() {
                                Ok((frames, exhausted))
                            } else {
                                let remaining = b.max_rx_frames.saturating_sub(frames);
                                if remaining == 0 {
                                    Ok((frames, true))
                                } else {
                                    let drv_now = driver.clock_now();
                                    let span = b.deadline.saturating_duration_since(b.now);
                                    driver
                                        .read_joint_states_budgeted(&mut sc.states, drv_now + span, remaining)
                                        .map(|(f2, ex2)| (frames.saturating_add(f2), exhausted || ex2))
                                }
                            }
                        }
                        (Some(b), None) => {
                            let drv_now = driver.clock_now();
                            let span = b.deadline.saturating_duration_since(b.now);
                            driver.read_joint_states_budgeted(&mut sc.states, drv_now + span, b.max_rx_frames)
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
                            driver.take_recovery_writes_into(&mut sc.recovered);
                            let has_loads = driver.joint_loads_into(&mut sc.loads);
                            self.feedback_numeric_valid = Self::states_valid(
                                &sc.states,
                                driver.capabilities(),
                                self.config.dof,
                                has_loads.then_some(sc.loads.as_slice()),
                            );
                            out.invalid_feedback = !self.feedback_numeric_valid;
                            driver.feedback_ages_into(&mut sc.ages);
                            if !sc.recovered.is_empty() || self.tracking.recovery_suspended {
                                out.recovery_ages = true;
                            }
                            let driver_valid = driver.is_feedback_valid();
                            let all_fresh_age = sc.ages.len() == self.config.dof
                                && sc.ages.iter().all(|a| a.is_some_and(|age| age <= Duration::from_millis(250)));
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
                                    driver.as_ref(),
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
                                driver.feedback_receive_instants_into(&mut sc.rx);
                                if sc.rx.len() == self.config.dof && sc.rx.iter().all(Option::is_some) {
                                    sc.rx_vals.clear();
                                    sc.rx_vals.extend(sc.rx.iter().map(|t| t.unwrap()));
                                    out.hw_rx = true;
                                }
                                let threshold = self.config.max_temperature - THERMAL_CLEAR_MARGIN_C;
                                let thermal_below = driver.capabilities().temperature_feedback
                                    && !self.temperatures.is_empty()
                                    && self.temperatures.iter().all(|t| t.is_finite() && *t <= threshold);
                                self.thermal_cooldown = self.evaluate_thermal_cooldown_with_driver(
                                    driver.as_ref(),
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
                            self.last_can_feedback_instant = None;
                            self.thermal_cooldown.reset();
                            out.reset_stillness_on_read_err = true;
                        }
                    }
                    out.bus_feedback_fresh = self.feedback_numeric_valid
                        && driver.feedback_all_within(self.config.dof, BUS_LOSS_TIMEOUT);
                    if driver.motor_status_into(&mut sc.status) {
                        let changed = sc.status.len() != self.motor_errors.len()
                            || sc.status.iter().zip(&self.motor_errors).any(|(code, e)| {
                                let decoded = e.as_ref().map(|e| e.code);
                                decoded != Some(*code) && !(decoded.is_none() && driver.decode_motor_error(*code).is_none())
                            });
                        if changed {
                            new_motor_errors = Some(
                                sc.status
                                    .iter()
                                    .map(|&code| {
                                        driver
                                            .decode_motor_error(code)
                                            .map(|name| crate::model::MotorError { code, name })
                                    })
                                    .collect(),
                            );
                        }
                    }
                    if self.is_torqued
                        && !self.disable_pending
                        && self.gravity_comp.is_none()
                        && self.safety.state().is_operational()
                        && driver.capabilities().enable_feedback
                        && driver.is_feedback_valid()
                        && driver.enable_state() != EnableState::Enabled
                        && !(self.pending_rearm && driver.has_known_partial_enable())
                    {
                        out.driver_torque_lost = true;
                    }
                }
                Err(_) => {
                    self.last_can_feedback_instant = None;
                    self.thermal_cooldown.reset();
                    out.reset_stillness_on_read_err = true;
                }
            }
        } else {
            // Driverless simulation mode: each control tick produces a fresh joint sample.
            self.core.simulate_feedback(self.config.dof, Instant::now());
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
        self.tick_control_inner(dt, None)
    }

    /// [`Self::tick_control`] bounded by an explicit control release deadline (`now`,
    /// `release_deadline`) so TX start-cutoffs never run past `release_deadline - publish_margin`.
    pub fn tick_control_with_deadline(&mut self, dt: f64, now: Instant, release_deadline: Instant) -> f64 {
        self.tick_control_inner(dt, Some((now, release_deadline)))
    }

    fn tick_control_inner(&mut self, dt: f64, release_bound: Option<(Instant, Instant)>) -> f64 {
        let applied_tx_cap = if let Some(ref d) = self.driver {
            if let Ok(mut drv) = d.lock() {
                let drv_now = drv.clock_now();
                let cap = match release_bound {
                    Some((now, release_deadline)) => self.driver_tx_release_cap(drv_now, now, release_deadline),
                    None => {
                        let period = Duration::from_secs_f64(1.0 / f64::from(self.config.control_frequency_hz.max(1)));
                        drv_now + period.saturating_sub(self.tx_release_publish_margin())
                    }
                };
                drv.set_tx_release_deadline(Some(cap));
                true
            } else {
                false
            }
        } else {
            false
        };
        self.consume_pending_estop();
        // Immediate atomic e-stop latch check before any processing (only latches once when not yet recorded;
        // every new E-stop request arrives via `consume_pending_estop` above and advances the epoch).
        if self.e_stop_latched.load(Ordering::SeqCst)
            && !self.safety.has_latched_fault_kind(crate::safety::FaultKind::Estop)
        {
            self.apply_stop_policy(StopPolicy::EStopLatch);
        }

        // Reuse Step 2's single budgeted hardware poll when present; legacy callers without
        // a preceding `poll_hardware` perform the receive pass here.
        let rx = if self.pending_rx_poll.active {
            std::mem::take(&mut self.pending_rx_poll)
        } else {
            self.poll_hardware_rx(None, None, None)
        };
        let invalid_feedback = rx.invalid_feedback;
        let driver_torque_lost = rx.driver_torque_lost;
        let bus_feedback_fresh = rx.bus_feedback_fresh;
        let recovery_ages = rx.recovery_ages;
        let hw_rx = rx.hw_rx;
        let mut sc = std::mem::take(&mut self.scratch);
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
        if driver_torque_lost && self.is_torqued && !self.disable_pending {
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
                crate::safety::FaultDetail::Static(crate::safety::StaticFault::UnexpectedTorqueLoss),
            );
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
            let (is_bus_fresh, has_torque_feedback, has_load_feedback, pos_force_info) = if let Some(ref d) = self.driver {
                if let Ok(driver) = d.lock() {
                    let fresh = self.feedback_numeric_valid
                        && self.last_can_feedback_instant.is_some_and(|t| t.elapsed() < Duration::from_millis(50))
                        && driver.is_feedback_valid();
                    let caps = driver.capabilities();
                    let pf = driver.gripper_pos_force_info().filter(|info| info.joint == g_idx);
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
                    tracing::warn!("Gripper torque cap: |tau| {:.2} N·m above {:.2} N·m at {p_meas:.3} rad (trip {trips} on this contact); holding",
                        tau_g.abs(), force_cap.unwrap_or(0.0));
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
                    tracing::warn!("Gripper torque cap tripped {} times on one contact under a stream; stopping the stream", crate::motion::CAP_RAMP_MAX_TRIPS);
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
                let high_effort = if has_torque_feedback {
                    // the stream contact regulator holds the grip instead of the stall hold at
                    // the measured jaw (which reads ~0 N·m on a rigid object and re-squeezes);
                    // the threshold is raised to the learned free-motion torque + margin where
                    // the jaw needs more than GRIPPER_STALL_LATCH_NM to move
                    // during a calibration stroke the command must also lead the jaw: a free jaw
                    // settling onto its target reads its friction torque at low speed with the
                    // command next to it, and every stall fails the calibration
                    self.gripper_stream_contact.is_none()
                        && tau_g.abs() >= stall_nm
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
                let active_gripper_motion = self.active_trajectory.as_ref().and_then(|a| match &a.reference {
                    crate::motion::Reference::Gripper(gm) if gm.gripper_index == g_idx => Some(gm),
                    _ => None,
                });
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

        // Bus loss: no fresh feedback from every joint within BUS_LOSS_TIMEOUT as observed
        // on this release's RX poll (avoids falsely latching BusLoss when an in-release
        // Step-3 `send_enable` resets post-enable feedback timestamps before aborting on E-stop).
        if self.is_torqued
            && self.driver.is_some()
            && !(self.feedback_numeric_valid && bus_feedback_fresh)
            && !self.safety.has_latched_fault_kind(crate::safety::FaultKind::BusLoss)
        {
            self.execute_stop_in_tick(StopPolicy::BusLossFallback);
        }
        self.sync_torque_state_after_stop();

        // Check hardware motor faults / error flags from feedback
        if self.is_torqued {
            if let Some(ref d) = self.driver {
                if let Ok(driver) = d.lock() {
                    if let Err(e) = driver.check_hardware_status() {
                        if !self.safety.has_latched_fault_kind(crate::safety::FaultKind::Other) {
                            tracing::error!("Hardware fault detected on driver: {}", e);
                        }
                        if self.measured_positions.iter().all(|p| p.is_finite())
                            && self.measured_positions.len() == self.config.dof
                        {
                            copy_into(&mut self.commanded_positions, &self.measured_positions);
                        }
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
                        copy_into(&mut self.target_positions, &self.commanded_positions);
                        self.commanded_velocities.fill(0.0);
                        if !self.safety.has_latched_fault_code(FaultCode::HardwareFault) {
                            self.safety.latch(
                                crate::safety::FaultKind::Other,
                                crate::safety::FaultDetail::formatted(FaultCode::HardwareFault, |buf| {
                                    let _ = core::fmt::write(buf, format_args!("{e}"));
                                }),
                            );
                        }
                    }
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
        // (a torque transition in flight confirms its own disable: `service_torque_transition`)
        if !self.confirm_background_disable_if_ready()
            && self.last_disable_attempt.is_none_or(|t| t.elapsed() >= Duration::from_millis(250))
        {
            if let Err(e) = self.request_disable() {
                tracing::warn!("Torque shutdown remains unconfirmed: {e}");
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
        // Enforce macro contact guard and child deadline BEFORE stepping any trajectory or
        // submitting targets: a success-on-contact stop immediately replaces the trajectory with
        // a hold at the measured pose so this release sends the replacement hold and commits
        // `SubState::Done` strictly through the post-output accepted-output gate.
        if self.macro_run.is_some() {
            self.step_macro_contact();
            self.expire_macro_child(Instant::now());
        }
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
        let gripper_idx_for_ceiling = self.config.gripper_index();
        // a gripper action that ended since the last tick without reporting done (lease loss,
        // fault, cancel), or one still installed while the arm is stopped: hold the measured jaw;
        // one that reported done short of its command (stopped on something) and is not holding
        // an object: the same, so the leading command does not keep pressing
        let grip_active = self.gripper_reference_active();
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
                self.safety.latch(crate::safety::FaultKind::BusLoss, crate::safety::FaultDetail::BusLoss);
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
        let mut tick_output = TickOutput::None;
        let mut allow_output_accepted = false;
        let mut driverless_accepted = false;
        let mut gravity_exiting_or_failed = false;
        let mut gravity_driverless_ok = false;
        let mut gravity_handback_ran = false;
        self.j7_intent = None;
        // a torque transition in flight owns the bus until it ends (the driver refuses targets)
        let transition_owns_bus = self.torque_transition.is_some();
        if !self.shutdown_committed && self.is_torqued && !self.disable_pending && !transition_owns_bus && self.gravity_comp.is_some() {
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
            if self.run_scheduled_gravity_comp_handback() {
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
                } else if let (Some(targets), Some(d)) = (self.gravity_comp_targets_array(), self.driver.as_ref()) {
                    if let Ok(mut driver) = d.lock() {
                        if driver.enable_state() != EnableState::Disabled {
                            let is_hold = self.safety.state().is_stopped();
                            let write_res = if is_hold {
                                driver.write_estop_hold_bounded(&targets)
                            } else {
                                driver.write_joint_targets_bounded(&targets)
                            };
                            tick_output = Self::tick_output_from_driver_write(&mut **driver, is_hold, write_res);
                            if matches!(tick_output, TickOutput::Complete { is_hold: false, .. }) {
                                allow_output_accepted = true;
                            }
                        }
                    }
                }
            }
        } else if !self.shutdown_committed && self.is_torqued && !self.disable_pending && !transition_owns_bus {
            let send_to_bus = feedback_fresh
                && self.hardware_feedback_fresh(BUS_LOSS_TIMEOUT)
                && self.driver.as_ref().is_none_or(|d|
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
                    let mut ramp = self.tracking.fallback_ramp.take().expect("checked above");
                    let governed = self.active_tracking_limits(servo_dt, time_scale);
                    let mut orig_limits = std::mem::take(&mut self.scratch.fallback_limits);
                    copy_limits(&mut orig_limits, &ramp.limits);
                    for (l, g) in ramp.limits.iter_mut().zip(&governed) {
                        l.max_velocity = l.max_velocity.min(g.max_velocity);
                        l.max_acceleration = l.max_acceleration.min(g.max_acceleration);
                    }
                    let mut tick_sweep_budget = MAX_KEEPOUT_EVALS_PER_TICK;
                    let mut pending = self.tracking_fallback_compute(ramp, servo_dt, &mut tick_sweep_budget);
                    std::mem::swap(&mut pending.ramp.limits, &mut orig_limits);
                    self.scratch.fallback_limits = orig_limits;
                    tick_output = self.apply_tracking_fallback_step(pending, gravity_output_enabled);
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
                            let mut targets = std::mem::take(&mut self.scratch.targets);
                            targets.clear();
                            targets.extend(
                                out.positions
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
                                    allow_output_accepted = true;
                                }
                                TickOutput::Deferred { .. }
                                | TickOutput::Expired { .. }
                                | TickOutput::Error { .. }
                                | TickOutput::EstopInhibited { .. }
                                | TickOutput::None => {
                                    self.tracking.invalidate();
                                }
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
                                    self.safety.latch(
                                        crate::safety::FaultKind::Other,
                                        crate::safety::FaultDetail::formatted(FaultCode::Other, |buf| {
                                            let _ = core::fmt::write(buf, format_args!("Joint tracking output rejected: {ws_err}"));
                                        }),
                                    );
                                }
                                crate::tracking::TrackingReject::JointLimit(msg) => {
                                    tracing::warn!("Joint tracking output exceeded limits ({msg}): terminating motion");
                                    self.hold_measured_pose();
                                    self.drop_macro_run("failed", Some("JOINT_LIMIT"));
                                    self.safety.latch(
                                        crate::safety::FaultKind::JointLimit,
                                        crate::safety::FaultDetail::formatted(FaultCode::JointLimit, |buf| {
                                            let _ = core::fmt::write(buf, format_args!("Joint tracking output rejected: {msg}"));
                                        }),
                                    );
                                }
                            }
                            tick_output = self.apply_tracking_fallback_step(pending, gravity_output_enabled);
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
            }
        }

        self.tick_phase(2);
        // An E-stop or authorized stop requested during this tick: its output did not reach
        // the bus (or the batch stopped at the next frame, `ArmDriver::set_estop_gate`); apply
        // it now instead of one period later. A batch the gate stopped is not a driver failure.
        if self.urgent_stop_unapplied() {
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
                TickOutput::EstopInhibited { .. } => {
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
        // Tick epilogue (BOTH Legacy and RT Executor owners): invoke the ONE J7 PosForce emitter
        // exactly once per control tick AFTER whatever arm-write path ran (whether arm output
        // succeeded, deferred, expired, errored, tripped the MIT fence before/mid-batch, hit
        // stale precheck / BuildRefused / invalid caller J7 target, or was skipped by send_to_bus).
        if !self.shutdown_committed
            && self.is_torqued
            && !self.disable_pending
            && !transition_owns_bus
            && !self.urgent_stop_unapplied()
        {
            let current_stamp = self.current_j7_intent_stamp();
            let j7_is_hold = self.safety.state().is_stopped()
                || self.e_stop_latched.load(Ordering::SeqCst);
            let j7_intent = if j7_is_hold {
                crate::driver::PosForceIntent::stop(current_stamp)
            } else if matches!(tick_output, TickOutput::Error { .. } | TickOutput::Expired { .. }) {
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
        let has_pending_motion = matches!(tick_output, TickOutput::None)
            && self.max_tx_joint_no_progress_ticks() > 0
            && self.driver.as_ref().is_some_and(|d| {
                d.lock().is_ok_and(|drv| drv.pending_motion_tx_len() > 0)
            });
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
        if self.macro_run.is_some() {
            self.expire_macro_child(Instant::now());
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
                        DeferredCompletionTarget::ParkRelease(phase) => {
                            self.pending_normal_unload && self.park_release == Some(phase)
                        }
                        DeferredCompletionTarget::Anonymous => {
                            !self.pending_normal_unload
                                && !self.last_action.as_ref().is_some_and(|a| a.is_running())
                        }
                    }
            });
            if !done_still_valid {
                self.pending_deferred_done = None;
            } else if tick_output_accepted {
                if let Some(done) = self.pending_deferred_done.take() {
                    if done.clear_releasing {
                        self.releasing = false;
                        self.held = None;
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
                                    at: Instant::now(),
                                });
                            }
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
        self.reconcile_action(servo_dt, tick_output_accepted);
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

        if applied_tx_cap {
            if let Some(ref d) = self.driver {
                if let Ok(mut drv) = d.lock() {
                    drv.set_tx_release_deadline(None);
                }
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

/// Ownership mode of [`ArmSupervisor`], mirrored atomically for lock-free routing checks.
pub const OWNER_LEGACY: u8 = 0;
pub const OWNER_TRANSFERRING: u8 = 1;
pub const OWNER_EXECUTOR: u8 = 2;
pub const OWNER_FAILED: u8 = 3;

enum OwnerSlot {
    Legacy,
    Transferring,
    Executor(crate::rt_core::executor::RtExecutorHandle),
    Failed,
}

/// Lock-free ticket returned synchronously by [`ArmSupervisor::begin_e_stop_checked`] after
/// latching `e_stop_latched` and advancing `requested_estop_count` via atomics (zero mutexes,
/// zero waiter scans).
pub(crate) struct CheckedEstopTicket {
    req: u64,
    mode: u8,
}

/// Default bounded capacity of each connection's finished-action terminal FIFO.
pub const TERMINAL_FIFO_CAPACITY: usize = 64;

type TerminalSinkFn = Box<dyn Fn(&ArmObservation) -> bool + Send + Sync>;

struct ConnectionTerminalQueue {
    sub_id: u64,
    fifo: std::collections::VecDeque<ActionStatus>,
    seen: std::collections::VecDeque<u64>,
    sink: TerminalSinkFn,
}

struct TerminalBroadcasterInner {
    next_id: u64,
    capacity: usize,
    sinks: Vec<ConnectionTerminalQueue>,
}

/// Lossless off-RT per-connection bounded FIFO of finished actions: every terminal action
/// (`!is_running()`) is appended in exact order to each subscribed connection's bounded FIFO,
/// from which `armd-dispatch` drains every terminal in order onto that connection's lossless
/// ordinary control queue. Never drops a terminal; if full, new action starts are refused with
/// [`SafetyError::TerminalsBusy`] until drained. Never locks on `armd-control`.
pub(crate) struct TerminalBroadcaster {
    subscriber_count: AtomicUsize,
    pending_total: AtomicUsize,
    max_fifo_len: AtomicUsize,
    capacity_atomic: AtomicUsize,
    dispatch_paused: AtomicBool,
    inner: std::sync::Mutex<TerminalBroadcasterInner>,
}

impl Default for TerminalBroadcaster {
    fn default() -> Self {
        Self {
            subscriber_count: AtomicUsize::new(0),
            pending_total: AtomicUsize::new(0),
            max_fifo_len: AtomicUsize::new(0),
            capacity_atomic: AtomicUsize::new(TERMINAL_FIFO_CAPACITY),
            dispatch_paused: AtomicBool::new(false),
            inner: std::sync::Mutex::new(TerminalBroadcasterInner {
                next_id: 1,
                capacity: TERMINAL_FIFO_CAPACITY,
                sinks: Vec::new(),
            }),
        }
    }
}

impl TerminalBroadcaster {
    fn recompute_atomics(&self, g: &TerminalBroadcasterInner) {
        self.subscriber_count.store(g.sinks.len(), Ordering::Release);
        let mut pending = 0usize;
        let mut max_len = 0usize;
        for q in &g.sinks {
            let l = q.fifo.len();
            pending += l;
            if l > max_len {
                max_len = l;
            }
        }
        self.pending_total.store(pending, Ordering::Release);
        self.max_fifo_len.store(max_len, Ordering::Release);
        self.capacity_atomic.store(g.capacity, Ordering::Release);
    }

    pub(crate) fn subscribe(&self, sink: TerminalSinkFn) -> u64 {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let id = g.next_id;
        g.next_id += 1;
        let cap = g.capacity;
        g.sinks.push(ConnectionTerminalQueue {
            sub_id: id,
            fifo: std::collections::VecDeque::with_capacity(cap.min(TERMINAL_FIFO_CAPACITY)),
            seen: std::collections::VecDeque::with_capacity(cap.max(128)),
            sink,
        });
        self.recompute_atomics(&g);
        id
    }

    pub(crate) fn unsubscribe(&self, id: u64) {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        g.sinks.retain(|q| q.sub_id != id);
        self.recompute_atomics(&g);
    }

    pub(crate) fn can_start_action(&self, has_running_action: bool, lock_free: bool) -> bool {
        if self.subscriber_count.load(Ordering::Acquire) == 0 {
            return true;
        }
        // Reserve 1 slot for the new action's eventual terminal, plus 1 immediate slot if a
        // currently running action will be superseded right now.
        let needed = if has_running_action { 2 } else { 1 };
        if lock_free {
            let cap = self.capacity_atomic.load(Ordering::Acquire);
            let max_len = self.max_fifo_len.load(Ordering::Acquire);
            return max_len.saturating_add(needed) <= cap;
        }
        let g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        if g.sinks.is_empty() {
            return true;
        }
        g.sinks
            .iter()
            .all(|q| q.fifo.len().saturating_add(needed) <= g.capacity)
    }

    pub(crate) fn record_finished(&self, action: &ActionStatus) -> bool {
        if action.is_running() || self.subscriber_count.load(Ordering::Acquire) == 0 {
            return false;
        }
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        if g.sinks.is_empty() {
            return false;
        }
        let cap = g.capacity;
        let mut enqueued = false;
        for q in &mut g.sinks {
            if q.seen.contains(&action.id) {
                continue;
            }
            // Never drop or skip a finished action's terminal even if `q.fifo.len() >= cap`:
            // `can_start_action` refuses NEW starts with `TERMINALS_BUSY` once `len >= cap`,
            // while any already-running action's terminal is preserved in strict FIFO order.
            let seen_cap = cap.max(128).max(q.fifo.len() + 1);
            if q.seen.len() >= seen_cap {
                q.seen.pop_front();
            }
            q.seen.push_back(action.id);
            q.fifo.push_back(action.clone());
            enqueued = true;
        }
        if enqueued {
            self.recompute_atomics(&g);
        }
        enqueued
    }

    pub(crate) fn record_from_observation(&self, obs: &ArmObservation) {
        if self.subscriber_count.load(Ordering::Acquire) == 0 {
            return;
        }
        // Check `previous_action` before `action` to preserve chronological FIFO order.
        for act in [obs.previous_action.as_ref(), obs.action.as_ref()]
            .into_iter()
            .flatten()
        {
            if !act.is_running() {
                self.record_finished(act);
            }
        }
    }

    pub(crate) fn update_failed_step(&self, action_id: u64, step: Option<usize>) {
        if self.subscriber_count.load(Ordering::Acquire) == 0 {
            return;
        }
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        for q in &mut g.sinks {
            if let Some(a) = q.fifo.iter_mut().find(|a| a.id == action_id) {
                a.failed_step = step;
            }
        }
    }

    pub(crate) fn has_pending(&self) -> bool {
        !self.dispatch_paused.load(Ordering::Acquire)
            && self.pending_total.load(Ordering::Acquire) > 0
    }

    pub(crate) fn drain_all(&self, base_obs: &ArmObservation) {
        if self.dispatch_paused.load(Ordering::Acquire)
            || self.pending_total.load(Ordering::Acquire) == 0
        {
            return;
        }
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        // Pop a terminal ONLY after its sink confirms a successful enqueue. On failure
        // (e.g., outbound queue full -> fatal close triggered), keep the terminal at the
        // head of the FIFO and keep the subscriber until connection teardown unsubscribes it.
        for q in &mut g.sinks {
            while let Some(finished) = q.fifo.front() {
                let mut obs = base_obs.clone();
                obs.action = Some(finished.clone());
                obs.previous_action = None;
                if (q.sink)(&obs) {
                    q.fifo.pop_front();
                } else {
                    break;
                }
            }
        }
        self.recompute_atomics(&g);
    }

    pub(crate) fn set_capacity(&self, capacity: usize) {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        g.capacity = capacity.max(1);
        self.recompute_atomics(&g);
    }

    pub(crate) fn set_dispatch_paused(&self, paused: bool) {
        self.dispatch_paused.store(paused, Ordering::Release);
    }

    pub(crate) fn pending_total(&self) -> usize {
        self.pending_total.load(Ordering::Acquire)
    }
}

/// RAII subscription unregistering a WS connection from [`TerminalBroadcaster`] on teardown.
pub(crate) struct TerminalSubscription {
    broadcaster: Arc<TerminalBroadcaster>,
    id: u64,
}

impl Drop for TerminalSubscription {
    fn drop(&mut self) {
        self.broadcaster.unsubscribe(self.id);
    }
}

/// Thread-safe Arm Supervisor managing the robotic arm daemon
pub struct ArmSupervisor {
    inner: Arc<SupervisorLock<SupervisorInner>>,
    telemetry_tx: broadcast::Sender<ArmObservation>,
    terminals: Arc<TerminalBroadcaster>,
    pub e_stop_latched: Arc<AtomicBool>,
    pub requested_estop_count: Arc<AtomicU64>,
    stop_gen: Arc<AtomicU64>,
    estop_epoch: Arc<AtomicU64>,
    lease_gen_mirror: Arc<AtomicU64>,
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
    owner_mode: AtomicU8,
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
    template_obs: ArmObservation,
    /// Shared periodic-loop timing stats attached in Legacy mode (`run_periodic`).
    legacy_loop_stats: std::sync::Mutex<Option<Arc<crate::rt_core::executor::ExecutorStats>>>,
    /// Unique identifier for this daemon/supervisor instance (changes across restarts).
    instance_id: String,
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

fn owner_transferred() -> ArmError {
    ArmError::Rejected {
        status: 503,
        code: "OWNER_TRANSFERRED".into(),
        message: "ArmCore is owned by the synchronous RT executor; legacy lock path is disabled".into(),
    }
}

fn executor_unavailable() -> ArmError {
    ArmError::Rejected {
        status: 503,
        code: "EXECUTOR_UNAVAILABLE".into(),
        message: "RT executor ownership is transitioning or unavailable".into(),
    }
}

/// RAII rollback guard for the single `.await` inside [`ArmSupervisor::start_executor_with_hooks`].
struct TransferGuard<'a> {
    sup: &'a ArmSupervisor,
    armed: bool,
}

impl Drop for TransferGuard<'_> {
    fn drop(&mut self) {
        if self.armed {
            let mut slot = self.sup.owner.lock().unwrap_or_else(|e| e.into_inner());
            if matches!(*slot, OwnerSlot::Transferring) {
                *slot = OwnerSlot::Legacy;
                self.sup.owner_mode.store(OWNER_LEGACY, Ordering::Release);
            }
        }
    }
}

/// A published sample, or the read guard when no sample answers (no control loop running).
enum Fresh<'a> {
    Sample(Arc<crate::rt_core::publish::Snapshot>),
    Locked(crate::supervisor_lock::SupReadGuard<'a, SupervisorInner>),
}

impl Drop for ArmSupervisor {
    fn drop(&mut self) { self.hub.close(); }
}

/// Immutable host/process boot identity exposed on `GET /api/v1/diagnostics/tick` so offline
/// qualification analysis can detect kernel reboots and `panic = "abort"` daemon restarts.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProcessIdentity {
    pub boot_id: Option<String>,
    pub process_start_unix_ns: Option<u64>,
}

/// Pure parser computing a process's Unix-epoch start timestamp (nanoseconds) from Linux
/// `/proc/stat` (`btime` seconds) and `/proc/self/stat` (field 22 `starttime` in clock ticks),
/// given `sysconf(_SC_CLK_TCK)`. Handles arbitrary `comm` names containing spaces or parentheses
/// by splitting after the final `')'`. Returns `None` on parse failure, `clk_tck <= 0`, or
/// arithmetic overflow (never emits a placeholder).
pub fn parse_linux_process_start_unix_ns(
    proc_stat: &str,
    proc_self_stat: &str,
    clk_tck: i64,
) -> Option<u64> {
    if clk_tck <= 0 {
        return None;
    }
    let btime_secs: u64 = proc_stat.lines().find_map(|line| {
        let mut toks = line.split_whitespace();
        if toks.next()? == "btime" {
            toks.next()?.parse::<u64>().ok()
        } else {
            None
        }
    })?;
    let after_comm = proc_self_stat.rsplit_once(')')?.1;
    // Fields after `)` begin at field 3 (`state`), so field 22 (`starttime`) is 0-based index 19.
    let start_ticks: u64 = after_comm.split_whitespace().nth(19)?.parse::<u64>().ok()?;
    let btime_ns = u128::from(btime_secs).checked_mul(1_000_000_000)?;
    let offset_ns = u128::from(start_ticks)
        .checked_mul(1_000_000_000)?
        .checked_div(u128::try_from(clk_tck).ok()?)?;
    let total_ns = btime_ns.checked_add(offset_ns)?;
    u64::try_from(total_ns).ok()
}

fn read_process_identity() -> ProcessIdentity {
    #[cfg(target_os = "linux")]
    {
        let boot_id = std::fs::read_to_string("/proc/sys/kernel/random/boot_id")
            .ok()
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty());
        // SAFETY: `sysconf(_SC_CLK_TCK)` is a thread-safe POSIX query with no pointer arguments.
        let clk_tck = unsafe { libc::sysconf(libc::_SC_CLK_TCK) as i64 };
        let process_start_unix_ns = (|| {
            let proc_stat = std::fs::read_to_string("/proc/stat").ok()?;
            let proc_self_stat = std::fs::read_to_string("/proc/self/stat").ok()?;
            parse_linux_process_start_unix_ns(&proc_stat, &proc_self_stat, clk_tck)
        })();
        ProcessIdentity {
            boot_id,
            process_start_unix_ns,
        }
    }
    #[cfg(not(target_os = "linux"))]
    {
        let process_start_unix_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .ok()
            .and_then(|d| u64::try_from(d.as_nanos()).ok())
            .filter(|&ns| ns > 0);
        ProcessIdentity {
            boot_id: None,
            process_start_unix_ns,
        }
    }
}

static PROCESS_IDENTITY: std::sync::OnceLock<ProcessIdentity> = std::sync::OnceLock::new();

/// Eagerly initializes and returns the process-wide immutable boot identity.
/// Called at the top of `main()` and in `ArmSupervisor::new()`.
pub fn init_process_identity() -> &'static ProcessIdentity {
    PROCESS_IDENTITY.get_or_init(read_process_identity)
}

impl ArmSupervisor {
    pub fn new(config: ArmConfig) -> Self {
        let _ = init_process_identity();
        let (tx, _) = broadcast::channel(128);
        let e_stop_latched = Arc::new(AtomicBool::new(false));
        let requested_estop_count = Arc::new(AtomicU64::new(0));
        let stop_gen = Arc::new(AtomicU64::new(0));
        let lease_gen_mirror = Arc::new(AtomicU64::new(0));
        let terminals = Arc::new(TerminalBroadcaster::default());
        let mut inner = SupervisorInner::new(config);
        inner.e_stop_latched = e_stop_latched.clone();
        inner.requested_estop_count = requested_estop_count.clone();
        inner.stop_gen = stop_gen.clone();
        inner.lease_gen_mirror = lease_gen_mirror.clone();
        inner.terminals = Some(Arc::clone(&terminals));
        let template_obs = inner.current_observation();
        let initial_pub_cfg = crate::rt_core::requests::config::PublishedConfig::from_inner(&inner);
        let estop_epoch = inner.safety.epoch_mirror();
        let hub = inner.core.hub().clone();
        let control_frequency_hz = inner.config.control_frequency_hz.max(1);
        let control_period_us = (1e6 / f64::from(control_frequency_hz)) as u64;
        static NEXT_INSTANCE_SEQ: AtomicU64 = AtomicU64::new(1);
        let boot_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_nanos() as u64);
        let seq = NEXT_INSTANCE_SEQ.fetch_add(1, Ordering::Relaxed);
        let instance_id = format!("armd-{}-{boot_ns:x}-{seq}", std::process::id());
        Self {
            inner: Arc::new(SupervisorLock::new(inner)),
            telemetry_tx: tx,
            terminals,
            e_stop_latched,
            requested_estop_count,
            stop_gen,
            estop_epoch,
            lease_gen_mirror,
            tick_stats: Arc::new(crate::diagnostics::tick::LockFreeTickStats::new()),
            control_sched: Arc::new(std::sync::Mutex::new(None)),
            hub,
            dispatcher_start: std::sync::Mutex::new(()),
            control_period_us: Arc::new(AtomicU64::new(control_period_us)),
            control_frequency_hz,
            obs_cache_hits: AtomicU64::new(0),
            obs_cache_misses: AtomicU64::new(0),
            obs_waits: AtomicU64::new(0),
            obs_unavailable: AtomicU64::new(0),
            config_write_mutex: Arc::new(tokio::sync::Mutex::new(())),
            published_config: Arc::new(crate::rt_core::requests::config::PublishedConfigHub::new(
                initial_pub_cfg,
                0,
            )),
            config_commit_hooks: std::sync::Mutex::new(
                crate::rt_core::requests::config::ConfigCommitTestHooks::default(),
            ),
            sequences_revision: Arc::new(AtomicU64::new(0)),
            owner: std::sync::Mutex::new(OwnerSlot::Legacy),
            owner_mode: AtomicU8::new(OWNER_LEGACY),
            executor_base_gen: AtomicU64::new(0),
            estop_reg: Arc::new(std::sync::Mutex::new(crate::rt_core::executor::EstopRegistry::default())),
            sinks: Arc::new(std::sync::Mutex::new(None)),
            template_obs,
            legacy_loop_stats: std::sync::Mutex::new(None),
            instance_id,
        }
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
            let inner = self.inner.read().await;
            inner.episode_delete_allowed(id)?;
            if inner.episodes.is_pinned(id) {
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
    /// live owner and removes it from the `EpisodeStore` catalog. Returns `Ok(true)` if removed,
    /// `Ok(false)` if absent, or `Err(409 IN_USE)` if referenced or pinned by a sequence save.
    pub async fn take_episode_for_delete(&self, id: &str) -> Result<bool, ArmError> {
        use crate::recording::store::TakeEpisodeOutcome;
        loop {
            let rev0 = self.sequences_revision();
            self.check_episode_delete_allowed(id).await?;
            let store = self.episode_store().await;
            match store.take_if_unpinned_and_rev_matches(id, &self.sequences_revision, rev0) {
                TakeEpisodeOutcome::Removed => return Ok(true),
                TakeEpisodeOutcome::NotFound => return Ok(false),
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

    /// Linearizably verifies every episode in `dataset` is unreferenced (saved, running, or
    /// reserved) and atomically removes all of them from the `EpisodeStore` catalog.
    pub async fn take_dataset_episodes_for_delete(&self, dataset: &str) -> Result<Vec<String>, ArmError> {
        use crate::recording::store::TakeDatasetOutcome;
        loop {
            let rev0 = self.sequences_revision();
            let store = self.episode_store().await;
            let ids = store.episodes_of(dataset);
            for id in &ids {
                self.check_episode_delete_allowed(id).await?;
            }
            match store.take_dataset_if_unpinned_and_rev_matches(&ids, &self.sequences_revision, rev0) {
                TakeDatasetOutcome::Removed(taken) => return Ok(taken),
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

    /// Current ownership mode (`OWNER_LEGACY`, `OWNER_TRANSFERRING`, `OWNER_EXECUTOR`, `OWNER_FAILED`).
    pub fn owner_mode(&self) -> u8 {
        self.owner_mode.load(Ordering::Acquire)
    }

    /// Configured control frequency (Hz, immutable after startup, lock-free).
    pub fn control_frequency_hz(&self) -> u32 {
        self.control_frequency_hz
    }

    /// Lock-free current stop generation (bumped under the owner lock on Stop).
    pub fn stop_generation(&self) -> u64 {
        self.stop_gen.load(Ordering::SeqCst)
    }

    /// Lock-free current E-stop / safety epoch id.
    pub fn estop_epoch(&self) -> u64 {
        self.estop_epoch.load(Ordering::SeqCst)
    }

    /// Lock-free mirror of the active lease generation (`0` when no lease is held).
    pub fn lease_generation(&self) -> u64 {
        self.lease_gen_mirror.load(Ordering::SeqCst)
    }

    /// Lock-free admission stamp captured ONCE at request entry (WS reader or REST entry).
    pub fn capture_admission_stamp(&self) -> AdmissionStamp {
        AdmissionStamp {
            stop_gen: self.stop_generation(),
            estop_epoch: self.estop_epoch(),
            lease_gen: self.lease_generation(),
            connection_id: 0,
            request_id: next_admission_request_id(),
        }
    }

    /// Human-readable ownership mode name exposed in `GET /api/v1/diagnostics/tick`.
    pub fn owner_mode_name(&self) -> &'static str {
        match self.owner_mode() {
            OWNER_LEGACY => "legacy",
            OWNER_TRANSFERRING => "transferring",
            OWNER_EXECUTOR => "executor",
            OWNER_FAILED => "failed",
            _ => "unknown",
        }
    }

    /// Attaches the legacy periodic control loop's [`ExecutorStats`] so `GET /api/v1/diagnostics/tick`
    /// reports `skipped_releases`, monotonic release peaks/thresholds, and single-completion publication
    /// into [`LockFreeTickStats`] in Legacy mode.
    pub fn attach_legacy_loop_stats(&self, stats: Arc<crate::rt_core::executor::ExecutorStats>) {
        stats.attach_legacy_tick_ring(Arc::clone(&self.tick_stats));
        *self.legacy_loop_stats.lock().unwrap_or_else(|e| e.into_inner()) = Some(stats);
    }

    /// Signals stop on the legacy periodic control loop, recording the monotonic shutdown timestamp
    /// and waking Linux futex sleep when legacy loop stats are attached.
    pub fn signal_legacy_stop(&self, stop: &AtomicBool) {
        let stats = self.legacy_loop_stats.lock().unwrap_or_else(|e| e.into_inner()).clone();
        if let Some(stats) = stats {
            stats.signal_stop(stop);
        } else {
            stop.store(true, Ordering::SeqCst);
        }
    }

    /// Handle to the synchronous RT executor when ownership has transferred.
    pub fn executor_handle(&self) -> Option<crate::rt_core::executor::RtExecutorHandle> {
        let slot = self.owner.lock().unwrap_or_else(|e| e.into_inner());
        match &*slot {
            OwnerSlot::Executor(h) => Some(h.clone()),
            _ => None,
        }
    }

    fn check_legacy_under_guard(&self, inner: &SupervisorInner) -> Result<(), ArmError> {
        if !inner.detached {
            Ok(())
        } else if self.owner_mode.load(Ordering::Acquire) == OWNER_EXECUTOR {
            Err(owner_transferred())
        } else {
            Err(executor_unavailable())
        }
    }

    fn drain_diag(&self) {
        let sink = self.sinks.lock().unwrap_or_else(|e| e.into_inner()).clone();
        if let Some(s) = sink {
            s.drain_diag(&self.tick_stats);
        }
    }

    /// Transfers ownership of `SupervisorInner` (`ArmCore` + bus driver) once at startup
    /// into the synchronous RT executor OS thread.
    pub async fn start_executor(
        &self,
        cfg: crate::rt_core::executor::ExecutorConfig,
    ) -> Result<crate::rt_core::executor::RtExecutorHandle, ArmError> {
        self.start_executor_with_hooks(cfg, crate::rt_core::executor::PersistTestHooks::default()).await
    }

    #[doc(hidden)]
    pub async fn start_executor_with_hooks(
        &self,
        cfg: crate::rt_core::executor::ExecutorConfig,
        hooks: crate::rt_core::executor::PersistTestHooks,
    ) -> Result<crate::rt_core::executor::RtExecutorHandle, ArmError> {
        self.ensure_dispatcher()?;
        {
            let mut slot = self.owner.lock().unwrap_or_else(|e| e.into_inner());
            if !matches!(*slot, OwnerSlot::Legacy) {
                return Err(ArmError::Rejected {
                    status: 409,
                    code: "OWNER_ALREADY_TRANSFERRED".into(),
                    message: "RT executor ownership can only be transferred once from Legacy mode".into(),
                });
            }
            *slot = OwnerSlot::Transferring;
            self.owner_mode.store(OWNER_TRANSFERRING, Ordering::Release);
        }
        let mut rollback = TransferGuard { sup: self, armed: true };

        // The single `.await` point in handoff: if cancelled here, `TransferGuard` rolls back to `Legacy`.
        let mut guard = self.inner.write().await;

        let restore_legacy = |rollback: &mut TransferGuard<'_>| {
            let mut slot = self.owner.lock().unwrap_or_else(|e| e.into_inner());
            if matches!(*slot, OwnerSlot::Transferring) {
                *slot = OwnerSlot::Legacy;
                self.owner_mode.store(OWNER_LEGACY, Ordering::Release);
            }
            rollback.armed = false;
        };

        // Zero `.await` points below this line: swap + synchronous spawn + commit/rollback.
        if guard.detached
            || guard.calibration_busy
            || guard.motor_zero_busy
            || guard.zero_check_persisting.is_some()
            || guard.gravity_comp.is_some()
            || guard.macro_run.is_some()
            || guard.mit_active()
            || guard.mit.revoke_pending
        {
            restore_legacy(&mut rollback);
            return Err(ArmError::Rejected {
                status: 409,
                code: "TRANSFER_BLOCKED".into(),
                message: "Cannot transfer ownership while calibration, zero_check persistence, motor_zero, gravity compensation, macro action, or MIT tracking is active".into(),
            });
        }
        #[cfg(feature = "policy-runtime")]
        if guard.policy_loading || guard.policy_run.is_some() {
            restore_legacy(&mut rollback);
            return Err(ArmError::Rejected {
                status: 409,
                code: "TRANSFER_BLOCKED".into(),
                message: "Cannot transfer ownership while policy runtime is active or loading".into(),
            });
        }
        if !crate::mit::mit_joints(&guard.config).is_empty() {
            restore_legacy(&mut rollback);
            return Err(ArmError::Rejected {
                status: 503,
                code: "UNSUPPORTED_RT_CONFIG".into(),
                message: "MIT-configured profiles require synchronous qualification/handover and are not supported under synchronous RT executor step 1".into(),
            });
        }
        if let Some(ref d) = guard.driver {
            let mut cached_pos = [None; crate::rt_core::mailbox::MAX_DOF];
            let dof = guard.config.dof.min(crate::rt_core::mailbox::MAX_DOF);
            let cap_res = match d.lock() {
                Ok(drv) => {
                    for (i, slot) in cached_pos.iter_mut().take(dof).enumerate() {
                        *slot = drv.position_limits(i);
                    }
                    drv.supports_rt_executor()
                }
                Err(_) => Err("driver mutex poisoned"),
            };
            guard.cached_driver_pos_limits = cached_pos;
            if let Err(reason) = cap_res {
                restore_legacy(&mut rollback);
                return Err(ArmError::Rejected {
                    status: 503,
                    code: "UNSUPPORTED_RT_DRIVER".into(),
                    message: format!("Cannot transfer ownership: {reason}"),
                });
            }
        }

        // Flush any legacy E-stop acks already applied before handoff so waiting callers
        // receive their exact recorded (state, epoch) instead of stalling after transfer,
        // and drop any legacy retired fault Arcs off-thread before transferring ownership.
        Self::flush_legacy_estop_acks_into(&self.estop_reg, &mut guard);
        guard.safety.drain_retired_state_arcs_off_thread();
        guard.prepare_workspace_eval_bufs();

        let initial_gen = self.inner.generation();
        self.published_config.publish(
            crate::rt_core::requests::config::PublishedConfig::from_inner(&guard),
            initial_gen,
        );
        let shell = SupervisorInner::detached_shell(
            &guard.config,
            self.e_stop_latched.clone(),
            self.requested_estop_count.clone(),
            self.stop_gen.clone(),
            self.lease_gen_mirror.clone(),
        );
        let real = std::mem::replace(&mut *guard, shell);

        match crate::rt_core::executor::spawn_executor(
            real,
            cfg,
            initial_gen,
            Arc::clone(&self.config_write_mutex),
            Arc::clone(&self.published_config),
            Arc::clone(&self.estop_reg),
            Arc::clone(&self.tick_stats),
            Arc::clone(&self.control_sched),
            Arc::clone(&self.control_period_us),
            hooks,
        ) {
            Ok((handle, sinks)) => {
                *self.sinks.lock().unwrap_or_else(|e| e.into_inner()) = Some(sinks);
                self.executor_base_gen.store(initial_gen, Ordering::Release);
                {
                    let mut slot = self.owner.lock().unwrap_or_else(|e| e.into_inner());
                    *slot = OwnerSlot::Executor(handle.clone());
                    self.owner_mode.store(OWNER_EXECUTOR, Ordering::Release);
                }
                rollback.armed = false;
                drop(guard);
                self.hub.wake();
                Ok(handle)
            }
            Err((Some(real), msg)) => {
                *guard = *real;
                restore_legacy(&mut rollback);
                drop(guard);
                Err(ArmError::Internal(msg))
            }
            Err((None, msg)) => {
                self.published_config.set_available(false);
                {
                    let mut slot = self.owner.lock().unwrap_or_else(|e| e.into_inner());
                    *slot = OwnerSlot::Failed;
                    self.owner_mode.store(OWNER_FAILED, Ordering::Release);
                }
                rollback.armed = false;
                drop(guard);
                Err(ArmError::Internal(msg))
            }
        }
    }

    pub fn config_write_lock(&self) -> Arc<tokio::sync::Mutex<()>> {
        Arc::clone(&self.config_write_mutex)
    }

    pub fn published_config_hub(&self) -> &Arc<crate::rt_core::requests::config::PublishedConfigHub> {
        &self.published_config
    }

    /// Returns the latest published configuration snapshot without touching `SupervisorLock`
    /// in `OWNER_EXECUTOR` mode. Fails closed with `503 CONFIG_UNAVAILABLE` if ownership is
    /// transitioning or failed.
    pub async fn published_config(
        &self,
    ) -> Result<Arc<crate::rt_core::requests::config::PublishedConfig>, ArmError> {
        match self.owner_mode() {
            OWNER_EXECUTOR => {
                let Some(handle) = self.executor_handle() else {
                    return Err(crate::rt_core::requests::config::config_unavailable());
                };
                if handle.is_exited() {
                    return Err(crate::rt_core::requests::config::config_unavailable());
                }
                self.published_config
                    .latest()
                    .ok_or_else(crate::rt_core::requests::config::config_unavailable)
            }
            OWNER_LEGACY => {
                let current_gen = self.inner.generation();
                let (cached, cached_gen) = self.published_config.latest_with_gen();
                if cached_gen == current_gen {
                    return Ok(cached);
                }
                let inner = self.inner.read().await;
                if inner.detached {
                    drop(inner);
                    if self.owner_mode() == OWNER_EXECUTOR {
                        let Some(handle) = self.executor_handle() else {
                            return Err(crate::rt_core::requests::config::config_unavailable());
                        };
                        if handle.is_exited() {
                            return Err(crate::rt_core::requests::config::config_unavailable());
                        }
                        return self
                            .published_config
                            .latest()
                            .ok_or_else(crate::rt_core::requests::config::config_unavailable);
                    }
                    return Err(crate::rt_core::requests::config::config_unavailable());
                }
                let gen = self.inner.generation();
                let snap = crate::rt_core::requests::config::PublishedConfig::from_inner(&inner);
                drop(inner);
                Ok(self.published_config.publish_if_newer(snap, gen))
            }
            _ => Err(crate::rt_core::requests::config::config_unavailable()),
        }
    }

    /// Routes one [`ConfigRequest`] through the active single owner (`Legacy` write lock or
    /// `Executor` `MaintenanceOp::Config`) using the identical owner-side handler.
    pub async fn dispatch_config_request(
        &self,
        req: crate::rt_core::requests::config::ConfigRequest,
    ) -> Result<
        crate::rt_core::requests::config::ConfigReply,
        crate::rt_core::requests::config::ConfigError,
    > {
        use crate::rt_core::executor::OwnerError;
        use crate::rt_core::requests::config::{
            handle_config_exchange, ConfigError, ConfigExchange,
        };
        match self.owner_mode() {
            OWNER_EXECUTOR => {
                let Some(handle) = self.executor_handle() else {
                    return Err(ConfigError::OwnerStopped);
                };
                handle.call_config_async(req).await.map_err(|e| match e {
                    OwnerError::QueueFull => ConfigError::OwnerQueueFull,
                    _ => ConfigError::OwnerStopped,
                })
            }
            OWNER_LEGACY => {
                let mut ex = ConfigExchange::new(req);
                let mut inner = self.inner.write().await;
                if inner.detached {
                    drop(inner);
                    let req = ex.req.take().expect("request still present");
                    if self.owner_mode() == OWNER_EXECUTOR {
                        let Some(handle) = self.executor_handle() else {
                            return Err(ConfigError::OwnerStopped);
                        };
                        return handle.call_config_async(req).await.map_err(|e| match e {
                            OwnerError::QueueFull => ConfigError::OwnerQueueFull,
                            _ => ConfigError::OwnerStopped,
                        });
                    }
                    return Err(ConfigError::OwnerStopped);
                }
                let ctx = crate::rt_core::ops::CommitCtx {
                    arm: inner.core.id(),
                    boot: crate::rt_core::broker::BootId::process(),
                    now: Instant::now(),
                    stop_generation: inner.current_stop_gen(),
                };
                handle_config_exchange(&mut inner, &ctx, &mut ex);
                drop(inner);
                ex.reply.take().ok_or(ConfigError::OwnerStopped)
            }
            _ => Err(ConfigError::OwnerStopped),
        }
    }

    /// Dispatches a `ConfigRequest` that must survive transient `MaintenanceOp` queue
    /// backpressure (used for `Persisted`, `Activate`, `Release`, `VerifyAuthority`,
    /// `CalibrationRevert`, and `SetCalibrationBusy(false)` cleanup).
    ///
    /// Retries `ConfigError::OwnerQueueFull` with short bounded backoff while the owner is
    /// still alive; returns immediately on any non-backpressure outcome or `OwnerStopped`.
    pub async fn dispatch_config_request_reliable(
        &self,
        mut build_req: impl FnMut() -> crate::rt_core::requests::config::ConfigRequest,
    ) -> Result<
        crate::rt_core::requests::config::ConfigReply,
        crate::rt_core::requests::config::ConfigError,
    > {
        use crate::rt_core::requests::config::ConfigError;
        let mut backoff_ms = 1u64;
        loop {
            match self.dispatch_config_request(build_req()).await {
                Err(ConfigError::OwnerQueueFull) => {
                    if self.owner_mode() == OWNER_EXECUTOR
                        && self.executor_handle().is_none_or(|h| h.is_exited())
                    {
                        return Err(ConfigError::OwnerStopped);
                    }
                    tokio::time::sleep(Duration::from_millis(backoff_ms)).await;
                    backoff_ms = (backoff_ms * 2).min(10);
                }
                other => return other,
            }
        }
    }

    /// Dispatches a one-shot non-Clone `ConfigRequest` (e.g., carrying a unique `Reservation`
    /// token or `ConfigCandidate`) with guaranteed retry across transient `OwnerQueueFull`.
    /// When the executor queue is full, `MaintClient::call_config_retryable_async` returns the
    /// exact untouched `ConfigRequest` so ownership is never lost.
    pub async fn dispatch_config_request_owned_reliable(
        &self,
        mut req: crate::rt_core::requests::config::ConfigRequest,
    ) -> Result<
        crate::rt_core::requests::config::ConfigReply,
        crate::rt_core::requests::config::ConfigError,
    > {
        use crate::rt_core::requests::config::{
            handle_config_exchange, ConfigError, ConfigExchange,
        };
        let mut backoff_ms = 1u64;
        loop {
            match self.owner_mode() {
                OWNER_EXECUTOR => {
                    let Some(handle) = self.executor_handle() else {
                        return Err(ConfigError::OwnerStopped);
                    };
                    if handle.is_exited() {
                        return Err(ConfigError::OwnerStopped);
                    }
                    match handle.call_config_try_async(req).await {
                        Ok(reply) => return Ok(reply),
                        Err((returned_req, crate::rt_core::executor::OwnerError::QueueFull)) => {
                            req = returned_req;
                            tokio::time::sleep(Duration::from_millis(backoff_ms)).await;
                            backoff_ms = (backoff_ms * 2).min(10);
                        }
                        Err((_, _)) => return Err(ConfigError::OwnerStopped),
                    }
                }
                OWNER_LEGACY => {
                    let mut ex = ConfigExchange::new(req);
                    let mut inner = self.inner.write().await;
                    if inner.detached {
                        drop(inner);
                        req = ex.req.take().expect("request still present");
                        continue;
                    }
                    let ctx = crate::rt_core::ops::CommitCtx {
                        arm: inner.core.id(),
                        boot: crate::rt_core::broker::BootId::process(),
                        now: Instant::now(),
                        stop_generation: inner.current_stop_gen(),
                    };
                    handle_config_exchange(&mut inner, &ctx, &mut ex);
                    drop(inner);
                    return ex.reply.take().ok_or(ConfigError::OwnerStopped);
                }
                _ => return Err(ConfigError::OwnerStopped),
            }
        }
    }

    #[doc(hidden)]
    pub fn set_config_commit_test_hooks(
        &self,
        hooks: crate::rt_core::requests::config::ConfigCommitTestHooks,
    ) {
        *self
            .config_commit_hooks
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = hooks;
    }

    #[doc(hidden)]
    pub fn config_commit_test_hooks(
        &self,
    ) -> crate::rt_core::requests::config::ConfigCommitTestHooks {
        self.config_commit_hooks
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
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
        self.published_config
            .update(|cfg| cfg.operator_auth_enforced = enforced);
    }

    /// Every tick's observation, forwarded by the dispatcher thread (started here when it
    /// does not run yet; an error when it cannot start); a lagging receiver skips
    /// observations, a lagging dispatcher forwards only the newest.
    pub fn subscribe_telemetry(&self) -> Result<broadcast::Receiver<ArmObservation>, ArmError> {
        self.ensure_dispatcher()?;
        Ok(self.telemetry_tx.subscribe())
    }

    /// Subscribes a WS connection's lossless ordinary control queue to unseen action-terminal
    /// observations so terminals are never lost to telemetry broadcast lag.
    pub(crate) fn subscribe_action_terminals(
        &self,
        sink: impl Fn(&ArmObservation) -> bool + Send + Sync + 'static,
    ) -> TerminalSubscription {
        let id = self.terminals.subscribe(Box::new(sink));
        TerminalSubscription {
            broadcaster: Arc::clone(&self.terminals),
            id,
        }
    }

    /// Starts the dispatcher thread (armd does at startup, before the control loop): it
    /// frees the observations the tick retired, forwards telemetry and drains the
    /// recording ring. Idempotent.
    pub fn start_dispatcher(&self) -> Result<(), ArmError> { self.ensure_dispatcher() }

    /// Starts the dispatcher thread once: a normal-priority thread woken by every
    /// publication that forwards samples to [`Self::subscribe_telemetry`] receivers,
    /// drains the recording ring, and drains executor diagnostics/reclamation sinks.
    pub(crate) fn ensure_dispatcher(&self) -> Result<(), ArmError> {
        if self.hub.has_dispatcher() { return Ok(()); }
        let _start = self.dispatcher_start.lock().unwrap_or_else(|e| e.into_inner());
        if self.hub.has_dispatcher() { return Ok(()); }
        let (hub, tx, terminals, sinks, tick_stats, template_obs) = (
            Arc::clone(&self.hub),
            self.telemetry_tx.clone(),
            Arc::clone(&self.terminals),
            Arc::clone(&self.sinks),
            Arc::clone(&self.tick_stats),
            self.template_obs.clone(),
        );
        let handle = std::thread::Builder::new()
            .name("armd-dispatch".into())
            .spawn(move || dispatch_loop(&hub, &tx, &terminals, &sinks, &tick_stats, &template_obs))
            .map_err(|e| ArmError::Internal(format!("cannot start the observation dispatcher thread: {e}")))?;
        self.hub.set_dispatcher(handle.thread().clone());
        Ok(())
    }

    /// Test-only hook: configure per-connection bounded terminal FIFO capacity.
    #[doc(hidden)]
    pub fn test_set_terminal_fifo_capacity(&self, capacity: usize) {
        self.terminals.set_capacity(capacity);
    }

    /// Test-only hook: pause or resume draining of per-connection terminal FIFOs in `armd-dispatch`.
    #[doc(hidden)]
    pub fn test_set_terminal_dispatch_paused(&self, paused: bool) {
        self.terminals.set_dispatch_paused(paused);
        if !paused {
            self.hub.wake();
        }
    }

    /// Test-only accessor for total pending finished actions across subscribed terminal FIFOs.
    #[doc(hidden)]
    pub fn test_pending_terminal_count(&self) -> usize {
        self.terminals.pending_total()
    }

    /// Test-only accessor for subscribed WS terminal FIFO count.
    #[doc(hidden)]
    pub fn test_terminal_subscriber_count(&self) -> usize {
        self.terminals.subscriber_count.load(Ordering::Acquire)
    }

    #[doc(hidden)]
    pub fn inner(&self) -> &Arc<SupervisorLock<SupervisorInner>> {
        &self.inner
    }

    #[doc(hidden)]
    pub fn hub(&self) -> &Arc<crate::rt_core::publish::StateHub> {
        &self.hub
    }

    /// The current observation, without the supervisor lock while the control loop runs
    /// (`GET /api/v1/observation`).
    pub async fn observation(&self) -> Result<ArmObservation, ArmError> {
        match self.fresh(true).await? {
            Fresh::Sample(s) => Ok(s.obs.clone()),
            Fresh::Locked(inner) => Ok(inner.current_observation()),
        }
    }

    /// [`Self::observation`] for in-process callers and tests: when it is unavailable in Legacy
    /// mode, waits for the read lock without a bound; in Executor mode, routes through the RT owner
    /// (never reading the detached shell).
    pub async fn get_observation(&self) -> ArmObservation {
        match self.observation().await {
            Ok(obs) => obs,
            Err(_) => {
                if self.owner_mode() == OWNER_EXECUTOR {
                    if let Some(h) = self.executor_handle() {
                        if let Ok(obs) = h.observation_async().await {
                            return obs;
                        }
                    }
                    if let Some(s) = self.hub.latest() {
                        return s.obs.clone();
                    }
                    return ArmObservation::blank();
                }
                let inner = self.inner.read().await;
                if self.check_legacy_under_guard(&inner).is_ok() {
                    return inner.current_observation();
                }
                drop(inner);
                if let Some(h) = self.executor_handle() {
                    if let Ok(obs) = h.observation_async().await {
                        return obs;
                    }
                }
                self.hub.latest().map_or_else(ArmObservation::blank, |s| s.obs.clone())
            }
        }
    }

    fn wait_bound(&self) -> Duration { Duration::from_micros(2 * self.control_period_us.load(Ordering::Relaxed)) }

    fn answers(s: &crate::rt_core::publish::Snapshot, want: u64, now: Instant) -> bool {
        s.meta.generation >= want && now <= s.meta.valid_until
    }

    async fn fresh(&self, count: bool) -> Result<Fresh<'_>, ArmError> {
        use crate::rt_core::publish::Poll;
        let mode = self.owner_mode();
        if mode == OWNER_TRANSFERRING || mode == OWNER_FAILED {
            if count { self.obs_unavailable.fetch_add(1, Ordering::Relaxed); }
            return Err(observation_unavailable());
        }
        let want = if mode == OWNER_EXECUTOR {
            self.executor_base_gen.load(Ordering::Acquire).saturating_add(1)
        } else {
            self.inner.generation()
        };
        let started = Instant::now();
        let bound = self.wait_bound();
        let mut waited = false;
        loop {
            let poll = self.hub.try_latest();
            let now = Instant::now();
            match poll {
                Poll::Ready(None) if mode != OWNER_EXECUTOR => break,
                Poll::Ready(None) => {}
                Poll::Ready(Some(s)) => {
                    if Self::answers(&s, want, now) {
                        if count {
                            self.obs_cache_hits.fetch_add(1, Ordering::Relaxed);
                            if waited { self.obs_waits.fetch_add(1, Ordering::Relaxed); }
                        }
                        return Ok(Fresh::Sample(s));
                    }
                    if mode != OWNER_EXECUTOR && now.saturating_duration_since(s.meta.sampled_at) > bound {
                        break;
                    }
                }
                Poll::Busy => {}
            }
            if now.duration_since(started) >= bound { break; }
            waited = true;
            tokio::time::sleep(OBS_POLL).await;
        }
        // Executor mode NEVER falls back to reading the detached shell under `self.inner`.
        if self.owner_mode() != OWNER_LEGACY {
            if let Poll::Ready(Some(s)) = self.hub.try_latest() {
                if Self::answers(&s, want, Instant::now()) {
                    if count { self.obs_cache_hits.fetch_add(1, Ordering::Relaxed); }
                    return Ok(Fresh::Sample(s));
                }
            }
            if count { self.obs_unavailable.fetch_add(1, Ordering::Relaxed); }
            return Err(observation_unavailable());
        }
        let seen_seq = self.hub.published_seq();
        let lock_deadline = tokio::time::Instant::now() + OBS_LOCK_WAIT;
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
        if self.check_legacy_under_guard(&inner).is_err() {
            drop(inner);
            return unavailable();
        }
        if self.hub.published_seq() != seen_seq {
            drop(inner);
            if let Poll::Ready(Some(s)) = self.hub.try_latest() {
                if Self::answers(&s, want, Instant::now()) {
                    if count { self.obs_cache_hits.fetch_add(1, Ordering::Relaxed); }
                    return Ok(Fresh::Sample(s));
                }
            }
            let Ok(inner) = tokio::time::timeout_at(lock_deadline, self.inner.read()).await else { return unavailable(); };
            if self.check_legacy_under_guard(&inner).is_err() {
                drop(inner);
                return unavailable();
            }
            if count { self.obs_cache_misses.fetch_add(1, Ordering::Relaxed); }
            return Ok(Fresh::Locked(inner));
        }
        if count { self.obs_cache_misses.fetch_add(1, Ordering::Relaxed); }
        Ok(Fresh::Locked(inner))
    }

    fn running_sample(&self) -> Option<Arc<crate::rt_core::publish::Snapshot>> {
        self.hub.latest().filter(|s| s.meta.sampled_at.elapsed() <= self.wait_bound())
    }

    pub async fn arm_feedback_snapshot(&self, arm: &[usize]) -> (Vec<f32>, Vec<f32>, Vec<Option<Instant>>, Vec<u64>) {
        if let Some(s) = self.running_sample() {
            return s.rt.arm_feedback(arm);
        }
        if self.owner_mode() != OWNER_LEGACY {
            return self
                .hub
                .latest()
                .map_or_else(|| (Vec::new(), Vec::new(), Vec::new(), Vec::new()), |s| s.rt.arm_feedback(arm));
        }
        let inner = self.inner.read().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            return inner.arm_feedback_snapshot();
        }
        drop(inner);
        self.hub
            .latest()
            .map_or_else(|| (Vec::new(), Vec::new(), Vec::new(), Vec::new()), |s| s.rt.arm_feedback(arm))
    }

    pub async fn measured_positions(&self) -> Vec<f32> {
        if let Some(s) = self.running_sample() {
            return s.rt.positions.clone();
        }
        if self.owner_mode() != OWNER_LEGACY {
            return self.hub.latest().map_or_else(Vec::new, |s| s.rt.positions.clone());
        }
        let inner = self.inner.read().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            return inner.measured_positions.clone();
        }
        drop(inner);
        self.hub.latest().map_or_else(Vec::new, |s| s.rt.positions.clone())
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
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            inner.follower = Some(name.to_string());
        }
    }

    pub async fn get_config(&self) -> ArmConfig {
        let inner = self.inner.read().await;
        inner.config.clone()
    }

    pub async fn try_set_driver(&self, mut driver: Box<dyn ArmDriver>) -> Result<(), ArmError> {
        driver.set_abort_flag(self.e_stop_latched.clone());
        let mut inner = self.inner.write().await;
        self.check_legacy_under_guard(&inner)?;
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
        inner.install_driver(drv);
        Ok(())
    }

    pub async fn set_driver(&self, driver: Box<dyn ArmDriver>) {
        let _ = self.try_set_driver(driver).await;
    }

    pub fn with_driver(self, mut driver: Box<dyn ArmDriver>) -> Self {
        driver.set_abort_flag(self.e_stop_latched.clone());
        if let Ok(mut inner) = self.inner.try_write() {
            if self.check_legacy_under_guard(&inner).is_ok() {
                driver.set_estop_gate(inner.estop_gate());
                driver.set_gripper_index(inner.config.gripper_index());
                driver.set_strict_mode_handover(inner.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled));
                if driver.capabilities().load_feedback {
                    inner.loads = vec![f32::NAN; inner.config.dof];
                }
                let driver: Box<dyn ArmDriver> = Box::new(crate::joint_zero::ZeroOffsetDriver::new(driver, &inner.joint_zero_offsets));
                inner.apply_driver_limits(driver.as_ref());
                inner.install_driver(driver);
            }
        }
        self
    }

    pub async fn enable_torque(&self, stamp: AdmissionStamp) -> Result<SafetyState, ArmError> {
        if self.e_stop_latched.load(Ordering::SeqCst) {
            return Err(ArmError::Safety(SafetyError::EStopActive));
        }
        let mut inner = self.inner.clone().write_owned().await;
        self.check_legacy_under_guard(&inner)?;
        // Post-lock double-check: verify atomic e-stop latch again after acquiring write lock
        if inner.estop_pending() {
            return Err(ArmError::Safety(SafetyError::EStopActive));
        }
        tokio::task::spawn_blocking(move || {
            let admitted = inner.check_admission(stamp)?;
            SupervisorInner::enable_torque(admitted)
        })
        .await
        .map_err(|e| ArmError::Internal(format!("enable_torque blocking task failed: {e}")))?
        .map_err(ArmError::Safety)
    }

    /// Exempt from `Admitted` (Invariant 1c): torque disable is a stop-direction action that
    /// removes actuator output and must never be blocked by a Stop or admission check.
    pub async fn disable_torque(&self) -> Result<SafetyState, ArmError> {
        let mut inner = self.inner.clone().write_owned().await;
        self.check_legacy_under_guard(&inner)?;
        tokio::task::spawn_blocking(move || inner.disable_torque())
            .await
            .map_err(|e| ArmError::Internal(format!("disable_torque blocking task failed: {e}")))?
            .map_err(ArmError::Safety)
    }

    pub async fn heartbeat(&self, client_id: &str, epoch_id: u64) -> Result<(), SafetyError> {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_err() {
            return Err(SafetyError::OwnerTransferred);
        }
        inner.heartbeat(client_id, epoch_id)
    }

    pub async fn acquire_lease(&self, client_id: &str) -> Result<u64, (String, u64)> {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_err() {
            return Err(("OWNER_TRANSFERRED".into(), 0));
        }
        inner.acquire_lease(client_id)
    }

    pub async fn preempt_lease(&self, conn: &str, label: &str) -> Result<(u64, Option<(String, u64)>), PreemptError> {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_err() {
            return Err(PreemptError::OwnerTransferred);
        }
        inner.preempt_lease(conn, label)
    }

    pub async fn is_handover_settling(&self) -> bool {
        let inner = self.inner.read().await;
        self.check_legacy_under_guard(&inner).is_ok() && inner.handover_settling_since.is_some()
    }

    pub async fn release_lease(&self, client_id: &str, generation: u64) -> bool {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_err() {
            return false;
        }
        inner.release_lease(client_id, generation)
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
            .is_some_and(|s| Self::answers(&s, want, now) && s.obs.lease_holder.is_some())
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

    pub async fn jog(
        &self,
        stamp: AdmissionStamp,
        target_positions: Vec<f32>,
        duration_secs: Option<f64>,
        epoch_id: Option<u64>,
    ) -> Result<u64, ArmError> {
        let mut inner = self.inner.write().await;
        self.check_legacy_under_guard(&inner)?;
        let admitted = inner.check_admission(stamp)?;
        SupervisorInner::jog(admitted, target_positions, duration_secs, epoch_id)
    }

    pub async fn home(
        &self,
        stamp: AdmissionStamp,
        duration_secs: Option<f64>,
        epoch_id: Option<u64>,
    ) -> Result<u64, ArmError> {
        let mut inner = self.inner.write().await;
        self.check_legacy_under_guard(&inner)?;
        let home_pos = inner.config.home_positions.clone();
        let admitted = inner.check_admission(stamp)?;
        SupervisorInner::jog_action(admitted, home_pos, duration_secs, epoch_id, "home").map(|(epoch, _, _)| epoch)
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
    pub fn attempt_legacy_panic_disable(&self, timeout: Duration) -> PanicDisableOutcome {
        self.e_stop_latched.store(true, Ordering::SeqCst);
        self.requested_estop_count.fetch_add(1, Ordering::SeqCst);
        let run_inline = |inner: Arc<SupervisorLock<SupervisorInner>>,
                          coop_deadline: Instant,
                          hard_deadline: Instant,
                          coop_budget: Duration|
         -> PanicDisableOutcome {
            std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                let mut guard = loop {
                    if let Ok(g) = inner.try_write() {
                        break g;
                    }
                    let rem = hard_deadline.saturating_duration_since(Instant::now());
                    if rem.is_zero() {
                        return PanicDisableOutcome::TimedOut;
                    }
                    std::thread::sleep(rem.min(Duration::from_millis(2)));
                };
                if guard.detached {
                    return PanicDisableOutcome::NoDriver;
                }
                if Instant::now() >= hard_deadline {
                    return PanicDisableOutcome::TimedOut;
                }
                guard.attempt_panic_disable_until(coop_deadline, hard_deadline, coop_budget)
            }))
            .unwrap_or_else(|_| PanicDisableOutcome::Failed("panic during legacy panic-disable".into()))
        };

        let origin = Instant::now();
        let (coop_deadline, hard_deadline) = driver_transitions::panic_disable_deadlines(origin, timeout);
        let coop_budget = coop_deadline.saturating_duration_since(origin);
        #[cfg(not(target_arch = "wasm32"))]
        {
            let inner = Arc::clone(&self.inner);
            let (tx, rx) = std::sync::mpsc::sync_channel(1);
            match std::thread::Builder::new()
                .name("armd-panic-disable".into())
                .spawn(move || {
                    let outcome = run_inline(inner, coop_deadline, hard_deadline, coop_budget);
                    let _ = tx.send((outcome, Instant::now()));
                }) {
                Ok(_) => {
                    let wait = hard_deadline.saturating_duration_since(Instant::now());
                    match rx.recv_timeout(wait) {
                        Ok((outcome, completed_at)) if completed_at < hard_deadline => outcome,
                        Ok(_) | Err(std::sync::mpsc::RecvTimeoutError::Timeout) => PanicDisableOutcome::TimedOut,
                        Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {
                            PanicDisableOutcome::Failed("panic-disable helper thread disconnected".into())
                        }
                    }
                }
                Err(e) => {
                    // Never fall back to an unbounded inline driver call when the bounded
                    // helper thread cannot be spawned; only confirm NoDriver via non-blocking try_write.
                    if let Ok(guard) = self.inner.try_write() {
                        if guard.detached || guard.driver.is_none() {
                            return PanicDisableOutcome::NoDriver;
                        }
                    }
                    PanicDisableOutcome::Failed(format!(
                        "failed to spawn bounded panic-disable helper thread: {e}"
                    ))
                }
            }
        }
        #[cfg(target_arch = "wasm32")]
        {
            run_inline(Arc::clone(&self.inner), coop_deadline, hard_deadline, coop_budget)
        }
    }

    pub async fn request_shutdown(&self) -> Result<bool, ArmError> {
        match self.owner_mode() {
            OWNER_EXECUTOR => match self.executor_handle() {
                Some(h) => h.request_shutdown_async().await.map_err(Into::into),
                None => Err(executor_unavailable()),
            },
            OWNER_LEGACY => {
                let mut inner = self.inner.clone().write_owned().await;
                if self.check_legacy_under_guard(&inner).is_ok() {
                    return tokio::task::spawn_blocking(move || inner.request_shutdown())
                        .await
                        .map_err(|e| ArmError::Internal(format!("request_shutdown blocking task failed: {e}")));
                }
                drop(inner);
                match self.executor_handle() {
                    Some(h) => h.request_shutdown_async().await.map_err(Into::into),
                    None => Err(executor_unavailable()),
                }
            }
            _ => Err(executor_unavailable()),
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
        self.e_stop_latched.store(true, Ordering::SeqCst);
        let req = self.requested_estop_count.fetch_add(1, Ordering::SeqCst) + 1;
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
                    guard.exit_gravity_comp("STOPPED", "Stop command");
                    guard.stop_motion();
                    guard.request_macro_stop();
                    let stop_gen = guard.advance_stop_generation();
                    let applied_epoch = guard.safety.epoch_id();
                    on_legacy_applied(applied_epoch, stop_gen);
                    return Ok(UrgentStopOutcome::Applied {
                        stop_generation: stop_gen,
                        applied_to_motion: true,
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

    /// Execute a control cycle step
    pub async fn tick(&self, dt: f64) -> ArmObservation {
        crate::rt_core::executor::assert_not_in_rt_release("ArmSupervisor::tick");
        self.tick_shared(dt, true, None).await.expect("tick keeps a copy when asked")
    }

    /// [`Self::tick`] for the control loop, which does not use the observation: saves a copy.
    pub async fn control_tick(&self, dt: f64) {
        crate::rt_core::executor::assert_not_in_rt_release("ArmSupervisor::control_tick");
        self.tick_shared(dt, false, None).await;
    }

    /// [`Self::control_tick`] bounded by the caller's absolute scheduled release grid deadline
    /// (as passed from [`crate::rt_core::executor::run_periodic_with_deadline`]).
    pub async fn control_tick_with_deadline(&self, dt: f64, release_deadline: Instant) {
        crate::rt_core::executor::assert_not_in_rt_release("ArmSupervisor::control_tick_with_deadline");
        self.tick_shared(dt, false, Some(release_deadline)).await;
    }

    /// One tick; publishes its observation (`crate::rt_core::publish`) and returns a copy
    /// when `keep`.
    async fn tick_shared(
        &self,
        dt: f64,
        keep: bool,
        scheduled_deadline: Option<Instant>,
    ) -> Option<ArmObservation> {
        crate::rt_core::executor::assert_not_in_rt_release("ArmSupervisor::tick_shared");
        let wake_lateness_ns = crate::diagnostics::tick::take_current_wake_lateness_ns();
        let wake_lateness_us = (wake_lateness_ns / 1_000).min(u64::from(u32::MAX)) as u32;
        if self.owner_mode() != OWNER_LEGACY {
            return None;
        }
        use crate::diagnostics::tick::{us, UsageMark};
        let started = Instant::now();
        let usage_start = UsageMark::now();
        let (obs, terminal_obs, wait, lock_held, control_hz, pending_zc, phases, log, locked, usage_locked, pre, publish, allocs) = {
            let (mut inner, generation) = self.inner.write_tick().await;
            if inner.detached || self.owner_mode() != OWNER_LEGACY {
                return None;
            }
            let locked = Instant::now();
            let usage_locked = UsageMark::now();
            let _ = crate::diagnostics::alloc::take();
            // No other guard exists now: the log is complete up to the guard waited for.
            let log = self.inner.take_log();
            inner.defer_zero_check_persist = true;
            inner.tick_phases_us = [0; 5];
            let body_start = Instant::now();
            inner.tick_phase_mark = Some(body_start);
            let release_deadline = scheduled_deadline.unwrap_or_else(|| {
                let period = Duration::from_secs_f64(1.0 / f64::from(inner.config.control_frequency_hz.max(1)));
                started + period
            });
            let TickPublished { kept, body_end } = inner.tick_publish_with_deadline(
                dt,
                generation,
                keep,
                Some((body_start, release_deadline)),
            );
            let body_end = body_end.unwrap_or(body_start);
            let phases = inner.tick_phases_us;
            inner.defer_zero_check_persist = false;
            let pending_zc = inner.pending_zero_check_persist.take();
            let control_hz = inner.config.control_frequency_hz;
            let lock_end = Instant::now();
            let allocs = crate::diagnostics::alloc::take();
            Self::flush_legacy_estop_acks_into(&self.estop_reg, &mut inner);
            // Capture any terminal action observation WITHOUT touching `self.hub.latest()` so a
            // preempted reader holding `StateHub`'s reader lock can never delay `tick_shared`.
            let terminal_obs = if inner.last_action.as_ref().is_some_and(|a| !a.is_running())
                || inner.previous_action.as_ref().is_some_and(|(a, _)| !a.is_running())
            {
                kept.clone().or_else(|| Some(inner.current_observation()))
            } else {
                None
            };
            (kept, terminal_obs, locked - started, lock_end - locked, control_hz, pending_zc, phases, log, locked, usage_locked,
                body_start - locked, lock_end - body_end, allocs)
        };
        let released = Instant::now();
        let usage_released = UsageMark::now();
        if let Some(ref tobs) = terminal_obs {
            self.terminals.record_from_observation(tobs);
            if self.terminals.has_pending() {
                self.hub.wake();
            }
        }
        self.control_period_us.store((1e6 / f64::from(control_hz.max(1))) as u64, Ordering::Relaxed);
        if let Some(job) = pending_zc {
            let sup_inner = Arc::clone(&self.inner);
            let cw_lock = Arc::clone(&self.config_write_mutex);
            let pub_hub = Arc::clone(&self.published_config);
            let hooks = self.config_commit_test_hooks();
            tokio::spawn(async move {
                crate::rt_core::requests::config::run_legacy_zero_check_commit(
                    sup_inner, cw_lock, pub_hub, hooks, job,
                )
                .await;
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
        let mut sample = crate::diagnostics::tick::TickSample {
            tick_us: us(done - started),
            release_us: 0,
            wait_us: us(wait),
            lock_us: us(lock_held),
            post_us: us(done - released),
            period_us: if dt.is_finite() && dt >= 0.0 { us(Duration::from_secs_f64(dt.min(4.0e3))) } else { 0 },
            wake_lateness_us,
            reader_hold_us: us(log.reader_hold),
            writer_hold_us: us(log.writer_hold),
            reader_acquire_us: us(log.reader_acquire),
            writer_acquire_us: us(log.writer_acquire),
            stats_wait_us: 0,
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
            release_seq: 0,
            safety_epoch: obs.as_ref().map_or(0, |o| o.epoch_id),
            maint_req_id: 0,
            release_flags: 0,
        };
        if crate::diagnostics::tick::periodic_defers_sample_publish() {
            // Single-completion point in `run_periodic_with_clock` will stamp full `release_us`
            // (covering wake through `block_on` return and stats publication) and record once.
            crate::diagnostics::tick::stash_periodic_sample(sample);
        } else {
            sample.release_us = us(started.elapsed());
            let _ = self.tick_stats.record(&sample);
        }
        obs
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
        let active_stats = if let Some(h) = self.executor_handle() {
            Some(Arc::clone(h.stats()))
        } else {
            self.legacy_loop_stats
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .clone()
        };

        let mut snap = if let Some(s) = active_stats {
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

/// The dispatcher thread: forwards each new sample to the telemetry receivers (one clone
/// per sample, off the control thread), drains the recording ring, and drains RT executor
/// diagnostics / E-stop acks / maintenance replies / orphaned persistence replies until
/// the supervisor is dropped.
fn dispatch_loop(
    hub: &crate::rt_core::publish::StateHub,
    tx: &broadcast::Sender<ArmObservation>,
    terminals: &TerminalBroadcaster,
    sinks: &std::sync::Mutex<Option<Arc<crate::rt_core::executor::OffThreadSinks>>>,
    tick_stats: &crate::diagnostics::tick::LockFreeTickStats,
    template_obs: &ArmObservation,
) {
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
    let drain_sinks = || {
        let s = sinks.lock().unwrap_or_else(|e| e.into_inner()).clone();
        if let Some(s) = s {
            s.drain_all(tick_stats);
        }
    };
    let mut last = 0;
    while !hub.is_closed() {
        drain_sinks();
        if let Some(ring) = hub.record_ring() { ring.drain(); }
        let mut new_obs: Option<ArmObservation> = None;
        if hub.published_seq() != last {
            if let Some(s) = hub.latest().filter(|s| s.meta.seq > last) {
                last = s.meta.seq;
                terminals.record_from_observation(&s.obs);
                new_obs = Some(s.obs.clone());
            }
        }
        // Drain every connection's bounded terminal FIFO into its higher-priority `ordinary_out_tx`
        // BEFORE forwarding telemetry so a newer telemetry snapshot can never overtake earlier
        // terminals on the wire.
        if terminals.has_pending() {
            let base = new_obs
                .clone()
                .or_else(|| hub.latest().map(|s| s.obs.clone()));
            terminals.drain_all(base.as_ref().unwrap_or(template_obs));
        }
        if let Some(obs) = new_obs {
            if terminals.pending_total() == 0 && tx.receiver_count() > 0 {
                let _ = tx.send(obs);
            }
        }
        std::thread::park_timeout(Duration::from_millis(50));
    }
    // Final drain on supervisor close so no retired/diagnostic/reply/terminal items are stranded,
    // and emit any final safety overflow summary.
    let s = sinks.lock().unwrap_or_else(|e| e.into_inner()).clone();
    if let Some(s) = s {
        s.flush_and_report_shutdown(tick_stats);
    }
    if let Some(ring) = hub.record_ring() { ring.drain(); }
    let latest_obs = hub.latest().map(|s| s.obs.clone());
    if let Some(obs) = latest_obs.as_ref() {
        terminals.record_from_observation(obs);
    }
    if terminals.has_pending() {
        terminals.drain_all(latest_obs.as_ref().unwrap_or(template_obs));
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
        tb.drain_all(&base_obs);
        assert_eq!(tb.pending_total(), 3, "failed enqueue must not pop terminal or drop subscriber");
        assert!(delivered.lock().unwrap().is_empty());

        // Once sink accepts again, all 3 terminals pop and deliver in exact FIFO order.
        accept.store(true, Ordering::SeqCst);
        tb.drain_all(&base_obs);
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
}
