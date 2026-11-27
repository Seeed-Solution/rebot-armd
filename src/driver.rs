use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::model::JointState;
pub use crate::model::{DriverCapabilities, DriverRangeCheck, EnableState, JointRangeReading, MotorZeroSupport, RangeCheckStatus};
use crate::protocol::damiao::{self, DamiaoLimits};
pub use crate::protocol::damiao::DAMIAO_GRIPPER_POSFORCE_VEL_RAW;
use crate::transport::{CanFrame, CanRxTime, CanTransport, TransportError, MAX_REALTIME_MONOTONIC_OFFSET_DRIFT_NS};
#[cfg(test)] use crate::transport::Transport;

pub mod ext;
pub mod zero_offset;
pub mod handshake;
pub mod bus_servo;
pub use bus_servo::FashionStarArmDriver;
pub use bus_servo::FeetechArmDriver;
pub mod can_servo;
pub use can_servo::RobStrideArmDriver;
pub use handshake::{TorqueOutcome, TorqueProgress};
pub use ext::{
    BoundedTxExt, BusRecoveryExt, CommissioningExt, JointModeExt, JointZeroExt, MitExt, ModeSwitchExt,
    NoExt, PosForceExt,
};
#[cfg(not(target_arch = "wasm32"))]
pub use ext::ShadowExt;
use handshake::{ConfirmWindow, Deadline, Service, TorqueDirection};

/// Stack-only, `Copy` hold target passed into `ModeSwitch` watchdog holds (zero heap alloc).
/// Preserves exact MIT `(position, velocity, kp, kd, torque)` parameters for axes verified in
/// `Impedance` mode while axes verified in `PositionVelocity` use `position`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ModeSwitchHoldTarget {
    pub position: f32,
    pub velocity: f32,
    pub kp: f32,
    pub kd: f32,
    pub torque: f32,
    pub is_mit: bool,
}

impl ModeSwitchHoldTarget {
    #[inline]
    pub const fn position(position: f32) -> Self {
        Self {
            position,
            velocity: 0.0,
            kp: 0.0,
            kd: 0.0,
            torque: 0.0,
            is_mit: false,
        }
    }

    #[inline]
    pub const fn mit(position: f32, velocity: f32, kp: f32, kd: f32, torque: f32) -> Self {
        Self {
            position,
            velocity,
            kp,
            kd,
            torque,
            is_mit: true,
        }
    }

    #[inline]
    pub fn to_joint_target(self) -> JointTarget {
        if self.is_mit {
            JointTarget::mit(self.position, self.velocity, self.kp, self.kd, self.torque)
        } else {
            JointTarget::position(self.position)
        }
    }
}

/// Desired control target for a single joint
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointTarget {
    /// Target position in radians
    pub position: f32,
    /// Target velocity in rad/s (optional)
    pub velocity: Option<f32>,
    /// Target feedforward torque in Nm (optional)
    pub torque: Option<f32>,
    /// Position stiffness gain Kp (optional, used in impedance/MIT control)
    pub kp: Option<f32>,
    /// Velocity damping gain Kd (optional, used in impedance/MIT control)
    pub kd: Option<f32>,
}

impl JointTarget {
    /// Create a pure position target
    pub fn position(position: f32) -> Self {
        Self {
            position,
            velocity: None,
            torque: None,
            kp: None,
            kd: None,
        }
    }

    /// Create an MIT/impedance control target
    pub fn mit(position: f32, velocity: f32, kp: f32, kd: f32, torque: f32) -> Self {
        Self {
            position,
            velocity: Some(velocity),
            torque: Some(torque),
            kp: Some(kp),
            kd: Some(kd),
        }
    }
}

/// Errors occurring at the unified driver layer
/// Transport failure kind stored inline in [`MitFenceTripReason`] (zero heap allocation).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TxFailKind {
    WouldBlock,
    Enobufs,
    Timeout,
    Other,
}

impl TxFailKind {
    #[inline]
    fn from_transport(e: &TransportError) -> Self {
        match e {
            TransportError::WouldBlock => Self::WouldBlock,
            TransportError::NoBufferSpace => Self::Enobufs,
            TransportError::Timeout => Self::Timeout,
            _ => Self::Other,
        }
    }
}

impl std::fmt::Display for TxFailKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::WouldBlock => f.write_str("Operation would block"),
            Self::Enobufs => f.write_str("Transmit queue full (ENOBUFS)"),
            Self::Timeout => f.write_str("Timeout error"),
            Self::Other => f.write_str("transport error"),
        }
    }
}

/// Inline zero-allocation reason that tripped the Damiao MIT deadline fence.
/// Formatted to a `String` only outside the RT tick (`Display` / `ArmDriver::mit_fence_trip`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MitFenceTripReason {
    FeedbackStale { joint: u8, age: Option<Duration>, deadline: Duration },
    OutputGap { gap: Duration, deadline: Duration },
    RxBacklog { frames: u16 },
    EstopBeforeBatch { total_joints: u8 },
    EstopDuringBatch { completed_joints: u8, total_joints: u8 },
    EstopBetweenAttempts { joint: u8, completed_joints: u8, total_joints: u8 },
    EstopBeforeFlush { total_joints: u8 },
    EstopAfterFlush { total_joints: u8 },
    TxBackpressuredBeforeBatch,
    TxBudgetExceededPreflight { needed: u16, remaining: u16, max: u16 },
    TxBudgetDeadlineBeforeBatch,
    TxBudgetExhausted { completed_joints: u8, total_joints: u8 },
    TargetUnavailable { joint: u8, completed_joints: u8, total_joints: u8 },
    TransmitFailed { joint: u8, completed_joints: u8, total_joints: u8, kind: TxFailKind },
    FlushFailed { kind: TxFailKind },
    PreflightRejected,
}

impl std::fmt::Display for MitFenceTripReason {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match *self {
            Self::FeedbackStale { joint, age, deadline } => {
                write!(f, "joint {joint} feedback age {age:?} reached {deadline:?}")
            }
            Self::OutputGap { gap, deadline } => {
                write!(f, "gap between dynamic outputs {gap:?} reached {deadline:?}")
            }
            Self::RxBacklog { frames } => {
                write!(f, "RX backlog: queue not drained after {frames} frames")
            }
            Self::EstopBeforeBatch { total_joints } => {
                write!(f, "E-stop before the batch (0 of {total_joints} joints)")
            }
            Self::EstopDuringBatch { completed_joints, total_joints } => {
                write!(f, "E-stop during the batch after {completed_joints} of {total_joints} joints")
            }
            Self::EstopBetweenAttempts { joint, completed_joints, total_joints } => {
                write!(f, "E-stop between transmit attempts to joint {joint} after {completed_joints} of {total_joints} joints")
            }
            Self::EstopBeforeFlush { total_joints } => {
                write!(f, "E-stop before target batch flush after {total_joints} of {total_joints} joints")
            }
            Self::EstopAfterFlush { total_joints } => {
                write!(f, "E-stop during final transmit or flush after {total_joints} of {total_joints} joints")
            }
            Self::TxBackpressuredBeforeBatch => {
                f.write_str("TX pass already backpressured before MIT batch")
            }
            Self::TxBudgetExceededPreflight { needed, remaining, max } => {
                write!(f, "MIT batch needs {needed} frames, exceeds remaining TX budget ({remaining} of {max})")
            }
            Self::TxBudgetDeadlineBeforeBatch => {
                f.write_str("TX budget deadline expired before MIT batch")
            }
            Self::TxBudgetExhausted { completed_joints, total_joints } => {
                write!(f, "TX budget exhausted after {completed_joints} of {total_joints} joints")
            }
            Self::TargetUnavailable { joint, completed_joints, total_joints } => {
                write!(f, "joint {joint} target unavailable after {completed_joints} of {total_joints} joints")
            }
            Self::TransmitFailed { joint, completed_joints, total_joints, kind } => {
                write!(f, "transmit to joint {joint} failed after {completed_joints} of {total_joints} joints: {kind}")
            }
            Self::FlushFailed { kind } => {
                write!(f, "flush failed: {kind}")
            }
            Self::PreflightRejected => {
                f.write_str("target batch rejected at preflight")
            }
        }
    }
}

#[derive(Debug, Error)]
pub enum ArmDriverError {
    #[error("Transport error: {0}")]
    Transport(#[from] TransportError),

    #[error("Protocol error: {0}")]
    Protocol(String),

    #[error("Protocol error: {0}")]
    ProtocolStatic(&'static str),

    #[error("Protocol error: ambiguous duplicate 1-byte register reply from servo {0}")]
    AmbiguousDuplicateReply(u8),

    #[error("Hardware fault: Feetech torque-enable readback failed ({error_code}) on servo {servo_id}")]
    FeetechReadbackFailed { servo_id: u8, error_code: &'static str },

    #[error("Joint count mismatch: expected {expected}, got {actual}")]
    JointCountMismatch { expected: usize, actual: usize },

    #[error("Hardware fault: {0}")]
    HardwareFault(String),

    #[error("Timeout waiting for response from joint {0}")]
    Timeout(u8),

    #[error("Transmit deferred by TX budget or queue backpressure ({pending} frames queued)")]
    TxDeferred { pending: usize },

    #[error("{ESTOP_INHIBITED}")]
    EstopInhibited,

    #[error("motion target dropped: lease generation fenced")]
    LeaseFenced,

    #[error("MIT deadline fence tripped: {0}")]
    MitFenceTripped(MitFenceTripReason),

    #[error("torque transition pending: no target before its confirmation")]
    TorqueTransitionPending,

    #[error("pending TX drain is disabled while the MIT fence is active")]
    PendingTxDisabledUnderMitFence,

    #[error("joint {} position-force mode not confirmed: capped operation unavailable, no frame sent", joint + 1)]
    PosForceModeUnconfirmed { joint: usize },

    #[error("joint {} position-force feedback stale: holding (no motion), no frame sent", joint + 1)]
    PosForceFeedbackStale { joint: usize },

    #[error("joint {} position-force feedback stale before transmit", joint + 1)]
    PosForceFeedbackStaleBeforeTransmit { joint: usize },

    #[error("joint {} position-force feedback stale or target invalid, no frame sent", joint + 1)]
    PosForceTargetUnavailable { joint: usize },

    #[error("joint {} position-force target is not finite, no frame sent", joint + 1)]
    PosForceTargetNotFinite { joint: usize },

    #[error("joint {} position-force speed encodes to 0 on the wire, no frame sent", joint + 1)]
    PosForceSpeedZero { joint: usize },

    #[error("joint {} CTRL_MODE readback: request not sent: {kind}; capped operation unavailable", joint + 1)]
    PosForceReadbackTxFailed { joint: usize, kind: TxFailKind },

    #[error("joint {} CTRL_MODE readback: reads {observed}, expected {expected}; capped operation unavailable", joint + 1)]
    CtrlModeReadbackMismatch { joint: usize, expected: u32, observed: u32 },

    #[error("joint {} CTRL_MODE readback: no fresh reply within {timeout_ms} ms; capped operation unavailable", joint + 1)]
    CtrlModeReadbackMissing { joint: usize, timeout_ms: u64 },

    #[error("joint {} CTRL_MODE readback: ambiguous or conflicting reply; capped operation unavailable", joint + 1)]
    CtrlModeReadbackAmbiguous { joint: usize },

    #[error("joint {} CTRL_MODE readback: no fresh measured position for pre-enable hold; capped operation unavailable", joint + 1)]
    PosForcePreEnableRefreshMissing { joint: usize },

    #[error("joint {} CTRL_MODE readback: pre-enable refresh not sent: {kind}; capped operation unavailable", joint + 1)]
    PosForcePreEnableRefreshTxFailed { joint: usize, kind: TxFailKind },

    #[error("joint {} CTRL_MODE readback: pre-enable hold refused: needs stationary torque-off feedback (still {still}, torque off {off}, status {status}); capped operation unavailable", joint + 1)]
    PosForcePreEnableHoldUnsafe { joint: usize, still: bool, off: bool, status: u8 },

    #[error("joint {} CTRL_MODE readback: pre-enable hold frame unavailable; capped operation unavailable", joint + 1)]
    PosForcePreEnableHoldUnavailable { joint: usize },

    #[error("joint {} TIMEOUT readback mismatch: expected {expected} counts, observed {observed} counts", joint + 1)]
    TimeoutReadbackMismatch { joint: usize, expected: u32, observed: u32 },

    #[error("joint {} TIMEOUT readback: no fresh reply within {timeout_ms} ms", joint + 1)]
    TimeoutReadbackMissing { joint: usize, timeout_ms: u64 },

    #[error("joint {} TIMEOUT readback: ambiguous or conflicting reply", joint + 1)]
    TimeoutReadbackAmbiguous { joint: usize },

    #[error("joint {} TIMEOUT readback: request not sent: {kind}", joint + 1)]
    TimeoutReadbackTxFailed { joint: usize, kind: TxFailKind },

    #[error("joint {} CTRL_MODE readback: pre-enable hold not sent: TX budget exhausted; capped operation unavailable", joint + 1)]
    PosForcePreEnableHoldBudgetExhausted { joint: usize },

    #[error("joint {} CTRL_MODE readback: pre-enable hold not sent: {kind}; capped operation unavailable", joint + 1)]
    PosForcePreEnableHoldTxFailed { joint: usize, kind: TxFailKind },

    #[error("TX pending queue capacity ({capacity}) exceeded (queued {queued}, new {new})")]
    TxQueueFull { queued: usize, new: usize, capacity: usize },

    #[error("Torque stays on: arm is not proven at resting pose (TORQUE_OFF_WOULD_DROP)")]
    TorqueOffWouldDrop,

    #[error("Torque disable request expired after 1 s while stopped (TORQUE_OFF_WOULD_DROP)")]
    DisableRequestExpired,

    #[error("Hardware fault: mapping range check unverified before budgeted enable")]
    RangeCheckUnverified,

    #[error("WATCHDOG_BUDGET_INSUFFICIENT: tx_budget.max_frames ({max_frames}) < required ({required}) for energized watchdog continuity")]
    WatchdogBudgetInsufficient { max_frames: usize, required: usize },

    #[error("WATCHDOG_CONTINUITY_ABORT: failed to service due continuity hold on joint {} within release budget", joint + 1)]
    WatchdogContinuityMissed { joint: usize },

    #[error("Hardware fault: mapping range mismatch before enable")]
    RangeCheckMismatch,

    #[error("Hardware fault: CAN RX buffer not drained before handshake")]
    RxDrainTimeout,

    #[error(
        "Hardware fault: Not all Damiao motors entered enabled state within {} ms: feedback_valid={feedback_valid}, status={:?}",
        DAMIAO_ENABLE_CONFIRM.as_millis(),
        &statuses[..usize::from(*dof)]
    )]
    EnableTimeout {
        feedback_valid: bool,
        statuses: [u8; crate::rt_core::mailbox::MAX_DOF],
        dof: u8,
    },

    #[error(
        "Hardware fault: Joint {joint} (CAN ID 0x{motor_id:X}) reported fault: {}",
        damiao_fault_name(*status).unwrap_or("Motor Hardware Fault")
    )]
    MotorHardwareFault {
        joint: usize,
        motor_id: u16,
        status: u8,
    },

    #[error("Hardware fault: Disable failed to complete; the MIT fence stays")]
    DisableMitFenceUnconfirmed,

    #[error("Enable authority expired before hardware transmit")]
    EnableAuthorityExpired,

    #[error(
        "Hardware fault: joint {} pre-enable freshen failed: no fresh measured position within {} ms (needs fresh stationary feedback: fresh false)",
        joint + 1,
        ENABLE_FRESHEN_TIMEOUT.as_millis()
    )]
    EnableFreshenTimeout { joint: usize },

    #[error(
        "Hardware fault: joint {} pre-enable freshen refused: needs fresh stationary feedback with torque off (fresh true, still {still}, torque off {off}, status {status}); capped operation unavailable",
        joint + 1
    )]
    EnableFreshenUnsafeGripper {
        joint: usize,
        still: bool,
        off: bool,
        status: u8,
    },

    #[error("{}", format_partial_disable(*reason, *applied_mask, *uncertain_mask, *remaining_mask, *dof))]
    PartialDisable {
        reason: PartialDisableReason,
        applied_mask: u64,
        uncertain_mask: u64,
        remaining_mask: u64,
        dof: u8,
    },

    #[error(
        "Hardware fault: joint {} watchdog continuity abort: no control frame transmitted for {} ms during enable (fail-closed before Damiao TIMEOUT expiry)",
        joint + 1,
        age_ms
    )]
    WatchdogContinuityAbort { joint: usize, age_ms: u64 },
}

/// Reason a multi-tick torque-disable sequence aborted after dispatching one or more motors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PartialDisableReason {
    Estop,
    Stopped,
    LeaseLost,
    DeadlineExpired,
    MotionBeyondRest { joint: u8 },
    TransportError,
}

fn write_joint_mask_list(f: &mut std::fmt::Formatter<'_>, mask: u64, dof: u8) -> std::fmt::Result {
    f.write_str("[")?;
    let mut first = true;
    let limit = usize::from(dof).min(64);
    for i in 0..limit {
        if (mask & (1u64 << i)) != 0 {
            if !first {
                f.write_str(", ")?;
            }
            first = false;
            write!(f, "J{}", i + 1)?;
        }
    }
    f.write_str("]")
}

struct PartialDisableDisplay {
    reason: PartialDisableReason,
    applied_mask: u64,
    uncertain_mask: u64,
    remaining_mask: u64,
    dof: u8,
}

impl std::fmt::Display for PartialDisableDisplay {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.reason {
            PartialDisableReason::Estop => f.write_str("Torque disable aborted (E-stop mid-sequence); applied ")?,
            PartialDisableReason::Stopped => f.write_str("Torque disable aborted (stop mid-sequence); applied ")?,
            PartialDisableReason::LeaseLost => f.write_str("Torque disable aborted (lease lost mid-sequence); applied ")?,
            PartialDisableReason::DeadlineExpired => f.write_str("Torque disable aborted (deadline expired mid-sequence); applied ")?,
            PartialDisableReason::MotionBeyondRest { joint } => {
                write!(f, "Torque disable aborted (J{} moved beyond rest tolerance); applied ", usize::from(joint) + 1)?;
            }
            PartialDisableReason::TransportError => f.write_str("Torque disable aborted (transport error mid-sequence); applied ")?,
        }
        write_joint_mask_list(f, self.applied_mask, self.dof)?;
        f.write_str(", uncertain ")?;
        write_joint_mask_list(f, self.uncertain_mask, self.dof)?;
        f.write_str(", not attempted ")?;
        write_joint_mask_list(f, self.remaining_mask, self.dof)
    }
}

fn format_partial_disable(
    reason: PartialDisableReason,
    applied_mask: u64,
    uncertain_mask: u64,
    remaining_mask: u64,
    dof: u8,
) -> PartialDisableDisplay {
    PartialDisableDisplay {
        reason,
        applied_mask,
        uncertain_mask,
        remaining_mask,
        dof,
    }
}

/// Maximum feedback age accepted at the transmit choke point for proving the arm is at
/// the rest pose before transmitting `DisableRequested` while stopped.
pub const STOPPED_DISABLE_FEEDBACK_MAX_AGE: Duration = Duration::from_millis(100);

/// Single bounded total lifetime of a `DisableRequested` request while stopped, checked
/// at send time in the transmit choke point from the request's `started_at`.
pub const STOPPED_DISABLE_REQUEST_LIFETIME: Duration = Duration::from_secs(1);

/// Exhaustive reasons that authorize a protective torque disable bypassing the stopped
/// rest-pose rule and 1 s request lifetime.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DisableProtectiveReason {
    /// Over-temperature protection (`StopPolicy::ThermalTrip`).
    Thermal,
    /// Actual process exit / panic / forced teardown disable.
    ProcessExit,
}

/// Intent of a hardware torque-disable operation, enforced at the driver transmit choke point.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DisableKind {
    /// User, program, operational TorqueOff, MIT recovery, or fault-clear disable: unconditionally
    /// gated at send time in EVERY safety state (operational and stopped alike) by the K=3
    /// rest-pose gate from fresh (`<= 100 ms`) feedback plus the single 1 s request lifetime from
    /// `started_at`.
    DisableRequested {
        /// Request start time on the driver's clock (`ArmDriver::clock_now`).
        started_at: Instant,
    },
    /// Protective disable exempt from the rest-pose gate and 1 s request lifetime:
    /// used ONLY by `Thermal` and `ProcessExit`.
    DisableProtective {
        reason: DisableProtectiveReason,
    },
}

/// Driver-side stopped-disable policy injected by the supervisor so every driver transmit
/// choke point enforces Rule A′ at send time without locks or heap allocations.
#[derive(Clone, Debug)]
pub struct StoppedDisableGate {
    /// Atomic safety-stopped signal (`SafetyState::is_stopped() || has_latched_fault()`).
    pub safety_stopped: Arc<AtomicBool>,
    /// Whether the active profile has a calibrated `resting_pose` configured.
    pub has_resting_pose: bool,
    /// Arm joint indices checked by Rule A′ (excluding the gripper), inline up to `MAX_DOF`.
    pub arm_joints: [usize; crate::rt_core::mailbox::MAX_DOF],
    pub arm_joints_len: usize,
    /// Rest pose positions indexed by joint index (shifted into motor coordinates by
    /// `ZeroOffsetDriver` for hardware drivers whose internal cache is in raw motor space).
    pub rest_positions: [f32; crate::rt_core::mailbox::MAX_DOF],
    /// Per-joint tolerance around `rest_positions` (rad).
    pub rest_pose_tolerance_rad: f32,
}

impl StoppedDisableGate {
    #[inline]
    pub fn is_safety_stopped(&self) -> bool {
        self.safety_stopped.load(Ordering::SeqCst)
    }

    #[inline]
    pub fn arm_joint_indices(&self) -> &[usize] {
        &self.arm_joints[..self.arm_joints_len]
    }

    /// Returns a copy with `rest_positions[i]` shifted into motor coordinates (`rest + offset`)
    /// so inner drivers comparing raw cached motor positions match calibrated joint-space rest.
    pub fn with_motor_offsets(&self, offsets: &[f32]) -> Self {
        let mut out = self.clone();
        for (i, slot) in out.rest_positions.iter_mut().enumerate() {
            let o = offsets.get(i).copied().unwrap_or(0.0);
            if o != 0.0 {
                *slot += o;
            }
        }
        out
    }

    #[inline]
    pub fn is_arm_joint(&self, joint: usize) -> bool {
        self.arm_joints[..self.arm_joints_len].contains(&joint)
    }

    #[inline]
    pub fn joint_at_rest(&self, joint: usize, position: f32) -> bool {
        if !self.has_resting_pose
            || !self.rest_pose_tolerance_rad.is_finite()
            || self.rest_pose_tolerance_rad <= 0.0
            || joint >= self.rest_positions.len()
        {
            return false;
        }
        let rest = self.rest_positions[joint];
        position.is_finite() && rest.is_finite() && (position - rest).abs() <= self.rest_pose_tolerance_rad
    }

    /// True when `position` satisfies the rest-pose predicate for motor `joint`:
    /// - Arm joints (`is_arm_joint(joint)`, J1–J6): finite position within
    ///   `rest_pose_tolerance_rad` of `rest_positions[joint]`.
    /// - Non-arm motors (e.g., J7 gripper): excluded from the rest-pose disable gate
    ///   (`evaluate_rest_pose_gate` checks `arm_joint_indices()` only); this helper still treats
    ///   any finite jaw position as rest when queried directly.
    #[inline]
    pub fn motor_at_rest(&self, joint: usize, position: f32) -> bool {
        if self.is_arm_joint(joint) {
            self.joint_at_rest(joint, position)
        } else {
            self.has_resting_pose && position.is_finite()
        }
    }

    /// Evaluates Rule A′ pose check against fresh arm-joint positions matching `arm_joint_indices()`.
    #[inline]
    pub fn check_positions(&self, positions: &[f32]) -> bool {
        if !self.has_resting_pose
            || !self.rest_pose_tolerance_rad.is_finite()
            || self.rest_pose_tolerance_rad <= 0.0
            || self.arm_joints_len == 0
            || positions.len() < self.arm_joints_len
        {
            return false;
        }
        for (&joint_idx, &pos) in self
            .arm_joints[..self.arm_joints_len]
            .iter()
            .zip(positions.iter())
        {
            if joint_idx >= self.rest_positions.len() {
                return false;
            }
            let rest = self.rest_positions[joint_idx];
            if !pos.is_finite()
                || !rest.is_finite()
                || (pos - rest).abs() > self.rest_pose_tolerance_rad
            {
                return false;
            }
        }
        true
    }
}

impl ArmDriverError {
    /// Writes this error's exact `Display` output into `out` without heap allocation.
    ///
    /// Lives alongside [`ArmDriverError`] and delegates to its `Display` implementation so every
    /// variant (including future variants) renders identically to API/log text without a second
    /// match table to keep in sync.
    pub fn write_bounded<const N: usize>(&self, out: &mut crate::safety::BoundedText<N>) {
        match self {
            Self::Transport(TransportError::Io(io_err)) => {
                if let Some(code) = io_err.raw_os_error() {
                    let _ = core::fmt::write(
                        out,
                        format_args!(
                            "Transport error: I/O error: {}",
                            crate::transport::format_os_error(Some(code), io_err.kind())
                        ),
                    );
                    return;
                }
                let _ = core::fmt::write(out, format_args!("{self}"));
            }
            _ => {
                let _ = core::fmt::write(out, format_args!("{self}"));
            }
        }
    }

    /// Converts this error into a bounded, allocation-free [`crate::safety::ClearFaultDriverError`]
    /// whose off-thread `Display` output matches [`ArmDriverError`]'s `Display` verbatim.
    pub fn to_clear_fault_error(&self) -> crate::safety::ClearFaultDriverError {
        use crate::safety::{BoundedText, ClearFaultDriverError as C};
        match self {
            Self::Transport(TransportError::Timeout) => C::TransportTimeout,
            Self::Transport(TransportError::WouldBlock) => C::TransportWouldBlock,
            Self::Transport(TransportError::NotOpen) => C::TransportNotOpen,
            Self::Transport(TransportError::ChecksumMismatch { expected, actual }) => {
                C::TransportChecksumMismatch { expected: *expected, actual: *actual }
            }
            Self::Transport(TransportError::FrameTooShort { expected, actual }) => {
                C::TransportFrameTooShort { expected: *expected, actual: *actual }
            }
            Self::JointCountMismatch { expected, actual } => {
                C::JointCountMismatch { expected: *expected, actual: *actual }
            }
            Self::Timeout(id) => C::JointTimeout(*id),
            Self::EstopInhibited => C::EstopInhibited,
            Self::Protocol(s) => C::Protocol(BoundedText::new(s)),
            Self::ProtocolStatic(s) => C::Protocol(BoundedText::new(s)),
            Self::HardwareFault(s) => C::HardwareFault(BoundedText::new(s)),
            other => {
                let mut buf = BoundedText::empty();
                other.write_bounded(&mut buf);
                C::Formatted(buf)
            }
        }
    }
}

/// Outcome of a bounded target write ([`ArmDriver::write_joint_targets_bounded`]) or pending
/// queue drain ([`DamiaoArmDriver::drain_pending_tx`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TxBatchOutcome {
    /// Every frame of the batch (and any carried-over pending frames) was transmitted and flushed
    /// with zero expired or deferred frames.
    Complete,
    /// Queue backpressure (`WouldBlock` / `ENOBUFS`) or the per-tick TX budget stopped the pass;
    /// `pending` fresh frames remain queued for the next tick. Callers must not treat this as
    /// accepted/completed motion output.
    Deferred { pending: usize },
    /// One or more queued frames expired (`age >= frame_max_age`) and were dropped instead of
    /// being transmitted; `pending` fresher frames (if any) remain queued. Never reported as
    /// `Complete`.
    Expired { dropped: usize, pending: usize },
}

/// Default maximum CAN transmit attempts started in one tick (shared across RX recovery/refresh,
/// J7 handshake TX, and target writes; safety disable / fault-clear frames are exempt).
pub const DAMIAO_TX_MAX_FRAMES: usize = 16;
/// Default cutoff elapsed time for starting new CAN transmit attempts in one tick (and any
/// non-no-op transport `flush()`). An in-flight `transmit()` started before this cutoff may
/// complete after it.
///
/// Sized from real SG2002 B601-DM measurements (`qual-smoke-20261124T000133Z`, `1.6.0-dev11` /
/// `df7487d`, `can0` `mcp251xfd` over SPI at 1 Mbit/s, `SCHED_FIFO` 45, ERROR-ACTIVE, `berr 0/0`,
/// `bus-off 0`):
/// - `tx_us`: p50 3.915 ms, p99 8.123 ms, max 13.383 ms
/// - `tx.send_us` (1000-tick window): p50 3.622 ms, p99 7.763 ms, max 13.184 ms
///   (`send_cpu_us`: p50 0.593 ms, p99 0.757 ms, max 1.259 ms)
/// - `tx.frame_max_us`: p50 1.058 ms, p99 6.104 ms, window max 11.554 ms
///   (`max_tx` peak frame 11.381 ms)
/// - `max_tick`: 7 target frames in `tx_send_us` 3.114 ms (`tx_frame_max_us` 0.994 ms)
///
/// The previous 5 ms cutoff deferred healthy 7-motor + refresh batches under normal SPI
/// preemption (`SCHED_FIFO` 45 vs SPI IRQ 50). 10 ms is a frame-start cutoff sized for the
/// common 7-target + 1-refresh case (~8 ms with margin inside the 20 ms release period); an
/// isolated >10 ms in-flight SPI tail completes cleanly and defers any remaining unstarted
/// frames to the next tick (bounded by `DAMIAO_TX_FRAME_MAX_AGE` = 45 ms). Overridable per
/// profile via `ArmConfig::tx_budget_ms`.
pub const DAMIAO_TX_MAX_DURATION: Duration = Duration::from_millis(10);
/// Publication margin reserved at the end of a control release when bounding the per-tick CAN TX
/// start-cutoff (`tx_deadline = min(first_tx + tx_budget, release_deadline - margin)`).
///
/// Measured on SG2002 B601-DM (`qual-smoke-20261124T000133Z`, `1.6.0-dev11`, `SCHED_FIFO` 45,
/// 1000-tick window): `finish_us` p50 7 µs / p99 298 µs, `obs_us` p50 751 µs / p99 1,257 µs,
/// `publish_us` p50 59 µs / p99 116 µs (combined p50 0.817 ms, p99 1.671 ms → 2 ms margin).
pub const DAMIAO_TX_RELEASE_PUBLISH_MARGIN: Duration = Duration::from_millis(2);
/// Default inclusive maximum age of a queued target frame before it is dropped as stale
/// (`age >= frame_max_age` drops).
pub const DAMIAO_TX_FRAME_MAX_AGE: Duration = Duration::from_millis(45);
/// Fixed inline capacity of the Damiao driver's pending CAN TX queue (zero heap allocation;
/// holds up to 16 dual-mode joints).
pub const DAMIAO_PENDING_TX_CAPACITY: usize = 32;

/// Per-tick CAN transmit budget for [`DamiaoArmDriver`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TxBudget {
    /// Maximum CAN transmit attempts started in one tick.
    pub max_frames: usize,
    /// Cutoff elapsed time for starting new CAN transmit attempts in one tick (not a syscall
    /// interrupt: the final in-flight transmit or flush may finish past this duration).
    pub max_duration: Duration,
    /// Inclusive maximum age of a queued motion frame before expiry (`age >= frame_max_age`).
    pub frame_max_age: Duration,
}

impl Default for TxBudget {
    fn default() -> Self {
        Self {
            max_frames: DAMIAO_TX_MAX_FRAMES,
            max_duration: DAMIAO_TX_MAX_DURATION,
            frame_max_age: DAMIAO_TX_FRAME_MAX_AGE,
        }
    }
}

/// Message of the error `write_joint_targets` returns when the E-stop gate stopped the
/// batch (an E-stop request arrived during the transmit; the tick applies it at once).
pub const ESTOP_INHIBITED: &str = "E-stop requested during the target batch; remaining frames not sent";

/// Universal robotic arm hardware driver trait
/// Name of a Damiao feedback state code that reports a fault (8 and above); None for
/// the normal states (0 disabled, 1 enabled).
/// The error is the E-stop gate's refusal ([`ESTOP_INHIBITED`]).
pub fn is_estop_inhibited(e: &ArmDriverError) -> bool {
    matches!(e, ArmDriverError::EstopInhibited)
        || matches!(e, ArmDriverError::Protocol(m) if m == ESTOP_INHIBITED)
        || matches!(e, ArmDriverError::ProtocolStatic(m) if *m == ESTOP_INHIBITED)
}

pub fn damiao_fault_name(code: u8) -> Option<&'static str> {
    Some(match code {
        0..=7 => return None,
        8 => "Overvoltage",
        9 => "Undervoltage",
        0xA => "Overcurrent",
        0xB => "MOS Overtemperature",
        0xC => "Rotor Overtemperature",
        0xD => "CAN Communication Loss",
        0xE => "Overload",
        _ => "Motor Hardware Fault",
    })
}

/// Damiao enable handshake: how long to wait for every motor to report state 1, and when to resend the
/// enable frame to motors not yet confirmed.
pub const DAMIAO_ENABLE_CONFIRM: Duration = Duration::from_millis(150);
/// Resend times of the enable frame to motors not yet confirmed.
const DAMIAO_ENABLE_RETRY_AT: [Duration; 2] = [Duration::from_millis(25), Duration::from_millis(75)];
/// Number of bounded `CMD_DISABLE` transmit passes sent per accepted Damiao disable (initial +
/// 2 repeats within the single 1 s request lifetime; no 150 ms early-abort confirmation deadline).
pub const DAMIAO_DISABLE_REPEATS: u8 = 3;
/// Spacing between bounded `CMD_DISABLE` repeats within the 1 s request lifetime.
pub const DAMIAO_DISABLE_REPEAT_INTERVAL: Duration = Duration::from_millis(20);
/// Causal settle window inside the rest-pose gate for CAN drivers (Damiao, RobStride): after a
/// motion command or enable write returns from `transmit()`, frames whose kernel/transport receive
/// time is not strictly greater than `write_done + TORQUE_CAUSAL_SETTLE_WINDOW` are never evidence
/// for the rest-pose gate.
pub const TORQUE_CAUSAL_SETTLE_WINDOW: Duration = Duration::from_millis(20);
/// Number of consecutive agreeing causal samples (distinct control releases) required before
/// `"at rest pose"` is confirmed by the rest-pose gate. A single sample never confirms.
pub const EVIDENCE_WINDOW_SAMPLES: u8 = 3;
/// Minimum time span between the 1st and K-th causal sample of the rest-pose evidence window
/// (spanning at least 2 control ticks at 50 Hz = 20 ms; prevents same-tick bursts from confirming).
pub const EVIDENCE_WINDOW_MIN_SPAN: Duration = Duration::from_millis(20);
/// Spacing between read-only causal status probes while awaiting the rest-pose gate.
const EVIDENCE_PROBE_INTERVAL: Duration = Duration::from_millis(10);
/// Time bound of one Damiao receive pass (with at most 64 frames).
pub const DAMIAO_RX_PASS: Duration = Duration::from_millis(5);
/// Maximum age for Damiao joint feedback to still be considered valid.
const DAMIAO_FEEDBACK_VALID_WINDOW: Duration = Duration::from_millis(250);

/// Zero-allocation per-pass rest compliance check on not-yet-disabled arm joints once K=3 rest
/// evidence was established before the first disable frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RemainingRestStatus {
    AllAtRest,
    NeedFreshFeedback,
    MovedBeyondRest { joint: usize },
}

/// Zero-allocation status of a driver's K=3 causal rest-pose evidence window (S2 / L1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RestPoseGateStatus {
    /// Completed RX drain and every arm joint has K=3 consecutive causal samples spanning >= 2
    /// control ticks within the rest-pose tolerance.
    Confirmed,
    /// No arm joint has been observed outside the rest pose; confirmation is waiting ONLY on
    /// completing an exhausted RX drain / poisoned line drain OR accumulating the remaining
    /// K=3 consecutive causal rest-pose samples across >= 2 control ticks. Retryable within
    /// the 1 s stopped-disable request lifetime (L1).
    PendingDrainOrWindow,
    /// RX drain is complete and no arm joint disagrees with rest, yet at least one arm joint
    /// lacks fresh (<= 100 ms) feedback (silent/stale bus).
    StaleOrMissing,
    /// Missing rest-pose configuration, non-finite feedback, or a real non-rest pose was observed.
    RefusedNonRest,
}

/// Result of feeding one agreeing rest-pose sample into [`CausalWindow`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum WindowRecord {
    Accepted,
    IgnoredOlder,
    OffsetDrifted,
}

/// Fixed capacity for distinct-release timestamps tracked inside [`CausalWindow`] (zero-alloc).
const CAUSAL_WINDOW_CAP: usize = 32;

/// Zero-allocation K=3 consecutive causal rest-pose evidence accumulator shared across all drivers.
///
/// Confirmation requires at least `EVIDENCE_WINDOW_SAMPLES` (3) **distinct control releases**
/// whose timestamps ALL lie within `max_age` of `now`, spanning at least
/// `EVIDENCE_WINDOW_MIN_SPAN` (20 ms) from the oldest still-fresh sample to the newest sample.
/// Multiple queued frames within the same control release update the current release's timestamp
/// without incrementing the distinct-release count. Any disagreeing sample, gap `> max_gap`, or
/// `CLOCK_REALTIME - CLOCK_MONOTONIC` offset change `> 1 ms` resets the window.
#[derive(Clone, Copy, Debug)]
struct CausalWindow {
    len: u8,
    samples: [Option<Instant>; CAUSAL_WINDOW_CAP],
    last_release: u64,
    anchor_offset_ns: Option<i128>,
}

impl Default for CausalWindow {
    fn default() -> Self {
        Self {
            len: 0,
            samples: [None; CAUSAL_WINDOW_CAP],
            last_release: 0,
            anchor_offset_ns: None,
        }
    }
}

impl CausalWindow {
    #[inline]
    fn reset(&mut self) {
        *self = Self::default();
    }

    #[inline]
    fn last_rx(&self) -> Option<Instant> {
        self.len.checked_sub(1).and_then(|i| self.samples[usize::from(i)])
    }

    #[inline]
    fn record_agree(
        &mut self,
        rx_at: Instant,
        release_id: u64,
        max_gap: Duration,
        offset_ns: Option<i128>,
    ) -> WindowRecord {
        if self.len > 0 {
            match (self.anchor_offset_ns, offset_ns) {
                (Some(anchor), Some(cur))
                    if (cur - anchor).abs() > MAX_REALTIME_MONOTONIC_OFFSET_DRIFT_NS =>
                {
                    self.reset();
                    return WindowRecord::OffsetDrifted;
                }
                (Some(_), None) | (None, Some(_)) => {
                    self.reset();
                    return WindowRecord::OffsetDrifted;
                }
                _ => {}
            }
        }
        if self.last_rx().is_some_and(|prev| rx_at < prev) {
            return WindowRecord::IgnoredOlder;
        }
        if self.len == 0
            || self
                .last_rx()
                .is_some_and(|prev| rx_at.saturating_duration_since(prev) > max_gap)
        {
            self.samples = [None; CAUSAL_WINDOW_CAP];
            self.samples[0] = Some(rx_at);
            self.len = 1;
            self.last_release = release_id;
            self.anchor_offset_ns = offset_ns;
        } else if release_id == self.last_release {
            // Same control release: multiple queued samples never count as > 1 tick.
            self.samples[usize::from(self.len - 1)] = Some(rx_at);
        } else {
            // Evict any prefix samples already older than `max_gap` relative to `rx_at`:
            // timestamps are monotonically non-decreasing, so aged-out entries can never be fresh.
            let len = usize::from(self.len);
            let mut keep_from = 0usize;
            while keep_from < len {
                if self.samples[keep_from]
                    .is_some_and(|ts| rx_at.saturating_duration_since(ts) > max_gap)
                {
                    keep_from += 1;
                } else {
                    break;
                }
            }
            if keep_from > 0 {
                let remaining = len - keep_from;
                for i in 0..remaining {
                    self.samples[i] = self.samples[keep_from + i];
                }
                for i in remaining..len {
                    self.samples[i] = None;
                }
                self.len = remaining as u8;
            }
            // If > CAUSAL_WINDOW_CAP distinct releases arrived inside `max_gap`, drop an interior
            // entry (`samples[1]`) so the oldest fresh timestamp (`samples[0]`) and all newest
            // entries remain exact without falsely shrinking the span.
            if usize::from(self.len) == CAUSAL_WINDOW_CAP {
                for i in 1..(CAUSAL_WINDOW_CAP - 1) {
                    self.samples[i] = self.samples[i + 1];
                }
                self.samples[CAUSAL_WINDOW_CAP - 1] = None;
                self.len -= 1;
            }
            let idx = usize::from(self.len);
            self.samples[idx] = Some(rx_at);
            self.len += 1;
            self.last_release = release_id;
        }
        WindowRecord::Accepted
    }

    #[inline]
    fn record_disagree(&mut self) {
        self.reset();
    }

    #[inline]
    fn is_confirmed(&self, now: Instant, max_age: Duration) -> bool {
        let len = usize::from(self.len);
        if len < usize::from(EVIDENCE_WINDOW_SAMPLES) {
            return false;
        }
        let Some(last) = self.samples[len - 1] else {
            return false;
        };
        if last > now || now.saturating_duration_since(last) > max_age {
            return false;
        }
        let mut first_fresh_idx = None;
        for i in 0..len {
            if let Some(ts) = self.samples[i] {
                if ts <= now && now.saturating_duration_since(ts) <= max_age {
                    first_fresh_idx = Some(i);
                    break;
                }
            }
        }
        let Some(start) = first_fresh_idx else {
            return false;
        };
        let fresh_count = len - start;
        let Some(first_fresh) = self.samples[start] else {
            return false;
        };
        fresh_count >= usize::from(EVIDENCE_WINDOW_SAMPLES)
            && last.saturating_duration_since(first_fresh) >= EVIDENCE_WINDOW_MIN_SPAN
    }
}

/// Per-joint hold status reported by measured-position hold servicing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointHoldStatus {
    /// Hold intentionally skipped by owner decision (`torque_intent == Off` or omitted from `hold_mask`).
    SkippedOff,
    /// Motor has fresh measured feedback and a verified hold mode; hold target is queued or
    /// pending transmission (not yet confirmed transmitted/flushed on the bus).
    Queued,
    /// Motor has fresh measured feedback and a verified hold mode; hold frame transmitted and
    /// flushed on the bus.
    Held,
    /// Motor feedback is missing or stale (> 100 ms), so the owner continues holding at its last
    /// captured hold target (never dropping holds while `torque_intent == On`).
    HeldStaleTarget,
    /// Motor could not be held: feedback has never been captured (missing/stale/unbounded).
    UnheldNoFreshFeedback,
    /// Motor could not be held: control mode is unverified/unavailable.
    UnheldNoVerifiedMode,
    /// Motor hold frame failed or expired at the transport boundary.
    UnheldTxFailed,
}

impl JointHoldStatus {
    #[inline]
    pub const fn short_tag(self) -> &'static str {
        match self {
            Self::SkippedOff => "skip",
            Self::Queued => "queued",
            Self::Held => "held",
            Self::HeldStaleTarget => "hold target stale",
            Self::UnheldNoFreshFeedback => "unheld(no_fresh_fb)",
            Self::UnheldNoVerifiedMode => "unheld(no_mode)",
            Self::UnheldTxFailed => "unheld(tx_err)",
        }
    }
}

/// Zero-allocation summary of one masked measured-position hold pass.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HoldPassReport {
    pub outcome: TxBatchOutcome,
    pub dof: usize,
    pub sent_any: bool,
    pub joint_status: [JointHoldStatus; crate::rt_core::mailbox::MAX_DOF],
}

impl HoldPassReport {
    pub fn empty(dof: usize) -> Self {
        Self {
            outcome: TxBatchOutcome::Complete,
            dof: dof.min(crate::rt_core::mailbox::MAX_DOF),
            sent_any: false,
            joint_status: [JointHoldStatus::SkippedOff; crate::rt_core::mailbox::MAX_DOF],
        }
    }
}

/// Per-joint numeric validity check for the driver's advertised feedback capabilities.
#[inline]
pub(crate) fn joint_state_valid(s: &JointState, caps: DriverCapabilities) -> bool {
    s.position.is_finite()
        && (!caps.velocity_feedback || s.velocity.is_finite())
        && (!caps.torque_feedback || s.torque.is_finite())
        && (!caps.temperature_feedback || s.temperature.is_finite())
}

/// Position-force gripper freshness status observed during the most recent bounded TX pass.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PosForceTxStatus {
    /// No active `PosForce` joint was checked in this pass (or pass did not run).
    #[default]
    Unobserved,
    /// Active `PosForce` joint had provably fresh (`<= 100 ms`) feedback throughout the pass.
    Fresh,
    /// Active `PosForce` joint feedback was stale (`> 100 ms` or missing receive-queue bound);
    /// a `0x307` freeze frame (or documented boot fallback when no fresh sample has been decoded
    /// since boot) was selected instead of any new target, and a `0xCC` refresh was scheduled.
    /// `fresh_gen` identifies the most recent provably fresh J7 feedback episode so a fresh RX
    /// sample arriving between two stale output passes always starts a new staleness episode.
    Stale { joint: usize, age: Option<Duration>, fresh_gen: u64 },
}

/// Boot fallback policy when an active `PosForce` J7 gripper has NEVER decoded a provably fresh
/// (`<= 100 ms`, bounded, finite) feedback sample since this `armd` process started
/// (`pf_last_fresh_position == None`).
///
/// - [`Self::NoOutputUntilFresh`]: send NO `0x307` frame to J7 until the first provably fresh
///   J7 feedback sample has been decoded (J7 has `TIMEOUT=0`, so any existing grip stays held),
///   while continuing to request `0xCC` status refreshes. Once the first fresh sample is decoded,
///   hold at that measured position (`vel_raw = 0`) with the configured `current_ceiling_raw`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PosForceBootFallback {
    NoOutputUntilFresh,
}

/// Active boot fallback when no provably fresh J7 sample has been decoded since this `armd`
/// process started.
pub const POS_FORCE_BOOT_FALLBACK: PosForceBootFallback =
    PosForceBootFallback::NoOutputUntilFresh;

/// Stop-generation and E-stop/safety epoch stamp captured when a J7 [`PosForceIntent`] is
/// produced. Every J7 intent carries this stamp; the single J7 emitter treats any `Target`
/// whose stamp is older than the current `(stop_gen, estop_epoch)` as `Hold` (never re-stamping
/// an old `Target` with a newer generation).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PosForceIntentStamp {
    /// Owner stop generation (`SupervisorInner::current_stop_gen()` / `EstopGate::stop_gen`).
    pub stop_gen: u64,
    /// Safety / E-stop epoch (`SupervisorInner::safety.epoch_id()` / `EstopGate::estop_gen`).
    pub estop_epoch: u64,
    /// Lease generation under which a `Target` intent was produced (`0` when hold/stop-class
    /// or not bound to a client lease).
    pub lease_gen: u64,
}

impl PosForceIntentStamp {
    /// True when a `Target` intent stamped with `self` is stale relative to `current`:
    /// stop generation or safety epoch changed, or a client-lease target (`self.lease_gen != 0`)
    /// no longer matches `current.lease_gen`.
    #[inline]
    pub const fn is_stale_against(self, current: Self) -> bool {
        self.stop_gen != current.stop_gen
            || self.estop_epoch != current.estop_epoch
            || (self.lease_gen != 0 && self.lease_gen != current.lease_gen)
    }
}

/// Desired position-force J7 gripper action before stop-generation / epoch resolution.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PosForceIntentKind {
    /// Drive the gripper toward `position` with requested `velocity` (rad/s, clamped by the
    /// active `PosForce` close/open ceiling) and optional `current_raw` cap (always clamped to
    /// `<= configured current_raw`).
    Target {
        position: f32,
        velocity: Option<f32>,
        current_raw: Option<u16>,
    },
    /// Stationary hold at the current fresh measured J7 position (`vel_raw = 0`, configured
    /// current cap) when fresh, or the freeze / boot-fallback frame when stale.
    Hold,
    /// Stopped / E-stop hold at the current fresh measured J7 position (`vel_raw = 0`, configured
    /// current cap) when fresh, or the freeze / boot-fallback frame when stale.
    Stop,
}

/// Stamped position-force J7 gripper intent for a control tick or driver operation.
///
/// Control-loop paths (motion, servo, stopped hold, E-stop, replay, policy, gripper actions,
/// MIT, gravity compensation) ONLY set a stamped `PosForceIntent` and NEVER construct or write a
/// `0x307` frame directly. The single J7 emitter ([`ArmDriver::emit_pos_force_j7`]) resolves
/// `(intent, current_stamp, freshness)` into the exact `0x307` wire frame inside the private
/// `pos_force_emitter` module, which holds the unconstructible `PosForceSendToken`.
///
/// Compile-time enforcement — callers outside `driver::pos_force_emitter` cannot access the
/// private emitter module or construct a `PosForceSendToken`:
///
/// ```compile_fail
/// use armd::driver::pos_force_emitter;
/// let _ = pos_force_emitter::PosForceSendToken;
/// ```
///
/// Nor is `build_pos_force_frame` re-exported from `armd::driver` or callable from outside
/// `driver::pos_force_emitter`:
///
/// ```compile_fail
/// let _ = armd::driver::build_pos_force_frame;
/// ```
///
/// ```compile_fail
/// let _ = armd::protocol::damiao::build_pos_force_frame;
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PosForceIntent {
    pub kind: PosForceIntentKind,
    pub stamp: PosForceIntentStamp,
}

impl PosForceIntent {
    pub const fn target(
        position: f32,
        velocity: Option<f32>,
        current_raw: Option<u16>,
        stamp: PosForceIntentStamp,
    ) -> Self {
        Self {
            kind: PosForceIntentKind::Target { position, velocity, current_raw },
            stamp,
        }
    }

    pub const fn hold(stamp: PosForceIntentStamp) -> Self {
        Self { kind: PosForceIntentKind::Hold, stamp }
    }

    pub const fn stop(stamp: PosForceIntentStamp) -> Self {
        Self { kind: PosForceIntentKind::Stop, stamp }
    }
}

/// Outcome of the single J7 position-force tick emitter ([`ArmDriver::emit_pos_force_j7`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PosForceEmitOutcome {
    /// Driver has no active/enabled `PosForce` gripper (complete no-op).
    NotApplicable,
    /// No provably fresh J7 sample has been decoded since this `armd` process started
    /// (`POS_FORCE_BOOT_FALLBACK == NoOutputUntilFresh`): no `0x307` frame was transmitted
    /// (J7 keeps its last command under `TIMEOUT=0`) while `0xCC` refresh polling stays active.
    AwaitingFirstFresh { joint: usize, pf_status: PosForceTxStatus },
    /// J7 `0x307` frame was transmitted and flushed to the transport.
    Sent { joint: usize, freeze: bool, pf_status: PosForceTxStatus },
    /// A requested J7 `Target` motion intent hit the per-release cutoff or budget under an armed
    /// MIT fence and degraded to a stationary continuity `Hold` frame that was transmitted and
    /// flushed. Continuity is preserved, while the requested `Target` was NOT accepted and stays
    /// pending/deferred for the next release.
    SentHoldInsteadOfTarget { joint: usize, freeze: bool, pf_status: PosForceTxStatus },
    /// J7 `0x307` frame was deferred by per-tick TX budget or backpressure.
    Deferred { joint: usize, pf_status: PosForceTxStatus },
}

/// Bounded maximum window for an active read-only shutdown disarm proof probe (`0xCC` polls).
pub const SHUTDOWN_DISARM_PROOF_WINDOW: Duration = Duration::from_millis(300);

/// Bounded zero-allocation outcome of clearing carried-over motor hardware errors on a disarmed arm
/// (`CMD_CLEAR_FAULTS` sent to affected motors only, then verified on a fresh post-clear status reply).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CarryoverMotorClearOutcome {
    /// No motor reported an active hardware fault bit.
    NoErrors,
    /// Every affected motor accepted `CMD_CLEAR_FAULTS`, replied with a fresh post-clear status,
    /// cleared its error bit, and remained disabled (`status == 0`).
    Cleared,
    /// At least one motor still reported a hardware fault code in a fresh post-clear reply.
    ErrorPersists { joint: usize, code: u8 },
    /// At least one motor reported torque enabled (`status == 1`) after the clear attempt.
    MotorEnabled { joint: usize },
    /// At least one affected motor did not return a fresh post-clear status reply within the bound.
    Unconfirmed { joint: usize },
    /// Pre-transmit live authority/stop gate revoked immediately before `can.transmit`.
    Revoked(&'static str),
}

pub trait ArmDriver: Send {
    fn capabilities(&self) -> DriverCapabilities { DriverCapabilities::default() }

    /// Reported enable state from motor feedback (TELEMETRY ONLY for observation and UI;
    /// never an input to hold, disable, rollback, or recovery decisions).
    fn enable_state(&self) -> EnableState { EnableState::Unknown }

    /// Driver-owned physical disarm proof for lifecycle teardown / replacement (DRV-F1 B6):
    /// returns `true` ONLY when hardware has confirmed all joints disabled with fresh, causal
    /// evidence (invalidated by staleness, ambiguous writes, or subsequent enable/motion writes).
    fn is_hardware_disarmed_confirmed(&self) -> bool {
        self.enable_state() == EnableState::Disabled
            && self.is_feedback_valid()
            && !self.has_any_joint_enabled()
            && !self.has_known_partial_enable()
            && !self.torque_transition_pending()
    }

    /// Actively obtain fresh causal per-axis physical disarm proof within `deadline` using ONLY
    /// read-only status polls (NEVER transmitting enable, motion, or disable frames).
    /// Returns `true` ONLY when [`Self::is_hardware_disarmed_confirmed`] holds after probing;
    /// otherwise logs which axis/axes failed proof and returns `false`.
    fn probe_hardware_disarmed_until(&mut self, _deadline: Instant) -> bool {
        self.is_hardware_disarmed_confirmed()
    }

    /// Monotonic counter incremented strictly when a fresh causal hardware sample carrying
    /// non-enabled torque state (`enable_state != Enabled`) is decoded from the bus (R12 M2).
    /// Returns `None` for drivers without an explicit per-sample counter.
    fn reported_torque_loss_sample_seq(&self) -> Option<u64> {
        None
    }

    /// True when this driver implements [`Self::fresh_measured_position_for`] directly from
    /// its own transport-provenance feedback cache (true for all real hardware and sim drivers).
    fn has_driver_fresh_position_api(&self) -> bool { false }

    /// Fresh, finite measured position for a single `joint` no older than `max_age`, with
    /// transport provenance (receive-queue bound when tracked). Zero heap allocations.
    fn fresh_measured_position_for(&self, joint: usize, max_age: Duration) -> Option<f32> {
        let mut pos = [0.0f32; 1];
        self.fresh_joint_positions_into(&[joint], max_age, &mut pos)
            .then_some(pos[0])
    }

    /// Returns `(position, continuity_epoch)` for joint `joint` when its transport-provenance
    /// feedback is finite and at most `max_age` old. `continuity_epoch` advances whenever
    /// feedback continuity breaks (`> 100 ms` gap or non-finite feedback).
    fn fresh_measured_sample_for(&self, joint: usize, max_age: Duration) -> Option<(f32, u64)> {
        self.fresh_measured_position_for(joint, max_age).map(|p| (p, 0))
    }

    /// Status of the driver's K=3 causal rest-pose evidence window for `StoppedDisableGate` (S2/L1).
    fn rest_pose_gate_status(&self) -> RestPoseGateStatus {
        RestPoseGateStatus::Confirmed
    }

    /// True when `begin_disable_torque_budgeted` + `poll_torque_budgeted` can hold a stopped
    /// disable request pending while an exhausted RX drain or incomplete K=3 rest-pose window
    /// completes within the 1 s request lifetime (L1).
    fn supports_budgeted_disable_await(&self) -> bool {
        false
    }

    /// Notify the driver that a new control release (`release_seq`) has begun.
    /// Once invoked by the owner, rest-pose K=3 ticks count distinct `release_seq` values
    /// rather than individual receive/drain passes.
    fn begin_control_release(&mut self, _release_seq: u64) {}

    /// End-of-release watchdog continuity fallback: after normal control/hold output has run,
    /// emit an urgent stationary hold ONLY for owner-authorized (`authorized_mask`) already-enabled
    /// arm joints that did NOT receive a confirmed control frame in this release and whose
    /// watchdog deadline needs service. When `hold_targets` is `Some`, ONLY finite owner-captured
    /// targets from that slice are permitted (no fallback to driver-cached positions).
    fn service_release_watchdog_fallback(
        &mut self,
        _authorized_mask: u64,
        _hold_targets: Option<&[Option<f32>]>,
    ) {}

    /// Age of the last received sample for each joint. Reading a cache does not refresh it.
    /// Implementation primitive (allocates): owners call `feedback_ages_into` (or the
    /// `dyn ArmDriver` snapshot helper); kept until the test doubles are regrouped (S6).
    fn feedback_ages(&self) -> Vec<Option<Duration>> { Vec::new() }

    /// Monotonic receive instant of the last received sample for each joint.
    /// Reading a cache does not refresh it. Default derives it from [`Self::feedback_ages`].
    /// Implementation primitive (allocates): owners call `feedback_instants_into` (or the
    /// `dyn ArmDriver` snapshot helper); kept until the test doubles are regrouped (S6).
    fn feedback_instants(&self) -> Vec<Option<Instant>> {
        let now = Instant::now();
        self.feedback_ages().into_iter().map(|a| a.and_then(|age| now.checked_sub(age))).collect()
    }

    /// Monotonic per-joint hardware feedback sequence counter, incremented once per decoded
    /// hardware reply for that joint. Reading a cache does not advance it. Empty means the
    /// driver does not report explicit counters (supervisor falls back to instant advances).
    /// Implementation primitive (allocates): owners call `feedback_seqs_into` (or the
    /// `dyn ArmDriver` snapshot helper); kept until the test doubles are regrouped (S6).
    fn feedback_seqs(&self) -> Vec<u64> { Vec::new() }

    /// Per joint, a monotonic instant at which the driver saw its receive queue empty before it
    /// read that joint's last decoded sample, so the sample was enqueued after this instant
    /// (None: no empty queue was seen before it). Decode-time instants cannot bound queued
    /// frames: a bounded drain can decode a backlog several ticks after it arrived. Empty means
    /// the driver has no receive queue between bus and decode (request/response transports);
    /// callers then use [`Self::feedback_instants`].
    /// Implementation primitive (allocates): owners call `feedback_queue_bounds_into` (or the
    /// `dyn ArmDriver` snapshot helper); kept until the test doubles are regrouped (S6).
    fn feedback_queue_bounds(&self) -> Vec<Option<Instant>> { Vec::new() }

    /// Hardware receive timestamps for each joint's latest feedback packet (if tracked).
    /// Implementation primitive (allocates): owners call `feedback_receive_instants_into` (or the
    /// `dyn ArmDriver` snapshot helper); kept until the test doubles are regrouped (S6).
    fn feedback_receive_instants(&self) -> Vec<Option<Instant>> { Vec::new() }

    // Allocation-free forms of the per-tick reads (the RT executor calls these with
    // preallocated buffers; armd 1.6 RT core). Each fills `out` with what the allocating
    // method returns. The defaults call the allocating method; drivers on the control path
    // override them so a steady-state tick does not touch the allocator.

    /// Every one of the first `dof` joints has feedback no older than `max_age` (and there are
    /// exactly `dof` ages): [`Self::feedback_ages`] without building the vector.
    fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
        let ages = self.feedback_ages();
        ages.len() == dof && ages.iter().all(|a| a.is_some_and(|age| age <= max_age))
    }
    /// Every listed joint index has feedback no older than `max_age`
    /// (without allocating in `DamiaoArmDriver`).
    fn feedback_joints_within(&self, joints: &[usize], max_age: Duration) -> bool {
        let ages = self.feedback_ages();
        joints.iter().all(|&i| ages.get(i).copied().flatten().is_some_and(|age| age <= max_age))
    }
    /// Read fresh positions for the listed `joints` from the same driver feedback snapshot whose
    /// timestamps (and receive-queue bounds, when tracked) are checked against `max_age`.
    ///
    /// Writes each listed joint's latest driver-cached position into `out[k]` (matching
    /// `joints[k]`). Returns `true` only when every listed joint has a non-missing timestamp
    /// within `max_age` and a finite cached position from that same feedback snapshot; unlisted
    /// joints (such as the gripper) are ignored. Zero heap allocations when overridden.
    fn fresh_joint_positions_into(
        &self,
        _joints: &[usize],
        _max_age: Duration,
        _out: &mut [f32],
    ) -> bool {
        false
    }
    /// [`Self::feedback_ages`] as whole milliseconds (the observation's `feedback_age_ms`).
    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
        out.clear();
        out.extend(self.feedback_ages().iter().map(|a| a.map(|v| v.as_millis().min(u64::MAX as u128) as u64)));
    }
    /// [`Self::read_joint_states`] into `out`.
    fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
        let states = self.read_joint_states()?;
        out.clear();
        out.extend_from_slice(&states);
        Ok(())
    }
    /// Budgeted single-pass hardware receive into `out`, stopping at `deadline` (driver clock)
    /// or `max_rx_frames`. Returns `(rx_frames_serviced, budget_exhausted)`.
    fn read_joint_states_budgeted(
        &mut self,
        out: &mut Vec<JointState>,
        _deadline: Instant,
        _max_rx_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        self.read_joint_states_into(out)?;
        Ok((0, false))
    }
    /// Bounded live RX pass used by SAFE-HOLD so every supported legacy/RT driver (including
    /// serial Feetech, RobStride, and Damiao) maintains live thermal/freshness monitoring without
    /// heap allocation (DRV-F1 B9). Defaults to [`Self::read_joint_states_budgeted`].
    fn read_joint_states_safe_hold(
        &mut self,
        out: &mut Vec<JointState>,
        deadline: Instant,
        max_rx_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        self.read_joint_states_budgeted(out, deadline, max_rx_frames)
    }
    /// Copy the driver's already-cached joint states into `out` without touching the bus.
    /// Returns `None` if the driver does not maintain an inspectable cache.
    fn read_cached_joint_states_into(&self, _out: &mut Vec<JointState>) -> Option<()> {
        None
    }
    /// [`Self::feedback_ages`] into `out`.
    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) { out.clear(); out.extend_from_slice(&self.feedback_ages()); }
    /// [`Self::feedback_instants`] into `out`.
    fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) { out.clear(); out.extend_from_slice(&self.feedback_instants()); }
    /// [`Self::feedback_seqs`] into `out`.
    fn feedback_seqs_into(&self, out: &mut Vec<u64>) { out.clear(); out.extend_from_slice(&self.feedback_seqs()); }
    /// [`Self::feedback_queue_bounds`] into `out`.
    fn feedback_queue_bounds_into(&self, out: &mut Vec<Option<Instant>>) { out.clear(); out.extend_from_slice(&self.feedback_queue_bounds()); }
    /// [`Self::feedback_receive_instants`] into `out`.
    fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) { out.clear(); out.extend_from_slice(&self.feedback_receive_instants()); }
    /// [`Self::joint_loads`] into `out`; false (and `out` empty) when the driver has none.
    fn joint_loads_into(&self, out: &mut Vec<f32>) -> bool {
        out.clear();
        match self.joint_loads() { Some(l) => { out.extend_from_slice(&l); true } None => false }
    }
    /// [`Self::motor_status`] into `out`; false (and `out` empty) when the driver has none.
    fn motor_status_into(&self, out: &mut Vec<u8>) -> bool {
        out.clear();
        match self.motor_status() { Some(s) => { out.extend_from_slice(&s); true } None => false }
    }
    /// True when [`Self::clear_fault_disables_joints`] includes any joint index `< arm_dof`,
    /// without allocating a `Vec`.
    fn clear_fault_disables_arm_joint(&self, arm_dof: usize) -> bool {
        self.clear_fault_disables_joints().into_iter().any(|j| j < arm_dof)
    }
    /// True when [`Self::check_hardware_status`] would report a hardware fault, without
    /// allocating a formatted error string.
    fn has_hardware_fault(&self) -> bool {
        self.check_hardware_status().is_err()
    }

    /// Fixed device-side velocity ceiling, when the active output mode has one.
    fn max_command_velocity(&self, _joint: usize) -> Option<f32> { None }

    /// Device-side position limits (min, max) in radians, when the hardware derives them.
    fn position_limits(&self, _joint: usize) -> Option<(f32, f32)> { None }

    /// Read real-time telemetry / joint states for all joints
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError>;

    /// Read real-time telemetry / joint states by draining RX only, without transmitting any
    /// autonomous `0xCC` refresh or recovery frame. Defaults to [`Self::read_joint_states`].
    fn read_joint_states_rx_only(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        self.read_joint_states()
    }

    /// Send target positions / commands to all joints
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError>;

    /// Write stationary restorative hold frames while a stop/E-stop is latched (anti-fall
    /// hold on a brakeless arm). Defaults to [`Self::write_joint_targets`] for mock/sim drivers;
    /// [`DamiaoArmDriver`] discards any queued motion, refuses pending stops, and routes every
    /// frame through [`DamiaoArmDriver::send`] with `TxKind::RollbackHold`, rebuilding each
    /// hold at send time from fresh (`<= 100 ms`) measured feedback.
    fn write_estop_hold(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.write_joint_targets(targets)
    }

    /// Bounded stopped-hold write: carries deferred hold frames in an inline hold queue across
    /// ticks instead of spamming errors or re-latching faults. Drivers without a bounded hold
    /// queue default to [`Self::write_estop_hold`] -> [`TxBatchOutcome::Complete`].
    fn write_estop_hold_bounded(&mut self, targets: &[JointTarget]) -> Result<TxBatchOutcome, ArmDriverError> {
        self.write_estop_hold(targets).map(|()| TxBatchOutcome::Complete)
    }

    /// Masked measured-position hold pass: services EVERY joint with bit set in `hold_mask`
    /// that has fresh (`<= 100 ms`) finite feedback with transport provenance and a verified
    /// hold mode. Never skips a joint based on feedback-reported enable state.
    fn write_measured_hold_masked(
        &mut self,
        hold_mask: u64,
        targets: &[JointTarget],
    ) -> Result<HoldPassReport, ArmDriverError> {
        let dof = targets.len().min(crate::rt_core::mailbox::MAX_DOF);
        let mut report = HoldPassReport::empty(dof);
        let mut eff_mask = 0u64;
        for i in 0..dof {
            if (hold_mask & (1u64 << i)) == 0 {
                report.joint_status[i] = if !self.joint_mode_ext().unwrap_or(&NoExt).joint_hold_mode_verified(i) {
                    JointHoldStatus::UnheldNoVerifiedMode
                } else {
                    JointHoldStatus::UnheldNoFreshFeedback
                };
            } else if !self.joint_mode_ext().unwrap_or(&NoExt).joint_hold_mode_verified(i) {
                report.joint_status[i] = JointHoldStatus::UnheldNoVerifiedMode;
            } else {
                eff_mask |= 1u64 << i;
                report.joint_status[i] = JointHoldStatus::Queued;
            }
        }
        match self.tx_ext_mut() {
            Some(tx) => tx.drop_pending_holds_outside_mask(eff_mask),
            None => NoExt.drop_pending_holds_outside_mask(eff_mask),
        }
        if eff_mask == 0 {
            return Ok(report);
        }
        let all_mask = if dof >= 64 { u64::MAX } else { (1u64 << dof) - 1 };
        // Fail closed for drivers that do not override masked hold writes: never forward a
        // partial mask to an unmasked whole-arm writer that would transmit placeholder targets
        // to excluded joints.
        if eff_mask != all_mask {
            for i in 0..dof {
                if (eff_mask & (1u64 << i)) != 0 {
                    report.joint_status[i] = JointHoldStatus::UnheldNoVerifiedMode;
                }
            }
            return Ok(report);
        }
        let outcome = self.write_estop_hold_bounded(targets)?;
        let sent_any = self.tx_ext().unwrap_or(&NoExt).last_hold_tx_sent_any();
        let fb_fresh = self.is_feedback_valid();
        for i in 0..dof {
            if (eff_mask & (1u64 << i)) != 0 {
                report.joint_status[i] = match outcome {
                    TxBatchOutcome::Complete if sent_any => {
                        if fb_fresh {
                            JointHoldStatus::Held
                        } else {
                            JointHoldStatus::HeldStaleTarget
                        }
                    }
                    TxBatchOutcome::Deferred { .. } => {
                        if fb_fresh {
                            JointHoldStatus::Queued
                        } else {
                            JointHoldStatus::HeldStaleTarget
                        }
                    }
                    _ => JointHoldStatus::UnheldTxFailed,
                };
            }
        }
        report.outcome = outcome;
        report.sent_any = sent_any;
        Ok(report)
    }

    /// Bounded target write: distinguishes complete submission ([`TxBatchOutcome::Complete`])
    /// from transient deferral ([`TxBatchOutcome::Deferred`]) without faulting or authorizing
    /// motion completion on deferral. Drivers and wrappers that support pending-queue carry-over
    /// override this method; the default forwards to fail-closed [`Self::write_joint_targets`].
    /// Stages unfenced motion targets into the driver's carry-over TX queue with zero bus TX when
    /// the release TX cutoff already expired before output started (avoids staging an already-expired
    /// driver deadline while preserving carry-over parity).
    fn defer_motion_targets_past_cutoff(&mut self, _targets: &[JointTarget]) {}

    fn write_joint_targets_bounded(&mut self, targets: &[JointTarget]) -> Result<TxBatchOutcome, ArmDriverError> {
        self.write_joint_targets(targets).map(|()| TxBatchOutcome::Complete)
    }

    /// Sets an absolute driver-clock ceiling on the active release's TX start-cutoff
    /// (`tx_deadline = min(first_tx + tx_budget, ceiling)`). Pass `None` when leaving a
    /// control release so off-tick calls are never clipped by a prior release deadline.
    fn set_tx_release_deadline(&mut self, _deadline: Option<Instant>) {}

    /// Enable or disable torque / power across all joints
    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError>;

    /// Blocking hardware torque disable with an explicit [`DisableKind`] enforced at the
    /// driver's transmit choke point.
    fn disable_torque(&mut self, kind: DisableKind) -> Result<(), ArmDriverError> {
        let _ = kind;
        self.set_torque(false)
    }

    /// Start a torque transition without waiting for its confirmation. `Pending { wake }`: serve
    /// it with [`ArmDriver::poll_torque`] no later than `wake` (absolute, driver clock) until it
    /// returns `Done`; nothing else may be sent to the arm meanwhile except a disable, which
    /// replaces a pending enable. Default: the blocking [`ArmDriver::set_torque`], `Done` on success.
    fn begin_torque(&mut self, enabled: bool) -> Result<TorqueProgress, ArmDriverError> {
        self.set_torque(enabled).map(|()| {
            let outcome = if self.capabilities().enable_feedback {
                TorqueOutcome::Confirmed
            } else {
                TorqueOutcome::Unconfirmed
            };
            TorqueProgress::Done(outcome)
        })
    }

    /// Non-blocking hardware torque disable with an explicit [`DisableKind`].
    /// Defaults to [`Self::begin_torque`]`(false)` so mock/custom drivers implementing
    /// non-blocking `begin_torque` preserve their async confirmation handshake; real
    /// hardware drivers override this to forward `kind` to their transmit choke point.
    fn begin_disable_torque(&mut self, kind: DisableKind) -> Result<TorqueProgress, ArmDriverError> {
        let _ = kind;
        self.begin_torque(false)
    }

    /// Budgeted [`Self::begin_torque`]: when `max_rx_frames == 0`, performs no bus RX drain
    /// before transmitting (relying on the release's Step-2 poll boundary).
    fn begin_torque_budgeted(&mut self, enabled: bool, _max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
        self.begin_torque(enabled)
    }

    /// Budgeted [`Self::begin_torque`] that also allows a bounded, ticked retry of an unverified
    /// startup mapping-range check (legacy WS path; RT executor keeps [`Self::begin_torque_budgeted`]).
    fn begin_torque_budgeted_with_range_retry(
        &mut self,
        enabled: bool,
        max_rx_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.begin_torque_budgeted(enabled, max_rx_frames)
    }

    /// Budgeted non-blocking hardware torque disable with an explicit [`DisableKind`].
    fn begin_disable_torque_budgeted(
        &mut self,
        kind: DisableKind,
        max_rx_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        let _ = kind;
        self.begin_torque_budgeted(false, max_rx_frames)
    }

    /// Starts a client-initiated park disable routed through the single `DisableRequested`
    /// rest-pose gate whose EVERY per-motor disable write is also gated against pending Stop,
    /// E-stop latch, and `expected_stop_gen` immediately before transmit (e.g.,
    /// `DamiaoArmDriver::send(TxKind::ClientParkDisable)` and
    /// `FeetechArmDriver::write_torque_off_checked_budgeted`).
    /// Watchdog / fault disables continue using [`Self::begin_disable_torque_budgeted`] /
    /// [`Self::begin_disable_torque`].
    fn begin_client_park_disable(
        &mut self,
        max_rx_frames: Option<u16>,
        expected_stop_gen: Option<u64>,
    ) -> Result<TorqueProgress, ArmDriverError> {
        let kind = DisableKind::DisableRequested {
            started_at: self.clock_now(),
        };
        self.begin_client_park_disable_with_kind(kind, max_rx_frames, expected_stop_gen)
    }

    /// Starts or retries a client-initiated park disable carrying an explicit [`DisableKind`]
    /// (preserving the original request `started_at` lifetime) and Stop generation fence.
    fn begin_client_park_disable_with_kind(
        &mut self,
        kind: DisableKind,
        max_rx_frames: Option<u16>,
        _expected_stop_gen: Option<u64>,
    ) -> Result<TorqueProgress, ArmDriverError> {
        match max_rx_frames {
            Some(limit) => self.begin_disable_torque_budgeted(kind, limit),
            None => self.begin_disable_torque(kind),
        }
    }

    /// Serve the pending torque transition once: one receive pass ending by `deadline` (and the
    /// driver's own frame / time bound), at most one retransmit per joint, no sleep. Err when the
    /// transition failed (or none is pending).
    fn poll_torque(&mut self, _deadline: Instant) -> Result<TorqueProgress, ArmDriverError> {
        Err(ArmDriverError::ProtocolStatic("no torque transition pending"))
    }

    /// Budgeted [`Self::poll_torque`] capping bus RX to `max_rx_frames` and `deadline`.
    fn poll_torque_budgeted(&mut self, deadline: Instant, _max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
        self.poll_torque(deadline)
    }

    /// Returns `(rx_frames, budget_exhausted)` from the most recent `poll_torque_budgeted` if
    /// it performed a bus receive pass that refreshed the driver's feedback cache.
    fn take_last_poll_rx(&mut self) -> Option<(u16, bool)> {
        None
    }

    /// A transition started by [`ArmDriver::begin_torque`] is not finished.
    fn torque_transition_pending(&self) -> bool { false }

    /// True once an active disable handshake has passed its rest-pose gate and transmitted at
    /// least one disable frame (e.g. `DamiaoTorque::Disable` repeat passes or partial per-motor
    /// dispatch). False while still awaiting rest-pose evidence (`AwaitRestGate`) or idle.
    fn disable_frames_dispatched(&self) -> bool { false }

    /// Bitmask of 0-based joint indices that have already received a disable frame in the
    /// active/partial disable sequence (R12 B3). Cleared when all joints complete or on enable.
    fn disable_dispatched_mask(&self) -> u64 { 0 }

    /// Aborts any pending enable handshake without transmitting further enable frames
    /// (called when E-stop or a fault intervenes mid-enable).
    fn abort_enable(&mut self) {}

    /// Last wire-committed target position for `joint` as `(commit_seq, tx_at, user_space_rad)`,
    /// recorded strictly at the confirmed transmit/flush boundary for non-skipped control frames
    /// (after zero-offset conversion and wire quantization). Zero heap allocations.
    fn last_committed_wire_position(&self, _joint: usize) -> Option<(u64, Instant, f32)> {
        None
    }
    /// Prepares an exclusively owned driver on SAFE-HOLD entry: aborts any interrupted enable
    /// handshake AND reconciles any abandoned disable handshake while preserving
    /// `disable_dispatched_mask` so remaining energized axes keep receiving measured-position holds
    /// (DRV-F1 B10), and switches PosForce J7 into standalone emitter mode.
    fn prepare_safe_hold(&mut self) {
        self.abort_enable();
        match self.pos_force_ext_mut() {
            Some(pf) => pf.set_external_pos_force_emitter(false),
            None => NoExt.set_external_pos_force_emitter(false),
        }
    }

    /// True if any enable command frame (`CMD_ENABLE`) has already been transmitted for the
    /// active enable handshake or any joint is known enabled. False while the handshake is still
    /// in pre-enable validation/drain phases before transmitting enable frames.
    fn enable_frames_dispatched(&self) -> bool {
        self.has_any_joint_enabled() || self.has_known_partial_enable()
    }

    /// Aborts a pending non-protective disable handshake ONLY if no disable frame has been
    /// transmitted yet (`!disable_frames_dispatched()`). Returns `true` when aborted or idle,
    /// and `false` when disable frames have already been dispatched or the disable is protective
    /// (so hardware completion must continue).
    fn abort_pending_disable(&mut self) -> bool {
        !self.disable_frames_dispatched()
    }

    /// 0-based joint indices whose torque-off is NOT yet confirmed by fresh state-0 feedback
    /// after the last enable/disable command (empty when all confirmed).
    fn unconfirmed_disable_joints(&self) -> Vec<usize> {
        if self.enable_state() == EnableState::Disabled {
            return Vec::new();
        }
        match self.motor_status() {
            Some(status) => {
                let non_zero: Vec<usize> = status
                    .iter()
                    .enumerate()
                    .filter_map(|(i, &st)| (st != 0).then_some(i))
                    .collect();
                if non_zero.is_empty() {
                    (0..status.len()).collect()
                } else {
                    non_zero
                }
            }
            None => Vec::new(),
        }
    }

    /// Ok when this driver and its underlying transport support non-blocking synchronous
    /// RT executor ownership (`armd-control`).
    fn supports_rt_executor(&self) -> Result<(), &'static str> {
        Err("driver does not support non-blocking RT executor ownership")
    }

    /// Enables or disables strict non-sleeping RT mode on the driver (set `true` once
    /// ownership transfers to `armd-control`).
    fn set_rt_mode(&mut self, _rt: bool) {}

    /// Check if real physical telemetry feedback has been received for all joints
    fn is_feedback_valid(&self) -> bool {
        false
    }

    /// Check if any motor hardware fault is reported
    fn check_hardware_status(&self) -> Result<(), ArmDriverError> {
        Ok(())
    }

    /// Reset internal fault state flags when clearing safety faults
    fn clear_fault(&mut self) -> Result<(), ArmDriverError> {
        Ok(())
    }

    /// Time-bounded [`Self::clear_fault`] for the RT executor; defaults fail closed with
    /// `UNSUPPORTED` until a driver implements bounded transmission and flush checks.
    fn clear_fault_budgeted(&mut self, _deadline: Instant) -> Result<(), ArmDriverError> {
        Err(ArmDriverError::Protocol("UNSUPPORTED: driver has no bounded clear_fault".into()))
    }

    /// Return indices of joints disabled during clear_fault
    fn clear_fault_disables_joints(&self) -> Vec<usize> {
        Vec::new()
    }

    /// True when clear_fault left some joints enabled and some disabled
    fn has_known_partial_enable(&self) -> bool {
        false
    }

    /// True when any joint is known to have torque enabled
    fn has_any_joint_enabled(&self) -> bool {
        self.enable_state() == EnableState::Enabled
    }

    /// Re-read physical enable state of all joints from hardware
    fn refresh_enable_state(&mut self) -> Result<EnableState, ArmDriverError> {
        Ok(self.enable_state())
    }

    /// Set an atomic abort flag that hardware loops should check to abort early on E-stop
    fn set_abort_flag(&mut self, _flag: std::sync::Arc<std::sync::atomic::AtomicBool>) {}

    /// Attach the arm's single packed [`crate::rt_core::urgent::EstopState`] so hardware loops
    /// observe the authoritative atomic E-stop latch bit without a separate mirror flag.
    fn set_estop_state(&mut self, _state: std::sync::Arc<crate::rt_core::urgent::EstopState>) {}

    /// The arm's E-stop gate ([`crate::rt_core::urgent::EstopGate`]): a driver that sends a
    /// target batch frame by frame checks it before each frame and stops the batch while an
    /// E-stop request is not yet applied by the tick (`write_joint_targets` then fails with
    /// [`ESTOP_INHIBITED`]). Default: the driver does not check (the tick checks before its
    /// output).
    fn set_estop_gate(&mut self, _gate: crate::rt_core::urgent::EstopGate) {}

    /// Inject the supervisor's stopped-disable policy so the driver's transmit choke point
    /// enforces Rule A′ at send time for `DisableRequested` frames while stopped.
    fn set_stopped_disable_gate(&mut self, _gate: StoppedDisableGate) {}

    /// Set the gripper joint index if the arm model defines a gripper
    fn set_gripper_index(&mut self, _index: Option<usize>) {}

    /// Check whether driver / hardware is actively enabled and online
    fn is_enabled(&self) -> bool {
        false
    }

    /// Return per-joint motor status or error code if supported by the driver
    /// Implementation primitive (allocates): owners call `motor_status_into` (or the
    /// `dyn ArmDriver` snapshot helper); kept until the test doubles are regrouped (S6).
    fn motor_status(&self) -> Option<Vec<u8>> {
        None
    }

    /// Return per-joint load telemetry (normalized fraction in [-1.0, 1.0]) if supported
    /// Implementation primitive (allocates): owners call `joint_loads_into` (or the
    /// `dyn ArmDriver` snapshot helper); kept until the test doubles are regrouped (S6).
    fn joint_loads(&self) -> Option<Vec<f32>> {
        None
    }

    /// Decode motor fault/status code into descriptive string.
    /// Implementation primitive (allocates): kept for the 3 legacy owner sites until S5;
    /// RT paths call [`Self::decode_motor_error_static`].
    fn decode_motor_error(&self, code: u8) -> Option<String> {
        damiao_fault_name(code).map(str::to_string)
    }

    /// Zero-allocation static motor fault name for the RT control thread.
    fn decode_motor_error_static(&self, code: u8) -> Option<&'static str> {
        damiao_fault_name(code)
    }

    /// Update internal software enabled flag without transmitting physical frames
    fn set_torque_flag(&mut self, _enabled: bool) {}

    /// What the motor zero tool can do with this driver (`calibration_tools.motor_zero`).
    fn motor_zero_support(&self) -> MotorZeroSupport {
        MotorZeroSupport::unsupported("this driver has no motor zero command")
    }

    /// Current time of the driver's clock (the transport's deterministic clock in tests).
    fn clock_now(&self) -> Instant { Instant::now() }

    /// Optional lock-free handle to the driver's underlying clock for RT off-thread forwarding
    /// timestamps (returns `None` when the driver uses wall-clock `Instant::now()`).
    fn driver_clock(&self) -> Option<std::sync::Arc<dyn crate::transport::DriverClock>> { None }

    /// Wait `d` on the driver's clock (the transport's deterministic clock advances in tests).
    fn clock_pause(&self, d: Duration) {
        if crate::diagnostics::tick::hold_io_active() {
            let t0 = Instant::now();
            std::thread::sleep(d);
            let wall_ns = t0.elapsed().as_nanos().min(u128::from(u64::MAX)) as u64;
            let d_ns = d.as_nanos().min(u128::from(u64::MAX)) as u64;
            crate::diagnostics::tick::record_hold_sleep_ns(wall_ns.max(d_ns));
        } else {
            std::thread::sleep(d);
        }
    }

    // Capability extensions (src/driver/ext.rs). Drivers override the pair for each
    // extension they implement; owners call the `dyn ArmDriver` getters (`drv.tx()`, ...).
    /// [`BoundedTxExt`] of this driver; `None` = the trait's default bodies ([`NoExt`]).
    fn tx_ext(&self) -> Option<&dyn BoundedTxExt> { None }
    fn tx_ext_mut(&mut self) -> Option<&mut dyn BoundedTxExt> { None }
    /// [`ModeSwitchExt`] of this driver; `None` = the trait's default bodies ([`NoExt`]).
    fn mode_switch_ext(&self) -> Option<&dyn ModeSwitchExt> { None }
    fn mode_switch_ext_mut(&mut self) -> Option<&mut dyn ModeSwitchExt> { None }
    /// [`CommissioningExt`] of this driver; `None` = the trait's default bodies ([`NoExt`]).
    fn commissioning_ext(&self) -> Option<&dyn CommissioningExt> { None }
    fn commissioning_ext_mut(&mut self) -> Option<&mut dyn CommissioningExt> { None }
    /// [`JointModeExt`] of this driver; `None` = the trait's default bodies ([`NoExt`]).
    fn joint_mode_ext(&self) -> Option<&dyn JointModeExt> { None }
    fn joint_mode_ext_mut(&mut self) -> Option<&mut dyn JointModeExt> { None }
    /// [`MitExt`] of this driver; `None` = the trait's default bodies ([`NoExt`]).
    fn mit_ext(&self) -> Option<&dyn MitExt> { None }
    fn mit_ext_mut(&mut self) -> Option<&mut dyn MitExt> { None }
    /// [`PosForceExt`] of this driver; `None` = the trait's default bodies ([`NoExt`]).
    fn pos_force_ext(&self) -> Option<&dyn PosForceExt> { None }
    fn pos_force_ext_mut(&mut self) -> Option<&mut dyn PosForceExt> { None }
    /// [`BusRecoveryExt`] of this driver; `None` = the trait's default bodies ([`NoExt`]).
    fn bus_recovery_ext(&self) -> Option<&dyn BusRecoveryExt> { None }
    fn bus_recovery_ext_mut(&mut self) -> Option<&mut dyn BusRecoveryExt> { None }
    /// [`ShadowExt`] of this driver; `None` = the trait's default bodies ([`NoExt`]).
    #[cfg(not(target_arch = "wasm32"))]
    fn shadow_ext(&self) -> Option<&dyn ShadowExt> { None }
    #[cfg(not(target_arch = "wasm32"))]
    fn shadow_ext_mut(&mut self) -> Option<&mut dyn ShadowExt> { None }
    /// [`JointZeroExt`] of this driver; `None` = the trait's default bodies ([`NoExt`]).
    fn joint_zero_ext(&self) -> Option<&dyn JointZeroExt> { None }
    fn joint_zero_ext_mut(&mut self) -> Option<&mut dyn JointZeroExt> { None }
}

/// Allocating snapshots for non-RT owner paths, built on the `_into` methods. The `Vec`
/// returning trait methods (`feedback_ages`, `motor_status`, ...) are implementation
/// primitives of drivers and doubles; owners call these helpers or the `_into` forms.
///
/// S6 note (Fable M3): because these helpers read `*_into` rather than the `Vec` primitives,
/// test doubles that override only `*_into` surface their `_into` data through these helpers
/// (while pre-split legacy fault-clear freshness/resting-pose gates call `feedback_ages()`
/// directly). Delete the `Vec` primitives once S6 regroups the doubles.
impl dyn ArmDriver + '_ {
    pub fn feedback_ages_snapshot(&self) -> Vec<Option<Duration>> {
        let mut out = Vec::new();
        self.feedback_ages_into(&mut out);
        out
    }
    pub fn feedback_seqs_snapshot(&self) -> Vec<u64> {
        let mut out = Vec::new();
        self.feedback_seqs_into(&mut out);
        out
    }
    pub fn joint_loads_snapshot(&self) -> Option<Vec<f32>> {
        let mut out = Vec::new();
        self.joint_loads_into(&mut out).then_some(out)
    }
    pub fn motor_status_snapshot(&self) -> Option<Vec<u8>> {
        let mut out = Vec::new();
        self.motor_status_into(&mut out).then_some(out)
    }
}

/// Feedback no older than this confirms a motor disabled for a set-zero command.
pub const ZERO_FEEDBACK_MAX_AGE: Duration = Duration::from_millis(250);

/// Output mode of one joint on drivers that can switch at runtime.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum JointMode {
    /// The driver's position mode (Damiao POS_VEL).
    Position,
    /// MIT impedance: kp, kd, velocity and torque feed-forward from the JointTarget.
    Mit,
}

#[cfg(not(target_arch = "wasm32"))]
#[derive(Clone, Copy)]
struct PendingDisableShadowBatch {
    kind: crate::torque::WriteKind,
    target_mask: crate::torque::ChannelMask,
    outcomes: [crate::torque::MotorWriteOutcome; crate::torque::MAX_CHANNELS],
    ambiguous_gap: bool,
    write_done: Option<Instant>,
}

#[cfg(not(target_arch = "wasm32"))]
impl PendingDisableShadowBatch {
    fn flush_into(
        slot: &mut Option<Self>,
        staging: &mut crate::torque::shadow::ShadowDriverStaging,
        fallback_now: Instant,
    ) {
        if let Some(batch) = slot.take() {
            if staging.is_enabled() && !batch.target_mask.is_empty() {
                let ts = batch.write_done.unwrap_or(fallback_now);
                staging.push(crate::torque::shadow::ShadowDriverObs::WriteResult {
                    device: 0,
                    kind: batch.kind,
                    write_done: ts,
                    now: ts,
                    target_mask: batch.target_mask,
                    outcomes: batch.outcomes,
                    ambiguous_gap: batch.ambiguous_gap,
                });
            }
        }
    }

    fn merge_batch(slot: &mut Option<Self>, incoming: Self) {
        if let Some(existing) = slot.as_mut() {
            if existing.kind == incoming.kind {
                existing.target_mask = crate::torque::ChannelMask(
                    existing.target_mask.raw() | incoming.target_mask.raw(),
                );
                for ch in incoming.target_mask.iter() {
                    existing.outcomes[ch as usize] = incoming.outcomes[ch as usize];
                }
                existing.ambiguous_gap |= incoming.ambiguous_gap;
                if incoming.write_done.is_some() {
                    existing.write_done = incoming.write_done;
                }
                return;
            }
        }
        *slot = Some(incoming);
    }
}


// =========================================================================
// 3. Damiao Brushless Arm Driver (reBot Arm B601-DM)
// =========================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DamiaoControlMode {
    PosVel,
    Mit,
    /// Position-force (CTRL_MODE 4): every frame carries a motor current limit.
    PosForce,
}

/// Gripper joint driven in position-force mode with a motor current ceiling
/// ([`DamiaoArmDriver::with_gripper_pos_force`]).
#[derive(Debug, Clone, PartialEq)]
pub struct GripperPosForce {
    /// Joint index (0-based).
    pub joint: usize,
    /// Current ceiling of every frame: 1/10000 of the maximum phase current.
    pub current_raw: u16,
    /// Speed limit of closing frames (rad/s): min(requested, this).
    pub close_speed_rad_s: f32,
    /// Jaw positions (rad): a frame closes when it moves from the measured position toward `closed_rad`.
    pub closed_rad: f32,
    pub open_rad: f32,
    /// The ceiling's torque equivalent was measured (commissioning); false: uncalibrated.
    pub calibrated: bool,
    /// Torque equivalent of the ceiling (N·m), when commissioned.
    pub ceiling_torque_nm: Option<f32>,
    /// What the torque equivalent was measured against (`external` / `feedback_equivalent`).
    pub torque_reference: Option<String>,
}

/// The profile's choice for the Damiao gripper joint.
#[derive(Debug, Clone, PartialEq)]
pub enum GripperDrive {
    PosForce(GripperPosForce),
    /// Legacy POS_VEL on this joint (no motor current cap).
    PosVel(usize),
}

/// `ArmConfig::damiao_gripper` as a driver setting (None: no block or no gripper joint).
/// Position-force needs the gripper policy's closed / open positions (closing direction).
pub fn gripper_drive_from_profile(cfg: &crate::model::ArmConfig) -> Result<Option<GripperDrive>, String> {
    use crate::model::DamiaoGripperMode;
    let (Some(g), Some(joint)) = (cfg.damiao_gripper.as_ref(), cfg.gripper_index()) else { return Ok(None) };
    Ok(Some(match g.mode {
        DamiaoGripperMode::PosVel => GripperDrive::PosVel(joint),
        DamiaoGripperMode::PosForce => {
            let p = cfg.gripper_policy.as_ref().ok_or("damiao_gripper pos_force needs gripper_policy (closed_rad / open_rad)")?;
            let close_speed_rad_s = cfg.limits.get(joint)
                .filter(|l| l.max_velocity.is_finite() && l.max_velocity > 0.0)
                .map_or(g.close_speed_rad_s, |l| g.close_speed_rad_s.min(l.max_velocity));
            GripperDrive::PosForce(GripperPosForce {
                joint, current_raw: g.current_ceiling_raw, close_speed_rad_s,
                closed_rad: p.closed_rad, open_rad: p.open_rad, calibrated: g.calibrated,
                ceiling_torque_nm: g.ceiling_torque_nm, torque_reference: g.torque_reference.clone(),
            })
        }
    }))
}

/// Allocation-free `Copy` summary of the gripper's position-force motor current cap,
/// safe to query on the RT control thread (`armd-control`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GripperPosForceInfo {
    pub joint: usize,
    pub active: bool,
    pub close_speed_rad_s: f32,
    pub ceiling_torque_estimate_nm: Option<f32>,
}

/// Motor-level current cap of the gripper, as the driver applies it (`ArmDriver::motor_current_cap`).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct MotorCurrentCap {
    pub joint: usize,
    /// `pos_force` (capped) or `pos_vel` (legacy, no motor current cap).
    pub mode: &'static str,
    /// Configured ceiling (raw, 1/10000 of the maximum phase current); None in `pos_vel`.
    pub current_raw: Option<u16>,
    /// The motor confirmed the capped mode (register reply and readback) at the last enable.
    pub active: bool,
    pub calibrated: bool,
    pub ceiling_torque_nm: Option<f32>,
    pub torque_reference: Option<String>,
    pub close_speed_rad_s: Option<f32>,
    pub note: &'static str,
}

pub struct DamiaoArmDriver<C: CanTransport> {
    can: C,
    /// Checked before each frame of a target batch (`ArmDriver::set_estop_gate`).
    estop_gate: Option<crate::rt_core::urgent::EstopGate>,
    /// Client lease generation stamped onto queued motion target frames (`0` for hold/autonomous).
    output_lease_gen: u64,
    /// Per-call client lease generation checked inside [`Self::send`] (set by `send_with_lease`).
    send_lease_gen: u64,
    /// Atomic E-stop latch (`ArmDriver::set_abort_flag`), checked before enable transmissions.
    abort_flag: Option<Arc<AtomicBool>>,
    /// Supervisor-injected stopped-disable gate enforced at send time for `DisableRequested`.
    stopped_disable_gate: Option<StoppedDisableGate>,
    /// Authoritative packed E-stop state (`ArmDriver::set_estop_state`).
    estop_state: Option<Arc<crate::rt_core::urgent::EstopState>>,
    /// Strict non-sleeping RT mode (`ArmDriver::set_rt_mode`), disables retry sleeps on WouldBlock.
    rt_no_sleep: bool,
    motor_ids: Vec<u16>,
    master_id: u32,
    /// Explicit master CAN ID from [`Self::with_master_id`]; `None` uses Damiao's per-motor `0x10 + motor_id`.
    configured_master_id: Option<u32>,
    /// MIT / feedback mapping range per joint (must equal each motor's PMAX / VMAX / TMAX).
    limits: Vec<DamiaoLimits>,
    control_mode: DamiaoControlMode,
    default_vlim: f32,
    default_kp: f32,
    default_kd: f32,
    cached_states: Vec<JointState>,
    last_target_frames: Vec<Option<CanFrame>>,
    /// Last MIT `(kp, kd, torque)` parameters transmitted per joint (used to rebuild fresh
    /// gravity-compensation recovery frames from live measured position without replaying bytes).
    last_mit_params: Vec<Option<(f32, f32, f32)>>,
    last_feedback_instant: Vec<Option<Instant>>,
    /// Most recent decoded feedback instant per joint, preserved across `send_enable()` (which
    /// clears `last_feedback_instant` to gate post-enable state-1 confirmation) and cleared on
    /// `awaiting_rx_boundary` reset so Rule A′ send-time pose checks during enable rollback still
    /// observe provably fresh (`<= 100 ms`) pose telemetry.
    pose_feedback_instant: Vec<Option<Instant>>,
    feedback_seq: Vec<u64>,
    /// Instant taken before the last receive call that found the RX queue empty.
    rx_empty_instant: Option<Instant>,
    /// `rx_empty_instant` when each joint's last sample was decoded.
    feedback_queue_bound: Vec<Option<Instant>>,
    motor_status: Vec<u8>,
    is_enabled: bool,
    pending_rearm: bool,
    last_query_instant: Option<Instant>,
    last_enable_command: Option<Instant>,
    /// `rx_seq` snapshot when `last_enable_command` was recorded so pre-command feedback cannot
    /// satisfy `enable_state()` even if a virtual clock does not advance between receive and send.
    last_enable_command_rx_seq: Option<u64>,
    /// True when `last_enable_command` was recorded by a disable command (as opposed to enable).
    last_command_was_disable: bool,
    awaiting_rx_boundary: bool,
    is_enabling: bool,
    last_warn_sec: AtomicU64,
    /// Confirmed control mode per joint; None = unconfirmed after a failed switch.
    joint_modes: Vec<Option<DamiaoControlMode>>,
    /// Per-joint control mode verified from a causal `RID_CTRL_MODE` readback (or confirmed
    /// `RID_CTRL_MODE` write reply) after this daemon start (E4). Starts `None` for every joint;
    /// Damiao holds are sent ONLY in a verified mode, never assumed `PosVel`.
    verified_modes: Vec<Option<DamiaoControlMode>>,
    /// Open RX sequence boundary for on-tick `RID_CTRL_MODE` readback requests per joint (L2).
    /// Only causal post-boundary, post-settle `REG_READ(RID_CTRL_MODE)` replies advance
    /// `verified_modes[i]`.
    mode_readback_boundary: Vec<Option<u64>>,
    /// Bitmask of joints with an unconfirmed mode-changing `REG_WRITE(RID_CTRL_MODE)` that may
    /// have reached the wire (including failed flush). Cleared only on causal confirmation.
    mode_write_uncertain_mask: u32,
    /// Bitmask of joints currently owned by an active `ModeSwitch` transaction (suppresses
    /// legacy `mode_readback_boundary` auto-verification in `process_rx_frame`).
    mode_switch_active_mask: u32,
    /// Per-joint hold targets used by `mode_switch_service_watchdog_holds` during an active
    /// `ModeSwitch` transaction (preserves exact MIT `kp`/`kd`/`torque` hold parameters, 0 alloc).
    mode_switch_hold_targets: [Option<ModeSwitchHoldTarget>; MAX_MODE_JOINTS],
    /// RX sequence boundary captured at the start of an active budgeted MIT expiry recovery
    /// (`mit_expiry_recover_budgeted`), cleared on completion or reset.
    mit_expiry_budgeted_boundary: Option<u64>,
    /// True while either legacy (`mit_expiry_recover_with_gate`) or executor
    /// (`mit_expiry_recover_budgeted`, including initial drain) MIT expiry recovery is active,
    /// keeping provisional `state == 0` feedback from clearing `motor_energized_mask` or
    /// `motor_last_ctrl_tx` before full two-round confirmation and enabling per-frame causal
    /// probe/hold reply matching on `mit_expiry_axes`.
    mit_expiry_recovery_active: bool,
    /// Bitmask of joints that have had `CMD_CLEAR_FAULT` sent during budgeted MIT expiry recovery.
    mit_expiry_cleared_mask: u64,
    /// Per-joint sequential causal disable confirmation state shared by both legacy and executor
    /// MIT expiry recovery: `CMD_DISABLE` -> `REG_READ(RID_CTRL_MODE)` barrier -> Round 1 `0xCC`
    /// (`state == 0`) -> Round 2 `0xCC` (`state == 0`), draining RX right before each axis's
    /// probe submission (0 alloc).
    mit_expiry_axes: [MitExpiryAxisState; MAX_MODE_JOINTS],
    /// Most recent `rx_seq` at which each joint reported an adverse (`state_or_error != 0`)
    /// feedback frame (recorded before timestamp filtering so untimestamped adverse frames also
    /// veto disable confirmation).
    last_adverse_feedback_seq: [u64; MAX_MODE_JOINTS],
    /// Per-release RX frame counter shared across Step 2 RX, transition polls, ModeSwitch, MIT
    /// recovery, and commissioning (one release ledger).
    release_rx_frames: u16,
    /// Per-release RX frame cap tightened by the owner's [`crate::rt_core::budget::ReleaseBudget`] (default 64).
    release_rx_limit: u16,
    /// Optional per-release budgeted TX cap tightened by the owner's [`crate::rt_core::budget::ReleaseBudget`].
    release_tx_limit: Option<u16>,
    /// Bitmask of axes causally confirmed disabled during the active MIT expiry recovery attempt
    /// so `watchdog_continuity_reserved_mask` and `service_mit_recovery_continuity_holds_inner`
    /// share the exact same due-axis eligibility predicate.
    mit_expiry_confirmed_disabled_mask: u64,
    /// Per-joint `CLOCK_REALTIME - monotonic` decode offset (ns) of the latest causal feedback.
    last_feedback_decode_offset_ns: Vec<Option<i128>>,
    /// Per-joint causal `REG_READ(RID_CTRL_MODE)` probe boundary
    /// `(after_seq, tx_at, retry_at, deadline, tx_offset_ns)` used by SAFE-HOLD / stopped recovery
    /// for joints in `mode_write_uncertain_mask` (0 alloc).
    uncertain_readback_probe: [Option<(u64, Instant, Instant, Instant, Option<i128>)>; MAX_MODE_JOINTS],
    /// PMAX / VMAX / TMAX read back from the motor and equal to `limits[i]`.
    mit_limits_verified: Vec<bool>,
    /// Opt-in expected Damiao `RID_TIMEOUT` counts per joint (`None` = any fresh reply accepted).
    expected_timeout_counts: Vec<Option<u32>>,
    /// True when pre-enable `RID_TIMEOUT` readback verification is enabled (via
    /// [`Self::with_timeout_verification`] or [`Self::with_expected_timeout_counts`]).
    timeout_check_enabled: bool,
    /// Per-joint causal `RID_TIMEOUT` verification state (cleared on startup and after BusLoss).
    timeout_verified: Vec<bool>,
    /// True when startup or post-BusLoss register verification is pending: next enable must run
    /// fresh causal `CTRL_MODE` and `TIMEOUT` readbacks across all motors and only clears upon
    /// confirmed enable ([`Self::finish_enable_confirmed`]).
    startup_register_check_pending: bool,
    /// Bitmask of joints whose `verified_modes[j]` transitioned from unverified to verified during
    /// the current enable attempt so an aborted startup/post-BusLoss attempt discards them.
    attempt_verified_modes_mask: u32,
    /// Staged quantized wire target positions `(tx_at, motor_space_rad)` awaiting `can.flush()`.
    staged_wire_pos: [Option<(Instant, f32)>; MAX_MODE_JOINTS],
    staged_wire_pos_mask: u32,
    /// Last wire-committed target position per joint as `(commit_seq, tx_at, motor_space_rad)`,
    /// recorded strictly at confirmed transmit/flush for non-skipped PosVel/MIT control frames.
    committed_wire_pos: [Option<(u64, Instant, f32)>; MAX_MODE_JOINTS],
    wire_pos_commit_seq: u64,
    /// Register replies seen by `read_joint_states` since the last clear.
    register_replies: Vec<damiao::RegisterReply>,
    /// Last mapping-range check (`check_mapping_ranges`); None = not run.
    range_check: Option<DriverRangeCheck>,
    /// Joints that received an autonomous recovery write during `read_joint_states`
    /// since the supervisor last queried `take_recovery_writes`.
    recovery_written_joints: Vec<usize>,
    /// `ArmDriver::set_strict_mode_handover`; false keeps the e4f72b6 `write_modes`.
    strict_mode_handover: bool,
    /// Register replies with the CAN ID they arrived on and their receive instant (MIT
    /// identity readback and TIMEOUT commissioning).
    register_reply_log: Vec<(damiao::RegisterReply, u32, Instant, u64, u64)>,
    /// Preallocated inline register reply buffer for RT commissioning (zero heap alloc/free).
    comm_reg_replies: [Option<crate::mit::CommissioningRegReply>; 64],
    comm_reg_reply_len: usize,
    /// True if more than 64 register replies arrived since the last `commissioning_clear_register_log`;
    /// forces `commissioning_find_register_reply` to fail closed as `Ambiguous`.
    comm_reg_reply_overflow: bool,
    /// Test-only hook invoked strictly AFTER section-6 pre-TX bookkeeping and immediately BEFORE
    /// the final authority/deadline check and `can.transmit(&frame)`.
    before_final_tx_gate_hook: Option<std::sync::Arc<dyn Fn(&CanFrame) + Send + Sync>>,
    /// RX sequence number of the last observed-empty receive queue: every frame dequeued after
    /// it (sequence number larger) was enqueued after that observation.
    rx_empty_seq: u64,
    /// The last `read_joint_states` observed an empty receive queue.
    last_read_drained: bool,
    /// Deterministic clock of the CAN transport (tests); None = the system clock.
    clock: Option<std::sync::Arc<dyn crate::transport::DriverClock>>,
    /// Sequence number of the last processed RX frame, and of each joint's last feedback.
    rx_seq: u64,
    /// Exact `(rx_seq, now)` mark captured immediately before `can.transmit(&frame)` on the most
    /// recent successful `send_inner` call (with zero intervening transport callbacks or hooks).
    last_tx_mark: Option<(u64, Instant)>,
    last_feedback_seq: Vec<u64>,
    /// Incremented whenever joint `i`'s feedback continuity breaks (`> 100 ms` gap or non-finite).
    feedback_continuity_epoch: Vec<u64>,
    /// Per-motor causal boundary captured after the last motion/enable write returns from
    /// `can.transmit()`. Used ONLY inside the rest-pose disable gate.
    rest_pose_boundary: Vec<Option<RestPoseBoundary>>,
    /// Active control-release id distinguishing separate ticks for the rest-pose K=3 window.
    control_release_id: u64,
    /// True once the owner has called `begin_control_release`, preventing receive passes from
    /// advancing `control_release_id` independently.
    control_release_managed: bool,
    /// Per-motor K=3 causal confirmation window for rest pose (the ONLY strict evidence gate).
    rest_pose_window: Vec<CausalWindow>,
    /// `rx_seq` snapshot when `CMD_ENABLE` was transmitted for motor `i`.
    motor_enable_seq: Vec<u64>,
    /// Per-axis `(rx_seq, tx_started)` boundary captured at the most recent `CMD_ENABLE`
    /// submission so ordinary disarm confirmation requires a fresh post-enable `state == 0` reply.
    motor_last_enable_tx: Vec<Option<(u64, Instant)>>,
    /// True once motor `i` has produced a causal `state == 0` feedback frame after its latest
    /// `CMD_ENABLE` submission (cleared on any `CMD_ENABLE` or non-zero `state_or_error`).
    motor_disarmed_after_enable: Vec<bool>,
    /// True once motor `i` has received feedback strictly after its latest `CMD_ENABLE`
    /// transmit (any state, including fault states, so `poll_enable` detects faults causally).
    motor_post_enable_feedback: Vec<bool>,
    /// Monotonic counter bumped strictly when a post-enable motor feedback frame carries
    /// `state_or_error != 1` (R12 M2).
    reported_torque_loss_sample_seq: u64,
    /// True while a `ProcessExit` panic-disable sequence is active and awaiting causal per-axis
    /// `state == 0` feedback replies after `CMD_DISABLE` transmission.
    panic_disable_active: bool,
    /// Per-axis `(rx_seq, tx_started)` boundary captured when a flushed `CMD_DISABLE` was sent
    /// after an empty RX boundary during `panic_disable_step`.
    panic_disable_tx_boundary: Vec<Option<(u64, Instant)>>,
    /// Per-axis causal confirmation that a fresh `state == 0` reply arrived after the axis's
    /// flushed `panic_disable_step` `CMD_DISABLE` transmission.
    panic_disable_confirmed: Vec<bool>,
    /// True while `probe_hardware_disarmed_until` is actively polling read-only `0xCC` status.
    disarm_probe_active: bool,
    /// Per-axis bitmask of axes that produced a non-disabled (`state != 0`) post-boundary feedback
    /// frame during an active `probe_hardware_disarmed_until` pass so a later `state == 0` frame
    /// in the same drain can never mask or cache proof for any adverse axis.
    disarm_probe_adverse_mask: u32,
    /// First non-disabled (`state != 0`) state recorded per axis during an active
    /// `probe_hardware_disarmed_until` pass.
    disarm_probe_adverse_state: [u8; MAX_MODE_JOINTS],
    /// MIT tracking deadline fence (`set_mit_deadline_fence`); None = off (unchanged behaviour).
    mit_fence: Option<Duration>,
    /// Per joint: when its last confirmed command started out (the earliest frame of it, never
    /// its completion), while the fence is armed. The output gap is measured from the oldest.
    mit_fence_last_cmd: Vec<Option<Instant>>,
    /// Trip reason and instant; refuses writes until a confirmed enable (zero allocation).
    mit_fence_trip: Option<(MitFenceTripReason, Instant)>,
    /// No autonomous frame (recovery, status poll) this long after a trip.
    mit_fence_holdoff: Duration,
    /// The operator cleared the trip (`acknowledge_mit_fence_trip`): polls may resume.
    mit_fence_acked: bool,
    /// True while supervisor MIT ownership is `Lost`: `verified_hold_mode` rejects MIT frames
    /// so MIT-configured joints fall back to verified `PosVel` holds or stay unheld under L2.
    mit_ownership_lost: bool,
    /// RX sequence boundary captured at the start of the most recent [`ArmDriver::mit_expiry_recover`]
    /// request; final MIT clear requires every joint's `last_feedback_seq` to exceed this boundary.
    last_mit_recovery_rx_seq: Option<u64>,
    /// MIT tracking armed a fence since the last confirmed enable: base-mode restores use POS_VEL
    /// frames only.
    mit_tracking_used: bool,
    /// Pending torque transition (`begin_torque` / `poll_torque`); None = idle.
    torque_hs: Option<DamiaoTorque>,
    /// Bitmask of 0-based motor indices that have already received `CMD_DISABLE` in an active
    /// or partial disable sequence (R12 B3). Cleared on full disable completion or enable.
    disable_dispatched_mask: u64,
    /// True once K=3 causal rest-pose evidence on J1–J6 has been confirmed BEFORE the first
    /// `CMD_DISABLE` frame of the active `DisableRequested` sequence.
    disable_rest_established: bool,
    /// Optional absolute driver-clock authority deadline for dispatching undispatched motors
    /// during a multi-tick `DisableRequested` sequence.
    disable_tx_deadline: Option<Instant>,
    /// Optional admitted client lease generation checked before dispatching undispatched motors
    /// during a multi-tick `DisableRequested` sequence.
    disable_lease_gen: Option<u64>,
    /// Service deadline capping the receive pass of the current `poll_torque` (None: 5 ms only).
    rx_deadline: Option<Instant>,
    /// True while inside `poll_torque_inner` so its internal `receive_feedback` calls do not
    /// reset the single TX budget pass opened at `poll_torque_inner` entry.
    in_poll_torque: bool,
    /// Frame limit capping the receive pass of the current budgeted read/poll (None: 64 frames).
    rx_frame_limit: Option<u16>,
    /// `(frames_processed, budget_exhausted)` of the most recent `receive_feedback` pass.
    last_poll_rx: Option<(u16, bool)>,
    /// Base (position-control) mode per joint: `control_mode`, or `PosForce` for the gripper.
    base_modes: Vec<DamiaoControlMode>,
    /// Gripper in position-force mode (None: every joint uses `control_mode`, legacy).
    pos_force: Option<GripperPosForce>,
    /// The gripper's CTRL_MODE 4 was read back at the last full enable.
    pos_force_verified: bool,
    /// Legacy profile: gripper joint driven in POS_VEL (reported without a motor current cap).
    legacy_gripper: Option<usize>,
    /// First timely post-boundary 0xCC refresh sample captured while a refresh is active.
    refresh_capture: Option<RefreshCapture>,
    /// Per-tick bounded TX budget (attempts, start-cutoff duration, inclusive frame expiry).
    tx_budget: TxBudget,
    /// Inline fixed-capacity queue of unsent motion frames carried to the next tick (zero alloc).
    pending_tx: PendingTxQueue,
    /// Inline fixed-capacity queue of unsent stopped-hold frames carried across ticks (separate
    /// from `pending_tx` so `strict_stop_check` while latched in E-stop never wipes hold carryover).
    pending_hold_tx: PendingTxQueue,
    /// Round-robin start joint for stopped-hold batches so slow-cadence partial passes never starve
    /// higher-index joints.
    hold_cursor: usize,
    /// True if the most recent `write_estop_hold_bounded` call transmitted at least one frame.
    last_hold_tx_sent_any: bool,
    /// Bitmask of 0-based joint indices whose motion target transmitted & flushed in the most
    /// recent `write_joint_targets_bounded` / `drain_pending_tx` pass with no frame still queued.
    last_target_tx_sent_mask: u64,
    /// Observed age of the oldest frame expired in the most recent bounded TX pass.
    last_expired_oldest_age: Option<Duration>,
    /// Shared TX pass state across `receive_feedback()`, J7 handshakes, and `write_joint_targets*()` in one tick.
    tick_tx: Option<TickTxPass>,
    /// Unified owner-local `0xCC` refresh scheduler (fixed `u64` bitmask + round-robin cursor,
    /// at most one `0xCC` slot per tick across enable, FailedHold, Idle-torqued, and stopped).
    refresh_scheduler: RefreshScheduler,
    /// Stop/E-stop generation stamp captured when an enable handshake starts so an applied stop
    /// (`requested == applied`) mid-handshake also inhibits subsequent enable frames.
    enable_stop_stamp: Option<StopStamp>,
    /// Most recent provably fresh (`queue_bound.is_some()` + finite position) feedback
    /// `(rx_at, queue_bound)` decoded for the position-force gripper. Unlike
    /// `last_feedback_instant` (cleared by `send_enable` to enforce post-enable state-1 replies),
    /// this preserves live J7 telemetry age across carried and retried `CMD_ENABLE` frames.
    pf_last_fresh_feedback: Option<(Instant, Instant)>,
    /// Last provably fresh (`<= 100 ms`, bounded, finite) measured J7 position decoded for the
    /// position-force gripper; preserved across staleness so J7 freeze frames always hold the
    /// last known fresh position with `vel_raw = 0` and the configured current cap.
    pf_last_fresh_position: Option<f32>,
    /// Optional absolute driver-clock ceiling on the active control release's TX start-cutoff
    /// (`release_deadline - publish_margin`), set during a release and cleared afterwards.
    tx_release_deadline: Option<Instant>,
    /// Optional absolute driver-clock authority ceiling (`min(lease_expiry, request_deadline)`)
    /// checked immediately before transmitting every `CMD_ENABLE` frame.
    enable_tx_deadline: Option<Instant>,
    /// Optional admitted client lease generation checked against the atomic lease fence
    /// immediately before transmitting every `CMD_ENABLE` frame.
    enable_lease_gen: Option<u64>,
    /// True once at least one `CMD_ENABLE` frame has been transmitted in the current enable attempt.
    enable_frames_sent_any: bool,
    /// Active/confirmed CAN transmit probe recorded for clearing `CAN_TX_STALLED` (zero alloc).
    tx_probe: Option<TxProbeState>,
    /// Attempt-scoped J7 position-force freshness status observed in the most recent bounded TX pass.
    last_tx_pf_status: PosForceTxStatus,
    /// Monotonic generation incremented each time a provably fresh (`<= 100 ms`, bounded, finite)
    /// J7 position-force feedback sample is decoded. Binds owner-side staleness episodes to the
    /// actual feedback sample so a fresh RX sample between two stale output passes never carries
    /// over an older stale timer.
    pf_fresh_gen: u64,
    /// True when the active `PosForce` joint needs a read-only `0xCC` refresh.
    pf_refresh_pending: bool,
    /// Consecutive ticks in which a needed J7 `0xCC` refresh was deferred by TX budget/backpressure;
    /// when `>= 1`, the next tick prioritizes J7's `0xCC` ahead of competing frames so refresh never starves.
    pf_refresh_deferred_ticks: u8,
    /// True when an output pass ran since the previous `receive_feedback()` pass.
    output_ran_since_rx: bool,
    /// True when the most recent bounded output pass was a stopped hold (`write_estop_hold_bounded`),
    /// so `receive_feedback()` yields the shared TX budget to due J1–J6 stopped holds instead of
    /// preempting them with a J7 `0xCC` refresh.
    last_output_was_hold: bool,
    /// Bitmask of `(joint, slot)` stopped-hold frames (`1u64 << (joint * 2 + slot)`) that were
    /// due in the most recent `write_estop_hold_bounded` pass (`0` when nothing was due or
    /// stop-inhibited).
    last_hold_tx_due_mask: u64,
    /// Bitmask of `(joint, slot)` stopped-hold frames (`1u64 << (joint * 2 + slot)`) that
    /// actually reached the transport (transmitted and flushed) in the most recent
    /// `write_estop_hold_bounded` pass.
    last_hold_tx_sent_mask: u64,
    /// When `true` (installed by `SupervisorInner` on both Legacy and RT Executor), control-tick
    /// arm write paths and `receive_feedback` NEVER write `0x307` frames; `SupervisorInner`'s
    /// tick epilogue calls [`ArmDriver::emit_pos_force_j7`] once per tick.
    external_pos_force_emitter: bool,
    /// Consecutive ticks in which the J7 `PosForce` emitter was deferred by TX budget or
    /// backpressure; when `>= 1`, arm writes reserve a TX budget slot for J7 so J7 never starves
    /// under tight per-tick frame budgets (e.g. 6 frames for 7 motors).
    pf_deferred_ticks: u8,
    /// True when J7's motion target was deferred and awaits transmission on the next tick.
    pf_deferred_motion: bool,
    /// Driver-clock timestamp when J7's current deferral streak began (used for `oldest_pending_tx_age`
    /// only while J7 feedback is fresh; stale J7 never counts as FIFO-expired).
    pf_deferred_since: Option<Instant>,
    /// Last deferred J7 intent (used by direct-driver `drain_pending_tx`).
    pf_deferred_intent: Option<PosForceIntent>,
    /// Per-`(joint, slot)` unchanged-hold state (`true` once a stop/hold/freeze frame has been
    /// flushed for that slot and no motion frame has superseded it; `false` after enable or any
    /// flushed motion frame). Drives TX Class 1 (motion/non-hold -> stop / hold / freeze) vs
    /// Class 3 (re-send of an unchanged hold) across ALL joints including J7.
    slot_in_unchanged_hold: [bool; MAX_MODE_JOINTS * 2],
    /// Monotonic counter incremented on every flushed joint TX frame (J1–J6 slots and J7).
    tx_progress_seq: u64,
    /// Per-`(joint, slot)` sequence number of the last flushed TX frame (`joint * 2 + slot`;
    /// smallest = waited longest = oldest pending first round-robin within each priority class
    /// across J1–J6 POS_VEL/MIT slots and J7).
    slot_last_tx_seq: [u64; MAX_MODE_JOINTS * 2],
    /// Hint set by `SupervisorInner` before arm output indicating whether this tick's upcoming
    /// J7 intent is `Hold`/`Stop` (rather than a `Target`). Cleared after `pos_force_emitter::emit_tick`.
    pf_next_intent_is_hold: bool,
    /// Latched while `pf_last_fresh_position == None` under `NoOutputUntilFresh`: forces the
    /// first `0x307` emission after the first provably fresh J7 sample arrives to be a stationary
    /// `Hold` at that measured position (`vel_raw = 0`, configured `current_raw`).
    pf_hold_on_first_fresh: bool,
    /// Inline fixed-capacity shadow observation staging (zero alloc, zero lock).
    #[cfg(not(target_arch = "wasm32"))]
    shadow_staging: crate::torque::shadow::ShadowDriverStaging,
    #[cfg(not(target_arch = "wasm32"))]
    defer_disable_shadow: bool,
    #[cfg(not(target_arch = "wasm32"))]
    shadow_disable_pass_sent_mask: u64,
    #[cfg(not(target_arch = "wasm32"))]
    pending_disable_shadow: Option<PendingDisableShadowBatch>,
    /// Bitmask of joints (`1u32 << idx`) that have received a `CMD_ENABLE` frame (or reported
    /// `state == 1`) and have not subsequently received `CMD_DISABLE` or causal post-enable
    /// `state == 0` confirmation. Used to guarantee Damiao `TIMEOUT` watchdog hold continuity
    /// (`<= 40 ms` between control frames) across multi-tick handshakes and blocking loops.
    motor_energized_mask: u32,
    /// Driver-clock timestamp of the most recent watchdog-refreshing control frame (`POS_VEL`,
    /// `MIT`, `POS_FORCE`) or `CMD_ENABLE` successfully transmitted to joint `idx`.
    motor_last_ctrl_tx: [Option<Instant>; MAX_MODE_JOINTS],
    /// Last finite measured position observed for joint `idx` (never cleared by transient
    /// staleness) so energized non-`PosForce` arm joints always have a stationary hold target.
    motor_keepalive_hold_pos: [Option<f32>; MAX_MODE_JOINTS],
    /// Bitmask of joints (`1u32 << idx`) that already received a confirmed watchdog-refreshing
    /// control frame (`POS_VEL`, `MIT`, `POS_FORCE`, or initial `CMD_ENABLE`) during the current
    /// control release (`control_release_id`). Prevents duplicate control/hold frames per release.
    release_ctrl_tx_mask: u32,
    /// Staged watchdog-refreshing control TX timestamps awaiting confirmed `can.flush()` on
    /// real-flush (`!flush_is_noop()`) transports. Committed atomically on flush success and
    /// discarded on any pre-flush error, cutoff, stop, or flush failure.
    staged_ctrl_tx: [Option<Instant>; MAX_MODE_JOINTS],
    /// Bitmask of joints (`1u32 << idx`) present in `staged_ctrl_tx`.
    staged_ctrl_tx_mask: u32,
    /// True from `begin_control_release` through Window 1 Step 5 while a subsequent normal/hold
    /// output step in THIS release still owns watchdog servicing. Cleared on Window 2 wakes,
    /// end-of-release fallback, and release-cap teardown so off-tick and Window 2 confirmations
    /// never skip inline watchdog holds.
    output_step_pending_in_release: bool,
    /// `control_release_id` for which the current per-release TX pass (`tick_tx`,
    /// `refresh_scheduler`, `release_ctrl_tx_mask`, `release_recovery_tx_count`) was opened.
    release_tx_pass_release_id: u64,
    /// Autonomous stale-feedback recovery TX frames emitted during the current control release.
    release_recovery_tx_count: u8,
    /// Round-robin cursor across joints for bounded per-release autonomous recovery TX.
    recovery_cursor: usize,
    /// Diagnostic tag distinguishing urgent watchdog continuity holds (`TxFrameKind::UrgentHold`)
    /// from normal stopped / SAFE-HOLD / FailedHold holds (`TxFrameKind::Hold`).
    in_urgent_watchdog_hold: bool,
}

/// Transmit priority classes (lower value = higher priority):
/// 1. Transition from motion to stop / hold / freeze for ANY joint (including J7).
/// 2. Motion targets.
/// 3. Re-sends of an unchanged hold / freeze.
///
/// (Class 4: J7 `0xCC` refresh and other reads — sent only when Classes 1–3 have no pending work
/// and spare TX budget remains.)
const TX_CLASS_STOP_TRANSITION: u8 = 1;
const TX_CLASS_MOTION_TARGET: u8 = 2;
const TX_CLASS_HOLD_RESEND: u8 = 3;

#[derive(Clone, Copy, Debug)]
struct RestPoseBoundary {
    /// Host monotonic instant captured AFTER `can.transmit()` returned `Ok(())`.
    write_done: Instant,
    /// Kernel or transport receive-clock timestamp captured immediately when `can.transmit()`
    /// returned `Ok(())`. Causal rest-pose evidence requires `rx_time > write_done_rx_clock + SETTLE`
    /// on this exact clock domain (`None` fails closed).
    write_done_rx_clock: Option<CanRxTime>,
    /// `CLOCK_REALTIME - CLOCK_MONOTONIC` (ns) sampled at `write_done` when `write_done_rx_clock`
    /// is `KernelRealtimeNs`. A drift `> 1 ms` at decode discards the sample and restarts the window.
    write_done_offset_ns: Option<i128>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RestBoundaryCheck {
    Satisfied,
    InSettle,
    OffsetDrifted,
}

/// Evaluate whether `rx_time` satisfies `boundary` for the rest-pose gate:
/// - On `KernelRealtimeNs`, compares `CLOCK_REALTIME - CLOCK_MONOTONIC` sampled at `write_done`
///   vs. decode; any drift `> MAX_REALTIME_MONOTONIC_OFFSET_DRIFT_NS` (1 ms) or missing offset
///   returns [`RestBoundaryCheck::OffsetDrifted`] so the caller discards and restarts the window.
/// - Compares `rx_time` against `write_done_rx_clock + TORQUE_CAUSAL_SETTLE_WINDOW` in the SAME
///   clock domain AND verifies host monotonic elapsed time `> TORQUE_CAUSAL_SETTLE_WINDOW`.
#[inline]
fn check_rest_pose_boundary(
    boundary: RestPoseBoundary,
    rx_t: CanRxTime,
    decode_now: Instant,
    decode_offset_ns: Option<i128>,
) -> RestBoundaryCheck {
    let Some(tx_t) = boundary.write_done_rx_clock else {
        return RestBoundaryCheck::InSettle;
    };
    match (rx_t, tx_t) {
        (CanRxTime::KernelRealtimeNs(_), CanRxTime::KernelRealtimeNs(_)) => {
            let (Some(w_off), Some(d_off)) = (boundary.write_done_offset_ns, decode_offset_ns) else {
                return RestBoundaryCheck::OffsetDrifted;
            };
            if (d_off - w_off).abs() > MAX_REALTIME_MONOTONIC_OFFSET_DRIFT_NS {
                return RestBoundaryCheck::OffsetDrifted;
            }
        }
        (CanRxTime::Monotonic(_), CanRxTime::Monotonic(_)) => {}
        _ => return RestBoundaryCheck::InSettle,
    }
    if rx_t.is_after_settle(tx_t, TORQUE_CAUSAL_SETTLE_WINDOW)
        && decode_now
            .checked_duration_since(boundary.write_done)
            .is_some_and(|d| d > TORQUE_CAUSAL_SETTLE_WINDOW)
    {
        RestBoundaryCheck::Satisfied
    } else {
        RestBoundaryCheck::InSettle
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CanRxTimeReject {
    Future,
    Unmappable,
}

/// Convert a present kernel/transport [`CanRxTime`] to `(monotonic_rx_at, realtime_minus_monotonic_ns)`.
///
/// Rejects future-dated timestamps with `Err(CanRxTimeReject::Future)` (NEVER clamped to `decode_now`),
/// and missing/unmappable clock samples with `Err(CanRxTimeReject::Unmappable)` in a single clock read.
#[inline]
fn can_rx_time_to_monotonic<C: CanTransport>(
    can: &C,
    rx_time: CanRxTime,
    decode_now: Instant,
) -> Result<(Instant, Option<i128>), CanRxTimeReject> {
    match rx_time {
        CanRxTime::Monotonic(ts) => {
            if ts > decode_now {
                return Err(CanRxTimeReject::Future);
            }
            Ok((ts, None))
        }
        CanRxTime::KernelRealtimeNs(rx_ns) => {
            let sample = can.rx_clock_sample().ok_or(CanRxTimeReject::Unmappable)?;
            let CanRxTime::KernelRealtimeNs(now_ns) = sample.rx_now else {
                return Err(CanRxTimeReject::Unmappable);
            };
            let offset_ns = sample
                .realtime_minus_monotonic_ns
                .ok_or(CanRxTimeReject::Unmappable)?;
            if rx_ns > now_ns {
                return Err(CanRxTimeReject::Future);
            }
            let age_ns = (now_ns - rx_ns).min(i128::from(u64::MAX)) as u64;
            let mapped = decode_now
                .checked_sub(Duration::from_nanos(age_ns))
                .ok_or(CanRxTimeReject::Unmappable)?;
            Ok((mapped, Some(offset_ns)))
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
#[inline]
fn can_rx_reject_to_shadow_stamp(reject: CanRxTimeReject) -> crate::torque::shadow::RxStampKind {
    match reject {
        CanRxTimeReject::Future => crate::torque::shadow::RxStampKind::Future,
        CanRxTimeReject::Unmappable => crate::torque::shadow::RxStampKind::Unmappable,
    }
}

#[cfg(not(target_arch = "wasm32"))]
#[inline]
fn damiao_mode_to_shadow_control_mode(mode: DamiaoControlMode) -> crate::motor_mode::ControlMode {
    match mode {
        DamiaoControlMode::Mit => crate::motor_mode::ControlMode::Impedance,
        DamiaoControlMode::PosVel => crate::motor_mode::ControlMode::PositionVelocity,
        DamiaoControlMode::PosForce => crate::motor_mode::ControlMode::PositionForce,
    }
}

#[cfg(not(target_arch = "wasm32"))]
#[inline]
fn control_mode_to_damiao(mode: crate::motor_mode::ControlMode) -> Option<DamiaoControlMode> {
    match mode {
        crate::motor_mode::ControlMode::Impedance => Some(DamiaoControlMode::Mit),
        crate::motor_mode::ControlMode::PositionVelocity => Some(DamiaoControlMode::PosVel),
        crate::motor_mode::ControlMode::PositionForce => Some(DamiaoControlMode::PosForce),
        crate::motor_mode::ControlMode::Position => None,
    }
}

#[derive(Clone, Copy, Debug)]
struct TxProbeState {
    joint: usize,
    boundary_seq: u64,
    sent_at: Instant,
}

#[derive(Clone, Copy, Debug)]
struct TickTxPass {
    deadline: Instant,
    attempts: usize,
    backpressured: bool,
}

/// Generation stamp captured when a motion frame is admitted into [`PendingTxQueue`].
/// Any E-stop or authorized stop (even after `requested == applied`) changes this stamp so
/// older deferred frames are never transmitted.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct StopStamp {
    estop_gen: u64,
    stop_gen: u64,
}

/// Classification of every Damiao CAN transmit routed through [`DamiaoArmDriver::send`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TxKind {
    /// Requested torque-disable frame (`CMD_DISABLE`): exempt from the motion TX budget;
    /// unconditionally subject in every safety state to send-time K=3 rest-pose evaluation from
    /// fresh (`<= 100 ms`) feedback and the single 1 s request lifetime.
    DisableRequested { started_at: Instant },
    /// Budgeted MIT expiry recovery disable (`CMD_DISABLE`): subject to BOTH the shared
    /// per-release TX budget (`is_disable_tx_kind` returns `false`) AND Rule A′ (`started_at`).
    MitRecoveryDisable { started_at: Instant },
    /// Per-motor disable frame of a client-initiated park (`CMD_DISABLE`): exempt from the motion
    /// TX budget, subject to the exact same K=3 rest-pose gate and 1 s request lifetime as
    /// `DisableRequested`, AND stop-gated (pending Stop/E-stop, E-stop latch, and stop-generation
    /// stamp rechecked immediately before EVERY motor's `can.transmit`).
    ClientParkDisable { started_at: Instant },
    /// Protective torque-off (`CMD_DISABLE`): exempt from the motion TX budget and never
    /// refused by the stopped rest-pose rule or 1 s lifetime. Used ONLY for `Thermal` and
    /// `ProcessExit`.
    DisableProtective { reason: DisableProtectiveReason },
    /// Read-only `0x7FF 0xCC` status probe during `AwaitRestGate` while collecting K=3 rest-pose
    /// evidence: exempt from motion TX budget and stop gate.
    DisableProbe,
    /// Motion target frame (`POS_VEL`, `MIT`, or `POS_FORCE`).
    Target,
    /// Non-rollback mode-switch hold frame.
    Hold,
    /// Base-mode rollback or post-E-stop stationary hold frame for `joint`: refused while a
    /// stop/E-stop is pending; allowed while latched only when `joint` has fresh (`<= 100 ms`)
    /// measured feedback and the hold frame is rebuilt in step 4. Never used for `CTRL_MODE`
    /// register writes (those are always `ModeWrite`).
    RollbackHold(usize),
    /// Owner-captured measured-position stationary hold frame for `joint` (emitted by
    /// `write_measured_hold_masked`): obeys the same stop gate as `RollbackHold` and never resets
    /// `rest_pose_window`, while continuing to hold the owner's last captured finite position if
    /// feedback later goes stale (`JointHoldStatus::HeldStaleTarget`).
    MeasuredHold(usize),
    /// `CTRL_MODE` write or non-read-only configuration write (`RID_TIMEOUT`, `CMD_SET_ZERO`).
    ModeWrite,
    /// Read-only status poll (`0xCC` refresh) or read-only register read: refused while a stop
    /// or E-stop is pending, allowed while latched.
    Refresh,
    /// Autonomous stale-feedback recovery hold in `receive_feedback`.
    Recovery,
    /// Hardware fault-clear frame (`CMD_CLEAR_FAULTS`): refused while a stop or E-stop is
    /// pending, allowed while latched (part of operator-authorized fault clearance).
    ClearFault,
    /// Torque-enable handshake frame (`CMD_ENABLE`, enable-time restore/readback/refresh/hold).
    Handshake,
}

/// Zero-allocation refusal/failure outcome returned by [`DamiaoArmDriver::send`].
#[derive(Debug)]
enum TxRefusal {
    /// Step 1: caller deadline already passed (`deadline.passed(now)`).
    DeadlinePassed,
    /// Step 2: per-tick TX budget exhausted (frame count, pass duration cutoff, or backpressure).
    Deferred,
    /// Step 3: stop or E-stop gate refused this `TxKind` (pending or latched).
    StopInhibited,
    /// Step 3/5: `DisableRequested` while stopped refused by Rule A′ (not at rest / stale / no rest pose).
    TorqueOffWouldDrop,
    /// Step 3/5: `DisableRequested` while stopped is waiting ONLY for an exhausted RX drain or
    /// incomplete K=3 rest-pose evidence window inside the 1 s request lifetime (L1).
    DisableWaitEvidence,
    /// Step 3/5: `DisableRequested` while stopped exceeded its 1 s total request lifetime.
    DisableRequestExpired,
    /// Step 4 (pre-build): active MIT deadline fence tripped.
    MitFenceTripped(MitFenceTripReason),
    /// Step 4/5: active `PosForce` joint feedback aged past `POS_FORCE_FEEDBACK_MAX_AGE` (100 ms)
    /// before/at submission (`can.transmit`); distinct from FIFO `DeadlinePassed` and `BuildRefused`.
    PosForceFeedbackExpired(usize),
    /// Step 4: send-time `build` returned `None` (or latched rollback lacked fresh feedback).
    BuildRefused,
    /// Step 5: stop/E-stop generation stamp changed since admission or operation start.
    StopStampChanged,
    /// Step 3/5: motion or `CMD_ENABLE` frame's client lease generation was fenced
    /// (disconnect/release/preempt/expiry).
    LeaseFenced,
    /// Step 5: `CMD_ENABLE` frame's request/lease authority deadline expired before CAN submission
    /// (terminal; distinct from per-release `DeadlinePassed`).
    EnableAuthorityExpired,
    /// Step 6: underlying CAN transport returned an error (backpressure already marked on pass).
    Transport(TransportError),
}

/// Stop-gate policy for a mode-write transaction (`write_modes_send` / `write_modes_confirmed`).
#[derive(Clone, Copy, Debug)]
enum ModeWritePolicy {
    /// Torque-enable base-mode restore handshake: enforces strict pending + latched +
    /// enable-stamp checks and tears down `torque_hs` on inhibition.
    EnableRestore,
    /// Forward mode switch (or non-base-restore rollback): refuses any transmit while a
    /// stop or E-stop is pending, latched, or advances past the captured stamp.
    Strict(StopStamp),
    /// Base-mode rollback (every changing joint restoring `base_mode(i)`):
    /// refused while a stop/E-stop is pending; allowed while latched ONLY with fresh
    /// (`<= 100 ms`) measured feedback, rebuilding every hold at the fresh measured position.
    BaseRollback { start_stamp: StopStamp, force_fresh_hold: bool },
}

/// Unencoded per-joint target parameters stored in [`PendingTxQueue`] so deferred frames are
/// always built at send time inside [`DamiaoArmDriver::send`] (never resending a cached frame).
#[derive(Clone, Copy, Debug)]
enum PendingTargetCmd {
    PosVel { position: f32 },
    Mit { position: f32, velocity: Option<f32>, kp: Option<f32>, kd: Option<f32>, torque: Option<f32> },
    RollbackHold { kp: Option<f32>, kd: Option<f32>, torque: Option<f32>, slot: u8 },
    MeasuredHold { position: f32, kp: Option<f32>, kd: Option<f32>, torque: Option<f32> },
}

#[derive(Clone, Copy, Debug)]
struct PendingTxFrame {
    joint: usize,
    frame_id: u32,
    cmd: PendingTargetCmd,
    queued_at: Instant,
    stop_stamp: StopStamp,
    /// Client lease generation under which this motion target was produced (`0` for hold/autonomous).
    lease_gen: u64,
    /// Counted once in `tx_deferred` per command instance; reset on in-place target replacement.
    was_deferred: bool,
}

/// Inline fixed-size FIFO queue of unsent target frames (zero heap allocation).
struct PendingTxQueue {
    buf: [Option<PendingTxFrame>; DAMIAO_PENDING_TX_CAPACITY],
    len: usize,
}

impl PendingTxQueue {
    const fn new() -> Self {
        Self { buf: [None; DAMIAO_PENDING_TX_CAPACITY], len: 0 }
    }

    #[inline]
    fn len(&self) -> usize { self.len }

    #[inline]
    fn front(&self) -> Option<PendingTxFrame> {
        if self.len > 0 { self.buf[0] } else { None }
    }

    fn pop_front(&mut self) -> Option<PendingTxFrame> {
        if self.len == 0 { return None; }
        let head = self.buf[0];
        for i in 1..self.len {
            self.buf[i - 1] = self.buf[i];
        }
        self.len -= 1;
        self.buf[self.len] = None;
        head
    }

    /// Peek the highest-priority stopped-hold entry ordered by
    /// `(priority_class, slot_last_tx_seq[joint * 2 + slot])` (Class 1 transition before Class 3
    /// unchanged-hold re-send; within each class, oldest waiting slot first, FIFO tie-break).
    fn peek_best_hold(
        &self,
        slot_in_unchanged_hold: &[bool; MAX_MODE_JOINTS * 2],
        slot_last_tx_seq: &[u64; MAX_MODE_JOINTS * 2],
    ) -> Option<(usize, PendingTxFrame, u8, u64)> {
        let mut best: Option<(usize, PendingTxFrame, u8, u64)> = None;
        for (idx, slot_opt) in self.buf[..self.len].iter().enumerate() {
            let Some(entry) = *slot_opt else { continue };
            let sub = match entry.cmd {
                PendingTargetCmd::RollbackHold { slot, .. } => usize::from(slot).min(1),
                _ => 0,
            };
            let flat = entry.joint.saturating_mul(2).saturating_add(sub);
            let class = if flat < slot_in_unchanged_hold.len() && slot_in_unchanged_hold[flat] {
                TX_CLASS_HOLD_RESEND
            } else {
                TX_CLASS_STOP_TRANSITION
            };
            let seq = slot_last_tx_seq.get(flat).copied().unwrap_or(0);
            let better = match best {
                None => true,
                Some((_, _, best_class, best_seq)) => (class, seq) < (best_class, best_seq),
            };
            if better {
                best = Some((idx, entry, class, seq));
            }
        }
        best
    }

    fn remove_at(&mut self, idx: usize) -> Option<PendingTxFrame> {
        if idx >= self.len {
            return None;
        }
        let item = self.buf[idx];
        for i in (idx + 1)..self.len {
            self.buf[i - 1] = self.buf[i];
        }
        self.len -= 1;
        self.buf[self.len] = None;
        item
    }

    fn clear(&mut self) -> usize {
        let dropped = self.len;
        for slot in &mut self.buf[..self.len] {
            *slot = None;
        }
        self.len = 0;
        dropped
    }

    fn discard_joint(&mut self, joint: usize) -> usize {
        let mut write = 0usize;
        let mut dropped = 0usize;
        for read in 0..self.len {
            if let Some(entry) = self.buf[read] {
                if entry.joint == joint {
                    dropped += 1;
                } else {
                    self.buf[write] = Some(entry);
                    write += 1;
                }
            }
        }
        for slot in &mut self.buf[write..self.len] {
            *slot = None;
        }
        self.len = write;
        dropped
    }

    fn find_key(&self, joint: usize, frame_id: u32) -> Option<usize> {
        self.buf[..self.len].iter().position(|slot| {
            slot.is_some_and(|e| e.joint == joint && e.frame_id == frame_id)
        })
    }

    /// Drop entries with `now - queued_at >= max_age` (inclusive), compacting in place.
    /// Returns `(dropped_count, oldest_expired_age)`.
    fn drop_stale(&mut self, now: Instant, max_age: Duration) -> (usize, Option<Duration>) {
        let mut write = 0usize;
        let mut dropped = 0usize;
        let mut oldest: Option<Duration> = None;
        for read in 0..self.len {
            if let Some(entry) = self.buf[read] {
                let age = now.saturating_duration_since(entry.queued_at);
                if age >= max_age {
                    dropped += 1;
                    oldest = Some(oldest.map_or(age, |o| o.max(age)));
                } else {
                    self.buf[write] = Some(entry);
                    write += 1;
                }
            }
        }
        for slot in &mut self.buf[write..self.len] {
            *slot = None;
        }
        self.len = write;
        (dropped, oldest)
    }

    /// Maximum age among currently queued entries, or `None` when empty.
    fn oldest_age(&self, now: Instant) -> Option<Duration> {
        self.buf[..self.len]
            .iter()
            .flatten()
            .map(|e| now.saturating_duration_since(e.queued_at))
            .max()
    }

    /// Drop entries whose stop generation stamp no longer matches `current`, compacting in place.
    fn drop_stopped(&mut self, current: StopStamp) -> usize {
        let mut write = 0usize;
        let mut dropped = 0usize;
        for read in 0..self.len {
            if let Some(entry) = self.buf[read] {
                if entry.stop_stamp != current {
                    dropped += 1;
                } else {
                    self.buf[write] = Some(entry);
                    write += 1;
                }
            }
        }
        for slot in &mut self.buf[write..self.len] {
            *slot = None;
        }
        self.len = write;
        dropped
    }

    /// True if any queued entry carries a stop generation stamp older than `current`.
    fn has_stopped_entry(&self, current: StopStamp) -> bool {
        self.buf[..self.len]
            .iter()
            .any(|slot| slot.is_some_and(|e| e.stop_stamp != current))
    }

    /// Drop entries whose client lease generation has been fenced, compacting in place.
    fn drop_fenced_lease(&mut self, mut is_fenced: impl FnMut(u64) -> bool) -> usize {
        let mut write = 0usize;
        let mut dropped = 0usize;
        for read in 0..self.len {
            if let Some(entry) = self.buf[read] {
                if is_fenced(entry.lease_gen) {
                    dropped += 1;
                } else {
                    self.buf[write] = Some(entry);
                    write += 1;
                }
            }
        }
        for slot in &mut self.buf[write..self.len] {
            *slot = None;
        }
        self.len = write;
        dropped
    }

    /// Remove every queued entry targeting `joint`, compacting in place (zero allocation).
    fn remove_joint(&mut self, joint: usize) {
        let mut write = 0usize;
        for read in 0..self.len {
            if let Some(entry) = self.buf[read] {
                if entry.joint != joint {
                    self.buf[write] = Some(entry);
                    write += 1;
                }
            }
        }
        for slot in &mut self.buf[write..self.len] {
            *slot = None;
        }
        self.len = write;
    }

    /// Drop every queued entry targeting `joint`, compacting in place (0 heap allocs).
    fn drop_joint(&mut self, joint: usize) -> usize {
        let mut write = 0usize;
        let mut dropped = 0usize;
        for read in 0..self.len {
            if let Some(entry) = self.buf[read] {
                if entry.joint == joint {
                    dropped += 1;
                } else {
                    self.buf[write] = Some(entry);
                    write += 1;
                }
            }
        }
        for slot in &mut self.buf[write..self.len] {
            *slot = None;
        }
        self.len = write;
        dropped
    }

    /// True if any queued entry targets `joint`.
    fn has_joint(&self, joint: usize) -> bool {
        self.buf[..self.len]
            .iter()
            .any(|slot| slot.is_some_and(|e| e.joint == joint))
    }

    /// Drop every queued entry whose joint bit is NOT set in `hold_mask`, compacting in place.
    fn drop_outside_mask(&mut self, hold_mask: u64) -> usize {
        let mut write = 0usize;
        let mut dropped = 0usize;
        for read in 0..self.len {
            if let Some(entry) = self.buf[read] {
                let keep = entry.joint < 64 && (hold_mask & (1u64 << entry.joint)) != 0;
                if !keep {
                    dropped += 1;
                } else {
                    self.buf[write] = Some(entry);
                    write += 1;
                }
            }
        }
        for slot in &mut self.buf[write..self.len] {
            *slot = None;
        }
        self.len = write;
        dropped
    }

    /// Replace an existing `(joint, frame_id)` slot in place without moving it to the back
    /// (preserving oldest-first carry-over FIFO order and its original `queued_at` under the same
    /// `stop_stamp` so carry-over frames always drain first with the latest target value and
    /// coalescing cannot renew the 45 ms expiry deadline indefinitely), or push to the back if
    /// capacity permits. If an existing slot carried a different `stop_stamp`, it is removed and
    /// re-appended at the back so entries stay monotonically ordered oldest-first by `queued_at`.
    fn upsert(
        &mut self,
        joint: usize,
        frame_id: u32,
        cmd: PendingTargetCmd,
        queued_at: Instant,
        stop_stamp: StopStamp,
        lease_gen: u64,
    ) -> bool {
        if let Some(idx) = self.find_key(joint, frame_id) {
            if let Some(prev) = self
                .buf[idx]
                .filter(|prev| prev.stop_stamp == stop_stamp && prev.lease_gen == lease_gen)
            {
                self.buf[idx] = Some(PendingTxFrame {
                    joint,
                    frame_id,
                    cmd,
                    queued_at: prev.queued_at,
                    stop_stamp,
                    lease_gen,
                    was_deferred: false,
                });
                return true;
            }
            for i in (idx + 1)..self.len {
                self.buf[i - 1] = self.buf[i];
            }
            self.len -= 1;
            self.buf[self.len] = None;
        }
        if self.len >= DAMIAO_PENDING_TX_CAPACITY {
            return false;
        }
        self.buf[self.len] = Some(PendingTxFrame {
            joint,
            frame_id,
            cmd,
            queued_at,
            stop_stamp,
            lease_gen,
            was_deferred: false,
        });
        self.len += 1;
        true
    }

    /// Mark every currently queued entry not yet counted as deferred; returns how many flipped.
    fn mark_newly_deferred(&mut self) -> usize {
        let mut newly = 0usize;
        for slot in &mut self.buf[..self.len] {
            if let Some(entry) = slot.as_mut() {
                if !entry.was_deferred {
                    entry.was_deferred = true;
                    newly += 1;
                }
            }
        }
        newly
    }
}

/// Unified owner-local `0xCC` read-only status refresh scheduler for [`DamiaoArmDriver`].
///
/// Zero locks, zero heap allocations. Uses a fixed `u64` bitmask (`unfinished_mask` / `swept_mask`)
/// and round-robin `cursor` to serve every joint lacking recent replies (including J7 under
/// `NoOutputUntilFresh`) across enable transitions, `FailedHold`, `Idle`-torqued, and stopped
/// states, enforcing at most ONE `0xCC` slot per tick (`slot_used_this_tick`).
#[derive(Debug, Default)]
struct RefreshScheduler {
    unfinished_mask: u64,
    swept_mask: u64,
    cursor: usize,
    last_sweep_completed: Option<Instant>,
    slot_used_this_tick: bool,
}

impl RefreshScheduler {
    #[inline]
    fn begin_tick(&mut self) {
        self.slot_used_this_tick = false;
    }

    #[inline]
    fn reset_sweep(&mut self) {
        self.unfinished_mask = 0;
        self.swept_mask = 0;
        self.cursor = 0;
    }

    #[inline]
    fn clear_joint(&mut self, joint: usize) {
        if joint < 64 {
            let bit = 1u64 << joint;
            self.unfinished_mask &= !bit;
            self.swept_mask &= !bit;
        }
    }

    #[inline]
    fn slot_available(&self) -> bool {
        !self.slot_used_this_tick
    }

    /// Consume this tick's single `0xCC` slot (used by causal enable `RefreshCapture` and
    /// post-freeze J7 refreshes so they share the 1-`0xCC`-per-tick cap without perturbing
    /// the round-robin sweep cursor or masks).
    #[inline]
    fn mark_slot_used(&mut self) {
        self.slot_used_this_tick = true;
    }

    /// Select the next stale joint to receive this tick's single `0xCC` slot, or `None` if no
    /// slot is available, no joint is stale, or every stale joint was already polled within
    /// `cooldown`. Newly stale joints (not in `swept_mask`) are served immediately without
    /// waiting for an unrelated sweep cooldown.
    fn pick_next(
        &mut self,
        stale_mask: u64,
        n: usize,
        now: Instant,
        cooldown: Duration,
    ) -> Option<usize> {
        let valid_mask = if n == 0 {
            0
        } else if n >= 64 {
            u64::MAX
        } else {
            (1u64 << n) - 1
        };
        let stale = stale_mask & valid_mask;
        // Any joint that received fresh feedback leaves both masks immediately.
        // If a previously polled joint recovered (replied), the bus is actively answering so do not
        // hold remaining stale joints behind the silent-sweep cooldown.
        if (self.swept_mask & !stale) != 0 {
            self.last_sweep_completed = None;
        }
        self.swept_mask &= stale;
        self.unfinished_mask &= stale;
        if stale == 0 || !self.slot_available() {
            return None;
        }
        let cooldown_ready = self
            .last_sweep_completed
            .is_none_or(|t| now.saturating_duration_since(t) >= cooldown);
        let candidates = if self.unfinished_mask != 0 {
            self.unfinished_mask
        } else if cooldown_ready {
            self.swept_mask = 0;
            stale
        } else {
            let new_stale = stale & !self.swept_mask;
            if new_stale != 0 {
                new_stale
            } else {
                return None;
            }
        };
        let start = self.cursor % n;
        for offset in 0..n {
            let i = (start + offset) % n;
            if (candidates & (1u64 << i)) != 0 {
                self.unfinished_mask = candidates;
                return Some(i);
            }
        }
        None
    }

    /// Record that `joint`'s `0xCC` frame transmitted and flushed in this tick.
    #[inline]
    fn commit_sent(&mut self, joint: usize, n: usize, now: Instant) {
        self.slot_used_this_tick = true;
        if joint < 64 {
            let bit = 1u64 << joint;
            self.unfinished_mask &= !bit;
            self.swept_mask |= bit;
        }
        if n > 0 {
            self.cursor = (joint + 1) % n;
        }
        if self.unfinished_mask == 0 {
            self.last_sweep_completed = Some(now);
        }
    }

    /// Consume this tick's `0xCC` slot without advancing `cursor` or clearing `unfinished_mask`
    /// when TX budget/backpressure deferred transmission, so the next tick retries the same joint.
    #[inline]
    fn record_deferred(&mut self) {
        self.slot_used_this_tick = true;
    }

    /// Advance past a joint that failed with a non-retryable transmit error so other stale joints
    /// are not starved on subsequent ticks.
    #[inline]
    fn skip_non_retryable(&mut self, joint: usize, n: usize, now: Instant) {
        self.commit_sent(joint, n, now);
    }
}

/// Read-only `0xCC` status refresh cadence / staleness threshold for joints lacking recent replies.
const DAMIAO_REFRESH_AFTER: Duration = Duration::from_millis(100);

/// Phase of a pending Damiao torque transition.
#[derive(Debug)]
#[allow(clippy::large_enum_variant)] // inline ModeReadback avoids RT heap allocation
enum DamiaoTorque {
    /// Budgeted legacy enable retrying mapping-range readback for joints still `Unverified` at
    /// startup before draining/restoring/enabling (zero heap allocations on the control release).
    RangeCheck(RangeCheckPhase),
    /// Budgeted enable waiting for an empty RX queue boundary before mode restore / enable TX.
    DrainBeforeEnable(Deadline),
    /// Enable, initial bounded FRESHEN phase: polls every joint with read-only `0xCC` status
    /// refreshes and waits for a fresh causal reply from every joint before CTRL_MODE restore,
    /// readback, pre-enable hold, and `CMD_ENABLE`.
    Freshen(EnableFreshen),
    /// Enable, pre-restore refresh: waiting for a fresh J7 0xCC feedback reply before writing CTRL_MODE.
    RefreshBeforeRestore {
        refresh: FeedbackRefresh,
        stray: [u8; MAX_MODE_JOINTS],
        stray_len: u8,
    },
    /// Enable, first phase: CTRL_MODE writes restoring the base mode, awaiting their replies.
    Restore { write: ModeWrite, pf_refreshed: bool },
    /// Enable, second phase: CTRL_MODE readback of the position-force gripper.
    Readback(ModeReadback),
    /// Enable, pre-enable refresh: CTRL_MODE 4 confirmed, waiting for a fresh J7 0xCC reply
    /// before transmitting the capped 0x307 pre-enable hold.
    PreEnableRefresh(FeedbackRefresh),
    /// Enable frames sent (or still queued in `unsent_mask` across TX-budgeted polls), awaiting
    /// fresh state-1 feedback from every motor. Reserves a bounded causal J7 `0xCC` refresh slot
    /// and fair J7 TX priority so a carried or retried J7 enable neither starves nor goes stale.
    Enable {
        window: ConfirmWindow,
        unsent_mask: u32,
        tx_cursor: u8,
        j7_prioritize: bool,
        pf_refresh: Option<FeedbackRefresh>,
        pf_refresh_count: u8,
    },
    /// Stopped disable waiting for an exhausted RX drain or incomplete K=3 rest-pose window
    /// to complete inside the 1 s request lifetime before transmitting any `CMD_DISABLE` (L1).
    AwaitRestGate {
        kind: DisableKind,
        next_probe_at: Instant,
        client_park: bool,
        expected_stamp: Option<StopStamp>,
        start_stop_stamp: StopStamp,
        started_in_estop: bool,
    },
    /// Accepted disable dispatching across as many ticks as the per-tick TX budget needs
    /// (`repeats_sent == 0` for the initial pass while still-enabled joints keep holding,
    /// `repeats_sent >= 1` for bounded `DAMIAO_DISABLE_REPEATS` passes and status confirmation).
    Disable {
        kind: DisableKind,
        repeats_sent: u8,
        repeat_pass_mask: u64,
        next_repeat_at: Instant,
        next_probe_at: Instant,
        mit_fenced: bool,
        start_stop_stamp: StopStamp,
        started_in_estop: bool,
        client_park: bool,
        rest_established: bool,
    },
}

/// Inline zero-allocation state for ticked mapping-range verification of joints that were
/// `RangeCheckStatus::Unverified` at daemon startup (legacy budgeted enable path).
#[derive(Debug)]
struct RangeCheckPhase {
    pending_mask: u32,
    active_joint: Option<u8>,
    boundary: Option<u64>,
    sent_rids: u8,
    deadline: Deadline,
}

/// Inline zero-allocation state for enable's initial bounded `0xCC` FRESHEN phase.
#[derive(Debug)]
struct EnableFreshen {
    target_mask: u32,
    sent_mask: u32,
    confirmed_mask: u32,
    started: Instant,
    base_rx_seq: u64,
    tx_boundary_delta: [u32; MAX_MODE_JOINTS],
    tx_offset_ns: [u32; MAX_MODE_JOINTS],
    next_retry_at: Option<Instant>,
    deadline: Deadline,
}

impl EnableFreshen {
    const UNSET_NS: u32 = u32::MAX;

    fn tx_boundary(&self, j: usize) -> u64 {
        self.base_rx_seq.saturating_add(u64::from(self.tx_boundary_delta[j]))
    }

    fn tx_at(&self, j: usize) -> Option<Instant> {
        let ns = self.tx_offset_ns[j];
        (ns != Self::UNSET_NS).then(|| self.started + Duration::from_nanos(u64::from(ns)))
    }

    fn record_tx(&mut self, j: usize, boundary: u64, tx_at: Instant) {
        self.sent_mask |= 1u32 << j;
        self.tx_boundary_delta[j] =
            u32::try_from(boundary.saturating_sub(self.base_rx_seq)).unwrap_or(u32::MAX);
        let ns = u32::try_from(tx_at.saturating_duration_since(self.started).as_nanos())
            .unwrap_or(Self::UNSET_NS - 1);
        self.tx_offset_ns[j] = ns.min(Self::UNSET_NS - 1);
    }
}

/// Time allowed for register replies during ticked range-check recovery (safe at 50 Hz ticks).
const TICKED_RANGE_CHECK_REPLY_TIMEOUT: Duration = Duration::from_millis(60);

/// Bounded 0xCC feedback refresh for the position-force gripper during enable: `boundary` is the
/// transmit-time RX sequence barrier established after draining RX right before sending 0xCC
/// (None: waiting to drain RX and send 0xCC).
#[derive(Debug)]
struct FeedbackRefresh { joint: usize, boundary: Option<u64>, deadline: Deadline }

#[derive(Debug, Clone, Copy)]
struct RefreshSample {
    position: f32,
    velocity: f32,
    motor_status: u8,
    rx_at: Instant,
    queue_bound: Option<Instant>,
    seq: u64,
}

#[derive(Debug)]
struct RefreshCapture {
    joint: usize,
    boundary: u64,
    tx_at: Instant,
    deadline: Deadline,
    /// When true (in-enable J7 refresh), `motor_status == 1` (already enabled on a retry after a
    /// dropped enable reply) is valid and does not trigger an immediate unsafe veto. Hardware
    /// faults and invalid states still veto immediately.
    allow_enabled_state: bool,
    first: Option<RefreshSample>,
    /// First post-boundary J7 reply observed during this refresh that violated stationary
    /// torque-off (`|v| > 0.05`, non-finite velocity, or `motor_status != 0`; when
    /// `allow_enabled_state` is true, `motor_status == 1` with finite velocity is permitted).
    /// Sticky across polls so a later benign reply cannot mask an intervening motion or fault event.
    unsafe_sample: Option<RefreshSample>,
}

enum RefreshStep {
    Ready(RefreshSample),
    Pending(FeedbackRefresh),
    TimedOut,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ReadbackStage {
    CtrlMode,
    Timeout,
}

/// Pre-enable `CTRL_MODE` and `TIMEOUT` readback (L2 + power-overload PR 1): verifies
/// `verified_modes[j] == Some(base_mode(j))` and fresh causal `RID_TIMEOUT` replies before any
/// `CMD_ENABLE` frame is transmitted. Stored inline with bitmasks (zero heap allocation).
#[derive(Debug)]
struct ModeReadback {
    stage: ReadbackStage,
    remaining_mask: u32,
    sent_mask: u32,
    timeout_pending_mask: u32,
    log_cleared: bool,
    tx_evidence: [Option<(u64, Instant, Option<i128>)>; MAX_MODE_JOINTS],
    deadline: Deadline,
}

/// Maximum joints supported in inline stack-allocated [`ModeWrite`] buffers.
const MAX_MODE_JOINTS: usize = 16;

/// Maximum unconsumed causal TX events (1 probe + up to 3 subsequent continuity holds) tracked
/// per axis within one MIT expiry probe window (zero alloc; overflow fails closed as ambiguous).
const MIT_EXPIRY_MAX_CAUSES: usize = 4;

/// Per-axis sequential causal disable confirmation state for MIT expiry recovery (zero alloc).
///
/// SocketCAN timestamps host submission and `flush()` is a no-op, so replies cannot be correlated
/// by transport flush alone. Confirmation is strictly sequential per axis:
/// 1. `CMD_DISABLE` (`dis_tx`)
/// 2. Drain RX -> `REG_READ(RID_CTRL_MODE)` barrier (`barrier_tx` -> `barrier_ack`)
/// 3. Drain RX -> Round 1 `0xCC` status probe (`probe1_tx` -> `probe1_ack`, `state == 0`)
/// 4. Drain RX -> Round 2 `0xCC` status probe (`probe2_tx` -> `probe2_ack`, `state == 0`),
///    submitted strictly after `probe1_ack` (`p2_tx_at > p1_ack_at`).
///
/// Retries advance `min_next_fb_seq` and drain RX before submitting a fresh probe so a retry
/// never reuses a prior reply identity. Any feedback arriving strictly after `reply_window_start`
/// without a strictly earlier unconsumed probe/hold cause latches `reply_ambiguous = true` and
/// invalidates the axis (both before and after `probe2_ack` is latched).
#[derive(Debug, Clone, Copy, Default)]
struct MitExpiryAxisState {
    dis_tx: Option<(u64, Instant)>,
    barrier_tx: Option<(u64, Instant, Option<i128>)>,
    barrier_ack: Option<(u64, Instant)>,
    probe1_tx: Option<(u64, Instant, Option<i128>)>,
    probe1_ack: Option<(u64, Instant)>,
    probe2_tx: Option<(u64, Instant, Option<i128>)>,
    probe2_ack: Option<(u64, Instant)>,
    min_next_fb_seq: u64,
    reply_window_start: Option<Instant>,
    pending_causes: [Option<Instant>; MIT_EXPIRY_MAX_CAUSES],
    pending_causes_len: u8,
    reply_ambiguous: bool,
}

impl MitExpiryAxisState {
    #[inline]
    fn clear_reply_window(&mut self) {
        self.reply_window_start = None;
        self.pending_causes = [None; MIT_EXPIRY_MAX_CAUSES];
        self.pending_causes_len = 0;
        self.reply_ambiguous = false;
    }

    #[inline]
    fn open_reply_window(&mut self, probe_tx_at: Instant) {
        self.reply_window_start = Some(probe_tx_at);
        self.pending_causes = [None; MIT_EXPIRY_MAX_CAUSES];
        self.pending_causes[0] = Some(probe_tx_at);
        self.pending_causes_len = 1;
        self.reply_ambiguous = false;
    }

    #[inline]
    fn record_hold_cause(&mut self, hold_at: Instant) {
        if let Some(w) = self.reply_window_start {
            if hold_at >= w {
                let len = self.pending_causes_len as usize;
                if len < MIT_EXPIRY_MAX_CAUSES {
                    self.pending_causes[len] = Some(hold_at);
                    self.pending_causes_len = (len + 1) as u8;
                } else {
                    self.reply_ambiguous = true;
                }
            }
        }
    }

    #[inline]
    fn observe_feedback(&mut self, rx_at: Instant) {
        if let Some(w) = self.reply_window_start {
            if rx_at > w {
                let len = self.pending_causes_len as usize;
                if len > 0 && self.pending_causes[0].is_some_and(|c| c < rx_at) {
                    for k in 1..len {
                        self.pending_causes[k - 1] = self.pending_causes[k];
                    }
                    self.pending_causes[len - 1] = None;
                    self.pending_causes_len = (len - 1) as u8;
                } else {
                    self.reply_ambiguous = true;
                }
            }
        }
    }

    #[inline]
    fn observe_untimestamped_feedback(&mut self) {
        if self.reply_window_start.is_some() {
            self.reply_ambiguous = true;
        }
    }

    #[inline]
    fn reset_probes(&mut self, seen_fb_seq: u64) {
        self.probe1_tx = None;
        self.probe1_ack = None;
        self.probe2_tx = None;
        self.probe2_ack = None;
        self.clear_reply_window();
        self.min_next_fb_seq = self.min_next_fb_seq.max(seen_fb_seq);
    }

    #[inline]
    fn reset_attempt(&mut self, seen_fb_seq: u64) {
        self.dis_tx = None;
        self.barrier_tx = None;
        self.barrier_ack = None;
        self.reset_probes(seen_fb_seq);
    }
}

/// CTRL_MODE writes sent by `write_modes_send`, awaiting their register replies until `deadline`.
/// Stored inline with `u8` joint indices (no heap vectors) so RT mode-restore completion,
/// budget-deferred retries across ticks, and E-stop abandonment never allocate or free.
#[derive(Debug)]
struct ModeWrite {
    modes: [DamiaoControlMode; MAX_MODE_JOINTS],
    sent: [u8; MAX_MODE_JOINTS],
    sent_len: u8,
    unsent: [u8; MAX_MODE_JOINTS],
    unsent_len: u8,
    hold_failed: [u8; MAX_MODE_JOINTS],
    hold_failed_len: u8,
    /// Joints with remaining torque-off enable-restore frames deferred by the per-tick TX budget.
    deferred: [u8; MAX_MODE_JOINTS],
    /// Per-entry next frame stage: `0` = CTRL_MODE write, `1` = hold frame 0, `2` = hold frame 1.
    deferred_stage: [u8; MAX_MODE_JOINTS],
    deferred_len: u8,
    /// Hold positions per joint for deferred torque-off enable-restore frames.
    hold_pos: [f32; MAX_MODE_JOINTS],
    /// Whether non-PosForce joints emit both POS_VEL and MIT hold frames during restore.
    dual_hold: bool,
    flush_ok: bool,
    deadline: Deadline,
}

/// Time allowed for register replies (mode switch, range readback).
const REGISTER_REPLY_TIMEOUT: Duration = Duration::from_millis(50);
/// Proactive interval between continuity holds on energized axes during synchronous legacy MIT
/// expiry recovery (12 ms reserves >= 8 ms headroom for bounded RX/TX slices so per-axis
/// hold-to-hold gaps stay <= 20 ms even when RX/TX advances time, well below Damiao's 100 ms
/// hardware watchdog).
const MIT_RECOVERY_CONTINUITY_HOLD_INTERVAL: Duration = Duration::from_millis(12);
/// Maximum single wait/sleep duration between continuity holds during synchronous legacy MIT
/// expiry recovery (<= 20 ms).
const MIT_RECOVERY_MAX_WAIT_BETWEEN_HOLDS: Duration = Duration::from_millis(12);
/// Time allowed per motor for the four register replies of the startup range check
/// (7 silent motors: 140 ms).
const RANGE_CHECK_REPLY_TIMEOUT: Duration = Duration::from_millis(20);
/// Feedback freshness bound for position-force gripper mode writes, pre-enable holds,
/// stale-feedback recovery, and opening-speed allowance.
const POS_FORCE_FEEDBACK_MAX_AGE: Duration = Duration::from_millis(100);
/// Feedback freshness threshold for skipping enable's initial bounded `0xCC` FRESHEN phase
/// on a `PosForce` gripper joint. Any age beyond this bound (or missing empty-queue bound)
/// triggers FRESHEN across every joint before CTRL_MODE restore/readback, hold, and `CMD_ENABLE`.
const ENABLE_FRESHEN_ENTRY_PF_MAX_AGE: Duration = Duration::from_millis(40);
/// Feedback freshness threshold for skipping enable's initial bounded `0xCC` FRESHEN phase
/// on non-`PosForce` arm joints (strictly below the 100 ms refresh/rest-gate bound so any
/// disabled idle period triggers FRESHEN while avoiding spurious stalls on freshly-read rigs).
const ENABLE_FRESHEN_ENTRY_ARM_MAX_AGE: Duration = Duration::from_millis(80);
/// Maximum feedback age allowed for arm joints when exiting enable's FRESHEN phase (matches
/// `DAMIAO_FEEDBACK_VALID_WINDOW` so an early arm reply stays confirmed while waiting on a
/// slower joint within the 180 ms FRESHEN budget).
const ENABLE_FRESHEN_EXIT_ARM_MAX_AGE: Duration = DAMIAO_FEEDBACK_VALID_WINDOW;
/// Maximum feedback age allowed for a `PosForce` gripper joint when exiting enable's FRESHEN
/// phase (leaves >= 40 ms margin before `POS_FORCE_FEEDBACK_MAX_AGE` for readback + hold).
const ENABLE_FRESHEN_EXIT_PF_MAX_AGE: Duration = Duration::from_millis(60);
/// Bounded wall-clock budget for enable's initial `0xCC` FRESHEN phase (<= 200 ms).
const ENABLE_FRESHEN_TIMEOUT: Duration = Duration::from_millis(180);
/// Minimum interval before retrying `0xCC` for unanswered joints within one FRESHEN window.
const ENABLE_FRESHEN_RETRY_INTERVAL: Duration = Duration::from_millis(60);
/// Safety margin below `POS_FORCE_FEEDBACK_MAX_AGE` when `poll_readback` decides whether J7
/// can proceed directly to `finish_pos_force_readback` without a `PreEnableRefresh`, closing
/// any boundary race near 100 ms.
const POS_FORCE_READBACK_FRESH_MARGIN: Duration = Duration::from_millis(85);
/// Bounded causal `0xCC` refresh window during `DamiaoTorque::Enable` (clamped to the enable
/// confirmation deadline and authority deadline; accommodates 30-60 ms Damiao replies across
/// 20 ms releases while staying strictly inside `POS_FORCE_FEEDBACK_MAX_AGE`).
const ENABLE_PF_REFRESH_TIMEOUT: Duration = Duration::from_millis(80);
/// Maximum bounded causal J7 `0xCC` refreshes permitted within one `DamiaoTorque::Enable` window.
const MAX_ENABLE_PF_REFRESHES: u8 = 2;
/// Proactive watchdog hold cadence for already-enabled arm joints during multi-tick handshakes:
/// refreshes every 50 Hz tick (20 ms >= 15 ms) whenever spare TX budget remains.
const WATCHDOG_HOLD_PROACTIVE_INTERVAL: Duration = Duration::from_millis(15);
/// Urgent watchdog hold threshold: strictly below one 50 Hz release (20 ms) so every release
/// during a multi-tick enable refreshes already-enabled arm joints FIRST before handshake TX,
/// guaranteeing <= 40 ms between control frames even with jitter or a 1-tick TX budget squeeze.
const WATCHDOG_HOLD_URGENT_INTERVAL: Duration = Duration::from_millis(18);
/// Sub-release wake threshold for watchdog continuity: strictly above one 50 Hz release (20 ms)
/// so healthy 50 Hz releases service energized motors once per release without scheduling a
/// redundant Window 2 sleep at t + 16 ms, while still waking before 40 ms if a release misses TX
/// or runs at a slower period.
const WATCHDOG_HOLD_WAKE_INTERVAL: Duration = Duration::from_millis(22);
/// Remaining release budget margin required to start optional (deferrable) CAN work such as
/// background `0xCC` status polls or autonomous recovery frames without pushing a release past
/// its deadline.
pub(crate) const OPTIONAL_WORK_RELEASE_MARGIN: Duration = Duration::from_millis(6);
/// Maximum autonomous stale-feedback recovery TX frames transmitted inside `receive_feedback`
/// during a single managed/release-bounded control release (round-robin across joints).
const DAMIAO_MAX_RECOVERY_PER_RELEASE: u8 = 2;
/// Fallback watchdog hold threshold during `AwaitRestGate` / `Disable` Phase A (`repeats_sent == 0`):
/// strictly above one 50 Hz control tick (20 ms) so normal ticked control (whose Step 3 already
/// runs `service_stopped_measured_hold` every 20 ms) never emits duplicate holds, while blocking
/// `disable_torque` and headless fallback still keep enabled motors refreshed within <= 40 ms.
const WATCHDOG_DISABLE_FALLBACK_INTERVAL: Duration = Duration::from_millis(30);
/// Fail-closed enable abort threshold: if a sustained multi-tick TX squeeze prevents refreshing an
/// energized arm joint for >= 60 ms during enable, abort the handshake cleanly into `FailedHold`
/// BEFORE Damiao's `TIMEOUT = 2000` (~100 ms) can expire with `0xD`.
const WATCHDOG_ENABLE_ABORT_AGE: Duration = Duration::from_millis(60);

/// Damiao diagnostics that left the `ArmDriver` trait in S4 (allocating twins of
/// `JointModeExt::with_range_check`, `JointModeExt::take_recovery_writes_into` and
/// `MitExt::mit_fence_trip_bounded`); for tests and tools, not for owners.
impl<C: CanTransport> DamiaoArmDriver<C> {
}










pub(crate) use pos_force_emitter::PosForceSendToken;


#[cfg(test)]
mod tx_chokepoint_tests {
    use super::*;
    use crate::rt_core::urgent::{EstopGate, StopRequests};
    use crate::transport::{CanFrame, CanTransport, DriverClock, MockClock, TransportError};
    use std::collections::VecDeque;
    use std::sync::atomic::{AtomicBool, AtomicU64};
    use std::sync::{Arc, Mutex};

    #[derive(Clone)]
    struct TestBus {
        clock: Arc<MockClock>,
        tx: Arc<Mutex<Vec<CanFrame>>>,
        rx: Arc<Mutex<VecDeque<(CanFrame, Option<Instant>)>>>,
        on_tx: Arc<Mutex<Option<Box<dyn FnMut(&CanFrame) -> Result<(), TransportError> + Send>>>>,
    }

    impl TestBus {
        fn new() -> (Self, Arc<MockClock>) {
            let clock = Arc::new(MockClock::new());
            let bus = Self {
                clock: clock.clone(),
                tx: Arc::new(Mutex::new(Vec::new())),
                rx: Arc::new(Mutex::new(VecDeque::new())),
                on_tx: Arc::new(Mutex::new(None)),
            };
            (bus, clock)
        }

        fn tx_len(&self) -> usize {
            self.tx.lock().unwrap().len()
        }

        fn tx_frames(&self) -> Vec<CanFrame> {
            self.tx.lock().unwrap().clone()
        }

        fn clear_tx(&self) {
            self.tx.lock().unwrap().clear();
        }

        fn set_on_tx(&self, cb: impl FnMut(&CanFrame) -> Result<(), TransportError> + Send + 'static) {
            *self.on_tx.lock().unwrap() = Some(Box::new(cb));
        }

        fn push_feedback(&self, motor_id: u16, pos: f32, state: u8, limits: &DamiaoLimits) {
            let p = damiao::float_to_uint(pos, limits.p_min, limits.p_max, 16);
            let v = damiao::float_to_uint(0.0, limits.v_min, limits.v_max, 12);
            let t = damiao::float_to_uint(0.0, limits.t_min, limits.t_max, 12);
            let data = [
                ((state & 0x0f) << 4) | (motor_id as u8 & 0x0f),
                (p >> 8) as u8,
                p as u8,
                (v >> 4) as u8,
                (((v & 0x0f) << 4) | ((t >> 8) & 0x0f)) as u8,
                t as u8,
                30,
                30,
            ];
            let ts = self.clock.now();
            self.rx
                .lock()
                .unwrap()
                .push_back((CanFrame::new_standard(0x10 + motor_id, &data).unwrap(), Some(ts)));
        }
    }

    impl CanTransport for TestBus {
        fn clock(&self) -> Option<Arc<dyn DriverClock>> {
            Some(self.clock.clone())
        }
        fn supports_nonblocking_rt(&self) -> bool { true }
        fn supports_bounded_flush(&self) -> bool { true }
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
            if let Some(cb) = self.on_tx.lock().unwrap().as_mut() {
                cb(frame)?;
            }
            self.tx.lock().unwrap().push(*frame);
            Ok(())
        }
        fn receive(&mut self) -> Result<CanFrame, TransportError> {
            self.receive_timestamped().map(|(f, _)| f)
        }
        fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
            self.rx.lock().unwrap().pop_front().ok_or(TransportError::WouldBlock)
        }
        fn flush(&mut self) -> Result<(), TransportError> {
            Ok(())
        }
    }

    fn all_tx_kinds(started_at: Instant) -> [TxKind; 14] {
        [
            TxKind::DisableRequested { started_at },
            TxKind::MitRecoveryDisable { started_at },
            TxKind::ClientParkDisable { started_at },
            TxKind::DisableProtective { reason: DisableProtectiveReason::Thermal },
            TxKind::DisableProtective { reason: DisableProtectiveReason::ProcessExit },
            TxKind::DisableProbe,
            TxKind::Target,
            TxKind::Hold,
            TxKind::RollbackHold(0),
            TxKind::ModeWrite,
            TxKind::Refresh,
            TxKind::Recovery,
            TxKind::ClearFault,
            TxKind::Handshake,
        ]
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum StopCase {
        NoStop,
        Pending,
        Latched,
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum BudgetCase {
        Available,
        Exhausted,
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum DeadlineCase {
        Ok,
        Passed,
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum Expected {
        Sent,
        Deferred,
        RefusedDeadline,
        RefusedStop,
    }

    fn expected_outcome(
        kind: TxKind,
        stop: StopCase,
        budget: BudgetCase,
        deadline: DeadlineCase,
    ) -> Expected {
        // Step 1: Deadline (if given) always runs first for every TxKind.
        if deadline == DeadlineCase::Passed {
            return Expected::RefusedDeadline;
        }
        // Step 2: Tick TX budget runs before stop gate (DisableRequested / ClientParkDisable / DisableProtective / DisableProbe exempt).
        if budget == BudgetCase::Exhausted
            && !matches!(
                kind,
                TxKind::DisableRequested { .. }
                    | TxKind::ClientParkDisable { .. }
                    | TxKind::DisableProtective { .. }
                    | TxKind::DisableProbe
            )
        {
            return Expected::Deferred;
        }
        // Step 3: Stop gate by TxKind.
        match kind {
            TxKind::DisableRequested { .. }
            | TxKind::MitRecoveryDisable { .. }
            | TxKind::DisableProtective { .. }
            | TxKind::DisableProbe => Expected::Sent,
            TxKind::ClientParkDisable { .. } => match stop {
                StopCase::NoStop => Expected::Sent,
                StopCase::Pending | StopCase::Latched => Expected::RefusedStop,
            },
            TxKind::Refresh | TxKind::RollbackHold(_) | TxKind::MeasuredHold(_) | TxKind::ClearFault => match stop {
                StopCase::Pending => Expected::RefusedStop,
                StopCase::NoStop | StopCase::Latched => Expected::Sent,
            },
            TxKind::Target
            | TxKind::Hold
            | TxKind::ModeWrite
            | TxKind::Recovery
            | TxKind::Handshake => match stop {
                StopCase::NoStop => Expected::Sent,
                StopCase::Pending | StopCase::Latched => Expected::RefusedStop,
            },
        }
    }

    #[test]
    fn send_chokepoint_table_all_kinds_stop_budget_deadline() {
        let limits = DamiaoLimits::default();
        let mut cells_checked = 0usize;

        let dummy_start = Instant::now();
        let kinds = all_tx_kinds(dummy_start);
        for &kind in &kinds {
            for &stop in &[StopCase::NoStop, StopCase::Pending, StopCase::Latched] {
                for &budget in &[BudgetCase::Available, BudgetCase::Exhausted] {
                    for &deadline_case in &[DeadlineCase::Ok, DeadlineCase::Passed] {
                        let (bus, clock) = TestBus::new();
                        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1])
                            .with_limits(limits);

                        // Configure stop/E-stop state and resting-pose gate matching J1's fresh
                        // feedback (0.25 rad) BEFORE feeding K=3 spaced rest samples (S2) so
                        // `DisableRequested` passes Rule A′ in this table; separate tests below
                        // verify raised/stale/expired/no-gate refusals on `DisableRequested`.
                        let req = Arc::new(AtomicU64::new(match stop {
                            StopCase::NoStop => 0,
                            StopCase::Pending | StopCase::Latched => 1,
                        }));
                        let app = Arc::new(AtomicU64::new(match stop {
                            StopCase::NoStop => 0,
                            StopCase::Pending => 0,
                            StopCase::Latched => 1,
                        }));
                        let latched = Arc::new(AtomicBool::new(stop == StopCase::Latched));
                        drv.set_estop_gate(EstopGate::new(req, app));
                        drv.set_abort_flag(latched);
                        let mut rest_positions = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
                        rest_positions[0] = 0.25;
                        drv.set_stopped_disable_gate(StoppedDisableGate {
                            safety_stopped: Arc::new(AtomicBool::new(stop != StopCase::NoStop)),
                            has_resting_pose: true,
                            arm_joints: [0; crate::rt_core::mailbox::MAX_DOF],
                            arm_joints_len: 1,
                            rest_positions,
                            rest_pose_tolerance_rad: 0.05,
                        });

                        // Establish an empty-queue receive bound + K=3 spaced fresh samples on J1
                        // so both RollbackHold(0) freshness and S2 rest-pose K-window are satisfied.
                        drv.read_joint_states().unwrap();
                        for step in 0..3 {
                            if step > 0 {
                                clock.advance(Duration::from_millis(10));
                            }
                            bus.push_feedback(1, 0.25, 1, &limits);
                            drv.read_joint_states().unwrap();
                        }
                        bus.clear_tx();
                        let kind = match kind {
                            TxKind::DisableRequested { .. } => {
                                TxKind::DisableRequested { started_at: clock.now() }
                            }
                            TxKind::MitRecoveryDisable { .. } => {
                                TxKind::MitRecoveryDisable { started_at: clock.now() }
                            }
                            TxKind::ClientParkDisable { .. } => {
                                TxKind::ClientParkDisable { started_at: clock.now() }
                            }
                            other => other,
                        };

                        // Configure per-tick TX budget.
                        drv.set_tx_budget(TxBudget {
                            max_frames: 1,
                            max_duration: Duration::from_millis(5),
                            frame_max_age: Duration::from_millis(45),
                        });
                        if budget == BudgetCase::Exhausted {
                            drv.record_tx_pass_attempt();
                        }

                        // Configure deadline.
                        let now = clock.now();
                        let deadline = match deadline_case {
                            DeadlineCase::Ok => Some(Deadline::after(now, Duration::from_millis(10))),
                            DeadlineCase::Passed => Some(Deadline { at: now }),
                        };

                        let stamp = drv.current_stop_stamp();
                        let mut build_called = false;
                        let res = drv.send_plain(kind, Some(stamp), deadline, false, |d| {
                            build_called = true;
                            Some(damiao::build_pos_vel_frame(0x100, d.motor_ids[0], 0.25, 1.0))
                        });

                        let want = expected_outcome(kind, stop, budget, deadline_case);
                        match want {
                            Expected::Sent => {
                                assert!(
                                    res.is_ok(),
                                    "cell ({kind:?}, {stop:?}, {budget:?}, {deadline_case:?}) expected Sent, got {res:?}"
                                );
                                assert_eq!(bus.tx_len(), 1, "expected 1 TX for {kind:?}/{stop:?}/{budget:?}/{deadline_case:?}");
                                assert!(build_called, "build must run for Sent cell");
                            }
                            Expected::Deferred => {
                                assert!(
                                    matches!(res, Err(TxRefusal::Deferred)),
                                    "cell ({kind:?}, {stop:?}, {budget:?}, {deadline_case:?}) expected Deferred, got {res:?}"
                                );
                                assert_eq!(bus.tx_len(), 0);
                                assert!(!build_called, "budget check must run before build");
                            }
                            Expected::RefusedDeadline => {
                                assert!(
                                    matches!(res, Err(TxRefusal::DeadlinePassed)),
                                    "cell ({kind:?}, {stop:?}, {budget:?}, {deadline_case:?}) expected DeadlinePassed, got {res:?}"
                                );
                                assert_eq!(bus.tx_len(), 0);
                                assert!(!build_called, "deadline check must run before build");
                            }
                            Expected::RefusedStop => {
                                assert!(
                                    matches!(res, Err(TxRefusal::StopInhibited)),
                                    "cell ({kind:?}, {stop:?}, {budget:?}, {deadline_case:?}) expected StopInhibited, got {res:?}"
                                );
                                assert_eq!(bus.tx_len(), 0);
                                assert!(!build_called, "stop gate must run before build");
                            }
                        }
                        cells_checked += 1;
                    }
                }
            }
        }
        assert_eq!(cells_checked, kinds.len() * 3 * 2 * 2);
    }

    #[test]
    fn send_chokepoint_enforces_fresh_rollback_stop_stamp_and_drain_deadline() {
        let limits = DamiaoLimits::default();

        // 1. Latched RollbackHold(0) with STALE feedback (> 100 ms) fails closed in step 4 even
        //    if the caller's build closure would return a frame; fresh feedback allows send.
        let (bus, clock) = TestBus::new();
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1]).with_limits(limits);
        drv.read_joint_states().unwrap();
        bus.push_feedback(1, 0.35, 1, &limits);
        drv.read_joint_states().unwrap();
        drv.set_estop_gate(EstopGate::new(Arc::new(AtomicU64::new(1)), Arc::new(AtomicU64::new(1))));
        drv.set_abort_flag(Arc::new(AtomicBool::new(true)));

        clock.advance(Duration::from_millis(101));
        bus.clear_tx();
        let mut called = false;
        let stale_res = drv.send_plain(TxKind::RollbackHold(0), None, None, false, |d| {
            called = true;
            Some(damiao::build_pos_vel_frame(0x100, d.motor_ids[0], 0.35, 1.0))
        });
        assert!(matches!(stale_res, Err(TxRefusal::BuildRefused)), "stale latched rollback must fail closed: {stale_res:?}");
        assert!(!called, "step 4 latched freshness guard must refuse before build");
        assert_eq!(bus.tx_len(), 0);

        // Refresh feedback -> now fresh (<= 100 ms) latched RollbackHold(0) sends.
        drv.read_joint_states().unwrap();
        bus.push_feedback(1, 0.35, 1, &limits);
        drv.read_joint_states().unwrap();
        bus.clear_tx();
        assert!(drv
            .send_plain(TxKind::RollbackHold(0), None, None, false, |d| {
                let pos = d.fresh_measured_position(0, POS_FORCE_FEEDBACK_MAX_AGE)?;
                Some(d.pos_vel_frame(0, pos))
            })
            .is_ok());
        assert_eq!(bus.tx_len(), 1);

        // 2. Step 4 `build == None` fails closed without consuming tick TX budget.
        drv.begin_tx_tick();
        drv.set_tx_budget(TxBudget {
            max_frames: 1,
            max_duration: Duration::from_millis(5),
            frame_max_age: Duration::from_millis(45),
        });
        drv.set_abort_flag(Arc::new(AtomicBool::new(false)));
        bus.clear_tx();
        assert!(matches!(
            drv.send_plain(TxKind::Target, None, None, false, |_| None),
            Err(TxRefusal::BuildRefused)
        ));
        assert_eq!(bus.tx_len(), 0);
        // Budget slot was NOT consumed by BuildRefused, so the next valid frame succeeds.
        assert!(drv
            .send_plain(TxKind::Target, None, None, false, |d| Some(d.pos_vel_frame(0, 0.0)))
            .is_ok());
        assert_eq!(bus.tx_len(), 1);

        // 3. Step 5 stop-generation stamp check: applied stop between admission and send
        //    refuses every non-Disable kind with StopStampChanged (0 TX), while Disable sends.
        for &kind in &all_tx_kinds(Instant::now()) {
            let (bus, clock) = TestBus::new();
            let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1]).with_limits(limits);
            let mut rest_positions = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
            rest_positions[0] = 0.0;
            drv.set_stopped_disable_gate(StoppedDisableGate {
                safety_stopped: Arc::new(AtomicBool::new(false)),
                has_resting_pose: true,
                arm_joints: [0; crate::rt_core::mailbox::MAX_DOF],
                arm_joints_len: 1,
                rest_positions,
                rest_pose_tolerance_rad: 0.05,
            });
            drv.read_joint_states().unwrap();
            for step in 0..3 {
                if step > 0 {
                    clock.advance(Duration::from_millis(10));
                }
                bus.push_feedback(1, 0.0, 1, &limits);
                drv.read_joint_states().unwrap();
            }
            let kind = match kind {
                TxKind::DisableRequested { .. } => {
                    TxKind::DisableRequested { started_at: clock.now() }
                }
                TxKind::MitRecoveryDisable { .. } => {
                    TxKind::MitRecoveryDisable { started_at: clock.now() }
                }
                other => other,
            };
            let stops = StopRequests::new();
            drv.set_estop_gate(
                EstopGate::new(Arc::new(AtomicU64::new(0)), Arc::new(AtomicU64::new(0)))
                    .with_stops(stops.clone()),
            );
            let old_stamp = drv.current_stop_stamp();
            let g = stops.request();
            stops.mark_applied(g);
            bus.clear_tx();
            let res = drv.send_plain(kind, Some(old_stamp), None, false, |d| {
                Some(damiao::build_disable_frame(d.motor_ids[0]))
            });
            if matches!(
                kind,
                TxKind::DisableRequested { .. }
                    | TxKind::MitRecoveryDisable { .. }
                    | TxKind::DisableProtective { .. }
                    | TxKind::DisableProbe
            ) {
                assert!(res.is_ok(), "Disable must ignore stop stamp: {res:?}");
                assert_eq!(bus.tx_len(), 1);
            } else {
                assert!(
                    matches!(res, Err(TxRefusal::StopStampChanged)),
                    "{kind:?} must fail with StopStampChanged after applied stop: {res:?}"
                );
                assert_eq!(bus.tx_len(), 0);
            }
        }

        // 4. Deferred restore drain stops once w.deadline passes mid-drain (0 frames after deadline).
        let (bus, clock) = TestBus::new();
        let clock_hook = clock.clone();
        *bus.on_tx.lock().unwrap() = Some(Box::new(move |_| {
            // Advance clock past the 50 ms restore deadline on the very first transmitted frame.
            clock_hook.advance(Duration::from_millis(60));
            Ok(())
        }));
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2]).with_limits(limits);
        drv.joint_modes[0] = Some(DamiaoControlMode::Mit);
        drv.joint_modes[1] = Some(DamiaoControlMode::Mit);
        let mut w = ModeWrite {
            modes: [DamiaoControlMode::PosVel; MAX_MODE_JOINTS],
            sent: [0; MAX_MODE_JOINTS],
            sent_len: 0,
            unsent: [0; MAX_MODE_JOINTS],
            unsent_len: 0,
            hold_failed: [0; MAX_MODE_JOINTS],
            hold_failed_len: 0,
            deferred: [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            deferred_stage: [0; MAX_MODE_JOINTS],
            deferred_len: 2,
            hold_pos: [0.0; MAX_MODE_JOINTS],
            dual_hold: false,
            flush_ok: true,
            deadline: Deadline::after(clock.now(), REGISTER_REPLY_TIMEOUT),
        };
        drv.drain_restore_deferred(&mut w).unwrap();
        assert_eq!(
            bus.tx_frames().len(),
            1,
            "drain_restore_deferred must stop starting frames once deadline passes: {:?}",
            bus.tx_frames()
        );
    }

    #[test]
    fn send_chokepoint_rechecks_latch_and_pending_after_build() {
        let limits = DamiaoLimits::default();

        // A. Flip the physical E-stop latch (`abort_flag`) INSIDE `build` WITHOUT advancing the
        //    stop generation (simulates `trigger_e_stop_checked` setting `e_stop_latched` before
        //    acquiring `estop_reg` and incrementing `requested_estop_count`).
        //    Every latch-inhibited kind AND an unlatched->latched `RollbackHold` MUST be refused
        //    with `StopInhibited` and transmit ZERO frames.
        for &kind in &[
            TxKind::Target,
            TxKind::Hold,
            TxKind::ModeWrite,
            TxKind::Recovery,
            TxKind::Handshake,
            TxKind::RollbackHold(0),
        ] {
            let (bus, _clock) = TestBus::new();
            let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1]).with_limits(limits);
            drv.read_joint_states().unwrap();
            bus.push_feedback(1, 0.25, 1, &limits);
            drv.read_joint_states().unwrap();
            bus.clear_tx();

            let req = Arc::new(AtomicU64::new(0));
            let app = Arc::new(AtomicU64::new(0));
            let latched = Arc::new(AtomicBool::new(false));
            drv.set_estop_gate(EstopGate::new(req, app));
            drv.set_abort_flag(latched.clone());

            let stamp = drv.current_stop_stamp();
            let mut build_ran = false;
            let res = drv.send_plain(kind, Some(stamp), None, false, |d| {
                build_ran = true;
                // Flip latch mid-build; generation (`req`/`app`) deliberately unchanged!
                latched.store(true, Ordering::SeqCst);
                Some(d.pos_vel_frame(0, 0.25))
            });
            assert!(build_ran, "build must have executed before post-build latch check for {kind:?}");
            assert!(
                matches!(res, Err(TxRefusal::StopInhibited)),
                "latch flip during build must refuse {kind:?} with StopInhibited, got {res:?}"
            );
            assert_eq!(
                bus.tx_len(),
                0,
                "latch flip during build must transmit 0 frames for {kind:?}"
            );
        }

        // B. Flip pending stop (`req > app`) INSIDE `build` with `expected_stamp = None` so only
        //    the post-build `stop_gate_for_kind` re-read catches it: even latch-allowed kinds
        //    (`Refresh`, latched `RollbackHold`, `ClearFault`) must refuse with `StopInhibited` and 0 TX.
        for &kind in &[TxKind::Refresh, TxKind::RollbackHold(0), TxKind::ClearFault] {
            let (bus, _clock) = TestBus::new();
            let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1]).with_limits(limits);
            drv.read_joint_states().unwrap();
            bus.push_feedback(1, 0.25, 1, &limits);
            drv.read_joint_states().unwrap();
            bus.clear_tx();

            let req = Arc::new(AtomicU64::new(1));
            let app = Arc::new(AtomicU64::new(1));
            let latched = Arc::new(AtomicBool::new(true));
            drv.set_estop_gate(EstopGate::new(req.clone(), app));
            drv.set_abort_flag(latched);

            let res = drv.send_plain(kind, None, None, false, |d| {
                req.fetch_add(1, Ordering::SeqCst);
                Some(damiao::build_refresh_frame(d.motor_ids[0]))
            });
            assert!(
                matches!(res, Err(TxRefusal::StopInhibited)),
                "pending stop raised during build must refuse {kind:?} with StopInhibited, got {res:?}"
            );
            assert_eq!(bus.tx_len(), 0);
        }

        // C. Protective `DisableProtective` still transmits even if both latch and pending flip during `build`.
        let (bus, _clock) = TestBus::new();
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1]).with_limits(limits);
        let req = Arc::new(AtomicU64::new(0));
        let app = Arc::new(AtomicU64::new(0));
        let latched = Arc::new(AtomicBool::new(false));
        drv.set_estop_gate(EstopGate::new(req.clone(), app));
        drv.set_abort_flag(latched.clone());
        let res = drv.send_plain(
            TxKind::DisableProtective { reason: DisableProtectiveReason::ProcessExit },
            None,
            None,
            false,
            |d| {
                latched.store(true, Ordering::SeqCst);
                req.fetch_add(1, Ordering::SeqCst);
                Some(damiao::build_disable_frame(d.motor_ids[0]))
            },
        );
        assert!(res.is_ok(), "DisableProtective must remain ungated across build-time stop flip: {res:?}");
        assert_eq!(bus.tx_len(), 1);

        // D. `DisableRequested` on a raised arm refuses if stop flips DURING `build` (Step 5 re-check):
        //    0 CAN frames transmitted.
        let (bus, clock) = TestBus::new();
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1]).with_limits(limits);
        drv.read_joint_states().unwrap();
        bus.push_feedback(1, 0.50, 1, &limits);
        drv.read_joint_states().unwrap();
        bus.clear_tx();
        let safety_stopped = Arc::new(AtomicBool::new(false));
        drv.set_stopped_disable_gate(StoppedDisableGate {
            safety_stopped: safety_stopped.clone(),
            has_resting_pose: true,
            arm_joints: [0; crate::rt_core::mailbox::MAX_DOF],
            arm_joints_len: 1,
            rest_positions: [0.0; crate::rt_core::mailbox::MAX_DOF],
            rest_pose_tolerance_rad: 0.05,
        });
        let started_at = clock.now();
        let res = drv.send_plain(
            TxKind::DisableRequested { started_at },
            None,
            None,
            false,
            |d| {
                safety_stopped.store(true, Ordering::SeqCst);
                Some(damiao::build_disable_frame(d.motor_ids[0]))
            },
        );
        assert!(
            matches!(res, Err(TxRefusal::TorqueOffWouldDrop)),
            "DisableRequested on raised arm must refuse even before stop flips: {res:?}"
        );
        assert_eq!(bus.tx_len(), 0);
    }

    #[test]
    fn send_chokepoint_enforces_rest_pose_and_1s_lifetime_for_disable_requested() {
        let limits = DamiaoLimits::default();
        let (bus, clock) = TestBus::new();
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1]).with_limits(limits);
        let safety_stopped = Arc::new(AtomicBool::new(true));
        drv.set_stopped_disable_gate(StoppedDisableGate {
            safety_stopped: safety_stopped.clone(),
            has_resting_pose: true,
            arm_joints: [0; crate::rt_core::mailbox::MAX_DOF],
            arm_joints_len: 1,
            rest_positions: [0.0; crate::rt_core::mailbox::MAX_DOF],
            rest_pose_tolerance_rad: 0.05,
        });

        // 1. Raised arm -> refused with TorqueOffWouldDrop, 0 TX.
        drv.read_joint_states().unwrap();
        bus.push_feedback(1, 0.40, 1, &limits);
        drv.read_joint_states().unwrap();
        bus.clear_tx();
        let t0 = clock.now();
        assert!(matches!(
            drv.send_plain(TxKind::DisableRequested { started_at: t0 }, None, None, false, |d| {
                Some(damiao::build_disable_frame(d.motor_ids[0]))
            }),
            Err(TxRefusal::TorqueOffWouldDrop)
        ));
        assert_eq!(bus.tx_len(), 0);

        // 2. At rest with K=3 spaced causal samples (S2) -> sent.
        drv.read_joint_states().unwrap();
        for step in 0..3 {
            if step > 0 {
                clock.advance(Duration::from_millis(10));
            }
            bus.push_feedback(1, 0.01, 1, &limits);
            drv.read_joint_states().unwrap();
        }
        bus.clear_tx();
        assert!(drv
            .send_plain(TxKind::DisableRequested { started_at: t0 }, None, None, false, |d| {
                Some(damiao::build_disable_frame(d.motor_ids[0]))
            })
            .is_ok());
        assert_eq!(bus.tx_len(), 1);

        // 3. Stale feedback (> 100 ms) within 1 s lifetime -> defers with DisableWaitEvidence (M3), 0 TX.
        clock.advance(Duration::from_millis(101));
        bus.clear_tx();
        assert!(matches!(
            drv.send_plain(TxKind::DisableRequested { started_at: t0 }, None, None, false, |d| {
                Some(damiao::build_disable_frame(d.motor_ids[0]))
            }),
            Err(TxRefusal::DisableWaitEvidence)
        ));
        assert_eq!(bus.tx_len(), 0);

        // 4. Fresh feedback at rest, but 1 s total request lifetime elapsed -> DisableRequestExpired, 0 TX.
        clock.advance(Duration::from_millis(950));
        drv.read_joint_states().unwrap();
        for step in 0..3 {
            if step > 0 {
                clock.advance(Duration::from_millis(10));
            }
            bus.push_feedback(1, 0.0, 1, &limits);
            drv.read_joint_states().unwrap();
        }
        bus.clear_tx();
        assert!(matches!(
            drv.send_plain(TxKind::DisableRequested { started_at: t0 }, None, None, false, |d| {
                Some(damiao::build_disable_frame(d.motor_ids[0]))
            }),
            Err(TxRefusal::DisableRequestExpired)
        ));
        assert_eq!(bus.tx_len(), 0);

        // 5. NO already-off bypass on a raised arm: fresh status 0x0 (disabled), 0xD (watchdog),
        //    0x1 (enabled), and 0x8 (overvoltage) are ALL refused with TorqueOffWouldDrop and 0 TX.
        let (bus2, clock2) = TestBus::new();
        let mut drv2 = DamiaoArmDriver::new(bus2.clone(), vec![1, 2, 3, 4]).with_limits(limits);
        let mut arm_joints = [0usize; crate::rt_core::mailbox::MAX_DOF];
        arm_joints[0] = 0;
        arm_joints[1] = 1;
        arm_joints[2] = 2;
        arm_joints[3] = 3;
        drv2.set_stopped_disable_gate(StoppedDisableGate {
            safety_stopped: Arc::new(AtomicBool::new(true)),
            has_resting_pose: true,
            arm_joints,
            arm_joints_len: 4,
            rest_positions: [0.0; crate::rt_core::mailbox::MAX_DOF],
            rest_pose_tolerance_rad: 0.05,
        });
        drv2.read_joint_states().unwrap();
        bus2.push_feedback(1, 0.50, 0x0, &limits);
        bus2.push_feedback(2, 0.50, 0xD, &limits);
        bus2.push_feedback(3, 0.50, 0x1, &limits);
        bus2.push_feedback(4, 0.50, 0x8, &limits);
        drv2.read_joint_states().unwrap();
        bus2.clear_tx();
        let t2 = clock2.now();
        for joint in 0..4 {
            assert!(matches!(
                drv2.send_plain(TxKind::DisableRequested { started_at: t2 }, None, None, false, |d| {
                    Some(damiao::build_disable_frame(d.motor_ids[joint]))
                }),
                Err(TxRefusal::TorqueOffWouldDrop)
            ));
        }
        assert_eq!(bus2.tx_len(), 0, "raised DisableRequested must send 0 frames regardless of motor status");

        // 6. Operational (not stopped) ALSO enforces the rest-pose gate and 1 s lifetime (B5):
        //    raised pose and expired request are BOTH refused with 0 TX even when safety_stopped == false.
        safety_stopped.store(false, Ordering::SeqCst);
        drv.read_joint_states().unwrap();
        bus.push_feedback(1, 0.80, 1, &limits);
        drv.read_joint_states().unwrap();
        let t_op = clock.now();
        bus.clear_tx();
        assert!(matches!(
            drv.send_plain(TxKind::DisableRequested { started_at: t_op }, None, None, false, |d| {
                Some(damiao::build_disable_frame(d.motor_ids[0]))
            }),
            Err(TxRefusal::TorqueOffWouldDrop)
        ));
        assert_eq!(bus.tx_len(), 0, "operational DisableRequested at raised pose must send 0 TX (B5)");
        clock.advance(Duration::from_millis(500));
        bus.clear_tx();
        assert!(matches!(
            drv.send_plain(TxKind::DisableRequested { started_at: t0 }, None, None, false, |d| {
                Some(damiao::build_disable_frame(d.motor_ids[0]))
            }),
            Err(TxRefusal::DisableRequestExpired)
        ));
        assert_eq!(bus.tx_len(), 0, "operational DisableRequested past 1 s lifetime must send 0 TX (B5)");
    }

    #[test]
    fn write_measured_hold_masked_holds_every_joint_even_when_feedback_claims_state_0() {
        let limits = damiao::DamiaoLimits::default();
        let (bus, clock) = TestBus::new();
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3])
            .with_joint_limits(vec![limits; 3])
            .unwrap();
        drv.verified_modes.fill(Some(DamiaoControlMode::PosVel));
        let mut arm_joints = [0usize; crate::rt_core::mailbox::MAX_DOF];
        arm_joints[0] = 0;
        arm_joints[1] = 1;
        arm_joints[2] = 2;
        drv.set_stopped_disable_gate(StoppedDisableGate {
            safety_stopped: Arc::new(AtomicBool::new(false)),
            has_resting_pose: true,
            arm_joints,
            arm_joints_len: 3,
            rest_positions: [0.50, 0.50, 0.50, 0.0, 0.0, 0.0, 0.0, 0.0],
            rest_pose_tolerance_rad: 0.05,
        });
        drv.read_joint_states().unwrap();
        for step in 0..3 {
            if step > 0 {
                clock.advance(Duration::from_millis(10));
            }
            for id in 1..=3 {
                bus.push_feedback(id, 0.50, 0x0, &limits);
            }
            drv.read_joint_states().unwrap();
        }
        // Feedback reports state=0 (telemetry Disabled), yet holds are commanded to EVERY joint.
        assert_eq!(drv.enable_state(), EnableState::Disabled);
        let targets = [
            JointTarget::position(0.50),
            JointTarget::position(0.50),
            JointTarget::position(0.50),
        ];
        bus.clear_tx();
        let report = drv.write_measured_hold_masked(0b111, &targets).unwrap();
        assert_eq!(report.joint_status[..3], [JointHoldStatus::Held; 3]);
        assert_eq!(bus.tx_len(), 3, "holds must be emitted to every joint even when state=0");

        // Bounded disable repeats: 3 repeats spaced 20 ms apart complete via decision without waiting
        // on 150 ms feedback confirmation.
        bus.clear_tx();
        let started_at = clock.now();
        let mut prog = drv
            .begin_disable_torque(DisableKind::DisableRequested { started_at })
            .unwrap();
        assert!(matches!(prog, TorqueProgress::Pending { .. }));
        assert_eq!(bus.tx_len(), 3, "repeat 1 sent to all 3 motors");
        clock.advance(DAMIAO_DISABLE_REPEAT_INTERVAL);
        prog = drv.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
        assert!(matches!(prog, TorqueProgress::Pending { .. }));
        assert_eq!(bus.tx_len(), 6, "repeat 2 sent to all 3 motors");
        clock.advance(DAMIAO_DISABLE_REPEAT_INTERVAL);
        prog = drv.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
        assert_eq!(prog, TorqueProgress::Done(TorqueOutcome::Confirmed));
        assert_eq!(bus.tx_len(), 9, "repeat 3 sent to all 3 motors");
    }

    #[test]
    fn fashionstar_and_robstride_gate_disable_per_write_across_estop_freshness_and_lifetime() {
        // Hookable serial transport for FashionStar per-write gating tests.
        #[derive(Clone)]
        struct HookSerial {
            clock: MockClock,
            writes: Arc<Mutex<Vec<Vec<u8>>>>,
            on_write: Arc<Mutex<Option<Box<dyn FnMut(usize) + Send>>>>,
        }
        impl HookSerial {
            fn new(clock: MockClock) -> Self {
                Self {
                    clock,
                    writes: Arc::new(Mutex::new(Vec::new())),
                    on_write: Arc::new(Mutex::new(None)),
                }
            }
            fn set_on_write(&self, cb: impl FnMut(usize) + Send + 'static) {
                *self.on_write.lock().unwrap() = Some(Box::new(cb));
            }
            fn clear_writes(&self) {
                self.writes.lock().unwrap().clear();
            }
            fn write_count(&self) -> usize {
                self.writes.lock().unwrap().len()
            }
        }
        impl Transport for HookSerial {
            fn write(&mut self, data: &[u8]) -> Result<usize, TransportError> {
                self.write_all(data)?;
                Ok(data.len())
            }
            fn write_all(&mut self, data: &[u8]) -> Result<(), TransportError> {
                let n = {
                    let mut w = self.writes.lock().unwrap();
                    w.push(data.to_vec());
                    w.len()
                };
                if let Some(cb) = self.on_write.lock().unwrap().as_mut() {
                    cb(n);
                }
                Ok(())
            }
            fn read(&mut self, _buf: &mut [u8]) -> Result<usize, TransportError> {
                Err(TransportError::Timeout)
            }
            fn flush(&mut self) -> Result<(), TransportError> {
                Ok(())
            }
            fn clear_input_buffer(&mut self) {}
            fn clock(&self) -> Option<Arc<dyn DriverClock>> {
                Some(Arc::new(self.clock.clone()))
            }
        }

        let mut arm3 = [0usize; crate::rt_core::mailbox::MAX_DOF];
        arm3[0] = 0;
        arm3[1] = 1;
        arm3[2] = 2;
        let seed_rest_window = |windows: &mut [CausalWindow], now: Instant| {
            let t0 = now.checked_sub(Duration::from_millis(20)).unwrap_or(now);
            let t1 = now.checked_sub(Duration::from_millis(10)).unwrap_or(now);
            for w in windows.iter_mut() {
                w.reset();
                let _ = w.record_agree(t0, 1, STOPPED_DISABLE_FEEDBACK_MAX_AGE, None);
                let _ = w.record_agree(t1, 2, STOPPED_DISABLE_FEEDBACK_MAX_AGE, None);
                let _ = w.record_agree(now, 3, STOPPED_DISABLE_FEEDBACK_MAX_AGE, None);
            }
        };

        // --- A. FashionStarArmDriver per-write gating ---
        {
            let clock = MockClock::new();
            let serial = HookSerial::new(clock.clone());
            let mut drv = FashionStarArmDriver::new(serial.clone(), vec![0, 1, 2]);
            let stopped = Arc::new(AtomicBool::new(false));
            drv.set_stopped_disable_gate(StoppedDisableGate {
                safety_stopped: Arc::clone(&stopped),
                has_resting_pose: true,
                arm_joints: arm3,
                arm_joints_len: 3,
                rest_positions: [0.0; crate::rt_core::mailbox::MAX_DOF],
                rest_pose_tolerance_rad: 0.05,
            });
            let t0 = clock.now();
            // 1. Raised arm when NOT stopped -> write #1 refused immediately (0 writes, B5)!
            for i in 0..3 {
                drv.cached_states[i].position = 0.50;
                drv.last_feedback_instant[i] = Some(t0);
            }
            let res = drv.disable_torque(DisableKind::DisableRequested { started_at: t0 });
            assert!(
                matches!(res, Err(ArmDriverError::TorqueOffWouldDrop)),
                "FashionStar raised operational DisableRequested must refuse write #1, got {res:?}"
            );
            assert_eq!(serial.write_count(), 0, "FashionStar must transmit 0 writes on raised arm (B5)");

            // 2. Mid-batch freshness cutoff (> 100 ms) at rest -> writes #2,#3 refused!
            serial.clear_writes();
            clock.advance(Duration::from_millis(20));
            let t1 = clock.now();
            for i in 0..3 {
                drv.cached_states[i].position = 0.0;
                drv.last_feedback_instant[i] = Some(t1);
            }
            seed_rest_window(&mut drv.rest_pose_window, t1);
            let clk_cb = clock.clone();
            serial.set_on_write(move |n| {
                if n == 1 {
                    clk_cb.advance(Duration::from_millis(105));
                }
            });
            let res = drv.disable_torque(DisableKind::DisableRequested { started_at: t1 });
            assert!(
                matches!(res, Err(ArmDriverError::TorqueOffWouldDrop)),
                "FashionStar mid-batch freshness cutoff must refuse write #2+, got {res:?}"
            );
            assert_eq!(serial.write_count(), 1);

            // 3. Mid-batch 1 s lifetime cutoff at rest (feedback stays fresh!) -> DisableRequestExpired!
            serial.clear_writes();
            let t_req = clock.now();
            clock.advance(Duration::from_millis(995));
            let t_fb = clock.now();
            for i in 0..3 {
                drv.cached_states[i].position = 0.0;
                drv.last_feedback_instant[i] = Some(t_fb);
            }
            seed_rest_window(&mut drv.rest_pose_window, t_fb);
            let clk_cb2 = clock.clone();
            serial.set_on_write(move |n| {
                if n == 1 {
                    clk_cb2.advance(Duration::from_millis(10)); // past 1 s, feedback still only 10 ms old
                }
            });
            let res = drv.disable_torque(DisableKind::DisableRequested { started_at: t_req });
            assert!(
                matches!(res, Err(ArmDriverError::DisableRequestExpired)),
                "FashionStar mid-batch 1 s lifetime cutoff must refuse write #2+, got {res:?}"
            );
            assert_eq!(serial.write_count(), 1);

            // 4. DisableProtective sends all 3 writes regardless of E-stop / staleness.
            serial.clear_writes();
            serial.set_on_write(|_| {});
            drv.disable_torque(DisableKind::DisableProtective {
                reason: DisableProtectiveReason::Thermal,
            })
            .unwrap();
            assert_eq!(serial.write_count(), 3);
        }

        // --- B. RobStrideArmDriver per-write gating ---
        {
            let (bus, clock) = TestBus::new();
            let mut drv = RobStrideArmDriver::new(bus.clone(), vec![1, 2, 3]);
            let stopped = Arc::new(AtomicBool::new(false));
            drv.set_stopped_disable_gate(StoppedDisableGate {
                safety_stopped: Arc::clone(&stopped),
                has_resting_pose: true,
                arm_joints: arm3,
                arm_joints_len: 3,
                rest_positions: [0.0; crate::rt_core::mailbox::MAX_DOF],
                rest_pose_tolerance_rad: 0.05,
            });
            let t0 = clock.now();
            // 1. Raised arm when NOT stopped -> frame #1 refused immediately (0 TX, B5)!
            for i in 0..3 {
                drv.cached_states[i].position = 0.50;
                drv.last_feedback_instant[i] = Some(t0);
                drv.pose_feedback_instant[i] = Some(t0);
                drv.feedback_queue_bound[i] = Some(t0);
            }
            let res = drv.disable_torque(DisableKind::DisableRequested { started_at: t0 });
            assert!(
                matches!(res, Err(ArmDriverError::TorqueOffWouldDrop)),
                "RobStride raised operational DisableRequested must refuse frame #1, got {res:?}"
            );
            assert_eq!(bus.tx_len(), 0, "RobStride must transmit 0 frames on raised arm (B5)");

            // 2. Mid-batch freshness cutoff (> 100 ms) at rest -> frames #2,#3 refused!
            bus.clear_tx();
            clock.advance(Duration::from_millis(20));
            let t1 = clock.now();
            for i in 0..3 {
                drv.cached_states[i].position = 0.0;
                drv.last_feedback_instant[i] = Some(t1);
                drv.pose_feedback_instant[i] = Some(t1);
                drv.feedback_queue_bound[i] = Some(t1);
            }
            seed_rest_window(&mut drv.rest_pose_window, t1);
            let clk_cb = clock.clone();
            let mut count2 = 0usize;
            bus.set_on_tx(move |_| {
                count2 += 1;
                if count2 == 1 {
                    clk_cb.advance(Duration::from_millis(105));
                }
                Ok(())
            });
            let res = drv.disable_torque(DisableKind::DisableRequested { started_at: t1 });
            assert!(
                matches!(res, Err(ArmDriverError::TorqueOffWouldDrop)),
                "RobStride mid-batch freshness cutoff must refuse frame #2+, got {res:?}"
            );
            assert_eq!(bus.tx_len(), 1);

            // 3. Mid-batch 1 s lifetime cutoff at rest (feedback stays fresh!) -> DisableRequestExpired!
            bus.clear_tx();
            let t_req = clock.now();
            clock.advance(Duration::from_millis(995));
            let t_fb = clock.now();
            for i in 0..3 {
                drv.cached_states[i].position = 0.0;
                drv.last_feedback_instant[i] = Some(t_fb);
                drv.pose_feedback_instant[i] = Some(t_fb);
                drv.feedback_queue_bound[i] = Some(t_fb);
            }
            seed_rest_window(&mut drv.rest_pose_window, t_fb);
            let clk_cb2 = clock.clone();
            let mut count3 = 0usize;
            bus.set_on_tx(move |_| {
                count3 += 1;
                if count3 == 1 {
                    clk_cb2.advance(Duration::from_millis(10));
                }
                Ok(())
            });
            let res = drv.disable_torque(DisableKind::DisableRequested { started_at: t_req });
            assert!(
                matches!(res, Err(ArmDriverError::DisableRequestExpired)),
                "RobStride mid-batch 1 s lifetime cutoff must refuse frame #2+, got {res:?}"
            );
            assert_eq!(bus.tx_len(), 1);

            // 4. DisableProtective sends all 3 frames regardless of E-stop / staleness.
            bus.clear_tx();
            bus.set_on_tx(|_| Ok(()));
            drv.disable_torque(DisableKind::DisableProtective {
                reason: DisableProtectiveReason::Thermal,
            })
            .unwrap();
            assert_eq!(bus.tx_len(), 3);

            // 5. RobStride `clear_fault()` (type-4 stop/fault-clear frames) routes through the
            //    exact same per-write disable choke point: refused raised / stale / mid-batch
            //    E-stop / freshness / 1 s lifetime, allowed at fresh rest pose.
            stopped.store(true, Ordering::SeqCst);
            let t_cf = clock.now();
            for i in 0..3 {
                drv.cached_states[i].position = 0.50;
                drv.last_feedback_instant[i] = Some(t_cf);
                drv.pose_feedback_instant[i] = Some(t_cf);
                drv.feedback_queue_bound[i] = Some(t_cf);
            }
            bus.clear_tx();
            assert!(matches!(
                drv.clear_fault(),
                Err(ArmDriverError::TorqueOffWouldDrop)
            ));
            assert_eq!(bus.tx_len(), 0, "RobStride clear_fault while stopped & raised must send 0 frames");

            // At rest with stale (> 100 ms) feedback -> refused with 0 frames.
            for i in 0..3 {
                drv.cached_states[i].position = 0.0;
            }
            clock.advance(Duration::from_millis(105));
            bus.clear_tx();
            assert!(matches!(
                drv.clear_fault(),
                Err(ArmDriverError::TorqueOffWouldDrop)
            ));
            assert_eq!(bus.tx_len(), 0, "RobStride clear_fault while stopped & stale must send 0 frames");

            // Raised arm when NOT stopped -> clear_fault refuses frame #1 immediately (0 TX, B5)!
            stopped.store(false, Ordering::SeqCst);
            let t_cf2 = clock.now();
            for i in 0..3 {
                drv.cached_states[i].position = 0.50;
                drv.last_feedback_instant[i] = Some(t_cf2);
                drv.pose_feedback_instant[i] = Some(t_cf2);
                drv.feedback_queue_bound[i] = Some(t_cf2);
            }
            bus.set_on_tx(|_| Ok(()));
            bus.clear_tx();
            assert!(matches!(
                drv.clear_fault(),
                Err(ArmDriverError::TorqueOffWouldDrop)
            ));
            assert_eq!(bus.tx_len(), 0, "RobStride clear_fault on raised operational arm must transmit 0 frames (B5)");

            // Mid-batch freshness cutoff (> 100 ms) while stopped at rest stops after frame #1.
            clock.advance(Duration::from_millis(20));
            let t_cf3 = clock.now();
            for i in 0..3 {
                drv.cached_states[i].position = 0.0;
                drv.last_feedback_instant[i] = Some(t_cf3);
                drv.pose_feedback_instant[i] = Some(t_cf3);
                drv.feedback_queue_bound[i] = Some(t_cf3);
            }
            seed_rest_window(&mut drv.rest_pose_window, t_cf3);
            let clk_cf = clock.clone();
            let mut count_cf2 = 0usize;
            bus.set_on_tx(move |_| {
                count_cf2 += 1;
                if count_cf2 == 1 {
                    clk_cf.advance(Duration::from_millis(105));
                }
                Ok(())
            });
            bus.clear_tx();
            assert!(matches!(
                drv.clear_fault(),
                Err(ArmDriverError::TorqueOffWouldDrop)
            ));
            assert_eq!(bus.tx_len(), 1, "RobStride clear_fault must stop after frame #1 on mid-batch staleness");

            // Fresh rest pose while stopped -> transmits all 3 type-4 clear-fault frames.
            clock.advance(Duration::from_millis(20));
            let t_cf4 = clock.now();
            for i in 0..3 {
                drv.cached_states[i].position = 0.0;
                drv.last_feedback_instant[i] = Some(t_cf4);
                drv.pose_feedback_instant[i] = Some(t_cf4);
                drv.feedback_queue_bound[i] = Some(t_cf4);
            }
            seed_rest_window(&mut drv.rest_pose_window, t_cf4);
            bus.set_on_tx(|_| Ok(()));
            bus.clear_tx();
            drv.clear_fault().expect("RobStride clear_fault at fresh rest pose must succeed");
            assert_eq!(bus.tx_len(), 3);
        }
    }

    /// Round 8 D1: Boot fallback (`PosForceBootFallback::NoOutputUntilFresh`).
    /// 1. With no provably fresh J7 sample since process start (`pf_last_fresh_position == None`),
    ///    emit ZERO `0x307` frames while keeping `0xCC` refresh requests active (J7 keeps its last
    ///    firmware command under `TIMEOUT=0`).
    /// 2. After the first provably fresh J7 sample is decoded, emit a stationary `0x307` Hold at
    ///    that measured position (`vel_raw = 0`) with the configured `current_raw` cap (even if a
    ///    `Target` intent is submitted on that first fresh tick).
    /// 3. A STOP before the first fresh sample still sends NO `0x307` frame (while sending `0xCC`),
    ///    and is applied as Hold at the measured position with the cap once fresh.
    #[test]
    fn pos_force_boot_fallback_no_output_until_fresh_then_holds_measured_with_cap() {
        let limits = damiao::DamiaoLimits::default();
        let (bus, clock) = TestBus::new();
        let pf = GripperPosForce {
            joint: 0,
            current_raw: 460,
            close_speed_rad_s: 0.30,
            closed_rad: 0.0,
            open_rad: -5.7,
            calibrated: false,
            ceiling_torque_nm: None,
            torque_reference: None,
        };
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![7])
            .with_limits(limits)
            .with_gripper_pos_force(pf)
            .unwrap();
        // Simulate verified PosForce mode with NO provably fresh J7 sample ever decoded since boot.
        drv.pos_force_verified = true;
        drv.joint_modes[0] = Some(DamiaoControlMode::PosForce);
        drv.pf_last_fresh_position = None;
        drv.is_enabled = true;

        assert_eq!(POS_FORCE_BOOT_FALLBACK, PosForceBootFallback::NoOutputUntilFresh);
        assert_eq!(drv.pos_force_freeze_params(0), None);

        // 1. Before first fresh sample: Target intent -> ZERO 0x307 frames, 0xCC refresh active.
        bus.clear_tx();
        drv.begin_tx_tick();
        let stamp = drv.current_j7_intent_stamp();
        let out0 = drv
            .emit_pos_force_j7(PosForceIntent::target(0.0, Some(0.30), None, stamp), stamp)
            .expect("boot fallback awaiting fresh");
        assert!(
            matches!(
                out0,
                PosForceEmitOutcome::AwaitingFirstFresh {
                    joint: 0,
                    pf_status: PosForceTxStatus::Stale { joint: 0, .. }
                }
            ),
            "expected AwaitingFirstFresh with Stale status, got {out0:?}"
        );
        let log0: Vec<CanFrame> = bus.tx.lock().unwrap().clone();
        assert!(
            log0.iter().all(|f| f.id != 0x307),
            "must send NO 0x307 frame before first fresh sample: {log0:?}"
        );
        assert!(
            log0.iter().any(|f| f.id == 0x7FF && f.data[0] == 7 && f.data[2] == 0xCC),
            "expected 0xCC refresh while awaiting first fresh sample: {log0:?}"
        );

        // 2. First fresh J7 sample arrives (-2.40 rad) -> next tick emits Hold at -2.40 rad with
        //    vel_raw = 0 and configured current_raw = 460 (even if a Target intent is submitted).
        let now1 = clock.now();
        drv.cached_states[0].position = -2.40;
        drv.last_feedback_instant[0] = Some(now1);
        drv.feedback_queue_bound[0] = Some(now1);
        drv.pf_last_fresh_position = Some(-2.40);
        bus.clear_tx();
        drv.begin_tx_tick();
        let out1 = drv
            .emit_pos_force_j7(PosForceIntent::target(0.0, Some(0.30), None, stamp), stamp)
            .expect("first fresh hold");
        assert!(matches!(out1, PosForceEmitOutcome::Sent { joint: 0, .. }));
        let log1: Vec<CanFrame> = bus.tx.lock().unwrap().clone();
        let j7_1: Vec<CanFrame> = log1.iter().copied().filter(|f| f.id == 0x307).collect();
        assert_eq!(j7_1.len(), 1, "expected 1 Hold 0x307 frame on first fresh sample: {log1:?}");
        assert!((f32::from_le_bytes(j7_1[0].data[0..4].try_into().unwrap()) - (-2.40)).abs() < 1e-6);
        assert_eq!(u16::from_le_bytes([j7_1[0].data[4], j7_1[0].data[5]]), 0);
        assert_eq!(u16::from_le_bytes([j7_1[0].data[6], j7_1[0].data[7]]), 460);

        // 3. STOP before the first fresh sample: still sends NO 0x307 frame (sends 0xCC), and
        //    once the first fresh sample arrives (-1.75 rad), applies as Hold with cap 460.
        drv.reset_pos_force_boot_state();
        let req = Arc::new(AtomicU64::new(1));
        let app = Arc::new(AtomicU64::new(1));
        let latched = Arc::new(AtomicBool::new(true));
        drv.set_estop_gate(EstopGate::new(req, app));
        drv.set_abort_flag(latched);

        bus.clear_tx();
        drv.begin_tx_tick();
        let targets = [JointTarget::position(0.0)];
        let outcome = drv.write_estop_hold_bounded(&targets).expect("stop before first fresh");
        assert_eq!(outcome, TxBatchOutcome::Complete);
        let log_stop_pre: Vec<CanFrame> = bus.tx.lock().unwrap().clone();
        assert!(
            log_stop_pre.iter().all(|f| f.id != 0x307),
            "STOP before first fresh sample must send NO 0x307 frame: {log_stop_pre:?}"
        );
        assert!(
            log_stop_pre.iter().any(|f| f.id == 0x7FF && f.data[0] == 7 && f.data[2] == 0xCC),
            "STOP before first fresh sample must still request 0xCC refresh: {log_stop_pre:?}"
        );

        let now2 = clock.now();
        drv.cached_states[0].position = -1.75;
        drv.last_feedback_instant[0] = Some(now2);
        drv.feedback_queue_bound[0] = Some(now2);
        drv.pf_last_fresh_position = Some(-1.75);
        bus.clear_tx();
        drv.begin_tx_tick();
        let outcome2 = drv.write_estop_hold_bounded(&targets).expect("stop applied once fresh");
        assert_eq!(outcome2, TxBatchOutcome::Complete);
        let log_stop_post: Vec<CanFrame> = bus.tx.lock().unwrap().clone();
        let j7_post: Vec<CanFrame> = log_stop_post.iter().copied().filter(|f| f.id == 0x307).collect();
        assert_eq!(j7_post.len(), 1, "STOP must apply as Hold once first fresh sample arrives: {log_stop_post:?}");
        assert!((f32::from_le_bytes(j7_post[0].data[0..4].try_into().unwrap()) - (-1.75)).abs() < 1e-6);
        assert_eq!(u16::from_le_bytes([j7_post[0].data[4], j7_post[0].data[5]]), 0);
        assert_eq!(u16::from_le_bytes([j7_post[0].data[6], j7_post[0].data[7]]), 460);
    }

    /// Round 8 M1: Type-system capability guard enforcement.
    /// 1. `DamiaoArmDriver::send` always passes `pos_force_token = None`, so even a forged
    ///    raw `0x307` `CanFrame` built without `PosForceSendToken` is refused with `BuildRefused`
    ///    and never reaches the transport.
    /// 2. `pos_force_emitter::emit_standalone_recovery_hold` refuses with `BuildRefused` when
    ///    `external_pos_force_emitter == true` (owner tick epilogue active).
    #[test]
    fn send_without_pos_force_emitter_token_rejects_forged_0x307_frame() {
        let limits = damiao::DamiaoLimits::default();
        let (bus, clock) = TestBus::new();
        let pf = GripperPosForce {
            joint: 0,
            current_raw: 460,
            close_speed_rad_s: 0.30,
            closed_rad: 0.0,
            open_rad: -5.7,
            calibrated: false,
            ceiling_torque_nm: None,
            torque_reference: None,
        };
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![7])
            .with_limits(limits)
            .with_gripper_pos_force(pf)
            .unwrap();
        let now = clock.now();
        drv.pos_force_verified = true;
        drv.joint_modes[0] = Some(DamiaoControlMode::PosForce);
        drv.is_enabled = true;
        drv.cached_states[0].position = -1.0;
        drv.last_feedback_instant[0] = Some(now);
        drv.feedback_queue_bound[0] = Some(now);
        drv.pf_last_fresh_position = Some(-1.0);

        // 1. Even with fresh feedback and valid payload, `drv.send(...)` (no `PosForceSendToken`)
        //    refuses any `0x307` frame with `BuildRefused` and emits 0 wire frames.
        bus.clear_tx();
        let forged = CanFrame::new_standard(
            damiao::POS_FORCE_BASE_ID + 7,
            &damiao::encode_pos_force_payload(-1.0, 0, 460),
        )
        .unwrap();
        let res = drv.send_plain(TxKind::Target, None, None, false, |_| Some(forged));
        assert!(
            matches!(res, Err(TxRefusal::BuildRefused)),
            "forged 0x307 through send() without PosForceSendToken must fail with BuildRefused, got {res:?}"
        );
        assert_eq!(bus.tx_len(), 0, "forged 0x307 must never reach CAN transport");

        // 2. Narrow `emit_standalone_recovery_hold` fails closed when an external tick owner is active.
        drv.set_external_pos_force_emitter(true);
        let rec_res = pos_force_emitter::emit_standalone_recovery_hold(&mut drv, None, -1.0, 0.30);
        assert!(
            matches!(rec_res, Err(TxRefusal::BuildRefused)),
            "emit_standalone_recovery_hold must refuse when external_pos_force_emitter is true, got {rec_res:?}"
        );
        assert_eq!(bus.tx_len(), 0, "standalone recovery must emit 0 frames under external owner");

        // 3. The single tick emitter succeeds and transmits exactly one 0x307 frame (both Target
        //    and Hold).
        drv.begin_tx_tick();
        let stamp = drv.current_j7_intent_stamp();
        let out_t = pos_force_emitter::emit_tick(
            &mut drv,
            PosForceIntent::target(-0.8, Some(0.30), None, stamp),
            stamp,
            ContinuityBypass::None,
        )
        .expect("emit_tick target");
        assert!(matches!(out_t, PosForceEmitOutcome::Sent { joint: 0, freeze: false, .. }));
        let tx_t = bus.tx.lock().unwrap().clone();
        assert_eq!(tx_t.len(), 1);
        assert_eq!(tx_t[0].id, 0x307);

        bus.clear_tx();
        drv.begin_tx_tick();
        let out_h = pos_force_emitter::emit_tick(
            &mut drv,
            PosForceIntent::hold(stamp),
            stamp,
            ContinuityBypass::None,
        )
        .expect("emit_tick hold");
        assert!(matches!(out_h, PosForceEmitOutcome::Sent { joint: 0, .. }));
        let tx_h = bus.tx.lock().unwrap().clone();
        assert_eq!(tx_h.len(), 1);
        assert_eq!(tx_h[0].id, 0x307);
        assert_eq!(u16::from_le_bytes([tx_h[0].data[4], tx_h[0].data[5]]), 0);
        assert_eq!(u16::from_le_bytes([tx_h[0].data[6], tx_h[0].data[7]]), 460);
    }
}
