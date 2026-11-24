use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::model::JointState;
pub use crate::model::{DriverCapabilities, DriverRangeCheck, EnableState, JointRangeReading, MotorZeroSupport, RangeCheckStatus};
use crate::protocol::damiao::{self, DamiaoLimits};
pub use crate::protocol::damiao::{build_pos_force_frame, DAMIAO_GRIPPER_POSFORCE_VEL_RAW};
use crate::protocol::fashionstar::{self, FashionStarPacket};
use crate::protocol::feetech;
use crate::protocol::robstride::{self, RobStrideLimits};
use crate::transport::{CanFrame, CanTransport, Transport, TransportError};

pub mod handshake;
pub use handshake::{TorqueOutcome, TorqueProgress};
use handshake::{ConfirmWindow, Deadline, Service, TorqueDirection};

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

    #[error("joint {} CTRL_MODE readback: pre-enable hold not sent: TX budget exhausted; capped operation unavailable", joint + 1)]
    PosForcePreEnableHoldBudgetExhausted { joint: usize },

    #[error("joint {} CTRL_MODE readback: pre-enable hold not sent: {kind}; capped operation unavailable", joint + 1)]
    PosForcePreEnableHoldTxFailed { joint: usize, kind: TxFailKind },

    #[error("TX pending queue capacity ({capacity}) exceeded (queued {queued}, new {new})")]
    TxQueueFull { queued: usize, new: usize, capacity: usize },
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
/// Resend times of the enable and disable frames to motors not yet confirmed.
const DAMIAO_ENABLE_RETRY_AT: [Duration; 2] = [Duration::from_millis(25), Duration::from_millis(75)];
/// Damiao disable handshake: how long to wait for every motor to report state 0.
pub const DAMIAO_DISABLE_CONFIRM: Duration = Duration::from_millis(150);
/// Time bound of one Damiao receive pass (with at most 64 frames).
pub const DAMIAO_RX_PASS: Duration = Duration::from_millis(5);
/// Maximum age for Damiao joint feedback to still be considered valid.
const DAMIAO_FEEDBACK_VALID_WINDOW: Duration = Duration::from_millis(250);

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
/// (`<= 100 ms`, bounded, finite) feedback sample since boot (`pf_last_fresh_position == None`).
///
/// - [`Self::ZeroCurrent`] (default): emit `vel_raw = 0, i_des = 0` at the last commanded `0x307`
///   position (or `0.0`).
/// - [`Self::HoldLastCommandedCurrent`]: emit `vel_raw = 0` at the last commanded `0x307` position
///   (or `0.0`) with the last commanded `0x307` current (capped to the configured current ceiling).
///
/// Switching policies is a single-line change to [`POS_FORCE_BOOT_FALLBACK`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PosForceBootFallback {
    ZeroCurrent,
    HoldLastCommandedCurrent,
}

/// Active boot fallback when no provably fresh J7 sample has been decoded since boot.
/// Change this single constant to [`PosForceBootFallback::HoldLastCommandedCurrent`] to retain
/// the last commanded current instead of `i_des = 0`.
pub const POS_FORCE_BOOT_FALLBACK: PosForceBootFallback = PosForceBootFallback::ZeroCurrent;

/// Resolves the `i_des` current-raw field for the J7 `0x307` boot fallback when no provably fresh
/// J7 sample has been decoded since boot.
#[inline]
pub const fn pos_force_boot_fallback_current(
    last_commanded_current_raw: Option<u16>,
    configured_current_raw: u16,
) -> u16 {
    match POS_FORCE_BOOT_FALLBACK {
        PosForceBootFallback::ZeroCurrent => 0,
        PosForceBootFallback::HoldLastCommandedCurrent => match last_commanded_current_raw {
            Some(c) if c < configured_current_raw => c,
            _ => configured_current_raw,
        },
    }
}

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
/// `(intent, current_stamp, freshness)` into the exact `0x307` wire frame.
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
    /// J7 `0x307` frame was transmitted and flushed to the transport.
    Sent { joint: usize, freeze: bool, pf_status: PosForceTxStatus },
    /// J7 `0x307` frame was deferred by per-tick TX budget or backpressure.
    Deferred { joint: usize, pf_status: PosForceTxStatus },
}

pub trait ArmDriver: Send {
    fn capabilities(&self) -> DriverCapabilities { DriverCapabilities::default() }

    fn enable_state(&self) -> EnableState { EnableState::Unknown }

    /// Age of the last received sample for each joint. Reading a cache does not refresh it.
    fn feedback_ages(&self) -> Vec<Option<Duration>> { Vec::new() }

    /// Monotonic receive instant of the last received sample for each joint.
    /// Reading a cache does not refresh it. Default derives it from [`Self::feedback_ages`].
    fn feedback_instants(&self) -> Vec<Option<Instant>> {
        let now = Instant::now();
        self.feedback_ages().into_iter().map(|a| a.and_then(|age| now.checked_sub(age))).collect()
    }

    /// Monotonic per-joint hardware feedback sequence counter, incremented once per decoded
    /// hardware reply for that joint. Reading a cache does not advance it. Empty means the
    /// driver does not report explicit counters (supervisor falls back to instant advances).
    fn feedback_seqs(&self) -> Vec<u64> { Vec::new() }

    /// Per joint, a monotonic instant at which the driver saw its receive queue empty before it
    /// read that joint's last decoded sample, so the sample was enqueued after this instant
    /// (None: no empty queue was seen before it). Decode-time instants cannot bound queued
    /// frames: a bounded drain can decode a backlog several ticks after it arrived. Empty means
    /// the driver has no receive queue between bus and decode (request/response transports);
    /// callers then use [`Self::feedback_instants`].
    fn feedback_queue_bounds(&self) -> Vec<Option<Instant>> { Vec::new() }

    /// Hardware receive timestamps for each joint's latest feedback packet (if tracked).
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
    /// Every joint listed in `joints` (e.g. arm joints) has feedback no older than `max_age`
    /// (without allocating in `DamiaoArmDriver`).
    fn feedback_joints_within(&self, joints: &[usize], max_age: Duration) -> bool {
        let ages = self.feedback_ages();
        joints.iter().all(|&j| ages.get(j).copied().flatten().is_some_and(|age| age <= max_age))
    }
    /// [`Self::feedback_ages`] as whole milliseconds (the observation's `feedback_age_ms`).
    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
        out.clear();
        out.extend(self.feedback_ages().iter().map(|a| a.map(|v| v.as_millis().min(u64::MAX as u128) as u64)));
    }
    /// Calls `f` with [`Self::range_check`] (a driver that stores the result passes a
    /// reference instead of a copy).
    fn with_range_check(&self, f: &mut dyn FnMut(Option<&DriverRangeCheck>)) { f(self.range_check().as_ref()) }
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
    /// [`Self::take_recovery_writes`] into `out`.
    fn take_recovery_writes_into(&mut self, out: &mut Vec<usize>) { let w = self.take_recovery_writes(); out.clear(); out.extend_from_slice(&w); }
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

    /// Enable/disable external tick-epilogue ownership of the `PosForce` J7 emitter.
    ///
    /// When `true` (installed by `SupervisorInner` for both Legacy and RT Executor), arm write
    /// paths (`write_joint_targets_bounded`, `write_estop_hold_bounded`, `receive_feedback`) NEVER
    /// write `0x307` frames; `SupervisorInner`'s tick epilogue calls [`Self::emit_pos_force_j7`]
    /// exactly once per control tick after whatever arm-write path ran.
    fn set_external_pos_force_emitter(&mut self, _enabled: bool) {}

    /// Validate a caller's `PosForce` target parameters (finite position/velocity, non-zero
    /// wire speed quantization, non-zero current cap) without checking live feedback freshness
    /// or transmitting any CAN frame.
    fn validate_pos_force_intent_target(
        &self,
        _position: f32,
        _velocity: Option<f32>,
        _current_raw: Option<u16>,
    ) -> Result<(), ArmDriverError> {
        Ok(())
    }

    /// Single J7 `PosForce` (`0x307`) emitter called once per control tick from the tick epilogue
    /// on both owners (Legacy and RT Executor), after whatever arm-write path ran (whether that
    /// arm path succeeded, refused, tripped the MIT fence, hit `BuildRefused`, had an invalid
    /// caller target, or returned early).
    ///
    /// Always unfenced: a tripped MIT fence suppresses only MIT arm joints, never J7.
    /// Any `Target` intent whose `(stop_gen, estop_epoch)` stamp differs from `current` is
    /// degraded to `Hold` inside this single emitter (never re-stamped with a newer generation).
    fn emit_pos_force_j7(
        &mut self,
        _intent: PosForceIntent,
        _current: PosForceIntentStamp,
    ) -> Result<PosForceEmitOutcome, ArmDriverError> {
        Ok(PosForceEmitOutcome::NotApplicable)
    }

    /// True if the most recent bounded stopped-hold pass submitted at least one frame to the bus.
    fn last_hold_tx_sent_any(&self) -> bool { true }

    /// Bitmask of `(joint, slot)` hold frames (`1u64 << (joint * 2 + slot)`) that were due in the
    /// most recent bounded stopped-hold pass (`0` when nothing was due or the driver has no
    /// bounded hold queue).
    fn last_hold_tx_due_mask(&self) -> u64 { 0 }

    /// Bitmask of `(joint, slot)` hold frames (`1u64 << (joint * 2 + slot)`) that actually reached
    /// the transport (transmitted and flushed) in the most recent bounded stopped-hold pass.
    fn last_hold_tx_sent_mask(&self) -> u64 { 0 }

    /// Bitmask of 0-based joint indices (`1u64 << i`) that completed a motion target transmit
    /// (and flush) with no unsent target remaining in the carry-over queue during the most recent
    /// [`Self::write_joint_targets_bounded`] pass. Non-CAN/sim drivers default to `u64::MAX`.
    fn last_target_tx_sent_mask(&self) -> u64 { u64::MAX }

    /// Consumes once the position-force gripper freshness status observed during the most recent
    /// bounded motion or stopped-hold write pass (`Unobserved` on non-pos-force drivers).
    fn take_last_pos_force_tx_status(&mut self) -> PosForceTxStatus { PosForceTxStatus::Unobserved }

    /// Live check whether the active position-force gripper joint currently has provably fresh
    /// (`<= 100 ms` on both receive instant and receive-queue bound, finite position) feedback.
    /// Returns `true` when no active position-force gripper is configured.
    fn pos_force_feedback_fresh(&self) -> bool { true }

    /// Last provably fresh (`<= 100 ms`, bounded, finite) measured position observed for the
    /// position-force gripper joint (`None` when no pos-force gripper is configured or no fresh
    /// sample has been observed yet). Used for J7 freeze frames when J7 feedback is stale.
    fn pos_force_last_fresh_position(&self) -> Option<f32> { None }

    /// Bounded target write: distinguishes complete submission ([`TxBatchOutcome::Complete`])
    /// from transient deferral ([`TxBatchOutcome::Deferred`]) without faulting or authorizing
    /// motion completion on deferral. Drivers and wrappers that support pending-queue carry-over
    /// override this method; the default forwards to fail-closed [`Self::write_joint_targets`].
    fn write_joint_targets_bounded(&mut self, targets: &[JointTarget]) -> Result<TxBatchOutcome, ArmDriverError> {
        self.write_joint_targets(targets).map(|()| TxBatchOutcome::Complete)
    }

    /// Discard any carried-over pending TX frames (no-op for drivers without a TX queue).
    fn discard_pending_tx(&mut self) {}

    /// Sets an absolute driver-clock ceiling on the active release's TX start-cutoff
    /// (`tx_deadline = min(first_tx + tx_budget, ceiling)`). Pass `None` when leaving a
    /// control release so off-tick calls are never clipped by a prior release deadline.
    fn set_tx_release_deadline(&mut self, _deadline: Option<Instant>) {}

    /// Configured inclusive maximum age for carried-over TX frames (default 45 ms).
    fn tx_frame_max_age(&self) -> Duration { DAMIAO_TX_FRAME_MAX_AGE }

    /// Observed age of the oldest frame dropped as expired during the most recent bounded write.
    fn last_tx_expired_age(&self) -> Option<Duration> { None }

    /// Observed age of the oldest frame currently queued in the pending TX queue.
    fn oldest_pending_tx_age(&self) -> Option<Duration> { None }

    /// Number of unsent motion target frames currently queued in the bounded TX carry-over queue.
    fn pending_motion_tx_len(&self) -> usize { 0 }

    /// Number of unsent stopped-hold frames currently queued in the bounded hold TX queue.
    fn pending_hold_tx_len(&self) -> usize { 0 }

    /// Total unsent frames across both motion and stopped-hold bounded TX carry-over queues.
    fn pending_tx_len_total(&self) -> usize {
        self.pending_motion_tx_len() + self.pending_hold_tx_len()
    }

    /// True when a bounded-CAN driver has confirmed healthy TX via a refresh/status transmit probe
    /// sent with no backpressure or error on an empty queue, followed by fresh feedback received
    /// strictly after that probe. Non-CAN/sim drivers default to `true`.
    fn tx_probe_confirmed(&self) -> bool { true }

    /// True when a TX probe was recently sent and is still awaiting post-probe fresh feedback.
    fn tx_probe_pending(&self) -> bool { false }

    /// Blocking transmit probe (Legacy `clear_fault` path): requires an empty TX queue, drains RX
    /// to establish a boundary, sends a read-only refresh/status frame without backpressure or
    /// error, flushes, and waits up to 50 ms for fresh post-probe feedback.
    /// Returns `Ok(true)` when post-probe fresh feedback arrived, `Ok(false)` when the probe
    /// transmitted cleanly but no reply arrived yet, or `Err` on backpressure/error.
    fn probe_tx_ready(&mut self) -> Result<bool, ArmDriverError> { Ok(true) }

    /// Non-blocking, zero-allocation budgeted transmit probe for the RT executor `clear_fault` path.
    fn probe_tx_ready_budgeted(&mut self, _deadline: Instant) -> Result<bool, ArmDriverError> { Ok(true) }

    /// Clear any recorded TX probe state (called on fault clear or new TX stall latch).
    fn clear_tx_probe(&mut self) {}

    /// Enable or disable torque / power across all joints
    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError>;

    /// Start a torque transition without waiting for its confirmation. `Pending { wake }`: serve
    /// it with [`ArmDriver::poll_torque`] no later than `wake` (absolute, driver clock) until it
    /// returns `Done`; nothing else may be sent to the arm meanwhile except a disable, which
    /// replaces a pending enable. Default: the blocking [`ArmDriver::set_torque`], `Done` on success.
    fn begin_torque(&mut self, enabled: bool) -> Result<TorqueProgress, ArmDriverError> {
        self.set_torque(enabled).map(|()| TorqueProgress::Done(TorqueOutcome::Confirmed))
    }

    /// Budgeted [`Self::begin_torque`]: when `max_rx_frames == 0`, performs no bus RX drain
    /// before transmitting (relying on the release's Step-2 poll boundary).
    fn begin_torque_budgeted(&mut self, enabled: bool, _max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
        self.begin_torque(enabled)
    }

    /// Serve the pending torque transition once: one receive pass ending by `deadline` (and the
    /// driver's own frame / time bound), at most one retransmit per joint, no sleep. Err when the
    /// transition failed (or none is pending).
    fn poll_torque(&mut self, _deadline: Instant) -> Result<TorqueProgress, ArmDriverError> {
        Err(ArmDriverError::Protocol("no torque transition pending".into()))
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

    /// Aborts any pending enable handshake without transmitting further enable frames
    /// (called when E-stop or a fault intervenes mid-enable).
    fn abort_enable(&mut self) {}

    /// One panic-disable retry step for multi-motor drivers: transmits disable to every motor
    /// not yet confirmed disabled without aborting remaining motors on a single-motor TX error,
    /// performs one budgeted RX pass within `deadline`, and returns the 0-based indices of motors
    /// still unconfirmed. Returns `None` to fall back to the generic `begin_torque_budgeted` loop.
    fn panic_disable_step(
        &mut self,
        _first: bool,
        _deadline: Instant,
    ) -> Option<Result<Vec<usize>, ArmDriverError>> {
        None
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

    /// The gripper's motor-level current cap (None: the driver has no such notion).
    fn motor_current_cap(&self) -> Option<MotorCurrentCap> { None }

    /// Allocation-free `Copy` summary of an active or configured position-force gripper cap,
    /// safe to call on `armd-control`.
    fn gripper_pos_force_info(&self) -> Option<GripperPosForceInfo> {
        let cap = self.motor_current_cap()?;
        if cap.mode != "pos_force" { return None; }
        let close_speed_rad_s = cap.close_speed_rad_s.filter(|v| v.is_finite() && *v > 0.0)?;
        let ceiling_torque_estimate_nm = cap.ceiling_torque_nm
            .or_else(|| cap.current_raw.map(|raw| f32::from(raw) * crate::model::DamiaoGripperDrive::FEEDBACK_EQUIVALENT_NM_PER_RAW))
            .filter(|v| v.is_finite() && *v > 0.0);
        Some(GripperPosForceInfo {
            joint: cap.joint,
            active: cap.active,
            close_speed_rad_s,
            ceiling_torque_estimate_nm,
        })
    }

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

    /// The arm's E-stop gate ([`crate::rt_core::urgent::EstopGate`]): a driver that sends a
    /// target batch frame by frame checks it before each frame and stops the batch while an
    /// E-stop request is not yet applied by the tick (`write_joint_targets` then fails with
    /// [`ESTOP_INHIBITED`]). Default: the driver does not check (the tick checks before its
    /// output).
    fn set_estop_gate(&mut self, _gate: crate::rt_core::urgent::EstopGate) {}

    /// Set the gripper joint index if the arm model defines a gripper
    fn set_gripper_index(&mut self, _index: Option<usize>) {}

    /// Check whether driver / hardware is actively enabled and online
    fn is_enabled(&self) -> bool {
        false
    }

    /// Return per-joint motor status or error code if supported by the driver
    fn motor_status(&self) -> Option<Vec<u8>> {
        None
    }

    /// Return per-joint load telemetry (normalized fraction in [-1.0, 1.0]) if supported
    fn joint_loads(&self) -> Option<Vec<f32>> {
        None
    }

    /// Decode motor fault/status code into descriptive string
    fn decode_motor_error(&self, code: u8) -> Option<String> {
        damiao_fault_name(code).map(str::to_string)
    }

    /// Update internal software enabled flag without transmitting physical frames
    fn set_torque_flag(&mut self, _enabled: bool) {}

    /// Update pending_rearm flag to force full hardware handshake on next enable
    fn set_pending_rearm(&mut self, _pending: bool) {}

    /// True when single joints can be switched between the driver's position mode and
    /// MIT impedance mode while torqued (`set_joint_modes`). Used by gravity compensation.
    fn supports_mit_mode_switch(&self) -> bool { false }

    /// Switch joint `i` to `modes[i]`. Each switched joint receives `hold[i]` in its new
    /// mode immediately after the switch, so it is never left without a target. A joint
    /// whose switch cannot be confirmed is switched back; joints that still cannot be
    /// confirmed are reported by `joint_modes_uncertain` and get both a position frame
    /// and an MIT frame on every write until a later switch confirms them.
    fn set_joint_modes(&mut self, _modes: &[JointMode], _hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        Err(ArmDriverError::Protocol("joint mode switching is not supported by this driver".into()))
    }

    /// [`Self::set_joint_modes`] invoked from within an active control release: shares the
    /// release's existing per-tick TX pass instead of opening a fresh pass and bypassing any
    /// budget/cutoff already consumed in that release.
    fn set_joint_modes_in_tick(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.set_joint_modes(modes, hold)
    }

    /// At least one joint is in an unconfirmed control mode after a failed switch.
    fn joint_modes_uncertain(&self) -> bool { false }

    /// Result of the startup check of the motors' mapping ranges against the
    /// configured ones; None when the driver has no such check or it has not run.
    fn range_check(&self) -> Option<DriverRangeCheck> { None }

    /// What the motor zero tool can do with this driver (`calibration_tools.motor_zero`).
    fn motor_zero_support(&self) -> MotorZeroSupport {
        MotorZeroSupport::unsupported("this driver has no motor zero command")
    }

    /// Make the current position of joint `joint` (0-based) its zero. Never moves the arm and
    /// never enables torque. Needs torque off; a driver with enable feedback also needs fresh
    /// feedback that confirms the joint disabled. Called from the motor zero command
    /// (`/api/v1/calibration/motor_zero`), never from the control tick.
    fn set_zero_position(&mut self, _joint: usize) -> Result<(), ArmDriverError> {
        Err(ArmDriverError::Protocol("motor zero is not supported by this driver".into()))
    }

    /// Joint zero offsets (motor - joint, rad) applied at the driver boundary
    /// ([`crate::joint_zero::ZeroOffsetDriver`]); a driver without the boundary ignores them.
    fn set_joint_zero_offsets(&mut self, _offsets: &[f32]) {}

    /// The offsets in effect at the boundary; None for a driver without it.
    fn joint_zero_offsets(&self) -> Option<Vec<f32>> { None }

    /// Joint indices that received an autonomous driver recovery write during
    /// `read_joint_states` since the previous call, clearing the record.
    fn take_recovery_writes(&mut self) -> Vec<usize> { Vec::new() }

    /// Strict control-mode handover (joint tracking enabled): a mode switch is confirmed
    /// only when its hold frames and the bus flush also succeed. Off (default) keeps the
    /// pre-tracking behaviour, where a matching mode acknowledgement confirms the switch.
    fn set_strict_mode_handover(&mut self, _strict: bool) {}

    /// Fresh register readback of the motors of `joints` (0-based): identity, firmware, mode,
    /// mapping ranges and the command timeout (MIT qualification gate, `src/mit.rs`). Read only.
    fn read_motor_identity(&mut self, _joints: &[usize]) -> Result<Vec<crate::mit::MotorIdentity>, ArmDriverError> {
        Err(ArmDriverError::Protocol("motor identity readback is not supported by this driver".into()))
    }

    /// MIT tracking handover (joint tracking Phase 2): `set_joint_modes` where a joint is
    /// confirmed only when its mode write, every hold frame, the flush and a matching mode
    /// acknowledgement succeed AND fresh feedback from that motor arrives after its
    /// acknowledgement (samples queued before the switch do not count). Otherwise every changed
    /// joint is switched back (dual holds) and an error is returned.
    fn set_joint_modes_confirmed(&mut self, _modes: &[JointMode], _hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        Err(ArmDriverError::Protocol("confirmed joint mode switching is not supported by this driver".into()))
    }

    /// Arm (`Some(deadline)`) or disarm the MIT tracking deadline fence. Armed: a joint feedback
    /// age or a gap between complete target batches reaching `deadline` trips it, checked before
    /// every transmit (retries and batch continuations included), and so does an RX backlog the
    /// bounded read cannot drain; no recovery write and no status poll is sent. Tripped: every
    /// target write and mode switch is refused and the cached frames are discarded until the
    /// next confirmed enable; no autonomous frame is sent for `holdoff`.
    fn set_mit_deadline_fence(&mut self, _deadline: Option<Duration>, _holdoff: Duration) {}

    /// Operator recovery after an MIT loss in which a motor disabled itself (its command TIMEOUT
    /// expired, error 0xD) while others stay enabled: clear-error to every motor in an error
    /// state, then disable every motor and require fresh per-motor confirmation (state 0). Sends
    /// no MIT and no POS_VEL target frame. Err names the joints that did not confirm.
    fn mit_expiry_recover(&mut self) -> Result<(), String> {
        Err("MIT expiry recovery is not supported by this driver".into())
    }

    /// Fresh state of every motor for the MIT expiry recovery decision: a read-only status poll
    /// to each motor after a receive boundary, and the state codes of feedback received after it.
    /// Err names the motors without fresh feedback.
    fn mit_fresh_status(&mut self) -> Result<Vec<u8>, String> {
        Err("fresh status read is not supported by this driver".into())
    }

    /// Current time of the driver's clock (the transport's deterministic clock in tests).
    fn clock_now(&self) -> Instant { Instant::now() }

    /// Optional lock-free handle to the driver's underlying clock for RT off-thread forwarding
    /// timestamps (returns `None` when the driver uses wall-clock `Instant::now()`).
    fn driver_clock(&self) -> Option<std::sync::Arc<dyn crate::transport::DriverClock>> { None }

    /// Wait `d` on the driver's clock (the transport's deterministic clock advances in tests).
    fn clock_pause(&self, d: Duration) { std::thread::sleep(d) }

    /// Why the MIT deadline fence tripped (None = not tripped).
    fn mit_fence_trip(&self) -> Option<String> { None }

    /// Explicit operator clearance of a tripped MIT fence: read-only status polls may resume
    /// (after the hold-off) so fresh feedback can return; target writes stay refused until the
    /// next confirmed enable.
    fn acknowledge_mit_fence_trip(&mut self) {}

    /// True when every joint `0..dof` has received fresh feedback with a receive sequence strictly
    /// after the most recent [`Self::mit_expiry_recover`] request.
    fn mit_feedback_after_recovery(&self, _dof: usize) -> bool { true }

    /// Write the motor command timeout of `joints` to `counts` in RAM (never flash), check the
    /// acknowledgement and confirm it with a separate read. Torque off only.
    fn commission_motor_timeout(&mut self, joints: &[usize], _counts: u32) -> Result<Vec<crate::mit::TimeoutCommissioning>, crate::mit::TimeoutBatchError> {
        Err(crate::mit::TimeoutBatchError { not_attempted: joints.to_vec(),
            ..ArmDriverError::Protocol("motor timeout commissioning is not supported by this driver".into()).into() })
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

// =========================================================================
// 1. Feetech STS3215 Arm Driver (SO-ARM 101)
// =========================================================================

pub struct FeetechArmDriver<T: Transport> {
    transport: T,
    clock: Option<Arc<dyn crate::transport::DriverClock>>,
    servo_ids: Vec<u8>,
    config: feetech::FeetechConfig,
    cached_states: Vec<JointState>,
    cached_loads: Vec<f32>,
    pub motor_status: Vec<u8>,
    last_feedback_instant: Vec<Option<Instant>>,
    feedback_seq: Vec<u64>,
    confirmed_enable_state: EnableState,
    tx_echo: Option<bool>,
    tx_echo_auto: bool,
    poll_cursor: usize,
    rx_buffer: Vec<u8>,
    pub effective_ranges: Vec<(u16, u16)>,
    cached_raw_steps: Vec<i32>,
    read_poll_count: u64,
    missed_poll_replies: u64,
    partial_enable_known: bool,
    joint_torque_enabled: Vec<Option<bool>>,
    abort_flag: Option<Arc<AtomicBool>>,
    pub last_enable_poll: Option<Instant>,
    gripper_index: Option<usize>,
    pub hw_angle_limits: Vec<Option<(u16, u16)>>,
    pub last_enable_confirmed: Vec<Option<Instant>>,
    pub missed_enable_polls: Vec<u8>,
    incomplete_chunk_idx: Option<usize>,
    incomplete_frame_started_at: Option<Instant>,
    partial_pos_rx: Vec<u8>,
    consecutive_missed_echoes: u32,
    warned_missed_echoes: bool,
}

impl<T: Transport> FeetechArmDriver<T> {
    pub fn new(transport: T, servo_ids: Vec<u8>) -> Self {
        let clock = transport.clock();
        let dof = servo_ids.len();
        let joints: Vec<feetech::FeetechJointCalib> = servo_ids
            .iter()
            .map(|&id| feetech::FeetechJointCalib {
                id,
                ..Default::default()
            })
            .collect();
        let effective_ranges = joints.iter().map(|j| j.effective_range_steps()).collect();
        Self {
            transport,
            clock,
            servo_ids,
            config: feetech::FeetechConfig {
                joints,
                ..Default::default()
            },
            cached_states: vec![JointState::default(); dof],
            cached_loads: vec![0.0; dof],
            motor_status: vec![0; dof],
            last_feedback_instant: vec![None; dof],
            feedback_seq: vec![0; dof],
            confirmed_enable_state: EnableState::Unknown,
            tx_echo: None,
            tx_echo_auto: false,
            poll_cursor: 0,
            rx_buffer: Vec::with_capacity(256),
            effective_ranges,
            cached_raw_steps: vec![0; dof],
            read_poll_count: 0,
            missed_poll_replies: 0,
            partial_enable_known: false,
            joint_torque_enabled: vec![None; dof],
            abort_flag: None,
            last_enable_poll: None,
            gripper_index: None,
            hw_angle_limits: vec![None; dof],
            last_enable_confirmed: vec![None; dof],
            missed_enable_polls: vec![0; dof],
            incomplete_chunk_idx: None,
            incomplete_frame_started_at: None,
            partial_pos_rx: Vec::with_capacity(64),
            consecutive_missed_echoes: 0,
            warned_missed_echoes: false,
        }
    }

    pub fn with_gripper_index(mut self, gripper_index: Option<usize>) -> Self {
        self.gripper_index = gripper_index;
        self
    }

    pub fn with_zero_offsets(mut self, offsets: Vec<u16>) -> Self {
        for (i, &offset) in offsets.iter().enumerate() {
            if i < self.config.joints.len() {
                self.config.joints[i].zero_step = offset;
            }
        }
        self
    }

    pub fn with_calibrated(mut self, calibrated: bool) -> Self {
        self.config.calibrated = calibrated;
        self
    }

    pub fn missed_poll_replies(&self) -> u64 {
        self.missed_poll_replies
    }

    pub fn config(&self) -> &feetech::FeetechConfig {
        &self.config
    }

    pub fn effective_ranges(&self) -> &[(u16, u16)] {
        &self.effective_ranges
    }

    pub fn with_config(mut self, cfg: feetech::FeetechConfig) -> Self {
        if cfg.tx_echo.is_some() {
            self.tx_echo = cfg.tx_echo;
            self.tx_echo_auto = false;
        }
        let old_ids = self.servo_ids.clone();
        let old_joints = self.config.joints.clone();
        self.config = cfg;
        if self.config.joints.is_empty() {
            self.config.joints = self
                .servo_ids
                .iter()
                .map(|&id| feetech::FeetechJointCalib {
                    id,
                    ..Default::default()
                })
                .collect();
        } else {
            self.servo_ids = self.config.joints.iter().map(|j| j.id).collect();
        }

        let dof = self.servo_ids.len();
        if self.servo_ids != old_ids || self.config.joints != old_joints {
            self.cached_states = vec![JointState::default(); dof];
            self.cached_loads = vec![0.0; dof];
            self.motor_status = vec![0; dof];
            self.last_feedback_instant = vec![None; dof];
            self.feedback_seq.resize(dof, 0);
            self.cached_raw_steps = vec![0; dof];
            self.last_enable_confirmed = vec![None; dof];
            self.missed_enable_polls = vec![0; dof];
            if self.servo_ids != old_ids {
                self.joint_torque_enabled = vec![None; dof];
                self.poll_cursor = 0;
                self.confirmed_enable_state = EnableState::Unknown;
                self.partial_enable_known = false;
                self.hw_angle_limits = vec![None; dof];
                self.invalidate_partial_pos_rx();
            } else if self.hw_angle_limits.len() != dof {
                self.hw_angle_limits.resize(dof, None);
            }
        }
        self.effective_ranges = self
            .config
            .joints
            .iter()
            .enumerate()
            .map(|(i, calib)| {
                let min_s = calib.range_min_steps
                    .or_else(|| self.hw_angle_limits.get(i).copied().flatten().map(|x| x.0))
                    .unwrap_or(0);
                let max_s = calib.range_max_steps
                    .or_else(|| self.hw_angle_limits.get(i).copied().flatten().map(|x| x.1))
                    .unwrap_or(4095);
                (min_s, max_s)
            })
            .collect();
        if self.effective_ranges.len() != dof {
            self.effective_ranges.resize(dof, (0, 4095));
        }
        self
    }

    pub fn with_tx_echo(mut self, tx_echo: Option<bool>) -> Self {
        self.tx_echo = tx_echo;
        self.tx_echo_auto = false;
        self
    }

    pub fn with_port_name(mut self, port: &str) -> Self {
        if port.starts_with("/dev/ttySC") && self.tx_echo.is_none() {
            self.tx_echo = Some(true);
            self.tx_echo_auto = true;
        }
        self
    }

    pub fn tx_echo(&self) -> Option<bool> {
        self.tx_echo
    }

    pub fn tx_echo_auto(&self) -> bool {
        self.tx_echo_auto
    }

    pub fn transport(&self) -> &T {
        &self.transport
    }

    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    pub fn rx_buffer(&self) -> &[u8] {
        &self.rx_buffer
    }

    #[doc(hidden)]
    pub fn confirmed_enable_state(&self) -> EnableState {
        self.confirmed_enable_state
    }

    #[doc(hidden)]
    pub fn set_confirmed_enable_state(&mut self, state: EnableState) {
        self.confirmed_enable_state = state;
    }

    #[doc(hidden)]
    pub fn set_tx_echo_state(&mut self, tx_echo: Option<bool>, auto: bool) {
        self.tx_echo = tx_echo;
        self.tx_echo_auto = auto;
    }

    #[doc(hidden)]
    pub fn read_poll_count(&self) -> u64 {
        self.read_poll_count
    }

    #[doc(hidden)]
    pub fn set_read_poll_count(&mut self, count: u64) {
        self.read_poll_count = count;
    }

    #[doc(hidden)]
    pub fn incomplete_chunk_idx(&self) -> Option<usize> {
        self.incomplete_chunk_idx
    }

    #[doc(hidden)]
    pub fn set_incomplete_chunk_idx(&mut self, idx: Option<usize>) {
        self.incomplete_chunk_idx = idx;
    }

    #[doc(hidden)]
    pub fn incomplete_frame_started_at(&self) -> Option<Instant> {
        self.incomplete_frame_started_at
    }

    #[doc(hidden)]
    pub fn set_incomplete_frame_started_at(&mut self, instant: Option<Instant>) {
        self.incomplete_frame_started_at = instant;
    }

    #[doc(hidden)]
    pub fn partial_pos_rx(&self) -> &[u8] {
        &self.partial_pos_rx
    }

    #[doc(hidden)]
    pub fn set_partial_pos_rx(&mut self, bytes: &[u8]) {
        self.partial_pos_rx = bytes.to_vec();
    }

    pub(crate) fn invalidate_partial_pos_rx(&mut self) {
        self.partial_pos_rx.clear();
        self.incomplete_chunk_idx = None;
        self.incomplete_frame_started_at = None;
    }

    fn is_aborted(&self) -> bool {
        self.abort_flag
            .as_ref()
            .is_some_and(|f| f.load(Ordering::SeqCst))
    }

    pub fn clock_now(&self) -> Instant {
        self.clock.as_ref().map_or_else(Instant::now, |c| c.now())
    }

    fn elapsed_since(&self, earlier: Instant) -> Duration {
        self.clock_now().saturating_duration_since(earlier)
    }

    fn clock_sleep(&self, duration: Duration) {
        if let Some(ref c) = self.clock {
            c.sleep(duration);
        } else {
            std::thread::sleep(duration);
        }
    }

    fn clock_yield(&self) {
        if let Some(ref c) = self.clock {
            c.yield_now();
        } else {
            std::thread::yield_now();
        }
    }

    fn assert_clock_matches_transport(&self) {
        let captured_id = self.clock.as_ref().map(|c| c.clock_id());
        let transport_id = self.transport.clock().map(|c| c.clock_id());
        assert_eq!(
            captured_id, transport_id,
            "FeetechArmDriver transport clock domain changed after construction (cannot replace transport with a different clock)"
        );
    }

    fn drain_tx_echo(&mut self, tx: &[u8]) -> Result<bool, ArmDriverError> {
        self.assert_clock_matches_transport();
        if tx.is_empty() || self.tx_echo == Some(false) {
            return Ok(false);
        }

        let mut temp = [0u8; 1];
        let start = self.clock_now();
        const DRAIN_TIMEOUT: Duration = Duration::from_millis(5);

        let resync_rx = |rx: &mut Vec<u8>| {
            let mut i = 0;
            while i < rx.len() {
                if rx[i] != 0xFF {
                    let next = rx[i..].iter().position(|&b| b == 0xFF).map(|p| i + p).unwrap_or(rx.len());
                    i = next;
                    if i >= rx.len() {
                        break;
                    }
                }
                if tx.starts_with(&[0xFF, 0xFF])
                    && tx.get(2) != Some(&0xFF)
                    && rx[i..].len() >= 3
                    && rx[i..i + 3] == [0xFF, 0xFF, 0xFF]
                {
                    i += 1;
                    continue;
                }
                let slice = &rx[i..];
                let is_echo = slice.windows(tx.len()).any(|w| w == tx) || tx.starts_with(slice);
                if !is_echo {
                    match feetech::parse_response(slice) {
                        Err(feetech::FeetechError::IncompleteFrame { .. }) | Ok(_) => break,
                        Err(_) => {
                            i += 1;
                            continue;
                        }
                    }
                }
                break;
            }
            if i > 0 {
                rx.drain(..i.min(rx.len()));
            }
        };

        let auto_detecting = self.tx_echo.is_none() || (self.tx_echo == Some(true) && self.tx_echo_auto);

        while !self.rx_buffer.windows(tx.len()).any(|w| w == tx)
            && self.elapsed_since(start) < DRAIN_TIMEOUT
            && self.rx_buffer.len() < tx.len() + 64
        {
            match self.transport.read(&mut temp) {
                Ok(n) if n > 0 => {
                    self.rx_buffer.extend_from_slice(&temp[..n]);
                }
                _ => {
                    self.clock_yield();
                }
            }

            resync_rx(&mut self.rx_buffer);

            if let Ok((pkt, consumed)) = feetech::parse_response(&self.rx_buffer) {
                if &self.rx_buffer[..consumed] != tx
                    && pkt.id != feetech::BROADCAST_ID
                    && !self.rx_buffer.windows(tx.len()).any(|w| w == tx)
                {
                    if auto_detecting {
                        tracing::warn!("Auto-detected tx_echo on /dev/ttySC* received valid non-echo response; disabling tx_echo");
                        self.tx_echo = Some(false);
                        self.tx_echo_auto = false;
                    }
                    break;
                }
            }
        }

        resync_rx(&mut self.rx_buffer);

        let mut drained = false;
        if let Some(pos) = self.rx_buffer.windows(tx.len()).position(|w| w == tx) {
            self.rx_buffer.drain(..pos + tx.len());
            drained = true;
            if self.tx_echo.is_none() || self.tx_echo_auto {
                self.tx_echo = Some(true);
                self.tx_echo_auto = false;
            }
        } else if auto_detecting {
            let has_partial_echo = (2..tx.len()).any(|k| self.rx_buffer.ends_with(&tx[..k]))
                || (self.rx_buffer.len() < 2 && tx.starts_with(&self.rx_buffer));
            let has_valid_non_echo = matches!(feetech::parse_response(&self.rx_buffer), Ok((pkt, c)) if &self.rx_buffer[..c] != tx && pkt.id != feetech::BROADCAST_ID);
            if self.tx_echo == Some(true) && self.tx_echo_auto && !has_partial_echo {
                tracing::warn!("Auto-detected tx_echo=true on /dev/ttySC* but no echo received within timeout; disabling tx_echo");
                self.tx_echo = Some(false);
                self.tx_echo_auto = false;
            } else if self.tx_echo.is_none() && has_valid_non_echo {
                self.tx_echo = Some(false);
            }
        }

        if self.tx_echo == Some(true) && !self.tx_echo_auto {
            if drained {
                self.consecutive_missed_echoes = 0;
            } else {
                self.consecutive_missed_echoes = self.consecutive_missed_echoes.saturating_add(1);
                if self.consecutive_missed_echoes >= 10 && !self.warned_missed_echoes {
                    tracing::warn!(
                        "Feetech driver has missed {} consecutive TX echoes with explicit tx_echo=true",
                        self.consecutive_missed_echoes
                    );
                    self.warned_missed_echoes = true;
                }
            }
        }

        Ok(drained)
    }

    /// Reads REG_TORQUE_ENABLE from each servo in `ids` (indexed like `servo_ids`), records
    /// the per-joint state, and returns (disabled count, enabled count, read errors).
    fn read_torque_enable_states(&mut self, ids: &[u8]) -> (usize, usize, Vec<String>) {
        let mut count_0 = 0;
        let mut count_1 = 0;
        let mut read_errors = Vec::new();
        for (idx, &id) in ids.iter().enumerate() {
            match self.read_register_u8_direct(id, feetech::REG_TORQUE_ENABLE) {
                Ok(v @ (0 | 1)) => {
                    if v == 0 { count_0 += 1; } else { count_1 += 1; }
                    self.joint_torque_enabled[idx] = Some(v == 1);
                    self.last_enable_confirmed[idx] = Some(self.clock_now());
                    self.missed_enable_polls[idx] = 0;
                }
                Ok(v) => {
                    self.joint_torque_enabled[idx] = None;
                    read_errors.push(format!("servo {} returned unexpected value {}", id, v));
                }
                Err(e) => {
                    self.joint_torque_enabled[idx] = None;
                    read_errors.push(format!("servo {} read failed: {}", id, e));
                }
            }
        }
        (count_0, count_1, read_errors)
    }

    pub fn read_register_u8_direct(&mut self, id: u8, reg: u8) -> Result<u8, ArmDriverError> {
        self.read_register_direct(id, reg, 1).map(|p| p[0])
    }

    pub fn read_register_u16_direct(&mut self, id: u8, reg: u8) -> Result<u16, ArmDriverError> {
        self.read_register_direct(id, reg, 2).map(|p| u16::from_le_bytes([p[0], p[1]]))
    }

    fn read_register_direct(&mut self, id: u8, reg: u8, len: u8) -> Result<Vec<u8>, ArmDriverError> {
        self.assert_clock_matches_transport();
        self.invalidate_partial_pos_rx();
        self.transport.clear_input_buffer();
        self.rx_buffer.clear();
        let req = feetech::build_read(id, reg, len);
        self.transport.write_all(&req)?;
        self.transport.flush()?;
        let echo_drained = self.drain_tx_echo(&req)?;

        let mut temp = [0u8; 1];
        let start = self.clock_now();
        const READ_TIMEOUT: Duration = Duration::from_millis(25);
        let mut can_skip_echo = !echo_drained && self.tx_echo != Some(false);

        loop {
            while !self.rx_buffer.is_empty() {
                match feetech::parse_response(&self.rx_buffer) {
                    Ok((pkt, consumed)) => {
                        let is_req_echo = can_skip_echo && consumed >= req.len() && self.rx_buffer[consumed - req.len()..consumed] == req[..];
                        self.rx_buffer.drain(..consumed);
                        if is_req_echo {
                            can_skip_echo = false;
                            self.tx_echo = Some(true);
                            self.tx_echo_auto = false;
                            continue;
                        }
                        if pkt.id == feetech::BROADCAST_ID {
                            continue;
                        }
                        if pkt.id == id && pkt.params.len() == len as usize {
                            if self.tx_echo.is_none() {
                                self.tx_echo = Some(false);
                            }
                            return Ok(pkt.params);
                        }
                    }
                    Err(feetech::FeetechError::IncompleteFrame { .. }) => {
                        break;
                    }
                    Err(_) => {
                        self.rx_buffer.remove(0);
                    }
                }
            }

            if self.elapsed_since(start) >= READ_TIMEOUT {
                break;
            }

            match self.transport.read(&mut temp) {
                Ok(n) if n > 0 => {
                    self.rx_buffer.extend_from_slice(&temp[..n]);
                }
                _ => {
                    self.clock_yield();
                }
            }
        }

        Err(ArmDriverError::Timeout(id))
    }

    fn write_register_u8_direct(&mut self, id: u8, reg: u8, val: u8) -> Result<(), ArmDriverError> {
        self.assert_clock_matches_transport();
        if self.is_aborted() {
            return Err(ArmDriverError::HardwareFault("Aborted by emergency stop".into()));
        }
        self.write_register_u8_cleanup(id, reg, val)
    }

    fn write_register_u8_cleanup(&mut self, id: u8, reg: u8, val: u8) -> Result<(), ArmDriverError> {
        self.assert_clock_matches_transport();
        self.invalidate_partial_pos_rx();
        self.transport.clear_input_buffer();
        self.rx_buffer.clear();
        let pkt = feetech::build_write_u8(id, reg, val);
        self.transport.write_all(&pkt)?;
        self.transport.flush()?;
        self.drain_tx_echo(&pkt)?;
        Ok(())
    }

    fn write_register_u16_direct(&mut self, id: u8, reg: u8, val: u16) -> Result<(), ArmDriverError> {
        self.assert_clock_matches_transport();
        if self.is_aborted() {
            return Err(ArmDriverError::HardwareFault("Aborted by emergency stop".into()));
        }
        self.invalidate_partial_pos_rx();
        self.transport.clear_input_buffer();
        self.rx_buffer.clear();
        let pkt = feetech::build_write_u16(id, reg, val);
        self.transport.write_all(&pkt)?;
        self.transport.flush()?;
        self.drain_tx_echo(&pkt)?;
        Ok(())
    }

    /// Configure EEPROM and RAM registers for all servos based on `FeetechConfig`.
    pub fn configure_servos(&mut self) -> Result<(), ArmDriverError> {
        self.assert_clock_matches_transport();
        self.invalidate_partial_pos_rx();
        if self.is_aborted() {
            return Err(ArmDriverError::HardwareFault("Aborted by emergency stop".into()));
        }
        let mut all_disabled = true;
        let mut all_enabled = true;

        for joint_idx in 0..self.servo_ids.len() {
            let id = self.servo_ids[joint_idx];
            let calib = self.config.joints.get(joint_idx).cloned().unwrap_or(feetech::FeetechJointCalib {
                id,
                ..Default::default()
            });

            // 1. Verify model
            let model = self.read_register_u16_direct(id, feetech::REG_MODEL_L)?;
            if model != feetech::EXPECTED_MODEL_STS3215 {
                return Err(ArmDriverError::Protocol(format!(
                    "Servo ID {} reported model {}, expected STS3215 ({})",
                    id, model, feetech::EXPECTED_MODEL_STS3215
                )));
            }

            // 2. Read torque enable status
            let torque_en = self.read_register_u8_direct(id, feetech::REG_TORQUE_ENABLE)?;
            match torque_en {
                0 => {
                    if joint_idx < self.joint_torque_enabled.len() {
                        self.joint_torque_enabled[joint_idx] = Some(false);
                        self.last_enable_confirmed[joint_idx] = Some(self.clock_now());
                        self.missed_enable_polls[joint_idx] = 0;
                    }
                    all_enabled = false;
                }
                1 => {
                    if joint_idx < self.joint_torque_enabled.len() {
                        self.joint_torque_enabled[joint_idx] = Some(true);
                        self.last_enable_confirmed[joint_idx] = Some(self.clock_now());
                        self.missed_enable_polls[joint_idx] = 0;
                    }
                    all_disabled = false;
                }
                _ => {
                    if joint_idx < self.joint_torque_enabled.len() {
                        self.joint_torque_enabled[joint_idx] = None;
                    }
                    all_enabled = false;
                    all_disabled = false;
                }
            }

            // 3. Read current registers
            let cur_return_delay = self.read_register_u8_direct(id, feetech::REG_RETURN_DELAY)?;
            let cur_status_level = self.read_register_u8_direct(id, feetech::REG_STATUS_RETURN_LEVEL)?;
            let cur_min_angle = self.read_register_u16_direct(id, feetech::REG_MIN_ANGLE_LIMIT_L)?;
            let cur_max_angle = self.read_register_u16_direct(id, feetech::REG_MAX_ANGLE_LIMIT_L)?;
            let cur_p_coef = self.read_register_u8_direct(id, feetech::REG_P_COEF)?;
            let cur_d_coef = self.read_register_u8_direct(id, feetech::REG_D_COEF)?;
            let cur_i_coef = self.read_register_u8_direct(id, feetech::REG_I_COEF)?;
            let cur_max_torque = self.read_register_u16_direct(id, feetech::REG_MAX_TORQUE_L)?;
            let cur_prot_current = self.read_register_u16_direct(id, feetech::REG_PROTECTION_CURRENT_L)?;
            let cur_overload_torque = self.read_register_u8_direct(id, feetech::REG_OVERLOAD_TORQUE)?;
            let cur_operating_mode = self.read_register_u8_direct(id, feetech::REG_OPERATING_MODE)?;

            // Effective angle limits validation and storage
            if joint_idx < self.hw_angle_limits.len() {
                self.hw_angle_limits[joint_idx] = Some((cur_min_angle, cur_max_angle));
            }
            let eff_min = calib.range_min_steps.unwrap_or(cur_min_angle);
            let eff_max = calib.range_max_steps.unwrap_or(cur_max_angle);
            if eff_min >= eff_max || eff_max > 4095 {
                return Err(ArmDriverError::HardwareFault(format!(
                    "Servo ID {} has invalid angle limits min={} max={}",
                    id, eff_min, eff_max
                )));
            }
            if joint_idx < self.effective_ranges.len() {
                self.effective_ranges[joint_idx] = (eff_min, eff_max);
            }

            // Expected homing offset verification
            if let Some(expected) = calib.expected_homing_offset_steps {
                let raw_offset = self.read_register_u16_direct(id, feetech::REG_HOMING_OFFSET_L)?;
                let decoded = feetech::decode_sign_magnitude(raw_offset, 0x0800);
                if decoded != expected {
                    return Err(ArmDriverError::HardwareFault(format!(
                        "Servo ID {} homing offset mismatch: expected {}, read {}",
                        id, expected, decoded
                    )));
                }
            }

            // Targets
            let target_return_delay = self.config.return_delay_raw;
            let target_status_level = 0u8;
            let target_p_coef = self.config.p_coef;
            let target_d_coef = self.config.d_coef;
            let target_i_coef = self.config.i_coef;
            let target_max_torque = calib.max_torque_raw.unwrap_or(self.config.max_torque_raw);
            let target_prot_current = calib.protection_current_raw.unwrap_or(self.config.protection_current_raw);
            let target_overload_torque = calib.overload_torque_raw.or(self.config.overload_torque_raw);
            let target_operating_mode = 0u8;

            let differs = cur_return_delay != target_return_delay
                || cur_status_level != target_status_level
                || calib.range_min_steps.is_some_and(|m| cur_min_angle != m)
                || calib.range_max_steps.is_some_and(|m| cur_max_angle != m)
                || cur_p_coef != target_p_coef
                || cur_d_coef != target_d_coef
                || cur_i_coef != target_i_coef
                || cur_max_torque != target_max_torque
                || cur_prot_current != target_prot_current
                || target_overload_torque.is_some_and(|ot| cur_overload_torque != ot)
                || cur_operating_mode != target_operating_mode;

            if differs {
                if torque_en != 0 {
                    return Err(ArmDriverError::HardwareFault(
                        "cannot configure EEPROM while torque is enabled".to_string(),
                    ));
                }

                let mut changed_registers: Vec<(u8, u8, u16)> = Vec::new();

                let write_res = (|| -> Result<(), ArmDriverError> {
                    // Unlock EEPROM inside closure so REG_LOCK=1 always runs
                    self.write_register_u8_direct(id, feetech::REG_LOCK, 0)?;
                    self.clock_sleep(Duration::from_millis(2));

                    if cur_return_delay != target_return_delay {
                        self.write_register_u8_direct(id, feetech::REG_RETURN_DELAY, target_return_delay)?;
                        self.clock_sleep(Duration::from_millis(2));
                        changed_registers.push((feetech::REG_RETURN_DELAY, 1, target_return_delay as u16));
                    }
                    if cur_status_level != target_status_level {
                        self.write_register_u8_direct(id, feetech::REG_STATUS_RETURN_LEVEL, target_status_level)?;
                        self.clock_sleep(Duration::from_millis(2));
                        changed_registers.push((feetech::REG_STATUS_RETURN_LEVEL, 1, target_status_level as u16));
                    }
                    if let Some(target_min) = calib.range_min_steps {
                        if cur_min_angle != target_min {
                            self.write_register_u16_direct(id, feetech::REG_MIN_ANGLE_LIMIT_L, target_min)?;
                            self.clock_sleep(Duration::from_millis(2));
                            changed_registers.push((feetech::REG_MIN_ANGLE_LIMIT_L, 2, target_min));
                        }
                    }
                    if let Some(target_max) = calib.range_max_steps {
                        if cur_max_angle != target_max {
                            self.write_register_u16_direct(id, feetech::REG_MAX_ANGLE_LIMIT_L, target_max)?;
                            self.clock_sleep(Duration::from_millis(2));
                            changed_registers.push((feetech::REG_MAX_ANGLE_LIMIT_L, 2, target_max));
                        }
                    }
                    if cur_p_coef != target_p_coef {
                        self.write_register_u8_direct(id, feetech::REG_P_COEF, target_p_coef)?;
                        self.clock_sleep(Duration::from_millis(2));
                        changed_registers.push((feetech::REG_P_COEF, 1, target_p_coef as u16));
                    }
                    if cur_d_coef != target_d_coef {
                        self.write_register_u8_direct(id, feetech::REG_D_COEF, target_d_coef)?;
                        self.clock_sleep(Duration::from_millis(2));
                        changed_registers.push((feetech::REG_D_COEF, 1, target_d_coef as u16));
                    }
                    if cur_i_coef != target_i_coef {
                        self.write_register_u8_direct(id, feetech::REG_I_COEF, target_i_coef)?;
                        self.clock_sleep(Duration::from_millis(2));
                        changed_registers.push((feetech::REG_I_COEF, 1, target_i_coef as u16));
                    }
                    if cur_max_torque != target_max_torque {
                        self.write_register_u16_direct(id, feetech::REG_MAX_TORQUE_L, target_max_torque)?;
                        self.clock_sleep(Duration::from_millis(2));
                        changed_registers.push((feetech::REG_MAX_TORQUE_L, 2, target_max_torque));
                    }
                    if cur_prot_current != target_prot_current {
                        self.write_register_u16_direct(id, feetech::REG_PROTECTION_CURRENT_L, target_prot_current)?;
                        self.clock_sleep(Duration::from_millis(2));
                        changed_registers.push((feetech::REG_PROTECTION_CURRENT_L, 2, target_prot_current));
                    }
                    if let Some(target_overload) = target_overload_torque {
                        if cur_overload_torque != target_overload {
                            self.write_register_u8_direct(id, feetech::REG_OVERLOAD_TORQUE, target_overload)?;
                            self.clock_sleep(Duration::from_millis(2));
                            changed_registers.push((feetech::REG_OVERLOAD_TORQUE, 1, target_overload as u16));
                        }
                    }
                    if cur_operating_mode != target_operating_mode {
                        self.write_register_u8_direct(id, feetech::REG_OPERATING_MODE, target_operating_mode)?;
                        self.clock_sleep(Duration::from_millis(2));
                        changed_registers.push((feetech::REG_OPERATING_MODE, 1, target_operating_mode as u16));
                    }

                    // Verify readback of all changed registers before lock
                    for &(reg, size, expected_val) in &changed_registers {
                        if size == 1 {
                            let read_val = self.read_register_u8_direct(id, reg)?;
                            if (read_val as u16) != expected_val {
                                return Err(ArmDriverError::HardwareFault(format!(
                                    "EEPROM readback verify failed for servo ID {} reg 0x{:02X}: expected {}, got {}",
                                    id, reg, expected_val, read_val
                                )));
                            }
                        } else {
                            let read_val = self.read_register_u16_direct(id, reg)?;
                            if read_val != expected_val {
                                return Err(ArmDriverError::HardwareFault(format!(
                                    "EEPROM readback verify failed for servo ID {} reg 0x{:02X}: expected {}, got {}",
                                    id, reg, expected_val, read_val
                                )));
                            }
                        }
                    }

                    Ok(())
                })();

                // ALWAYS lock EEPROM before returning
                let lock_res = self.write_register_u8_cleanup(id, feetech::REG_LOCK, 1);
                if let Err(e) = write_res {
                    if let Err(le) = lock_res {
                        tracing::error!("Failed to relock EEPROM for servo {id} during error recovery: {le}");
                    }
                    return Err(e);
                }
                lock_res?;
                if cur_max_torque != target_max_torque {
                    self.write_register_u16_direct(id, feetech::REG_TORQUE_LIMIT_L, target_max_torque)?;
                }
            }
        }

        if !self.servo_ids.is_empty() {
            if all_disabled {
                self.confirmed_enable_state = EnableState::Disabled;
                self.partial_enable_known = false;
            } else if all_enabled {
                self.confirmed_enable_state = EnableState::Enabled;
                self.partial_enable_known = false;
            } else {
                self.confirmed_enable_state = EnableState::Unknown;
                self.partial_enable_known = true;
            }
        }

        Ok(())
    }
    fn is_benign_gripper_overload(&self, joint_idx: usize, status: u8) -> bool {
        self.gripper_index == Some(joint_idx) && status == 0x20
    }

    fn decode_position_frames_from(
        &mut self,
        rx: &mut Vec<u8>,
        expected_ids: &std::collections::HashSet<u8>,
        replied_ids: &mut std::collections::HashSet<u8>,
        read_time: Instant,
    ) {
        let default_calib = feetech::FeetechJointCalib::default();
        while !rx.is_empty() {
            match feetech::parse_response(rx) {
                Ok((pkt, consumed)) => {
                    rx.drain(..consumed);
                    if pkt.id == feetech::BROADCAST_ID {
                        continue;
                    }
                    if expected_ids.contains(&pkt.id) && !replied_ids.contains(&pkt.id) {
                        if let Some(joint_idx) = self.servo_ids.iter().position(|&id| id == pkt.id) {
                            if pkt.params.len() == 8 {
                                let pos_raw = u16::from_le_bytes([pkt.params[0], pkt.params[1]]);
                                let speed_raw = u16::from_le_bytes([pkt.params[2], pkt.params[3]]);
                                let load_raw = u16::from_le_bytes([pkt.params[4], pkt.params[5]]);
                                let temp_raw = pkt.params[7];

                                let calib = self.config.joints.get(joint_idx).unwrap_or(&default_calib);
                                let raw_vel = feetech::raw_speed_to_rad_s(speed_raw);
                                let velocity = if calib.drive_mode_invert { -raw_vel } else { raw_vel };
                                let raw_load = feetech::raw_load_to_fraction(load_raw);
                                let load = if calib.drive_mode_invert { -raw_load } else { raw_load };
                                let pos_steps = feetech::decode_present_position(pos_raw);
                                self.cached_raw_steps[joint_idx] = pos_steps;
                                self.cached_states[joint_idx] = JointState {
                                    position: calib.steps_to_rad(pos_steps),
                                    velocity,
                                    torque: 0.0,
                                    temperature: feetech::raw_to_temperature(temp_raw),
                                };
                                self.cached_loads[joint_idx] = load;
                                self.motor_status[joint_idx] = pkt.error;
                                self.last_feedback_instant[joint_idx] = Some(read_time);
                                self.feedback_seq[joint_idx] = self.feedback_seq[joint_idx].wrapping_add(1);

                                replied_ids.insert(pkt.id);
                            }
                        }
                    }
                }
                Err(feetech::FeetechError::IncompleteFrame { .. }) => {
                    break;
                }
                Err(_) => {
                    rx.remove(0);
                }
            }
        }
    }

    fn decode_position_frames(
        &mut self,
        expected_ids: &std::collections::HashSet<u8>,
        replied_ids: &mut std::collections::HashSet<u8>,
        read_time: Instant,
    ) {
        let mut rx = std::mem::take(&mut self.rx_buffer);
        self.decode_position_frames_from(&mut rx, expected_ids, replied_ids, read_time);
        self.rx_buffer = rx;
    }

    fn decode_partial_position_frames(
        &mut self,
        expected_ids: &std::collections::HashSet<u8>,
        replied_ids: &mut std::collections::HashSet<u8>,
        read_time: Instant,
    ) {
        let mut rx = std::mem::take(&mut self.partial_pos_rx);
        self.decode_position_frames_from(&mut rx, expected_ids, replied_ids, read_time);
        self.partial_pos_rx = rx;
    }
}

impl<T: Transport> ArmDriver for FeetechArmDriver<T> {
    fn motor_zero_support(&self) -> MotorZeroSupport {
        MotorZeroSupport::unsupported("Feetech: the zero is the profile's feetech.joints calibration (zero_step, \
            homing_offset_steps), written by the SO-ARM calibration, not a motor command in armd")
    }

    fn position_limits(&self, joint: usize) -> Option<(f32, f32)> {
        self.effective_ranges.get(joint).map(|&(min_s, max_s)| {
            let calib = self.config.joint_calib(joint);
            let r0 = calib.steps_to_rad(min_s as i32);
            let r1 = calib.steps_to_rad(max_s as i32);
            (r0.min(r1), r0.max(r1))
        })
    }

    fn set_gripper_index(&mut self, index: Option<usize>) {
        self.assert_clock_matches_transport();
        self.gripper_index = index;
    }

    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            velocity_feedback: true,
            torque_feedback: false,
            temperature_feedback: true,
            torque_feedforward: false,
            enable_feedback: true,
            load_feedback: true,
        }
    }

    fn enable_state(&self) -> EnableState {
        self.confirmed_enable_state
    }

    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        self.last_feedback_instant
            .iter()
            .map(|t| t.map(|t| self.elapsed_since(t)))
            .collect()
    }

    fn feedback_instants(&self) -> Vec<Option<Instant>> {
        self.last_feedback_instant.clone()
    }

    fn feedback_seqs(&self) -> Vec<u64> {
        self.feedback_seq.clone()
    }

    fn feedback_receive_instants(&self) -> Vec<Option<Instant>> {
        self.last_feedback_instant.clone()
    }

    fn is_feedback_valid(&self) -> bool {
        if self.last_feedback_instant.is_empty() {
            return false;
        }
        const FRESHNESS_LIMIT: Duration = Duration::from_millis(250);
        self.last_feedback_instant
            .iter()
            .all(|opt| opt.is_some_and(|instant| self.elapsed_since(instant) <= FRESHNESS_LIMIT))
    }

    fn is_enabled(&self) -> bool {
        self.confirmed_enable_state == EnableState::Enabled && self.is_feedback_valid()
    }

    fn read_joint_states_budgeted(
        &mut self,
        _out: &mut Vec<JointState>,
        _deadline: Instant,
        _max_rx_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        Err(ArmDriverError::Protocol("UNSUPPORTED: serial driver does not support budgeted RX".into()))
    }

    fn begin_torque_budgeted(&mut self, _enabled: bool, _max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
        Err(ArmDriverError::Protocol("UNSUPPORTED: serial driver does not support budgeted torque transitions".into()))
    }

    fn poll_torque_budgeted(&mut self, _deadline: Instant, _max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
        Err(ArmDriverError::Protocol("UNSUPPORTED: serial driver does not support budgeted torque transitions".into()))
    }

    fn motor_status(&self) -> Option<Vec<u8>> {
        Some(self.motor_status.clone())
    }

    fn joint_loads(&self) -> Option<Vec<f32>> {
        Some(self.cached_loads.clone())
    }

    fn decode_motor_error(&self, code: u8) -> Option<String> {
        feetech::feetech_error_name(code)
    }

    fn check_hardware_status(&self) -> Result<(), ArmDriverError> {
        for (i, &status) in self.motor_status.iter().enumerate() {
            if status != 0 {
                if self.is_benign_gripper_overload(i, status) {
                    continue;
                }
                let id = self.servo_ids.get(i).copied().unwrap_or(0);
                let desc = feetech::feetech_error_name(status)
                    .unwrap_or_else(|| format!("0x{:02X}", status));
                return Err(ArmDriverError::HardwareFault(format!(
                    "Feetech joint {} (ID {}) fault 0x{:02X}: {}",
                    i, id, status, desc
                )));
            }
        }
        Ok(())
    }

    fn clear_fault_disables_joints(&self) -> Vec<usize> {
        self.motor_status
            .iter()
            .enumerate()
            .filter(|(i, &s)| s != 0 && !self.is_benign_gripper_overload(*i, s))
            .map(|(i, _)| i)
            .collect()
    }

    fn clear_fault_disables_arm_joint(&self, arm_dof: usize) -> bool {
        self.motor_status
            .iter()
            .enumerate()
            .any(|(i, &s)| i < arm_dof && s != 0 && !self.is_benign_gripper_overload(i, s))
    }

    fn has_hardware_fault(&self) -> bool {
        self.motor_status
            .iter()
            .enumerate()
            .any(|(i, &s)| s != 0 && !self.is_benign_gripper_overload(i, s))
    }

    fn has_known_partial_enable(&self) -> bool {
        self.partial_enable_known
    }

    fn has_any_joint_enabled(&self) -> bool {
        self.confirmed_enable_state == EnableState::Enabled
            || self.partial_enable_known
            || self.joint_torque_enabled.contains(&Some(true))
    }

    fn set_abort_flag(&mut self, flag: Arc<AtomicBool>) {
        self.assert_clock_matches_transport();
        self.abort_flag = Some(flag);
    }

    fn refresh_enable_state(&mut self) -> Result<EnableState, ArmDriverError> {
        self.assert_clock_matches_transport();
        if self.servo_ids.is_empty() {
            return Ok(self.confirmed_enable_state);
        }
        let ids = self.servo_ids.clone();
        let (count_0, count_1, read_errors) = self.read_torque_enable_states(&ids);
        let read_all_valid = read_errors.is_empty();

        if read_all_valid && count_0 == ids.len() {
            self.confirmed_enable_state = EnableState::Disabled;
            self.partial_enable_known = false;
        } else if read_all_valid && count_1 == ids.len() {
            self.confirmed_enable_state = EnableState::Enabled;
            self.partial_enable_known = false;
        } else if read_all_valid && count_0 > 0 && count_1 > 0 && count_0 + count_1 == ids.len() {
            self.confirmed_enable_state = EnableState::Unknown;
            self.partial_enable_known = true;
        } else {
            self.confirmed_enable_state = EnableState::Unknown;
            self.partial_enable_known = false;
        }

        Ok(self.confirmed_enable_state)
    }

    fn clear_fault(&mut self) -> Result<(), ArmDriverError> {
        self.assert_clock_matches_transport();
        let faulted_ids: Vec<u8> = self
            .motor_status
            .iter()
            .enumerate()
            .filter(|(i, &s)| s != 0 && !self.is_benign_gripper_overload(*i, s))
            .filter_map(|(i, _)| self.servo_ids.get(i).copied())
            .collect();

        let mut write_err = None;
        for &id in &faulted_ids {
            let pkt = feetech::build_set_torque(id, false);
            if let Err(e) = self.transport.write_all(&pkt) {
                write_err = Some(e.into());
            }
            let _ = self.transport.flush();
            let _ = self.drain_tx_echo(&pkt);
        }

        if let Some(e) = write_err {
            self.confirmed_enable_state = EnableState::Unknown;
            self.partial_enable_known = false;
            return Err(e);
        }

        let ids = self.servo_ids.clone();
        let (count_0, count_1, read_errors) = self.read_torque_enable_states(&ids);

        if !read_errors.is_empty() {
            self.confirmed_enable_state = EnableState::Unknown;
            self.partial_enable_known = false;
            return Err(ArmDriverError::HardwareFault(format!(
                "clear_fault readback error: {}",
                read_errors.join("; ")
            )));
        }

        if count_0 == self.servo_ids.len() {
            self.confirmed_enable_state = EnableState::Disabled;
            self.partial_enable_known = false;
        } else if count_1 == self.servo_ids.len() {
            self.confirmed_enable_state = EnableState::Enabled;
            self.partial_enable_known = false;
        } else if count_0 + count_1 == self.servo_ids.len() {
            self.confirmed_enable_state = EnableState::Unknown;
            self.partial_enable_known = true;
        } else {
            self.confirmed_enable_state = EnableState::Unknown;
            self.partial_enable_known = false;
        }

        Ok(())
    }

    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        self.assert_clock_matches_transport();
        self.invalidate_partial_pos_rx();
        let ids = self.servo_ids.clone();
        if enabled {
            self.check_hardware_status()?;
            let prev_joint_enabled = self.joint_torque_enabled.clone();
            if self.is_aborted() {
                return Err(ArmDriverError::HardwareFault("Aborted by emergency stop".into()));
            }
            if !self.config.calibrated {
                return Err(ArmDriverError::HardwareFault(
                    "Feetech arm is uncalibrated: calibrate servos and set feetech.calibrated=true".into(),
                ));
            }
            if !self.is_feedback_valid() {
                if self.last_feedback_instant.iter().any(Option::is_some) {
                    return Err(ArmDriverError::HardwareFault(
                        "Stale joint feedback (>250ms); refusing to enable torque with unverified/stale goal positions".into(),
                    ));
                }
                return Err(ArmDriverError::HardwareFault(
                    "No valid joint feedback received (<250ms); refusing to enable torque with unverified/stale goal positions".into(),
                ));
            }
            let default_calib = feetech::FeetechJointCalib::default();
            for (i, &raw) in self.cached_raw_steps.iter().enumerate() {
                let calib = self.config.joints.get(i).unwrap_or(&default_calib);
                let range = self.effective_ranges.get(i).copied().unwrap_or_else(|| calib.effective_range_steps());
                let pos_rad = calib.steps_to_rad(raw);
                calib.checked_rad_to_steps_in_range(pos_rad, range).map_err(|e| {
                    ArmDriverError::HardwareFault(format!(
                        "Joint {} (ID {}) raw position {} outside calibrated range: {}",
                        i, self.servo_ids[i], raw, e
                    ))
                })?;
            }

            // Write targets only for joints that are not already holding (Some(true))
            let mut positions_to_write = Vec::new();
            for (i, s) in self.cached_states.iter().enumerate() {
                if prev_joint_enabled.get(i).copied().flatten() == Some(true) {
                    continue;
                }
                let calib = self.config.joints.get(i).unwrap_or(&default_calib);
                let range = self.effective_ranges.get(i).copied().unwrap_or_else(|| calib.effective_range_steps());
                let raw_steps = calib.checked_rad_to_steps_in_range(s.position, range)
                    .map_err(|e| ArmDriverError::Protocol(e.to_string()))?;
                positions_to_write.push((self.servo_ids[i], raw_steps));
            }
            if !positions_to_write.is_empty() {
                let sync_pkt = feetech::build_sync_write_positions(&positions_to_write)
                    .map_err(|e| ArmDriverError::Protocol(e.to_string()))?;
                self.transport.write_all(&sync_pkt)?;
                self.transport.flush()?;
                self.drain_tx_echo(&sync_pkt)?;
            }

            let mut write_err = None;
            let mut written_indices = Vec::new();
            for (idx, &id) in ids.iter().enumerate() {
                if self.is_aborted() {
                    write_err = Some(ArmDriverError::HardwareFault("Aborted by emergency stop".into()));
                    break;
                }
                let pkt = feetech::build_set_torque(id, true);
                if let Err(e) = self.transport.write_all(&pkt) {
                    write_err = Some(e.into());
                    break;
                }
                if let Err(e) = self.transport.flush() {
                    write_err = Some(e.into());
                    break;
                }
                if let Err(e) = self.drain_tx_echo(&pkt) {
                    write_err = Some(e);
                    break;
                }
                written_indices.push(idx);
            }

            if self.is_aborted() {
                if !written_indices.is_empty() {
                    for &idx in &written_indices {
                        self.joint_torque_enabled[idx] = None;
                    }
                    self.partial_enable_known = true;
                } else {
                    self.partial_enable_known = false;
                }
                self.confirmed_enable_state = EnableState::Unknown;
                return Err(ArmDriverError::HardwareFault("Aborted by emergency stop".into()));
            }

            let mut all_enabled = write_err.is_none();
            let mut read_errors = Vec::new();
            let mut one_count = 0;

            for (idx, &id) in ids.iter().enumerate() {
                if self.is_aborted() {
                    for &w_idx in &written_indices {
                        if w_idx >= idx {
                            self.joint_torque_enabled[w_idx] = None;
                        }
                    }
                    self.partial_enable_known = true;
                    self.confirmed_enable_state = EnableState::Unknown;
                    return Err(ArmDriverError::HardwareFault("Aborted by emergency stop".into()));
                }
                match self.read_register_u8_direct(id, feetech::REG_TORQUE_ENABLE) {
                    Ok(1) => {
                        one_count += 1;
                        self.joint_torque_enabled[idx] = Some(true);
                        self.last_enable_confirmed[idx] = Some(self.clock_now());
                        self.missed_enable_polls[idx] = 0;
                    }
                    Ok(0) => {
                        all_enabled = false;
                        self.joint_torque_enabled[idx] = Some(false);
                        self.last_enable_confirmed[idx] = Some(self.clock_now());
                        self.missed_enable_polls[idx] = 0;
                        read_errors.push(format!("servo {} returned 0", id));
                    }
                    Ok(v) => {
                        all_enabled = false;
                        self.joint_torque_enabled[idx] = None;
                        read_errors.push(format!("servo {} returned unexpected value {}", id, v));
                    }
                    Err(e) => {
                        all_enabled = false;
                        self.joint_torque_enabled[idx] = None;
                        read_errors.push(format!("servo {} read failed: {}", id, e));
                    }
                }
            }

            if all_enabled && one_count == ids.len() {
                self.confirmed_enable_state = EnableState::Enabled;
                self.partial_enable_known = false;
                self.last_enable_poll = Some(self.clock_now());
                Ok(())
            } else {
                let mut rb_write_err = false;
                for (i, &id) in ids.iter().enumerate() {
                    if prev_joint_enabled.get(i).copied().flatten() == Some(true) {
                        continue;
                    }
                    let pkt = feetech::build_set_torque(id, false);
                    let _ = self.transport.write_all(&pkt);
                    let _ = self.transport.flush();
                    let _ = self.drain_tx_echo(&pkt);

                    match self.read_register_u8_direct(id, feetech::REG_TORQUE_ENABLE) {
                        Ok(0) => {
                            self.joint_torque_enabled[i] = Some(false);
                            self.last_enable_confirmed[i] = Some(self.clock_now());
                            self.missed_enable_polls[i] = 0;
                        }
                        _ => {
                            self.joint_torque_enabled[i] = None;
                            rb_write_err = true;
                        }
                    }
                }

                if !rb_write_err && write_err.is_none() && self.joint_torque_enabled.iter().all(|s| *s == Some(false)) {
                    self.confirmed_enable_state = EnableState::Disabled;
                    self.partial_enable_known = false;
                } else if !rb_write_err && write_err.is_none()
                    && self.joint_torque_enabled.iter().all(|s| s.is_some())
                    && self.joint_torque_enabled.contains(&Some(true))
                    && self.joint_torque_enabled.contains(&Some(false))
                {
                    self.confirmed_enable_state = EnableState::Unknown;
                    self.partial_enable_known = true;
                } else {
                    self.confirmed_enable_state = EnableState::Unknown;
                    self.partial_enable_known = false;
                }

                let err_msg = if let Some(e) = write_err {
                    format!("Torque enable write failed: {e}")
                } else {
                    format!("Torque enable readback failed: {}", read_errors.join("; "))
                };
                Err(ArmDriverError::HardwareFault(err_msg))
            }
        } else {
            let mut write_err = None;
            for &id in &ids {
                let pkt = feetech::build_set_torque(id, false);
                if let Err(e) = self.transport.write_all(&pkt) {
                    write_err = Some(e.into());
                }
                let _ = self.transport.flush();
                let _ = self.drain_tx_echo(&pkt);
            }

            let (count_0, _, _) = self.read_torque_enable_states(&ids);

            if count_0 == ids.len() {
                self.confirmed_enable_state = EnableState::Disabled;
                self.partial_enable_known = false;
            } else {
                self.confirmed_enable_state = EnableState::Unknown;
                self.partial_enable_known = false;
            }

            if let Some(e) = write_err {
                Err(e)
            } else {
                Ok(())
            }
        }
    }

    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.assert_clock_matches_transport();
        if targets.len() != self.servo_ids.len() {
            return Err(ArmDriverError::JointCountMismatch {
                expected: self.servo_ids.len(),
                actual: targets.len(),
            });
        }

        let mut positions = Vec::with_capacity(targets.len());
        let default_calib = feetech::FeetechJointCalib::default();
        for (i, target) in targets.iter().enumerate() {
            let calib = self.config.joints.get(i).unwrap_or(&default_calib);
            let range = self.effective_ranges.get(i).copied().unwrap_or_else(|| calib.effective_range_steps());
            let raw_steps = calib.checked_rad_to_steps_in_range(target.position, range)
                .map_err(|e| ArmDriverError::Protocol(e.to_string()))?;
            positions.push((self.servo_ids[i], raw_steps));
        }

        let sync_pkt = feetech::build_sync_write_positions(&positions)
            .map_err(|e| ArmDriverError::Protocol(e.to_string()))?;

        self.transport.write_all(&sync_pkt)?;
        self.transport.flush()?;
        self.drain_tx_echo(&sync_pkt)?;
        if self.incomplete_chunk_idx.is_some() && !self.rx_buffer.is_empty() {
            self.partial_pos_rx.append(&mut self.rx_buffer);
        }
        Ok(())
    }

    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        self.assert_clock_matches_transport();
        let dof = self.servo_ids.len();
        if dof == 0 {
            return Ok(Vec::new());
        }

        const MAX_SYNC_READ_SERVOS: usize = 3;
        const MAX_READ_DURATION: Duration = Duration::from_millis(8);
        let start_time = self.clock_now();
        let mut temp_chunk = [0u8; 1];

        const INCOMPLETE_FRAME_TTL: Duration = Duration::from_millis(25);
        if let Some(inc_idx) = self.incomplete_chunk_idx.take() {
            let inc_ts = self.incomplete_frame_started_at.take().unwrap_or_else(|| self.clock_now());
            if self.elapsed_since(inc_ts) >= INCOMPLETE_FRAME_TTL {
                self.invalidate_partial_pos_rx();
            } else if let Some(inc_chunk) = self.servo_ids.chunks(MAX_SYNC_READ_SERVOS).nth(inc_idx).map(|c| c.to_vec()) {
                let expected_ids: std::collections::HashSet<u8> = inc_chunk.iter().copied().collect();
                let mut replied_ids = std::collections::HashSet::new();
                self.decode_partial_position_frames(&expected_ids, &mut replied_ids, inc_ts);

                let reassembly_start = self.clock_now();
                const REASSEMBLY_TIMEOUT: Duration = Duration::from_millis(2);
                let mut drain_buf = [0u8; 32];

                while replied_ids.len() < inc_chunk.len()
                    && self.elapsed_since(reassembly_start) < REASSEMBLY_TIMEOUT
                    && self.elapsed_since(start_time) < MAX_READ_DURATION
                    && self.elapsed_since(inc_ts) < INCOMPLETE_FRAME_TTL
                {
                    match self.transport.read(&mut drain_buf) {
                        Ok(n) if n > 0 => {
                            self.partial_pos_rx.extend_from_slice(&drain_buf[..n]);
                            self.decode_partial_position_frames(&expected_ids, &mut replied_ids, inc_ts);
                        }
                        _ => {
                            if self.elapsed_since(inc_ts) >= INCOMPLETE_FRAME_TTL {
                                break;
                            }
                            self.clock_yield();
                        }
                    }
                    if self.elapsed_since(inc_ts) >= INCOMPLETE_FRAME_TTL && replied_ids.len() < inc_chunk.len() {
                        if let Ok(n) = self.transport.read(&mut drain_buf) {
                            if n > 0 {
                                self.partial_pos_rx.extend_from_slice(&drain_buf[..n]);
                                self.decode_partial_position_frames(&expected_ids, &mut replied_ids, inc_ts);
                            }
                        }
                        break;
                    }
                }

                let all_replied = replied_ids.len() == inc_chunk.len();
                let inc_timed_out = self.elapsed_since(inc_ts) >= INCOMPLETE_FRAME_TTL;

                if !all_replied && !inc_timed_out {
                    if let Some(tail) = feetech::trailing_partial_frame_for_sync_read(&self.partial_pos_rx, &inc_chunk, 10) {
                        self.partial_pos_rx = tail.to_vec();
                        self.incomplete_chunk_idx = Some(inc_idx);
                        self.incomplete_frame_started_at = Some(inc_ts);
                    } else {
                        self.invalidate_partial_pos_rx();
                    }
                } else {
                    self.invalidate_partial_pos_rx();
                }
            } else {
                self.invalidate_partial_pos_rx();
            }
        }

        self.read_poll_count += 1;
        let did_enable_poll = self.incomplete_chunk_idx.is_none()
            && (self.confirmed_enable_state == EnableState::Enabled || self.partial_enable_known)
            && (self.read_poll_count.is_multiple_of(10) || self.last_enable_poll.is_none_or(|t| self.elapsed_since(t) >= Duration::from_millis(200)));
        if did_enable_poll {
            self.last_enable_poll = Some(self.clock_now());
            self.transport.clear_input_buffer();
            self.rx_buffer.clear();
            if let Ok(sync_read) = feetech::build_sync_read(&self.servo_ids, feetech::REG_TORQUE_ENABLE, 1) {
                let _ = self.transport.write_all(&sync_read);
                let _ = self.transport.flush();
                let _ = self.drain_tx_echo(&sync_read);

                let mut temp_en = [0u8; 1];
                let start_en = self.clock_now();
                let mut replies: std::collections::HashMap<u8, u8> = std::collections::HashMap::new();

                while replies.len() < self.servo_ids.len()
                    && self.elapsed_since(start_en) < Duration::from_millis(3)
                    && self.elapsed_since(start_time) < Duration::from_millis(10)
                {
                    if let Ok(n) = self.transport.read(&mut temp_en) {
                        if n > 0 {
                            self.rx_buffer.extend_from_slice(&temp_en[..n]);
                        }
                    }
                    while !self.rx_buffer.is_empty() {
                        match feetech::parse_response(&self.rx_buffer) {
                            Ok((pkt, consumed)) => {
                                self.rx_buffer.drain(..consumed);
                                if pkt.id != feetech::BROADCAST_ID && self.servo_ids.contains(&pkt.id) && pkt.params.len() == 1 {
                                    replies.insert(pkt.id, pkt.params[0]);
                                }
                            }
                            Err(feetech::FeetechError::IncompleteFrame { .. }) => break,
                            Err(_) => {
                                self.rx_buffer.remove(0);
                            }
                        }
                    }
                    if replies.len() < self.servo_ids.len() {
                        self.clock_yield();
                    }
                }

                let now = self.clock_now();
                let prev_holding_lost = self.partial_enable_known && self.servo_ids.iter().enumerate().any(|(idx, id)| {
                    self.joint_torque_enabled.get(idx).copied().flatten() == Some(true)
                        && replies.get(id).copied() == Some(0)
                });

                for (idx, &id) in self.servo_ids.iter().enumerate() {
                    match replies.get(&id) {
                        Some(&0) => {
                            self.joint_torque_enabled[idx] = Some(false);
                            self.last_enable_confirmed[idx] = Some(now);
                            self.missed_enable_polls[idx] = 0;
                        }
                        Some(&1) => {
                            self.joint_torque_enabled[idx] = Some(true);
                            self.last_enable_confirmed[idx] = Some(now);
                            self.missed_enable_polls[idx] = 0;
                        }
                        Some(&_v) => {
                            self.joint_torque_enabled[idx] = None;
                            self.confirmed_enable_state = EnableState::Unknown;
                            self.partial_enable_known = false;
                        }
                        None => {
                            self.missed_enable_polls[idx] = self.missed_enable_polls[idx].saturating_add(1);
                            if self.missed_enable_polls[idx] >= 3
                                || self.last_enable_confirmed[idx].is_none_or(|t| now.saturating_duration_since(t) > Duration::from_millis(600))
                            {
                                self.joint_torque_enabled[idx] = None;
                                self.confirmed_enable_state = EnableState::Unknown;
                                self.partial_enable_known = false;
                            }
                        }
                    }
                }

                if replies.len() == self.servo_ids.len() && replies.values().all(|&v| v == 0) {
                    self.confirmed_enable_state = EnableState::Disabled;
                    self.partial_enable_known = false;
                } else if self.confirmed_enable_state == EnableState::Enabled && replies.values().any(|&v| v == 0) {
                    self.confirmed_enable_state = EnableState::Unknown;
                    self.partial_enable_known = false;
                } else if prev_holding_lost {
                    self.partial_enable_known = false;
                    self.confirmed_enable_state = EnableState::Unknown;
                }
            }
        }

        let chunks: Vec<Vec<u8>> = self.servo_ids.chunks(MAX_SYNC_READ_SERVOS).map(|c| c.to_vec()).collect();
        let num_chunks = chunks.len();
        let start_chunk = self.poll_cursor % num_chunks;

        if self.incomplete_chunk_idx.is_some() {
            return Ok(self.cached_states.clone());
        }

        'chunk_loop: for offset in 0..num_chunks {
            let chunk_idx = (start_chunk + offset) % num_chunks;
            let chunk = &chunks[chunk_idx];

            if self.elapsed_since(start_time) >= MAX_READ_DURATION {
                self.poll_cursor = chunk_idx;
                break 'chunk_loop;
            }

            self.transport.clear_input_buffer();
            self.rx_buffer.clear();
            let sync_read = feetech::build_sync_read(chunk, feetech::REG_PRESENT_POSITION_L, 8)
                .map_err(|e| ArmDriverError::Protocol(e.to_string()))?;

            self.transport.write_all(&sync_read)?;
            self.transport.flush()?;
            self.drain_tx_echo(&sync_read)?;

            let expected_ids: std::collections::HashSet<u8> = chunk.iter().copied().collect();
            let mut replied_ids = std::collections::HashSet::new();
            let chunk_start = self.clock_now();
            const CHUNK_TIMEOUT: Duration = Duration::from_millis(4);
            let mut read_time = self.clock_now();

            while replied_ids.len() < chunk.len() && self.elapsed_since(chunk_start) < CHUNK_TIMEOUT && self.elapsed_since(start_time) < MAX_READ_DURATION {
                self.decode_position_frames(&expected_ids, &mut replied_ids, read_time);
                if replied_ids.len() >= chunk.len() {
                    break;
                }

                match self.transport.read(&mut temp_chunk) {
                    Ok(n) if n > 0 => {
                        read_time = self.clock_now();
                        self.rx_buffer.extend_from_slice(&temp_chunk[..n]);
                    }
                    _ => {}
                }

                self.decode_position_frames(&expected_ids, &mut replied_ids, read_time);

                if replied_ids.len() < chunk.len() {
                    self.clock_yield();
                }
            }

            if replied_ids.len() < chunk.len() {
                self.missed_poll_replies += 1;
                if self.elapsed_since(start_time) < MAX_READ_DURATION {
                    self.clock_sleep(Duration::from_micros(500));
                }
                let mut drain_buf = [0u8; 64];
                let mut drained_bytes = 0;
                while self.elapsed_since(start_time) < MAX_READ_DURATION && drained_bytes < 256 {
                    match self.transport.read(&mut drain_buf) {
                        Ok(n) if n > 0 => {
                            let drain_time = self.clock_now();
                            self.rx_buffer.extend_from_slice(&drain_buf[..n]);
                            drained_bytes += n;
                            self.decode_position_frames(&expected_ids, &mut replied_ids, drain_time);
                        }
                        _ => break,
                    }
                }
                if self.rx_buffer.len() > 512 {
                    let trim = self.rx_buffer.len() - 256;
                    self.rx_buffer.drain(..trim);
                }
            }

            if replied_ids.len() < chunk.len() {
                if let Some(tail) = feetech::trailing_partial_frame_for_sync_read(&self.rx_buffer, chunk, 10) {
                    self.partial_pos_rx = tail.to_vec();
                    self.incomplete_chunk_idx = Some(chunk_idx);
                    self.incomplete_frame_started_at = Some(read_time);
                    self.rx_buffer.clear();
                    self.poll_cursor = (chunk_idx + 1) % num_chunks;
                    break 'chunk_loop;
                }
            }
            self.incomplete_chunk_idx = None;
            self.incomplete_frame_started_at = None;
            self.partial_pos_rx.clear();
            self.rx_buffer.clear();
            self.poll_cursor = (chunk_idx + 1) % num_chunks;
        }

        if self.rx_buffer.len() > 512 {
            let trim = self.rx_buffer.len() - 256;
            self.rx_buffer.drain(..trim);
        }

        Ok(self.cached_states.clone())
    }
}

// =========================================================================
// 2. FashionStar UartServo Arm Driver (Star Arm 102)
// =========================================================================

pub struct FashionStarArmDriver<T: Transport> {
    transport: T,
    servo_ids: Vec<u8>,
    zero_offsets: Vec<u16>,
    cached_states: Vec<JointState>,
    last_feedback_instant: Vec<Option<Instant>>,
    feedback_seq: Vec<u64>,
    poll_cursor: usize,
    rx_buffer: Vec<u8>,
}

impl<T: Transport> FashionStarArmDriver<T> {
    pub fn new(transport: T, servo_ids: Vec<u8>) -> Self {
        let dof = servo_ids.len();
        Self {
            transport,
            servo_ids,
            zero_offsets: vec![fashionstar::DEFAULT_ZERO_OFFSET; dof],
            cached_states: vec![JointState::default(); dof],
            last_feedback_instant: vec![None; dof],
            feedback_seq: vec![0; dof],
            poll_cursor: 0,
            rx_buffer: Vec::with_capacity(256),
        }
    }

    pub fn with_zero_offsets(mut self, offsets: Vec<u16>) -> Self {
        self.zero_offsets = offsets;
        self
    }

    pub fn transport(&self) -> &T {
        &self.transport
    }

    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    pub fn rx_buffer(&self) -> &[u8] {
        &self.rx_buffer
    }
}

impl<T: Transport> ArmDriver for FashionStarArmDriver<T> {
    fn motor_zero_support(&self) -> MotorZeroSupport {
        MotorZeroSupport::unsupported("FashionStar: armd implements no zero command for these servos")
    }

    fn capabilities(&self) -> DriverCapabilities { DriverCapabilities::default() }

    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        self.last_feedback_instant.iter().map(|t| t.map(|t| t.elapsed())).collect()
    }

    fn feedback_instants(&self) -> Vec<Option<Instant>> {
        self.last_feedback_instant.clone()
    }

    fn feedback_seqs(&self) -> Vec<u64> {
        self.feedback_seq.clone()
    }

    fn feedback_receive_instants(&self) -> Vec<Option<Instant>> {
        self.last_feedback_instant.clone()
    }

    fn is_feedback_valid(&self) -> bool {
        if self.last_feedback_instant.is_empty() {
            return false;
        }
        const FRESHNESS_LIMIT: Duration = Duration::from_millis(250);
        self.last_feedback_instant
            .iter()
            .all(|opt| opt.is_some_and(|instant| instant.elapsed() <= FRESHNESS_LIMIT))
    }

    fn read_joint_states_budgeted(
        &mut self,
        _out: &mut Vec<JointState>,
        _deadline: Instant,
        _max_rx_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        Err(ArmDriverError::Protocol("UNSUPPORTED: serial driver does not support budgeted RX".into()))
    }

    fn begin_torque_budgeted(&mut self, _enabled: bool, _max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
        Err(ArmDriverError::Protocol("UNSUPPORTED: serial driver does not support budgeted torque transitions".into()))
    }

    fn poll_torque_budgeted(&mut self, _deadline: Instant, _max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
        Err(ArmDriverError::Protocol("UNSUPPORTED: serial driver does not support budgeted torque transitions".into()))
    }

    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        if !enabled {
            let mut last_err = None;
            for &id in &self.servo_ids {
                let pkt = fashionstar::build_set_torque(id, false);
                if let Err(e) = self.transport.write_all(&pkt) {
                    last_err = Some(e.into());
                }
            }
            if let Err(e) = self.transport.flush() {
                last_err = Some(e.into());
            }
            if let Some(e) = last_err {
                return Err(e);
            }
            Ok(())
        } else {
            for &id in &self.servo_ids {
                let pkt = fashionstar::build_set_torque(id, true);
                self.transport.write_all(&pkt)?;
            }
            self.transport.flush()?;
            Ok(())
        }
    }

    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        if targets.len() != self.servo_ids.len() {
            return Err(ArmDriverError::JointCountMismatch {
                expected: self.servo_ids.len(),
                actual: targets.len(),
            });
        }

        for (i, target) in targets.iter().enumerate() {
            let angle_tenth = fashionstar::rad_to_tenth_deg(target.position);
            let pkt = fashionstar::build_set_angle(self.servo_ids[i], angle_tenth, 20, 1000);
            self.transport.write_all(&pkt)?;
        }
        self.transport.flush()?;
        Ok(())
    }

    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let dof = self.servo_ids.len();
        if dof == 0 {
            return Ok(Vec::new());
        }

        let mut temp_chunk = [0u8; 1];
        let start_time = Instant::now();
        const MAX_READ_DURATION: Duration = Duration::from_millis(3);
        const MAX_TOTAL_BYTES: usize = 256;
        let mut total_bytes_read = 0;
        let start_cursor = self.poll_cursor;

        'poll: for offset in 0..dof {
            let joint_idx = (start_cursor + offset) % dof;
            let id = self.servo_ids[joint_idx];

            if start_time.elapsed() >= MAX_READ_DURATION || total_bytes_read >= MAX_TOTAL_BYTES {
                self.poll_cursor = joint_idx;
                break 'poll;
            }

            let query = fashionstar::build_query_angle(id);
            self.transport.write_all(&query)?;
            self.transport.flush()?;

            let mut attempts = 0;
            while attempts < 10 {
                if start_time.elapsed() >= MAX_READ_DURATION || total_bytes_read >= MAX_TOTAL_BYTES {
                    self.poll_cursor = (joint_idx + 1) % dof;
                    break 'poll;
                }

                match fashionstar::parse_packet(&self.rx_buffer) {
                    Ok((FashionStarPacket { header, id: pkt_id, cmd_id, content }, consumed)) => {
                        self.rx_buffer.drain(..consumed);
                        // Filter out local echo of request frame ([0x12, 0x4C]) on half-duplex RS485 bus
                        if header == fashionstar::FASHIONSTAR_REQUEST_HEADER {
                            continue;
                        }
                        if header == fashionstar::FASHIONSTAR_RESPONSE_HEADER
                            && pkt_id == id
                            && cmd_id == fashionstar::CMD_QUERY_SERVO_ANGLE
                            && content.len() >= 2
                        {
                            let tenth_deg = i16::from_le_bytes([content[0], content[1]]);
                            self.cached_states[joint_idx].position = fashionstar::tenth_deg_to_rad(tenth_deg);
                            self.last_feedback_instant[joint_idx] = Some(Instant::now());
                            self.feedback_seq[joint_idx] = self.feedback_seq[joint_idx].wrapping_add(1);
                            break;
                        }
                    }
                    Err(fashionstar::FashionStarError::IncompleteFrame { .. }) => {
                        // Incomplete frame; read more bytes from transport
                    }
                    Err(_) => {
                        // Checksum mismatch or corruption: slide window by 1 byte to recover
                        if !self.rx_buffer.is_empty() {
                            self.rx_buffer.remove(0);
                        }
                    }
                }

                // Bound buffer size to prevent memory leaks / lockups
                if self.rx_buffer.len() > 1024 {
                    let trim = self.rx_buffer.len() - 256;
                    self.rx_buffer.drain(..trim);
                }

                match self.transport.read(&mut temp_chunk) {
                    Ok(n) if n > 0 => {
                        total_bytes_read += n;
                        self.rx_buffer.extend_from_slice(&temp_chunk[..n]);
                    }
                    _ => {
                        attempts += 1;
                        if attempts >= 3 {
                            break;
                        }
                    }
                }
            }
        }

        Ok(self.cached_states.clone())
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
    /// Atomic E-stop latch (`ArmDriver::set_abort_flag`), checked before enable transmissions.
    abort_flag: Option<Arc<AtomicBool>>,
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
    /// Sticky torque-off proof: set once every motor confirmed `state == 0` via post-command
    /// feedback after a disable request; stays `Disabled` for `enable_state()` even without
    /// newer polls (e.g. after an unacknowledged MIT fence trip) until a later enable request
    /// or non-zero motor state feedback is observed.
    disable_confirmed_sticky: bool,
    awaiting_rx_boundary: bool,
    is_enabling: bool,
    last_warn_sec: AtomicU64,
    last_refresh_instant: Option<Instant>,
    /// Confirmed control mode per joint; None = unconfirmed after a failed switch.
    joint_modes: Vec<Option<DamiaoControlMode>>,
    /// PMAX / VMAX / TMAX read back from the motor and equal to `limits[i]`.
    mit_limits_verified: Vec<bool>,
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
    /// RX sequence number of the last observed-empty receive queue: every frame dequeued after
    /// it (sequence number larger) was enqueued after that observation.
    rx_empty_seq: u64,
    /// The last `read_joint_states` observed an empty receive queue.
    last_read_drained: bool,
    /// Deterministic clock of the CAN transport (tests); None = the system clock.
    clock: Option<std::sync::Arc<dyn crate::transport::DriverClock>>,
    /// Sequence number of the last processed RX frame, and of each joint's last feedback.
    rx_seq: u64,
    last_feedback_seq: Vec<u64>,
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
    /// RX sequence boundary captured at the start of the most recent [`ArmDriver::mit_expiry_recover`]
    /// request; final MIT clear requires every joint's `last_feedback_seq` to exceed this boundary.
    last_mit_recovery_rx_seq: Option<u64>,
    /// MIT tracking armed a fence since the last confirmed enable: base-mode restores use POS_VEL
    /// frames only.
    mit_tracking_used: bool,
    /// Pending torque transition (`begin_torque` / `poll_torque`); None = idle.
    torque_hs: Option<DamiaoTorque>,
    /// Service deadline capping the receive pass of the current `poll_torque` (None: 5 ms only).
    rx_deadline: Option<Instant>,
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
    /// Round-robin start index for budget-constrained 0xCC refresh sweeps.
    refresh_cursor: usize,
    /// Bitmask of stale joints not yet polled in the current 0xCC refresh sweep.
    refresh_unfinished: u64,
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
    /// True once a J7 `0xCC` refresh has been attempted during the active tick (`begin_tx_tick()`).
    pf_refresh_attempted_this_tick: bool,
    /// True when an output pass ran since the previous `receive_feedback()` pass.
    output_ran_since_rx: bool,
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
    /// Safety torque-off (`CMD_DISABLE`): never stop-gated and exempt from the motion TX budget.
    Disable,
    /// Motion target frame (`POS_VEL`, `MIT`, or `POS_FORCE`).
    Target,
    /// Non-rollback mode-switch hold frame.
    Hold,
    /// Base-mode rollback or post-E-stop stationary hold frame for `joint`: refused while a
    /// stop/E-stop is pending; allowed while latched only when `joint` has fresh (`<= 100 ms`)
    /// measured feedback and the hold frame is rebuilt in step 4. Never used for `CTRL_MODE`
    /// register writes (those are always `ModeWrite`).
    RollbackHold(usize),
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
    /// Step 4 (pre-build): active MIT deadline fence tripped.
    MitFenceTripped(MitFenceTripReason),
    /// Step 4/5: active `PosForce` joint feedback aged past `POS_FORCE_FEEDBACK_MAX_AGE` (100 ms)
    /// before/at submission (`can.transmit`); distinct from FIFO `DeadlinePassed` and `BuildRefused`.
    PosForceFeedbackExpired(usize),
    /// Step 4: send-time `build` returned `None` (or latched rollback lacked fresh feedback).
    BuildRefused,
    /// Step 5: stop/E-stop generation stamp changed since admission or operation start.
    StopStampChanged,
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
}

#[derive(Clone, Copy, Debug)]
struct PendingTxFrame {
    joint: usize,
    frame_id: u32,
    cmd: PendingTargetCmd,
    queued_at: Instant,
    stop_stamp: StopStamp,
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

    fn clear(&mut self) -> usize {
        let dropped = self.len;
        for slot in &mut self.buf[..self.len] {
            *slot = None;
        }
        self.len = 0;
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
    ) -> bool {
        if let Some(idx) = self.find_key(joint, frame_id) {
            if let Some(prev) = self.buf[idx].filter(|prev| prev.stop_stamp == stop_stamp) {
                self.buf[idx] = Some(PendingTxFrame {
                    joint,
                    frame_id,
                    cmd,
                    queued_at: prev.queued_at,
                    stop_stamp,
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

/// Phase of a pending Damiao torque transition.
#[derive(Debug)]
enum DamiaoTorque {
    /// Budgeted enable waiting for an empty RX queue boundary before mode restore / enable TX.
    DrainBeforeEnable(Deadline),
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
    /// fresh state-1 feedback from every motor.
    Enable { window: ConfirmWindow, unsent_mask: u32 },
    /// Disable frames sent, awaiting fresh state-0 feedback from every motor.
    Disable { window: ConfirmWindow, mit_fenced: bool, after_seq: u64 },
}

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
    first: Option<RefreshSample>,
    /// First post-boundary J7 reply observed during this refresh that violated stationary
    /// torque-off (`|v| > 0.05`, non-finite velocity, or `motor_status != 0`). Sticky across
    /// polls so a later benign reply cannot mask an intervening motion or fault event.
    unsafe_sample: Option<RefreshSample>,
}

enum RefreshStep {
    Ready(RefreshSample),
    Pending(FeedbackRefresh),
    TimedOut,
}

/// CTRL_MODE read of the position-force gripper: `boundary` is the RX sequence of the empty
/// queue observed before the request (None: not sent yet, waiting for an empty queue).
#[derive(Debug)]
struct ModeReadback { joint: usize, boundary: Option<u64>, deadline: Deadline }

/// Maximum joints supported in inline stack-allocated [`ModeWrite`] buffers.
const MAX_MODE_JOINTS: usize = 16;

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
/// Time allowed per motor for the four register replies of the startup range check
/// (7 silent motors: 140 ms).
const RANGE_CHECK_REPLY_TIMEOUT: Duration = Duration::from_millis(20);
/// Feedback freshness bound for position-force gripper mode writes, pre-enable holds,
/// stale-feedback recovery, and opening-speed allowance.
const POS_FORCE_FEEDBACK_MAX_AGE: Duration = Duration::from_millis(100);

impl<C: CanTransport> DamiaoArmDriver<C> {
    pub fn new(can: C, motor_ids: Vec<u16>) -> Self {
        let dof = motor_ids.len();
        let clock = can.clock();
        Self {
            can,
            motor_ids,
            master_id: 0,
            configured_master_id: None,
            limits: vec![DamiaoLimits::default(); dof],
            control_mode: DamiaoControlMode::PosVel,
            default_vlim: 2.0,
            default_kp: 40.0,
            default_kd: 1.0,
            cached_states: vec![JointState::default(); dof],
            last_target_frames: vec![None; dof],
            last_mit_params: vec![None; dof],
            last_feedback_instant: vec![None; dof],
            feedback_seq: vec![0; dof],
            rx_empty_instant: None,
            feedback_queue_bound: vec![None; dof],
            motor_status: vec![0; dof],
            is_enabled: false,
            pending_rearm: true,
            last_query_instant: None,
            last_enable_command: None,
            last_enable_command_rx_seq: None,
            last_command_was_disable: false,
            disable_confirmed_sticky: false,
            awaiting_rx_boundary: false,
            is_enabling: false,
            last_warn_sec: AtomicU64::new(0),
            last_refresh_instant: None,
            joint_modes: vec![Some(DamiaoControlMode::PosVel); dof],
            mit_limits_verified: vec![false; dof],
            register_replies: Vec::with_capacity(64),
            range_check: None,
            recovery_written_joints: Vec::with_capacity(dof.max(16)),
            strict_mode_handover: false,
            register_reply_log: Vec::with_capacity(64),
            clock,
            rx_seq: 0,
            last_feedback_seq: vec![0; dof],
            rx_empty_seq: 0,
            last_read_drained: false,
            mit_fence: None,
            mit_fence_last_cmd: vec![None; dof],
            estop_gate: None,
            abort_flag: None,
            rt_no_sleep: false,
            mit_fence_trip: None,
            mit_fence_holdoff: Duration::ZERO,
            mit_fence_acked: false,
            last_mit_recovery_rx_seq: None,
            mit_tracking_used: false,
            torque_hs: None,
            rx_deadline: None,
            rx_frame_limit: None,
            last_poll_rx: None,
            base_modes: vec![DamiaoControlMode::PosVel; dof],
            pos_force: None,
            pos_force_verified: false,
            legacy_gripper: None,
            refresh_capture: None,
            tx_budget: TxBudget::default(),
            pending_tx: PendingTxQueue::new(),
            pending_hold_tx: PendingTxQueue::new(),
            hold_cursor: 0,
            last_hold_tx_sent_any: false,
            last_target_tx_sent_mask: 0,
            last_expired_oldest_age: None,
            tick_tx: None,
            refresh_cursor: 0,
            refresh_unfinished: 0,
            enable_stop_stamp: None,
            pf_last_fresh_feedback: None,
            pf_last_fresh_position: None,
            tx_release_deadline: None,
            tx_probe: None,
            last_tx_pf_status: PosForceTxStatus::Unobserved,
            pf_fresh_gen: 0,
            pf_refresh_pending: false,
            pf_refresh_deferred_ticks: 0,
            pf_refresh_attempted_this_tick: false,
            output_ran_since_rx: false,
            last_hold_tx_due_mask: 0,
            last_hold_tx_sent_mask: 0,
            external_pos_force_emitter: false,
            pf_deferred_ticks: 0,
            pf_deferred_motion: false,
            pf_deferred_since: None,
            pf_deferred_intent: None,
        }
    }

    /// Active per-tick CAN transmit budget.
    pub fn tx_budget(&self) -> TxBudget { self.tx_budget }

    /// Override the per-tick CAN transmit budget (builder form).
    pub fn with_tx_budget(mut self, budget: TxBudget) -> Self {
        self.tx_budget = budget;
        self
    }

    /// Override the per-tick CAN transmit budget and open a fresh tick pass.
    pub fn set_tx_budget(&mut self, budget: TxBudget) {
        self.tx_budget = budget;
        self.tick_tx = None;
        self.tx_release_deadline = None;
    }

    /// Open a fresh per-tick TX budget window. Called automatically at the start of
    /// `receive_feedback()`; exposed for direct driver tests and executors that step TX
    /// independently of `read_joint_states()`.
    pub fn begin_tx_tick(&mut self) {
        self.tick_tx = None;
        self.pf_refresh_attempted_this_tick = false;
    }

    #[inline]
    fn current_stop_stamp(&self) -> StopStamp {
        let (estop_gen, stop_gen) = self
            .estop_gate
            .as_ref()
            .map_or((0, 0), crate::rt_core::urgent::EstopGate::generations);
        StopStamp { estop_gen, stop_gen }
    }

    /// Current driver-gate `(stop_gen, estop_epoch)` stamp for direct-driver callers of
    /// [`ArmDriver::emit_pos_force_j7`].
    #[inline]
    pub fn current_j7_intent_stamp(&self) -> PosForceIntentStamp {
        let s = self.current_stop_stamp();
        PosForceIntentStamp {
            stop_gen: s.stop_gen,
            estop_epoch: s.estop_gen,
        }
    }

    /// Number of unsent target frames currently queued for the next tick.
    pub fn pending_tx_len(&self) -> usize { self.pending_tx.len() }

    /// Number of unsent stopped-hold frames currently queued for the next tick.
    pub fn pending_hold_tx_len(&self) -> usize { self.pending_hold_tx.len() }

    #[inline]
    fn total_pending_tx_len(&self) -> u32 {
        (self.pending_tx.len().saturating_add(self.pending_hold_tx.len())) as u32
    }

    /// Explicitly drop all queued motion frames (e.g. on an ordinary stop from a bounded caller).
    /// Returns how many frames were dropped and counts them under `tx_dropped_stopped`.
    pub fn discard_pending_tx(&mut self) -> usize {
        self.tx_probe = None;
        let n = self.pending_tx.clear();
        self.pf_deferred_motion = false;
        self.pf_deferred_since = None;
        self.pf_deferred_intent = None;
        if n > 0 {
            crate::diagnostics::tick::record_tx_dropped_stopped(n as u32);
        }
        crate::diagnostics::tick::record_tx_deferred(0, self.total_pending_tx_len());
        n
    }

    /// Drop carried stopped-hold frames from an older stop generation.
    fn drop_stopped_hold_tx(&mut self, stamp: StopStamp) -> usize {
        let dropped = self.pending_hold_tx.drop_stopped(stamp);
        if dropped > 0 {
            crate::diagnostics::tick::record_tx_dropped_stopped(dropped as u32);
        }
        dropped
    }

    /// Drop all queued stopped-hold frames and reset the round-robin hold cursor.
    fn discard_pending_hold_tx(&mut self) {
        self.tx_probe = None;
        let n = self.pending_hold_tx.clear();
        self.hold_cursor = 0;
        if n > 0 {
            crate::diagnostics::tick::record_tx_dropped_stopped(n as u32);
        }
        crate::diagnostics::tick::record_tx_deferred(0, self.total_pending_tx_len());
    }

    /// Drop both queued motion and queued stopped-hold frames (lifecycle transitions out of hold).
    fn discard_all_pending_tx(&mut self) {
        let _ = self.discard_pending_tx();
        self.discard_pending_hold_tx();
        self.pf_deferred_ticks = 0;
    }

    #[inline]
    fn ensure_tx_pass(&mut self) -> TickTxPass {
        if let Some(mut pass) = self.tick_tx {
            if let Some(cap) = self.tx_release_deadline {
                pass.deadline = pass.deadline.min(cap);
                self.tick_tx = Some(pass);
            }
            pass
        } else {
            let mut deadline = self.now() + self.tx_budget.max_duration;
            if let Some(cap) = self.tx_release_deadline {
                deadline = deadline.min(cap);
            }
            let pass = TickTxPass {
                deadline,
                attempts: 0,
                backpressured: false,
            };
            self.tick_tx = Some(pass);
            pass
        }
    }

    #[inline]
    fn can_start_tick_tx(&self) -> bool {
        self.can_start_tick_tx_n(1)
    }

    /// True when a transport with a real (`!flush_is_noop()`) flush cannot start `flush()`
    /// because the current tick TX pass deadline has already elapsed.
    #[inline]
    fn flush_cutoff_exceeded(&self) -> bool {
        !self.can.flush_is_noop()
            && (self.tick_tx.is_some_and(|pass| self.now() >= pass.deadline)
                || self.tx_release_deadline.is_some_and(|cap| self.now() >= cap))
    }

    /// Shared mode-switch implementation used by both off-tick [`ArmDriver::set_joint_modes`]
    /// (which opens a fresh TX pass) and in-tick [`ArmDriver::set_joint_modes_in_tick`] (which
    /// shares the active release's TX pass).
    fn set_joint_modes_impl(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        let start_stamp = self.current_stop_stamp();
        let n = self.motor_ids.len();
        if modes.len() != n || hold.len() != n {
            return Err(ArmDriverError::JointCountMismatch { expected: n, actual: modes.len().min(hold.len()) });
        }
        if !self.supports_mit_mode_switch() {
            return Err(ArmDriverError::Protocol("joint mode switching needs the POS_VEL base mode".into()));
        }
        let target: Vec<DamiaoControlMode> = modes.iter().enumerate().map(|(i, m)| match m {
            JointMode::Position => self.base_mode(i),
            JointMode::Mit => DamiaoControlMode::Mit,
        }).collect();
        let changing: Vec<usize> = (0..n).filter(|&i| self.joint_modes[i] != Some(target[i])).collect();
        if changing.is_empty() { return Ok(()); }
        let is_base_restore = changing.iter().all(|&i| target[i] == self.base_mode(i));
        // CTRL_MODE writes (forward and rollback alike) are refused while a stop/E-stop is pending OR latched.
        self.strict_stop_check(Some(start_stamp))?;
        let forward_policy = if !is_base_restore {
            ModeWritePolicy::Strict(start_stamp)
        } else {
            ModeWritePolicy::BaseRollback { start_stamp, force_fresh_hold: false }
        };
        let effective_hold = hold.to_vec();
        self.validate_pos_force_holds(&changing, &target, &effective_hold, false)?;
        let to_mit: Vec<usize> = changing.iter().copied()
            .filter(|&i| target[i] == DamiaoControlMode::Mit && !self.mit_limits_verified[i]).collect();
        if !to_mit.is_empty() { self.verify_mit_limits(&to_mit, start_stamp)?; }
        let previous = self.joint_modes.clone();
        let failed = self.write_modes(&changing, &target, &effective_hold, false, forward_policy)?;
        if failed.is_empty() {
            tracing::info!("Damiao control mode switched for joints {:?}", changing.iter().map(|i| i + 1).collect::<Vec<_>>());
            return Ok(());
        }
        // Not confirmed: switch every changed joint back to its previous mode, holding it
        // with both frame kinds meanwhile so it holds whichever mode it is actually in.
        for &i in &changing { self.joint_modes[i] = None; }
        let back: Vec<DamiaoControlMode> = (0..n).map(|i| previous[i].unwrap_or(self.base_mode(i))).collect();
        let back_is_base = changing.iter().all(|&i| back[i] == self.base_mode(i));
        // Rollback CTRL_MODE writes are also ModeWrite: refused while pending, latched, or after a stop.
        self.strict_stop_check(Some(start_stamp))?;
        let rollback_policy = if !back_is_base {
            ModeWritePolicy::Strict(start_stamp)
        } else {
            ModeWritePolicy::BaseRollback { start_stamp, force_fresh_hold: false }
        };
        let rollback_hold = hold.to_vec();
        let unrecovered = self.write_modes(&changing, &back, &rollback_hold, true, rollback_policy)?;
        Err(ArmDriverError::HardwareFault(format!(
            "Control-mode switch not confirmed for joints {:?}; switched back{}",
            failed.iter().map(|i| i + 1).collect::<Vec<_>>(),
            if unrecovered.is_empty() { String::new() } else {
                format!(", joints {:?} still unconfirmed", unrecovered.iter().map(|i| i + 1).collect::<Vec<_>>())
            })))
    }

    #[inline]
    fn can_start_tick_tx_n(&self, needed: usize) -> bool {
        let Some(pass) = self.tick_tx else {
            return needed <= self.tx_budget.max_frames
                && self.tx_release_deadline.is_none_or(|cap| self.now() < cap);
        };
        let effective_deadline = self
            .tx_release_deadline
            .map_or(pass.deadline, |cap| pass.deadline.min(cap));
        !pass.backpressured
            && pass.attempts.saturating_add(needed) <= self.tx_budget.max_frames
            && self.now() < effective_deadline
    }

    /// Per-tick TX frame slots reserved for the J7 `PosForce` tick-epilogue emitter when J7 (or
    /// its stale `0xCC` refresh) was deferred on the previous tick, guaranteeing J7 never starves
    /// under tight budgets (e.g. `max_frames = 6` on a 7-motor arm).
    #[inline]
    fn pos_force_reserved_slots(&self) -> usize {
        if !self.is_enabled || self.active_pos_force_joint().is_none() {
            return 0;
        }
        let j7_frame = usize::from(self.pf_deferred_ticks >= 1);
        let refresh = usize::from(
            self.pf_refresh_pending
                && self.pf_refresh_deferred_ticks >= 1
                && !self.pf_refresh_attempted_this_tick
                && self.poll_tx_allowed(),
        );
        (j7_frame + refresh).min(self.tx_budget.max_frames)
    }

    /// True when an arm joint (J1–J6) may start one more frame transmit in the current tick pass
    /// while leaving [`Self::pos_force_reserved_slots`] available for the J7 tick-epilogue emitter.
    #[inline]
    fn can_start_arm_tx(&self) -> bool {
        let arm_cap = self.tx_budget.max_frames.saturating_sub(self.pos_force_reserved_slots());
        let Some(pass) = self.tick_tx else {
            return 1 <= arm_cap
                && self.tx_release_deadline.is_none_or(|cap| self.now() < cap);
        };
        let effective_deadline = self
            .tx_release_deadline
            .map_or(pass.deadline, |cap| pass.deadline.min(cap));
        !pass.backpressured
            && pass.attempts.saturating_add(1) <= arm_cap
            && self.now() < effective_deadline
    }

    #[inline]
    fn record_tx_pass_attempt(&mut self) {
        let mut pass = self.ensure_tx_pass();
        pass.attempts = pass.attempts.saturating_add(1);
        self.tick_tx = Some(pass);
    }

    #[inline]
    fn mark_tx_pass_backpressured(&mut self) {
        self.tx_probe = None;
        let mut pass = self.ensure_tx_pass();
        pass.backpressured = true;
        self.tick_tx = Some(pass);
    }

    fn discard_pending_tx_stopped(&mut self) {
        let _ = self.discard_pending_tx();
    }

    /// Centralized preflight rejection cleanup: invalidates any carried-over pending frames and
    /// trips an active MIT deadline fence (clearing cached frames) while preserving the caller's
    /// concrete validation error (zero allocation).
    fn reject_target_tx(&mut self, reason: MitFenceTripReason) {
        self.discard_all_pending_tx();
        if self.mit_fence.is_some() || self.mit_fence_trip.is_some() {
            self.trip_fence(reason);
        }
    }

    #[inline]
    fn is_stop_pending(&self) -> bool {
        self.estop_gate.as_ref().is_some_and(|g| g.pending())
    }

    #[inline]
    fn is_stop_latched(&self) -> bool {
        self.abort_flag.as_ref().is_some_and(|f| f.load(Ordering::SeqCst))
    }

    #[inline]
    fn strict_stop_check(&self, expected_stamp: Option<StopStamp>) -> Result<(), ArmDriverError> {
        if self.is_stop_pending()
            || self.is_stop_latched()
            || expected_stamp.is_some_and(|s| self.current_stop_stamp() != s)
        {
            return Err(ArmDriverError::EstopInhibited);
        }
        Ok(())
    }

    #[inline]
    fn pending_stop_check(&self, expected_stamp: Option<StopStamp>) -> Result<(), ArmDriverError> {
        if self.is_stop_pending()
            || expected_stamp.is_some_and(|s| self.current_stop_stamp() != s)
        {
            return Err(ArmDriverError::EstopInhibited);
        }
        Ok(())
    }

    fn abort_enable_for_estop(&mut self) {
        if matches!(self.torque_hs, Some(DamiaoTorque::DrainBeforeEnable(_))) {
            self.awaiting_rx_boundary = false;
        }
        self.torque_hs = None;
        self.enable_failed();
    }

    fn build_pending_target_frame(&self, entry: &PendingTxFrame) -> Option<CanFrame> {
        match entry.cmd {
            PendingTargetCmd::PosVel { position } => Some(self.pos_vel_frame(entry.joint, position)),
            PendingTargetCmd::Mit { position, velocity, kp, kd, torque } => {
                let target = JointTarget { position, velocity, kp, kd, torque };
                Some(self.mit_frame(entry.joint, &target))
            }
            PendingTargetCmd::RollbackHold { .. } => None,
        }
    }

    #[inline]
    fn stop_gate_for_kind(&self, kind: TxKind) -> Result<bool, TxRefusal> {
        let pending = self.is_stop_pending();
        let latched = self.is_stop_latched();
        match kind {
            TxKind::Disable => {}
            TxKind::Refresh | TxKind::RollbackHold(_) | TxKind::ClearFault => {
                if pending {
                    return Err(TxRefusal::StopInhibited);
                }
            }
            TxKind::Target
            | TxKind::Hold
            | TxKind::ModeWrite
            | TxKind::Recovery
            | TxKind::Handshake => {
                if pending || latched {
                    return Err(TxRefusal::StopInhibited);
                }
            }
        }
        Ok(latched)
    }

    /// Single choke point for EVERY Damiao CAN transmit (including safety disable and
    /// panic-disable). Enforces all invariants in fixed order immediately before transmitting,
    /// without heap allocation:
    /// 1. Deadline check (if given): refuse once passed (includes drains and retries).
    /// 2. Tick TX budget check (`Disable` exempt): exhaustion returns `Deferred`, not failure.
    /// 3. Stop gate by `TxKind`:
    ///    - `Disable`: never gated.
    ///    - `Refresh` / `RollbackHold` / `ClearFault`: refused while pending; allowed while
    ///      latched (`RollbackHold` additionally requires fresh `<= 100 ms` feedback in step 4).
    ///    - All other kinds: refused while pending OR latched.
    /// 4. Optional MIT deadline fence + build frame at send time via `build` (`None` fails closed).
    /// 5. Post-build re-read of the physical stop latch, pending gate, and stop-generation stamp
    ///    immediately before `can.transmit` (closes the race where `trigger_e_stop_checked` sets
    ///    `e_stop_latched` before advancing `requested_estop_count`).
    /// 6. Account budget attempt, transmit, and record per-frame wall/CPU diagnostics.
    fn send(
        &mut self,
        kind: TxKind,
        expected_stamp: Option<StopStamp>,
        deadline: Option<Deadline>,
        fenced: bool,
        build: impl FnOnce(&Self) -> Option<CanFrame>,
    ) -> Result<CanFrame, TxRefusal> {
        // 1. Deadline check (inclusive: now >= deadline.at).
        if deadline.is_some_and(|d| d.passed(self.now())) {
            return Err(TxRefusal::DeadlinePassed);
        }

        // 2. Tick TX budget check (Disable / panic-disable exempt from motion TX budget).
        let budgeted = !matches!(kind, TxKind::Disable);
        if budgeted {
            let _ = self.ensure_tx_pass();
            if !self.can_start_tick_tx() {
                return Err(TxRefusal::Deferred);
            }
        }

        // 3. Stop gate by TxKind.
        let latched_before = self.stop_gate_for_kind(kind)?;

        // 4. MIT deadline fence (when fenced) + build frame fresh at send time.
        if fenced {
            if let Err(ArmDriverError::MitFenceTripped(reason)) = self.fence_check() {
                return Err(TxRefusal::MitFenceTripped(reason));
            }
        }
        if let TxKind::RollbackHold(joint) = kind {
            if latched_before
                && self.fresh_measured_position(joint, POS_FORCE_FEEDBACK_MAX_AGE).is_none()
                && self.active_pos_force_joint() != Some(joint)
            {
                return Err(TxRefusal::BuildRefused);
            }
        }
        let Some(frame) = build(self) else {
            if let TxKind::RollbackHold(joint) = kind {
                if self.active_pos_force_joint() == Some(joint)
                    && !self.pos_force_feedback_valid(joint, self.now())
                {
                    return Err(TxRefusal::PosForceFeedbackExpired(joint));
                }
            }
            return Err(TxRefusal::BuildRefused);
        };

        // 5. Re-read physical stop latch, pending gate, stop-generation stamp, MIT fence,
        //    position-force submission freshness, deadline, and tick TX cutoff AFTER `build`,
        //    immediately before `can.transmit` (host submission boundary).
        let latched_after = self.stop_gate_for_kind(kind)?;
        if matches!(kind, TxKind::RollbackHold(_)) && !latched_before && latched_after {
            return Err(TxRefusal::StopInhibited);
        }
        if !matches!(kind, TxKind::Disable)
            && expected_stamp.is_some_and(|stamp| self.current_stop_stamp() != stamp)
        {
            return Err(TxRefusal::StopStampChanged);
        }
        if fenced {
            if let Err(ArmDriverError::MitFenceTripped(reason)) = self.fence_check() {
                return Err(TxRefusal::MitFenceTripped(reason));
            }
        }
        let submit_now = self.now();
        if let Some(pf_joint) = self.frame_pos_force_joint(&frame) {
            if !self.pos_force_feedback_valid(pf_joint, submit_now)
                && !self.is_pos_force_freeze_frame(&frame)
            {
                return Err(TxRefusal::PosForceFeedbackExpired(pf_joint));
            }
        }
        if deadline.is_some_and(|d| d.passed(submit_now)) {
            return Err(TxRefusal::DeadlinePassed);
        }
        if budgeted && !self.can_start_tick_tx() {
            return Err(TxRefusal::Deferred);
        }

        // 6. Account budget attempt, transmit, and record diagnostics.
        if budgeted {
            self.record_tx_pass_attempt();
        }
        let t0 = self.now();
        let cpu0 = crate::diagnostics::tick::thread_cpu_ns();
        let res = self.can.transmit(&frame);
        let wall_ns = self.now().saturating_duration_since(t0).as_nanos().min(u128::from(u64::MAX)) as u64;
        let cpu_ns = crate::diagnostics::tick::thread_cpu_ns().saturating_sub(cpu0);
        let attempt_kind = match &res {
            Ok(()) => crate::diagnostics::tick::TxAttemptKind::Sent,
            Err(TransportError::WouldBlock) => crate::diagnostics::tick::TxAttemptKind::WouldBlock,
            Err(TransportError::NoBufferSpace) => crate::diagnostics::tick::TxAttemptKind::Enobufs,
            Err(_) => crate::diagnostics::tick::TxAttemptKind::Error,
        };
        crate::diagnostics::tick::record_tx_attempt(attempt_kind, wall_ns, cpu_ns);
        match res {
            Ok(()) => Ok(frame),
            Err(e) => {
                self.tx_probe = None;
                if budgeted && e.is_tx_backpressure() {
                    self.mark_tx_pass_backpressured();
                }
                Err(TxRefusal::Transport(e))
            }
        }
    }

    /// Off-tick retry wrapper over [`Self::send`]: retries transient backpressure up to 3 times
    /// 100 µs apart ONLY when `!rt_no_sleep && !fenced`, re-running ALL 6 steps of `send`
    /// (deadline, budget, stop gate, fresh `build`, stamp, transmit) on every attempt.
    fn send_with_retry(
        &mut self,
        kind: TxKind,
        expected_stamp: Option<StopStamp>,
        deadline: Option<Deadline>,
        fenced: bool,
        mut build: impl FnMut(&Self) -> Option<CanFrame>,
    ) -> Result<CanFrame, TxRefusal> {
        let mut retries = 0u8;
        loop {
            match self.send(kind, expected_stamp, deadline, fenced, |d| build(d)) {
                Ok(frame) => return Ok(frame),
                Err(TxRefusal::Transport(e))
                    if e.is_tx_backpressure() && !self.rt_no_sleep && !fenced && retries < 3 =>
                {
                    retries += 1;
                    // Allow off-tick retry attempts within the remaining max_frames / max_duration
                    // budget while keeping backpressure latched once retries exhaust.
                    if let Some(pass) = self.tick_tx.as_mut() {
                        pass.backpressured = false;
                    }
                    self.pause(Duration::from_micros(100));
                }
                Err(r) => return Err(r),
            }
        }
    }

    /// Drive `cfg.joint` in position-force mode with `cfg.current_raw` on every frame. The mode
    /// (RAM) is written at the next enable with torque off and confirmed by its register reply
    /// and a readback; until then no command frame reaches that joint.
    pub fn with_gripper_pos_force(mut self, cfg: GripperPosForce) -> Result<Self, ArmDriverError> {
        if cfg.joint >= self.motor_ids.len() {
            return Err(ArmDriverError::Protocol(format!("pos-force gripper joint {} outside {} joints", cfg.joint + 1, self.motor_ids.len())));
        }
        if cfg.current_raw == 0 || cfg.current_raw > damiao::POS_FORCE_I_DES_FULL
            || !(cfg.close_speed_rad_s.is_finite() && cfg.close_speed_rad_s > 0.0)
            || damiao::pos_force_vel_raw(cfg.close_speed_rad_s) == 0
            || !(cfg.closed_rad.is_finite() && cfg.open_rad.is_finite()) || cfg.closed_rad == cfg.open_rad {
            return Err(ArmDriverError::Protocol("pos-force gripper: current_raw 1..=10000, close_speed quantizing >= 0.01 rad/s, closed_rad != open_rad".into()));
        }
        self.base_modes[cfg.joint] = DamiaoControlMode::PosForce;
        self.pos_force = Some(cfg);
        Ok(self)
    }

    /// Apply the profile's gripper drive (`gripper_drive_from_profile`).
    pub fn with_gripper_drive(self, drive: Option<GripperDrive>) -> Result<Self, ArmDriverError> {
        match drive {
            None => Ok(self),
            Some(GripperDrive::PosVel(j)) => Ok(self.with_gripper_pos_vel(j)),
            Some(GripperDrive::PosForce(cfg)) => self.with_gripper_pos_force(cfg),
        }
    }

    /// Legacy profile: `joint` is the gripper, driven in POS_VEL; reported as "no motor current cap".
    pub fn with_gripper_pos_vel(mut self, joint: usize) -> Self {
        self.legacy_gripper = Some(joint);
        self
    }

    /// Base mode of joint `i` (what `JointMode::Position` and an enable restore mean).
    fn base_mode(&self, i: usize) -> DamiaoControlMode { self.base_modes.get(i).copied().unwrap_or(self.control_mode) }

    pub fn with_master_id(mut self, master_id: u32) -> Self {
        self.master_id = master_id;
        self.configured_master_id = Some(master_id);
        self
    }

    /// Expected CAN arbitration ID (MST_ID) for replies from joint `i`:
    /// the configured `master_id` when set, otherwise Damiao's per-motor `0x10 + motor_id`
    /// (e.g. J7 motor ID 7 -> 0x17 on B601-DM).
    fn expected_reply_id(&self, i: usize) -> u32 {
        self.configured_master_id.unwrap_or(0x10 + u32::from(self.motor_ids[i]))
    }

    /// Global base mode (POS_VEL or MIT); a position-force gripper keeps its own base mode.
    pub fn with_control_mode(mut self, mode: DamiaoControlMode) -> Self {
        self.control_mode = mode;
        self.joint_modes.fill(Some(mode));
        self.base_modes.fill(mode);
        if let Some(j) = self.pos_force.as_ref().map(|p| p.joint) { self.base_modes[j] = DamiaoControlMode::PosForce; }
        self
    }

    /// One MIT / feedback mapping range for every joint. It must equal the motors'
    /// PMAX / VMAX / TMAX registers; gravity compensation reads them back and refuses
    /// to start otherwise.
    pub fn with_limits(mut self, limits: DamiaoLimits) -> Self {
        self.limits.fill(limits);
        self.mit_limits_verified.fill(false);
        self.range_check = None;
        self
    }

    /// Per-joint mapping ranges, one per motor ID in order.
    pub fn with_joint_limits(mut self, limits: Vec<DamiaoLimits>) -> Result<Self, ArmDriverError> {
        if limits.len() != self.motor_ids.len() {
            return Err(ArmDriverError::JointCountMismatch { expected: self.motor_ids.len(), actual: limits.len() });
        }
        self.limits = limits;
        self.mit_limits_verified.fill(false);
        self.range_check = None;
        Ok(self)
    }

    /// Read CTRL_MODE, PMAX, VMAX and TMAX (registers 10, 21-23) from every motor and
    /// compare the ranges with the configured ones. Register reads do not change the
    /// enable state or the targets, so this is safe with the motors disabled; it sends
    /// no enable frame. A mismatch makes `set_torque(true)` refuse to arm. A motor that
    /// does not answer leaves the check `Unverified`: the configured ranges are used
    /// and gravity compensation stays refused. A joint whose CTRL_MODE is not the base
    /// mode is marked unconfirmed, so the enable handshake writes the base mode first.
    pub fn check_mapping_ranges(&mut self) -> &DriverRangeCheck {
        const RIDS: [u8; 4] = [damiao::RID_CTRL_MODE, damiao::RID_PMAX, damiao::RID_VMAX, damiao::RID_TMAX];
        let start = self.now();
        let mut joints = Vec::with_capacity(self.motor_ids.len());
        for i in 0..self.motor_ids.len() {
            let id = self.motor_ids[i];
            let base_mode = self.base_mode(i);
            let base = Self::mode_value(base_mode);
            let pre_seq = if base_mode == DamiaoControlMode::PosForce {
                self.rx_boundary().ok()
            } else {
                None
            };
            self.register_replies.clear();
            if base_mode == DamiaoControlMode::PosForce {
                self.register_reply_log.clear();
            }
            let start_stamp = self.current_stop_stamp();
            let sent = RIDS.iter().all(|&rid| {
                self.send_with_retry(TxKind::Refresh, Some(start_stamp), None, false, |_| {
                    Some(damiao::build_read_register_frame(id, rid))
                })
                .is_ok()
            });
            let _ = self.can.flush();
            let expected: Vec<(usize, bool, u8)> = RIDS.iter().map(|&rid| (i, false, rid)).collect();
            let replies = if sent { self.await_register_replies_for(&expected, RANGE_CHECK_REPLY_TIMEOUT) } else { vec![None; RIDS.len()] };
            let l = self.limits[i];
            let configured = [l.p_max, l.v_max, l.t_max];
            let ctrl_mode = replies[0].map(|r| r.as_u32());
            let reported = match (replies[1], replies[2], replies[3]) {
                (Some(p), Some(v), Some(t)) => Some([p.as_f32(), v.as_f32(), t.as_f32()]),
                _ => None,
            };
            let status = match reported {
                None => RangeCheckStatus::Unverified,
                Some(r) if Self::ranges_match(&r, &configured) => RangeCheckStatus::Passed,
                Some(_) => RangeCheckStatus::Mismatch,
            };
            self.mit_limits_verified[i] = status == RangeCheckStatus::Passed;
            if base_mode == DamiaoControlMode::PosForce {
                let expected_id = self.expected_reply_id(i);
                let verified_ctrl = pre_seq.and_then(|pre| {
                    self.register_reply_log.iter().rev()
                        .find(|(r, fid, _, seq, _)| *fid == expected_id && r.motor_id == id && !r.write && r.rid == damiao::RID_CTRL_MODE && *seq > pre)
                        .map(|(r, ..)| r.as_u32())
                });
                if verified_ctrl == Some(base) {
                    self.joint_modes[i] = Some(DamiaoControlMode::PosForce);
                } else if let Some(m) = verified_ctrl.or(ctrl_mode) {
                    if m != base {
                        tracing::warn!("Damiao joint {} (CAN ID 0x{:X}) is in CTRL_MODE {}; the base mode {} is restored before enable",
                            i + 1, id, m, base);
                    }
                    self.joint_modes[i] = None;
                }
            } else if ctrl_mode.is_some_and(|m| m != base) {
                tracing::warn!("Damiao joint {} (CAN ID 0x{:X}) is in CTRL_MODE {}; the base mode {} is restored before enable",
                    i + 1, id, ctrl_mode.unwrap_or(0), base);
                self.joint_modes[i] = None;
            }
            joints.push(JointRangeReading { joint: i + 1, motor_id: id, status, ctrl_mode, reported, configured });
        }
        let fmt = |v: &[f32; 3]| format!("{:.3}/{:.3}/{:.3}", v[0], v[1], v[2]);
        let mismatched: Vec<String> = joints.iter().filter(|j| j.status == RangeCheckStatus::Mismatch)
            .map(|j| format!("J{} reports PMAX/VMAX/TMAX {} (configured {})", j.joint, fmt(&j.reported.unwrap_or_default()), fmt(&j.configured))).collect();
        let silent: Vec<usize> = joints.iter().filter(|j| j.status == RangeCheckStatus::Unverified).map(|j| j.joint).collect();
        let duration_ms = self.since(start).as_millis() as u64;
        let (status, message) = if !mismatched.is_empty() {
            let m = format!("Damiao mapping range mismatch: {}; arming refused until damiao_ranges matches the motors", mismatched.join("; "));
            tracing::error!("{m}");
            (RangeCheckStatus::Mismatch, m)
        } else if !silent.is_empty() {
            let m = format!("Damiao joints {silent:?} did not answer the range read; using the configured ranges unverified");
            tracing::warn!("{m}");
            (RangeCheckStatus::Unverified, m)
        } else {
            let m = format!("Damiao mapping ranges of {} motors match the configuration", joints.len());
            tracing::info!("{m} ({duration_ms} ms)");
            (RangeCheckStatus::Passed, m)
        };
        self.range_check.insert(DriverRangeCheck { status, message, joints, duration_ms })
    }

    fn ranges_match(got: &[f32; 3], want: &[f32; 3]) -> bool {
        got.iter().zip(want).all(|(g, w)| g.is_finite() && (g - w).abs() <= 0.01 + w.abs() * 1e-3)
    }

    /// Mapping range used for each joint.
    pub fn joint_limits(&self) -> &[DamiaoLimits] {
        &self.limits
    }

    /// Confirmed control mode of each joint (None = unconfirmed).
    pub fn joint_control_modes(&self) -> &[Option<DamiaoControlMode>] {
        &self.joint_modes
    }

    fn mode_value(mode: DamiaoControlMode) -> u32 {
        match mode {
            DamiaoControlMode::Mit => damiao::CTRL_MODE_MIT,
            DamiaoControlMode::PosVel => damiao::CTRL_MODE_POS_VEL,
            DamiaoControlMode::PosForce => damiao::CTRL_MODE_POS_FORCE,
        }
    }

    fn vlim(&self) -> f32 { self.default_vlim.abs().clamp(0.1, 4.0) }

    fn pos_vel_frame(&self, i: usize, position: f32) -> CanFrame {
        damiao::build_pos_vel_frame(0x100, self.motor_ids[i], position, self.vlim())
    }

    fn resolved_mit_params(&self, target: &JointTarget) -> (f32, f32, f32) {
        (
            target.kp.filter(|k| k.is_finite()).unwrap_or(self.default_kp),
            target.kd.filter(|k| k.is_finite()).unwrap_or(self.default_kd),
            target.torque.filter(|t| t.is_finite()).unwrap_or(0.0),
        )
    }

    fn record_sent_target(&mut self, i: usize, frame: CanFrame, mit_params: Option<(f32, f32, f32)>) {
        self.last_target_frames[i] = Some(frame);
        self.last_mit_params[i] = if frame.id == u32::from(self.motor_ids[i]) {
            mit_params
        } else {
            None
        };
    }

    fn clear_cached_targets(&mut self) {
        self.last_target_frames.fill(None);
        self.last_mit_params.fill(None);
    }

    fn mit_frame(&self, i: usize, target: &JointTarget) -> CanFrame {
        let (kp, kd, torque) = self.resolved_mit_params(target);
        damiao::build_mit_frame(
            self.motor_ids[i],
            target.position,
            target.velocity.filter(|v| v.is_finite()).unwrap_or(0.0),
            kp,
            kd,
            torque,
            &self.limits[i],
        )
    }

    /// Fresh, finite measured position of joint `i` with a provable receive-age bound:
    /// both the decode instant and the prior observed-empty receive-queue boundary must
    /// exist and be within `max_age` (samples queued before the first empty queue have
    /// Measured position of joint `i` ONLY when both the receive instant AND the observed-empty
    /// receive-queue bound are within `max_age` and the cached position is finite (unbounded
    /// samples have unknown age and return `None`).
    fn fresh_measured_position_at(&self, i: usize, max_age: Duration, now: Instant) -> Option<f32> {
        let fresh_rx = self.last_feedback_instant.get(i).copied().flatten()
            .is_some_and(|t| now.saturating_duration_since(t) <= max_age);
        let fresh_bound = self.feedback_queue_bound.get(i).copied().flatten()
            .is_some_and(|b| now.saturating_duration_since(b) <= max_age);
        if !(fresh_rx && fresh_bound) { return None; }
        self.cached_states.get(i).map(|s| s.position).filter(|p| p.is_finite())
    }

    fn fresh_measured_position(&self, i: usize, max_age: Duration) -> Option<f32> {
        self.fresh_measured_position_at(i, max_age, self.now())
    }

    /// Joint index of the active, verified `PosForce` gripper (if configured and currently in `PosForce`).
    #[inline]
    fn active_pos_force_joint(&self) -> Option<usize> {
        let pf = self.pos_force.as_ref()?;
        (self.pos_force_verified
            && self.joint_modes.get(pf.joint).copied().flatten() == Some(DamiaoControlMode::PosForce))
            .then_some(pf.joint)
    }

    /// Single freshness authority for the `PosForce` gripper: inclusive `<= 100 ms` on BOTH
    /// `last_feedback_instant[joint]` AND `feedback_queue_bound[joint]`, plus finite position.
    #[inline]
    fn pos_force_feedback_valid(&self, joint: usize, now: Instant) -> bool {
        self.fresh_measured_position_at(joint, POS_FORCE_FEEDBACK_MAX_AGE, now).is_some()
    }

    /// Diagnostic staleness age for `joint` at `now`: max of receive age and receive-queue-bound
    /// age ONLY when both timestamps exist and the cached position is finite; otherwise `None` so
    /// non-finite or unbounded samples fall back to the episode elapsed timer cleanly.
    fn pos_force_stale_age(&self, joint: usize, now: Instant) -> Option<Duration> {
        if !self.cached_states.get(joint).is_some_and(|s| s.position.is_finite()) {
            return None;
        }
        let rx_age = self
            .last_feedback_instant
            .get(joint)
            .copied()
            .flatten()
            .map(|t| now.saturating_duration_since(t))?;
        let qb_age = self
            .feedback_queue_bound
            .get(joint)
            .copied()
            .flatten()
            .map(|b| now.saturating_duration_since(b))?;
        Some(rx_age.max(qb_age))
    }

    /// Returns `Some(pf.joint)` iff `frame` is a Damiao `0x300 + motor_id` PosForce command frame
    /// targeting the configured `PosForce` gripper joint.
    #[inline]
    fn frame_pos_force_joint(&self, frame: &CanFrame) -> Option<usize> {
        let pf = self.pos_force.as_ref()?;
        let motor_id = *self.motor_ids.get(pf.joint)?;
        let expected_id = u32::from(damiao::POS_FORCE_BASE_ID) + u32::from(motor_id);
        (frame.id == expected_id).then_some(pf.joint)
    }

    /// Pure parameter computation `(position, vel_raw = 0, i_des)` for the Damiao `0x307` FREEZE
    /// (or boot-fallback) frame of joint `i` — performs NO `CanFrame` construction.
    /// - When at least one provably fresh (`<= 100 ms` at decode time) J7 sample has been observed
    ///   since boot (`pf_last_fresh_position == Some(pos)`): `(pos, 0, pf.current_raw)`.
    /// - When NO provably fresh J7 sample has ever been observed since boot
    ///   (`pf_last_fresh_position == None`): `(last_cmd_pos_or_0, 0, pos_force_boot_fallback_current(...))`.
    fn pos_force_freeze_params(&self, i: usize) -> Option<(f32, u16, u16)> {
        let pf = self.pos_force.as_ref().filter(|pf| pf.joint == i)?;
        let motor_id = *self.motor_ids.get(i)?;
        match self.pf_last_fresh_position.filter(|p| p.is_finite()) {
            Some(pos) => Some((pos, 0, pf.current_raw)),
            None => {
                let expected_id = u32::from(damiao::POS_FORCE_BASE_ID) + u32::from(motor_id);
                let last_cmd = self
                    .last_target_frames
                    .get(i)
                    .and_then(|f| f.as_ref())
                    .filter(|f| f.id == expected_id && f.dlc == 8);
                let last_cmd_pos = last_cmd
                    .map(|f| f32::from_le_bytes([f.data[0], f.data[1], f.data[2], f.data[3]]))
                    .filter(|p| p.is_finite())
                    .unwrap_or(0.0);
                let last_cmd_cur = last_cmd
                    .map(|f| u16::from_le_bytes([f.data[6], f.data[7]]))
                    .filter(|&c| c > 0);
                let boot_cur = pos_force_boot_fallback_current(last_cmd_cur, pf.current_raw);
                Some((last_cmd_pos, 0, boot_cur))
            }
        }
    }

    /// True iff `frame` matches bit-for-bit the expected `0x307` FREEZE (or boot-fallback) payload
    /// produced by [`Self::pos_force_freeze_params`] for the configured `PosForce` joint.
    /// Guarantees stale moving targets (`vel_raw != 0` or any new target position) can never
    /// bypass `send()`'s freshness check while freeze frames always reach the wire.
    fn is_pos_force_freeze_frame(&self, frame: &CanFrame) -> bool {
        let Some(pf) = self.pos_force.as_ref() else {
            return false;
        };
        let Some(&motor_id) = self.motor_ids.get(pf.joint) else {
            return false;
        };
        let expected_id = u32::from(damiao::POS_FORCE_BASE_ID) + u32::from(motor_id);
        if frame.id != expected_id || frame.dlc != 8 {
            return false;
        }
        let Some((pos, vel_raw, cur_raw)) = self.pos_force_freeze_params(pf.joint) else {
            return false;
        };
        frame.data[0..4] == pos.to_le_bytes()
            && frame.data[4..6] == vel_raw.to_le_bytes()
            && frame.data[6..8] == cur_raw.to_le_bytes()
    }

    /// SINGLE `0x307` (`POS_FORCE_BASE_ID + motor_id`) frame constructor and transmitter in
    /// `DamiaoArmDriver` — the ONLY production call site of `damiao::build_pos_force_frame` in
    /// `src/`. Every `0x307` wire frame (control-tick epilogue on both Legacy & RT Executor,
    /// direct-driver writes, pre-enable hold, mode-switch hold, standalone recovery) goes through
    /// this ONE function.
    ///
    /// Decision table from `(intent, freshness)`:
    /// - `fresh + Target { position, velocity, current_raw }` -> capped target frame (`vel_raw >= 1`,
    ///   `i_des <= pf.current_raw`; fails closed if non-finite, `vel_raw == 0`, or `i_des == 0`).
    /// - `fresh + Hold | Stop` -> stationary hold at current fresh measured J7 position,
    ///   `vel_raw = 0`, `i_des = pf.current_raw`.
    /// - `stale` (`allow_freeze == true`) + `pf_last_fresh_position == Some(pos)` -> FREEZE frame
    ///   (`pos`, `vel_raw = 0`, `i_des = pf.current_raw`).
    /// - `stale` (`allow_freeze == true`) + `pf_last_fresh_position == None` -> documented boot
    ///   fallback via [`POS_FORCE_BOOT_FALLBACK`] / [`pos_force_boot_fallback_current`].
    /// - `stale` (`allow_freeze == false`, e.g. pre-enable / mode-switch / standalone recovery) ->
    ///   fails closed (`BuildRefused` / `PosForceFeedbackExpired`), emitting 0 frames.
    fn emit_pos_force_frame(
        &mut self,
        kind: TxKind,
        expected_stamp: Option<StopStamp>,
        intent: PosForceIntentKind,
        allow_freeze: bool,
        retry: bool,
        fenced: bool,
    ) -> Result<CanFrame, TxRefusal> {
        let build = |d: &Self| -> Option<CanFrame> {
            let pf = d.pos_force.as_ref()?;
            let i = pf.joint;
            let motor_id = *d.motor_ids.get(i)?;
            let now = d.now();
            let (pos, vel_raw, cur_raw) = if d.pos_force_feedback_valid(i, now) {
                match intent {
                    PosForceIntentKind::Target { position, velocity, current_raw } => {
                        if !position.is_finite() || velocity.is_some_and(|v| !v.is_finite()) {
                            return None;
                        }
                        let measured = d.fresh_measured_position_at(i, POS_FORCE_FEEDBACK_MAX_AGE, now)?;
                        let requested = velocity
                            .filter(|v| *v != 0.0)
                            .map_or(d.vlim(), f32::abs)
                            .min(d.vlim());
                        let strictly_opening =
                            (position - measured) * (pf.closed_rad - pf.open_rad) < 0.0;
                        let speed = if strictly_opening {
                            requested
                        } else {
                            requested.min(pf.close_speed_rad_s)
                        };
                        let vel_raw = damiao::pos_force_vel_raw(speed);
                        if vel_raw == 0 {
                            return None;
                        }
                        let cur = current_raw.map_or(pf.current_raw, |c| c.min(pf.current_raw));
                        if cur == 0 {
                            return None;
                        }
                        (position, vel_raw, cur)
                    }
                    PosForceIntentKind::Hold | PosForceIntentKind::Stop => {
                        let measured = d.fresh_measured_position_at(i, POS_FORCE_FEEDBACK_MAX_AGE, now)?;
                        (measured, 0u16, pf.current_raw)
                    }
                }
            } else if allow_freeze {
                d.pos_force_freeze_params(i)?
            } else {
                return None;
            };
            Some(damiao::build_pos_force_frame(
                damiao::POS_FORCE_BASE_ID,
                motor_id,
                pos,
                vel_raw,
                cur_raw,
            ))
        };
        if retry {
            self.send_with_retry(kind, expected_stamp, None, fenced, build)
        } else {
            self.send(kind, expected_stamp, None, fenced, build)
        }
    }

    /// Control-tick J7 `PosForce` (`0x307`) emitter: called ONCE per control tick from the tick
    /// epilogue on both owners (Legacy and RT Executor), after whatever arm-write path ran
    /// (whether that arm path succeeded, refused, tripped the MIT fence, hit `BuildRefused`, had
    /// an invalid caller target, or returned early).
    ///
    /// Always runs unfenced (`fenced = false`) with `allow_freeze = true` so a tripped MIT fence
    /// suppresses ONLY MIT arm joints and J7 (`TIMEOUT=0`) always receives its resolved frame.
    ///
    /// Single stop-generation / E-stop epoch enforcement rule: any `Target` intent whose
    /// `(stop_gen, estop_epoch)` stamp is older than `current` (differs from `current`) is
    /// degraded to `Hold` (stationary measured hold when fresh, freeze when stale) and is NEVER
    /// re-stamped with the newer generation.
    fn emit_pos_force_j7_impl(
        &mut self,
        intent: PosForceIntent,
        current: PosForceIntentStamp,
    ) -> Result<PosForceEmitOutcome, ArmDriverError> {
        let Some(pf_j) = self.active_pos_force_joint() else {
            return Ok(PosForceEmitOutcome::NotApplicable);
        };
        if !self.is_enabled || self.torque_hs.is_some() {
            return Ok(PosForceEmitOutcome::NotApplicable);
        }
        let stamp_stale = intent.stamp.stop_gen != current.stop_gen
            || intent.stamp.estop_epoch != current.estop_epoch;
        let effective_kind = match intent.kind {
            PosForceIntentKind::Target { .. } if stamp_stale => PosForceIntentKind::Hold,
            other => other,
        };
        let is_target_intent = matches!(effective_kind, PosForceIntentKind::Target { .. });
        if is_target_intent {
            self.strict_stop_check(None)?;
        } else {
            self.pending_stop_check(None)?;
        }
        let start_stamp = self.current_stop_stamp();
        let entry_now = self.now();
        let _ = self.check_pos_force_freshness_for_tx_at(entry_now);

        let mut pf_refresh_sent = false;
        if self.pf_refresh_pending
            && self.pf_refresh_deferred_ticks >= 1
            && !self.pf_refresh_attempted_this_tick
            && self.poll_tx_allowed()
        {
            pf_refresh_sent |= self.try_send_pf_refresh(start_stamp)?;
        }

        let mut active_kind = effective_kind;
        let mut retried = false;
        let sent_frame = loop {
            let kind = match active_kind {
                PosForceIntentKind::Target { .. } => TxKind::Target,
                PosForceIntentKind::Hold | PosForceIntentKind::Stop => TxKind::RollbackHold(pf_j),
            };
            match self.emit_pos_force_frame(kind, Some(start_stamp), active_kind, true, false, false) {
                Ok(frame) => break Some(frame),
                Err(TxRefusal::PosForceFeedbackExpired(_)) => {
                    let _ = self.check_pos_force_freshness_for_tx();
                    if !retried {
                        retried = true;
                        continue;
                    }
                    break None;
                }
                Err(TxRefusal::BuildRefused) => {
                    let _ = self.check_pos_force_freshness_for_tx();
                    if !retried {
                        retried = true;
                        // An invalid/unencodable target never suppresses J7: degrade to Hold.
                        if matches!(active_kind, PosForceIntentKind::Target { .. }) {
                            active_kind = PosForceIntentKind::Hold;
                        }
                        continue;
                    }
                    break None;
                }
                Err(TxRefusal::Deferred | TxRefusal::DeadlinePassed) => {
                    if !self.pos_force_feedback_valid(pf_j, self.now()) {
                        let _ = self.check_pos_force_freshness_for_tx();
                    }
                    break None;
                }
                Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                    return Err(ArmDriverError::EstopInhibited);
                }
                Err(TxRefusal::MitFenceTripped(reason)) => {
                    return Err(ArmDriverError::MitFenceTripped(reason));
                }
                Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => {
                    let stop_res = if is_target_intent {
                        self.strict_stop_check(Some(start_stamp))
                    } else {
                        self.pending_stop_check(Some(start_stamp))
                    };
                    if stop_res.is_err() {
                        return Err(ArmDriverError::EstopInhibited);
                    }
                    if !self.pos_force_feedback_valid(pf_j, self.now()) {
                        let _ = self.check_pos_force_freshness_for_tx();
                    }
                    break None;
                }
                Err(TxRefusal::Transport(e)) => {
                    self.mark_tx_pass_backpressured();
                    let stop_res = if is_target_intent {
                        self.strict_stop_check(Some(start_stamp))
                    } else {
                        self.pending_stop_check(Some(start_stamp))
                    };
                    if stop_res.is_err() {
                        return Err(ArmDriverError::EstopInhibited);
                    }
                    return Err(e.into());
                }
            }
        };

        let Some(sent_frame) = sent_frame else {
            if pf_refresh_sent {
                let stop_res = if is_target_intent {
                    self.strict_stop_check(Some(start_stamp))
                } else {
                    self.pending_stop_check(Some(start_stamp))
                };
                if stop_res.is_err() {
                    return Err(ArmDriverError::EstopInhibited);
                }
                if self.flush_cutoff_exceeded() {
                    self.mark_tx_pass_backpressured();
                    return Err(TransportError::Timeout.into());
                }
                if let Err(e) = self.can.flush() {
                    self.mark_tx_pass_backpressured();
                    return Err(e.into());
                }
            }
            if self.pf_refresh_pending && !self.pf_refresh_attempted_this_tick && self.poll_tx_allowed() {
                self.pf_refresh_deferred_ticks = self.pf_refresh_deferred_ticks.saturating_add(1);
            }
            self.pf_deferred_ticks = self.pf_deferred_ticks.saturating_add(1);
            if is_target_intent {
                self.pf_deferred_motion = true;
                self.pf_deferred_since.get_or_insert(entry_now);
                self.pf_deferred_intent = Some(intent);
            }
            let pf_status = self.take_last_pos_force_tx_status();
            return Ok(PosForceEmitOutcome::Deferred { joint: pf_j, pf_status });
        };

        let is_freeze = self.is_pos_force_freeze_frame(&sent_frame);
        if is_freeze && !matches!(self.last_tx_pf_status, PosForceTxStatus::Stale { .. }) {
            let _ = self.check_pos_force_freshness_for_tx();
        }
        self.record_sent_target(pf_j, sent_frame, None);

        if self.pf_refresh_pending
            && !self.pf_refresh_attempted_this_tick
            && self.poll_tx_allowed()
        {
            pf_refresh_sent |= self.try_send_pf_refresh(start_stamp)?;
        }
        let _ = pf_refresh_sent;

        let stop_check = |d: &Self| {
            if is_target_intent {
                d.strict_stop_check(Some(start_stamp))
            } else {
                d.pending_stop_check(Some(start_stamp))
            }
        };
        if stop_check(self).is_err() {
            return Err(ArmDriverError::EstopInhibited);
        }
        if self.flush_cutoff_exceeded() {
            self.mark_tx_pass_backpressured();
            return Err(TransportError::Timeout.into());
        }
        if let Err(e) = self.can.flush() {
            self.mark_tx_pass_backpressured();
            if stop_check(self).is_err() {
                return Err(ArmDriverError::EstopInhibited);
            }
            return Err(e.into());
        }
        if stop_check(self).is_err() {
            return Err(ArmDriverError::EstopInhibited);
        }

        self.pf_deferred_ticks = 0;
        self.pf_deferred_motion = false;
        self.pf_deferred_since = None;
        self.pf_deferred_intent = None;
        if pf_j < 64 {
            self.last_target_tx_sent_mask |= 1u64 << pf_j;
        }
        let pf_status = self.take_last_pos_force_tx_status();
        Ok(PosForceEmitOutcome::Sent { joint: pf_j, freeze: is_freeze, pf_status })
    }

    /// Single shared J7 position-force freshness gate used by the single J7 emitter.
    ///
    /// - When J7 feedback is provably fresh (`<= 100 ms`), records `Fresh` (unless already marked
    ///   `Stale` in this pass), clears `pf_refresh_pending` / `pf_refresh_deferred_ticks`, and
    ///   returns `None`.
    /// - When J7 feedback is stale (`> 100 ms` or missing queue bound), records
    ///   `Stale { joint, age, fresh_gen }`, schedules a `0xCC` refresh (`pf_refresh_pending = true`),
    ///   and returns `Some(pf_joint)`.
    fn check_pos_force_freshness_for_tx_at(&mut self, now: Instant) -> Option<usize> {
        let pf_joint = self.active_pos_force_joint()?;
        if self.pos_force_feedback_valid(pf_joint, now) {
            if !matches!(self.last_tx_pf_status, PosForceTxStatus::Stale { .. }) {
                self.last_tx_pf_status = PosForceTxStatus::Fresh;
            }
            self.pf_refresh_pending = false;
            self.pf_refresh_deferred_ticks = 0;
            return None;
        }
        let age = self.pos_force_stale_age(pf_joint, now);
        self.last_tx_pf_status = PosForceTxStatus::Stale {
            joint: pf_joint,
            age,
            fresh_gen: self.pf_fresh_gen,
        };
        self.pf_refresh_pending = true;
        Some(pf_joint)
    }

    fn check_pos_force_freshness_for_tx(&mut self) -> Option<usize> {
        let now = self.now();
        self.check_pos_force_freshness_for_tx_at(now)
    }

    /// Single refresh sender for a stale active `PosForce` joint (`0xCC` read-only status poll).
    /// Sends at most ONE `0xCC` per tick (`begin_tx_tick()`) via `send(TxKind::Refresh, ...)`.
    /// Callers own flushing when this returns `Ok(true)`.
    fn try_send_pf_refresh(&mut self, expected_stamp: StopStamp) -> Result<bool, ArmDriverError> {
        let Some(pf_joint) = self.active_pos_force_joint() else {
            self.pf_refresh_pending = false;
            self.pf_refresh_deferred_ticks = 0;
            return Ok(false);
        };
        if !self.pf_refresh_pending
            || self.pf_refresh_attempted_this_tick
            || !self.poll_tx_allowed()
        {
            return Ok(false);
        }
        self.pf_refresh_attempted_this_tick = true;
        match self.send(TxKind::Refresh, Some(expected_stamp), None, false, |d| {
            Some(damiao::build_refresh_frame(d.motor_ids[pf_joint]))
        }) {
            Ok(_) => {
                self.pf_refresh_pending = false;
                self.pf_refresh_deferred_ticks = 0;
                Ok(true)
            }
            Err(TxRefusal::Deferred) => {
                self.pf_refresh_pending = true;
                self.pf_refresh_deferred_ticks = self.pf_refresh_deferred_ticks.saturating_add(1);
                Ok(false)
            }
            Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                self.discard_all_pending_tx();
                Err(ArmDriverError::EstopInhibited)
            }
            Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => {
                if self.pending_stop_check(Some(expected_stamp)).is_err() {
                    self.discard_all_pending_tx();
                    return Err(ArmDriverError::EstopInhibited);
                }
                self.pf_refresh_pending = true;
                self.pf_refresh_deferred_ticks = self.pf_refresh_deferred_ticks.saturating_add(1);
                Ok(false)
            }
            Err(_) => {
                self.pf_refresh_pending = true;
                self.pf_refresh_deferred_ticks = self.pf_refresh_deferred_ticks.saturating_add(1);
                Ok(false)
            }
        }
    }

    /// Stationary hold target at joint `i`'s fresh (`<= 100 ms`) measured position for
    /// base-mode rollback while a stop or E-stop is active; returns `None` without fresh
    /// feedback so callers fail closed without sending a hold.
    fn fresh_rollback_hold_target_at(&self, i: usize, now: Instant) -> Option<JointTarget> {
        let pos = self.fresh_measured_position_at(i, POS_FORCE_FEEDBACK_MAX_AGE, now)?;
        if let Some(pf) = self.pos_force.as_ref().filter(|pf| pf.joint == i) {
            Some(JointTarget {
                position: pos,
                velocity: Some(pf.close_speed_rad_s),
                ..JointTarget::position(pos)
            })
        } else {
            Some(JointTarget::position(pos))
        }
    }

    fn fresh_rollback_hold_target(&self, i: usize) -> Option<JointTarget> {
        self.fresh_rollback_hold_target_at(i, self.now())
    }

    fn rollback_requires_fresh(&self, policy: ModeWritePolicy) -> bool {
        match policy {
            ModeWritePolicy::BaseRollback { start_stamp, force_fresh_hold } => {
                force_fresh_hold
                    || self.is_stop_pending()
                    || self.is_stop_latched()
                    || self.current_stop_stamp() != start_stamp
            }
            ModeWritePolicy::EnableRestore | ModeWritePolicy::Strict(_) => false,
        }
    }

    fn policy_check(&mut self, policy: ModeWritePolicy) -> Result<(), ArmDriverError> {
        match policy {
            ModeWritePolicy::EnableRestore => self.enable_estop_inhibited(),
            ModeWritePolicy::Strict(stamp) => self.strict_stop_check(Some(stamp)),
            ModeWritePolicy::BaseRollback { start_stamp, .. } => self.pending_stop_check(Some(start_stamp)),
        }
    }

    fn mode_hold_slot_count(mode: DamiaoControlMode, dual_hold: bool) -> usize {
        match mode {
            DamiaoControlMode::PosForce => 1,
            _ if dual_hold => 2,
            DamiaoControlMode::Mit | DamiaoControlMode::PosVel => 1,
        }
    }

    fn build_mode_hold_frame(
        &self,
        i: usize,
        mode: DamiaoControlMode,
        hold: &JointTarget,
        dual_hold: bool,
        slot: usize,
    ) -> Option<CanFrame> {
        self.mode_frames(i, mode, hold, dual_hold)
            .and_then(|fs| fs.get(slot).copied().flatten())
    }

    /// True when joint `i` is the configured `PosForce` joint, has provably fresh (`<= 100 ms`)
    /// measured feedback at `now`, and `target` encodes a valid finite target with `vel_raw >= 1`
    /// (NO CAN frame construction — all `0x307` frames are constructed solely in
    /// [`Self::emit_pos_force_frame`]).
    fn pos_force_target_valid_at(&self, i: usize, target: &JointTarget, now: Instant) -> bool {
        let Some(pf) = self.pos_force.as_ref().filter(|pf| pf.joint == i) else {
            return false;
        };
        if !target.position.is_finite() || target.velocity.is_some_and(|v| !v.is_finite()) {
            return false;
        }
        let Some(measured) = self.fresh_measured_position_at(i, POS_FORCE_FEEDBACK_MAX_AGE, now) else {
            return false;
        };
        let requested = target.velocity.filter(|v| *v != 0.0).map_or(self.vlim(), f32::abs).min(self.vlim());
        let strictly_opening = (target.position - measured) * (pf.closed_rad - pf.open_rad) < 0.0;
        let speed = if strictly_opening { requested } else { requested.min(pf.close_speed_rad_s) };
        damiao::pos_force_vel_raw(speed) != 0
    }

    fn pos_force_target_valid(&self, i: usize, target: &JointTarget) -> bool {
        self.pos_force_target_valid_at(i, target, self.now())
    }

    /// True if joint `i`'s mode hold slot `slot` must be emitted via the single `PosForce` emitter
    /// ([`Self::emit_pos_force_frame`]).
    #[inline]
    fn is_pos_force_mode_hold_slot(&self, i: usize, mode: DamiaoControlMode, dual: bool, slot: usize) -> bool {
        slot == 0
            && (mode == DamiaoControlMode::PosForce
                || (dual && self.base_mode(i) == DamiaoControlMode::PosForce))
    }

    /// Hold frames of non-PosForce mode slots (`PosVel` / `Mit`) on the stack without allocation.
    /// `PosForce` hold slots are validated here and emitted exclusively through
    /// [`Self::emit_pos_force_frame`].
    fn mode_frames(&self, i: usize, mode: DamiaoControlMode, hold: &JointTarget, dual: bool) -> Option<[Option<CanFrame>; 2]> {
        match mode {
            DamiaoControlMode::PosForce => {
                self.pos_force_target_valid(i, hold).then_some([None, None])
            }
            _ if dual => {
                let pos = if self.base_mode(i) == DamiaoControlMode::PosForce {
                    if !self.pos_force_target_valid(i, hold) {
                        return None;
                    }
                    None
                } else {
                    Some(self.pos_vel_frame(i, hold.position))
                };
                Some([pos, Some(self.mit_frame(i, hold))])
            }
            DamiaoControlMode::Mit => Some([Some(self.mit_frame(i, hold)), None]),
            DamiaoControlMode::PosVel => Some([Some(self.pos_vel_frame(i, hold.position)), None]),
        }
    }

    /// Validate that every position-force joint switching into `PosForce` can produce a valid
    /// capped hold frame before any `CTRL_MODE` write goes on the bus.
    fn validate_pos_force_holds(&self, joints: &[usize], modes: &[DamiaoControlMode], hold: &[JointTarget], require_when_disabled: bool) -> Result<(), ArmDriverError> {
        for &i in joints {
            if modes[i] == DamiaoControlMode::PosForce
                && (require_when_disabled || self.is_enabled)
                && !self.pos_force_target_valid(i, &hold[i])
            {
                return Err(ArmDriverError::HardwareFault(format!(
                    "joint {} position-force mode switch refused: hold target invalid or feedback stale", i + 1)));
            }
        }
        Ok(())
    }

    /// The frames of non-PosForce joint `i`'s target in its active/confirmed mode (one, or both
    /// kinds while its mode is unconfirmed), on the stack: the control tick's write does not
    /// allocate. Active `PosForce` joints are NEVER built here — they are emitted exclusively by
    /// the single J7 emitter ([`Self::emit_pos_force_j7_impl`]).
    fn target_frames_checked(
        &self,
        i: usize,
        target: &JointTarget,
    ) -> Result<([Option<CanFrame>; 2], Option<usize>), ArmDriverError> {
        if self.control_mode == DamiaoControlMode::Mit && self.base_mode(i) != DamiaoControlMode::PosForce {
            return Ok(([Some(self.mit_frame(i, target)), None], None));
        }
        match self.joint_modes[i] {
            Some(DamiaoControlMode::PosVel) => Ok(([Some(self.pos_vel_frame(i, target.position)), None], None)),
            Some(DamiaoControlMode::Mit) => Ok(([Some(self.mit_frame(i, target)), None], None)),
            Some(DamiaoControlMode::PosForce) => Ok(([None, None], Some(i))),
            None if self.base_mode(i) == DamiaoControlMode::PosForce => {
                Err(ArmDriverError::PosForceModeUnconfirmed { joint: i })
            }
            None => Ok(([Some(self.pos_vel_frame(i, target.position)), Some(self.mit_frame(i, target))], None)),
        }
    }

    /// Validate a `PosForce` target's static parameters (configured joint, finite position/velocity,
    /// nonzero wire speed encoding, nonzero current cap) WITHOUT checking live feedback freshness.
    fn validate_pos_force_target_static(&self, i: usize, target: &JointTarget) -> Result<u32, ArmDriverError> {
        let Some(pf) = self.pos_force.as_ref().filter(|pf| pf.joint == i) else {
            return Err(ArmDriverError::PosForceTargetUnavailable { joint: i });
        };
        if !target.position.is_finite() || target.velocity.is_some_and(|v| !v.is_finite()) {
            return Err(ArmDriverError::PosForceTargetNotFinite { joint: i });
        }
        let requested = target
            .velocity
            .filter(|v| *v != 0.0)
            .map_or(self.vlim(), f32::abs)
            .min(self.vlim());
        if damiao::pos_force_vel_raw(requested) == 0
            || damiao::pos_force_vel_raw(pf.close_speed_rad_s) == 0
        {
            return Err(ArmDriverError::PosForceSpeedZero { joint: i });
        }
        Ok(u32::from(damiao::POS_FORCE_BASE_ID) + u32::from(self.motor_ids[i]))
    }

    /// Unencoded target commands for non-PosForce arm joint `i` (stack-allocated, zero heap alloc)
    /// staged into [`PendingTxQueue`]. Active `PosForce` joints return `[None, None]` because
    /// they are emitted exclusively by the single J7 emitter ([`Self::emit_pos_force_j7_impl`]).
    fn pending_target_cmds_checked(
        &self,
        i: usize,
        target: &JointTarget,
    ) -> Result<[Option<(u32, PendingTargetCmd)>; 2], ArmDriverError> {
        let id = self.motor_ids[i];
        if self.control_mode == DamiaoControlMode::Mit && self.base_mode(i) != DamiaoControlMode::PosForce {
            return Ok([
                Some((
                    u32::from(id),
                    PendingTargetCmd::Mit {
                        position: target.position,
                        velocity: target.velocity,
                        kp: target.kp,
                        kd: target.kd,
                        torque: target.torque,
                    },
                )),
                None,
            ]);
        }
        match self.joint_modes[i] {
            Some(DamiaoControlMode::PosVel) => Ok([
                Some((0x100 + u32::from(id), PendingTargetCmd::PosVel { position: target.position })),
                None,
            ]),
            Some(DamiaoControlMode::Mit) => Ok([
                Some((
                    u32::from(id),
                    PendingTargetCmd::Mit {
                        position: target.position,
                        velocity: target.velocity,
                        kp: target.kp,
                        kd: target.kd,
                        torque: target.torque,
                    },
                )),
                None,
            ]),
            Some(DamiaoControlMode::PosForce) => Ok([None, None]),
            None if self.base_mode(i) == DamiaoControlMode::PosForce => {
                Err(ArmDriverError::PosForceModeUnconfirmed { joint: i })
            }
            None => Ok([
                Some((0x100 + u32::from(id), PendingTargetCmd::PosVel { position: target.position })),
                Some((
                    u32::from(id),
                    PendingTargetCmd::Mit {
                        position: target.position,
                        velocity: target.velocity,
                        kp: target.kp,
                        kd: target.kd,
                        torque: target.torque,
                    },
                )),
            ]),
        }
    }

    /// A batch is refused before its first frame when the position-force gripper's mode is not
    /// confirmed, its target position or velocity is non-finite, or its requested speed encodes
    /// to 0 on the wire. When `allow_stale_pf` is `false` (MIT-fenced paths), stale J7 feedback
    /// also refuses the entire batch before frame 0; unfenced paths (`allow_stale_pf = true`)
    /// route stale J7 through [`Self::check_pos_force_freshness_for_tx`] so J1–J6 continue.
    fn target_frames_refused(&self, targets: Option<&[JointTarget]>, allow_stale_pf: bool) -> Option<ArmDriverError> {
        let pf = self.pos_force.as_ref()?;
        let mode = self.joint_modes.get(pf.joint).copied().flatten();
        let allowed = (mode == Some(DamiaoControlMode::PosForce) && self.pos_force_verified)
            || mode == Some(DamiaoControlMode::Mit);
        if !allowed {
            return Some(ArmDriverError::PosForceModeUnconfirmed { joint: pf.joint });
        }
        if mode == Some(DamiaoControlMode::PosForce) {
            if let Some(target) = targets.and_then(|ts| ts.get(pf.joint)) {
                if let Err(err) = self.validate_pos_force_target_static(pf.joint, target) {
                    return Some(err);
                }
            }
            if !allow_stale_pf && !self.pos_force_feedback_valid(pf.joint, self.now()) {
                return Some(ArmDriverError::PosForceFeedbackStale { joint: pf.joint });
            }
        }
        None
    }

    /// Poll feedback until every `(joint, write, rid)` has a reply or the timeout passes.
    fn await_register_replies(&mut self, expected: &[(usize, bool, u8)]) -> Vec<Option<damiao::RegisterReply>> {
        self.await_register_replies_for(expected, REGISTER_REPLY_TIMEOUT)
    }

    fn await_register_replies_for(&mut self, expected: &[(usize, bool, u8)], timeout: Duration) -> Vec<Option<damiao::RegisterReply>> {
        let start = self.now();
        loop {
            let _ = self.read_joint_states();
            let found: Vec<Option<damiao::RegisterReply>> = expected.iter().map(|&(i, write, rid)| {
                let id = self.motor_ids[i];
                self.register_replies.iter().rev().copied()
                    .find(|r| r.motor_id == id && r.write == write && r.rid == rid)
            }).collect();
            if found.iter().all(Option::is_some) || self.since(start) >= timeout {
                return found;
            }
            self.pause(Duration::from_micros(500));
        }
    }

    fn refusal_to_driver_err(r: TxRefusal) -> ArmDriverError {
        match r {
            TxRefusal::DeadlinePassed => ArmDriverError::Transport(TransportError::Timeout),
            TxRefusal::Deferred => ArmDriverError::Transport(TransportError::WouldBlock),
            TxRefusal::StopInhibited | TxRefusal::StopStampChanged => ArmDriverError::EstopInhibited,
            TxRefusal::MitFenceTripped(reason) => ArmDriverError::MitFenceTripped(reason),
            TxRefusal::PosForceFeedbackExpired(joint) => {
                ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint }
            }
            TxRefusal::BuildRefused => ArmDriverError::HardwareFault("frame build refused (stale or invalid state)".into()),
            TxRefusal::Transport(e) => e.into(),
        }
    }

    /// Read PMAX / VMAX / TMAX of `joints` and require each to equal that joint's range:
    /// an MIT frame encoded with the wrong TMAX would scale every torque command.
    fn verify_mit_limits(&mut self, joints: &[usize], expected_stamp: StopStamp) -> Result<(), ArmDriverError> {
        const RIDS: [u8; 3] = [damiao::RID_PMAX, damiao::RID_VMAX, damiao::RID_TMAX];
        for &i in joints {
            self.strict_stop_check(Some(expected_stamp))?;
            self.register_replies.clear();
            for rid in RIDS {
                self.send_with_retry(TxKind::Refresh, Some(expected_stamp), None, false, |d| {
                    Some(damiao::build_read_register_frame(d.motor_ids[i], rid))
                })
                .map_err(Self::refusal_to_driver_err)?;
            }
            self.strict_stop_check(Some(expected_stamp))?;
            self.can.flush()?;
            let expected: Vec<(usize, bool, u8)> = RIDS.map(|rid| (i, false, rid)).to_vec();
            let replies = self.await_register_replies(&expected);
            let want = [self.limits[i].p_max, self.limits[i].v_max, self.limits[i].t_max];
            let got: Vec<Option<f32>> = (0..3).map(|r| replies[r].map(|reply| reply.as_f32())).collect();
            if got.iter().any(Option::is_none) {
                return Err(ArmDriverError::HardwareFault(format!(
                    "Joint {} (CAN ID 0x{:X}) did not answer the PMAX/VMAX/TMAX register read", i + 1, self.motor_ids[i])));
            }
            let got: Vec<f32> = got.into_iter().flatten().collect();
            if !Self::ranges_match(&[got[0], got[1], got[2]], &want) {
                return Err(ArmDriverError::HardwareFault(format!(
                    "Joint {} (CAN ID 0x{:X}) reports PMAX {:.3} VMAX {:.3} TMAX {:.3}; the driver maps MIT frames with {:.3} / {:.3} / {:.3}",
                    i + 1, self.motor_ids[i], got[0], got[1], got[2], want[0], want[1], want[2])));
            }
            self.mit_limits_verified[i] = true;
        }
        Ok(())
    }

    /// Write CTRL_MODE for `joints` (each followed at once by `hold` in the new mode)
    /// and wait for the write acknowledgements. Returns the joints that were not confirmed,
    /// or `Err(EstopInhibited)` when stopped by `policy`.
    ///
    /// With `strict_mode_handover` (joint tracking enabled) a joint is confirmed only when
    /// the mode write, every hold frame transmit, the bus flush and the matching register
    /// reply all succeed. Without it the matching reply alone confirms the joint, as before
    /// joint tracking (e4f72b6): hold transmit and flush errors are ignored.
    fn write_modes(
        &mut self,
        joints: &[usize],
        modes: &[DamiaoControlMode],
        hold: &[JointTarget],
        dual_hold: bool,
        policy: ModeWritePolicy,
    ) -> Result<Vec<usize>, ArmDriverError> {
        let w = match self.write_modes_send(joints, modes, hold, dual_hold, policy) {
            Ok(w) => w,
            Err(e) if is_estop_inhibited(&e) => return Err(e),
            Err(_) => return Ok(joints.to_vec()),
        };
        if w.sent_len == 0 {
            let (failed, len) = self.write_modes_finish(w);
            return Ok(failed[..len].iter().map(|&j| usize::from(j)).collect());
        }
        loop {
            let _ = self.receive_feedback();
            if self.mode_write_replied(&w) || w.deadline.passed(self.now()) {
                let (failed, len) = self.write_modes_finish(w);
                return Ok(failed[..len].iter().map(|&j| usize::from(j)).collect());
            }
            self.pause(Duration::from_micros(500));
        }
    }

    /// First half of `write_modes`: the mode writes and holds; the replies are awaited until the
    /// returned deadline (REGISTER_REPLY_TIMEOUT after the flush).
    /// Enforces `policy` via [`Self::stop_gate_check`]: forward switches and enable restores
    /// refuse every frame while a stop/E-stop is pending or latched; base-mode rollbacks while
    /// stopped only transmit with fresh (`<= 100 ms`) measured feedback, overriding holds to
    /// the fresh measured position.
    fn write_modes_send(
        &mut self,
        joints: &[usize],
        modes: &[DamiaoControlMode],
        hold: &[JointTarget],
        dual_hold: bool,
        policy: ModeWritePolicy,
    ) -> Result<ModeWrite, ArmDriverError> {
        fn restoring_pos_force<C: CanTransport>(d: &DamiaoArmDriver<C>, _i: usize, mode: DamiaoControlMode) -> bool {
            mode == DamiaoControlMode::PosForce && !d.is_enabled
        }
        if modes.len() > MAX_MODE_JOINTS || joints.len() > MAX_MODE_JOINTS {
            return Err(ArmDriverError::Protocol("joint count exceeds MAX_MODE_JOINTS".into()));
        }
        self.policy_check(policy)?;
        let strict = self.strict_mode_handover;
        self.register_replies.clear();
        self.register_reply_log.clear();
        let mut mode_arr = [DamiaoControlMode::PosVel; MAX_MODE_JOINTS];
        mode_arr[..modes.len()].copy_from_slice(modes);

        // Torque-off enable restore defers any TX-budget-limited CTRL_MODE or hold frames inline
        // in `ModeWrite` so `poll_restore` retries them across ticks until `deadline`.
        if matches!(policy, ModeWritePolicy::EnableRestore) {
            let mut deferred = [0u8; MAX_MODE_JOINTS];
            for (k, &j) in joints.iter().enumerate() {
                deferred[k] = j as u8;
            }
            let mut hold_pos = [0.0f32; MAX_MODE_JOINTS];
            for (k, t) in hold.iter().enumerate().take(MAX_MODE_JOINTS) {
                hold_pos[k] = t.position;
            }
            let mut w = ModeWrite {
                modes: mode_arr,
                sent: [0u8; MAX_MODE_JOINTS],
                sent_len: 0,
                unsent: [0u8; MAX_MODE_JOINTS],
                unsent_len: 0,
                hold_failed: [0u8; MAX_MODE_JOINTS],
                hold_failed_len: 0,
                deferred,
                deferred_stage: [0u8; MAX_MODE_JOINTS],
                deferred_len: joints.len() as u8,
                hold_pos,
                dual_hold,
                flush_ok: true,
                deadline: Deadline::after(self.now(), REGISTER_REPLY_TIMEOUT),
            };
            self.drain_restore_deferred(&mut w)?;
            return Ok(w);
        }

        let expected_stamp = match policy {
            ModeWritePolicy::EnableRestore => self.enable_stop_stamp,
            ModeWritePolicy::Strict(stamp) | ModeWritePolicy::BaseRollback { start_stamp: stamp, .. } => Some(stamp),
        };
        let is_rollback = matches!(policy, ModeWritePolicy::BaseRollback { .. });
        let mut unsent = [0u8; MAX_MODE_JOINTS];
        let mut unsent_len = 0usize;
        let mut hold_failed = [0u8; MAX_MODE_JOINTS];
        let mut hold_failed_len = 0usize;
        let mut any_tx_sent = false;
        for &i in joints {
            self.policy_check(policy)?;
            // Validate hold availability before writing CTRL_MODE so an enabled position-force
            // joint never switches mode without a valid capped hold frame, and a base-mode
            // rollback while stopped never writes CTRL_MODE without fresh measured feedback.
            let restoring_pf = restoring_pos_force(self, i, modes[i]);
            let need_fresh_rollback = !restoring_pf && self.rollback_requires_fresh(policy);
            let hold_slots = if restoring_pf {
                if self.pos_force_write_refused(&[i]).is_some() {
                    unsent[unsent_len] = i as u8;
                    unsent_len += 1;
                    continue;
                }
                0
            } else {
                let effective_hold = if need_fresh_rollback {
                    let Some(fresh_target) = self.fresh_rollback_hold_target(i) else {
                        unsent[unsent_len] = i as u8;
                        unsent_len += 1;
                        continue;
                    };
                    fresh_target
                } else {
                    hold[i].clone()
                };
                if self.mode_frames(i, modes[i], &effective_hold, dual_hold).is_none() {
                    unsent[unsent_len] = i as u8;
                    unsent_len += 1;
                    continue;
                }
                Self::mode_hold_slot_count(modes[i], dual_hold)
            };
            let id = self.motor_ids[i];
            let is_pf_joint = self.base_mode(i) == DamiaoControlMode::PosForce;
            let needs_pf_hold = !restoring_pf
                && (modes[i] == DamiaoControlMode::PosForce
                    || (dual_hold && is_pf_joint));
            let needed = 1 + hold_slots;
            let _ = self.ensure_tx_pass();
            if !self.can_start_tick_tx_n(needed) {
                unsent[unsent_len] = i as u8;
                unsent_len += 1;
                continue;
            }
            let ctrl_kind = TxKind::ModeWrite;
            let hold_kind = if is_rollback { TxKind::RollbackHold(i) } else { TxKind::Hold };
            match self.send_with_retry(ctrl_kind, expected_stamp, None, false, |d| {
                if restoring_pf {
                    if d.pos_force_write_refused(&[i]).is_some() {
                        return None;
                    }
                } else {
                    let rb_fresh = d.rollback_requires_fresh(policy);
                    if rb_fresh && d.fresh_rollback_hold_target(i).is_none() {
                        return None;
                    }
                    if needs_pf_hold {
                        let pf_hold = if rb_fresh {
                            d.fresh_rollback_hold_target(i)?
                        } else {
                            hold[i].clone()
                        };
                        if !d.pos_force_target_valid(i, &pf_hold) {
                            return None;
                        }
                    }
                }
                Some(damiao::build_set_control_mode_frame(id, Self::mode_value(modes[i])))
            }) {
                Ok(_) => {
                    any_tx_sent = true;
                }
                Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                    if any_tx_sent {
                        for &j in joints { self.joint_modes[j] = None; }
                    }
                    return Err(ArmDriverError::EstopInhibited);
                }
                Err(_) => {
                    unsent[unsent_len] = i as u8;
                    unsent_len += 1;
                    continue;
                }
            }
            // The hold frame directly follows the mode write on the bus, rebuilt fresh at send time
            // inside `send` (never resending a cached pre-stop hold frame across attempts or retries).
            let mut all_holds_ok = true;
            for slot in 0..hold_slots {
                let hold_res = if self.is_pos_force_mode_hold_slot(i, modes[i], dual_hold, slot) {
                    let intent = if self.rollback_requires_fresh(policy) {
                        PosForceIntentKind::Hold
                    } else {
                        PosForceIntentKind::Target {
                            position: hold[i].position,
                            velocity: hold[i].velocity,
                            current_raw: None,
                        }
                    };
                    self.emit_pos_force_frame(hold_kind, expected_stamp, intent, false, true, false)
                } else {
                    self.send_with_retry(hold_kind, expected_stamp, None, false, |d| {
                        let eff_hold = if d.rollback_requires_fresh(policy) {
                            d.fresh_rollback_hold_target(i)?
                        } else {
                            hold[i].clone()
                        };
                        d.build_mode_hold_frame(i, modes[i], &eff_hold, dual_hold, slot)
                    })
                };
                match hold_res {
                    Ok(sent_frame) => {
                        let mit_params = if self.rollback_requires_fresh(policy) {
                            (self.default_kp, self.default_kd, 0.0)
                        } else {
                            self.resolved_mit_params(&hold[i])
                        };
                        self.record_sent_target(i, sent_frame, Some(mit_params));
                    }
                    Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                        for &j in joints { self.joint_modes[j] = None; }
                        return Err(ArmDriverError::EstopInhibited);
                    }
                    Err(_) => {
                        all_holds_ok = false;
                    }
                }
            }
            if !all_holds_ok
                && (strict
                    || self.base_mode(i) == DamiaoControlMode::PosForce
                    || self.rollback_requires_fresh(policy))
            {
                hold_failed[hold_failed_len] = i as u8;
                hold_failed_len += 1;
                if self.base_mode(i) == DamiaoControlMode::PosForce || self.rollback_requires_fresh(policy) {
                    self.joint_modes[i] = None;
                    self.last_target_frames[i] = None;
                    self.last_mit_params[i] = None;
                }
            }
        }
        self.policy_check(policy)?;
        let flush_ok = if any_tx_sent {
            self.can.flush().is_ok() || !strict
        } else {
            true
        };
        let mut sent = [0u8; MAX_MODE_JOINTS];
        let mut sent_len = 0usize;
        for &i in joints {
            let idx = i as u8;
            if !unsent[..unsent_len].contains(&idx) {
                sent[sent_len] = idx;
                sent_len += 1;
            }
        }
        Ok(ModeWrite {
            modes: mode_arr,
            sent,
            sent_len: sent_len as u8,
            unsent,
            unsent_len: unsent_len as u8,
            hold_failed,
            hold_failed_len: hold_failed_len as u8,
            deferred: [0u8; MAX_MODE_JOINTS],
            deferred_stage: [0u8; MAX_MODE_JOINTS],
            deferred_len: 0,
            hold_pos: [0.0; MAX_MODE_JOINTS],
            dual_hold,
            flush_ok,
            deadline: Deadline::after(self.now(), REGISTER_REPLY_TIMEOUT),
        })
    }

    fn push_mode_joint_unique(buf: &mut [u8; MAX_MODE_JOINTS], len: &mut u8, val: u8) {
        let n = usize::from(*len);
        if !buf[..n].contains(&val) && n < MAX_MODE_JOINTS {
            buf[n] = val;
            *len = (n + 1) as u8;
        }
    }

    fn pop_deferred_front(w: &mut ModeWrite) {
        let n = usize::from(w.deferred_len);
        if n == 0 { return; }
        for k in 1..n {
            w.deferred[k - 1] = w.deferred[k];
            w.deferred_stage[k - 1] = w.deferred_stage[k];
        }
        w.deferred_len = (n - 1) as u8;
    }

    /// Transmit as many queued torque-off enable-restore frames (`CTRL_MODE` writes and holds)
    /// as `w.deadline` and the active per-tick TX budget allow, routing every attempt and retry
    /// through [`Self::send`] with `Some(w.deadline)` and deferring remaining frames inline in
    /// `w` for subsequent `poll_restore` ticks (zero heap allocation).
    fn drain_restore_deferred(&mut self, w: &mut ModeWrite) -> Result<(), ArmDriverError> {
        let strict = self.strict_mode_handover;
        let expected_stamp = self.enable_stop_stamp;
        let deadline = w.deadline;
        let mut sent_any = false;
        while w.deferred_len > 0 {
            let raw_i = w.deferred[0];
            let i = usize::from(raw_i);
            let stage = w.deferred_stage[0];
            let mode = w.modes[i];
            let restoring_pf = mode == DamiaoControlMode::PosForce && !self.is_enabled;
            let id = self.motor_ids[i];
            if stage == 0 {
                match self.send_with_retry(TxKind::Handshake, expected_stamp, Some(deadline), false, |d| {
                    if restoring_pf {
                        if d.pos_force_write_refused(&[i]).is_some() {
                            return None;
                        }
                    } else {
                        let hold = JointTarget::position(w.hold_pos[i]);
                        d.mode_frames(i, mode, &hold, w.dual_hold)?;
                    }
                    Some(damiao::build_set_control_mode_frame(id, Self::mode_value(mode)))
                }) {
                    Ok(_) => {
                        sent_any = true;
                        Self::push_mode_joint_unique(&mut w.sent, &mut w.sent_len, raw_i);
                        if restoring_pf {
                            Self::pop_deferred_front(w);
                        } else {
                            w.deferred_stage[0] = 1;
                        }
                    }
                    Err(TxRefusal::DeadlinePassed | TxRefusal::Deferred) => break,
                    Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                        self.abort_enable_for_estop();
                        return Err(ArmDriverError::EstopInhibited);
                    }
                    Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => break,
                    Err(_) => {
                        Self::push_mode_joint_unique(&mut w.unsent, &mut w.unsent_len, raw_i);
                        Self::pop_deferred_front(w);
                    }
                }
            } else {
                let slot = usize::from(stage - 1);
                if slot >= Self::mode_hold_slot_count(mode, w.dual_hold) {
                    Self::pop_deferred_front(w);
                    continue;
                }
                match self.send_with_retry(TxKind::Handshake, expected_stamp, Some(deadline), false, |d| {
                    let hold = JointTarget::position(w.hold_pos[i]);
                    d.build_mode_hold_frame(i, mode, &hold, w.dual_hold, slot)
                }) {
                    Ok(sent_frame) => {
                        sent_any = true;
                        let mit_params = (self.default_kp, self.default_kd, 0.0);
                        self.record_sent_target(i, sent_frame, Some(mit_params));
                        if stage == 1 && w.dual_hold && mode != DamiaoControlMode::PosForce {
                            w.deferred_stage[0] = 2;
                        } else {
                            Self::pop_deferred_front(w);
                        }
                    }
                    Err(TxRefusal::DeadlinePassed | TxRefusal::Deferred) => break,
                    Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                        self.abort_enable_for_estop();
                        return Err(ArmDriverError::EstopInhibited);
                    }
                    Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => break,
                    Err(_) => {
                        if strict {
                            Self::push_mode_joint_unique(&mut w.hold_failed, &mut w.hold_failed_len, raw_i);
                        }
                        Self::pop_deferred_front(w);
                    }
                }
            }
        }
        if sent_any {
            self.enable_estop_inhibited()?;
            w.flush_ok &= self.can.flush().is_ok() || !strict;
        }
        Ok(())
    }

    fn mode_write_reply(&self, i: usize) -> Option<damiao::RegisterReply> {
        let id = self.motor_ids[i];
        if self.base_mode(i) == DamiaoControlMode::PosForce {
            let expected_id = self.expected_reply_id(i);
            self.register_reply_log.iter().rev()
                .find(|(r, fid, ..)| *fid == expected_id && r.motor_id == id && r.write && r.rid == damiao::RID_CTRL_MODE)
                .map(|(r, ..)| *r)
        } else {
            self.register_replies.iter().rev().copied().find(|r| r.motor_id == id && r.write && r.rid == damiao::RID_CTRL_MODE)
        }
    }

    /// Every sent mode write has a register reply.
    fn mode_write_replied(&self, w: &ModeWrite) -> bool {
        w.sent[..usize::from(w.sent_len)].iter().all(|&i| self.mode_write_reply(usize::from(i)).is_some())
    }

    /// Second half of `write_modes`: confirm the joints whose reply carries the written mode.
    /// Returns `(failed_joints, len)` inline without heap allocation.
    fn write_modes_finish(&mut self, w: ModeWrite) -> ([u8; MAX_MODE_JOINTS], usize) {
        let strict = self.strict_mode_handover;
        let mut failed = w.unsent;
        let mut failed_len = usize::from(w.unsent_len);
        let hold_failed = &w.hold_failed[..usize::from(w.hold_failed_len)];
        for &raw_i in &w.sent[..usize::from(w.sent_len)] {
            let i = usize::from(raw_i);
            if w.flush_ok
                && !hold_failed.contains(&raw_i)
                && self.mode_write_reply(i).is_some_and(|r| r.as_u32() == Self::mode_value(w.modes[i]))
            {
                self.joint_modes[i] = Some(w.modes[i]);
            } else {
                if strict || self.base_mode(i) == DamiaoControlMode::PosForce {
                    self.joint_modes[i] = None;
                }
                if failed_len < MAX_MODE_JOINTS {
                    failed[failed_len] = raw_i;
                    failed_len += 1;
                }
            }
        }
        (failed, failed_len)
    }

    /// Receive boundary of an MIT transaction, established before its requests go out: read until
    /// the receive queue is observed empty (bounded). Every frame dequeued afterwards (RX sequence
    /// number larger than the returned one) was enqueued after the boundary, so frames queued
    /// before the transaction never count, and a reply arriving right after its request does.
    /// Pure RX-only drain (no autonomous recovery/refresh TX, no `begin_tx_tick`, zero heap
    /// allocation). Returns `true` when the transport explicitly reported an empty RX queue.
    fn drain_rx_frames_only(&mut self, deadline: Option<Deadline>, max_frames: usize) -> bool {
        let mut frames = 0usize;
        while frames < max_frames {
            if deadline.is_some_and(|d| d.passed(self.now())) {
                return false;
            }
            let attempt = self.now();
            match self.can.receive_timestamped() {
                Ok((frame, rx_ts)) => {
                    frames += 1;
                    self.process_rx_frame(&frame, false, rx_ts);
                }
                Err(TransportError::Timeout) | Err(TransportError::WouldBlock) => {
                    self.mark_rx_empty(attempt);
                    return true;
                }
                Err(_) => return false,
            }
        }
        false
    }

    /// Transmit a read-only `0xCC` status refresh probe on an empty software queue through the
    /// single `send()` chokepoint, flush, and check for fresh post-probe feedback.
    /// Zero heap allocations when `blocking == false` (RT executor path).
    fn probe_tx_ready_inner(
        &mut self,
        deadline: Option<Deadline>,
        blocking: bool,
    ) -> Result<bool, ArmDriverError> {
        if !blocking && (!self.can.supports_nonblocking_rt() || !self.can.supports_bounded_flush()) {
            return Err(ArmDriverError::Protocol(
                "UNSUPPORTED: transport does not support non-blocking RT I/O and bounded flush".into(),
            ));
        }
        let pending = self.total_pending_tx_len() as usize;
        if pending > 0 {
            self.tx_probe = None;
            return Err(ArmDriverError::TxDeferred { pending });
        }
        if self.motor_ids.is_empty() {
            return Ok(true);
        }
        let start_stamp = self.current_stop_stamp();
        self.pending_stop_check(Some(start_stamp))?;
        if let Some((_, at)) = self.mit_fence_trip {
            if self.since(at) < self.mit_fence_holdoff {
                return Err(ArmDriverError::Transport(TransportError::WouldBlock));
            }
        }
        let release_cap_passed = |d: &Self| d.tx_release_deadline.is_some_and(|cap| d.now() >= cap);
        if deadline.is_some_and(|d| d.passed(self.now())) || release_cap_passed(self) {
            return Err(ArmDriverError::Transport(TransportError::Timeout));
        }
        // Pre-drain pre-existing queued RX backlog before writing the probe.
        let mut drained = self.drain_rx_frames_only(deadline, 64);
        if !drained && blocking {
            for _ in 0..7 {
                drained = self.drain_rx_frames_only(deadline, 64);
                if drained {
                    break;
                }
            }
        }
        if !drained {
            return Err(ArmDriverError::Transport(TransportError::Timeout));
        }
        self.pending_stop_check(Some(start_stamp))?;
        if deadline.is_some_and(|d| d.passed(self.now())) || release_cap_passed(self) {
            return Err(ArmDriverError::Transport(TransportError::Timeout));
        }
        self.tx_probe = None;
        // Preserve any active `self.tx_release_deadline` cap installed by the release owner so
        // the clear-fault probe uses the exact same `release_deadline - publish_margin` cutoff
        // as every other TX in the release.
        self.begin_tx_tick();
        let probe_joint = 0usize;
        match self.send(TxKind::Refresh, Some(start_stamp), deadline, false, |d| {
            Some(damiao::build_refresh_frame(d.motor_ids[probe_joint]))
        }) {
            Ok(_) => {}
            Err(TxRefusal::DeadlinePassed) => {
                return Err(ArmDriverError::Transport(TransportError::Timeout));
            }
            Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                return Err(ArmDriverError::EstopInhibited);
            }
            Err(TxRefusal::Deferred) => {
                if release_cap_passed(self) {
                    return Err(ArmDriverError::Transport(TransportError::Timeout));
                }
                return Err(ArmDriverError::TxDeferred { pending: 1 });
            }
            Err(TxRefusal::Transport(e)) => {
                return Err(ArmDriverError::Transport(e));
            }
            Err(r) => {
                return Err(Self::refusal_to_driver_err(r));
            }
        }
        self.pending_stop_check(Some(start_stamp))?;
        if deadline.is_some_and(|d| d.passed(self.now())) || self.flush_cutoff_exceeded() {
            return Err(ArmDriverError::Transport(TransportError::Timeout));
        }
        if let Err(e) = self.can.flush() {
            self.mark_tx_pass_backpressured();
            return Err(ArmDriverError::Transport(e));
        }
        // Snapshot the empty-queue RX sequence boundary strictly AFTER the probe write and flush
        // return successfully, draining any pre-probe frames that arrived between the pre-drain
        // and the probe write. Because Linux SocketCAN supplies no per-frame hardware arrival
        // timestamps (`CanTransport::receive_timestamped` returns `None`), this post-write empty
        // sequence boundary is the causal proof: only feedback decoded strictly after it
        // (`last_feedback_seq[probe_joint] > boundary_seq`) can satisfy `tx_probe_confirmed()`.
        let mut post_drained = self.drain_rx_frames_only(deadline, 64);
        if !post_drained && blocking {
            for _ in 0..7 {
                post_drained = self.drain_rx_frames_only(deadline, 64);
                if post_drained {
                    break;
                }
            }
        }
        if !post_drained {
            return Err(ArmDriverError::Transport(TransportError::Timeout));
        }
        let boundary_seq = self.rx_empty_seq;
        let sent_at = self.now();
        self.tx_probe = Some(TxProbeState {
            joint: probe_joint,
            boundary_seq,
            sent_at,
        });
        if blocking {
            loop {
                let _ = self.drain_rx_frames_only(deadline, 64);
                if self.tx_probe_confirmed() {
                    return Ok(true);
                }
                if self.since(sent_at) >= REGISTER_REPLY_TIMEOUT
                    || deadline.is_some_and(|d| d.passed(self.now()))
                {
                    return Ok(false);
                }
                self.pause(Duration::from_micros(500));
            }
        } else {
            let _ = self.drain_rx_frames_only(deadline, 64);
            Ok(self.tx_probe_confirmed())
        }
    }

    fn rx_boundary(&mut self) -> Result<u64, ArmDriverError> {
        for _ in 0..8 {
            let _ = self.read_joint_states();
            if self.last_read_drained { return Ok(self.rx_empty_seq); }
        }
        Err(ArmDriverError::HardwareFault("receive queue not drained: no boundary for the register transaction".into()))
    }

    /// Bounded reply wait (MIT transactions): poll until every `(joint, write, rid)` has a reply
    /// dequeued after the boundary `pre`, or REGISTER_REPLY_TIMEOUT passes.
    fn await_replies_after(&mut self, expected: &[(usize, bool, u8)], sent: u64) {
        let start = self.now();
        loop {
            let _ = self.read_joint_states();
            let all = expected.iter().all(|&(i, write, rid)| {
                let id = self.motor_ids[i];
                self.register_reply_log.iter().any(|(r, _, _, seq, _)| r.motor_id == id && r.write == write && r.rid == rid && *seq > sent)
            });
            if all || self.since(start) >= REGISTER_REPLY_TIMEOUT { return; }
            self.pause(Duration::from_micros(500));
        }
    }

    fn now(&self) -> Instant { self.clock.as_ref().map_or_else(Instant::now, |c| c.now()) }
    fn since(&self, t: Instant) -> Duration { self.now().saturating_duration_since(t) }
    fn pause(&self, d: Duration) {
        match &self.clock { Some(c) => c.sleep(d), None => std::thread::sleep(d) }
    }

    fn trip_fence(&mut self, reason: MitFenceTripReason) {
        if self.mit_fence_trip.is_none() {
            self.mit_fence_trip = Some((reason, self.now()));
        }
        // nothing cached or queued may be replayed: the group's outputs are uncertain
        self.clear_cached_targets();
        self.refresh_unfinished = 0;
        self.discard_all_pending_tx();
    }

    /// Armed fence: every joint's feedback and the last complete batch younger than the deadline
    /// (zero allocation).
    fn fence_check(&mut self) -> Result<(), ArmDriverError> {
        if let Some((reason, _)) = self.mit_fence_trip {
            return Err(ArmDriverError::MitFenceTripped(reason));
        }
        let Some(deadline) = self.mit_fence else { return Ok(()) };
        let now = self.now();
        // one comparison for every check: elapsed >= deadline trips
        let past = |d: Duration| d >= deadline;
        if let Some(i) = (0..self.motor_ids.len()).find(|&i| self.last_feedback_instant[i].is_none_or(|t| past(now.saturating_duration_since(t)))) {
            let age = self.last_feedback_instant[i].map(|t| now.saturating_duration_since(t));
            self.trip_fence(MitFenceTripReason::FeedbackStale { joint: (i + 1) as u8, age, deadline });
        } else if let Some(gap) = self.mit_fence_last_cmd.iter().flatten().min().map(|t| now.saturating_duration_since(*t)).filter(|g| past(*g)) {
            self.trip_fence(MitFenceTripReason::OutputGap { gap, deadline });
        }
        match self.mit_fence_trip {
            Some((reason, _)) => Err(ArmDriverError::MitFenceTripped(reason)),
            None => Ok(()),
        }
    }

    /// Recovery writes (cached MIT replay, POS_VEL hold at the measured pose): never while the
    /// fence is armed or tripped.
    fn recovery_tx_allowed(&self) -> bool { self.mit_fence.is_none() && self.mit_fence_trip.is_none() }

    /// Read-only status polls: never while the fence is armed; after a trip only once the
    /// operator cleared it and its hold-off has passed (whether a poll refreshes the motor
    /// watchdog is not verified, so elapsed time alone never resumes them).
    fn poll_tx_allowed(&self) -> bool {
        match &self.mit_fence_trip {
            Some((_, at)) => self.mit_fence_acked && self.since(*at) >= self.mit_fence_holdoff,
            None => self.mit_fence.is_none(),
        }
    }

    /// Drain queued arm (non-PosForce) motion frames within the active per-tick TX pass
    /// (zero heap allocation). Leaves [`Self::pos_force_reserved_slots`] available in the budget
    /// whenever J7 was deferred on the previous tick so J7 never starves under tight budgets.
    fn drain_pending_pass(
        &mut self,
        pass_stop_stamp: StopStamp,
        pre_dropped_stale: usize,
        pre_oldest_expired: Option<Duration>,
        stop_on_expiry: bool,
    ) -> Result<TxBatchOutcome, ArmDriverError> {
        let _ = self.ensure_tx_pass();
        self.last_target_tx_sent_mask = 0;
        let mut sent_any = false;
        let mut sent_mask: u64 = 0;
        let mut dropped_stale_in_pass = 0usize;
        let mut oldest_expired = pre_oldest_expired;
        while let Some(entry) = self.pending_tx.front() {
            if !self.can_start_arm_tx() {
                break;
            }
            let entry_deadline = Deadline::after(entry.queued_at, self.tx_budget.frame_max_age);
            match self.send(
                TxKind::Target,
                Some(entry.stop_stamp),
                Some(entry_deadline),
                false,
                |d| d.build_pending_target_frame(&entry),
            ) {
                Ok(sent_frame) => {
                    let _ = self.pending_tx.pop_front();
                    let mit_params = match entry.cmd {
                        PendingTargetCmd::Mit { kp, kd, torque, .. } => Some((
                            kp.filter(|k| k.is_finite()).unwrap_or(self.default_kp),
                            kd.filter(|k| k.is_finite()).unwrap_or(self.default_kd),
                            torque.filter(|t| t.is_finite()).unwrap_or(0.0),
                        )),
                        PendingTargetCmd::PosVel { .. }
                        | PendingTargetCmd::RollbackHold { .. } => None,
                    };
                    self.record_sent_target(entry.joint, sent_frame, mit_params);
                    sent_any = true;
                    if entry.joint < 64 {
                        sent_mask |= 1u64 << entry.joint;
                    }
                }
                Err(TxRefusal::PosForceFeedbackExpired(_)) => {
                    let _ = self.pending_tx.pop_front();
                    continue;
                }
                Err(TxRefusal::DeadlinePassed) => {
                    let now = self.now();
                    let _ = self.pending_tx.pop_front();
                    let age = now.saturating_duration_since(entry.queued_at);
                    oldest_expired = Some(oldest_expired.map_or(age, |o| o.max(age)));
                    dropped_stale_in_pass += 1;
                    crate::diagnostics::tick::record_tx_dropped_stale(1);
                    if stop_on_expiry {
                        break;
                    }
                }
                Err(TxRefusal::Deferred) => break,
                Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                    self.discard_pending_tx_stopped();
                    return Err(ArmDriverError::EstopInhibited);
                }
                Err(TxRefusal::BuildRefused) => {
                    self.discard_pending_tx_stopped();
                    return Err(ArmDriverError::PosForceTargetUnavailable { joint: entry.joint });
                }
                Err(TxRefusal::MitFenceTripped(reason)) => {
                    self.discard_pending_tx_stopped();
                    return Err(ArmDriverError::MitFenceTripped(reason));
                }
                Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => {
                    if let Err(estop) = self.strict_stop_check(Some(pass_stop_stamp)) {
                        self.discard_pending_tx_stopped();
                        return Err(estop);
                    }
                    break;
                }
                Err(TxRefusal::Transport(e)) => {
                    self.mark_tx_pass_backpressured();
                    self.discard_pending_tx_stopped();
                    return Err(e.into());
                }
            }
        }
        // Pre-flush stop check: a stop/E-stop arriving during the final frame transmit skips flush.
        if let Err(e) = self.strict_stop_check(Some(pass_stop_stamp)) {
            self.discard_pending_tx_stopped();
            return Err(e);
        }
        if sent_any {
            if self.flush_cutoff_exceeded() {
                self.mark_tx_pass_backpressured();
                self.discard_pending_tx_stopped();
                return Err(TransportError::Timeout.into());
            }
            if let Err(e) = self.can.flush() {
                self.mark_tx_pass_backpressured();
                self.discard_pending_tx_stopped();
                return Err(e.into());
            }
        }
        // Post-flush stop check: catches pending, latched, or already-applied stops raised during flush.
        if let Err(e) = self.strict_stop_check(Some(pass_stop_stamp)) {
            self.discard_pending_tx_stopped();
            return Err(e);
        }
        // Record per-joint wire progress only after flush and post-flush stop checks succeed,
        // excluding any joint that still has an unsent frame remaining in `pending_tx`.
        let mut completed_mask = sent_mask;
        for entry in self.pending_tx.buf[..self.pending_tx.len].iter().flatten() {
            if entry.joint < 64 {
                completed_mask &= !(1u64 << entry.joint);
            }
        }
        self.last_target_tx_sent_mask = completed_mask;
        // Recheck freshness AFTER flush so frames expiring during send or flush never carry over.
        let post_now = self.now();
        let (late_stale, late_oldest) = self.pending_tx.drop_stale(post_now, self.tx_budget.frame_max_age);
        if late_stale > 0 {
            dropped_stale_in_pass += late_stale;
            if let Some(age) = late_oldest {
                oldest_expired = Some(oldest_expired.map_or(age, |o| o.max(age)));
            }
            crate::diagnostics::tick::record_tx_dropped_stale(late_stale as u32);
        }
        let total_dropped_stale = pre_dropped_stale.saturating_add(dropped_stale_in_pass);
        if stop_on_expiry && total_dropped_stale > 0 {
            self.discard_pending_tx_stopped();
            self.last_expired_oldest_age = oldest_expired;
            return Ok(TxBatchOutcome::Expired { dropped: total_dropped_stale, pending: 0 });
        }
        let newly_deferred = self.pending_tx.mark_newly_deferred();
        let pending = self.pending_tx.len();
        crate::diagnostics::tick::record_tx_deferred(newly_deferred as u32, self.total_pending_tx_len());
        if total_dropped_stale > 0 {
            self.last_expired_oldest_age = oldest_expired;
            Ok(TxBatchOutcome::Expired { dropped: total_dropped_stale, pending })
        } else if pending == 0 {
            self.last_expired_oldest_age = None;
            Ok(TxBatchOutcome::Complete)
        } else {
            self.last_expired_oldest_age = None;
            Ok(TxBatchOutcome::Deferred { pending })
        }
    }

    /// Drain carried-over pending target frames within the active tick's TX budget (no new targets staged).
    pub fn drain_pending_tx(&mut self) -> Result<TxBatchOutcome, ArmDriverError> {
        self.last_expired_oldest_age = None;
        self.last_tx_pf_status = PosForceTxStatus::Unobserved;
        if self.torque_hs.is_some() {
            self.reject_target_tx(MitFenceTripReason::PreflightRejected);
            return Err(ArmDriverError::TorqueTransitionPending);
        }
        if self.mit_fence.is_some() || self.mit_fence_trip.is_some() {
            self.reject_target_tx(MitFenceTripReason::PreflightRejected);
            return Err(ArmDriverError::PendingTxDisabledUnderMitFence);
        }
        if let Some(e) = self.target_frames_refused(None, true) {
            self.reject_target_tx(MitFenceTripReason::PreflightRejected);
            return Err(e);
        }
        if let Err(e) = self.strict_stop_check(None) {
            self.discard_pending_tx_stopped();
            return Err(e);
        }
        self.output_ran_since_rx = true;
        let current_stamp = self.current_stop_stamp();
        if self.pending_tx.has_stopped_entry(current_stamp) {
            self.discard_pending_tx_stopped();
            return Err(ArmDriverError::EstopInhibited);
        }
        let cleanup_now = self.now();
        let (stale, pre_oldest) = self.pending_tx.drop_stale(cleanup_now, self.tx_budget.frame_max_age);
        if stale > 0 {
            crate::diagnostics::tick::record_tx_dropped_stale(stale as u32);
        }
        let arm_out = self.drain_pending_pass(current_stamp, stale, pre_oldest, false)?;
        if self.external_pos_force_emitter || !self.pf_deferred_motion {
            return Ok(arm_out);
        }
        let cur_intent_stamp = self.current_j7_intent_stamp();
        let intent = self.pf_deferred_intent.unwrap_or_else(|| PosForceIntent::hold(cur_intent_stamp));
        match self.emit_pos_force_j7_impl(intent, cur_intent_stamp)? {
            PosForceEmitOutcome::NotApplicable => Ok(arm_out),
            PosForceEmitOutcome::Sent { pf_status, .. } => {
                self.last_tx_pf_status = pf_status;
                Ok(arm_out)
            }
            PosForceEmitOutcome::Deferred { pf_status, .. } => {
                self.last_tx_pf_status = pf_status;
                Ok(match arm_out {
                    TxBatchOutcome::Complete => TxBatchOutcome::Deferred { pending: 1 },
                    TxBatchOutcome::Deferred { pending } => TxBatchOutcome::Deferred { pending: pending + 1 },
                    TxBatchOutcome::Expired { dropped, pending } => TxBatchOutcome::Expired { dropped, pending: pending + 1 },
                })
            }
        }
    }

    /// Read u32 / f32 registers `rids` of joint `i`: requests sent after the log is cleared,
    /// replies matched by motor ID, read opcode and RID, newer than the request. Returns
    /// (value, reply CAN ID) per RID.
    fn read_registers_fresh(
        &mut self,
        i: usize,
        rids: &[u8],
        expected_stamp: StopStamp,
        allow_latched: bool,
    ) -> Result<Vec<([u8; 4], u32)>, ArmDriverError> {
        if allow_latched {
            self.pending_stop_check(Some(expected_stamp))?;
        } else {
            self.strict_stop_check(Some(expected_stamp))?;
        }
        let id = self.motor_ids[i];
        let sent = self.rx_boundary()?;
        if allow_latched {
            self.pending_stop_check(Some(expected_stamp))?;
        } else {
            self.strict_stop_check(Some(expected_stamp))?;
        }
        self.register_replies.clear();
        self.register_reply_log.clear();
        for &rid in rids {
            if !allow_latched {
                self.strict_stop_check(Some(expected_stamp))?;
            }
            self.send_with_retry(TxKind::Refresh, Some(expected_stamp), None, false, |_| {
                Some(damiao::build_read_register_frame(id, rid))
            })
            .map_err(Self::refusal_to_driver_err)?;
        }
        if allow_latched {
            self.pending_stop_check(Some(expected_stamp))?;
        } else {
            self.strict_stop_check(Some(expected_stamp))?;
        }
        self.can.flush()?;
        let expected: Vec<(usize, bool, u8)> = rids.iter().map(|&rid| (i, false, rid)).collect();
        self.await_replies_after(&expected, sent);
        rids.iter().map(|&rid| {
            self.register_reply_log.iter().rev()
                .find(|(r, _, _, seq, _)| r.motor_id == id && !r.write && r.rid == rid && *seq > sent)
                .map(|(r, fid, _, _, _)| (r.value, *fid))
                .ok_or_else(|| ArmDriverError::HardwareFault(format!("Joint {} (CAN ID 0x{id:X}) did not answer the read of register {rid}", i + 1)))
        }).collect()
    }

    fn identity_of(&mut self, i: usize) -> Result<crate::mit::MotorIdentity, ArmDriverError> {
        const RIDS: [u8; 8] = [damiao::RID_ESC_ID, damiao::RID_MST_ID, damiao::RID_SW_VER, damiao::RID_CTRL_MODE,
            damiao::RID_PMAX, damiao::RID_VMAX, damiao::RID_TMAX, damiao::RID_TIMEOUT];
        let start_stamp = self.current_stop_stamp();
        let v = self.read_registers_fresh(i, &RIDS, start_stamp, true)?;
        let reply_id = v[0].1;
        if let Some((_, other)) = v.iter().find(|(_, fid)| *fid != reply_id) {
            return Err(ArmDriverError::HardwareFault(format!("Joint {}: register replies arrived on CAN IDs 0x{reply_id:X} and 0x{other:X}", i + 1)));
        }
        let u = |k: usize| u32::from_le_bytes(v[k].0);
        let f = |k: usize| f32::from_le_bytes(v[k].0);
        Ok(crate::mit::MotorIdentity { joint: i + 1, motor_id: self.motor_ids[i], reply_id, esc_id: u(0), mst_id: u(1), sw_ver: u(2),
            ctrl_mode: u(3), pmax: f(4), vmax: f(5), tmax: f(6), timeout_raw: u(7) })
    }

    /// MIT-path mode write: the CTRL_MODE writes and holds of `joints`, then a matching
    /// acknowledgement and fresh feedback per joint, each received after the writes went out (enqueue
    /// bound after the send mark) and the feedback after the acknowledgement in RX order; a
    /// read-only status poll asks once for missing feedback. Every hold transmit and the flush must
    /// succeed. Returns the joints not confirmed (their mode becomes unknown) and why.
    fn write_modes_confirmed(
        &mut self,
        joints: &[usize],
        modes: &[DamiaoControlMode],
        hold: &[JointTarget],
        dual_hold: bool,
        policy: ModeWritePolicy,
    ) -> Result<(Vec<usize>, &'static str), ArmDriverError> {
        let fenced = self.mit_fence.is_some() || self.mit_fence_trip.is_some();
        let total_joints = self.motor_ids.len().min(u8::MAX as usize) as u8;
        let expected_stamp = match policy {
            ModeWritePolicy::EnableRestore => self.enable_stop_stamp,
            ModeWritePolicy::Strict(stamp) | ModeWritePolicy::BaseRollback { start_stamp: stamp, .. } => Some(stamp),
        };
        let is_rollback = matches!(policy, ModeWritePolicy::BaseRollback { .. });
        if let Err(e) = self.policy_check(policy) {
            if fenced {
                self.trip_fence(MitFenceTripReason::EstopBeforeBatch { total_joints });
            }
            for &i in joints { self.joint_modes[i] = None; }
            return Err(e);
        }
        let Ok(sent) = self.rx_boundary() else {
            for &i in joints { self.joint_modes[i] = None; }
            return Ok((joints.to_vec(), "receive boundary (queue not drained)"));
        };
        if let Err(e) = self.policy_check(policy) {
            if fenced {
                self.trip_fence(MitFenceTripReason::EstopBeforeBatch { total_joints });
            }
            for &i in joints { self.joint_modes[i] = None; }
            return Err(e);
        }
        self.register_replies.clear();
        self.register_reply_log.clear();
        let mut failed: Vec<usize> = Vec::new();
        let mut hold_started: Vec<(usize, Instant)> = Vec::new();
        let mut any_tx_sent = false;
        for (completed, &i) in joints.iter().enumerate() {
            if let Err(e) = self.policy_check(policy) {
                if fenced {
                    self.trip_fence(MitFenceTripReason::EstopDuringBatch {
                        completed_joints: completed as u8,
                        total_joints,
                    });
                }
                for &j in joints { self.joint_modes[j] = None; }
                return Err(e);
            }
            let effective_hold = if self.rollback_requires_fresh(policy) {
                let Some(fresh_target) = self.fresh_rollback_hold_target(i) else {
                    failed.push(i);
                    continue;
                };
                fresh_target
            } else {
                hold[i].clone()
            };
            if self.mode_frames(i, modes[i], &effective_hold, dual_hold).is_none() {
                failed.push(i);
                continue;
            }
            let id = self.motor_ids[i];
            let is_pf_joint = self.base_mode(i) == DamiaoControlMode::PosForce;
            let needs_pf_hold = modes[i] == DamiaoControlMode::PosForce
                || (dual_hold && is_pf_joint);
            let hold_slots = Self::mode_hold_slot_count(modes[i], dual_hold);
            let needed = 1 + hold_slots;
            let _ = self.ensure_tx_pass();
            if !self.can_start_tick_tx_n(needed) {
                failed.push(i);
                continue;
            }
            let ctrl_kind = TxKind::ModeWrite;
            let hold_kind = if is_rollback { TxKind::RollbackHold(i) } else { TxKind::Hold };
            match self.send_with_retry(ctrl_kind, expected_stamp, None, fenced, |d| {
                let rb_fresh = d.rollback_requires_fresh(policy);
                if rb_fresh && d.fresh_rollback_hold_target(i).is_none() {
                    return None;
                }
                if needs_pf_hold {
                    let pf_hold = if rb_fresh {
                        d.fresh_rollback_hold_target(i)?
                    } else {
                        hold[i].clone()
                    };
                    if !d.pos_force_target_valid(i, &pf_hold) {
                        return None;
                    }
                }
                Some(damiao::build_set_control_mode_frame(id, Self::mode_value(modes[i])))
            }) {
                Ok(_) => {
                    any_tx_sent = true;
                }
                Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                    if fenced {
                        self.trip_fence(MitFenceTripReason::EstopDuringBatch {
                            completed_joints: completed as u8,
                            total_joints,
                        });
                    }
                    for &j in joints { self.joint_modes[j] = None; }
                    return Err(ArmDriverError::EstopInhibited);
                }
                Err(_) => {
                    failed.push(i);
                    continue;
                }
            }
            hold_started.push((i, self.now()));
            for slot in 0..hold_slots {
                let hold_res = if self.is_pos_force_mode_hold_slot(i, modes[i], dual_hold, slot) {
                    let intent = if self.rollback_requires_fresh(policy) {
                        PosForceIntentKind::Hold
                    } else {
                        PosForceIntentKind::Target {
                            position: hold[i].position,
                            velocity: hold[i].velocity,
                            current_raw: None,
                        }
                    };
                    self.emit_pos_force_frame(hold_kind, expected_stamp, intent, false, true, fenced)
                } else {
                    self.send_with_retry(hold_kind, expected_stamp, None, fenced, |d| {
                        let eff_hold = if d.rollback_requires_fresh(policy) {
                            d.fresh_rollback_hold_target(i)?
                        } else {
                            hold[i].clone()
                        };
                        d.build_mode_hold_frame(i, modes[i], &eff_hold, dual_hold, slot)
                    })
                };
                match hold_res {
                    Ok(sent_frame) => {
                        let mit_params = if self.rollback_requires_fresh(policy) {
                            (self.default_kp, self.default_kd, 0.0)
                        } else {
                            self.resolved_mit_params(&hold[i])
                        };
                        self.record_sent_target(i, sent_frame, Some(mit_params));
                    }
                    Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                        if fenced {
                            self.trip_fence(MitFenceTripReason::EstopDuringBatch {
                                completed_joints: completed as u8,
                                total_joints,
                            });
                        }
                        for &j in joints { self.joint_modes[j] = None; }
                        return Err(ArmDriverError::EstopInhibited);
                    }
                    Err(_) => {
                        if !failed.contains(&i) { failed.push(i); }
                        if self.base_mode(i) == DamiaoControlMode::PosForce || self.rollback_requires_fresh(policy) {
                            self.joint_modes[i] = None;
                            self.last_target_frames[i] = None;
                            self.last_mit_params[i] = None;
                        }
                    }
                }
            }
        }
        if let Err(e) = self.policy_check(policy) {
            if fenced {
                self.trip_fence(MitFenceTripReason::EstopBeforeFlush { total_joints });
            }
            for &j in joints { self.joint_modes[j] = None; }
            return Err(e);
        }
        let mut why = "mode write, hold or flush";
        if any_tx_sent && self.can.flush().is_err() { failed = joints.to_vec(); }
        if let Err(e) = self.policy_check(policy) {
            if fenced {
                self.trip_fence(MitFenceTripReason::EstopAfterFlush { total_joints });
            }
            for &j in joints { self.joint_modes[j] = None; }
            return Err(e);
        }
        if self.mit_fence.is_some() || self.mit_fence_trip.is_some() {
            // holds completing past the deadline trip it: the switch fails, nothing more is sent
            if self.fence_check().is_err() {
                for &i in joints { self.joint_modes[i] = None; }
                return Ok((joints.to_vec(), "deadline fence (holds completed past it)"));
            }
            if failed.is_empty() {
                // each switched joint's hold is its complete command, dated by its first frame
                for (i, t) in hold_started { self.mit_fence_last_cmd[i] = Some(t); }
            }
        }
        let pending: Vec<usize> = joints.iter().copied().filter(|i| !failed.contains(i)).collect();
        let expected: Vec<(usize, bool, u8)> = pending.iter().map(|&i| (i, true, damiao::RID_CTRL_MODE)).collect();
        if !expected.is_empty() {
            self.await_replies_after(&expected, sent);
        }
        let ack_seq: Vec<Option<u64>> = pending.iter().map(|&i| {
            let (id, want) = (self.motor_ids[i], Self::mode_value(modes[i]));
            let expected_id = self.expected_reply_id(i);
            let check_id = self.base_mode(i) == DamiaoControlMode::PosForce;
            self.register_reply_log.iter().rev()
                .find(|(r, fid, _, seq, _)| (!check_id || *fid == expected_id) && r.motor_id == id && r.write && r.rid == damiao::RID_CTRL_MODE && *seq > sent && r.as_u32() == want)
                .map(|(_, _, _, seq, _)| *seq)
        }).collect();
        if ack_seq.iter().any(Option::is_none) { why = "matching acknowledgement received after the mode write"; }
        let unconfirmed = |d: &Self| -> Vec<usize> {
            pending.iter().zip(&ack_seq)
                .filter(|(&i, ack)| !ack.is_some_and(|a| d.last_feedback_seq[i] > a))
                .map(|(&i, _)| i).collect()
        };
        let confirm_deadline = Deadline::after(self.now(), REGISTER_REPLY_TIMEOUT);
        let mut polled = false;
        // a joint without a matching acknowledgement fails at once (nothing to wait for), so the
        // rollback starts within the deadline
        let acks_complete = ack_seq.iter().all(Option::is_some);
        loop {
            let missing = unconfirmed(self);
            if missing.is_empty() || !acks_complete || confirm_deadline.passed(self.now()) { break; }
            if !polled && ack_seq.iter().all(Option::is_some) {
                if let Err(e) = self.policy_check(policy) {
                    if fenced {
                        self.trip_fence(MitFenceTripReason::EstopAfterFlush { total_joints });
                    }
                    for &j in joints { self.joint_modes[j] = None; }
                    return Err(e);
                }
                let mut refresh_sent = false;
                for &i in &missing {
                    if !is_rollback && self.is_stop_latched() {
                        if fenced {
                            self.trip_fence(MitFenceTripReason::EstopAfterFlush { total_joints });
                        }
                        for &j in joints { self.joint_modes[j] = None; }
                        return Err(ArmDriverError::EstopInhibited);
                    }
                    match self.send_with_retry(
                        TxKind::Refresh,
                        expected_stamp,
                        Some(confirm_deadline),
                        fenced,
                        |d| Some(damiao::build_refresh_frame(d.motor_ids[i])),
                    ) {
                        Ok(_) => {
                            refresh_sent = true;
                        }
                        Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                            if fenced {
                                self.trip_fence(MitFenceTripReason::EstopAfterFlush { total_joints });
                            }
                            for &j in joints { self.joint_modes[j] = None; }
                            return Err(ArmDriverError::EstopInhibited);
                        }
                        Err(_) => break,
                    }
                }
                if refresh_sent {
                    if let Err(e) = self.policy_check(policy) {
                        if fenced {
                            self.trip_fence(MitFenceTripReason::EstopAfterFlush { total_joints });
                        }
                        for &j in joints { self.joint_modes[j] = None; }
                        return Err(e);
                    }
                    let _ = self.can.flush();
                }
                polled = true;
            }
            self.pause(Duration::from_micros(500));
            let _ = self.read_joint_states();
        }
        let missing = unconfirmed(self);
        if !missing.is_empty() && why == "mode write, hold or flush" { why = "fresh feedback after the mode acknowledgement"; }
        failed.extend(missing);
        for &i in joints {
            self.joint_modes[i] = if failed.contains(&i) { None } else { Some(modes[i]) };
        }
        Ok((failed, why))
    }

    /// `set_joint_modes_confirmed`: the strict switch, then fresh feedback from every switched
    /// motor received after its mode acknowledgement (RX order, so samples queued before the
    /// switch never count; a read-only status poll asks for it once). Anything missing switches
    /// every changed joint back with dual holds; a rollback that is not confirmed leaves the
    /// joint uncertain (`joint_modes_uncertain`).
    fn switch_joint_modes_confirmed(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.discard_all_pending_tx();
        self.begin_tx_tick();
        let start_stamp = self.current_stop_stamp();
        let n = self.motor_ids.len();
        if modes.len() != n || hold.len() != n {
            return Err(ArmDriverError::JointCountMismatch { expected: n, actual: modes.len().min(hold.len()) });
        }
        if self.control_mode != DamiaoControlMode::PosVel {
            return Err(ArmDriverError::Protocol("joint mode switching needs the POS_VEL base mode".into()));
        }
        // armed fence (hand back): the deadline holds for the switch too; tripped: refused
        self.fence_check().map_err(|e| ArmDriverError::HardwareFault(format!("{e}; rearm first")))?;
        let target: Vec<DamiaoControlMode> = modes.iter().enumerate().map(|(i, m)| match m {
            JointMode::Position => self.base_mode(i),
            JointMode::Mit => DamiaoControlMode::Mit,
        }).collect();
        let changing: Vec<usize> = (0..n).filter(|&i| self.joint_modes[i] != Some(target[i])).collect();
        if changing.is_empty() { return Ok(()); }
        let fenced = self.mit_fence.is_some() || self.mit_fence_trip.is_some();
        let is_base_restore = changing.iter().all(|&i| target[i] == self.base_mode(i));
        // CTRL_MODE writes (forward and rollback alike) are refused while a stop/E-stop is pending OR latched.
        self.strict_stop_check(Some(start_stamp))?;
        let forward_policy = if !is_base_restore || fenced {
            ModeWritePolicy::Strict(start_stamp)
        } else {
            ModeWritePolicy::BaseRollback { start_stamp, force_fresh_hold: false }
        };
        let effective_hold = hold.to_vec();
        self.validate_pos_force_holds(&changing, &target, &effective_hold, true)?;
        let to_mit: Vec<usize> = changing.iter().copied()
            .filter(|&i| target[i] == DamiaoControlMode::Mit && !self.mit_limits_verified[i]).collect();
        for &i in &to_mit {
            self.strict_stop_check(Some(start_stamp))?;
            // bounded reads: PMAX / VMAX / TMAX replies received after the request
            let v = self.read_registers_fresh(i, &[damiao::RID_PMAX, damiao::RID_VMAX, damiao::RID_TMAX], start_stamp, false)?;
            let got = [f32::from_le_bytes(v[0].0), f32::from_le_bytes(v[1].0), f32::from_le_bytes(v[2].0)];
            let want = [self.limits[i].p_max, self.limits[i].v_max, self.limits[i].t_max];
            if !Self::ranges_match(&got, &want) {
                return Err(ArmDriverError::HardwareFault(format!("Joint {} reports PMAX/VMAX/TMAX {got:?}; the driver maps MIT frames with {want:?}", i + 1)));
            }
            self.mit_limits_verified[i] = true;
        }
        let previous = self.joint_modes.clone();
        let (failed, why) = self.write_modes_confirmed(&changing, &target, &effective_hold, false, forward_policy)?;
        // the confirmation wait counts against the fence too: a trip there fails the switch
        let fence_ok = !(self.mit_fence.is_some() || self.mit_fence_trip.is_some()) || self.fence_check().is_ok();
        if failed.is_empty() && fence_ok {
            tracing::info!("Damiao control mode switched and confirmed for joints {:?}", changing.iter().map(|i| i + 1).collect::<Vec<_>>());
            return Ok(());
        }
        let failed = if failed.is_empty() { changing.clone() } else { failed };
        for &i in &changing { self.joint_modes[i] = None; }
        let back: Vec<DamiaoControlMode> = (0..n).map(|i| previous[i].unwrap_or(self.base_mode(i))).collect();
        let back_is_base = changing.iter().all(|&i| back[i] == self.base_mode(i));
        // Rollback CTRL_MODE writes are also ModeWrite: refused while pending, latched, or after a stop.
        self.strict_stop_check(Some(start_stamp))?;
        let rollback_policy = if !back_is_base || fenced {
            ModeWritePolicy::Strict(start_stamp)
        } else {
            ModeWritePolicy::BaseRollback { start_stamp, force_fresh_hold: false }
        };
        let rollback_hold = hold.to_vec();
        // the rollback is confirmed like the forward switch (acknowledgement + fresh feedback)
        let (unrecovered, _) = self.write_modes_confirmed(&changing, &back, &rollback_hold, true, rollback_policy)?;
        Err(ArmDriverError::HardwareFault(format!(
            "Control-mode switch not confirmed for joints {:?} (no {why}); switched back{}",
            failed.iter().map(|i| i + 1).collect::<Vec<_>>(),
            if unrecovered.is_empty() { String::new() } else {
                format!(", joints {:?} still unconfirmed", unrecovered.iter().map(|i| i + 1).collect::<Vec<_>>())
            })))
    }

    /// One joint of `commission_motor_timeout`; `written` is set once the RAM write was sent.
    fn commission_one(&mut self, i: usize, counts: u32, written: &mut bool) -> Result<crate::mit::TimeoutCommissioning, ArmDriverError> {
        {
            let start_stamp = self.current_stop_stamp();
            self.strict_stop_check(Some(start_stamp))?;
            let id = self.motor_ids[i];
            let before = self.identity_of(i)?;
            if before.reply_id != before.mst_id {
                return Err(ArmDriverError::HardwareFault(format!("joint {}: register replies on 0x{:X} but MST_ID {}", i + 1, before.reply_id, before.mst_id)));
            }
            // RAM write (0x55), checked acknowledgement
            let sent = self.rx_boundary()?;
            self.strict_stop_check(Some(start_stamp))?;
            self.register_replies.clear();
            self.register_reply_log.clear();
            // uncertain from the moment the transport accepted the write (a write that never left,
            // WouldBlock until the retries ran out, changed nothing)
            self.send_with_retry(TxKind::ModeWrite, Some(start_stamp), None, false, |_| {
                Some(damiao::build_write_register_u32_frame(id, damiao::RID_TIMEOUT, counts))
            })
            .map_err(Self::refusal_to_driver_err)?;
            *written = true;
            self.strict_stop_check(Some(start_stamp))?;
            self.can.flush()?;
            self.await_replies_after(&[(i, true, damiao::RID_TIMEOUT)], sent);
            let ack = self.register_reply_log.iter().rev()
                .find(|(r, _, _, seq, _)| r.motor_id == id && r.write && r.rid == damiao::RID_TIMEOUT && *seq > sent).copied();
            let Some((ack, ack_id, _, _, _)) = ack else {
                return Err(ArmDriverError::HardwareFault(format!("joint {} (CAN ID 0x{id:X}) did not acknowledge the TIMEOUT write", i + 1)));
            };
            if ack_id != before.mst_id || ack.as_u32() != counts {
                return Err(ArmDriverError::HardwareFault(format!("joint {}: TIMEOUT write acknowledged with {} on 0x{ack_id:X} (wrote {counts}, MST_ID {})",
                    i + 1, ack.as_u32(), before.mst_id)));
            }
            // separate read: fresh motor ID, reply ID, opcode, RID and value
            let readback = self.identity_of(i)?;
            let same = readback.motor_id == before.motor_id && readback.reply_id == before.reply_id && readback.esc_id == before.esc_id
                && readback.mst_id == before.mst_id && readback.sw_ver == before.sw_ver;
            if !same || readback.timeout_raw != counts {
                return Err(ArmDriverError::HardwareFault(format!("joint {}: TIMEOUT readback {} (wrote {counts}) or identity changed", i + 1, readback.timeout_raw)));
            }
            tracing::warn!("Damiao joint {} (CAN ID 0x{id:X}): TIMEOUT {} -> {counts} counts (RAM)", i + 1, before.timeout_raw);
            Ok(crate::mit::TimeoutCommissioning { before, written_raw: counts, ack_raw: ack.as_u32(), readback,
                unit_us: damiao::TIMEOUT_UNIT_US_DOCUMENTED, unit_verified: false })
        }
    }

    /// A failed or abandoned enable: the arm needs a full rearm.
    fn enable_failed(&mut self) {
        self.is_enabling = false;
        self.is_enabled = false;
        self.pending_rearm = true;
        self.pos_force_verified = false;
        self.refresh_capture = None;
        self.enable_stop_stamp = None;
    }

    /// Aborts an enable handshake if E-stop is latched, an urgent stop is pending, or a
    /// stop/E-stop generation advanced since the handshake began (even after `requested == applied`).
    fn enable_estop_inhibited(&mut self) -> Result<(), ArmDriverError> {
        if let Err(e) = self.strict_stop_check(self.enable_stop_stamp) {
            self.abort_enable_for_estop();
            return Err(e);
        }
        Ok(())
    }

    /// Enable, first step: fast path, range check, receive boundary, then either the base-mode
    /// restore (`Restore`) or the enable frames (`Enable`).
    fn begin_enable(&mut self) -> Result<TorqueProgress, ArmDriverError> {
        self.begin_enable_budgeted(None)
    }

    fn begin_enable_budgeted(&mut self, max_rx_frames: Option<u16>) -> Result<TorqueProgress, ArmDriverError> {
        if self.torque_hs.is_some() {
            return Err(ArmDriverError::Protocol("a torque transition is pending".into()));
        }
        self.discard_all_pending_tx();
        if max_rx_frames.is_none() {
            self.begin_tx_tick();
        }
        self.refresh_unfinished = 0;
        self.enable_stop_stamp = None;
        self.enable_estop_inhibited()?;
        // Fast-path: only if !pending_rearm && is_enabled && feedback_valid && all state == 1
        // (never after an MIT deadline trip: the rearm restores the modes and re-enables;
        // a position-force gripper also requires verified mode 4 and fresh <= 100 ms feedback).
        if self.mit_fence_trip.is_none()
            && !self.pending_rearm
            && self.is_enabled
            && self.pos_force.as_ref().is_none_or(|pf| {
                self.pos_force_verified
                    && self.joint_modes.get(pf.joint).copied().flatten() == Some(DamiaoControlMode::PosForce)
                    && self.fresh_measured_position(pf.joint, POS_FORCE_FEEDBACK_MAX_AGE).is_some()
            })
            && self.is_feedback_valid()
            && self.motor_status.iter().all(|&st| st == 1)
        {
            return Ok(TorqueProgress::Done(TorqueOutcome::Confirmed));
        }

        // A motor that did not answer the startup range read gets another (read-only)
        // try before each enable in the legacy path; budgeted zero-RX commits fail closed
        // instead of blocking for register readback.
        if self.range_check.as_ref().is_some_and(|c| c.status == RangeCheckStatus::Unverified) {
            if max_rx_frames == Some(0) {
                self.is_enabled = false;
                self.pending_rearm = true;
                return Err(ArmDriverError::HardwareFault(
                    "Mapping range check unverified before budgeted enable".into(),
                ));
            }
            self.check_mapping_ranges();
        }
        if let Some(check) = self.range_check.as_ref().filter(|c| c.status == RangeCheckStatus::Mismatch) {
            self.is_enabled = false;
            self.pending_rearm = true;
            return Err(ArmDriverError::HardwareFault(check.message.clone()));
        }

        self.enable_stop_stamp = Some(self.current_stop_stamp());
        self.is_enabling = true;
        self.last_command_was_disable = false;
        self.disable_confirmed_sticky = false;

        let budget_armed = match max_rx_frames {
            Some(limit) if limit > 0 => {
                self.last_poll_rx = None;
                self.rx_frame_limit = Some(limit.min(64));
                true
            }
            _ => false,
        };
        let res = (|| -> Result<TorqueProgress, ArmDriverError> {
            let drained_now = match max_rx_frames {
                Some(0) => {
                    if !self.last_read_drained {
                        // Do not transmit enable frames or drain RX in Step 3; defer draining
                        // to `poll_torque_budgeted` until an empty-queue boundary is reached.
                        self.awaiting_rx_boundary = true;
                        let now = self.now();
                        self.torque_hs = Some(DamiaoTorque::DrainBeforeEnable(Deadline::after(now, DAMIAO_RX_PASS)));
                        return Ok(TorqueProgress::Pending { wake: now });
                    }
                    false
                }
                Some(limit) => {
                    if !self.drain_rx_before_command_limit(usize::from(limit)) {
                        self.awaiting_rx_boundary = true;
                        self.enable_failed();
                        return Err(ArmDriverError::HardwareFault("CAN RX buffer not drained before handshake".to_string()));
                    }
                    true
                }
                None => {
                    if !self.drain_rx_before_command() {
                        self.awaiting_rx_boundary = true;
                        self.enable_failed();
                        return Err(ArmDriverError::HardwareFault("CAN RX buffer not drained before handshake".to_string()));
                    }
                    true
                }
            };
            self.continue_enable_after_drain(drained_now)
        })();
        if budget_armed {
            self.rx_frame_limit = None;
        }
        res
    }

    fn continue_enable_after_drain(&mut self, drained_now: bool) -> Result<TorqueProgress, ArmDriverError> {
        self.enable_estop_inhibited()?;
        self.awaiting_rx_boundary = false;

        // CTRL_MODE lives in motor RAM: a joint left in MIT by an interrupted gravity
        // compensation session must be back in the base mode before it is enabled.
        let dof = self.motor_ids.len();
        if dof > MAX_MODE_JOINTS {
            self.enable_failed();
            return Err(ArmDriverError::Protocol("joint count exceeds MAX_MODE_JOINTS".into()));
        }
        let mut stray = [0usize; MAX_MODE_JOINTS];
        let mut stray_u8 = [0u8; MAX_MODE_JOINTS];
        let mut stray_len = 0usize;
        for i in 0..dof {
            if self.joint_modes[i] != Some(self.base_mode(i)) {
                stray[stray_len] = i;
                stray_u8[stray_len] = i as u8;
                stray_len += 1;
            }
        }
        let stray_slice = &stray[..stray_len];
        if let Some(j) = self.pos_force.as_ref().map(|p| p.joint).filter(|j| stray_slice.contains(j)) {
            if self.fresh_measured_position(j, POS_FORCE_FEEDBACK_MAX_AGE).is_none() {
                let refresh = match self.start_feedback_refresh(j, drained_now) {
                    Ok(r) => r,
                    Err(e) if is_estop_inhibited(&e) => return Err(e),
                    Err(e) => {
                        self.enable_failed();
                        return Err(e);
                    }
                };
                let wake = self.now().min(refresh.deadline.at);
                self.torque_hs = Some(DamiaoTorque::RefreshBeforeRestore {
                    refresh,
                    stray: stray_u8,
                    stray_len: stray_len as u8,
                });
                return Ok(TorqueProgress::Pending { wake });
            }
        }
        if let Some(e) = self.pos_force_write_refused(stray_slice) {
            self.enable_failed();
            return Err(e);
        }
        self.start_restore_or_readback(stray_slice, false)
    }

    fn start_restore_or_readback(&mut self, stray: &[usize], pf_refreshed: bool) -> Result<TorqueProgress, ArmDriverError> {
        self.enable_estop_inhibited()?;
        if !stray.is_empty() {
            let dof = self.motor_ids.len();
            if dof > MAX_MODE_JOINTS {
                self.enable_failed();
                return Err(ArmDriverError::Protocol("joint count exceeds MAX_MODE_JOINTS".into()));
            }
            let mut base = [DamiaoControlMode::PosVel; MAX_MODE_JOINTS];
            base[..dof].copy_from_slice(&self.base_modes[..dof]);
            let mut hold: [JointTarget; MAX_MODE_JOINTS] = std::array::from_fn(|_| JointTarget::position(0.0));
            for i in 0..dof {
                hold[i] = JointTarget::position(self.cached_states[i].position);
            }
            // after MIT tracking (fence armed, tripped or used since the last confirmed enable)
            // the base mode is restored with POS_VEL frames only: no MIT frame from cached
            // positions before the mode is confirmed (a new handover is the only way back)
            let dual = !self.mit_tracking_used;
            let w = self.write_modes_send(stray, &base[..dof], &hold[..dof], dual, ModeWritePolicy::EnableRestore)?;
            let wake = self.now();
            self.torque_hs = Some(DamiaoTorque::Restore { write: w, pf_refreshed });
            return Ok(TorqueProgress::Pending { wake });
        }
        self.readback_or_enable()
    }

    /// Budgeted enable pre-drain: waits under `poll_torque_budgeted`'s budget until a receive
    /// pass observes an empty RX queue, then continues into mode restore / enable TX.
    fn poll_drain_before_enable(&mut self, deadline: Deadline) -> Result<TorqueProgress, ArmDriverError> {
        self.enable_estop_inhibited()?;
        let _ = self.receive_feedback();
        self.enable_estop_inhibited()?;
        if self.last_read_drained {
            return self.continue_enable_after_drain(true);
        }
        if deadline.passed(self.now()) {
            self.awaiting_rx_boundary = true;
            self.enable_failed();
            return Err(ArmDriverError::HardwareFault("CAN RX buffer not drained before handshake".to_string()));
        }
        let wake = deadline.at;
        self.torque_hs = Some(DamiaoTorque::DrainBeforeEnable(deadline));
        Ok(TorqueProgress::Pending { wake })
    }

    /// Remaining frames allowed in the current budgeted RX pass (defaults to 64 when unbudgeted).
    fn remaining_rx_frame_limit(&self) -> usize {
        usize::from(self.rx_frame_limit.unwrap_or(64).min(64))
    }

    /// Account frames consumed by an RX pass into `last_poll_rx` and deduct them from the active
    /// `rx_frame_limit` so multiple drains within one release share a single frame budget.
    fn account_rx_pass(&mut self, frames_processed: usize, exhausted: bool) {
        let frames = u16::try_from(frames_processed).unwrap_or(u16::MAX);
        if let Some(rem) = self.rx_frame_limit.as_mut() {
            *rem = rem.saturating_sub(frames);
        }
        let (prev_frames, prev_exhausted) = self.last_poll_rx.unwrap_or((0, false));
        self.last_poll_rx = Some((
            prev_frames.saturating_add(frames),
            prev_exhausted || exhausted,
        ));
    }

    /// Drain up to the remaining `rx_frame_limit` (max 64) RX frames within `rx_over` and
    /// `deadline`, stopping on `Timeout` / `WouldBlock` and marking the empty-queue boundary.
    /// Every frame consumed counts against the release's RX budget and `last_poll_rx`.
    fn drain_rx_for_boundary(&mut self, deadline: Deadline) -> bool {
        let max_frames = self.remaining_rx_frame_limit();
        self.last_read_drained = false;
        let t0 = self.now();
        let mut frames_processed = 0usize;
        let mut rx_drained = false;
        let mut hit_err = false;
        while frames_processed < max_frames && !self.rx_over(t0) && !deadline.passed(self.now()) {
            let attempt = self.now();
            match self.can.receive_timestamped() {
                Ok((frame, rx_ts)) => {
                    frames_processed += 1;
                    self.process_rx_frame(&frame, true, rx_ts);
                }
                Err(TransportError::Timeout) | Err(TransportError::WouldBlock) => {
                    rx_drained = true;
                    self.mark_rx_empty(attempt);
                    break;
                }
                Err(_) => {
                    hit_err = true;
                    break;
                }
            }
        }
        self.account_rx_pass(frames_processed, !rx_drained && !hit_err);
        rx_drained
    }

    /// Establish a tight pre-transmit RX boundary, transmit `0xCC` for `joint` within the
    /// per-tick TX budget, and arm `refresh_capture`. Returns `Ok(None)` if RX cannot be drained
    /// or the TX budget is exhausted/backpressured before `deadline`.
    fn send_refresh_with_boundary(&mut self, joint: usize, deadline: Deadline) -> Result<Option<u64>, ArmDriverError> {
        self.enable_estop_inhibited()?;
        if !self.drain_rx_for_boundary(deadline) || deadline.passed(self.now()) {
            return Ok(None);
        }
        // Snapshot the receive sequence and transmit instant right before transmitting `0xCC`:
        // only replies with `seq > boundary` AND receive time `rx_at >= tx_at` are accepted.
        let boundary = self.rx_seq;
        let tx_at = self.now();
        let id = self.motor_ids[joint];
        match self.send(TxKind::Handshake, self.enable_stop_stamp, Some(deadline), false, |_| {
            Some(damiao::build_refresh_frame(id))
        }) {
            Ok(_) => {}
            Err(TxRefusal::DeadlinePassed | TxRefusal::Deferred) => return Ok(None),
            Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                self.abort_enable_for_estop();
                return Err(ArmDriverError::EstopInhibited);
            }
            Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => return Ok(None),
            Err(r) => return Err(Self::refusal_to_driver_err(r)),
        }
        self.enable_estop_inhibited()?;
        self.can.flush()?;
        self.refresh_capture = Some(RefreshCapture {
            joint,
            boundary,
            tx_at,
            deadline,
            first: None,
            unsafe_sample: None,
        });
        Ok(Some(boundary))
    }

    /// Start a bounded 0xCC feedback refresh for `joint`. Captures the fixed `REGISTER_REPLY_TIMEOUT`
    /// (50 ms) deadline before any drain or transmit so slow transport I/O never extends the window.
    /// Sends immediately only when `drained_now` is true; otherwise leaves `boundary: None` so
    /// `poll_feedback_refresh` establishes a fresh boundary first and never admits pre-request frames.
    fn start_feedback_refresh(&mut self, joint: usize, drained_now: bool) -> Result<FeedbackRefresh, ArmDriverError> {
        self.enable_estop_inhibited()?;
        self.refresh_capture = None;
        let start = self.now();
        let deadline = Deadline::after(start, REGISTER_REPLY_TIMEOUT);
        let boundary = if drained_now && !deadline.passed(self.now()) {
            self.send_refresh_with_boundary(joint, deadline)?
        } else {
            None
        };
        Ok(FeedbackRefresh { joint, boundary, deadline })
    }

    /// Serve a bounded 0xCC feedback refresh: accepts only when a post-boundary J7 reply
    /// arrived strictly before `r.deadline` (with `rx_at >= tx_at`, within
    /// `POS_FORCE_FEEDBACK_MAX_AGE`) AND the RX queue reached an observed empty-queue boundary
    /// (`last_read_drained`), so newer queued motion or fault replies beyond the RX budget can
    /// never be skipped. Any post-boundary unsafe J7 sample (motion or non-zero status/fault)
    /// vetoes immediately. Even at the expiry wake, runs one bounded RX drain before deciding;
    /// if the queue still has not drained at the deadline, fails closed (`TimedOut`).
    fn poll_feedback_refresh(&mut self, mut r: FeedbackRefresh) -> Result<RefreshStep, ArmDriverError> {
        self.enable_estop_inhibited()?;
        let expired_before_read = r.deadline.passed(self.now());
        match r.boundary {
            None => {
                if !expired_before_read {
                    r.boundary = self.send_refresh_with_boundary(r.joint, r.deadline)?;
                }
            }
            Some(b) => {
                let _ = self.receive_feedback();
                self.enable_estop_inhibited()?;
                let cap_state = self
                    .refresh_capture
                    .as_ref()
                    .filter(|c| c.joint == r.joint && c.boundary == b)
                    .map(|c| (c.tx_at, c.first, c.unsafe_sample));
                if let Some((tx_at, first, unsafe_sample)) = cap_state {
                    if let Some(bad) = unsafe_sample {
                        self.refresh_capture = None;
                        return Ok(RefreshStep::Ready(bad));
                    }
                    if self.last_read_drained {
                        if let Some(sample) = first {
                            let fresh_rx = self.since(sample.rx_at) <= POS_FORCE_FEEDBACK_MAX_AGE;
                            let fresh_bound = sample
                                .queue_bound
                                .is_some_and(|qb| self.since(qb) <= POS_FORCE_FEEDBACK_MAX_AGE);
                            if sample.seq > b
                                && sample.rx_at >= tx_at
                                && !r.deadline.passed(sample.rx_at)
                                && fresh_rx
                                && fresh_bound
                                && sample.position.is_finite()
                            {
                                self.refresh_capture = None;
                                // Do NOT roll back live cached_states/motor_status: return Ready(sample)
                                // and let callers enforce safety checks on BOTH `sample` and live
                                // telemetry (now up to date through the empty-queue boundary).
                                return Ok(RefreshStep::Ready(sample));
                            }
                        }
                    }
                }
            }
        }
        if r.deadline.passed(self.now()) {
            self.refresh_capture = None;
            return Ok(RefreshStep::TimedOut);
        }
        Ok(RefreshStep::Pending(r))
    }

    fn poll_refresh_before_restore(
        &mut self,
        refresh: FeedbackRefresh,
        stray_u8: [u8; MAX_MODE_JOINTS],
        stray_len: u8,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.enable_estop_inhibited()?;
        let joint = refresh.joint;
        let len = usize::from(stray_len).min(MAX_MODE_JOINTS);
        let mut stray = [0usize; MAX_MODE_JOINTS];
        for i in 0..len {
            stray[i] = usize::from(stray_u8[i]);
        }
        let stray_slice = &stray[..len];
        match self.poll_feedback_refresh(refresh) {
            Ok(RefreshStep::Ready(sample)) => {
                let sample_fresh = sample.position.is_finite()
                    && self.since(sample.rx_at) <= POS_FORCE_FEEDBACK_MAX_AGE
                    && sample.queue_bound.is_some_and(|qb| self.since(qb) <= POS_FORCE_FEEDBACK_MAX_AGE);
                if let Some(e) = self.pos_force_sample_refused(
                    joint,
                    "position-force mode not written",
                    sample_fresh,
                    sample.velocity,
                    sample.motor_status,
                ) {
                    self.enable_failed();
                    return Err(e);
                }
                if let Some(e) = self.pos_force_write_refused(stray_slice) {
                    self.enable_failed();
                    return Err(e);
                }
                self.start_restore_or_readback(stray_slice, true)
            }
            Ok(RefreshStep::Pending(r)) => {
                let wake = r.deadline.at;
                self.torque_hs = Some(DamiaoTorque::RefreshBeforeRestore {
                    refresh: r,
                    stray: stray_u8,
                    stray_len,
                });
                Ok(TorqueProgress::Pending { wake })
            }
            Ok(RefreshStep::TimedOut) => {
                let e = self.pos_force_write_refused(stray_slice).unwrap_or_else(|| ArmDriverError::HardwareFault(format!(
                    "joint {} position-force mode not written: no timely post-refresh feedback within {} ms; capped operation unavailable",
                    joint + 1,
                    REGISTER_REPLY_TIMEOUT.as_millis()
                )));
                self.enable_failed();
                Err(e)
            }
            Err(e) if is_estop_inhibited(&e) => Err(e),
            Err(e) => {
                self.enable_failed();
                Err(e)
            }
        }
    }

    /// Serve the base-mode restore: replies recorded strictly before the deadline complete ->
    /// readback / enable; at or after the deadline, no new RX frames are accepted and already-recorded
    /// timely replies are still evaluated before timing out missing joints. If the position-force
    /// gripper's feedback expired mid-restore (after earlier stray joints transmitted) and its mode
    /// was not yet written, transition once into `RefreshBeforeRestore` for that joint.
    fn poll_restore(&mut self, mut w: ModeWrite, pf_refreshed: bool) -> Result<TorqueProgress, ArmDriverError> {
        self.enable_estop_inhibited()?;
        let expired_before_read = w.deadline.passed(self.now());
        if !expired_before_read {
            let _ = self.receive_feedback();
            self.enable_estop_inhibited()?;
            if w.deferred_len > 0 && !w.deadline.passed(self.now()) {
                self.drain_restore_deferred(&mut w)?;
            }
        }
        let finished = if w.deadline.passed(self.now()) {
            self.register_reply_log.retain(|(_, _, rx_at, _, _)| !w.deadline.passed(*rx_at));
            self.register_replies.clear();
            for (rep, ..) in &self.register_reply_log {
                if self.register_replies.len() < 64 { self.register_replies.push(*rep); }
            }
            for k in 0..usize::from(w.deferred_len) {
                let raw_i = w.deferred[k];
                if w.deferred_stage[k] == 0 {
                    Self::push_mode_joint_unique(&mut w.unsent, &mut w.unsent_len, raw_i);
                } else {
                    Self::push_mode_joint_unique(&mut w.hold_failed, &mut w.hold_failed_len, raw_i);
                }
            }
            w.deferred_len = 0;
            true
        } else {
            w.deferred_len == 0 && self.mode_write_replied(&w)
        };
        let pf_unsent = self.pos_force.as_ref().map(|p| p.joint)
            .filter(|&j| w.unsent[..usize::from(w.unsent_len)].contains(&(j as u8)));
        if !finished {
            let wake = if w.deferred_len > 0 { self.now() } else { w.deadline.at };
            self.torque_hs = Some(DamiaoTorque::Restore { write: w, pf_refreshed });
            return Ok(TorqueProgress::Pending { wake });
        }
        let (failed, failed_len) = self.write_modes_finish(w);
        if failed_len > 0 {
            if !pf_refreshed
                && pf_unsent.is_some_and(|j| failed_len == 1 && usize::from(failed[0]) == j
                    && self.fresh_measured_position(j, POS_FORCE_FEEDBACK_MAX_AGE).is_none())
            {
                let j = pf_unsent.unwrap();
                let drained_now = !expired_before_read && self.last_read_drained;
                let refresh = match self.start_feedback_refresh(j, drained_now) {
                    Ok(r) => r,
                    Err(e) if is_estop_inhibited(&e) => return Err(e),
                    Err(e) => {
                        self.enable_failed();
                        return Err(e);
                    }
                };
                let mut stray = [0u8; MAX_MODE_JOINTS];
                stray[0] = j as u8;
                let wake = self.now().min(refresh.deadline.at);
                self.torque_hs = Some(DamiaoTorque::RefreshBeforeRestore {
                    refresh,
                    stray,
                    stray_len: 1,
                });
                return Ok(TorqueProgress::Pending { wake });
            }
            self.enable_failed();
            return Err(ArmDriverError::HardwareFault(format!(
                "Control mode of joints {:?} could not be restored before enable",
                failed[..failed_len].iter().map(|&i| usize::from(i) + 1).collect::<Vec<_>>())));
        }
        self.readback_or_enable()
    }

    fn pos_force_sample_refused(&self, joint: usize, label: &str, fresh: bool, velocity: f32, status: u8) -> Option<ArmDriverError> {
        let still = velocity.is_finite() && velocity.abs() <= 0.05;
        let off = !self.is_enabled && status == 0;
        if fresh && still && off {
            return None;
        }
        let fault = damiao_fault_name(status).map_or(String::new(), |name| format!(" ({name})"));
        Some(ArmDriverError::HardwareFault(format!(
            "joint {} {label}: needs fresh stationary feedback with torque off (fresh {fresh}, still {still}, torque off {off}, status {status}{fault}); capped operation unavailable",
            joint + 1
        )))
    }

    /// With torque off, the position-force gripper's mode is written only on fresh (100 ms),
    /// stationary (|v| <= 0.05 rad/s), state-0 feedback.
    fn pos_force_write_refused(&self, stray: &[usize]) -> Option<ArmDriverError> {
        let j = self.pos_force.as_ref()?.joint;
        if !stray.contains(&j) { return None; }
        let fresh = self.fresh_measured_position(j, POS_FORCE_FEEDBACK_MAX_AGE).is_some();
        self.pos_force_sample_refused(j, "position-force mode not written", fresh, self.cached_states[j].velocity, self.motor_status[j])
    }

    /// Every full enable of a position-force gripper reads its CTRL_MODE back before the enable
    /// frames (a motor power cycle resets the RAM mode).
    fn readback_or_enable(&mut self) -> Result<TorqueProgress, ArmDriverError> {
        let Some(j) = self.pos_force.as_ref().map(|p| p.joint) else { return self.send_enable() };
        self.pos_force_verified = false;
        let now = self.now();
        self.torque_hs = Some(DamiaoTorque::Readback(ModeReadback { joint: j, boundary: None, deadline: Deadline::after(now, REGISTER_REPLY_TIMEOUT) }));
        Ok(TorqueProgress::Pending { wake: now })
    }

    fn readback_failed(&mut self, j: usize, why: String) -> Result<TorqueProgress, ArmDriverError> {
        self.readback_failed_err(
            j,
            ArmDriverError::HardwareFault(format!("joint {} CTRL_MODE readback: {why}; capped operation unavailable", j + 1)),
        )
    }

    fn readback_failed_err(&mut self, j: usize, err: ArmDriverError) -> Result<TorqueProgress, ArmDriverError> {
        self.joint_modes[j] = None;
        self.pos_force_verified = false;
        self.enable_failed();
        Err(err)
    }

    /// Serve the readback: the request goes out once a receive pass observed an empty queue; only
    /// a read reply dequeued after that (expected master CAN ID, motor ID, read opcode, CTRL_MODE)
    /// and recorded strictly before the 50 ms deadline counts; value 4 enables even when processed
    /// at the advertised expiry wake.
    fn poll_readback(&mut self, mut r: ModeReadback) -> Result<TorqueProgress, ArmDriverError> {
        self.enable_estop_inhibited()?;
        let expired_before_read = r.deadline.passed(self.now());
        if !expired_before_read {
            let _ = self.receive_feedback();
            self.enable_estop_inhibited()?;
        }
        let id = self.motor_ids[r.joint];
        match r.boundary {
            None if !expired_before_read && !r.deadline.passed(self.now()) && self.last_read_drained => {
                let boundary = self.rx_empty_seq;
                self.register_reply_log.clear();
                match self.send(TxKind::Handshake, self.enable_stop_stamp, Some(r.deadline), false, |_| {
                    Some(damiao::build_read_register_frame(id, damiao::RID_CTRL_MODE))
                }) {
                    Ok(_) => {
                        self.enable_estop_inhibited()?;
                        if let Err(e) = self.can.flush() {
                            return self.readback_failed_err(
                                r.joint,
                                ArmDriverError::PosForceReadbackTxFailed {
                                    joint: r.joint,
                                    kind: TxFailKind::from_transport(&e),
                                },
                            );
                        }
                        r.boundary = Some(boundary);
                    }
                    Err(TxRefusal::DeadlinePassed | TxRefusal::Deferred) => {}
                    Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                        self.abort_enable_for_estop();
                        return Err(ArmDriverError::EstopInhibited);
                    }
                    Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => {}
                    Err(TxRefusal::Transport(e)) => {
                        return self.readback_failed_err(
                            r.joint,
                            ArmDriverError::PosForceReadbackTxFailed {
                                joint: r.joint,
                                kind: TxFailKind::from_transport(&e),
                            },
                        );
                    }
                    Err(r_err) => {
                        return self.readback_failed_err(r.joint, Self::refusal_to_driver_err(r_err));
                    }
                }
            }
            None => {}
            Some(b) => {
                let expected_id = self.expected_reply_id(r.joint);
                let reply = self.register_reply_log.iter().rev()
                    .find(|(rep, fid, rx_at, seq, _)| !r.deadline.passed(*rx_at) && *fid == expected_id && rep.motor_id == id && !rep.write && rep.rid == damiao::RID_CTRL_MODE && *seq > b)
                    .map(|(rep, ..)| rep.as_u32());
                match reply {
                    Some(v) if v == damiao::CTRL_MODE_POS_FORCE => {
                        if self.last_read_drained
                            && self.fresh_measured_position(r.joint, POS_FORCE_FEEDBACK_MAX_AGE).is_some()
                        {
                            return self.finish_pos_force_readback(r.joint, None);
                        }
                        let drained_now = !expired_before_read && self.last_read_drained;
                        let refresh = match self.start_feedback_refresh(r.joint, drained_now) {
                            Ok(ref_state) => ref_state,
                            Err(e) if is_estop_inhibited(&e) => return Err(e),
                            Err(e) => return self.readback_failed(r.joint, format!("pre-enable refresh not sent: {e}")),
                        };
                        let wake = self.now().min(refresh.deadline.at);
                        self.torque_hs = Some(DamiaoTorque::PreEnableRefresh(refresh));
                        return Ok(TorqueProgress::Pending { wake });
                    }
                    Some(v) => return self.readback_failed(r.joint, format!("reads {v}, expected {}", damiao::CTRL_MODE_POS_FORCE)),
                    None => {}
                }
            }
        }
        if r.deadline.passed(self.now()) {
            return self.readback_failed(r.joint, format!("no fresh reply within {} ms", REGISTER_REPLY_TIMEOUT.as_millis()));
        }
        let wake = r.deadline.at;
        self.torque_hs = Some(DamiaoTorque::Readback(r));
        Ok(TorqueProgress::Pending { wake })
    }

    fn poll_pre_enable_refresh(&mut self, refresh: FeedbackRefresh) -> Result<TorqueProgress, ArmDriverError> {
        self.enable_estop_inhibited()?;
        let joint = refresh.joint;
        match self.poll_feedback_refresh(refresh) {
            Ok(RefreshStep::Ready(sample)) => self.finish_pos_force_readback(joint, Some(sample)),
            Ok(RefreshStep::Pending(r)) => {
                let wake = r.deadline.at;
                self.torque_hs = Some(DamiaoTorque::PreEnableRefresh(r));
                Ok(TorqueProgress::Pending { wake })
            }
            Ok(RefreshStep::TimedOut) => {
                self.readback_failed(joint, "no fresh measured position for pre-enable hold".into())
            }
            Err(e) if is_estop_inhibited(&e) => Err(e),
            Err(e) => self.readback_failed(joint, format!("pre-enable refresh not sent: {e}")),
        }
    }

    fn finish_pos_force_readback(&mut self, joint: usize, refresh_sample: Option<RefreshSample>) -> Result<TorqueProgress, ArmDriverError> {
        self.enable_estop_inhibited()?;
        if let Some(sample) = refresh_sample {
            let sample_still = sample.velocity.is_finite() && sample.velocity.abs() <= 0.05;
            let sample_off = !self.is_enabled && sample.motor_status == 0;
            if !(sample_still && sample_off) {
                let fault = damiao_fault_name(sample.motor_status).map_or(String::new(), |name| format!(" ({name})"));
                return self.readback_failed(
                    joint,
                    format!(
                        "pre-enable hold refused: needs stationary torque-off feedback (still {sample_still}, torque off {sample_off}, status {}{fault})",
                        sample.motor_status
                    ),
                );
            }
            let sample_fresh = sample.position.is_finite()
                && self.since(sample.rx_at) <= POS_FORCE_FEEDBACK_MAX_AGE
                && sample.queue_bound.is_some_and(|qb| self.since(qb) <= POS_FORCE_FEEDBACK_MAX_AGE);
            if !sample_fresh {
                return self.readback_failed(joint, "no fresh measured position for pre-enable hold".into());
            }
        }
        // Hold at the position of the newest fresh sample that passes the live freshness,
        // stationarity, and torque-off checks below (not an older first-timely sample).
        let Some(hold_pos) = self.fresh_measured_position(joint, POS_FORCE_FEEDBACK_MAX_AGE) else {
            return self.readback_failed(joint, "no fresh measured position for pre-enable hold".into());
        };
        let still = self.cached_states[joint].velocity.is_finite() && self.cached_states[joint].velocity.abs() <= 0.05;
        let status = self.motor_status[joint];
        let off = !self.is_enabled && status == 0;
        if !(still && off) {
            let fault = damiao_fault_name(status).map_or(String::new(), |name| format!(" ({name})"));
            return self.readback_failed(
                joint,
                format!("pre-enable hold refused: needs stationary torque-off feedback (still {still}, torque off {off}, status {status}{fault})"),
            );
        }
        let Some(close_speed) = self.pos_force.as_ref().filter(|pf| pf.joint == joint).map(|pf| pf.close_speed_rad_s) else {
            return self.readback_failed(joint, "pre-enable hold frame unavailable".into());
        };
        // Build and emit the capped 0x307 pre-enable hold frame through the ONE PosForce emitter
        // (`allow_freeze = false`: enable requires provably fresh <= 100 ms stationary feedback).
        let hold = match self.emit_pos_force_frame(
            TxKind::Handshake,
            self.enable_stop_stamp,
            PosForceIntentKind::Target {
                position: hold_pos,
                velocity: Some(close_speed),
                current_raw: None,
            },
            false,
            false,
            false,
        ) {
            Ok(frame) => frame,
            Err(TxRefusal::Deferred) => {
                return self.readback_failed_err(
                    joint,
                    ArmDriverError::PosForcePreEnableHoldBudgetExhausted { joint },
                );
            }
            Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                self.abort_enable_for_estop();
                return Err(ArmDriverError::EstopInhibited);
            }
            Err(TxRefusal::BuildRefused) => {
                return self.readback_failed(joint, "pre-enable hold frame unavailable".into());
            }
            Err(TxRefusal::Transport(e)) => {
                return self.readback_failed_err(
                    joint,
                    ArmDriverError::PosForcePreEnableHoldTxFailed {
                        joint,
                        kind: TxFailKind::from_transport(&e),
                    },
                );
            }
            Err(r_err) => {
                return self.readback_failed_err(joint, Self::refusal_to_driver_err(r_err));
            }
        };
        self.enable_estop_inhibited()?;
        if let Err(e) = self.can.flush() {
            return self.readback_failed_err(
                joint,
                ArmDriverError::PosForcePreEnableHoldTxFailed {
                    joint,
                    kind: TxFailKind::from_transport(&e),
                },
            );
        }
        self.record_sent_target(joint, hold, None);
        self.joint_modes[joint] = Some(DamiaoControlMode::PosForce);
        self.pos_force_verified = true;
        tracing::info!("Damiao joint {} confirmed in position-force mode (CTRL_MODE 4); pre-enable hold sent at {:.4} rad", joint + 1, hold_pos);
        self.send_enable()
    }

    /// Revalidate that a position-force gripper joint still has provably fresh (`<= 100 ms`)
    /// feedback (both decode instant and empty-queue bound) and verified CTRL_MODE 4 right before
    /// transmitting an initial, carried, or retried `CMD_ENABLE` frame.
    fn pf_enable_feedback_fresh(&self, joint: usize) -> bool {
        if !self.pos_force.as_ref().is_some_and(|pf| pf.joint == joint) {
            return true;
        }
        if !self.pos_force_verified {
            return false;
        }
        let Some((rx_at, queue_bound)) = self.pf_last_fresh_feedback else {
            return false;
        };
        self.cached_states.get(joint).is_some_and(|s| s.position.is_finite())
            && self.since(rx_at) <= POS_FORCE_FEEDBACK_MAX_AGE
            && self.since(queue_bound) <= POS_FORCE_FEEDBACK_MAX_AGE
    }

    /// Transmit enable frames for joints in `mask` through [`Self::send`] (`TxKind::Handshake`),
    /// enforcing **deadline -> budget -> stop gate -> J7 freshness revalidation -> stamp -> transmit**
    /// on every frame and checking the stop gate before flush. Returns the bitmask of joints still
    /// unsent because the TX budget or backpressure stopped this pass (to be carried into subsequent
    /// `poll_enable` ticks).
    fn send_enable_masked(
        &mut self,
        mut mask: u32,
        fail_on_error: bool,
        deadline: Option<Deadline>,
    ) -> Result<u32, ArmDriverError> {
        self.enable_estop_inhibited()?;
        let mut sent_any = false;
        for i in handshake::mask_joints(mask) {
            let Some(&id) = self.motor_ids.get(i) else {
                mask &= !(1 << i);
                continue;
            };
            match self.send(TxKind::Handshake, self.enable_stop_stamp, deadline, false, |d| {
                if !d.pf_enable_feedback_fresh(i) {
                    return None;
                }
                Some(damiao::build_enable_frame(id))
            }) {
                Ok(_) => {
                    mask &= !(1 << i);
                    sent_any = true;
                }
                Err(TxRefusal::DeadlinePassed | TxRefusal::Deferred) => break,
                Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                    self.abort_enable_for_estop();
                    return Err(ArmDriverError::EstopInhibited);
                }
                Err(TxRefusal::BuildRefused | TxRefusal::PosForceFeedbackExpired(_)) => {
                    self.torque_hs = None;
                    self.enable_failed();
                    return Err(ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint: i });
                }
                Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => {
                    if fail_on_error {
                        self.enable_failed();
                        return Err(e.into());
                    }
                    tracing::warn!("Retry transmit enable frame backpressured for motor {}: {:?}", id, e);
                    break;
                }
                Err(TxRefusal::Transport(e)) => {
                    if fail_on_error {
                        self.enable_failed();
                        return Err(e.into());
                    }
                    tracing::warn!("Retry transmit enable frame failed for motor {}: {:?}", id, e);
                }
                Err(TxRefusal::MitFenceTripped(reason)) => {
                    self.torque_hs = None;
                    self.enable_failed();
                    return Err(ArmDriverError::MitFenceTripped(reason));
                }
            }
        }
        if sent_any {
            self.enable_estop_inhibited()?;
            if let Err(e) = self.can.flush() {
                if fail_on_error {
                    self.enable_failed();
                    return Err(e.into());
                }
                tracing::warn!("Retry flush failed: {:?}", e);
            }
        }
        Ok(mask)
    }

    /// Enable frames to every motor (bounded by the per-tick TX budget, carrying any remaining
    /// motors into subsequent `poll_enable` ticks), then the confirmation window
    /// (DAMIAO_ENABLE_CONFIRM, bounded by time only).
    fn send_enable(&mut self) -> Result<TorqueProgress, ArmDriverError> {
        self.enable_estop_inhibited()?;
        let handshake_start = self.now();
        self.last_enable_command = Some(handshake_start);
        self.last_enable_command_rx_seq = Some(self.rx_seq);
        self.last_command_was_disable = false;
        self.disable_confirmed_sticky = false;
        self.last_feedback_instant.fill(None);

        let n = self.motor_ids.len().min(handshake::MAX_MASK_JOINTS);
        let all_mask = if n >= 32 { u32::MAX } else { (1u32 << n) - 1 };
        let initial_deadline = Deadline::after(handshake_start, DAMIAO_ENABLE_CONFIRM);
        let unsent_mask = self.send_enable_masked(all_mask, true, Some(initial_deadline))?;
        let poll_start = self.now();
        let window = ConfirmWindow::new(TorqueDirection::Enable, self.motor_ids.len(), handshake_start, poll_start,
            DAMIAO_ENABLE_CONFIRM, &DAMIAO_ENABLE_RETRY_AT);
        self.torque_hs = Some(DamiaoTorque::Enable { window, unsent_mask });
        Ok(TorqueProgress::Pending { wake: poll_start })
    }

    /// Single per-motor post-command feedback boundary shared by [`Self::enable_state`],
    /// [`Self::fresh_state_mask`] (`poll_enable`/`poll_disable`), MIT recovery, TIMEOUT
    /// commissioning, and `set_zero_position`.
    fn motor_feedback_after(
        &self,
        i: usize,
        after_instant: Option<Instant>,
        after_rx_seq: Option<u64>,
    ) -> bool {
        !self.awaiting_rx_boundary
            && self.last_feedback_instant.get(i).copied().flatten().is_some_and(|t| {
                after_instant.is_none_or(|cmd| t >= cmd)
                    && after_rx_seq.is_none_or(|seq| self.last_feedback_seq[i] > seq)
            })
    }

    fn joint_disable_confirmed_fresh(&self, i: usize) -> bool {
        self.motor_feedback_after(i, self.last_enable_command, self.last_enable_command_rx_seq)
            && self.motor_status[i] == 0
            && self
                .last_feedback_instant
                .get(i)
                .and_then(|&t| t)
                .is_some_and(|t| self.since(t) <= DAMIAO_FEEDBACK_VALID_WINDOW)
    }

    /// Joints (mask) whose feedback arrived strictly after `(after, after_rx_seq)` and reports
    /// `state`; and whether every joint answered after that boundary.
    fn fresh_state_mask(&self, after: Instant, after_rx_seq: Option<u64>, state: u8) -> (u32, bool) {
        let mut mask = 0u32;
        let mut all_fresh = !self.last_feedback_instant.is_empty() && !self.awaiting_rx_boundary;
        for i in 0..self.motor_ids.len().min(handshake::MAX_MASK_JOINTS) {
            let fresh = self.motor_feedback_after(i, Some(after), after_rx_seq);
            all_fresh &= fresh;
            if fresh && self.motor_status[i] == state { mask |= 1 << i; }
        }
        (mask, all_fresh)
    }

    fn poll_enable(&mut self, mut window: ConfirmWindow, mut unsent_mask: u32) -> Result<TorqueProgress, ArmDriverError> {
        self.enable_estop_inhibited()?;
        if window.expire(self.now()).is_some() {
            self.enable_failed();
            return Err(ArmDriverError::HardwareFault(format!(
                "Not all Damiao motors entered enabled state within {} ms: feedback_valid={}, status={:?}",
                DAMIAO_ENABLE_CONFIRM.as_millis(), self.is_feedback_valid(), self.motor_status
            )));
        }
        let _ = self.receive_feedback();
        self.enable_estop_inhibited()?;
        let deadline = Deadline { at: window.deadline };
        if unsent_mask != 0 {
            unsent_mask = self.send_enable_masked(unsent_mask, true, Some(deadline))?;
        }
        let (mask, all_fresh) = self.fresh_state_mask(window.fresh_after, self.last_enable_command_rx_seq, 1);
        let fault = if all_fresh { self.motor_status.iter().position(|&st| damiao_fault_name(st).is_some()) } else { None };
        match window.service(self.now(), mask, all_fresh, fault) {
            Service::Confirmed => {
                tracing::info!("All Damiao motors confirmed enabled (state=1) in {:?}", self.since(window.started));
                self.is_enabling = false;
                self.is_enabled = true;
                self.pending_rearm = false;
                self.enable_stop_stamp = None;
                // confirmed enable with the base modes restored: a new MIT handover arms a new fence
                self.mit_fence = None;
                self.mit_fence_trip = None;
                self.mit_fence_acked = false;
                self.last_mit_recovery_rx_seq = None;
                self.mit_tracking_used = false;
                self.mit_fence_last_cmd.fill(None);
                Ok(TorqueProgress::Done(TorqueOutcome::Confirmed))
            }
            Service::Fault { .. } => {
                let e = self.check_hardware_status().err()
                    .unwrap_or_else(|| ArmDriverError::HardwareFault("motor reported a fault during enable".into()));
                self.enable_failed();
                Err(e)
            }
            Service::Retry { mask, wake } => {
                unsent_mask = self.send_enable_masked(unsent_mask | mask, false, Some(deadline))?;
                let wake = if unsent_mask != 0
                    && self.tx_release_deadline.is_none_or(|cap| self.now() < cap)
                {
                    self.now()
                } else {
                    wake
                };
                self.torque_hs = Some(DamiaoTorque::Enable { window, unsent_mask });
                Ok(TorqueProgress::Pending { wake })
            }
            Service::Wait { wake } => {
                let wake = if unsent_mask != 0
                    && self.tx_release_deadline.is_none_or(|cap| self.now() < cap)
                {
                    self.now()
                } else {
                    wake
                };
                self.torque_hs = Some(DamiaoTorque::Enable { window, unsent_mask });
                Ok(TorqueProgress::Pending { wake })
            }
        }
    }

    /// Disable: frames to every motor at once (a pending enable is abandoned), then the
    /// confirmation window. Safety torque-off is exempt from the motion TX frame budget.
    fn begin_disable(&mut self) -> Result<TorqueProgress, ArmDriverError> {
        self.begin_disable_budgeted(None)
    }

    fn begin_disable_budgeted(&mut self, max_rx_frames: Option<u16>) -> Result<TorqueProgress, ArmDriverError> {
        self.refresh_capture = None;
        if let Some(hs) = self.torque_hs.take() {
            if matches!(hs, DamiaoTorque::DrainBeforeEnable(_)) {
                self.awaiting_rx_boundary = false;
            }
            if !matches!(hs, DamiaoTorque::Disable { .. }) {
                self.enable_failed();
            }
        }
        self.discard_all_pending_tx();
        self.begin_tx_tick();
        self.refresh_unfinished = 0;
        // MIT tracking: before the first disable frame, discard every cached target so no
        // later recovery can replay one, whatever the disable's outcome (the fence stays
        // armed until the disable is confirmed).
        if self.mit_fence.is_some() || self.mit_fence_trip.is_some() {
            self.clear_cached_targets();
        }
        // Still send disable if draining times out: thermal protection must not
        // be blocked by traffic. Until a new empty-queue boundary, ignore ACKs.
        self.awaiting_rx_boundary = match max_rx_frames {
            Some(0) => !self.last_read_drained,
            Some(limit) => {
                self.last_poll_rx = None;
                self.rx_frame_limit = Some(limit.min(64));
                let drained = self.drain_rx_before_command_limit(usize::from(limit));
                self.rx_frame_limit = None;
                !drained
            }
            None => !self.drain_rx_before_command(),
        };
        let after_seq = self.rx_seq;
        let cmd_start = self.now();
        self.last_enable_command = Some(cmd_start);
        self.last_enable_command_rx_seq = Some(self.rx_seq);
        self.last_command_was_disable = true;
        self.disable_confirmed_sticky = false;
        let mut last_err = None;
        for idx in 0..self.motor_ids.len() {
            if let Err(r) = self.send(TxKind::Disable, None, None, false, |d| {
                Some(damiao::build_disable_frame(d.motor_ids[idx]))
            }) {
                last_err = Some(Self::refusal_to_driver_err(r));
            }
        }
        if let Err(e) = self.can.flush() {
            last_err = Some(e.into());
        }
        if let Some(e) = last_err {
            // A failed or partial write cannot establish the physical power state.
            return Err(e);
        }
        self.is_enabled = false;
        self.pending_rearm = true;
        self.pos_force_verified = false;
        self.clear_cached_targets();
        // MIT tracking: the fence stays until every motor is confirmed disabled
        let mit_fenced = self.mit_fence.is_some() || self.mit_fence_trip.is_some();
        self.last_query_instant = Some(self.now());
        // Confirmation for DAMIAO_DISABLE_CONFIRM, bounded by time only (a cap of 10 checks with
        // 2 ms sleeps ended it after ~20 ms, before a late motor answered); the disable frame is
        // resent to unconfirmed motors like the enable does.
        let poll_start = self.now();
        let window = ConfirmWindow::new(TorqueDirection::Disable, self.motor_ids.len(), cmd_start, poll_start,
            DAMIAO_DISABLE_CONFIRM, &DAMIAO_ENABLE_RETRY_AT);
        self.torque_hs = Some(DamiaoTorque::Disable { window, mit_fenced, after_seq });
        Ok(TorqueProgress::Pending { wake: poll_start })
    }

    fn poll_disable(&mut self, mut window: ConfirmWindow, mit_fenced: bool, after_seq: u64) -> Result<TorqueProgress, ArmDriverError> {
        if window.expire(self.now()).is_some() {
            if mit_fenced {
                // unconfirmed: the motors may still be energized in MIT; keep the fence (no recovery,
                // no poll) and report the failure
                return Err(ArmDriverError::HardwareFault(format!(
                    "Disable not confirmed by fresh state-0 feedback from every motor within {} ms; the MIT fence stays",
                    DAMIAO_DISABLE_CONFIRM.as_millis())));
            }
            return Ok(TorqueProgress::Done(TorqueOutcome::Unconfirmed));
        }
        let _ = self.receive_feedback();
        let boundary_seq = self.last_enable_command_rx_seq.map_or(after_seq, |s| s.max(after_seq));
        let (mask, all_fresh) = self.fresh_state_mask(window.fresh_after, Some(boundary_seq), 0);
        match window.service(self.now(), mask, all_fresh, None) {
            Service::Confirmed => {
                tracing::info!("All Damiao motors confirmed disabled (state=0) in {:?}", self.since(window.started));
                self.disable_confirmed_sticky = true;
                if mit_fenced { self.mit_fence = None; }
                Ok(TorqueProgress::Done(TorqueOutcome::Confirmed))
            }
            Service::Retry { mask, wake } => {
                for i in handshake::mask_joints(mask) {
                    let Some(&id) = self.motor_ids.get(i) else { continue };
                    tracing::debug!("Retrying disable for motor {} (CAN ID 0x{:X})", i, id);
                    if let Err(r) = self.send(TxKind::Disable, None, None, false, |_| {
                        Some(damiao::build_disable_frame(id))
                    }) {
                        tracing::warn!("Retry transmit disable frame failed for motor {}: {:?}", id, r);
                    }
                }
                if let Err(e) = self.can.flush() {
                    tracing::warn!("Retry flush failed: {:?}", e);
                }
                self.torque_hs = Some(DamiaoTorque::Disable { window, mit_fenced, after_seq });
                Ok(TorqueProgress::Pending { wake })
            }
            // a disable window never reports a fault (no fault index is passed)
            Service::Wait { .. } | Service::Fault { .. } => {
                let wake = window.wake();
                self.torque_hs = Some(DamiaoTorque::Disable { window, mit_fenced, after_seq });
                Ok(TorqueProgress::Pending { wake })
            }
        }
    }

    pub fn with_vlim(mut self, vlim: f32) -> Self {
        self.default_vlim = vlim;
        self
    }

    pub fn with_gains(mut self, kp: f32, kd: f32) -> Self {
        self.default_kp = kp;
        self.default_kd = kd;
        self
    }

    pub fn can_mut(&mut self) -> &mut C {
        &mut self.can
    }

    /// A command acknowledgement must be separated from buffered pre-command telemetry.
    /// Drained telemetry frames update the feedback cache so pre-write checks (state 0,
    /// stationarity, position) reflect the newest bus state; stale register replies are
    /// ignored so they cannot satisfy a subsequent register transaction.
    fn drain_rx_before_command(&mut self) -> bool {
        self.drain_rx_before_command_limit(128)
    }

    fn drain_rx_before_command_limit(&mut self, max_frames: usize) -> bool {
        let limit = self.rx_frame_limit.map_or(max_frames, |rem| max_frames.min(usize::from(rem)));
        self.last_read_drained = false;
        let start = self.now();
        let mut frames_processed = 0usize;
        let mut rx_drained = false;
        let mut hit_err = false;
        while frames_processed < limit && self.since(start) < Duration::from_millis(5) {
            let attempt = self.now();
            match self.can.receive_timestamped() {
                Ok((frame, rx_ts)) => {
                    frames_processed += 1;
                    self.process_rx_frame(&frame, false, rx_ts);
                }
                Err(TransportError::Timeout) | Err(TransportError::WouldBlock) => {
                    rx_drained = true;
                    self.mark_rx_empty(attempt);
                    break;
                }
                Err(_) => {
                    hit_err = true;
                    break;
                }
            }
        }
        if self.rx_frame_limit.is_some() {
            self.account_rx_pass(frames_processed, !rx_drained && !hit_err);
        }
        rx_drained
    }

    fn poll_torque_inner(&mut self, deadline: Instant, max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
        self.begin_tx_tick();
        self.last_poll_rx = None;
        self.rx_deadline = Some(deadline);
        self.rx_frame_limit = Some(max_rx_frames.min(64));
        let r = match self.torque_hs.take() {
            None => Err(ArmDriverError::Protocol("no torque transition pending".into())),
            Some(DamiaoTorque::DrainBeforeEnable(d)) => self.poll_drain_before_enable(d),
            Some(DamiaoTorque::RefreshBeforeRestore { refresh, stray, stray_len }) => {
                self.poll_refresh_before_restore(refresh, stray, stray_len)
            }
            Some(DamiaoTorque::Restore { write, pf_refreshed }) => self.poll_restore(write, pf_refreshed),
            Some(DamiaoTorque::Readback(r)) => self.poll_readback(r),
            Some(DamiaoTorque::PreEnableRefresh(refresh)) => self.poll_pre_enable_refresh(refresh),
            Some(DamiaoTorque::Enable { window, unsent_mask }) => self.poll_enable(window, unsent_mask),
            Some(DamiaoTorque::Disable { window, mit_fenced, after_seq }) => {
                self.poll_disable(window, mit_fenced, after_seq)
            }
        };
        self.rx_deadline = None;
        self.rx_frame_limit = None;
        r
    }
}

impl<C: CanTransport> ArmDriver for DamiaoArmDriver<C> {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            velocity_feedback: true,
            torque_feedback: true,
            temperature_feedback: true,
            torque_feedforward: self.control_mode == DamiaoControlMode::Mit,
            enable_feedback: true,
            load_feedback: false,
        }
    }

    fn enable_state(&self) -> EnableState {
        if self.awaiting_rx_boundary || self.motor_status.is_empty() {
            return EnableState::Unknown;
        }
        // A disable confirmed by post-command feedback is sticky for shutdown: `enable_state()`
        // stays `Disabled` until a later enable request (or non-zero motor feedback) arrives,
        // even when no newer polls can be sent (e.g. after an unacknowledged MIT fence trip).
        if self.disable_confirmed_sticky
            && !self.is_enabled
            && !self.is_enabling
            && self.motor_status.iter().all(|&state| state == 0)
        {
            return EnableState::Disabled;
        }
        if !self.is_feedback_valid() {
            return EnableState::Unknown;
        }
        // Every motor must have reported current feedback since startup, and (once an enable or
        // disable request has been issued) strictly after that request using the exact same
        // per-motor boundary as `fresh_state_mask`. Initial zeroed `motor_status`, pre-command
        // samples, or stale feedback after CAN silence never prove `Disabled`.
        if (0..self.motor_ids.len()).any(|i| {
            !self.motor_feedback_after(i, self.last_enable_command, self.last_enable_command_rx_seq)
        }) {
            return EnableState::Unknown;
        }
        if self.motor_status.iter().all(|&state| state == 0) {
            EnableState::Disabled
        } else if self.is_enabled && self.motor_status.iter().all(|&state| state == 1) {
            EnableState::Enabled
        } else {
            EnableState::Unknown
        }
    }

    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        self.last_feedback_instant.iter().map(|t| t.map(|t| self.since(t))).collect()
    }

    fn feedback_instants(&self) -> Vec<Option<Instant>> {
        self.last_feedback_instant.clone()
    }

    fn feedback_seqs(&self) -> Vec<u64> {
        self.feedback_seq.clone()
    }

    fn feedback_queue_bounds(&self) -> Vec<Option<Instant>> {
        self.feedback_queue_bound.clone()
    }

    fn feedback_receive_instants(&self) -> Vec<Option<Instant>> {
        self.last_feedback_instant.clone()
    }


    fn max_command_velocity(&self, joint: usize) -> Option<f32> {
        if joint >= self.motor_ids.len() || self.control_mode != DamiaoControlMode::PosVel {
            return None;
        }
        Some(self.default_vlim.abs().clamp(0.1, 4.0))
    }

    fn is_enabled(&self) -> bool {
        self.is_enabled && self.enable_state() == EnableState::Enabled
    }

    fn motor_status(&self) -> Option<Vec<u8>> {
        Some(self.motor_status.clone())
    }

    fn set_torque_flag(&mut self, enabled: bool) {
        self.is_enabled = enabled;
        if enabled {
            self.last_command_was_disable = false;
            self.disable_confirmed_sticky = false;
        } else {
            self.pending_rearm = true;
            self.discard_all_pending_tx();
        }
    }

    fn set_pending_rearm(&mut self, pending: bool) {
        self.pending_rearm = pending;
    }

    fn supports_mit_mode_switch(&self) -> bool {
        self.control_mode == DamiaoControlMode::PosVel
    }

    fn set_joint_modes(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        // MIT tracking owns the joints (fence armed) or lost them (tripped): only the fenced,
        // confirmed switch may touch the modes
        if self.mit_fence.is_some() || self.mit_fence_trip.is_some() {
            return Err(ArmDriverError::HardwareFault("MIT tracking holds the deadline fence: plain mode switch refused".into()));
        }
        self.discard_all_pending_tx();
        self.begin_tx_tick();
        self.set_joint_modes_impl(modes, hold)
    }

    fn set_joint_modes_in_tick(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        if self.mit_fence.is_some() || self.mit_fence_trip.is_some() {
            return Err(ArmDriverError::HardwareFault("MIT tracking holds the deadline fence: plain mode switch refused".into()));
        }
        self.discard_all_pending_tx();
        let _ = self.ensure_tx_pass();
        self.set_joint_modes_impl(modes, hold)
    }

    fn pending_motion_tx_len(&self) -> usize {
        self.pending_tx.len() + usize::from(self.pf_deferred_motion)
    }

    fn pending_hold_tx_len(&self) -> usize {
        self.pending_hold_tx.len()
    }

    /// True only when a `0xCC` TX probe write & flush completed and fresh feedback for the probed
    /// motor arrived strictly after the post-write empty-queue RX sequence boundary
    /// (`last_feedback_seq[probe.joint] > probe.boundary_seq`). Because SocketCAN supplies no
    /// arrival timestamps (`CanTransport::receive_timestamped` returns `None`), this post-write
    /// RX sequence boundary is the proof that confirming feedback came after the probe write.
    fn tx_probe_confirmed(&self) -> bool {
        let Some(probe) = self.tx_probe else { return false };
        !self.awaiting_rx_boundary
            && self.total_pending_tx_len() == 0
            && probe.joint < self.last_feedback_seq.len()
            && self.last_feedback_seq[probe.joint] > probe.boundary_seq
            && self.last_feedback_instant[probe.joint]
                .is_some_and(|t| self.since(t) <= Duration::from_millis(250))
            && self.is_feedback_valid()
    }

    fn tx_probe_pending(&self) -> bool {
        let Some(probe) = self.tx_probe else { return false };
        !self.tx_probe_confirmed() && self.since(probe.sent_at) < REGISTER_REPLY_TIMEOUT
    }

    fn probe_tx_ready(&mut self) -> Result<bool, ArmDriverError> {
        self.probe_tx_ready_inner(None, true)
    }

    fn probe_tx_ready_budgeted(&mut self, deadline: Instant) -> Result<bool, ArmDriverError> {
        self.probe_tx_ready_inner(Some(Deadline { at: deadline }), false)
    }

    fn clear_tx_probe(&mut self) {
        self.tx_probe = None;
    }

    fn set_joint_modes_confirmed(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.switch_joint_modes_confirmed(modes, hold)
    }

    fn set_mit_deadline_fence(&mut self, deadline: Option<Duration>, holdoff: Duration) {
        if deadline.is_some() {
            self.mit_tracking_used = true;
            self.discard_all_pending_tx();
        }
        self.mit_fence = deadline;
        self.mit_fence_holdoff = holdoff;
        let now = self.now();
        self.mit_fence_last_cmd.fill(deadline.map(|_| now));
    }

    fn mit_fence_trip(&self) -> Option<String> { self.mit_fence_trip.as_ref().map(|(r, _)| r.to_string()) }

    fn clock_now(&self) -> Instant { self.now() }

    fn driver_clock(&self) -> Option<std::sync::Arc<dyn crate::transport::DriverClock>> {
        self.clock.clone()
    }

    fn mit_fresh_status(&mut self) -> Result<Vec<u8>, String> {
        // Enforce the post-trip quiet period directly without requiring or mutating
        // `mit_fence_acked`: autonomous background CAN refreshes stay inhibited until
        // `acknowledge_mit_fence_trip()` runs AFTER `clear_check` passes.
        if let Some((_, at)) = &self.mit_fence_trip {
            if self.since(*at) < self.mit_fence_holdoff {
                let remaining = self.mit_fence_holdoff.saturating_sub(self.since(*at));
                return Err(format!("{}: the motors' command timeout may still be running; no frame is sent for {} ms more after the loss",
                    crate::mit::QUIET_PERIOD_CODE, remaining.as_millis().max(1)));
            }
        }
        let start_stamp = self.current_stop_stamp();
        self.pending_stop_check(Some(start_stamp)).map_err(|e| format!("status poll not sent: {e}"))?;
        let pre = self.rx_boundary().map_err(|e| e.to_string())?;
        self.pending_stop_check(Some(start_stamp)).map_err(|e| format!("status poll not sent: {e}"))?;
        for idx in 0..self.motor_ids.len() {
            match self.send(TxKind::Refresh, Some(start_stamp), None, false, |d| {
                Some(damiao::build_refresh_frame(d.motor_ids[idx]))
            }) {
                Ok(_) => {}
                Err(TxRefusal::Deferred) => {
                    return Err("status poll not sent: TX budget exhausted".into());
                }
                Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                    return Err(format!("status poll not sent: {}", ArmDriverError::EstopInhibited));
                }
                Err(r) => {
                    return Err(format!("status poll not sent: {}", Self::refusal_to_driver_err(r)));
                }
            }
        }
        self.pending_stop_check(Some(start_stamp)).map_err(|e| format!("status poll not sent: {e}"))?;
        self.can.flush().map_err(|e| format!("flush: {e}"))?;
        let start = self.now();
        loop {
            let _ = self.read_joint_states();
            let stale: Vec<String> = (0..self.motor_ids.len()).filter(|&i| self.last_feedback_seq[i] <= pre).map(|i| format!("J{}", i + 1)).collect();
            if stale.is_empty() {
                return Ok(self.motor_status.clone());
            }
            if self.since(start) >= REGISTER_REPLY_TIMEOUT {
                return Err(format!("no fresh feedback from {}", stale.join(", ")));
            }
            self.pause(Duration::from_micros(500));
        }
    }

    fn mit_expiry_recover(&mut self) -> Result<(), String> {
        self.last_mit_recovery_rx_seq = None;
        self.discard_all_pending_tx();
        self.begin_tx_tick();
        let start_stamp = self.current_stop_stamp();
        let req_boundary = self
            .rx_boundary()
            .map_err(|e| format!("receive boundary before MIT recovery: {e}"))?;
        // 1. clear-error (FF..FB on the motor's ID) to every motor reporting an error state
        let faulted: Vec<usize> = (0..self.motor_ids.len()).filter(|&i| self.motor_status[i] >= 8).collect();
        for &i in &faulted {
            match self.send(TxKind::ClearFault, Some(start_stamp), None, false, |d| {
                Some(damiao::build_clear_faults_frame(d.motor_ids[i]))
            }) {
                Ok(_) => {}
                Err(TxRefusal::Deferred) => {
                    return Err(format!("clear-error to J{} not sent: TX budget exhausted", i + 1));
                }
                Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                    return Err(format!("clear-error to J{} inhibited: {}", i + 1, ArmDriverError::EstopInhibited));
                }
                Err(r) => {
                    return Err(format!("clear-error to J{} not sent: {}", i + 1, Self::refusal_to_driver_err(r)));
                }
            }
        }
        if !faulted.is_empty() {
            self.pending_stop_check(Some(start_stamp)).map_err(|e| format!("clear-error flush inhibited: {e}"))?;
            self.can.flush().map_err(|e| format!("flush: {e}"))?;
        }
        // 2. no cached frame may ever be replayed; then the disable of every motor
        self.clear_cached_targets();
        let res = self.set_torque(false);
        let boundary = self
            .last_enable_command_rx_seq
            .map_or(req_boundary, |s| s.max(req_boundary));
        let unconfirmed: Vec<String> = (0..self.motor_ids.len()).filter(|&i| {
            !(self.motor_feedback_after(i, self.last_enable_command, Some(boundary))
                && self.last_feedback_seq[i] > boundary
                && self.motor_status[i] == 0)
        }).map(|i| format!("J{}", i + 1)).collect();
        if res.is_err() || !unconfirmed.is_empty() {
            return Err(format!("{} did not confirm the disable (fresh state 0)", unconfirmed.join(", ")));
        }
        self.last_mit_recovery_rx_seq = Some(boundary);
        self.disable_confirmed_sticky = true;
        self.mit_fence = None;
        Ok(())
    }

    fn acknowledge_mit_fence_trip(&mut self) {
        if self.mit_fence_trip.is_some() { self.mit_fence_acked = true; }
    }

    fn mit_feedback_after_recovery(&self, dof: usize) -> bool {
        !self.awaiting_rx_boundary
            && self.last_mit_recovery_rx_seq.is_some_and(|req| {
                self.last_feedback_seq.len() == dof
                    && self.last_feedback_seq.iter().all(|&seq| seq > req)
            })
    }

    fn joint_modes_uncertain(&self) -> bool {
        self.joint_modes.iter().any(Option::is_none)
    }

    fn range_check(&self) -> Option<DriverRangeCheck> { self.range_check.clone() }

    fn take_recovery_writes(&mut self) -> Vec<usize> {
        self.recovery_written_joints.drain(..).collect()
    }

    fn set_strict_mode_handover(&mut self, strict: bool) {
        self.strict_mode_handover = strict;
    }

    fn read_motor_identity(&mut self, joints: &[usize]) -> Result<Vec<crate::mit::MotorIdentity>, ArmDriverError> {
        let n = self.motor_ids.len();
        if let Some(&j) = joints.iter().find(|&&j| j >= n) { return Err(ArmDriverError::JointCountMismatch { expected: n, actual: j + 1 }); }
        joints.iter().map(|&i| self.identity_of(i)).collect()
    }

    fn commission_motor_timeout(&mut self, joints: &[usize], counts: u32) -> Result<Vec<crate::mit::TimeoutCommissioning>, crate::mit::TimeoutBatchError> {
        let none_attempted = |e: ArmDriverError| crate::mit::TimeoutBatchError { not_attempted: joints.to_vec(), ..e.into() };
        let n = self.motor_ids.len();
        if let Some(&j) = joints.iter().find(|&&j| j >= n) { return Err(none_attempted(ArmDriverError::JointCountMismatch { expected: n, actual: j + 1 })); }
        if counts == 0 || counts > crate::mit::MAX_TIMEOUT_COUNTS {
            return Err(none_attempted(ArmDriverError::Protocol(format!("TIMEOUT counts must be in 1..={}", crate::mit::MAX_TIMEOUT_COUNTS))));
        }
        if self.is_enabled || self.is_enabling {
            return Err(none_attempted(ArmDriverError::Protocol("the TIMEOUT commissioning needs torque off".into())));
        }
        for &i in joints {
            // fresh feedback after the last enable / disable command, reporting state 0 (disabled)
            let fresh = self.motor_feedback_after(i, self.last_enable_command, self.last_enable_command_rx_seq)
                && self.last_feedback_instant[i].is_some_and(|t| self.since(t) <= ZERO_FEEDBACK_MAX_AGE);
            if !fresh || self.motor_status[i] != 0 {
                return Err(none_attempted(ArmDriverError::HardwareFault(format!("joint {} (CAN ID 0x{:X}) is not confirmed disabled (state {})",
                    i + 1, self.motor_ids[i], self.motor_status[i]))));
            }
        }
        let mut out = Vec::with_capacity(joints.len());
        for (k, &i) in joints.iter().enumerate() {
            let mut written = false;
            let res = self.commission_one(i, counts, &mut written);
            match res {
                Ok(c) => out.push(c),
                Err(error) => return Err(crate::mit::TimeoutBatchError { applied: out, uncertain: if written { vec![i] } else { Vec::new() },
                    not_attempted: joints[k + 1..].to_vec(), error }),
            }
        }
        Ok(out)
    }


    fn motor_zero_support(&self) -> MotorZeroSupport {
        MotorZeroSupport::set_current("motor_flash", false, vec![
            "Damiao set-zero frame FF FF FF FF FF FF FF FE on the motor's CAN ID, sent with every motor disabled \
             and confirmed disabled from fresh feedback, one motor at a time (reBotArm_control_py RebotArm.set_zero)".into(),
            "Not verified on hardware whether the zero survives a power cycle: the official wiki says the B601-DM is \
             calibrated once after assembly; motorbridge's Damiao notes say the zero is lost at power-off for the \
             gripper (multi-turn travel)".into(),
        ])
    }

    fn set_zero_position(&mut self, joint: usize) -> Result<(), ArmDriverError> {
        let start_stamp = self.current_stop_stamp();
        self.strict_stop_check(Some(start_stamp))?;
        let n = self.motor_ids.len();
        if joint >= n { return Err(ArmDriverError::JointCountMismatch { expected: n, actual: joint + 1 }); }
        let id = self.motor_ids[joint];
        if self.is_enabled || self.is_enabling {
            return Err(ArmDriverError::Protocol(format!("joint {} (CAN ID 0x{id:X}): set-zero needs torque off", joint + 1)));
        }
        // fresh feedback after the last enable / disable command, reporting state 0 (disabled)
        let fresh = self.motor_feedback_after(joint, self.last_enable_command, self.last_enable_command_rx_seq)
            && self.last_feedback_instant[joint].is_some_and(|t| self.since(t) <= ZERO_FEEDBACK_MAX_AGE);
        if !fresh || self.motor_status[joint] != 0 {
            return Err(ArmDriverError::HardwareFault(format!(
                "joint {} (CAN ID 0x{id:X}) is not confirmed disabled (state {}, feedback age {:?})",
                joint + 1, self.motor_status[joint], self.last_feedback_instant[joint].map(|t| self.since(t)))));
        }
        self.send_with_retry(TxKind::ModeWrite, Some(start_stamp), None, false, |_| {
            Some(damiao::build_set_zero_frame(id))
        })
        .map_err(Self::refusal_to_driver_err)?;
        self.strict_stop_check(Some(start_stamp))?;
        self.can.flush()?;
        tracing::warn!("Damiao joint {} (CAN ID 0x{id:X}): set-zero sent at reading {:.4} rad", joint + 1, self.cached_states[joint].position);
        Ok(())
    }

    fn is_feedback_valid(&self) -> bool {
        if self.last_feedback_instant.is_empty() {
            return false;
        }
        let limit = DAMIAO_FEEDBACK_VALID_WINDOW;
        let mut all_valid = true;
        // Use monotonic process uptime to prevent clock-jump discontinuities on RTC synchronization
        static MONOTONIC_START: std::sync::LazyLock<Instant> = std::sync::LazyLock::new(Instant::now);
        let now_sec = MONOTONIC_START.elapsed().as_secs();
        let last_warn = self.last_warn_sec.load(Ordering::Relaxed);
        let should_warn = now_sec.saturating_sub(last_warn) >= 3;

        for (i, opt) in self.last_feedback_instant.iter().enumerate() {
            let ok = opt.is_some_and(|instant| self.since(instant) <= limit);
            if !ok {
                all_valid = false;
                if should_warn && (self.is_enabled || self.is_enabling) {
                    tracing::warn!(
                        "Damiao Joint {} (CAN ID 0x{:X}) feedback stale or missing: elapsed={:?} > {:?}",
                        i,
                        self.motor_ids.get(i).copied().unwrap_or(0),
                        opt.map(|t| self.since(t)),
                        limit
                    );
                }
            }
        }
        if !all_valid && should_warn && (self.is_enabled || self.is_enabling) {
            self.last_warn_sec.store(now_sec, Ordering::Relaxed);
        }
        all_valid
    }

    fn check_hardware_status(&self) -> Result<(), ArmDriverError> {
        for (i, &st) in self.motor_status.iter().enumerate() {
            let motor_id = self.motor_ids.get(i).copied().unwrap_or(0);
            if let Some(reason) = damiao_fault_name(st) {
                return Err(ArmDriverError::HardwareFault(format!(
                    "Joint {} (CAN ID 0x{:X}) reported fault: {}",
                    i, motor_id, reason
                )));
            }
        }
        Ok(())
    }

    fn clear_fault(&mut self) -> Result<(), ArmDriverError> {
        self.discard_all_pending_tx();
        self.begin_tx_tick();
        let start_stamp = self.current_stop_stamp();
        let mut err: Option<ArmDriverError> = None;
        let mut sent_any = false;
        for idx in 0..self.motor_ids.len() {
            match self.send(TxKind::ClearFault, Some(start_stamp), None, false, |d| {
                Some(damiao::build_clear_faults_frame(d.motor_ids[idx]))
            }) {
                Ok(_) => sent_any = true,
                Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                    return Err(ArmDriverError::EstopInhibited);
                }
                Err(TxRefusal::Deferred) => {
                    let pending = self.motor_ids.len().saturating_sub(idx);
                    if err.is_none() {
                        err = Some(ArmDriverError::TxDeferred { pending });
                    }
                    break;
                }
                Err(TxRefusal::Transport(e)) => {
                    err = Some(ArmDriverError::HardwareFault(e.to_string()));
                }
                Err(r) => {
                    err = Some(Self::refusal_to_driver_err(r));
                }
            }
        }
        self.pending_stop_check(Some(start_stamp))?;
        if sent_any {
            if let Err(e) = self.can.flush() {
                if err.is_none() {
                    err = Some(ArmDriverError::HardwareFault(e.to_string()));
                }
            }
        }
        match err {
            Some(e) => Err(e),
            None => Ok(()),
        }
    }

    fn clear_fault_budgeted(&mut self, deadline: Instant) -> Result<(), ArmDriverError> {
        if !self.can.supports_nonblocking_rt() || !self.can.supports_bounded_flush() {
            return Err(ArmDriverError::Protocol(
                "UNSUPPORTED: transport does not support non-blocking RT I/O and bounded flush".into(),
            ));
        }
        self.discard_all_pending_tx();
        let dl = Deadline { at: deadline };
        let start_stamp = self.current_stop_stamp();
        let mut err: Option<ArmDriverError> = None;
        let mut sent_any = false;
        for idx in 0..self.motor_ids.len() {
            match self.send(TxKind::ClearFault, Some(start_stamp), Some(dl), false, |d| {
                Some(damiao::build_clear_faults_frame(d.motor_ids[idx]))
            }) {
                Ok(_) => sent_any = true,
                Err(TxRefusal::DeadlinePassed) => {
                    return Err(ArmDriverError::Transport(TransportError::Timeout));
                }
                Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                    return Err(ArmDriverError::EstopInhibited);
                }
                Err(TxRefusal::Deferred) => {
                    let pending = self.motor_ids.len().saturating_sub(idx);
                    if err.is_none() {
                        err = Some(ArmDriverError::TxDeferred { pending });
                    }
                    break;
                }
                Err(TxRefusal::Transport(e)) => {
                    err = Some(ArmDriverError::Transport(e));
                }
                Err(r) => {
                    err = Some(Self::refusal_to_driver_err(r));
                }
            }
        }
        if dl.passed(self.now()) {
            return Err(ArmDriverError::Transport(TransportError::Timeout));
        }
        self.pending_stop_check(Some(start_stamp))?;
        if sent_any {
            if let Err(e) = self.can.flush() {
                if err.is_none() {
                    err = Some(ArmDriverError::Transport(e));
                }
            }
        }
        if dl.passed(self.now()) {
            return Err(ArmDriverError::Transport(TransportError::Timeout));
        }
        match err {
            Some(e) => Err(e),
            None => Ok(()),
        }
    }

    /// Blocking torque transition: [`ArmDriver::begin_torque`], then served every 0.5 ms until
    /// it finishes. An unconfirmed disable returns Ok (callers check the enable state) unless the
    /// MIT fence is armed.
    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        let mut progress = self.begin_torque(enabled)?;
        let mut first = true;
        loop {
            if let TorqueProgress::Done(_) = progress { return Ok(()); }
            if !first { self.pause(Duration::from_micros(500)); }
            first = false;
            let deadline = self.now() + DAMIAO_RX_PASS;
            progress = self.poll_torque(deadline)?;
        }
    }

    fn begin_torque(&mut self, enabled: bool) -> Result<TorqueProgress, ArmDriverError> {
        if enabled { self.begin_enable() } else { self.begin_disable() }
    }

    fn begin_torque_budgeted(&mut self, enabled: bool, max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
        if !self.can.supports_nonblocking_rt() {
            return Err(ArmDriverError::Protocol(
                "UNSUPPORTED: CAN transport does not support non-blocking RT I/O".into(),
            ));
        }
        if enabled {
            self.begin_enable_budgeted(Some(max_rx_frames))
        } else {
            self.begin_disable_budgeted(Some(max_rx_frames))
        }
    }

    fn poll_torque(&mut self, deadline: Instant) -> Result<TorqueProgress, ArmDriverError> {
        self.poll_torque_inner(deadline, 64)
    }

    fn poll_torque_budgeted(&mut self, deadline: Instant, max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
        if !self.can.supports_nonblocking_rt() {
            return Err(ArmDriverError::Protocol(
                "UNSUPPORTED: CAN transport does not support non-blocking RT I/O".into(),
            ));
        }
        self.poll_torque_inner(deadline, max_rx_frames)
    }

    fn take_last_poll_rx(&mut self) -> Option<(u16, bool)> {
        self.last_poll_rx.take()
    }

    fn torque_transition_pending(&self) -> bool { self.torque_hs.is_some() }

    fn abort_enable(&mut self) {
        if self.torque_hs.as_ref().is_some_and(|hs| !matches!(hs, DamiaoTorque::Disable { .. })) {
            if matches!(self.torque_hs, Some(DamiaoTorque::DrainBeforeEnable(_))) {
                self.awaiting_rx_boundary = false;
            }
            self.torque_hs = None;
            self.enable_failed();
        }
    }

    fn panic_disable_step(
        &mut self,
        first: bool,
        deadline: Instant,
    ) -> Option<Result<Vec<usize>, ArmDriverError>> {
        if first {
            self.refresh_capture = None;
            if let Some(hs) = self.torque_hs.take() {
                if !matches!(hs, DamiaoTorque::Disable { .. }) {
                    self.enable_failed();
                }
            }
            self.enable_stop_stamp = None;
            self.discard_all_pending_tx();
            self.refresh_unfinished = 0;
            self.pos_force_verified = false;
            self.clear_cached_targets();
            self.awaiting_rx_boundary = !self.last_read_drained;
            let cmd_start = self.now();
            self.last_enable_command = Some(cmd_start);
            self.last_enable_command_rx_seq = Some(self.rx_seq);
            self.last_command_was_disable = true;
            self.disable_confirmed_sticky = false;
            self.is_enabled = false;
            self.pending_rearm = true;
            self.last_query_instant = Some(cmd_start);
        }

        let unconfirmed_before = self.unconfirmed_disable_joints();
        if unconfirmed_before.is_empty()
            && self.is_feedback_valid()
            && self.enable_state() == EnableState::Disabled
        {
            self.disable_confirmed_sticky = true;
            self.mit_fence = None;
            return Some(Ok(Vec::new()));
        }

        // Send disable to every unconfirmed or stale motor; a send error on one motor must NEVER
        // stop attempts on the remaining motors unless the step deadline itself has passed.
        let tx_deadline = Deadline { at: deadline };
        for i in unconfirmed_before {
            let Some(&id) = self.motor_ids.get(i) else { continue };
            match self.send(TxKind::Disable, None, Some(tx_deadline), false, |_| {
                Some(damiao::build_disable_frame(id))
            }) {
                Ok(_) => {}
                Err(TxRefusal::DeadlinePassed) => {
                    tracing::warn!(
                        "Panic disable step deadline passed before TX for Damiao joint J{} (CAN ID 0x{:X})",
                        i + 1,
                        id
                    );
                    break;
                }
                Err(r) => {
                    tracing::warn!(
                        "Panic disable TX failed for Damiao joint J{} (CAN ID 0x{:X}): {:?}",
                        i + 1,
                        id,
                        r
                    );
                }
            }
        }
        if let Err(e) = self.can.flush() {
            tracing::warn!("Panic disable CAN flush failed: {e}");
        }

        self.last_poll_rx = None;
        self.rx_deadline = Some(deadline);
        self.rx_frame_limit = Some(64);
        let _ = self.receive_feedback();
        self.rx_deadline = None;
        self.rx_frame_limit = None;

        let remaining = self.unconfirmed_disable_joints();
        if remaining.is_empty()
            && self.is_feedback_valid()
            && self.enable_state() == EnableState::Disabled
        {
            self.disable_confirmed_sticky = true;
            self.mit_fence = None;
        }
        Some(Ok(remaining))
    }

    fn unconfirmed_disable_joints(&self) -> Vec<usize> {
        (0..self.motor_ids.len())
            .filter(|&i| !self.joint_disable_confirmed_fresh(i))
            .collect()
    }

    fn supports_rt_executor(&self) -> Result<(), &'static str> {
        if self.can.supports_nonblocking_rt() && self.can.supports_bounded_flush() {
            Ok(())
        } else {
            Err("CAN transport does not support non-blocking RT I/O and bounded flush")
        }
    }

    fn set_rt_mode(&mut self, rt: bool) {
        self.rt_no_sleep = rt;
    }

    fn motor_current_cap(&self) -> Option<MotorCurrentCap> {
        if let Some(pf) = &self.pos_force {
            return Some(MotorCurrentCap {
                joint: pf.joint, mode: "pos_force", current_raw: Some(pf.current_raw),
                active: self.pos_force_verified && self.joint_modes.get(pf.joint).copied().flatten() == Some(DamiaoControlMode::PosForce),
                calibrated: pf.calibrated, ceiling_torque_nm: pf.ceiling_torque_nm, torque_reference: pf.torque_reference.clone(),
                close_speed_rad_s: Some(pf.close_speed_rad_s),
                note: if pf.calibrated { "motor current ceiling on every gripper frame (commissioned)" }
                    else { "motor current ceiling on every gripper frame; its torque equivalent is uncalibrated" },
            });
        }
        self.legacy_gripper.map(|joint| MotorCurrentCap {
            joint, mode: "pos_vel", current_raw: None, active: false, calibrated: false, ceiling_torque_nm: None,
            torque_reference: None, close_speed_rad_s: None, note: "no motor current cap (legacy POS_VEL gripper)",
        })
    }

    fn gripper_pos_force_info(&self) -> Option<GripperPosForceInfo> {
        let pf = self.pos_force.as_ref()?;
        if !(pf.close_speed_rad_s.is_finite() && pf.close_speed_rad_s > 0.0) {
            return None;
        }
        let active = self.pos_force_verified
            && self.joint_modes.get(pf.joint).copied().flatten() == Some(DamiaoControlMode::PosForce);
        let ceiling_torque_estimate_nm = pf.ceiling_torque_nm
            .or(Some(f32::from(pf.current_raw) * crate::model::DamiaoGripperDrive::FEEDBACK_EQUIVALENT_NM_PER_RAW))
            .filter(|v| v.is_finite() && *v > 0.0);
        Some(GripperPosForceInfo {
            joint: pf.joint,
            active,
            close_speed_rad_s: pf.close_speed_rad_s,
            ceiling_torque_estimate_nm,
        })
    }

    fn clock_pause(&self, d: Duration) { self.pause(d) }

    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        match self.write_joint_targets_bounded(targets) {
            Ok(TxBatchOutcome::Complete) => Ok(()),
            Ok(TxBatchOutcome::Deferred { pending }) => {
                // Legacy callers treat Err as output failure and stop motion without notifying
                // the driver; discard queued motion frames so none survive into a faulted state.
                self.discard_pending_tx_stopped();
                Err(ArmDriverError::TxDeferred { pending })
            }
            Ok(TxBatchOutcome::Expired { .. }) => {
                self.discard_pending_tx_stopped();
                Err(ArmDriverError::Transport(TransportError::Timeout))
            }
            Err(e) => {
                self.discard_pending_tx_stopped();
                Err(e)
            }
        }
    }

    /// Stationary restorative hold while a stop/E-stop is latched (zero allocation):
    /// discards queued motion, refuses pending stops, enforces the tick TX budget and optional
    /// MIT fence, and rebuilds every frame at send time from fresh (`<= 100 ms`) measured
    /// feedback through [`Self::send`] with `TxKind::RollbackHold`.
    fn write_estop_hold(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        match self.write_estop_hold_bounded(targets) {
            Ok(TxBatchOutcome::Complete) => Ok(()),
            Ok(TxBatchOutcome::Deferred { pending }) => {
                self.discard_pending_hold_tx();
                Err(ArmDriverError::TxDeferred { pending })
            }
            Ok(TxBatchOutcome::Expired { .. }) => {
                self.discard_pending_hold_tx();
                Err(ArmDriverError::Transport(TransportError::Timeout))
            }
            Err(e) => {
                self.discard_pending_hold_tx();
                Err(e)
            }
        }
    }

    fn write_estop_hold_bounded(&mut self, targets: &[JointTarget]) -> Result<TxBatchOutcome, ArmDriverError> {
        self.discard_pending_tx_stopped();
        self.last_hold_tx_sent_any = false;
        self.last_hold_tx_due_mask = 0;
        self.last_hold_tx_sent_mask = 0;
        self.last_expired_oldest_age = None;
        self.last_tx_pf_status = PosForceTxStatus::Unobserved;
        if targets.len() != self.motor_ids.len() {
            self.reject_target_tx(MitFenceTripReason::PreflightRejected);
            return Err(ArmDriverError::JointCountMismatch {
                expected: self.motor_ids.len(),
                actual: targets.len(),
            });
        }
        if self.torque_hs.is_some() {
            self.reject_target_tx(MitFenceTripReason::PreflightRejected);
            return Err(ArmDriverError::TorqueTransitionPending);
        }
        let fenced = self.mit_fence.is_some() || self.mit_fence_trip.is_some();
        // Stopped-hold NEVER validates caller's J7 target (so an invalid caller target can NEVER
        // suppress J7 hold/freeze); it only requires mode confirmation (`targets = None`).
        if let Some(e) = self.target_frames_refused(None, true) {
            self.reject_target_tx(MitFenceTripReason::PreflightRejected);
            return Err(e);
        }
        let total_joints = targets.len().min(u8::MAX as usize) as u8;
        let arm_res: Result<TxBatchOutcome, ArmDriverError> = if fenced {
            self.discard_pending_hold_tx();
            if let Err(e) = self.pending_stop_check(None) {
                self.trip_fence(MitFenceTripReason::EstopBeforeBatch { total_joints });
                return Err(e);
            }
            let start_stamp = self.current_stop_stamp();
            (|| -> Result<TxBatchOutcome, ArmDriverError> {
                self.fence_check()?;
                let pass = self.ensure_tx_pass();
                let mut started = [None::<Instant>; 16];
                if targets.len() > started.len() {
                    self.reject_target_tx(MitFenceTripReason::PreflightRejected);
                    return Err(ArmDriverError::JointCountMismatch {
                        expected: started.len(),
                        actual: targets.len(),
                    });
                }
                let mut batch_frames = 0usize;
                let mut due_mask: u64 = 0;
                let mut sent_any = false;
                let mut sent_slot_mask: u64 = 0;
                for (i, target) in targets.iter().enumerate() {
                    if self.joint_modes.get(i).copied().flatten() == Some(DamiaoControlMode::PosForce) {
                        continue;
                    }
                    let count = match self.target_frames_checked(i, target) {
                        Ok((frames, _)) => frames.into_iter().flatten().count(),
                        Err(e) => {
                            self.trip_fence(MitFenceTripReason::TargetUnavailable {
                                joint: (i + 1) as u8,
                                completed_joints: 0,
                                total_joints,
                            });
                            return Err(e);
                        }
                    };
                    batch_frames += count;
                    for slot in 0..count.min(2) {
                        let bit = i.saturating_mul(2).saturating_add(slot);
                        if bit < 64 {
                            due_mask |= 1u64 << bit;
                        }
                    }
                }
                if pass.backpressured {
                    let reason = MitFenceTripReason::TxBackpressuredBeforeBatch;
                    self.trip_fence(reason);
                    return Err(ArmDriverError::MitFenceTripped(reason));
                }
                if pass.attempts.saturating_add(batch_frames) > self.tx_budget.max_frames {
                    let remaining = self.tx_budget.max_frames.saturating_sub(pass.attempts);
                    let reason = MitFenceTripReason::TxBudgetExceededPreflight {
                        needed: batch_frames.min(u16::MAX as usize) as u16,
                        remaining: remaining.min(u16::MAX as usize) as u16,
                        max: self.tx_budget.max_frames.min(u16::MAX as usize) as u16,
                    };
                    self.trip_fence(reason);
                    return Err(ArmDriverError::MitFenceTripped(reason));
                }
                if self.now() >= pass.deadline {
                    let reason = MitFenceTripReason::TxBudgetDeadlineBeforeBatch;
                    self.trip_fence(reason);
                    return Err(ArmDriverError::MitFenceTripped(reason));
                }
                for (i, target) in targets.iter().enumerate() {
                    if self.joint_modes.get(i).copied().flatten() == Some(DamiaoControlMode::PosForce) {
                        continue;
                    }
                    let slot_count = match self.target_frames_checked(i, target) {
                        Ok((frames, _)) => frames.into_iter().flatten().count(),
                        Err(e) => {
                            self.trip_fence(MitFenceTripReason::TargetUnavailable {
                                joint: (i + 1) as u8,
                                completed_joints: i as u8,
                                total_joints,
                            });
                            return Err(e);
                        }
                    };
                    for slot in 0..slot_count {
                        if started[i].is_none() {
                            started[i] = Some(self.now());
                        }
                        match self.send(TxKind::RollbackHold(i), Some(start_stamp), None, true, |d| {
                            let fresh_pos = d.fresh_measured_position(i, POS_FORCE_FEEDBACK_MAX_AGE)?;
                            let hold_target = JointTarget {
                                position: fresh_pos,
                                velocity: Some(0.0),
                                kp: target.kp,
                                kd: target.kd,
                                torque: target.torque,
                            };
                            d.target_frames_checked(i, &hold_target)
                                .ok()?
                                .0
                                .get(slot)
                                .copied()
                                .flatten()
                        }) {
                            Ok(sent_frame) => {
                                let mit_params = self.resolved_mit_params(target);
                                self.record_sent_target(i, sent_frame, Some(mit_params));
                                sent_any = true;
                                let bit = i.saturating_mul(2).saturating_add(slot);
                                if bit < 64 {
                                    sent_slot_mask |= 1u64 << bit;
                                }
                            }
                            Err(TxRefusal::PosForceFeedbackExpired(joint)) => {
                                self.trip_fence(MitFenceTripReason::TargetUnavailable {
                                    joint: (joint + 1) as u8,
                                    completed_joints: i as u8,
                                    total_joints,
                                });
                                return Err(ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint });
                            }
                            Err(TxRefusal::Deferred) => {
                                let reason = MitFenceTripReason::TxBudgetExhausted {
                                    completed_joints: i as u8,
                                    total_joints,
                                };
                                self.trip_fence(reason);
                                return Err(ArmDriverError::MitFenceTripped(reason));
                            }
                            Err(TxRefusal::MitFenceTripped(reason)) => {
                                return Err(ArmDriverError::MitFenceTripped(reason));
                            }
                            Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                                self.trip_fence(MitFenceTripReason::EstopDuringBatch {
                                    completed_joints: i as u8,
                                    total_joints,
                                });
                                return Err(ArmDriverError::EstopInhibited);
                            }
                            Err(TxRefusal::BuildRefused) => {
                                self.trip_fence(MitFenceTripReason::TargetUnavailable {
                                    joint: (i + 1) as u8,
                                    completed_joints: i as u8,
                                    total_joints,
                                });
                                if self.fresh_measured_position(i, POS_FORCE_FEEDBACK_MAX_AGE).is_none() {
                                    return Err(ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint: i });
                                }
                                return Err(ArmDriverError::PosForceTargetUnavailable { joint: i });
                            }
                            Err(TxRefusal::Transport(e)) => {
                                self.mark_tx_pass_backpressured();
                                if self.pending_stop_check(Some(start_stamp)).is_err() {
                                    self.trip_fence(MitFenceTripReason::EstopBetweenAttempts {
                                        joint: (i + 1) as u8,
                                        completed_joints: i as u8,
                                        total_joints,
                                    });
                                    return Err(ArmDriverError::EstopInhibited);
                                }
                                self.last_hold_tx_due_mask = due_mask;
                                self.last_hold_tx_sent_mask = if self.can.flush_is_noop() { sent_slot_mask } else { 0 };
                                self.trip_fence(MitFenceTripReason::TransmitFailed {
                                    joint: (i + 1) as u8,
                                    completed_joints: i as u8,
                                    total_joints,
                                    kind: TxFailKind::from_transport(&e),
                                });
                                return Err(e.into());
                            }
                            Err(TxRefusal::DeadlinePassed) => {
                                let reason = MitFenceTripReason::TxBudgetDeadlineBeforeBatch;
                                self.trip_fence(reason);
                                return Err(ArmDriverError::MitFenceTripped(reason));
                            }
                        }
                    }
                }
                if self.pending_stop_check(Some(start_stamp)).is_err() {
                    self.trip_fence(MitFenceTripReason::EstopBeforeFlush { total_joints });
                    return Err(ArmDriverError::EstopInhibited);
                }
                if sent_any {
                    if self.flush_cutoff_exceeded() {
                        self.mark_tx_pass_backpressured();
                        self.last_hold_tx_sent_any = false;
                        self.last_hold_tx_due_mask = due_mask;
                        self.last_hold_tx_sent_mask = 0;
                        self.trip_fence(MitFenceTripReason::FlushFailed {
                            kind: TxFailKind::Timeout,
                        });
                        return Err(TransportError::Timeout.into());
                    }
                    if let Err(e) = self.can.flush() {
                        self.mark_tx_pass_backpressured();
                        self.last_hold_tx_sent_any = false;
                        if self.pending_stop_check(Some(start_stamp)).is_err() {
                            self.trip_fence(MitFenceTripReason::EstopAfterFlush { total_joints });
                            return Err(ArmDriverError::EstopInhibited);
                        }
                        self.last_hold_tx_due_mask = due_mask;
                        self.last_hold_tx_sent_mask = 0;
                        self.trip_fence(MitFenceTripReason::FlushFailed {
                            kind: TxFailKind::from_transport(&e),
                        });
                        return Err(e.into());
                    }
                }
                if self.pending_stop_check(Some(start_stamp)).is_err() {
                    self.trip_fence(MitFenceTripReason::EstopAfterFlush { total_joints });
                    return Err(ArmDriverError::EstopInhibited);
                }
                self.fence_check()?;
                for (i, t) in started.into_iter().enumerate().take(targets.len()) {
                    if t.is_some() {
                        self.mit_fence_last_cmd[i] = t;
                    }
                }
                self.last_hold_tx_sent_any = sent_any;
                self.last_hold_tx_due_mask = due_mask;
                self.last_hold_tx_sent_mask = sent_slot_mask;
                Ok(TxBatchOutcome::Complete)
            })()
        } else {
            // Unfenced bounded arm-only stopped-hold path: carries deferred holds for J1–J6 inline
            // in `pending_hold_tx` with round-robin `hold_cursor` rotation. Active PosForce J7 is
            // NEVER staged here; it is emitted strictly once via `emit_pos_force_j7_impl`.
            if let Err(e) = self.pending_stop_check(None) {
                self.discard_pending_hold_tx();
                return Err(e);
            }
            self.output_ran_since_rx = true;
            let start_stamp = self.current_stop_stamp();
            let _ = self.drop_stopped_hold_tx(start_stamp);
            let cleanup_now = self.now();
            let (pre_dropped_stale, mut oldest_expired) =
                self.pending_hold_tx.drop_stale(cleanup_now, self.tx_budget.frame_max_age);
            if pre_dropped_stale > 0 {
                crate::diagnostics::tick::record_tx_dropped_stale(pre_dropped_stale as u32);
            }

            let n = targets.len();
            if n > MAX_MODE_JOINTS {
                self.discard_pending_hold_tx();
                return Err(ArmDriverError::JointCountMismatch { expected: MAX_MODE_JOINTS, actual: n });
            }
            let mut staged_ids: [[Option<u32>; 2]; MAX_MODE_JOINTS] = [[None; 2]; MAX_MODE_JOINTS];
            let mut new_keys = 0usize;
            let mut due_mask: u64 = 0;
            let mut first_build_refused_joint: Option<usize> = None;
            for (i, target) in targets.iter().enumerate() {
                if self.joint_modes.get(i).copied().flatten() == Some(DamiaoControlMode::PosForce) {
                    continue;
                }
                if self.fresh_measured_position_at(i, POS_FORCE_FEEDBACK_MAX_AGE, cleanup_now).is_none() {
                    first_build_refused_joint.get_or_insert(i);
                    self.pending_hold_tx.remove_joint(i);
                    continue;
                }
                let cmds = match self.pending_target_cmds_checked(i, target) {
                    Ok(v) => v,
                    Err(e) => {
                        self.discard_pending_hold_tx();
                        return Err(e);
                    }
                };
                for (slot, maybe_cmd) in cmds.into_iter().enumerate() {
                    if let Some((frame_id, _)) = maybe_cmd {
                        staged_ids[i][slot] = Some(frame_id);
                        let flat_bit = i.saturating_mul(2).saturating_add(slot);
                        if flat_bit < 64 {
                            due_mask |= 1u64 << flat_bit;
                        }
                        if self.pending_hold_tx.find_key(i, frame_id).is_none() {
                            new_keys += 1;
                        }
                    }
                }
            }
            if self.pending_hold_tx.len() + new_keys > DAMIAO_PENDING_TX_CAPACITY {
                let queued = self.pending_hold_tx.len();
                self.discard_pending_hold_tx();
                return Err(ArmDriverError::TxQueueFull {
                    queued,
                    new: new_keys,
                    capacity: DAMIAO_PENDING_TX_CAPACITY,
                });
            }

            let batch_ts = self.now();
            let total_slots = n.saturating_mul(2);
            let start_flat = if total_slots > 0 { self.hold_cursor % total_slots } else { 0 };
            for offset in 0..total_slots {
                let flat = (start_flat + offset) % total_slots;
                let i = flat / 2;
                let slot = flat % 2;
                if let Some(frame_id) = staged_ids[i][slot] {
                    let target = &targets[i];
                    let cmd = PendingTargetCmd::RollbackHold {
                        kp: target.kp,
                        kd: target.kd,
                        torque: target.torque,
                        slot: slot as u8,
                    };
                    let ok = self.pending_hold_tx.upsert(i, frame_id, cmd, batch_ts, start_stamp);
                    debug_assert!(ok, "preflight checked pending_hold_tx capacity");
                }
            }

            let _ = self.ensure_tx_pass();
            let mut sent_any = false;
            let mut sent_slot_mask: u64 = 0;
            let mut dropped_stale_in_pass = 0usize;
            (|| -> Result<TxBatchOutcome, ArmDriverError> {
                while let Some(entry) = self.pending_hold_tx.front() {
                    if !self.can_start_arm_tx() {
                        break;
                    }
                    let i = entry.joint;
                    let entry_deadline = Deadline::after(entry.queued_at, self.tx_budget.frame_max_age);
                    let PendingTargetCmd::RollbackHold { kp, kd, torque, slot } = entry.cmd else {
                        let _ = self.pending_hold_tx.pop_front();
                        continue;
                    };
                    match self.send(
                        TxKind::RollbackHold(i),
                        Some(entry.stop_stamp),
                        Some(entry_deadline),
                        false,
                        |d| {
                            let fresh_pos = d.fresh_measured_position(i, POS_FORCE_FEEDBACK_MAX_AGE)?;
                            let hold_target = JointTarget {
                                position: fresh_pos,
                                velocity: Some(0.0),
                                kp,
                                kd,
                                torque,
                            };
                            d.target_frames_checked(i, &hold_target)
                                .ok()?
                                .0
                                .get(usize::from(slot))
                                .copied()
                                .flatten()
                        },
                    ) {
                        Ok(sent_frame) => {
                            let _ = self.pending_hold_tx.pop_front();
                            let mit_params = (
                                kp.filter(|k| k.is_finite()).unwrap_or(self.default_kp),
                                kd.filter(|k| k.is_finite()).unwrap_or(self.default_kd),
                                torque.filter(|t| t.is_finite()).unwrap_or(0.0),
                            );
                            self.record_sent_target(i, sent_frame, Some(mit_params));
                            sent_any = true;
                            let flat_bit = i.saturating_mul(2).saturating_add(usize::from(slot));
                            if flat_bit < 64 {
                                sent_slot_mask |= 1u64 << flat_bit;
                            }
                            if total_slots > 0 {
                                self.hold_cursor = (i * 2 + usize::from(slot) + 1) % total_slots;
                            }
                        }
                        Err(TxRefusal::PosForceFeedbackExpired(_)) => {
                            let _ = self.pending_hold_tx.pop_front();
                            continue;
                        }
                        Err(TxRefusal::DeadlinePassed) => {
                            let now = self.now();
                            let _ = self.pending_hold_tx.pop_front();
                            let age = now.saturating_duration_since(entry.queued_at);
                            oldest_expired = Some(oldest_expired.map_or(age, |o| o.max(age)));
                            dropped_stale_in_pass += 1;
                            crate::diagnostics::tick::record_tx_dropped_stale(1);
                        }
                        Err(TxRefusal::Deferred) => break,
                        Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                            self.last_hold_tx_sent_any = sent_any && self.can.flush_is_noop();
                            self.last_hold_tx_due_mask = 0;
                            self.last_hold_tx_sent_mask = 0;
                            self.discard_pending_hold_tx();
                            return Err(ArmDriverError::EstopInhibited);
                        }
                        Err(TxRefusal::BuildRefused) => {
                            let flat_bit = i.saturating_mul(2).saturating_add(usize::from(slot));
                            if flat_bit < 64 {
                                due_mask &= !(1u64 << flat_bit);
                            }
                            first_build_refused_joint.get_or_insert(i);
                            let _ = self.pending_hold_tx.pop_front();
                            continue;
                        }
                        Err(TxRefusal::MitFenceTripped(reason)) => {
                            self.last_hold_tx_sent_any = sent_any && self.can.flush_is_noop();
                            self.last_hold_tx_due_mask = 0;
                            self.last_hold_tx_sent_mask = 0;
                            self.discard_pending_hold_tx();
                            return Err(ArmDriverError::MitFenceTripped(reason));
                        }
                        Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => {
                            if self.pending_stop_check(Some(start_stamp)).is_err() {
                                self.last_hold_tx_sent_any = sent_any && self.can.flush_is_noop();
                                self.last_hold_tx_due_mask = 0;
                                self.last_hold_tx_sent_mask = 0;
                                self.discard_pending_hold_tx();
                                return Err(ArmDriverError::EstopInhibited);
                            }
                            break;
                        }
                        Err(TxRefusal::Transport(e)) => {
                            self.last_hold_tx_sent_any = sent_any && self.can.flush_is_noop();
                            self.mark_tx_pass_backpressured();
                            self.discard_pending_hold_tx();
                            if self.pending_stop_check(Some(start_stamp)).is_err() {
                                self.last_hold_tx_due_mask = 0;
                                self.last_hold_tx_sent_mask = 0;
                                return Err(ArmDriverError::EstopInhibited);
                            }
                            self.last_hold_tx_due_mask = due_mask;
                            self.last_hold_tx_sent_mask = if self.can.flush_is_noop() { sent_slot_mask } else { 0 };
                            return Err(e.into());
                        }
                    }
                }

                if self.pending_stop_check(Some(start_stamp)).is_err() {
                    self.last_hold_tx_sent_any = sent_any && self.can.flush_is_noop();
                    self.last_hold_tx_due_mask = 0;
                    self.last_hold_tx_sent_mask = 0;
                    self.discard_pending_hold_tx();
                    return Err(ArmDriverError::EstopInhibited);
                }
                if sent_any {
                    if self.flush_cutoff_exceeded() {
                        self.mark_tx_pass_backpressured();
                        self.discard_pending_hold_tx();
                        self.last_hold_tx_sent_any = false;
                        self.last_hold_tx_due_mask = due_mask;
                        self.last_hold_tx_sent_mask = 0;
                        return Err(TransportError::Timeout.into());
                    }
                    if let Err(e) = self.can.flush() {
                        self.mark_tx_pass_backpressured();
                        self.discard_pending_hold_tx();
                        self.last_hold_tx_sent_any = false;
                        if self.pending_stop_check(Some(start_stamp)).is_err() {
                            self.last_hold_tx_due_mask = 0;
                            self.last_hold_tx_sent_mask = 0;
                            return Err(ArmDriverError::EstopInhibited);
                        }
                        self.last_hold_tx_due_mask = due_mask;
                        self.last_hold_tx_sent_mask = 0;
                        return Err(e.into());
                    }
                }
                if self.pending_stop_check(Some(start_stamp)).is_err() {
                    self.last_hold_tx_due_mask = 0;
                    self.last_hold_tx_sent_mask = 0;
                    self.discard_pending_hold_tx();
                    return Err(ArmDriverError::EstopInhibited);
                }
                if let Some(refused_joint) = first_build_refused_joint {
                    let post_now = self.now();
                    let (late_stale, late_oldest) =
                        self.pending_hold_tx.drop_stale(post_now, self.tx_budget.frame_max_age);
                    if late_stale > 0 {
                        if let Some(age) = late_oldest {
                            oldest_expired = Some(oldest_expired.map_or(age, |o| o.max(age)));
                        }
                        self.last_expired_oldest_age = oldest_expired;
                        crate::diagnostics::tick::record_tx_dropped_stale(late_stale as u32);
                    }
                    let newly_deferred = self.pending_hold_tx.mark_newly_deferred();
                    crate::diagnostics::tick::record_tx_deferred(newly_deferred as u32, self.total_pending_tx_len());
                    self.last_hold_tx_sent_any = sent_any;
                    self.last_hold_tx_due_mask = due_mask;
                    self.last_hold_tx_sent_mask = sent_slot_mask;
                    if self.fresh_measured_position(refused_joint, POS_FORCE_FEEDBACK_MAX_AGE).is_none() {
                        return Err(ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint: refused_joint });
                    }
                    return Err(ArmDriverError::PosForceTargetUnavailable { joint: refused_joint });
                }
                let post_now = self.now();
                let (late_stale, late_oldest) =
                    self.pending_hold_tx.drop_stale(post_now, self.tx_budget.frame_max_age);
                if late_stale > 0 {
                    dropped_stale_in_pass += late_stale;
                    if let Some(age) = late_oldest {
                        oldest_expired = Some(oldest_expired.map_or(age, |o| o.max(age)));
                    }
                    crate::diagnostics::tick::record_tx_dropped_stale(late_stale as u32);
                }

                self.last_hold_tx_sent_any = sent_any;
                self.last_hold_tx_due_mask = due_mask;
                self.last_hold_tx_sent_mask = sent_slot_mask;
                let newly_deferred = self.pending_hold_tx.mark_newly_deferred();
                let pending = self.pending_hold_tx.len();
                crate::diagnostics::tick::record_tx_deferred(newly_deferred as u32, self.total_pending_tx_len());
                let total_dropped_stale = pre_dropped_stale.saturating_add(dropped_stale_in_pass);
                if total_dropped_stale > 0 {
                    self.last_expired_oldest_age = oldest_expired;
                    Ok(TxBatchOutcome::Expired { dropped: total_dropped_stale, pending })
                } else if pending == 0 {
                    self.last_expired_oldest_age = None;
                    Ok(TxBatchOutcome::Complete)
                } else {
                    self.last_expired_oldest_age = None;
                    Ok(TxBatchOutcome::Deferred { pending })
                }
            })()
        };

        // When attached to `SupervisorInner` (`external_pos_force_emitter == true`), the tick
        // epilogue invokes `emit_pos_force_j7` once per tick after every arm path. Direct driver
        // callers invoke the exact same ONE emitter right here after the arm stopped-hold.
        if self.external_pos_force_emitter || matches!(arm_res, Err(ArmDriverError::EstopInhibited)) {
            return arm_res;
        }
        let stop_intent_stamp = self.current_j7_intent_stamp();
        match self.emit_pos_force_j7_impl(PosForceIntent::stop(stop_intent_stamp), stop_intent_stamp) {
            Ok(PosForceEmitOutcome::NotApplicable) => arm_res,
            Ok(PosForceEmitOutcome::Sent { joint, pf_status, .. }) => {
                let pf_bit = joint.saturating_mul(2);
                let pf_mask = if pf_bit < 64 { 1u64 << pf_bit } else { 0 };
                self.last_hold_tx_sent_any = true;
                self.last_hold_tx_due_mask |= pf_mask;
                self.last_hold_tx_sent_mask |= pf_mask;
                self.last_tx_pf_status = pf_status;
                arm_res
            }
            Ok(PosForceEmitOutcome::Deferred { joint, pf_status }) => {
                let pf_bit = joint.saturating_mul(2);
                let pf_mask = if pf_bit < 64 { 1u64 << pf_bit } else { 0 };
                self.last_hold_tx_due_mask |= pf_mask;
                self.last_tx_pf_status = pf_status;
                match arm_res {
                    Ok(TxBatchOutcome::Complete) => Ok(TxBatchOutcome::Deferred { pending: 1 }),
                    Ok(TxBatchOutcome::Deferred { pending }) => Ok(TxBatchOutcome::Deferred { pending: pending + 1 }),
                    Ok(TxBatchOutcome::Expired { dropped, pending }) => {
                        Ok(TxBatchOutcome::Expired { dropped, pending: pending + 1 })
                    }
                    Err(e) => Err(e),
                }
            }
            Err(ArmDriverError::EstopInhibited) => Err(ArmDriverError::EstopInhibited),
            Err(e) => {
                if let Some(pf_j) = self.active_pos_force_joint() {
                    let pf_bit = pf_j.saturating_mul(2);
                    if pf_bit < 64 {
                        self.last_hold_tx_due_mask |= 1u64 << pf_bit;
                    }
                }
                match arm_res {
                    Err(arm_err) => Err(arm_err),
                    Ok(_) => Err(e),
                }
            }
        }
    }

    fn set_external_pos_force_emitter(&mut self, enabled: bool) {
        self.external_pos_force_emitter = enabled;
    }

    fn validate_pos_force_intent_target(
        &self,
        position: f32,
        velocity: Option<f32>,
        current_raw: Option<u16>,
    ) -> Result<(), ArmDriverError> {
        let Some(pf) = self.pos_force.as_ref() else {
            return Ok(());
        };
        if self.joint_modes.get(pf.joint).copied().flatten() != Some(DamiaoControlMode::PosForce) {
            return Ok(());
        }
        let target = JointTarget {
            position,
            velocity,
            ..JointTarget::position(position)
        };
        let _ = self.validate_pos_force_target_static(pf.joint, &target)?;
        if current_raw.is_some_and(|c| c == 0) {
            return Err(ArmDriverError::PosForceTargetUnavailable { joint: pf.joint });
        }
        Ok(())
    }

    fn emit_pos_force_j7(
        &mut self,
        intent: PosForceIntent,
        current: PosForceIntentStamp,
    ) -> Result<PosForceEmitOutcome, ArmDriverError> {
        self.emit_pos_force_j7_impl(intent, current)
    }

    fn last_hold_tx_sent_any(&self) -> bool {
        self.last_hold_tx_sent_any
    }

    fn last_hold_tx_due_mask(&self) -> u64 {
        self.last_hold_tx_due_mask
    }

    fn last_hold_tx_sent_mask(&self) -> u64 {
        self.last_hold_tx_sent_mask
    }

    fn last_target_tx_sent_mask(&self) -> u64 {
        self.last_target_tx_sent_mask
    }

    fn take_last_pos_force_tx_status(&mut self) -> PosForceTxStatus {
        std::mem::take(&mut self.last_tx_pf_status)
    }

    fn pos_force_feedback_fresh(&self) -> bool {
        // Resolve the configured PosForce joint independently of `pos_force_verified` or active
        // TX mode so a disable or mode invalidation can NEVER bypass the <= 100 ms clear check.
        let Some(pf) = self.pos_force.as_ref() else {
            return true;
        };
        self.pos_force_feedback_valid(pf.joint, self.now())
    }

    fn pos_force_last_fresh_position(&self) -> Option<f32> {
        let _ = self.pos_force.as_ref()?;
        self.pf_last_fresh_position
    }

    fn write_joint_targets_bounded(&mut self, targets: &[JointTarget]) -> Result<TxBatchOutcome, ArmDriverError> {
        self.last_target_tx_sent_mask = 0;
        self.last_tx_pf_status = PosForceTxStatus::Unobserved;
        self.discard_pending_hold_tx();
        if targets.len() != self.motor_ids.len() {
            self.reject_target_tx(MitFenceTripReason::PreflightRejected);
            return Err(ArmDriverError::JointCountMismatch {
                expected: self.motor_ids.len(),
                actual: targets.len(),
            });
        }
        if self.torque_hs.is_some() {
            self.reject_target_tx(MitFenceTripReason::PreflightRejected);
            return Err(ArmDriverError::TorqueTransitionPending);
        }
        let fenced = self.mit_fence.is_some() || self.mit_fence_trip.is_some();
        // Mode confirmation check always runs before frame 0. For direct-driver callers
        // (`!external_pos_force_emitter`), static J7 target validity (finite position/velocity,
        // non-zero wire speed) also fails closed before sending arm frames; under `SupervisorInner`,
        // `write_output_slice` validates the J7 target up front and degrades invalid targets to
        // `Hold`/`Stop` for the tick epilogue.
        if let Some(e) = self.target_frames_refused(None, true) {
            self.reject_target_tx(MitFenceTripReason::PreflightRejected);
            return Err(e);
        }
        if !self.external_pos_force_emitter {
            if let Some(pf_j) = self.active_pos_force_joint() {
                if let Err(e) = self.validate_pos_force_target_static(pf_j, &targets[pf_j]) {
                    self.reject_target_tx(MitFenceTripReason::PreflightRejected);
                    return Err(e);
                }
            }
        }

        let _ = self.ensure_tx_pass();

        if fenced {
            // MIT tracking owns arm joints: every arm frame goes through `send(TxKind::Target, .., fenced=true)`.
            // Active PosForce J7 is NEVER written inside this fenced loop; it is emitted strictly
            // AFTER this arm path via `emit_pos_force_j7_impl` (by `SupervisorInner`'s tick epilogue
            // or directly below for standalone driver callers), whether this path succeeds, refuses
            // on stale J7 precheck, or trips the fence before/mid-batch.
            self.discard_pending_tx_stopped();
            let total_joints = targets.len().min(u8::MAX as usize) as u8;
            if let Err(e) = self.strict_stop_check(None) {
                self.trip_fence(MitFenceTripReason::EstopBeforeBatch { total_joints });
                return Err(e);
            }
            let pre_arm_intent_stamp = self.current_j7_intent_stamp();
            let arm_res: Result<TxBatchOutcome, ArmDriverError> = (|| {
                if let Some(pf_j) = self.active_pos_force_joint() {
                    if !self.pos_force_feedback_valid(pf_j, self.now()) {
                        self.reject_target_tx(MitFenceTripReason::PreflightRejected);
                        return Err(ArmDriverError::PosForceFeedbackStale { joint: pf_j });
                    }
                }
                self.fence_check()?;
                let mut started = [None::<Instant>; 16];
                if targets.len() > started.len() {
                    self.reject_target_tx(MitFenceTripReason::PreflightRejected);
                    return Err(ArmDriverError::JointCountMismatch {
                        expected: started.len(),
                        actual: targets.len(),
                    });
                }
                let start_stamp = self.current_stop_stamp();
                let mut batch_frames = 0usize;
                for (i, target) in targets.iter().enumerate() {
                    if self.joint_modes.get(i).copied().flatten() == Some(DamiaoControlMode::PosForce) {
                        continue;
                    }
                    match self.target_frames_checked(i, target) {
                        Ok((frames, _)) => batch_frames += frames.into_iter().flatten().count(),
                        Err(e) => {
                            self.trip_fence(MitFenceTripReason::TargetUnavailable {
                                joint: (i + 1) as u8,
                                completed_joints: 0,
                                total_joints,
                            });
                            return Err(e);
                        }
                    }
                }
                if !self.external_pos_force_emitter && self.active_pos_force_joint().is_some() {
                    batch_frames = batch_frames.saturating_add(1);
                }
                let pass = self.ensure_tx_pass();
                if pass.backpressured {
                    let reason = MitFenceTripReason::TxBackpressuredBeforeBatch;
                    self.trip_fence(reason);
                    return Err(ArmDriverError::MitFenceTripped(reason));
                }
                if pass.attempts.saturating_add(batch_frames) > self.tx_budget.max_frames {
                    let remaining = self.tx_budget.max_frames.saturating_sub(pass.attempts);
                    let reason = MitFenceTripReason::TxBudgetExceededPreflight {
                        needed: batch_frames.min(u16::MAX as usize) as u16,
                        remaining: remaining.min(u16::MAX as usize) as u16,
                        max: self.tx_budget.max_frames.min(u16::MAX as usize) as u16,
                    };
                    self.trip_fence(reason);
                    return Err(ArmDriverError::MitFenceTripped(reason));
                }
                if self.now() >= pass.deadline {
                    let reason = MitFenceTripReason::TxBudgetDeadlineBeforeBatch;
                    self.trip_fence(reason);
                    return Err(ArmDriverError::MitFenceTripped(reason));
                }
                for (i, target) in targets.iter().enumerate() {
                    if self.joint_modes.get(i).copied().flatten() == Some(DamiaoControlMode::PosForce) {
                        continue;
                    }
                    let slot_count = match self.target_frames_checked(i, target) {
                        Ok((frames, _)) => frames.into_iter().flatten().count(),
                        Err(e) => {
                            self.trip_fence(MitFenceTripReason::TargetUnavailable {
                                joint: (i + 1) as u8,
                                completed_joints: i as u8,
                                total_joints,
                            });
                            return Err(e);
                        }
                    };
                    for slot in 0..slot_count {
                        if started[i].is_none() {
                            started[i] = Some(self.now());
                        }
                        match self.send(TxKind::Target, Some(start_stamp), None, true, |d| {
                            d.target_frames_checked(i, target).ok()?.0.get(slot).copied().flatten()
                        }) {
                            Ok(sent_frame) => {
                                let mit_params = self.resolved_mit_params(target);
                                self.record_sent_target(i, sent_frame, Some(mit_params));
                            }
                            Err(TxRefusal::Deferred) => {
                                let reason = MitFenceTripReason::TxBudgetExhausted {
                                    completed_joints: i as u8,
                                    total_joints,
                                };
                                self.trip_fence(reason);
                                return Err(ArmDriverError::MitFenceTripped(reason));
                            }
                            Err(TxRefusal::MitFenceTripped(reason)) => {
                                return Err(ArmDriverError::MitFenceTripped(reason));
                            }
                            Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                                self.trip_fence(MitFenceTripReason::EstopDuringBatch {
                                    completed_joints: i as u8,
                                    total_joints,
                                });
                                return Err(ArmDriverError::EstopInhibited);
                            }
                            Err(TxRefusal::PosForceFeedbackExpired(joint)) => {
                                self.trip_fence(MitFenceTripReason::TargetUnavailable {
                                    joint: (joint + 1) as u8,
                                    completed_joints: i as u8,
                                    total_joints,
                                });
                                return Err(ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint });
                            }
                            Err(TxRefusal::BuildRefused) => {
                                self.trip_fence(MitFenceTripReason::TargetUnavailable {
                                    joint: (i + 1) as u8,
                                    completed_joints: i as u8,
                                    total_joints,
                                });
                                return Err(ArmDriverError::PosForceTargetUnavailable { joint: i });
                            }
                            Err(TxRefusal::Transport(e)) => {
                                self.mark_tx_pass_backpressured();
                                if self.strict_stop_check(Some(start_stamp)).is_err() {
                                    self.trip_fence(MitFenceTripReason::EstopBetweenAttempts {
                                        joint: (i + 1) as u8,
                                        completed_joints: i as u8,
                                        total_joints,
                                    });
                                    return Err(ArmDriverError::EstopInhibited);
                                }
                                self.trip_fence(MitFenceTripReason::TransmitFailed {
                                    joint: (i + 1) as u8,
                                    completed_joints: i as u8,
                                    total_joints,
                                    kind: TxFailKind::from_transport(&e),
                                });
                                return Err(e.into());
                            }
                            Err(TxRefusal::DeadlinePassed) => {
                                let reason = MitFenceTripReason::TxBudgetDeadlineBeforeBatch;
                                self.trip_fence(reason);
                                return Err(ArmDriverError::MitFenceTripped(reason));
                            }
                        }
                    }
                }
                if self.strict_stop_check(Some(start_stamp)).is_err() {
                    self.trip_fence(MitFenceTripReason::EstopBeforeFlush { total_joints });
                    return Err(ArmDriverError::EstopInhibited);
                }
                if self.flush_cutoff_exceeded() {
                    self.mark_tx_pass_backpressured();
                    self.trip_fence(MitFenceTripReason::FlushFailed {
                        kind: TxFailKind::Timeout,
                    });
                    return Err(TransportError::Timeout.into());
                }
                if let Err(e) = self.can.flush() {
                    self.mark_tx_pass_backpressured();
                    if self.strict_stop_check(Some(start_stamp)).is_err() {
                        self.trip_fence(MitFenceTripReason::EstopAfterFlush { total_joints });
                        return Err(ArmDriverError::EstopInhibited);
                    }
                    self.trip_fence(MitFenceTripReason::FlushFailed {
                        kind: TxFailKind::from_transport(&e),
                    });
                    return Err(e.into());
                }
                if self.strict_stop_check(Some(start_stamp)).is_err() {
                    self.trip_fence(MitFenceTripReason::EstopAfterFlush { total_joints });
                    return Err(ArmDriverError::EstopInhibited);
                }
                self.fence_check()?;
                let mut sent_mask: u64 = 0;
                for (i, t) in started.into_iter().enumerate().take(targets.len()) {
                    if t.is_some() {
                        self.mit_fence_last_cmd[i] = t;
                        if i < 64 {
                            sent_mask |= 1u64 << i;
                        }
                    }
                }
                self.last_target_tx_sent_mask = sent_mask;
                Ok(TxBatchOutcome::Complete)
            })();

            if self.external_pos_force_emitter || matches!(arm_res, Err(ArmDriverError::EstopInhibited)) {
                return arm_res;
            }
            let Some(pf_j) = self.active_pos_force_joint() else {
                return arm_res;
            };
            let j7_intent = match &arm_res {
                Ok(TxBatchOutcome::Complete) => PosForceIntent::target(
                    targets[pf_j].position,
                    targets[pf_j].velocity,
                    None,
                    pre_arm_intent_stamp,
                ),
                _ => PosForceIntent::hold(pre_arm_intent_stamp),
            };
            let current_intent_stamp = self.current_j7_intent_stamp();
            return match self.emit_pos_force_j7_impl(j7_intent, current_intent_stamp) {
                Ok(PosForceEmitOutcome::NotApplicable) => arm_res,
                Ok(PosForceEmitOutcome::Sent { pf_status, .. }) => {
                    self.last_tx_pf_status = pf_status;
                    if arm_res.is_ok() {
                        self.mit_fence_last_cmd[pf_j] = Some(self.now());
                        if let PosForceTxStatus::Stale { .. } = pf_status {
                            self.trip_fence(MitFenceTripReason::TargetUnavailable {
                                joint: (pf_j + 1) as u8,
                                completed_joints: pf_j as u8,
                                total_joints,
                            });
                            return Err(ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint: pf_j });
                        }
                    }
                    arm_res
                }
                Ok(PosForceEmitOutcome::Deferred { pf_status, .. }) => {
                    self.last_tx_pf_status = pf_status;
                    if arm_res.is_ok() {
                        let reason = MitFenceTripReason::TxBudgetExhausted {
                            completed_joints: pf_j as u8,
                            total_joints,
                        };
                        self.trip_fence(reason);
                        return Err(ArmDriverError::MitFenceTripped(reason));
                    }
                    arm_res
                }
                Err(ArmDriverError::EstopInhibited) => Err(ArmDriverError::EstopInhibited),
                Err(e) => match arm_res {
                    Err(arm_err) => Err(arm_err),
                    Ok(_) => Err(e),
                },
            };
        }

        // Unfenced arm-only motion path (POS_VEL, unfenced MIT): stages & drains J1–J6 in
        // `pending_tx`. Active PosForce J7 is NEVER staged in `pending_tx`; it is emitted strictly
        // AFTER via the single J7 emitter (`SupervisorInner`'s tick epilogue, or directly below
        // for standalone driver callers).
        if let Err(e) = self.strict_stop_check(None) {
            self.discard_pending_tx_stopped();
            return Err(e);
        }
        self.output_ran_since_rx = true;
        let batch_stamp = self.current_stop_stamp();
        let stopped_carry = self.pending_tx.drop_stopped(batch_stamp);
        if stopped_carry > 0 {
            crate::diagnostics::tick::record_tx_dropped_stopped(stopped_carry as u32);
        }
        let cleanup_now = self.now();
        self.last_expired_oldest_age = None;
        let (stale, pre_oldest) = self.pending_tx.drop_stale(cleanup_now, self.tx_budget.frame_max_age);
        if stale > 0 {
            crate::diagnostics::tick::record_tx_dropped_stale(stale as u32);
            self.discard_pending_tx_stopped();
            self.last_expired_oldest_age = pre_oldest;
            return Ok(TxBatchOutcome::Expired { dropped: stale, pending: 0 });
        }
        let mut new_keys = 0usize;
        for (i, target) in targets.iter().enumerate() {
            if self.joint_modes.get(i).copied().flatten() == Some(DamiaoControlMode::PosForce) {
                continue;
            }
            let cmds = match self.pending_target_cmds_checked(i, target) {
                Ok(cmds) => cmds,
                Err(e) => {
                    self.reject_target_tx(MitFenceTripReason::PreflightRejected);
                    return Err(e);
                }
            };
            for (frame_id, _) in cmds.into_iter().flatten() {
                if self.pending_tx.find_key(i, frame_id).is_none() {
                    new_keys += 1;
                }
            }
        }
        if self.pending_tx.len() + new_keys > DAMIAO_PENDING_TX_CAPACITY {
            let queued = self.pending_tx.len();
            self.reject_target_tx(MitFenceTripReason::PreflightRejected);
            return Err(ArmDriverError::TxQueueFull {
                queued,
                new: new_keys,
                capacity: DAMIAO_PENDING_TX_CAPACITY,
            });
        }
        let batch_ts = self.now();
        for (i, target) in targets.iter().enumerate() {
            if self.joint_modes.get(i).copied().flatten() == Some(DamiaoControlMode::PosForce) {
                continue;
            }
            let cmds = match self.pending_target_cmds_checked(i, target) {
                Ok(cmds) => cmds,
                Err(e) => {
                    self.reject_target_tx(MitFenceTripReason::PreflightRejected);
                    return Err(e);
                }
            };
            for (frame_id, cmd) in cmds.into_iter().flatten() {
                let ok = self.pending_tx.upsert(i, frame_id, cmd, batch_ts, batch_stamp);
                debug_assert!(ok, "preflight checked pending queue capacity");
            }
        }
        let pre_arm_intent_stamp = self.current_j7_intent_stamp();
        let arm_out = self.drain_pending_pass(batch_stamp, 0, None, true)?;
        if self.external_pos_force_emitter {
            return Ok(arm_out);
        }
        let Some(pf_j) = self.active_pos_force_joint() else {
            return Ok(arm_out);
        };
        let j7_intent = match arm_out {
            TxBatchOutcome::Expired { .. } => PosForceIntent::hold(pre_arm_intent_stamp),
            _ => PosForceIntent::target(
                targets[pf_j].position,
                targets[pf_j].velocity,
                None,
                pre_arm_intent_stamp,
            ),
        };
        let current_intent_stamp = self.current_j7_intent_stamp();
        match self.emit_pos_force_j7_impl(j7_intent, current_intent_stamp)? {
            PosForceEmitOutcome::NotApplicable => Ok(arm_out),
            PosForceEmitOutcome::Sent { pf_status, .. } => {
                self.last_tx_pf_status = pf_status;
                Ok(arm_out)
            }
            PosForceEmitOutcome::Deferred { pf_status, .. } => {
                self.last_tx_pf_status = pf_status;
                Ok(match arm_out {
                    TxBatchOutcome::Complete => TxBatchOutcome::Deferred { pending: 1 },
                    TxBatchOutcome::Deferred { pending } => TxBatchOutcome::Deferred { pending: pending + 1 },
                    TxBatchOutcome::Expired { dropped, pending } => TxBatchOutcome::Expired { dropped, pending },
                })
            }
        }
    }

    fn discard_pending_tx(&mut self) {
        self.discard_all_pending_tx();
    }

    fn set_tx_release_deadline(&mut self, deadline: Option<Instant>) {
        self.tx_release_deadline = deadline;
        if let (Some(cap), Some(pass)) = (deadline, self.tick_tx.as_mut()) {
            pass.deadline = pass.deadline.min(cap);
        }
    }

    fn tx_frame_max_age(&self) -> Duration {
        self.tx_budget.frame_max_age
    }

    fn last_tx_expired_age(&self) -> Option<Duration> {
        self.last_expired_oldest_age
    }

    fn oldest_pending_tx_age(&self) -> Option<Duration> {
        let now = self.now();
        let pf_age = self.pf_deferred_since.and_then(|since| {
            let pf_j = self.active_pos_force_joint()?;
            self.pos_force_feedback_valid(pf_j, now)
                .then(|| now.saturating_duration_since(since))
        });
        [
            self.pending_tx.oldest_age(now),
            self.pending_hold_tx.oldest_age(now),
            pf_age,
        ]
        .into_iter()
        .flatten()
        .max()
    }

    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        self.receive_feedback()?;
        Ok(self.cached_states.clone())
    }

    fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
        self.receive_feedback()?;
        out.clear();
        out.extend_from_slice(&self.cached_states);
        Ok(())
    }

    fn read_joint_states_budgeted(
        &mut self,
        out: &mut Vec<JointState>,
        deadline: Instant,
        max_rx_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        if !self.can.supports_nonblocking_rt() {
            return Err(ArmDriverError::Protocol(
                "UNSUPPORTED: CAN transport does not support non-blocking RT I/O".into(),
            ));
        }
        self.last_poll_rx = None;
        self.rx_deadline = Some(deadline);
        self.rx_frame_limit = Some(max_rx_frames.min(64));
        let r = self.receive_feedback();
        self.rx_deadline = None;
        self.rx_frame_limit = None;
        r?;
        out.clear();
        out.extend_from_slice(&self.cached_states);
        Ok(self.last_poll_rx.take().unwrap_or((0, false)))
    }

    fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
        out.clear();
        out.extend_from_slice(&self.cached_states);
        Some(())
    }

    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
        out.clear();
        out.extend(self.last_feedback_instant.iter().map(|t| t.map(|t| self.since(t))));
    }
    fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
        self.last_feedback_instant.len() == dof && self.last_feedback_instant.iter().all(|t| t.is_some_and(|t| self.since(t) <= max_age))
    }
    fn feedback_joints_within(&self, joints: &[usize], max_age: Duration) -> bool {
        joints
            .iter()
            .all(|&j| self.last_feedback_instant.get(j).copied().flatten().is_some_and(|t| self.since(t) <= max_age))
    }
    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
        out.clear();
        out.extend(self.last_feedback_instant.iter().map(|t| t.map(|t| self.since(t).as_millis().min(u64::MAX as u128) as u64)));
    }
    fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) { out.clear(); out.extend_from_slice(&self.last_feedback_instant); }
    fn feedback_seqs_into(&self, out: &mut Vec<u64>) { out.clear(); out.extend_from_slice(&self.feedback_seq); }
    fn feedback_queue_bounds_into(&self, out: &mut Vec<Option<Instant>>) { out.clear(); out.extend_from_slice(&self.feedback_queue_bound); }
    fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) { out.clear(); out.extend_from_slice(&self.last_feedback_instant); }
    fn motor_status_into(&self, out: &mut Vec<u8>) -> bool { out.clear(); out.extend_from_slice(&self.motor_status); true }
    fn clear_fault_disables_arm_joint(&self, _arm_dof: usize) -> bool { false }
    fn has_hardware_fault(&self) -> bool {
        self.motor_status.iter().any(|&st| damiao_fault_name(st).is_some())
    }
    fn take_recovery_writes_into(&mut self, out: &mut Vec<usize>) {
        out.clear();
        out.extend_from_slice(&self.recovery_written_joints);
        self.recovery_written_joints.clear();
    }
    fn with_range_check(&self, f: &mut dyn FnMut(Option<&DriverRangeCheck>)) { f(self.range_check.as_ref()) }
    fn set_abort_flag(&mut self, flag: Arc<AtomicBool>) { self.abort_flag = Some(flag); }
    fn set_estop_gate(&mut self, gate: crate::rt_core::urgent::EstopGate) { self.estop_gate = Some(gate); }
}

impl<C: CanTransport> DamiaoArmDriver<C> {
    /// The bounded receive of `read_joint_states` (at most 64 frames or 5 ms) with its
    /// recovery holds and stale-motor refresh; updates `cached_states`.
    /// The receive pass started at `t0` is over: 5 ms, or the caller's service deadline
    /// (`poll_torque`), whichever comes first.
    fn rx_over(&self, t0: Instant) -> bool {
        let now = self.now();
        now.saturating_duration_since(t0) >= DAMIAO_RX_PASS || self.rx_deadline.is_some_and(|d| now >= d)
    }

    /// Decode one received CAN frame into the driver's feedback cache and, when
    /// `record_register_replies` is true, into the register reply buffers.
    fn process_rx_frame(&mut self, frame: &CanFrame, record_register_replies: bool, rx_timestamp: Option<Instant>) {
        self.rx_seq += 1;
        if self.awaiting_rx_boundary {
            return;
        }
        // Ignore transmit commands: on SocketCAN with local echo enabled,
        // host transmit frames for PosVel are 0x100 + tx_id, gripper echo 0x300 + tx_id,
        // or special 0x7FF, or command frames.
        if frame.id == 0x7FF
            || self
                .motor_ids
                .iter()
                .any(|&tx_id| frame.id == (0x100 + tx_id as u32) || frame.id == (0x300 + tx_id as u32))
            || frame.data == damiao::CMD_ENABLE
            || frame.data == damiao::CMD_DISABLE
            || frame.data == damiao::CMD_CLEAR_FAULTS
            || frame.data == damiao::CMD_SET_ZERO
        {
            return;
        }
        // Local echo of an MIT command (same ID as some feedback paths).
        if self.last_target_frames.iter().flatten().any(|f| f.id == frame.id && f.data == frame.data) {
            return;
        }
        if let Some(reply) = damiao::parse_register_reply(frame)
            .filter(|r| self.motor_ids.contains(&r.motor_id))
        {
            // A register reply is not feedback: byte 0 would decode as state 0.
            if record_register_replies {
                let decode_now = self.now();
                let rx_at = rx_timestamp.map_or(decode_now, |ts| ts.min(decode_now));
                if self.register_replies.len() < 64 { self.register_replies.push(reply); }
                if self.register_reply_log.len() < 64 { self.register_reply_log.push((reply, frame.id, rx_at, self.rx_seq, self.rx_empty_seq)); }
            }
            return;
        }
        if !frame.is_extended && !frame.is_rtr && frame.dlc == 8 {
            // Byte 0 names the motor; bind per-motor feedback CAN IDs (own ID and 0x10 + own ID)
            // to that specific joint so another motor's MST_ID (e.g. 0x16) cannot spoof it, while
            // preserving shared-master addressing (0x00 and master_id).
            let fid = damiao::feedback_motor_id(frame);
            let Some(idx) = self.motor_ids.iter().position(|&id| id as u8 == fid) else {
                return;
            };
            let motor_id = u32::from(self.motor_ids[idx]);
            let is_valid_feedback_id = match self.base_mode(idx) {
                DamiaoControlMode::PosForce => frame.id == self.expected_reply_id(idx),
                _ => frame.id == 0
                    || frame.id == self.master_id
                    || frame.id == motor_id
                    || frame.id == (0x10 + motor_id),
            };
            if !is_valid_feedback_id {
                return;
            }
            if let Ok(feedback) = damiao::unpack_feedback_frame(frame, &self.limits[idx]) {
                let decode_now = self.now();
                let rx_at = rx_timestamp.map_or(decode_now, |ts| ts.min(decode_now));
                let queue_bound = self.rx_empty_instant;
                let seq = self.rx_seq;
                self.cached_states[idx] = JointState {
                    position: feedback.position,
                    velocity: feedback.velocity,
                    torque: feedback.torque,
                    temperature: feedback.t_mos.max(feedback.t_rotor),
                };
                self.last_feedback_instant[idx] = Some(rx_at);
                self.feedback_seq[idx] = self.feedback_seq[idx].wrapping_add(1);
                self.feedback_queue_bound[idx] = queue_bound;
                self.last_feedback_seq[idx] = seq;
                self.motor_status[idx] = feedback.state_or_error;
                if self.pos_force.as_ref().is_some_and(|pf| pf.joint == idx) {
                    self.pf_last_fresh_feedback = if feedback.position.is_finite() {
                        queue_bound.map(|qb| (rx_at, qb))
                    } else {
                        None
                    };
                    // Use the EXACT SAME freshness predicate as live checks at `decode_now`:
                    // BOTH `decode_now - rx_at <= 100 ms` AND `decode_now - queue_bound <= 100 ms`
                    // AND finite position must hold, so a frame that sat > 100 ms in the RX queue
                    // (even with an earlier `rx_timestamp` close to `queue_bound`) can NEVER
                    // overwrite `pf_last_fresh_position`.
                    if self.pos_force_feedback_valid(idx, decode_now) {
                        self.pf_last_fresh_position = Some(feedback.position);
                        self.pf_fresh_gen = self.pf_fresh_gen.wrapping_add(1);
                    }
                }
                if feedback.state_or_error != 0 {
                    self.disable_confirmed_sticky = false;
                }
                if let Some(cap) = self.refresh_capture.as_mut() {
                    if cap.joint == idx && seq > cap.boundary && rx_at >= cap.tx_at {
                        let sample = RefreshSample {
                            position: feedback.position,
                            velocity: feedback.velocity,
                            motor_status: feedback.state_or_error,
                            rx_at,
                            queue_bound,
                            seq,
                        };
                        let unsafe_motion_or_state =
                            !(feedback.velocity.is_finite() && feedback.velocity.abs() <= 0.05)
                                || feedback.state_or_error != 0;
                        if unsafe_motion_or_state && cap.unsafe_sample.is_none() {
                            cap.unsafe_sample = Some(sample);
                        }
                        if cap.first.is_none()
                            && !cap.deadline.passed(rx_at)
                            && feedback.position.is_finite()
                            && queue_bound.is_some()
                        {
                            cap.first = Some(sample);
                        }
                    }
                }
            }
        }
    }

    fn mark_rx_empty(&mut self, attempt: Instant) {
        self.rx_empty_instant = Some(attempt);
        self.last_read_drained = true;
        self.rx_seq += 1;
        self.rx_empty_seq = self.rx_seq;
        if self.awaiting_rx_boundary {
            self.awaiting_rx_boundary = false;
            self.last_feedback_instant.fill(None);
            self.pf_last_fresh_feedback = None;
            self.last_enable_command = Some(self.now());
            self.last_enable_command_rx_seq = Some(self.rx_seq);
        }
    }

    fn receive_feedback(&mut self) -> Result<(), ArmDriverError> {
        // Every hardware receive pass opens a fresh per-tick TX budget window; recovery writes,
        // 0xCC refreshes, J7 handshakes, and the tick's target batch share one TickTxPass once any TX starts.
        self.begin_tx_tick();
        let max_frames = self.remaining_rx_frame_limit();
        let t0 = self.now();
        let mut frames_processed = 0;
        let mut rx_drained = false;
        let mut hit_err = false;
        self.last_read_drained = false;
        while frames_processed < max_frames && !self.rx_over(t0) {
            // Taken before the receive: an empty result means the queue was empty after it.
            let attempt = self.now();
            match self.can.receive_timestamped() {
                Ok((frame, rx_ts)) => {
                    frames_processed += 1;
                    self.process_rx_frame(&frame, true, rx_ts);
                    if frames_processed >= max_frames || self.rx_over(t0) {
                        break;
                    }
                }
                Err(TransportError::Timeout) | Err(TransportError::WouldBlock) => {
                    rx_drained = true;
                    self.mark_rx_empty(attempt);
                    break;
                }
                Err(_) => {
                    hit_err = true;
                    break;
                }
            }
        }
        self.account_rx_pass(frames_processed, !rx_drained && !hit_err);
        if self.last_command_was_disable
            && !self.is_enabled
            && !self.is_enabling
            && !self.awaiting_rx_boundary
            && !self.motor_status.is_empty()
            && self.is_feedback_valid()
            && (0..self.motor_ids.len()).all(|i| {
                self.motor_feedback_after(i, self.last_enable_command, self.last_enable_command_rx_seq)
                    && self.motor_status[i] == 0
            })
        {
            self.disable_confirmed_sticky = true;
        }

        // Evaluate feedback age only after receive() explicitly reports an empty
        // queue. If the bounded budget expires first, defer recovery to the next
        // tick so a frame beyond the budget cannot be mistaken for a bus timeout.
        // MIT tracking fence: a stale joint trips it; no recovery frame of any kind is sent
        // (no cached MIT replay, no invented POS_VEL hold) while it is armed or tripped.
        if self.mit_fence.is_some() {
            if rx_drained {
                let _ = self.fence_check();
            } else {
                // the bounded read (64 frames or 5 ms) could not reach the newest sample: the
                // queued latency is unknown and nothing dequeued counts as fresh
                self.trip_fence(MitFenceTripReason::RxBacklog {
                    frames: frames_processed.min(u16::MAX as usize) as u16,
                });
            }
        }
        if self.strict_stop_check(None).is_err() {
            self.discard_pending_tx_stopped();
        }
        let rx_stop_stamp = self.current_stop_stamp();
        let stopped_pending = self.pending_tx.drop_stopped(rx_stop_stamp);
        if stopped_pending > 0 {
            crate::diagnostics::tick::record_tx_dropped_stopped(stopped_pending as u32);
        }
        if self.pending_stop_check(None).is_err() {
            self.discard_pending_hold_tx();
        } else {
            let _ = self.drop_stopped_hold_tx(rx_stop_stamp);
        }
        // Priority J7 0xCC refresh when enabled: if a needed refresh was previously deferred by
        // full-budget contention (`pf_refresh_deferred_ticks >= 1`) OR no output pass ran since the
        // previous RX pass (`pf_refresh_pending && !output_ran_since_rx`), emit ONE 0xCC before
        // competing RX recovery frames so J7 refresh never starves. Otherwise yield the shared TX
        // budget to the upcoming output pass so healthy J1–J6 targets/holds go first.
        let mut rx_pf_refresh_sent = false;
        let enabled_pf_joint = if self.is_enabled { self.active_pos_force_joint() } else { None };
        if let Some(pf_j) = enabled_pf_joint {
            if rx_drained
                && !self.awaiting_rx_boundary
                && !self.is_enabling
                && self.poll_tx_allowed()
                && self.pending_stop_check(Some(rx_stop_stamp)).is_ok()
            {
                if self.pos_force_feedback_valid(pf_j, self.now()) {
                    self.pf_refresh_pending = false;
                    self.pf_refresh_deferred_ticks = 0;
                } else {
                    if self.pf_refresh_pending
                        && (self.pf_refresh_deferred_ticks >= 1 || !self.output_ran_since_rx)
                    {
                        if let Ok(true) = self.try_send_pf_refresh(rx_stop_stamp) {
                            rx_pf_refresh_sent = true;
                        }
                    } else {
                        self.pf_refresh_pending = true;
                    }
                }
            }
        }
        if rx_drained && self.is_enabled && !self.is_enabling && self.recovery_tx_allowed()
            && self.enable_state() == EnableState::Enabled {
            let n = self.motor_ids.len();
            for i in 0..n {
                if self.strict_stop_check(Some(rx_stop_stamp)).is_err() {
                    self.discard_pending_tx_stopped();
                    break;
                }
                let stale = self
                    .last_feedback_instant
                    .get(i)
                    .and_then(|&opt| opt)
                    .is_none_or(|t| self.since(t) >= Duration::from_millis(45));
                if !stale {
                    continue;
                }

                // Recovery never replays a queued motion target. When owned by `SupervisorInner`
                // (`external_pos_force_emitter == true`), `PosForce` J7 frames are emitted solely
                // by the tick epilogue (never duplicated here); standalone driver callers still
                // route through the ONE `emit_pos_force_frame` helper (`allow_freeze = false`).
                let resolved_mode = match self.joint_modes.get(i).copied().flatten() {
                    Some(m) => m,
                    None if self.base_mode(i) == DamiaoControlMode::PosForce => continue,
                    None => self.control_mode,
                };
                let send_res = if resolved_mode == DamiaoControlMode::PosForce {
                    if self.external_pos_force_emitter {
                        continue;
                    }
                    let Some(pf) = self.pos_force.as_ref().filter(|pf| pf.joint == i) else {
                        continue;
                    };
                    let Some(pos) = self.fresh_measured_position(i, POS_FORCE_FEEDBACK_MAX_AGE) else {
                        continue;
                    };
                    let id = self.motor_ids[i];
                    let last_raw = self.last_target_frames[i]
                        .filter(|f| f.id == u32::from(damiao::POS_FORCE_BASE_ID + id))
                        .map(|f| u16::from_le_bytes([f.data[4], f.data[5]]))
                        .filter(|&r| r >= 1);
                    let recovery_vel = last_raw
                        .map(crate::model::DamiaoGripperDrive::wire_speed_f32)
                        .map_or(pf.close_speed_rad_s, |v| v.min(pf.close_speed_rad_s));
                    self.emit_pos_force_frame(
                        TxKind::Recovery,
                        Some(rx_stop_stamp),
                        PosForceIntentKind::Target {
                            position: pos,
                            velocity: Some(recovery_vel),
                            current_raw: None,
                        },
                        false,
                        false,
                        false,
                    )
                } else {
                    self.send(TxKind::Recovery, Some(rx_stop_stamp), None, false, |d| {
                        let id = d.motor_ids[i];
                        let p = d.cached_states.get(i).map_or(0.0, |s| s.position);
                        match resolved_mode {
                            DamiaoControlMode::PosForce => None,
                            DamiaoControlMode::Mit if d.control_mode == DamiaoControlMode::PosVel => {
                                // A joint switched to MIT by set_joint_modes (gravity compensation):
                                // rebuild a fresh stationary MIT hold from current fresh measured state
                                // (same <= 100 ms + observed-empty receive bound as J7 PosForce) and
                                // the last MIT gains/torque instead of replaying cached bytes.
                                let pos = d.fresh_measured_position(i, POS_FORCE_FEEDBACK_MAX_AGE)?;
                                let (kp, kd, torque) = d.last_mit_params.get(i).copied().flatten()?;
                                Some(damiao::build_mit_frame(
                                    id,
                                    pos,
                                    0.0,
                                    kp,
                                    kd,
                                    torque,
                                    &d.limits[i],
                                ))
                            }
                            DamiaoControlMode::Mit => Some(damiao::build_mit_frame(
                                id,
                                p,
                                0.0,
                                d.default_kp,
                                d.default_kd,
                                0.0,
                                &d.limits[i],
                            )),
                            DamiaoControlMode::PosVel => Some(damiao::build_pos_vel_frame(
                                0x100,
                                id,
                                p,
                                d.default_vlim.abs().clamp(0.1, 4.0),
                            )),
                        }
                    })
                };
                let recovery_sent = match send_res {
                    Ok(sent_frame) => {
                        self.last_target_frames[i] = Some(sent_frame);
                        true
                    }
                    Err(TxRefusal::Deferred) => break,
                    Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                        self.discard_pending_tx_stopped();
                        break;
                    }
                    Err(TxRefusal::BuildRefused | TxRefusal::PosForceFeedbackExpired(_)) => continue,
                    Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => {
                        if self.strict_stop_check(Some(rx_stop_stamp)).is_err() {
                            self.discard_pending_tx_stopped();
                        }
                        break;
                    }
                    Err(_) => false,
                };
                if recovery_sent {
                    self.discard_pending_tx_stopped();
                    if !self.recovery_written_joints.contains(&i) {
                        self.recovery_written_joints.push(i);
                    }
                }
                if self.strict_stop_check(Some(rx_stop_stamp)).is_err() {
                    self.discard_pending_tx_stopped();
                    break;
                }
            }
        }
        // Damiao motors only transmit in reply to a frame. When feedback goes stale
        // (idle, bus glitch, motor power dip) nothing would ever refresh it, so a
        // fault could never be cleared. Poll stale motors with the read-only 0xCC
        // refresh (`TxKind::Refresh`: allowed while latched, refused while pending).
        const REFRESH_AFTER: Duration = Duration::from_millis(100);
        if rx_drained
            && !self.awaiting_rx_boundary
            && !self.is_enabling
            && self.poll_tx_allowed()
            && self.pending_stop_check(Some(rx_stop_stamp)).is_ok()
        {
            let cooldown_ready = self.last_refresh_instant.is_none_or(|t| self.since(t) >= REFRESH_AFTER);
            if cooldown_ready || self.refresh_unfinished != 0 {
                // the stale motors as a bit set taken before the first transmit (no allocation)
                let n = self.motor_ids.len().min(self.last_feedback_instant.len()).min(64);
                let rx_now = self.now();
                let stale = (0..n)
                    .filter(|&i| {
                        // When enabled, the single J7 refresh scheduler owns the active PosForce
                        // joint so RX does not double-poll or steal J1–J6 budget on tick 1.
                        if enabled_pf_joint == Some(i) {
                            return false;
                        }
                        if self.active_pos_force_joint() == Some(i) {
                            return !self.pos_force_feedback_valid(i, rx_now);
                        }
                        self.last_feedback_instant[i].is_none_or(|t| self.since(t) >= REFRESH_AFTER)
                    })
                    .fold(0u64, |m, i| m | (1 << i));
                let mut remaining = if self.refresh_unfinished != 0 {
                    let cont = stale & self.refresh_unfinished;
                    if cont != 0 {
                        cont
                    } else if cooldown_ready {
                        stale
                    } else {
                        0
                    }
                } else {
                    stale
                };
                if remaining != 0 && n > 0 {
                    let _ = self.ensure_tx_pass();
                    let start = self.refresh_cursor % n;
                    let mut refresh_sent = false;
                    let mut stopped_during_refresh = false;
                    for offset in 0..n {
                        let i = (start + offset) % n;
                        if remaining & (1 << i) == 0 {
                            continue;
                        }
                        match self.send(TxKind::Refresh, Some(rx_stop_stamp), None, false, |d| {
                            Some(damiao::build_refresh_frame(d.motor_ids[i]))
                        }) {
                            Ok(_) => {
                                refresh_sent = true;
                                remaining &= !(1 << i);
                                self.refresh_cursor = (i + 1) % n;
                            }
                            Err(TxRefusal::Deferred) => break,
                            Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                                self.discard_all_pending_tx();
                                stopped_during_refresh = true;
                                break;
                            }
                            Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => {
                                if self.pending_stop_check(Some(rx_stop_stamp)).is_err() {
                                    self.discard_all_pending_tx();
                                    stopped_during_refresh = true;
                                }
                                break;
                            }
                            Err(_) => {
                                remaining &= !(1 << i);
                                self.refresh_cursor = (i + 1) % n;
                            }
                        }
                    }
                    if (refresh_sent || rx_pf_refresh_sent)
                        && !stopped_during_refresh
                        && self.pending_stop_check(Some(rx_stop_stamp)).is_ok()
                    {
                        if self.flush_cutoff_exceeded() {
                            self.mark_tx_pass_backpressured();
                            self.discard_all_pending_tx();
                            return Err(TransportError::Timeout.into());
                        }
                        if let Err(e) = self.can.flush() {
                            self.mark_tx_pass_backpressured();
                            self.discard_all_pending_tx();
                            return Err(e.into());
                        }
                        rx_pf_refresh_sent = false;
                    }
                    self.refresh_unfinished = remaining;
                    if remaining == 0 && refresh_sent {
                        self.last_refresh_instant = Some(self.now());
                    }
                } else {
                    self.refresh_unfinished = 0;
                }
            }
        }
        if rx_pf_refresh_sent && self.pending_stop_check(Some(rx_stop_stamp)).is_ok() {
            if self.flush_cutoff_exceeded() {
                self.mark_tx_pass_backpressured();
                self.discard_all_pending_tx();
                return Err(TransportError::Timeout.into());
            }
            if let Err(e) = self.can.flush() {
                self.mark_tx_pass_backpressured();
                self.discard_all_pending_tx();
                return Err(e.into());
            }
        }
        self.output_ran_since_rx = false;
        crate::diagnostics::tick::record_tx_deferred(0, self.total_pending_tx_len());
        Ok(())
    }
}

// =========================================================================
// 4. RobStride Actuator Arm Driver (reBot Arm B601-RS)
// =========================================================================

pub struct RobStrideArmDriver<C: CanTransport> {
    can: C,
    motor_ids: Vec<u8>,
    master_id: u8,
    limits: RobStrideLimits,
    default_kp: f32,
    default_kd: f32,
    cached_states: Vec<JointState>,
    last_feedback_instant: Vec<Option<Instant>>,
    feedback_seq: Vec<u64>,
    /// Instant taken before the last receive call that found the RX queue empty.
    rx_empty_instant: Option<Instant>,
    /// `rx_empty_instant` when each joint's last sample was decoded.
    feedback_queue_bound: Vec<Option<Instant>>,
    fault_flags: Vec<u8>,
}

impl<C: CanTransport> RobStrideArmDriver<C> {
    pub fn new(can: C, motor_ids: Vec<u8>) -> Self {
        let dof = motor_ids.len();
        Self {
            can,
            motor_ids,
            master_id: 0xFD,
            limits: RobStrideLimits::default(),
            default_kp: 40.0,
            default_kd: 1.0,
            cached_states: vec![JointState::default(); dof],
            last_feedback_instant: vec![None; dof],
            feedback_seq: vec![0; dof],
            rx_empty_instant: None,
            feedback_queue_bound: vec![None; dof],
            fault_flags: vec![0; dof],
        }
    }

    pub fn with_gains(mut self, kp: f32, kd: f32) -> Self {
        self.default_kp = kp;
        self.default_kd = kd;
        self
    }

    pub fn can_mut(&mut self) -> &mut C {
        &mut self.can
    }
}

impl<C: CanTransport> ArmDriver for RobStrideArmDriver<C> {
    fn motor_zero_support(&self) -> MotorZeroSupport {
        MotorZeroSupport::unsupported("RobStride: src/protocol/robstride.rs has TYPE_SET_ZERO (communication type 6) but no \
            verified zero sequence: motorbridge, the SDK of the official reBotArm_control_py, lists RobStride zero calibration \
            as unresolved (device-side zero_sta / mechPos verification can fail), and whether type 6 persists without \
            type 22 (save parameters) is not checked")
    }

    fn capabilities(&self) -> DriverCapabilities { DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true, torque_feedforward: true, enable_feedback: false, load_feedback: false } }

    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        self.last_feedback_instant.iter().map(|t| t.map(|t| t.elapsed())).collect()
    }

    fn feedback_instants(&self) -> Vec<Option<Instant>> {
        self.last_feedback_instant.clone()
    }

    fn feedback_seqs(&self) -> Vec<u64> {
        self.feedback_seq.clone()
    }

    fn feedback_queue_bounds(&self) -> Vec<Option<Instant>> {
        self.feedback_queue_bound.clone()
    }

    fn feedback_receive_instants(&self) -> Vec<Option<Instant>> {
        self.last_feedback_instant.clone()
    }

    fn is_feedback_valid(&self) -> bool {
        if self.last_feedback_instant.is_empty() {
            return false;
        }
        const FRESHNESS_LIMIT: Duration = Duration::from_millis(250);
        self.last_feedback_instant
            .iter()
            .all(|opt| opt.is_some_and(|instant| instant.elapsed() <= FRESHNESS_LIMIT))
    }

    fn read_joint_states_budgeted(
        &mut self,
        out: &mut Vec<JointState>,
        deadline: Instant,
        max_rx_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        let max_frames = usize::from(max_rx_frames.min(64));
        let t0 = Instant::now();
        let mut frames_processed = 0;
        let mut rx_drained = false;
        let mut hit_err = false;
        while frames_processed < max_frames && Instant::now() < deadline && t0.elapsed() < Duration::from_micros(2000) {
            let attempt = Instant::now();
            match self.can.receive() {
                Ok(frame) => {
                    frames_processed += 1;
                    if let Ok(feedback) = robstride::unpack_feedback_frame(&frame, &self.limits) {
                        if let Some(idx) = self.motor_ids.iter().position(|&id| id == feedback.motor_id) {
                            self.cached_states[idx] = JointState {
                                position: feedback.position,
                                velocity: feedback.velocity,
                                torque: feedback.torque,
                                temperature: feedback.mos_temperature,
                            };
                            self.last_feedback_instant[idx] = Some(Instant::now());
                            self.feedback_seq[idx] = self.feedback_seq[idx].wrapping_add(1);
                            self.feedback_queue_bound[idx] = self.rx_empty_instant;
                            self.fault_flags[idx] = feedback.fault_flags;
                        }
                    }
                }
                Err(TransportError::Timeout) | Err(TransportError::WouldBlock) => {
                    rx_drained = true;
                    self.rx_empty_instant = Some(attempt);
                    break;
                }
                Err(_) => {
                    hit_err = true;
                    break;
                }
            }
        }
        out.clear();
        out.extend_from_slice(&self.cached_states);
        Ok((frames_processed as u16, !rx_drained && !hit_err))
    }

    fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
        out.clear();
        out.extend_from_slice(&self.cached_states);
        Some(())
    }

    fn check_hardware_status(&self) -> Result<(), ArmDriverError> {
        for (i, &flags) in self.fault_flags.iter().enumerate() {
            if flags != 0 {
                let motor_id = self.motor_ids.get(i).copied().unwrap_or(0);
                return Err(ArmDriverError::HardwareFault(format!(
                    "RobStride Joint {} (CAN ID 0x{:X}) reported fault flags: 0x{:02X}",
                    i, motor_id, flags
                )));
            }
        }
        Ok(())
    }

    fn clear_fault_disables_arm_joint(&self, _arm_dof: usize) -> bool { false }

    fn has_hardware_fault(&self) -> bool {
        self.fault_flags.iter().any(|&flags| flags != 0)
    }

    fn clear_fault(&mut self) -> Result<(), ArmDriverError> {
        let mut err = None;
        for &id in &self.motor_ids {
            let frame = robstride::build_clear_fault_frame(id, self.master_id);
            if let Err(e) = self.can.transmit(&frame) {
                err = Some(e);
            }
        }
        if let Err(e) = self.can.flush() { err = Some(e); }
        // A sent reset is not proof that the reported fault has cleared.
        // The next telemetry response updates fault_flags.
        if let Some(e) = err {
            Err(ArmDriverError::HardwareFault(e.to_string()))
        } else {
            Ok(())
        }
    }

    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        if !enabled {
            let mut last_err = None;
            for &id in &self.motor_ids {
                let frame = robstride::build_stop_frame(id, self.master_id);
                if let Err(e) = self.can.transmit(&frame) {
                    last_err = Some(e.into());
                }
            }
            if let Err(e) = self.can.flush() {
                last_err = Some(e.into());
            }
            if let Some(e) = last_err {
                return Err(e);
            }
            Ok(())
        } else {
            for &id in &self.motor_ids {
                let frame = robstride::build_enable_frame(id, self.master_id);
                self.can.transmit(&frame)?;
            }
            self.can.flush()?;
            Ok(())
        }
    }

    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        if targets.len() != self.motor_ids.len() {
            return Err(ArmDriverError::JointCountMismatch {
                expected: self.motor_ids.len(),
                actual: targets.len(),
            });
        }

        for (i, target) in targets.iter().enumerate() {
            let kp = target.kp.unwrap_or(self.default_kp);
            let kd = target.kd.unwrap_or(self.default_kd);
            let vel = target.velocity.unwrap_or(0.0);
            let torque = target.torque.unwrap_or(0.0);

            let frame = robstride::build_mit_frame(
                self.motor_ids[i],
                target.position,
                vel,
                kp,
                kd,
                torque,
                &self.limits,
            );
            let t0 = Instant::now();
            let cpu0 = crate::diagnostics::tick::thread_cpu_ns();
            let res = self.can.transmit(&frame);
            let wall_ns = t0.elapsed().as_nanos().min(u128::from(u64::MAX)) as u64;
            let cpu_ns = crate::diagnostics::tick::thread_cpu_ns().saturating_sub(cpu0);
            let kind = match &res {
                Ok(()) => crate::diagnostics::tick::TxAttemptKind::Sent,
                Err(TransportError::WouldBlock) => crate::diagnostics::tick::TxAttemptKind::WouldBlock,
                Err(TransportError::NoBufferSpace) => crate::diagnostics::tick::TxAttemptKind::Enobufs,
                Err(_) => crate::diagnostics::tick::TxAttemptKind::Error,
            };
            crate::diagnostics::tick::record_tx_attempt(kind, wall_ns, cpu_ns);
            res?;
        }
        self.can.flush()?;
        Ok(())
    }

    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let t0 = Instant::now();
        let mut frames_processed = 0;
        while frames_processed < 64 {
            // Taken before the receive: an empty result means the queue was empty after it.
            let attempt = Instant::now();
            match self.can.receive() {
                Ok(frame) => {
                    frames_processed += 1;
                    if let Ok(feedback) = robstride::unpack_feedback_frame(&frame, &self.limits) {
                        if let Some(idx) = self
                            .motor_ids
                            .iter()
                            .position(|&id| id == feedback.motor_id)
                        {
                            self.cached_states[idx] = JointState {
                                position: feedback.position,
                                velocity: feedback.velocity,
                                torque: feedback.torque,
                                temperature: feedback.mos_temperature,
                            };
                            self.last_feedback_instant[idx] = Some(Instant::now());
                            self.feedback_seq[idx] = self.feedback_seq[idx].wrapping_add(1);
                            self.feedback_queue_bound[idx] = self.rx_empty_instant;
                            self.fault_flags[idx] = feedback.fault_flags;
                        }
                    }
                    if frames_processed >= 64 || t0.elapsed() >= Duration::from_micros(2000) {
                        break;
                    }
                }
                Err(TransportError::Timeout) | Err(TransportError::WouldBlock) => {
                    self.rx_empty_instant = Some(attempt);
                    break;
                }
                Err(_) => break,
            }
        }
        Ok(self.cached_states.clone())
    }
}

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
        rx: Arc<Mutex<VecDeque<CanFrame>>>,
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
            self.rx
                .lock()
                .unwrap()
                .push_back(CanFrame::new_standard(0x10 + motor_id, &data).unwrap());
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
            self.rx.lock().unwrap().pop_front().ok_or(TransportError::WouldBlock)
        }
        fn flush(&mut self) -> Result<(), TransportError> {
            Ok(())
        }
    }

    const ALL_TX_KINDS: [TxKind; 9] = [
        TxKind::Disable,
        TxKind::Target,
        TxKind::Hold,
        TxKind::RollbackHold(0),
        TxKind::ModeWrite,
        TxKind::Refresh,
        TxKind::Recovery,
        TxKind::ClearFault,
        TxKind::Handshake,
    ];

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
        // Step 2: Tick TX budget runs before stop gate (Disable exempt).
        if budget == BudgetCase::Exhausted && !matches!(kind, TxKind::Disable) {
            return Expected::Deferred;
        }
        // Step 3: Stop gate by TxKind.
        match kind {
            TxKind::Disable => Expected::Sent,
            TxKind::Refresh | TxKind::RollbackHold(_) | TxKind::ClearFault => match stop {
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

        for &kind in &ALL_TX_KINDS {
            for &stop in &[StopCase::NoStop, StopCase::Pending, StopCase::Latched] {
                for &budget in &[BudgetCase::Available, BudgetCase::Exhausted] {
                    for &deadline_case in &[DeadlineCase::Ok, DeadlineCase::Passed] {
                        let (bus, clock) = TestBus::new();
                        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1])
                            .with_limits(limits);

                        // Establish an empty-queue receive bound + fresh feedback on J1 so
                        // RollbackHold(0) satisfies step 4 freshness whenever step 3 admits it.
                        drv.read_joint_states().unwrap();
                        bus.push_feedback(1, 0.25, 1, &limits);
                        drv.read_joint_states().unwrap();
                        bus.clear_tx();

                        // Configure stop/E-stop state.
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
                        let res = drv.send(kind, Some(stamp), deadline, false, |d| {
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
        assert_eq!(cells_checked, ALL_TX_KINDS.len() * 3 * 2 * 2);
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
        let stale_res = drv.send(TxKind::RollbackHold(0), None, None, false, |d| {
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
            .send(TxKind::RollbackHold(0), None, None, false, |d| {
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
            drv.send(TxKind::Target, None, None, false, |_| None),
            Err(TxRefusal::BuildRefused)
        ));
        assert_eq!(bus.tx_len(), 0);
        // Budget slot was NOT consumed by BuildRefused, so the next valid frame succeeds.
        assert!(drv
            .send(TxKind::Target, None, None, false, |d| Some(d.pos_vel_frame(0, 0.0)))
            .is_ok());
        assert_eq!(bus.tx_len(), 1);

        // 3. Step 5 stop-generation stamp check: applied stop between admission and send
        //    refuses every non-Disable kind with StopStampChanged (0 TX), while Disable sends.
        for &kind in &ALL_TX_KINDS {
            let (bus, _clock) = TestBus::new();
            let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1]).with_limits(limits);
            let stops = StopRequests::new();
            drv.set_estop_gate(
                EstopGate::new(Arc::new(AtomicU64::new(0)), Arc::new(AtomicU64::new(0)))
                    .with_stops(stops.clone()),
            );
            let old_stamp = drv.current_stop_stamp();
            let g = stops.request();
            stops.mark_applied(g);
            bus.clear_tx();
            let res = drv.send(kind, Some(old_stamp), None, false, |d| {
                Some(damiao::build_disable_frame(d.motor_ids[0]))
            });
            if matches!(kind, TxKind::Disable) {
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
            let res = drv.send(kind, Some(stamp), None, false, |d| {
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

            let res = drv.send(kind, None, None, false, |d| {
                req.fetch_add(1, Ordering::SeqCst);
                Some(damiao::build_refresh_frame(d.motor_ids[0]))
            });
            assert!(
                matches!(res, Err(TxRefusal::StopInhibited)),
                "pending stop raised during build must refuse {kind:?} with StopInhibited, got {res:?}"
            );
            assert_eq!(bus.tx_len(), 0);
        }

        // C. Safety `Disable` still transmits even if both latch and pending flip during `build`.
        let (bus, _clock) = TestBus::new();
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1]).with_limits(limits);
        let req = Arc::new(AtomicU64::new(0));
        let app = Arc::new(AtomicU64::new(0));
        let latched = Arc::new(AtomicBool::new(false));
        drv.set_estop_gate(EstopGate::new(req.clone(), app));
        drv.set_abort_flag(latched.clone());
        let res = drv.send(TxKind::Disable, None, None, false, |d| {
            latched.store(true, Ordering::SeqCst);
            req.fetch_add(1, Ordering::SeqCst);
            Some(damiao::build_disable_frame(d.motor_ids[0]))
        });
        assert!(res.is_ok(), "Disable must remain ungated across build-time stop flip: {res:?}");
        assert_eq!(bus.tx_len(), 1);
    }

    #[test]
    fn pos_force_boot_fallback_with_no_fresh_sample_emits_zero_vel_zero_current() {
        let limits = damiao::DamiaoLimits::default();
        let (bus, _clock) = TestBus::new();
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

        assert_eq!(POS_FORCE_BOOT_FALLBACK, PosForceBootFallback::ZeroCurrent);
        assert_eq!(pos_force_boot_fallback_current(Some(320), 460), 0);
        assert_eq!(pos_force_boot_fallback_current(None, 460), 0);

        // 1. No prior target ever commanded -> emits 0x307 with pos=0.0, vel_raw=0, i_des=0.
        bus.clear_tx();
        drv.begin_tx_tick();
        let stamp = drv.current_j7_intent_stamp();
        let out0 = drv.emit_pos_force_j7(PosForceIntent::stop(stamp), stamp).expect("boot fallback emit");
        assert!(matches!(out0, PosForceEmitOutcome::Sent { joint: 0, freeze: true, .. }));
        let log0: Vec<CanFrame> = bus.tx.lock().unwrap().clone();
        let j7_0: Vec<CanFrame> = log0.iter().copied().filter(|f| f.id == 0x307).collect();
        assert_eq!(j7_0.len(), 1, "expected 1 boot-fallback 0x307 frame: {log0:?}");
        let frame0 = j7_0[0];
        assert_eq!(frame0.id, 0x307);
        assert_eq!(f32::from_le_bytes(frame0.data[0..4].try_into().unwrap()), 0.0);
        assert_eq!(u16::from_le_bytes([frame0.data[4], frame0.data[5]]), 0);
        assert_eq!(u16::from_le_bytes([frame0.data[6], frame0.data[7]]), 0);
        assert!(drv.is_pos_force_freeze_frame(&frame0));

        // 2. Prior commanded target exists (-1.25 rad) -> holds -1.25 with vel_raw=0, i_des=0 (zero force).
        drv.last_target_frames[0] = Some(damiao::build_pos_force_frame(
            damiao::POS_FORCE_BASE_ID,
            7,
            -1.25,
            30,
            460,
        ));
        bus.clear_tx();
        drv.begin_tx_tick();
        let targets = [JointTarget::position(0.0)];
        let outcome = drv.write_estop_hold_bounded(&targets).expect("boot fallback hold");
        assert_eq!(outcome, TxBatchOutcome::Complete);
        let log: Vec<CanFrame> = bus.tx.lock().unwrap().clone();
        let j7_frames: Vec<CanFrame> = log.iter().copied().filter(|f| f.id == 0x307).collect();
        assert_eq!(j7_frames.len(), 1, "expected 1 boot-fallback 0x307 frame: {log:?}");
        assert!(
            log.iter().any(|f| f.id == 0x7FF && f.data[0] == 7 && f.data[2] == 0xCC),
            "expected 0xCC refresh alongside boot fallback: {log:?}"
        );
        let sent = j7_frames[0];
        assert_eq!(sent.id, 0x307);
        assert!((f32::from_le_bytes(sent.data[0..4].try_into().unwrap()) - (-1.25)).abs() < 1e-6);
        assert_eq!(u16::from_le_bytes([sent.data[4], sent.data[5]]), 0);
        assert_eq!(u16::from_le_bytes([sent.data[6], sent.data[7]]), 0);
        assert!(drv.is_pos_force_freeze_frame(&sent));
    }
}
