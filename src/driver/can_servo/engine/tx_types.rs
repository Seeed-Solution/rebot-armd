//! Damiao bounded TX queue, pass accounting, refresh scheduler, and TX priority types.

#[allow(unused_imports)]
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
#[allow(unused_imports)]
use std::sync::Arc;
#[allow(unused_imports)]
use std::time::{Duration, Instant};

#[allow(unused_imports)]
use crate::driver::*;
#[allow(unused_imports)]
use crate::driver::can_servo::*;
#[allow(unused_imports)]
use crate::model::JointState;
#[allow(unused_imports)]
use crate::protocol::damiao::{self, DamiaoLimits};
#[allow(unused_imports)]
use crate::transport::{CanFrame, CanRxTime, CanTransport, TransportError, MAX_REALTIME_MONOTONIC_OFFSET_DRIFT_NS};

/// Transmit priority classes (lower value = higher priority):
/// 1. Transition from motion to stop / hold / freeze for ANY joint (including J7).
/// 2. Motion targets.
/// 3. Re-sends of an unchanged hold / freeze.
///
/// (Class 4: J7 `0xCC` refresh and other reads — sent only when Classes 1–3 have no pending work
/// and spare TX budget remains.)
pub(in crate::driver::can_servo) const TX_CLASS_STOP_TRANSITION: u8 = 1;
pub(in crate::driver::can_servo) const TX_CLASS_MOTION_TARGET: u8 = 2;
pub(in crate::driver::can_servo) const TX_CLASS_HOLD_RESEND: u8 = 3;

#[derive(Clone, Copy, Debug)]
pub(in crate::driver::can_servo) struct TxProbeState {
    pub(in crate::driver::can_servo) joint: usize,
    pub(in crate::driver::can_servo) boundary_seq: u64,
    pub(in crate::driver::can_servo) sent_at: Instant,
}

#[derive(Clone, Copy, Debug)]
pub(in crate::driver::can_servo) struct TickTxPass {
    pub(in crate::driver::can_servo) deadline: Instant,
    pub(in crate::driver::can_servo) attempts: usize,
    pub(in crate::driver::can_servo) backpressured: bool,
}

/// Generation stamp captured when a motion frame is admitted into [`PendingTxQueue`].
/// Any E-stop or authorized stop (even after `requested == applied`) changes this stamp so
/// older deferred frames are never transmitted.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(in crate::driver::can_servo) struct StopStamp {
    pub(in crate::driver::can_servo) estop_gen: u64,
    pub(in crate::driver::can_servo) stop_gen: u64,
}

/// Classification of every Damiao CAN transmit routed through [`DamiaoArmDriver::send`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(in crate::driver) enum TxKind {
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
pub(in crate::driver) enum TxRefusal {
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
