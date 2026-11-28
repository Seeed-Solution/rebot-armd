//! CAN brushless-servo driver family.

pub(crate) mod damiao;
mod engine;
mod gripper_j7;
mod mit;
mod modes;
mod registers;
mod robstride;
mod torque;

pub use engine::DamiaoArmDriver;
pub use gripper_j7::{
    gripper_drive_from_profile, GripperDrive, GripperPosForce, GripperPosForceInfo,
    MotorCurrentCap,
};
pub use modes::DamiaoControlMode;
pub use robstride::RobStrideArmDriver;

pub(crate) use engine::tx_types::OPTIONAL_WORK_RELEASE_MARGIN;
pub(crate) use gripper_j7::pos_force_emitter::PosForceSendToken;

pub(in crate::driver) use engine::tx_types::{StopStamp, TxKind, TxRefusal};
pub(in crate::driver) use gripper_j7::{
    pos_force_emitter, ContinuityBypass, DamiaoTxKind, POS_FORCE_FEEDBACK_MAX_AGE,
};
pub(in crate::driver) use modes::{ModeWrite, MAX_MODE_JOINTS};
pub(in crate::driver) use registers::REGISTER_REPLY_TIMEOUT;
pub(in crate::driver) use torque::ENABLE_FRESHEN_TIMEOUT;

use engine::recovery::{
    can_rx_time_to_monotonic, check_rest_pose_boundary, RestBoundaryCheck,
    RestPoseBoundary,
};
#[cfg(not(target_arch = "wasm32"))]
use engine::recovery::{
    can_rx_reject_to_shadow_stamp, control_mode_to_damiao, damiao_mode_to_shadow_control_mode,
};
use engine::tx_types::{
    PendingTargetCmd, PendingTxFrame, PendingTxQueue, RefreshScheduler, TickTxPass,
    TxProbeState, DAMIAO_MAX_RECOVERY_PER_RELEASE, DAMIAO_REFRESH_AFTER, TX_CLASS_HOLD_RESEND,
    TX_CLASS_MOTION_TARGET, TX_CLASS_STOP_TRANSITION, WATCHDOG_DISABLE_FALLBACK_INTERVAL,
    WATCHDOG_ENABLE_ABORT_AGE, WATCHDOG_HOLD_PROACTIVE_INTERVAL, WATCHDOG_HOLD_URGENT_INTERVAL,
    WATCHDOG_HOLD_WAKE_INTERVAL,
};
use gripper_j7::{
    MitRecoveryHoldErr, MAX_ENABLE_PF_REFRESHES,
    POS_FORCE_READBACK_FRESH_MARGIN,
};
use mit::recovery::{
    MitExpiryAxisState, MIT_RECOVERY_CONTINUITY_HOLD_INTERVAL,
};
use modes::{ModeReadback, ModeWritePolicy, ReadbackStage};
use torque::{
    DamiaoTorque, EnableFreshen, FeedbackRefresh, RangeCheckPhase, RefreshCapture,
    RefreshSample, RefreshStep, ENABLE_FRESHEN_ENTRY_ARM_MAX_AGE,
    ENABLE_FRESHEN_ENTRY_PF_MAX_AGE, ENABLE_FRESHEN_EXIT_ARM_MAX_AGE,
    ENABLE_FRESHEN_EXIT_PF_MAX_AGE, ENABLE_FRESHEN_RETRY_INTERVAL,
    RANGE_CHECK_REPLY_TIMEOUT, TICKED_RANGE_CHECK_REPLY_TIMEOUT,
};
