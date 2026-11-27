//! Damiao CAN register IDs, constants, limits, and protocol data types.

use thiserror::Error;
use crate::motor_mode::{ControlMode, ImpedanceLimits};

/// Special motor control commands
pub const CMD_ENABLE: [u8; 8] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC];
pub const CMD_DISABLE: [u8; 8] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD];
pub const CMD_SET_ZERO: [u8; 8] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE];
pub const CMD_CLEAR_FAULTS: [u8; 8] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB];

/// Serial bridge packet header
pub const SERIAL_BRIDGE_HEADER: [u8; 2] = [0xAA, 0x55];

/// MIT-frame and feedback mapping ranges of one Damiao motor. The motor maps the
/// 16 / 12-bit fields onto [-PMAX, PMAX], [-VMAX, VMAX] and [-TMAX, TMAX] (registers
/// 21 / 22 / 23), so these must equal that motor's registers. `Default` is the
/// single range the driver used before per-joint ranges (12.5 / 30 / 18); it does
/// not match every motor (on the B601-DM, J1-J3 report 12.5 / 10 / 28 and J4-J7
/// 12.5 / 30 / 10).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DamiaoLimits {
    pub p_min: f32,
    pub p_max: f32,
    pub v_min: f32,
    pub v_max: f32,
    pub kp_min: f32,
    pub kp_max: f32,
    pub kd_min: f32,
    pub kd_max: f32,
    pub t_min: f32,
    pub t_max: f32,
}

impl Default for DamiaoLimits {
    fn default() -> Self {
        Self {
            p_min: -12.5,
            p_max: 12.5,
            v_min: -30.0,
            v_max: 30.0,
            kp_min: 0.0,
            kp_max: 500.0,
            kd_min: 0.0,
            kd_max: 5.0,
            t_min: -18.0,
            t_max: 18.0,
        }
    }
}

impl DamiaoLimits {
    /// Symmetric position / velocity / torque ranges with the default gain ranges.
    pub fn symmetric(p_max: f32, v_max: f32, t_max: f32) -> Self {
        Self { p_min: -p_max, p_max, v_min: -v_max, v_max, t_min: -t_max, t_max, ..Self::default() }
    }

    /// Build wire mapping ranges from typed [`ImpedanceLimits`].
    ///
    /// Wire gain ranges stay at Damiao's firmware constants (`kp: 0..=500`, `kd: 0..=5`);
    /// `lim.kp_max` / `lim.kd_max` act as pre-quantization command ceilings in [`encode_command`].
    pub fn from_impedance_limits(lim: &ImpedanceLimits) -> Self {
        Self::symmetric(lim.p_max, lim.v_max, lim.t_max)
    }
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum DamiaoError {
    #[error("Incomplete frame: expected {expected} bytes, got {actual}")]
    IncompleteFrame { expected: usize, actual: usize },

    #[error("Checksum mismatch: expected 0x{expected:02X}, calculated 0x{calculated:02X}")]
    ChecksumMismatch { expected: u8, calculated: u8 },

    #[error("Header mismatch in serial bridge: expected [0xAA, 0x55]")]
    InvalidBridgeHeader,

    #[error("Parameter error: {0}")]
    ParameterError(String),
}

/// Damiao motor feedback state
#[derive(Debug, Clone, PartialEq)]
pub struct DamiaoFeedback {
    pub motor_id: u8,
    pub state_or_error: u8,
    pub position: f32,
    pub velocity: f32,
    pub torque: f32,
    pub t_mos: f32,
    pub t_rotor: f32,
}

/// Raw velocity command for DamiAO J7 gripper in Position-Force mode.
///
/// Unit: 0.01 rad/s (2000 corresponds to 20.00 rad/s).
pub const DAMIAO_GRIPPER_POSFORCE_VEL_RAW: u16 = 2000;

/// Register read opcode.
pub const REG_READ: u8 = 0x33;
/// Register write opcode (RAM, lost at power-off).
pub const REG_WRITE: u8 = 0x55;
/// Control mode register: 1 = MIT, 2 = POS_VEL, 3 = VEL, 4 = POS_FORCE.
pub const RID_CTRL_MODE: u8 = 10;
/// Position mapping range of the MIT frame and of feedback (rad, f32).
pub const RID_PMAX: u8 = 21;
/// Velocity mapping range (rad/s, f32).
pub const RID_VMAX: u8 = 22;
/// Torque mapping range (N·m, f32).
pub const RID_TMAX: u8 = 23;
pub const CTRL_MODE_MIT: u32 = 1;
pub const CTRL_MODE_POS_VEL: u32 = 2;
/// Position-force mode (frame `0x300 + id`: f32 position, u16 speed limit in 0.01 rad/s, u16
/// current limit in 1/10000 of the maximum phase current).
pub const CTRL_MODE_POS_FORCE: u32 = 4;
/// Command CAN ID base of position-velocity frames (`0x100 + motor_id`).
pub const POS_VEL_BASE_ID: u16 = 0x100;
/// Maximum standard 11-bit CAN arbitration ID.
pub const MAX_STANDARD_CAN_ID: u16 = 0x7FF;
/// Command CAN ID base of position-force frames.
pub const POS_FORCE_BASE_ID: u16 = 0x300;
/// Full scale of the position-force current field (= the maximum phase current).
pub const POS_FORCE_I_DES_FULL: u16 = 10_000;

/// Feedback (master) CAN ID of the motor (u32).
pub const RID_MST_ID: u8 = 7;
/// Command (ESC) CAN ID of the motor (u32).
pub const RID_ESC_ID: u8 = 8;
/// Command timeout (u32 counts; 0 = no timeout). Damiao documents 50 µs per count
/// (wiki.seeedstudio.com/damiao_series); the unit, the expiry latency and the motor's
/// response on expiry are NOT verified on the B601-DM firmware (sw_ver 942747703).
pub const RID_TIMEOUT: u8 = 9;
/// Firmware version (u32).
pub const RID_SW_VER: u8 = 14;
/// Documented TIMEOUT unit (µs per count). Unverified on this firmware: keep it a recorded
/// value of every commissioning, never a hidden assumption.
pub const TIMEOUT_UNIT_US_DOCUMENTED: u32 = 50;

/// Reply to a register read or write.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RegisterReply {
    pub motor_id: u16,
    pub write: bool,
    pub rid: u8,
    pub value: [u8; 4],
}

impl RegisterReply {
    pub fn as_u32(&self) -> u32 { u32::from_le_bytes(self.value) }
    pub fn as_f32(&self) -> f32 { f32::from_le_bytes(self.value) }
}

/// Pure wire output of one Damiao standard-ID CAN command frame (zero-allocation, `Copy`).
///
/// Intentionally carries raw `{ can_id, data }` bytes rather than a `CanFrame` so the
/// compile-time `PosForceSendToken` seal on `build_pos_force_frame` remains intact until
/// Step 2 migrates the unified per-tick emitter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DamiaoEncodedFrame {
    pub can_id: u16,
    pub data: [u8; 8],
}

/// Typed per-mode command payload for [`encode_command`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DamiaoIntentPayload {
    /// Position-velocity (`CTRL_MODE 2`, `0x100 + motor_id`): position (rad) and optional
    /// speed ceiling (rad/s; defaults to `PositionVelocityLimits::max_velocity_rad_s`).
    PositionVelocity {
        position: f32,
        velocity: Option<f32>,
    },
    /// Impedance / MIT (`CTRL_MODE 1`, `motor_id`): position (rad), velocity (rad/s),
    /// stiffness `kp`, damping `kd`, and feedforward torque (N·m).
    Impedance {
        position: f32,
        velocity: f32,
        kp: f32,
        kd: f32,
        torque: f32,
    },
    /// Position-force (`CTRL_MODE 4`, `0x300 + motor_id`): target position (rad), optional
    /// speed ceiling (rad/s; `None` defaults to `close_speed_rad_s` for targets and `0` for holds),
    /// optional current ceiling override (raw `1..=10000`, clamped `<= current_ceiling_raw`),
    /// and `hold` distinguishing stationary Hold/Stop/Freeze (`vel_raw = 0`) from moving Targets
    /// (which refuse `vel_raw == 0` fail-closed).
    PositionForce {
        position: f32,
        velocity: Option<f32>,
        current_raw: Option<u16>,
        hold: bool,
    },
}

/// Typed zero-allocation refusal returned by [`encode_command`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum DamiaoEncodeRefusal {
    #[error("Damiao codec does not support control mode '{0}'")]
    UnsupportedMode(ControlMode),

    #[error("control mode, intent payload, and mode limits do not match")]
    ModeMismatch,

    #[error("invalid Damiao standard CAN motor_id (must be >= 1 and base_id + motor_id <= 0x7FF)")]
    InvalidMotorId,

    #[error("non-finite floating-point value in Damiao command payload or limits")]
    NonFiniteInput,

    #[error("invalid Damiao mode limits")]
    InvalidLimits,

    #[error("Damiao command speed is non-positive or quantizes to 0 on the wire")]
    ZeroWireSpeed,

    #[error("Damiao position-force current limit must be >= 1 raw")]
    ZeroCurrentLimit,

    #[error("Damiao position-force hold frame requires zero velocity")]
    NonZeroHoldVelocity,
}
