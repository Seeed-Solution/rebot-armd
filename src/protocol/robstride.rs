use std::f32::consts::PI;
use thiserror::Error;
use crate::transport::CanFrame;

// Communication types for RobStride extended 29-bit CAN IDs
pub const TYPE_GET_DEVICE_ID: u8 = 0;
pub const TYPE_MOTION_CONTROL: u8 = 1;
pub const TYPE_FEEDBACK: u8 = 2;
pub const TYPE_ENABLE: u8 = 3;
pub const TYPE_STOP: u8 = 4;
pub const TYPE_SET_ZERO: u8 = 6;
pub const TYPE_PARAM_READ: u8 = 17;
pub const TYPE_PARAM_WRITE: u8 = 18;

/// Default physical limits for RobStride actuators
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RobStrideLimits {
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

impl Default for RobStrideLimits {
    fn default() -> Self {
        Self {
            p_min: -4.0 * PI, // -12.56637 rad
            p_max: 4.0 * PI,  // +12.56637 rad
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

#[derive(Debug, Error, PartialEq, Eq)]
pub enum RobStrideError {
    #[error("Incomplete frame: expected at least {expected} bytes, got {actual}")]
    IncompleteFrame { expected: usize, actual: usize },

    #[error("Unexpected frame type: expected extended feedback frame (0x02), got comm_type {comm_type:#x}, extended={is_extended}")]
    UnexpectedType { comm_type: u8, is_extended: bool },

    #[error("Protocol violation: invalid or RTR frame received")]
    ProtocolViolation,

    #[error("Fault reported in actuator feedback: 0x{0:02X}")]
    Fault(u8),

    #[error("Parameter error: {0}")]
    ParameterError(String),
}

/// Parsed feedback telemetry from RobStride actuator
#[derive(Debug, Clone, PartialEq)]
pub struct RobStrideFeedback {
    pub motor_id: u8,
    pub position: f32,
    pub velocity: f32,
    pub torque: f32,
    pub mos_temperature: f32,
    pub fault_flags: u8,
}

/// Linear quantization from float to uint
pub fn float_to_uint(val: f32, min_val: f32, max_val: f32, bits: u32) -> u16 {
    let span = max_val - min_val;
    if span <= 0.0 {
        return 0;
    }
    let clamped = val.clamp(min_val, max_val);
    let max_int = ((1u32 << bits) - 1) as f32;
    ((clamped - min_val) * max_int / span).round() as u16
}

/// Linear de-quantization from uint to float
pub fn uint_to_float(raw: u16, min_val: f32, max_val: f32, bits: u32) -> f32 {
    let span = max_val - min_val;
    let max_int = ((1u32 << bits) - 1) as f32;
    min_val + (raw as f32) * span / max_int
}

/// Construct 29-bit extended CAN ID: `(comm_type << 24) | (data_field << 8) | motor_id`
/// bits 28..24: comm_type (5-bit)
/// bits 23..8:  data_field (16-bit)
/// bits 7..0:   motor_id (8-bit)
pub fn make_extended_id(comm_type: u8, data_field: u16, motor_id: u8) -> u32 {
    ((comm_type as u32 & 0x1F) << 24) | ((data_field as u32 & 0xFFFF) << 8) | (motor_id as u32)
}

/// Extract fields from 29-bit extended CAN ID
pub fn parse_extended_id(can_id: u32) -> (u8, u16, u8) {
    let comm_type = ((can_id >> 24) & 0x1F) as u8;
    let data_field = ((can_id >> 8) & 0xFFFF) as u16;
    let motor_id = (can_id & 0xFF) as u8;
    (comm_type, data_field, motor_id)
}

/// Build RobStride actuator enable frame
/// master_id is directly passed as low 8 bits of data_field
pub fn build_enable_frame(motor_id: u8, master_id: u8) -> CanFrame {
    let ext_id = make_extended_id(TYPE_ENABLE, master_id as u16, motor_id);
    CanFrame::new_extended(ext_id, &[0u8; 8]).unwrap()
}

/// Build RobStride actuator stop / disable frame
/// master_id is directly passed as low 8 bits of data_field
pub fn build_stop_frame(motor_id: u8, master_id: u8) -> CanFrame {
    let ext_id = make_extended_id(TYPE_STOP, master_id as u16, motor_id);
    CanFrame::new_extended(ext_id, &[0u8; 8]).unwrap()
}

/// Build RobStride Type-1 motion control CAN frame:
/// Extended CAN ID:
///   - comm_type = 1 (bits 28..24)
///   - torque T quantized as 16-bit unsigned integer (0..65535, zero torque at 32768, bits 23..8)
///   - motor_id (bits 7..0)
/// 8-byte Data Payload: 4 x 16-bit big-endian unsigned integers:
///   - byte 0-1: p_des (16-bit be)
///   - byte 2-3: v_des (16-bit be)
///   - byte 4-5: Kp (16-bit be)
///   - byte 6-7: Kd (16-bit be)
pub fn build_mit_frame(
    motor_id: u8,
    pos: f32,
    vel: f32,
    kp: f32,
    kd: f32,
    torque: f32,
    limits: &RobStrideLimits,
) -> CanFrame {
    let p_int = float_to_uint(pos, limits.p_min, limits.p_max, 16);
    let v_int = float_to_uint(vel, limits.v_min, limits.v_max, 16);
    let kp_int = float_to_uint(kp, limits.kp_min, limits.kp_max, 16);
    let kd_int = float_to_uint(kd, limits.kd_min, limits.kd_max, 16);
    let t_int = float_to_uint(torque, limits.t_min, limits.t_max, 16);

    let mut data = [0u8; 8];
    data[0..2].copy_from_slice(&p_int.to_be_bytes());
    data[2..4].copy_from_slice(&v_int.to_be_bytes());
    data[4..6].copy_from_slice(&kp_int.to_be_bytes());
    data[6..8].copy_from_slice(&kd_int.to_be_bytes());

    let ext_id = make_extended_id(TYPE_MOTION_CONTROL, t_int, motor_id);
    CanFrame::new_extended(ext_id, &data).unwrap()
}

/// Construct RobStride Type-2 feedback extended CAN ID (29-bit):
///   - bit 28..24 (5 bit): 0x02 (Type 2 Feedback)
///   - bit 23..16 (8 bit): fault_flags
///   - bit 15..8  (8 bit): motor_id
///   - bit 7..0   (8 bit): master_id (typically 0xFD)
pub fn build_feedback_extended_id(fault_flags: u8, motor_id: u8, master_id: u8) -> u32 {
    ((TYPE_FEEDBACK as u32 & 0x1F) << 24)
        | ((fault_flags as u32) << 16)
        | ((motor_id as u32) << 8)
        | (master_id as u32)
}

/// Unpack feedback frame from RobStride actuator
/// Type-2 Feedback Extended CAN ID (29-bit):
///   - bit 28..24: Type 2 (0x02)
///   - bit 23..16: fault_flags
///   - bit 15..8:  motor_id
///   - bit 7..0:   master_id
/// Data Payload:
///   - byte 0-1: position (16-bit be)
///   - byte 2-3: velocity (16-bit be)
///   - byte 4-5: torque (16-bit be)
///   - byte 6-7: temperature (16-bit be, unit 0.1°C, mos_temperature = raw as f32 / 10.0)
pub fn unpack_feedback_frame(
    frame: &CanFrame,
    limits: &RobStrideLimits,
) -> Result<RobStrideFeedback, RobStrideError> {
    if frame.dlc < 8 {
        return Err(RobStrideError::IncompleteFrame {
            expected: 8,
            actual: frame.dlc as usize,
        });
    }

    if frame.is_rtr {
        return Err(RobStrideError::ProtocolViolation);
    }

    if !frame.is_extended {
        return Err(RobStrideError::UnexpectedType {
            comm_type: 0,
            is_extended: false,
        });
    }

    let comm_type = ((frame.id >> 24) & 0x1F) as u8;
    if comm_type != TYPE_FEEDBACK {
        return Err(RobStrideError::UnexpectedType {
            comm_type,
            is_extended: true,
        });
    }

    let motor_id = ((frame.id >> 8) & 0xFF) as u8;
    let fault_flags = ((frame.id >> 16) & 0xFF) as u8;

    let data = &frame.data;
    let p_raw = u16::from_be_bytes([data[0], data[1]]);
    let v_raw = u16::from_be_bytes([data[2], data[3]]);
    let t_raw = u16::from_be_bytes([data[4], data[5]]);
    let temp_raw = u16::from_be_bytes([data[6], data[7]]);

    let position = uint_to_float(p_raw, limits.p_min, limits.p_max, 16);
    let velocity = uint_to_float(v_raw, limits.v_min, limits.v_max, 16);
    let torque = uint_to_float(t_raw, limits.t_min, limits.t_max, 16);
    let mos_temperature = (temp_raw as f32) / 10.0;

    Ok(RobStrideFeedback {
        motor_id,
        position,
        velocity,
        torque,
        mos_temperature,
        fault_flags,
    })
}
