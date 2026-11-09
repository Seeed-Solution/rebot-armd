use thiserror::Error;
use crate::transport::CanFrame;

/// Special motor control commands
pub const CMD_ENABLE: [u8; 8] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC];
pub const CMD_DISABLE: [u8; 8] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD];
pub const CMD_SET_ZERO: [u8; 8] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE];
pub const CMD_CLEAR_FAULTS: [u8; 8] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB];

/// Serial bridge packet header
pub const SERIAL_BRIDGE_HEADER: [u8; 2] = [0xAA, 0x55];

/// Default physical parameter limits for Damiao DM4310 / DM6006 / DM8006
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

/// Convert float to uint using linear quantization
pub fn float_to_uint(val: f32, min_val: f32, max_val: f32, bits: u32) -> u16 {
    let span = max_val - min_val;
    if span <= 0.0 {
        return 0;
    }
    let clamped = val.clamp(min_val, max_val);
    let max_int = ((1u32 << bits) - 1) as f32;
    ((clamped - min_val) * max_int / span).round() as u16
}

/// Convert uint to float using linear de-quantization
pub fn uint_to_float(raw: u16, min_val: f32, max_val: f32, bits: u32) -> f32 {
    let span = max_val - min_val;
    let max_int = ((1u32 << bits) - 1) as f32;
    min_val + (raw as f32) * span / max_int
}

/// Build Damiao motor enable CAN frame
pub fn build_enable_frame(motor_id: u16) -> CanFrame {
    CanFrame::new_standard(motor_id, &CMD_ENABLE).unwrap()
}

/// Build Damiao motor disable CAN frame
pub fn build_disable_frame(motor_id: u16) -> CanFrame {
    CanFrame::new_standard(motor_id, &CMD_DISABLE).unwrap()
}

/// Build Damiao set zero position CAN frame
pub fn build_set_zero_frame(motor_id: u16) -> CanFrame {
    CanFrame::new_standard(motor_id, &CMD_SET_ZERO).unwrap()
}

/// Build Damiao clear faults CAN frame
pub fn build_clear_faults_frame(motor_id: u16) -> CanFrame {
    CanFrame::new_standard(motor_id, &CMD_CLEAR_FAULTS).unwrap()
}

/// Build Damiao read-only status refresh frame (0x7FF, [id_l, id_h, 0xCC, 0..]).
/// The motor answers with a normal feedback frame and keeps its enable state.
pub fn build_refresh_frame(motor_id: u16) -> CanFrame {
    let [lo, hi] = motor_id.to_le_bytes();
    CanFrame::new_standard(0x7FF, &[lo, hi, 0xCC, 0, 0, 0, 0, 0]).unwrap()
}

/// Build Damiao MIT mode control CAN frame
/// Packs: position (16-bit), velocity (12-bit), Kp (12-bit), Kd (12-bit), Torque (12-bit)
pub fn build_mit_frame(
    motor_id: u16,
    pos: f32,
    vel: f32,
    kp: f32,
    kd: f32,
    torque: f32,
    limits: &DamiaoLimits,
) -> CanFrame {
    let p_int = float_to_uint(pos, limits.p_min, limits.p_max, 16);
    let v_int = float_to_uint(vel, limits.v_min, limits.v_max, 12);
    let kp_int = float_to_uint(kp, limits.kp_min, limits.kp_max, 12);
    let kd_int = float_to_uint(kd, limits.kd_min, limits.kd_max, 12);
    let t_int = float_to_uint(torque, limits.t_min, limits.t_max, 12);

    let mut data = [0u8; 8];
    data[0] = (p_int >> 8) as u8;
    data[1] = (p_int & 0xFF) as u8;
    data[2] = (v_int >> 4) as u8;
    data[3] = (((v_int & 0x0F) << 4) | (kp_int >> 8)) as u8;
    data[4] = (kp_int & 0xFF) as u8;
    data[5] = (kd_int >> 4) as u8;
    data[6] = (((kd_int & 0x0F) << 4) | (t_int >> 8)) as u8;
    data[7] = (t_int & 0xFF) as u8;

    CanFrame::new_standard(motor_id, &data).unwrap()
}

/// Build Position-Velocity mode CAN frame (CAN ID = base_id + motor_id, float32 pos, float32 vel)
pub fn build_pos_vel_frame(base_id: u16, motor_id: u16, pos: f32, vel: f32) -> CanFrame {
    let p_bytes = pos.to_le_bytes();
    let v_bytes = vel.to_le_bytes();
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&p_bytes);
    data[4..8].copy_from_slice(&v_bytes);
    CanFrame::new_standard(base_id + motor_id, &data).unwrap()
}

/// Raw velocity command for DamiAO J7 gripper in Position-Force mode.
///
/// Unit: 0.01 rad/s (2000 corresponds to 20.00 rad/s).
pub const DAMIAO_GRIPPER_POSFORCE_VEL_RAW: u16 = 2000;

/// Build Position-Force mode CAN frame (CAN ID = base_id + motor_id, float32 pos, u16 vel_raw, u16 i_des)
///
/// `vel_raw` is in units of 0.01 rad/s (e.g. `DAMIAO_GRIPPER_POSFORCE_VEL_RAW` = 2000 for 20.00 rad/s).
pub fn build_pos_force_frame(base_id: u16, motor_id: u16, pos: f32, vel_raw: u16, i_des: u16) -> CanFrame {
    let p_bytes = pos.to_le_bytes();
    let v_bytes = vel_raw.to_le_bytes();
    let i_bytes = i_des.to_le_bytes();
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&p_bytes);
    data[4..6].copy_from_slice(&v_bytes);
    data[6..8].copy_from_slice(&i_bytes);
    CanFrame::new_standard(base_id + motor_id, &data).unwrap()
}

/// Unpack feedback frame from Damiao motor
pub fn unpack_feedback_frame(
    frame: &CanFrame,
    limits: &DamiaoLimits,
) -> Result<DamiaoFeedback, DamiaoError> {
    if frame.dlc < 8 {
        return Err(DamiaoError::IncompleteFrame {
            expected: 8,
            actual: frame.dlc as usize,
        });
    }

    let data = &frame.data;
    let motor_id = data[0] & 0x0F;
    let state_or_error = (data[0] >> 4) & 0x0F;

    let p_raw = ((data[1] as u16) << 8) | (data[2] as u16);
    let v_raw = ((data[3] as u16) << 4) | ((data[4] as u16) >> 4);
    let t_raw = (((data[4] as u16) & 0x0F) << 8) | (data[5] as u16);

    let position = uint_to_float(p_raw, limits.p_min, limits.p_max, 16);
    let velocity = uint_to_float(v_raw, limits.v_min, limits.v_max, 12);
    let torque = uint_to_float(t_raw, limits.t_min, limits.t_max, 12);

    let t_mos = data[6] as f32;
    let t_rotor = data[7] as f32;

    Ok(DamiaoFeedback {
        motor_id,
        state_or_error,
        position,
        velocity,
        torque,
        t_mos,
        t_rotor,
    })
}

// =========================================================================
// Serial Bridge Framing (大喵串口桥接封装)
// =========================================================================

/// Encapsulate a CAN frame into a 14-byte Serial Bridge packet
/// Format: `[0xAA, 0x55, ID_L, ID_H, DLC, D0..D7, CHECKSUM]`
pub fn encode_serial_bridge_frame(frame: &CanFrame) -> Vec<u8> {
    let mut packet = Vec::with_capacity(14);
    packet.extend_from_slice(&SERIAL_BRIDGE_HEADER);
    let id_bytes = (frame.id as u16).to_le_bytes();
    packet.push(id_bytes[0]);
    packet.push(id_bytes[1]);
    packet.push(frame.dlc.min(8));
    packet.extend_from_slice(&frame.data);

    let sum: u32 = packet.iter().map(|&b| b as u32).sum();
    let checksum = (sum & 0xFF) as u8;
    packet.push(checksum);
    packet
}

/// Decode a 14-byte Serial Bridge packet from a byte stream into a `CanFrame`
pub fn decode_serial_bridge_frame(buf: &[u8]) -> Result<(CanFrame, usize), DamiaoError> {
    let header_pos = match buf.windows(2).position(|w| w == SERIAL_BRIDGE_HEADER) {
        Some(pos) => pos,
        None => {
            return Err(DamiaoError::IncompleteFrame {
                expected: 14,
                actual: buf.len(),
            });
        }
    };

    let slice = &buf[header_pos..];
    if slice.len() < 14 {
        return Err(DamiaoError::IncompleteFrame {
            expected: 14,
            actual: slice.len(),
        });
    }

    let id = u16::from_le_bytes([slice[2], slice[3]]);
    let dlc = slice[4];
    let mut data = [0u8; 8];
    data.copy_from_slice(&slice[5..13]);
    let expected_checksum = slice[13];

    let sum: u32 = slice[..13].iter().map(|&b| b as u32).sum();
    let calculated_checksum = (sum & 0xFF) as u8;

    if calculated_checksum != expected_checksum {
        return Err(DamiaoError::ChecksumMismatch {
            expected: expected_checksum,
            calculated: calculated_checksum,
        });
    }

    let frame = CanFrame {
        id: id as u32,
        is_extended: false,
        is_rtr: false,
        dlc,
        data,
    };

    Ok((frame, header_pos + 14))
}

// =========================================================================
// Register access (0x7FF): control mode and the MIT mapping ranges
// =========================================================================
//
// Layout as used by Damiao's DM_CAN reference library (not verified on the
// B601-DM yet): request on CAN ID 0x7FF, `[id_l, id_h, op, rid, d0, d1, d2, d3]`
// with op 0x33 = read and 0x55 = write (RAM only; 0xAA would store to flash).
// The motor answers on its feedback ID with the same layout and the current
// value. Integer registers (such as CTRL_MODE) are u32, the others f32, both
// little-endian.

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

/// Build a register read request (`0x7FF`, `[id_l, id_h, 0x33, rid, 0, 0, 0, 0]`).
pub fn build_read_register_frame(motor_id: u16, rid: u8) -> CanFrame {
    let [lo, hi] = motor_id.to_le_bytes();
    CanFrame::new_standard(0x7FF, &[lo, hi, REG_READ, rid, 0, 0, 0, 0]).unwrap()
}

/// Build a RAM register write (`0x7FF`, `[id_l, id_h, 0x55, rid, value LE]`).
pub fn build_write_register_frame(motor_id: u16, rid: u8, value: [u8; 4]) -> CanFrame {
    let [lo, hi] = motor_id.to_le_bytes();
    let [v0, v1, v2, v3] = value;
    CanFrame::new_standard(0x7FF, &[lo, hi, REG_WRITE, rid, v0, v1, v2, v3]).unwrap()
}

/// Build a CTRL_MODE write.
pub fn build_set_control_mode_frame(motor_id: u16, mode: u32) -> CanFrame {
    build_write_register_frame(motor_id, RID_CTRL_MODE, mode.to_le_bytes())
}

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

/// Parse a register reply. A feedback frame has `(state << 4) | id` in byte 0 and the
/// position high byte in byte 1; byte 1 = 0 would mean a position below -12.1 rad, so a
/// frame with id_h = 0 in byte 1 and a register opcode in byte 2 is taken as a reply.
/// Callers must still check `motor_id` and `rid` against what they asked for.
pub fn parse_register_reply(frame: &CanFrame) -> Option<RegisterReply> {
    if frame.is_extended || frame.is_rtr || frame.dlc != 8 { return None; }
    let d = &frame.data;
    if d[1] != 0 || !(d[2] == REG_READ || d[2] == REG_WRITE) { return None; }
    Some(RegisterReply {
        motor_id: u16::from_le_bytes([d[0], d[1]]),
        write: d[2] == REG_WRITE,
        rid: d[3],
        value: [d[4], d[5], d[6], d[7]],
    })
}
