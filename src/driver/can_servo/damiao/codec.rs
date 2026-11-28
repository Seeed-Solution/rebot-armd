//! Damiao CAN frame encoding, decoding, and serial bridge framing.

use crate::motor_mode::{
    is_valid_quantized_range, pos_force_max_wire_speed_f32, ControlMode, ModeLimits,
    IMPEDANCE_WIRE_KD_MAX, IMPEDANCE_WIRE_KP_MAX,
};
use crate::transport::CanFrame;
use super::table::*;

/// Motor ID in byte 0 of a feedback frame (low nibble), read before choosing the
/// mapping range to decode it with.
pub fn feedback_motor_id(frame: &CanFrame) -> u8 { frame.data[0] & 0x0F }


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

/// Pure 8-byte MIT payload packer shared by [`build_mit_frame`] and [`encode_command`].
fn encode_mit_payload(
    pos: f32,
    vel: f32,
    kp: f32,
    kd: f32,
    torque: f32,
    limits: &DamiaoLimits,
) -> [u8; 8] {
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
    data
}

/// Pure 8-byte POS_VEL payload packer shared by [`build_pos_vel_frame`] and [`encode_command`].
fn encode_pos_vel_payload(pos: f32, vel: f32) -> [u8; 8] {
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&pos.to_le_bytes());
    data[4..8].copy_from_slice(&vel.to_le_bytes());
    data
}

/// Pure 8-byte feedback payload unpacker shared by [`unpack_feedback_frame`] and [`decode_feedback_bytes`].
fn decode_feedback_payload(data: &[u8; 8], limits: &DamiaoLimits) -> DamiaoFeedback {
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

    DamiaoFeedback {
        motor_id,
        state_or_error,
        position,
        velocity,
        torque,
        t_mos,
        t_rotor,
    }
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
    let data = encode_mit_payload(pos, vel, kp, kd, torque, limits);
    CanFrame::new_standard(motor_id, &data).unwrap()
}

/// Build Position-Velocity mode CAN frame (CAN ID = base_id + motor_id, float32 pos, float32 vel)
pub fn build_pos_vel_frame(base_id: u16, motor_id: u16, pos: f32, vel: f32) -> CanFrame {
    let data = encode_pos_vel_payload(pos, vel);
    CanFrame::new_standard(base_id + motor_id, &data).unwrap()
}


/// Encode the 8-byte Damiao Position-Force payload (`float32 pos, u16 vel_raw, u16 i_des`).
///
/// `vel_raw` is in units of 0.01 rad/s (e.g. `DAMIAO_GRIPPER_POSFORCE_VEL_RAW` = 2000 for 20.00 rad/s).
/// This is a pure byte encoder; constructing a `0x300 + motor_id` `CanFrame` is sealed to
/// `crate::driver::pos_force_emitter` via [`build_pos_force_frame`].
pub fn encode_pos_force_payload(pos: f32, vel_raw: u16, i_des: u16) -> [u8; 8] {
    let p_bytes = pos.to_le_bytes();
    let v_bytes = vel_raw.to_le_bytes();
    let i_bytes = i_des.to_le_bytes();
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&p_bytes);
    data[4..6].copy_from_slice(&v_bytes);
    data[6..8].copy_from_slice(&i_bytes);
    data
}

/// Build Position-Force mode CAN frame (`CAN ID = base_id + motor_id`, `float32 pos, u16 vel_raw, u16 i_des`).
///
/// Sealed by the Rust type system to private `crate::driver::pos_force_emitter`: only that module
/// can construct [`crate::driver::PosForceSendToken`], preventing any other module from building a
/// `0x307` `CanFrame` at compile time.
pub(crate) fn build_pos_force_frame(
    _token: &crate::driver::PosForceSendToken,
    base_id: u16,
    motor_id: u16,
    pos: f32,
    vel_raw: u16,
    i_des: u16,
) -> CanFrame {
    let data = encode_pos_force_payload(pos, vel_raw, i_des);
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
    Ok(decode_feedback_payload(&frame.data, limits))
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
// Layout as used by Damiao's DM_CAN reference library. Reads were confirmed on the
// B601-DM on 2026-11-09 (registers 10 and 21-23 of all seven motors); writes have not
// been checked on hardware yet. Request on CAN ID 0x7FF, `[id_l, id_h, op, rid, d0, d1, d2, d3]`
// with op 0x33 = read and 0x55 = write (RAM only; 0xAA would store to flash).
// The motor answers on its feedback ID with the same layout and the current
// value. Integer registers (such as CTRL_MODE) are u32, the others f32, both
// little-endian.


/// Speed limit of a position-force frame: |speed| in 0.01 rad/s, quantized downward in f64
/// so the encoded wire speed never exceeds `speed_rad_s`, saturated to u16 (non-finite: 0).
pub fn pos_force_vel_raw(speed_rad_s: f32) -> u16 {
    crate::model::DamiaoGripperDrive::quantize_vel_raw(speed_rad_s)
}

/// Build a RAM register write of a u32 value (`0x55`; never the flash store `0xAA`).
pub fn build_write_register_u32_frame(motor_id: u16, rid: u8, value: u32) -> CanFrame {
    build_write_register_frame(motor_id, rid, value.to_le_bytes())
}

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

// =========================================================================
// Pure mode-semantic command encoder and feedback decoder (Step 1 codec)
// =========================================================================


/// Pure Damiao command encoder: `(motor_id, mode, payload, limits) -> { can_id, data }`.
///
/// Validates finite inputs and mode limits, clamps command values to the configured per-mode
/// ceilings before wire quantization, and produces byte-exact frames matching
/// [`build_pos_vel_frame`], [`build_mit_frame`], and [`encode_pos_force_payload`].
pub fn encode_command(
    motor_id: u16,
    mode: ControlMode,
    payload: DamiaoIntentPayload,
    limits: &ModeLimits,
) -> Result<DamiaoEncodedFrame, DamiaoEncodeRefusal> {
    if mode == ControlMode::Position {
        return Err(DamiaoEncodeRefusal::UnsupportedMode(mode));
    }
    if limits.mode() != mode {
        return Err(DamiaoEncodeRefusal::ModeMismatch);
    }
    if motor_id == 0 {
        return Err(DamiaoEncodeRefusal::InvalidMotorId);
    }

    match (mode, payload, limits) {
        (
            ControlMode::PositionVelocity,
            DamiaoIntentPayload::PositionVelocity { position, velocity },
            ModeLimits::PositionVelocity(lim),
        ) => {
            let can_id = POS_VEL_BASE_ID
                .checked_add(motor_id)
                .filter(|&id| id <= MAX_STANDARD_CAN_ID)
                .ok_or(DamiaoEncodeRefusal::InvalidMotorId)?;
            if !lim.max_velocity_rad_s.is_finite() || lim.max_velocity_rad_s <= 0.0 {
                return Err(DamiaoEncodeRefusal::InvalidLimits);
            }
            if !position.is_finite() || velocity.is_some_and(|v| !v.is_finite()) {
                return Err(DamiaoEncodeRefusal::NonFiniteInput);
            }
            let eff_vel = match velocity {
                Some(v) => {
                    if v <= 0.0 {
                        return Err(DamiaoEncodeRefusal::ZeroWireSpeed);
                    }
                    v.min(lim.max_velocity_rad_s)
                }
                None => lim.max_velocity_rad_s,
            };
            let data = encode_pos_vel_payload(position, eff_vel);
            Ok(DamiaoEncodedFrame { can_id, data })
        }
        (
            ControlMode::Impedance,
            DamiaoIntentPayload::Impedance {
                position,
                velocity,
                kp,
                kd,
                torque,
            },
            ModeLimits::Impedance(lim),
        ) => {
            if motor_id > MAX_STANDARD_CAN_ID {
                return Err(DamiaoEncodeRefusal::InvalidMotorId);
            }
            let valid_limits = is_valid_quantized_range(-lim.p_max, lim.p_max, 16)
                && is_valid_quantized_range(-lim.v_max, lim.v_max, 12)
                && is_valid_quantized_range(-lim.t_max, lim.t_max, 12)
                && lim.kp_max.is_finite()
                && (0.0..=IMPEDANCE_WIRE_KP_MAX).contains(&lim.kp_max)
                && lim.kd_max.is_finite()
                && (0.0..=IMPEDANCE_WIRE_KD_MAX).contains(&lim.kd_max)
                && lim
                    .effort_limit_nm
                    .is_none_or(|e| e.is_finite() && e > 0.0 && e <= lim.t_max);
            if !valid_limits {
                return Err(DamiaoEncodeRefusal::InvalidLimits);
            }
            if !(position.is_finite()
                && velocity.is_finite()
                && kp.is_finite()
                && kd.is_finite()
                && torque.is_finite())
            {
                return Err(DamiaoEncodeRefusal::NonFiniteInput);
            }

            let wire = DamiaoLimits::from_impedance_limits(lim);
            let pos_c = position.clamp(wire.p_min, wire.p_max);
            let vel_c = velocity.clamp(wire.v_min, wire.v_max);
            let kp_c = kp.clamp(wire.kp_min, lim.kp_max);
            let kd_c = kd.clamp(wire.kd_min, lim.kd_max);
            let t_cap = lim.effective_effort_limit_nm();
            let torque_c = torque.clamp(-t_cap, t_cap);

            let data = encode_mit_payload(pos_c, vel_c, kp_c, kd_c, torque_c, &wire);
            Ok(DamiaoEncodedFrame {
                can_id: motor_id,
                data,
            })
        }
        (
            ControlMode::PositionForce,
            DamiaoIntentPayload::PositionForce {
                position,
                velocity,
                current_raw,
                hold,
            },
            ModeLimits::PositionForce(lim),
        ) => {
            let can_id = POS_FORCE_BASE_ID
                .checked_add(motor_id)
                .filter(|&id| id <= MAX_STANDARD_CAN_ID)
                .ok_or(DamiaoEncodeRefusal::InvalidMotorId)?;
            let max_wire_speed = pos_force_max_wire_speed_f32();
            let valid_limits = (1..=POS_FORCE_I_DES_FULL).contains(&lim.current_ceiling_raw)
                && lim.close_speed_rad_s.is_finite()
                && lim.close_speed_rad_s > 0.0
                && lim.close_speed_rad_s <= max_wire_speed
                && pos_force_vel_raw(lim.close_speed_rad_s) >= 1
                && lim.max_speed_rad_s.is_none_or(|m| {
                    m.is_finite() && m >= lim.close_speed_rad_s && m <= max_wire_speed
                });
            if !valid_limits {
                return Err(DamiaoEncodeRefusal::InvalidLimits);
            }
            if !position.is_finite() || velocity.is_some_and(|v| !v.is_finite()) {
                return Err(DamiaoEncodeRefusal::NonFiniteInput);
            }
            if current_raw == Some(0) {
                return Err(DamiaoEncodeRefusal::ZeroCurrentLimit);
            }
            let i_des = current_raw.map_or(lim.current_ceiling_raw, |c| c.min(lim.current_ceiling_raw));
            let vel_raw = if hold {
                if velocity.is_some_and(|v| v != 0.0) {
                    return Err(DamiaoEncodeRefusal::NonZeroHoldVelocity);
                }
                0u16
            } else {
                let cap = lim.max_speed_rad_s.unwrap_or(lim.close_speed_rad_s);
                let eff_vel = velocity.map_or(lim.close_speed_rad_s, f32::abs).min(cap);
                let raw = pos_force_vel_raw(eff_vel);
                if raw == 0 {
                    return Err(DamiaoEncodeRefusal::ZeroWireSpeed);
                }
                raw
            };
            let data = encode_pos_force_payload(position, vel_raw, i_des);
            Ok(DamiaoEncodedFrame { can_id, data })
        }
        _ => Err(DamiaoEncodeRefusal::ModeMismatch),
    }
}

/// Pure Damiao 8-byte feedback payload decoder (no `CanFrame` / transport dependency).
///
/// Decodes identical values to [`unpack_feedback_frame`] while rejecting invalid or
/// arithmetic-overflowing mapping ranges.
pub fn decode_feedback_bytes(
    data: &[u8],
    limits: &DamiaoLimits,
) -> Result<DamiaoFeedback, DamiaoError> {
    if data.len() < 8 {
        return Err(DamiaoError::IncompleteFrame {
            expected: 8,
            actual: data.len(),
        });
    }
    if data.len() > 8 {
        return Err(DamiaoError::ParameterError(format!(
            "Damiao feedback payload must be 8 bytes, got {}",
            data.len()
        )));
    }
    if !(is_valid_quantized_range(limits.p_min, limits.p_max, 16)
        && is_valid_quantized_range(limits.v_min, limits.v_max, 12)
        && is_valid_quantized_range(limits.t_min, limits.t_max, 12))
    {
        return Err(DamiaoError::ParameterError(
            "DamiaoLimits must have finite non-overflowing quantized ranges p/v/t".into(),
        ));
    }
    let bytes: [u8; 8] = [
        data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
    ];
    Ok(decode_feedback_payload(&bytes, limits))
}
