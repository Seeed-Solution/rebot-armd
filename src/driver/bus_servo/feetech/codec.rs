//! Feetech STS/SMS binary packet codec, checksum, framing, and unit conversions.

use std::f32::consts::PI;
use thiserror::Error;
use super::table::*;

#[derive(Debug, Error, PartialEq, Eq)]
pub enum FeetechError {
    #[error("Header mismatch: expected [0xFF, 0xFF]")]
    InvalidHeader,

    #[error("Incomplete frame: expected at least {expected} bytes, got {actual}")]
    IncompleteFrame { expected: usize, actual: usize },

    #[error("Checksum mismatch: expected 0x{expected:02X}, calculated 0x{calculated:02X}")]
    ChecksumMismatch { expected: u8, calculated: u8 },

    #[error("Servo error flag reported: 0x{0:02X}")]
    ServoError(u8),

    #[error("Parameter error: {0}")]
    ParameterError(String),
}

/// Parsed Feetech response / status frame
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FeetechPacket {
    pub id: u8,
    pub error: u8,
    pub params: Vec<u8>,
}

/// Calculate Feetech checksum: `~(id + length + instruction_or_error + sum(params)) & 0xFF`
pub fn calculate_checksum(id: u8, length: u8, instruction_or_error: u8, params: &[u8]) -> u8 {
    let mut sum: u32 = id as u32 + length as u32 + instruction_or_error as u32;
    for &p in params {
        sum += p as u32;
    }
    !((sum & 0xFF) as u8)
}

/// Build a standard instruction packet for a single servo
pub fn build_packet(id: u8, instruction: u8, params: &[u8]) -> Vec<u8> {
    let length = (params.len() + 2) as u8;
    let checksum = calculate_checksum(id, length, instruction, params);

    let mut packet = Vec::with_capacity(6 + params.len());
    packet.extend_from_slice(&FEETECH_HEADER);
    packet.push(id);
    packet.push(length);
    packet.push(instruction);
    packet.extend_from_slice(params);
    packet.push(checksum);
    packet
}

/// Build a Ping instruction packet
pub fn build_ping(id: u8) -> Vec<u8> {
    build_packet(id, INST_PING, &[])
}

/// Zero-allocation stack-array Ping packet (identical bytes to [`build_ping`]).
#[inline]
pub fn build_ping_array(id: u8) -> [u8; 6] {
    let checksum = calculate_checksum(id, 2, INST_PING, &[]);
    [FEETECH_HEADER[0], FEETECH_HEADER[1], id, 2, INST_PING, checksum]
}

/// Build a Read register packet
pub fn build_read(id: u8, start_addr: u8, length_to_read: u8) -> Vec<u8> {
    build_packet(id, INST_READ, &[start_addr, length_to_read])
}

/// Zero-allocation stack-array Read register packet (identical bytes to [`build_read`]).
#[inline]
pub fn build_read_array(id: u8, start_addr: u8, length_to_read: u8) -> [u8; 8] {
    let params = [start_addr, length_to_read];
    let checksum = calculate_checksum(id, 4, INST_READ, &params);
    [
        FEETECH_HEADER[0],
        FEETECH_HEADER[1],
        id,
        4,
        INST_READ,
        start_addr,
        length_to_read,
        checksum,
    ]
}

/// Build a single Write register packet
pub fn build_write(id: u8, start_addr: u8, data: &[u8]) -> Vec<u8> {
    let mut params = Vec::with_capacity(1 + data.len());
    params.push(start_addr);
    params.extend_from_slice(data);
    build_packet(id, INST_WRITE, &params)
}

/// Build Write 1 byte packet
pub fn build_write_u8(id: u8, reg: u8, val: u8) -> Vec<u8> {
    build_write(id, reg, &[val])
}

/// Build Write 2 byte packet (little endian)
pub fn build_write_u16(id: u8, reg: u8, val: u16) -> Vec<u8> {
    let bytes = val.to_le_bytes();
    build_write(id, reg, &bytes)
}

/// Build a Torque Enable/Disable packet (Address 0x28)
pub fn build_set_torque(id: u8, enabled: bool) -> Vec<u8> {
    let val = if enabled { 1u8 } else { 0u8 };
    build_write(id, REG_TORQUE_ENABLE, &[val])
}

/// Zero-allocation stack-array Torque Enable/Disable packet (identical bytes to [`build_set_torque`]).
#[inline]
pub fn build_set_torque_array(id: u8, enabled: bool) -> [u8; 8] {
    let val = u8::from(enabled);
    let params = [REG_TORQUE_ENABLE, val];
    let checksum = calculate_checksum(id, 4, INST_WRITE, &params);
    [
        FEETECH_HEADER[0],
        FEETECH_HEADER[1],
        id,
        4,
        INST_WRITE,
        REG_TORQUE_ENABLE,
        val,
        checksum,
    ]
}

/// Build Goal Position write packet (Address 0x2A) with position, time and speed
pub fn build_set_goal(id: u8, position: u16, time: u16, speed: u16) -> Vec<u8> {
    let p_bytes = position.to_le_bytes();
    let t_bytes = time.to_le_bytes();
    let s_bytes = speed.to_le_bytes();
    let data = [
        p_bytes[0], p_bytes[1],
        t_bytes[0], t_bytes[1],
        s_bytes[0], s_bytes[1],
    ];
    build_write(id, REG_GOAL_POSITION, &data)
}

/// Build `SYNC_READ` packet to query status/telemetry across multiple servos simultaneously.
pub fn build_sync_read(ids: &[u8], start_addr: u8, data_len: u8) -> Result<Vec<u8>, FeetechError> {
    if ids.is_empty() {
        return Err(FeetechError::ParameterError(
            "SYNC_READ requires at least one servo ID".to_string(),
        ));
    }
    if ids.len() > 32 {
        return Err(FeetechError::ParameterError(
            "SYNC_READ supports at most 32 servos".to_string(),
        ));
    }

    let mut params = Vec::with_capacity(2 + ids.len());
    params.push(start_addr);
    params.push(data_len);
    params.extend_from_slice(ids);

    Ok(build_packet(BROADCAST_ID, INST_SYNC_READ, &params))
}

/// Build `SYNC_WRITE` packet to command multiple servos simultaneously.
/// `start_addr`: target start register address (e.g. `REG_GOAL_POSITION` = 0x2A).
/// `data_len_per_servo`: number of data bytes for each servo (e.g. 2 for position only).
/// `servo_entries`: list of `(servo_id, &[u8])`.
pub fn build_sync_write(
    start_addr: u8,
    data_len_per_servo: u8,
    servo_entries: &[(u8, Vec<u8>)],
) -> Result<Vec<u8>, FeetechError> {
    let num_servos = servo_entries.len();
    if num_servos == 0 {
        return Err(FeetechError::ParameterError(
            "SYNC_WRITE requires at least one servo entry".to_string(),
        ));
    }

    // Length = (data_len + 1) * num_servos + 4
    let length = (data_len_per_servo as usize + 1) * num_servos + 4;
    if length > 255 {
        return Err(FeetechError::ParameterError(
            "SYNC_WRITE packet length exceeds 255 bytes".to_string(),
        ));
    }

    let mut params = Vec::with_capacity(length - 2);
    params.push(start_addr);
    params.push(data_len_per_servo);

    for (id, data) in servo_entries {
        if data.len() != data_len_per_servo as usize {
            return Err(FeetechError::ParameterError(format!(
                "Servo ID {} data length {} does not match expected {}",
                id,
                data.len(),
                data_len_per_servo
            )));
        }
        params.push(*id);
        params.extend_from_slice(data);
    }

    Ok(build_packet(BROADCAST_ID, INST_SYNC_WRITE, &params))
}

/// Zero-allocation builder for `SYNC_WRITE` goal positions (2 bytes per servo) into a caller-
/// provided preallocated buffer `out`. Produces byte-for-byte identical wire output to
/// [`build_sync_write_positions`].
pub fn write_sync_write_positions_into(
    positions: &[(u8, u16)],
    out: &mut Vec<u8>,
) -> Result<(), FeetechError> {
    let num_servos = positions.len();
    if num_servos == 0 {
        return Err(FeetechError::ParameterError(
            "SYNC_WRITE requires at least one servo entry".to_string(),
        ));
    }
    // Length = (data_len + 1) * num_servos + 4 = 3 * num_servos + 4
    let length = 3 * num_servos + 4;
    if length > 255 {
        return Err(FeetechError::ParameterError(
            "SYNC_WRITE packet length exceeds 255 bytes".to_string(),
        ));
    }
    let length_u8 = length as u8;
    out.clear();
    out.extend_from_slice(&FEETECH_HEADER);
    out.push(BROADCAST_ID);
    out.push(length_u8);
    out.push(INST_SYNC_WRITE);
    out.push(REG_GOAL_POSITION);
    out.push(2u8);
    let mut sum: u32 = u32::from(BROADCAST_ID)
        + u32::from(length_u8)
        + u32::from(INST_SYNC_WRITE)
        + u32::from(REG_GOAL_POSITION)
        + 2u32;
    for &(id, pos) in positions {
        let [lo, hi] = pos.to_le_bytes();
        out.push(id);
        out.push(lo);
        out.push(hi);
        sum += u32::from(id) + u32::from(lo) + u32::from(hi);
    }
    out.push(!((sum & 0xFF) as u8));
    Ok(())
}

/// Convenience builder: build `SYNC_WRITE` for goal positions only (2 bytes per servo)
pub fn build_sync_write_positions(positions: &[(u8, u16)]) -> Result<Vec<u8>, FeetechError> {
    let mut out = Vec::with_capacity(8 + positions.len() * 3);
    write_sync_write_positions_into(positions, &mut out)?;
    Ok(out)
}

/// Zero-allocation borrowed view of a parsed Feetech response packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FeetechPacketRef<'a> {
    pub id: u8,
    pub error: u8,
    pub params: &'a [u8],
}

/// Zero-allocation parse error for [`parse_response_ref`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FeetechRefParseError {
    Incomplete,
    InvalidLength,
    ChecksumMismatch,
}

/// Zero-allocation version of [`parse_response`]: returns a borrowed slice into `buf` and the
/// number of bytes consumed without allocating `Vec` or error `String`s.
pub fn parse_response_ref(buf: &[u8]) -> Result<(FeetechPacketRef<'_>, usize), FeetechRefParseError> {
    let header_pos = match buf.windows(2).position(|w| w == FEETECH_HEADER) {
        Some(pos) => pos,
        None => return Err(FeetechRefParseError::Incomplete),
    };
    let slice = &buf[header_pos..];
    if slice.len() < 6 {
        return Err(FeetechRefParseError::Incomplete);
    }
    let id = slice[2];
    let length = slice[3] as usize;
    if !(2..=64).contains(&length) {
        return Err(FeetechRefParseError::InvalidLength);
    }
    let total_packet_len = 4 + length;
    if slice.len() < total_packet_len {
        return Err(FeetechRefParseError::Incomplete);
    }
    let error_byte = slice[4];
    let params_len = length - 2;
    let params = &slice[5..5 + params_len];
    let expected_checksum = slice[4 + length - 1];
    let calc_checksum = calculate_checksum(id, slice[3], error_byte, params);
    if calc_checksum != expected_checksum {
        return Err(FeetechRefParseError::ChecksumMismatch);
    }
    Ok((
        FeetechPacketRef {
            id,
            error: error_byte,
            params,
        },
        header_pos + total_packet_len,
    ))
}

/// Zero-allocation `SYNC_READ` builder writing directly into a caller-owned slice, returning the
/// written packet byte length.
pub fn write_sync_read_slice(
    ids: &[u8],
    start_addr: u8,
    data_len: u8,
    out: &mut [u8],
) -> Result<usize, &'static str> {
    if ids.is_empty() {
        return Err("SYNC_READ_EMPTY_IDS");
    }
    if ids.len() > 32 {
        return Err("SYNC_READ_TOO_MANY_IDS");
    }
    let total = 8 + ids.len();
    if out.len() < total {
        return Err("SYNC_READ_BUF_TOO_SMALL");
    }
    let length = (4 + ids.len()) as u8;
    out[0] = 0xFF;
    out[1] = 0xFF;
    out[2] = BROADCAST_ID;
    out[3] = length;
    out[4] = INST_SYNC_READ;
    out[5] = start_addr;
    out[6] = data_len;
    out[7..7 + ids.len()].copy_from_slice(ids);
    let mut sum: u32 = u32::from(BROADCAST_ID)
        + u32::from(length)
        + u32::from(INST_SYNC_READ)
        + u32::from(start_addr)
        + u32::from(data_len);
    for &id in ids {
        sum += u32::from(id);
    }
    out[7 + ids.len()] = (!(sum & 0xFF)) as u8;
    Ok(total)
}

/// Zero-allocation `SYNC_READ` builder writing directly into a caller-owned buffer.
pub fn write_sync_read_into(
    ids: &[u8],
    start_addr: u8,
    data_len: u8,
    out: &mut Vec<u8>,
) -> Result<(), &'static str> {
    if ids.is_empty() {
        return Err("SYNC_READ_EMPTY_IDS");
    }
    if ids.len() > 32 {
        return Err("SYNC_READ_TOO_MANY_IDS");
    }
    out.clear();
    let length = (4 + ids.len()) as u8;
    out.push(0xFF);
    out.push(0xFF);
    out.push(BROADCAST_ID);
    out.push(length);
    out.push(INST_SYNC_READ);
    out.push(start_addr);
    out.push(data_len);
    out.extend_from_slice(ids);
    let mut sum: u32 = u32::from(BROADCAST_ID)
        + u32::from(length)
        + u32::from(INST_SYNC_READ)
        + u32::from(start_addr)
        + u32::from(data_len);
    for &id in ids {
        sum += u32::from(id);
    }
    out.push((!(sum & 0xFF)) as u8);
    Ok(())
}

/// Parse a response packet from a byte stream.
/// Returns the parsed `FeetechPacket` and the number of bytes consumed.
pub fn parse_response(buf: &[u8]) -> Result<(FeetechPacket, usize), FeetechError> {
    // Scan for header 0xFF 0xFF
    let header_pos = match buf.windows(2).position(|w| w == FEETECH_HEADER) {
        Some(pos) => pos,
        None => {
            return Err(FeetechError::IncompleteFrame {
                expected: 6,
                actual: buf.len(),
            });
        }
    };

    let slice = &buf[header_pos..];
    if slice.len() < 6 {
        return Err(FeetechError::IncompleteFrame {
            expected: 6,
            actual: slice.len(),
        });
    }

    let id = slice[2];
    let length = slice[3] as usize;
    if !(2..=64).contains(&length) {
        return Err(FeetechError::ParameterError(format!(
            "Invalid Feetech packet length field: {}",
            length
        )));
    }

    let total_packet_len = 4 + length; // 2 header + 1 id + 1 length + (length)
    if slice.len() < total_packet_len {
        return Err(FeetechError::IncompleteFrame {
            expected: total_packet_len,
            actual: slice.len(),
        });
    }

    let error_byte = slice[4];
    let params_len = length - 2;
    let params = slice[5..5 + params_len].to_vec();
    let expected_checksum = slice[4 + length - 1];

    let calc_checksum = calculate_checksum(id, slice[3], error_byte, &params);
    if calc_checksum != expected_checksum {
        return Err(FeetechError::ChecksumMismatch {
            expected: expected_checksum,
            calculated: calc_checksum,
        });
    }

    let consumed = header_pos + total_packet_len;
    Ok((
        FeetechPacket {
            id,
            error: error_byte,
            params,
        },
        consumed,
    ))
}

/// Extract the trailing unparsed partial frame slice from `buf` for a SyncRead response, if present.
/// Validates that:
/// - Single `0xFF` or `[0xFF, 0xFF]` are accepted as partial header starts.
/// - Once `remaining.len() >= 3`, `expected_ids.contains(&remaining[2])` must hold, else advances cursor.
/// - Once `remaining.len() >= 4`, `remaining[3] == expected_len` must hold, else advances cursor.
pub fn trailing_partial_frame_for_sync_read<'a>(
    buf: &'a [u8],
    expected_ids: &[u8],
    expected_len: u8,
) -> Option<&'a [u8]> {
    let mut cursor = 0;
    while cursor < buf.len() {
        if buf[cursor] != 0xFF {
            let next_ff = buf[cursor..]
                .iter()
                .position(|&b| b == 0xFF)
                .map(|p| cursor + p)
                .unwrap_or(buf.len());
            cursor = next_ff;
            if cursor >= buf.len() {
                break;
            }
        }
        let remaining = &buf[cursor..];
        if remaining.len() == 1 {
            return Some(remaining);
        }
        if remaining[1] != 0xFF {
            cursor += 1;
            continue;
        }
        if remaining.len() >= 3 && !expected_ids.contains(&remaining[2]) {
            cursor += 1;
            continue;
        }
        if remaining.len() >= 4 && remaining[3] != expected_len {
            cursor += 1;
            continue;
        }
        match parse_response(remaining) {
            Ok((_pkt, consumed)) => {
                cursor += consumed;
            }
            Err(FeetechError::IncompleteFrame { .. }) => {
                return Some(remaining);
            }
            Err(_) => {
                cursor += 1;
            }
        }
    }
    None
}

/// Extract the trailing unparsed partial frame slice from `buf`, if present.
pub fn trailing_partial_frame(buf: &[u8]) -> Option<&[u8]> {
    let mut cursor = 0;
    while cursor < buf.len() {
        if buf[cursor] != 0xFF {
            let next_ff = buf[cursor..]
                .iter()
                .position(|&b| b == 0xFF)
                .map(|p| cursor + p)
                .unwrap_or(buf.len());
            cursor = next_ff;
            if cursor >= buf.len() {
                break;
            }
        }
        let remaining = &buf[cursor..];
        if remaining.len() == 1 {
            // Reply split right after first header byte 0xFF!
            return Some(remaining);
        }
        if remaining[1] != 0xFF {
            cursor += 1;
            continue;
        }
        match parse_response(remaining) {
            Ok((_pkt, consumed)) => {
                cursor += consumed;
            }
            Err(FeetechError::IncompleteFrame { .. }) => {
                return Some(remaining);
            }
            Err(_) => {
                cursor += 1;
            }
        }
    }
    None
}

// =========================================================================
// Physical Unit Conversions (Radians <-> Raw Steps, Temperature, Torque)
// =========================================================================

/// Convert raw 12-bit encoder steps (0..4095) to radians around zero offset
pub fn steps_to_rad(steps: u16, zero_offset: u16) -> f32 {
    let diff = steps as f32 - zero_offset as f32;
    diff * (2.0 * PI / STEPS_PER_REVOLUTION)
}

/// Convert radians to raw 12-bit encoder steps (clamped to 0..4095)
pub fn rad_to_steps(rad: f32, zero_offset: u16) -> u16 {
    let raw = (rad * STEPS_PER_REVOLUTION / (2.0 * PI) + zero_offset as f32).round();
    raw.clamp(0.0, 4095.0) as u16
}

/// Convert raw temperature reading (u8) to degrees Celsius
pub fn raw_to_temperature(raw: u8) -> f32 {
    raw as f32
}

/// Convert raw load value to torque in Newton-meters (Nm)
/// Raw load: bit 10 is direction, bits 0..9 represent 0..1000 (0.1% max torque)
pub fn raw_load_to_torque(raw: u16) -> f32 {
    let direction = if (raw & 0x0400) != 0 { -1.0 } else { 1.0 };
    let magnitude = (raw & 0x03FF).min(1000) as f32 / 1000.0;
    direction * magnitude * RATED_TORQUE_NM
}

/// Convert raw load value to signed fraction in [-1.0, 1.0]
/// Raw load: bit 10 is negative direction, bits 0..=9 clamped to 1000 / 1000.0
pub fn raw_load_to_fraction(raw: u16) -> f32 {
    let direction = if (raw & 0x0400) != 0 { -1.0 } else { 1.0 };
    let magnitude = (raw & 0x03FF).min(1000) as f32 / 1000.0;
    (direction * magnitude).clamp(-1.0, 1.0)
}

/// Convert raw 16-bit STS3215 speed to rad/s.
/// Sign-magnitude representation: bit 15 (0x8000) is negative sign, bits 0..=14 are magnitude in steps/s.
pub fn raw_speed_to_rad_s(raw: u16) -> f32 {
    let sign = if (raw & 0x8000) != 0 { -1.0 } else { 1.0 };
    let mag = (raw & 0x7FFF) as f32;
    sign * mag * (2.0 * PI / STEPS_PER_REVOLUTION)
}

/// Encode signed integer into sign-magnitude representation with sign_bit.
pub fn encode_sign_magnitude(val: i16, sign_bit: u16) -> u16 {
    let mask = sign_bit.saturating_sub(1);
    if val < 0 {
        let mag = val.unsigned_abs().min(mask);
        sign_bit | mag
    } else {
        (val as u16).min(mask)
    }
}

/// Decode sign-magnitude raw representation with sign_bit into signed integer.
pub fn decode_sign_magnitude(raw: u16, sign_bit: u16) -> i16 {
    let mask = sign_bit.saturating_sub(1);
    let mag = (raw & mask).min(i16::MAX as u16) as i16;
    if (raw & sign_bit) != 0 {
        -mag
    } else {
        mag
    }
}

/// Decode STS3215 Present_Position 16-bit register value into signed continuous steps.
/// Bit 15 (0x8000) indicates negative sign, bits 0..=14 are magnitude in steps.
pub fn decode_present_position(raw: u16) -> i32 {
    let mag = (raw & 0x7FFF) as i32;
    if (raw & 0x8000) != 0 {
        -mag
    } else {
        mag
    }
}

/// Decode Feetech STS3215 status/error byte into human-readable flags
pub fn feetech_error_name(status_byte: u8) -> Option<String> {
    if status_byte == 0 {
        return None;
    }
    let mut flags = Vec::new();
    if (status_byte & 0x01) != 0 {
        flags.push("Voltage".to_string());
    }
    if (status_byte & 0x02) != 0 {
        flags.push("Sensor".to_string());
    }
    if (status_byte & 0x04) != 0 {
        flags.push("Overtemperature".to_string());
    }
    if (status_byte & 0x08) != 0 {
        flags.push("Overcurrent".to_string());
    }
    if (status_byte & 0x20) != 0 {
        flags.push("Overload".to_string());
    }
    let known_mask = 0x01 | 0x02 | 0x04 | 0x08 | 0x20;
    let unknown = status_byte & !known_mask;
    if unknown != 0 {
        flags.push(format!("Error(0x{:02X})", unknown));
    }
    if flags.is_empty() {
        None
    } else {
        Some(flags.join(" | "))
    }
}
