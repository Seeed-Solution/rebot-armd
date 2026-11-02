use std::f32::consts::PI;
use thiserror::Error;

/// Header bytes for FashionStar UART/RS485 bus servo protocol
pub const FASHIONSTAR_HEADER: [u8; 2] = [0x12, 0x4C];
pub const FASHIONSTAR_REQUEST_HEADER: [u8; 2] = [0x12, 0x4C];
pub const FASHIONSTAR_RESPONSE_HEADER: [u8; 2] = [0x05, 0x1C];
/// Broadcast ID
pub const BROADCAST_ID: u8 = 0xFE;

// Command IDs
pub const CMD_PING: u8 = 0x01;
pub const CMD_RESET: u8 = 0x02;
pub const CMD_SET_SERVO_ANGLE: u8 = 0x08;
pub const CMD_QUERY_SERVO_ANGLE: u8 = 0x0A;
pub const CMD_QUERY_RAW_MAGNETIC: u8 = 0x0C;
pub const CMD_SET_TORQUE: u8 = 0x1A;
pub const CMD_QUERY_STATUS: u8 = 0x1E;
pub const CMD_BATCH_QUERY_ANGLES: u8 = 0x20;

/// Total resolution steps in one full revolution for magnetic encoder
pub const STEPS_PER_REVOLUTION: f32 = 4096.0;
/// Default center position (0.0 rad)
pub const DEFAULT_ZERO_OFFSET: u16 = 2048;

#[derive(Debug, Error, PartialEq, Eq)]
pub enum FashionStarError {
    #[error("Header mismatch: expected [0x12, 0x4C] or [0x05, 0x1C]")]
    InvalidHeader,

    #[error("Incomplete frame: expected at least {expected} bytes, got {actual}")]
    IncompleteFrame { expected: usize, actual: usize },

    #[error("Checksum mismatch: expected 0x{expected:02X}, calculated 0x{calculated:02X}")]
    ChecksumMismatch { expected: u8, calculated: u8 },

    #[error("Parameter error: {0}")]
    ParameterError(String),
}

/// Parsed FashionStar packet
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FashionStarPacket {
    pub header: [u8; 2],
    pub id: u8,
    pub cmd_id: u8,
    pub content: Vec<u8>,
}

/// Calculate FashionStar checksum: `(header[0] + header[1] + cmd_id + length + id + sum(payload)) & 0xFF`
pub fn calculate_checksum(header: [u8; 2], cmd_id: u8, length: u8, id: u8, payload: &[u8]) -> u8 {
    let mut sum: u32 = header[0] as u32 + header[1] as u32 + cmd_id as u32 + length as u32 + id as u32;
    for &b in payload {
        sum += b as u32;
    }
    (sum & 0xFF) as u8
}

/// Build a FashionStar packet with a specified header
/// Wire frame: `[Header0, Header1, Cmd, Length, Id, ...payload, Checksum]`
/// where `Length = payload.len() + 1` (includes Id byte)
pub fn build_packet_with_header(header: [u8; 2], id: u8, cmd_id: u8, content: &[u8]) -> Vec<u8> {
    let length = (content.len() + 1) as u8;
    let checksum = calculate_checksum(header, cmd_id, length, id, content);

    let mut packet = Vec::with_capacity(6 + content.len());
    packet.extend_from_slice(&header);
    packet.push(cmd_id);
    packet.push(length);
    packet.push(id);
    packet.extend_from_slice(content);
    packet.push(checksum);
    packet
}

/// Build a FashionStar request packet (default request header [0x12, 0x4C])
pub fn build_packet(id: u8, cmd_id: u8, content: &[u8]) -> Vec<u8> {
    build_packet_with_header(FASHIONSTAR_REQUEST_HEADER, id, cmd_id, content)
}

/// Build Ping packet
pub fn build_ping(id: u8) -> Vec<u8> {
    build_packet(id, CMD_PING, &[])
}

/// Build Set Torque packet (1 byte: 1=on, 0=off)
pub fn build_set_torque(id: u8, enabled: bool) -> Vec<u8> {
    let val = if enabled { 1u8 } else { 0u8 };
    build_packet(id, CMD_SET_TORQUE, &[val])
}

/// Build Query Angle packet (CMD 0x0A)
pub fn build_query_angle(id: u8) -> Vec<u8> {
    build_packet(id, CMD_QUERY_SERVO_ANGLE, &[])
}

/// Build Query Raw Magnetic Encoder packet (CMD 0x0C)
pub fn build_query_magnetic(id: u8) -> Vec<u8> {
    build_packet(id, CMD_QUERY_RAW_MAGNETIC, &[])
}

/// Build Query Status packet (CMD 0x1E)
pub fn build_query_status(id: u8) -> Vec<u8> {
    build_packet(id, CMD_QUERY_STATUS, &[])
}

/// Build Batch Query packet for multiple servo IDs
pub fn build_batch_query(ids: &[u8]) -> Vec<u8> {
    build_packet(BROADCAST_ID, CMD_BATCH_QUERY_ANGLES, ids)
}

/// Build Set Angle packet (angle in 0.1 degrees, interval in ms, power)
pub fn build_set_angle(id: u8, angle_tenth_deg: i16, interval_ms: u16, power: u16) -> Vec<u8> {
    let a_bytes = angle_tenth_deg.to_le_bytes();
    let i_bytes = interval_ms.to_le_bytes();
    let p_bytes = power.to_le_bytes();
    let content = [
        a_bytes[0], a_bytes[1],
        i_bytes[0], i_bytes[1],
        p_bytes[0], p_bytes[1],
    ];
    build_packet(id, CMD_SET_SERVO_ANGLE, &content)
}

/// Parse a FashionStar packet from a byte stream, matching either request header [0x12, 0x4C] or response header [0x05, 0x1C]
/// Wire frame: `[Header0, Header1, Cmd, Length, Id, ...payload, Checksum]`
pub fn parse_packet(buf: &[u8]) -> Result<(FashionStarPacket, usize), FashionStarError> {
    let header_pos = match buf.windows(2).position(|w| w == FASHIONSTAR_REQUEST_HEADER || w == FASHIONSTAR_RESPONSE_HEADER) {
        Some(pos) => pos,
        None => {
            return Err(FashionStarError::IncompleteFrame {
                expected: 6,
                actual: buf.len(),
            });
        }
    };

    let slice = &buf[header_pos..];
    // Minimum frame: Header(2) + Cmd(1) + Length(1) + Id(1) + Checksum(1) = 6 bytes
    if slice.len() < 6 {
        return Err(FashionStarError::IncompleteFrame {
            expected: 6,
            actual: slice.len(),
        });
    }

    let header = [slice[0], slice[1]];
    let cmd_id = slice[2];
    let length = slice[3] as usize;
    if length < 1 {
        return Err(FashionStarError::ParameterError(format!(
            "Invalid FashionStar length field: {}",
            length
        )));
    }
    let total_len = 5 + length; // Header(2) + Cmd(1) + Length(1) + payload_and_id(length) + Checksum(1)

    if slice.len() < total_len {
        return Err(FashionStarError::IncompleteFrame {
            expected: total_len,
            actual: slice.len(),
        });
    }

    let id = slice[4];
    let payload = if length > 1 {
        slice[5..4 + length].to_vec()
    } else {
        Vec::new()
    };
    let expected_checksum = slice[4 + length];
    let calculated_checksum = calculate_checksum(header, cmd_id, length as u8, id, &payload);

    if calculated_checksum != expected_checksum {
        return Err(FashionStarError::ChecksumMismatch {
            expected: expected_checksum,
            calculated: calculated_checksum,
        });
    }

    let consumed = header_pos + total_len;
    Ok((
        FashionStarPacket {
            header,
            id,
            cmd_id,
            content: payload,
        },
        consumed,
    ))
}

// =========================================================================
// Physical Unit Conversions (Radians <-> Tenth-degrees / Magnetic Encoder)
// =========================================================================

/// Convert angle in 0.1 degrees (-1800..1800) to radians
pub fn tenth_deg_to_rad(tenth_deg: i16) -> f32 {
    (tenth_deg as f32) * 0.1 * (PI / 180.0)
}

/// Convert radians to angle in 0.1 degrees
pub fn rad_to_tenth_deg(rad: f32) -> i16 {
    (rad * (180.0 / PI) * 10.0).round() as i16
}

/// Convert raw 12-bit magnetic encoder reading (0..4095) to radians around zero offset
pub fn magnetic_to_rad(encoder: u16, zero_offset: u16) -> f32 {
    let diff = encoder as f32 - zero_offset as f32;
    diff * (2.0 * PI / STEPS_PER_REVOLUTION)
}

/// Convert radians to raw 12-bit magnetic encoder reading (clamped to 0..4095)
pub fn rad_to_magnetic(rad: f32, zero_offset: u16) -> u16 {
    let raw = (rad * STEPS_PER_REVOLUTION / (2.0 * PI) + zero_offset as f32).round();
    raw.clamp(0.0, 4095.0) as u16
}

/// Parsed status telemetry for a FashionStar servo
#[derive(Debug, Clone, PartialEq)]
pub struct FashionStarTelemetry {
    pub id: u8,
    pub angle_rad: f32,
    pub temperature_c: f32,
    pub voltage_v: f32,
    pub current_ma: f32,
    pub status: u8,
}

/// Decode status telemetry from CMD_QUERY_STATUS response content
/// Content: [volt_lo, volt_hi (mV), temp (C), curr_lo, curr_hi (mA), status]
pub fn decode_status_content(id: u8, content: &[u8]) -> Result<FashionStarTelemetry, FashionStarError> {
    if content.len() < 6 {
        return Err(FashionStarError::IncompleteFrame {
            expected: 6,
            actual: content.len(),
        });
    }

    let volt_mv = u16::from_le_bytes([content[0], content[1]]);
    let temp_c = content[2] as f32;
    let curr_ma = i16::from_le_bytes([content[3], content[4]]) as f32;
    let status = content[5];

    Ok(FashionStarTelemetry {
        id,
        angle_rad: 0.0,
        temperature_c: temp_c,
        voltage_v: volt_mv as f32 / 1000.0,
        current_ma: curr_ma,
        status,
    })
}
