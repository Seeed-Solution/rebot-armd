use std::f32::consts::PI;
use serde::{Deserialize, Serialize};
use thiserror::Error;

/// Header bytes for Feetech STS/SMS series servo packets
pub const FEETECH_HEADER: [u8; 2] = [0xFF, 0xFF];
/// Broadcast ID for synchronizing all servos
pub const BROADCAST_ID: u8 = 0xFE;

// Instruction codes
pub const INST_PING: u8 = 0x01;
pub const INST_READ: u8 = 0x02;
pub const INST_WRITE: u8 = 0x03;
pub const INST_REG_WRITE: u8 = 0x04;
pub const INST_ACTION: u8 = 0x05;
pub const INST_RESET: u8 = 0x06;
pub const INST_SYNC_READ: u8 = 0x82;
pub const INST_SYNC_WRITE: u8 = 0x83;

// Memory register addresses for Feetech STS3215
pub const REG_MODEL_L: u8 = 0x03;
pub const EXPECTED_MODEL_STS3215: u16 = 777;
pub const REG_ID: u8 = 0x05;
pub const REG_BAUD_RATE: u8 = 0x06;
pub const REG_RETURN_DELAY: u8 = 0x07;
pub const REG_STATUS_RETURN_LEVEL: u8 = 0x08;
pub const REG_MIN_ANGLE_LIMIT_L: u8 = 0x09;
pub const REG_MIN_ANGLE: u8 = 0x09;
pub const REG_MAX_ANGLE_LIMIT_L: u8 = 0x0B;
pub const REG_MAX_ANGLE: u8 = 0x0B;
pub const REG_MAX_TEMP_LIMIT: u8 = 0x0D;
pub const REG_MAX_TEMP: u8 = 0x0D;
pub const REG_MAX_TORQUE_L: u8 = 0x10;
pub const REG_UNLOADING_CONDITION: u8 = 0x13;
pub const REG_P_COEF: u8 = 0x15;
pub const REG_D_COEF: u8 = 0x16;
pub const REG_I_COEF: u8 = 0x17;
pub const REG_MIN_STARTUP_FORCE_L: u8 = 0x18;
pub const REG_CW_DEAD_ZONE: u8 = 0x1A;
pub const REG_CCW_DEAD_ZONE: u8 = 0x1B;
pub const REG_PROTECTION_CURRENT_L: u8 = 0x1C;
pub const REG_HOMING_OFFSET_L: u8 = 0x1F;
pub const REG_OPERATING_MODE: u8 = 0x21;
pub const REG_PROTECTIVE_TORQUE: u8 = 0x22;
pub const REG_PROTECTION_TIME: u8 = 0x23;
pub const REG_OVERLOAD_TORQUE: u8 = 0x24;
pub const REG_TORQUE_ENABLE: u8 = 0x28;
pub const REG_ACC: u8 = 0x29;
pub const REG_ACCELERATION: u8 = 0x29;
pub const REG_GOAL_POSITION_L: u8 = 0x2A;
pub const REG_GOAL_POSITION: u8 = 0x2A;
pub const REG_GOAL_TIME_L: u8 = 0x2C;
pub const REG_GOAL_TIME: u8 = 0x2C;
pub const REG_GOAL_VELOCITY_L: u8 = 0x2E;
pub const REG_GOAL_SPEED: u8 = 0x2E;
pub const REG_TORQUE_LIMIT_L: u8 = 0x30;
pub const REG_LOCK: u8 = 0x37;
pub const REG_PRESENT_POSITION_L: u8 = 0x38;
pub const REG_PRESENT_POSITION: u8 = 0x38;
pub const REG_PRESENT_VELOCITY_L: u8 = 0x3A;
pub const REG_PRESENT_SPEED: u8 = 0x3A;
pub const REG_PRESENT_LOAD_L: u8 = 0x3C;
pub const REG_PRESENT_LOAD: u8 = 0x3C;
pub const REG_PRESENT_VOLTAGE: u8 = 0x3E;
pub const REG_PRESENT_TEMPERATURE: u8 = 0x3F;
pub const REG_MOVING: u8 = 0x42;
pub const REG_PRESENT_CURRENT_L: u8 = 0x45;
pub const REG_PRESENT_CURRENT: u8 = 0x45;

/// Total resolution steps in one full revolution (360 degrees)
pub const STEPS_PER_REVOLUTION: f32 = 4096.0;
/// Default center position (0.0 rad)
pub const DEFAULT_ZERO_OFFSET: u16 = 2048;
/// Rated stall torque of STS3215 in Nm (~30 kg-cm at 7.4V)
pub const RATED_TORQUE_NM: f32 = 3.0;

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

/// Build a Read register packet
pub fn build_read(id: u8, start_addr: u8, length_to_read: u8) -> Vec<u8> {
    build_packet(id, INST_READ, &[start_addr, length_to_read])
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

/// Convenience builder: build `SYNC_WRITE` for goal positions only (2 bytes per servo)
pub fn build_sync_write_positions(positions: &[(u8, u16)]) -> Result<Vec<u8>, FeetechError> {
    let entries: Vec<(u8, Vec<u8>)> = positions
        .iter()
        .map(|&(id, pos)| (id, pos.to_le_bytes().to_vec()))
        .collect();
    build_sync_write(REG_GOAL_POSITION, 2, &entries)
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
    let mask = sign_bit - 1;
    if val < 0 {
        let mag = val.unsigned_abs().min(mask);
        sign_bit | mag
    } else {
        (val as u16).min(mask)
    }
}

/// Decode sign-magnitude raw representation with sign_bit into signed integer.
pub fn decode_sign_magnitude(raw: u16, sign_bit: u16) -> i16 {
    let mask = sign_bit - 1;
    let mag = (raw & mask) as i16;
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

fn default_zero_step() -> u16 { 2048 }
fn default_true() -> bool { true }
fn default_p_coef() -> u8 { 16 }
fn default_d_coef() -> u8 { 32 }
fn default_max_torque() -> u16 { 1000 }
fn default_prot_current() -> u16 { 500 }

/// Per-joint Feetech servo calibration and limits
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct FeetechJointCalib {
    pub id: u8,
    #[serde(default)]
    pub drive_mode_invert: bool,
    #[serde(default = "default_zero_step")]
    pub zero_step: u16,
    #[serde(default)]
    pub homing_offset_steps: i16,
    #[serde(default = "default_true")]
    pub homing_applied_in_hw: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub range_min_steps: Option<u16>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub range_max_steps: Option<u16>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub expected_homing_offset_steps: Option<i16>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_torque_raw: Option<u16>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub protection_current_raw: Option<u16>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub overload_torque_raw: Option<u8>,
}

impl Default for FeetechJointCalib {
    fn default() -> Self {
        Self {
            id: 1,
            drive_mode_invert: false,
            zero_step: 2048,
            homing_offset_steps: 0,
            homing_applied_in_hw: true,
            range_min_steps: None,
            range_max_steps: None,
            expected_homing_offset_steps: None,
            max_torque_raw: None,
            protection_current_raw: None,
            overload_torque_raw: None,
        }
    }
}

impl FeetechJointCalib {
    pub fn effective_range_steps(&self) -> (u16, u16) {
        (
            self.range_min_steps.unwrap_or(0),
            self.range_max_steps.unwrap_or(4095),
        )
    }

    pub fn steps_to_rad(&self, raw: i32) -> f32 {
        let effective_zero = if self.homing_applied_in_hw {
            self.zero_step as f32
        } else {
            self.zero_step as f32 + self.homing_offset_steps as f32
        };
        let diff = if self.drive_mode_invert {
            effective_zero - (raw as f32)
        } else {
            (raw as f32) - effective_zero
        };
        diff * (2.0 * PI / STEPS_PER_REVOLUTION)
    }

    pub fn rad_to_steps(&self, rad: f32) -> u16 {
        let (min, max) = self.effective_range_steps();
        if !rad.is_finite() {
            return self.zero_step.clamp(min, max);
        }
        let delta = rad * STEPS_PER_REVOLUTION / (2.0 * PI);
        let effective_zero = if self.homing_applied_in_hw {
            self.zero_step as f32
        } else {
            self.zero_step as f32 + self.homing_offset_steps as f32
        };
        let raw_f = if self.drive_mode_invert {
            effective_zero - delta
        } else {
            effective_zero + delta
        };
        let target = raw_f.round() as i32;
        let clamped = target.clamp(min as i32, max as i32);
        clamped as u16
    }

    pub fn checked_rad_to_steps_in_range(
        &self,
        rad: f32,
        range: (u16, u16),
    ) -> Result<u16, FeetechError> {
        if !rad.is_finite() {
            return Err(FeetechError::ParameterError(format!(
                "Non-finite joint target position (rad = {}) for servo ID {}",
                rad, self.id
            )));
        }
        let delta = rad * STEPS_PER_REVOLUTION / (2.0 * PI);
        let effective_zero = if self.homing_applied_in_hw {
            self.zero_step as f32
        } else {
            self.zero_step as f32 + self.homing_offset_steps as f32
        };
        let raw_f = if self.drive_mode_invert {
            effective_zero - delta
        } else {
            effective_zero + delta
        };
        let target = raw_f.round() as i32;
        let (min, max) = range;
        const STEP_TOLERANCE: i32 = 2;
        if target < (min as i32 - STEP_TOLERANCE) || target > (max as i32 + STEP_TOLERANCE) {
            return Err(FeetechError::ParameterError(format!(
                "Joint target step {} (from rad = {}) outside range [{}, {}] for servo ID {}",
                target, rad, min, max, self.id
            )));
        }
        let clamped = target.clamp(min as i32, max as i32);
        Ok(clamped as u16)
    }

    pub fn checked_rad_to_steps(&self, rad: f32) -> Result<u16, FeetechError> {
        self.checked_rad_to_steps_in_range(rad, self.effective_range_steps())
    }
}

/// Global Feetech bus configuration and default servo parameters
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct FeetechConfig {
    #[serde(default)]
    pub calibrated: bool,
    #[serde(default)]
    pub tx_echo: Option<bool>,
    #[serde(default)]
    pub return_delay_raw: u8,
    #[serde(default = "default_p_coef")]
    pub p_coef: u8,
    #[serde(default)]
    pub i_coef: u8,
    #[serde(default = "default_d_coef")]
    pub d_coef: u8,
    #[serde(default = "default_max_torque")]
    pub max_torque_raw: u16,
    #[serde(default = "default_prot_current")]
    pub protection_current_raw: u16,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub overload_torque_raw: Option<u8>,
    #[serde(default)]
    pub joints: Vec<FeetechJointCalib>,
}

impl Default for FeetechConfig {
    fn default() -> Self {
        Self {
            calibrated: false,
            tx_echo: None,
            return_delay_raw: 0,
            p_coef: 16,
            i_coef: 0,
            d_coef: 32,
            max_torque_raw: 1000,
            protection_current_raw: 500,
            overload_torque_raw: None,
            joints: Vec::new(),
        }
    }
}
