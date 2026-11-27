//! Feetech STS/SMS register addresses, constants, and servo calibration tables.

use std::f32::consts::PI;
use serde::{Deserialize, Serialize};
use super::codec::FeetechError;

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

impl FeetechConfig {
    pub fn joint_calib(&self, joint: usize) -> FeetechJointCalib {
        self.joints.get(joint).cloned().unwrap_or_default()
    }
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
