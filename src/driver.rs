use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::model::JointState;
pub use crate::model::{DriverCapabilities, DriverRangeCheck, EnableState, JointRangeReading, RangeCheckStatus};
use crate::protocol::damiao::{self, DamiaoLimits};
pub use crate::protocol::damiao::{build_pos_force_frame, DAMIAO_GRIPPER_POSFORCE_VEL_RAW};
use crate::protocol::fashionstar::{self, FashionStarPacket};
use crate::protocol::feetech;
use crate::protocol::robstride::{self, RobStrideLimits};
use crate::transport::{CanFrame, CanTransport, Transport, TransportError};

/// Desired control target for a single joint
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointTarget {
    /// Target position in radians
    pub position: f32,
    /// Target velocity in rad/s (optional)
    pub velocity: Option<f32>,
    /// Target feedforward torque in Nm (optional)
    pub torque: Option<f32>,
    /// Position stiffness gain Kp (optional, used in impedance/MIT control)
    pub kp: Option<f32>,
    /// Velocity damping gain Kd (optional, used in impedance/MIT control)
    pub kd: Option<f32>,
}

impl JointTarget {
    /// Create a pure position target
    pub fn position(position: f32) -> Self {
        Self {
            position,
            velocity: None,
            torque: None,
            kp: None,
            kd: None,
        }
    }

    /// Create an MIT/impedance control target
    pub fn mit(position: f32, velocity: f32, kp: f32, kd: f32, torque: f32) -> Self {
        Self {
            position,
            velocity: Some(velocity),
            torque: Some(torque),
            kp: Some(kp),
            kd: Some(kd),
        }
    }
}

/// Errors occurring at the unified driver layer
#[derive(Debug, Error)]
pub enum ArmDriverError {
    #[error("Transport error: {0}")]
    Transport(#[from] TransportError),

    #[error("Protocol error: {0}")]
    Protocol(String),

    #[error("Joint count mismatch: expected {expected}, got {actual}")]
    JointCountMismatch { expected: usize, actual: usize },

    #[error("Hardware fault: {0}")]
    HardwareFault(String),

    #[error("Timeout waiting for response from joint {0}")]
    Timeout(u8),
}

/// Universal robotic arm hardware driver trait
/// Name of a Damiao feedback state code that reports a fault (8 and above); None for
/// the normal states (0 disabled, 1 enabled).
pub fn damiao_fault_name(code: u8) -> Option<&'static str> {
    Some(match code {
        0..=7 => return None,
        8 => "Overvoltage",
        9 => "Undervoltage",
        0xA => "Overcurrent",
        0xB => "MOS Overtemperature",
        0xC => "Rotor Overtemperature",
        0xD => "CAN Communication Loss",
        0xE => "Overload",
        _ => "Motor Hardware Fault",
    })
}

pub trait ArmDriver: Send {
    fn capabilities(&self) -> DriverCapabilities { DriverCapabilities::default() }

    fn enable_state(&self) -> EnableState { EnableState::Unknown }

    /// Age of the last received sample for each joint. Reading a cache does not refresh it.
    fn feedback_ages(&self) -> Vec<Option<Duration>> { Vec::new() }

    /// Fixed device-side velocity ceiling, when the active output mode has one.
    fn max_command_velocity(&self, _joint: usize) -> Option<f32> { None }

    /// Device-side position limits (min, max) in radians, when the hardware derives them.
    fn position_limits(&self, _joint: usize) -> Option<(f32, f32)> { None }

    /// Read real-time telemetry / joint states for all joints
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError>;

    /// Send target positions / commands to all joints
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError>;

    /// Enable or disable torque / power across all joints
    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError>;

    /// Check if real physical telemetry feedback has been received for all joints
    fn is_feedback_valid(&self) -> bool {
        false
    }

    /// Check if any motor hardware fault is reported
    fn check_hardware_status(&self) -> Result<(), ArmDriverError> {
        Ok(())
    }

    /// Reset internal fault state flags when clearing safety faults
    fn clear_fault(&mut self) -> Result<(), ArmDriverError> {
        Ok(())
    }

    /// Return indices of joints disabled during clear_fault
    fn clear_fault_disables_joints(&self) -> Vec<usize> {
        Vec::new()
    }

    /// True when clear_fault left some joints enabled and some disabled
    fn has_known_partial_enable(&self) -> bool {
        false
    }

    /// True when any joint is known to have torque enabled
    fn has_any_joint_enabled(&self) -> bool {
        self.enable_state() == EnableState::Enabled
    }

    /// Re-read physical enable state of all joints from hardware
    fn refresh_enable_state(&mut self) -> Result<EnableState, ArmDriverError> {
        Ok(self.enable_state())
    }

    /// Set an atomic abort flag that hardware loops should check to abort early on E-stop
    fn set_abort_flag(&mut self, _flag: std::sync::Arc<std::sync::atomic::AtomicBool>) {}

    /// Set the gripper joint index if the arm model defines a gripper
    fn set_gripper_index(&mut self, _index: Option<usize>) {}

    /// Check whether driver / hardware is actively enabled and online
    fn is_enabled(&self) -> bool {
        false
    }

    /// Return per-joint motor status or error code if supported by the driver
    fn motor_status(&self) -> Option<Vec<u8>> {
        None
    }

    /// Return per-joint load telemetry (normalized fraction in [-1.0, 1.0]) if supported
    fn joint_loads(&self) -> Option<Vec<f32>> {
        None
    }

    /// Decode motor fault/status code into descriptive string
    fn decode_motor_error(&self, code: u8) -> Option<String> {
        damiao_fault_name(code).map(str::to_string)
    }

    /// Update internal software enabled flag without transmitting physical frames
    fn set_torque_flag(&mut self, _enabled: bool) {}

    /// Update pending_rearm flag to force full hardware handshake on next enable
    fn set_pending_rearm(&mut self, _pending: bool) {}

    /// True when single joints can be switched between the driver's position mode and
    /// MIT impedance mode while torqued (`set_joint_modes`). Used by gravity compensation.
    fn supports_mit_mode_switch(&self) -> bool { false }

    /// Switch joint `i` to `modes[i]`. Each switched joint receives `hold[i]` in its new
    /// mode immediately after the switch, so it is never left without a target. A joint
    /// whose switch cannot be confirmed is switched back; joints that still cannot be
    /// confirmed are reported by `joint_modes_uncertain` and get both a position frame
    /// and an MIT frame on every write until a later switch confirms them.
    fn set_joint_modes(&mut self, _modes: &[JointMode], _hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        Err(ArmDriverError::Protocol("joint mode switching is not supported by this driver".into()))
    }

    /// At least one joint is in an unconfirmed control mode after a failed switch.
    fn joint_modes_uncertain(&self) -> bool { false }

    /// Result of the startup check of the motors' mapping ranges against the
    /// configured ones; None when the driver has no such check or it has not run.
    fn range_check(&self) -> Option<DriverRangeCheck> { None }
}

/// Output mode of one joint on drivers that can switch at runtime.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum JointMode {
    /// The driver's position mode (Damiao POS_VEL).
    Position,
    /// MIT impedance: kp, kd, velocity and torque feed-forward from the JointTarget.
    Mit,
}

// =========================================================================
// 1. Feetech STS3215 Arm Driver (SO-ARM 101)
// =========================================================================

pub struct FeetechArmDriver<T: Transport> {
    transport: T,
    servo_ids: Vec<u8>,
    config: feetech::FeetechConfig,
    cached_states: Vec<JointState>,
    cached_loads: Vec<f32>,
    pub motor_status: Vec<u8>,
    last_feedback_instant: Vec<Option<Instant>>,
    confirmed_enable_state: EnableState,
    tx_echo: Option<bool>,
    tx_echo_auto: bool,
    poll_cursor: usize,
    rx_buffer: Vec<u8>,
    pub effective_ranges: Vec<(u16, u16)>,
    cached_raw_steps: Vec<i32>,
    read_poll_count: u64,
    missed_poll_replies: u64,
    partial_enable_known: bool,
    joint_torque_enabled: Vec<Option<bool>>,
    abort_flag: Option<Arc<AtomicBool>>,
    pub last_enable_poll: Option<Instant>,
    gripper_index: Option<usize>,
    pub hw_angle_limits: Vec<Option<(u16, u16)>>,
    pub last_enable_confirmed: Vec<Option<Instant>>,
    pub missed_enable_polls: Vec<u8>,
    incomplete_chunk_idx: Option<usize>,
    incomplete_frame_started_at: Option<Instant>,
    partial_pos_rx: Vec<u8>,
    consecutive_missed_echoes: u32,
    warned_missed_echoes: bool,
}

impl<T: Transport> FeetechArmDriver<T> {
    pub fn new(transport: T, servo_ids: Vec<u8>) -> Self {
        let dof = servo_ids.len();
        let joints: Vec<feetech::FeetechJointCalib> = servo_ids
            .iter()
            .map(|&id| feetech::FeetechJointCalib {
                id,
                ..Default::default()
            })
            .collect();
        let effective_ranges = joints.iter().map(|j| j.effective_range_steps()).collect();
        Self {
            transport,
            servo_ids,
            config: feetech::FeetechConfig {
                joints,
                ..Default::default()
            },
            cached_states: vec![JointState::default(); dof],
            cached_loads: vec![0.0; dof],
            motor_status: vec![0; dof],
            last_feedback_instant: vec![None; dof],
            confirmed_enable_state: EnableState::Unknown,
            tx_echo: None,
            tx_echo_auto: false,
            poll_cursor: 0,
            rx_buffer: Vec::with_capacity(256),
            effective_ranges,
            cached_raw_steps: vec![0; dof],
            read_poll_count: 0,
            missed_poll_replies: 0,
            partial_enable_known: false,
            joint_torque_enabled: vec![None; dof],
            abort_flag: None,
            last_enable_poll: None,
            gripper_index: None,
            hw_angle_limits: vec![None; dof],
            last_enable_confirmed: vec![None; dof],
            missed_enable_polls: vec![0; dof],
            incomplete_chunk_idx: None,
            incomplete_frame_started_at: None,
            partial_pos_rx: Vec::with_capacity(64),
            consecutive_missed_echoes: 0,
            warned_missed_echoes: false,
        }
    }

    pub fn with_gripper_index(mut self, gripper_index: Option<usize>) -> Self {
        self.gripper_index = gripper_index;
        self
    }

    pub fn with_zero_offsets(mut self, offsets: Vec<u16>) -> Self {
        for (i, &offset) in offsets.iter().enumerate() {
            if i < self.config.joints.len() {
                self.config.joints[i].zero_step = offset;
            }
        }
        self
    }

    pub fn with_calibrated(mut self, calibrated: bool) -> Self {
        self.config.calibrated = calibrated;
        self
    }

    pub fn missed_poll_replies(&self) -> u64 {
        self.missed_poll_replies
    }

    pub fn config(&self) -> &feetech::FeetechConfig {
        &self.config
    }

    pub fn effective_ranges(&self) -> &[(u16, u16)] {
        &self.effective_ranges
    }

    pub fn with_config(mut self, cfg: feetech::FeetechConfig) -> Self {
        if cfg.tx_echo.is_some() {
            self.tx_echo = cfg.tx_echo;
            self.tx_echo_auto = false;
        }
        let old_ids = self.servo_ids.clone();
        let old_joints = self.config.joints.clone();
        self.config = cfg;
        if self.config.joints.is_empty() {
            self.config.joints = self
                .servo_ids
                .iter()
                .map(|&id| feetech::FeetechJointCalib {
                    id,
                    ..Default::default()
                })
                .collect();
        } else {
            self.servo_ids = self.config.joints.iter().map(|j| j.id).collect();
        }

        let dof = self.servo_ids.len();
        if self.servo_ids != old_ids || self.config.joints != old_joints {
            self.cached_states = vec![JointState::default(); dof];
            self.cached_loads = vec![0.0; dof];
            self.motor_status = vec![0; dof];
            self.last_feedback_instant = vec![None; dof];
            self.cached_raw_steps = vec![0; dof];
            self.last_enable_confirmed = vec![None; dof];
            self.missed_enable_polls = vec![0; dof];
            if self.servo_ids != old_ids {
                self.joint_torque_enabled = vec![None; dof];
                self.poll_cursor = 0;
                self.confirmed_enable_state = EnableState::Unknown;
                self.partial_enable_known = false;
                self.hw_angle_limits = vec![None; dof];
                self.invalidate_partial_pos_rx();
            } else if self.hw_angle_limits.len() != dof {
                self.hw_angle_limits.resize(dof, None);
            }
        }
        self.effective_ranges = self
            .config
            .joints
            .iter()
            .enumerate()
            .map(|(i, calib)| {
                let min_s = calib.range_min_steps
                    .or_else(|| self.hw_angle_limits.get(i).copied().flatten().map(|x| x.0))
                    .unwrap_or(0);
                let max_s = calib.range_max_steps
                    .or_else(|| self.hw_angle_limits.get(i).copied().flatten().map(|x| x.1))
                    .unwrap_or(4095);
                (min_s, max_s)
            })
            .collect();
        if self.effective_ranges.len() != dof {
            self.effective_ranges.resize(dof, (0, 4095));
        }
        self
    }

    pub fn with_tx_echo(mut self, tx_echo: Option<bool>) -> Self {
        self.tx_echo = tx_echo;
        self.tx_echo_auto = false;
        self
    }

    pub fn with_port_name(mut self, port: &str) -> Self {
        if port.starts_with("/dev/ttySC") && self.tx_echo.is_none() {
            self.tx_echo = Some(true);
            self.tx_echo_auto = true;
        }
        self
    }

    pub fn tx_echo(&self) -> Option<bool> {
        self.tx_echo
    }

    pub fn tx_echo_auto(&self) -> bool {
        self.tx_echo_auto
    }

    pub fn transport(&self) -> &T {
        &self.transport
    }

    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    pub fn rx_buffer(&self) -> &[u8] {
        &self.rx_buffer
    }

    #[doc(hidden)]
    pub fn confirmed_enable_state(&self) -> EnableState {
        self.confirmed_enable_state
    }

    #[doc(hidden)]
    pub fn set_confirmed_enable_state(&mut self, state: EnableState) {
        self.confirmed_enable_state = state;
    }

    #[doc(hidden)]
    pub fn set_tx_echo_state(&mut self, tx_echo: Option<bool>, auto: bool) {
        self.tx_echo = tx_echo;
        self.tx_echo_auto = auto;
    }

    #[doc(hidden)]
    pub fn read_poll_count(&self) -> u64 {
        self.read_poll_count
    }

    #[doc(hidden)]
    pub fn set_read_poll_count(&mut self, count: u64) {
        self.read_poll_count = count;
    }

    #[doc(hidden)]
    pub fn incomplete_chunk_idx(&self) -> Option<usize> {
        self.incomplete_chunk_idx
    }

    #[doc(hidden)]
    pub fn set_incomplete_chunk_idx(&mut self, idx: Option<usize>) {
        self.incomplete_chunk_idx = idx;
    }

    #[doc(hidden)]
    pub fn incomplete_frame_started_at(&self) -> Option<Instant> {
        self.incomplete_frame_started_at
    }

    #[doc(hidden)]
    pub fn set_incomplete_frame_started_at(&mut self, instant: Option<Instant>) {
        self.incomplete_frame_started_at = instant;
    }

    #[doc(hidden)]
    pub fn partial_pos_rx(&self) -> &[u8] {
        &self.partial_pos_rx
    }

    #[doc(hidden)]
    pub fn set_partial_pos_rx(&mut self, bytes: &[u8]) {
        self.partial_pos_rx = bytes.to_vec();
    }

    pub(crate) fn invalidate_partial_pos_rx(&mut self) {
        self.partial_pos_rx.clear();
        self.incomplete_chunk_idx = None;
        self.incomplete_frame_started_at = None;
    }

    fn is_aborted(&self) -> bool {
        self.abort_flag
            .as_ref()
            .is_some_and(|f| f.load(Ordering::SeqCst))
    }

    fn drain_tx_echo(&mut self, tx: &[u8]) -> Result<bool, ArmDriverError> {
        if tx.is_empty() || self.tx_echo == Some(false) {
            return Ok(false);
        }

        let mut temp = [0u8; 1];
        let start = Instant::now();
        const DRAIN_TIMEOUT: Duration = Duration::from_millis(5);

        let resync_rx = |rx: &mut Vec<u8>| {
            let mut i = 0;
            while i < rx.len() {
                if rx[i] != 0xFF {
                    let next = rx[i..].iter().position(|&b| b == 0xFF).map(|p| i + p).unwrap_or(rx.len());
                    i = next;
                    if i >= rx.len() {
                        break;
                    }
                }
                if tx.starts_with(&[0xFF, 0xFF])
                    && tx.get(2) != Some(&0xFF)
                    && rx[i..].len() >= 3
                    && rx[i..i + 3] == [0xFF, 0xFF, 0xFF]
                {
                    i += 1;
                    continue;
                }
                let slice = &rx[i..];
                let is_echo = slice.windows(tx.len()).any(|w| w == tx) || tx.starts_with(slice);
                if !is_echo {
                    match feetech::parse_response(slice) {
                        Err(feetech::FeetechError::IncompleteFrame { .. }) | Ok(_) => break,
                        Err(_) => {
                            i += 1;
                            continue;
                        }
                    }
                }
                break;
            }
            if i > 0 {
                rx.drain(..i.min(rx.len()));
            }
        };

        let auto_detecting = self.tx_echo.is_none() || (self.tx_echo == Some(true) && self.tx_echo_auto);

        while !self.rx_buffer.windows(tx.len()).any(|w| w == tx)
            && start.elapsed() < DRAIN_TIMEOUT
            && self.rx_buffer.len() < tx.len() + 64
        {
            match self.transport.read(&mut temp) {
                Ok(n) if n > 0 => {
                    self.rx_buffer.extend_from_slice(&temp[..n]);
                }
                _ => {
                    std::thread::yield_now();
                }
            }

            resync_rx(&mut self.rx_buffer);

            if let Ok((pkt, consumed)) = feetech::parse_response(&self.rx_buffer) {
                if &self.rx_buffer[..consumed] != tx
                    && pkt.id != feetech::BROADCAST_ID
                    && !self.rx_buffer.windows(tx.len()).any(|w| w == tx)
                {
                    if auto_detecting {
                        tracing::warn!("Auto-detected tx_echo on /dev/ttySC* received valid non-echo response; disabling tx_echo");
                        self.tx_echo = Some(false);
                        self.tx_echo_auto = false;
                    }
                    break;
                }
            }
        }

        resync_rx(&mut self.rx_buffer);

        let mut drained = false;
        if let Some(pos) = self.rx_buffer.windows(tx.len()).position(|w| w == tx) {
            self.rx_buffer.drain(..pos + tx.len());
            drained = true;
            if self.tx_echo.is_none() || self.tx_echo_auto {
                self.tx_echo = Some(true);
                self.tx_echo_auto = false;
            }
        } else if auto_detecting {
            let has_partial_echo = (2..tx.len()).any(|k| self.rx_buffer.ends_with(&tx[..k]))
                || (self.rx_buffer.len() < 2 && tx.starts_with(&self.rx_buffer));
            let has_valid_non_echo = matches!(feetech::parse_response(&self.rx_buffer), Ok((pkt, c)) if &self.rx_buffer[..c] != tx && pkt.id != feetech::BROADCAST_ID);
            if self.tx_echo == Some(true) && self.tx_echo_auto && !has_partial_echo {
                tracing::warn!("Auto-detected tx_echo=true on /dev/ttySC* but no echo received within timeout; disabling tx_echo");
                self.tx_echo = Some(false);
                self.tx_echo_auto = false;
            } else if self.tx_echo.is_none() && has_valid_non_echo {
                self.tx_echo = Some(false);
            }
        }

        if self.tx_echo == Some(true) && !self.tx_echo_auto {
            if drained {
                self.consecutive_missed_echoes = 0;
            } else {
                self.consecutive_missed_echoes = self.consecutive_missed_echoes.saturating_add(1);
                if self.consecutive_missed_echoes >= 10 && !self.warned_missed_echoes {
                    tracing::warn!(
                        "Feetech driver has missed {} consecutive TX echoes with explicit tx_echo=true",
                        self.consecutive_missed_echoes
                    );
                    self.warned_missed_echoes = true;
                }
            }
        }

        Ok(drained)
    }

    pub fn read_register_u8_direct(&mut self, id: u8, reg: u8) -> Result<u8, ArmDriverError> {
        self.invalidate_partial_pos_rx();
        self.transport.clear_input_buffer();
        self.rx_buffer.clear();
        let req = feetech::build_read(id, reg, 1);
        self.transport.write_all(&req)?;
        self.transport.flush()?;
        let echo_drained = self.drain_tx_echo(&req)?;

        let mut temp = [0u8; 1];
        let start = Instant::now();
        const READ_TIMEOUT: Duration = Duration::from_millis(25);
        let mut can_skip_echo = !echo_drained && self.tx_echo != Some(false);

        loop {
            while !self.rx_buffer.is_empty() {
                match feetech::parse_response(&self.rx_buffer) {
                    Ok((pkt, consumed)) => {
                        let is_req_echo = can_skip_echo && consumed >= req.len() && self.rx_buffer[consumed - req.len()..consumed] == req[..];
                        self.rx_buffer.drain(..consumed);
                        if is_req_echo {
                            can_skip_echo = false;
                            self.tx_echo = Some(true);
                            self.tx_echo_auto = false;
                            continue;
                        }
                        if pkt.id == feetech::BROADCAST_ID {
                            continue;
                        }
                        if pkt.id == id && pkt.params.len() == 1 {
                            if self.tx_echo.is_none() {
                                self.tx_echo = Some(false);
                            }
                            return Ok(pkt.params[0]);
                        }
                    }
                    Err(feetech::FeetechError::IncompleteFrame { .. }) => {
                        break;
                    }
                    Err(_) => {
                        self.rx_buffer.remove(0);
                    }
                }
            }

            if start.elapsed() >= READ_TIMEOUT {
                break;
            }

            match self.transport.read(&mut temp) {
                Ok(n) if n > 0 => {
                    self.rx_buffer.extend_from_slice(&temp[..n]);
                }
                _ => {
                    std::thread::yield_now();
                }
            }
        }

        Err(ArmDriverError::Timeout(id))
    }

    pub fn read_register_u16_direct(&mut self, id: u8, reg: u8) -> Result<u16, ArmDriverError> {
        self.invalidate_partial_pos_rx();
        self.transport.clear_input_buffer();
        self.rx_buffer.clear();
        let req = feetech::build_read(id, reg, 2);
        self.transport.write_all(&req)?;
        self.transport.flush()?;
        let echo_drained = self.drain_tx_echo(&req)?;

        let mut temp = [0u8; 1];
        let start = Instant::now();
        const READ_TIMEOUT: Duration = Duration::from_millis(25);
        let mut can_skip_echo = !echo_drained && self.tx_echo != Some(false);

        loop {
            while !self.rx_buffer.is_empty() {
                match feetech::parse_response(&self.rx_buffer) {
                    Ok((pkt, consumed)) => {
                        let is_req_echo = can_skip_echo && consumed >= req.len() && self.rx_buffer[consumed - req.len()..consumed] == req[..];
                        self.rx_buffer.drain(..consumed);
                        if is_req_echo {
                            can_skip_echo = false;
                            self.tx_echo = Some(true);
                            self.tx_echo_auto = false;
                            continue;
                        }
                        if pkt.id == feetech::BROADCAST_ID {
                            continue;
                        }
                        if pkt.id == id && pkt.params.len() == 2 {
                            if self.tx_echo.is_none() {
                                self.tx_echo = Some(false);
                            }
                            return Ok(u16::from_le_bytes([pkt.params[0], pkt.params[1]]));
                        }
                    }
                    Err(feetech::FeetechError::IncompleteFrame { .. }) => {
                        break;
                    }
                    Err(_) => {
                        self.rx_buffer.remove(0);
                    }
                }
            }

            if start.elapsed() >= READ_TIMEOUT {
                break;
            }

            match self.transport.read(&mut temp) {
                Ok(n) if n > 0 => {
                    self.rx_buffer.extend_from_slice(&temp[..n]);
                }
                _ => {
                    std::thread::yield_now();
                }
            }
        }

        Err(ArmDriverError::Timeout(id))
    }

    fn write_register_u8_direct(&mut self, id: u8, reg: u8, val: u8) -> Result<(), ArmDriverError> {
        if self.is_aborted() {
            return Err(ArmDriverError::HardwareFault("Aborted by emergency stop".into()));
        }
        self.write_register_u8_cleanup(id, reg, val)
    }

    fn write_register_u8_cleanup(&mut self, id: u8, reg: u8, val: u8) -> Result<(), ArmDriverError> {
        self.invalidate_partial_pos_rx();
        self.transport.clear_input_buffer();
        self.rx_buffer.clear();
        let pkt = feetech::build_write_u8(id, reg, val);
        self.transport.write_all(&pkt)?;
        self.transport.flush()?;
        self.drain_tx_echo(&pkt)?;
        Ok(())
    }

    fn write_register_u16_direct(&mut self, id: u8, reg: u8, val: u16) -> Result<(), ArmDriverError> {
        if self.is_aborted() {
            return Err(ArmDriverError::HardwareFault("Aborted by emergency stop".into()));
        }
        self.invalidate_partial_pos_rx();
        self.transport.clear_input_buffer();
        self.rx_buffer.clear();
        let pkt = feetech::build_write_u16(id, reg, val);
        self.transport.write_all(&pkt)?;
        self.transport.flush()?;
        self.drain_tx_echo(&pkt)?;
        Ok(())
    }

    /// Configure EEPROM and RAM registers for all servos based on `FeetechConfig`.
    pub fn configure_servos(&mut self) -> Result<(), ArmDriverError> {
        self.invalidate_partial_pos_rx();
        if self.is_aborted() {
            return Err(ArmDriverError::HardwareFault("Aborted by emergency stop".into()));
        }
        let mut all_disabled = true;
        let mut all_enabled = true;

        for joint_idx in 0..self.servo_ids.len() {
            let id = self.servo_ids[joint_idx];
            let calib = self.config.joints.get(joint_idx).cloned().unwrap_or(feetech::FeetechJointCalib {
                id,
                ..Default::default()
            });

            // 1. Verify model
            let model = self.read_register_u16_direct(id, feetech::REG_MODEL_L)?;
            if model != feetech::EXPECTED_MODEL_STS3215 {
                return Err(ArmDriverError::Protocol(format!(
                    "Servo ID {} reported model {}, expected STS3215 ({})",
                    id, model, feetech::EXPECTED_MODEL_STS3215
                )));
            }

            // 2. Read torque enable status
            let torque_en = self.read_register_u8_direct(id, feetech::REG_TORQUE_ENABLE)?;
            match torque_en {
                0 => {
                    if joint_idx < self.joint_torque_enabled.len() {
                        self.joint_torque_enabled[joint_idx] = Some(false);
                        self.last_enable_confirmed[joint_idx] = Some(Instant::now());
                        self.missed_enable_polls[joint_idx] = 0;
                    }
                    all_enabled = false;
                }
                1 => {
                    if joint_idx < self.joint_torque_enabled.len() {
                        self.joint_torque_enabled[joint_idx] = Some(true);
                        self.last_enable_confirmed[joint_idx] = Some(Instant::now());
                        self.missed_enable_polls[joint_idx] = 0;
                    }
                    all_disabled = false;
                }
                _ => {
                    if joint_idx < self.joint_torque_enabled.len() {
                        self.joint_torque_enabled[joint_idx] = None;
                    }
                    all_enabled = false;
                    all_disabled = false;
                }
            }

            // 3. Read current registers
            let cur_return_delay = self.read_register_u8_direct(id, feetech::REG_RETURN_DELAY)?;
            let cur_status_level = self.read_register_u8_direct(id, feetech::REG_STATUS_RETURN_LEVEL)?;
            let cur_min_angle = self.read_register_u16_direct(id, feetech::REG_MIN_ANGLE_LIMIT_L)?;
            let cur_max_angle = self.read_register_u16_direct(id, feetech::REG_MAX_ANGLE_LIMIT_L)?;
            let cur_p_coef = self.read_register_u8_direct(id, feetech::REG_P_COEF)?;
            let cur_d_coef = self.read_register_u8_direct(id, feetech::REG_D_COEF)?;
            let cur_i_coef = self.read_register_u8_direct(id, feetech::REG_I_COEF)?;
            let cur_max_torque = self.read_register_u16_direct(id, feetech::REG_MAX_TORQUE_L)?;
            let cur_prot_current = self.read_register_u16_direct(id, feetech::REG_PROTECTION_CURRENT_L)?;
            let cur_overload_torque = self.read_register_u8_direct(id, feetech::REG_OVERLOAD_TORQUE)?;
            let cur_operating_mode = self.read_register_u8_direct(id, feetech::REG_OPERATING_MODE)?;

            // Effective angle limits validation and storage
            if joint_idx < self.hw_angle_limits.len() {
                self.hw_angle_limits[joint_idx] = Some((cur_min_angle, cur_max_angle));
            }
            let eff_min = calib.range_min_steps.unwrap_or(cur_min_angle);
            let eff_max = calib.range_max_steps.unwrap_or(cur_max_angle);
            if eff_min >= eff_max || eff_max > 4095 {
                return Err(ArmDriverError::HardwareFault(format!(
                    "Servo ID {} has invalid angle limits min={} max={}",
                    id, eff_min, eff_max
                )));
            }
            if joint_idx < self.effective_ranges.len() {
                self.effective_ranges[joint_idx] = (eff_min, eff_max);
            }

            // Expected homing offset verification
            if let Some(expected) = calib.expected_homing_offset_steps {
                let raw_offset = self.read_register_u16_direct(id, feetech::REG_HOMING_OFFSET_L)?;
                let decoded = feetech::decode_sign_magnitude(raw_offset, 0x0800);
                if decoded != expected {
                    return Err(ArmDriverError::HardwareFault(format!(
                        "Servo ID {} homing offset mismatch: expected {}, read {}",
                        id, expected, decoded
                    )));
                }
            }

            // Targets
            let target_return_delay = self.config.return_delay_raw;
            let target_status_level = 0u8;
            let target_p_coef = self.config.p_coef;
            let target_d_coef = self.config.d_coef;
            let target_i_coef = self.config.i_coef;
            let target_max_torque = calib.max_torque_raw.unwrap_or(self.config.max_torque_raw);
            let target_prot_current = calib.protection_current_raw.unwrap_or(self.config.protection_current_raw);
            let target_overload_torque = calib.overload_torque_raw.or(self.config.overload_torque_raw);
            let target_operating_mode = 0u8;

            let differs = cur_return_delay != target_return_delay
                || cur_status_level != target_status_level
                || calib.range_min_steps.is_some_and(|m| cur_min_angle != m)
                || calib.range_max_steps.is_some_and(|m| cur_max_angle != m)
                || cur_p_coef != target_p_coef
                || cur_d_coef != target_d_coef
                || cur_i_coef != target_i_coef
                || cur_max_torque != target_max_torque
                || cur_prot_current != target_prot_current
                || target_overload_torque.is_some_and(|ot| cur_overload_torque != ot)
                || cur_operating_mode != target_operating_mode;

            if differs {
                if torque_en != 0 {
                    return Err(ArmDriverError::HardwareFault(
                        "cannot configure EEPROM while torque is enabled".to_string(),
                    ));
                }

                let mut changed_registers: Vec<(u8, u8, u16)> = Vec::new();

                let write_res = (|| -> Result<(), ArmDriverError> {
                    // Unlock EEPROM inside closure so REG_LOCK=1 always runs
                    self.write_register_u8_direct(id, feetech::REG_LOCK, 0)?;
                    std::thread::sleep(Duration::from_millis(2));

                    if cur_return_delay != target_return_delay {
                        self.write_register_u8_direct(id, feetech::REG_RETURN_DELAY, target_return_delay)?;
                        std::thread::sleep(Duration::from_millis(2));
                        changed_registers.push((feetech::REG_RETURN_DELAY, 1, target_return_delay as u16));
                    }
                    if cur_status_level != target_status_level {
                        self.write_register_u8_direct(id, feetech::REG_STATUS_RETURN_LEVEL, target_status_level)?;
                        std::thread::sleep(Duration::from_millis(2));
                        changed_registers.push((feetech::REG_STATUS_RETURN_LEVEL, 1, target_status_level as u16));
                    }
                    if let Some(target_min) = calib.range_min_steps {
                        if cur_min_angle != target_min {
                            self.write_register_u16_direct(id, feetech::REG_MIN_ANGLE_LIMIT_L, target_min)?;
                            std::thread::sleep(Duration::from_millis(2));
                            changed_registers.push((feetech::REG_MIN_ANGLE_LIMIT_L, 2, target_min));
                        }
                    }
                    if let Some(target_max) = calib.range_max_steps {
                        if cur_max_angle != target_max {
                            self.write_register_u16_direct(id, feetech::REG_MAX_ANGLE_LIMIT_L, target_max)?;
                            std::thread::sleep(Duration::from_millis(2));
                            changed_registers.push((feetech::REG_MAX_ANGLE_LIMIT_L, 2, target_max));
                        }
                    }
                    if cur_p_coef != target_p_coef {
                        self.write_register_u8_direct(id, feetech::REG_P_COEF, target_p_coef)?;
                        std::thread::sleep(Duration::from_millis(2));
                        changed_registers.push((feetech::REG_P_COEF, 1, target_p_coef as u16));
                    }
                    if cur_d_coef != target_d_coef {
                        self.write_register_u8_direct(id, feetech::REG_D_COEF, target_d_coef)?;
                        std::thread::sleep(Duration::from_millis(2));
                        changed_registers.push((feetech::REG_D_COEF, 1, target_d_coef as u16));
                    }
                    if cur_i_coef != target_i_coef {
                        self.write_register_u8_direct(id, feetech::REG_I_COEF, target_i_coef)?;
                        std::thread::sleep(Duration::from_millis(2));
                        changed_registers.push((feetech::REG_I_COEF, 1, target_i_coef as u16));
                    }
                    if cur_max_torque != target_max_torque {
                        self.write_register_u16_direct(id, feetech::REG_MAX_TORQUE_L, target_max_torque)?;
                        std::thread::sleep(Duration::from_millis(2));
                        changed_registers.push((feetech::REG_MAX_TORQUE_L, 2, target_max_torque));
                    }
                    if cur_prot_current != target_prot_current {
                        self.write_register_u16_direct(id, feetech::REG_PROTECTION_CURRENT_L, target_prot_current)?;
                        std::thread::sleep(Duration::from_millis(2));
                        changed_registers.push((feetech::REG_PROTECTION_CURRENT_L, 2, target_prot_current));
                    }
                    if let Some(target_overload) = target_overload_torque {
                        if cur_overload_torque != target_overload {
                            self.write_register_u8_direct(id, feetech::REG_OVERLOAD_TORQUE, target_overload)?;
                            std::thread::sleep(Duration::from_millis(2));
                            changed_registers.push((feetech::REG_OVERLOAD_TORQUE, 1, target_overload as u16));
                        }
                    }
                    if cur_operating_mode != target_operating_mode {
                        self.write_register_u8_direct(id, feetech::REG_OPERATING_MODE, target_operating_mode)?;
                        std::thread::sleep(Duration::from_millis(2));
                        changed_registers.push((feetech::REG_OPERATING_MODE, 1, target_operating_mode as u16));
                    }

                    // Verify readback of all changed registers before lock
                    for &(reg, size, expected_val) in &changed_registers {
                        if size == 1 {
                            let read_val = self.read_register_u8_direct(id, reg)?;
                            if (read_val as u16) != expected_val {
                                return Err(ArmDriverError::HardwareFault(format!(
                                    "EEPROM readback verify failed for servo ID {} reg 0x{:02X}: expected {}, got {}",
                                    id, reg, expected_val, read_val
                                )));
                            }
                        } else {
                            let read_val = self.read_register_u16_direct(id, reg)?;
                            if read_val != expected_val {
                                return Err(ArmDriverError::HardwareFault(format!(
                                    "EEPROM readback verify failed for servo ID {} reg 0x{:02X}: expected {}, got {}",
                                    id, reg, expected_val, read_val
                                )));
                            }
                        }
                    }

                    Ok(())
                })();

                // ALWAYS lock EEPROM before returning
                let lock_res = self.write_register_u8_cleanup(id, feetech::REG_LOCK, 1);
                if let Err(e) = write_res {
                    if let Err(le) = lock_res {
                        tracing::error!("Failed to relock EEPROM for servo {id} during error recovery: {le}");
                    }
                    return Err(e);
                }
                lock_res?;
                if cur_max_torque != target_max_torque {
                    self.write_register_u16_direct(id, feetech::REG_TORQUE_LIMIT_L, target_max_torque)?;
                }
            }
        }

        if !self.servo_ids.is_empty() {
            if all_disabled {
                self.confirmed_enable_state = EnableState::Disabled;
                self.partial_enable_known = false;
            } else if all_enabled {
                self.confirmed_enable_state = EnableState::Enabled;
                self.partial_enable_known = false;
            } else {
                self.confirmed_enable_state = EnableState::Unknown;
                self.partial_enable_known = true;
            }
        }

        Ok(())
    }
    fn is_benign_gripper_overload(&self, joint_idx: usize, status: u8) -> bool {
        self.gripper_index == Some(joint_idx) && status == 0x20
    }

    fn decode_position_frames_from(
        &mut self,
        rx: &mut Vec<u8>,
        expected_ids: &std::collections::HashSet<u8>,
        replied_ids: &mut std::collections::HashSet<u8>,
        read_time: Instant,
    ) {
        let default_calib = feetech::FeetechJointCalib::default();
        while !rx.is_empty() {
            match feetech::parse_response(rx) {
                Ok((pkt, consumed)) => {
                    rx.drain(..consumed);
                    if pkt.id == feetech::BROADCAST_ID {
                        continue;
                    }
                    if expected_ids.contains(&pkt.id) && !replied_ids.contains(&pkt.id) {
                        if let Some(joint_idx) = self.servo_ids.iter().position(|&id| id == pkt.id) {
                            if pkt.params.len() == 8 {
                                let pos_raw = u16::from_le_bytes([pkt.params[0], pkt.params[1]]);
                                let speed_raw = u16::from_le_bytes([pkt.params[2], pkt.params[3]]);
                                let load_raw = u16::from_le_bytes([pkt.params[4], pkt.params[5]]);
                                let temp_raw = pkt.params[7];

                                let calib = self.config.joints.get(joint_idx).unwrap_or(&default_calib);
                                let raw_vel = feetech::raw_speed_to_rad_s(speed_raw);
                                let velocity = if calib.drive_mode_invert { -raw_vel } else { raw_vel };
                                let raw_load = feetech::raw_load_to_fraction(load_raw);
                                let load = if calib.drive_mode_invert { -raw_load } else { raw_load };
                                let pos_steps = feetech::decode_present_position(pos_raw);
                                self.cached_raw_steps[joint_idx] = pos_steps;
                                self.cached_states[joint_idx] = JointState {
                                    position: calib.steps_to_rad(pos_steps),
                                    velocity,
                                    torque: 0.0,
                                    temperature: feetech::raw_to_temperature(temp_raw),
                                };
                                self.cached_loads[joint_idx] = load;
                                self.motor_status[joint_idx] = pkt.error;
                                self.last_feedback_instant[joint_idx] = Some(read_time);

                                replied_ids.insert(pkt.id);
                            }
                        }
                    }
                }
                Err(feetech::FeetechError::IncompleteFrame { .. }) => {
                    break;
                }
                Err(_) => {
                    rx.remove(0);
                }
            }
        }
    }

    fn decode_position_frames(
        &mut self,
        expected_ids: &std::collections::HashSet<u8>,
        replied_ids: &mut std::collections::HashSet<u8>,
        read_time: Instant,
    ) {
        let mut rx = std::mem::take(&mut self.rx_buffer);
        self.decode_position_frames_from(&mut rx, expected_ids, replied_ids, read_time);
        self.rx_buffer = rx;
    }

    fn decode_partial_position_frames(
        &mut self,
        expected_ids: &std::collections::HashSet<u8>,
        replied_ids: &mut std::collections::HashSet<u8>,
        read_time: Instant,
    ) {
        let mut rx = std::mem::take(&mut self.partial_pos_rx);
        self.decode_position_frames_from(&mut rx, expected_ids, replied_ids, read_time);
        self.partial_pos_rx = rx;
    }
}

impl<T: Transport> ArmDriver for FeetechArmDriver<T> {
    fn position_limits(&self, joint: usize) -> Option<(f32, f32)> {
        self.effective_ranges.get(joint).map(|&(min_s, max_s)| {
            let calib = self.config.joint_calib(joint);
            let r0 = calib.steps_to_rad(min_s as i32);
            let r1 = calib.steps_to_rad(max_s as i32);
            (r0.min(r1), r0.max(r1))
        })
    }

    fn set_gripper_index(&mut self, index: Option<usize>) {
        self.gripper_index = index;
    }

    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            velocity_feedback: true,
            torque_feedback: false,
            temperature_feedback: true,
            torque_feedforward: false,
            enable_feedback: true,
            load_feedback: true,
        }
    }

    fn enable_state(&self) -> EnableState {
        self.confirmed_enable_state
    }

    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        self.last_feedback_instant.iter().map(|t| t.map(|t| t.elapsed())).collect()
    }

    fn is_feedback_valid(&self) -> bool {
        if self.last_feedback_instant.is_empty() {
            return false;
        }
        const FRESHNESS_LIMIT: Duration = Duration::from_millis(250);
        self.last_feedback_instant
            .iter()
            .all(|opt| opt.is_some_and(|instant| instant.elapsed() <= FRESHNESS_LIMIT))
    }

    fn is_enabled(&self) -> bool {
        self.confirmed_enable_state == EnableState::Enabled && self.is_feedback_valid()
    }

    fn motor_status(&self) -> Option<Vec<u8>> {
        Some(self.motor_status.clone())
    }

    fn joint_loads(&self) -> Option<Vec<f32>> {
        Some(self.cached_loads.clone())
    }

    fn decode_motor_error(&self, code: u8) -> Option<String> {
        feetech::feetech_error_name(code)
    }

    fn check_hardware_status(&self) -> Result<(), ArmDriverError> {
        for (i, &status) in self.motor_status.iter().enumerate() {
            if status != 0 {
                if self.is_benign_gripper_overload(i, status) {
                    continue;
                }
                let id = self.servo_ids.get(i).copied().unwrap_or(0);
                let desc = feetech::feetech_error_name(status)
                    .unwrap_or_else(|| format!("0x{:02X}", status));
                return Err(ArmDriverError::HardwareFault(format!(
                    "Feetech joint {} (ID {}) fault 0x{:02X}: {}",
                    i, id, status, desc
                )));
            }
        }
        Ok(())
    }

    fn clear_fault_disables_joints(&self) -> Vec<usize> {
        self.motor_status
            .iter()
            .enumerate()
            .filter(|(i, &s)| s != 0 && !self.is_benign_gripper_overload(*i, s))
            .map(|(i, _)| i)
            .collect()
    }

    fn has_known_partial_enable(&self) -> bool {
        self.partial_enable_known
    }

    fn has_any_joint_enabled(&self) -> bool {
        self.confirmed_enable_state == EnableState::Enabled
            || self.partial_enable_known
            || self.joint_torque_enabled.contains(&Some(true))
    }

    fn set_abort_flag(&mut self, flag: Arc<AtomicBool>) {
        self.abort_flag = Some(flag);
    }

    fn refresh_enable_state(&mut self) -> Result<EnableState, ArmDriverError> {
        if self.servo_ids.is_empty() {
            return Ok(self.confirmed_enable_state);
        }
        let mut count_0 = 0;
        let mut count_1 = 0;
        let mut read_all_valid = true;
        let ids = self.servo_ids.clone();
        for (idx, &id) in ids.iter().enumerate() {
            match self.read_register_u8_direct(id, feetech::REG_TORQUE_ENABLE) {
                Ok(0) => {
                    self.joint_torque_enabled[idx] = Some(false);
                    self.last_enable_confirmed[idx] = Some(Instant::now());
                    self.missed_enable_polls[idx] = 0;
                    count_0 += 1;
                }
                Ok(1) => {
                    self.joint_torque_enabled[idx] = Some(true);
                    self.last_enable_confirmed[idx] = Some(Instant::now());
                    self.missed_enable_polls[idx] = 0;
                    count_1 += 1;
                }
                _ => {
                    self.joint_torque_enabled[idx] = None;
                    read_all_valid = false;
                }
            }
        }

        if read_all_valid && count_0 == ids.len() {
            self.confirmed_enable_state = EnableState::Disabled;
            self.partial_enable_known = false;
        } else if read_all_valid && count_1 == ids.len() {
            self.confirmed_enable_state = EnableState::Enabled;
            self.partial_enable_known = false;
        } else if read_all_valid && count_0 > 0 && count_1 > 0 && count_0 + count_1 == ids.len() {
            self.confirmed_enable_state = EnableState::Unknown;
            self.partial_enable_known = true;
        } else {
            self.confirmed_enable_state = EnableState::Unknown;
            self.partial_enable_known = false;
        }

        Ok(self.confirmed_enable_state)
    }

    fn clear_fault(&mut self) -> Result<(), ArmDriverError> {
        let faulted_ids: Vec<u8> = self
            .motor_status
            .iter()
            .enumerate()
            .filter(|(i, &s)| s != 0 && !self.is_benign_gripper_overload(*i, s))
            .filter_map(|(i, _)| self.servo_ids.get(i).copied())
            .collect();

        let mut write_err = None;
        for &id in &faulted_ids {
            let pkt = feetech::build_set_torque(id, false);
            if let Err(e) = self.transport.write_all(&pkt) {
                write_err = Some(e.into());
            }
            let _ = self.transport.flush();
            let _ = self.drain_tx_echo(&pkt);
        }

        if let Some(e) = write_err {
            self.confirmed_enable_state = EnableState::Unknown;
            self.partial_enable_known = false;
            return Err(e);
        }

        let mut read_errors = Vec::new();
        let mut count_0 = 0;
        let mut count_1 = 0;
        let ids = self.servo_ids.clone();
        for (idx, &id) in ids.iter().enumerate() {
            match self.read_register_u8_direct(id, feetech::REG_TORQUE_ENABLE) {
                Ok(0) => {
                    count_0 += 1;
                    self.joint_torque_enabled[idx] = Some(false);
                    self.last_enable_confirmed[idx] = Some(Instant::now());
                    self.missed_enable_polls[idx] = 0;
                }
                Ok(1) => {
                    count_1 += 1;
                    self.joint_torque_enabled[idx] = Some(true);
                    self.last_enable_confirmed[idx] = Some(Instant::now());
                    self.missed_enable_polls[idx] = 0;
                }
                Ok(v) => {
                    self.joint_torque_enabled[idx] = None;
                    read_errors.push(format!("servo {} returned unexpected value {}", id, v));
                }
                Err(e) => {
                    self.joint_torque_enabled[idx] = None;
                    read_errors.push(format!("servo {} read failed: {}", id, e));
                }
            }
        }

        if !read_errors.is_empty() {
            self.confirmed_enable_state = EnableState::Unknown;
            self.partial_enable_known = false;
            return Err(ArmDriverError::HardwareFault(format!(
                "clear_fault readback error: {}",
                read_errors.join("; ")
            )));
        }

        if count_0 == self.servo_ids.len() {
            self.confirmed_enable_state = EnableState::Disabled;
            self.partial_enable_known = false;
        } else if count_1 == self.servo_ids.len() {
            self.confirmed_enable_state = EnableState::Enabled;
            self.partial_enable_known = false;
        } else if count_0 + count_1 == self.servo_ids.len() {
            self.confirmed_enable_state = EnableState::Unknown;
            self.partial_enable_known = true;
        } else {
            self.confirmed_enable_state = EnableState::Unknown;
            self.partial_enable_known = false;
        }

        Ok(())
    }

    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        self.invalidate_partial_pos_rx();
        let ids = self.servo_ids.clone();
        if enabled {
            self.check_hardware_status()?;
            let prev_joint_enabled = self.joint_torque_enabled.clone();
            if self.is_aborted() {
                return Err(ArmDriverError::HardwareFault("Aborted by emergency stop".into()));
            }
            if !self.config.calibrated {
                return Err(ArmDriverError::HardwareFault(
                    "Feetech arm is uncalibrated: calibrate servos and set feetech.calibrated=true".into(),
                ));
            }
            if !self.is_feedback_valid() {
                if self.last_feedback_instant.iter().any(Option::is_some) {
                    return Err(ArmDriverError::HardwareFault(
                        "Stale joint feedback (>250ms); refusing to enable torque with unverified/stale goal positions".into(),
                    ));
                }
                return Err(ArmDriverError::HardwareFault(
                    "No valid joint feedback received (<250ms); refusing to enable torque with unverified/stale goal positions".into(),
                ));
            }
            let default_calib = feetech::FeetechJointCalib::default();
            for (i, &raw) in self.cached_raw_steps.iter().enumerate() {
                let calib = self.config.joints.get(i).unwrap_or(&default_calib);
                let range = self.effective_ranges.get(i).copied().unwrap_or_else(|| calib.effective_range_steps());
                let pos_rad = calib.steps_to_rad(raw);
                calib.checked_rad_to_steps_in_range(pos_rad, range).map_err(|e| {
                    ArmDriverError::HardwareFault(format!(
                        "Joint {} (ID {}) raw position {} outside calibrated range: {}",
                        i, self.servo_ids[i], raw, e
                    ))
                })?;
            }

            // Write targets only for joints that are not already holding (Some(true))
            let mut positions_to_write = Vec::new();
            for (i, s) in self.cached_states.iter().enumerate() {
                if prev_joint_enabled.get(i).copied().flatten() == Some(true) {
                    continue;
                }
                let calib = self.config.joints.get(i).unwrap_or(&default_calib);
                let range = self.effective_ranges.get(i).copied().unwrap_or_else(|| calib.effective_range_steps());
                let raw_steps = calib.checked_rad_to_steps_in_range(s.position, range)
                    .map_err(|e| ArmDriverError::Protocol(e.to_string()))?;
                positions_to_write.push((self.servo_ids[i], raw_steps));
            }
            if !positions_to_write.is_empty() {
                let sync_pkt = feetech::build_sync_write_positions(&positions_to_write)
                    .map_err(|e| ArmDriverError::Protocol(e.to_string()))?;
                self.transport.write_all(&sync_pkt)?;
                self.transport.flush()?;
                self.drain_tx_echo(&sync_pkt)?;
            }

            let mut write_err = None;
            let mut written_indices = Vec::new();
            for (idx, &id) in ids.iter().enumerate() {
                if self.is_aborted() {
                    write_err = Some(ArmDriverError::HardwareFault("Aborted by emergency stop".into()));
                    break;
                }
                let pkt = feetech::build_set_torque(id, true);
                if let Err(e) = self.transport.write_all(&pkt) {
                    write_err = Some(e.into());
                    break;
                }
                if let Err(e) = self.transport.flush() {
                    write_err = Some(e.into());
                    break;
                }
                if let Err(e) = self.drain_tx_echo(&pkt) {
                    write_err = Some(e);
                    break;
                }
                written_indices.push(idx);
            }

            if self.is_aborted() {
                if !written_indices.is_empty() {
                    for &idx in &written_indices {
                        self.joint_torque_enabled[idx] = None;
                    }
                    self.partial_enable_known = true;
                } else {
                    self.partial_enable_known = false;
                }
                self.confirmed_enable_state = EnableState::Unknown;
                return Err(ArmDriverError::HardwareFault("Aborted by emergency stop".into()));
            }

            let mut all_enabled = write_err.is_none();
            let mut read_errors = Vec::new();
            let mut one_count = 0;

            for (idx, &id) in ids.iter().enumerate() {
                if self.is_aborted() {
                    for &w_idx in &written_indices {
                        if w_idx >= idx {
                            self.joint_torque_enabled[w_idx] = None;
                        }
                    }
                    self.partial_enable_known = true;
                    self.confirmed_enable_state = EnableState::Unknown;
                    return Err(ArmDriverError::HardwareFault("Aborted by emergency stop".into()));
                }
                match self.read_register_u8_direct(id, feetech::REG_TORQUE_ENABLE) {
                    Ok(1) => {
                        one_count += 1;
                        self.joint_torque_enabled[idx] = Some(true);
                        self.last_enable_confirmed[idx] = Some(Instant::now());
                        self.missed_enable_polls[idx] = 0;
                    }
                    Ok(0) => {
                        all_enabled = false;
                        self.joint_torque_enabled[idx] = Some(false);
                        self.last_enable_confirmed[idx] = Some(Instant::now());
                        self.missed_enable_polls[idx] = 0;
                        read_errors.push(format!("servo {} returned 0", id));
                    }
                    Ok(v) => {
                        all_enabled = false;
                        self.joint_torque_enabled[idx] = None;
                        read_errors.push(format!("servo {} returned unexpected value {}", id, v));
                    }
                    Err(e) => {
                        all_enabled = false;
                        self.joint_torque_enabled[idx] = None;
                        read_errors.push(format!("servo {} read failed: {}", id, e));
                    }
                }
            }

            if all_enabled && one_count == ids.len() {
                self.confirmed_enable_state = EnableState::Enabled;
                self.partial_enable_known = false;
                self.last_enable_poll = Some(Instant::now());
                Ok(())
            } else {
                let mut rb_write_err = false;
                for (i, &id) in ids.iter().enumerate() {
                    if prev_joint_enabled.get(i).copied().flatten() == Some(true) {
                        continue;
                    }
                    let pkt = feetech::build_set_torque(id, false);
                    let _ = self.transport.write_all(&pkt);
                    let _ = self.transport.flush();
                    let _ = self.drain_tx_echo(&pkt);

                    match self.read_register_u8_direct(id, feetech::REG_TORQUE_ENABLE) {
                        Ok(0) => {
                            self.joint_torque_enabled[i] = Some(false);
                            self.last_enable_confirmed[i] = Some(Instant::now());
                            self.missed_enable_polls[i] = 0;
                        }
                        _ => {
                            self.joint_torque_enabled[i] = None;
                            rb_write_err = true;
                        }
                    }
                }

                if !rb_write_err && write_err.is_none() && self.joint_torque_enabled.iter().all(|s| *s == Some(false)) {
                    self.confirmed_enable_state = EnableState::Disabled;
                    self.partial_enable_known = false;
                } else if !rb_write_err && write_err.is_none()
                    && self.joint_torque_enabled.iter().all(|s| s.is_some())
                    && self.joint_torque_enabled.contains(&Some(true))
                    && self.joint_torque_enabled.contains(&Some(false))
                {
                    self.confirmed_enable_state = EnableState::Unknown;
                    self.partial_enable_known = true;
                } else {
                    self.confirmed_enable_state = EnableState::Unknown;
                    self.partial_enable_known = false;
                }

                let err_msg = if let Some(e) = write_err {
                    format!("Torque enable write failed: {e}")
                } else {
                    format!("Torque enable readback failed: {}", read_errors.join("; "))
                };
                Err(ArmDriverError::HardwareFault(err_msg))
            }
        } else {
            let mut write_err = None;
            for &id in &ids {
                let pkt = feetech::build_set_torque(id, false);
                if let Err(e) = self.transport.write_all(&pkt) {
                    write_err = Some(e.into());
                }
                let _ = self.transport.flush();
                let _ = self.drain_tx_echo(&pkt);
            }

            let mut all_disabled = true;
            for (idx, &id) in ids.iter().enumerate() {
                match self.read_register_u8_direct(id, feetech::REG_TORQUE_ENABLE) {
                    Ok(0) => {
                        self.joint_torque_enabled[idx] = Some(false);
                        self.last_enable_confirmed[idx] = Some(Instant::now());
                        self.missed_enable_polls[idx] = 0;
                    }
                    Ok(1) => {
                        all_disabled = false;
                        self.joint_torque_enabled[idx] = Some(true);
                        self.last_enable_confirmed[idx] = Some(Instant::now());
                        self.missed_enable_polls[idx] = 0;
                    }
                    _ => {
                        all_disabled = false;
                        self.joint_torque_enabled[idx] = None;
                    }
                }
            }

            if all_disabled {
                self.confirmed_enable_state = EnableState::Disabled;
                self.partial_enable_known = false;
            } else {
                self.confirmed_enable_state = EnableState::Unknown;
                self.partial_enable_known = false;
            }

            if let Some(e) = write_err {
                Err(e)
            } else {
                Ok(())
            }
        }
    }

    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        if targets.len() != self.servo_ids.len() {
            return Err(ArmDriverError::JointCountMismatch {
                expected: self.servo_ids.len(),
                actual: targets.len(),
            });
        }

        let mut positions = Vec::with_capacity(targets.len());
        let default_calib = feetech::FeetechJointCalib::default();
        for (i, target) in targets.iter().enumerate() {
            let calib = self.config.joints.get(i).unwrap_or(&default_calib);
            let range = self.effective_ranges.get(i).copied().unwrap_or_else(|| calib.effective_range_steps());
            let raw_steps = calib.checked_rad_to_steps_in_range(target.position, range)
                .map_err(|e| ArmDriverError::Protocol(e.to_string()))?;
            positions.push((self.servo_ids[i], raw_steps));
        }

        let sync_pkt = feetech::build_sync_write_positions(&positions)
            .map_err(|e| ArmDriverError::Protocol(e.to_string()))?;

        self.transport.write_all(&sync_pkt)?;
        self.transport.flush()?;
        self.drain_tx_echo(&sync_pkt)?;
        if self.incomplete_chunk_idx.is_some() && !self.rx_buffer.is_empty() {
            self.partial_pos_rx.append(&mut self.rx_buffer);
        }
        Ok(())
    }

    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let dof = self.servo_ids.len();
        if dof == 0 {
            return Ok(Vec::new());
        }

        const MAX_SYNC_READ_SERVOS: usize = 3;
        const MAX_READ_DURATION: Duration = Duration::from_millis(8);
        let start_time = Instant::now();
        let mut temp_chunk = [0u8; 1];

        const INCOMPLETE_FRAME_TTL: Duration = Duration::from_millis(25);
        if let Some(inc_idx) = self.incomplete_chunk_idx.take() {
            let inc_ts = self.incomplete_frame_started_at.take().unwrap_or_else(Instant::now);
            if inc_ts.elapsed() >= INCOMPLETE_FRAME_TTL {
                self.invalidate_partial_pos_rx();
            } else if let Some(inc_chunk) = self.servo_ids.chunks(MAX_SYNC_READ_SERVOS).nth(inc_idx).map(|c| c.to_vec()) {
                let expected_ids: std::collections::HashSet<u8> = inc_chunk.iter().copied().collect();
                let mut replied_ids = std::collections::HashSet::new();
                self.decode_partial_position_frames(&expected_ids, &mut replied_ids, inc_ts);

                let reassembly_start = Instant::now();
                const REASSEMBLY_TIMEOUT: Duration = Duration::from_millis(2);
                let mut drain_buf = [0u8; 32];

                while replied_ids.len() < inc_chunk.len()
                    && reassembly_start.elapsed() < REASSEMBLY_TIMEOUT
                    && start_time.elapsed() < MAX_READ_DURATION
                    && inc_ts.elapsed() < INCOMPLETE_FRAME_TTL
                {
                    match self.transport.read(&mut drain_buf) {
                        Ok(n) if n > 0 => {
                            self.partial_pos_rx.extend_from_slice(&drain_buf[..n]);
                            self.decode_partial_position_frames(&expected_ids, &mut replied_ids, inc_ts);
                        }
                        _ => {
                            if inc_ts.elapsed() >= INCOMPLETE_FRAME_TTL {
                                break;
                            }
                            std::thread::yield_now();
                        }
                    }
                    if inc_ts.elapsed() >= INCOMPLETE_FRAME_TTL && replied_ids.len() < inc_chunk.len() {
                        if let Ok(n) = self.transport.read(&mut drain_buf) {
                            if n > 0 {
                                self.partial_pos_rx.extend_from_slice(&drain_buf[..n]);
                                self.decode_partial_position_frames(&expected_ids, &mut replied_ids, inc_ts);
                            }
                        }
                        break;
                    }
                }

                let all_replied = replied_ids.len() == inc_chunk.len();
                let inc_timed_out = inc_ts.elapsed() >= INCOMPLETE_FRAME_TTL;

                if !all_replied && !inc_timed_out {
                    if let Some(tail) = feetech::trailing_partial_frame_for_sync_read(&self.partial_pos_rx, &inc_chunk, 10) {
                        self.partial_pos_rx = tail.to_vec();
                        self.incomplete_chunk_idx = Some(inc_idx);
                        self.incomplete_frame_started_at = Some(inc_ts);
                    } else {
                        self.invalidate_partial_pos_rx();
                    }
                } else {
                    self.invalidate_partial_pos_rx();
                }
            } else {
                self.invalidate_partial_pos_rx();
            }
        }

        self.read_poll_count += 1;
        let did_enable_poll = self.incomplete_chunk_idx.is_none()
            && (self.confirmed_enable_state == EnableState::Enabled || self.partial_enable_known)
            && (self.read_poll_count.is_multiple_of(10) || self.last_enable_poll.is_none_or(|t| t.elapsed() >= Duration::from_millis(200)));
        if did_enable_poll {
            self.last_enable_poll = Some(Instant::now());
            self.transport.clear_input_buffer();
            self.rx_buffer.clear();
            if let Ok(sync_read) = feetech::build_sync_read(&self.servo_ids, feetech::REG_TORQUE_ENABLE, 1) {
                let _ = self.transport.write_all(&sync_read);
                let _ = self.transport.flush();
                let _ = self.drain_tx_echo(&sync_read);

                let mut temp_en = [0u8; 1];
                let start_en = Instant::now();
                let mut replies: std::collections::HashMap<u8, u8> = std::collections::HashMap::new();

                while replies.len() < self.servo_ids.len()
                    && start_en.elapsed() < Duration::from_millis(3)
                    && start_time.elapsed() < Duration::from_millis(10)
                {
                    if let Ok(n) = self.transport.read(&mut temp_en) {
                        if n > 0 {
                            self.rx_buffer.extend_from_slice(&temp_en[..n]);
                        }
                    }
                    while !self.rx_buffer.is_empty() {
                        match feetech::parse_response(&self.rx_buffer) {
                            Ok((pkt, consumed)) => {
                                self.rx_buffer.drain(..consumed);
                                if pkt.id != feetech::BROADCAST_ID && self.servo_ids.contains(&pkt.id) && pkt.params.len() == 1 {
                                    replies.insert(pkt.id, pkt.params[0]);
                                }
                            }
                            Err(feetech::FeetechError::IncompleteFrame { .. }) => break,
                            Err(_) => {
                                self.rx_buffer.remove(0);
                            }
                        }
                    }
                    if replies.len() < self.servo_ids.len() {
                        std::thread::yield_now();
                    }
                }

                let now = Instant::now();
                let prev_holding_lost = self.partial_enable_known && self.servo_ids.iter().enumerate().any(|(idx, id)| {
                    self.joint_torque_enabled.get(idx).copied().flatten() == Some(true)
                        && replies.get(id).copied() == Some(0)
                });

                for (idx, &id) in self.servo_ids.iter().enumerate() {
                    match replies.get(&id) {
                        Some(&0) => {
                            self.joint_torque_enabled[idx] = Some(false);
                            self.last_enable_confirmed[idx] = Some(now);
                            self.missed_enable_polls[idx] = 0;
                        }
                        Some(&1) => {
                            self.joint_torque_enabled[idx] = Some(true);
                            self.last_enable_confirmed[idx] = Some(now);
                            self.missed_enable_polls[idx] = 0;
                        }
                        Some(&_v) => {
                            self.joint_torque_enabled[idx] = None;
                            self.confirmed_enable_state = EnableState::Unknown;
                            self.partial_enable_known = false;
                        }
                        None => {
                            self.missed_enable_polls[idx] = self.missed_enable_polls[idx].saturating_add(1);
                            if self.missed_enable_polls[idx] >= 3
                                || self.last_enable_confirmed[idx].is_none_or(|t| now.duration_since(t) > Duration::from_millis(600))
                            {
                                self.joint_torque_enabled[idx] = None;
                                self.confirmed_enable_state = EnableState::Unknown;
                                self.partial_enable_known = false;
                            }
                        }
                    }
                }

                if replies.len() == self.servo_ids.len() && replies.values().all(|&v| v == 0) {
                    self.confirmed_enable_state = EnableState::Disabled;
                    self.partial_enable_known = false;
                } else if self.confirmed_enable_state == EnableState::Enabled && replies.values().any(|&v| v == 0) {
                    self.confirmed_enable_state = EnableState::Unknown;
                    self.partial_enable_known = false;
                } else if prev_holding_lost {
                    self.partial_enable_known = false;
                    self.confirmed_enable_state = EnableState::Unknown;
                }
            }
        }

        let chunks: Vec<Vec<u8>> = self.servo_ids.chunks(MAX_SYNC_READ_SERVOS).map(|c| c.to_vec()).collect();
        let num_chunks = chunks.len();
        let start_chunk = self.poll_cursor % num_chunks;

        if self.incomplete_chunk_idx.is_some() {
            return Ok(self.cached_states.clone());
        }

        'chunk_loop: for offset in 0..num_chunks {
            let chunk_idx = (start_chunk + offset) % num_chunks;
            let chunk = &chunks[chunk_idx];

            if start_time.elapsed() >= MAX_READ_DURATION {
                self.poll_cursor = chunk_idx;
                break 'chunk_loop;
            }

            self.transport.clear_input_buffer();
            self.rx_buffer.clear();
            let sync_read = feetech::build_sync_read(chunk, feetech::REG_PRESENT_POSITION_L, 8)
                .map_err(|e| ArmDriverError::Protocol(e.to_string()))?;

            self.transport.write_all(&sync_read)?;
            self.transport.flush()?;
            self.drain_tx_echo(&sync_read)?;

            let expected_ids: std::collections::HashSet<u8> = chunk.iter().copied().collect();
            let mut replied_ids = std::collections::HashSet::new();
            let chunk_start = Instant::now();
            const CHUNK_TIMEOUT: Duration = Duration::from_millis(4);
            let mut read_time = Instant::now();

            while replied_ids.len() < chunk.len() && chunk_start.elapsed() < CHUNK_TIMEOUT && start_time.elapsed() < MAX_READ_DURATION {
                self.decode_position_frames(&expected_ids, &mut replied_ids, read_time);
                if replied_ids.len() >= chunk.len() {
                    break;
                }

                match self.transport.read(&mut temp_chunk) {
                    Ok(n) if n > 0 => {
                        read_time = Instant::now();
                        self.rx_buffer.extend_from_slice(&temp_chunk[..n]);
                    }
                    _ => {}
                }

                self.decode_position_frames(&expected_ids, &mut replied_ids, read_time);

                if replied_ids.len() < chunk.len() {
                    std::thread::yield_now();
                }
            }

            if replied_ids.len() < chunk.len() {
                self.missed_poll_replies += 1;
                if start_time.elapsed() < MAX_READ_DURATION {
                    std::thread::sleep(Duration::from_micros(500));
                }
                let mut drain_buf = [0u8; 64];
                let mut drained_bytes = 0;
                while start_time.elapsed() < MAX_READ_DURATION && drained_bytes < 256 {
                    match self.transport.read(&mut drain_buf) {
                        Ok(n) if n > 0 => {
                            let drain_time = Instant::now();
                            self.rx_buffer.extend_from_slice(&drain_buf[..n]);
                            drained_bytes += n;
                            self.decode_position_frames(&expected_ids, &mut replied_ids, drain_time);
                        }
                        _ => break,
                    }
                }
                if self.rx_buffer.len() > 512 {
                    let trim = self.rx_buffer.len() - 256;
                    self.rx_buffer.drain(..trim);
                }
            }

            if replied_ids.len() < chunk.len() {
                if let Some(tail) = feetech::trailing_partial_frame_for_sync_read(&self.rx_buffer, chunk, 10) {
                    self.partial_pos_rx = tail.to_vec();
                    self.incomplete_chunk_idx = Some(chunk_idx);
                    self.incomplete_frame_started_at = Some(read_time);
                    self.rx_buffer.clear();
                    self.poll_cursor = (chunk_idx + 1) % num_chunks;
                    break 'chunk_loop;
                }
            }
            self.incomplete_chunk_idx = None;
            self.incomplete_frame_started_at = None;
            self.partial_pos_rx.clear();
            self.rx_buffer.clear();
            self.poll_cursor = (chunk_idx + 1) % num_chunks;
        }

        if self.rx_buffer.len() > 512 {
            let trim = self.rx_buffer.len() - 256;
            self.rx_buffer.drain(..trim);
        }

        Ok(self.cached_states.clone())
    }
}

// =========================================================================
// 2. FashionStar UartServo Arm Driver (Star Arm 102)
// =========================================================================

pub struct FashionStarArmDriver<T: Transport> {
    transport: T,
    servo_ids: Vec<u8>,
    zero_offsets: Vec<u16>,
    cached_states: Vec<JointState>,
    last_feedback_instant: Vec<Option<Instant>>,
    poll_cursor: usize,
    rx_buffer: Vec<u8>,
}

impl<T: Transport> FashionStarArmDriver<T> {
    pub fn new(transport: T, servo_ids: Vec<u8>) -> Self {
        let dof = servo_ids.len();
        Self {
            transport,
            servo_ids,
            zero_offsets: vec![fashionstar::DEFAULT_ZERO_OFFSET; dof],
            cached_states: vec![JointState::default(); dof],
            last_feedback_instant: vec![None; dof],
            poll_cursor: 0,
            rx_buffer: Vec::with_capacity(256),
        }
    }

    pub fn with_zero_offsets(mut self, offsets: Vec<u16>) -> Self {
        self.zero_offsets = offsets;
        self
    }

    pub fn transport(&self) -> &T {
        &self.transport
    }

    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    pub fn rx_buffer(&self) -> &[u8] {
        &self.rx_buffer
    }
}

impl<T: Transport> ArmDriver for FashionStarArmDriver<T> {
    fn capabilities(&self) -> DriverCapabilities { DriverCapabilities::default() }

    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        self.last_feedback_instant.iter().map(|t| t.map(|t| t.elapsed())).collect()
    }

    fn is_feedback_valid(&self) -> bool {
        if self.last_feedback_instant.is_empty() {
            return false;
        }
        const FRESHNESS_LIMIT: Duration = Duration::from_millis(250);
        self.last_feedback_instant
            .iter()
            .all(|opt| opt.is_some_and(|instant| instant.elapsed() <= FRESHNESS_LIMIT))
    }

    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        if !enabled {
            let mut last_err = None;
            for &id in &self.servo_ids {
                let pkt = fashionstar::build_set_torque(id, false);
                if let Err(e) = self.transport.write_all(&pkt) {
                    last_err = Some(e.into());
                }
            }
            if let Err(e) = self.transport.flush() {
                last_err = Some(e.into());
            }
            if let Some(e) = last_err {
                return Err(e);
            }
            Ok(())
        } else {
            for &id in &self.servo_ids {
                let pkt = fashionstar::build_set_torque(id, true);
                self.transport.write_all(&pkt)?;
            }
            self.transport.flush()?;
            Ok(())
        }
    }

    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        if targets.len() != self.servo_ids.len() {
            return Err(ArmDriverError::JointCountMismatch {
                expected: self.servo_ids.len(),
                actual: targets.len(),
            });
        }

        for (i, target) in targets.iter().enumerate() {
            let angle_tenth = fashionstar::rad_to_tenth_deg(target.position);
            let pkt = fashionstar::build_set_angle(self.servo_ids[i], angle_tenth, 20, 1000);
            self.transport.write_all(&pkt)?;
        }
        self.transport.flush()?;
        Ok(())
    }

    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let dof = self.servo_ids.len();
        if dof == 0 {
            return Ok(Vec::new());
        }

        let mut temp_chunk = [0u8; 1];
        let start_time = Instant::now();
        const MAX_READ_DURATION: Duration = Duration::from_millis(3);
        const MAX_TOTAL_BYTES: usize = 256;
        let mut total_bytes_read = 0;
        let start_cursor = self.poll_cursor;

        'poll: for offset in 0..dof {
            let joint_idx = (start_cursor + offset) % dof;
            let id = self.servo_ids[joint_idx];

            if start_time.elapsed() >= MAX_READ_DURATION || total_bytes_read >= MAX_TOTAL_BYTES {
                self.poll_cursor = joint_idx;
                break 'poll;
            }

            let query = fashionstar::build_query_angle(id);
            self.transport.write_all(&query)?;
            self.transport.flush()?;

            let mut attempts = 0;
            while attempts < 10 {
                if start_time.elapsed() >= MAX_READ_DURATION || total_bytes_read >= MAX_TOTAL_BYTES {
                    self.poll_cursor = (joint_idx + 1) % dof;
                    break 'poll;
                }

                match fashionstar::parse_packet(&self.rx_buffer) {
                    Ok((FashionStarPacket { header, id: pkt_id, cmd_id, content }, consumed)) => {
                        self.rx_buffer.drain(..consumed);
                        // Filter out local echo of request frame ([0x12, 0x4C]) on half-duplex RS485 bus
                        if header == fashionstar::FASHIONSTAR_REQUEST_HEADER {
                            continue;
                        }
                        if header == fashionstar::FASHIONSTAR_RESPONSE_HEADER
                            && pkt_id == id
                            && cmd_id == fashionstar::CMD_QUERY_SERVO_ANGLE
                            && content.len() >= 2
                        {
                            let tenth_deg = i16::from_le_bytes([content[0], content[1]]);
                            self.cached_states[joint_idx].position = fashionstar::tenth_deg_to_rad(tenth_deg);
                            self.last_feedback_instant[joint_idx] = Some(Instant::now());
                            break;
                        }
                    }
                    Err(fashionstar::FashionStarError::IncompleteFrame { .. }) => {
                        // Incomplete frame; read more bytes from transport
                    }
                    Err(_) => {
                        // Checksum mismatch or corruption: slide window by 1 byte to recover
                        if !self.rx_buffer.is_empty() {
                            self.rx_buffer.remove(0);
                        }
                    }
                }

                // Bound buffer size to prevent memory leaks / lockups
                if self.rx_buffer.len() > 1024 {
                    let trim = self.rx_buffer.len() - 256;
                    self.rx_buffer.drain(..trim);
                }

                match self.transport.read(&mut temp_chunk) {
                    Ok(n) if n > 0 => {
                        total_bytes_read += n;
                        self.rx_buffer.extend_from_slice(&temp_chunk[..n]);
                    }
                    _ => {
                        attempts += 1;
                        if attempts >= 3 {
                            break;
                        }
                    }
                }
            }
        }

        Ok(self.cached_states.clone())
    }
}

// =========================================================================
// 3. Damiao Brushless Arm Driver (reBot Arm B601-DM)
// =========================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DamiaoControlMode {
    PosVel,
    Mit,
}

pub struct DamiaoArmDriver<C: CanTransport> {
    can: C,
    motor_ids: Vec<u16>,
    master_id: u32,
    /// MIT / feedback mapping range per joint (must equal each motor's PMAX / VMAX / TMAX).
    limits: Vec<DamiaoLimits>,
    control_mode: DamiaoControlMode,
    default_vlim: f32,
    default_kp: f32,
    default_kd: f32,
    cached_states: Vec<JointState>,
    last_target_frames: Vec<Option<CanFrame>>,
    last_feedback_instant: Vec<Option<Instant>>,
    motor_status: Vec<u8>,
    is_enabled: bool,
    pending_rearm: bool,
    last_query_instant: Option<Instant>,
    last_enable_command: Option<Instant>,
    awaiting_rx_boundary: bool,
    is_enabling: bool,
    last_warn_sec: AtomicU64,
    last_refresh_instant: Option<Instant>,
    /// Confirmed control mode per joint; None = unconfirmed after a failed switch.
    joint_modes: Vec<Option<DamiaoControlMode>>,
    /// PMAX / VMAX / TMAX read back from the motor and equal to `limits[i]`.
    mit_limits_verified: Vec<bool>,
    /// Register replies seen by `read_joint_states` since the last clear.
    register_replies: Vec<damiao::RegisterReply>,
    /// Last mapping-range check (`check_mapping_ranges`); None = not run.
    range_check: Option<DriverRangeCheck>,
}

/// Time allowed for register replies (mode switch, range readback).
const REGISTER_REPLY_TIMEOUT: Duration = Duration::from_millis(50);
/// Time allowed per motor for the four register replies of the startup range check
/// (7 silent motors: 140 ms).
const RANGE_CHECK_REPLY_TIMEOUT: Duration = Duration::from_millis(20);

impl<C: CanTransport> DamiaoArmDriver<C> {
    pub fn new(can: C, motor_ids: Vec<u16>) -> Self {
        let dof = motor_ids.len();
        Self {
            can,
            motor_ids,
            master_id: 0,
            limits: vec![DamiaoLimits::default(); dof],
            control_mode: DamiaoControlMode::PosVel,
            default_vlim: 2.0,
            default_kp: 40.0,
            default_kd: 1.0,
            cached_states: vec![JointState::default(); dof],
            last_target_frames: vec![None; dof],
            last_feedback_instant: vec![None; dof],
            motor_status: vec![0; dof],
            is_enabled: false,
            pending_rearm: true,
            last_query_instant: None,
            last_enable_command: None,
            awaiting_rx_boundary: false,
            is_enabling: false,
            last_warn_sec: AtomicU64::new(0),
            last_refresh_instant: None,
            joint_modes: vec![Some(DamiaoControlMode::PosVel); dof],
            mit_limits_verified: vec![false; dof],
            register_replies: Vec::new(),
            range_check: None,
        }
    }

    pub fn with_master_id(mut self, master_id: u32) -> Self {
        self.master_id = master_id;
        self
    }

    pub fn with_control_mode(mut self, mode: DamiaoControlMode) -> Self {
        self.control_mode = mode;
        self.joint_modes.fill(Some(mode));
        self
    }

    /// One MIT / feedback mapping range for every joint. It must equal the motors'
    /// PMAX / VMAX / TMAX registers; gravity compensation reads them back and refuses
    /// to start otherwise.
    pub fn with_limits(mut self, limits: DamiaoLimits) -> Self {
        self.limits.fill(limits);
        self.mit_limits_verified.fill(false);
        self.range_check = None;
        self
    }

    /// Per-joint mapping ranges, one per motor ID in order.
    pub fn with_joint_limits(mut self, limits: Vec<DamiaoLimits>) -> Result<Self, ArmDriverError> {
        if limits.len() != self.motor_ids.len() {
            return Err(ArmDriverError::JointCountMismatch { expected: self.motor_ids.len(), actual: limits.len() });
        }
        self.limits = limits;
        self.mit_limits_verified.fill(false);
        self.range_check = None;
        Ok(self)
    }

    /// Read CTRL_MODE, PMAX, VMAX and TMAX (registers 10, 21-23) from every motor and
    /// compare the ranges with the configured ones. Register reads do not change the
    /// enable state or the targets, so this is safe with the motors disabled; it sends
    /// no enable frame. A mismatch makes `set_torque(true)` refuse to arm. A motor that
    /// does not answer leaves the check `Unverified`: the configured ranges are used
    /// and gravity compensation stays refused. A joint whose CTRL_MODE is not the base
    /// mode is marked unconfirmed, so the enable handshake writes the base mode first.
    pub fn check_mapping_ranges(&mut self) -> &DriverRangeCheck {
        const RIDS: [u8; 4] = [damiao::RID_CTRL_MODE, damiao::RID_PMAX, damiao::RID_VMAX, damiao::RID_TMAX];
        let start = Instant::now();
        let base = Self::mode_value(self.control_mode);
        let mut joints = Vec::with_capacity(self.motor_ids.len());
        for i in 0..self.motor_ids.len() {
            let id = self.motor_ids[i];
            self.register_replies.clear();
            let sent = RIDS.iter().all(|&rid| self.transmit_with_retry(&damiao::build_read_register_frame(id, rid)).is_ok());
            let _ = self.can.flush();
            let expected: Vec<(usize, bool, u8)> = RIDS.iter().map(|&rid| (i, false, rid)).collect();
            let replies = if sent { self.await_register_replies_for(&expected, RANGE_CHECK_REPLY_TIMEOUT) } else { vec![None; RIDS.len()] };
            let l = self.limits[i];
            let configured = [l.p_max, l.v_max, l.t_max];
            let ctrl_mode = replies[0].map(|r| r.as_u32());
            let reported = match (replies[1], replies[2], replies[3]) {
                (Some(p), Some(v), Some(t)) => Some([p.as_f32(), v.as_f32(), t.as_f32()]),
                _ => None,
            };
            let status = match reported {
                None => RangeCheckStatus::Unverified,
                Some(r) if Self::ranges_match(&r, &configured) => RangeCheckStatus::Passed,
                Some(_) => RangeCheckStatus::Mismatch,
            };
            self.mit_limits_verified[i] = status == RangeCheckStatus::Passed;
            if ctrl_mode.is_some_and(|m| m != base) {
                tracing::warn!("Damiao joint {} (CAN ID 0x{:X}) is in CTRL_MODE {}; the base mode {} is restored before enable",
                    i + 1, id, ctrl_mode.unwrap_or(0), base);
                self.joint_modes[i] = None;
            }
            joints.push(JointRangeReading { joint: i + 1, motor_id: id, status, ctrl_mode, reported, configured });
        }
        let fmt = |v: &[f32; 3]| format!("{:.3}/{:.3}/{:.3}", v[0], v[1], v[2]);
        let mismatched: Vec<String> = joints.iter().filter(|j| j.status == RangeCheckStatus::Mismatch)
            .map(|j| format!("J{} reports PMAX/VMAX/TMAX {} (configured {})", j.joint, fmt(&j.reported.unwrap_or_default()), fmt(&j.configured))).collect();
        let silent: Vec<usize> = joints.iter().filter(|j| j.status == RangeCheckStatus::Unverified).map(|j| j.joint).collect();
        let duration_ms = start.elapsed().as_millis() as u64;
        let (status, message) = if !mismatched.is_empty() {
            let m = format!("Damiao mapping range mismatch: {}; arming refused until damiao_ranges matches the motors", mismatched.join("; "));
            tracing::error!("{m}");
            (RangeCheckStatus::Mismatch, m)
        } else if !silent.is_empty() {
            let m = format!("Damiao joints {silent:?} did not answer the range read; using the configured ranges unverified");
            tracing::warn!("{m}");
            (RangeCheckStatus::Unverified, m)
        } else {
            let m = format!("Damiao mapping ranges of {} motors match the configuration", joints.len());
            tracing::info!("{m} ({duration_ms} ms)");
            (RangeCheckStatus::Passed, m)
        };
        self.range_check.insert(DriverRangeCheck { status, message, joints, duration_ms })
    }

    fn ranges_match(got: &[f32; 3], want: &[f32; 3]) -> bool {
        got.iter().zip(want).all(|(g, w)| g.is_finite() && (g - w).abs() <= 0.01 + w.abs() * 1e-3)
    }

    /// Mapping range used for each joint.
    pub fn joint_limits(&self) -> &[DamiaoLimits] {
        &self.limits
    }

    /// Confirmed control mode of each joint (None = unconfirmed).
    pub fn joint_control_modes(&self) -> &[Option<DamiaoControlMode>] {
        &self.joint_modes
    }

    fn mode_value(mode: DamiaoControlMode) -> u32 {
        match mode {
            DamiaoControlMode::Mit => damiao::CTRL_MODE_MIT,
            DamiaoControlMode::PosVel => damiao::CTRL_MODE_POS_VEL,
        }
    }

    fn vlim(&self) -> f32 { self.default_vlim.abs().clamp(0.1, 4.0) }

    fn pos_vel_frame(&self, i: usize, position: f32) -> CanFrame {
        damiao::build_pos_vel_frame(0x100, self.motor_ids[i], position, self.vlim())
    }

    fn mit_frame(&self, i: usize, target: &JointTarget) -> CanFrame {
        let torque = target.torque.filter(|t| t.is_finite()).unwrap_or(0.0);
        damiao::build_mit_frame(
            self.motor_ids[i],
            target.position,
            target.velocity.filter(|v| v.is_finite()).unwrap_or(0.0),
            target.kp.filter(|k| k.is_finite()).unwrap_or(self.default_kp),
            target.kd.filter(|k| k.is_finite()).unwrap_or(self.default_kd),
            torque,
            &self.limits[i],
        )
    }

    /// Frames for joint `i` in its confirmed mode; both kinds when the mode is unconfirmed.
    fn target_frames(&self, i: usize, target: &JointTarget) -> Vec<CanFrame> {
        match self.joint_modes[i] {
            Some(DamiaoControlMode::PosVel) => vec![self.pos_vel_frame(i, target.position)],
            Some(DamiaoControlMode::Mit) => vec![self.mit_frame(i, target)],
            None => vec![self.pos_vel_frame(i, target.position), self.mit_frame(i, target)],
        }
    }

    /// Poll feedback until every `(joint, write, rid)` has a reply or the timeout passes.
    fn await_register_replies(&mut self, expected: &[(usize, bool, u8)]) -> Vec<Option<damiao::RegisterReply>> {
        self.await_register_replies_for(expected, REGISTER_REPLY_TIMEOUT)
    }

    fn await_register_replies_for(&mut self, expected: &[(usize, bool, u8)], timeout: Duration) -> Vec<Option<damiao::RegisterReply>> {
        let start = Instant::now();
        loop {
            let _ = self.read_joint_states();
            let found: Vec<Option<damiao::RegisterReply>> = expected.iter().map(|&(i, write, rid)| {
                let id = self.motor_ids[i];
                self.register_replies.iter().rev().copied()
                    .find(|r| r.motor_id == id && r.write == write && r.rid == rid)
            }).collect();
            if found.iter().all(Option::is_some) || start.elapsed() >= timeout {
                return found;
            }
            std::thread::sleep(Duration::from_micros(500));
        }
    }

    /// Read PMAX / VMAX / TMAX of `joints` and require each to equal that joint's range:
    /// an MIT frame encoded with the wrong TMAX would scale every torque command.
    fn verify_mit_limits(&mut self, joints: &[usize]) -> Result<(), ArmDriverError> {
        const RIDS: [u8; 3] = [damiao::RID_PMAX, damiao::RID_VMAX, damiao::RID_TMAX];
        self.register_replies.clear();
        for &i in joints {
            for rid in RIDS {
                self.transmit_with_retry(&damiao::build_read_register_frame(self.motor_ids[i], rid))?;
            }
        }
        self.can.flush()?;
        let expected: Vec<(usize, bool, u8)> = joints.iter().flat_map(|&i| RIDS.map(|rid| (i, false, rid))).collect();
        let replies = self.await_register_replies(&expected);
        for (k, &i) in joints.iter().enumerate() {
            let want = [self.limits[i].p_max, self.limits[i].v_max, self.limits[i].t_max];
            let got: Vec<Option<f32>> = (0..3).map(|r| replies[k * 3 + r].map(|reply| reply.as_f32())).collect();
            if got.iter().any(Option::is_none) {
                return Err(ArmDriverError::HardwareFault(format!(
                    "Joint {} (CAN ID 0x{:X}) did not answer the PMAX/VMAX/TMAX register read", i + 1, self.motor_ids[i])));
            }
            let got: Vec<f32> = got.into_iter().flatten().collect();
            if !Self::ranges_match(&[got[0], got[1], got[2]], &want) {
                return Err(ArmDriverError::HardwareFault(format!(
                    "Joint {} (CAN ID 0x{:X}) reports PMAX {:.3} VMAX {:.3} TMAX {:.3}; the driver maps MIT frames with {:.3} / {:.3} / {:.3}",
                    i + 1, self.motor_ids[i], got[0], got[1], got[2], want[0], want[1], want[2])));
            }
            self.mit_limits_verified[i] = true;
        }
        Ok(())
    }

    /// Write CTRL_MODE for `joints` (each followed at once by `hold` in the new mode)
    /// and wait for the write acknowledgements. Returns the joints that were not confirmed.
    fn write_modes(&mut self, joints: &[usize], modes: &[DamiaoControlMode], hold: &[JointTarget], dual_hold: bool) -> Vec<usize> {
        self.register_replies.clear();
        let mut unsent = Vec::new();
        for &i in joints {
            let id = self.motor_ids[i];
            if self.transmit_with_retry(&damiao::build_set_control_mode_frame(id, Self::mode_value(modes[i]))).is_err() {
                unsent.push(i);
                continue;
            }
            // The hold frame directly follows the mode write on the bus, so the time a
            // joint spends in the new mode without a target is one frame (about 0.13 ms).
            let frames = if dual_hold {
                vec![self.pos_vel_frame(i, hold[i].position), self.mit_frame(i, &hold[i])]
            } else if modes[i] == DamiaoControlMode::Mit {
                vec![self.mit_frame(i, &hold[i])]
            } else {
                vec![self.pos_vel_frame(i, hold[i].position)]
            };
            for frame in frames {
                let _ = self.transmit_with_retry(&frame);
                self.last_target_frames[i] = Some(frame);
            }
        }
        let _ = self.can.flush();
        let sent: Vec<usize> = joints.iter().copied().filter(|i| !unsent.contains(i)).collect();
        let expected: Vec<(usize, bool, u8)> = sent.iter().map(|&i| (i, true, damiao::RID_CTRL_MODE)).collect();
        let replies = self.await_register_replies(&expected);
        let mut failed = unsent;
        for (k, &i) in sent.iter().enumerate() {
            if replies[k].is_some_and(|r| r.as_u32() == Self::mode_value(modes[i])) {
                self.joint_modes[i] = Some(modes[i]);
            } else {
                failed.push(i);
            }
        }
        failed
    }

    pub fn with_vlim(mut self, vlim: f32) -> Self {
        self.default_vlim = vlim;
        self
    }

    pub fn with_gains(mut self, kp: f32, kd: f32) -> Self {
        self.default_kp = kp;
        self.default_kd = kd;
        self
    }

    pub fn can_mut(&mut self) -> &mut C {
        &mut self.can
    }

    /// A command acknowledgement must be separated from buffered pre-command telemetry.
    fn drain_rx_before_command(&mut self) -> bool {
        let start = Instant::now();
        for _ in 0..128 {
            if start.elapsed() >= Duration::from_millis(5) { return false; }
            match self.can.receive() {
                Ok(_) => {},
                Err(TransportError::Timeout) | Err(TransportError::WouldBlock) => return true,
                Err(_) => return false,
            }
        }
        false
    }

    pub fn transmit_with_retry(&mut self, frame: &CanFrame) -> Result<(), ArmDriverError> {
        let mut retries = 0;
        loop {
            match self.can.transmit(frame) {
                Ok(()) => return Ok(()),
                Err(TransportError::WouldBlock) if retries < 3 => {
                    retries += 1;
                    std::thread::sleep(Duration::from_micros(100));
                }
                Err(e) => return Err(e.into()),
            }
        }
    }
}

impl<C: CanTransport> ArmDriver for DamiaoArmDriver<C> {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            velocity_feedback: true,
            torque_feedback: true,
            temperature_feedback: true,
            torque_feedforward: self.control_mode == DamiaoControlMode::Mit,
            enable_feedback: true,
            load_feedback: false,
        }
    }

    fn enable_state(&self) -> EnableState {
        if self.awaiting_rx_boundary {
            return EnableState::Unknown;
        }
        if self.last_feedback_instant.iter().any(|t| {
            self.last_enable_command.is_some_and(|command| t.is_none_or(|sample| sample < command))
        }) {
            return EnableState::Unknown;
        }
        if self.is_enabled {
            if !self.is_feedback_valid() {
                return EnableState::Unknown;
            }
            // Fresh feedback from every motor reporting state 0 is physical proof of
            // torque-off (e.g. motor power dip), whatever the cached enable flag says.
            if self.motor_status.iter().all(|&state| state == 0) {
                return EnableState::Disabled;
            }
            if self.motor_status.iter().all(|&state| state == 1) {
                EnableState::Enabled
            } else {
                EnableState::Unknown
            }
        } else {
            if self.motor_status.iter().all(|&state| state == 0) {
                EnableState::Disabled
            } else {
                EnableState::Unknown
            }
        }
    }

    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        self.last_feedback_instant.iter().map(|t| t.map(|t| t.elapsed())).collect()
    }


    fn max_command_velocity(&self, joint: usize) -> Option<f32> {
        if joint >= self.motor_ids.len() || self.control_mode != DamiaoControlMode::PosVel {
            return None;
        }
        Some(self.default_vlim.abs().clamp(0.1, 4.0))
    }

    fn is_enabled(&self) -> bool {
        self.is_enabled && self.enable_state() == EnableState::Enabled
    }

    fn motor_status(&self) -> Option<Vec<u8>> {
        Some(self.motor_status.clone())
    }

    fn set_torque_flag(&mut self, enabled: bool) {
        self.is_enabled = enabled;
        if !enabled {
            self.pending_rearm = true;
        }
    }

    fn set_pending_rearm(&mut self, pending: bool) {
        self.pending_rearm = pending;
    }

    fn supports_mit_mode_switch(&self) -> bool {
        self.control_mode == DamiaoControlMode::PosVel
    }

    fn set_joint_modes(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        let n = self.motor_ids.len();
        if modes.len() != n || hold.len() != n {
            return Err(ArmDriverError::JointCountMismatch { expected: n, actual: modes.len().min(hold.len()) });
        }
        if !self.supports_mit_mode_switch() {
            return Err(ArmDriverError::Protocol("joint mode switching needs the POS_VEL base mode".into()));
        }
        let target: Vec<DamiaoControlMode> = modes.iter().map(|m| match m {
            JointMode::Position => self.control_mode,
            JointMode::Mit => DamiaoControlMode::Mit,
        }).collect();
        let changing: Vec<usize> = (0..n).filter(|&i| self.joint_modes[i] != Some(target[i])).collect();
        if changing.is_empty() { return Ok(()); }
        let to_mit: Vec<usize> = changing.iter().copied()
            .filter(|&i| target[i] == DamiaoControlMode::Mit && !self.mit_limits_verified[i]).collect();
        if !to_mit.is_empty() { self.verify_mit_limits(&to_mit)?; }
        let previous = self.joint_modes.clone();
        let failed = self.write_modes(&changing, &target, hold, false);
        if failed.is_empty() {
            tracing::info!("Damiao control mode switched for joints {:?}", changing.iter().map(|i| i + 1).collect::<Vec<_>>());
            return Ok(());
        }
        // Not confirmed: switch every changed joint back to its previous mode, holding it
        // with both frame kinds meanwhile so it holds whichever mode it is actually in.
        for &i in &changing { self.joint_modes[i] = None; }
        let back: Vec<DamiaoControlMode> = (0..n).map(|i| previous[i].unwrap_or(self.control_mode)).collect();
        let unrecovered = self.write_modes(&changing, &back, hold, true);
        Err(ArmDriverError::HardwareFault(format!(
            "Control-mode switch not confirmed for joints {:?}; switched back{}",
            failed.iter().map(|i| i + 1).collect::<Vec<_>>(),
            if unrecovered.is_empty() { String::new() } else {
                format!(", joints {:?} still unconfirmed", unrecovered.iter().map(|i| i + 1).collect::<Vec<_>>())
            })))
    }

    fn joint_modes_uncertain(&self) -> bool {
        self.joint_modes.iter().any(Option::is_none)
    }

    fn range_check(&self) -> Option<DriverRangeCheck> { self.range_check.clone() }

    fn is_feedback_valid(&self) -> bool {
        if self.last_feedback_instant.is_empty() {
            return false;
        }
        let limit = Duration::from_millis(250);
        let mut all_valid = true;
        // Use monotonic process uptime to prevent clock-jump discontinuities on RTC synchronization
        static MONOTONIC_START: std::sync::LazyLock<Instant> = std::sync::LazyLock::new(Instant::now);
        let now_sec = MONOTONIC_START.elapsed().as_secs();
        let last_warn = self.last_warn_sec.load(Ordering::Relaxed);
        let should_warn = now_sec.saturating_sub(last_warn) >= 3;

        for (i, opt) in self.last_feedback_instant.iter().enumerate() {
            let ok = opt.is_some_and(|instant| instant.elapsed() <= limit);
            if !ok {
                all_valid = false;
                if should_warn && (self.is_enabled || self.is_enabling) {
                    tracing::warn!(
                        "Damiao Joint {} (CAN ID 0x{:X}) feedback stale or missing: elapsed={:?} > {:?}",
                        i,
                        self.motor_ids.get(i).copied().unwrap_or(0),
                        opt.map(|t| t.elapsed()),
                        limit
                    );
                }
            }
        }
        if !all_valid && should_warn && (self.is_enabled || self.is_enabling) {
            self.last_warn_sec.store(now_sec, Ordering::Relaxed);
        }
        all_valid
    }

    fn check_hardware_status(&self) -> Result<(), ArmDriverError> {
        for (i, &st) in self.motor_status.iter().enumerate() {
            let motor_id = self.motor_ids.get(i).copied().unwrap_or(0);
            if let Some(reason) = damiao_fault_name(st) {
                return Err(ArmDriverError::HardwareFault(format!(
                    "Joint {} (CAN ID 0x{:X}) reported fault: {}",
                    i, motor_id, reason
                )));
            }
        }
        Ok(())
    }

    fn clear_fault(&mut self) -> Result<(), ArmDriverError> {
        let mut err = None;
        for &id in &self.motor_ids {
            let frame = damiao::build_clear_faults_frame(id);
            if let Err(e) = self.can.transmit(&frame) {
                err = Some(e);
            }
        }
        if let Err(e) = self.can.flush() { err = Some(e); }
        if let Some(e) = err {
            Err(ArmDriverError::HardwareFault(e.to_string()))
        } else {
            Ok(())
        }
    }

    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        if !enabled {
            // Still send disable if draining times out: thermal protection must not
            // be blocked by traffic. Until a new empty-queue boundary, ignore ACKs.
            self.awaiting_rx_boundary = !self.drain_rx_before_command();
            let cmd_start = Instant::now();
            self.last_enable_command = Some(cmd_start);
            let mut last_err = None;
            for &id in &self.motor_ids {
                let frame = damiao::build_disable_frame(id);
                if let Err(e) = self.can.transmit(&frame) {
                    last_err = Some(e.into());
                }
            }
            if let Err(e) = self.can.flush() {
                last_err = Some(e.into());
            }
            if let Some(e) = last_err {
                // A failed or partial write cannot establish the physical power state.
                return Err(e);
            }
            self.is_enabled = false;
            self.pending_rearm = true;
            self.last_target_frames.fill(None);
            self.last_query_instant = Some(Instant::now());
            // Poll for responses confirming disable up to 25ms
            let poll_start = Instant::now();
            let mut check_count = 0;
            while poll_start.elapsed() < Duration::from_millis(25) && check_count < 10 {
                check_count += 1;
                let _ = self.read_joint_states();
                let all_fresh = !self.last_feedback_instant.is_empty()
                    && self.last_feedback_instant.iter().all(|opt| {
                        opt.is_some_and(|t| t >= cmd_start)
                    });
                if all_fresh && self.motor_status.iter().all(|&st| st == 0) {
                    tracing::info!("All Damiao motors confirmed disabled (state=0) in {:?}", poll_start.elapsed());
                    break;
                }
                std::thread::sleep(Duration::from_millis(2));
            }
            Ok(())
        } else {
            // Fast-path: only if !pending_rearm && is_enabled && feedback_valid && all state == 1
            if !self.pending_rearm
                && self.is_enabled
                && self.is_feedback_valid()
                && self.motor_status.iter().all(|&st| st == 1)
            {
                return Ok(());
            }

            // A motor that did not answer the startup range read gets another (read-only)
            // try before each enable; a range mismatch refuses arming.
            if self.range_check.as_ref().is_some_and(|c| c.status == RangeCheckStatus::Unverified) {
                self.check_mapping_ranges();
            }
            if let Some(check) = self.range_check.as_ref().filter(|c| c.status == RangeCheckStatus::Mismatch) {
                self.is_enabled = false;
                self.pending_rearm = true;
                return Err(ArmDriverError::HardwareFault(check.message.clone()));
            }

            self.is_enabling = true;

            if !self.drain_rx_before_command() {
                self.awaiting_rx_boundary = true;
                self.is_enabling = false;
                self.is_enabled = false;
                self.pending_rearm = true;
                return Err(ArmDriverError::HardwareFault("CAN RX buffer not drained before handshake".to_string()));
            }
            self.awaiting_rx_boundary = false;

            // CTRL_MODE lives in motor RAM: a joint left in MIT by an interrupted gravity
            // compensation session must be back in the base mode before it is enabled.
            let stray: Vec<usize> = (0..self.motor_ids.len()).filter(|&i| self.joint_modes[i] != Some(self.control_mode)).collect();
            if !stray.is_empty() {
                let base = vec![self.control_mode; self.motor_ids.len()];
                let hold: Vec<JointTarget> = self.cached_states.iter().map(|s| JointTarget::position(s.position)).collect();
                let failed = self.write_modes(&stray, &base, &hold, true);
                if !failed.is_empty() {
                    self.is_enabling = false;
                    self.is_enabled = false;
                    self.pending_rearm = true;
                    return Err(ArmDriverError::HardwareFault(format!(
                        "Control mode of joints {:?} could not be restored before enable",
                        failed.iter().map(|i| i + 1).collect::<Vec<_>>())));
                }
            }

            let handshake_start = Instant::now();
            self.last_enable_command = Some(handshake_start);
            self.last_feedback_instant.fill(None);

            for &id in &self.motor_ids {
                let frame = damiao::build_enable_frame(id);
                if let Err(e) = self.can.transmit(&frame) {
                    self.is_enabling = false;
                    self.is_enabled = false;
                    self.pending_rearm = true;
                    return Err(e.into());
                }
            }
            if let Err(e) = self.can.flush() {
                self.is_enabling = false;
                self.is_enabled = false;
                self.pending_rearm = true;
                return Err(e.into());
            }

            // Wait and poll for responses up to 80ms, max 30 checks
            let poll_start = Instant::now();
            let mut retried = false;
            let mut all_enabled = false;
            let mut check_count = 0;
            while poll_start.elapsed() < Duration::from_millis(80) && check_count < 30 {
                check_count += 1;
                let _ = self.read_joint_states();
                let all_fresh = !self.last_feedback_instant.is_empty()
                    && self.last_feedback_instant.iter().all(|opt| {
                        opt.is_some_and(|t| t >= handshake_start)
                    });

                if all_fresh {
                    // Check if any motor reported a fault
                    if let Err(e) = self.check_hardware_status() {
                        self.is_enabling = false;
                        self.is_enabled = false;
                        self.pending_rearm = true;
                        return Err(e);
                    }
                    // Check if all motors report enabled (state == 1)
                    if self.motor_status.iter().all(|&st| st == 1) {
                        all_enabled = true;
                        tracing::info!("All Damiao motors confirmed enabled (state=1) in {:?}", poll_start.elapsed());
                        break;
                    }
                }

                // If not all enabled after 25ms or 10 checks, retry once for motors that have not responded or are not enabled
                if !retried && (poll_start.elapsed() >= Duration::from_millis(25) || check_count >= 10) {
                    for (i, &id) in self.motor_ids.iter().enumerate() {
                        let motor_fresh_and_enabled = self.last_feedback_instant.get(i).copied().flatten().is_some_and(|t| t >= handshake_start)
                            && self.motor_status.get(i).copied() == Some(1);
                        if !motor_fresh_and_enabled {
                            tracing::debug!("Retrying enable for motor {} (CAN ID 0x{:X})", i, id);
                            let frame = damiao::build_enable_frame(id);
                            if let Err(e) = self.can.transmit(&frame) {
                                tracing::warn!("Retry transmit enable frame failed for motor {}: {:?}", id, e);
                            }
                        }
                    }
                    if let Err(e) = self.can.flush() {
                        tracing::warn!("Retry flush failed: {:?}", e);
                    }
                    retried = true;
                }

                std::thread::sleep(Duration::from_micros(500));
            }

            self.is_enabling = false;

            if !all_enabled {
                self.is_enabled = false;
                self.pending_rearm = true;
                return Err(ArmDriverError::HardwareFault(format!(
                    "Not all Damiao motors entered enabled state within 80ms: feedback_valid={}, status={:?}",
                    self.is_feedback_valid(), self.motor_status
                )));
            }
            self.is_enabled = true;
            self.pending_rearm = false;
            Ok(())
        }
    }

    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        if targets.len() != self.motor_ids.len() {
            return Err(ArmDriverError::JointCountMismatch {
                expected: self.motor_ids.len(),
                actual: targets.len(),
            });
        }

        match self.control_mode {
            DamiaoControlMode::PosVel => {
                // Per joint: POS_VEL, MIT for joints switched by set_joint_modes, both when unconfirmed.
                for (i, target) in targets.iter().enumerate() {
                    for frame in self.target_frames(i, target) {
                        self.transmit_with_retry(&frame)?;
                        self.last_target_frames[i] = Some(frame);
                    }
                }
            }
            DamiaoControlMode::Mit => {
                for (i, target) in targets.iter().enumerate() {
                    let kp = target.kp.unwrap_or(self.default_kp);
                    let kd = target.kd.unwrap_or(self.default_kd);
                    let vel = target.velocity.unwrap_or(0.0);
                    let torque = target
                        .torque
                        .map(|t| if t.is_finite() { t } else { 0.0 })
                        .unwrap_or(0.0);

                    let frame = damiao::build_mit_frame(
                        self.motor_ids[i],
                        target.position,
                        vel,
                        kp,
                        kd,
                        torque,
                        &self.limits[i],
                    );
                    self.transmit_with_retry(&frame)?;
                    self.last_target_frames[i] = Some(frame);
                }
            }
        }
        self.can.flush()?;
        Ok(())
    }

    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        const MAX_RX_DURATION: Duration = Duration::from_millis(5);
        let t0 = Instant::now();
        let mut frames_processed = 0;
        let mut rx_drained = false;
        while frames_processed < 64 && t0.elapsed() < MAX_RX_DURATION {
            match self.can.receive() {
                Ok(frame) => {
                    frames_processed += 1;
                    if self.awaiting_rx_boundary {
                        if frames_processed >= 64 || t0.elapsed() >= MAX_RX_DURATION { break; }
                        continue;
                    }
                    // Ignore transmit commands: on SocketCAN with local echo enabled,
                    // host transmit frames for PosVel are 0x100 + tx_id, gripper echo 0x300 + tx_id,
                    // or special 0x7FF, or command frames.
                    if frame.id == 0x7FF
                        || self
                            .motor_ids
                            .iter()
                            .any(|&tx_id| frame.id == (0x100 + tx_id as u32) || frame.id == (0x300 + tx_id as u32))
                        || frame.data == damiao::CMD_ENABLE
                        || frame.data == damiao::CMD_DISABLE
                        || frame.data == damiao::CMD_CLEAR_FAULTS
                        || frame.data == damiao::CMD_SET_ZERO
                    {
                        if frames_processed >= 64 || t0.elapsed() >= MAX_RX_DURATION {
                            break;
                        }
                        continue;
                    }
                    // Local echo of an MIT command (same ID as some feedback paths).
                    if self.last_target_frames.iter().flatten().any(|f| f.id == frame.id && f.data == frame.data) {
                        if frames_processed >= 64 || t0.elapsed() >= MAX_RX_DURATION { break; }
                        continue;
                    }
                    if let Some(reply) = damiao::parse_register_reply(&frame)
                        .filter(|r| self.motor_ids.contains(&r.motor_id))
                    {
                        // A register reply is not feedback: byte 0 would decode as state 0.
                        if self.register_replies.len() < 64 { self.register_replies.push(reply); }
                        if frames_processed >= 64 || t0.elapsed() >= MAX_RX_DURATION { break; }
                        continue;
                    }
                    if !frame.is_extended && !frame.is_rtr && frame.dlc == 8 {
                        // In Damiao protocol, feedback is addressed to master_id (or 0x00),
                        // or broadcast with the motor's own ID, or 0x10 + motor_id.
                        let is_valid_feedback_id = frame.id == 0
                            || frame.id == self.master_id
                            || self
                                .motor_ids
                                .iter()
                                .any(|&m| frame.id == m as u32 || frame.id == (0x10 + m as u32));
                        if !is_valid_feedback_id {
                            if frames_processed >= 64 || t0.elapsed() >= MAX_RX_DURATION {
                                break;
                            }
                            continue;
                        }

                        // Byte 0 names the motor; decode with that joint's own range.
                        let fid = damiao::feedback_motor_id(&frame);
                        let idx = self.motor_ids.iter().position(|&id| id as u8 == fid);
                        if let Some((idx, Ok(feedback))) = idx.map(|idx| (idx, damiao::unpack_feedback_frame(&frame, &self.limits[idx]))) {
                            self.cached_states[idx] = JointState {
                                position: feedback.position,
                                velocity: feedback.velocity,
                                torque: feedback.torque,
                                temperature: feedback.t_mos.max(feedback.t_rotor),
                            };
                            self.last_feedback_instant[idx] = Some(Instant::now());
                            self.motor_status[idx] = feedback.state_or_error;
                        }
                    }
                    if frames_processed >= 64 || t0.elapsed() >= MAX_RX_DURATION {
                        break;
                    }
                }
                Err(TransportError::Timeout) | Err(TransportError::WouldBlock) => {
                    rx_drained = true;
                    if self.awaiting_rx_boundary {
                        self.awaiting_rx_boundary = false;
                        self.last_feedback_instant.fill(None);
                        self.last_enable_command = Some(Instant::now());
                    }
                    break;
                }
                Err(_) => break,
            }
        }

        // Evaluate feedback age only after receive() explicitly reports an empty
        // queue. If the bounded budget expires first, defer recovery to the next
        // tick so a frame beyond the budget cannot be mistaken for a bus timeout.
        if rx_drained && self.is_enabled && !self.is_enabling
            && self.enable_state() == EnableState::Enabled {
            for (i, &id) in self.motor_ids.iter().enumerate() {
                let stale = self
                    .last_feedback_instant
                    .get(i)
                    .and_then(|&opt| opt)
                    .is_none_or(|t| t.elapsed() >= Duration::from_millis(45));
                if !stale {
                    continue;
                }

                // Recovery never replays a queued motion target. A still-fresh,
                // confirmed enabled joint may only hold its last measured pose.
                let p = self.cached_states.get(i).map_or(0.0, |s| s.position);
                match self.joint_modes.get(i).copied().flatten().unwrap_or(self.control_mode) {
                    DamiaoControlMode::Mit if self.control_mode == DamiaoControlMode::PosVel => {
                        // A joint switched to MIT by set_joint_modes (gravity compensation):
                        // repeat the last MIT command instead of inventing gains or dropping
                        // its torque feed-forward; the supervisor owns the hold decision.
                        if let Some(frame) = self.last_target_frames[i].filter(|f| f.id == u32::from(id)) {
                            let _ = self.can.transmit(&frame);
                        }
                    }
                    DamiaoControlMode::Mit => {
                        // Recovery must use the active protocol mode. Re-send a
                        // zero-velocity hold with the configured impedance gains.
                        let frame = damiao::build_mit_frame(
                            id,
                            p,
                            0.0,
                            self.default_kp,
                            self.default_kd,
                            0.0,
                            &self.limits[i],
                        );
                        let _ = self.can.transmit(&frame);
                    }
                    DamiaoControlMode::PosVel => {
                        let frame = damiao::build_pos_vel_frame(
                            0x100,
                            id,
                            p,
                            self.default_vlim.abs().clamp(0.1, 4.0),
                        );
                        let _ = self.can.transmit(&frame);
                    }
                }
            }
        }
        // Damiao motors only transmit in reply to a frame. When feedback goes stale
        // (idle, bus glitch, motor power dip) nothing would ever refresh it, so a
        // fault could never be cleared. Poll stale motors with the read-only 0xCC
        // refresh; it does not change enable state or targets.
        const REFRESH_AFTER: Duration = Duration::from_millis(100);
        if rx_drained && !self.awaiting_rx_boundary && !self.is_enabling
            && self.last_refresh_instant.is_none_or(|t| t.elapsed() >= REFRESH_AFTER)
        {
            let stale: Vec<u16> = self.motor_ids.iter().zip(&self.last_feedback_instant)
                .filter(|(_, t)| t.is_none_or(|t| t.elapsed() >= REFRESH_AFTER))
                .map(|(&id, _)| id).collect();
            if !stale.is_empty() {
                for id in stale {
                    let _ = self.can.transmit(&damiao::build_refresh_frame(id));
                }
                let _ = self.can.flush();
                self.last_refresh_instant = Some(Instant::now());
            }
        }
        Ok(self.cached_states.clone())
    }
}

// =========================================================================
// 4. RobStride Actuator Arm Driver (reBot Arm B601-RS)
// =========================================================================

pub struct RobStrideArmDriver<C: CanTransport> {
    can: C,
    motor_ids: Vec<u8>,
    master_id: u8,
    limits: RobStrideLimits,
    default_kp: f32,
    default_kd: f32,
    cached_states: Vec<JointState>,
    last_feedback_instant: Vec<Option<Instant>>,
    fault_flags: Vec<u8>,
}

impl<C: CanTransport> RobStrideArmDriver<C> {
    pub fn new(can: C, motor_ids: Vec<u8>) -> Self {
        let dof = motor_ids.len();
        Self {
            can,
            motor_ids,
            master_id: 0xFD,
            limits: RobStrideLimits::default(),
            default_kp: 40.0,
            default_kd: 1.0,
            cached_states: vec![JointState::default(); dof],
            last_feedback_instant: vec![None; dof],
            fault_flags: vec![0; dof],
        }
    }

    pub fn with_gains(mut self, kp: f32, kd: f32) -> Self {
        self.default_kp = kp;
        self.default_kd = kd;
        self
    }

    pub fn can_mut(&mut self) -> &mut C {
        &mut self.can
    }
}

impl<C: CanTransport> ArmDriver for RobStrideArmDriver<C> {
    fn capabilities(&self) -> DriverCapabilities { DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true, torque_feedforward: true, enable_feedback: false, load_feedback: false } }

    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        self.last_feedback_instant.iter().map(|t| t.map(|t| t.elapsed())).collect()
    }

    fn is_feedback_valid(&self) -> bool {
        if self.last_feedback_instant.is_empty() {
            return false;
        }
        const FRESHNESS_LIMIT: Duration = Duration::from_millis(250);
        self.last_feedback_instant
            .iter()
            .all(|opt| opt.is_some_and(|instant| instant.elapsed() <= FRESHNESS_LIMIT))
    }

    fn check_hardware_status(&self) -> Result<(), ArmDriverError> {
        for (i, &flags) in self.fault_flags.iter().enumerate() {
            if flags != 0 {
                let motor_id = self.motor_ids.get(i).copied().unwrap_or(0);
                return Err(ArmDriverError::HardwareFault(format!(
                    "RobStride Joint {} (CAN ID 0x{:X}) reported fault flags: 0x{:02X}",
                    i, motor_id, flags
                )));
            }
        }
        Ok(())
    }

    fn clear_fault(&mut self) -> Result<(), ArmDriverError> {
        let mut err = None;
        for &id in &self.motor_ids {
            let frame = robstride::build_clear_fault_frame(id, self.master_id);
            if let Err(e) = self.can.transmit(&frame) {
                err = Some(e);
            }
        }
        if let Err(e) = self.can.flush() { err = Some(e); }
        // A sent reset is not proof that the reported fault has cleared.
        // The next telemetry response updates fault_flags.
        if let Some(e) = err {
            Err(ArmDriverError::HardwareFault(e.to_string()))
        } else {
            Ok(())
        }
    }

    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        if !enabled {
            let mut last_err = None;
            for &id in &self.motor_ids {
                let frame = robstride::build_stop_frame(id, self.master_id);
                if let Err(e) = self.can.transmit(&frame) {
                    last_err = Some(e.into());
                }
            }
            if let Err(e) = self.can.flush() {
                last_err = Some(e.into());
            }
            if let Some(e) = last_err {
                return Err(e);
            }
            Ok(())
        } else {
            for &id in &self.motor_ids {
                let frame = robstride::build_enable_frame(id, self.master_id);
                self.can.transmit(&frame)?;
            }
            self.can.flush()?;
            Ok(())
        }
    }

    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        if targets.len() != self.motor_ids.len() {
            return Err(ArmDriverError::JointCountMismatch {
                expected: self.motor_ids.len(),
                actual: targets.len(),
            });
        }

        for (i, target) in targets.iter().enumerate() {
            let kp = target.kp.unwrap_or(self.default_kp);
            let kd = target.kd.unwrap_or(self.default_kd);
            let vel = target.velocity.unwrap_or(0.0);
            let torque = target.torque.unwrap_or(0.0);

            let frame = robstride::build_mit_frame(
                self.motor_ids[i],
                target.position,
                vel,
                kp,
                kd,
                torque,
                &self.limits,
            );
            self.can.transmit(&frame)?;
        }
        self.can.flush()?;
        Ok(())
    }

    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let t0 = Instant::now();
        let mut frames_processed = 0;
        while frames_processed < 64 {
            match self.can.receive() {
                Ok(frame) => {
                    frames_processed += 1;
                    if let Ok(feedback) = robstride::unpack_feedback_frame(&frame, &self.limits) {
                        if let Some(idx) = self
                            .motor_ids
                            .iter()
                            .position(|&id| id == feedback.motor_id)
                        {
                            self.cached_states[idx] = JointState {
                                position: feedback.position,
                                velocity: feedback.velocity,
                                torque: feedback.torque,
                                temperature: feedback.mos_temperature,
                            };
                            self.last_feedback_instant[idx] = Some(Instant::now());
                            self.fault_flags[idx] = feedback.fault_flags;
                        }
                    }
                    if frames_processed >= 64 || t0.elapsed() >= Duration::from_micros(2000) {
                        break;
                    }
                }
                Err(_) => break,
            }
        }
        Ok(self.cached_states.clone())
    }
}
