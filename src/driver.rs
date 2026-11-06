use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::model::JointState;
use crate::protocol::damiao::{self, DamiaoLimits};
pub use crate::protocol::damiao::{build_pos_force_frame, DAMIAO_GRIPPER_POSFORCE_VEL_RAW};
use crate::protocol::fashionstar::{self, FashionStarPacket};
use crate::protocol::feetech::{self, FeetechPacket};
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

/// Hardware enable state confirmed by fresh telemetry, not by a successful write.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum EnableState {
    Enabled,
    Disabled,
    #[default]
    Unknown,
}

/// Telemetry and command features implemented by a driver in its active mode.
/// A false feedback flag means the numeric placeholder is not a measurement.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct DriverCapabilities {
    pub velocity_feedback: bool,
    pub torque_feedback: bool,
    pub temperature_feedback: bool,
    pub torque_feedforward: bool,
    pub enable_feedback: bool,
}

/// Universal robotic arm hardware driver trait
pub trait ArmDriver: Send {
    fn capabilities(&self) -> DriverCapabilities { DriverCapabilities::default() }

    fn enable_state(&self) -> EnableState { EnableState::Unknown }

    /// Age of the last received sample for each joint. Reading a cache does not refresh it.
    fn feedback_ages(&self) -> Vec<Option<Duration>> { Vec::new() }

    /// Fixed device-side velocity ceiling, when the active output mode has one.
    fn max_command_velocity(&self, _joint: usize) -> Option<f32> { None }

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

    /// Check whether driver / hardware is actively enabled and online
    fn is_enabled(&self) -> bool {
        false
    }

    /// Return per-joint motor status or error code if supported by the driver
    fn motor_status(&self) -> Option<Vec<u8>> {
        None
    }

    /// Update internal software enabled flag without transmitting physical frames
    fn set_torque_flag(&mut self, _enabled: bool) {}

    /// Update pending_rearm flag to force full hardware handshake on next enable
    fn set_pending_rearm(&mut self, _pending: bool) {}
}

// =========================================================================
// 1. Feetech STS3215 Arm Driver (SO-ARM 101)
// =========================================================================

pub struct FeetechArmDriver<T: Transport> {
    transport: T,
    servo_ids: Vec<u8>,
    zero_offsets: Vec<u16>,
    cached_states: Vec<JointState>,
    last_feedback_instant: Vec<Option<Instant>>,
    poll_cursor: usize,
    rx_buffer: Vec<u8>,
}

impl<T: Transport> FeetechArmDriver<T> {
    pub fn new(transport: T, servo_ids: Vec<u8>) -> Self {
        let dof = servo_ids.len();
        Self {
            transport,
            servo_ids,
            zero_offsets: vec![feetech::DEFAULT_ZERO_OFFSET; dof],
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

    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }
}

impl<T: Transport> ArmDriver for FeetechArmDriver<T> {
    fn capabilities(&self) -> DriverCapabilities { DriverCapabilities { velocity_feedback: true, temperature_feedback: true, ..DriverCapabilities::default() } }

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
            .all(|opt| opt.map_or(false, |instant| instant.elapsed() <= FRESHNESS_LIMIT))
    }

    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        if !enabled {
            let mut last_err = None;
            for &id in &self.servo_ids {
                let pkt = feetech::build_set_torque(id, false);
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
                let pkt = feetech::build_set_torque(id, true);
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

        let mut positions = Vec::with_capacity(targets.len());
        for (i, target) in targets.iter().enumerate() {
            let raw_steps = feetech::rad_to_steps(target.position, self.zero_offsets[i]);
            positions.push((self.servo_ids[i], raw_steps));
        }

        let sync_pkt = feetech::build_sync_write_positions(&positions)
            .map_err(|e| ArmDriverError::Protocol(e.to_string()))?;

        self.transport.write_all(&sync_pkt)?;
        self.transport.flush()?;
        Ok(())
    }

    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let dof = self.servo_ids.len();
        if dof == 0 {
            return Ok(Vec::new());
        }

        let mut temp_chunk = [0u8; 64];
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

            // Read present position, speed, load, voltage, temp (start 0x38, len 8)
            let read_req = feetech::build_read(id, feetech::REG_PRESENT_POSITION, 8);
            self.transport.write_all(&read_req)?;
            self.transport.flush()?;

            // Attempt to parse response from rx_buffer, reading more if needed
            let mut attempts = 0;
            while attempts < 10 {
                if start_time.elapsed() >= MAX_READ_DURATION || total_bytes_read >= MAX_TOTAL_BYTES {
                    self.poll_cursor = (joint_idx + 1) % dof;
                    break 'poll;
                }

                match feetech::parse_response(&self.rx_buffer) {
                    Ok((FeetechPacket { id: pkt_id, params, .. }, consumed)) => {
                        self.rx_buffer.drain(..consumed);
                        if pkt_id == id && params.len() >= 8 {
                            let pos_raw = u16::from_le_bytes([params[0], params[1]]);
                            let speed_raw = i16::from_le_bytes([params[2], params[3]]);
                            let load_raw = u16::from_le_bytes([params[4], params[5]]);
                            let temp_raw = params[7];

                            self.cached_states[joint_idx] = JointState {
                                position: feetech::steps_to_rad(pos_raw, self.zero_offsets[joint_idx]),
                                velocity: feetech::raw_speed_to_rad_s(speed_raw),
                                torque: feetech::raw_load_to_torque(load_raw),
                                temperature: feetech::raw_to_temperature(temp_raw),
                            };
                            self.last_feedback_instant[joint_idx] = Some(Instant::now());
                            break;
                        }
                    }
                    Err(feetech::FeetechError::IncompleteFrame { .. }) => {
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

                // Read more data into rx_buffer
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

    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
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
            .all(|opt| opt.map_or(false, |instant| instant.elapsed() <= FRESHNESS_LIMIT))
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

        let mut temp_chunk = [0u8; 64];
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
    limits: DamiaoLimits,
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
}

impl<C: CanTransport> DamiaoArmDriver<C> {
    pub fn new(can: C, motor_ids: Vec<u16>) -> Self {
        let dof = motor_ids.len();
        Self {
            can,
            motor_ids,
            master_id: 0,
            limits: DamiaoLimits::default(),
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
        }
    }

    pub fn with_master_id(mut self, master_id: u32) -> Self {
        self.master_id = master_id;
        self
    }

    pub fn with_control_mode(mut self, mode: DamiaoControlMode) -> Self {
        self.control_mode = mode;
        self
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
        }
    }

    fn enable_state(&self) -> EnableState {
        if self.awaiting_rx_boundary {
            return EnableState::Unknown;
        }
        if self.last_feedback_instant.iter().any(|t| {
            self.last_enable_command.map_or(false, |command| t.map_or(true, |sample| sample < command))
        }) {
            return EnableState::Unknown;
        }
        if self.is_enabled {
            if !self.is_feedback_valid() {
                return EnableState::Unknown;
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
            let ok = opt.map_or(false, |instant| instant.elapsed() <= limit);
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
            if st >= 8 {
                let reason = match st {
                    8 => "Overvoltage",
                    9 => "Undervoltage",
                    0xA => "Overcurrent",
                    0xB => "MOS Overtemperature",
                    0xC => "Rotor Overtemperature",
                    0xD => "CAN Communication Loss",
                    0xE => "Overload",
                    _ => "Motor Hardware Fault",
                };
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
                        opt.map_or(false, |t| t >= cmd_start)
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

            self.is_enabling = true;

            if !self.drain_rx_before_command() {
                self.awaiting_rx_boundary = true;
                self.is_enabling = false;
                self.is_enabled = false;
                self.pending_rearm = true;
                return Err(ArmDriverError::HardwareFault("CAN RX buffer not drained before handshake".to_string()));
            }
            self.awaiting_rx_boundary = false;

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
                        opt.map_or(false, |t| t >= handshake_start)
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
                        let motor_fresh_and_enabled = self.last_feedback_instant.get(i).copied().flatten().map_or(false, |t| t >= handshake_start)
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
                for (i, target) in targets.iter().enumerate() {
                    let vlim = self.default_vlim.abs().clamp(0.1, 4.0);
                    let frame = damiao::build_pos_vel_frame(
                        0x100,
                        self.motor_ids[i],
                        target.position,
                        vlim,
                    );
                    self.transmit_with_retry(&frame)?;
                    self.last_target_frames[i] = Some(frame);
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
                        &self.limits,
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

                        if let Ok(feedback) = damiao::unpack_feedback_frame(&frame, &self.limits) {
                            if let Some(idx) = self
                                .motor_ids
                                .iter()
                                .position(|&id| id as u8 == feedback.motor_id)
                            {
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
                    .map_or(true, |t| t.elapsed() >= Duration::from_millis(45));
                if !stale {
                    continue;
                }

                // Recovery never replays a queued motion target. A still-fresh,
                // confirmed enabled joint may only hold its last measured pose.
                let p = self.cached_states.get(i).map_or(0.0, |s| s.position);
                match self.control_mode {
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
                            &self.limits,
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
    fn capabilities(&self) -> DriverCapabilities { DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true, torque_feedforward: true, enable_feedback: false } }

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
            .all(|opt| opt.map_or(false, |instant| instant.elapsed() <= FRESHNESS_LIMIT))
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
