use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::{broadcast, RwLock};
use thiserror::Error;

use crate::driver::{ArmDriver, JointTarget};
use crate::model::{ArmConfig, ArmObservation, ObservationSchema};
use crate::recording::{EpisodeRecorder, EpisodeState, RecordFrame, RecordingError};
use crate::safety::{FaultReason, SafetyError, SafetyManager, SafetyState, StopPolicy};
use crate::trajectory::{MultiJointQuinticTrajectory, TrajectoryError, TrajectoryPlanner};

#[derive(Debug, Error)]
pub enum ArmError {
    #[error("Safety error: {0}")]
    Safety(#[from] SafetyError),

    #[error("Trajectory error: {0}")]
    Trajectory(#[from] TrajectoryError),

    #[error("Recording error: {0}")]
    Recording(#[from] RecordingError),

    #[error("Internal error: {0}")]
    Internal(String),
}

/// Active WebSocket client lease record
#[derive(Debug, Clone, PartialEq)]
pub struct ActiveLease {
    pub client_id: String,
    pub generation: u64,
    pub acquired_at: Instant,
    pub last_valid_motion_instant: Option<Instant>,
}

/// Internal mutable state of the robotic arm supervisor
pub struct SupervisorInner {
    pub config: ArmConfig,
    pub safety: SafetyManager,
    pub measured_positions: Vec<f32>,
    pub measured_velocities: Vec<f32>,
    pub commanded_positions: Vec<f32>,
    pub commanded_velocities: Vec<f32>,
    pub last_commanded_velocities: Vec<f32>,
    pub torques: Vec<f32>,
    pub temperatures: Vec<f32>,
    pub target_positions: Vec<f32>,
    pub servo_target: Option<Vec<f32>>,
    pub servo_settled_ticks: Vec<u32>,
    pub is_torqued: bool,
    pub active_trajectory: Option<MultiJointQuinticTrajectory>,
    pub last_heartbeat: Instant,
    pub last_motion_frame: Instant,
    pub motion_frame_timeout: Duration,
    pub lease_timeout: Duration,
    pub lease_enabled: bool,
    pub active_lease: Option<ActiveLease>,
    pub lease_generation_counter: u64,
    pub gripper_stalled: bool,
    pub gripper_high_torque_ticks: u32,
    pub gripper_low_torque_ticks: u32,
    pub gripper_stall_sign: f32,
    pub pending_normal_unload: bool,
    pub unload_settle_ticks: Option<u32>,
    pub recorder: EpisodeRecorder,
    pub start_time: Instant,
    pub driver: Option<std::sync::Mutex<Box<dyn ArmDriver>>>,
    pub gravity_compensated: bool,
    pub gravity_torques: Vec<f32>,
    pub gravity_scale: f32,
    pub gravity_torques_ramped: Vec<f32>,
    pub last_can_feedback_instant: Option<Instant>,
    pub bus_loss_instant: Option<Instant>,
    pub e_stop_latched: Arc<AtomicBool>,
}

impl SupervisorInner {
    pub fn new(mut config: ArmConfig) -> Self {
        config.validate().expect("Invalid ArmConfig: invariant violation");
        let dof = config.dof;
        let positions = config.home_positions.clone();
        let target_positions = positions.clone();
        let lease_timeout = Duration::from_millis(config.lease_timeout_ms);
        let motion_frame_timeout = Duration::from_millis(200);
        let now = Instant::now();

        let candidate_urdf_paths = [
            "assets/reBot-DevArm_gripper.urdf",
            "/usr/share/rebot-teleop/assets/reBot-DevArm_gripper.urdf",
            "/usr/share/rebot-teleop/reBot-DevArm_gripper.urdf",
        ];
        let mut gravity_compensated = false;
        for path in &candidate_urdf_paths {
            if std::path::Path::new(path).exists() {
                match crate::gravity::verify_urdf_sha256(path) {
                    Ok(()) => {
                        tracing::info!("URDF model verified ({}); gravity compensation enabled.", path);
                        gravity_compensated = true;
                        break;
                    }
                    Err(e) => {
                        tracing::warn!("URDF verification failed for {} ({:?})", path, e);
                    }
                }
            }
        }
        if !gravity_compensated {
            tracing::warn!("Running in degraded mode without gravity compensation.");
        }

        Self {
            config,
            safety: SafetyManager::new(),
            measured_positions: positions.clone(),
            measured_velocities: vec![0.0; dof],
            commanded_positions: positions,
            commanded_velocities: vec![0.0; dof],
            last_commanded_velocities: vec![0.0; dof],
            torques: vec![0.0; dof],
            temperatures: vec![25.0; dof],
            target_positions,
            servo_target: None,
            servo_settled_ticks: vec![0; dof],
            is_torqued: false,
            active_trajectory: None,
            last_heartbeat: now,
            last_motion_frame: now,
            motion_frame_timeout,
            lease_timeout,
            lease_enabled: true,
            active_lease: None,
            lease_generation_counter: 0,
            gripper_stalled: false,
            gripper_high_torque_ticks: 0,
            gripper_low_torque_ticks: 0,
            gripper_stall_sign: 1.0,
            pending_normal_unload: false,
            unload_settle_ticks: None,
            recorder: EpisodeRecorder::new(),
            start_time: now,
            driver: None,
            gravity_compensated,
            gravity_torques: Vec::new(),
            gravity_scale: 0.5,
            gravity_torques_ramped: vec![0.0; dof],
            last_can_feedback_instant: None,
            bus_loss_instant: None,
            e_stop_latched: Arc::new(AtomicBool::new(false)),
        }
    }

    pub fn with_driver(mut self, driver: Box<dyn ArmDriver>) -> Self {
        self.driver = Some(std::sync::Mutex::new(driver));
        self
    }

    pub fn current_observation(&self) -> ArmObservation {
        let timestamp = self.start_time.elapsed().as_secs_f64();
        ArmObservation {
            timestamp,
            positions: self.measured_positions.clone(),
            velocities: self.measured_velocities.clone(),
            torques: self.torques.clone(),
            temperatures: self.temperatures.clone(),
            is_torqued: self.is_torqued,
            safety_state: self.safety.state().clone(),
            epoch_id: self.safety.epoch_id(),
            gravity_compensated: self.gravity_compensated,
            gravity_torques: self.gravity_torques.clone(),
        }
    }

    pub fn is_at_safe_resting_pose(&self) -> bool {
        let dof = self.config.dof;
        if self.measured_positions.len() < dof || self.config.home_positions.len() < dof {
            return false;
        }

        // 硬件前置条件: 若挂载了真实硬件驱动，必须处于有效使能闭环且反馈新鲜，且非停止/故障态
        if let Some(ref d) = self.driver {
            if let Ok(drv) = d.lock() {
                if !drv.is_feedback_valid() || !drv.is_enabled() {
                    return false;
                }
            } else {
                return false;
            }
            if self.safety.state().is_stopped() {
                return false;
            }
            if self.last_can_feedback_instant.map_or(true, |t| t.elapsed() > Duration::from_millis(100)) {
                return false;
            }
        }

        // 判定1 (几何): 遍历 J1~J6（若 dof > 6，前 6 轴为主臂体轴），逐轴偏差 abs <= 0.08 rad，且前 6 轴绝对偏差之和 <= 0.16 rad；
        // 若存在第 7 轴（夹爪），不阻塞主体安全支撑判定。
        let arm_dof = dof.min(6);
        let mut total_deviation = 0.0f32;
        for i in 0..arm_dof {
            let p = self.measured_positions[i];
            let h = self.config.home_positions[i];
            if !p.is_finite() || !h.is_finite() {
                return false;
            }
            let dev = (p - h).abs();
            if dev > 0.08 {
                return false;
            }
            total_deviation += dev;
        }
        if total_deviation > 0.16 {
            return false;
        }

        // 判定2 (静止): 各轴实测速度绝对值 <= 0.015 rad/s
        if self.measured_velocities.len() < dof {
            return false;
        }
        for i in 0..dof {
            let v = self.measured_velocities[i];
            if !v.is_finite() || v.abs() > 0.015 {
                return false;
            }
        }

        // 判定3 (力矩): 检查主承重俯仰轴 J2 与 J3 的实测力矩绝对值
        // 原位折叠就位时: J2 受底座直接承托实测力矩 <= 1.5 N·m (悬空约为 1.1~2.5 N·m)
        //                 J3 小臂折叠受托实测力矩 <= 4.5 N·m (悬空全伸出时重力矩高达 7.65 N·m)
        // 若是仿真驱动 (driver.is_none())，视为满足
        if self.driver.is_none() {
            return true;
        }

        let j2_j3_limits = [(1usize, 1.5f32), (2usize, 4.5f32)];
        for &(idx, max_tau) in &j2_j3_limits {
            if idx < dof {
                let measured_tau = self.torques.get(idx).copied().unwrap_or(f32::NAN);
                if !measured_tau.is_finite() || measured_tau.abs() > max_tau {
                    return false;
                }
            }
        }

        true
    }

    pub fn set_torque(&mut self, enable: bool) -> Result<SafetyState, SafetyError> {
        if enable {
            // Atomic e-stop check
            if self.e_stop_latched.load(Ordering::SeqCst) {
                return Err(SafetyError::EStopActive);
            }

            // Strict pre-check: If stopped (E-Stop or Fault), reject IMMEDIATELY before touching driver!
            if self.safety.state().is_stopped() {
                if let SafetyState::Fault(ref reason) = self.safety.state() {
                    return Err(SafetyError::FaultActive(reason.clone()));
                }
                return Err(SafetyError::EStopActive);
            }

            // Enable hardware torque so bus devices wake up and publish state, then verify physical feedback
            let was_torqued = self.is_torqued;
            let prev_gripper_cmd = if was_torqued && self.config.dof > 0 {
                Some(self.commanded_positions[self.config.dof - 1])
            } else {
                None
            };

            if let Some(ref d) = self.driver {
                // Secondary double-check latch right before hardware actuation
                if self.e_stop_latched.load(Ordering::SeqCst) {
                    return Err(SafetyError::EStopActive);
                }
                let mut driver = d.lock().map_err(|_| SafetyError::FeedbackUnavailable)?;
                if self.e_stop_latched.load(Ordering::SeqCst) {
                    return Err(SafetyError::EStopActive);
                }
                if let Err(_) = driver.set_torque(true) {
                    if !was_torqued {
                        let _ = driver.set_torque(false);
                        self.is_torqued = false;
                    }
                    self.safety.trigger_fault(FaultReason::Other("Hardware torque enable failed".to_string()));
                    return Err(SafetyError::FeedbackUnavailable);
                }
                let states = match driver.read_joint_states() {
                    Ok(s) => s,
                    Err(_) => {
                        if !was_torqued {
                            let _ = driver.set_torque(false);
                            self.is_torqued = false;
                        }
                        return Err(SafetyError::FeedbackUnavailable);
                    }
                };
                if !driver.is_feedback_valid() || states.len() < self.config.dof {
                    if !was_torqued {
                        let _ = driver.set_torque(false);
                        self.is_torqued = false;
                    }
                    return Err(SafetyError::FeedbackUnavailable);
                }
                self.last_can_feedback_instant = Some(Instant::now());
                for (i, state) in states.iter().enumerate() {
                    if i < self.config.dof {
                        self.measured_positions[i] = state.position;
                        self.measured_velocities[i] = state.velocity;
                        self.torques[i] = state.torque;
                        self.temperatures[i] = state.temperature;
                    }
                }
            }

            // Initialize commanded_positions to verified measured_positions to prevent sudden jumps
            self.commanded_positions = self.measured_positions.clone();
            self.target_positions = self.measured_positions.clone();
            // Preserve gripper holding position when re-arming an already torqued arm so grasped objects are not released
            if let Some(g_cmd) = prev_gripper_cmd {
                let grip_idx = self.config.dof - 1;
                self.commanded_positions[grip_idx] = g_cmd;
                self.target_positions[grip_idx] = g_cmd;
            }
            self.commanded_velocities = vec![0.0; self.config.dof];
            self.measured_velocities = vec![0.0; self.config.dof];
            self.active_trajectory = None;
            self.pending_normal_unload = false;
            self.unload_settle_ticks = None;
            self.servo_target = None;

            let state = self.safety.set_torque(true)?;
            self.is_torqued = true;
            let now = Instant::now();
            self.last_heartbeat = now;
            self.last_motion_frame = now;
            Ok(state)
        } else {
            if self.safety.state().is_stopped() || self.e_stop_latched.load(Ordering::SeqCst) {
                return Err(SafetyError::MotionForbiddenInFaultState(
                    "Cannot disable torque in EStop or Fault. Call clear_fault first.".into(),
                ));
            }

            if !self.is_at_safe_resting_pose() {
                return Err(SafetyError::ArmNotAtRestingPose {
                    current: self.measured_positions.clone(),
                    expected: self.config.home_positions.clone(),
                });
            }

            let state = self.safety.set_torque(false)?;
            self.active_trajectory = None;
            self.pending_normal_unload = false;
            self.unload_settle_ticks = None;
            self.servo_target = None;
            self.commanded_velocities = vec![0.0; self.config.dof];
            if self.driver.is_none() {
                self.measured_velocities = vec![0.0; self.config.dof];
            }
            if let Some(ref d) = self.driver {
                if let Ok(mut driver) = d.lock() {
                    let _ = driver.set_torque(false);
                }
            }
            self.is_torqued = false;
            Ok(state)
        }
    }

    pub fn execute_stop(&mut self, policy: StopPolicy) {
        self.apply_stop_policy(policy);
    }

    pub fn apply_stop_policy(&mut self, policy: StopPolicy) {
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        match policy {
            StopPolicy::NormalUnload => {
                // If in EStop or Fault: strictly forbid planning motion, maintain holding torque if torqued!
                if self.safety.state().is_stopped() || self.e_stop_latched.load(Ordering::SeqCst) {
                    tracing::warn!("NormalUnload rejected in EStop or Fault state; maintaining holding torque");
                    if self.is_torqued {
                        self.pending_normal_unload = false;
                        self.unload_settle_ticks = None;
                        self.active_trajectory = None;
                        self.servo_target = None;
                    }
                    return;
                }

                // Return to home position cleanly, then disable torque.
                // Allow unload from Operational or Idle.
                let can_unload = self.is_torqued && (
                    self.safety.state().is_operational()
                    || matches!(self.safety.state(), SafetyState::Idle)
                );
                if can_unload {
                    let home = self.config.home_positions.clone();
                    let clamped_start: Vec<f32> = self
                        .commanded_positions
                        .iter()
                        .enumerate()
                        .map(|(i, &p)| {
                            if i < self.config.limits.len() {
                                let lim = &self.config.limits[i];
                                p.clamp(lim.min_position, lim.max_position)
                            } else {
                                p
                            }
                        })
                        .collect();
                    if let Ok(traj) = TrajectoryPlanner::plan(
                        &clamped_start,
                        &self.commanded_velocities,
                        &home,
                        &vec![0.0; self.config.dof],
                        Some(1.5),
                        &self.config.limits,
                    ) {
                        let _ = self.safety.set_torque(true);
                        let _ = self.safety.mark_motion_started();
                        self.active_trajectory = Some(traj);
                        self.target_positions = home;
                        self.pending_normal_unload = true;
                        self.unload_settle_ticks = None;
                        let now = Instant::now();
                        self.last_heartbeat = now;
                        self.last_motion_frame = now;
                        return;
                    }
                }
                // If the arm is torqued, maintain holding torque if motion cannot be planned or arm is not operational!
                // NEVER cut power under gravity when motion planning fails or arm is in stopped/idle state!
                if self.is_torqued {
                    tracing::warn!("NormalUnload cannot plan motion or arm not operational; maintaining holding torque to prevent dropping arm");
                    self.pending_normal_unload = false;
                    self.unload_settle_ticks = None;
                    self.active_trajectory = None;
                    self.servo_target = None;
                    return;
                }
                // If not torqued, ensure state is clean
                self.pending_normal_unload = false;
                self.unload_settle_ticks = None;
                self.is_torqued = false;
                self.active_trajectory = None;
                self.commanded_velocities = vec![0.0; self.config.dof];
                if self.driver.is_none() {
                    self.measured_velocities = vec![0.0; self.config.dof];
                }
                let _ = self.safety.set_torque(false);
                if let Some(ref d) = self.driver {
                    if let Ok(mut driver) = d.lock() {
                        let _ = driver.set_torque(false);
                    }
                }
            }
            StopPolicy::EStopLatch => {
                self.e_stop_latched.store(true, Ordering::SeqCst);
                // Immediate rigid lock at current position, latch E-Stop, advance epoch
                // Maintain active restorative torque hold: do NOT cut power or set is_torqued=false
                self.pending_normal_unload = false;
                self.unload_settle_ticks = None;
                self.active_trajectory = None;
                self.servo_target = None;
                let hold_pos = if self.measured_positions.iter().all(|p| p.is_finite())
                    && self.measured_positions.len() == self.config.dof
                {
                    self.measured_positions.clone()
                } else {
                    self.commanded_positions.clone()
                };
                self.target_positions = hold_pos.clone();
                self.commanded_positions = hold_pos;
                self.commanded_velocities = vec![0.0; self.config.dof];
                if self.driver.is_none() {
                    self.measured_velocities = vec![0.0; self.config.dof];
                }
                self.safety.trigger_e_stop();
            }
            StopPolicy::ThermalTrip => {
                // Thermal trip: stop motion, disable torque immediately to protect motor coils, latch fault
                self.pending_normal_unload = false;
                self.unload_settle_ticks = None;
                self.active_trajectory = None;
                self.servo_target = None;
                self.commanded_velocities = vec![0.0; self.config.dof];
                if self.driver.is_none() {
                    self.measured_velocities = vec![0.0; self.config.dof];
                }
                self.is_torqued = false;
                let _ = self.safety.set_torque(false);
                if let Some(ref d) = self.driver {
                    if let Ok(mut driver) = d.lock() {
                        let _ = driver.set_torque(false);
                    }
                }
                let max_temp = self
                    .temperatures
                    .iter()
                    .copied()
                    .max_by(|a, b| a.total_cmp(b))
                    .unwrap_or(0.0);
                self.safety.trigger_fault(FaultReason::ThermalTrip {
                    joint_idx: 0,
                    temperature: max_temp,
                    limit: self.config.max_temperature,
                });
            }
            StopPolicy::BusLossFallback => {
                // Bus loss / disconnect: hold position locked, latch fault, advance epoch
                // Maintain holding torque: DO NOT disable hardware torque to prevent gravitational collapse!
                self.pending_normal_unload = false;
                self.unload_settle_ticks = None;
                self.active_trajectory = None;
                self.servo_target = None;
                let hold_pos = if self.measured_positions.iter().all(|p| p.is_finite())
                    && self.measured_positions.len() == self.config.dof
                {
                    self.measured_positions.clone()
                } else {
                    self.commanded_positions.clone()
                };
                self.target_positions = hold_pos.clone();
                self.commanded_positions = hold_pos;
                self.commanded_velocities = vec![0.0; self.config.dof];
                if self.driver.is_none() {
                    self.measured_velocities = vec![0.0; self.config.dof];
                }
                self.bus_loss_instant = Some(Instant::now());
                self.safety.trigger_fault(FaultReason::BusLoss);
            }
        }
    }

    pub fn clear_fault(&mut self) -> Result<SafetyState, SafetyError> {
        if self.safety.state().is_stopped() {
            let is_driver_hardware_fault = matches!(self.safety.state(), SafetyState::Fault(FaultReason::Other(_)));
            let is_thermal_trip = matches!(self.safety.state(), SafetyState::Fault(FaultReason::ThermalTrip { .. }));

            if is_driver_hardware_fault {
                // If clearing an active hardware Fault from physical motor driver, clear driver fault registers first
                if let Some(ref d) = self.driver {
                    if let Ok(mut driver) = d.lock() {
                        if let Err(e) = driver.clear_fault() {
                            tracing::error!("Failed to clear hardware driver fault: {:?}", e);
                            return Err(SafetyError::ClearFaultRejected(e.to_string()));
                        }
                    }
                }
                self.is_torqued = false;
            } else if is_thermal_trip {
                // ThermalTrip has safely de-energized motors to prevent hardware burn.
                // Keep torque disabled until operator explicitly re-enables via set_torque(true).
                self.is_torqued = false;
            }

            let new_state = self.safety.clear_fault()?;
            self.e_stop_latched.store(false, Ordering::SeqCst);

            // Slew-limited resynchronization instead of jumping if measured is displaced
            if self.measured_positions.iter().all(|p| p.is_finite())
                && self.measured_positions.len() == self.config.dof
                && self.commanded_positions.len() == self.config.dof
            {
                let dof = self.config.dof;
                for i in 0..dof {
                    let delta = (self.measured_positions[i] - self.commanded_positions[i]).clamp(-0.04, 0.04);
                    self.commanded_positions[i] += delta;
                    self.target_positions[i] = self.commanded_positions[i];
                }
            }
            self.commanded_velocities = vec![0.0; self.config.dof];
            self.last_commanded_velocities = vec![0.0; self.config.dof];
            self.active_trajectory = None;
            self.servo_target = None;
            self.servo_settled_ticks.fill(0);
            self.bus_loss_instant = None;

            // For recoverable software faults (Watchdog, BusLoss, JointLimitExceeded) or E-Stop:
            // Maintain continuous holding torque if already torqued so arm does not fall under gravity.
            let now = Instant::now();
            self.last_heartbeat = now;
            self.last_motion_frame = now;
            self.pending_normal_unload = false;
            self.unload_settle_ticks = None;
            Ok(new_state)
        } else {
            Ok(self.safety.state().clone())
        }
    }

    pub fn acquire_lease(&mut self, client_id: &str) -> Result<u64, (String, u64)> {
        let now = Instant::now();
        if let Some(ref active) = self.active_lease {
            let is_expired = active.acquired_at.elapsed() >= self.lease_timeout;
            if !is_expired && active.client_id != client_id {
                return Err((active.client_id.clone(), active.generation));
            }
        }
        self.lease_generation_counter += 1;
        let gen = self.lease_generation_counter;
        self.active_lease = Some(ActiveLease {
            client_id: client_id.to_string(),
            generation: gen,
            acquired_at: now,
            last_valid_motion_instant: None,
        });
        Ok(gen)
    }

    pub fn release_lease(&mut self, client_id: &str, generation: u64) -> bool {
        if let Some(ref active) = self.active_lease {
            if active.client_id == client_id && active.generation == generation {
                self.active_lease = None;
                return true;
            }
        }
        false
    }

    pub fn is_lease_active(&self) -> bool {
        if let Some(ref active) = self.active_lease {
            active.acquired_at.elapsed() < self.lease_timeout
        } else {
            false
        }
    }

    pub fn validate_servo_lease(&mut self, client_id: &str, generation: u64) -> bool {
        if let Some(ref mut active) = self.active_lease {
            let not_expired = active.acquired_at.elapsed() < self.lease_timeout;
            if not_expired && active.generation == generation && !client_id.is_empty() && active.client_id == client_id {
                active.last_valid_motion_instant = Some(Instant::now());
                true
            } else {
                false
            }
        } else {
            false
        }
    }

    pub fn heartbeat(&mut self, client_id: &str, epoch_id: u64) -> Result<(), SafetyError> {
        if epoch_id != self.safety.epoch_id() {
            return Err(SafetyError::EpochMismatch {
                expected: self.safety.epoch_id(),
                received: epoch_id,
            });
        }
        if let Some(ref mut active) = self.active_lease {
            if active.client_id == client_id {
                active.acquired_at = Instant::now();
                self.last_heartbeat = Instant::now();
            } else {
                tracing::debug!(
                    "Heartbeat ignored from non-lease owner '{}' (lease owner is '{}')",
                    client_id, active.client_id
                );
            }
        }
        // NOTE: When there is no active lease, do NOT refresh last_heartbeat.
        // Watchdog can only be refreshed by the authenticated active lease owner.
        Ok(())
    }

    pub fn servo(&mut self, targets: Vec<f32>, epoch_id: u64) -> Result<(), SafetyError> {
        if self.e_stop_latched.load(Ordering::SeqCst) {
            return Err(SafetyError::EStopActive);
        }

        if epoch_id != self.safety.epoch_id() {
            return Err(SafetyError::EpochMismatch {
                expected: self.safety.epoch_id(),
                received: epoch_id,
            });
        }

        if self.active_trajectory.is_some() || self.pending_normal_unload {
            return Err(SafetyError::NotArmed(self.safety.state().clone()));
        }

        // 1. Validate motion command against epoch, safety state, and soft limits
        self.safety.validate_motion_command(
            Some(epoch_id),
            &targets,
            &self.config.limits,
        )?;

        // Only renew lease and motion frame timestamp AFTER command validation succeeds
        self.last_heartbeat = Instant::now();
        self.last_motion_frame = Instant::now();
        if let Some(ref mut l) = self.active_lease {
            l.last_valid_motion_instant = Some(Instant::now());
        }

        // J7 Gripper stall release check on reverse command
        if self.config.dof >= 7 && self.gripper_stalled && targets.len() >= 7 {
            let delta = targets[6] - self.commanded_positions[6];
            let tau_7 = self.torques.get(6).copied().unwrap_or(0.0);
            if (delta * tau_7 < -1e-4) || (delta * self.gripper_stall_sign < -1e-4) {
                self.gripper_stalled = false;
                self.gripper_high_torque_ticks = 0;
                self.gripper_low_torque_ticks = 0;
            }
        }

        let _ = self.safety.mark_motion_started();

        // 2. Store servo target for tick(dt) to step with actual dt and velocity limits
        self.target_positions = targets.clone();
        self.servo_target = Some(targets);
        self.servo_settled_ticks.fill(0);

        Ok(())
    }

    /// Semi-implicit reference step: q_next = q + v_next * dt. All constraint
    /// arithmetic is f64; storage/CAN values remain f32 (and incur rounding).
    /// This bounds acceleration, not jerk. An infeasible initial state must
    /// take the fault/hold path rather than silently overriding acceleration.
    pub fn bounded_servo_step(
        q: f32,
        target: f32,
        v_prev: f32,
        dt: f64,
        limit: &crate::model::JointLimit,
    ) -> Option<(f32, f32, bool)> {
        const POS_EPS: f64 = 1.0e-3;

        if !q.is_finite() || !target.is_finite() || !v_prev.is_finite()
            || !dt.is_finite() || dt <= 0.0
            || !limit.max_velocity.is_finite() || limit.max_velocity <= 0.0
            || !limit.max_acceleration.is_finite() || limit.max_acceleration <= 0.0
            || !limit.min_position.is_finite() || !limit.max_position.is_finite()
            || limit.min_position >= limit.max_position
        {
            return None;
        }

        let lo = f64::from(limit.min_position);
        let hi = f64::from(limit.max_position);
        let q64 = f64::from(q);

        if q64 < lo - POS_EPS || q64 > hi + POS_EPS {
            return None;
        }

        let q_projected = q64.clamp(lo, hi);
        let h = dt;
        let vmax = f64::from(limit.max_velocity).min(2.0);
        let amax = f64::from(limit.max_acceleration);
        let dv = amax * h;
        let v_prev64 = f64::from(v_prev);

        if (q_projected <= lo && v_prev64 < -dv) || (q_projected >= hi && v_prev64 > dv) {
            return None;
        }

        // Rationalized positive solution of u*h + u^2/(2*a) <= distance.
        // The continuous stopping-distance term also covers variable dt.
        let safe_speed = |distance: f64| -> f64 {
            if distance <= 0.0 { return 0.0; }
            (2.0 * amax * distance
                / ((dv * dv + 2.0 * amax * distance).sqrt() + dv)).min(vmax)
        };

        let clamped_target = f64::from(target).clamp(lo, hi);
        let target_error = clamped_target - q_projected;
        let desired = if target_error.abs() <= 1.0e-5 {
            0.0
        } else {
            target_error.signum() * safe_speed(target_error.abs())
        };

        let lower = (v_prev64 - dv)
            .max(-safe_speed(q_projected - lo))
            .max(-vmax);
        let upper = (v_prev64 + dv)
            .min(safe_speed(hi - q_projected))
            .min(vmax);

        // Never collapse an empty interval to a midpoint: that would violate
        // at least one of the acceleration or stopping-distance constraints.
        if lower > upper { return None; }

        let v_next64 = desired.clamp(lower, upper);
        let q_next64 = (q_projected + v_next64 * h).clamp(lo, hi);
        let v_final64 = if h > 0.0 {
            (q_next64 - q_projected) / h
        } else {
            0.0
        };

        let error = clamped_target - q_next64;
        let reached = error.abs() <= 1.0e-4 && v_final64.abs() <= 1.5e-2;

        Some((q_next64 as f32, v_final64 as f32, reached))
    }

    /// An already infeasible state cannot satisfy both acceleration and joint
    /// limits. Latch a fault and retain holding torque; normal smoothing does
    /// not apply to this exceptional safety interruption.
    fn latch_servo_fault(&mut self, joint_idx: usize) {
        let limit = &self.config.limits[joint_idx];
        let reason = FaultReason::Other(format!(
            "Joint '{}' has an invalid or infeasible servo braking state (q={}, v={}, limits=[{}, {}], vmax={}, amax={})",
            limit.name, self.commanded_positions[joint_idx], self.commanded_velocities[joint_idx],
            limit.min_position, limit.max_position, limit.max_velocity, limit.max_acceleration,
        ));
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        self.active_trajectory = None;
        self.commanded_velocities.fill(0.0);
        self.target_positions = self.commanded_positions.clone();
        self.safety.trigger_fault(reason);
    }

    pub fn tick(&mut self, dt: f64) -> ArmObservation {
        // Immediate atomic e-stop latch check before any processing
        if self.e_stop_latched.load(Ordering::SeqCst) && !matches!(self.safety.state(), SafetyState::EStop) {
            self.apply_stop_policy(StopPolicy::EStopLatch);
        }

        // 0. Poll hardware driver telemetry if available (updates measured state only)
        if let Some(ref d) = self.driver {
            if let Ok(mut driver) = d.lock() {
                if let Ok(states) = driver.read_joint_states() {
                    if !states.is_empty() && driver.is_feedback_valid() {
                        self.last_can_feedback_instant = Some(Instant::now());
                    }
                    for (i, state) in states.iter().enumerate() {
                        if i < self.config.dof {
                            self.measured_positions[i] = state.position;
                            self.measured_velocities[i] = state.velocity;
                            self.torques[i] = state.torque;
                            self.temperatures[i] = state.temperature;
                        }
                    }
                }
            }
        }

        // 0.1 Gripper stall protection state machine for J7 (joint 6)
        if self.config.dof >= 7 {
            let is_bus_fresh = if let Some(ref d) = self.driver {
                if let Ok(driver) = d.lock() {
                    self.last_can_feedback_instant.map_or(false, |t| t.elapsed() < Duration::from_millis(50)) && driver.is_feedback_valid()
                } else {
                    false
                }
            } else {
                true // In simulation without a physical driver
            };

            let tau_7 = self.torques.get(6).copied().unwrap_or(0.0);
            if !is_bus_fresh {
                self.gripper_high_torque_ticks = 0;
                self.gripper_low_torque_ticks = 0;
            } else if !self.gripper_stalled {
                if tau_7.abs() >= 0.30 {
                    self.gripper_high_torque_ticks += 1;
                    if self.gripper_high_torque_ticks >= 6 {
                        self.gripper_stalled = true;
                        self.gripper_stall_sign = if tau_7 >= 0.0 { 1.0 } else { -1.0 };
                        self.gripper_low_torque_ticks = 0;
                        let hold = self.measured_positions[6];
                        self.commanded_positions[6] = hold;
                        self.target_positions[6] = hold;
                        if let Some(ref mut st) = self.servo_target {
                            if st.len() > 6 {
                                st[6] = hold;
                            }
                        }
                    }
                } else {
                    self.gripper_high_torque_ticks = 0;
                }
            } else {
                self.gripper_high_torque_ticks = 0;
                // Exit condition B: |tau_7| < 0.15 for 6 consecutive ticks
                if tau_7.abs() < 0.15 {
                    self.gripper_low_torque_ticks += 1;
                    if self.gripper_low_torque_ticks >= 6 {
                        self.gripper_stalled = false;
                        self.gripper_low_torque_ticks = 0;
                    }
                } else {
                    self.gripper_low_torque_ticks = 0;
                }
            }
        }

        // 0.2 Compute gravity torques if gravity compensation is enabled, driver feedback is valid, and measured positions available
        let driver_feedback_ok = self
            .driver
            .as_ref()
            .map_or(true, |d| d.lock().map_or(false, |drv| drv.is_feedback_valid()));

        if self.gravity_compensated && driver_feedback_ok && self.measured_positions.len() >= 6 {
            let mut q = [0.0f32; 6];
            q.copy_from_slice(&self.measured_positions[0..6]);
            match crate::gravity::compute_gravity_torques(&q) {
                Ok(tau) => {
                    let all_finite = tau.iter().all(|t| t.is_finite());
                    if all_finite {
                        self.gravity_torques = tau.to_vec();
                    } else {
                        tracing::warn!("Gravity torque contains non-finite values, zeroing out");
                        self.gravity_torques.clear();
                    }
                }
                Err(e) => {
                    tracing::warn!("Gravity torque computation error: {:?}", e);
                    self.gravity_torques.clear();
                }
            }
        } else {
            self.gravity_torques.clear();
        }

        // Apply Slew-Rate Limiter (15.0 N·m/s) and safety limits to gravity torques
        if self.gravity_torques_ramped.len() != self.config.dof {
            self.gravity_torques_ramped = vec![0.0; self.config.dof];
        }
        let dt_sec = (dt as f32).clamp(0.001, 0.1);
        let max_dtau = 15.0 * dt_sec; // Slew rate: 15.0 N·m/s (0.30 N·m/tick at 50Hz)
        for i in 0..self.config.dof {
            let target_tau = if self.gravity_compensated && driver_feedback_ok && i < self.gravity_torques.len() {
                let raw = self.gravity_torques[i];
                let raw_finite = if raw.is_finite() { raw } else { 0.0 };
                let scaled = raw_finite * self.gravity_scale;
                let limit = if i < 3 { 12.0 } else { 6.0 };
                scaled.clamp(-limit, limit)
            } else {
                0.0
            };
            let diff = target_tau - self.gravity_torques_ramped[i];
            let ramped = self.gravity_torques_ramped[i] + diff.clamp(-max_dtau, max_dtau);
            self.gravity_torques_ramped[i] = if ramped.is_finite() { ramped } else { 0.0 };
        }

        // Check telemetry freshness: trigger BusLossFallback if communication lost or feedback stale (>80ms)
        if self.is_torqued
            && !self.safety.state().is_stopped()
            && self
                .driver
                .as_ref()
                .map_or(false, |d| d.lock().map_or(true, |drv| !drv.is_feedback_valid()))
        {
            self.execute_stop(StopPolicy::BusLossFallback);
        }

        // Check hardware motor faults / error flags from feedback
        if self.is_torqued && !self.safety.state().is_stopped() {
            if let Some(ref d) = self.driver {
                if let Ok(driver) = d.lock() {
                    if let Err(e) = driver.check_hardware_status() {
                        tracing::error!("Hardware fault detected on driver: {}", e);
                        let hold_pos = if self.measured_positions.iter().all(|p| p.is_finite())
                            && self.measured_positions.len() == self.config.dof
                        {
                            self.measured_positions.clone()
                        } else {
                            self.commanded_positions.clone()
                        };
                        self.pending_normal_unload = false;
                        self.unload_settle_ticks = None;
                        self.active_trajectory = None;
                        self.servo_target = None; self.servo_settled_ticks.fill(0);
                        self.target_positions = hold_pos.clone();
                        self.commanded_positions = hold_pos;
                        self.commanded_velocities = vec![0.0; self.config.dof];
                        self.safety.trigger_fault(FaultReason::Other(e.to_string()));
                    }
                }
            }
        }

        // 1. Check lease watchdog (skip during autonomous stow/unload)
        if self.lease_enabled
            && self.is_torqued
            && self.safety.state().is_operational()
            && !self.pending_normal_unload
            && self.last_heartbeat.elapsed() > self.lease_timeout
        {
            // Watchdog expired: rigid lock at current measured/commanded position, latch fault
            // Maintain torque hold: do NOT cut power or disable torque
            let hold_pos = if self.measured_positions.iter().all(|p| p.is_finite())
                && self.measured_positions.len() == self.config.dof
            {
                self.measured_positions.clone()
            } else {
                self.commanded_positions.clone()
            };
            self.pending_normal_unload = false;
            self.unload_settle_ticks = None;
            self.active_trajectory = None;
            self.servo_target = None; self.servo_settled_ticks.fill(0);
            self.target_positions = hold_pos.clone();
            self.commanded_positions = hold_pos;
            self.commanded_velocities = vec![0.0; self.config.dof];
            self.safety.trigger_fault(FaultReason::WatchdogTimeout);
        }

        // 2. Check thermal limit (only check when torqued and not already stopped to prevent reentrant loop)
        if self.is_torqued && !self.safety.state().is_stopped() {
            for (i, &temp) in self.temperatures.iter().enumerate() {
                if temp.is_finite() && temp > self.config.max_temperature {
                    tracing::error!(
                        "Thermal limit exceeded on joint {} (temp={:.1}°C > max={:.1}°C); de-energizing arm immediately!",
                        i, temp, self.config.max_temperature
                    );
                    self.pending_normal_unload = false;
                    self.unload_settle_ticks = None;
                    self.active_trajectory = None;
                    self.servo_target = None; self.servo_settled_ticks.fill(0);
                    self.commanded_velocities = vec![0.0; self.config.dof];
                    if self.driver.is_none() {
                        self.measured_velocities = vec![0.0; self.config.dof];
                    }
                    self.is_torqued = false;
                    let _ = self.safety.set_torque(false);
                    if let Some(ref d) = self.driver {
                        if let Ok(mut driver) = d.lock() {
                            let _ = driver.set_torque(false);
                        }
                    }
                    self.safety.trigger_fault(FaultReason::ThermalTrip {
                        joint_idx: i,
                        temperature: temp,
                        limit: self.config.max_temperature,
                    });
                    break;
                }
            }
        }

        // Use one integration interval and never integrate a reference twice in a tick.
        let servo_dt = dt.clamp(0.001, 0.1);
        let mut reference_stepped = false;

        // 3. Step trajectory if active, or step servo target
        if self.pending_normal_unload {
            self.last_heartbeat = Instant::now();
        }
        if let Some(ref mut traj) = self.active_trajectory {
            reference_stepped = true;
            let point = traj.step(dt);
            self.commanded_positions = point.positions;
            self.commanded_velocities = point.velocities;

            if traj.is_finished() {
                self.active_trajectory = None;
                self.commanded_velocities = vec![0.0; self.config.dof];
                if !self.pending_normal_unload {
                    self.safety.mark_motion_finished();
                }
            }
        }

        // 3.1 Handle pending normal unload settling phase
        if self.pending_normal_unload && self.active_trajectory.is_none() {
            let at_home_and_still = self.is_at_safe_resting_pose();

            let ticks = self.unload_settle_ticks.get_or_insert(40); // 40 ticks = 800ms
            if at_home_and_still {
                if *ticks > 0 {
                    *ticks -= 1;
                } else {
                    self.unload_settle_ticks = None;
                    self.pending_normal_unload = false;
                    tracing::info!("Arm settled cleanly at home position for 800ms; safely cutting torque to Standby/Idle.");
                    if let Err(e) = self.set_torque(false) {
                        tracing::warn!("Failed to cut torque after normal unload settle: {}", e);
                        self.safety.mark_motion_finished();
                    }
                }
            } else {
                // If not yet stably at rest within tolerances, reset settle tick window and clamp targets
                *ticks = 40;
                self.commanded_positions = self.config.home_positions.clone();
                self.target_positions = self.config.home_positions.clone();
                self.commanded_velocities = vec![0.0; self.config.dof];
            }
        } else if let Some(ref target) = self.servo_target.clone() {
            // Check continuous motion frame timeout:
            // If > 200ms without a new pose frame, clear servo_target to begin controlled smooth deceleration
            if self.last_motion_frame.elapsed() > self.motion_frame_timeout {
                self.servo_target = None;
                self.servo_settled_ticks.fill(0);
            } else {
                // Check if any target joint has instantaneous jump > 0.25 rad relative to commanded_positions
                let has_dirty_jump = target.iter().enumerate().any(|(i, &tgt)| {
                    if i < self.commanded_positions.len() {
                        let jump = (tgt - self.commanded_positions[i]).abs();
                        jump > 0.25
                    } else {
                        false
                    }
                });

                if has_dirty_jump {
                    tracing::warn!(
                        "Discarding dirty servo target frame: instantaneous jump exceeds 0.25 rad relative to commanded_positions"
                    );
                    self.servo_target = None;
                    self.servo_settled_ticks.fill(0);
                } else {
                    reference_stepped = true;
                    let mut reached_flags = vec![false; self.config.dof];
                    let mut next_positions = self.commanded_positions.clone();
                    let mut next_velocities = self.commanded_velocities.clone();
                    let mut infeasible = None;
                    for (i, &tgt) in target.iter().enumerate() {
                        if i < self.config.dof {
                            let limit = &self.config.limits[i];
                            let clamped_tgt = tgt.clamp(limit.min_position, limit.max_position);
                            let actual_tgt = if i == 6 && self.gripper_stalled {
                                self.commanded_positions[6]
                            } else {
                                clamped_tgt
                            };
                            let step = Self::bounded_servo_step(
                                next_positions[i],
                                actual_tgt,
                                next_velocities[i],
                                servo_dt,
                                limit,
                            );
                            let Some((q_next, v_next, reached)) = step else {
                                infeasible = Some(i);
                                break;
                            };
                            next_positions[i] = q_next;
                            next_velocities[i] = v_next;
                            reached_flags[i] = reached;
                        }
                    }
                    if let Some(i) = infeasible {
                        self.latch_servo_fault(i);
                    } else {
                        self.commanded_positions = next_positions;
                        self.commanded_velocities = next_velocities;
                        self.target_positions = self.commanded_positions.clone();

                        // Update hysteresis counter for each axis: if reached, increment; if not reached, reset to 0
                        for i in 0..self.config.dof {
                            if reached_flags[i] {
                                self.servo_settled_ticks[i] = self.servo_settled_ticks[i].saturating_add(1);
                            } else {
                                self.servo_settled_ticks[i] = 0;
                            }
                        }

                        // If all axes have servo_settled_ticks[i] >= 2, mark motion finished
                        if self.servo_settled_ticks.iter().all(|&ticks| ticks >= 2) {
                            self.servo_target = None;
                            self.safety.mark_motion_finished();
                        }
                    }
                }
            }
        }

        if !reference_stepped
            && self.servo_target.is_none()
            && self.active_trajectory.is_none()
            && !self.pending_normal_unload
            && !self.safety.state().is_stopped()
        {
            // A stale/dropped target requests zero velocity. The same feasible
            // interval governs braking, with no small-velocity truncation.
            let mut next_positions = self.commanded_positions.clone();
            let mut next_velocities = self.commanded_velocities.clone();
            let mut infeasible = None;
            for i in 0..self.config.dof {
                let step = Self::bounded_servo_step(
                    self.commanded_positions[i],
                    self.commanded_positions[i],
                    self.commanded_velocities[i],
                    servo_dt,
                    &self.config.limits[i],
                );
                let Some((q_next, v_next, _)) = step else {
                    infeasible = Some(i);
                    break;
                };
                next_positions[i] = q_next;
                next_velocities[i] = v_next;
            }
            if let Some(i) = infeasible {
                self.latch_servo_fault(i);
            } else {
                self.commanded_positions = next_positions;
                self.commanded_velocities = next_velocities;
                self.target_positions = self.commanded_positions.clone();
                if self.commanded_velocities.iter().all(|&v| v == 0.0) {
                    self.safety.mark_motion_finished();
                }
            }
        }

        self.last_commanded_velocities = self.commanded_velocities.clone();

        // 4. Output commanded positions to hardware driver if torqued (generates anti-gravity restorative torque)
        // Continuously output whenever is_torqued is true, maintaining rigid hold even during E-Stop, BusLossFallback or WatchdogTimeout
        if self.is_torqued {
            let send_to_bus = true;

            if send_to_bus {
                if let Some(ref d) = self.driver {
                    if let Ok(mut driver) = d.lock() {
                        // Write commanded holding targets whenever driver is enabled
                        if driver.is_enabled() {
                            let targets: Vec<JointTarget> = self
                                .commanded_positions
                                .iter()
                                .zip(self.commanded_velocities.iter())
                                .enumerate()
                                .map(|(i, (&p, &v))| JointTarget {
                                    position: p,
                                    velocity: Some(v),
                                    torque: if self.gravity_compensated {
                                        self.gravity_torques_ramped.get(i).copied()
                                    } else {
                                        Some(0.0)
                                    },
                                    kp: None,
                                    kd: None,
                                })
                                .collect();
                            let _ = driver.write_joint_targets(&targets);
                        }
                    }
                }
            }
        }

        // If in simulation (no hardware driver attached), mirror commanded into measured
        if self.driver.is_none() {
            self.measured_positions = self.commanded_positions.clone();
            self.measured_velocities = self.commanded_velocities.clone();
        }

        let obs = self.current_observation();

        // 5. Record frame if episode recording is active
        if self.recorder.state() == &EpisodeState::Recording {
            self.recorder.append_frame(RecordFrame {
                timestamp: obs.timestamp,
                observation_state: self.measured_positions.clone(),
                action: self.commanded_positions.clone(),
            });
        }

        obs
    }
}

/// Thread-safe Arm Supervisor managing the robotic arm daemon
pub struct ArmSupervisor {
    inner: Arc<RwLock<SupervisorInner>>,
    telemetry_tx: broadcast::Sender<ArmObservation>,
    pub e_stop_latched: Arc<AtomicBool>,
}

impl ArmSupervisor {
    pub fn new(config: ArmConfig) -> Self {
        let (tx, _) = broadcast::channel(128);
        let e_stop_latched = Arc::new(AtomicBool::new(false));
        let mut inner = SupervisorInner::new(config);
        inner.e_stop_latched = e_stop_latched.clone();
        Self {
            inner: Arc::new(RwLock::new(inner)),
            telemetry_tx: tx,
            e_stop_latched,
        }
    }

    pub fn subscribe_telemetry(&self) -> broadcast::Receiver<ArmObservation> {
        self.telemetry_tx.subscribe()
    }

    #[doc(hidden)]
    pub fn inner(&self) -> &Arc<RwLock<SupervisorInner>> {
        &self.inner
    }

    pub async fn get_observation(&self) -> ArmObservation {
        let inner = self.inner.read().await;
        inner.current_observation()
    }

    pub async fn get_schema(&self) -> ObservationSchema {
        let inner = self.inner.read().await;
        ObservationSchema::from_config(&inner.config)
    }

    pub async fn get_config(&self) -> ArmConfig {
        let inner = self.inner.read().await;
        inner.config.clone()
    }

    pub async fn set_driver(&self, driver: Box<dyn ArmDriver>) {
        let mut inner = self.inner.write().await;
        let mut drv = driver;
        if let Ok(states) = drv.read_joint_states() {
            if !states.is_empty() && drv.is_feedback_valid() {
                for (i, s) in states.iter().enumerate().take(inner.config.dof) {
                    if s.position.is_finite() {
                        inner.measured_positions[i] = s.position;
                        inner.commanded_positions[i] = s.position;
                        inner.target_positions[i] = s.position;
                    }
                }
            }
        }
        inner.driver = Some(std::sync::Mutex::new(drv));
    }

    pub fn with_driver(self, driver: Box<dyn ArmDriver>) -> Self {
        if let Ok(mut inner) = self.inner.try_write() {
            inner.driver = Some(std::sync::Mutex::new(driver));
        }
        self
    }

    pub async fn set_torque(&self, enable: bool) -> Result<SafetyState, ArmError> {
        if enable && self.e_stop_latched.load(Ordering::SeqCst) {
            return Err(ArmError::Safety(SafetyError::EStopActive));
        }
        let mut inner = self.inner.write().await;
        // Post-lock double-check: verify atomic e-stop latch again after acquiring write lock
        if enable && self.e_stop_latched.load(Ordering::SeqCst) {
            return Err(ArmError::Safety(SafetyError::EStopActive));
        }
        let state = inner.set_torque(enable)?;
        Ok(state)
    }

    pub async fn heartbeat(&self, client_id: &str, epoch_id: u64) -> Result<(), SafetyError> {
        let mut inner = self.inner.write().await;
        inner.heartbeat(client_id, epoch_id)
    }

    pub async fn acquire_lease(&self, client_id: &str) -> Result<u64, (String, u64)> {
        let mut inner = self.inner.write().await;
        inner.acquire_lease(client_id)
    }

    pub async fn release_lease(&self, client_id: &str, generation: u64) -> bool {
        let mut inner = self.inner.write().await;
        inner.release_lease(client_id, generation)
    }

    pub async fn is_lease_active(&self) -> bool {
        let inner = self.inner.read().await;
        inner.is_lease_active()
    }

    pub async fn validate_servo_lease(&self, client_id: &str, generation: u64) -> bool {
        let mut inner = self.inner.write().await;
        inner.validate_servo_lease(client_id, generation)
    }

    pub async fn set_torques(&self, torques: Vec<f32>) {
        let mut inner = self.inner.write().await;
        inner.torques = torques;
    }

    pub async fn is_gripper_stalled(&self) -> bool {
        let inner = self.inner.read().await;
        inner.gripper_stalled
    }

    pub async fn is_gravity_compensated(&self) -> bool {
        let inner = self.inner.read().await;
        inner.gravity_compensated
    }

    pub async fn get_gravity_torques(&self) -> Vec<f32> {
        let inner = self.inner.read().await;
        inner.gravity_torques.clone()
    }

    pub async fn set_gravity_compensated(&self, enabled: bool) {
        let mut inner = self.inner.write().await;
        inner.gravity_compensated = enabled;
    }

    pub async fn get_gravity_scale(&self) -> f32 {
        let inner = self.inner.read().await;
        inner.gravity_scale
    }

    pub async fn set_gravity_scale(&self, scale: f32) {
        let mut inner = self.inner.write().await;
        inner.gravity_scale = scale.clamp(0.0, 1.0);
    }

    pub async fn get_gravity_torques_ramped(&self) -> Vec<f32> {
        let inner = self.inner.read().await;
        inner.gravity_torques_ramped.clone()
    }

    pub async fn servo(&self, targets: Vec<f32>, epoch_id: u64) -> Result<(), ArmError> {
        if self.e_stop_latched.load(Ordering::SeqCst) {
            return Err(ArmError::Safety(SafetyError::EStopActive));
        }
        let mut inner = self.inner.write().await;
        inner.servo(targets, epoch_id)?;
        Ok(())
    }

    pub async fn execute_stop(&self, policy: StopPolicy) {
        let mut inner = self.inner.write().await;
        inner.execute_stop(policy);
    }

    pub async fn set_lease_enabled(&self, enabled: bool) {
        let mut inner = self.inner.write().await;
        inner.lease_enabled = enabled;
    }

    pub async fn set_temperatures(&self, temps: Vec<f32>) {
        let mut inner = self.inner.write().await;
        inner.temperatures = temps;
    }

    pub async fn jog(
        &self,
        target_positions: Vec<f32>,
        duration_secs: Option<f64>,
        epoch_id: Option<u64>,
    ) -> Result<u64, ArmError> {
        if self.e_stop_latched.load(Ordering::SeqCst) {
            return Err(ArmError::Safety(SafetyError::EStopActive));
        }
        let mut inner = self.inner.write().await;

        if inner.pending_normal_unload {
            return Err(ArmError::Safety(SafetyError::NotArmed(inner.safety.state().clone())));
        }
        inner.pending_normal_unload = false;

        // 1. Validate command against single controller authority, safety state, and soft limits
        inner.safety.validate_motion_command(
            epoch_id,
            &target_positions,
            &inner.config.limits,
        )?;

        // J7 Gripper stall release check on reverse command
        if inner.config.dof >= 7 && inner.gripper_stalled && target_positions.len() >= 7 {
            let delta = target_positions[6] - inner.commanded_positions[6];
            let tau_7 = inner.torques.get(6).copied().unwrap_or(0.0);
            if (delta * tau_7 < -1e-4) || (delta * inner.gripper_stall_sign < -1e-4) {
                inner.gripper_stalled = false;
                inner.gripper_high_torque_ticks = 0;
                inner.gripper_low_torque_ticks = 0;
            }
        }

        // 2. Plan quintic polynomial trajectory from commanded state to target
        let traj = TrajectoryPlanner::plan(
            &inner.commanded_positions,
            &inner.commanded_velocities,
            &target_positions,
            &vec![0.0; inner.config.dof],
            duration_secs,
            &inner.config.limits,
        )?;

        // 3. Only renew lease AFTER command validation and planning succeed
        inner.last_heartbeat = Instant::now();

        inner.servo_target = None;
        inner.target_positions = target_positions;
        inner.active_trajectory = Some(traj);
        inner.safety.mark_motion_started()?;

        Ok(inner.safety.epoch_id())
    }

    pub async fn home(
        &self,
        duration_secs: Option<f64>,
        epoch_id: Option<u64>,
    ) -> Result<u64, ArmError> {
        let home_pos = {
            let inner = self.inner.read().await;
            inner.config.home_positions.clone()
        };
        self.jog(home_pos, duration_secs, epoch_id).await
    }

    pub async fn is_at_safe_resting_pose(&self) -> bool {
        let inner = self.inner.read().await;
        inner.is_at_safe_resting_pose()
    }

    pub async fn trigger_e_stop(&self) -> (SafetyState, u64) {
        self.e_stop_latched.store(true, Ordering::SeqCst);
        let mut inner = self.inner.write().await;
        inner.e_stop_latched.store(true, Ordering::SeqCst);
        inner.apply_stop_policy(StopPolicy::EStopLatch);
        (inner.safety.state().clone(), inner.safety.epoch_id())
    }

    pub async fn clear_fault(&self) -> Result<SafetyState, ArmError> {
        let mut inner = self.inner.write().await;
        let new_state = inner.clear_fault()?;
        self.e_stop_latched.store(false, Ordering::SeqCst);
        Ok(new_state)
    }

    pub async fn apply_stop_policy(&self, policy: StopPolicy) {
        let mut inner = self.inner.write().await;
        inner.apply_stop_policy(policy);
    }

    pub async fn is_unloading(&self) -> bool {
        let inner = self.inner.read().await;
        inner.pending_normal_unload || inner.active_trajectory.is_some() || inner.unload_settle_ticks.is_some()
    }

    pub async fn has_active_trajectory(&self) -> bool {
        let inner = self.inner.read().await;
        inner.active_trajectory.is_some()
    }

    pub async fn start_recording(
        &self,
        episode_id: Option<String>,
        output_path: Option<PathBuf>,
    ) -> Result<(), ArmError> {
        let mut inner = self.inner.write().await;
        inner.recorder.start(episode_id, output_path)?;
        Ok(())
    }

    pub async fn save_recording(&self) -> Result<PathBuf, ArmError> {
        let mut inner = self.inner.write().await;
        let path = inner.recorder.save()?;
        Ok(path)
    }

    pub async fn discard_recording(&self) -> Result<(), ArmError> {
        let mut inner = self.inner.write().await;
        inner.recorder.discard()?;
        Ok(())
    }

    pub async fn get_recording_state(&self) -> (EpisodeState, usize) {
        let inner = self.inner.read().await;
        (inner.recorder.state().clone(), inner.recorder.frame_count())
    }

    /// Execute a control cycle step
    pub async fn tick(&self, dt: f64) -> ArmObservation {
        let obs = {
            let mut inner = self.inner.write().await;
            inner.tick(dt)
        };
        // Broadcast observation to any active WebSocket listeners
        let _ = self.telemetry_tx.send(obs.clone());
        obs
    }
}
