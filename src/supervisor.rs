use std::path::PathBuf;
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

/// Internal mutable state of the robotic arm supervisor
pub struct SupervisorInner {
    pub config: ArmConfig,
    pub safety: SafetyManager,
    pub measured_positions: Vec<f32>,
    pub measured_velocities: Vec<f32>,
    pub commanded_positions: Vec<f32>,
    pub commanded_velocities: Vec<f32>,
    pub torques: Vec<f32>,
    pub temperatures: Vec<f32>,
    pub target_positions: Vec<f32>,
    pub servo_target: Option<Vec<f32>>,
    pub is_torqued: bool,
    pub active_trajectory: Option<MultiJointQuinticTrajectory>,
    pub last_heartbeat: Instant,
    pub lease_timeout: Duration,
    pub lease_enabled: bool,
    pub pending_normal_unload: bool,
    pub recorder: EpisodeRecorder,
    pub start_time: Instant,
    pub driver: Option<std::sync::Mutex<Box<dyn ArmDriver>>>,
}

impl SupervisorInner {
    pub fn new(config: ArmConfig) -> Self {
        let dof = config.dof;
        let positions = config.home_positions.clone();
        let target_positions = positions.clone();
        let lease_timeout = Duration::from_millis(config.lease_timeout_ms);

        Self {
            config,
            safety: SafetyManager::new(),
            measured_positions: positions.clone(),
            measured_velocities: vec![0.0; dof],
            commanded_positions: positions,
            commanded_velocities: vec![0.0; dof],
            torques: vec![0.0; dof],
            temperatures: vec![25.0; dof],
            target_positions,
            servo_target: None,
            is_torqued: false,
            active_trajectory: None,
            last_heartbeat: Instant::now(),
            lease_timeout,
            lease_enabled: true,
            pending_normal_unload: false,
            recorder: EpisodeRecorder::new(),
            start_time: Instant::now(),
            driver: None,
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
        }
    }

    pub fn set_torque(&mut self, enable: bool) -> Result<SafetyState, SafetyError> {
        if enable {
            // Strict pre-check: If stopped (E-Stop or Fault), reject IMMEDIATELY before touching driver!
            if self.safety.state().is_stopped() {
                if let SafetyState::Fault(ref reason) = self.safety.state() {
                    return Err(SafetyError::FaultActive(reason.clone()));
                }
                return Err(SafetyError::EStopActive);
            }

            // Read valid hardware telemetry first if driver is mounted; MUST verify physical feedback before enabling
            if let Some(ref d) = self.driver {
                let mut driver = d.lock().map_err(|_| SafetyError::FeedbackUnavailable)?;
                let states = driver.read_joint_states().map_err(|_| SafetyError::FeedbackUnavailable)?;
                if !driver.is_feedback_valid() || states.len() < self.config.dof {
                    return Err(SafetyError::FeedbackUnavailable);
                }
                for (i, state) in states.iter().enumerate() {
                    if i < self.config.dof {
                        self.measured_positions[i] = state.position;
                        self.measured_velocities[i] = state.velocity;
                        self.torques[i] = state.torque;
                        self.temperatures[i] = state.temperature;
                    }
                }
                // Send hardware enable frame only AFTER physical angles verified; abort & rollback on error
                if let Err(_) = driver.set_torque(true) {
                    let _ = driver.set_torque(false);
                    self.is_torqued = false;
                    self.safety.trigger_fault(FaultReason::Other("Hardware torque enable failed".to_string()));
                    return Err(SafetyError::FeedbackUnavailable);
                }
            }

            // Initialize commanded_positions to verified measured_positions to prevent sudden jumps
            self.commanded_positions = self.measured_positions.clone();
            self.target_positions = self.measured_positions.clone();
            self.commanded_velocities = vec![0.0; self.config.dof];
            self.measured_velocities = vec![0.0; self.config.dof];
            self.active_trajectory = None;
            self.pending_normal_unload = false;
            self.servo_target = None;

            let state = self.safety.set_torque(true)?;
            self.is_torqued = true;
            self.last_heartbeat = Instant::now();
            Ok(state)
        } else {
            self.active_trajectory = None;
            self.pending_normal_unload = false;
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
            let state = self.safety.set_torque(false)?;
            self.is_torqued = false;
            Ok(state)
        }
    }

    pub fn execute_stop(&mut self, policy: StopPolicy) {
        self.apply_stop_policy(policy);
    }

    pub fn apply_stop_policy(&mut self, policy: StopPolicy) {
        self.servo_target = None;
        match policy {
            StopPolicy::NormalUnload => {
                // Return to home position cleanly, then disable torque
                if self.is_torqued && self.safety.state().is_operational() {
                    let home = self.config.home_positions.clone();
                    if let Ok(traj) = TrajectoryPlanner::plan(
                        &self.commanded_positions,
                        &self.commanded_velocities,
                        &home,
                        &vec![0.0; self.config.dof],
                        Some(1.5),
                        &self.config.limits,
                    ) {
                        self.active_trajectory = Some(traj);
                        self.target_positions = home;
                        self.pending_normal_unload = true;
                        let _ = self.safety.mark_motion_started();
                        return;
                    }
                }
                // If not operational or planning failed, directly disable torque
                self.pending_normal_unload = false;
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
                // Immediate rigid lock at current position, latch E-Stop, advance epoch
                self.pending_normal_unload = false;
                self.is_torqued = false;
                self.active_trajectory = None;
                self.target_positions = self.commanded_positions.clone();
                self.commanded_velocities = vec![0.0; self.config.dof];
                if self.driver.is_none() {
                    self.measured_velocities = vec![0.0; self.config.dof];
                }
                self.safety.trigger_e_stop();
                if let Some(ref d) = self.driver {
                    if let Ok(mut driver) = d.lock() {
                        let _ = driver.set_torque(false);
                    }
                }
            }
            StopPolicy::ThermalTrip => {
                // Thermal trip: stop motion, latch fault, advance epoch
                self.pending_normal_unload = false;
                self.is_torqued = false;
                self.active_trajectory = None;
                self.target_positions = self.commanded_positions.clone();
                self.commanded_velocities = vec![0.0; self.config.dof];
                if self.driver.is_none() {
                    self.measured_velocities = vec![0.0; self.config.dof];
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
                if let Some(ref d) = self.driver {
                    if let Ok(mut driver) = d.lock() {
                        let _ = driver.set_torque(false);
                    }
                }
            }
            StopPolicy::BusLossFallback => {
                // Bus loss / disconnect: hold position locked, latch fault, advance epoch
                self.pending_normal_unload = false;
                self.is_torqued = false;
                self.active_trajectory = None;
                self.target_positions = self.commanded_positions.clone();
                self.commanded_velocities = vec![0.0; self.config.dof];
                if self.driver.is_none() {
                    self.measured_velocities = vec![0.0; self.config.dof];
                }
                self.safety.trigger_fault(FaultReason::BusLoss);
                if let Some(ref d) = self.driver {
                    if let Ok(mut driver) = d.lock() {
                        let _ = driver.set_torque(false);
                    }
                }
            }
        }
    }

    pub fn heartbeat(&mut self, _client_id: &str, epoch_id: u64) -> Result<(), SafetyError> {
        if epoch_id != self.safety.epoch_id() {
            return Err(SafetyError::EpochMismatch {
                expected: self.safety.epoch_id(),
                received: epoch_id,
            });
        }
        self.last_heartbeat = Instant::now();
        Ok(())
    }

    pub fn servo(&mut self, targets: Vec<f32>, epoch_id: u64) -> Result<(), SafetyError> {
        if epoch_id != self.safety.epoch_id() {
            return Err(SafetyError::EpochMismatch {
                expected: self.safety.epoch_id(),
                received: epoch_id,
            });
        }

        // 1. Validate motion command against epoch, safety state, and soft limits
        self.safety.validate_motion_command(
            Some(epoch_id),
            &targets,
            &self.config.limits,
        )?;

        // Only renew lease AFTER command validation succeeds
        self.last_heartbeat = Instant::now();

        // 2. Clear any active quintic trajectory & unload state
        self.active_trajectory = None;
        self.pending_normal_unload = false;
        let _ = self.safety.mark_motion_started();

        // 3. Store servo target for tick(dt) to step with actual dt and velocity limits
        self.target_positions = targets.clone();
        self.servo_target = Some(targets);

        Ok(())
    }

    pub fn tick(&mut self, dt: f64) -> ArmObservation {
        // 0. Poll hardware driver telemetry if available (updates measured state only)
        if let Some(ref d) = self.driver {
            if let Ok(mut driver) = d.lock() {
                if let Ok(states) = driver.read_joint_states() {
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

        // Check telemetry freshness: trigger BusLossFallback if communication lost or feedback stale (>500ms)
        if self.is_torqued
            && self
                .driver
                .as_ref()
                .map_or(false, |d| d.lock().map_or(true, |drv| !drv.is_feedback_valid()))
        {
            self.execute_stop(StopPolicy::BusLossFallback);
        }

        // 1. Check lease watchdog
        if self.lease_enabled
            && self.is_torqued
            && self.safety.state().is_operational()
            && self.last_heartbeat.elapsed() > self.lease_timeout
        {
            // Watchdog expired: smooth deceleration / lock at current position
            self.pending_normal_unload = false;
            self.is_torqued = false;
            self.active_trajectory = None;
            self.servo_target = None;
            self.target_positions = self.commanded_positions.clone();
            self.commanded_velocities = vec![0.0; self.config.dof];
            self.safety.trigger_fault(FaultReason::WatchdogTimeout);
            if let Some(ref d) = self.driver {
                if let Ok(mut driver) = d.lock() {
                    let _ = driver.set_torque(false);
                }
            }
        }

        // 2. Check thermal limit
        for (i, &temp) in self.temperatures.iter().enumerate() {
            if temp.is_finite() && temp > self.config.max_temperature {
                self.pending_normal_unload = false;
                self.is_torqued = false;
                self.active_trajectory = None;
                self.servo_target = None;
                self.target_positions = self.commanded_positions.clone();
                self.commanded_velocities = vec![0.0; self.config.dof];
                self.safety.trigger_fault(FaultReason::ThermalTrip {
                    joint_idx: i,
                    temperature: temp,
                    limit: self.config.max_temperature,
                });
                if let Some(ref d) = self.driver {
                    if let Ok(mut driver) = d.lock() {
                        let _ = driver.set_torque(false);
                    }
                }
                break;
            }
        }

        // 3. Step trajectory if active, or step servo target
        if let Some(ref mut traj) = self.active_trajectory {
            let point = traj.step(dt);
            self.commanded_positions = point.positions;
            self.commanded_velocities = point.velocities;

            if traj.is_finished() {
                self.active_trajectory = None;
                self.commanded_velocities = vec![0.0; self.config.dof];
                if self.pending_normal_unload {
                    self.pending_normal_unload = false;
                    self.is_torqued = false;
                    let _ = self.safety.set_torque(false);
                    if let Some(ref d) = self.driver {
                        if let Ok(mut driver) = d.lock() {
                            let _ = driver.set_torque(false);
                        }
                    }
                } else {
                    self.safety.mark_motion_finished();
                }
            }
        } else if let Some(ref target) = self.servo_target.clone() {
            let dt_f32 = dt as f32;
            let mut all_reached = true;
            for (i, &tgt) in target.iter().enumerate() {
                if i < self.config.dof {
                    let limit = &self.config.limits[i];
                    let clamped_tgt = tgt.clamp(limit.min_position, limit.max_position);
                    let max_step = limit.max_velocity * dt_f32;
                    let current = self.commanded_positions[i];
                    let delta = clamped_tgt - current;
                    if delta.abs() > max_step {
                        all_reached = false;
                        let step = if delta > 0.0 { max_step } else { -max_step };
                        self.commanded_positions[i] = current + step;
                        self.commanded_velocities[i] = if dt_f32 > 0.0 { step / dt_f32 } else { 0.0 };
                    } else {
                        self.commanded_positions[i] = clamped_tgt;
                        self.commanded_velocities[i] = if dt_f32 > 0.0 { delta / dt_f32 } else { 0.0 };
                    }
                    self.target_positions[i] = clamped_tgt;
                }
            }
            if all_reached {
                self.servo_target = None;
                self.commanded_velocities = vec![0.0; self.config.dof];
                self.safety.mark_motion_finished();
            }
        } else {
            // Trajectory/servo not active (holding phase):
            // commanded_positions stays strictly locked at target_positions, commanded_velocities is 0
            self.commanded_velocities = vec![0.0; self.config.dof];
        }

        // 4. Output commanded positions to hardware driver if torqued (generates anti-gravity restorative torque)
        if self.is_torqued && self.safety.state().is_operational() {
            if let Some(ref d) = self.driver {
                if let Ok(mut driver) = d.lock() {
                    let targets: Vec<JointTarget> = self
                        .commanded_positions
                        .iter()
                        .zip(self.commanded_velocities.iter())
                        .map(|(&p, &v)| JointTarget {
                            position: p,
                            velocity: Some(v),
                            torque: None,
                            kp: None,
                            kd: None,
                        })
                        .collect();
                    let _ = driver.write_joint_targets(&targets);
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
}

impl ArmSupervisor {
    pub fn new(config: ArmConfig) -> Self {
        let (tx, _) = broadcast::channel(128);
        Self {
            inner: Arc::new(RwLock::new(SupervisorInner::new(config))),
            telemetry_tx: tx,
        }
    }

    pub fn subscribe_telemetry(&self) -> broadcast::Receiver<ArmObservation> {
        self.telemetry_tx.subscribe()
    }

    pub async fn get_observation(&self) -> ArmObservation {
        let inner = self.inner.read().await;
        inner.current_observation()
    }

    pub async fn get_schema(&self) -> ObservationSchema {
        let inner = self.inner.read().await;
        ObservationSchema::from_config(&inner.config)
    }

    pub async fn set_driver(&self, driver: Box<dyn ArmDriver>) {
        let mut inner = self.inner.write().await;
        inner.driver = Some(std::sync::Mutex::new(driver));
    }

    pub fn with_driver(self, driver: Box<dyn ArmDriver>) -> Self {
        if let Ok(mut inner) = self.inner.try_write() {
            inner.driver = Some(std::sync::Mutex::new(driver));
        }
        self
    }

    pub async fn set_torque(&self, enable: bool) -> Result<SafetyState, ArmError> {
        let mut inner = self.inner.write().await;
        let state = inner.set_torque(enable)?;
        Ok(state)
    }

    pub async fn heartbeat(&self, client_id: &str, epoch_id: u64) -> Result<(), SafetyError> {
        let mut inner = self.inner.write().await;
        inner.heartbeat(client_id, epoch_id)
    }

    pub async fn servo(&self, targets: Vec<f32>, epoch_id: u64) -> Result<(), ArmError> {
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
        let mut inner = self.inner.write().await;

        // 1. Validate command against single controller authority, safety state, and soft limits
        inner.safety.validate_motion_command(
            epoch_id,
            &target_positions,
            &inner.config.limits,
        )?;

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

    pub async fn trigger_e_stop(&self) -> (SafetyState, u64) {
        let mut inner = self.inner.write().await;
        inner.apply_stop_policy(StopPolicy::EStopLatch);
        (inner.safety.state().clone(), inner.safety.epoch_id())
    }

    pub async fn clear_fault(&self) -> Result<SafetyState, ArmError> {
        let mut inner = self.inner.write().await;
        if inner.safety.state().is_stopped() {
            let new_state = inner.safety.clear_fault()?;
            inner.last_heartbeat = Instant::now();
            inner.is_torqued = false; // Require explicit re-torque after clearing fault
            Ok(new_state)
        } else {
            Ok(inner.safety.state().clone())
        }
    }

    pub async fn apply_stop_policy(&self, policy: StopPolicy) {
        let mut inner = self.inner.write().await;
        inner.apply_stop_policy(policy);
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
