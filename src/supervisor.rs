use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::{broadcast, RwLock};
use thiserror::Error;

use crate::driver::{ArmDriver, JointTarget, EnableState, DriverCapabilities};
use crate::model::{ArmConfig, ArmObservation, ObservationSchema, RestingPosePolicy};
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

    #[error("Unsupported operation: {0}")]
    Unsupported(String),

    #[error("Internal error: {0}")]
    Internal(String),
}

#[derive(Debug, thiserror::Error, PartialEq, Eq, Clone)]
pub enum PreemptError {
    #[error("Preemption cooldown active: {0}ms remaining")]
    Cooldown(u64),
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
    pub commanded_accelerations: Vec<f32>,
    pub following_error_since: Option<Instant>,
    pub disable_pending: bool,
    pub last_disable_attempt: Option<Instant>,
    pub output_suspended: bool,
    pub feedback_numeric_valid: bool,
    pub torques: Vec<f32>,
    pub temperatures: Vec<f32>,
    pub target_positions: Vec<f32>,
    pub servo_target: Option<Vec<f32>>,
    pub servo_settled_ticks: Vec<u32>,
    pub is_torqued: bool,
    pub pending_rearm: bool,
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
    pub last_preemption: Option<Instant>,
    pub handover_settling_since: Option<Instant>,
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
                        tracing::info!("URDF model verified ({}); output still requires an explicit model profile and compatible driver.", path);
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
            commanded_accelerations: vec![0.0; dof],
            following_error_since: None,
            disable_pending: false,
            last_disable_attempt: None,
            output_suspended: false,
            feedback_numeric_valid: true,
            torques: vec![0.0; dof],
            temperatures: vec![25.0; dof],
            target_positions,
            servo_target: None,
            servo_settled_ticks: vec![0; dof],
            is_torqued: false,
            pending_rearm: false,
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
            last_preemption: None,
            handover_settling_since: None,
        }
    }

    pub fn with_driver(mut self, driver: Box<dyn ArmDriver>) -> Self {
        self.apply_driver_limits(driver.as_ref());
        self.driver = Some(std::sync::Mutex::new(driver));
        self
    }

    fn apply_driver_limits(&mut self, driver: &dyn ArmDriver) {
        for (i, limit) in self.config.limits.iter_mut().enumerate() {
            if let Some(max) = driver.max_command_velocity(i) {
                if max.is_finite() && max > 0.0 { limit.max_velocity = limit.max_velocity.min(max); }
            }
        }
    }

    fn gravity_output_enabled(&self) -> bool {
        self.gravity_compensated && self.config.gravity_model.is_some() && matches!(self.config.dof, 6 | 7) && self.driver.as_ref().map_or(true, |d|
            d.lock().map_or(false, |drv| drv.capabilities().torque_feedforward))
    }

    fn states_valid(states: &[crate::model::JointState], caps: DriverCapabilities, dof: usize) -> bool {
        states.len() == dof && states.iter().all(|s| s.position.is_finite()
            && (!caps.velocity_feedback || s.velocity.is_finite())
            && (!caps.torque_feedback || s.torque.is_finite())
            && (!caps.temperature_feedback || s.temperature.is_finite()))
    }

    pub fn hardware_feedback_fresh(&self, max_age: Duration) -> bool {
        if !self.feedback_numeric_valid { return false; }
        self.driver.as_ref().map_or(true, |d| d.lock().map_or(false, |drv| {
            let ages = drv.feedback_ages();
            drv.is_feedback_valid() && ages.len() == self.config.dof
                && ages.iter().all(|a| a.is_some_and(|age| age <= max_age))
        }))
    }

    /// Explicit user release uses the same bounded deceleration as a stream timeout.
    pub fn stop_motion(&mut self) {
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        self.active_trajectory = None;
        self.pending_normal_unload = false;
        self.unload_settle_ticks = None;
        self.target_positions = self.commanded_positions.clone();
    }

    /// A successful transport write is not confirmation that torque was removed.
    fn request_disable(&mut self) -> Result<(), SafetyError> {
        self.disable_pending = true;
        self.last_disable_attempt = Some(Instant::now());
        if let Some(ref d) = self.driver {
            let mut drv = d.lock().map_err(|_| SafetyError::FeedbackUnavailable)?;
            drv.set_torque(false).map_err(|e| SafetyError::ClearFaultRejected(format!("Torque disable transmission failed: {e}")))?;
            let poll_start = Instant::now();
            while poll_start.elapsed() < Duration::from_millis(25) {
                let _ = drv.read_joint_states();
                if drv.is_feedback_valid() && drv.enable_state() == EnableState::Disabled {
                    break;
                }
                std::thread::sleep(Duration::from_millis(2));
            }
            if !drv.is_feedback_valid() || drv.enable_state() != EnableState::Disabled {
                return Err(SafetyError::ClearFaultRejected("Torque disable has not been confirmed by fresh hardware feedback".into()));
            }
        }
        self.is_torqued = false;
        self.pending_rearm = false;
        self.disable_pending = false;
        Ok(())
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
            gravity_compensated: self.gravity_output_enabled(),
            gravity_torques: self.gravity_torques.clone(),
            driver_capabilities: self.driver.as_ref().and_then(|d| d.lock().ok().map(|drv| drv.capabilities())).unwrap_or_default(),
            enable_state: self.driver.as_ref().map_or(if self.is_torqued { EnableState::Enabled } else { EnableState::Disabled }, |d| d.lock().map_or(EnableState::Unknown, |drv| drv.enable_state())),
            feedback_age_ms: self.driver.as_ref().and_then(|d| d.lock().ok().map(|drv| drv.feedback_ages().iter().map(|a| a.map(|v| v.as_millis().min(u64::MAX as u128) as u64)).collect())).unwrap_or_else(|| vec![Some(0); self.config.dof]),
            disable_pending: self.disable_pending,
            feedback_valid: self.hardware_feedback_fresh(Duration::from_millis(250)),
        }
    }

    pub fn is_at_safe_resting_pose(&self) -> bool {
        let dof = self.config.dof;
        if self.measured_positions.len() != dof || self.measured_velocities.len() != dof { return false; }
        let simulation_policy = RestingPosePolicy::rebot_dm();
        let policy = match &self.config.resting_pose {
            Some(policy) => policy,
            None if self.driver.is_none() => &simulation_policy,
            None => return false,
        };
        if let Some(ref d) = self.driver {
            let Ok(drv) = d.lock() else { return false; };
            let caps = drv.capabilities();
            if !caps.velocity_feedback || !caps.torque_feedback || !caps.enable_feedback
                || drv.enable_state() != EnableState::Enabled || self.safety.state().is_stopped() { return false; }
            drop(drv);
            if !self.hardware_feedback_fresh(Duration::from_millis(policy.feedback_max_age_ms)) { return false; }
        }
        let mut total = 0.0;
        for (i, tolerance) in policy.position_tolerances.iter().enumerate().take(dof) {
            let dev = (self.measured_positions[i] - self.config.home_positions[i]).abs();
            if !dev.is_finite() || dev > *tolerance { return false; }
            total += dev;
        }
        if total > policy.max_total_deviation || self.measured_velocities.iter().any(|v| !v.is_finite() || v.abs() > policy.max_velocity) { return false; }
        if self.driver.is_some() {
            for &(i, max_tau) in &policy.support_torque_limits {
                if i >= dof { return false; }
                let tau = self.torques.get(i).copied().unwrap_or(f32::NAN);
                if !tau.is_finite() || tau.abs() > max_tau { return false; }
            }
        }
        true
    }

    pub fn set_torque(&mut self, enable: bool) -> Result<SafetyState, SafetyError> {
        if enable {
            if self.handover_settling_since.is_some() {
                return Err(SafetyError::HandoverSettling);
            }
            if self.disable_pending { return Err(SafetyError::ClearFaultRejected("Torque disable is still pending".into())); }
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
                if let Err(e) = driver.set_torque(true) {
                    if !was_torqued {
                        let sent = driver.set_torque(false).is_ok();
                        let _ = driver.read_joint_states();
                        let confirmed_off = sent && driver.is_feedback_valid() && driver.enable_state() == EnableState::Disabled;
                        self.is_torqued = !confirmed_off;
                        self.disable_pending = !confirmed_off;
                        self.last_disable_attempt = Some(Instant::now());
                    }
                    let err_msg = format!("Hardware torque enable failed: {}", e);
                    self.safety.trigger_fault(FaultReason::Other(err_msg.clone()));
                    return Err(SafetyError::ClearFaultRejected(err_msg));
                }
                let states = match driver.read_joint_states() {
                    Ok(s) => s,
                    Err(_) => {
                        if !was_torqued {
                            let sent = driver.set_torque(false).is_ok();
                            let _ = driver.read_joint_states();
                            let confirmed_off = sent && driver.is_feedback_valid() && driver.enable_state() == EnableState::Disabled;
                            self.is_torqued = !confirmed_off;
                            self.disable_pending = !confirmed_off;
                            self.last_disable_attempt = Some(Instant::now());
                        }
                        return Err(SafetyError::FeedbackUnavailable);
                    }
                };
                if !driver.is_feedback_valid() || !Self::states_valid(&states, driver.capabilities(), self.config.dof)
                    || (driver.capabilities().enable_feedback && driver.enable_state() != EnableState::Enabled) {
                    if !was_torqued {
                        let sent = driver.set_torque(false).is_ok();
                        let _ = driver.read_joint_states();
                        let confirmed_off = sent && driver.is_feedback_valid() && driver.enable_state() == EnableState::Disabled;
                        self.is_torqued = !confirmed_off;
                        self.disable_pending = !confirmed_off;
                        self.last_disable_attempt = Some(Instant::now());
                    }
                    return Err(SafetyError::FeedbackUnavailable);
                }
                self.feedback_numeric_valid = true;
                self.last_can_feedback_instant = driver.feedback_ages().into_iter().collect::<Option<Vec<_>>>()
                    .and_then(|ages| ages.into_iter().max()).and_then(|age| Instant::now().checked_sub(age));
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
            self.commanded_accelerations.fill(0.0);
            if self.driver.is_none() { self.measured_velocities.fill(0.0); }
            self.active_trajectory = None;
            self.pending_normal_unload = false;
            self.unload_settle_ticks = None;
            self.servo_target = None;

            let state = self.safety.set_torque(true)?;
            self.is_torqued = true;
            self.pending_rearm = false;
            self.disable_pending = false;
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

            self.stop_motion();
            self.commanded_velocities.fill(0.0);
            if let Err(error) = self.request_disable() {
                self.safety.trigger_fault(FaultReason::Other(error.to_string()));
                return Err(error);
            }
            let state = self.safety.set_torque(false)?;
            if self.driver.is_none() { self.measured_velocities.fill(0.0); }
            Ok(state)
        }
    }

    /// Automatic unload requires a configured resting pose and observable
    /// velocity, supporting torque and enable state. This is a capability check,
    /// not a claim that the arm is currently resting or ready to disable.
    pub fn supports_automatic_unload(&self) -> bool {
        self.driver.as_ref().map_or(true, |driver| {
            self.config.resting_pose.is_some() && driver.lock().map_or(false, |driver| {
                let capabilities = driver.capabilities();
                capabilities.velocity_feedback && capabilities.torque_feedback && capabilities.enable_feedback
            })
        })
    }

    pub fn execute_stop(&mut self, policy: StopPolicy) {
        self.apply_stop_policy(policy);
    }

    pub fn apply_stop_policy(&mut self, policy: StopPolicy) {
        if policy == StopPolicy::NormalUnload && !self.supports_automatic_unload() {
            tracing::warn!("NormalUnload unsupported: resting pose or confirmed velocity/torque/enable feedback unavailable; no unload motion started");
            return;
        }
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        match policy {
            StopPolicy::NormalUnload => {
                if self.handover_settling_since.is_some() {
                    tracing::warn!("NormalUnload rejected while handover settling; maintaining holding torque");
                    return;
                }
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
                let can_unload = self.is_torqued && !self.pending_rearm && (
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
                self.stop_motion();
                self.commanded_velocities.fill(0.0);
                self.commanded_accelerations.fill(0.0);
                let (joint_idx, temperature) = self.temperatures.iter().copied().enumerate()
                    .filter(|(_, t)| t.is_finite()).max_by(|a,b| a.1.total_cmp(&b.1)).unwrap_or((0, 0.0));
                if !matches!(self.safety.state(), SafetyState::Fault(FaultReason::ThermalTrip { .. })) {
                    self.safety.trigger_fault(FaultReason::ThermalTrip { joint_idx, temperature, limit: self.config.max_temperature });
                }
                if let Err(e) = self.request_disable() { tracing::error!("Thermal shutdown not confirmed: {e}"); }
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
        if !self.safety.state().is_stopped() { return Ok(self.safety.state().clone()); }
        if self.disable_pending { return Err(SafetyError::ClearFaultRejected("Torque disable is still unconfirmed".into())); }
        let thermal = matches!(self.safety.state(), SafetyState::Fault(FaultReason::ThermalTrip { .. }));
        let confirmed = if let Some(ref d) = self.driver {
            let mut drv = d.lock().map_err(|_| SafetyError::FeedbackUnavailable)?;
            // Clearing a latched motor error requires a later feedback observation, not a cleared cache.
            if drv.check_hardware_status().is_err() {
                drv.clear_fault().map_err(|e| SafetyError::ClearFaultRejected(e.to_string()))?;
                return Err(SafetyError::ClearFaultRejected("Hardware reset sent; wait for fresh fault-free feedback before retrying".into()));
            }
            let states = drv.read_joint_states().map_err(|e| SafetyError::ClearFaultRejected(e.to_string()))?;
            let ages = drv.feedback_ages();
            if !drv.is_feedback_valid() || ages.len() != self.config.dof
                || ages.iter().any(|a| a.is_none_or(|t| t > Duration::from_millis(250)))
                || !Self::states_valid(&states, drv.capabilities(), self.config.dof) {
                return Err(SafetyError::ClearFaultRejected("Fresh feedback is required before resetting a fault".into()));
            }
            drv.check_hardware_status().map_err(|e| SafetyError::ClearFaultRejected(e.to_string()))?;
            if thermal && (!drv.capabilities().temperature_feedback || states.iter().any(|state|
                !state.temperature.is_finite() || state.temperature > self.config.max_temperature - 5.0)) {
                return Err(SafetyError::ClearFaultRejected("Temperature must be confirmed below the reset threshold".into()));
            }
            let state = drv.enable_state();
            if state == EnableState::Unknown {
                return Err(SafetyError::ClearFaultRejected("Hardware enable state is unknown; reset remains latched".into()));
            }
            self.feedback_numeric_valid = true;
            for (i, state) in states.iter().enumerate() {
                self.measured_positions[i] = state.position;
                self.measured_velocities[i] = state.velocity;
                self.torques[i] = state.torque;
                self.temperatures[i] = state.temperature;
            }
            state
        } else {
            if thermal && self.temperatures.iter().any(|t| !t.is_finite() || *t > self.config.max_temperature - 5.0) {
                return Err(SafetyError::ClearFaultRejected("Temperature is above the reset threshold".into()));
            }
            if self.is_torqued { EnableState::Enabled } else { EnableState::Disabled }
        };
        self.is_torqued = confirmed == EnableState::Enabled;
        self.pending_rearm = true;
        self.stop_motion();
        // Reset at the measured position; never retain a residual old trajectory target.
        self.commanded_positions = self.measured_positions.clone();
        self.target_positions = self.measured_positions.clone();
        self.commanded_velocities.fill(0.0);
        self.last_commanded_velocities.fill(0.0);
        self.commanded_accelerations.fill(0.0);
        self.following_error_since = None;
        self.bus_loss_instant = None;
        let state = self.safety.clear_fault()?;
        self.handover_settling_since = None;
        self.e_stop_latched.store(false, Ordering::SeqCst);
        self.last_heartbeat = Instant::now();
        self.last_motion_frame = Instant::now();
        Ok(state)
    }

    pub fn shutdown_ready(&self) -> bool {
        if self.disable_pending { return false; }
        match &self.driver {
            None => !self.is_torqued,
            Some(driver) => !self.is_torqued
                && driver.lock().map_or(false, |drv| {
                    if !drv.capabilities().enable_feedback {
                        true
                    } else {
                        drv.enable_state() == EnableState::Disabled
                    }
                }),
        }
    }

    pub fn is_fully_settled_hold(&self) -> bool {
        // Safe condition 1: Confirmed disabled (torque off, Idle, not disable pending)
        if !self.is_torqued && matches!(self.safety.state(), SafetyState::Idle) && !self.disable_pending {
            return true;
        }

        let (has_velocity_capability, is_valid) = match &self.driver {
            None => (true, true),
            Some(driver) => driver.lock().map_or((false, false), |drv| (drv.capabilities().velocity_feedback, drv.is_feedback_valid())),
        };

        if !is_valid {
            return false;
        }

        let velocity_settled = if has_velocity_capability {
            self.measured_velocities.len() == self.config.dof
                && self.measured_velocities.iter().all(|v| v.is_finite() && v.abs() <= 0.05)
        } else {
            true
        };

        (self.safety.state() == &SafetyState::Armed || self.safety.state() == &SafetyState::Idle)
            && self.active_trajectory.is_none()
            && self.servo_target.is_none()
            && self.commanded_velocities.len() == self.config.dof
            && self.commanded_velocities.iter().all(|v| *v == 0.0)
            && self.hardware_feedback_fresh(Duration::from_millis(100))
            && velocity_settled
    }

    pub fn is_fully_settled_armed_hold(&self) -> bool {
        self.is_fully_settled_hold()
    }

    pub fn preempt_lease(&mut self, conn: &str, label: &str) -> Result<(u64, Option<(String, u64)>), PreemptError> {
        let now = Instant::now();
        if let Some(ref active) = self.active_lease {
            let is_expired = active.acquired_at.elapsed() >= self.lease_timeout;
            if active.client_id == conn && !is_expired {
                return Ok((active.generation, None));
            }

            if active.client_id != conn {
                let cooldown = Duration::from_secs(3);
                if let Some(last) = self.last_preemption {
                    let elapsed = last.elapsed();
                    if elapsed < cooldown {
                        let remaining_ms = (cooldown.saturating_sub(elapsed).as_millis() as u64).max(1);
                        return Err(PreemptError::Cooldown(remaining_ms));
                    }
                }
            }
        }

        // Four-category state categorization (strictly fail-closed, no unwrap/panic, driver lock poison returns false)
        let confirmed_hardware_disabled = match &self.driver {
            None => !self.is_torqued && !self.disable_pending && self.safety.state() == &SafetyState::Idle,
            Some(driver) => !self.is_torqued
                && !self.disable_pending
                && self.safety.state() == &SafetyState::Idle
                && driver.lock().map_or(false, |drv| {
                    if !drv.capabilities().enable_feedback {
                        true
                    } else {
                        drv.enable_state() == EnableState::Disabled
                    }
                }),
        };

        if confirmed_hardware_disabled {
            self.handover_settling_since = None;
        } else if self.safety.state().is_stopped() {
            self.handover_settling_since = None;
        } else if self.is_fully_settled_armed_hold() {
            self.handover_settling_since = None;
        } else {
            self.stop_motion();
            self.handover_settling_since = Some(now);
        }

        let evicted = self.active_lease.as_ref().filter(|a| a.client_id != conn).map(|a| (a.client_id.clone(), a.generation));

        self.lease_generation_counter += 1;
        let gen = self.lease_generation_counter;
        self.active_lease = Some(ActiveLease {
            client_id: conn.to_string(),
            generation: gen,
            acquired_at: now,
            last_valid_motion_instant: None,
        });
        self.last_heartbeat = now;
        if evicted.is_some() {
            self.last_preemption = Some(now);
        }

        tracing::info!("Preempted lease for connection {} (label {}), generation {}, evicted: {:?}", conn, label, gen, evicted);
        Ok((gen, evicted))
    }

    pub fn acquire_lease(&mut self, client_id: &str) -> Result<u64, (String, u64)> {
        let now = Instant::now();
        if let Some(ref active) = self.active_lease {
            let is_expired = active.acquired_at.elapsed() >= self.lease_timeout;
            if !is_expired {
                if active.client_id == client_id { return Ok(active.generation); }
                return Err((active.client_id.clone(), active.generation));
            }
        }
        // A new controller never inherits an expired controller's queued motion.
        if self.active_lease.is_some() { self.stop_motion(); }
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
            if active.client_id == client_id && active.acquired_at.elapsed() < self.lease_timeout {
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
        if self.handover_settling_since.is_some() {
            return Err(SafetyError::HandoverSettling);
        }

        if self.e_stop_latched.load(Ordering::SeqCst) {
            return Err(SafetyError::EStopActive);
        }

        if epoch_id != self.safety.epoch_id() {
            return Err(SafetyError::EpochMismatch {
                expected: self.safety.epoch_id(),
                received: epoch_id,
            });
        }

        if self.pending_rearm || self.active_trajectory.is_some() || self.pending_normal_unload {
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

    pub fn jog(
        &mut self,
        target_positions: Vec<f32>,
        duration_secs: Option<f64>,
        epoch_id: Option<u64>,
    ) -> Result<u64, ArmError> {
        if self.handover_settling_since.is_some() {
            return Err(ArmError::Safety(SafetyError::HandoverSettling));
        }

        if self.e_stop_latched.load(Ordering::SeqCst) {
            return Err(ArmError::Safety(SafetyError::EStopActive));
        }

        if self.pending_rearm || self.pending_normal_unload {
            return Err(ArmError::Safety(SafetyError::NotArmed(self.safety.state().clone())));
        }
        self.pending_normal_unload = false;

        // 1. Validate command against single controller authority, safety state, and soft limits
        self.safety.validate_motion_command(
            epoch_id,
            &target_positions,
            &self.config.limits,
        )?;

        // J7 Gripper stall release check on reverse command
        if self.config.dof >= 7 && self.gripper_stalled && target_positions.len() >= 7 {
            let delta = target_positions[6] - self.commanded_positions[6];
            let tau_7 = self.torques.get(6).copied().unwrap_or(0.0);
            if (delta * tau_7 < -1e-4) || (delta * self.gripper_stall_sign < -1e-4) {
                self.gripper_stalled = false;
                self.gripper_high_torque_ticks = 0;
                self.gripper_low_torque_ticks = 0;
            }
        }

        // 2. Plan quintic polynomial trajectory from commanded state to target
        let traj = TrajectoryPlanner::plan(
            &self.commanded_positions,
            &self.commanded_velocities,
            &target_positions,
            &vec![0.0; self.config.dof],
            duration_secs,
            &self.config.limits,
        )?;

        // 3. Only renew lease AFTER command validation and planning succeed
        self.last_heartbeat = Instant::now();

        self.servo_target = None;
        self.target_positions = target_positions;
        self.active_trajectory = Some(traj);
        self.safety.mark_motion_started()?;

        Ok(self.safety.epoch_id())
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
        Self::bounded_servo_step_with_jerk(q, target, v_prev, dt, limit, 0.0, None)
    }

    pub fn bounded_servo_step_with_jerk(
        q: f32, target: f32, v_prev: f32, dt: f64,
        limit: &crate::model::JointLimit, previous_acceleration: f32, max_jerk: Option<f32>,
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

        let preferred = if let Some(jerk) = max_jerk {
            if !jerk.is_finite() || jerk <= 0.0 || !previous_acceleration.is_finite() { return None; }
            let acceleration = ((desired - v_prev64) / h).clamp(-amax, amax);
            let previous = f64::from(previous_acceleration);
            let da = f64::from(jerk) * h;
            v_prev64 + acceleration.clamp(previous - da, previous + da) * h
        } else { desired };
        // Do not let acceleration history carry a stationary target past its braking
        // envelope. This also removes the small limit cycle that a pure jerk filter
        // creates near arrival. Changed targets can make this envelope infeasible;
        // the final projection still gives acceleration and joint limits priority.
        let preferred = if desired >= 0.0 && v_prev64 >= 0.0 {
            preferred.clamp(0.0, desired)
        } else if desired <= 0.0 && v_prev64 <= 0.0 {
            preferred.clamp(desired, 0.0)
        } else { preferred };
        // Safety bounds override the comfort preference when braking space is short.
        let v_next64 = preferred.clamp(lower, upper);
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
        if self.e_stop_latched.load(Ordering::SeqCst) && !self.safety.state().is_stopped() {
            self.apply_stop_policy(StopPolicy::EStopLatch);
        }

        // Polling a cached sample never refreshes its hardware receive timestamp.
        let mut invalid_feedback = false;
        if let Some(ref d) = self.driver {
            self.feedback_numeric_valid = false;
            if let Ok(mut driver) = d.lock() {
                if let Ok(states) = driver.read_joint_states() {
                    self.feedback_numeric_valid = Self::states_valid(&states, driver.capabilities(), self.config.dof);
                    invalid_feedback = !self.feedback_numeric_valid;
                    let ages = driver.feedback_ages();
                    if self.feedback_numeric_valid && ages.len() == self.config.dof && driver.is_feedback_valid() {
                        self.last_can_feedback_instant = ages.iter().copied().collect::<Option<Vec<_>>>()
                            .and_then(|ages| ages.into_iter().max()).and_then(|age| Instant::now().checked_sub(age));
                    } else { self.last_can_feedback_instant = None; }
                    if self.feedback_numeric_valid {
                        for (i, state) in states.iter().enumerate() {
                            self.measured_positions[i] = state.position;
                            self.measured_velocities[i] = state.velocity;
                            self.torques[i] = state.torque;
                            self.temperatures[i] = state.temperature;
                        }
                    }
                }
            }
        }
        if invalid_feedback && self.is_torqued && !self.safety.state().is_stopped() {
            self.stop_motion();
            self.commanded_velocities.fill(0.0);
            self.safety.trigger_fault(FaultReason::Other("Invalid hardware telemetry".into()));
        }

        // A single late tick only shortens integration (dt is clamped below); the motors
        // keep holding their last POS_VEL target meanwhile. Fault on a real stall, using
        // the same 250 ms bound as the hardware feedback deadline.
        if (!dt.is_finite() || dt <= 0.0 || dt > 0.250)
            && self.is_torqued && self.safety.state().is_operational() {
            self.stop_motion();
            if self.measured_positions.iter().all(|p| p.is_finite()) {
                self.commanded_positions = self.measured_positions.clone();
                self.target_positions = self.measured_positions.clone();
            }
            self.commanded_velocities.fill(0.0);
            self.commanded_accelerations.fill(0.0);
            self.safety.trigger_fault(FaultReason::Other("Control loop deadline exceeded".into()));
        }
        let dt = if dt.is_finite() && dt > 0.0 { dt.min(0.080) } else { 0.020 };
        let before_velocities = self.commanded_velocities.clone();

        // 0.1 Gripper stall protection state machine for J7 (joint 6)
        if self.config.dof >= 7 {
            let is_bus_fresh = if let Some(ref d) = self.driver {
                if let Ok(driver) = d.lock() {
                    self.last_can_feedback_instant.map_or(false, |t| t.elapsed() < Duration::from_millis(50)) && driver.is_feedback_valid() && driver.capabilities().torque_feedback
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
                if tau_7.abs() >= 0.80 {
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
                // Exit condition B: |tau_7| < 0.40 for 6 consecutive ticks
                if tau_7.abs() < 0.40 {
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
        let driver_feedback_ok = self.hardware_feedback_fresh(Duration::from_millis(250));

        if self.gravity_output_enabled() && driver_feedback_ok && self.measured_positions.len() >= 6 {
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
            let target_tau = if self.gravity_output_enabled() && driver_feedback_ok && i < self.gravity_torques.len() {
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

        // Check the driver's per-joint feedback deadline (250 ms for the bundled drivers).
        if self.is_torqued
            && !self.safety.state().is_stopped()
            && self.driver.is_some()
            && !self.hardware_feedback_fresh(Duration::from_millis(250))
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

        // Thermal protection remains active while stopped or shutdown is unconfirmed.
        let temperature_available = self.driver.as_ref().map_or(true, |d| d.lock().map_or(false, |drv| drv.capabilities().temperature_feedback));
        if (self.is_torqued || self.disable_pending) && temperature_available {
            if self.temperatures.iter().any(|t| t.is_finite() && *t > self.config.max_temperature)
                && !matches!(self.safety.state(), SafetyState::Fault(FaultReason::ThermalTrip { .. })) {
                self.apply_stop_policy(StopPolicy::ThermalTrip);
            }
        }
        if self.disable_pending {
            let confirmed = self.driver.as_ref().map_or(true, |d| d.lock().map_or(false, |drv|
                drv.is_feedback_valid() && drv.enable_state() == EnableState::Disabled));
            if confirmed {
                self.is_torqued = false;
                self.pending_rearm = false;
                self.disable_pending = false;
            } else if self.last_disable_attempt.is_none_or(|t| t.elapsed() >= Duration::from_millis(250)) {
                if let Err(e) = self.request_disable() { tracing::warn!("Torque shutdown remains unconfirmed: {e}"); }
            }
        }
        // Detect sustained physical tracking error, never compare against stale samples.
        if self.driver.is_some() && self.is_torqued && self.safety.state().is_operational()
            && self.hardware_feedback_fresh(Duration::from_millis(100)) {
            let threshold = self.config.following_error.max_error_rad;
            let outside = self.commanded_positions.iter().zip(&self.measured_positions)
                .any(|(q, measured)| !measured.is_finite() || (q - measured).abs() > threshold);
            if outside {
                let since = self.following_error_since.get_or_insert_with(Instant::now);
                if since.elapsed() >= Duration::from_millis(self.config.following_error.persistence_ms) {
                    self.stop_motion();
                    self.commanded_positions = self.measured_positions.clone();
                    self.target_positions = self.measured_positions.clone();
                    self.commanded_velocities.fill(0.0);
                    self.commanded_accelerations.fill(0.0);
                    self.safety.trigger_fault(FaultReason::Other("Persistent joint tracking error".into()));
                }
            } else { self.following_error_since = None; }
        } else { self.following_error_since = None; }

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
                // Check if any target joint has instantaneous jump > 0.35 rad relative to commanded_positions
                let has_dirty_jump = target.iter().enumerate().any(|(i, &tgt)| {
                    if i < self.commanded_positions.len() {
                        let jump = (tgt - self.commanded_positions[i]).abs();
                        jump > 0.35
                    } else {
                        false
                    }
                });

                if has_dirty_jump {
                    tracing::warn!(
                        "Discarding dirty servo target frame: instantaneous jump exceeds 0.35 rad relative to commanded_positions"
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
                            let step = Self::bounded_servo_step_with_jerk(
                                next_positions[i],
                                actual_tgt,
                                next_velocities[i],
                                servo_dt,
                                limit,
                                self.commanded_accelerations[i],
                                self.config.max_jerk,
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
                let step = Self::bounded_servo_step_with_jerk(
                    self.commanded_positions[i],
                    self.commanded_positions[i],
                    self.commanded_velocities[i],
                    servo_dt,
                    &self.config.limits[i],
                    self.commanded_accelerations[i],
                    self.config.max_jerk,
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

        for i in 0..self.config.dof {
            self.commanded_accelerations[i] = if self.safety.state().is_stopped() { 0.0 }
                else { (self.commanded_velocities[i] - before_velocities[i]) / servo_dt as f32 };
        }
        self.last_commanded_velocities = self.commanded_velocities.clone();

        let feedback_fresh = self.hardware_feedback_fresh(Duration::from_millis(250));
        if self.driver.is_some() && self.is_torqued {
            if !feedback_fresh { self.output_suspended = true; }
            else if self.output_suspended {
                self.stop_motion();
                self.commanded_positions = self.measured_positions.clone();
                self.target_positions = self.measured_positions.clone();
                self.commanded_velocities.fill(0.0);
                self.commanded_accelerations.fill(0.0);
                self.output_suspended = false;
                if !self.safety.state().is_stopped() {
                    self.safety.trigger_fault(FaultReason::BusLoss);
                }
            }
        }
        let gravity_output_enabled = self.gravity_output_enabled();
        // Hold only while torque may be active and fresh valid feedback permits output.
        // Pending disable and invalid feedback suppress ordinary commands; reconnect resynchronizes first.
        let mut output_error = None;
        if self.is_torqued && !self.disable_pending {
            let send_to_bus = feedback_fresh && self.driver.as_ref().map_or(true, |d|
                d.lock().map_or(false, |drv| drv.enable_state() != EnableState::Disabled));

            if send_to_bus {
                if let Some(ref d) = self.driver {
                    if let Ok(mut driver) = d.lock() {
                        let targets: Vec<JointTarget> = self
                            .commanded_positions
                            .iter()
                            .zip(self.commanded_velocities.iter())
                            .enumerate()
                            .map(|(i, (&p, &v))| JointTarget {
                                position: p,
                                velocity: Some(v),
                                torque: if gravity_output_enabled {
                                    self.gravity_torques_ramped.get(i).copied()
                                } else {
                                    Some(0.0)
                                },
                                kp: None,
                                kd: None,
                            })
                            .collect();
                        if let Err(e) = driver.write_joint_targets(&targets) { output_error = Some(e.to_string()); }
                    }
                }
            }
        }

        if let Some(error) = output_error {
            self.stop_motion();
            self.commanded_velocities.fill(0.0);
            if self.measured_positions.iter().all(|q| q.is_finite()) {
                self.commanded_positions = self.measured_positions.clone();
                self.target_positions = self.measured_positions.clone();
            }
            if !self.safety.state().is_stopped() {
                self.safety.trigger_fault(FaultReason::Other(format!("Control output failed: {error}")));
            }
            tracing::error!("Control output failed: {error}");
        }

        // If in simulation (no hardware driver attached), mirror commanded into measured
        if self.driver.is_none() {
            self.measured_positions = self.commanded_positions.clone();
            self.measured_velocities = self.commanded_velocities.clone();
        }

        if let Some(settling_since) = self.handover_settling_since {
            if self.safety.state().is_stopped() {
                // If arm entered Stopped (Fault, e.g. ThermalTrip, BusLoss, or EStop), terminate
                // handover settling immediately and preserve the existing fault reason!
                self.handover_settling_since = None;
            } else if self.is_fully_settled_armed_hold() {
                self.handover_settling_since = None;
            } else if settling_since.elapsed() > Duration::from_millis(1500) {
                self.stop_motion();
                self.commanded_velocities = vec![0.0; self.config.dof];
                self.safety.trigger_fault(FaultReason::Other("Handover settling timeout".into()));
                self.handover_settling_since = None;
            }
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
        inner.apply_driver_limits(drv.as_ref());
        inner.driver = Some(std::sync::Mutex::new(drv));
    }

    pub fn with_driver(self, driver: Box<dyn ArmDriver>) -> Self {
        if let Ok(mut inner) = self.inner.try_write() {
            inner.apply_driver_limits(driver.as_ref());
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

    pub async fn preempt_lease(&self, conn: &str, label: &str) -> Result<(u64, Option<(String, u64)>), PreemptError> {
        let mut inner = self.inner.write().await;
        inner.preempt_lease(conn, label)
    }

    pub async fn is_handover_settling(&self) -> bool {
        let inner = self.inner.read().await;
        inner.handover_settling_since.is_some()
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
        inner.gravity_output_enabled()
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
        let mut inner = self.inner.write().await;
        inner.jog(target_positions, duration_secs, epoch_id)
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

    pub async fn stop_motion(&self) { self.inner.write().await.stop_motion(); }

    pub async fn shutdown_ready(&self) -> bool { self.inner.read().await.shutdown_ready() }

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
