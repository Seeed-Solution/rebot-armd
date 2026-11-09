use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::{broadcast, RwLock};
use thiserror::Error;

use crate::driver::{ArmDriver, JointTarget, EnableState, DriverCapabilities};
use crate::model::{ArmConfig, ArmObservation, ObservationSchema, RestingPosePolicy};
use crate::recording::{
    write_frames_to_parquet, EpisodeRecorder, EpisodeState, ParquetRecordingError, RecordFrame,
    RecordingError,
};
use crate::safety::{FaultReason, SafetyError, SafetyManager, SafetyState, StopPolicy};
use crate::trajectory::{MultiJointQuinticTrajectory, TrajectoryError, TrajectoryPlanner};
use crate::motion::{
    ActionStatus, ActiveTrajectory, GripperMotion, GripperRequest, JointPath, MotionError, PathTrajectory,
    PlanInput, Reference, Settle, StepCtx, StepOut, StepStatus,
};

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

    #[error("Motion error: {0}")]
    Motion(#[from] MotionError),

    #[error("Internal error: {0}")]
    Internal(String),

    /// Request refused with a specific code (gravity compensation mode changes).
    #[error("{code}: {message}")]
    Rejected { status: u16, code: String, message: String },
}

#[derive(Debug, thiserror::Error, PartialEq, Eq, Clone)]
pub enum PreemptError {
    #[error("Preemption cooldown active: {0}ms remaining")]
    Cooldown(u64),
}

/// J7 stall latch: |τ7| at or above this (N·m) with the jaw still, for 6 ticks.
/// Tuned on hardware as 0.80 when τ7 was decoded with TMAX 18; the J7 motor's TMAX
/// is 10, so the same physical torque is 0.80 × 10 / 18.
pub const GRIPPER_STALL_LATCH_NM: f32 = 0.444;
/// J7 stall release: |τ7| below this (N·m) for 6 ticks (formerly 0.40 at TMAX 18;
/// 0.40 × 10 / 18 = 0.2222). Raw torque 2093 decodes to exactly that value and was read
/// as 0.39999962 < 0.40 before, so the bound sits just above it.
pub const GRIPPER_STALL_RELEASE_NM: f32 = 0.2223;

/// Feedback gap that latches BusLoss while torqued. The Damiao motors run with
/// CAN TIMEOUT=0 and keep holding their last POS_VEL target through a gap, so a
/// short interruption is not a hazard by itself. 500 ms tolerates the ~260 ms
/// CAN gaps observed on the reBot rig (2026-11-06, coinciding with 24 V sags);
/// a real power loss drops the arm regardless of this bound.
pub const BUS_LOSS_TIMEOUT: Duration = Duration::from_millis(500);
/// Lower bound for the automatic park started by an expired lease (trajectory + settle).
/// The actual budget is `max(WATCHDOG_PARK_TIMEOUT, deceleration + planned duration +
/// UNLOAD_SETTLE_HOLD + WATCHDOG_PARK_MARGIN)`, counted in control-loop time.
pub const WATCHDOG_PARK_TIMEOUT: Duration = Duration::from_secs(10);
/// Tracking lag and slack added to the watchdog park budget.
pub const WATCHDOG_PARK_MARGIN: Duration = Duration::from_secs(3);
/// Extra time granted once when the watchdog park budget runs out with the arm at the
/// resting pose (position, support torque, enable and feedback all pass) and only the
/// stationary check pending. A blocked or unsupported arm gets no grace.
pub const WATCHDOG_PARK_SETTLE_GRACE: Duration = Duration::from_secs(5);
/// Time the arm must stay at the resting pose before an unload cuts torque.
pub const UNLOAD_SETTLE_HOLD: Duration = Duration::from_millis(800);
/// Stationary evidence over the settle hold window, per joint: max - min measured
/// position (rad). 4 mrad in 0.8 s bounds the average speed at 0.005 rad/s.
pub const UNLOAD_STILL_POSITION_SPAN_RAD: f32 = 0.004;
/// Per-sample |velocity| bound during the settle hold, as a multiple of the joint's
/// resting-pose velocity limit (J1-J3: 0.02 rad/s = 4 LSB at VMAX 10). The window mean
/// must stay within the limit itself.
pub const UNLOAD_STILL_VELOCITY_PEAK_FACTOR: f32 = 4.0;
/// The mean velocity over this most recent part of the settle window must also stay within
/// the resting-pose limit, so motion starting late in the window blocks the release
/// before it accumulates the position span.
pub const UNLOAD_STILL_RECENT: Duration = Duration::from_millis(300);
/// A settle pending longer than the hold plus this logs its blocker, at most once per second.
const UNLOAD_SETTLE_REPORT_AFTER_S: f64 = 1.0;

/// Settle phase of a NormalUnload after its trajectory finished.
#[derive(Debug, Clone, Default)]
pub struct UnloadSettle {
    /// Time in the settle phase (s, control-loop time).
    pub pending_s: f64,
    /// Time from the first to the latest sample of the current run of samples that met
    /// the static resting-pose conditions (s).
    pub held_s: f64,
    /// `pending_s` of the first sample of that run.
    pub held_since_s: Option<f64>,
    /// (time, positions, velocities) over the last [`UNLOAD_SETTLE_HOLD`] while held.
    pub samples: std::collections::VecDeque<(f64, Vec<f32>, Vec<f32>)>,
    /// `pending_s` of the last blocker report.
    pub last_report_s: Option<f64>,
}

/// Park started by the lease watchdog: elapsed control-loop time against its budget.
#[derive(Debug, Clone, PartialEq)]
pub struct WatchdogPark {
    pub elapsed_s: f64,
    pub budget_s: f64,
    /// True once [`WATCHDOG_PARK_SETTLE_GRACE`] was added to the budget.
    pub grace_granted: bool,
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
    pub active_trajectory: Option<ActiveTrajectory>,
    /// Most recent action record (published as `observation.action`).
    pub last_action: Option<ActionStatus>,
    /// Record replaced by a newer action and the timestamp until which it is published.
    pub previous_action: Option<(ActionStatus, f64)>,
    pub next_action_id: u64,
    /// Lease generation when the running action started (lease loss detection).
    pub action_lease_generation: Option<u64>,
    /// Settle phase of a finished joint_move / home quintic: (action id, settle state).
    pub quintic_settle: Option<(u64, Settle)>,
    /// Running tap-to-grasp orchestration (its sub-motions carry the grasp action id).
    pub grasp_run: Option<crate::grasp::GraspRun>,
    /// Test hook run once by the WS action handler between planning and install.
    #[doc(hidden)]
    pub before_install_hook: Option<Box<dyn FnOnce(&mut SupervisorInner) + Send + Sync>>,
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
    pub unload_settle: Option<UnloadSettle>,
    /// Set while a park started by the lease watchdog is running.
    pub watchdog_park: Option<WatchdogPark>,
    /// Duration of the most recently planned NormalUnload trajectory (s).
    pub unload_planned_duration: Option<f64>,
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
    /// Connection and generation most recently evicted by a forced acquire.
    pub last_evicted: Option<(String, u64)>,
    pub handover_settling_since: Option<Instant>,
    /// Running gravity-compensation session (`src/gravity_comp.rs`).
    pub gravity_comp: Option<crate::gravity_comp::GravityCompRun>,
    pub gravity_comp_last_exit: Option<crate::model::GravityCompExit>,
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
            last_action: None,
            previous_action: None,
            next_action_id: 0,
            action_lease_generation: None,
            grasp_run: None,
            quintic_settle: None,
            before_install_hook: None,
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
            unload_settle: None,
            watchdog_park: None,
            unload_planned_duration: None,
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
            last_evicted: None,
            handover_settling_since: None,
            gravity_comp: None,
            gravity_comp_last_exit: None,
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

    /// Joint limits used for planning and servo stepping: `config.limits` with
    /// `max_velocity` capped at `config.planned_max_velocity` for every joint
    /// except the 7-DOF gripper (index 6).
    pub fn planning_limits(&self) -> Vec<crate::model::JointLimit> {
        let mut limits = self.config.limits.clone();
        if let Some(cap) = self.config.planned_max_velocity {
            let gripper_idx = (self.config.dof >= 7).then_some(6);
            for (i, limit) in limits.iter_mut().enumerate() {
                if Some(i) != gripper_idx {
                    limit.max_velocity = limit.max_velocity.min(cap);
                }
            }
        }
        limits
    }

    /// Workspace policy and camera mount when the Cartesian check applies
    /// (workspace and kinematics configured, 7-DOF arm).
    pub fn workspace_policy(&self) -> Option<(&crate::model::WorkspacePolicy, &crate::model::CameraMount)> {
        match (&self.config.workspace, self.config.kinematics) {
            (Some(ws), Some(_)) if self.config.dof == 7 => Some((ws, &self.config.camera_mount)),
            _ => None,
        }
    }

    /// Reject a move whose samples leave the workspace further than `start` already is.
    pub fn check_workspace_path<'a>(
        &self,
        start: &[f32],
        samples: impl IntoIterator<Item = &'a [f32]>,
    ) -> Result<(), SafetyError> {
        let Some((ws, cam)) = self.workspace_policy() else { return Ok(()); };
        let v0 = crate::kinematics::workspace_violation(start, ws, cam);
        for q in samples {
            let v = crate::kinematics::workspace_violation(q, ws, cam);
            if crate::kinematics::workspace_rejects(&v, &v0) {
                let w = v.worst.unwrap_or(crate::kinematics::ViolationDetail { point: 0, limit: "invalid", value: f64::NAN, bound: 0.0 });
                return Err(SafetyError::WorkspaceViolation {
                    point: w.point, limit: w.limit.to_string(), value: w.value as f32, bound: w.bound as f32,
                });
            }
        }
        Ok(())
    }

    /// The planner's 21 verification points of a trajectory.
    pub fn trajectory_samples(traj: &MultiJointQuinticTrajectory) -> Vec<Vec<f32>> {
        (0..=20).map(|k| traj.sample(traj.duration * f64::from(k) / 20.0).positions).collect()
    }

    fn gravity_output_enabled(&self) -> bool {
        self.gravity_compensated && self.config.gravity_model.is_some() && matches!(self.config.dof, 6 | 7) && self.driver.as_ref().is_none_or(|d|
            d.lock().is_ok_and(|drv| drv.capabilities().torque_feedforward))
    }

    fn states_valid(states: &[crate::model::JointState], caps: DriverCapabilities, dof: usize) -> bool {
        states.len() == dof && states.iter().all(|s| s.position.is_finite()
            && (!caps.velocity_feedback || s.velocity.is_finite())
            && (!caps.torque_feedback || s.torque.is_finite())
            && (!caps.temperature_feedback || s.temperature.is_finite()))
    }

    pub fn hardware_feedback_fresh(&self, max_age: Duration) -> bool {
        if !self.feedback_numeric_valid { return false; }
        self.driver.as_ref().is_none_or(|d| d.lock().is_ok_and(|drv| {
            // Ages alone decide: drivers' is_feedback_valid() hard-codes 250 ms,
            // which would silently cap callers asking for a longer bound.
            let ages = drv.feedback_ages();
            ages.len() == self.config.dof
                && ages.iter().all(|a| a.is_some_and(|age| age <= max_age))
        }))
    }

    /// Explicit user release uses the same bounded deceleration as a stream timeout.
    pub fn stop_motion(&mut self) {
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        self.active_trajectory = None;
        self.pending_normal_unload = false;
        self.unload_settle = None;
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
            commanded_positions: self.commanded_positions.clone(),
            action: self.last_action.clone(),
            previous_action: self.previous_action.as_ref().filter(|(_, until)| timestamp < *until).map(|(a, _)| a.clone()),
            control_mode: if self.gravity_comp.is_some() { crate::model::ControlMode::GravityComp } else { crate::model::ControlMode::Position },
            gravity_comp: self.gravity_comp_observation(),
            driver_range_check: self.driver.as_ref().and_then(|d| d.lock().ok().and_then(|drv| drv.range_check())),
        }
    }

    pub fn is_at_safe_resting_pose(&self) -> bool {
        self.resting_pose_violation().is_none()
    }

    /// The resting-pose conditions that fail on the current sample, or `None` when the arm
    /// may be released. Velocity is judged on this one sample.
    fn resting_pose_violation(&self) -> Option<String> {
        self.resting_pose_static_violation().or_else(|| {
            let policy = self.resting_policy()?;
            let moving: Vec<String> = self.measured_velocities.iter().enumerate()
                .filter(|(i, v)| !v.is_finite() || v.abs() > policy.max_velocity_of(*i))
                .map(|(i, v)| format!("joint {} moving at {v:.4} rad/s (limit {})", i + 1, policy.max_velocity_of(i)))
                .collect();
            (!moving.is_empty()).then(|| moving.join("; "))
        })
    }

    fn resting_policy(&self) -> Option<RestingPosePolicy> {
        match &self.config.resting_pose {
            Some(policy) => Some(policy.clone()),
            None if self.driver.is_none() => Some(RestingPosePolicy::rebot_dm()),
            None => None,
        }
    }

    /// Every resting-pose condition except stationarity: feedback, enable state, position,
    /// support torque. Lists every failing joint with its value.
    fn resting_pose_static_violation(&self) -> Option<String> {
        let dof = self.config.dof;
        if self.measured_positions.len() != dof || self.measured_velocities.len() != dof {
            return Some("feedback length does not match dof".into());
        }
        let Some(policy) = self.resting_policy() else { return Some("no resting pose configured".into()); };
        if let Some(ref d) = self.driver {
            let Ok(drv) = d.lock() else { return Some("driver lock poisoned".into()); };
            let caps = drv.capabilities();
            if !caps.velocity_feedback || !caps.torque_feedback || !caps.enable_feedback {
                return Some("driver lacks velocity/torque/enable feedback".into());
            }
            let enable = drv.enable_state();
            if enable != EnableState::Enabled { return Some(format!("enable state {enable:?}")); }
            if self.safety.state().is_stopped() { return Some(format!("safety state {:?}", self.safety.state())); }
            drop(drv);
            if !self.hardware_feedback_fresh(Duration::from_millis(policy.feedback_max_age_ms)) {
                return Some(format!("feedback older than {} ms", policy.feedback_max_age_ms));
            }
        }
        let mut failures = Vec::new();
        let mut total = 0.0;
        for (i, tolerance) in policy.position_tolerances.iter().enumerate().take(dof) {
            let dev = (self.measured_positions[i] - self.config.home_positions[i]).abs();
            if !dev.is_finite() || dev > *tolerance {
                failures.push(format!("joint {} off home by {dev:.4} rad (limit {tolerance})", i + 1));
            }
            total += dev;
        }
        if failures.is_empty() && total > policy.max_total_deviation {
            failures.push(format!("total deviation {total:.4} rad (limit {})", policy.max_total_deviation));
        }
        if self.driver.is_some() {
            for &(i, max_tau) in &policy.support_torque_limits {
                if i >= dof { return Some(format!("support torque index {i} out of range")); }
                let tau = self.torques.get(i).copied().unwrap_or(f32::NAN);
                if !tau.is_finite() || tau.abs() > max_tau {
                    failures.push(format!("joint {} torque {tau:.2} Nm (limit {max_tau})", i + 1));
                }
            }
        }
        (!failures.is_empty()).then(|| failures.join("; "))
    }

    /// Stationary check over the settle hold window: per joint, the position span, the
    /// mean velocity over the window and over its last [`UNLOAD_STILL_RECENT`], and every
    /// velocity sample. A single quantized velocity sample above
    /// the resting-pose limit is noise when the position does not move and the mean holds.
    fn unload_still_violation(&self, settle: &UnloadSettle) -> Option<String> {
        let policy = self.resting_policy()?;
        let hold = UNLOAD_SETTLE_HOLD.as_secs_f64();
        if settle.held_s + 1e-6 < hold || settle.samples.is_empty() {
            return Some(format!("held {:.2} s of {hold:.2} s", settle.held_s));
        }
        let n = settle.samples.len() as f32;
        let recent_from = settle.pending_s - UNLOAD_STILL_RECENT.as_secs_f64() - 1e-6;
        let mut failures = Vec::new();
        for i in 0..self.config.dof {
            let (mut lo, mut hi, mut sum, mut peak) = (f32::INFINITY, f32::NEG_INFINITY, 0.0f32, 0.0f32);
            let (mut recent_sum, mut recent_n) = (0.0f32, 0u32);
            for (t, q, v) in &settle.samples {
                let (q, v) = (q.get(i).copied().unwrap_or(f32::NAN), v.get(i).copied().unwrap_or(f32::NAN));
                if !q.is_finite() || !v.is_finite() { return Some(format!("joint {} non-finite feedback", i + 1)); }
                lo = lo.min(q); hi = hi.max(q); sum += v; peak = peak.max(v.abs());
                if *t >= recent_from { recent_sum += v; recent_n += 1; }
            }
            let limit = policy.max_velocity_of(i);
            let (span, mean) = (hi - lo, sum / n);
            let recent = recent_sum / recent_n.max(1) as f32;
            if recent.abs() > limit {
                failures.push(format!("joint {} mean velocity over the last {} ms {recent:.4} rad/s (limit {limit})",
                    i + 1, UNLOAD_STILL_RECENT.as_millis()));
            }
            if span > UNLOAD_STILL_POSITION_SPAN_RAD {
                failures.push(format!("joint {} moved {span:.4} rad in {hold:.1} s (limit {UNLOAD_STILL_POSITION_SPAN_RAD})", i + 1));
            }
            if mean.abs() > limit {
                failures.push(format!("joint {} mean velocity {mean:.4} rad/s (limit {limit})", i + 1));
            }
            if peak > limit * UNLOAD_STILL_VELOCITY_PEAK_FACTOR {
                failures.push(format!("joint {} velocity peak {peak:.4} rad/s (limit {})", i + 1, limit * UNLOAD_STILL_VELOCITY_PEAK_FACTOR));
            }
        }
        (!failures.is_empty()).then(|| failures.join("; "))
    }

    pub fn set_torque(&mut self, enable: bool) -> Result<SafetyState, SafetyError> {
        // Torque changes and re-arming are refused in gravity compensation: exit first.
        self.gravity_comp_servo_busy()?;
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
            self.unload_settle = None;
            self.watchdog_park = None;
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
            self.torque_off(false)
        }
    }

    /// Torque off at the resting pose. `stillness_verified` is set by the unload settle,
    /// which judged stationarity over its hold window; otherwise the current velocity
    /// sample must be within the resting-pose limit.
    fn torque_off(&mut self, stillness_verified: bool) -> Result<SafetyState, SafetyError> {
        if self.safety.state().is_stopped() || self.e_stop_latched.load(Ordering::SeqCst) {
            return Err(SafetyError::MotionForbiddenInFaultState(
                "Cannot disable torque in EStop or Fault. Call clear_fault first.".into(),
            ));
        }

        let violation = if stillness_verified { self.resting_pose_static_violation() } else { self.resting_pose_violation() };
        if let Some(reason) = violation {
            tracing::warn!("Torque off refused, not at resting pose: {reason}");
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

    /// Automatic unload requires a configured resting pose and observable
    /// velocity, supporting torque and enable state. This is a capability check,
    /// not a claim that the arm is currently resting or ready to disable.
    pub fn supports_automatic_unload(&self) -> bool {
        self.driver.as_ref().is_none_or(|driver| {
            self.config.resting_pose.is_some() && driver.lock().is_ok_and(|driver| {
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
                        self.unload_settle = None;
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
                        &self.planning_limits(),
                    ) {
                        // Park is exempt from the workspace check: it must always remain possible.
                        let samples = Self::trajectory_samples(&traj);
                        if let Err(e) = self.check_workspace_path(&clamped_start, samples.iter().map(Vec::as_slice)) {
                            tracing::warn!("Park path leaves the workspace ({e}); parking anyway");
                        }
                        let _ = self.safety.set_torque(true);
                        let _ = self.safety.mark_motion_started();
                        self.unload_planned_duration = Some(traj.duration);
                        self.active_trajectory = Some(ActiveTrajectory::quintic(traj, None));
                        self.target_positions = home;
                        self.pending_normal_unload = true;
                        self.unload_settle = None;
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
                    self.unload_settle = None;
                    self.active_trajectory = None;
                    self.servo_target = None;
                    return;
                }
                // If not torqued, ensure state is clean
                self.pending_normal_unload = false;
                self.unload_settle = None;
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
                self.exit_gravity_comp("ESTOP", "E-stop");
                // Immediate rigid lock at current position, latch E-Stop, advance epoch
                // Maintain active restorative torque hold: do NOT cut power or set is_torqued=false
                self.pending_normal_unload = false;
                self.unload_settle = None;
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
                self.exit_gravity_comp("BUS_LOSS", "CAN feedback lost");
                // Bus loss / disconnect: hold position locked, latch fault, advance epoch
                // Maintain holding torque: DO NOT disable hardware torque to prevent gravitational collapse!
                self.pending_normal_unload = false;
                self.unload_settle = None;
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
        if self.gravity_comp_blocks_clear() {
            return Err(SafetyError::ClearFaultRejected("Gravity compensation: the switch back to POS_VEL is not confirmed yet".into()));
        }
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
                && driver.lock().is_ok_and(|drv| {
                    if !drv.capabilities().enable_feedback {
                        true
                    } else {
                        drv.enable_state() == EnableState::Disabled
                    }
                }),
        }
    }

    pub fn is_fully_settled_hold(&self) -> bool {
        if self.gravity_comp.is_some() { return false; }
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

        self.cancel_watchdog_park_for_new_lease(conn);

        // Four-category state categorization (strictly fail-closed, no unwrap/panic, driver lock poison returns false)
        let confirmed_hardware_disabled = match &self.driver {
            None => !self.is_torqued && !self.disable_pending && self.safety.state() == &SafetyState::Idle,
            Some(driver) => !self.is_torqued
                && !self.disable_pending
                && self.safety.state() == &SafetyState::Idle
                && driver.lock().is_ok_and(|drv| {
                    if !drv.capabilities().enable_feedback {
                        true
                    } else {
                        drv.enable_state() == EnableState::Disabled
                    }
                }),
        };

        if confirmed_hardware_disabled
            || self.safety.state().is_stopped()
            || self.is_fully_settled_armed_hold()
        {
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
            self.last_evicted = evicted.clone();
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
        // A new controller never inherits an expired controller's queued motion,
        // nor a watchdog park started after the previous lease was released.
        self.cancel_watchdog_park_for_new_lease(client_id);
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

    /// A new lease owner takes over from a running watchdog park: the park's
    /// motion is cancelled like any other queued motion and the arm brakes to a hold.
    fn cancel_watchdog_park_for_new_lease(&mut self, client_id: &str) {
        if self.watchdog_park.take().is_some() && self.pending_normal_unload {
            tracing::info!("Watchdog park cancelled: new lease acquired by {client_id}; holding position");
            self.stop_motion();
        }
    }

    /// Budget for a watchdog park, in control-loop time: deceleration of the physical arm
    /// from its measured velocity (max over joints of |v| / planning acceleration), the
    /// planned park duration (which already starts from the commanded velocity), the
    /// settle hold and [`WATCHDOG_PARK_MARGIN`]; never less than [`WATCHDOG_PARK_TIMEOUT`].
    pub fn watchdog_park_timeout(&self) -> Duration {
        let decel = self.measured_velocities.iter().zip(self.planning_limits())
            .filter(|(v, l)| v.is_finite() && l.max_acceleration > 1e-6)
            .map(|(v, l)| f64::from(v.abs() / l.max_acceleration))
            .fold(0.0, f64::max);
        let planned = self.unload_planned_duration
            .filter(|d| d.is_finite() && *d >= 0.0)
            .map_or(Duration::ZERO, |d| Duration::from_secs_f64((decel + d).min(3600.0)) + UNLOAD_SETTLE_HOLD + WATCHDOG_PARK_MARGIN);
        planned.max(WATCHDOG_PARK_TIMEOUT)
    }

    /// Advance a running watchdog park by `dt`. When the budget runs out with the arm at the
    /// resting pose and only the stationary check pending, the budget is extended once by
    /// [`WATCHDOG_PARK_SETTLE_GRACE`]; otherwise torque is held and WatchdogTimeout latched.
    fn supervise_watchdog_park(&mut self, dt: f64) {
        let Some(mut park) = self.watchdog_park.take() else { return; };
        if !self.pending_normal_unload {
            tracing::warn!("Watchdog park interrupted (safety state {:?})", self.safety.state());
            if self.is_torqued && !self.safety.state().is_stopped() {
                self.latch_watchdog_fault();
            }
            return;
        }
        if dt.is_finite() && dt > 0.0 { park.elapsed_s += dt; }
        if park.elapsed_s < park.budget_s {
            self.watchdog_park = Some(park);
            return;
        }
        let blocker = self.unload_settle_blocker().unwrap_or_else(|| "settle pending".into());
        let at_pose = self.active_trajectory.is_none() && self.resting_pose_static_violation().is_none();
        if at_pose && !park.grace_granted {
            tracing::warn!("Watchdog park at the resting pose after {:.2} s but not settled ({blocker}); waiting up to {:?} more",
                park.elapsed_s, WATCHDOG_PARK_SETTLE_GRACE);
            park.budget_s += WATCHDOG_PARK_SETTLE_GRACE.as_secs_f64();
            park.grace_granted = true;
            self.watchdog_park = Some(park);
            return;
        }
        tracing::warn!("Watchdog park did not finish within {:.2} s ({blocker}); holding torque and latching WatchdogTimeout", park.budget_s);
        self.latch_watchdog_fault();
    }

    /// Conditions for parking automatically when the lease watchdog expires.
    /// Returns the reason the park is not allowed, or `None` when it is.
    fn watchdog_park_blocker(&self) -> Option<String> {
        let Some(policy) = &self.config.resting_pose else { return Some("no resting pose configured".into()); };
        if !self.supports_automatic_unload() { return Some("driver lacks velocity/torque/enable feedback".into()); }
        if self.e_stop_latched.load(Ordering::SeqCst) { return Some("E-stop latched".into()); }
        if !self.safety.state().is_operational() { return Some(format!("safety state {:?}", self.safety.state())); }
        if self.pending_rearm || self.disable_pending || self.output_suspended || self.handover_settling_since.is_some() {
            return Some("re-arm, disable, output suspension or handover pending".into());
        }
        if !self.hardware_feedback_fresh(Duration::from_millis(policy.feedback_max_age_ms)) {
            return Some(format!("feedback older than {} ms", policy.feedback_max_age_ms));
        }
        None
    }

    /// Previous watchdog behaviour: rigid hold at the measured (or commanded) position
    /// and latch Fault(WatchdogTimeout). Torque stays on.
    fn latch_watchdog_fault(&mut self) {
        let hold_pos = if self.measured_positions.iter().all(|p| p.is_finite())
            && self.measured_positions.len() == self.config.dof
        {
            self.measured_positions.clone()
        } else {
            self.commanded_positions.clone()
        };
        self.watchdog_park = None;
        self.pending_normal_unload = false;
        self.unload_settle = None;
        self.active_trajectory = None;
        self.servo_target = None; self.servo_settled_ticks.fill(0);
        self.target_positions = hold_pos.clone();
        self.commanded_positions = hold_pos;
        self.commanded_velocities = vec![0.0; self.config.dof];
        self.safety.trigger_fault(FaultReason::WatchdogTimeout);
    }

    /// Drops the lease if `client_id`/`generation` own it. Returns true only when the
    /// lease was still valid: callers stop motion on true. An expired lease is dropped
    /// without that, because its expiry may already have started a watchdog park that a
    /// client without a valid lease must not cancel.
    pub fn release_lease(&mut self, client_id: &str, generation: u64) -> bool {
        if let Some(ref active) = self.active_lease {
            if active.client_id == client_id && active.generation == generation {
                let valid = active.acquired_at.elapsed() < self.lease_timeout;
                self.active_lease = None;
                return valid;
            }
        }
        false
    }

    /// Why `client_id` no longer holds `generation`: "preempted" when a forced acquire
    /// evicted exactly that lease, otherwise "expired" (heartbeats stopped arriving).
    pub fn lease_loss_reason(&self, client_id: &str, generation: u64) -> &'static str {
        match &self.last_evicted {
            Some((conn, gen)) if conn == client_id && *gen == generation => "preempted",
            _ => "expired",
        }
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
        self.gravity_comp_servo_busy()?;
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

        if self.pending_rearm || self.pending_normal_unload {
            return Err(SafetyError::NotArmed(self.safety.state().clone()));
        }
        if self.active_trajectory.is_some() || self.grasp_run.is_some() {
            return Err(SafetyError::MotionBusy);
        }

        // 1. Validate motion command against epoch, safety state, and soft limits
        self.safety.validate_motion_command(
            Some(epoch_id),
            &targets,
            &self.config.limits,
        )?;

        // 1b. Cartesian workspace: the target may not leave the box further than now.
        self.check_workspace_path(&self.commanded_positions, [targets.as_slice()])?;

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
        self.jog_action(target_positions, duration_secs, epoch_id, "joint_move").map(|(epoch, _, _)| epoch)
    }

    /// Jog that records an action of `kind` (`joint_move` or `home`).
    /// Returns (epoch, action id, planned duration).
    pub fn jog_action(
        &mut self,
        target_positions: Vec<f32>,
        duration_secs: Option<f64>,
        epoch_id: Option<u64>,
        kind: &str,
    ) -> Result<(u64, u64, f64), ArmError> {
        self.gravity_comp_motion_busy()?;
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
            &self.planning_limits(),
        )?;
        let samples = Self::trajectory_samples(&traj);
        self.check_workspace_path(&self.commanded_positions, samples.iter().map(Vec::as_slice))?;

        // 3. Only renew lease AFTER command validation and planning succeed
        self.last_heartbeat = Instant::now();

        self.servo_target = None;
        self.target_positions = target_positions;
        self.safety.mark_motion_started()?;
        let duration = traj.duration;
        let id = self.begin_action(kind, None);
        self.active_trajectory = Some(ActiveTrajectory::quintic(traj, Some(id)));

        Ok((self.safety.epoch_id(), id, duration))
    }

    // ---- actions ----

    fn now_s(&self) -> f64 { self.start_time.elapsed().as_secs_f64() }

    /// Start a new action record; a running one is cancelled as SUPERSEDED.
    pub fn begin_action(&mut self, kind: &str, tag: Option<String>) -> u64 {
        let now = self.now_s();
        // any new action ends a running grasp (its orchestrator sees the record cancelled)
        self.grasp_run = None;
        if let Some(mut old) = self.last_action.take() {
            if old.is_running() {
                old.state = "cancelled".into();
                old.code = Some("SUPERSEDED".into());
                old.message = Some(format!("Replaced by a new {kind} action"));
                old.finished_at = Some(now);
            }
            self.previous_action = Some((old, now + 2.0));
        }
        self.next_action_id += 1;
        let id = self.next_action_id;
        self.last_action = Some(ActionStatus {
            id, kind: kind.into(), tag, state: "running".into(), code: None, message: None, progress: 0.0,
            started_at: now, finished_at: None, holding: None, gripper_gap_rad: None, settled: None,
            phase: None, iteration: None, error_px: None,
        });
        self.action_lease_generation = self.active_lease.as_ref().map(|l| l.generation);
        self.quintic_settle = None;
        id
    }

    fn finish_action(&mut self, id: u64, state: &str, code: Option<&str>, message: Option<String>,
                     settled: Option<bool>, holding: Option<bool>, gap: Option<f32>) {
        let now = self.now_s();
        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id && a.is_running()) {
            a.state = state.into();
            a.code = code.map(Into::into);
            a.message = message;
            a.finished_at = Some(now);
            if state == "done" { a.progress = 1.0; }
            if settled.is_some() { a.settled = settled; }
            if holding.is_some() { a.holding = holding; }
            if gap.is_some() { a.gripper_gap_rad = gap; }
            tracing::info!("Action {} ({}) {}{}", id, a.kind, state, code.map(|c| format!(" {c}")).unwrap_or_default());
        }
        if self.quintic_settle.as_ref().is_some_and(|(sid, _)| *sid == id) { self.quintic_settle = None; }
        if self.grasp_run.as_ref().is_some_and(|g| g.id == id) { self.grasp_run = None; }
    }

    /// Start a park (NormalUnload) and record it as a `park` action. Returns its id when
    /// the park started.
    pub fn unload_action(&mut self) -> Option<u64> {
        self.execute_stop(StopPolicy::NormalUnload);
        if !self.pending_normal_unload { return None; }
        let id = self.begin_action("park", None);
        if let Some(active) = self.active_trajectory.as_mut() { active.action_id = Some(id); }
        Some(id)
    }

    /// Preconditions shared by Cartesian and gripper actions.
    fn action_preconditions(&self) -> Result<(), ArmError> {
        if self.handover_settling_since.is_some() { return Err(SafetyError::HandoverSettling.into()); }
        if self.e_stop_latched.load(Ordering::SeqCst) { return Err(SafetyError::EStopActive.into()); }
        match self.safety.state() {
            SafetyState::EStop => return Err(SafetyError::EStopActive.into()),
            SafetyState::Fault(r) => return Err(SafetyError::FaultActive(r.clone()).into()),
            s if !s.is_operational() => return Err(SafetyError::NotArmed(s.clone()).into()),
            _ => {}
        }
        if self.pending_rearm || self.pending_normal_unload {
            return Err(SafetyError::NotArmed(self.safety.state().clone()).into());
        }
        if let Some(reason) = self.busy_reason() { return Err(MotionError::MotionBusy(reason).into()); }
        Ok(())
    }

    fn busy_reason(&self) -> Option<String> {
        if self.active_trajectory.is_some() { return Some("a trajectory is running".into()); }
        if self.servo_target.is_some() { return Some("servo streaming is active".into()); }
        if self.commanded_velocities.iter().any(|v| v.abs() >= 1e-3) { return Some("the arm is still moving".into()); }
        None
    }

    /// Snapshot for planning a Cartesian action outside the lock.
    pub fn plan_input(&self) -> Result<PlanInput, ArmError> {
        if self.config.kinematics.is_none() || self.config.dof != 7 {
            return Err(MotionError::Unsupported("Cartesian actions need a 7-DOF profile with a kinematics model".into()).into());
        }
        self.action_preconditions()?;
        self.gravity_comp_motion_busy()?;
        Ok(PlanInput {
            q0: crate::kinematics::q6(&self.commanded_positions),
            j7: self.commanded_positions[6],
            limits: self.planning_limits(),
            ws: self.workspace_policy().map(|(ws, _)| ws.clone()),
            cam: self.config.camera_mount.clone(),
            epoch: self.safety.epoch_id(),
        })
    }

    /// Install a planned path if nothing changed since `input` was taken.
    /// Returns (action id, expected duration).
    pub fn install_path(&mut self, input: &PlanInput, path: JointPath, kind: &str, tag: Option<String>) -> Result<(u64, f64), ArmError> {
        self.check_plan_fresh(input)?;
        let duration = self.start_path_motion(input, &path)?;
        let id = self.begin_action(kind, tag);
        self.active_trajectory = Some(ActiveTrajectory { action_id: Some(id), reference: Reference::Path(PathTrajectory::new(path)) });
        Ok((id, duration + 0.5))
    }

    /// PLAN_STALE checks shared by every installed plan.
    fn check_plan_fresh(&self, input: &PlanInput) -> Result<(), ArmError> {
        if self.safety.epoch_id() != input.epoch {
            return Err(MotionError::PlanStale("epoch changed while planning".into()).into());
        }
        match self.action_preconditions() {
            Err(ArmError::Motion(MotionError::MotionBusy(reason))) => {
                return Err(MotionError::PlanStale(format!("arm no longer idle ({reason})")).into());
            }
            other => other?,
        }
        let moved = (0..6).any(|i| (f64::from(self.commanded_positions[i]) - input.q0[i]).abs() >= 1e-4)
            || (self.commanded_positions[6] - input.j7).abs() >= 1e-4;
        if moved { return Err(MotionError::PlanStale("commanded positions changed while planning".into()).into()); }
        Ok(())
    }

    fn start_path_motion(&mut self, input: &PlanInput, path: &JointPath) -> Result<f64, ArmError> {
        let last = path.last().to_vec();
        self.safety.validate_motion_command(Some(input.epoch), &last, &self.config.limits)?;
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        self.target_positions = last;
        self.safety.mark_motion_started()?;
        self.last_heartbeat = Instant::now();
        Ok(path.duration)
    }

    // ---- grasp ----

    /// Runtime grasp setup (`config.grasp`, with the camera tilt taken from `camera_mount`).
    /// None when the profile cannot grasp (no kinematics, gripper policy or grasp section).
    pub fn grasp_setup(&self) -> Option<crate::grasp::GraspSetup> {
        if self.config.dof != 7 || self.config.kinematics.is_none() || self.config.gripper_policy.is_none() { return None; }
        let mut s = self.config.grasp.clone()?;
        s.camera_tilt_deg = f64::from(self.config.camera_mount.tilt_deg);
        Some(s)
    }

    /// Replace the runtime setup; `camera_tilt_deg` also sets `camera_mount.tilt_deg`.
    pub fn set_grasp_setup(&mut self, setup: crate::grasp::GraspSetup) {
        self.config.camera_mount.tilt_deg = setup.camera_tilt_deg as f32;
        self.config.grasp = Some(setup);
    }

    /// Begin a `grasp` action after its approach was planned from `input`. Returns its id.
    pub fn begin_grasp(&mut self, input: &PlanInput, tag: Option<String>) -> Result<u64, ArmError> {
        self.check_plan_fresh(input)?;
        let id = self.begin_action("grasp", tag);
        self.grasp_run = Some(crate::grasp::GraspRun { id, epoch: input.epoch, sub: crate::grasp::SubState::Idle, stop_requested: false });
        self.set_grasp_phase(id, "start", 0.0, None, None);
        Ok(id)
    }

    /// The grasp `id` is still the running action.
    pub fn grasp_alive(&self, id: u64) -> bool {
        self.grasp_run.as_ref().is_some_and(|g| g.id == id)
            && self.last_action.as_ref().is_some_and(|a| a.id == id && a.is_running())
    }

    pub fn grasp_sub(&self, id: u64) -> Option<crate::grasp::SubState> {
        self.grasp_run.as_ref().filter(|g| g.id == id).map(|g| g.sub.clone())
    }

    pub fn set_grasp_phase(&mut self, id: u64, phase: &str, progress: f32, iteration: Option<u32>, error_px: Option<f32>) {
        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id && a.is_running()) {
            a.phase = Some(phase.into());
            a.progress = progress;
            // centring stats stay in the record after the phase (result of the closed loop)
            if iteration.is_some() { a.iteration = iteration; }
            if error_px.is_some() { a.error_px = error_px; }
        }
    }

    fn grasp_sub_ready(&self, id: u64) -> Result<u64, ArmError> {
        let run = self.grasp_run.as_ref().filter(|g| g.id == id && self.grasp_alive(id))
            .ok_or_else(|| ArmError::Internal("grasp is no longer running".into()))?;
        if run.sub == crate::grasp::SubState::Running {
            return Err(MotionError::MotionBusy("a grasp motion is running".into()).into());
        }
        Ok(run.epoch)
    }

    /// Install a planned sub-motion of grasp `id`. Returns its planned duration.
    pub fn install_grasp_path(&mut self, id: u64, input: &PlanInput, path: JointPath) -> Result<f64, ArmError> {
        let epoch = self.grasp_sub_ready(id)?;
        if input.epoch != epoch { return Err(MotionError::PlanStale("epoch changed during the grasp".into()).into()); }
        self.check_plan_fresh(input)?;
        let duration = self.start_path_motion(input, &path)?;
        self.active_trajectory = Some(ActiveTrajectory { action_id: Some(id), reference: Reference::Path(PathTrajectory::new(path)) });
        if let Some(g) = self.grasp_run.as_mut() { g.sub = crate::grasp::SubState::Running; }
        Ok(duration + 0.5)
    }

    /// Start a gripper sub-motion of grasp `id`. Returns its expected duration.
    pub fn start_grasp_gripper(&mut self, id: u64, request: GripperRequest) -> Result<f64, ArmError> {
        let epoch = self.grasp_sub_ready(id)?;
        let Some(policy) = self.config.gripper_policy.clone().filter(|_| self.config.dof >= 7) else {
            return Err(MotionError::Unsupported("Gripper actions need a 7-DOF profile with a gripper policy".into()).into());
        };
        if epoch != self.safety.epoch_id() { return Err(SafetyError::EpochMismatch { expected: self.safety.epoch_id(), received: epoch }.into()); }
        self.action_preconditions()?;
        let motion = GripperMotion::new(request, policy, self.config.limits[6].clone(), self.commanded_positions[6]);
        let duration = motion.expected_duration();
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        let mut target = self.commanded_positions.clone();
        target[6] = motion.target;
        self.target_positions = target;
        self.safety.mark_motion_started()?;
        self.last_heartbeat = Instant::now();
        self.active_trajectory = Some(ActiveTrajectory { action_id: Some(id), reference: Reference::Gripper(motion) });
        if let Some(g) = self.grasp_run.as_mut() { g.sub = crate::grasp::SubState::Running; }
        Ok(duration)
    }

    /// End grasp `id` from its orchestrator (no-op when it already ended).
    pub fn finish_grasp(&mut self, id: u64, state: &str, code: Option<&str>, message: Option<String>, holding: Option<bool>, gap: Option<f32>) {
        if !self.grasp_alive(id) { return; }
        if self.active_trajectory.as_ref().is_some_and(|a| a.action_id == Some(id)) { self.stop_motion(); }
        if state == "done" { self.set_grasp_phase(id, "done", 1.0, None, None); }
        self.finish_action(id, state, code, message, None, holding, gap);
    }

    /// `/motion/stop`: also ends a grasp waiting between motions.
    pub fn request_grasp_stop(&mut self) {
        if let Some(g) = self.grasp_run.as_mut() { g.stop_requested = true; }
    }

    /// Start a gripper action. Returns (action id, expected duration).
    pub fn start_gripper(&mut self, request: GripperRequest, epoch: u64) -> Result<(u64, f64), ArmError> {
        let Some(policy) = self.config.gripper_policy.clone().filter(|_| self.config.dof >= 7) else {
            return Err(MotionError::Unsupported("Gripper actions need a 7-DOF profile with a gripper policy".into()).into());
        };
        if epoch != self.safety.epoch_id() {
            return Err(SafetyError::EpochMismatch { expected: self.safety.epoch_id(), received: epoch }.into());
        }
        self.action_preconditions()?;
        let motion = GripperMotion::new(request, policy, self.config.limits[6].clone(), self.commanded_positions[6]);
        let duration = motion.expected_duration();
        let tag = motion.request.tag.clone();
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        let mut target = self.commanded_positions.clone();
        target[6] = motion.target;
        self.target_positions = target;
        self.safety.mark_motion_started()?;
        self.last_heartbeat = Instant::now();
        let id = self.begin_action("gripper", tag);
        self.active_trajectory = Some(ActiveTrajectory { action_id: Some(id), reference: Reference::Gripper(motion) });
        Ok((id, duration))
    }

    /// Single place that ends a running action whose reference went away (spec 2.3).
    fn reconcile_action(&mut self, dt: f64) {
        let Some(action) = self.last_action.as_ref().filter(|a| a.is_running()) else { return; };
        let (id, kind) = (action.id, action.kind.clone());
        let carried = self.active_trajectory.as_ref().is_some_and(|a| a.action_id == Some(id));
        let lease_now = self.active_lease.as_ref().map(|l| l.generation);
        let cause: Option<(&str, &str, String)> = match self.safety.state() {
            SafetyState::EStop => Some(("failed", "ESTOP_ACTIVE", "E-stop latched".into())),
            SafetyState::Fault(FaultReason::WatchdogTimeout) => Some(("failed", "WATCHDOG", "Lease watchdog expired".into())),
            SafetyState::Fault(r) => Some(("failed", "FAULT", match r { FaultReason::Other(m) => m.clone(), r => format!("{r:?}") })),
            _ if kind != "park" && self.watchdog_park.is_some() => Some(("failed", "WATCHDOG", "Lease watchdog expired; parking".into())),
            _ if lease_now != self.action_lease_generation => Some(("cancelled", "LEASE_LOST", "Control lease lost or released".into())),
            _ => None,
        };
        if let Some((state, code, message)) = cause {
            // a grasp sub-motion must not outlive its grasp
            if kind == "grasp" && carried { self.stop_motion(); }
            self.finish_action(id, state, Some(code), Some(message), None, None, None);
            return;
        }
        if kind == "grasp" {
            if carried { return; }
            let superseded = self.active_trajectory.is_some() || self.servo_target.is_some();
            let ended: Option<(&str, &str, String)> = match self.grasp_run.as_ref().filter(|g| g.id == id) {
                None => Some(("failed", "INTERNAL", "Grasp orchestrator ended".into())),
                Some(_) if superseded => Some(("cancelled", "SUPERSEDED", "Replaced by new motion".into())),
                Some(g) if g.sub == crate::grasp::SubState::Running || g.stop_requested => Some(("cancelled", "STOPPED", "Motion stopped".into())),
                Some(_) => None,
            };
            if let Some((state, code, message)) = ended { self.finish_action(id, state, Some(code), Some(message), None, None, None); }
            return;
        }
        let superseded = (self.active_trajectory.is_some() && !carried) || self.servo_target.is_some();
        if kind == "park" {
            if self.pending_normal_unload {
                let progress = self.active_trajectory.as_ref().and_then(|a| a.as_quintic()).map_or(1.0, |t| (t.elapsed / t.duration).min(1.0) as f32);
                if let Some(a) = self.last_action.as_mut() { a.progress = progress; }
            } else if !self.is_torqued && matches!(self.safety.state(), SafetyState::Idle) {
                self.finish_action(id, "done", None, None, None, None, None);
            } else if superseded {
                self.finish_action(id, "cancelled", Some("SUPERSEDED"), Some("Replaced by new motion".into()), None, None, None);
            } else {
                self.finish_action(id, "failed", Some("UNLOAD_FAILED"), Some("Park ended with torque still on".into()), None, None, None);
            }
            return;
        }
        if carried { return; }
        if self.quintic_settle.as_ref().is_some_and(|(sid, _)| *sid == id) {
            if superseded {
                self.finish_action(id, "cancelled", Some("SUPERSEDED"), Some("Replaced by new motion".into()), None, None, None);
                return;
            }
            let (cmd, meas, vel) = (&self.commanded_positions, &self.measured_positions, &self.measured_velocities);
            let result = self.quintic_settle.as_mut().and_then(|(_, settle)| settle.step(dt, cmd, meas, vel));
            if let Some(settled) = result {
                self.finish_action(id, "done", None, None, Some(settled), None, None);
            }
            return;
        }
        if superseded {
            self.finish_action(id, "cancelled", Some("SUPERSEDED"), Some("Replaced by new motion".into()), None, None, None);
        } else {
            self.finish_action(id, "cancelled", Some("STOPPED"), Some("Motion stopped".into()), None, None, None);
        }
    }

    /// Apply one executor step of a Path / Gripper reference.
    fn apply_action_step(&mut self, action_id: Option<u64>, out: StepOut) {
        self.commanded_positions = out.positions;
        self.commanded_velocities = out.velocities;
        if out.clear_gripper_stall {
            self.gripper_stalled = false;
            self.gripper_high_torque_ticks = 0;
            self.gripper_low_torque_ticks = 0;
        }
        let grasp_sub = self.grasp_run.as_ref().is_some_and(|g| Some(g.id) == action_id);
        if let (Some(id), Some(a)) = (action_id, self.last_action.as_mut()) {
            if a.id == id && a.is_running() && !grasp_sub { a.progress = out.progress; }
        }
        match out.status {
            StepStatus::Running => {}
            StepStatus::Done { settled, holding, gripper_gap_rad } => {
                self.active_trajectory = None;
                self.commanded_velocities.fill(0.0);
                self.target_positions = self.commanded_positions.clone();
                self.safety.mark_motion_finished();
                if grasp_sub {
                    if let Some(g) = self.grasp_run.as_mut() {
                        g.sub = crate::grasp::SubState::Done(crate::grasp::SubDone { settled, holding, gripper_gap_rad, at: Instant::now() });
                    }
                } else if let Some(id) = action_id { self.finish_action(id, "done", None, None, settled, holding, gripper_gap_rad); }
            }
            StepStatus::Failed { code, message } => {
                if code == "TRACKING_STALL" {
                    self.stop_motion();
                    if self.measured_positions.iter().all(|q| q.is_finite()) {
                        self.commanded_positions = self.measured_positions.clone();
                    }
                    self.commanded_accelerations.fill(0.0);
                } else {
                    self.active_trajectory = None;
                }
                self.commanded_velocities.fill(0.0);
                self.target_positions = self.commanded_positions.clone();
                self.safety.mark_motion_finished();
                if grasp_sub {
                    if let Some(g) = self.grasp_run.as_mut() { g.sub = crate::grasp::SubState::Failed { code: code.into(), message }; }
                } else if let Some(id) = action_id { self.finish_action(id, "failed", Some(code), Some(message), None, None, None); }
            }
        }
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

    /// One control tick of the unload settle: the static resting-pose conditions must hold
    /// continuously for [`UNLOAD_SETTLE_HOLD`] and the arm must be stationary over that
    /// window ([`Self::unload_still_violation`]); then torque is cut. A settle pending past
    /// the hold logs which condition blocks it, at most once per second.
    fn step_unload_settle(&mut self, dt: f64) {
        let mut settle = self.unload_settle.take().unwrap_or_default();
        settle.pending_s += dt;
        let static_violation = self.resting_pose_static_violation();
        if static_violation.is_none() {
            let since = *settle.held_since_s.get_or_insert(settle.pending_s);
            settle.held_s = settle.pending_s - since;
            settle.samples.push_back((settle.pending_s, self.measured_positions.clone(), self.measured_velocities.clone()));
            let oldest = settle.pending_s - UNLOAD_SETTLE_HOLD.as_secs_f64() - 1e-6;
            while settle.samples.front().is_some_and(|(t, _, _)| *t < oldest) { settle.samples.pop_front(); }
        } else {
            settle.held_s = 0.0;
            settle.held_since_s = None;
            settle.samples.clear();
        }
        let blocker = static_violation.or_else(|| self.unload_still_violation(&settle));
        let Some(blocker) = blocker else {
            self.pending_normal_unload = false;
            tracing::info!("Arm settled cleanly at home position for {} ms after {:.2} s; safely cutting torque to Standby/Idle.",
                UNLOAD_SETTLE_HOLD.as_millis(), settle.pending_s);
            let watchdog_park = self.watchdog_park.take().is_some();
            match self.torque_off(true) {
                Ok(_) if watchdog_park => tracing::info!("Watchdog park complete: torque off at resting pose"),
                Ok(_) => {}
                Err(e) => {
                    tracing::warn!("Failed to cut torque after normal unload settle: {}", e);
                    self.safety.mark_motion_finished();
                    if watchdog_park && self.is_torqued && !self.safety.state().is_stopped() {
                        tracing::warn!("Watchdog park failed at torque off; holding torque and latching WatchdogTimeout");
                        self.latch_watchdog_fault();
                    }
                }
            }
            return;
        };
        let report_after = UNLOAD_SETTLE_HOLD.as_secs_f64() + UNLOAD_SETTLE_REPORT_AFTER_S;
        if settle.pending_s >= report_after && settle.last_report_s.is_none_or(|t| settle.pending_s - t >= 1.0) {
            tracing::warn!("Unload settle pending for {:.1} s: {blocker}", settle.pending_s);
            settle.last_report_s = Some(settle.pending_s);
        }
        // Not yet stably at rest: hold the home target.
        self.commanded_positions = self.config.home_positions.clone();
        self.target_positions = self.config.home_positions.clone();
        self.commanded_velocities = vec![0.0; self.config.dof];
        self.unload_settle = Some(settle);
    }

    /// Blocker of a pending unload settle, for watchdog reporting.
    fn unload_settle_blocker(&self) -> Option<String> {
        if self.active_trajectory.is_some() { return Some("park trajectory still running".into()); }
        self.resting_pose_static_violation()
            .or_else(|| self.unload_settle.as_ref().and_then(|s| self.unload_still_violation(s)))
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
                    self.last_can_feedback_instant.is_some_and(|t| t.elapsed() < Duration::from_millis(50)) && driver.is_feedback_valid() && driver.capabilities().torque_feedback
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
                // A stall is high torque with the jaw not moving. Torque alone also rises
                // from friction during free travel over the 5.9 rad stroke.
                let v_7 = self.measured_velocities.get(6).copied().unwrap_or(0.0);
                if tau_7.abs() >= GRIPPER_STALL_LATCH_NM && v_7.abs() < 0.10 {
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
                // Exit condition B: |tau_7| < GRIPPER_STALL_RELEASE_NM for 6 consecutive ticks
                if tau_7.abs() < GRIPPER_STALL_RELEASE_NM {
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

        // Bus loss: no fresh feedback from every joint within BUS_LOSS_TIMEOUT.
        if self.is_torqued
            && !self.safety.state().is_stopped()
            && self.driver.is_some()
            && !self.hardware_feedback_fresh(BUS_LOSS_TIMEOUT)
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
                        self.unload_settle = None;
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

        // 0.3 Gravity compensation: exits (to a hold) on any safety event before the
        // watchdog below may start a park, which then runs from position mode.
        self.gravity_comp_tick(dt);

        // 1. Check lease watchdog (skip during autonomous stow/unload)
        if self.lease_enabled
            && self.gravity_comp.is_none()
            && self.is_torqued
            && self.safety.state().is_operational()
            && !self.pending_normal_unload
            && self.last_heartbeat.elapsed() > self.lease_timeout
        {
            // Watchdog expired with torque on. When the arm is healthy and has a resting
            // pose, run the normal unload (park at home, settle, torque off). Otherwise
            // rigid lock at the current position and latch the fault, keeping torque on.
            match self.watchdog_park_blocker() {
                None => {
                    tracing::warn!("Watchdog: lease expired while torqued, parking");
                    self.apply_stop_policy(StopPolicy::NormalUnload);
                    if self.pending_normal_unload {
                        let budget = self.watchdog_park_timeout();
                        tracing::info!("Watchdog park budget {:.2} s (planned {:.2} s)", budget.as_secs_f64(), self.unload_planned_duration.unwrap_or(0.0));
                        self.watchdog_park = Some(WatchdogPark { elapsed_s: 0.0, budget_s: budget.as_secs_f64(), grace_granted: false });
                    } else {
                        tracing::warn!("Watchdog park could not be planned; holding torque and latching WatchdogTimeout");
                        self.latch_watchdog_fault();
                    }
                }
                Some(reason) => {
                    tracing::warn!("Watchdog: lease expired while torqued, not parking ({reason}); holding torque and latching WatchdogTimeout");
                    self.latch_watchdog_fault();
                }
            }
        }

        // Watchdog park supervision: a park that stops without completing, or runs past
        // its budget, falls back to the hold + Fault(WatchdogTimeout) behaviour.
        self.supervise_watchdog_park(dt);

        // Thermal protection remains active while stopped or shutdown is unconfirmed.
        let temperature_available = self.driver.as_ref().is_none_or(|d| d.lock().is_ok_and(|drv| drv.capabilities().temperature_feedback));
        if (self.is_torqued || self.disable_pending) && temperature_available
            && self.temperatures.iter().any(|t| t.is_finite() && *t > self.config.max_temperature)
                && !matches!(self.safety.state(), SafetyState::Fault(FaultReason::ThermalTrip { .. })) {
                self.apply_stop_policy(StopPolicy::ThermalTrip);
            }
        if self.disable_pending {
            let confirmed = self.driver.as_ref().is_none_or(|d| d.lock().is_ok_and(|drv|
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
            // The 7-DOF gripper is excluded: Damiao POS_VEL trails its command by about
            // v / 2.2 rad (0.9 rad at 2 rad/s), and gripping an object leaves a lasting
            // error by design. Its own stall detection covers it.
            let gripper_idx = (self.config.dof >= 7).then_some(6);
            let outside = self.commanded_positions.iter().zip(&self.measured_positions).enumerate()
                .find(|(i, (q, measured))| Some(*i) != gripper_idx
                    && (!measured.is_finite() || (*q - *measured).abs() > threshold))
                .map(|(i, (q, measured))| (i, *q, *measured));
            if let Some((joint, cmd, meas)) = outside {
                let since = self.following_error_since.get_or_insert_with(Instant::now);
                if since.elapsed() >= Duration::from_millis(self.config.following_error.persistence_ms) {
                    self.stop_motion();
                    self.commanded_positions = self.measured_positions.clone();
                    self.target_positions = self.measured_positions.clone();
                    self.commanded_velocities.fill(0.0);
                    self.commanded_accelerations.fill(0.0);
                    self.safety.trigger_fault(FaultReason::Other(format!(
                        "Persistent joint tracking error: joint {} cmd {cmd:.3} meas {meas:.3} rad", joint + 1)));
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
        enum Stepped { Quintic(crate::trajectory::TrajectoryPoint, bool, f32), Action(StepOut) }
        let stepped = if let Some(active) = self.active_trajectory.as_mut() {
            let ctx = StepCtx {
                dt: servo_dt,
                measured: &self.measured_positions,
                measured_velocities: &self.measured_velocities,
                commanded: &self.commanded_positions,
                commanded_velocities: &self.commanded_velocities,
                torques: &self.torques,
                gripper_stalled: self.gripper_stalled,
                gripper_stall_sign: self.gripper_stall_sign,
            };
            let out = match &mut active.reference {
                Reference::Quintic(traj) => {
                    let point = traj.step(dt);
                    Stepped::Quintic(point, traj.is_finished(), (traj.elapsed / traj.duration).min(1.0) as f32)
                }
                Reference::Path(path) => Stepped::Action(path.step(&ctx)),
                Reference::Gripper(grip) => Stepped::Action(grip.step(&ctx)),
            };
            Some((active.action_id, out))
        } else { None };
        match stepped {
            None => {}
            Some((action_id, Stepped::Action(out))) => {
                reference_stepped = true;
                self.apply_action_step(action_id, out);
            }
            Some((action_id, Stepped::Quintic(point, finished, progress))) => {
                reference_stepped = true;
                self.commanded_positions = point.positions;
                self.commanded_velocities = point.velocities;
                if let (Some(id), Some(a)) = (action_id, self.last_action.as_mut()) {
                    if a.id == id && a.is_running() { a.progress = progress; }
                }
                if finished {
                    self.active_trajectory = None;
                    self.commanded_velocities = vec![0.0; self.config.dof];
                    if !self.pending_normal_unload {
                        self.safety.mark_motion_finished();
                        if let Some(id) = action_id { self.quintic_settle = Some((id, Settle::default())); }
                    }
                }
            }
        }

        // 3.1 Handle pending normal unload settling phase
        if self.pending_normal_unload && self.active_trajectory.is_none() {
            self.step_unload_settle(servo_dt);
        } else if let Some(ref target) = self.servo_target.clone() {
            // Check continuous motion frame timeout:
            // If > 200ms without a new pose frame, clear servo_target to begin controlled smooth deceleration
            if self.last_motion_frame.elapsed() > self.motion_frame_timeout {
                self.servo_target = None;
                self.servo_settled_ticks.fill(0);
            } else {
                // Check if any target joint has instantaneous jump > 0.35 rad relative to commanded_positions
                // The 7-DOF gripper is exempt: its far targets are expected (POS_VEL needs a
                // lead to reach speed) and the per-joint servo step below still bounds its
                // velocity and acceleration.
                let gripper_idx = (self.config.dof >= 7).then_some(6);
                let has_dirty_jump = target.iter().enumerate().any(|(i, &tgt)| {
                    if Some(i) == gripper_idx {
                        false
                    } else if i < self.commanded_positions.len() {
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
                    // teleop streaming keeps the raw joint limits: clients bound their own lead (the cockpit keeps
                    // it under 0.2 rad), so the tracking error cannot approach the fault threshold here
                    let step_limits = self.config.limits.clone();
                    for (i, &tgt) in target.iter().enumerate() {
                        if i < self.config.dof {
                            let limit = &step_limits[i];
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
            && self.gravity_comp.is_none()
            && !self.safety.state().is_stopped()
        {
            // A stale/dropped target requests zero velocity. The same feasible
            // interval governs braking, with no small-velocity truncation.
            let mut next_positions = self.commanded_positions.clone();
            let mut next_velocities = self.commanded_velocities.clone();
            let mut infeasible = None;
            let step_limits = self.config.limits.clone(); // braking a stream: same limits as streaming
            for i in 0..self.config.dof {
                let step = Self::bounded_servo_step_with_jerk(
                    self.commanded_positions[i],
                    self.commanded_positions[i],
                    self.commanded_velocities[i],
                    servo_dt,
                    &step_limits[i],
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

        let feedback_fresh = self.hardware_feedback_fresh(BUS_LOSS_TIMEOUT);
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
        if self.is_torqued && !self.disable_pending && self.gravity_comp.is_some() {
            // Gravity compensation (any phase): MIT targets for J1-J6. Sent without the
            // feedback-freshness gate so that an exit hold reaches the motors even when
            // feedback is late; a floating phase never outlives fresh feedback (it exits).
            if let (Some(targets), Some(d)) = (self.gravity_comp_targets(), self.driver.as_ref()) {
                if let Ok(mut driver) = d.lock() {
                    if driver.enable_state() != EnableState::Disabled {
                        if let Err(e) = driver.write_joint_targets(&targets) { output_error = Some(e.to_string()); }
                    }
                }
            }
        } else if self.is_torqued && !self.disable_pending {
            let send_to_bus = feedback_fresh && self.driver.as_ref().is_none_or(|d|
                d.lock().is_ok_and(|drv| drv.enable_state() != EnableState::Disabled));

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

        self.reconcile_action(servo_dt);
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
        let mut schema = ObservationSchema::from_config(&inner.config);
        schema.capabilities.gravity_comp = inner.gravity_comp_support().is_ok();
        schema
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
        let mut inner = self.inner.write().await;
        let home_pos = inner.config.home_positions.clone();
        inner.jog_action(home_pos, duration_secs, epoch_id, "home").map(|(epoch, _, _)| epoch)
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
        inner.pending_normal_unload || inner.active_trajectory.is_some() || inner.unload_settle.is_some()
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
        self.save_recording_with(write_frames_to_parquet).await
    }

    /// Save with a caller-supplied writer. The lock is held only to move the
    /// frames out and to record the outcome; the write itself runs on a blocking
    /// thread so a slow disk cannot stall the control tick, which needs the same lock.
    #[doc(hidden)]
    pub async fn save_recording_with<W>(&self, write: W) -> Result<PathBuf, ArmError>
    where
        W: FnOnce(&Path, &[RecordFrame]) -> Result<(), ParquetRecordingError> + Send + 'static,
    {
        let (path, frames) = self.inner.write().await.recorder.begin_save()?;
        let (frames, result) = tokio::task::spawn_blocking(move || {
            let result = write(&path, &frames);
            (frames, result)
        })
        .await
        .unwrap_or_else(|e| {
            // Frames are lost with the panicked task; leave Finalizing so a new
            // recording can start.
            let err = std::io::Error::other(format!("recording writer panicked: {e}"));
            (Vec::new(), Err(ParquetRecordingError::Io(err)))
        });
        Ok(self.inner.write().await.recorder.finish_save(frames, result)?)
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
