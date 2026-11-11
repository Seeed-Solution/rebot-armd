//! Gravity compensation ("hand-guiding / teach") mode.
//!
//! J1-J6 are switched from Damiao POS_VEL to MIT and float: kp = 0, damping kd and the
//! URDF gravity torque times `scale` as feed-forward. The gripper (J7) stays in POS_VEL
//! at its commanded position. Every transition keeps the arm held:
//!
//! * entry: the switch frame for each joint carries an MIT hold at the measured pose
//!   with `hold_kp` and the torque the motor was producing (feedback). Over `ramp_in_s`
//!   the stiffness fades to 0 and the feed-forward fades from that torque to the model.
//! * exit (requested or forced): the measured pose is captured, the joints get an MIT
//!   stiff hold there (`hold_kp`, model feed-forward kept) at once, and after
//!   `exit_hold_ms` they are switched back to a POS_VEL hold at that pose. POS_VEL frames
//!   carry no torque, so the feed-forward ends with the switch and the motor's position
//!   loop carries the load from then on.
//!
//! Lease loss, E-stop, faults, bus loss, stale feedback, the watchdog, overspeed and the
//! joint-limit margin all exit this way; none of them cuts torque. The lease watchdog
//! park runs only after the arm is back in position mode.

use std::time::Duration;

use crate::driver::{EnableState, JointMode, JointTarget, RangeCheckStatus};
use crate::model::{GravityCompExit, GravityCompObservation, GravityCompPolicy, ModeReason};
use crate::motion::MotionError;
use crate::safety::{FaultReason, SafetyError, SafetyState};
use crate::supervisor::{ArmError, SupervisorInner};

const ARM: usize = 6;

#[derive(Debug, Clone, PartialEq)]
pub enum GravityCompPhase {
    RampIn,
    Active,
    /// MIT stiff hold at `hold` for `elapsed_s` before switching back to POS_VEL.
    Exiting { hold: [f32; ARM], elapsed_s: f64 },
    /// The switch back to POS_VEL was not confirmed: MIT stiff hold, retried periodically.
    ExitFailed { hold: [f32; ARM], retry_in_s: f64 },
}

impl GravityCompPhase {
    pub fn name(&self) -> &'static str {
        match self {
            Self::RampIn => "ramp_in",
            Self::Active => "active",
            Self::Exiting { .. } => "exiting",
            Self::ExitFailed { .. } => "exit_failed",
        }
    }
    pub fn floating(&self) -> bool { matches!(self, Self::RampIn | Self::Active) }
}

/// Running session.
#[derive(Debug, Clone)]
pub struct GravityCompRun {
    pub phase: GravityCompPhase,
    pub policy: GravityCompPolicy,
    pub elapsed_s: f64,
    pub scale: f32,
    pub damping_kd: [f32; ARM],
    pub entry_pose: [f32; ARM],
    pub entry_torque: [f32; ARM],
    /// Values sent this tick.
    pub tau_ff: [f32; ARM],
    pub kp: [f32; ARM],
    pub kd: [f32; ARM],
    pub position: [f32; ARM],
    pub overspeed_count: u32,
    pub prev_measured: Option<[f32; ARM]>,
    pub lease_generation: Option<u64>,
}

impl GravityCompRun {
    pub fn ramp(&self) -> f32 {
        if self.phase.floating() { (self.elapsed_s / f64::from(self.policy.ramp_in_s)).clamp(0.0, 1.0) as f32 } else { 0.0 }
    }
}

/// Optional per-session overrides from the API.
#[derive(Debug, Clone, Default)]
pub struct GravityCompRequest {
    pub scale: Option<f32>,
    pub damping_kd: Option<Vec<f32>>,
}

fn reason(code: &str, message: impl Into<String>) -> ModeReason { ModeReason { code: code.into(), message: message.into() } }

fn arm6(v: &[f32]) -> [f32; ARM] {
    let mut a = [0.0; ARM];
    for (dst, src) in a.iter_mut().zip(v) { *dst = *src; }
    a
}

impl SupervisorInner {
    pub fn gravity_comp_active(&self) -> bool { self.gravity_comp.is_some() }

    fn gravity_comp_policy(&self) -> Option<&GravityCompPolicy> {
        self.config.gravity_comp.as_ref().filter(|_| self.config.six_plus_gripper() && self.config.gravity_model.is_some())
    }

    /// The profile and driver support the mode (independent of the current state).
    pub fn gravity_comp_support(&self) -> Result<(), ModeReason> {
        if self.gravity_comp_policy().is_none() {
            return Err(reason("UNSUPPORTED", "Gravity compensation needs a 7-DOF profile with a gravity model and a gravity_comp section"));
        }
        if !self.gravity_compensated {
            return Err(reason("UNSUPPORTED", "The URDF gravity model was not found or failed its checksum"));
        }
        if let Some(d) = &self.driver {
            let Ok(drv) = d.lock() else { return Err(reason("UNSUPPORTED", "driver lock poisoned")); };
            if !drv.supports_mit_mode_switch() {
                return Err(reason("UNSUPPORTED", "The driver cannot switch joints to MIT mode"));
            }
            let caps = drv.capabilities();
            if !caps.velocity_feedback || !caps.torque_feedback || !caps.enable_feedback {
                return Err(reason("UNSUPPORTED", "Gravity compensation needs velocity, torque and enable feedback"));
            }
        }
        Ok(())
    }

    /// Model torque at the measured pose, scaled and clamped per joint.
    fn gravity_comp_model_torque(&self, policy: &GravityCompPolicy, scale: f32) -> Option<[f32; ARM]> {
        let tau = self.config.arm_model.gravity_torques(self.measured_positions.get(..ARM)?).ok()?;
        if !tau.iter().all(|t| t.is_finite()) { return None; }
        let mut out = [0.0; ARM];
        for i in 0..ARM {
            let lim = policy.torque_limits_nm[i];
            out[i] = (tau[i] * scale).clamp(-lim, lim);
        }
        Some(out)
    }

    fn gravity_comp_near_limit(&self, policy: &GravityCompPolicy) -> Option<String> {
        let m = policy.joint_limit_margin_rad;
        (0..ARM).find_map(|i| {
            let q = *self.measured_positions.get(i)?;
            let l = self.config.limits.get(i)?;
            (!q.is_finite() || q < l.min_position + m || q > l.max_position - m)
                .then(|| format!("joint {} at {q:.3} rad is within {m} rad of its limit [{}, {}]", i + 1, l.min_position, l.max_position))
        })
    }

    /// Why entering is refused now; None when it is allowed (or the mode is already on).
    pub fn gravity_comp_blocker(&self) -> Option<ModeReason> {
        if let Err(r) = self.gravity_comp_support() { return Some(r); }
        if self.gravity_comp.is_some() { return None; }
        // MIT torque commands are only as right as the mapping ranges: require the
        // startup range check to have passed when the driver ran one.
        let check = self.driver.as_ref().and_then(|d| d.lock().ok().and_then(|drv| drv.range_check()));
        match check.as_ref().map(|c| c.status) {
            Some(RangeCheckStatus::Mismatch) => return Some(reason("DRIVER_RANGE_MISMATCH", check.map(|c| c.message).unwrap_or_default())),
            Some(RangeCheckStatus::Unverified) => return Some(reason("DRIVER_RANGE_UNVERIFIED", check.map(|c| c.message).unwrap_or_default())),
            _ => {}
        }
        let policy = self.gravity_comp_policy()?;
        if self.e_stop_latched.load(std::sync::atomic::Ordering::SeqCst) || self.safety.state() == &SafetyState::EStop {
            return Some(reason("ESTOP_ACTIVE", "E-stop is latched"));
        }
        if let SafetyState::Fault(r) = self.safety.state() { return Some(reason("FAULT_ACTIVE", format!("Fault active: {r:?}"))); }
        if self.handover_settling_since.is_some() { return Some(reason("HANDOVER_SETTLING", "Arm is settling after a control handover")); }
        if !self.is_torqued || self.pending_rearm || self.disable_pending || self.output_suspended
            || !self.safety.state().is_operational() {
            return Some(reason("NOT_ARMED", "Enable the arm first"));
        }
        if self.pending_normal_unload || self.active_trajectory.is_some() || self.servo_target.is_some()
            || self.macro_run.is_some() || self.safety.state() != &SafetyState::Armed
            || self.commanded_velocities.iter().any(|v| v.abs() >= 1e-3) {
            return Some(reason("MOTION_BUSY", "Wait until the arm has stopped"));
        }
        if self.lease_enabled && !self.is_lease_active() { return Some(reason("LEASE_REQUIRED", "The control lease is required")); }
        if !self.hardware_feedback_fresh(Duration::from_millis(policy.feedback_max_age_ms)) || !self.feedback_numeric_valid {
            return Some(reason("FEEDBACK_STALE", format!("Feedback older than {} ms", policy.feedback_max_age_ms)));
        }
        if let Some((i, v)) = self.measured_velocities.iter().take(ARM).enumerate()
            .find(|(_, v)| !v.is_finite() || v.abs() > policy.entry_max_velocity) {
            return Some(reason("GRAVITY_COMP_NOT_STATIONARY", format!("Joint {} moves at {v:.3} rad/s (limit {})", i + 1, policy.entry_max_velocity)));
        }
        if let Some(msg) = self.gravity_comp_near_limit(policy) { return Some(reason("GRAVITY_COMP_JOINT_LIMIT", msg)); }
        if self.gravity_comp_model_torque(policy, policy.scale).is_none() {
            return Some(reason("GRAVITY_MODEL_UNAVAILABLE", "Gravity torque could not be computed at the measured pose"));
        }
        None
    }

    fn reject(code: &str, message: String) -> ArmError {
        let status = match code {
            "UNSUPPORTED" | "GRAVITY_MODEL_UNAVAILABLE" => 422,
            "NOT_ARMED" => 412,
            "INVALID_COMMAND" => 400,
            _ => 409,
        };
        ArmError::Rejected { status, code: code.into(), message }
    }

    /// Enter gravity compensation. Idempotent while the mode is on.
    pub fn enter_gravity_comp(&mut self, request: GravityCompRequest) -> Result<(), ArmError> {
        if self.gravity_comp.is_some() { return Ok(()); }
        self.collision_hold_check()?;
        if let Some(r) = self.gravity_comp_blocker() { return Err(Self::reject(&r.code, r.message)); }
        let policy = self.gravity_comp_policy().cloned().ok_or_else(|| Self::reject("UNSUPPORTED", "no policy".into()))?;
        let [lo, hi] = policy.scale_range;
        let scale = request.scale.unwrap_or(policy.scale);
        if !scale.is_finite() || !(lo..=hi).contains(&scale) {
            return Err(Self::reject("INVALID_COMMAND", format!("scale must be in [{lo}, {hi}]")));
        }
        let damping = match request.damping_kd {
            None => arm6(&policy.damping_kd),
            Some(v) if v.len() == ARM && v.iter().all(|k| k.is_finite() && *k >= 0.0 && *k <= policy.damping_kd_max) => arm6(&v),
            Some(_) => return Err(Self::reject("INVALID_COMMAND", format!("damping_kd must be 6 values in [0, {}]", policy.damping_kd_max))),
        };
        let model = self.gravity_comp_model_torque(&policy, scale)
            .ok_or_else(|| Self::reject("GRAVITY_MODEL_UNAVAILABLE", "Gravity torque could not be computed".into()))?;
        let pose = arm6(&self.measured_positions);
        // Start from the torque the position loop is producing now, so the handover to MIT
        // has no torque step (simulation has no torque feedback: use the model).
        let mut entry_torque = if self.driver.is_some() { arm6(&self.torques) } else { model };
        for (i, t) in entry_torque.iter_mut().enumerate() {
            let lim = policy.torque_limits_nm[i];
            *t = if t.is_finite() { t.clamp(-lim, lim) } else { model[i] };
        }
        let gripper = self.commanded_positions[ARM];
        if let Some(d) = &self.driver {
            let mut drv = d.lock().map_err(|_| ArmError::Internal("driver lock poisoned".into()))?;
            let mut modes = vec![JointMode::Mit; ARM];
            modes.push(JointMode::Position);
            let mut hold: Vec<JointTarget> = (0..ARM)
                .map(|i| JointTarget::mit(pose[i], 0.0, policy.hold_kp[i], policy.hold_kd[i], entry_torque[i])).collect();
            hold.push(JointTarget::position(gripper));
            if let Err(e) = drv.set_joint_modes(&modes, &hold) {
                let uncertain = drv.joint_modes_uncertain();
                drop(drv);
                if uncertain {
                    self.safety.trigger_fault_coded(crate::safety::FaultCode::GravityCompFailed, FaultReason::Other(format!("GRAVITY_COMP_SWITCH_FAILED: {e}")));
                }
                return Err(Self::reject("GRAVITY_COMP_SWITCH_FAILED", e.to_string()));
            }
        }
        self.stop_motion();
        for i in 0..ARM {
            self.commanded_positions[i] = pose[i];
            self.commanded_velocities[i] = 0.0;
        }
        self.target_positions = self.commanded_positions.clone();
        self.following_error_since = None;
        let _ = self.safety.mark_motion_started();
        self.gravity_comp = Some(GravityCompRun {
            phase: GravityCompPhase::RampIn,
            kp: arm6(&policy.hold_kp),
            kd: arm6(&policy.hold_kd),
            policy,
            elapsed_s: 0.0,
            scale,
            damping_kd: damping,
            entry_pose: pose,
            entry_torque,
            tau_ff: entry_torque,
            position: pose,
            overspeed_count: 0,
            prev_measured: None,
            lease_generation: self.active_lease.as_ref().map(|l| l.generation),
        });
        tracing::info!("Gravity compensation on (scale {scale}, kd {damping:?})");
        Ok(())
    }

    /// Leave the mode (API exit, stop, or a safety event). The arm is caught at the
    /// measured pose at once; the switch back to POS_VEL follows after `exit_hold_ms`.
    pub fn exit_gravity_comp(&mut self, code: &str, message: impl Into<String>) {
        let Some(run) = self.gravity_comp.as_mut() else { return; };
        if !run.phase.floating() { return; }
        let message = message.into();
        let measured = arm6(&self.measured_positions);
        let hold = if measured.iter().all(|q| q.is_finite()) { measured } else { arm6(&self.commanded_positions) };
        run.phase = GravityCompPhase::Exiting { hold, elapsed_s: 0.0 };
        run.position = hold;
        run.kp = arm6(&run.policy.hold_kp);
        run.kd = arm6(&run.policy.hold_kd);
        // keep the feed-forward that was supporting the arm until POS_VEL takes over
        for i in 0..ARM {
            self.commanded_positions[i] = hold[i];
            self.commanded_velocities[i] = 0.0;
        }
        self.target_positions = self.commanded_positions.clone();
        let at = self.start_time.elapsed().as_secs_f64();
        if code == "REQUESTED" || code == "STOPPED" {
            tracing::info!("Gravity compensation exit: {code} {message}");
        } else {
            tracing::warn!("Gravity compensation exit: {code} {message}");
        }
        self.gravity_comp_last_exit = Some(GravityCompExit { code: code.into(), message, at });
        self.gravity_comp_write_now();
    }

    /// Send this tick's gravity-comp targets immediately (exit paths outside the tick).
    fn gravity_comp_write_now(&mut self) {
        if !self.is_torqued || self.disable_pending { return; }
        let Some(targets) = self.gravity_comp_targets() else { return; };
        if let Some(d) = &self.driver {
            if let Ok(mut drv) = d.lock() {
                if let Err(e) = drv.write_joint_targets(&targets) { tracing::error!("Gravity comp hold write failed: {e}"); }
            }
        }
    }

    /// Joint targets while the mode is on: MIT for J1-J6, position for the gripper.
    pub fn gravity_comp_targets(&self) -> Option<Vec<JointTarget>> {
        let run = self.gravity_comp.as_ref()?;
        let mut targets: Vec<JointTarget> = (0..ARM)
            .map(|i| JointTarget::mit(run.position[i], 0.0, run.kp[i], run.kd[i], run.tau_ff[i])).collect();
        targets.push(JointTarget {
            position: self.commanded_positions[ARM],
            velocity: Some(self.commanded_velocities[ARM]),
            torque: Some(0.0), kp: None, kd: None,
        });
        Some(targets)
    }

    fn gravity_comp_exit_cause(&mut self, dt: f64) -> Option<(&'static str, String)> {
        let run = self.gravity_comp.as_ref()?;
        let policy = run.policy.clone();
        if self.e_stop_latched.load(std::sync::atomic::Ordering::SeqCst) || self.safety.state() == &SafetyState::EStop {
            return Some(("ESTOP", "E-stop".into()));
        }
        match self.safety.state() {
            SafetyState::Fault(FaultReason::BusLoss) => return Some(("BUS_LOSS", "CAN feedback lost".into())),
            SafetyState::Fault(FaultReason::WatchdogTimeout) => return Some(("WATCHDOG", "Lease watchdog".into())),
            SafetyState::Fault(r) => return Some(("FAULT", match r { FaultReason::Other(m) => m.clone(), r => format!("{r:?}") })),
            _ => {}
        }
        if !self.is_torqued || self.disable_pending { return Some(("TORQUE_LOST", "Torque is off".into())); }
        if let Some(d) = &self.driver {
            let enabled = d.lock().is_ok_and(|drv| !drv.capabilities().enable_feedback || drv.enable_state() == EnableState::Enabled);
            if !enabled { return Some(("TORQUE_LOST", "Motors no longer report enabled".into())); }
        }
        if self.lease_enabled {
            if self.last_heartbeat.elapsed() > self.lease_timeout {
                return Some(("WATCHDOG", format!("No heartbeat for {} ms", self.lease_timeout.as_millis())));
            }
            let generation = self.active_lease.as_ref().map(|l| l.generation);
            if generation != run.lease_generation || !self.is_lease_active() || self.handover_settling_since.is_some() {
                return Some(("LEASE_LOST", "Control lease lost, released or taken over".into()));
            }
        }
        if !self.feedback_numeric_valid || !self.hardware_feedback_fresh(Duration::from_millis(policy.feedback_max_age_ms)) {
            return Some(("FEEDBACK_STALE", format!("Feedback older than {} ms", policy.feedback_max_age_ms)));
        }
        if let Some(msg) = self.gravity_comp_near_limit(&policy) { return Some(("GRAVITY_COMP_JOINT_LIMIT", msg)); }
        if run.phase == GravityCompPhase::RampIn {
            if let Some((i, d)) = (0..ARM).map(|i| (i, (self.measured_positions[i] - run.entry_pose[i]).abs()))
                .find(|(_, d)| !d.is_finite() || *d > policy.ramp_max_deviation_rad) {
                return Some(("GRAVITY_COMP_SAG", format!(
                    "Joint {} moved {d:.3} rad during the ramp-in (limit {} rad): the model does not hold the arm", i + 1, policy.ramp_max_deviation_rad)));
            }
        }
        // Speed: reported velocity, and the position difference over the tick (independent
        // of the velocity mapping range VMAX).
        let measured = arm6(&self.measured_positions);
        let prev = run.prev_measured;
        let mut fastest: Option<(usize, f32)> = None;
        for i in 0..ARM {
            let reported = self.measured_velocities.get(i).copied().unwrap_or(0.0).abs();
            let differenced = prev.map_or(0.0, |p| ((measured[i] - p[i]) / dt.max(1e-3) as f32).abs());
            let speed = reported.max(differenced);
            if speed > policy.speed_limit(i) && fastest.is_none_or(|(_, s)| speed > s) { fastest = Some((i, speed)); }
        }
        let run = self.gravity_comp.as_mut()?;
        run.prev_measured = Some(measured);
        match fastest {
            Some((i, speed)) => {
                run.overspeed_count += 1;
                if run.overspeed_count >= policy.overspeed_ticks {
                    return Some(("GRAVITY_COMP_OVERSPEED", format!(
                        "Joint {} at {speed:.2} rad/s for {} ticks (limit {} rad/s)", i + 1, run.overspeed_count, policy.speed_limit(i))));
                }
            }
            None => run.overspeed_count = 0,
        }
        None
    }

    /// One control tick of the mode. Runs after the feedback read and the fault checks,
    /// before the lease watchdog.
    pub fn gravity_comp_tick(&mut self, dt: f64) {
        let Some(phase) = self.gravity_comp.as_ref().map(|r| r.phase.clone()) else { return; };
        match phase {
            GravityCompPhase::RampIn | GravityCompPhase::Active => {
                if let Some((code, message)) = self.gravity_comp_exit_cause(dt) {
                    self.exit_gravity_comp(code, message);
                    return;
                }
                let Some(run) = self.gravity_comp.as_ref() else { return; };
                let (policy, scale) = (run.policy.clone(), run.scale);
                let Some(model) = self.gravity_comp_model_torque(&policy, scale) else {
                    self.exit_gravity_comp("GRAVITY_MODEL_UNAVAILABLE", "Gravity torque could not be computed");
                    return;
                };
                let measured = arm6(&self.measured_positions);
                let Some(run) = self.gravity_comp.as_mut() else { return; };
                run.elapsed_s += dt;
                let r = run.ramp();
                for i in 0..ARM {
                    let lim = policy.torque_limits_nm[i];
                    run.tau_ff[i] = ((1.0 - r) * run.entry_torque[i] + r * model[i]).clamp(-lim, lim);
                    run.kp[i] = (1.0 - r) * policy.hold_kp[i];
                    run.kd[i] = (1.0 - r) * policy.hold_kd[i] + r * run.damping_kd[i];
                    run.position[i] = if r < 1.0 { run.entry_pose[i] } else { measured[i] };
                }
                if r >= 1.0 && run.phase == GravityCompPhase::RampIn {
                    run.phase = GravityCompPhase::Active;
                    tracing::info!("Gravity compensation ramp complete");
                }
                // The hand moves the arm: the reference follows the measurement, so the
                // tracking-error check sees no error from intended motion.
                for i in 0..ARM {
                    self.commanded_positions[i] = measured[i];
                    self.commanded_velocities[i] = 0.0;
                }
                self.target_positions = self.commanded_positions.clone();
            }
            GravityCompPhase::Exiting { hold, elapsed_s } => {
                let exit_hold = self.gravity_comp.as_ref().map_or(0.1, |r| r.policy.exit_hold_ms as f64 / 1000.0);
                let elapsed = elapsed_s + dt;
                if elapsed + 1e-9 < exit_hold {
                    if let Some(run) = self.gravity_comp.as_mut() { run.phase = GravityCompPhase::Exiting { hold, elapsed_s: elapsed }; }
                } else {
                    self.gravity_comp_switch_back(hold);
                }
            }
            GravityCompPhase::ExitFailed { hold, retry_in_s } => {
                let left = retry_in_s - dt;
                if left > 0.0 {
                    if let Some(run) = self.gravity_comp.as_mut() { run.phase = GravityCompPhase::ExitFailed { hold, retry_in_s: left }; }
                } else {
                    self.gravity_comp_switch_back(hold);
                }
            }
        }
    }

    fn gravity_comp_switch_back(&mut self, hold: [f32; ARM]) {
        let gripper = self.commanded_positions[ARM];
        let result = match &self.driver {
            None => Ok(()),
            Some(d) => match d.lock() {
                Ok(mut drv) => {
                    let mut hold_targets: Vec<JointTarget> = hold.iter().map(|&p| JointTarget::position(p)).collect();
                    hold_targets.push(JointTarget::position(gripper));
                    drv.set_joint_modes(&[JointMode::Position; ARM + 1], &hold_targets).map_err(|e| e.to_string())
                }
                Err(_) => Err("driver lock poisoned".into()),
            },
        };
        match result {
            Ok(()) => {
                self.gravity_comp = None;
                for i in 0..ARM {
                    self.commanded_positions[i] = hold[i];
                    self.commanded_velocities[i] = 0.0;
                }
                self.target_positions = self.commanded_positions.clone();
                self.commanded_accelerations.fill(0.0);
                self.following_error_since = None;
                self.safety.mark_motion_finished();
                tracing::info!("Gravity compensation off: POS_VEL hold at {hold:?}");
            }
            Err(e) => {
                tracing::error!("Gravity compensation: switch back to POS_VEL not confirmed: {e}");
                if let Some(run) = self.gravity_comp.as_mut() { run.phase = GravityCompPhase::ExitFailed { hold, retry_in_s: 0.5 }; }
                if !self.safety.state().is_stopped() {
                    self.safety.trigger_fault_coded(crate::safety::FaultCode::GravityCompFailed, FaultReason::Other(format!("GRAVITY_COMP_EXIT_FAILED: {e}")));
                }
            }
        }
    }

    /// Motion commands are refused while the mode is on.
    pub fn gravity_comp_motion_busy(&self) -> Result<(), MotionError> {
        if self.gravity_comp.is_some() {
            return Err(MotionError::MotionBusy("gravity compensation is active; exit it first".into()));
        }
        Ok(())
    }

    pub fn gravity_comp_servo_busy(&self) -> Result<(), SafetyError> {
        if self.gravity_comp.is_some() { return Err(SafetyError::MotionBusy); }
        Ok(())
    }

    pub fn gravity_comp_observation(&self) -> GravityCompObservation {
        let support = self.gravity_comp_support();
        let mut obs = GravityCompObservation {
            supported: support.is_ok(),
            blocker: self.gravity_comp_blocker(),
            last_exit: self.gravity_comp_last_exit.clone(),
            scale: self.gravity_comp_policy().map_or(0.0, |p| p.scale),
            damping_kd: self.gravity_comp_policy().map_or_else(Vec::new, |p| p.damping_kd.clone()),
            ..Default::default()
        };
        if let Some(run) = &self.gravity_comp {
            obs.phase = Some(run.phase.name().into());
            obs.ramp = run.ramp();
            obs.scale = run.scale;
            obs.damping_kd = run.damping_kd.to_vec();
            obs.tau_ff = run.tau_ff.to_vec();
        }
        obs
    }

    /// Used by clear_fault: a fault cannot be cleared before POS_VEL is confirmed again.
    pub fn gravity_comp_blocks_clear(&self) -> bool {
        self.gravity_comp.as_ref().is_some_and(|r| matches!(r.phase, GravityCompPhase::ExitFailed { .. }))
    }

    #[doc(hidden)]
    pub fn gravity_comp_run(&self) -> Option<&GravityCompRun> { self.gravity_comp.as_ref() }
}
