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

fn reason(code: &'static str, message: impl Into<std::borrow::Cow<'static, str>>) -> ModeReason {
    ModeReason { code: code.into(), message: message.into() }
}

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
        if !self.config.dynamics_qualified {
            return Err(reason("UNSUPPORTED", "Gravity compensation is disabled: active tool dynamics are unqualified"));
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
        // the check by reference: copied (its message) only when it blocks
        let mut blocked = None;
        if let Some(Ok(drv)) = self.driver.as_ref().map(|d| d.lock()) {
            drv.with_range_check(&mut |c| match c.map(|c| (c.status, c)) {
                Some((RangeCheckStatus::Mismatch, c)) => blocked = Some(reason("DRIVER_RANGE_MISMATCH", c.message.clone())),
                Some((RangeCheckStatus::Unverified, c)) => blocked = Some(reason("DRIVER_RANGE_UNVERIFIED", c.message.clone())),
                _ => {}
            });
        }
        if blocked.is_some() { return blocked; }
        let policy = self.gravity_comp_policy()?;
        if self.shutdown_committed || self.e_stop_latched.load(std::sync::atomic::Ordering::SeqCst) || self.safety.state() == &SafetyState::EStop {
            return Some(reason("ESTOP_ACTIVE", "E-stop is latched"));
        }
        if let SafetyState::Fault(r) = self.safety.state() { return Some(reason("FAULT_ACTIVE", format!("Fault active: {r:?}"))); }
        if self.handover_settling_since.is_some() { return Some(reason("HANDOVER_SETTLING", "Arm is settling after a control handover")); }
        if !self.is_torqued || self.pending_rearm || self.disable_pending || self.output_suspended
            || !self.safety.state().is_operational() {
            return Some(reason("NOT_ARMED", "Enable the arm first"));
        }
        #[cfg(feature = "policy-runtime")]
        if self.policy_loading {
            return Some(reason("MOTION_BUSY", "a policy is loading"));
        }
        #[cfg(feature = "policy-runtime")]
        if self.policy_run.is_some() {
            return Some(reason("MOTION_BUSY", "a policy run is active"));
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
        if let Some(r) = self.gravity_comp_blocker() { return Err(Self::reject(&r.code, r.message.into_owned())); }
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
        // hand guiding and MIT tracking own the output exclusively: once the request is valid,
        // hand back first (confirmed, at rest); a refused request never changes the owner
        if self.mit_active() {
            self.mit_hand_back("gravity compensation").map_err(|e| Self::reject("MIT_HAND_BACK_FAILED", e))?;
        }
        let pose = arm6(&self.measured_positions);
        self.servo_ws_min_depths = None;
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
                // Mode switch (and any rollback dual-hold) already wrote hold frames to the bus;
                // never keep a stale pre-entry `last_issued_positions` as tracking baseline.
                self.tracking.invalidate();
                if uncertain {
                    self.safety.trigger_fault_coded(crate::safety::FaultCode::GravityCompFailed, FaultReason::Other(format!("GRAVITY_COMP_SWITCH_FAILED: {e}")));
                }
                return Err(Self::reject("GRAVITY_COMP_SWITCH_FAILED", e.to_string()));
            }
        }
        self.stop_motion();
        self.servo_stream_active = false;
        self.tracking.invalidate();
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
        // Hand guiding is the way out of a collision hold: the guard does not watch while
        // floating, so the hold ends here.
        if let Some(h) = self.collision.hold.take() {
            tracing::info!("Collision hold on joint {} ended: gravity compensation entered", h.joint + 1);
        }
        tracing::info!("Gravity compensation on (scale {scale}, kd {damping:?})");
        Ok(())
    }

    /// Leave the mode (API exit, stop, or a safety event). The arm is caught at the
    /// measured pose at once; the switch back to POS_VEL follows after `exit_hold_ms`.
    pub fn exit_gravity_comp(&mut self, code: &str, message: impl Into<String>) {
        self.exit_gravity_comp_inner(code, message, true);
    }

    pub(crate) fn exit_gravity_comp_inner(&mut self, code: &str, message: impl Into<String>, write_now: bool) {
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
        self.tracking.invalidate();
        crate::rt_core::scratch::copy_into(&mut self.target_positions, &self.commanded_positions);
        let at = self.start_time.elapsed().as_secs_f64();
        if code == "REQUESTED" || code == "STOPPED" {
            tracing::info!("Gravity compensation exit: {code} {message}");
        } else {
            tracing::warn!("Gravity compensation exit: {code} {message}");
        }
        self.gravity_comp_last_exit = Some(GravityCompExit { code: code.into(), message, at });
        if write_now {
            self.gravity_comp_write_now();
        }
    }

    /// Send this tick's gravity-comp targets immediately (exit paths outside the tick),
    /// routing the bounded result through the owner's output classifier.
    fn gravity_comp_write_now(&mut self) {
        let driver = self.driver.take();
        let out = self.write_gravity_comp_immediate_with_driver(driver.as_ref(), false, false);
        self.driver = driver;
        let _ = self.classify_tick_output(out);
    }

    /// Shared immediate gravity-comp write using `scratch.targets` (0 alloc).
    /// Callers pass `use_estop_hold` explicitly when the physical E-stop latch is already set
    /// before `SafetyState::EStop` is installed, and `nonblocking = true` to preserve `try_lock`.
    pub(crate) fn write_gravity_comp_immediate_with_driver(
        &mut self,
        driver: Option<&std::sync::Mutex<Box<dyn crate::driver::ArmDriver>>>,
        use_estop_hold: bool,
        nonblocking: bool,
    ) -> crate::supervisor::TickOutput {
        use crate::driver::TxBatchOutcome;
        use crate::supervisor::TickOutput;
        if !self.is_torqued || self.disable_pending { return TickOutput::None; }
        let mut targets = std::mem::take(&mut self.scratch.targets);
        let filled = self.gravity_comp_targets_into(&mut targets);
        let out = if !filled {
            TickOutput::None
        } else if let Some(d) = driver {
            let guard = if nonblocking { d.try_lock().ok() } else { d.lock().ok() };
            if let Some(mut drv) = guard {
                if drv.enable_state() == EnableState::Disabled {
                    TickOutput::None
                } else {
                    let is_hold = use_estop_hold
                        || self.safety.state().is_stopped()
                        || self.e_stop_latched.load(std::sync::atomic::Ordering::SeqCst);
                    let write_res = if is_hold {
                        drv.write_estop_hold_bounded(&targets)
                    } else {
                        drv.write_joint_targets_bounded(&targets)
                    };
                    let now = drv.clock_now();
                    let max_age = drv.tx_frame_max_age();
                    let hold_sent_any = is_hold && drv.last_hold_tx_sent_any();
                    match write_res {
                        Ok(TxBatchOutcome::Complete) => TickOutput::Complete { is_hold },
                        Ok(TxBatchOutcome::Deferred { pending }) => TickOutput::Deferred {
                            pending,
                            now,
                            max_age,
                            oldest_age: drv.oldest_pending_tx_age(),
                            is_hold,
                            sent_any: hold_sent_any,
                        },
                        Ok(TxBatchOutcome::Expired { dropped, pending }) => TickOutput::Expired {
                            dropped,
                            pending,
                            now,
                            max_age,
                            oldest_age: drv.last_tx_expired_age(),
                            is_hold,
                            sent_any: hold_sent_any,
                        },
                        Err(e) => TickOutput::Error {
                            message: e.to_string(),
                            now,
                            is_hold,
                        },
                    }
                }
            } else {
                TickOutput::None
            }
        } else {
            TickOutput::None
        };
        self.scratch.targets = targets;
        out
    }

    /// Populate `out` with gravity-comp targets (MIT for J1-J6, position for gripper) without allocating.
    pub(crate) fn gravity_comp_targets_into(&self, out: &mut Vec<JointTarget>) -> bool {
        let Some(run) = self.gravity_comp.as_ref() else { return false; };
        out.clear();
        for i in 0..ARM {
            out.push(JointTarget::mit(run.position[i], 0.0, run.kp[i], run.kd[i], run.tau_ff[i]));
        }
        out.push(JointTarget {
            position: self.commanded_positions[ARM],
            velocity: Some(self.commanded_velocities[ARM]),
            torque: Some(0.0), kp: None, kd: None,
        });
        true
    }

    /// Joint targets while the mode is on: MIT for J1-J6, position for the gripper.
    pub fn gravity_comp_targets(&self) -> Option<Vec<JointTarget>> {
        let mut targets = Vec::with_capacity(ARM + 1);
        self.gravity_comp_targets_into(&mut targets).then_some(targets)
    }

    fn gravity_comp_exit_cause(&mut self, dt: f64) -> Option<(&'static str, String)> {
        let mut run = self.gravity_comp.take()?;
        let cause = self.gravity_comp_exit_cause_inner(&mut run, dt);
        self.gravity_comp = Some(run);
        cause
    }

    fn gravity_comp_exit_cause_inner(&self, run: &mut GravityCompRun, dt: f64) -> Option<(&'static str, String)> {
        let policy = &run.policy;
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
        if let Some(msg) = self.gravity_comp_near_limit(policy) { return Some(("GRAVITY_COMP_JOINT_LIMIT", msg)); }
        if run.phase == GravityCompPhase::RampIn {
            if let Some((i, d)) = (0..ARM).map(|i| (i, (self.measured_positions[i] - run.entry_pose[i]).abs()))
                .find(|(_, d)| !d.is_finite() || *d > policy.ramp_max_deviation_rad) {
                return Some(("GRAVITY_COMP_SAG", format!(
                    "Joint {} moved {d:.3} rad during the ramp-in (limit {} rad): the model does not hold the arm", i + 1, policy.ramp_max_deviation_rad)));
            }
        }
        // Speed: reported velocity, and the position difference over the tick (independent
        // of the velocity mapping range VMAX).
        // The limit depends on the direction: moving with gravity (falling) uses the
        // overspeed limit, moving against it (lifted by hand) the lift limit.
        let measured = arm6(&self.measured_positions);
        let prev = run.prev_measured;
        let gravity = self.config.arm_model.gravity_torques(&measured).ok()
            .filter(|t| t.len() >= ARM && t.iter().all(|x| x.is_finite()));
        let mut fastest: Option<(usize, f32, f32)> = None;
        for i in 0..ARM {
            let reported = self.measured_velocities.get(i).copied().unwrap_or(0.0);
            let differenced = prev.map_or(0.0, |p| (measured[i] - p[i]) / dt.max(1e-3) as f32);
            let v = if differenced.abs() > reported.abs() { differenced } else { reported };
            let speed = v.abs();
            let limit = policy.directional_speed_limit(i, v, gravity.as_ref().map(|t| t[i]));
            if speed > limit && fastest.is_none_or(|(_, s, _)| speed > s) { fastest = Some((i, speed, limit)); }
        }
        run.prev_measured = Some(measured);
        match fastest {
            Some((i, speed, limit)) => {
                run.overspeed_count += 1;
                if run.overspeed_count >= policy.overspeed_ticks {
                    return Some(("GRAVITY_COMP_OVERSPEED", format!(
                        "Joint {} at {speed:.2} rad/s for {} ticks (limit {limit} rad/s)", i + 1, run.overspeed_count)));
                }
            }
            None => run.overspeed_count = 0,
        }
        None
    }

    /// Pre-output control tick of gravity compensation (`RampIn` / `Active` only).
    /// Runs after the feedback read and fault checks, before the lease watchdog.
    /// `Exiting` / `ExitFailed` advance in [`Self::step_gravity_comp_exit`] AFTER this release's
    /// stiff hold output completes cleanly.
    pub fn gravity_comp_tick(&mut self, dt: f64) {
        let Some(phase) = self.gravity_comp.as_ref().map(|r| r.phase.clone()) else { return; };
        match phase {
            GravityCompPhase::RampIn | GravityCompPhase::Active => {
                if let Some((code, message)) = self.gravity_comp_exit_cause(dt) {
                    self.exit_gravity_comp_inner(code, message, false);
                    return;
                }
                let Some(mut run) = self.gravity_comp.take() else { return; };
                let model = self.gravity_comp_model_torque(&run.policy, run.scale);
                let Some(model) = model else {
                    self.gravity_comp = Some(run);
                    self.exit_gravity_comp_inner("GRAVITY_MODEL_UNAVAILABLE", "Gravity torque could not be computed", false);
                    return;
                };
                let measured = arm6(&self.measured_positions);
                run.elapsed_s += dt;
                let r = run.ramp();
                for i in 0..ARM {
                    let lim = run.policy.torque_limits_nm[i];
                    run.tau_ff[i] = ((1.0 - r) * run.entry_torque[i] + r * model[i]).clamp(-lim, lim);
                    run.kp[i] = (1.0 - r) * run.policy.hold_kp[i];
                    run.kd[i] = (1.0 - r) * run.policy.hold_kd[i] + r * run.damping_kd[i];
                    run.position[i] = if r < 1.0 { run.entry_pose[i] } else { measured[i] };
                }
                if r >= 1.0 && run.phase == GravityCompPhase::RampIn {
                    run.phase = GravityCompPhase::Active;
                    tracing::info!("Gravity compensation ramp complete");
                }
                self.gravity_comp = Some(run);
                // The hand moves the arm: the reference follows the measurement, so the
                // tracking-error check sees no error from intended motion.
                for i in 0..ARM {
                    self.commanded_positions[i] = measured[i];
                    self.commanded_velocities[i] = 0.0;
                }
                crate::rt_core::scratch::copy_into(&mut self.target_positions, &self.commanded_positions);
            }
            GravityCompPhase::Exiting { hold, elapsed_s } => {
                // Once `exit_hold` of confirmed stiff holds has ALREADY completed on prior ticks
                // and the arm is now stationary and fresh, complete the POS_VEL handback before
                // this tick's output so tracking can validate and issue its baseline on the same tick.
                let (exit_hold, max_age_ms, max_vel) = self.gravity_comp.as_ref().map_or((0.1, 100, 0.05), |r| {
                    (r.policy.exit_hold_ms as f64 / 1000.0, r.policy.feedback_max_age_ms, r.policy.entry_max_velocity)
                });
                if elapsed_s + 1e-9 >= exit_hold {
                    let tracking_active = self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled);
                    let waiting_to_settle = tracking_active
                        && self.feedback_numeric_valid
                        && self.hardware_feedback_fresh(Duration::from_millis(max_age_ms))
                        && !self.gravity_comp_is_stationary(max_vel)
                        && elapsed_s < exit_hold + 1.0;
                    if !waiting_to_settle {
                        self.gravity_comp_switch_back(hold);
                    }
                }
            }
            GravityCompPhase::ExitFailed { .. } => {}
        }
    }

    /// Post-output advancement of gravity-comp exit handback (`Exiting` / `ExitFailed`).
    /// `Exiting` advances ONLY when this release's stiff hold write completed (`Complete`),
    /// preventing premature POS_VEL handback while stiff holds are skipped, deferred, or stalled.
    pub(crate) fn step_gravity_comp_exit(&mut self, dt: f64, stiff_hold_completed: bool) {
        let Some(phase) = self.gravity_comp.as_ref().map(|r| r.phase.clone()) else { return; };
        match phase {
            GravityCompPhase::RampIn | GravityCompPhase::Active => {}
            GravityCompPhase::Exiting { hold, elapsed_s } => {
                if !stiff_hold_completed && self.is_torqued && !self.disable_pending && !self.shutdown_committed {
                    return;
                }
                let (exit_hold, max_age_ms, max_vel) = self.gravity_comp.as_ref().map_or((0.1, 100, 0.05), |r| {
                    (r.policy.exit_hold_ms as f64 / 1000.0, r.policy.feedback_max_age_ms, r.policy.entry_max_velocity)
                });
                let elapsed = elapsed_s + dt;
                if elapsed + 1e-9 < exit_hold {
                    if let Some(run) = self.gravity_comp.as_mut() { run.phase = GravityCompPhase::Exiting { hold, elapsed_s: elapsed }; }
                } else {
                    let tracking_active = self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled);
                    if tracking_active {
                        let fresh = self.feedback_numeric_valid
                            && self.hardware_feedback_fresh(Duration::from_millis(max_age_ms));
                        let stationary = self.gravity_comp_is_stationary(max_vel);
                        // When tracking will take over POS_VEL output, wait for the arm to settle
                        // under the stiff MIT hold before transferring ownership (bounded by exit_hold + 1.0 s).
                        if fresh && !stationary && elapsed < exit_hold + 1.0 {
                            if let Some(run) = self.gravity_comp.as_mut() { run.phase = GravityCompPhase::Exiting { hold, elapsed_s: elapsed }; }
                            return;
                        }
                    }
                    self.gravity_comp_switch_back(hold);
                }
            }
            GravityCompPhase::ExitFailed { hold, retry_in_s } => {
                if !stiff_hold_completed && self.is_torqued && !self.disable_pending {
                    return;
                }
                let left = retry_in_s - dt;
                if left > 0.0 {
                    if let Some(run) = self.gravity_comp.as_mut() { run.phase = GravityCompPhase::ExitFailed { hold, retry_in_s: left }; }
                } else {
                    self.gravity_comp_switch_back(hold);
                }
            }
        }
    }

    fn gravity_comp_is_stationary(&self, max_vel: f32) -> bool {
        let has_vel = self
            .driver
            .as_ref()
            .is_none_or(|d| d.lock().is_ok_and(|drv| drv.capabilities().velocity_feedback));
        if !has_vel {
            return true;
        }
        self.measured_velocities.len() >= ARM
            && self.measured_velocities[..ARM]
                .iter()
                .all(|v| v.is_finite() && v.abs() <= max_vel)
    }

    fn gravity_comp_switch_back(&mut self, hold: [f32; ARM]) {
        let tracking_active = self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled);
        let (max_age_ms, max_vel) = self
            .gravity_comp
            .as_ref()
            .map_or((100, 0.05), |r| (r.policy.feedback_max_age_ms, r.policy.entry_max_velocity));
        let fresh = !tracking_active
            || (self.feedback_numeric_valid
                && self.hardware_feedback_fresh(Duration::from_millis(max_age_ms))
                && self.measured_positions.len() >= ARM
                && self.measured_positions[..ARM].iter().all(|q| q.is_finite()));
        let stationary = !tracking_active || self.gravity_comp_is_stationary(max_vel);
        let gripper = self.commanded_positions[ARM];
        let result = if !fresh {
            Err(format!("fresh feedback (<= {max_age_ms} ms) required before switching back to POS_VEL"))
        } else if !stationary {
            Err(format!("arm not stationary (limit {max_vel} rad/s) before switching back to POS_VEL"))
        } else {
            match &self.driver {
                None => Ok(()),
                Some(d) => match d.lock() {
                    Ok(mut drv) => {
                        let mut hold_targets: Vec<JointTarget> = hold.iter().map(|&p| JointTarget::position(p)).collect();
                        hold_targets.push(JointTarget::position(gripper));
                        drv.set_joint_modes(&[JointMode::Position; ARM + 1], &hold_targets).map_err(|e| e.to_string())
                    }
                    Err(_) => Err("driver lock poisoned".into()),
                },
            }
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
                if self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled) {
                    self.tracking.seed(&self.commanded_positions);
                } else {
                    self.tracking.invalidate();
                }
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
        let mut obs = GravityCompObservation::default();
        self.gravity_comp_observation_into(&mut obs);
        obs
    }

    /// [`Self::gravity_comp_observation`] into `obs`, reusing its memory.
    pub fn gravity_comp_observation_into(&self, obs: &mut GravityCompObservation) {
        obs.supported = self.gravity_comp_support().is_ok();
        let blocker = self.gravity_comp_blocker();
        if obs.blocker != blocker { obs.blocker = blocker; }
        crate::rt_core::scratch::set_opt_eq(&mut obs.last_exit, self.gravity_comp_last_exit.as_ref());
        obs.scale = self.gravity_comp_policy().map_or(0.0, |p| p.scale);
        match self.gravity_comp_policy() {
            Some(p) => obs.damping_kd.clone_from(&p.damping_kd),
            None => obs.damping_kd.clear(),
        }
        obs.ramp = 0.0;
        obs.tau_ff.clear();
        match &self.gravity_comp {
            Some(run) => {
                obs.phase = Some(crate::model::HolderLabel::new(run.phase.name()));
                obs.ramp = run.ramp();
                obs.scale = run.scale;
                obs.damping_kd.clear();
                obs.damping_kd.extend_from_slice(&run.damping_kd);
                obs.tau_ff.extend_from_slice(&run.tau_ff);
            }
            None => obs.phase = None,
        }
    }

    /// Used by clear_fault: a fault cannot be cleared before POS_VEL is confirmed again.
    pub fn gravity_comp_blocks_clear(&self) -> bool {
        self.gravity_comp.as_ref().is_some_and(|r| matches!(r.phase, GravityCompPhase::ExitFailed { .. }))
    }

    #[doc(hidden)]
    pub fn gravity_comp_run(&self) -> Option<&GravityCompRun> { self.gravity_comp.as_ref() }
}
