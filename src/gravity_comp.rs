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

use crate::driver::{JointMode, JointTarget, RangeCheckStatus};
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

fn reason(code: &str, message: &str) -> ModeReason {
    ModeReason {
        code: crate::safety::BoundedText::new(code),
        message: crate::safety::BoundedText::new(message),
    }
}

fn reason_fmt(code: &str, f: impl FnOnce(&mut crate::safety::BoundedText<256>)) -> ModeReason {
    let mut message = crate::safety::BoundedText::empty();
    f(&mut message);
    ModeReason {
        code: crate::safety::BoundedText::new(code),
        message,
    }
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
    #[allow(clippy::result_large_err)] // inline ModeReason keeps fill_observation zero-alloc on RT
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
        if self.driver.is_some() {
            let Some(drv) = self.read_driver_for_tick() else {
                return Err(reason("UNSUPPORTED", "driver lock unavailable"));
            };
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

    fn gravity_comp_near_limit(&self, policy: &GravityCompPolicy) -> Option<crate::safety::BoundedText<256>> {
        let m = policy.joint_limit_margin_rad;
        (0..ARM).find_map(|i| {
            let q = *self.measured_positions.get(i)?;
            let l = self.config.limits.get(i)?;
            (!q.is_finite() || q < l.min_position + m || q > l.max_position - m).then(|| {
                let mut msg = crate::safety::BoundedText::empty();
                let _ = core::fmt::write(
                    &mut msg,
                    format_args!(
                        "joint {} at {q:.3} rad is within {m} rad of its limit [{}, {}]",
                        i + 1,
                        l.min_position,
                        l.max_position
                    ),
                );
                msg
            })
        })
    }

    /// Why entering is refused now; None when it is allowed (or the mode is already on).
    pub fn gravity_comp_blocker(&self) -> Option<ModeReason> {
        if let Err(r) = self.gravity_comp_support() { return Some(r); }
        if self.gravity_comp.is_some() { return None; }
        // MIT torque commands are only as right as the mapping ranges: require the
        // startup range check to have passed when the driver ran one.
        let mut blocked = None;
        if let Some(drv) = self.read_driver_for_tick() {
            drv.with_range_check(&mut |c| match c.map(|c| (c.status, c)) {
                Some((RangeCheckStatus::Mismatch, c)) => blocked = Some(reason("DRIVER_RANGE_MISMATCH", &c.message)),
                Some((RangeCheckStatus::Unverified, c)) => blocked = Some(reason("DRIVER_RANGE_UNVERIFIED", &c.message)),
                _ => {}
            });
        }
        if blocked.is_some() { return blocked; }
        let policy = self.gravity_comp_policy()?;
        if self.shutdown_committed || self.e_stop_latched.load(std::sync::atomic::Ordering::SeqCst) || self.safety.state() == &SafetyState::EStop {
            return Some(reason("ESTOP_ACTIVE", "E-stop is latched"));
        }
        if let SafetyState::Fault(r) = self.safety.state() {
            let br = crate::safety::BoundedFaultReason::from_reason(r);
            return Some(reason_fmt("FAULT_ACTIVE", |msg| {
                let _ = core::fmt::write(msg, format_args!("Fault active: {br:?}"));
            }));
        }
        if self.core.motion_inhibited_by_config() {
            return Some(reason("CONFIG_BUSY", "A configuration change is activating or reconciling"));
        }
        if self.handover_settling_since.is_some() { return Some(reason("HANDOVER_SETTLING", "Arm is settling after a control handover")); }
        if !self.is_torqued || self.pending_rearm || self.disable_pending || self.output_suspended
            || !self.safety.state().is_operational() {
            return Some(reason("NOT_ARMED", "Enable the arm first"));
        }
        if self.power.is_static_restricted() {
            return Some(reason("STATIC_LOAD_LIMIT", crate::power::STATIC_LOAD_LIMIT_MSG));
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
        #[cfg(not(target_arch = "wasm32"))]
        if self.exclusive_session_owns_output() {
            return Some(reason("SESSION_BUSY", "An exclusive output session is active"));
        }
        if self.lease_enabled && !self.is_lease_active() { return Some(reason("LEASE_REQUIRED", "The control lease is required")); }
        if !self.hardware_feedback_fresh(Duration::from_millis(policy.feedback_max_age_ms)) || !self.feedback_numeric_valid {
            return Some(reason_fmt("FEEDBACK_STALE", |msg| {
                let _ = core::fmt::write(msg, format_args!("Feedback older than {} ms", policy.feedback_max_age_ms));
            }));
        }
        if let Some((i, v)) = self.measured_velocities.iter().take(ARM).enumerate()
            .find(|(_, v)| !v.is_finite() || v.abs() > policy.entry_max_velocity) {
            return Some(reason_fmt("GRAVITY_COMP_NOT_STATIONARY", |msg| {
                let _ = core::fmt::write(
                    msg,
                    format_args!("Joint {} moves at {v:.3} rad/s (limit {})", i + 1, policy.entry_max_velocity),
                );
            }));
        }
        if let Some(msg) = self.gravity_comp_near_limit(policy) {
            return Some(ModeReason {
                code: crate::safety::BoundedText::new("GRAVITY_COMP_JOINT_LIMIT"),
                message: msg,
            });
        }
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
    pub fn enter_gravity_comp(admitted: crate::supervisor::Admitted<'_>, request: GravityCompRequest) -> Result<(), ArmError> {
        let (this, _proof) = admitted.consume()?;
        this.enter_gravity_comp_inner(request)
    }

    /// Zero-allocation RT entry into gravity compensation on a driverless (mock) supervisor.
    /// Hardware drivers with synchronous MIT mode switching are refused fail-closed before calling this.
    #[allow(clippy::result_large_err)]
    pub(crate) fn enter_gravity_comp_rt(
        admitted: crate::supervisor::Admitted<'_>,
        policy: Option<GravityCompPolicy>,
        scale_override: Option<f32>,
        damping_override: Option<[f32; ARM]>,
    ) -> Result<Option<GravityCompPolicy>, (ModeReason, Option<GravityCompPolicy>)> {
        let (this, _proof) = match admitted.consume() {
            Ok(pair) => pair,
            Err(_) => {
                return Err((
                    reason("STOPPED", "Admission proof invalidated before entering gravity compensation"),
                    policy,
                ));
            }
        };
        if this.gravity_comp.is_some() {
            return Ok(policy);
        }
        if let Some(r) = this.gravity_comp_blocker() {
            return Err((r, policy));
        }
        let Some(policy) = policy else {
            return Err((reason("UNSUPPORTED", "no gravity_comp policy"), None));
        };
        let [lo, hi] = policy.scale_range;
        let scale = scale_override.unwrap_or(policy.scale);
        if !scale.is_finite() || !(lo..=hi).contains(&scale) {
            return Err((reason("INVALID_COMMAND", "scale is outside policy.scale_range"), Some(policy)));
        }
        let damping = match damping_override {
            None => arm6(&policy.damping_kd),
            Some(v) if v.iter().all(|k| k.is_finite() && *k >= 0.0 && *k <= policy.damping_kd_max) => v,
            Some(_) => {
                return Err((
                    reason("INVALID_COMMAND", "damping_kd values must be finite and within [0, damping_kd_max]"),
                    Some(policy),
                ));
            }
        };
        let Some(model) = this.gravity_comp_model_torque(&policy, scale) else {
            return Err((
                reason("GRAVITY_MODEL_UNAVAILABLE", "Gravity torque could not be computed"),
                Some(policy),
            ));
        };
        let pose = arm6(&this.measured_positions);
        this.retire_servo_ws_min_depths();
        let mut entry_torque = model;
        for (i, t) in entry_torque.iter_mut().enumerate() {
            let lim = policy.torque_limits_nm[i];
            *t = if t.is_finite() { t.clamp(-lim, lim) } else { model[i] };
        }
        this.stop_motion();
        this.servo_stream_active = false;
        this.tracking.invalidate();
        for i in 0..ARM {
            this.commanded_positions[i] = pose[i];
            this.commanded_velocities[i] = 0.0;
        }
        crate::rt_core::scratch::copy_into(&mut this.target_positions, &this.commanded_positions);
        this.following_error_since = None;
        let _ = this.safety.mark_motion_started();
        this.gravity_comp = Some(GravityCompRun {
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
            lease_generation: this.active_lease.as_ref().map(|l| l.generation),
        });
        this.bump_motion_generation();
        let _ = this.collision.hold.take();
        Ok(None)
    }

    /// Zero-allocation RT exit from gravity compensation (does not trace or write synchronously on RT).
    pub(crate) fn exit_gravity_comp_rt(&mut self, code: &'static str, message: &str) {
        let Some(run) = self.gravity_comp.as_mut() else { return; };
        if !run.phase.floating() { return; }
        let measured = arm6(&self.measured_positions);
        let hold = if measured.iter().all(|q| q.is_finite()) { measured } else { arm6(&self.commanded_positions) };
        run.phase = GravityCompPhase::Exiting { hold, elapsed_s: 0.0 };
        self.gravity_comp_handback_ready = false;
        run.position = hold;
        run.kp = arm6(&run.policy.hold_kp);
        run.kd = arm6(&run.policy.hold_kd);
        for i in 0..ARM {
            self.commanded_positions[i] = hold[i];
            self.commanded_velocities[i] = 0.0;
        }
        self.tracking.invalidate();
        crate::rt_core::scratch::copy_into(&mut self.target_positions, &self.commanded_positions);
        let at = self.start_time.elapsed().as_secs_f64();
        self.gravity_comp_last_exit = Some(GravityCompExit {
            code: crate::model::HolderLabel::new(code),
            message: crate::model::HolderLabel::new(message),
            at,
        });
        self.bump_motion_generation();
    }

    fn enter_gravity_comp_inner(&mut self, request: GravityCompRequest) -> Result<(), ArmError> {
        if self.gravity_comp.is_some() { return Ok(()); }
        if let Some(r) = self.gravity_comp_blocker() { return Err(Self::reject(r.code.as_str(), r.message.to_string())); }
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
        if let Some(d) = self.driver.as_mut() {
            let mut modes = vec![JointMode::Mit; ARM];
            modes.push(JointMode::Position);
            let mut hold: Vec<JointTarget> = (0..ARM)
                .map(|i| JointTarget::mit(pose[i], 0.0, policy.hold_kp[i], policy.hold_kd[i], entry_torque[i])).collect();
            hold.push(JointTarget::position(gripper));
            let switch_res = d
                .with_mut(|drv| match drv.set_joint_modes(&modes, &hold) {
                    Ok(()) => Ok(()),
                    Err(e) => Err((e, drv.joint_modes_uncertain())),
                })
                .ok_or_else(|| ArmError::Internal("driver lock poisoned".into()))?;
            if let Err((e, uncertain)) = switch_res {
                // Mode switch (and any rollback dual-hold) already wrote hold frames to the bus;
                // never keep a stale pre-entry `last_issued_positions` as tracking baseline.
                self.tracking.invalidate();
                if uncertain {
                    self.safety.latch(
                        crate::safety::FaultKind::Other,
                        crate::safety::FaultDetail::formatted(crate::safety::FaultCode::GravityCompFailed, |buf| {
                            let _ = core::fmt::write(buf, format_args!("GRAVITY_COMP_SWITCH_FAILED: {e}"));
                        }),
                    );
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
        self.bump_motion_generation();
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
        self.gravity_comp_handback_ready = false;
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
        self.gravity_comp_last_exit = Some(GravityCompExit { code: code.into(), message: message.into(), at });
        self.bump_motion_generation();
        if write_now {
            self.gravity_comp_write_now();
        }
    }

    /// Send this tick's gravity-comp targets immediately (exit paths outside the tick),
    /// routing the bounded result through the owner's output classifier without counting
    /// off-tick calls against periodic release streaks.
    fn gravity_comp_write_now(&mut self) {
        let out = self.write_gravity_comp_immediate_inner(None, false, false);
        let _ = self.classify_tick_output(out, false, false);
    }

    /// Shared immediate gravity-comp write using stack-allocated targets (0 alloc).
    /// Callers pass `use_estop_hold` explicitly when the physical E-stop latch is already set
    /// before `SafetyState::EStop` is installed, and `nonblocking = true` to preserve `try_lock`.
    /// When `ext_driver` is `None`, operates directly on `self.driver` in place (zero mutex in
    /// `DriverSlot::Exclusive`, guarded lock in `DriverSlot::Legacy`); when `Some`, uses the
    /// externally shared mutex (for legacy two-thread contention tests).
    pub(crate) fn write_gravity_comp_immediate_inner(
        &mut self,
        ext_driver: Option<&std::sync::Mutex<Box<dyn crate::driver::ArmDriver>>>,
        use_estop_hold: bool,
        nonblocking: bool,
    ) -> crate::supervisor::TickOutput {
        use crate::supervisor::TickOutput;
        if !self.is_torqued
            || self.disable_pending
            || self.torque_intent != crate::supervisor::TorqueIntent::On
        {
            return TickOutput::None;
        }
        let Some(targets) = self.gravity_comp_targets_array() else { return TickOutput::None; };
        let is_hold = use_estop_hold
            || self.safety.state().is_stopped()
            || self.e_stop_latched.load(std::sync::atomic::Ordering::SeqCst);
        let is_exiting = self.gravity_comp.as_ref().is_some_and(|r| {
            matches!(
                r.phase,
                crate::gravity_comp::GravityCompPhase::Exiting { .. }
                    | crate::gravity_comp::GravityCompPhase::ExitFailed { .. }
            )
        });
        let output_lease_gen = if is_hold || is_exiting { 0 } else { self.current_active_lease_gen() };
        let maybe_out = if let Some(raw) = ext_driver {
            let fence = self.live_disconnect_fence_view();
            let guard = if nonblocking { raw.try_lock().ok() } else { raw.lock().ok() };
            guard.and_then(|mut g| {
                crate::supervisor::SupervisorInner::write_gravity_comp_targets_checked(
                    &mut **g,
                    &targets,
                    is_hold,
                    output_lease_gen,
                    &fence,
                )
            })
        } else {
            let e_stop_latched = &self.e_stop_latched;
            let pending_disconnect_hold = &self.pending_disconnect_hold;
            let atomic_fenced_lease_gen = &self.atomic_fenced_lease_gen;
            let fenced_lease_generation = self.fenced_lease_generation;
            let active_lease_conn_id = self.active_lease_conn_id;
            let active_lease_gen = self.active_lease.as_ref().map(|a| a.generation);
            let closed_connections = &self.closed_connections;
            let rt_closed_connections = self.rt_closed_connections.as_deref();
            let shared_closed_bitset = &self.shared_closed_bitset;
            let fence = crate::supervisor::LiveDisconnectFenceView::new(
                e_stop_latched,
                pending_disconnect_hold,
                atomic_fenced_lease_gen,
                fenced_lease_generation,
                active_lease_conn_id,
                active_lease_gen,
                closed_connections,
                rt_closed_connections,
                shared_closed_bitset,
            );
            let Some(slot) = self.driver.as_mut() else { return TickOutput::None; };
            let run_write = |drv: &mut dyn crate::driver::ArmDriver| {
                crate::supervisor::SupervisorInner::write_gravity_comp_targets_checked(
                    drv,
                    &targets,
                    is_hold,
                    output_lease_gen,
                    &fence,
                )
            };
            if nonblocking {
                slot.try_with_mut(run_write).ok().flatten()
            } else {
                slot.with_mut(run_write).flatten()
            }
        };
        let Some((out, effective_hold)) = maybe_out else { return TickOutput::None; };
        // Never write a 0x307 frame outside the tick epilogue's ONE J7 emitter: record intent
        // only so an in-tick urgent E-stop emits J7 strictly once in the epilogue and an off-tick
        // exit hands J7 off to the next control tick.
        let intent_stamp = self.current_j7_intent_stamp();
        self.j7_intent = Some(if effective_hold {
            crate::driver::PosForceIntent::stop(intent_stamp)
        } else {
            crate::driver::PosForceIntent::hold(intent_stamp)
        });
        out
    }

    /// Stack-allocated gravity-comp targets (`ARM` MIT joints + 1 position gripper), 0 heap allocs.
    pub(crate) fn gravity_comp_targets_array(&self) -> Option<[JointTarget; ARM + 1]> {
        let run = self.gravity_comp.as_ref()?;
        Some(std::array::from_fn(|i| {
            if i < ARM {
                JointTarget::mit(run.position[i], 0.0, run.kp[i], run.kd[i], run.tau_ff[i])
            } else {
                JointTarget {
                    position: self.commanded_positions[ARM],
                    velocity: Some(self.commanded_velocities[ARM]),
                    torque: Some(0.0), kp: None, kd: None,
                }
            }
        }))
    }

    /// Joint targets while the mode is on: MIT for J1-J6, position for the gripper (bounded inline array, 0 heap allocs).
    pub fn gravity_comp_targets(&self) -> Option<[JointTarget; ARM + 1]> {
        self.gravity_comp_targets_array()
    }

    fn gravity_comp_exit_cause(&mut self, dt: f64) -> Option<(&'static str, crate::model::HolderLabel)> {
        use crate::model::HolderLabel;
        let fmt_label = |f: &mut dyn FnMut(&mut crate::safety::BoundedText<128>)| -> HolderLabel {
            let mut buf = crate::safety::BoundedText::<128>::new("");
            f(&mut buf);
            HolderLabel::new(buf.as_str())
        };
        let run = self.gravity_comp.as_ref()?;
        let policy = &run.policy;
        if self.e_stop_latched.load(std::sync::atomic::Ordering::SeqCst) || self.safety.state() == &SafetyState::EStop {
            return Some(("ESTOP", HolderLabel::new("E-stop")));
        }
        match self.safety.state() {
            SafetyState::Fault(FaultReason::BusLoss) => return Some(("BUS_LOSS", HolderLabel::new("CAN feedback lost"))),
            SafetyState::Fault(FaultReason::WatchdogTimeout) => return Some(("WATCHDOG", HolderLabel::new("Lease watchdog"))),
            SafetyState::Fault(FaultReason::J7FeedbackStale) => return Some(("J7_FEEDBACK_STALE", HolderLabel::new("J7 position-force feedback stale"))),
            SafetyState::Fault(r) => {
                let msg = match r {
                    FaultReason::Other(m) => HolderLabel::new(m.as_str()),
                    _ => HolderLabel::new("Fault"),
                };
                return Some(("FAULT", msg));
            }
            _ => {}
        }
        if !self.is_torqued
            || self.disable_pending
            || self.torque_intent != crate::supervisor::TorqueIntent::On
        {
            return Some(("TORQUE_LOST", HolderLabel::new("Torque is off")));
        }
        if self.lease_enabled {
            if self.last_heartbeat.elapsed() > self.lease_timeout {
                let ms = self.lease_timeout.as_millis();
                return Some((
                    "WATCHDOG",
                    fmt_label(&mut |b| {
                        let _ = core::fmt::write(b, format_args!("No heartbeat for {ms} ms"));
                    }),
                ));
            }
            let generation = self.active_lease.as_ref().map(|l| l.generation);
            if generation != run.lease_generation || !self.is_lease_active() || self.handover_settling_since.is_some() {
                return Some(("LEASE_LOST", HolderLabel::new("Control lease lost, released or taken over")));
            }
        }
        if !self.feedback_numeric_valid || !self.hardware_arm_feedback_fresh(Duration::from_millis(policy.feedback_max_age_ms)) {
            let max_ms = policy.feedback_max_age_ms;
            return Some((
                "FEEDBACK_STALE",
                fmt_label(&mut |b| {
                    let _ = core::fmt::write(b, format_args!("Feedback older than {max_ms} ms"));
                }),
            ));
        }
        if let Some(msg) = self.gravity_comp_near_limit(policy) {
            return Some(("GRAVITY_COMP_JOINT_LIMIT", HolderLabel::new(msg.as_str())));
        }
        if run.phase == GravityCompPhase::RampIn {
            if let Some((i, d)) = (0..ARM).map(|i| (i, (self.measured_positions[i] - run.entry_pose[i]).abs()))
                .find(|(_, d)| !d.is_finite() || *d > policy.ramp_max_deviation_rad) {
                let lim = policy.ramp_max_deviation_rad;
                return Some((
                    "GRAVITY_COMP_SAG",
                    fmt_label(&mut |b| {
                        let _ = core::fmt::write(
                            b,
                            format_args!("Joint {} moved {d:.3} rad during the ramp-in (limit {lim} rad): the model does not hold the arm", i + 1),
                        );
                    }),
                ));
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
        let overspeed_ticks = policy.overspeed_ticks;
        let run = self.gravity_comp.as_mut()?;
        run.prev_measured = Some(measured);
        match fastest {
            Some((i, speed, limit)) => {
                run.overspeed_count += 1;
                let count = run.overspeed_count;
                if count >= overspeed_ticks {
                    return Some((
                        "GRAVITY_COMP_OVERSPEED",
                        fmt_label(&mut |b| {
                            let _ = core::fmt::write(
                                b,
                                format_args!("Joint {} at {speed:.2} rad/s for {count} ticks (limit {limit} rad/s)", i + 1),
                            );
                        }),
                    ));
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
                    if self.rt_executor_owned {
                        self.exit_gravity_comp_rt(code, message.as_str());
                    } else {
                        self.exit_gravity_comp_inner(code, message.as_str(), false);
                    }
                    return;
                }
                let model = {
                    let Some(run) = self.gravity_comp.as_ref() else { return; };
                    self.gravity_comp_model_torque(&run.policy, run.scale)
                };
                let Some(model) = model else {
                    if self.rt_executor_owned {
                        self.exit_gravity_comp_rt("GRAVITY_MODEL_UNAVAILABLE", "Gravity torque could not be computed");
                    } else {
                        self.exit_gravity_comp_inner("GRAVITY_MODEL_UNAVAILABLE", "Gravity torque could not be computed", false);
                    }
                    return;
                };
                let measured = arm6(&self.measured_positions);
                let mut ramp_completed = false;
                {
                    let Some(run) = self.gravity_comp.as_mut() else { return; };
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
                        ramp_completed = true;
                    }
                }
                if ramp_completed {
                    self.safety.log_gravity_comp_ramp_complete();
                }
                // The hand moves the arm: the reference follows the measurement, so the
                // tracking-error check sees no error from intended motion.
                for i in 0..ARM {
                    self.commanded_positions[i] = measured[i];
                    self.commanded_velocities[i] = 0.0;
                }
                crate::rt_core::scratch::copy_into(&mut self.target_positions, &self.commanded_positions);
            }
            GravityCompPhase::Exiting { .. } | GravityCompPhase::ExitFailed { .. } => {}
        }
    }

    /// Post-output advancement of gravity-comp exit handback (`Exiting` / `ExitFailed`).
    /// Advances ONLY when this release's stiff hold write completed (`Complete`), arming
    /// `gravity_comp_handback_ready` so the POS_VEL mode switch executes in the next release's
    /// dedicated TX budget without bypassing the current release's cutoff or exceeding 16 frames.
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
                    if let Some(run) = self.gravity_comp.as_mut() { run.phase = GravityCompPhase::Exiting { hold, elapsed_s: elapsed }; }
                    let tracking_active = self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled);
                    if tracking_active {
                        let fresh = self.feedback_numeric_valid
                            && self.hardware_arm_feedback_fresh(Duration::from_millis(max_age_ms));
                        let stationary = self.gravity_comp_is_stationary(max_vel);
                        // When tracking will take over POS_VEL output, wait for the arm to settle
                        // under the stiff MIT hold before scheduling the handback release (bounded
                        // by exit_hold + 1.0 s).
                        if fresh && !stationary && elapsed < exit_hold + 1.0 {
                            return;
                        }
                    }
                    self.gravity_comp_handback_ready = true;
                }
            }
            GravityCompPhase::ExitFailed { hold, retry_in_s } => {
                if !stiff_hold_completed && self.is_torqued && !self.disable_pending && !self.shutdown_committed {
                    return;
                }
                let left = retry_in_s - dt;
                if left > 0.0 {
                    if let Some(run) = self.gravity_comp.as_mut() { run.phase = GravityCompPhase::ExitFailed { hold, retry_in_s: left }; }
                } else {
                    if let Some(run) = self.gravity_comp.as_mut() { run.phase = GravityCompPhase::ExitFailed { hold, retry_in_s: 0.0 }; }
                    self.gravity_comp_handback_ready = true;
                }
            }
        }
    }

    /// Executes a scheduled POS_VEL handback on its dedicated transition release using the
    /// release's existing TX pass (`set_joint_modes_in_tick`), returning `true` when this
    /// release was consumed by the handback attempt.
    ///
    /// Exempt from `Admitted` (Invariant 1a): stop-direction exit handback transitioning a
    /// gravity-comp exit (`Exiting`/`ExitFailed`) back to stiff `POS_VEL` hold, never starting
    /// new client motion.
    pub(crate) fn run_scheduled_gravity_comp_handback(&mut self) -> bool {
        if !self.gravity_comp_handback_ready {
            return false;
        }
        let Some((hold, exiting_elapsed)) = self.gravity_comp.as_ref().and_then(|r| match r.phase {
            GravityCompPhase::Exiting { hold, elapsed_s } => Some((hold, Some(elapsed_s))),
            GravityCompPhase::ExitFailed { hold, .. } => Some((hold, None)),
            _ => None,
        }) else {
            self.gravity_comp_handback_ready = false;
            return false;
        };
        if self.urgent_stop_unapplied() {
            self.gravity_comp_handback_ready = false;
            return false;
        }
        if let Some(elapsed_s) = exiting_elapsed {
            let tracking_active = self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled);
            if tracking_active {
                let (exit_hold, max_age_ms, max_vel) = self.gravity_comp.as_ref().map_or((0.1, 100, 0.05), |r| {
                    (r.policy.exit_hold_ms as f64 / 1000.0, r.policy.feedback_max_age_ms, r.policy.entry_max_velocity)
                });
                let fresh = self.feedback_numeric_valid
                    && self.hardware_arm_feedback_fresh(Duration::from_millis(max_age_ms));
                let stationary = self.gravity_comp_is_stationary(max_vel);
                // If the arm resumed moving before the settle deadline, cancel the scheduled
                // transition release and continue sending stiff MIT holds.
                if fresh && !stationary && elapsed_s < exit_hold + 1.0 {
                    self.gravity_comp_handback_ready = false;
                    return false;
                }
            }
        }
        self.gravity_comp_handback_ready = false;
        self.gravity_comp_switch_back(hold);
        true
    }

    fn gravity_comp_is_stationary(&self, max_vel: f32) -> bool {
        let has_vel = self
            .driver
            .as_ref()
            .is_none_or(|d| d.with_ref(|drv| drv.capabilities().velocity_feedback).unwrap_or(false));
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
                && self.hardware_arm_feedback_fresh(Duration::from_millis(max_age_ms))
                && self.measured_positions.len() >= ARM
                && self.measured_positions[..ARM].iter().all(|q| q.is_finite()));
        let stationary = !tracking_active || self.gravity_comp_is_stationary(max_vel);
        let gripper = self.commanded_positions[ARM];
        let result = if !fresh {
            Err(format!("fresh feedback (<= {max_age_ms} ms) required before switching back to POS_VEL"))
        } else if !stationary {
            Err(format!("arm not stationary (limit {max_vel} rad/s) before switching back to POS_VEL"))
        } else {
            match self.driver.as_mut() {
                None => Ok(()),
                Some(d) => d
                    .with_mut(|drv| {
                        let hold_targets: [JointTarget; ARM + 1] = std::array::from_fn(|i| {
                            if i < ARM {
                                JointTarget::position(hold[i])
                            } else {
                                JointTarget::position(gripper)
                            }
                        });
                        drv.set_joint_modes_in_tick(&[JointMode::Position; ARM + 1], &hold_targets).map_err(|e| e.to_string())
                    })
                    .unwrap_or_else(|| Err("driver lock poisoned".into())),
            }
        };
        match result {
            Ok(()) => {
                if let Some(old) = self.gravity_comp.take() {
                    if self.rt_executor_owned {
                        self.retired_gravity_comp = Some(old);
                    }
                    self.bump_motion_generation();
                }
                for i in 0..ARM {
                    self.commanded_positions[i] = hold[i];
                    self.commanded_velocities[i] = 0.0;
                }
                crate::rt_core::scratch::copy_into(&mut self.target_positions, &self.commanded_positions);
                self.commanded_accelerations.fill(0.0);
                self.following_error_since = None;
                if self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled) {
                    self.tracking.seed(&self.commanded_positions);
                } else {
                    self.tracking.invalidate();
                }
                self.safety.mark_motion_finished();
                if !self.rt_executor_owned {
                    tracing::info!("Gravity compensation off: POS_VEL hold at {hold:?}");
                }
            }
            Err(e) => {
                if !self.rt_executor_owned {
                    tracing::error!("Gravity compensation: switch back to POS_VEL not confirmed: {e}");
                }
                if let Some(run) = self.gravity_comp.as_mut() { run.phase = GravityCompPhase::ExitFailed { hold, retry_in_s: 0.5 }; }
                self.safety.latch(
                    crate::safety::FaultKind::Other,
                    crate::safety::FaultDetail::formatted(crate::safety::FaultCode::GravityCompFailed, |buf| {
                        let _ = core::fmt::write(buf, format_args!("GRAVITY_COMP_EXIT_FAILED: {e}"));
                    }),
                );
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
