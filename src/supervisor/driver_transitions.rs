//! Driver transitions of the supervisor: driver attachment, feedback intake, driver limits, torque enable/disable and fault clearing (track A).
//!
//! Moved from `supervisor.rs` without changes (armd 1.6 RT-core foundation); private
//! methods are `pub(super)` so the rest of the supervisor keeps calling them.

#[allow(unused_imports)]
use super::*;

impl SupervisorInner {
    pub fn with_driver(mut self, mut driver: Box<dyn ArmDriver>) -> Self {
        driver.set_estop_gate(self.estop_gate());
        driver.set_gripper_index(self.config.gripper_index());
        driver.set_strict_mode_handover(self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled));
        if driver.capabilities().load_feedback {
            self.loads = vec![f32::NAN; self.config.dof];
        }
        let driver: Box<dyn ArmDriver> = Box::new(crate::joint_zero::ZeroOffsetDriver::new(driver, &self.joint_zero_offsets));
        self.apply_driver_limits(driver.as_ref());
        self.driver = Some(std::sync::Mutex::new(driver));
        self
    }

    pub fn sync_driver_feedback(
        &mut self,
        states: &[crate::model::JointState],
        loads: Option<&[f32]>,
    ) {
        Self::apply_feedback(
            &mut self.measured_positions,
            &mut self.measured_velocities,
            &mut self.torques,
            &mut self.temperatures,
            &mut self.loads,
            self.config.dof,
            states,
            loads,
        );
    }

    pub(super) fn apply_feedback(
        measured_positions: &mut [f32],
        measured_velocities: &mut [f32],
        torques: &mut [f32],
        temperatures: &mut [f32],
        loads_vec: &mut Vec<f32>,
        dof: usize,
        states: &[crate::model::JointState],
        loads: Option<&[f32]>,
    ) {
        for (i, state) in states.iter().enumerate().take(dof) {
            measured_positions[i] = state.position;
            measured_velocities[i] = state.velocity;
            torques[i] = state.torque;
            temperatures[i] = state.temperature;
        }
        if let Some(loads) = loads {
            if loads.len() == dof && loads.iter().all(|x| x.is_finite()) {
                copy_into(loads_vec, loads);
            }
        }
    }

    pub fn apply_driver_limits(&mut self, driver: &dyn ArmDriver) {
        let vel: Vec<_> = (0..self.config.dof).map(|i| driver.max_command_velocity(i)).collect();
        let pos: Vec<_> = (0..self.config.dof).map(|i| driver.position_limits(i)).collect();
        self.apply_extracted_driver_limits(&vel, &pos);
    }

    pub fn apply_extracted_driver_limits(
        &mut self,
        vel_limits: &[Option<f32>],
        pos_limits: &[Option<(f32, f32)>],
    ) {
        self.apply_extracted_driver_limits_inner(vel_limits, pos_limits, true);
    }

    pub fn apply_extracted_driver_limits_inner(
        &mut self,
        vel_limits: &[Option<f32>],
        pos_limits: &[Option<(f32, f32)>],
        trigger_fault: bool,
    ) {
        self.driver_config_error = None;
        let mut candidate = self.config.clone();

        for (i, limit) in candidate.limits.iter_mut().enumerate() {
            if let Some(Some(max)) = vel_limits.get(i) {
                if max.is_finite() && *max > 0.0 {
                    limit.max_velocity = limit.max_velocity.min(*max);
                }
            }
            if let Some(Some((d_min, d_max))) = pos_limits.get(i).copied() {
                if !d_min.is_finite() || !d_max.is_finite() || d_min >= d_max {
                    let reason = format!(
                        "Driver position limits [{:.4}, {:.4}] for joint '{}' are invalid",
                        d_min, d_max, limit.name
                    );
                    self.driver_config_error = Some(reason.clone());
                    if trigger_fault {
                        self.safety.trigger_fault_coded(FaultCode::JointLimit, FaultReason::Other(reason));
                    }
                    return;
                }
                let new_min = limit.min_position.max(d_min);
                let new_max = limit.max_position.min(d_max);
                if !new_min.is_finite() || !new_max.is_finite() || new_max - new_min < 1e-2 {
                    let reason = format!(
                        "Driver position limits [{:.4}, {:.4}] for joint '{}' conflict with profile limits [{:.4}, {:.4}] (intersected range < 0.01)",
                        d_min, d_max, limit.name, limit.min_position, limit.max_position
                    );
                    self.driver_config_error = Some(reason.clone());
                    if trigger_fault {
                        self.safety.trigger_fault_coded(FaultCode::JointLimit, FaultReason::Other(reason));
                    }
                    return;
                }
                if let Some(pos) = candidate.resting_pose.as_ref().and_then(|r| r.positions.as_ref()).and_then(|p| p.get(i).copied()) {
                    if pos < new_min || pos > new_max {
                        let reason = format!(
                            "Driver position limits [{:.4}, {:.4}] for joint '{}' conflict with resting pose position {:.4}",
                            d_min, d_max, limit.name, pos
                        );
                        self.driver_config_error = Some(reason.clone());
                        if trigger_fault {
                            self.safety.trigger_fault_coded(FaultCode::JointLimit, FaultReason::Other(reason));
                        }
                        return;
                    }
                }
                if (new_min - limit.min_position).abs() > 1e-4 || (new_max - limit.max_position).abs() > 1e-4 {
                    tracing::info!(
                        "Intersected joint '{}' position limits with driver range: profile=[{:.4}, {:.4}], hw=[{:.4}, {:.4}] -> [{:.4}, {:.4}]",
                        limit.name, limit.min_position, limit.max_position, d_min, d_max, new_min, new_max
                    );
                }
                limit.min_position = new_min;
                limit.max_position = new_max;
            }
        }

        if candidate.validate().is_err()
            && self.base_gripper_policy.is_some()
            && (self.gripper_calib_offset.is_some() || self.gripper_calib_jaw_opening.is_some())
        {
            let mut base_test = candidate.clone();
            base_test.gripper_policy = self.base_gripper_policy.clone();
            if base_test.validate().is_ok() {
                tracing::warn!("Driver limits invalidated outdated calibrated gripper policy; reverting to base uncalibrated policy");
                self.gripper_calib_invalid = true;
                self.gripper_calib_offset = None;
                self.gripper_calib_jaw_opening = None;
                self.config.gripper_policy = self.base_gripper_policy.clone();
                candidate = base_test;
            }
        }

        if let Err(err) = candidate.validate() {
            let reason = format!("Driver limits violated config invariants: {err}");
            self.driver_config_error = Some(reason.clone());
            if trigger_fault {
                self.safety.trigger_fault_coded(FaultCode::JointLimit, FaultReason::Other(reason));
            }
            return;
        }

        self.config = candidate;
    }

    pub fn reapply_driver_limits(&mut self) -> bool {
        let limits = if let Some(ref d) = self.driver {
            if let Ok(drv) = d.lock() {
                let vel: Vec<_> = (0..self.config.dof).map(|i| drv.max_command_velocity(i)).collect();
                let pos: Vec<_> = (0..self.config.dof).map(|i| drv.position_limits(i)).collect();
                Some((vel, pos))
            } else {
                None
            }
        } else {
            None
        };
        if let Some((vel_limits, pos_limits)) = limits {
            self.apply_extracted_driver_limits_inner(&vel_limits, &pos_limits, false);
            self.driver_config_error.is_none()
        } else {
            false
        }
    }

    /// Store the motor error codes, logging each per-joint change once.
    pub(super) fn update_motor_errors(&mut self, errors: Vec<Option<crate::model::MotorError>>) {
        if errors == self.motor_errors { return; }
        for (i, e) in errors.iter().enumerate() {
            let prev = self.motor_errors.get(i).cloned().flatten();
            match (prev, e) {
                (p, Some(e)) if p.as_ref() != Some(e) =>
                    tracing::warn!("Joint {} motor reports error code 0x{:X} ({})", i + 1, e.code, e.name),
                (Some(p), None) => tracing::info!("Joint {} motor error 0x{:X} ({}) cleared", i + 1, p.code, p.name),
                _ => {}
            }
        }
        self.motor_errors = errors;
    }

    pub(super) fn states_valid(
        states: &[crate::model::JointState],
        caps: DriverCapabilities,
        dof: usize,
        loads: Option<&[f32]>,
    ) -> bool {
        states.len() == dof
            && states.iter().all(|s| {
                s.position.is_finite()
                    && (!caps.velocity_feedback || s.velocity.is_finite())
                    && (!caps.torque_feedback || s.torque.is_finite())
                    && (!caps.temperature_feedback || s.temperature.is_finite())
            })
            && (!caps.load_feedback
                || loads.is_some_and(|l| l.len() == dof && l.iter().all(|x| x.is_finite())))
    }

    pub fn hardware_feedback_fresh(&self, max_age: Duration) -> bool {
        if !self.feedback_numeric_valid { return false; }
        // Ages alone decide: drivers' is_feedback_valid() hard-codes 250 ms,
        // which would silently cap callers asking for a longer bound.
        self.driver.as_ref().is_none_or(|d| d.lock().is_ok_and(|drv| drv.feedback_all_within(self.config.dof, max_age)))
    }

    /// Update per-joint feedback instants and monotonic sequence counters from a hardware driver
    /// (read into `sc`, no allocation).
    pub(super) fn record_driver_feedback_timing(
        joint_feedback_instants: &mut Vec<Option<Instant>>,
        joint_feedback_seqs: &mut Vec<u64>,
        joint_feedback_queue_bounds: &mut Vec<Option<Instant>>,
        dof: usize,
        driver: &dyn ArmDriver,
        sc: &mut crate::rt_core::scratch::FeedbackScratch,
    ) {
        driver.feedback_instants_into(&mut sc.instants);
        driver.feedback_seqs_into(&mut sc.seqs);
        driver.feedback_queue_bounds_into(&mut sc.bounds);
        if sc.bounds.len() == dof {
            copy_into(joint_feedback_queue_bounds, &sc.bounds);
        } else {
            joint_feedback_queue_bounds.clear();
        }
        if joint_feedback_seqs.len() != dof {
            joint_feedback_seqs.resize(dof, 0);
        }
        if sc.seqs.len() == dof {
            copy_into(joint_feedback_seqs, &sc.seqs);
        } else {
            for i in 0..dof {
                let prev = joint_feedback_instants.get(i).copied().flatten();
                let cur = sc.instants.get(i).copied().flatten();
                if let Some(t) = cur {
                    if prev.is_none_or(|p| t > p) {
                        joint_feedback_seqs[i] = joint_feedback_seqs[i].wrapping_add(1);
                    }
                }
            }
        }
        copy_into(joint_feedback_instants, &sc.instants);
    }

    /// Per arm joint, an instant before which its last sample cannot have been enqueued: the
    /// driver's receive-queue bound, or the receive instant for drivers without a receive queue.
    pub fn arm_feedback_queue_bounds(&self) -> Vec<Option<Instant>> {
        self.config.arm_joint_indices().iter().map(|&i| match self.joint_feedback_queue_bounds.get(i) {
            Some(b) => *b,
            None => self.core.joint_feedback_instants.get(i).copied().flatten(),
        }).collect()
    }

    /// Snapshot of `(measured_positions, arm_velocities, arm_feedback_instants, arm_feedback_seqs)`
    /// read coherently under the supervisor lock.
    pub fn arm_feedback_snapshot(&self) -> (Vec<f32>, Vec<f32>, Vec<Option<Instant>>, Vec<u64>) {
        let arm = self.config.arm_joint_indices();
        let vels = arm.iter().map(|&i| self.measured_velocities.get(i).copied().unwrap_or(0.0)).collect();
        let instants = arm
            .iter()
            .map(|&i| self.core.joint_feedback_instants.get(i).copied().flatten())
            .collect();
        let seqs = arm
            .iter()
            .map(|&i| self.core.joint_feedback_seqs.get(i).copied().unwrap_or(0))
            .collect();
        (self.measured_positions.clone(), vels, instants, seqs)
    }

    /// A successful transport write is not confirmation that torque was removed.
    pub(super) fn request_disable(&mut self) -> Result<(), SafetyError> {
        self.mit_torque_off();
        self.disable_pending = true;
        self.tracking.invalidate();
        self.tracking.clear_recovery_suspension();
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

    pub fn set_torque(&mut self, enable: bool) -> Result<SafetyState, SafetyError> {
        // Torque changes and re-arming are refused in gravity compensation: exit first.
        self.gravity_comp_servo_busy()?;
        if enable {
            if let Some(ref err) = self.driver_config_error {
                return Err(SafetyError::MotionForbiddenInFaultState(format!("Driver limit configuration error: {err}")));
            }
            if self.handover_settling_since.is_some() {
                return Err(SafetyError::HandoverSettling);
            }
            if self.disable_pending { return Err(SafetyError::ClearFaultRejected("Torque disable is still pending".into())); }
            // Atomic e-stop check
            if self.estop_pending() {
                return Err(SafetyError::EStopActive);
            }

            // Strict pre-check: If stopped (E-Stop or Fault), reject IMMEDIATELY before touching driver!
            if self.safety.state().is_stopped() {
                if let SafetyState::Fault(ref reason) = self.safety.state() {
                    return Err(SafetyError::FaultActive(reason.clone()));
                }
                return Err(SafetyError::EStopActive);
            }

            // MIT tracking (Phase 2): every arm and rearm rechecks the qualification against a fresh
            // register readback (nonzero TIMEOUT, identity, ranges); unqualified hardware refuses
            // MIT before any motion. MIT not requested: no readback, no change.
            // A rearm while MIT owns joints hands them back first (at rest), so neither the
            // register readback below nor the driver's enable runs under an active MIT owner
            // (no dynamic output during the reads, no silent mode restore).
            if self.mit_active() {
                self.mit.armed_ok = false;
                if let Err(e) = self.mit_hand_back("rearm") {
                    // not at rest: the authorization is gone, so the motion brakes to rest and MIT
                    // hands back there (the tick); no motion starts meanwhile
                    if self.mit_active() {
                        self.active_trajectory = None;
                        self.servo_target = None;
                        self.mit.revoke_pending = true;
                    }
                    return Err(SafetyError::ClearFaultRejected(e));
                }
            }
            if let Err(e) = self.mit_gate() {
                // revoked before returning: no handover until a later arm passes the gate
                self.mit.armed_ok = false;
                let msg = format!("MIT_UNQUALIFIED: MIT tracking refused: {e}");
                tracing::error!("{msg}");
                self.mit_gate_error = Some(msg.clone());
                return Err(SafetyError::ClearFaultRejected(msg));
            }
            self.mit_gate_error = None;

            // Enable hardware torque so bus devices wake up and publish state, then verify physical feedback
            let any_driver_enabled = self.driver.as_ref().is_some_and(|d| d.lock().is_ok_and(|drv| drv.has_any_joint_enabled()));
            let was_torqued = self.is_torqued || any_driver_enabled;
            let prev_gripper_cmd = if was_torqued {
                self.config.gripper_index().and_then(|i| self.commanded_positions.get(i).copied())
            } else {
                None
            };

            if let Some(ref d) = self.driver {
                // Secondary double-check latch right before hardware actuation
                if self.estop_pending() {
                    return Err(SafetyError::EStopActive);
                }
                let mut driver = d.lock().map_err(|_| SafetyError::FeedbackUnavailable)?;
                if self.estop_pending() {
                    return Err(SafetyError::EStopActive);
                }
                if !driver.is_feedback_valid() {
                    if let Ok(states) = driver.read_joint_states() {
                        let loads = driver.joint_loads();
                        if !states.is_empty() && driver.is_feedback_valid() && SupervisorInner::states_valid(&states, driver.capabilities(), self.config.dof, loads.as_deref()) {
                            Self::apply_feedback(
                                &mut self.measured_positions,
                                &mut self.measured_velocities,
                                &mut self.torques,
                                &mut self.temperatures,
                                &mut self.loads,
                                self.config.dof,
                                &states,
                                loads.as_deref(),
                            );
                            for (i, s) in states.iter().enumerate().take(self.config.dof) {
                                if s.position.is_finite() {
                                    self.commanded_positions[i] = s.position;
                                    self.target_positions[i] = s.position;
                                }
                            }
                        }
                    }
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
                    self.safety.trigger_fault_coded(FaultCode::EnableFailed, FaultReason::Other(err_msg.clone()));
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
                let loads = driver.joint_loads();
                if !driver.is_feedback_valid() || !Self::states_valid(&states, driver.capabilities(), self.config.dof, loads.as_deref())
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
                Self::record_driver_feedback_timing(
                    &mut self.core.joint_feedback_instants,
                    &mut self.core.joint_feedback_seqs,
                    &mut self.joint_feedback_queue_bounds,
                    self.config.dof,
                    driver.as_ref(),
                    &mut self.scratch.fb,
                );
                Self::apply_feedback(
                    &mut self.measured_positions,
                    &mut self.measured_velocities,
                    &mut self.torques,
                    &mut self.temperatures,
                    &mut self.loads,
                    self.config.dof,
                    &states,
                    loads.as_deref(),
                );
            }

            // Initialize commanded_positions to verified measured_positions to prevent sudden jumps
            self.commanded_positions = self.measured_positions.clone();
            self.target_positions = self.measured_positions.clone();
            // Preserve gripper holding position when re-arming an already torqued arm so grasped objects are not released
            if let Some(g_cmd) = prev_gripper_cmd {
                if let Some(grip_idx) = self.config.gripper_index() {
                    self.commanded_positions[grip_idx] = g_cmd;
                    self.target_positions[grip_idx] = g_cmd;
                }
            }
            self.commanded_velocities = vec![0.0; self.config.dof];
            self.commanded_accelerations.fill(0.0);
            if self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled) {
                let arm_indices = self.config.arm_joint_indices();
                let has_vel_fb = self.driver.as_ref().is_none_or(|d| {
                    d.lock().is_ok_and(|drv| drv.capabilities().velocity_feedback)
                });
                // Positive evidence only: without velocity feedback a single sample cannot show
                // rest; the history stays Unknown until two samples do (plain output meanwhile).
                let arm_stationary = has_vel_fb
                    && arm_indices.iter().all(|&j| {
                        self.measured_velocities
                            .get(j)
                            .is_some_and(|v| v.is_finite() && v.abs() <= crate::tracking::STATIONARY_VELOCITY_RAD_S)
                    });
                let aligned_with_issued = !self
                    .tracking
                    .has_residual_lead(&self.commanded_positions, &arm_indices);
                if was_torqued && self.tracking.last_issued_positions.is_some() {
                    if arm_stationary && aligned_with_issued {
                        self.tracking.seed(&self.commanded_positions);
                    } else {
                        // Already torqued and moving or holding active lead: preserve real
                        // issued history so the next tick ramps smoothly instead of jumping.
                        self.tracking.ws_min_depths = None;
                    }
                } else if arm_stationary {
                    self.tracking.seed(&self.commanded_positions);
                } else {
                    self.tracking.invalidate();
                }
            } else {
                self.tracking.invalidate();
            }
            if self.driver.is_none() { self.measured_velocities.fill(0.0); }
            self.active_trajectory = None;
            self.pending_normal_unload = false;
            self.unload_settle = None;
            self.watchdog_park = None;
            self.servo_target = None;
            self.servo_stream_active = false;
            self.servo_ws_min_depths = None;

            let state = self.safety.set_torque(true)?;
            self.is_torqued = true;
            self.pending_rearm = false;
            self.disable_pending = false;
            self.mit.armed_ok = !crate::mit::mit_joints(&self.config).is_empty();
            self.mit.handover_error = None;
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
    pub(super) fn torque_off(&mut self, stillness_verified: bool) -> Result<SafetyState, SafetyError> {
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
                expected: self.config.resting_positions().to_vec(),
                reason: Some(reason),
            });
        }

        self.stop_motion();
        self.commanded_velocities.fill(0.0);
        if let Err(error) = self.request_disable() {
            self.safety.trigger_fault_coded(FaultCode::DriverWriteFailed, FaultReason::Other(error.to_string()));
            return Err(error);
        }
        let state = self.safety.set_torque(false)?;
        self.reset_gripper_stall();
        if self.driver.is_none() { self.measured_velocities.fill(0.0); }
        Ok(state)
    }

    /// In Fault / EStop with torque on: fresh feedback from every motor reporting Disabled
    /// (Damiao: all fresh 0xCC / feedback states 0) means the motors lost torque, e.g. a
    /// motor power reset during a bus loss (real arm, 2026-11-12: all seven motors came
    /// back Disabled after BusLoss and armd kept `is_torqued: true`). armd then records
    /// torque off (`is_torqued` false, re-arm needs the full enable handshake) and keeps the
    /// Fault until `clear_fault`. Not applied during gravity compensation (its exit path
    /// owns the motor modes) or while a disable is pending.
    pub(super) fn sync_torque_state_after_stop(&mut self) {
        if !self.is_torqued || self.disable_pending || self.gravity_comp.is_some() || !self.safety.state().is_stopped() {
            return;
        }
        let Some(d) = self.driver.as_ref() else { return; };
        let off = d.lock().is_ok_and(|drv| drv.capabilities().enable_feedback && drv.is_feedback_valid()
            && drv.enable_state() == EnableState::Disabled);
        if !off || !self.hardware_feedback_fresh(Duration::from_millis(250)) { return; }
        if let Ok(mut drv) = d.lock() { drv.set_torque_flag(false); }
        tracing::warn!("Motors report Disabled with fresh feedback in {:?}: torque is off (is_torqued = false); the fault stays latched until clear_fault",
            self.safety.state());
        self.is_torqued = false;
        self.pending_rearm = true;
        self.output_suspended = false;
        self.gripper_stalled = false;
        self.gripper_stall_sign = 0.0;
        self.gripper_prev_hw_overload = false;
        self.stop_motion();
        self.servo_stream_active = false;
        self.tracking.invalidate();
        if self.measured_positions.iter().all(|q| q.is_finite()) {
            self.commanded_positions = self.measured_positions.clone();
            self.target_positions = self.measured_positions.clone();
        }
        self.commanded_velocities.fill(0.0);
        self.commanded_accelerations.fill(0.0);
    }

    pub fn clear_fault(&mut self) -> Result<SafetyState, SafetyError> {
        Self::check_concurrent_estop(self)?;
        if self.driver_config_error.is_some() {
            self.reapply_driver_limits();
        }
        if let Some(ref err) = self.driver_config_error {
            return Err(SafetyError::ClearFaultRejected(format!("Driver limit configuration error: {err}")));
        }
        if !self.safety.state().is_stopped() { return Ok(self.safety.state().clone()); }
        if self.gravity_comp_blocks_clear() {
            return Err(SafetyError::ClearFaultRejected("Gravity compensation: the switch back to POS_VEL is not confirmed yet".into()));
        }
        if self.disable_pending { return Err(SafetyError::ClearFaultRejected("Torque disable is still unconfirmed".into())); }
        if self.mit.owner == crate::mit::MitOwner::Lost {
            // the operator's clearance lets the driver poll again (fresh feedback for the checks
            // below); writes stay refused until the rearm's confirmed enable
            if let Some(mut drv) = self.driver.as_ref().and_then(|d| d.lock().ok()) { drv.acknowledge_mit_fence_trip(); }
        }
        if self.mit.owner == crate::mit::MitOwner::Lost
            && self.driver.as_ref().is_some_and(|d| d.lock().is_ok_and(|drv| drv.joint_modes_uncertain())) {
            return Err(SafetyError::ClearFaultRejected("MIT tracking: the control mode of a joint is uncertain after a failed switch; \
                check the arm and power-cycle the motors".into()));
        }
        if self.mit.owner == crate::mit::MitOwner::Lost {
            // decided from feedback read for this request, never from the cache
            let fresh = match self.driver.as_ref().and_then(|d| d.lock().ok()) {
                Some(mut drv) => drv.mit_fresh_status(),
                None => Err("no driver".into()),
            };
            let st = fresh.map_err(|e| SafetyError::ClearFaultRejected(format!(
                "MIT tracking: fresh feedback is required before clearing ({e}); nothing was sent")))?;
            if st.iter().any(|&c| c >= 8) || (st.contains(&0) && st.contains(&1)) {
                if !std::mem::take(&mut self.mit.recovery_confirmed) {
                    return Err(SafetyError::ClearFaultRejected(format!("{}: {}", crate::mit::RECOVERY_CONFIRM_CODE, crate::mit::RECOVERY_WARNING)));
                }
                return self.mit_expiry_recovery();
            }
        }
        let thermal = matches!(self.safety.state(), SafetyState::Fault(FaultReason::ThermalTrip { .. }));
        if let Some(ref d) = self.driver {
            let mut drv = d.lock().map_err(|_| SafetyError::FeedbackUnavailable)?;
            let disabling = drv.clear_fault_disables_joints();
            if (self.is_torqued || drv.has_any_joint_enabled() || drv.enable_state() != EnableState::Disabled)
                && disabling.iter().any(|&j| j < self.config.arm_dof())
            {
                let states = drv.read_joint_states().map_err(|e| SafetyError::ClearFaultRejected(e.to_string()))?;
                let ages = drv.feedback_ages();
                let loads = drv.joint_loads();
                let max_age = self.resting_policy().map(|p| Duration::from_millis(p.feedback_max_age_ms)).unwrap_or(Duration::from_millis(250));
                if !drv.is_feedback_valid()
                    || ages.len() != self.config.dof
                    || !ages.iter().all(|a| a.is_some_and(|t| t <= max_age))
                    || !Self::states_valid(&states, drv.capabilities(), self.config.dof, loads.as_deref())
                {
                    return Err(SafetyError::ClearFaultRejected("Fresh feedback is required before checking resting pose support".into()));
                }
                self.feedback_numeric_valid = true;
                self.last_can_feedback_instant = ages.into_iter().collect::<Option<Vec<_>>>()
                    .and_then(|a| a.into_iter().max()).and_then(|age| Instant::now().checked_sub(age));
                Self::record_driver_feedback_timing(
                    &mut self.core.joint_feedback_instants,
                    &mut self.core.joint_feedback_seqs,
                    &mut self.joint_feedback_queue_bounds,
                    self.config.dof,
                    drv.as_ref(),
                    &mut self.scratch.fb,
                );
                Self::apply_feedback(
                    &mut self.measured_positions,
                    &mut self.measured_velocities,
                    &mut self.torques,
                    &mut self.temperatures,
                    &mut self.loads,
                    self.config.dof,
                    &states,
                    loads.as_deref(),
                );
                if let Some(reason) = self.resting_pose_support_violation_with_driver(Some(drv.as_ref())) {
                    return Err(SafetyError::ClearFaultRejected(format!(
                        "Cannot disable faulted supporting arm joint away from resting pose ({reason}); support the arm at rest first"
                    )));
                }
            }
            // Clearing a latched motor error requires a later feedback observation, not a cleared cache.
            if drv.check_hardware_status().is_err() {
                drv.clear_fault().map_err(|e| SafetyError::ClearFaultRejected(e.to_string()))?;
                return Err(SafetyError::ClearFaultRejected("Hardware reset sent; wait for fresh fault-free feedback before retrying".into()));
            }
            let states = drv.read_joint_states().map_err(|e| SafetyError::ClearFaultRejected(e.to_string()))?;
            let ages = drv.feedback_ages();
            let loads = drv.joint_loads();
            if !drv.is_feedback_valid() || ages.len() != self.config.dof
                || ages.iter().any(|a| a.is_none_or(|t| t > Duration::from_millis(250)))
                || !Self::states_valid(&states, drv.capabilities(), self.config.dof, loads.as_deref()) {
                return Err(SafetyError::ClearFaultRejected("Fresh feedback is required before resetting a fault".into()));
            }
            drv.check_hardware_status().map_err(|e| SafetyError::ClearFaultRejected(e.to_string()))?;
            if thermal && (!drv.capabilities().temperature_feedback || states.iter().any(|state|
                !state.temperature.is_finite() || state.temperature > self.config.max_temperature - THERMAL_CLEAR_MARGIN_C)) {
                return Err(SafetyError::ClearFaultRejected("Temperature must be confirmed below the reset threshold".into()));
            }
            let mut state = drv.enable_state();
            let mut partial = drv.has_known_partial_enable();
            let mut any_enabled = drv.has_any_joint_enabled();
            if state == EnableState::Unknown && !partial {
                let _ = drv.refresh_enable_state();
                state = drv.enable_state();
                partial = drv.has_known_partial_enable();
                any_enabled = drv.has_any_joint_enabled();
            }
            if state == EnableState::Unknown && !partial {
                return Err(SafetyError::ClearFaultRejected("Hardware enable state is unknown; reset remains latched".into()));
            }
            self.feedback_numeric_valid = true;
            self.last_can_feedback_instant = ages.into_iter().collect::<Option<Vec<_>>>()
                .and_then(|a| a.into_iter().max()).and_then(|age| Instant::now().checked_sub(age));
            Self::record_driver_feedback_timing(
                &mut self.core.joint_feedback_instants,
                &mut self.core.joint_feedback_seqs,
                &mut self.joint_feedback_queue_bounds,
                self.config.dof,
                drv.as_ref(),
                &mut self.scratch.fb,
            );
            Self::apply_feedback(
                &mut self.measured_positions,
                &mut self.measured_velocities,
                &mut self.torques,
                &mut self.temperatures,
                &mut self.loads,
                self.config.dof,
                &states,
                loads.as_deref(),
            );
            self.is_torqued = state == EnableState::Enabled || partial || any_enabled;
        } else if thermal && self.temperatures.iter().any(|t| !t.is_finite() || *t > self.config.max_temperature - THERMAL_CLEAR_MARGIN_C) {
            return Err(SafetyError::ClearFaultRejected("Temperature is above the reset threshold".into()));
        }
        self.pending_rearm = true;
        if self.mit.owner == crate::mit::MitOwner::Lost {
            // cleared: a rearm (fresh readback, base modes restored) and a fresh handover follow
            self.mit.owner = crate::mit::MitOwner::Off;
            self.mit.joints.clear();
            self.mit.armed_ok = false;
        }
        self.stop_motion();
        self.servo_stream_active = false;
        self.tracking.invalidate();
        self.tracking.clear_recovery_suspension();
        // Reset at the measured position; never retain a residual old trajectory target.
        self.commanded_positions = self.measured_positions.clone();
        self.target_positions = self.measured_positions.clone();
        self.commanded_velocities.fill(0.0);
        self.last_commanded_velocities.fill(0.0);
        self.commanded_accelerations.fill(0.0);
        self.following_error_since = None;
        self.bus_loss_instant = None;
        self.reset_gripper_stall();
        Self::check_concurrent_estop(self)?;
        let was_estop = *self.safety.state() == SafetyState::EStop;
        let state = self.safety.clear_fault()?;
        self.handover_settling_since = None;
        if !was_estop && self.e_stop_latched.load(Ordering::SeqCst) {
            self.apply_stop_policy(StopPolicy::EStopLatch);
            self.last_heartbeat = Instant::now();
            self.last_motion_frame = Instant::now();
            return Ok(self.safety.state().clone());
        }
        self.e_stop_latched.store(false, Ordering::SeqCst);
        Self::check_concurrent_estop(self)?;
        self.last_heartbeat = Instant::now();
        self.last_motion_frame = Instant::now();
        Ok(state)
    }

    pub fn driver_feedback_confirmed(&self) -> bool {
        match &self.driver {
            None => false,
            Some(d) => d.lock().is_ok_and(|drv| drv.is_feedback_valid()),
        }
    }
}
