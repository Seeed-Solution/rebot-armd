//! Servo streams and the policy runner of the supervisor: servo target validation, bounded servo steps, tracking fallback, servo holds and faults, policy start/step/stop (track D).
//!
//! Moved from `supervisor.rs` without changes (armd 1.6 RT-core foundation); private
//! methods are `pub(super)` so the rest of the supervisor keeps calling them.

#[allow(unused_imports)]
use super::*;

impl SupervisorInner {
    /// Joint limits used for servo stepping: `config.limits` with `max_velocity` capped
    /// at `config.servo_max_velocity` per joint, except the 7-DOF gripper (index 6).
    /// When a live policy run has a `verified_release_gate`, each joint's `max_velocity`
    /// is further scaled by `gate.max_speed_scale().clamp(0.1, 1.0)`.
    pub fn servo_limits(&self) -> Vec<crate::model::JointLimit> {
        let mut limits = Vec::with_capacity(self.config.limits.len());
        self.servo_limits_into(&mut limits);
        limits
    }

    /// [`Self::servo_limits`] into `limits`, reusing its memory (the tick's form).
    pub fn servo_limits_into(&self, limits: &mut Vec<crate::model::JointLimit>) {
        self.config.servo_limits_into(limits);
        #[cfg(feature = "policy-runtime")]
        if let Some(ref run) = self.policy_run {
            if run.mode == crate::policy_runtime::PolicyMode::Live {
                if let Some(ref gate) = run.verified_release_gate {
                    let scale = (gate.max_speed_scale() as f32).clamp(0.1, 1.0);
                    for limit in limits.iter_mut() {
                        limit.max_velocity *= scale;
                    }
                }
            }
        }
    }

    /// The control loop dropped or could not follow a live policy target: end the owning
    /// policy action as failed with `reason` (no-op without a live policy run).
    #[allow(unused_variables)]
    pub(super) fn fail_live_policy_from_control(&mut self, reason: &str, message: String) {
        #[cfg(feature = "policy-runtime")]
        if self.policy_run.as_ref().is_some_and(|r| r.mode == crate::policy_runtime::PolicyMode::Live) {
            tracing::warn!("Policy run ended by the control loop: {reason}: {message}");
            self.terminate_policy_run(reason, "failed", Some(reason), Some(message));
        }
    }

    pub(super) fn clear_blocked_hold(&mut self) {
        self.servo_blocked_hold = false;
        if self.motion_limit.as_ref().is_some_and(|l| l.reason == MotionLimitReason::Blocked) {
            self.motion_limit = None;
        }
    }

    /// Servo streaming found joint `joint` blocked: end the stream and hold the measured
    /// pose with torque on, without a Fault. The gripper keeps its command so that a held
    /// object is not released. A tracking error that persists in this hold latches the
    /// usual tracking Fault (the stream is no longer active).
    pub(super) fn enter_servo_blocked_hold(&mut self, joint: usize, why: &str) {
        tracing::warn!(
            "Joint {} blocked during servo streaming ({why}; cmd {:.3} meas {:.3} rad): stream stopped, holding position",
            joint + 1, self.commanded_positions[joint], self.measured_positions[joint]);
        #[cfg(feature = "policy-runtime")]
        self.terminate_policy_run(
            "BLOCKED_HOLD",
            "cancelled",
            Some("BLOCKED_HOLD"),
            Some(format!("Joint {} blocked during policy streaming: {why}", joint + 1)),
        );
        self.servo_hold(MotionLimitReason::Blocked, Some(joint));
    }

    /// The servo / braking step to `next` violates the workspace: same recoverable hold.
    pub(super) fn enter_servo_keepout_hold(&mut self, e: &SafetyError) {
        tracing::warn!("Servo step violates workspace ({e}): stream stopped, holding position");
        #[cfg(feature = "policy-runtime")]
        self.terminate_policy_run(
            "WORKSPACE_HOLD",
            "cancelled",
            Some("WORKSPACE_HOLD"),
            Some(format!("Policy step violated workspace: {e}")),
        );
        self.servo_hold(MotionLimitReason::Workspace, None);
        if self.motion_limit.as_ref().is_some_and(|m| m.reason == MotionLimitReason::Workspace) {
            self.workspace_violation = e.workspace_detail();
        }
    }

    /// Joint-space step from the commanded position to `next` (linear in joint space)
    /// against the full workspace (floor, outer box, reach, keep-out boxes, link capsules),
    /// using the persistent per-constraint monotonic escape rule and up to [`MAX_KEEPOUT_EVALS_PER_TICK`]
    /// added FK evaluations.
    pub fn servo_step_keepout(&mut self, next: &[f32]) -> Result<(), SafetyError> {
        self.servo_step_workspace(next, "servo_step")
    }

    /// Full workspace sweep check for a servo streaming or braking tick.
    pub fn servo_step_workspace(&mut self, next: &[f32], phase: &str) -> Result<(), SafetyError> {
        // the evaluator's vectors are reused from tick to tick (no allocation in the tick)
        let mut bufs = std::mem::take(&mut self.ws_eval_bufs);
        let result = self.servo_step_workspace_in(next, phase, &mut bufs);
        self.ws_eval_bufs = bufs;
        let cand_min_d = result?;
        if let Some(min_d) = cand_min_d { self.servo_ws_min_depths = Some((self.workspace_revision, min_d)); }
        Ok(())
    }

    /// [`Self::servo_step_workspace`]'s check; the new minimum depths (None: no workspace).
    pub(super) fn servo_step_workspace_in(&self, next: &[f32], phase: &str, bufs: &mut crate::chain::EvalBufs)
        -> Result<Option<crate::kinematics::SlotBuf>, SafetyError> {
        let Some((ws, cam)) = self.workspace_policy() else { return Ok(None); };
        let eval = crate::chain::WorkspaceEvaluator::new_with_tcp_in(
            &self.config.arm_model,
            ws,
            cam,
            self.config.tcp_offset,
            bufs,
        );
        let checked = (|| {
            let mut c0 = eval.alloc_buf();
            eval.eval_into(&self.commanded_positions, &mut c0, phase)
                .map_err(SafetyError::from_workspace)?;
            let mut cand_min_d = match &self.servo_ws_min_depths {
                Some((rev, min_d)) if *rev == self.workspace_revision => min_d.clone(),
                _ => eval.init_min_depths(&c0),
            };
            let mut budget = MAX_KEEPOUT_EVALS_PER_TICK;
            eval.sweep_linear_step(
                &self.commanded_positions,
                next,
                &c0,
                &mut cand_min_d,
                &mut budget,
                MAX_KEEPOUT_EVALS_PER_TICK,
                phase,
                phase,
            )
            .map_err(|(_, f)| SafetyError::from_workspace(f))?;
            Ok(Some(cand_min_d))
        })();
        eval.recycle(bufs);
        checked
    }

    pub(super) fn servo_hold(&mut self, reason: MotionLimitReason, joint: Option<usize>) {
        self.hold_measured_pose();
        self.note_motion_limit(reason, joint);
        self.servo_blocked_hold = true;
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

    pub(super) fn validate_and_filter_servo_targets(
        &mut self,
        targets: &mut [f32],
        epoch_id: u64,
    ) -> Result<(), SafetyError> {
        if self.config.active_tool.commissioning_required {
            return Err(SafetyError::CommissioningRequired("servo".into()));
        }
        self.gravity_comp_servo_busy()?;
        self.collision_hold_check()?;
        if self.handover_settling_since.is_some() {
            return Err(SafetyError::HandoverSettling);
        }

        if self.estop_pending() {
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
        if self.active_trajectory.is_some() || self.macro_run.is_some() {
            return Err(SafetyError::MotionBusy);
        }

        // 1. Validate motion command against epoch, safety state, and soft limits
        if let Err(e) = self.safety.validate_motion_command(Some(epoch_id), targets, &self.config.limits) {
            if let SafetyError::SoftLimitExceeded { joint, .. } = &e {
                let idx = self.config.limits.iter().position(|l| &l.name == joint);
                self.note_motion_limit(MotionLimitReason::JointLimit, idx);
            }
            return Err(e);
        }

        // 1b. Cartesian workspace: the target may not leave the box further than now.
        if let Err(e) = self.check_workspace_path(&self.commanded_positions, [&*targets]) {
            let e = e.with_workspace_phase("servo_target");
            self.workspace_violation = e.workspace_detail();
            self.note_motion_limit(MotionLimitReason::Workspace, None);
            return Err(e);
        }
        self.torque_hold_filter(targets);
        self.power_hold_filter(targets);
        Ok(())
    }

    /// Servo / policy intake of the gripper target: records it as the stream's gripper target and
    /// releases the stall latch on a target away from the stall. A torque-cap latch needs the
    /// target [`GRIPPER_FORCE_RELEASE_MARGIN_RAD`] more open than the hold.
    pub(super) fn release_gripper_stall_on_reverse(&mut self, targets: &[f32]) {
        if let Some(g_idx) = self.config.gripper_index() {
            self.gripper_stream_target = targets.get(g_idx).copied().filter(|t| t.is_finite());
            if self.gripper_stalled && targets.len() > g_idx && self.commanded_positions.len() > g_idx {
                let delta = targets[g_idx] - self.commanded_positions[g_idx];
                let tau_g = self.torques.get(g_idx).copied().unwrap_or(0.0);
                let reversed = if self.gripper_force_capped {
                    delta * self.gripper_stall_sign <= -GRIPPER_FORCE_RELEASE_MARGIN_RAD
                } else {
                    (delta * tau_g < -1e-4) || (delta * self.gripper_stall_sign < -1e-4)
                };
                if reversed {
                    self.gripper_stalled = false;
                    self.gripper_high_torque_ticks = 0;
                    self.gripper_low_torque_ticks = 0;
                }
            }
        }
    }

    pub fn servo(&mut self, mut targets: Vec<f32>, epoch_id: u64) -> Result<(), SafetyError> {
        #[cfg(feature = "policy-runtime")]
        if self.policy_run.is_some() || self.policy_loading {
            return Err(SafetyError::MotionBusy);
        }
        if self.mit_motion_blocker().is_some() { return Err(SafetyError::MotionBusy); }
        self.validate_and_filter_servo_targets(&mut targets, epoch_id)?;
        if self.servo_blocked_hold { self.clear_blocked_hold(); }

        // Only renew lease and motion frame timestamp AFTER command validation succeeds
        self.last_heartbeat = Instant::now();
        self.last_motion_frame = Instant::now();
        if let Some(ref mut l) = self.active_lease {
            l.last_valid_motion_instant = Some(Instant::now());
        }

        // Gripper stall release check on reverse command
        self.release_gripper_stall_on_reverse(&targets);

        let _ = self.safety.mark_motion_started();

        // 2. Store servo target for tick(dt) to step with actual dt and velocity limits
        self.target_positions = targets.clone();
        self.servo_target = Some(targets);
        self.servo_stream_active = true;
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

    /// Write one rejection-fallback step (`tracking_fallback_compute`) and keep the braking
    /// ramp going while the output is not at rest. When no step is feasible,
    /// latch a fault and send no frame this tick; the command stays on the last issued
    /// output so the following fault-hold frames repeat what the motors already hold.
    /// Returns a driver write error.
    pub(super) fn apply_tracking_fallback_step(
        &mut self,
        pending: crate::tracking::PendingFallback,
        gravity_output_enabled: bool,
    ) -> Option<String> {
        let crate::tracking::PendingFallback { step, ramp, prev_confirmed, other_depths } = pending;
        match step {
            Ok(fb) => {
                self.commanded_positions = fb.positions.clone();
                self.target_positions = fb.positions.clone();
                self.commanded_velocities = fb.velocities.clone();
                self.commanded_accelerations.fill(0.0);
                let targets: Vec<JointTarget> = fb
                    .positions
                    .iter()
                    .zip(fb.velocities.iter())
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
                let write_res = self.write_output(targets, 1.0 / f64::from(self.config.control_frequency_hz.max(1)));
                match write_res {
                    Some(Ok(())) => {
                        let done = fb.done;
                        if done {
                            self.commanded_velocities.fill(0.0);
                        }
                        // Commit the step's escape depths to the workspace it was checked
                        // against; the other workspace's depths stay as they were.
                        let (full, park) = if ramp.keepout_only {
                            (other_depths, fb.ws_min_depths)
                        } else {
                            (fb.ws_min_depths, other_depths)
                        };
                        self.tracking.record_issued_with_ws_depths(fb.positions, fb.velocities, full, park);
                        self.tracking.fallback_ramp = (!done).then_some(ramp);
                        None
                    }
                    Some(Err(e)) => {
                        self.tracking.invalidate();
                        Some(e.to_string())
                    }
                    None => {
                        self.tracking.invalidate();
                        None
                    }
                }
            }
            Err(why) => {
                tracing::error!("Joint tracking fallback cannot brake within the limits ({why}): no frame sent");
                if let Some(pq) = prev_confirmed.as_deref().filter(|p| p.len() == self.commanded_positions.len()) {
                    let gripper_idx = self.config.gripper_index();
                    for (i, (c, &p)) in self.commanded_positions.iter_mut().zip(pq).enumerate() {
                        if Some(i) != gripper_idx {
                            *c = p;
                        }
                    }
                }
                self.target_positions = self.commanded_positions.clone();
                self.commanded_velocities.fill(0.0);
                self.commanded_accelerations.fill(0.0);
                self.tracking.invalidate();
                if !self.safety.state().is_stopped() {
                    self.safety.trigger_fault_coded(
                        FaultCode::ServoNoBrake,
                        FaultReason::Other(format!("Joint tracking fallback cannot brake within the limits: {why}")),
                    );
                }
                None
            }
        }
    }

    /// An already infeasible state cannot satisfy both acceleration and joint
    /// limits. Latch a fault and retain holding torque; normal smoothing does
    /// not apply to this exceptional safety interruption.
    pub(super) fn latch_servo_fault(&mut self, joint_idx: usize) {
        self.fail_live_policy_from_control("SERVO_INFEASIBLE",
            format!("Joint {} has an infeasible servo braking state", joint_idx + 1));
        let limit = &self.config.limits[joint_idx];
        let reason = FaultReason::Other(format!(
            "Joint '{}' has an invalid or infeasible servo braking state (q={}, v={}, limits=[{}, {}], vmax={}, amax={})",
            limit.name, self.commanded_positions[joint_idx], self.commanded_velocities[joint_idx],
            limit.min_position, limit.max_position, limit.max_velocity, limit.max_acceleration,
        ));
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        self.servo_ws_min_depths = None;
        self.servo_stream_active = false;
        self.tracking.invalidate();
        self.active_trajectory = None;
        self.commanded_velocities.fill(0.0);
        self.target_positions = self.commanded_positions.clone();
        self.safety.trigger_fault_coded(FaultCode::ServoNoBrake, reason);
    }
}

#[cfg(feature = "policy-runtime")]
impl SupervisorInner {
    /// Phase 1/3 real-arm gate: `Live` policy mode without release gate is only allowed for explicit mock or sim drivers.
    pub fn policy_live_allowed(&self) -> bool {
        matches!(
            (self.driver.is_some(), self.follower.as_deref()),
            (false, None) | (false, Some("mock")) | (_, Some("sim"))
        )
    }

    pub fn policy_run_active(&self) -> bool {
        self.policy_run.is_some()
    }

    pub fn is_policy_run_active(&self, action_id: u64) -> bool {
        self.policy_run
            .as_ref()
            .is_some_and(|r| r.action_id == action_id)
    }

    pub fn active_policy_release_gate(
        &self,
        action_id: u64,
    ) -> Option<&crate::policy_runtime::VerifiedReleaseGate> {
        self.policy_run
            .as_ref()
            .filter(|r| {
                r.action_id == action_id && r.mode == crate::policy_runtime::PolicyMode::Live
            })
            .and_then(|r| r.verified_release_gate.as_ref())
    }

    /// [`Self::policy_observation`] into `out`, reusing its memory.
    pub fn policy_observation_into(&self, out: &mut Option<crate::model::PolicyObservation>) {
        match &self.policy_run {
            Some(run) => run.to_observation_into(true, out.get_or_insert_with(Default::default)),
            None => crate::rt_core::scratch::set_opt(out, self.last_policy_obs.as_ref()),
        }
    }

    pub fn policy_observation(&self) -> Option<crate::model::PolicyObservation> {
        if let Some(ref run) = self.policy_run {
            Some(run.to_observation(true))
        } else {
            self.last_policy_obs.clone()
        }
    }

    pub fn policy_input_age_ms(&self) -> Option<u64> {
        self.policy_input_age_range_ms().map(|(_, max_ms)| max_ms)
    }

    pub fn policy_input_age_range_ms(&self) -> Option<(u64, u64)> {
        match &self.driver {
            None => Some((0, 0)),
            Some(d) => d.lock().ok().and_then(|drv| {
                let ages = drv.feedback_ages();
                if ages.len() != self.config.dof {
                    return None;
                }
                let durs = ages.into_iter().collect::<Option<Vec<_>>>()?;
                let mut min_ms = u64::MAX;
                let mut max_ms = 0u64;
                for dur in durs {
                    let ms = dur.as_millis().min(u64::MAX as u128) as u64;
                    min_ms = min_ms.min(ms);
                    max_ms = max_ms.max(ms);
                }
                if min_ms > max_ms {
                    return None;
                }
                Some((min_ms, max_ms))
            }),
        }
    }

    /// Preconditions checked under lock before loading a policy pack outside the lock.
    pub fn check_policy_start_preconditions(
        &self,
        mode: crate::policy_runtime::PolicyMode,
        epoch_id: u64,
    ) -> Result<(), ArmError> {
        self.check_policy_start_preconditions_with(mode, false, false, epoch_id)
    }

    /// Preconditions checked under lock with explicit `sim_only` and operator presence flags.
    pub fn check_policy_start_preconditions_with(
        &self,
        mode: crate::policy_runtime::PolicyMode,
        sim_only: bool,
        i_am_at_the_arm: bool,
        epoch_id: u64,
    ) -> Result<(), ArmError> {
        use crate::policy_runtime::PolicyMode;

        self.check_commissioning_for_ordinary_motion("policy/start")?;
        if self.estop_pending() {
            return Err(SafetyError::EStopActive.into());
        }
        if epoch_id != self.safety.epoch_id() {
            return Err(SafetyError::EpochMismatch {
                expected: self.safety.epoch_id(),
                received: epoch_id,
            }
            .into());
        }
        if self.handover_settling_since.is_some() {
            return Err(SafetyError::HandoverSettling.into());
        }
        if sim_only && !self.policy_live_allowed() {
            return Err(ArmError::Rejected {
                status: 403,
                code: "POLICY_SIM_ONLY".into(),
                message: "live_sim mode is not permitted on real or unknown hardware drivers".into(),
            });
        }
        match self.safety.state() {
            SafetyState::EStop => return Err(SafetyError::EStopActive.into()),
            SafetyState::Fault(r) => return Err(SafetyError::FaultActive(r.clone()).into()),
            s if mode == PolicyMode::Live && !s.is_operational() => {
                return Err(SafetyError::NotArmed(s.clone()).into());
            }
            _ => {}
        }
        if mode == PolicyMode::Live {
            if let Some(reason) = self.mit_motion_blocker() {
                return Err(MotionError::MotionBusy(reason).into());
            }
            if !self.policy_live_allowed() && !i_am_at_the_arm {
                return Err(ArmError::Rejected {
                    status: 403,
                    code: "OPERATOR_CONFIRMATION_REQUIRED".into(),
                    message: "Real-arm live policy execution requires explicit i_am_at_the_arm confirmation".into(),
                });
            }
            self.gravity_comp_servo_busy().map_err(ArmError::Safety)?;
            self.collision_hold_check().map_err(ArmError::Safety)?;
            if self.pending_rearm || self.pending_normal_unload || !self.is_torqued {
                return Err(SafetyError::NotArmed(self.safety.state().clone()).into());
            }
            if self.servo_blocked_hold {
                return Err(ArmError::Rejected {
                    status: 409,
                    code: "BLOCKED_HOLD".into(),
                    message: "Cannot start live policy while servo blocked hold is active".into(),
                });
            }
            if self.commanded_velocities.iter().any(|v| v.abs() >= 1e-3) {
                return Err(ArmError::Safety(SafetyError::MotionBusy));
            }
        }
        if self.calibration_busy
            || self.zero_check_persisting.is_some()
            || self.gravity_comp.is_some()
            || self.pending_normal_unload
            || self.active_trajectory.is_some()
            || self.macro_run.is_some()
            || self.policy_loading
            || self.policy_run.is_some()
            || self.servo_target.is_some()
        {
            return Err(ArmError::Safety(SafetyError::MotionBusy));
        }
        Ok(())
    }

    /// Install an exclusive streaming `PolicyRun` after lease, epoch, driver gate, and mutual exclusion checks.
    pub fn begin_policy_run(
        &mut self,
        req: crate::policy_runtime::PolicyRunStart,
    ) -> Result<u64, ArmError> {
        use crate::policy_runtime::{PolicyMode, PolicyRun, DEFAULT_MAX_FIRST_STEP_RAD};

        self.check_policy_start_preconditions_with(
            req.mode,
            req.sim_only,
            req.i_am_at_the_arm,
            req.epoch_id,
        )?;
        if req.mode == PolicyMode::Live && !self.policy_live_allowed() {
            if !req.i_am_at_the_arm {
                return Err(ArmError::Rejected {
                    status: 403,
                    code: "OPERATOR_CONFIRMATION_REQUIRED".into(),
                    message: "Real-arm live policy execution requires explicit i_am_at_the_arm confirmation".into(),
                });
            }
            let Some(ref gate) = req.verified_release_gate else {
                return Err(ArmError::Rejected {
                    status: 403,
                    code: "RELEASE_GATE_REQUIRED".into(),
                    message: "Real-arm live policy execution requires a verified rebot-release-gate/v1 bound to the policy artifact digest".into(),
                });
            };
            if req.artifact_digest.is_empty()
                || !gate.policy_sha256().eq_ignore_ascii_case(&req.artifact_digest)
            {
                return Err(ArmError::Rejected {
                    status: 403,
                    code: "RELEASE_GATE_REQUIRED".into(),
                    message: "Real-arm live policy execution requires a verified rebot-release-gate/v1 bound to the policy artifact digest".into(),
                });
            }
            let active_calib = self.active_calibration_digest();
            if !gate.calibration_digest().eq_ignore_ascii_case(&active_calib) {
                return Err(crate::policy_runtime::PolicyError::ReleaseGate(format!(
                    "calibration_digest mismatch: gate declares {}, active calibration is {active_calib}",
                    gate.calibration_digest()
                ))
                .into());
            }
            if !self.feedback_numeric_valid
                || !self.driver_feedback_confirmed()
                || self.policy_input_age_range_ms().is_none()
            {
                return Err(ArmError::Rejected {
                    status: 409,
                    code: "FEEDBACK_UNCONFIRMED".into(),
                    message: "Real-arm live policy execution requires confirmed, valid hardware feedback".into(),
                });
            }
            if gate.authorization_tier() == crate::policy_runtime::AuthorizationTier::ProductionRelease {
                use crate::policy_runtime::qualification;
                let Some(path) = self.policy_ledger_path.as_deref() else {
                    return Err(ArmError::Rejected {
                        status: 403,
                        code: "QUALIFICATION_REQUIRED".into(),
                        message: "production_release needs the device qualification ledger, and none is configured".into(),
                    });
                };
                let records = qualification::load_records(path)?;
                qualification::check_production_qualified(&records, gate).map_err(|e| ArmError::Rejected {
                    status: 403,
                    code: "QUALIFICATION_REQUIRED".into(),
                    message: e.to_string(),
                })?;
            }
        }
        if !self.validate_servo_lease(&req.connection_id, req.lease_generation) {
            return Err(ArmError::Rejected {
                status: 409,
                code: "LEASE_LOST".into(),
                message: "Connection does not own an active lease".into(),
            });
        }

        let max_first_step_rad = req
            .max_first_step_rad
            .filter(|v| v.is_finite() && *v > 0.0)
            .map_or(DEFAULT_MAX_FIRST_STEP_RAD, |v| {
                v.min(DEFAULT_MAX_FIRST_STEP_RAD)
            });

        let action_id = self.begin_action("policy", req.tag);
        let run = PolicyRun {
            action_id,
            epoch_id: req.epoch_id,
            connection_id: req.connection_id,
            lease_generation: req.lease_generation,
            policy_id: req.policy_id,
            artifact_digest: req.artifact_digest,
            backend: req.backend,
            mode: req.mode,
            steps: 0,
            last_input_age_ms: None,
            last_vision_age_ms: None,
            vision_frame_id: None,
            last_shadow_target: None,
            last_live_target: None,
            max_first_step_rad,
            max_step_rad: req.max_step_rad,
            i_am_at_the_arm: req.i_am_at_the_arm,
            verified_release_gate: req.verified_release_gate,
            stop_reason: None,
            max_input_age_ms: crate::policy_runtime::max_policy_input_age_ms(crate::policy_runtime::MIN_LIVE_POLICY_FPS),
            step_stats: crate::policy_runtime::PolicyStepStats::default(),
            authorization: crate::policy_runtime::PolicyAuthorization::default(),
            max_vision_age_ms: None,
            vision_stale_hold: false,
        };
        self.last_policy_obs = Some(run.to_observation(true));
        self.set_action_policy_stats(action_id, run.step_stats.report());
        self.policy_run = Some(run);
        Ok(action_id)
    }

    /// Submit one policy inference target frame.
    ///
    /// Enforces strict identity/epoch/lease checks, never renews `last_heartbeat`, never clears
    /// `servo_blocked_hold`, enforces first-step and per-step jump caps in `Live` mode, and
    /// shares `validate_and_filter_servo_targets` with `servo()`.
    pub fn policy_step(
        &mut self,
        action_id: u64,
        targets: Vec<f32>,
        input_age_ms: Option<u64>,
    ) -> Result<(), ArmError> {
        self.policy_step_with_vision(action_id, targets, input_age_ms, None, None)
    }

    pub fn policy_step_with_vision(
        &mut self,
        action_id: u64,
        mut targets: Vec<f32>,
        input_age_ms: Option<u64>,
        vision_age_ms: Option<u64>,
        vision_frame_id: Option<u64>,
    ) -> Result<(), ArmError> {
        use crate::policy_runtime::PolicyMode;

        let (epoch_id, conn_id, lease_gen, mode, max_first, max_step, prev_live, max_input_age_ms) = {
            let Some(active) = self.policy_run.as_ref() else {
                return Err(ArmError::Rejected {
                    status: 409,
                    code: "POLICY_NOT_ACTIVE".into(),
                    message: "No active policy run".into(),
                });
            };
            if active.action_id != action_id {
                return Err(ArmError::Rejected {
                    status: 409,
                    code: "POLICY_NOT_ACTIVE".into(),
                    message: "Stale policy action_id".into(),
                });
            }
            (
                active.epoch_id,
                active.connection_id.clone(),
                active.lease_generation,
                active.mode,
                active.max_first_step_rad,
                active.max_step_rad,
                active.last_live_target.clone(),
                active.max_input_age_ms,
            )
        };

        if self.estop_pending() || matches!(self.safety.state(), SafetyState::EStop) {
            self.terminate_policy_run("ESTOP", "failed", Some("ESTOP_ACTIVE"), Some("E-stop latched".into()));
            return Err(SafetyError::EStopActive.into());
        }
        if epoch_id != self.safety.epoch_id() {
            let current = self.safety.epoch_id();
            self.terminate_policy_run(
                "STOPPED",
                "cancelled",
                Some("EPOCH_MISMATCH"),
                Some("Epoch changed during policy run".into()),
            );
            return Err(SafetyError::EpochMismatch {
                expected: current,
                received: epoch_id,
            }
            .into());
        }
        if let SafetyState::Fault(r) = self.safety.state().clone() {
            let reason = if matches!(r, FaultReason::WatchdogTimeout) {
                "WATCHDOG_TIMEOUT"
            } else {
                "FAULT"
            };
            self.terminate_policy_run(
                reason,
                "failed",
                Some(reason),
                Some(format!("Fault active: {r:?}")),
            );
            return Err(SafetyError::FaultActive(r).into());
        }

        if !self.validate_servo_lease(&conn_id, lease_gen) {
            self.terminate_policy_run(
                "LEASE_LOST",
                "cancelled",
                Some("LEASE_LOST"),
                Some("Control lease lost or expired".into()),
            );
            return Err(ArmError::Rejected {
                status: 409,
                code: "LEASE_LOST".into(),
                message: "Control lease lost or expired".into(),
            });
        }

        if self.servo_blocked_hold {
            self.terminate_policy_run(
                "BLOCKED_HOLD",
                "cancelled",
                Some("BLOCKED_HOLD"),
                Some("Servo blocked hold active".into()),
            );
            return Err(ArmError::Rejected {
                status: 409,
                code: "BLOCKED_HOLD".into(),
                message: "Servo blocked hold active".into(),
            });
        }

        if targets.len() != self.config.dof || targets.iter().any(|v| !v.is_finite()) {
            self.terminate_policy_run(
                "INVALID_TARGET",
                "failed",
                Some("INVALID_TARGET"),
                Some("Policy produced non-finite or wrong-dimension target".into()),
            );
            return Err(SafetyError::InvalidValue.into());
        }

        if mode == PolicyMode::Shadow {
            if let Some(run) = self.policy_run.as_mut().filter(|r| r.action_id == action_id) {
                run.steps += 1;
                run.last_input_age_ms = input_age_ms;
                run.last_vision_age_ms = vision_age_ms;
                run.vision_frame_id = vision_frame_id;
                run.last_shadow_target = Some(targets);
                self.last_policy_obs = Some(run.to_observation(true));
            }
            return Ok(());
        }

        // Live mode checks: release-gate calibration binding, real-arm gate, stale joint input
        // (max_policy_input_age_ms), jump limits, and shared servo validation.
        if let Some(gate) = self
            .policy_run
            .as_ref()
            .filter(|r| r.action_id == action_id)
            .and_then(|r| r.verified_release_gate.as_ref())
        {
            let active_calib = self.active_calibration_digest();
            if !gate.calibration_digest().eq_ignore_ascii_case(&active_calib) {
                let msg = format!(
                    "calibration_digest mismatch: gate declares {}, active deployment is {active_calib}",
                    gate.calibration_digest()
                );
                self.terminate_policy_run(
                    "DEPLOYMENT_CHANGED",
                    "failed",
                    Some("DEPLOYMENT_CHANGED"),
                    Some(msg.clone()),
                );
                return Err(ArmError::Rejected {
                    status: 409,
                    code: "DEPLOYMENT_CHANGED".into(),
                    message: msg,
                });
            }
        }
        if !self.policy_live_allowed() {
            let (has_confirm, has_gate) = self.policy_run.as_ref().map_or((false, false), |run| {
                (
                    run.i_am_at_the_arm,
                    run.verified_release_gate.as_ref().is_some_and(|g| {
                        !run.artifact_digest.is_empty()
                            && g.policy_sha256().eq_ignore_ascii_case(&run.artifact_digest)
                    }),
                )
            });
            if !has_confirm {
                self.terminate_policy_run(
                    "OPERATOR_CONFIRMATION_REQUIRED",
                    "failed",
                    Some("OPERATOR_CONFIRMATION_REQUIRED"),
                    Some("Real-arm live policy execution requires operator confirmation".into()),
                );
                return Err(ArmError::Rejected {
                    status: 403,
                    code: "OPERATOR_CONFIRMATION_REQUIRED".into(),
                    message: "Real-arm live policy execution requires operator confirmation".into(),
                });
            }
            if !has_gate {
                self.terminate_policy_run(
                    "RELEASE_GATE_REQUIRED",
                    "failed",
                    Some("RELEASE_GATE_REQUIRED"),
                    Some("Real-arm live policy execution requires a verified release gate".into()),
                );
                return Err(ArmError::Rejected {
                    status: 403,
                    code: "RELEASE_GATE_REQUIRED".into(),
                    message: "Real-arm live policy execution requires a verified release gate".into(),
                });
            }
            if !self.feedback_numeric_valid
                || !self.driver_feedback_confirmed()
                || self.policy_input_age_range_ms().is_none()
            {
                self.terminate_policy_run(
                    "FEEDBACK_UNCONFIRMED",
                    "failed",
                    Some("FEEDBACK_UNCONFIRMED"),
                    Some("Hardware feedback is unconfirmed during live policy step".into()),
                );
                return Err(ArmError::Rejected {
                    status: 409,
                    code: "FEEDBACK_UNCONFIRMED".into(),
                    message: "Hardware feedback is unconfirmed during live policy step".into(),
                });
            }
        }

        // Capture-to-commit age of the joint positions this target was computed from: a
        // target from an old pose (slow inference while the arm moved) never reaches the servo.
        let stale = match input_age_ms {
            Some(age) => (age > max_input_age_ms).then(|| format!("{age} ms > {max_input_age_ms} ms")),
            None if !self.policy_live_allowed() => Some("input age unknown".to_string()),
            None => None,
        };
        if let Some(why) = stale {
            let msg = format!("Live policy target computed from stale joint positions ({why})");
            self.terminate_policy_run("STALE_INPUT", "failed", Some("STALE_INPUT"), Some(msg.clone()));
            return Err(ArmError::Rejected { status: 409, code: "STALE_INPUT".into(), message: msg });
        }

        if prev_live.is_none() {
            if self.measured_positions.len() != self.config.dof
                || self.measured_positions.iter().any(|m| !m.is_finite())
            {
                self.terminate_policy_run(
                    "INVALID_FEEDBACK",
                    "failed",
                    Some("INVALID_FEEDBACK"),
                    Some("Measured joint positions are invalid on first live policy step".into()),
                );
                return Err(SafetyError::FeedbackUnavailable.into());
            }
            for i in 0..self.config.dof {
                let delta = (targets[i] - self.measured_positions[i]).abs();
                if delta > max_first {
                    let msg = format!(
                        "First live policy step on joint {} exceeds max_first_step_rad ({delta:.4} > {max_first:.4})",
                        i + 1
                    );
                    self.terminate_policy_run("JUMP_LIMIT", "failed", Some("JUMP_LIMIT"), Some(msg.clone()));
                    return Err(ArmError::Rejected {
                        status: 422,
                        code: "JUMP_LIMIT".into(),
                        message: msg,
                    });
                }
            }
        } else if let (Some(prev), Some(step_cap)) = (prev_live.as_ref(), max_step) {
            for i in 0..self.config.dof {
                let delta = (targets[i] - prev[i]).abs();
                if delta > step_cap {
                    let msg = format!(
                        "Live policy step on joint {} exceeds max_step_rad ({delta:.4} > {step_cap:.4})",
                        i + 1
                    );
                    self.terminate_policy_run("STEP_LIMIT", "failed", Some("STEP_LIMIT"), Some(msg.clone()));
                    return Err(ArmError::Rejected {
                        status: 422,
                        code: "STEP_LIMIT".into(),
                        message: msg,
                    });
                }
            }
        }

        if let Err(e) = self.validate_and_filter_servo_targets(&mut targets, epoch_id) {
            let (reason, state, code) = match &e {
                SafetyError::EStopActive => ("ESTOP", "failed", "ESTOP_ACTIVE"),
                SafetyError::FaultActive(_) => ("FAULT", "failed", "FAULT_ACTIVE"),
                SafetyError::CollisionHold(_) => ("COLLISION_HOLD", "cancelled", "COLLISION_HOLD"),
                SafetyError::SoftLimitExceeded { .. } => ("SOFT_LIMIT_EXCEEDED", "failed", "SOFT_LIMIT_EXCEEDED"),
                SafetyError::WorkspaceViolation { .. } => ("WORKSPACE_VIOLATION", "failed", "WORKSPACE_VIOLATION"),
                SafetyError::NotArmed(_) => ("NOT_ARMED", "failed", "NOT_ARMED"),
                _ => ("SAFETY_REJECTED", "failed", "SAFETY_ERROR"),
            };
            self.terminate_policy_run(reason, state, Some(code), Some(e.to_string()));
            return Err(ArmError::Safety(e));
        }

        // CRITICAL 1: NEVER touch self.last_heartbeat here!
        // CRITICAL 2: NEVER clear self.servo_blocked_hold here!
        self.last_motion_frame = Instant::now();
        self.release_gripper_stall_on_reverse(&targets);
        let _ = self.safety.mark_motion_started();

        self.target_positions = targets.clone();
        self.servo_target = Some(targets.clone());
        self.servo_stream_active = true;
        self.servo_settled_ticks.fill(0);

        if let Some(run) = self.policy_run.as_mut().filter(|r| r.action_id == action_id) {
            run.steps += 1;
            run.last_input_age_ms = input_age_ms;
            run.last_vision_age_ms = vision_age_ms;
            run.vision_frame_id = vision_frame_id;
            run.last_shadow_target = Some(targets.clone());
            run.last_live_target = Some(targets);
            self.last_policy_obs = Some(run.to_observation(true));
        }

        Ok(())
    }

    /// Vision hold of a live run: drop its servo target so the control loop brakes now
    /// (instead of tracking the last target until the motion-frame timeout). The next target
    /// is checked against the measured pose with the first-step limit.
    pub fn hold_policy_run(&mut self, action_id: u64) {
        let Some(run) = self.policy_run.as_mut().filter(|r| r.action_id == action_id && r.mode == crate::policy_runtime::PolicyMode::Live) else { return; };
        // a target computed before the hold and still waiting in the mailbox is not applied
        run.authorization.revoke();
        run.last_live_target = None;
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        self.servo_ws_min_depths = None;
        self.target_positions = self.commanded_positions.clone();
    }

    /// Set the run's [`crate::policy_runtime::max_policy_input_age_ms`] bound.
    pub fn set_policy_max_input_age(&mut self, action_id: u64, max_ms: u64) {
        if let Some(run) = self.policy_run.as_mut().filter(|r| r.action_id == action_id) {
            run.max_input_age_ms = max_ms;
        }
    }

    /// Set the camera-sample age bound the RT applies when it consumes a target of the run
    /// (the vision spec's staleness), and whether an older target holds the run.
    pub fn set_policy_max_vision_age(&mut self, action_id: u64, max_ms: Option<u64>, hold_on_stale: bool) {
        if let Some(run) = self.policy_run.as_mut().filter(|r| r.action_id == action_id) {
            run.max_vision_age_ms = max_ms;
            run.vision_stale_hold = hold_on_stale;
        }
    }

    /// Set the run's period (`policy_stats.period_us`).
    pub fn set_policy_period_us(&mut self, action_id: u64, period_us: u64) {
        let Some(run) = self.policy_run.as_mut().filter(|r| r.action_id == action_id) else { return; };
        run.step_stats.set_period_us(period_us);
        let stats = run.step_stats.report();
        self.set_action_policy_stats(action_id, stats);
    }

    /// Record one inference of the run (see [`crate::policy_runtime::PolicyStepStats::record`])
    /// and publish `policy_stats` in the action status.
    pub fn record_policy_step_timing(&mut self, action_id: u64, infer_us: u64, step_us: u64) {
        let Some(run) = self.policy_run.as_mut().filter(|r| r.action_id == action_id) else { return; };
        run.step_stats.record(infer_us, step_us);
        let stats = run.step_stats.report();
        self.set_action_policy_stats(action_id, stats);
    }

    pub(super) fn set_action_policy_stats(&mut self, action_id: u64, stats: crate::motion::PolicyStats) {
        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == action_id) {
            a.policy_stats = Some(stats);
        }
    }

    pub fn stop_policy_run(&mut self, action_id: Option<u64>, reason: &str) -> bool {
        let matches = match (self.policy_run.as_ref(), action_id) {
            (Some(_), None) => true,
            (Some(r), Some(id)) => r.action_id == id,
            (None, _) => false,
        };
        if !matches {
            return false;
        }
        self.terminate_policy_run(
            reason,
            "cancelled",
            Some(reason),
            Some(format!("Policy stopped: {reason}")),
        );
        true
    }

    pub fn finish_policy_run(&mut self, action_id: u64, reason: &str) -> bool {
        if !self.is_policy_run_active(action_id) {
            return false;
        }
        // a fault or E-stop that the runner has not seen yet is not a normal completion
        if let SafetyState::Fault(r) = self.safety.state().clone() {
            self.terminate_policy_run("FAULT", "failed", Some("FAULT"), Some(format!("Fault active: {r:?}")));
            return true;
        }
        if self.estop_pending() || matches!(self.safety.state(), SafetyState::EStop) {
            self.terminate_policy_run("ESTOP", "failed", Some("ESTOP_ACTIVE"), Some("E-stop latched".into()));
            return true;
        }
        self.terminate_policy_run(reason, "done", None, None);
        true
    }

    pub fn fail_policy_run(
        &mut self,
        action_id: u64,
        reason: &str,
        message: impl Into<String>,
    ) -> bool {
        if !self.is_policy_run_active(action_id) {
            return false;
        }
        self.terminate_policy_run(reason, "failed", Some(reason), Some(message.into()));
        true
    }

    /// Append the outcome of a supervised_first_run live run on real hardware to the device
    /// qualification ledger. Passed = ran to its deadline (`MAX_SECONDS`, action done).
    pub(super) fn record_policy_qualification(&self, run: &crate::policy_runtime::PolicyRun, stop_reason: &str, action_state: &str) {
        use crate::policy_runtime::{qualification, AuthorizationTier, PolicyMode};
        if run.mode != PolicyMode::Live || self.policy_live_allowed() { return; }
        let Some(gate) = run.verified_release_gate.as_ref().filter(|g| g.authorization_tier() == AuthorizationTier::SupervisedFirstRun) else { return; };
        let Some(path) = self.policy_ledger_path.as_deref() else {
            tracing::warn!("No policy qualification ledger configured; supervised run {} not recorded", run.action_id);
            return;
        };
        let passed = stop_reason == "MAX_SECONDS" && action_state == "done";
        let record = qualification::QualificationRecord::for_run(gate, run.action_id, run.steps, stop_reason, passed)
            .with_policy_stats(run.step_stats.report());
        if let Err(e) = qualification::append_record(path, &record) {
            tracing::warn!("Cannot append policy qualification record to {}: {e}", path.display());
        }
    }

    pub(super) fn terminate_policy_run(
        &mut self,
        stop_reason: &str,
        action_state: &str,
        action_code: Option<&str>,
        action_message: Option<String>,
    ) {
        let Some(mut run) = self.policy_run.take() else {
            return;
        };
        run.stop_reason = Some(stop_reason.to_string());
        self.set_action_policy_stats(run.action_id, run.step_stats.report());
        self.record_policy_qualification(&run, stop_reason, action_state);
        if run.mode == crate::policy_runtime::PolicyMode::Live {
            self.servo_target = None;
            self.servo_settled_ticks.fill(0);
            self.servo_ws_min_depths = None;
            self.target_positions = self.commanded_positions.clone();
        }
        self.last_policy_obs = Some(run.to_observation(false));
        self.finish_action(
            run.action_id,
            action_state,
            action_code,
            action_message,
            None,
            None,
            None,
        );
    }
}
