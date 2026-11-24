//! Servo streams and the policy runner of the supervisor: servo target validation, bounded servo steps, tracking fallback, servo holds and faults, policy start/step/stop (track D).
//!
//! Moved from `supervisor.rs` without changes (armd 1.6 RT-core foundation); private
//! methods are `pub(super)` so the rest of the supervisor keeps calling them.

#[allow(unused_imports)]
use super::*;

/// Copies `src` into `dst`, reusing its memory when it holds a vector.
fn set_vec(dst: &mut Option<Vec<f32>>, src: &[f32]) {
    match dst {
        Some(v) => copy_into(v, src),
        None => *dst = Some(src.to_vec()),
    }
}

impl SupervisorInner {
    /// Clears the servo target, keeping its buffer for the next one (no free).
    pub(crate) fn release_servo_target(&mut self) {
        if let Some(v) = self.servo_target.take() {
            // the spare is empty (capacity 0) while a target holds the buffer: nothing is freed
            if v.capacity() >= self.servo_target_spare.capacity() { self.servo_target_spare = v; }
        }
    }

    /// Sets the servo target, in the existing or the spare buffer (no allocation while one
    /// has the capacity).
    fn set_servo_target(&mut self, src: &[f32]) {
        if self.servo_target.is_none() {
            let mut v = std::mem::take(&mut self.servo_target_spare);
            copy_into(&mut v, src);
            self.servo_target = Some(v);
        } else {
            set_vec(&mut self.servo_target, src);
        }
    }
}

/// The run's observation into `dst`, reusing its memory.
#[cfg(feature = "policy-runtime")]
fn set_policy_obs(dst: &mut Option<crate::model::PolicyObservation>, run: &crate::policy_runtime::PolicyRun) {
    match dst {
        Some(o) => run.to_observation_into(true, o),
        None => *dst = Some(run.to_observation(true)),
    }
}

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
    /// Pre-warms `ws_eval_bufs` before `armd-control` enters `SCHED_FIFO` so the first
    /// braking tick after an RT urgent stop performs zero heap allocations.
    pub fn prewarm_rt_buffers(&mut self) {
        let mut bufs = std::mem::take(&mut self.ws_eval_bufs);
        if let Some((ws, cam)) = self.workspace_policy() {
            let eval = crate::chain::WorkspaceEvaluator::new_with_tcp_in(
                &self.config.arm_model,
                ws,
                cam,
                self.config.tcp_offset,
                &mut bufs,
            );
            eval.recycle(&mut bufs);
        }
        self.ws_eval_bufs = bufs;
    }

    pub fn servo_step_workspace(&mut self, next: &[f32], phase: &str) -> Result<(), SafetyError> {
        // the evaluator's vectors are reused from tick to tick (no allocation in the tick)
        let mut bufs = std::mem::take(&mut self.ws_eval_bufs);
        let result = self.servo_step_workspace_in(next, phase, &mut bufs);
        self.ws_eval_bufs = bufs;
        let cand_min_d = result?;
        if let Some(min_d) = cand_min_d { self.servo_ws_min_depths = Some((self.workspace_revision, min_d)); }
        Ok(())
    }

    /// `check_workspace_path(commanded, [target])` with the evaluator's reused vectors (no
    /// allocation): the servo / policy target check.
    pub(super) fn servo_target_workspace(&mut self, target: &[f32]) -> Result<(), SafetyError> {
        let mut bufs = std::mem::take(&mut self.ws_eval_bufs);
        let result = (|| {
            let Some((ws, cam)) = self.workspace_policy() else { return Ok(()); };
            let eval = crate::chain::WorkspaceEvaluator::new_with_tcp_in(&self.config.arm_model, ws, cam, self.config.tcp_offset, &mut bufs)
                .with_escape_tol(crate::kinematics::WORKSPACE_EPS);
            let checked = (|| {
                let mut c0 = eval.alloc_buf();
                eval.eval_into(&self.commanded_positions, &mut c0, "plan").map_err(SafetyError::from_workspace)?;
                let mut min_depths = eval.init_min_depths(&c0);
                let mut clr = eval.alloc_buf();
                eval.eval_into(target, &mut clr, "plan").map_err(SafetyError::from_workspace)?;
                eval.check_and_advance(&clr, &mut min_depths, "plan").map_err(SafetyError::from_workspace)
            })();
            eval.recycle(&mut bufs);
            checked
        })();
        self.ws_eval_bufs = bufs;
        result
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
        self.validate_lease_at(client_id, generation, Instant::now())
    }

    /// Read-only owner-serialized lease check at `now` (zero mutation, zero allocs) used by
    /// mid-transaction authority re-checks such as M3 `clear_fault` before hardware reset and
    /// before final safety clear.
    pub(crate) fn is_lease_valid_for_conn_at(
        &self,
        client_id: &str,
        generation: u64,
        now: Instant,
    ) -> bool {
        let fenced_gen = self.effective_fenced_lease_generation();
        if self.detached
            || generation == 0
            || generation <= fenced_gen
            || client_id.is_empty()
            || (self.active_lease_conn_id != 0
                && self.is_connection_closed(self.active_lease_conn_id))
        {
            return false;
        }
        self.active_lease.as_ref().is_some_and(|active| {
            now.saturating_duration_since(active.acquired_at) < self.lease_timeout
                && active.generation == generation
                && active.client_id == client_id
        })
    }

    /// Owner-serialized lease validation at `now`: verifies non-detached state, non-zero
    /// generation, matching connection identity, and non-expiry at `now`, updating
    /// `last_valid_motion_instant` on success. Zero allocations and zero frees.
    pub fn validate_lease_at(&mut self, client_id: &str, generation: u64, now: Instant) -> bool {
        if !self.is_lease_valid_for_conn_at(client_id, generation, now) {
            return false;
        }
        if let Some(ref mut active) = self.active_lease {
            active.last_valid_motion_instant = Some(now);
            true
        } else {
            false
        }
    }

    /// [`Self::validate_servo_lease`] for the connection of policy run `action_id`, without
    /// copying its connection id.
    #[cfg(feature = "policy-runtime")]
    fn validate_policy_lease(&mut self, action_id: u64, generation: u64) -> bool {
        let timeout = self.lease_timeout;
        let Some(conn) = self.policy_run.as_ref().filter(|r| r.action_id == action_id).map(|r| r.connection_id.as_str()) else { return false; };
        match self.active_lease.as_mut() {
            Some(active) if active.acquired_at.elapsed() < timeout && active.generation == generation && !conn.is_empty() && active.client_id == conn => {
                active.last_valid_motion_instant = Some(Instant::now());
                true
            }
            _ => false,
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

        if self.shutdown_committed || self.estop_pending() {
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
        if let Err(e) = self.servo_target_workspace(targets) {
            let e = e.with_workspace_phase("servo_target");
            self.workspace_violation = e.workspace_detail();
            self.note_motion_limit(MotionLimitReason::Workspace, None);
            return Err(e);
        }
        self.torque_hold_filter(targets);
        self.power_hold_filter(targets);
        if self.is_pos_force_j7_stale() {
            if let Some(g) = self.config.gripper_index() {
                if targets.len() > g {
                    if let Some(freeze_pos) = self.pos_force_j7_freeze_position() {
                        targets[g] = freeze_pos;
                    } else if g < self.commanded_positions.len() {
                        targets[g] = self.commanded_positions[g];
                    }
                }
            }
        }
        Ok(())
    }

    /// Servo / policy intake of the gripper target: records it as the stream's gripper target and
    /// releases the stall latch on a target away from the stall. A torque-cap latch needs the
    /// target [`GRIPPER_FORCE_RELEASE_MARGIN_RAD`] more open than the hold.
    pub(super) fn release_gripper_stall_on_reverse(&mut self, targets: &[f32]) {
        if self.is_pos_force_j7_stale() {
            self.gripper_stream_target = None;
            return;
        }
        if let Some(g_idx) = self.config.gripper_index() {
            self.gripper_stream_target = targets.get(g_idx).copied().filter(|t| t.is_finite());
            self.gripper_stream_lease = self.active_lease.as_ref().map(|l| l.generation);
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
                    self.gripper_stall_window.reset();
                }
            }
        }
    }

    pub fn servo(admitted: Admitted<'_>, targets: Vec<f32>, epoch_id: u64) -> Result<(), SafetyError> {
        Self::servo_slice(admitted, &targets, epoch_id)
    }

    /// [`Self::servo`] from a slice: no allocation once the servo buffers hold a target
    /// (the latest-target path consumes through it).
    pub fn servo_slice(admitted: Admitted<'_>, targets_in: &[f32], epoch_id: u64) -> Result<(), SafetyError> {
        let (this, _proof) = admitted.consume()?;
        this.servo_slice_inner(targets_in, epoch_id)
    }

    fn servo_slice_inner(&mut self, targets_in: &[f32], epoch_id: u64) -> Result<(), SafetyError> {
        if targets_in.len() > crate::rt_core::mailbox::MAX_DOF { return self.servo_slice_oversized(targets_in.to_vec(), epoch_id); }
        let mut buf = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
        let targets = &mut buf[..targets_in.len()];
        targets.copy_from_slice(targets_in);
        #[cfg(feature = "policy-runtime")]
        if self.policy_run.is_some() || self.policy_loading {
            return Err(SafetyError::MotionBusy);
        }
        if self.mit_motion_blocked() { return Err(SafetyError::MotionBusy); }
        self.validate_and_filter_servo_targets(targets, epoch_id)?;
        if self.servo_blocked_hold { self.clear_blocked_hold(); }

        // Only renew lease and motion frame timestamp AFTER command validation succeeds
        self.last_heartbeat = Instant::now();
        self.last_motion_frame = Instant::now();
        if let Some(ref mut l) = self.active_lease {
            l.last_valid_motion_instant = Some(Instant::now());
        }

        // Gripper stall release check on reverse command
        self.release_gripper_stall_on_reverse(targets);

        let _ = self.safety.mark_motion_started();

        // 2. Store servo target for tick(dt) to step with actual dt and velocity limits
        copy_into(&mut self.target_positions, targets);
        self.set_servo_target(targets);
        self.servo_stream_active = true;
        self.servo_settled_ticks.fill(0);

        Ok(())
    }

    /// More joints than a target frame carries: the same checks refuse it (dimension).
    fn servo_slice_oversized(&mut self, mut targets: Vec<f32>, epoch_id: u64) -> Result<(), SafetyError> {
        #[cfg(feature = "policy-runtime")]
        if self.policy_run.is_some() || self.policy_loading {
            return Err(SafetyError::MotionBusy);
        }
        if self.mit_motion_blocked() { return Err(SafetyError::MotionBusy); }
        self.validate_and_filter_servo_targets(&mut targets, epoch_id)?;
        Err(SafetyError::DimensionMismatch { expected: self.config.dof, received: targets.len() })
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
    /// Returns the classified [`crate::supervisor::TickOutput`].
    pub(super) fn apply_tracking_fallback_step(
        &mut self,
        pending: crate::tracking::PendingFallback,
        gravity_output_enabled: bool,
    ) -> crate::supervisor::TickOutput {
        use crate::supervisor::TickOutput;
        let crate::tracking::PendingFallback { step, ramp, other_depths } = pending;
        match step {
            Ok(fb) => {
                crate::rt_core::scratch::copy_into(&mut self.commanded_positions, &fb.positions);
                crate::rt_core::scratch::copy_into(&mut self.target_positions, &fb.positions);
                crate::rt_core::scratch::copy_into(&mut self.commanded_velocities, &fb.velocities);
                self.commanded_accelerations.fill(0.0);
                let mut targets = std::mem::take(&mut self.scratch.targets);
                targets.clear();
                targets.extend(
                    fb.positions
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
                        }),
                );
                self.scratch.targets = targets;
                let dt = 1.0 / f64::from(self.config.control_frequency_hz.max(1));
                let write_res = self.write_output_scratch_with_limits(dt, &ramp.limits);
                match &write_res {
                    TickOutput::Complete { .. } => {
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
                    }
                    TickOutput::Deferred { .. } => {
                        // Keep the in-progress braking ramp intact across transient deferral
                        // without advancing issued history or marking the ramp done.
                        self.tracking.fallback_ramp = Some(ramp);
                    }
                    TickOutput::Expired { .. }
                    | TickOutput::Error { .. }
                    | TickOutput::EstopInhibited { .. }
                    | TickOutput::ActionDeadlineRefused { .. }
                    | TickOutput::None => {
                        self.tracking.invalidate();
                    }
                }
                write_res
            }
            Err(why) => {
                tracing::error!("Joint tracking fallback cannot brake within the limits ({why}): no frame sent");
                if self.scratch.fallback_prev_confirmed_valid
                    && self.scratch.fallback_prev_confirmed.len() == self.commanded_positions.len()
                {
                    let gripper_idx = self.config.gripper_index();
                    for (i, (c, &p)) in self
                        .commanded_positions
                        .iter_mut()
                        .zip(self.scratch.fallback_prev_confirmed.iter())
                        .enumerate()
                    {
                        if Some(i) != gripper_idx {
                            *c = p;
                        }
                    }
                }
                crate::rt_core::scratch::copy_into(&mut self.target_positions, &self.commanded_positions);
                self.commanded_velocities.fill(0.0);
                self.commanded_accelerations.fill(0.0);
                self.tracking.invalidate();
                self.safety.latch(
                    crate::safety::FaultKind::Other,
                    crate::safety::FaultDetail::formatted(FaultCode::ServoNoBrake, |buf| {
                        let _ = core::fmt::write(buf, format_args!("Joint tracking fallback cannot brake within the limits: {why}"));
                    }),
                );
                TickOutput::None
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
        let q = self.commanded_positions[joint_idx];
        let v = self.commanded_velocities[joint_idx];
        let detail = crate::safety::FaultDetail::formatted(FaultCode::ServoNoBrake, |buf| {
            let _ = core::fmt::write(
                buf,
                format_args!(
                    "Joint '{}' has an invalid or infeasible servo braking state (q={q}, v={v}, limits=[{}, {}], vmax={}, amax={})",
                    limit.name, limit.min_position, limit.max_position, limit.max_velocity, limit.max_acceleration,
                ),
            );
        });
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        self.servo_ws_min_depths = None;
        self.servo_stream_active = false;
        self.tracking.invalidate();
        self.active_trajectory = None;
        self.commanded_velocities.fill(0.0);
        crate::rt_core::scratch::copy_into(&mut self.target_positions, &self.commanded_positions);
        self.safety.latch(crate::safety::FaultKind::Other, detail);
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

    /// Zero-allocation, non-blocking equivalent of `self.policy_input_age_range_ms().is_some()`
    /// safe to call on `armd-control`.
    pub(super) fn has_policy_input_age_range(&self) -> bool {
        if self.config.dof == 0 {
            return false;
        }
        match &self.driver {
            None => true,
            Some(d) => match d.try_lock() {
                Ok(drv) => drv.feedback_all_within(self.config.dof, Duration::MAX),
                Err(std::sync::TryLockError::Poisoned(p)) => {
                    p.into_inner().feedback_all_within(self.config.dof, Duration::MAX)
                }
                Err(std::sync::TryLockError::WouldBlock) => false,
            },
        }
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

    /// Install an exclusive streaming `PolicyRun` after stamp, lease, epoch, driver gate, and mutual exclusion checks.
    pub fn begin_policy_run(admitted: Admitted<'_>, req: crate::policy_runtime::PolicyRunStart) -> Result<u64, ArmError> {
        let (this, _proof) = admitted.consume()?;
        this.begin_policy_run_inner(req)
    }

    fn begin_policy_run_inner(&mut self, req: crate::policy_runtime::PolicyRunStart) -> Result<u64, ArmError> {
        use crate::policy_runtime::{PolicyMode, PolicyRun, DEFAULT_MAX_FIRST_STEP_RAD};

        self.check_policy_start_preconditions_with(
            req.mode,
            req.sim_only,
            req.i_am_at_the_arm,
            req.epoch_id,
        )?;
        if req.mode == PolicyMode::Live && self.config.gripper_index().is_some() {
            self.check_pos_force_gripper_fresh()?;
        }
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
                || !self.has_policy_input_age_range()
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

        self.check_terminal_fifo_for_new_action()?;
        if !self.can_admit_policy_run() {
            return Err(ArmError::Safety(SafetyError::MotionBusy));
        }
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
        // the run's targets are the ones consumed from the latest-target mailbox now
        self.core.authorize_stream(crate::rt_core::mailbox::TargetAuthority {
            stream: Some(targets::policy_stream(action_id)),
            policy_generation: 1,
            ..Default::default()
        });
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
        targets: Vec<f32>,
        input_age_ms: Option<u64>,
        vision_age_ms: Option<u64>,
        vision_frame_id: Option<u64>,
    ) -> Result<(), ArmError> {
        self.policy_step_slice(action_id, &targets, input_age_ms, vision_age_ms, vision_frame_id)
    }

    /// [`Self::policy_step_with_vision`] from a slice: no allocation on an accepted step once
    /// the run's buffers hold a target (the latest-target path consumes through it).
    ///
    /// Exempt from `Admitted` (Invariant 1b): continuation step of a policy run that was already
    /// admitted via `begin_policy_run(Admitted<'_>, ...)`; gated by the active run's
    /// `action_id`, authorization generation, epoch, lease, and E-stop/Stop termination.
    pub fn policy_step_slice(
        &mut self,
        action_id: u64,
        targets_in: &[f32],
        input_age_ms: Option<u64>,
        vision_age_ms: Option<u64>,
        vision_frame_id: Option<u64>,
    ) -> Result<(), ArmError> {
        use crate::policy_runtime::PolicyMode;

        let mut buf = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
        let n = targets_in.len().min(crate::rt_core::mailbox::MAX_DOF);
        buf[..n].copy_from_slice(&targets_in[..n]);
        let targets = &mut buf[..n];

        let (epoch_id, lease_gen, mode, max_first, max_step, has_prev_live, max_input_age_ms) = {
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
                active.lease_generation,
                active.mode,
                active.max_first_step_rad,
                active.max_step_rad,
                active.last_live_target.is_some(),
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
        if let SafetyState::Fault(r) = self.safety.state() {
            let r = r.clone();
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

        if !self.validate_policy_lease(action_id, lease_gen) {
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

        if targets_in.len() != self.config.dof || targets.iter().any(|v| !v.is_finite()) {
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
                set_vec(&mut run.last_shadow_target, targets);
                set_policy_obs(&mut self.last_policy_obs, run);
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
            if !gate.calibration_digest().eq_ignore_ascii_case(self.active_calibration_digest_str()) {
                let active_calib = self.active_calibration_digest();
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
                || !self.has_policy_input_age_range()
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

        if !has_prev_live {
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
        } else if let Some(step_cap) = max_step {
            let dof = self.config.dof;
            let violation = self.policy_run.as_ref().and_then(|r| r.last_live_target.as_ref()).and_then(|prev| {
                (0..dof).find_map(|i| { let delta = (targets[i] - prev[i]).abs(); (delta > step_cap).then_some((i, delta)) })
            });
            if let Some((i, delta)) = violation {
                {
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

        if self.config.gripper_index().is_some() {
            if let Err(err) = self.check_pos_force_gripper_fresh() {
                self.terminate_policy_run_static(
                    "J7_FEEDBACK_STALE",
                    "failed",
                    Some("J7_FEEDBACK_STALE"),
                    Some("Gripper (J7) position-force feedback is stale (> 100 ms)"),
                );
                return Err(err);
            }
        }

        if let Err(e) = self.validate_and_filter_servo_targets(targets, epoch_id) {
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
        self.release_gripper_stall_on_reverse(targets);
        let _ = self.safety.mark_motion_started();

        copy_into(&mut self.target_positions, targets);
        self.set_servo_target(targets);
        self.servo_stream_active = true;
        self.servo_settled_ticks.fill(0);

        if let Some(run) = self.policy_run.as_mut().filter(|r| r.action_id == action_id) {
            run.steps += 1;
            run.last_input_age_ms = input_age_ms;
            run.last_vision_age_ms = vision_age_ms;
            run.vision_frame_id = vision_frame_id;
            set_vec(&mut run.last_shadow_target, targets);
            set_vec(&mut run.last_live_target, targets);
            set_policy_obs(&mut self.last_policy_obs, run);
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
        if let SafetyState::Fault(r) = self.safety.state() {
            let r = r.clone();
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

    /// Zero-allocation termination helper for static messages on `armd-control`: fills a
    /// preallocated `action_msg_spares` buffer and terminates without heap allocation or free.
    pub(crate) fn terminate_policy_run_static(
        &mut self,
        stop_reason: &'static str,
        action_state: &'static str,
        action_code: Option<&'static str>,
        action_message: Option<&'static str>,
    ) {
        let Some(action_id) = self.policy_run.as_ref().map(|r| r.action_id) else {
            return;
        };
        let msg = action_message.map(|text| {
            let mut buf = self.take_action_msg_buf(action_id, 256);
            buf.clear();
            buf.push_str(text);
            buf
        });
        self.terminate_policy_run(stop_reason, action_state, action_code, msg);
    }

    pub(crate) fn terminate_policy_run(
        &mut self,
        stop_reason: &str,
        action_state: &str,
        action_code: Option<&str>,
        action_message: Option<String>,
    ) {
        let Some(mut run) = self.policy_run.take() else {
            return;
        };
        self.end_policy_stream(run.action_id);
        self.set_action_policy_stats(run.action_id, run.step_stats.report());
        if !self.rt_executor_owned {
            run.stop_reason = Some(stop_reason.to_string());
            self.record_policy_qualification(&run, stop_reason, action_state);
        }
        if run.mode == crate::policy_runtime::PolicyMode::Live {
            self.release_servo_target();
            self.servo_settled_ticks.fill(0);
            self.servo_ws_min_depths = None;
            copy_into(&mut self.target_positions, &self.commanded_positions);
        }
        if let Some(obs) = self.last_policy_obs.as_mut() {
            run.to_observation_into(false, obs);
            obs.stop_reason = Some(crate::model::HolderLabel::new(stop_reason));
        } else {
            let mut obs = run.to_observation(false);
            obs.stop_reason = Some(crate::model::HolderLabel::new(stop_reason));
            self.last_policy_obs = Some(obs);
        }
        let action_id = run.action_id;
        self.finish_action(
            action_id,
            action_state,
            action_code,
            action_message,
            None,
            None,
            None,
        );
        self.retire_policy_run(run);
    }
}

// Re-exported as `supervisor::targets`. The broker-side items have no production caller
// until the WS loop and the policy runner route through the mailbox (stage 3 removes this).
#[allow(dead_code)]
pub mod targets {
    //! Latest-target path of servo, teleop and policy streams (armd 1.6 RT core, track D).
    //!
    //! **Broker side** ([`TargetPort`], normal priority): applies the WS servo sequence rules
    //! of both servo forms (`servo` and `cartesian_joint_stream`) per connection
    //! ([`ServoStream`]), stamps each target with the arm and boot, its stream, the authority
    //! it was received under, its original input / camera times and frame id and a deadline,
    //! and submits it under the port's strictly increasing sequence (a client sequence that
    //! wraps stays valid). Servo frames are replaceable (a burst coalesces to the newest);
    //! policy frames are not (each live step is checked against the step before it). The
    //! RT's outcomes come back through [`TargetPort::outcomes`]; [`servo_outcome_reply`] turns
    //! a refusal into the WS reply the legacy loop sends for the same condition.
    //!
    //! **RT side** ([`SupervisorInner::consume_target_frame`], the body of
    //! `CoreOps::consume_target`): [`TargetFrame::check`] (dof, identity, deadline, stop,
    //! epoch, lease valid now, the authorized stream, policy generation, input and camera
    //! ages), then a lease when leases are enforced, the config hold, and the existing servo /
    //! policy step checks (lead, braking, workspace, jump caps). Streams are authorized by
    //! the core: [`SupervisorInner::authorize_servo_stream`] when a connection holds the lease,
    //! the policy stream while a run is active (its generation follows the run's
    //! [`crate::policy_runtime::PolicyAuthorization`], so a vision hold or invalidation
    //! refuses its buffered targets). A refused policy target holds or ends the run where the
    //! legacy commit holds or ends it. Shadow runs keep no output authority. Accepted targets
    //! are applied without allocating once the servo buffers hold a target.
    //!
    //! Targets carry no plan revision: they are checked against the active workspace when
    //! applied, so a workspace change does not refuse a buffered target as `PLAN_STALE`.

    use std::time::{Duration, Instant};

    use serde_json::{json, Value};

    use crate::rt_core::broker::{Authority, BootId};
    use crate::rt_core::ids::ArmId;
    use crate::rt_core::mailbox::{
        Refusal, RefusalDetail, StreamId, Submitted, TargetAuthority, TargetFrame, TargetOutcome, TargetWriter, MAX_DOF,
    };
    use crate::rt_core::ops::{CommitCtx, CoreOps};
    use crate::safety::SafetyError;
    #[cfg(feature = "policy-runtime")]
    use crate::safety::SafetyState;
    use crate::supervisor::{ArmError, SupervisorInner};

    /// Servo stream ids carry this bit; policy stream ids are action ids (without it).
    pub const SERVO_STREAM_BIT: u64 = 1 << 63;
    /// A servo frame not applied within this time is refused (`DEADLINE_EXPIRED`): the servo
    /// motion-frame timeout.
    pub const SERVO_TARGET_TTL: Duration = Duration::from_millis(200);

    /// The stream of WS servo frames under lease `lease_generation`.
    pub fn servo_stream(lease_generation: u64) -> StreamId { StreamId(SERVO_STREAM_BIT | lease_generation) }

    /// The stream of policy run `action_id`.
    pub fn policy_stream(action_id: u64) -> StreamId { StreamId(action_id & !SERVO_STREAM_BIT) }

    /// Codes of the policy step that a refusal carries as they are (others: `REJECTED`).
    const POLICY_CODES: &[&str] = &[
        "POLICY_NOT_ACTIVE", "LEASE_LOST", "BLOCKED_HOLD", "DEPLOYMENT_CHANGED", "OPERATOR_CONFIRMATION_REQUIRED",
        "RELEASE_GATE_REQUIRED", "FEEDBACK_UNCONFIRMED", "STALE_INPUT", "JUMP_LIMIT", "STEP_LIMIT", "QUALIFICATION_REQUIRED",
        "J7_FEEDBACK_STALE",
    ];

    /// The protocol code of a servo error, as `ApiError::from` gives it (no allocation).
    pub fn safety_code(e: &SafetyError) -> &'static str {
        match e {
            SafetyError::HandoverSettling => "HANDOVER_SETTLING",
            SafetyError::ArmNotAtRestingPose { .. } => "ARM_NOT_AT_RESTING_POSE",
            SafetyError::MotionForbiddenInFaultState(_) => "MOTION_FORBIDDEN_IN_FAULT_STATE",
            SafetyError::ClearFaultRejected(_) => "CLEAR_FAULT_REJECTED",
            SafetyError::EStopActive => "ESTOP_ACTIVE",
            SafetyError::CollisionHold(_) => "COLLISION_HOLD",
            SafetyError::CommissioningRequired(_) => "COMMISSIONING_REQUIRED",
            SafetyError::MotionBusy => "MOTION_BUSY",
            SafetyError::WorkspaceViolation { .. } => "WORKSPACE_VIOLATION",
            SafetyError::EpochMismatch { .. } => "EPOCH_MISMATCH",
            SafetyError::SoftLimitExceeded { .. } => "SOFT_LIMIT_EXCEEDED",
            SafetyError::FaultActive(_) => "FAULT_ACTIVE",
            SafetyError::NotArmed(_) => "NOT_ARMED",
            SafetyError::OwnerTransferred => "OWNER_TRANSFERRED",
            SafetyError::Stopped => "STOPPED",
            SafetyError::LeaseLost => "LEASE_LOST",
            SafetyError::TerminalsBusy => "TERMINALS_BUSY",
            SafetyError::StampAlreadyConsumed => "STAMP_REUSED",
            _ => "SAFETY_ERROR",
        }
    }

    /// The protocol code of a policy step error (no allocation).
    pub fn policy_code(e: &ArmError) -> &'static str {
        match e {
            ArmError::Safety(s) => safety_code(s),
            ArmError::RejectedStatic { code, .. } => code,
            ArmError::Rejected { code, .. } => POLICY_CODES.iter().find(|k| **k == code.as_str()).copied().unwrap_or("REJECTED"),
            ArmError::Internal(_) => "INTERNAL_ERROR",
            _ => "REJECTED",
        }
    }

    /// Status, `error` and `safety_state` of a refusal code, as `ApiError::from` gives them.
    fn reject_fields(code: &str) -> (u16, &str, Option<&'static str>) {
        match code {
            "ESTOP_ACTIVE" => (409, code, Some("EStop")),
            "FAULT_ACTIVE" => (409, code, Some("Fault")),
            "ARM_NOT_AT_RESTING_POSE" => (409, "ArmNotAtRestingPose", Some("Armed")),
            "MOTION_FORBIDDEN_IN_FAULT_STATE" => (409, "MotionForbiddenInFaultState", Some("Fault")),
            "CLEAR_FAULT_REJECTED" => (409, "ClearFaultRejected", Some("Fault")),
            "WORKSPACE_VIOLATION" | "JUMP_LIMIT" | "STEP_LIMIT" => (422, code, None),
            "SOFT_LIMIT_EXCEEDED" | "SAFETY_ERROR" | "INVALID_COMMAND" => (400, code, None),
            "NOT_ARMED" => (412, code, None),
            "OPERATOR_CONFIRMATION_REQUIRED" | "RELEASE_GATE_REQUIRED" => (403, code, None),
            "INTERNAL_ERROR" => (500, code, None),
            _ => (409, code, None),
        }
    }

    /// What a WS reply needs from the state at reply time.
    #[derive(Debug, Clone)]
    pub struct ReplyCtx {
        pub epoch_id: u64,
        pub safety_state: Value,
        /// The connection's lease generation.
        pub lease_generation: u64,
    }

    /// `servo_reject` with the legacy fields (`status`, `code`, `error`, `message`, `safety_state`, `epoch_id`).
    pub fn servo_reject(code: &str, message: &str, epoch_id: u64) -> Value {
        let (status, error, safety_state) = reject_fields(code);
        json!({"type":"servo_reject", "status":status, "code":code, "error":error, "message":message,
            "safety_state":safety_state, "epoch_id":epoch_id})
    }

    /// The WS reply to a servo target the RT refused (None: no reply).
    ///
    /// As the legacy loop answers the same conditions: an unauthorized frame (lease lost,
    /// stream not authorized, no lease) with `lease_lost` (with the loss reason when known),
    /// a stale epoch with `epoch_sync`, a calibration with `servo_reject`
    /// `reason: calibration_busy`, other refusals with `servo_reject` (and `current_revision`
    /// when known). A frame refused because a stop overtook it gets no reply (legacy applied
    /// it before the stop, which has its own reply). Messages are generic.
    pub fn servo_outcome_reply(refusal: &Refusal, ctx: &ReplyCtx) -> Option<Value> {
        Some(match refusal.code {
            "STOPPED" => return None,
            "LEASE_LOST" | "NO_LEASE" | "STREAM_NOT_AUTHORIZED" => {
                let mut v = json!({"type":"lease_lost", "generation":ctx.lease_generation});
                if let (RefusalDetail::LeaseLoss { reason }, true) = (refusal.detail, ctx.lease_generation != 0) { v["reason"] = json!(reason); }
                v
            }
            "EPOCH_MISMATCH" => json!({"type":"epoch_sync", "epoch_id":ctx.epoch_id, "safety_state":ctx.safety_state}),
            "CALIBRATION_BUSY" => json!({"type":"servo_reject", "reason":"calibration_busy"}),
            code => {
                let mut v = servo_reject(code, "servo target refused when the control loop applied it", ctx.epoch_id);
                if let RefusalDetail::CurrentRevision(rev) = refusal.detail { v["current_revision"] = json!(rev); }
                v
            }
        })
    }

    /// Per-connection servo stream state (the WS loop's `last_sequence`).
    #[derive(Debug, Default)]
    pub struct ServoStream {
        last_sequence: Option<u64>,
        lease_generation: u64,
    }

    impl ServoStream {
        pub fn new() -> Self { Self::default() }

        /// The connection got lease `generation`: a new lease starts a new sequence.
        pub fn lease_granted(&mut self, generation: u64) {
            if self.lease_generation != generation { self.last_sequence = None; }
            self.lease_generation = generation;
        }

        pub fn last_sequence(&self) -> Option<u64> { self.last_sequence }
    }

    /// One WS servo message (either form), already authorized and epoch-checked by the loop.
    #[derive(Debug, Clone, Copy)]
    pub struct ServoMessage<'a> {
        pub sequence_id: Option<u64>,
        pub positions: &'a [f32],
        /// Lease generation, the message's epoch and the stop generation at receipt.
        pub authority: Authority,
        pub received_at: Instant,
        /// A calibration apply / restore runs (the loop's snapshot; the RT checks again).
        pub calibration_busy: bool,
    }

    /// One policy target, stamped when its inputs were sampled.
    #[derive(Debug, Clone, Copy)]
    pub struct PolicyTarget<'a> {
        pub action_id: u64,
        pub stamp: PolicyStamp,
        pub targets: &'a [f32],
        /// Capture time of the oldest joint sample the target was computed from.
        pub input_at: Instant,
        /// Capture time and frame id of the camera sample behind the target.
        pub vision_at: Option<Instant>,
        pub vision_frame_id: Option<u64>,
    }

    /// The broker's side of the latest-target mailbox.
    pub struct TargetPort {
        writer: TargetWriter,
        arm: ArmId,
        boot: BootId,
        seq: u64,
    }

    impl TargetPort {
        pub fn new(writer: TargetWriter, arm: ArmId, boot: BootId) -> Self { Self { writer, arm, boot, seq: 0 } }

        /// Frames replaced before consumption.
        pub fn coalesced(&self) -> u64 { self.writer.coalesced() }

        /// Drains the RT's outcomes of consumed frames.
        pub fn outcomes(&mut self, each: impl FnMut(TargetOutcome)) -> usize { self.writer.outcomes(each) }

        /// The connection holding lease `lease_generation` closed: forget its stream.
        pub fn close_servo(&mut self, lease_generation: u64) { self.writer.close_stream(servo_stream(lease_generation)); }

        /// Policy run `action_id` ended: forget its stream.
        pub fn close_policy(&mut self, action_id: u64) { self.writer.close_stream(policy_stream(action_id)); }

        #[allow(clippy::too_many_arguments)]
        fn frame(&mut self, stream: StreamId, authority: Authority, policy_generation: u64, q: &[f32], input_at: Instant,
            vision_at: Option<Instant>, vision_frame_id: Option<u64>, deadline: Instant, replaceable: bool) -> Option<TargetFrame> {
            if q.is_empty() || q.len() > MAX_DOF || q.iter().any(|v| !v.is_finite()) { return None; }
            let mut a = [0.0; MAX_DOF];
            a[..q.len()].copy_from_slice(q);
            self.seq += 1;
            Some(TargetFrame { arm: self.arm, boot: self.boot, stream, seq: self.seq, authority, policy_generation, input_at, vision_at,
                vision_frame_id, deadline, dof: q.len() as u8, q: a, replaceable })
        }

        /// Submits one WS servo message; returns the WS reply (None: queued, no reply).
        /// Sequence rules as the legacy loop: a missing `sequence_id`, a repeat or a jump of
        /// more than 1000 (wrapping) is refused; the sequence advances once it passes them.
        pub fn submit_servo(&mut self, stream: &mut ServoStream, msg: &ServoMessage<'_>) -> Option<Value> {
            if msg.calibration_busy { return Some(json!({"type":"servo_reject", "reason":"calibration_busy"})); }
            let Some(sequence) = msg.sequence_id else { return Some(json!({"type":"servo_reject", "reason":"missing_sequence_id"})); };
            if stream.last_sequence.is_some_and(|last| { let delta = sequence.wrapping_sub(last); delta == 0 || delta > 1000 }) {
                return Some(json!({"type":"servo_reject", "reason":"invalid_sequence_id"}));
            }
            stream.last_sequence = Some(sequence);
            let epoch = msg.authority.epoch;
            let stream_id = servo_stream(msg.authority.lease_generation);
            let Some(frame) = self.frame(stream_id, msg.authority, 0, msg.positions, msg.received_at, None, None,
                msg.received_at + SERVO_TARGET_TTL, true) else {
                return Some(servo_reject("SAFETY_ERROR", "servo target is empty, has non-finite values or more joints than the arm", epoch));
            };
            match self.writer.submit(frame) {
                Submitted::Accepted | Submitted::Coalesced { .. } => None,
                Submitted::Busy => Some(servo_reject("MOTION_BUSY", "a policy target is waiting to be applied", epoch)),
                Submitted::StreamsFull => Some(servo_reject("STREAMS_FULL", "too many open target streams", epoch)),
                Submitted::OutOfOrder => Some(json!({"type":"servo_reject", "reason":"invalid_sequence_id"})),
            }
        }

        /// Submits one policy target (not replaceable; deadline: input time + twice the run's
        /// input age bound). `Busy`: the previous target was not consumed yet; the runner counts the
        /// step as skipped.
        pub fn submit_policy(&mut self, t: &PolicyTarget<'_>) -> Submitted {
            // after the input age bound, so an aged input ends the run as `STALE_INPUT`
            let deadline = t.input_at + 2 * t.stamp.max_input_age;
            match self.frame(policy_stream(t.action_id), t.stamp.authority, t.stamp.authorization.max(1), t.targets, t.input_at,
                t.vision_at, t.vision_frame_id, deadline, false) {
                Some(frame) => self.writer.submit(frame),
                None => Submitted::Busy,
            }
        }
    }

    /// What the runner stamps a policy target with, read when it samples the inputs.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct PolicyStamp {
        pub authority: Authority,
        pub authorization: u64,
        pub max_input_age: Duration,
    }

    impl SupervisorInner {
        /// Consume the servo frames of the connection holding lease `lease_generation`
        /// (replaces the authorized stream; requires a single-use [`Admitted`] proof).
        pub fn authorize_servo_stream(admitted: crate::supervisor::Admitted<'_>, lease_generation: u64) -> Result<(), SafetyError> {
            // Authorizing a stream does not itself produce a J7 target; preserve any existing
            // J7 admission stamp rather than overwriting it during post-stop policy cleanup.
            let prev_j7_stamp = admitted.inner.j7_admitted_stamp;
            let (this, proof) = admitted.consume()?;
            this.j7_admitted_stamp = prev_j7_stamp;
            let stamp = proof.stamp();
            if lease_generation == 0 || stamp.lease_gen != lease_generation {
                return Err(SafetyError::LeaseLost);
            }
            this.core.authorize_stream(TargetAuthority { stream: Some(servo_stream(lease_generation)), ..TargetAuthority::default() });
            Ok(())
        }

        /// After a policy run of `action_id`: hand the targets back to the lease holder's
        /// servo stream (none without a lease or under E-stop) when the run's stream was authorized.
        #[cfg(feature = "policy-runtime")]
        pub(in crate::supervisor) fn end_policy_stream(&mut self, action_id: u64) {
            if self.core.target_authority().stream != Some(policy_stream(action_id)) { return; }
            match self.active_lease.as_ref().map(|l| l.generation) {
                Some(g) => {
                    let stamp = crate::supervisor::AdmissionStamp::for_rt_internal(
                        self.current_stop_gen(),
                        self.safety.epoch_id(),
                        g,
                    );
                    if let Ok(admitted) = self.check_admission(stamp) {
                        let _ = Self::authorize_servo_stream(admitted, g);
                    } else {
                        self.core.revoke_stream();
                    }
                }
                None => self.core.revoke_stream(),
            }
        }

        /// The stamp of run `action_id`'s next target (None: no such run).
        #[cfg(feature = "policy-runtime")]
        pub fn policy_target_stamp(&self, action_id: u64, stop_generation: u64) -> Option<PolicyStamp> {
            let run = self.policy_run.as_ref().filter(|r| r.action_id == action_id)?;
            let authority = Authority { lease_generation: run.lease_generation, epoch: run.epoch_id, stop_generation,
                config_revision: self.workspace_revision, ..Authority::default() };
            Some(PolicyStamp { authority, authorization: run.authorization.current(), max_input_age: Duration::from_millis(run.max_input_age_ms) })
        }

        /// The authorized policy stream follows its run: authorization generation and age
        /// bounds (the input age is unbounded when the driver reports none, as in the legacy
        /// runner; the step then decides).
        #[cfg(feature = "policy-runtime")]
        fn sync_policy_target_authority(&mut self) {
            let Some(run) = self.policy_run.as_ref() else { return; };
            let stream = policy_stream(run.action_id);
            if self.core.target_authority().stream != Some(stream) { return; }
            let input_known = self.has_policy_input_age_range();
            let run = self.policy_run.as_ref().expect("checked");
            let a = TargetAuthority {
                stream: Some(stream),
                policy_generation: run.authorization.current(),
                max_input_age: input_known.then(|| Duration::from_millis(run.max_input_age_ms)),
                max_vision_age: run.max_vision_age_ms.map(Duration::from_millis),
            };
            if a != self.core.target_authority() { self.core.authorize_stream(a); }
        }

        fn target_refusal(&self, code: &'static str, frame: &TargetFrame) -> Refusal {
            let detail = match code {
                "LEASE_LOST" => {
                    let preempted = self.last_evicted.as_ref().is_some_and(|(_, g)| *g == frame.authority.lease_generation);
                    RefusalDetail::LeaseLoss { reason: if preempted { "preempted" } else { "expired" } }
                }
                "PLAN_STALE" => RefusalDetail::CurrentRevision(self.workspace_revision),
                _ => RefusalDetail::None,
            };
            Refusal { code, detail }
        }

        /// Applies one target from the latest-target mailbox after rechecking it (see the
        /// module doc) at the release `ctx` (its urgent-stop generation and clock).
        pub fn consume_target_frame(&mut self, frame: &TargetFrame, ctx: &CommitCtx) -> Result<(), Refusal> {
            #[cfg(feature = "policy-runtime")]
            self.sync_policy_target_authority();
            // a target is checked against the active workspace when applied; it carries no plan revision
            let current = Authority { config_revision: frame.authority.config_revision, ..CoreOps::authority(self, ctx) };
            let policy = frame.policy_generation != 0;
            if let Some(code) = frame.check(ctx.arm, ctx.boot, ctx.now, &current, &self.core.target_authority()) {
                if policy { self.policy_target_refused(code, frame); }
                // a servo frame while a policy run owns the targets: the legacy MOTION_BUSY
                #[cfg(feature = "policy-runtime")]
                if code == "STREAM_NOT_AUTHORIZED" && !policy && (self.policy_run.is_some() || self.policy_loading) {
                    return Err(Refusal::code("MOTION_BUSY"));
                }
                return Err(self.target_refusal(code, frame));
            }
            if self.lease_enabled && frame.authority.lease_generation == 0 { return Err(Refusal::code("NO_LEASE")); }
            if self.core.motion_inhibited_by_config() { return Err(Refusal::code("CONFIG_BUSY")); }
            if policy { return self.consume_policy_target(frame, ctx.now); }
            if self.calibration_busy { return Err(Refusal::code("CALIBRATION_BUSY")); }
            self.sync_stop_generation(ctx.stop_generation);
            let stamp = crate::supervisor::AdmissionStamp::for_rt_internal(
                frame.authority.stop_generation,
                frame.authority.epoch,
                frame.authority.lease_generation,
            );
            let admitted = self.check_admission(stamp).map_err(|e| Refusal::code(safety_code(&e)))?;
            Self::servo_slice(admitted, frame.positions(), frame.authority.epoch).map_err(|e| {
                let joint = match &e {
                    SafetyError::SoftLimitExceeded { joint, .. } => self.config.limits.iter().position(|l| &l.name == joint).map(|i| i as u8),
                    _ => None,
                };
                let code = safety_code(&e);
                Refusal { code, detail: if joint.is_some() { RefusalDetail::Limit { joint } } else { RefusalDetail::None } }
            })
        }

        #[cfg(not(feature = "policy-runtime"))]
        fn policy_target_refused(&mut self, _code: &str, _frame: &TargetFrame) {}

        /// A target of the active run was refused by the consumption check: hold or end the
        /// run where the legacy commit holds or ends it.
        #[cfg(feature = "policy-runtime")]
        fn policy_target_refused(&mut self, code: &str, frame: &TargetFrame) {
            let Some(run) = self.policy_run.as_ref().filter(|r| frame.stream == policy_stream(r.action_id)) else { return; };
            let (action_id, hold_on_stale) = (run.action_id, run.vision_stale_hold);
            match code {
                // revoked after the target was computed (vision hold or invalidation) or too late: brake now
                "POLICY_REVOKED" | "DEADLINE_EXPIRED" => self.hold_policy_run(action_id),
                "VISION_STALE" if hold_on_stale => self.hold_policy_run(action_id),
                "VISION_STALE" => { self.fail_policy_run(action_id, "VISION_STALE", "Vision sample became stale before the target was applied"); }
                "INPUT_STALE" => self.terminate_policy_run("STALE_INPUT", "failed", Some("STALE_INPUT"),
                    Some("Live policy target computed from stale joint positions (aged while buffered)".into())),
                "LEASE_LOST" => self.terminate_policy_run("LEASE_LOST", "cancelled", Some("LEASE_LOST"), Some("Control lease lost or expired".into())),
                "EPOCH_MISMATCH" => {
                    if self.estop_pending() || matches!(self.safety.state(), SafetyState::EStop) {
                        self.terminate_policy_run("ESTOP", "failed", Some("ESTOP_ACTIVE"), Some("E-stop latched".into()));
                    } else if let SafetyState::Fault(r) = self.safety.state().clone() {
                        let reason = if matches!(r, crate::safety::FaultReason::WatchdogTimeout) { "WATCHDOG_TIMEOUT" } else { "FAULT" };
                        self.terminate_policy_run(reason, "failed", Some(reason), Some(format!("Fault active: {r:?}")));
                    } else {
                        self.terminate_policy_run("STOPPED", "cancelled", Some("EPOCH_MISMATCH"), Some("Epoch changed during policy run".into()));
                    }
                }
                _ => {}
            }
        }

        #[cfg(not(feature = "policy-runtime"))]
        fn consume_policy_target(&mut self, _frame: &TargetFrame, _now: Instant) -> Result<(), Refusal> { Err(Refusal::code("POLICY_NOT_ACTIVE")) }

        /// A policy frame that passed the consumption check: `policy_step_slice` (holds, gates,
        /// input age, jump caps, servo checks; it ends the run where the legacy commit ends it).
        #[cfg(feature = "policy-runtime")]
        fn consume_policy_target(&mut self, frame: &TargetFrame, now: Instant) -> Result<(), Refusal> {
            let ms = |t: Instant| now.saturating_duration_since(t).as_millis().min(u128::from(u64::MAX)) as u64;
            let Some(action_id) = self.policy_run.as_ref().map(|r| r.action_id).filter(|id| frame.stream == policy_stream(*id)) else {
                return Err(Refusal::code("POLICY_NOT_ACTIVE"));
            };
            let input_age_ms = self.has_policy_input_age_range().then(|| ms(frame.input_at));
            self.policy_step_slice(action_id, frame.positions(), input_age_ms, frame.vision_at.map(ms), frame.vision_frame_id)
                .map_err(|e| Refusal::code(policy_code(&e)))
        }
    }
}

#[cfg(test)]
#[path = "streams_target_tests.rs"]
mod target_tests;
