//! Servo streams and the policy runner of the supervisor: servo target validation, bounded servo steps, tracking fallback, servo holds and faults, policy start/step/stop (track D).
//!
//! Moved from `supervisor.rs` without changes (armd 1.6 RT-core foundation); private
//! methods are `pub(super)` so the rest of the supervisor keeps calling them.

#[allow(unused_imports)]
use super::*;

mod policy;

#[inline]
pub(super) fn buf_len_ok(v: &[f64], n: usize) -> bool {
    v.len() >= n
}

/// Copies `src` into `dst`, reusing its memory when it holds a vector.
pub(super) fn set_vec(dst: &mut Option<Vec<f32>>, src: &[f32]) {
    match dst {
        Some(v) => copy_into(v, src),
        None => *dst = Some(src.to_vec()),
    }
}

impl SupervisorInner {
    /// Clears the servo target, keeping its buffer for the next one (no free).
    pub(crate) fn release_servo_target(&mut self) {
        if let Some(v) = self.servo_target.take() {
            self.bump_motion_generation();
            // the spare is empty (capacity 0) while a target holds the buffer: nothing is freed
            if v.capacity() >= self.servo_target_spare.capacity() { self.servo_target_spare = v; }
        }
    }

    /// Sets the servo target, in the existing or the spare buffer (no allocation while one
    /// has the capacity).
    pub(super) fn set_servo_target(&mut self, src: &[f32]) {
        if self.servo_target.is_none() {
            self.bump_motion_generation();
            let mut v = std::mem::take(&mut self.servo_target_spare);
            copy_into(&mut v, src);
            self.servo_target = Some(v);
        } else {
            set_vec(&mut self.servo_target, src);
        }
    }
}

/// Copies `src` into `dst`, reusing `vec_spare` for `last_shadow_target` (0 RT alloc/free).
#[cfg(feature = "policy-runtime")]
pub(super) fn copy_policy_obs_with_vec_spare(
    dst: &mut crate::model::PolicyObservation,
    src: &crate::model::PolicyObservation,
    vec_spare: &mut Vec<f32>,
) {
    use crate::rt_core::scratch::{copy_into, set_str};
    dst.active = src.active;
    dst.action_id = src.action_id;
    set_str(&mut dst.policy_id, &src.policy_id);
    set_str(&mut dst.artifact_digest, &src.artifact_digest);
    set_str(&mut dst.backend, &src.backend);
    set_str(&mut dst.mode, &src.mode);
    dst.steps = src.steps;
    dst.last_input_age_ms = src.last_input_age_ms;
    dst.last_vision_age_ms = src.last_vision_age_ms;
    dst.vision_frame_id = src.vision_frame_id;
    match src.last_shadow_target.as_deref() {
        Some(v) => match dst.last_shadow_target.as_mut() {
            Some(d) => copy_into(d, v),
            None => {
                let mut buf = std::mem::take(vec_spare);
                copy_into(&mut buf, v);
                dst.last_shadow_target = Some(buf);
            }
        },
        None => {
            if let Some(old) = dst.last_shadow_target.take() {
                if old.capacity() >= vec_spare.capacity() {
                    *vec_spare = old;
                }
            }
        }
    }
    dst.stop_reason = src.stop_reason;
}

/// Bounded copy into a preallocated `String` without ever reallocating past `buf.capacity()`.
#[cfg(feature = "policy-runtime")]
pub(crate) fn push_str_bounded(buf: &mut String, text: &str) {
    buf.clear();
    let cap = buf.capacity();
    if cap == 0 || text.is_empty() {
        return;
    }
    let mut n = text.len().min(cap);
    while n > 0 && !text.is_char_boundary(n) {
        n -= 1;
    }
    buf.push_str(&text[..n]);
}

/// The run's observation into `dst`, reusing its memory.
#[cfg(feature = "policy-runtime")]
pub(super) fn set_policy_obs(dst: &mut Option<crate::model::PolicyObservation>, run: &crate::policy_runtime::PolicyRun) {
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
                if let Some(scale) = run.release_gate_max_speed_scale() {
                    let scale = (scale as f32).clamp(0.1, 1.0);
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
            if !self.rt_executor_owned {
                tracing::warn!("Policy run ended by the control loop: {reason}: {message}");
            }
            self.terminate_policy_run(reason, "failed", Some(reason), Some(message));
        }
    }

    /// Zero-allocation RT equivalent of [`Self::fail_live_policy_from_control`].
    #[allow(unused_variables)]
    pub(super) fn fail_live_policy_from_control_static(&mut self, reason: &'static str, message: &'static str) {
        #[cfg(feature = "policy-runtime")]
        if self.policy_run.as_ref().is_some_and(|r| r.mode == crate::policy_runtime::PolicyMode::Live) {
            self.terminate_policy_run_static(reason, "failed", Some(reason), Some(message));
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
        if !self.rt_executor_owned {
            tracing::warn!(
                "Joint {} blocked during servo streaming ({why}; cmd {:.3} meas {:.3} rad): stream stopped, holding position",
                joint + 1, self.commanded_positions[joint], self.measured_positions[joint]);
        }
        #[cfg(feature = "policy-runtime")]
        if self.policy_run.is_some() {
            if self.rt_executor_owned {
                self.terminate_policy_run_static(
                    "BLOCKED_HOLD",
                    "cancelled",
                    Some("BLOCKED_HOLD"),
                    Some("Joint blocked during policy streaming"),
                );
            } else {
                self.terminate_policy_run(
                    "BLOCKED_HOLD",
                    "cancelled",
                    Some("BLOCKED_HOLD"),
                    Some(format!("Joint {} blocked during policy streaming: {why}", joint + 1)),
                );
            }
        }
        self.servo_hold(MotionLimitReason::Blocked, Some(joint));
    }

    /// Zero-allocation RT workspace refusal hold for `armd-control`: static codes & preallocated
    /// `WorkspaceViolationDetail` text (zero heap allocations, zero frees).
    pub(super) fn enter_servo_keepout_hold_rt(&mut self, f: crate::chain::WorkspaceFailureRt) {
        #[cfg(feature = "policy-runtime")]
        if self.policy_run.is_some() {
            if self.rt_executor_owned {
                self.terminate_policy_run_static(
                    "WORKSPACE_HOLD",
                    "cancelled",
                    Some("WORKSPACE_HOLD"),
                    Some("Policy step violated workspace"),
                );
            } else {
                self.terminate_policy_run(
                    "WORKSPACE_HOLD",
                    "cancelled",
                    Some("WORKSPACE_HOLD"),
                    Some("Policy step violated workspace".to_string()),
                );
            }
        }
        self.servo_hold(MotionLimitReason::Workspace, None);
        if self.motion_limit.as_ref().is_some_and(|m| m.reason == MotionLimitReason::Workspace) {
            let mut bufs = std::mem::take(&mut self.ws_eval_bufs);
            let mut detail = self
                .workspace_violation
                .take()
                .or_else(|| self.workspace_violation_spare.take())
                .unwrap_or_else(crate::kinematics::WorkspaceViolationDetail::preallocated_rt);
            if let Some((ws, cam)) = self.workspace_policy() {
                let eval = crate::chain::WorkspaceEvaluator::new_with_tcp_in(
                    &self.config.arm_model,
                    ws,
                    cam,
                    self.config.tcp_offset,
                    &mut bufs,
                );
                eval.write_failure_detail_rt(&f, &mut detail);
                eval.recycle(&mut bufs);
            }
            self.ws_eval_bufs = bufs;
            self.workspace_violation = Some(detail);
        }
    }

    /// Zero-allocation RT static-load refusal hold for `armd-control`: keeps the last accepted
    /// reference pose without committing a worsening intermediate step, drops any active servo
    /// target, and ends a live policy stream with `STATIC_LOAD_LIMIT`.
    pub(super) fn enter_servo_static_load_hold_rt(&mut self) {
        #[cfg(feature = "policy-runtime")]
        if self.policy_run.is_some() {
            self.terminate_policy_run_static(
                "STATIC_LOAD_LIMIT",
                "failed",
                Some("STATIC_LOAD_LIMIT"),
                Some(crate::power::STATIC_LOAD_LIMIT_MSG),
            );
        }
        self.release_servo_target();
        self.servo_settled_ticks.fill(0);
        self.commanded_velocities.fill(0.0);
        copy_into(&mut self.target_positions, &self.commanded_positions);
        self.safety.mark_motion_finished();
        self.note_motion_limit(MotionLimitReason::Power, None);
    }

    /// Ensure a refused static-load update never leaves an active servo target worsening the load.
    pub(super) fn drop_worsening_servo_target_on_static_refusal(&mut self) {
        if self.servo_target.is_some() {
            self.release_servo_target();
            self.servo_settled_ticks.fill(0);
            copy_into(&mut self.target_positions, &self.commanded_positions);
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
    /// Pre-warms `ws_eval_bufs` and `scratch` joint-limit name buffers before `armd-control`
    /// enters `SCHED_FIFO` so the very first servo or braking tick performs zero heap allocations.
    pub fn prewarm_rt_buffers(&mut self) {
        self.prepare_workspace_eval_bufs();
        // Populate `JointLimit::name` String buffers once off-RT so `copy_limits` on the first
        // servo or fallback braking tick reuses them in place via `clone_from` (0 heap allocs).
        crate::rt_core::scratch::copy_limits(&mut self.scratch.step_limits, &self.config.limits);
        crate::rt_core::scratch::copy_limits(&mut self.scratch.fallback_limits, &self.config.limits);
        self.tracking.prewarm_fallback_spares(&self.config.limits);
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

    /// Zero-allocation RT workspace sweep for servo streaming and braking ticks on `armd-control`:
    /// uses pre-sized clearance slices and sweep slab from `self.ws_eval_bufs` (zero heap ops
    /// for any legal workspace size, including 32 end points + multiple keep-out boxes and
    /// refusal / budget-exceeded paths).
    ///
    /// Does NOT commit `self.servo_ws_min_depths`: returns `Ok(Some(updated_len))` when candidate
    /// clearance depths were computed into `self.ws_eval_bufs.scratch_cand_min_d[..updated_len]`,
    /// and the caller must invoke [`Self::commit_servo_ws_min_depths_rt`] ONLY after all sibling
    /// step gates (including static-load admission) also pass.
    pub(super) fn servo_step_workspace_rt(
        &mut self,
        next: &[f32],
        phase: &'static str,
    ) -> Result<Option<usize>, crate::chain::WorkspaceFailureRt> {
        let mut bufs = std::mem::take(&mut self.ws_eval_bufs);
        let mut updated_len = 0usize;
        let result = (|| {
            let Some((ws, cam)) = self.workspace_policy() else {
                return Ok(false);
            };
            let mut eval = crate::chain::WorkspaceEvaluator::new_with_tcp_in(
                &self.config.arm_model,
                ws,
                cam,
                self.config.tcp_offset,
                &mut bufs,
            );
            let mut n = eval.total_slots;
            if !buf_len_ok(&bufs.scratch_c0, n)
                || !buf_len_ok(&bufs.scratch_c1, n)
                || !buf_len_ok(&bufs.scratch_cm, n)
                || !buf_len_ok(&bufs.scratch_cand_min_d, n)
                || !buf_len_ok(&bufs.geom_bounds, eval.num_geoms)
                || bufs.sweep_slab.len() < 2 * n
            {
                // Fallback only if a non-RT caller/test mutated `config.workspace` directly
                // without calling `prepare_workspace_eval_bufs()`; on `armd-control`, every
                // workspace load and config commit pre-sizes `ws_eval_bufs` off-thread.
                eval.recycle(&mut bufs);
                bufs.prepare_for(&self.config.arm_model, ws, cam, self.config.tcp_offset);
                eval = crate::chain::WorkspaceEvaluator::new_with_tcp_in(
                    &self.config.arm_model,
                    ws,
                    cam,
                    self.config.tcp_offset,
                    &mut bufs,
                );
                n = eval.total_slots;
            }
            let checked = (|| {
                eval.eval_into_slice_rt(
                    &self.commanded_positions,
                    &mut bufs.scratch_c0,
                    &mut bufs.world_caps,
                    phase,
                )?;
                if buf_len_ok(&bufs.scratch_cand_min_d, n) {
                    match &self.servo_ws_min_depths {
                        Some((rev, min_d))
                            if *rev == self.workspace_revision && min_d.as_slice().len() == n =>
                        {
                            bufs.scratch_cand_min_d[..n].copy_from_slice(min_d.as_slice());
                        }
                        _ => eval.init_min_depths_slice(&bufs.scratch_c0, &mut bufs.scratch_cand_min_d),
                    }
                } else {
                    return Err(crate::chain::WorkspaceFailureRt {
                        kind: crate::chain::WorkspaceFailureKindRt::Invalid("workspace scratch undersized"),
                        phase,
                    });
                }
                let mut budget = MAX_KEEPOUT_EVALS_PER_TICK;
                eval.sweep_linear_step_rt(
                    &self.commanded_positions,
                    next,
                    &bufs.scratch_c0,
                    &mut bufs.scratch_cand_min_d,
                    &mut budget,
                    MAX_KEEPOUT_EVALS_PER_TICK,
                    phase,
                    phase,
                    &mut bufs.scratch_c1,
                    &mut bufs.scratch_cm,
                    &mut bufs.sweep_slab,
                    &mut bufs.sweep_stack,
                    &mut bufs.sweep_free,
                    &mut bufs.geom_bounds,
                    &mut bufs.world_caps,
                )?;
                updated_len = n;
                Ok(true)
            })();
            eval.recycle(&mut bufs);
            checked
        })();
        self.ws_eval_bufs = bufs;
        result.map(|updated| updated.then_some(updated_len))
    }

    /// Commit candidate workspace clearance depths computed by [`Self::servo_step_workspace_rt`]
    /// into `self.servo_ws_min_depths` (0 heap ops) atomically alongside the accepted step.
    pub(super) fn commit_servo_ws_min_depths_rt(&mut self, updated_len: usize) {
        let rev = self.workspace_revision;
        let mut bufs = std::mem::take(&mut self.ws_eval_bufs);
        if updated_len <= bufs.scratch_cand_min_d.len() {
            let src = &bufs.scratch_cand_min_d[..updated_len];
            let updated_in_place = match self.servo_ws_min_depths.as_mut() {
                Some((cur_rev, dst)) if dst.as_slice().len() == updated_len => {
                    *cur_rev = rev;
                    dst.as_mut_slice().copy_from_slice(src);
                    true
                }
                _ => false,
            };
            if !updated_in_place {
                if let Some((_, old)) = self.servo_ws_min_depths.take() {
                    if old.is_heap() {
                        if bufs.min_depths_spare.is_none() {
                            bufs.min_depths_spare = Some(old);
                        } else {
                            self.push_retired_stop(crate::rt_core::urgent::RetiredStopPayloads {
                                servo_ws_min_depths: Some((0, old)),
                                ..Default::default()
                            });
                        }
                    }
                }
                if let Some(mut spare) = bufs
                    .min_depths_spare
                    .take()
                    .filter(|s| s.as_slice().len() == updated_len)
                {
                    spare.as_mut_slice().copy_from_slice(src);
                    self.servo_ws_min_depths = Some((rev, spare));
                } else {
                    let mut b = crate::kinematics::SlotBuf::zeros(updated_len);
                    b.as_mut_slice().copy_from_slice(src);
                    self.servo_ws_min_depths = Some((rev, b));
                }
            }
        }
        self.ws_eval_bufs = bufs;
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

    /// Zero-allocation RT variant of [`Self::servo_target_workspace`]: uses pre-sized clearance
    /// scratch slices from `self.ws_eval_bufs` (zero heap allocations for any legal workspace
    /// size, including 32 end points + multiple keep-out boxes, and never builds allocating
    /// [`crate::chain::WorkspaceFailure`] strings on refusal).
    pub(super) fn servo_target_workspace_rt(&mut self, target: &[f32]) -> Result<(), crate::rt_core::mailbox::Refusal> {
        use crate::rt_core::mailbox::Refusal;
        let mut bufs = std::mem::take(&mut self.ws_eval_bufs);
        let result = (|| {
            let Some((ws, cam)) = self.workspace_policy() else { return Ok(()); };
            let mut eval = crate::chain::WorkspaceEvaluator::new_with_tcp_in(
                &self.config.arm_model,
                ws,
                cam,
                self.config.tcp_offset,
                &mut bufs,
            )
            .with_escape_tol(crate::kinematics::WORKSPACE_EPS);
            let mut n = eval.total_slots;
            if !buf_len_ok(&bufs.scratch_c0, n)
                || !buf_len_ok(&bufs.scratch_c1, n)
                || !buf_len_ok(&bufs.scratch_min_d, n)
            {
                eval.recycle(&mut bufs);
                bufs.prepare_for(&self.config.arm_model, ws, cam, self.config.tcp_offset);
                eval = crate::chain::WorkspaceEvaluator::new_with_tcp_in(
                    &self.config.arm_model,
                    ws,
                    cam,
                    self.config.tcp_offset,
                    &mut bufs,
                )
                .with_escape_tol(crate::kinematics::WORKSPACE_EPS);
                n = eval.total_slots;
            }
            let checked = if !eval.capsules_valid()
                || !buf_len_ok(&bufs.scratch_c0, n)
                || !buf_len_ok(&bufs.scratch_c1, n)
                || !buf_len_ok(&bufs.scratch_min_d, n)
            {
                Err(Refusal::code("WORKSPACE_VIOLATION"))
            } else {
                match eval
                    .eval_into_slice_rt(
                        &self.commanded_positions,
                        &mut bufs.scratch_c0,
                        &mut bufs.world_caps,
                        "plan",
                    )
                    .and_then(|()| {
                        eval.init_min_depths_slice(&bufs.scratch_c0, &mut bufs.scratch_min_d);
                        eval.eval_into_slice_rt(target, &mut bufs.scratch_c1, &mut bufs.world_caps, "plan")
                    })
                    .and_then(|()| {
                        eval.check_and_advance_slice_rt(
                            &bufs.scratch_c1,
                            &mut bufs.scratch_min_d,
                            "servo_target",
                        )
                    }) {
                    Ok(()) => Ok(()),
                    Err(f) => Err(Refusal {
                        code: "WORKSPACE_VIOLATION",
                        detail: eval.refusal_detail(&f),
                    }),
                }
            };
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
    #[cfg(feature = "policy-runtime")]
    pub(super) fn validate_policy_lease(&mut self, action_id: u64, generation: u64) -> bool {
        self.validate_policy_lease_at(action_id, generation, Instant::now())
    }

    #[cfg(feature = "policy-runtime")]
    pub(super) fn validate_policy_lease_at(&mut self, action_id: u64, generation: u64, now: Instant) -> bool {
        let timeout = self.lease_timeout;
        let fenced = self.effective_fenced_lease_generation();
        if self.pending_disconnect_hold.load(Ordering::SeqCst) > 0 {
            return false;
        }
        if self.active_lease_conn_id != 0 && self.is_connection_closed(self.active_lease_conn_id) {
            return false;
        }
        let Some(conn) = self.policy_run.as_ref().filter(|r| r.action_id == action_id).map(|r| r.connection_id.as_str()) else { return false; };
        match self.active_lease.as_mut() {
            Some(active)
                if active.generation > fenced
                    && now.saturating_duration_since(active.acquired_at) < timeout
                    && active.generation == generation
                    && !conn.is_empty()
                    && active.client_id == conn =>
            {
                active.last_valid_motion_instant = Some(now);
                true
            }
            _ => false,
        }
    }

    pub(super) fn clamp_j7_if_stale(&self, targets: &mut [f32], j7_gate: crate::supervisor::J7PosForceGate) {
        if j7_gate.stale {
            if let Some(g) = self.config.gripper_index() {
                if targets.len() > g {
                    if let Some(freeze_pos) = j7_gate.freeze_pos {
                        targets[g] = freeze_pos;
                    } else if g < self.commanded_positions.len() {
                        targets[g] = self.commanded_positions[g];
                    }
                }
            }
        }
    }

    pub(super) fn validate_and_filter_servo_targets(
        &mut self,
        targets: &mut [f32],
        epoch_id: u64,
    ) -> Result<(), SafetyError> {
        let j7_gate = self.sample_pos_force_j7_gate();
        self.validate_and_filter_servo_targets_with_gate(targets, epoch_id, j7_gate)
    }

    pub(super) fn validate_and_filter_servo_targets_with_gate(
        &mut self,
        targets: &mut [f32],
        epoch_id: u64,
        j7_gate: crate::supervisor::J7PosForceGate,
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
        self.clamp_j7_if_stale(targets, j7_gate);
        if self.check_static_load_admission(targets).is_err() {
            self.note_motion_limit(MotionLimitReason::Power, None);
            self.drop_worsening_servo_target_on_static_refusal();
            return Err(SafetyError::StaticLoadLimit);
        }
        Ok(())
    }

    /// Zero-allocation RT validation & filtering for mailbox servo targets (`armd-control`).
    pub(super) fn validate_and_filter_servo_targets_rt(
        &mut self,
        targets: &mut [f32],
        epoch_id: u64,
        j7_gate: crate::supervisor::J7PosForceGate,
    ) -> Result<(), crate::rt_core::mailbox::Refusal> {
        use crate::rt_core::mailbox::{Refusal, RefusalDetail};
        if self.config.active_tool.commissioning_required {
            return Err(Refusal::code("COMMISSIONING_REQUIRED"));
        }
        if self.gravity_comp.is_some() {
            return Err(Refusal::code("MOTION_BUSY"));
        }
        if self.collision.hold.is_some() {
            return Err(Refusal::code("COLLISION_HOLD"));
        }
        if self.handover_settling_since.is_some() {
            return Err(Refusal::code("HANDOVER_SETTLING"));
        }
        if self.shutdown_committed || self.estop_pending() {
            return Err(Refusal::code("ESTOP_ACTIVE"));
        }
        if epoch_id != self.safety.epoch_id() {
            return Err(Refusal::code("EPOCH_MISMATCH"));
        }
        if self.pending_rearm || self.pending_normal_unload {
            return Err(Refusal::code("NOT_ARMED"));
        }
        if self.active_trajectory.is_some() || self.macro_run.is_some() {
            return Err(Refusal::code("MOTION_BUSY"));
        }
        if let Err((code, idx)) =
            self.safety
                .validate_motion_command_rt(Some(epoch_id), targets, &self.config.limits)
        {
            if code == "SOFT_LIMIT_EXCEEDED" {
                self.note_motion_limit(MotionLimitReason::JointLimit, idx);
                return Err(Refusal {
                    code,
                    detail: RefusalDetail::Limit {
                        joint: idx.map(|i| i as u8),
                    },
                });
            }
            return Err(Refusal::code(code));
        }
        if let Err(refusal) = self.servo_target_workspace_rt(targets) {
            self.clear_workspace_violation_rt();
            self.note_motion_limit(MotionLimitReason::Workspace, None);
            return Err(refusal);
        }
        self.torque_hold_filter(targets);
        self.power_hold_filter(targets);
        self.clamp_j7_if_stale(targets, j7_gate);
        if self.check_static_load_admission(targets).is_err() {
            self.note_motion_limit(MotionLimitReason::Power, None);
            self.drop_worsening_servo_target_on_static_refusal();
            return Err(Refusal::code("STATIC_LOAD_LIMIT"));
        }
        Ok(())
    }

    /// Servo / policy intake of the gripper target: records it as the stream's gripper target and
    /// releases the stall latch on a target away from the stall. A torque-cap latch needs the
    /// target [`GRIPPER_FORCE_RELEASE_MARGIN_RAD`] more open than the hold.
    pub(super) fn release_gripper_stall_on_reverse(&mut self, targets: &[f32]) {
        let j7_gate = self.sample_pos_force_j7_gate();
        self.release_gripper_stall_on_reverse_with_gate(targets, j7_gate);
    }

    pub(super) fn release_gripper_stall_on_reverse_with_gate(
        &mut self,
        targets: &[f32],
        j7_gate: crate::supervisor::J7PosForceGate,
    ) {
        if j7_gate.stale {
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

    pub(super) fn frame_moves_j7(&self, targets: &[f32], prev_cmd_j7: f32, prev_tgt_j7: f32) -> bool {
        self.config.gripper_index().is_some_and(|g| {
            g < targets.len()
                && ((targets[g] - prev_cmd_j7).abs() > 1e-6 || (targets[g] - prev_tgt_j7).abs() > 1e-6)
        })
    }

    pub fn servo(admitted: Admitted<'_>, targets: Vec<f32>, epoch_id: u64) -> Result<(), SafetyError> {
        Self::servo_slice(admitted, &targets, epoch_id)
    }

    /// [`Self::servo`] from a slice: no allocation once the servo buffers hold a target
    /// (the latest-target path consumes through it).
    pub fn servo_slice(admitted: Admitted<'_>, targets_in: &[f32], epoch_id: u64) -> Result<(), SafetyError> {
        let prev_j7_stamp = admitted.inner.j7_admitted_stamp;
        let (this, proof) = admitted.consume()?;
        // Restore prior J7 stamp until validation succeeds AND the frame moves J7 while J7 is fresh.
        this.j7_admitted_stamp = prev_j7_stamp;
        let j7_gate = this.sample_pos_force_j7_gate();
        this.servo_slice_inner(targets_in, epoch_id, j7_gate, proof.stamp())
    }

    pub(super) fn servo_slice_inner(
        &mut self,
        targets_in: &[f32],
        epoch_id: u64,
        j7_gate: crate::supervisor::J7PosForceGate,
        stamp: crate::supervisor::AdmissionStamp,
    ) -> Result<(), SafetyError> {
        if targets_in.len() > crate::model::MAX_MOTION_DOF { return self.servo_slice_oversized(targets_in.to_vec(), epoch_id); }
        let mut buf = [0.0f32; crate::model::MAX_MOTION_DOF];
        let targets = &mut buf[..targets_in.len()];
        targets.copy_from_slice(targets_in);
        #[cfg(feature = "policy-runtime")]
        if self.policy_run.is_some() || self.policy_loading {
            return Err(SafetyError::MotionBusy);
        }
        if self.mit_motion_blocked() { return Err(SafetyError::MotionBusy); }
        let (prev_cmd_j7, prev_tgt_j7) = self
            .config
            .gripper_index()
            .map(|g| {
                (
                    self.commanded_positions.get(g).copied().unwrap_or(0.0),
                    self.target_positions.get(g).copied().unwrap_or(0.0),
                )
            })
            .unwrap_or((0.0, 0.0));
        self.validate_and_filter_servo_targets_with_gate(targets, epoch_id, j7_gate)?;
        if self.servo_blocked_hold { self.clear_blocked_hold(); }

        // Only renew lease and motion frame timestamp AFTER command validation succeeds
        self.last_heartbeat = Instant::now();
        self.last_motion_frame = Instant::now();
        if let Some(ref mut l) = self.active_lease {
            l.last_valid_motion_instant = Some(Instant::now());
        }

        // Gripper stall release check on reverse command (reusing the same single J7 freshness sample)
        self.release_gripper_stall_on_reverse_with_gate(targets, j7_gate);
        if !j7_gate.stale && self.frame_moves_j7(targets, prev_cmd_j7, prev_tgt_j7) {
            self.j7_admitted_stamp = Some(crate::driver::PosForceIntentStamp {
                stop_gen: stamp.stop_gen,
                estop_epoch: stamp.estop_epoch,
                lease_gen: stamp.lease_gen,
            });
        }

        let _ = self.safety.mark_motion_started();

        // 2. Store servo target for tick(dt) to step with actual dt and velocity limits
        copy_into(&mut self.target_positions, targets);
        self.set_servo_target(targets);
        self.servo_stream_active = true;
        self.servo_settled_ticks.fill(0);

        Ok(())
    }

    /// Zero-allocation RT servo target intake (used by [`Self::consume_target_frame`]).
    /// Never allocates on either acceptance or refusal, samples J7 freshness at most once from
    /// Step 2's snapshot, and updates `j7_admitted_stamp` only when a fresh accepted frame moves J7.
    pub(crate) fn servo_slice_rt(
        admitted: Admitted<'_>,
        targets_in: &[f32],
        epoch_id: u64,
    ) -> Result<(), crate::rt_core::mailbox::Refusal> {
        use crate::rt_core::mailbox::Refusal;
        let prev_j7_stamp = admitted.inner.j7_admitted_stamp;
        let (this, proof) = admitted
            .consume()
            .map_err(|e| Refusal::code(targets::safety_code(&e)))?;
        this.j7_admitted_stamp = prev_j7_stamp;
        if targets_in.len() > crate::rt_core::mailbox::MAX_DOF {
            return Err(Refusal::code("SAFETY_ERROR"));
        }
        let mut buf = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
        let targets = &mut buf[..targets_in.len()];
        targets.copy_from_slice(targets_in);
        #[cfg(feature = "policy-runtime")]
        if this.policy_run.is_some() || this.policy_loading {
            return Err(Refusal::code("MOTION_BUSY"));
        }
        if this.mit_motion_blocked() {
            return Err(Refusal::code("MOTION_BUSY"));
        }
        let (prev_cmd_j7, prev_tgt_j7) = this
            .config
            .gripper_index()
            .map(|g| {
                (
                    this.commanded_positions.get(g).copied().unwrap_or(0.0),
                    this.target_positions.get(g).copied().unwrap_or(0.0),
                )
            })
            .unwrap_or((0.0, 0.0));
        let j7_gate = this.sample_pos_force_j7_gate();
        this.validate_and_filter_servo_targets_rt(targets, epoch_id, j7_gate)?;
        if this.servo_blocked_hold {
            this.clear_blocked_hold();
        }
        let now = Instant::now();
        this.last_heartbeat = now;
        this.last_motion_frame = now;
        if let Some(ref mut l) = this.active_lease {
            l.last_valid_motion_instant = Some(now);
        }
        this.release_gripper_stall_on_reverse_with_gate(targets, j7_gate);
        if !j7_gate.stale && this.frame_moves_j7(targets, prev_cmd_j7, prev_tgt_j7) {
            let stamp = proof.stamp();
            this.j7_admitted_stamp = Some(crate::driver::PosForceIntentStamp {
                stop_gen: stamp.stop_gen,
                estop_epoch: stamp.estop_epoch,
                lease_gen: stamp.lease_gen,
            });
        }
        let _ = this.safety.mark_motion_started();
        copy_into(&mut this.target_positions, targets);
        this.set_servo_target(targets);
        this.servo_stream_active = true;
        this.servo_settled_ticks.fill(0);
        Ok(())
    }

    /// More joints than a target frame carries: the same checks refuse it (dimension).
    pub(super) fn servo_slice_oversized(&mut self, mut targets: Vec<f32>, epoch_id: u64) -> Result<(), SafetyError> {
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

    /// Zero-allocation RT variant of [`Self::apply_tracking_fallback_step`] (`dof <= MAX_DOF`).
    /// Reuses preallocated `TrackingRuntime` buffers and recycles `FallbackRamp` on completion
    /// or failure so RT tracking rejection & braking never allocates or frees heap memory.
    pub(super) fn apply_tracking_fallback_step_inline(
        &mut self,
        pending: crate::tracking::PendingFallbackInline,
        gravity_output_enabled: bool,
    ) -> crate::supervisor::TickOutput {
        use crate::supervisor::TickOutput;
        let crate::tracking::PendingFallbackInline { step, ramp, other_depths_rev } = pending;
        match step {
            Ok(fb) => {
                let pos = &fb.positions[..fb.dof];
                let vel = &fb.velocities[..fb.dof];
                if self
                    .check_static_load_tracking_output_with_fallback(pos, true, true)
                    .is_err()
                {
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
                    self.tracking.recycle_fallback_ramp(ramp);
                    self.tracking.invalidate();
                    self.tracking.holding_fallback_pose = true;
                    self.note_motion_limit(MotionLimitReason::Power, None);
                    let mut targets = std::mem::take(&mut self.scratch.targets);
                    targets.clear();
                    targets.extend(
                        self.commanded_positions
                            .iter()
                            .enumerate()
                            .map(|(i, &p)| JointTarget {
                                position: p,
                                velocity: Some(0.0),
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
                    return self.write_output_scratch(dt);
                }
                crate::rt_core::scratch::copy_into(&mut self.commanded_positions, pos);
                crate::rt_core::scratch::copy_into(&mut self.target_positions, pos);
                crate::rt_core::scratch::copy_into(&mut self.commanded_velocities, vel);
                self.commanded_accelerations.fill(0.0);
                let mut targets = std::mem::take(&mut self.scratch.targets);
                targets.clear();
                targets.extend(
                    pos.iter()
                        .zip(vel.iter())
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
                        let fb_slice = fb
                            .ws_min_depths_rev
                            .map(|(rev, n)| (rev, &self.ws_eval_bufs.tracking_staged_ws[..n]));
                        let other_slice = other_depths_rev
                            .map(|(rev, n)| (rev, &self.ws_eval_bufs.tracking_staged_other[..n]));
                        let (full, park) = if ramp.keepout_only {
                            (other_slice, fb_slice)
                        } else {
                            (fb_slice, other_slice)
                        };
                        self.tracking.record_issued_with_ws_depths_inline(pos, vel, full, park);
                        if done {
                            self.tracking.recycle_fallback_ramp(ramp);
                            self.tracking.fallback_ramp = None;
                        } else {
                            self.tracking.fallback_ramp = Some(ramp);
                        }
                        self.tracking.holding_fallback_pose = done;
                    }
                    TickOutput::Deferred { .. } => {
                        self.tracking.fallback_ramp = Some(ramp);
                    }
                    TickOutput::Expired { .. }
                    | TickOutput::Error { .. }
                    | TickOutput::EstopInhibited { .. }
                    | TickOutput::LeaseFenced
                    | TickOutput::ActionDeadlineRefused { .. }
                    | TickOutput::None => {
                        self.tracking.recycle_fallback_ramp(ramp);
                        self.tracking.invalidate();
                    }
                }
                write_res
            }
            Err(why) => {
                let mut bufs = std::mem::take(&mut self.ws_eval_bufs);
                let mut formatted = crate::safety::BoundedText::<384>::empty();
                {
                    let eval = self.workspace_policy().map(|(ws, cam)| {
                        crate::chain::WorkspaceEvaluator::new_with_tcp_in(
                            &self.config.arm_model,
                            ws,
                            cam,
                            self.config.tcp_offset,
                            &mut bufs,
                        )
                    });
                    why.write_with_evaluator(&mut formatted, eval.as_ref());
                    if let Some(ev) = eval {
                        ev.recycle(&mut bufs);
                    }
                }
                self.ws_eval_bufs = bufs;
                if !self.safety.deferred_logging() {
                    tracing::error!(
                        "Joint tracking fallback cannot brake within the limits ({}): no frame sent",
                        formatted.as_str()
                    );
                }
                self.tracking.recycle_fallback_ramp(ramp);
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
                self.tracking.holding_fallback_pose = true;
                self.safety.latch(
                    crate::safety::FaultKind::Other,
                    crate::safety::FaultDetail::formatted(FaultCode::ServoNoBrake, |buf| {
                        let _ = core::fmt::write(
                            buf,
                            format_args!("Joint tracking fallback cannot brake within the limits: {}", formatted.as_str()),
                        );
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
        if self.rt_executor_owned {
            self.fail_live_policy_from_control_static(
                "SERVO_INFEASIBLE",
                "Joint has an infeasible servo braking state",
            );
        } else {
            self.fail_live_policy_from_control("SERVO_INFEASIBLE",
                format!("Joint {} has an infeasible servo braking state", joint_idx + 1));
        }
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
        self.release_servo_target();
        self.servo_settled_ticks.fill(0);
        self.clear_servo_ws_min_depths_rt();
        self.servo_stream_active = false;
        self.tracking.invalidate();
        self.active_trajectory = None;
        self.commanded_velocities.fill(0.0);
        crate::rt_core::scratch::copy_into(&mut self.target_positions, &self.commanded_positions);
        self.safety.latch(crate::safety::FaultKind::Other, detail);
    }
}

// Re-exported as `supervisor::targets`. The broker-side items have no production caller
// until the WS loop and the policy runner route through the mailbox (stage 3 removes this).
#[allow(dead_code)]
pub mod targets;

#[cfg(test)]
mod target_tests;
