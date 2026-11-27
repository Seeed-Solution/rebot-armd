//! Servo streams and the policy runner of the supervisor: servo target validation, bounded servo steps, tracking fallback, servo holds and faults, policy start/step/stop (track D).
//!
//! Moved from `supervisor.rs` without changes (armd 1.6 RT-core foundation); private
//! methods are `pub(super)` so the rest of the supervisor keeps calling them.

#[allow(unused_imports)]
use super::*;

#[inline]
fn buf_len_ok(v: &[f64], n: usize) -> bool {
    v.len() >= n
}

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
            self.bump_motion_generation();
            // the spare is empty (capacity 0) while a target holds the buffer: nothing is freed
            if v.capacity() >= self.servo_target_spare.capacity() { self.servo_target_spare = v; }
        }
    }

    /// Sets the servo target, in the existing or the spare buffer (no allocation while one
    /// has the capacity).
    fn set_servo_target(&mut self, src: &[f32]) {
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
fn copy_policy_obs_with_vec_spare(
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
    fn servo_target_workspace_rt(&mut self, target: &[f32]) -> Result<(), crate::rt_core::mailbox::Refusal> {
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
    fn validate_policy_lease(&mut self, action_id: u64, generation: u64) -> bool {
        self.validate_policy_lease_at(action_id, generation, Instant::now())
    }

    #[cfg(feature = "policy-runtime")]
    fn validate_policy_lease_at(&mut self, action_id: u64, generation: u64, now: Instant) -> bool {
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

    fn clamp_j7_if_stale(&self, targets: &mut [f32], j7_gate: crate::supervisor::J7PosForceGate) {
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

    fn validate_and_filter_servo_targets_with_gate(
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
    fn validate_and_filter_servo_targets_rt(
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

    fn release_gripper_stall_on_reverse_with_gate(
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

    fn frame_moves_j7(&self, targets: &[f32], prev_cmd_j7: f32, prev_tgt_j7: f32) -> bool {
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

    fn servo_slice_inner(
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
        self.policy_observation_into_with_spares(out, None);
    }

    /// Zero-allocation RT policy observation sync using per-slot [`ObservationSpares`] so
    /// `None <-> Some(PolicyObservation)` and `last_shadow_target` transitions perform 0 allocs/frees.
    pub(crate) fn policy_observation_into_with_spares(
        &self,
        out: &mut Option<crate::model::PolicyObservation>,
        spares: Option<&mut crate::rt_core::publish::ObservationSpares>,
    ) {
        let (obs_spare, vec_spare) = match spares {
            Some(s) => (Some(&mut s.policy_obs), Some(&mut s.policy_shadow_vec)),
            None => (None, None),
        };
        match &self.policy_run {
            Some(run) => {
                if out.is_none() {
                    if let Some(slot) = obs_spare.and_then(Option::take) {
                        *out = Some(slot);
                    }
                }
                let dst = out.get_or_insert_with(crate::model::PolicyObservation::preallocated_rt);
                if let Some(vs) = vec_spare {
                    run.to_observation_into_with_vec_spare(true, dst, vs);
                } else {
                    run.to_observation_into(true, dst);
                }
            }
            None => match self.last_policy_obs.as_ref() {
                Some(src) => {
                    if out.is_none() {
                        if let Some(slot) = obs_spare.and_then(Option::take) {
                            *out = Some(slot);
                        }
                    }
                    let dst = out.get_or_insert_with(crate::model::PolicyObservation::preallocated_rt);
                    if let Some(vs) = vec_spare {
                        copy_policy_obs_with_vec_spare(dst, src, vs);
                    } else {
                        dst.clone_from(src);
                    }
                }
                None => {
                    if let Some(old) = out.take() {
                        if let Some(slot) = obs_spare {
                            if slot.is_none() {
                                *slot = Some(old);
                            }
                        }
                    }
                }
            },
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
    pub(crate) fn has_policy_input_age_range(&self) -> bool {
        if self.config.dof == 0 {
            return false;
        }
        match &self.driver {
            None => true,
            Some(d) => d
                .try_with_ref_recover(|drv| drv.feedback_all_within(self.config.dof, Duration::MAX))
                .unwrap_or(false),
        }
    }

    pub fn policy_input_age_range_ms(&self) -> Option<(u64, u64)> {
        match &self.driver {
            None => Some((0, 0)),
            Some(d) => d.with_ref(|drv| {
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
            })?,
        }
    }

    /// Exact oldest joint-feedback instant (no millisecond floor loss) together with the
    /// integer-millisecond `(min, max)` telemetry age range at `now`.
    pub fn policy_input_age_and_oldest_at(
        &self,
        now: std::time::Instant,
    ) -> Option<((u64, u64), std::time::Instant)> {
        match &self.driver {
            None => Some(((0, 0), now)),
            Some(d) => d.with_ref(|drv| {
                let ages = drv.feedback_ages();
                if ages.len() != self.config.dof {
                    return None;
                }
                let durs = ages.into_iter().collect::<Option<Vec<_>>>()?;
                let mut min_ms = u64::MAX;
                let mut max_ms = 0u64;
                let mut max_dur = std::time::Duration::ZERO;
                for dur in durs {
                    if dur > max_dur {
                        max_dur = dur;
                    }
                    let ms = dur.as_millis().min(u64::MAX as u128) as u64;
                    min_ms = min_ms.min(ms);
                    max_ms = max_ms.max(ms);
                }
                if min_ms > max_ms {
                    return None;
                }
                let oldest = now.checked_sub(max_dur).unwrap_or(now);
                Some(((min_ms, max_ms), oldest))
            })?,
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
            rt_gate_binding: None,
            live_target_auth_gen: None,
            rt_owned: false,
            load_token: 0,
            cancel_slot: 0,
            deadline: None,
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

        let mut buf = [0.0f32; crate::model::MAX_MOTION_DOF];
        let n = targets_in.len().min(crate::model::MAX_MOTION_DOF);
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
                SafetyError::StaticLoadLimit => ("STATIC_LOAD_LIMIT", "failed", "STATIC_LOAD_LIMIT"),
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

    /// Zero-allocation RT policy step intake called from [`Self::consume_policy_target`].
    /// Enforces full parity with [`Self::policy_step_slice`] (first-step `JUMP_LIMIT`,
    /// subsequent-step `STEP_LIMIT`, real-arm gate binding, single-sample J7 PosForce gate,
    /// shared servo validation) with 0 heap allocations and 0 frees.
    ///
    /// Two-transition J7 contract:
    /// - A rejected candidate never installs a new `j7_admitted_stamp`.
    /// - Terminating or braking the live run clears `j7_admitted_stamp = None` so the single
    ///   J7 emitter degrades subsequent ticks to `Hold`.
    /// - An accepted frame stamps `j7_admitted_stamp` ONLY after all checks pass and targets
    ///   are installed, and ONLY when `!j7_gate.stale && frame_moves_j7(...)`.
    pub(crate) fn policy_step_slice_rt(
        &mut self,
        action_id: u64,
        frame: &crate::rt_core::mailbox::TargetFrame,
        input_age_ms: Option<u64>,
        vision_age_ms: Option<u64>,
        vision_frame_id: Option<u64>,
        now: Instant,
    ) -> Result<(), crate::rt_core::mailbox::Refusal> {
        use crate::policy_runtime::PolicyMode;
        use crate::rt_core::mailbox::Refusal;

        let targets_in = frame.positions();
        let mut buf = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
        let n = targets_in.len().min(crate::rt_core::mailbox::MAX_DOF);
        buf[..n].copy_from_slice(&targets_in[..n]);
        let targets = &mut buf[..n];

        let (epoch_id, lease_gen, mode, max_first, max_step, has_prev_live, max_input_age_ms) = {
            let Some(active) = self.policy_run.as_ref() else {
                return Err(Refusal::code("POLICY_NOT_ACTIVE"));
            };
            if active.action_id != action_id {
                return Err(Refusal::code("POLICY_NOT_ACTIVE"));
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
            self.terminate_policy_run_static("ESTOP", "failed", Some("ESTOP_ACTIVE"), Some("E-stop latched"));
            return Err(Refusal::code("ESTOP_ACTIVE"));
        }
        if epoch_id != self.safety.epoch_id() {
            self.terminate_policy_run_static(
                "STOPPED",
                "cancelled",
                Some("EPOCH_MISMATCH"),
                Some("Epoch changed during policy run"),
            );
            return Err(Refusal::code("EPOCH_MISMATCH"));
        }
        if let SafetyState::Fault(ref r) = self.safety.state() {
            let reason = if matches!(r, FaultReason::WatchdogTimeout) {
                "WATCHDOG_TIMEOUT"
            } else {
                "FAULT"
            };
            self.terminate_policy_run_static(reason, "failed", Some(reason), Some("Fault active during policy run"));
            return Err(Refusal::code("FAULT_ACTIVE"));
        }
        if !self.validate_policy_lease_at(action_id, lease_gen, now) {
            self.terminate_policy_run_static(
                "LEASE_LOST",
                "cancelled",
                Some("LEASE_LOST"),
                Some("Control lease lost or expired"),
            );
            return Err(Refusal::code("LEASE_LOST"));
        }
        if self.servo_blocked_hold {
            self.terminate_policy_run_static(
                "BLOCKED_HOLD",
                "cancelled",
                Some("BLOCKED_HOLD"),
                Some("Servo blocked hold active"),
            );
            return Err(Refusal::code("BLOCKED_HOLD"));
        }
        if targets_in.len() != self.config.dof || targets.iter().any(|v| !v.is_finite()) {
            self.terminate_policy_run_static(
                "INVALID_TARGET",
                "failed",
                Some("INVALID_TARGET"),
                Some("Policy produced non-finite or wrong-dimension target"),
            );
            return Err(Refusal::code("SAFETY_ERROR"));
        }

        if mode == PolicyMode::Shadow {
            let mut shadow_spare = std::mem::take(&mut self.policy_shadow_vec_spare);
            let mut obs_spare = self.last_policy_obs_spare.take();
            let mut obs_vec_spare = std::mem::take(&mut self.last_policy_obs_vec_spare);
            if let Some(run) = self.policy_run.as_mut().filter(|r| r.action_id == action_id) {
                run.steps += 1;
                run.last_input_age_ms = input_age_ms;
                run.last_vision_age_ms = vision_age_ms;
                run.vision_frame_id = vision_frame_id;
                match run.last_shadow_target.as_mut() {
                    Some(v) => copy_into(v, targets),
                    None => {
                        copy_into(&mut shadow_spare, targets);
                        run.last_shadow_target = Some(std::mem::take(&mut shadow_spare));
                    }
                }
                if self.last_policy_obs.is_none() {
                    self.last_policy_obs = obs_spare.take();
                }
                let dst = self.last_policy_obs.get_or_insert_with(crate::model::PolicyObservation::preallocated_rt);
                run.to_observation_into_with_vec_spare(true, dst, &mut obs_vec_spare);
            }
            if shadow_spare.capacity() > self.policy_shadow_vec_spare.capacity() {
                self.policy_shadow_vec_spare = shadow_spare;
            }
            if obs_spare.is_some() && self.last_policy_obs_spare.is_none() {
                self.last_policy_obs_spare = obs_spare;
            }
            self.last_policy_obs_vec_spare = obs_vec_spare;
            return Ok(());
        }

        // Live mode checks: release-gate calibration binding, real-arm gate, stale joint input,
        // first-step and per-step jump limits, single-sample J7 gate, and shared servo validation.
        let gate_calib_mismatch = self
            .policy_run
            .as_ref()
            .filter(|r| r.action_id == action_id)
            .and_then(|r| r.gate_calibration_digest())
            .is_some_and(|d| !d.eq_ignore_ascii_case(self.active_calibration_digest_str()));
        if gate_calib_mismatch {
            self.terminate_policy_run_static(
                "DEPLOYMENT_CHANGED",
                "failed",
                Some("DEPLOYMENT_CHANGED"),
                Some("Active calibration digest changed during live policy run"),
            );
            return Err(Refusal::code("DEPLOYMENT_CHANGED"));
        }
        if !self.policy_live_allowed() {
            let (has_confirm, has_gate) = self.policy_run.as_ref().map_or((false, false), |run| {
                (
                    run.i_am_at_the_arm,
                    !run.artifact_digest.is_empty()
                        && run
                            .gate_policy_sha256()
                            .is_some_and(|sha| sha.eq_ignore_ascii_case(&run.artifact_digest)),
                )
            });
            if !has_confirm {
                self.terminate_policy_run_static(
                    "OPERATOR_CONFIRMATION_REQUIRED",
                    "failed",
                    Some("OPERATOR_CONFIRMATION_REQUIRED"),
                    Some("Real-arm live policy execution requires operator confirmation"),
                );
                return Err(Refusal::code("OPERATOR_CONFIRMATION_REQUIRED"));
            }
            if !has_gate {
                self.terminate_policy_run_static(
                    "RELEASE_GATE_REQUIRED",
                    "failed",
                    Some("RELEASE_GATE_REQUIRED"),
                    Some("Real-arm live policy execution requires a verified release gate"),
                );
                return Err(Refusal::code("RELEASE_GATE_REQUIRED"));
            }
            if !self.feedback_numeric_valid
                || !self.rt_driver_feedback_confirmed()
                || !self.has_policy_input_age_range()
            {
                self.terminate_policy_run_static(
                    "FEEDBACK_UNCONFIRMED",
                    "failed",
                    Some("FEEDBACK_UNCONFIRMED"),
                    Some("Hardware feedback is unconfirmed during live policy step"),
                );
                return Err(Refusal::code("FEEDBACK_UNCONFIRMED"));
            }
        }

        let input_stale = match input_age_ms {
            Some(age) => age > max_input_age_ms,
            None => !self.policy_live_allowed(),
        };
        if input_stale {
            self.terminate_policy_run_static(
                "STALE_INPUT",
                "failed",
                Some("STALE_INPUT"),
                Some("Live policy target computed from stale joint positions"),
            );
            return Err(Refusal::code("STALE_INPUT"));
        }

        if !has_prev_live {
            if self.measured_positions.len() != self.config.dof
                || self.measured_positions.iter().any(|m| !m.is_finite())
            {
                self.terminate_policy_run_static(
                    "INVALID_FEEDBACK",
                    "failed",
                    Some("INVALID_FEEDBACK"),
                    Some("Measured joint positions are invalid on first live policy step"),
                );
                return Err(Refusal::code("SAFETY_ERROR"));
            }
            for i in 0..self.config.dof {
                if (targets[i] - self.measured_positions[i]).abs() > max_first {
                    self.terminate_policy_run_static(
                        "JUMP_LIMIT",
                        "failed",
                        Some("JUMP_LIMIT"),
                        Some("First live policy step exceeds max_first_step_rad"),
                    );
                    return Err(Refusal::code("JUMP_LIMIT"));
                }
            }
        } else if let Some(step_cap) = max_step {
            let dof = self.config.dof;
            let violated = self
                .policy_run
                .as_ref()
                .and_then(|r| r.last_live_target.as_deref())
                .is_some_and(|prev| {
                    (0..dof).any(|i| (targets[i] - prev[i]).abs() > step_cap)
                });
            if violated {
                self.terminate_policy_run_static(
                    "STEP_LIMIT",
                    "failed",
                    Some("STEP_LIMIT"),
                    Some("Live policy step exceeds max_step_rad"),
                );
                return Err(Refusal::code("STEP_LIMIT"));
            }
        }

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
        let j7_gate = self.sample_pos_force_j7_gate();
        if self.config.gripper_index().is_some() && j7_gate.stale {
            self.terminate_policy_run_static(
                "J7_FEEDBACK_STALE",
                "failed",
                Some("J7_FEEDBACK_STALE"),
                Some("Gripper (J7) position-force feedback is stale (> 100 ms)"),
            );
            return Err(Refusal::code("J7_FEEDBACK_STALE"));
        }

        if let Err(refusal) = self.validate_and_filter_servo_targets_rt(targets, epoch_id, j7_gate) {
            let (reason, state, code, msg) = match refusal.code {
                "ESTOP_ACTIVE" => ("ESTOP", "failed", "ESTOP_ACTIVE", "E-stop latched"),
                "FAULT_ACTIVE" => ("FAULT", "failed", "FAULT_ACTIVE", "Fault active during policy step"),
                "COLLISION_HOLD" => ("COLLISION_HOLD", "cancelled", "COLLISION_HOLD", "Collision hold active"),
                "SOFT_LIMIT_EXCEEDED" => ("SOFT_LIMIT_EXCEEDED", "failed", "SOFT_LIMIT_EXCEEDED", "Policy target exceeded joint soft limits"),
                "WORKSPACE_VIOLATION" => ("WORKSPACE_VIOLATION", "failed", "WORKSPACE_VIOLATION", "Policy target violated workspace bounds"),
                "STATIC_LOAD_LIMIT" => ("STATIC_LOAD_LIMIT", "failed", "STATIC_LOAD_LIMIT", crate::power::STATIC_LOAD_LIMIT_MSG),
                "NOT_ARMED" => ("NOT_ARMED", "failed", "NOT_ARMED", "Arm is not armed"),
                "COMMISSIONING_REQUIRED" => ("COMMISSIONING_REQUIRED", "failed", "COMMISSIONING_REQUIRED", "Tool commissioning required"),
                _ => ("SAFETY_REJECTED", "failed", "SAFETY_ERROR", "Policy target rejected by safety filter"),
            };
            self.terminate_policy_run_static(reason, state, Some(code), Some(msg));
            return Err(refusal);
        }

        // All checks passed: install live target, update run state using preallocated spares,
        // and stamp `j7_admitted_stamp` ONLY when this accepted fresh frame moves J7.
        self.last_motion_frame = now;
        self.release_gripper_stall_on_reverse_with_gate(targets, j7_gate);
        let _ = self.safety.mark_motion_started();

        copy_into(&mut self.target_positions, targets);
        self.set_servo_target(targets);
        self.servo_stream_active = true;
        self.servo_settled_ticks.fill(0);

        let mut live_spare = std::mem::take(&mut self.policy_live_vec_spare);
        let mut shadow_spare = std::mem::take(&mut self.policy_shadow_vec_spare);
        let mut obs_spare = self.last_policy_obs_spare.take();
        let mut obs_vec_spare = std::mem::take(&mut self.last_policy_obs_vec_spare);
        if let Some(run) = self.policy_run.as_mut().filter(|r| r.action_id == action_id) {
            run.steps += 1;
            run.last_input_age_ms = input_age_ms;
            run.last_vision_age_ms = vision_age_ms;
            run.vision_frame_id = vision_frame_id;
            run.live_target_auth_gen = Some(frame.policy_generation);
            match run.last_live_target.as_mut() {
                Some(v) => copy_into(v, targets),
                None => {
                    copy_into(&mut live_spare, targets);
                    run.last_live_target = Some(std::mem::take(&mut live_spare));
                }
            }
            match run.last_shadow_target.as_mut() {
                Some(v) => copy_into(v, targets),
                None => {
                    copy_into(&mut shadow_spare, targets);
                    run.last_shadow_target = Some(std::mem::take(&mut shadow_spare));
                }
            }
            if self.last_policy_obs.is_none() {
                self.last_policy_obs = obs_spare.take();
            }
            let dst = self.last_policy_obs.get_or_insert_with(crate::model::PolicyObservation::preallocated_rt);
            run.to_observation_into_with_vec_spare(true, dst, &mut obs_vec_spare);
        }
        if live_spare.capacity() > self.policy_live_vec_spare.capacity() {
            self.policy_live_vec_spare = live_spare;
        }
        if shadow_spare.capacity() > self.policy_shadow_vec_spare.capacity() {
            self.policy_shadow_vec_spare = shadow_spare;
        }
        if obs_spare.is_some() && self.last_policy_obs_spare.is_none() {
            self.last_policy_obs_spare = obs_spare;
        }
        self.last_policy_obs_vec_spare = obs_vec_spare;

        if !j7_gate.stale && self.frame_moves_j7(targets, prev_cmd_j7, prev_tgt_j7) {
            self.j7_admitted_stamp = Some(crate::driver::PosForceIntentStamp {
                stop_gen: frame.authority.stop_generation,
                estop_epoch: frame.authority.epoch,
                lease_gen: frame.authority.lease_generation,
            });
        }
        Ok(())
    }

    /// Drops the live policy's `servo_target`, recycles target buffers, clears
    /// `live_target_auth_gen` and `j7_admitted_stamp`, and snaps `target_positions` to
    /// `commanded_positions` with 0 allocs and 0 frees, WITHOUT bumping `run.authorization`
    /// (used at Step 4 `POLICY_REVOKED` and Step 5 output-boundary recheck so a newer already-
    /// bumped generation `G + 1` is not spuriously double-revoked).
    pub(crate) fn brake_live_policy_in_place(&mut self, action_id: u64) {
        let Some(run) = self
            .policy_run
            .as_mut()
            .filter(|r| r.action_id == action_id && r.mode == crate::policy_runtime::PolicyMode::Live)
        else {
            return;
        };
        let reclaimed_live = run.last_live_target.take();
        run.live_target_auth_gen = None;
        if let Some(mut v) = reclaimed_live {
            if v.capacity() > 0 {
                v.clear();
                if self.policy_live_vec_spare.capacity() == 0 {
                    self.policy_live_vec_spare = v;
                } else if let Some(spare) = self
                    .policy_run_spare
                    .as_mut()
                    .filter(|s| s.last_live_target.is_none())
                {
                    spare.last_live_target = Some(v);
                }
            }
        }
        self.release_servo_target();
        self.servo_settled_ticks.fill(0);
        self.clear_servo_ws_min_depths_rt();
        self.gripper_stream_target = None;
        self.gripper_stream_contact = None;
        self.gripper_stream_contact_detector.reset();
        self.gripper_stream_lease = None;
        self.j7_admitted_stamp = None;
        copy_into(&mut self.target_positions, &self.commanded_positions);
    }

    /// Zero-allocation Step-5 output-boundary deadline and revocation recheck: if `run.deadline`
    /// (`max_seconds`) elapsed, terminates cleanly as `MAX_SECONDS` (`done`); otherwise if
    /// `PolicyAuthorization` changed after the installed live target was accepted, immediately
    /// drops both `servo_target` and policy-owned `gripper_stream_*` state before any J1-J7
    /// output advances.
    #[inline]
    pub(crate) fn enforce_policy_deadline_and_auth_rt(&mut self, now: Instant) {
        if self
            .policy_run
            .as_ref()
            .is_some_and(|r| r.rt_owned && r.deadline.is_some_and(|dl| now >= dl))
        {
            self.terminate_policy_run_static("MAX_SECONDS", "done", None, None);
            return;
        }
        self.recheck_live_policy_authorization_rt();
    }

    #[inline]
    pub(crate) fn recheck_live_policy_authorization_rt(&mut self) {
        if let Some(aid) = self
            .policy_run
            .as_ref()
            .filter(|r| {
                r.mode == crate::policy_runtime::PolicyMode::Live
                    && r.live_target_auth_gen
                        .is_some_and(|g| r.authorization.current() != g)
            })
            .map(|r| r.action_id)
        {
            self.brake_live_policy_in_place(aid);
        }
    }

    /// Vision hold of a live run: drop its servo target so the control loop brakes now
    /// (instead of tracking the last target until the motion-frame timeout). The next target
    /// is checked against the measured pose with the first-step limit.
    pub fn hold_policy_run(&mut self, action_id: u64) {
        let Some(run) = self.policy_run.as_ref().filter(|r| r.action_id == action_id && r.mode == crate::policy_runtime::PolicyMode::Live) else { return; };
        // a target computed before the hold and still waiting in the mailbox is not applied
        run.authorization.revoke();
        self.brake_live_policy_in_place(action_id);
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
    /// preallocated `action_msg_spares` buffer (bounded, 0 reallocs) and terminates with 0 alloc/free.
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
            push_str_bounded(&mut buf, text);
            buf
        });
        self.terminate_policy_run_with_stats(stop_reason, action_state, action_code, msg, None);
    }

    /// Executor-only zero-alloc teardown shared by termination (`reset_hold_target = true`) and
    /// supersession (`reset_hold_target = false`). Never allocates, never frees, seals diagnostics
    /// BEFORE Step 6 publish, and queues post-publication stream close.
    pub(crate) fn teardown_rt_policy_run(
        &mut self,
        mut run: crate::policy_runtime::PolicyRun,
        stop_reason: &str,
        reset_hold_target: bool,
        now: Instant,
    ) {
        if run.rt_owned {
            run.authorization.unbind_action(run.action_id);
        } else {
            run.authorization.revoke();
        }
        self.atomic_diag_sealed_policy_action_id
            .fetch_max(run.action_id, Ordering::Release);
        self.record_pending_policy_close(run.action_id);
        if run.mode == crate::policy_runtime::PolicyMode::Live {
            self.release_servo_target();
            self.servo_settled_ticks.fill(0);
            self.clear_servo_ws_min_depths_rt();
            self.gripper_stream_target = None;
            self.gripper_stream_contact = None;
            self.gripper_stream_contact_detector.reset();
            self.gripper_stream_lease = None;
            if reset_hold_target {
                self.j7_admitted_stamp = None;
                copy_into(&mut self.target_positions, &self.commanded_positions);
            }
        }
        if self.last_policy_obs.is_none() {
            self.last_policy_obs = self.last_policy_obs_spare.take();
        }
        let dst = self
            .last_policy_obs
            .get_or_insert_with(crate::model::PolicyObservation::preallocated_rt);
        run.to_observation_into_with_vec_spare(false, dst, &mut self.last_policy_obs_vec_spare);
        dst.stop_reason = Some(crate::model::HolderLabel::new(stop_reason));

        run.live_target_auth_gen = None;
        run.rt_gate_binding = None;
        run.load_token = 0;
        run.cancel_slot = 0;
        run.deadline = None;
        #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
        self.policy_cancel_slots.set_active_run_token(0);
        if run.rt_owned && self.policy_run_spare.is_none() {
            if self.policy_live_vec_spare.capacity() == 0 {
                if let Some(mut v) = run.last_live_target.take() {
                    v.clear();
                    self.policy_live_vec_spare = v;
                }
            }
            if self.policy_shadow_vec_spare.capacity() == 0 {
                if let Some(mut v) = run.last_shadow_target.take() {
                    v.clear();
                    self.policy_shadow_vec_spare = v;
                }
            }
            self.policy_run_spare = Some(run);
        } else {
            // Leave heap target buffers attached to `run` so `retire_policy_run` frees them
            // off-thread with 0 frees on `armd-control`.
            self.retire_policy_run(run);
        }
        self.reconcile_target_stream_authority(now);
    }

    pub(crate) fn terminate_policy_run_with_stats(
        &mut self,
        stop_reason: &str,
        action_state: &str,
        action_code: Option<&str>,
        action_message: Option<String>,
        final_stats: Option<crate::motion::PolicyStats>,
    ) {
        let Some(mut run) = self.policy_run.take() else {
            return;
        };
        let action_id = run.action_id;
        if self.rt_executor_owned {
            let final_steps = run.steps;
            // Only write `policy_stats` when explicitly supplied (e.g. `EndPolicy::{Finish, Fail}`);
            // leave `None` on RT-initiated termination so `StateHub` attaches the frozen runner stats.
            if let Some(stats) = final_stats {
                self.set_action_policy_stats(action_id, stats);
            }
            self.teardown_rt_policy_run(run, stop_reason, true, Instant::now());
            self.finish_action(
                action_id,
                action_state,
                action_code,
                action_message,
                None,
                None,
                None,
            );
            #[cfg(not(target_arch = "wasm32"))]
            self.push_rt_policy_terminal(
                action_id,
                final_steps,
                stop_reason,
                action_state,
                final_stats.unwrap_or_default(),
            );
            return;
        }
        run.authorization.revoke();
        self.record_pending_policy_close(action_id);
        self.end_policy_stream(action_id);
        let stats_report = final_stats.unwrap_or_else(|| run.step_stats.report());
        self.set_action_policy_stats(action_id, stats_report);
        if run.mode == crate::policy_runtime::PolicyMode::Live {
            self.release_servo_target();
            self.servo_settled_ticks.fill(0);
            self.clear_servo_ws_min_depths_rt();
            self.j7_admitted_stamp = None;
            copy_into(&mut self.target_positions, &self.commanded_positions);
        }
        if self.last_policy_obs.is_none() {
            self.last_policy_obs = self.last_policy_obs_spare.take();
        }
        if let Some(obs) = self.last_policy_obs.as_mut() {
            run.to_observation_into_with_vec_spare(false, obs, &mut self.last_policy_obs_vec_spare);
            obs.stop_reason = Some(crate::model::HolderLabel::new(stop_reason));
        } else {
            let mut obs = run.to_observation(false);
            obs.stop_reason = Some(crate::model::HolderLabel::new(stop_reason));
            self.last_policy_obs = Some(obs);
        }
        self.finish_action(
            action_id,
            action_state,
            action_code,
            action_message,
            None,
            None,
            None,
        );
        if self.defer_zero_check_persist {
            // Defer supervised_first_run ledger I/O (open/write/sync_all) off the Legacy control
            // release lock without allocating on the control path (B4).
            let needs_qual = run.mode == crate::policy_runtime::PolicyMode::Live
                && !self.policy_live_allowed()
                && self.policy_ledger_path.is_some()
                && run.verified_release_gate.as_ref().is_some_and(|g| {
                    g.authorization_tier() == crate::policy_runtime::AuthorizationTier::SupervisedFirstRun
                });
            if needs_qual {
                self.pending_policy_qual_persist = Some((
                    run,
                    crate::model::HolderLabel::new(stop_reason),
                    crate::model::HolderLabel::new(action_state),
                ));
            } else {
                self.retire_policy_run(run);
            }
        } else {
            run.stop_reason = Some(stop_reason.to_string());
            self.record_policy_qualification(&run, stop_reason, action_state);
            self.retire_policy_run(run);
        }
    }

    pub(crate) fn terminate_policy_run(
        &mut self,
        stop_reason: &str,
        action_state: &str,
        action_code: Option<&str>,
        action_message: Option<String>,
    ) {
        self.terminate_policy_run_with_stats(stop_reason, action_state, action_code, action_message, None);
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
            SafetyError::TorqueOffWouldDrop { .. } => "TORQUE_OFF_WOULD_DROP",
            SafetyError::ClearFaultRejected(_) => "CLEAR_FAULT_REJECTED",
            SafetyError::EStopActive => "ESTOP_ACTIVE",
            SafetyError::CollisionHold(_) => "COLLISION_HOLD",
            SafetyError::CommissioningRequired(_) => "COMMISSIONING_REQUIRED",
            SafetyError::MotionBusy => "MOTION_BUSY",
            SafetyError::WorkspaceViolation { .. } => "WORKSPACE_VIOLATION",
            SafetyError::StaticLoadLimit => "STATIC_LOAD_LIMIT",
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
    pub(crate) fn reject_fields(code: &str) -> (u16, &str, Option<&'static str>) {
        match code {
            "ESTOP_ACTIVE" => (409, code, Some("EStop")),
            "FAULT_ACTIVE" => (409, code, Some("Fault")),
            "ARM_NOT_AT_RESTING_POSE" => (409, "ArmNotAtRestingPose", Some("Armed")),
            "MOTION_FORBIDDEN_IN_FAULT_STATE" => (409, "MotionForbiddenInFaultState", Some("Fault")),
            "TORQUE_OFF_WOULD_DROP" => (409, code, None),
            "CLEAR_FAULT_REJECTED" => (409, "ClearFaultRejected", Some("Fault")),
            "WORKSPACE_VIOLATION" | "JUMP_LIMIT" | "STEP_LIMIT" | "STATIC_LOAD_LIMIT" => (422, code, None),
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
    /// As the legacy loop answers the same conditions: a stop-preempted frame with `servo_reject`
    /// (`code: "STOPPED"`, `reason: "stopped"` matching `api/ws/servo.rs`), an unauthorized frame
    /// (lease lost, stream not authorized, no lease) with `lease_lost` (with the loss reason when
    /// known), a stale epoch with `epoch_sync`, a calibration with `servo_reject`
    /// `reason: calibration_busy`, other refusals with `servo_reject` (and `current_revision`
    /// when known). Messages match the legacy path.
    pub fn servo_outcome_reply(refusal: &Refusal, ctx: &ReplyCtx) -> Option<Value> {
        Some(match refusal.code {
            "STOPPED" => {
                let mut v = servo_reject("STOPPED", "Stopped before motion began", ctx.epoch_id);
                v["reason"] = json!("stopped");
                v
            }
            "LEASE_LOST" | "NO_LEASE" | "STREAM_NOT_AUTHORIZED" => {
                let mut v = json!({"type":"lease_lost", "generation":ctx.lease_generation});
                if let (RefusalDetail::LeaseLoss { reason }, true) = (refusal.detail, ctx.lease_generation != 0) { v["reason"] = json!(reason); }
                v
            }
            "EPOCH_MISMATCH" => json!({"type":"epoch_sync", "epoch_id":ctx.epoch_id, "safety_state":ctx.safety_state}),
            "CALIBRATION_BUSY" => json!({"type":"servo_reject", "reason":"calibration_busy"}),
            "STATIC_LOAD_LIMIT" => servo_reject("STATIC_LOAD_LIMIT", crate::power::STATIC_LOAD_LIMIT_MSG, ctx.epoch_id),
            code => {
                let mut v = servo_reject(code, "servo target refused when the control loop applied it", ctx.epoch_id);
                if let RefusalDetail::CurrentRevision(rev) = refusal.detail {
                    v["current_revision"] = json!(rev);
                } else {
                    crate::chain::attach_workspace_refusal_fields(&mut v, refusal.detail, "servo_target");
                }
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

        /// Synchronize sequence state across an owner handoff (legacy <-> executor) on the same lease.
        pub fn sync_from(&mut self, generation: u64, last_sequence: Option<u64>) {
            self.lease_generation = generation;
            self.last_sequence = last_sequence;
        }

        /// Advance `last_sequence` after passing sequence validation on a malformed-payload frame.
        pub fn advance_sequence(&mut self, sequence: u64) {
            self.last_sequence = Some(sequence);
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
        /// Monotonic high-water mark of closed servo lease generations: late submissions for any
        /// non-zero generation `<= closed_servo_gen` are rejected immediately off-thread.
        closed_servo_gen: u64,
        /// Non-zero servo lease generations currently open in `writer` (bounded by `MAX_STREAMS`).
        open_servo_gens: Vec<u64>,
        /// Monotonic high-water mark of closed policy action IDs: durable late-submission fence.
        closed_policy_action_id: u64,
        /// Non-zero policy action IDs currently open in `writer` (bounded by `MAX_STREAMS`).
        open_policy_actions: Vec<u64>,
    }

    impl TargetPort {
        pub fn new(writer: TargetWriter, arm: ArmId, boot: BootId) -> Self {
            Self {
                writer,
                arm,
                boot,
                seq: 0,
                closed_servo_gen: 0,
                open_servo_gens: Vec::with_capacity(crate::rt_core::mailbox::MAX_STREAMS),
                closed_policy_action_id: 0,
                open_policy_actions: Vec::with_capacity(crate::rt_core::mailbox::MAX_STREAMS),
            }
        }

        /// Frames replaced before consumption.
        pub fn coalesced(&self) -> u64 { self.writer.coalesced() }

        /// Sequence number assigned to the most recently built frame (0 before first submission).
        pub fn last_seq(&self) -> u64 { self.seq }

        /// Highest lease generation permanently closed on this port (0 if none closed yet).
        pub fn closed_servo_generation(&self) -> u64 { self.closed_servo_gen }

        /// True when `lease_generation` (non-zero) has been closed via [`Self::close_servo`].
        pub fn is_servo_closed(&self, lease_generation: u64) -> bool {
            lease_generation != 0 && lease_generation <= self.closed_servo_gen
        }

        /// Highest policy action ID permanently closed on this port (0 if none closed yet).
        pub fn closed_policy_action_id(&self) -> u64 { self.closed_policy_action_id }

        /// True when `action_id` (non-zero) has been closed via [`Self::close_policy`].
        pub fn is_policy_closed(&self, action_id: u64) -> bool {
            action_id != 0 && action_id <= self.closed_policy_action_id
        }

        /// Underlying target writer (for low-level mailbox access / tests).
        pub fn writer(&self) -> &TargetWriter { &self.writer }

        /// Mutable access to the underlying target writer (for low-level mailbox access / tests).
        pub fn writer_mut(&mut self) -> &mut TargetWriter { &mut self.writer }

        /// Drains the RT's outcomes of consumed frames.
        pub fn outcomes(&mut self, each: impl FnMut(TargetOutcome)) -> usize { self.writer.outcomes(each) }

        /// The connection holding lease `lease_generation` ended (release, preemption, expiry, or
        /// disconnect): forget its stream (and any older open servo streams) in the mailbox writer
        /// and fence late submissions from that generation.
        pub fn close_servo(&mut self, lease_generation: u64) {
            if lease_generation == 0 {
                return;
            }
            self.closed_servo_gen = self.closed_servo_gen.max(lease_generation);
            self.writer.close_stream(servo_stream(lease_generation));
            let closed = self.closed_servo_gen;
            let writer = &mut self.writer;
            self.open_servo_gens.retain(|&g| {
                if g <= closed {
                    writer.close_stream(servo_stream(g));
                    false
                } else {
                    true
                }
            });
        }

        /// Policy run `action_id` ended: forget its stream (and any older open policy streams)
        /// and fence late submissions for `<= action_id`.
        pub fn close_policy(&mut self, action_id: u64) {
            if action_id == 0 {
                return;
            }
            self.closed_policy_action_id = self.closed_policy_action_id.max(action_id);
            self.writer.close_stream(policy_stream(action_id));
            let closed = self.closed_policy_action_id;
            let writer = &mut self.writer;
            self.open_policy_actions.retain(|&id| {
                if id <= closed {
                    writer.close_stream(policy_stream(id));
                    false
                } else {
                    true
                }
            });
        }

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
        /// Rejects late submissions from closed lease generations before touching sequence state.
        /// Sequence rules as the legacy loop: a missing `sequence_id`, a repeat or a jump of
        /// more than 1000 (wrapping) is refused; the sequence advances once it passes them.
        pub fn submit_servo(&mut self, stream: &mut ServoStream, msg: &ServoMessage<'_>) -> Option<Value> {
            let lease_gen = msg.authority.lease_generation;
            if self.is_servo_closed(lease_gen) {
                return Some(json!({"type":"lease_lost", "generation": lease_gen}));
            }
            if msg.calibration_busy { return Some(json!({"type":"servo_reject", "reason":"calibration_busy"})); }
            let Some(sequence) = msg.sequence_id else { return Some(json!({"type":"servo_reject", "reason":"missing_sequence_id"})); };
            if stream.last_sequence.is_some_and(|last| { let delta = sequence.wrapping_sub(last); delta == 0 || delta > 1000 }) {
                return Some(json!({"type":"servo_reject", "reason":"invalid_sequence_id"}));
            }
            stream.last_sequence = Some(sequence);
            let epoch = msg.authority.epoch;
            let stream_id = servo_stream(lease_gen);
            let Some(frame) = self.frame(stream_id, msg.authority, 0, msg.positions, msg.received_at, None, None,
                msg.received_at + SERVO_TARGET_TTL, true) else {
                return Some(servo_reject("SAFETY_ERROR", "servo target is empty, has non-finite values or more joints than the arm", epoch));
            };
            match self.writer.submit(frame) {
                Submitted::Accepted | Submitted::Coalesced { .. } => {
                    if lease_gen != 0 && !self.open_servo_gens.contains(&lease_gen) {
                        self.open_servo_gens.push(lease_gen);
                    }
                    None
                }
                Submitted::Busy => Some(servo_reject("MOTION_BUSY", "a policy target is waiting to be applied", epoch)),
                Submitted::StreamsFull => Some(servo_reject("STREAMS_FULL", "too many open target streams", epoch)),
                Submitted::OutOfOrder => Some(json!({"type":"servo_reject", "reason":"invalid_sequence_id"})),
            }
        }

        /// Submits one policy target (not replaceable; deadline: input time + twice the run's
        /// input age bound). `Busy`: the previous target was not consumed yet; the runner counts the
        /// step as skipped. Rejects late submissions for closed action IDs immediately off-thread.
        pub fn submit_policy(&mut self, t: &PolicyTarget<'_>) -> Submitted {
            self.submit_policy_with_validity(
                t,
                crate::rt_core::mailbox::PolicyValidityBounds::default(),
            )
        }

        /// Submits one policy target together with atomic [`PolicyValidityBounds`] (tracking both
        /// `Accepted` and `Coalesced` publications so policy streams that overwrite an unconsumed
        /// replaceable servo frame are never left untracked).
        pub fn submit_policy_with_validity(
            &mut self,
            t: &PolicyTarget<'_>,
            validity: crate::rt_core::mailbox::PolicyValidityBounds,
        ) -> Submitted {
            if self.is_policy_closed(t.action_id) {
                return Submitted::OutOfOrder;
            }
            // after the input age bound, so an aged input ends the run as `STALE_INPUT`
            let deadline = t.input_at + 2 * t.stamp.max_input_age;
            match self.frame(policy_stream(t.action_id), t.stamp.authority, t.stamp.authorization.max(1), t.targets, t.input_at,
                t.vision_at, t.vision_frame_id, deadline, false) {
                Some(frame) => {
                    let sub = self.writer.submit_with_validity(frame, validity);
                    if matches!(sub, Submitted::Accepted | Submitted::Coalesced { .. })
                        && t.action_id != 0
                        && !self.open_policy_actions.contains(&t.action_id)
                    {
                        self.open_policy_actions.push(t.action_id);
                    }
                    sub
                }
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

        /// Single centralized owner target-stream authority selector (Live policy stream, Shadow /
        /// loading revoked stream, or valid current-lease-only servo stream). Fails closed
        /// whenever no valid unfenced open lease exists or admission refuses.
        pub(crate) fn reconcile_target_stream_authority(&mut self, now: Instant) {
            #[cfg(feature = "policy-runtime")]
            if let Some(run) = self.policy_run.as_ref() {
                if run.mode == crate::policy_runtime::PolicyMode::Live || !run.rt_owned {
                    let input_known = self.has_policy_input_age_range();
                    let run = self.policy_run.as_ref().expect("checked");
                    self.core.authorize_stream(TargetAuthority {
                        stream: Some(policy_stream(run.action_id)),
                        policy_generation: run.authorization.current(),
                        max_input_age: input_known.then(|| Duration::from_millis(run.max_input_age_ms)),
                        max_vision_age: run.max_vision_age_ms.map(Duration::from_millis),
                    });
                } else {
                    // Executor Shadow run: zero mailbox authority; servo frames still get MOTION_BUSY.
                    self.core.revoke_stream();
                }
                return;
            }
            #[cfg(feature = "policy-runtime")]
            if self.policy_loading {
                self.core.revoke_stream();
                return;
            }
            let g = if self.pending_disconnect_hold.load(std::sync::atomic::Ordering::SeqCst) > 0 {
                0
            } else {
                self.active_lease_gen_at(now)
            };
            if g == 0 {
                self.core.revoke_stream();
                return;
            }
            let stamp = crate::supervisor::AdmissionStamp::for_rt_internal(
                self.current_stop_gen(),
                self.safety.epoch_id(),
                g,
            );
            match self.check_admission(stamp) {
                Ok(admitted) => {
                    if Self::authorize_servo_stream(admitted, g).is_err() {
                        self.core.revoke_stream();
                    }
                }
                Err(_) => {
                    self.core.revoke_stream();
                }
            }
        }

        /// Authorizes the committed lease generation's servo stream through
        /// [`Self::authorize_servo_stream`] (zero allocations, zero locks), preserving any active
        /// policy run (Live or Shadow) or active load reservation. Fails closed (revokes the
        /// stream) if admission cannot currently pass.
        pub(crate) fn authorize_servo_stream_for_committed_lease(&mut self, lease_generation: u64) {
            if lease_generation == 0 {
                self.core.revoke_stream();
                return;
            }
            #[cfg(feature = "policy-runtime")]
            if self.policy_run.is_some() || self.policy_loading {
                return;
            }
            let stamp = crate::supervisor::AdmissionStamp::for_rt_internal(
                self.current_stop_gen(),
                self.safety.epoch_id(),
                lease_generation,
            );
            match self.check_admission(stamp) {
                Ok(admitted) => {
                    if Self::authorize_servo_stream(admitted, lease_generation).is_err() {
                        self.core.revoke_stream();
                    }
                }
                Err(_) => {
                    self.core.revoke_stream();
                }
            }
        }

        /// After a policy run of `action_id`: restore the valid current lease holder's servo
        /// stream (revoking when lease was lost, expired, fenced, or disconnected, or under E-stop).
        #[cfg(feature = "policy-runtime")]
        pub(in crate::supervisor) fn end_policy_stream(&mut self, action_id: u64) {
            let was_authorized = self.core.target_authority().stream == Some(policy_stream(action_id));
            if !was_authorized && self.core.target_authority().stream.is_some() {
                return;
            }
            self.reconcile_target_stream_authority(Instant::now());
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
            self.consume_target_frame_with_validity(
                frame,
                crate::rt_core::mailbox::PolicyValidityBounds::default(),
                ctx,
            )
        }

        /// Applies one target together with its atomic [`PolicyValidityBounds`] at `ctx`.
        pub fn consume_target_frame_with_validity(
            &mut self,
            frame: &TargetFrame,
            validity: crate::rt_core::mailbox::PolicyValidityBounds,
            ctx: &CommitCtx,
        ) -> Result<(), Refusal> {
            #[cfg(feature = "policy-runtime")]
            self.sync_policy_target_authority();
            // a target is checked against the active workspace when applied; it carries no plan revision
            let current = Authority { config_revision: frame.authority.config_revision, ..CoreOps::authority(self, ctx) };
            let policy = frame.policy_generation != 0;
            let check_res = if policy {
                frame.check_policy_with_validity(
                    ctx.arm,
                    ctx.boot,
                    ctx.now,
                    &current,
                    &self.core.target_authority(),
                    validity,
                )
            } else {
                frame.check(ctx.arm, ctx.boot, ctx.now, &current, &self.core.target_authority())
            };
            if let Some(code) = check_res {
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
            #[cfg(not(target_arch = "wasm32"))]
            if self.sessions.blocks_ordinary_output(None) { return Err(Refusal::code("MOTION_BUSY")); }
            if policy { return self.consume_policy_target(frame, ctx.now); }
            if self.calibration_busy { return Err(Refusal::code("CALIBRATION_BUSY")); }
            self.sync_stop_generation(ctx.stop_generation);
            let stamp = crate::supervisor::AdmissionStamp::for_rt_internal(
                frame.authority.stop_generation,
                frame.authority.epoch,
                frame.authority.lease_generation,
            );
            let admitted = self.check_admission(stamp).map_err(|e| Refusal::code(safety_code(&e)))?;
            Self::servo_slice_rt(admitted, frame.positions(), frame.authority.epoch)
        }

        #[cfg(not(feature = "policy-runtime"))]
        pub(crate) fn policy_target_refused(&mut self, _code: &str, _frame: &TargetFrame) {}

        /// A target of the active run was refused by the consumption check: hold or end the
        /// run where the legacy commit holds or ends it (zero heap allocations / frees on RT).
        #[cfg(feature = "policy-runtime")]
        pub(crate) fn policy_target_refused(&mut self, code: &str, frame: &TargetFrame) {
            let Some(run) = self.policy_run.as_ref().filter(|r| frame.stream == policy_stream(r.action_id)) else { return; };
            let (action_id, hold_on_stale) = (run.action_id, run.vision_stale_hold);
            match code {
                // revoked after the target was computed (vision hold or invalidation): brake now
                // without double-revoking the newer generation.
                "POLICY_REVOKED" => self.brake_live_policy_in_place(action_id),
                "DEADLINE_EXPIRED" => self.hold_policy_run(action_id),
                "HANDSHAKE_EXPIRED" => self.terminate_policy_run_static(
                    "HANDSHAKE_EXPIRED",
                    "failed",
                    Some("HANDSHAKE_EXPIRED"),
                    Some("Vision clock handshake expired before the target was applied"),
                ),
                "VISION_STALE" if hold_on_stale => self.hold_policy_run(action_id),
                "VISION_STALE" => self.terminate_policy_run_static(
                    "VISION_STALE",
                    "failed",
                    Some("VISION_STALE"),
                    Some("Vision sample became stale before the target was applied"),
                ),
                "INPUT_STALE" => self.terminate_policy_run_static(
                    "STALE_INPUT",
                    "failed",
                    Some("STALE_INPUT"),
                    Some("Live policy target computed from stale joint positions (aged while buffered)"),
                ),
                "LEASE_LOST" => self.terminate_policy_run_static(
                    "LEASE_LOST",
                    "cancelled",
                    Some("LEASE_LOST"),
                    Some("Control lease lost or expired"),
                ),
                "EPOCH_MISMATCH" => {
                    if self.estop_pending() || matches!(self.safety.state(), SafetyState::EStop) {
                        self.terminate_policy_run_static("ESTOP", "failed", Some("ESTOP_ACTIVE"), Some("E-stop latched"));
                    } else if let SafetyState::Fault(ref r) = self.safety.state() {
                        let reason = if matches!(r, crate::safety::FaultReason::WatchdogTimeout) { "WATCHDOG_TIMEOUT" } else { "FAULT" };
                        self.terminate_policy_run_static(reason, "failed", Some(reason), Some("Fault active during policy run"));
                    } else {
                        self.terminate_policy_run_static("STOPPED", "cancelled", Some("EPOCH_MISMATCH"), Some("Epoch changed during policy run"));
                    }
                }
                _ => {}
            }
        }

        #[cfg(not(feature = "policy-runtime"))]
        fn consume_policy_target(&mut self, _frame: &TargetFrame, _now: Instant) -> Result<(), Refusal> { Err(Refusal::code("POLICY_NOT_ACTIVE")) }

        /// A policy frame that passed the consumption check: zero-alloc `policy_step_slice_rt`
        /// (holds, gates, input age, jump/step caps, single-sample J7 gate, servo checks).
        #[cfg(feature = "policy-runtime")]
        fn consume_policy_target(&mut self, frame: &TargetFrame, now: Instant) -> Result<(), Refusal> {
            let ms = |t: Instant| now.saturating_duration_since(t).as_millis().min(u128::from(u64::MAX)) as u64;
            let Some((action_id, deadline)) = self
                .policy_run
                .as_ref()
                .filter(|r| frame.stream == policy_stream(r.action_id))
                .map(|r| (r.action_id, r.deadline))
            else {
                return Err(Refusal::code("POLICY_NOT_ACTIVE"));
            };
            if deadline.is_some_and(|dl| now >= dl) {
                self.terminate_policy_run_static("MAX_SECONDS", "done", None, None);
                return Err(Refusal::code("POLICY_NOT_ACTIVE"));
            }
            let input_age_ms = self.has_policy_input_age_range().then(|| ms(frame.input_at));
            self.policy_step_slice_rt(action_id, frame, input_age_ms, frame.vision_at.map(ms), frame.vision_frame_id, now)
        }
    }
}

#[cfg(test)]
#[path = "streams_target_tests.rs"]
mod target_tests;
