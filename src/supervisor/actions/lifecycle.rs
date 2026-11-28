//! Action lifecycle, table-capture stillness, gripper zero-check sessions, and action preconditions.

#[allow(unused_imports)]
use super::super::*;
#[allow(unused_imports)]
use crate::macro_action::{ChildTicket, ContactGuard, ContactWatch};

impl SupervisorInner {
    /// True on a real/physical driver (non-sim, non-mock) where missing jaw calibration
    /// curves must not fall back to `JawOpening::sim_default()`.
    pub fn requires_real_jaw_curve(&self) -> bool {
        self.driver.is_some()
            && matches!(self.follower.as_deref(), Some(f) if f != "sim" && f != "mock")
    }

    /// `max_w` (a grasp setup's `max_grasp_width_m`) against the gripper's jaw curve; None
    /// without a gripper policy.
    pub fn grasp_width_check(&self, max_w: f64) -> Option<crate::model::GraspWidthCheck> {
        let gp = self.config.gripper_policy.as_ref()?;
        Some(crate::model::check_grasp_width(gp, max_w, self.requires_real_jaw_curve()))
    }

    pub(crate) fn reset_table_stillness(&mut self) {
        self.table_still_first_rx_instants.clear();
        self.table_still_last_rx_instants.clear();
        self.table_still_anchor_positions.clear();
    }

    /// Zero-allocation stillness check backing [`Self::check_table_capture_stillness`].
    pub(crate) fn check_table_capture_stillness_static(&self) -> Result<(), &'static str> {
        if !self.feedback_numeric_valid {
            return Err("table_z_from_tcp requires valid joint feedback");
        }
        if self.driver.is_some() && !self.hardware_feedback_fresh(Duration::from_millis(250)) {
            return Err("table_z_from_tcp requires fresh joint feedback");
        }
        if self.active_trajectory.is_some()
            || self.servo_target.is_some()
            || self.commanded_velocities.iter().any(|v| v.abs() > 1e-3)
        {
            return Err("table_z_from_tcp requires the arm to be stationary");
        }
        if self.measured_velocities.len() != self.config.dof
            || self.measured_velocities.iter().any(|v| !v.is_finite() || v.abs() > 0.02)
        {
            return Err("table_z_from_tcp requires measured joint velocities <= 0.02 rad/s");
        }
        if self.driver.is_some() || self.still_window_ever_moved {
            let settled = if self.driver.is_some() {
                self.table_still_first_rx_instants.len() == self.config.dof
                    && self.table_still_last_rx_instants.len() == self.config.dof
                    && self.table_still_first_rx_instants
                        .iter()
                        .zip(&self.table_still_last_rx_instants)
                        .all(|(first, last)| {
                            last.checked_duration_since(*first)
                                .is_some_and(|d| d >= Duration::from_millis(200))
                        })
                    && self.table_still_anchor_positions.len() == self.measured_positions.len()
                    && self.measured_positions.iter().zip(&self.table_still_anchor_positions)
                        .all(|(m, a)| m.is_finite() && (m - a).abs() <= UNLOAD_STILL_POSITION_SPAN_RAD)
            } else {
                self.last_still_since_s.is_some_and(|t0| self.sim_time_s - t0 >= 0.20)
                    && self.still_anchor_positions.len() == self.measured_positions.len()
                    && self.measured_positions.iter().zip(&self.still_anchor_positions)
                        .all(|(m, a)| m.is_finite() && (m - a).abs() <= UNLOAD_STILL_POSITION_SPAN_RAD)
            };
            if !settled {
                return Err("table_z_from_tcp requires the arm to remain still for at least 0.2 s with fresh distinct hardware samples");
            }
        }
        Ok(())
    }

    pub fn check_table_capture_stillness(&self) -> Result<(), ArmError> {
        self.check_table_capture_stillness_static()
            .map_err(|msg| MotionError::InvalidCommand(msg.into()).into())
    }

    /// Start a new action record; a running one is cancelled as SUPERSEDED.
    pub fn begin_action(&mut self, kind: &str, tag: Option<String>) -> u64 {
        self.begin_action_inner(kind, tag, true)
    }

    /// Returns the `ActionId` that the next `begin_action` / `begin_action_rt` call will allocate.
    #[inline]
    pub(crate) fn peek_next_action_id(&self) -> u64 {
        self.next_action_id + 1
    }

    /// RT-safe [`Self::begin_action`]: no tag and no heap-formatted superseded message, so
    /// starting or superseding an RT action performs zero allocations and zero frees.
    pub(crate) fn begin_action_rt(&mut self, kind: &str) -> u64 {
        self.begin_action_inner(kind, None, false)
    }

    pub(in crate::supervisor) fn begin_action_inner(&mut self, kind: &str, tag: Option<String>, format_superseded_msg: bool) -> u64 {
        self.bump_motion_generation();
        let now = self.now_s();
        // Clean up old macro bookkeeping without overwriting the newly prepared motion's
        // `target_positions` or resetting `safety` back to `Armed` (caller already quiesced
        // any old macro gripper hold before building the replacement trajectory).
        if let Some(old_id) = self.macro_run.as_ref().map(|m| m.id) {
            if self.quintic_settle.as_ref().is_some_and(|(sid, _)| *sid == old_id) {
                self.quintic_settle = None;
            }
        }
        self.pending_deferred_done = None;
        self.drop_macro_run("cancelled", Some("SUPERSEDED"));
        #[cfg(feature = "policy-runtime")]
        if let Some(mut run) = self.policy_run.take() {
            if self.rt_executor_owned {
                // Executor supersession: revoke authorization, seal/close stream, recycle spares
                // in place with 0 allocs/frees, emit immutable terminal outcome, and preserve
                // replacement `target_positions` and `j7_admitted_stamp` (`reset_hold_target = false`).
                let sup_aid = run.action_id;
                let sup_steps = run.steps;
                self.teardown_rt_policy_run(run, "SUPERSEDED", false, Instant::now());
                #[cfg(not(target_arch = "wasm32"))]
                self.push_rt_policy_terminal(
                    sup_aid,
                    sup_steps,
                    "SUPERSEDED",
                    "cancelled",
                    crate::motion::PolicyStats::default(),
                );
            } else {
                run.stop_reason = Some("SUPERSEDED".to_string());
                if run.mode == crate::policy_runtime::PolicyMode::Live {
                    self.servo_target = None;
                    self.servo_settled_ticks.fill(0);
                    self.servo_ws_min_depths = None;
                }
                self.last_policy_obs = Some(run.to_observation(false));
            }
        }
        // Off-tick Legacy start: drain any queued in-tick terminal captures while `previous_action`
        // and `last_action` are still live so no rich macro payload can be evicted before capture.
        if !self.rt_executor_owned && !self.defer_terminal_capture {
            self.drain_terminal_captures_with_live_slots();
        }
        if let Some(mut old) = self.last_action.take() {
            let was_running = old.is_running();
            if was_running {
                old.state = "cancelled".into();
                old.code = Some("SUPERSEDED".into());
                if format_superseded_msg {
                    old.message = Some(format!("Replaced by a new {kind} action"));
                }
                old.finished_at = Some(now);
                self.record_terminal_transition_for(&old);
            }
            if let Some((evicting_prev, _)) = self.previous_action.as_ref() {
                if !evicting_prev.is_running() && self.staged_terminals.has_deferred(evicting_prev.id)
                {
                    let evicting_id = evicting_prev.id;
                    self.materialize_deferred_staged_by_id(evicting_id, now);
                }
            }
            if let Some((mut prev, _)) = self.previous_action.replace((old, now + 2.0)) {
                if let Some(msg) = prev.message.take() {
                    self.recycle_action_msg_spare(msg);
                }
                if let Some(warn) = prev.durability_warning.take() {
                    self.recycle_action_msg_spare(warn);
                }
            }
        }
        if format_superseded_msg {
            // Off-RT start: ensure two 256-byte spares exist (matching `last_action` and
            // `previous_action`) so back-to-back RT failures never allocate or free.
            let ready = self
                .action_msg_spares
                .iter()
                .filter(|s| s.as_ref().is_some_and(|b| b.capacity() >= 256))
                .count();
            for _ in ready..2 {
                if let Some(slot) = self
                    .action_msg_spares
                    .iter_mut()
                    .find(|s| s.as_ref().is_none_or(|b| b.capacity() < 256))
                {
                    *slot = Some(String::with_capacity(256));
                }
            }
        }
        self.next_action_id += 1;
        let id = self.next_action_id;
        let state = crate::model::HolderLabel::new("running");
        self.last_action = Some(ActionStatus {
            id, kind: kind.into(), tag, state, code: None, message: None, progress: 0.0,
            started_at: now, finished_at: None, holding: None, gripper_gap_rad: None, settled: None,
            task: None, phase: None, iteration: None, error_px: None, final_offset_px: None, acquire_height_m: None,
            episode: None, t_s: None, power_scale: None,
            sequence_id: None, step_index: None, step_type: None, repeat_index: None, repeats: None, failed_step: None, episodes: Vec::new(),
            align: None, align_turn_rad: None, angle_spread_deg: None, orientation: None, grasp_width_m: None, height_source: None, height_reason: None, object_height_m: None, close_height_m: None, probe_dz_m: None, depth_anchor: None, object_depth_m: None, object_length_m: None, jaw_clear: None, border: None, border_shift_px: None, probe_mode: None, lateral_baseline_m: None, lateral_baseline_err_mm: None, height_estimates: Vec::new(), yaw_estimates: Vec::new(), width_estimates: Vec::new(),
            recentre: None, recentre_iterations: None, descend_peak_residual_nm: None, descend_peak_tracking_rad: None,
            descend_contact: None, descend_contact_height_m: None, post_lift: None,
            pre_open: None, pre_open_m: None, held_width_m: None,
            stop_rad: None, offset_rad: None, pass: None,
            approach: None, approach_reason: None, approach_signs: None, approach_flipped: None,
            phase_ms: std::collections::BTreeMap::new(), motions: Vec::new(),
            constraint_id: None, geometry_id: None, violation_phase: None, reason: None,
            durability_confirmed: None, durability_warning: None, policy_stats: None, measurement: None,
        });
        if !self.rt_executor_owned && !self.defer_terminal_capture {
            if let Some(a) = self.last_action.as_ref().filter(|a| a.tag.is_some()) {
                self.mirror_rich_action_off_rt(a);
            }
        }
        self.action_lease_generation = self.active_lease.as_ref().map(|l| l.generation);
        self.quintic_settle = None;
        id
    }

    /// Take a preallocated message buffer (`capacity >= min_cap` when available) for RT action
    /// termination without allocating or freeing on `armd-control`. If the target action already
    /// holds a smaller buffer, it is recycled into `action_msg_spares` before taking a spare.
    pub(crate) fn take_action_msg_buf(&mut self, action_id: u64, min_cap: usize) -> String {
        if let Some(existing) = self
            .last_action
            .as_mut()
            .filter(|a| a.id == action_id)
            .and_then(|a| a.message.take())
        {
            if existing.capacity() >= min_cap {
                return existing;
            }
            self.recycle_action_msg_spare(existing);
        }
        if let Some(slot) = self
            .action_msg_spares
            .iter_mut()
            .find(|s| s.as_ref().is_some_and(|b| b.capacity() >= min_cap))
        {
            return slot.take().unwrap_or_default();
        }
        let mut best_idx = None;
        let mut best_cap = 0usize;
        for (i, s) in self.action_msg_spares.iter().enumerate() {
            if let Some(b) = s {
                if b.capacity() > best_cap {
                    best_cap = b.capacity();
                    best_idx = Some(i);
                }
            }
        }
        best_idx.and_then(|i| self.action_msg_spares[i].take()).unwrap_or_default()
    }

    /// Return a `String` message buffer to `action_msg_spares` without freeing on `armd-control`
    /// whenever a spare slot is free.
    pub(crate) fn recycle_action_msg_spare(&mut self, mut msg: String) {
        if msg.capacity() == 0 {
            return;
        }
        msg.clear();
        if let Some(slot) = self.action_msg_spares.iter_mut().find(|s| s.is_none()) {
            *slot = Some(msg);
            return;
        }
        if let Some(min_slot) = self
            .action_msg_spares
            .iter_mut()
            .min_by_key(|s| s.as_ref().map_or(0, String::capacity))
        {
            if min_slot.as_ref().is_none_or(|b| msg.capacity() > b.capacity()) {
                *min_slot = Some(msg);
            }
        }
    }

    pub(crate) fn retire_zero_check_ctx(&mut self, ctx: Box<ZeroCheckPersistContext>) {
        if self.retired_zero_check_ctx.len() < self.retired_zero_check_ctx.capacity() {
            self.retired_zero_check_ctx.push_back(ctx);
        } else {
            // Only reachable if off-thread drain was never attached (standalone legacy tests).
            drop(ctx);
        }
    }

    pub(crate) fn pop_retired_zero_check_ctx(&mut self) -> Option<Box<ZeroCheckPersistContext>> {
        self.retired_zero_check_ctx.pop_front()
    }

    /// Queues an off-RT staged `ZeroCheckPersistContext` ID (`!= 0`) for guaranteed off-thread
    /// discard without allocating or freeing on `armd-control`: first tries `rt_core/stream`,
    /// falling back to the inline fixed queue drained into `OffloadReclaim::DiscardZeroCheckCtx`.
    pub(crate) fn discard_zero_check_ctx_id_rt(&mut self, persist_ctx_id: u64) {
        if persist_ctx_id == 0 {
            return;
        }
        if let Some(tx) = self.zero_check_stream_tx.as_mut() {
            if tx
                .try_push(crate::supervisor::ZeroCheckStreamEvent::Discard { persist_ctx_id })
                .is_ok()
            {
                self.zero_check_stream_pushed = true;
                return;
            }
        }
        if self.retired_zero_check_ctx_id_len < self.retired_zero_check_ctx_ids.len() {
            self.retired_zero_check_ctx_ids[self.retired_zero_check_ctx_id_len] = persist_ctx_id;
            self.retired_zero_check_ctx_id_len += 1;
        }
    }

    pub(crate) fn pop_retired_zero_check_ctx_id(&mut self) -> Option<u64> {
        if self.retired_zero_check_ctx_id_len == 0 {
            return None;
        }
        let id = self.retired_zero_check_ctx_ids[0];
        let new_len = self.retired_zero_check_ctx_id_len - 1;
        self.retired_zero_check_ctx_ids.copy_within(1..=new_len, 0);
        self.retired_zero_check_ctx_ids[new_len] = 0;
        self.retired_zero_check_ctx_id_len = new_len;
        Some(id)
    }

    /// Zero-allocation in-place finalization for a running `gripper_zero_check` action aborted
    /// by PR0B session revocation or deadline expiry during motion (before persistence handoff).
    pub(in crate::supervisor) fn finish_zero_check_session_abort_rt(
        &mut self,
        action_id: u64,
        terminal_state: &str,
        code: &str,
        message: &str,
    ) {
        let now = self.now_s();
        let mut transitioned = false;
        if let Some(a) = self
            .last_action
            .as_mut()
            .filter(|a| a.id == action_id && a.is_running())
        {
            a.state = crate::model::HolderLabel::new(terminal_state);
            a.code = Some(crate::model::HolderLabel::new(code));
            a.finished_at = Some(now);
            transitioned = true;
        }
        if transitioned {
            let mut msg_buf = self.take_action_msg_buf(action_id, message.len());
            msg_buf.clear();
            if msg_buf.capacity() >= message.len() {
                msg_buf.push_str(message);
                let old = self
                    .last_action
                    .as_mut()
                    .filter(|a| a.id == action_id)
                    .and_then(|a| a.message.replace(msg_buf));
                if let Some(old_msg) = old {
                    self.recycle_action_msg_spare(old_msg);
                }
            } else {
                self.recycle_action_msg_spare(msg_buf);
            }
            self.bump_motion_generation();
            self.record_terminal_transition_by_id(action_id);
        }
    }

    /// Action-ID and Session-ID checked owner teardown for `SessionKind::GripperZeroCheck`.
    /// Retires only the matching trajectory/params, delivers any optional worker reply (never
    /// gating cleanup on reply acceptance), and clears `SessionSlot::ExclusiveOutput` even if
    /// another path already transitioned the slot to `SessionState::Terminal`.
    ///
    /// When `preserve_safety_hold` is `true` (E-stop / thermal trip / bus loss), retires ONLY
    /// the matching trajectory without overwriting `commanded_positions` or `target_positions`
    /// so the safety stop policy's freshness-gated hold capture owns the hold target.
    pub(crate) fn teardown_gripper_zero_check_session_rt(
        &mut self,
        action_id: u64,
        sid: Option<crate::rt_core::session::SessionId>,
        stop_motion_if_matching: bool,
        preserve_safety_hold: bool,
        worker_reply: Option<crate::rt_core::session::WorkerReply>,
        reason: crate::rt_core::session::SessionTerminalReason,
        now: Instant,
    ) {
        use crate::rt_core::broker::ActionId;
        use crate::rt_core::session::{SessionKind, SessionSlot};

        let matching_traj = self
            .active_trajectory
            .as_ref()
            .is_some_and(|t| t.action_id == Some(action_id));
        if matching_traj {
            if preserve_safety_hold {
                self.replace_active_trajectory(None);
            } else if stop_motion_if_matching {
                self.stop_motion();
            }
        }
        if self
            .zero_check_params
            .as_ref()
            .is_some_and(|p| p.action_id == action_id)
        {
            if let Some(params) = self.zero_check_params.take() {
                if let Some(ctx) = params.persist_ctx {
                    self.retire_zero_check_ctx(ctx);
                }
                if params.persist_ctx_id != 0 {
                    self.discard_zero_check_ctx_id_rt(params.persist_ctx_id);
                }
            }
        }
        if self.zero_check_persisting != Some(action_id)
            && self
                .zero_check_persisting_grant
                .is_some_and(|(gid, _)| gid == action_id)
        {
            self.zero_check_persisting_grant = None;
        }
        if let Some(active) = self.sessions.slot(SessionSlot::ExclusiveOutput).copied() {
            if active.kind == SessionKind::GripperZeroCheck
                && active.action_id == Some(ActionId(action_id))
                && sid.is_none_or(|want| want == active.id)
            {
                if let Some(reply) = worker_reply {
                    let _ = self.sessions.deliver_worker_reply(reply, now, Ok(()));
                }
                if active.is_live() {
                    let _ = self
                        .sessions
                        .finish_slot(SessionSlot::ExclusiveOutput, active.id, reason, false);
                }
                let _ = self
                    .sessions
                    .clear_terminal_slot(SessionSlot::ExclusiveOutput, active.id);
            }
        }
    }

    /// Release-side PR0B session stepper for `SessionKind::GripperZeroCheck` (runs in Step 2b
    /// before Steps 3-5, 0 allocs / 0 `SupervisorLock`).
    ///
    /// Ownership invariant:
    /// - During `SessionState::Running` (motion + measurement): `SessionDriver` governs authority,
    ///   deadlines, TX cutoff clamping, and cancellation before bus output.
    /// - During `SessionState::AwaitingWorker` (after persistence handoff): PR C (`CommonCommitWorker`
    ///   / `zero_check_owner_*`) owns the disk reservation, rename/fsync boundary, reconciliation,
    ///   and action terminal; `service_gripper_zero_check_session` never prematurely aborts or
    ///   overwrites `last_action` while `zero_check_persisting == Some(action_id)`.
    pub(crate) fn service_gripper_zero_check_session_with_budget(
        &mut self,
        ctx: &crate::rt_core::ops::CommitCtx,
        release_budget: &mut crate::rt_core::budget::ReleaseBudget,
        owner_now_fn: &dyn Fn() -> Instant,
    ) {
        let release_deadline = release_budget.deadline();
        use crate::rt_core::session::{
            PhaseStepOutcome, SessionKind, SessionSlot, SessionState, SessionStep,
            SessionTerminalReason,
        };

        let Some(active) = self.sessions.slot(SessionSlot::ExclusiveOutput).copied() else {
            return;
        };
        if active.kind != SessionKind::GripperZeroCheck {
            return;
        }
        let Some(action_id) = active.action_id.map(|a| a.0) else {
            let _ = self
                .sessions
                .clear_terminal_slot(SessionSlot::ExclusiveOutput, active.id);
            return;
        };
        let now = owner_now_fn();
        match active.state {
            SessionState::Terminal(outcome) => {
                if self.zero_check_persisting == Some(action_id) {
                    // Persistence handoff already occurred; PR C owns settlement.
                    return;
                }
                self.teardown_gripper_zero_check_session_rt(
                    action_id,
                    Some(active.id),
                    true,
                    false,
                    None,
                    outcome.reason,
                    now,
                );
            }
            SessionState::AwaitingWorker(_) => {
                // PR C owns persistence settlement and the action terminal. Clear the session slot
                // only once PR C (or safety reconciliation) has settled `zero_check_persisting`
                // and `last_action`.
                let still_running = self.zero_check_persisting == Some(action_id)
                    || self
                        .last_action
                        .as_ref()
                        .is_some_and(|a| a.id == action_id && a.is_running());
                if !still_running {
                    self.teardown_gripper_zero_check_session_rt(
                        action_id,
                        Some(active.id),
                        false,
                        false,
                        None,
                        SessionTerminalReason::Completed,
                        now,
                    );
                }
            }
            SessionState::Running => {
                let action_running = self
                    .last_action
                    .as_ref()
                    .is_some_and(|a| a.id == action_id && a.is_running());
                if !action_running && self.zero_check_persisting != Some(action_id) {
                    self.teardown_gripper_zero_check_session_rt(
                        action_id,
                        Some(active.id),
                        false,
                        false,
                        None,
                        SessionTerminalReason::Completed,
                        now,
                    );
                    return;
                }
                let total_span = release_deadline.saturating_duration_since(now);
                let margin = self.tx_release_publish_margin().min(total_span / 2);
                let base_cutoff = release_deadline.checked_sub(margin).unwrap_or(now);
                let rctx = self.build_session_release_ctx(
                    &crate::rt_core::ops::CommitCtx { now, ..*ctx },
                    release_deadline,
                    base_cutoff,
                );
                let conn_closed = active.authority.connection_id != 0
                    && self.is_connection_closed(active.authority.connection_id);
                let lease_ok = !active.authority.require_lease
                    || self.is_session_lease_valid_at(
                        active.authority.connection_id,
                        active.authority.lease_generation,
                        now,
                    );
                let op_ok = self
                    .check_operator_requirement(active.authority.operator, now)
                    .is_ok();
                if let Some(SessionStep::Terminal(outcome)) = self.sessions.step_slot(
                    SessionSlot::ExclusiveOutput,
                    &rctx,
                    |_| conn_closed,
                    |_, _, _| lease_ok,
                    |_, _| op_ok,
                    |_, _| false,
                    |_, _, _| PhaseStepOutcome::Pending,
                ) {
                    let (t_state, t_code, t_msg) = match outcome.reason {
                        SessionTerminalReason::Revoked("OPERATOR_REQUIRED") => (
                            "failed",
                            "OPERATOR_REQUIRED",
                            "Operator session expired or locked",
                        ),
                        SessionTerminalReason::Revoked("STOPPED") => {
                            ("cancelled", "STOPPED", "Stopped")
                        }
                        SessionTerminalReason::Revoked("LEASE_LOST") => (
                            "cancelled",
                            "LEASE_LOST",
                            "Control lease lost or released",
                        ),
                        SessionTerminalReason::Revoked("EPOCH_MISMATCH") => {
                            if rctx.estop_latched {
                                ("failed", "ESTOP_ACTIVE", "E-stop latched")
                            } else {
                                ("cancelled", "EPOCH_MISMATCH", "Safety epoch changed")
                            }
                        }
                        SessionTerminalReason::DeadlineExpired(code) => {
                            ("failed", code, "Gripper zero check timed out")
                        }
                        SessionTerminalReason::Revoked(code) => {
                            ("failed", code, "Gripper zero check revoked")
                        }
                        other => ("failed", other.code(), "Gripper zero check ended"),
                    };
                    self.finish_zero_check_session_abort_rt(action_id, t_state, t_code, t_msg);
                    self.teardown_gripper_zero_check_session_rt(
                        action_id,
                        Some(active.id),
                        true,
                        rctx.estop_latched,
                        None,
                        outcome.reason,
                        now,
                    );
                }
            }
        }
    }

    pub(in crate::supervisor) fn zero_check_revoke_terminal(reason: &'static str) -> (&'static str, &'static str, &'static str) {
        match reason {
            "STOPPED" => ("cancelled", "STOPPED", "Motion stopped"),
            "LEASE_LOST" => ("cancelled", "LEASE_LOST", "Control lease lost or released"),
            "DISCONNECTED" => ("cancelled", "DISCONNECTED", "Client connection closed"),
            "ESTOP_ACTIVE" => ("failed", "ESTOP_ACTIVE", "E-stop latched"),
            "THERMAL_TRIP" => ("failed", "THERMAL_TRIP", "Thermal trip"),
            "BUS_LOSS" => ("failed", "BUS_LOSS", "CAN feedback lost"),
            "INTERNAL_FAULT" => ("failed", "INTERNAL_FAULT", "Internal fault latched"),
            "OPERATOR_REQUIRED" => ("failed", "OPERATOR_REQUIRED", "Operator session expired or locked"),
            other => ("cancelled", other, "Gripper zero check cancelled"),
        }
    }

    /// Immediate owner-side cleanup hook for Stop / E-stop / SAFE-HOLD / shutdown during the
    /// motion phase of `SessionKind::GripperZeroCheck`. If persistence handoff already started
    /// (`zero_check_persisting == Some(action_id)`), PR C owns settlement.
    pub(crate) fn cancel_active_gripper_zero_check_on_stop(&mut self, reason: &'static str) {
        use crate::rt_core::session::{SessionKind, SessionSlot, SessionTerminalReason};
        let Some(active) = self.sessions.slot(SessionSlot::ExclusiveOutput).copied() else {
            return;
        };
        if active.kind != SessionKind::GripperZeroCheck {
            return;
        }
        let Some(action_id) = active.action_id.map(|a| a.0) else {
            return;
        };
        if self.zero_check_persisting == Some(action_id) {
            return;
        }
        let preserve_safety_hold = matches!(reason, "ESTOP_ACTIVE" | "THERMAL_TRIP" | "BUS_LOSS");
        let (t_state, t_code, t_msg) = Self::zero_check_revoke_terminal(reason);
        self.finish_zero_check_session_abort_rt(action_id, t_state, t_code, t_msg);
        self.teardown_gripper_zero_check_session_rt(
            action_id,
            Some(active.id),
            !preserve_safety_hold,
            preserve_safety_hold,
            None,
            SessionTerminalReason::Revoked(reason),
            Instant::now(),
        );
    }

    /// Immediate owner-side cleanup hook for connection disconnect during the motion phase of
    /// `SessionKind::GripperZeroCheck`.
    pub(crate) fn cancel_active_gripper_zero_check_for_connection(
        &mut self,
        conn_id: u64,
        reason: &'static str,
    ) {
        use crate::rt_core::session::{SessionKind, SessionSlot, SessionTerminalReason};
        if conn_id == 0 {
            return;
        }
        let Some(active) = self.sessions.slot(SessionSlot::ExclusiveOutput).copied() else {
            return;
        };
        if active.kind != SessionKind::GripperZeroCheck
            || active.authority.connection_id != conn_id
        {
            return;
        }
        let Some(action_id) = active.action_id.map(|a| a.0) else {
            return;
        };
        if self.zero_check_persisting == Some(action_id) {
            return;
        }
        let (t_state, t_code, t_msg) = Self::zero_check_revoke_terminal(reason);
        self.finish_zero_check_session_abort_rt(action_id, t_state, t_code, t_msg);
        self.teardown_gripper_zero_check_session_rt(
            action_id,
            Some(active.id),
            true,
            false,
            None,
            SessionTerminalReason::Revoked(reason),
            Instant::now(),
        );
    }

    pub fn finish_action(&mut self, id: u64, state: &str, code: Option<&str>, message: Option<String>,
                     settled: Option<bool>, holding: Option<bool>, gap: Option<f32>) {
        let now = self.now_s();
        let mut store_offset_req: Option<(f32, u64, Box<ZeroCheckPersistContext>)> = None;
        let mut store_offset_stream_req: Option<(
            f32,
            u64,
            u64,
            Option<crate::rt_core::session::SessionId>,
            Option<crate::auth::OperatorGrant>,
        )> = None;
        let mut finish_zero_check_session: Option<(u64, Option<crate::rt_core::session::SessionId>)> = None;
        let mut retired_ctx: Option<Box<ZeroCheckPersistContext>> = None;
        let mut incoming_message = message;
        let mut displaced_message: Option<String> = None;
        let mut transitioned_terminal = false;
        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id && a.is_running()) {
            transitioned_terminal = true;
            a.state = state.into();
            a.code = code.map(Into::into);
            displaced_message = std::mem::replace(&mut a.message, incoming_message.take());
            a.finished_at = Some(now);
            if state != "failed" || code != Some("WORKSPACE_VIOLATION") {
                a.constraint_id = None;
                a.geometry_id = None;
                a.violation_phase = None;
                a.reason = None;
            }
            if state == "done" { a.progress = 1.0; }
            if settled.is_some() { a.settled = settled; }
            if holding.is_some() { a.holding = holding; }
            if gap.is_some() { a.gripper_gap_rad = gap; }
            if a.kind == "gripper_zero_check"
                && self.zero_check_params.as_ref().is_some_and(|p| p.action_id == id)
            {
                let params = self.zero_check_params.take().expect("checked action_id match");
                let mut unused_ctx = params.persist_ctx;
                let mut unused_ctx_id = params.persist_ctx_id;
                let sess_id = params.session_id;
                let mut handed_off_stream = false;
                if state == "done" {
                    // FOLLOW-UP: open contract question (kept for 1.6.1 parity with legacy on both
                    // owners): a stall against an obstruction ends the seek as `done` and the stall
                    // position is taken as the mechanical zero (`stop_rad`, `offset_rad`, and with
                    // `store_offset` the stored offset). Should an obstructed seek be told apart
                    // from the jaw stop (e.g. `CALIBRATION_OBSTRUCTED`) instead of being accepted?
                    // Pinned by `dual_owner_zero_check_obstructed_seek_stall_accepted_parity`.
                    let stop_rad = self.config.gripper_index().and_then(|idx| self.measured_positions.get(idx)).copied().unwrap_or(0.0);
                    let offset_rad = stop_rad - params.base_closed_rad;
                    let pass = offset_rad.abs() <= params.threshold_rad;
                    a.stop_rad = Some(stop_rad);
                    a.offset_rad = Some(offset_rad);
                    a.pass = Some(pass);
                    self.gripper_zero_checked = true;
                    self.last_zero_check_offset = Some(offset_rad);
                    if params.store_offset {
                        if offset_rad.abs() <= 0.5 {
                            if let Some(ctx) = unused_ctx.take() {
                                store_offset_req = Some((offset_rad, params.connection_id, ctx));
                            } else if unused_ctx_id != 0 {
                                store_offset_stream_req = Some((
                                    offset_rad,
                                    params.connection_id,
                                    unused_ctx_id,
                                    sess_id,
                                    params.operator_grant,
                                ));
                                unused_ctx_id = 0;
                                handed_off_stream = true;
                            }
                        } else if !self.rt_executor_owned && !self.defer_zero_check_persist {
                            tracing::warn!("Gripper zero check offset {offset_rad:.4} exceeds 0.5 rad; storing refused");
                        }
                    }
                }
                if !handed_off_stream && (unused_ctx_id != 0 || sess_id.is_some()) {
                    finish_zero_check_session = Some((unused_ctx_id, sess_id));
                }
                retired_ctx = unused_ctx;
            }
            if !self.rt_executor_owned && !self.defer_zero_check_persist {
                tracing::info!(
                    "Action {} ({}) {}{}{}",
                    id,
                    a.kind,
                    state,
                    if code.is_some() { " " } else { "" },
                    code.unwrap_or(""),
                );
            }
        }
        if let Some(unused_msg) = displaced_message.or(incoming_message) {
            self.recycle_action_msg_spare(unused_msg);
        }
        if let Some(ctx) = retired_ctx {
            self.retire_zero_check_ctx(ctx);
        }
        if let Some((unused_ctx_id, sess_id)) = finish_zero_check_session {
            if unused_ctx_id != 0 {
                self.discard_zero_check_ctx_id_rt(unused_ctx_id);
            }
            if let Some(sid) = sess_id {
                let reason = match state {
                    "done" => crate::rt_core::session::SessionTerminalReason::Completed,
                    "cancelled" => crate::rt_core::session::SessionTerminalReason::Cancelled(
                        match code {
                            Some("STOPPED") => "STOPPED",
                            Some("LEASE_LOST") => "LEASE_LOST",
                            Some("EPOCH_MISMATCH") => "EPOCH_MISMATCH",
                            _ => "CANCELLED",
                        },
                    ),
                    _ => crate::rt_core::session::SessionTerminalReason::Failed(
                        match code {
                            Some("GRIPPER_TIMEOUT") => "GRIPPER_TIMEOUT",
                            Some("J7_FEEDBACK_STALE") => "J7_FEEDBACK_STALE",
                            Some("ESTOP_ACTIVE") => "ESTOP_ACTIVE",
                            _ => "FAILED",
                        },
                    ),
                };
                self.teardown_gripper_zero_check_session_rt(
                    id,
                    Some(sid),
                    false,
                    false,
                    None,
                    reason,
                    Instant::now(),
                );
            }
        }
        if let Some((offset_rad, connection_id, persist_ctx_id, sess_id, op_grant)) =
            store_offset_stream_req
        {
            let stream_ready = self
                .zero_check_stream_tx
                .as_ref()
                .is_some_and(|tx| !tx.is_closed())
                && self.pending_zero_check_stream_persist.is_none();
            if !stream_ready {
                const MSG: &str = "Persistence channel unavailable or saturated";
                let mut msg_buf = self.take_action_msg_buf(id, MSG.len());
                msg_buf.clear();
                let set_msg = if msg_buf.capacity() >= MSG.len() {
                    msg_buf.push_str(MSG);
                    Some(msg_buf)
                } else {
                    self.recycle_action_msg_spare(msg_buf);
                    None
                };
                let mut replaced = None;
                if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
                    a.state = crate::model::HolderLabel::new("failed");
                    a.code = Some(crate::model::HolderLabel::new("PERSIST_FAILED"));
                    if let Some(m) = set_msg {
                        replaced = a.message.replace(m);
                    }
                } else if let Some(m) = set_msg {
                    self.recycle_action_msg_spare(m);
                }
                if let Some(old_msg) = replaced {
                    self.recycle_action_msg_spare(old_msg);
                }
                self.discard_zero_check_ctx_id_rt(persist_ctx_id);
                if let Some(sid) = sess_id {
                    self.teardown_gripper_zero_check_session_rt(
                        id,
                        Some(sid),
                        false,
                        false,
                        None,
                        crate::rt_core::session::SessionTerminalReason::Failed("PERSIST_FAILED"),
                        Instant::now(),
                    );
                }
            } else {
                // Hold action running until PR C config worker settles the disk transaction.
                transitioned_terminal = false;
                if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
                    a.state = crate::model::HolderLabel::new("running");
                    a.finished_at = None;
                }
                self.zero_check_persisting = Some(id);
                self.zero_check_persisting_grant = op_grant.map(|g| (id, g));
                let worker_ticket = sess_id.and_then(|sid| {
                    self.sessions.enter_await_worker(
                        crate::rt_core::session::SessionSlot::ExclusiveOutput,
                        sid,
                        None,
                        "",
                    )
                });
                let lease_owner = self
                    .active_lease
                    .as_ref()
                    .map(|l| (crate::model::HolderLabel::new(&l.client_id), l.generation));
                self.pending_zero_check_stream_persist =
                    Some(crate::supervisor::ZeroCheckStreamEvent::Persist {
                        action_id: id,
                        offset_rad,
                        base_revision: self.gripper_calib_revision,
                        connection_id,
                        lease_owner,
                        estop_epoch: self.safety.epoch_id(),
                        stop_gen: self.current_stop_gen(),
                        deadline: Instant::now() + crate::supervisor::config::CONFIG_PROPOSAL_BUDGET,
                        persist_ctx_id,
                        worker_ticket,
                    });
            }
        }
        if let Some((offset_rad, connection_id, mut ctx)) = store_offset_req {
            if self.defer_zero_check_persist {
                // Zero-allocation RT path: candidate validation and JSON serialization run
                // completely off `armd-control` (on `armd-persist` or the legacy async worker).
                if !self.persist_capacity_available {
                    if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
                        a.state = crate::model::HolderLabel::new("failed");
                        a.code = Some(crate::model::HolderLabel::new("PERSIST_FAILED"));
                        let mut msg = std::mem::take(&mut ctx.sat_msg_buf);
                        msg.clear();
                        msg.push_str("Persistence channel unavailable or saturated");
                        if let Some(old_msg) = a.message.replace(msg) {
                            ctx.sat_msg_buf = old_msg;
                        }
                    }
                    self.retire_zero_check_ctx(ctx);
                } else {
                    // Hold action as running until atomic persist-before-commit completes off-thread.
                    transitioned_terminal = false;
                    if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
                        a.state = crate::model::HolderLabel::new("running");
                        a.finished_at = None;
                    }
                    self.zero_check_persisting = Some(id);
                    let lease_owner = self
                        .active_lease
                        .as_ref()
                        .map(|l| (crate::model::HolderLabel::new(&l.client_id), l.generation));
                    self.pending_zero_check_persist = Some(PendingZeroCheckPersist {
                        action_id: id,
                        offset_rad,
                        base_revision: self.gripper_calib_revision,
                        connection_id,
                        lease_owner,
                        estop_epoch: self.safety.epoch_id(),
                        stop_gen: self.current_stop_gen(),
                        deadline: Instant::now() + crate::supervisor::config::CONFIG_PROPOSAL_BUDGET,
                        ctx,
                    });
                }
            } else {
                // Non-deferred legacy path (direct `SupervisorInner::tick` tests): route through
                // the EXACT same common commit state machine (`reserve -> serialize + stage + fsync
                // -> fresh authority recheck -> rename -> parent-dir fsync -> authority recheck
                // -> activate`, with dir-sync failure entering `Reconciling`).
                // `commit_zero_check_sync` owns the authoritative terminal transition emission.
                transitioned_terminal = false;
                crate::rt_core::requests::config::commit_zero_check_sync(self, id, offset_rad, ctx);
            }
        }
        if holding == Some(false) { self.clear_held_rt(); }
        // a standalone close that detected an object counts as holding until a release completes
        if holding == Some(true) && self.held.is_none() {
            self.held = Some(HeldObject { label: String::new(), spill_safe: false });
        }
        if self.quintic_settle.as_ref().is_some_and(|(sid, _)| *sid == id) { self.quintic_settle = None; }
        if self.macro_run.as_ref().is_some_and(|m| m.id == id) {
            if let Some((state, code)) = self.last_action.as_ref().filter(|a| a.id == id && !a.is_running()).map(|a| (a.state, a.code)) {
                self.record_macro_terminal(id, &state, code.as_deref());
            }
            if let Some(run) = self.macro_run.take() {
                self.retire_macro_run(run);
            }
        }
        if transitioned_terminal {
            self.bump_motion_generation();
            self.record_terminal_transition_by_id(id);
        }
    }

    /// Start a park (NormalUnload) and record it as a `park` action. Returns its id when
    /// the park started.
    /// A park whose path enters a keep-out box is `WORKSPACE_VIOLATION`.
    /// After a fault or E-stop clear the motors may still hold the arm: safety Idle, torque on,
    /// re-arm pending (clear_fault keeps the hold rather than drop an arm that is not at rest).
    pub fn holding_after_clear(&self) -> bool {
        self.is_torqued && self.pending_rearm && !self.disable_pending
            && matches!(self.safety.state(), SafetyState::Idle)
            && !self.e_stop_latched.load(Ordering::SeqCst)
    }

    pub fn unload_action(admitted: Admitted<'_>) -> Result<Option<u64>, ArmError> {
        let (this, proof) = admitted.consume()?;
        this.check_terminal_fifo_for_new_action()?;
        this.park_rejection = None;
        // A park from the hold after a clear re-arms through the verified enable path first (fresh
        // feedback, commanded = measured); without it NormalUnload keeps holding and parks nothing.
        if this.holding_after_clear() {
            this.enable_torque_inner(&proof)?;
        }
        this.execute_stop(StopPolicy::NormalUnload);
        if let Some(e) = this.park_rejection.take() { return Err(e.into()); }
        if this.mit.park_after_hand_back && !this.pending_normal_unload {
            // deferred behind the MIT brake and hand back: the action runs from now on
            let id = this.begin_action("park", None);
            this.mit.deferred_park_action = Some(id);
            return Ok(Some(id));
        }
        if !this.pending_normal_unload { return Ok(None); }
        let id = this.begin_action("park", None);
        if let Some(active) = this.active_trajectory.as_mut() { active.action_id = Some(id); }
        Ok(Some(id))
    }

    /// Zero-allocation mirror of [`Self::action_preconditions`] returning static terminal codes
    /// for RT commit paths (`armd-control`).
    pub(crate) fn action_preconditions_static(&self) -> Result<(), &'static str> {
        if self.handover_settling_since.is_some() { return Err("REJECTED"); }
        if self.e_stop_latched.load(Ordering::SeqCst) { return Err("ESTOP_ACTIVE"); }
        match self.safety.state() {
            SafetyState::EStop => return Err("ESTOP_ACTIVE"),
            SafetyState::Fault(_) => return Err("FAULT_ACTIVE"),
            s if !s.is_operational() => return Err("NOT_ARMED"),
            _ => {}
        }
        if self.pending_rearm || self.pending_normal_unload {
            return Err("NOT_ARMED");
        }
        if self.collision.hold.is_some() { return Err("COLLISION_HOLD"); }
        if self.core.motion_inhibited_by_config() { return Err("CONFIG_BUSY"); }
        if self.mit_motion_blocked() { return Err("MOTION_BUSY"); }
        if self.has_busy_reason() { return Err("MOTION_BUSY"); }
        Ok(())
    }

    /// Preconditions shared by Cartesian and gripper actions.
    pub(crate) fn action_preconditions(&self) -> Result<(), ArmError> {
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
        if self.disable_pending || self.torque_transition.is_some() || self.park_continuation.is_some() {
            return Err(MotionError::MotionBusy("a torque transition or park is pending".into()).into());
        }
        self.collision_hold_check()?;
        self.config_motion_check()?;
        if let Some(reason) = self.mit_motion_blocker() { return Err(MotionError::MotionBusy(reason).into()); }
        if let Some(reason) = self.busy_reason() { return Err(MotionError::MotionBusy(reason).into()); }
        Ok(())
    }
}
