//! Gripper action starters, zero-check starters, and action reconciliation/stepping.

#[allow(unused_imports)]
use super::super::*;
#[allow(unused_imports)]
use crate::macro_action::{ChildTicket, ContactGuard, ContactWatch};

impl SupervisorInner {
    /// Start a gripper action. Returns (action id, expected duration).
    pub fn start_gripper(admitted: Admitted<'_>, request: GripperRequest, epoch: u64) -> Result<(u64, f64), ArmError> {
        Self::start_gripper_with_baseline(admitted, request, epoch, GripperBaselineCmd::None)
    }

    /// [`Self::start_gripper`] with a baseline command; a reset happens only once the action is
    /// admitted (a refused action leaves the learned baseline as it was).
    pub fn start_gripper_with_baseline(admitted: Admitted<'_>, request: GripperRequest, epoch: u64, baseline_cmd: GripperBaselineCmd) -> Result<(u64, f64), ArmError> {
        let (this, _proof) = admitted.consume()?;
        this.start_gripper_with_baseline_inner(request, epoch, baseline_cmd)
    }

    pub(super) fn start_gripper_with_baseline_inner(&mut self, request: GripperRequest, epoch: u64, baseline_cmd: GripperBaselineCmd) -> Result<(u64, f64), ArmError> {
        self.check_terminal_fifo_for_new_action()?;
        let gripper_idx = self.config.gripper_index().ok_or_else(|| {
            MotionError::Unsupported("Gripper actions need a profile with a gripper joint".into())
        })?;
        let Some(policy) = self.config.gripper_policy.clone() else {
            return Err(MotionError::Unsupported("Gripper actions need a profile with a gripper policy".into()).into());
        };
        if epoch != self.safety.epoch_id() {
            return Err(SafetyError::EpochMismatch { expected: self.safety.epoch_id(), received: epoch }.into());
        }
        if self.config.active_tool.commissioning_required && request.speed > 0.4 + 1e-6 {
            return Err(SafetyError::CommissioningRequired(format!(
                "gripper (speed {:.2} rad/s > 0.40 rad/s)",
                request.speed
            )).into());
        }
        self.action_preconditions()?;
        self.check_pos_force_gripper_fresh()?;
        self.check_gripper_close_torque_feedback(&request, &policy, gripper_idx)?;
        let close_cap = self.gripper_close_speed_limit();
        let ceiling_est = self.gripper_ceiling_torque_estimate();
        let motion = GripperMotion::try_with_close_speed_limit(
            request, policy, self.config.limits[gripper_idx].clone(),
            self.commanded_positions[gripper_idx], gripper_idx, close_cap,
        )?.with_ceiling_torque_estimate(ceiling_est);
        let duration = motion.expected_duration();
        let tag = motion.request.tag.clone();
        self.safety.mark_motion_started()?;
        self.releasing = matches!(motion.request.command, GripperCommand::Open);
        self.gripper_hold_vel_ceiling = Some(motion.wire_velocity_ceiling());
        if baseline_cmd == GripperBaselineCmd::Reset { self.reset_gripper_baseline(); }
        let learn = if baseline_cmd == GripperBaselineCmd::None { crate::motion::LearnMode::Opening } else { crate::motion::LearnMode::Calibration };
        let motion = motion.with_baseline(self.gripper_baseline_for_motion(), learn);
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        self.clear_servo_ws_min_depths_rt();
        let mut target = self.commanded_positions.clone();
        target[gripper_idx] = motion.target;
        self.target_positions = target;
        self.last_heartbeat = Instant::now();
        let id = self.begin_action("gripper", tag);
        self.active_trajectory = Some(ActiveTrajectory::new(Some(id), Reference::Gripper(Box::new(motion))));
        Ok((id, duration))
    }

    /// Start a gripper zero check action. Returns (action id, expected duration).
    pub fn start_gripper_zero_check(admitted: Admitted<'_>, body: &serde_json::Value, epoch: u64) -> Result<(u64, f64), ArmError> {
        Self::start_gripper_zero_check_with_auth(admitted, body, epoch, None)
    }

    pub fn start_gripper_zero_check_with_auth(admitted: Admitted<'_>, body: &serde_json::Value, epoch: u64, operator_auth: Option<(crate::auth::OperatorAuth, Option<String>)>) -> Result<(u64, f64), ArmError> {
        let (this, _proof) = admitted.consume()?;
        this.start_gripper_zero_check_with_auth_inner(body, epoch, operator_auth)
    }

    pub(super) fn start_gripper_zero_check_with_auth_inner(&mut self, body: &serde_json::Value, epoch: u64, operator_auth: Option<(crate::auth::OperatorAuth, Option<String>)>) -> Result<(u64, f64), ArmError> {
        self.check_terminal_fifo_for_new_action()?;
        let gripper_idx = self.config.gripper_index().ok_or_else(|| {
            MotionError::Unsupported("Gripper zero check needs a profile with a gripper joint".into())
        })?;
        let Some(policy) = self.config.gripper_policy.clone() else {
            return Err(MotionError::Unsupported("Gripper zero check needs a profile with a gripper policy".into()).into());
        };
        if !policy.zero_check_supported {
            return Err(MotionError::Unsupported("Gripper zero check is not supported for this gripper policy".into()).into());
        }
        if epoch != self.safety.epoch_id() {
            return Err(SafetyError::EpochMismatch { expected: self.safety.epoch_id(), received: epoch }.into());
        }
        self.action_preconditions()?;
        self.check_pos_force_gripper_fresh()?;
        if !self.is_torqued {
            return Err(SafetyError::NotArmed(self.safety.state().clone()).into());
        }
        if self.held.is_some() {
            return Err(ArmError::Rejected {
                status: 409,
                code: "OBJECT_HELD".into(),
                message: "Gripper holds an object (or its release did not finish); open the gripper fully before the zero check".into(),
            });
        }
        let store_offset = body.get("store_offset").and_then(serde_json::Value::as_bool).unwrap_or(false);
        let threshold_rad = body.get("threshold_rad").and_then(serde_json::Value::as_f64).map(|v| v as f32).unwrap_or(0.10);
        if !threshold_rad.is_finite() || threshold_rad <= 0.0 {
            return Err(MotionError::InvalidCommand("threshold_rad must be positive and finite".into()).into());
        }

        let request = GripperRequest {
            command: GripperCommand::Seek,
            fraction: None,
            grasp: false,
            speed: 0.3,
            // derived from the travel at 0.3 rad/s (fully open -> closed takes about 20 s)
            timeout_s: None,
            tag: None,
        };
        self.check_gripper_close_torque_feedback(&request, &policy, gripper_idx)?;
        let close_cap = self.gripper_close_speed_limit();
        let ceiling_est = self.gripper_ceiling_torque_estimate();
        let motion = GripperMotion::try_with_close_speed_limit(
            request, policy, self.config.limits[gripper_idx].clone(),
            self.commanded_positions[gripper_idx], gripper_idx, close_cap,
        )?.with_ceiling_torque_estimate(ceiling_est);
        let duration = motion.expected_duration();
        self.gripper_hold_vel_ceiling = Some(motion.wire_velocity_ceiling());
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        self.clear_servo_ws_min_depths_rt();
        let mut target = self.commanded_positions.clone();
        target[gripper_idx] = motion.target;
        self.target_positions = target;
        self.safety.mark_motion_started()?;
        self.last_heartbeat = Instant::now();
        let id = self.begin_action("gripper_zero_check", None);
        self.active_trajectory = Some(ActiveTrajectory::new(Some(id), Reference::Gripper(Box::new(motion))));
        let base_closed_rad = self.base_gripper_policy.as_ref().map_or(0.0, |p| p.closed_rad);
        self.gripper_stalled = false;
        self.gripper_high_torque_ticks = 0;
        self.gripper_low_torque_ticks = 0;
        self.gripper_stall_window.reset();
        let persist_ctx = if store_offset {
            Some(Box::new(ZeroCheckPersistContext::from_inner(self, operator_auth)))
        } else {
            None
        };
        self.zero_check_params = Some(ZeroCheckParams {
            action_id: id,
            store_offset,
            threshold_rad,
            base_closed_rad,
            persist_ctx,
            persist_ctx_id: 0,
            session_id: None,
            connection_id: 0,
            operator_grant: None,
        });
        Ok((id, duration))
    }

    pub fn gripper_zero_observation(&self) -> Option<crate::model::GripperZeroObservation> {
        let mut out = None;
        self.gripper_zero_observation_into(&mut out);
        out
    }

    /// [`Self::gripper_zero_observation`] into `out`, reusing its memory.
    pub fn gripper_zero_observation_into(&self, out: &mut Option<crate::model::GripperZeroObservation>) {
        let (Some(idx), Some(policy)) = (self.config.gripper_index(), self.config.gripper_policy.as_ref()) else {
            *out = None;
            return;
        };
        let j7 = self.measured_positions.get(idx).copied().unwrap_or(0.0);
        let offset_rad = self.gripper_calib_offset.or(self.last_zero_check_offset);
        let state = if self.gripper_calib_invalid {
            "calib_invalid"
        } else if self.gripper_zero_checked {
            "checked"
        } else {
            let min_j7 = policy.closed_rad.min(policy.open_rad) - 0.3;
            let max_j7 = policy.closed_rad.max(policy.open_rad) + 0.3;
            if j7 < min_j7 || j7 > max_j7 { "out_of_range" } else { "ok" }
        };
        match out {
            Some(o) => {
                crate::rt_core::scratch::set_str(&mut o.state, state);
                o.j7 = j7;
                o.offset_rad = offset_rad;
            }
            None => *out = Some(crate::model::GripperZeroObservation { state: state.to_string(), j7, offset_rad }),
        }
    }

    pub fn set_mock_gripper_stall_offset(&mut self, offset: Option<f32>) {
        self.mock_gripper_stall_offset = offset;
    }

    /// Single place that ends a running action whose reference went away (spec 2.3).
    pub(super) fn reconcile_action(&mut self, dt: f64, tick_output_accepted: bool, now: Instant) {
        let Some(action) = self.last_action.as_ref().filter(|a| a.is_running()) else { return; };
        let (id, is_park, is_replay) = (action.id, action.kind == "park", action.kind == "replay");
        let is_macro = self.macro_run.as_ref().is_some_and(|m| m.id == id);
        #[cfg(feature = "policy-runtime")]
        let is_policy = self.policy_run.as_ref().is_some_and(|p| p.action_id == id);
        #[cfg(not(feature = "policy-runtime"))]
        let is_policy = false;
        let carried = self.active_trajectory.as_ref().is_some_and(|a| a.action_id == Some(id));
        let lease_now = self.active_lease.as_ref().map(|l| l.generation);
        enum ReconcileCause {
            Static(&'static str, &'static str, &'static str),
            SafetyMessage(&'static str, &'static str),
            SafetyDebug(&'static str, &'static str),
        }
        let cause = match self.safety.state() {
            SafetyState::EStop => Some(ReconcileCause::Static("failed", "ESTOP_ACTIVE", "E-stop latched")),
            SafetyState::Fault(FaultReason::WatchdogTimeout) => {
                Some(ReconcileCause::Static("failed", "WATCHDOG", "Lease watchdog expired"))
            }
            SafetyState::Fault(FaultReason::CanTxStalled) => {
                Some(ReconcileCause::SafetyMessage("failed", "CAN_TX_STALLED"))
            }
            SafetyState::Fault(FaultReason::J7FeedbackStale) => {
                Some(ReconcileCause::SafetyMessage("failed", "J7_FEEDBACK_STALE"))
            }
            SafetyState::Fault(FaultReason::Other(_)) => {
                Some(ReconcileCause::SafetyMessage("failed", "FAULT"))
            }
            SafetyState::Fault(_) => Some(ReconcileCause::SafetyDebug("failed", "FAULT")),
            _ if !is_park && self.watchdog_park.is_some() => {
                Some(ReconcileCause::Static("failed", "WATCHDOG", "Lease watchdog expired; parking"))
            }
            _ if lease_now != self.action_lease_generation => {
                Some(ReconcileCause::Static("cancelled", "LEASE_LOST", "Control lease lost or released"))
            }
            _ => None,
        };
        if let Some(cause) = cause {
            let (state, code) = match cause {
                ReconcileCause::Static(s, c, _)
                | ReconcileCause::SafetyMessage(s, c)
                | ReconcileCause::SafetyDebug(s, c) => (s, c),
            };
            // a macro sub-motion or a replay must not outlive its action
            if (is_macro || is_replay) && carried { self.stop_motion(); }
            if is_park && self.park_continuation.is_some_and(|c| c.action_id == id) {
                self.park_continuation = None;
                self.mit.park_after_hand_back = false;
                self.mit.deferred_park_off_owner = false;
                self.abort_park_rearm_transition();
                self.notify_park_continuation();
            }
            self.pending_deferred_done = None;
            if self.rt_executor_owned && self.zero_check_persisting == Some(id) {
                // On `OWNER_EXECUTOR`, PR C owns post-handoff persistence settlement, authority
                // recheck, and `SessionSlot::ExclusiveOutput` teardown (hardware safety stop/E-stop
                // has already taken effect); do not overwrite `last_action` or drop the grant early.
                return;
            }
            if self.zero_check_persisting == Some(id) {
                self.zero_check_persisting = None;
                if self.zero_check_persisting_grant.is_some_and(|(gid, _)| gid == id) {
                    self.zero_check_persisting_grant = None;
                }
            }
            self.teardown_gripper_zero_check_session_rt(
                id,
                None,
                false,
                false,
                None,
                crate::rt_core::session::SessionTerminalReason::Revoked(code),
                now,
            );
            let mut message = self.take_action_msg_buf(id, 256);
            message.clear();
            match cause {
                ReconcileCause::Static(_, _, text) => message.push_str(text),
                ReconcileCause::SafetyMessage(_, _) => {
                    if let Some(info) = self.safety.fault() {
                        use std::fmt::Write;
                        let _ = write!(message, "{}", info.message);
                    } else {
                        message.push_str("CAN TX stalled");
                    }
                }
                ReconcileCause::SafetyDebug(_, _) => {
                    if let SafetyState::Fault(r) = self.safety.state() {
                        use std::fmt::Write;
                        let _ = write!(message, "{r:?}");
                    }
                }
            }
            #[cfg(feature = "policy-runtime")]
            if is_policy {
                let reason = match code {
                    "ESTOP_ACTIVE" => "ESTOP",
                    "WATCHDOG" => "WATCHDOG_TIMEOUT",
                    "LEASE_LOST" => "LEASE_LOST",
                    "CAN_TX_STALLED" => "CAN_TX_STALLED",
                    "J7_FEEDBACK_STALE" => "J7_FEEDBACK_STALE",
                    _ => "FAULT",
                };
                self.terminate_policy_run(reason, state, Some(code), Some(message));
                return;
            }
            self.finish_action(id, state, Some(code), Some(message), None, None, None);
            return;
        }
        if is_policy {
            return;
        }
        if self.macro_run.as_ref().is_some_and(|m| m.id == id) {
            // Post-output reconciliation through the single `enforce_macro_deadlines` entry point
            // (also enforced pre-step to avoid reference advance, and via the single per-tick
            // `effective_cutoff` at the driver TX choke point).
            if self.enforce_macro_deadlines(now) && self.macro_run.as_ref().is_none_or(|m| m.finishing) {
                return;
            }
        }
        let carried = self.active_trajectory.as_ref().is_some_and(|a| a.action_id == Some(id));
        let pending_done_for_id = self.pending_deferred_done.as_ref().is_some_and(|d| {
            matches!(
                d.target,
                DeferredCompletionTarget::Action(aid)
                    | DeferredCompletionTarget::MacroSub { action_id: aid, .. }
                    | DeferredCompletionTarget::MacroFinish { action_id: aid, .. }
                    if aid == id
            )
        });
        if let Some(run) = self.macro_run.as_ref().filter(|m| m.id == id) {
            // between sub-motions the orchestrator owns the action; it ends here only when
            // something else took the arm or a stop arrived
            let superseded = self.active_trajectory.is_some() && !carried || self.servo_target.is_some();
            if superseded {
                self.pending_deferred_done = None;
                self.finish_action(id, "cancelled", Some("SUPERSEDED"), None, None, None, None);
                return;
            }
            let pending_finish = self.pending_deferred_done.as_ref().is_some_and(|d| {
                matches!(d.target, DeferredCompletionTarget::MacroFinish { action_id, .. } if action_id == id)
            });
            if carried || pending_finish || (pending_done_for_id && !run.stop_requested && run.sub == SubState::Running) || self.zero_check_persisting == Some(id) {
                return;
            }
            // a joint-space sub-motion (sequence pose step) settles before it counts as done
            if !run.stop_requested && run.sub == SubState::Running && self.quintic_settle.as_ref().is_some_and(|(sid, _)| *sid == id) {
                if !tick_output_accepted { return; }
                let (cmd, meas, vel) = (&self.commanded_positions, &self.measured_positions, &self.measured_velocities);
                if let Some(settled) = self.quintic_settle.as_mut().and_then(|(_, settle)| settle.step(dt, cmd, meas, vel)) {
                    self.quintic_settle = None;
                    if let Some(m) = self.macro_run.as_mut() {
                        m.sub = SubState::Done(SubDone { settled: Some(settled), holding: None, gripper_gap_rad: None, at: Instant::now() });
                    }
                }
                return;
            }
            let ended: Option<(&str, &str)> = if run.sub == SubState::Running || run.stop_requested {
                Some(("cancelled", "STOPPED"))
            } else { None };
            if let Some((state, code)) = ended { self.finish_action(id, state, Some(code), None, None, None, None); }
            return;
        }
        let superseded = (self.active_trajectory.is_some() && !carried) || self.servo_target.is_some();
        if is_park {
            if self.mit.deferred_park_action == Some(id)
                || self.park_continuation.is_some_and(|c| c.action_id == id)
                || self.park_disable_in_flight_for(id)
            {
                return;
            }
            if self.pending_normal_unload {
                let progress = self.active_trajectory.as_ref().and_then(|a| a.as_quintic()).map_or(1.0, |t| (t.elapsed / t.duration).min(1.0) as f32);
                if let Some(a) = self.last_action.as_mut() { a.progress = progress; }
            } else if !self.is_torqued
                && matches!(self.safety.state(), SafetyState::Idle)
                && !self.disable_pending
                && !self.park_disable_in_flight()
            {
                self.finish_action(id, "done", None, None, None, None, None);
            } else if superseded {
                self.pending_deferred_done = None;
                self.finish_action(id, "cancelled", Some("SUPERSEDED"), None, None, None, None);
            } else {
                let mut msg = self.take_action_msg_buf(id, 64);
                msg.clear();
                msg.push_str("Park ended with torque still on");
                self.finish_action(id, "failed", Some("UNLOAD_FAILED"), Some(msg), None, None, None);
            }
            return;
        }
        if superseded {
            self.pending_deferred_done = None;
            self.finish_action(id, "cancelled", Some("SUPERSEDED"), None, None, None, None);
            return;
        }
        if carried || pending_done_for_id || self.zero_check_persisting == Some(id) { return; }
        if self.quintic_settle.as_ref().is_some_and(|(sid, _)| *sid == id) {
            if !tick_output_accepted { return; }
            let (cmd, meas, vel) = (&self.commanded_positions, &self.measured_positions, &self.measured_velocities);
            let result = self.quintic_settle.as_mut().and_then(|(_, settle)| settle.step(dt, cmd, meas, vel));
            if let Some(settled) = result {
                self.finish_action(id, "done", None, None, Some(settled), None, None);
            }
            return;
        }
        self.finish_action(id, "cancelled", Some("STOPPED"), None, None, None, None);
    }

    /// Apply one executor step of a Path / Gripper / Replay reference.
    /// Always returns a deferred completion payload on `StepStatus::Done` (both tracking enabled
    /// and disabled) so `"done"` and `releasing = false` are only published after that tick's
    /// output is accepted (`Complete`) by the driver without an intervening stop or supersession.
    pub(super) fn apply_action_step(&mut self, action_id: Option<u64>, mut out: StepOut) -> Option<DeferredActionDone> {
        // swap: the previous command's vectors go back to the scratch for the next step
        std::mem::swap(&mut self.commanded_positions, &mut out.positions);
        std::mem::swap(&mut self.commanded_velocities, &mut out.velocities);
        let status = std::mem::replace(&mut out.status, StepStatus::Running);
        let (progress, clear_gripper_stall) = (out.progress, out.clear_gripper_stall);
        self.scratch.step_out = out;
        if clear_gripper_stall {
            self.gripper_stalled = false;
            self.gripper_high_torque_ticks = 0;
            self.gripper_low_torque_ticks = 0;
            self.gripper_stall_window.reset();
        }
        let macro_sub = self.macro_run.as_ref().is_some_and(|m| Some(m.id) == action_id);
        if let (Some(id), Some(a)) = (action_id, self.last_action.as_mut()) {
            if a.id == id && a.is_running() && !macro_sub { a.progress = progress; }
        }
        match status {
            StepStatus::Running => None,
            StepStatus::Done { settled, holding, gripper_gap_rad } => {
                let is_gripper_ref = matches!(self.active_trajectory.as_ref().map(|a| &a.reference), Some(Reference::Gripper(_)));
                let is_gripper = self.active_reference_drives_gripper();
                let clear_releasing = self.releasing && is_gripper_ref;
                let target = if macro_sub {
                    match (action_id, self.macro_run.as_ref().and_then(|m| m.ticket())) {
                        (Some(id), Some(ticket)) => DeferredCompletionTarget::MacroSub { action_id: id, ticket },
                        (Some(id), None) => DeferredCompletionTarget::Action(id),
                        (None, _) => DeferredCompletionTarget::Anonymous,
                    }
                } else if let Some(id) = action_id {
                    DeferredCompletionTarget::Action(id)
                } else if self.pending_normal_unload {
                    match self.park_release {
                        Some(phase) => DeferredCompletionTarget::ParkRelease(phase),
                        None => DeferredCompletionTarget::Anonymous,
                    }
                } else {
                    DeferredCompletionTarget::Anonymous
                };
                if let Some(old_traj) = self.active_trajectory.take() {
                    self.retire_active_trajectory(old_traj);
                }
                self.commanded_velocities.fill(0.0);
                crate::rt_core::scratch::copy_into(&mut self.target_positions, &self.commanded_positions);
                Some(DeferredActionDone {
                    target,
                    epoch_id: self.safety.epoch_id(),
                    lease_generation: self.active_lease.as_ref().map(|l| l.generation),
                    settled,
                    holding,
                    gripper_gap_rad,
                    clear_releasing,
                    is_gripper,
                })
            }
            StepStatus::Failed { code, message } => {
                if code == "TRACKING_STALL" {
                    self.stop_motion();
                    self.tracking.invalidate();
                    if self.measured_positions.iter().all(|q| q.is_finite()) {
                        crate::rt_core::scratch::copy_into(&mut self.commanded_positions, &self.measured_positions);
                    }
                    self.commanded_accelerations.fill(0.0);
                } else if let Some(old_traj) = self.active_trajectory.take() {
                    self.retire_active_trajectory(old_traj);
                }
                if action_id.is_none() && self.park_release == Some(ParkRelease::Opening) {
                    tracing::warn!("Park release opening failed ({code}: {message}); skipping re-close and keeping held state");
                    self.park_release = None;
                    self.park_gripper_contact = false;
                    self.releasing = false;
                }
                self.commanded_velocities.fill(0.0);
                crate::rt_core::scratch::copy_into(&mut self.target_positions, &self.commanded_positions);
                if !self.pending_normal_unload { self.safety.mark_motion_finished(); }
                if macro_sub {
                    if let Some(m) = self.macro_run.as_mut() {
                        m.sub = SubState::Failed {
                            code: crate::safety::BoundedText::new(code),
                            message,
                        };
                    }
                } else if let Some(id) = action_id {
                    let mut msg = self.take_action_msg_buf(id, message.as_str().len().max(160));
                    msg.clear();
                    msg.push_str(message.as_str());
                    self.finish_action(id, "failed", Some(code), Some(msg), None, None, None);
                }
                None
            }
        }
    }
}
