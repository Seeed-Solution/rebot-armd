//! Core bounded executor clear-fault state machine (`clear_fault_executor_inner`).

#[allow(unused_imports)]
use super::*;
#[allow(unused_imports)]
use super::super::*;

impl SupervisorInner {
    #[allow(clippy::result_large_err)] // inline refusal keeps RT clear_fault zero-alloc
    pub(in crate::supervisor) fn clear_fault_executor_inner(
        &mut self,
        now_fn: &mut dyn FnMut() -> Instant,
        deadline: Instant,
        expected_epoch: Option<u64>,
        authority: Option<crate::rt_core::requests::lease::ClearFaultAuthority>,
        mut release_budget: Option<&mut crate::rt_core::budget::ReleaseBudget>,
        mit_recovery_started_at: Option<Instant>,
    ) -> Result<SafetyState, ClearFaultWithAuthError> {
        use crate::rt_core::requests::lease::recheck_clear_fault_authority_readonly;
        use crate::safety::{ClearFaultError as E, ClearRefusal as R};
        let apply_estop_refusal = |this: &mut Self| -> ClearFaultWithAuthError {
            this.consume_pending_estop();
            if let Some(received) = expected_epoch {
                if received != this.safety.epoch_id() {
                    return ClearFaultWithAuthError::Clear(E::EpochMismatch {
                        expected: this.safety.epoch_id(),
                        received,
                    });
                }
            }
            ClearFaultWithAuthError::Clear(E::EStopActive)
        };
        let had_driver_config_error = self.driver_config_error.is_some();
        let (candidate_limits, driver_err_override) = if had_driver_config_error {
            match self.probe_driver_limits_bounded() {
                Ok(cand) => (Some(cand), None),
                Err(err) => (None, Some(err)),
            }
        } else {
            (None, None)
        };
        let is_stopped_or_latched = self.safety.state().is_stopped() || self.safety.has_latched_fault();

        // Zero-allocation, release-bounded MIT expiry recovery on RT executor when `mit.owner == Lost`.
        // Enforces explicit confirmation (`mit_recovery == true`), Rule A′ rest-pose gate at send
        // time (`TxKind::DisableRequested { started_at }`), live authority + new-estop-since baseline
        // checks, physical disable sync on Done, and causal all-axis disabled feedback before
        // clearing `MitOwner::Lost` and the fault latch.
        #[cfg(not(target_arch = "wasm32"))]
        if self.mit.owner == crate::mit::MitOwner::Lost && is_stopped_or_latched {
            let mut pre_ctx = self.build_mit_recovery_clear_context();
            pre_ctx.driver_config_error = if candidate_limits.is_some() {
                None
            } else {
                driver_err_override.or(pre_ctx.driver_config_error)
            };
            pre_ctx.expected_epoch = expected_epoch;
            pre_ctx.pending_estop = self.estop_request_unapplied();
            let mut scratch_ages = std::mem::take(&mut self.scratch.ages);
            let mut scratch_rx = std::mem::take(&mut self.scratch.rx);
            if let Some(drv) = self.driver.as_ref().and_then(|d| d.read()) {
                drv.feedback_ages_into(&mut scratch_ages);
                let ages_valid = scratch_ages.len() == self.config.dof
                    && !scratch_ages.iter().any(|a| a.is_none_or(|t| t > THERMAL_FEEDBACK_FRESHNESS_LIMIT));
                let caps = drv.capabilities();
                pre_ctx.feedback_fresh = ages_valid && drv.is_feedback_valid();
                pre_ctx.j7_feedback_fresh = drv.pos_force().pos_force_feedback_fresh();
                pre_ctx.thermal_sensor_available = caps.temperature_feedback;
                let threshold = self.config.max_temperature - THERMAL_CLEAR_MARGIN_C;
                let thermal_below = pre_ctx.feedback_fresh
                    && caps.temperature_feedback
                    && !self.temperatures.is_empty()
                    && self.temperatures.iter().all(|t| t.is_finite() && *t <= threshold);
                let next_cd = Self::evaluate_thermal_cooldown_fields(
                    self.config.dof,
                    self.thermal_cooldown,
                    drv.as_ref(),
                    &scratch_ages,
                    thermal_below,
                    &mut scratch_rx,
                );
                pre_ctx.thermal_below_threshold = thermal_below;
                pre_ctx.thermal_cooldown_elapsed_ms = if thermal_below { next_cd.elapsed_ms() } else { 0 };
                pre_ctx.enable_state_known = true;
                pre_ctx.joint_limit_violation =
                    self.joint_limit_clear_violation_with_candidates(candidate_limits.as_ref());
            } else {
                pre_ctx.feedback_fresh = false;
                pre_ctx.j7_feedback_fresh = false;
                pre_ctx.enable_state_known = false;
            }
            self.scratch.ages = scratch_ages;
            self.scratch.rx = scratch_rx;

            if let Err(refusal) = self.safety.clear_check(&pre_ctx) {
                self.mit.recovery_baseline = None;
                if refusal == R::EStopActive {
                    return Err(apply_estop_refusal(self));
                }
                return Err(ClearFaultWithAuthError::Clear(E::from(refusal)));
            }
            // Rule A′ rest-pose check BEFORE checking confirmation or issuing any CAN TX.
            let rest_check = {
                let drv_check = self
                    .driver
                    .as_ref()
                    .and_then(|d| d.read())
                    .map(|drv| Self::check_stopped_torque_off_with_driver(&self.config, drv.as_ref()));
                match drv_check {
                    Some(res) => res,
                    None => self.eval_stopped_torque_off(),
                }
            };
            if let Err(refusal) = rest_check {
                self.mit.recovery_baseline = None;
                self.last_stopped_torque_off_refusal = Some(refusal);
                return Err(ClearFaultWithAuthError::Clear(refusal.into_clear_fault_error()));
            }
            if !self.mit.recovery_confirmed {
                self.mit.recovery_baseline = None;
                return Err(ClearFaultWithAuthError::Clear(E::Rejected(
                    R::MitRecoveryConfirmRequired,
                )));
            }
            if now_fn() >= deadline {
                self.mit.recovery_baseline = None;
                return Err(ClearFaultWithAuthError::Clear(E::Rejected(R::BudgetExpired)));
            }
            // Run bounded, zero-alloc clear-error + Rule A′ DisableRequested + causal confirmation.
            #[allow(clippy::large_enum_variant)] // stack-only RT enum avoids heap allocation
            enum MitRecStep {
                Done,
                Pending,
                TorqueOffWouldDrop(crate::supervisor::StoppedTorqueOffRefusal),
                Authority(crate::rt_core::requests::lease::ClearFaultAuthorityError),
                WatchdogBudgetInsufficient { max_frames: u16, required: u16 },
                WatchdogContinuityMissed { joint: u8 },
                Refusal(R),
            }
            let attempt_started_at = mit_recovery_started_at.unwrap_or_else(&mut *now_fn);
            let is_new_attempt = self.mit.recovery_attempt_started_at != Some(attempt_started_at);
            self.mit.recovery_attempt_started_at = Some(attempt_started_at);
            let recovery_baseline = match self.mit.recovery_baseline {
                Some(b) if !is_new_attempt => b,
                _ => {
                    let b = self.capture_baseline();
                    self.mit.recovery_baseline = Some(b);
                    b
                }
            };
            let mit_budget_rem = release_budget
                .as_deref()
                .map(|b| (b.rx_remaining(), b.tx_remaining()));
            let mut mit_rx_delta = 0u16;
            let mut mit_tx_delta = 0u16;
            let rec_step_opt = self.with_driver_and_owner_ref(|owner, drv| {
                if is_new_attempt {
                    drv.mit_mut().mit_expiry_recover_reset();
                }
                if let Some(auth) = authority {
                    if let Err(auth_err) =
                        recheck_clear_fault_authority_readonly(owner, auth, now_fn())
                    {
                        drv.mit_mut().mit_expiry_recover_reset();
                        return MitRecStep::Authority(auth_err);
                    }
                }
                if let Err(refusal) =
                    Self::check_stopped_torque_off_with_driver(&owner.config, &*drv)
                {
                    drv.mit_mut().mit_expiry_recover_reset();
                    return MitRecStep::TorqueOffWouldDrop(refusal);
                }
                if let Err((max_frames, required)) = drv.tx().check_watchdog_tx_budget_for_energized() {
                    drv.mit_mut().mit_expiry_recover_reset();
                    return MitRecStep::WatchdogBudgetInsufficient {
                        max_frames: max_frames.min(u16::MAX as usize) as u16,
                        required: required.min(u16::MAX as usize) as u16,
                    };
                }
                let Ok((drv_now, owner_now, drv_deadline)) =
                    crate::supervisor::translate_driver_cutoff_conservative(
                        drv, deadline, now_fn,
                    )
                else {
                    return MitRecStep::Refusal(R::BudgetExpired);
                };
                let elapsed_since_start =
                    owner_now.saturating_duration_since(attempt_started_at);
                let drv_started_at = drv_now
                    .checked_sub(elapsed_since_start)
                    .unwrap_or(drv_now);
                let mut auth_abort: Option<crate::rt_core::requests::lease::ClearFaultAuthorityError> = None;
                let mut estop_abort = false;
                let mut pre_tx_gate = || -> Result<(), &'static str> {
                    if owner.new_estop_since(recovery_baseline) {
                        estop_abort = true;
                        return Err("ESTOP_ACTIVE");
                    }
                    if let Some(auth) = authority {
                        if let Err(auth_err) =
                            recheck_clear_fault_authority_readonly(owner, auth, now_fn())
                        {
                            auth_abort = Some(auth_err);
                            return Err("AUTH_REVOKED");
                        }
                    }
                    Ok(())
                };
                let before_budget = drv.tx().release_budget_consumed();
                if let Some((rx_rem, tx_rem)) = mit_budget_rem {
                    drv.tx_mut().sync_release_budget_limits(rx_rem, tx_rem);
                }
                drv.set_tx_release_deadline(Some(drv_deadline));
                let r = drv.mit_mut().mit_expiry_recover_budgeted(
                    drv_deadline,
                    drv_started_at,
                    &mut pre_tx_gate,
                );
                drv.set_tx_release_deadline(None);
                let after_budget = drv.tx().release_budget_consumed();
                mit_rx_delta = after_budget.0.saturating_sub(before_budget.0);
                mit_tx_delta = after_budget.1.saturating_sub(before_budget.1);
                if let Some(auth_err) = auth_abort {
                    drv.mit_mut().mit_expiry_recover_reset();
                    return MitRecStep::Authority(auth_err);
                }
                if estop_abort {
                    drv.mit_mut().mit_expiry_recover_reset();
                    return MitRecStep::Refusal(R::EStopActive);
                }
                match r {
                    Ok(true) => {
                        drv.mit_mut().mit_expiry_recover_reset();
                        MitRecStep::Done
                    }
                    Ok(false) => MitRecStep::Pending,
                    Err(crate::driver::ArmDriverError::DisableRequestExpired) => {
                        drv.mit_mut().mit_expiry_recover_reset();
                        MitRecStep::Refusal(R::BudgetExpired)
                    }
                    Err(crate::driver::ArmDriverError::TorqueOffWouldDrop) => {
                        drv.mit_mut().mit_expiry_recover_reset();
                        let ref_err = Self::check_stopped_torque_off_with_driver(
                            &owner.config,
                            &*drv,
                        )
                        .err()
                        .unwrap_or(
                            crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback,
                        );
                        MitRecStep::TorqueOffWouldDrop(ref_err)
                    }
                    Err(crate::driver::ArmDriverError::WatchdogBudgetInsufficient {
                        max_frames,
                        required,
                    }) => {
                        drv.mit_mut().mit_expiry_recover_reset();
                        MitRecStep::WatchdogBudgetInsufficient {
                            max_frames: max_frames.min(u16::MAX as usize) as u16,
                            required: required.min(u16::MAX as usize) as u16,
                        }
                    }
                    Err(crate::driver::ArmDriverError::WatchdogContinuityMissed { joint }) => {
                        drv.mit_mut().mit_expiry_recover_reset();
                        MitRecStep::WatchdogContinuityMissed {
                            joint: joint.min(u8::MAX as usize) as u8,
                        }
                    }
                    Err(e) if crate::driver::is_estop_inhibited(&e) => {
                        drv.mit_mut().mit_expiry_recover_reset();
                        MitRecStep::Refusal(R::EStopActive)
                    }
                    Err(_) => MitRecStep::Pending,
                }
            });
            if let Some(b) = release_budget.as_deref_mut() {
                let rx_exhausted = mit_rx_delta > 0
                    && mit_budget_rem.is_some_and(|(rx_rem, _)| mit_rx_delta >= rx_rem);
                if mit_rx_delta > 0 || rx_exhausted {
                    b.charge_rx(mit_rx_delta, rx_exhausted);
                }
                if mit_tx_delta > 0 {
                    b.charge_tx(mit_tx_delta);
                }
            }
            let Some(rec_step) = rec_step_opt else {
                self.mit.recovery_baseline = None;
                return Err(ClearFaultWithAuthError::Clear(E::FeedbackUnavailable));
            };
            match rec_step {
                MitRecStep::Done => {
                    self.mit.recovery_attempt_started_at = None;
                    self.mit.recovery_baseline = None;
                    // Synchronize confirmed physical disable immediately after bus disable confirmation;
                    // logical fault clear and MIT-off state clear wait for final authority/clear_check below.
                    self.stop_motion();
                    self.servo_stream_active = false;
                    self.tracking.invalidate();
                    self.tracking.clear_recovery_suspension();
                    self.disable_accepted();
                    let _ = self.safety.set_torque_rt(false);
                    self.is_torqued = false;
                }
                MitRecStep::Pending => {
                    return Err(ClearFaultWithAuthError::Clear(E::Rejected(
                        R::MitRecoveryIncomplete,
                    )));
                }
                MitRecStep::TorqueOffWouldDrop(r) => {
                    self.mit.recovery_attempt_started_at = None;
                    self.mit.recovery_baseline = None;
                    self.last_stopped_torque_off_refusal = Some(r);
                    return Err(ClearFaultWithAuthError::Clear(r.into_clear_fault_error()));
                }
                MitRecStep::Authority(e) => {
                    self.mit.recovery_attempt_started_at = None;
                    self.mit.recovery_baseline = None;
                    return Err(ClearFaultWithAuthError::Authority(e));
                }
                MitRecStep::WatchdogBudgetInsufficient {
                    max_frames,
                    required,
                } => {
                    self.mit.recovery_attempt_started_at = None;
                    self.mit.recovery_baseline = None;
                    return Err(ClearFaultWithAuthError::Clear(E::Rejected(
                        R::WatchdogBudgetInsufficient {
                            max_frames,
                            required,
                        },
                    )));
                }
                MitRecStep::WatchdogContinuityMissed { joint } => {
                    self.mit.recovery_attempt_started_at = None;
                    self.mit.recovery_baseline = None;
                    #[cfg(not(target_arch = "wasm32"))]
                    if !crate::rt_core::executor::core_is_confirmed_disarmed(self) {
                        self.estop_state.request_and_latch();
                        self.internal_fault_lane.latch_control_safe_hold_fence();
                        self.sessions.request_safe_hold();
                    }
                    return Err(ClearFaultWithAuthError::Clear(E::Rejected(
                        R::MitRecoveryWatchdogAbort { joint },
                    )));
                }
                MitRecStep::Refusal(R::EStopActive) => {
                    self.mit.recovery_attempt_started_at = None;
                    self.mit.recovery_baseline = None;
                    return Err(apply_estop_refusal(self));
                }
                MitRecStep::Refusal(r) => {
                    self.mit.recovery_attempt_started_at = None;
                    self.mit.recovery_baseline = None;
                    return Err(ClearFaultWithAuthError::Clear(E::from(r)));
                }
            }
            // All motors causally confirmed disabled after req_boundary: rebuild the final clear
            // context directly from the fresh post-disable driver snapshot (0 RT allocs) so any
            // out-of-limit position, overtemperature sample, or carried hardware fault observed on
            // the disable confirmation gates the clear identically to legacy.
            let mut post_ctx = self.build_mit_recovery_clear_context();
            post_ctx.driver_config_error = if candidate_limits.is_some() {
                None
            } else {
                driver_err_override.or(post_ctx.driver_config_error)
            };
            post_ctx.expected_epoch = expected_epoch;
            post_ctx.pending_estop = self.estop_request_unapplied();
            post_ctx.hardware_reset_required = false;
            let mut post_cd = self.thermal_cooldown;
            let mut post_thermal_trip: Option<(usize, f32)> = None;
            let mut post_positions = [0.0f32; crate::model::MAX_MOTION_DOF];
            let mut post_pos_len = 0usize;
            let mut scratch_states = std::mem::take(&mut self.scratch.states);
            let mut scratch_ages = std::mem::take(&mut self.scratch.ages);
            let mut scratch_rx = std::mem::take(&mut self.scratch.rx);
            if let Some(drv) = self.driver.as_ref().and_then(|d| d.read()) {
                drv.feedback_ages_into(&mut scratch_ages);
                let ages_valid = scratch_ages.len() == self.config.dof
                    && !scratch_ages.iter().any(|a| a.is_none_or(|t| t > THERMAL_FEEDBACK_FRESHNESS_LIMIT));
                let caps = drv.capabilities();
                let cached_ok = drv.read_cached_joint_states_into(&mut scratch_states).is_some()
                    && scratch_states.len() >= self.config.dof;
                let states_finite = cached_ok
                    && scratch_states[..self.config.dof]
                        .iter()
                        .all(|s| s.position.is_finite() && (!caps.temperature_feedback || s.temperature.is_finite()));
                let fb_fresh = ages_valid && drv.is_feedback_valid() && (!cached_ok || states_finite);
                post_ctx.feedback_fresh = fb_fresh;
                post_ctx.j7_feedback_fresh = drv.pos_force().pos_force_feedback_fresh();
                post_ctx.thermal_sensor_available = caps.temperature_feedback;
                post_ctx.enable_state_known = true;
                post_ctx.hardware_reset_required = drv.has_hardware_fault();
                post_ctx.hardware_error = None;
                let threshold = self.config.max_temperature - THERMAL_CLEAR_MARGIN_C;
                let thermal_below = if cached_ok {
                    if fb_fresh && caps.temperature_feedback {
                        post_thermal_trip = scratch_states[..self.config.dof]
                            .iter()
                            .map(|s| s.temperature)
                            .enumerate()
                            .filter(|(_, t)| t.is_finite() && *t > self.config.max_temperature)
                            .max_by(|a, b| a.1.total_cmp(&b.1));
                    }
                    fb_fresh
                        && caps.temperature_feedback
                        && self.config.dof > 0
                        && scratch_states[..self.config.dof]
                            .iter()
                            .all(|s| s.temperature.is_finite() && s.temperature <= threshold)
                } else {
                    fb_fresh
                        && caps.temperature_feedback
                        && !self.temperatures.is_empty()
                        && self.temperatures.iter().all(|t| t.is_finite() && *t <= threshold)
                };
                post_cd = Self::evaluate_thermal_cooldown_fields(
                    self.config.dof,
                    self.thermal_cooldown,
                    drv.as_ref(),
                    &scratch_ages,
                    thermal_below,
                    &mut scratch_rx,
                );
                post_ctx.thermal_below_threshold = thermal_below;
                post_ctx.thermal_cooldown_elapsed_ms = if thermal_below { post_cd.elapsed_ms() } else { 0 };
                if cached_ok && fb_fresh && self.config.dof <= crate::model::MAX_MOTION_DOF {
                    for (dst, s) in post_positions[..self.config.dof]
                        .iter_mut()
                        .zip(&scratch_states[..self.config.dof])
                    {
                        *dst = s.position;
                    }
                    post_pos_len = self.config.dof;
                }
            } else {
                post_ctx.feedback_fresh = false;
                post_ctx.j7_feedback_fresh = false;
                post_ctx.enable_state_known = false;
            }
            self.scratch.states = scratch_states;
            self.scratch.ages = scratch_ages;
            self.scratch.rx = scratch_rx;
            if let Some((joint_idx, temperature)) = post_thermal_trip {
                self.safety.latch(
                    crate::safety::FaultKind::ThermalTrip,
                    crate::safety::FaultDetail::ThermalTrip {
                        joint_idx,
                        temperature,
                        limit: self.config.max_temperature,
                    },
                );
            }
            post_ctx.joint_limit_violation = if post_pos_len == self.config.dof {
                self.joint_limit_clear_violation_for_positions(
                    &post_positions[..post_pos_len],
                    candidate_limits.as_ref(),
                )
            } else {
                self.joint_limit_clear_violation_with_candidates(candidate_limits.as_ref())
            };
            if self.estop_request_unapplied() {
                self.consume_pending_estop();
            }
            if self.e_stop_latched.load(std::sync::atomic::Ordering::SeqCst) {
                if let Some(auth) = authority {
                    recheck_clear_fault_authority_readonly(self, auth, now_fn())
                        .map_err(ClearFaultWithAuthError::Authority)?;
                }
                if let Some(received) = expected_epoch {
                    if received != self.safety.epoch_id() {
                        return Err(ClearFaultWithAuthError::Clear(E::EpochMismatch {
                            expected: self.safety.epoch_id(),
                            received,
                        }));
                    }
                }
                self.mit.owner = crate::mit::MitOwner::Off;
                self.mit.joints.clear();
                self.mit.last_ff.clear();
                self.mit.last_output = None;
                self.mit.armed_ok = false;
                self.mit.revoke_pending = false;
                self.mit.recovery_confirmed = false;
                if let Some(d) = self.driver.as_mut() {
                    let _ = d.try_with_mut(|drv| drv.mit_mut().acknowledge_mit_fence_trip());
                }
                let _ = self.safety.clear_with_check(&post_ctx);
                self.apply_stop_policy(crate::safety::StopPolicy::EStopLatch);
                return Err(ClearFaultWithAuthError::Clear(E::EStopActive));
            }
            let commit = ClearFaultCommit {
                candidate_limits,
                reapply_legacy_limits: false,
                is_torqued: Some(false),
                reset_disarmed_hold: true,
                thermal_cooldown: Some(post_cd),
            };
            let state = self.clear_fault_finish_bounded_with_authority(
                &post_ctx,
                commit,
                authority,
                Some(now_fn()),
                |this| {
                    this.mit.owner = crate::mit::MitOwner::Off;
                    this.mit.joints.clear();
                    this.mit.last_ff.clear();
                    this.mit.last_output = None;
                    this.mit.armed_ok = false;
                    this.mit.revoke_pending = false;
                    this.mit.recovery_confirmed = false;
                    if let Some(d) = this.driver.as_mut() {
                        let _ = d.try_with_mut(|drv| drv.mit_mut().acknowledge_mit_fence_trip());
                    }
                },
            )?;
            return Ok(state);
        }

        if !is_stopped_or_latched {
            let mut ctx = self.build_clear_context_base(driver_err_override);
            if candidate_limits.is_some() {
                ctx.driver_config_error = None;
            }
            ctx.expected_epoch = expected_epoch;
            ctx.pending_estop = self.estop_request_unapplied();
            ctx.mit_lost_unsupported = self.mit.owner == crate::mit::MitOwner::Lost;
            if let Err(refusal) = self.safety.clear_check(&ctx) {
                if refusal == R::EStopActive {
                    return Err(apply_estop_refusal(self));
                }
                return Err(ClearFaultWithAuthError::Clear(E::from(refusal)));
            }
            let has_motor_errors = self.has_reported_motor_errors()
                || self
                    .driver
                    .as_ref()
                    .and_then(|d| d.read().map(|drv| drv.has_hardware_fault()))
                    .unwrap_or(false);
            if has_motor_errors {
                let state = self.clear_carried_over_motor_errors_inner(
                    now_fn,
                    Some(deadline),
                    expected_epoch,
                    authority,
                    release_budget,
                )?;
                if let Some(ref cand) = candidate_limits {
                    self.commit_driver_limits_bounded(cand);
                }
                return Ok(state);
            }
            if let Some(ref cand) = candidate_limits {
                self.commit_driver_limits_bounded(cand);
            }
            return Ok(self.safety.state().clone());
        }

        let mut ctx;
        let mut commit = ClearFaultCommit {
            candidate_limits,
            reapply_legacy_limits: false,
            is_torqued: None,
            reset_disarmed_hold: false,
            thermal_cooldown: None,
        };
        if self.driver.is_some() {
            let mut scratch_ages = std::mem::take(&mut self.scratch.ages);
            let mut scratch_rx = std::mem::take(&mut self.scratch.rx);
            let mut ws_bufs = std::mem::take(&mut self.ws_eval_bufs);
            let drv = match self.driver.as_ref().and_then(|d| d.read()) {
                Some(g) => g,
                None => {
                    self.scratch.ages = scratch_ages;
                    self.scratch.rx = scratch_rx;
                    self.ws_eval_bufs = ws_bufs;
                    return Err(ClearFaultWithAuthError::Clear(E::FeedbackUnavailable));
                }
            };
            // Pure read-only driver snapshot FIRST — ZERO motor resets or mutations.
            drv.feedback_ages_into(&mut scratch_ages);
            let ages_valid = scratch_ages.len() == self.config.dof
                && !scratch_ages.iter().any(|a| a.is_none_or(|t| t > THERMAL_FEEDBACK_FRESHNESS_LIMIT));
            let fb_valid = drv.is_feedback_valid();
            let j7_fb_fresh = drv.pos_force().pos_force_feedback_fresh();
            let tx_queue_drained = drv.tx().pending_tx_len_total() == 0;
            let tx_probe_confirmed = drv.tx().tx_probe_confirmed();
            let tx_probe_sent = drv.tx().tx_probe_pending();
            let hw_fault = drv.has_hardware_fault();
            let caps = drv.capabilities();
            let temp_fb = caps.temperature_feedback;
            let enable_fb = caps.enable_feedback;
            // M1: Resting-pose check necessity follows owner decision state (`torque_intent == On`
            // or `is_torqued`), never feedback-reported `enable_state()`.
            let needs_resting_check = (self.is_torqued || self.torque_intent == TorqueIntent::On)
                && drv.clear_fault_disables_arm_joint(self.config.arm_dof());
            let mut resting_feedback_missing = false;
            let mut resting_pose_violation = None;
            if needs_resting_check {
                let max_age = self
                    .config
                    .resting_pose
                    .as_ref()
                    .map(|p| Duration::from_millis(p.feedback_max_age_ms))
                    .unwrap_or(Duration::from_millis(250));
                if !self.feedback_numeric_valid
                    || !fb_valid
                    || scratch_ages.len() != self.config.dof
                    || !scratch_ages.iter().all(|a| a.is_some_and(|t| t <= max_age))
                {
                    resting_feedback_missing = true;
                } else {
                    resting_pose_violation = self.resting_pose_support_violation_bounded(
                        Some(drv.as_ref()),
                        &scratch_ages,
                        &mut ws_bufs,
                    );
                }
            }
            let fb_fresh = self.feedback_numeric_valid && fb_valid && ages_valid;
            // Cooldown accrues ONLY across consecutive fresh cool samples; evaluate without
            // mutating `self.thermal_cooldown` until clear commits (or ThermalCooldownNotElapsed).
            let threshold = self.config.max_temperature - THERMAL_CLEAR_MARGIN_C;
            let thermal_below = fb_fresh
                && temp_fb
                && !self.temperatures.is_empty()
                && self.temperatures.iter().all(|t| t.is_finite() && *t <= threshold);
            let next_cd = self.evaluate_thermal_cooldown_with_driver(
                drv.as_ref(),
                &scratch_ages,
                thermal_below,
                &mut scratch_rx,
            );
            drop(drv);
            self.scratch.ages = scratch_ages;
            self.scratch.rx = scratch_rx;
            self.ws_eval_bufs = ws_bufs;

            ctx = self.build_clear_context_base(driver_err_override);
            if candidate_limits.is_some() {
                ctx.driver_config_error = None;
            }
            ctx.expected_epoch = expected_epoch;
            ctx.pending_estop = self.estop_request_unapplied();
            ctx.mit_lost_unsupported = self.mit.owner == crate::mit::MitOwner::Lost;
            ctx.feedback_fresh = fb_fresh;
            ctx.j7_feedback_fresh = j7_fb_fresh;
            ctx.tx_queue_drained = tx_queue_drained;
            ctx.tx_probe_confirmed = tx_probe_confirmed;
            ctx.tx_probe_sent = tx_probe_sent;
            ctx.thermal_sensor_available = temp_fb;
            ctx.thermal_below_threshold = thermal_below;
            ctx.thermal_cooldown_elapsed_ms = if thermal_below { next_cd.elapsed_ms() } else { 0 };
            ctx.joint_limit_violation =
                self.joint_limit_clear_violation_with_candidates(candidate_limits.as_ref());
            ctx.resting_feedback_missing = resting_feedback_missing;
            ctx.resting_pose_violation = resting_pose_violation;
            ctx.hardware_reset_required = hw_fault;
            self.populate_enable_clear_context(&mut ctx, enable_fb);

            // Evaluate `clear_check` FIRST over EVERY recorded kind before any TX probe, motor
            // reset, or concurrent E-stop / config mutation.
            let mut clear_res = self.safety.clear_check(&ctx);
            if clear_res == Err(R::TxProbeRequired) {
                if now_fn() >= deadline {
                    return Err(ClearFaultWithAuthError::Clear(E::Rejected(R::BudgetExpired)));
                }
                enum BudgetedProbeAttempt<T> {
                    Ran(T),
                    BudgetExpired,
                    Authority(crate::rt_core::requests::lease::ClearFaultAuthorityError),
                }
                let budget_rem = release_budget
                    .as_deref()
                    .map(|b| (b.rx_remaining(), b.tx_remaining()));
                let mut probe_rx_delta = 0u16;
                let mut probe_tx_delta = 0u16;
                let mut scratch_ages = std::mem::take(&mut self.scratch.ages);
                let probe_outcome = self.with_driver_and_owner_ref(|owner, drv| {
                    if let Some(auth) = authority {
                        if let Err(auth_err) =
                            recheck_clear_fault_authority_readonly(owner, auth, now_fn())
                        {
                            return BudgetedProbeAttempt::Authority(auth_err);
                        }
                    }
                    // Sample driver clock FIRST, then live owner clock, and refuse expired budgets
                    // before staging any driver TX deadline so guard wait or `clock_now()` latency
                    // can never add time back to the caller's cutoff.
                    let Ok((_, _, drv_deadline)) =
                        crate::supervisor::translate_driver_cutoff_conservative(drv, deadline, now_fn)
                    else {
                        return BudgetedProbeAttempt::BudgetExpired;
                    };
                    let before_budget = drv.tx().release_budget_consumed();
                    if let Some((rx_rem, tx_rem)) = budget_rem {
                        drv.tx_mut().sync_release_budget_limits(rx_rem, tx_rem);
                    } else {
                        // Standalone off-release clear_fault call: exit any prior managed release
                        // and open a fresh TX pass before installing the release-margin deadline
                        // so a backpressured/expired pass from a prior release never blocks 0xCC.
                        drv.tx_mut().reset_standalone_tx_pass();
                    }
                    drv.set_tx_release_deadline(Some(drv_deadline));
                    let res = drv.tx_mut().probe_tx_ready_budgeted(drv_deadline);
                    drv.set_tx_release_deadline(None);
                    let after_budget = drv.tx().release_budget_consumed();
                    probe_rx_delta = after_budget.0.saturating_sub(before_budget.0);
                    probe_tx_delta = after_budget.1.saturating_sub(before_budget.1);
                    drv.feedback_ages_into(&mut scratch_ages);
                    let ages_ok = scratch_ages.len() == owner.config.dof
                        && !scratch_ages.iter().any(|a| a.is_none_or(|t| t > THERMAL_FEEDBACK_FRESHNESS_LIMIT));
                    BudgetedProbeAttempt::Ran((
                        res,
                        drv.tx().pending_tx_len_total() == 0,
                        drv.tx().tx_probe_confirmed(),
                        drv.tx().tx_probe_pending(),
                        drv.has_hardware_fault(),
                        drv.is_feedback_valid(),
                        drv.pos_force().pos_force_feedback_fresh(),
                        ages_ok,
                    ))
                });
                self.scratch.ages = scratch_ages;
                if let Some(b) = release_budget.as_deref_mut() {
                    if probe_rx_delta > 0 {
                        b.charge_rx(probe_rx_delta, false);
                    }
                    if probe_tx_delta > 0 {
                        b.charge_tx(probe_tx_delta);
                    }
                }
                let (probe_res, drained_after, probe_ok, probe_pending, hw_fault_after, fb_valid_after, j7_fresh_after, ages_valid_after) =
                    match probe_outcome
                        .ok_or(ClearFaultWithAuthError::Clear(E::FeedbackUnavailable))?
                    {
                        BudgetedProbeAttempt::Ran(v) => v,
                        BudgetedProbeAttempt::BudgetExpired => {
                            return Err(ClearFaultWithAuthError::Clear(E::Rejected(R::BudgetExpired)));
                        }
                        BudgetedProbeAttempt::Authority(e) => {
                            return Err(ClearFaultWithAuthError::Authority(e));
                        }
                    };
                ctx.tx_queue_drained = drained_after;
                self.populate_enable_clear_context(&mut ctx, enable_fb);
                ctx.hardware_reset_required = hw_fault_after;
                ctx.j7_feedback_fresh = j7_fresh_after;
                ctx.pending_estop = self.estop_request_unapplied();
                match probe_res {
                    Ok(confirmed) => {
                        ctx.tx_probe_sent = true;
                        ctx.tx_probe_confirmed = confirmed && probe_ok;
                        ctx.feedback_fresh = fb_fresh && fb_valid_after && ages_valid_after;
                    }
                    Err(e) => {
                        ctx.tx_probe_confirmed = false;
                        ctx.tx_probe_sent = probe_pending;
                        ctx.hardware_error = Some(clear_fault_driver_error(&e));
                    }
                }
                clear_res = self.safety.clear_check(&ctx);
            }
            match clear_res {
                Ok(()) => {}
                Err(R::EStopActive) => return Err(apply_estop_refusal(self)),
                Err(R::HardwareResetRequired) => {
                    // Explicitly modeled exception: every OTHER recorded kind's condition passed,
                    // so a budgeted motor reset is now permitted. Recheck live grant, lease,
                    // operator session (M3), J7 freshness, Rule A′ rest-pose gate (when the reset
                    // itself disables an arm joint, e.g. Feetech STS3215), and pending E-stop under
                    // the SAME driver lock guard / exclusive borrow before sending the reset (zero gap, zero alloc).
                    if now_fn() >= deadline {
                        return Err(ClearFaultWithAuthError::Clear(E::Rejected(R::BudgetExpired)));
                    }
                    #[allow(clippy::large_enum_variant)]
                    enum BudgetedResetAttempt {
                        Ran(Result<(), crate::driver::ArmDriverError>),
                        Refusal(R),
                        StoppedTorqueOff(crate::supervisor::StoppedTorqueOffRefusal),
                        Authority(crate::rt_core::requests::lease::ClearFaultAuthorityError),
                    }
                    let budget_rem = release_budget
                        .as_deref()
                        .map(|b| (b.rx_remaining(), b.tx_remaining()));
                    let mut reset_rx_delta = 0u16;
                    let mut reset_tx_delta = 0u16;
                    let reset_outcome = self
                        .with_driver_and_owner_ref(|owner, drv| {
                            if let Some(auth) = authority {
                                if let Err(auth_err) =
                                    recheck_clear_fault_authority_readonly(owner, auth, now_fn())
                                {
                                    return BudgetedResetAttempt::Authority(auth_err);
                                }
                            }
                            if needs_resting_check {
                                if let Err(refusal) =
                                    Self::check_stopped_torque_off_with_driver(&owner.config, &*drv)
                                {
                                    return BudgetedResetAttempt::StoppedTorqueOff(refusal);
                                }
                            }
                            ctx.j7_feedback_fresh = drv.pos_force().pos_force_feedback_fresh();
                            ctx.feedback_fresh = ctx.feedback_fresh && drv.is_feedback_valid();
                            ctx.pending_estop = owner.estop_request_unapplied();
                            if let Err(refusal) = owner.safety.clear_check(&ctx) {
                                if refusal != R::HardwareResetRequired {
                                    return BudgetedResetAttempt::Refusal(refusal);
                                }
                            }
                            // Sample driver clock FIRST, then live owner clock, and refuse expired
                            // budgets before staging any driver TX deadline.
                            let Ok((_, _, drv_deadline)) =
                                crate::supervisor::translate_driver_cutoff_conservative(
                                    drv, deadline, now_fn,
                                )
                            else {
                                return BudgetedResetAttempt::Refusal(R::BudgetExpired);
                            };
                            let before_budget = drv.tx().release_budget_consumed();
                            if let Some((rx_rem, tx_rem)) = budget_rem {
                                drv.tx_mut().sync_release_budget_limits(rx_rem, tx_rem);
                            }
                            drv.set_tx_release_deadline(Some(drv_deadline));
                            let res = drv.clear_fault_budgeted(drv_deadline);
                            drv.set_tx_release_deadline(None);
                            let after_budget = drv.tx().release_budget_consumed();
                            reset_rx_delta = after_budget.0.saturating_sub(before_budget.0);
                            reset_tx_delta = after_budget.1.saturating_sub(before_budget.1);
                            BudgetedResetAttempt::Ran(res)
                        })
                        .ok_or(ClearFaultWithAuthError::Clear(E::FeedbackUnavailable))?;
                    if let Some(b) = release_budget {
                        if reset_rx_delta > 0 {
                            b.charge_rx(reset_rx_delta, false);
                        }
                        if reset_tx_delta > 0 {
                            b.charge_tx(reset_tx_delta);
                        }
                    }
                    let reset_res = match reset_outcome {
                        BudgetedResetAttempt::Ran(res) => res,
                        BudgetedResetAttempt::Refusal(R::EStopActive) => {
                            return Err(apply_estop_refusal(self));
                        }
                        BudgetedResetAttempt::Refusal(refusal) => {
                            return Err(ClearFaultWithAuthError::Clear(E::from(refusal)));
                        }
                        BudgetedResetAttempt::StoppedTorqueOff(refusal) => {
                            self.last_stopped_torque_off_refusal = Some(refusal);
                            return Err(ClearFaultWithAuthError::Clear(
                                refusal.into_clear_fault_error(),
                            ));
                        }
                        BudgetedResetAttempt::Authority(e) => {
                            return Err(ClearFaultWithAuthError::Authority(e));
                        }
                    };
                    ctx.hardware_reset_required = false;
                    ctx.pending_estop = self.estop_request_unapplied();
                    match reset_res {
                        Ok(()) => ctx.hardware_reset_sent = true,
                        Err(
                            crate::driver::ArmDriverError::TorqueOffWouldDrop
                            | crate::driver::ArmDriverError::DisableRequestExpired,
                        ) => {
                            let r = self
                                .may_disable_torque_while_stopped()
                                .err()
                                .unwrap_or(crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback);
                            self.last_stopped_torque_off_refusal = Some(r);
                            return Err(ClearFaultWithAuthError::Clear(r.into_clear_fault_error()));
                        }
                        Err(e) => ctx.hardware_error = Some(clear_fault_driver_error(&e)),
                    }
                    let post_refusal = self.safety.clear_check(&ctx).unwrap_err();
                    if post_refusal == R::EStopActive {
                        return Err(apply_estop_refusal(self));
                    }
                    return Err(ClearFaultWithAuthError::Clear(E::from(post_refusal)));
                }
                Err(refusal) => {
                    if matches!(refusal, R::ThermalCooldownNotElapsed { .. })
                        && (!self.safety.has_latched_fault_kind(crate::safety::FaultKind::J7FeedbackStale)
                            || ctx.j7_feedback_fresh)
                    {
                        self.thermal_cooldown = next_cd;
                    }
                    return Err(ClearFaultWithAuthError::Clear(E::from(refusal)));
                }
            }
            if self.torque_intent == TorqueIntent::Off {
                commit.is_torqued = Some(false);
                commit.reset_disarmed_hold = true;
            } else if self.enable_lifecycle == EnableLifecycle::FailedHold {
                commit.is_torqued = Some(true);
            }
            commit.thermal_cooldown = Some(next_cd);
        } else {
            // Driverless simulation mode: evaluate cooldown without pre-committing (0 alloc).
            let next_cd = self.evaluate_thermal_cooldown_driverless(Instant::now());
            ctx = self.build_clear_context_base(driver_err_override);
            if candidate_limits.is_some() {
                ctx.driver_config_error = None;
            }
            ctx.expected_epoch = expected_epoch;
            ctx.pending_estop = self.estop_request_unapplied();
            ctx.mit_lost_unsupported = self.mit.owner == crate::mit::MitOwner::Lost;
            ctx.thermal_cooldown_elapsed_ms = if ctx.thermal_below_threshold {
                next_cd.elapsed_ms()
            } else {
                0
            };
            ctx.joint_limit_violation =
                self.joint_limit_clear_violation_with_candidates(candidate_limits.as_ref());
            if let Err(refusal) = self.safety.clear_check(&ctx) {
                if refusal == R::EStopActive {
                    return Err(apply_estop_refusal(self));
                }
                if matches!(refusal, R::ThermalCooldownNotElapsed { .. })
                    && (!self.safety.has_latched_fault_kind(crate::safety::FaultKind::J7FeedbackStale)
                        || ctx.j7_feedback_fresh)
                {
                    self.thermal_cooldown = next_cd;
                }
                return Err(ClearFaultWithAuthError::Clear(E::from(refusal)));
            }
            commit.thermal_cooldown = Some(next_cd);
        }
        self.clear_fault_finish_bounded_with_authority(&ctx, commit, authority, Some(now_fn()), |_| {})
    }
}
