//! Legacy fault-clearing context construction, limit probing, and authority finish.

#[allow(unused_imports)]
use super::*;
#[allow(unused_imports)]
use super::super::*;

// scaffold-extra-0
impl SupervisorInner {
    /// Builds the common zero-allocation [`crate::safety::ClearContext`] baseline from current
    /// supervisor state. Callers fill in live driver freshness, thermal capability, hardware error,
    /// enable state, and resting-pose checks before invoking [`SafetyManager::clear_check`].
    pub(crate) fn build_clear_context_base(
        &self,
        driver_config_error_override: Option<crate::safety::BoundedText<256>>,
    ) -> crate::safety::ClearContext {
        use crate::safety::{BoundedText, ClearContext};
        let threshold = self.config.max_temperature - THERMAL_CLEAR_MARGIN_C;
        let thermal_below = !self.temperatures.is_empty()
            && self.temperatures.iter().all(|t| t.is_finite() && *t <= threshold);
        let cooldown_elapsed_ms = if thermal_below {
            self.thermal_cooldown.elapsed_ms()
        } else {
            0
        };
        // An expired lease counts as NO lease for the WatchdogTimeout heartbeat condition.
        let has_active_lease = self.is_lease_active();
        let lease_heartbeat_fresh =
            has_active_lease && self.last_heartbeat.elapsed() <= self.lease_timeout;
        ClearContext {
            expected_epoch: None,
            pending_estop: self.estop_request_unapplied(),
            shutdown_committed: self.shutdown_committed,
            driver_config_error: driver_config_error_override
                .or_else(|| self.driver_config_error.as_deref().map(BoundedText::new)),
            config_reconcile_fault: self.config_reconcile_fault,
            gravity_comp_unconfirmed: self.gravity_comp_blocks_clear(),
            disable_pending: self.disable_pending,
            has_driver: self.driver.is_some(),
            feedback_fresh: self.driver.is_none(),
            j7_feedback_fresh: self.driver.is_none()
                && !self.safety.has_latched_fault_kind(crate::safety::FaultKind::J7FeedbackStale),
            tx_queue_drained: self.driver.is_none(),
            tx_probe_confirmed: self.driver.is_none(),
            tx_probe_sent: false,
            thermal_sensor_available: self.driver.is_none(),
            thermal_below_threshold: thermal_below,
            thermal_cooldown_elapsed_ms: cooldown_elapsed_ms,
            thermal_cooldown_required_ms: self.thermal_cooldown_duration.as_millis().min(u128::from(u64::MAX)) as u64,
            has_active_lease,
            lease_heartbeat_fresh,
            joint_limit_violation: self.joint_limit_clear_violation(),
            enable_state_known: self.driver.is_none(),
            failed_hold_recovery_allowed: false,
            hardware_reset_required: false,
            hardware_reset_sent: false,
            hardware_error: None,
            resting_pose_violation: None,
            resting_feedback_missing: false,
            mit_modes_uncertain: false,
            mit_lost_unsupported: false,
        }
    }

    /// Builds [`crate::safety::ClearContext`] for the MIT expiry recovery path (checks every
    /// recorded kind including `ThermalTrip`, `BusLoss`, `WatchdogTimeout`, `JointLimit`,
    /// `CollisionLatched`, `Estop`, `Other`, while deferring motor-timeout disable to recovery).
    pub(crate) fn build_mit_recovery_clear_context(&self) -> crate::safety::ClearContext {
        let mut ctx = self.build_clear_context_base(None);
        if let Some(ref d) = self.driver {
            let _ = d.with_ref(|drv| {
                ctx.mit_modes_uncertain = self.mit.owner == crate::mit::MitOwner::Lost && drv.joint_mode().joint_modes_uncertain();
                ctx.thermal_sensor_available = drv.capabilities().temperature_feedback;
                ctx.feedback_fresh = self.feedback_numeric_valid
                    && drv.is_feedback_valid()
                    && drv.feedback_all_within(self.config.dof, Duration::from_millis(250));
                ctx.j7_feedback_fresh = drv.pos_force().pos_force_feedback_fresh();
                ctx.tx_queue_drained = drv.tx().pending_tx_len_total() == 0;
                ctx.tx_probe_confirmed = drv.tx().tx_probe_confirmed();
                ctx.tx_probe_sent = drv.tx().tx_probe_pending();
                ctx.enable_state_known = true;
            });
        }
        ctx
    }

    /// Pure read-only probe of driver limits for legacy `clear_fault` (`dof > MAX_DOF` fallback
    /// clones candidate config without mutating `self`; `dof <= MAX_DOF` uses the bounded stack probe).
    #[allow(clippy::result_large_err)] // inline bounded text avoids RT heap allocation
    pub(in crate::supervisor) fn probe_driver_limits_legacy(
        &self,
    ) -> Result<Option<CandidateDriverLimits>, crate::safety::BoundedText<256>> {
        if self.driver_config_error.is_none() {
            return Ok(None);
        }
        if self.config.dof <= crate::rt_core::mailbox::MAX_DOF {
            return self.probe_driver_limits_bounded().map(Some);
        }
        use crate::safety::BoundedText;
        let dof = self.config.dof;
        let Some(ref d) = self.driver else {
            return Err(self
                .driver_config_error
                .as_deref()
                .map(BoundedText::new)
                .unwrap_or_else(|| BoundedText::new("driver unavailable while revalidating limits")));
        };
        let Some(drv) = d.read() else {
            return Err(self
                .driver_config_error
                .as_deref()
                .map(BoundedText::new)
                .unwrap_or_else(|| BoundedText::new("driver unavailable while revalidating limits")));
        };
        let mut candidate = self.config.clone();
        for (i, limit) in candidate.limits.iter_mut().enumerate().take(dof) {
            if let Some(max) = drv.max_command_velocity(i) {
                if max.is_finite() && max > 0.0 {
                    limit.max_velocity = limit.max_velocity.min(max);
                }
            }
            if let Some((d_min, d_max)) = drv.position_limits(i) {
                if !d_min.is_finite() || !d_max.is_finite() || d_min >= d_max {
                    return Err(BoundedText::new(&format!(
                        "Driver position limits [{d_min:.4}, {d_max:.4}] for joint '{}' are invalid",
                        limit.name
                    )));
                }
                let new_min = limit.min_position.max(d_min);
                let new_max = limit.max_position.min(d_max);
                if !new_min.is_finite() || !new_max.is_finite() || new_max - new_min < 1e-2 {
                    return Err(BoundedText::new(&format!(
                        "Driver position limits [{d_min:.4}, {d_max:.4}] for joint '{}' conflict with profile limits [{:.4}, {:.4}] (intersected range < 0.01)",
                        limit.name, limit.min_position, limit.max_position
                    )));
                }
                if let Some(pos) = candidate.resting_pose.as_ref().and_then(|r| r.positions.as_ref()).and_then(|p| p.get(i).copied()) {
                    if pos < new_min || pos > new_max {
                        return Err(BoundedText::new(&format!(
                            "Driver position limits [{d_min:.4}, {d_max:.4}] for joint '{}' conflict with resting pose position {pos:.4}",
                            limit.name
                        )));
                    }
                }
                limit.min_position = new_min;
                limit.max_position = new_max;
            }
        }
        drop(drv);
        if let Err(err) = candidate.validate_runtime_narrowed() {
            return Err(BoundedText::new(&format!("Driver limits violated config invariants: {err}")));
        }
        Ok(None)
    }

    pub fn clear_fault(&mut self) -> Result<SafetyState, SafetyError> {
        self.clear_fault_for_epoch(None)
    }

    /// Clear fault, optionally requiring an exact safety epoch (`expected_epoch`).
    /// `SafetyManager::clear_check` runs FIRST before any motor reset, driver limit mutation,
    /// concurrent E-stop consumption, or other side effect.
    pub fn clear_fault_for_epoch(&mut self, expected_epoch: Option<u64>) -> Result<SafetyState, SafetyError> {
        match self.clear_fault_for_epoch_with_authority(expected_epoch, None) {
            Ok(state) => Ok(state),
            Err(ClearFaultLegacyInnerError::Safety(e)) => Err(e),
            Err(ClearFaultLegacyInnerError::Authority(_)) => {
                unreachable!("authority is None on direct clear_fault_for_epoch")
            }
        }
    }

    pub(crate) fn clear_fault_for_epoch_with_authority(
        &mut self,
        expected_epoch: Option<u64>,
        authority: Option<crate::rt_core::requests::lease::ClearFaultAuthority>,
    ) -> Result<SafetyState, ClearFaultLegacyInnerError> {
        use crate::rt_core::requests::lease::recheck_clear_fault_authority_readonly;
        use crate::safety::ClearRefusal;
        let had_driver_config_error = self.driver_config_error.is_some();
        let (candidate_limits, driver_err_override, clear_driver_err_field) = match self.probe_driver_limits_legacy() {
            Ok(cand) => (cand, None, had_driver_config_error),
            Err(err) => (None, Some(err), false),
        };
        let apply_estop_refusal = |this: &mut Self| -> ClearFaultLegacyInnerError {
            this.consume_pending_estop();
            if let Some(received) = expected_epoch {
                if received != this.safety.epoch_id() {
                    return ClearFaultLegacyInnerError::Safety(SafetyError::EpochMismatch {
                        expected: this.safety.epoch_id(),
                        received,
                    });
                }
            }
            ClearFaultLegacyInnerError::Safety(SafetyError::EStopActive)
        };
        let commit_probed_limits = |this: &mut Self| {
            if let Some(ref cand) = candidate_limits {
                this.commit_driver_limits_bounded(cand);
            } else if clear_driver_err_field {
                this.reapply_driver_limits();
            }
        };

        // Run `clear_check` FIRST on universal and supervisor-known blockers (pending E-stop,
        // epoch, shutdown, driver config error, gravity comp, disable_pending, uncertain MIT
        // modes, active-lease watchdog heartbeat) before ANY side effect or bus poll (no
        // `consume_pending_estop`, no driver limit mutation, no motor reset, no MIT fence ack).
        let mit_modes_uncertain = self.mit.owner == crate::mit::MitOwner::Lost
            && self
                .driver
                .as_ref()
                .and_then(|d| d.with_ref(|drv| drv.joint_mode().joint_modes_uncertain()))
                .unwrap_or(false);
        let mut pre_ctx = self.build_clear_context_base(driver_err_override);
        if clear_driver_err_field && driver_err_override.is_none() {
            pre_ctx.driver_config_error = None;
        }
        pre_ctx.expected_epoch = expected_epoch;
        pre_ctx.pending_estop = self.estop_request_unapplied();
        pre_ctx.mit_modes_uncertain = mit_modes_uncertain;
        // Bus/telemetry-derived conditions (freshness, live temperature/cooldown, live joint
        // position, enable state) are evaluated immediately below after updating/reading live
        // telemetry; universal blockers and supervisor-known checks (including `has_active_lease`
        // / `lease_heartbeat_fresh`) run FIRST here before any bus poll or state change.
        pre_ctx.feedback_fresh = true;
        pre_ctx.j7_feedback_fresh = true;
        pre_ctx.tx_queue_drained = true;
        pre_ctx.tx_probe_confirmed = true;
        pre_ctx.thermal_sensor_available = true;
        pre_ctx.thermal_below_threshold = true;
        pre_ctx.thermal_cooldown_elapsed_ms = pre_ctx.thermal_cooldown_required_ms;
        pre_ctx.joint_limit_violation = None;
        pre_ctx.enable_state_known = true;
        if let Err(refusal) = self.safety.clear_check(&pre_ctx) {
            if refusal == ClearRefusal::EStopActive {
                return Err(apply_estop_refusal(self));
            }
            return Err(ClearFaultLegacyInnerError::Safety(refusal.into_safety_error()));
        }
        if !self.safety.state().is_stopped() && !self.safety.has_latched_fault() {
            let has_motor_errors = self.has_reported_motor_errors()
                || self
                    .driver
                    .as_ref()
                    .and_then(|d| d.read().map(|drv| drv.has_hardware_fault()))
                    .unwrap_or(false);
            if has_motor_errors {
                let mut wall_now = Instant::now;
                let state = self
                    .clear_carried_over_motor_errors_inner(
                        &mut wall_now,
                        None,
                        expected_epoch,
                        authority,
                        None,
                    )
                    .map_err(|e| match e {
                        ClearFaultWithAuthError::Authority(a) => {
                            ClearFaultLegacyInnerError::Authority(a)
                        }
                        ClearFaultWithAuthError::Clear(c) => {
                            ClearFaultLegacyInnerError::Safety(SafetyError::from(c))
                        }
                    })?;
                commit_probed_limits(self);
                return Ok(state);
            }
            commit_probed_limits(self);
            return Ok(self.safety.state().clone());
        }

        if self.mit.owner == crate::mit::MitOwner::Lost {
            // Do NOT acknowledge the MIT fence trip or mutate supervisor state before the full
            // `clear_check` passes: `mit_fresh_status()` enforces the quiet period directly while
            // leaving `mit_fence_acked == false` so autonomous CAN refreshes remain inhibited.
            let fresh = match self.driver.as_mut().and_then(|d| d.with_mut(|drv| drv.mit_mut().mit_fresh_status())) {
                Some(r) => r,
                None => Err("no driver".into()),
            };
            let st = fresh.map_err(|e| SafetyError::ClearFaultRejected(format!(
                "MIT tracking: fresh feedback is required before clearing ({e}); nothing was sent")))?;
            if st.iter().any(|&c| c >= 8) || (st.contains(&0) && st.contains(&1)) {
                // Evaluate the full MIT recovery `clear_check` read-only from the fresh status
                // snapshot — ZERO fence ack and ZERO supervisor state mutation until `clear_check`
                // and operator confirmation both pass.
                let dof = self.config.dof;
                let max_temp = self.config.max_temperature;
                let thermal_cooldown = self.thermal_cooldown;
                let snapshot = self.driver.as_mut().and_then(|d| {
                    d.with_mut(|drv| {
                        let states = drv.read_joint_states().ok()?;
                        let ages = drv.feedback_ages();
                        let loads = drv.joint_loads_snapshot();
                        let caps = drv.capabilities();
                        let modes_uncertain = drv.joint_mode().joint_modes_uncertain();
                        let tx_queue_drained = drv.tx().pending_tx_len_total() == 0;
                        let tx_probe_confirmed = drv.tx().tx_probe_confirmed();
                        let tx_probe_sent = drv.tx().tx_probe_pending();
                        let fb_fresh = drv.is_feedback_valid()
                            && ages.len() == dof
                            && ages.iter().all(|a| a.is_some_and(|t| t <= THERMAL_FEEDBACK_FRESHNESS_LIMIT))
                            && Self::states_valid(&states, caps, dof, loads.as_deref());
                        let j7_fb_fresh = drv.pos_force().pos_force_feedback_fresh();
                        let threshold = max_temp - THERMAL_CLEAR_MARGIN_C;
                        let thermal_below = fb_fresh
                            && caps.temperature_feedback
                            && !states.is_empty()
                            && states.iter().all(|s| s.temperature.is_finite() && s.temperature <= threshold);
                        let mut rx_buf = Vec::new();
                        let next_cd = Self::evaluate_thermal_cooldown_fields(
                            dof,
                            thermal_cooldown,
                            drv,
                            &ages,
                            thermal_below,
                            &mut rx_buf,
                        );
                        Some((
                            states,
                            caps.temperature_feedback,
                            fb_fresh,
                            j7_fb_fresh,
                            tx_queue_drained,
                            tx_probe_confirmed,
                            tx_probe_sent,
                            thermal_below,
                            next_cd,
                            modes_uncertain,
                        ))
                    })
                    .flatten()
                });
                let mut mit_ctx = self.build_clear_context_base(driver_err_override);
                if clear_driver_err_field && driver_err_override.is_none() {
                    mit_ctx.driver_config_error = None;
                }
                mit_ctx.expected_epoch = expected_epoch;
                mit_ctx.pending_estop = self.estop_request_unapplied();
                if let Some((ref states, temp_fb, fb_fresh, j7_fb_fresh, tx_queue_drained, tx_probe_confirmed, tx_probe_sent, thermal_below, next_cd, modes_uncertain)) = snapshot {
                    mit_ctx.mit_modes_uncertain = modes_uncertain;
                    mit_ctx.thermal_sensor_available = temp_fb;
                    mit_ctx.feedback_fresh = fb_fresh;
                    mit_ctx.j7_feedback_fresh = j7_fb_fresh;
                    mit_ctx.tx_queue_drained = tx_queue_drained;
                    mit_ctx.tx_probe_confirmed = tx_probe_confirmed;
                    mit_ctx.tx_probe_sent = tx_probe_sent;
                    mit_ctx.thermal_below_threshold = thermal_below;
                    mit_ctx.thermal_cooldown_elapsed_ms = if thermal_below {
                        next_cd.elapsed_ms()
                    } else {
                        0
                    };
                    mit_ctx.enable_state_known = true;
                    let positions: Vec<f32> = states.iter().take(self.config.dof).map(|s| s.position).collect();
                    mit_ctx.joint_limit_violation =
                        self.joint_limit_clear_violation_for_positions(&positions, candidate_limits.as_ref());
                } else {
                    mit_ctx.feedback_fresh = false;
                    mit_ctx.j7_feedback_fresh = false;
                    mit_ctx.tx_queue_drained = false;
                    mit_ctx.tx_probe_confirmed = false;
                    mit_ctx.tx_probe_sent = false;
                    mit_ctx.thermal_sensor_available = false;
                    mit_ctx.thermal_below_threshold = false;
                    mit_ctx.thermal_cooldown_elapsed_ms = 0;
                    mit_ctx.enable_state_known = false;
                    mit_ctx.joint_limit_violation =
                        self.joint_limit_clear_violation_with_candidates(candidate_limits.as_ref());
                }
                if let Err(refusal) = self.safety.clear_check(&mit_ctx) {
                    if refusal == ClearRefusal::EStopActive {
                        return Err(apply_estop_refusal(self));
                    }
                    return Err(ClearFaultLegacyInnerError::Safety(refusal.into_safety_error()));
                }
                // MIT expiry recovery disables every motor on the bus and therefore always passes
                // the single Rule A′ rest-pose decision with no override or already-off bypass.
                self.eval_stopped_torque_off()
                    .map_err(|r| r.into_safety_error(&self.config.joint_names))?;
                if !self.mit.recovery_confirmed {
                    return Err(ClearFaultLegacyInnerError::Safety(SafetyError::ClearFaultRejected(format!(
                        "{}: {}",
                        crate::mit::RECOVERY_CONFIRM_CODE,
                        crate::mit::RECOVERY_WARNING
                    ))));
                }
                // Keep operator confirmation, feedback, cooldown, and probed limits staged through
                // `mit_expiry_recovery_for_epoch`'s final `clear_with_check` decision (fail-closed:
                // zero premature commits if pre- or post-recovery clear_check refuses).
                let mit_commit = ClearFaultCommit {
                    candidate_limits,
                    reapply_legacy_limits: clear_driver_err_field && candidate_limits.is_none(),
                    is_torqued: None,
                    reset_disarmed_hold: false,
                    thermal_cooldown: None,
                };
                if let Some(auth) = authority {
                    recheck_clear_fault_authority_readonly(self, auth, Instant::now())
                        .map_err(ClearFaultLegacyInnerError::Authority)?;
                }
                return self.mit_expiry_recovery_for_epoch(
                    expected_epoch,
                    mit_ctx,
                    mit_commit,
                    driver_err_override,
                    authority,
                );
            }
        }

        let mut ctx;
        let mut commit = ClearFaultCommit {
            candidate_limits,
            reapply_legacy_limits: clear_driver_err_field && candidate_limits.is_none(),
            is_torqued: None,
            reset_disarmed_hold: false,
            thermal_cooldown: None,
        };
        let mut staged_feedback = None;
        if self.driver.is_some() {
            let mut ws_bufs = std::mem::take(&mut self.ws_eval_bufs);
            let dof = self.config.dof;
            let arm_dof = self.config.arm_dof();
            let max_temp = self.config.max_temperature;
            let max_rest_age = self
                .resting_policy()
                .map(|p| Duration::from_millis(p.feedback_max_age_ms))
                .unwrap_or(Duration::from_millis(250));
            let is_torqued = self.is_torqued;
            let torque_intent = self.torque_intent;
            let thermal_cooldown = self.thermal_cooldown;
            let fallback_loads = &self.loads;
            let (
                states,
                ages,
                loads,
                caps,
                hw_fault,
                mut j7_fb_fresh,
                tx_queue_drained,
                tx_probe_confirmed,
                tx_probe_sent,
                fb_fresh,
                needs_resting_check,
                resting_fb_ok,
                staged_positions,
                staged_velocities,
                staged_torques,
                staged_loads_vec,
                thermal_below,
                next_cd,
            ) = {
                let Some(ref mut d) = self.driver else { unreachable!() };
                let Some(mut drv) = d.write() else {
                    self.ws_eval_bufs = ws_bufs;
                    return Err(ClearFaultLegacyInnerError::Safety(SafetyError::FeedbackUnavailable));
                };
                // Pure read-only telemetry & status snapshot FIRST — ZERO motor resets or side effects.
                let states = match drv.read_joint_states() {
                    Ok(s) => s,
                    Err(e) => {
                        drop(drv);
                        self.ws_eval_bufs = ws_bufs;
                        return Err(ClearFaultLegacyInnerError::Safety(SafetyError::ClearFaultRejected(e.to_string())));
                    }
                };
                let ages = drv.feedback_ages();
                let loads = drv.joint_loads_snapshot();
                let caps = drv.capabilities();
                let disabling = drv.clear_fault_disables_joints();
                let hw_fault = drv.check_hardware_status().is_err();
                let fb_valid = drv.is_feedback_valid();
                let j7_fb_fresh = drv.pos_force().pos_force_feedback_fresh();
                let tx_queue_drained = drv.tx().pending_tx_len_total() == 0;
                let tx_probe_confirmed = drv.tx().tx_probe_confirmed();
                let tx_probe_sent = drv.tx().tx_probe_pending();
                let states_ok = Self::states_valid(&states, caps, dof, loads.as_deref());
                let fb_fresh = fb_valid
                    && ages.len() == dof
                    && !ages.iter().any(|a| a.is_none_or(|t| t > Duration::from_millis(250)))
                    && states_ok;
                // M1: Resting-pose check necessity follows owner decision state (`torque_intent == On`
                // or `is_torqued`), never feedback-reported `enable_state()`.
                let needs_resting_check = (is_torqued || torque_intent == TorqueIntent::On)
                    && disabling.iter().any(|&j| j < arm_dof);
                let resting_fb_ok = fb_valid
                    && ages.len() == dof
                    && ages.iter().all(|a| a.is_some_and(|t| t <= max_rest_age))
                    && states_ok;

                // Stage sampled feedback and thermal cooldown WITHOUT mutating supervisor state
                // before `clear_check` / `clear_with_check` succeeds.
                let staged_positions: Vec<f32> = states.iter().take(dof).map(|s| s.position).collect();
                let staged_velocities: Vec<f32> = states.iter().take(dof).map(|s| s.velocity).collect();
                let staged_torques: Vec<f32> = states.iter().take(dof).map(|s| s.torque).collect();
                let staged_loads_vec: Vec<f32> = match loads.as_deref() {
                    Some(ls) => ls.iter().take(dof).copied().collect(),
                    None => fallback_loads.clone(),
                };
                let threshold = max_temp - THERMAL_CLEAR_MARGIN_C;
                let thermal_below = fb_fresh
                    && caps.temperature_feedback
                    && !states.is_empty()
                    && states.iter().all(|s| s.temperature.is_finite() && s.temperature <= threshold);
                // Cooldown accrues ONLY across consecutive fresh cool samples; stale/unavailable/gapped temp resets it.
                let mut rx_buf = Vec::new();
                let next_cd = Self::evaluate_thermal_cooldown_fields(
                    dof,
                    thermal_cooldown,
                    drv.as_ref(),
                    &ages,
                    thermal_below,
                    &mut rx_buf,
                );
                (
                    states,
                    ages,
                    loads,
                    caps,
                    hw_fault,
                    j7_fb_fresh,
                    tx_queue_drained,
                    tx_probe_confirmed,
                    tx_probe_sent,
                    fb_fresh,
                    needs_resting_check,
                    resting_fb_ok,
                    staged_positions,
                    staged_velocities,
                    staged_torques,
                    staged_loads_vec,
                    thermal_below,
                    next_cd,
                )
            };

            let mut resting_feedback_missing = false;
            let mut resting_pose_violation = None;
            if needs_resting_check {
                if !resting_fb_ok {
                    resting_feedback_missing = true;
                } else {
                    resting_pose_violation = self.resting_pose_support_violation_for_feedback_caps(
                        Some(caps),
                        &ages,
                        &staged_positions,
                        &staged_velocities,
                        &staged_torques,
                        &staged_loads_vec,
                        true,
                        &mut ws_bufs,
                    );
                }
            }
            self.ws_eval_bufs = ws_bufs;

            ctx = self.build_clear_context_base(driver_err_override);
            if clear_driver_err_field && driver_err_override.is_none() {
                ctx.driver_config_error = None;
            }
            ctx.expected_epoch = expected_epoch;
            ctx.pending_estop = self.estop_request_unapplied();
            ctx.feedback_fresh = fb_fresh;
            ctx.j7_feedback_fresh = j7_fb_fresh;
            ctx.tx_queue_drained = tx_queue_drained;
            ctx.tx_probe_confirmed = tx_probe_confirmed;
            ctx.tx_probe_sent = tx_probe_sent;
            ctx.thermal_sensor_available = caps.temperature_feedback;
            ctx.thermal_below_threshold = thermal_below;
            ctx.thermal_cooldown_elapsed_ms = if thermal_below { next_cd.elapsed_ms() } else { 0 };
            ctx.joint_limit_violation = if fb_fresh {
                self.joint_limit_clear_violation_for_positions(&staged_positions, candidate_limits.as_ref())
            } else {
                self.joint_limit_clear_violation_with_candidates(candidate_limits.as_ref())
            };
            ctx.resting_feedback_missing = resting_feedback_missing;
            ctx.resting_pose_violation = resting_pose_violation;
            ctx.hardware_reset_required = hw_fault;
            self.populate_enable_clear_context(&mut ctx, caps.enable_feedback);

            // Evaluate `clear_check` FIRST over EVERY recorded kind before any TX probe or motor
            // reset side effect.
            let mut clear_res = self.safety.clear_check(&ctx);
            if clear_res == Err(ClearRefusal::TxProbeRequired) {
                // Every OTHER recorded kind's condition passed and the software TX queue is empty:
                // run one read-only refresh/status TX probe and verify fresh post-probe feedback.
                let (probe_res, drained_after, probe_ok, probe_pending, hw_fault_after, fb_valid_after, j7_fresh_after, ages_after) =
                    self.with_driver_and_owner_ref(|owner, drv| {
                        if let Some(auth) = authority {
                            recheck_clear_fault_authority_readonly(owner, auth, Instant::now())?;
                        }
                        let res = drv.tx_mut().probe_tx_ready();
                        Ok((
                            res,
                            drv.tx().pending_tx_len_total() == 0,
                            drv.tx().tx_probe_confirmed(),
                            drv.tx().tx_probe_pending(),
                            drv.check_hardware_status().is_err(),
                            drv.is_feedback_valid(),
                            drv.pos_force().pos_force_feedback_fresh(),
                            drv.feedback_ages(),
                        ))
                    })
                    .ok_or(ClearFaultLegacyInnerError::Safety(SafetyError::FeedbackUnavailable))?
                    .map_err(ClearFaultLegacyInnerError::Authority)?;
                j7_fb_fresh = j7_fresh_after;
                ctx.tx_queue_drained = drained_after;
                self.populate_enable_clear_context(&mut ctx, caps.enable_feedback);
                ctx.hardware_reset_required = hw_fault_after;
                ctx.j7_feedback_fresh = j7_fb_fresh;
                ctx.pending_estop = self.estop_request_unapplied();
                match probe_res {
                    Ok(confirmed) => {
                        ctx.tx_probe_sent = true;
                        ctx.tx_probe_confirmed = confirmed && probe_ok;
                        let fresh_after = fb_valid_after
                            && ages_after.len() == self.config.dof
                            && !ages_after.iter().any(|a| a.is_none_or(|t| t > Duration::from_millis(250)));
                        ctx.feedback_fresh = fb_fresh && fresh_after;
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
                Err(ClearRefusal::EStopActive) => return Err(apply_estop_refusal(self)),
                Err(ClearRefusal::HardwareResetRequired) => {
                    // Explicitly modeled exception: every OTHER recorded kind's condition passed,
                    // so transmitting the motor fault reset is now permitted. Recheck live grant,
                    // lease, operator session (M3), J7 freshness, Rule A′ rest-pose gate (when the
                    // reset itself disables an arm joint, e.g. Feetech STS3215), and pending E-stop
                    // under the SAME driver lock guard / exclusive borrow before sending the reset (zero gap).
                    #[allow(clippy::large_enum_variant)]
                    enum ResetAttempt {
                        Ran(Result<(), crate::driver::ArmDriverError>),
                        Refusal(ClearRefusal),
                        StoppedTorqueOff(crate::supervisor::StoppedTorqueOffRefusal),
                        Authority(crate::rt_core::requests::lease::ClearFaultAuthorityError),
                    }
                    let reset_outcome = self
                        .with_driver_and_owner_ref(|owner, drv| {
                            if let Some(auth) = authority {
                                if let Err(auth_err) =
                                    recheck_clear_fault_authority_readonly(owner, auth, Instant::now())
                                {
                                    return ResetAttempt::Authority(auth_err);
                                }
                            }
                            if needs_resting_check {
                                if let Err(r) =
                                    Self::check_stopped_torque_off_with_driver(&owner.config, &*drv)
                                {
                                    return ResetAttempt::StoppedTorqueOff(r);
                                }
                            }
                            ctx.j7_feedback_fresh = drv.pos_force().pos_force_feedback_fresh();
                            ctx.feedback_fresh = ctx.feedback_fresh && drv.is_feedback_valid();
                            ctx.pending_estop = owner.estop_request_unapplied();
                            if let Err(refusal) = owner.safety.clear_check(&ctx) {
                                if refusal != ClearRefusal::HardwareResetRequired {
                                    return ResetAttempt::Refusal(refusal);
                                }
                            }
                            ResetAttempt::Ran(drv.clear_fault())
                        })
                        .ok_or(ClearFaultLegacyInnerError::Safety(SafetyError::FeedbackUnavailable))?;
                    let reset_res = match reset_outcome {
                        ResetAttempt::Ran(res) => res,
                        ResetAttempt::Refusal(ClearRefusal::EStopActive) => {
                            return Err(apply_estop_refusal(self));
                        }
                        ResetAttempt::Refusal(refusal) => {
                            return Err(ClearFaultLegacyInnerError::Safety(refusal.into_safety_error()));
                        }
                        ResetAttempt::StoppedTorqueOff(r) => {
                            self.last_stopped_torque_off_refusal = Some(r);
                            return Err(ClearFaultLegacyInnerError::Safety(
                                r.into_safety_error(&self.config.joint_names),
                            ));
                        }
                        ResetAttempt::Authority(e) => {
                            return Err(ClearFaultLegacyInnerError::Authority(e));
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
                            return Err(ClearFaultLegacyInnerError::Safety(
                                r.into_safety_error(&self.config.joint_names),
                            ));
                        }
                        Err(e) => ctx.hardware_error = Some(clear_fault_driver_error(&e)),
                    }
                    let post_refusal = self.safety.clear_check(&ctx).unwrap_err();
                    if post_refusal == ClearRefusal::EStopActive {
                        return Err(apply_estop_refusal(self));
                    }
                    return Err(ClearFaultLegacyInnerError::Safety(post_refusal.into_safety_error()));
                }
                Err(refusal) => {
                    if matches!(refusal, ClearRefusal::ThermalCooldownNotElapsed { .. })
                        && (!self.safety.has_latched_fault_kind(crate::safety::FaultKind::J7FeedbackStale)
                            || ctx.j7_feedback_fresh)
                    {
                        self.thermal_cooldown = next_cd;
                    }
                    return Err(ClearFaultLegacyInnerError::Safety(refusal.into_safety_error()));
                }
            }
            if fb_fresh {
                staged_feedback = Some((states, loads, ages));
            }
            if self.torque_intent == TorqueIntent::Off {
                commit.is_torqued = Some(false);
                commit.reset_disarmed_hold = true;
            } else if self.enable_lifecycle == EnableLifecycle::FailedHold {
                commit.is_torqued = Some(true);
            }
            commit.thermal_cooldown = Some(next_cd);
        } else {
            let next_cd = self.evaluate_thermal_cooldown_driverless(Instant::now());
            ctx = self.build_clear_context_base(driver_err_override);
            if clear_driver_err_field && driver_err_override.is_none() {
                ctx.driver_config_error = None;
            }
            ctx.expected_epoch = expected_epoch;
            ctx.pending_estop = self.estop_request_unapplied();
            ctx.thermal_cooldown_elapsed_ms = if ctx.thermal_below_threshold {
                next_cd.elapsed_ms()
            } else {
                0
            };
            ctx.joint_limit_violation =
                self.joint_limit_clear_violation_with_candidates(candidate_limits.as_ref());
            if let Err(refusal) = self.safety.clear_check(&ctx) {
                if refusal == ClearRefusal::EStopActive {
                    return Err(apply_estop_refusal(self));
                }
                if matches!(refusal, ClearRefusal::ThermalCooldownNotElapsed { .. })
                    && (!self.safety.has_latched_fault_kind(crate::safety::FaultKind::J7FeedbackStale)
                        || ctx.j7_feedback_fresh)
                {
                    self.thermal_cooldown = next_cd;
                }
                return Err(ClearFaultLegacyInnerError::Safety(refusal.into_safety_error()));
            }
            commit.thermal_cooldown = Some(next_cd);
        }
        self.clear_fault_finish_bounded_with_authority(&ctx, commit, authority, None, move |this| {
            if let Some((states, loads, ages)) = staged_feedback {
                this.feedback_numeric_valid = true;
                this.last_can_feedback_instant = ages
                    .iter()
                    .copied()
                    .try_fold(Duration::ZERO, |m, a| a.map(|t| m.max(t)))
                    .and_then(|age| Instant::now().checked_sub(age));
                if let Some(ref d) = this.driver {
                    let _ = d.with_ref(|drv| {
                        this.core.poll_feedback(
                            &mut this.joint_feedback_queue_bounds,
                            this.config.dof,
                            drv,
                            &mut this.scratch.fb,
                        );
                    });
                }
                Self::apply_feedback(
                    &mut this.measured_positions,
                    &mut this.measured_velocities,
                    &mut this.torques,
                    &mut this.temperatures,
                    &mut this.loads,
                    this.config.dof,
                    &states,
                    loads.as_deref(),
                );
            }
        })
        .map_err(|e| match e {
            ClearFaultWithAuthError::Authority(a) => ClearFaultLegacyInnerError::Authority(a),
            ClearFaultWithAuthError::Clear(c) => ClearFaultLegacyInnerError::Safety(SafetyError::from(c)),
        })
    }

    #[allow(clippy::result_large_err)] // inline refusal keeps RT clear_fault zero-alloc
    pub(in crate::supervisor) fn clear_fault_finish_bounded_with_authority(
        &mut self,
        ctx: &crate::safety::ClearContext,
        commit: ClearFaultCommit,
        authority: Option<crate::rt_core::requests::lease::ClearFaultAuthority>,
        now_override: Option<Instant>,
        on_committed: impl FnOnce(&mut Self),
    ) -> Result<SafetyState, ClearFaultWithAuthError> {
        use crate::rt_core::requests::lease::recheck_clear_fault_authority_readonly;
        use crate::safety::{ClearFaultError, ClearRefusal, FaultKind};
        if self.internal_fault_lane.is_fenced() || self.shutdown_committed {
            return Err(ClearFaultWithAuthError::Clear(ClearFaultError::Rejected(ClearRefusal::ShutdownCommitted)));
        }
        // Build live pre-commit ClearContext right before committing so any concurrent E-stop,
        // authority revocation (M3), or J7 freshness expiry between preflight/probe and commit
        // fails closed with ZERO mutations.
        let mut final_ctx = *ctx;
        let was_estop = *self.safety.state() == SafetyState::EStop;
        let state = if let Some(ref d) = self.driver {
            let drv = match d.read() {
                Some(g) => g,
                None => {
                    final_ctx.j7_feedback_fresh = false;
                    final_ctx.feedback_fresh = false;
                    final_ctx.pending_estop = self.estop_request_unapplied();
                    return Err(ClearFaultWithAuthError::Clear(
                        self.safety
                            .clear_check(&final_ctx)
                            .err()
                            .map_or(ClearFaultError::FeedbackUnavailable, ClearFaultError::from),
                    ));
                }
            };
            if let Some(auth) = authority {
                let check_now = now_override.unwrap_or_else(Instant::now);
                if let Err(auth_err) = recheck_clear_fault_authority_readonly(self, auth, check_now) {
                    drop(drv);
                    return Err(ClearFaultWithAuthError::Authority(auth_err));
                }
            }
            final_ctx.pending_estop = self.estop_request_unapplied();
            if self.safety.has_latched_fault_kind(FaultKind::J7FeedbackStale) {
                final_ctx.j7_feedback_fresh = drv.pos_force().pos_force_feedback_fresh();
                final_ctx.feedback_fresh = final_ctx.feedback_fresh && drv.is_feedback_valid();
            }
            if let Err(refusal) = self.safety.clear_check(&final_ctx) {
                drop(drv);
                if refusal == ClearRefusal::EStopActive {
                    self.consume_pending_estop();
                    if let Some(received) = final_ctx.expected_epoch {
                        if received != self.safety.epoch_id() {
                            return Err(ClearFaultWithAuthError::Clear(ClearFaultError::EpochMismatch {
                                expected: self.safety.epoch_id(),
                                received,
                            }));
                        }
                    }
                    return Err(ClearFaultWithAuthError::Clear(ClearFaultError::EStopActive));
                }
                return Err(ClearFaultWithAuthError::Clear(ClearFaultError::from(refusal)));
            }
            if self.estop_request_unapplied() {
                drop(drv);
                self.consume_pending_estop();
                if let Some(received) = final_ctx.expected_epoch {
                    if received != self.safety.epoch_id() {
                        return Err(ClearFaultWithAuthError::Clear(ClearFaultError::EpochMismatch {
                            expected: self.safety.epoch_id(),
                            received,
                        }));
                    }
                }
                return Err(ClearFaultWithAuthError::Clear(ClearFaultError::EStopActive));
            }
            // Commit the safety clear under the SAME driver lock guard before applying staged
            // limits, torque flag, or cleanup (zero check-to-commit gap).
            self.safety
                .clear_with_check(&final_ctx)
                .map_err(|r| ClearFaultWithAuthError::Clear(ClearFaultError::from(r)))?
        } else {
            if let Some(auth) = authority {
                let check_now = now_override.unwrap_or_else(Instant::now);
                recheck_clear_fault_authority_readonly(self, auth, check_now)
                    .map_err(ClearFaultWithAuthError::Authority)?;
            }
            final_ctx.pending_estop = self.estop_request_unapplied();
            if self.safety.has_latched_fault_kind(FaultKind::J7FeedbackStale) {
                final_ctx.j7_feedback_fresh = false;
            }
            if let Err(refusal) = self.safety.clear_check(&final_ctx) {
                if refusal == ClearRefusal::EStopActive {
                    self.consume_pending_estop();
                    if let Some(received) = final_ctx.expected_epoch {
                        if received != self.safety.epoch_id() {
                            return Err(ClearFaultWithAuthError::Clear(ClearFaultError::EpochMismatch {
                                expected: self.safety.epoch_id(),
                                received,
                            }));
                        }
                    }
                    return Err(ClearFaultWithAuthError::Clear(ClearFaultError::EStopActive));
                }
                return Err(ClearFaultWithAuthError::Clear(ClearFaultError::from(refusal)));
            }
            if self.consume_pending_estop() {
                if let Some(received) = final_ctx.expected_epoch {
                    if received != self.safety.epoch_id() {
                        return Err(ClearFaultWithAuthError::Clear(ClearFaultError::EpochMismatch {
                            expected: self.safety.epoch_id(),
                            received,
                        }));
                    }
                }
                return Err(ClearFaultWithAuthError::Clear(ClearFaultError::EStopActive));
            }
            self.safety
                .clear_with_check(&final_ctx)
                .map_err(|r| ClearFaultWithAuthError::Clear(ClearFaultError::from(r)))?
        };

        let clear_estop_gen = self.applied_estop_count;
        on_committed(self);
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.torque_shadow
                .enqueue(crate::torque::shadow::ShadowInputKind::FaultClear {
                    live_refusal: None,
                    live_intent: self.shadow_live_intent(),
                    live_is_torqued: self.is_torqued,
                    live_stopped: self.safety.state().is_stopped(),
                });
            if !self.rt_executor_owned {
                self.flush_torque_shadow();
            }
        }
        if let Some(ref cand) = commit.candidate_limits {
            self.commit_driver_limits_bounded(cand);
        } else if commit.reapply_legacy_limits {
            self.reapply_driver_limits();
        }
        if let Some(is_torqued) = commit.is_torqued {
            self.is_torqued = is_torqued;
        }
        if commit.reset_disarmed_hold {
            self.enable_lifecycle = EnableLifecycle::Disarmed;
            self.measured_hold.clear();
        }
        if let Some(cd) = commit.thermal_cooldown {
            self.thermal_cooldown = cd;
        }
        let was_mit_lost = self.mit.owner == crate::mit::MitOwner::Lost;
        self.pending_rearm = true;
        self.stop_motion();
        self.servo_stream_active = false;
        self.tracking.invalidate();
        self.tracking.clear_recovery_suspension();
        self.reset_tx_joint_progress();
        self.reset_hold_tx_joint_progress();
        self.hold_tx_stalled_latched = false;
        self.last_hold_tx_stall_log = None;
        self.j7_stale_since = None;
        // Preserve `self.last_hold_tx_error_log` across fault clears so rapid clear/re-fault
        // cycles cannot bypass the 1/s stopped-hold error log rate limit.
        self.pending_deferred_done = None;
        let mut synced_status: Option<(EnableState, [Option<&'static str>; crate::rt_core::mailbox::MAX_DOF])> = None;
        let mut scratch_status = std::mem::take(&mut self.scratch.status);
        if let Some(ref mut d) = self.driver {
            let _ = d.with_mut(|drv| {
                drv.tx_mut().discard_pending_tx();
                drv.tx_mut().clear_tx_probe();
                if drv.motor_status_into(&mut scratch_status) {
                    let mut names = [None; crate::rt_core::mailbox::MAX_DOF];
                    for (slot, &code) in names.iter_mut().zip(scratch_status.iter()) {
                        *slot = drv.decode_motor_error_static(code);
                    }
                    synced_status = Some((drv.enable_state(), names));
                }
            });
        }
        if let Some((en_state, names)) = synced_status {
            self.apply_synced_motor_status_bounded(&scratch_status, &names, en_state);
        }
        self.scratch.status = scratch_status;
        // Reset at the measured position; never retain a residual old trajectory target.
        copy_into(&mut self.commanded_positions, &self.measured_positions);
        copy_into(&mut self.target_positions, &self.measured_positions);
        self.commanded_velocities.fill(0.0);
        self.last_commanded_velocities.fill(0.0);
        self.commanded_accelerations.fill(0.0);
        self.following_error_since = None;
        self.watchdog_park_blocked_reason = None;
        self.bus_loss_instant = None;
        self.output_suspended = false;
        self.reset_gripper_stall();
        self.handover_settling_since = None;
        if let Some(hook) = self.before_estop_unlatch_hook.take() {
            hook();
        }
        if was_mit_lost {
            // Only after full clear_with_check passed: release MIT lost ownership and acknowledge
            // the driver MIT fence BEFORE any E-stop-latch early return so a cleared MIT fault
            // never re-latches `MIT_LOST` over `SafetyState::EStop` on the next control tick.
            self.mit.owner = crate::mit::MitOwner::Off;
            self.mit.joints.clear();
            self.mit.last_ff.clear();
            self.mit.last_output = None;
            self.mit.armed_ok = false;
            self.mit.revoke_pending = false;
            self.mit.recovery_confirmed = false;
            if let Some(ref mut d) = self.driver {
                if self.rt_executor_owned {
                    let _ = d.try_with_mut(|drv| drv.mit_mut().acknowledge_mit_fence_trip());
                } else {
                    let _ = d.with_mut(|drv| drv.mit_mut().acknowledge_mit_fence_trip());
                }
            }
        }
        if !was_estop {
            // Clearing a non-E-stop fault must never overwrite a prior or concurrent E-stop latch.
            if self.consume_pending_estop() || self.estop_state.is_latched() {
                if self.safety.state() != &SafetyState::EStop {
                    self.apply_stop_policy(StopPolicy::EStopLatch);
                }
                self.last_heartbeat = Instant::now();
                self.last_motion_frame = Instant::now();
                return Ok(self.safety.state().clone());
            }
            self.shutdown_refused_unknown = false;
        } else {
            // Atomically clear bit 0 ONLY if no newer E-stop has advanced `generation` past
            // `clear_estop_gen`. A concurrent newer E-stop causes CAS failure without ever
            // transiently clearing the latch bit.
            match self.estop_state.try_clear_if_generation(clear_estop_gen) {
                Ok(()) => {
                    self.shutdown_refused_unknown = false;
                    if self.consume_pending_estop() {
                        // A concurrent E-stop arrived strictly AFTER the CAS unlatched the prior
                        // E-stop; keep the newly applied E-stop state and return Ok(EStop).
                        self.last_heartbeat = Instant::now();
                        self.last_motion_frame = Instant::now();
                        return Ok(self.safety.state().clone());
                    }
                }
                Err(_newer_gen) => {
                    // A newer E-stop arrived before unlatch: latch bit stayed true throughout
                    // (zero transient-false window). Apply the newer E-stop immediately.
                    let _ = self.consume_pending_estop();
                    if self.safety.state() != &SafetyState::EStop {
                        self.apply_stop_policy(StopPolicy::EStopLatch);
                    }
                    self.last_heartbeat = Instant::now();
                    self.last_motion_frame = Instant::now();
                    return Ok(self.safety.state().clone());
                }
            }
        }
        // If a valid lease was acquired while E-stop was latched, its servo stream could not be
        // authorized until E-stop cleared; authorize it now when no policy stream owns targets.
        if self.core.target_authority().stream.is_none() {
            let check_now = now_override.unwrap_or_else(Instant::now);
            let active_gen = self.active_lease_gen_at(check_now);
            if active_gen != 0 {
                self.authorize_servo_stream_for_committed_lease(active_gen);
            }
        }
        self.last_heartbeat = Instant::now();
        self.last_motion_frame = Instant::now();
        Ok(state)
    }
}
