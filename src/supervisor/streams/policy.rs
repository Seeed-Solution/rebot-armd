//! Policy runtime stream preconditions, live/shadow step execution, and teardown.

#[allow(unused_imports)]
use super::*;
#[allow(unused_imports)]
use super::super::*;

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
                let ages = drv.feedback_ages_snapshot();
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
                let ages = drv.feedback_ages_snapshot();
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

    pub(super) fn begin_policy_run_inner(&mut self, req: crate::policy_runtime::PolicyRunStart) -> Result<u64, ArmError> {
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
