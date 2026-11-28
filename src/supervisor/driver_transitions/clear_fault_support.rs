//! Resting-pose support violation checks and executor clear-fault wrappers/carryover.

#[allow(unused_imports)]
use super::*;
#[allow(unused_imports)]
use super::super::*;

impl SupervisorInner {
    /// Allocation-free bounded equivalent of [`Self::resting_pose_support_violation_with_driver`].
    pub(crate) fn resting_pose_support_violation_bounded(
        &self,
        drv: Option<&dyn ArmDriver>,
        ages: &[Option<Duration>],
        bufs: &mut crate::chain::EvalBufs,
    ) -> Option<crate::safety::RestingSupportViolation> {
        self.resting_pose_support_violation_for_feedback(
            drv,
            ages,
            &self.measured_positions,
            &self.measured_velocities,
            &self.torques,
            &self.loads,
            self.feedback_numeric_valid,
            bufs,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub(in crate::supervisor) fn resting_pose_support_violation_for_feedback(
        &self,
        drv: Option<&dyn ArmDriver>,
        ages: &[Option<Duration>],
        positions: &[f32],
        velocities: &[f32],
        torques: &[f32],
        loads: &[f32],
        feedback_numeric_valid: bool,
        bufs: &mut crate::chain::EvalBufs,
    ) -> Option<crate::safety::RestingSupportViolation> {
        self.resting_pose_support_violation_for_feedback_caps(
            drv.map(ArmDriver::capabilities),
            ages,
            positions,
            velocities,
            torques,
            loads,
            feedback_numeric_valid,
            bufs,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub(in crate::supervisor) fn resting_pose_support_violation_for_feedback_caps(
        &self,
        drv_caps: Option<DriverCapabilities>,
        ages: &[Option<Duration>],
        positions: &[f32],
        velocities: &[f32],
        torques: &[f32],
        loads: &[f32],
        feedback_numeric_valid: bool,
        bufs: &mut crate::chain::EvalBufs,
    ) -> Option<crate::safety::RestingSupportViolation> {
        use crate::safety::{RestingSupportJointItem as Item, RestingSupportViolation as V, MAX_RESTING_SUPPORT_ITEMS};
        let dof = self.config.dof;
        if positions.len() != dof || velocities.len() != dof {
            return Some(V::FeedbackLengthMismatch);
        }
        let Some(policy) = self.config.resting_pose.as_ref() else { return Some(V::NoRestingPoseConfigured); };
        if self.driver.is_some() {
            let Some(caps) = drv_caps else { return Some(V::DriverFeedbackUnavailable); };
            if !caps.velocity_feedback { return Some(V::DriverLacksVelocityFeedback); }
            if !policy.support_torque_limits.is_empty() && !caps.torque_feedback { return Some(V::DriverLacksTorqueFeedback); }
            if !policy.support_load_limits.is_empty() && !caps.load_feedback { return Some(V::DriverLacksLoadFeedback); }
            let max_age = Duration::from_millis(policy.feedback_max_age_ms);
            if !feedback_numeric_valid || ages.len() != dof || !ages.iter().all(|a| a.is_some_and(|age| age <= max_age)) {
                return Some(V::FeedbackTooOld { max_age_ms: policy.feedback_max_age_ms });
            }
        }
        let mut items = [None; MAX_RESTING_SUPPORT_ITEMS];
        let mut len = 0usize;
        let mut overflowed = 0u16;
        for (i, &v) in velocities.iter().enumerate().take(dof) {
            let limit = policy.max_velocity_of(i);
            if !v.is_finite() || v.abs() > limit {
                if len < MAX_RESTING_SUPPORT_ITEMS {
                    items[len] = Some(Item::JointMoving { joint: i as u16, velocity: v, limit });
                    len += 1;
                } else {
                    overflowed = overflowed.saturating_add(1);
                }
            }
        }
        let mut total = 0.0;
        let rest = self.config.resting_positions();
        for (i, &tol) in policy.position_tolerances.iter().enumerate().take(dof) {
            let dev = (positions[i] - rest[i]).abs();
            if !dev.is_finite() || dev > tol {
                if len < MAX_RESTING_SUPPORT_ITEMS {
                    items[len] = Some(Item::JointOffHome { joint: i as u16, deviation: dev, limit: tol });
                    len += 1;
                } else {
                    overflowed = overflowed.saturating_add(1);
                }
            }
            total += dev;
        }
        if len == 0 && total > policy.max_total_deviation {
            items[0] = Some(Item::TotalDeviation { total, limit: policy.max_total_deviation });
            len = 1;
        }
        if self.driver.is_some() {
            for &(i, max_tau) in &policy.support_torque_limits {
                if i >= dof { return Some(V::SupportTorqueIndexOutOfRange { index: i }); }
                let tau = torques.get(i).copied().unwrap_or(f32::NAN);
                if !tau.is_finite() || tau.abs() > max_tau {
                    if len < MAX_RESTING_SUPPORT_ITEMS {
                        items[len] = Some(Item::JointTorque { joint: i as u16, torque: tau, limit: max_tau });
                        len += 1;
                    } else {
                        overflowed = overflowed.saturating_add(1);
                    }
                }
            }
            for &(i, max_load) in &policy.support_load_limits {
                if i >= dof { return Some(V::SupportLoadIndexOutOfRange { index: i }); }
                let load = loads.get(i).copied().unwrap_or(f32::NAN);
                if !load.is_finite() || load.abs() > max_load {
                    if len < MAX_RESTING_SUPPORT_ITEMS {
                        items[len] = Some(Item::JointLoad { joint: i as u16, load, limit: max_load });
                        len += 1;
                    } else {
                        overflowed = overflowed.saturating_add(1);
                    }
                }
            }
        }
        let mut workspace = self.workspace_violation_for_positions_bounded(positions, bufs);
        if workspace.is_some() && len >= MAX_RESTING_SUPPORT_ITEMS {
            workspace = None;
            overflowed = overflowed.saturating_add(1);
        }
        (len > 0 || workspace.is_some()).then_some(V::Failures { items, len: len as u8, overflowed, workspace })
    }

    /// Bounded, allocation-free fault clearing for the RT executor; refusal text is rendered off-thread.
    /// `SafetyManager::clear_check` runs FIRST before any concurrent-E-stop mutation, driver limit
    /// commit, motor reset, or other side effect.
    #[allow(clippy::result_large_err)] // inline refusal keeps RT clear_fault zero-alloc
    pub fn clear_fault_executor_bounded(
        &mut self,
        deadline: Instant,
    ) -> Result<SafetyState, crate::safety::ClearFaultError> {
        self.clear_fault_executor_bounded_for_epoch(deadline, None)
    }

    /// Bounded, allocation-free fault clearing for the RT executor enforcing exact `expected_epoch`
    /// parity with [`Self::clear_fault_for_epoch`].
    #[allow(clippy::result_large_err)] // inline refusal keeps RT clear_fault zero-alloc
    pub fn clear_fault_executor_bounded_for_epoch(
        &mut self,
        deadline: Instant,
        expected_epoch: Option<u64>,
    ) -> Result<SafetyState, crate::safety::ClearFaultError> {
        let mut wall_now = Instant::now;
        match self.clear_fault_executor_inner(&mut wall_now, deadline, expected_epoch, None, None, None) {
            Ok(state) => Ok(state),
            Err(ClearFaultWithAuthError::Clear(e)) => Err(e),
            Err(ClearFaultWithAuthError::Authority(_)) => {
                unreachable!("authority is None on clear_fault_executor_bounded_for_epoch")
            }
        }
    }

    /// Release-bounded, allocation-free fault clearing for Step 2b of an RT release: caps every
    /// TX (0xCC probe and motor fault reset) at `release_deadline - tx_release_publish_margin()`.
    #[allow(clippy::result_large_err)] // inline refusal keeps RT clear_fault zero-alloc
    pub fn clear_fault_executor_bounded_for_release(
        &mut self,
        now: Instant,
        release_deadline: Instant,
        expected_epoch: Option<u64>,
    ) -> Result<SafetyState, crate::safety::ClearFaultError> {
        let mut live_now = || now.max(Instant::now());
        self.clear_fault_executor_bounded_for_release_with_clock(
            now,
            release_deadline,
            expected_epoch,
            &mut live_now,
        )
    }

    /// Like [`Self::clear_fault_executor_bounded_for_release`], threading a live owner clock
    /// (`now_fn`) into under-guard TX deadline translation so wait time or driver-clock sampling
    /// can never extend TX beyond `release_deadline - tx_release_publish_margin()`.
    #[allow(clippy::result_large_err)] // inline refusal keeps RT clear_fault zero-alloc
    pub(crate) fn clear_fault_executor_bounded_for_release_with_clock(
        &mut self,
        now: Instant,
        release_deadline: Instant,
        expected_epoch: Option<u64>,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> Result<SafetyState, crate::safety::ClearFaultError> {
        let tx_deadline = release_deadline
            .checked_sub(self.tx_release_publish_margin())
            .unwrap_or(now);
        match self.clear_fault_executor_inner(now_fn, tx_deadline, expected_epoch, None, None, None) {
            Ok(state) => Ok(state),
            Err(ClearFaultWithAuthError::Clear(e)) => Err(e),
            Err(ClearFaultWithAuthError::Authority(_)) => {
                unreachable!("authority is None on clear_fault_executor_bounded_for_release")
            }
        }
    }

    /// Release-ledger-threaded form of [`Self::clear_fault_executor_bounded_for_release_with_clock`]:
    /// enforces and debits the shared [`crate::rt_core::budget::ReleaseBudget`] across RX drain,
    /// `0xCC` TX probes, and motor fault resets so maintenance never regains TX/RX spent earlier
    /// in the same release.
    #[allow(clippy::result_large_err)]
    pub fn clear_fault_executor_bounded_for_release_with_budget(
        &mut self,
        now: Instant,
        release_budget: &mut crate::rt_core::budget::ReleaseBudget,
        expected_epoch: Option<u64>,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> Result<SafetyState, crate::safety::ClearFaultError> {
        let tx_deadline = release_budget
            .deadline()
            .checked_sub(self.tx_release_publish_margin())
            .unwrap_or(now);
        match self.clear_fault_executor_inner(
            now_fn,
            tx_deadline,
            expected_epoch,
            None,
            Some(release_budget),
            None,
        ) {
            Ok(state) => Ok(state),
            Err(ClearFaultWithAuthError::Clear(e)) => Err(e),
            Err(ClearFaultWithAuthError::Authority(_)) => {
                unreachable!("authority is None on clear_fault_executor_bounded_for_release_with_budget")
            }
        }
    }

    /// Release-bounded, allocation-free fault clearing carrying M3 live owner authority through
    /// driver preflight, hardware reset (`drv.clear_fault_budgeted`), and final `safety.clear_with_check`.
    #[allow(clippy::result_large_err)] // inline refusal keeps RT clear_fault zero-alloc
    pub(crate) fn clear_fault_executor_bounded_for_release_with_authority_clock(
        &mut self,
        now: Instant,
        release_deadline: Instant,
        authority: crate::rt_core::requests::lease::ClearFaultAuthority,
        mit_recovery_started_at: Option<Instant>,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> Result<SafetyState, ClearFaultWithAuthError> {
        let tx_deadline = release_deadline
            .checked_sub(self.tx_release_publish_margin())
            .unwrap_or(now);
        self.clear_fault_executor_inner(
            now_fn,
            tx_deadline,
            Some(authority.expected_epoch),
            Some(authority),
            None,
            mit_recovery_started_at,
        )
    }

    /// Release-ledger-threaded form of [`Self::clear_fault_executor_bounded_for_release_with_authority_clock`].
    #[allow(clippy::result_large_err)]
    pub(crate) fn clear_fault_executor_bounded_for_release_with_authority_budget(
        &mut self,
        now: Instant,
        release_budget: &mut crate::rt_core::budget::ReleaseBudget,
        authority: crate::rt_core::requests::lease::ClearFaultAuthority,
        mit_recovery_started_at: Option<Instant>,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> Result<SafetyState, ClearFaultWithAuthError> {
        let tx_deadline = release_budget
            .deadline()
            .checked_sub(self.tx_release_publish_margin())
            .unwrap_or(now);
        self.clear_fault_executor_inner(
            now_fn,
            tx_deadline,
            Some(authority.expected_epoch),
            Some(authority),
            Some(release_budget),
            mit_recovery_started_at,
        )
    }


    /// Bounded fault clearing for the RT executor rendered into [`SafetyError`] for direct callers.
    pub fn clear_fault_executor(&mut self, deadline: Instant) -> Result<SafetyState, SafetyError> {
        self.clear_fault_executor_for_epoch(deadline, None)
    }

    /// Zero-allocation rest-pose verification on fresh feedback for J1–J6 (`config.arm_joints()`)
    /// before clearing carried-over motor hardware errors on a disarmed arm.
    #[allow(clippy::result_large_err)] // inline refusal keeps RT clear_fault zero-alloc
    pub(in crate::supervisor) fn check_carryover_rest_pose_with_driver(
        config: &crate::model::ArmConfig,
        drv: &dyn crate::driver::ArmDriver,
    ) -> Result<(), crate::safety::ClearRefusal> {
        use crate::safety::{
            ClearRefusal as R, RestingSupportJointItem as Item, RestingSupportViolation as V,
            MAX_RESTING_SUPPORT_ITEMS,
        };
        let Some(policy) = config.resting_pose.as_ref() else {
            return Err(R::MotorErrorNotAtRestPose(V::NoRestingPoseConfigured));
        };
        let tol = config.rest_pose_tolerance_rad;
        if !tol.is_finite() || tol <= 0.0 {
            return Err(R::FreshFeedbackRequiredForRestingSupport);
        }
        let max_age = Duration::from_millis(policy.feedback_max_age_ms);
        let rest = config.resting_positions();
        let arm_joints = config.arm_joints();
        let mut arm_positions = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
        if arm_joints.is_empty()
            || arm_joints.len() > arm_positions.len()
            || rest.len() < config.dof
        {
            return Err(R::FreshFeedbackRequiredForRestingSupport);
        }
        if !drv.fresh_joint_positions_into(
            &arm_joints,
            max_age,
            &mut arm_positions[..arm_joints.len()],
        ) {
            return Err(R::FreshFeedbackRequiredForRestingSupport);
        }
        let mut items = [None; MAX_RESTING_SUPPORT_ITEMS];
        let mut len = 0usize;
        let mut overflowed = 0u16;
        for (&i, &pos) in arm_joints.iter().zip(arm_positions[..arm_joints.len()].iter()) {
            let rest_pos = rest[i];
            let dev = (pos - rest_pos).abs();
            if !pos.is_finite() || !rest_pos.is_finite() || !dev.is_finite() || dev > tol {
                if len < MAX_RESTING_SUPPORT_ITEMS {
                    items[len] = Some(Item::JointOffHome {
                        joint: i as u16,
                        deviation: dev,
                        limit: tol,
                    });
                    len += 1;
                } else {
                    overflowed = overflowed.saturating_add(1);
                }
            }
        }
        if len > 0 {
            return Err(R::MotorErrorNotAtRestPose(V::Failures {
                items,
                len: len as u8,
                overflowed,
                workspace: None,
            }));
        }
        Ok(())
    }

    /// Zero-allocation in-place sync of `self.motor_errors` and cached enable state from a
    /// pre-captured driver status snapshot.
    pub(in crate::supervisor) fn apply_synced_motor_status_bounded(
        &mut self,
        status: &[u8],
        names: &[Option<&'static str>; crate::rt_core::mailbox::MAX_DOF],
        enable_state: EnableState,
    ) {
        self.last_poll_enable_state = enable_state;
        self.capture_driver_cache.enable_state = enable_state;
        let len = status.len().min(names.len());
        if self.rt_executor_owned && self.motor_errors.capacity() < len {
            // Never grow on the RT thread; truncate to existing capacity if pre-reserve missed.
            let cap = self.motor_errors.capacity().min(len);
            self.motor_errors.resize_with(cap, || None);
        } else {
            self.motor_errors.resize_with(len, || None);
        }
        for i in 0..self.motor_errors.len().min(len) {
            self.motor_errors[i] = names[i].map(|name| crate::model::MotorError {
                code: status[i],
                name: std::borrow::Cow::Borrowed(name),
            });
        }
    }

    /// Clear carried-over motor hardware errors on a disarmed arm when no armd fault is latched.
    ///
    /// Shared single-authority path for both legacy and RT executor `clear_fault`:
    /// - requires disarmed supervisor + driver (`Disabled` or `Unknown`, no joint enabled, no torque);
    /// - requires J1–J6 at the resting pose on fresh feedback;
    /// - requires operator authority (when invoked via WS/lease authority);
    /// - transmits `CMD_CLEAR_FAULTS` to affected motors only, then verifies on a fresh post-clear
    ///   status reply that the error bit is cleared and the motor stayed disabled (never enables);
    /// - zero heap allocations on the RT executor path.
    #[allow(clippy::result_large_err)]
    pub(in crate::supervisor) fn clear_carried_over_motor_errors_inner(
        &mut self,
        now_fn: &mut dyn FnMut() -> Instant,
        deadline: Option<Instant>,
        expected_epoch: Option<u64>,
        authority: Option<crate::rt_core::requests::lease::ClearFaultAuthority>,
        release_budget: Option<&mut crate::rt_core::budget::ReleaseBudget>,
    ) -> Result<SafetyState, ClearFaultWithAuthError> {
        use crate::driver::CarryoverMotorClearOutcome;
        use crate::rt_core::requests::lease::recheck_clear_fault_authority_readonly_with_op;
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

        let supervisor_disarmed = !self.is_torqued
            && !self.disable_pending
            && self.gravity_comp.is_none()
            && !self.torque_transition_pending()
            && self.enable_lifecycle == EnableLifecycle::Disarmed
            && !self.safety.state().is_operational()
            && self.active_trajectory.is_none()
            && self.servo_target.is_none()
            && !self.last_action.as_ref().is_some_and(|a| a.is_running())
            && self.macro_run.is_none();
        if !supervisor_disarmed {
            return Err(ClearFaultWithAuthError::Clear(E::Rejected(
                R::MotorErrorClearRequiresDisarmed,
            )));
        }

        if let Some(auth) = authority {
            recheck_clear_fault_authority_readonly_with_op(self, auth, now_fn(), true)
                .map_err(ClearFaultWithAuthError::Authority)?;
        }

        if self.driver.is_none() {
            return Err(ClearFaultWithAuthError::Clear(E::FeedbackUnavailable));
        }

        if let Some(dl) = deadline {
            if now_fn() >= dl {
                return Err(ClearFaultWithAuthError::Clear(E::Rejected(R::BudgetExpired)));
            }
        }

        #[allow(clippy::large_enum_variant)]
        enum CarryoverAttempt {
            Ran {
                outcome: Result<CarryoverMotorClearOutcome, crate::driver::ArmDriverError>,
                persist_name: Option<&'static str>,
                enable_state: EnableState,
                has_status: bool,
                names: [Option<&'static str>; crate::rt_core::mailbox::MAX_DOF],
            },
            Refusal(R),
            Authority(crate::rt_core::requests::lease::ClearFaultAuthorityError),
        }

        let budget_rem = release_budget
            .as_deref()
            .map(|b| (b.rx_remaining(), b.tx_remaining()));
        let mut co_rx_delta = 0u16;
        let mut co_tx_delta = 0u16;
        let mut scratch_status = std::mem::take(&mut self.scratch.status);
        let attempt = self
            .with_driver_and_owner_ref(|owner, drv| {
                let before_budget = drv.tx().release_budget_consumed();
                if let Some((rx_rem, tx_rem)) = budget_rem {
                    drv.tx_mut().sync_release_budget_limits(rx_rem, tx_rem);
                }
                let drv_deadline = if let Some(dl) = deadline {
                    let Ok((_, _, translated)) =
                        crate::supervisor::translate_driver_cutoff_conservative(drv, dl, now_fn)
                    else {
                        return CarryoverAttempt::Refusal(R::BudgetExpired);
                    };
                    drv.set_tx_release_deadline(Some(translated));
                    Some(translated)
                } else {
                    None
                };

                // 1. Drain any queued RX backlog FIRST so newer off-rest or enabled-motor frames
                //    are never missed by the rest-pose or disarm interlocks.
                if let Err(e) = drv.bus_recovery_mut().drain_rx_only_bounded(drv_deadline) {
                    if deadline.is_some() {
                        drv.set_tx_release_deadline(None);
                    }
                    let after_budget = drv.tx().release_budget_consumed();
                    co_rx_delta = after_budget.0.saturating_sub(before_budget.0);
                    co_tx_delta = after_budget.1.saturating_sub(before_budget.1);
                    return CarryoverAttempt::Refusal(R::DriverError(clear_fault_driver_error(&e)));
                }

                // 2. Re-check live authority and pending E-stop after RX drain.
                if let Some(auth) = authority {
                    if let Err(auth_err) =
                        recheck_clear_fault_authority_readonly_with_op(owner, auth, now_fn(), true)
                    {
                        if deadline.is_some() {
                            drv.set_tx_release_deadline(None);
                        }
                        let after_budget = drv.tx().release_budget_consumed();
                        co_rx_delta = after_budget.0.saturating_sub(before_budget.0);
                        co_tx_delta = after_budget.1.saturating_sub(before_budget.1);
                        return CarryoverAttempt::Authority(auth_err);
                    }
                }
                if owner.estop_request_unapplied() {
                    if deadline.is_some() {
                        drv.set_tx_release_deadline(None);
                    }
                    let after_budget = drv.tx().release_budget_consumed();
                    co_rx_delta = after_budget.0.saturating_sub(before_budget.0);
                    co_tx_delta = after_budget.1.saturating_sub(before_budget.1);
                    return CarryoverAttempt::Refusal(R::EStopActive);
                }

                // 3. Require fresh disarm evidence across ALL motors (including J7) so a stale
                //    enabled motor can never satisfy `enable_state == Unknown`.
                let has_all_fresh_feedback = drv.is_feedback_valid();
                let has_status_pre = drv.motor_status_into(&mut scratch_status);
                let any_status_enabled = has_status_pre && scratch_status.contains(&1);
                if !matches!(drv.enable_state(), EnableState::Disabled | EnableState::Unknown)
                    || drv.has_any_joint_enabled()
                    || any_status_enabled
                {
                    if deadline.is_some() {
                        drv.set_tx_release_deadline(None);
                    }
                    let after_budget = drv.tx().release_budget_consumed();
                    co_rx_delta = after_budget.0.saturating_sub(before_budget.0);
                    co_tx_delta = after_budget.1.saturating_sub(before_budget.1);
                    return CarryoverAttempt::Refusal(R::MotorErrorClearRequiresDisarmed);
                }
                if !has_all_fresh_feedback {
                    if deadline.is_some() {
                        drv.set_tx_release_deadline(None);
                    }
                    let after_budget = drv.tx().release_budget_consumed();
                    co_rx_delta = after_budget.0.saturating_sub(before_budget.0);
                    co_tx_delta = after_budget.1.saturating_sub(before_budget.1);
                    return CarryoverAttempt::Refusal(R::FreshFeedbackRequiredForRestingSupport);
                }

                // 4. Evaluate J1–J6 rest-pose gate strictly AFTER the RX drain.
                if let Err(refusal) = Self::check_carryover_rest_pose_with_driver(&owner.config, &*drv)
                {
                    if deadline.is_some() {
                        drv.set_tx_release_deadline(None);
                    }
                    let after_budget = drv.tx().release_budget_consumed();
                    co_rx_delta = after_budget.0.saturating_sub(before_budget.0);
                    co_tx_delta = after_budget.1.saturating_sub(before_budget.1);
                    return CarryoverAttempt::Refusal(refusal);
                }

                // 5. Submit targeted `CMD_CLEAR_FAULTS` with a live pre-TX authority & E-stop gate
                //    checked immediately before every CAN frame write.
                let mut revoked_auth: Option<crate::rt_core::requests::lease::ClearFaultAuthorityError> =
                    None;
                let mut pre_tx_gate = || -> Result<(), &'static str> {
                    if let Some(auth) = authority {
                        if let Err(auth_err) = recheck_clear_fault_authority_readonly_with_op(
                            owner,
                            auth,
                            now_fn(),
                            true,
                        ) {
                            revoked_auth = Some(auth_err);
                            return Err("AUTHORITY_REVOKED");
                        }
                    }
                    if owner.estop_request_unapplied() {
                        return Err("ESTOP_PENDING");
                    }
                    Ok(())
                };
                let outcome = drv.bus_recovery_mut().clear_carried_over_motor_errors(drv_deadline, &mut pre_tx_gate);
                if deadline.is_some() {
                    drv.set_tx_release_deadline(None);
                }
                let after_budget = drv.tx().release_budget_consumed();
                co_rx_delta = after_budget.0.saturating_sub(before_budget.0);
                co_tx_delta = after_budget.1.saturating_sub(before_budget.1);
                if let Ok(CarryoverMotorClearOutcome::Revoked(code)) = outcome {
                    if let Some(auth_err) = revoked_auth {
                        return CarryoverAttempt::Authority(auth_err);
                    }
                    if code == "ESTOP_PENDING" {
                        return CarryoverAttempt::Refusal(R::EStopActive);
                    }
                    return CarryoverAttempt::Refusal(R::BudgetExpired);
                }
                if owner.estop_request_unapplied() {
                    return CarryoverAttempt::Refusal(R::EStopActive);
                }
                let persist_name = match outcome {
                    Ok(CarryoverMotorClearOutcome::ErrorPersists { code, .. }) => {
                        drv.decode_motor_error_static(code)
                    }
                    _ => None,
                };
                let has_status = drv.motor_status_into(&mut scratch_status);
                let mut names = [None; crate::rt_core::mailbox::MAX_DOF];
                if has_status {
                    for (slot, &code) in names.iter_mut().zip(scratch_status.iter()) {
                        *slot = drv.decode_motor_error_static(code);
                    }
                }
                CarryoverAttempt::Ran {
                    outcome,
                    persist_name,
                    enable_state: drv.enable_state(),
                    has_status,
                    names,
                }
            })
            .ok_or(ClearFaultWithAuthError::Clear(E::FeedbackUnavailable));
        let attempt = match attempt {
            Ok(a) => a,
            Err(e) => {
                self.scratch.status = scratch_status;
                return Err(e);
            }
        };
        if let Some(b) = release_budget {
            if co_rx_delta > 0 {
                b.charge_rx(co_rx_delta, false);
            }
            if co_tx_delta > 0 {
                b.charge_tx(co_tx_delta);
            }
        }

        let result = match attempt {
            CarryoverAttempt::Ran {
                outcome,
                persist_name,
                enable_state,
                has_status,
                names,
            } => {
                if has_status {
                    self.apply_synced_motor_status_bounded(&scratch_status, &names, enable_state);
                }
                match outcome {
                    Ok(CarryoverMotorClearOutcome::NoErrors | CarryoverMotorClearOutcome::Cleared) => {
                        if self.estop_request_unapplied() {
                            Err(apply_estop_refusal(self))
                        } else {
                            Ok(self.safety.state().clone())
                        }
                    }
                    Ok(CarryoverMotorClearOutcome::ErrorPersists { joint, code }) => {
                        Err(ClearFaultWithAuthError::Clear(E::Rejected(
                            R::MotorErrorPersists {
                                joint: joint.min(usize::from(u8::MAX)) as u8,
                                code,
                                name: persist_name,
                            },
                        )))
                    }
                    Ok(CarryoverMotorClearOutcome::MotorEnabled { joint }) => {
                        Err(ClearFaultWithAuthError::Clear(E::Rejected(
                            R::MotorErrorClearMotorEnabled {
                                joint: joint.min(usize::from(u8::MAX)) as u8,
                            },
                        )))
                    }
                    Ok(CarryoverMotorClearOutcome::Unconfirmed { joint }) => {
                        Err(ClearFaultWithAuthError::Clear(E::Rejected(
                            R::MotorErrorClearUnconfirmed {
                                joint: joint.min(usize::from(u8::MAX)) as u8,
                            },
                        )))
                    }
                    Ok(CarryoverMotorClearOutcome::Revoked(_)) => {
                        Err(ClearFaultWithAuthError::Clear(E::Rejected(R::BudgetExpired)))
                    }
                    Err(crate::driver::ArmDriverError::EstopInhibited) => {
                        Err(apply_estop_refusal(self))
                    }
                    Err(e) => Err(ClearFaultWithAuthError::Clear(E::Rejected(
                        R::DriverError(clear_fault_driver_error(&e)),
                    ))),
                }
            }
            CarryoverAttempt::Refusal(R::EStopActive) => Err(apply_estop_refusal(self)),
            CarryoverAttempt::Refusal(refusal) => {
                Err(ClearFaultWithAuthError::Clear(E::from(refusal)))
            }
            CarryoverAttempt::Authority(e) => Err(ClearFaultWithAuthError::Authority(e)),
        };
        self.scratch.status = scratch_status;
        result
    }

    /// Bounded fault clearing for the RT executor with exact epoch check rendered into [`SafetyError`].
    pub fn clear_fault_executor_for_epoch(
        &mut self,
        deadline: Instant,
        expected_epoch: Option<u64>,
    ) -> Result<SafetyState, SafetyError> {
        self.clear_fault_executor_bounded_for_epoch(deadline, expected_epoch)
            .map_err(SafetyError::from)
    }
}
