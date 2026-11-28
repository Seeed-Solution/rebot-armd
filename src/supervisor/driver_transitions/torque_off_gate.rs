//! Torque-off admission gates and RT/broker torque transition initiation.

#[allow(unused_imports)]
use super::*;
#[allow(unused_imports)]
use super::super::*;

// scaffold-extra-0
impl SupervisorInner {
    /// Torque off at the resting pose. `stillness_verified` is set by the unload settle,
    /// which judged stationarity over its hold window; otherwise the current velocity
    /// sample must be within the resting-pose limit.
    pub(in crate::supervisor) fn torque_off(&mut self, stillness_verified: bool) -> Result<SafetyState, SafetyError> {
        self.torque_off_in_place(stillness_verified)?;
        Ok(self.safety.state().clone())
    }

    /// True when E-stop, a Fault state, any latched fault kind bit, or a pending E-stop
    /// keeps the arm stopped: torque-off must apply Rule A′ and must never clear or mutate
    /// the fault record or epoch.
    #[inline]
    pub(in crate::supervisor) fn torque_off_stopped(&self) -> bool {
        self.safety.state().is_stopped()
            || self.safety.has_latched_fault()
            || self.estop_pending()
    }

    pub(in crate::supervisor) fn torque_off_in_place(&mut self, stillness_verified: bool) -> Result<(), SafetyError> {
        if let Err(err) = self.torque_off_check(stillness_verified) {
            #[cfg(not(target_arch = "wasm32"))]
            {
                let live_op_id = self.torque_shadow.alloc_live_op_id();
                let live_refusal = match &err {
                    SafetyError::TorqueOffWouldDrop { .. } => {
                        crate::torque::shadow::LiveRefusalKind::TorqueOffWouldDrop
                    }
                    SafetyError::ArmNotAtRestingPose { .. } => {
                        crate::torque::shadow::LiveRefusalKind::ArmNotAtRestingPose
                    }
                    SafetyError::EStopActive => {
                        crate::torque::shadow::LiveRefusalKind::EStopActive
                    }
                    SafetyError::FaultActive(_) => {
                        crate::torque::shadow::LiveRefusalKind::FaultLatched
                    }
                    _ => crate::torque::shadow::LiveRefusalKind::Other,
                };
                self.torque_shadow
                    .enqueue(crate::torque::shadow::ShadowInputKind::DisableRequest {
                        live_op_id,
                        explicit_release_authorized: false,
                        live_refusal: Some(live_refusal),
                        live_intent: self.shadow_live_intent(),
                        live_is_torqued: self.is_torqued,
                        live_stopped: self.safety.state().is_stopped(),
                    });
            }
            return Err(err);
        }
        let started_at = self.driver_clock_now();
        let disable_kind = crate::driver::DisableKind::DisableRequested { started_at };
        self.stop_motion();
        self.commanded_velocities.fill(0.0);
        if let Err(error) = self.request_disable(disable_kind) {
            let refused = matches!(error, SafetyError::TorqueOffWouldDrop { .. });
            if !refused && !self.torque_off_stopped() {
                self.safety.latch(
                    crate::safety::FaultKind::Other,
                    crate::safety::FaultDetail::formatted(FaultCode::DriverWriteFailed, |buf| {
                        let _ = core::fmt::write(buf, format_args!("{error}"));
                    }),
                );
            }
            return Err(error);
        }
        if !self.disable_pending {
            self.torque_off_finish_in_place()?;
        }
        Ok(())
    }

    /// Record one contended `driver.try_lock()` skip on an RT disable dispatch or poll path (M4).
    /// Escalates at `STOPPED_HOLD_CONTENTION_ESCALATE_SKIPS` (25 consecutive skips, 0.5 s at 50 Hz)
    /// with a counted diagnostic escalation and a non-recoverable `HardwareFault` (zero allocation).
    pub(crate) fn record_disable_lock_contention(&mut self) {
        crate::diagnostics::tick::record_disable_lock_contended();
        self.disable_lock_contended_skips = self.disable_lock_contended_skips.saturating_add(1);
        if self.disable_lock_contended_skips >= STOPPED_HOLD_CONTENTION_ESCALATE_SKIPS
            && !self.disable_lock_contended_escalated
        {
            self.disable_lock_contended_escalated = true;
            crate::diagnostics::tick::record_disable_contention_escalation();
            let skips = self.disable_lock_contended_skips;
            let detail = crate::safety::FaultDetail::formatted(FaultCode::HardwareFault, |buf| {
                let _ = core::fmt::write(
                    buf,
                    format_args!(
                        "RT torque-disable path skipped {} consecutive ticks due to driver mutex contention",
                        skips
                    ),
                );
            });
            self.safety.latch(crate::safety::FaultKind::Other, detail);
        }
    }

    #[inline]
    pub(crate) fn reset_disable_lock_contention(&mut self) {
        self.disable_lock_contended_skips = 0;
        self.disable_lock_contended_escalated = false;
    }

    /// True when the driver can keep a stopped disable pending inside its 1 s lifetime while an
    /// exhausted RX drain, incomplete K=3 rest-pose window, or stale/missing evidence on a drained
    /// bus runs the probe loop (L1/M3). Non-blocking variant for `armd-control` (M4).
    pub(in crate::supervisor) fn can_defer_stopped_disable_to_driver_nonblocking(&self) -> Result<bool, ()> {
        let Some(d) = self.driver.as_ref() else {
            return Ok(false);
        };
        match d.try_with_ref(|drv| {
            drv.supports_budgeted_disable_await()
                && matches!(
                    drv.rest_pose_gate_status(),
                    crate::driver::RestPoseGateStatus::PendingDrainOrWindow
                        | crate::driver::RestPoseGateStatus::StaleOrMissing
                )
        }) {
            Ok(can) => Ok(can),
            Err(crate::supervisor_lock::DriverTryError::WouldBlock) => Err(()),
            Err(crate::supervisor_lock::DriverTryError::Poisoned) => Ok(false),
        }
    }

    /// True when the driver can keep a stopped disable pending inside its 1 s lifetime while an
    /// exhausted RX drain, incomplete K=3 rest-pose window, or stale/missing evidence on a drained
    /// bus runs the probe loop (L1/M3). Zero heap allocations.
    pub(in crate::supervisor) fn can_defer_stopped_disable_to_driver(&self) -> bool {
        if self.rt_executor_owned {
            return self.can_defer_stopped_disable_to_driver_nonblocking().unwrap_or(false);
        }
        self.driver.as_ref().is_some_and(|d| {
            d.with_ref(|drv| {
                drv.supports_budgeted_disable_await()
                    && matches!(
                        drv.rest_pose_gate_status(),
                        crate::driver::RestPoseGateStatus::PendingDrainOrWindow
                            | crate::driver::RestPoseGateStatus::StaleOrMissing
                    )
            })
            .unwrap_or(false)
        })
    }

    /// Zero-allocation RT torque-off precheck (static protocol error codes): enforces the
    /// unified rest-pose gate in EVERY safety state (operational and stopped alike), plus the
    /// operational stillness/support-torque/workspace check when not stopped.
    pub(in crate::supervisor) fn torque_off_check_rt(&mut self, stillness_verified: bool) -> Result<(), &'static str> {
        if self.calibration_busy && self.motor_zero_busy {
            return Err("MOTION_BUSY");
        }
        if self.torque_transition.is_some() {
            return Err("TRANSITION_BUSY");
        }
        if self.gravity_comp.is_some() {
            return Err("MOTION_BUSY");
        }
        if !self.torque_off_stopped() && self.has_resting_pose_violation_rt(stillness_verified) {
            return Err("ARM_NOT_AT_RESTING_POSE");
        }
        self.eval_stopped_torque_off()
            .map_err(|_| "TORQUE_OFF_WOULD_DROP")
    }

    /// Zero-allocation, non-blocking rest-pose gate for ALL RT budgeted torque-off transitions
    /// (operational and stopped alike): uses `driver.try_lock()` with counted escalation on
    /// contention, checks operational stillness/support-torque/workspace when not stopped,
    /// defers incomplete drains / K=3 windows / stale-or-missing evidence to `AwaitRestGate`,
    /// and refuses off-rest poses with zero heap allocations.
    pub(in crate::supervisor) fn torque_off_check_rt_transition(&mut self) -> Result<(), &'static str> {
        if self.calibration_busy && self.motor_zero_busy {
            return Err("MOTION_BUSY");
        }
        if self.torque_transition.is_some() {
            return Err("TRANSITION_BUSY");
        }
        if self.gravity_comp.is_some() {
            return Err("MOTION_BUSY");
        }
        if !self.torque_off_stopped() {
            match self.check_resting_pose_rt(false) {
                super::RtRestingCheck::AtRest => {}
                super::RtRestingCheck::OffRest => return Err("ARM_NOT_AT_RESTING_POSE"),
                super::RtRestingCheck::LockContended => {
                    self.record_disable_lock_contention();
                    return Err("DRIVER_BUSY");
                }
            }
        }
        match self.may_disable_torque_while_stopped_nonblocking() {
            Err(()) => {
                self.record_disable_lock_contention();
                Err("DRIVER_BUSY")
            }
            Ok(Ok(())) => {
                self.last_stopped_torque_off_refusal = None;
                Ok(())
            }
            Ok(Err(crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback)) => {
                match self.can_defer_stopped_disable_to_driver_nonblocking() {
                    Err(()) => {
                        self.record_disable_lock_contention();
                        Err("DRIVER_BUSY")
                    }
                    Ok(true) => {
                        self.last_stopped_torque_off_refusal = None;
                        Ok(())
                    }
                    Ok(false) => {
                        self.last_stopped_torque_off_refusal =
                            Some(crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback);
                        Err("TORQUE_OFF_WOULD_DROP")
                    }
                }
            }
            Ok(Err(r)) => {
                self.last_stopped_torque_off_refusal = Some(r);
                Err("TORQUE_OFF_WOULD_DROP")
            }
        }
    }

    /// Unified torque-off refusals for operational and stopped states alike: when operational,
    /// checks the resting-pose stillness/support-torque/workspace policy first, then ALWAYS
    /// enforces the authoritative K=3 rest-pose gate (`eval_stopped_torque_off`).
    pub(in crate::supervisor) fn torque_off_check(&mut self, stillness_verified: bool) -> Result<(), SafetyError> {
        if !self.torque_off_stopped() {
            let violation = if stillness_verified {
                self.resting_pose_static_violation()
            } else {
                self.resting_pose_violation()
            };
            if let Some(reason) = violation {
                tracing::warn!("Torque off refused, not at resting pose: {reason}");
                return Err(SafetyError::ArmNotAtRestingPose {
                    current: self.measured_positions.clone(),
                    expected: self.config.resting_positions().to_vec(),
                    reason: Some(reason),
                });
            }
        }
        match self.eval_stopped_torque_off() {
            Ok(()) => Ok(()),
            Err(crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback)
                if self.can_defer_stopped_disable_to_driver() =>
            {
                self.last_stopped_torque_off_refusal = None;
                Ok(())
            }
            Err(r) => Err(r.into_safety_error(&self.config.joint_names)),
        }
    }

    /// Torque off after the accepted disable without cloning `SafetyState` (zero-alloc on RT).
    pub(in crate::supervisor) fn torque_off_finish_in_place(&mut self) -> Result<(), SafetyError> {
        if !self.torque_off_stopped() {
            let _ = self.safety.set_torque(false)?;
        }
        self.reset_gripper_stall();
        self.gripper_hold_vel_ceiling = None;
        if self.driver.is_none() { self.measured_velocities.fill(0.0); }
        Ok(())
    }

    /// Zero-allocation RT torque-off finish after accepted disable (preserves stopped Fault/EStop).
    pub(in crate::supervisor) fn torque_off_finish_rt(&mut self) -> Result<(), &'static str> {
        if !self.torque_off_stopped() {
            self.safety.set_torque_rt(false)?;
        }
        self.reset_gripper_stall();
        self.gripper_hold_vel_ceiling = None;
        if self.driver.is_none() {
            self.measured_velocities.fill(0.0);
        }
        Ok(())
    }

    /// Re-checks live authority for an in-flight `TorqueTransition::Enable` BEFORE any retry TX.
    pub(crate) fn enable_authority_revoked(
        &self,
        auth: EnableAuthority,
        now: Instant,
    ) -> Option<&'static str> {
        if self.shutdown_committed
            || self.estop_pending()
            || matches!(self.safety.state(), SafetyState::EStop)
        {
            return Some("ESTOP_ACTIVE");
        }
        if self.safety.state().is_stopped() {
            return Some("FAULT_ACTIVE");
        }
        if auth.admitted_epoch != 0 && self.safety.epoch_id() != auth.admitted_epoch {
            return Some("EPOCH_MISMATCH");
        }
        if auth.owner_conn_id != 0
            && self.check_client_connection_live(auth.owner_conn_id).is_err()
        {
            return Some("DISCONNECTED");
        }
        if self.pending_disconnect_hold.load(Ordering::SeqCst) > 0 {
            return Some("LEASE_LOST");
        }
        let fenced_gen = self.effective_fenced_lease_generation();
        if auth.lease_generation != 0 {
            if auth.lease_generation <= fenced_gen {
                return Some("LEASE_LOST");
            }
            let owner_matches = auth.owner_conn_id == 0
                || self.active_lease_conn_id == auth.owner_conn_id;
            let lease_matches = owner_matches
                && self.active_lease.as_ref().is_some_and(|a| {
                    a.generation == auth.lease_generation
                        && a.generation > fenced_gen
                        && (self.active_lease_conn_id == 0
                            || !self.is_connection_closed(self.active_lease_conn_id))
                        && now.saturating_duration_since(a.acquired_at) < self.lease_timeout
                });
            if !lease_matches {
                return Some("LEASE_LOST");
            }
        }
        None
    }

    /// Aborts any in-flight `TorqueTransition::Enable` owned by `displaced_gen` (called by
    /// `fence_displaced_lease_generation` on release, expiry, preemption, or disconnect),
    /// transitioning through [`Self::enter_failed_hold`] (no rollback disable frames at any pose).
    pub(crate) fn abort_enable_transition_for_fenced_lease_impl(&mut self, displaced_gen: u64) {
        let matches = matches!(
            self.torque_transition,
            Some(TorqueTransition::Enable(reducer))
                if displaced_gen != 0
                    && reducer
                        .authority
                        .is_some_and(|a| a.lease_generation == displaced_gen)
        );
        if !matches {
            return;
        }
        let Some(TorqueTransition::Enable(reducer)) = self.torque_transition.take() else {
            return;
        };
        let closed = reducer.authority.is_some_and(|a| {
            a.owner_conn_id != 0 && self.is_connection_closed(a.owner_conn_id)
        });
        let code = if closed { "DISCONNECTED" } else { "LEASE_LOST" };
        let _ = self.enter_failed_hold(reducer.was_torqued, |buf| {
            buf.push_str(if closed {
                "interrupted by disconnect"
            } else {
                "interrupted by lease loss"
            });
        });
        match reducer.origin {
            EnableOrigin::BrokerTransition => self.core.begin_transition(Err(code)),
            EnableOrigin::ParkRearm { action_id, .. } => self.finish_park_rearm_failed(action_id, code),
            EnableOrigin::LegacyWs { token, slot, .. } => {
                self.settle_legacy_ws_failed(token, slot, LegacyTorqueFailure::LeaseLost);
            }
            EnableOrigin::DetachedCancel => {}
        }
    }

    /// A transition is in flight ([`TorqueTransition`]): ordinary output and blocking torque
    /// changes wait for it.
    pub fn torque_transition_pending(&self) -> bool { self.torque_transition.is_some() }

    pub(crate) fn park_disable_in_flight(&self) -> bool {
        matches!(self.torque_transition, Some(TorqueTransition::Disable { origin: DisableOrigin::Park { .. }, .. }))
    }

    pub(crate) fn park_disable_in_flight_for(&self, id: u64) -> bool {
        matches!(self.torque_transition, Some(TorqueTransition::Disable { origin: DisableOrigin::Park { action_id: Some(aid), .. }, .. }) if aid == id)
    }

    pub(crate) fn is_watchdog_park_disable_in_flight(&self) -> bool {
        matches!(self.torque_transition, Some(TorqueTransition::Disable { origin: DisableOrigin::Park { watchdog: true, .. }, .. }))
    }

    pub(crate) fn has_non_broker_torque_transition(&self) -> bool {
        matches!(
            self.torque_transition,
            Some(
                TorqueTransition::Enable(EnableReducer {
                    origin: EnableOrigin::ParkRearm { .. }
                        | EnableOrigin::LegacyWs { .. }
                        | EnableOrigin::DetachedCancel,
                    ..
                })
                    | TorqueTransition::Disable {
                        origin: DisableOrigin::Park { .. }
                            | DisableOrigin::LegacyWs { .. }
                            | DisableOrigin::Detached { .. },
                        ..
                    }
            )
        )
    }

    pub(crate) fn has_detached_torque_transition(&self) -> bool {
        matches!(
            self.torque_transition,
            Some(
                TorqueTransition::Enable(EnableReducer {
                    origin: EnableOrigin::DetachedCancel,
                    ..
                })
                    | TorqueTransition::Disable {
                        origin: DisableOrigin::Detached { .. },
                        ..
                    }
            )
        )
    }

    pub(crate) fn has_legacy_torque_transition_token(&self, token: u64) -> bool {
        matches!(
            self.torque_transition,
            Some(
                TorqueTransition::Enable(EnableReducer {
                    origin: EnableOrigin::LegacyWs { token: t, .. },
                    ..
                })
                    | TorqueTransition::Disable {
                        origin: DisableOrigin::LegacyWs { token: t, .. },
                        ..
                    }
            ) if t == token
        )
    }

    #[allow(dead_code)]
    pub(in crate::supervisor) fn driver_any_enabled(&self) -> bool {
        self.driver.as_ref().is_some_and(|d| {
            d.with_ref(|drv| {
                drv.enable_state() != EnableState::Disabled
                    || drv.has_any_joint_enabled()
                    || drv.has_known_partial_enable()
            })
            .unwrap_or(false)
        })
    }

    /// Starts a torque-enable transition without waiting for its confirmation (the RT's
    /// `commit_transition`). Requires a valid [`AdmissionStamp`] checked first under the owner lock
    /// and drives the single authoritative budgeted enable reducer (`Enable -> Validate -> Armed | FailedHold`)
    /// carrying live [`EnableAuthority`] checks (lease generation, owner conn, epoch) into `FailedHold`.
    pub(crate) fn begin_enable_torque_transition(
        admitted: Admitted<'_>,
    ) -> Result<Result<crate::rt_core::ops::TorqueProgress, &'static str>, &'static str> {
        let req_id = admitted.stamp().request_id;
        Self::begin_enable_torque_transition_with_seq(admitted, req_id)
    }

    pub(crate) fn begin_enable_torque_transition_with_seq(
        admitted: Admitted<'_>,
        client_seq: u64,
    ) -> Result<Result<crate::rt_core::ops::TorqueProgress, &'static str>, &'static str> {
        let (this, proof) = admitted
            .consume()
            .map_err(|e| torque_admission_error_code(&e))?;
        let s = proof.stamp();
        let authority = EnableAuthority {
            lease_generation: s.lease_gen,
            owner_conn_id: s.connection_id,
            admitted_epoch: s.estop_epoch,
            admission_request_id: client_seq,
            stop_gen: s.stop_gen,
        };
        this.begin_enable_torque_transition_inner(
            EnableOrigin::BrokerTransition,
            authority,
            false,
            false,
        )
    }

    /// Driverless immediate torque-enable on RT (zero-alloc, consumes A1's [`Admitted`]).
    pub(crate) fn enable_torque_immediate_rt(
        admitted: Admitted<'_>,
        now: Instant,
    ) -> Result<(), &'static str> {
        let (this, proof) = admitted
            .consume()
            .map_err(|e| torque_admission_error_code(&e))?;
        if this.driver.is_some() {
            return Err("UNSUPPORTED");
        }
        let s = proof.stamp();
        let authority = EnableAuthority {
            lease_generation: s.lease_gen,
            owner_conn_id: s.connection_id,
            admitted_epoch: s.estop_epoch,
            admission_request_id: s.request_id,
            stop_gen: s.stop_gen,
        };
        let (was_torqued, prev_gripper_cmd, mit_qualified) = this.enable_prepare_rt(Some(authority))?;
        let _ = now;
        match this.start_enable_reducer(
            EnableOrigin::BrokerTransition,
            was_torqued,
            prev_gripper_cmd,
            Some(authority),
            true,
            mit_qualified,
        ) {
            EnableStepOutcome::Armed(_) => Ok(()),
            EnableStepOutcome::Failed { rt_code, .. } => Err(rt_code),
            EnableStepOutcome::Pending { .. } => Err("INTERNAL"),
        }
    }

    /// Driverless immediate torque-disable on RT (zero-alloc, exempt from `Admitted`).
    pub(crate) fn disable_torque_immediate_rt(&mut self, now: Instant) -> Result<(), &'static str> {
        if self.driver.is_some() {
            return Err("UNSUPPORTED");
        }
        self.torque_off_check_rt(false)?;
        self.stop_motion();
        self.commanded_velocities.fill(0.0);
        self.disable_requested_at(now);
        #[cfg(not(target_arch = "wasm32"))]
        {
            let live_op_id = self.torque_shadow.alloc_live_op_id();
            self.torque_shadow
                .enqueue(crate::torque::shadow::ShadowInputKind::DisableRequest {
                    live_op_id,
                    explicit_release_authorized: false,
                    live_refusal: None,
                    live_intent: self.shadow_live_intent(),
                    live_is_torqued: self.is_torqued,
                    live_stopped: self.safety.state().is_stopped(),
                });
        }
        self.disable_accepted();
        self.torque_off_finish_rt()
    }

    pub(in crate::supervisor) fn begin_enable_torque_transition_inner(
        &mut self,
        origin: EnableOrigin,
        authority: EnableAuthority,
        allow_sync_mit: bool,
        allow_legacy_fallback: bool,
    ) -> Result<Result<crate::rt_core::ops::TorqueProgress, &'static str>, &'static str> {
        use crate::rt_core::ops::TorqueProgress as Rt;
        let use_rt_path = self.rt_executor_owned || matches!(origin, EnableOrigin::BrokerTransition);
        let prep_res: Result<(bool, Option<f32>, bool), &'static str> = if use_rt_path {
            self.enable_prepare_rt(Some(authority))
        } else {
            (|| {
                if self.torque_transition.is_some() {
                    return Err("TRANSITION_BUSY");
                }
                self.gravity_comp_servo_busy().map_err(|_| "MOTION_BUSY")?;
                let (w, g) = self.enable_prepare_inner(false, allow_sync_mit, false)
                    .map_err(|e| torque_error_code(&e))?;
                let mit_q = self.config.joint_tracking.as_ref().is_some_and(|jt| jt.mit_requested());
                Ok((w, g, mit_q))
            })()
        };
        let (was_torqued, prev_gripper_cmd, mit_qualified) = match prep_res {
            Ok(v) => v,
            Err(code) => {
                #[cfg(not(target_arch = "wasm32"))]
                {
                    let live_op_id = self.torque_shadow.alloc_live_op_id();
                    let live_refusal = match code {
                        "ESTOP_ACTIVE" => crate::torque::shadow::LiveRefusalKind::EStopActive,
                        "FAULT_ACTIVE" => crate::torque::shadow::LiveRefusalKind::FaultLatched,
                        "DISABLE_PENDING" => crate::torque::shadow::LiveRefusalKind::DisableInProgress,
                        "MOTION_BUSY" | "TRANSITION_BUSY" | "HANDOVER_SETTLING" => {
                            crate::torque::shadow::LiveRefusalKind::BusyOrTransition
                        }
                        "UNSUPPORTED" => crate::torque::shadow::LiveRefusalKind::Unsupported,
                        _ => crate::torque::shadow::LiveRefusalKind::EnableFailed,
                    };
                    self.torque_shadow
                        .enqueue(crate::torque::shadow::ShadowInputKind::EnableStart {
                            live_op_id,
                            live_refusal: Some(live_refusal),
                            live_intent: self.shadow_live_intent(),
                            live_is_torqued: self.is_torqued,
                            live_stopped: self.safety.state().is_stopped(),
                        });
                }
                return Err(code);
            }
        };
        let budgeted = !allow_legacy_fallback
            || self.rt_executor_owned
            || self.driver.is_none()
            || self.driver_supports_rt_executor_nonblocking();
        Ok(
            match self.start_enable_reducer(
                origin,
                was_torqued,
                prev_gripper_cmd,
                Some(authority),
                budgeted,
                mit_qualified,
            ) {
                EnableStepOutcome::Armed(_) => Ok(Rt::Done),
                EnableStepOutcome::Pending { wake } => Ok(Rt::Pending { wake }),
                EnableStepOutcome::Failed { rt_code, .. } => Err(rt_code),
            },
        )
    }

    /// Starts a bounded, confirmed `TorqueTransition::Enable` inside park `action_id` when
    /// parking from `holding_after_clear`. Requires the already-consumed [`AdmittedProof`].
    /// Zero heap allocations on RT failure paths.
    pub(crate) fn begin_park_rearm_transition(
        &mut self,
        proof: &crate::supervisor::AdmittedProof,
        action_id: u64,
        stop_gen: u64,
    ) -> Result<ParkRearmStart, crate::rt_core::requests::motion::MotionInstallError> {
        use crate::rt_core::requests::motion::MotionInstallError;
        if self.urgent_stop_unapplied() || self.current_stop_gen() != stop_gen {
            return Err(MotionInstallError::Stopped);
        }
        let s = proof.stamp();
        let authority = EnableAuthority {
            lease_generation: s.lease_gen,
            owner_conn_id: s.connection_id,
            admitted_epoch: s.estop_epoch,
            admission_request_id: s.request_id,
            stop_gen: s.stop_gen,
        };
        let map_code = |c: &'static str| match c {
            "ESTOP_ACTIVE" => MotionInstallError::EStopActive,
            "FAULT_ACTIVE" => MotionInstallError::FaultActive,
            "HANDOVER_SETTLING" => MotionInstallError::HandoverSettling,
            "FEEDBACK_UNAVAILABLE" | "FEEDBACK_STALE" => MotionInstallError::FeedbackStale,
            "CONFIG_BUSY" => MotionInstallError::ConfigBusy,
            "MOTION_BUSY" | "TRANSITION_BUSY" | "DISABLE_PENDING" => MotionInstallError::MotionBusy,
            _ => MotionInstallError::EnableFailed,
        };
        match self
            .begin_enable_torque_transition_inner(
                EnableOrigin::ParkRearm { action_id, stop_gen },
                authority,
                !self.rt_executor_owned,
                true,
            )
            .map_err(map_code)?
        {
            Ok(crate::rt_core::ops::TorqueProgress::Done) => Ok(ParkRearmStart::ConfirmedNow),
            Ok(crate::rt_core::ops::TorqueProgress::Pending { .. }) => Ok(ParkRearmStart::Pending),
            Err(_) => Err(MotionInstallError::EnableFailed),
        }
    }

    /// Aborts a pending `ParkRearm` enable transition safely (non-blocking `try_lock`, preserving
    /// holding torque and retaining `DetachedCancel` on driver mutex contention).
    pub(crate) fn abort_park_rearm_transition(&mut self) {
        let Some(TorqueTransition::Enable(EnableReducer {
            origin: EnableOrigin::ParkRearm { .. },
            was_torqued,
            ..
        })) = self.torque_transition
        else {
            return;
        };
        self.torque_transition = None;
        let (drv_any_enabled, _) = self.abort_enable_driver_nonblocking(was_torqued, true);
        self.is_torqued = was_torqued || drv_any_enabled;
        self.pending_rearm = true;
        #[cfg(not(target_arch = "wasm32"))]
        if !was_torqued && !drv_any_enabled {
            self.torque_shadow.enqueue(
                crate::torque::shadow::ShadowInputKind::EnableCancelledPreTx {
                    live_op_id: self.shadow_enable_op_id,
                    live_intent: self.shadow_live_intent(),
                    live_is_torqued: self.is_torqued,
                    live_stopped: self.safety.state().is_stopped(),
                },
            );
        } else {
            self.torque_shadow.enqueue(
                crate::torque::shadow::ShadowInputKind::HandshakeResult {
                    live_op_id: self.shadow_enable_op_id,
                    confirmed_mask: crate::torque::ChannelMask::EMPTY,
                    outcome: crate::torque::HandshakeOutcome::Failed,
                    live_intent: self.shadow_live_intent(),
                    live_is_torqued: self.is_torqued,
                    live_stopped: self.safety.state().is_stopped(),
                },
            );
        }
    }

    /// Starts a torque-disable transition without waiting for its confirmation.
    ///
    /// Exempt from `Admitted` (Invariant 1c): torque disable is a stop-direction action that
    /// removes actuator output and must never be blocked by a Stop or admission check (still
    /// governed by resting-pose and torque-off safety checks, zero-alloc on RT).
    pub(crate) fn begin_disable_torque_transition(
        &mut self,
    ) -> Result<Result<crate::rt_core::ops::TorqueProgress, &'static str>, &'static str> {
        use crate::rt_core::ops::TorqueProgress as Rt;
        if let Err(code) = self.torque_off_check_rt_transition() {
            #[cfg(not(target_arch = "wasm32"))]
            {
                let live_op_id = self.torque_shadow.alloc_live_op_id();
                let live_refusal = match code {
                    "TORQUE_OFF_WOULD_DROP" | "ARM_NOT_AT_RESTING_POSE" => {
                        crate::torque::shadow::LiveRefusalKind::TorqueOffWouldDrop
                    }
                    "DRIVER_BUSY" => crate::torque::shadow::LiveRefusalKind::DriverBusy,
                    "MOTION_BUSY" | "TRANSITION_BUSY" => {
                        crate::torque::shadow::LiveRefusalKind::BusyOrTransition
                    }
                    _ => crate::torque::shadow::LiveRefusalKind::Other,
                };
                self.torque_shadow
                    .enqueue(crate::torque::shadow::ShadowInputKind::DisableRequest {
                        live_op_id,
                        explicit_release_authorized: false,
                        live_refusal: Some(live_refusal),
                        live_intent: self.shadow_live_intent(),
                        live_is_torqued: self.is_torqued,
                        live_stopped: self.safety.state().is_stopped(),
                    });
            }
            return Err(code);
        }
        #[cfg(not(target_arch = "wasm32"))]
        let live_op_id = self.torque_shadow.alloc_live_op_id();
        let started = if self.driver.is_none() {
            #[cfg(not(target_arch = "wasm32"))]
            self.torque_shadow
                .enqueue(crate::torque::shadow::ShadowInputKind::DisableRequest {
                    live_op_id,
                    explicit_release_authorized: false,
                    live_refusal: None,
                    live_intent: self.shadow_live_intent(),
                    live_is_torqued: self.is_torqued,
                    live_stopped: self.safety.state().is_stopped(),
                });
            self.stop_motion();
            self.commanded_velocities.fill(0.0);
            self.background_disable_retry = None;
            self.disable_requested();
            None
        } else {
            let dof = self.config.dof;
            let feedback_numeric_valid = self.feedback_numeric_valid;
            let active_lease_gen = (self.lease_enabled && !self.safety.state().is_stopped())
                .then(|| self.current_active_lease_gen())
                .filter(|&g| g > 0);
            let mut ages_buf = std::mem::take(&mut self.scratch.ages);
            let mut bounds_buf = std::mem::take(&mut self.scratch.fb.bounds);
            let locked = {
                #[cfg(not(target_arch = "wasm32"))]
                let live_intent = self.shadow_live_intent();
                #[cfg(not(target_arch = "wasm32"))]
                let live_is_torqued = self.is_torqued;
                #[cfg(not(target_arch = "wasm32"))]
                let live_stopped = self.safety.state().is_stopped();
                #[cfg(not(target_arch = "wasm32"))]
                let shadow_rt = &mut self.torque_shadow;
                let d = self.driver.as_mut().expect("checked is_some");
                match d.try_with_mut(|drv| {
                    let requested_at = drv.clock_now();
                    // Enqueue accepted DisableRequest at `requested_at` BEFORE draining the
                    // driver's DisableRequested WriteResult so causal order is exact AND a
                    // failed try_lock never records a spurious accepted request!
                    #[cfg(not(target_arch = "wasm32"))]
                    shadow_rt.enqueue_at(
                        requested_at,
                        crate::torque::shadow::ShadowInputKind::DisableRequest {
                            live_op_id,
                            explicit_release_authorized: false,
                            live_refusal: None,
                            live_intent,
                            live_is_torqued,
                            live_stopped,
                        },
                    );
                    let disable_kind = crate::driver::DisableKind::DisableRequested {
                        started_at: requested_at,
                    };
                    drv.tx_mut().set_disable_authority(None, active_lease_gen);
                    let res = drv.begin_disable_torque_budgeted(disable_kind, 0);
                    let frames_dispatched = drv.disable_frames_dispatched();
                    let dispatched_mask = drv.disable_dispatched_mask();
                    let post_now = drv.clock_now();
                    let snap = OwnerDriverSnapshot::capture_from_driver(
                        &*drv,
                        dof,
                        feedback_numeric_valid,
                        &self.measured_positions,
                        &mut ages_buf,
                        &mut bounds_buf,
                    );
                    #[cfg(not(target_arch = "wasm32"))]
                    shadow_rt.drain_from_driver(drv);
                    (res, requested_at, post_now, disable_kind, frames_dispatched, dispatched_mask, snap)
                }) {
                    Ok(v) => Ok(v),
                    Err(crate::supervisor_lock::DriverTryError::WouldBlock) => Err(true),
                    Err(crate::supervisor_lock::DriverTryError::Poisoned) => Err(false),
                }
            };
            self.scratch.ages = ages_buf;
            self.scratch.fb.bounds = bounds_buf;
            match locked {
                Ok((res, requested_at, post_now, disable_kind, frames_dispatched, dispatched_mask, snap)) => {
                    self.reset_disable_lock_contention();
                    self.owner_driver_snapshot = snap;
                    self.stop_motion();
                    self.commanded_velocities.fill(0.0);
                    self.background_disable_retry = None;
                    self.disable_requested_at(requested_at);
                    Some((res, post_now, disable_kind, frames_dispatched, dispatched_mask))
                }
                Err(true) => {
                    #[cfg(not(target_arch = "wasm32"))]
                    self.torque_shadow
                        .enqueue(crate::torque::shadow::ShadowInputKind::DisableRequest {
                            live_op_id,
                            explicit_release_authorized: false,
                            live_refusal: Some(crate::torque::shadow::LiveRefusalKind::DriverBusy),
                            live_intent: self.shadow_live_intent(),
                            live_is_torqued: self.is_torqued,
                            live_stopped: self.safety.state().is_stopped(),
                        });
                    self.record_disable_lock_contention();
                    return Err("DRIVER_BUSY");
                }
                Err(false) => {
                    #[cfg(not(target_arch = "wasm32"))]
                    self.torque_shadow
                        .enqueue(crate::torque::shadow::ShadowInputKind::DisableRequest {
                            live_op_id,
                            explicit_release_authorized: false,
                            live_refusal: Some(crate::torque::shadow::LiveRefusalKind::Other),
                            live_intent: self.shadow_live_intent(),
                            live_is_torqued: self.is_torqued,
                            live_stopped: self.safety.state().is_stopped(),
                        });
                    return Err("FEEDBACK_UNAVAILABLE");
                }
            }
        };
        if let Some((_, _, disable_kind, _, _)) = started {
            self.active_disable_kind = Some(disable_kind);
            self.active_disable_client_park = None;
        }
        Ok(match started {
            None => {
                self.disable_accepted();
                self.torque_off_finish_rt().map(|()| Rt::Done)
            }
            Some((Ok(crate::driver::TorqueProgress::Pending { .. }), post_now, disable_kind, frames_dispatched, dispatched_mask)) => {
                if self.all_joints_dispatched_for_disable(frames_dispatched, dispatched_mask) {
                    self.disable_first_frame_accepted();
                }
                self.torque_transition = Some(TorqueTransition::Disable {
                    origin: DisableOrigin::BrokerTransition,
                    kind: disable_kind,
                });
                Ok(Rt::Pending { wake: post_now })
            }
            Some((Ok(crate::driver::TorqueProgress::Done(_)), _, _, _, _)) => {
                self.disable_accepted();
                self.torque_off_finish_rt().map(|()| Rt::Done)
            }
            Some((Err(e @ crate::driver::ArmDriverError::PartialDisable { .. }), _, _, _, _)) => {
                let (_, code) = self.handle_partial_disable_abort(e);
                Err(code)
            }
            Some((Err(crate::driver::ArmDriverError::LeaseFenced), _, _, _, _)) => {
                self.clear_active_disable_retry_state();
                Err("LEASE_LOST")
            }
            Some((Err(e), post_now, disable_kind, frames_dispatched, dispatched_mask)) => {
                let refused = matches!(
                    e,
                    crate::driver::ArmDriverError::TorqueOffWouldDrop
                        | crate::driver::ArmDriverError::DisableRequestExpired
                );
                let partial_remaining = self.is_partial_disable_remaining(dispatched_mask);
                let partial_retry = self.commit_partial_disable_retry_kind(
                    disable_kind,
                    post_now,
                    frames_dispatched,
                    partial_remaining,
                );
                if let Some(rk) = partial_retry {
                    let _ = self.arm_background_disable_retry_immediate_with_kind(rk, post_now);
                } else if !refused {
                    let _ = self.arm_background_disable_retry_with_kind(disable_kind, post_now);
                }
                Err(self.disable_step_failed_inner(e, partial_retry.is_some()))
            }
        })
    }
}
