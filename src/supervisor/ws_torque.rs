//! WebSocket torque/clear-fault RPC handlers and prepared-motion wrappers.

#[allow(unused_imports)]
use super::*;

// scaffold-extra-0
impl ArmSupervisor {
    /// Unified owner-routed WS `/api/v1/torque` and `/api/v1/arm` across legacy and RT executor.
    ///
    /// Legacy runs off-lock snapshot + short write lock using `install_torque_on_legacy` (consuming
    /// A1's `Admitted<'_>`) or `install_torque_off_legacy` + `StateHub` publication.
    /// Executor prepares off-thread from `StateHub`, submits `TransitionOp::{TorqueOn, TorqueOff}`
    /// via `BrokerDispatcher::submit_action`, and awaits BOTH `Ack` AND confirmed `Terminal` before
    /// returning `Accepted` (never acknowledging on early `Ack` before hardware confirmation).
    #[allow(clippy::too_many_arguments)]
    pub async fn torque_ws(
        &self,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
        local_generation: u64,
        supplied_epoch: Option<u64>,
        request_id_valid: bool,
        parsed_request_id: u64,
        enable: Result<bool, &'static str>,
        auth_policy: crate::rt_core::requests::lease::OwnerAuthPolicy,
        operator: crate::auth::OperatorGrant,
        stamp: AdmissionStamp,
    ) -> Result<crate::rt_core::requests::lease::TorqueWsOutcome, ArmError> {
        let override_ms = self.legacy_torque_deadline_override_ms.load(Ordering::Acquire);
        let request_deadline = if override_ms > 0 {
            Duration::from_millis(override_ms)
        } else {
            crate::rt_core::requests::lease::DEFAULT_LEASE_REQUEST_DEADLINE
        };
        self.torque_ws_with_deadline(
            conn,
            local_generation,
            supplied_epoch,
            request_id_valid,
            parsed_request_id,
            enable,
            auth_policy,
            operator,
            stamp,
            request_deadline,
        )
        .await
    }

    /// Unified owner-routed WS `/api/v1/safety/clear_fault` across legacy and RT executor owners.
    ///
    /// Both owners share the exact same authority, pending-E-stop, live [`OperatorGrant`] policy
    /// (supplied by the WS dispatch table), and `motor_zero_busy` precedence in
    /// `requests::lease::install_lease_clear_fault_*`.
    #[allow(clippy::too_many_arguments)]
    pub async fn clear_fault_ws(
        &self,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
        local_generation: u64,
        supplied_epoch: Option<u64>,
        request_id_valid: bool,
        body_error: Option<&'static str>,
        mit_recovery: bool,
        auth_policy: crate::rt_core::requests::lease::OwnerAuthPolicy,
        operator: crate::auth::OperatorGrant,
    ) -> Result<
        Result<crate::rt_core::requests::lease::LeaseClearFaultOutcome, (ArmError, u64)>,
        ArmError,
    > {
        use crate::rt_core::requests::lease::install_lease_clear_fault_legacy;
        match self.owner_mode() {
            OWNER_LEGACY => {
                let mut inner = self.inner.write().await;
                inner.set_command("/api/v1/safety/clear_fault");
                self.check_legacy_under_guard(&inner)?;
                let res = install_lease_clear_fault_legacy(
                    &mut inner,
                    conn,
                    local_generation,
                    supplied_epoch,
                    request_id_valid,
                    body_error,
                    mit_recovery,
                    auth_policy,
                    operator,
                    Instant::now(),
                );
                Ok(res)
            }
            OWNER_EXECUTOR => {
                let Some(handle) = self.executor_handle() else {
                    return Err(executor_unavailable());
                };
                let recovery_started_at = mit_recovery.then(Instant::now);
                let mut final_attempt = false;
                loop {
                    let out = handle
                        .lease_clear_fault_async(
                            conn,
                            local_generation,
                            supplied_epoch,
                            request_id_valid,
                            body_error,
                            mit_recovery,
                            recovery_started_at,
                            auth_policy,
                            operator,
                        )
                        .await
                        .map_err(ArmError::from)?;
                    let incomplete = matches!(
                        out,
                        crate::rt_core::requests::lease::LeaseClearFaultOutcome::ClearFailed {
                            err: crate::safety::ClearFaultError::Rejected(
                                crate::safety::ClearRefusal::MitRecoveryIncomplete
                            ),
                            ..
                        }
                    );
                    if !incomplete || final_attempt {
                        return Ok(Ok(out));
                    }
                    let Some(started) = recovery_started_at else {
                        return Ok(Ok(out));
                    };
                    if Instant::now().saturating_duration_since(started) >= Duration::from_secs(1) {
                        // One final RT step after the 1 s Rule A′ lifetime expires so the driver
                        // observes `DisableRequestExpired` and resets recovery state cleanly.
                        final_attempt = true;
                    } else {
                        tokio::time::sleep(Duration::from_millis(5)).await;
                    }
                }
            }
            _ => Err(executor_unavailable()),
        }
    }

    /// Fast-path lock-free check whether E-stop is currently latched.
    pub(crate) fn is_estop_latched(&self) -> bool {
        self.e_stop_latched.load(Ordering::SeqCst)
    }

    /// Fast-path latest published snapshot for high-rate executor WS servo intake: returns the
    /// current `StateHub` snapshot without waiting once the executor has published at least once,
    /// falling back to `fresh_snapshot(None)` only before the first post-handoff publish.
    pub(crate) async fn latest_executor_snapshot(
        &self,
    ) -> Result<Arc<crate::rt_core::publish::Snapshot>, ArmError> {
        if self.owner_mode() != OWNER_EXECUTOR {
            return Err(executor_unavailable());
        }
        let want = self.executor_base_gen.load(Ordering::Acquire).saturating_add(1);
        let now = Instant::now();
        if let Some(s) = self.hub.latest() {
            if Self::answers(&s, want, now) {
                return Ok(s);
            }
        }
        self.fresh_snapshot(None).await
    }

    pub(super) fn executor_lease_active(&self) -> bool {
        if self.owner_mode() != OWNER_EXECUTOR {
            return false;
        }
        let Some(handle) = self.executor_handle() else {
            return false;
        };
        if handle.is_exited() {
            return false;
        }
        let want = self.executor_base_gen.load(Ordering::Acquire).saturating_add(1);
        let now = Instant::now();
        self.hub
            .latest()
            .is_some_and(|s| Self::answers(&s, want, now) && s.owner.is_lease_active_at(now))
    }

    pub async fn is_lease_active(&self) -> bool {
        if self.owner_mode() == OWNER_EXECUTOR {
            return self.executor_lease_active();
        }
        let inner = self.inner.read().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            return inner.is_lease_active();
        }
        drop(inner);
        self.executor_lease_active()
    }

    pub async fn validate_servo_lease(&self, client_id: &str, generation: u64) -> bool {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_err() {
            return false;
        }
        inner.validate_servo_lease(client_id, generation)
    }

    pub async fn set_torques(&self, torques: Vec<f32>) {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            inner.torques = torques;
        }
    }

    pub async fn is_gripper_stalled(&self) -> bool {
        let inner = self.inner.read().await;
        inner.gripper_stalled
    }

    pub async fn is_gravity_compensated(&self) -> bool {
        let inner = self.inner.read().await;
        inner.gravity_output_enabled()
    }

    pub async fn get_gravity_torques(&self) -> Vec<f32> {
        let inner = self.inner.read().await;
        inner.gravity_torques.clone()
    }

    pub async fn set_gravity_compensated(&self, enabled: bool) {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            inner.gravity_compensated = enabled;
        }
    }

    pub async fn get_gravity_scale(&self) -> f32 {
        let inner = self.inner.read().await;
        inner.gravity_scale
    }

    pub async fn set_gravity_scale(&self, scale: f32) {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            inner.gravity_scale = scale.clamp(0.0, 1.0);
        }
    }

    pub async fn get_gravity_torques_ramped(&self) -> Vec<f32> {
        let inner = self.inner.read().await;
        inner.gravity_torques_ramped.clone()
    }

    pub async fn servo(&self, stamp: AdmissionStamp, targets: Vec<f32>, epoch_id: u64) -> Result<(), ArmError> {
        if self.e_stop_latched.load(Ordering::SeqCst) {
            return Err(ArmError::Safety(SafetyError::EStopActive));
        }
        let mut inner = self.inner.write().await;
        self.check_legacy_under_guard(&inner)?;
        if inner.estop_pending() {
            return Err(ArmError::Safety(SafetyError::EStopActive));
        }
        let admitted = inner.check_admission(stamp)?;
        SupervisorInner::servo(admitted, targets, epoch_id)?;
        Ok(())
    }

    pub async fn execute_stop(&self, policy: StopPolicy) {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            inner.execute_stop(policy);
        }
    }

    pub async fn set_lease_enabled(&self, enabled: bool) {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            inner.lease_enabled = enabled;
        }
    }

    pub async fn set_temperatures(&self, temps: Vec<f32>) {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            inner.temperatures = temps;
        }
    }

    pub(super) async fn run_prepared_motion(
        &self,
        stamp: AdmissionStamp,
        request: crate::rt_core::requests::motion::MotionRequest,
        epoch_id: Option<u64>,
    ) -> Result<u64, ArmError> {
        use crate::rt_core::requests::motion as req;
        const MAX_RETRIES: usize = 5;
        let request_at = Instant::now();
        let cached_cfg = self.get_fresh_legacy_motion_config().await;
        let (mut snapshot, cache_hit) = {
            let inner = self.inner.read().await;
            self.check_legacy_under_guard(&inner)?;
            req::preflight_motion_request_at_with_config(
                &inner,
                stamp,
                &request,
                epoch_id,
                request_at,
                cached_cfg.as_ref(),
            )?
        };
        drop(cached_cfg);
        if !cache_hit {
            self.hub.set_legacy_motion_config_if_newer(Arc::clone(&snapshot.config));
        }
        for attempt in 0..MAX_RETRIES {
            let (snap_plan, req_plan) = (snapshot.clone(), request.clone());
            let prepared = tokio::task::spawn_blocking(move || req::prepare_motion(&snap_plan, &req_plan))
                .await
                .map_err(|e| ArmError::Internal(format!("motion planner failed: {e}")))?;
            let mut inner = self.inner.write().await;
            inner.set_command(match request.kind {
                req::MotionKind::Jog => "/api/v1/motion/jog",
                req::MotionKind::Home => "/api/v1/motion/home",
                req::MotionKind::Park => "/api/v1/motion/unload",
            });
            self.check_legacy_under_guard(&inner)?;
            if let Some(hook) = inner.before_install_hook.take() { hook(&mut inner); }
            let plan = prepared?;
            let res = match request.kind {
                req::MotionKind::Jog => req::install_jog(&mut inner, stamp, &plan.slot, plan.heap_traj, epoch_id, false),
                req::MotionKind::Home => req::install_home(&mut inner, stamp, &plan.slot, plan.heap_traj, epoch_id, false),
                req::MotionKind::Park => req::install_park(&mut inner, stamp, &plan.slot, plan.heap_traj, epoch_id, false),
            };
            match res {
                Ok(installed) => {
                    let aid = match installed {
                        req::MotionInstalled::Started { action_id, .. }
                        | req::MotionInstalled::DeferredContinuation { action_id, .. } => action_id,
                    };
                    inner.set_action_id(aid);
                    return Ok(installed.epoch());
                }
                Err(req::MotionInstallError::StartDrifted) if attempt + 1 < MAX_RETRIES => {
                    if Instant::now() > snapshot.authority.deadline {
                        return Err(req::MotionInstallError::DeadlineExpired.into_arm_error(&inner, request.action_kind, epoch_id));
                    }
                    let Some(next) = req::MotionPlanSnapshot::refresh_for_drift_retry(&inner, &snapshot) else {
                        return Err(req::MotionInstallError::PlanStale.into_arm_error(&inner, request.action_kind, epoch_id));
                    };
                    snapshot = next;
                }
                Err(e) => return Err(e.into_arm_error(&inner, request.action_kind, epoch_id)),
            }
        }
        let inner = self.inner.read().await;
        Err(req::MotionInstallError::PlanStale.into_arm_error(&inner, request.action_kind, epoch_id))
    }

    pub async fn jog(
        &self,
        stamp: AdmissionStamp,
        target_positions: Vec<f32>,
        duration_secs: Option<f64>,
        epoch_id: Option<u64>,
    ) -> Result<u64, ArmError> {
        let req = crate::rt_core::requests::motion::MotionRequest::jog(target_positions, duration_secs, None);
        self.run_prepared_motion(stamp, req, epoch_id).await
    }

    pub async fn home(
        &self,
        stamp: AdmissionStamp,
        duration_secs: Option<f64>,
        epoch_id: Option<u64>,
    ) -> Result<u64, ArmError> {
        let req = crate::rt_core::requests::motion::MotionRequest::home(duration_secs);
        self.run_prepared_motion(stamp, req, epoch_id).await
    }

    pub async fn stop_motion(&self) {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            inner.stop_motion();
        }
    }

    #[doc(hidden)]
    #[allow(clippy::too_many_arguments)]
    pub async fn torque_ws_with_deadline(
        &self,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
        local_generation: u64,
        supplied_epoch: Option<u64>,
        request_id_valid: bool,
        parsed_request_id: u64,
        enable: Result<bool, &'static str>,
        auth_policy: crate::rt_core::requests::lease::OwnerAuthPolicy,
        operator: crate::auth::OperatorGrant,
        stamp: AdmissionStamp,
        request_deadline: Duration,
    ) -> Result<crate::rt_core::requests::lease::TorqueWsOutcome, ArmError> {
        use crate::rt_core::broker::{Outcome, Reply, TransitionOp};
        use crate::rt_core::requests::lease::{
            check_torque_owner_preflight, map_torque_code_to_api_error, prepare_torque,
            prepare_torque_ws, OwnerAuthPolicy, TorquePreflightError, TorqueWsOutcome,
        };
        let map_preflight_err = |err: TorquePreflightError| -> TorqueWsOutcome {
            match err {
                TorquePreflightError::LeaseLost { epoch_id } => {
                    TorqueWsOutcome::LeaseLost { epoch_id }
                }
                TorquePreflightError::EpochMismatch { epoch_id } => {
                    TorqueWsOutcome::EpochMismatch { epoch_id }
                }
                TorquePreflightError::InvalidRequestId => TorqueWsOutcome::InvalidRequestId,
                TorquePreflightError::OperatorRequired {
                    operation,
                    epoch_id,
                } => TorqueWsOutcome::Rejected {
                    error: crate::api::rest::ApiError::operator_required(operation),
                    epoch_id,
                },
                TorquePreflightError::MotionBusy { epoch_id } => TorqueWsOutcome::Rejected {
                    error: crate::api::rest::ApiError(
                        axum::http::StatusCode::CONFLICT,
                        "a calibration apply, restore or motor zero is running".into(),
                        "MOTION_BUSY".into(),
                    ),
                    epoch_id,
                },
                TorquePreflightError::InvalidBody { message, epoch_id } => {
                    TorqueWsOutcome::InvalidBody { message, epoch_id }
                }
            }
        };
        // Off-lock typed preparation shared by both legacy and RT executor owners.
        let snap = self.fresh_snapshot(None).await?;
        let now = Instant::now();
        let deadline = now + request_deadline;
        let prep_ws = prepare_torque_ws(
            &snap.owner,
            conn,
            local_generation,
            supplied_epoch,
            request_id_valid,
            parsed_request_id,
            enable,
            auth_policy,
            operator,
            stamp,
            now,
            deadline,
        )
        .map_err(|_| observation_unavailable())?;
        match self.owner_mode() {
            OWNER_LEGACY => {
                // Drop the pre-admission snapshot immediately so the off-lock wait never pins a
                // `StateHub` publication slot during the legacy torque transition.
                drop(snap);
                use crate::supervisor::driver_transitions::{
                    LegacyTorqueFailure, LegacyTorqueOutcome, LegacyTorqueSlot, LegacyTorqueStart,
                };
                let map_legacy_failure = |failure: LegacyTorqueFailure,
                                          epoch_id: u64,
                                          safety_state: &SafetyState,
                                          joint_names: &[String]|
                 -> TorqueWsOutcome {
                    match failure {
                        LegacyTorqueFailure::TorqueOffWouldDrop(refusal) => {
                            let err = ArmError::Safety(
                                refusal.into_safety_error(joint_names),
                            );
                            TorqueWsOutcome::Rejected {
                                error: err.into(),
                                epoch_id,
                            }
                        }
                        LegacyTorqueFailure::EnableFailed {
                            detail,
                            feedback_unavailable,
                            estop_interrupted,
                        } => {
                            // Preserve exact legacy WS wire mapping (`SafetyError::ClearFaultRejected`
                            // -> `CLEAR_FAULT_REJECTED` with `safety_state: "Fault"`), while Safety
                            // still latches `FaultCode::EnableFailed` internally.
                            let err = if feedback_unavailable {
                                ArmError::Safety(SafetyError::FeedbackUnavailable)
                            } else if estop_interrupted {
                                ArmError::Safety(SafetyError::EStopActive)
                            } else {
                                let msg = if detail.as_str().is_empty() {
                                    "Hardware torque enable failed".to_string()
                                } else {
                                    detail.as_str().to_string()
                                };
                                ArmError::Safety(SafetyError::ClearFaultRejected(msg))
                            };
                            TorqueWsOutcome::Rejected {
                                error: err.into(),
                                epoch_id,
                            }
                        }
                        LegacyTorqueFailure::DisableFailed { detail } => {
                            // Preserve exact legacy WS wire mapping (`SafetyError::ClearFaultRejected`
                            // -> `CLEAR_FAULT_REJECTED` with `safety_state: "Fault"`), while Safety
                            // still latches `FaultCode::DriverWriteFailed` internally.
                            let msg = if detail.as_str().is_empty() {
                                "Torque disable transmission failed".to_string()
                            } else {
                                detail.as_str().to_string()
                            };
                            TorqueWsOutcome::Rejected {
                                error: ArmError::Safety(SafetyError::ClearFaultRejected(msg)).into(),
                                epoch_id,
                            }
                        }
                        LegacyTorqueFailure::Stopped => TorqueWsOutcome::Rejected {
                            error: ArmError::Safety(SafetyError::Stopped).into(),
                            epoch_id,
                        },
                        LegacyTorqueFailure::LeaseLost => TorqueWsOutcome::LeaseLost { epoch_id },
                        LegacyTorqueFailure::EpochMismatch => {
                            TorqueWsOutcome::EpochMismatch { epoch_id }
                        }
                        LegacyTorqueFailure::FeedbackUnavailable => TorqueWsOutcome::Rejected {
                            error: ArmError::Safety(SafetyError::FeedbackUnavailable).into(),
                            epoch_id,
                        },
                        LegacyTorqueFailure::EStopActive => TorqueWsOutcome::Rejected {
                            error: ArmError::Safety(SafetyError::EStopActive).into(),
                            epoch_id,
                        },
                        LegacyTorqueFailure::StaticCode(code) => TorqueWsOutcome::Rejected {
                            error: map_torque_code_to_api_error(code, Some(safety_state)),
                            epoch_id,
                        },
                    }
                };

                #[cfg(not(target_arch = "wasm32"))]
                if prep_ws.enable == Some(true)
                    && self
                        .published_config()
                        .await?
                        .driver_supports_nonblocking_mode_switch
                {
                    let needs_handback = {
                        let mut inner = self.inner.write().await;
                        let io_scope = crate::diagnostics::tick::HoldIoScope::enter();
                        inner.set_command("/api/v1/torque");
                        inner.set_direction("on");
                        if let Err(e) = self.check_legacy_under_guard(&inner) {
                            inner.set_io_timing(io_scope.finish());
                            return Err(e);
                        }
                        if let Err(err) = check_torque_owner_preflight(
                            &mut inner,
                            prep_ws.connection,
                            prep_ws.local_generation,
                            prep_ws.supplied_epoch,
                            prep_ws.request_id_valid,
                            prep_ws.body_error,
                            prep_ws.auth_policy,
                            prep_ws.operator,
                            Instant::now(),
                        ) {
                            inner.set_io_timing(io_scope.finish());
                            return Ok(map_preflight_err(err));
                        }
                        if Instant::now() >= deadline {
                            let epoch_id = inner.safety.epoch_id();
                            let safety_state = inner.safety.state().clone();
                            inner.set_io_timing(io_scope.finish());
                            return Ok(map_legacy_failure(
                                LegacyTorqueFailure::StaticCode("DEADLINE_EXPIRED"),
                                epoch_id,
                                &safety_state,
                                &[],
                            ));
                        }
                        let active =
                            inner.driver_supports_nonblocking_mode_switch() && inner.mit_active();
                        inner.set_io_timing(io_scope.finish());
                        active
                    };
                    if needs_handback {
                        let lease_holder = (prep_ws.local_generation != 0).then(|| {
                            (
                                prep_ws.connection.to_holder_label(),
                                prep_ws.local_generation,
                            )
                        });
                        let mut qual_stamp = prep_ws.stamp;
                        qual_stamp.request_id = next_admission_request_id();
                        qual_stamp.connection_id = prep_ws.connection.id();
                        qual_stamp.lease_gen = prep_ws.local_generation;
                        if let Some(ep) = prep_ws.supplied_epoch {
                            qual_stamp.estop_epoch = ep;
                        }
                        if let Err(err) = crate::rt_core::requests::config::run_executor_mit_hand_back(
                            self,
                            crate::supervisor::MitHandBackReason::Rearm,
                            qual_stamp,
                            lease_holder,
                            crate::auth::OperatorRequirement::NotRequired,
                            crate::rt_core::broker::ClientId(prep_ws.connection.id()),
                            Duration::from_secs(2),
                        )
                        .await
                        {
                            let epoch_id = self.inner.read().await.safety.epoch_id();
                            return Ok(TorqueWsOutcome::Rejected {
                                error: err.into(),
                                epoch_id,
                            });
                        }
                    }
                }

                let (enabled, start_res, pre_epoch, joint_names, tick_seq_counter, mut last_tick_seq) = {
                    let mut inner = self.inner.write().await;
                    let io_scope = crate::diagnostics::tick::HoldIoScope::enter();
                    inner.set_command("/api/v1/torque");
                    if let Ok(en) = enable {
                        inner.set_direction(if en { "on" } else { "off" });
                    }
                    if let Err(e) = self.check_legacy_under_guard(&inner) {
                        inner.set_io_timing(io_scope.finish());
                        return Err(e);
                    }
                    if let Err(err) = check_torque_owner_preflight(
                        &mut inner,
                        prep_ws.connection,
                        prep_ws.local_generation,
                        prep_ws.supplied_epoch,
                        prep_ws.request_id_valid,
                        prep_ws.body_error,
                        prep_ws.auth_policy,
                        prep_ws.operator,
                        Instant::now(),
                    ) {
                        inner.set_io_timing(io_scope.finish());
                        return Ok(map_preflight_err(err));
                    }
                    let enabled = prep_ws.enable.expect("checked by check_torque_owner_preflight");
                    if Instant::now() >= deadline {
                        let epoch_id = inner.safety.epoch_id();
                        let safety_state = inner.safety.state().clone();
                        inner.set_io_timing(io_scope.finish());
                        return Ok(map_legacy_failure(
                            LegacyTorqueFailure::StaticCode("DEADLINE_EXPIRED"),
                            epoch_id,
                            &safety_state,
                            &[],
                        ));
                    }
                    let joint_names = if !enabled {
                        inner.config.joint_names.clone()
                    } else {
                        Vec::new()
                    };
                    let tick_seq_counter = Arc::clone(&inner.control_release_seq_atomic);
                    let last_tick_seq = tick_seq_counter.load(Ordering::Acquire);
                    let res = if enabled {
                        if inner.driver.is_some() && !inner.driver_supports_rt_executor_nonblocking() {
                            // Serial & RobStride drivers stay on the untouched synchronous
                            // `enable_torque` path (full bus read, calibration, range check, and
                            // measured-goal pre-seed with blocking driver lock).
                            inner
                                .check_admission(prep_ws.stamp)
                                .and_then(SupervisorInner::enable_torque)
                                .map(|_| LegacyTorqueStart::ImmediateAccepted {
                                    epoch_id: inner.safety.epoch_id(),
                                    safety_state: inner.safety.state().clone(),
                                    is_torqued: inner.is_torqued,
                                    control_mode_gravity_comp: inner.gravity_comp_active(),
                                })
                                .map_err(ArmError::from)
                        } else {
                            inner
                                .check_admission(prep_ws.stamp)
                                .and_then(|a| {
                                    SupervisorInner::begin_enable_torque_transition_legacy(a, deadline)
                                })
                                .map_err(ArmError::from)
                        }
                    } else {
                        inner
                            .begin_disable_torque_transition_legacy(deadline)
                            .map_err(ArmError::from)
                    };
                    let epoch_id = inner.safety.epoch_id();
                    inner.set_io_timing(io_scope.finish());
                    (enabled, res, epoch_id, joint_names, tick_seq_counter, last_tick_seq)
                };

                let (token, slot_idx, slot) = match start_res {
                    Ok(LegacyTorqueStart::ImmediateAccepted {
                        epoch_id,
                        safety_state,
                        is_torqued,
                        control_mode_gravity_comp,
                    }) => {
                        return Ok(TorqueWsOutcome::Accepted {
                            epoch_id,
                            safety_state,
                            is_torqued,
                            control_mode_gravity_comp,
                        });
                    }
                    Ok(LegacyTorqueStart::ImmediateRejected {
                        failure,
                        epoch_id,
                        safety_state,
                    }) => {
                        return Ok(map_legacy_failure(
                            failure,
                            epoch_id,
                            &safety_state,
                            &joint_names,
                        ));
                    }
                    Ok(LegacyTorqueStart::Pending {
                        token,
                        slot_idx,
                        slot,
                    }) => (token, slot_idx, slot),
                    Err(err) => {
                        return Ok(TorqueWsOutcome::Rejected {
                            error: err.into(),
                            epoch_id: pre_epoch,
                        });
                    }
                };

                struct LegacySlotGuard {
                    slot: Arc<LegacyTorqueSlot>,
                    token: u64,
                    consumed: bool,
                }
                impl Drop for LegacySlotGuard {
                    fn drop(&mut self) {
                        if !self.consumed {
                            self.slot.abandon(self.token);
                        }
                    }
                }
                let mut slot_guard = LegacySlotGuard {
                    slot: Arc::clone(&slot),
                    token,
                    consumed: false,
                };

                // Bounded off-lock waiter polling + headless fallback:
                // - Normal live control loop (50 Hz): `tick_shared` publishes wait-free to the
                //   atomic slot and calls `self.legacy_torque_notify.notify_waiters()` strictly
                //   OFF-LOCK after dropping `SupWriteGuard`; the 5 ms poll cap catches any
                //   non-tick settlement (Stop / E-stop / lease fence / handoff race) cleanly.
                // - Headless mode (no ticking control loop): fallback is throttled to >= 25 ms AND
                //   re-verifies `inner.control_release_seq == last_tick_seq` under the write guard
                //   so a live control tick that ran while queued keeps single progress ownership.
                const WAIT_POLL_INTERVAL: Duration = Duration::from_millis(5);
                // Wait > one 50 Hz period (25 ms) before assuming headless mode (no live ticker);
                // once confirmed headless, step every 5 ms so multi-phase Damiao POS_FORCE
                // handshakes complete well inside phase/freshness bounds while still keeping every
                // write guard <= 500 us and yielding immediately if a live ticker resumes.
                const FALLBACK_IDLE_DETECT: Duration = Duration::from_millis(25);
                const FALLBACK_HEADLESS_INTERVAL: Duration = Duration::from_millis(5);
                const FALLBACK_STEP_BUDGET: Duration = Duration::from_micros(500);
                let mut last_fallback_step = Instant::now();
                let mut headless_active = false;

                let outcome = loop {
                    if let Some(done) = slot.take_settled(token) {
                        slot_guard.consumed = true;
                        break done;
                    }
                    let now_wait = Instant::now();
                    if now_wait >= deadline {
                        let (done, needs_headless_drain, settled_any, cur_seq) = {
                            let mut inner = self.inner.write().await;
                            let io_scope = crate::diagnostics::tick::HoldIoScope::enter();
                            inner.set_command("/api/v1/torque");
                            inner.set_direction(if enabled { "on" } else { "off" });
                            let out = if inner.detached {
                                slot.take_settled(token)
                                    .unwrap_or(LegacyTorqueOutcome::OwnerTransferred)
                            } else {
                                inner.timeout_legacy_torque_transition(token, slot_idx, enabled)
                            };
                            let needs_drain =
                                !inner.detached && inner.has_detached_torque_transition();
                            let settled_any =
                                std::mem::replace(&mut inner.legacy_torque_settled_this_tick, false);
                            let seq = inner.control_release_seq;
                            inner.set_io_timing(io_scope.finish());
                            (out, needs_drain, settled_any, seq)
                        };
                        if settled_any {
                            self.legacy_torque_notify.notify_waiters();
                        }
                        if needs_headless_drain {
                            self.spawn_headless_detached_drain_if_needed(cur_seq);
                        }
                        slot_guard.consumed = true;
                        break done;
                    }
                    let notified = self.legacy_torque_notify.notified();
                    if let Some(done) = slot.take_settled(token) {
                        slot_guard.consumed = true;
                        break done;
                    }
                    let rem = deadline.saturating_duration_since(now_wait);
                    let wait_for = rem.min(WAIT_POLL_INTERVAL);
                    let _ = tokio::time::timeout(wait_for, notified).await;
                    if let Some(done) = slot.take_settled(token) {
                        slot_guard.consumed = true;
                        break done;
                    }
                    let cur_tick_seq = tick_seq_counter.load(Ordering::Acquire);
                    let step_interval = if headless_active {
                        FALLBACK_HEADLESS_INTERVAL
                    } else {
                        FALLBACK_IDLE_DETECT
                    };
                    if cur_tick_seq != last_tick_seq {
                        last_tick_seq = cur_tick_seq;
                        last_fallback_step = Instant::now();
                        headless_active = false;
                    } else if last_fallback_step.elapsed() >= step_interval
                        && Instant::now() < deadline
                    {
                        last_fallback_step = Instant::now();
                        let (expired_out, needs_headless_drain, settled_any, observed_seq, did_headless_step) = {
                            let mut inner = self.inner.write().await;
                            let io_scope = crate::diagnostics::tick::HoldIoScope::enter();
                            inner.set_command("/api/v1/torque");
                            inner.set_direction(if enabled { "on" } else { "off" });
                            let step_now = Instant::now();
                            let mut stepped = false;
                            let out = if step_now >= deadline {
                                Some(if inner.detached {
                                    slot.take_settled(token)
                                        .unwrap_or(LegacyTorqueOutcome::OwnerTransferred)
                                } else {
                                    inner.timeout_legacy_torque_transition(
                                        token, slot_idx, enabled,
                                    )
                                })
                            } else {
                                // Re-verify under the write guard that no live control tick ran
                                // while we waited to acquire `SupervisorLock::write`, then advance
                                // `control_release_seq` so Damiao's per-release TX budget refreshes
                                // cleanly on each headless fallback step.
                                if !inner.detached
                                    && inner.control_release_seq == last_tick_seq
                                    && inner.has_legacy_torque_transition_token(token)
                                {
                                    inner.control_release_seq =
                                        inner.control_release_seq.wrapping_add(1).max(1);
                                    inner
                                        .control_release_seq_atomic
                                        .store(inner.control_release_seq, Ordering::Release);
                                    let budget = crate::rt_core::ops::ServiceBudget {
                                        now: step_now,
                                        deadline: step_now + FALLBACK_STEP_BUDGET,
                                        max_rx_frames: crate::rt_core::ops::RX_FRAME_BUDGET,
                                    };
                                    #[cfg(not(target_arch = "wasm32"))]
                                    {
                                        let live_intent = inner.shadow_live_intent();
                                        let live_is_torqued = inner.is_torqued;
                                        let live_stopped = inner.safety.state().is_stopped();
                                        inner.torque_shadow.enqueue_at(
                                            step_now,
                                            crate::torque::shadow::ShadowInputKind::TransitionWake {
                                                live_intent,
                                                live_is_torqued,
                                                live_stopped,
                                            },
                                        );
                                    }
                                    let _ = inner.service_torque_transition(&budget);
                                    inner.flush_torque_shadow();
                                    stepped = true;
                                }
                                None
                            };
                            let needs_drain =
                                !inner.detached && inner.has_detached_torque_transition();
                            let settled_any =
                                std::mem::replace(&mut inner.legacy_torque_settled_this_tick, false);
                            let seq = inner.control_release_seq;
                            inner.set_io_timing(io_scope.finish());
                            (out, needs_drain, settled_any, seq, stepped)
                        };
                        if settled_any {
                            self.legacy_torque_notify.notify_waiters();
                        }
                        if needs_headless_drain {
                            self.spawn_headless_detached_drain_if_needed(observed_seq);
                        }
                        if let Some(done) = expired_out {
                            slot_guard.consumed = true;
                            break done;
                        }
                        headless_active = did_headless_step;
                        last_tick_seq = observed_seq;
                    }
                };
                // Schedule bounded headless cleanup if ANY settlement path (contended Stop,
                // E-stop, lease loss, or timeout) left a detached transition pending (R5 M4):
                // if a live ticker is running, the drain task exits on its first wake once
                // `control_release_seq` advances.
                if self.detached_torque_pending_atomic.load(Ordering::Acquire) {
                    let cur_seq = tick_seq_counter.load(Ordering::Acquire);
                    self.spawn_headless_detached_drain_if_needed(cur_seq);
                }

                match outcome {
                    LegacyTorqueOutcome::Accepted {
                        epoch_id,
                        safety_state,
                        is_torqued,
                        control_mode_gravity_comp,
                    } => Ok(TorqueWsOutcome::Accepted {
                        epoch_id,
                        safety_state: (*safety_state).clone(),
                        is_torqued,
                        control_mode_gravity_comp,
                    }),
                    LegacyTorqueOutcome::Failed {
                        failure,
                        epoch_id,
                        safety_state,
                    } => Ok(map_legacy_failure(
                        failure,
                        epoch_id,
                        &safety_state,
                        &joint_names,
                    )),
                    LegacyTorqueOutcome::OwnerTransferred => {
                        Err(ArmError::Safety(SafetyError::OwnerTransferred))
                    }
                }
            }
            OWNER_EXECUTOR => {
                let Some(handle) = self.executor_handle() else {
                    return Err(executor_unavailable());
                };
                // Malformed requests or rows carrying an owner auth policy evaluate LIVE owner
                // authority through the single shared `check_torque_owner_preflight` (never stale
                // snapshot-only checks), preserving exact legacy/executor precedence parity.
                if prep_ws.broker.is_none() || prep_ws.auth_policy != OwnerAuthPolicy::None {
                    let preflight = handle
                        .lease_torque_preflight_async(
                            prep_ws.connection,
                            prep_ws.local_generation,
                            prep_ws.supplied_epoch,
                            prep_ws.request_id_valid,
                            prep_ws.body_error,
                            prep_ws.auth_policy,
                            prep_ws.operator,
                        )
                        .await
                        .map_err(ArmError::from)?;
                    if let Err(err) = preflight {
                        return Ok(map_preflight_err(err));
                    }
                }
                let enabled = prep_ws.enable.expect("checked above");
                let op = if enabled {
                    TransitionOp::TorqueOn
                } else {
                    TransitionOp::TorqueOff
                };
                let mut prep = prep_ws.broker.expect("checked above");

                // MIT-configured profiles on OWNER_EXECUTOR: run off-RT qualification preflight
                // (sched::mit_refusal + optional rearm handback + ReadIdentityForEnable +
                // mit::check_gate + admission-bound MitGateProof) BEFORE submitting TorqueOn.
                if enabled && crate::mit::mit_joint_mask(&self.published_config().await?.config) != 0 {
                    let lease_holder = (prep_ws.local_generation != 0).then(|| {
                        (
                            prep_ws.connection.to_holder_label(),
                            prep_ws.local_generation,
                        )
                    });
                    let mut qual_stamp = prep.stamp;
                    qual_stamp.request_id = prep.client_seq;
                    qual_stamp.connection_id = prep.client.0;
                    qual_stamp.lease_gen = prep.authority.lease_generation;
                    qual_stamp.estop_epoch = prep.authority.epoch;
                    qual_stamp.stop_gen = prep.authority.stop_generation;
                    if let Err(err) = crate::mit::run_mit_qualification_for_enable_executor(
                        self,
                        qual_stamp,
                        lease_holder,
                        prep_ws.operator,
                        prep.client,
                    )
                    .await
                    {
                        let epoch_id = self
                            .hub
                            .latest()
                            .map_or(snap.obs.epoch_id, |s| s.obs.epoch_id);
                        return Ok(TorqueWsOutcome::Rejected {
                            error: ArmError::Safety(err).into(),
                            epoch_id,
                        });
                    }
                }

                for attempt in 0..2 {
                    let reply = handle
                        .submit_transition_async(
                            prep.client,
                            prep.client_seq,
                            prep.authority,
                            prep.deadline,
                            op,
                        )
                        .await
                        .map_err(Self::map_broker_dispatch_err)?;
                    match reply {
                        Reply::Refused {
                            code: "PLAN_STALE",
                            ..
                        } if attempt == 0 => {
                            let fresh = self.fresh_snapshot(None).await?;
                            prep = prepare_torque(
                                &fresh.owner,
                                prep_ws.connection,
                                prep_ws.local_generation,
                                prep_ws.supplied_epoch,
                                enabled,
                                prep_ws.parsed_request_id,
                                prep_ws.stamp,
                                Instant::now(),
                                deadline,
                            )
                            .map_err(|_| observation_unavailable())?;
                        }
                        Reply::Refused {
                            code: "LEASE_LOST" | "NO_LEASE" | "DISCONNECTED",
                            ..
                        } => {
                            let epoch_id = self
                                .hub
                                .latest()
                                .map_or(snap.obs.epoch_id, |s| s.obs.epoch_id);
                            return Ok(TorqueWsOutcome::LeaseLost { epoch_id });
                        }
                        Reply::Refused {
                            code: "EPOCH_MISMATCH",
                            ..
                        } => {
                            let epoch_id = self
                                .hub
                                .latest()
                                .map_or(snap.obs.epoch_id, |s| s.obs.epoch_id);
                            return Ok(TorqueWsOutcome::EpochMismatch { epoch_id });
                        }
                        Reply::Refused { code, .. } => {
                            let post = self.hub.latest();
                            let epoch_id = post
                                .as_ref()
                                .map_or(snap.obs.epoch_id, |s| s.obs.epoch_id);
                            let st = post.as_ref().map(|s| &s.obs.safety_state);
                            return Ok(TorqueWsOutcome::Rejected {
                                error: map_torque_code_to_api_error(code, st),
                                epoch_id,
                            });
                        }
                        Reply::Terminal {
                            outcome: Outcome::Completed,
                            ..
                        } => {
                            let post = self.hub.latest().unwrap_or(snap);
                            return Ok(TorqueWsOutcome::Accepted {
                                epoch_id: post.obs.epoch_id,
                                safety_state: post.obs.safety_state.clone(),
                                is_torqued: post.obs.is_torqued,
                                control_mode_gravity_comp: post.owner.gravity_comp_active,
                            });
                        }
                        Reply::Terminal { outcome, .. } => {
                            let code = match outcome {
                                Outcome::Failed(c) | Outcome::Cancelled(c) => c,
                                Outcome::Stopped => "STOPPED",
                                Outcome::Superseded => "SUPERSEDED",
                                Outcome::Completed => unreachable!(),
                            };
                            let post = self.hub.latest();
                            let epoch_id = post
                                .as_ref()
                                .map_or(snap.obs.epoch_id, |s| s.obs.epoch_id);
                            if matches!(code, "LEASE_LOST" | "NO_LEASE" | "DISCONNECTED") {
                                return Ok(TorqueWsOutcome::LeaseLost { epoch_id });
                            }
                            if code == "EPOCH_MISMATCH" {
                                return Ok(TorqueWsOutcome::EpochMismatch { epoch_id });
                            }
                            let st = post.as_ref().map(|s| &s.obs.safety_state);
                            return Ok(TorqueWsOutcome::Rejected {
                                error: map_torque_code_to_api_error(code, st),
                                epoch_id,
                            });
                        }
                        _ => {
                            return Err(ArmError::Internal(
                                "Unexpected broker reply for torque transition".into(),
                            ));
                        }
                    }
                }
                Err(ArmError::Rejected {
                    status: 409,
                    code: "PLAN_STALE".into(),
                    message: "Torque transition superseded by concurrent config change".into(),
                })
            }
            _ => Err(executor_unavailable()),
        }
    }
}
