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
}
