//! Mode-switch runner state machine and `ArmSupervisor` session helpers.

#[allow(unused_imports)]
use super::*;

// scaffold-impl-0
#[cfg(not(target_arch = "wasm32"))]
impl ModeSwitchRunner {
}

// scaffold-impl-1
#[cfg(not(target_arch = "wasm32"))]
impl SupervisorInner {
}

// scaffold-extra-0
impl ArmSupervisor {
    // ---- PR0C: Dual-owner ModeSwitch session orchestrator (PR0A/PR0B shared layer) ----

    /// Admits a PR0C `ModeSwitch` session on the active owner (`OWNER_LEGACY` or `OWNER_EXECUTOR`)
    /// after verifying live service coherence via [`Self::read_services`].
    #[cfg(not(target_arch = "wasm32"))]
    pub async fn start_mode_switch_session(
        &self,
        req: crate::rt_core::requests::config::ModeSwitchStartRequest,
    ) -> Result<u64, crate::rt_core::requests::config::ConfigError> {
        use crate::rt_core::requests::config::{ConfigError, ConfigReply, ConfigRequest};
        let _svc = self
            .read_services(crate::rt_core::publish::ReadRequirements::identity())
            .await
            .map_err(|_| ConfigError::OwnerStopped)?;
        // Routed requests on OWNER_EXECUTOR (`req.client.0 != 0`) MUST obtain a typed P-F1
        // terminal reservation via the shared RAII guard BEFORE enqueue; missing destination or
        // disconnected clients fail closed (never silently downgraded to `None`).
        let handle = self.executor_handle();
        let mut terminal_guard = match handle.as_ref() {
            Some(h) if req.client.0 != 0 => {
                use crate::rt_core::reply_router::{ExternalDestinationPolicy, ExternalSubmitError};
                match h.reserve_session_terminal(
                    req.client,
                    next_mode_switch_terminal_action_id(),
                    ExternalDestinationPolicy::ConnectionOnly,
                ) {
                    Ok(guard) => Some(guard),
                    Err(ExternalSubmitError::Admit(crate::rt_core::broker::AdmitError::RepliesFull)) => {
                        return Err(ConfigError::Busy("REPLIES_FULL"));
                    }
                    Err(ExternalSubmitError::Admit(crate::rt_core::broker::AdmitError::Disconnected)) => {
                        return Err(ConfigError::ConnectionClosed);
                    }
                    Err(ExternalSubmitError::NoDestination) => {
                        return Err(ConfigError::CalibPrecondition {
                            status: 409,
                            code: "NO_DESTINATION",
                            message: "routed mode-switch session has no registered reply destination",
                        });
                    }
                    Err(ExternalSubmitError::Stopped) => return Err(ConfigError::OwnerStopped),
                    Err(ExternalSubmitError::NotAnExternalAction) => {
                        return Err(ConfigError::CalibPrecondition {
                            status: 400,
                            code: "INVALID_COMMAND",
                            message: "session terminal reservation requires an external routed action",
                        });
                    }
                    Err(ExternalSubmitError::Admit(_)) => {
                        return Err(ConfigError::Busy("ADMIT_REJECTED"));
                    }
                }
            }
            _ => None,
        };
        let reservation = terminal_guard
            .as_mut()
            .and_then(|g| g.stage_for_admission());
        let res = match self
            .dispatch_config_request_owned_reliable(ConfigRequest::StartModeSwitch(
                req,
                reservation,
            ))
            .await
        {
            Ok(ConfigReply::ModeSwitchStarted(res)) => res,
            Ok(_) => Err(ConfigError::OwnerStopped),
            Err(e) => Err(e),
        };
        if res.is_ok() {
            if let Some(g) = terminal_guard.as_mut() {
                g.commit_transferred();
            }
        }
        res
    }

    /// Polls the authoritative [`crate::motor_mode::ModeSwitchSnapshot`] for `session_id`
    /// directly via [`Self::read_services`] (no config exchange, no write lock, no command label
    /// mutation, and unaffected by off-RT event stream saturation). Checks both the active slot
    /// and the bounded recent-terminal ring, and falls back to the latest published `StateHub`
    /// snapshot if the owner just transitioned to SAFE-HOLD.
    #[cfg(not(target_arch = "wasm32"))]
    pub async fn poll_mode_switch_session(
        &self,
        session_id: u64,
    ) -> Result<Option<crate::motor_mode::ModeSwitchSnapshot>, crate::rt_core::requests::config::ConfigError>
    {
        let lookup = |meta: &crate::rt_core::publish::PublishedOwnerMeta| {
            meta.mode_switch
                .filter(|s| s.session_id == session_id)
                .or_else(|| {
                    meta.prev_mode_switch_terminal
                        .filter(|s| s.session_id == session_id)
                })
                .map(crate::motor_mode::ModeSwitchSnapshot::from)
        };
        if let Ok(svc) = self
            .read_services(crate::rt_core::publish::ReadRequirements::identity())
            .await
        {
            return Ok(lookup(&svc.snapshot.rt.owner));
        }
        if let Some(snap) = self.hub.latest().and_then(|latest| lookup(&latest.rt.owner)) {
            return Ok(Some(snap));
        }
        Err(crate::rt_core::requests::config::ConfigError::OwnerStopped)
    }

    /// Cancels an active `ModeSwitch` session matching `session_id` (stale IDs return `Ok(None)`).
    #[cfg(not(target_arch = "wasm32"))]
    pub async fn cancel_mode_switch_session(
        &self,
        session_id: u64,
        reason: &'static str,
    ) -> Result<Option<crate::motor_mode::ModeSwitchSnapshot>, crate::rt_core::requests::config::ConfigError>
    {
        use crate::rt_core::requests::config::{ConfigError, ConfigReply, ConfigRequest};
        match self
            .dispatch_config_request_owned_reliable(ConfigRequest::CancelModeSwitch {
                session_id,
                reason,
            })
            .await?
        {
            ConfigReply::ModeSwitchCancelled(snap) => Ok(snap),
            _ => Err(ConfigError::OwnerStopped),
        }
    }

    /// Clears a terminal `ModeSwitch` slot matching `session_id` (returns `false` for stale IDs
    /// or while a router-bound terminal is still pending flush).
    #[cfg(not(target_arch = "wasm32"))]
    pub async fn clear_mode_switch_session(
        &self,
        session_id: u64,
    ) -> Result<bool, crate::rt_core::requests::config::ConfigError> {
        use crate::rt_core::requests::config::{ConfigError, ConfigReply, ConfigRequest};
        match self
            .dispatch_config_request_owned_reliable(ConfigRequest::ClearModeSwitch { session_id })
            .await?
        {
            ConfigReply::ModeSwitchCleared(cleared) => Ok(cleared),
            _ => Err(ConfigError::OwnerStopped),
        }
    }

    /// Waits via `read_services` (`poll_mode_switch_session`) until `session_id` reaches terminal
    /// or `wait_deadline` elapses (cancelling on timeout), clearing the slot when safe.
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) async fn await_mode_switch_session(
        &self,
        session_id: u64,
        wait_deadline: Instant,
    ) -> Result<Option<crate::motor_mode::ModeSwitchSnapshot>, crate::rt_core::requests::config::ConfigError>
    {
        loop {
            match self.poll_mode_switch_session(session_id).await {
                Ok(Some(snap)) if !snap.running => {
                    if !snap.progress.is_ambiguous() && !self.internal_fault_lane.is_fenced() {
                        let _ = self.clear_mode_switch_session(session_id).await;
                    }
                    return Ok(Some(snap));
                }
                Ok(Some(_)) | Ok(None) => {}
                Err(e) => return Err(e),
            }
            if Instant::now() >= wait_deadline {
                let mut snap = self.cancel_mode_switch_session(session_id, "TIMEOUT").await?;
                if snap.is_none() {
                    snap = self
                        .poll_mode_switch_session(session_id)
                        .await?
                        .filter(|s| !s.running);
                }
                if !snap.as_ref().is_some_and(|s| s.progress.is_ambiguous())
                    && !self.internal_fault_lane.is_fenced()
                {
                    let _ = self.clear_mode_switch_session(session_id).await;
                }
                return Ok(snap);
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }

    /// Runs a `ModeSwitch` session to completion on the active owner (`OWNER_LEGACY` or
    /// `OWNER_EXECUTOR`), polling via [`Self::read_services`] until terminal and clearing the slot
    /// before returning the final authoritative snapshot.
    #[cfg(not(target_arch = "wasm32"))]
    pub async fn run_mode_switch_session(
        &self,
        req: crate::rt_core::requests::config::ModeSwitchStartRequest,
        poll_timeout: Duration,
    ) -> Result<crate::motor_mode::ModeSwitchSnapshot, crate::rt_core::requests::config::ConfigError>
    {
        use crate::rt_core::requests::config::ConfigError;
        // R2 Major 8: validate deadline arithmetic BEFORE admitting the session!
        let Some(wait_deadline) = Instant::now().checked_add(poll_timeout) else {
            return Err(ConfigError::CalibPrecondition {
                status: 400,
                code: "INVALID_COMMAND",
                message: "mode-switch poll timeout overflows monotonic clock",
            });
        };
        let session_id = self.start_mode_switch_session(req).await?;
        self.await_mode_switch_session(session_id, wait_deadline)
            .await?
            .ok_or(ConfigError::OwnerStopped)
    }

    /// Reference adapter (PR0C): runs causal J7 gripper mode confirmation (`PositionVelocity` or
    /// `PositionForce`) as a PR0B `SessionKind::ModeSwitch` session on either owner.
    #[cfg(not(target_arch = "wasm32"))]
    pub async fn confirm_gripper_mode_session(
        &self,
        joint: usize,
        mode: crate::motor_mode::ControlMode,
        admission: AdmissionStamp,
        operator_grant: crate::auth::OperatorGrant,
        timeout: Duration,
    ) -> Result<crate::motor_mode::ModeSwitchSnapshot, crate::rt_core::requests::config::ConfigError>
    {
        use crate::rt_core::requests::config::ConfigError;
        let plan = crate::motor_mode::ModeSwitchPlan::confirm_gripper(joint, mode).map_err(|_| {
            ConfigError::CalibPrecondition {
                status: 400,
                code: "INVALID_COMMAND",
                message: "invalid gripper joint index or control mode",
            }
        })?;
        // R3 Minor 1: validate against the configured gripper joint via `read_services`.
        let svc = self
            .read_services(crate::rt_core::publish::ReadRequirements::identity())
            .await
            .map_err(|_| ConfigError::OwnerStopped)?;
        if svc.identity.dof <= svc.identity.arm_dof || joint != svc.identity.arm_dof {
            return Err(ConfigError::CalibPrecondition {
                status: 400,
                code: "INVALID_COMMAND",
                message: "joint is not the configured gripper axis",
            });
        }
        // R2 Major 8: checked deadline & grace arithmetic BEFORE admission!
        let now = Instant::now();
        let Some(deadline) = now.checked_add(timeout) else {
            return Err(ConfigError::CalibPrecondition {
                status: 400,
                code: "INVALID_COMMAND",
                message: "gripper mode confirmation timeout overflows clock",
            });
        };
        let Some(poll_timeout) = timeout.checked_add(Duration::from_millis(250)) else {
            return Err(ConfigError::CalibPrecondition {
                status: 400,
                code: "INVALID_COMMAND",
                message: "gripper mode confirmation poll timeout overflows clock",
            });
        };
        let req = crate::rt_core::requests::config::ModeSwitchStartRequest {
            plan,
            admission,
            operator: crate::auth::OperatorRequirement::Required(operator_grant),
            deadline,
            client: crate::rt_core::broker::ClientId(admission.connection_id),
        };
        self.run_mode_switch_session(req, poll_timeout).await
    }
}
