//! Config request dispatch, test hooks, and legacy motion-config cache helpers.

#[allow(unused_imports)]
use super::super::*;

impl ArmSupervisor {
    /// Routes one [`ConfigRequest`] through the active single owner (`Legacy` write lock or
    /// `Executor` `MaintenanceOp::Config`) using the identical owner-side handler.
    pub async fn dispatch_config_request(
        &self,
        req: crate::rt_core::requests::config::ConfigRequest,
    ) -> Result<
        crate::rt_core::requests::config::ConfigReply,
        crate::rt_core::requests::config::ConfigError,
    > {
        use crate::rt_core::executor::OwnerError;
        use crate::rt_core::requests::config::{
            handle_config_exchange, ConfigError, ConfigExchange,
        };
        match self.owner_mode() {
            OWNER_EXECUTOR => {
                let Some(handle) = self.executor_handle() else {
                    return Err(ConfigError::OwnerStopped);
                };
                handle.call_config_async(req).await.map_err(|e| match e {
                    OwnerError::QueueFull => ConfigError::OwnerQueueFull,
                    _ => ConfigError::OwnerStopped,
                })
            }
            OWNER_LEGACY => {
                let cmd_label = req.command_label();
                let mut ex = ConfigExchange::new(req);
                let mut inner = self.inner.write().await;
                inner.set_command(cmd_label);
                if inner.detached {
                    drop(inner);
                    let req = ex.req.take().expect("request still present");
                    if self.owner_mode() == OWNER_EXECUTOR {
                        let Some(handle) = self.executor_handle() else {
                            return Err(ConfigError::OwnerStopped);
                        };
                        return handle.call_config_async(req).await.map_err(|e| match e {
                            OwnerError::QueueFull => ConfigError::OwnerQueueFull,
                            _ => ConfigError::OwnerStopped,
                        });
                    }
                    return Err(ConfigError::OwnerStopped);
                }
                let ctx = crate::rt_core::ops::CommitCtx {
                    arm: inner.core.id(),
                    boot: crate::rt_core::broker::BootId::process(),
                    now: Instant::now(),
                    stop_generation: inner.current_stop_gen(),
                };
                handle_config_exchange(&mut inner, &ctx, &mut ex);
                drop(inner);
                ex.reply.take().ok_or(ConfigError::OwnerStopped)
            }
            _ => Err(ConfigError::OwnerStopped),
        }
    }

    /// Dispatches a `ConfigRequest` that must survive transient `MaintenanceOp` queue
    /// backpressure (used for `Persisted`, `Activate`, `Release`, `VerifyAuthority`,
    /// `CalibrationRevert`, and `SetCalibrationBusy(false)` cleanup).
    ///
    /// Retries `ConfigError::OwnerQueueFull` with short bounded backoff while the owner is
    /// still alive; returns immediately on any non-backpressure outcome or `OwnerStopped`.
    pub async fn dispatch_config_request_reliable(
        &self,
        mut build_req: impl FnMut() -> crate::rt_core::requests::config::ConfigRequest,
    ) -> Result<
        crate::rt_core::requests::config::ConfigReply,
        crate::rt_core::requests::config::ConfigError,
    > {
        use crate::rt_core::requests::config::ConfigError;
        let mut backoff_ms = 1u64;
        loop {
            match self.dispatch_config_request(build_req()).await {
                Err(ConfigError::OwnerQueueFull) => {
                    if self.owner_mode() == OWNER_EXECUTOR
                        && self.executor_handle().is_none_or(|h| h.is_exited())
                    {
                        return Err(ConfigError::OwnerStopped);
                    }
                    tokio::time::sleep(Duration::from_millis(backoff_ms)).await;
                    backoff_ms = (backoff_ms * 2).min(10);
                }
                other => return other,
            }
        }
    }

    /// Dispatches a one-shot non-Clone `ConfigRequest` (e.g., carrying a unique `Reservation`
    /// token or `ConfigCandidate`) with guaranteed retry across transient `OwnerQueueFull`.
    /// When the executor queue is full, `MaintClient::call_config_retryable_async` returns the
    /// exact untouched `ConfigRequest` so ownership is never lost.
    pub async fn dispatch_config_request_owned_reliable(
        &self,
        mut req: crate::rt_core::requests::config::ConfigRequest,
    ) -> Result<
        crate::rt_core::requests::config::ConfigReply,
        crate::rt_core::requests::config::ConfigError,
    > {
        use crate::rt_core::requests::config::{
            handle_config_exchange, ConfigError, ConfigExchange,
        };
        let mut backoff_ms = 1u64;
        loop {
            match self.owner_mode() {
                OWNER_EXECUTOR => {
                    let Some(handle) = self.executor_handle() else {
                        return Err(ConfigError::OwnerStopped);
                    };
                    if handle.is_exited() {
                        return Err(ConfigError::OwnerStopped);
                    }
                    match handle.call_config_try_async(req).await {
                        Ok(reply) => return Ok(reply),
                        Err((returned_req, crate::rt_core::executor::OwnerError::QueueFull)) => {
                            req = returned_req;
                            tokio::time::sleep(Duration::from_millis(backoff_ms)).await;
                            backoff_ms = (backoff_ms * 2).min(10);
                        }
                        Err((_, _)) => return Err(ConfigError::OwnerStopped),
                    }
                }
                OWNER_LEGACY => {
                    let cmd_label = req.command_label();
                    let mut ex = ConfigExchange::new(req);
                    let mut inner = self.inner.write().await;
                    inner.set_command(cmd_label);
                    if inner.detached {
                        drop(inner);
                        req = ex.req.take().expect("request still present");
                        continue;
                    }
                    let ctx = crate::rt_core::ops::CommitCtx {
                        arm: inner.core.id(),
                        boot: crate::rt_core::broker::BootId::process(),
                        now: Instant::now(),
                        stop_generation: inner.current_stop_gen(),
                    };
                    handle_config_exchange(&mut inner, &ctx, &mut ex);
                    drop(inner);
                    return ex.reply.take().ok_or(ConfigError::OwnerStopped);
                }
                _ => return Err(ConfigError::OwnerStopped),
            }
        }
    }

    #[doc(hidden)]
    pub fn set_config_commit_test_hooks(
        &self,
        hooks: crate::rt_core::requests::config::ConfigCommitTestHooks,
    ) {
        *self
            .config_commit_hooks
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = hooks;
    }

    #[doc(hidden)]
    pub fn config_commit_test_hooks(
        &self,
    ) -> crate::rt_core::requests::config::ConfigCommitTestHooks {
        self.config_commit_hooks
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
    }

    /// Refreshes the legacy owner's off-lock [`MotionConfigSnapshot`] cached on `StateHub` after
    /// startup loaders so the first legacy WS motion command hits the cache without cloning
    /// `ArmConfig` inside `SupervisorLock::write()`. Never writes to the executor's `motion_config`.
    pub async fn refresh_motion_config_cache(&self) {
        if self.owner_mode() != OWNER_LEGACY {
            return;
        }
        let mc = {
            let inner = self.inner.read().await;
            if inner.detached {
                return;
            }
            Arc::new(crate::rt_core::requests::motion::MotionConfigSnapshot::from_supervisor(&inner))
        };
        self.hub.set_legacy_motion_config_if_newer(mc);
    }

    /// Returns the legacy owner's cached [`MotionConfigSnapshot`], performing a zero-alloc
    /// stack copy of updated joint limits and revisions off-lock if `motion_config_revision`
    /// advanced (for example after `commit_driver_limits_bounded`) so Lock #1 never deep-clones
    /// `ArmConfig`.
    pub(crate) async fn get_fresh_legacy_motion_config(
        &self,
    ) -> Option<Arc<crate::rt_core::requests::motion::MotionConfigSnapshot>> {
        let cur = self.hub.legacy_motion_config()?;
        let pub_revs = self
            .hub
            .latest()
            .map(|s| (s.rt.workspace_revision, s.rt.motion_config_revision));
        let Some((ws_rev, mc_rev)) = pub_revs else {
            return Some(cur);
        };
        if cur.workspace_revision == ws_rev && cur.motion_config_revision == mc_rev {
            return Some(cur);
        }
        if cur.workspace_revision == ws_rev {
            let stack_patch = {
                let inner = self.inner.read().await;
                if inner.detached || inner.workspace_revision != ws_rev {
                    None
                } else {
                    let dof = inner.config.limits.len().min(crate::model::MAX_MOTION_DOF);
                    let mut lims = [(0.0f32, 0.0f32, 0.0f32); crate::model::MAX_MOTION_DOF];
                    for (slot, l) in lims.iter_mut().zip(inner.config.limits.iter().take(dof)) {
                        *slot = (l.min_position, l.max_position, l.max_velocity);
                    }
                    Some((
                        dof,
                        lims,
                        inner.motion_config_revision,
                        inner.driver.is_some(),
                        inner.driver_caps,
                        inner.config.active_tool.commissioning_required,
                    ))
                }
            };
            if let Some((dof, lims, live_mc_rev, driver_present, driver_caps, comm_req)) = stack_patch {
                let mut next = (*cur).clone();
                if next.config.limits.len() == dof {
                    for (l, &(mn, mx, vel)) in next.config.limits.iter_mut().zip(lims.iter().take(dof)) {
                        l.min_position = mn;
                        l.max_position = mx;
                        l.max_velocity = vel;
                    }
                }
                next.motion_config_revision = live_mc_rev;
                next.driver_present = driver_present;
                next.driver_caps = driver_caps;
                next.config.active_tool.commissioning_required = comm_req;
                let arc = Arc::new(next);
                self.hub.set_legacy_motion_config_if_newer(Arc::clone(&arc));
                return Some(arc);
            }
        }
        Some(cur)
    }

    pub async fn set_motion_plan_deadline(&self, deadline: Duration) {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            inner.set_motion_plan_deadline(deadline);
        }
    }
}
