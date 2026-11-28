//! Published configuration snapshot and read-side accessors for `ArmSupervisor`.

#[allow(unused_imports)]
use super::super::*;

impl ArmSupervisor {
    pub fn config_write_lock(&self) -> Arc<tokio::sync::Mutex<()>> {
        Arc::clone(&self.config_write_mutex)
    }

    pub fn published_config_hub(&self) -> &Arc<crate::rt_core::requests::config::PublishedConfigHub> {
        &self.published_config
    }

    /// Filters `PublishedConfig::sequences.pins` on `OWNER_EXECUTOR` against the live published
    /// `StateHub` action state so stale sequence pins are never exposed once the owning action has
    /// reached a terminal state (even before async `cleanup()` clears `pins`).
    fn filter_executor_published_config(
        &self,
        cfg: Arc<crate::rt_core::requests::config::PublishedConfig>,
    ) -> Arc<crate::rt_core::requests::config::PublishedConfig> {
        if cfg.sequences.pins.is_none() {
            return cfg;
        }
        let running_action_id = self
            .hub()
            .latest()
            .and_then(|s| s.obs.action.as_ref().filter(|a| a.is_running()).map(|a| a.id));
        if cfg.sequences.active_pins(running_action_id).is_some() {
            return cfg;
        }
        let mut filtered = (*cfg).clone();
        filtered.sequences.pins = None;
        Arc::new(filtered)
    }

    /// Returns the latest published configuration snapshot without touching `SupervisorLock`
    /// in `OWNER_EXECUTOR` mode. Fails closed with `503 CONFIG_UNAVAILABLE` if ownership is
    /// transitioning or failed.
    pub async fn published_config(
        &self,
    ) -> Result<Arc<crate::rt_core::requests::config::PublishedConfig>, ArmError> {
        match self.owner_mode() {
            OWNER_EXECUTOR => {
                let Some(handle) = self.executor_handle() else {
                    return Err(crate::rt_core::requests::config::config_unavailable());
                };
                if handle.is_exited() {
                    return Err(crate::rt_core::requests::config::config_unavailable());
                }
                let cfg = self
                    .published_config
                    .latest()
                    .ok_or_else(crate::rt_core::requests::config::config_unavailable)?;
                Ok(self.filter_executor_published_config(cfg))
            }
            OWNER_LEGACY => {
                let current_gen = self.inner.generation();
                let (cached, cached_gen) = self.published_config.latest_with_gen();
                if cached_gen == current_gen {
                    return Ok(cached);
                }
                let inner = self.inner.read().await;
                if inner.detached {
                    drop(inner);
                    if self.owner_mode() == OWNER_EXECUTOR {
                        let Some(handle) = self.executor_handle() else {
                            return Err(crate::rt_core::requests::config::config_unavailable());
                        };
                        if handle.is_exited() {
                            return Err(crate::rt_core::requests::config::config_unavailable());
                        }
                        let cfg = self
                            .published_config
                            .latest()
                            .ok_or_else(crate::rt_core::requests::config::config_unavailable)?;
                        return Ok(self.filter_executor_published_config(cfg));
                    }
                    return Err(crate::rt_core::requests::config::config_unavailable());
                }
                let gen = self.inner.generation();
                let snap = crate::rt_core::requests::config::PublishedConfig::from_inner(&inner);
                drop(inner);
                Ok(self.published_config.publish_if_newer(snap, gen))
            }
            _ => Err(crate::rt_core::requests::config::config_unavailable()),
        }
    }
}
