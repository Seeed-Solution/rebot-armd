//! Policy start of the WS command path: the pending start and the loading guard (track D).
//!
//! Moved from `api/ws.rs` without changes (armd 1.6 RT-core foundation); moved items, their
//! fields and inherent methods are `pub(super)` so the connection loop keeps using them.

#[allow(unused_imports)]
use super::*;

#[cfg(feature = "policy-runtime")]
pub(super) struct PendingPolicyStart {
    pub(super) request: crate::policy_runtime::PolicyStartRequest,
    pub(super) config: crate::model::ArmConfig,
    pub(super) epoch: u64,
    pub(super) policy_live_allowed: bool,
}

#[cfg(feature = "policy-runtime")]
pub(super) struct PolicyLoadingGuard(Option<Arc<ArmSupervisor>>);

#[cfg(feature = "policy-runtime")]
impl PolicyLoadingGuard {
    pub(super) fn new(sup: Arc<ArmSupervisor>) -> Self {
        Self(Some(sup))
    }

    pub(super) fn clear(&mut self, inner: &mut SupervisorInner) {
        if self.0.take().is_some() {
            inner.policy_loading = false;
        }
    }
}

#[cfg(feature = "policy-runtime")]
impl Drop for PolicyLoadingGuard {
    fn drop(&mut self) {
        let Some(sup) = self.0.take() else {
            return;
        };
        if let Ok(mut inner) = sup.inner().try_write() {
            inner.policy_loading = false;
            return;
        }
        match tokio::runtime::Handle::try_current() {
            Ok(h) => {
                h.spawn(async move {
                    sup.inner().write().await.policy_loading = false;
                });
            }
            Err(_) => sup.inner().blocking_write().policy_loading = false,
        }
        tracing::warn!("policy loading ended without explicit clear; cleared policy_loading");
    }
}
