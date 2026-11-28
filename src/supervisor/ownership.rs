//! Ownership mode transitions, executor handoff/handback, `Drop`, and process identity.

#[allow(unused_imports)]
use super::*;

// scaffold-impl-0
impl Drop for TransferGuard<'_> {
}

// scaffold-impl-1
#[cfg(not(target_arch = "wasm32"))]
impl Drop for SupervisorInner {
}

// scaffold-impl-2
impl Drop for ArmSupervisor {
}

// scaffold-extra-0
impl ArmSupervisor {
}
