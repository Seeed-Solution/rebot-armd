//! Urgent stop requests (design section 4): an E-stop is published as an atomic request
//! generation before any lock is taken (`ArmSupervisor::trigger_e_stop`); the control tick
//! applies it (`SupervisorInner::consume_pending_estop`) and records the applied generation.
//!
//! [`EstopGate`] is the check between the two: while a request is not yet applied, no
//! dynamic command may reach the bus. The tick checks it before its output, and the bus
//! driver before each frame of a batch, so an E-stop that arrives during a transmit stops
//! the remaining frames of that batch. Allocation and lock free (two atomic loads).

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

/// Requested vs applied E-stop generations of one arm.
#[derive(Debug, Clone)]
pub struct EstopGate {
    requested: Arc<AtomicU64>,
    applied: Arc<AtomicU64>,
}

impl EstopGate {
    pub fn new(requested: Arc<AtomicU64>, applied: Arc<AtomicU64>) -> Self { Self { requested, applied } }

    /// An E-stop was requested and the tick has not applied it yet.
    #[inline]
    pub fn pending(&self) -> bool {
        self.requested.load(Ordering::SeqCst) != self.applied.load(Ordering::SeqCst)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pending_until_applied() {
        let (req, app) = (Arc::new(AtomicU64::new(0)), Arc::new(AtomicU64::new(0)));
        let gate = EstopGate::new(req.clone(), app.clone());
        assert!(!gate.pending());
        req.fetch_add(1, Ordering::SeqCst);
        assert!(gate.pending());
        app.store(1, Ordering::SeqCst);
        assert!(!gate.pending());
    }
}
