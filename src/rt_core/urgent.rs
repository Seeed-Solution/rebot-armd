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

/// Requested vs applied E-stop generations of one arm, and optionally its authorized-stop
/// generations ([`Self::with_stops`]): the output gate.
#[derive(Debug, Clone)]
pub struct EstopGate {
    requested: Arc<AtomicU64>,
    applied: Arc<AtomicU64>,
    stops: Option<StopRequests>,
}

impl EstopGate {
    pub fn new(requested: Arc<AtomicU64>, applied: Arc<AtomicU64>) -> Self { Self { requested, applied, stops: None } }

    /// The gate also holds output while an authorized stop is requested and not applied
    /// ([`StopRequests::pending`]): a stop that arrives after the release's last check
    /// still stops the remaining frames.
    pub fn with_stops(mut self, stops: StopRequests) -> Self {
        self.stops = Some(stops);
        self
    }

    /// An E-stop (or, with [`Self::with_stops`], an authorized stop) was requested and the
    /// tick has not applied it yet.
    #[inline]
    pub fn pending(&self) -> bool {
        self.requested.load(Ordering::SeqCst) != self.applied.load(Ordering::SeqCst)
            || self.stops.as_ref().is_some_and(StopRequests::pending)
    }

    /// Monotonic `(estop_requested, stop_requested)` generations observed through this gate
    /// (lock and allocation free). Both counters remain advanced after a stop is applied
    /// (`requested == applied`), so deferred frames stamped at admission are invalidated.
    #[inline]
    pub fn generations(&self) -> (u64, u64) {
        (
            self.requested.load(Ordering::SeqCst),
            self.stops.as_ref().map_or(0, StopRequests::current),
        )
    }
}

/// Authorized-stop requests (design section 4): a stop increments the generation directly,
/// without the command ring, so it is never refused for capacity. Commands carry the
/// generation they were admitted under ([`super::broker::Authority::stop_generation`]); the
/// RT refuses older ones (`STOPPED`) and ends the active motion of an older generation.
///
/// The RT records the generation it applied ([`Self::mark_applied`]); until then the stop is
/// [`Self::pending`] and an output gate built [`EstopGate::with_stops`] holds output.
#[derive(Debug, Clone, Default)]
pub struct StopRequests {
    generation: Arc<AtomicU64>,
    applied: Arc<AtomicU64>,
}

impl StopRequests {
    pub fn new() -> Self { Self::default() }

    /// Requests a stop; returns its generation (lock and allocation free).
    pub fn request(&self) -> u64 { self.generation.fetch_add(1, Ordering::SeqCst) + 1 }

    /// The current generation.
    #[inline]
    pub fn current(&self) -> u64 { self.generation.load(Ordering::SeqCst) }

    /// A stop was requested after a command admitted at `admitted`.
    #[inline]
    pub fn stopped_since(&self, admitted: u64) -> bool { self.current() != admitted }

    /// The RT applied every stop up to `generation` (it stopped motion after reading it).
    pub fn mark_applied(&self, generation: u64) { self.applied.fetch_max(generation, Ordering::SeqCst); }

    /// The last generation the RT applied.
    #[inline]
    pub fn applied(&self) -> u64 { self.applied.load(Ordering::SeqCst) }

    /// A stop was requested and the RT has not applied it yet.
    #[inline]
    pub fn pending(&self) -> bool { self.current() != self.applied() }
}

#[cfg(test)]
mod tests {
    #[test]
    fn stop_requests_invalidate_older_generations() {
        let s = super::StopRequests::new();
        let admitted = s.current();
        assert!(!s.stopped_since(admitted));
        assert_eq!(s.clone().request(), 1);
        assert!(s.stopped_since(admitted));
        assert!(!s.stopped_since(s.current()));
    }

    use super::*;

    #[test]
    fn the_output_gate_observes_ordinary_stops_until_applied() {
        let (req, app) = (Arc::new(AtomicU64::new(0)), Arc::new(AtomicU64::new(0)));
        let stops = StopRequests::new();
        let plain = EstopGate::new(req.clone(), app.clone());
        let gate = EstopGate::new(req, app).with_stops(stops.clone());
        assert!(!gate.pending());
        let g = stops.request();
        assert!(gate.pending() && stops.pending());
        assert!(!plain.pending(), "a gate without stops sees E-stops only");
        stops.mark_applied(g);
        assert!(!gate.pending());
        stops.mark_applied(g - 1);
        assert_eq!(stops.applied(), g, "applied never goes back");
    }

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
