//! Urgent stop requests (design section 4): an E-stop is published as an atomic request
//! generation before any lock is taken (`ArmSupervisor::trigger_e_stop`); the control tick
//! applies it (`SupervisorInner::consume_pending_estop`) and records the applied generation.
//!
//! [`EstopGate`] is the check between the two: while a request is not yet applied, no
//! dynamic command may reach the bus. The tick checks it before its output, and the bus
//! driver before each frame of a batch, so an E-stop that arrives during a transmit stops
//! the remaining frames of that batch. Allocation and lock free (two atomic loads).

use std::collections::{HashMap, VecDeque};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::time::Duration;

use rtrb::{Consumer, Producer, RingBuffer};

/// Bounded capacity of the dedicated urgent-stop request/reply rings, independent of
/// ordinary `Broker` command/plan/reply admission and `MaintClient` capacity.
pub const URGENT_STOP_CAPACITY: usize = 64;

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

/// Bounded urgent stop request handed off from WS (or caller) to the single RT owner.
/// Pure inline `Copy` fields: zero heap allocation on producer or RT consumer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UrgentStop {
    pub connection: crate::model::HolderLabel,
    pub lease_generation: u64,
    pub epoch: Option<u64>,
    pub request_id: u64,
    pub request_id_valid: bool,
    pub body_error: Option<&'static str>,
}

/// Errors returned by [`UrgentStopClient`] when the urgent stop handoff cannot accept or
/// complete a request.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum UrgentStopError {
    #[error("urgent stop queue full")]
    QueueFull,
    #[error("RT executor stopped")]
    Stopped,
}

impl From<UrgentStopError> for crate::supervisor::ArmError {
    fn from(e: UrgentStopError) -> Self {
        let code = match e {
            UrgentStopError::QueueFull => "URGENT_STOP_BUSY",
            UrgentStopError::Stopped => "EXECUTOR_STOPPED",
        };
        crate::supervisor::ArmError::Rejected {
            status: 503,
            code: code.into(),
            message: e.to_string(),
        }
    }
}

/// Outcome computed by the single owner for one [`UrgentStop`] request.
#[derive(Debug, Clone, PartialEq)]
pub enum UrgentStopOutcome {
    /// Authorized stop: `StopRequests` advanced and applied at `stop_generation`.
    Applied {
        stop_generation: u64,
        /// `true` when client motion was stopped; `false` when an active watchdog park was kept.
        applied_to_motion: bool,
        epoch: u64,
        safety_state: Arc<crate::safety::SafetyState>,
        is_torqued: bool,
        control_mode_gravity_comp: bool,
    },
    /// Connection / generation / expiry check failed at the owner; `StopRequests` untouched.
    LeaseLost {
        epoch: u64,
    },
    /// Supplied epoch did not match owner's current safety epoch; `StopRequests` untouched.
    EpochMismatch {
        epoch: u64,
    },
    /// `request_id` was missing or did not strictly increase; `StopRequests` untouched.
    InvalidRequestId,
    /// Command body validation failed (consumes `request_id`); `StopRequests` untouched.
    InvalidBody {
        message: &'static str,
        epoch: u64,
    },
    /// Executor stopped or panicked before publishing this stop.
    ExecutorStopped,
}

/// Heap-owning motion payloads retired during an RT urgent stop and dropped strictly
/// off-thread by [`UrgentStopClient`] (zero RT frees).
#[derive(Default)]
pub struct RetiredStopPayloads {
    pub trajectory: Option<crate::motion::ActiveTrajectory>,
    pub unload_settle: Option<crate::supervisor::UnloadSettle>,
    pub servo_ws_min_depths: Option<(u64, crate::kinematics::SlotBuf)>,
}

/// One queued urgent stop descriptor on the SPSC request ring (`UrgentStopClient` -> RT).
#[derive(Debug, Clone, Copy)]
pub struct UrgentStopRequest {
    pub id: u64,
    pub stop: UrgentStop,
}

/// One reply on the SPSC reply ring (RT -> `UrgentStopClient`).
pub struct UrgentStopReply {
    pub id: u64,
    pub outcome: UrgentStopOutcome,
    /// Preserves an `Applied` reply's `Arc<SafetyState>` if a panic converts the reply to
    /// `ExecutorStopped` before publication, avoiding any `Arc` drop on `armd-control`.
    pub returned_state: Option<Arc<crate::safety::SafetyState>>,
    pub retired: RetiredStopPayloads,
}

/// RT-side intake for bounded urgent stop requests (owned exclusively by `ReleaseEngine` /
/// `armd-control`). Preallocates its pending reply deque at construction so runtime urgent
/// stop handling performs zero heap allocations and zero heap frees.
pub struct UrgentStopIntake {
    req_rx: Consumer<UrgentStopRequest>,
    rep_tx: Producer<UrgentStopReply>,
    pending: VecDeque<UrgentStopReply>,
    capacity: usize,
}

impl UrgentStopIntake {
    /// Drains at most `capacity` queued urgent stop requests (bounded per-checkpoint workload)
    /// into the preallocated `pending` deque. Returns `true` when at least one request was
    /// authorized and applied.
    pub fn drain_requests<F>(&mut self, mut handle: F) -> bool
    where
        F: FnMut(&UrgentStop) -> (UrgentStopOutcome, RetiredStopPayloads),
    {
        let mut drained = 0usize;
        let mut applied_any = false;
        while drained < self.capacity && self.pending.len() < self.capacity {
            let Ok(req) = self.req_rx.pop() else {
                break;
            };
            drained += 1;
            let (outcome, retired) = handle(&req.stop);
            if matches!(outcome, UrgentStopOutcome::Applied { .. }) {
                applied_any = true;
            }
            self.pending.push_back(UrgentStopReply {
                id: req.id,
                outcome,
                returned_state: None,
                retired,
            });
        }
        applied_any
    }

    pub fn has_pending_replies(&self) -> bool {
        !self.pending.is_empty()
    }

    pub fn has_applied_pending(&self) -> bool {
        self.pending
            .iter()
            .any(|r| matches!(r.outcome, UrgentStopOutcome::Applied { .. }))
    }

    /// Flushes completed replies into `rep_tx` strictly after publication. Returns `true` when
    /// at least one reply was pushed.
    pub fn flush_replies(&mut self) -> bool {
        let mut pushed = false;
        while !self.rep_tx.is_full() {
            let Some(reply) = self.pending.pop_front() else {
                break;
            };
            let _ = self.rep_tx.push(reply);
            pushed = true;
        }
        pushed
    }

    /// Exit/panic drain on `armd-control`:
    /// - `flush_existing == true` (clean exit after publication): flushes already-computed
    ///   pending replies as-is.
    /// - `flush_existing == false` (panic exit): converts unpublished pending replies into
    ///   `ExecutorStopped` while moving any `Arc<SafetyState>` into `returned_state` and
    ///   keeping `retired` payloads intact so nothing drops on `armd-control`.
    /// Then converts any remaining unconsumed requests on `req_rx` into `ExecutorStopped`.
    pub fn drain_stopped(&mut self, flush_existing: bool) -> bool {
        let mut pushed = false;
        if flush_existing {
            pushed |= self.flush_replies();
        } else {
            while !self.rep_tx.is_full() {
                let Some(mut reply) = self.pending.pop_front() else {
                    break;
                };
                if let UrgentStopOutcome::Applied { safety_state, .. } = reply.outcome {
                    reply.returned_state = Some(safety_state);
                }
                reply.outcome = UrgentStopOutcome::ExecutorStopped;
                let _ = self.rep_tx.push(reply);
                pushed = true;
            }
        }
        while !self.rep_tx.is_full() {
            let Ok(req) = self.req_rx.pop() else {
                break;
            };
            let _ = self.rep_tx.push(UrgentStopReply {
                id: req.id,
                outcome: UrgentStopOutcome::ExecutorStopped,
                returned_state: None,
                retired: RetiredStopPayloads::default(),
            });
            pushed = true;
        }
        pushed
    }
}

enum UrgentStopWaiter {
    Sync(mpsc::SyncSender<Result<UrgentStopOutcome, UrgentStopError>>),
    Async(tokio::sync::oneshot::Sender<Result<UrgentStopOutcome, UrgentStopError>>),
}

impl UrgentStopWaiter {
    fn deliver(self, r: Result<UrgentStopOutcome, UrgentStopError>) {
        match self {
            Self::Sync(tx) => {
                let _ = tx.send(r);
            }
            Self::Async(tx) => {
                let _ = tx.send(r);
            }
        }
    }
}

struct UrgentStopClientInner {
    req_tx: Producer<UrgentStopRequest>,
    rep_rx: Consumer<UrgentStopReply>,
    capacity: usize,
    in_flight: usize,
    next_id: u64,
    closed: bool,
    waiters: HashMap<u64, UrgentStopWaiter>,
    exited: Arc<AtomicBool>,
}

impl UrgentStopClientInner {
    fn pop_all_replies(&mut self) {
        while let Ok(reply) = self.rep_rx.pop() {
            self.in_flight = self.in_flight.saturating_sub(1);
            // Explicitly drop retired RT payloads and returned state Arcs off-thread.
            drop(reply.returned_state);
            drop(reply.retired);
            if let Some(waiter) = self.waiters.remove(&reply.id) {
                let res = match reply.outcome {
                    UrgentStopOutcome::ExecutorStopped => Err(UrgentStopError::Stopped),
                    other => Ok(other),
                };
                waiter.deliver(res);
            }
        }
    }

    fn drain_replies_with_hook(&mut self, mut between_pop_and_exit_check: impl FnMut()) {
        self.pop_all_replies();
        between_pop_and_exit_check();
        if self.exited.load(Ordering::Acquire) {
            // Double-drain after observing `exited` (Release -> Acquire) so a final reply
            // pushed immediately before thread exit is never lost to `Stopped`.
            self.pop_all_replies();
            self.fail_remaining(UrgentStopError::Stopped);
        }
    }

    fn drain_replies(&mut self) {
        self.drain_replies_with_hook(|| {});
    }

    fn fail_remaining(&mut self, err: UrgentStopError) {
        let remaining = std::mem::take(&mut self.waiters);
        for (_, w) in remaining {
            w.deliver(Err(err.clone()));
        }
    }
}

/// Off-thread lock-serialized client for the dedicated bounded urgent-stop channel.
/// Reserves one reply credit before enqueueing and retains it until off-thread reply drain
/// even if the calling waiter disconnects, preserving strict SPSC producer semantics and
/// bounded RT reply storage.
pub struct UrgentStopClient {
    inner: Mutex<UrgentStopClientInner>,
}

impl UrgentStopClient {
    pub fn drain_replies(&self) {
        self.inner.lock().unwrap_or_else(|e| e.into_inner()).drain_replies();
    }

    pub fn in_flight(&self) -> usize {
        self.inner.lock().unwrap_or_else(|e| e.into_inner()).in_flight
    }

    pub fn close(&self) {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        g.closed = true;
        g.drain_replies();
    }

    pub fn finish_stopped(&self) {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        g.closed = true;
        g.drain_replies();
        g.fail_remaining(UrgentStopError::Stopped);
    }

    /// Reserves reply credit and pushes one urgent stop non-blockingly, returning the
    /// oneshot receiver so a WS receive loop can `select!` on it without blocking E-stop intake.
    pub fn enqueue_async(
        &self,
        stop: UrgentStop,
    ) -> Result<tokio::sync::oneshot::Receiver<Result<UrgentStopOutcome, UrgentStopError>>, UrgentStopError> {
        let (tx, rx) = tokio::sync::oneshot::channel();
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        g.drain_replies();
        if g.closed || g.exited.load(Ordering::Acquire) || g.req_tx.is_abandoned() {
            return Err(UrgentStopError::Stopped);
        }
        if g.in_flight >= g.capacity || g.req_tx.is_full() {
            return Err(UrgentStopError::QueueFull);
        }
        let id = g.next_id;
        g.next_id += 1;
        if g.req_tx.push(UrgentStopRequest { id, stop }).is_err() {
            return Err(UrgentStopError::QueueFull);
        }
        g.in_flight += 1;
        g.waiters.insert(id, UrgentStopWaiter::Async(tx));
        Ok(rx)
    }

    /// Awaits a previously enqueued urgent-stop ticket while periodically draining replies.
    pub async fn await_ticket(
        &self,
        mut rx: tokio::sync::oneshot::Receiver<Result<UrgentStopOutcome, UrgentStopError>>,
    ) -> Result<UrgentStopOutcome, UrgentStopError> {
        loop {
            match tokio::time::timeout(Duration::from_millis(2), &mut rx).await {
                Ok(Ok(res)) => return res,
                Ok(Err(_)) => return Err(UrgentStopError::Stopped),
                Err(_) => self.drain_replies(),
            }
        }
    }

    pub async fn submit_async(&self, stop: UrgentStop) -> Result<UrgentStopOutcome, UrgentStopError> {
        let rx = self.enqueue_async(stop)?;
        self.await_ticket(rx).await
    }

    pub fn submit_sync(&self, stop: UrgentStop) -> Result<UrgentStopOutcome, UrgentStopError> {
        let (tx, rx) = mpsc::sync_channel(1);
        {
            let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
            g.drain_replies();
            if g.closed || g.exited.load(Ordering::Acquire) || g.req_tx.is_abandoned() {
                return Err(UrgentStopError::Stopped);
            }
            if g.in_flight >= g.capacity || g.req_tx.is_full() {
                return Err(UrgentStopError::QueueFull);
            }
            let id = g.next_id;
            g.next_id += 1;
            if g.req_tx.push(UrgentStopRequest { id, stop }).is_err() {
                return Err(UrgentStopError::QueueFull);
            }
            g.in_flight += 1;
            g.waiters.insert(id, UrgentStopWaiter::Sync(tx));
        }
        loop {
            match rx.recv_timeout(Duration::from_millis(2)) {
                Ok(res) => return res,
                Err(mpsc::RecvTimeoutError::Timeout) => self.drain_replies(),
                Err(mpsc::RecvTimeoutError::Disconnected) => return Err(UrgentStopError::Stopped),
            }
        }
    }
}

/// Constructs a paired bounded urgent-stop client and RT intake with matching capacities.
pub fn urgent_stop_channel(capacity: usize, exited: Arc<AtomicBool>) -> (UrgentStopClient, UrgentStopIntake) {
    let cap = capacity.max(1);
    let (req_tx, req_rx) = RingBuffer::<UrgentStopRequest>::new(cap);
    let (rep_tx, rep_rx) = RingBuffer::<UrgentStopReply>::new(cap);
    let client = UrgentStopClient {
        inner: Mutex::new(UrgentStopClientInner {
            req_tx,
            rep_rx,
            capacity: cap,
            in_flight: 0,
            next_id: 1,
            closed: false,
            waiters: HashMap::new(),
            exited,
        }),
    };
    let intake = UrgentStopIntake {
        req_rx,
        rep_tx,
        pending: VecDeque::with_capacity(cap),
        capacity: cap,
    };
    (client, intake)
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

    #[test]
    fn urgent_stop_credit_reservation_survives_disconnected_waiter_and_saturates_explicitly() {
        let exited = Arc::new(AtomicBool::new(false));
        let (client, mut intake) = urgent_stop_channel(2, exited);
        let stop = UrgentStop {
            connection: crate::model::HolderLabel::new("connection:1"),
            lease_generation: 1,
            epoch: Some(1),
            request_id: 1,
            request_id_valid: true,
            body_error: None,
        };
        let rx1 = client.enqueue_async(stop).unwrap();
        let rx2 = client.enqueue_async(UrgentStop { request_id: 2, ..stop }).unwrap();
        assert_eq!(client.in_flight(), 2);
        // Dropping rx1 before drain must NOT release its reply credit!
        drop(rx1);
        assert_eq!(
            client.enqueue_async(UrgentStop { request_id: 3, ..stop }).unwrap_err(),
            UrgentStopError::QueueFull
        );
        assert_eq!(client.in_flight(), 2);

        let applied = intake.drain_requests(|_| {
            (UrgentStopOutcome::LeaseLost { epoch: 1 }, RetiredStopPayloads::default())
        });
        assert!(!applied);
        assert!(intake.flush_replies());
        client.drain_replies();
        assert_eq!(client.in_flight(), 0);
        assert_eq!(rx2.blocking_recv().unwrap().unwrap(), UrgentStopOutcome::LeaseLost { epoch: 1 });
        // Capacity is fully reclaimed after off-thread drain.
        let _rx3 = client.enqueue_async(UrgentStop { request_id: 3, ..stop }).unwrap();
        assert_eq!(client.in_flight(), 1);
    }

    #[test]
    fn urgent_stop_client_drains_final_ack_after_observing_exited() {
        let exited = Arc::new(AtomicBool::new(false));
        let (client, mut intake) = urgent_stop_channel(4, exited.clone());
        let stop = UrgentStop {
            connection: crate::model::HolderLabel::new("connection:1"),
            lease_generation: 1,
            epoch: Some(5),
            request_id: 1,
            request_id_valid: true,
            body_error: None,
        };
        let mut rx = client.enqueue_async(stop).unwrap();
        assert!(intake.drain_requests(|_| (
            UrgentStopOutcome::Applied {
                stop_generation: 1,
                applied_to_motion: true,
                epoch: 5,
                safety_state: Arc::new(crate::safety::SafetyState::Armed),
                is_torqued: true,
                control_mode_gravity_comp: false,
            },
            RetiredStopPayloads::default(),
        )));

        // Simulate the exact race: client observes empty reply queue, then RT flushes final
        // Applied reply and stores `exited = true` before client checks `exited`.
        let mut g = client.inner.lock().unwrap();
        g.drain_replies_with_hook(|| {
            assert!(intake.flush_replies());
            exited.store(true, Ordering::Release);
        });
        drop(g);

        let delivered = rx
            .try_recv()
            .expect("final urgent stop ACK must be delivered, not lost to exited")
            .expect("must not be converted to Stopped");
        assert!(matches!(
            delivered,
            UrgentStopOutcome::Applied { stop_generation: 1, epoch: 5, .. }
        ));
        assert_eq!(client.in_flight(), 0);
    }

    #[test]
    fn urgent_stop_panic_drain_preserves_state_arc_off_rt_and_reports_stopped() {
        let exited = Arc::new(AtomicBool::new(false));
        let (client, mut intake) = urgent_stop_channel(4, exited.clone());
        let stop = UrgentStop {
            connection: crate::model::HolderLabel::new("connection:1"),
            lease_generation: 1,
            epoch: Some(5),
            request_id: 1,
            request_id_valid: true,
            body_error: None,
        };
        let mut rx1 = client.enqueue_async(stop).unwrap();
        let mut rx2 = client.enqueue_async(UrgentStop { request_id: 2, ..stop }).unwrap();
        let state_arc = Arc::new(crate::safety::SafetyState::Fault(
            crate::safety::FaultReason::Other("heap fault payload".into()),
        ));
        let weak = Arc::downgrade(&state_arc);
        // Drain only req 1 into pending (simulate panic between apply and publish, with req 2 still queued).
        let req1 = intake.req_rx.pop().unwrap();
        intake.pending.push_back(UrgentStopReply {
            id: req1.id,
            outcome: UrgentStopOutcome::Applied {
                stop_generation: 1,
                applied_to_motion: true,
                epoch: 5,
                safety_state: state_arc,
                is_torqued: true,
                control_mode_gravity_comp: false,
            },
            returned_state: None,
            retired: RetiredStopPayloads::default(),
        });
        assert!(intake.drain_stopped(false));
        // Arc must STILL be alive (moved into reply ring, NOT dropped on RT)!
        assert!(weak.upgrade().is_some(), "Arc<SafetyState> must survive panic drain for off-thread drop");

        exited.store(true, Ordering::Release);
        client.drain_replies();
        assert!(weak.upgrade().is_none(), "Arc<SafetyState> must be dropped off-thread by drain_replies");
        assert_eq!(rx1.try_recv().unwrap(), Err(UrgentStopError::Stopped));
        assert_eq!(rx2.try_recv().unwrap(), Err(UrgentStopError::Stopped));
        assert_eq!(client.in_flight(), 0);
    }
}
