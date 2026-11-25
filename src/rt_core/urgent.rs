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

/// Single lock-free atomic state packing the monotonic E-stop request generation (upper 63
/// bits) and the E-stop latch flag (bit 0) into ONE `AtomicU64`.
///
/// Layout: `(generation << 1) | (latched as u64)`
///
/// Because generation and latch live in the same 64-bit word:
/// - Triggering an E-stop ([`Self::request_and_latch`]) atomically increments `generation`
///   and sets bit 0 in one CAS loop (zero gap between counter bump and latch).
/// - Clearing a prior E-stop ([`Self::try_clear_if_generation`]) clears bit 0 via a single
///   `compare_exchange` only when `generation == expected_gen`. If any concurrent E-stop
///   has already incremented `generation`, the CAS fails and bit 0 is NEVER cleared (zero
///   transient-false window).
#[derive(Debug, Default)]
pub struct EstopState {
    word: AtomicU64,
}

impl EstopState {
    const LATCH_BIT: u64 = 1;

    #[inline]
    pub const fn new() -> Self {
        Self { word: AtomicU64::new(0) }
    }

    #[inline]
    pub const fn pack(generation: u64, latched: bool) -> u64 {
        (generation << 1) | (latched as u64)
    }

    #[inline]
    pub const fn unpack_gen(word: u64) -> u64 {
        word >> 1
    }

    #[inline]
    pub const fn unpack_latched(word: u64) -> bool {
        (word & Self::LATCH_BIT) != 0
    }

    /// Atomically increments the request generation AND sets the latch bit in one step.
    /// Returns the new generation (`>= 1`). Lock-free and allocation-free.
    #[inline]
    pub fn request_and_latch(&self) -> u64 {
        let prev = self
            .word
            .fetch_update(Ordering::SeqCst, Ordering::SeqCst, |w| {
                let next_gen = (w >> 1).checked_add(1)?;
                Some((next_gen << 1) | Self::LATCH_BIT)
            })
            .expect("E-stop generation overflow (2^63)");
        (prev >> 1) + 1
    }

    /// Atomically sets the latch bit without advancing the request generation (for owner-side
    /// internal latches such as shutdown, panic, or `StopPolicy::EStopLatch`).
    #[inline]
    pub fn latch(&self) {
        self.word.fetch_or(Self::LATCH_BIT, Ordering::SeqCst);
    }

    /// Atomically returns `(generation, latched)` from a single load.
    #[inline]
    pub fn snapshot(&self) -> (u64, bool) {
        let w = self.word.load(Ordering::SeqCst);
        (Self::unpack_gen(w), Self::unpack_latched(w))
    }

    #[inline]
    pub fn is_latched(&self) -> bool {
        Self::unpack_latched(self.word.load(Ordering::SeqCst))
    }

    #[inline]
    pub fn is_latched_with(&self, order: Ordering) -> bool {
        Self::unpack_latched(self.word.load(order))
    }

    #[inline]
    pub fn requested_count(&self) -> u64 {
        Self::unpack_gen(self.word.load(Ordering::SeqCst))
    }

    #[inline]
    pub fn requested_count_with(&self, order: Ordering) -> u64 {
        Self::unpack_gen(self.word.load(order))
    }

    /// Atomically clears the latch bit ONLY if `generation == expected_gen`.
    ///
    /// Returns `Ok(())` when the latch bit is confirmed `false` at `expected_gen`, or
    /// `Err(observed_gen)` when a newer E-stop has already advanced `generation` — leaving
    /// the latch bit untouched (never transiently `false`).
    #[inline]
    pub fn try_clear_if_generation(&self, expected_gen: u64) -> Result<(), u64> {
        let latched_word = Self::pack(expected_gen, true);
        let cleared_word = Self::pack(expected_gen, false);
        match self.word.compare_exchange(
            latched_word,
            cleared_word,
            Ordering::SeqCst,
            Ordering::SeqCst,
        ) {
            Ok(_) => Ok(()),
            Err(obs) if obs == cleared_word => Ok(()),
            Err(obs) => Err(Self::unpack_gen(obs)),
        }
    }

    /// Advances the upper 63-bit generation by `delta` while preserving the latch bit.
    /// Returns the previous generation.
    #[inline]
    pub fn fetch_add_generation(&self, delta: u64, order: Ordering) -> u64 {
        let prev = self.word.fetch_add(delta << 1, order);
        Self::unpack_gen(prev)
    }

    /// Replaces the upper 63-bit generation while preserving the latch bit (test helper).
    #[inline]
    pub fn store_generation_preserving_latch(&self, gen: u64, order: Ordering) {
        let _ = self.word.fetch_update(order, Ordering::SeqCst, |w| {
            Some((gen << 1) | (w & Self::LATCH_BIT))
        });
    }

    /// Sets or clears bit 0 while preserving generation (used by [`EstopLatchView::store`]
    /// in legacy tests; production code clears ONLY via [`Self::try_clear_if_generation`]).
    #[inline]
    pub fn store_latched_raw(&self, val: bool, order: Ordering) {
        if val {
            self.word.fetch_or(Self::LATCH_BIT, order);
        } else {
            self.word.fetch_and(!Self::LATCH_BIT, order);
        }
    }
}

/// Zero-overhead view of the latch bit inside the arm's single packed [`EstopState`].
#[derive(Debug, Clone)]
#[repr(transparent)]
pub struct EstopLatchView(Arc<EstopState>);

impl EstopLatchView {
    #[inline]
    pub fn new(state: Arc<EstopState>) -> Self { Self(state) }

    #[inline]
    pub fn load(&self, order: Ordering) -> bool { self.0.is_latched_with(order) }

    #[inline]
    pub fn store(&self, val: bool, order: Ordering) { self.0.store_latched_raw(val, order); }

    #[inline]
    pub fn state(&self) -> &Arc<EstopState> { &self.0 }
}

/// Zero-overhead view of the monotonic request counter inside the arm's single packed [`EstopState`].
#[derive(Debug, Clone)]
#[repr(transparent)]
pub struct EstopCounterView(Arc<EstopState>);

impl EstopCounterView {
    #[inline]
    pub fn new(state: Arc<EstopState>) -> Self { Self(state) }

    #[inline]
    pub fn load(&self, order: Ordering) -> u64 { self.0.requested_count_with(order) }

    #[inline]
    pub fn fetch_add(&self, val: u64, order: Ordering) -> u64 { self.0.fetch_add_generation(val, order) }

    #[inline]
    pub fn store(&self, val: u64, order: Ordering) { self.0.store_generation_preserving_latch(val, order); }

    #[inline]
    pub fn state(&self) -> &Arc<EstopState> { &self.0 }
}

#[derive(Debug, Clone)]
enum EstopRequestedSource {
    /// Production packed `(generation << 1) | latched` atomic word.
    Packed(Arc<EstopState>),
    /// Standalone counter for isolated driver/gate unit tests.
    Raw(Arc<AtomicU64>),
}

impl EstopRequestedSource {
    #[inline]
    fn generation(&self) -> u64 {
        match self {
            Self::Packed(state) => state.requested_count(),
            Self::Raw(counter) => counter.load(Ordering::SeqCst),
        }
    }

    #[inline]
    fn is_latched(&self) -> bool {
        match self {
            Self::Packed(state) => state.is_latched(),
            Self::Raw(_) => false,
        }
    }
}

/// Requested vs applied E-stop generations of one arm, and optionally its authorized-stop
/// generations ([`Self::with_stops`]): the output gate.
#[derive(Debug, Clone)]
/// Lock-free view of the arm's live lease generation and fence-first disconnect atomics,
/// carried on [`EstopGate`] so the driver's single TX choke point (`DamiaoArmDriver::send`)
/// and the single J7 emitter drop stale client-lease motion frames immediately after a
/// disconnect, release, preemption, or expiry without acquiring any lock.
pub struct LeaseGate {
    current_gen: Arc<AtomicU64>,
    fenced_gen: Arc<AtomicU64>,
    pending_disconnect_hold: Arc<AtomicU64>,
}

impl LeaseGate {
    pub fn new(
        current_gen: Arc<AtomicU64>,
        fenced_gen: Arc<AtomicU64>,
        pending_disconnect_hold: Arc<AtomicU64>,
    ) -> Self {
        Self { current_gen, fenced_gen, pending_disconnect_hold }
    }

    /// Current unfenced live lease generation (`0` when no lease, fenced, or disconnect pending).
    #[inline]
    pub fn current_lease_gen(&self) -> u64 {
        if self.pending_disconnect_hold.load(Ordering::SeqCst) > 0 {
            return 0;
        }
        let cur = self.current_gen.load(Ordering::SeqCst);
        let fenced = self.fenced_gen.load(Ordering::SeqCst);
        if cur > 0 && cur > fenced { cur } else { 0 }
    }

    /// True when a motion frame produced under `entry_lease_gen` (`> 0`) has been fenced by
    /// disconnect, release, preemption, or expiry. `entry_lease_gen == 0` (stop/hold, autonomous
    /// watchdog park/unload, or lease-disabled) is never fenced.
    #[inline]
    pub fn is_fenced(&self, entry_lease_gen: u64) -> bool {
        if entry_lease_gen == 0 {
            return false;
        }
        if self.pending_disconnect_hold.load(Ordering::SeqCst) > 0 {
            return true;
        }
        let fenced = self.fenced_gen.load(Ordering::SeqCst);
        if entry_lease_gen <= fenced {
            return true;
        }
        self.current_gen.load(Ordering::SeqCst) != entry_lease_gen
    }
}

#[derive(Debug, Clone)]
pub struct EstopGate {
    requested: EstopRequestedSource,
    applied: Arc<AtomicU64>,
    stops: Option<StopRequests>,
    owner_stop_gen: Option<Arc<AtomicU64>>,
    lease: Option<LeaseGate>,
}

impl EstopGate {
    pub fn new(requested: Arc<AtomicU64>, applied: Arc<AtomicU64>) -> Self {
        Self {
            requested: EstopRequestedSource::Raw(requested),
            applied,
            stops: None,
            owner_stop_gen: None,
            lease: None,
        }
    }

    /// Builds a gate reading directly from the arm's single packed [`EstopState`].
    pub fn from_state(state: Arc<EstopState>, applied: Arc<AtomicU64>) -> Self {
        Self {
            requested: EstopRequestedSource::Packed(state),
            applied,
            stops: None,
            owner_stop_gen: None,
            lease: None,
        }
    }

    /// The gate also holds output while an authorized stop is requested and not applied
    /// ([`StopRequests::pending`]): a stop that arrives after the release's last check
    /// still stops the remaining frames.
    pub fn with_stops(mut self, stops: StopRequests) -> Self {
        self.stops = Some(stops);
        self
    }

    /// Attaches the owner's monotonic `stop_gen` atomic so legacy/non-executor Stop requests
    /// are visible through [`Self::generations`] even when [`StopRequests`] is not attached.
    pub fn with_owner_stop_gen(mut self, stop_gen: Arc<AtomicU64>) -> Self {
        self.owner_stop_gen = Some(stop_gen);
        self
    }

    /// Attaches the arm's lock-free lease fence atomics so the driver TX choke point drops
    /// stale client-lease motion frames immediately after disconnect, release, preemption, or expiry.
    pub fn with_lease_gate(mut self, lease: LeaseGate) -> Self {
        self.lease = Some(lease);
        self
    }

    /// An E-stop (or, with [`Self::with_stops`], an authorized stop) was requested and the
    /// tick has not applied it yet.
    #[inline]
    pub fn pending(&self) -> bool {
        self.requested.generation() != self.applied.load(Ordering::SeqCst)
            || self.stops.as_ref().is_some_and(StopRequests::pending)
    }

    /// Whether the underlying packed [`EstopState`] currently has its latch bit set.
    #[inline]
    pub fn is_latched(&self) -> bool {
        self.requested.is_latched()
    }

    /// Monotonic `(estop_requested, stop_requested)` generations observed through this gate
    /// (lock and allocation free). Both counters remain advanced after a stop is applied
    /// (`requested == applied`), so deferred frames stamped at admission are invalidated.
    #[inline]
    pub fn generations(&self) -> (u64, u64) {
        let req_stop = self.stops.as_ref().map_or(0, StopRequests::current);
        let own_stop = self
            .owner_stop_gen
            .as_ref()
            .map_or(0, |a| a.load(Ordering::SeqCst));
        (self.requested.generation(), req_stop.max(own_stop))
    }

    /// Current live unfenced lease generation (`0` when no lease gate is attached, no active
    /// lease exists, or a disconnect fence is active).
    #[inline]
    pub fn current_lease_gen(&self) -> u64 {
        self.lease.as_ref().map_or(0, LeaseGate::current_lease_gen)
    }

    /// True when a motion frame produced under `entry_lease_gen` (`> 0`) has been fenced.
    #[inline]
    pub fn is_lease_fenced(&self, entry_lease_gen: u64) -> bool {
        self.lease.as_ref().is_some_and(|l| l.is_fenced(entry_lease_gen))
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
        F: FnMut(&UrgentStop) -> UrgentStopOutcome,
    {
        let mut drained = 0usize;
        let mut applied_any = false;
        while drained < self.capacity && self.pending.len() < self.capacity {
            let Ok(req) = self.req_rx.pop() else { break; };
            drained += 1;
            let outcome = handle(&req.stop);
            if matches!(outcome, UrgentStopOutcome::Applied { .. }) { applied_any = true; }
            self.pending.push_back(UrgentStopReply { id: req.id, outcome, returned_state: None });
        }
        applied_any
    }

    pub fn has_pending_replies(&self) -> bool { !self.pending.is_empty() }

    pub fn has_applied_pending(&self) -> bool {
        self.pending.iter().any(|r| matches!(r.outcome, UrgentStopOutcome::Applied { .. }))
    }

    /// Flushes completed replies into `rep_tx` strictly after publication.
    pub fn flush_replies(&mut self) -> bool {
        let mut pushed = false;
        while !self.rep_tx.is_full() {
            let Some(reply) = self.pending.pop_front() else { break; };
            let _ = self.rep_tx.push(reply);
            pushed = true;
        }
        pushed
    }

    /// Exit/panic drain on `armd-control`:
    /// - `flush_existing == true` (clean exit after publication): flushes pending replies as-is.
    /// - `flush_existing == false` (panic exit): converts unpublished pending replies into
    ///   `ExecutorStopped`, moving any `Arc<SafetyState>` into `returned_state` so nothing drops
    ///   on `armd-control`.
    ///
    /// Then converts any remaining unconsumed requests on `req_rx` into `ExecutorStopped`.
    pub fn drain_stopped(&mut self, flush_existing: bool) -> bool {
        let mut pushed = false;
        if flush_existing {
            pushed |= self.flush_replies();
        } else {
            while !self.rep_tx.is_full() {
                let Some(mut reply) = self.pending.pop_front() else { break; };
                if let UrgentStopOutcome::Applied { safety_state, .. } = reply.outcome {
                    reply.returned_state = Some(safety_state);
                }
                reply.outcome = UrgentStopOutcome::ExecutorStopped;
                let _ = self.rep_tx.push(reply);
                pushed = true;
            }
        }
        while !self.rep_tx.is_full() {
            let Ok(req) = self.req_rx.pop() else { break; };
            let _ = self.rep_tx.push(UrgentStopReply {
                id: req.id,
                outcome: UrgentStopOutcome::ExecutorStopped,
                returned_state: None,
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
            drop(reply.returned_state);
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
        for (_, w) in remaining { w.deliver(Err(err.clone())); }
    }

    fn enqueue(&mut self, stop: UrgentStop, waiter: UrgentStopWaiter) -> Result<(), UrgentStopError> {
        self.drain_replies();
        if self.closed || self.exited.load(Ordering::Acquire) || self.req_tx.is_abandoned() {
            return Err(UrgentStopError::Stopped);
        }
        if self.in_flight >= self.capacity || self.req_tx.is_full() {
            return Err(UrgentStopError::QueueFull);
        }
        let id = self.next_id;
        self.next_id += 1;
        if self.req_tx.push(UrgentStopRequest { id, stop }).is_err() {
            return Err(UrgentStopError::QueueFull);
        }
        self.in_flight += 1;
        self.waiters.insert(id, waiter);
        Ok(())
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
    pub fn drain_replies(&self) { self.inner.lock().unwrap_or_else(|e| e.into_inner()).drain_replies(); }
    pub fn in_flight(&self) -> usize { self.inner.lock().unwrap_or_else(|e| e.into_inner()).in_flight }

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
        self.inner.lock().unwrap_or_else(|e| e.into_inner()).enqueue(stop, UrgentStopWaiter::Async(tx))?;
        Ok(rx)
    }

    /// Awaits a previously enqueued urgent-stop ticket while periodically draining replies.
    pub async fn await_ticket(
        &self,
        rx: &mut tokio::sync::oneshot::Receiver<Result<UrgentStopOutcome, UrgentStopError>>,
    ) -> Result<UrgentStopOutcome, UrgentStopError> {
        loop {
            match tokio::time::timeout(Duration::from_millis(2), &mut *rx).await {
                Ok(Ok(res)) => return res,
                Ok(Err(_)) => return Err(UrgentStopError::Stopped),
                Err(_) => self.drain_replies(),
            }
        }
    }

    pub async fn submit_async(&self, stop: UrgentStop) -> Result<UrgentStopOutcome, UrgentStopError> {
        let mut rx = self.enqueue_async(stop)?;
        self.await_ticket(&mut rx).await
    }

    pub fn submit_sync(&self, stop: UrgentStop) -> Result<UrgentStopOutcome, UrgentStopError> {
        let (tx, rx) = mpsc::sync_channel(1);
        self.inner.lock().unwrap_or_else(|e| e.into_inner()).enqueue(stop, UrgentStopWaiter::Sync(tx))?;
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
            req_tx, rep_rx, capacity: cap, in_flight: 0, next_id: 1, closed: false, waiters: HashMap::new(), exited,
        }),
    };
    let intake = UrgentStopIntake { req_rx, rep_tx, pending: VecDeque::with_capacity(cap), capacity: cap };
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

    fn sample_stop(request_id: u64, epoch: u64) -> UrgentStop {
        UrgentStop {
            connection: crate::model::HolderLabel::new("connection:1"),
            lease_generation: 1,
            epoch: Some(epoch),
            request_id,
            request_id_valid: true,
            body_error: None,
        }
    }

    #[test]
    fn urgent_stop_credit_reservation_survives_disconnected_waiter_and_saturates_explicitly() {
        let (client, mut intake) = urgent_stop_channel(2, Arc::new(AtomicBool::new(false)));
        let rx1 = client.enqueue_async(sample_stop(1, 1)).unwrap();
        let rx2 = client.enqueue_async(sample_stop(2, 1)).unwrap();
        assert_eq!(client.in_flight(), 2);
        drop(rx1);
        assert_eq!(client.enqueue_async(sample_stop(3, 1)).unwrap_err(), UrgentStopError::QueueFull);
        assert_eq!(client.in_flight(), 2);

        assert!(!intake.drain_requests(|_| UrgentStopOutcome::LeaseLost { epoch: 1 }));
        assert!(intake.flush_replies());
        client.drain_replies();
        assert_eq!(client.in_flight(), 0);
        assert_eq!(rx2.blocking_recv().unwrap().unwrap(), UrgentStopOutcome::LeaseLost { epoch: 1 });
        let _rx3 = client.enqueue_async(sample_stop(3, 1)).unwrap();
        assert_eq!(client.in_flight(), 1);
    }

    #[test]
    fn urgent_stop_client_drains_final_ack_after_observing_exited() {
        let exited = Arc::new(AtomicBool::new(false));
        let (client, mut intake) = urgent_stop_channel(4, exited.clone());
        let mut rx = client.enqueue_async(sample_stop(1, 5)).unwrap();
        assert!(intake.drain_requests(|_| UrgentStopOutcome::Applied {
            stop_generation: 1, applied_to_motion: true, epoch: 5,
            safety_state: Arc::new(crate::safety::SafetyState::Armed),
            is_torqued: true, control_mode_gravity_comp: false,
        }));
        let mut g = client.inner.lock().unwrap();
        g.drain_replies_with_hook(|| {
            assert!(intake.flush_replies());
            exited.store(true, Ordering::Release);
        });
        drop(g);
        let delivered = rx.try_recv().expect("final ACK lost to exited").expect("converted to Stopped");
        assert!(matches!(delivered, UrgentStopOutcome::Applied { stop_generation: 1, epoch: 5, .. }));
        assert_eq!(client.in_flight(), 0);
    }

    #[test]
    fn urgent_stop_panic_drain_preserves_state_arc_off_rt_and_reports_stopped() {
        let exited = Arc::new(AtomicBool::new(false));
        let (client, mut intake) = urgent_stop_channel(4, exited.clone());
        let mut rx1 = client.enqueue_async(sample_stop(1, 5)).unwrap();
        let mut rx2 = client.enqueue_async(sample_stop(2, 5)).unwrap();
        let state_arc = Arc::new(crate::safety::SafetyState::Fault(crate::safety::FaultReason::Other("heap".into())));
        let weak = Arc::downgrade(&state_arc);
        let req1 = intake.req_rx.pop().unwrap();
        intake.pending.push_back(UrgentStopReply {
            id: req1.id,
            outcome: UrgentStopOutcome::Applied {
                stop_generation: 1, applied_to_motion: true, epoch: 5,
                safety_state: state_arc, is_torqued: true, control_mode_gravity_comp: false,
            },
            returned_state: None,
        });
        assert!(intake.drain_stopped(false));
        assert!(weak.upgrade().is_some(), "Arc<SafetyState> must survive panic drain for off-thread drop");
        exited.store(true, Ordering::Release);
        client.drain_replies();
        assert!(weak.upgrade().is_none(), "Arc<SafetyState> must drop off-thread in drain_replies");
        assert_eq!(rx1.try_recv().unwrap(), Err(UrgentStopError::Stopped));
        assert_eq!(rx2.try_recv().unwrap(), Err(UrgentStopError::Stopped));
        assert_eq!(client.in_flight(), 0);
    }
}
