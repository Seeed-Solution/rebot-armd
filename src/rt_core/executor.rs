//! Synchronous RT executor owning one arm's [`ArmCore`] (`SupervisorInner`) and bus endpoint
//! (armd 1.6 RT stage 3, step 1).
//!
//! The executor runs on its own OS thread (`armd-control`) without a Tokio runtime handle:
//! it sleeps until absolute `CLOCK_MONOTONIC` releases (`clock_nanosleep` with `TIMER_ABSTIME`
//! on Linux), skips missed releases while preserving real elapsed `dt`, and executes each
//! release in exact order through [`super::ops::ReleaseEngine`]. Diagnostics, two-phase
//! gripper-calibration persistence, E-stop waiter resolution, and payload reclamation run off
//! the executor thread.

use std::any::Any;
use std::cell::Cell;
use std::collections::{BTreeMap, HashMap};
use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{mpsc, Arc, Condvar, Mutex};
use std::thread::{self, JoinHandle, Thread, ThreadId};
use std::time::{Duration, Instant};

use rtrb::{Consumer, Producer, RingBuffer};

use super::broker::{Authority, BootId, Broker, Capacities, ClientId, RequestId, Stamp};
use super::config_iface::{Activated, CandidateId, ConfigAuthority, ConfigDomain, ConfigProposal, Durability, Reservation, Reserved};
use super::ids::ArmId;
use super::mailbox::TargetWriter;
use super::ops::{CoreOps, PlanSlot, PlanningSnapshot, ReleaseEngine, MAX_SERVICE_WAKES};
use super::publish::StateHub;
use super::sched::{self, MemoryLock, RtReadiness, RtSchedConfig};
use super::urgent::StopRequests;
use crate::diagnostics::tick::{
    LockPhases, ThreadUsage, TickSample, UsageMark, RELEASE_FLAG_CLEAR_OK,
    RELEASE_FLAG_CLEAR_REFUSED, RELEASE_FLAG_ESTOP_APPLIED, RELEASE_FLAG_INJECT_ALLOC,
    RELEASE_FLAG_SAFETY_LATCH, RELEASE_FLAG_THERMAL_TRIP, RELEASE_FLAG_WATCHDOG_LATCH,
};
use crate::model::{ArmObservation, GripperPolicy, JawOpening};
use crate::persist::{PublishOutcome, StagedWrite};
use crate::safety::{
    ClearFaultError, FaultKind, SafetyError, SafetyLogEvent, SafetyState, FAULT_ARC_POOL_CAPACITY,
    FORMATTED_FAULT_ARC_POOL_CAPACITY, JOINT_LIMIT_ARC_POOL_CAPACITY, SAFETY_LOG_CAPACITY,
};
use crate::supervisor::{ArmError, PendingZeroCheckPersist, SupervisorInner};

/// Elapsed time above which a tick counts as late in the log (as in 1.5).
pub const LATE_LOG_AFTER: Duration = Duration::from_millis(80);

/// Capacity of the off-thread diagnostics ring (`TickSample`).
pub const DIAG_RING_CAPACITY: usize = 256;
/// Capacity of the pre-reserved 1-to-1 maintenance request/reply rings.
pub const MAINT_QUEUE_CAPACITY: usize = 16;
/// Capacity of the E-stop acknowledgement ring (`armd-control` -> `EstopRegistry`).
pub const ESTOP_ACK_CAPACITY: usize = 64;
/// Bounded capacity of the off-thread fault-state `Arc<SafetyState>` reclamation ring.
pub const RECLAIM_STATE_CAPACITY: usize = 64;
/// Capacity of the persistence reply ring (`armd-control` -> `armd-persist` / `OffThreadSinks`).
/// Strictly carries 1-to-1 `PersistCmd` replies (never unsolicited job reclamation).
pub const PERSIST_REP_CAPACITY: usize = 8;
/// Lock-free SPSC capacity for off-thread reclamation (runtime rejected persistence jobs,
/// deferred `legacy_j6` warnings, and clean-exit overflow payloads when consumer rings stay full).
/// Sized strictly >= `17 maint + 6 persist + 12 runtime jobs/logs + 64 pending_estop_acks + 64 retired`.
const OFFLOAD_RECLAIM_CAPACITY: usize =
    MAINT_QUEUE_CAPACITY + 1 + 6 + 12 + ESTOP_ACK_CAPACITY + RECLAIM_STATE_CAPACITY;

/// Startup owner selection for `ArmCore` and the actuator bus.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RtOwner {
    /// Legacy supervisor-lock owner (default in stage 3 step 1).
    #[default]
    Legacy,
    /// Synchronous RT executor OS thread owning `SupervisorInner` and the bus driver.
    Executor,
}

impl RtOwner {
    pub fn from_env() -> Result<Self, String> {
        Self::from_var(std::env::var("ARMD_RT_OWNER").ok().as_deref())
    }

    pub fn from_var(raw: Option<&str>) -> Result<Self, String> {
        match raw.map(str::trim).filter(|s| !s.is_empty()) {
            None | Some("legacy") => Ok(Self::Legacy),
            Some("executor") => Ok(Self::Executor),
            Some(other) => Err(format!("ARMD_RT_OWNER must be 'legacy' or 'executor', got {other:?}")),
        }
    }
}

/// Step-1 startup gate: CLI `ARMD_RT_OWNER=executor` is positively allowed only for
/// driverless `"mock"` until step 2 routes WS commands through the broker.
pub fn validate_startup_owner(owner: RtOwner, follower: &str) -> Result<(), String> {
    match owner {
        RtOwner::Legacy => Ok(()),
        RtOwner::Executor if follower == "mock" => Ok(()),
        RtOwner::Executor => Err(format!(
            "ARMD_RT_OWNER=executor is gated to '--follower mock' in stage 3 step 1 (got '--follower {follower}'); use ARMD_RT_OWNER=legacy for sim/hardware until step 2 broker routing"
        )),
    }
}

thread_local! {
    static IN_RT_RELEASE: Cell<bool> = const { Cell::new(false) };
    static ON_RT_CONTROL_THREAD: Cell<bool> = const { Cell::new(false) };
}

/// True while the current thread is inside an RT release or sub-release wake.
pub fn in_rt_release() -> bool {
    IN_RT_RELEASE.with(Cell::get)
}

/// True while the current thread is marked as the RT executor control thread
/// (`armd-control`), from thread entry to exit and independent of [`ReleaseGuard`].
pub fn on_rt_control_thread() -> bool {
    ON_RT_CONTROL_THREAD.with(Cell::get)
}

/// Panics if invoked while inside an RT release (asserted on Tokio / legacy lock entry paths).
pub fn assert_not_in_rt_release(site: &'static str) {
    assert!(
        !in_rt_release(),
        "forbidden Tokio or legacy supervisor lock entry during RT release at {site}"
    );
}

/// Panics if invoked on the RT executor control thread (asserted on legacy supervisor lock
/// entry and async poll paths, independent of [`ReleaseGuard`]).
pub fn assert_not_on_rt_control_thread(site: &'static str) {
    assert!(
        !on_rt_control_thread(),
        "forbidden legacy supervisor lock entry on RT control thread at {site}"
    );
}

/// RAII thread-local guard active during every RT executor release and sub-release wake.
pub struct ReleaseGuard(bool);

impl ReleaseGuard {
    pub fn enter() -> Self {
        Self(IN_RT_RELEASE.with(|flag| flag.replace(true)))
    }
}

impl Drop for ReleaseGuard {
    fn drop(&mut self) {
        IN_RT_RELEASE.with(|flag| flag.set(self.0));
    }
}

/// RAII thread-local guard marking the RT executor control thread (`armd-control`) from
/// thread entry to exit, independent of [`ReleaseGuard`].
///
/// Intentionally `!Send` and `!Sync` so the guard is always dropped on the thread it marked.
pub struct ControlThreadGuard {
    prev: bool,
    _not_send: std::marker::PhantomData<*mut ()>,
}

impl ControlThreadGuard {
    pub fn enter() -> Self {
        Self {
            prev: ON_RT_CONTROL_THREAD.with(|flag| flag.replace(true)),
            _not_send: std::marker::PhantomData,
        }
    }
}

impl Drop for ControlThreadGuard {
    fn drop(&mut self) {
        ON_RT_CONTROL_THREAD.with(|flag| flag.set(self.prev));
    }
}

/// What the executor thread does if a release panics.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PanicPolicy {
    /// Latch E-stop, flush logs, and terminate the daemon (production `armd` binary).
    AbortProcess,
    /// Latch E-stop, drain pending maintenance/E-stop waiters, and exit the thread cleanly (tests).
    StopExecutor,
}

/// Configuration for starting [`RtExecutor`].
#[derive(Debug, Clone)]
pub struct ExecutorConfig {
    pub period: Duration,
    pub boot: BootId,
    pub capacities: Capacities,
    pub rt_sched: RtSchedConfig,
    pub memory_lock: MemoryLock,
    pub alloc_timing: bool,
    pub panic_policy: PanicPolicy,
    pub apply_rt_sched: bool,
}

impl ExecutorConfig {
    /// Deterministic test defaults: normal OS scheduling, no process exit on panic.
    pub fn for_tests(period: Duration) -> Self {
        Self {
            period,
            boot: BootId(1),
            capacities: Capacities::default(),
            rt_sched: RtSchedConfig::default(),
            memory_lock: MemoryLock::Skipped("test".into()),
            alloc_timing: false,
            panic_policy: PanicPolicy::StopExecutor,
            apply_rt_sched: false,
        }
    }
}

/// Cumulative per-operation heap counters recorded on `armd-control` outside the droppable
/// diagnostic ring (zero unless `alloc_timing` is enabled with [`crate::diagnostics::alloc::TimedAlloc`]).
#[derive(Debug, Default)]
pub struct AtomicAllocCounters {
    pub alloc: AtomicU64,
    pub alloc_zeroed: AtomicU64,
    pub realloc: AtomicU64,
    pub dealloc: AtomicU64,
}

impl AtomicAllocCounters {
    #[inline]
    pub fn record(&self, delta: crate::diagnostics::alloc::AllocCounts) {
        if delta.alloc > 0 {
            self.alloc.fetch_add(delta.alloc, Ordering::AcqRel);
        }
        if delta.alloc_zeroed > 0 {
            self.alloc_zeroed.fetch_add(delta.alloc_zeroed, Ordering::AcqRel);
        }
        if delta.realloc > 0 {
            self.realloc.fetch_add(delta.realloc, Ordering::AcqRel);
        }
        if delta.dealloc > 0 {
            self.dealloc.fetch_add(delta.dealloc, Ordering::AcqRel);
        }
    }

    pub fn snapshot(&self) -> crate::diagnostics::alloc::AllocCounts {
        crate::diagnostics::alloc::AllocCounts {
            alloc: self.alloc.load(Ordering::Acquire),
            alloc_zeroed: self.alloc_zeroed.load(Ordering::Acquire),
            realloc: self.realloc.load(Ordering::Acquire),
            dealloc: self.dealloc.load(Ordering::Acquire),
        }
    }
}

/// Counters the executor thread updates with plain atomic stores.
#[derive(Debug, Default)]
pub struct ExecutorStats {
    /// Steps started.
    pub steps: AtomicU64,
    /// Whole releases completed (Release-incremented strictly AFTER `release_allocs` is updated).
    pub completed_releases: AtomicU64,
    /// Steps whose elapsed time exceeded [`LATE_LOG_AFTER`].
    pub late: AtomicU64,
    /// Elapsed time of the latest late step, microseconds.
    pub last_late_us: AtomicU64,
    /// Releases skipped because a step ended after them.
    pub skipped_releases: AtomicU64,
    /// Diagnostic `TickSample` values dropped because the off-thread ring was full.
    pub diag_dropped: AtomicU64,
    /// Cumulative releases whose raw wakeup lateness exceeded 1 ms / 5 ms / 10 ms.
    pub wake_lateness_over_1ms: AtomicU64,
    pub wake_lateness_over_5ms: AtomicU64,
    pub wake_lateness_over_10ms: AtomicU64,
    /// Monotonic count of deferred safety log events dropped due to queue overflow (never reset).
    pub safety_log_dropped: AtomicU64,
    /// Sub-release transition wakes started by `armd-control`.
    pub wakes: AtomicU64,
    /// Sub-release transition wakes completed (Release-incremented strictly AFTER `wake_allocs` is updated).
    pub completed_wakes: AtomicU64,
    /// Allocator calls observed across full sub-release wake bodies (when `alloc_timing` is enabled).
    pub wake_alloc_calls: AtomicU64,
    /// Cumulative per-op heap counts across whole releases (entry through diagnostic enqueue,
    /// publication, maintenance, reply handling, and local destruction; disjoint from wakes).
    pub release_allocs: AtomicAllocCounters,
    /// Cumulative per-op heap counts across all sub-release wakes (entry through wake service,
    /// publication, reply/ack handling, and local destruction; disjoint from releases).
    pub wake_allocs: AtomicAllocCounters,
    /// Release sequence (`TickSample::release_seq`) of the most recent fault latch on `armd-control`.
    pub last_fault_latch_release_seq: AtomicU64,
    /// Release sequence (`TickSample::release_seq`) of the most recent E-stop applied on `armd-control`.
    pub last_estop_release_seq: AtomicU64,
    /// Release sequence (`TickSample::release_seq`) of the most recent `ClearFault` serviced on `armd-control`.
    pub last_clear_release_seq: AtomicU64,
    /// Release sequence (`TickSample::release_seq`) of the most recent injected test allocation on `armd-control`.
    pub last_inject_alloc_release_seq: AtomicU64,
}

impl ExecutorStats {
    /// Classifies raw wakeup lateness (`actual_release_start - scheduled_release`, ns), updates
    /// the cumulative `> 1 / 5 / 10 ms` counters atomically (zero allocation), and returns the
    /// saturating microsecond value for `TickSample::wake_lateness_us`.
    #[inline]
    pub fn record_wake_lateness_ns(&self, lateness_ns: u64) -> u32 {
        let (us, over_1ms, over_5ms, over_10ms) =
            crate::diagnostics::tick::classify_wake_lateness_ns(lateness_ns);
        if over_1ms {
            self.wake_lateness_over_1ms.fetch_add(1, Ordering::Relaxed);
        }
        if over_5ms {
            self.wake_lateness_over_5ms.fetch_add(1, Ordering::Relaxed);
        }
        if over_10ms {
            self.wake_lateness_over_10ms.fetch_add(1, Ordering::Relaxed);
        }
        us
    }

    /// Combined cumulative heap counts across both whole releases and sub-release wakes
    /// (independent of the droppable diagnostic ring).
    pub fn total_allocs(&self) -> crate::diagnostics::alloc::AllocCounts {
        self.release_allocs.snapshot().saturating_add(self.wake_allocs.snapshot())
    }

    /// A normal-priority thread that logs late steps, at most once per second.
    pub fn spawn_monitor(self: &Arc<Self>, stop: Arc<AtomicBool>) -> std::io::Result<JoinHandle<()>> {
        let stats = self.clone();
        thread::Builder::new().name("armd-rt-monitor".into()).spawn(move || {
            let mut seen = stats.late.load(Ordering::Acquire);
            while !stop.load(Ordering::Acquire) {
                thread::park_timeout(Duration::from_secs(1));
                let late = stats.late.load(Ordering::Acquire);
                if late != seen {
                    let ms = stats.last_late_us.load(Ordering::Relaxed) as f64 / 1000.0;
                    tracing::warn!(
                        "Control tick late: {ms:.0} ms since previous tick ({} late ticks in the last second)",
                        late - seen
                    );
                    seen = late;
                }
            }
        })
    }
}

/// Errors returned by owner-routed maintenance and shutdown operations.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum OwnerError {
    #[error("maintenance queue full")]
    QueueFull,
    #[error("RT executor stopped")]
    Stopped,
    #[error("shutdown has not been confirmed by the RT owner")]
    ShutdownNotConfirmed,
    #[error("unexpected maintenance reply")]
    Protocol,
}

impl From<OwnerError> for ArmError {
    fn from(e: OwnerError) -> Self {
        let code = match e {
            OwnerError::QueueFull => "MAINTENANCE_BUSY",
            OwnerError::Stopped => "EXECUTOR_STOPPED",
            OwnerError::ShutdownNotConfirmed => "SHUTDOWN_NOT_CONFIRMED",
            OwnerError::Protocol => "INTERNAL",
        };
        ArmError::Rejected { status: 503, code: code.into(), message: e.to_string() }
    }
}

/// Bounded maintenance and shutdown operations routed through the single RT owner.
pub enum MaintenanceOp {
    RequestShutdown,
    CommitShutdown,
    ShutdownReady,
    PlanningSnapshot,
    Observation,
    ClearFault { expected_epoch: Option<u64> },
    SetJointZeroOffsets { offsets: Vec<f32> },
    SetMotorZero { joint: usize },
    #[doc(hidden)]
    ProbeOwnedPayload { payload: Box<dyn Any + Send> },
}

/// Reply payload of one [`MaintenanceOp`].
#[allow(clippy::large_enum_variant)] // bounded stack refusal keeps RT ClearFault zero-alloc
pub enum MaintenanceResult {
    ShutdownRequested(bool),
    ShutdownCommitted(bool),
    ShutdownReady(bool),
    PlanningSnapshot(PlanningSnapshot),
    Observation(Box<ArmObservation>),
    FaultCleared(Result<SafetyState, ClearFaultError>),
    JointZeroOffsetsSet(Result<(), String>),
    MotorZeroSet(Result<(), String>),
    ProbeReturned,
    Stopped,
}

struct MaintRequest {
    id: u64,
    op: MaintenanceOp,
}

struct MaintReply {
    id: u64,
    result: MaintenanceResult,
    returned_offsets: Option<Vec<f32>>,
    returned_probe: Option<Box<dyn Any + Send>>,
    returned_driver_config_error: Option<String>,
}

enum MaintWaiter {
    Sync(mpsc::SyncSender<Result<MaintenanceResult, OwnerError>>),
    Async(tokio::sync::oneshot::Sender<Result<MaintenanceResult, OwnerError>>),
}

impl MaintWaiter {
    fn deliver(self, r: Result<MaintenanceResult, OwnerError>) {
        match self {
            Self::Sync(tx) => { let _ = tx.send(r); }
            Self::Async(tx) => { let _ = tx.send(r); }
        }
    }
}

struct MaintClientInner {
    req_tx: Producer<MaintRequest>,
    rep_rx: Consumer<MaintReply>,
    in_flight: usize,
    next_id: u64,
    closed: bool,
    waiters: HashMap<u64, MaintWaiter>,
    exited: Arc<AtomicBool>,
}

impl MaintClientInner {
    fn pop_all_replies(&mut self) {
        while let Ok(reply) = self.rep_rx.pop() {
            self.in_flight = self.in_flight.saturating_sub(1);
            // Explicitly drop returned payloads here on the off-RT caller/dispatcher thread.
            drop(reply.returned_offsets);
            drop(reply.returned_probe);
            if let Some(waiter) = self.waiters.remove(&reply.id) {
                let res = match reply.result {
                    MaintenanceResult::Stopped => Err(OwnerError::Stopped),
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
            // Drain once more after observing `exited` (Release -> Acquire): any reply pushed
            // immediately before thread exit (e.g. `CommitShutdown(true)`) is guaranteed visible
            // now and must not be lost to `fail_remaining`.
            self.pop_all_replies();
            self.fail_remaining(OwnerError::Stopped);
        }
    }

    fn drain_replies(&mut self) {
        self.drain_replies_with_hook(|| {});
    }

    fn fail_remaining(&mut self, err: OwnerError) {
        let remaining = std::mem::take(&mut self.waiters);
        for (_, w) in remaining {
            w.deliver(Err(err.clone()));
        }
    }
}

/// Off-thread client for the pre-reserved 1-to-1 maintenance channel.
pub(crate) struct MaintClient {
    inner: Mutex<MaintClientInner>,
}

impl MaintClient {
    fn new(req_tx: Producer<MaintRequest>, rep_rx: Consumer<MaintReply>, exited: Arc<AtomicBool>) -> Self {
        Self {
            inner: Mutex::new(MaintClientInner {
                req_tx,
                rep_rx,
                in_flight: 0,
                next_id: 1,
                closed: false,
                waiters: HashMap::new(),
                exited,
            }),
        }
    }

    pub(crate) fn drain_replies(&self) {
        self.inner.lock().unwrap_or_else(|e| e.into_inner()).drain_replies();
    }

    fn close(&self) {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        g.closed = true;
        g.drain_replies();
    }

    fn finish_stopped(&self) {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        g.closed = true;
        g.drain_replies();
        g.fail_remaining(OwnerError::Stopped);
    }

    fn call_sync(&self, op: MaintenanceOp) -> Result<MaintenanceResult, OwnerError> {
        let (tx, rx) = mpsc::sync_channel(1);
        {
            let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
            g.drain_replies();
            if g.closed || g.exited.load(Ordering::Acquire) || g.req_tx.is_abandoned() {
                drop(op);
                return Err(OwnerError::Stopped);
            }
            if g.in_flight >= MAINT_QUEUE_CAPACITY || g.req_tx.is_full() {
                drop(op);
                return Err(OwnerError::QueueFull);
            }
            let id = g.next_id;
            g.next_id += 1;
            if g.req_tx.push(MaintRequest { id, op }).is_err() {
                return Err(OwnerError::QueueFull);
            }
            g.in_flight += 1;
            g.waiters.insert(id, MaintWaiter::Sync(tx));
        }
        loop {
            match rx.recv_timeout(Duration::from_millis(2)) {
                Ok(res) => return res,
                Err(mpsc::RecvTimeoutError::Timeout) => {
                    self.drain_replies();
                }
                Err(mpsc::RecvTimeoutError::Disconnected) => return Err(OwnerError::Stopped),
            }
        }
    }

    async fn call_async(&self, op: MaintenanceOp) -> Result<MaintenanceResult, OwnerError> {
        let (tx, mut rx) = tokio::sync::oneshot::channel();
        {
            let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
            g.drain_replies();
            if g.closed || g.exited.load(Ordering::Acquire) || g.req_tx.is_abandoned() {
                drop(op);
                return Err(OwnerError::Stopped);
            }
            if g.in_flight >= MAINT_QUEUE_CAPACITY || g.req_tx.is_full() {
                drop(op);
                return Err(OwnerError::QueueFull);
            }
            let id = g.next_id;
            g.next_id += 1;
            if g.req_tx.push(MaintRequest { id, op }).is_err() {
                return Err(OwnerError::QueueFull);
            }
            g.in_flight += 1;
            g.waiters.insert(id, MaintWaiter::Async(tx));
        }
        loop {
            match tokio::time::timeout(Duration::from_millis(2), &mut rx).await {
                Ok(Ok(res)) => return res,
                Ok(Err(_)) => return Err(OwnerError::Stopped),
                Err(_) => self.drain_replies(),
            }
        }
    }
}

/// One contiguous E-stop application ack captured immutably at `consume_pending_estop()`.
#[derive(Debug, Clone)]
pub struct EstopAck {
    pub first_gen: u64,
    pub last_gen: u64,
    pub state: Arc<SafetyState>,
    pub epoch: u64,
}

type EstopResult = Result<(SafetyState, u64), ArmError>;

/// Off-thread lock-serialized E-stop waiter registry (never touched by `armd-control`).
#[derive(Default)]
pub(crate) struct EstopRegistry {
    waiters: BTreeMap<u64, Vec<tokio::sync::oneshot::Sender<EstopResult>>>,
    ack_rx: Option<Consumer<EstopAck>>,
    overflow_gen: Option<Arc<AtomicU64>>,
    exited: Option<Arc<AtomicBool>>,
}

impl EstopRegistry {
    pub(crate) fn register(&mut self, req: u64) -> tokio::sync::oneshot::Receiver<EstopResult> {
        let (tx, rx) = tokio::sync::oneshot::channel();
        self.waiters.entry(req).or_default().push(tx);
        rx
    }

    pub(crate) fn record_ack(&mut self, ack: EstopAck) {
        let matched: Vec<u64> = self
            .waiters
            .range(ack.first_gen..=ack.last_gen)
            .map(|(&g, _)| g)
            .collect();
        for g in matched {
            if let Some(senders) = self.waiters.remove(&g) {
                for tx in senders {
                    let _ = tx.send(Ok(((*ack.state).clone(), ack.epoch)));
                }
            }
        }
    }

    pub(crate) fn record_overflow(&mut self, max_dropped: u64) {
        if max_dropped == 0 {
            return;
        }
        let dropped: Vec<u64> = self.waiters.range(..=max_dropped).map(|(&g, _)| g).collect();
        for g in dropped {
            if let Some(senders) = self.waiters.remove(&g) {
                for tx in senders {
                    let _ = tx.send(Err(ArmError::Rejected {
                        status: 503,
                        code: "ESTOP_ACK_OVERFLOW".into(),
                        message: "E-stop acknowledgement ring overflowed; epoch cannot be pinned".into(),
                    }));
                }
            }
        }
    }

    pub(crate) fn cancel_waiter(&mut self, req: u64, status: u16, code: &'static str, message: &'static str) {
        if let Some(senders) = self.waiters.remove(&req) {
            for tx in senders {
                let _ = tx.send(Err(ArmError::Rejected {
                    status,
                    code: code.into(),
                    message: message.into(),
                }));
            }
        }
    }

    pub(crate) fn attach_executor(
        &mut self,
        ack_rx: Consumer<EstopAck>,
        overflow_gen: Arc<AtomicU64>,
        exited: Arc<AtomicBool>,
    ) {
        self.ack_rx = Some(ack_rx);
        self.overflow_gen = Some(overflow_gen);
        self.exited = Some(exited);
        self.drain();
    }

    fn pop_acks_and_overflow(&mut self) {
        let mut drained = Vec::new();
        if let Some(rx) = self.ack_rx.as_mut() {
            while let Ok(ack) = rx.pop() {
                drained.push(ack);
            }
        }
        for ack in drained {
            self.record_ack(ack);
        }
        if let Some(ov) = self.overflow_gen.as_ref() {
            let max_dropped = ov.load(Ordering::Acquire);
            self.record_overflow(max_dropped);
        }
    }

    fn drain_with_hook(&mut self, mut between_pop_and_exit_check: impl FnMut()) {
        self.pop_acks_and_overflow();
        between_pop_and_exit_check();
        if self.exited.as_ref().is_some_and(|e| e.load(Ordering::Acquire)) {
            // Drain once more after observing `exited` so final acks / overflow recorded before
            // thread exit are never lost to `EXECUTOR_STOPPED`.
            self.pop_acks_and_overflow();
            let remaining = std::mem::take(&mut self.waiters);
            for (_, senders) in remaining {
                for tx in senders {
                    let _ = tx.send(Err(ArmError::Rejected {
                        status: 503,
                        code: "EXECUTOR_STOPPED".into(),
                        message: "RT executor stopped before acknowledging E-stop".into(),
                    }));
                }
            }
        }
    }

    pub(crate) fn drain(&mut self) {
        self.drain_with_hook(|| {});
    }
}

/// Deterministic clock hook for [`RtExecutor`] tests (`PersistTestHooks::clock`).
/// Production runs leave `hooks.clock == None` and use real `Instant::now()`, `CLOCK_MONOTONIC`,
/// and `clock_nanosleep` with zero overhead.
#[doc(hidden)]
pub trait ExecutorClock: Send + Sync {
    fn now(&self) -> Instant;
    fn now_ns(&self) -> u64;
    fn sleep_until_ns(&self, target_ns: u64, stop: &AtomicBool);
    fn sleep(&self, dur: Duration, stop: &AtomicBool);
    fn wake(&self) {}
}

/// RAII owner of one raw heap allocation created for allocation-counter positive controls.
/// Freed on drop (typically off-thread, or at release/wake local destruction for final-free controls).
#[doc(hidden)]
pub struct ParkedAlloc {
    ptr: *mut u8,
    layout: std::alloc::Layout,
}

// SAFETY: uniquely owns the raw allocation until dropped or reallocated.
unsafe impl Send for ParkedAlloc {}

impl ParkedAlloc {
    /// Allocates `size` bytes (align 8) via `std::alloc::alloc` and writes a sentinel byte.
    pub fn new_alloc(size: usize) -> Self {
        let layout = std::alloc::Layout::from_size_align(size.max(8), 8).expect("valid layout");
        // SAFETY: layout has non-zero size.
        let ptr = unsafe { std::alloc::alloc(layout) };
        assert!(!ptr.is_null(), "allocation failed");
        // SAFETY: ptr is valid for `layout.size()` bytes.
        unsafe { *ptr = 0xA5; }
        Self { ptr, layout }
    }

    /// Allocates `size` zeroed bytes (align 8) via `std::alloc::alloc_zeroed`.
    pub fn new_alloc_zeroed(size: usize) -> Self {
        let layout = std::alloc::Layout::from_size_align(size.max(8), 8).expect("valid layout");
        // SAFETY: layout has non-zero size.
        let ptr = unsafe { std::alloc::alloc_zeroed(layout) };
        assert!(!ptr.is_null(), "zeroed allocation failed");
        Self { ptr, layout }
    }

    /// Reallocates this buffer in place via `std::alloc::realloc` without triggering `Drop`.
    pub fn realloc_to(self, new_size: usize) -> Self {
        let this = std::mem::ManuallyDrop::new(self);
        let new_size = new_size.max(8);
        let new_layout = std::alloc::Layout::from_size_align(new_size, this.layout.align()).expect("valid layout");
        // SAFETY: `this.ptr` was allocated with `this.layout`, `new_size > 0`, and `ManuallyDrop` prevents double-free.
        let ptr = unsafe { std::alloc::realloc(this.ptr, this.layout, new_size) };
        assert!(!ptr.is_null(), "reallocation failed");
        Self { ptr, layout: new_layout }
    }
}

impl Drop for ParkedAlloc {
    fn drop(&mut self) {
        // SAFETY: `self.ptr` was allocated with `self.layout` and is uniquely owned here.
        unsafe { std::alloc::dealloc(self.ptr, self.layout); }
    }
}

/// Allocation-free positive-control descriptor injected in the release tail or sub-release wake tail
/// (exact 1-for-1 heap-op counts with zero wrapper allocations or frees).
#[doc(hidden)]
pub enum TailAllocProbe {
    /// Performs exactly 1 `GlobalAlloc::alloc` and parks it off-thread (`alloc=1, dealloc=0`).
    AllocOnce { park: Arc<Mutex<Option<ParkedAlloc>>> },
    /// Performs exactly 1 `GlobalAlloc::alloc_zeroed` and parks it off-thread (`alloc_zeroed=1`).
    AllocZeroedOnce { park: Arc<Mutex<Option<ParkedAlloc>>> },
    /// Performs exactly 1 `GlobalAlloc::realloc` on a pre-allocated buffer and parks it off-thread (`realloc=1`).
    ReallocOnce { existing: Option<ParkedAlloc>, park: Arc<Mutex<Option<ParkedAlloc>>> },
    /// Moves a pre-allocated buffer into a release/wake-scope local so its final `dealloc` runs
    /// during scope-exit local destruction (`dealloc=1`).
    FinalFreeLocal { local: Option<ParkedAlloc> },
    /// Performs BOTH 1 `GlobalAlloc::alloc` (parked off-thread) AND 1 final `dealloc` via
    /// scope-exit local destruction (`alloc=1, dealloc=1`).
    AllocAndFinalFreeLocal {
        local: Option<ParkedAlloc>,
        park: Arc<Mutex<Option<ParkedAlloc>>>,
    },
}

/// Deterministic test hooks for persistence barriers, failure injection, and overrun tests.
#[doc(hidden)]
#[derive(Clone, Default)]
pub struct PersistTestHooks {
    /// Runs on `armd-persist` after grant and before `rename_only()`; `true` exits worker immediately.
    pub after_grant_before_rename: Option<Arc<dyn Fn() -> bool + Send + Sync>>,
    /// Runs on `armd-persist` after `rename_only()` + dir sync and before `Activate`; `true` exits worker immediately.
    pub after_rename_before_activate: Option<Arc<dyn Fn() -> bool + Send + Sync>>,
    /// Runs on `armd-control` after `persist_cmd_rx.is_abandoned()` and before `pop()`.
    pub after_abandoned_check_before_pop: Option<Arc<dyn Fn() + Send + Sync>>,
    /// Overrides parent-directory `sync_all` (e.g. simulate EIO -> `DurableUnconfirmed`).
    pub dir_sync_override: Option<Arc<dyn Fn(&Path) -> std::io::Result<()> + Send + Sync>>,
    /// Forces `ConfigAuthority::persisted()` to fail once (tests Persistence #5 fail-closed reconcile).
    pub fail_persisted_once: Option<Arc<AtomicBool>>,
    /// Forces `armd-persist` to exit cleanly right after pushing the terminal command of its first job.
    pub exit_after_first_job: bool,
    /// Triggers a panic on the N-th release (1-based) on `armd-control`.
    pub panic_on_release: Option<u64>,
    /// Forces `armd-control` init to fail (`Some(true)` returns `real`; `Some(false)` drops `real`).
    pub fail_init: Option<bool>,
    /// Injects a one-shot sleep inside the next release body (tests overrun + `skipped_releases`).
    pub release_delay_once: Option<Arc<Mutex<Option<Duration>>>>,
    /// Optional deterministic clock for testing release cadence, skipped releases, and sub-release wakes.
    pub clock: Option<Arc<dyn ExecutorClock>>,
    /// Pauses `armd-persist` while `true` (tests persistence queue saturation).
    pub pause_persist_worker: Option<Arc<AtomicBool>>,
    /// Forces `armd-persist` to leave the final `PersistReply` unpopped when exiting (tests surviving off-thread drain).
    pub leave_final_reply_queued: bool,
    /// Optional probe payload attached to the `PersistReply` (tests off-thread drop `ThreadId`).
    pub reply_drop_probe: Option<Arc<Mutex<Option<Box<dyn Any + Send>>>>>,
    /// Optional probe payload carried across `PersistCmd::Reserve` -> `PersistReply::Reserved`
    /// (tests off-thread drop `ThreadId` of actual persistence request payloads).
    pub payload_drop_probe: Option<Arc<Mutex<Option<Box<dyn Any + Send>>>>>,
    /// Positive control: performs one deliberate heap allocation+drop inside the next sub-release wake body.
    pub inject_wake_alloc_once: Option<Arc<AtomicBool>>,
    /// Positive control: exact one-shot heap probe run in the release tail (strictly AFTER `diag_tx.push`
    /// and `hub.wake()`) whose `local` drops at release-scope local destruction.
    pub inject_release_tail_once: Option<Arc<Mutex<Option<TailAllocProbe>>>>,
    /// Positive control: exact one-shot heap probe run in the sub-release wake tail (strictly AFTER
    /// `service_wake` / ack / reclaim drains) whose `local` drops at wake-scope local destruction.
    pub inject_wake_tail_once: Option<Arc<Mutex<Option<TailAllocProbe>>>>,
    /// Injects one pre-built `(PendingZeroCheckPersist, Option<drop_probe>)` on the next release
    /// (tests executor release-tail `persist_job_tx` saturation and off-thread rejected-job drop
    /// with 0 allocations in `actions.rs`).
    pub inject_pending_persist_job_once:
        Option<Arc<Mutex<Option<(PendingZeroCheckPersist, Option<Box<dyn Any + Send>>)>>>>,
    /// Positive control: performs one deliberate `Box::new` allocation inside the next release
    /// body when the `AtomicBool` is true, storing the raw pointer in the `AtomicPtr` for
    /// off-thread reclamation so the release measures `lock_alloc_calls == 1`.
    pub inject_release_alloc_once: Option<Arc<(AtomicBool, std::sync::atomic::AtomicPtr<u64>)>>,
    /// Runs on `armd-control` while handling `MaintenanceOp::CommitShutdown` before replying.
    pub on_commit_shutdown: Option<Arc<dyn Fn() + Send + Sync>>,
    /// Overrides `PANIC_DISABLE_TIMEOUT` for deterministic deadline-observer tests.
    pub panic_disable_timeout: Option<Duration>,
    /// Invoked by the independent panic deadline observer when a blocking driver call stalls
    /// past the panic-disable deadline under `PanicPolicy::StopExecutor`.
    pub on_panic_disable_timeout: Option<Arc<dyn Fn() + Send + Sync>>,
    /// Runs on `armd-control` strictly before Step 6 `StateHub` publication.
    pub before_publish: Option<Arc<dyn Fn() + Send + Sync>>,
}

enum PersistCmd {
    Reserve {
        action_id: u64,
        base_revision: u64,
        lease_owner: Option<(crate::model::HolderLabel, u64)>,
        new_policy: GripperPolicy,
        jaw_opening: Option<JawOpening>,
        probe: Option<Box<dyn Any + Send>>,
        code_buf: String,
        msg_buf: String,
    },
    Fail {
        action_id: u64,
        code: String,
        message: String,
        unused_policy: Option<GripperPolicy>,
        unused_jaw: Option<JawOpening>,
    },
    Abort {
        action_id: u64,
        reservation: Reservation,
        code: String,
        message: String,
        unused_policy: Option<GripperPolicy>,
        unused_jaw: Option<JawOpening>,
    },
    Activate {
        action_id: u64,
        reservation: Reservation,
        base_revision: u64,
        lease_owner: Option<(crate::model::HolderLabel, u64)>,
        offset_rad: f32,
        new_policy: GripperPolicy,
        new_jaw: Option<JawOpening>,
        durability: Durability,
        warning: Option<String>,
        code_buf: String,
        msg_buf: String,
    },
}

enum PersistReply {
    Reserved {
        reservation: Option<Reservation>,
        policy: Option<GripperPolicy>,
        returned_jaw: Option<JawOpening>,
        _returned_code: Option<String>,
        _returned_msg: Option<String>,
        _returned_probe: Option<Box<dyn Any + Send>>,
    },
    Done {
        _returned_policy: Option<GripperPolicy>,
        _returned_jaw: Option<JawOpening>,
        _returned_code: Option<String>,
        _returned_msg: Option<String>,
        _returned_warning: Option<String>,
        _returned_probe: Option<Box<dyn Any + Send>>,
    },
}

/// Dedicated bounded lock-free SPSC off-thread reclamation & deferred-log ring drained
/// continuously by `OffThreadSinks::drain_all()` (separate from `persist_rep_tx` so unsolicited
/// reclamation never breaks `armd-persist`'s 1-to-1 command/reply protocol).
#[allow(dead_code, clippy::large_enum_variant)] // inline in preallocated SPSC ring: boxing would allocate on armd-control
enum OffloadReclaim {
    MaintReply(MaintReply),
    PersistReply(PersistReply),
    PersistJob(PendingZeroCheckPersist, Option<Box<dyn Any + Send>>),
    Probe(Box<dyn Any + Send>),
    SafetyState(Arc<SafetyState>),
    Buf(String),
    LogLegacyJ6(crate::joint_zero::ZeroConvention),
}

/// Off-thread consumer sinks drained by `armd-dispatch`, `tick_stats()`, and `shutdown_and_join()`.
pub(crate) struct OffThreadSinks {
    diag_rx: Mutex<Consumer<TickSample>>,
    estop_reg: Arc<Mutex<EstopRegistry>>,
    reclaim_state_rx: Mutex<Consumer<Arc<SafetyState>>>,
    reclaim_return_tx: Mutex<Producer<Arc<SafetyState>>>,
    /// `(consumer, last_reported_dropped, shutdown_reported_total)` guarded by one mutex so
    /// best-effort warning deltas never race and never reset `stats.safety_log_dropped`.
    safety_log_rx: Mutex<(Consumer<SafetyLogEvent>, u64, u64)>,
    stats: Arc<ExecutorStats>,
    persist_rep_rx: Arc<Mutex<Consumer<PersistReply>>>,
    persist_worker_exited: Arc<AtomicBool>,
    maint_client: Arc<MaintClient>,
    offload_reclaim_rx: Mutex<Consumer<OffloadReclaim>>,
}

impl OffThreadSinks {
    /// Authoritative monotonic count of dropped deferred safety log events (never reset).
    pub(crate) fn safety_log_dropped(&self) -> u64 {
        self.stats.safety_log_dropped.load(Ordering::Relaxed)
    }

    pub(crate) fn drain_diag(&self, tick_stats: &Mutex<crate::diagnostics::tick::TickStats>) {
        let mut rx = self.diag_rx.lock().unwrap_or_else(|e| e.into_inner());
        let mut batch = Vec::new();
        while let Ok(sample) = rx.pop() {
            batch.push(sample);
        }
        drop(rx);
        if !batch.is_empty() {
            let mut stats = tick_stats.lock().unwrap_or_else(|e| e.into_inner());
            for s in &batch {
                stats.record(s);
            }
        }
    }

    pub(crate) fn drain_all(&self, tick_stats: &Mutex<crate::diagnostics::tick::TickStats>) {
        self.drain_diag(tick_stats);
        self.estop_reg.lock().unwrap_or_else(|e| e.into_inner()).drain();
        {
            let mut rx = self.reclaim_state_rx.lock().unwrap_or_else(|e| e.into_inner());
            let mut ret_tx = self.reclaim_return_tx.lock().unwrap_or_else(|e| e.into_inner());
            while let Ok(mut arc) = rx.pop() {
                if !ret_tx.is_abandoned() && !ret_tx.is_full() {
                    if let Some(slot) = Arc::get_mut(&mut arc) {
                        crate::safety::SafetyManager::reset_recycled_fault_arc(slot);
                        let _ = ret_tx.push(arc);
                        continue;
                    }
                }
                drop(arc);
            }
        }
        {
            let mut log_g = self.safety_log_rx.lock().unwrap_or_else(|e| e.into_inner());
            let total = self.stats.safety_log_dropped.load(Ordering::Relaxed);
            let delta = total.saturating_sub(log_g.1);
            if delta > 0 {
                log_g.1 = total;
                crate::safety::emit_safety_log_dropped(delta, total);
            }
            while let Ok(ev) = log_g.0.pop() {
                ev.emit();
            }
        }
        self.maint_client.drain_replies();
        if self.persist_worker_exited.load(Ordering::Acquire) {
            let mut rx = self.persist_rep_rx.lock().unwrap_or_else(|e| e.into_inner());
            while let Ok(rep) = rx.pop() {
                drop(rep);
            }
        }
        {
            let mut rx = self.offload_reclaim_rx.lock().unwrap_or_else(|e| e.into_inner());
            while let Ok(item) = rx.pop() {
                match item {
                    OffloadReclaim::LogLegacyJ6(zc) => {
                        if let Some(msg) = zc.message() {
                            tracing::warn!("Joint zero check: {msg}");
                        }
                    }
                    other => drop(other),
                }
            }
        }
    }

    /// Final off-thread drain at executor/dispatcher shutdown: emits any remaining events and a
    /// shutdown summary carrying the authoritative monotonic safety drop count.
    pub(crate) fn flush_and_report_shutdown(&self, tick_stats: &Mutex<crate::diagnostics::tick::TickStats>) {
        self.drain_all(tick_stats);
        let mut log_g = self.safety_log_rx.lock().unwrap_or_else(|e| e.into_inner());
        let total = self.stats.safety_log_dropped.load(Ordering::Relaxed);
        if total > 0 && total != log_g.2 {
            log_g.2 = total;
            crate::safety::emit_safety_log_shutdown_summary(total);
        }
    }
}

#[derive(Default)]
struct TeardownDisable {
    requested: AtomicBool,
    timed_out: AtomicBool,
    outcome: Mutex<Option<crate::supervisor::PanicDisableOutcome>>,
    done: Condvar,
}

impl TeardownDisable {
    fn set_outcome(&self, outcome: crate::supervisor::PanicDisableOutcome) {
        if outcome == crate::supervisor::PanicDisableOutcome::TimedOut {
            self.timed_out.store(true, Ordering::Release);
        }
        let mut g = self.outcome.lock().unwrap_or_else(|e| e.into_inner());
        *g = Some(outcome);
        self.done.notify_all();
    }

    fn wait_outcome(&self, timeout: Duration) -> Option<crate::supervisor::PanicDisableOutcome> {
        let deadline = Instant::now() + timeout;
        let mut g = self.outcome.lock().unwrap_or_else(|e| e.into_inner());
        while g.is_none() {
            let rem = deadline.saturating_duration_since(Instant::now());
            if rem.is_zero() {
                break;
            }
            let (next, res) = self.done.wait_timeout(g, rem).unwrap_or_else(|e| e.into_inner());
            g = next;
            if res.timed_out() {
                break;
            }
        }
        g.clone()
    }
}

struct RtExecutorShared {
    broker: Arc<Mutex<Broker<PlanSlot>>>,
    target_writer: Arc<Mutex<TargetWriter>>,
    stop_requests: StopRequests,
    stats: Arc<ExecutorStats>,
    maint_client: Arc<MaintClient>,
    hub: Arc<StateHub>,
    tick_stats: Arc<Mutex<crate::diagnostics::tick::TickStats>>,
    sinks: Arc<OffThreadSinks>,
    executor_thread_id: ThreadId,
    control_stop: Arc<AtomicBool>,
    persist_stop: Arc<AtomicBool>,
    exited: Arc<AtomicBool>,
    control_exited_cv: Arc<(Mutex<bool>, Condvar)>,
    persist_worker_exited: Arc<AtomicBool>,
    persist_exited_cv: Arc<(Mutex<bool>, Condvar)>,
    shutdown_confirmed: Arc<AtomicBool>,
    teardown_disable: Arc<TeardownDisable>,
    teardown_disable_timeout: Duration,
    persist_thread: Mutex<Option<JoinHandle<()>>>,
    control_thread: Mutex<Option<JoinHandle<()>>>,
    monitor_thread: Mutex<Option<JoinHandle<()>>>,
    persist_park_thread: Thread,
    clock: Option<Arc<dyn ExecutorClock>>,
}

impl RtExecutorShared {
    fn wait_flag(cv: &(Mutex<bool>, Condvar), timeout: Duration) -> bool {
        let deadline = Instant::now() + timeout;
        let mut g = cv.0.lock().unwrap_or_else(|e| e.into_inner());
        while !*g {
            let rem = deadline.saturating_duration_since(Instant::now());
            if rem.is_zero() {
                break;
            }
            let (next, res) = cv.1.wait_timeout(g, rem).unwrap_or_else(|e| e.into_inner());
            g = next;
            if res.timed_out() {
                break;
            }
        }
        *g
    }

    /// Stops the RT owner (`armd-control`) FIRST so a stalled persistence worker can never block
    /// owner shutdown or emergency hardware disable, then joins or bounds persistence cleanup.
    fn teardown_and_join_inner(&self, emergency_bound: Option<Duration>) {
        // Non-blockingly ask persistence to wind down, but NEVER wait for it before stopping
        // `armd-control`: a blocked directory `sync_all()` must not delay torque-off.
        self.persist_stop.store(true, Ordering::Release);
        self.persist_park_thread.unpark();
        self.maint_client.close();
        self.control_stop.store(true, Ordering::Release);
        if let Some(clock) = self.clock.as_ref() {
            clock.wake();
        }

        if let Some(bound) = emergency_bound {
            // Wait up to `bound` for `armd-control` to finish its bounded disable and exit;
            // if a driver syscall stalls past the bound, take and detach the join handle so no
            // later teardown or `Drop` can join it without a deadline.
            let _ = self.teardown_disable.wait_outcome(bound);
            let finished = !self.teardown_disable.timed_out.load(Ordering::Acquire)
                && Self::wait_flag(&self.control_exited_cv, Duration::from_millis(50));
            if let Some(jh) = self.control_thread.lock().unwrap_or_else(|e| e.into_inner()).take() {
                if finished {
                    let _ = jh.join();
                } else {
                    self.exited.store(true, Ordering::Release);
                    tracing::error!(
                        "Emergency teardown: detaching stalled RT executor thread (`armd-control`) after {:?} timeout so later teardown/Drop cannot block indefinitely",
                        bound
                    );
                    drop(jh);
                }
            }
        } else if self.teardown_disable.timed_out.load(Ordering::Acquire) {
            // A prior panic or emergency deadline observer already recorded `TimedOut`:
            // wait only briefly for clean exit, then detach so `Drop`/`stop_and_join` stay bounded.
            let finished = Self::wait_flag(&self.control_exited_cv, Duration::from_millis(50));
            if let Some(jh) = self.control_thread.lock().unwrap_or_else(|e| e.into_inner()).take() {
                if finished {
                    let _ = jh.join();
                } else {
                    self.exited.store(true, Ordering::Release);
                    tracing::error!(
                        "Teardown: detaching stalled RT executor thread (`armd-control`) after panic/emergency timeout so Drop cannot block indefinitely"
                    );
                    drop(jh);
                }
            }
        } else if let Some(jh) = self.control_thread.lock().unwrap_or_else(|e| e.into_inner()).take() {
            let _ = jh.join();
        }

        if let Some(jh) = self.monitor_thread.lock().unwrap_or_else(|e| e.into_inner()).take() {
            jh.thread().unpark();
            let _ = jh.join();
        }

        // Now wait for persistence cleanup: bounded + detach on emergency teardown (so a hung
        // `sync_all` cannot block returning the torque-off outcome or a subsequent `Drop`),
        // full join on normal teardown / Drop.
        if emergency_bound.is_some() {
            let finished = Self::wait_flag(&self.persist_exited_cv, Duration::from_millis(50));
            if let Some(jh) = self.persist_thread.lock().unwrap_or_else(|e| e.into_inner()).take() {
                if finished {
                    let _ = jh.join();
                } else {
                    tracing::warn!(
                        "Emergency teardown: detaching stalled persistence worker (`armd-persist`) after bounded wait so later teardown/Drop cannot block indefinitely"
                    );
                    drop(jh);
                }
            }
        } else if let Some(jh) = self.persist_thread.lock().unwrap_or_else(|e| e.into_inner()).take() {
            let _ = jh.join();
        }

        // Final off-thread drain and reclamation.
        self.maint_client.finish_stopped();
        self.sinks.flush_and_report_shutdown(&self.tick_stats);
        if let Some(ring) = self.hub.record_ring() {
            ring.drain();
        }
        let _ = self.hub.latest();
        self.broker.lock().unwrap_or_else(|e| e.into_inner()).poll(|_, _| true);
    }

    fn teardown_and_join(&self) {
        self.teardown_and_join_inner(None);
    }
}

impl Drop for RtExecutorShared {
    fn drop(&mut self) {
        self.teardown_and_join();
    }
}

/// Cloneable handle to the running synchronous RT executor.
#[derive(Clone)]
pub struct RtExecutorHandle {
    shared: Arc<RtExecutorShared>,
}

impl RtExecutorHandle {
    pub fn broker(&self) -> &Arc<Mutex<Broker<PlanSlot>>> { &self.shared.broker }
    pub fn target_writer(&self) -> &Arc<Mutex<TargetWriter>> { &self.shared.target_writer }
    pub fn stop_requests(&self) -> &StopRequests { &self.shared.stop_requests }
    pub fn stats(&self) -> &Arc<ExecutorStats> { &self.shared.stats }
    pub fn executor_thread_id(&self) -> ThreadId { self.shared.executor_thread_id }
    pub fn is_exited(&self) -> bool { self.shared.exited.load(Ordering::Acquire) }
    #[doc(hidden)]
    pub fn persist_worker_exited(&self) -> bool { self.shared.persist_worker_exited.load(Ordering::Acquire) }

    pub fn call_maintenance(&self, op: MaintenanceOp) -> Result<MaintenanceResult, OwnerError> {
        self.shared.maint_client.call_sync(op)
    }

    pub async fn call_maintenance_async(&self, op: MaintenanceOp) -> Result<MaintenanceResult, OwnerError> {
        self.shared.maint_client.call_async(op).await
    }

    pub fn request_shutdown(&self) -> Result<bool, OwnerError> {
        match self.call_maintenance(MaintenanceOp::RequestShutdown)? {
            MaintenanceResult::ShutdownRequested(confirmed) => Ok(confirmed),
            _ => Err(OwnerError::Protocol),
        }
    }

    pub async fn request_shutdown_async(&self) -> Result<bool, OwnerError> {
        match self.call_maintenance_async(MaintenanceOp::RequestShutdown).await? {
            MaintenanceResult::ShutdownRequested(confirmed) => Ok(confirmed),
            _ => Err(OwnerError::Protocol),
        }
    }

    pub fn shutdown_ready(&self) -> Result<bool, OwnerError> {
        match self.call_maintenance(MaintenanceOp::ShutdownReady)? {
            MaintenanceResult::ShutdownReady(v) => Ok(v),
            _ => Err(OwnerError::Protocol),
        }
    }

    pub async fn shutdown_ready_async(&self) -> Result<bool, OwnerError> {
        match self.call_maintenance_async(MaintenanceOp::ShutdownReady).await? {
            MaintenanceResult::ShutdownReady(v) => Ok(v),
            _ => Err(OwnerError::Protocol),
        }
    }

    pub fn planning_snapshot(&self) -> Result<PlanningSnapshot, OwnerError> {
        match self.call_maintenance(MaintenanceOp::PlanningSnapshot)? {
            MaintenanceResult::PlanningSnapshot(s) => Ok(s),
            _ => Err(OwnerError::Protocol),
        }
    }

    pub fn observation(&self) -> Result<ArmObservation, OwnerError> {
        match self.call_maintenance(MaintenanceOp::Observation)? {
            MaintenanceResult::Observation(o) => Ok(*o),
            _ => Err(OwnerError::Protocol),
        }
    }

    pub async fn observation_async(&self) -> Result<ArmObservation, OwnerError> {
        match self.call_maintenance_async(MaintenanceOp::Observation).await? {
            MaintenanceResult::Observation(o) => Ok(*o),
            _ => Err(OwnerError::Protocol),
        }
    }

    pub fn clear_fault(&self) -> Result<Result<SafetyState, SafetyError>, OwnerError> {
        self.clear_fault_for_epoch(None)
    }

    pub fn clear_fault_for_epoch(
        &self,
        expected_epoch: Option<u64>,
    ) -> Result<Result<SafetyState, SafetyError>, OwnerError> {
        match self.call_maintenance(MaintenanceOp::ClearFault { expected_epoch })? {
            MaintenanceResult::FaultCleared(r) => Ok(r.map_err(SafetyError::from)),
            _ => Err(OwnerError::Protocol),
        }
    }

    pub async fn clear_fault_async(&self) -> Result<Result<SafetyState, SafetyError>, OwnerError> {
        self.clear_fault_for_epoch_async(None).await
    }

    pub async fn clear_fault_for_epoch_async(
        &self,
        expected_epoch: Option<u64>,
    ) -> Result<Result<SafetyState, SafetyError>, OwnerError> {
        match self
            .call_maintenance_async(MaintenanceOp::ClearFault { expected_epoch })
            .await?
        {
            MaintenanceResult::FaultCleared(r) => Ok(r.map_err(SafetyError::from)),
            _ => Err(OwnerError::Protocol),
        }
    }

    pub fn set_joint_zero_offsets(&self, offsets: Vec<f32>) -> Result<Result<(), String>, OwnerError> {
        match self.call_maintenance(MaintenanceOp::SetJointZeroOffsets { offsets })? {
            MaintenanceResult::JointZeroOffsetsSet(r) => Ok(r),
            _ => Err(OwnerError::Protocol),
        }
    }

    pub fn set_motor_zero(&self, joint: usize) -> Result<Result<(), String>, OwnerError> {
        match self.call_maintenance(MaintenanceOp::SetMotorZero { joint })? {
            MaintenanceResult::MotorZeroSet(r) => Ok(r),
            _ => Err(OwnerError::Protocol),
        }
    }

    #[doc(hidden)]
    pub fn probe_owned_payload(&self, payload: Box<dyn Any + Send>) -> Result<(), OwnerError> {
        match self.call_maintenance(MaintenanceOp::ProbeOwnedPayload { payload })? {
            MaintenanceResult::ProbeReturned => Ok(()),
            _ => Err(OwnerError::Protocol),
        }
    }

    /// Atomically revalidates shutdown confirmation on the RT owner (`armd-control`), commits
    /// stop on the owner thread, and joins all worker threads. Returns `Err(ShutdownNotConfirmed)`
    /// without stopping the executor if shutdown was never confirmed or the arm was rearmed since.
    pub fn shutdown_and_join(&self) -> Result<(), OwnerError> {
        match self.call_maintenance(MaintenanceOp::CommitShutdown) {
            Ok(MaintenanceResult::ShutdownCommitted(true)) => {
                self.shared.shutdown_confirmed.store(true, Ordering::Release);
                self.shared.teardown_and_join();
                Ok(())
            }
            Ok(MaintenanceResult::ShutdownCommitted(false)) => Err(OwnerError::ShutdownNotConfirmed),
            Ok(_) => Err(OwnerError::Protocol),
            Err(OwnerError::Stopped) if self.shared.shutdown_confirmed.load(Ordering::Acquire) => {
                self.shared.teardown_and_join();
                Ok(())
            }
            Err(e) => Err(e),
        }
    }

    /// Unconditional stop and join for tests and forced teardown.
    pub fn stop_and_join(&self) {
        self.shared.teardown_and_join();
    }

    /// Requests the RT owner thread (`armd-control`) to perform one bounded hardware torque
    /// disable during teardown before exiting, then collects the bounded outcome before waiting
    /// on persistence. Preserves the single-bus-writer invariant when final shutdown commit was
    /// refused or failed, and never stalls behind persistence I/O.
    pub fn emergency_disable_and_join(&self) -> crate::supervisor::PanicDisableOutcome {
        self.shared.teardown_disable.requested.store(true, Ordering::Release);
        let bound = self.shared.teardown_disable_timeout + Duration::from_millis(100);
        self.shared.teardown_and_join_inner(Some(bound));
        self.shared
            .teardown_disable
            .outcome
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
            .unwrap_or(crate::supervisor::PanicDisableOutcome::TimedOut)
    }
}

/// Spawns the synchronous RT executor (`armd-control`) and dedicated persistence worker (`armd-persist`).
pub(crate) fn spawn_executor(
    mut core: SupervisorInner,
    cfg: ExecutorConfig,
    initial_generation: u64,
    config_write_mutex: Arc<tokio::sync::Mutex<()>>,
    estop_reg: Arc<Mutex<EstopRegistry>>,
    tick_stats: Arc<Mutex<crate::diagnostics::tick::TickStats>>,
    control_sched: Arc<Mutex<Option<crate::diagnostics::tick::ControlThreadSched>>>,
    control_period_us: Arc<AtomicU64>,
    hooks: PersistTestHooks,
) -> Result<(RtExecutorHandle, Arc<OffThreadSinks>), (Option<Box<SupervisorInner>>, String)> {
    let hub = core.core.hub().clone();
    let e_stop_latched = core.e_stop_latched.clone();
    let (engine, broker, target_writer, stop_requests) = ReleaseEngine::new(cfg.boot, cfg.capacities);
    core.attach_stop_requests(stop_requests.clone());

    let (diag_tx, diag_rx) = RingBuffer::<TickSample>::new(DIAG_RING_CAPACITY);
    let (maint_req_tx, maint_req_rx) = RingBuffer::<MaintRequest>::new(MAINT_QUEUE_CAPACITY);
    let (maint_rep_tx, maint_rep_rx) = RingBuffer::<MaintReply>::new(MAINT_QUEUE_CAPACITY);
    let (estop_ack_tx, estop_ack_rx) = RingBuffer::<EstopAck>::new(ESTOP_ACK_CAPACITY);
    let (reclaim_state_tx, reclaim_state_rx) = RingBuffer::<Arc<SafetyState>>::new(RECLAIM_STATE_CAPACITY);
    let (reclaim_return_tx, reclaim_return_rx) = RingBuffer::<Arc<SafetyState>>::new(
        FAULT_ARC_POOL_CAPACITY + FORMATTED_FAULT_ARC_POOL_CAPACITY + JOINT_LIMIT_ARC_POOL_CAPACITY,
    );
    let (safety_log_tx, safety_log_rx) = RingBuffer::<SafetyLogEvent>::new(SAFETY_LOG_CAPACITY);
    let (persist_job_tx, persist_job_rx) = RingBuffer::<PendingZeroCheckPersist>::new(4);
    let (persist_cmd_tx, persist_cmd_rx) = RingBuffer::<PersistCmd>::new(2);
    let (persist_rep_tx, persist_rep_rx) = RingBuffer::<PersistReply>::new(PERSIST_REP_CAPACITY);
    let (offload_reclaim_tx, offload_reclaim_rx) =
        RingBuffer::<OffloadReclaim>::new(OFFLOAD_RECLAIM_CAPACITY);

    let estop_overflow_gen = Arc::new(AtomicU64::new(0));
    let control_stop = Arc::new(AtomicBool::new(false));
    let persist_stop = Arc::new(AtomicBool::new(false));
    let exited = Arc::new(AtomicBool::new(false));
    let control_exited_cv = Arc::new((Mutex::new(false), Condvar::new()));
    let persist_worker_exited = Arc::new(AtomicBool::new(false));
    let persist_exited_cv = Arc::new((Mutex::new(false), Condvar::new()));
    let teardown_disable = Arc::new(TeardownDisable::default());
    let stats = Arc::new(ExecutorStats::default());

    let persist_rep_rx = Arc::new(Mutex::new(persist_rep_rx));
    let maint_client = Arc::new(MaintClient::new(maint_req_tx, maint_rep_rx, exited.clone()));
    let sinks = Arc::new(OffThreadSinks {
        diag_rx: Mutex::new(diag_rx),
        estop_reg: estop_reg.clone(),
        reclaim_state_rx: Mutex::new(reclaim_state_rx),
        reclaim_return_tx: Mutex::new(reclaim_return_tx),
        safety_log_rx: Mutex::new((safety_log_rx, 0, 0)),
        stats: stats.clone(),
        persist_rep_rx: persist_rep_rx.clone(),
        persist_worker_exited: persist_worker_exited.clone(),
        maint_client: maint_client.clone(),
        offload_reclaim_rx: Mutex::new(offload_reclaim_rx),
    });

    // Spawn dedicated persistence worker first so its thread handle can be woken on job push.
    let persist_thread = match spawn_persist_worker(
        persist_job_rx,
        persist_cmd_tx,
        persist_rep_rx.clone(),
        config_write_mutex,
        persist_stop.clone(),
        exited.clone(),
        persist_worker_exited.clone(),
        persist_exited_cv.clone(),
        hub.clone(),
        hooks.clone(),
    ) {
        Ok(jh) => jh,
        Err(e) => return Err((Some(Box::new(core)), format!("failed to spawn armd-persist: {e}"))),
    };
    let persist_park_thread = persist_thread.thread().clone();

    let (ready_tx, ready_rx) = mpsc::sync_channel::<Result<ThreadId, (Option<Box<SupervisorInner>>, String)>>(1);
    let control_thread = {
        let cfg_clone = cfg.clone();
        let control_stop = control_stop.clone();
        let exited = exited.clone();
        let control_exited_cv = control_exited_cv.clone();
        let teardown_disable = teardown_disable.clone();
        let stats = stats.clone();
        let estop_overflow_gen = estop_overflow_gen.clone();
        let hub_clone = hub.clone();
        let persist_wake = persist_park_thread.clone();
        let hooks = hooks.clone();
        match thread::Builder::new().name("armd-control".into()).spawn(move || {
            let _control_thread_guard = ControlThreadGuard::enter();
            control_thread_main(
                core,
                engine,
                cfg_clone,
                initial_generation,
                ready_tx,
                control_stop,
                exited,
                control_exited_cv,
                teardown_disable,
                stats,
                control_sched,
                control_period_us,
                diag_tx,
                maint_req_rx,
                maint_rep_tx,
                estop_ack_tx,
                estop_overflow_gen,
                reclaim_state_tx,
                reclaim_return_rx,
                safety_log_tx,
                persist_job_tx,
                persist_cmd_rx,
                persist_rep_tx,
                offload_reclaim_tx,
                persist_wake,
                e_stop_latched,
                hub_clone,
                hooks,
            );
        }) {
            Ok(jh) => jh,
            Err(e) => {
                persist_stop.store(true, Ordering::Release);
                persist_park_thread.unpark();
                let _ = persist_thread.join();
                return Err((None, format!("failed to spawn armd-control: {e}")));
            }
        }
    };

    let executor_thread_id = match ready_rx.recv() {
        Ok(Ok(tid)) => tid,
        Ok(Err((returned_core, msg))) => {
            persist_stop.store(true, Ordering::Release);
            persist_park_thread.unpark();
            let _ = persist_thread.join();
            let _ = control_thread.join();
            return Err((returned_core, msg));
        }
        Err(_) => {
            persist_stop.store(true, Ordering::Release);
            persist_park_thread.unpark();
            let _ = persist_thread.join();
            let _ = control_thread.join();
            return Err((None, "armd-control exited before signaling readiness".into()));
        }
    };

    // Attach executor ack consumer to EstopRegistry atomically before returning handle.
    estop_reg
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .attach_executor(estop_ack_rx, estop_overflow_gen, exited.clone());

    let monitor_thread = stats.spawn_monitor(control_stop.clone()).ok();

    let shared = Arc::new(RtExecutorShared {
        broker: Arc::new(Mutex::new(broker)),
        target_writer: Arc::new(Mutex::new(target_writer)),
        stop_requests,
        stats,
        maint_client,
        hub,
        tick_stats,
        sinks: sinks.clone(),
        executor_thread_id,
        control_stop,
        persist_stop,
        exited,
        control_exited_cv,
        persist_worker_exited,
        persist_exited_cv,
        shutdown_confirmed: Arc::new(AtomicBool::new(false)),
        teardown_disable,
        teardown_disable_timeout: hooks
            .panic_disable_timeout
            .unwrap_or(crate::supervisor::PANIC_DISABLE_TIMEOUT),
        persist_thread: Mutex::new(Some(persist_thread)),
        control_thread: Mutex::new(Some(control_thread)),
        monitor_thread: Mutex::new(monitor_thread),
        persist_park_thread,
        clock: hooks.clock.clone(),
    });
    Ok((RtExecutorHandle { shared }, sinks))
}

struct ControlState {
    shutdown_committed: bool,
    control_stop: Arc<AtomicBool>,
    stop_requests: StopRequests,
    estop_ack_tx: Producer<EstopAck>,
    estop_overflow_gen: Arc<AtomicU64>,
    reclaim_state_tx: Producer<Arc<SafetyState>>,
    reclaim_return_rx: Consumer<Arc<SafetyState>>,
    safety_log_tx: Producer<SafetyLogEvent>,
    stats: Arc<ExecutorStats>,
    maint_req_rx: Consumer<MaintRequest>,
    maint_rep_tx: Producer<MaintReply>,
    /// Reply for a maintenance op serviced before Step 3 commit / Step 6 publish, held until
    /// strictly after Step 6 publication (or flushed immediately when `CommitShutdown` stops).
    pending_maint_reply: Option<MaintReply>,
    persist_job_tx: Producer<PendingZeroCheckPersist>,
    persist_cmd_rx: Consumer<PersistCmd>,
    persist_rep_tx: Producer<PersistReply>,
    /// Retained persistence reply if `persist_rep_tx` was temporarily full (flushed before popping).
    pending_persist_reply: Option<PersistReply>,
    /// Retained rejected persistence job if `persist_job_tx` and `offload_reclaim_tx` were both saturated.
    retained_persist_job: Option<(PendingZeroCheckPersist, Option<Box<dyn Any + Send>>)>,
    /// Dedicated bounded lock-free SPSC off-thread reclamation & deferred-log ring.
    offload_reclaim_tx: Producer<OffloadReclaim>,
    persist_wake: Thread,
    persist_granted: Option<(u64, ConfigDomain)>,
    next_persist_req: u64,
    disconnect_code_buf: Option<String>,
    disconnect_msg_buf: Option<String>,
    /// Preallocated message buffer for release-tail persistence queue saturation (recycled if returned).
    sat_msg_buf: Option<String>,
    hooks: PersistTestHooks,
}

/// True when `armd-control` itself permanently or currently holds another strong reference to
/// `ack.state` inside `core.safety`, proving that dropping `ack` on `armd-control` cannot be
/// the final `Arc` drop (single-thread guaranteed, zero race with `armd-dispatch`).
#[inline]
fn is_owner_retained_state(ack: &EstopAck, core: &SupervisorInner) -> bool {
    !matches!(*ack.state, SafetyState::Fault(_))
        || std::ptr::eq(Arc::as_ptr(&ack.state), core.safety.state() as *const SafetyState)
}

impl ControlState {
    fn emit_estop_ack_if_advanced(&mut self, core: &mut SupervisorInner) {
        let mut woke = false;
        while let Some(front) = core.pending_estop_acks.front() {
            if !self.estop_ack_tx.is_full() {
                let ack = core.pending_estop_acks.pop_front().expect("checked front");
                let _ = self.estop_ack_tx.push(ack);
                woke = true;
            } else if is_owner_retained_state(front, core) {
                let ack = core.pending_estop_acks.pop_front().expect("checked front");
                self.estop_overflow_gen.fetch_max(ack.last_gen, Ordering::Release);
                // Safe: `core.safety` retains a live strong ref on this thread (`is_owner_retained_state`).
                drop(ack);
                woke = true;
            } else if !self.reclaim_state_tx.is_full() {
                let ack = core.pending_estop_acks.pop_front().expect("checked front");
                self.estop_overflow_gen.fetch_max(ack.last_gen, Ordering::Release);
                let _ = self.reclaim_state_tx.push(ack.state);
                woke = true;
            } else {
                // Both `estop_ack_tx` and `reclaim_state_tx` are full and `front.state` is a
                // retired Fault that may own the final strong reference: record the overflow
                // watermark and leave it in bounded `core.pending_estop_acks` until a ring drains.
                self.estop_overflow_gen.fetch_max(front.last_gen, Ordering::Release);
                woke = true;
                break;
            }
        }
        if core.estop_ack_overflow_gen > 0 {
            let ov = std::mem::replace(&mut core.estop_ack_overflow_gen, 0);
            self.estop_overflow_gen.fetch_max(ov, Ordering::Release);
            woke = true;
        }
        if woke {
            core.core.hub().wake();
        }
    }

    /// Clean-exit E-stop drain: moves any remaining pending E-stop acks / retired fault arcs into
    /// `estop_ack_tx`, `reclaim_state_tx`, or lock-free `offload_reclaim_tx` so nothing drops on RT.
    fn drain_estop_and_retired_stopped(&mut self, core: &mut SupervisorInner) {
        self.emit_estop_ack_if_advanced(core);
        while let Some(ack) = core.pending_estop_acks.pop_front() {
            self.estop_overflow_gen.fetch_max(ack.last_gen, Ordering::Release);
            if let Err(rtrb::PushError::Full(state)) = self.reclaim_state_tx.push(ack.state) {
                let _ = self.offload_reclaim_tx.push(OffloadReclaim::SafetyState(state));
            }
        }
        self.drain_retired_states(core);
        while let Some(arc) = core.safety.pop_retired_state_arc() {
            if let Err(rtrb::PushError::Full(state)) = self.reclaim_state_tx.push(arc) {
                let _ = self.offload_reclaim_tx.push(OffloadReclaim::SafetyState(state));
            }
        }
    }

    /// Moves retired fault `Arc<SafetyState>` snapshots and deferred [`SafetyLogEvent`]s off
    /// `armd-control` into the bounded SPSC rings, and replenishes clean recycled fault slots
    /// without allocating or dropping on `armd-control`.
    fn drain_retired_states(&mut self, core: &mut SupervisorInner) {
        let mut drained = false;
        while !self.reclaim_state_tx.is_full()
            && (core.safety.fault_arc_pool_len() < FAULT_ARC_POOL_CAPACITY
                || core.safety.formatted_fault_arc_pool_len() < FORMATTED_FAULT_ARC_POOL_CAPACITY
                || core.safety.joint_limit_arc_pool_len() < JOINT_LIMIT_ARC_POOL_CAPACITY)
        {
            let Ok(arc) = self.reclaim_return_rx.pop() else {
                break;
            };
            if let Err(unusable) = core.safety.return_fault_arc(arc) {
                let _ = self.reclaim_state_tx.push(unusable);
                drained = true;
            }
        }
        while !self.reclaim_state_tx.is_full() {
            let Some(arc) = core.safety.pop_retired_state_arc() else {
                break;
            };
            let _ = self.reclaim_state_tx.push(arc);
            drained = true;
        }
        let lost_inline = core.safety.advance_log_dropped_forwarded();
        if lost_inline > 0 {
            self.stats.safety_log_dropped.fetch_add(lost_inline, Ordering::Relaxed);
            drained = true;
        }
        while let Some(ev) = core.safety.pop_log_event() {
            if self.safety_log_tx.push(ev).is_err() {
                core.safety_log_ring_dropped = core.safety_log_ring_dropped.saturating_add(1);
                self.stats.safety_log_dropped.fetch_add(1, Ordering::Relaxed);
            }
            drained = true;
        }
        if drained {
            core.core.hub().wake();
        }
    }

    fn service_one_maintenance(
        &mut self,
        core: &mut SupervisorInner,
        ctx: &super::ops::CommitCtx,
        release_deadline: Instant,
        step_num: u64,
        release_flags: &mut u32,
        maint_req_id: &mut u64,
    ) {
        if self.pending_maint_reply.is_some() || self.maint_rep_tx.is_full() {
            return;
        }
        let Ok(req) = self.maint_req_rx.pop() else { return; };
        *maint_req_id = req.id;
        let mut returned_offsets = None;
        let mut returned_probe = None;
        let result = match req.op {
            MaintenanceOp::RequestShutdown => {
                let confirmed = core.request_shutdown();
                self.shutdown_committed = confirmed;
                MaintenanceResult::ShutdownRequested(confirmed)
            }
            MaintenanceOp::CommitShutdown => {
                if let Some(hook) = self.hooks.on_commit_shutdown.as_ref() {
                    hook();
                }
                let valid = self.shutdown_committed && core.commit_shutdown();
                if valid {
                    self.control_stop.store(true, Ordering::Release);
                } else {
                    self.shutdown_committed = false;
                }
                MaintenanceResult::ShutdownCommitted(valid)
            }
            MaintenanceOp::ShutdownReady => MaintenanceResult::ShutdownReady(core.shutdown_ready()),
            MaintenanceOp::PlanningSnapshot => {
                MaintenanceResult::PlanningSnapshot(core.planning_snapshot(ctx))
            }
            MaintenanceOp::Observation => MaintenanceResult::Observation(Box::new(core.current_observation())),
            MaintenanceOp::ClearFault { expected_epoch } => {
                self.shutdown_committed = false;
                let res = core.clear_fault_executor_bounded_for_epoch(release_deadline, expected_epoch);
                if res.is_ok() {
                    *release_flags |= RELEASE_FLAG_CLEAR_OK;
                } else {
                    *release_flags |= RELEASE_FLAG_CLEAR_REFUSED;
                }
                self.stats.last_clear_release_seq.store(step_num, Ordering::Release);
                MaintenanceResult::FaultCleared(res)
            }
            MaintenanceOp::SetJointZeroOffsets { offsets } => {
                let res = if *core.safety.state() != SafetyState::Idle
                    || core.is_torqued
                    || core.disable_pending
                    || core.active_trajectory.is_some()
                {
                    Err("Arm must be Idle with torque off and no active trajectory to set joint zero offsets".into())
                } else {
                    core.set_joint_zero_offsets(&offsets)
                };
                returned_offsets = Some(offsets);
                MaintenanceResult::JointZeroOffsetsSet(res)
            }
            MaintenanceOp::SetMotorZero { joint } => {
                let res = if core.driver.is_some() {
                    Err("UNSUPPORTED: motor zero requires legacy path when hardware driver is attached".into())
                } else if *core.safety.state() != SafetyState::Idle
                    || core.is_torqued
                    || core.disable_pending
                {
                    Err("Arm must be Idle with torque off to set motor zero".into())
                } else {
                    core.set_motor_zero(joint)
                };
                MaintenanceResult::MotorZeroSet(res)
            }
            MaintenanceOp::ProbeOwnedPayload { payload } => {
                returned_probe = Some(payload);
                MaintenanceResult::ProbeReturned
            }
        };
        let returned_driver_config_error = core.retired_driver_config_error.take();
        self.pending_maint_reply = Some(MaintReply {
            id: req.id,
            result,
            returned_offsets,
            returned_probe,
            returned_driver_config_error,
        });
    }

    fn flush_pending_maint_reply(&mut self, core: &SupervisorInner) {
        if let Some(reply) = self.pending_maint_reply.take() {
            match self.maint_rep_tx.push(reply) {
                Ok(()) => core.core.hub().wake(),
                Err(rtrb::PushError::Full(reply)) => {
                    self.pending_maint_reply = Some(reply);
                }
            }
        }
    }

    fn push_maint_reply_or_offload(&mut self, reply: MaintReply) {
        if let Err(rtrb::PushError::Full(reply)) = self.maint_rep_tx.push(reply) {
            let _ = self.offload_reclaim_tx.push(OffloadReclaim::MaintReply(reply));
        }
    }

    fn drain_maintenance_stopped(&mut self) {
        if let Some(reply) = self.pending_maint_reply.take() {
            self.push_maint_reply_or_offload(MaintReply {
                id: reply.id,
                result: MaintenanceResult::Stopped,
                returned_offsets: reply.returned_offsets,
                returned_probe: reply.returned_probe,
                returned_driver_config_error: reply.returned_driver_config_error,
            });
        }
        while let Ok(req) = self.maint_req_rx.pop() {
            let (returned_offsets, returned_probe) = match req.op {
                MaintenanceOp::SetJointZeroOffsets { offsets } => (Some(offsets), None),
                MaintenanceOp::ProbeOwnedPayload { payload } => (None, Some(payload)),
                _ => (None, None),
            };
            self.push_maint_reply_or_offload(MaintReply {
                id: req.id,
                result: MaintenanceResult::Stopped,
                returned_offsets,
                returned_probe,
                returned_driver_config_error: None,
            });
        }
    }

    fn push_persist_reply_or_retain(&mut self, reply: PersistReply, core: &SupervisorInner) {
        match self.persist_rep_tx.push(reply) {
            Ok(()) => {
                self.persist_wake.unpark();
                core.core.hub().wake();
            }
            Err(rtrb::PushError::Full(reply)) => {
                debug_assert!(
                    self.pending_persist_reply.is_none(),
                    "persist reply headroom rule violated: pending_persist_reply already occupied"
                );
                self.pending_persist_reply = Some(reply);
            }
        }
    }

    fn push_persist_reply_or_offload(&mut self, reply: PersistReply) {
        if let Err(rtrb::PushError::Full(reply)) = self.persist_rep_tx.push(reply) {
            let _ = self.offload_reclaim_tx.push(OffloadReclaim::PersistReply(reply));
        }
    }

    fn flush_retained_persist(&mut self, core: &SupervisorInner) {
        let mut woke_persist = false;
        let mut woke_hub = false;
        if let Some(reply) = self.pending_persist_reply.take() {
            match self.persist_rep_tx.push(reply) {
                Ok(()) => {
                    woke_persist = true;
                    woke_hub = true;
                }
                Err(rtrb::PushError::Full(reply)) => {
                    self.pending_persist_reply = Some(reply);
                }
            }
        }
        if let Some((job, probe)) = self.retained_persist_job.take() {
            match self.offload_reclaim_tx.push(OffloadReclaim::PersistJob(job, probe)) {
                Ok(()) => woke_hub = true,
                Err(rtrb::PushError::Full(OffloadReclaim::PersistJob(job, probe))) => {
                    self.retained_persist_job = Some((job, probe));
                }
                Err(_) => unreachable!(),
            }
        }
        if woke_persist {
            self.persist_wake.unpark();
        }
        if woke_hub {
            core.core.hub().wake();
        }
    }

    /// Exit-path persistence drain: returns any queued `PersistCmd` heap payloads off-thread
    /// via `persist_rep_tx` (falling back to lock-free `offload_reclaim_tx` if full) and wakes
    /// `armd-persist` WITHOUT performing any unpublished live state mutations.
    fn drain_persistence_stopped(&mut self, core: &SupervisorInner) {
        let mut woke = false;
        if let Some(reply) = self.pending_persist_reply.take() {
            self.push_persist_reply_or_offload(reply);
            woke = true;
        }
        if let Some((job, probe)) = self.retained_persist_job.take() {
            let _ = self.offload_reclaim_tx.push(OffloadReclaim::PersistJob(job, probe));
            woke = true;
        }
        while let Ok(cmd) = self.persist_cmd_rx.pop() {
            let reply = match cmd {
                PersistCmd::Reserve {
                    new_policy,
                    jaw_opening,
                    probe,
                    code_buf,
                    msg_buf,
                    ..
                } => PersistReply::Reserved {
                    reservation: None,
                    policy: Some(new_policy),
                    returned_jaw: jaw_opening,
                    _returned_code: Some(code_buf),
                    _returned_msg: Some(msg_buf),
                    _returned_probe: probe,
                },
                PersistCmd::Fail {
                    code,
                    message,
                    unused_policy,
                    unused_jaw,
                    ..
                }
                | PersistCmd::Abort {
                    code,
                    message,
                    unused_policy,
                    unused_jaw,
                    ..
                } => PersistReply::Done {
                    _returned_policy: unused_policy,
                    _returned_jaw: unused_jaw,
                    _returned_code: Some(code),
                    _returned_msg: Some(message),
                    _returned_warning: None,
                    _returned_probe: self.take_reply_probe(),
                },
                PersistCmd::Activate {
                    new_policy,
                    new_jaw,
                    warning,
                    code_buf,
                    msg_buf,
                    ..
                } => PersistReply::Done {
                    _returned_policy: Some(new_policy),
                    _returned_jaw: new_jaw,
                    _returned_code: Some(code_buf),
                    _returned_msg: Some(msg_buf),
                    _returned_warning: warning,
                    _returned_probe: self.take_reply_probe(),
                },
            };
            self.push_persist_reply_or_offload(reply);
            woke = true;
        }
        self.persist_granted = None;
        if self.disconnect_code_buf.is_some() || self.disconnect_msg_buf.is_some() {
            let code = self.disconnect_code_buf.take();
            let msg = self.disconnect_msg_buf.take();
            self.push_persist_reply_or_offload(PersistReply::Done {
                _returned_policy: None,
                _returned_jaw: None,
                _returned_code: code,
                _returned_msg: msg,
                _returned_warning: None,
                _returned_probe: None,
            });
            woke = true;
        }
        if let Some(buf) = self.sat_msg_buf.take() {
            let _ = self.offload_reclaim_tx.push(OffloadReclaim::Buf(buf));
            woke = true;
        }
        if woke {
            self.persist_wake.unpark();
            core.core.hub().wake();
        }
    }

    fn service_persistence(&mut self, core: &mut SupervisorInner, ctx: &super::ops::CommitCtx) {
        self.flush_retained_persist(core);
        // B1 race-free abandonment rule: sample `is_abandoned()` BEFORE draining queued commands!
        // Because `rtrb` preserves queued items after producer drop, nothing can be pushed after
        // `abandoned == true` is observed.
        let abandoned = self.persist_cmd_rx.is_abandoned();
        if let Some(hook) = self.hooks.after_abandoned_check_before_pop.as_ref() {
            hook();
        }
        // Capacity rule: pop `persist_cmd_rx` only while `persist_rep_tx` retains >= 2 free slots
        // (1 for this command's 1-to-1 reply + 1 reserved for post-abandonment disconnect), so no
        // `PersistReply` is ever dropped on `armd-control`. Rejected jobs use `offload_reclaim_tx`.
        while self.pending_persist_reply.is_none() && self.persist_rep_tx.slots() >= 2 {
            let Ok(cmd) = self.persist_cmd_rx.pop() else { break; };
            self.handle_persist_cmd(core, ctx, cmd);
        }
        if abandoned
            && self.persist_cmd_rx.is_empty()
            && self.pending_persist_reply.is_none()
            && !self.persist_rep_tx.is_full()
        {
            if let Some((action_id, domain)) = self.persist_granted.take() {
                let active_rev = ConfigAuthority::revision(core, domain);
                core.core.config_mut().force_reconcile(domain, active_rev);
                if core.zero_check_persisting == Some(action_id) {
                    core.zero_check_persisting = None;
                }
                let (old_code, old_msg) = core.finish_zero_check_action_in_place(
                    action_id,
                    "failed",
                    "PERSIST_FAILED",
                    "Persistence worker disconnected after reservation",
                    self.disconnect_code_buf.take(),
                    self.disconnect_msg_buf.take(),
                );
                if old_code.is_some() || old_msg.is_some() {
                    self.push_persist_reply_or_retain(
                        PersistReply::Done {
                            _returned_policy: None,
                            _returned_jaw: None,
                            _returned_code: old_code,
                            _returned_msg: old_msg,
                            _returned_warning: None,
                            _returned_probe: None,
                        },
                        core,
                    );
                }
            }
        }
    }

    /// Release-tail handler when `persist_job_tx.push(job)` fails: marks the action failed without
    /// heap allocation (recycling `sat_msg_buf` or updating inline `HolderLabel`s when depleted)
    /// and offloads `(job, probe)` via dedicated `offload_reclaim_tx` (NEVER `persist_rep_tx`,
    /// preserving strict 1-to-1 `PersistCmd`/`PersistReply` protocol framing with `armd-persist`).
    fn reject_saturated_persist_job(
        &mut self,
        core: &mut SupervisorInner,
        job: PendingZeroCheckPersist,
        probe: Option<Box<dyn Any + Send>>,
    ) {
        let mut returned_msg: Option<String> = None;
        let can_offload_msg = self.offload_reclaim_tx.slots() >= 2;
        if core.zero_check_persisting == Some(job.action_id) {
            core.zero_check_persisting = None;
            let id = job.action_id;
            const SAT_MSG: &str = "Persistence queue saturated";
            if can_offload_msg {
                if let Some(buf) = self.sat_msg_buf.take() {
                    let (_, old_msg) = core.finish_zero_check_action_in_place(
                        id,
                        "failed",
                        "PERSIST_FAILED",
                        SAT_MSG,
                        None,
                        Some(buf),
                    );
                    if let Some(m) = old_msg {
                        if m.capacity() >= SAT_MSG.len() {
                            self.sat_msg_buf = Some(m);
                        } else {
                            returned_msg = Some(m);
                        }
                    }
                } else {
                    Self::fail_zero_check_inline_only(core, id, SAT_MSG);
                }
            } else {
                Self::fail_zero_check_inline_only(core, id, SAT_MSG);
            }
        }
        if let Some(m) = returned_msg {
            let _ = self.offload_reclaim_tx.push(OffloadReclaim::Buf(m));
        }
        match self.offload_reclaim_tx.push(OffloadReclaim::PersistJob(job, probe)) {
            Ok(()) => core.core.hub().wake(),
            Err(rtrb::PushError::Full(OffloadReclaim::PersistJob(job, probe))) => {
                debug_assert!(self.retained_persist_job.is_none());
                self.retained_persist_job = Some((job, probe));
            }
            Err(_) => unreachable!(),
        }
    }

    /// Marks a running zero-check action failed using only inline `HolderLabel`s (reusing an
    /// existing `String` buffer ONLY if its capacity already fits `msg` — zero heap operations).
    fn fail_zero_check_inline_only(core: &mut SupervisorInner, id: u64, msg: &str) {
        let now = core.now_s();
        if let Some(a) = core.last_action.as_mut().filter(|a| a.id == id && a.is_running()) {
            a.state = crate::model::HolderLabel::new("failed");
            a.code = Some(crate::model::HolderLabel::new("PERSIST_FAILED"));
            if let Some(existing) = a.message.as_mut() {
                if existing.capacity() >= msg.len() {
                    existing.clear();
                    existing.push_str(msg);
                }
            }
            a.finished_at = Some(now);
        }
    }

    /// Executes one optional positive-control heap probe in the release/wake tail, returning any
    /// `ParkedAlloc` bound to the caller's scope so its final `dealloc` runs at local destruction.
    fn run_tail_alloc_probe(&self, slot: &Option<Arc<Mutex<Option<TailAllocProbe>>>>) -> Option<ParkedAlloc> {
        let probe = slot.as_ref().and_then(|p| p.lock().ok().and_then(|mut g| g.take()))?;
        match probe {
            TailAllocProbe::AllocOnce { park } => {
                let a = ParkedAlloc::new_alloc(64);
                if let Ok(mut g) = park.lock() {
                    *g = Some(a);
                }
                None
            }
            TailAllocProbe::AllocZeroedOnce { park } => {
                let a = ParkedAlloc::new_alloc_zeroed(64);
                if let Ok(mut g) = park.lock() {
                    *g = Some(a);
                }
                None
            }
            TailAllocProbe::ReallocOnce { existing, park } => {
                if let Some(a) = existing {
                    let resized = a.realloc_to(128);
                    if let Ok(mut g) = park.lock() {
                        *g = Some(resized);
                    }
                }
                None
            }
            TailAllocProbe::FinalFreeLocal { local } => local,
            TailAllocProbe::AllocAndFinalFreeLocal { local, park } => {
                let a = ParkedAlloc::new_alloc(64);
                if let Ok(mut g) = park.lock() {
                    *g = Some(a);
                }
                local
            }
        }
    }

    fn take_reply_probe(&self) -> Option<Box<dyn Any + Send>> {
        self.hooks
            .reply_drop_probe
            .as_ref()
            .and_then(|p| p.lock().ok().and_then(|mut g| g.take()))
    }

    fn handle_persist_cmd(&mut self, core: &mut SupervisorInner, ctx: &super::ops::CommitCtx, cmd: PersistCmd) {
        match cmd {
            PersistCmd::Fail { action_id, code, message, unused_policy, unused_jaw } => {
                if core.zero_check_persisting == Some(action_id) {
                    core.zero_check_persisting = None;
                }
                let (old_code, old_msg) = core.fail_zero_check_action_owned(action_id, code, message);
                let probe = self.take_reply_probe();
                self.push_persist_reply_or_retain(
                    PersistReply::Done {
                        _returned_policy: unused_policy,
                        _returned_jaw: unused_jaw,
                        _returned_code: old_code,
                        _returned_msg: old_msg,
                        _returned_warning: None,
                        _returned_probe: probe,
                    },
                    core,
                );
            }
            PersistCmd::Reserve {
                action_id,
                base_revision,
                lease_owner,
                new_policy,
                jaw_opening,
                probe,
                code_buf,
                msg_buf,
            } => {
                let now = ctx.now;
                let still_persisting = core.zero_check_persisting == Some(action_id);
                let action_running = core.last_action.as_ref().is_some_and(|a| a.id == action_id && a.is_running());
                let lease_ok = match &lease_owner {
                    None => true,
                    Some((c, g)) => core.active_lease.as_ref().is_some_and(|l| {
                        l.acquired_at.elapsed() < core.lease_timeout
                            && l.generation == *g
                            && crate::model::HolderLabel::new(&l.client_id).as_str() == c.as_str()
                    }),
                };
                let rev_ok = core.gripper_calib_revision == base_revision;
                let urgent_pending = core.estop_pending()
                    || self.stop_requests.current() != self.stop_requests.applied();
                let stopped = core.safety.state().is_stopped() || urgent_pending;

                if !still_persisting || !action_running || !lease_ok || stopped {
                    if core.zero_check_persisting == Some(action_id) {
                        core.zero_check_persisting = None;
                    }
                    let (old_code, old_msg) = if action_running {
                        let (code, msg) = if stopped {
                            ("STOPPED", "Safety stop intervened before gripper calibration persisted")
                        } else if !lease_ok {
                            ("LEASE_LOST", "Control lease lost or expired before gripper calibration persisted")
                        } else {
                            ("CANCELLED", "Gripper zero check cancelled before calibration persisted")
                        };
                        core.finish_zero_check_action_in_place(
                            action_id,
                            "cancelled",
                            code,
                            msg,
                            Some(code_buf),
                            Some(msg_buf),
                        )
                    } else {
                        (Some(code_buf), Some(msg_buf))
                    };
                    self.push_persist_reply_or_retain(
                        PersistReply::Reserved {
                            reservation: None,
                            policy: Some(new_policy),
                            returned_jaw: jaw_opening,
                            _returned_code: old_code,
                            _returned_msg: old_msg,
                            _returned_probe: probe,
                        },
                        core,
                    );
                    return;
                }
                if !rev_ok {
                    if core.zero_check_persisting == Some(action_id) {
                        core.zero_check_persisting = None;
                    }
                    let (old_code, old_msg) = core.finish_zero_check_action_in_place(
                        action_id,
                        "failed",
                        "POLICY_REVISION_MISMATCH",
                        "Gripper calibration revision changed concurrently",
                        Some(code_buf),
                        Some(msg_buf),
                    );
                    self.push_persist_reply_or_retain(
                        PersistReply::Reserved {
                            reservation: None,
                            policy: Some(new_policy),
                            returned_jaw: jaw_opening,
                            _returned_code: old_code,
                            _returned_msg: old_msg,
                            _returned_probe: probe,
                        },
                        core,
                    );
                    return;
                }
                if core.driver_config_error.is_some() {
                    if core.zero_check_persisting == Some(action_id) {
                        core.zero_check_persisting = None;
                    }
                    let (old_code, old_msg) = core.finish_zero_check_action_in_place(
                        action_id,
                        "failed",
                        "INVALID_CALIBRATION",
                        "Driver limit configuration error prevents gripper calibration",
                        Some(code_buf),
                        Some(msg_buf),
                    );
                    self.push_persist_reply_or_retain(
                        PersistReply::Reserved {
                            reservation: None,
                            policy: Some(new_policy),
                            returned_jaw: jaw_opening,
                            _returned_code: old_code,
                            _returned_msg: old_msg,
                            _returned_probe: probe,
                        },
                        core,
                    );
                    return;
                }
                let auth = core.authority(ctx);
                self.next_persist_req += 1;
                let proposal = ConfigProposal {
                    domain: ConfigDomain::GripperCalibration,
                    expected_revision: base_revision,
                    candidate: CandidateId(self.next_persist_req),
                    stamp: Stamp {
                        arm: ArmId::LOCAL,
                        boot: ctx.boot,
                        request: RequestId(self.next_persist_req),
                        client: ClientId(0),
                        client_seq: self.next_persist_req,
                        authority: Authority { lease_generation: auth.lease_generation, ..auth },
                        deadline: now + Duration::from_secs(5),
                    },
                };
                match ConfigAuthority::reserve(core, &proposal, ctx) {
                    Reserved::Granted(reservation) => {
                        self.persist_granted = Some((action_id, ConfigDomain::GripperCalibration));
                        self.push_persist_reply_or_retain(
                            PersistReply::Reserved {
                                reservation: Some(reservation),
                                policy: Some(new_policy),
                                returned_jaw: jaw_opening,
                                _returned_code: Some(code_buf),
                                _returned_msg: Some(msg_buf),
                                _returned_probe: probe,
                            },
                            core,
                        );
                    }
                    Reserved::Refused { code, .. } => {
                        if core.zero_check_persisting == Some(action_id) {
                            core.zero_check_persisting = None;
                        }
                        let (old_code, old_msg) = core.finish_zero_check_action_in_place(
                            action_id,
                            "failed",
                            code,
                            "Failed to reserve gripper calibration domain",
                            Some(code_buf),
                            Some(msg_buf),
                        );
                        self.push_persist_reply_or_retain(
                            PersistReply::Reserved {
                                reservation: None,
                                policy: Some(new_policy),
                                returned_jaw: jaw_opening,
                                _returned_code: old_code,
                                _returned_msg: old_msg,
                                _returned_probe: probe,
                            },
                            core,
                        );
                    }
                }
            }
            PersistCmd::Abort { action_id, reservation, code, message, unused_policy, unused_jaw } => {
                self.persist_granted = None;
                let _ = ConfigAuthority::release(core, reservation);
                if core.zero_check_persisting == Some(action_id) {
                    core.zero_check_persisting = None;
                }
                let (old_code, old_msg) = core.fail_zero_check_action_owned(action_id, code, message);
                let probe = self.take_reply_probe();
                self.push_persist_reply_or_retain(
                    PersistReply::Done {
                        _returned_policy: unused_policy,
                        _returned_jaw: unused_jaw,
                        _returned_code: old_code,
                        _returned_msg: old_msg,
                        _returned_warning: None,
                        _returned_probe: probe,
                    },
                    core,
                );
            }
            PersistCmd::Activate {
                action_id,
                reservation,
                base_revision,
                lease_owner,
                offset_rad,
                new_policy,
                new_jaw,
                durability,
                warning,
                code_buf,
                msg_buf,
            } => {
                self.persist_granted = None;
                let simulated_err = self
                    .hooks
                    .fail_persisted_once
                    .as_ref()
                    .is_some_and(|f| f.swap(false, Ordering::AcqRel));
                let persisted_res = if simulated_err {
                    Err("SIMULATED_PERSISTED_ERR")
                } else {
                    ConfigAuthority::persisted(core, &reservation, durability)
                };
                if persisted_res.is_err() {
                    // Persistence #5: disk may already have been renamed — unconditionally force
                    // Reconciling (inhibiting motion even if another domain occupied the slot).
                    let active_rev = ConfigAuthority::revision(core, ConfigDomain::GripperCalibration);
                    core.core.config_mut().force_reconcile(ConfigDomain::GripperCalibration, active_rev);
                    if core.zero_check_persisting == Some(action_id) {
                        core.zero_check_persisting = None;
                    }
                    let (old_code, old_msg) = core.finish_zero_check_action_in_place(
                        action_id,
                        "failed",
                        "PERSIST_FAILED",
                        "Failed to record post-rename persistence state; reconciling required",
                        Some(code_buf),
                        Some(msg_buf),
                    );
                    let probe = self.take_reply_probe();
                    self.push_persist_reply_or_retain(
                        PersistReply::Done {
                            _returned_policy: Some(new_policy),
                            _returned_jaw: new_jaw,
                            _returned_code: old_code,
                            _returned_msg: old_msg,
                            _returned_warning: warning,
                            _returned_probe: probe,
                        },
                        core,
                    );
                    return;
                }

                let still_persisting = core.zero_check_persisting == Some(action_id);
                let action_running = core.last_action.as_ref().is_some_and(|a| a.id == action_id && a.is_running());
                let lease_ok = match &lease_owner {
                    None => true,
                    Some((c, g)) => core.active_lease.as_ref().is_some_and(|l| {
                        l.acquired_at.elapsed() < core.lease_timeout
                            && l.generation == *g
                            && crate::model::HolderLabel::new(&l.client_id).as_str() == c.as_str()
                    }),
                };
                let rev_ok = core.gripper_calib_revision == base_revision;
                let urgent_pending = core.estop_pending()
                    || self.stop_requests.current() != self.stop_requests.applied();
                let stopped = core.safety.state().is_stopped() || urgent_pending;

                if still_persisting && action_running && lease_ok && rev_ok && !stopped {
                    let (old_policy, old_jaw) = core.swap_gripper_calibration_in_place(offset_rad, new_policy, new_jaw);
                    if !matches!(ConfigAuthority::activate(core, reservation), Activated::Active { .. }) {
                        let active_rev = ConfigAuthority::revision(core, ConfigDomain::GripperCalibration);
                        core.core.config_mut().force_reconcile(ConfigDomain::GripperCalibration, active_rev);
                    }
                    if core.zero_check_persisting == Some(action_id) {
                        core.zero_check_persisting = None;
                    }
                    let unused_warning = core.complete_zero_check_action(action_id, durability, warning);
                    // Return unused scratch strings + any displaced warning/policy/jaw off-thread.
                    let probe = self.take_reply_probe();
                    self.push_persist_reply_or_retain(
                        PersistReply::Done {
                            _returned_policy: old_policy,
                            _returned_jaw: old_jaw,
                            _returned_code: Some(code_buf),
                            _returned_msg: Some(msg_buf),
                            _returned_warning: unused_warning,
                            _returned_probe: probe,
                        },
                        core,
                    );
                } else {
                    // Post-rename revocation: keep live state/revision unchanged and enter Reconciling.
                    let code = if stopped {
                        "STOPPED"
                    } else if !lease_ok {
                        "LEASE_LOST"
                    } else if !rev_ok {
                        "POLICY_REVISION_MISMATCH"
                    } else {
                        "CANCELLED"
                    };
                    if !matches!(core.core.config_mut().activation_failed(reservation, code), Activated::Reconciling { .. }) {
                        let active_rev = ConfigAuthority::revision(core, ConfigDomain::GripperCalibration);
                        core.core.config_mut().force_reconcile(ConfigDomain::GripperCalibration, active_rev);
                    }
                    if core.zero_check_persisting == Some(action_id) {
                        core.zero_check_persisting = None;
                    }
                    let (old_code, old_msg) = if action_running {
                        core.finish_zero_check_action_in_place(
                            action_id,
                            "cancelled",
                            code,
                            "Gripper calibration revoked after disk rename; reconciliation required",
                            Some(code_buf),
                            Some(msg_buf),
                        )
                    } else {
                        (Some(code_buf), Some(msg_buf))
                    };
                    let probe = self.take_reply_probe();
                    self.push_persist_reply_or_retain(
                        PersistReply::Done {
                            _returned_policy: Some(new_policy),
                            _returned_jaw: new_jaw,
                            _returned_code: old_code,
                            _returned_msg: old_msg,
                            _returned_warning: warning,
                            _returned_probe: probe,
                        },
                        core,
                    );
                }
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn spawn_persist_worker(
    mut job_rx: Consumer<PendingZeroCheckPersist>,
    mut cmd_tx: Producer<PersistCmd>,
    rep_rx: Arc<Mutex<Consumer<PersistReply>>>,
    config_write_mutex: Arc<tokio::sync::Mutex<()>>,
    persist_stop: Arc<AtomicBool>,
    control_exited: Arc<AtomicBool>,
    persist_worker_exited: Arc<AtomicBool>,
    persist_exited_cv: Arc<(Mutex<bool>, Condvar)>,
    hub: Arc<StateHub>,
    hooks: PersistTestHooks,
) -> std::io::Result<JoinHandle<()>> {
    struct ExitFlag(Arc<AtomicBool>, Arc<(Mutex<bool>, Condvar)>, Arc<StateHub>);
    impl Drop for ExitFlag {
        fn drop(&mut self) {
            self.0.store(true, Ordering::Release);
            *self.1 .0.lock().unwrap_or_else(|e| e.into_inner()) = true;
            self.1 .1.notify_all();
            self.2.wake();
        }
    }

    thread::Builder::new().name("armd-persist".into()).spawn(move || {
        let _exit_guard = ExitFlag(persist_worker_exited, persist_exited_cv, hub);
        let push_cmd = |tx: &mut Producer<PersistCmd>, cmd: PersistCmd, stop: &AtomicBool, dead: &AtomicBool| -> bool {
            let mut pending = Some(cmd);
            while let Some(c) = pending.take() {
                if stop.load(Ordering::Acquire) || dead.load(Ordering::Acquire) || tx.is_abandoned() {
                    return false;
                }
                match tx.push(c) {
                    Ok(()) => return true,
                    Err(rtrb::PushError::Full(c)) => {
                        pending = Some(c);
                        thread::park_timeout(Duration::from_millis(1));
                    }
                }
            }
            true
        };
        let wait_reply = |rx: &Arc<Mutex<Consumer<PersistReply>>>, stop: &AtomicBool, dead: &AtomicBool| -> Option<PersistReply> {
            loop {
                if let Ok(rep) = rx.lock().unwrap_or_else(|e| e.into_inner()).pop() {
                    return Some(rep);
                }
                if stop.load(Ordering::Acquire) || dead.load(Ordering::Acquire) {
                    return None;
                }
                thread::park_timeout(Duration::from_millis(1));
            }
        };

        while !persist_stop.load(Ordering::Acquire) && !control_exited.load(Ordering::Acquire) {
            if hooks.pause_persist_worker.as_ref().is_some_and(|p| p.load(Ordering::Acquire)) {
                thread::park_timeout(Duration::from_millis(2));
                continue;
            }
            let Ok(job) = job_rx.pop() else {
                thread::park_timeout(Duration::from_millis(2));
                continue;
            };
            // Stop-aware config write lock acquisition: teardown unparks this thread and sets
            // `persist_stop`, preventing deadlock even if the caller holds `config_write_mutex`.
            let _cw = loop {
                if persist_stop.load(Ordering::Acquire) || control_exited.load(Ordering::Acquire) {
                    return;
                }
                match config_write_mutex.try_lock() {
                    Ok(guard) => break guard,
                    Err(_) => thread::park_timeout(Duration::from_millis(2)),
                }
            };
            let auth_ok = job
                .operator_auth
                .as_ref()
                .is_none_or(|(a, tok)| !a.is_enforced() || a.is_operator(tok.as_deref()));
            if !auth_ok {
                let _ = push_cmd(
                    &mut cmd_tx,
                    PersistCmd::Fail {
                        action_id: job.action_id,
                        code: "OPERATOR_REQUIRED".into(),
                        message: "Operator session expired or locked before gripper calibration persisted".into(),
                        unused_policy: None,
                        unused_jaw: job.jaw_opening,
                    },
                    &persist_stop,
                    &control_exited,
                );
                let _ = wait_reply(&rep_rx, &persist_stop, &control_exited);
                continue;
            }
            // Validate & build candidate GripperPolicy completely off `armd-control`.
            let candidate_policy = match SupervisorInner::validate_gripper_calibration_off_thread(
                job.base_gripper_policy.as_ref(),
                job.current_calib_jaw.as_ref(),
                job.gripper_limit.as_ref(),
                job.active_tool_is_stock,
                Some(job.offset_rad),
                job.jaw_opening.as_ref(),
            ) {
                Ok(p) => p,
                Err(e) => {
                    let _ = push_cmd(
                        &mut cmd_tx,
                        PersistCmd::Fail {
                            action_id: job.action_id,
                            code: "INVALID_CALIBRATION".into(),
                            message: e,
                            unused_policy: None,
                            unused_jaw: job.jaw_opening,
                        },
                        &persist_stop,
                        &control_exited,
                    );
                    let _ = wait_reply(&rep_rx, &persist_stop, &control_exited);
                    if hooks.exit_after_first_job { break; }
                    continue;
                }
            };
            let staged = match StagedWrite::stage(job.write.path.clone(), job.write.text.clone()) {
                Ok(s) => s,
                Err(e) => {
                    let _ = push_cmd(
                        &mut cmd_tx,
                        PersistCmd::Fail {
                            action_id: job.action_id,
                            code: "PERSIST_FAILED".into(),
                            message: format!("Failed to stage gripper calibration: {e}"),
                            unused_policy: Some(candidate_policy),
                            unused_jaw: job.jaw_opening,
                        },
                        &persist_stop,
                        &control_exited,
                    );
                    let _ = wait_reply(&rep_rx, &persist_stop, &control_exited);
                    continue;
                }
            };
            let lease_owner_inline = job
                .lease_owner
                .as_ref()
                .map(|(c, g)| (crate::model::HolderLabel::new(c), *g));
            let payload_probe = hooks
                .payload_drop_probe
                .as_ref()
                .and_then(|p| p.lock().ok().and_then(|mut g| g.take()));
            if !push_cmd(
                &mut cmd_tx,
                PersistCmd::Reserve {
                    action_id: job.action_id,
                    base_revision: job.base_revision,
                    lease_owner: lease_owner_inline,
                    new_policy: candidate_policy,
                    jaw_opening: job.jaw_opening,
                    probe: payload_probe,
                    code_buf: String::with_capacity(64),
                    msg_buf: String::with_capacity(160),
                },
                &persist_stop,
                &control_exited,
            ) {
                staged.abort();
                break;
            }
            let Some(PersistReply::Reserved {
                reservation,
                policy,
                returned_jaw,
                _returned_code,
                _returned_msg,
                _returned_probe,
            }) = wait_reply(&rep_rx, &persist_stop, &control_exited)
            else {
                staged.abort();
                break;
            };
            drop(_returned_code);
            drop(_returned_msg);
            drop(_returned_probe);
            let (reservation, new_policy, new_jaw) = match (reservation, policy) {
                (Some(reservation), Some(new_policy)) => (reservation, new_policy, returned_jaw),
                (_, returned_policy) => {
                    drop(returned_policy);
                    drop(returned_jaw);
                    staged.abort();
                    if hooks.exit_after_first_job { break; }
                    continue;
                }
            };
            if hooks.after_grant_before_rename.as_ref().is_some_and(|h| h()) {
                // Simulated worker crash/disconnect before rename (reservation dropped with worker).
                staged.abort();
                break;
            }
            // Re-verify operator authorization immediately before rename (operator session may
            // have expired or locked during staging or reservation wait).
            let auth_still_ok = job
                .operator_auth
                .as_ref()
                .is_none_or(|(a, tok)| !a.is_enforced() || a.is_operator(tok.as_deref()));
            if !auth_still_ok {
                staged.abort();
                let _ = push_cmd(
                    &mut cmd_tx,
                    PersistCmd::Abort {
                        action_id: job.action_id,
                        reservation,
                        code: "OPERATOR_REQUIRED".into(),
                        message: "Operator session expired or locked before gripper calibration persisted".into(),
                        unused_policy: Some(new_policy),
                        unused_jaw: new_jaw,
                    },
                    &persist_stop,
                    &control_exited,
                );
                if !hooks.leave_final_reply_queued {
                    let _ = wait_reply(&rep_rx, &persist_stop, &control_exited);
                }
                if hooks.exit_after_first_job { break; }
                continue;
            }
            let dir_sync = match staged.rename_only() {
                Ok(ds) => ds,
                Err(e) => {
                    let _ = push_cmd(
                        &mut cmd_tx,
                        PersistCmd::Abort {
                            action_id: job.action_id,
                            reservation,
                            code: "PERSIST_FAILED".into(),
                            message: format!("Failed to rename gripper calibration: {e}"),
                            unused_policy: Some(new_policy),
                            unused_jaw: new_jaw,
                        },
                        &persist_stop,
                        &control_exited,
                    );
                    if !hooks.leave_final_reply_queued {
                        let _ = wait_reply(&rep_rx, &persist_stop, &control_exited);
                    }
                    if hooks.exit_after_first_job { break; }
                    continue;
                }
            };
            let outcome = match hooks.dir_sync_override.as_ref() {
                Some(f) => dir_sync.sync_with(|dir| f(dir)),
                None => dir_sync.sync_parent_dir(),
            };
            let (durability, warning) = match outcome {
                PublishOutcome::Published => (Durability::Durable, None),
                PublishOutcome::PublishedDurabilityUnconfirmed(w) => (Durability::DurableUnconfirmed, Some(w)),
            };
            if hooks.after_rename_before_activate.as_ref().is_some_and(|h| h()) {
                // Simulated worker crash/disconnect after rename without pushing Activate.
                break;
            }
            let _ = push_cmd(
                &mut cmd_tx,
                PersistCmd::Activate {
                    action_id: job.action_id,
                    reservation,
                    base_revision: job.base_revision,
                    lease_owner: lease_owner_inline,
                    offset_rad: job.offset_rad,
                    new_policy,
                    new_jaw,
                    durability,
                    warning,
                    code_buf: String::with_capacity(64),
                    msg_buf: String::with_capacity(160),
                },
                &persist_stop,
                &control_exited,
            );
            if !hooks.leave_final_reply_queued {
                let _ = wait_reply(&rep_rx, &persist_stop, &control_exited);
            }
            if hooks.exit_after_first_job {
                break;
            }
        }
    })
}

/// RAII allocation-counting window for one whole release or one sub-release wake.
///
/// Declared FIRST in its lexical scope so Rust drops it LAST — strictly after all locals
/// (including `TickSample`, `ReleaseGuard`, reply/ack drains, and tail probes) are destroyed —
/// and updates `ExecutorStats`'s cumulative counters outside the droppable diagnostic ring before
/// Release-incrementing `completed_releases` / `completed_wakes`.
struct WholeReleaseAllocGuard<'a> {
    stats: &'a ExecutorStats,
    start: crate::diagnostics::alloc::AllocCounts,
    is_wake: bool,
}

impl<'a> WholeReleaseAllocGuard<'a> {
    #[inline]
    fn enter(stats: &'a ExecutorStats, is_wake: bool) -> Self {
        Self {
            stats,
            start: crate::diagnostics::alloc::snapshot_counts(),
            is_wake,
        }
    }
}

impl Drop for WholeReleaseAllocGuard<'_> {
    #[inline]
    fn drop(&mut self) {
        let delta = crate::diagnostics::alloc::snapshot_counts().delta_since(self.start);
        if self.is_wake {
            if delta.total() > 0 {
                self.stats.wake_allocs.record(delta);
                self.stats.wake_alloc_calls.fetch_add(delta.total(), Ordering::AcqRel);
            }
            self.stats.completed_wakes.fetch_add(1, Ordering::Release);
        } else {
            if delta.total() > 0 {
                self.stats.release_allocs.record(delta);
            }
            self.stats.completed_releases.fetch_add(1, Ordering::Release);
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn control_thread_main(
    mut core: SupervisorInner,
    engine: ReleaseEngine,
    cfg: ExecutorConfig,
    initial_generation: u64,
    ready_tx: mpsc::SyncSender<Result<ThreadId, (Option<Box<SupervisorInner>>, String)>>,
    control_stop: Arc<AtomicBool>,
    exited: Arc<AtomicBool>,
    control_exited_cv: Arc<(Mutex<bool>, Condvar)>,
    teardown_disable: Arc<TeardownDisable>,
    stats: Arc<ExecutorStats>,
    control_sched: Arc<Mutex<Option<crate::diagnostics::tick::ControlThreadSched>>>,
    control_period_us: Arc<AtomicU64>,
    mut diag_tx: Producer<TickSample>,
    maint_req_rx: Consumer<MaintRequest>,
    maint_rep_tx: Producer<MaintReply>,
    estop_ack_tx: Producer<EstopAck>,
    estop_overflow_gen: Arc<AtomicU64>,
    reclaim_state_tx: Producer<Arc<SafetyState>>,
    reclaim_return_rx: Consumer<Arc<SafetyState>>,
    safety_log_tx: Producer<SafetyLogEvent>,
    persist_job_tx: Producer<PendingZeroCheckPersist>,
    persist_cmd_rx: Consumer<PersistCmd>,
    persist_rep_tx: Producer<PersistReply>,
    offload_reclaim_tx: Producer<OffloadReclaim>,
    persist_wake: Thread,
    e_stop_latched: Arc<AtomicBool>,
    hub: Arc<StateHub>,
    hooks: PersistTestHooks,
) {
    if let Some(return_core) = hooks.fail_init {
        if return_core {
            let _ = ready_tx.send(Err((Some(Box::new(core)), "simulated executor init failure (recoverable)".into())));
        } else {
            drop(core);
            let _ = ready_tx.send(Err((None, "simulated executor init failure (core lost)".into())));
        }
        exited.store(true, Ordering::Release);
        *control_exited_cv.0.lock().unwrap_or_else(|e| e.into_inner()) = true;
        control_exited_cv.1.notify_all();
        hub.wake();
        return;
    }

    // Complete driver RT mode handoff (including any startup transport writer synchronization)
    // and nonblocking observation-slot prewarming BEFORE applying SCHED_FIFO so normal-priority
    // helper threads (e.g. `armd-sim-rx`) can never suffer priority inversion during startup and
    // release #1 allocates nothing even under stalled readers.
    core.rt_executor_owned = true;
    core.before_publish_hook = hooks.before_publish.clone();
    core.safety.set_off_thread_reclaim(true);
    if let Some(ref d) = core.driver {
        if let Ok(mut drv) = d.lock() {
            drv.set_rt_mode(true);
        }
    }
    core.prewarm_observation_slots();

    if cfg.apply_rt_sched {
        let s = sched::apply_to_current_thread(&cfg.rt_sched, &cfg.memory_lock);
        if s.realtime {
            tracing::info!(
                "RT executor running at SCHED_FIFO priority {}{}",
                s.priority,
                s.cpu.map(|c| format!(" on CPU {c}")).unwrap_or_default()
            );
        } else {
            tracing::warn!(
                "RT executor real-time scheduling unavailable ({}); using normal scheduling, MIT tracking (when configured) refused",
                s.errors.join("; ")
            );
        }
        sched::set_readiness(RtReadiness::from_sched(&s));
        *control_sched.lock().unwrap_or_else(|e| e.into_inner()) = Some(s);
    } else {
        let s = sched::observe_current_thread(&cfg.memory_lock);
        *control_sched.lock().unwrap_or_else(|e| e.into_inner()) = Some(s);
    }
    if cfg.alloc_timing {
        crate::diagnostics::alloc::time_this_thread();
    }

    let mut engine = engine;
    let stop_requests = engine.stop_requests().clone();
    let mut state = ControlState {
        shutdown_committed: false,
        control_stop: Arc::clone(&control_stop),
        stop_requests,
        estop_ack_tx,
        estop_overflow_gen,
        reclaim_state_tx,
        reclaim_return_rx,
        safety_log_tx,
        stats: Arc::clone(&stats),
        maint_req_rx,
        maint_rep_tx,
        pending_maint_reply: None,
        persist_job_tx,
        persist_cmd_rx,
        persist_rep_tx,
        pending_persist_reply: None,
        retained_persist_job: None,
        offload_reclaim_tx,
        persist_wake,
        persist_granted: None,
        next_persist_req: 0,
        disconnect_code_buf: Some(String::with_capacity(64)),
        disconnect_msg_buf: Some(String::with_capacity(160)),
        sat_msg_buf: Some(String::with_capacity(160)),
        hooks,
    };

    if ready_tx.send(Ok(thread::current().id())).is_err() {
        exited.store(true, Ordering::Release);
        *control_exited_cv.0.lock().unwrap_or_else(|e| e.into_inner()) = true;
        control_exited_cv.1.notify_all();
        hub.wake();
        return;
    }

    let clock_hook = state.hooks.clock.clone();
    let now_inst = || -> Instant {
        clock_hook.as_ref().map_or_else(Instant::now, |c| c.now())
    };
    let now_ns_fn = || -> u64 {
        clock_hook.as_ref().map_or_else(mono_now_ns, |c| c.now_ns())
    };
    let sleep_until = |target_ns: u64| {
        if let Some(c) = clock_hook.as_ref() {
            c.sleep_until_ns(target_ns, &control_stop);
        } else {
            sleep_until_ns(target_ns);
        }
    };
    let sleep_dur = |d: Duration| {
        if let Some(c) = clock_hook.as_ref() {
            c.sleep(d, &control_stop);
        } else {
            thread::sleep(d);
        }
    };

    let mut releases = Releases::start_at(now_ns_fn(), cfg.period);
    let mut last_release_start = now_inst();
    let mut last_release_end_ns: Option<u64> = None;
    let mut publish_gen = initial_generation;
    let mut panicked = false;

    // Compile-time check: only `Copy` state crosses from Window 1 (whole release) into Window 2
    // (sub-release wakes), ensuring zero uncounted destruction across the boundary.
    const _: () = {
        const fn assert_copy<T: Copy>() {}
        assert_copy::<(super::ops::ReleaseReport, Instant, u64)>();
    };

    while !control_stop.load(Ordering::Acquire) {
        let scheduled_ns = releases.next_ns();
        sleep_until(scheduled_ns);
        if control_stop.load(Ordering::Acquire) {
            break;
        }

        // Sample actual wake time FIRST — right after sleep returns and stop check,
        // strictly before allocation bookkeeping (`WholeReleaseAllocGuard`) and `UsageMark::now()`
        // (matching `run_periodic_with_clock`).
        let actual_ns = now_ns_fn();
        let release_start = now_inst();
        let wake_lateness_ns = actual_ns.saturating_sub(scheduled_ns);

        // ---- Window 1: Whole release (disjoint from sub-release wakes) ----
        let release_res = {
            // Declared FIRST -> drops LAST, after catch_unwind and ALL release locals drop!
            let _alloc_scope = WholeReleaseAllocGuard::enter(&stats, false);
            let wake_lateness_us = stats.record_wake_lateness_ns(wake_lateness_ns);
            let usage_start = UsageMark::now();
            let skipped = releases.advance(actual_ns);
            if skipped > 0 {
                stats.skipped_releases.fetch_add(skipped, Ordering::Relaxed);
            }
            let elapsed = release_start.saturating_duration_since(last_release_start);
            last_release_start = release_start;
            if elapsed > LATE_LOG_AFTER {
                stats.last_late_us.store(elapsed.as_micros() as u64, Ordering::Relaxed);
                stats.late.fetch_add(1, Ordering::Release);
            }
            let step_num = stats.steps.fetch_add(1, Ordering::Relaxed) + 1;
            let dt = elapsed.as_secs_f64();
            let next_release_ns = releases.next_ns();
            let remaining_ns = next_release_ns.saturating_sub(now_ns_fn());
            let release_deadline = now_inst() + Duration::from_nanos(remaining_ns);
            publish_gen += 1;

            std::panic::catch_unwind(std::panic::AssertUnwindSafe(
                || -> Option<(super::ops::ReleaseReport, Instant, u64)> {
                    let _guard = ReleaseGuard::enter();
                    assert!(
                        tokio::runtime::Handle::try_current().is_err(),
                        "RT executor thread must never enter a Tokio runtime during a release"
                    );
                    if state.hooks.panic_on_release == Some(step_num) {
                        panic!("simulated RT executor panic on release {step_num}");
                    }
                    if let Some(delay) = state
                        .hooks
                        .release_delay_once
                        .as_ref()
                        .and_then(|d| d.lock().ok().and_then(|mut g| g.take()))
                    {
                        sleep_dur(delay);
                    }
                    let _ = crate::diagnostics::alloc::take();
                    let body_start = now_inst();
                    let latch_count_before = core.safety.latch_count();
                    let kinds_before = core.safety.latched_fault_kinds();
                    let estop_applied_before = core.applied_estop_count;
                    let mut release_flags: u32 = 0;
                    let mut maint_req_id: u64 = 0;
                    core.defer_zero_check_persist = true;
                    core.persist_capacity_available = !state.persist_job_tx.is_abandoned()
                        && state.persist_job_tx.slots() > 0
                        && !state.offload_reclaim_tx.is_full()
                        && state.retained_persist_job.is_none();
                    let mut injected_job_probe: Option<Box<dyn Any + Send>> = None;
                    if core.pending_zero_check_persist.is_none()
                        && state.retained_persist_job.is_none()
                    {
                        if let Some((injected, probe)) = state
                            .hooks
                            .inject_pending_persist_job_once
                            .as_ref()
                            .and_then(|slot| slot.lock().ok().and_then(|mut g| g.take()))
                        {
                            if core.zero_check_persisting.is_none()
                                && core
                                    .last_action
                                    .as_ref()
                                    .is_some_and(|a| a.id == injected.action_id && a.is_running())
                            {
                                core.zero_check_persisting = Some(injected.action_id);
                            }
                            core.pending_zero_check_persist = Some(injected);
                            injected_job_probe = probe;
                        }
                    }
                    core.begin_tick_phases(body_start);

                    let report = engine.step_with_pre_commit(
                        &mut core,
                        now_inst,
                        dt,
                        release_deadline,
                        publish_gen,
                        |core, ctx| {
                            // Run persistence reconciliation and at most one bounded maintenance op
                            // strictly AFTER Step 2 fresh feedback intake and strictly BEFORE Step 3
                            // commit, Step 5 output, and Step 6 publication. Hold E-stop acknowledgements
                            // in `core.pending_estop_acks` (keeping their `Arc<SafetyState>` alive) and
                            // deliver them ONLY after Step 6 publishes the corresponding snapshot.
                            state.service_persistence(core, ctx);
                            state.service_one_maintenance(
                                core,
                                ctx,
                                release_deadline,
                                step_num,
                                &mut release_flags,
                                &mut maint_req_id,
                            );
                            control_stop.load(Ordering::Acquire)
                        },
                    );
                    if control_stop.load(Ordering::Acquire) {
                        if let Some(p) = injected_job_probe.take() {
                            if state.offload_reclaim_tx.push(OffloadReclaim::Probe(p)).is_ok() {
                                core.core.hub().wake();
                            }
                        }
                        state.flush_pending_maint_reply(&core);
                        state.emit_estop_ack_if_advanced(&mut core);
                        state.drain_retired_states(&mut core);
                        return None;
                    }
                    let (phases, body_end) = core.take_tick_phases();
                    let body_end = body_end.unwrap_or_else(now_inst);
                    core.defer_zero_check_persist = false;
                    if let Some(job) = core.pending_zero_check_persist.take() {
                        match state.persist_job_tx.push(job) {
                            Ok(()) => {
                                state.persist_wake.unpark();
                                if let Some(p) = injected_job_probe.take() {
                                    if state.offload_reclaim_tx.push(OffloadReclaim::Probe(p)).is_ok() {
                                        core.core.hub().wake();
                                    }
                                }
                            }
                            Err(rtrb::PushError::Full(job)) => {
                                state.reject_saturated_persist_job(&mut core, job, injected_job_probe.take());
                            }
                        }
                    }
                    if let Some(zc) = core.pending_legacy_j6_log.take() {
                        if state.offload_reclaim_tx.push(OffloadReclaim::LogLegacyJ6(zc)).is_ok() {
                            core.core.hub().wake();
                        }
                    }
                    // Deliver the pre-commit maintenance reply strictly AFTER Step 6 publication, then
                    // drain any E-stop acks / retired states produced during Steps 3-7.
                    state.flush_pending_maint_reply(&core);
                    state.emit_estop_ack_if_advanced(&mut core);
                    state.drain_retired_states(&mut core);
                    if let Some((armed, slot)) = state.hooks.inject_release_alloc_once.as_deref() {
                        if armed.swap(false, Ordering::AcqRel) {
                            let ptr = Box::into_raw(Box::new(0xB3_u64));
                            slot.store(ptr, Ordering::Release);
                            release_flags |= RELEASE_FLAG_INJECT_ALLOC;
                            stats.last_inject_alloc_release_seq.store(step_num, Ordering::Release);
                        }
                    }
                    let kinds_after = core.safety.latched_fault_kinds();
                    let newly_set_kinds = kinds_after & !kinds_before;
                    if core.safety.latch_count() != latch_count_before || newly_set_kinds != 0 {
                        release_flags |= RELEASE_FLAG_SAFETY_LATCH;
                        if (newly_set_kinds & FaultKind::ThermalTrip.bit()) != 0 {
                            release_flags |= RELEASE_FLAG_THERMAL_TRIP;
                        }
                        if (newly_set_kinds & FaultKind::WatchdogTimeout.bit()) != 0 {
                            release_flags |= RELEASE_FLAG_WATCHDOG_LATCH;
                        }
                        stats.last_fault_latch_release_seq.store(step_num, Ordering::Release);
                    }
                    if core.applied_estop_count > estop_applied_before {
                        release_flags |= RELEASE_FLAG_ESTOP_APPLIED;
                        stats.last_estop_release_seq.store(step_num, Ordering::Release);
                    }
                    let control_hz = core.config.control_frequency_hz;
                    let done = now_inst();
                    let allocs = crate::diagnostics::alloc::take();
                    let usage_done = UsageMark::now();
                    control_period_us.store((1e6 / f64::from(control_hz.max(1))) as u64, Ordering::Relaxed);

                    let sample = TickSample {
                        tick_us: crate::diagnostics::tick::us(done.saturating_duration_since(release_start)),
                        wait_us: 0,
                        lock_us: crate::diagnostics::tick::us(done.saturating_duration_since(body_start)),
                        post_us: 0,
                        period_us: if dt.is_finite() && dt >= 0.0 {
                            crate::diagnostics::tick::us(Duration::from_secs_f64(dt.min(4.0e3)))
                        } else {
                            0
                        },
                        wake_lateness_us,
                        reader_hold_us: 0,
                        writer_hold_us: 0,
                        reader_acquire_us: 0,
                        writer_acquire_us: 0,
                        stats_wait_us: 0,
                        phases: LockPhases {
                            pre_us: crate::diagnostics::tick::us(body_start.saturating_duration_since(release_start)),
                            rx_us: phases[0],
                            control_us: phases[1],
                            tx_us: phases[2],
                            finish_us: phases[3],
                            obs_us: phases[4],
                            publish_us: crate::diagnostics::tick::us(done.saturating_duration_since(body_end)),
                        },
                        usage: ThreadUsage {
                            lock_alloc_calls: allocs.calls.min(u64::from(u32::MAX)) as u32,
                            lock_alloc_us: (allocs.total_ns / 1000).min(u64::from(u32::MAX)) as u32,
                            lock_alloc_max_us: (allocs.max_ns / 1000).min(u64::from(u32::MAX)) as u32,
                            ..crate::diagnostics::tick::thread_usage(&usage_start, &usage_start, &usage_done)
                        },
                        waited_for: None,
                        longest_hold: None,
                        late: !dt.is_finite() || dt > 2.0 / f64::from(control_hz.max(1)),
                        release_seq: step_num,
                        safety_epoch: core.safety.epoch_id(),
                        maint_req_id,
                        release_flags,
                    };
                    if diag_tx.push(sample).is_err() {
                        stats.diag_dropped.fetch_add(1, Ordering::Relaxed);
                    }
                    hub.wake();

                    // Positive-control tail probe (strictly AFTER `diag_tx.push` and `hub.wake()`);
                    // `_tail_local` drops at release-scope exit BEFORE `_alloc_scope` drops.
                    let _tail_local = state.run_tail_alloc_probe(&state.hooks.inject_release_tail_once);
                    let next_wake = report.poll.wake_before(release_deadline);
                    drop(_guard);
                    next_wake.map(|_| (report, release_deadline, next_release_ns))
                },
            ))
        };

        // ---- Window 2: Disjoint sub-release transition wakes before `next_release_ns` ----
        let step_res: Result<(), Box<dyn Any + Send>> = match release_res {
            Err(err) => Err(err),
            Ok(None) => Ok(()),
            Ok(Some((mut report, release_deadline, next_release_ns))) => {
                let mut wakes = 0u32;
                let mut next_wake = report.poll.wake_before(release_deadline);
                let mut wake_err: Option<Box<dyn Any + Send>> = None;
                while let Some(wake) = next_wake {
                    if wakes >= MAX_SERVICE_WAKES || control_stop.load(Ordering::Acquire) {
                        break;
                    }
                    let now = now_inst();
                    if wake > now {
                        let wake_ns = now_ns_fn().saturating_add(
                            wake.saturating_duration_since(now)
                                .as_nanos()
                                .min(u128::from(u64::MAX)) as u64,
                        );
                        sleep_until(wake_ns.min(next_release_ns));
                    }
                    if control_stop.load(Ordering::Acquire) {
                        break;
                    }
                    wakes += 1;
                    let wake_res = {
                        // Declared FIRST -> drops LAST, after ALL wake locals drop!
                        let _wake_alloc_scope = WholeReleaseAllocGuard::enter(&stats, true);
                        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| -> Option<Instant> {
                            stats.wakes.fetch_add(1, Ordering::Relaxed);
                            let _wake_guard = ReleaseGuard::enter();
                            assert!(
                                tokio::runtime::Handle::try_current().is_err(),
                                "RT executor thread must never enter a Tokio runtime during a sub-release wake"
                            );
                            let published = engine.service_wake(
                                &mut core,
                                now_inst(),
                                release_deadline,
                                publish_gen + 1,
                                &mut report,
                            );
                            if published {
                                publish_gen += 1;
                                hub.wake();
                            }
                            if state
                                .hooks
                                .inject_wake_alloc_once
                                .as_ref()
                                .is_some_and(|f| f.swap(false, Ordering::AcqRel))
                            {
                                let v: Vec<u8> = vec![0xA5; 64];
                                std::hint::black_box(&v);
                                drop(v);
                            }
                            state.emit_estop_ack_if_advanced(&mut core);
                            state.drain_retired_states(&mut core);
                            let _wake_tail_local =
                                state.run_tail_alloc_probe(&state.hooks.inject_wake_tail_once);
                            report.poll.wake_before(release_deadline)
                        }))
                    };
                    match wake_res {
                        Ok(next) => next_wake = next,
                        Err(err) => {
                            wake_err = Some(err);
                            break;
                        }
                    }
                }
                match wake_err {
                    Some(err) => Err(err),
                    None => Ok(()),
                }
            }
        };

        let release_end_ns = now_ns_fn();
        let overrun_skipped = releases.advance_overrun(release_end_ns);
        if overrun_skipped > 0 {
            stats.skipped_releases.fetch_add(overrun_skipped, Ordering::Relaxed);
        }
        last_release_end_ns = Some(release_end_ns);

        if let Err(err) = step_res {
            if let Some(end_ns) = last_release_end_ns.take() {
                let final_skipped = releases.missed_at_exit(end_ns);
                if final_skipped > 0 {
                    stats.skipped_releases.fetch_add(final_skipped, Ordering::Relaxed);
                }
            }
            panicked = true;
            e_stop_latched.store(true, Ordering::SeqCst);
            tracing::error!("RT executor thread panicked during release: {:?}", err);
            let disable_timeout = state
                .hooks
                .panic_disable_timeout
                .unwrap_or(crate::supervisor::PANIC_DISABLE_TIMEOUT);
            let policy = cfg.panic_policy;
            let td_obs = Arc::clone(&teardown_disable);
            let exited_obs = Arc::clone(&exited);
            let hub_obs = Arc::clone(&hub);
            let timeout_hook = state.hooks.on_panic_disable_timeout.clone();
            let completed = crate::supervisor::run_with_panic_deadline_observer(
                disable_timeout,
                || {
                    std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        core.attempt_panic_disable(disable_timeout)
                    }))
                    .unwrap_or_else(|_| {
                        crate::supervisor::PanicDisableOutcome::Failed(
                            "secondary panic during RT executor panic-disable".into(),
                        )
                    })
                },
                move || {
                    let timed_out = crate::supervisor::PanicDisableOutcome::TimedOut;
                    timed_out.log("RT executor panic", disable_timeout);
                    td_obs.set_outcome(timed_out);
                    match policy {
                        PanicPolicy::AbortProcess => {
                            crate::diagnostics::logwriter::flush_all(Duration::from_millis(200));
                            std::process::exit(1);
                        }
                        PanicPolicy::StopExecutor => {
                            // Do NOT mark `control_exited_cv` here: `armd-control` is still blocked
                            // inside the driver call and must be detached rather than joined.
                            exited_obs.store(true, Ordering::Release);
                            hub_obs.wake();
                            if let Some(hook) = timeout_hook {
                                hook();
                            }
                        }
                    }
                },
            );
            if let Some(outcome) = completed {
                outcome.log("RT executor panic", disable_timeout);
                teardown_disable.set_outcome(outcome);
                match cfg.panic_policy {
                    PanicPolicy::AbortProcess => {
                        crate::diagnostics::logwriter::flush_all(Duration::from_millis(500));
                        std::process::exit(1);
                    }
                    PanicPolicy::StopExecutor => {
                        break;
                    }
                }
            } else {
                // Deadline observer already handled timeout logging / exit signaling.
                break;
            }
        }
    }

    // Account any final release overrun that elapsed past `releases.next_ns()` right before loop exit.
    if let Some(end_ns) = last_release_end_ns.take() {
        let final_skipped = releases.missed_at_exit(end_ns);
        if final_skipped > 0 {
            stats.skipped_releases.fetch_add(final_skipped, Ordering::Relaxed);
        }
    }

    if !panicked && teardown_disable.requested.load(Ordering::Acquire) {
        let disable_timeout = state
            .hooks
            .panic_disable_timeout
            .unwrap_or(crate::supervisor::PANIC_DISABLE_TIMEOUT);
        let td_obs = Arc::clone(&teardown_disable);
        let completed = crate::supervisor::run_with_panic_deadline_observer(
            disable_timeout,
            || {
                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    core.attempt_panic_disable(disable_timeout)
                }))
                .unwrap_or_else(|_| {
                    crate::supervisor::PanicDisableOutcome::Failed(
                        "panic during emergency teardown disable".into(),
                    )
                })
            },
            move || {
                let timed_out = crate::supervisor::PanicDisableOutcome::TimedOut;
                timed_out.log("RT executor emergency shutdown", disable_timeout);
                td_obs.set_outcome(timed_out);
            },
        );
        if let Some(outcome) = completed {
            outcome.log("RT executor emergency shutdown", disable_timeout);
            teardown_disable.set_outcome(outcome);
        }
    }

    // Clean exit drain: return queued maintenance and persistence payloads off-thread without
    // performing unpublished live state mutations, publish before flushing any unacked E-stops,
    // and signal exit before waking sinks.
    state.drain_maintenance_stopped();
    state.drain_persistence_stopped(&core);
    if !core.pending_estop_acks.is_empty() {
        publish_gen += 1;
        let ts = core.now_timestamp();
        core.publish(ts, publish_gen);
    }
    state.drain_estop_and_retired_stopped(&mut core);
    exited.store(true, Ordering::Release);
    *control_exited_cv.0.lock().unwrap_or_else(|e| e.into_inner()) = true;
    control_exited_cv.1.notify_all();
    hub.wake();
}

/// Monotonic nanoseconds of `CLOCK_MONOTONIC` (the clock of `Instant` on Linux).
#[cfg(target_os = "linux")]
fn mono_now_ns() -> u64 {
    // SAFETY: `ts` is a valid out-pointer.
    let mut ts: libc::timespec = unsafe { std::mem::zeroed() };
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
}

/// Sleeps until the absolute monotonic time `at_ns` (returns at once when it has passed).
#[cfg(target_os = "linux")]
fn sleep_until_ns(at_ns: u64) {
    // SAFETY: timespec is plain old data; all-zero is valid
    let mut ts: libc::timespec = unsafe { std::mem::zeroed() };
    ts.tv_sec = (at_ns / 1_000_000_000) as _;
    ts.tv_nsec = (at_ns % 1_000_000_000) as _;
    loop {
        // SAFETY: `ts` is valid; the remaining-time pointer is unused with TIMER_ABSTIME.
        let rc = unsafe { libc::clock_nanosleep(libc::CLOCK_MONOTONIC, libc::TIMER_ABSTIME, &ts, std::ptr::null_mut()) };
        if rc != libc::EINTR {
            return;
        }
    }
}

#[cfg(not(target_os = "linux"))]
fn mono_now_ns() -> u64 {
    static START: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    START.get_or_init(Instant::now).elapsed().as_nanos() as u64
}

#[cfg(not(target_os = "linux"))]
fn sleep_until_ns(at_ns: u64) {
    let now = mono_now_ns();
    if at_ns > now {
        thread::sleep(Duration::from_nanos(at_ns - now));
    }
}

/// The release schedule: the next absolute release and the skip rule.
#[derive(Debug, Clone, Copy)]
pub struct Releases {
    period_ns: u64,
    next_ns: u64,
}

impl Releases {
    /// Inclusive count of grid points `first_missed_ns + k * period_ns` that are `<= now_ns`.
    #[inline]
    fn missed_since(first_missed_ns: u64, now_ns: u64, period_ns: u64) -> u64 {
        if now_ns >= first_missed_ns {
            (now_ns - first_missed_ns) / period_ns + 1
        } else {
            0
        }
    }

    /// First release one period from now.
    pub fn start(period: Duration) -> Self {
        Self::start_at(mono_now_ns(), period)
    }

    /// First release one period from an explicit starting monotonic timestamp (nanoseconds).
    pub fn start_at(now_ns: u64, period: Duration) -> Self {
        let period_ns = (period.as_nanos() as u64).max(1);
        Self { period_ns, next_ns: now_ns + period_ns }
    }

    /// Advances past a release at `now_ns`, preserving the original absolute release grid phase:
    /// the next release is the earliest grid point strictly after `now_ns` (`next_ns + k * period_ns`),
    /// and missed grid points are counted in the returned skip count.
    pub fn advance(&mut self, now_ns: u64) -> u64 {
        self.next_ns = self.next_ns.saturating_add(self.period_ns);
        let skipped = Self::missed_since(self.next_ns, now_ns, self.period_ns);
        self.next_ns = self.next_ns.saturating_add(skipped.saturating_mul(self.period_ns));
        skipped
    }

    /// Accounts intermediate deadlines strictly missed by an overrun during the release that
    /// finished at `end_ns`, advancing `next_ns` while keeping `next_ns <= end_ns` so an immediate
    /// catch-up release still runs if the loop continues without double-counting skips.
    pub fn advance_overrun(&mut self, end_ns: u64) -> u64 {
        let second_missed = self.next_ns.saturating_add(self.period_ns);
        let skipped = Self::missed_since(second_missed, end_ns, self.period_ns);
        self.next_ns = self.next_ns.saturating_add(skipped.saturating_mul(self.period_ns));
        skipped
    }

    /// Accounts all remaining scheduled deadlines at or before `end_ns` when the control loop
    /// exits after a release ending at `end_ns` (returns `0` when `end_ns < self.next_ns`).
    pub fn missed_at_exit(&self, end_ns: u64) -> u64 {
        Self::missed_since(self.next_ns, end_ns, self.period_ns)
    }

    pub fn next_ns(&self) -> u64 { self.next_ns }
}

thread_local! {
    /// Active `(next_ns, period_ns)` of an in-flight `run_periodic_with_clock` release on this
    /// thread so explicit panic-exit wrappers (e.g. `src/main.rs` before `process::exit`) and
    /// unwind guards account the final overrun exactly once.
    static ACTIVE_PERIODIC_GRID: std::cell::Cell<Option<(u64, u64)>> = const { std::cell::Cell::new(None) };
}

/// Accounts any final overrun skips for the calling thread's in-flight `run_periodic` release
/// immediately before a panic-exit handler calls `attempt_legacy_panic_disable` / `process::exit`.
/// Consumes the thread-local grid once so unwind paths cannot double-count.
pub fn account_periodic_overrun_on_panic_exit(stats: &ExecutorStats) {
    if let Some((next_ns, period_ns)) = ACTIVE_PERIODIC_GRID.try_with(|c| c.take()).ok().flatten() {
        let skipped = Releases { period_ns, next_ns }.missed_at_exit(mono_now_ns());
        if skipped > 0 {
            stats.skipped_releases.fetch_add(skipped, Ordering::Relaxed);
        }
    }
}

/// Runs `step(dt)` at every release until `stop` is set. `dt` is the real time since the
/// previous step's start (seconds). Allocation free.
pub fn run_periodic(period: Duration, stop: &AtomicBool, stats: &ExecutorStats, step: impl FnMut(f64)) {
    run_periodic_with_clock(period, stop, stats, None, step);
}

/// [`run_periodic`] with an optional deterministic [`ExecutorClock`] for tests.
pub fn run_periodic_with_clock(
    period: Duration,
    stop: &AtomicBool,
    stats: &ExecutorStats,
    clock: Option<&dyn ExecutorClock>,
    mut step: impl FnMut(f64),
) {
    let now_inst = || clock.map_or_else(Instant::now, ExecutorClock::now);
    let now_ns = || clock.map_or_else(mono_now_ns, ExecutorClock::now_ns);
    let mut releases = Releases::start_at(now_ns(), period);
    let mut last = now_inst();
    let mut last_release_end_ns: Option<u64> = None;
    while !stop.load(Ordering::Acquire) {
        let scheduled_ns = releases.next_ns();
        if let Some(c) = clock {
            c.sleep_until_ns(scheduled_ns, stop);
        } else {
            sleep_until_ns(scheduled_ns);
        }
        if stop.load(Ordering::Acquire) {
            break;
        }
        let actual_ns = now_ns();
        let woke = now_inst();
        let wake_lateness_ns = actual_ns.saturating_sub(scheduled_ns);
        let _ = stats.record_wake_lateness_ns(wake_lateness_ns);
        let skipped = releases.advance(actual_ns);
        if skipped > 0 { stats.skipped_releases.fetch_add(skipped, Ordering::Relaxed); }
        let elapsed = woke.saturating_duration_since(last);
        last = woke;
        if elapsed > LATE_LOG_AFTER {
            stats.last_late_us.store(elapsed.as_micros() as u64, Ordering::Relaxed);
            stats.late.fetch_add(1, Ordering::Release);
        }
        stats.steps.fetch_add(1, Ordering::Relaxed);
        let _lateness_guard = crate::diagnostics::tick::WakeLatenessGuard::enter(wake_lateness_ns);
        let _ = ACTIVE_PERIODIC_GRID.try_with(|c| c.set(Some((releases.next_ns, releases.period_ns))));
        struct PeriodicUnwindGridGuard<'a> {
            stats: &'a ExecutorStats,
            clock: Option<&'a dyn ExecutorClock>,
        }
        impl Drop for PeriodicUnwindGridGuard<'_> {
            fn drop(&mut self) {
                if let Some((next_ns, period_ns)) = ACTIVE_PERIODIC_GRID.try_with(|c| c.take()).ok().flatten() {
                    if std::thread::panicking() {
                        let end_ns = self.clock.map_or_else(mono_now_ns, ExecutorClock::now_ns);
                        let skipped = Releases { period_ns, next_ns }.missed_at_exit(end_ns);
                        if skipped > 0 {
                            self.stats.skipped_releases.fetch_add(skipped, Ordering::Relaxed);
                        }
                    }
                }
            }
        }
        let _unwind_grid = PeriodicUnwindGridGuard { stats, clock };
        step(elapsed.as_secs_f64());
        let _ = ACTIVE_PERIODIC_GRID.try_with(|c| c.take());
        drop(_unwind_grid);
        let end_ns = now_ns();
        let overrun_skipped = releases.advance_overrun(end_ns);
        if overrun_skipped > 0 {
            stats.skipped_releases.fetch_add(overrun_skipped, Ordering::Relaxed);
        }
        last_release_end_ns = Some(end_ns);
    }
    if let Some(end_ns) = last_release_end_ns.take() {
        let final_skipped = releases.missed_at_exit(end_ns);
        if final_skipped > 0 {
            stats.skipped_releases.fetch_add(final_skipped, Ordering::Relaxed);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn releases_are_absolute_and_skip_missed_ones() {
        let mut r = Releases { period_ns: 20, next_ns: 100 };
        assert_eq!(r.advance(101), 0);
        assert_eq!(r.next_ns(), 120);
        assert_eq!(r.advance(135), 0);
        assert_eq!(r.next_ns(), 140);
        assert_eq!(r.advance(190), 2);
        assert_eq!(r.next_ns(), 200);
        assert_eq!(r.advance(220), 1);
        assert_eq!(r.next_ns(), 240);
    }

    #[test]
    fn releases_overrun_and_exit_skip_accounting_is_exact_and_never_double_counts() {
        // Case 1: clean release (ends before next_ns) -> 0 overrun, 0 exit skips.
        let mut r = Releases { period_ns: 20, next_ns: 20 };
        assert_eq!(r.advance(20), 0); // next_ns = 40
        assert_eq!(r.advance_overrun(25), 0);
        assert_eq!(r.missed_at_exit(25), 0);

        // Case 2: 25ms release on 20ms grid (starts at 20, ends at 45) -> 0 intermediate, 1 at exit.
        let mut r = Releases { period_ns: 20, next_ns: 20 };
        assert_eq!(r.advance(20), 0); // next_ns = 40
        assert_eq!(r.advance_overrun(45), 0);
        assert_eq!(r.next_ns(), 40);
        assert_eq!(r.missed_at_exit(45), 1);
        // Exact boundary end_ns == next_ns also counts 1 at exit.
        assert_eq!(r.missed_at_exit(40), 1);
        assert_eq!(r.missed_at_exit(39), 0);

        // Case 3: 12ms release on 5ms grid (starts at 5, ends at 17):
        // intermediate skip (1) committed immediately at release end, +1 at exit (= 2 total: 10, 15).
        let mut r = Releases { period_ns: 5, next_ns: 5 };
        assert_eq!(r.advance(5), 0); // next_ns = 10
        assert_eq!(r.advance_overrun(17), 1); // 10 left for catch-up? next_ns advances to 15 <= 17
        assert_eq!(r.next_ns(), 15);
        assert_eq!(r.missed_at_exit(17), 1); // total 1 + 1 = 2 (10 & 15 both missed on exit)
        // If loop instead continues immediately at 17: advance(17) adds 0 (no double count!).
        assert_eq!(r.advance(17), 0);
        assert_eq!(r.next_ns(), 20);

        // Case 4: 45ms release on 10ms grid (starts at 30, ends at 75):
        // intermediate skip (3: 50/60/70) committed at release end; continuing adds 0; exit adds 1 (= 4 total).
        let mut r = Releases { period_ns: 10, next_ns: 30 };
        assert_eq!(r.advance(30), 0); // next_ns = 40
        assert_eq!(r.advance_overrun(75), 3);
        assert_eq!(r.next_ns(), 70);
        assert_eq!(r.missed_at_exit(75), 1);
        assert_eq!(r.advance(75), 0);
        assert_eq!(r.next_ns(), 80);
    }

    #[test]
    fn steps_run_with_elapsed_time_until_stopped() {
        let stop = AtomicBool::new(false);
        let stats = ExecutorStats::default();
        let mut dts = Vec::with_capacity(16);
        run_periodic(Duration::from_millis(2), &stop, &stats, |dt| {
            dts.push(dt);
            if dts.len() == 5 { stop.store(true, Ordering::Release); }
        });
        assert_eq!(dts.len(), 5);
        assert!(dts.iter().all(|dt| *dt > 0.0 && *dt < 0.5), "{dts:?}");
        assert_eq!(stats.steps.load(Ordering::Relaxed), 5);
    }

    #[test]
    fn owner_env_and_startup_gate_fail_closed() {
        assert_eq!(RtOwner::from_var(None).unwrap(), RtOwner::Legacy);
        assert_eq!(RtOwner::from_var(Some("legacy")).unwrap(), RtOwner::Legacy);
        assert_eq!(RtOwner::from_var(Some("executor")).unwrap(), RtOwner::Executor);
        assert!(RtOwner::from_var(Some("unknown")).is_err());
        assert!(validate_startup_owner(RtOwner::Legacy, "rebot-dm").is_ok());
        assert!(validate_startup_owner(RtOwner::Executor, "mock").is_ok());
        assert!(validate_startup_owner(RtOwner::Executor, "sim").is_err());
        assert!(validate_startup_owner(RtOwner::Executor, "rebot-dm").is_err());
    }

    #[test]
    fn maint_client_drains_final_commit_reply_after_observing_exited() {
        let (req_tx, _req_rx) = RingBuffer::<MaintRequest>::new(4);
        let (mut rep_tx, rep_rx) = RingBuffer::<MaintReply>::new(4);
        let exited = Arc::new(AtomicBool::new(false));
        let mut inner = MaintClientInner {
            req_tx,
            rep_rx,
            in_flight: 1,
            next_id: 2,
            closed: false,
            waiters: HashMap::new(),
            exited: exited.clone(),
        };
        let (tx, rx) = mpsc::sync_channel(1);
        inner.waiters.insert(1, MaintWaiter::Sync(tx));

        // Simulate the exact race: consumer observes an empty reply queue, then producer pushes
        // `ShutdownCommitted(true)` and stores `exited = true` before consumer checks `exited`.
        inner.drain_replies_with_hook(|| {
            rep_tx
                .push(MaintReply {
                    id: 1,
                    result: MaintenanceResult::ShutdownCommitted(true),
                    returned_offsets: None,
                    returned_probe: None,
                    returned_driver_config_error: None,
                })
                .unwrap();
            exited.store(true, Ordering::Release);
        });

        let delivered = rx.try_recv().expect("commit reply must be delivered, not lost to exited");
        assert!(matches!(delivered, Ok(MaintenanceResult::ShutdownCommitted(true))));
        assert!(inner.waiters.is_empty());
        assert_eq!(inner.in_flight, 0);
    }

    #[test]
    fn estop_registry_drains_final_ack_after_observing_exited() {
        let (mut ack_tx, ack_rx) = RingBuffer::<EstopAck>::new(4);
        let overflow = Arc::new(AtomicU64::new(0));
        let exited = Arc::new(AtomicBool::new(false));
        let mut reg = EstopRegistry::default();
        reg.attach_executor(ack_rx, overflow, exited.clone());
        let mut rx = reg.register(7);

        reg.drain_with_hook(|| {
            ack_tx
                .push(EstopAck {
                    first_gen: 7,
                    last_gen: 7,
                    state: Arc::new(SafetyState::EStop),
                    epoch: 42,
                })
                .unwrap();
            exited.store(true, Ordering::Release);
        });

        let delivered = rx.try_recv().expect("final estop ack must be delivered, not lost to exited");
        assert_eq!(delivered.unwrap(), (SafetyState::EStop, 42));
        assert!(reg.waiters.is_empty());
    }

    #[test]
    fn exit_drain_returns_persist_payloads_without_mutating_live_state() {
        let mut core = SupervisorInner::new(crate::model::ArmConfig::rebot_dm());
        core.lease_enabled = false;
        core.set_torque(true).unwrap();
        let epoch = core.safety.epoch_id();
        let (action_id, _) = core
            .start_gripper_zero_check(&serde_json::json!({"threshold_rad": 0.10, "store_offset": true}), epoch)
            .unwrap();
        core.zero_check_persisting = Some(action_id);
        let action_before = core.last_action.clone().expect("running zero_check action");
        assert!(action_before.is_running());

        let (_, maint_req_rx) = RingBuffer::<MaintRequest>::new(2);
        let (maint_rep_tx, _) = RingBuffer::<MaintReply>::new(2);
        let (estop_ack_tx, _) = RingBuffer::<EstopAck>::new(2);
        let (reclaim_state_tx, _) = RingBuffer::<Arc<SafetyState>>::new(2);
        let (_, reclaim_return_rx) = RingBuffer::<Arc<SafetyState>>::new(2);
        let (safety_log_tx, _) = RingBuffer::<SafetyLogEvent>::new(2);
        let (persist_job_tx, _) = RingBuffer::<PendingZeroCheckPersist>::new(2);
        let (mut persist_cmd_tx, persist_cmd_rx) = RingBuffer::<PersistCmd>::new(2);
        let (persist_rep_tx, mut persist_rep_rx) = RingBuffer::<PersistReply>::new(PERSIST_REP_CAPACITY);
        let (offload_reclaim_tx, _offload_reclaim_rx) =
            RingBuffer::<OffloadReclaim>::new(OFFLOAD_RECLAIM_CAPACITY);

        let probe_dropped_on = Arc::new(Mutex::new(None::<ThreadId>));
        struct DropTracker(Arc<Mutex<Option<ThreadId>>>);
        impl Drop for DropTracker {
            fn drop(&mut self) {
                *self.0.lock().unwrap() = Some(thread::current().id());
            }
        }
        let hooks = PersistTestHooks {
            reply_drop_probe: Some(Arc::new(Mutex::new(Some(Box::new(DropTracker(
                probe_dropped_on.clone(),
            )))))),
            ..Default::default()
        };

        let (_, _, _, stop_requests) = ReleaseEngine::new(BootId(1), Capacities::default());
        let mut state = ControlState {
            shutdown_committed: false,
            control_stop: Arc::new(AtomicBool::new(true)),
            stop_requests,
            estop_ack_tx,
            estop_overflow_gen: Arc::new(AtomicU64::new(0)),
            reclaim_state_tx,
            reclaim_return_rx,
            safety_log_tx,
            stats: Arc::new(ExecutorStats::default()),
            maint_req_rx,
            maint_rep_tx,
            pending_maint_reply: None,
            persist_job_tx,
            persist_cmd_rx,
            persist_rep_tx,
            pending_persist_reply: None,
            retained_persist_job: None,
            offload_reclaim_tx,
            persist_wake: thread::current(),
            persist_granted: Some((action_id, ConfigDomain::GripperCalibration)),
            next_persist_req: 1,
            disconnect_code_buf: Some(String::from("buf")),
            disconnect_msg_buf: Some(String::from("msg")),
            sat_msg_buf: Some(String::with_capacity(64)),
            hooks,
        };

        // Queue a `PersistCmd::Fail` after the last release AND simulate an active reservation
        // with an abandoned producer (`drop(persist_cmd_tx)`).
        persist_cmd_tx
            .push(PersistCmd::Fail {
                action_id,
                code: "PERSIST_FAILED".into(),
                message: "late fail queued during clean exit".into(),
                unused_policy: None,
                unused_jaw: None,
            })
            .unwrap();
        drop(persist_cmd_tx);

        state.drain_persistence_stopped(&core);

        // Live state must NOT have been mutated after the last publish!
        assert_eq!(core.last_action.as_ref().map(|a| a.state.as_str()), Some("running"));
        assert_eq!(core.zero_check_persisting, Some(action_id));
        assert!(
            !core.core.config().motion_inhibited(),
            "exit drain must not transition live config authority to Reconciling"
        );
        assert!(state.persist_granted.is_none());
        // Payloads must still be queued in `persist_rep_rx` (not dropped on the owner thread).
        assert!(
            probe_dropped_on.lock().unwrap().is_none(),
            "returned persistence payloads must remain alive until dropped off-thread"
        );
        let mut drained = 0usize;
        while let Ok(rep) = persist_rep_rx.pop() {
            drained += 1;
            drop(rep);
        }
        assert_eq!(drained, 2, "expected returned Fail payload reply and disconnect buffer reply");
        assert!(probe_dropped_on.lock().unwrap().is_some());
    }

    #[test]
    fn saturated_estop_and_reclaim_rings_retain_retired_fault_and_offload_on_drain_or_exit() {
        let mut core = SupervisorInner::new(crate::model::ArmConfig::rebot_dm());
        core.safety.set_off_thread_reclaim(true);
        // Consume 1 slot from `fault_arc_pool` so `drain_retired_states` wants to replenish it,
        // then clear back to `Idle` and drain the retired arc off-thread.
        core.safety.trigger_fault(crate::safety::FaultReason::ThermalTrip {
            joint_idx: 0,
            temperature: 95.0,
            limit: 85.0,
        });
        let _ = core.safety.clear_fault_bounded().unwrap();
        while let Some(a) = core.safety.pop_retired_state_arc() { drop(a); }
        assert!(core.safety.fault_arc_pool_len() < FAULT_ARC_POOL_CAPACITY);

        let (mut estop_ack_tx, mut estop_ack_rx) = RingBuffer::<EstopAck>::new(2);
        let (mut reclaim_state_tx, mut reclaim_state_rx) = RingBuffer::<Arc<SafetyState>>::new(2);
        let (mut reclaim_return_tx, reclaim_return_rx) = RingBuffer::<Arc<SafetyState>>::new(2);
        let (safety_log_tx, _safety_log_rx) = RingBuffer::<SafetyLogEvent>::new(2);
        let (_, maint_req_rx) = RingBuffer::<MaintRequest>::new(2);
        let (maint_rep_tx, _) = RingBuffer::<MaintReply>::new(2);
        let (persist_job_tx, _) = RingBuffer::<PendingZeroCheckPersist>::new(2);
        let (_, persist_cmd_rx) = RingBuffer::<PersistCmd>::new(2);
        let (persist_rep_tx, _) = RingBuffer::<PersistReply>::new(PERSIST_REP_CAPACITY);
        let (offload_reclaim_tx, mut offload_reclaim_rx) =
            RingBuffer::<OffloadReclaim>::new(OFFLOAD_RECLAIM_CAPACITY);

        // Fill both `estop_ack_tx` and `reclaim_state_tx` to 100% capacity.
        for g in 1..=2 {
            estop_ack_tx
                .push(EstopAck { first_gen: g, last_gen: g, state: Arc::new(SafetyState::EStop), epoch: g })
                .unwrap();
            reclaim_state_tx.push(Arc::new(SafetyState::Idle)).unwrap();
        }
        assert!(estop_ack_tx.is_full());
        assert!(reclaim_state_tx.is_full());

        let estop_overflow_gen = Arc::new(AtomicU64::new(0));
        let (_, _, _, stop_requests) = ReleaseEngine::new(BootId(1), Capacities::default());
        let mut state = ControlState {
            shutdown_committed: false,
            control_stop: Arc::new(AtomicBool::new(false)),
            stop_requests,
            estop_ack_tx,
            estop_overflow_gen: estop_overflow_gen.clone(),
            reclaim_state_tx,
            reclaim_return_rx,
            safety_log_tx,
            stats: Arc::new(ExecutorStats::default()),
            maint_req_rx,
            maint_rep_tx,
            pending_maint_reply: None,
            persist_job_tx,
            persist_cmd_rx,
            persist_rep_tx,
            pending_persist_reply: None,
            retained_persist_job: None,
            offload_reclaim_tx,
            persist_wake: thread::current(),
            persist_granted: None,
            next_persist_req: 0,
            disconnect_code_buf: None,
            disconnect_msg_buf: None,
            sat_msg_buf: None,
            hooks: PersistTestHooks::default(),
        };

        // Queue two retired sole-ref Fault acks (not equal to `core.safety.state()`, which is Idle).
        let fault_a = Arc::new(SafetyState::Fault(crate::safety::FaultReason::Other("sole-a".into())));
        let weak_a = Arc::downgrade(&fault_a);
        let fault_b = Arc::new(SafetyState::Fault(crate::safety::FaultReason::Other("sole-b".into())));
        let weak_b = Arc::downgrade(&fault_b);
        core.pending_estop_acks.push_back(EstopAck { first_gen: 10, last_gen: 10, state: fault_a, epoch: 10 });
        core.pending_estop_acks.push_back(EstopAck { first_gen: 11, last_gen: 11, state: fault_b, epoch: 11 });

        // Both rings full: overflow watermark advances, neither sole-ref Fault is dropped on RT.
        state.emit_estop_ack_if_advanced(&mut core);
        assert_eq!(estop_overflow_gen.load(Ordering::Acquire), 10);
        assert_eq!(core.pending_estop_acks.len(), 2);
        assert!(weak_a.upgrade().is_some(), "sole-ref fault_a must not drop while both rings are full");
        assert!(weak_b.upgrade().is_some(), "sole-ref fault_b must not drop while both rings are full");

        // Free 1 slot in `reclaim_state_rx`: fault_a moves to `reclaim_state_tx`, fault_b stays queued.
        let _ = reclaim_state_rx.pop().unwrap();
        state.emit_estop_ack_if_advanced(&mut core);
        assert_eq!(core.pending_estop_acks.len(), 1);
        assert_eq!(estop_overflow_gen.load(Ordering::Acquire), 11);
        assert!(weak_a.upgrade().is_some(), "fault_a must stay alive in reclaim_state_tx");
        assert!(weak_b.upgrade().is_some(), "fault_b must stay alive in pending_estop_acks");

        // With `reclaim_state_tx` full, `fault_arc_pool` below capacity, and `formatted_fault_arc_pool`
        // full, push a uniquely-owned (`strong_count == 1, weak_count == 0`), empty, pre-sized
        // `Other` slot so `return_fault_arc` passes the ownership check and rejects strictly on the
        // full target variant pool — verifying it stays queued while `reclaim_state_tx` is full
        // and forwards the exact `Arc` pointer to `reclaim_state_rx` once a slot opens.
        assert_eq!(core.safety.formatted_fault_arc_pool_len(), FORMATTED_FAULT_ARC_POOL_CAPACITY);
        let rejected_ret = Arc::new(SafetyState::Fault(crate::safety::FaultReason::Other(
            String::with_capacity(crate::safety::SAFETY_LOG_TEXT_CAPACITY),
        )));
        let rejected_ptr = Arc::as_ptr(&rejected_ret);
        reclaim_return_tx.push(rejected_ret).unwrap();
        state.drain_retired_states(&mut core);
        assert_eq!(reclaim_return_tx.slots(), 1, "rejected return must stay queued while reclaim_state_tx is full");

        // Free 1 slot in `reclaim_state_rx`: `drain_retired_states` pops the rejected return and
        // forwards the exact same `Arc` into `reclaim_state_tx` (not dropped on RT).
        let _ = reclaim_state_rx.pop().unwrap();
        state.drain_retired_states(&mut core);
        assert_eq!(reclaim_return_tx.slots(), 2, "rejected return popped once reclaim_state_tx had a free slot");

        // Clean-exit drain while both rings remain full moves fault_b into lock-free `offload_reclaim_tx`.
        state.drain_estop_and_retired_stopped(&mut core);
        assert!(core.pending_estop_acks.is_empty());
        assert!(weak_b.upgrade().is_some(), "fault_b must stay alive until off-thread drain");

        // Off-thread drain observes the exact forwarded `rejected_ptr` and drops all arcs cleanly.
        let mut saw_rejected_ptr = false;
        while let Ok(a) = estop_ack_rx.pop() { drop(a); }
        while let Ok(a) = reclaim_state_rx.pop() {
            if Arc::as_ptr(&a) == rejected_ptr {
                saw_rejected_ptr = true;
            }
            drop(a);
        }
        while let Ok(item) = offload_reclaim_rx.pop() { drop(item); }
        assert!(saw_rejected_ptr, "variant-rejected return Arc must reach outbound reclaim_state_rx");
        assert!(weak_a.upgrade().is_none(), "fault_a dropped off-thread");
        assert!(weak_b.upgrade().is_none(), "fault_b dropped off-thread");
    }

    #[test]
    fn saturated_persist_and_maint_reply_rings_backpressure_and_hand_off_at_exit() {
        let mut core = SupervisorInner::new(crate::model::ArmConfig::rebot_dm());
        let (mut maint_req_tx, maint_req_rx) = RingBuffer::<MaintRequest>::new(2);
        let (mut maint_rep_tx, mut maint_rep_rx) = RingBuffer::<MaintReply>::new(2);
        let (estop_ack_tx, _) = RingBuffer::<EstopAck>::new(2);
        let (reclaim_state_tx, _) = RingBuffer::<Arc<SafetyState>>::new(2);
        let (_reclaim_return_tx, reclaim_return_rx) = RingBuffer::<Arc<SafetyState>>::new(2);
        let (safety_log_tx, _safety_log_rx) = RingBuffer::<SafetyLogEvent>::new(2);
        let (persist_job_tx, _) = RingBuffer::<PendingZeroCheckPersist>::new(2);
        let (mut persist_cmd_tx, persist_cmd_rx) = RingBuffer::<PersistCmd>::new(2);
        let (mut persist_rep_tx, mut persist_rep_rx) = RingBuffer::<PersistReply>::new(PERSIST_REP_CAPACITY);
        let (mut offload_reclaim_tx, mut offload_reclaim_rx) = RingBuffer::<OffloadReclaim>::new(2);

        // Fill `maint_rep_tx` to full (2/2), `persist_rep_tx` to slots() == 1 (7/8),
        // and `offload_reclaim_tx` to full (2/2) to exercise retained-job fallback.
        for id in 1..=2 {
            maint_rep_tx.push(MaintReply { id, result: MaintenanceResult::ProbeReturned, returned_offsets: None, returned_probe: None, returned_driver_config_error: None }).unwrap();
        }
        for _ in 0..(PERSIST_REP_CAPACITY - 1) {
            persist_rep_tx.push(PersistReply::Done {
                _returned_policy: None, _returned_jaw: None, _returned_code: None,
                _returned_msg: None, _returned_warning: None, _returned_probe: None,
            }).unwrap();
        }
        assert_eq!(persist_rep_tx.slots(), 1);
        for _ in 0..2 {
            offload_reclaim_tx.push(OffloadReclaim::Buf(String::new())).unwrap();
        }
        assert!(offload_reclaim_tx.is_full());

        let maint_probe_dropped = Arc::new(AtomicBool::new(false));
        let persist_probe_dropped = Arc::new(AtomicBool::new(false));
        let rejected_probe_dropped = Arc::new(AtomicBool::new(false));
        struct FlagDrop(Arc<AtomicBool>);
        impl Drop for FlagDrop { fn drop(&mut self) { self.0.store(true, Ordering::SeqCst); } }

        maint_req_tx.push(MaintRequest {
            id: 99,
            op: MaintenanceOp::ProbeOwnedPayload { payload: Box::new(FlagDrop(maint_probe_dropped.clone())) },
        }).unwrap();
        persist_cmd_tx.push(PersistCmd::Reserve {
            action_id: 1,
            base_revision: 0,
            lease_owner: None,
            new_policy: crate::model::GripperPolicy::rebot_dm(),
            jaw_opening: None,
            probe: Some(Box::new(FlagDrop(persist_probe_dropped.clone()))),
            code_buf: String::with_capacity(32),
            msg_buf: String::with_capacity(64),
        }).unwrap();

        let (engine, _, _, stop_requests) = ReleaseEngine::new(BootId(1), Capacities::default());
        let ctx = engine.ctx(Instant::now());
        let mut state = ControlState {
            shutdown_committed: false,
            control_stop: Arc::new(AtomicBool::new(false)),
            stop_requests,
            estop_ack_tx,
            estop_overflow_gen: Arc::new(AtomicU64::new(0)),
            reclaim_state_tx,
            reclaim_return_rx,
            safety_log_tx,
            stats: Arc::new(ExecutorStats::default()),
            maint_req_rx,
            maint_rep_tx,
            pending_maint_reply: None,
            persist_job_tx,
            persist_cmd_rx,
            persist_rep_tx,
            pending_persist_reply: None,
            retained_persist_job: None,
            offload_reclaim_tx,
            persist_wake: thread::current(),
            persist_granted: None,
            next_persist_req: 0,
            disconnect_code_buf: Some(String::with_capacity(64)),
            disconnect_msg_buf: Some(String::with_capacity(160)),
            sat_msg_buf: Some(String::with_capacity(160)),
            hooks: PersistTestHooks::default(),
        };

        // Under saturation: neither maint nor persist pops or drops its payload.
        let mut flags = 0u32;
        let mut req_id = 0u64;
        state.service_one_maintenance(&mut core, &ctx, Instant::now() + Duration::from_millis(20), 1, &mut flags, &mut req_id);
        state.service_persistence(&mut core, &ctx);
        assert!(!maint_probe_dropped.load(Ordering::SeqCst), "maint payload must not drop under full reply ring");
        assert!(!persist_probe_dropped.load(Ordering::SeqCst), "persist payload must not drop when slots() < 2");

        // Saturated persist job rejection NEVER writes an unsolicited PersistReply onto `persist_rep_tx`:
        // when `offload_reclaim_tx` is also full, it retains the job until a slot opens.
        let sample_job = PendingZeroCheckPersist {
            action_id: 7,
            offset_rad: 0.01,
            jaw_opening: None,
            base_revision: 0,
            lease_owner: None,
            operator_auth: None,
            write: crate::persist::PendingWrite::new(
                std::path::PathBuf::from("/tmp/armd-unused-test.json"),
                "{}".to_string(),
            ),
            base_gripper_policy: Some(crate::model::GripperPolicy::rebot_dm()),
            current_calib_jaw: None,
            gripper_limit: None,
            active_tool_is_stock: true,
        };
        state.reject_saturated_persist_job(
            &mut core,
            sample_job,
            Some(Box::new(FlagDrop(rejected_probe_dropped.clone()))),
        );
        assert_eq!(state.persist_rep_tx.slots(), 1, "rejected persist job must never touch persist_rep_tx");
        assert!(state.retained_persist_job.is_some(), "full offload_reclaim_tx must retain rejected job");
        assert!(!rejected_probe_dropped.load(Ordering::SeqCst), "rejected job probe must not drop on RT");

        // Draining offload_reclaim_rx lets flush_retained_persist hand off the rejected job off-thread.
        while let Ok(item) = offload_reclaim_rx.pop() { drop(item); }
        state.flush_retained_persist(&core);
        assert!(state.retained_persist_job.is_none());
        assert!(!rejected_probe_dropped.load(Ordering::SeqCst), "rejected job probe must remain alive until off-thread drain");
        while let Ok(item) = offload_reclaim_rx.pop() { drop(item); }
        assert!(rejected_probe_dropped.load(Ordering::SeqCst), "rejected job probe must drop off-thread");

        // Swap in a full-sized offload ring and fill the last slot of `persist_rep_tx` so exit drain
        // spills into `offload_reclaim_tx` without dropping anything on RT.
        let (full_offload_tx, mut full_offload_rx) =
            RingBuffer::<OffloadReclaim>::new(OFFLOAD_RECLAIM_CAPACITY);
        state.offload_reclaim_tx = full_offload_tx;
        state.persist_rep_tx.push(PersistReply::Done {
            _returned_policy: None, _returned_jaw: None, _returned_code: None,
            _returned_msg: None, _returned_warning: None, _returned_probe: None,
        }).unwrap();
        assert!(state.persist_rep_tx.is_full());
        state.drain_maintenance_stopped();
        state.drain_persistence_stopped(&core);
        assert!(!maint_probe_dropped.load(Ordering::SeqCst), "maint payload must stay alive in offload_reclaim_tx");
        assert!(!persist_probe_dropped.load(Ordering::SeqCst), "persist payload must stay alive in offload_reclaim_tx");

        // Off-thread drain drops everything cleanly.
        while let Ok(r) = maint_rep_rx.pop() { drop(r); }
        while let Ok(r) = persist_rep_rx.pop() { drop(r); }
        while let Ok(item) = full_offload_rx.pop() { drop(item); }
        assert!(maint_probe_dropped.load(Ordering::SeqCst));
        assert!(persist_probe_dropped.load(Ordering::SeqCst));
    }
}
