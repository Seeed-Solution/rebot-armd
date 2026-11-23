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
use crate::diagnostics::tick::{LockPhases, ThreadUsage, TickSample, UsageMark};
use crate::model::{ArmObservation, GripperPolicy, JawOpening};
use crate::persist::{PublishOutcome, StagedWrite};
use crate::safety::{SafetyError, SafetyState};
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

/// Counters the executor thread updates with plain atomic stores.
#[derive(Debug, Default)]
pub struct ExecutorStats {
    /// Steps run.
    pub steps: AtomicU64,
    /// Steps whose elapsed time exceeded [`LATE_LOG_AFTER`].
    pub late: AtomicU64,
    /// Elapsed time of the latest late step, microseconds.
    pub last_late_us: AtomicU64,
    /// Releases skipped because a step ended after them.
    pub skipped_releases: AtomicU64,
    /// Diagnostic `TickSample` values dropped because the off-thread ring was full.
    pub diag_dropped: AtomicU64,
    /// Sub-release transition wakes executed by `armd-control`.
    pub wakes: AtomicU64,
    /// Allocator calls observed across full sub-release wake bodies (when `alloc_timing` is enabled).
    pub wake_alloc_calls: AtomicU64,
}

impl ExecutorStats {
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
    ClearFault,
    SetJointZeroOffsets { offsets: Vec<f32> },
    SetMotorZero { joint: usize },
    #[doc(hidden)]
    ProbeOwnedPayload { payload: Box<dyn Any + Send> },
}

/// Reply payload of one [`MaintenanceOp`].
pub enum MaintenanceResult {
    ShutdownRequested(bool),
    ShutdownCommitted(bool),
    ShutdownReady(bool),
    PlanningSnapshot(PlanningSnapshot),
    Observation(Box<ArmObservation>),
    FaultCleared(Result<SafetyState, SafetyError>),
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

/// Off-thread consumer sinks drained by `armd-dispatch`, `tick_stats()`, and `shutdown_and_join()`.
pub(crate) struct OffThreadSinks {
    diag_rx: Mutex<Consumer<TickSample>>,
    estop_reg: Arc<Mutex<EstopRegistry>>,
    reclaim_state_rx: Mutex<Consumer<Arc<SafetyState>>>,
    persist_rep_rx: Arc<Mutex<Consumer<PersistReply>>>,
    persist_worker_exited: Arc<AtomicBool>,
    maint_client: Arc<MaintClient>,
}

impl OffThreadSinks {
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
            while let Ok(arc) = rx.pop() {
                drop(arc);
            }
        }
        self.maint_client.drain_replies();
        if self.persist_worker_exited.load(Ordering::Acquire) {
            let mut rx = self.persist_rep_rx.lock().unwrap_or_else(|e| e.into_inner());
            while let Ok(rep) = rx.pop() {
                drop(rep);
            }
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
        self.sinks.drain_all(&self.tick_stats);
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
        match self.call_maintenance(MaintenanceOp::ClearFault)? {
            MaintenanceResult::FaultCleared(r) => Ok(r),
            _ => Err(OwnerError::Protocol),
        }
    }

    pub async fn clear_fault_async(&self) -> Result<Result<SafetyState, SafetyError>, OwnerError> {
        match self.call_maintenance_async(MaintenanceOp::ClearFault).await? {
            MaintenanceResult::FaultCleared(r) => Ok(r),
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
    let (persist_job_tx, persist_job_rx) = RingBuffer::<PendingZeroCheckPersist>::new(4);
    let (persist_cmd_tx, persist_cmd_rx) = RingBuffer::<PersistCmd>::new(2);
    let (persist_rep_tx, persist_rep_rx) = RingBuffer::<PersistReply>::new(4);

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
        persist_rep_rx: persist_rep_rx.clone(),
        persist_worker_exited: persist_worker_exited.clone(),
        maint_client: maint_client.clone(),
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
                persist_job_tx,
                persist_cmd_rx,
                persist_rep_tx,
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
    maint_req_rx: Consumer<MaintRequest>,
    maint_rep_tx: Producer<MaintReply>,
    /// Reply for a maintenance op serviced before Step 3 commit / Step 6 publish, held until
    /// strictly after Step 6 publication (or flushed immediately when `CommitShutdown` stops).
    pending_maint_reply: Option<MaintReply>,
    persist_job_tx: Producer<PendingZeroCheckPersist>,
    persist_cmd_rx: Consumer<PersistCmd>,
    persist_rep_tx: Producer<PersistReply>,
    persist_wake: Thread,
    persist_granted: Option<(u64, ConfigDomain)>,
    next_persist_req: u64,
    disconnect_code_buf: Option<String>,
    disconnect_msg_buf: Option<String>,
    hooks: PersistTestHooks,
}

impl ControlState {
    fn emit_estop_ack_if_advanced(&mut self, core: &mut SupervisorInner) {
        let mut woke = false;
        while let Some(ack) = core.pending_estop_acks.pop_front() {
            let last_gen = ack.last_gen;
            if self.estop_ack_tx.push(ack).is_err() {
                self.estop_overflow_gen.fetch_max(last_gen, Ordering::Release);
            }
            woke = true;
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

    /// Moves retired fault `Arc<SafetyState>` snapshots off `armd-control` into the bounded SPSC
    /// reclamation ring (never pops when the ring is full so no `Arc` can drop on `armd-control`).
    fn drain_retired_states(&mut self, core: &mut SupervisorInner) {
        let mut drained = false;
        while !self.reclaim_state_tx.is_full() {
            let Some(arc) = core.safety.pop_retired_state_arc() else {
                break;
            };
            let _ = self.reclaim_state_tx.push(arc);
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
    ) {
        let Ok(req) = self.maint_req_rx.pop() else { return; };
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
            MaintenanceOp::ClearFault => {
                self.shutdown_committed = false;
                let res = core.clear_fault_executor(release_deadline);
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
        self.pending_maint_reply = Some(MaintReply {
            id: req.id,
            result,
            returned_offsets,
            returned_probe,
        });
    }

    fn flush_pending_maint_reply(&mut self, core: &SupervisorInner) {
        if let Some(reply) = self.pending_maint_reply.take() {
            let _ = self.maint_rep_tx.push(reply);
            core.core.hub().wake();
        }
    }

    fn drain_maintenance_stopped(&mut self) {
        if let Some(reply) = self.pending_maint_reply.take() {
            let _ = self.maint_rep_tx.push(MaintReply {
                id: reply.id,
                result: MaintenanceResult::Stopped,
                returned_offsets: reply.returned_offsets,
                returned_probe: reply.returned_probe,
            });
        }
        while let Ok(req) = self.maint_req_rx.pop() {
            let (returned_offsets, returned_probe) = match req.op {
                MaintenanceOp::SetJointZeroOffsets { offsets } => (Some(offsets), None),
                MaintenanceOp::ProbeOwnedPayload { payload } => (None, Some(payload)),
                _ => (None, None),
            };
            let _ = self.maint_rep_tx.push(MaintReply {
                id: req.id,
                result: MaintenanceResult::Stopped,
                returned_offsets,
                returned_probe,
            });
        }
    }

    /// Exit-path persistence drain: returns any queued `PersistCmd` heap payloads off-thread
    /// via `persist_rep_tx` and wakes `armd-persist` WITHOUT performing any unpublished live
    /// state mutations (`last_action`, `gripper_calib_revision`, `force_reconcile`).
    fn drain_persistence_stopped(&mut self, core: &SupervisorInner) {
        let mut woke = false;
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
            let _ = self.persist_rep_tx.push(reply);
            woke = true;
        }
        self.persist_granted = None;
        if self.disconnect_code_buf.is_some() || self.disconnect_msg_buf.is_some() {
            let _ = self.persist_rep_tx.push(PersistReply::Done {
                _returned_policy: None,
                _returned_jaw: None,
                _returned_code: self.disconnect_code_buf.take(),
                _returned_msg: self.disconnect_msg_buf.take(),
                _returned_warning: None,
                _returned_probe: None,
            });
            woke = true;
        }
        if woke {
            self.persist_wake.unpark();
            core.core.hub().wake();
        }
    }

    fn service_persistence(&mut self, core: &mut SupervisorInner, ctx: &super::ops::CommitCtx) {
        // B1 race-free abandonment rule: sample `is_abandoned()` BEFORE draining queued commands!
        // Because `rtrb` preserves queued items after producer drop, nothing can be pushed after
        // `abandoned == true` is observed.
        let abandoned = self.persist_cmd_rx.is_abandoned();
        if let Some(hook) = self.hooks.after_abandoned_check_before_pop.as_ref() {
            hook();
        }
        while let Ok(cmd) = self.persist_cmd_rx.pop() {
            self.handle_persist_cmd(core, ctx, cmd);
        }
        if abandoned {
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
                    let _ = self.persist_rep_tx.push(PersistReply::Done {
                        _returned_policy: None,
                        _returned_jaw: None,
                        _returned_code: old_code,
                        _returned_msg: old_msg,
                        _returned_warning: None,
                        _returned_probe: None,
                    });
                    core.core.hub().wake();
                }
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
                let _ = self.persist_rep_tx.push(PersistReply::Done {
                    _returned_policy: unused_policy,
                    _returned_jaw: unused_jaw,
                    _returned_code: old_code,
                    _returned_msg: old_msg,
                    _returned_warning: None,
                    _returned_probe: self.take_reply_probe(),
                });
                self.persist_wake.unpark();
                core.core.hub().wake();
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
                    let _ = self.persist_rep_tx.push(PersistReply::Reserved {
                        reservation: None,
                        policy: Some(new_policy),
                        returned_jaw: jaw_opening,
                        _returned_code: old_code,
                        _returned_msg: old_msg,
                        _returned_probe: probe,
                    });
                    self.persist_wake.unpark();
                    core.core.hub().wake();
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
                    let _ = self.persist_rep_tx.push(PersistReply::Reserved {
                        reservation: None,
                        policy: Some(new_policy),
                        returned_jaw: jaw_opening,
                        _returned_code: old_code,
                        _returned_msg: old_msg,
                        _returned_probe: probe,
                    });
                    self.persist_wake.unpark();
                    core.core.hub().wake();
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
                    let _ = self.persist_rep_tx.push(PersistReply::Reserved {
                        reservation: None,
                        policy: Some(new_policy),
                        returned_jaw: jaw_opening,
                        _returned_code: old_code,
                        _returned_msg: old_msg,
                        _returned_probe: probe,
                    });
                    self.persist_wake.unpark();
                    core.core.hub().wake();
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
                        let _ = self.persist_rep_tx.push(PersistReply::Reserved {
                            reservation: Some(reservation),
                            policy: Some(new_policy),
                            returned_jaw: jaw_opening,
                            _returned_code: Some(code_buf),
                            _returned_msg: Some(msg_buf),
                            _returned_probe: probe,
                        });
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
                        let _ = self.persist_rep_tx.push(PersistReply::Reserved {
                            reservation: None,
                            policy: Some(new_policy),
                            returned_jaw: jaw_opening,
                            _returned_code: old_code,
                            _returned_msg: old_msg,
                            _returned_probe: probe,
                        });
                    }
                }
                self.persist_wake.unpark();
                core.core.hub().wake();
            }
            PersistCmd::Abort { action_id, reservation, code, message, unused_policy, unused_jaw } => {
                self.persist_granted = None;
                let _ = ConfigAuthority::release(core, reservation);
                if core.zero_check_persisting == Some(action_id) {
                    core.zero_check_persisting = None;
                }
                let (old_code, old_msg) = core.fail_zero_check_action_owned(action_id, code, message);
                let _ = self.persist_rep_tx.push(PersistReply::Done {
                    _returned_policy: unused_policy,
                    _returned_jaw: unused_jaw,
                    _returned_code: old_code,
                    _returned_msg: old_msg,
                    _returned_warning: None,
                    _returned_probe: self.take_reply_probe(),
                });
                self.persist_wake.unpark();
                core.core.hub().wake();
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
                    let _ = self.persist_rep_tx.push(PersistReply::Done {
                        _returned_policy: Some(new_policy),
                        _returned_jaw: new_jaw,
                        _returned_code: old_code,
                        _returned_msg: old_msg,
                        _returned_warning: warning,
                        _returned_probe: self.take_reply_probe(),
                    });
                    self.persist_wake.unpark();
                    core.core.hub().wake();
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
                    let _ = self.persist_rep_tx.push(PersistReply::Done {
                        _returned_policy: old_policy,
                        _returned_jaw: old_jaw,
                        _returned_code: Some(code_buf),
                        _returned_msg: Some(msg_buf),
                        _returned_warning: unused_warning,
                        _returned_probe: self.take_reply_probe(),
                    });
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
                    let _ = self.persist_rep_tx.push(PersistReply::Done {
                        _returned_policy: Some(new_policy),
                        _returned_jaw: new_jaw,
                        _returned_code: old_code,
                        _returned_msg: old_msg,
                        _returned_warning: warning,
                        _returned_probe: self.take_reply_probe(),
                    });
                }
                self.persist_wake.unpark();
                core.core.hub().wake();
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
    persist_job_tx: Producer<PendingZeroCheckPersist>,
    persist_cmd_rx: Consumer<PersistCmd>,
    persist_rep_tx: Producer<PersistReply>,
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
    // BEFORE applying SCHED_FIFO so normal-priority helper threads (e.g. `armd-sim-rx`) can
    // never suffer single-CPU priority inversion during startup.
    core.rt_executor_owned = true;
    core.before_publish_hook = hooks.before_publish.clone();
    core.safety.set_off_thread_reclaim(true);
    if let Some(ref d) = core.driver {
        if let Ok(mut drv) = d.lock() {
            drv.set_rt_mode(true);
        }
    }

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
        maint_req_rx,
        maint_rep_tx,
        pending_maint_reply: None,
        persist_job_tx,
        persist_cmd_rx,
        persist_rep_tx,
        persist_wake,
        persist_granted: None,
        next_persist_req: 0,
        disconnect_code_buf: Some(String::with_capacity(64)),
        disconnect_msg_buf: Some(String::with_capacity(160)),
        hooks,
    };

    if ready_tx.send(Ok(thread::current().id())).is_err() {
        exited.store(true, Ordering::Release);
        *control_exited_cv.0.lock().unwrap_or_else(|e| e.into_inner()) = true;
        control_exited_cv.1.notify_all();
        hub.wake();
        return;
    }

    let mut releases = Releases::start(cfg.period);
    let mut last_release_start = Instant::now();
    let mut publish_gen = initial_generation;
    let mut panicked = false;

    while !control_stop.load(Ordering::Acquire) {
        sleep_until_ns(releases.next_ns());
        if control_stop.load(Ordering::Acquire) {
            break;
        }
        let release_start = Instant::now();
        let usage_start = UsageMark::now();
        let now_ns = mono_now_ns();
        let skipped = releases.advance(now_ns);
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
        let remaining_ns = next_release_ns.saturating_sub(mono_now_ns());
        let release_deadline = Instant::now() + Duration::from_nanos(remaining_ns);
        publish_gen += 1;

        let step_res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
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
                thread::sleep(delay);
            }
            let _ = crate::diagnostics::alloc::take();
            let body_start = Instant::now();
            core.defer_zero_check_persist = true;
            core.persist_capacity_available =
                !state.persist_job_tx.is_abandoned() && state.persist_job_tx.slots() > 0;
            core.begin_tick_phases(body_start);

            let mut report = engine.step_with_pre_commit(
                &mut core,
                Instant::now,
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
                    state.service_one_maintenance(core, ctx, release_deadline);
                    control_stop.load(Ordering::Acquire)
                },
            );
            if control_stop.load(Ordering::Acquire) {
                state.flush_pending_maint_reply(&core);
                state.emit_estop_ack_if_advanced(&mut core);
                state.drain_retired_states(&mut core);
                return;
            }
            let (phases, body_end) = core.take_tick_phases();
            let body_end = body_end.unwrap_or_else(Instant::now);
            core.defer_zero_check_persist = false;
            if let Some(job) = core.pending_zero_check_persist.take() {
                if state.persist_job_tx.push(job).is_err() {
                    if let Some(id) = core.zero_check_persisting.take() {
                        let (old_code, old_msg) = core.finish_zero_check_action_in_place(
                            id,
                            "failed",
                            "PERSIST_FAILED",
                            "Persistence queue saturated",
                            state.disconnect_code_buf.take(),
                            state.disconnect_msg_buf.take(),
                        );
                        if old_code.is_some() || old_msg.is_some() {
                            let _ = state.persist_rep_tx.push(PersistReply::Done {
                                _returned_policy: None,
                                _returned_jaw: None,
                                _returned_code: old_code,
                                _returned_msg: old_msg,
                                _returned_warning: None,
                                _returned_probe: None,
                            });
                        }
                    }
                } else {
                    state.persist_wake.unpark();
                }
            }
            // Deliver the pre-commit maintenance reply strictly AFTER Step 6 publication, then
            // drain any E-stop acks / retired states produced during Steps 3-7.
            state.flush_pending_maint_reply(&core);
            state.emit_estop_ack_if_advanced(&mut core);
            state.drain_retired_states(&mut core);
            let control_hz = core.config.control_frequency_hz;
            let done = Instant::now();
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
            };
            if diag_tx.push(sample).is_err() {
                stats.diag_dropped.fetch_add(1, Ordering::Relaxed);
            }
            hub.wake();

            // Sub-release transition wakes before `next_release_ns` (never resetting `last_release_start`).
            let mut wakes = 0u32;
            let mut next_wake = report.poll.wake_before(release_deadline);
            drop(_guard);
            while let Some(wake) = next_wake {
                if wakes >= MAX_SERVICE_WAKES || control_stop.load(Ordering::Acquire) {
                    break;
                }
                let now = Instant::now();
                if wake > now {
                    let wake_ns = mono_now_ns()
                        .saturating_add(wake.saturating_duration_since(now).as_nanos().min(u128::from(u64::MAX)) as u64);
                    sleep_until_ns(wake_ns.min(next_release_ns));
                }
                if control_stop.load(Ordering::Acquire) {
                    break;
                }
                wakes += 1;
                stats.wakes.fetch_add(1, Ordering::Relaxed);
                let _wake_guard = ReleaseGuard::enter();
                let _ = crate::diagnostics::alloc::take();
                assert!(
                    tokio::runtime::Handle::try_current().is_err(),
                    "RT executor thread must never enter a Tokio runtime during a sub-release wake"
                );
                let published = engine
                    .service_wake(&mut core, Instant::now(), release_deadline, publish_gen + 1, &mut report);
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
                let wake_allocs = crate::diagnostics::alloc::take();
                if wake_allocs.calls > 0 {
                    stats.wake_alloc_calls.fetch_add(wake_allocs.calls, Ordering::Relaxed);
                }
                next_wake = report.poll.wake_before(release_deadline);
            }
        }));

        if let Err(err) = step_res {
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
    state.emit_estop_ack_if_advanced(&mut core);
    state.drain_retired_states(&mut core);
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
    /// First release one period from now.
    pub fn start(period: Duration) -> Self {
        let period_ns = (period.as_nanos() as u64).max(1);
        Self { period_ns, next_ns: mono_now_ns() + period_ns }
    }

    /// Advances past a release at `now_ns`, preserving the original absolute release grid phase:
    /// the next release is the earliest grid point strictly after `now_ns` (`next_ns + k * period_ns`),
    /// and missed grid points are counted in the returned skip count.
    pub fn advance(&mut self, now_ns: u64) -> u64 {
        self.next_ns += self.period_ns;
        if self.next_ns <= now_ns {
            let skipped = (now_ns - self.next_ns) / self.period_ns + 1;
            self.next_ns += skipped * self.period_ns;
            skipped
        } else {
            0
        }
    }

    pub fn next_ns(&self) -> u64 { self.next_ns }
}

/// Runs `step(dt)` at every release until `stop` is set. `dt` is the real time since the
/// previous step's start (seconds). Allocation free.
pub fn run_periodic(period: Duration, stop: &AtomicBool, stats: &ExecutorStats, mut step: impl FnMut(f64)) {
    let mut releases = Releases::start(period);
    let mut last = Instant::now();
    while !stop.load(Ordering::Acquire) {
        sleep_until_ns(releases.next_ns());
        let woke = Instant::now();
        let skipped = releases.advance(mono_now_ns());
        if skipped > 0 { stats.skipped_releases.fetch_add(skipped, Ordering::Relaxed); }
        let elapsed = woke - last;
        last = woke;
        if elapsed > LATE_LOG_AFTER {
            stats.last_late_us.store(elapsed.as_micros() as u64, Ordering::Relaxed);
            stats.late.fetch_add(1, Ordering::Release);
        }
        stats.steps.fetch_add(1, Ordering::Relaxed);
        step(elapsed.as_secs_f64());
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
        let (persist_job_tx, _) = RingBuffer::<PendingZeroCheckPersist>::new(2);
        let (mut persist_cmd_tx, persist_cmd_rx) = RingBuffer::<PersistCmd>::new(2);
        let (persist_rep_tx, mut persist_rep_rx) = RingBuffer::<PersistReply>::new(4);

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
            maint_req_rx,
            maint_rep_tx,
            pending_maint_reply: None,
            persist_job_tx,
            persist_cmd_rx,
            persist_rep_tx,
            persist_wake: thread::current(),
            persist_granted: Some((action_id, ConfigDomain::GripperCalibration)),
            next_persist_req: 1,
            disconnect_code_buf: Some(String::from("buf")),
            disconnect_msg_buf: Some(String::from("msg")),
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
}
