//! Control tick timing (`GET /api/v1/diagnostics/tick`): a fixed ring of the last
//! [`TICK_WINDOW`] ticks, recorded by [`crate::ArmSupervisor::tick`].

use std::cell::Cell;
use std::sync::atomic::{fence, AtomicBool, AtomicI64, AtomicPtr, AtomicU32, AtomicU64, AtomicU8, AtomicUsize, Ordering};

use serde::{Serialize, Serializer};

/// Ticks the percentiles cover.
pub const TICK_WINDOW: usize = 1000;

/// Capacity of [`LockFreeTickStats`] (power of two, strictly larger than [`TICK_WINDOW`] so
/// concurrent HTTP readers have a >1000-tick cushion against slot reuse).
pub const LOCK_FREE_RING_CAPACITY: usize = 2048;

const MAX_SEQLOCK_RETRIES: usize = 64;
const MAX_WINDOW_ATTEMPTS: usize = 4;

/// Strict wakeup-lateness thresholds evaluated on raw nanoseconds (`actual - scheduled`).
pub const WAKE_LATENESS_1MS_NS: u64 = 1_000_000;
pub const WAKE_LATENESS_5MS_NS: u64 = 5_000_000;
pub const WAKE_LATENESS_10MS_NS: u64 = 10_000_000;

/// Inclusive full-release duration thresholds evaluated on raw nanoseconds (`>= 10 / 15 / 20 ms`),
/// matching the strict Stage 3 §6 `< 20 ms` gate (`release_ns >= 20_000_000` fails `< 20 ms`).
pub const RELEASE_10MS_NS: u64 = 10_000_000;
pub const RELEASE_15MS_NS: u64 = 15_000_000;
pub const RELEASE_20MS_NS: u64 = 20_000_000;

/// Converts nanoseconds to saturating microseconds (`u32`).
#[inline]
pub fn ns_to_us_u32(ns: u64) -> u32 {
    (ns / 1_000).min(u64::from(u32::MAX)) as u32
}

/// Converts raw full release duration (ns) to saturating microseconds and inclusive threshold
/// flags (`>= 10 ms`, `>= 15 ms`, `>= 20 ms`).
#[inline]
pub fn classify_release_duration_ns(release_ns: u64) -> (u32, bool, bool, bool) {
    let us = ns_to_us_u32(release_ns);
    (
        us,
        release_ns >= RELEASE_10MS_NS,
        release_ns >= RELEASE_15MS_NS,
        release_ns >= RELEASE_20MS_NS,
    )
}

/// Converts raw wakeup lateness (`actual_release_start - scheduled_release`, ns) to saturating
/// microseconds and strict threshold flags (`> 1 ms`, `> 5 ms`, `> 10 ms` on raw nanoseconds).
#[inline]
pub fn classify_wake_lateness_ns(lateness_ns: u64) -> (u32, bool, bool, bool) {
    let us = ns_to_us_u32(lateness_ns);
    (
        us,
        lateness_ns > WAKE_LATENESS_1MS_NS,
        lateness_ns > WAKE_LATENESS_5MS_NS,
        lateness_ns > WAKE_LATENESS_10MS_NS,
    )
}

thread_local! {
    static CURRENT_WAKE_LATENESS_NS: Cell<u64> = const { Cell::new(0) };
    static PERIODIC_DEFER_SAMPLE_PUBLISH: Cell<bool> = const { Cell::new(false) };
    static PENDING_PERIODIC_SAMPLE: Cell<Option<TickSample>> = const { Cell::new(None) };
    static PENDING_PERIODIC_POST_END: Cell<Option<std::time::Instant>> = const { Cell::new(None) };
}

/// Scoped thread-local handoff carrying raw wakeup lateness (ns) and periodic single-completion
/// sample handoff between `run_periodic_with_clock` and `ArmSupervisor::tick_shared` on the same
/// OS thread; restores the prior values on drop (including panic unwinds and early returns).
pub struct WakeLatenessGuard {
    prev_lateness: u64,
    prev_defer_publish: bool,
    prev_pending_sample: Option<TickSample>,
    prev_pending_post_end: Option<std::time::Instant>,
}

impl WakeLatenessGuard {
    #[inline]
    pub fn enter(lateness_ns: u64) -> Self {
        Self::enter_periodic(lateness_ns, false)
    }

    #[inline]
    pub fn enter_periodic(lateness_ns: u64, defer_sample_publish: bool) -> Self {
        let prev_lateness = CURRENT_WAKE_LATENESS_NS
            .try_with(|c| c.replace(lateness_ns))
            .unwrap_or(0);
        let prev_defer_publish = PERIODIC_DEFER_SAMPLE_PUBLISH
            .try_with(|c| c.replace(defer_sample_publish))
            .unwrap_or(false);
        let prev_pending_sample = PENDING_PERIODIC_SAMPLE
            .try_with(|c| c.replace(None))
            .unwrap_or(None);
        let prev_pending_post_end = PENDING_PERIODIC_POST_END
            .try_with(|c| c.replace(None))
            .unwrap_or(None);
        Self {
            prev_lateness,
            prev_defer_publish,
            prev_pending_sample,
            prev_pending_post_end,
        }
    }
}

impl Drop for WakeLatenessGuard {
    #[inline]
    fn drop(&mut self) {
        let _ = CURRENT_WAKE_LATENESS_NS.try_with(|c| c.set(self.prev_lateness));
        let _ = PERIODIC_DEFER_SAMPLE_PUBLISH.try_with(|c| c.set(self.prev_defer_publish));
        let _ = PENDING_PERIODIC_SAMPLE.try_with(|c| c.set(self.prev_pending_sample.take()));
        let _ = PENDING_PERIODIC_POST_END.try_with(|c| c.set(self.prev_pending_post_end.take()));
    }
}

/// Takes and clears the calling thread's pending raw wakeup lateness (ns).
#[inline]
pub fn take_current_wake_lateness_ns() -> u64 {
    CURRENT_WAKE_LATENESS_NS
        .try_with(|c| c.replace(0))
        .unwrap_or(0)
}

/// Returns `true` when the calling thread is inside a `run_periodic` release whose completion
/// point owns publishing the completed `TickSample` with full `release_us`.
#[inline]
pub fn periodic_defers_sample_publish() -> bool {
    PERIODIC_DEFER_SAMPLE_PUBLISH
        .try_with(Cell::get)
        .unwrap_or(false)
}

/// Stashes the calling thread's prepared legacy `TickSample` so `run_periodic_with_clock`
/// stamps its full `release_us` and publishes it once at release completion.
#[inline]
pub fn stash_periodic_sample(sample: TickSample) {
    stash_periodic_sample_with_post_end(sample, std::time::Instant::now());
}

/// Stashes the calling thread's prepared legacy `TickSample` together with its `post_end`
/// monotonic instant so `run_periodic_with_clock` stamps both `release_us` and `phases_us.publish`.
#[inline]
pub fn stash_periodic_sample_with_post_end(sample: TickSample, post_end: std::time::Instant) {
    let _ = PENDING_PERIODIC_SAMPLE.try_with(|c| c.set(Some(sample)));
    let _ = PENDING_PERIODIC_POST_END.try_with(|c| c.set(Some(post_end)));
}

/// Takes the calling thread's stashed legacy `TickSample` at periodic release completion.
#[inline]
pub fn take_periodic_sample() -> Option<TickSample> {
    let _ = PENDING_PERIODIC_POST_END.try_with(|c| c.take());
    PENDING_PERIODIC_SAMPLE
        .try_with(|c| c.take())
        .ok()
        .flatten()
}

/// Takes the calling thread's stashed legacy `TickSample` and its `post_end` monotonic instant.
#[inline]
pub fn take_periodic_sample_with_post_end() -> Option<(TickSample, std::time::Instant)> {
    let sample = PENDING_PERIODIC_SAMPLE.try_with(|c| c.take()).ok().flatten()?;
    let post_end = PENDING_PERIODIC_POST_END
        .try_with(|c| c.take())
        .ok()
        .flatten()
        .unwrap_or_else(std::time::Instant::now);
    Some((sample, post_end))
}

/// Outcome of one CAN `transmit` attempt recorded in the tick's TX diagnostics.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TxAttemptKind {
    Sent,
    WouldBlock,
    Enobufs,
    Error,
}

/// Per-tick CAN transmit metrics accumulated on the control thread (zero heap allocation).
///
/// Timed interval: each individual `CanTransport::transmit` call (`Sent`, `WouldBlock`,
/// `Enobufs`, and `Error` all contribute to wall/CPU sums and maxima; flush is excluded).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct TxTickMetrics {
    /// Frames successfully transmitted (`Ok(())`).
    pub frames: u32,
    /// `WouldBlock` (`EAGAIN` / `EWOULDBLOCK`) results on `transmit`.
    pub would_block: u32,
    /// `ENOBUFS` (`NoBufferSpace`) results on `transmit`.
    pub enobufs: u32,
    /// Commands newly deferred past their originating tick (counted once per command instance).
    pub deferred: u32,
    /// End-of-tick pending TX queue depth (gauge; not summed across ticks).
    pub pending: u32,
    /// Queued frames dropped on inclusive age expiry (`age >= frame_max_age`).
    pub dropped_stale: u32,
    /// Queued frames dropped on stop / E-stop / disable / mode switch / fence trip / error.
    pub dropped_stopped: u32,
    /// Sum of single-frame `transmit` wall times, ns.
    pub sum_wall_ns: u64,
    /// Sum of single-frame `transmit` thread CPU times (`CLOCK_THREAD_CPUTIME_ID`), ns.
    pub sum_cpu_ns: u64,
    /// Maximum single-frame `transmit` wall time, ns.
    pub max_wall_ns: u64,
    /// Maximum single-frame `transmit` thread CPU time (`CLOCK_THREAD_CPUTIME_ID`), ns.
    pub max_cpu_ns: u64,
}

impl TxTickMetrics {
    pub const ZERO: Self = Self {
        frames: 0,
        would_block: 0,
        enobufs: 0,
        deferred: 0,
        pending: 0,
        dropped_stale: 0,
        dropped_stopped: 0,
        sum_wall_ns: 0,
        sum_cpu_ns: 0,
        max_wall_ns: 0,
        max_cpu_ns: 0,
    };
}

/// Per-hold driver I/O timing breakdown (µs) attributed to a `SupervisorLock` hold or a
/// control-tick torque transition step (zero heap allocation).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize)]
pub struct HoldIoTiming {
    pub tx_us: u32,
    pub rx_us: u32,
    pub sleep_us: u32,
}

/// Control-tick torque transition attribution recorded on [`TickSample`] when a tick services a
/// torque transition step (omitted from JSON when `None`, zero heap allocation).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct TorqueStepAttribution {
    pub command: &'static str,
    pub direction: &'static str,
    pub tx_us: u32,
    pub rx_us: u32,
    pub sleep_us: u32,
}

#[derive(Clone, Copy, Default)]
struct HoldIoAccum {
    active: bool,
    tx_ns: u64,
    rx_ns: u64,
    sleep_ns: u64,
}

thread_local! {
    static TX_TICK: Cell<TxTickMetrics> = const { Cell::new(TxTickMetrics::ZERO) };
    static HOLD_IO: Cell<HoldIoAccum> = const { Cell::new(HoldIoAccum { active: false, tx_ns: 0, rx_ns: 0, sleep_ns: 0 }) };
}

/// Scoped synchronous thread-local accumulator for TX / RX / sleep time spent while holding a
/// `SupervisorLock` guard (or servicing a torque transition step in a control tick). Zero heap
/// allocations; restores the outer scope cleanly on drop (including early returns and panics).
pub struct HoldIoScope {
    prev: HoldIoAccum,
}

impl HoldIoScope {
    #[inline]
    pub fn enter() -> Self {
        let prev = HOLD_IO
            .try_with(|c| {
                c.replace(HoldIoAccum {
                    active: true,
                    tx_ns: 0,
                    rx_ns: 0,
                    sleep_ns: 0,
                })
            })
            .unwrap_or_default();
        Self { prev }
    }

    #[inline]
    pub fn snapshot(&self) -> HoldIoTiming {
        let a = HOLD_IO.try_with(Cell::get).unwrap_or_default();
        HoldIoTiming {
            tx_us: ns_to_us_u32(a.tx_ns),
            rx_us: ns_to_us_u32(a.rx_ns),
            sleep_us: ns_to_us_u32(a.sleep_ns),
        }
    }

    #[inline]
    pub fn finish(self) -> HoldIoTiming {
        self.snapshot()
    }
}

impl Drop for HoldIoScope {
    #[inline]
    fn drop(&mut self) {
        let _ = HOLD_IO.try_with(|c| c.set(self.prev));
    }
}

/// Fast check whether a [`HoldIoScope`] is currently active on the calling thread.
#[inline]
pub fn hold_io_active() -> bool {
    HOLD_IO.try_with(|c| c.get().active).unwrap_or(false)
}

/// Record TX wall nanoseconds into the active [`HoldIoScope`], if any (zero allocation).
#[inline]
pub fn record_hold_tx_ns(wall_ns: u64) {
    let _ = HOLD_IO.try_with(|c| {
        let mut a = c.get();
        if a.active {
            a.tx_ns = a.tx_ns.saturating_add(wall_ns);
            c.set(a);
        }
    });
}

/// Record RX wall nanoseconds into the active [`HoldIoScope`], if any (zero allocation).
#[inline]
pub fn record_hold_rx_ns(wall_ns: u64) {
    let _ = HOLD_IO.try_with(|c| {
        let mut a = c.get();
        if a.active {
            a.rx_ns = a.rx_ns.saturating_add(wall_ns);
            c.set(a);
        }
    });
}

/// Record sleep/pause nanoseconds into the active [`HoldIoScope`], if any (zero allocation).
#[inline]
pub fn record_hold_sleep_ns(sleep_ns: u64) {
    let _ = HOLD_IO.try_with(|c| {
        let mut a = c.get();
        if a.active {
            a.sleep_ns = a.sleep_ns.saturating_add(sleep_ns);
            c.set(a);
        }
    });
}

/// Monotonic counter of stopped-hold TX ticks skipped because `driver.try_lock()` was contended
/// (L3: stopped-hold TX never blocks on the driver mutex).
static STOPPED_HOLD_LOCK_CONTENDED: AtomicU64 = AtomicU64::new(0);
/// Monotonic counter of stopped-hold `try_lock` contention escalations triggered after
/// `STOPPED_HOLD_CONTENTION_ESCALATE_SKIPS` (25) consecutive contended ticks (0.5 s at 50 Hz).
static STOPPED_HOLD_CONTENTION_ESCALATIONS: AtomicU64 = AtomicU64::new(0);

/// Record one stopped-hold TX tick skipped due to driver mutex contention (zero allocation).
#[inline]
pub fn record_stopped_hold_lock_contended() {
    STOPPED_HOLD_LOCK_CONTENDED.fetch_add(1, Ordering::Relaxed);
}

/// Total stopped-hold TX ticks skipped due to driver mutex contention since boot.
#[inline]
pub fn stopped_hold_lock_contended_count() -> u64 {
    STOPPED_HOLD_LOCK_CONTENDED.load(Ordering::Relaxed)
}

/// Reset the stopped-hold driver lock contention counter (tests).
#[inline]
pub fn reset_stopped_hold_lock_contended() {
    STOPPED_HOLD_LOCK_CONTENDED.store(0, Ordering::Relaxed);
}

/// Record one stopped-hold `try_lock` contention escalation warning/fault (zero allocation).
#[inline]
pub fn record_stopped_hold_contention_escalation() {
    STOPPED_HOLD_CONTENTION_ESCALATIONS.fetch_add(1, Ordering::Relaxed);
}

/// Total stopped-hold `try_lock` contention escalations since boot.
#[inline]
pub fn stopped_hold_contention_escalations_count() -> u64 {
    STOPPED_HOLD_CONTENTION_ESCALATIONS.load(Ordering::Relaxed)
}

/// Reset the stopped-hold `try_lock` contention escalation counter (tests).
#[inline]
pub fn reset_stopped_hold_contention_escalations() {
    STOPPED_HOLD_CONTENTION_ESCALATIONS.store(0, Ordering::Relaxed);
}

/// Monotonic counter of RT disable dispatch/poll ticks skipped because `driver.try_lock()` was
/// contended (M4: `armd-control` disable paths never block on the driver mutex).
static DISABLE_LOCK_CONTENDED: AtomicU64 = AtomicU64::new(0);
/// Monotonic counter of RT disable `try_lock` contention escalations triggered after
/// `STOPPED_HOLD_CONTENTION_ESCALATE_SKIPS` (25) consecutive contended ticks (0.5 s at 50 Hz).
static DISABLE_CONTENTION_ESCALATIONS: AtomicU64 = AtomicU64::new(0);

/// Record one RT disable dispatch/poll tick skipped due to driver mutex contention (zero allocation).
#[inline]
pub fn record_disable_lock_contended() {
    DISABLE_LOCK_CONTENDED.fetch_add(1, Ordering::Relaxed);
}

/// Total RT disable dispatch/poll ticks skipped due to driver mutex contention since boot.
#[inline]
pub fn disable_lock_contended_count() -> u64 {
    DISABLE_LOCK_CONTENDED.load(Ordering::Relaxed)
}

/// Reset the RT disable driver lock contention counter (tests).
#[inline]
pub fn reset_disable_lock_contended() {
    DISABLE_LOCK_CONTENDED.store(0, Ordering::Relaxed);
}

/// Record one RT disable `try_lock` contention escalation warning/fault (zero allocation).
#[inline]
pub fn record_disable_contention_escalation() {
    DISABLE_CONTENTION_ESCALATIONS.fetch_add(1, Ordering::Relaxed);
}

/// Total RT disable `try_lock` contention escalations since boot.
#[inline]
pub fn disable_contention_escalations_count() -> u64 {
    DISABLE_CONTENTION_ESCALATIONS.load(Ordering::Relaxed)
}

/// Reset the RT disable `try_lock` contention escalation counter (tests).
#[inline]
pub fn reset_disable_contention_escalations() {
    DISABLE_CONTENTION_ESCALATIONS.store(0, Ordering::Relaxed);
}

/// Calling thread's CPU time in nanoseconds from `CLOCK_THREAD_CPUTIME_ID` (Linux; 0 elsewhere).
#[inline]
pub fn thread_cpu_ns() -> u64 {
    #[cfg(target_os = "linux")]
    {
        // SAFETY: plain syscall on the calling thread with a valid out-pointer.
        unsafe {
            let mut ts: libc::timespec = std::mem::zeroed();
            if libc::clock_gettime(libc::CLOCK_THREAD_CPUTIME_ID, &mut ts) != 0 {
                return 0;
            }
            let sec = u64::try_from(ts.tv_sec).unwrap_or(0);
            let nsec = u64::try_from(ts.tv_nsec).unwrap_or(0);
            sec.saturating_mul(1_000_000_000).saturating_add(nsec)
        }
    }
    #[cfg(not(target_os = "linux"))]
    0
}

/// Record one CAN `transmit` attempt on the calling thread (zero allocation).
#[inline]
pub fn record_tx_attempt(kind: TxAttemptKind, wall_ns: u64, cpu_ns: u64) {
    record_hold_tx_ns(wall_ns);
    let _ = TX_TICK.try_with(|c| {
        let mut m = c.get();
        match kind {
            TxAttemptKind::Sent => m.frames = m.frames.saturating_add(1),
            TxAttemptKind::WouldBlock => m.would_block = m.would_block.saturating_add(1),
            TxAttemptKind::Enobufs => m.enobufs = m.enobufs.saturating_add(1),
            TxAttemptKind::Error => {}
        }
        m.sum_wall_ns = m.sum_wall_ns.saturating_add(wall_ns);
        m.sum_cpu_ns = m.sum_cpu_ns.saturating_add(cpu_ns);
        if wall_ns > m.max_wall_ns { m.max_wall_ns = wall_ns; }
        if cpu_ns > m.max_cpu_ns { m.max_cpu_ns = cpu_ns; }
        c.set(m);
    });
}

/// Record newly deferred command count and end-of-tick pending queue depth (zero allocation).
#[inline]
pub fn record_tx_deferred(newly_deferred: u32, pending_depth: u32) {
    let _ = TX_TICK.try_with(|c| {
        let mut m = c.get();
        m.deferred = m.deferred.saturating_add(newly_deferred);
        m.pending = pending_depth;
        c.set(m);
    });
}

/// Record queued frames dropped due to age expiry (zero allocation).
#[inline]
pub fn record_tx_dropped_stale(count: u32) {
    if count == 0 { return; }
    let _ = TX_TICK.try_with(|c| {
        let mut m = c.get();
        m.dropped_stale = m.dropped_stale.saturating_add(count);
        c.set(m);
    });
}

/// Record queued frames dropped due to stop / E-stop / disable / mode switch / error (zero allocation).
#[inline]
pub fn record_tx_dropped_stopped(count: u32) {
    if count == 0 { return; }
    let _ = TX_TICK.try_with(|c| {
        let mut m = c.get();
        m.dropped_stopped = m.dropped_stopped.saturating_add(count);
        c.set(m);
    });
}

/// Take and reset the calling thread's accumulated per-tick TX metrics (zero allocation).
#[inline]
pub fn take_tx_metrics() -> TxTickMetrics {
    TX_TICK.try_with(|c| c.replace(TxTickMetrics::ZERO)).unwrap_or(TxTickMetrics::ZERO)
}

/// Reset the calling thread's accumulated per-tick TX metrics (zero allocation).
#[inline]
pub fn reset_tx_metrics() {
    let _ = TX_TICK.try_with(|c| c.set(TxTickMetrics::ZERO));
}

/// Phases of the tick's own write-lock hold, µs. They cover the hold: `lock_us` is their sum
/// up to the clock reads between them.
#[derive(Debug, Clone, Copy, Default, Serialize, PartialEq, Eq)]
pub struct LockPhases {
    /// From the lock to `SupervisorInner::tick`: thread usage read, hold log taken.
    pub pre_us: u32,
    /// Driver feedback read (CAN RX) and its bookkeeping.
    pub rx_us: u32,
    /// Safety checks, guards, trajectory / servo step.
    pub control_us: u32,
    /// Driver output (CAN TX).
    pub tx_us: u32,
    /// Output error handling, action reconcile, simulation mirror, handover.
    pub finish_us: u32,
    /// Observation build and episode frame (and anything after an early return).
    pub obs_us: u32,
    /// After `SupervisorInner::tick` until the lock is released: snapshot expiry and publication.
    pub publish_us: u32,
}

/// Sub-phase monotonic duration deltas (µs) across `lock_end → release completion`
/// for the legacy control release.
#[derive(Debug, Clone, Copy, Default, Serialize, PartialEq, Eq)]
pub struct ReleasePhasesUs {
    /// `lock_end` → after allocator-counter read (`alloc::take()`).
    pub alloc_read: u32,
    /// After allocator-counter read → after legacy E-stop ACK flush.
    pub ack_flush: u32,
    /// After ACK flush → after conditional terminal-observation capture.
    pub terminal: u32,
    /// After terminal capture → after write-guard drop (`released`).
    pub guard_drop: u32,
    /// `released` → post-lock work end (`done`).
    pub post: u32,
    /// Post-lock work end (`done`) → ring publication / completion seal.
    pub publish: u32,
}

/// Overrun-only (`release >= 15 ms`) monotonic phase durations (µs) across
/// `step return → ring publication → completion seal` (zero heap allocation).
#[derive(Debug, Clone, Copy, Default, Serialize, PartialEq, Eq)]
pub struct PostOverrunPhasesUs {
    /// Step return → start of ring publication (executor: `hub.wake`, `flush_torque_shadow`,
    /// guard drop, Window 2 wakes, pre-push tail; legacy: guard teardown, pre-publish tail).
    pub step_to_ring_us: u32,
    /// Ring publication (`diag_tx.push` + `hub.wake` in executor;
    /// `LockFreeTickStats::record_staged_with_publish` in legacy).
    pub ring_publish_us: u32,
    /// After ring publication → start of completion seal (tail allocs, amendment, pre-seal work).
    pub ring_to_seal_us: u32,
    /// Completion seal (`ExecutorStats::commit_release_outcome_with_clock`).
    pub seal_us: u32,
}

/// What the control thread did while it waited and while it held the lock, from
/// `getrusage(RUSAGE_THREAD)` and `CLOCK_THREAD_CPUTIME_ID` around the phases (Linux; zero
/// elsewhere). Valid for the control thread, which polls the tick on one OS thread.
#[derive(Debug, Clone, Copy, Default, Serialize, PartialEq, Eq)]
pub struct ThreadUsage {
    /// CPU time of the thread during the lock wait (wall minus this: off CPU).
    pub wait_cpu_us: u32,
    /// CPU time of the thread while it held the lock.
    pub lock_cpu_us: u32,
    /// Voluntary / involuntary context switches during the wait (voluntary: parked).
    pub wait_vcsw: u32,
    pub wait_ivcsw: u32,
    /// Voluntary (blocked, slept) / involuntary (preempted) switches while holding the lock.
    pub lock_vcsw: u32,
    pub lock_ivcsw: u32,
    /// Voluntary / involuntary context switches across `lock_end → released` (Linux; 0 elsewhere).
    pub mid_nvcsw: u32,
    pub mid_nivcsw: u32,
    /// Major / minor page faults during the wait and during the hold.
    pub wait_majflt: u32,
    pub lock_majflt: u32,
    pub lock_minflt: u32,
    /// Allocator calls while holding the lock, their total and longest time (µs); zero
    /// unless armd runs with `ARMD_ALLOC_TIMING=1` (src/diagnostics/alloc.rs).
    pub lock_alloc_calls: u32,
    pub lock_alloc_us: u32,
    pub lock_alloc_max_us: u32,
    /// CAN frames successfully transmitted during the tick.
    pub tx_frames: u32,
    /// `WouldBlock` results during CAN TX attempts in this tick.
    pub tx_would_block: u32,
    /// `ENOBUFS` results during CAN TX attempts in this tick.
    pub tx_enobufs: u32,
    /// Commands newly deferred to a later tick by backpressure or the TX budget.
    pub tx_deferred: u32,
    /// End-of-tick pending TX queue depth (gauge).
    pub tx_pending: u32,
    /// Queued frames dropped due to age expiry (`age >= frame_max_age`).
    pub tx_dropped_stale: u32,
    /// Queued frames dropped due to stop / E-stop / disable / mode switch / error.
    pub tx_dropped_stopped: u32,
    /// Sum of single-frame `can.transmit` wall times in this tick, µs.
    pub tx_send_us: u32,
    /// Sum of single-frame `can.transmit` thread CPU times (`CLOCK_THREAD_CPUTIME_ID`), µs.
    pub tx_send_cpu_us: u32,
    /// Maximum single-frame `can.transmit` wall time in this tick, µs.
    pub tx_frame_max_us: u32,
    /// Maximum single-frame `can.transmit` thread CPU time (`CLOCK_THREAD_CPUTIME_ID`), µs.
    pub tx_frame_cpu_max_us: u32,
}

/// The guard of another task the tick waited for: the last one released before the tick
/// got the lock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Holder {
    pub kind: &'static str,
    pub location: &'static std::panic::Location<'static>,
    /// Static WS/command path attributed to this hold (zero allocation on hot path).
    pub command: Option<&'static str>,
    /// Optional static direction tag (e.g. `"on"` / `"off"` for `/api/v1/torque`).
    pub direction: Option<&'static str>,
    /// Action id created or installed under this hold, if any.
    pub action_id: Option<u64>,
    /// Optional TX / RX / sleep timing (µs) spent under this hold.
    pub io_timing: Option<HoldIoTiming>,
    pub hold_us: u32,
    pub acquire_us: u32,
    /// How long before the tick started waiting the holder took the lock (negative: after).
    pub acquired_before_wait_us: i64,
    /// How long before the tick got the lock the holder released it: the tick's wake-up
    /// latency when the tick waited for this holder.
    pub released_before_lock_us: i64,
}

impl Serialize for Holder {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        use serde::ser::SerializeStruct;
        let field_count = 6
            + usize::from(self.command.is_some())
            + usize::from(self.direction.is_some())
            + usize::from(self.action_id.is_some())
            + if self.io_timing.is_some() { 3 } else { 0 };
        let mut st = s.serialize_struct("Holder", field_count)?;
        st.serialize_field("kind", self.kind)?;
        st.serialize_field("location", &format!("{}:{}", self.location.file(), self.location.line()))?;
        if let Some(cmd) = self.command {
            st.serialize_field("command", cmd)?;
        }
        if let Some(dir) = self.direction {
            st.serialize_field("direction", dir)?;
        }
        if let Some(id) = self.action_id {
            st.serialize_field("action_id", &id)?;
        }
        if let Some(io) = self.io_timing {
            st.serialize_field("tx_us", &io.tx_us)?;
            st.serialize_field("rx_us", &io.rx_us)?;
            st.serialize_field("sleep_us", &io.sleep_us)?;
        }
        st.serialize_field("hold_us", &self.hold_us)?;
        st.serialize_field("acquire_us", &self.acquire_us)?;
        st.serialize_field("acquired_before_wait_us", &self.acquired_before_wait_us)?;
        st.serialize_field("released_before_lock_us", &self.released_before_lock_us)?;
        st.end()
    }
}

/// Release-event bitflags recorded on [`TickSample::release_flags`] by the RT executor.
pub const RELEASE_FLAG_SAFETY_LATCH: u32 = 1 << 0;
pub const RELEASE_FLAG_ESTOP_APPLIED: u32 = 1 << 1;
pub const RELEASE_FLAG_CLEAR_OK: u32 = 1 << 2;
pub const RELEASE_FLAG_CLEAR_REFUSED: u32 = 1 << 3;
pub const RELEASE_FLAG_INJECT_ALLOC: u32 = 1 << 4;
pub const RELEASE_FLAG_THERMAL_TRIP: u32 = 1 << 5;
pub const RELEASE_FLAG_WATCHDOG_LATCH: u32 = 1 << 6;

/// One tick, µs. Serialized as the breakdown of `max_tick` / `max_release` / `max_lock`.
#[derive(Debug, Clone, Copy, Default, Serialize, PartialEq, Eq)]
pub struct TickSample {
    /// Whole `ArmSupervisor::tick` / executor body call (kept for backward compatibility).
    pub tick_us: u32,
    /// Full release duration measured on the control thread from wake (right after lateness
    /// sampling) through control, TX, usage collection, publication, dispatcher wake, cleanup,
    /// and stats publication preparation, µs.
    pub release_us: u32,
    /// Waiting for the supervisor write lock.
    pub wait_us: u32,
    /// Holding the supervisor write lock.
    pub lock_us: u32,
    /// After the lock: telemetry broadcast, spawned persist job.
    pub post_us: u32,
    /// `dt` of the tick: start-to-start interval of the control loop.
    pub period_us: u32,
    /// Wakeup lateness `actual_release_start - scheduled_release`, µs (0 when driven directly).
    pub wake_lateness_us: u32,
    /// Longest read / write hold by another task released since the previous tick.
    pub reader_hold_us: u32,
    pub writer_hold_us: u32,
    /// Longest `read()` / `write()` of another task released since the previous tick: long
    /// when the lock was handed to a task that was not scheduled.
    pub reader_acquire_us: u32,
    pub writer_acquire_us: u32,
    /// Wait for the tick-stats mutex of the previous tick (not part of `tick_us`).
    pub stats_wait_us: u32,
    #[serde(flatten)]
    pub phases: LockPhases,
    /// Monotonic sub-phase deltas (µs) across `lock_end → release completion`.
    pub phases_us: ReleasePhasesUs,
    /// Overrun-only (`release >= 15 ms`) monotonic phase breakdown (µs) across
    /// `step return → ring publication → completion seal` (`None` on non-overrun releases).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub post_phases_us: Option<PostOverrunPhasesUs>,
    /// Overrun-only thread CPU time (`CLOCK_THREAD_CPUTIME_ID`, µs) across the same post-step
    /// span (`None` on non-overrun releases).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub post_cpu_us: Option<u32>,
    #[serde(flatten)]
    pub usage: ThreadUsage,
    /// The guard the tick waited for; None when no other guard was released since the
    /// previous tick (or the log was busy).
    pub waited_for: Option<Holder>,
    /// The longest other hold since the previous tick (may differ from `waited_for`).
    pub longest_hold: Option<Holder>,
    /// Torque transition step attribution recorded when this control tick serviced a torque
    /// handshake (`None` on ordinary ticks; omitted from JSON when absent).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub torque_step: Option<TorqueStepAttribution>,
    #[serde(skip)]
    pub late: bool,
    /// Monotonic RT executor release sequence (`1`-based; `0` outside RT executor).
    #[serde(skip)]
    pub release_seq: u64,
    /// Safety epoch at the end of this release (`0` outside RT executor).
    #[serde(skip)]
    pub safety_epoch: u64,
    /// Maintenance request id serviced during this release (`0` when none).
    #[serde(skip)]
    pub maint_req_id: u64,
    /// Bitmask of `RELEASE_FLAG_*` safety events executed during this release.
    #[serde(skip)]
    pub release_flags: u32,
}

/// Cumulative since-boot TX event totals kept by [`TickStats`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct TxTotals {
    frames: u64,
    would_block: u64,
    enobufs: u64,
    deferred: u64,
    dropped_stale: u64,
    dropped_stopped: u64,
}

/// Ring buffer of per-tick samples. Recording is O(1); percentiles are computed on request
/// from a copy, outside the mutex that guards the ring.
pub struct TickStats {
    rows: Box<[TickSample]>,
    len: usize,
    next: usize,
    late_count: u64,
    ticks: u64,
    tx_totals: TxTotals,
}

#[derive(Debug, Clone, Copy, Default, Serialize, PartialEq, Eq)]
pub struct Percentiles {
    pub p50: u32,
    pub p99: u32,
    pub max: u32,
}

/// A recorded tick with its position in the window.
#[derive(Debug, Clone, Copy, Serialize, PartialEq, Eq)]
pub struct TickBreakdown {
    #[serde(flatten)]
    pub sample: TickSample,
    /// Holder release timestamp → tick acquire timestamp (µs) when `waited_for` is present.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub handoff_us: Option<i64>,
    /// Ticks recorded after this one (0: the latest tick).
    pub ticks_ago: usize,
}

/// Scheduling of the control thread as read back after the real-time request.
#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
pub struct ControlThreadSched {
    /// `"fifo"`, `"rr"`, `"other"` or the raw policy number.
    pub policy: String,
    pub priority: i32,
    /// Whether the SCHED_FIFO request succeeded.
    pub realtime: bool,
    /// Priority asked for (`ARMD_RT_PRIORITY`, default 79; `crate::rt_core::sched`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub requested_priority: Option<i32>,
    /// CPU the executor is pinned to (`ARMD_RT_CPU`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cpu: Option<usize>,
    /// Whether `mlockall(MCL_CURRENT | MCL_FUTURE)` succeeded at startup.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub memory_locked: Option<bool>,
    /// `"locked"`, `"skipped: ..."` or `"failed: ..."`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub memory_lock: Option<String>,
    /// Scheduling requests that failed.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub errors: Vec<String>,
}

/// Observations served by `GET /api/v1/observation` from the tick's published sample
/// (`hits`, `crate::rt_core::publish`) or computed under the supervisor read lock
/// (`misses`: no control loop running); a hit after waiting for the read lock (a tick
/// published meanwhile) counts as a hit.
#[derive(Debug, Clone, Copy, Default, Serialize, PartialEq, Eq)]
pub struct ObservationCacheStats {
    pub hits: u64,
    pub misses: u64,
    /// Always 0 since 1.6: the tick publishes without a mutex and never skips (kept for
    /// clients of 1.5).
    pub publish_skipped: u64,
    /// Hits that waited for the next tick's sample (a write or an expiry since the last one).
    #[serde(default)]
    pub waited: u64,
    /// Requests answered 503 `OBSERVATION_UNAVAILABLE` (no sample and the lock stayed busy).
    #[serde(default)]
    pub unavailable: u64,
    /// Displaced observations the tick freed itself because no reader drained the retire
    /// ring (0 while the dispatcher runs).
    #[serde(default)]
    pub tick_frees: u64,
    /// Samples published by the tick so far.
    #[serde(default)]
    pub published: u64,
}

/// Edge-triggered terminal-observation capture counters (`armd-control` -> off-thread drain).
#[derive(Debug, Clone, Copy, Default, Serialize, PartialEq, Eq)]
pub struct TerminalCaptureStats {
    /// Edge-triggered terminal records captured on the control release path since boot.
    pub captured: u64,
    /// Terminal records dropped because the preallocated SPSC capture ring was saturated.
    pub saturated: u64,
    /// Latched indicator set true if capture ring saturation has ever occurred since boot.
    pub ever_saturated: bool,
    /// Captured records currently queued in flight awaiting off-thread drain.
    pub queued: u64,
}

/// Percentiles of the tick's lock phases.
#[derive(Debug, Clone, Copy, Default, Serialize, PartialEq, Eq)]
pub struct LockPhasePercentiles {
    pub pre_us: Percentiles,
    pub rx_us: Percentiles,
    pub control_us: Percentiles,
    pub tx_us: Percentiles,
    pub finish_us: Percentiles,
    pub obs_us: Percentiles,
    pub publish_us: Percentiles,
}

/// Per-frame CAN TX timing and queue-pressure summary over the tick window
/// (`GET /api/v1/diagnostics/tick` -> `tx`).
#[derive(Debug, Clone, Copy, Default, Serialize, PartialEq, Eq)]
pub struct TxDiagnostics {
    /// Frames transmitted per tick (p50 / p99 / max over the window).
    pub frames: Percentiles,
    /// End-of-tick pending TX queue depth (gauge percentiles over the window).
    pub pending: Percentiles,
    /// Sum of single-frame `transmit` wall times per tick, µs.
    pub send_us: Percentiles,
    /// Sum of single-frame `transmit` thread CPU times (`CLOCK_THREAD_CPUTIME_ID`) per tick, µs.
    pub send_cpu_us: Percentiles,
    /// Maximum single-frame `transmit` wall time per tick, µs.
    pub frame_max_us: Percentiles,
    /// Maximum single-frame `transmit` thread CPU time (`CLOCK_THREAD_CPUTIME_ID`) per tick, µs.
    pub frame_cpu_max_us: Percentiles,
    /// Event sums across the ticks in the window.
    pub frames_total: u64,
    pub would_block_total: u64,
    pub enobufs_total: u64,
    pub deferred_total: u64,
    pub dropped_stale_total: u64,
    pub dropped_stopped_total: u64,
    /// Monotonic event sums since boot (not limited to the window).
    pub since_boot_frames: u64,
    pub since_boot_would_block: u64,
    pub since_boot_enobufs: u64,
    pub since_boot_deferred: u64,
    pub since_boot_dropped_stale: u64,
    pub since_boot_dropped_stopped: u64,
    /// Stopped-hold TX ticks skipped because `driver.try_lock()` was contended (L3).
    #[serde(default)]
    pub stopped_hold_lock_contended: u64,
    /// Stopped-hold `try_lock` contention escalations triggered after 25 consecutive skips (0.5 s).
    #[serde(default)]
    pub stopped_hold_contention_escalations: u64,
    /// RT disable dispatch/poll ticks skipped because `driver.try_lock()` was contended (M4).
    #[serde(default)]
    pub disable_lock_contended: u64,
    /// RT disable `try_lock` contention escalations triggered after 25 consecutive skips (0.5 s).
    #[serde(default)]
    pub disable_contention_escalations: u64,
}

/// Coherent per-run baseline deltas and peaks since the latest operator
/// `POST /api/v1/diagnostics/tick/mark`, acknowledged by the control thread at a single
/// release completion boundary.
#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
pub struct TickSinceMarker {
    pub marker_id: u64,
    /// `"pending"` until the control thread completes a release and captures the coherent
    /// baseline; `"active"` once acknowledged.
    pub status: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub releases_total_delta: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub releases_over_10ms_delta: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub releases_over_15ms_delta: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub releases_over_20ms_delta: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub skipped_releases_delta: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub diag_dropped_delta: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wake_lateness_over_1ms_delta: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wake_lateness_over_5ms_delta: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wake_lateness_over_10ms_delta: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub release_us_max: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wake_late_max: Option<u64>,
}

#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
pub struct TickSnapshot {
    /// Unique identifier for this daemon/supervisor instance (changes on every restart).
    #[serde(default)]
    pub instance_id: String,
    /// Kernel boot UUID (`/proc/sys/kernel/random/boot_id` on Linux), or `None` if unavailable.
    #[serde(default)]
    pub boot_id: Option<String>,
    /// Process start wall-clock timestamp (Unix epoch nanoseconds) derived once at startup from
    /// Linux `/proc/stat` `btime` + `/proc/self/stat` `starttime`, or `None` if unavailable.
    #[serde(default)]
    pub process_start_unix_ns: Option<u64>,
    /// Ticks in the window (at most [`TICK_WINDOW`]).
    pub window: usize,
    /// Whole `ArmSupervisor::tick` call: lock wait, tick body and telemetry broadcast.
    pub tick_us: Percentiles,
    /// Full release duration on the control thread from wake through all release work, µs.
    pub release_us: Percentiles,
    /// Monotonic releases started since process start (control-thread incremented at wake).
    pub releases_started: u64,
    /// Monotonic completed releases since process start (control-thread written only).
    pub releases_total: u64,
    /// Elapsed wall duration (µs) of a currently in-flight control release, or 0 when idle.
    pub in_flight_release_us: u64,
    /// Monotonic all-time maximum full `release_us` since process start, µs (never ages out).
    pub release_us_max_ever: u64,
    /// Monotonic releases since process start with full duration `>= 10 ms` / `>= 15 ms` / `>= 20 ms`.
    pub releases_over_10ms: u64,
    pub releases_over_15ms: u64,
    pub releases_over_20ms: u64,
    /// Monotonic all-time maximum wakeup lateness since process start, µs (never ages out).
    pub wake_late_max_ever: u64,
    /// Time the supervisor write lock was held by the tick.
    pub lock_us: Percentiles,
    /// Ticks since start whose `dt` exceeded two control periods.
    pub late_count: u64,
    pub since_boot_ticks: u64,
    /// Time the tick waited for the supervisor write lock.
    pub wait_us: Percentiles,
    /// Time after the lock was released (telemetry broadcast).
    pub post_us: Percentiles,
    /// Start-to-start interval of the control loop (`dt`).
    pub period_us: Percentiles,
    /// Wakeup lateness (`actual_release_start - scheduled_release`) over the window, µs.
    pub wake_lateness_us: Percentiles,
    /// Cumulative releases since boot with wakeup lateness strictly greater than 1 ms / 5 ms / 10 ms.
    pub wake_lateness_over_1ms: u64,
    pub wake_lateness_over_5ms: u64,
    pub wake_lateness_over_10ms: u64,
    /// Cumulative scheduled releases skipped since boot (`ExecutorStats::skipped_releases` in
    /// executor mode, or `run_periodic`'s skip counter in legacy mode).
    pub skipped_releases: u64,
    /// Cumulative diagnostic `TickSample` values dropped because the off-thread ring was full
    /// (executor mode; 0 in legacy mode).
    pub diag_dropped: u64,
    /// Active control-loop owner mode (`"legacy"`, `"transferring"`, `"executor"`, `"failed"`).
    pub owner_mode: &'static str,
    /// Effective control-loop owner (`"legacy"` or `"executor"`).
    pub effective_owner: &'static str,
    /// Why the daemon is running under `effective_owner`.
    pub owner_reason: &'static str,
    /// True while the owning periodic control loop is actively running.
    #[serde(default)]
    pub loop_active: bool,
    /// Longest read-lock hold by another task per tick interval (includes preemption of that task).
    pub reader_hold_us: Percentiles,
    /// Longest write-lock hold by another task per tick interval.
    pub writer_hold_us: Percentiles,
    /// Wait for the tick-stats mutex (contention with this route).
    pub stats_wait_us: Percentiles,
    /// Longest `read().await` of another task per tick interval.
    pub reader_acquire_us: Percentiles,
    /// Longest `write().await` of another task per tick interval.
    pub writer_acquire_us: Percentiles,
    /// The tick's lock hold split into phases.
    pub lock_phases: LockPhasePercentiles,
    /// The slowest tick in the window (`tick_us`), all phases.
    pub max_tick: Option<TickBreakdown>,
    /// The slowest full release in the window (`release_us`), all phases.
    pub max_release: Option<TickBreakdown>,
    /// The tick with the longest lock hold in the window, all phases.
    pub max_lock: Option<TickBreakdown>,
    /// The tick with the largest supervisor-lock wait (`wait_us`) in the window, all phases.
    pub max_wait: Option<TickBreakdown>,
    /// The tick with the largest summed CAN TX send wall time in the window.
    pub max_tx: Option<TickBreakdown>,
    /// Per-frame CAN TX timing and backpressure metrics over the window.
    pub tx: TxDiagnostics,
    /// Control thread scheduling; None until the control thread reported it (tests).
    pub control_thread: Option<ControlThreadSched>,
    pub observation_cache: ObservationCacheStats,
    /// Edge-triggered terminal-observation capture counters (`armd-control` -> off-thread drain).
    #[serde(default)]
    pub terminal_capture: TerminalCaptureStats,
    /// Monotonic count of deferred safety log events dropped on overflow (never reset).
    #[serde(default)]
    pub safety_log_dropped: u64,
    /// Monotonic count of log lines dropped by registered non-blocking log writers (never reset).
    #[serde(default)]
    pub log_writer_dropped: u64,
    /// True when a lock-free stats reader exhausted bounded retries due to concurrent slot reuse
    /// or writer contention (explicit coverage loss; `window` is 0 on a lapped snapshot).
    #[serde(default)]
    pub stats_snapshot_lapped: bool,
    /// Cumulative lock-free seqlock reader retries observed on `LockFreeTickStats`.
    #[serde(default)]
    pub stats_reader_retries: u64,
    /// Cumulative non-blocking writer contention drops on `LockFreeTickStats` (0 with single control writer).
    #[serde(default)]
    pub stats_writer_drops: u64,
    /// Coherent deltas and peaks since the latest `POST /api/v1/diagnostics/tick/mark`, if requested.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub since_marker: Option<TickSinceMarker>,
    /// Cumulative forbidden lock attempts from `armd-control` (`SupervisorLock` + `DriverSlot` +
    /// terminal capture) when the RT executor owns the arm (`None` in legacy mode).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub control_thread_lock_attempts: Option<u64>,
    /// Cumulative control-thread heap ops across whole releases and sub-release wakes when
    /// `ARMD_ALLOC_TIMING=1` is active in executor mode (`None` otherwise).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub control_thread_alloc_ops: Option<u64>,
    /// True when `ARMD_ALLOC_TIMING=1` is active on the owning control thread.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub alloc_timing_enabled: Option<bool>,
    /// Cumulative completed sub-release transition wakes in executor mode (`None` in legacy mode).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub executor_wakes: Option<u64>,
}

impl Default for TickStats {
    fn default() -> Self {
        Self {
            rows: vec![TickSample::default(); TICK_WINDOW].into_boxed_slice(),
            len: 0,
            next: 0,
            late_count: 0,
            ticks: 0,
            tx_totals: TxTotals::default(),
        }
    }
}

/// Thread CPU time and counters at one instant (see [`ThreadUsage`]).
#[derive(Debug, Clone, Copy, Default)]
pub struct UsageMark {
    cpu_ns: u64,
    vcsw: i64,
    ivcsw: i64,
    majflt: i64,
    minflt: i64,
    tx: TxTickMetrics,
}

impl UsageMark {
    /// Two syscalls on Linux (zero elsewhere), plus taking and resetting the calling thread's
    /// accumulated per-tick CAN TX counters (zero allocation).
    pub fn now() -> Self {
        let tx = take_tx_metrics();
        #[cfg(target_os = "linux")]
        {
            // SAFETY: plain syscalls on the calling thread with valid out-pointers.
            unsafe {
                let mut ru: libc::rusage = std::mem::zeroed();
                libc::getrusage(libc::RUSAGE_THREAD, &mut ru);
                UsageMark {
                    cpu_ns: thread_cpu_ns(),
                    vcsw: ru.ru_nvcsw as i64,
                    ivcsw: ru.ru_nivcsw as i64,
                    majflt: ru.ru_majflt as i64,
                    minflt: ru.ru_minflt as i64,
                    tx,
                }
            }
        }
        #[cfg(not(target_os = "linux"))]
        UsageMark { tx, ..UsageMark::default() }
    }

    /// Thread CPU time (`CLOCK_THREAD_CPUTIME_ID`, ns) captured by this mark (0 outside Linux).
    #[inline]
    pub fn cpu_ns(&self) -> u64 {
        self.cpu_ns
    }
}

/// Lightweight thread context-switch mark from `getrusage(RUSAGE_THREAD)` (Linux; zero elsewhere).
/// Does not touch `TX_TICK` or `CLOCK_THREAD_CPUTIME_ID` (zero allocation).
#[derive(Debug, Clone, Copy, Default)]
pub struct CswMark {
    vcsw: i64,
    ivcsw: i64,
}

impl CswMark {
    /// One syscall on Linux (zero elsewhere), zero allocation.
    #[inline]
    pub fn now() -> Self {
        #[cfg(target_os = "linux")]
        {
            // SAFETY: plain syscall on the calling thread with a valid out-pointer.
            unsafe {
                let mut ru: libc::rusage = std::mem::zeroed();
                libc::getrusage(libc::RUSAGE_THREAD, &mut ru);
                CswMark {
                    vcsw: ru.ru_nvcsw as i64,
                    ivcsw: ru.ru_nivcsw as i64,
                }
            }
        }
        #[cfg(not(target_os = "linux"))]
        CswMark::default()
    }
}

/// Voluntary and involuntary context-switch deltas between a `CswMark` (at `lock_end`)
/// and a `UsageMark` (at `released`).
#[inline]
pub fn mid_csw_delta(lock_end: &CswMark, released: &UsageMark) -> (u32, u32) {
    let n = |a: i64, b: i64| u32::try_from(b.saturating_sub(a).max(0)).unwrap_or(u32::MAX);
    (n(lock_end.vcsw, released.vcsw), n(lock_end.ivcsw, released.ivcsw))
}

/// [`ThreadUsage`] from marks at tick start, lock acquired and lock released.
pub fn thread_usage(start: &UsageMark, locked: &UsageMark, released: &UsageMark) -> ThreadUsage {
    let n = |a: i64, b: i64| u32::try_from(b.saturating_sub(a).max(0)).unwrap_or(u32::MAX);
    let cpu = |a: u64, b: u64| u32::try_from(b.saturating_sub(a) / 1000).unwrap_or(u32::MAX);
    let ns_to_us = |ns: u64| u32::try_from(ns / 1000).unwrap_or(u32::MAX);
    let tx = released.tx;
    ThreadUsage {
        wait_cpu_us: cpu(start.cpu_ns, locked.cpu_ns),
        lock_cpu_us: cpu(locked.cpu_ns, released.cpu_ns),
        wait_vcsw: n(start.vcsw, locked.vcsw),
        wait_ivcsw: n(start.ivcsw, locked.ivcsw),
        lock_vcsw: n(locked.vcsw, released.vcsw),
        lock_ivcsw: n(locked.ivcsw, released.ivcsw),
        wait_majflt: n(start.majflt, locked.majflt),
        lock_majflt: n(locked.majflt, released.majflt),
        lock_minflt: n(locked.minflt, released.minflt),
        tx_frames: tx.frames,
        tx_would_block: tx.would_block,
        tx_enobufs: tx.enobufs,
        tx_deferred: tx.deferred,
        tx_pending: tx.pending,
        tx_dropped_stale: tx.dropped_stale,
        tx_dropped_stopped: tx.dropped_stopped,
        tx_send_us: ns_to_us(tx.sum_wall_ns),
        tx_send_cpu_us: ns_to_us(tx.sum_cpu_ns),
        tx_frame_max_us: ns_to_us(tx.max_wall_ns),
        tx_frame_cpu_max_us: ns_to_us(tx.max_cpu_ns),
        ..ThreadUsage::default()
    }
}

/// Microseconds, saturating.
pub fn us(d: std::time::Duration) -> u32 {
    d.as_micros().min(u128::from(u32::MAX)) as u32
}

/// Nearest-rank percentiles of `values` (sorted in place).
fn percentiles(values: &mut [u32]) -> Percentiles {
    if values.is_empty() { return Percentiles::default(); }
    values.sort_unstable();
    let rank = |p: usize| values[(values.len() * p).div_ceil(100).max(1) - 1];
    Percentiles { p50: rank(50), p99: rank(99), max: values[values.len() - 1] }
}

/// Copy of the ring taken from [`TickStats`] or [`LockFreeTickStats`]; [`TickWindow::snapshot`]
/// sorts outside any writer path.
pub struct TickWindow {
    rows: Vec<TickSample>,
    late_count: u64,
    ticks: u64,
    tx_totals: TxTotals,
    snapshot_lapped: bool,
}

impl TickStats {
    pub fn record(&mut self, s: &TickSample) {
        self.rows[self.next] = *s;
        self.next = (self.next + 1) % TICK_WINDOW;
        self.len = (self.len + 1).min(TICK_WINDOW);
        self.ticks += 1;
        if s.late { self.late_count += 1; }
        self.tx_totals.frames = self.tx_totals.frames.saturating_add(u64::from(s.usage.tx_frames));
        self.tx_totals.would_block = self.tx_totals.would_block.saturating_add(u64::from(s.usage.tx_would_block));
        self.tx_totals.enobufs = self.tx_totals.enobufs.saturating_add(u64::from(s.usage.tx_enobufs));
        self.tx_totals.deferred = self.tx_totals.deferred.saturating_add(u64::from(s.usage.tx_deferred));
        self.tx_totals.dropped_stale = self.tx_totals.dropped_stale.saturating_add(u64::from(s.usage.tx_dropped_stale));
        self.tx_totals.dropped_stopped = self.tx_totals.dropped_stopped.saturating_add(u64::from(s.usage.tx_dropped_stopped));
    }

    /// Rows oldest first.
    pub fn window(&self) -> TickWindow {
        let start = if self.len < TICK_WINDOW { 0 } else { self.next };
        let rows = (0..self.len).map(|i| self.rows[(start + i) % TICK_WINDOW]).collect();
        TickWindow {
            rows,
            late_count: self.late_count,
            ticks: self.ticks,
            tx_totals: self.tx_totals,
            snapshot_lapped: false,
        }
    }

    pub fn snapshot(&self) -> TickSnapshot { self.window().snapshot() }
}

// ============================================================================
// Lock-Free Seqlock Ring (`LockFreeTickStats`)
// ============================================================================

#[derive(Default)]
struct AtomicHolder {
    /// 0 = None, 1 = "read", 2 = "write"
    kind: AtomicU8,
    location: AtomicPtr<std::panic::Location<'static>>,
    /// Null when `command` is `None`; otherwise points to the immutable `&'static str` bytes.
    command_ptr: AtomicPtr<u8>,
    command_len: AtomicUsize,
    /// Null when `direction` is `None`; otherwise points to the immutable `&'static str` bytes.
    direction_ptr: AtomicPtr<u8>,
    direction_len: AtomicUsize,
    has_action_id: AtomicBool,
    action_id: AtomicU64,
    has_io_timing: AtomicBool,
    io_tx_us: AtomicU32,
    io_rx_us: AtomicU32,
    io_sleep_us: AtomicU32,
    hold_us: AtomicU32,
    acquire_us: AtomicU32,
    acquired_before_wait_us: AtomicI64,
    released_before_lock_us: AtomicI64,
}

#[derive(Clone, Copy)]
struct RawHolder {
    kind: u8,
    location: *const std::panic::Location<'static>,
    command_ptr: *const u8,
    command_len: usize,
    direction_ptr: *const u8,
    direction_len: usize,
    has_action_id: bool,
    action_id: u64,
    has_io_timing: bool,
    io_tx_us: u32,
    io_rx_us: u32,
    io_sleep_us: u32,
    hold_us: u32,
    acquire_us: u32,
    acquired_before_wait_us: i64,
    released_before_lock_us: i64,
}

impl AtomicHolder {
    fn store_relaxed(&self, h: Option<Holder>) {
        match h {
            None => {
                self.kind.store(0, Ordering::Relaxed);
                self.location.store(std::ptr::null_mut(), Ordering::Relaxed);
                self.command_ptr.store(std::ptr::null_mut(), Ordering::Relaxed);
                self.command_len.store(0, Ordering::Relaxed);
                self.direction_ptr.store(std::ptr::null_mut(), Ordering::Relaxed);
                self.direction_len.store(0, Ordering::Relaxed);
                self.has_action_id.store(false, Ordering::Relaxed);
                self.action_id.store(0, Ordering::Relaxed);
                self.has_io_timing.store(false, Ordering::Relaxed);
                self.io_tx_us.store(0, Ordering::Relaxed);
                self.io_rx_us.store(0, Ordering::Relaxed);
                self.io_sleep_us.store(0, Ordering::Relaxed);
            }
            Some(h) => {
                let kind = if h.kind == "write" { 2u8 } else { 1u8 };
                self.kind.store(kind, Ordering::Relaxed);
                self.location.store(
                    h.location as *const std::panic::Location<'static> as *mut std::panic::Location<'static>,
                    Ordering::Relaxed,
                );
                match h.command {
                    Some(cmd) => {
                        self.command_ptr.store(cmd.as_ptr() as *mut u8, Ordering::Relaxed);
                        self.command_len.store(cmd.len(), Ordering::Relaxed);
                    }
                    None => {
                        self.command_ptr.store(std::ptr::null_mut(), Ordering::Relaxed);
                        self.command_len.store(0, Ordering::Relaxed);
                    }
                }
                match h.direction {
                    Some(dir) => {
                        self.direction_ptr.store(dir.as_ptr() as *mut u8, Ordering::Relaxed);
                        self.direction_len.store(dir.len(), Ordering::Relaxed);
                    }
                    None => {
                        self.direction_ptr.store(std::ptr::null_mut(), Ordering::Relaxed);
                        self.direction_len.store(0, Ordering::Relaxed);
                    }
                }
                match h.action_id {
                    Some(id) => {
                        self.has_action_id.store(true, Ordering::Relaxed);
                        self.action_id.store(id, Ordering::Relaxed);
                    }
                    None => {
                        self.has_action_id.store(false, Ordering::Relaxed);
                        self.action_id.store(0, Ordering::Relaxed);
                    }
                }
                match h.io_timing {
                    Some(io) => {
                        self.has_io_timing.store(true, Ordering::Relaxed);
                        self.io_tx_us.store(io.tx_us, Ordering::Relaxed);
                        self.io_rx_us.store(io.rx_us, Ordering::Relaxed);
                        self.io_sleep_us.store(io.sleep_us, Ordering::Relaxed);
                    }
                    None => {
                        self.has_io_timing.store(false, Ordering::Relaxed);
                        self.io_tx_us.store(0, Ordering::Relaxed);
                        self.io_rx_us.store(0, Ordering::Relaxed);
                        self.io_sleep_us.store(0, Ordering::Relaxed);
                    }
                }
                self.hold_us.store(h.hold_us, Ordering::Relaxed);
                self.acquire_us.store(h.acquire_us, Ordering::Relaxed);
                self.acquired_before_wait_us.store(h.acquired_before_wait_us, Ordering::Relaxed);
                self.released_before_lock_us.store(h.released_before_lock_us, Ordering::Relaxed);
            }
        }
    }

    fn load_raw_relaxed(&self) -> RawHolder {
        RawHolder {
            kind: self.kind.load(Ordering::Relaxed),
            location: self.location.load(Ordering::Relaxed) as *const std::panic::Location<'static>,
            command_ptr: self.command_ptr.load(Ordering::Relaxed) as *const u8,
            command_len: self.command_len.load(Ordering::Relaxed),
            direction_ptr: self.direction_ptr.load(Ordering::Relaxed) as *const u8,
            direction_len: self.direction_len.load(Ordering::Relaxed),
            has_action_id: self.has_action_id.load(Ordering::Relaxed),
            action_id: self.action_id.load(Ordering::Relaxed),
            has_io_timing: self.has_io_timing.load(Ordering::Relaxed),
            io_tx_us: self.io_tx_us.load(Ordering::Relaxed),
            io_rx_us: self.io_rx_us.load(Ordering::Relaxed),
            io_sleep_us: self.io_sleep_us.load(Ordering::Relaxed),
            hold_us: self.hold_us.load(Ordering::Relaxed),
            acquire_us: self.acquire_us.load(Ordering::Relaxed),
            acquired_before_wait_us: self.acquired_before_wait_us.load(Ordering::Relaxed),
            released_before_lock_us: self.released_before_lock_us.load(Ordering::Relaxed),
        }
    }
}

impl RawHolder {
    /// Converts a validated raw holder into `Option<Holder>`.
    ///
    /// # Safety
    /// Must only be called after the enclosing seqlock read succeeded (`v1 == v2`, even, Acquire-ordered
    /// with the writer's Release store) so `self.location` is guaranteed to be either null or an
    /// immutable `&'static std::panic::Location<'static>` pointer and `(command_ptr, command_len)` /
    /// `(direction_ptr, direction_len)` are either null or immutable `&'static str` slices written by
    /// `AtomicHolder::store_relaxed`.
    unsafe fn into_holder_validated(self) -> Option<Holder> {
        if self.kind == 0 || self.location.is_null() {
            return None;
        }
        // SAFETY: upheld by caller's validated seqlock read and `'static` location provenance.
        let location: &'static std::panic::Location<'static> = unsafe { &*self.location };
        let command = if self.command_ptr.is_null() {
            None
        } else {
            // SAFETY: upheld by caller's validated seqlock read and `'static str` provenance.
            let bytes: &'static [u8] =
                unsafe { std::slice::from_raw_parts(self.command_ptr, self.command_len) };
            // SAFETY: bytes were obtained from a valid `&'static str`.
            Some(unsafe { std::str::from_utf8_unchecked(bytes) })
        };
        let direction = if self.direction_ptr.is_null() {
            None
        } else {
            // SAFETY: upheld by caller's validated seqlock read and `'static str` provenance.
            let bytes: &'static [u8] =
                unsafe { std::slice::from_raw_parts(self.direction_ptr, self.direction_len) };
            // SAFETY: bytes were obtained from a valid `&'static str`.
            Some(unsafe { std::str::from_utf8_unchecked(bytes) })
        };
        let action_id = self.has_action_id.then_some(self.action_id);
        let io_timing = self.has_io_timing.then_some(HoldIoTiming {
            tx_us: self.io_tx_us,
            rx_us: self.io_rx_us,
            sleep_us: self.io_sleep_us,
        });
        let kind = if self.kind == 2 { "write" } else { "read" };
        Some(Holder {
            kind,
            location,
            command,
            direction,
            action_id,
            io_timing,
            hold_us: self.hold_us,
            acquire_us: self.acquire_us,
            acquired_before_wait_us: self.acquired_before_wait_us,
            released_before_lock_us: self.released_before_lock_us,
        })
    }
}

#[derive(Default)]
struct AtomicTickSlot {
    /// Per-slot seqlock counter: even = stable, odd = write in progress.
    seq: AtomicU64,
    /// 1-based monotonic sample sequence stored in this slot.
    sample_seq: AtomicU64,
    tick_us: AtomicU32,
    release_us: AtomicU32,
    wait_us: AtomicU32,
    lock_us: AtomicU32,
    post_us: AtomicU32,
    period_us: AtomicU32,
    wake_lateness_us: AtomicU32,
    reader_hold_us: AtomicU32,
    writer_hold_us: AtomicU32,
    reader_acquire_us: AtomicU32,
    writer_acquire_us: AtomicU32,
    stats_wait_us: AtomicU32,
    // LockPhases
    pre_us: AtomicU32,
    rx_us: AtomicU32,
    control_us: AtomicU32,
    tx_us: AtomicU32,
    finish_us: AtomicU32,
    obs_us: AtomicU32,
    publish_us: AtomicU32,
    // ReleasePhasesUs (`phases_us`)
    phase_alloc_read: AtomicU32,
    phase_ack_flush: AtomicU32,
    phase_terminal: AtomicU32,
    phase_guard_drop: AtomicU32,
    phase_post: AtomicU32,
    phase_publish: AtomicU32,
    // Overrun-only PostOverrunPhasesUs (`post_phases_us`) + `post_cpu_us`
    has_post_overrun: AtomicBool,
    post_step_to_ring_us: AtomicU32,
    post_ring_publish_us: AtomicU32,
    post_ring_to_seal_us: AtomicU32,
    post_seal_us: AtomicU32,
    post_cpu_us: AtomicU32,
    // ThreadUsage
    wait_cpu_us: AtomicU32,
    lock_cpu_us: AtomicU32,
    wait_vcsw: AtomicU32,
    wait_ivcsw: AtomicU32,
    lock_vcsw: AtomicU32,
    lock_ivcsw: AtomicU32,
    mid_nvcsw: AtomicU32,
    mid_nivcsw: AtomicU32,
    wait_majflt: AtomicU32,
    lock_majflt: AtomicU32,
    lock_minflt: AtomicU32,
    lock_alloc_calls: AtomicU32,
    lock_alloc_us: AtomicU32,
    lock_alloc_max_us: AtomicU32,
    tx_frames: AtomicU32,
    tx_would_block: AtomicU32,
    tx_enobufs: AtomicU32,
    tx_deferred: AtomicU32,
    tx_pending: AtomicU32,
    tx_dropped_stale: AtomicU32,
    tx_dropped_stopped: AtomicU32,
    tx_send_us: AtomicU32,
    tx_send_cpu_us: AtomicU32,
    tx_frame_max_us: AtomicU32,
    tx_frame_cpu_max_us: AtomicU32,
    waited_for: AtomicHolder,
    longest_hold: AtomicHolder,
    has_torque_step: AtomicBool,
    torque_step_cmd_ptr: AtomicPtr<u8>,
    torque_step_cmd_len: AtomicUsize,
    torque_step_dir_ptr: AtomicPtr<u8>,
    torque_step_dir_len: AtomicUsize,
    torque_step_tx_us: AtomicU32,
    torque_step_rx_us: AtomicU32,
    torque_step_sleep_us: AtomicU32,
    late: AtomicBool,
    release_seq: AtomicU64,
    safety_epoch: AtomicU64,
    maint_req_id: AtomicU64,
    release_flags: AtomicU32,
}

#[derive(Clone, Copy)]
struct RawTickSlot {
    sample_seq: u64,
    tick_us: u32,
    release_us: u32,
    wait_us: u32,
    lock_us: u32,
    post_us: u32,
    period_us: u32,
    wake_lateness_us: u32,
    reader_hold_us: u32,
    writer_hold_us: u32,
    reader_acquire_us: u32,
    writer_acquire_us: u32,
    stats_wait_us: u32,
    phases: LockPhases,
    phases_us: ReleasePhasesUs,
    post_phases_us: Option<PostOverrunPhasesUs>,
    post_cpu_us: Option<u32>,
    usage: ThreadUsage,
    waited_for: RawHolder,
    longest_hold: RawHolder,
    has_torque_step: bool,
    torque_step_cmd_ptr: *const u8,
    torque_step_cmd_len: usize,
    torque_step_dir_ptr: *const u8,
    torque_step_dir_len: usize,
    torque_step_tx_us: u32,
    torque_step_rx_us: u32,
    torque_step_sleep_us: u32,
    late: bool,
    release_seq: u64,
    safety_epoch: u64,
    maint_req_id: u64,
    release_flags: u32,
}

impl AtomicTickSlot {
    fn store_relaxed(&self, sample_seq: u64, s: &TickSample) {
        self.sample_seq.store(sample_seq, Ordering::Relaxed);
        self.tick_us.store(s.tick_us, Ordering::Relaxed);
        self.release_us.store(s.release_us, Ordering::Relaxed);
        self.wait_us.store(s.wait_us, Ordering::Relaxed);
        self.lock_us.store(s.lock_us, Ordering::Relaxed);
        self.post_us.store(s.post_us, Ordering::Relaxed);
        self.period_us.store(s.period_us, Ordering::Relaxed);
        self.wake_lateness_us.store(s.wake_lateness_us, Ordering::Relaxed);
        self.reader_hold_us.store(s.reader_hold_us, Ordering::Relaxed);
        self.writer_hold_us.store(s.writer_hold_us, Ordering::Relaxed);
        self.reader_acquire_us.store(s.reader_acquire_us, Ordering::Relaxed);
        self.writer_acquire_us.store(s.writer_acquire_us, Ordering::Relaxed);
        self.stats_wait_us.store(s.stats_wait_us, Ordering::Relaxed);
        self.pre_us.store(s.phases.pre_us, Ordering::Relaxed);
        self.rx_us.store(s.phases.rx_us, Ordering::Relaxed);
        self.control_us.store(s.phases.control_us, Ordering::Relaxed);
        self.tx_us.store(s.phases.tx_us, Ordering::Relaxed);
        self.finish_us.store(s.phases.finish_us, Ordering::Relaxed);
        self.obs_us.store(s.phases.obs_us, Ordering::Relaxed);
        self.publish_us.store(s.phases.publish_us, Ordering::Relaxed);
        self.phase_alloc_read.store(s.phases_us.alloc_read, Ordering::Relaxed);
        self.phase_ack_flush.store(s.phases_us.ack_flush, Ordering::Relaxed);
        self.phase_terminal.store(s.phases_us.terminal, Ordering::Relaxed);
        self.phase_guard_drop.store(s.phases_us.guard_drop, Ordering::Relaxed);
        self.phase_post.store(s.phases_us.post, Ordering::Relaxed);
        self.phase_publish.store(s.phases_us.publish, Ordering::Relaxed);
        match (s.post_phases_us, s.post_cpu_us) {
            (Some(pp), Some(cpu)) => {
                self.has_post_overrun.store(true, Ordering::Relaxed);
                self.post_step_to_ring_us.store(pp.step_to_ring_us, Ordering::Relaxed);
                self.post_ring_publish_us.store(pp.ring_publish_us, Ordering::Relaxed);
                self.post_ring_to_seal_us.store(pp.ring_to_seal_us, Ordering::Relaxed);
                self.post_seal_us.store(pp.seal_us, Ordering::Relaxed);
                self.post_cpu_us.store(cpu, Ordering::Relaxed);
            }
            (Some(pp), None) => {
                self.has_post_overrun.store(true, Ordering::Relaxed);
                self.post_step_to_ring_us.store(pp.step_to_ring_us, Ordering::Relaxed);
                self.post_ring_publish_us.store(pp.ring_publish_us, Ordering::Relaxed);
                self.post_ring_to_seal_us.store(pp.ring_to_seal_us, Ordering::Relaxed);
                self.post_seal_us.store(pp.seal_us, Ordering::Relaxed);
                self.post_cpu_us.store(0, Ordering::Relaxed);
            }
            (None, _) => {
                self.has_post_overrun.store(false, Ordering::Relaxed);
                self.post_step_to_ring_us.store(0, Ordering::Relaxed);
                self.post_ring_publish_us.store(0, Ordering::Relaxed);
                self.post_ring_to_seal_us.store(0, Ordering::Relaxed);
                self.post_seal_us.store(0, Ordering::Relaxed);
                self.post_cpu_us.store(0, Ordering::Relaxed);
            }
        }
        self.wait_cpu_us.store(s.usage.wait_cpu_us, Ordering::Relaxed);
        self.lock_cpu_us.store(s.usage.lock_cpu_us, Ordering::Relaxed);
        self.wait_vcsw.store(s.usage.wait_vcsw, Ordering::Relaxed);
        self.wait_ivcsw.store(s.usage.wait_ivcsw, Ordering::Relaxed);
        self.lock_vcsw.store(s.usage.lock_vcsw, Ordering::Relaxed);
        self.lock_ivcsw.store(s.usage.lock_ivcsw, Ordering::Relaxed);
        self.mid_nvcsw.store(s.usage.mid_nvcsw, Ordering::Relaxed);
        self.mid_nivcsw.store(s.usage.mid_nivcsw, Ordering::Relaxed);
        self.wait_majflt.store(s.usage.wait_majflt, Ordering::Relaxed);
        self.lock_majflt.store(s.usage.lock_majflt, Ordering::Relaxed);
        self.lock_minflt.store(s.usage.lock_minflt, Ordering::Relaxed);
        self.lock_alloc_calls.store(s.usage.lock_alloc_calls, Ordering::Relaxed);
        self.lock_alloc_us.store(s.usage.lock_alloc_us, Ordering::Relaxed);
        self.lock_alloc_max_us.store(s.usage.lock_alloc_max_us, Ordering::Relaxed);
        self.tx_frames.store(s.usage.tx_frames, Ordering::Relaxed);
        self.tx_would_block.store(s.usage.tx_would_block, Ordering::Relaxed);
        self.tx_enobufs.store(s.usage.tx_enobufs, Ordering::Relaxed);
        self.tx_deferred.store(s.usage.tx_deferred, Ordering::Relaxed);
        self.tx_pending.store(s.usage.tx_pending, Ordering::Relaxed);
        self.tx_dropped_stale.store(s.usage.tx_dropped_stale, Ordering::Relaxed);
        self.tx_dropped_stopped.store(s.usage.tx_dropped_stopped, Ordering::Relaxed);
        self.tx_send_us.store(s.usage.tx_send_us, Ordering::Relaxed);
        self.tx_send_cpu_us.store(s.usage.tx_send_cpu_us, Ordering::Relaxed);
        self.tx_frame_max_us.store(s.usage.tx_frame_max_us, Ordering::Relaxed);
        self.tx_frame_cpu_max_us.store(s.usage.tx_frame_cpu_max_us, Ordering::Relaxed);
        self.waited_for.store_relaxed(s.waited_for);
        self.longest_hold.store_relaxed(s.longest_hold);
        match s.torque_step {
            Some(ts) => {
                self.has_torque_step.store(true, Ordering::Relaxed);
                self.torque_step_cmd_ptr.store(ts.command.as_ptr() as *mut u8, Ordering::Relaxed);
                self.torque_step_cmd_len.store(ts.command.len(), Ordering::Relaxed);
                self.torque_step_dir_ptr.store(ts.direction.as_ptr() as *mut u8, Ordering::Relaxed);
                self.torque_step_dir_len.store(ts.direction.len(), Ordering::Relaxed);
                self.torque_step_tx_us.store(ts.tx_us, Ordering::Relaxed);
                self.torque_step_rx_us.store(ts.rx_us, Ordering::Relaxed);
                self.torque_step_sleep_us.store(ts.sleep_us, Ordering::Relaxed);
            }
            None => {
                self.has_torque_step.store(false, Ordering::Relaxed);
                self.torque_step_cmd_ptr.store(std::ptr::null_mut(), Ordering::Relaxed);
                self.torque_step_cmd_len.store(0, Ordering::Relaxed);
                self.torque_step_dir_ptr.store(std::ptr::null_mut(), Ordering::Relaxed);
                self.torque_step_dir_len.store(0, Ordering::Relaxed);
                self.torque_step_tx_us.store(0, Ordering::Relaxed);
                self.torque_step_rx_us.store(0, Ordering::Relaxed);
                self.torque_step_sleep_us.store(0, Ordering::Relaxed);
            }
        }
        self.late.store(s.late, Ordering::Relaxed);
        self.release_seq.store(s.release_seq, Ordering::Relaxed);
        self.safety_epoch.store(s.safety_epoch, Ordering::Relaxed);
        self.maint_req_id.store(s.maint_req_id, Ordering::Relaxed);
        self.release_flags.store(s.release_flags, Ordering::Relaxed);
    }

    fn load_raw_relaxed(&self) -> RawTickSlot {
        RawTickSlot {
            sample_seq: self.sample_seq.load(Ordering::Relaxed),
            tick_us: self.tick_us.load(Ordering::Relaxed),
            release_us: self.release_us.load(Ordering::Relaxed),
            wait_us: self.wait_us.load(Ordering::Relaxed),
            lock_us: self.lock_us.load(Ordering::Relaxed),
            post_us: self.post_us.load(Ordering::Relaxed),
            period_us: self.period_us.load(Ordering::Relaxed),
            wake_lateness_us: self.wake_lateness_us.load(Ordering::Relaxed),
            reader_hold_us: self.reader_hold_us.load(Ordering::Relaxed),
            writer_hold_us: self.writer_hold_us.load(Ordering::Relaxed),
            reader_acquire_us: self.reader_acquire_us.load(Ordering::Relaxed),
            writer_acquire_us: self.writer_acquire_us.load(Ordering::Relaxed),
            stats_wait_us: self.stats_wait_us.load(Ordering::Relaxed),
            phases: LockPhases {
                pre_us: self.pre_us.load(Ordering::Relaxed),
                rx_us: self.rx_us.load(Ordering::Relaxed),
                control_us: self.control_us.load(Ordering::Relaxed),
                tx_us: self.tx_us.load(Ordering::Relaxed),
                finish_us: self.finish_us.load(Ordering::Relaxed),
                obs_us: self.obs_us.load(Ordering::Relaxed),
                publish_us: self.publish_us.load(Ordering::Relaxed),
            },
            phases_us: ReleasePhasesUs {
                alloc_read: self.phase_alloc_read.load(Ordering::Relaxed),
                ack_flush: self.phase_ack_flush.load(Ordering::Relaxed),
                terminal: self.phase_terminal.load(Ordering::Relaxed),
                guard_drop: self.phase_guard_drop.load(Ordering::Relaxed),
                post: self.phase_post.load(Ordering::Relaxed),
                publish: self.phase_publish.load(Ordering::Relaxed),
            },
            post_phases_us: self.has_post_overrun.load(Ordering::Relaxed).then(|| PostOverrunPhasesUs {
                step_to_ring_us: self.post_step_to_ring_us.load(Ordering::Relaxed),
                ring_publish_us: self.post_ring_publish_us.load(Ordering::Relaxed),
                ring_to_seal_us: self.post_ring_to_seal_us.load(Ordering::Relaxed),
                seal_us: self.post_seal_us.load(Ordering::Relaxed),
            }),
            post_cpu_us: self
                .has_post_overrun
                .load(Ordering::Relaxed)
                .then(|| self.post_cpu_us.load(Ordering::Relaxed)),
            usage: ThreadUsage {
                wait_cpu_us: self.wait_cpu_us.load(Ordering::Relaxed),
                lock_cpu_us: self.lock_cpu_us.load(Ordering::Relaxed),
                wait_vcsw: self.wait_vcsw.load(Ordering::Relaxed),
                wait_ivcsw: self.wait_ivcsw.load(Ordering::Relaxed),
                lock_vcsw: self.lock_vcsw.load(Ordering::Relaxed),
                lock_ivcsw: self.lock_ivcsw.load(Ordering::Relaxed),
                mid_nvcsw: self.mid_nvcsw.load(Ordering::Relaxed),
                mid_nivcsw: self.mid_nivcsw.load(Ordering::Relaxed),
                wait_majflt: self.wait_majflt.load(Ordering::Relaxed),
                lock_majflt: self.lock_majflt.load(Ordering::Relaxed),
                lock_minflt: self.lock_minflt.load(Ordering::Relaxed),
                lock_alloc_calls: self.lock_alloc_calls.load(Ordering::Relaxed),
                lock_alloc_us: self.lock_alloc_us.load(Ordering::Relaxed),
                lock_alloc_max_us: self.lock_alloc_max_us.load(Ordering::Relaxed),
                tx_frames: self.tx_frames.load(Ordering::Relaxed),
                tx_would_block: self.tx_would_block.load(Ordering::Relaxed),
                tx_enobufs: self.tx_enobufs.load(Ordering::Relaxed),
                tx_deferred: self.tx_deferred.load(Ordering::Relaxed),
                tx_pending: self.tx_pending.load(Ordering::Relaxed),
                tx_dropped_stale: self.tx_dropped_stale.load(Ordering::Relaxed),
                tx_dropped_stopped: self.tx_dropped_stopped.load(Ordering::Relaxed),
                tx_send_us: self.tx_send_us.load(Ordering::Relaxed),
                tx_send_cpu_us: self.tx_send_cpu_us.load(Ordering::Relaxed),
                tx_frame_max_us: self.tx_frame_max_us.load(Ordering::Relaxed),
                tx_frame_cpu_max_us: self.tx_frame_cpu_max_us.load(Ordering::Relaxed),
            },
            waited_for: self.waited_for.load_raw_relaxed(),
            longest_hold: self.longest_hold.load_raw_relaxed(),
            has_torque_step: self.has_torque_step.load(Ordering::Relaxed),
            torque_step_cmd_ptr: self.torque_step_cmd_ptr.load(Ordering::Relaxed) as *const u8,
            torque_step_cmd_len: self.torque_step_cmd_len.load(Ordering::Relaxed),
            torque_step_dir_ptr: self.torque_step_dir_ptr.load(Ordering::Relaxed) as *const u8,
            torque_step_dir_len: self.torque_step_dir_len.load(Ordering::Relaxed),
            torque_step_tx_us: self.torque_step_tx_us.load(Ordering::Relaxed),
            torque_step_rx_us: self.torque_step_rx_us.load(Ordering::Relaxed),
            torque_step_sleep_us: self.torque_step_sleep_us.load(Ordering::Relaxed),
            late: self.late.load(Ordering::Relaxed),
            release_seq: self.release_seq.load(Ordering::Relaxed),
            safety_epoch: self.safety_epoch.load(Ordering::Relaxed),
            maint_req_id: self.maint_req_id.load(Ordering::Relaxed),
            release_flags: self.release_flags.load(Ordering::Relaxed),
        }
    }
}

impl RawTickSlot {
    /// # Safety
    /// Must only be called after the enclosing seqlock read succeeded (`v1 == v2`, even, Acquire-ordered).
    unsafe fn into_sample_validated(self) -> (u64, TickSample) {
        (
            self.sample_seq,
            TickSample {
                tick_us: self.tick_us,
                release_us: self.release_us,
                wait_us: self.wait_us,
                lock_us: self.lock_us,
                post_us: self.post_us,
                period_us: self.period_us,
                wake_lateness_us: self.wake_lateness_us,
                reader_hold_us: self.reader_hold_us,
                writer_hold_us: self.writer_hold_us,
                reader_acquire_us: self.reader_acquire_us,
                writer_acquire_us: self.writer_acquire_us,
                stats_wait_us: self.stats_wait_us,
                phases: self.phases,
                phases_us: self.phases_us,
                post_phases_us: self.post_phases_us,
                post_cpu_us: self.post_cpu_us,
                usage: self.usage,
                // SAFETY: seqlock read validated by caller.
                waited_for: unsafe { self.waited_for.into_holder_validated() },
                // SAFETY: seqlock read validated by caller.
                longest_hold: unsafe { self.longest_hold.into_holder_validated() },
                torque_step: if self.has_torque_step
                    && !self.torque_step_cmd_ptr.is_null()
                    && !self.torque_step_dir_ptr.is_null()
                {
                    // SAFETY: pointers and lengths originate from `'static str` written in `store_relaxed`,
                    // validated by the enclosing seqlock Acquire fence.
                    let cmd_bytes: &'static [u8] = unsafe {
                        std::slice::from_raw_parts(self.torque_step_cmd_ptr, self.torque_step_cmd_len)
                    };
                    let dir_bytes: &'static [u8] = unsafe {
                        std::slice::from_raw_parts(self.torque_step_dir_ptr, self.torque_step_dir_len)
                    };
                    Some(TorqueStepAttribution {
                        command: unsafe { std::str::from_utf8_unchecked(cmd_bytes) },
                        direction: unsafe { std::str::from_utf8_unchecked(dir_bytes) },
                        tx_us: self.torque_step_tx_us,
                        rx_us: self.torque_step_rx_us,
                        sleep_us: self.torque_step_sleep_us,
                    })
                } else {
                    None
                },
                late: self.late,
                release_seq: self.release_seq,
                safety_epoch: self.safety_epoch,
                maint_req_id: self.maint_req_id,
                release_flags: self.release_flags,
            },
        )
    }
}

/// Lock-free single-writer, multi-reader seqlock ring storing the last [`TICK_WINDOW`]
/// control-tick samples plus cumulative since-boot totals.
///
/// The control thread (in legacy mode) or off-thread `drain_diag` (in executor mode) publishes
/// complete [`TickSample`] values without ever taking a mutex or allocating. Concurrent HTTP
/// readers copy a coherent window lock-free with bounded retries and explicit coverage-loss
/// reporting (`stats_snapshot_lapped`) if stalled across ring reuse.
pub struct LockFreeTickStats {
    slots: Box<[AtomicTickSlot]>,
    writer_busy: AtomicBool,
    meta_seq: AtomicU64,
    write_seq: AtomicU64,
    late_count: AtomicU64,
    tx_frames: AtomicU64,
    tx_would_block: AtomicU64,
    tx_enobufs: AtomicU64,
    tx_deferred: AtomicU64,
    tx_dropped_stale: AtomicU64,
    tx_dropped_stopped: AtomicU64,
    reader_retries: AtomicU64,
    reader_window_retries: AtomicU64,
    reader_lapped_snapshots: AtomicU64,
    writer_contention_drops: AtomicU64,
}

impl std::fmt::Debug for LockFreeTickStats {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LockFreeTickStats")
            .field("write_seq", &self.write_seq.load(Ordering::Relaxed))
            .field("reader_retries", &self.reader_retries())
            .field("reader_lapped_snapshots", &self.reader_lapped_snapshots())
            .field("writer_contention_drops", &self.writer_contention_drops())
            .finish()
    }
}

impl Default for LockFreeTickStats {
    fn default() -> Self {
        Self::new()
    }
}

impl LockFreeTickStats {
    pub fn new() -> Self {
        const { assert!(LOCK_FREE_RING_CAPACITY.is_power_of_two() && LOCK_FREE_RING_CAPACITY > TICK_WINDOW) };
        let slots: Vec<AtomicTickSlot> = (0..LOCK_FREE_RING_CAPACITY)
            .map(|_| AtomicTickSlot::default())
            .collect();
        Self {
            slots: slots.into_boxed_slice(),
            writer_busy: AtomicBool::new(false),
            meta_seq: AtomicU64::new(0),
            write_seq: AtomicU64::new(0),
            late_count: AtomicU64::new(0),
            tx_frames: AtomicU64::new(0),
            tx_would_block: AtomicU64::new(0),
            tx_enobufs: AtomicU64::new(0),
            tx_deferred: AtomicU64::new(0),
            tx_dropped_stale: AtomicU64::new(0),
            tx_dropped_stopped: AtomicU64::new(0),
            reader_retries: AtomicU64::new(0),
            reader_window_retries: AtomicU64::new(0),
            reader_lapped_snapshots: AtomicU64::new(0),
            writer_contention_drops: AtomicU64::new(0),
        }
    }

    /// Wait-free publication of one complete sample (never blocks, never allocates).
    /// Returns `false` and increments `writer_contention_drops` only if two writers call
    /// `record` concurrently.
    pub fn record(&self, s: &TickSample) -> bool {
        self.record_staged(s, || s.release_us).is_some()
    }

    /// Wait-free staged publication: writes all sample payload fields into the next unpublished
    /// slot BEFORE invoking `finalize()` (which measures the final release completion timestamp
    /// and commits monotonic counters), stamps the authoritative `release_us` returned by
    /// `finalize()`, and publishes both the slot and metadata seqlocks.
    ///
    /// Returns `Some(sample_seq)` on success, or `None` (incrementing `writer_contention_drops`)
    /// if two writers contend concurrently.
    pub fn record_staged(&self, s: &TickSample, finalize: impl FnOnce() -> u32) -> Option<u64> {
        let initial_publish = s.phases_us.publish;
        self.record_staged_with_publish(s, || (finalize(), initial_publish))
    }

    /// Wait-free staged publication that stamps both authoritative `release_us` and
    /// `phases_us.publish` from `finalize()` immediately before sealing the slot seqlock.
    pub fn record_staged_with_publish(
        &self,
        s: &TickSample,
        finalize: impl FnOnce() -> (u32, u32),
    ) -> Option<u64> {
        if self
            .writer_busy
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            self.writer_contention_drops.fetch_add(1, Ordering::Relaxed);
            return None;
        }
        struct BusyGuard<'a>(&'a AtomicBool);
        impl Drop for BusyGuard<'_> {
            #[inline]
            fn drop(&mut self) {
                self.0.store(false, Ordering::Release);
            }
        }
        let _busy = BusyGuard(&self.writer_busy);

        let prev_seq = self.write_seq.load(Ordering::Relaxed);
        let sample_seq = prev_seq.wrapping_add(1);
        let slot = &self.slots[(prev_seq as usize) & (LOCK_FREE_RING_CAPACITY - 1)];

        // Per-slot seqlock write on the unpublished slot (readers do not read this slot until
        // `write_seq` advances in `meta_seq` below).
        let s0 = slot.seq.load(Ordering::Relaxed);
        slot.seq.store(s0.wrapping_add(1), Ordering::Relaxed);
        fence(Ordering::Release);
        slot.store_relaxed(sample_seq, s);
        let (final_release_us, final_publish_us) = finalize();
        slot.release_us.store(final_release_us, Ordering::Relaxed);
        slot.phase_publish.store(final_publish_us, Ordering::Relaxed);
        slot.seq.store(s0.wrapping_add(2), Ordering::Release);

        // Coherent metadata seqlock write: odd -> Release fence -> totals + write_seq -> even (Release).
        let m0 = self.meta_seq.load(Ordering::Relaxed);
        self.meta_seq.store(m0.wrapping_add(1), Ordering::Relaxed);
        fence(Ordering::Release);
        self.write_seq.store(sample_seq, Ordering::Relaxed);
        if s.late {
            let next = self.late_count.load(Ordering::Relaxed).saturating_add(1);
            self.late_count.store(next, Ordering::Relaxed);
        }
        let add_tx = |a: &AtomicU64, delta: u32| {
            if delta > 0 {
                let next = a.load(Ordering::Relaxed).saturating_add(u64::from(delta));
                a.store(next, Ordering::Relaxed);
            }
        };
        add_tx(&self.tx_frames, s.usage.tx_frames);
        add_tx(&self.tx_would_block, s.usage.tx_would_block);
        add_tx(&self.tx_enobufs, s.usage.tx_enobufs);
        add_tx(&self.tx_deferred, s.usage.tx_deferred);
        add_tx(&self.tx_dropped_stale, s.usage.tx_dropped_stale);
        add_tx(&self.tx_dropped_stopped, s.usage.tx_dropped_stopped);
        self.meta_seq.store(m0.wrapping_add(2), Ordering::Release);
        Some(sample_seq)
    }

    /// Wait-free single-writer tail amendment of `slot.release_us` for a specific `sample_seq`
    /// if a post-publication instrumentation tail extended the release duration.
    pub fn amend_slot_release_us(&self, sample_seq: u64, release_us: u32) {
        self.amend_slot_release_and_publish_us(sample_seq, release_us, 0);
    }

    /// Wait-free single-writer tail amendment of both `slot.release_us` and `slot.phase_publish`
    /// monotonically for a specific `sample_seq`.
    pub fn amend_slot_release_and_publish_us(&self, sample_seq: u64, release_us: u32, publish_us: u32) {
        if sample_seq == 0 {
            return;
        }
        if self
            .writer_busy
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            self.writer_contention_drops.fetch_add(1, Ordering::Relaxed);
            return;
        }
        struct BusyGuard<'a>(&'a AtomicBool);
        impl Drop for BusyGuard<'_> {
            #[inline]
            fn drop(&mut self) {
                self.0.store(false, Ordering::Release);
            }
        }
        let _busy = BusyGuard(&self.writer_busy);

        let slot_idx = ((sample_seq.wrapping_sub(1)) as usize) & (LOCK_FREE_RING_CAPACITY - 1);
        let slot = &self.slots[slot_idx];
        if slot.sample_seq.load(Ordering::Relaxed) == sample_seq {
            let cur_rel = slot.release_us.load(Ordering::Relaxed);
            let cur_pub = slot.phase_publish.load(Ordering::Relaxed);
            if cur_rel < release_us || cur_pub < publish_us {
                let s0 = slot.seq.load(Ordering::Relaxed);
                slot.seq.store(s0.wrapping_add(1), Ordering::Relaxed);
                fence(Ordering::Release);
                if cur_rel < release_us {
                    slot.release_us.store(release_us, Ordering::Relaxed);
                }
                if cur_pub < publish_us {
                    slot.phase_publish.store(publish_us, Ordering::Relaxed);
                }
                slot.seq.store(s0.wrapping_add(2), Ordering::Release);
            }
        }
    }

    /// Wait-free single-writer overrun-only amendment of `release_us`, `phases_us.publish`,
    /// `post_phases_us`, and `post_cpu_us` for a specific `sample_seq` (legacy owner).
    pub fn amend_slot_post_overrun(
        &self,
        sample_seq: u64,
        release_us: u32,
        publish_us: u32,
        post_phases: PostOverrunPhasesUs,
        post_cpu_us: u32,
    ) {
        if sample_seq == 0 {
            return;
        }
        if self
            .writer_busy
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            self.writer_contention_drops.fetch_add(1, Ordering::Relaxed);
            return;
        }
        struct BusyGuard<'a>(&'a AtomicBool);
        impl Drop for BusyGuard<'_> {
            #[inline]
            fn drop(&mut self) {
                self.0.store(false, Ordering::Release);
            }
        }
        let _busy = BusyGuard(&self.writer_busy);

        let slot_idx = ((sample_seq.wrapping_sub(1)) as usize) & (LOCK_FREE_RING_CAPACITY - 1);
        let slot = &self.slots[slot_idx];
        if slot.sample_seq.load(Ordering::Relaxed) == sample_seq {
            let s0 = slot.seq.load(Ordering::Relaxed);
            slot.seq.store(s0.wrapping_add(1), Ordering::Relaxed);
            fence(Ordering::Release);
            if slot.release_us.load(Ordering::Relaxed) < release_us {
                slot.release_us.store(release_us, Ordering::Relaxed);
            }
            if slot.phase_publish.load(Ordering::Relaxed) < publish_us {
                slot.phase_publish.store(publish_us, Ordering::Relaxed);
            }
            slot.has_post_overrun.store(true, Ordering::Relaxed);
            slot.post_step_to_ring_us.store(post_phases.step_to_ring_us, Ordering::Relaxed);
            slot.post_ring_publish_us.store(post_phases.ring_publish_us, Ordering::Relaxed);
            slot.post_ring_to_seal_us.store(post_phases.ring_to_seal_us, Ordering::Relaxed);
            slot.post_seal_us.store(post_phases.seal_us, Ordering::Relaxed);
            slot.post_cpu_us.store(post_cpu_us, Ordering::Relaxed);
            slot.seq.store(s0.wrapping_add(2), Ordering::Release);
        }
    }

    /// Wait-free single-writer tail amendment by executor `release_seq` (called only by off-thread
    /// `drain_diag` under the `diag_rx` consumer mutex so `LockFreeTickStats` stays single-writer).
    /// Scans up to [`TICK_WINDOW`] recent slots (`1000 > DIAG_RING_CAPACITY`) and returns `true`
    /// if the target sample was found in the ring, or `false` if evicted or contended.
    pub fn amend_release_us_by_release_seq(&self, release_seq: u64, release_us: u32) -> bool {
        self.amend_by_release_seq(release_seq, release_us, None)
    }

    /// Wait-free single-writer tail amendment by executor `release_seq` updating `release_us`
    /// monotonically and optionally attaching overrun-only `post_phases_us` and `post_cpu_us`.
    pub fn amend_by_release_seq(
        &self,
        release_seq: u64,
        release_us: u32,
        post_overrun: Option<(PostOverrunPhasesUs, u32)>,
    ) -> bool {
        if release_seq == 0 {
            return false;
        }
        if self
            .writer_busy
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            self.writer_contention_drops.fetch_add(1, Ordering::Relaxed);
            return false;
        }
        struct BusyGuard<'a>(&'a AtomicBool);
        impl Drop for BusyGuard<'_> {
            #[inline]
            fn drop(&mut self) {
                self.0.store(false, Ordering::Release);
            }
        }
        let _busy = BusyGuard(&self.writer_busy);

        let end_seq = self.write_seq.load(Ordering::Relaxed);
        let scan = (end_seq as usize).min(TICK_WINDOW);
        for back in 0..scan {
            let seq = end_seq - back as u64;
            let slot = &self.slots[((seq - 1) as usize) & (LOCK_FREE_RING_CAPACITY - 1)];
            if slot.sample_seq.load(Ordering::Relaxed) == seq
                && slot.release_seq.load(Ordering::Relaxed) == release_seq
            {
                let raise_rel = slot.release_us.load(Ordering::Relaxed) < release_us;
                if raise_rel || post_overrun.is_some() {
                    let s0 = slot.seq.load(Ordering::Relaxed);
                    slot.seq.store(s0.wrapping_add(1), Ordering::Relaxed);
                    fence(Ordering::Release);
                    if raise_rel {
                        slot.release_us.store(release_us, Ordering::Relaxed);
                    }
                    if let Some((pp, cpu_us)) = post_overrun {
                        slot.has_post_overrun.store(true, Ordering::Relaxed);
                        slot.post_step_to_ring_us.store(pp.step_to_ring_us, Ordering::Relaxed);
                        slot.post_ring_publish_us.store(pp.ring_publish_us, Ordering::Relaxed);
                        slot.post_ring_to_seal_us.store(pp.ring_to_seal_us, Ordering::Relaxed);
                        slot.post_seal_us.store(pp.seal_us, Ordering::Relaxed);
                        slot.post_cpu_us.store(cpu_us, Ordering::Relaxed);
                    }
                    slot.seq.store(s0.wrapping_add(2), Ordering::Release);
                }
                return true;
            }
        }
        false
    }

    fn read_meta_seqlock(&self) -> Option<(u64, u64, TxTotals)> {
        for _ in 0..MAX_SEQLOCK_RETRIES {
            let m1 = self.meta_seq.load(Ordering::Acquire);
            if m1 & 1 != 0 {
                self.reader_retries.fetch_add(1, Ordering::Relaxed);
                std::hint::spin_loop();
                continue;
            }
            let write_seq = self.write_seq.load(Ordering::Relaxed);
            let late_count = self.late_count.load(Ordering::Relaxed);
            let tx_totals = TxTotals {
                frames: self.tx_frames.load(Ordering::Relaxed),
                would_block: self.tx_would_block.load(Ordering::Relaxed),
                enobufs: self.tx_enobufs.load(Ordering::Relaxed),
                deferred: self.tx_deferred.load(Ordering::Relaxed),
                dropped_stale: self.tx_dropped_stale.load(Ordering::Relaxed),
                dropped_stopped: self.tx_dropped_stopped.load(Ordering::Relaxed),
            };
            fence(Ordering::Acquire);
            let m2 = self.meta_seq.load(Ordering::Relaxed);
            if m1 == m2 {
                return Some((write_seq, late_count, tx_totals));
            }
            self.reader_retries.fetch_add(1, Ordering::Relaxed);
            std::hint::spin_loop();
        }
        None
    }

    fn read_slot_seqlock(&self, slot: &AtomicTickSlot) -> Option<(u64, TickSample)> {
        for _ in 0..MAX_SEQLOCK_RETRIES {
            let v1 = slot.seq.load(Ordering::Acquire);
            if v1 & 1 != 0 {
                self.reader_retries.fetch_add(1, Ordering::Relaxed);
                std::hint::spin_loop();
                continue;
            }
            let raw = slot.load_raw_relaxed();
            fence(Ordering::Acquire);
            let v2 = slot.seq.load(Ordering::Relaxed);
            if v1 == v2 {
                // SAFETY: `v1 == v2` (even) with Acquire on `v1` and Acquire fence before `v2`.
                return Some(unsafe { raw.into_sample_validated() });
            }
            self.reader_retries.fetch_add(1, Ordering::Relaxed);
            std::hint::spin_loop();
        }
        None
    }

    /// Lock-free coherent snapshot of the last `min(ticks, TICK_WINDOW)` samples, oldest first.
    pub fn window(&self) -> TickWindow {
        self.window_with_hook(|_| {})
    }

    /// [`Self::window`] with a per-slot callback for deterministic concurrency / lap tests.
    #[doc(hidden)]
    pub fn window_with_hook(&self, mut on_slot_read: impl FnMut(usize)) -> TickWindow {
        for _ in 0..MAX_WINDOW_ATTEMPTS {
            let Some((end_seq, late_count, tx_totals)) = self.read_meta_seqlock() else {
                self.reader_window_retries.fetch_add(1, Ordering::Relaxed);
                continue;
            };
            let count = (end_seq as usize).min(TICK_WINDOW);
            let start_seq = end_seq.saturating_sub(count as u64);
            let mut rows = Vec::with_capacity(count);
            let mut coherent = true;
            for i in 0..count {
                on_slot_read(i);
                let zero_based = start_seq + i as u64;
                let expected_sample_seq = zero_based.wrapping_add(1);
                let slot = &self.slots[(zero_based as usize) & (LOCK_FREE_RING_CAPACITY - 1)];
                match self.read_slot_seqlock(slot) {
                    Some((got_seq, sample)) if got_seq == expected_sample_seq => rows.push(sample),
                    _ => {
                        coherent = false;
                        break;
                    }
                }
            }
            if coherent {
                return TickWindow {
                    rows,
                    late_count,
                    ticks: end_seq,
                    tx_totals,
                    snapshot_lapped: false,
                };
            }
            self.reader_window_retries.fetch_add(1, Ordering::Relaxed);
        }
        self.reader_lapped_snapshots.fetch_add(1, Ordering::Relaxed);
        TickWindow {
            rows: Vec::new(),
            late_count: self.late_count.load(Ordering::Relaxed),
            ticks: self.write_seq.load(Ordering::Acquire),
            tx_totals: TxTotals::default(),
            snapshot_lapped: true,
        }
    }

    pub fn snapshot(&self) -> TickSnapshot {
        let mut snap = self.window().snapshot();
        snap.stats_reader_retries = self.reader_retries();
        snap.stats_writer_drops = self.writer_contention_drops();
        snap
    }

    pub fn reader_retries(&self) -> u64 {
        self.reader_retries.load(Ordering::Relaxed)
    }

    pub fn reader_window_retries(&self) -> u64 {
        self.reader_window_retries.load(Ordering::Relaxed)
    }

    pub fn reader_lapped_snapshots(&self) -> u64 {
        self.reader_lapped_snapshots.load(Ordering::Relaxed)
    }

    pub fn writer_contention_drops(&self) -> u64 {
        self.writer_contention_drops.load(Ordering::Relaxed)
    }

    #[doc(hidden)]
    pub fn set_odd_meta_seq_for_test(&self, odd: bool) {
        let cur = self.meta_seq.load(Ordering::Relaxed);
        let target = if odd { cur | 1 } else { cur & !1 };
        self.meta_seq.store(target, Ordering::Release);
    }

    #[doc(hidden)]
    pub fn set_odd_slot_seq_for_test(&self, slot_idx: usize, odd: bool) {
        let slot = &self.slots[slot_idx & (LOCK_FREE_RING_CAPACITY - 1)];
        let cur = slot.seq.load(Ordering::Relaxed);
        let target = if odd { cur | 1 } else { cur & !1 };
        slot.seq.store(target, Ordering::Release);
    }
}

impl TickWindow {
    /// The samples, oldest first.
    pub fn samples(&self) -> &[TickSample] { &self.rows }

    /// Returns `true` when this window exhausted bounded seqlock retries.
    pub fn is_lapped(&self) -> bool { self.snapshot_lapped }

    fn col(&self, f: impl Fn(&TickSample) -> u32) -> Percentiles {
        let mut v: Vec<u32> = self.rows.iter().map(f).collect();
        percentiles(&mut v)
    }

    /// Thread-CPU-time percentiles of lock-held tick execution (`CLOCK_THREAD_CPUTIME_ID` on Linux).
    pub fn lock_cpu_us(&self) -> Percentiles {
        self.col(|r| r.usage.lock_cpu_us)
    }

    fn sum(&self, f: impl Fn(&TickSample) -> u32) -> u64 {
        self.rows.iter().fold(0u64, |acc, r| acc.saturating_add(u64::from(f(r))))
    }

    /// The latest of equal maxima of `key`.
    fn max_by(&self, key: impl Fn(&TickSample) -> u32) -> Option<TickBreakdown> {
        let n = self.rows.len();
        self.rows.iter().enumerate().rev().max_by_key(|(_, r)| key(r))
            .map(|(i, r)| TickBreakdown {
                sample: *r,
                handoff_us: r.waited_for.map(|h| h.released_before_lock_us),
                ticks_ago: n - 1 - i,
            })
    }

    pub fn snapshot(&self) -> TickSnapshot {
        TickSnapshot {
            instance_id: String::new(),
            boot_id: None,
            process_start_unix_ns: None,
            window: self.rows.len(),
            tick_us: self.col(|r| r.tick_us),
            release_us: self.col(|r| r.release_us),
            releases_started: 0,
            releases_total: 0,
            in_flight_release_us: 0,
            release_us_max_ever: 0,
            releases_over_10ms: 0,
            releases_over_15ms: 0,
            releases_over_20ms: 0,
            wake_late_max_ever: 0,
            lock_us: self.col(|r| r.lock_us),
            late_count: self.late_count,
            since_boot_ticks: self.ticks,
            wait_us: self.col(|r| r.wait_us),
            post_us: self.col(|r| r.post_us),
            period_us: self.col(|r| r.period_us),
            wake_lateness_us: self.col(|r| r.wake_lateness_us),
            wake_lateness_over_1ms: 0,
            wake_lateness_over_5ms: 0,
            wake_lateness_over_10ms: 0,
            skipped_releases: 0,
            diag_dropped: 0,
            owner_mode: "legacy",
            effective_owner: "legacy",
            owner_reason: "default",
            loop_active: false,
            reader_hold_us: self.col(|r| r.reader_hold_us),
            writer_hold_us: self.col(|r| r.writer_hold_us),
            stats_wait_us: self.col(|r| r.stats_wait_us),
            reader_acquire_us: self.col(|r| r.reader_acquire_us),
            writer_acquire_us: self.col(|r| r.writer_acquire_us),
            lock_phases: LockPhasePercentiles {
                pre_us: self.col(|r| r.phases.pre_us),
                rx_us: self.col(|r| r.phases.rx_us),
                control_us: self.col(|r| r.phases.control_us),
                tx_us: self.col(|r| r.phases.tx_us),
                finish_us: self.col(|r| r.phases.finish_us),
                obs_us: self.col(|r| r.phases.obs_us),
                publish_us: self.col(|r| r.phases.publish_us),
            },
            max_tick: self.max_by(|r| r.tick_us),
            max_release: self.max_by(|r| r.release_us),
            max_lock: self.max_by(|r| r.lock_us),
            max_wait: self.max_by(|r| r.wait_us),
            max_tx: self.max_by(|r| r.usage.tx_send_us),
            tx: TxDiagnostics {
                frames: self.col(|r| r.usage.tx_frames),
                pending: self.col(|r| r.usage.tx_pending),
                send_us: self.col(|r| r.usage.tx_send_us),
                send_cpu_us: self.col(|r| r.usage.tx_send_cpu_us),
                frame_max_us: self.col(|r| r.usage.tx_frame_max_us),
                frame_cpu_max_us: self.col(|r| r.usage.tx_frame_cpu_max_us),
                frames_total: self.sum(|r| r.usage.tx_frames),
                would_block_total: self.sum(|r| r.usage.tx_would_block),
                enobufs_total: self.sum(|r| r.usage.tx_enobufs),
                deferred_total: self.sum(|r| r.usage.tx_deferred),
                dropped_stale_total: self.sum(|r| r.usage.tx_dropped_stale),
                dropped_stopped_total: self.sum(|r| r.usage.tx_dropped_stopped),
                since_boot_frames: self.tx_totals.frames,
                since_boot_would_block: self.tx_totals.would_block,
                since_boot_enobufs: self.tx_totals.enobufs,
                since_boot_deferred: self.tx_totals.deferred,
                since_boot_dropped_stale: self.tx_totals.dropped_stale,
                since_boot_dropped_stopped: self.tx_totals.dropped_stopped,
                stopped_hold_lock_contended: stopped_hold_lock_contended_count(),
                stopped_hold_contention_escalations: stopped_hold_contention_escalations_count(),
                disable_lock_contended: disable_lock_contended_count(),
                disable_contention_escalations: disable_contention_escalations_count(),
            },
            control_thread: None,
            observation_cache: ObservationCacheStats::default(),
            terminal_capture: TerminalCaptureStats::default(),
            safety_log_dropped: 0,
            log_writer_dropped: 0,
            stats_snapshot_lapped: self.snapshot_lapped,
            stats_reader_retries: 0,
            stats_writer_drops: 0,
            since_marker: None,
            control_thread_lock_attempts: None,
            control_thread_alloc_ops: None,
            alloc_timing_enabled: None,
            executor_wakes: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ring_keeps_the_last_window_and_counts_late_ticks() {
        let mut s = TickStats::default();
        assert_eq!(s.snapshot().window, 0);
        assert_eq!(s.snapshot().max_tick, None);
        for i in 1..=(TICK_WINDOW as u32 + 100) {
            s.record(&TickSample { tick_us: i, lock_us: 1, late: i % 10 == 0, ..Default::default() });
        }
        let snap = s.snapshot();
        assert_eq!(snap.window, TICK_WINDOW);
        assert_eq!(snap.since_boot_ticks, TICK_WINDOW as u64 + 100);
        assert_eq!(snap.late_count, 110);
        assert_eq!(snap.tick_us.max, 1100);
        assert_eq!(snap.tick_us.p50, 600);
        assert_eq!(snap.tick_us.p99, 1090);
        assert_eq!(snap.lock_us, Percentiles { p50: 1, p99: 1, max: 1 });
        assert_eq!(snap.max_tick.unwrap().ticks_ago, 0);
    }

    #[test]
    fn max_tick_and_max_lock_carry_their_phases_and_holder() {
        let mut s = TickStats::default();
        let holder = Holder {
            kind: "read",
            location: std::panic::Location::caller(),
            command: None,
            direction: None,
            action_id: None,
            io_timing: None,
            hold_us: 31_000,
            acquire_us: 3,
            acquired_before_wait_us: 1_000,
            released_before_lock_us: 40,
        };
        for i in 0..50u32 {
            let (slow, heavy) = (i == 20, i == 30);
            s.record(&TickSample {
                tick_us: if slow { 34_000 } else if heavy { 15_000 } else { 1_000 },
                wait_us: if slow { 32_000 } else { 0 },
                lock_us: if heavy { 14_000 } else { 1_000 },
                post_us: 100,
                period_us: 20_000,
                reader_hold_us: if slow { 31_000 } else { 200 },
                phases: LockPhases { rx_us: if heavy { 12_000 } else { 300 }, ..Default::default() },
                usage: ThreadUsage { lock_majflt: u32::from(heavy) * 3, ..Default::default() },
                waited_for: slow.then_some(holder),
                ..Default::default()
            });
        }
        let snap = s.snapshot();
        let m = snap.max_tick.unwrap();
        assert_eq!((m.sample.tick_us, m.sample.wait_us, m.ticks_ago), (34_000, 32_000, 29));
        let l = snap.max_lock.unwrap();
        assert_eq!((l.sample.lock_us, l.sample.phases.rx_us, l.ticks_ago), (14_000, 12_000, 19));
        assert_eq!(snap.lock_phases.rx_us.max, 12_000);
        let json = serde_json::to_value(&snap).unwrap();
        for key in ["tick_us", "lock_us", "wait_us", "post_us", "period_us", "reader_hold_us", "writer_hold_us", "stats_wait_us", "reader_acquire_us", "writer_acquire_us"] {
            assert!(json[key]["max"].is_u64(), "{key}");
        }
        let mt = &json["max_tick"];
        assert_eq!(mt["wait_us"], 32_000);
        assert_eq!(mt["waited_for"]["kind"], "read");
        assert_eq!(mt["waited_for"]["hold_us"], 31_000);
        assert!(mt["waited_for"].get("command").is_none(), "{mt}");
        assert!(mt["waited_for"].get("action_id").is_none(), "{mt}");
        assert!(mt["waited_for"]["location"].as_str().unwrap().starts_with("src/diagnostics/tick.rs:"), "{mt}");
        assert!(mt["longest_hold"].is_null());
        assert!(mt.get("late").is_none());
        assert_eq!(json["max_lock"]["rx_us"], 12_000);
        assert_eq!(json["max_lock"]["lock_majflt"], 3);
        assert_eq!(json["lock_phases"]["rx_us"]["max"], 12_000);
        assert!(json["control_thread"].is_null());
        assert!(json["tx"]["send_us"]["max"].is_u64());

        // Holder with command + direction + action_id + io_timing attribution serializes all fields
        // and round-trips through LockFreeTickStats.
        let attributed = Holder {
            kind: "write",
            location: std::panic::Location::caller(),
            command: Some("/api/v1/torque"),
            direction: Some("on"),
            action_id: Some(19),
            io_timing: Some(HoldIoTiming { tx_us: 120, rx_us: 45, sleep_us: 500 }),
            hold_us: 420,
            acquire_us: 12,
            acquired_before_wait_us: 50,
            released_before_lock_us: 8,
        };
        let lf = LockFreeTickStats::new();
        assert!(lf.record(&TickSample {
            tick_us: 900,
            wait_us: 400,
            lock_us: 300,
            waited_for: Some(attributed),
            longest_hold: Some(attributed),
            ..Default::default()
        }));
        let lf_snap = lf.snapshot();
        let lf_json = serde_json::to_value(&lf_snap).unwrap();
        assert_eq!(lf_json["max_tick"]["waited_for"]["command"], "/api/v1/torque");
        assert_eq!(lf_json["max_tick"]["waited_for"]["direction"], "on");
        assert_eq!(lf_json["max_tick"]["waited_for"]["action_id"], 19);
        assert_eq!(lf_json["max_tick"]["waited_for"]["tx_us"], 120);
        assert_eq!(lf_json["max_tick"]["waited_for"]["rx_us"], 45);
        assert_eq!(lf_json["max_tick"]["waited_for"]["sleep_us"], 500);
        assert_eq!(lf_json["max_tick"]["longest_hold"]["command"], "/api/v1/torque");
        assert_eq!(lf_json["max_tick"]["longest_hold"]["direction"], "on");
        assert_eq!(lf_json["max_tick"]["longest_hold"]["action_id"], 19);
        assert_eq!(lf_json["max_tick"]["longest_hold"]["tx_us"], 120);
        assert_eq!(lf_json["max_tick"]["longest_hold"]["rx_us"], 45);
        assert_eq!(lf_json["max_tick"]["longest_hold"]["sleep_us"], 500);
    }

    #[test]
    fn tx_metrics_capture_per_frame_max_totals_and_json_fields() {
        reset_tx_metrics();
        // pre-tick TX on the same thread must be cleared by the locked UsageMark
        record_tx_attempt(TxAttemptKind::Sent, 9_000_000, 1_000_000);
        let start = UsageMark::now();
        let locked = UsageMark::now();
        // unequal single-frame durations (150 us and 900 us) + WouldBlock + ENOBUFS
        record_tx_attempt(TxAttemptKind::Sent, 150_000, 30_000);
        record_tx_attempt(TxAttemptKind::Sent, 900_000, 80_000);
        record_tx_attempt(TxAttemptKind::WouldBlock, 40_000, 10_000);
        record_tx_attempt(TxAttemptKind::Enobufs, 50_000, 15_000);
        record_tx_deferred(2, 3);
        record_tx_dropped_stale(1);
        record_tx_dropped_stopped(4);
        let released = UsageMark::now();
        // accumulator reset by UsageMark::now()
        assert_eq!(take_tx_metrics(), TxTickMetrics::ZERO);

        let u = thread_usage(&start, &locked, &released);
        assert_eq!(u.tx_frames, 2);
        assert_eq!(u.tx_would_block, 1);
        assert_eq!(u.tx_enobufs, 1);
        assert_eq!(u.tx_deferred, 2);
        assert_eq!(u.tx_pending, 3);
        assert_eq!(u.tx_dropped_stale, 1);
        assert_eq!(u.tx_dropped_stopped, 4);
        assert_eq!(u.tx_send_us, 1140);
        assert_eq!(u.tx_send_cpu_us, 135);
        assert_eq!(u.tx_frame_max_us, 900);
        assert_eq!(u.tx_frame_cpu_max_us, 80);

        let mut stats = TickStats::default();
        // push one tick with TX stats, then enough empty ticks to roll past TICK_WINDOW
        stats.record(&TickSample { tick_us: 2_000, lock_us: 1_500, usage: u, ..Default::default() });
        let snap1 = stats.snapshot();
        assert_eq!(snap1.tx.frames_total, 2);
        assert_eq!(snap1.tx.would_block_total, 1);
        assert_eq!(snap1.tx.enobufs_total, 1);
        assert_eq!(snap1.tx.deferred_total, 2);
        assert_eq!(snap1.tx.pending.max, 3);
        assert_eq!(snap1.tx.dropped_stale_total, 1);
        assert_eq!(snap1.tx.dropped_stopped_total, 4);
        assert_eq!(snap1.tx.send_us.max, 1140);
        assert_eq!(snap1.tx.send_cpu_us.max, 135);
        assert_eq!(snap1.tx.frame_max_us.max, 900);
        assert_eq!(snap1.tx.frame_cpu_max_us.max, 80);
        assert_eq!(snap1.max_tx.unwrap().sample.usage.tx_frame_max_us, 900);

        let json = serde_json::to_value(&snap1).unwrap();
        assert_eq!(json["tx"]["frames_total"], 2);
        assert_eq!(json["tx"]["would_block_total"], 1);
        assert_eq!(json["tx"]["enobufs_total"], 1);
        assert_eq!(json["tx"]["frame_max_us"]["max"], 900);
        assert_eq!(json["tx"]["frame_cpu_max_us"]["max"], 80);
        assert_eq!(json["tx"]["send_us"]["max"], 1140);
        assert_eq!(json["tx"]["send_cpu_us"]["max"], 135);
        assert_eq!(json["max_tx"]["tx_frame_max_us"], 900);
        assert_eq!(json["max_tx"]["tx_enobufs"], 1);

        // roll the ring past TICK_WINDOW: window sums clear while since_boot sums remain
        for _ in 0..TICK_WINDOW {
            stats.record(&TickSample::default());
        }
        let snap2 = stats.snapshot();
        assert_eq!(snap2.tx.frames_total, 0);
        assert_eq!(snap2.tx.enobufs_total, 0);
        assert_eq!(snap2.tx.since_boot_frames, 2);
        assert_eq!(snap2.tx.since_boot_would_block, 1);
        assert_eq!(snap2.tx.since_boot_enobufs, 1);
        assert_eq!(snap2.tx.since_boot_deferred, 2);
        assert_eq!(snap2.tx.since_boot_dropped_stale, 1);
        assert_eq!(snap2.tx.since_boot_dropped_stopped, 4);
    }

    #[test]
    fn lock_free_tick_stats_matches_tick_stats_across_rollover_holders_and_tx_totals() {
        let mut plain = TickStats::default();
        let lf = LockFreeTickStats::new();
        let read_holder = Holder {
            kind: "read",
            location: std::panic::Location::caller(),
            command: None,
            direction: None,
            action_id: None,
            io_timing: None,
            hold_us: 12_345,
            acquire_us: 67,
            acquired_before_wait_us: 890,
            released_before_lock_us: -12,
        };
        let write_holder = Holder {
            kind: "write",
            location: std::panic::Location::caller(),
            command: Some("/api/v1/motion/jog"),
            direction: None,
            action_id: Some(42),
            io_timing: None,
            hold_us: 23_456,
            acquire_us: 89,
            acquired_before_wait_us: -34,
            released_before_lock_us: 56,
        };
        for i in 1..=(TICK_WINDOW as u32 + 150) {
            let sample = TickSample {
                tick_us: i * 3,
                release_us: i * 3 + 50,
                wait_us: if i % 25 == 0 { 400 } else { 0 },
                lock_us: i * 2,
                post_us: 15,
                period_us: 20_000 + (i % 7),
                wake_lateness_us: i % 500,
                reader_hold_us: if i % 10 == 0 { 300 } else { 0 },
                writer_hold_us: if i % 20 == 0 { 500 } else { 0 },
                reader_acquire_us: 5,
                writer_acquire_us: 9,
                stats_wait_us: 0,
                phases: LockPhases {
                    pre_us: 10,
                    rx_us: 20 + (i % 5),
                    control_us: 30,
                    tx_us: 40,
                    finish_us: 5,
                    obs_us: 12,
                    publish_us: 3,
                },
                phases_us: ReleasePhasesUs {
                    alloc_read: 1 + (i % 3),
                    ack_flush: 2 + (i % 5),
                    terminal: 3 + (i % 7),
                    guard_drop: 4 + (i % 11),
                    post: 15,
                    publish: 6 + (i % 13),
                },
                post_phases_us: None,
                post_cpu_us: None,
                usage: ThreadUsage {
                    mid_nvcsw: u32::from(i % 17 == 0),
                    mid_nivcsw: u32::from(i % 19 == 0),
                    tx_frames: if i % 3 == 0 { 2 } else { 1 },
                    tx_would_block: u32::from(i % 11 == 0),
                    tx_enobufs: u32::from(i % 13 == 0),
                    tx_deferred: u32::from(i % 17 == 0),
                    tx_pending: i % 4,
                    tx_dropped_stale: u32::from(i % 19 == 0),
                    tx_dropped_stopped: u32::from(i % 23 == 0),
                    tx_send_us: 100 + (i % 90),
                    tx_send_cpu_us: 40,
                    tx_frame_max_us: 60 + (i % 40),
                    tx_frame_cpu_max_us: 25,
                    ..Default::default()
                },
                waited_for: (i == TICK_WINDOW as u32 + 100).then_some(read_holder),
                longest_hold: (i == TICK_WINDOW as u32 + 100).then_some(write_holder),
                torque_step: None,
                late: i % 9 == 0,
                release_seq: u64::from(i),
                safety_epoch: u64::from(i % 5),
                maint_req_id: u64::from(i % 7),
                release_flags: i & 0x7f,
            };
            plain.record(&sample);
            assert!(lf.record(&sample));
        }

        let w_plain = plain.window();
        let w_lf = lf.window();
        assert_eq!(w_plain.samples(), w_lf.samples());
        let mut s_plain = w_plain.snapshot();
        let s_lf = lf.snapshot();
        s_plain.stats_reader_retries = s_lf.stats_reader_retries;
        assert_eq!(s_plain, s_lf);
        assert!(!s_lf.stats_snapshot_lapped);
        assert_eq!(s_lf.stats_writer_drops, 0);
    }

    #[test]
    fn lock_free_tick_stats_handles_paused_reader_reuse_and_bounded_odd_seqlock() {
        let lf = LockFreeTickStats::new();
        for i in 1..=(TICK_WINDOW as u32 + 10) {
            assert!(lf.record(&TickSample { tick_us: i, ..Default::default() }));
        }

        // 1. Reader paused on slot 0 while writer overwrites the entire 2048-slot ring once:
        //    first attempt detects slot reuse (`sample_seq != expected`), retries, and succeeds coherently.
        let mut lapped_once = false;
        let win = lf.window_with_hook(|slot_i| {
            if slot_i == 0 && !lapped_once {
                lapped_once = true;
                for j in 0..(LOCK_FREE_RING_CAPACITY as u32 + 16) {
                    assert!(lf.record(&TickSample { tick_us: 50_000 + j, ..Default::default() }));
                }
            }
        });
        assert!(!win.snapshot_lapped);
        assert_eq!(win.samples().len(), TICK_WINDOW);
        assert!(lf.reader_window_retries() >= 1);
        assert_eq!(
            win.samples().last().unwrap().tick_us,
            50_000 + LOCK_FREE_RING_CAPACITY as u32 + 15
        );

        // 2. Reader repeatedly lapped across every attempt -> explicit `snapshot_lapped = true`.
        let lapped = lf.window_with_hook(|slot_i| {
            if slot_i == 0 {
                for j in 0..(LOCK_FREE_RING_CAPACITY as u32 + 2) {
                    assert!(lf.record(&TickSample { tick_us: 90_000 + j, ..Default::default() }));
                }
            }
        });
        assert!(lapped.snapshot_lapped);
        assert!(lapped.snapshot().stats_snapshot_lapped);
        assert_eq!(lf.reader_lapped_snapshots(), 1);

        // 3. Odd meta_seq or odd slot seq held across full retry budget terminates deterministically.
        let retries_before = lf.reader_retries();
        lf.set_odd_meta_seq_for_test(true);
        let odd_meta = lf.window();
        lf.set_odd_meta_seq_for_test(false);
        assert!(odd_meta.snapshot_lapped);
        assert!(lf.reader_retries() >= retries_before + MAX_SEQLOCK_RETRIES as u64);

        let retries_before_slot = lf.reader_retries();
        let end_seq = lf.write_seq.load(Ordering::Acquire);
        let first_slot = end_seq.saturating_sub(TICK_WINDOW as u64) as usize;
        lf.set_odd_slot_seq_for_test(first_slot, true);
        let odd_slot = lf.window();
        lf.set_odd_slot_seq_for_test(first_slot, false);
        assert!(odd_slot.snapshot_lapped);
        assert!(lf.reader_retries() >= retries_before_slot + MAX_SEQLOCK_RETRIES as u64);
    }

    #[test]
    fn lock_free_tick_stats_record_staged_and_tail_amendments_are_exact() {
        let lf = LockFreeTickStats::new();
        let seq1 = lf
            .record_staged(
                &TickSample {
                    tick_us: 1_200,
                    release_us: 0,
                    release_seq: 101,
                    ..Default::default()
                },
                || 4_500,
            )
            .expect("staged record");
        assert_eq!(seq1, 1);
        assert_eq!(lf.window().samples()[0].release_us, 4_500);

        // Amend downward is ignored; amend upward updates cleanly.
        lf.amend_slot_release_us(seq1, 3_000);
        assert_eq!(lf.window().samples()[0].release_us, 4_500);
        lf.amend_slot_release_us(seq1, 25_200);
        assert_eq!(lf.window().samples()[0].release_us, 25_200);

        // Executor release_seq-based amendment updates the recorded slot monotonically even > 16 slots deep.
        let _ = lf.record(&TickSample {
            tick_us: 900,
            release_us: 1_100,
            release_seq: 202,
            ..Default::default()
        });
        for k in 0..32u64 {
            let _ = lf.record(&TickSample {
                tick_us: 500,
                release_us: 600,
                release_seq: 203 + k,
                ..Default::default()
            });
        }
        assert!(lf.amend_release_us_by_release_seq(202, 26_700));
        let samples = lf.window().samples().to_vec();
        assert_eq!(samples[1].release_seq, 202);
        assert_eq!(samples[1].release_us, 26_700);
        // Unknown / evicted sequence returns `false` so callers record explicit coverage loss.
        assert!(!lf.amend_release_us_by_release_seq(999_999, 10_000));
        assert_eq!(lf.writer_contention_drops(), 0);
    }

    #[test]
    fn lock_free_tick_stats_multithreaded_tear_free_and_writer_nonblocking() {
        use std::sync::{mpsc, Arc};
        use std::thread;

        let lf = Arc::new(LockFreeTickStats::new());
        let loc_read: &'static std::panic::Location<'static> = std::panic::Location::caller();
        let loc_write: &'static std::panic::Location<'static> = std::panic::Location::caller();

        let make_sample = move |k: u64| -> TickSample {
            let ku = k as u32;
            let waited_for = if k.is_multiple_of(2) {
                Some(Holder {
                    kind: "read",
                    location: loc_read,
                    command: None,
                    direction: None,
                    action_id: None,
                    io_timing: None,
                    hold_us: ku * 7 + 1,
                    acquire_us: ku * 3 + 2,
                    acquired_before_wait_us: k as i64 * 11,
                    released_before_lock_us: -(k as i64),
                })
            } else {
                None
            };
            let longest_hold = if k.is_multiple_of(3) {
                Some(Holder {
                    kind: "write",
                    location: loc_write,
                    command: Some("/api/v1/motion/home"),
                    direction: None,
                    action_id: Some(k + 100),
                    io_timing: None,
                    hold_us: ku * 13 + 5,
                    acquire_us: ku * 5 + 4,
                    acquired_before_wait_us: -(k as i64 * 2),
                    released_before_lock_us: k as i64 * 17,
                })
            } else {
                None
            };
            TickSample {
                tick_us: ku * 10 + 1,
                release_us: ku * 10 + 7,
                wait_us: ku % 97,
                lock_us: ku * 4 + 3,
                post_us: ku % 53,
                period_us: 20_000 + (ku % 113),
                wake_lateness_us: ku % 887,
                reader_hold_us: ku % 311,
                writer_hold_us: ku % 211,
                reader_acquire_us: ku % 41,
                writer_acquire_us: ku % 43,
                stats_wait_us: 0,
                phases: LockPhases {
                    pre_us: ku % 19,
                    rx_us: ku % 23,
                    control_us: ku % 29,
                    tx_us: ku % 31,
                    finish_us: ku % 37,
                    obs_us: ku % 41,
                    publish_us: ku % 43,
                },
                phases_us: ReleasePhasesUs {
                    alloc_read: ku % 7,
                    ack_flush: ku % 11,
                    terminal: ku % 13,
                    guard_drop: ku % 17,
                    post: ku % 53,
                    publish: ku % 29,
                },
                post_phases_us: None,
                post_cpu_us: None,
                usage: ThreadUsage {
                    lock_cpu_us: ku % 500,
                    mid_nvcsw: ku % 3,
                    mid_nivcsw: ku % 5,
                    tx_frames: 2,
                    tx_would_block: u32::from(k.is_multiple_of(5)),
                    tx_enobufs: u32::from(k.is_multiple_of(11)),
                    tx_deferred: u32::from(k.is_multiple_of(13)),
                    tx_pending: ku % 7,
                    tx_dropped_stale: u32::from(k.is_multiple_of(17)),
                    tx_dropped_stopped: u32::from(k.is_multiple_of(19)),
                    tx_send_us: ku % 600,
                    tx_send_cpu_us: ku % 200,
                    tx_frame_max_us: ku % 300,
                    tx_frame_cpu_max_us: ku % 100,
                    ..Default::default()
                },
                waited_for,
                longest_hold,
                torque_step: None,
                late: k.is_multiple_of(7),
                release_seq: k,
                safety_epoch: k * 3 + 1,
                maint_req_id: k * 5 + 2,
                release_flags: ku & 0x7f,
            }
        };

        // Pre-populate 16 samples so a frozen reader immediately enters the slot loop.
        for k in 1..=16u64 {
            assert!(lf.record(&make_sample(k)));
        }

        // Freeze one reader mid-window while the writer publishes 6,000 samples (wrapping 2048-slot ring ~3x).
        let (paused_tx, paused_rx) = mpsc::channel();
        let (resume_tx, resume_rx) = mpsc::channel();
        let lf_frozen = Arc::clone(&lf);
        let frozen_jh = thread::spawn(move || {
            let mut paused_once = false;
            lf_frozen.window_with_hook(|slot_i| {
                if slot_i == 2 && !paused_once {
                    paused_once = true;
                    let _ = paused_tx.send(());
                    let _ = resume_rx.recv();
                }
            })
        });
        paused_rx.recv_timeout(std::time::Duration::from_secs(2)).expect("frozen reader entered hook");

        // Spawn 4 concurrent readers hammering `window()` and checking tear-freedom on every field + pointer.
        let writer_done = Arc::new(AtomicBool::new(false));
        let verified_windows = Arc::new(AtomicU64::new(0));
        let mut reader_jhs = Vec::new();
        for _ in 0..4 {
            let lf_r = Arc::clone(&lf);
            let done_r = Arc::clone(&writer_done);
            let verified_r = Arc::clone(&verified_windows);
            reader_jhs.push(thread::spawn(move || {
                while !done_r.load(Ordering::Acquire) || verified_r.load(Ordering::Relaxed) < 40 {
                    let win = lf_r.window();
                    if win.snapshot_lapped || win.rows.is_empty() {
                        continue;
                    }
                    let end_seq = win.ticks;
                    let expected_len = (end_seq as usize).min(TICK_WINDOW);
                    assert_eq!(win.rows.len(), expected_len, "window length must match ticks");
                    assert_eq!(win.late_count, end_seq / 7, "late_count torn against ticks={end_seq}");
                    assert_eq!(win.tx_totals.frames, end_seq * 2, "tx_frames torn against ticks={end_seq}");
                    assert_eq!(win.tx_totals.would_block, end_seq / 5);
                    assert_eq!(win.tx_totals.enobufs, end_seq / 11);
                    assert_eq!(win.tx_totals.deferred, end_seq / 13);
                    assert_eq!(win.tx_totals.dropped_stale, end_seq / 17);
                    assert_eq!(win.tx_totals.dropped_stopped, end_seq / 19);

                    let first_k = end_seq - (expected_len as u64) + 1;
                    for (idx, got) in win.rows.iter().enumerate() {
                        let k = first_k + idx as u64;
                        let expected = make_sample(k);
                        assert_eq!(
                            *got, expected,
                            "torn read detected in slot idx={idx}, expected release_seq={k}"
                        );
                    }
                    verified_r.fetch_add(1, Ordering::Relaxed);
                }
            }));
        }

        // Single writer runs to completion BEFORE the frozen reader is allowed to resume!
        const TOTAL_SAMPLES: u64 = 6_000;
        for k in 17..=TOTAL_SAMPLES {
            assert!(lf.record(&make_sample(k)));
            if k % 64 == 0 {
                thread::yield_now();
            }
        }
        writer_done.store(true, Ordering::Release);
        for jh in reader_jhs {
            jh.join().expect("reader thread panicked on torn read");
        }
        // Writer completed all 6,000 samples while frozen reader was parked: zero blocking, zero drops!
        assert_eq!(lf.writer_contention_drops(), 0);
        assert!(verified_windows.load(Ordering::Relaxed) >= 40);

        resume_tx.send(()).unwrap();
        let frozen_win = frozen_jh.join().expect("frozen reader panicked");
        // Frozen reader detected slot reuse, retried coherently, and returned an intact final window.
        assert!(!frozen_win.snapshot_lapped);
        assert_eq!(frozen_win.ticks, TOTAL_SAMPLES);
        assert_eq!(frozen_win.rows.len(), TICK_WINDOW);
        assert_eq!(*frozen_win.rows.last().unwrap(), make_sample(TOTAL_SAMPLES));
    }

    #[test]
    fn phases_us_mid_csw_and_max_wait_serialize_and_round_trip() {
        let lf = LockFreeTickStats::new();
        let holder = Holder {
            kind: "write",
            location: std::panic::Location::caller(),
            command: Some("/api/v1/motion/jog"),
            direction: None,
            action_id: Some(77),
            io_timing: None,
            hold_us: 1_800,
            acquire_us: 12,
            acquired_before_wait_us: 400,
            released_before_lock_us: 350,
        };
        let slow_rel = TickSample {
            tick_us: 24_000,
            release_us: 0,
            wait_us: 120,
            lock_us: 1_500,
            post_us: 80,
            phases_us: ReleasePhasesUs {
                alloc_read: 5,
                ack_flush: 12,
                terminal: 18_200,
                guard_drop: 42,
                post: 80,
                publish: 0,
            },
            usage: ThreadUsage {
                mid_nvcsw: 2,
                mid_nivcsw: 5,
                ..Default::default()
            },
            ..Default::default()
        };
        let seq1 = lf
            .record_staged_with_publish(&slow_rel, || (25_100, 95))
            .expect("staged record");
        // Monotonic amendment increases both release_us and publish without decreasing either.
        lf.amend_slot_release_and_publish_us(seq1, 24_000, 50);
        lf.amend_slot_release_and_publish_us(seq1, 25_600, 140);

        let long_wait = TickSample {
            tick_us: 9_500,
            release_us: 9_600,
            wait_us: 8_200,
            lock_us: 1_100,
            post_us: 40,
            phases_us: ReleasePhasesUs {
                alloc_read: 2,
                ack_flush: 3,
                terminal: 4,
                guard_drop: 5,
                post: 40,
                publish: 15,
            },
            waited_for: Some(holder),
            longest_hold: Some(holder),
            ..Default::default()
        };
        assert!(lf.record(&long_wait));

        let snap = lf.snapshot();
        let mr = snap.max_release.expect("max_release");
        assert_eq!(mr.sample.release_us, 25_600);
        assert_eq!(
            mr.sample.phases_us,
            ReleasePhasesUs {
                alloc_read: 5,
                ack_flush: 12,
                terminal: 18_200,
                guard_drop: 42,
                post: 80,
                publish: 140,
            }
        );
        assert_eq!(mr.sample.usage.mid_nvcsw, 2);
        assert_eq!(mr.sample.usage.mid_nivcsw, 5);
        assert_eq!(mr.handoff_us, None);

        let mw = snap.max_wait.expect("max_wait");
        assert_eq!(mw.sample.wait_us, 8_200);
        assert_eq!(mw.handoff_us, Some(350));
        assert_eq!(mw.ticks_ago, 0);
        assert_eq!(mw.sample.waited_for.unwrap().command, Some("/api/v1/motion/jog"));

        let json = serde_json::to_value(&snap).unwrap();
        assert_eq!(json["max_release"]["phases_us"]["alloc_read"], 5);
        assert_eq!(json["max_release"]["phases_us"]["ack_flush"], 12);
        assert_eq!(json["max_release"]["phases_us"]["terminal"], 18_200);
        assert_eq!(json["max_release"]["phases_us"]["guard_drop"], 42);
        assert_eq!(json["max_release"]["phases_us"]["post"], 80);
        assert_eq!(json["max_release"]["phases_us"]["publish"], 140);
        assert_eq!(json["max_release"]["mid_nvcsw"], 2);
        assert_eq!(json["max_release"]["mid_nivcsw"], 5);
        assert!(json["max_release"].get("handoff_us").is_none());

        assert_eq!(json["max_tick"]["phases_us"]["terminal"], 18_200);
        assert_eq!(json["max_tick"]["mid_nvcsw"], 2);
        assert_eq!(json["max_tick"]["mid_nivcsw"], 5);

        assert_eq!(json["max_wait"]["wait_us"], 8_200);
        assert_eq!(json["max_wait"]["handoff_us"], 350);
        assert_eq!(json["max_wait"]["waited_for"]["command"], "/api/v1/motion/jog");
        assert_eq!(json["max_wait"]["waited_for"]["released_before_lock_us"], 350);
    }
}
