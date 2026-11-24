//! Control tick timing (`GET /api/v1/diagnostics/tick`): a fixed ring of the last
//! [`TICK_WINDOW`] ticks, recorded by [`crate::ArmSupervisor::tick`].

use std::cell::Cell;

use serde::{Serialize, Serializer};

/// Ticks the percentiles cover.
pub const TICK_WINDOW: usize = 1000;

/// Strict wakeup-lateness thresholds evaluated on raw nanoseconds (`actual - scheduled`).
pub const WAKE_LATENESS_1MS_NS: u64 = 1_000_000;
pub const WAKE_LATENESS_5MS_NS: u64 = 5_000_000;
pub const WAKE_LATENESS_10MS_NS: u64 = 10_000_000;

/// Converts raw wakeup lateness (`actual_release_start - scheduled_release`, ns) to saturating
/// microseconds and strict threshold flags (`> 1 ms`, `> 5 ms`, `> 10 ms` on raw nanoseconds).
#[inline]
pub fn classify_wake_lateness_ns(lateness_ns: u64) -> (u32, bool, bool, bool) {
    let us = (lateness_ns / 1_000).min(u64::from(u32::MAX)) as u32;
    (
        us,
        lateness_ns > WAKE_LATENESS_1MS_NS,
        lateness_ns > WAKE_LATENESS_5MS_NS,
        lateness_ns > WAKE_LATENESS_10MS_NS,
    )
}

thread_local! {
    static CURRENT_WAKE_LATENESS_NS: Cell<u64> = const { Cell::new(0) };
}

/// Scoped thread-local handoff carrying raw wakeup lateness (ns) from `run_periodic` into
/// `ArmSupervisor::tick_shared` on the same OS thread; restores the prior value on drop
/// (including panic unwinds and early returns).
pub struct WakeLatenessGuard {
    prev: u64,
}

impl WakeLatenessGuard {
    #[inline]
    pub fn enter(lateness_ns: u64) -> Self {
        let prev = CURRENT_WAKE_LATENESS_NS
            .try_with(|c| c.replace(lateness_ns))
            .unwrap_or(0);
        Self { prev }
    }
}

impl Drop for WakeLatenessGuard {
    #[inline]
    fn drop(&mut self) {
        let _ = CURRENT_WAKE_LATENESS_NS.try_with(|c| c.set(self.prev));
    }
}

/// Takes and clears the calling thread's pending raw wakeup lateness (ns).
#[inline]
pub fn take_current_wake_lateness_ns() -> u64 {
    CURRENT_WAKE_LATENESS_NS
        .try_with(|c| c.replace(0))
        .unwrap_or(0)
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

thread_local! {
    static TX_TICK: Cell<TxTickMetrics> = const { Cell::new(TxTickMetrics::ZERO) };
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
        let mut st = s.serialize_struct("Holder", 6)?;
        st.serialize_field("kind", self.kind)?;
        st.serialize_field("location", &format!("{}:{}", self.location.file(), self.location.line()))?;
        st.serialize_field("hold_us", &self.hold_us)?;
        st.serialize_field("acquire_us", &self.acquire_us)?;
        st.serialize_field("acquired_before_wait_us", &self.acquired_before_wait_us)?;
        st.serialize_field("released_before_lock_us", &self.released_before_lock_us)?;
        st.end()
    }
}

/// One tick, µs. Serialized as the breakdown of `max_tick` / `max_lock`.
#[derive(Debug, Clone, Copy, Default, Serialize, PartialEq, Eq)]
pub struct TickSample {
    /// Whole `ArmSupervisor::tick` call.
    pub tick_us: u32,
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
    #[serde(flatten)]
    pub usage: ThreadUsage,
    /// The guard the tick waited for; None when no other guard was released since the
    /// previous tick (or the log was busy).
    pub waited_for: Option<Holder>,
    /// The longest other hold since the previous tick (may differ from `waited_for`).
    pub longest_hold: Option<Holder>,
    #[serde(skip)]
    pub late: bool,
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
    /// Priority asked for (`ARMD_RT_PRIORITY`, default 45; `crate::rt_core::sched`).
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
}

#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
pub struct TickSnapshot {
    /// Ticks in the window (at most [`TICK_WINDOW`]).
    pub window: usize,
    /// Whole `ArmSupervisor::tick` call: lock wait, tick body and telemetry broadcast.
    pub tick_us: Percentiles,
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
    /// The slowest tick in the window, all phases.
    pub max_tick: Option<TickBreakdown>,
    /// The tick with the longest lock hold in the window, all phases.
    pub max_lock: Option<TickBreakdown>,
    /// The tick with the largest summed CAN TX send wall time in the window.
    pub max_tx: Option<TickBreakdown>,
    /// Per-frame CAN TX timing and backpressure metrics over the window.
    pub tx: TxDiagnostics,
    /// Control thread scheduling; None until the control thread reported it (tests).
    pub control_thread: Option<ControlThreadSched>,
    pub observation_cache: ObservationCacheStats,
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

/// Copy of the ring taken under the mutex; [`TickWindow::snapshot`] sorts outside it.
pub struct TickWindow {
    rows: Vec<TickSample>,
    late_count: u64,
    ticks: u64,
    tx_totals: TxTotals,
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
        }
    }

    pub fn snapshot(&self) -> TickSnapshot { self.window().snapshot() }
}

impl TickWindow {
    /// The samples, oldest first.
    pub fn samples(&self) -> &[TickSample] { &self.rows }

    fn col(&self, f: impl Fn(&TickSample) -> u32) -> Percentiles {
        let mut v: Vec<u32> = self.rows.iter().map(f).collect();
        percentiles(&mut v)
    }

    fn sum(&self, f: impl Fn(&TickSample) -> u32) -> u64 {
        self.rows.iter().fold(0u64, |acc, r| acc.saturating_add(u64::from(f(r))))
    }

    /// The latest of equal maxima of `key`.
    fn max_by(&self, key: impl Fn(&TickSample) -> u32) -> Option<TickBreakdown> {
        let n = self.rows.len();
        self.rows.iter().enumerate().rev().max_by_key(|(_, r)| key(r))
            .map(|(i, r)| TickBreakdown { sample: *r, ticks_ago: n - 1 - i })
    }

    pub fn snapshot(&self) -> TickSnapshot {
        TickSnapshot {
            window: self.rows.len(),
            tick_us: self.col(|r| r.tick_us),
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
            max_lock: self.max_by(|r| r.lock_us),
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
            },
            control_thread: None,
            observation_cache: ObservationCacheStats::default(),
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
        assert!(mt["waited_for"]["location"].as_str().unwrap().starts_with("src/diagnostics/tick.rs:"), "{mt}");
        assert!(mt["longest_hold"].is_null());
        assert!(mt.get("late").is_none());
        assert_eq!(json["max_lock"]["rx_us"], 12_000);
        assert_eq!(json["max_lock"]["lock_majflt"], 3);
        assert_eq!(json["lock_phases"]["rx_us"]["max"], 12_000);
        assert!(json["control_thread"].is_null());
        assert!(json["tx"]["send_us"]["max"].is_u64());
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
}
