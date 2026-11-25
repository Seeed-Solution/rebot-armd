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
use std::collections::{HashMap, VecDeque};
use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::{mpsc, Arc, Condvar, Mutex};
use std::thread::{self, JoinHandle, Thread, ThreadId};
use std::time::{Duration, Instant};

use rtrb::{Consumer, Producer, RingBuffer};

use super::broker::{BootId, Broker, Capacities, ClientId};
use super::config_iface::{ConfigAuthority, ConfigDomain, ConfigProposal, Reservation};
use super::ids::ArmId;
use super::mailbox::TargetWriter;
use super::ops::{CoreOps, PlanSlot, PlanningSnapshot, ReleaseEngine, MAX_SERVICE_WAKES};
use super::publish::{Snapshot, StateHub};
use super::sched::{self, MemoryLock, RtReadiness, RtSchedConfig};
use super::urgent::{
    urgent_stop_channel, RetiredStopPayloads, StopRequests, UrgentStop, UrgentStopClient,
    UrgentStopError, UrgentStopOutcome, URGENT_STOP_CAPACITY,
};
use crate::diagnostics::tick::{
    LockPhases, ThreadUsage, TickSample, UsageMark, RELEASE_FLAG_CLEAR_OK,
    RELEASE_FLAG_CLEAR_REFUSED, RELEASE_FLAG_ESTOP_APPLIED, RELEASE_FLAG_INJECT_ALLOC,
    RELEASE_FLAG_SAFETY_LATCH, RELEASE_FLAG_THERMAL_TRIP, RELEASE_FLAG_WATCHDOG_LATCH,
};
use crate::model::{ArmObservation, GripperPolicy, JawOpening};
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
/// Bounded capacity of the off-thread retired `ActiveTrajectory` reclamation ring.
pub const RECLAIM_TRAJ_CAPACITY: usize = 8;
/// Capacity of the persistence reply ring (`armd-control` -> `armd-persist` / `OffThreadSinks`).
/// Strictly carries 1-to-1 `PersistCmd` replies (never unsolicited job reclamation).
pub const PERSIST_REP_CAPACITY: usize = 8;
/// Lock-free SPSC capacity for off-thread reclamation (runtime rejected persistence jobs,
/// deferred `legacy_j6` warnings, and clean-exit overflow payloads when consumer rings stay full).
/// Sized strictly >= `17 maint + 6 persist + 12 runtime jobs/logs + 64 pending_estop_acks + 64 retired states + 64 retired stops + 8 retired trajectories + 4 retired zero-check contexts`.
const OFFLOAD_RECLAIM_CAPACITY: usize = MAINT_QUEUE_CAPACITY
    + 1
    + 6
    + 12
    + ESTOP_ACK_CAPACITY
    + RECLAIM_STATE_CAPACITY
    + URGENT_STOP_CAPACITY
    + RECLAIM_TRAJ_CAPACITY
    + 4
    + 2;

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

/// Coherent snapshot of control-thread-written monotonic qualification counters and any
/// operator-requested baseline marker, read lock-free via [`ExecutorStats::snapshot_monotonic`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MonotonicStatsSnapshot {
    pub coherent: bool,
    pub loop_active: bool,
    pub releases_started: u64,
    pub releases_total: u64,
    pub in_flight_release_us: u64,
    pub release_us_max_ever: u64,
    pub releases_over_10ms: u64,
    pub releases_over_15ms: u64,
    pub releases_over_20ms: u64,
    pub wake_late_max_ever: u64,
    pub skipped_releases: u64,
    pub diag_dropped: u64,
    pub wake_lateness_over_1ms: u64,
    pub wake_lateness_over_5ms: u64,
    pub wake_lateness_over_10ms: u64,
    pub since_marker: Option<crate::diagnostics::tick::TickSinceMarker>,
}

const MAX_MONOTONIC_SEQLOCK_RETRIES: usize = 64;

/// Counters the executor / periodic control thread updates with plain atomic stores.
#[derive(Debug, Default)]
pub struct ExecutorStats {
    /// Steps started.
    pub steps: AtomicU64,
    /// Whole releases completed (Release-incremented strictly AFTER `release_allocs` is updated).
    pub completed_releases: AtomicU64,
    /// Monotonic releases started since process start (control-thread incremented at wake).
    pub releases_started: AtomicU64,
    /// Monotonic completed releases since process start (control-thread written only).
    pub releases_total: AtomicU64,
    /// Monotonic start timestamp (`mono_now_ns().max(1)`) of an in-flight release, or `0` when idle.
    in_flight_start_mono_ns: AtomicU64,
    /// Monotonic all-time maximum full `release_us` since process start, µs (control-thread written only).
    pub release_us_max_ever: AtomicU64,
    /// Monotonic releases since process start with full duration `>= 10 ms` / `>= 15 ms` / `>= 20 ms`.
    pub releases_over_10ms: AtomicU64,
    pub releases_over_15ms: AtomicU64,
    pub releases_over_20ms: AtomicU64,
    /// Monotonic all-time maximum wakeup lateness since process start, µs (control-thread written only).
    pub wake_late_max_ever: AtomicU64,
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
    /// True while the owning periodic control loop is actively running (`control_thread_main` or
    /// `run_periodic_with_clock`); cleared via RAII guard on exit.
    loop_active: AtomicBool,
    /// Monotonic marker request generation written ONLY by `POST /api/v1/diagnostics/tick/mark`.
    marker_requested_gen: AtomicU64,
    /// Control-thread seqlock protecting coherent publication of all monotonic counters,
    /// `marker_ack_gen`, baseline counter snapshots, and `since_*` peaks.
    stats_seq: AtomicU64,
    /// Control-thread acknowledged marker generation.
    marker_ack_gen: AtomicU64,
    base_releases_total: AtomicU64,
    base_releases_over_10ms: AtomicU64,
    base_releases_over_15ms: AtomicU64,
    base_releases_over_20ms: AtomicU64,
    base_skipped_releases: AtomicU64,
    base_diag_dropped: AtomicU64,
    base_wake_over_1ms: AtomicU64,
    base_wake_over_5ms: AtomicU64,
    base_wake_over_10ms: AtomicU64,
    since_release_us_max: AtomicU64,
    since_wake_late_max: AtomicU64,
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
    /// Optional one-shot test delay (ns) injected strictly AFTER sample staging / diag enqueue
    /// and strictly BEFORE final timestamp, sample amendment, and completion publication (0 in production).
    #[doc(hidden)]
    pub post_stats_delay_once_ns: AtomicU64,
    /// Optional one-shot test delay (ns) injected strictly AFTER sample amendment and tail-allocation
    /// accounting and strictly BEFORE the `stats_seq` completion seal (0 in production).
    #[doc(hidden)]
    pub pre_seal_delay_once_ns: AtomicU64,
    /// Optional one-shot test delay (ns) injected strictly AFTER completion publication and right
    /// BEFORE wait-entry (`sleep_until_ns`) / loop-exit check to test post-commit preemption
    /// accounting on both continuation and shutdown (0 in production).
    #[doc(hidden)]
    pub before_sleep_delay_once_ns: AtomicU64,
    /// Optional one-shot test delay (ns) injected inside Linux `sleep_until_ns` strictly AFTER the
    /// `mono_now_ns() >= at_ns` entry check and BEFORE the kernel sleep syscall (0 in production).
    #[doc(hidden)]
    pub check_to_sleep_delay_once_ns: AtomicU64,
    /// 4-byte aligned futex word mirrored from `stop` (`0` running, `1` stop requested) so Linux
    /// `FUTEX_WAIT_BITSET` atomically detects stop-before-sleep (`EAGAIN`) and wakes immediately
    /// on shutdown without spinning.
    stop_word: AtomicU32,
    /// Monotonic `CLOCK_MONOTONIC` timestamp (`mono_now_ns().max(1)`) recorded once when
    /// [`Self::signal_stop`] is called (`0` if stop was flipped via a raw `&AtomicBool`).
    stop_requested_mono_ns: AtomicU64,
    /// Cumulative count of off-thread `diag_amend_rx` amendments whose target sample had already
    /// been evicted from `LockFreeTickStats` (merged into `stats_writer_drops` as explicit coverage loss).
    pub amend_missed: AtomicU64,
    /// Optional lock-free tick ring attached in legacy mode (`attach_legacy_loop_stats`) so
    /// `run_periodic_with_clock` publishes each legacy sample once at release completion.
    legacy_tick_ring: std::sync::OnceLock<Arc<crate::diagnostics::tick::LockFreeTickStats>>,
}

impl ExecutorStats {
    /// Records the monotonic shutdown request timestamp, sets `stop` and `stop_word`,
    /// and wakes any thread parked in Linux `FUTEX_WAIT_BITSET`.
    pub fn signal_stop(&self, stop: &AtomicBool) {
        let now = mono_now_ns().max(1);
        let _ = self.stop_requested_mono_ns.compare_exchange(
            0,
            now,
            Ordering::AcqRel,
            Ordering::Acquire,
        );
        stop.store(true, Ordering::Release);
        self.stop_word.store(1, Ordering::Release);
        #[cfg(target_os = "linux")]
        unsafe {
            const FUTEX_WAKE: libc::c_int = 1;
            const FUTEX_PRIVATE_FLAG: libc::c_int = 128;
            libc::syscall(
                libc::SYS_futex,
                &self.stop_word as *const AtomicU32,
                FUTEX_WAKE | FUTEX_PRIVATE_FLAG,
                i32::MAX,
                std::ptr::null::<libc::timespec>(),
                std::ptr::null::<u32>(),
                0u32,
            );
        }
    }

    /// Attaches the supervisor's [`LockFreeTickStats`] ring for legacy single-completion publication.
    pub(crate) fn attach_legacy_tick_ring(&self, ring: Arc<crate::diagnostics::tick::LockFreeTickStats>) {
        let _ = self.legacy_tick_ring.set(ring);
    }

    #[inline]
    pub(crate) fn has_legacy_tick_ring(&self) -> bool {
        self.legacy_tick_ring.get().is_some()
    }

    /// Returns `true` while the owning control loop is actively running.
    #[inline]
    pub fn is_loop_active(&self) -> bool {
        self.loop_active.load(Ordering::Acquire)
    }

    /// Marks the owning control loop active inside the `stats_seq` seqlock.
    #[inline]
    pub fn mark_loop_active(&self) {
        let s0 = self.stats_seq.load(Ordering::Relaxed);
        self.stats_seq.store(s0.wrapping_add(1), Ordering::Relaxed);
        std::sync::atomic::fence(Ordering::Release);
        self.loop_active.store(true, Ordering::Relaxed);
        self.stats_seq.store(s0.wrapping_add(2), Ordering::Release);
    }

    /// Atomically publishes loop shutdown inside the `stats_seq` seqlock: adds any final
    /// `exit_skips` to `skipped_releases`, clears `in_flight_start_mono_ns = 0`, and sets
    /// `loop_active = false`. Idempotent when already inactive with `exit_skips == 0`.
    #[inline]
    pub fn finalize_loop_exit(&self, exit_skips: u64) {
        if exit_skips == 0
            && !self.loop_active.load(Ordering::Relaxed)
            && self.in_flight_start_mono_ns.load(Ordering::Relaxed) == 0
        {
            return;
        }
        let s0 = self.stats_seq.load(Ordering::Relaxed);
        self.stats_seq.store(s0.wrapping_add(1), Ordering::Relaxed);
        std::sync::atomic::fence(Ordering::Release);
        if exit_skips > 0 {
            let skips = self.skipped_releases.load(Ordering::Relaxed).saturating_add(exit_skips);
            self.skipped_releases.store(skips, Ordering::Relaxed);
        }
        self.in_flight_start_mono_ns.store(0, Ordering::Relaxed);
        self.loop_active.store(false, Ordering::Relaxed);
        self.stats_seq.store(s0.wrapping_add(2), Ordering::Release);
    }

    /// Records scheduled deadlines crossed at a release-wait boundary after the prior release
    /// sealed (separate `skipped_releases` event under `stats_seq`; never rewrites sealed
    /// release durations or prior marker baselines).
    #[inline]
    pub fn record_boundary_skips(&self, skipped_delta: u64) {
        if skipped_delta == 0 {
            return;
        }
        let s0 = self.stats_seq.load(Ordering::Relaxed);
        self.stats_seq.store(s0.wrapping_add(1), Ordering::Relaxed);
        std::sync::atomic::fence(Ordering::Release);
        let skips = self.skipped_releases.load(Ordering::Relaxed).saturating_add(skipped_delta);
        self.skipped_releases.store(skips, Ordering::Relaxed);
        self.stats_seq.store(s0.wrapping_add(2), Ordering::Release);
    }

    /// Requests a coherent control-thread baseline marker (`POST /api/v1/diagnostics/tick/mark`).
    /// Writes ONLY `marker_requested_gen`; never touches any control-thread counter/peak atomic.
    pub fn request_marker(&self) -> u64 {
        self.marker_requested_gen.fetch_add(1, Ordering::AcqRel).wrapping_add(1)
    }

    /// Marks the start of one control release right after wake lateness sampling (zero allocation).
    #[inline]
    pub fn mark_release_started(&self) {
        let start_mono = mono_now_ns().max(1);
        let s0 = self.stats_seq.load(Ordering::Relaxed);
        self.stats_seq.store(s0.wrapping_add(1), Ordering::Relaxed);
        std::sync::atomic::fence(Ordering::Release);
        let started = self.releases_started.load(Ordering::Relaxed).saturating_add(1);
        self.releases_started.store(started, Ordering::Relaxed);
        self.in_flight_start_mono_ns.store(start_mono, Ordering::Relaxed);
        self.stats_seq.store(s0.wrapping_add(2), Ordering::Release);
    }

    /// Classifies raw wakeup lateness (`actual_release_start - scheduled_release`, ns), updates
    /// the cumulative `> 1 / 5 / 10 ms` counters and `wake_late_max_ever` atomically (zero
    /// allocation), and returns the saturating microsecond value for `TickSample::wake_lateness_us`.
    #[inline]
    pub fn record_wake_lateness_ns(&self, lateness_ns: u64) -> u32 {
        let (us, over_1ms, over_5ms, over_10ms) =
            crate::diagnostics::tick::classify_wake_lateness_ns(lateness_ns);
        let s0 = self.stats_seq.load(Ordering::Relaxed);
        self.stats_seq.store(s0.wrapping_add(1), Ordering::Relaxed);
        std::sync::atomic::fence(Ordering::Release);
        if over_1ms {
            self.wake_lateness_over_1ms.fetch_add(1, Ordering::Relaxed);
        }
        if over_5ms {
            self.wake_lateness_over_5ms.fetch_add(1, Ordering::Relaxed);
        }
        if over_10ms {
            self.wake_lateness_over_10ms.fetch_add(1, Ordering::Relaxed);
        }
        self.wake_late_max_ever.fetch_max(u64::from(us), Ordering::Relaxed);
        self.stats_seq.store(s0.wrapping_add(2), Ordering::Release);
        us
    }

    /// Commits one completed (or panicked) release's full monotonic qualification outcome STRICTLY
    /// LAST inside a single control-thread seqlock transaction (zero allocation, zero mutex):
    /// - atomically clears `in_flight_start_mono_ns = 0` BEFORE taking the final release end timestamp;
    /// - samples `end_ns = now_ns()` strictly AFTER all release work (sample push/amend, dispatcher
    ///   wake, B2 tail allocation accounting, and clearing `in_flight_start_mono_ns`);
    /// - computes `overrun_skipped = releases.advance_overrun(end_ns)` from that same `end_ns`;
    /// - updates `wake_lateness_over_{1,5,10}ms`, `wake_late_max_ever`, `release_us_max_ever`,
    ///   `releases_over_{10,15,20}ms`, `skipped_releases`, `diag_dropped`, active `since_marker`
    ///   peaks, any pending marker acknowledgement, and `releases_total` as the final payload store
    ///   before publishing `stats_seq` with `Ordering::Release`.
    #[inline]
    pub fn commit_release_outcome_with_clock(
        &self,
        actual_ns: u64,
        wake_lateness_ns: u64,
        wake_skipped: u64,
        diag_dropped_delta: u64,
        releases: &mut Releases,
        now_ns: impl FnOnce() -> u64,
    ) {
        let s0 = self.stats_seq.load(Ordering::Relaxed);
        self.stats_seq.store(s0.wrapping_add(1), Ordering::Relaxed);
        std::sync::atomic::fence(Ordering::Release);

        self.in_flight_start_mono_ns.store(0, Ordering::Relaxed);
        let end_ns = now_ns();
        let release_ns = end_ns.saturating_sub(actual_ns);
        let overrun_skipped = releases.advance_overrun(end_ns);
        let skipped_delta = wake_skipped.saturating_add(overrun_skipped);
        let (wake_us, w1, w5, w10) = crate::diagnostics::tick::classify_wake_lateness_ns(wake_lateness_ns);
        let (rel_us, r10, r15, r20) = crate::diagnostics::tick::classify_release_duration_ns(release_ns);

        let prev_rel_max = self.release_us_max_ever.load(Ordering::Relaxed);
        if u64::from(rel_us) > prev_rel_max {
            self.release_us_max_ever.store(u64::from(rel_us), Ordering::Relaxed);
        }
        let rel_10 = self.releases_over_10ms.load(Ordering::Relaxed).saturating_add(u64::from(r10));
        self.releases_over_10ms.store(rel_10, Ordering::Relaxed);
        let rel_15 = self.releases_over_15ms.load(Ordering::Relaxed).saturating_add(u64::from(r15));
        self.releases_over_15ms.store(rel_15, Ordering::Relaxed);
        let rel_20 = self.releases_over_20ms.load(Ordering::Relaxed).saturating_add(u64::from(r20));
        self.releases_over_20ms.store(rel_20, Ordering::Relaxed);

        let prev_wake_max = self.wake_late_max_ever.load(Ordering::Relaxed);
        if u64::from(wake_us) > prev_wake_max {
            self.wake_late_max_ever.store(u64::from(wake_us), Ordering::Relaxed);
        }
        let wake_1 = self.wake_lateness_over_1ms.load(Ordering::Relaxed).saturating_add(u64::from(w1));
        self.wake_lateness_over_1ms.store(wake_1, Ordering::Relaxed);
        let wake_5 = self.wake_lateness_over_5ms.load(Ordering::Relaxed).saturating_add(u64::from(w5));
        self.wake_lateness_over_5ms.store(wake_5, Ordering::Relaxed);
        let wake_10 = self.wake_lateness_over_10ms.load(Ordering::Relaxed).saturating_add(u64::from(w10));
        self.wake_lateness_over_10ms.store(wake_10, Ordering::Relaxed);

        let skips = self.skipped_releases.load(Ordering::Relaxed).saturating_add(skipped_delta);
        self.skipped_releases.store(skips, Ordering::Relaxed);
        let diag_drops = self.diag_dropped.load(Ordering::Relaxed).saturating_add(diag_dropped_delta);
        self.diag_dropped.store(diag_drops, Ordering::Relaxed);

        let rel_total = self.releases_total.load(Ordering::Relaxed).saturating_add(1);

        let ack = self.marker_ack_gen.load(Ordering::Relaxed);
        if ack > 0 {
            let sr = self.since_release_us_max.load(Ordering::Relaxed);
            if u64::from(rel_us) > sr {
                self.since_release_us_max.store(u64::from(rel_us), Ordering::Relaxed);
            }
            let sw = self.since_wake_late_max.load(Ordering::Relaxed);
            if u64::from(wake_us) > sw {
                self.since_wake_late_max.store(u64::from(wake_us), Ordering::Relaxed);
            }
        }

        let req = self.marker_requested_gen.load(Ordering::Acquire);
        if req != ack {
            self.base_releases_total.store(rel_total, Ordering::Relaxed);
            self.base_releases_over_10ms.store(rel_10, Ordering::Relaxed);
            self.base_releases_over_15ms.store(rel_15, Ordering::Relaxed);
            self.base_releases_over_20ms.store(rel_20, Ordering::Relaxed);
            self.base_skipped_releases.store(skips, Ordering::Relaxed);
            self.base_diag_dropped.store(diag_drops, Ordering::Relaxed);
            self.base_wake_over_1ms.store(wake_1, Ordering::Relaxed);
            self.base_wake_over_5ms.store(wake_5, Ordering::Relaxed);
            self.base_wake_over_10ms.store(wake_10, Ordering::Relaxed);
            self.since_release_us_max.store(0, Ordering::Relaxed);
            self.since_wake_late_max.store(0, Ordering::Relaxed);
            self.marker_ack_gen.store(req, Ordering::Relaxed);
        }

        self.releases_total.store(rel_total, Ordering::Relaxed);
        self.stats_seq.store(s0.wrapping_add(2), Ordering::Release);
    }

    /// Reads one atomically coherent snapshot of all monotonic counters and `since_marker`
    /// with bounded seqlock retries (never spins infinitely).
    pub fn snapshot_monotonic(&self) -> MonotonicStatsSnapshot {
        for _ in 0..MAX_MONOTONIC_SEQLOCK_RETRIES {
            let s1 = self.stats_seq.load(Ordering::Acquire);
            if s1 & 1 != 0 {
                std::hint::spin_loop();
                continue;
            }
            let loop_active = self.loop_active.load(Ordering::Relaxed);
            let releases_started = self.releases_started.load(Ordering::Relaxed);
            let releases_total = self.releases_total.load(Ordering::Relaxed);
            let in_flight_start = self.in_flight_start_mono_ns.load(Ordering::Relaxed);
            let release_us_max_ever = self.release_us_max_ever.load(Ordering::Relaxed);
            let releases_over_10ms = self.releases_over_10ms.load(Ordering::Relaxed);
            let releases_over_15ms = self.releases_over_15ms.load(Ordering::Relaxed);
            let releases_over_20ms = self.releases_over_20ms.load(Ordering::Relaxed);
            let wake_late_max_ever = self.wake_late_max_ever.load(Ordering::Relaxed);
            let skipped_releases = self.skipped_releases.load(Ordering::Relaxed);
            let diag_dropped = self.diag_dropped.load(Ordering::Relaxed);
            let wake_lateness_over_1ms = self.wake_lateness_over_1ms.load(Ordering::Relaxed);
            let wake_lateness_over_5ms = self.wake_lateness_over_5ms.load(Ordering::Relaxed);
            let wake_lateness_over_10ms = self.wake_lateness_over_10ms.load(Ordering::Relaxed);
            let ack = self.marker_ack_gen.load(Ordering::Relaxed);
            let b_rel = self.base_releases_total.load(Ordering::Relaxed);
            let b_r10 = self.base_releases_over_10ms.load(Ordering::Relaxed);
            let b_r15 = self.base_releases_over_15ms.load(Ordering::Relaxed);
            let b_r20 = self.base_releases_over_20ms.load(Ordering::Relaxed);
            let b_skip = self.base_skipped_releases.load(Ordering::Relaxed);
            let b_diag = self.base_diag_dropped.load(Ordering::Relaxed);
            let b_w1 = self.base_wake_over_1ms.load(Ordering::Relaxed);
            let b_w5 = self.base_wake_over_5ms.load(Ordering::Relaxed);
            let b_w10 = self.base_wake_over_10ms.load(Ordering::Relaxed);
            let s_rel_max = self.since_release_us_max.load(Ordering::Relaxed);
            let s_wake_max = self.since_wake_late_max.load(Ordering::Relaxed);
            // Sample wall clock STRICTLY BEFORE `fence(Acquire)` + `s2 == s1` check so a
            // reader preempted between reading `in_flight_start` and calling `mono_now_ns()`
            // rejects the snapshot if the release sealed or the loop exited meanwhile.
            let now_mono = if loop_active && in_flight_start != 0 && releases_started > releases_total {
                mono_now_ns()
            } else {
                0
            };
            std::sync::atomic::fence(Ordering::Acquire);
            let s2 = self.stats_seq.load(Ordering::Relaxed);
            if s1 == s2 {
                let in_flight_release_us = if now_mono != 0 {
                    now_mono.saturating_sub(in_flight_start) / 1_000
                } else {
                    0
                };
                let req = self.marker_requested_gen.load(Ordering::Acquire);
                let since_marker = if req == 0 {
                    None
                } else if ack == req {
                    Some(crate::diagnostics::tick::TickSinceMarker {
                        marker_id: req,
                        status: "active",
                        releases_total_delta: Some(releases_total.saturating_sub(b_rel)),
                        releases_over_10ms_delta: Some(releases_over_10ms.saturating_sub(b_r10)),
                        releases_over_15ms_delta: Some(releases_over_15ms.saturating_sub(b_r15)),
                        releases_over_20ms_delta: Some(releases_over_20ms.saturating_sub(b_r20)),
                        skipped_releases_delta: Some(skipped_releases.saturating_sub(b_skip)),
                        diag_dropped_delta: Some(diag_dropped.saturating_sub(b_diag)),
                        wake_lateness_over_1ms_delta: Some(wake_lateness_over_1ms.saturating_sub(b_w1)),
                        wake_lateness_over_5ms_delta: Some(wake_lateness_over_5ms.saturating_sub(b_w5)),
                        wake_lateness_over_10ms_delta: Some(wake_lateness_over_10ms.saturating_sub(b_w10)),
                        release_us_max: Some(s_rel_max),
                        wake_late_max: Some(s_wake_max),
                    })
                } else {
                    Some(crate::diagnostics::tick::TickSinceMarker {
                        marker_id: req,
                        status: if loop_active { "pending" } else { "inactive" },
                        releases_total_delta: None,
                        releases_over_10ms_delta: None,
                        releases_over_15ms_delta: None,
                        releases_over_20ms_delta: None,
                        skipped_releases_delta: None,
                        diag_dropped_delta: None,
                        wake_lateness_over_1ms_delta: None,
                        wake_lateness_over_5ms_delta: None,
                        wake_lateness_over_10ms_delta: None,
                        release_us_max: None,
                        wake_late_max: None,
                    })
                };
                return MonotonicStatsSnapshot {
                    coherent: true,
                    loop_active,
                    releases_started,
                    releases_total,
                    in_flight_release_us,
                    release_us_max_ever,
                    releases_over_10ms,
                    releases_over_15ms,
                    releases_over_20ms,
                    wake_late_max_ever,
                    skipped_releases,
                    diag_dropped,
                    wake_lateness_over_1ms,
                    wake_lateness_over_5ms,
                    wake_lateness_over_10ms,
                    since_marker,
                };
            }
            std::hint::spin_loop();
        }
        MonotonicStatsSnapshot {
            coherent: false,
            loop_active: self.loop_active.load(Ordering::Relaxed),
            releases_started: self.releases_started.load(Ordering::Relaxed),
            releases_total: self.releases_total.load(Ordering::Relaxed),
            in_flight_release_us: 0,
            release_us_max_ever: self.release_us_max_ever.load(Ordering::Relaxed),
            releases_over_10ms: self.releases_over_10ms.load(Ordering::Relaxed),
            releases_over_15ms: self.releases_over_15ms.load(Ordering::Relaxed),
            releases_over_20ms: self.releases_over_20ms.load(Ordering::Relaxed),
            wake_late_max_ever: self.wake_late_max_ever.load(Ordering::Relaxed),
            skipped_releases: self.skipped_releases.load(Ordering::Relaxed),
            diag_dropped: self.diag_dropped.load(Ordering::Relaxed),
            wake_lateness_over_1ms: self.wake_lateness_over_1ms.load(Ordering::Relaxed),
            wake_lateness_over_5ms: self.wake_lateness_over_5ms.load(Ordering::Relaxed),
            wake_lateness_over_10ms: self.wake_lateness_over_10ms.load(Ordering::Relaxed),
            since_marker: None,
        }
    }

    #[doc(hidden)]
    pub fn set_odd_stats_seq_for_test(&self, odd: bool) {
        let cur = self.stats_seq.load(Ordering::Relaxed);
        let target = if odd { cur | 1 } else { cur & !1 };
        self.stats_seq.store(target, Ordering::Release);
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

/// Immutable tool/follower/path identity for gripper baseline persistence, captured once before
/// transferring `SupervisorInner` onto `armd-control` so baseline maintenance never clones strings
/// or paths on the control thread.
#[derive(Debug, Clone)]
pub struct BaselineIdentity {
    pub tool_id: String,
    pub tool_config_digest: String,
    pub follower: Option<String>,
    pub path: Option<std::path::PathBuf>,
}

/// Payload returned by [`MaintenanceOp::TakeBaselinePending`] without allocating or freeing on
/// `armd-control`.
pub struct BaselineTakeReply {
    pub moves: Vec<crate::gripper_baseline::MoveRecord>,
    pub current: Option<crate::gripper_baseline::BaselineData>,
    /// `(closed_rad, open_rad, initial_max_nm, calibration_initial_max_nm)` when a gripper policy exists.
    pub policy: Option<(f32, f32, f32, f32)>,
    pub generation: u64,
    pub dirty: bool,
}

/// Bounded maintenance and shutdown operations routed through the single RT owner.
#[allow(clippy::large_enum_variant)] // inline FreeTorqueBaseline keeps RT baseline store zero-alloc
pub enum MaintenanceOp {
    RequestShutdown,
    CommitShutdown,
    ShutdownReady,
    PlanningSnapshot,
    Observation,
    ClearFault { expected_epoch: Option<u64> },
    LeaseClearFault {
        connection: super::requests::lease::ConnectionHandle,
        lease_generation: u64,
        supplied_epoch: Option<u64>,
        request_id_valid: bool,
        body_error: Option<&'static str>,
        auth_policy: super::requests::lease::OwnerAuthPolicy,
        operator: crate::auth::OperatorGrant,
    },
    LeaseTorquePreflight {
        connection: super::requests::lease::ConnectionHandle,
        lease_generation: u64,
        supplied_epoch: Option<u64>,
        request_id_valid: bool,
        body_error: Option<&'static str>,
        auth_policy: super::requests::lease::OwnerAuthPolicy,
        operator: crate::auth::OperatorGrant,
    },
    SetJointZeroOffsets {
        offsets: Vec<f32>,
    },
    SetMotorZero { joint: usize },
    TakeBaselinePending {
        fresh_pending: Vec<crate::gripper_baseline::MoveRecord>,
        fresh_samples: Vec<crate::gripper_baseline::MoveSamples>,
        clear_dirty: bool,
    },
    StoreBaseline {
        baseline: crate::gripper_baseline::FreeTorqueBaseline,
        expected_generation: u64,
        mark_dirty: bool,
    },
    MarkBaselineDirty,
    Config(Box<super::requests::config::ConfigExchange>),
    #[doc(hidden)]
    ProbeOwnedPayload { payload: Box<dyn Any + Send> },
}

/// Reply payload of one [`MaintenanceOp`].
#[allow(clippy::large_enum_variant)] // bounded stack refusal keeps RT ClearFault and baseline zero-alloc
pub enum MaintenanceResult {
    ShutdownRequested(bool),
    ShutdownCommitted(bool),
    ShutdownReady(bool),
    PlanningSnapshot(PlanningSnapshot),
    Observation(Box<ArmObservation>),
    FaultCleared(Result<SafetyState, ClearFaultError>),
    LeaseFaultCleared(super::requests::lease::LeaseClearFaultOutcome),
    LeaseTorquePreflight(Result<u64, super::requests::lease::TorquePreflightError>),
    JointZeroOffsetsSet(Result<(), &'static str>),
    MotorZeroSet(Result<(), &'static str>),
    BaselineTaken(BaselineTakeReply),
    BaselineStored(bool),
    BaselineDirtyMarked,
    Config(Box<super::requests::config::ConfigExchange>),
    StoppedWithConfig(Box<super::requests::config::ConfigExchange>),
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
    returned_pending: Option<Vec<crate::gripper_baseline::MoveRecord>>,
    returned_samples: Option<Vec<crate::gripper_baseline::MoveSamples>>,
    returned_baseline: Option<crate::gripper_baseline::FreeTorqueBaseline>,
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
    hub: Arc<StateHub>,
}

impl MaintClientInner {
    fn pop_all_replies(&mut self) {
        while let Ok(reply) = self.rep_rx.pop() {
            self.in_flight = self.in_flight.saturating_sub(1);
            // Explicitly drop returned payloads here on the off-RT caller/dispatcher thread.
            drop(reply.returned_offsets);
            drop(reply.returned_probe);
            drop(reply.returned_driver_config_error);
            drop(reply.returned_pending);
            drop(reply.returned_samples);
            drop(reply.returned_baseline);
            if let Some(waiter) = self.waiters.remove(&reply.id) {
                let res = match reply.result {
                    MaintenanceResult::Stopped | MaintenanceResult::StoppedWithConfig(_) => {
                        Err(OwnerError::Stopped)
                    }
                    MaintenanceResult::Observation(mut obs) => {
                        self.hub.attach_policy_diag(&mut obs);
                        Ok(MaintenanceResult::Observation(obs))
                    }
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
    fn new(
        req_tx: Producer<MaintRequest>,
        rep_rx: Consumer<MaintReply>,
        exited: Arc<AtomicBool>,
        hub: Arc<StateHub>,
    ) -> Self {
        Self {
            inner: Mutex::new(MaintClientInner {
                req_tx,
                rep_rx,
                in_flight: 0,
                next_id: 1,
                closed: false,
                waiters: HashMap::new(),
                exited,
                hub,
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

    async fn call_try_async(
        &self,
        op: MaintenanceOp,
    ) -> Result<MaintenanceResult, (MaintenanceOp, OwnerError)> {
        let (tx, mut rx) = tokio::sync::oneshot::channel();
        {
            let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
            g.drain_replies();
            if g.closed || g.exited.load(Ordering::Acquire) || g.req_tx.is_abandoned() {
                return Err((op, OwnerError::Stopped));
            }
            if g.in_flight >= MAINT_QUEUE_CAPACITY || g.req_tx.is_full() {
                return Err((op, OwnerError::QueueFull));
            }
            let id = g.next_id;
            g.next_id += 1;
            if let Err(rtrb::PushError::Full(req)) = g.req_tx.push(MaintRequest { id, op }) {
                return Err((req.op, OwnerError::QueueFull));
            }
            g.in_flight += 1;
            g.waiters.insert(id, MaintWaiter::Async(tx));
        }
        loop {
            match tokio::time::timeout(Duration::from_millis(2), &mut rx).await {
                Ok(Ok(Ok(res))) => return Ok(res),
                Ok(Ok(Err(e))) => return Err((MaintenanceOp::ShutdownReady, e)),
                Ok(Err(_)) => return Err((MaintenanceOp::ShutdownReady, OwnerError::Stopped)),
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

pub(crate) enum EstopWait {
    Resolved(EstopResult),
    Pending(tokio::sync::oneshot::Receiver<EstopResult>),
}

/// Bounded capacity of the off-thread E-stop waiter registry and recent-ACK replay window
/// (keeps off-reader registration bounded with zero heap growth past capacity; the reader
/// path itself is lock-free and never touches [`EstopRegistry`]).
pub(crate) const ESTOP_REGISTRY_CAPACITY: usize = 256;

/// Off-thread lock-serialized E-stop waiter registry (never touched by `armd-control` or by
/// the WS reader task).
///
/// The reader only executes lock-free atomics (`e_stop_latched` + `requested_estop_count`),
/// and the spawned off-reader waiter calls [`Self::register_or_resolve`]. Waiters are kept
/// sorted by ascending `req` (O(1) fast path when in-order, bounded insert when out-of-order),
/// and a bounded `recent_acks` window preserves exact pinned `(SafetyState, epoch)` results
/// when an ACK is drained before the off-reader task registers (even across `clear_fault`).
pub(crate) struct EstopRegistry {
    waiters: VecDeque<(u64, tokio::sync::oneshot::Sender<EstopResult>)>,
    recent_acks: VecDeque<EstopAck>,
    evicted_ack_gen: u64,
    max_overflow_gen: u64,
    ack_rx: Option<Consumer<EstopAck>>,
    overflow_gen: Option<Arc<AtomicU64>>,
    exited: Option<Arc<AtomicBool>>,
}

impl Default for EstopRegistry {
    fn default() -> Self {
        Self {
            waiters: VecDeque::with_capacity(ESTOP_REGISTRY_CAPACITY),
            recent_acks: VecDeque::with_capacity(ESTOP_REGISTRY_CAPACITY),
            evicted_ack_gen: 0,
            max_overflow_gen: 0,
            ack_rx: None,
            overflow_gen: None,
            exited: None,
        }
    }
}

impl EstopRegistry {
    fn check_resolved(&self, req: u64) -> Option<EstopResult> {
        for ack in self.recent_acks.iter().rev() {
            if req >= ack.first_gen && req <= ack.last_gen {
                return Some(Ok(((*ack.state).clone(), ack.epoch)));
            }
        }
        if req <= self.max_overflow_gen || req <= self.evicted_ack_gen {
            return Some(Err(ArmError::Rejected {
                status: 503,
                code: "ESTOP_ACK_OVERFLOW".into(),
                message: "E-stop acknowledgement ring overflowed; epoch cannot be pinned".into(),
            }));
        }
        None
    }

    pub(crate) fn register_or_resolve(&mut self, req: u64) -> EstopWait {
        self.pop_acks_and_overflow();
        if let Some(res) = self.check_resolved(req) {
            return EstopWait::Resolved(res);
        }
        if self.exited.as_ref().is_some_and(|e| e.load(Ordering::Acquire)) {
            self.pop_acks_and_overflow();
            if let Some(res) = self.check_resolved(req) {
                return EstopWait::Resolved(res);
            }
            return EstopWait::Resolved(Err(ArmError::Rejected {
                status: 503,
                code: "EXECUTOR_STOPPED".into(),
                message: "RT executor stopped before acknowledging E-stop".into(),
            }));
        }
        if self.waiters.len() >= ESTOP_REGISTRY_CAPACITY {
            // Off-reader pruning of abandoned waiters before rejecting.
            self.waiters.retain(|(_, s)| !s.is_closed());
        }
        if self.waiters.len() >= ESTOP_REGISTRY_CAPACITY {
            return EstopWait::Resolved(Err(ArmError::Rejected {
                status: 503,
                code: "ESTOP_QUEUE_FULL".into(),
                message: "E-stop waiter registry is full; physical E-stop is latched".into(),
            }));
        }
        let (tx, rx) = tokio::sync::oneshot::channel();
        if self.waiters.back().is_none_or(|&(last, _)| req >= last) {
            self.waiters.push_back((req, tx));
        } else {
            let pos = self.waiters.partition_point(|&(g, _)| g <= req);
            self.waiters.insert(pos, (req, tx));
        }
        EstopWait::Pending(rx)
    }

    #[cfg(test)]
    pub(crate) fn register(&mut self, req: u64) -> tokio::sync::oneshot::Receiver<EstopResult> {
        match self.register_or_resolve(req) {
            EstopWait::Pending(rx) => rx,
            EstopWait::Resolved(res) => {
                let (tx, rx) = tokio::sync::oneshot::channel();
                let _ = tx.send(res);
                rx
            }
        }
    }

    pub(crate) fn record_ack(&mut self, ack: EstopAck) {
        if self.recent_acks.len() >= ESTOP_REGISTRY_CAPACITY {
            if let Some(evicted) = self.recent_acks.pop_front() {
                self.evicted_ack_gen = self.evicted_ack_gen.max(evicted.last_gen);
            }
        }
        self.recent_acks.push_back(ack.clone());
        // Fast path (monotonic FIFO): pop every front waiter covered by this ACK in O(1) each.
        while self
            .waiters
            .front()
            .is_some_and(|&(g, _)| g >= ack.first_gen && g <= ack.last_gen)
        {
            if let Some((_, tx)) = self.waiters.pop_front() {
                let _ = tx.send(Ok(((*ack.state).clone(), ack.epoch)));
            }
        }
        // Bounded fallback (<= ESTOP_REGISTRY_CAPACITY, zero heap allocation) if an earlier
        // waiter generation precedes `ack.first_gen`.
        if self.waiters.front().is_some_and(|&(g, _)| g < ack.first_gen) {
            let mut i = 0;
            while i < self.waiters.len() {
                let g = self.waiters[i].0;
                if g > ack.last_gen {
                    break;
                }
                if g >= ack.first_gen {
                    if let Some((_, tx)) = self.waiters.remove(i) {
                        let _ = tx.send(Ok(((*ack.state).clone(), ack.epoch)));
                    }
                } else {
                    i += 1;
                }
            }
        }
    }

    pub(crate) fn record_overflow(&mut self, max_dropped: u64) {
        if max_dropped == 0 {
            return;
        }
        self.max_overflow_gen = self.max_overflow_gen.max(max_dropped);
        while self.waiters.front().is_some_and(|&(g, _)| g <= max_dropped) {
            if let Some((_, tx)) = self.waiters.pop_front() {
                let _ = tx.send(Err(ArmError::Rejected {
                    status: 503,
                    code: "ESTOP_ACK_OVERFLOW".into(),
                    message: "E-stop acknowledgement ring overflowed; epoch cannot be pinned".into(),
                }));
            }
        }
    }

    pub(crate) fn cancel_waiter(&mut self, req: u64, status: u16, code: &'static str, message: &'static str) {
        let mut i = 0;
        while i < self.waiters.len() {
            let g = self.waiters[i].0;
            if g > req {
                break;
            }
            if g == req {
                if let Some((_, tx)) = self.waiters.remove(i) {
                    let _ = tx.send(Err(ArmError::Rejected {
                        status,
                        code: code.into(),
                        message: message.into(),
                    }));
                }
            } else {
                i += 1;
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
        loop {
            let next = self.ack_rx.as_mut().and_then(|rx| rx.pop().ok());
            match next {
                Some(ack) => self.record_ack(ack),
                None => break,
            }
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
            while let Some((_, tx)) = self.waiters.pop_front() {
                let _ = tx.send(Err(ArmError::Rejected {
                    status: 503,
                    code: "EXECUTOR_STOPPED".into(),
                    message: "RT executor stopped before acknowledging E-stop".into(),
                }));
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
    /// Sleeps until `target_ns` (or until `stop` is set). Returns `false` without sleeping if
    /// `now_ns() >= target_ns` on entry, or `true` if the thread genuinely slept/parked.
    fn sleep_until_ns(&self, target_ns: u64, stop: &AtomicBool) -> bool;
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
    /// Simulates `armd-control` thread spawn failure before ownership is sent to the thread.
    pub fail_control_spawn: bool,
    /// Injects a one-shot sleep inside the next release body (tests overrun + `skipped_releases`).
    pub release_delay_once: Option<Arc<Mutex<Option<Duration>>>>,
    /// Injects a one-shot sleep strictly AFTER StateHub publication, `tick_us` end mark,
    /// `UsageMark::now()`, and the first dispatcher wake (tests full `release_us` tail coverage).
    pub post_publish_delay_once: Option<Arc<Mutex<Option<Duration>>>>,
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
    /// Overrides the bounded reconciliation retry interval for `armd-persist` dir-fsync recovery.
    pub reconcile_retry_interval: Option<Duration>,
    /// Overrides the maximum reconciliation retry attempts for `armd-persist` dir-fsync recovery.
    pub reconcile_max_attempts: Option<u32>,
}

enum PersistCmd {
    Reserve {
        action_id: u64,
        base_revision: u64,
        lease_owner: Option<(crate::model::HolderLabel, u64)>,
        estop_epoch: u64,
        stop_gen: u64,
        deadline: Instant,
        new_policy: GripperPolicy,
        jaw_opening: Option<JawOpening>,
        probe: Option<Box<dyn Any + Send>>,
        code_buf: String,
        msg_buf: String,
    },
    VerifyAuthority {
        action_id: u64,
        reservation: Reservation,
        proposal: ConfigProposal,
        auth: super::requests::config::ConfigAuthContext,
        base_revision: u64,
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
    Persisted {
        action_id: u64,
        reservation: Reservation,
        code_buf: String,
        msg_buf: String,
    },
    DirSyncFailed {
        action_id: u64,
        reservation: Reservation,
        warning: String,
        unused_policy: Option<GripperPolicy>,
        unused_jaw: Option<JawOpening>,
        msg_buf: String,
    },
    Activate {
        action_id: u64,
        reservation: Reservation,
        proposal: ConfigProposal,
        auth: super::requests::config::ConfigAuthContext,
        base_revision: u64,
        offset_rad: f32,
        new_policy: GripperPolicy,
        new_jaw: Option<JawOpening>,
        operator_revoked: bool,
        code_buf: String,
        msg_buf: String,
    },
    FinishReconcile {
        proposal: ConfigProposal,
        auth: super::requests::config::ConfigAuthContext,
        base_revision: u64,
        offset_rad: f32,
        new_policy: GripperPolicy,
        new_jaw: Option<JawOpening>,
        operator_revoked: bool,
    },
    LatchConfigReconcileFault {
        message: crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>,
    },
}

// Zero-allocation RT reply enum; boxing `Reserved` would heap-allocate on `armd-control`.
#[allow(clippy::large_enum_variant)]
enum PersistReply {
    Reserved {
        reservation: Option<(
            Reservation,
            ConfigProposal,
            super::requests::config::ConfigAuthContext,
        )>,
        policy: Option<GripperPolicy>,
        returned_jaw: Option<JawOpening>,
        _returned_code: Option<String>,
        _returned_msg: Option<String>,
        _returned_probe: Option<Box<dyn Any + Send>>,
    },
    Verified {
        reservation: Option<Reservation>,
        _returned_code: Option<String>,
        _returned_msg: Option<String>,
        _returned_probe: Option<Box<dyn Any + Send>>,
    },
    Persisted {
        reservation: Option<Reservation>,
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
    Activated {
        activated_revision: u64,
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
    ZeroCheckCtx(Box<crate::supervisor::ZeroCheckPersistContext>),
    GravityCompRun(crate::gravity_comp::GravityCompRun),
    Probe(Box<dyn Any + Send>),
    SafetyState(Arc<SafetyState>),
    RetiredStop(RetiredStopPayloads),
    ActiveTrajectory(crate::motion::ActiveTrajectory),
    MacroRun(crate::macro_action::MacroRun),
    #[cfg(feature = "policy-runtime")]
    PolicyRun(crate::policy_runtime::PolicyRun),
    Buf(String),
    LogLegacyJ6(crate::joint_zero::ZeroConvention),
}

/// Off-thread broker dispatcher coordinating `Broker<PlanSlot>` submissions, connection-liveness
/// linearization against [`ConnLifecycleClient`], and reply/terminal reclamation (even after
/// caller disconnect or cancellation). Never touched by `armd-control`.
///
/// Strict acyclic lock order: `BrokerDispatcher::state` -> `ConnLifecycleClient::inner` -> `broker` -> `reply_router`.
pub(crate) struct BrokerDispatcher {
    broker: Arc<Mutex<Broker<PlanSlot>>>,
    conn_lifecycle: Arc<super::requests::lease::ConnLifecycleClient>,
    reply_router: Arc<Mutex<ActionReplyRouter>>,
    state: Mutex<BrokerDispatcherState>,
}

struct ManagedEntry {
    client: super::broker::ClientId,
    waiter: ManagedWaiter,
}

enum ManagedWaiter {
    OneShot(Option<tokio::sync::oneshot::Sender<Result<super::broker::Reply, OwnerError>>>),
    Action {
        ack: Option<tokio::sync::oneshot::Sender<Result<super::broker::Reply, OwnerError>>>,
        terminal: Option<tokio::sync::oneshot::Sender<Result<super::broker::Reply, OwnerError>>>,
        acked: bool,
    },
}

struct BrokerDispatcherState {
    managed: HashMap<super::broker::RequestId, ManagedEntry>,
    /// True after a client with in-flight managed requests disconnects so `drain_replies`
    /// continues polling `broker` until every abandoned reservation/plan slot is reclaimed.
    draining_disconnected: bool,
    exited: Arc<AtomicBool>,
}

/// Submission error returned by [`BrokerDispatcher`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BrokerDispatchError {
    Admit(super::broker::AdmitError),
    Stopped,
}

impl BrokerDispatcher {
    fn new(
        broker: Arc<Mutex<Broker<PlanSlot>>>,
        conn_lifecycle: Arc<super::requests::lease::ConnLifecycleClient>,
        reply_router: Arc<Mutex<ActionReplyRouter>>,
        exited: Arc<AtomicBool>,
    ) -> Self {
        Self {
            broker,
            conn_lifecycle,
            reply_router,
            state: Mutex::new(BrokerDispatcherState {
                managed: HashMap::with_capacity(super::broker::REPLY_CAPACITY),
                draining_disconnected: false,
                exited,
            }),
        }
    }

    fn submit_one_shot(
        &self,
        client: super::broker::ClientId,
        client_seq: u64,
        authority: super::broker::Authority,
        deadline: Instant,
        what: super::broker::Submit<'_, PlanSlot>,
    ) -> Result<tokio::sync::oneshot::Receiver<Result<super::broker::Reply, OwnerError>>, BrokerDispatchError>
    {
        let mut st = self.state.lock().unwrap_or_else(|e| e.into_inner());
        if st.exited.load(Ordering::Acquire) {
            return Err(BrokerDispatchError::Stopped);
        }
        if client.0 == 0 || !self.conn_lifecycle.is_slot_open(client.0) {
            return Err(BrokerDispatchError::Admit(
                super::broker::AdmitError::Disconnected,
            ));
        }
        let req = self
            .broker
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .submit(client, client_seq, authority, deadline, what)
            .map_err(BrokerDispatchError::Admit)?;
        let (tx, rx) = tokio::sync::oneshot::channel();
        st.managed.insert(
            req,
            ManagedEntry {
                client,
                waiter: ManagedWaiter::OneShot(Some(tx)),
            },
        );
        Ok(rx)
    }

    fn submit_action(
        &self,
        client: super::broker::ClientId,
        client_seq: u64,
        authority: super::broker::Authority,
        deadline: Instant,
        what: super::broker::Submit<'_, PlanSlot>,
    ) -> Result<
        (
            tokio::sync::oneshot::Receiver<Result<super::broker::Reply, OwnerError>>,
            tokio::sync::oneshot::Receiver<Result<super::broker::Reply, OwnerError>>,
        ),
        BrokerDispatchError,
    > {
        let mut st = self.state.lock().unwrap_or_else(|e| e.into_inner());
        if st.exited.load(Ordering::Acquire) {
            return Err(BrokerDispatchError::Stopped);
        }
        if client.0 == 0 || !self.conn_lifecycle.is_slot_open(client.0) {
            return Err(BrokerDispatchError::Admit(
                super::broker::AdmitError::Disconnected,
            ));
        }
        let req = self
            .broker
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .submit(client, client_seq, authority, deadline, what)
            .map_err(BrokerDispatchError::Admit)?;
        let (ack_tx, ack_rx) = tokio::sync::oneshot::channel();
        let (term_tx, term_rx) = tokio::sync::oneshot::channel();
        st.managed.insert(
            req,
            ManagedEntry {
                client,
                waiter: ManagedWaiter::Action {
                    ack: Some(ack_tx),
                    terminal: Some(term_tx),
                    acked: false,
                },
            },
        );
        Ok((ack_rx, term_rx))
    }

    #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
    fn submit_policy_command(
        &self,
        client: super::broker::ClientId,
        client_seq: u64,
        authority: super::broker::Authority,
        deadline: Instant,
        cmd: super::requests::policy::PolicyCommand,
        on_admitted: impl FnOnce(super::broker::RequestId),
    ) -> Result<
        (
            tokio::sync::oneshot::Receiver<Result<super::broker::Reply, OwnerError>>,
            Option<tokio::sync::oneshot::Receiver<Result<super::broker::Reply, OwnerError>>>,
        ),
        BrokerDispatchError,
    > {
        let mut st = self.state.lock().unwrap_or_else(|e| e.into_inner());
        if st.exited.load(Ordering::Acquire) {
            return Err(BrokerDispatchError::Stopped);
        }
        if client.0 == 0 || !self.conn_lifecycle.is_slot_open(client.0) {
            return Err(BrokerDispatchError::Admit(
                super::broker::AdmitError::Disconnected,
            ));
        }
        let starts_action = cmd.starts_action();
        let req = self
            .broker
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .submit_with_pre_push(
                client,
                client_seq,
                authority,
                deadline,
                super::broker::Submit::Policy(cmd),
                on_admitted,
            )
            .map_err(BrokerDispatchError::Admit)?;
        let (ack_tx, ack_rx) = tokio::sync::oneshot::channel();
        if starts_action {
            let (term_tx, term_rx) = tokio::sync::oneshot::channel();
            st.managed.insert(
                req,
                ManagedEntry {
                    client,
                    waiter: ManagedWaiter::Action {
                        ack: Some(ack_tx),
                        terminal: Some(term_tx),
                        acked: false,
                    },
                },
            );
            Ok((ack_rx, Some(term_rx)))
        } else {
            st.managed.insert(
                req,
                ManagedEntry {
                    client,
                    waiter: ManagedWaiter::OneShot(Some(ack_tx)),
                },
            );
            Ok((ack_rx, None))
        }
    }

    /// Linearizes connection teardown off-thread: prevents any subsequent `BrokerDispatcher`
    /// submission for `connection.id()`, marks the broker client disconnected (retiring its
    /// managed waiters immediately while continuing broker reclamation), and pushes a
    /// guaranteed-non-full priority `LifecycleCmd::Disconnect` onto the lifecycle channel.
    async fn disconnect_connection(
        &self,
        connection: super::requests::lease::ConnectionHandle,
    ) -> Result<super::requests::lease::LeaseOutcome, super::requests::lease::ConnLifecycleError>
    {
        let rx = {
            let mut st = self.state.lock().unwrap_or_else(|e| e.into_inner());
            let (tx, rx) = tokio::sync::oneshot::channel();
            // Strict acyclic lock order: `state` (1) -> `conn_lifecycle` (2) -> `broker` (3).
            self.conn_lifecycle.enqueue_disconnect_oneshot(connection, tx)?;
            Self::disconnect_broker_client_locked(&mut st, &self.broker, connection.id());
            rx
        };
        match rx.await {
            Ok(Ok(super::requests::lease::LifecycleReply::Disconnected { outcome, .. })) => {
                Ok(outcome)
            }
            Ok(Err(e)) => Err(e),
            _ => Err(super::requests::lease::ConnLifecycleError::Stopped),
        }
    }

    fn disconnect_connection_sync(
        &self,
        connection: super::requests::lease::ConnectionHandle,
    ) -> Result<super::requests::lease::LeaseOutcome, super::requests::lease::ConnLifecycleError>
    {
        let rx = {
            let mut st = self.state.lock().unwrap_or_else(|e| e.into_inner());
            let (tx, rx) = mpsc::sync_channel(1);
            self.conn_lifecycle.enqueue_disconnect_sync(connection, tx)?;
            Self::disconnect_broker_client_locked(&mut st, &self.broker, connection.id());
            rx
        };
        match rx.recv() {
            Ok(Ok(super::requests::lease::LifecycleReply::Disconnected { outcome, .. })) => {
                Ok(outcome)
            }
            Ok(Err(e)) => Err(e),
            _ => Err(super::requests::lease::ConnLifecycleError::Stopped),
        }
    }

    fn enqueue_disconnect_detached(
        &self,
        connection: super::requests::lease::ConnectionHandle,
    ) {
        let mut st = self.state.lock().unwrap_or_else(|e| e.into_inner());
        self.conn_lifecycle.enqueue_disconnect_detached(connection);
        Self::disconnect_broker_client_locked(&mut st, &self.broker, connection.id());
    }

    fn disconnect_broker_client_locked(
        st: &mut BrokerDispatcherState,
        broker: &Mutex<Broker<PlanSlot>>,
        conn_id: u64,
    ) {
        if conn_id == 0 {
            return;
        }
        let client = super::broker::ClientId(conn_id);
        let before = st.managed.len();
        st.managed.retain(|_, entry| entry.client != client);
        let mut guard = broker.lock().unwrap_or_else(|e| e.into_inner());
        guard.disconnect(client);
        if st.managed.len() < before || guard.outstanding() > 0 {
            st.draining_disconnected = true;
        }
    }

    fn poll_all_locked(
        st: &mut BrokerDispatcherState,
        broker: &Mutex<Broker<PlanSlot>>,
        reply_router: &Mutex<ActionReplyRouter>,
        force_poll: bool,
    ) -> (usize, Option<Arc<dyn ExternalReplySink>>) {
        use super::broker::DeliveryDecision;
        let mut guard = broker.lock().unwrap_or_else(|e| e.into_inner());
        let mut router = reply_router.lock().unwrap_or_else(|e| e.into_inner());
        let has_consumers = force_poll
            || !st.managed.is_empty()
            || st.draining_disconnected
            || !router.waiters.is_empty()
            || router.external_sink.is_some();
        if !has_consumers {
            return (0, None);
        }
        let n = guard.try_poll(|client, reply| {
            let req = reply.request();
            if let Some(entry) = st.managed.get_mut(&req) {
                let delivered = match &mut entry.waiter {
                    ManagedWaiter::OneShot(slot) => {
                        let ok = slot.take().is_some_and(|tx| tx.send(Ok(reply)).is_ok());
                        st.managed.remove(&req);
                        ok
                    }
                    ManagedWaiter::Action { ack, terminal, acked } => match reply {
                        super::broker::Reply::Ack { .. } => {
                            *acked = true;
                            ack.take().is_some_and(|tx| tx.send(Ok(reply)).is_ok())
                        }
                        super::broker::Reply::Refused { .. }
                        | super::broker::Reply::Lease { .. } => {
                            let ok = ack.take().is_some_and(|tx| tx.send(Ok(reply)).is_ok());
                            st.managed.remove(&req);
                            ok
                        }
                        super::broker::Reply::Terminal { .. } => {
                            let ok =
                                terminal.take().is_some_and(|tx| tx.send(Ok(reply)).is_ok());
                            st.managed.remove(&req);
                            ok
                        }
                    },
                };
                if delivered {
                    DeliveryDecision::Delivered
                } else {
                    DeliveryDecision::Abandoned
                }
            } else if let Some(tx) = router.waiters.remove(&req) {
                let _ = tx.send(reply);
                DeliveryDecision::Delivered
            } else if router.external.len() < router.external_cap {
                router.external.push_back((client, reply));
                DeliveryDecision::Delivered
            } else {
                DeliveryDecision::Backpressured
            }
        });
        guard.reclaim_plans();
        if guard.outstanding() == 0 && st.managed.is_empty() {
            st.draining_disconnected = false;
        }
        (n, router.external_sink.clone())
    }

    fn pump_to_router(&self) -> (usize, Option<Arc<dyn ExternalReplySink>>) {
        let mut st = self.state.lock().unwrap_or_else(|e| e.into_inner());
        Self::poll_all_locked(&mut st, &self.broker, &self.reply_router, true)
    }

    fn drain_replies(&self) {
        let sink = {
            let mut st = self.state.lock().unwrap_or_else(|e| e.into_inner());
            let (_, sink) = Self::poll_all_locked(&mut st, &self.broker, &self.reply_router, false);
            if st.exited.load(Ordering::Acquire) {
                let _ = Self::poll_all_locked(&mut st, &self.broker, &self.reply_router, false);
                Self::fail_remaining(&mut st.managed, OwnerError::Stopped);
                st.draining_disconnected = false;
            }
            sink
        };
        if let Some(sink) = sink {
            use super::broker::DeliveryDecision;
            let mut delivered_any = false;
            loop {
                let next = self
                    .reply_router
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .external
                    .pop_front();
                let Some((client, reply)) = next else {
                    break;
                };
                match sink.deliver(client, reply) {
                    DeliveryDecision::Delivered | DeliveryDecision::Abandoned => {
                        delivered_any = true;
                    }
                    DeliveryDecision::Backpressured => {
                        self.reply_router
                            .lock()
                            .unwrap_or_else(|e| e.into_inner())
                            .external
                            .push_front((client, reply));
                        break;
                    }
                }
            }
            if delivered_any {
                let mut st = self.state.lock().unwrap_or_else(|e| e.into_inner());
                let _ = Self::poll_all_locked(&mut st, &self.broker, &self.reply_router, false);
            }
        }
    }

    fn finish_stopped(&self) {
        let mut st = self.state.lock().unwrap_or_else(|e| e.into_inner());
        let _ = Self::poll_all_locked(&mut st, &self.broker, &self.reply_router, false);
        Self::fail_remaining(&mut st.managed, OwnerError::Stopped);
        st.draining_disconnected = false;
    }

    fn fail_remaining(
        managed: &mut HashMap<super::broker::RequestId, ManagedEntry>,
        err: OwnerError,
    ) {
        for (_, entry) in managed.drain() {
            match entry.waiter {
                ManagedWaiter::OneShot(Some(tx)) => {
                    let _ = tx.send(Err(err.clone()));
                }
                ManagedWaiter::OneShot(None) => {}
                ManagedWaiter::Action { ack, terminal, .. } => {
                    if let Some(tx) = ack {
                        let _ = tx.send(Err(err.clone()));
                    }
                    if let Some(tx) = terminal {
                        let _ = tx.send(Err(err.clone()));
                    }
                }
            }
        }
    }
}

/// Off-thread consumer sinks drained by `armd-dispatch`, `tick_stats()`, and `shutdown_and_join()`.
pub(crate) struct OffThreadSinks {
    /// Pair of preallocated SPSC diagnostic consumers (`diag_rx`, `diag_amend_rx`) under one
    /// mutex so `drain_diag` is the sole off-thread writer to `LockFreeTickStats`.
    diag_rx: Mutex<(Consumer<TickSample>, Consumer<(u64, u32)>)>,
    estop_reg: Arc<Mutex<EstopRegistry>>,
    reclaim_state_rx: Mutex<Consumer<Arc<SafetyState>>>,
    reclaim_stop_rx: Mutex<Consumer<RetiredStopPayloads>>,
    reclaim_traj_rx: Mutex<Consumer<crate::motion::ActiveTrajectory>>,
    reclaim_macro_rx: Mutex<Consumer<crate::macro_action::MacroRun>>,
    #[cfg(feature = "policy-runtime")]
    reclaim_policy_rx: Mutex<Consumer<crate::policy_runtime::PolicyRun>>,
    reclaim_return_tx: Mutex<Producer<Arc<SafetyState>>>,
    /// `(consumer, last_reported_dropped, shutdown_reported_total)` guarded by one mutex so
    /// best-effort warning deltas never race and never reset `stats.safety_log_dropped`.
    safety_log_rx: Mutex<(Consumer<SafetyLogEvent>, u64, u64)>,
    /// Final pending `"Stopped hold output failed"` aggregate handed off ONLY during control-thread
    /// teardown when `safety_log_tx` remained full (never touched on RT ticks).
    shutdown_hold_error: Arc<Mutex<Option<SafetyLogEvent>>>,
    stats: Arc<ExecutorStats>,
    persist_rep_rx: Arc<Mutex<Consumer<PersistReply>>>,
    persist_worker_exited: Arc<AtomicBool>,
    maint_client: Arc<MaintClient>,
    urgent_stop_client: Arc<UrgentStopClient>,
    conn_lifecycle_client: Arc<super::requests::lease::ConnLifecycleClient>,
    broker_dispatcher: Arc<BrokerDispatcher>,
    eviction_rx: Mutex<Consumer<(super::requests::lease::ConnectionHandle, u64)>>,
    servo_close_rx: Mutex<Consumer<u64>>,
    atomic_fenced_lease_gen: Arc<AtomicU64>,
    servo_dispatcher: Arc<ServoTargetDispatcher>,
    hub: Arc<StateHub>,
    offload_reclaim_rx: Mutex<Consumer<OffloadReclaim>>,
}

impl OffThreadSinks {
    /// Authoritative monotonic count of dropped deferred safety log events (never reset).
    pub(crate) fn safety_log_dropped(&self) -> u64 {
        self.stats.safety_log_dropped.load(Ordering::Relaxed)
    }

    pub(crate) fn drain_diag(&self, tick_stats: &crate::diagnostics::tick::LockFreeTickStats) {
        let mut g = self.diag_rx.lock().unwrap_or_else(|e| e.into_inner());
        let (diag_rx, amend_rx) = &mut *g;
        let mut samples = Vec::new();
        let mut max_popped_seq: u64 = 0;
        while let Ok(sample) = diag_rx.pop() {
            max_popped_seq = max_popped_seq.max(sample.release_seq);
            samples.push(sample);
        }
        let mut amends = Vec::new();
        let mut max_amend_seq: u64 = 0;
        while let Ok((seq, us)) = amend_rx.pop() {
            max_amend_seq = max_amend_seq.max(seq);
            amends.push((seq, us));
        }
        // Control thread pushes `sample(seq)` to `diag_tx` strictly before pushing `(seq, us)`
        // to `diag_amend_tx` (and skips `diag_amend_tx` whenever `diag_tx` was full). Catch up
        // `diag_rx` if a release pushed both between the two pops so its amendment merges in place.
        while max_popped_seq < max_amend_seq {
            match diag_rx.pop() {
                Ok(sample) => {
                    max_popped_seq = max_popped_seq.max(sample.release_seq);
                    samples.push(sample);
                }
                Err(_) => break,
            }
        }
        let mut unmatched_amends = Vec::new();
        for (seq, us) in amends {
            if let Some(sample) = samples.iter_mut().find(|s| s.release_seq == seq) {
                if sample.release_us < us {
                    sample.release_us = us;
                }
            } else {
                unmatched_amends.push((seq, us));
            }
        }
        for sample in &samples {
            let _ = tick_stats.record(sample);
        }
        for (seq, us) in unmatched_amends {
            if !tick_stats.amend_release_us_by_release_seq(seq, us) {
                self.stats.amend_missed.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    pub(crate) fn drain_all(&self, tick_stats: &crate::diagnostics::tick::LockFreeTickStats) {
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
            let mut rx = self.reclaim_stop_rx.lock().unwrap_or_else(|e| e.into_inner());
            while let Ok(retired) = rx.pop() {
                drop(retired);
            }
        }
        {
            let mut rx = self.reclaim_traj_rx.lock().unwrap_or_else(|e| e.into_inner());
            while let Ok(traj) = rx.pop() {
                drop(traj);
            }
        }
        {
            let mut rx = self.reclaim_macro_rx.lock().unwrap_or_else(|e| e.into_inner());
            while let Ok(_run) = rx.pop() {}
        }
        #[cfg(feature = "policy-runtime")]
        {
            let mut rx = self.reclaim_policy_rx.lock().unwrap_or_else(|e| e.into_inner());
            while let Ok(run) = rx.pop() {
                drop(run);
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
            if let Some(ev) = self
                .shutdown_hold_error
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .take()
            {
                ev.emit();
            }
        }
        self.maint_client.drain_replies();
        self.urgent_stop_client.drain_replies();
        // Preserve owner Step 7 causal ordering off-thread: broker terminals (e.g. an aborted
        // TorqueOn fenced by Disconnect) resolve strictly BEFORE lifecycle waiters (Disconnect).
        self.broker_dispatcher.drain_replies();
        self.conn_lifecycle_client.drain_replies();
        {
            let mut ev_rx = self.eviction_rx.lock().unwrap_or_else(|e| e.into_inner());
            while let Ok((ev_conn, ev_gen)) = ev_rx.pop() {
                crate::api::ws::notify_evicted_connection(ev_conn.as_str(), ev_gen);
            }
        }
        // Drain RT servo target outcomes BEFORE applying `close_servo(old_gen)` so an already-
        // consumed frame from a displaced lease resolves with its exact RT refusal before closing.
        self.servo_dispatcher.drain_outcomes(&self.hub);
        {
            let mut close_rx = self.servo_close_rx.lock().unwrap_or_else(|e| e.into_inner());
            while let Ok(old_gen) = close_rx.pop() {
                self.servo_dispatcher.close_servo(old_gen);
            }
        }
        let fenced_gen = self.atomic_fenced_lease_gen.load(Ordering::Acquire);
        if fenced_gen > 0 {
            self.servo_dispatcher.sync_fenced_generation(fenced_gen);
        }
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
    pub(crate) fn flush_and_report_shutdown(&self, tick_stats: &crate::diagnostics::tick::LockFreeTickStats) {
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

    fn try_set_outcome_before(
        &self,
        outcome: crate::supervisor::PanicDisableOutcome,
        hard_deadline: Instant,
    ) -> bool {
        let mut g = self.outcome.lock().unwrap_or_else(|e| e.into_inner());
        if g.is_some() || Instant::now() >= hard_deadline {
            return false;
        }
        if outcome == crate::supervisor::PanicDisableOutcome::TimedOut {
            self.timed_out.store(true, Ordering::Release);
        }
        *g = Some(outcome);
        self.done.notify_all();
        true
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

/// Single plug-in hook for PR L's per-connection WebSocket reply router (A6/A7).
///
/// In PR P, WebSocket ordinary commands on `OWNER_EXECUTOR` remain gated at `503
/// EXECUTOR_ROUTING_PENDING` (`src/api/ws.rs:492, 2461`) because connection lifecycle and
/// per-client reply routing belong to PR L. PR L installs its per-connection router once via
/// [`RtExecutorHandle::attach_external_reply_sink`]. Until a sink is attached,
/// [`crate::macro_action::begin_macro_executor`] refuses new executor macros at admission with
/// `503 EXECUTOR_ROUTING_PENDING` so no macro can commit without a terminal reply sink.
pub trait ExternalReplySink: Send + Sync {
    fn deliver(
        &self,
        client: ClientId,
        reply: super::broker::Reply,
    ) -> super::broker::DeliveryDecision;
}

impl<F> ExternalReplySink for F
where
    F: Fn(ClientId, super::broker::Reply) -> super::broker::DeliveryDecision + Send + Sync,
{
    fn deliver(
        &self,
        client: ClientId,
        reply: super::broker::Reply,
    ) -> super::broker::DeliveryDecision {
        (self)(client, reply)
    }
}

/// Shared bounded FIFO broker reply dispatcher (A7 / Major 10):
/// routes replies for internal broker RPCs (`BeginMacro` ACK/Refused, `InstallChild`, `FinishMacro`)
/// directly to their keyed oneshot waiters without head-of-line blocking behind unrelated replies,
/// while buffering all other replies (including `BeginMacro` `Terminal`s and external client traffic)
/// in a bounded (`<= external_cap`) FIFO queue and forwarding them to the attached
/// [`ExternalReplySink`] (PR L's per-connection WS reply router hook).
///
/// Lock order: ALWAYS lock `RtExecutorShared::broker` BEFORE `RtExecutorShared::reply_router`.
/// NEVER invoke `ExternalReplySink::deliver` while holding either lock.
struct ActionReplyRouter {
    waiters: std::collections::HashMap<super::broker::RequestId, tokio::sync::oneshot::Sender<super::broker::Reply>>,
    external: VecDeque<(ClientId, super::broker::Reply)>,
    external_cap: usize,
    external_sink: Option<Arc<dyn ExternalReplySink>>,
}

impl ActionReplyRouter {
    fn new(external_cap: usize) -> Self {
        let cap = external_cap.max(1);
        Self {
            waiters: std::collections::HashMap::new(),
            external: VecDeque::with_capacity(cap),
            external_cap: cap,
            external_sink: None,
        }
    }
}

/// Outcome of a correlated WS `/api/v1/motion/servo` command routed through [`TargetPort::submit_servo`].
pub(crate) enum ServoCommandOutcome {
    Applied {
        result: Result<(), super::mailbox::Refusal>,
        post: Option<Arc<Snapshot>>,
    },
    /// Overwritten in the triple buffer before consumption by a newer target frame.
    Coalesced,
    /// Displaced/closed generation was overwritten before consumption.
    Closed,
    /// Executor stopped before consuming the frame.
    Stopped,
}

/// Immediate refusal from [`RtExecutorHandle::submit_servo_command`].
pub(crate) enum ServoCommandSubmitError {
    Immediate(serde_json::Value),
    Unavailable,
}

/// Drop-safe waiter for one correlated `/api/v1/motion/servo` submission.
pub(crate) struct ServoCommandWaiter {
    dispatcher: Arc<ServoTargetDispatcher>,
    key: (super::mailbox::StreamId, u64),
    rx: tokio::sync::oneshot::Receiver<ServoCommandOutcome>,
    completed: bool,
}

impl ServoCommandWaiter {
    pub(crate) async fn wait(mut self) -> ServoCommandOutcome {
        let out = match (&mut self.rx).await {
            Ok(outcome) => outcome,
            Err(_) => ServoCommandOutcome::Stopped,
        };
        self.completed = true;
        out
    }
}

impl Drop for ServoCommandWaiter {
    fn drop(&mut self) {
        if !self.completed {
            self.dispatcher.cancel_command_waiter(self.key);
        }
    }
}

type ServoStreamReplySink = Arc<dyn Fn(serde_json::Value) + Send + Sync>;

struct ServoStreamOwner {
    conn_id: u64,
    conn_name: String,
    lease_generation: u64,
    reply_sink: ServoStreamReplySink,
    pending_seqs: VecDeque<u64>,
}

const UNCLAIMED_TARGET_OUTCOMES_CAP: usize = 64;
const STREAM_PENDING_SEQS_CAP: usize = 64;

struct ServoTargetDispatcherInner {
    port: crate::supervisor::targets::TargetPort,
    last_inflight_key: Option<(super::mailbox::StreamId, u64)>,
    command_waiters: std::collections::HashMap<
        (super::mailbox::StreamId, u64),
        tokio::sync::oneshot::Sender<ServoCommandOutcome>,
    >,
    stream_owners: std::collections::HashMap<super::mailbox::StreamId, ServoStreamOwner>,
    unclaimed_outcomes: VecDeque<super::mailbox::TargetOutcome>,
    #[cfg(feature = "policy-runtime")]
    policy_outcomes: VecDeque<(u64, super::mailbox::TargetOutcome)>,
    #[cfg(feature = "policy-runtime")]
    tracked_policy_streams: VecDeque<u64>,
    submitted_count: u64,
    #[cfg(feature = "policy-runtime")]
    policy_submitted_count: u64,
    #[cfg(feature = "policy-runtime")]
    policy_busy_count: u64,
    lease_refused_by_gen: std::collections::HashMap<u64, u64>,
    closed: bool,
}

type DrainedServoDeliveries = (
    Vec<(tokio::sync::oneshot::Sender<ServoCommandOutcome>, ServoCommandOutcome)>,
    Vec<(ServoStreamReplySink, String, u64, super::mailbox::Refusal)>,
);

impl ServoTargetDispatcherInner {
    fn prune_closed_stream_owners_locked(&mut self) {
        let closed = self.port.closed_servo_generation();
        if closed == 0 {
            return;
        }
        self.stream_owners.retain(|_, owner| {
            !(owner.lease_generation != 0
                && owner.lease_generation <= closed
                && owner.pending_seqs.is_empty())
        });
    }

    fn pop_outcomes_locked(
        &mut self,
        hub: &StateHub,
        resolved_cmds: &mut Vec<(tokio::sync::oneshot::Sender<ServoCommandOutcome>, ServoCommandOutcome)>,
        stream_refusals: &mut Vec<(ServoStreamReplySink, String, u64, super::mailbox::Refusal)>,
    ) {
        let mut popped = Vec::new();
        self.port.outcomes(|o| popped.push(o));
        let closed_gen = self.port.closed_servo_generation();
        for o in popped {
            #[cfg(feature = "policy-runtime")]
            if (o.stream.0 & crate::supervisor::targets::SERVO_STREAM_BIT) == 0
                && o.stream.0 != 0
                && self.tracked_policy_streams.contains(&o.stream.0)
            {
                if self.policy_outcomes.len() >= UNCLAIMED_TARGET_OUTCOMES_CAP {
                    self.policy_outcomes.pop_front();
                }
                self.policy_outcomes.push_back((o.stream.0, o));
                continue;
            }
            if (o.stream.0 & crate::supervisor::targets::SERVO_STREAM_BIT) != 0 {
                if let Err(ref r) = o.result {
                    if matches!(r.code, "LEASE_LOST" | "NO_LEASE" | "STREAM_NOT_AUTHORIZED") {
                        let gen = o.stream.0 & !crate::supervisor::targets::SERVO_STREAM_BIT;
                        *self.lease_refused_by_gen.entry(gen).or_insert(0) += 1;
                    }
                }
            }
            if let Some(tx) = self.command_waiters.remove(&(o.stream, o.seq)) {
                resolved_cmds.push((
                    tx,
                    ServoCommandOutcome::Applied {
                        result: o.result,
                        post: hub.latest(),
                    },
                ));
                continue;
            }
            let mut claimed_by_stream = false;
            let mut drop_closed_owner = false;
            if let Some(owner) = self.stream_owners.get_mut(&o.stream) {
                if owner.pending_seqs.iter().any(|&s| s == o.seq) {
                    claimed_by_stream = true;
                    while let Some(&front) = owner.pending_seqs.front() {
                        owner.pending_seqs.pop_front();
                        if front == o.seq {
                            break;
                        }
                    }
                    if let Err(refusal) = o.result {
                        stream_refusals.push((
                            Arc::clone(&owner.reply_sink),
                            owner.conn_name.clone(),
                            owner.lease_generation,
                            refusal,
                        ));
                    }
                    if owner.lease_generation != 0
                        && owner.lease_generation <= closed_gen
                        && owner.pending_seqs.is_empty()
                    {
                        drop_closed_owner = true;
                    }
                }
            }
            if drop_closed_owner {
                self.stream_owners.remove(&o.stream);
            }
            if !claimed_by_stream {
                if self.unclaimed_outcomes.len() >= UNCLAIMED_TARGET_OUTCOMES_CAP {
                    self.unclaimed_outcomes.pop_front();
                }
                self.unclaimed_outcomes.push_back(o);
            }
        }
    }

    fn note_coalesced_locked(
        &mut self,
        prev_key: Option<(super::mailbox::StreamId, u64)>,
        resolved_cmds: &mut Vec<(tokio::sync::oneshot::Sender<ServoCommandOutcome>, ServoCommandOutcome)>,
    ) {
        let Some(old_key) = prev_key else {
            return;
        };
        if let Some(tx) = self.command_waiters.remove(&old_key) {
            let old_gen = old_key.0 .0 & !crate::supervisor::targets::SERVO_STREAM_BIT;
            let is_servo = (old_key.0 .0 & crate::supervisor::targets::SERVO_STREAM_BIT) != 0;
            let outcome = if is_servo && self.port.is_servo_closed(old_gen) {
                ServoCommandOutcome::Closed
            } else {
                ServoCommandOutcome::Coalesced
            };
            resolved_cmds.push((tx, outcome));
        }
        // Retire the overwritten streaming seq from its owner too so a closed stream whose
        // buffered frame was coalesced by a replacement generation cannot pin its owner forever.
        let closed_gen = self.port.closed_servo_generation();
        let mut drop_closed_owner = false;
        if let Some(owner) = self.stream_owners.get_mut(&old_key.0) {
            while let Some(&front) = owner.pending_seqs.front() {
                if front <= old_key.1 {
                    owner.pending_seqs.pop_front();
                } else {
                    break;
                }
            }
            if owner.lease_generation != 0
                && owner.lease_generation <= closed_gen
                && owner.pending_seqs.is_empty()
            {
                drop_closed_owner = true;
            }
        }
        if drop_closed_owner {
            self.stream_owners.remove(&old_key.0);
        }
    }
}

/// Off-thread coordinator owning [`TargetPort`], `close_servo` dispatch, streaming outcome
/// delivery, and `/api/v1/motion/servo` command reply correlation (never touched by `armd-control`).
pub(crate) struct ServoTargetDispatcher {
    inner: Mutex<ServoTargetDispatcherInner>,
    exited: Arc<AtomicBool>,
    atomic_fenced_lease_gen: Arc<AtomicU64>,
    #[cfg(feature = "policy-runtime")]
    atomic_closed_policy_action_id: Arc<AtomicU64>,
    #[cfg(feature = "policy-runtime")]
    policy_refusal_gen: Arc<AtomicU64>,
}

impl ServoTargetDispatcher {
    fn new(
        writer: TargetWriter,
        arm: ArmId,
        boot: BootId,
        exited: Arc<AtomicBool>,
        atomic_fenced_lease_gen: Arc<AtomicU64>,
        #[cfg(feature = "policy-runtime")]
        atomic_closed_policy_action_id: Arc<AtomicU64>,
    ) -> Self {
        #[cfg(feature = "policy-runtime")]
        let policy_refusal_gen = writer.policy_refusal_gen_arc();
        Self {
            inner: Mutex::new(ServoTargetDispatcherInner {
                port: crate::supervisor::targets::TargetPort::new(writer, arm, boot),
                last_inflight_key: None,
                command_waiters: std::collections::HashMap::new(),
                stream_owners: std::collections::HashMap::new(),
                unclaimed_outcomes: VecDeque::with_capacity(UNCLAIMED_TARGET_OUTCOMES_CAP),
                #[cfg(feature = "policy-runtime")]
                policy_outcomes: VecDeque::with_capacity(UNCLAIMED_TARGET_OUTCOMES_CAP),
                #[cfg(feature = "policy-runtime")]
                tracked_policy_streams: VecDeque::with_capacity(64),
                submitted_count: 0,
                #[cfg(feature = "policy-runtime")]
                policy_submitted_count: 0,
                #[cfg(feature = "policy-runtime")]
                policy_busy_count: 0,
                lease_refused_by_gen: std::collections::HashMap::new(),
                closed: false,
            }),
            exited,
            atomic_fenced_lease_gen,
            #[cfg(feature = "policy-runtime")]
            atomic_closed_policy_action_id,
            #[cfg(feature = "policy-runtime")]
            policy_refusal_gen,
        }
    }

    #[cfg(feature = "policy-runtime")]
    pub(crate) fn policy_refusal_gen(&self) -> u64 {
        self.policy_refusal_gen.load(Ordering::Acquire)
    }

    fn sync_authoritative_fence_locked(&self, g: &mut ServoTargetDispatcherInner) {
        let fenced_gen = self.atomic_fenced_lease_gen.load(Ordering::Acquire);
        if fenced_gen > g.port.closed_servo_generation() {
            g.port.close_servo(fenced_gen);
            g.prune_closed_stream_owners_locked();
        }
        #[cfg(feature = "policy-runtime")]
        {
            let closed_policy = self.atomic_closed_policy_action_id.load(Ordering::Acquire);
            if closed_policy > g.port.closed_policy_action_id() {
                g.port.close_policy(closed_policy);
            }
        }
    }

    fn deliver_drained(hub: &StateHub, (resolved_cmds, stream_refusals): DrainedServoDeliveries) {
        for (tx, outcome) in resolved_cmds {
            let _ = tx.send(outcome);
        }
        if stream_refusals.is_empty() {
            return;
        }
        let snap = hub.latest();
        for (sink, conn_name, lease_gen, refusal) in stream_refusals {
            let ctx = crate::supervisor::targets::ReplyCtx {
                epoch_id: snap.as_ref().map_or(0, |s| s.obs.epoch_id),
                safety_state: snap
                    .as_ref()
                    .map_or_else(|| serde_json::json!("Idle"), |s| serde_json::json!(s.obs.safety_state)),
                lease_generation: lease_gen,
            };
            if let Some(mut reply) = crate::supervisor::targets::servo_outcome_reply(&refusal, &ctx) {
                if reply.get("type").and_then(serde_json::Value::as_str) == Some("lease_lost")
                    && lease_gen != 0
                    && reply.get("reason").is_none()
                {
                    if let Some(ref s) = snap {
                        reply["reason"] =
                            serde_json::json!(s.owner.lease_loss_reason(&conn_name, lease_gen));
                    }
                }
                (sink)(reply);
            }
        }
    }

    pub(crate) fn drain_outcomes(&self, hub: &StateHub) {
        let mut resolved_cmds = Vec::new();
        let mut stream_refusals = Vec::new();
        {
            let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
            self.sync_authoritative_fence_locked(&mut g);
            g.pop_outcomes_locked(hub, &mut resolved_cmds, &mut stream_refusals);
            if self.exited.load(Ordering::Acquire) {
                g.pop_outcomes_locked(hub, &mut resolved_cmds, &mut stream_refusals);
                for (_, tx) in g.command_waiters.drain() {
                    resolved_cmds.push((tx, ServoCommandOutcome::Stopped));
                }
            }
        }
        Self::deliver_drained(hub, (resolved_cmds, stream_refusals));
    }

    pub(crate) fn close_servo(&self, lease_generation: u64) {
        if lease_generation == 0 {
            return;
        }
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        g.port.close_servo(lease_generation);
        g.prune_closed_stream_owners_locked();
    }

    /// Non-blocking variant of [`Self::close_servo`] for disconnect teardown: attempts `try_lock`
    /// and returns `false` without blocking if the dispatcher mutex is contended (the caller's
    /// lock-free `atomic_fenced_lease_gen` update already fences the generation at every boundary
    /// and `armd-dispatch` drains the close asynchronously).
    pub(crate) fn try_close_servo(&self, lease_generation: u64) -> bool {
        if lease_generation == 0 {
            return true;
        }
        match self.inner.try_lock() {
            Ok(mut g) => {
                g.port.close_servo(lease_generation);
                g.prune_closed_stream_owners_locked();
                true
            }
            Err(std::sync::TryLockError::Poisoned(p)) => {
                let mut g = p.into_inner();
                g.port.close_servo(lease_generation);
                g.prune_closed_stream_owners_locked();
                true
            }
            Err(std::sync::TryLockError::WouldBlock) => false,
        }
    }

    pub(crate) fn sync_fenced_generation(&self, fenced_gen: u64) {
        if fenced_gen == 0 {
            return;
        }
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        if fenced_gen > g.port.closed_servo_generation() {
            g.port.close_servo(fenced_gen);
            g.prune_closed_stream_owners_locked();
        }
    }

    pub(crate) fn is_servo_closed(&self, lease_generation: u64) -> bool {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        self.sync_authoritative_fence_locked(&mut g);
        g.port.is_servo_closed(lease_generation)
    }

    pub(crate) fn closed_servo_generation(&self) -> u64 {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        self.sync_authoritative_fence_locked(&mut g);
        g.port.closed_servo_generation()
    }

    pub(crate) fn coalesced(&self) -> u64 {
        self.inner
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .port
            .coalesced()
    }

    pub(crate) fn submitted_count(&self) -> u64 {
        self.inner
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .submitted_count
    }

    pub(crate) fn lease_refused_count(&self, lease_generation: u64) -> u64 {
        self.inner
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .lease_refused_by_gen
            .get(&lease_generation)
            .copied()
            .unwrap_or(0)
    }

    pub(crate) fn unregister_connection(&self, conn_id: u64) {
        if conn_id == 0 {
            return;
        }
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        g.stream_owners.retain(|_, owner| owner.conn_id != conn_id);
    }

    /// Non-blocking variant of [`Self::unregister_connection`] for disconnect teardown (returns
    /// immediately if the dispatcher mutex is contended; `prune_closed_stream_owners_locked`
    /// reaps closed generations once the mutex is free).
    pub(crate) fn try_unregister_connection(&self, conn_id: u64) -> bool {
        if conn_id == 0 {
            return true;
        }
        match self.inner.try_lock() {
            Ok(mut g) => {
                g.stream_owners.retain(|_, owner| owner.conn_id != conn_id);
                true
            }
            Err(std::sync::TryLockError::Poisoned(p)) => {
                let mut g = p.into_inner();
                g.stream_owners.retain(|_, owner| owner.conn_id != conn_id);
                true
            }
            Err(std::sync::TryLockError::WouldBlock) => false,
        }
    }

    fn cancel_command_waiter(&self, key: (super::mailbox::StreamId, u64)) {
        self.inner
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .command_waiters
            .remove(&key);
    }

    pub(crate) fn finish_stopped(&self, hub: &StateHub) {
        let mut resolved_cmds = Vec::new();
        let mut stream_refusals = Vec::new();
        {
            let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
            g.closed = true;
            g.pop_outcomes_locked(hub, &mut resolved_cmds, &mut stream_refusals);
            for (_, tx) in g.command_waiters.drain() {
                resolved_cmds.push((tx, ServoCommandOutcome::Stopped));
            }
            g.stream_owners.clear();
        }
        Self::deliver_drained(hub, (resolved_cmds, stream_refusals));
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn submit_servo_stream(
        &self,
        hub: &StateHub,
        conn_id: u64,
        conn_name: &str,
        reply_sink: &ServoStreamReplySink,
        stream: &mut crate::supervisor::targets::ServoStream,
        msg: &crate::supervisor::targets::ServoMessage<'_>,
    ) -> Result<Option<serde_json::Value>, ()> {
        let mut resolved_cmds = Vec::new();
        let mut stream_refusals = Vec::new();
        let res = {
            let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
            self.sync_authoritative_fence_locked(&mut g);
            g.pop_outcomes_locked(hub, &mut resolved_cmds, &mut stream_refusals);
            if g.closed || self.exited.load(Ordering::Acquire) {
                Err(())
            } else {
                let coalesced_before = g.port.coalesced();
                let prev_key = g.last_inflight_key;
                let seq_before = g.port.last_seq();
                let immediate = g.port.submit_servo(stream, msg);
                if immediate.is_none() && g.port.last_seq() > seq_before {
                    g.submitted_count = g.submitted_count.saturating_add(1);
                    let assigned_seq = g.port.last_seq();
                    let sid = crate::supervisor::targets::servo_stream(msg.authority.lease_generation);
                    g.last_inflight_key = Some((sid, assigned_seq));
                    if g.port.coalesced() > coalesced_before {
                        g.note_coalesced_locked(prev_key, &mut resolved_cmds);
                    }
                    let owner = g.stream_owners.entry(sid).or_insert_with(|| ServoStreamOwner {
                        conn_id,
                        conn_name: conn_name.to_owned(),
                        lease_generation: msg.authority.lease_generation,
                        reply_sink: Arc::clone(reply_sink),
                        pending_seqs: VecDeque::with_capacity(STREAM_PENDING_SEQS_CAP),
                    });
                    owner.conn_id = conn_id;
                    if owner.conn_name != conn_name {
                        owner.conn_name.clear();
                        owner.conn_name.push_str(conn_name);
                    }
                    owner.lease_generation = msg.authority.lease_generation;
                    owner.reply_sink = Arc::clone(reply_sink);
                    if owner.pending_seqs.len() >= STREAM_PENDING_SEQS_CAP {
                        owner.pending_seqs.pop_front();
                    }
                    owner.pending_seqs.push_back(assigned_seq);
                }
                Ok(immediate)
            }
        };
        Self::deliver_drained(hub, (resolved_cmds, stream_refusals));
        res
    }

    pub(crate) fn submit_servo_command(
        self: &Arc<Self>,
        hub: &StateHub,
        stream: &mut crate::supervisor::targets::ServoStream,
        msg: &crate::supervisor::targets::ServoMessage<'_>,
    ) -> Result<ServoCommandWaiter, ServoCommandSubmitError> {
        let mut resolved_cmds = Vec::new();
        let mut stream_refusals = Vec::new();
        let res = {
            let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
            self.sync_authoritative_fence_locked(&mut g);
            g.pop_outcomes_locked(hub, &mut resolved_cmds, &mut stream_refusals);
            if g.closed || self.exited.load(Ordering::Acquire) {
                Err(ServoCommandSubmitError::Unavailable)
            } else {
                let coalesced_before = g.port.coalesced();
                let prev_key = g.last_inflight_key;
                let seq_before = g.port.last_seq();
                match g.port.submit_servo(stream, msg) {
                    Some(reply) => Err(ServoCommandSubmitError::Immediate(reply)),
                    None if g.port.last_seq() > seq_before => {
                        g.submitted_count = g.submitted_count.saturating_add(1);
                        let assigned_seq = g.port.last_seq();
                        let sid = crate::supervisor::targets::servo_stream(msg.authority.lease_generation);
                        g.last_inflight_key = Some((sid, assigned_seq));
                        if g.port.coalesced() > coalesced_before {
                            g.note_coalesced_locked(prev_key, &mut resolved_cmds);
                        }
                        let (tx, rx) = tokio::sync::oneshot::channel();
                        g.command_waiters.insert((sid, assigned_seq), tx);
                        Ok(ServoCommandWaiter {
                            dispatcher: Arc::clone(self),
                            key: (sid, assigned_seq),
                            rx,
                            completed: false,
                        })
                    }
                    None => Err(ServoCommandSubmitError::Unavailable),
                }
            }
        };
        Self::deliver_drained(hub, (resolved_cmds, stream_refusals));
        res
    }

    #[cfg(feature = "policy-runtime")]
    pub(crate) fn submit_policy(
        &self,
        hub: &StateHub,
        target: &crate::supervisor::targets::PolicyTarget<'_>,
        validity: super::mailbox::PolicyValidityBounds,
    ) -> super::mailbox::Submitted {
        let mut resolved_cmds = Vec::new();
        let mut stream_refusals = Vec::new();
        let sub = {
            let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
            self.sync_authoritative_fence_locked(&mut g);
            g.pop_outcomes_locked(hub, &mut resolved_cmds, &mut stream_refusals);
            if g.closed || self.exited.load(Ordering::Acquire) {
                super::mailbox::Submitted::OutOfOrder
            } else {
                let coalesced_before = g.port.coalesced();
                let prev_key = g.last_inflight_key;
                let sub = g.port.submit_policy_with_validity(target, validity);
                match sub {
                    super::mailbox::Submitted::Accepted
                    | super::mailbox::Submitted::Coalesced { .. } => {
                        g.policy_submitted_count = g.policy_submitted_count.saturating_add(1);
                        if target.action_id != 0
                            && !g.tracked_policy_streams.contains(&target.action_id)
                        {
                            if g.tracked_policy_streams.len() >= 64 {
                                g.tracked_policy_streams.pop_front();
                            }
                            g.tracked_policy_streams.push_back(target.action_id);
                        }
                        let assigned_seq = g.port.last_seq();
                        let sid = crate::supervisor::targets::policy_stream(target.action_id);
                        g.last_inflight_key = Some((sid, assigned_seq));
                        if matches!(sub, super::mailbox::Submitted::Coalesced { .. })
                            || g.port.coalesced() > coalesced_before
                        {
                            g.note_coalesced_locked(prev_key, &mut resolved_cmds);
                        }
                    }
                    super::mailbox::Submitted::Busy => {
                        g.policy_busy_count = g.policy_busy_count.saturating_add(1);
                    }
                    _ => {}
                }
                sub
            }
        };
        Self::deliver_drained(hub, (resolved_cmds, stream_refusals));
        sub
    }

    /// Runs a synchronous final policy commit gate and optional mailbox publication under a
    /// single acquisition of the dispatcher mutex (after draining outcomes and verifying stream
    /// liveness), so no mutex stall can occur between slot selection (`gate_now`) and triple-buffer
    /// publication.
    #[cfg(feature = "policy-runtime")]
    pub(crate) fn submit_policy_gated<R, F>(
        &self,
        hub: &StateHub,
        action_id: u64,
        gate: F,
    ) -> R
    where
        F: FnOnce(
            bool,
            bool,
            &mut dyn FnMut(
                &crate::supervisor::targets::PolicyTarget<'_>,
                super::mailbox::PolicyValidityBounds,
            ) -> super::mailbox::Submitted,
        ) -> R,
    {
        let mut resolved_cmds = Vec::new();
        let mut stream_refusals = Vec::new();
        let res = {
            let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
            self.sync_authoritative_fence_locked(&mut g);
            g.pop_outcomes_locked(hub, &mut resolved_cmds, &mut stream_refusals);
            let mut has_pending_refusal = false;
            g.policy_outcomes.retain(|(aid, o)| {
                if *aid == action_id {
                    if o.result.is_err() {
                        has_pending_refusal = true;
                    }
                    false
                } else {
                    true
                }
            });
            let is_closed = g.closed
                || self.exited.load(Ordering::Acquire)
                || g.port.is_policy_closed(action_id);
            let mut submit_fn =
                |target: &crate::supervisor::targets::PolicyTarget<'_>,
                 validity: super::mailbox::PolicyValidityBounds|
                 -> super::mailbox::Submitted {
                    if is_closed {
                        return super::mailbox::Submitted::OutOfOrder;
                    }
                    let coalesced_before = g.port.coalesced();
                    let prev_key = g.last_inflight_key;
                    let sub = g.port.submit_policy_with_validity(target, validity);
                    match sub {
                        super::mailbox::Submitted::Accepted
                        | super::mailbox::Submitted::Coalesced { .. } => {
                            g.policy_submitted_count = g.policy_submitted_count.saturating_add(1);
                            if target.action_id != 0
                                && !g.tracked_policy_streams.contains(&target.action_id)
                            {
                                if g.tracked_policy_streams.len() >= 64 {
                                    g.tracked_policy_streams.pop_front();
                                }
                                g.tracked_policy_streams.push_back(target.action_id);
                            }
                            let assigned_seq = g.port.last_seq();
                            let sid = crate::supervisor::targets::policy_stream(target.action_id);
                            g.last_inflight_key = Some((sid, assigned_seq));
                            if matches!(sub, super::mailbox::Submitted::Coalesced { .. })
                                || g.port.coalesced() > coalesced_before
                            {
                                g.note_coalesced_locked(prev_key, &mut resolved_cmds);
                            }
                        }
                        super::mailbox::Submitted::Busy => {
                            g.policy_busy_count = g.policy_busy_count.saturating_add(1);
                        }
                        _ => {}
                    }
                    sub
                };
            gate(is_closed, has_pending_refusal, &mut submit_fn)
        };
        Self::deliver_drained(hub, (resolved_cmds, stream_refusals));
        res
    }

    #[cfg(feature = "policy-runtime")]
    pub(crate) fn close_policy(&self, action_id: u64) {
        if action_id == 0 {
            return;
        }
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        g.port.close_policy(action_id);
    }

    #[cfg(feature = "policy-runtime")]
    pub(crate) fn is_policy_closed(&self, action_id: u64) -> bool {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        self.sync_authoritative_fence_locked(&mut g);
        g.port.is_policy_closed(action_id)
    }

    #[cfg(feature = "policy-runtime")]
    pub(crate) fn closed_policy_action_id(&self) -> u64 {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        self.sync_authoritative_fence_locked(&mut g);
        g.port.closed_policy_action_id()
    }

    #[cfg(feature = "policy-runtime")]
    pub(crate) fn policy_submitted_count(&self) -> u64 {
        self.inner
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .policy_submitted_count
    }

    #[cfg(feature = "policy-runtime")]
    pub(crate) fn policy_busy_count(&self) -> u64 {
        self.inner
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .policy_busy_count
    }

    #[cfg(feature = "policy-runtime")]
    pub(crate) fn drain_policy_outcomes(
        &self,
        hub: &StateHub,
        action_id: u64,
    ) -> Vec<super::mailbox::TargetOutcome> {
        let mut resolved_cmds = Vec::new();
        let mut stream_refusals = Vec::new();
        let mut matching = Vec::new();
        {
            let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
            self.sync_authoritative_fence_locked(&mut g);
            g.pop_outcomes_locked(hub, &mut resolved_cmds, &mut stream_refusals);
            let closed_id = g.port.closed_policy_action_id();
            let mut remaining = VecDeque::with_capacity(g.policy_outcomes.len());
            while let Some((id, outcome)) = g.policy_outcomes.pop_front() {
                if id == action_id {
                    matching.push(outcome);
                } else if id > closed_id {
                    remaining.push_back((id, outcome));
                }
            }
            g.policy_outcomes = remaining;
        }
        Self::deliver_drained(hub, (resolved_cmds, stream_refusals));
        matching
    }
}

/// Low-level mutex view onto the executor's [`TargetWriter`] (preserving compatibility for raw
/// mailbox tests without stealing outcomes owned by [`ServoTargetDispatcher`]).
pub struct TargetWriterHandle {
    dispatcher: Arc<ServoTargetDispatcher>,
    hub: Arc<StateHub>,
}

/// Guard returned by [`TargetWriterHandle::lock`].
pub struct TargetWriterGuard<'a> {
    inner: std::sync::MutexGuard<'a, ServoTargetDispatcherInner>,
    hub: &'a StateHub,
}

impl TargetWriterHandle {
    pub fn lock(&self) -> std::sync::LockResult<TargetWriterGuard<'_>> {
        match self.dispatcher.inner.lock() {
            Ok(inner) => Ok(TargetWriterGuard { inner, hub: &self.hub }),
            Err(poisoned) => Err(std::sync::PoisonError::new(TargetWriterGuard {
                inner: poisoned.into_inner(),
                hub: &self.hub,
            })),
        }
    }
}

impl TargetWriterGuard<'_> {
    pub fn submit(&mut self, frame: super::mailbox::TargetFrame) -> super::mailbox::Submitted {
        let coalesced_before = self.inner.port.coalesced();
        let prev_key = self.inner.last_inflight_key;
        let sub = self.inner.port.writer_mut().submit(frame);
        if matches!(
            sub,
            super::mailbox::Submitted::Accepted | super::mailbox::Submitted::Coalesced { .. }
        ) {
            self.inner.last_inflight_key = Some((frame.stream, frame.seq));
            if self.inner.port.coalesced() > coalesced_before {
                let mut resolved = Vec::new();
                self.inner.note_coalesced_locked(prev_key, &mut resolved);
                for (tx, outcome) in resolved {
                    let _ = tx.send(outcome);
                }
            }
        }
        sub
    }

    pub fn outcomes(&mut self, mut each: impl FnMut(super::mailbox::TargetOutcome)) -> usize {
        let mut resolved_cmds = Vec::new();
        let mut stream_refusals = Vec::new();
        self.inner
            .pop_outcomes_locked(self.hub, &mut resolved_cmds, &mut stream_refusals);
        let mut n = 0;
        while let Some(o) = self.inner.unclaimed_outcomes.pop_front() {
            each(o);
            n += 1;
        }
        for (tx, outcome) in resolved_cmds {
            let _ = tx.send(outcome);
        }
        if !stream_refusals.is_empty() {
            ServoTargetDispatcher::deliver_drained(self.hub, (Vec::new(), stream_refusals));
        }
        n
    }

    pub fn close_stream(&mut self, stream: super::mailbox::StreamId) {
        self.inner.port.writer_mut().close_stream(stream);
    }

    pub fn coalesced(&self) -> u64 {
        self.inner.port.coalesced()
    }
}

/// Synchronized completion state for `armd-control`:
/// - `dropping` becomes `true` under the mutex only after the final `teardown_disable.requested`
///   checkpoint immediately before `drop(core)`.
/// - `exited` becomes `true` strictly AFTER `drop(core)` completes on `armd-control`.
#[derive(Clone, Copy, Default)]
struct ControlExitState {
    dropping: bool,
    exited: bool,
}

struct RtExecutorShared {
    boot: BootId,
    broker: Arc<Mutex<Broker<PlanSlot>>>,
    reply_router: Arc<Mutex<ActionReplyRouter>>,
    target_writer: TargetWriterHandle,
    servo_dispatcher: Arc<ServoTargetDispatcher>,
    stop_requests: StopRequests,
    estop_state: Arc<crate::rt_core::urgent::EstopState>,
    stats: Arc<ExecutorStats>,
    maint_client: Arc<MaintClient>,
    urgent_stop_client: Arc<UrgentStopClient>,
    conn_lifecycle_client: Arc<super::requests::lease::ConnLifecycleClient>,
    broker_dispatcher: Arc<BrokerDispatcher>,
    hub: Arc<StateHub>,
    tick_stats: Arc<crate::diagnostics::tick::LockFreeTickStats>,
    sinks: Arc<OffThreadSinks>,
    executor_thread_id: ThreadId,
    control_stop: Arc<AtomicBool>,
    persist_stop: Arc<AtomicBool>,
    exited: Arc<AtomicBool>,
    control_exited_cv: Arc<(Mutex<ControlExitState>, Condvar)>,
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
    baseline_identity: BaselineIdentity,
}

impl RtExecutorShared {
    /// Stops the RT owner (`armd-control`) FIRST so a stalled persistence worker can never block
    /// owner shutdown or emergency hardware disable, then joins or bounds persistence cleanup.
    fn teardown_and_join_inner(&self, emergency_bound: Option<Duration>) {
        // Non-blockingly ask persistence to wind down, but NEVER wait for it before stopping
        // `armd-control`: a blocked directory `sync_all()` must not delay torque-off.
        self.persist_stop.store(true, Ordering::Release);
        self.persist_park_thread.unpark();
        self.maint_client.close();
        self.urgent_stop_client.close();
        self.conn_lifecycle_client.close();
        self.stats.signal_stop(&self.control_stop);
        if let Some(clock) = self.clock.as_ref() {
            clock.wake();
        }

        // Unified bounded completion protocol across emergency, panic, and ordinary teardown:
        // - Emergency teardown shares ONE absolute deadline across `wait_outcome` and thread exit.
        // - `control_exited_cv.exited` is published strictly AFTER `drop(core)` completes.
        // - On timeout before `armd-control` finishes, lock `control_exited_cv.0`: if not yet
        //   exited and shutdown was NOT owner-confirmed, latch `estop_state` and set
        //   `teardown_disable.requested = true` UNDER THAT SAME MUTEX so `armd-control` cannot
        //   enter `drop(core)` without observing the disable request and fencing remaining TX.
        let mut finished = if let Some(bound) = emergency_bound {
            let control_deadline = Instant::now() + bound;
            let _ = self
                .teardown_disable
                .wait_outcome(control_deadline.saturating_duration_since(Instant::now()));
            wait_control_exited_or_timed_out_until(
                &self.control_exited_cv,
                &self.teardown_disable,
                control_deadline.max(Instant::now() + Duration::from_millis(25)),
            )
        } else {
            let control_deadline =
                Instant::now() + self.teardown_disable_timeout + Duration::from_millis(100);
            wait_control_exited_or_timed_out_until(
                &self.control_exited_cv,
                &self.teardown_disable,
                control_deadline,
            )
        };
        if !finished {
            let st = self
                .control_exited_cv
                .0
                .lock()
                .unwrap_or_else(|e| e.into_inner());
            if st.exited {
                finished = true;
            } else if !self.shutdown_confirmed.load(Ordering::Acquire) {
                self.estop_state.request_and_latch();
                if !st.dropping {
                    self.teardown_disable.requested.store(true, Ordering::Release);
                }
            }
        }
        if let Some(jh) = self.control_thread.lock().unwrap_or_else(|e| e.into_inner()).take() {
            if finished {
                let _ = jh.join();
            } else {
                self.exited.store(true, Ordering::Release);
                tracing::error!(
                    "Teardown: detaching stalled RT executor thread (`armd-control`) after bounded wait so later teardown/Drop cannot block indefinitely"
                );
                drop(jh);
            }
        }

        if let Some(jh) = self.monitor_thread.lock().unwrap_or_else(|e| e.into_inner()).take() {
            jh.thread().unpark();
            let _ = jh.join();
        }

        // Bounded persistence cleanup on BOTH emergency (`50ms`) and ordinary teardown
        // (`teardown_disable_timeout + 100ms`): normal fsync finishes in milliseconds and joins
        // cleanly with durability intact, while a stalled parent-directory fsync detaches after
        // the bound instead of hanging `shutdown_and_join`, `stop_and_join`, or `Drop`.
        let persist_bound = if emergency_bound.is_some() {
            Duration::from_millis(50)
        } else {
            self.teardown_disable_timeout + Duration::from_millis(100)
        };
        if let Some(jh) = self.persist_thread.lock().unwrap_or_else(|e| e.into_inner()).take() {
            stop_and_join_persist_bounded(
                &self.persist_stop,
                &self.persist_park_thread,
                &self.persist_exited_cv,
                jh,
                Instant::now() + persist_bound,
            );
        }

        // Final off-thread drain and reclamation.
        self.maint_client.finish_stopped();
        self.urgent_stop_client.finish_stopped();
        self.conn_lifecycle_client.finish_stopped();
        self.broker_dispatcher.finish_stopped();
        self.sinks.flush_and_report_shutdown(&self.tick_stats);
        self.servo_dispatcher.finish_stopped(&self.hub);
        if let Some(ring) = self.hub.record_ring() {
            ring.drain();
        }
        let _ = self.hub.latest();
        self.broker.lock().unwrap_or_else(|e| e.into_inner()).poll(|_, _| true);
        self.reply_router.lock().unwrap_or_else(|e| e.into_inner()).waiters.clear();
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
    pub fn boot(&self) -> BootId { self.shared.boot }
    pub fn broker(&self) -> &Arc<Mutex<Broker<PlanSlot>>> { &self.shared.broker }

    /// Submits an internal A6/A7 action command (`BeginMacro`, `InstallChild`, `FinishMacro`, or
    /// `InstallPrepared`) and atomically registers a keyed oneshot waiter in the shared bounded
    /// FIFO reply router BEFORE releasing the broker lock (preventing ACK stealing and
    /// head-of-line deadlock behind unrelated replies).
    ///
    /// For `BeginMacro` (which produces `Ack` followed later by `Terminal` under the same
    /// `RequestId`), the oneshot receives ONLY the initial `Ack`/`Refused`; the subsequent
    /// `Terminal` automatically routes into the external FIFO queue.
    pub fn submit_internal_action<'a>(
        &self,
        client: ClientId,
        seq: u64,
        auth: super::broker::Authority,
        deadline: Instant,
        sub: super::requests::actions::ActionSubmit<'a, PlanSlot>,
    ) -> Result<
        (
            super::broker::RequestId,
            tokio::sync::oneshot::Receiver<super::broker::Reply>,
        ),
        super::broker::AdmitError,
    > {
        if client.0 != 0 && !self.shared.conn_lifecycle_client.is_slot_open(client.0) {
            return Err(super::broker::AdmitError::Disconnected);
        }
        let mut broker = self.shared.broker.lock().unwrap_or_else(|e| e.into_inner());
        broker.reclaim_plans();
        let req_id = broker.submit(
            client,
            seq,
            auth,
            deadline,
            super::broker::Submit::Action(sub),
        )?;
        let (tx, rx) = tokio::sync::oneshot::channel();
        let mut router = self
            .shared
            .reply_router
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        router.waiters.insert(req_id, tx);
        Ok((req_id, rx))
    }

    /// Removes a previously registered internal waiter (e.g. on timeout/cancellation).
    pub fn cancel_internal_waiter(&self, req_id: super::broker::RequestId) {
        self.shared
            .reply_router
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .waiters
            .remove(&req_id);
    }

    /// Single hook where PR L plugs in its per-connection WebSocket reply router.
    /// Once attached, [`Self::has_external_reply_sink`] becomes `true`,
    /// [`crate::macro_action::begin_macro_executor`] admits executor macros, and
    /// [`Self::pump_broker_replies`] forwards external replies (including `BeginMacro`
    /// `Reply::Terminal`s) to `sink` in strict FIFO order without holding broker/router locks.
    pub fn attach_external_reply_sink(&self, sink: Arc<dyn ExternalReplySink>) {
        self.shared
            .reply_router
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .external_sink = Some(sink);
        self.pump_broker_replies();
    }

    /// Detaches the external reply sink (returning executor macro admission to fail-closed
    /// `503 EXECUTOR_ROUTING_PENDING`).
    pub fn detach_external_reply_sink(&self) {
        self.shared
            .reply_router
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .external_sink = None;
    }

    /// Returns `true` when an external reply sink (PR L's per-connection WS reply router) is
    /// currently attached to this executor handle.
    pub fn has_external_reply_sink(&self) -> bool {
        self.shared
            .reply_router
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .external_sink
            .is_some()
    }

    fn pump_broker_to_router(&self) -> (usize, Option<Arc<dyn ExternalReplySink>>) {
        self.shared.broker_dispatcher.pump_to_router()
    }

    /// Pumps ready replies from the broker into matching internal waiters or the bounded external
    /// FIFO buffer (`<= reply_capacity`), forwarding buffered external replies to the attached
    /// [`ExternalReplySink`] (if any) without holding broker/router locks, and reclaims retired
    /// plan slots.
    pub fn pump_broker_replies(&self) -> usize {
        use super::broker::DeliveryDecision;
        let (mut total, sink) = self.pump_broker_to_router();
        if let Some(sink) = sink {
            let mut delivered_any = false;
            loop {
                let next = self
                    .shared
                    .reply_router
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .external
                    .pop_front();
                let Some((client, reply)) = next else {
                    break;
                };
                match sink.deliver(client, reply) {
                    DeliveryDecision::Delivered | DeliveryDecision::Abandoned => {
                        delivered_any = true;
                    }
                    DeliveryDecision::Backpressured => {
                        self.shared
                            .reply_router
                            .lock()
                            .unwrap_or_else(|e| e.into_inner())
                            .external
                            .push_front((client, reply));
                        break;
                    }
                }
            }
            if delivered_any {
                total += self.pump_broker_to_router().0;
            }
        }
        total
    }

    /// Drains external (non-internal-waiter) replies in strict broker FIFO order while honouring
    /// downstream backpressure (`DeliveryDecision::Backpressured` keeps the undelivered reply at
    /// the front of the bounded queue and never steals internal macro ACKs).
    pub fn poll_external_replies<F>(&self, mut deliver: F) -> usize
    where
        F: FnMut(ClientId, super::broker::Reply) -> super::broker::DeliveryDecision,
    {
        use super::broker::DeliveryDecision;
        self.pump_broker_replies();
        let mut delivered = 0usize;
        {
            let mut router = self
                .shared
                .reply_router
                .lock()
                .unwrap_or_else(|e| e.into_inner());
            while let Some((client, reply)) = router.external.pop_front() {
                match deliver(client, reply) {
                    DeliveryDecision::Delivered | DeliveryDecision::Abandoned => delivered += 1,
                    DeliveryDecision::Backpressured => {
                        router.external.push_front((client, reply));
                        break;
                    }
                }
            }
        }
        if delivered > 0 {
            self.pump_broker_replies();
        }
        delivered
    }

    /// Number of buffered external replies currently waiting in the bounded router queue.
    pub fn pending_external_replies(&self) -> usize {
        self.shared
            .reply_router
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .external
            .len()
    }

    pub fn disconnect_client(&self, client: ClientId) {
        self.shared.broker.lock().unwrap_or_else(|e| e.into_inner()).disconnect(client);
    }
    pub fn target_writer(&self) -> &TargetWriterHandle { &self.shared.target_writer }

    /// Dispatches `TargetPort::close_servo(lease_generation)` off-thread (idempotent; fences late
    /// submissions from `lease_generation` and frees its stream slot in `TargetWriter`).
    pub fn close_servo(&self, lease_generation: u64) {
        self.shared.servo_dispatcher.close_servo(lease_generation);
    }

    /// Non-blocking `try_lock` variant of [`Self::close_servo`] for disconnect teardown.
    pub fn try_close_servo(&self, lease_generation: u64) -> bool {
        self.shared.servo_dispatcher.try_close_servo(lease_generation)
    }

    /// True when `lease_generation` (non-zero) has been closed via `close_servo`.
    pub fn is_servo_closed(&self, lease_generation: u64) -> bool {
        self.shared.servo_dispatcher.is_servo_closed(lease_generation)
    }

    /// Highest closed servo lease generation on this executor's `TargetPort`.
    pub fn closed_servo_generation(&self) -> u64 {
        self.shared.servo_dispatcher.closed_servo_generation()
    }

    /// Total servo target frames coalesced before RT consumption.
    pub fn servo_coalesced(&self) -> u64 {
        self.shared.servo_dispatcher.coalesced()
    }

    /// Total servo target frames accepted into the triple-buffer mailbox by `TargetPort::submit_servo`.
    pub fn servo_submitted_count(&self) -> u64 {
        self.shared.servo_dispatcher.submitted_count()
    }

    /// Total RT owner-side lease/stream refusals observed for `servo_stream(lease_generation)`.
    pub fn servo_lease_refused_count(&self, lease_generation: u64) -> u64 {
        self.shared.servo_dispatcher.lease_refused_count(lease_generation)
    }

    /// Unregisters streaming outcome delivery state for a closing WS connection.
    pub fn unregister_servo_connection(&self, conn_id: u64) {
        self.shared.servo_dispatcher.unregister_connection(conn_id);
    }

    /// Non-blocking `try_lock` variant of [`Self::unregister_servo_connection`] for disconnect teardown.
    pub fn try_unregister_servo_connection(&self, conn_id: u64) -> bool {
        self.shared.servo_dispatcher.try_unregister_connection(conn_id)
    }

    /// Routes one WS streaming servo frame (`servo`, `cartesian_joint_stream`, or implicit
    /// `positions`) through [`TargetPort::submit_servo`] off-thread.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn submit_servo_stream(
        &self,
        conn_id: u64,
        conn_name: &str,
        reply_sink: &ServoStreamReplySink,
        stream: &mut crate::supervisor::targets::ServoStream,
        msg: &crate::supervisor::targets::ServoMessage<'_>,
    ) -> Result<Option<serde_json::Value>, ()> {
        self.shared.servo_dispatcher.submit_servo_stream(
            &self.shared.hub,
            conn_id,
            conn_name,
            reply_sink,
            stream,
            msg,
        )
    }

    /// Routes one WS `/api/v1/motion/servo` command through [`TargetPort::submit_servo`] and
    /// returns a drop-safe waiter correlated to the submitted frame's `(stream, seq)`.
    pub(crate) fn submit_servo_command(
        &self,
        stream: &mut crate::supervisor::targets::ServoStream,
        msg: &crate::supervisor::targets::ServoMessage<'_>,
    ) -> Result<ServoCommandWaiter, ServoCommandSubmitError> {
        self.shared
            .servo_dispatcher
            .submit_servo_command(&self.shared.hub, stream, msg)
    }

    /// Submits one live policy target frame synchronously through `TargetPort::submit_policy`
    /// (called off-thread under `HubCommitGuard`, never called in Shadow mode).
    #[cfg(feature = "policy-runtime")]
    pub fn submit_policy(
        &self,
        target: &crate::supervisor::targets::PolicyTarget<'_>,
    ) -> super::mailbox::Submitted {
        self.submit_policy_with_validity(target, super::mailbox::PolicyValidityBounds::default())
    }

    /// Submits one live policy target frame together with its atomic absolute validity bounds
    /// (`vision_window_expires_at`, `handshake_expires_at`) synchronously under `HubCommitGuard`.
    #[cfg(feature = "policy-runtime")]
    pub fn submit_policy_with_validity(
        &self,
        target: &crate::supervisor::targets::PolicyTarget<'_>,
        validity: super::mailbox::PolicyValidityBounds,
    ) -> super::mailbox::Submitted {
        self.shared
            .servo_dispatcher
            .submit_policy(&self.shared.hub, target, validity)
    }

    /// Runs `gate` synchronously inside the target-dispatcher lock (after draining outcomes and
    /// checking stream closure) so `commit_gate(now)` slot selection and triple-buffer publication
    /// execute atomically without intervening mutex waits.
    #[cfg(feature = "policy-runtime")]
    pub(crate) fn submit_policy_gated<R, F>(&self, action_id: u64, gate: F) -> R
    where
        F: FnOnce(
            bool,
            bool,
            &mut dyn FnMut(
                &crate::supervisor::targets::PolicyTarget<'_>,
                super::mailbox::PolicyValidityBounds,
            ) -> super::mailbox::Submitted,
        ) -> R,
    {
        self.shared
            .servo_dispatcher
            .submit_policy_gated(&self.shared.hub, action_id, gate)
    }

    /// Closes policy stream `action_id` off-thread (idempotent; frees stream slot in `TargetWriter`).
    #[cfg(feature = "policy-runtime")]
    pub fn close_policy(&self, action_id: u64) {
        self.shared.servo_dispatcher.close_policy(action_id);
    }

    /// True once `action_id` has been closed via `close_policy` or the RT close watermark.
    #[cfg(feature = "policy-runtime")]
    pub fn is_policy_closed(&self, action_id: u64) -> bool {
        self.shared.servo_dispatcher.is_policy_closed(action_id)
    }

    /// Highest closed policy action ID on this executor's `TargetPort`.
    #[cfg(feature = "policy-runtime")]
    pub fn closed_policy_action_id(&self) -> u64 {
        self.shared.servo_dispatcher.closed_policy_action_id()
    }

    /// Total policy target frames accepted into the triple-buffer mailbox (must stay 0 in Shadow mode).
    #[cfg(feature = "policy-runtime")]
    pub fn policy_submitted_count(&self) -> u64 {
        self.shared.servo_dispatcher.policy_submitted_count()
    }

    /// Total policy target submissions refused `Submitted::Busy` (previous target unconsumed).
    #[cfg(feature = "policy-runtime")]
    pub fn policy_busy_count(&self) -> u64 {
        self.shared.servo_dispatcher.policy_busy_count()
    }

    /// Lock-free monotonic RT policy-refusal generation counter (bumped immediately on Step 4
    /// whenever RT refuses a policy target frame, before Step 7 outcome delivery).
    #[cfg(feature = "policy-runtime")]
    pub fn policy_refusal_gen(&self) -> u64 {
        self.shared.servo_dispatcher.policy_refusal_gen()
    }

    /// Drains RT target outcomes belonging to policy run `action_id`.
    #[cfg(feature = "policy-runtime")]
    pub fn drain_policy_outcomes(&self, action_id: u64) -> Vec<super::mailbox::TargetOutcome> {
        self.shared
            .servo_dispatcher
            .drain_policy_outcomes(&self.shared.hub, action_id)
    }

    /// Submits one `Copy` policy lifecycle command (`ReserveLoad`, `BeginPolicy`, `EndPolicy`),
    /// invoking `on_admitted` strictly BEFORE ring publication on admitted descriptors.
    #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
    pub async fn submit_policy_command_async(
        &self,
        client: super::broker::ClientId,
        client_seq: u64,
        authority: super::broker::Authority,
        deadline: Instant,
        cmd: super::requests::policy::PolicyCommand,
        on_admitted: impl FnOnce(super::broker::RequestId),
    ) -> Result<super::broker::Reply, BrokerDispatchError> {
        let (ack_rx, _term_rx) = self.shared.broker_dispatcher.submit_policy_command(
            client,
            client_seq,
            authority,
            deadline,
            cmd,
            on_admitted,
        )?;
        match ack_rx.await {
            Ok(Ok(reply)) => Ok(reply),
            _ => Err(BrokerDispatchError::Stopped),
        }
    }
    pub fn stop_requests(&self) -> &StopRequests { &self.shared.stop_requests }
    pub fn stats(&self) -> &Arc<ExecutorStats> { &self.shared.stats }
    pub fn executor_thread_id(&self) -> ThreadId { self.shared.executor_thread_id }
    pub fn is_exited(&self) -> bool { self.shared.exited.load(Ordering::Acquire) }
    pub fn baseline_identity(&self) -> &BaselineIdentity { &self.shared.baseline_identity }
    #[doc(hidden)]
    pub fn persist_worker_exited(&self) -> bool { self.shared.persist_worker_exited.load(Ordering::Acquire) }

    pub fn call_maintenance(&self, op: MaintenanceOp) -> Result<MaintenanceResult, OwnerError> {
        self.shared.maint_client.call_sync(op)
    }

    pub async fn call_maintenance_async(&self, op: MaintenanceOp) -> Result<MaintenanceResult, OwnerError> {
        self.shared.maint_client.call_async(op).await
    }

    pub(crate) fn urgent_stop_client(&self) -> Arc<UrgentStopClient> {
        Arc::clone(&self.shared.urgent_stop_client)
    }

    pub fn urgent_stop(&self, stop: UrgentStop) -> Result<UrgentStopOutcome, UrgentStopError> {
        self.shared.urgent_stop_client.submit_sync(stop)
    }

    pub async fn urgent_stop_async(&self, stop: UrgentStop) -> Result<UrgentStopOutcome, UrgentStopError> {
        self.shared.urgent_stop_client.submit_async(stop).await
    }

    pub fn enqueue_urgent_stop(
        &self,
        stop: UrgentStop,
    ) -> Result<tokio::sync::oneshot::Receiver<Result<UrgentStopOutcome, UrgentStopError>>, UrgentStopError> {
        self.shared.urgent_stop_client.enqueue_async(stop)
    }

    pub fn drain_urgent_stop_replies(&self) {
        self.shared.urgent_stop_client.drain_replies();
    }

    pub fn urgent_stop_in_flight(&self) -> usize {
        self.shared.urgent_stop_client.in_flight()
    }

    pub fn open_connection_sync(
        &self,
        conn_id: u64,
    ) -> Result<Result<(), &'static str>, super::requests::lease::ConnLifecycleError> {
        self.shared.conn_lifecycle_client.open_sync(conn_id)
    }

    pub async fn open_connection_async(
        &self,
        conn_id: u64,
    ) -> Result<Result<(), &'static str>, super::requests::lease::ConnLifecycleError> {
        self.shared.conn_lifecycle_client.open_async(conn_id).await
    }

    pub fn disconnect_connection_sync(
        &self,
        connection: super::requests::lease::ConnectionHandle,
    ) -> Result<super::requests::lease::LeaseOutcome, super::requests::lease::ConnLifecycleError>
    {
        self.shared
            .broker_dispatcher
            .disconnect_connection_sync(connection)
    }

    pub async fn disconnect_connection_async(
        &self,
        connection: super::requests::lease::ConnectionHandle,
    ) -> Result<super::requests::lease::LeaseOutcome, super::requests::lease::ConnLifecycleError>
    {
        self.shared
            .broker_dispatcher
            .disconnect_connection(connection)
            .await
    }

    /// Non-blocking fire-and-forget disconnect used strictly as a Drop safety net when an async
    /// WS handler task is aborted externally before reaching its normal async teardown.
    pub fn enqueue_disconnect_detached(
        &self,
        connection: super::requests::lease::ConnectionHandle,
    ) {
        self.shared
            .broker_dispatcher
            .enqueue_disconnect_detached(connection);
    }

    pub async fn submit_lease_async(
        &self,
        client: super::broker::ClientId,
        client_seq: u64,
        authority: super::broker::Authority,
        deadline: Instant,
        req: super::requests::lease::LeaseRequest,
    ) -> Result<super::broker::Reply, BrokerDispatchError> {
        let rx = self.shared.broker_dispatcher.submit_one_shot(
            client,
            client_seq,
            authority,
            deadline,
            super::broker::Submit::Lease(req),
        )?;
        match rx.await {
            Ok(Ok(reply)) => Ok(reply),
            _ => Err(BrokerDispatchError::Stopped),
        }
    }

    pub async fn submit_immediate_async(
        &self,
        client: super::broker::ClientId,
        client_seq: u64,
        authority: super::broker::Authority,
        deadline: Instant,
        op: super::broker::ImmediateOp,
    ) -> Result<super::broker::Reply, BrokerDispatchError> {
        let rx = self.shared.broker_dispatcher.submit_one_shot(
            client,
            client_seq,
            authority,
            deadline,
            super::broker::Submit::Immediate(op),
        )?;
        match rx.await {
            Ok(Ok(reply)) => Ok(reply),
            _ => Err(BrokerDispatchError::Stopped),
        }
    }

    /// Submits a `TransitionOp` (`TorqueOn` or `TorqueOff`) via `BrokerDispatcher::submit_action`
    /// and awaits BOTH its Step 3 `Ack`/`Refused` AND (when accepted) its post-confirmation
    /// `Reply::Terminal`. Never reports success on early `Ack` before hardware confirmation.
    pub async fn submit_transition_async(
        &self,
        client: super::broker::ClientId,
        client_seq: u64,
        authority: super::broker::Authority,
        deadline: Instant,
        op: super::broker::TransitionOp,
    ) -> Result<super::broker::Reply, BrokerDispatchError> {
        let (ack_rx, term_rx) = self.shared.broker_dispatcher.submit_action(
            client,
            client_seq,
            authority,
            deadline,
            super::broker::Submit::Transition(op),
        )?;
        let ack_reply = match ack_rx.await {
            Ok(Ok(reply)) => reply,
            _ => return Err(BrokerDispatchError::Stopped),
        };
        match ack_reply {
            super::broker::Reply::Ack { .. } => match term_rx.await {
                Ok(Ok(term_reply)) => Ok(term_reply),
                _ => Err(BrokerDispatchError::Stopped),
            },
            other => Ok(other),
        }
    }

    /// Routes WS `/api/v1/safety/clear_fault` through the single RT owner via
    /// [`MaintenanceOp::LeaseClearFault`] with live lock-free [`crate::auth::OperatorGrant`]
    /// governed by the WS dispatch table's [`super::requests::lease::OwnerAuthPolicy`].
    #[allow(clippy::too_many_arguments)]
    pub async fn lease_clear_fault_async(
        &self,
        connection: super::requests::lease::ConnectionHandle,
        lease_generation: u64,
        supplied_epoch: Option<u64>,
        request_id_valid: bool,
        body_error: Option<&'static str>,
        auth_policy: super::requests::lease::OwnerAuthPolicy,
        operator: crate::auth::OperatorGrant,
    ) -> Result<super::requests::lease::LeaseClearFaultOutcome, OwnerError> {
        match self
            .call_maintenance_async(MaintenanceOp::LeaseClearFault {
                connection,
                lease_generation,
                supplied_epoch,
                request_id_valid,
                body_error,
                auth_policy,
                operator,
            })
            .await?
        {
            MaintenanceResult::LeaseFaultCleared(out) => Ok(out),
            _ => Err(OwnerError::Protocol),
        }
    }

    /// Evaluates live owner authority, pending E-stop, operator auth policy, and body/request_id
    /// precedence on the RT owner for malformed or policy-gated WS `/api/v1/torque` & `/api/v1/arm`
    /// commands (zero heap allocations on `armd-control`).
    #[allow(clippy::too_many_arguments)]
    pub async fn lease_torque_preflight_async(
        &self,
        connection: super::requests::lease::ConnectionHandle,
        lease_generation: u64,
        supplied_epoch: Option<u64>,
        request_id_valid: bool,
        body_error: Option<&'static str>,
        auth_policy: super::requests::lease::OwnerAuthPolicy,
        operator: crate::auth::OperatorGrant,
    ) -> Result<Result<u64, super::requests::lease::TorquePreflightError>, OwnerError> {
        match self
            .call_maintenance_async(MaintenanceOp::LeaseTorquePreflight {
                connection,
                lease_generation,
                supplied_epoch,
                request_id_valid,
                body_error,
                auth_policy,
                operator,
            })
            .await?
        {
            MaintenanceResult::LeaseTorquePreflight(res) => Ok(res),
            _ => Err(OwnerError::Protocol),
        }
    }

    pub fn drain_broker_replies(&self) {
        self.shared.broker_dispatcher.drain_replies();
    }

    pub fn drain_conn_lifecycle_replies(&self) {
        self.shared.conn_lifecycle_client.drain_replies();
    }

    pub fn live_connection_slots(&self) -> usize {
        self.shared.conn_lifecycle_client.live_slot_count()
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
            MaintenanceResult::JointZeroOffsetsSet(r) => Ok(r.map_err(String::from)),
            _ => Err(OwnerError::Protocol),
        }
    }

    pub async fn set_joint_zero_offsets_async(&self, offsets: Vec<f32>) -> Result<Result<(), String>, OwnerError> {
        match self
            .call_maintenance_async(MaintenanceOp::SetJointZeroOffsets { offsets })
            .await?
        {
            MaintenanceResult::JointZeroOffsetsSet(r) => Ok(r.map_err(String::from)),
            _ => Err(OwnerError::Protocol),
        }
    }

    pub fn set_motor_zero(&self, joint: usize) -> Result<Result<(), String>, OwnerError> {
        match self.call_maintenance(MaintenanceOp::SetMotorZero { joint })? {
            MaintenanceResult::MotorZeroSet(r) => Ok(r.map_err(String::from)),
            _ => Err(OwnerError::Protocol),
        }
    }

    pub async fn set_motor_zero_async(&self, joint: usize) -> Result<Result<(), String>, OwnerError> {
        match self.call_maintenance_async(MaintenanceOp::SetMotorZero { joint }).await? {
            MaintenanceResult::MotorZeroSet(r) => Ok(r.map_err(String::from)),
            _ => Err(OwnerError::Protocol),
        }
    }

    pub fn call_config(
        &self,
        req: super::requests::config::ConfigRequest,
    ) -> Result<super::requests::config::ConfigReply, OwnerError> {
        let ex = super::requests::config::ConfigExchange::new(req);
        match self.call_maintenance(MaintenanceOp::Config(ex))? {
            MaintenanceResult::Config(mut ex) => ex.reply.take().ok_or(OwnerError::Protocol),
            _ => Err(OwnerError::Protocol),
        }
    }

    pub async fn call_config_async(
        &self,
        req: super::requests::config::ConfigRequest,
    ) -> Result<super::requests::config::ConfigReply, OwnerError> {
        let ex = super::requests::config::ConfigExchange::new(req);
        match self.call_maintenance_async(MaintenanceOp::Config(ex)).await? {
            MaintenanceResult::Config(mut ex) => ex.reply.take().ok_or(OwnerError::Protocol),
            _ => Err(OwnerError::Protocol),
        }
    }

    /// Like [`Self::call_config_async`], but if the maintenance queue is full before enqueue,
    /// returns the untouched `ConfigRequest` (preserving linear `Reservation` tokens for retry).
    pub async fn call_config_try_async(
        &self,
        req: super::requests::config::ConfigRequest,
    ) -> Result<super::requests::config::ConfigReply, (super::requests::config::ConfigRequest, OwnerError)> {
        let ex = super::requests::config::ConfigExchange::new(req);
        match self.shared.maint_client.call_try_async(MaintenanceOp::Config(ex)).await {
            Ok(MaintenanceResult::Config(mut ex)) => ex
                .reply
                .take()
                .ok_or((
                    super::requests::config::ConfigRequest::SampleTableCapture,
                    OwnerError::Protocol,
                )),
            Ok(_) => Err((
                super::requests::config::ConfigRequest::SampleTableCapture,
                OwnerError::Protocol,
            )),
            Err((MaintenanceOp::Config(mut ex), err)) => Err((
                ex.req
                    .take()
                    .unwrap_or(super::requests::config::ConfigRequest::SampleTableCapture),
                err,
            )),
            Err((_, err)) => Err((
                super::requests::config::ConfigRequest::SampleTableCapture,
                err,
            )),
        }
    }

    pub async fn take_baseline_pending_async(
        &self,
        clear_dirty: bool,
    ) -> Result<BaselineTakeReply, OwnerError> {
        let fresh_pending = Vec::with_capacity(crate::supervisor::GRIPPER_BASELINE_PENDING_MAX);
        let mut fresh_samples = Vec::with_capacity(crate::supervisor::GRIPPER_BASELINE_PENDING_MAX);
        for _ in 0..crate::supervisor::GRIPPER_BASELINE_PENDING_MAX {
            fresh_samples.push(crate::gripper_baseline::MoveSamples::default());
        }
        match self
            .call_maintenance_async(MaintenanceOp::TakeBaselinePending {
                fresh_pending,
                fresh_samples,
                clear_dirty,
            })
            .await?
        {
            MaintenanceResult::BaselineTaken(reply) => Ok(reply),
            _ => Err(OwnerError::Protocol),
        }
    }

    pub async fn store_baseline_async(
        &self,
        baseline: crate::gripper_baseline::FreeTorqueBaseline,
        expected_generation: u64,
        mark_dirty: bool,
    ) -> Result<bool, OwnerError> {
        match self
            .call_maintenance_async(MaintenanceOp::StoreBaseline {
                baseline,
                expected_generation,
                mark_dirty,
            })
            .await?
        {
            MaintenanceResult::BaselineStored(stored) => Ok(stored),
            _ => Err(OwnerError::Protocol),
        }
    }

    pub async fn mark_baseline_dirty_async(&self) -> Result<(), OwnerError> {
        match self.call_maintenance_async(MaintenanceOp::MarkBaselineDirty).await? {
            MaintenanceResult::BaselineDirtyMarked => Ok(()),
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

    #[doc(hidden)]
    pub fn last_teardown_disable_outcome(&self) -> Option<crate::supervisor::PanicDisableOutcome> {
        self.shared
            .teardown_disable
            .outcome
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
    }
}

/// Typed startup error preserving `503 UNSUPPORTED_RT_DRIVER` and untouched Legacy core rollback
/// vs fatal `OWNER_FAILED` transitions.
pub(crate) enum SpawnExecutorError {
    Recoverable {
        core: Box<SupervisorInner>,
        unsupported_reason: Option<&'static str>,
        msg: String,
    },
    Fatal(String),
}

/// Spawns the synchronous RT executor (`armd-control`) and dedicated persistence worker (`armd-persist`).
pub(crate) fn spawn_executor(
    core: SupervisorInner,
    cfg: ExecutorConfig,
    initial_generation: u64,
    config_write_mutex: Arc<tokio::sync::Mutex<()>>,
    published_config: Arc<super::requests::config::PublishedConfigHub>,
    estop_reg: Arc<Mutex<EstopRegistry>>,
    tick_stats: Arc<crate::diagnostics::tick::LockFreeTickStats>,
    control_sched: Arc<Mutex<Option<crate::diagnostics::tick::ControlThreadSched>>>,
    control_period_us: Arc<AtomicU64>,
    hooks: PersistTestHooks,
) -> Result<(RtExecutorHandle, Arc<OffThreadSinks>), SpawnExecutorError> {
    let hub = core.core.hub().clone();
    let estop_state = core.estop_state.clone();
    let baseline_identity = BaselineIdentity {
        tool_id: core.config.active_tool.id.clone(),
        tool_config_digest: core.config.active_tool.config_digest.clone(),
        follower: core.follower.clone(),
        path: core.gripper_baseline_path.clone(),
    };
    let (mut engine, broker, target_writer, stop_requests) = ReleaseEngine::new(cfg.boot, cfg.capacities);
    let closed_bitset = broker.closed_bitset();
    // Defer mutating `core`'s boot ID, `StopRequests`, driver E-stop gate, closed-connection
    // bitset, and `ActionPrepEnv` until inside `armd-control`'s `catch_unwind` startup transaction
    // so: (a) recoverable spawn/init failure returns `core` cleanly with Legacy routing intact,
    // and (b) any driver panic during gate/capability setup is caught on `armd-control` and
    // transitions cleanly to `OWNER_FAILED` instead of unwinding past the moved core on the caller.

    let (diag_tx, diag_rx) = RingBuffer::<TickSample>::new(DIAG_RING_CAPACITY);
    let (diag_amend_tx, diag_amend_rx) = RingBuffer::<(u64, u32)>::new(DIAG_RING_CAPACITY);
    let (maint_req_tx, maint_req_rx) = RingBuffer::<MaintRequest>::new(MAINT_QUEUE_CAPACITY);
    let (maint_rep_tx, maint_rep_rx) = RingBuffer::<MaintReply>::new(MAINT_QUEUE_CAPACITY);
    let (estop_ack_tx, estop_ack_rx) = RingBuffer::<EstopAck>::new(ESTOP_ACK_CAPACITY);
    let (reclaim_state_tx, reclaim_state_rx) = RingBuffer::<Arc<SafetyState>>::new(RECLAIM_STATE_CAPACITY);
    let (reclaim_stop_tx, reclaim_stop_rx) = RingBuffer::<RetiredStopPayloads>::new(URGENT_STOP_CAPACITY);
    let (reclaim_traj_tx, reclaim_traj_rx) =
        RingBuffer::<crate::motion::ActiveTrajectory>::new(RECLAIM_TRAJ_CAPACITY);
    let (reclaim_macro_tx, reclaim_macro_rx) = RingBuffer::<crate::macro_action::MacroRun>::new(8);
    #[cfg(feature = "policy-runtime")]
    let (reclaim_policy_tx, reclaim_policy_rx) = RingBuffer::<crate::policy_runtime::PolicyRun>::new(8);
    let (reclaim_return_tx, reclaim_return_rx) = RingBuffer::<Arc<SafetyState>>::new(
        FAULT_ARC_POOL_CAPACITY + FORMATTED_FAULT_ARC_POOL_CAPACITY + JOINT_LIMIT_ARC_POOL_CAPACITY,
    );
    let (safety_log_tx, safety_log_rx) = RingBuffer::<SafetyLogEvent>::new(SAFETY_LOG_CAPACITY);
    let (persist_job_tx, persist_job_rx) = RingBuffer::<PendingZeroCheckPersist>::new(4);
    let (persist_cmd_tx, persist_cmd_rx) = RingBuffer::<PersistCmd>::new(2);
    let (persist_rep_tx, persist_rep_rx) = RingBuffer::<PersistReply>::new(PERSIST_REP_CAPACITY);
    let (watchdog_plan_tx, watchdog_plan_rx) = RingBuffer::<WatchdogParkPlan>::new(2);
    let (offload_reclaim_tx, offload_reclaim_rx) =
        RingBuffer::<OffloadReclaim>::new(OFFLOAD_RECLAIM_CAPACITY);

    let estop_overflow_gen = Arc::new(AtomicU64::new(0));
    let control_stop = Arc::new(AtomicBool::new(false));
    let persist_stop = Arc::new(AtomicBool::new(false));
    let exited = Arc::new(AtomicBool::new(false));
    let (urgent_stop_client, urgent_stop_intake) =
        urgent_stop_channel(URGENT_STOP_CAPACITY, exited.clone());
    engine.attach_urgent_stops(urgent_stop_intake);
    let urgent_stop_client = Arc::new(urgent_stop_client);
    let (conn_lifecycle_client, conn_lifecycle_intake) =
        super::requests::lease::conn_lifecycle_channel(exited.clone());
    conn_lifecycle_client.seed_live_connections(core.live_connection_ids());
    engine.attach_conn_lifecycle(conn_lifecycle_intake);
    let (eviction_tx, eviction_rx) =
        RingBuffer::<(super::requests::lease::ConnectionHandle, u64)>::new(32);
    engine.attach_eviction_tx(eviction_tx);
    let (servo_close_tx, servo_close_rx) = RingBuffer::<u64>::new(64);
    engine.attach_servo_close_tx(servo_close_tx);
    let atomic_fenced_lease_gen = core.atomic_fenced_lease_gen.clone();
    #[cfg(feature = "policy-runtime")]
    let atomic_closed_policy_action_id = core.atomic_closed_policy_action_id.clone();
    let servo_dispatcher = Arc::new(ServoTargetDispatcher::new(
        target_writer,
        core.core.id(),
        cfg.boot,
        exited.clone(),
        Arc::clone(&atomic_fenced_lease_gen),
        #[cfg(feature = "policy-runtime")]
        atomic_closed_policy_action_id,
    ));
    let target_writer_handle = TargetWriterHandle {
        dispatcher: Arc::clone(&servo_dispatcher),
        hub: Arc::clone(&hub),
    };
    let conn_lifecycle_client = Arc::new(conn_lifecycle_client);
    let broker = Arc::new(Mutex::new(broker));
    let reply_router = Arc::new(Mutex::new(ActionReplyRouter::new(cfg.capacities.replies)));
    let broker_dispatcher = Arc::new(BrokerDispatcher::new(
        Arc::clone(&broker),
        Arc::clone(&conn_lifecycle_client),
        Arc::clone(&reply_router),
        exited.clone(),
    ));
    let control_exited_cv = Arc::new((Mutex::new(ControlExitState::default()), Condvar::new()));
    let persist_worker_exited = Arc::new(AtomicBool::new(false));
    let persist_exited_cv = Arc::new((Mutex::new(false), Condvar::new()));
    let teardown_disable = Arc::new(TeardownDisable::default());
    let stats = Arc::new(ExecutorStats::default());

    let shutdown_hold_error = Arc::new(Mutex::new(None));
    let persist_rep_rx = Arc::new(Mutex::new(persist_rep_rx));
    let maint_client = Arc::new(MaintClient::new(
        maint_req_tx,
        maint_rep_rx,
        exited.clone(),
        Arc::clone(&hub),
    ));
    let sinks = Arc::new(OffThreadSinks {
        diag_rx: Mutex::new((diag_rx, diag_amend_rx)),
        estop_reg: estop_reg.clone(),
        reclaim_state_rx: Mutex::new(reclaim_state_rx),
        reclaim_stop_rx: Mutex::new(reclaim_stop_rx),
        reclaim_traj_rx: Mutex::new(reclaim_traj_rx),
        reclaim_macro_rx: Mutex::new(reclaim_macro_rx),
        #[cfg(feature = "policy-runtime")]
        reclaim_policy_rx: Mutex::new(reclaim_policy_rx),
        reclaim_return_tx: Mutex::new(reclaim_return_tx),
        safety_log_rx: Mutex::new((safety_log_rx, 0, 0)),
        shutdown_hold_error: shutdown_hold_error.clone(),
        stats: stats.clone(),
        persist_rep_rx: persist_rep_rx.clone(),
        persist_worker_exited: persist_worker_exited.clone(),
        maint_client: maint_client.clone(),
        urgent_stop_client: urgent_stop_client.clone(),
        conn_lifecycle_client: conn_lifecycle_client.clone(),
        broker_dispatcher: broker_dispatcher.clone(),
        eviction_rx: Mutex::new(eviction_rx),
        servo_close_rx: Mutex::new(servo_close_rx),
        atomic_fenced_lease_gen,
        servo_dispatcher: Arc::clone(&servo_dispatcher),
        hub: Arc::clone(&hub),
        offload_reclaim_rx: Mutex::new(offload_reclaim_rx),
    });

    let startup_bound = hooks
        .panic_disable_timeout
        .unwrap_or(crate::supervisor::PANIC_DISABLE_TIMEOUT);
    let startup_deadline = Instant::now() + startup_bound;

    // Spawn dedicated persistence worker first so its thread handle can be woken on job push.
    let persist_thread = match spawn_persist_worker(
        persist_job_rx,
        persist_cmd_tx,
        persist_rep_rx.clone(),
        watchdog_plan_tx,
        config_write_mutex,
        Arc::clone(&published_config),
        persist_stop.clone(),
        exited.clone(),
        persist_worker_exited.clone(),
        persist_exited_cv.clone(),
        hub.clone(),
        hooks.clone(),
    ) {
        Ok(jh) => jh,
        Err(e) => {
            return Err(SpawnExecutorError::Recoverable {
                core: Box::new(core),
                unsupported_reason: None,
                msg: format!("failed to spawn armd-persist: {e}"),
            });
        }
    };
    let persist_park_thread = persist_thread.thread().clone();

    let startup_gate = StartupGate::new();
    let (core_tx, core_rx) = mpsc::sync_channel::<SupervisorInner>(1);
    let estop_state_caller = Arc::clone(&estop_state);
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
        let stop_requests_ctrl = stop_requests.clone();
        let closed_bitset_ctrl = Arc::clone(&closed_bitset);
        let startup_gate_ctrl = startup_gate.clone();
        let spawn_res = if hooks.fail_control_spawn {
            Err(std::io::Error::other("simulated armd-control spawn failure"))
        } else {
            thread::Builder::new().name("armd-control".into()).spawn(move || {
                let _control_thread_guard = ControlThreadGuard::enter();
                let Ok(core) = core_rx.recv() else {
                    return;
                };
                control_thread_main(
                    core,
                    engine,
                    cfg_clone,
                    initial_generation,
                    stop_requests_ctrl,
                    closed_bitset_ctrl,
                    startup_gate_ctrl,
                    control_stop,
                    exited,
                    control_exited_cv,
                    teardown_disable,
                    stats,
                    control_sched,
                    control_period_us,
                    diag_tx,
                    diag_amend_tx,
                    maint_req_rx,
                    maint_rep_tx,
                    estop_ack_tx,
                    estop_overflow_gen,
                    reclaim_state_tx,
                    reclaim_stop_tx,
                    reclaim_traj_tx,
                    reclaim_macro_tx,
                    #[cfg(feature = "policy-runtime")]
                    reclaim_policy_tx,
                    reclaim_return_rx,
                    safety_log_tx,
                    shutdown_hold_error,
                    persist_job_tx,
                    persist_cmd_rx,
                    persist_rep_tx,
                    watchdog_plan_rx,
                    offload_reclaim_tx,
                    persist_wake,
                    estop_state,
                    hub_clone,
                    hooks,
                );
            })
        };
        match spawn_res {
            Ok(jh) => jh,
            Err(e) => {
                stop_and_join_persist_bounded(
                    &persist_stop,
                    &persist_park_thread,
                    &persist_exited_cv,
                    persist_thread,
                    startup_deadline,
                );
                return Err(SpawnExecutorError::Recoverable {
                    core: Box::new(core),
                    unsupported_reason: None,
                    msg: format!("failed to spawn armd-control: {e}"),
                });
            }
        }
    };
    if let Err(mpsc::SendError(core)) = core_tx.send(core) {
        stop_and_join_persist_bounded(
            &persist_stop,
            &persist_park_thread,
            &persist_exited_cv,
            persist_thread,
            startup_deadline,
        );
        if wait_control_exited_or_timed_out_until(
            &control_exited_cv,
            &teardown_disable,
            startup_deadline,
        ) {
            let _ = control_thread.join();
        }
        return Err(SpawnExecutorError::Recoverable {
            core: Box::new(core),
            unsupported_reason: None,
            msg: "armd-control exited before receiving core ownership".into(),
        });
    }

    let executor_thread_id = match startup_gate.wait_until(startup_deadline) {
        StartupWaitOutcome::Ready(tid) => tid,
        StartupWaitOutcome::Failed(err) => {
            stop_and_join_persist_bounded(
                &persist_stop,
                &persist_park_thread,
                &persist_exited_cv,
                persist_thread,
                startup_deadline,
            );
            if wait_control_exited_or_timed_out_until(
                &control_exited_cv,
                &teardown_disable,
                startup_deadline,
            ) {
                let _ = control_thread.join();
            }
            return Err(err);
        }
        StartupWaitOutcome::TimedOut => {
            // Driver stalled during preflight (`position_limits` / `supports_rt_executor`) or
            // startup init (`set_estop_gate` / `set_rt_mode`): latch E-stop immediately, detach
            // `armd-control` (late worker sees `TimedOut`, runs bounded disable, and drops `core`
            // exclusively on `armd-control`), and fail startup so `OWNER_FAILED` is committed.
            estop_state_caller.request_and_latch();
            exited.store(true, Ordering::Release);
            stop_and_join_persist_bounded(
                &persist_stop,
                &persist_park_thread,
                &persist_exited_cv,
                persist_thread,
                Instant::now() + Duration::from_millis(50),
            );
            drop(control_thread);
            return Err(SpawnExecutorError::Fatal("RT executor startup timed out".into()));
        }
    };

    // Attach executor ack consumer to EstopRegistry atomically before returning handle.
    estop_reg
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .attach_executor(estop_ack_rx, estop_overflow_gen, exited.clone());

    let monitor_thread = stats.spawn_monitor(control_stop.clone()).ok();

    let shared = Arc::new(RtExecutorShared {
        boot: cfg.boot,
        broker,
        reply_router,
        target_writer: target_writer_handle,
        servo_dispatcher,
        stop_requests,
        estop_state: estop_state_caller,
        stats,
        maint_client,
        urgent_stop_client,
        conn_lifecycle_client,
        broker_dispatcher,
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
        teardown_disable_timeout: startup_bound,
        persist_thread: Mutex::new(Some(persist_thread)),
        control_thread: Mutex::new(Some(control_thread)),
        monitor_thread: Mutex::new(monitor_thread),
        persist_park_thread,
        clock: hooks.clock.clone(),
        baseline_identity,
    });
    Ok((RtExecutorHandle { shared }, sinks))
}

/// Copy-only off-RT watchdog park preparation result pushed by `armd-persist` and installed on
/// `armd-control` with zero heap allocations and zero frees.
#[derive(Clone, Copy)]
struct WatchdogParkPlan {
    generation: u64,
    attempt: u8,
    plan: Result<super::requests::motion::PreparedMotionSlot, crate::safety::BoundedText<160>>,
}

/// Single production forwarding path from `core.safety`'s inline log ring into the bounded
/// downstream SPSC `safety_log_tx` ring (zero RT heap allocations, zero frees).
///
/// Converges BOTH freshly popped and retained `"Stopped hold output failed"` occurrences into a
/// single pending aggregate and enforces ONE active-sink delivery gate timestamped on the exact
/// driver clock right at acceptance (zero double delivery within 1 s, lossless across ring full).
#[doc(hidden)]
pub fn forward_deferred_safety_logs(
    core: &mut SupervisorInner,
    safety_log_tx: &mut Producer<SafetyLogEvent>,
    stats: &ExecutorStats,
    now: Instant,
) -> bool {
    let mut drained = false;
    while let Some(ev) = core.safety.pop_log_event() {
        match ev {
            SafetyLogEvent::StoppedHoldOutputFailed {
                partial,
                suppressed,
                message,
                logged_at,
            } => {
                core.hold_tx_error_suppressed = core
                    .hold_tx_error_suppressed
                    .saturating_add(1)
                    .saturating_add(suppressed);
                core.last_hold_tx_error_log = core.last_hold_tx_error_delivered;
                core.pending_hold_tx_error_retry = Some((partial, message, logged_at));
            }
            other => {
                if safety_log_tx.push(other).is_err() {
                    core.safety_log_ring_dropped = core.safety_log_ring_dropped.saturating_add(1);
                    stats.safety_log_dropped.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
        drained = true;
    }
    if core.hold_tx_error_suppressed > 0 {
        if let Some((partial, message, logged_at)) = core.pending_hold_tx_error_retry {
            let check_at = core.driver_clock_now_or(now);
            let stamp = check_at.max(logged_at);
            let window_ready = core
                .last_hold_tx_error_delivered
                .is_none_or(|d| stamp.saturating_duration_since(d) >= Duration::from_secs(1));
            if window_ready {
                let out_ev = SafetyLogEvent::StoppedHoldOutputFailed {
                    partial,
                    suppressed: core.hold_tx_error_suppressed - 1,
                    message,
                    logged_at,
                };
                if safety_log_tx.push(out_ev).is_ok() {
                    let delivered_at = core.driver_clock_now_or(stamp).max(stamp);
                    core.hold_tx_error_suppressed = 0;
                    core.pending_hold_tx_error_retry = None;
                    core.last_hold_tx_error_log = Some(delivered_at);
                    core.last_hold_tx_error_delivered = Some(delivered_at);
                    drained = true;
                }
            }
        }
    }
    drained
}

struct ControlState {
    shutdown_committed: bool,
    control_stop: Arc<AtomicBool>,
    stop_requests: StopRequests,
    estop_ack_tx: Producer<EstopAck>,
    estop_overflow_gen: Arc<AtomicU64>,
    reclaim_state_tx: Producer<Arc<SafetyState>>,
    reclaim_stop_tx: Producer<RetiredStopPayloads>,
    reclaim_traj_tx: Producer<crate::motion::ActiveTrajectory>,
    reclaim_macro_tx: Producer<crate::macro_action::MacroRun>,
    #[cfg(feature = "policy-runtime")]
    reclaim_policy_tx: Producer<crate::policy_runtime::PolicyRun>,
    reclaim_return_rx: Consumer<Arc<SafetyState>>,
    safety_log_tx: Producer<SafetyLogEvent>,
    shutdown_hold_error: Arc<Mutex<Option<SafetyLogEvent>>>,
    stats: Arc<ExecutorStats>,
    maint_req_rx: Consumer<MaintRequest>,
    maint_rep_tx: Producer<MaintReply>,
    /// Reply for a maintenance op serviced before Step 3 commit / Step 6 publish, held until
    /// strictly after Step 6 publication (or flushed immediately when `CommitShutdown` stops).
    pending_maint_reply: Option<MaintReply>,
    persist_job_tx: Producer<PendingZeroCheckPersist>,
    persist_cmd_rx: Consumer<PersistCmd>,
    persist_rep_tx: Producer<PersistReply>,
    /// Precomputed off-RT watchdog park plans (Copy-only, 0 alloc / 0 free on `armd-control`).
    watchdog_plan_rx: Consumer<WatchdogParkPlan>,
    last_woken_watchdog_park: Option<(u64, u8)>,
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

    /// Clean-exit E-stop drain: moves any remaining pending E-stop acks / retired fault arcs /
    /// retired stop payloads / retired trajectories / retired macro runs into their reclaim rings
    /// or lock-free `offload_reclaim_tx` so nothing drops on RT.
    fn drain_estop_and_retired_stopped(&mut self, core: &mut SupervisorInner, now: Instant) {
        self.emit_estop_ack_if_advanced(core);
        while let Some(ack) = core.pending_estop_acks.pop_front() {
            self.estop_overflow_gen.fetch_max(ack.last_gen, Ordering::Release);
            if let Err(rtrb::PushError::Full(state)) = self.reclaim_state_tx.push(ack.state) {
                let _ = self.offload_reclaim_tx.push(OffloadReclaim::SafetyState(state));
            }
        }
        self.drain_retired_states(core, now);
        while let Some(arc) = core.safety.pop_retired_state_arc() {
            if let Err(rtrb::PushError::Full(state)) = self.reclaim_state_tx.push(arc) {
                let _ = self.offload_reclaim_tx.push(OffloadReclaim::SafetyState(state));
            }
        }
        while let Some(retired) = core.pending_retired_stops.pop_front() {
            if let Err(rtrb::PushError::Full(retired)) = self.reclaim_stop_tx.push(retired) {
                let _ = self.offload_reclaim_tx.push(OffloadReclaim::RetiredStop(retired));
            }
        }
        while let Some(traj) = core.pop_retired_trajectory() {
            if let Err(rtrb::PushError::Full(traj)) = self.reclaim_traj_tx.push(traj) {
                let _ = self.offload_reclaim_tx.push(OffloadReclaim::ActiveTrajectory(traj));
            }
        }
        while let Some(run) = core.pop_retired_macro_run() {
            if let Err(rtrb::PushError::Full(run)) = self.reclaim_macro_tx.push(run) {
                let _ = self.offload_reclaim_tx.push(OffloadReclaim::MacroRun(run));
            }
        }
        #[cfg(feature = "policy-runtime")]
        while let Some(run) = core.pop_retired_policy_run() {
            if let Err(rtrb::PushError::Full(run)) = self.reclaim_policy_tx.push(run) {
                let _ = self.offload_reclaim_tx.push(OffloadReclaim::PolicyRun(run));
            }
        }
        while let Some(ctx) = core.pop_retired_zero_check_ctx() {
            let _ = self.offload_reclaim_tx.push(OffloadReclaim::ZeroCheckCtx(ctx));
        }
        if let Some(run) = core.retired_gravity_comp.take() {
            let _ = self.offload_reclaim_tx.push(OffloadReclaim::GravityCompRun(run));
        }
    }

    /// Moves retired fault `Arc<SafetyState>` snapshots, retired heap motion payloads, retired
    /// `ActiveTrajectory` / `MacroRun` instances, and deferred [`SafetyLogEvent`]s off
    /// `armd-control` into bounded SPSC rings, and replenishes clean recycled fault slots without
    /// allocating or dropping on `armd-control`.
    fn drain_retired_states(&mut self, core: &mut SupervisorInner, now: Instant) {
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
        while !self.reclaim_stop_tx.is_full() {
            let Some(retired) = core.pending_retired_stops.pop_front() else {
                break;
            };
            let _ = self.reclaim_stop_tx.push(retired);
            drained = true;
        }
        while !self.reclaim_traj_tx.is_full() {
            let Some(traj) = core.pop_retired_trajectory() else {
                break;
            };
            let _ = self.reclaim_traj_tx.push(traj);
            drained = true;
        }
        while !self.reclaim_macro_tx.is_full() {
            let Some(run) = core.pop_retired_macro_run() else {
                break;
            };
            let _ = self.reclaim_macro_tx.push(run);
            drained = true;
        }
        #[cfg(feature = "policy-runtime")]
        while !self.reclaim_policy_tx.is_full() {
            let Some(run) = core.pop_retired_policy_run() else {
                break;
            };
            let _ = self.reclaim_policy_tx.push(run);
            drained = true;
        }
        while !self.offload_reclaim_tx.is_full() {
            let Some(ctx) = core.pop_retired_zero_check_ctx() else {
                break;
            };
            let _ = self.offload_reclaim_tx.push(OffloadReclaim::ZeroCheckCtx(ctx));
            drained = true;
        }
        if !self.offload_reclaim_tx.is_full() {
            if let Some(run) = core.retired_gravity_comp.take() {
                let _ = self.offload_reclaim_tx.push(OffloadReclaim::GravityCompRun(run));
                drained = true;
            }
        }
        let lost_inline = core.safety.advance_log_dropped_forwarded();
        if lost_inline > 0 {
            self.stats.safety_log_dropped.fetch_add(lost_inline, Ordering::Relaxed);
            drained = true;
        }
        if forward_deferred_safety_logs(core, &mut self.safety_log_tx, &self.stats, now) {
            drained = true;
        }
        if drained {
            core.core.hub().wake();
        }
    }

    /// Teardown-only handoff (never called on RT ticks): transfers any retained `"Stopped hold
    /// output failed"` aggregate that could not fit into a full `safety_log_tx` ring to
    /// `OffThreadSinks` without ever blocking `armd-control` on a mutex (`try_lock` with a
    /// counted drop on contention).
    fn flush_pending_hold_error_for_shutdown(&mut self, core: &mut SupervisorInner) {
        if let Some(ev) = core.take_pending_hold_tx_error_for_shutdown() {
            let mut slot = match self.shutdown_hold_error.try_lock() {
                Ok(guard) => guard,
                Err(std::sync::TryLockError::Poisoned(p)) => p.into_inner(),
                Err(std::sync::TryLockError::WouldBlock) => {
                    core.safety_log_ring_dropped = core.safety_log_ring_dropped.saturating_add(1);
                    self.stats.safety_log_dropped.fetch_add(1, Ordering::Relaxed);
                    return;
                }
            };
            *slot = Some(match slot.take() {
                None => ev,
                Some(SafetyLogEvent::StoppedHoldOutputFailed { suppressed: prev_sup, .. }) => {
                    match ev {
                        SafetyLogEvent::StoppedHoldOutputFailed {
                            partial,
                            suppressed,
                            message,
                            logged_at,
                        } => SafetyLogEvent::StoppedHoldOutputFailed {
                            partial,
                            suppressed: suppressed.saturating_add(1).saturating_add(prev_sup),
                            message,
                            logged_at,
                        },
                        other => other,
                    }
                }
                Some(_) => ev,
            });
            core.core.hub().wake();
        }
    }

    /// On panic before publication: retire unpublished `pending_estop_acks` state `Arc`s
    /// off-thread WITHOUT emitting `EstopAck` (waiters receive `503 EXECUTOR_STOPPED` while
    /// `e_stop_latched` remains true).
    fn retire_unpublished_estop_acks(&mut self, core: &mut SupervisorInner) {
        let mut drained = false;
        while let Some(ack) = core.pending_estop_acks.pop_front() {
            if let Err(rtrb::PushError::Full(state)) = self.reclaim_state_tx.push(ack.state) {
                let _ = self.offload_reclaim_tx.push(OffloadReclaim::SafetyState(state));
            }
            drained = true;
        }
        core.estop_ack_overflow_gen = 0;
        if drained {
            core.core.hub().wake();
        }
    }

    fn service_watchdog_park_plan(&mut self, core: &mut SupervisorInner) {
        use super::requests::motion as req;
        const MAX_DRIFT_RETRIES: u8 = 5;
        let Ok(wp) = self.watchdog_plan_rx.pop() else { return; };
        if !core.watchdog_park_pending.is_some_and(|p| p.generation == wp.generation && p.attempt == wp.attempt) {
            return;
        }
        match wp.plan {
            Ok(slot) => match req::install_watchdog_park(core, wp.generation, &slot, None) {
                Ok(_) | Err(req::MotionInstallError::ContinuationMismatch) => {}
                Err(req::MotionInstallError::StartDrifted) => {
                    if let Some(p) = core.watchdog_park_pending.as_mut().filter(|p| p.generation == wp.generation) {
                        if p.attempt.saturating_add(1) < MAX_DRIFT_RETRIES {
                            p.attempt = p.attempt.saturating_add(1);
                            return;
                        }
                    }
                    req::fail_watchdog_park_plan(
                        core,
                        wp.generation,
                        crate::safety::BoundedText::new("watchdog park drifted repeatedly"),
                    );
                }
                Err(e) => {
                    req::fail_watchdog_park_plan(core, wp.generation, crate::safety::BoundedText::new(e.code()));
                }
            },
            Err(reason) => req::fail_watchdog_park_plan(core, wp.generation, reason),
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
        let mut returned_pending = None;
        let mut returned_samples = None;
        let mut returned_baseline = None;
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
                let clock_hook = self.hooks.clock.clone();
                let mut live_now = || clock_hook.as_ref().map_or_else(Instant::now, |c| c.now());
                let res = core.clear_fault_executor_bounded_for_release_with_clock(
                    ctx.now,
                    release_deadline,
                    expected_epoch,
                    &mut live_now,
                );
                if res.is_ok() {
                    *release_flags |= RELEASE_FLAG_CLEAR_OK;
                } else {
                    *release_flags |= RELEASE_FLAG_CLEAR_REFUSED;
                }
                self.stats.last_clear_release_seq.store(step_num, Ordering::Release);
                MaintenanceResult::FaultCleared(res)
            }
            MaintenanceOp::LeaseClearFault {
                connection,
                lease_generation,
                supplied_epoch,
                request_id_valid,
                body_error,
                auth_policy,
                operator,
            } => {
                let clock_hook = self.hooks.clock.clone();
                let mut live_now = || clock_hook.as_ref().map_or_else(Instant::now, |c| c.now());
                let out = super::requests::lease::install_lease_clear_fault_rt_with_clock(
                    core,
                    connection,
                    lease_generation,
                    supplied_epoch,
                    request_id_valid,
                    body_error,
                    auth_policy,
                    operator,
                    ctx.now,
                    release_deadline,
                    &mut live_now,
                );
                if matches!(out, super::requests::lease::LeaseClearFaultOutcome::Cleared { .. }) {
                    self.shutdown_committed = false;
                    *release_flags |= RELEASE_FLAG_CLEAR_OK;
                } else {
                    *release_flags |= RELEASE_FLAG_CLEAR_REFUSED;
                }
                self.stats.last_clear_release_seq.store(step_num, Ordering::Release);
                MaintenanceResult::LeaseFaultCleared(out)
            }
            MaintenanceOp::LeaseTorquePreflight {
                connection,
                lease_generation,
                supplied_epoch,
                request_id_valid,
                body_error,
                auth_policy,
                operator,
            } => {
                let res = super::requests::lease::check_torque_owner_preflight(
                    core,
                    connection,
                    lease_generation,
                    supplied_epoch,
                    request_id_valid,
                    body_error,
                    auth_policy,
                    operator,
                    ctx.now,
                );
                MaintenanceResult::LeaseTorquePreflight(res)
            }
            MaintenanceOp::SetJointZeroOffsets { offsets } => {
                returned_offsets = Some(offsets);
                MaintenanceResult::JointZeroOffsetsSet(Err(
                    "EXECUTOR_UNSUPPORTED: set_joint_zero_offsets is not available while the RT executor owns the arm; use the legacy owner",
                ))
            }
            MaintenanceOp::SetMotorZero { .. } => MaintenanceResult::MotorZeroSet(Err(
                "EXECUTOR_UNSUPPORTED: set_motor_zero is not available while the RT executor owns the arm; use the legacy owner",
            )),
            MaintenanceOp::TakeBaselinePending { fresh_pending, mut fresh_samples, clear_dirty } => {
                let moves = if core.gripper_baseline_pending.is_empty() {
                    returned_pending = Some(fresh_pending);
                    Vec::new()
                } else {
                    std::mem::replace(&mut core.gripper_baseline_pending, fresh_pending)
                };
                // Replenish preallocated `MoveSamples` spares without RT allocation or deallocation.
                while core.move_samples_spares.len() < core.move_samples_spares.capacity() {
                    let Some(s) = fresh_samples.pop() else { break; };
                    core.move_samples_spares.push(s);
                }
                returned_samples = Some(fresh_samples);
                let tool = &core.config.active_tool;
                let stroke = core
                    .config
                    .gripper_policy
                    .as_ref()
                    .map(|p| (p.closed_rad.min(p.open_rad), p.closed_rad.max(p.open_rad)));
                let current = core
                    .gripper_baseline
                    .as_ref()
                    .filter(|b| {
                        b.belongs_to(&tool.id, &tool.config_digest, core.follower.as_deref())
                            && stroke.is_some_and(|(lo, hi)| {
                                (b.lo_rad - lo).abs() < 1e-3 && (b.hi_rad - hi).abs() < 1e-3
                            })
                    })
                    .map(crate::gripper_baseline::FreeTorqueBaseline::data_snapshot);
                let dirty = core.gripper_baseline_dirty;
                // Only clear `dirty` here when draining an idle dirty flush (`moves.is_empty()`)
                // with a matching baseline to persist; non-empty `moves` clear `dirty` atomically
                // in `StoreBaseline` so transient maintenance/policy errors never drop dirty state.
                if moves.is_empty() && dirty && clear_dirty && current.is_some() {
                    core.gripper_baseline_dirty = false;
                }
                let policy = core.config.gripper_policy.as_ref().map(|p| {
                    (
                        p.closed_rad,
                        p.open_rad,
                        crate::gripper_calib::initial_max_nm(core),
                        crate::gripper_calib::calibration_initial_max_nm(core),
                    )
                });
                MaintenanceResult::BaselineTaken(BaselineTakeReply {
                    moves,
                    current,
                    policy,
                    generation: core.gripper_baseline_generation,
                    dirty,
                })
            }
            MaintenanceOp::StoreBaseline { baseline, expected_generation, mark_dirty } => {
                if core.gripper_baseline_generation == expected_generation {
                    returned_baseline = core.gripper_baseline.replace(baseline);
                    core.gripper_baseline_generation += 1;
                    core.gripper_baseline_dirty = mark_dirty;
                    MaintenanceResult::BaselineStored(true)
                } else {
                    returned_baseline = Some(baseline);
                    MaintenanceResult::BaselineStored(false)
                }
            }
            MaintenanceOp::MarkBaselineDirty => {
                core.gripper_baseline_dirty = true;
                MaintenanceResult::BaselineDirtyMarked
            }
            MaintenanceOp::Config(mut ex) => {
                super::requests::config::handle_config_exchange(core, ctx, &mut ex);
                MaintenanceResult::Config(ex)
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
            returned_pending,
            returned_samples,
            returned_baseline,
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
            let (result, returned_pending) = match reply.result {
                MaintenanceResult::BaselineTaken(taken) if !taken.moves.is_empty() => {
                    (MaintenanceResult::Stopped, Some(taken.moves))
                }
                MaintenanceResult::Config(ex) | MaintenanceResult::StoppedWithConfig(ex) => {
                    (MaintenanceResult::StoppedWithConfig(ex), reply.returned_pending)
                }
                _ => (MaintenanceResult::Stopped, reply.returned_pending),
            };
            self.push_maint_reply_or_offload(MaintReply {
                id: reply.id,
                result,
                returned_offsets: reply.returned_offsets,
                returned_probe: reply.returned_probe,
                returned_driver_config_error: reply.returned_driver_config_error,
                returned_pending,
                returned_samples: reply.returned_samples,
                returned_baseline: reply.returned_baseline,
            });
        }
        while let Ok(req) = self.maint_req_rx.pop() {
            let (result, returned_offsets, returned_probe, returned_pending, returned_samples, returned_baseline) = match req.op {
                MaintenanceOp::SetJointZeroOffsets { offsets, .. } => {
                    (MaintenanceResult::Stopped, Some(offsets), None, None, None, None)
                }
                MaintenanceOp::ProbeOwnedPayload { payload } => {
                    (MaintenanceResult::Stopped, None, Some(payload), None, None, None)
                }
                MaintenanceOp::TakeBaselinePending { fresh_pending, fresh_samples, .. } => {
                    (MaintenanceResult::Stopped, None, None, Some(fresh_pending), Some(fresh_samples), None)
                }
                MaintenanceOp::StoreBaseline { baseline, .. } => {
                    (MaintenanceResult::Stopped, None, None, None, None, Some(baseline))
                }
                MaintenanceOp::Config(ex) => {
                    (MaintenanceResult::StoppedWithConfig(ex), None, None, None, None, None)
                }
                _ => (MaintenanceResult::Stopped, None, None, None, None, None),
            };
            self.push_maint_reply_or_offload(MaintReply {
                id: req.id,
                result,
                returned_offsets,
                returned_probe,
                returned_driver_config_error: None,
                returned_pending,
                returned_samples,
                returned_baseline,
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
                PersistCmd::VerifyAuthority {
                    code_buf,
                    msg_buf,
                    ..
                } => PersistReply::Verified {
                    reservation: None,
                    _returned_code: Some(code_buf),
                    _returned_msg: Some(msg_buf),
                    _returned_probe: self.take_reply_probe(),
                },
                PersistCmd::Persisted {
                    code_buf,
                    msg_buf,
                    ..
                } => PersistReply::Persisted {
                    reservation: None,
                    _returned_code: Some(code_buf),
                    _returned_msg: Some(msg_buf),
                    _returned_probe: self.take_reply_probe(),
                },
                PersistCmd::DirSyncFailed {
                    warning,
                    unused_policy,
                    unused_jaw,
                    msg_buf,
                    ..
                } => PersistReply::Done {
                    _returned_policy: unused_policy,
                    _returned_jaw: unused_jaw,
                    _returned_code: None,
                    _returned_msg: Some(msg_buf),
                    _returned_warning: Some(warning),
                    _returned_probe: self.take_reply_probe(),
                },
                PersistCmd::Activate {
                    new_policy,
                    new_jaw,
                    code_buf,
                    msg_buf,
                    ..
                } => PersistReply::Done {
                    _returned_policy: Some(new_policy),
                    _returned_jaw: new_jaw,
                    _returned_code: Some(code_buf),
                    _returned_msg: Some(msg_buf),
                    _returned_warning: None,
                    _returned_probe: self.take_reply_probe(),
                },
                PersistCmd::FinishReconcile {
                    new_policy,
                    new_jaw,
                    ..
                } => PersistReply::Done {
                    _returned_policy: Some(new_policy),
                    _returned_jaw: new_jaw,
                    _returned_code: None,
                    _returned_msg: None,
                    _returned_warning: None,
                    _returned_probe: self.take_reply_probe(),
                },
                PersistCmd::LatchConfigReconcileFault { .. } => PersistReply::Done {
                    _returned_policy: None,
                    _returned_jaw: None,
                    _returned_code: None,
                    _returned_msg: None,
                    _returned_warning: None,
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
                estop_epoch,
                stop_gen,
                deadline,
                new_policy,
                jaw_opening,
                probe,
                code_buf,
                msg_buf,
            } => {
                self.next_persist_req += 1;
                let req_seq = self.next_persist_req;
                match super::requests::config::zero_check_owner_reserve(
                    core,
                    ctx,
                    action_id,
                    base_revision,
                    lease_owner,
                    estop_epoch,
                    stop_gen,
                    deadline,
                    &new_policy,
                    req_seq,
                    Some(code_buf),
                    Some(msg_buf),
                ) {
                    Ok((reservation, proposal, auth, ret_code, ret_msg)) => {
                        self.persist_granted = Some((action_id, ConfigDomain::GripperCalibration));
                        self.push_persist_reply_or_retain(
                            PersistReply::Reserved {
                                reservation: Some((reservation, proposal, auth)),
                                policy: Some(new_policy),
                                returned_jaw: jaw_opening,
                                _returned_code: ret_code,
                                _returned_msg: ret_msg,
                                _returned_probe: probe,
                            },
                            core,
                        );
                    }
                    Err((old_code, old_msg)) => {
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
            PersistCmd::VerifyAuthority {
                action_id,
                reservation,
                proposal,
                auth,
                base_revision,
                code_buf,
                msg_buf,
            } => {
                let probe = self.take_reply_probe();
                match super::requests::config::zero_check_owner_verify_authority(
                    core,
                    ctx,
                    action_id,
                    reservation,
                    &proposal,
                    &auth,
                    base_revision,
                    Some(code_buf),
                    Some(msg_buf),
                ) {
                    Ok((reservation, ret_code, ret_msg)) => {
                        self.push_persist_reply_or_retain(
                            PersistReply::Verified {
                                reservation: Some(reservation),
                                _returned_code: ret_code,
                                _returned_msg: ret_msg,
                                _returned_probe: probe,
                            },
                            core,
                        );
                    }
                    Err((old_code, old_msg)) => {
                        self.persist_granted = None;
                        self.push_persist_reply_or_retain(
                            PersistReply::Verified {
                                reservation: None,
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
            PersistCmd::Persisted {
                action_id,
                reservation,
                code_buf,
                msg_buf,
            } => {
                let simulated_err = self
                    .hooks
                    .fail_persisted_once
                    .as_ref()
                    .is_some_and(|f| f.swap(false, Ordering::AcqRel));
                let probe = self.take_reply_probe();
                match super::requests::config::zero_check_owner_persisted(
                    core,
                    action_id,
                    reservation,
                    simulated_err,
                    Some(code_buf),
                    Some(msg_buf),
                ) {
                    Ok((reservation, ret_code, ret_msg)) => {
                        self.push_persist_reply_or_retain(
                            PersistReply::Persisted {
                                reservation: Some(reservation),
                                _returned_code: ret_code,
                                _returned_msg: ret_msg,
                                _returned_probe: probe,
                            },
                            core,
                        );
                    }
                    Err((old_code, old_msg)) => {
                        self.persist_granted = None;
                        self.push_persist_reply_or_retain(
                            PersistReply::Persisted {
                                reservation: None,
                                _returned_code: old_code,
                                _returned_msg: old_msg,
                                _returned_probe: probe,
                            },
                            core,
                        );
                    }
                }
            }
            PersistCmd::DirSyncFailed {
                action_id,
                reservation,
                warning,
                unused_policy,
                unused_jaw,
                msg_buf,
            } => {
                self.persist_granted = None;
                let (old_msg, old_warn) =
                    super::requests::config::zero_check_owner_reconcile_dir_sync_failed(
                        core,
                        action_id,
                        reservation,
                        warning,
                        Some(msg_buf),
                    );
                let probe = self.take_reply_probe();
                self.push_persist_reply_or_retain(
                    PersistReply::Done {
                        _returned_policy: unused_policy,
                        _returned_jaw: unused_jaw,
                        _returned_code: None,
                        _returned_msg: old_msg,
                        _returned_warning: old_warn,
                        _returned_probe: probe,
                    },
                    core,
                );
            }
            PersistCmd::Activate {
                action_id,
                reservation,
                proposal,
                auth,
                base_revision,
                offset_rad,
                new_policy,
                new_jaw,
                operator_revoked,
                code_buf,
                msg_buf,
            } => {
                self.persist_granted = None;
                let probe = self.take_reply_probe();
                match super::requests::config::zero_check_owner_activate(
                    core,
                    ctx,
                    action_id,
                    reservation,
                    &proposal,
                    &auth,
                    base_revision,
                    offset_rad,
                    new_policy,
                    new_jaw,
                    operator_revoked,
                    Some(code_buf),
                    Some(msg_buf),
                ) {
                    Ok((old_policy, old_jaw, ret_code, ret_msg)) => {
                        let activated_revision = core.gripper_calib_revision;
                        self.push_persist_reply_or_retain(
                            PersistReply::Activated {
                                activated_revision,
                                _returned_policy: old_policy,
                                _returned_jaw: old_jaw,
                                _returned_code: ret_code,
                                _returned_msg: ret_msg,
                                _returned_warning: None,
                                _returned_probe: probe,
                            },
                            core,
                        );
                    }
                    Err((ret_policy, ret_jaw, old_code, old_msg)) => {
                        self.push_persist_reply_or_retain(
                            PersistReply::Done {
                                _returned_policy: ret_policy,
                                _returned_jaw: ret_jaw,
                                _returned_code: old_code,
                                _returned_msg: old_msg,
                                _returned_warning: None,
                                _returned_probe: probe,
                            },
                            core,
                        );
                    }
                }
            }
            PersistCmd::FinishReconcile {
                proposal,
                auth,
                base_revision,
                offset_rad,
                new_policy,
                new_jaw,
                operator_revoked,
            } => {
                let probe = self.take_reply_probe();
                match super::requests::config::zero_check_owner_finish_reconcile(
                    core,
                    ctx,
                    &proposal,
                    &auth,
                    base_revision,
                    offset_rad,
                    new_policy,
                    new_jaw,
                    operator_revoked,
                ) {
                    Ok((old_policy, old_jaw)) => {
                        let activated_revision = core.gripper_calib_revision;
                        self.push_persist_reply_or_retain(
                            PersistReply::Activated {
                                activated_revision,
                                _returned_policy: old_policy,
                                _returned_jaw: old_jaw,
                                _returned_code: None,
                                _returned_msg: None,
                                _returned_warning: None,
                                _returned_probe: probe,
                            },
                            core,
                        );
                    }
                    Err((_code, ret_policy, ret_jaw)) => {
                        self.push_persist_reply_or_retain(
                            PersistReply::Done {
                                _returned_policy: ret_policy,
                                _returned_jaw: ret_jaw,
                                _returned_code: None,
                                _returned_msg: None,
                                _returned_warning: None,
                                _returned_probe: probe,
                            },
                            core,
                        );
                    }
                }
            }
            PersistCmd::LatchConfigReconcileFault { message } => {
                super::requests::config::install_latch_config_reconcile_fault(
                    core,
                    ConfigDomain::GripperCalibration,
                    message,
                );
                let probe = self.take_reply_probe();
                self.push_persist_reply_or_retain(
                    PersistReply::Done {
                        _returned_policy: None,
                        _returned_jaw: None,
                        _returned_code: None,
                        _returned_msg: None,
                        _returned_warning: None,
                        _returned_probe: probe,
                    },
                    core,
                );
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn spawn_persist_worker(
    mut job_rx: Consumer<PendingZeroCheckPersist>,
    mut cmd_tx: Producer<PersistCmd>,
    rep_rx: Arc<Mutex<Consumer<PersistReply>>>,
    mut watchdog_plan_tx: Producer<WatchdogParkPlan>,
    config_write_mutex: Arc<tokio::sync::Mutex<()>>,
    published_config: Arc<super::requests::config::PublishedConfigHub>,
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
        let hub_watch = Arc::clone(&hub);
        let hub_for_refresh = Arc::clone(&hub);
        let _exit_guard = ExitFlag(persist_worker_exited, persist_exited_cv, hub);
        let mut last_sent_watchdog_park: Option<(u64, u8)> = None;
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
            if let Some(snap) = hub_watch.latest() {
                let wd_gen = snap.rt.watchdog_park_pending_gen;
                if wd_gen != 0 {
                    let key = (wd_gen, snap.rt.watchdog_park_pending_attempt);
                    if last_sent_watchdog_park != Some(key) {
                        let plan = match hub_watch.motion_config().and_then(|cfg| {
                            super::requests::motion::MotionPlanSnapshot::from_published(&snap, cfg)
                        }) {
                            Some(plan_snap) => match super::requests::motion::prepare_motion(
                                &plan_snap,
                                &super::requests::motion::MotionRequest::park(),
                            ) {
                                Ok(p) => Ok(p.slot),
                                Err(e) => {
                                    let mut msg = crate::safety::BoundedText::<160>::empty();
                                    let _ = core::fmt::write(&mut msg, format_args!("watchdog park could not be planned: {e}"));
                                    Err(msg)
                                }
                            },
                            None => Err(crate::safety::BoundedText::new("watchdog park snapshot or motion config unavailable")),
                        };
                        if watchdog_plan_tx.push(WatchdogParkPlan { generation: key.0, attempt: key.1, plan }).is_ok() {
                            last_sent_watchdog_park = Some(key);
                        }
                    }
                }
            }
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
                .ctx
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
                        unused_jaw: job.ctx.jaw_opening,
                    },
                    &persist_stop,
                    &control_exited,
                );
                let _ = wait_reply(&rep_rx, &persist_stop, &control_exited);
                continue;
            }
            // Validate, build candidate GripperPolicy, and serialize JSON completely off `armd-control`.
            let candidate_policy = match SupervisorInner::validate_gripper_calibration_off_thread(
                job.ctx.base_gripper_policy.as_ref(),
                job.ctx.jaw_opening.as_ref(),
                job.ctx.gripper_limit.as_ref(),
                job.ctx.active_tool_is_stock,
                Some(job.offset_rad),
                job.ctx.jaw_opening.as_ref(),
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
                            unused_jaw: job.ctx.jaw_opening,
                        },
                        &persist_stop,
                        &control_exited,
                    );
                    let _ = wait_reply(&rep_rx, &persist_stop, &control_exited);
                    if hooks.exit_after_first_job { break; }
                    continue;
                }
            };
            let mut backend = ExecutorPersistBackend {
                cmd_tx: &mut cmd_tx,
                rep_rx: &rep_rx,
                config_write_mutex: &config_write_mutex,
                cw_guard: Some(_cw),
                published_config: &published_config,
                hub: &hub_for_refresh,
                persist_stop: &persist_stop,
                control_exited: &control_exited,
                hooks: &hooks,
                job,
                candidate_policy: Some(candidate_policy),
                new_policy: None,
                new_jaw: None,
                reservation: None,
                proposal: None,
                auth: None,
            };
            let res = super::requests::config::run_common_commit_sync(&mut backend);
            if matches!(res, Err(super::requests::config::ZeroCheckCommitStop::WorkerDisconnected))
                || hooks.exit_after_first_job
            {
                break;
            }
        }
    })
}

struct ExecutorPersistBackend<'a> {
    cmd_tx: &'a mut Producer<PersistCmd>,
    rep_rx: &'a Arc<Mutex<Consumer<PersistReply>>>,
    config_write_mutex: &'a Arc<tokio::sync::Mutex<()>>,
    cw_guard: Option<tokio::sync::MutexGuard<'a, ()>>,
    published_config: &'a Arc<super::requests::config::PublishedConfigHub>,
    hub: &'a Arc<StateHub>,
    persist_stop: &'a Arc<AtomicBool>,
    control_exited: &'a Arc<AtomicBool>,
    hooks: &'a PersistTestHooks,
    job: PendingZeroCheckPersist,
    candidate_policy: Option<GripperPolicy>,
    new_policy: Option<GripperPolicy>,
    new_jaw: Option<JawOpening>,
    reservation: Option<Reservation>,
    proposal: Option<ConfigProposal>,
    auth: Option<super::requests::config::ConfigAuthContext>,
}

impl ExecutorPersistBackend<'_> {
    fn push_cmd(&mut self, cmd: PersistCmd) -> bool {
        let mut pending = Some(cmd);
        while let Some(c) = pending.take() {
            if self.persist_stop.load(Ordering::Acquire)
                || self.control_exited.load(Ordering::Acquire)
                || self.cmd_tx.is_abandoned()
            {
                return false;
            }
            match self.cmd_tx.push(c) {
                Ok(()) => return true,
                Err(rtrb::PushError::Full(c)) => {
                    pending = Some(c);
                    thread::park_timeout(Duration::from_millis(1));
                }
            }
        }
        true
    }

    fn wait_reply(&self) -> Option<PersistReply> {
        loop {
            if let Ok(rep) = self.rep_rx.lock().unwrap_or_else(|e| e.into_inner()).pop() {
                return Some(rep);
            }
            if self.persist_stop.load(Ordering::Acquire) || self.control_exited.load(Ordering::Acquire) {
                return None;
            }
            thread::park_timeout(Duration::from_millis(1));
        }
    }

    fn op_auth_ok(&self) -> bool {
        self.job
            .ctx
            .operator_auth
            .as_ref()
            .is_none_or(|(a, tok)| !a.is_enforced() || a.is_operator(tok.as_deref()))
    }
}

impl super::requests::config::CommonCommitWorker for ExecutorPersistBackend<'_> {
    type Ok = ();
    type Err = super::requests::config::ZeroCheckCommitStop;
    const RUN_BLOCKING_ON_TOKIO: bool = false;

    fn dir_sync_override(&self) -> Option<super::requests::config::DirSyncOverride> {
        self.hooks.dir_sync_override.clone()
    }

    async fn reserve(&mut self) -> Result<(), Self::Err> {
        use super::requests::config::ZeroCheckCommitStop;
        let new_policy = self
            .candidate_policy
            .take()
            .expect("candidate_policy validated before reserve");
        let payload_probe = self
            .hooks
            .payload_drop_probe
            .as_ref()
            .and_then(|p| p.lock().ok().and_then(|mut g| g.take()));
        if !self.push_cmd(PersistCmd::Reserve {
            action_id: self.job.action_id,
            base_revision: self.job.base_revision,
            lease_owner: self.job.lease_owner,
            estop_epoch: self.job.estop_epoch,
            stop_gen: self.job.stop_gen,
            deadline: self.job.deadline,
            new_policy,
            jaw_opening: self.job.ctx.jaw_opening.clone(),
            probe: payload_probe,
            code_buf: String::with_capacity(64),
            msg_buf: String::with_capacity(160),
        }) {
            return Err(ZeroCheckCommitStop::WorkerDisconnected);
        }
        let Some(PersistReply::Reserved {
            reservation,
            policy,
            returned_jaw,
            _returned_code,
            _returned_msg,
            _returned_probe,
        }) = self.wait_reply()
        else {
            return Err(ZeroCheckCommitStop::WorkerDisconnected);
        };
        drop(_returned_code);
        drop(_returned_msg);
        drop(_returned_probe);
        match (reservation, policy) {
            (Some((r, p, a)), Some(pol)) => {
                self.reservation = Some(r);
                self.proposal = Some(p);
                self.auth = Some(a);
                self.new_policy = Some(pol);
                self.new_jaw = returned_jaw;
                Ok(())
            }
            (_, returned_policy) => {
                drop(returned_policy);
                drop(returned_jaw);
                Err(ZeroCheckCommitStop::Aborted)
            }
        }
    }

    async fn prepare_write_after_reserve(
        &mut self,
    ) -> Result<Option<crate::persist::PendingWrite>, Self::Err> {
        Ok(Some(self.job.ctx.serialize_write(self.job.offset_rad)))
    }

    fn after_reserve_before_rename(&mut self) -> bool {
        self.hooks
            .after_grant_before_rename
            .as_ref()
            .is_some_and(|h| h())
    }

    async fn verify_authority_before_rename(&mut self) -> Result<(), Self::Err> {
        use super::requests::config::ZeroCheckCommitStop;
        if !self.op_auth_ok() {
            if let Some(reservation) = self.reservation.take() {
                let unused_policy = self.new_policy.take();
                let unused_jaw = self.new_jaw.take();
                let _ = self.push_cmd(PersistCmd::Abort {
                    action_id: self.job.action_id,
                    reservation,
                    code: "OPERATOR_REQUIRED".into(),
                    message: "Operator session expired or locked before gripper calibration persisted"
                        .into(),
                    unused_policy,
                    unused_jaw,
                });
                if !self.hooks.leave_final_reply_queued {
                    let _ = self.wait_reply();
                }
            }
            return Err(ZeroCheckCommitStop::Aborted);
        }
        let reservation = self.reservation.take().expect("reservation held");
        let proposal = self.proposal.expect("proposal held");
        let auth = self.auth.expect("auth held");
        if !self.push_cmd(PersistCmd::VerifyAuthority {
            action_id: self.job.action_id,
            reservation,
            proposal,
            auth,
            base_revision: self.job.base_revision,
            code_buf: String::with_capacity(64),
            msg_buf: String::with_capacity(160),
        }) {
            return Err(ZeroCheckCommitStop::WorkerDisconnected);
        }
        match self.wait_reply() {
            Some(PersistReply::Verified { reservation: Some(r), .. }) => {
                self.reservation = Some(r);
                Ok(())
            }
            Some(_) => {
                drop(self.new_policy.take());
                drop(self.new_jaw.take());
                Err(ZeroCheckCommitStop::Aborted)
            }
            None => {
                drop(self.new_policy.take());
                drop(self.new_jaw.take());
                Err(ZeroCheckCommitStop::WorkerDisconnected)
            }
        }
    }

    async fn mark_persisted(&mut self) -> Result<(), Self::Err> {
        use super::requests::config::ZeroCheckCommitStop;
        let reservation = self.reservation.take().expect("reservation held");
        if !self.push_cmd(PersistCmd::Persisted {
            action_id: self.job.action_id,
            reservation,
            code_buf: String::with_capacity(64),
            msg_buf: String::with_capacity(160),
        }) {
            return Err(ZeroCheckCommitStop::WorkerDisconnected);
        }
        match self.wait_reply() {
            Some(PersistReply::Persisted { reservation: Some(r), .. }) => {
                self.reservation = Some(r);
                Ok(())
            }
            Some(_) => {
                drop(self.new_policy.take());
                drop(self.new_jaw.take());
                Err(ZeroCheckCommitStop::Aborted)
            }
            None => {
                drop(self.new_policy.take());
                drop(self.new_jaw.take());
                Err(ZeroCheckCommitStop::WorkerDisconnected)
            }
        }
    }

    async fn reconcile_after_dir_sync_failure(
        &mut self,
        target_path: std::path::PathBuf,
        warn: String,
    ) -> Self::Err {
        use super::requests::config::{
            ZeroCheckCommitStop, DEFAULT_RECONCILE_MAX_ATTEMPTS, DEFAULT_RECONCILE_RETRY_INTERVAL,
        };
        if let Some(reservation) = self.reservation.take() {
            let keep_for_retry = !self.hooks.leave_final_reply_queued && !self.hooks.exit_after_first_job;
            let (unused_policy, unused_jaw) = if keep_for_retry {
                (None, None)
            } else {
                (self.new_policy.take(), self.new_jaw.take())
            };
            let _ = self.push_cmd(PersistCmd::DirSyncFailed {
                action_id: self.job.action_id,
                reservation,
                warning: warn.clone(),
                unused_policy,
                unused_jaw,
                msg_buf: String::with_capacity(160),
            });
            if !self.hooks.leave_final_reply_queued {
                let _ = self.wait_reply();
            }
            if keep_for_retry {
                if let (Some(proposal), Some(auth), Some(new_policy)) = (
                    self.proposal,
                    self.auth,
                    self.new_policy.take(),
                ) {
                    let new_jaw = self.new_jaw.take();
                    let interval = self
                        .hooks
                        .reconcile_retry_interval
                        .unwrap_or(DEFAULT_RECONCILE_RETRY_INTERVAL);
                    let max_attempts = self
                        .hooks
                        .reconcile_max_attempts
                        .unwrap_or(DEFAULT_RECONCILE_MAX_ATTEMPTS)
                        .max(1);
                    let file_name = target_path
                        .file_name()
                        .map(|n| n.to_string_lossy().into_owned())
                        .unwrap_or_else(|| target_path.display().to_string());
                    let mut last_warn = warn;
                    let mut policy_opt = Some(new_policy);
                    let mut jaw_opt = new_jaw;
                    // Release `config_write_mutex` during reconciliation retry backoff so
                    // concurrent config writes immediately observe `HoldState::Reconciling`
                    // (`CONFIG_BUSY`) rather than stalling behind the sleep.
                    drop(self.cw_guard.take());
                    for attempt in 1..=max_attempts {
                        let wake_at = Instant::now() + interval;
                        while Instant::now() < wake_at {
                            if self.persist_stop.load(Ordering::Acquire)
                                || self.control_exited.load(Ordering::Acquire)
                            {
                                return ZeroCheckCommitStop::WorkerDisconnected;
                            }
                            let rem = wake_at.saturating_duration_since(Instant::now());
                            thread::park_timeout(rem.min(Duration::from_millis(5)));
                        }
                        if self.persist_stop.load(Ordering::Acquire)
                            || self.control_exited.load(Ordering::Acquire)
                        {
                            return ZeroCheckCommitStop::WorkerDisconnected;
                        }
                        let pds = crate::persist::PendingDirSync::for_path(&target_path);
                        let outcome = match self.hooks.dir_sync_override.as_ref() {
                            Some(f) => pds.sync_with(|dir| f(dir)),
                            None => pds.sync_parent_dir(),
                        };
                        match outcome {
                            crate::persist::PublishOutcome::Published => {}
                            crate::persist::PublishOutcome::PublishedDurabilityUnconfirmed(w) => {
                                last_warn = w;
                                tracing::warn!(
                                    "Executor zero_check reconciliation retry {attempt}/{max_attempts} for {file_name} failed dir fsync: {last_warn}"
                                );
                                continue;
                            }
                        }
                        let _cw_reconcile = loop {
                            if self.persist_stop.load(Ordering::Acquire)
                                || self.control_exited.load(Ordering::Acquire)
                            {
                                return ZeroCheckCommitStop::WorkerDisconnected;
                            }
                            match self.config_write_mutex.try_lock() {
                                Ok(guard) => break guard,
                                Err(_) => thread::park_timeout(Duration::from_millis(2)),
                            }
                        };
                        let operator_revoked = !self.op_auth_ok();
                        let Some(pol) = policy_opt.take() else {
                            return ZeroCheckCommitStop::Aborted;
                        };
                        let pub_pol = pol.clone();
                        let jaw = jaw_opt.take();
                        let pub_jaw = jaw.clone();
                        if !self.push_cmd(PersistCmd::FinishReconcile {
                            proposal,
                            auth,
                            base_revision: self.job.base_revision,
                            offset_rad: self.job.offset_rad,
                            new_policy: pol,
                            new_jaw: jaw,
                            operator_revoked,
                        }) {
                            return ZeroCheckCommitStop::WorkerDisconnected;
                        }
                        match self.wait_reply() {
                            Some(PersistReply::Activated { activated_revision, .. }) => {
                                let prep_pol = pub_pol.clone();
                                self.published_config.apply_zero_check_commit(
                                    self.job.offset_rad,
                                    pub_pol,
                                    pub_jaw,
                                    activated_revision,
                                );
                                if let Some(cur) = self.hub.prep_env() {
                                    let mut next = (*cur).clone();
                                    next.config.gripper_policy = Some(prep_pol);
                                    next.gripper_calib_revision = activated_revision;
                                    self.hub.update_prep_env(Arc::new(next));
                                }
                                tracing::info!(
                                    "Executor zero_check reconciliation succeeded for {file_name} on retry {attempt}/{max_attempts}"
                                );
                                return ZeroCheckCommitStop::Aborted;
                            }
                            Some(_) => {
                                let msg = format!(
                                    "Configuration persistence failed for {file_name}: post-rename reconciliation rejected"
                                );
                                tracing::error!("{msg}");
                                if self.push_cmd(PersistCmd::LatchConfigReconcileFault {
                                    message: crate::safety::BoundedText::new(&msg),
                                }) {
                                    let _ = self.wait_reply();
                                }
                                return ZeroCheckCommitStop::Aborted;
                            }
                            None => return ZeroCheckCommitStop::WorkerDisconnected,
                        }
                    }
                    let msg = format!(
                        "Configuration persistence failed for {file_name} after {max_attempts} reconciliation attempts: {last_warn}"
                    );
                    tracing::error!("{msg}");
                    if self.push_cmd(PersistCmd::LatchConfigReconcileFault {
                        message: crate::safety::BoundedText::new(&msg),
                    }) {
                        let _ = self.wait_reply();
                    }
                }
            }
        }
        ZeroCheckCommitStop::Aborted
    }

    fn after_rename_before_activate(&mut self) -> bool {
        self.hooks
            .after_rename_before_activate
            .as_ref()
            .is_some_and(|h| h())
    }

    async fn activate_and_publish(&mut self, _disk_published: bool) -> Result<Self::Ok, Self::Err> {
        let operator_revoked = !self.op_auth_ok();
        let reservation = self.reservation.take().expect("reservation held");
        let proposal = self.proposal.expect("proposal held");
        let auth = self.auth.expect("auth held");
        let new_policy = self.new_policy.take().expect("new_policy held");
        let new_jaw = self.new_jaw.take();
        let pub_policy = new_policy.clone();
        let pub_jaw = new_jaw.clone();
        let _ = self.push_cmd(PersistCmd::Activate {
            action_id: self.job.action_id,
            reservation,
            proposal,
            auth,
            base_revision: self.job.base_revision,
            offset_rad: self.job.offset_rad,
            new_policy,
            new_jaw,
            operator_revoked,
            code_buf: String::with_capacity(64),
            msg_buf: String::with_capacity(160),
        });
        if !self.hooks.leave_final_reply_queued {
            if let Some(PersistReply::Activated { activated_revision, .. }) = self.wait_reply() {
                let prep_pol = pub_policy.clone();
                self.published_config.apply_zero_check_commit(
                    self.job.offset_rad,
                    pub_policy,
                    pub_jaw,
                    activated_revision,
                );
                if let Some(cur) = self.hub.prep_env() {
                    let mut next = (*cur).clone();
                    next.config.gripper_policy = Some(prep_pol);
                    next.gripper_calib_revision = activated_revision;
                    self.hub.update_prep_env(Arc::new(next));
                }
            }
        }
        Ok(())
    }

    async fn release_pre_rename(&mut self) {
        // Owner already released the reservation inside VerifyAuthority / Abort if taken.
        drop(self.new_policy.take());
        drop(self.new_jaw.take());
    }

    async fn abort_pre_rename_fs(&mut self, phase: &'static str, err: String) -> Self::Err {
        use super::requests::config::ZeroCheckCommitStop;
        if let Some(reservation) = self.reservation.take() {
            let unused_policy = self.new_policy.take();
            let unused_jaw = self.new_jaw.take();
            let _ = self.push_cmd(PersistCmd::Abort {
                action_id: self.job.action_id,
                reservation,
                code: "PERSIST_FAILED".into(),
                message: format!("Failed to {phase} gripper calibration: {err}"),
                unused_policy,
                unused_jaw,
            });
            if !self.hooks.leave_final_reply_queued {
                let _ = self.wait_reply();
            }
        }
        ZeroCheckCommitStop::Aborted
    }

    async fn reconcile_rename_join_error(&mut self, _err: String) -> Self::Err {
        super::requests::config::ZeroCheckCommitStop::Aborted
    }

    fn worker_disconnected_err(&mut self) -> Self::Err {
        super::requests::config::ZeroCheckCommitStop::WorkerDisconnected
    }
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
            // Note: `completed_releases` is Release-incremented at the single release-completion
            // point after Window 2, diagnostic enqueue, tail probe, and overrun accounting.
        }
    }
}

enum StartupSlot {
    Pending,
    Ready(ThreadId),
    Failed(SpawnExecutorError),
    TimedOut,
    Taken,
}

enum StartupWaitOutcome {
    Ready(ThreadId),
    Failed(SpawnExecutorError),
    TimedOut,
}

/// Single-authority synchronized startup slot carrying BOTH the readiness decision AND any
/// rollback payload atomically under one mutex/condvar (zero split publication).
#[derive(Clone)]
struct StartupGate(Arc<(Mutex<StartupSlot>, Condvar)>);

impl StartupGate {
    fn new() -> Self {
        Self(Arc::new((Mutex::new(StartupSlot::Pending), Condvar::new())))
    }

    fn is_timed_out(&self) -> bool {
        matches!(
            *self.0 .0.lock().unwrap_or_else(|e| e.into_inner()),
            StartupSlot::TimedOut
        )
    }

    fn try_settle_ready(&self, tid: ThreadId) -> bool {
        let mut g = self.0 .0.lock().unwrap_or_else(|e| e.into_inner());
        if !matches!(*g, StartupSlot::Pending) {
            return false;
        }
        *g = StartupSlot::Ready(tid);
        self.0 .1.notify_all();
        true
    }

    /// Attempts to publish `err` as the single startup outcome. If the caller already timed out,
    /// returns any rejected recoverable `core` BACK to `armd-control` so the worker can run
    /// bounded teardown disable before dropping the driver.
    fn try_settle_failed(&self, err: SpawnExecutorError) -> Option<Box<SupervisorInner>> {
        let mut g = self.0 .0.lock().unwrap_or_else(|e| e.into_inner());
        if !matches!(*g, StartupSlot::Pending) {
            return match err {
                SpawnExecutorError::Recoverable { core, .. } => Some(core),
                SpawnExecutorError::Fatal(_) => None,
            };
        }
        *g = StartupSlot::Failed(err);
        self.0 .1.notify_all();
        None
    }

    fn wait_until(&self, deadline: Instant) -> StartupWaitOutcome {
        let mut g = self.0 .0.lock().unwrap_or_else(|e| e.into_inner());
        while matches!(*g, StartupSlot::Pending) {
            let rem = deadline.saturating_duration_since(Instant::now());
            if rem.is_zero() {
                *g = StartupSlot::TimedOut;
                return StartupWaitOutcome::TimedOut;
            }
            let (next, res) = self.0 .1.wait_timeout(g, rem).unwrap_or_else(|e| e.into_inner());
            g = next;
            if matches!(*g, StartupSlot::Pending) && res.timed_out() {
                *g = StartupSlot::TimedOut;
                return StartupWaitOutcome::TimedOut;
            }
        }
        match std::mem::replace(&mut *g, StartupSlot::Taken) {
            StartupSlot::Ready(tid) => StartupWaitOutcome::Ready(tid),
            StartupSlot::Failed(err) => StartupWaitOutcome::Failed(err),
            StartupSlot::TimedOut | StartupSlot::Pending | StartupSlot::Taken => {
                StartupWaitOutcome::TimedOut
            }
        }
    }
}

/// RAII fallback guard on `armd-control`: if startup exits or unwinds unexpectedly before
/// settling `StartupGate`, latches shared E-stop BEFORE publishing terminal failure.
struct StartupAbortGuard {
    gate: StartupGate,
    estop_state: Arc<crate::rt_core::urgent::EstopState>,
    control_exited_cv: Arc<(Mutex<ControlExitState>, Condvar)>,
    exited: Arc<AtomicBool>,
    hub: Arc<StateHub>,
}

impl Drop for StartupAbortGuard {
    fn drop(&mut self) {
        let still_pending = matches!(
            *self.gate.0 .0.lock().unwrap_or_else(|e| e.into_inner()),
            StartupSlot::Pending
        );
        if still_pending {
            self.estop_state.request_and_latch();
            let _ = self.gate.try_settle_failed(SpawnExecutorError::Fatal(
                "armd-control exited unexpectedly before signaling readiness".into(),
            ));
            self.exited.store(true, Ordering::Release);
            {
                let mut st = self
                    .control_exited_cv
                    .0
                    .lock()
                    .unwrap_or_else(|e| e.into_inner());
                st.dropping = true;
                st.exited = true;
            }
            self.control_exited_cv.1.notify_all();
            self.hub.wake();
        }
    }
}

fn wait_bool_cv_until(cv: &(Mutex<bool>, Condvar), deadline: Instant) -> bool {
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

fn stop_and_join_persist_bounded(
    persist_stop: &AtomicBool,
    persist_park_thread: &Thread,
    persist_exited_cv: &(Mutex<bool>, Condvar),
    persist_thread: JoinHandle<()>,
    deadline: Instant,
) {
    persist_stop.store(true, Ordering::Release);
    persist_park_thread.unpark();
    if wait_bool_cv_until(persist_exited_cv, deadline) {
        let _ = persist_thread.join();
    } else {
        tracing::warn!(
            "Teardown: detaching stalled persistence worker (`armd-persist`) after bounded wait so startup/shutdown/Drop cannot block indefinitely"
        );
        drop(persist_thread);
    }
}

/// Shared bounded wait for `armd-control` completion across startup failure, ordinary teardown,
/// panic teardown, and emergency teardown. Returns `true` ONLY when `control_exited_cv.exited`
/// became `true` (published strictly AFTER `drop(core)` completes on `armd-control`); returns
/// `false` immediately if `teardown_disable.timed_out` is set or `deadline` elapses.
fn wait_control_exited_or_timed_out_until(
    control_exited_cv: &(Mutex<ControlExitState>, Condvar),
    teardown_disable: &TeardownDisable,
    deadline: Instant,
) -> bool {
    let (lock, cv) = control_exited_cv;
    let mut st = lock.lock().unwrap_or_else(|e| e.into_inner());
    while !st.exited {
        if teardown_disable.timed_out.load(Ordering::Acquire) {
            return false;
        }
        let rem = deadline.saturating_duration_since(Instant::now());
        if rem.is_zero() {
            return false;
        }
        let (next, res) = cv.wait_timeout(st, rem).unwrap_or_else(|e| e.into_inner());
        st = next;
        if st.exited {
            return true;
        }
        if teardown_disable.timed_out.load(Ordering::Acquire) || res.timed_out() {
            return false;
        }
    }
    true
}

/// Shared bounded teardown/panic-disable helper used across startup failure, startup panic,
/// release-loop panic, and emergency shutdown so there is one deadline/observer/abort path.
///
/// When `probe_enable_with_estop` is `Some(estop_state)`, the driver enable-state probe runs
/// INSIDE the armed deadline observer and `catch_unwind` before deciding whether disable can be
/// skipped: any non-disabled state, unknown state, or probe panic latches E-stop and runs
/// `attempt_panic_disable_until`; any probe or disable stall latches E-stop in the deadline
/// observer before publishing timeout.
#[allow(clippy::too_many_arguments)]
fn run_bounded_teardown_disable(
    core: &mut SupervisorInner,
    teardown_disable: &Arc<TeardownDisable>,
    exited: &Arc<AtomicBool>,
    control_exited_cv: &Arc<(Mutex<ControlExitState>, Condvar)>,
    hub: &Arc<StateHub>,
    hooks: &PersistTestHooks,
    policy: Option<PanicPolicy>,
    context: &'static str,
    secondary_panic_msg: &'static str,
    startup_gate_on_timeout: Option<StartupGate>,
    probe_enable_with_estop: Option<&Arc<crate::rt_core::urgent::EstopState>>,
) {
    let disable_timeout = hooks
        .panic_disable_timeout
        .unwrap_or(crate::supervisor::PANIC_DISABLE_TIMEOUT);
    let td_obs = Arc::clone(teardown_disable);
    let exited_obs = Arc::clone(exited);
    let cv_obs = Arc::clone(control_exited_cv);
    let hub_obs = Arc::clone(hub);
    let timeout_hook = hooks.on_panic_disable_timeout.clone();
    let estop_obs = probe_enable_with_estop.map(Arc::clone);
    let origin = Instant::now();
    let (coop_deadline, hard_deadline) =
        crate::supervisor::panic_disable_deadlines(origin, disable_timeout);
    let coop_budget = coop_deadline.saturating_duration_since(origin);
    let completed = crate::supervisor::run_with_panic_deadline_observer_until(
        hard_deadline,
        || {
            std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                if let Some(estop_state) = probe_enable_with_estop {
                    let confirmed_disabled = !core.is_torqued
                        && !core.disable_pending
                        && match core.driver.as_ref() {
                            None => true,
                            Some(d) => std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                                d.with_ref(|drv| {
                                    drv.enable_state() == crate::driver::EnableState::Disabled
                                        && !drv.has_any_joint_enabled()
                                        && !drv.has_known_partial_enable()
                                })
                                .unwrap_or(false)
                            }))
                            .unwrap_or(false),
                        };
                    if confirmed_disabled {
                        return crate::supervisor::PanicDisableOutcome::Confirmed;
                    }
                    estop_state.request_and_latch();
                }
                core.attempt_panic_disable_until(coop_deadline, hard_deadline, coop_budget)
            }))
            .unwrap_or_else(|_| {
                crate::supervisor::PanicDisableOutcome::Failed(secondary_panic_msg.into())
            })
        },
        |outcome| teardown_disable.try_set_outcome_before(outcome.clone(), hard_deadline),
        move || {
            // If the enable-state probe stalled before reaching `request_and_latch`, latch the
            // shared E-stop BEFORE publishing timeout so callers always observe an E-stop latch.
            if let Some(ref estop) = estop_obs {
                estop.request_and_latch();
            }
            let timed_out = crate::supervisor::PanicDisableOutcome::TimedOut;
            td_obs.set_outcome(timed_out.clone());
            // Wake any teardown waiter on `control_exited_cv` WITHOUT setting `exited = true`,
            // so `wait_control_exited_or_timed_out_until` observes `timed_out` immediately and detaches.
            {
                let _g = cv_obs.0.lock().unwrap_or_else(|e| e.into_inner());
                cv_obs.1.notify_all();
            }
            timed_out.log(context, disable_timeout);
            if matches!(policy, Some(PanicPolicy::AbortProcess)) {
                crate::diagnostics::logwriter::flush_all(Duration::from_millis(200));
                std::process::exit(1);
            }
            if let Some(ref gate) = startup_gate_on_timeout {
                let _ = gate.try_settle_failed(SpawnExecutorError::Fatal(format!(
                    "{context}: bounded disable timed out"
                )));
            }
            if policy.is_some() || startup_gate_on_timeout.is_some() {
                // Do NOT mark `control_exited_cv.exited` true here: `armd-control` is still
                // blocked inside the driver call and must be detached rather than joined.
                exited_obs.store(true, Ordering::Release);
                hub_obs.wake();
                if let Some(hook) = timeout_hook {
                    hook();
                }
            }
        },
    );
    if let Some(outcome) = completed {
        outcome.log(context, disable_timeout);
        if matches!(policy, Some(PanicPolicy::AbortProcess)) {
            crate::diagnostics::logwriter::flush_all(Duration::from_millis(500));
            std::process::exit(1);
        }
    }
}

fn mark_control_exited(
    exited: &AtomicBool,
    control_exited_cv: &(Mutex<ControlExitState>, Condvar),
    hub: &StateHub,
) {
    exited.store(true, Ordering::Release);
    {
        let mut st = control_exited_cv.0.lock().unwrap_or_else(|e| e.into_inner());
        st.dropping = true;
        st.exited = true;
    }
    control_exited_cv.1.notify_all();
    hub.wake();
}

#[allow(clippy::too_many_arguments)]
fn control_thread_main(
    mut core: SupervisorInner,
    engine: ReleaseEngine,
    cfg: ExecutorConfig,
    initial_generation: u64,
    stop_requests_init: super::urgent::StopRequests,
    closed_bitset_init: Arc<super::requests::actions::ClosedBitset>,
    startup_gate: StartupGate,
    control_stop: Arc<AtomicBool>,
    exited: Arc<AtomicBool>,
    control_exited_cv: Arc<(Mutex<ControlExitState>, Condvar)>,
    teardown_disable: Arc<TeardownDisable>,
    stats: Arc<ExecutorStats>,
    control_sched: Arc<Mutex<Option<crate::diagnostics::tick::ControlThreadSched>>>,
    control_period_us: Arc<AtomicU64>,
    mut diag_tx: Producer<TickSample>,
    mut diag_amend_tx: Producer<(u64, u32)>,
    maint_req_rx: Consumer<MaintRequest>,
    maint_rep_tx: Producer<MaintReply>,
    estop_ack_tx: Producer<EstopAck>,
    estop_overflow_gen: Arc<AtomicU64>,
    reclaim_state_tx: Producer<Arc<SafetyState>>,
    reclaim_stop_tx: Producer<RetiredStopPayloads>,
    reclaim_traj_tx: Producer<crate::motion::ActiveTrajectory>,
    reclaim_macro_tx: Producer<crate::macro_action::MacroRun>,
    #[cfg(feature = "policy-runtime")]
    reclaim_policy_tx: Producer<crate::policy_runtime::PolicyRun>,
    reclaim_return_rx: Consumer<Arc<SafetyState>>,
    safety_log_tx: Producer<SafetyLogEvent>,
    shutdown_hold_error: Arc<Mutex<Option<SafetyLogEvent>>>,
    persist_job_tx: Producer<PendingZeroCheckPersist>,
    persist_cmd_rx: Consumer<PersistCmd>,
    persist_rep_tx: Producer<PersistReply>,
    watchdog_plan_rx: Consumer<WatchdogParkPlan>,
    offload_reclaim_tx: Producer<OffloadReclaim>,
    persist_wake: Thread,
    estop_state: Arc<crate::rt_core::urgent::EstopState>,
    hub: Arc<StateHub>,
    hooks: PersistTestHooks,
) {
    let _abort_guard = StartupAbortGuard {
        gate: startup_gate.clone(),
        estop_state: Arc::clone(&estop_state),
        control_exited_cv: Arc::clone(&control_exited_cv),
        exited: Arc::clone(&exited),
        hub: Arc::clone(&hub),
    };

    enum StartupStepOutcome {
        Ready {
            staged_motion_cfg: Arc<super::requests::motion::MotionConfigSnapshot>,
            staged_prep_env: Arc<super::requests::actions::ActionPrepEnv>,
        },
        Recoverable {
            unsupported_reason: Option<&'static str>,
            msg: String,
        },
        NonRecoverable {
            msg: String,
        },
        TimedOut,
    }

    // Complete ALL driver preflight (`position_limits`, `supports_rt_executor`), executor routing,
    // driver RT mode handoff, and observation-slot prewarming INSIDE a single unwind boundary
    // while retaining `core` outside `catch_unwind` for cleanup. Shared hub snapshots are staged
    // locally and only published AFTER `startup_gate.try_settle_ready` wins so a timed-out worker
    // never mutates `hub`. Under unwind-enabled builds (`dev`/`test` or `panic = "unwind"`),
    // `catch_unwind` latches E-stop and runs bounded panic-disable before obeying `cfg.panic_policy`;
    // under `[profile.release]` (`panic = "abort"`), a Rust panic aborts immediately without
    // unwinding (relying on motor hardware watchdog / external E-stop).
    let init_res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        assert!(
            core.driver
                .as_ref()
                .is_none_or(crate::supervisor_lock::DriverSlot::is_exclusive),
            "RT executor must exclusively own driver without mutex"
        );
        if let Some(ref mut slot) = core.driver {
            slot.bind_current_thread();
        }
        if let Some(return_core) = hooks.fail_init {
            return if return_core {
                StartupStepOutcome::Recoverable {
                    unsupported_reason: None,
                    msg: "simulated executor init failure (recoverable)".into(),
                }
            } else {
                StartupStepOutcome::NonRecoverable {
                    msg: "simulated executor init failure (core lost)".into(),
                }
            };
        }
        // Driver preflight before any mutation of `core`'s routing/config fields so explicit
        // `Err(reason)` from `supports_rt_executor()` returns the untouched core cleanly to Legacy.
        let mut cached_pos = [None; crate::rt_core::mailbox::MAX_DOF];
        if let Some(ref d) = core.driver {
            let dof = core.config.dof.min(crate::rt_core::mailbox::MAX_DOF);
            let cap_res = d
                .with_ref(|drv| {
                    for (i, slot) in cached_pos.iter_mut().take(dof).enumerate() {
                        *slot = drv.position_limits(i);
                    }
                    drv.supports_rt_executor()
                })
                .unwrap_or(Err("driver unavailable"));
            if let Err(reason) = cap_res {
                return StartupStepOutcome::Recoverable {
                    unsupported_reason: Some(reason),
                    msg: format!("Cannot transfer ownership: {reason}"),
                };
            }
        }
        if startup_gate.is_timed_out() {
            return StartupStepOutcome::TimedOut;
        }
        core.cached_driver_pos_limits = cached_pos;
        let staged_motion_cfg = Arc::new(
            super::requests::motion::MotionConfigSnapshot::from_supervisor(&core),
        );
        core.set_boot_id(cfg.boot);
        core.attach_stop_requests(stop_requests_init);
        core.attach_closed_bitset(closed_bitset_init);
        let staged_prep_env = Arc::new(super::requests::actions::ActionPrepEnv::from_inner(&core));
        core.rt_executor_owned = true;
        core.before_publish_hook = hooks.before_publish.clone();
        core.safety.set_off_thread_reclaim(true);
        core.prewarm_rt_buffers();
        if let Some(drv) = core
            .driver
            .as_mut()
            .and_then(crate::supervisor_lock::DriverSlot::as_exclusive_mut)
        {
            drv.set_rt_mode(true);
        }
        core.prewarm_observation_slots();
        // Preserve servo stream authorization across legacy -> executor handoff if a valid lease was
        // already committed before `start_executor`.
        let pre_lease_gen = core.active_lease_gen_at(Instant::now());
        if pre_lease_gen != 0 && core.core.target_authority().stream.is_none() {
            core.authorize_servo_stream_for_committed_lease(pre_lease_gen);
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
        } else {
            let s = sched::observe_current_thread(&cfg.memory_lock);
            *control_sched.lock().unwrap_or_else(|e| e.into_inner()) = Some(s);
        }
        if cfg.alloc_timing {
            crate::diagnostics::alloc::time_this_thread();
        }
        StartupStepOutcome::Ready {
            staged_motion_cfg,
            staged_prep_env,
        }
    }));

    let teardown_timeout_loser = |mut core: SupervisorInner| {
        if core.driver.is_some() || core.is_torqued || core.disable_pending {
            run_bounded_teardown_disable(
                &mut core,
                &teardown_disable,
                &exited,
                &control_exited_cv,
                &hub,
                &hooks,
                None,
                "RT executor late startup abort",
                "secondary panic during late startup abort disable",
                None,
                Some(&estop_state),
            );
        }
        drop(core);
        mark_control_exited(&exited, &control_exited_cv, &hub);
    };

    let (staged_motion_cfg, staged_prep_env) = match init_res {
        Ok(StartupStepOutcome::Ready {
            staged_motion_cfg,
            staged_prep_env,
        }) => (staged_motion_cfg, staged_prep_env),
        Ok(StartupStepOutcome::Recoverable {
            unsupported_reason,
            msg,
        }) => {
            if let Some(rejected_core) = startup_gate.try_settle_failed(
                SpawnExecutorError::Recoverable {
                    core: Box::new(core),
                    unsupported_reason,
                    msg,
                },
            ) {
                // Caller already timed out while preflight was blocked: never drop a potentially
                // torqued driver without running bounded disable first.
                teardown_timeout_loser(*rejected_core);
            } else {
                mark_control_exited(&exited, &control_exited_cv, &hub);
            }
            return;
        }
        Ok(StartupStepOutcome::TimedOut) => {
            teardown_timeout_loser(core);
            return;
        }
        Ok(StartupStepOutcome::NonRecoverable { msg }) => {
            if core.driver.is_some() || core.is_torqued || core.disable_pending {
                run_bounded_teardown_disable(
                    &mut core,
                    &teardown_disable,
                    &exited,
                    &control_exited_cv,
                    &hub,
                    &hooks,
                    None,
                    "RT executor init failure",
                    "secondary panic during RT executor init-failure disable",
                    Some(startup_gate.clone()),
                    Some(&estop_state),
                );
            }
            // Publish terminal startup failure BEFORE destroying `core` so a stalling driver
            // destructor cannot block startup beyond `startup_deadline`.
            let _ = startup_gate.try_settle_failed(SpawnExecutorError::Fatal(msg));
            drop(core);
            mark_control_exited(&exited, &control_exited_cv, &hub);
            return;
        }
        Err(err) => {
            estop_state.request_and_latch();
            tracing::error!("RT executor thread panicked during startup initialization: {:?}", err);
            run_bounded_teardown_disable(
                &mut core,
                &teardown_disable,
                &exited,
                &control_exited_cv,
                &hub,
                &hooks,
                Some(cfg.panic_policy),
                "RT executor startup panic",
                "secondary panic during RT executor startup panic-disable",
                Some(startup_gate.clone()),
                None,
            );
            let _ = startup_gate.try_settle_failed(SpawnExecutorError::Fatal(
                "RT executor panicked during startup initialization".into(),
            ));
            drop(core);
            mark_control_exited(&exited, &control_exited_cv, &hub);
            return;
        }
    };

    if !startup_gate.try_settle_ready(thread::current().id()) {
        // Caller timed out (and latched E-stop + committed OWNER_FAILED) while startup was in
        // flight: NEVER mutate `hub` or enter the release loop; run bounded teardown disable and
        // destroy `core` exclusively on `armd-control`.
        teardown_timeout_loser(core);
        return;
    }
    // Publish executor-only snapshots into `hub` strictly AFTER winning startup readiness and
    // BEFORE release #1 can consume them.
    hub.set_motion_config(staged_motion_cfg);
    hub.update_prep_env(staged_prep_env);

    let mut engine = engine;
    let stop_requests = engine.stop_requests().clone();
    let mut state = ControlState {
        shutdown_committed: false,
        control_stop: Arc::clone(&control_stop),
        stop_requests,
        estop_ack_tx,
        estop_overflow_gen,
        reclaim_state_tx,
        reclaim_stop_tx,
        reclaim_traj_tx,
        reclaim_macro_tx,
        #[cfg(feature = "policy-runtime")]
        reclaim_policy_tx,
        reclaim_return_rx,
        safety_log_tx,
        shutdown_hold_error,
        stats: Arc::clone(&stats),
        maint_req_rx,
        maint_rep_tx,
        pending_maint_reply: None,
        persist_job_tx,
        persist_cmd_rx,
        persist_rep_tx,
        watchdog_plan_rx,
        last_woken_watchdog_park: None,
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

    let clock_hook = state.hooks.clock.clone();
    let now_inst = || -> Instant {
        clock_hook.as_ref().map_or_else(Instant::now, |c| c.now())
    };
    let now_ns_fn = || -> u64 {
        clock_hook.as_ref().map_or_else(mono_now_ns, |c| c.now_ns())
    };
    let sleep_until = |target_ns: u64| -> bool {
        if let Some(c) = clock_hook.as_ref() {
            c.sleep_until_ns(target_ns, &control_stop)
        } else {
            sleep_until_ns(target_ns, &control_stop, &stats)
        }
    };
    let stop_bound_ns = || -> u64 {
        if clock_hook.is_none() {
            stats.stop_requested_mono_ns.load(Ordering::Acquire)
        } else {
            0
        }
    };
    let sleep_dur = |d: Duration| {
        if let Some(c) = clock_hook.as_ref() {
            c.sleep(d, &control_stop);
        } else {
            thread::sleep(d);
        }
    };

    struct LoopActiveGuard<'a>(&'a ExecutorStats);
    impl Drop for LoopActiveGuard<'_> {
        fn drop(&mut self) {
            self.0.finalize_loop_exit(0);
        }
    }
    stats.mark_loop_active();
    let _loop_active = LoopActiveGuard(&stats);

    let mut releases = Releases::start_at(now_ns_fn(), cfg.period);
    let mut last_release_start = now_inst();
    let mut publish_gen = initial_generation;
    let mut panicked = false;
    let mut ran_any_release = false;
    let mut stopped_cleanly = true;

    // Compile-time check: only `Copy` state crosses from Window 1 (whole release) into Window 2
    // (sub-release wakes) and the single release-completion point, ensuring zero uncounted destruction.
    const _: () = {
        const fn assert_copy<T: Copy>() {}
        assert_copy::<(TickSample, Option<(super::ops::ReleaseReport, Instant, u64)>)>();
    };

    loop {
        let scheduled_ns = match wait_for_next_release(
            &control_stop,
            &mut releases,
            &stats,
            ran_any_release,
            &now_ns_fn,
            &stop_bound_ns,
            &sleep_until,
            &sleep_dur,
        ) {
            WaitOutcome::Wake { scheduled_ns } => scheduled_ns,
            WaitOutcome::Stop { stopped_cleanly: clean } => {
                stopped_cleanly = clean;
                break;
            }
        };
        ran_any_release = true;

        // Sample actual wake time FIRST — right after sleep returns and stop check,
        // strictly before allocation bookkeeping (`WholeReleaseAllocGuard`) and `UsageMark::now()`
        // (matching `run_periodic_with_clock`).
        let actual_ns = now_ns_fn();
        let release_start = now_inst();
        let wake_lateness_ns = actual_ns.saturating_sub(scheduled_ns);
        let wake_lateness_us = crate::diagnostics::tick::ns_to_us_u32(wake_lateness_ns);
        stats.mark_release_started();
        let wake_skipped;
        let step_num;

        // ---- Window 1: Whole release (disjoint from sub-release wakes) ----
        let release_res = {
            // Declared FIRST -> drops LAST, after catch_unwind and ALL release locals drop!
            let _alloc_scope = WholeReleaseAllocGuard::enter(&stats, false);
            let usage_start = UsageMark::now();
            wake_skipped = releases.advance(actual_ns);
            let elapsed = release_start.saturating_duration_since(last_release_start);
            last_release_start = release_start;
            if elapsed > LATE_LOG_AFTER {
                stats.last_late_us.store(elapsed.as_micros() as u64, Ordering::Relaxed);
                stats.late.fetch_add(1, Ordering::Release);
            }
            step_num = stats.steps.fetch_add(1, Ordering::Relaxed) + 1;
            let dt = elapsed.as_secs_f64();
            let next_release_ns = releases.next_ns();
            let remaining_ns = next_release_ns.saturating_sub(now_ns_fn());
            let release_deadline = now_inst() + Duration::from_nanos(remaining_ns);
            publish_gen += 1;

            std::panic::catch_unwind(std::panic::AssertUnwindSafe(
                || -> Option<(TickSample, Option<(super::ops::ReleaseReport, Instant, u64)>)> {
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
                            state.service_watchdog_park_plan(core);
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
                        state.drain_retired_states(&mut core, now_inst());
                        if engine.take_urgent_replies_flushed()
                            || engine.take_lifecycle_replies_flushed()
                            || engine.take_servo_closes_flushed()
                        {
                            hub.wake();
                        }
                        return None;
                    }
                    let (phases, body_end) = core.take_tick_phases();
                    let body_end = body_end.unwrap_or_else(now_inst);
                    core.defer_zero_check_persist = false;
                    if let Some(p) = core.watchdog_park_pending.filter(|p| !p.waiting_mit_handback) {
                        let key = (p.generation, p.attempt);
                        if state.last_woken_watchdog_park != Some(key) {
                            state.last_woken_watchdog_park = Some(key);
                            state.persist_wake.unpark();
                        }
                    } else {
                        state.last_woken_watchdog_park = None;
                    }
                    if let Some(mut job) = core.pending_zero_check_persist.take() {
                        job.stop_gen = state.stop_requests.current();
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
                    // Publish safety/E-stop release_seq markers BEFORE waking maintenance or E-stop
                    // callers so Acquire loads after `trigger_e_stop_checked` / `clear_fault` always
                    // observe the tagged release sequence without racing `armd-control`.
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
                    if let Some((armed, slot)) = state.hooks.inject_release_alloc_once.as_deref() {
                        if armed.load(Ordering::Acquire) {
                            let ptr = Box::into_raw(Box::new(0xB3_u64));
                            slot.store(ptr, Ordering::Release);
                            release_flags |= RELEASE_FLAG_INJECT_ALLOC;
                            stats.last_inject_alloc_release_seq.store(step_num, Ordering::Release);
                            armed.store(false, Ordering::Release);
                        }
                    }
                    // Deliver the pre-commit maintenance reply strictly AFTER Step 6 publication, then
                    // drain any E-stop acks / retired states produced during Steps 3-7.
                    state.flush_pending_maint_reply(&core);
                    state.emit_estop_ack_if_advanced(&mut core);
                    state.drain_retired_states(&mut core, now_inst());
                    let control_hz = core.config.control_frequency_hz;
                    let done = now_inst();
                    let allocs = crate::diagnostics::alloc::take();
                    let usage_done = UsageMark::now();
                    control_period_us.store((1e6 / f64::from(control_hz.max(1))) as u64, Ordering::Relaxed);

                    let sample = TickSample {
                        tick_us: crate::diagnostics::tick::us(done.saturating_duration_since(release_start)),
                        release_us: 0, // Stamped once at the single release-completion point.
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
                    // Wake dispatcher immediately after Window 1 publication/drain work (preserving
                    // pre-Window-2 service latency for observations, acks, and safety logs).
                    hub.wake();

                    let next_wake = report.poll.wake_before(release_deadline);
                    drop(_guard);
                    Some((sample, next_wake.map(|_| (report, release_deadline, next_release_ns))))
                },
            ))
        };

        // ---- Window 2: Disjoint sub-release transition wakes before `next_release_ns` ----
        let mut step_sample: Option<TickSample> = None;
        let step_res: Result<(), Box<dyn Any + Send>> = match release_res {
            Err(err) => Err(err),
            Ok(None) => Ok(()),
            Ok(Some((sample, None))) => {
                step_sample = Some(sample);
                Ok(())
            }
            Ok(Some((sample, Some((mut report, release_deadline, next_release_ns))))) => {
                step_sample = Some(sample);
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
                        let _ = sleep_until(wake_ns.min(next_release_ns));
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
                            let urgent_flushed = engine.take_urgent_replies_flushed()
                                || engine.take_lifecycle_replies_flushed()
                                || engine.take_servo_closes_flushed();
                            if published {
                                publish_gen += 1;
                            }
                            if published || urgent_flushed {
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
                            state.drain_retired_states(&mut core, now_inst());
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

        // ---- Single release-completion point (full wall time from wake through all release work) ----
        let tail_alloc_start = crate::diagnostics::alloc::snapshot_counts();
        if let Some(delay) = state
            .hooks
            .post_publish_delay_once
            .as_ref()
            .and_then(|d| d.lock().ok().and_then(|mut g| g.take()))
        {
            sleep_dur(delay);
        }
        let pre_diag_end_ns = now_ns_fn();
        let pre_diag_release_ns = pre_diag_end_ns.saturating_sub(actual_ns);
        let stamped_release_us = crate::diagnostics::tick::ns_to_us_u32(pre_diag_release_ns);
        let mut diag_dropped_delta: u64 = 0;
        let pushed_sample = step_sample.is_some() || step_res.is_err();
        if pushed_sample {
            let mut sample = step_sample.unwrap_or(TickSample {
                tick_us: stamped_release_us,
                release_us: 0,
                wake_lateness_us,
                release_seq: step_num,
                safety_epoch: core.safety.epoch_id(),
                ..Default::default()
            });
            sample.release_us = stamped_release_us;
            if diag_tx.push(sample).is_err() {
                diag_dropped_delta = 1;
            }
            let _ = engine.take_urgent_replies_flushed();
            let _ = engine.take_lifecycle_replies_flushed();
            let _ = engine.take_servo_closes_flushed();
            hub.wake();
        }
        // Positive-control tail probe (strictly AFTER `diag_tx.push` and `hub.wake()`);
        // `_tail_local` drops BEFORE tail allocations are recorded and completion is published.
        {
            let _tail_local = state.run_tail_alloc_probe(&state.hooks.inject_release_tail_once);
        }
        // Optional test tail-stall hook (strictly AFTER `diag_tx.push` + `hub.wake()` + tail probe,
        // BEFORE final timestamp, sample amendment, and completion seal).
        let post_stats_delay_ns = stats.post_stats_delay_once_ns.swap(0, Ordering::AcqRel);
        if post_stats_delay_ns > 0 {
            sleep_dur(Duration::from_nanos(post_stats_delay_ns));
        }

        // Probe duration after diag push, dispatcher wake, B2 tail probe, and any post-push
        // delay. If duration grew past `stamped_release_us`, push one lock-free amendment on the
        // preallocated SPSC `diag_amend_tx` ring and wake the dispatcher BEFORE recording tail
        // allocations and taking the final release end timestamp inside the completion seal.
        let probe_release_us =
            crate::diagnostics::tick::ns_to_us_u32(now_ns_fn().saturating_sub(actual_ns));
        if pushed_sample && diag_dropped_delta == 0 && probe_release_us > stamped_release_us {
            if diag_amend_tx.push((step_num, probe_release_us)).is_err() {
                diag_dropped_delta = diag_dropped_delta.saturating_add(1);
            }
            hub.wake();
        }

        // Record ALL tail allocations (covering `post_publish_delay_once`, `diag_tx.push`,
        // `hub.wake()`, B2 tail probe + local drop, and `diag_amend_tx.push` + `hub.wake()`)
        // strictly BEFORE the final release end timestamp and completion publication.
        let tail_alloc_delta = crate::diagnostics::alloc::snapshot_counts().delta_since(tail_alloc_start);
        if tail_alloc_delta.total() > 0 {
            stats.release_allocs.record(tail_alloc_delta);
        }

        // Optional test preemption hook strictly AFTER tail allocation accounting and BEFORE
        // the completion seal (exercises preemption after sample amendment / alloc accounting).
        let pre_seal_delay_ns = stats.pre_seal_delay_once_ns.swap(0, Ordering::AcqRel);
        if pre_seal_delay_ns > 0 {
            sleep_dur(Duration::from_nanos(pre_seal_delay_ns));
        }

        // PUBLISH COMPLETION STRICTLY LAST: single `stats_seq` Release seal atomically clears
        // `in_flight_start_mono_ns = 0`, samples the final release end timestamp STRICTLY AFTER
        // all release work and clearing the in-flight marker, computes overrun skips from that
        // exact timestamp, updates all breach/peak/skip/drop/marker counters, and publishes
        // `releases_total` + `stats_seq` last; `completed_releases` follows with Release ordering
        // for direct B2 alloc readers.
        stats.commit_release_outcome_with_clock(
            actual_ns,
            wake_lateness_ns,
            wake_skipped,
            diag_dropped_delta,
            &mut releases,
            now_ns_fn,
        );
        stats.completed_releases.fetch_add(1, Ordering::Release);

        if let Err(err) = step_res {
            panicked = true;
            estop_state.request_and_latch();
            tracing::error!("RT executor thread panicked during release: {:?}", err);
            run_bounded_teardown_disable(
                &mut core,
                &teardown_disable,
                &exited,
                &control_exited_cv,
                &hub,
                &state.hooks,
                Some(cfg.panic_policy),
                "RT executor panic",
                "secondary panic during RT executor panic-disable",
                None,
                None,
            );
            break;
        }
    }

    // Loop-exit final accounting pass (normal shutdown): if the loop exited after a release
    // without entering a genuine sleep, account any scheduled deadlines crossed before shutdown.
    let exit_skips = if !panicked && ran_any_release && !stopped_cleanly {
        let sb = stop_bound_ns();
        releases.advance_overrun(if sb != 0 { sb } else { now_ns_fn() })
    } else {
        0
    };
    stats.finalize_loop_exit(exit_skips);

    let mut disable_ran = false;
    if !panicked && teardown_disable.requested.load(Ordering::Acquire) {
        run_bounded_teardown_disable(
            &mut core,
            &teardown_disable,
            &exited,
            &control_exited_cv,
            &hub,
            &state.hooks,
            None,
            "RT executor emergency shutdown",
            "panic during emergency teardown disable",
            None,
            None,
        );
        disable_ran = true;
    }

    // Clean exit drain: return queued maintenance and persistence payloads off-thread without
    // performing unpublished live state mutations, publish before flushing any unacked E-stops
    // or applied urgent stops (skipping publish if the release panicked), and signal exit before
    // waking sinks.
    state.drain_maintenance_stopped();
    state.drain_persistence_stopped(&core);
    if !panicked {
        if !core.pending_estop_acks.is_empty()
            || engine.has_applied_urgent_stop_pending()
            || engine.has_pending_lifecycle_replies()
        {
            publish_gen += 1;
            let ts = core.now_timestamp();
            core.publish(ts, publish_gen);
        }
        engine.drain_urgent_stops_stopped(true);
        engine.drain_conn_lifecycle_stopped(true);
        let _ = engine.flush_servo_closes(&mut core);
        state.emit_estop_ack_if_advanced(&mut core);
    } else {
        engine.drain_urgent_stops_stopped(false);
        engine.drain_conn_lifecycle_stopped(false);
        state.retire_unpublished_estop_acks(&mut core);
    }
    state.drain_estop_and_retired_stopped(&mut core, now_inst());
    state.flush_pending_hold_error_for_shutdown(&mut core);
    // Synchronized pre-drop checkpoint under `control_exited_cv.0`: if a timed-out forced teardown
    // requested disable while `armd-control` was finishing a release or post-loop drain, run
    // bounded disable BEFORE marking `dropping = true` and destroying `core`.
    loop {
        let mut st = control_exited_cv.0.lock().unwrap_or_else(|e| e.into_inner());
        if !panicked && !disable_ran && teardown_disable.requested.load(Ordering::Acquire) {
            drop(st);
            run_bounded_teardown_disable(
                &mut core,
                &teardown_disable,
                &exited,
                &control_exited_cv,
                &hub,
                &state.hooks,
                None,
                "RT executor forced teardown disable",
                "panic during forced teardown disable",
                None,
                Some(&estop_state),
            );
            disable_ran = true;
            continue;
        }
        st.dropping = true;
        break;
    }
    // Destroy `core` (and the exclusively owned driver) BEFORE publishing `control_exited_cv.exited`,
    // so `teardown_and_join_inner` never enters an unbounded `join()` while a driver destructor
    // is stalled.
    drop(core);
    mark_control_exited(&exited, &control_exited_cv, &hub);
}

/// Monotonic nanoseconds of `CLOCK_MONOTONIC` (the clock of `Instant` on Linux).
#[cfg(target_os = "linux")]
fn mono_now_ns() -> u64 {
    // SAFETY: `ts` is a valid out-pointer.
    let mut ts: libc::timespec = unsafe { std::mem::zeroed() };
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
}

/// Sleeps until the absolute `CLOCK_MONOTONIC` time `at_ns`, or wakes early on `stats.signal_stop()`.
///
/// Returns `false` if the thread did NOT enter a clean pre-deadline sleep (either `mono_now_ns() >= at_ns`
/// on entry, or shutdown was requested after `at_ns` / at the check-to-sleep boundary), so callers
/// account any crossed deadlines. Returns `true` if the thread genuinely slept until `at_ns` or was
/// woken cleanly before `at_ns`.
#[cfg(target_os = "linux")]
fn sleep_until_ns(at_ns: u64, stop: &AtomicBool, stats: &ExecutorStats) -> bool {
    if mono_now_ns() >= at_ns {
        return false;
    }
    if stats.check_to_sleep_delay_once_ns.load(Ordering::Relaxed) != 0 {
        let delay_ns = stats.check_to_sleep_delay_once_ns.swap(0, Ordering::AcqRel);
        if delay_ns > 0 {
            thread::sleep(Duration::from_nanos(delay_ns));
        }
    }
    if stop.load(Ordering::Acquire) {
        let stop_req = stats.stop_requested_mono_ns.load(Ordering::Acquire);
        return stop_req != 0 && stop_req < at_ns;
    }
    // SAFETY: timespec is plain old data; all-zero is valid.
    let mut ts: libc::timespec = unsafe { std::mem::zeroed() };
    ts.tv_sec = (at_ns / 1_000_000_000) as _;
    ts.tv_nsec = (at_ns % 1_000_000_000) as _;
    const FUTEX_WAIT_BITSET: libc::c_int = 9;
    const FUTEX_PRIVATE_FLAG: libc::c_int = 128;
    const FUTEX_BITSET_MATCH_ANY: u32 = 0xffff_ffff;
    loop {
        // Atomically verify `stop_word == 0` and wait on an absolute `CLOCK_MONOTONIC` hrtimer.
        // SAFETY: `stop_word` is a 4-byte aligned `AtomicU32`; `ts` is a valid `timespec`.
        let rc = unsafe {
            libc::syscall(
                libc::SYS_futex,
                &stats.stop_word as *const AtomicU32,
                FUTEX_WAIT_BITSET | FUTEX_PRIVATE_FLAG,
                0u32,
                &ts as *const libc::timespec,
                std::ptr::null::<u32>(),
                FUTEX_BITSET_MATCH_ANY,
            )
        };
        if rc == 0 {
            if stop.load(Ordering::Acquire) || stats.stop_word.load(Ordering::Acquire) != 0 {
                // Woken by `FUTEX_WAKE` while genuinely parked.
                let stop_req = stats.stop_requested_mono_ns.load(Ordering::Acquire);
                return !(stop_req != 0 && stop_req >= at_ns);
            }
            // Spurious futex wakeup while still running: keep sleeping until `at_ns`.
            if mono_now_ns() >= at_ns {
                return true;
            }
            continue;
        }
        let err = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
        if err == libc::EINTR {
            if stop.load(Ordering::Acquire) {
                let stop_req = stats.stop_requested_mono_ns.load(Ordering::Acquire);
                return stop_req != 0 && stop_req < at_ns;
            }
            continue;
        }
        if err == libc::EAGAIN {
            // `stop_word != 0` was already set before entering the kernel sleep (never slept).
            let stop_req = stats.stop_requested_mono_ns.load(Ordering::Acquire);
            return stop_req != 0 && stop_req < at_ns;
        }
        if err == libc::ETIMEDOUT {
            if stop.load(Ordering::Acquire) {
                let stop_req = stats.stop_requested_mono_ns.load(Ordering::Acquire);
                if stop_req != 0 {
                    return stop_req < at_ns;
                }
            }
            return true;
        }
        return false;
    }
}

#[cfg(not(target_os = "linux"))]
fn mono_now_ns() -> u64 {
    static START: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    START.get_or_init(Instant::now).elapsed().as_nanos() as u64
}

#[cfg(not(target_os = "linux"))]
fn sleep_until_ns(at_ns: u64, stop: &AtomicBool, stats: &ExecutorStats) -> bool {
    let now = mono_now_ns();
    if now >= at_ns {
        return false;
    }
    if stats.check_to_sleep_delay_once_ns.load(Ordering::Relaxed) != 0 {
        let delay_ns = stats.check_to_sleep_delay_once_ns.swap(0, Ordering::AcqRel);
        if delay_ns > 0 {
            thread::sleep(Duration::from_nanos(delay_ns));
        }
    }
    if stop.load(Ordering::Acquire) {
        let stop_req = stats.stop_requested_mono_ns.load(Ordering::Acquire);
        return stop_req != 0 && stop_req < at_ns;
    }
    let rem = at_ns.saturating_sub(mono_now_ns());
    if rem == 0 {
        return false;
    }
    thread::sleep(Duration::from_nanos(rem));
    true
}

enum WaitOutcome {
    Wake { scheduled_ns: u64 },
    Stop { stopped_cleanly: bool },
}

/// Shared wait-entry transition for both `control_thread_main` and `run_periodic_with_clock`.
/// Ensures deadlines crossed after completion publication are accounted consistently whether
/// the loop continues or stops, without spinning on an expired first deadline or fabricating
/// skips on a clean stop during genuine sleep.
#[inline]
#[allow(clippy::too_many_arguments)]
fn wait_for_next_release(
    stop: &AtomicBool,
    releases: &mut Releases,
    stats: &ExecutorStats,
    ran_any_release: bool,
    now_ns: &impl Fn() -> u64,
    stop_bound_ns: &impl Fn() -> u64,
    sleep_until: &impl Fn(u64) -> bool,
    sleep_dur: &impl Fn(Duration),
) -> WaitOutcome {
    loop {
        if ran_any_release && stats.before_sleep_delay_once_ns.load(Ordering::Relaxed) != 0 {
            let delay_ns = stats.before_sleep_delay_once_ns.swap(0, Ordering::AcqRel);
            if delay_ns > 0 {
                sleep_dur(Duration::from_nanos(delay_ns));
            }
        }
        if stop.load(Ordering::Acquire) {
            return WaitOutcome::Stop {
                stopped_cleanly: !ran_any_release,
            };
        }
        let scheduled_ns = releases.next_ns();
        let actually_slept = sleep_until(scheduled_ns);
        if !actually_slept {
            if !ran_any_release {
                if stop.load(Ordering::Acquire) {
                    return WaitOutcome::Stop { stopped_cleanly: true };
                }
                return WaitOutcome::Wake { scheduled_ns };
            }
            let stopping = stop.load(Ordering::Acquire);
            let sb = if stopping { stop_bound_ns() } else { 0 };
            let crossed = releases.advance_overrun(if sb != 0 { sb } else { now_ns() });
            if crossed > 0 {
                stats.record_boundary_skips(crossed);
            }
            if stopping {
                return WaitOutcome::Stop { stopped_cleanly: true };
            }
            continue;
        }
        if stop.load(Ordering::Acquire) {
            return WaitOutcome::Stop { stopped_cleanly: true };
        }
        return WaitOutcome::Wake { scheduled_ns };
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

    /// Accounts all scheduled deadlines at or before `end_ns` crossed during an overrunning
    /// release and advances `next_ns` to the earliest grid point strictly after `end_ns`.
    /// Because every crossed deadline is committed at release completion (whether the loop
    /// continues or exits), overrun skips are never lost on shutdown or catch-up and never
    /// misattributed to a newly acknowledged baseline marker.
    pub fn advance_overrun(&mut self, end_ns: u64) -> u64 {
        let skipped = Self::missed_since(self.next_ns, end_ns, self.period_ns);
        self.next_ns = self.next_ns.saturating_add(skipped.saturating_mul(self.period_ns));
        skipped
    }

    /// Accounts all remaining scheduled deadlines at or before `end_ns` when a release aborts
    /// mid-body (e.g. panic unwind / panic-exit).
    pub fn missed_at_exit(&self, end_ns: u64) -> u64 {
        Self::missed_since(self.next_ns, end_ns, self.period_ns)
    }

    pub fn next_ns(&self) -> u64 { self.next_ns }
}

thread_local! {
    /// Active `(actual_ns, wake_lateness_ns, wake_skipped, next_ns, period_ns)` of an in-flight
    /// `run_periodic_with_deadline_and_clock` release on this thread so explicit panic-exit wrappers
    /// (`src/main.rs` before `process::exit`) and unwind guards commit the release outcome and
    /// final overrun exactly once.
    static ACTIVE_PERIODIC_RELEASE: std::cell::Cell<Option<(u64, u64, u64, u64, u64)>> =
        const { std::cell::Cell::new(None) };
}

/// Accounts release duration and any final overrun skips for the calling thread's in-flight
/// `run_periodic` release immediately before a panic-exit handler calls `attempt_legacy_panic_disable`
/// / `process::exit`. Consumes the thread-local state once so unwind paths cannot double-count.
pub fn account_periodic_overrun_on_panic_exit(stats: &ExecutorStats) {
    if let Some((actual_ns, wake_lateness_ns, wake_skipped, next_ns, period_ns)) =
        ACTIVE_PERIODIC_RELEASE.try_with(|c| c.take()).ok().flatten()
    {
        let mut exit_releases = Releases { period_ns, next_ns };
        stats.commit_release_outcome_with_clock(
            actual_ns,
            wake_lateness_ns,
            wake_skipped,
            0,
            &mut exit_releases,
            mono_now_ns,
        );
    }
}

/// Runs `step(dt)` at every release until `stop` is set. `dt` is the real time since the
/// previous step's start (seconds). Allocation free.
pub fn run_periodic(period: Duration, stop: &AtomicBool, stats: &ExecutorStats, mut step: impl FnMut(f64)) {
    run_periodic_with_deadline_and_clock(period, stop, stats, None, |dt, _| step(dt));
}

/// [`run_periodic`] with an optional deterministic [`ExecutorClock`] for tests.
pub fn run_periodic_with_clock(
    period: Duration,
    stop: &AtomicBool,
    stats: &ExecutorStats,
    clock: Option<&dyn ExecutorClock>,
    mut step: impl FnMut(f64),
) {
    run_periodic_with_deadline_and_clock(period, stop, stats, clock, |dt, _| step(dt));
}

/// Runs `step(dt, release_deadline)` at every scheduled grid release until `stop` is set.
/// `dt` is the real time since the previous step's start (seconds), and `release_deadline` is
/// the absolute scheduled grid instant of the next release (`releases.next_ns()` mapped onto
/// `Instant`, matching `control_thread_main`). Allocation free.
pub fn run_periodic_with_deadline(
    period: Duration,
    stop: &AtomicBool,
    stats: &ExecutorStats,
    step: impl FnMut(f64, Instant),
) {
    run_periodic_with_deadline_and_clock(period, stop, stats, None, step);
}

/// [`run_periodic_with_deadline`] with an optional deterministic [`ExecutorClock`] for tests.
pub fn run_periodic_with_deadline_and_clock(
    period: Duration,
    stop: &AtomicBool,
    stats: &ExecutorStats,
    clock: Option<&dyn ExecutorClock>,
    mut step: impl FnMut(f64, Instant),
) {
    struct LoopActiveGuard<'a>(&'a ExecutorStats);
    impl Drop for LoopActiveGuard<'_> {
        fn drop(&mut self) {
            self.0.finalize_loop_exit(0);
        }
    }
    stats.mark_loop_active();
    let _loop_active = LoopActiveGuard(stats);

    let now_inst = || clock.map_or_else(Instant::now, ExecutorClock::now);
    let now_ns = || clock.map_or_else(mono_now_ns, ExecutorClock::now_ns);
    let sleep_until = |target_ns: u64| -> bool {
        if let Some(c) = clock {
            c.sleep_until_ns(target_ns, stop)
        } else {
            sleep_until_ns(target_ns, stop, stats)
        }
    };
    let stop_bound_ns = || -> u64 {
        if clock.is_none() {
            stats.stop_requested_mono_ns.load(Ordering::Acquire)
        } else {
            0
        }
    };
    let sleep_dur = |d: Duration| {
        if let Some(c) = clock {
            c.sleep(d, stop);
        } else {
            thread::sleep(d);
        }
    };
    let mut releases = Releases::start_at(now_ns(), period);
    let mut last = now_inst();
    let mut ran_any_release = false;
    let stopped_cleanly = loop {
        let scheduled_ns = match wait_for_next_release(
            stop,
            &mut releases,
            stats,
            ran_any_release,
            &now_ns,
            &stop_bound_ns,
            &sleep_until,
            &sleep_dur,
        ) {
            WaitOutcome::Wake { scheduled_ns } => scheduled_ns,
            WaitOutcome::Stop { stopped_cleanly } => break stopped_cleanly,
        };
        ran_any_release = true;
        let actual_ns = now_ns();
        let woke = now_inst();
        let wake_lateness_ns = actual_ns.saturating_sub(scheduled_ns);
        stats.mark_release_started();
        let wake_skipped = releases.advance(actual_ns);
        let remaining_ns = releases.next_ns().saturating_sub(actual_ns);
        let release_deadline = woke + Duration::from_nanos(remaining_ns);
        let elapsed = woke.saturating_duration_since(last);
        last = woke;
        if elapsed > LATE_LOG_AFTER {
            stats.last_late_us.store(elapsed.as_micros() as u64, Ordering::Relaxed);
            stats.late.fetch_add(1, Ordering::Release);
        }
        stats.steps.fetch_add(1, Ordering::Relaxed);
        let stashed_sample = {
            let _lateness_guard = crate::diagnostics::tick::WakeLatenessGuard::enter_periodic(
                wake_lateness_ns,
                stats.has_legacy_tick_ring(),
            );
            let _ = ACTIVE_PERIODIC_RELEASE.try_with(|c| {
                c.set(Some((
                    actual_ns,
                    wake_lateness_ns,
                    wake_skipped,
                    releases.next_ns,
                    releases.period_ns,
                )))
            });
            struct PeriodicUnwindGridGuard<'a> {
                stats: &'a ExecutorStats,
                clock: Option<&'a dyn ExecutorClock>,
            }
            impl Drop for PeriodicUnwindGridGuard<'_> {
                fn drop(&mut self) {
                    if let Some((actual_ns, wake_lateness_ns, wake_skipped, next_ns, period_ns)) =
                        ACTIVE_PERIODIC_RELEASE.try_with(|c| c.take()).ok().flatten()
                    {
                        if std::thread::panicking() {
                            let sample = crate::diagnostics::tick::take_periodic_sample();
                            if let (Some(sample), Some(ring)) = (sample, self.stats.legacy_tick_ring.get()) {
                                let _ = ring.record_staged(&sample, || {
                                    let t = self.clock.map_or_else(mono_now_ns, ExecutorClock::now_ns);
                                    crate::diagnostics::tick::ns_to_us_u32(t.saturating_sub(actual_ns))
                                });
                            }
                            let mut exit_releases = Releases { period_ns, next_ns };
                            self.stats.commit_release_outcome_with_clock(
                                actual_ns,
                                wake_lateness_ns,
                                wake_skipped,
                                0,
                                &mut exit_releases,
                                || self.clock.map_or_else(mono_now_ns, ExecutorClock::now_ns),
                            );
                        }
                    }
                }
            }
            let _unwind_grid = PeriodicUnwindGridGuard { stats, clock };
            step(elapsed.as_secs_f64(), release_deadline);
            let _ = ACTIVE_PERIODIC_RELEASE.try_with(|c| c.take());
            let sample = crate::diagnostics::tick::take_periodic_sample();
            drop(_unwind_grid);
            sample
            // `_lateness_guard` drops here, strictly BEFORE final timing & ring publication.
        };

        // Optional test tail-stall hook (strictly BEFORE sample publication and completion seal).
        let post_stats_delay_ns = stats.post_stats_delay_once_ns.swap(0, Ordering::AcqRel);
        if post_stats_delay_ns > 0 {
            sleep_dur(Duration::from_nanos(post_stats_delay_ns));
        }

        // Publish rolling sample FIRST (before completion seal). `record_staged` stages all
        // payload fields into the unpublished slot, stamps `release_us` from `now_ns()`, and
        // publishes `slot.seq` + `meta_seq`.
        let mut stamped_us: u32 = 0;
        let recorded_seq = match (stashed_sample, stats.legacy_tick_ring.get()) {
            (Some(sample), Some(ring)) => ring.record_staged(&sample, || {
                let t = now_ns();
                stamped_us = crate::diagnostics::tick::ns_to_us_u32(t.saturating_sub(actual_ns));
                stamped_us
            }),
            _ => None,
        };

        // If OS preemption crossed `record_staged`, amend the slot monotonically BEFORE the
        // completion seal.
        let probe_release_us =
            crate::diagnostics::tick::ns_to_us_u32(now_ns().saturating_sub(actual_ns));
        if let (Some(seq), Some(ring)) = (recorded_seq, stats.legacy_tick_ring.get()) {
            if probe_release_us > stamped_us {
                ring.amend_slot_release_us(seq, probe_release_us);
            }
        }

        // Optional test preemption hook strictly AFTER sample amendment and BEFORE the
        // completion seal (exercises preemption after sample publication / amendment).
        let pre_seal_delay_ns = stats.pre_seal_delay_once_ns.swap(0, Ordering::AcqRel);
        if pre_seal_delay_ns > 0 {
            sleep_dur(Duration::from_nanos(pre_seal_delay_ns));
        }

        // PUBLISH COMPLETION STRICTLY LAST: atomically clears `in_flight_start_mono_ns = 0`,
        // samples the final release end timestamp strictly AFTER all release work and clearing
        // the in-flight marker, computes overrun skips from that timestamp, and publishes all
        // breach, peak, skip, marker, and `releases_total` counters under one `stats_seq` Release.
        stats.commit_release_outcome_with_clock(
            actual_ns,
            wake_lateness_ns,
            wake_skipped,
            0,
            &mut releases,
            now_ns,
        );
    };

    // Loop-exit final accounting pass (normal shutdown): if the loop exited after a release
    // without entering a genuine sleep, account any scheduled deadlines crossed before shutdown.
    let exit_skips = if ran_any_release && !stopped_cleanly {
        let sb = stop_bound_ns();
        releases.advance_overrun(if sb != 0 { sb } else { now_ns() })
    } else {
        0
    };
    stats.finalize_loop_exit(exit_skips);
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
        assert_eq!(r.next_ns(), 40);
        assert_eq!(r.missed_at_exit(25), 0);

        // Case 2: 25ms release on 20ms grid (starts at 20, ends at 45) -> 1 skip immediately,
        // next_ns advances past end_ns to 60 so continuing OR exiting yields 1 (never 0, never 2).
        let mut r = Releases { period_ns: 20, next_ns: 20 };
        assert_eq!(r.advance(20), 0); // next_ns = 40
        assert_eq!(r.advance_overrun(45), 1);
        assert_eq!(r.next_ns(), 60);
        assert_eq!(r.missed_at_exit(45), 0);
        assert_eq!(r.advance(60), 0); // next on-grid release at 60 adds 0
        assert_eq!(r.next_ns(), 80);

        // Exact boundary end_ns == next_ns also counts 1 immediately.
        let mut r = Releases { period_ns: 20, next_ns: 20 };
        assert_eq!(r.advance(20), 0); // next_ns = 40
        assert_eq!(r.advance_overrun(39), 0);
        assert_eq!(r.advance_overrun(40), 1);
        assert_eq!(r.next_ns(), 60);

        // Case 3: 12ms release on 5ms grid (starts at 5, ends at 17): skips 10 & 15 (2 total).
        let mut r = Releases { period_ns: 5, next_ns: 5 };
        assert_eq!(r.advance(5), 0); // next_ns = 10
        assert_eq!(r.advance_overrun(17), 2);
        assert_eq!(r.next_ns(), 20);
        assert_eq!(r.missed_at_exit(17), 0);
        assert_eq!(r.advance(20), 0);
        assert_eq!(r.next_ns(), 25);

        // Case 4: 45ms release on 10ms grid (starts at 30, ends at 75): skips 40, 50, 60, 70 (4 total).
        let mut r = Releases { period_ns: 10, next_ns: 30 };
        assert_eq!(r.advance(30), 0); // next_ns = 40
        assert_eq!(r.advance_overrun(75), 4);
        assert_eq!(r.next_ns(), 80);
        assert_eq!(r.missed_at_exit(75), 0);
        assert_eq!(r.advance(80), 0);
        assert_eq!(r.next_ns(), 90);
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
    fn run_periodic_with_deadline_tracks_scheduled_grid_after_overrun() {
        // Part 1: Real-clock overrun skips missed grid points at completion (`advance_overrun`)
        // and resynchronizes step 2 onto the exact scheduled grid without phase drift.
        let period = Duration::from_millis(20);
        let stop = AtomicBool::new(false);
        let stats = ExecutorStats::default();
        let mut step_idx = 0usize;
        let mut first_deadline: Option<Instant> = None;
        let mut second_deadline: Option<Instant> = None;
        run_periodic_with_deadline(period, &stop, &stats, |_dt, release_deadline| {
            step_idx += 1;
            if step_idx == 1 {
                first_deadline = Some(release_deadline);
                // Overrun past the 20 ms grid boundary into the middle of the next slot (+30 ms).
                std::thread::sleep(Duration::from_millis(30));
            } else if step_idx == 2 {
                second_deadline = Some(release_deadline);
                stop.store(true, Ordering::Release);
            }
        });
        let d1 = first_deadline.expect("step 1 deadline");
        let d2 = second_deadline.expect("step 2 deadline");
        assert!(
            stats.skipped_releases.load(Ordering::Acquire) >= 1,
            "30 ms overrun on 20 ms grid must account >= 1 skipped release"
        );
        // Step 2's deadline must align to an integer multiple (>= 2) of `period` from step 1's
        // deadline (within timer/Instant read jitter).
        let grid_delta = d2.saturating_duration_since(d1);
        let periods = ((grid_delta.as_nanos() as f64) / (period.as_nanos() as f64)).round();
        assert!(periods >= 2.0, "expected >= 2 period advance after 30 ms overrun, got {grid_delta:?}");
        let expected = Duration::from_nanos((periods as u64) * (period.as_nanos() as u64));
        let phase_err = grid_delta.abs_diff(expected);
        assert!(
            phase_err < Duration::from_millis(3),
            "scheduled grid phase drifted: d1->d2={grid_delta:?}, expected={expected:?}, err={phase_err:?}"
        );

        // Part 2: When a release wakes late into the middle of a grid slot (or crosses a grid
        // boundary due to wake lateness), `release_deadline` stays pinned to the scheduled grid
        // (`remaining < period`), NEVER `woke + period`.
        struct ScriptedWakeClock {
            origin: Instant,
            now_ns: AtomicU64,
            wakes: AtomicU64,
        }
        impl ExecutorClock for ScriptedWakeClock {
            fn now(&self) -> Instant {
                self.origin + Duration::from_nanos(self.now_ns.load(Ordering::Acquire))
            }
            fn now_ns(&self) -> u64 {
                self.now_ns.load(Ordering::Acquire)
            }
            fn sleep_until_ns(&self, target_ns: u64, _stop: &AtomicBool) -> bool {
                let cur = self.now_ns.load(Ordering::Acquire);
                if cur >= target_ns {
                    return false;
                }
                let wake_idx = self.wakes.fetch_add(1, Ordering::AcqRel);
                let lateness_ns = match wake_idx {
                    0 => 0,           // step 1: on time (+0 ms)
                    1 => 10_000_000,  // step 2: +10 ms late into 20 ms slot -> 10 ms remaining
                    _ => 30_000_000,  // step 3: +30 ms late across 1 slot -> 1 skip, 10 ms remaining
                };
                self.now_ns.store(target_ns + lateness_ns, Ordering::Release);
                true
            }
            fn sleep(&self, dur: Duration, _stop: &AtomicBool) {
                self.now_ns.fetch_add(dur.as_nanos() as u64, Ordering::AcqRel);
            }
        }
        let clock = ScriptedWakeClock {
            origin: Instant::now(),
            now_ns: AtomicU64::new(0),
            wakes: AtomicU64::new(0),
        };
        let stop2 = AtomicBool::new(false);
        let stats2 = ExecutorStats::default();
        let mut deadlines = Vec::new();
        let mut remainings = Vec::new();
        run_periodic_with_deadline_and_clock(period, &stop2, &stats2, Some(&clock), |_dt, deadline| {
            let now = clock.now();
            deadlines.push(deadline);
            remainings.push(deadline.saturating_duration_since(now));
            if deadlines.len() == 3 {
                stop2.store(true, Ordering::Release);
            }
        });
        assert_eq!(remainings.len(), 3);
        assert_eq!(remainings[0], Duration::from_millis(20));
        assert_eq!(
            remainings[1],
            Duration::from_millis(10),
            "late step 2 (+10 ms wake) must keep scheduled grid deadline (10 ms remaining, not woke + 20 ms)"
        );
        assert_eq!(
            remainings[2],
            Duration::from_millis(10),
            "late step 3 (+30 ms wake crossing 1 slot) must keep scheduled grid deadline (10 ms remaining)"
        );
        assert_eq!(deadlines[1].saturating_duration_since(deadlines[0]), Duration::from_millis(20));
        assert_eq!(deadlines[2].saturating_duration_since(deadlines[1]), Duration::from_millis(40));
        assert_eq!(stats2.skipped_releases.load(Ordering::Acquire), 1);
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
            hub: crate::rt_core::publish::state_channel(7).1,
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
                    returned_pending: None,
                    returned_samples: None,
                    returned_baseline: None,
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
    fn estop_registry_is_bounded_o1_and_prunes_closed_waiters() {
        let mut reg = EstopRegistry::default();
        let mut rxs = Vec::with_capacity(ESTOP_REGISTRY_CAPACITY);
        for g in 1..=(ESTOP_REGISTRY_CAPACITY as u64) {
            rxs.push(reg.register(g));
        }
        assert_eq!(reg.waiters.len(), ESTOP_REGISTRY_CAPACITY);
        // Full registry refuses without growing past ESTOP_REGISTRY_CAPACITY.
        let mut overflow_rx = reg.register(ESTOP_REGISTRY_CAPACITY as u64 + 1);
        assert_eq!(reg.waiters.len(), ESTOP_REGISTRY_CAPACITY);
        let err = overflow_rx.try_recv().expect("immediate rejection").unwrap_err();
        assert!(err.to_string().contains("ESTOP_QUEUE_FULL"), "{err}");

        // Dropping the oldest waiter lets the next register prune it in O(1) and succeed.
        drop(rxs.remove(0));
        let mut rx_reused = reg.register(ESTOP_REGISTRY_CAPACITY as u64 + 2);
        assert_eq!(reg.waiters.len(), ESTOP_REGISTRY_CAPACITY);
        assert!(rx_reused.try_recv().is_err(), "reused slot must stay pending until ACK");

        // Monotonic ACK pops all matching waiters from the front without allocation.
        reg.record_ack(EstopAck {
            first_gen: 1,
            last_gen: ESTOP_REGISTRY_CAPACITY as u64 + 2,
            state: Arc::new(SafetyState::EStop),
            epoch: 99,
        });
        assert!(reg.waiters.is_empty());
        assert_eq!(rx_reused.try_recv().unwrap().unwrap(), (SafetyState::EStop, 99));
        for mut rx in rxs {
            assert_eq!(rx.try_recv().unwrap().unwrap(), (SafetyState::EStop, 99));
        }
    }

    #[test]
    fn estop_registry_resolves_early_ack_and_out_of_order_off_reader() {
        let mut reg = EstopRegistry::default();
        // ACK for req=1 arrives and is drained BEFORE the off-reader task registers req=1.
        reg.record_ack(EstopAck {
            first_gen: 1,
            last_gen: 1,
            state: Arc::new(SafetyState::EStop),
            epoch: 11,
        });
        // Even after a later ACK for req=2 at epoch 15, registering req=1 resolves to pinned epoch 11.
        reg.record_ack(EstopAck {
            first_gen: 2,
            last_gen: 2,
            state: Arc::new(SafetyState::EStop),
            epoch: 15,
        });
        match reg.register_or_resolve(1) {
            EstopWait::Resolved(Ok((st, ep))) => assert_eq!((st, ep), (SafetyState::EStop, 11)),
            _ => panic!("early ACK for req=1 must resolve immediately to pinned epoch 11"),
        }
        // Out-of-order off-reader registration: req=5 registers before req=3 and req=4.
        let EstopWait::Pending(mut rx5) = reg.register_or_resolve(5) else { panic!("req 5 pending") };
        let EstopWait::Pending(mut rx3) = reg.register_or_resolve(3) else { panic!("req 3 pending") };
        let EstopWait::Pending(mut rx4) = reg.register_or_resolve(4) else { panic!("req 4 pending") };
        reg.record_ack(EstopAck {
            first_gen: 3,
            last_gen: 4,
            state: Arc::new(SafetyState::EStop),
            epoch: 21,
        });
        assert_eq!(rx3.try_recv().unwrap().unwrap(), (SafetyState::EStop, 21));
        assert_eq!(rx4.try_recv().unwrap().unwrap(), (SafetyState::EStop, 21));
        assert!(rx5.try_recv().is_err(), "req=5 must stay pending");
        reg.record_ack(EstopAck {
            first_gen: 5,
            last_gen: 5,
            state: Arc::new(SafetyState::EStop),
            epoch: 25,
        });
        assert_eq!(rx5.try_recv().unwrap().unwrap(), (SafetyState::EStop, 25));
        assert!(reg.waiters.is_empty());
    }

    #[test]
    fn exit_drain_returns_persist_payloads_without_mutating_live_state() {
        let mut core = SupervisorInner::new(crate::model::ArmConfig::rebot_dm());
        core.lease_enabled = false;
        let stamp = core.current_admission_stamp();
        let admitted = core.check_admission(stamp).unwrap();
        SupervisorInner::enable_torque(admitted).unwrap();
        let epoch = core.safety.epoch_id();
        let stamp = core.current_admission_stamp();
        let admitted = core.check_admission(stamp).unwrap();
        let (action_id, _) = SupervisorInner::start_gripper_zero_check(
            admitted,
            &serde_json::json!({"threshold_rad": 0.10, "store_offset": true}),
            epoch,
        )
        .unwrap();
        core.zero_check_persisting = Some(action_id);
        let action_before = core.last_action.clone().expect("running zero_check action");
        assert!(action_before.is_running());

        let (_, maint_req_rx) = RingBuffer::<MaintRequest>::new(2);
        let (maint_rep_tx, _) = RingBuffer::<MaintReply>::new(2);
        let (estop_ack_tx, _) = RingBuffer::<EstopAck>::new(2);
        let (reclaim_state_tx, _) = RingBuffer::<Arc<SafetyState>>::new(2);
        let (reclaim_stop_tx, _) = RingBuffer::<RetiredStopPayloads>::new(2);
        let (reclaim_traj_tx, _) = RingBuffer::<crate::motion::ActiveTrajectory>::new(2);
        let (reclaim_macro_tx, _) = RingBuffer::<crate::macro_action::MacroRun>::new(2);
        #[cfg(feature = "policy-runtime")]
        let (reclaim_policy_tx, _) = RingBuffer::<crate::policy_runtime::PolicyRun>::new(2);
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
            reclaim_stop_tx,
            reclaim_traj_tx,
            reclaim_macro_tx,
            #[cfg(feature = "policy-runtime")]
            reclaim_policy_tx,
            reclaim_return_rx,
            safety_log_tx,
            shutdown_hold_error: Arc::new(Mutex::new(None)),
            stats: Arc::new(ExecutorStats::default()),
            maint_req_rx,
            maint_rep_tx,
            pending_maint_reply: None,
            persist_job_tx,
            persist_cmd_rx,
            persist_rep_tx,
            watchdog_plan_rx: RingBuffer::<WatchdogParkPlan>::new(2).1,
            last_woken_watchdog_park: None,
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
    fn drain_retired_states_conserves_hold_error_counts_and_retries_immediately_on_overflow() {
        let mut core = SupervisorInner::new(crate::model::ArmConfig::rebot_dm());
        core.lease_enabled = false;
        {
            let stamp = core.current_admission_stamp();
            let admitted = core.check_admission(stamp).unwrap();
            SupervisorInner::enable_torque(admitted).unwrap();
        }
        core.rt_executor_owned = true;
        core.safety.set_off_thread_reclaim(true);

        let (_, maint_req_rx) = RingBuffer::<MaintRequest>::new(2);
        let (maint_rep_tx, _) = RingBuffer::<MaintReply>::new(2);
        let (estop_ack_tx, _) = RingBuffer::<EstopAck>::new(2);
        let (reclaim_state_tx, _) = RingBuffer::<Arc<SafetyState>>::new(2);
        let (reclaim_stop_tx, _) = RingBuffer::<RetiredStopPayloads>::new(2);
        let (reclaim_traj_tx, _) = RingBuffer::<crate::motion::ActiveTrajectory>::new(2);
        let (reclaim_macro_tx, mut reclaim_macro_rx) = RingBuffer::<crate::macro_action::MacroRun>::new(2);
        #[cfg(feature = "policy-runtime")]
        let (reclaim_policy_tx, _) = RingBuffer::<crate::policy_runtime::PolicyRun>::new(2);
        let (_, reclaim_return_rx) = RingBuffer::<Arc<SafetyState>>::new(2);
        let (mut safety_log_tx, mut safety_log_rx) = RingBuffer::<SafetyLogEvent>::new(2);
        let (persist_job_tx, _) = RingBuffer::<PendingZeroCheckPersist>::new(2);
        let (_, persist_cmd_rx) = RingBuffer::<PersistCmd>::new(2);
        let (persist_rep_tx, _) = RingBuffer::<PersistReply>::new(2);
        let (_, watchdog_plan_rx) = RingBuffer::<WatchdogParkPlan>::new(4);
        let (offload_reclaim_tx, _) = RingBuffer::<OffloadReclaim>::new(OFFLOAD_RECLAIM_CAPACITY);

        // Pre-fill downstream `safety_log_tx` (cap 2) with 2 unrelated events.
        safety_log_tx.push(SafetyLogEvent::GravityCompRampComplete).unwrap();
        safety_log_tx.push(SafetyLogEvent::GravityCompRampComplete).unwrap();

        let (_, _, _, stop_requests) = ReleaseEngine::new(BootId(1), Capacities::default());
        let mut state = ControlState {
            shutdown_committed: false,
            control_stop: Arc::new(AtomicBool::new(false)),
            stop_requests,
            estop_ack_tx,
            estop_overflow_gen: Arc::new(AtomicU64::new(0)),
            reclaim_state_tx,
            reclaim_stop_tx,
            reclaim_traj_tx,
            reclaim_macro_tx,
            #[cfg(feature = "policy-runtime")]
            reclaim_policy_tx,
            reclaim_return_rx,
            safety_log_tx,
            shutdown_hold_error: Arc::new(Mutex::new(None)),
            stats: Arc::new(ExecutorStats::default()),
            maint_req_rx,
            maint_rep_tx,
            pending_maint_reply: None,
            persist_job_tx,
            persist_cmd_rx,
            persist_rep_tx,
            pending_persist_reply: None,
            retained_persist_job: None,
            watchdog_plan_rx,
            last_woken_watchdog_park: None,
            offload_reclaim_tx,
            persist_wake: thread::current(),
            persist_granted: None,
            next_persist_req: 1,
            disconnect_code_buf: None,
            disconnect_msg_buf: None,
            sat_msg_buf: None,
            hooks: PersistTestHooks::default(),
        };

        let t0 = Instant::now();
        // 1. Hold error at t0 queues into inline ring, then `drain_retired_states` hits full
        //    downstream `safety_log_tx`: count conserved (1), NOT dropped, throttle rolled back!
        core.log_stopped_hold_output_failed_throttled(t0, false, "tx err #1");
        assert_eq!(core.hold_tx_error_suppressed(), 0);
        assert_eq!(core.last_hold_tx_error_log, Some(t0));
        state.drain_retired_states(&mut core, t0);
        assert_eq!(core.hold_tx_error_suppressed(), 1, "rejected event must fold back into suppressed counter");
        assert_eq!(core.safety_log_ring_dropped, 0, "hold error must not be dropped on downstream full");
        assert_eq!(core.last_hold_tx_error_log, None, "throttle timestamp must roll back when sink rejects");

        // 2. Consumer drains `safety_log_rx` just 20 ms later -> next hold error at t0 + 20 ms
        //    MUST deliver immediately (NOT wait 1 s!) with exact `suppressed == 1` (1 + 1 = 2)!
        assert!(matches!(safety_log_rx.pop(), Ok(SafetyLogEvent::GravityCompRampComplete)));
        assert!(matches!(safety_log_rx.pop(), Ok(SafetyLogEvent::GravityCompRampComplete)));
        let t1 = t0 + Duration::from_millis(20);
        core.log_stopped_hold_output_failed_throttled(t1, false, "tx err #2");
        state.drain_retired_states(&mut core, t1);
        match safety_log_rx.pop().expect("immediate delivery after consumer drain") {
            SafetyLogEvent::StoppedHoldOutputFailed { partial, suppressed, message, logged_at } => {
                assert!(!partial);
                assert_eq!(suppressed, 1, "must carry exact 1 prior rejected occurrence");
                assert_eq!(message.as_str(), "tx err #2");
                assert_eq!(logged_at, t1);
            }
            other => panic!("unexpected log event: {other:?}"),
        }
        assert_eq!(core.hold_tx_error_suppressed(), 0);
        assert_eq!(core.last_hold_tx_error_delivered, Some(t1));

        // 3. Subsequent error 20 ms later (within 1 s of delivered t1) IS throttled cleanly.
        let t2 = t1 + Duration::from_millis(20);
        core.log_stopped_hold_output_failed_throttled(t2, false, "tx err #3");
        state.drain_retired_states(&mut core, t2);
        assert!(safety_log_rx.pop().is_err(), "error within 1 s of delivered event must stay throttled");
        assert_eq!(core.hold_tx_error_suppressed(), 1);

        // 4. Delayed recovery (R2 Major 1): past 1 s (t3 = t1 + 1050 ms), downstream ring is full
        //    -> rejected (total 2). Consumer drains much later at t4 = t3 + 1200 ms with NO
        //    intervening hold error -> retry delivers with diagnostic `logged_at == t3`, AND
        //    starts the new 1 s throttle window at acceptance time `t4` so a new error 20 ms later
        //    at `t4 + 20 ms` IS throttled (no back-to-back burst!).
        state.safety_log_tx.push(SafetyLogEvent::GravityCompRampComplete).unwrap();
        state.safety_log_tx.push(SafetyLogEvent::GravityCompRampComplete).unwrap();
        let t3 = t1 + Duration::from_millis(1050);
        core.log_stopped_hold_output_failed_throttled(t3, true, "tx err #4");
        state.drain_retired_states(&mut core, t3);
        assert_eq!(core.hold_tx_error_suppressed(), 2);
        assert_eq!(core.last_hold_tx_error_log, Some(t1), "rolled back to last delivered t1");
        assert!(safety_log_rx.pop().is_ok());
        assert!(safety_log_rx.pop().is_ok());
        let t4 = t3 + Duration::from_millis(1200);
        state.drain_retired_states(&mut core, t4);
        match safety_log_rx.pop().expect("retained retry must flush as soon as downstream ring drains") {
            SafetyLogEvent::StoppedHoldOutputFailed { partial, suppressed, message, logged_at } => {
                assert!(partial);
                assert_eq!(suppressed, 1, "1 emitted + 1 suppressed == 2 exact occurrences (t2 + t3)");
                assert_eq!(message.as_str(), "tx err #4");
                assert_eq!(logged_at, t3, "event retains original occurrence timestamp for diagnostics");
            }
            other => panic!("unexpected log event: {other:?}"),
        }
        assert_eq!(core.hold_tx_error_suppressed(), 0);
        assert_eq!(core.last_hold_tx_error_log, Some(t4), "throttle window must start at retry acceptance t4");
        assert_eq!(core.last_hold_tx_error_delivered, Some(t4));
        // New error 20 ms after retry acceptance (`t4 + 20 ms`) MUST be throttled!
        let t5 = t4 + Duration::from_millis(20);
        core.log_stopped_hold_output_failed_throttled(t5, false, "tx err #5");
        state.drain_retired_states(&mut core, t5);
        assert!(safety_log_rx.pop().is_err(), "error 20 ms after delayed retry acceptance must be throttled");
        assert_eq!(core.hold_tx_error_suppressed(), 1);

        // 5. Production macro reclaim forwarding: fill downstream `reclaim_macro_tx` (cap 2) +
        //    local `retired_macro_runs` (cap 8) -> admission backpressures -> drain downstream ->
        //    forwarding frees local slots -> admission resumes!
        for _ in 0..10 {
            let ep = core.safety.epoch_id();
            let stamp = core.current_admission_stamp();
            let admitted = core.check_admission(stamp).unwrap();
            let _ = SupervisorInner::begin_sequence_action(admitted, None, ep).expect("admit macro");
            core.request_macro_stop();
            state.drain_retired_states(&mut core, t5);
        }
        assert!(!core.can_admit_macro_run(), "2 in reclaim_macro_tx + 8 in retired_macro_runs == saturated");
        let ep = core.safety.epoch_id();
        {
            let stamp = core.current_admission_stamp();
            let admitted = core.check_admission(stamp).unwrap();
            assert!(
                SupervisorInner::begin_sequence_action(admitted, None, ep).is_err(),
                "admission must backpressure when full"
            );
        }
        // Off-thread consumer drains `reclaim_macro_rx` -> next drain forwards -> admission resumes!
        assert!(reclaim_macro_rx.pop().is_ok());
        assert!(reclaim_macro_rx.pop().is_ok());
        state.drain_retired_states(&mut core, t5);
        assert!(core.can_admit_macro_run());
        {
            let stamp = core.current_admission_stamp();
            let admitted = core.check_admission(stamp).unwrap();
            assert!(SupervisorInner::begin_sequence_action(admitted, None, ep).is_ok());
        }

        // 6. R3 Major 4 (single-sink throttle enforcement): even if an extra
        //    `StoppedHoldOutputFailed` enters the inline ring within 1 s of the last accepted
        //    delivery (`t4`), `forward_deferred_safety_logs` enforces `window_ready` at the single
        //    push site so two events are NEVER accepted within one driver-clock second!
        let t6 = t4 + Duration::from_millis(500);
        let mut msg = crate::safety::BoundedText::<160>::empty();
        msg.push_str("tx err #6");
        assert!(core.safety.log_stopped_hold_output_failed(false, 0, msg, t6));
        state.drain_retired_states(&mut core, t6);
        assert!(
            safety_log_rx.pop().is_err(),
            "single-sink gate must never accept a second hold error within 1 s of delivered t4"
        );
        assert_eq!(core.hold_tx_error_suppressed(), 2, "t5 (1) + t6 (1) must be conserved");

        // 7. R3 Major 5 (lossless shutdown when downstream `safety_log_tx` remains full):
        //    fill `safety_log_tx` to capacity and advance past 1 s, run final drain + shutdown
        //    handoff -> exact pending aggregate (2 occurrences => suppressed == 1) transfers to
        //    `shutdown_hold_error` so `OffThreadSinks` emits it without losing any count!
        state.safety_log_tx.push(SafetyLogEvent::GravityCompRampComplete).unwrap();
        state.safety_log_tx.push(SafetyLogEvent::GravityCompRampComplete).unwrap();
        let t7 = t4 + Duration::from_millis(1200);
        state.drain_retired_states(&mut core, t7);
        assert_eq!(core.hold_tx_error_suppressed(), 2, "full ring keeps exact count pending");
        state.flush_pending_hold_error_for_shutdown(&mut core);
        assert_eq!(core.hold_tx_error_suppressed(), 0);
        let shutdown_ev = state
            .shutdown_hold_error
            .lock()
            .unwrap()
            .take()
            .expect("shutdown handoff must capture pending hold error when ring is full");
        match shutdown_ev {
            SafetyLogEvent::StoppedHoldOutputFailed { partial, suppressed, message, logged_at } => {
                assert!(!partial);
                assert_eq!(suppressed, 1, "1 emitted + 1 suppressed == 2 exact pending occurrences");
                assert_eq!(message.as_str(), "tx err #6");
                assert_eq!(logged_at, t6);
            }
            other => panic!("unexpected shutdown handoff event: {other:?}"),
        }

        // 8. Round 4 non-blocking teardown handoff: when `shutdown_hold_error` is contended,
        //    `flush_pending_hold_error_for_shutdown` MUST NOT block `armd-control`; it uses
        //    `try_lock()` and records a counted drop on both `core.safety_log_ring_dropped` and
        //    `state.stats.safety_log_dropped`.
        let t8 = t7 + Duration::from_millis(1200);
        let mut msg8 = crate::safety::BoundedText::<160>::empty();
        msg8.push_str("tx err #8");
        assert!(core.safety.log_stopped_hold_output_failed(false, 0, msg8, t8));
        state.drain_retired_states(&mut core, t8);
        assert_eq!(core.hold_tx_error_suppressed(), 1);
        let ring_dropped_before = core.safety_log_ring_dropped;
        let stats_dropped_before = state.stats.safety_log_dropped.load(Ordering::Relaxed);
        {
            let lock = Arc::clone(&state.shutdown_hold_error);
            let _held = lock.lock().unwrap();
            state.flush_pending_hold_error_for_shutdown(&mut core);
        }
        assert_eq!(core.hold_tx_error_suppressed(), 0);
        assert_eq!(core.safety_log_ring_dropped, ring_dropped_before + 1);
        assert_eq!(
            state.stats.safety_log_dropped.load(Ordering::Relaxed),
            stats_dropped_before + 1
        );
        assert!(state.shutdown_hold_error.lock().unwrap().is_none());
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
        let (reclaim_stop_tx, _reclaim_stop_rx) = RingBuffer::<RetiredStopPayloads>::new(2);
        let (reclaim_traj_tx, mut reclaim_traj_rx) =
            RingBuffer::<crate::motion::ActiveTrajectory>::new(2);
        let (reclaim_macro_tx, _reclaim_macro_rx) =
            RingBuffer::<crate::macro_action::MacroRun>::new(2);
        #[cfg(feature = "policy-runtime")]
        let (reclaim_policy_tx, _reclaim_policy_rx) =
            RingBuffer::<crate::policy_runtime::PolicyRun>::new(2);
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
            reclaim_stop_tx,
            reclaim_traj_tx,
            reclaim_macro_tx,
            #[cfg(feature = "policy-runtime")]
            reclaim_policy_tx,
            reclaim_return_rx,
            safety_log_tx,
            shutdown_hold_error: Arc::new(Mutex::new(None)),
            stats: Arc::new(ExecutorStats::default()),
            maint_req_rx,
            maint_rep_tx,
            pending_maint_reply: None,
            persist_job_tx,
            persist_cmd_rx,
            persist_rep_tx,
            watchdog_plan_rx: RingBuffer::<WatchdogParkPlan>::new(2).1,
            last_woken_watchdog_park: None,
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
        state.drain_retired_states(&mut core, Instant::now());
        assert_eq!(reclaim_return_tx.slots(), 1, "rejected return must stay queued while reclaim_state_tx is full");

        // Free 1 slot in `reclaim_state_rx`: `drain_retired_states` pops the rejected return and
        // forwards the exact same `Arc` into `reclaim_state_tx` (not dropped on RT).
        let _ = reclaim_state_rx.pop().unwrap();
        state.drain_retired_states(&mut core, Instant::now());
        assert_eq!(reclaim_return_tx.slots(), 2, "rejected return popped once reclaim_state_tx had a free slot");

        // Clean-exit drain while both rings remain full moves fault_b into lock-free `offload_reclaim_tx`.
        state.drain_estop_and_retired_stopped(&mut core, Instant::now());
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
        while let Ok(t) = reclaim_traj_rx.pop() { drop(t); }
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
        let (reclaim_stop_tx, _) = RingBuffer::<RetiredStopPayloads>::new(2);
        let (reclaim_traj_tx, _) = RingBuffer::<crate::motion::ActiveTrajectory>::new(2);
        let (reclaim_macro_tx, _) = RingBuffer::<crate::macro_action::MacroRun>::new(2);
        #[cfg(feature = "policy-runtime")]
        let (reclaim_policy_tx, _) = RingBuffer::<crate::policy_runtime::PolicyRun>::new(2);
        let (_reclaim_return_tx, reclaim_return_rx) = RingBuffer::<Arc<SafetyState>>::new(2);
        let (safety_log_tx, _safety_log_rx) = RingBuffer::<SafetyLogEvent>::new(2);
        let (persist_job_tx, _) = RingBuffer::<PendingZeroCheckPersist>::new(2);
        let (mut persist_cmd_tx, persist_cmd_rx) = RingBuffer::<PersistCmd>::new(2);
        let (mut persist_rep_tx, mut persist_rep_rx) = RingBuffer::<PersistReply>::new(PERSIST_REP_CAPACITY);
        let (mut offload_reclaim_tx, mut offload_reclaim_rx) = RingBuffer::<OffloadReclaim>::new(2);

        // Fill `maint_rep_tx` to full (2/2), `persist_rep_tx` to slots() == 1 (7/8),
        // and `offload_reclaim_tx` to full (2/2) to exercise retained-job fallback.
        for id in 1..=2 {
            maint_rep_tx
                .push(MaintReply {
                    id,
                    result: MaintenanceResult::ProbeReturned,
                    returned_offsets: None,
                    returned_probe: None,
                    returned_driver_config_error: None,
                    returned_pending: None,
                    returned_samples: None,
                    returned_baseline: None,
                })
                .unwrap();
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
            estop_epoch: 1,
            stop_gen: 0,
            deadline: Instant::now() + Duration::from_secs(5),
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
            reclaim_stop_tx,
            reclaim_traj_tx,
            reclaim_macro_tx,
            #[cfg(feature = "policy-runtime")]
            reclaim_policy_tx,
            reclaim_return_rx,
            safety_log_tx,
            shutdown_hold_error: Arc::new(Mutex::new(None)),
            stats: Arc::new(ExecutorStats::default()),
            maint_req_rx,
            maint_rep_tx,
            pending_maint_reply: None,
            persist_job_tx,
            persist_cmd_rx,
            persist_rep_tx,
            watchdog_plan_rx: RingBuffer::<WatchdogParkPlan>::new(2).1,
            last_woken_watchdog_park: None,
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
            base_revision: 0,
            lease_owner: None,
            estop_epoch: 1,
            stop_gen: 0,
            deadline: Instant::now() + Duration::from_secs(5),
            ctx: Box::new(crate::supervisor::ZeroCheckPersistContext {
                path: std::path::PathBuf::from("/tmp/armd-unused-test.json"),
                jaw_opening: None,
                stale: None,
                tool_id: "stock".into(),
                tool_config_digest: String::new(),
                active_tool_is_stock: true,
                base_gripper_policy: Some(crate::model::GripperPolicy::rebot_dm()),
                gripper_limit: None,
                operator_auth: None,
                sat_msg_buf: String::with_capacity(64),
            }),
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
