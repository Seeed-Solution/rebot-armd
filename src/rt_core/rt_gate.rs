//! Primitive 5: reusable real-time allocation and lock gate.
//!
//! Provides:
//! 1. Unified RT lock-boundary guard (`assert_rt_lock_free`, `RtGuardedMutex`, `RtGuardedRwLock`)
//!    and process-wide atomic attempt counter (`rt_lock_attempts`) enforced strictly before
//!    touching any guarded lock on a marked RT control thread (`on_rt_control_thread()`) or inside
//!    a marked release (`in_rt_release()`).
//! 2. Reusable zero-allocation and zero-lock gate (`RtReleaseGate`, `RtGateSnapshot`,
//!    `RtGateDelta`, and `measure_marked_release`) for executor phases and direct `ReleaseFixture`
//!    / `SupervisorInner` harnesses, asserting that the intended phase actually executed while
//!    recording strictly zero heap allocations, deallocations, and lock acquisitions.

use std::cell::{Cell, RefCell};
use std::fmt;
use std::future::Future;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{
    Arc, LockResult, Mutex, MutexGuard, RwLock, RwLockReadGuard, RwLockWriteGuard, TryLockResult,
};

use crate::diagnostics::alloc::{self, AllocCounts};
use crate::rt_core::executor::{
    assert_not_in_rt_release, assert_not_on_rt_control_thread, in_rt_release,
    on_rt_control_thread, ControlThreadGuard, ExecutorStats, ReleaseGuard, RtExecutorHandle,
};
use crate::supervisor::ArmSupervisor;

/// Process-wide counter of lock acquisition attempts rejected at any RT-guarded lock boundary
/// (`SupervisorLock`, `DriverLock`, `StateHub`, `RecordRing`, `RecorderAssembly`,
/// `TerminalCaptureChannel`, `ExecutorRecorder`, `UrgentStopClient`, `ConnLifecycleClient`,
/// `PublishedConfigHub`, `WsWireAckTracker`, `SerialDrainGate`, `MaintClient`,
/// `ServoTargetDispatcher`).
///
/// Incremented strictly BEFORE panicking and BEFORE touching the underlying lock.
/// Per-thread (`thread_rt_lock_attempts`) and per-executor (`ExecutorStats::rt_lock_attempts`)
/// counters are incremented alongside it so concurrent tests never cross-contaminate one another.
static RT_LOCK_ATTEMPTS: AtomicU64 = AtomicU64::new(0);

thread_local! {
    static THREAD_RT_LOCK_ATTEMPTS: Cell<u64> = const { Cell::new(0) };
    static BOUND_EXECUTOR_STATS: RefCell<Option<Arc<ExecutorStats>>> = const { RefCell::new(None) };
}

/// RAII guard binding `stats.rt_lock_attempts` to the current `armd-control` thread so rejected
/// lock attempts on this thread are attributed strictly to this executor instance.
pub(crate) struct ExecutorLockCounterGuard {
    prev: Option<Arc<ExecutorStats>>,
}

impl Drop for ExecutorLockCounterGuard {
    fn drop(&mut self) {
        let prev = self.prev.take();
        let _ = BOUND_EXECUTOR_STATS.try_with(|slot| {
            *slot.borrow_mut() = prev;
        });
    }
}

/// Binds an executor's [`ExecutorStats::rt_lock_attempts`] counter to the current thread for the
/// lifetime of the returned guard.
pub(crate) fn bind_executor_lock_counter(stats: Arc<ExecutorStats>) -> ExecutorLockCounterGuard {
    let prev = BOUND_EXECUTOR_STATS
        .try_with(|slot| slot.borrow_mut().replace(stats))
        .unwrap_or(None);
    ExecutorLockCounterGuard { prev }
}

/// Returns the cumulative number of lock acquisition attempts rejected across all RT-guarded
/// lock boundaries in this process.
#[inline]
pub fn rt_lock_attempts() -> u64 {
    RT_LOCK_ATTEMPTS.load(Ordering::Acquire)
}

/// Returns the cumulative number of lock acquisition attempts rejected on the **current thread**
/// across all RT-guarded lock boundaries (isolated from concurrent threads/tests).
#[inline]
pub fn thread_rt_lock_attempts() -> u64 {
    THREAD_RT_LOCK_ATTEMPTS.try_with(Cell::get).unwrap_or(0)
}

/// Records one rejected RT lock acquisition attempt in the process-wide, per-thread, and
/// (when bound on `armd-control`) per-executor counters.
///
/// Called by specialized locks (`SupervisorLock`, `DriverLock`, `RecorderAssembly`,
/// `TerminalCaptureChannel`, `ExecutorRecorder`) that maintain their own per-instance counters
/// in addition to these unified counters.
#[inline]
pub fn record_rt_lock_attempt() {
    RT_LOCK_ATTEMPTS.fetch_add(1, Ordering::SeqCst);
    let _ = THREAD_RT_LOCK_ATTEMPTS.try_with(|c| c.set(c.get().saturating_add(1)));
    let _ = BOUND_EXECUTOR_STATS.try_with(|slot| {
        if let Some(stats) = slot.borrow().as_ref() {
            stats.rt_lock_attempts.fetch_add(1, Ordering::SeqCst);
        }
    });
}

/// Asserts that the current thread is neither a marked RT control thread nor inside a marked
/// RT release. On violation, increments [`RT_LOCK_ATTEMPTS`] strictly before panicking and
/// before any underlying lock is touched.
#[inline]
pub fn assert_rt_lock_free(site: &'static str) {
    if on_rt_control_thread() {
        record_rt_lock_attempt();
        assert_not_on_rt_control_thread(site);
    }
    if in_rt_release() {
        record_rt_lock_attempt();
        assert_not_in_rt_release(site);
    }
}

/// `std::sync::Mutex` wrapper that rejects any `lock()` or `try_lock()` attempt from a marked
/// RT control thread or marked RT release via [`assert_rt_lock_free`], strictly before touching
/// the underlying mutex.
pub struct RtGuardedMutex<T: ?Sized> {
    site: &'static str,
    inner: Mutex<T>,
}

impl<T> RtGuardedMutex<T> {
    #[inline]
    pub const fn new(site: &'static str, value: T) -> Self {
        Self {
            site,
            inner: Mutex::new(value),
        }
    }
}

impl<T: ?Sized> RtGuardedMutex<T> {
    #[inline]
    pub fn lock(&self) -> LockResult<MutexGuard<'_, T>> {
        assert_rt_lock_free(self.site);
        self.inner.lock()
    }

    #[inline]
    pub fn try_lock(&self) -> TryLockResult<MutexGuard<'_, T>> {
        assert_rt_lock_free(self.site);
        self.inner.try_lock()
    }
}

impl<T: ?Sized + fmt::Debug> fmt::Debug for RtGuardedMutex<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RtGuardedMutex")
            .field("site", &self.site)
            .finish_non_exhaustive()
    }
}

/// `std::sync::RwLock` wrapper that rejects any `read()`, `try_read()`, `write()`, or
/// `try_write()` attempt from a marked RT control thread or marked RT release via
/// [`assert_rt_lock_free`], strictly before touching the underlying lock.
pub struct RtGuardedRwLock<T: ?Sized> {
    site: &'static str,
    inner: RwLock<T>,
}

impl<T> RtGuardedRwLock<T> {
    #[inline]
    pub const fn new(site: &'static str, value: T) -> Self {
        Self {
            site,
            inner: RwLock::new(value),
        }
    }
}

impl<T: ?Sized> RtGuardedRwLock<T> {
    #[inline]
    pub fn read(&self) -> LockResult<RwLockReadGuard<'_, T>> {
        assert_rt_lock_free(self.site);
        self.inner.read()
    }

    #[inline]
    pub fn try_read(&self) -> TryLockResult<RwLockReadGuard<'_, T>> {
        assert_rt_lock_free(self.site);
        self.inner.try_read()
    }

    #[inline]
    pub fn write(&self) -> LockResult<RwLockWriteGuard<'_, T>> {
        assert_rt_lock_free(self.site);
        self.inner.write()
    }

    #[inline]
    pub fn try_write(&self) -> TryLockResult<RwLockWriteGuard<'_, T>> {
        assert_rt_lock_free(self.site);
        self.inner.try_write()
    }
}

impl<T: ?Sized + fmt::Debug> fmt::Debug for RtGuardedRwLock<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RtGuardedRwLock")
            .field("site", &self.site)
            .finish_non_exhaustive()
    }
}

/// Point-in-time snapshot of RT release count, heap operation counters, and lock attempt counters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RtGateSnapshot {
    pub completed_releases: u64,
    pub release_allocs: AllocCounts,
    pub wake_allocs: AllocCounts,
    pub rt_lock_attempts: u64,
    pub supervisor_lock_attempts: u64,
    pub driver_lock_attempts: u64,
}

/// Delta of RT releases, heap operations, and lock attempts observed during one gated phase.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RtGateDelta {
    pub phase: &'static str,
    pub releases: u64,
    pub release_allocs: AllocCounts,
    pub wake_allocs: AllocCounts,
    pub rt_lock_attempts: u64,
    pub supervisor_lock_attempts: u64,
    pub driver_lock_attempts: u64,
}

impl RtGateDelta {
    /// Combined heap operation counts across Window-1 releases and Window-2 sub-release wakes.
    #[inline]
    pub fn total_allocs(&self) -> AllocCounts {
        AllocCounts {
            alloc: self.release_allocs.alloc.saturating_add(self.wake_allocs.alloc),
            alloc_zeroed: self
                .release_allocs
                .alloc_zeroed
                .saturating_add(self.wake_allocs.alloc_zeroed),
            realloc: self
                .release_allocs
                .realloc
                .saturating_add(self.wake_allocs.realloc),
            dealloc: self
                .release_allocs
                .dealloc
                .saturating_add(self.wake_allocs.dealloc),
        }
    }

    /// True iff at least one release completed with strictly zero allocations, zero deallocations,
    /// and zero lock attempts across all RT-guarded boundaries.
    #[inline]
    pub fn is_clean(&self) -> bool {
        self.releases > 0
            && self.release_allocs == AllocCounts::ZERO
            && self.wake_allocs == AllocCounts::ZERO
            && self.rt_lock_attempts == 0
            && self.supervisor_lock_attempts == 0
            && self.driver_lock_attempts == 0
    }

    /// Panics with a diagnostic report if fewer than `min_releases` ran or any heap operation or
    /// lock acquisition attempt occurred during the phase.
    pub fn assert_clean_with_min_releases(&self, min_releases: u64) {
        assert!(
            self.releases >= min_releases,
            "RT gate phase '{}' ran {} releases (expected >= {min_releases})",
            self.phase,
            self.releases
        );
        assert_eq!(
            self.release_allocs,
            AllocCounts::ZERO,
            "RT gate phase '{}' performed heap operations during release(s): {:?}",
            self.phase,
            self.release_allocs
        );
        assert_eq!(
            self.wake_allocs,
            AllocCounts::ZERO,
            "RT gate phase '{}' performed heap operations during sub-release wake(s): {:?}",
            self.phase,
            self.wake_allocs
        );
        assert_eq!(
            self.rt_lock_attempts,
            0,
            "RT gate phase '{}' attempted {} RT-guarded lock(s)",
            self.phase,
            self.rt_lock_attempts
        );
        assert_eq!(
            self.supervisor_lock_attempts,
            0,
            "RT gate phase '{}' attempted SupervisorLock {} time(s)",
            self.phase,
            self.supervisor_lock_attempts
        );
        assert_eq!(
            self.driver_lock_attempts,
            0,
            "RT gate phase '{}' attempted DriverLock {} time(s)",
            self.phase,
            self.driver_lock_attempts
        );
    }

    /// Panics if zero releases ran or any heap operation or lock acquisition attempt occurred.
    pub fn assert_clean(&self) {
        self.assert_clean_with_min_releases(1);
    }
}

/// Reusable RT allocation and lock gate bound to a live [`ArmSupervisor`] and [`RtExecutorHandle`].
pub struct RtReleaseGate<'a> {
    sup: &'a ArmSupervisor,
    handle: &'a RtExecutorHandle,
}

impl<'a> RtReleaseGate<'a> {
    pub fn new(sup: &'a ArmSupervisor, handle: &'a RtExecutorHandle) -> Self {
        Self { sup, handle }
    }

    /// Captures the current monotonic snapshot of releases, RT heap operations, and
    /// executor-attributed lock attempts (strictly isolated to this executor instance).
    pub fn snapshot(&self) -> RtGateSnapshot {
        let stats = self.handle.stats();
        RtGateSnapshot {
            completed_releases: stats.completed_releases.load(Ordering::Acquire),
            release_allocs: stats.release_allocs.snapshot(),
            wake_allocs: stats.wake_allocs.snapshot(),
            rt_lock_attempts: stats.rt_lock_attempts.load(Ordering::Acquire),
            supervisor_lock_attempts: self.sup.inner().control_thread_attempts(),
            driver_lock_attempts: self.sup.driver_lock_attempts(),
        }
    }

    /// Begins a named gated phase by recording the baseline counters.
    pub fn begin_phase(&self, phase: &'static str) -> RtPhaseGuard<'a> {
        RtPhaseGuard {
            gate: RtReleaseGate {
                sup: self.sup,
                handle: self.handle,
            },
            phase,
            before: self.snapshot(),
        }
    }

    /// Executes a synchronous phase body returning `(value, phase_ran)`, asserting that
    /// `phase_ran` is `true`, at least one RT release completed, and zero allocations,
    /// deallocations, or lock acquisitions occurred during the phase.
    pub fn run_phase<R>(&self, phase: &'static str, body: impl FnOnce() -> (R, bool)) -> (R, RtGateDelta) {
        let guard = self.begin_phase(phase);
        let (result, phase_ran) = body();
        let delta = guard.finish_and_assert(phase_ran);
        (result, delta)
    }

    /// Executes an async phase body returning `(value, phase_ran)`, asserting that `phase_ran`
    /// is `true`, at least one RT release completed, and zero allocations, deallocations, or
    /// lock acquisitions occurred during the phase.
    pub async fn run_phase_async<R, F>(&self, phase: &'static str, body: F) -> (R, RtGateDelta)
    where
        F: Future<Output = (R, bool)>,
    {
        let guard = self.begin_phase(phase);
        let (result, phase_ran) = body.await;
        let delta = guard.finish_and_assert(phase_ran);
        (result, delta)
    }
}

/// Active phase measurement guard produced by [`RtReleaseGate::begin_phase`].
pub struct RtPhaseGuard<'a> {
    gate: RtReleaseGate<'a>,
    phase: &'static str,
    before: RtGateSnapshot,
}

impl RtPhaseGuard<'_> {
    /// Computes the counter deltas since [`RtReleaseGate::begin_phase`] without asserting.
    pub fn finish(self) -> RtGateDelta {
        let after = self.gate.snapshot();
        RtGateDelta {
            phase: self.phase,
            releases: after
                .completed_releases
                .saturating_sub(self.before.completed_releases),
            release_allocs: after.release_allocs.delta_since(self.before.release_allocs),
            wake_allocs: after.wake_allocs.delta_since(self.before.wake_allocs),
            rt_lock_attempts: after
                .rt_lock_attempts
                .saturating_sub(self.before.rt_lock_attempts),
            supervisor_lock_attempts: after
                .supervisor_lock_attempts
                .saturating_sub(self.before.supervisor_lock_attempts),
            driver_lock_attempts: after
                .driver_lock_attempts
                .saturating_sub(self.before.driver_lock_attempts),
        }
    }

    /// Computes the counter deltas, asserts that the intended phase executed (`phase_ran == true`),
    /// and asserts strictly zero RT heap operations and zero lock attempts.
    pub fn finish_and_assert(self, phase_ran: bool) -> RtGateDelta {
        let phase = self.phase;
        let delta = self.finish();
        assert!(
            phase_ran,
            "RT gate phase '{phase}' did not execute its intended code path (releases={})",
            delta.releases
        );
        delta.assert_clean();
        delta
    }
}

/// Result of measuring one direct synchronous marked release via [`measure_marked_release`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MarkedReleaseDelta {
    pub phase: &'static str,
    pub allocs: AllocCounts,
    pub rt_lock_attempts: u64,
}

impl MarkedReleaseDelta {
    pub fn assert_clean(&self) {
        assert_eq!(
            self.allocs,
            AllocCounts::ZERO,
            "marked RT release '{}' performed heap operations: {:?}",
            self.phase,
            self.allocs
        );
        assert_eq!(
            self.rt_lock_attempts,
            0,
            "marked RT release '{}' attempted {} RT-guarded lock(s)",
            self.phase,
            self.rt_lock_attempts
        );
    }
}

/// Measures a direct synchronous RT release body (`ReleaseFixture`, `ModeSwitch`, `SupervisorInner`)
/// on the current thread inside both [`ControlThreadGuard`] and [`ReleaseGuard`], sampling
/// monotonic thread-local allocation counters ([`alloc::snapshot_counts`]) and thread-local
/// [`thread_rt_lock_attempts`] across all instrumented RT-guarded lock boundaries.
///
/// Callers must have [`crate::diagnostics::alloc::TimedAlloc`] installed as the binary's global
/// allocator and should run on a dedicated thread when asserting exact thread-local allocation
/// counts.
pub fn measure_marked_release<R>(
    phase: &'static str,
    body: impl FnOnce() -> (R, bool),
) -> (R, MarkedReleaseDelta) {
    alloc::time_this_thread();
    let _thread_guard = ControlThreadGuard::enter();
    let _release_guard = ReleaseGuard::enter();
    let before_alloc = alloc::snapshot_counts();
    let before_locks = thread_rt_lock_attempts();
    let (result, phase_ran) = body();
    let after_alloc = alloc::snapshot_counts();
    let after_locks = thread_rt_lock_attempts();
    drop(_release_guard);
    drop(_thread_guard);
    let delta = MarkedReleaseDelta {
        phase,
        allocs: after_alloc.delta_since(before_alloc),
        rt_lock_attempts: after_locks.saturating_sub(before_locks),
    };
    assert!(
        phase_ran,
        "marked RT release '{phase}' did not execute its intended code path"
    );
    (result, delta)
}
