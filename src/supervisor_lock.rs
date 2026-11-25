//! The supervisor's `RwLock`, with additions for the control tick:
//!
//! - a write generation, bumped each time a write guard is taken, so a reader can tell
//!   whether the observation the tick published is still the current state;
//! - a hold log: every guard of another task records, before it releases the lock, how long
//!   it held it, how long its `read()` / `write()` took, and the caller's source location
//!   (`#[track_caller]`). The record is written before the release because the release
//!   wakes the real-time tick, which on one core preempts the releasing task at once: a
//!   record written after the release would reach the log only after that tick had read it.
//!   A hold includes any time the holding task was preempted. A long acquire with a short
//!   hold means tokio handed the lock to a queued task that was not scheduled (a starved
//!   tokio worker): it blocked others from the hand-over on, without a guard existing.
//!
//! The tick reads the log with [`SupervisorLock::take_log`] while it holds the write guard;
//! no other guard exists then, so the log's mutex is free. `last_release` is the guard the
//! tick waited for (tokio grants the lock in queue order).
//!
//! There is no `Deref` to the inner `tokio::sync::RwLock`: every way to take a guard goes
//! through this type, so none is untimed and no write bypasses the generation.

use std::borrow::Borrow;
use std::future::Future;
use std::mem::ManuallyDrop;
use std::ops::{Deref, DerefMut};
use std::panic::Location;
use std::pin::Pin;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::task::{Context, Poll};
use std::time::{Duration, Instant};

use tokio::sync::{OwnedRwLockWriteGuard, RwLock, RwLockReadGuard, RwLockWriteGuard, TryLockError};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HoldKind { Read, Write }

impl HoldKind {
    pub fn as_str(self) -> &'static str { match self { Self::Read => "read", Self::Write => "write" } }
}

/// One guard of another task.
#[derive(Debug, Clone, Copy)]
pub struct HoldRecord {
    pub kind: HoldKind,
    /// Where `read()` / `write()` was called.
    pub location: &'static Location<'static>,
    /// Static WS/command path attributed to this hold (zero allocation on hot path).
    pub command: Option<&'static str>,
    /// Action id created or installed under this hold, if any.
    pub action_id: Option<u64>,
    /// `read()` / `write()` called.
    pub called: Instant,
    /// Guard handed to the caller.
    pub acquired: Instant,
    /// Recorded just before the release.
    pub released: Instant,
}

impl HoldRecord {
    pub fn hold(&self) -> Duration { self.released - self.acquired }
    pub fn acquire(&self) -> Duration { self.acquired - self.called }
}

/// Guards of other tasks released since the previous [`SupervisorLock::take_log`].
#[derive(Debug, Clone, Copy, Default)]
pub struct HoldLog {
    pub reader_hold: Duration,
    pub writer_hold: Duration,
    pub reader_acquire: Duration,
    pub writer_acquire: Duration,
    /// The last guard released (the one a waiting writer waited for).
    pub last_release: Option<HoldRecord>,
    /// The guard with the longest hold.
    pub longest: Option<HoldRecord>,
    /// The guard with the longest acquire.
    pub slowest_acquire: Option<HoldRecord>,
}

impl HoldLog {
    fn add(&mut self, r: HoldRecord) {
        let (hold, acquire) = (r.hold(), r.acquire());
        let (h, a) = match r.kind {
            HoldKind::Read => (&mut self.reader_hold, &mut self.reader_acquire),
            HoldKind::Write => (&mut self.writer_hold, &mut self.writer_acquire),
        };
        *h = (*h).max(hold);
        *a = (*a).max(acquire);
        if self.longest.is_none_or(|l| hold > l.hold()) { self.longest = Some(r); }
        if self.slowest_acquire.is_none_or(|l| acquire > l.acquire()) { self.slowest_acquire = Some(r); }
        self.last_release = Some(r);
    }
}

#[derive(Default)]
struct Holds {
    generation: AtomicU64,
    control_thread_attempts: AtomicU64,
    log: std::sync::Mutex<HoldLog>,
}

impl Holds {
    fn record(
        &self,
        kind: HoldKind,
        location: &'static Location<'static>,
        command: Option<&'static str>,
        action_id: Option<u64>,
        called: Instant,
        acquired: Instant,
    ) {
        let r = HoldRecord { kind, location, command, action_id, called, acquired, released: Instant::now() };
        self.log.lock().unwrap_or_else(|e| e.into_inner()).add(r);
    }

    /// Counts any forbidden acquisition/poll attempt on the marked RT control thread strictly
    /// before asserting and before touching the underlying lock, and independently enforces
    /// the `ReleaseGuard` invariant.
    #[inline]
    fn check_not_on_control_thread(&self, site: &'static str) {
        if crate::rt_core::executor::on_rt_control_thread() {
            self.control_thread_attempts.fetch_add(1, Ordering::SeqCst);
            crate::rt_core::executor::assert_not_on_rt_control_thread(site);
        }
        crate::rt_core::executor::assert_not_in_rt_release(site);
    }
}

/// Future wrapper checking [`Holds::check_not_on_control_thread`] at every poll before
/// polling the underlying `tokio::sync::RwLock` acquisition future.
struct GuardedFuture<H, F> {
    holds: H,
    site: &'static str,
    inner: F,
}

impl<H, F> Future for GuardedFuture<H, F>
where
    H: Borrow<Holds>,
    F: Future,
{
    type Output = F::Output;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // SAFETY: `holds` and `site` are never structurally pinned (only shared-borrowed
        // here); `inner` is structurally pinned because `GuardedFuture` never moves `inner`,
        // never exposes `&mut F` unpinned, has no `Drop` impl, and is only `Unpin` when
        // `H: Unpin` and `F: Unpin`.
        let this = unsafe { self.get_unchecked_mut() };
        this.holds.borrow().check_not_on_control_thread(this.site);
        // SAFETY: `inner` is structurally pinned within `self`.
        let inner = unsafe { Pin::new_unchecked(&mut this.inner) };
        inner.poll(cx)
    }
}

pub struct SupervisorLock<T> {
    lock: Arc<RwLock<T>>,
    holds: Arc<Holds>,
}

impl<T> SupervisorLock<T> {
    pub fn new(value: T) -> Self {
        Self { lock: Arc::new(RwLock::new(value)), holds: Arc::default() }
    }

    /// Write guards taken so far.
    pub fn generation(&self) -> u64 { self.holds.generation.load(Ordering::Acquire) }

    /// Forbidden acquisition or async-poll attempts observed on a marked RT control thread.
    /// Incremented before asserting and before touching the lock.
    pub fn control_thread_attempts(&self) -> u64 {
        self.holds.control_thread_attempts.load(Ordering::Acquire)
    }

    fn bump(&self) -> u64 { self.holds.generation.fetch_add(1, Ordering::AcqRel) + 1 }

    #[track_caller]
    pub fn read(&self) -> impl Future<Output = SupReadGuard<'_, T>> + '_ {
        let (location, called) = (Location::caller(), Instant::now());
        self.holds.check_not_on_control_thread("SupervisorLock::read");
        GuardedFuture {
            holds: self.holds.as_ref(),
            site: "SupervisorLock::read",
            inner: async move {
                let guard = self.lock.read().await;
                SupReadGuard { guard: ManuallyDrop::new(guard), t: Timing::new(location, called), holds: &self.holds }
            },
        }
    }

    #[track_caller]
    pub fn try_read(&self) -> Result<SupReadGuard<'_, T>, TryLockError> {
        let (location, called) = (Location::caller(), Instant::now());
        self.holds.check_not_on_control_thread("SupervisorLock::try_read");
        let guard = self.lock.try_read()?;
        Ok(SupReadGuard { guard: ManuallyDrop::new(guard), t: Timing::new(location, called), holds: &self.holds })
    }

    #[track_caller]
    pub fn write(&self) -> impl Future<Output = SupWriteGuard<'_, T>> + '_ {
        let (location, called) = (Location::caller(), Instant::now());
        self.holds.check_not_on_control_thread("SupervisorLock::write");
        GuardedFuture {
            holds: self.holds.as_ref(),
            site: "SupervisorLock::write",
            inner: async move {
                let guard = self.lock.write().await;
                self.bump();
                SupWriteGuard { guard: ManuallyDrop::new(guard), t: Timing::new(location, called), holds: &self.holds }
            },
        }
    }

    #[track_caller]
    pub fn try_write(&self) -> Result<SupWriteGuard<'_, T>, TryLockError> {
        let (location, called) = (Location::caller(), Instant::now());
        self.holds.check_not_on_control_thread("SupervisorLock::try_write");
        let guard = self.lock.try_write()?;
        self.bump();
        Ok(SupWriteGuard { guard: ManuallyDrop::new(guard), t: Timing::new(location, called), holds: &self.holds })
    }

    #[track_caller]
    pub fn blocking_write(&self) -> SupWriteGuard<'_, T> {
        let (location, called) = (Location::caller(), Instant::now());
        self.holds.check_not_on_control_thread("SupervisorLock::blocking_write");
        let guard = self.lock.blocking_write();
        self.bump();
        SupWriteGuard { guard: ManuallyDrop::new(guard), t: Timing::new(location, called), holds: &self.holds }
    }

    #[track_caller]
    pub fn write_owned(self: Arc<Self>) -> impl Future<Output = SupOwnedWriteGuard<T>> {
        let (location, called) = (Location::caller(), Instant::now());
        self.holds.check_not_on_control_thread("SupervisorLock::write_owned");
        let holds = Arc::clone(&self.holds);
        GuardedFuture {
            holds,
            site: "SupervisorLock::write_owned",
            inner: async move {
                let guard = self.lock.clone().write_owned().await;
                self.bump();
                SupOwnedWriteGuard { guard: ManuallyDrop::new(guard), t: Timing::new(location, called), holds: self.holds.clone() }
            },
        }
    }

    /// The control tick's write guard: counted in the generation (returned) but not logged,
    /// the tick times itself.
    pub fn write_tick(&self) -> impl Future<Output = (RwLockWriteGuard<'_, T>, u64)> + '_ {
        self.holds.check_not_on_control_thread("SupervisorLock::write_tick");
        GuardedFuture {
            holds: self.holds.as_ref(),
            site: "SupervisorLock::write_tick",
            inner: async move {
                let guard = self.lock.write().await;
                let generation = self.bump();
                (guard, generation)
            },
        }
    }

    /// The log since the previous call, reset. Never blocks: None when the log's mutex is
    /// taken (not expected while the caller holds the write guard).
    pub fn take_log(&self) -> Option<HoldLog> {
        match self.holds.log.try_lock() {
            Ok(mut log) => Some(std::mem::take(&mut *log)),
            Err(std::sync::TryLockError::Poisoned(p)) => Some(std::mem::take(&mut *p.into_inner())),
            Err(std::sync::TryLockError::WouldBlock) => None,
        }
    }
}

#[derive(Clone, Copy)]
struct Timing {
    location: &'static Location<'static>,
    command: Option<&'static str>,
    action_id: Option<u64>,
    called: Instant,
    acquired: Instant,
}

impl Timing {
    #[inline]
    fn new(location: &'static Location<'static>, called: Instant) -> Self {
        Self { location, command: None, action_id: None, called, acquired: Instant::now() }
    }
}

pub struct SupReadGuard<'a, T> {
    guard: ManuallyDrop<RwLockReadGuard<'a, T>>,
    t: Timing,
    holds: &'a Holds,
}

impl<T> Drop for SupReadGuard<'_, T> {
    fn drop(&mut self) {
        self.holds.record(HoldKind::Read, self.t.location, self.t.command, self.t.action_id, self.t.called, self.t.acquired);
        // SAFETY: dropped once, here; the field is not used afterwards.
        unsafe { ManuallyDrop::drop(&mut self.guard) };
    }
}

impl<T> Deref for SupReadGuard<'_, T> {
    type Target = T;
    fn deref(&self) -> &T { &self.guard }
}

pub struct SupWriteGuard<'a, T> {
    guard: ManuallyDrop<RwLockWriteGuard<'a, T>>,
    t: Timing,
    holds: &'a Holds,
}

impl<T> SupWriteGuard<'_, T> {
    /// Attribute this write-lock hold to a static command path (zero allocation).
    #[inline]
    pub fn set_command(&mut self, command: &'static str) {
        self.t.command = Some(command);
    }

    /// Attribute this write-lock hold to the action id created or installed under it (zero allocation).
    #[inline]
    pub fn set_action_id(&mut self, action_id: u64) {
        self.t.action_id = Some(action_id);
    }
}

impl<T> Drop for SupWriteGuard<'_, T> {
    fn drop(&mut self) {
        self.holds.record(HoldKind::Write, self.t.location, self.t.command, self.t.action_id, self.t.called, self.t.acquired);
        // SAFETY: dropped once, here; the field is not used afterwards.
        unsafe { ManuallyDrop::drop(&mut self.guard) };
    }
}

impl<T> Deref for SupWriteGuard<'_, T> {
    type Target = T;
    fn deref(&self) -> &T { &self.guard }
}

impl<T> DerefMut for SupWriteGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut T { &mut self.guard }
}

pub struct SupOwnedWriteGuard<T> {
    guard: ManuallyDrop<OwnedRwLockWriteGuard<T>>,
    t: Timing,
    holds: Arc<Holds>,
}

impl<T> SupOwnedWriteGuard<T> {
    /// Attribute this write-lock hold to a static command path (zero allocation).
    #[inline]
    pub fn set_command(&mut self, command: &'static str) {
        self.t.command = Some(command);
    }

    /// Attribute this write-lock hold to the action id created or installed under it (zero allocation).
    #[inline]
    pub fn set_action_id(&mut self, action_id: u64) {
        self.t.action_id = Some(action_id);
    }
}

impl<T> Drop for SupOwnedWriteGuard<T> {
    fn drop(&mut self) {
        self.holds.record(HoldKind::Write, self.t.location, self.t.command, self.t.action_id, self.t.called, self.t.acquired);
        // SAFETY: dropped once, here; the field is not used afterwards.
        unsafe { ManuallyDrop::drop(&mut self.guard) };
    }
}

impl<T> Deref for SupOwnedWriteGuard<T> {
    type Target = T;
    fn deref(&self) -> &T { &self.guard }
}

impl<T> DerefMut for SupOwnedWriteGuard<T> {
    fn deref_mut(&mut self) -> &mut T { &mut self.guard }
}

/// Non-blocking failure reason when trying to borrow a legacy [`DriverLock`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriverTryError {
    WouldBlock,
    Poisoned,
}

/// Guarded legacy mutex around `Box<dyn ArmDriver>`, mirroring [`SupervisorLock`].
///
/// Any `lock()` or `try_lock()` attempt on a marked RT control thread (`on_rt_control_thread()`)
/// increments `control_thread_attempts` strictly before asserting and before touching the
/// underlying `std::sync::Mutex`. Both methods also enforce `ReleaseGuard` on any thread.
pub struct DriverLock {
    lock: std::sync::Mutex<Box<dyn crate::driver::ArmDriver>>,
    control_thread_attempts: Arc<AtomicU64>,
}

impl DriverLock {
    pub fn new(driver: Box<dyn crate::driver::ArmDriver>) -> Self {
        Self::with_counter(driver, Arc::new(AtomicU64::new(0)))
    }

    pub fn with_counter(
        driver: Box<dyn crate::driver::ArmDriver>,
        control_thread_attempts: Arc<AtomicU64>,
    ) -> Self {
        Self {
            lock: std::sync::Mutex::new(driver),
            control_thread_attempts,
        }
    }

    pub fn from_mutex(
        lock: std::sync::Mutex<Box<dyn crate::driver::ArmDriver>>,
        control_thread_attempts: Arc<AtomicU64>,
    ) -> Self {
        Self {
            lock,
            control_thread_attempts,
        }
    }

    /// Forbidden acquisition attempts observed on a marked RT control thread.
    pub fn control_thread_attempts(&self) -> u64 {
        self.control_thread_attempts.load(Ordering::Acquire)
    }

    /// Rebinds the shared control-thread attempt counter (e.g. when attaching or rolling back).
    pub fn attach_counter(&mut self, control_thread_attempts: Arc<AtomicU64>) {
        self.control_thread_attempts = control_thread_attempts;
    }

    pub fn is_poisoned(&self) -> bool {
        self.lock.is_poisoned()
    }

    #[inline]
    fn check_not_on_control_thread(&self, site: &'static str) {
        if crate::rt_core::executor::on_rt_control_thread() {
            self.control_thread_attempts.fetch_add(1, Ordering::SeqCst);
            crate::rt_core::executor::assert_not_on_rt_control_thread(site);
        }
        crate::rt_core::executor::assert_not_in_rt_release(site);
    }

    pub fn lock(
        &self,
    ) -> std::sync::LockResult<std::sync::MutexGuard<'_, Box<dyn crate::driver::ArmDriver>>> {
        self.check_not_on_control_thread("DriverLock::lock");
        self.lock.lock()
    }

    pub fn try_lock(
        &self,
    ) -> std::sync::TryLockResult<std::sync::MutexGuard<'_, Box<dyn crate::driver::ArmDriver>>> {
        self.check_not_on_control_thread("DriverLock::try_lock");
        self.lock.try_lock()
    }

    /// Consumes the legacy mutex and returns the exclusive `Box<dyn ArmDriver>` for RT handover.
    /// Fails closed if poisoned without losing poison state or dropping the driver.
    pub fn into_inner(self) -> Result<Box<dyn crate::driver::ArmDriver>, (Self, &'static str)> {
        if self.lock.is_poisoned() {
            return Err((self, "driver mutex poisoned"));
        }
        match self.lock.into_inner() {
            Ok(drv) => Ok(drv),
            Err(poisoned) => {
                // Defensive: `is_poisoned()` was checked above under exclusive ownership of `self`,
                // so this branch is unreachable in practice; preserve the driver if ever hit.
                let counter = self.control_thread_attempts;
                Err((
                    Self::with_counter(poisoned.into_inner(), counter),
                    "driver mutex poisoned",
                ))
            }
        }
    }

    /// Extracts the underlying `Box<dyn ArmDriver>` even if poisoned, strictly for Legacy
    /// SAFE-HOLD recovery after a control-thread unwind (single exclusive writer, zero locks
    /// thereafter).
    pub fn into_inner_recover_for_safe_hold(self) -> Box<dyn crate::driver::ArmDriver> {
        match self.lock.into_inner() {
            Ok(drv) => drv,
            Err(poisoned) => poisoned.into_inner(),
        }
    }

    /// Extracts the underlying `std::sync::Mutex` (for legacy multi-thread contention tests).
    #[doc(hidden)]
    pub fn into_raw_mutex(self) -> std::sync::Mutex<Box<dyn crate::driver::ArmDriver>> {
        self.lock
    }

    /// Borrows the underlying `std::sync::Mutex` (for legacy contention tests on a non-RT thread).
    #[doc(hidden)]
    pub fn raw_mutex(&self) -> &std::sync::Mutex<Box<dyn crate::driver::ArmDriver>> {
        &self.lock
    }
}

/// Single-thread-pinned wrapper for an exclusively owned `Box<dyn ArmDriver>`.
///
/// `ExclusiveDriver` is `Send` (transferring ownership to `armd-control` or back on rollback) and
/// `Sync` solely because immutable `&self` access enforces at runtime that the caller is the
/// single OS thread currently bound to `owner_thread`. Because `owner_thread` can only be changed
/// through `&mut self` (when no `&self` references exist), two threads can never observe
/// `&dyn ArmDriver` concurrently, preserving Rust soundness for `!Sync` drivers with zero locks,
/// zero atomics, and zero heap allocations on the RT path.
pub struct ExclusiveDriver {
    driver: Box<dyn crate::driver::ArmDriver>,
    owner_thread: u64,
}

static NEXT_EXCLUSIVE_THREAD_KEY: AtomicU64 = AtomicU64::new(1);

thread_local! {
    static EXCLUSIVE_THREAD_KEY: u64 = NEXT_EXCLUSIVE_THREAD_KEY
        .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |k| k.checked_add(1))
        .expect("exclusive driver thread key exhausted");
}

#[inline]
fn current_exclusive_thread_key() -> u64 {
    EXCLUSIVE_THREAD_KEY.with(|&k| k)
}

impl ExclusiveDriver {
    pub fn new(driver: Box<dyn crate::driver::ArmDriver>) -> Self {
        Self {
            driver,
            owner_thread: current_exclusive_thread_key(),
        }
    }

    #[inline]
    pub fn bind_current_thread(&mut self) {
        self.owner_thread = current_exclusive_thread_key();
    }

    #[inline]
    pub fn get_ref(&self) -> &(dyn crate::driver::ArmDriver + 'static) {
        let cur = current_exclusive_thread_key();
        assert_eq!(
            self.owner_thread, cur,
            "ExclusiveDriver immutable access from non-owning thread (owner_key={}, current_key={cur})",
            self.owner_thread
        );
        self.driver.as_ref()
    }

    #[inline]
    pub fn get_mut(&mut self) -> &mut Box<dyn crate::driver::ArmDriver> {
        self.owner_thread = current_exclusive_thread_key();
        &mut self.driver
    }

    #[inline]
    pub fn into_inner(self) -> Box<dyn crate::driver::ArmDriver> {
        self.driver
    }
}

// SAFETY: `ExclusiveDriver` auto-implements `Send` (`Box<dyn ArmDriver>: Send`, `u64: Send`).
// `&ExclusiveDriver` only exposes `&self.driver` after asserting
// `self.owner_thread == current_exclusive_thread_key()`. Since `owner_thread` is only mutated
// via `&mut self` and thread keys are allocated monotonically via checked_add (never reused),
// concurrent `&ExclusiveDriver` accesses can succeed on at most one OS thread, so `&dyn ArmDriver`
// is never shared across threads.
unsafe impl Sync for ExclusiveDriver {}

/// Single-owner driver storage on [`crate::supervisor::SupervisorInner`].
///
/// - `Legacy(DriverLock)`: guarded mutex used under `OWNER_LEGACY` (supports `&self` readers
///   behind `SupervisorLock::read()` and legacy poison/contention paths).
/// - `Exclusive(ExclusiveDriver)`: exclusive mutable value moved into the RT executor at
///   handover — zero mutex, zero atomic lock state, zero poison checks on `armd-control`.
pub enum DriverSlot {
    Legacy(DriverLock),
    Exclusive(ExclusiveDriver),
}

impl DriverSlot {
    pub fn legacy(driver: Box<dyn crate::driver::ArmDriver>) -> Self {
        Self::Legacy(DriverLock::new(driver))
    }

    pub fn legacy_with_counter(
        driver: Box<dyn crate::driver::ArmDriver>,
        counter: Arc<AtomicU64>,
    ) -> Self {
        Self::Legacy(DriverLock::with_counter(driver, counter))
    }

    pub fn from_mutex(mutex: std::sync::Mutex<Box<dyn crate::driver::ArmDriver>>) -> Self {
        Self::Legacy(DriverLock::from_mutex(mutex, Arc::new(AtomicU64::new(0))))
    }

    pub fn from_mutex_with_counter(
        mutex: std::sync::Mutex<Box<dyn crate::driver::ArmDriver>>,
        counter: Arc<AtomicU64>,
    ) -> Self {
        Self::Legacy(DriverLock::from_mutex(mutex, counter))
    }

    pub fn exclusive(driver: Box<dyn crate::driver::ArmDriver>) -> Self {
        Self::Exclusive(ExclusiveDriver::new(driver))
    }

    #[inline]
    pub fn bind_current_thread(&mut self) {
        if let Self::Exclusive(drv) = self {
            drv.bind_current_thread();
        }
    }

    #[inline]
    pub fn is_exclusive(&self) -> bool {
        matches!(self, Self::Exclusive(_))
    }

    #[inline]
    pub fn is_legacy(&self) -> bool {
        matches!(self, Self::Legacy(_))
    }

    pub fn attach_counter(&mut self, counter: Arc<AtomicU64>) {
        if let Self::Legacy(lock) = self {
            lock.attach_counter(counter);
        }
    }

    /// Converts a `Legacy(DriverLock)` slot into `Exclusive(ExclusiveDriver)` for RT handover.
    /// Fails closed if the legacy mutex is poisoned, returning the intact `Legacy` slot.
    pub fn into_exclusive(self) -> Result<Self, (Self, &'static str)> {
        match self {
            Self::Exclusive(drv) => Ok(Self::Exclusive(drv)),
            Self::Legacy(lock) => lock
                .into_inner()
                .map(Self::exclusive)
                .map_err(|(lock, reason)| (Self::Legacy(lock), reason)),
        }
    }

    /// Extracts the underlying `Box<dyn ArmDriver>` unconditionally (recovering legacy mutex
    /// poison if present), strictly for single-writer SAFE-HOLD recovery after control panic.
    pub fn into_driver_recover_for_safe_hold(self) -> Box<dyn crate::driver::ArmDriver> {
        match self {
            Self::Exclusive(drv) => drv.into_inner(),
            Self::Legacy(lock) => lock.into_inner_recover_for_safe_hold(),
        }
    }

    /// Wraps an `Exclusive` driver back into `Legacy(DriverLock)` on recoverable start failure,
    /// or rebinds the shared counter if already `Legacy`.
    pub fn into_legacy(self, counter: Arc<AtomicU64>) -> Self {
        match self {
            Self::Legacy(mut lock) => {
                lock.attach_counter(counter);
                Self::Legacy(lock)
            }
            Self::Exclusive(drv) => Self::Legacy(DriverLock::with_counter(drv.into_inner(), counter)),
        }
    }

    #[inline]
    pub fn as_exclusive_ref(&self) -> Option<&dyn crate::driver::ArmDriver> {
        match self {
            Self::Exclusive(drv) => Some(drv.get_ref()),
            Self::Legacy(_) => None,
        }
    }

    #[inline]
    pub fn as_exclusive_mut(&mut self) -> Option<&mut dyn crate::driver::ArmDriver> {
        match self {
            Self::Exclusive(drv) => Some(drv.get_mut().as_mut()),
            Self::Legacy(_) => None,
        }
    }

    #[inline]
    pub fn as_legacy_lock(&self) -> Option<&DriverLock> {
        match self {
            Self::Legacy(lock) => Some(lock),
            Self::Exclusive(_) => None,
        }
    }

    /// Extracts the underlying `std::sync::Mutex` if in `Legacy` mode (for legacy contention tests).
    #[doc(hidden)]
    pub fn into_legacy_mutex(self) -> Option<std::sync::Mutex<Box<dyn crate::driver::ArmDriver>>> {
        match self {
            Self::Legacy(lock) => Some(lock.into_raw_mutex()),
            Self::Exclusive(_) => None,
        }
    }

    /// Borrows the driver immutably: zero-lock direct borrow in `Exclusive`, guarded `lock()` in
    /// `Legacy` (returning `None` if poisoned).
    #[inline]
    pub fn with_ref<R>(&self, f: impl FnOnce(&dyn crate::driver::ArmDriver) -> R) -> Option<R> {
        match self {
            Self::Exclusive(drv) => Some(f(drv.get_ref())),
            Self::Legacy(lock) => lock.lock().ok().map(|g| f(g.as_ref())),
        }
    }

    /// Borrows the driver immutably, recovering from poison in `Legacy` mode.
    #[inline]
    pub fn with_ref_recover<R>(&self, f: impl FnOnce(&dyn crate::driver::ArmDriver) -> R) -> R {
        match self {
            Self::Exclusive(drv) => f(drv.get_ref()),
            Self::Legacy(lock) => {
                let g = lock.lock().unwrap_or_else(|e| e.into_inner());
                f(g.as_ref())
            }
        }
    }

    /// Non-blocking immutable borrow: zero-lock in `Exclusive`, guarded `try_lock()` in `Legacy`.
    #[inline]
    pub fn try_with_ref<R>(
        &self,
        f: impl FnOnce(&dyn crate::driver::ArmDriver) -> R,
    ) -> Result<R, DriverTryError> {
        match self {
            Self::Exclusive(drv) => Ok(f(drv.get_ref())),
            Self::Legacy(lock) => match lock.try_lock() {
                Ok(g) => Ok(f(g.as_ref())),
                Err(std::sync::TryLockError::WouldBlock) => Err(DriverTryError::WouldBlock),
                Err(std::sync::TryLockError::Poisoned(_)) => Err(DriverTryError::Poisoned),
            },
        }
    }

    /// Non-blocking immutable borrow that recovers from poison in `Legacy` (returns `None` only on
    /// `Legacy` `WouldBlock`). Zero-lock in `Exclusive`.
    #[inline]
    pub fn try_with_ref_recover<R>(
        &self,
        f: impl FnOnce(&dyn crate::driver::ArmDriver) -> R,
    ) -> Option<R> {
        match self {
            Self::Exclusive(drv) => Some(f(drv.get_ref())),
            Self::Legacy(lock) => match lock.try_lock() {
                Ok(g) => Some(f(g.as_ref())),
                Err(std::sync::TryLockError::Poisoned(p)) => Some(f(p.into_inner().as_ref())),
                Err(std::sync::TryLockError::WouldBlock) => None,
            },
        }
    }

    /// Borrows the driver mutably: zero-lock direct borrow in `Exclusive`, guarded `lock()` in
    /// `Legacy` (returning `None` if poisoned).
    #[inline]
    pub fn with_mut<R>(
        &mut self,
        f: impl FnOnce(&mut dyn crate::driver::ArmDriver) -> R,
    ) -> Option<R> {
        match self {
            Self::Exclusive(drv) => Some(f(drv.get_mut().as_mut())),
            Self::Legacy(lock) => lock.lock().ok().map(|mut g| f(&mut **g)),
        }
    }

    /// Borrows the driver mutably, recovering from poison in `Legacy` mode.
    #[inline]
    pub fn with_mut_recover<R>(
        &mut self,
        f: impl FnOnce(&mut dyn crate::driver::ArmDriver) -> R,
    ) -> R {
        match self {
            Self::Exclusive(drv) => f(drv.get_mut().as_mut()),
            Self::Legacy(lock) => {
                let mut g = lock.lock().unwrap_or_else(|e| e.into_inner());
                f(&mut **g)
            }
        }
    }

    /// Non-blocking mutable borrow: zero-lock in `Exclusive`, guarded `try_lock()` in `Legacy`.
    #[inline]
    pub fn try_with_mut<R>(
        &mut self,
        f: impl FnOnce(&mut dyn crate::driver::ArmDriver) -> R,
    ) -> Result<R, DriverTryError> {
        match self {
            Self::Exclusive(drv) => Ok(f(drv.get_mut().as_mut())),
            Self::Legacy(lock) => match lock.try_lock() {
                Ok(mut g) => Ok(f(&mut **g)),
                Err(std::sync::TryLockError::WouldBlock) => Err(DriverTryError::WouldBlock),
                Err(std::sync::TryLockError::Poisoned(_)) => Err(DriverTryError::Poisoned),
            },
        }
    }

    /// Non-blocking mutable borrow that recovers from poison in `Legacy` (returns `None` only on
    /// `Legacy` `WouldBlock`). Zero-lock in `Exclusive`.
    #[inline]
    pub fn try_with_mut_recover<R>(
        &mut self,
        f: impl FnOnce(&mut dyn crate::driver::ArmDriver) -> R,
    ) -> Option<R> {
        match self {
            Self::Exclusive(drv) => Some(f(drv.get_mut().as_mut())),
            Self::Legacy(lock) => match lock.try_lock() {
                Ok(mut g) => Some(f(&mut **g)),
                Err(std::sync::TryLockError::Poisoned(p)) => Some(f(&mut **p.into_inner())),
                Err(std::sync::TryLockError::WouldBlock) => None,
            },
        }
    }

    /// Borrows the driver immutably as a guard: zero-lock reference in `Exclusive`,
    /// guarded `DriverLock::lock()` in `Legacy` (`None` if poisoned).
    #[inline]
    pub fn read(&self) -> Option<DriverReadGuard<'_>> {
        match self {
            Self::Exclusive(drv) => Some(DriverReadGuard::Exclusive(drv.get_ref())),
            Self::Legacy(lock) => lock.lock().ok().map(DriverReadGuard::Legacy),
        }
    }

    /// Non-blocking immutable guard: zero-lock reference in `Exclusive`,
    /// guarded `DriverLock::try_lock()` in `Legacy` (`None` if contended or poisoned).
    #[inline]
    pub fn try_read(&self) -> Option<DriverReadGuard<'_>> {
        match self {
            Self::Exclusive(drv) => Some(DriverReadGuard::Exclusive(drv.get_ref())),
            Self::Legacy(lock) => lock.try_lock().ok().map(DriverReadGuard::Legacy),
        }
    }

    /// Borrows the driver mutably as a guard: zero-lock mutable reference in `Exclusive`,
    /// guarded `DriverLock::lock()` in `Legacy` (`None` if poisoned).
    #[inline]
    pub fn write(&mut self) -> Option<DriverWriteGuard<'_>> {
        match self {
            Self::Exclusive(drv) => Some(DriverWriteGuard::Exclusive(drv.get_mut())),
            Self::Legacy(lock) => lock.lock().ok().map(DriverWriteGuard::Legacy),
        }
    }

    /// Non-blocking mutable guard: zero-lock mutable reference in `Exclusive`,
    /// guarded `DriverLock::try_lock()` in `Legacy` (`None` if contended or poisoned).
    #[inline]
    pub fn try_write(&mut self) -> Option<DriverWriteGuard<'_>> {
        match self {
            Self::Exclusive(drv) => Some(DriverWriteGuard::Exclusive(drv.get_mut())),
            Self::Legacy(lock) => lock.try_lock().ok().map(DriverWriteGuard::Legacy),
        }
    }
}

/// Shared borrow guard returned by [`DriverSlot::read`].
pub enum DriverReadGuard<'a> {
    Legacy(std::sync::MutexGuard<'a, Box<dyn crate::driver::ArmDriver>>),
    Exclusive(&'a (dyn crate::driver::ArmDriver + 'static)),
}

impl Deref for DriverReadGuard<'_> {
    type Target = dyn crate::driver::ArmDriver + 'static;
    #[inline]
    fn deref(&self) -> &Self::Target {
        match self {
            Self::Legacy(g) => g.as_ref(),
            Self::Exclusive(drv) => *drv,
        }
    }
}

impl AsRef<dyn crate::driver::ArmDriver + 'static> for DriverReadGuard<'_> {
    #[inline]
    fn as_ref(&self) -> &(dyn crate::driver::ArmDriver + 'static) {
        &**self
    }
}

/// Exclusive mutable borrow guard returned by [`DriverSlot::write`].
pub enum DriverWriteGuard<'a> {
    Legacy(std::sync::MutexGuard<'a, Box<dyn crate::driver::ArmDriver>>),
    Exclusive(&'a mut Box<dyn crate::driver::ArmDriver>),
}

impl Deref for DriverWriteGuard<'_> {
    type Target = Box<dyn crate::driver::ArmDriver>;
    #[inline]
    fn deref(&self) -> &Self::Target {
        match self {
            Self::Legacy(g) => g,
            Self::Exclusive(drv) => drv,
        }
    }
}

impl DerefMut for DriverWriteGuard<'_> {
    #[inline]
    fn deref_mut(&mut self) -> &mut Self::Target {
        match self {
            Self::Legacy(g) => g,
            Self::Exclusive(drv) => drv,
        }
    }
}

impl From<std::sync::Mutex<Box<dyn crate::driver::ArmDriver>>> for DriverSlot {
    fn from(mutex: std::sync::Mutex<Box<dyn crate::driver::ArmDriver>>) -> Self {
        Self::from_mutex(mutex)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn generation_counts_write_guards_and_holds_are_logged() {
        let lock = Arc::new(SupervisorLock::new(0u32));
        assert_eq!(lock.generation(), 0);
        { let _r = lock.read().await; }
        { let _r = lock.try_read().unwrap(); }
        assert_eq!(lock.generation(), 0);
        { *lock.write().await += 1; }
        { *lock.try_write().unwrap() += 1; }
        { *lock.clone().write_owned().await += 1; }
        let (g, generation) = lock.write_tick().await;
        assert_eq!(generation, 4);
        drop(g);
        assert_eq!(lock.generation(), 4);
        assert_eq!(*lock.read().await, 3);
        lock.take_log();
        let line = line!() + 1;
        let r = lock.read().await;
        std::thread::sleep(Duration::from_millis(5));
        drop(r);
        let log = lock.take_log().unwrap();
        assert!(log.reader_hold >= Duration::from_millis(5), "{log:?}");
        assert!(log.reader_acquire < Duration::from_millis(5), "{log:?}");
        assert_eq!(log.writer_hold, Duration::ZERO);
        let longest = log.longest.unwrap();
        assert_eq!((longest.kind, longest.location.file(), longest.location.line()), (HoldKind::Read, file!(), line));
        assert!(lock.take_log().unwrap().last_release.is_none());
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_writer_that_waited_names_the_reader_it_waited_for() {
        let lock = Arc::new(SupervisorLock::new(0u32));
        let line = line!() + 1;
        let r = lock.read().await;
        let w = tokio::spawn({
            let lock = lock.clone();
            async move {
                let (g, _) = lock.write_tick().await;
                // what the tick does under its guard
                let log = lock.take_log().unwrap();
                drop(g);
                log
            }
        });
        tokio::time::sleep(Duration::from_millis(10)).await;
        drop(r);
        let log = w.await.unwrap();
        let last = log.last_release.unwrap();
        assert_eq!((last.kind, last.location.line()), (HoldKind::Read, line));
        assert!(last.hold() >= Duration::from_millis(10), "{last:?}");
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_queued_writer_records_its_acquire_time() {
        let lock = Arc::new(SupervisorLock::new(0u32));
        let r = lock.read().await;
        let (tx, rx) = tokio::sync::oneshot::channel();
        let w = tokio::spawn({
            let lock = lock.clone();
            async move {
                let write = lock.write();
                tx.send(()).unwrap();
                *write.await += 1;
            }
        });
        rx.await.unwrap();
        tokio::time::sleep(Duration::from_millis(10)).await;
        drop(r);
        w.await.unwrap();
        let log = lock.take_log().unwrap();
        assert!(log.writer_acquire >= Duration::from_millis(5), "{log:?}");
        assert_eq!(log.slowest_acquire.unwrap().kind, HoldKind::Write);
    }

    #[tokio::test]
    async fn write_guard_records_static_command_and_action_id() {
        let lock = Arc::new(SupervisorLock::new(0u32));
        {
            let mut g = lock.write().await;
            g.set_command("/api/v1/motion/jog");
            g.set_action_id(77);
            *g += 1;
        }
        let log = lock.take_log().unwrap();
        let last = log.last_release.unwrap();
        assert_eq!(last.kind, HoldKind::Write);
        assert_eq!(last.command, Some("/api/v1/motion/jog"));
        assert_eq!(last.action_id, Some(77));
        {
            let mut g = lock.clone().write_owned().await;
            g.set_command("/api/v1/motion/home");
            g.set_action_id(88);
            *g += 1;
        }
        let log2 = lock.take_log().unwrap();
        let last2 = log2.longest.unwrap();
        assert_eq!(last2.command, Some("/api/v1/motion/home"));
        assert_eq!(last2.action_id, Some(88));
    }

    struct StubUnitDriver {
        token: u64,
    }

    impl crate::driver::ArmDriver for StubUnitDriver {
        fn read_joint_states(&mut self) -> Result<Vec<crate::model::JointState>, crate::driver::ArmDriverError> {
            Ok(vec![])
        }
        fn write_joint_targets(&mut self, _targets: &[crate::driver::JointTarget]) -> Result<(), crate::driver::ArmDriverError> {
            Ok(())
        }
        fn set_torque(&mut self, _enabled: bool) -> Result<(), crate::driver::ArmDriverError> {
            Ok(())
        }
        fn is_enabled(&self) -> bool {
            self.token == 0xDEAD_BEEF
        }
    }

    fn panic_str(p: Box<dyn std::any::Any + Send>) -> String {
        if let Some(s) = p.downcast_ref::<&str>() {
            (*s).to_string()
        } else if let Some(s) = p.downcast_ref::<String>() {
            s.clone()
        } else {
            "<non-string panic>".to_string()
        }
    }

    #[test]
    fn driver_lock_counts_and_panics_on_marked_thread_before_touching_mutex() {
        use crate::rt_core::executor::{ControlThreadGuard, ReleaseGuard};

        // (a) Contended DriverLock: another thread holds the mutex throughout. Both `lock()` and
        //     `try_lock()` must count +1 and panic BEFORE touching the underlying `std::sync::Mutex`.
        let counter = Arc::new(AtomicU64::new(0));
        let lock = Arc::new(DriverLock::with_counter(
            Box::new(StubUnitDriver { token: 1 }),
            Arc::clone(&counter),
        ));
        let (held_tx, held_rx) = std::sync::mpsc::sync_channel::<()>(1);
        let (rel_tx, rel_rx) = std::sync::mpsc::sync_channel::<()>(1);
        let holder_lock = Arc::clone(&lock);
        let holder = std::thread::spawn(move || {
            let _g = holder_lock.lock().unwrap();
            let _ = held_tx.send(());
            let _ = rel_rx.recv_timeout(Duration::from_secs(5));
        });
        held_rx.recv_timeout(Duration::from_secs(2)).expect("holder acquired mutex");

        let marked_lock = Arc::clone(&lock);
        std::thread::spawn(move || {
            let _g = ControlThreadGuard::enter();
            let err1 = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                drop(marked_lock.lock());
            }))
            .expect_err("DriverLock::lock must panic on marked thread");
            let msg1 = panic_str(err1);
            assert!(msg1.contains("RT control thread") && msg1.contains("DriverLock::lock"), "{msg1}");
            assert_eq!(marked_lock.control_thread_attempts(), 1);

            let err2 = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                drop(marked_lock.try_lock());
            }))
            .expect_err("DriverLock::try_lock must panic on marked thread");
            let msg2 = panic_str(err2);
            assert!(msg2.contains("RT control thread") && msg2.contains("DriverLock::try_lock"), "{msg2}");
            assert_eq!(marked_lock.control_thread_attempts(), 2);
        })
        .join()
        .unwrap();
        let _ = rel_tx.send(());
        holder.join().unwrap();
        assert_eq!(counter.load(Ordering::Acquire), 2);

        // (b) Uncontended DriverLock on marked thread: still counts +1 and panics before locking.
        let free = DriverLock::with_counter(Box::new(StubUnitDriver { token: 2 }), Arc::new(AtomicU64::new(0)));
        {
            let _g = ControlThreadGuard::enter();
            let e1 = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                drop(free.lock());
            }))
            .expect_err("uncontended lock must panic");
            assert!(panic_str(e1).contains("DriverLock::lock"));
            let e2 = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                drop(free.try_lock());
            }))
            .expect_err("uncontended try_lock must panic");
            assert!(panic_str(e2).contains("DriverLock::try_lock"));
        }
        assert_eq!(free.control_thread_attempts(), 2);

        // (c) ReleaseGuard alone on unmarked thread: panics with "during RT release" WITHOUT
        //     incrementing `control_thread_attempts`.
        let rel_only = DriverLock::new(Box::new(StubUnitDriver { token: 3 }));
        {
            let _r = ReleaseGuard::enter();
            let e1 = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                drop(rel_only.lock());
            }))
            .expect_err("lock in release must panic");
            assert!(panic_str(e1).contains("during RT release"));
            let e2 = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                drop(rel_only.try_lock());
            }))
            .expect_err("try_lock in release must panic");
            assert!(panic_str(e2).contains("during RT release"));
        }
        assert_eq!(rel_only.control_thread_attempts(), 0);
    }

    #[test]
    fn exclusive_driver_soundness_and_poisoned_into_exclusive_rollback() {
        // (a) ExclusiveDriver panics if `&self` is used from a non-owning thread, and succeeds
        //     after `bind_current_thread()` / `&mut` ownership transfer.
        let mut slot = DriverSlot::exclusive(Box::new(StubUnitDriver { token: 0xDEAD_BEEF }));
        assert!(slot.with_ref(|d| d.is_enabled()).unwrap());
        std::thread::scope(|s| {
            let slot_ref = &slot;
            let res = s
                .spawn(move || {
                    std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        let _ = slot_ref.with_ref(|d| d.is_enabled());
                    }))
                })
                .join()
                .unwrap();
            let msg = panic_str(res.expect_err("cross-thread &ExclusiveDriver must panic"));
            assert!(msg.contains("non-owning thread"), "{msg}");
        });
        std::thread::spawn(move || {
            slot.bind_current_thread();
            assert!(slot.with_ref(|d| d.is_enabled()).unwrap());
            assert!(slot.with_mut(|d| d.is_enabled()).unwrap());
        })
        .join()
        .unwrap();

        // (b) Poisoned Legacy DriverSlot::into_exclusive() fails closed and returns the exact
        //     intact Legacy slot with poison state, shared counter, and original driver preserved.
        let counter = Arc::new(AtomicU64::new(7));
        let poisoned_mutex = {
            let m = Arc::new(std::sync::Mutex::new(
                Box::new(StubUnitDriver { token: 0xDEAD_BEEF }) as Box<dyn crate::driver::ArmDriver>,
            ));
            let m2 = Arc::clone(&m);
            let _ = std::thread::spawn(move || {
                let _g = m2.lock().unwrap();
                panic!("poison driver mutex");
            })
            .join();
            Arc::try_unwrap(m).ok().unwrap()
        };
        let slot = DriverSlot::from_mutex_with_counter(poisoned_mutex, Arc::clone(&counter));
        let (returned, reason) = slot
            .into_exclusive()
            .err()
            .expect("poisoned driver must fail into_exclusive");
        assert_eq!(reason, "driver mutex poisoned");
        assert!(returned.is_legacy());
        let lock = returned.as_legacy_lock().expect("still legacy");
        assert!(lock.is_poisoned(), "poison state must be preserved");
        assert_eq!(lock.control_thread_attempts(), 7, "shared counter must be preserved");
        assert!(
            returned.with_ref_recover(|d| d.is_enabled()),
            "exact original driver instance must be preserved"
        );
    }
}
