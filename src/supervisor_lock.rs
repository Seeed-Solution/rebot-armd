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
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::task::{Context, Poll};
use std::time::{Duration, Instant};

use tokio::sync::{OwnedRwLockWriteGuard, RwLock, RwLockReadGuard, RwLockWriteGuard, TryLockError};

use crate::diagnostics::tick::HoldIoTiming;

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
    /// Optional static direction tag (e.g. `"on"` / `"off"` for `/api/v1/torque`).
    pub direction: Option<&'static str>,
    /// Action id created or installed under this hold, if any.
    pub action_id: Option<u64>,
    /// Optional TX / RX / sleep timing (µs) recorded under this hold.
    pub io_timing: Option<HoldIoTiming>,
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
    /// Optional per-hold recording for tests asserting every individual guard hold (disabled in
    /// production: a single Relaxed atomic load avoids taking the history mutex or allocating).
    history_enabled: AtomicBool,
    history: std::sync::Mutex<Vec<HoldRecord>>,
}

impl Holds {
    #[allow(clippy::too_many_arguments)]
    fn record(
        &self,
        kind: HoldKind,
        location: &'static Location<'static>,
        command: Option<&'static str>,
        direction: Option<&'static str>,
        action_id: Option<u64>,
        io_timing: Option<HoldIoTiming>,
        called: Instant,
        acquired: Instant,
    ) {
        let r = HoldRecord {
            kind,
            location,
            command,
            direction,
            action_id,
            io_timing,
            called,
            acquired,
            released: Instant::now(),
        };
        self.log.lock().unwrap_or_else(|e| e.into_inner()).add(r);
        if self.history_enabled.load(Ordering::Relaxed) {
            self.history.lock().unwrap_or_else(|e| e.into_inner()).push(r);
        }
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

    /// Enable or disable per-hold history recording (for lock-proof tests asserting every guard
    /// hold, not only window maxima). Clears any buffered history when toggled.
    #[doc(hidden)]
    pub fn set_hold_history_enabled(&self, enabled: bool) {
        self.holds.history_enabled.store(enabled, Ordering::Release);
        let mut h = self.holds.history.lock().unwrap_or_else(|e| e.into_inner());
        h.clear();
        if enabled && h.capacity() < 256 {
            let need = 256 - h.capacity();
            h.reserve(need);
        }
    }

    /// Record a control-tick write-guard hold into opt-in test history ONLY (never touches
    /// `HoldLog` / `waited_for`). One Relaxed atomic load when disabled.
    #[doc(hidden)]
    #[allow(clippy::too_many_arguments)]
    pub fn record_tick_hold_for_history(
        &self,
        location: &'static Location<'static>,
        command: Option<&'static str>,
        direction: Option<&'static str>,
        io_timing: Option<HoldIoTiming>,
        called: Instant,
        acquired: Instant,
        released: Instant,
    ) {
        if self.holds.history_enabled.load(Ordering::Relaxed) {
            self.holds
                .history
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .push(HoldRecord {
                    kind: HoldKind::Write,
                    location,
                    command,
                    direction,
                    action_id: None,
                    io_timing,
                    called,
                    acquired,
                    released,
                });
        }
    }

    /// Drain all individual [`HoldRecord`] entries recorded while hold history was enabled.
    #[doc(hidden)]
    pub fn take_hold_history(&self) -> Vec<HoldRecord> {
        std::mem::take(&mut *self.holds.history.lock().unwrap_or_else(|e| e.into_inner()))
    }
}

#[derive(Clone, Copy)]
struct Timing {
    location: &'static Location<'static>,
    command: Option<&'static str>,
    direction: Option<&'static str>,
    action_id: Option<u64>,
    io_timing: Option<HoldIoTiming>,
    called: Instant,
    acquired: Instant,
}

impl Timing {
    #[inline]
    fn new(location: &'static Location<'static>, called: Instant) -> Self {
        Self {
            location,
            command: None,
            direction: None,
            action_id: None,
            io_timing: None,
            called,
            acquired: Instant::now(),
        }
    }
}

pub struct SupReadGuard<'a, T> {
    guard: ManuallyDrop<RwLockReadGuard<'a, T>>,
    t: Timing,
    holds: &'a Holds,
}

impl<T> Drop for SupReadGuard<'_, T> {
    fn drop(&mut self) {
        // Release the underlying RwLock guard BEFORE sampling `released` and acquiring
        // hold-log / history mutexes so lock recording never extends the actual hold duration.
        // SAFETY: dropped once, here; the field is not used afterwards.
        unsafe { ManuallyDrop::drop(&mut self.guard) };
        self.holds.record(
            HoldKind::Read,
            self.t.location,
            self.t.command,
            self.t.direction,
            self.t.action_id,
            self.t.io_timing,
            self.t.called,
            self.t.acquired,
        );
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

    /// Attribute this write-lock hold to a static direction tag (e.g. `"on"` / `"off"`, zero allocation).
    #[inline]
    pub fn set_direction(&mut self, direction: &'static str) {
        self.t.direction = Some(direction);
    }

    /// Attribute this write-lock hold to the action id created or installed under it (zero allocation).
    #[inline]
    pub fn set_action_id(&mut self, action_id: u64) {
        self.t.action_id = Some(action_id);
    }

    /// Attach TX / RX / sleep timing (µs) measured while this write guard was held (zero allocation).
    #[inline]
    pub fn set_io_timing(&mut self, io: HoldIoTiming) {
        self.t.io_timing = Some(io);
    }
}

impl<T> Drop for SupWriteGuard<'_, T> {
    fn drop(&mut self) {
        // Release the underlying RwLock guard BEFORE sampling `released` and acquiring
        // hold-log / history mutexes so lock recording never extends the actual hold duration.
        // SAFETY: dropped once, here; the field is not used afterwards.
        unsafe { ManuallyDrop::drop(&mut self.guard) };
        self.holds.record(
            HoldKind::Write,
            self.t.location,
            self.t.command,
            self.t.direction,
            self.t.action_id,
            self.t.io_timing,
            self.t.called,
            self.t.acquired,
        );
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

    /// Attribute this write-lock hold to a static direction tag (e.g. `"on"` / `"off"`, zero allocation).
    #[inline]
    pub fn set_direction(&mut self, direction: &'static str) {
        self.t.direction = Some(direction);
    }

    /// Attribute this write-lock hold to the action id created or installed under it (zero allocation).
    #[inline]
    pub fn set_action_id(&mut self, action_id: u64) {
        self.t.action_id = Some(action_id);
    }

    /// Attach TX / RX / sleep timing (µs) measured while this write guard was held (zero allocation).
    #[inline]
    pub fn set_io_timing(&mut self, io: HoldIoTiming) {
        self.t.io_timing = Some(io);
    }
}

impl<T> Drop for SupOwnedWriteGuard<T> {
    fn drop(&mut self) {
        // Release the underlying RwLock guard BEFORE sampling `released` and acquiring
        // hold-log / history mutexes so lock recording never extends the actual hold duration.
        // SAFETY: dropped once, here; the field is not used afterwards.
        unsafe { ManuallyDrop::drop(&mut self.guard) };
        self.holds.record(
            HoldKind::Write,
            self.t.location,
            self.t.command,
            self.t.direction,
            self.t.action_id,
            self.t.io_timing,
            self.t.called,
            self.t.acquired,
        );
    }
}

impl<T> Deref for SupOwnedWriteGuard<T> {
    type Target = T;
    fn deref(&self) -> &T { &self.guard }
}

impl<T> DerefMut for SupOwnedWriteGuard<T> {
    fn deref_mut(&mut self) -> &mut T { &mut self.guard }
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

    #[tokio::test]
    async fn write_guard_records_direction_io_timing_and_hold_history() {
        let lock = Arc::new(SupervisorLock::new(0u32));
        lock.set_hold_history_enabled(true);
        {
            let mut g = lock.write().await;
            g.set_command("/api/v1/torque");
            g.set_direction("on");
            g.set_io_timing(HoldIoTiming { tx_us: 15, rx_us: 25, sleep_us: 350 });
            *g += 1;
        }
        {
            let mut g = lock.write().await;
            g.set_command("/api/v1/torque");
            g.set_direction("off");
            g.set_io_timing(HoldIoTiming { tx_us: 5, rx_us: 10, sleep_us: 0 });
            *g += 1;
        }
        let hist = lock.take_hold_history();
        assert_eq!(hist.len(), 2);
        assert_eq!(hist[0].command, Some("/api/v1/torque"));
        assert_eq!(hist[0].direction, Some("on"));
        assert_eq!(hist[0].io_timing, Some(HoldIoTiming { tx_us: 15, rx_us: 25, sleep_us: 350 }));
        assert_eq!(hist[1].direction, Some("off"));
        assert_eq!(hist[1].io_timing, Some(HoldIoTiming { tx_us: 5, rx_us: 10, sleep_us: 0 }));
        lock.set_hold_history_enabled(false);
    }
}
