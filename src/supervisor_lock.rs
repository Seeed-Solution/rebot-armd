//! The supervisor's `RwLock`, with two additions for the control tick:
//!
//! - a write generation, bumped each time a write guard is taken, so a reader can tell
//!   whether the observation the tick published is still the current state;
//! - the longest read and write hold by other tasks since the tick last looked, so a tick
//!   that waited for the lock can name what held it. A hold includes any time the holding
//!   task was preempted, which is what a real-time tick waiting on a normal-priority
//!   reader sees. The hold is measured after the release, so it also covers a holder that
//!   was preempted while releasing;
//! - the longest time other tasks spent in `read().await` / `write().await`. tokio hands the
//!   lock to a queued task when the previous holder releases it, before that task runs: a
//!   task that is runnable but not scheduled (a starved tokio worker) holds the lock from
//!   then on without a guard existing. Its hold looks short, its acquire time is long.
//!
//! Guards deref to the tokio guards' target, so call sites stay `lock.read().await` /
//! `lock.write().await`. Other `tokio::sync::RwLock` methods (`try_read`, ...) are reached
//! through `Deref` and are neither timed nor counted; every way to take a write guard is
//! implemented here so the generation cannot be bypassed.

use std::mem::ManuallyDrop;
use std::ops::{Deref, DerefMut};
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use tokio::sync::{OwnedRwLockWriteGuard, RwLock, RwLockReadGuard, RwLockWriteGuard, TryLockError};

#[derive(Default)]
struct Holds {
    generation: AtomicU64,
    reader_max_us: AtomicU32,
    writer_max_us: AtomicU32,
    reader_acquire_max_us: AtomicU32,
    writer_acquire_max_us: AtomicU32,
}

/// Longest other holds and acquires since the previous [`SupervisorLock::take_maxima`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LockMaxima {
    pub reader_hold: Duration,
    pub writer_hold: Duration,
    pub reader_acquire: Duration,
    pub writer_acquire: Duration,
}

fn us(d: Duration) -> u32 { d.as_micros().min(u128::from(u32::MAX)) as u32 }

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

    fn bump(&self) -> u64 { self.holds.generation.fetch_add(1, Ordering::AcqRel) + 1 }

    pub async fn read(&self) -> SupReadGuard<'_, T> {
        let called = Instant::now();
        let guard = self.lock.read().await;
        self.holds.reader_acquire_max_us.fetch_max(us(called.elapsed()), Ordering::AcqRel);
        SupReadGuard { guard: ManuallyDrop::new(guard), since: Instant::now(), holds: &self.holds }
    }

    pub async fn write(&self) -> SupWriteGuard<'_, T> {
        let called = Instant::now();
        let guard = self.lock.write().await;
        self.bump();
        self.holds.writer_acquire_max_us.fetch_max(us(called.elapsed()), Ordering::AcqRel);
        SupWriteGuard { guard: ManuallyDrop::new(guard), since: Instant::now(), holds: &self.holds }
    }

    pub fn try_write(&self) -> Result<SupWriteGuard<'_, T>, TryLockError> {
        let guard = self.lock.try_write()?;
        self.bump();
        Ok(SupWriteGuard { guard: ManuallyDrop::new(guard), since: Instant::now(), holds: &self.holds })
    }

    pub fn blocking_write(&self) -> SupWriteGuard<'_, T> {
        let called = Instant::now();
        let guard = self.lock.blocking_write();
        self.bump();
        self.holds.writer_acquire_max_us.fetch_max(us(called.elapsed()), Ordering::AcqRel);
        SupWriteGuard { guard: ManuallyDrop::new(guard), since: Instant::now(), holds: &self.holds }
    }

    pub async fn write_owned(self: Arc<Self>) -> SupOwnedWriteGuard<T> {
        let called = Instant::now();
        let guard = self.lock.clone().write_owned().await;
        self.bump();
        self.holds.writer_acquire_max_us.fetch_max(us(called.elapsed()), Ordering::AcqRel);
        SupOwnedWriteGuard { guard: ManuallyDrop::new(guard), since: Instant::now(), holds: self.holds.clone() }
    }

    /// The control tick's write guard: counted in the generation (returned) but not timed,
    /// the tick times itself.
    pub async fn write_tick(&self) -> (RwLockWriteGuard<'_, T>, u64) {
        let guard = self.lock.write().await;
        let generation = self.bump();
        (guard, generation)
    }

    /// Longest other holds (ended) and acquires (completed) since the previous call; resets them.
    pub fn take_maxima(&self) -> LockMaxima {
        let take = |a: &AtomicU32| Duration::from_micros(a.swap(0, Ordering::AcqRel).into());
        LockMaxima {
            reader_hold: take(&self.holds.reader_max_us),
            writer_hold: take(&self.holds.writer_max_us),
            reader_acquire: take(&self.holds.reader_acquire_max_us),
            writer_acquire: take(&self.holds.writer_acquire_max_us),
        }
    }
}

impl<T> Deref for SupervisorLock<T> {
    type Target = RwLock<T>;
    fn deref(&self) -> &RwLock<T> { &self.lock }
}

pub struct SupReadGuard<'a, T> {
    guard: ManuallyDrop<RwLockReadGuard<'a, T>>,
    since: Instant,
    holds: &'a Holds,
}

impl<T> Drop for SupReadGuard<'_, T> {
    fn drop(&mut self) {
        // SAFETY: dropped once, here; the field is not used afterwards.
        unsafe { ManuallyDrop::drop(&mut self.guard) };
        self.holds.reader_max_us.fetch_max(us(self.since.elapsed()), Ordering::AcqRel);
    }
}

impl<T> Deref for SupReadGuard<'_, T> {
    type Target = T;
    fn deref(&self) -> &T { &self.guard }
}

pub struct SupWriteGuard<'a, T> {
    guard: ManuallyDrop<RwLockWriteGuard<'a, T>>,
    since: Instant,
    holds: &'a Holds,
}

impl<T> Drop for SupWriteGuard<'_, T> {
    fn drop(&mut self) {
        // SAFETY: dropped once, here; the field is not used afterwards.
        unsafe { ManuallyDrop::drop(&mut self.guard) };
        self.holds.writer_max_us.fetch_max(us(self.since.elapsed()), Ordering::AcqRel);
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
    since: Instant,
    holds: Arc<Holds>,
}

impl<T> Drop for SupOwnedWriteGuard<T> {
    fn drop(&mut self) {
        // SAFETY: dropped once, here; the field is not used afterwards.
        unsafe { ManuallyDrop::drop(&mut self.guard) };
        self.holds.writer_max_us.fetch_max(us(self.since.elapsed()), Ordering::AcqRel);
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
    async fn generation_counts_write_guards_and_holds_are_timed() {
        let lock = Arc::new(SupervisorLock::new(0u32));
        assert_eq!(lock.generation(), 0);
        { let _r = lock.read().await; }
        assert_eq!(lock.generation(), 0);
        { *lock.write().await += 1; }
        { *lock.try_write().unwrap() += 1; }
        { *lock.clone().write_owned().await += 1; }
        let (_g, generation) = lock.write_tick().await;
        assert_eq!(generation, 4);
        drop(_g);
        assert_eq!(lock.generation(), 4);
        assert_eq!(*lock.read().await, 3);
        lock.take_maxima();
        {
            let _r = lock.read().await;
            std::thread::sleep(Duration::from_millis(5));
        }
        let m = lock.take_maxima();
        assert!(m.reader_hold >= Duration::from_millis(5), "{m:?}");
        assert!(m.reader_acquire < Duration::from_millis(5), "{m:?}");
        assert_eq!(m.writer_hold, Duration::ZERO);
        assert_eq!(lock.take_maxima(), LockMaxima::default());
        // a writer queued behind a reader: its acquire time covers the reader's hold
        let r = lock.read().await;
        let w = tokio::spawn({ let lock = lock.clone(); async move { *lock.write().await += 1; } });
        tokio::time::sleep(Duration::from_millis(5)).await;
        drop(r);
        w.await.unwrap();
        let m = lock.take_maxima();
        assert!(m.writer_acquire >= Duration::from_millis(5), "{m:?}");
    }
}
