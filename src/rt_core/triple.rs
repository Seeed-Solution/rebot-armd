//! Triple buffer: one producer and one consumer exchange ownership of three slots.
//!
//! The producer owns the back slot, the consumer the front slot, and the third slot sits in
//! the shared middle word together with a FRESH bit. Publishing swaps the back slot into the
//! middle; consuming swaps the front slot with the middle when it is fresh. Neither side
//! ever waits for the other: no lock, no retry loop, one atomic swap each. The three indices
//! always form a permutation of 0..3, so a slot is touched by one side at a time.
//!
//! Ordering: both swaps are `AcqRel`. The producer's writes to a slot happen before the swap
//! that publishes it, and the consumer's swap that takes it acquires them; the same holds in
//! the other direction for a slot the consumer hands back (it may have taken a value out).

use std::cell::{Cell, UnsafeCell};
use std::marker::PhantomData;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

const INDEX: usize = 0b011;
const FRESH: usize = 0b100;

struct Shared<T> {
    slots: [UnsafeCell<T>; 3],
    middle: AtomicUsize,
}

// SAFETY: a slot is accessed only by the side that owns its index (see the module docs);
// ownership moves with the atomic swaps, which order the accesses.
unsafe impl<T: Send> Sync for Shared<T> {}

/// Writing side. Not `Clone`: exactly one producer.
pub struct Producer<T> {
    shared: Arc<Shared<T>>,
    back: usize,
}

/// Reading side. Not `Clone`: exactly one consumer (wrap it in a mutex for several readers).
/// Not `Sync`: [`front`](Self::front) hands out `&T` without requiring `T: Sync`, so a
/// shared `&Consumer` must not cross threads (it is `Send` and can move to one).
pub struct Consumer<T> {
    shared: Arc<Shared<T>>,
    front: usize,
    _not_sync: PhantomData<Cell<()>>,
}

/// A triple buffer whose slots start as `a`, `b`, `c` (back, middle, front).
pub fn triple_buffer<T: Send>(a: T, b: T, c: T) -> (Producer<T>, Consumer<T>) {
    let shared = Arc::new(Shared { slots: [UnsafeCell::new(a), UnsafeCell::new(b), UnsafeCell::new(c)], middle: AtomicUsize::new(1) });
    (Producer { shared: shared.clone(), back: 0 }, Consumer { shared, front: 2, _not_sync: PhantomData })
}

impl<T> Producer<T> {
    /// The slot the next [`publish`](Self::publish) hands over.
    pub fn back_mut(&mut self) -> &mut T {
        // SAFETY: the back index is owned by this producer until it publishes.
        unsafe { &mut *self.shared.slots[self.back].get() }
    }

    /// Hands the back slot to the consumer and takes the middle slot as the new back slot
    /// (an older, unconsumed sample or the slot the consumer released). Never blocks.
    /// True when the previous publication was still unconsumed and is now overwritten: the
    /// same atomic swap decides it, so the answer is exact even while the consumer runs.
    pub fn publish(&mut self) -> bool {
        let prev = self.shared.middle.swap(self.back | FRESH, Ordering::AcqRel);
        self.back = prev & INDEX;
        prev & FRESH != 0
    }

    /// A publication is waiting for the consumer (it may take it right after this returns).
    pub fn unconsumed(&self) -> bool { self.shared.middle.load(Ordering::Acquire) & FRESH != 0 }

    /// Swaps the back slot with the middle slot WITHOUT setting `FRESH` when no publication is
    /// pending (startup prewarming under reader contention: `Consumer::refresh` sees `FRESH == 0`
    /// and never accesses `middle`). Returns `true` if the swap succeeded.
    pub fn swap_unfresh(&mut self) -> bool {
        let cur = self.shared.middle.load(Ordering::Acquire);
        if cur & FRESH != 0 {
            return false;
        }
        // Only the producer ever sets `FRESH`; since `FRESH == 0`, the consumer cannot swap
        // `middle`, so CAS from `cur` to `self.back` is wait-free and unique.
        if self.shared.middle.compare_exchange(cur, self.back, Ordering::AcqRel, Ordering::Acquire).is_ok() {
            self.back = cur & INDEX;
            true
        } else {
            false
        }
    }

    /// Mutably visits all 3 slots when the caller holds exclusive access to both the producer
    /// and its paired consumer (startup prewarming only).
    pub fn for_each_slot_mut(&mut self, consumer: &mut Consumer<T>, mut f: impl FnMut(&mut T)) {
        assert!(Arc::ptr_eq(&self.shared, &consumer.shared), "mismatched triple-buffer endpoints");
        for slot in &self.shared.slots {
            // SAFETY: caller holds `&mut Producer` and `&mut Consumer` for the same `Shared`,
            // and neither type is `Clone` or `Sync`, so no other reference to any slot exists.
            f(unsafe { &mut *slot.get() });
        }
    }
}

impl<T> Consumer<T> {
    /// True when the producer has published a newer slot not yet swapped into `front`.
    /// Uses `Acquire` ordering so any startup state published before `Producer::publish`
    /// (such as `StateHub::prewarm_template`) is visible before the consumer rotates `front`.
    #[inline]
    pub fn has_fresh(&self) -> bool {
        self.shared.middle.load(Ordering::Acquire) & FRESH != 0
    }

    /// Takes the newest published slot when there is one; false when nothing was published
    /// since the last refresh (the front slot is unchanged). Never blocks.
    pub fn refresh(&mut self) -> bool {
        if self.shared.middle.load(Ordering::Relaxed) & FRESH == 0 {
            return false;
        }
        let prev = self.shared.middle.swap(self.front, Ordering::AcqRel);
        self.front = prev & INDEX;
        true
    }

    pub fn front(&self) -> &T {
        // SAFETY: the front index is owned by this consumer until it refreshes.
        unsafe { &*self.shared.slots[self.front].get() }
    }

    pub fn front_mut(&mut self) -> &mut T {
        // SAFETY: as in `front`.
        unsafe { &mut *self.shared.slots[self.front].get() }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Compile-time: `Consumer<T>` is never `Sync` (even for `T: Sync`), but it is `Send`.
    // `<C as NotSync<_>>::CHECK` is ambiguous, and fails to compile, when `C: Sync`.
    trait NotSync<A> { const CHECK: () = (); }
    impl<T: ?Sized> NotSync<()> for T {}
    #[allow(dead_code)]
    struct IsSync;
    impl<T: ?Sized + Sync> NotSync<IsSync> for T {}
    const _: () = <Consumer<std::cell::Cell<u32>> as NotSync<_>>::CHECK;
    const _: () = <Consumer<u32> as NotSync<_>>::CHECK;
    fn _consumer_is_send<T: Send>() { fn send<S: Send>() {} send::<Consumer<T>>(); }

    #[test]
    fn newest_wins_and_nothing_is_read_twice() {
        let (mut p, mut c) = triple_buffer(0u64, 0, 0);
        assert!(!c.refresh());
        *p.back_mut() = 1;
        p.publish();
        *p.back_mut() = 2;
        p.publish();
        assert!(c.refresh());
        assert_eq!(*c.front(), 2);
        assert!(!c.refresh());
        assert_eq!(*c.front(), 2);
        // wraparound of the slot rotation
        for i in 3..1000 {
            *p.back_mut() = i;
            p.publish();
            if i % 3 == 0 {
                assert!(c.refresh());
                assert_eq!(*c.front(), i);
            }
        }
    }

    #[test]
    fn indices_stay_a_permutation() {
        let (mut p, mut c) = triple_buffer((), (), ());
        for i in 0..64u32 {
            if i % 2 == 0 { p.publish(); }
            if i % 3 == 0 { c.refresh(); }
            let m = p.shared.middle.load(Ordering::Relaxed) & INDEX;
            let mut seen = [p.back, m, c.front];
            seen.sort_unstable();
            assert_eq!(seen, [0, 1, 2], "step {i}");
        }
    }

    /// A stalled consumer never blocks the producer, and a reader sees only whole samples
    /// in publication order.
    #[test]
    fn concurrent_producer_and_stalling_consumer() {
        const N: u64 = 200_000;
        let (mut p, mut c) = triple_buffer([0u64; 8], [0; 8], [0; 8]);
        let producer = std::thread::spawn(move || {
            for i in 1..=N {
                *p.back_mut() = [i; 8];
                p.publish();
            }
        });
        let mut last = 0;
        let mut stalls = 0u32;
        while last < N {
            if c.refresh() {
                let s = *c.front();
                assert!(s.iter().all(|v| *v == s[0]), "torn sample {s:?}");
                assert!(s[0] > last, "{} after {last}", s[0]);
                last = s[0];
                if stalls < 50 && last % 1000 == 0 {
                    stalls += 1;
                    std::thread::sleep(std::time::Duration::from_micros(200));
                }
            }
        }
        producer.join().unwrap();
    }
}
