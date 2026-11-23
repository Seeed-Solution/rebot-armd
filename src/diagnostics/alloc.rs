//! Allocator time of the control thread (`ARMD_ALLOC_TIMING=1`).
//!
//! The device build links musl, whose malloc takes one process-wide lock. A normal-priority
//! thread preempted inside malloc or free makes the SCHED_FIFO control thread wait in its
//! next allocation. [`TimedAlloc`] is the system allocator that, on threads that opted in with
//! [`time_this_thread`], times every call; the tick reads the totals with [`take`].
//! The armd binary installs it as the global allocator; the clock reads cost two
//! `clock_gettime` per call, so the control thread opts in only when `ARMD_ALLOC_TIMING=1`.
//! Without the allocator or the opt-in every total stays zero.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::time::Instant;

thread_local! {
    static TIMED: Cell<bool> = const { Cell::new(false) };
    static CALLS: Cell<u64> = const { Cell::new(0) };
    static TOTAL_NS: Cell<u64> = const { Cell::new(0) };
    static MAX_NS: Cell<u64> = const { Cell::new(0) };
}

/// Allocator calls of this thread since the previous [`take`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct AllocTotals {
    pub calls: u64,
    pub total_ns: u64,
    pub max_ns: u64,
}

/// Times this thread's allocator calls from now on (when [`TimedAlloc`] is installed).
pub fn time_this_thread() { TIMED.with(|t| t.set(true)); }

/// Totals since the previous call, reset.
pub fn take() -> AllocTotals {
    AllocTotals {
        calls: CALLS.with(|c| c.replace(0)),
        total_ns: TOTAL_NS.with(|c| c.replace(0)),
        max_ns: MAX_NS.with(|c| c.replace(0)),
    }
}

#[inline]
fn timed<R>(f: impl FnOnce() -> R) -> R {
    // try_with: a thread that is being torn down has no thread-locals left
    if !TIMED.try_with(Cell::get).unwrap_or(false) { return f(); }
    let t = Instant::now();
    let r = f();
    let ns = t.elapsed().as_nanos().min(u128::from(u64::MAX)) as u64;
    let _ = CALLS.try_with(|c| c.set(c.get() + 1));
    let _ = TOTAL_NS.try_with(|c| c.set(c.get().saturating_add(ns)));
    let _ = MAX_NS.try_with(|c| c.set(c.get().max(ns)));
    r
}

/// The system allocator, timed on opted-in threads.
pub struct TimedAlloc;

// SAFETY: every call forwards to `System` with the same arguments; the timing touches only
// const-initialized thread-locals without destructors, which do not allocate.
unsafe impl GlobalAlloc for TimedAlloc {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 { timed(|| unsafe { System.alloc(layout) }) }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 { timed(|| unsafe { System.alloc_zeroed(layout) }) }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) { timed(|| unsafe { System.dealloc(ptr, layout) }) }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        timed(|| unsafe { System.realloc(ptr, layout, new_size) })
    }
}
