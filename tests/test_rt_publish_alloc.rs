//! The tick's publication path (state sample and recording sample) neither allocates nor
//! frees: a counting allocator records every allocation and deallocation made by the
//! calling thread, inside armd and inside std / crate internals.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::time::{Duration, Instant};

use armd::rt_core::ArmCore;

thread_local! {
    static WATCH: Cell<bool> = const { Cell::new(false) };
    static ALLOCS: Cell<u64> = const { Cell::new(0) };
    static FREES: Cell<u64> = const { Cell::new(0) };
}

struct Counting;

fn count(c: &'static std::thread::LocalKey<Cell<u64>>) {
    if WATCH.try_with(Cell::get).unwrap_or(false) { let _ = c.try_with(|n| n.set(n.get() + 1)); }
}

// SAFETY: forwards to System; the counters are const-initialized thread-locals without
// destructors and do not allocate.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 { count(&ALLOCS); unsafe { System.alloc(l) } }
    unsafe fn alloc_zeroed(&self, l: Layout) -> *mut u8 { count(&ALLOCS); unsafe { System.alloc_zeroed(l) } }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) { count(&FREES); unsafe { System.dealloc(p, l) } }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 { count(&ALLOCS); count(&FREES); unsafe { System.realloc(p, l, n) } }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

/// (allocations, deallocations) of this thread inside `f`.
fn counted<R>(f: impl FnOnce() -> R) -> (R, u64, u64) {
    ALLOCS.with(|c| c.set(0));
    FREES.with(|c| c.set(0));
    WATCH.with(|w| w.set(true));
    let r = f();
    WATCH.with(|w| w.set(false));
    (r, ALLOCS.with(Cell::get), FREES.with(Cell::get))
}

#[test]
fn publish_and_record_push_do_not_touch_the_allocator() {
    let cfg = armd::ArmConfig::rebot_dm();
    let dof = cfg.dof;
    let mut core = ArmCore::new(dof);
    let hub = core.hub().clone();
    // a registered dispatcher thread: publish wakes it with unpark
    let stop = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    let dispatcher = {
        let stop = stop.clone();
        std::thread::spawn(move || while !stop.load(std::sync::atomic::Ordering::Relaxed) { std::thread::park_timeout(Duration::from_millis(5)); })
    };
    hub.set_dispatcher(dispatcher.thread().clone());
    let template = armd::SupervisorInner::new(cfg).current_observation();
    let q = vec![0.1f32; dof];
    core.begin_recording();
    for i in 0..1000u64 {
        // built outside the counted region (the observation itself is the tick's existing cost)
        let mut obs = template.clone();
        obs.timestamp = i as f64;
        let now = Instant::now();
        // the whole tick-side publication, including dropping what publish hands back: the
        // readers lag (one read every 7 ticks), so most publications displace an unread
        // observation, which must go to the retire ring and be freed by the reader
        let ((none_back, pushed), a, f) = counted(|| {
            let back = core.publish(i, now, now + Duration::from_millis(40), obs, &q, &q);
            let none_back = back.is_none();
            drop(back);
            (none_back, core.record_sample(i, i as f64, &q, &q))
        });
        assert_eq!((a, f), (0, 0), "publication {i}: {a} allocations, {f} deallocations");
        assert!(none_back, "publication {i}: the tick got an observation back to free");
        assert!(pushed, "ring full");
        if i % 7 == 0 {
            // a reader takes samples in between (its own allocations are not the tick's)
            let s = hub.latest().unwrap();
            assert_eq!(s.obs.timestamp, i as f64);
            assert_eq!(s.meta.seq, i + 1);
        }
        if i % 100 == 0 { core.record_ring().unwrap().drain(); }
    }
    stop.store(true, std::sync::atomic::Ordering::Relaxed);
    dispatcher.thread().unpark();
    dispatcher.join().unwrap();
    assert_eq!(hub.tick_frees(), 0);
    let done = core.finish_recording();
    assert_eq!(done.frames.len() as u64 + done.dropped, 1000);
}
