//! `diagnostics::alloc::TimedAlloc` installed as this test binary's allocator.

use armd::diagnostics::alloc::{self, TimedAlloc};

#[global_allocator]
static GLOBAL: TimedAlloc = TimedAlloc;

#[test]
fn only_opted_in_threads_are_timed() {
    std::thread::spawn(|| {
        std::hint::black_box(vec![0u8; 1024]);
        assert_eq!(alloc::take(), alloc::AllocTotals::default());
        alloc::time_this_thread();
        let v: Vec<Vec<u8>> = (0..10).map(|i| vec![0u8; 64 * (i + 1)]).collect();
        drop(std::hint::black_box(v));
        let t = alloc::take();
        assert!(t.calls >= 22, "{t:?}"); // 11 allocations, 11 frees
        assert!(t.total_ns >= t.max_ns && t.max_ns > 0, "{t:?}");
        assert_eq!(alloc::take().calls, 0);
    }).join().unwrap();
}

#[test]
fn the_tick_reports_the_control_threads_allocator_time() {
    let rt = tokio::runtime::Builder::new_current_thread().enable_all().build().unwrap();
    let sup = armd::ArmSupervisor::new(armd::ArmConfig::rebot_dm());
    alloc::time_this_thread();
    for _ in 0..3 { rt.block_on(sup.control_tick(0.02)); }
    let m = sup.tick_stats().max_lock.unwrap().sample;
    assert!(m.usage.lock_alloc_calls > 0, "{m:?}");
    assert!(m.usage.lock_alloc_us <= m.lock_us, "{m:?}");
}
