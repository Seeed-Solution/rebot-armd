//! Host load test: control tick latency while observation readers compete for the CPU.
//!
//! Emulates the single-core reCamera: every thread of this test is pinned to one CPU, the
//! control thread runs at nice 0 (it is SCHED_FIFO on the device), the tokio workers that
//! serve observation readers and two busy threads (the vision process) run at nice 10, so a
//! reader can be preempted while it holds the supervisor lock. Prints
//! `GET /api/v1/diagnostics/tick` as JSON. Timing dependent, so ignored by default:
//!
//! `cargo test --test test_tick_load -- --ignored --nocapture`

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use armd::{ArmConfig, ArmSupervisor};

#[cfg(target_os = "linux")]
fn pin_to_cpu0() {
    // SAFETY: cpu_set_t is plain old data; the calls only touch the calling thread.
    unsafe {
        let mut set: libc::cpu_set_t = std::mem::zeroed();
        libc::CPU_SET(0, &mut set);
        assert_eq!(libc::sched_setaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &set), 0);
    }
}

#[cfg(target_os = "linux")]
fn nice_this_thread(nice: i32) {
    // SAFETY: plain syscalls on the calling thread; raising nice needs no privilege.
    unsafe {
        let tid = libc::syscall(libc::SYS_gettid) as libc::id_t;
        libc::setpriority(libc::PRIO_PROCESS, tid, nice);
    }
}

#[cfg(target_os = "linux")]
#[test]
#[ignore = "timing dependent load test; run with --ignored --nocapture"]
fn tick_latency_under_observation_readers_and_cpu_hogs() {
    run_load(false);
}

/// As above, plus a writer at 30 Hz (a WS servo stream's lock use): every write makes the
/// published snapshot older than the state, which readers must not answer with the lock.
#[cfg(target_os = "linux")]
#[test]
#[ignore = "timing dependent load test; run with --ignored --nocapture"]
fn tick_latency_under_observation_readers_writer_and_cpu_hogs() {
    run_load(true);
}

#[cfg(target_os = "linux")]
fn run_load(with_writer: bool) {
    const SECONDS: u64 = 20;
    const READERS: usize = 8;
    const READER_PERIOD: Duration = Duration::from_millis(1);
    const HOGS: usize = 3;
    pin_to_cpu0();
    let stop = Arc::new(AtomicBool::new(false));

    let hogs: Vec<_> = (0..HOGS)
        .map(|_| {
            let stop = stop.clone();
            std::thread::spawn(move || {
                nice_this_thread(10);
                let mut x = 0u64;
                while !stop.load(Ordering::Relaxed) {
                    x = x.wrapping_mul(6364136223846793005).wrapping_add(1);
                    std::hint::black_box(x);
                }
            })
        })
        .collect();

    let rt = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .enable_all()
        .on_thread_start(|| nice_this_thread(10))
        .build()
        .unwrap();
    let supervisor = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));

    let mut readers = Vec::new();
    for _ in 0..READERS {
        let sup = supervisor.clone();
        let stop = stop.clone();
        readers.push(rt.spawn(async move {
            let mut n = 0u64;
            while !stop.load(Ordering::Relaxed) {
                let obs = sup.get_observation().await;
                std::hint::black_box(serde_json::to_vec(&obs).unwrap());
                n += 1;
                tokio::time::sleep(READER_PERIOD).await;
            }
            n
        }));
    }

    if with_writer {
        let sup = supervisor.clone();
        let stop = stop.clone();
        rt.spawn(async move {
            let mut t = 30.0f32;
            while !stop.load(Ordering::Relaxed) {
                t = if t > 40.0 { 30.0 } else { t + 0.1 };
                sup.set_temperatures(vec![t; 7]).await;
                tokio::time::sleep(Duration::from_millis(33)).await;
            }
        });
    }

    let control_sup = supervisor.clone();
    let control_stop = stop.clone();
    let handle = rt.handle().clone();
    let control = std::thread::spawn(move || {
        let period = Duration::from_millis(20);
        let mut next = Instant::now() + period;
        let mut last = Instant::now();
        let mut walls = Vec::new();
        while !control_stop.load(Ordering::Relaxed) {
            let now = Instant::now();
            if next > now { std::thread::sleep(next - now); }
            next += period;
            let now = Instant::now();
            if next < now { next = now + period; }
            let dt = last.elapsed().as_secs_f64();
            last = now;
            let t = Instant::now();
            handle.block_on(control_sup.control_tick(dt));
            walls.push(t.elapsed().as_micros() as u64);
        }
        walls
    });

    std::thread::sleep(Duration::from_secs(SECONDS));
    let snapshot = serde_json::to_value(supervisor.tick_stats()).unwrap();
    stop.store(true, Ordering::Relaxed);
    let mut walls = control.join().unwrap();
    let reads: u64 = readers.into_iter().map(|r| rt.block_on(r).unwrap()).sum();
    for h in hogs { h.join().unwrap(); }
    walls.sort_unstable();
    let p = |q: usize| walls[(walls.len() * q).div_ceil(100).max(1) - 1];
    println!("writer {with_writer}: ticks {} reads {reads}; tick wall p50 {} p99 {} max {} us", walls.len(), p(50), p(99), walls[walls.len() - 1]);
    println!("TICK_STATS {snapshot}");
}
