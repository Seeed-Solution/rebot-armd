//! Stage 3 PR Q1: timing diagnostics in `GET /api/v1/diagnostics/tick` and `analyze.py` tests.
//!
//! Verifies:
//! 1. Exact raw-ns boundary classification (`> 1 ms`, `> 5 ms`, `> 10 ms`), `TickStats` fallback,
//!    and `WakeLatenessGuard` unwind safety.
//! 2. Real HTTP `GET /api/v1/diagnostics/tick` under a scripted clock with injected wake lateness
//!    and skips for BOTH owners:
//!    - Legacy owner (`run_periodic_with_clock` + `attach_legacy_loop_stats`)
//!    - Executor owner (`start_executor_with_hooks` with `alloc_timing = true`, proving 0 heap ops
//!      on `armd-control` and verifying `diag_dropped > 0` while cumulative counters stay exact).
//! 3. `scripts/rt_qual/analyze.py` consumes the real Q1 fields, omits closed gaps, handles
//!    `diag_dropped > 0` as `INCOMPLETE` (or `FAIL` on an observed breach), and preserves full
//!    backward compatibility with older JSON lacking Q1 fields.

use std::collections::VecDeque;
use std::process::Command;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Condvar, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use armd::api::{create_router_full, proxy::DetectorUpstream};
use armd::diagnostics::alloc::{AllocCounts, TimedAlloc};
use armd::diagnostics::tick::{
    classify_release_duration_ns, classify_wake_lateness_ns, take_current_wake_lateness_ns,
    TickSample, TickStats, WakeLatenessGuard, TICK_WINDOW,
};
use armd::diagnostics::DiagnosticsState;
use armd::rt_core::executor::{
    run_periodic_with_clock, ExecutorClock, ExecutorConfig, ExecutorStats, PersistTestHooks,
    DIAG_RING_CAPACITY,
};
use armd::rt_core::sched::{observe_current_thread, MemoryLock};
use armd::{ArmConfig, ArmSupervisor};

#[global_allocator]
static ALLOC: TimedAlloc = TimedAlloc;

// ============================================================================
// Scripted Clock with Deterministic Per-Release Wakeup Lateness Injection
// ============================================================================

struct LatenessClock {
    base: Instant,
    state: Mutex<LatenessClockState>,
    cv: Condvar,
}

struct LatenessClockState {
    now_ns: u64,
    remaining_releases: u64,
    completed_sleeps: u64,
    lateness_queue_ns: VecDeque<u64>,
    waiting_for_ns: Option<u64>,
}

impl LatenessClock {
    fn new() -> Arc<Self> {
        Arc::new(Self {
            base: Instant::now(),
            state: Mutex::new(LatenessClockState {
                now_ns: 0,
                remaining_releases: 0,
                completed_sleeps: 0,
                lateness_queue_ns: VecDeque::new(),
                waiting_for_ns: None,
            }),
            cv: Condvar::new(),
        })
    }

    /// Advances the scripted clock monotonically by `delta_ns` (simulating in-release work).
    fn advance_ns(&self, delta_ns: u64) {
        let mut s = self.state.lock().unwrap();
        s.now_ns = s.now_ns.saturating_add(delta_ns);
        self.cv.notify_all();
    }

    /// Runs `latenesses_ns.len()` releases in lockstep, waking each release at
    /// `scheduled_ns + lateness_ns` and waiting until the control loop parks for the next release.
    fn run_schedule(&self, latenesses_ns: &[u64]) {
        let mut s = self.state.lock().unwrap();
        let start_completed = s.completed_sleeps;
        let expected = start_completed + latenesses_ns.len() as u64;
        for &l in latenesses_ns {
            s.lateness_queue_ns.push_back(l);
        }
        s.remaining_releases += latenesses_ns.len() as u64;
        self.cv.notify_all();
        let deadline = Instant::now() + Duration::from_secs(5);
        while s.completed_sleeps < expected || s.waiting_for_ns.is_none() {
            let rem = deadline.saturating_duration_since(Instant::now());
            assert!(
                !rem.is_zero(),
                "timed out waiting for scripted lateness schedule (completed={}/{}, wait={:?})",
                s.completed_sleeps,
                expected,
                s.waiting_for_ns
            );
            let (next, _) = self.cv.wait_timeout(s, rem).unwrap();
            s = next;
        }
    }
}

impl ExecutorClock for LatenessClock {
    fn now(&self) -> Instant {
        self.base + Duration::from_nanos(self.state.lock().unwrap().now_ns)
    }

    fn now_ns(&self) -> u64 {
        self.state.lock().unwrap().now_ns
    }

    fn sleep_until_ns(&self, target_ns: u64, stop: &AtomicBool) -> bool {
        let mut s = self.state.lock().unwrap();
        if s.now_ns >= target_ns {
            self.cv.notify_all();
            return false;
        }
        while !stop.load(Ordering::Acquire) && s.remaining_releases == 0 {
            s.waiting_for_ns = Some(target_ns);
            self.cv.notify_all();
            s = self.cv.wait(s).unwrap();
        }
        s.waiting_for_ns = None;
        if stop.load(Ordering::Acquire) {
            self.cv.notify_all();
            return true;
        }
        s.remaining_releases -= 1;
        let lateness = s.lateness_queue_ns.pop_front().unwrap_or(0);
        s.now_ns = target_ns.max(s.now_ns).saturating_add(lateness);
        s.completed_sleeps += 1;
        self.cv.notify_all();
        true
    }

    fn sleep(&self, dur: Duration, stop: &AtomicBool) {
        if stop.load(Ordering::Acquire) {
            return;
        }
        let mut s = self.state.lock().unwrap();
        s.now_ns = s.now_ns.saturating_add(dur.as_nanos().min(u128::from(u64::MAX)) as u64);
        self.cv.notify_all();
    }

    fn wake(&self) {
        let _g = self.state.lock().unwrap();
        self.cv.notify_all();
    }
}

async fn serve(supervisor: Arc<ArmSupervisor>) -> std::net::SocketAddr {
    serve_with_auth(supervisor, armd::OperatorAuth::disabled()).await
}

async fn serve_with_auth(
    supervisor: Arc<ArmSupervisor>,
    auth: armd::OperatorAuth,
) -> std::net::SocketAddr {
    let grasp = armd::grasp::GraspDeps::http(DetectorUpstream::from_env(), None);
    let router = create_router_full(
        supervisor,
        DetectorUpstream::from_env(),
        grasp,
        DiagnosticsState::default(),
        auth,
    );
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap() });
    addr
}

fn pause_hub(sup: &Arc<ArmSupervisor>) -> (mpsc::SyncSender<()>, thread::JoinHandle<()>) {
    let (held_tx, held_rx) = mpsc::sync_channel::<()>(1);
    let (rel_tx, rel_rx) = mpsc::sync_channel::<()>(1);
    let hub = sup.hub().clone();
    let jh = thread::spawn(move || {
        let _guard = hub.reader_guard();
        let _ = held_tx.send(());
        let _ = rel_rx.recv();
    });
    held_rx
        .recv_timeout(Duration::from_secs(2))
        .expect("acquire hub reader_guard");
    (rel_tx, jh)
}

/// Acquires `hub.reader_guard()` and runs one clean primer release so `armd-dispatch` wakes,
/// finishes its pre-lock `drain_sinks()` call, and deterministically blocks on `hub.latest()`
/// before the caller's paused burst starts.
fn pause_dispatch_at_hub(
    sup: &Arc<ArmSupervisor>,
    clock: &LatenessClock,
) -> (mpsc::SyncSender<()>, thread::JoinHandle<()>) {
    let pair = pause_hub(sup);
    clock.run_schedule(&[0]);
    thread::sleep(Duration::from_millis(10));
    pair
}

/// Shared 10-release schedule on a 10 ms period exercising exact boundaries, sub-µs >1 ms
/// classification, and multi-period skips:
/// - Sorted `wake_lateness_us`: [200, 500, 1000, 1000, 2500, 5000, 6200, 10000, 12000, 23400]
///   -> p50 = 2500, p99 = 23400, max = 23400
/// - Strict raw-ns thresholds: over_1ms = 7, over_5ms = 4, over_10ms = 2
/// - Skipped releases on 10 ms period: 1 (10 ms) + 2 (23.4 ms) + 1 (12 ms) = 4
const SCRIPTED_PERIOD_NS: u64 = 10_000_000;
const SCRIPTED_LATENESS_NS: [u64; 10] = [
    200_000,    //  200 us | over: (0,0,0) | skip: 0
    1_000_000,  // 1000 us | over: (0,0,0) | skip: 0 (exact 1.000000 ms is NOT > 1 ms)
    1_000_500,  // 1000 us | over: (1,0,0) | skip: 0 (1.000500 ms IS > 1 ms despite 1000 us trunc)
    2_500_000,  // 2500 us | over: (1,0,0) | skip: 0
    5_000_000,  // 5000 us | over: (1,0,0) | skip: 0 (exact 5.000000 ms is NOT > 5 ms)
    6_200_000,  // 6200 us | over: (1,1,0) | skip: 0
    10_000_000, // 10000 us | over: (1,1,0) | skip: 1 (exact 10.000000 ms -> hits next grid point)
    23_400_000, // 23400 us | over: (1,1,1) | skip: 2
    500_000,    //  500 us | over: (0,0,0) | skip: 0
    12_000_000, // 12000 us | over: (1,1,1) | skip: 1
];

// ============================================================================
// 1. Classifier boundaries, TickStats fallback, and WakeLatenessGuard unwind
// ============================================================================

#[test]
fn classify_wake_lateness_ns_boundaries_and_guard_unwind_safety() {
    assert_eq!(classify_wake_lateness_ns(0), (0, false, false, false));
    assert_eq!(classify_wake_lateness_ns(999_999), (999, false, false, false));
    assert_eq!(classify_wake_lateness_ns(1_000_000), (1_000, false, false, false));
    assert_eq!(classify_wake_lateness_ns(1_000_001), (1_000, true, false, false));
    assert_eq!(classify_wake_lateness_ns(5_000_000), (5_000, true, false, false));
    assert_eq!(classify_wake_lateness_ns(5_000_001), (5_000, true, true, false));
    assert_eq!(classify_wake_lateness_ns(10_000_000), (10_000, true, true, false));
    assert_eq!(classify_wake_lateness_ns(10_000_001), (10_000, true, true, true));
    assert_eq!(classify_wake_lateness_ns(u64::MAX), (u32::MAX, true, true, true));

    // Single authority: ExecutorStats owns cumulative raw-ns threshold counts, while TickStats
    // computes window percentiles from TickSample::wake_lateness_us.
    let exec_stats = ExecutorStats::default();
    let mut stats = TickStats::default();
    for &ns in &SCRIPTED_LATENESS_NS {
        let us = exec_stats.record_wake_lateness_ns(ns);
        stats.record(&TickSample {
            tick_us: 500,
            wake_lateness_us: us,
            ..Default::default()
        });
    }
    let snap = stats.snapshot();
    assert_eq!(snap.wake_lateness_us.p50, 2_500);
    assert_eq!(snap.wake_lateness_us.p99, 23_400);
    assert_eq!(snap.wake_lateness_us.max, 23_400);
    assert_eq!(exec_stats.wake_lateness_over_1ms.load(Ordering::Relaxed), 7);
    assert_eq!(exec_stats.wake_lateness_over_5ms.load(Ordering::Relaxed), 4);
    assert_eq!(exec_stats.wake_lateness_over_10ms.load(Ordering::Relaxed), 2);

    // WakeLatenessGuard restores prior thread-local value even across panic unwind.
    let _ = take_current_wake_lateness_ns();
    let res = std::panic::catch_unwind(|| {
        let _g = WakeLatenessGuard::enter(7_500_000);
        panic!("simulated step panic");
    });
    assert!(res.is_err());
    assert_eq!(take_current_wake_lateness_ns(), 0);
}

// ============================================================================
// 2. Legacy owner: HTTP /api/v1/diagnostics/tick with scripted clock
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn legacy_owner_http_diagnostics_reports_exact_lateness_skips_and_sched() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    sup.set_lease_enabled(false).await;
    let clock = LatenessClock::new();
    let stop = Arc::new(AtomicBool::new(false));
    let loop_stats = Arc::new(ExecutorStats::default());
    sup.attach_legacy_loop_stats(loop_stats.clone());

    let rt = tokio::runtime::Handle::current();
    let sup_t = sup.clone();
    let clock_t = clock.clone();
    let stop_t = stop.clone();
    let stats_t = loop_stats.clone();
    let jh = thread::Builder::new()
        .name("armd-control".into())
        .spawn(move || {
            sup_t.set_control_thread_sched(observe_current_thread(&MemoryLock::Skipped(
                "test".into(),
            )));
            run_periodic_with_clock(
                Duration::from_nanos(SCRIPTED_PERIOD_NS),
                &stop_t,
                &stats_t,
                Some(clock_t.as_ref()),
                |dt| {
                    rt.block_on(sup_t.control_tick(dt));
                },
            );
        })
        .unwrap();

    clock.run_schedule(&SCRIPTED_LATENESS_NS);

    let addr = serve(sup.clone()).await;
    let body: serde_json::Value = reqwest::get(format!("http://{addr}/api/v1/diagnostics/tick"))
        .await
        .unwrap()
        .json()
        .await
        .unwrap();

    stop.store(true, Ordering::Release);
    clock.wake();
    jh.join().unwrap();

    assert_eq!(body["owner_mode"], "legacy");
    assert!(body["control_thread"]["policy"].as_str().is_some_and(|p| !p.is_empty()), "{body}");
    assert!(body["control_thread"]["priority"].is_i64(), "{body}");
    assert_eq!(body["window"], 10);
    assert_eq!(body["since_boot_ticks"], 10);
    assert_eq!(body["skipped_releases"], 4);
    assert_eq!(body["diag_dropped"], 0);
    assert_eq!(body["wake_lateness_us"]["p50"], 2_500);
    assert_eq!(body["wake_lateness_us"]["p99"], 23_400);
    assert_eq!(body["wake_lateness_us"]["max"], 23_400);
    assert_eq!(body["wake_lateness_over_1ms"], 7);
    assert_eq!(body["wake_lateness_over_5ms"], 4);
    assert_eq!(body["wake_lateness_over_10ms"], 2);
    assert!(body["max_tick"]["wake_lateness_us"].is_u64(), "{body}");
    for key in ["tick_us", "lock_us", "wait_us", "post_us", "period_us"] {
        assert!(body[key]["max"].is_u64(), "missing {key}");
    }
}

// ============================================================================
// 3. Executor owner: HTTP /api/v1/diagnostics/tick + diag_dropped + zero alloc
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_owner_http_diagnostics_exact_lateness_skips_diag_dropped_and_zero_alloc() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    sup.set_lease_enabled(false).await;
    let clock = LatenessClock::new();

    let mut cfg = ExecutorConfig::for_tests(Duration::from_nanos(SCRIPTED_PERIOD_NS));
    cfg.alloc_timing = true;
    let handle = sup
        .start_executor_with_hooks(
            cfg,
            PersistTestHooks {
                clock: Some(clock.clone()),
                ..Default::default()
            },
        )
        .await
        .unwrap();

    clock.run_schedule(&SCRIPTED_LATENESS_NS);
    assert_eq!(
        handle.stats().completed_releases.load(Ordering::Acquire),
        10
    );
    assert_eq!(
        handle.stats().total_allocs(),
        AllocCounts::ZERO,
        "Q1 timing instrumentation must perform 0 heap ops on armd-control"
    );

    let addr = serve(sup.clone()).await;
    let body: serde_json::Value = reqwest::get(format!("http://{addr}/api/v1/diagnostics/tick"))
        .await
        .unwrap()
        .json()
        .await
        .unwrap();

    assert_eq!(body["owner_mode"], "executor");
    assert!(body["control_thread"]["policy"].as_str().is_some_and(|p| !p.is_empty()), "{body}");
    assert!(body["control_thread"]["priority"].is_i64(), "{body}");
    assert_eq!(body["window"], 10);
    assert_eq!(body["since_boot_ticks"], 10);
    assert_eq!(body["skipped_releases"], 4);
    assert_eq!(body["diag_dropped"], 0);
    assert_eq!(body["wake_lateness_us"]["p50"], 2_500);
    assert_eq!(body["wake_lateness_us"]["p99"], 23_400);
    assert_eq!(body["wake_lateness_us"]["max"], 23_400);
    assert_eq!(body["wake_lateness_over_1ms"], 7);
    assert_eq!(body["wake_lateness_over_5ms"], 4);
    assert_eq!(body["wake_lateness_over_10ms"], 2);
    assert!(body["max_tick"]["wake_lateness_us"].is_u64(), "{body}");

    // Saturate the droppable `diag_tx` ring (cap 256) by stalling `armd-dispatch` at `hub.latest()`
    // via `pause_dispatch_at_hub`, and place a 15 ms lateness + 1 skip on the final dropped release.
    let (rel_hub, hub_jh) = pause_dispatch_at_hub(&sup, &clock);
    let mut sat_schedule = vec![100_000u64; DIAG_RING_CAPACITY + 19];
    sat_schedule.push(15_000_000); // > 10 ms, +1 skip on a guaranteed-dropped TickSample!
    clock.run_schedule(&sat_schedule);
    rel_hub.send(()).unwrap();
    hub_jh.join().unwrap();

    let body2: serde_json::Value = reqwest::get(format!("http://{addr}/api/v1/diagnostics/tick"))
        .await
        .unwrap()
        .json()
        .await
        .unwrap();

    assert_eq!(body2["owner_mode"], "executor");
    assert!(
        body2["diag_dropped"].as_u64().unwrap() >= 10,
        "expected diag_dropped >= 10 under full ring, got {}",
        body2["diag_dropped"]
    );
    // Cumulative counters outside `diag_tx` must include the dropped final release (+1 skip, +1 over 1/5/10 ms).
    assert_eq!(body2["skipped_releases"], 5);
    assert_eq!(body2["wake_lateness_over_1ms"], 8);
    assert_eq!(body2["wake_lateness_over_5ms"], 5);
    assert_eq!(body2["wake_lateness_over_10ms"], 3);
    assert_eq!(
        handle.stats().total_allocs(),
        AllocCounts::ZERO,
        "saturated diag ring Q1 updates must remain 0 heap ops on armd-control"
    );

    handle.stop_and_join();
}

// ============================================================================
// 4. `scripts/rt_qual/analyze.py`: Q1 fields, diag_dropped, and legacy compat
// ============================================================================

fn run_analyze(load_path: &std::path::Path, devstat_path: Option<&std::path::Path>) -> String {
    let manifest = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let script = manifest.join("scripts/rt_qual/analyze.py");
    let mut cmd = Command::new("python3");
    cmd.arg(&script).arg("--load").arg(load_path);
    if let Some(ds) = devstat_path {
        cmd.arg("--devstat").arg(ds);
    }
    let out = cmd.output().expect("run python3 analyze.py");
    assert!(
        out.status.success(),
        "analyze.py failed ({}): {}",
        out.status,
        String::from_utf8_lossy(&out.stderr)
    );
    String::from_utf8(out.stdout).expect("utf-8 report")
}

fn base_tick_json(period_max: u64, tick_max: u64) -> serde_json::Value {
    serde_json::json!({
        "window": 100,
        "tick_us": { "p50": 500, "p99": 900, "max": tick_max },
        "lock_us": { "p50": 400, "p99": 800, "max": tick_max.saturating_sub(50) },
        "late_count": 0,
        "since_boot_ticks": 100,
        "wait_us": { "p50": 0, "p99": 0, "max": 0 },
        "post_us": { "p50": 10, "p99": 20, "max": 30 },
        "period_us": { "p50": 20000, "p99": 20100, "max": period_max },
        "reader_hold_us": { "p50": 0, "p99": 0, "max": 0 },
        "writer_hold_us": { "p50": 0, "p99": 0, "max": 0 },
        "stats_wait_us": { "p50": 0, "p99": 0, "max": 0 },
        "reader_acquire_us": { "p50": 0, "p99": 0, "max": 0 },
        "writer_acquire_us": { "p50": 0, "p99": 0, "max": 0 },
        "lock_phases": {
            "pre_us": { "p50": 10, "p99": 20, "max": 30 },
            "rx_us": { "p50": 100, "p99": 200, "max": 300 },
            "control_us": { "p50": 100, "p99": 200, "max": 300 },
            "tx_us": { "p50": 100, "p99": 200, "max": 300 },
            "finish_us": { "p50": 20, "p99": 30, "max": 40 },
            "obs_us": { "p50": 50, "p99": 80, "max": 100 },
            "publish_us": { "p50": 10, "p99": 20, "max": 30 }
        },
        "max_tick": {
            "tick_us": tick_max, "wait_us": 0, "lock_us": tick_max.saturating_sub(50),
            "post_us": 30, "period_us": period_max,
            "pre_us": 30, "rx_us": 300, "control_us": 300, "tx_us": 300,
            "finish_us": 40, "obs_us": 100, "publish_us": 30,
            "lock_cpu_us": 500, "lock_vcsw": 0, "lock_ivcsw": 0,
            "lock_majflt": 0, "lock_minflt": 0, "lock_alloc_calls": 0,
            "waited_for": null, "longest_hold": null
        },
        "max_lock": null,
        "control_thread": {
            "policy": "fifo", "priority": 45, "realtime": true,
            "requested_priority": 45, "memory_locked": true, "memory_lock": "locked"
        }
    })
}

#[test]
fn analyze_py_uses_q1_fields_handles_diag_dropped_and_keeps_legacy_compat() {
    let dir = tempfile::tempdir().unwrap();

    // (a) Legacy tickload.json lacking Q1/monotonic fields: proxies & gaps preserved, verdict DIAGNOSTIC ONLY.
    let legacy_path = dir.path().join("legacy_tickload.json");
    let t0 = base_tick_json(22_000, 1_500);
    let mut t1 = base_tick_json(23_000, 1_800);
    t1["since_boot_ticks"] = serde_json::json!(200);
    let legacy = serde_json::json!({ "fault": null, "ticks": [t0, t1] });
    std::fs::write(&legacy_path, serde_json::to_string_pretty(&legacy).unwrap()).unwrap();
    let legacy_rep = run_analyze(&legacy_path, None);
    assert!(legacy_rep.contains("**Overall Verdict: DIAGNOSTIC ONLY"), "{legacy_rep}");
    assert!(legacy_rep.contains("PASS (proxy)"), "{legacy_rep}");
    assert!(legacy_rep.contains("Wake Overrun Proxy (ms)"), "{legacy_rep}");
    assert!(legacy_rep.contains("3.00 (sched overrun proxy)"), "{legacy_rep}");
    assert!(legacy_rep.contains("`skipped_releases`:"), "{legacy_rep}");
    assert!(legacy_rep.contains("`wake_lateness_us`:"), "{legacy_rep}");

    // (b) Full Q1 JSONL with release_us + monotonic counters + diag_dropped == 0:
    //     real fields used, closed gaps omitted!
    let q1_path = dir.path().join("q1_clean.jsonl");
    let mut q0 = base_tick_json(26_000, 1_500);
    q0["instance_id"] = serde_json::json!("armd-test-1");
    q0["boot_id"] = serde_json::json!("boot-uuid-test-1");
    q0["process_start_unix_ns"] = serde_json::json!(1_700_000_000_000_000_000u64);
    q0["owner_mode"] = serde_json::json!("executor");
    q0["loop_active"] = serde_json::json!(true);
    q0["stats_snapshot_lapped"] = serde_json::json!(false);
    q0["stats_writer_drops"] = serde_json::json!(0);
    q0["in_flight_release_us"] = serde_json::json!(0);
    q0["release_us"] = serde_json::json!({ "p50": 600, "p99": 1000, "max": 1600 });
    q0["releases_started"] = serde_json::json!(100);
    q0["releases_total"] = serde_json::json!(100);
    q0["release_us_max_ever"] = serde_json::json!(1600);
    q0["releases_over_10ms"] = serde_json::json!(0);
    q0["releases_over_15ms"] = serde_json::json!(0);
    q0["releases_over_20ms"] = serde_json::json!(0);
    q0["wake_late_max_ever"] = serde_json::json!(750);
    q0["skipped_releases"] = serde_json::json!(0);
    q0["diag_dropped"] = serde_json::json!(0);
    q0["wake_lateness_us"] = serde_json::json!({ "p50": 120, "p99": 400, "max": 750 });
    q0["wake_lateness_over_1ms"] = serde_json::json!(0);
    q0["wake_lateness_over_5ms"] = serde_json::json!(0);
    q0["wake_lateness_over_10ms"] = serde_json::json!(0);

    let mut q_mid = q0.clone();
    q_mid["since_boot_ticks"] = serde_json::json!(250);
    q_mid["releases_started"] = serde_json::json!(250);
    q_mid["releases_total"] = serde_json::json!(250);
    q_mid["wake_lateness_us"] = serde_json::json!({ "p50": 150, "p99": 500, "max": 800 });
    q_mid["wake_late_max_ever"] = serde_json::json!(800);

    let mut q1 = q_mid.clone();
    q1["since_boot_ticks"] = serde_json::json!(300);
    q1["releases_started"] = serde_json::json!(300);
    q1["releases_total"] = serde_json::json!(300);

    let q1_lines = format!(
        "{}\n{}\n{}\n{}\n{}\n",
        serde_json::json!({ "type": "meta", "run_id": "q1-clean", "enabled_loads": { "obs_poll": true } }),
        serde_json::json!({ "type": "sample", "run_id": "q1-clean", "phase": "baseline", "elapsed_s": 0.0, "tick": q0, "counters": { "obs_ok": 0 } }),
        serde_json::json!({ "type": "sample", "run_id": "q1-clean", "phase": "idle_tail_start", "elapsed_s": 3.0, "tick": q_mid, "counters": { "obs_ok": 20 } }),
        serde_json::json!({ "type": "sample", "run_id": "q1-clean", "phase": "final", "elapsed_s": 4.2, "tick": q1, "counters": { "obs_ok": 20 }, "safety_state": "Idle" }),
        serde_json::json!({ "type": "summary", "run_id": "q1-clean", "stop_reason": "completed", "fault": null, "safety_state": "Idle", "terminal_safety_verified": true, "counters": { "obs_ok": 20 } }),
    );
    std::fs::write(&q1_path, q1_lines).unwrap();
    let q1_rep = run_analyze(&q1_path, None);
    assert!(q1_rep.contains("**Owner mode**: `executor`"), "{q1_rep}");
    assert!(q1_rep.contains("`wake_lateness_us` p99/max (ms)"), "{q1_rep}");
    assert!(q1_rep.contains("0.80 (wake lateness)"), "must use real wake_lateness_us.max (0.80 ms), not period proxy (6.00 ms):\n{q1_rep}");
    assert!(q1_rep.contains("skipped_releases delta=0"), "{q1_rep}");
    assert!(!q1_rep.contains("Wake Overrun Proxy"), "{q1_rep}");
    let gaps_sec = q1_rep.split("## 6. Diagnostic Gaps").nth(1).expect("gaps section");
    assert!(!gaps_sec.contains("`skipped_releases`:"), "closed gap should be omitted:\n{gaps_sec}");
    assert!(!gaps_sec.contains("`wake_lateness_us`:"), "closed gap should be omitted:\n{gaps_sec}");

    // (c) All other gates PASS + devstat confirms ARMD_ALLOC_TIMING=1, EXCEPT diag_dropped delta > 0
    // -> MUST force Overall Verdict to INCOMPLETE (not PASS)!
    let devstat_path = dir.path().join("devstat.txt");
    std::fs::write(
        &devstat_path,
        "=== META ===\nrun_id=q1-drop\n=== ONCE_START ===\n--- armd_version ---\nARMD_ALLOC_TIMING=1\n=== ONCE_END ===\n",
    )
    .unwrap();
    let drop_path = dir.path().join("q1_drop.jsonl");
    let mut d0 = q0.clone();
    d0["control_thread_lock_attempts"] = serde_json::json!(0);
    d0["control_thread_alloc_ops"] = serde_json::json!(0);
    d0["enqueue_to_wire_us"] = serde_json::json!({ "p50": 100, "p99": 200, "max": 250 });
    let mut d_mid = d0.clone();
    d_mid["since_boot_ticks"] = serde_json::json!(250);
    d_mid["releases_started"] = serde_json::json!(250);
    d_mid["releases_total"] = serde_json::json!(250);
    d_mid["diag_dropped"] = serde_json::json!(5);
    let mut d1 = d_mid.clone();
    d1["since_boot_ticks"] = serde_json::json!(300);
    d1["releases_started"] = serde_json::json!(300);
    d1["releases_total"] = serde_json::json!(300);
    let drop_lines = format!(
        "{}\n{}\n{}\n{}\n{}\n",
        serde_json::json!({ "type": "meta", "run_id": "q1-drop", "enabled_loads": { "obs_poll": true } }),
        serde_json::json!({ "type": "sample", "run_id": "q1-drop", "phase": "baseline", "elapsed_s": 0.0, "tick": d0, "counters": { "obs_ok": 0 } }),
        serde_json::json!({ "type": "sample", "run_id": "q1-drop", "phase": "idle_tail_start", "elapsed_s": 3.0, "tick": d_mid, "counters": { "obs_ok": 20 } }),
        serde_json::json!({ "type": "sample", "run_id": "q1-drop", "phase": "final", "elapsed_s": 4.2, "tick": d1, "counters": { "obs_ok": 20 }, "safety_state": "Idle" }),
        serde_json::json!({ "type": "summary", "run_id": "q1-drop", "stop_reason": "completed", "fault": null, "safety_state": "Idle", "terminal_safety_verified": true, "counters": { "obs_ok": 20 } }),
    );
    std::fs::write(&drop_path, drop_lines).unwrap();
    let drop_rep = run_analyze(&drop_path, Some(&devstat_path));
    assert!(
        drop_rep.contains("**Overall Verdict: INCOMPLETE (diagnostic samples dropped: diag_dropped delta=5)**"),
        "{drop_rep}"
    );
    assert!(drop_rep.contains("INCOMPLETE (diag_dropped > 0)"), "{drop_rep}");

    // (d) diag_dropped > 0 WITH an observed in-run release breach (>= 20 ms) -> MUST still be FAIL!
    let breach_path = dir.path().join("q1_drop_breach.jsonl");
    let mut b1 = d1.clone();
    b1["release_us"]["max"] = serde_json::json!(25_000);
    b1["release_us_max_ever"] = serde_json::json!(25_000);
    b1["releases_over_20ms"] = serde_json::json!(1);
    b1["tick_us"]["max"] = serde_json::json!(25_000);
    b1["max_tick"]["tick_us"] = serde_json::json!(25_000);
    let breach_lines = format!(
        "{}\n{}\n{}\n{}\n",
        serde_json::json!({ "type": "meta", "run_id": "q1-drop-breach" }),
        serde_json::json!({ "type": "sample", "run_id": "q1-drop-breach", "phase": "baseline", "tick": d0 }),
        serde_json::json!({ "type": "sample", "run_id": "q1-drop-breach", "phase": "final", "tick": b1 }),
        serde_json::json!({ "type": "summary", "run_id": "q1-drop-breach", "stop_reason": "completed", "fault": null }),
    );
    std::fs::write(&breach_path, breach_lines).unwrap();
    let breach_rep = run_analyze(&breach_path, Some(&devstat_path));
    assert!(breach_rep.contains("**Overall Verdict: FAIL**"), "{breach_rep}");

    // (e) Missing required `diag_dropped` with all other criteria clean -> MUST be INVALID (not PASS)!
    let missing_dd_path = dir.path().join("q1_missing_diag_dropped.jsonl");
    let mut m0 = d0.clone();
    m0.as_object_mut().unwrap().remove("diag_dropped");
    let mut m1 = m0.clone();
    m1["since_boot_ticks"] = serde_json::json!(300);
    m1["releases_started"] = serde_json::json!(300);
    m1["releases_total"] = serde_json::json!(300);
    std::fs::write(
        &missing_dd_path,
        format!(
            "{}\n{}\n{}\n{}\n",
            serde_json::json!({ "type": "meta", "run_id": "q1-missing-dd" }),
            serde_json::json!({ "type": "sample", "run_id": "q1-missing-dd", "phase": "baseline", "tick": m0 }),
            serde_json::json!({ "type": "sample", "run_id": "q1-missing-dd", "phase": "final", "tick": m1 }),
            serde_json::json!({ "type": "summary", "run_id": "q1-missing-dd", "stop_reason": "completed", "fault": null }),
        ),
    )
    .unwrap();
    let missing_dd_rep = run_analyze(&missing_dd_path, Some(&devstat_path));
    assert!(missing_dd_rep.contains("**Overall Verdict: INVALID"), "{missing_dd_rep}");
    assert!(missing_dd_rep.contains("diag_dropped (2/2)"), "{missing_dd_rep}");

    // (f) Mid-run counter reset `0 -> 7 -> 0` on `diag_dropped` & `since_boot_ticks` decrease -> INVALID!
    let reset_dd_path = dir.path().join("q1_reset_dd.jsonl");
    let mut r0 = d0.clone();
    r0["diag_dropped"] = serde_json::json!(0);
    let mut r1 = d0.clone();
    r1["since_boot_ticks"] = serde_json::json!(200);
    r1["releases_started"] = serde_json::json!(200);
    r1["releases_total"] = serde_json::json!(200);
    r1["diag_dropped"] = serde_json::json!(7);
    let mut r2 = d0.clone();
    r2["since_boot_ticks"] = serde_json::json!(50);
    r2["releases_started"] = serde_json::json!(50);
    r2["releases_total"] = serde_json::json!(50);
    r2["diag_dropped"] = serde_json::json!(0);
    std::fs::write(
        &reset_dd_path,
        format!(
            "{}\n{}\n{}\n{}\n{}\n",
            serde_json::json!({ "type": "meta", "run_id": "q1-reset-dd" }),
            serde_json::json!({ "type": "sample", "run_id": "q1-reset-dd", "phase": "baseline", "tick": r0 }),
            serde_json::json!({ "type": "sample", "run_id": "q1-reset-dd", "phase": "full_load", "tick": r1 }),
            serde_json::json!({ "type": "sample", "run_id": "q1-reset-dd", "phase": "final", "tick": r2 }),
            serde_json::json!({ "type": "summary", "run_id": "q1-reset-dd", "stop_reason": "completed", "fault": null }),
        ),
    )
    .unwrap();
    let reset_dd_rep = run_analyze(&reset_dd_path, Some(&devstat_path));
    assert!(reset_dd_rep.contains("**Overall Verdict: INVALID"), "{reset_dd_rep}");
    assert!(reset_dd_rep.contains("counter decreased mid-run (restart/reset): releases_total"), "{reset_dd_rep}");

    // (g) Mid-run counter reset `0 -> 7 -> 0` on `skipped_releases` (verified in-run breach 0->7 before reset) -> FAIL!
    let reset_skip_path = dir.path().join("q1_reset_skip.jsonl");
    let mut s0 = d0.clone();
    s0["skipped_releases"] = serde_json::json!(0);
    let mut s1 = d0.clone();
    s1["since_boot_ticks"] = serde_json::json!(200);
    s1["releases_started"] = serde_json::json!(200);
    s1["releases_total"] = serde_json::json!(200);
    s1["skipped_releases"] = serde_json::json!(7);
    let mut s2 = d0.clone();
    s2["since_boot_ticks"] = serde_json::json!(50);
    s2["releases_started"] = serde_json::json!(50);
    s2["releases_total"] = serde_json::json!(50);
    s2["skipped_releases"] = serde_json::json!(0);
    std::fs::write(
        &reset_skip_path,
        format!(
            "{}\n{}\n{}\n{}\n{}\n",
            serde_json::json!({ "type": "meta", "run_id": "q1-reset-skip" }),
            serde_json::json!({ "type": "sample", "run_id": "q1-reset-skip", "phase": "baseline", "tick": s0 }),
            serde_json::json!({ "type": "sample", "run_id": "q1-reset-skip", "phase": "full_load", "tick": s1 }),
            serde_json::json!({ "type": "sample", "run_id": "q1-reset-skip", "phase": "final", "tick": s2 }),
            serde_json::json!({ "type": "summary", "run_id": "q1-reset-skip", "stop_reason": "completed", "fault": null }),
        ),
    )
    .unwrap();
    let reset_skip_rep = run_analyze(&reset_skip_path, Some(&devstat_path));
    assert!(reset_skip_rep.contains("**Overall Verdict: FAIL**"), "{reset_skip_rep}");
    assert!(reset_skip_rep.contains("skipped_releases delta=7"), "{reset_skip_rep}");

    // (h) Partial missing `diag_dropped` on middle sample -> INVALID!
    let partial_dd_path = dir.path().join("q1_partial_dd.jsonl");
    let mut p1 = d0.clone();
    p1["since_boot_ticks"] = serde_json::json!(200);
    p1["releases_started"] = serde_json::json!(200);
    p1["releases_total"] = serde_json::json!(200);
    p1.as_object_mut().unwrap().remove("diag_dropped");
    let mut p2 = d0.clone();
    p2["since_boot_ticks"] = serde_json::json!(300);
    p2["releases_started"] = serde_json::json!(300);
    p2["releases_total"] = serde_json::json!(300);
    std::fs::write(
        &partial_dd_path,
        format!(
            "{}\n{}\n{}\n{}\n{}\n",
            serde_json::json!({ "type": "meta", "run_id": "q1-partial-dd" }),
            serde_json::json!({ "type": "sample", "run_id": "q1-partial-dd", "phase": "baseline", "tick": d0 }),
            serde_json::json!({ "type": "sample", "run_id": "q1-partial-dd", "phase": "full_load", "tick": p1 }),
            serde_json::json!({ "type": "sample", "run_id": "q1-partial-dd", "phase": "final", "tick": p2 }),
            serde_json::json!({ "type": "summary", "run_id": "q1-partial-dd", "stop_reason": "completed", "fault": null }),
        ),
    )
    .unwrap();
    let partial_dd_rep = run_analyze(&partial_dd_path, Some(&devstat_path));
    assert!(partial_dd_rep.contains("**Overall Verdict: INVALID"), "{partial_dd_rep}");
    assert!(partial_dd_rep.contains("diag_dropped (1/3)"), "{partial_dd_rep}");

    // (i) Recurring phase `full_load -> cooldown -> full_load`: skip occurring during `cooldown`
    // must be attributed strictly to `cooldown` (2 / 0), NOT `full_load` (0 / 0)!
    let recur_path = dir.path().join("q1_recurring_phase.jsonl");
    let mut f0 = q0.clone();
    f0["skipped_releases"] = serde_json::json!(0);
    let mut c1 = q0.clone();
    c1["since_boot_ticks"] = serde_json::json!(200);
    c1["releases_started"] = serde_json::json!(200);
    c1["releases_total"] = serde_json::json!(200);
    c1["skipped_releases"] = serde_json::json!(2);
    let mut f2 = q0.clone();
    f2["since_boot_ticks"] = serde_json::json!(300);
    f2["releases_started"] = serde_json::json!(300);
    f2["releases_total"] = serde_json::json!(300);
    f2["skipped_releases"] = serde_json::json!(2);
    std::fs::write(
        &recur_path,
        format!(
            "{}\n{}\n{}\n{}\n{}\n",
            serde_json::json!({ "type": "meta", "run_id": "q1-recur" }),
            serde_json::json!({ "type": "sample", "run_id": "q1-recur", "phase": "full_load", "tick": f0 }),
            serde_json::json!({ "type": "sample", "run_id": "q1-recur", "phase": "cooldown", "tick": c1 }),
            serde_json::json!({ "type": "sample", "run_id": "q1-recur", "phase": "full_load", "tick": f2 }),
            serde_json::json!({ "type": "summary", "run_id": "q1-recur", "stop_reason": "completed", "fault": null }),
        ),
    )
    .unwrap();
    let recur_rep = run_analyze(&recur_path, None);
    let fl_line = recur_rep.lines().find(|l| l.starts_with("| `full_load` |")).expect("full_load row");
    let cd_line = recur_rep.lines().find(|l| l.starts_with("| `cooldown` |")).expect("cooldown row");
    assert!(fl_line.contains("| 0 / 0 |"), "full_load must show 0 skips, got: {fl_line}");
    assert!(cd_line.contains("| 2 / 0 |"), "cooldown must show 2 skips, got: {cd_line}");
}

// ============================================================================
// 5. Full `release_us` (25 ms post-publish), >1000-tick rollover, & baseline mark (both owners)
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn post_publish_25ms_delay_rollover_and_baseline_mark_both_owners() {
    // Exact boundary classification: 19_999_999 ns is < 20 ms; 20_000_000 ns is >= 20 ms.
    assert_eq!(classify_release_duration_ns(9_999_999), (9_999, false, false, false));
    assert_eq!(classify_release_duration_ns(10_000_000), (10_000, true, false, false));
    assert_eq!(classify_release_duration_ns(14_999_999), (14_999, true, false, false));
    assert_eq!(classify_release_duration_ns(15_000_000), (15_000, true, true, false));
    assert_eq!(classify_release_duration_ns(19_999_999), (19_999, true, true, false));
    assert_eq!(classify_release_duration_ns(20_000_000), (20_000, true, true, true));
    assert_eq!(classify_release_duration_ns(u64::MAX), (u32::MAX, true, true, true));

    // ---- (A) Legacy owner: 25 ms post-publish work + >1000-tick rollover + baseline mark ----
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.set_lease_enabled(false).await;
        let clock = LatenessClock::new();
        let stop = Arc::new(AtomicBool::new(false));
        let loop_stats = Arc::new(ExecutorStats::default());
        sup.attach_legacy_loop_stats(loop_stats.clone());
        let post_delay_ns = Arc::new(std::sync::atomic::AtomicU64::new(25_000_000));

        let rt = tokio::runtime::Handle::current();
        let sup_t = sup.clone();
        let clock_t = clock.clone();
        let stop_t = stop.clone();
        let stats_t = loop_stats.clone();
        let delay_t = post_delay_ns.clone();
        let jh = thread::Builder::new()
            .name("armd-control".into())
            .spawn(move || {
                run_periodic_with_clock(
                    Duration::from_millis(20),
                    &stop_t,
                    &stats_t,
                    Some(clock_t.as_ref()),
                    |dt| {
                        rt.block_on(sup_t.control_tick(dt));
                        // Simulate work AFTER tick_shared/StateHub publication and tick_us end mark.
                        let d = delay_t.swap(0, Ordering::AcqRel);
                        if d > 0 {
                            clock_t.advance_ns(d);
                        }
                    },
                );
            })
            .unwrap();

        clock.run_schedule(&[0]);
        let addr = serve(sup.clone()).await;
        let client = reqwest::Client::new();
        let body: serde_json::Value = client
            .get(format!("http://{addr}/api/v1/diagnostics/tick"))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!(body["owner_mode"], "legacy");
        assert!(body["instance_id"].as_str().is_some_and(|s| !s.is_empty()));
        assert!(body["tick_us"]["max"].as_u64().unwrap() < 20_000, "tick_us missed post-publish delay: {body}");
        assert!(body["release_us"]["max"].as_u64().unwrap() >= 25_000, "release_us must include 25ms post-publish work: {body}");
        assert!(body["release_us_max_ever"].as_u64().unwrap() >= 25_000, "{body}");
        assert_eq!(body["releases_over_20ms"], 1);
        assert_eq!(body["releases_over_15ms"], 1);
        assert_eq!(body["releases_over_10ms"], 1);
        assert_eq!(body["releases_total"], 1);

        // Roll over TICK_WINDOW + 50 clean releases: window max ages out, monotonic peaks persist!
        let clean = vec![0u64; TICK_WINDOW + 50];
        clock.run_schedule(&clean);
        let rolled: serde_json::Value = client
            .get(format!("http://{addr}/api/v1/diagnostics/tick"))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!(rolled["window"], TICK_WINDOW);
        assert!(
            rolled["release_us"]["max"].as_u64().unwrap() < 20_000,
            "windowed release_us.max should have aged out past 1000 ticks: {}",
            rolled["release_us"]
        );
        assert!(rolled["release_us_max_ever"].as_u64().unwrap() >= 25_000);
        assert_eq!(rolled["releases_over_20ms"], 1);
        assert_eq!(rolled["releases_total"], (TICK_WINDOW + 51) as u64);

        // Operator baseline marker: pending -> active after next release; superseded POSTs collapse cleanly.
        let m1: serde_json::Value = client
            .post(format!("http://{addr}/api/v1/diagnostics/tick/mark"))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!(m1["marker_id"], 1);
        assert_eq!(m1["status"], "pending");
        let pending_get: serde_json::Value = client
            .get(format!("http://{addr}/api/v1/diagnostics/tick"))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!(pending_get["since_marker"]["marker_id"], 1);
        assert_eq!(pending_get["since_marker"]["status"], "pending");

        // Overlapping mark request #2 supersedes #1 before the next release acknowledges.
        let m2: serde_json::Value = client
            .post(format!("http://{addr}/api/v1/diagnostics/tick/mark"))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!(m2["marker_id"], 2);

        // 1 release acknowledges marker #2 at its completion (not charged to marker #2),
        // then 2 post-marker releases run (one with 12 ms duration + 3 ms wake lateness).
        clock.run_schedule(&[0]);
        post_delay_ns.store(12_000_000, Ordering::Release);
        clock.run_schedule(&[3_000_000, 0]);
        let marked: serde_json::Value = client
            .get(format!("http://{addr}/api/v1/diagnostics/tick"))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!(marked["since_marker"]["marker_id"], 2);
        assert_eq!(marked["since_marker"]["status"], "active");
        assert_eq!(marked["since_marker"]["releases_total_delta"], 2);
        assert_eq!(marked["since_marker"]["releases_over_10ms_delta"], 1);
        assert_eq!(marked["since_marker"]["releases_over_20ms_delta"], 0);
        assert_eq!(marked["since_marker"]["release_us_max"], 12_000);
        assert_eq!(marked["since_marker"]["wake_late_max"], 3_000);
        // Process-lifetime peaks never reset!
        assert!(marked["release_us_max_ever"].as_u64().unwrap() >= 25_000);
        assert_eq!(marked["releases_over_20ms"], 1);

        stop.store(true, Ordering::Release);
        clock.wake();
        jh.join().unwrap();

        // Stopped legacy loop -> POST /mark must return 503.
        let stopped_resp = client
            .post(format!("http://{addr}/api/v1/diagnostics/tick/mark"))
            .send()
            .await
            .unwrap();
        assert_eq!(stopped_resp.status(), reqwest::StatusCode::SERVICE_UNAVAILABLE);
    }

    // ---- (B) Legacy owner on production wall clock: real 25 ms post-publish sleep ----
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.set_lease_enabled(false).await;
        let stop = Arc::new(AtomicBool::new(false));
        let loop_stats = Arc::new(ExecutorStats::default());
        sup.attach_legacy_loop_stats(loop_stats.clone());
        let slept = Arc::new(AtomicBool::new(false));

        let rt = tokio::runtime::Handle::current();
        let sup_t = sup.clone();
        let stop_t = stop.clone();
        let stats_t = loop_stats.clone();
        let slept_t = slept.clone();
        let jh = thread::Builder::new()
            .name("armd-control".into())
            .spawn(move || {
                armd::rt_core::executor::run_periodic(
                    Duration::from_millis(5),
                    &stop_t,
                    &stats_t,
                    |dt| {
                        rt.block_on(sup_t.control_tick(dt));
                        if !slept_t.swap(true, Ordering::AcqRel) {
                            thread::sleep(Duration::from_millis(25));
                        } else {
                            stop_t.store(true, Ordering::Release);
                        }
                    },
                );
            })
            .unwrap();
        jh.join().unwrap();
        let snap = sup.tick_stats();
        assert!(snap.release_us.max >= 25_000, "real clock release_us.max={}", snap.release_us.max);
        assert!(snap.release_us_max_ever >= 25_000);
        assert_eq!(snap.releases_over_20ms, 1);
    }

    // ---- (C) Executor owner: 25 ms post-publish work + >1000-tick rollover + mark + 0 alloc ----
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.set_lease_enabled(false).await;
        let clock = LatenessClock::new();
        let post_delay = Arc::new(Mutex::new(Some(Duration::from_millis(25))));

        let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(20));
        cfg.alloc_timing = true;
        let handle = sup
            .start_executor_with_hooks(
                cfg,
                PersistTestHooks {
                    clock: Some(clock.clone()),
                    post_publish_delay_once: Some(post_delay.clone()),
                    ..Default::default()
                },
            )
            .await
            .unwrap();

        clock.run_schedule(&[0]);
        assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);
        let addr = serve(sup.clone()).await;
        let client = reqwest::Client::new();
        let body: serde_json::Value = client
            .get(format!("http://{addr}/api/v1/diagnostics/tick"))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!(body["owner_mode"], "executor");
        assert!(
            body["boot_id"].as_str().is_some_and(|s| !s.is_empty()),
            "GET /diagnostics/tick must expose non-empty boot_id: {body}"
        );
        assert!(
            body["process_start_unix_ns"].as_u64().is_some_and(|ns| ns > 0),
            "GET /diagnostics/tick must expose positive process_start_unix_ns: {body}"
        );
        assert!(body["tick_us"]["max"].as_u64().unwrap() < 20_000, "tick_us should exclude post-publish delay: {body}");
        assert!(body["release_us"]["max"].as_u64().unwrap() >= 25_000, "release_us must include 25ms post-publish delay: {body}");
        assert!(body["release_us_max_ever"].as_u64().unwrap() >= 25_000);
        assert_eq!(body["releases_over_20ms"], 1);
        assert_eq!(body["releases_over_15ms"], 1);
        assert_eq!(body["releases_over_10ms"], 1);
        assert_eq!(body["releases_total"], 1);

        // Roll over >1000 clean releases in chunks smaller than DIAG_RING_CAPACITY so diag_dropped stays 0.
        let chunk = vec![0u64; 100];
        for _ in 0..11 {
            clock.run_schedule(&chunk);
            let _ = sup.tick_stats();
        }
        let rolled: serde_json::Value = client
            .get(format!("http://{addr}/api/v1/diagnostics/tick"))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!(rolled["window"], TICK_WINDOW);
        assert_eq!(rolled["diag_dropped"], 0);
        assert!(
            rolled["release_us"]["max"].as_u64().unwrap() < 20_000,
            "windowed release_us.max must have aged out after 1100 ticks: {}",
            rolled["release_us"]
        );
        assert!(rolled["release_us_max_ever"].as_u64().unwrap() >= 25_000);
        assert_eq!(rolled["releases_over_20ms"], 1);
        assert_eq!(rolled["releases_total"], 1101);

        // Baseline mark in executor mode + 0 heap ops on armd-control.
        let mark_resp: serde_json::Value = client
            .post(format!("http://{addr}/api/v1/diagnostics/tick/mark"))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!(mark_resp["marker_id"], 1);
        assert_eq!(mark_resp["status"], "pending");
        clock.run_schedule(&[0]); // acknowledges marker #1 at end of release
        *post_delay.lock().unwrap() = Some(Duration::from_millis(16));
        clock.run_schedule(&[2_000_000, 0]);
        let marked: serde_json::Value = client
            .get(format!("http://{addr}/api/v1/diagnostics/tick"))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!(marked["since_marker"]["marker_id"], 1);
        assert_eq!(marked["since_marker"]["status"], "active");
        assert_eq!(marked["since_marker"]["releases_total_delta"], 2);
        assert_eq!(marked["since_marker"]["releases_over_15ms_delta"], 1);
        assert_eq!(marked["since_marker"]["releases_over_20ms_delta"], 0);
        assert_eq!(marked["since_marker"]["release_us_max"], 16_000);
        assert_eq!(marked["since_marker"]["wake_late_max"], 2_000);
        assert_eq!(
            handle.stats().total_allocs(),
            AllocCounts::ZERO,
            "monotonic counters + baseline marker must perform 0 heap ops on armd-control"
        );

        handle.stop_and_join();
        let stopped_resp = client
            .post(format!("http://{addr}/api/v1/diagnostics/tick/mark"))
            .send()
            .await
            .unwrap();
        assert_eq!(stopped_resp.status(), reqwest::StatusCode::SERVICE_UNAVAILABLE);
    }
}

// ============================================================================
// 6. Operator auth & same-origin enforcement on POST /api/v1/diagnostics/tick/mark (both owners)
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn mark_endpoint_enforces_operator_auth_and_same_origin_both_owners() {
    let client = reqwest::Client::new();

    // (a) Legacy owner with OperatorAuth enforced
    {
        let auth = armd::OperatorAuth::enforced("1234", 3600);
        let (token, _) = auth.unlock("1234").unwrap();
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let clock = LatenessClock::new();
        let stop = Arc::new(AtomicBool::new(false));
        let loop_stats = Arc::new(ExecutorStats::default());
        sup.attach_legacy_loop_stats(loop_stats.clone());
        let rt = tokio::runtime::Handle::current();
        let sup_t = sup.clone();
        let clock_t = clock.clone();
        let stop_t = stop.clone();
        let stats_t = loop_stats.clone();
        let jh = thread::spawn(move || {
            run_periodic_with_clock(Duration::from_millis(20), &stop_t, &stats_t, Some(clock_t.as_ref()), |dt| {
                rt.block_on(sup_t.control_tick(dt));
            });
        });
        clock.run_schedule(&[0]);
        let addr = serve_with_auth(sup.clone(), auth).await;
        let url = format!("http://{addr}/api/v1/diagnostics/tick/mark");

        // Missing token -> 401/403
        let r_unauth = client.post(&url).send().await.unwrap();
        assert!(
            matches!(r_unauth.status(), reqwest::StatusCode::UNAUTHORIZED | reqwest::StatusCode::FORBIDDEN),
            "expected 401/403 without token, got {}",
            r_unauth.status()
        );
        // Cross-origin -> 403 even with valid token
        let r_xorigin = client
            .post(&url)
            .header("Origin", "http://evil.example")
            .header("Authorization", format!("Bearer {token}"))
            .send()
            .await
            .unwrap();
        assert_eq!(r_xorigin.status(), reqwest::StatusCode::FORBIDDEN);
        // Valid same-origin operator token -> 200
        let r_ok = client
            .post(&url)
            .header("Authorization", format!("Bearer {token}"))
            .send()
            .await
            .unwrap();
        assert_eq!(r_ok.status(), reqwest::StatusCode::OK);

        stop.store(true, Ordering::Release);
        clock.wake();
        jh.join().unwrap();
    }

    // (b) Executor owner with OperatorAuth enforced
    {
        let auth = armd::OperatorAuth::enforced("1234", 3600);
        let (token, _) = auth.unlock("1234").unwrap();
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let clock = LatenessClock::new();
        let handle = sup
            .start_executor_with_hooks(
                ExecutorConfig::for_tests(Duration::from_millis(20)),
                PersistTestHooks {
                    clock: Some(clock.clone()),
                    ..Default::default()
                },
            )
            .await
            .unwrap();
        clock.run_schedule(&[0]);
        let addr = serve_with_auth(sup.clone(), auth).await;
        let url = format!("http://{addr}/api/v1/diagnostics/tick/mark");

        let r_unauth = client.post(&url).send().await.unwrap();
        assert!(
            matches!(r_unauth.status(), reqwest::StatusCode::UNAUTHORIZED | reqwest::StatusCode::FORBIDDEN),
            "expected 401/403 in executor mode without token, got {}",
            r_unauth.status()
        );
        let r_xorigin = client
            .post(&url)
            .header("Origin", "http://evil.example")
            .header("Authorization", format!("Bearer {token}"))
            .send()
            .await
            .unwrap();
        assert_eq!(r_xorigin.status(), reqwest::StatusCode::FORBIDDEN);
        let r_ok = client
            .post(&url)
            .header("Authorization", format!("Bearer {token}"))
            .send()
            .await
            .unwrap();
        assert_eq!(r_ok.status(), reqwest::StatusCode::OK);
        handle.stop_and_join();
    }
}

// ============================================================================
// 7. Shutdown final-overrun skips, clean shutdown, panic accounting, & bounded odd stats_seq
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn shutdown_overrun_skips_clean_exit_and_panic_accounting_both_owners() {
    // (a) Legacy: 25 ms overrun on final release (20 ms period) -> 1 skip accounted at loop exit!
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let clock = LatenessClock::new();
        let stop = Arc::new(AtomicBool::new(false));
        let loop_stats = Arc::new(ExecutorStats::default());
        sup.attach_legacy_loop_stats(loop_stats.clone());
        let rt = tokio::runtime::Handle::current();
        let sup_t = sup.clone();
        let clock_t = clock.clone();
        let stop_t = stop.clone();
        let stats_t = loop_stats.clone();
        let jh = thread::spawn(move || {
            run_periodic_with_clock(Duration::from_millis(20), &stop_t, &stats_t, Some(clock_t.as_ref()), |dt| {
                rt.block_on(sup_t.control_tick(dt));
                clock_t.advance_ns(25_000_000);
            });
        });
        clock.run_schedule(&[0]);
        stop.store(true, Ordering::Release);
        clock.wake();
        jh.join().unwrap();
        assert_eq!(loop_stats.skipped_releases.load(Ordering::Acquire), 1);
        assert_eq!(loop_stats.releases_over_20ms.load(Ordering::Acquire), 1);
        assert_eq!(loop_stats.releases_total.load(Ordering::Acquire), 1);
    }

    // (b) Legacy: clean release followed by late stop wake past next_ns -> 0 false-positive skips!
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let clock = LatenessClock::new();
        let stop = Arc::new(AtomicBool::new(false));
        let loop_stats = Arc::new(ExecutorStats::default());
        sup.attach_legacy_loop_stats(loop_stats.clone());
        let rt = tokio::runtime::Handle::current();
        let sup_t = sup.clone();
        let clock_t = clock.clone();
        let stop_t = stop.clone();
        let stats_t = loop_stats.clone();
        let jh = thread::spawn(move || {
            run_periodic_with_clock(Duration::from_millis(20), &stop_t, &stats_t, Some(clock_t.as_ref()), |dt| {
                rt.block_on(sup_t.control_tick(dt));
            });
        });
        clock.run_schedule(&[0]);
        // Advance clock 30 ms while parked in sleep_until, then stop: release did NOT overrun!
        clock.advance_ns(30_000_000);
        stop.store(true, Ordering::Release);
        clock.wake();
        jh.join().unwrap();
        assert_eq!(loop_stats.skipped_releases.load(Ordering::Acquire), 0);
        assert_eq!(loop_stats.releases_total.load(Ordering::Acquire), 1);
    }

    // (c) Legacy: first-release panic after 25 ms overrun accounts both release duration and exit skip!
    {
        let clock = LatenessClock::new();
        let stop = Arc::new(AtomicBool::new(false));
        let loop_stats = Arc::new(ExecutorStats::default());
        let clock_t = clock.clone();
        let stop_t = stop.clone();
        let stats_t = loop_stats.clone();
        let jh = thread::spawn(move || {
            let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                run_periodic_with_clock(Duration::from_millis(20), &stop_t, &stats_t, Some(clock_t.as_ref()), |_| {
                    clock_t.advance_ns(25_000_000);
                    panic!("simulated legacy release panic");
                });
            }));
        });
        // Unblock 1 release; thread exits via panic unwind rather than parking again.
        {
            let mut s = clock.state.lock().unwrap();
            s.lateness_queue_ns.push_back(0);
            s.remaining_releases = 1;
            clock.cv.notify_all();
        }
        jh.join().unwrap();
        assert_eq!(loop_stats.skipped_releases.load(Ordering::Acquire), 1);
        assert_eq!(loop_stats.releases_over_20ms.load(Ordering::Acquire), 1);
        assert_eq!(loop_stats.releases_total.load(Ordering::Acquire), 1);
        assert!(!loop_stats.is_loop_active());
    }

    // (d) Executor: 25 ms final overrun -> 1 skip on shutdown; clean exit -> 0 skips; odd stats_seq terminates!
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let clock = LatenessClock::new();
        let handle = sup
            .start_executor_with_hooks(
                ExecutorConfig::for_tests(Duration::from_millis(20)),
                PersistTestHooks {
                    clock: Some(clock.clone()),
                    post_publish_delay_once: Some(Arc::new(Mutex::new(Some(Duration::from_millis(25))))),
                    ..Default::default()
                },
            )
            .await
            .unwrap();
        clock.run_schedule(&[0]);
        handle.stop_and_join();
        assert_eq!(handle.stats().skipped_releases.load(Ordering::Acquire), 1);
        assert_eq!(handle.stats().releases_over_20ms.load(Ordering::Acquire), 1);
        assert_eq!(handle.stats().releases_total.load(Ordering::Acquire), 1);

        // Bounded termination when monotonic stats_seq is held odd.
        handle.stats().set_odd_stats_seq_for_test(true);
        let snap = sup.tick_stats();
        handle.stats().set_odd_stats_seq_for_test(false);
        assert!(snap.stats_snapshot_lapped);
    }

    // (d2) Executor: clean release followed by late stop wake past next_ns -> 0 false-positive skips!
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let clock = LatenessClock::new();
        let handle = sup
            .start_executor_with_hooks(
                ExecutorConfig::for_tests(Duration::from_millis(20)),
                PersistTestHooks {
                    clock: Some(clock.clone()),
                    ..Default::default()
                },
            )
            .await
            .unwrap();
        clock.run_schedule(&[0]);
        clock.advance_ns(30_000_000);
        handle.stop_and_join();
        assert_eq!(handle.stats().skipped_releases.load(Ordering::Acquire), 0);
        assert_eq!(handle.stats().releases_total.load(Ordering::Acquire), 1);
    }

    // (e) Executor: first-release panic with 25 ms overrun accounts skips & duration before StopExecutor exit.
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let clock = LatenessClock::new();
        let handle = sup
            .start_executor_with_hooks(
                ExecutorConfig::for_tests(Duration::from_millis(20)),
                PersistTestHooks {
                    clock: Some(clock.clone()),
                    panic_on_release: Some(1),
                    post_publish_delay_once: Some(Arc::new(Mutex::new(Some(Duration::from_millis(25))))),
                    ..Default::default()
                },
            )
            .await
            .unwrap();
        {
            let mut s = clock.state.lock().unwrap();
            s.lateness_queue_ns.push_back(0);
            s.remaining_releases = 1;
            clock.cv.notify_all();
        }
        let deadline = Instant::now() + Duration::from_secs(2);
        while !handle.is_exited() && Instant::now() < deadline {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        assert!(handle.is_exited());
        assert_eq!(handle.stats().skipped_releases.load(Ordering::Acquire), 1);
        assert_eq!(handle.stats().releases_over_20ms.load(Ordering::Acquire), 1);
        assert_eq!(handle.stats().releases_total.load(Ordering::Acquire), 1);
        handle.stop_and_join();
    }

    // (f) Mark requested DURING a 25 ms final release (20 ms period), then stop (BOTH owners):
    // final overrun skip must be attributed to the pre-marker release, NEVER to the 0-release marker!
    // Plus (g) stop-vs-mark overlap exposes terminal `"inactive"` instead of hanging `"pending"`.
    {
        // Legacy owner:
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let clock = LatenessClock::new();
        let stop = Arc::new(AtomicBool::new(false));
        let loop_stats = Arc::new(ExecutorStats::default());
        sup.attach_legacy_loop_stats(loop_stats.clone());
        let (entered_tx, entered_rx) = std::sync::mpsc::channel();
        let (resume_tx, resume_rx) = std::sync::mpsc::channel();
        let rt = tokio::runtime::Handle::current();
        let sup_t = sup.clone();
        let clock_t = clock.clone();
        let stop_t = stop.clone();
        let stats_t = loop_stats.clone();
        let jh = thread::spawn(move || {
            run_periodic_with_clock(Duration::from_millis(20), &stop_t, &stats_t, Some(clock_t.as_ref()), |dt| {
                rt.block_on(sup_t.control_tick(dt));
                clock_t.advance_ns(25_000_000);
                let _ = entered_tx.send(());
                let _ = resume_rx.recv();
            });
        });
        {
            let mut s = clock.state.lock().unwrap();
            s.lateness_queue_ns.push_back(0);
            s.remaining_releases = 1;
            clock.cv.notify_all();
        }
        entered_rx.recv_timeout(Duration::from_secs(2)).unwrap();
        // Request marker WHILE this final 25 ms release is in flight, then arm post-stats tail too.
        let marker_id = sup.mark_tick_baseline().unwrap();
        assert_eq!(marker_id, 1);
        loop_stats.post_stats_delay_once_ns.store(2_000_000, Ordering::Release);
        resume_tx.send(()).unwrap();
        // Wait for loop to park after release #1, then stop cleanly.
        {
            let mut s = clock.state.lock().unwrap();
            let deadline = Instant::now() + Duration::from_secs(2);
            while s.waiting_for_ns.is_none() && Instant::now() < deadline {
                let (next, _) = clock.cv.wait_timeout(s, Duration::from_millis(20)).unwrap();
                s = next;
            }
            assert!(s.waiting_for_ns.is_some());
        }
        stop.store(true, Ordering::Release);
        clock.wake();
        jh.join().unwrap();

        let snap = sup.tick_stats();
        assert_eq!(snap.releases_total, 1);
        assert_eq!(snap.skipped_releases, 1);
        assert_eq!(snap.releases_over_20ms, 1);
        let sm = snap.since_marker.expect("active marker");
        assert_eq!(sm.status, "active");
        assert_eq!(sm.releases_total_delta, Some(0));
        assert_eq!(sm.skipped_releases_delta, Some(0));
        assert_eq!(sm.releases_over_20ms_delta, Some(0));
        assert_eq!(sm.release_us_max, Some(0));

        // Stop-vs-mark overlap: a request published right as loop exits exposes terminal `"inactive"`.
        let pending_after_stop = loop_stats.request_marker();
        let snap_inactive = sup.tick_stats();
        let sm_inactive = snap_inactive.since_marker.expect("inactive marker");
        assert_eq!(sm_inactive.marker_id, pending_after_stop);
        assert_eq!(sm_inactive.status, "inactive");
    }

    {
        // Executor owner: request marker before/during a 25 ms final release, then stop.
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let clock = LatenessClock::new();
        let handle = sup
            .start_executor_with_hooks(
                ExecutorConfig::for_tests(Duration::from_millis(20)),
                PersistTestHooks {
                    clock: Some(clock.clone()),
                    post_publish_delay_once: Some(Arc::new(Mutex::new(Some(Duration::from_millis(25))))),
                    ..Default::default()
                },
            )
            .await
            .unwrap();
        // Wait until executor is parked before release #1, then issue mark + run 1 overrunning release + stop.
        {
            let mut s = clock.state.lock().unwrap();
            let deadline = Instant::now() + Duration::from_secs(2);
            while s.waiting_for_ns.is_none() && Instant::now() < deadline {
                let (next, _) = clock.cv.wait_timeout(s, Duration::from_millis(20)).unwrap();
                s = next;
            }
            assert!(s.waiting_for_ns.is_some());
        }
        assert_eq!(sup.mark_tick_baseline().unwrap(), 1);
        handle.stats().post_stats_delay_once_ns.store(2_000_000, Ordering::Release);
        clock.run_schedule(&[0]);
        handle.stop_and_join();

        let snap = sup.tick_stats();
        assert_eq!(snap.releases_total, 1);
        assert_eq!(snap.skipped_releases, 1);
        assert_eq!(snap.releases_over_20ms, 1);
        let sm = snap.since_marker.expect("active marker");
        assert_eq!(sm.status, "active");
        assert_eq!(sm.releases_total_delta, Some(0));
        assert_eq!(sm.skipped_releases_delta, Some(0));
        assert_eq!(sm.releases_over_20ms_delta, Some(0));
        assert_eq!(sm.release_us_max, Some(0));

        // Stop-vs-mark overlap on executor exposes terminal `"inactive"`.
        let pending_after_stop = handle.stats().request_marker();
        let snap_inactive = sup.tick_stats();
        let sm_inactive = snap_inactive.since_marker.expect("inactive marker");
        assert_eq!(sm_inactive.marker_id, pending_after_stop);
        assert_eq!(sm_inactive.status, "inactive");
    }
}

// ============================================================================
// 8. Legacy lock-free `tick_stats` under concurrent HTTP reader flood & paused reader
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn legacy_lock_free_stats_under_concurrent_http_readers_never_blocks_control_thread() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    sup.set_lease_enabled(false).await;
    let stop = Arc::new(AtomicBool::new(false));
    let loop_stats = Arc::new(ExecutorStats::default());
    sup.attach_legacy_loop_stats(loop_stats.clone());

    let rt = tokio::runtime::Handle::current();
    let sup_t = sup.clone();
    let stop_t = stop.clone();
    let stats_t = loop_stats.clone();
    let control_jh = thread::Builder::new()
        .name("armd-control".into())
        .spawn(move || {
            armd::rt_core::executor::run_periodic(
                Duration::from_millis(1),
                &stop_t,
                &stats_t,
                |dt| {
                    rt.block_on(sup_t.control_tick(dt));
                },
            );
        })
        .unwrap();

    // Causal non-blocking proof: freeze a reader thread mid-snapshot inside `window_with_hook`
    // and assert the live legacy control thread publishes >= 20 additional releases BEFORE
    // the paused reader is allowed to resume!
    let wait_initial = Instant::now() + Duration::from_secs(2);
    while loop_stats.releases_total.load(Ordering::Acquire) < 10 && Instant::now() < wait_initial {
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
    let (paused_tx, paused_rx) = std::sync::mpsc::channel();
    let (resume_tx, resume_rx) = std::sync::mpsc::channel();
    let ring = Arc::clone(sup.tick_stats_ring_for_test());
    let paused_reader_jh = thread::spawn(move || {
        let mut paused_once = false;
        let _ = ring.window_with_hook(|slot_i| {
            if slot_i == 1 && !paused_once {
                paused_once = true;
                let _ = paused_tx.send(());
                let _ = resume_rx.recv();
            }
        });
    });
    paused_rx.recv_timeout(Duration::from_secs(2)).expect("reader paused mid-snapshot");
    let before_while_paused = loop_stats.releases_total.load(Ordering::Acquire);
    let pause_deadline = Instant::now() + Duration::from_secs(2);
    while loop_stats.releases_total.load(Ordering::Acquire) < before_while_paused + 20
        && Instant::now() < pause_deadline
    {
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
    let after_while_paused = loop_stats.releases_total.load(Ordering::Acquire);
    assert!(
        after_while_paused >= before_while_paused + 20,
        "legacy control thread must keep publishing while reader is frozen mid-snapshot ({before_while_paused} -> {after_while_paused})"
    );
    resume_tx.send(()).unwrap();
    paused_reader_jh.join().unwrap();

    let addr = serve(sup.clone()).await;
    let url = format!("http://{addr}/api/v1/diagnostics/tick");
    let readers_stop = Arc::new(AtomicBool::new(false));
    let reads_ok = Arc::new(std::sync::atomic::AtomicU64::new(0));
    let mut reader_handles = Vec::new();
    for _ in 0..4 {
        let url_r = url.clone();
        let stop_r = readers_stop.clone();
        let ok_r = reads_ok.clone();
        reader_handles.push(tokio::spawn(async move {
            let client = reqwest::Client::new();
            while !stop_r.load(Ordering::Acquire) {
                let body: serde_json::Value = client
                    .get(&url_r)
                    .send()
                    .await
                    .unwrap()
                    .json()
                    .await
                    .unwrap();
                assert_eq!(body["owner_mode"], "legacy");
                assert_eq!(body["stats_wait_us"]["max"], 0);
                ok_r.fetch_add(1, Ordering::Relaxed);
            }
        }));
    }

    // Let control loop execute >= 150 releases while 4 HTTP readers continuously hammer /diagnostics/tick.
    let deadline = Instant::now() + Duration::from_secs(5);
    while (loop_stats.releases_total.load(Ordering::Acquire) < 150
        || reads_ok.load(Ordering::Relaxed) < 40)
        && Instant::now() < deadline
    {
        tokio::time::sleep(Duration::from_millis(5)).await;
    }

    readers_stop.store(true, Ordering::Release);
    for h in reader_handles {
        h.await.unwrap();
    }
    stop.store(true, Ordering::Release);
    control_jh.join().unwrap();

    let snap = sup.tick_stats();
    assert!(snap.releases_total >= 150, "releases_total={}", snap.releases_total);
    assert!(reads_ok.load(Ordering::Relaxed) >= 40);
    // Serialization compatibility field (causal non-blocking progress is proved by the paused-reader
    // hook above where >= 20 releases publish while a reader thread is frozen mid-snapshot).
    assert_eq!(snap.stats_wait_us.max, 0, "legacy stats_wait_us compatibility field must stay 0");
    assert_eq!(snap.stats_writer_drops, 0, "single legacy control thread must have 0 writer drops");
    assert!(!snap.stats_snapshot_lapped);
}

// ============================================================================
// 9. `analyze.py`: INVALID rules, contradiction/rollover/MIT/alloc checks, & `load.py` boundaries
// ============================================================================

#[test]
fn analyze_py_invalid_rules_rollover_breach_and_clean_pass() {
    let dir = tempfile::tempdir().unwrap();
    let devstat_path = dir.path().join("devstat.txt");
    std::fs::write(
        &devstat_path,
        "=== META ===\nrun_id=q1-fix\n=== ONCE_START ===\n--- armd_version ---\nARMD_ALLOC_TIMING=1\n=== ONCE_END ===\n",
    )
    .unwrap();

    let make_clean = |ticks: u64, elapsed_s: f64| -> serde_json::Value {
        let mut t = base_tick_json(20_100, 1_200);
        t["instance_id"] = serde_json::json!("armd-inst-1");
        t["boot_id"] = serde_json::json!("boot-uuid-1111");
        t["process_start_unix_ns"] = serde_json::json!(1_700_000_000_000_000_000u64);
        t["owner_mode"] = serde_json::json!("executor");
        t["loop_active"] = serde_json::json!(true);
        t["stats_snapshot_lapped"] = serde_json::json!(false);
        t["stats_writer_drops"] = serde_json::json!(0);
        t["in_flight_release_us"] = serde_json::json!(0);
        t["since_boot_ticks"] = serde_json::json!(ticks);
        t["release_us"] = serde_json::json!({ "p50": 550, "p99": 950, "max": 1_400 });
        t["releases_started"] = serde_json::json!(ticks);
        t["releases_total"] = serde_json::json!(ticks);
        t["release_us_max_ever"] = serde_json::json!(1_400);
        t["releases_over_10ms"] = serde_json::json!(0);
        t["releases_over_15ms"] = serde_json::json!(0);
        t["releases_over_20ms"] = serde_json::json!(0);
        t["wake_late_max_ever"] = serde_json::json!(400);
        t["skipped_releases"] = serde_json::json!(0);
        t["diag_dropped"] = serde_json::json!(0);
        t["wake_lateness_us"] = serde_json::json!({ "p50": 100, "p99": 300, "max": 400 });
        t["wake_lateness_over_1ms"] = serde_json::json!(0);
        t["wake_lateness_over_5ms"] = serde_json::json!(0);
        t["wake_lateness_over_10ms"] = serde_json::json!(0);
        t["control_thread_lock_attempts"] = serde_json::json!(0);
        t["control_thread_alloc_ops"] = serde_json::json!(0);
        t["enqueue_to_wire_us"] = serde_json::json!({ "p50": 80, "p99": 150, "max": 200 });
        serde_json::json!({
            "type": "sample",
            "phase": "baseline",
            "elapsed_s": elapsed_s,
            "tick": t,
            "counters": { "obs_ok": if elapsed_s > 0.0 { 25 } else { 0 } },
            "safety_state": "Idle"
        })
    };

    let make_idle_start = |ticks: u64, elapsed_s: f64| -> serde_json::Value {
        let mut s = make_clean(ticks, elapsed_s);
        s["phase"] = serde_json::json!("idle_tail_start");
        s["counters"] = serde_json::json!({ "obs_ok": 25 });
        s
    };

    let write_and_run_raw = |name: &str, lines: &[serde_json::Value], include_summary: bool| -> String {
        let p = dir.path().join(name);
        let mut buf = format!(
            "{}\n",
            serde_json::json!({ "type": "meta", "run_id": name, "enabled_loads": { "obs_poll": true } })
        );
        for l in lines {
            let mut rec = l.clone();
            if let Some(obj) = rec.as_object_mut() {
                obj.entry("run_id").or_insert_with(|| serde_json::json!(name));
            }
            buf.push_str(&format!("{rec}\n"));
        }
        if include_summary {
            buf.push_str(&format!(
                "{}\n",
                serde_json::json!({
                    "type": "summary",
                    "run_id": name,
                    "stop_reason": "completed",
                    "fault": null,
                    "safety_state": "Idle",
                    "terminal_safety_verified": true,
                    "counters": { "obs_ok": 25 }
                })
            ));
        }
        std::fs::write(&p, buf).unwrap();
        run_analyze(&p, Some(&devstat_path))
    };

    let write_and_run = |name: &str, lines: &[serde_json::Value]| -> String {
        let mut owned: Vec<serde_json::Value> = lines.to_vec();
        if let Some(last) = owned.last_mut() {
            if last.get("type").and_then(|v| v.as_str()) == Some("sample")
                && last.get("tick").is_some_and(|v| v.is_object())
            {
                last["phase"] = serde_json::json!("final");
            }
        }
        write_and_run_raw(name, &owned, true)
    };

    // 1. Clean full-coverage run (baseline -> idle_tail_start -> settled final >= 1.0 s later) -> PASS
    let clean_rep = write_and_run(
        "clean.jsonl",
        &[make_clean(100, 0.0), make_idle_start(250, 3.0), make_clean(300, 4.2)],
    );
    assert!(clean_rep.contains("**Overall Verdict: PASS**"), "{clean_rep}");

    // 2. Counter reset (releases_total decreased 300 -> 50) -> INVALID
    let reset_rep = write_and_run("reset.jsonl", &[make_clean(300, 0.0), make_clean(50, 4.0)]);
    assert!(reset_rep.contains("**Overall Verdict: INVALID"), "{reset_rep}");
    assert!(reset_rep.contains("counter decreased mid-run (restart/reset): releases_total"), "{reset_rep}");

    // 3. Same/higher counters across a daemon restart (instance_id changed) -> INVALID
    let mut restart_s1 = make_clean(250, 4.0);
    restart_s1["tick"]["instance_id"] = serde_json::json!("armd-inst-2");
    let restart_rep = write_and_run("restart_id.jsonl", &[make_clean(100, 0.0), restart_s1]);
    assert!(restart_rep.contains("**Overall Verdict: INVALID"), "{restart_rep}");
    assert!(restart_rep.contains("instance_id changed"), "{restart_rep}");

    // 3b. Kernel reboot (`boot_id` changed) or `panic = "abort"` process restart (`process_start_unix_ns` changed),
    //     or missing/placeholder/boolean/zero boot identity -> INVALID!
    let mut reboot_s1 = make_clean(250, 4.0);
    reboot_s1["tick"]["boot_id"] = serde_json::json!("boot-uuid-2222");
    let reboot_rep = write_and_run("reboot_id.jsonl", &[make_clean(100, 0.0), reboot_s1]);
    assert!(reboot_rep.contains("**Overall Verdict: INVALID"), "{reboot_rep}");
    assert!(reboot_rep.contains("boot_id changed"), "{reboot_rep}");

    let mut proc_restart_s1 = make_clean(250, 4.0);
    proc_restart_s1["tick"]["process_start_unix_ns"] = serde_json::json!(1_700_000_005_000_000_000u64);
    let proc_restart_rep = write_and_run("proc_restart.jsonl", &[make_clean(100, 0.0), proc_restart_s1]);
    assert!(proc_restart_rep.contains("**Overall Verdict: INVALID"), "{proc_restart_rep}");
    assert!(proc_restart_rep.contains("process_start_unix_ns changed"), "{proc_restart_rep}");

    let mut placeholder_boot = make_clean(250, 4.0);
    placeholder_boot["tick"]["boot_id"] = serde_json::json!("unknown-boot");
    let placeholder_rep = write_and_run("placeholder_boot.jsonl", &[make_clean(100, 0.0), placeholder_boot]);
    assert!(placeholder_rep.contains("**Overall Verdict: INVALID"), "{placeholder_rep}");
    assert!(placeholder_rep.contains("boot_id (1/2)"), "{placeholder_rep}");

    let mut bool_start = make_clean(250, 4.0);
    bool_start["tick"]["process_start_unix_ns"] = serde_json::json!(true);
    let bool_start_rep = write_and_run("bool_start.jsonl", &[make_clean(100, 0.0), bool_start]);
    assert!(bool_start_rep.contains("**Overall Verdict: INVALID"), "{bool_start_rep}");
    assert!(bool_start_rep.contains("process_start_unix_ns (1/2)"), "{bool_start_rep}");

    let mut zero_start = make_clean(250, 4.0);
    zero_start["tick"]["process_start_unix_ns"] = serde_json::json!(0);
    let zero_start_rep = write_and_run("zero_start.jsonl", &[make_clean(100, 0.0), zero_start]);
    assert!(zero_start_rep.contains("**Overall Verdict: INVALID"), "{zero_start_rep}");
    assert!(zero_start_rep.contains("process_start_unix_ns (1/2)"), "{zero_start_rep}");

    // 4. Failed poll (tick: null + tick_error) -> INVALID
    let failed_poll_line = serde_json::json!({
        "type": "sample", "phase": "full_load", "elapsed_s": 2.0,
        "tick": null, "tick_error": "URLError: timed out"
    });
    let poll_fail_rep = write_and_run(
        "failed_poll.jsonl",
        &[make_clean(100, 0.0), failed_poll_line.clone(), make_clean(300, 4.0)],
    );
    assert!(poll_fail_rep.contains("**Overall Verdict: INVALID"), "{poll_fail_rep}");
    assert!(poll_fail_rep.contains("failed poll(s): 1"), "{poll_fail_rep}");

    // 5. All-failed JSONL (0 valid samples) -> INVALID (must not crash!)
    let all_fail_rep = write_and_run("all_failed.jsonl", &[failed_poll_line]);
    assert!(all_fail_rep.contains("**Overall Verdict: INVALID (all 1 poll(s) failed"), "{all_fail_rep}");

    // 6. Single-sample run -> INVALID (cannot compute delta)
    let one_rep = write_and_run("single.jsonl", &[make_clean(100, 0.0)]);
    assert!(one_rep.contains("**Overall Verdict: INVALID"), "{one_rep}");
    assert!(one_rep.contains("fewer than 2 tick samples"), "{one_rep}");

    // 7. Zero releases completed (releases_total: 100 -> 100) -> INVALID
    let zero_rep = write_and_run("zero_rel.jsonl", &[make_clean(100, 0.0), make_clean(100, 4.0)]);
    assert!(zero_rep.contains("**Overall Verdict: INVALID"), "{zero_rep}");
    assert!(zero_rep.contains("zero releases completed"), "{zero_rep}");

    // 8. Missing or malformed required fields (release_us: {}, missing wait_us) -> INVALID (never 0!)
    let mut bad_rel = make_clean(300, 4.0);
    bad_rel["tick"]["release_us"] = serde_json::json!({});
    let bad_rel_rep = write_and_run("malformed_rel.jsonl", &[make_clean(100, 0.0), bad_rel]);
    assert!(bad_rel_rep.contains("**Overall Verdict: INVALID"), "{bad_rel_rep}");
    assert!(bad_rel_rep.contains("release_us (1/2)"), "{bad_rel_rep}");

    let mut no_wait = make_clean(300, 4.0);
    no_wait["tick"].as_object_mut().unwrap().remove("wait_us");
    let no_wait_rep = write_and_run("missing_wait.jsonl", &[make_clean(100, 0.0), no_wait]);
    assert!(no_wait_rep.contains("**Overall Verdict: INVALID"), "{no_wait_rep}");
    assert!(no_wait_rep.contains("wait_us.max (1/2)"), "{no_wait_rep}");

    // 9. Sample gap > 1000 ticks with missing monotonic counters -> INVALID mentioning sample gap!
    let mut gap0 = make_clean(100, 0.0);
    let mut gap1 = make_clean(2500, 48.0);
    for key in [
        "releases_started",
        "releases_total",
        "release_us_max_ever",
        "releases_over_10ms",
        "releases_over_15ms",
        "releases_over_20ms",
        "wake_late_max_ever",
    ] {
        gap0["tick"].as_object_mut().unwrap().remove(key);
        gap1["tick"].as_object_mut().unwrap().remove(key);
    }
    let gap_rep = write_and_run("window_gap.jsonl", &[gap0, gap1]);
    assert!(gap_rep.contains("**Overall Verdict: INVALID"), "{gap_rep}");
    assert!(gap_rep.contains("sample gap (2400 ticks > window 1000) with missing monotonic counters"), "{gap_rep}");

    // 10. Owner mode changed mid-run -> INVALID
    let mut sw1 = make_clean(300, 4.0);
    sw1["tick"]["owner_mode"] = serde_json::json!("legacy");
    let sw_rep = write_and_run("owner_switch.jsonl", &[make_clean(100, 0.0), sw1]);
    assert!(sw_rep.contains("**Overall Verdict: INVALID"), "{sw_rep}");
    assert!(sw_rep.contains("owner_mode changed mid-run"), "{sw_rep}");

    // 11. Lock-free lapped snapshot -> INVALID
    let mut lap1 = make_clean(300, 4.0);
    lap1["tick"]["stats_snapshot_lapped"] = serde_json::json!(true);
    let lap_rep = write_and_run("lapped.jsonl", &[make_clean(100, 0.0), lap1]);
    assert!(lap_rep.contains("**Overall Verdict: INVALID"), "{lap_rep}");
    assert!(lap_rep.contains("stats_snapshot_lapped=true"), "{lap_rep}");

    // 12. Pre-run boot spike in samples[0] (releases_over_20ms: 1 -> 1) does NOT falsely FAIL clean run!
    let mut pre0 = make_clean(100, 0.0);
    pre0["tick"]["release_us"]["max"] = serde_json::json!(25_000);
    pre0["tick"]["release_us_max_ever"] = serde_json::json!(25_000);
    pre0["tick"]["releases_over_20ms"] = serde_json::json!(1);
    let mut pre_mid = make_idle_start(250, 3.0);
    pre_mid["tick"]["release_us"]["max"] = serde_json::json!(25_000);
    pre_mid["tick"]["release_us_max_ever"] = serde_json::json!(25_000);
    pre_mid["tick"]["releases_over_20ms"] = serde_json::json!(1);
    let mut pre1 = make_clean(300, 4.2);
    pre1["tick"]["release_us"]["max"] = serde_json::json!(25_000);
    pre1["tick"]["release_us_max_ever"] = serde_json::json!(25_000);
    pre1["tick"]["releases_over_20ms"] = serde_json::json!(1);
    let pre_rep = write_and_run("prerun_spike.jsonl", &[pre0, pre_mid, pre1]);
    assert!(pre_rep.contains("**Overall Verdict: PASS**"), "{pre_rep}");

    // 13. In-run 25 ms spike that aged out past 1000-tick window (releases_over_20ms: 0 -> 1) -> FAIL!
    let mut roll1 = make_clean(1600, 30.0);
    roll1["tick"]["release_us"]["max"] = serde_json::json!(1_500);
    roll1["tick"]["release_us_max_ever"] = serde_json::json!(25_000);
    roll1["tick"]["releases_over_20ms"] = serde_json::json!(1);
    let roll_rep = write_and_run("rollover_breach.jsonl", &[make_clean(100, 0.0), roll1]);
    assert!(roll_rep.contains("**Overall Verdict: FAIL**"), "{roll_rep}");

    // 14. Blocker #3: direct 25 ms in-run evidence (HWM 1_400 -> 25_000 + ticks_ago=0) with
    //     releases_over_20ms 0->0 MUST FAIL (never PASS)!
    let mut contra1 = make_clean(300, 4.0);
    contra1["tick"]["release_us"]["max"] = serde_json::json!(25_000);
    contra1["tick"]["release_us_max_ever"] = serde_json::json!(25_000);
    contra1["tick"]["max_release"] = serde_json::json!({ "release_us": 25_000, "ticks_ago": 0 });
    let contra_rep = write_and_run("contra_breach.jsonl", &[make_clean(100, 0.0), contra1]);
    assert!(contra_rep.contains("**Overall Verdict: FAIL**"), "{contra_rep}");

    // 15. Blocker #3: ambiguous contradiction (both samples report 25 ms max with releases_over_20ms=0) -> INVALID!
    let mut amb0 = make_clean(100, 0.0);
    amb0["tick"]["release_us"]["max"] = serde_json::json!(25_000);
    amb0["tick"]["release_us_max_ever"] = serde_json::json!(25_000);
    let mut amb1 = make_clean(300, 4.0);
    amb1["tick"]["release_us"]["max"] = serde_json::json!(25_000);
    amb1["tick"]["release_us_max_ever"] = serde_json::json!(25_000);
    let amb_rep = write_and_run("contra_ambig.jsonl", &[amb0, amb1]);
    assert!(amb_rep.contains("**Overall Verdict: INVALID"), "{amb_rep}");
    assert!(amb_rep.contains("contradictory telemetry"), "{amb_rep}");

    // 16. Blocker #4: truncated JSONL (no summary or non-final tail), terminal uncompleted release,
    //     and live in-flight stall >= 20 ms!
    let mut fin_ok = make_clean(300, 4.0);
    fin_ok["phase"] = serde_json::json!("final");
    let no_sum_rep = write_and_run_raw("no_summary.jsonl", &[make_clean(100, 0.0), fin_ok], false);
    assert!(no_sum_rep.contains("**Overall Verdict: INVALID"), "{no_sum_rep}");
    assert!(no_sum_rep.contains("missing terminal summary record"), "{no_sum_rep}");

    let non_final_rep = write_and_run_raw("non_final.jsonl", &[make_clean(100, 0.0), make_clean(300, 4.0)], true);
    assert!(non_final_rep.contains("**Overall Verdict: INVALID"), "{non_final_rep}");
    assert!(non_final_rep.contains("expected 'final'"), "{non_final_rep}");

    let mut uncomp1 = make_clean(300, 4.0);
    uncomp1["tick"]["releases_started"] = serde_json::json!(301);
    let uncomp_rep = write_and_run("uncompleted_inflight.jsonl", &[make_clean(100, 0.0), uncomp1]);
    assert!(uncomp_rep.contains("**Overall Verdict: INVALID"), "{uncomp_rep}");
    assert!(uncomp_rep.contains("uncompleted in-flight release"), "{uncomp_rep}");

    let mut stall1 = make_clean(300, 4.0);
    stall1["tick"]["releases_started"] = serde_json::json!(301);
    stall1["tick"]["in_flight_release_us"] = serde_json::json!(25_000);
    let stall_rep = write_and_run("stalled_inflight.jsonl", &[make_clean(100, 0.0), stall1]);
    assert!(stall_rep.contains("**Overall Verdict: FAIL**"), "{stall_rep}");

    // 17. Major #1: malformed coherence metadata ("true" string, missing stats_writer_drops, unknown owner) -> INVALID!
    let mut str_lap = make_clean(300, 4.0);
    str_lap["tick"]["stats_snapshot_lapped"] = serde_json::json!("true");
    let str_lap_rep = write_and_run("str_lapped.jsonl", &[make_clean(100, 0.0), str_lap]);
    assert!(str_lap_rep.contains("**Overall Verdict: INVALID"), "{str_lap_rep}");
    assert!(str_lap_rep.contains("stats_snapshot_lapped (1/2)"), "{str_lap_rep}");

    let mut no_wd = make_clean(300, 4.0);
    no_wd["tick"].as_object_mut().unwrap().remove("stats_writer_drops");
    let no_wd_rep = write_and_run("no_writer_drops.jsonl", &[make_clean(100, 0.0), no_wd]);
    assert!(no_wd_rep.contains("**Overall Verdict: INVALID"), "{no_wd_rep}");
    assert!(no_wd_rep.contains("stats_writer_drops (1/2)"), "{no_wd_rep}");

    let mut bad_owner = make_clean(300, 4.0);
    bad_owner["tick"]["owner_mode"] = serde_json::json!("transferring");
    let bad_owner_rep = write_and_run("bad_owner.jsonl", &[make_clean(100, 0.0), bad_owner]);
    assert!(bad_owner_rep.contains("**Overall Verdict: INVALID"), "{bad_owner_rep}");
    assert!(bad_owner_rep.contains("owner_mode (1/2)"), "{bad_owner_rep}");

    // 18. Major #2: rolled wake peak (100 -> 25_000 on wake_late_max_ever with 100 us windows) -> MIT FAIL!
    //     And pre-run high wake_late_max_ever across >1000-tick gap without since_marker -> INCOMPLETE!
    let mut wake_roll0 = make_clean(100, 0.0);
    wake_roll0["tick"]["wake_lateness_us"] = serde_json::json!({ "p50": 50, "p99": 80, "max": 100 });
    wake_roll0["tick"]["wake_late_max_ever"] = serde_json::json!(100);
    let mut wake_roll1 = make_clean(1600, 30.0);
    wake_roll1["tick"]["wake_lateness_us"] = serde_json::json!({ "p50": 50, "p99": 80, "max": 100 });
    wake_roll1["tick"]["wake_late_max_ever"] = serde_json::json!(25_000);
    let wake_roll_rep = write_and_run("rolled_wake.jsonl", &[wake_roll0, wake_roll1]);
    assert!(wake_roll_rep.contains("**Overall Verdict: FAIL**"), "{wake_roll_rep}");

    let mut wake_mask0 = make_clean(100, 0.0);
    wake_mask0["tick"]["wake_late_max_ever"] = serde_json::json!(20_000);
    let mut wake_mask_mid = make_idle_start(1600, 30.0);
    wake_mask_mid["tick"]["wake_late_max_ever"] = serde_json::json!(20_000);
    let mut wake_mask1 = make_clean(1650, 31.2);
    wake_mask1["tick"]["wake_late_max_ever"] = serde_json::json!(20_000);
    let wake_mask_rep = write_and_run("masked_wake.jsonl", &[wake_mask0, wake_mask_mid, wake_mask1]);
    assert!(wake_mask_rep.contains("**Overall Verdict: INCOMPLETE"), "{wake_mask_rep}");
    assert!(wake_mask_rep.contains("masks in-run wake peak"), "{wake_mask_rep}");

    // 19. Major #4: missing/proxy-only allocation evidence MUST NEVER produce PASS!
    let mut no_alloc0 = make_clean(100, 0.0);
    let mut no_alloc_mid = make_idle_start(250, 3.0);
    let mut no_alloc1 = make_clean(300, 4.2);
    for sample in [&mut no_alloc0, &mut no_alloc_mid, &mut no_alloc1] {
        let obj = sample["tick"].as_object_mut().unwrap();
        obj.remove("control_thread_alloc_ops");
        obj.remove("max_tick");
        obj.remove("max_lock");
        obj.remove("max_release");
    }
    let no_alloc_rep = write_and_run("no_alloc_evidence.jsonl", &[no_alloc0, no_alloc_mid, no_alloc1]);
    assert!(no_alloc_rep.contains("**Overall Verdict: INCOMPLETE"), "{no_alloc_rep}");
    assert!(no_alloc_rep.contains("INCOMPLETE (missing/malformed allocation evidence)"), "{no_alloc_rep}");

    let mut proxy_alloc0 = make_clean(100, 0.0);
    let mut proxy_alloc_mid = make_idle_start(250, 3.0);
    let mut proxy_alloc1 = make_clean(300, 4.2);
    proxy_alloc0["tick"].as_object_mut().unwrap().remove("control_thread_alloc_ops");
    proxy_alloc_mid["tick"].as_object_mut().unwrap().remove("control_thread_alloc_ops");
    proxy_alloc1["tick"].as_object_mut().unwrap().remove("control_thread_alloc_ops");
    let proxy_alloc_rep = write_and_run("proxy_alloc_only.jsonl", &[proxy_alloc0, proxy_alloc_mid, proxy_alloc1]);
    assert!(proxy_alloc_rep.contains("**Overall Verdict: INCOMPLETE"), "{proxy_alloc_rep}");
    assert!(proxy_alloc_rep.contains("PASS (proxy)"), "{proxy_alloc_rep}");

    let mut alloc_breach1 = make_clean(300, 4.0);
    alloc_breach1["tick"]["control_thread_alloc_ops"] = serde_json::json!(2);
    let alloc_breach_rep = write_and_run("alloc_breach.jsonl", &[make_clean(100, 0.0), alloc_breach1]);
    assert!(alloc_breach_rep.contains("**Overall Verdict: FAIL**"), "{alloc_breach_rep}");
}

// ============================================================================
// 10. Post-stats tail stalls, executor sub-release overrun catch-up, in-flight stall, & load.py boundaries
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn tail_stall_sub_release_overrun_in_flight_stall_and_load_py_boundaries() {
    // (a) Legacy: 25 ms stall STRICTLY AFTER ring.record_staged & commit_release_outcome (right before sleep)
    //     MUST still appear in rolling `release_us.max`, `release_us_max_ever`, `releases_over_20ms`, and `skipped_releases`!
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let clock = LatenessClock::new();
        let stop = Arc::new(AtomicBool::new(false));
        let loop_stats = Arc::new(ExecutorStats::default());
        sup.attach_legacy_loop_stats(loop_stats.clone());
        let rt = tokio::runtime::Handle::current();
        let sup_t = sup.clone();
        let clock_t = clock.clone();
        let stop_t = stop.clone();
        let stats_t = loop_stats.clone();
        let jh = thread::spawn(move || {
            run_periodic_with_clock(Duration::from_millis(20), &stop_t, &stats_t, Some(clock_t.as_ref()), |dt| {
                rt.block_on(sup_t.control_tick(dt));
            });
        });
        loop_stats.post_stats_delay_once_ns.store(25_000_000, Ordering::Release);
        clock.run_schedule(&[0]);
        let snap = sup.tick_stats();
        assert!(snap.release_us.max >= 25_000, "rolling release_us missed post-stats tail: {snap:?}");
        assert!(snap.release_us_max_ever >= 25_000);
        assert_eq!(snap.releases_over_20ms, 1);
        assert_eq!(snap.skipped_releases, 1);
        assert_eq!(snap.releases_started, 1);
        assert_eq!(snap.releases_total, 1);

        // Continuing clean release must NOT double-count skips or thresholds.
        clock.run_schedule(&[0]);
        let snap2 = sup.tick_stats();
        assert_eq!(snap2.releases_over_20ms, 1);
        assert_eq!(snap2.skipped_releases, 1);
        assert_eq!(snap2.releases_total, 2);

        stop.store(true, Ordering::Release);
        clock.wake();
        jh.join().unwrap();
    }

    // (b) Executor: 25 ms stall STRICTLY AFTER diag_tx.push + hub.wake + commit_release_outcome + completed_releases
    //     MUST update rolling `release_us.max`, `release_us_max_ever`, `releases_over_20ms`, `skipped_releases`, with 0 allocs!
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let clock = LatenessClock::new();
        let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(20));
        cfg.alloc_timing = true;
        let handle = sup
            .start_executor_with_hooks(
                cfg,
                PersistTestHooks {
                    clock: Some(clock.clone()),
                    ..Default::default()
                },
            )
            .await
            .unwrap();
        handle.stats().post_stats_delay_once_ns.store(25_000_000, Ordering::Release);
        clock.run_schedule(&[0]);
        let snap = sup.tick_stats();
        assert!(snap.release_us.max >= 25_000, "executor rolling release_us missed post-stats tail: {snap:?}");
        assert!(snap.release_us_max_ever >= 25_000);
        assert_eq!(snap.releases_over_20ms, 1);
        assert_eq!(snap.skipped_releases, 1);
        assert_eq!(snap.releases_started, 1);
        assert_eq!(snap.releases_total, 1);
        assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);

        // Continuing loop preserves exact skip count (no catch-up skip loss, no double count).
        clock.run_schedule(&[0]);
        let snap2 = sup.tick_stats();
        assert_eq!(snap2.releases_over_20ms, 1);
        assert_eq!(snap2.skipped_releases, 1);
        assert_eq!(snap2.releases_total, 2);
        handle.stop_and_join();
    }

    // (c) Blocker #2: Executor release with 10 ms scheduled sub-release sleep + 15 ms wake work
    //     (= 25 ms wall span on a 20 ms period) followed by continuing releases -> MUST record
    //     `release_us >= 25 ms` (no sub-release wait subtraction), `releases_over_20ms == 1`, AND
    //     `skipped_releases == 1` even while the loop continues!
    {
        use armd::driver::{
            ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget, TorqueOutcome,
            TorqueProgress,
        };
        use armd::model::JointState;
        use armd::rt_core::broker::{Authority, ClientId, Submit, TransitionOp};

        struct SubReleaseOverrunDriver {
            dof: usize,
            q: Vec<f32>,
            enabled: bool,
            rx_at: Vec<Option<Instant>>,
            seqs: Vec<u64>,
            pending_target: Option<(bool, Instant)>,
            clock: Arc<LatenessClock>,
            wake_delay: Duration,
            wake_work_ns: u64,
        }
        impl ArmDriver for SubReleaseOverrunDriver {
            fn capabilities(&self) -> DriverCapabilities {
                DriverCapabilities {
                    velocity_feedback: true,
                    torque_feedback: true,
                    temperature_feedback: true,
                    torque_feedforward: false,
                    enable_feedback: true,
                    load_feedback: false,
                }
            }
            fn supports_rt_executor(&self) -> Result<(), &'static str> { Ok(()) }
            fn clock_now(&self) -> Instant { self.clock.now() }
            fn enable_state(&self) -> EnableState {
                if self.enabled { EnableState::Enabled } else { EnableState::Disabled }
            }
            fn is_feedback_valid(&self) -> bool { true }
            fn feedback_ages(&self) -> Vec<Option<Duration>> {
                let now = self.clock.now();
                self.rx_at.iter().map(|t| t.map(|t| now.saturating_duration_since(t))).collect()
            }
            fn feedback_instants(&self) -> Vec<Option<Instant>> { self.rx_at.clone() }
            fn feedback_seqs(&self) -> Vec<u64> { self.seqs.clone() }
            fn feedback_receive_instants(&self) -> Vec<Option<Instant>> { self.rx_at.clone() }
            fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
                let now = self.clock.now();
                out.clear();
                out.extend(self.rx_at.iter().map(|t| t.map(|t| now.saturating_duration_since(t))));
            }
            fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
                let now = self.clock.now();
                self.rx_at.len() == dof
                    && self.rx_at.iter().all(|t| t.is_some_and(|t| now.saturating_duration_since(t) <= max_age))
            }
            fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
                let now = self.clock.now();
                out.clear();
                out.extend(self.rx_at.iter().map(|t| t.map(|t| now.saturating_duration_since(t).as_millis() as u64)));
            }
            fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) {
                out.clear();
                out.extend_from_slice(&self.rx_at);
            }
            fn feedback_seqs_into(&self, out: &mut Vec<u64>) {
                out.clear();
                out.extend_from_slice(&self.seqs);
            }
            fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) {
                out.clear();
                out.extend_from_slice(&self.rx_at);
            }
            fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
                let mut out = Vec::with_capacity(self.dof);
                self.read_joint_states_into(&mut out)?;
                Ok(out)
            }
            fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
                let now = self.clock.now();
                out.clear();
                for i in 0..self.dof {
                    out.push(JointState { position: self.q[i], velocity: 0.0, torque: 0.0, temperature: 30.0 });
                    self.rx_at[i] = Some(now);
                    self.seqs[i] += 1;
                }
                Ok(())
            }
            fn read_joint_states_budgeted(
                &mut self,
                out: &mut Vec<JointState>,
                _deadline: Instant,
                _max_frames: u16,
            ) -> Result<(u16, bool), ArmDriverError> {
                self.read_joint_states_into(out)?;
                Ok((self.dof as u16, false))
            }
            fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
                for (i, t) in targets.iter().enumerate().take(self.dof) {
                    self.q[i] = t.position;
                }
                Ok(())
            }
            fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
                self.enabled = enable;
                Ok(())
            }
            fn begin_torque_budgeted(&mut self, enable: bool, _max_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
                let wake = self.clock.now() + self.wake_delay;
                self.pending_target = Some((enable, wake));
                Ok(TorqueProgress::Pending { wake })
            }
            fn poll_torque_budgeted(&mut self, _deadline: Instant, _max_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
                let Some((target, wake)) = self.pending_target else {
                    return Ok(TorqueProgress::Done(TorqueOutcome::Confirmed));
                };
                if self.clock.now() < wake {
                    return Ok(TorqueProgress::Pending { wake });
                }
                self.pending_target = None;
                // Simulate 15 ms work inside the sub-release wake after 10 ms scheduled sub-release wait -> 25 ms total!
                self.clock.advance_ns(self.wake_work_ns);
                self.enabled = target;
                Ok(TorqueProgress::Done(TorqueOutcome::Confirmed))
            }
            fn abort_enable(&mut self) {
                if matches!(self.pending_target, Some((true, _))) {
                    self.pending_target = None;
                }
            }
            fn torque_transition_pending(&self) -> bool {
                self.pending_target.is_some()
            }
        }

        let clock = LatenessClock::new();
        let now = clock.now();
        let drv = SubReleaseOverrunDriver {
            dof: 7,
            q: vec![0.0; 7],
            enabled: false,
            rx_at: vec![Some(now); 7],
            seqs: vec![1; 7],
            pending_target: None,
            clock: clock.clone(),
            // 30 ms confirm delay on a 20 ms period: release #2 starts TorqueOn at T+40ms (wake=T+70ms),
            // release #3 wakes at T+60ms, sleeps 10 ms to sub-release wake T+70ms, then does 15 ms work -> 25 ms!
            wake_delay: Duration::from_millis(30),
            wake_work_ns: 15_000_000,
        };
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
        sup.set_lease_enabled(false).await;
        let handle = sup
            .start_executor_with_hooks(
                ExecutorConfig::for_tests(Duration::from_millis(20)),
                PersistTestHooks {
                    clock: Some(clock.clone()),
                    ..Default::default()
                },
            )
            .await
            .unwrap();
        // Run 1 clean baseline release so epoch is published, then submit TorqueOn to trigger
        // a 10 ms scheduled sub-release wait + 15 ms wake work (= 25 ms) followed by 2 continuing releases.
        clock.run_schedule(&[0]);
        let auth = Authority {
            lease_generation: 0,
            epoch: sup.hub().latest().map_or(1, |s| s.obs.epoch_id),
            stop_generation: handle.stop_requests().current(),
            config_revision: 0,
            parent_generation: 0,
        };
        handle
            .broker()
            .lock()
            .unwrap()
            .submit(
                ClientId(1),
                1,
                auth,
                clock.now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOn),
            )
            .unwrap();
        // 4 sleep_until_ns calls: release #2 periodic wake, release #2 sub-release wake (10 ms),
        // release #3 periodic wake, release #4 periodic wake.
        clock.run_schedule(&[0, 0, 0, 0]);
        let snap = sup.tick_stats();
        assert_eq!(handle.stats().wakes.load(Ordering::Acquire), 1, "must exercise real Window-2 sub-release wake");
        assert!(snap.release_us.max >= 25_000, "sub-release wait must NOT be subtracted from release_us: {snap:?}");
        assert!(snap.release_us_max_ever >= 25_000);
        assert_eq!(snap.releases_over_20ms, 1);
        assert_eq!(snap.skipped_releases, 1, "continuing loop must account 25 ms overrun skip immediately");
        assert_eq!(snap.releases_total, 4);
        handle.stop_and_join();
    }

    // (d) Blocker #4: Live in-flight release stall (> 20 ms) is immediately visible via `releases_started > releases_total`
    //     and `in_flight_release_us >= 20_000`, AND `load.py` captures baseline strictly BEFORE workers start
    //     even with `--baseline-s 0` and writes settled `final` + `summary`!
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let stop = Arc::new(AtomicBool::new(false));
        let loop_stats = Arc::new(ExecutorStats::default());
        sup.attach_legacy_loop_stats(loop_stats.clone());
        let stall_armed = Arc::new(AtomicBool::new(true));
        let (stalled_tx, stalled_rx) = std::sync::mpsc::channel();
        let (release_tx, release_rx) = std::sync::mpsc::channel();
        let rt = tokio::runtime::Handle::current();
        let sup_t = sup.clone();
        let stop_t = stop.clone();
        let stats_t = loop_stats.clone();
        let armed_t = stall_armed.clone();
        let jh = thread::spawn(move || {
            armd::rt_core::executor::run_periodic(Duration::from_millis(5), &stop_t, &stats_t, |dt| {
                rt.block_on(sup_t.control_tick(dt));
                if armed_t.swap(false, Ordering::AcqRel) {
                    let _ = stalled_tx.send(());
                    let _ = release_rx.recv();
                }
            });
        });

        stalled_rx.recv_timeout(Duration::from_secs(2)).unwrap();
        tokio::time::sleep(Duration::from_millis(25)).await;
        let live = sup.tick_stats();
        assert!(live.releases_started > live.releases_total, "expected in-flight release: {live:?}");
        assert!(
            live.in_flight_release_us >= 20_000,
            "expected live in_flight_release_us >= 20_000, got {}",
            live.in_flight_release_us
        );
        release_tx.send(()).unwrap();

        // Run real `scripts/rt_qual/load.py` with `--baseline-s 0` against live HTTP router.
        let addr = serve(sup.clone()).await;
        let dir = tempfile::tempdir().unwrap();
        let out_jsonl = dir.path().join("load.jsonl");
        let script = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("scripts/rt_qual/load.py");
        let output = tokio::task::spawn_blocking(move || {
            Command::new("python3")
                .arg(&script)
                .args([
                    "--host",
                    "127.0.0.1",
                    "--port",
                    &addr.port().to_string(),
                    "--duration",
                    "0.4",
                    "--baseline-s",
                    "0",
                    "--cooldown-s",
                    "0",
                    "--sample-interval",
                    "0.2",
                    "--no-motion",
                    "--no-record",
                    "--no-measure",
                    "--no-ws-stream",
                    "--no-blackbox-log",
                    "--no-bulk-http",
                    "--obs-poll",
                    "--output",
                ])
                .arg(&out_jsonl)
                .output()
                .expect("run load.py")
        })
        .await
        .unwrap();
        assert!(
            output.status.success(),
            "load.py failed: stdout={} stderr={}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
        let raw = std::fs::read_to_string(dir.path().join("load.jsonl")).unwrap();
        let recs: Vec<serde_json::Value> = raw
            .lines()
            .filter(|l| !l.trim().is_empty())
            .map(|l| serde_json::from_str(l).unwrap())
            .collect();
        let sample_recs: Vec<&serde_json::Value> = recs
            .iter()
            .filter(|r| r.get("type").and_then(|v| v.as_str()) == Some("sample"))
            .collect();
        assert!(sample_recs.len() >= 3, "expected >= 3 samples (baseline, idle_tail_start, final): {raw}");
        assert_eq!(sample_recs[0]["phase"], "baseline");
        assert_eq!(
            sample_recs[0]["counters"]["obs_ok"], 0,
            "baseline sample MUST complete before any load worker fires even with --baseline-s 0"
        );
        assert_eq!(
            sample_recs[sample_recs.len() - 2]["phase"],
            "idle_tail_start",
            "load.py must persist idle_tail_start immediately before final: {raw}"
        );
        let last_sample = *sample_recs.last().unwrap();
        assert_eq!(last_sample["phase"], "final");
        assert_eq!(
            last_sample["tick"]["releases_started"],
            last_sample["tick"]["releases_total"],
            "settled final sample must cover all started releases: {last_sample}"
        );
        assert_eq!(
            last_sample["tick"]["in_flight_release_us"], 0,
            "settled final sample must have in_flight_release_us == 0: {last_sample}"
        );
        let summary_rec = recs.last().unwrap();
        assert_eq!(summary_rec["type"], "summary");
        assert_eq!(summary_rec["stop_reason"], "completed");
        assert_eq!(summary_rec["terminal_safety_verified"], true);
        assert!(
            summary_rec["idle_tail_elapsed_s"].as_f64().unwrap_or(0.0) >= 1.0,
            "load.py must wait >= 1.0 s of idle releases before final sample: {summary_rec}"
        );
        // Verify live load.py output has zero INVALID findings in analyze.py!
        let live_rep = run_analyze(&dir.path().join("load.jsonl"), None);
        assert!(
            !live_rep.contains("**Overall Verdict: INVALID"),
            "live load.py run must not be INVALID:\n{live_rep}"
        );

        stop.store(true, Ordering::Release);
        jh.join().unwrap();
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn completion_published_strictly_last_never_visible_during_tail_both_owners() {
    /// Allocation-free wrapper clock that pauses inside `sleep()` when armed so the test thread
    /// can poll `sup.tick_stats()` deterministically while post-push/post-step tail work is open.
    struct TailBarrierClock {
        inner: Arc<LatenessClock>,
        arm_barrier: AtomicBool,
        barrier_state: Mutex<(bool, bool)>, // (entered, released)
        barrier_cv: Condvar,
    }

    impl TailBarrierClock {
        fn new() -> Arc<Self> {
            Arc::new(Self {
                inner: LatenessClock::new(),
                arm_barrier: AtomicBool::new(false),
                barrier_state: Mutex::new((false, false)),
                barrier_cv: Condvar::new(),
            })
        }

        fn wait_entered(&self) {
            let mut g = self.barrier_state.lock().unwrap();
            let deadline = Instant::now() + Duration::from_secs(2);
            while !g.0 {
                let rem = deadline.saturating_duration_since(Instant::now());
                assert!(!rem.is_zero(), "timed out waiting for tail barrier entry");
                g = self.barrier_cv.wait_timeout(g, rem).unwrap().0;
            }
        }

        fn release_barrier(&self) {
            let mut g = self.barrier_state.lock().unwrap();
            g.1 = true;
            self.barrier_cv.notify_all();
        }
    }

    impl ExecutorClock for TailBarrierClock {
        fn now(&self) -> Instant { self.inner.now() }
        fn now_ns(&self) -> u64 { self.inner.now_ns() }
        fn sleep_until_ns(&self, target_ns: u64, stop: &AtomicBool) -> bool {
            self.inner.sleep_until_ns(target_ns, stop)
        }
        fn sleep(&self, dur: Duration, stop: &AtomicBool) {
            self.inner.sleep(dur, stop);
            if self.arm_barrier.swap(false, Ordering::AcqRel) {
                let mut g = self.barrier_state.lock().unwrap();
                g.0 = true;
                self.barrier_cv.notify_all();
                while !g.1 && !stop.load(Ordering::Acquire) {
                    g = self.barrier_cv.wait(g).unwrap();
                }
            }
        }
        fn wake(&self) {
            self.inner.wake();
            self.release_barrier();
        }
    }

    // (a) Legacy owner: poll during open tail stall MUST see releases_started == 1, releases_total == 0,
    //     in_flight_release_us > 0, and NEVER releases_total == 1 before the tail seals.
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let clock = TailBarrierClock::new();
        let stop = Arc::new(AtomicBool::new(false));
        let loop_stats = Arc::new(ExecutorStats::default());
        sup.attach_legacy_loop_stats(loop_stats.clone());
        let rt = tokio::runtime::Handle::current();
        let sup_t = sup.clone();
        let clock_t = clock.clone();
        let stop_t = stop.clone();
        let stats_t = loop_stats.clone();
        let jh = thread::spawn(move || {
            run_periodic_with_clock(Duration::from_millis(20), &stop_t, &stats_t, Some(clock_t.as_ref()), |dt| {
                rt.block_on(sup_t.control_tick(dt));
            });
        });

        loop_stats.post_stats_delay_once_ns.store(25_000_000, Ordering::Release);
        clock.arm_barrier.store(true, Ordering::Release);
        {
            let mut s = clock.inner.state.lock().unwrap();
            s.lateness_queue_ns.push_back(0);
            s.remaining_releases = 1;
            clock.inner.cv.notify_all();
        }
        clock.wait_entered();
        tokio::time::sleep(Duration::from_millis(2)).await;

        for _ in 0..8 {
            let mid = sup.tick_stats();
            assert!(mid.loop_active);
            assert_eq!(mid.releases_started, 1, "release started must be visible during tail: {mid:?}");
            assert_eq!(mid.releases_total, 0, "completion MUST NOT publish before tail finishes: {mid:?}");
            assert!(mid.in_flight_release_us > 0, "in_flight_release_us must stay nonzero during tail: {mid:?}");
            assert!(!mid.stats_snapshot_lapped);
        }

        clock.release_barrier();
        // Wait until parked for release #2.
        {
            let mut s = clock.inner.state.lock().unwrap();
            let deadline = Instant::now() + Duration::from_secs(2);
            while (s.completed_sleeps < 1 || s.waiting_for_ns.is_none()) && Instant::now() < deadline {
                let (next, _) = clock.inner.cv.wait_timeout(s, Duration::from_millis(20)).unwrap();
                s = next;
            }
            assert!(s.waiting_for_ns.is_some());
        }
        let sealed = sup.tick_stats();
        assert!(sealed.loop_active);
        assert_eq!(sealed.releases_started, 1);
        assert_eq!(sealed.releases_total, 1);
        assert_eq!(sealed.in_flight_release_us, 0);
        assert!(sealed.release_us.max >= 25_000);
        assert!(sealed.release_us_max_ever >= 25_000);
        assert_eq!(sealed.releases_over_20ms, 1);
        assert_eq!(sealed.skipped_releases, 1);

        stop.store(true, Ordering::Release);
        clock.wake();
        jh.join().unwrap();
        let after_stop = sup.tick_stats();
        assert!(!after_stop.loop_active);
        assert_eq!(after_stop.in_flight_release_us, 0);
    }

    // (b) Executor owner: poll during open post-diag_tx.push tail MUST see releases_started == 1,
    //     releases_total == 0, in_flight_release_us > 0, and NEVER releases_total == 1 until sealed.
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let clock = TailBarrierClock::new();
        let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(20));
        cfg.alloc_timing = true;
        let handle = sup
            .start_executor_with_hooks(
                cfg,
                PersistTestHooks {
                    clock: Some(clock.clone()),
                    ..Default::default()
                },
            )
            .await
            .unwrap();

        handle.stats().post_stats_delay_once_ns.store(25_000_000, Ordering::Release);
        clock.arm_barrier.store(true, Ordering::Release);
        {
            let mut s = clock.inner.state.lock().unwrap();
            s.lateness_queue_ns.push_back(0);
            s.remaining_releases = 1;
            clock.inner.cv.notify_all();
        }
        clock.wait_entered();
        tokio::time::sleep(Duration::from_millis(2)).await;

        for _ in 0..8 {
            let mid = sup.tick_stats();
            assert!(mid.loop_active);
            assert_eq!(mid.releases_started, 1);
            assert_eq!(mid.releases_total, 0, "executor completion MUST NOT publish before tail finishes: {mid:?}");
            assert!(mid.in_flight_release_us > 0, "executor in_flight_release_us must stay nonzero during tail: {mid:?}");
            assert!(!mid.stats_snapshot_lapped);
        }

        clock.release_barrier();
        {
            let mut s = clock.inner.state.lock().unwrap();
            let deadline = Instant::now() + Duration::from_secs(2);
            while (s.completed_sleeps < 1 || s.waiting_for_ns.is_none()) && Instant::now() < deadline {
                let (next, _) = clock.inner.cv.wait_timeout(s, Duration::from_millis(20)).unwrap();
                s = next;
            }
            assert!(s.waiting_for_ns.is_some());
        }
        let sealed = sup.tick_stats();
        assert!(sealed.loop_active);
        assert_eq!(sealed.releases_started, 1);
        assert_eq!(sealed.releases_total, 1);
        assert_eq!(sealed.in_flight_release_us, 0);
        assert!(sealed.release_us.max >= 25_000);
        assert!(sealed.release_us_max_ever >= 25_000);
        assert_eq!(sealed.releases_over_20ms, 1);
        assert_eq!(sealed.skipped_releases, 1);
        assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);

        handle.stop_and_join();
        let after_stop = sup.tick_stats();
        assert!(!after_stop.loop_active);
        assert_eq!(after_stop.in_flight_release_us, 0);
    }

    // (c) Finding #3: Preemption strictly AFTER sample amendment and B2 tail-allocation accounting
    //     and BEFORE the completion seal, with next grid point still in the future (50 ms period,
    //     18 ms pre-amend + 4 ms pre-seal = 22 ms >= 20 ms while next grid at 50 ms is NOT crossed).
    //     Both owners MUST keep releases_total == 0 / in_flight > 0 during pre-seal preemption,
    //     then record releases_over_20ms == 1 and release_us_max_ever >= 22_000 at seal!
    for use_executor in [false, true] {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let clock = TailBarrierClock::new();
        let stop = Arc::new(AtomicBool::new(false));
        let mut legacy_jh = None;
        let mut exec_handle = None;
        let stats_ref = if use_executor {
            let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(50));
            cfg.alloc_timing = true;
            let h = sup
                .start_executor_with_hooks(
                    cfg,
                    PersistTestHooks {
                        clock: Some(clock.clone()),
                        ..Default::default()
                    },
                )
                .await
                .unwrap();
            let s = h.stats().clone();
            exec_handle = Some(h);
            s
        } else {
            let loop_stats = Arc::new(ExecutorStats::default());
            sup.attach_legacy_loop_stats(loop_stats.clone());
            let rt = tokio::runtime::Handle::current();
            let sup_t = sup.clone();
            let clock_t = clock.clone();
            let stop_t = stop.clone();
            let stats_t = loop_stats.clone();
            legacy_jh = Some(thread::spawn(move || {
                run_periodic_with_clock(Duration::from_millis(50), &stop_t, &stats_t, Some(clock_t.as_ref()), |dt| {
                    rt.block_on(sup_t.control_tick(dt));
                });
            }));
            loop_stats
        };

        // First release: stall 22 ms strictly AFTER sample amendment / B2 tail-allocation
        // accounting and BEFORE `commit_release_outcome_with_clock`, pausing on the barrier to
        // verify zero premature completion while the next 50 ms grid point is still in the future.
        stats_ref.pre_seal_delay_once_ns.store(22_000_000, Ordering::Release);
        clock.arm_barrier.store(true, Ordering::Release);
        {
            let mut s = clock.inner.state.lock().unwrap();
            s.lateness_queue_ns.push_back(0);
            s.remaining_releases = 1;
            clock.inner.cv.notify_all();
        }
        clock.wait_entered();
        tokio::time::sleep(Duration::from_millis(2)).await;

        let mid = sup.tick_stats();
        assert_eq!(mid.releases_started, 1);
        assert_eq!(
            mid.releases_total, 0,
            "owner(exec={use_executor}) MUST NOT publish completion during pre-seal preemption: {mid:?}"
        );
        assert!(mid.in_flight_release_us > 0);

        clock.release_barrier();
        {
            let mut s = clock.inner.state.lock().unwrap();
            let deadline = Instant::now() + Duration::from_secs(2);
            while (s.completed_sleeps < 1 || s.waiting_for_ns.is_none()) && Instant::now() < deadline {
                let (next, _) = clock.inner.cv.wait_timeout(s, Duration::from_millis(20)).unwrap();
                s = next;
            }
            assert!(s.waiting_for_ns.is_some());
        }

        let sealed = sup.tick_stats();
        assert_eq!(sealed.releases_total, 1);
        assert_eq!(sealed.in_flight_release_us, 0);
        assert_eq!(
            sealed.releases_over_20ms, 1,
            "pre-seal preemption crossing 20 ms (with next grid point still in future) MUST record breach (exec={use_executor}): {sealed:?}"
        );
        assert!(sealed.release_us_max_ever >= 22_000, "{sealed:?}");
        assert_eq!(sealed.skipped_releases, 0, "50 ms grid point was NOT crossed: {sealed:?}");

        // Second release: 18 ms pre-amend + 4 ms pre-seal (split across the old timestamp line!) = 22 ms!
        stats_ref.post_stats_delay_once_ns.store(18_000_000, Ordering::Release);
        stats_ref.pre_seal_delay_once_ns.store(4_000_000, Ordering::Release);
        clock.inner.run_schedule(&[0]);
        let sealed2 = sup.tick_stats();
        assert_eq!(sealed2.releases_total, 2);
        assert_eq!(
            sealed2.releases_over_20ms, 2,
            "18 ms pre-amend + 4 ms post-amend/pre-seal preemption MUST cross 20 ms threshold (exec={use_executor}): {sealed2:?}"
        );
        assert_eq!(sealed2.skipped_releases, 0);

        if let Some(h) = exec_handle {
            assert_eq!(h.stats().total_allocs(), AllocCounts::ZERO);
            h.stop_and_join();
        } else {
            stop.store(true, Ordering::Release);
            clock.wake();
            legacy_jh.unwrap().join().unwrap();
        }
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn loop_exit_and_wait_boundary_skip_accounting_both_owners() {
    /// Wrapper clock that advances `now_ns` inside `sleep()` and optionally flips `stop`
    /// immediately after advancing (simulating post-commit preemption followed by shutdown
    /// before the next sleep entry).
    struct BoundaryDelayClock {
        inner: Arc<LatenessClock>,
        stop_after_sleep: Mutex<Option<Arc<AtomicBool>>>,
    }

    impl BoundaryDelayClock {
        fn new() -> Arc<Self> {
            Arc::new(Self {
                inner: LatenessClock::new(),
                stop_after_sleep: Mutex::new(None),
            })
        }
    }

    impl ExecutorClock for BoundaryDelayClock {
        fn now(&self) -> Instant { self.inner.now() }
        fn now_ns(&self) -> u64 { self.inner.now_ns() }
        fn sleep_until_ns(&self, target_ns: u64, stop: &AtomicBool) -> bool {
            self.inner.sleep_until_ns(target_ns, stop)
        }
        fn sleep(&self, dur: Duration, _stop: &AtomicBool) {
            let add_ns = dur.as_nanos().min(u128::from(u64::MAX)) as u64;
            self.inner.advance_ns(add_ns);
            if let Some(flag) = self.stop_after_sleep.lock().unwrap().take() {
                flag.store(true, Ordering::Release);
            }
        }
        fn wake(&self) { self.inner.wake(); }
    }

    // ---- (1) Legacy owner: all boundary & loop-exit cases ----
    // (1a) Post-commit exact-deadline equality (20 ms on 20 ms grid) & 25 ms preemption on CONTINUATION
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let clock = LatenessClock::new();
        let stop = Arc::new(AtomicBool::new(false));
        let loop_stats = Arc::new(ExecutorStats::default());
        sup.attach_legacy_loop_stats(loop_stats.clone());
        let rt = tokio::runtime::Handle::current();
        let sup_t = sup.clone();
        let clock_t = clock.clone();
        let stop_t = stop.clone();
        let stats_t = loop_stats.clone();
        let jh = thread::spawn(move || {
            run_periodic_with_clock(Duration::from_millis(20), &stop_t, &stats_t, Some(clock_t.as_ref()), |dt| {
                rt.block_on(sup_t.control_tick(dt));
            });
        });

        // Exact-deadline equality: now_ns == next_ns after release #1 seals -> 1 skip, no spin, release #2 runs.
        loop_stats.before_sleep_delay_once_ns.store(20_000_000, Ordering::Release);
        clock.run_schedule(&[0]);
        let s1 = sup.tick_stats();
        assert_eq!(s1.releases_total, 1);
        assert_eq!(s1.skipped_releases, 1, "exact now_ns == next_ns must account 1 skip: {s1:?}");

        // 25 ms post-commit preemption on continuation -> +1 skip (total 2), release #2 runs cleanly.
        loop_stats.before_sleep_delay_once_ns.store(25_000_000, Ordering::Release);
        clock.run_schedule(&[0]);
        let s2 = sup.tick_stats();
        assert_eq!(s2.releases_total, 2);
        assert_eq!(s2.skipped_releases, 2, "25 ms post-commit preemption must account +1 skip: {s2:?}");

        // Clean release #3 followed by late stop during genuine sleep -> 0 extra skips!
        clock.run_schedule(&[0]);
        clock.advance_ns(50_000_000);
        stop.store(true, Ordering::Release);
        clock.wake();
        jh.join().unwrap();
        let s3 = sup.tick_stats();
        assert_eq!(s3.releases_total, 3);
        assert_eq!(s3.skipped_releases, 2, "clean stop during genuine sleep must add 0 skips: {s3:?}");
        assert!(!s3.loop_active);
    }

    // (1b) Legacy owner: post-commit 25 ms preemption + immediate stop -> loop-exit final pass accounts 1 skip!
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let clock = BoundaryDelayClock::new();
        let stop = Arc::new(AtomicBool::new(false));
        let loop_stats = Arc::new(ExecutorStats::default());
        sup.attach_legacy_loop_stats(loop_stats.clone());
        let rt = tokio::runtime::Handle::current();
        let sup_t = sup.clone();
        let clock_t = clock.clone();
        let stop_t = stop.clone();
        let stats_t = loop_stats.clone();
        let jh = thread::spawn(move || {
            run_periodic_with_clock(Duration::from_millis(20), &stop_t, &stats_t, Some(clock_t.as_ref()), |dt| {
                rt.block_on(sup_t.control_tick(dt));
            });
        });

        *clock.stop_after_sleep.lock().unwrap() = Some(stop.clone());
        loop_stats.before_sleep_delay_once_ns.store(25_000_000, Ordering::Release);
        {
            let mut s = clock.inner.state.lock().unwrap();
            s.lateness_queue_ns.push_back(0);
            s.remaining_releases = 1;
            clock.inner.cv.notify_all();
        }
        jh.join().unwrap();
        let snap = sup.tick_stats();
        assert_eq!(snap.releases_total, 1);
        assert_eq!(snap.skipped_releases, 1, "loop-exit final accounting pass must count tail-crossed deadline: {snap:?}");
        assert!(!snap.loop_active);
    }

    // (1c) Legacy owner: stop before first release -> 0 skips (even if clock jumped past first deadline).
    {
        let clock = LatenessClock::new();
        let stop = Arc::new(AtomicBool::new(false));
        let loop_stats = Arc::new(ExecutorStats::default());
        let clock_t = clock.clone();
        let stop_t = stop.clone();
        let stats_t = loop_stats.clone();
        let jh = thread::spawn(move || {
            run_periodic_with_clock(Duration::from_millis(20), &stop_t, &stats_t, Some(clock_t.as_ref()), |_| {});
        });
        // Wait for loop to park on first deadline, advance clock past it, then stop.
        {
            let mut s = clock.state.lock().unwrap();
            let deadline = Instant::now() + Duration::from_secs(2);
            while s.waiting_for_ns.is_none() && Instant::now() < deadline {
                s = clock.cv.wait_timeout(s, Duration::from_millis(20)).unwrap().0;
            }
            assert!(s.waiting_for_ns.is_some());
        }
        clock.advance_ns(50_000_000);
        stop.store(true, Ordering::Release);
        clock.wake();
        jh.join().unwrap();
        assert_eq!(loop_stats.releases_total.load(Ordering::Acquire), 0);
        assert_eq!(loop_stats.skipped_releases.load(Ordering::Acquire), 0);
        assert!(!loop_stats.is_loop_active());
    }

    // ---- (2) Executor owner: all boundary & loop-exit cases ----
    // (2a) Exact-deadline equality + 25 ms post-commit preemption on continuation + clean stop during sleep
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let clock = LatenessClock::new();
        let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(20));
        cfg.alloc_timing = true;
        let handle = sup
            .start_executor_with_hooks(
                cfg,
                PersistTestHooks {
                    clock: Some(clock.clone()),
                    ..Default::default()
                },
            )
            .await
            .unwrap();

        handle.stats().before_sleep_delay_once_ns.store(20_000_000, Ordering::Release);
        clock.run_schedule(&[0]);
        let s1 = sup.tick_stats();
        assert_eq!(s1.releases_total, 1);
        assert_eq!(s1.skipped_releases, 1, "executor exact now_ns == next_ns must account 1 skip: {s1:?}");

        handle.stats().before_sleep_delay_once_ns.store(25_000_000, Ordering::Release);
        clock.run_schedule(&[0]);
        let s2 = sup.tick_stats();
        assert_eq!(s2.releases_total, 2);
        assert_eq!(s2.skipped_releases, 2, "executor 25 ms post-commit preemption must account +1 skip: {s2:?}");
        assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);

        clock.run_schedule(&[0]);
        clock.advance_ns(50_000_000);
        handle.stop_and_join();
        let s3 = sup.tick_stats();
        assert_eq!(s3.releases_total, 3);
        assert_eq!(s3.skipped_releases, 2, "executor clean stop during genuine sleep must add 0 skips: {s3:?}");
        assert!(!s3.loop_active);
    }

    // (2b) Executor owner: post-commit 25 ms preemption + stop -> loop-exit final pass accounts 1 skip!
    {
        struct StopOnBoundaryClock {
            inner: Arc<LatenessClock>,
            trigger_stop: AtomicBool,
            entered_sleep: Mutex<bool>,
            cv: Condvar,
        }
        impl ExecutorClock for StopOnBoundaryClock {
            fn now(&self) -> Instant { self.inner.now() }
            fn now_ns(&self) -> u64 { self.inner.now_ns() }
            fn sleep_until_ns(&self, target_ns: u64, stop: &AtomicBool) -> bool {
                self.inner.sleep_until_ns(target_ns, stop)
            }
            fn sleep(&self, dur: Duration, stop: &AtomicBool) {
                let add_ns = dur.as_nanos().min(u128::from(u64::MAX)) as u64;
                self.inner.advance_ns(add_ns);
                if self.trigger_stop.swap(false, Ordering::AcqRel) {
                    stop.store(true, Ordering::Release);
                    let mut g = self.entered_sleep.lock().unwrap();
                    *g = true;
                    self.cv.notify_all();
                }
            }
            fn wake(&self) { self.inner.wake(); }
        }

        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let clock = Arc::new(StopOnBoundaryClock {
            inner: LatenessClock::new(),
            trigger_stop: AtomicBool::new(true),
            entered_sleep: Mutex::new(false),
            cv: Condvar::new(),
        });
        let handle = sup
            .start_executor_with_hooks(
                ExecutorConfig::for_tests(Duration::from_millis(20)),
                PersistTestHooks {
                    clock: Some(clock.clone()),
                    ..Default::default()
                },
            )
            .await
            .unwrap();
        handle.stats().before_sleep_delay_once_ns.store(25_000_000, Ordering::Release);
        {
            let mut s = clock.inner.state.lock().unwrap();
            s.lateness_queue_ns.push_back(0);
            s.remaining_releases = 1;
            clock.inner.cv.notify_all();
        }
        let deadline = Instant::now() + Duration::from_secs(2);
        while !handle.is_exited() && Instant::now() < deadline {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        assert!(handle.is_exited(), "executor should exit after stop set in post-commit boundary");
        handle.stop_and_join();
        let snap = sup.tick_stats();
        assert_eq!(snap.releases_total, 1);
        assert_eq!(snap.skipped_releases, 1, "executor loop-exit final pass must count tail-crossed deadline: {snap:?}");
        assert!(!snap.loop_active);
    }

    // (2c) Executor owner: stop before first release -> 0 skips!
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let clock = LatenessClock::new();
        let handle = sup
            .start_executor_with_hooks(
                ExecutorConfig::for_tests(Duration::from_millis(20)),
                PersistTestHooks {
                    clock: Some(clock.clone()),
                    ..Default::default()
                },
            )
            .await
            .unwrap();
        {
            let mut s = clock.state.lock().unwrap();
            let deadline = Instant::now() + Duration::from_secs(2);
            while s.waiting_for_ns.is_none() && Instant::now() < deadline {
                s = clock.cv.wait_timeout(s, Duration::from_millis(20)).unwrap().0;
            }
            assert!(s.waiting_for_ns.is_some());
        }
        clock.advance_ns(50_000_000);
        handle.stop_and_join();
        let snap = sup.tick_stats();
        assert_eq!(snap.releases_total, 0);
        assert_eq!(snap.skipped_releases, 0);
        assert!(!snap.loop_active);
    }

    // ---- (3) Finding #4: Real Linux wall-clock check-to-sleep shutdown test on BOTH owners ----
    // Preempt strictly AFTER `mono_now_ns() >= at_ns` passes inside `sleep_until_ns` and BEFORE
    // entering the kernel futex sleep; request shutdown AFTER the next grid point has expired.
    // Both owners MUST preserve the crossed deadline (`skipped_releases >= 1`), while a genuine
    // idle-sleep shutdown (stop requested BEFORE next grid point) MUST add 0 skips!
    for use_executor in [false, true] {
        // (3a) Check-to-sleep boundary preemption across 10 ms period + shutdown -> skipped_releases >= 1!
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let stop = Arc::new(AtomicBool::new(false));
        let mut legacy_jh = None;
        let mut exec_handle = None;
        let stats_ref = if use_executor {
            let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(10));
            cfg.alloc_timing = true;
            let h = sup
                .start_executor_with_hooks(cfg, PersistTestHooks::default())
                .await
                .unwrap();
            let s = h.stats().clone();
            exec_handle = Some(h);
            s
        } else {
            let loop_stats = Arc::new(ExecutorStats::default());
            sup.attach_legacy_loop_stats(loop_stats.clone());
            let rt = tokio::runtime::Handle::current();
            let sup_t = sup.clone();
            let stop_t = stop.clone();
            let stats_t = loop_stats.clone();
            legacy_jh = Some(thread::spawn(move || {
                armd::rt_core::executor::run_periodic(Duration::from_millis(10), &stop_t, &stats_t, |dt| {
                    rt.block_on(sup_t.control_tick(dt));
                });
            }));
            loop_stats
        };

        // Wait for at least 1 real release to complete cleanly, then arm a 35 ms stall right at
        // the check-to-sleep boundary (after `mono_now_ns() >= at_ns`, before kernel wait).
        let wait_r1 = Instant::now() + Duration::from_secs(2);
        while stats_ref.releases_total.load(Ordering::Acquire) < 1 && Instant::now() < wait_r1 {
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        assert!(stats_ref.releases_total.load(Ordering::Acquire) >= 1);
        let skips_before = stats_ref.skipped_releases.load(Ordering::Acquire);
        stats_ref.check_to_sleep_delay_once_ns.store(35_000_000, Ordering::Release);
        // Wait until the control thread consumes the one-shot hook (entered the check-to-sleep stall).
        let wait_hook = Instant::now() + Duration::from_secs(2);
        while stats_ref.check_to_sleep_delay_once_ns.load(Ordering::Acquire) != 0 && Instant::now() < wait_hook {
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
        assert_eq!(stats_ref.check_to_sleep_delay_once_ns.load(Ordering::Acquire), 0);
        // Let >= 18 ms elapse so the 10 ms deadline expires BEFORE shutdown is requested.
        tokio::time::sleep(Duration::from_millis(18)).await;
        if let Some(h) = exec_handle {
            h.stop_and_join();
            assert_eq!(h.stats().total_allocs(), AllocCounts::ZERO);
        } else {
            stats_ref.signal_stop(&stop);
            legacy_jh.unwrap().join().unwrap();
        }
        let snap = sup.tick_stats();
        assert!(!snap.loop_active);
        assert!(
            snap.skipped_releases > skips_before,
            "check-to-sleep shutdown past deadline MUST preserve crossed grid point (exec={use_executor}, before={skips_before}, snap={snap:?})"
        );

        // (3b) Real Linux wall-clock genuine idle sleep shutdown (40 ms period, stop 2 ms into sleep) -> 0 skips!
        let sup_idle = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let stop_idle = Arc::new(AtomicBool::new(false));
        let mut legacy_idle_jh = None;
        let mut exec_idle = None;
        let stats_idle = if use_executor {
            let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(40));
            cfg.alloc_timing = true;
            let h = sup_idle
                .start_executor_with_hooks(cfg, PersistTestHooks::default())
                .await
                .unwrap();
            let s = h.stats().clone();
            exec_idle = Some(h);
            s
        } else {
            let loop_stats = Arc::new(ExecutorStats::default());
            sup_idle.attach_legacy_loop_stats(loop_stats.clone());
            let rt = tokio::runtime::Handle::current();
            let sup_t = sup_idle.clone();
            let stop_t = stop_idle.clone();
            let stats_t = loop_stats.clone();
            legacy_idle_jh = Some(thread::spawn(move || {
                armd::rt_core::executor::run_periodic(Duration::from_millis(40), &stop_t, &stats_t, |dt| {
                    rt.block_on(sup_t.control_tick(dt));
                });
            }));
            loop_stats
        };
        let wait_idle_r1 = Instant::now() + Duration::from_secs(2);
        while stats_idle.releases_total.load(Ordering::Acquire) < 1 && Instant::now() < wait_idle_r1 {
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        assert!(stats_idle.releases_total.load(Ordering::Acquire) >= 1);
        let skips_idle_before = stats_idle.skipped_releases.load(Ordering::Acquire);
        tokio::time::sleep(Duration::from_millis(3)).await;
        if let Some(h) = exec_idle {
            h.stop_and_join();
            assert_eq!(h.stats().total_allocs(), AllocCounts::ZERO);
        } else {
            stats_idle.signal_stop(&stop_idle);
            legacy_idle_jh.unwrap().join().unwrap();
        }
        let snap_idle = sup_idle.tick_stats();
        assert!(!snap_idle.loop_active);
        assert_eq!(
            snap_idle.skipped_releases, skips_idle_before,
            "genuine idle-sleep shutdown BEFORE deadline must add 0 skips (exec={use_executor}): {snap_idle:?}"
        );
    }
}

// ============================================================================
// 11. Three synthetic false-PASS cases (final-during-tail, legacy-unsealed, interrupted),
//     strict JSONL grammar, conservative MIT wake scoping, and live load.py SIGINT capture
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn synthetic_false_pass_cases_grammar_mit_and_load_py_interrupt() {
    let dir = tempfile::tempdir().unwrap();
    let devstat_path = dir.path().join("devstat.txt");
    std::fs::write(
        &devstat_path,
        "=== META ===\nrun_id=q1-fix3\n=== ONCE_START ===\n--- armd_version ---\nARMD_ALLOC_TIMING=1\n=== ONCE_END ===\n",
    )
    .unwrap();

    let make_tick = |ticks: u64| -> serde_json::Value {
        let mut t = base_tick_json(20_100, 1_200);
        t["instance_id"] = serde_json::json!("armd-fix3-1");
        t["boot_id"] = serde_json::json!("boot-uuid-fix3-1");
        t["process_start_unix_ns"] = serde_json::json!(1_700_000_000_000_000_000u64);
        t["owner_mode"] = serde_json::json!("executor");
        t["loop_active"] = serde_json::json!(true);
        t["stats_snapshot_lapped"] = serde_json::json!(false);
        t["stats_writer_drops"] = serde_json::json!(0);
        t["in_flight_release_us"] = serde_json::json!(0);
        t["since_boot_ticks"] = serde_json::json!(ticks);
        t["release_us"] = serde_json::json!({ "p50": 550, "p99": 950, "max": 1_400 });
        t["releases_started"] = serde_json::json!(ticks);
        t["releases_total"] = serde_json::json!(ticks);
        t["release_us_max_ever"] = serde_json::json!(1_400);
        t["releases_over_10ms"] = serde_json::json!(0);
        t["releases_over_15ms"] = serde_json::json!(0);
        t["releases_over_20ms"] = serde_json::json!(0);
        t["wake_late_max_ever"] = serde_json::json!(400);
        t["skipped_releases"] = serde_json::json!(0);
        t["diag_dropped"] = serde_json::json!(0);
        t["wake_lateness_us"] = serde_json::json!({ "p50": 100, "p99": 300, "max": 400 });
        t["wake_lateness_over_1ms"] = serde_json::json!(0);
        t["wake_lateness_over_5ms"] = serde_json::json!(0);
        t["wake_lateness_over_10ms"] = serde_json::json!(0);
        t["control_thread_lock_attempts"] = serde_json::json!(0);
        t["control_thread_alloc_ops"] = serde_json::json!(0);
        t["enqueue_to_wire_us"] = serde_json::json!({ "p50": 80, "p99": 150, "max": 200 });
        t
    };

    let write_jsonl_full = |name: &str,
                            run_id: &str,
                            t0: serde_json::Value,
                            t_mid: serde_json::Value,
                            t1: serde_json::Value,
                            summary: serde_json::Value|
     -> std::path::PathBuf {
        let p = dir.path().join(name);
        let content = format!(
            "{}\n{}\n{}\n{}\n{}\n",
            serde_json::json!({ "type": "meta", "run_id": run_id, "enabled_loads": { "obs_poll": true } }),
            serde_json::json!({ "type": "sample", "run_id": run_id, "phase": "baseline", "elapsed_s": 0.0, "tick": t0, "counters": { "obs_ok": 0 }, "safety_state": "Idle" }),
            serde_json::json!({ "type": "sample", "run_id": run_id, "phase": "idle_tail_start", "elapsed_s": 3.0, "tick": t_mid, "counters": { "obs_ok": 25 }, "safety_state": "Idle" }),
            serde_json::json!({ "type": "sample", "run_id": run_id, "phase": "final", "elapsed_s": 4.2, "tick": t1, "counters": { "obs_ok": 25 }, "safety_state": "Idle" }),
            summary,
        );
        std::fs::write(&p, content).unwrap();
        p
    };

    let complete_summary = |run_id: &str, stop_reason: &str, stop_detail: Option<&str>| -> serde_json::Value {
        serde_json::json!({
            "type": "summary",
            "run_id": run_id,
            "stop_reason": stop_reason,
            "stop_detail": stop_detail,
            "fault": null,
            "safety_state": "Idle",
            "terminal_safety_verified": true,
            "terminal_observation_error": null,
            "counters": { "obs_ok": 25 }
        })
    };

    // 0. Positive control: complete native capture -> PASS.
    let pos_path = write_jsonl_full(
        "positive_control.jsonl",
        "pos-1",
        make_tick(100),
        make_tick(250),
        make_tick(300),
        complete_summary("pos-1", "completed", None),
    );
    let pos_rep = run_analyze(&pos_path, Some(&devstat_path));
    assert!(pos_rep.contains("**Overall Verdict: PASS**"), "positive control must PASS:\n{pos_rep}");

    // 1. Synthetic case 1: `final-during-tail` (mutates ONLY final sample's in_flight_release_us = 5000) -> INVALID!
    let mut tail_t1 = make_tick(300);
    tail_t1["in_flight_release_us"] = serde_json::json!(5_000);
    let tail_path = write_jsonl_full(
        "final-during-tail.jsonl",
        "final-during-tail",
        make_tick(100),
        make_tick(250),
        tail_t1,
        complete_summary("final-during-tail", "completed", None),
    );
    let tail_rep = run_analyze(&tail_path, Some(&devstat_path));
    assert!(tail_rep.contains("**Overall Verdict: INVALID"), "final-during-tail must be INVALID:\n{tail_rep}");
    assert!(
        tail_rep.contains("final sample captured during active in-flight release"),
        "final-during-tail must cite exact reason:\n{tail_rep}"
    );

    // 2. Synthetic case 2: `legacy-unsealed` (legacy tickload.json) -> ALWAYS DIAGNOSTIC ONLY (never PASS)!
    let legacy_path = dir.path().join("legacy-unsealed.json");
    let lt0 = base_tick_json(20_100, 1_200);
    let mut lt1 = base_tick_json(20_100, 1_250);
    lt1["since_boot_ticks"] = serde_json::json!(300);
    let legacy_obj = serde_json::json!({ "fault": null, "ticks": [lt0, lt1] });
    std::fs::write(&legacy_path, serde_json::to_string_pretty(&legacy_obj).unwrap()).unwrap();
    let legacy_rep = run_analyze(&legacy_path, Some(&devstat_path));
    assert!(
        legacy_rep.contains("**Overall Verdict: DIAGNOSTIC ONLY (legacy tickload.json; cannot certify Stage 3 §6 gate)**"),
        "legacy-unsealed must be DIAGNOSTIC ONLY:\n{legacy_rep}"
    );
    assert!(legacy_rep.contains("PASS (proxy)"), "legacy-unsealed must retain proxy breakdowns:\n{legacy_rep}");

    // 3. Synthetic case 3: `interrupted` (mutates ONLY summary.stop_reason = "interrupted") -> INVALID!
    let int_path = write_jsonl_full(
        "interrupted.jsonl",
        "interrupted",
        make_tick(100),
        make_tick(250),
        make_tick(300),
        complete_summary("interrupted", "interrupted", Some("signal:2")),
    );
    let int_rep = run_analyze(&int_path, Some(&devstat_path));
    assert!(int_rep.contains("**Overall Verdict: INVALID"), "interrupted must be INVALID:\n{int_rep}");
    assert!(
        int_rep.contains("stop_reason='interrupted'"),
        "interrupted must cite exact stop_reason:\n{int_rep}"
    );

    // 4. Additional stop_reason enum cases: "error" -> INVALID, "fault" -> FAIL, unknown -> INVALID.
    let err_path = write_jsonl_full(
        "stop_error.jsonl",
        "err-1",
        make_tick(100),
        make_tick(250),
        make_tick(300),
        complete_summary("err-1", "error", Some("idle_tail_insufficient_release_progress")),
    );
    let err_rep = run_analyze(&err_path, Some(&devstat_path));
    assert!(err_rep.contains("**Overall Verdict: INVALID"), "{err_rep}");
    assert!(err_rep.contains("stop_reason='error'"), "{err_rep}");

    let fault_path = write_jsonl_full(
        "stop_fault.jsonl",
        "flt-1",
        make_tick(100),
        make_tick(250),
        make_tick(300),
        complete_summary("flt-1", "fault", Some("safety_state=EStop")),
    );
    let fault_rep = run_analyze(&fault_path, Some(&devstat_path));
    assert!(fault_rep.contains("**Overall Verdict: FAIL**"), "{fault_rep}");

    // 5. Strict JSONL grammar checks: malformed JSON, duplicate object keys at top-level & nested,
    //    nonstandard NaN/Infinity constants, trailing record after summary, foreign run_id,
    //    loop_active == false -> ALL INVALID!
    let malformed_path = dir.path().join("malformed.jsonl");
    std::fs::write(
        &malformed_path,
        format!(
            "{}\n{}\n{{broken json\n{}\n{}\n",
            serde_json::json!({ "type": "meta", "run_id": "m1", "enabled_loads": { "obs_poll": true } }),
            serde_json::json!({ "type": "sample", "run_id": "m1", "phase": "baseline", "elapsed_s": 0.0, "tick": make_tick(100) }),
            serde_json::json!({ "type": "sample", "run_id": "m1", "phase": "final", "elapsed_s": 4.2, "tick": make_tick(300) }),
            complete_summary("m1", "completed", None),
        ),
    )
    .unwrap();
    let mal_rep = run_analyze(&malformed_path, Some(&devstat_path));
    assert!(mal_rep.contains("**Overall Verdict: INVALID"), "{mal_rep}");
    assert!(mal_rep.contains("malformed JSON at line 3"), "{mal_rep}");

    // Blocker #2a: duplicate key in summary ("interrupted" overwritten by "completed") with otherwise valid gate evidence -> INVALID!
    let dup_sum_path = dir.path().join("dup_summary_key.jsonl");
    std::fs::write(
        &dup_sum_path,
        format!(
            "{}\n{}\n{}\n{}\n{{\"type\":\"summary\",\"run_id\":\"dup-s\",\"stop_reason\":\"interrupted\",\"stop_reason\":\"completed\",\"fault\":null,\"safety_state\":\"Idle\",\"terminal_safety_verified\":true,\"counters\":{{\"obs_ok\":25}}}}\n",
            serde_json::json!({ "type": "meta", "run_id": "dup-s", "enabled_loads": { "obs_poll": true } }),
            serde_json::json!({ "type": "sample", "run_id": "dup-s", "phase": "baseline", "elapsed_s": 0.0, "tick": make_tick(100), "counters": { "obs_ok": 0 }, "safety_state": "Idle" }),
            serde_json::json!({ "type": "sample", "run_id": "dup-s", "phase": "idle_tail_start", "elapsed_s": 3.0, "tick": make_tick(250), "counters": { "obs_ok": 25 }, "safety_state": "Idle" }),
            serde_json::json!({ "type": "sample", "run_id": "dup-s", "phase": "final", "elapsed_s": 4.2, "tick": make_tick(300), "counters": { "obs_ok": 25 }, "safety_state": "Idle" }),
        ),
    )
    .unwrap();
    let dup_sum_rep = run_analyze(&dup_sum_path, Some(&devstat_path));
    assert!(dup_sum_rep.contains("**Overall Verdict: INVALID"), "duplicate summary key must be INVALID:\n{dup_sum_rep}");
    assert!(dup_sum_rep.contains("duplicate JSON object key 'stop_reason'"), "{dup_sum_rep}");

    // Blocker #2b: nested duplicate key in tick ("releases_over_20ms":1 overwritten by 0) -> INVALID!
    let dup_tick_raw = serde_json::to_string(&make_tick(300))
        .unwrap()
        .replacen("\"releases_over_20ms\":0", "\"releases_over_20ms\":1,\"releases_over_20ms\":0", 1);
    let dup_tick_path = dir.path().join("dup_tick_key.jsonl");
    std::fs::write(
        &dup_tick_path,
        format!(
            "{}\n{}\n{}\n{{\"type\":\"sample\",\"run_id\":\"dup-t\",\"phase\":\"final\",\"elapsed_s\":4.2,\"tick\":{},\"counters\":{{\"obs_ok\":25}},\"safety_state\":\"Idle\"}}\n{}\n",
            serde_json::json!({ "type": "meta", "run_id": "dup-t", "enabled_loads": { "obs_poll": true } }),
            serde_json::json!({ "type": "sample", "run_id": "dup-t", "phase": "baseline", "elapsed_s": 0.0, "tick": make_tick(100), "counters": { "obs_ok": 0 }, "safety_state": "Idle" }),
            serde_json::json!({ "type": "sample", "run_id": "dup-t", "phase": "idle_tail_start", "elapsed_s": 3.0, "tick": make_tick(250), "counters": { "obs_ok": 25 }, "safety_state": "Idle" }),
            dup_tick_raw,
            complete_summary("dup-t", "completed", None),
        ),
    )
    .unwrap();
    let dup_tick_rep = run_analyze(&dup_tick_path, Some(&devstat_path));
    assert!(dup_tick_rep.contains("**Overall Verdict: INVALID"), "nested duplicate key must be INVALID:\n{dup_tick_rep}");
    assert!(dup_tick_rep.contains("duplicate JSON object key 'releases_over_20ms'"), "{dup_tick_rep}");

    // Blocker #2c: nonstandard NaN constant -> INVALID!
    let nan_path = dir.path().join("nonstandard_nan.jsonl");
    std::fs::write(
        &nan_path,
        format!(
            "{}\n{}\n{}\n{{\"type\":\"sample\",\"run_id\":\"nan-1\",\"phase\":\"final\",\"elapsed_s\":NaN,\"tick\":{},\"counters\":{{\"obs_ok\":25}},\"safety_state\":\"Idle\"}}\n{}\n",
            serde_json::json!({ "type": "meta", "run_id": "nan-1", "enabled_loads": { "obs_poll": true } }),
            serde_json::json!({ "type": "sample", "run_id": "nan-1", "phase": "baseline", "elapsed_s": 0.0, "tick": make_tick(100), "counters": { "obs_ok": 0 }, "safety_state": "Idle" }),
            serde_json::json!({ "type": "sample", "run_id": "nan-1", "phase": "idle_tail_start", "elapsed_s": 3.0, "tick": make_tick(250), "counters": { "obs_ok": 25 }, "safety_state": "Idle" }),
            serde_json::to_string(&make_tick(300)).unwrap(),
            complete_summary("nan-1", "completed", None),
        ),
    )
    .unwrap();
    let nan_rep = run_analyze(&nan_path, Some(&devstat_path));
    assert!(nan_rep.contains("**Overall Verdict: INVALID"), "NaN constant must be INVALID:\n{nan_rep}");
    assert!(nan_rep.contains("nonstandard JSON constant 'NaN'"), "{nan_rep}");

    // Blocker #1a: meta + full_load + final 10 ms later + completed (no baseline, no idle_tail_start) -> INVALID!
    let short_nobase_path = dir.path().join("full_load_10ms_final.jsonl");
    std::fs::write(
        &short_nobase_path,
        format!(
            "{}\n{}\n{}\n{}\n",
            serde_json::json!({ "type": "meta", "run_id": "short-1", "enabled_loads": { "obs_poll": true } }),
            serde_json::json!({ "type": "sample", "run_id": "short-1", "phase": "full_load", "elapsed_s": 0.0, "tick": make_tick(100), "counters": { "obs_ok": 5 }, "safety_state": "Idle" }),
            serde_json::json!({ "type": "sample", "run_id": "short-1", "phase": "final", "elapsed_s": 0.01, "tick": make_tick(101), "counters": { "obs_ok": 5 }, "safety_state": "Idle" }),
            complete_summary("short-1", "completed", None),
        ),
    )
    .unwrap();
    let short_nobase_rep = run_analyze(&short_nobase_path, Some(&devstat_path));
    assert!(short_nobase_rep.contains("**Overall Verdict: INVALID"), "full_load->final 10 ms capture must be INVALID:\n{short_nobase_rep}");
    assert!(short_nobase_rep.contains("expected 'baseline'"), "{short_nobase_rep}");
    assert!(short_nobase_rep.contains("missing idle_tail_start"), "{short_nobase_rep}");

    // Blocker #1b: short idle tail duration (0.4 s < 1.0 s) and insufficient releases_total progress (2 < 10) -> INVALID!
    let short_tail_path = dir.path().join("short_idle_tail.jsonl");
    std::fs::write(
        &short_tail_path,
        format!(
            "{}\n{}\n{}\n{}\n{}\n",
            serde_json::json!({ "type": "meta", "run_id": "st-1", "enabled_loads": { "obs_poll": true } }),
            serde_json::json!({ "type": "sample", "run_id": "st-1", "phase": "baseline", "elapsed_s": 0.0, "tick": make_tick(100), "counters": { "obs_ok": 0 }, "safety_state": "Idle" }),
            serde_json::json!({ "type": "sample", "run_id": "st-1", "phase": "idle_tail_start", "elapsed_s": 3.0, "tick": make_tick(250), "counters": { "obs_ok": 25 }, "safety_state": "Idle" }),
            serde_json::json!({ "type": "sample", "run_id": "st-1", "phase": "final", "elapsed_s": 3.4, "tick": make_tick(252), "counters": { "obs_ok": 25 }, "safety_state": "Idle" }),
            complete_summary("st-1", "completed", None),
        ),
    )
    .unwrap();
    let short_tail_rep = run_analyze(&short_tail_path, Some(&devstat_path));
    assert!(short_tail_rep.contains("**Overall Verdict: INVALID"), "{short_tail_rep}");
    assert!(short_tail_rep.contains("idle tail duration 0.400s < required 1.000s"), "{short_tail_rep}");
    assert!(short_tail_rep.contains("insufficient releases_total progress across idle tail"), "{short_tail_rep}");

    // Blocker #1c: requested obs_poll enabled in meta but obs_ok == 0 -> INVALID!
    let mut no_load_sum = complete_summary("nl-1", "completed", None);
    no_load_sum["counters"]["obs_ok"] = serde_json::json!(0);
    let no_load_path = write_jsonl_full(
        "requested_load_skipped.jsonl",
        "nl-1",
        make_tick(100),
        make_tick(250),
        make_tick(300),
        no_load_sum,
    );
    let no_load_rep = run_analyze(&no_load_path, Some(&devstat_path));
    assert!(no_load_rep.contains("**Overall Verdict: INVALID"), "{no_load_rep}");
    assert!(no_load_rep.contains("requested load(s) did not run (obs_poll)"), "{no_load_rep}");

    // Major #6a: unverified terminal safety state (terminal_safety_verified == false) -> INVALID!
    let mut unv_sum = complete_summary("unv-1", "completed", None);
    unv_sum["terminal_safety_verified"] = serde_json::json!(false);
    unv_sum["terminal_observation_error"] = serde_json::json!("HTTP_500");
    let unv_path = write_jsonl_full(
        "unverified_safety.jsonl",
        "unv-1",
        make_tick(100),
        make_tick(250),
        make_tick(300),
        unv_sum,
    );
    let unv_rep = run_analyze(&unv_path, Some(&devstat_path));
    assert!(unv_rep.contains("**Overall Verdict: INVALID"), "{unv_rep}");
    assert!(unv_rep.contains("unverified terminal safety state"), "{unv_rep}");
    assert!(unv_rep.contains("| Fault / stop trace check | `0 faults` | unverified terminal safety observation | INVALID |"), "{unv_rep}");

    // Major #7: mixed process restart + fault AND mixed host reboot + release breach -> INVALID
    //           (process continuity checked BEFORE hard_fail), while retaining observed FAIL rows!
    let mut restart_flt_t1 = make_tick(300);
    restart_flt_t1["process_start_unix_ns"] = serde_json::json!(1_700_000_999_000_000_000u64);
    let restart_flt_path = write_jsonl_full(
        "restart_and_fault.jsonl",
        "rf-1",
        make_tick(100),
        make_tick(250),
        restart_flt_t1,
        complete_summary("rf-1", "fault", Some("safety_state=EStop")),
    );
    let restart_flt_rep = run_analyze(&restart_flt_path, Some(&devstat_path));
    assert!(
        restart_flt_rep.contains("**Overall Verdict: INVALID (process restart: process_start_unix_ns changed"),
        "restart+fault MUST return INVALID (not FAIL):\n{restart_flt_rep}"
    );
    assert!(
        restart_flt_rep.contains("| Fault / stop trace check | `0 faults` | stop_reason=fault (safety_state=EStop) | FAIL |"),
        "restart+fault MUST retain observed fault in gate table:\n{restart_flt_rep}"
    );

    let mut reboot_breach_mid = make_tick(250);
    reboot_breach_mid["releases_over_20ms"] = serde_json::json!(1);
    reboot_breach_mid["release_us_max_ever"] = serde_json::json!(25_000);
    reboot_breach_mid["release_us"]["max"] = serde_json::json!(25_000);
    let mut reboot_breach_t1 = reboot_breach_mid.clone();
    reboot_breach_t1["boot_id"] = serde_json::json!("boot-uuid-fix3-2");
    reboot_breach_t1["since_boot_ticks"] = serde_json::json!(300);
    reboot_breach_t1["releases_started"] = serde_json::json!(300);
    reboot_breach_t1["releases_total"] = serde_json::json!(300);
    let reboot_breach_path = write_jsonl_full(
        "reboot_and_breach.jsonl",
        "rb-1",
        make_tick(100),
        reboot_breach_mid,
        reboot_breach_t1,
        complete_summary("rb-1", "completed", None),
    );
    let reboot_breach_rep = run_analyze(&reboot_breach_path, Some(&devstat_path));
    assert!(
        reboot_breach_rep.contains("**Overall Verdict: INVALID (host reboot: boot_id changed"),
        "reboot+breach MUST return INVALID:\n{reboot_breach_rep}"
    );
    assert!(
        reboot_breach_rep.contains("| Every release (`release_us`) |") && reboot_breach_rep.contains("| FAIL |"),
        "reboot+breach MUST retain observed release FAIL in gate table:\n{reboot_breach_rep}"
    );

    let trailing_path = dir.path().join("trailing_after_summary.jsonl");
    std::fs::write(
        &trailing_path,
        format!(
            "{}\n{}\n{}\n{}\n{}\n",
            serde_json::json!({ "type": "meta", "run_id": "t1", "enabled_loads": { "obs_poll": true } }),
            serde_json::json!({ "type": "sample", "run_id": "t1", "phase": "baseline", "elapsed_s": 0.0, "tick": make_tick(100) }),
            serde_json::json!({ "type": "sample", "run_id": "t1", "phase": "final", "elapsed_s": 4.2, "tick": make_tick(300) }),
            complete_summary("t1", "completed", None),
            serde_json::json!({ "type": "sample", "run_id": "t1", "phase": "full_load", "elapsed_s": 4.5, "tick": make_tick(305) }),
        ),
    )
    .unwrap();
    let trailing_rep = run_analyze(&trailing_path, Some(&devstat_path));
    assert!(trailing_rep.contains("**Overall Verdict: INVALID"), "{trailing_rep}");
    assert!(trailing_rep.contains("trailing sample record after terminal summary"), "{trailing_rep}");

    let foreign_path = dir.path().join("foreign_run_id.jsonl");
    std::fs::write(
        &foreign_path,
        format!(
            "{}\n{}\n{}\n{}\n",
            serde_json::json!({ "type": "meta", "run_id": "r-a", "enabled_loads": { "obs_poll": true } }),
            serde_json::json!({ "type": "sample", "run_id": "r-a", "phase": "baseline", "elapsed_s": 0.0, "tick": make_tick(100) }),
            serde_json::json!({ "type": "sample", "run_id": "r-other", "phase": "final", "elapsed_s": 4.2, "tick": make_tick(300) }),
            complete_summary("r-a", "completed", None),
        ),
    )
    .unwrap();
    let foreign_rep = run_analyze(&foreign_path, Some(&devstat_path));
    assert!(foreign_rep.contains("**Overall Verdict: INVALID"), "{foreign_rep}");
    assert!(foreign_rep.contains("foreign run_id"), "{foreign_rep}");

    let mut inactive_t1 = make_tick(300);
    inactive_t1["loop_active"] = serde_json::json!(false);
    let inactive_path = write_jsonl_full(
        "loop_inactive.jsonl",
        "inact-1",
        make_tick(100),
        make_tick(250),
        inactive_t1,
        complete_summary("inact-1", "completed", None),
    );
    let inactive_rep = run_analyze(&inactive_path, Some(&devstat_path));
    assert!(inactive_rep.contains("**Overall Verdict: INVALID"), "{inactive_rep}");
    assert!(inactive_rep.contains("loop_active=false"), "{inactive_rep}");

    // 6. Conservative MIT wake scoping:
    // (6a) Baseline-only pre-run 19.5 ms wake carried in mixed window (ticks 100 -> 300) -> INCOMPLETE (NOT false FAIL!).
    let mut b19_0 = make_tick(100);
    b19_0["wake_lateness_us"] = serde_json::json!({ "p50": 100, "p99": 300, "max": 19_500 });
    b19_0["wake_late_max_ever"] = serde_json::json!(19_500);
    let mut b19_mid = make_tick(250);
    b19_mid["wake_lateness_us"] = serde_json::json!({ "p50": 100, "p99": 300, "max": 19_500 });
    b19_mid["wake_late_max_ever"] = serde_json::json!(19_500);
    let mut b19_1 = make_tick(300);
    b19_1["wake_lateness_us"] = serde_json::json!({ "p50": 100, "p99": 300, "max": 19_500 });
    b19_1["wake_late_max_ever"] = serde_json::json!(19_500);
    let b19_path = write_jsonl_full(
        "mit_prerun_19ms.jsonl",
        "mit-19",
        b19_0,
        b19_mid,
        b19_1,
        complete_summary("mit-19", "completed", None),
    );
    let b19_rep = run_analyze(&b19_path, Some(&devstat_path));
    assert!(b19_rep.contains("**Overall Verdict: INCOMPLETE"), "baseline-only 19.5 ms wake must be INCOMPLETE, not FAIL/PASS:\n{b19_rep}");

    // (6b) Pre-run 30 ms HWM + 25 ms mixed window (no aligned marker) -> INCOMPLETE (never false PASS!).
    let mut hw30_0 = make_tick(100);
    hw30_0["wake_lateness_us"] = serde_json::json!({ "p50": 100, "p99": 300, "max": 400 });
    hw30_0["wake_late_max_ever"] = serde_json::json!(30_000);
    let mut hw30_mid = make_tick(250);
    hw30_mid["wake_lateness_us"] = serde_json::json!({ "p50": 100, "p99": 300, "max": 25_000 });
    hw30_mid["wake_late_max_ever"] = serde_json::json!(30_000);
    let mut hw30_1 = make_tick(300);
    hw30_1["wake_lateness_us"] = serde_json::json!({ "p50": 100, "p99": 300, "max": 25_000 });
    hw30_1["wake_late_max_ever"] = serde_json::json!(30_000);
    let hw30_path = write_jsonl_full(
        "mit_mixed_25ms_hwm30.jsonl",
        "mit-hw30",
        hw30_0,
        hw30_mid,
        hw30_1,
        complete_summary("mit-hw30", "completed", None),
    );
    let hw30_rep = run_analyze(&hw30_path, Some(&devstat_path));
    assert!(hw30_rep.contains("**Overall Verdict: INCOMPLETE"), "mixed 25 ms window under 30 ms HWM must not PASS:\n{hw30_rep}");

    // (6c) Late marker across 1500-tick gap with 20 ms pre-run HWM (upper bound 46.6 ms >= 45 ms) -> INCOMPLETE!
    let mut lm0 = make_tick(100);
    lm0["wake_late_max_ever"] = serde_json::json!(20_000);
    let mut lm_mid = make_tick(1600);
    lm_mid["wake_late_max_ever"] = serde_json::json!(20_000);
    lm_mid["since_marker"] = serde_json::json!({
        "marker_id": 1, "status": "active", "releases_total_delta": 50, "wake_late_max": 200
    });
    let mut lm1 = make_tick(1650);
    lm1["wake_late_max_ever"] = serde_json::json!(20_000);
    lm1["since_marker"] = serde_json::json!({
        "marker_id": 1, "status": "active", "releases_total_delta": 100, "wake_late_max": 200
    });
    let lm_path = write_jsonl_full(
        "mit_late_marker.jsonl",
        "mit-lm",
        lm0,
        lm_mid,
        lm1,
        complete_summary("mit-lm", "completed", None),
    );
    let lm_rep = run_analyze(&lm_path, Some(&devstat_path));
    assert!(lm_rep.contains("**Overall Verdict: INCOMPLETE"), "late unaligned marker must NOT mask 1500-tick gap with 20 ms HWM:\n{lm_rep}");

    // (6d) Properly baseline-aligned marker across 1500-tick gap with 20 ms pre-run HWM -> PASS!
    let mut am0 = make_tick(100);
    am0["wake_late_max_ever"] = serde_json::json!(20_000);
    am0["since_marker"] = serde_json::json!({
        "marker_id": 2, "status": "active", "releases_total_delta": 0, "wake_late_max": 0
    });
    let mut am_mid = make_tick(1600);
    am_mid["wake_late_max_ever"] = serde_json::json!(20_000);
    am_mid["since_marker"] = serde_json::json!({
        "marker_id": 2, "status": "active", "releases_total_delta": 1500, "wake_late_max": 300
    });
    let mut am1 = make_tick(1650);
    am1["wake_late_max_ever"] = serde_json::json!(20_000);
    am1["since_marker"] = serde_json::json!({
        "marker_id": 2, "status": "active", "releases_total_delta": 1550, "wake_late_max": 300
    });
    let am_path = write_jsonl_full(
        "mit_aligned_marker.jsonl",
        "mit-am",
        am0,
        am_mid,
        am1,
        complete_summary("mit-am", "completed", None),
    );
    let am_rep = run_analyze(&am_path, Some(&devstat_path));
    assert!(am_rep.contains("**Overall Verdict: PASS**"), "properly baseline-aligned marker must PASS:\n{am_rep}");

    // (6e) Strictly post-rollover window (1200 - 100 >= 1000) with 25 ms wake even under 30 ms pre-run HWM -> FAIL!
    let mut pr0 = make_tick(100);
    pr0["wake_late_max_ever"] = serde_json::json!(30_000);
    let mut pr_mid = make_tick(1200);
    pr_mid["wake_lateness_us"] = serde_json::json!({ "p50": 100, "p99": 300, "max": 25_000 });
    pr_mid["wake_late_max_ever"] = serde_json::json!(30_000);
    let mut pr1 = make_tick(1250);
    pr1["wake_lateness_us"] = serde_json::json!({ "p50": 100, "p99": 300, "max": 25_000 });
    pr1["wake_late_max_ever"] = serde_json::json!(30_000);
    let pr_path = write_jsonl_full(
        "mit_post_rollover_25ms.jsonl",
        "mit-pr",
        pr0,
        pr_mid,
        pr1,
        complete_summary("mit-pr", "completed", None),
    );
    let pr_rep = run_analyze(&pr_path, Some(&devstat_path));
    assert!(pr_rep.contains("**Overall Verdict: FAIL**"), "post-rollover 25 ms wake must FAIL:\n{pr_rep}");

    // (6f) Minor #8: 1,100-tick gap with 500 us pre-run wake HWM and 400 us observed peak
    //      (conservative MIT upper bound = 20 + 0.50 + 1.40 + 0.20 + 5 = 27.10 ms << 45 ms) -> PASS!
    let mut lo0 = make_tick(100);
    lo0["wake_late_max_ever"] = serde_json::json!(500);
    lo0["wake_lateness_us"] = serde_json::json!({ "p50": 100, "p99": 300, "max": 400 });
    let mut lo_mid = make_tick(1200);
    lo_mid["wake_late_max_ever"] = serde_json::json!(500);
    lo_mid["wake_lateness_us"] = serde_json::json!({ "p50": 100, "p99": 300, "max": 400 });
    let mut lo1 = make_tick(1250);
    lo1["wake_late_max_ever"] = serde_json::json!(500);
    lo1["wake_lateness_us"] = serde_json::json!({ "p50": 100, "p99": 300, "max": 400 });
    let lo_path = write_jsonl_full(
        "mit_gap_low_hwm_500us.jsonl",
        "mit-lo",
        lo0,
        lo_mid,
        lo1,
        complete_summary("mit-lo", "completed", None),
    );
    let lo_rep = run_analyze(&lo_path, Some(&devstat_path));
    assert!(
        lo_rep.contains("**Overall Verdict: PASS**"),
        "1,100-tick gap bounded by 500 us HWM (< 45 ms MIT) MUST PASS:\n{lo_rep}"
    );

    // 7. Live `load.py` tests:
    //    (7a) Major #5: signal handler re-entry while holding BOTH SharedState._lock & Event._cond -> 0 deadlock!
    {
        let scripts_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("scripts/rt_qual");
        let py_code = r#"
import os, signal, sys
sys.path.insert(0, sys.argv[1])
import load
state = load.SharedState()
handler = load.make_signal_handler(state)
prev = signal.signal(signal.SIGINT, handler)
try:
    # Hold BOTH SharedState._lock and interrupt_event._cond on the main thread while delivering
    # two nested SIGINTs to verify zero same-thread lock re-entry deadlock!
    with state._lock:
        with state.interrupt_event._cond:
            os.kill(os.getpid(), signal.SIGINT)
            os.kill(os.getpid(), signal.SIGINT)
    assert state.apply_pending_signal() is True
    assert state.get_stop_reason() == ("interrupted", f"signal:{signal.SIGINT}")
    assert state.workers_stop.is_set()
    assert state.interrupt_event.is_set()
finally:
    signal.signal(signal.SIGINT, prev)
"#;
        let out = Command::new("python3")
            .args(["-B", "-c", py_code])
            .arg(&scripts_dir)
            .output()
            .expect("run signal re-entry test");
        assert!(
            out.status.success(),
            "signal re-entry test failed: stdout={} stderr={}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        );
    }

    //    (7b) Blocker #1e + Major #6b: live `load.py` against real/stub HTTP servers verifying:
    //         - deadline expiring during --baseline-s before load starts -> stop_reason="error", exit 3, INVALID!
    //         - post-idle /api/v1/observation HTTP 500 while /diagnostics/tick succeeds -> stop_reason="error",
    //           terminal_safety_verified=false, safety_state=null, INVALID!
    {
        let scripts_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("scripts/rt_qual");
        let out_deadline = dir.path().join("live_deadline_before_load.jsonl");
        let out_obs_fail = dir.path().join("live_terminal_obs_fail.jsonl");
        let py_stub = r#"
import http.server, json, socketserver, subprocess, sys, threading, time

scripts_dir, out_deadline, out_obs_fail = sys.argv[1], sys.argv[2], sys.argv[3]
mode = {"fail_obs_after": 999999, "obs_calls": 0, "tick_calls": 0}

class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass
    def do_GET(self):
        if self.path.startswith("/api/v1/diagnostics/tick"):
            mode["tick_calls"] += 1
            n = 100 + mode["tick_calls"] * 25
            payload = {
                "instance_id": "stub-1", "boot_id": "boot-stub-1",
                "process_start_unix_ns": 1700000000000000000,
                "owner_mode": "executor", "loop_active": True,
                "stats_snapshot_lapped": False, "stats_writer_drops": 0,
                "in_flight_release_us": 0, "since_boot_ticks": n,
                "releases_started": n, "releases_total": n,
                "release_us": {"p50": 500, "p99": 900, "max": 1200},
                "release_us_max_ever": 1200, "releases_over_10ms": 0,
                "releases_over_15ms": 0, "releases_over_20ms": 0,
                "wake_lateness_us": {"p50": 50, "p99": 100, "max": 200},
                "wake_late_max_ever": 200, "skipped_releases": 0,
                "diag_dropped": 0, "tick_us": {"p50": 500, "p99": 900, "max": 1200},
                "period_us": {"p50": 20000, "p99": 20100, "max": 20200},
                "lock_us": {"max": 0}, "wait_us": {"max": 0},
                "reader_hold_us": {"max": 0}, "writer_hold_us": {"max": 0},
            }
            body = json.dumps(payload).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith("/api/v1/observation"):
            mode["obs_calls"] += 1
            if mode["obs_calls"] > mode["fail_obs_after"]:
                self.send_response(500)
                self.end_headers()
                self.wfile.write(b'{"error":"simulated terminal obs failure"}')
                return
            body = json.dumps({"safety_state": "Idle", "fault": None, "feedback_age_ms": [1.0]}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b"{}")

class Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True

srv = Server(("127.0.0.1", 0), Handler)
port = srv.server_address[1]
t = threading.Thread(target=srv.serve_forever, daemon=True)
t.start()
try:
    # Case A: deadline (0.05 s) expires during --baseline-s (0.25 s) before load starts -> exit 3, error!
    rc_a = subprocess.run([
        sys.executable, "-B", f"{scripts_dir}/load.py",
        "--host", "127.0.0.1", "--port", str(port),
        "--duration", "0.05", "--baseline-s", "0.25", "--cooldown-s", "0",
        "--no-motion", "--no-record", "--no-measure", "--no-ws-stream",
        "--no-blackbox-log", "--no-bulk-http", "--obs-poll",
        "--output", out_deadline,
    ], check=False)
    assert rc_a.returncode == 3, f"expected exit 3 on skipped load, got {rc_a.returncode}"

    # Case B: post-idle /observation fails (HTTP 500) after load & pre-idle observation succeed -> exit 3!
    # With --duration 0.15, obs-poll at 5 Hz runs ~1 call + pre-idle call = fail after call 2!
    mode["obs_calls"] = 0
    mode["tick_calls"] = 0
    mode["fail_obs_after"] = 2
    rc_b = subprocess.run([
        sys.executable, "-B", f"{scripts_dir}/load.py",
        "--host", "127.0.0.1", "--port", str(port),
        "--duration", "0.15", "--baseline-s", "0", "--cooldown-s", "0",
        "--obs-hz", "5", "--no-motion", "--no-record", "--no-measure",
        "--no-ws-stream", "--no-blackbox-log", "--no-bulk-http", "--obs-poll",
        "--output", out_obs_fail,
    ], check=False)
    assert rc_b.returncode == 3, f"expected exit 3 on terminal obs failure, got {rc_b.returncode}"
finally:
    srv.shutdown()
"#;
        let out = Command::new("python3")
            .args(["-B", "-c", py_stub])
            .arg(&scripts_dir)
            .arg(&out_deadline)
            .arg(&out_obs_fail)
            .output()
            .expect("run load.py stub tests");
        assert!(
            out.status.success(),
            "load.py stub tests failed: stdout={} stderr={}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        );

        let dl_raw = std::fs::read_to_string(&out_deadline).unwrap();
        let dl_sum: serde_json::Value = serde_json::from_str(dl_raw.lines().last().unwrap()).unwrap();
        assert_eq!(dl_sum["stop_reason"], "error");
        let dl_rep = run_analyze(&out_deadline, Some(&devstat_path));
        assert!(dl_rep.contains("**Overall Verdict: INVALID"), "{dl_rep}");

        let ob_raw = std::fs::read_to_string(&out_obs_fail).unwrap();
        let ob_sum: serde_json::Value = serde_json::from_str(ob_raw.lines().last().unwrap()).unwrap();
        assert_eq!(ob_sum["stop_reason"], "error");
        assert_eq!(ob_sum["terminal_safety_verified"], false);
        assert!(ob_sum["safety_state"].is_null(), "failed terminal obs must NOT leave stale safety_state: {ob_sum}");
        let ob_rep = run_analyze(&out_obs_fail, Some(&devstat_path));
        assert!(ob_rep.contains("**Overall Verdict: INVALID"), "{ob_rep}");
    }

    //    (7c) Live `load.py` SIGINT interruption test (on lock-free RT executor so Lock Gate stays clean):
    //         sends SIGINT mid-run and verifies `stop_reason == "interrupted"` + exit code 130 + `analyze.py` INVALID!
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let handle = sup
            .start_executor_with_hooks(
                ExecutorConfig::for_tests(Duration::from_millis(5)),
                PersistTestHooks::default(),
            )
            .await
            .unwrap();
        let addr = serve(sup.clone()).await;
        let out_jsonl = dir.path().join("live_interrupted.jsonl");
        let script = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("scripts/rt_qual/load.py");
        let mut child = Command::new("python3")
            .arg(&script)
            .args([
                "--host",
                "127.0.0.1",
                "--port",
                &addr.port().to_string(),
                "--duration",
                "10.0",
                "--baseline-s",
                "0",
                "--cooldown-s",
                "0",
                "--sample-interval",
                "0.2",
                "--no-motion",
                "--no-record",
                "--no-measure",
                "--no-ws-stream",
                "--no-blackbox-log",
                "--no-bulk-http",
                "--obs-poll",
                "--output",
            ])
            .arg(&out_jsonl)
            .spawn()
            .expect("spawn load.py");
        // Wait until baseline sample is written, then send SIGINT.
        let wait_deadline = Instant::now() + Duration::from_secs(5);
        while Instant::now() < wait_deadline {
            if let Ok(text) = std::fs::read_to_string(&out_jsonl) {
                if text.lines().count() >= 2 {
                    break;
                }
            }
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
        #[cfg(unix)]
        unsafe {
            libc::kill(child.id() as libc::pid_t, libc::SIGINT);
        }
        let status = tokio::task::spawn_blocking(move || child.wait().expect("wait load.py"))
            .await
            .unwrap();
        assert_eq!(status.code(), Some(130), "SIGINT load.py should exit 130, got {status:?}");
        let raw = std::fs::read_to_string(&out_jsonl).unwrap();
        let recs: Vec<serde_json::Value> = raw
            .lines()
            .filter(|l| !l.trim().is_empty())
            .map(|l| serde_json::from_str(l).unwrap())
            .collect();
        let summary = recs.last().unwrap();
        assert_eq!(summary["type"], "summary");
        assert_eq!(summary["stop_reason"], "interrupted");
        let rep = run_analyze(&out_jsonl, None);
        assert!(rep.contains("**Overall Verdict: INVALID"), "live interrupted run must be INVALID:\n{rep}");
        assert!(rep.contains("stop_reason='interrupted'"), "{rep}");

        handle.stop_and_join();
    }
}

// ============================================================================
// 12. Monotonic write/reset exclusivity under concurrent HTTP GET + POST /mark flood,
//     and executor 32-release paused-consumer amendment retention + saturation accounting
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn monotonic_counters_exclusive_and_mark_resets_only_since_marker_both_owners() {
    async fn run_owner_mark_flood(use_executor: bool) {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.set_lease_enabled(false).await;
        let stop = Arc::new(AtomicBool::new(false));
        let mut legacy_jh: Option<thread::JoinHandle<()>> = None;
        let mut exec_handle = None;

        if use_executor {
            let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(5));
            cfg.alloc_timing = true;
            let h = sup
                .start_executor_with_hooks(
                    cfg,
                    PersistTestHooks {
                        post_publish_delay_once: Some(Arc::new(Mutex::new(Some(Duration::from_millis(25))))),
                        ..Default::default()
                    },
                )
                .await
                .unwrap();
            let wait_spike = Instant::now() + Duration::from_secs(3);
            while h.stats().releases_over_20ms.load(Ordering::Acquire) == 0 && Instant::now() < wait_spike {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
            assert!(h.stats().releases_over_20ms.load(Ordering::Acquire) >= 1);
            exec_handle = Some(h);
        } else {
            let loop_stats = Arc::new(ExecutorStats::default());
            sup.attach_legacy_loop_stats(loop_stats.clone());
            loop_stats.post_stats_delay_once_ns.store(25_000_000, Ordering::Release);
            let rt = tokio::runtime::Handle::current();
            let sup_t = sup.clone();
            let stop_t = stop.clone();
            let stats_t = loop_stats.clone();
            legacy_jh = Some(
                thread::Builder::new()
                    .name("armd-control".into())
                    .spawn(move || {
                        armd::rt_core::executor::run_periodic(
                            Duration::from_millis(5),
                            &stop_t,
                            &stats_t,
                            |dt| {
                                rt.block_on(sup_t.control_tick(dt));
                            },
                        );
                    })
                    .unwrap(),
            );
            let wait_spike = Instant::now() + Duration::from_secs(3);
            while loop_stats.releases_over_20ms.load(Ordering::Acquire) == 0 && Instant::now() < wait_spike {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
            assert!(loop_stats.releases_over_20ms.load(Ordering::Acquire) >= 1);
        }

        let addr = serve(sup.clone()).await;
        let tick_url = format!("http://{addr}/api/v1/diagnostics/tick");
        let mark_url = format!("http://{addr}/api/v1/diagnostics/tick/mark");

        let flood_stop = Arc::new(AtomicBool::new(false));
        let marks_issued = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let marker_task = {
            let stop_m = flood_stop.clone();
            let count_m = marks_issued.clone();
            let url_m = mark_url.clone();
            tokio::spawn(async move {
                let client = reqwest::Client::new();
                while !stop_m.load(Ordering::Acquire) {
                    let resp = client.post(&url_m).send().await.unwrap();
                    assert_eq!(resp.status(), reqwest::StatusCode::OK);
                    count_m.fetch_add(1, Ordering::Relaxed);
                    tokio::time::sleep(Duration::from_millis(12)).await;
                }
            })
        };

        let client = reqwest::Client::new();
        let keys = [
            "releases_started",
            "releases_total",
            "release_us_max_ever",
            "releases_over_10ms",
            "releases_over_15ms",
            "releases_over_20ms",
            "wake_late_max_ever",
            "skipped_releases",
            "diag_dropped",
            "wake_lateness_over_1ms",
            "wake_lateness_over_5ms",
            "wake_lateness_over_10ms",
        ];
        let mut prev = [0u64; 12];
        let mut saw_active_clean_marker = false;
        let mut polls = 0u32;
        let deadline = Instant::now() + Duration::from_secs(3);
        while (polls < 35 || marks_issued.load(Ordering::Relaxed) < 5 || !saw_active_clean_marker)
            && Instant::now() < deadline
        {
            let body: serde_json::Value = client
                .get(&tick_url)
                .send()
                .await
                .unwrap()
                .json()
                .await
                .unwrap();
            for (i, k) in keys.iter().enumerate() {
                let v = body[*k].as_u64().expect(k);
                assert!(
                    v >= prev[i],
                    "process-lifetime monotonic counter `{k}` decreased ({} -> {v}) under GET+POST /mark flood",
                    prev[i]
                );
                prev[i] = v;
            }
            // Initial 25 ms spike must NEVER be erased from lifetime counters by POST /mark.
            assert!(body["release_us_max_ever"].as_u64().unwrap() >= 25_000);
            assert!(body["releases_over_20ms"].as_u64().unwrap() >= 1);
            assert!(body["skipped_releases"].as_u64().unwrap() >= 1);
            assert_eq!(body["stats_writer_drops"], 0);
            if let Some(sm) = body["since_marker"].as_object() {
                if sm.get("status").and_then(|v| v.as_str()) == Some("active")
                    && sm.get("releases_total_delta").and_then(|v| v.as_u64()).unwrap_or(0) >= 1
                    && sm.get("releases_over_20ms_delta").and_then(|v| v.as_u64()) == Some(0)
                    && sm.get("release_us_max").and_then(|v| v.as_u64()).unwrap_or(u64::MAX) < 20_000
                {
                    saw_active_clean_marker = true;
                }
            }
            polls += 1;
            tokio::time::sleep(Duration::from_millis(8)).await;
        }
        assert!(saw_active_clean_marker, "explicit POST /mark must reset since_marker interval cleanly while keeping lifetime peaks");

        flood_stop.store(true, Ordering::Release);
        marker_task.await.unwrap();
        if let Some(h) = exec_handle {
            assert_eq!(h.stats().total_allocs(), AllocCounts::ZERO);
            h.stop_and_join();
        } else {
            stop.store(true, Ordering::Release);
            legacy_jh.unwrap().join().unwrap();
        }
    }

    run_owner_mark_flood(false).await;
    run_owner_mark_flood(true).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_amend_ring_retains_32_paused_releases_and_accounts_saturation() {
    // 1. Genuinely pause `armd-dispatch` via `pause_hub(&sup)` for 32 consecutive executor releases
    //    (> old 16-slot table) with distinct tail amendments: all 32 samples and all 32 amendments
    //    accumulate undrained in `diag_tx` / `diag_amend_tx`, and draining preserves every amended
    //    `release_us` in `LockFreeTickStats` with `diag_dropped == 0` and `stats_writer_drops == 0`.
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let clock = LatenessClock::new();
    let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(20));
    cfg.alloc_timing = true;
    let handle = sup
        .start_executor_with_hooks(
            cfg,
            PersistTestHooks {
                clock: Some(clock.clone()),
                ..Default::default()
            },
        )
        .await
        .unwrap();

    const COUNT: usize = 32;
    let (rel_hub1, hub_jh1) = pause_dispatch_at_hub(&sup, &clock);
    for i in 0..COUNT {
        let delay_ns = 2_000_000u64 + (i as u64) * 100_000;
        handle.stats().post_stats_delay_once_ns.store(delay_ns, Ordering::Release);
        clock.run_schedule(&[0]);
    }
    rel_hub1.send(()).unwrap();
    hub_jh1.join().unwrap();

    let snap = sup.tick_stats();
    assert_eq!(snap.releases_total, (COUNT + 1) as u64);
    assert_eq!(snap.diag_dropped, 0, "32 paused amended releases must not overflow 256-slot diag/amend rings");
    assert_eq!(snap.stats_writer_drops, 0);
    assert!(!snap.stats_snapshot_lapped);
    let samples = sup.tick_samples();
    assert_eq!(samples.len(), COUNT + 1);
    for (i, s) in samples[1..].iter().enumerate() {
        let expected_min_us = 2_000u32 + (i as u32) * 100;
        assert!(
            s.release_us >= expected_min_us,
            "release #{i} (seq={}) lost tail amendment: got {} us, expected >= {expected_min_us} us",
            s.release_seq,
            s.release_us
        );
    }
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);

    // 2. Overflow test: block `armd-dispatch` at `hub.latest()` and burst > DIAG_RING_CAPACITY (256)
    //    amended releases so ring saturation is deterministically accounted in `diag_dropped` with 0 heap allocs.
    const OVERFLOW_BURST: usize = DIAG_RING_CAPACITY + 32;
    let (rel_hub2, hub_jh2) = pause_dispatch_at_hub(&sup, &clock);
    for _ in 0..OVERFLOW_BURST {
        handle.stats().post_stats_delay_once_ns.store(1_500_000, Ordering::Release);
        clock.run_schedule(&[0]);
    }
    rel_hub2.send(()).unwrap();
    hub_jh2.join().unwrap();

    let after = sup.tick_stats();
    assert_eq!(after.releases_total, (COUNT + 2 + OVERFLOW_BURST) as u64);
    assert!(
        after.diag_dropped >= 32,
        "overflowing 256-slot diag/amend rings must increment diag_dropped >= 32, got {}",
        after.diag_dropped
    );
    assert_eq!(after.stats_writer_drops, 0);
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);
    handle.stop_and_join();
}
