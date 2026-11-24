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

    // Saturate the droppable `diag_tx` ring (cap 256) by stalling `armd-dispatch` via
    // `hub.reader_guard()`, and place a 15 ms lateness + 1 skip on the final dropped release.
    let (rel_hub, hub_jh) = pause_hub(&sup);
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

    // (a) Legacy tickload.json lacking Q1/monotonic fields: proxies & gaps preserved, verdict INVALID.
    let legacy_path = dir.path().join("legacy_tickload.json");
    let t0 = base_tick_json(22_000, 1_500);
    let mut t1 = base_tick_json(23_000, 1_800);
    t1["since_boot_ticks"] = serde_json::json!(200);
    let legacy = serde_json::json!({ "fault": null, "ticks": [t0, t1] });
    std::fs::write(&legacy_path, serde_json::to_string_pretty(&legacy).unwrap()).unwrap();
    let legacy_rep = run_analyze(&legacy_path, None);
    assert!(legacy_rep.contains("**Overall Verdict: INVALID"), "{legacy_rep}");
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

    let mut q1 = q0.clone();
    q1["since_boot_ticks"] = serde_json::json!(300);
    q1["releases_started"] = serde_json::json!(300);
    q1["releases_total"] = serde_json::json!(300);
    q1["wake_lateness_us"] = serde_json::json!({ "p50": 150, "p99": 500, "max": 800 });
    q1["wake_late_max_ever"] = serde_json::json!(800);

    let q1_lines = format!(
        "{}\n{}\n{}\n{}\n",
        serde_json::json!({ "type": "meta", "run_id": "q1-clean" }),
        serde_json::json!({ "type": "sample", "phase": "baseline", "tick": q0 }),
        serde_json::json!({ "type": "sample", "phase": "final", "tick": q1 }),
        serde_json::json!({ "type": "summary", "run_id": "q1-clean", "stop_reason": "completed", "fault": null }),
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
    let mut d1 = d0.clone();
    d1["since_boot_ticks"] = serde_json::json!(300);
    d1["releases_started"] = serde_json::json!(300);
    d1["releases_total"] = serde_json::json!(300);
    d1["diag_dropped"] = serde_json::json!(5);
    let drop_lines = format!(
        "{}\n{}\n{}\n{}\n",
        serde_json::json!({ "type": "meta", "run_id": "q1-drop" }),
        serde_json::json!({ "type": "sample", "phase": "baseline", "tick": d0 }),
        serde_json::json!({ "type": "sample", "phase": "final", "tick": d1 }),
        serde_json::json!({ "type": "summary", "run_id": "q1-drop", "stop_reason": "completed", "fault": null }),
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
        serde_json::json!({ "type": "sample", "phase": "baseline", "tick": d0 }),
        serde_json::json!({ "type": "sample", "phase": "final", "tick": b1 }),
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
            serde_json::json!({ "type": "sample", "phase": "baseline", "tick": m0 }),
            serde_json::json!({ "type": "sample", "phase": "final", "tick": m1 }),
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
            serde_json::json!({ "type": "sample", "phase": "baseline", "tick": r0 }),
            serde_json::json!({ "type": "sample", "phase": "full_load", "tick": r1 }),
            serde_json::json!({ "type": "sample", "phase": "final", "tick": r2 }),
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
            serde_json::json!({ "type": "sample", "phase": "baseline", "tick": s0 }),
            serde_json::json!({ "type": "sample", "phase": "full_load", "tick": s1 }),
            serde_json::json!({ "type": "sample", "phase": "final", "tick": s2 }),
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
            serde_json::json!({ "type": "sample", "phase": "baseline", "tick": d0 }),
            serde_json::json!({ "type": "sample", "phase": "full_load", "tick": p1 }),
            serde_json::json!({ "type": "sample", "phase": "final", "tick": p2 }),
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
            serde_json::json!({ "type": "sample", "phase": "full_load", "tick": f0 }),
            serde_json::json!({ "type": "sample", "phase": "cooldown", "tick": c1 }),
            serde_json::json!({ "type": "sample", "phase": "full_load", "tick": f2 }),
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
    assert_eq!(snap.stats_wait_us.max, 0, "control thread must never wait for tick_stats");
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
        serde_json::json!({ "type": "sample", "phase": "baseline", "elapsed_s": elapsed_s, "tick": t })
    };

    let write_and_run_raw = |name: &str, lines: &[serde_json::Value], include_summary: bool| -> String {
        let p = dir.path().join(name);
        let mut buf = format!("{}\n", serde_json::json!({ "type": "meta", "run_id": name }));
        for l in lines {
            buf.push_str(&format!("{l}\n"));
        }
        if include_summary {
            buf.push_str(&format!(
                "{}\n",
                serde_json::json!({ "type": "summary", "run_id": name, "stop_reason": "completed", "fault": null })
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

    // 1. Clean full-coverage run -> PASS
    let clean_rep = write_and_run("clean.jsonl", &[make_clean(100, 0.0), make_clean(300, 4.0)]);
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
    let mut pre1 = make_clean(300, 4.0);
    pre1["tick"]["release_us"]["max"] = serde_json::json!(25_000);
    pre1["tick"]["release_us_max_ever"] = serde_json::json!(25_000);
    pre1["tick"]["releases_over_20ms"] = serde_json::json!(1);
    let pre_rep = write_and_run("prerun_spike.jsonl", &[pre0, pre1]);
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
    wake_mask0["tick"]["wake_late_max_ever"] = serde_json::json!(15_000);
    let mut wake_mask1 = make_clean(1600, 30.0);
    wake_mask1["tick"]["wake_late_max_ever"] = serde_json::json!(15_000);
    let wake_mask_rep = write_and_run("masked_wake.jsonl", &[wake_mask0, wake_mask1]);
    assert!(wake_mask_rep.contains("**Overall Verdict: INCOMPLETE"), "{wake_mask_rep}");
    assert!(wake_mask_rep.contains("masks in-run wake peak"), "{wake_mask_rep}");

    // 19. Major #4: missing/proxy-only allocation evidence MUST NEVER produce PASS!
    let mut no_alloc0 = make_clean(100, 0.0);
    let mut no_alloc1 = make_clean(300, 4.0);
    for sample in [&mut no_alloc0, &mut no_alloc1] {
        let obj = sample["tick"].as_object_mut().unwrap();
        obj.remove("control_thread_alloc_ops");
        obj.remove("max_tick");
        obj.remove("max_lock");
        obj.remove("max_release");
    }
    let no_alloc_rep = write_and_run("no_alloc_evidence.jsonl", &[no_alloc0, no_alloc1]);
    assert!(no_alloc_rep.contains("**Overall Verdict: INCOMPLETE"), "{no_alloc_rep}");
    assert!(no_alloc_rep.contains("INCOMPLETE (missing/malformed allocation evidence)"), "{no_alloc_rep}");

    let mut proxy_alloc0 = make_clean(100, 0.0);
    let mut proxy_alloc1 = make_clean(300, 4.0);
    proxy_alloc0["tick"].as_object_mut().unwrap().remove("control_thread_alloc_ops");
    proxy_alloc1["tick"].as_object_mut().unwrap().remove("control_thread_alloc_ops");
    let proxy_alloc_rep = write_and_run("proxy_alloc_only.jsonl", &[proxy_alloc0, proxy_alloc1]);
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
        assert!(sample_recs.len() >= 2, "expected >= 2 samples: {raw}");
        assert_eq!(sample_recs[0]["phase"], "baseline");
        assert_eq!(
            sample_recs[0]["counters"]["obs_ok"], 0,
            "baseline sample MUST complete before any load worker fires even with --baseline-s 0"
        );
        let last_sample = *sample_recs.last().unwrap();
        assert_eq!(last_sample["phase"], "final");
        assert_eq!(
            last_sample["tick"]["releases_started"],
            last_sample["tick"]["releases_total"],
            "settled final sample must cover all started releases: {last_sample}"
        );
        assert_eq!(recs.last().unwrap()["type"], "summary");

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
}
