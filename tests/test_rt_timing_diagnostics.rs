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
    classify_wake_lateness_ns, take_current_wake_lateness_ns, TickSample, TickStats,
    WakeLatenessGuard,
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

    fn sleep_until_ns(&self, target_ns: u64, stop: &AtomicBool) {
        let mut s = self.state.lock().unwrap();
        while !stop.load(Ordering::Acquire) && s.remaining_releases == 0 {
            s.waiting_for_ns = Some(target_ns);
            self.cv.notify_all();
            s = self.cv.wait(s).unwrap();
        }
        s.waiting_for_ns = None;
        if stop.load(Ordering::Acquire) {
            self.cv.notify_all();
            return;
        }
        s.remaining_releases -= 1;
        let lateness = s.lateness_queue_ns.pop_front().unwrap_or(0);
        s.now_ns = target_ns.max(s.now_ns).saturating_add(lateness);
        s.completed_sleeps += 1;
        self.cv.notify_all();
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
    let grasp = armd::grasp::GraspDeps::http(DetectorUpstream::from_env(), None);
    let router = create_router_full(
        supervisor,
        DetectorUpstream::from_env(),
        grasp,
        DiagnosticsState::default(),
        armd::OperatorAuth::disabled(),
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

    // TickStats fallback preserves raw-ns threshold classification even when no ExecutorStats is attached.
    let mut stats = TickStats::default();
    for &ns in &SCRIPTED_LATENESS_NS {
        let (us, _, _, _) = classify_wake_lateness_ns(ns);
        stats.record(&TickSample {
            tick_us: 500,
            wake_lateness_us: us,
            wake_lateness_ns: ns,
            ..Default::default()
        });
    }
    let snap = stats.snapshot();
    assert_eq!(snap.wake_lateness_us.p50, 2_500);
    assert_eq!(snap.wake_lateness_us.p99, 23_400);
    assert_eq!(snap.wake_lateness_us.max, 23_400);
    assert_eq!(snap.wake_lateness_over_1ms, 7);
    assert_eq!(snap.wake_lateness_over_5ms, 4);
    assert_eq!(snap.wake_lateness_over_10ms, 2);

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

    // (a) Legacy tickload.json lacking Q1 fields: proxies & gaps preserved.
    let legacy_path = dir.path().join("legacy_tickload.json");
    let t0 = base_tick_json(22_000, 1_500);
    let mut t1 = base_tick_json(23_000, 1_800);
    t1["since_boot_ticks"] = serde_json::json!(200);
    let legacy = serde_json::json!({ "fault": null, "ticks": [t0, t1] });
    std::fs::write(&legacy_path, serde_json::to_string_pretty(&legacy).unwrap()).unwrap();
    let legacy_rep = run_analyze(&legacy_path, None);
    assert!(legacy_rep.contains("PASS (proxy)"), "{legacy_rep}");
    assert!(legacy_rep.contains("Wake Overrun Proxy (ms)"), "{legacy_rep}");
    assert!(legacy_rep.contains("3.00 (sched overrun proxy)"), "{legacy_rep}");
    assert!(legacy_rep.contains("`skipped_releases`:"), "{legacy_rep}");
    assert!(legacy_rep.contains("`wake_lateness_us`:"), "{legacy_rep}");

    // (b) New Q1 JSONL with diag_dropped == 0: real fields used, closed gaps omitted!
    let q1_path = dir.path().join("q1_clean.jsonl");
    let mut q0 = base_tick_json(26_000, 1_500);
    q0["owner_mode"] = serde_json::json!("executor");
    q0["skipped_releases"] = serde_json::json!(0);
    q0["diag_dropped"] = serde_json::json!(0);
    q0["wake_lateness_us"] = serde_json::json!({ "p50": 120, "p99": 400, "max": 750 });
    q0["wake_lateness_over_1ms"] = serde_json::json!(0);
    q0["wake_lateness_over_5ms"] = serde_json::json!(0);
    q0["wake_lateness_over_10ms"] = serde_json::json!(0);

    let mut q1 = q0.clone();
    q1["since_boot_ticks"] = serde_json::json!(300);
    q1["wake_lateness_us"] = serde_json::json!({ "p50": 150, "p99": 500, "max": 800 });

    let q1_lines = format!(
        "{}\n{}\n{}\n",
        serde_json::json!({ "type": "meta", "run_id": "q1-clean" }),
        serde_json::json!({ "type": "sample", "phase": "baseline", "tick": q0 }),
        serde_json::json!({ "type": "sample", "phase": "full_load", "tick": q1 }),
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
    d0["enqueue_to_wire_us"] = serde_json::json!({ "p50": 100, "p99": 200, "max": 250 });
    let mut d1 = d0.clone();
    d1["since_boot_ticks"] = serde_json::json!(300);
    d1["diag_dropped"] = serde_json::json!(5);
    let drop_lines = format!(
        "{}\n{}\n{}\n",
        serde_json::json!({ "type": "meta", "run_id": "q1-drop" }),
        serde_json::json!({ "type": "sample", "phase": "baseline", "tick": d0 }),
        serde_json::json!({ "type": "sample", "phase": "full_load", "tick": d1 }),
    );
    std::fs::write(&drop_path, drop_lines).unwrap();
    let drop_rep = run_analyze(&drop_path, Some(&devstat_path));
    assert!(
        drop_rep.contains("**Overall Verdict: INCOMPLETE (diagnostic samples dropped: diag_dropped delta=5)**"),
        "{drop_rep}"
    );
    assert!(drop_rep.contains("INCOMPLETE (diag_dropped > 0)"), "{drop_rep}");

    // (d) diag_dropped > 0 WITH an observed release breach (>= 20 ms) -> MUST still be FAIL!
    let breach_path = dir.path().join("q1_drop_breach.jsonl");
    let mut b1 = d1.clone();
    b1["tick_us"]["max"] = serde_json::json!(25_000);
    b1["max_tick"]["tick_us"] = serde_json::json!(25_000);
    let breach_lines = format!(
        "{}\n{}\n{}\n",
        serde_json::json!({ "type": "meta", "run_id": "q1-drop-breach" }),
        serde_json::json!({ "type": "sample", "phase": "baseline", "tick": d0 }),
        serde_json::json!({ "type": "sample", "phase": "full_load", "tick": b1 }),
    );
    std::fs::write(&breach_path, breach_lines).unwrap();
    let breach_rep = run_analyze(&breach_path, Some(&devstat_path));
    assert!(breach_rep.contains("**Overall Verdict: FAIL**"), "{breach_rep}");
}
