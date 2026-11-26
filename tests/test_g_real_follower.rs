//! Step G: Real Damiao follower RT executor opt-in, admission, MIT refusal,
//! FakeBus end-to-end (enable -> J1 motion -> rest-pose torque-off),
//! energized handback, and executor qualification report tests.

mod common;
mod mit_support;

use std::process::Command;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::{Duration, Instant};

use armd::diagnostics::alloc::{AllocCounts, TimedAlloc};
use armd::driver::{
    ArmDriver, ArmDriverError, DamiaoArmDriver, DriverCapabilities, EnableState, JointTarget,
};
use armd::model::{ArmConfig, JointState};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::executor::{validate_startup_owner, ExecutorConfig, RtOwner};
use armd::safety::SafetyState;
use armd::tracking::TrackingStrategy;
use armd::{create_router, ArmError, ArmSupervisor, OperatorAuth};
use common::{controller, rpc};
use mit_support::FakeBus;
use serde_json::json;
use tokio::net::TcpListener;

#[global_allocator]
static ALLOC: TimedAlloc = TimedAlloc;

fn dm_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm()
        .damiao_ranges
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect()
}

fn build_dm_driver(bus: FakeBus) -> DamiaoArmDriver<FakeBus> {
    let cfg = ArmConfig::rebot_dm();
    let gripper_drive = armd::driver::gripper_drive_from_profile(&cfg).unwrap();
    let mut drv = DamiaoArmDriver::new(bus.clone(), (1..=7).collect())
        .with_tx_budget(cfg.damiao_tx_budget())
        .with_joint_limits(dm_limits())
        .unwrap()
        .with_gripper_drive(gripper_drive)
        .unwrap();
    bus.with(|s| s.push_all_feedback());
    drv.read_joint_states().unwrap();
    drv.check_mapping_ranges();
    drv
}

async fn serve(sup: Arc<ArmSupervisor>) -> std::net::SocketAddr {
    let router = create_router(sup, OperatorAuth::disabled());
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        let _ = axum::serve(listener, router).await;
    });
    addr
}

/// Non-RT driver stub (`supports_rt_executor()` defaults to `Err`).
struct NonRtDriver;
impl ArmDriver for NonRtDriver {
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
    fn enable_state(&self) -> EnableState {
        EnableState::Disabled
    }
    fn is_enabled(&self) -> bool {
        false
    }
    fn is_feedback_valid(&self) -> bool {
        true
    }
    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        vec![Some(Duration::ZERO); 7]
    }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        Ok(vec![JointState::default(); 7])
    }
    fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> {
        Ok(())
    }
    fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> {
        Ok(())
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn damiao_fakebus_executor_e2e_enable_j1_motion_and_rest_pose_torque_off() {
    let bus = FakeBus::new(&dm_limits());
    bus.with(|s| {
        s.tx_log.reserve(8192);
        s.tx_times.reserve(8192);
    });
    let drv = build_dm_driver(bus.clone());
    let cfg = ArmConfig {
        lease_timeout_ms: 60_000,
        ..ArmConfig::rebot_dm()
    };
    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
    sup.start_dispatcher().unwrap();
    let addr = serve(sup.clone()).await;

    let mut exec_cfg = ExecutorConfig::for_tests(Duration::from_millis(10));
    exec_cfg.alloc_timing = true;
    let handle = sup.start_executor(exec_cfg).await.expect("start_executor on Damiao FakeBus");

    // Wait for initial releases to populate diagnostics & observation cache.
    let ready_dl = Instant::now() + Duration::from_secs(2);
    while handle.stats().completed_releases.load(Ordering::Acquire) < 5 && Instant::now() < ready_dl
    {
        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    let tick0: serde_json::Value = reqwest::get(format!("http://{addr}/api/v1/diagnostics/tick"))
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(tick0["owner_mode"], "executor", "{tick0}");
    assert_eq!(tick0["loop_active"], true, "{tick0}");
    assert_eq!(tick0["control_thread_lock_attempts"], 0, "{tick0}");
    assert_eq!(tick0["control_thread_alloc_ops"], 0, "{tick0}");
    assert_eq!(tick0["alloc_timing_enabled"], true, "{tick0}");
    assert!(tick0["executor_wakes"].is_u64(), "{tick0}");
    assert!(tick0["max_release"]["phases_us"].is_object(), "{tick0}");

    // Connect WS controller and acquire lease.
    let (mut ws, gen) = controller(addr).await;
    let obs0 = sup.get_observation().await;
    assert_eq!(obs0.safety_state, SafetyState::Idle);
    assert!(!obs0.is_torqued);

    // 1. Enable torque through RT executor on Damiao FakeBus.
    let ack_en = rpc(&mut ws, gen, obs0.epoch_id, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(ack_en["type"], "command_ack", "enable failed: {ack_en}");
    let obs_en = sup.get_observation().await;
    assert!(obs_en.is_torqued, "arm must be torqued after enable: {obs_en:?}");
    let enable_frames = bus.tx_log().iter().filter(|f| f.data == damiao::CMD_ENABLE).count();
    assert!(enable_frames >= 7, "expected >=7 CMD_ENABLE frames, got {enable_frames}");

    // 2. J1 motion (+0.15 rad -> -0.15 rad -> rest 0.0 rad) at safe duration.
    for target_j1 in [0.15f32, -0.15f32, 0.0f32] {
        let mut target = vec![0.0f32; 7];
        target[0] = target_j1;
        let epoch = sup.get_observation().await.epoch_id;
        let ack_mv = rpc(
            &mut ws,
            gen,
            epoch,
            "/api/v1/motion/jog",
            json!({"target_positions": target, "duration_secs": 0.45}),
        )
        .await;
        assert_eq!(ack_mv["type"], "command_ack", "jog({target_j1}) failed: {ack_mv}");

        let move_dl = Instant::now() + Duration::from_secs(4);
        loop {
            let obs = sup.get_observation().await;
            assert!(
                obs.safety_state.is_operational(),
                "unexpected fault during J1 motion: {obs:?}"
            );
            let q0 = bus.with(|s| s.motors[0].q);
            let done = obs
                .action
                .as_ref()
                .is_none_or(|a| a.state.as_str() != "running");
            if done && (q0 - target_j1).abs() < 0.025 {
                break;
            }
            assert!(
                Instant::now() < move_dl,
                "J1 move to {target_j1} did not settle (q0={q0}, action={:?})",
                obs.action
            );
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    }

    // Confirm J1-J6 are within rest-pose tolerance before torque-off.
    let q_rest: Vec<f32> = bus.with(|s| s.motors.iter().map(|m| m.q).collect());
    for (i, q) in q_rest.iter().take(6).enumerate() {
        assert!(q.abs() < 0.03, "joint {i} not at rest before torque-off: {q_rest:?}");
    }

    // 3. Rest-pose torque-off via RT executor.
    let epoch_off = sup.get_observation().await.epoch_id;
    let ack_off = rpc(
        &mut ws,
        gen,
        epoch_off,
        "/api/v1/torque",
        json!({"enable": false}),
    )
    .await;
    assert_eq!(ack_off["type"], "command_ack", "rest-pose torque-off failed: {ack_off}");
    let obs_off = sup.get_observation().await;
    assert!(!obs_off.is_torqued, "arm must be disarmed after rest-pose torque-off: {obs_off:?}");
    assert_eq!(obs_off.safety_state, SafetyState::Idle);
    let disable_frames = bus.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count();
    assert!(disable_frames >= 7, "expected >=7 CMD_DISABLE frames, got {disable_frames}");

    // 4. Verify zero supervisor-lock attempts, zero RT allocations, and clean diagnostics.
    let tick1: serde_json::Value = reqwest::get(format!("http://{addr}/api/v1/diagnostics/tick"))
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(tick1["owner_mode"], "executor", "{tick1}");
    assert_eq!(tick1["control_thread_lock_attempts"], 0, "{tick1}");
    assert_eq!(tick1["control_thread_alloc_ops"], 0, "{tick1}");
    assert_eq!(sup.inner().control_thread_attempts(), 0);
    assert_eq!(
        handle.stats().total_allocs(),
        AllocCounts::ZERO,
        "Damiao FakeBus executor E2E must perform 0 heap ops on armd-control"
    );

    handle.stop_and_join();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn damiao_fakebus_executor_keeps_driver_admission_mit_refusal_and_qualification_checks() {
    // 1. Startup gate: rebot-dm and mock allowed; Feetech / other followers refused.
    assert!(validate_startup_owner(RtOwner::Executor, "rebot-dm").is_ok());
    assert!(validate_startup_owner(RtOwner::Executor, "mock").is_ok());
    for other in ["so-arm101", "so-arm100", "rebot-rs", "fashionstar", "star-arm102", "sim"] {
        let err = validate_startup_owner(RtOwner::Executor, other).unwrap_err();
        assert!(err.contains(other), "error must name follower {other}: {err}");
    }

    // 2. Driver admission (`supports_rt_executor`): non-RT driver is refused and stays Legacy.
    let sup_nonrt = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(NonRtDriver)));
    sup_nonrt.start_dispatcher().unwrap();
    let err = sup_nonrt
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(10)))
        .await
        .err()
        .expect("non-RT driver must fail supports_rt_executor admission");
    assert!(
        matches!(err, ArmError::Rejected { status: 503, ref code, .. } if code == "UNSUPPORTED_RT_DRIVER"),
        "unexpected driver admission err: {err:?}"
    );
    assert_eq!(sup_nonrt.owner_mode(), armd::supervisor::OWNER_LEGACY);

    // 3. MIT-configured Damiao profile hands over cleanly to OWNER_EXECUTOR (Adapter 4),
    //    while qualification record checks remain enforced before arming.
    let bus = FakeBus::new(&dm_limits());
    bus.with(|s| {
        s.tx_log.reserve(8192);
        s.tx_times.reserve(8192);
    });
    let drv = build_dm_driver(bus.clone());
    let mut mit_cfg = ArmConfig::rebot_dm();
    if let Some(jt) = mit_cfg.joint_tracking.as_mut() {
        jt.enabled = true;
        jt.joints[0].strategy = TrackingStrategy::Mit;
        jt.joints[0].kp = 10.0;
        jt.joints[0].kd = 0.5;
        jt.joints[0].effort_limit_nm = 5.0;
        jt.joints[0].hold_kp = Some(10.0);
        jt.joints[0].hold_kd = Some(0.5);
    }
    let sup_mit = Arc::new(ArmSupervisor::new(mit_cfg).with_driver(Box::new(drv)));
    sup_mit.start_dispatcher().unwrap();
    let exec_mit = sup_mit
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(10)))
        .await
        .expect("MIT-configured Damiao profile must hand over to executor (Adapter 4)");
    assert_eq!(sup_mit.owner_mode(), armd::supervisor::OWNER_EXECUTOR);
    tokio::task::spawn_blocking(move || exec_mit.stop_and_join())
        .await
        .expect("clean executor stop after MIT handoff");

    // 4. Qualification record checks remain enforced (missing/invalid record rejected).
    let mut inner = sup_mit.inner().write().await;
    let q_err = inner
        .mit_qualification_check(&json!({"record": {"version": 999}}))
        .expect_err("invalid mit_qualification record must be rejected");
    assert_eq!(q_err.status, 400, "{q_err:?}");
    let gate_err = inner
        .mit_gate()
        .expect_err("mit_gate without qualification record / RT readiness must refuse");
    assert!(!gate_err.is_empty());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn damiao_fakebus_energized_handback_never_torques_off_away_from_rest() {
    let bus = FakeBus::new(&dm_limits());
    bus.with(|s| {
        s.tx_log.reserve(8192);
        s.tx_times.reserve(8192);
    });
    let drv = build_dm_driver(bus.clone());
    let cfg = ArmConfig {
        lease_timeout_ms: 60_000,
        ..ArmConfig::rebot_dm()
    };
    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
    sup.start_dispatcher().unwrap();
    let addr = serve(sup.clone()).await;

    let _handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(10)))
        .await
        .unwrap();

    // Enable and move J1 away from rest (+0.30 rad).
    let (mut ws, gen) = controller(addr).await;
    let ep0 = sup.get_observation().await.epoch_id;
    assert_eq!(
        rpc(&mut ws, gen, ep0, "/api/v1/torque", json!({"enable": true})).await["type"],
        "command_ack"
    );
    let ep1 = sup.get_observation().await.epoch_id;
    assert_eq!(
        rpc(
            &mut ws,
            gen,
            ep1,
            "/api/v1/motion/jog",
            json!({"target_positions": [0.30, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "duration_secs": 0.4})
        )
        .await["type"],
        "command_ack"
    );
    let dl = Instant::now() + Duration::from_secs(3);
    while (bus.with(|s| s.motors[0].q) - 0.30).abs() > 0.03 && Instant::now() < dl {
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    assert!((bus.with(|s| s.motors[0].q) - 0.30).abs() <= 0.03);
    bus.clear_log();

    // Hand back energized arm to live legacy owner (DRV-F1 handback).
    let (legacy_stop, legacy_jh) = sup
        .handback_to_legacy_with_live_loop(Duration::from_secs(2), Duration::from_millis(10))
        .await
        .expect("energized handback to legacy live loop must succeed");
    assert_eq!(sup.owner_mode(), armd::supervisor::OWNER_LEGACY);

    // Let legacy owner hold for several ticks and assert zero CMD_DISABLE frames away from rest.
    tokio::time::sleep(Duration::from_millis(120)).await;
    let disables = bus.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count();
    assert_eq!(disables, 0, "energized handback away from rest must send 0 CMD_DISABLE frames");
    assert!(bus.with(|s| s.motors.iter().all(|m| m.enabled)), "motors must remain enabled");

    // Deliberate ProcessExit teardown for clean test exit.
    sup.signal_legacy_stop(&legacy_stop);
    let _ = sup
        .emergency_process_exit_disable_until(Instant::now() + Duration::from_millis(500))
        .await;
    let _ = legacy_jh.join();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn analyze_py_executor_report_shows_executor_stats_and_covered_in_sim() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    sup.start_dispatcher().unwrap();
    let addr = serve(sup.clone()).await;

    let mut exec_cfg = ExecutorConfig::for_tests(Duration::from_millis(10));
    exec_cfg.alloc_timing = true;
    let handle = sup.start_executor(exec_cfg).await.unwrap();

    // Wait a few releases, mark baseline, then capture baseline / idle_tail_start / final.
    tokio::time::sleep(Duration::from_millis(80)).await;
    let client = reqwest::Client::new();
    let mark_resp = client
        .post(format!("http://{addr}/api/v1/diagnostics/tick/mark"))
        .header("Origin", format!("http://{addr}"))
        .send()
        .await
        .unwrap();
    assert!(mark_resp.status().is_success());
    tokio::time::sleep(Duration::from_millis(40)).await;

    let get_tick = || async {
        client
            .get(format!("http://{addr}/api/v1/diagnostics/tick"))
            .send()
            .await
            .unwrap()
            .json::<serde_json::Value>()
            .await
            .unwrap()
    };

    let t0 = get_tick().await;
    tokio::time::sleep(Duration::from_millis(150)).await;
    let t_mid = get_tick().await;
    tokio::time::sleep(Duration::from_millis(1100)).await;
    let mut t_end = get_tick().await;
    // Settle if a release is in flight.
    for _ in 0..8 {
        if t_end["in_flight_release_us"] == 0 && t_end["releases_started"] == t_end["releases_total"] {
            break;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
        t_end = get_tick().await;
    }

    let dir = tempfile::tempdir().unwrap();
    let load_path = dir.path().join("exec_load.jsonl");
    let run_id = "test-g-exec";
    let lines = [
        json!({"type": "meta", "run_id": run_id, "enabled_loads": {"obs_poll": true}}),
        json!({"type": "sample", "run_id": run_id, "phase": "baseline", "elapsed_s": 0.0, "tick": t0, "counters": {"obs_ok": 0}, "safety_state": "Idle"}),
        json!({"type": "sample", "run_id": run_id, "phase": "idle_tail_start", "elapsed_s": 0.15, "tick": t_mid, "counters": {"obs_ok": 10}, "safety_state": "Idle"}),
        json!({"type": "sample", "run_id": run_id, "phase": "final", "elapsed_s": 1.35, "tick": t_end, "counters": {"obs_ok": 20}, "safety_state": "Idle"}),
        json!({"type": "summary", "run_id": run_id, "stop_reason": "completed", "fault": null, "safety_state": "Idle", "terminal_safety_verified": true, "counters": {"obs_ok": 20}}),
    ];
    let body = lines
        .iter()
        .map(|v| v.to_string())
        .collect::<Vec<_>>()
        .join("\n")
        + "\n";
    std::fs::write(&load_path, body).unwrap();

    let script = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("scripts/rt_qual/analyze.py");
    let out = Command::new("python3")
        .arg(&script)
        .arg("--load")
        .arg(&load_path)
        .output()
        .expect("run analyze.py");
    assert!(out.status.success(), "{}", String::from_utf8_lossy(&out.stderr));
    let report = String::from_utf8(out.stdout).unwrap();

    assert!(report.contains("- **Owner mode**: `executor`"), "{report}");
    assert!(report.contains("control_thread_lock_attempts=0"), "{report}");
    assert!(report.contains("control_thread_alloc_ops Δ=0 (ARMD_ALLOC_TIMING=1 confirmed=True"), "{report}");
    assert!(report.contains("Executor sub-release wakes"), "{report}");
    assert!(report.contains("Release Phase Marks (`max_release`)"), "{report}");
    assert!(report.contains("Simulation-Only Qualification Coverage (Covered in Sim)"), "{report}");
    assert!(report.contains("covered in sim"), "{report}");

    handle.stop_and_join();
}
