//! Stage 3 PR Y (C2) end-to-end integration, FakeBus CAN TX, J7 PosForce, Shadow zero-mailbox,
//! embedding `InferenceTicket`, lease-loss reclamation, zero-alloc (`TimedAlloc`), and zero-lock
//! (`control_thread_attempts() == 0`) test suite for RT-executor-owned policy live inference.

#![cfg(feature = "policy-runtime")]

mod common;
mod mit_support;

#[global_allocator]
static ALLOC: armd::diagnostics::alloc::TimedAlloc = armd::diagnostics::alloc::TimedAlloc;

use std::fs;
use std::net::SocketAddr;
use std::path::Path;
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

use serde_json::{json, Value};
use tempfile::tempdir;
use tokio::net::TcpListener;
use tokio_tungstenite::connect_async;
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

use armd::api::create_router_with_sandbox;
use armd::api::sandbox::SandboxState;
use armd::auth::OperatorAuth;
use armd::driver::{ArmDriver, DamiaoArmDriver};
use armd::policy_runtime::{
    compute_release_digest, encode_rmlp_bytes, Activation, MlpLayerSpec,
    EMPTY_CALIBRATION_DIGEST,
};
use armd::protocol::damiao::DamiaoLimits;
use armd::rt_core::executor::{ExecutorConfig, PersistTestHooks};
use armd::supervisor::OWNER_EXECUTOR;
use armd::transport::CanFrame;
use armd::{ArmConfig, ArmSupervisor};
use common::{next_of as next_of_any, send, TestSocket};
use mit_support::FakeBus;

fn ensure_tracing_subscriber() {
    static INIT: OnceLock<()> = OnceLock::new();
    INIT.get_or_init(|| {
        let _ = tracing_subscriber::registry()
            .with(
                tracing_subscriber::fmt::layer()
                    .with_writer(std::io::sink)
                    .with_ansi(false),
            )
            .try_init();
    });
}

async fn next_of(ws: &mut TestSocket, kind: &str) -> Value {
    next_of_any(ws, &[kind]).await
}

fn dm_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm()
        .damiao_ranges
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect()
}

fn decode_pos_f32(f: &CanFrame) -> f32 {
    f32::from_le_bytes([f.data[0], f.data[1], f.data[2], f.data[3]])
}

fn decode_j7_pf_vel_raw(f: &CanFrame) -> u16 {
    u16::from_le_bytes([f.data[4], f.data[5]])
}

async fn wait_until(timeout: Duration, mut pred: impl FnMut() -> bool) {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        if pred() {
            return;
        }
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
    assert!(pred(), "condition not satisfied within {timeout:?}");
}

async fn serve(
    sup: Arc<ArmSupervisor>,
    sandbox: Arc<SandboxState>,
    auth: OperatorAuth,
) -> SocketAddr {
    let router = create_router_with_sandbox(sup, auth, sandbox);
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        axum::serve(
            listener,
            router.into_make_service_with_connect_info::<SocketAddr>(),
        )
        .await
        .unwrap();
    });
    addr
}

async fn confirm_clock_handshake(
    client: &reqwest::Client,
    addr: SocketAddr,
    op_token: &str,
    producer_epoch: u64,
) -> u64 {
    for attempt in 0..5 {
        let now_ms = armd::policy_runtime::vision::monotonic_now_ms() as i64;
        let hs1: Value = client
            .post(format!("http://{addr}/api/v1/vision/clock-handshake"))
            .header("Authorization", format!("Bearer {op_token}"))
            .json(&json!({
                "phase": "init",
                "producer_epoch": producer_epoch,
                "producer_monotonic_ms": now_ms
            }))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        let challenge_id = hs1["challenge_id"]
            .as_str()
            .unwrap_or_else(|| panic!("expected challenge_id in {hs1}"));
        let confirm_ms = armd::policy_runtime::vision::monotonic_now_ms() as i64;
        let hs2 = client
            .post(format!("http://{addr}/api/v1/vision/clock-handshake"))
            .header("Authorization", format!("Bearer {op_token}"))
            .json(&json!({
                "phase": "confirm",
                "challenge_id": challenge_id,
                "producer_epoch": producer_epoch,
                "producer_monotonic_ms": confirm_ms,
                "producer_uncertainty_ms": 0,
                "rtt_ms": 0,
                "ttl_ms": 5000
            }))
            .send()
            .await
            .unwrap();
        if hs2.status() == 200 {
            let hs2_val: Value = hs2.json().await.unwrap();
            return hs2_val["handshake_generation"].as_u64().expect("hs_gen");
        }
        let status = hs2.status();
        let body = hs2.text().await.unwrap();
        assert!(
            status == 400 && attempt < 4,
            "clock-handshake confirm failed (status={status}, attempt={attempt}): {body}"
        );
    }
    unreachable!()
}

async fn connect_ws(addr: SocketAddr) -> TestSocket {
    connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap().0
}

async fn acquire(ws: &mut TestSocket, client_id: &str, force: bool, seq: u64) -> u64 {
    let mut frame = json!({"type": "acquire_lease", "client_id": client_id, "seq": seq});
    if force {
        frame["force"] = json!(true);
    }
    send(ws, frame).await;
    let rep = next_of_any(ws, &["lease_granted", "lease_rejected"]).await;
    assert_eq!(rep["type"], "lease_granted", "acquire({client_id}, force={force}) failed: {rep}");
    rep["generation"].as_u64().expect("lease generation")
}

async fn arm_via_ws(ws: &mut TestSocket, gen: u64, epoch: u64, req_id: u64) {
    send(
        ws,
        json!({
            "type": "command",
            "path": "/api/v1/torque",
            "body": {"enable": true},
            "generation": gen,
            "epoch_id": epoch,
            "request_id": req_id,
        }),
    )
    .await;
    let ack = next_of_any(ws, &["command_ack", "command_reject"]).await;
    assert_eq!(ack["type"], "command_ack", "arm_via_ws failed: {ack}");
    assert_eq!(ack["request_id"], req_id, "{ack}");
    assert_eq!(ack["is_torqued"], true, "{ack}");
}

fn write_dm_rmlp_pack(
    sandbox_root: &Path,
    policy_id: &str,
    bias_rad: [f32; 7],
    extra_manifest: Value,
) -> String {
    let dir = sandbox_root.join("policies").join(policy_id);
    fs::create_dir_all(&dir).unwrap();
    let joint_names = [
        "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "gripper",
    ];
    let layers = vec![MlpLayerSpec::new(
        7,
        7,
        Activation::None,
        vec![0.0; 49],
        bias_rad.to_vec(),
    )];
    let rmlp = encode_rmlp_bytes(&layers).unwrap();
    let mut manifest = json!({
        "format": "rebot-onnx-policy/v1",
        "name": policy_id,
        "unit_mode": "rad",
        "action_dim": 7,
        "action_space": "joint_positions",
        "joint_names": joint_names,
        "model_file": "model.rmlp"
    });
    if let (Some(m), Some(e)) = (manifest.as_object_mut(), extra_manifest.as_object()) {
        for (k, v) in e {
            m.insert(k.clone(), v.clone());
        }
    }
    let pj = serde_json::to_vec_pretty(&manifest).unwrap();
    fs::write(dir.join("policy.json"), &pj).unwrap();
    fs::write(dir.join("model.rmlp"), &rmlp).unwrap();
    compute_release_digest(&pj, &rmlp, "model.rmlp")
}

fn write_dm_embedding_rmlp_pack(
    sandbox_root: &Path,
    policy_id: &str,
    bias_rad: [f32; 7],
    emb_dim: usize,
    history_len: usize,
    encoder_sha: &str,
    preprocess_sha: &str,
) -> String {
    let dir = sandbox_root.join("policies").join(policy_id);
    fs::create_dir_all(&dir).unwrap();
    let joint_names = [
        "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "gripper",
    ];
    let in_dim = 7 + emb_dim * history_len;
    let layers = vec![MlpLayerSpec::new(
        in_dim,
        7,
        Activation::None,
        vec![0.0; in_dim * 7],
        bias_rad.to_vec(),
    )];
    let rmlp = encode_rmlp_bytes(&layers).unwrap();
    let manifest = json!({
        "format": "rebot-onnx-policy/v1",
        "name": policy_id,
        "unit_mode": "rad",
        "action_dim": 7,
        "action_space": "joint_positions",
        "joint_names": joint_names,
        "model_file": "model.rmlp",
        "vision": {
            "source": "video_embedding",
            "layout": "embedding_v1",
            "embedding_dim": emb_dim,
            "history_len": history_len,
            "max_staleness_ms": 400,
            "max_window_span_ms": 2000,
            "encoder_artifact_sha256": encoder_sha,
            "preprocess_config_sha256": preprocess_sha,
            "on_stale": "stop"
        }
    });
    let pj = serde_json::to_vec_pretty(&manifest).unwrap();
    fs::write(dir.join("policy.json"), &pj).unwrap();
    fs::write(dir.join("model.rmlp"), &rmlp).unwrap();
    compute_release_digest(&pj, &rmlp, "model.rmlp")
}

fn make_release_gate_json(
    policy_sha256: &str,
    tier: &str,
    fps: u64,
    max_speed_scale: f64,
    scene_id: &str,
    tool_id: &str,
    calibration_digest: &str,
) -> Value {
    json!({
        "schema_version": "rebot-release-gate/v1",
        "authorization_tier": tier,
        "policy_sha256": policy_sha256,
        "scene_id": scene_id,
        "tool_id": tool_id,
        "profile_id": "rebot_dm",
        "calibration_digest": calibration_digest,
        "runner_config": {
            "fps": fps,
            "max_speed_scale": max_speed_scale,
            "unit_mode": "rad",
            "action_space": "joint_positions"
        },
        "sim_eval": {
            "seeds_total": 10,
            "seeds_passed": 10,
            "min_success_rate": 0.8,
            "collisions": 0,
            "keepout_violations": 0
        },
        "prerequisites": {
            "calibration_verified": true,
            "power_cut_witnessed": true,
            "shadow_run_passed": true
        }
    })
}

fn make_fakebus_supervisor(cfg: ArmConfig, pos_force_j7: bool) -> (Arc<ArmSupervisor>, FakeBus) {
    let limits = dm_limits();
    let bus = FakeBus::new(&limits);
    bus.with(|s| {
        if pos_force_j7 {
            s.motors[6].set_reg_u32(
                armd::protocol::damiao::RID_CTRL_MODE,
                armd::protocol::damiao::CTRL_MODE_POS_FORCE,
            );
            s.motors[6].q = -3.0;
        }
        s.immediate_replies = true;
        s.follow_commands = true;
        s.push_all_feedback();
    });
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap()
        .with_tx_budget(cfg.damiao_tx_budget());
    if pos_force_j7 {
        let grip = armd::driver::gripper_drive_from_profile(&cfg).expect("pos-force gripper drive");
        drv = drv.with_gripper_drive(grip).unwrap();
    }
    drv.read_joint_states().unwrap();
    drv.read_joint_states().unwrap();
    drv.check_mapping_ranges();

    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
    (sup, bus)
}

fn dm_bias_from_home(dq0: f32, dq6: f32) -> [f32; 7] {
    let home = ArmConfig::rebot_dm().home_positions;
    [
        home[0] + dq0,
        home[1],
        home[2],
        home[3],
        home[4],
        home[5],
        home[6] + dq6,
    ]
}

// ============================================================================
// 1. Delayed inference across age fence (1.5x and >2x) + already-buffered revoked
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn y1_delayed_inference_across_age_fence_and_already_buffered_revoked() {
    ensure_tracing_subscriber();
    let dir = tempdir().unwrap();
    let sandbox = Arc::new(SandboxState::new(dir.path().to_path_buf()));
    let cfg = ArmConfig {
        lease_timeout_ms: 10_000,
        ..ArmConfig::rebot_dm()
    };
    let sup = Arc::new(ArmSupervisor::new(cfg));
    sup.set_operator_auth_enforced(false).await;
    let delay_hook: Arc<Mutex<Option<Duration>>> = Arc::new(Mutex::new(None));
    let handle = sup
        .start_executor_with_hooks(
            ExecutorConfig::for_tests(Duration::from_millis(10)),
            PersistTestHooks {
                release_delay_once: Some(delay_hook.clone()),
                ..Default::default()
            },
        )
        .await
        .unwrap();
    assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);
    wait_until(Duration::from_secs(2), || sup.hub().latest().is_some()).await;

    let addr = serve(sup.clone(), sandbox.clone(), OperatorAuth::disabled()).await;
    let mut ws = connect_ws(addr).await;
    let gen = acquire(&mut ws, "y1-client", false, 1).await;
    let ep = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;
    arm_via_ws(&mut ws, gen, ep, 1).await;
    let q0_init = sup.hub().latest().unwrap().obs.commanded_positions[0];

    write_dm_rmlp_pack(
        dir.path(),
        "p_move",
        dm_bias_from_home(0.08, 0.0),
        json!({}),
    );

    // (a) 1.5x age fence (75 ms delay > 50 ms max_input_age at 50 fps, < 100 ms deadline,
    //     and well below the 200 ms control-loop safety deadline fault!).
    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/action/policy/start",
            "body": {"policy_id": "p_move", "mode": "live", "fps": 50.0, "max_seconds": 2.0},
            "generation": gen,
            "epoch_id": ep,
            "request_id": 2,
        }),
    )
    .await;
    let ack = next_of_any(&mut ws, &["command_ack", "command_reject"]).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let aid1 = ack["action_id"].as_u64().expect("action_id");
    *delay_hook.lock().unwrap() = Some(Duration::from_millis(75));
    wait_until(Duration::from_secs(2), || {
        sup.hub().latest().is_some_and(|s| {
            s.obs
                .policy
                .as_ref()
                .is_some_and(|p| p.action_id == aid1 && !p.active)
        })
    })
    .await;
    let snap1 = sup.hub().latest().unwrap();
    let p1 = snap1.obs.policy.as_ref().unwrap();
    assert_eq!(
        p1.stop_reason.as_deref(),
        Some("STALE_INPUT"),
        "1.5x delayed target must terminate as STALE_INPUT: {p1:?}"
    );
    wait_until(Duration::from_secs(1), || handle.is_policy_closed(aid1)).await;

    // (b) >2x age fence (130 ms > 100 ms deadline AND > 50 ms max_input_age, < 200 ms control deadline)
    //     -> STILL STALE_INPUT!
    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/action/policy/start",
            "body": {"policy_id": "p_move", "mode": "live", "fps": 50.0, "max_seconds": 2.0},
            "generation": gen,
            "epoch_id": ep,
            "request_id": 3,
        }),
    )
    .await;
    let ack2 = next_of_any(&mut ws, &["command_ack", "command_reject"]).await;
    assert_eq!(ack2["type"], "command_ack", "{ack2}");
    let aid2 = ack2["action_id"].as_u64().expect("action_id 2");
    *delay_hook.lock().unwrap() = Some(Duration::from_millis(130));
    wait_until(Duration::from_secs(2), || {
        sup.hub().latest().is_some_and(|s| {
            s.obs
                .policy
                .as_ref()
                .is_some_and(|p| p.action_id == aid2 && !p.active)
        })
    })
    .await;
    let snap2 = sup.hub().latest().unwrap();
    assert_eq!(
        snap2
            .obs
            .policy
            .as_ref()
            .and_then(|p| p.stop_reason.as_deref()),
        Some("STALE_INPUT"),
        ">2x delayed target must still classify as STALE_INPUT"
    );
    wait_until(Duration::from_secs(1), || handle.is_policy_closed(aid2)).await;

    // (c) Already-buffered target revoked BEFORE consumption: start a fresh run, wait for a
    // submitted target to buffer while a short release pause holds `armd-control`, revoke
    // `rt_policy_authorization()`, and confirm the buffered target is rejected without moving.
    write_dm_rmlp_pack(
        dir.path(),
        "p_rev",
        dm_bias_from_home(0.06, 0.0),
        json!({}),
    );
    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/action/policy/start",
            "body": {"policy_id": "p_rev", "mode": "live", "fps": 10.0, "max_seconds": 5.0},
            "generation": gen,
            "epoch_id": ep,
            "request_id": 4,
        }),
    )
    .await;
    let ack3 = next_of_any(&mut ws, &["command_ack", "command_reject"]).await;
    assert_eq!(ack3["type"], "command_ack", "y1(c) start failed: {ack3}");
    let aid3 = ack3["action_id"].as_u64().expect("action_id 3");
    // Wait for first step to run, then pause a release, submit a frame directly with OLD auth gen,
    // revoke authorization, and verify the buffered frame is refused `POLICY_REVOKED`!
    wait_until(Duration::from_secs(2), || {
        sup.hub()
            .latest()
            .is_some_and(|s| s.obs.policy.as_ref().is_some_and(|p| p.action_id == aid3 && p.steps >= 1))
    })
    .await;
    let old_gen = sup.rt_policy_authorization().current();
    let snap_mid = sup.hub().latest().unwrap();
    let q_before_rev = snap_mid.obs.commanded_positions[0];
    *delay_hook.lock().unwrap() = Some(Duration::from_millis(60));
    tokio::time::sleep(Duration::from_millis(12)).await;
    // Submit a target with a far-away position stamped with `old_gen`, then immediately revoke!
    let mut rev_q = snap_mid.obs.commanded_positions.clone();
    rev_q[0] = q0_init - 0.12;
    let _ = handle.submit_policy(&armd::supervisor::targets::PolicyTarget {
        action_id: aid3,
        stamp: armd::supervisor::targets::PolicyStamp {
            authority: snap_mid.rt.owner.authority,
            authorization: old_gen,
            max_input_age: Duration::from_millis(200),
        },
        targets: &rev_q,
        input_at: Instant::now(),
        vision_at: None,
        vision_frame_id: None,
    });
    sup.rt_policy_authorization().revoke();
    // Wait for the paused release to finish and confirm position NEVER moved toward `q0_init - 0.12`!
    let seq_after = sup.hub().published_seq() + 3;
    wait_until(Duration::from_secs(2), || sup.hub().published_seq() >= seq_after).await;
    let q_after_rev = sup.hub().latest().unwrap().obs.commanded_positions[0];
    assert!(
        q_after_rev >= q_before_rev - 0.01,
        "revoked buffered target ({}) must not pull arm negative from {q_before_rev} (got {q_after_rev})",
        rev_q[0]
    );

    // Clean stop via WS
    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/action/policy/stop",
            "body": {},
            "generation": gen,
            "epoch_id": ep,
            "request_id": 5,
        }),
    )
    .await;
    let stop_ack = next_of(&mut ws, "command_ack").await;
    assert_eq!(stop_ack["stopped"], true, "{stop_ack}");
    wait_until(Duration::from_secs(1), || handle.is_policy_closed(aid3)).await;
    handle.stop_and_join();
}

// ============================================================================
// 2. Shadow mode: 0 mailbox submissions, 0 policy-driven CAN TX, diag overlay,
//    pre-delivery MaintClient enrichment, servo blocked during Shadow & restored after
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn y2_shadow_mode_zero_mailbox_zero_output_diag_overlay_and_servo_restoration() {
    ensure_tracing_subscriber();
    let dir = tempdir().unwrap();
    let sandbox = Arc::new(SandboxState::new(dir.path().to_path_buf()));
    let cfg = ArmConfig {
        lease_timeout_ms: 10_000,
        ..ArmConfig::rebot_dm()
    };
    let (sup, bus) = make_fakebus_supervisor(cfg, false);
    sup.set_operator_auth_enforced(false).await;
    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(10)))
        .await
        .unwrap();
    wait_until(Duration::from_secs(2), || sup.hub().latest().is_some()).await;

    let addr = serve(sup.clone(), sandbox.clone(), OperatorAuth::disabled()).await;
    let mut ws = connect_ws(addr).await;
    let gen = acquire(&mut ws, "y2-shadow", false, 1).await;
    let ep = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;
    arm_via_ws(&mut ws, gen, ep, 1).await;

    let shadow_bias = dm_bias_from_home(0.18, 0.0);
    write_dm_rmlp_pack(dir.path(), "p_shadow", shadow_bias, json!({}));

    let q_before = sup.hub().latest().unwrap().obs.commanded_positions.clone();
    bus.clear_log();
    let sub_before = handle.policy_submitted_count();
    let diag_before = sup.hub().policy_diag_seq();

    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/action/policy/start",
            "body": {"policy_id": "p_shadow", "mode": "shadow", "fps": 25.0, "max_seconds": 0.45},
            "generation": gen,
            "epoch_id": ep,
            "request_id": 2,
        }),
    )
    .await;
    let ack = next_of(&mut ws, "command_ack").await;
    assert_eq!(ack["mode"], "shadow", "{ack}");
    let aid = ack["action_id"].as_u64().unwrap();

    // Wait for Shadow diagnostic overlay updates while running
    wait_until(Duration::from_secs(2), || {
        sup.hub().policy_diag_seq() > diag_before
            && sup.hub().latest().is_some_and(|s| {
                s.obs
                    .policy
                    .as_ref()
                    .is_some_and(|p| p.action_id == aid && p.steps >= 2)
            })
    })
    .await;

    // Raw maintenance `get_observation()` MUST already carry enriched Shadow steps/stats!
    let raw_obs = sup.get_observation().await;
    let raw_p = raw_obs.policy.as_ref().expect("policy obs on raw maint");
    assert_eq!(raw_p.action_id, aid);
    assert!(raw_p.steps >= 2, "pre-delivery MaintClient enrichment must populate Shadow steps");
    assert!(
        raw_p.last_shadow_target.as_ref().is_some_and(|t| (t[0] - shadow_bias[0]).abs() < 1e-4),
        "raw maint obs must carry last_shadow_target: {raw_p:?}"
    );
    assert!(
        raw_obs
            .action
            .as_ref()
            .and_then(|a| a.policy_stats)
            .is_some_and(|st| st.steps >= 2),
        "raw maint obs action.policy_stats must be enriched: {:?}",
        raw_obs.action
    );

    // Servo frame while Shadow is active must be refused (`STREAM_NOT_AUTHORIZED`)!
    let mut servo_q = q_before.clone();
    servo_q[0] += 0.02;
    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/motion/servo",
            "body": {"targets": servo_q},
            "generation": gen,
            "epoch_id": ep,
            "request_id": 3,
        }),
    )
    .await;
    let servo_rej = next_of(&mut ws, "command_reject").await;
    assert_eq!(servo_rej["request_id"], 3, "servo must be rejected while Shadow active: {servo_rej}");
    assert_ne!(servo_rej["code"], "INVALID_COMMAND", "servo body must be valid: {servo_rej}");

    // Wait for Shadow run to finish at max_seconds
    wait_until(Duration::from_secs(3), || {
        sup.hub().latest().is_some_and(|s| {
            s.obs
                .policy
                .as_ref()
                .is_some_and(|p| p.action_id == aid && !p.active)
        })
    })
    .await;

    // ZERO mailbox submissions & ZERO policy-driven CAN TX toward `shadow_bias[0]`!
    assert_eq!(
        handle.policy_submitted_count(),
        sub_before,
        "Shadow mode must make ZERO mailbox submissions"
    );
    let tx_frames = bus.tx_log();
    for f in &tx_frames {
        if f.id == 0x101 && f.data.len() >= 4 {
            let cmd_j1 = decode_pos_f32(f);
            assert!(
                (cmd_j1 - q_before[0]).abs() < 0.02,
                "Shadow mode must produce ZERO policy-driven CAN TX (got J1={cmd_j1} vs hold={})",
                q_before[0]
            );
        }
    }

    // After Shadow ends cleanly with lease still valid, servo authority IS restored!
    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/motion/servo",
            "body": {"targets": servo_q},
            "generation": gen,
            "epoch_id": ep,
            "request_id": 4,
        }),
    )
    .await;
    let servo_ok = next_of_any(&mut ws, &["command_ack", "command_reject"]).await;
    assert_eq!(servo_ok["type"], "command_ack", "servo must be restored for valid lease after Shadow: {servo_ok}");
    assert_eq!(servo_ok["request_id"], 4, "{servo_ok}");

    handle.stop_and_join();
}

// ============================================================================
// 3. J7 PosForce: fresh target emits through single emitter, stale J7 refused,
//    rejected frame installs no stamp
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn y3_j7_pos_force_fresh_emits_stale_refused_and_rejected_installs_no_stamp() {
    ensure_tracing_subscriber();
    let dir = tempdir().unwrap();
    let sandbox = Arc::new(SandboxState::new(dir.path().to_path_buf()));
    let cfg = ArmConfig {
        lease_timeout_ms: 10_000,
        ..ArmConfig::rebot_dm()
    };
    let (sup, bus) = make_fakebus_supervisor(cfg, true);
    sup.set_operator_auth_enforced(false).await;
    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(10)))
        .await
        .unwrap();
    wait_until(Duration::from_secs(2), || sup.hub().latest().is_some()).await;

    let addr = serve(sup.clone(), sandbox.clone(), OperatorAuth::disabled()).await;
    let mut ws = connect_ws(addr).await;
    let gen = acquire(&mut ws, "y3-j7", false, 1).await;
    let ep = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;
    arm_via_ws(&mut ws, gen, ep, 1).await;

    // Wait for feedback to settle so we know current J7 position (-3.0 rad).
    let settle_seq = sup.hub().published_seq() + 4;
    wait_until(Duration::from_secs(1), || sup.hub().published_seq() >= settle_seq).await;
    let cur = sup.hub().latest().unwrap().obs.commanded_positions.clone();

    // (a) Fresh J7: policy targeting J7 moved by +0.10 rad with a valid supervised release gate!
    let mut bias_fresh = [0.0f32; 7];
    bias_fresh.copy_from_slice(&cur);
    bias_fresh[6] = (cur[6] + 0.10).min(0.0);
    let dig_fresh = write_dm_rmlp_pack(dir.path(), "p_j7_fresh", bias_fresh, json!({}));
    let (scene_id, tool_id) = sandbox.active_scene_and_tool().await;
    let gate_fresh = make_release_gate_json(
        &dig_fresh,
        "supervised_first_run",
        20,
        0.25,
        &scene_id,
        &tool_id,
        EMPTY_CALIBRATION_DIGEST,
    );

    bus.clear_log();
    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/action/policy/start",
            "body": {
                "policy_id": "p_j7_fresh",
                "mode": "live",
                "fps": 20.0,
                "max_seconds": 0.35,
                "i_am_at_the_arm": true,
                "release_gate": gate_fresh
            },
            "generation": gen,
            "epoch_id": ep,
            "request_id": 2,
        }),
    )
    .await;
    let ack = next_of(&mut ws, "command_ack").await;
    let aid1 = ack["action_id"].as_u64().expect("action_id j7 fresh");
    wait_until(Duration::from_secs(2), || {
        sup.hub().latest().is_some_and(|s| {
            s.obs
                .policy
                .as_ref()
                .is_some_and(|p| p.action_id == aid1 && !p.active)
        })
    })
    .await;
    let j7_frames: Vec<(f32, u16)> = bus
        .tx_log()
        .iter()
        .filter(|f| f.id == 0x307 && f.data.len() >= 6)
        .map(|f| (decode_pos_f32(f), decode_j7_pf_vel_raw(f)))
        .collect();
    let j7_tx: Vec<f32> = j7_frames.iter().map(|&(p, _)| p).collect();
    assert!(
        j7_tx.iter().any(|&p| (p - cur[6]).abs() > 0.01),
        "fresh J7 policy target must emit PosForce J7 CAN frames (0x307): {j7_tx:?}"
    );
    // Major 5: verify encoded 0x307 PosForce velocity honors `release_gate.max_speed_scale = 0.25`.
    let max_j7_vel_scaled = ArmConfig::rebot_dm().limits[6].max_velocity * 0.25;
    let max_v_raw_scaled = armd::protocol::damiao::pos_force_vel_raw(max_j7_vel_scaled);
    assert!(
        j7_frames.iter().any(|&(_, v)| v > 0 && v <= max_v_raw_scaled),
        "moving J7 PosForce frames must emit positive velocity <= {max_v_raw_scaled} (0.25x): {j7_frames:?}"
    );
    for &(p, v_raw) in &j7_frames {
        assert!(
            v_raw <= max_v_raw_scaled,
            "gated J7 PosForce frame exceeded 0.25x velocity cap ({v_raw} > {max_v_raw_scaled}, pos={p})"
        );
    }

    // (b) Rejected policy frame (JUMP_LIMIT on J1 while J7 also changes): zero J7 TX for rejected target!
    let cur2 = sup.hub().latest().unwrap().obs.commanded_positions.clone();
    let mut bias_jump = [0.0f32; 7];
    bias_jump.copy_from_slice(&cur2);
    bias_jump[0] += 0.50; // exceeds default 0.25 first-step jump limit!
    bias_jump[6] = cur2[6] - 0.15;
    let dig_jump = write_dm_rmlp_pack(dir.path(), "p_j7_jump", bias_jump, json!({}));
    let gate_jump = make_release_gate_json(
        &dig_jump,
        "supervised_first_run",
        20,
        0.25,
        &scene_id,
        &tool_id,
        EMPTY_CALIBRATION_DIGEST,
    );
    bus.clear_log();
    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/action/policy/start",
            "body": {
                "policy_id": "p_j7_jump",
                "mode": "live",
                "fps": 20.0,
                "max_seconds": 1.0,
                "i_am_at_the_arm": true,
                "release_gate": gate_jump
            },
            "generation": gen,
            "epoch_id": ep,
            "request_id": 3,
        }),
    )
    .await;
    let ack_j = next_of(&mut ws, "command_ack").await;
    let aid_j = ack_j["action_id"].as_u64().unwrap();
    wait_until(Duration::from_secs(2), || {
        sup.hub().latest().is_some_and(|s| {
            s.obs
                .policy
                .as_ref()
                .is_some_and(|p| p.action_id == aid_j && !p.active)
        })
    })
    .await;
    assert_eq!(
        sup.hub()
            .latest()
            .unwrap()
            .obs
            .policy
            .as_ref()
            .and_then(|p| p.stop_reason.as_deref()),
        Some("JUMP_LIMIT")
    );
    for f in bus.tx_log() {
        if f.id == 0x307 && f.data.len() >= 4 {
            let p = decode_pos_f32(&f);
            assert!(
                (p - bias_jump[6]).abs() > 0.05,
                "rejected JUMP_LIMIT frame must never emit J7 target ({p} vs rejected {})",
                bias_jump[6]
            );
        }
    }

    // (c) Stale J7 feedback (> 100 ms): silence motor 7 replies on FakeBus, wait 140 ms, then try
    // to start a live policy -> refused `J7_FEEDBACK_STALE`, zero stale J7 TX!
    bus.with(|s| {
        s.motors[6].silent = true;
    });
    tokio::time::sleep(Duration::from_millis(150)).await;
    bus.clear_log();
    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/action/policy/start",
            "body": {
                "policy_id": "p_j7_fresh",
                "mode": "live",
                "fps": 20.0,
                "max_seconds": 1.0,
                "i_am_at_the_arm": true,
                "release_gate": gate_fresh
            },
            "generation": gen,
            "epoch_id": ep,
            "request_id": 4,
        }),
    )
    .await;
    let rej_stale = next_of(&mut ws, "command_reject").await;
    assert_eq!(
        rej_stale["code"], "J7_FEEDBACK_STALE",
        "stale J7 must refuse live policy start with J7_FEEDBACK_STALE: {rej_stale}"
    );
    for f in bus.tx_log().into_iter().filter(|f| f.id == 0x307 && f.data.len() >= 6) {
        assert_eq!(
            decode_j7_pf_vel_raw(&f),
            0,
            "stale J7 must emit freeze (vel_raw == 0), never policy motion"
        );
        assert!(
            (decode_pos_f32(&f) - cur2[6]).abs() < 0.05,
            "stale J7 target leaked on wire: {} vs frozen {}",
            decode_pos_f32(&f),
            cur2[6]
        );
    }

    handle.stop_and_join();
}

// ============================================================================
// 4. Lease loss mid-run (disconnect, release, preemption, expiry) -> zero old-gen TX,
//    is_policy_closed == true, > MAX_STREAMS sequential runs reclaim slots
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn y4_lease_loss_mid_run_zero_old_gen_tx_close_watermark_and_slot_reclamation() {
    ensure_tracing_subscriber();
    let dir = tempdir().unwrap();
    let sandbox = Arc::new(SandboxState::new(dir.path().to_path_buf()));
    let cfg = ArmConfig {
        lease_timeout_ms: 1_200,
        ..ArmConfig::rebot_dm()
    };
    let (sup, bus) = make_fakebus_supervisor(cfg, false);
    sup.set_operator_auth_enforced(false).await;
    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(10)))
        .await
        .unwrap();
    wait_until(Duration::from_secs(2), || sup.hub().latest().is_some()).await;

    let addr = serve(sup.clone(), sandbox.clone(), OperatorAuth::disabled()).await;
    let (scene_id, tool_id) = sandbox.active_scene_and_tool().await;
    let bias_old = dm_bias_from_home(0.14, 0.0);
    let dig = write_dm_rmlp_pack(dir.path(), "p_lease", bias_old, json!({}));
    let gate = make_release_gate_json(
        &dig,
        "supervised_first_run",
        20,
        0.25,
        &scene_id,
        &tool_id,
        EMPTY_CALIBRATION_DIGEST,
    );

    // (a) Disconnect mid-run -> LEASE_LOST + is_policy_closed + zero further old-gen movement!
    {
        let mut ws1 = connect_ws(addr).await;
        let g1 = acquire(&mut ws1, "y4-c1", false, 1).await;
        let ep = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;
        arm_via_ws(&mut ws1, g1, ep, 1).await;
        send(
            &mut ws1,
            json!({
                "type": "command",
                "path": "/api/v1/action/policy/start",
                "body": {"policy_id": "p_lease", "mode": "live", "fps": 20.0, "max_seconds": 5.0, "i_am_at_the_arm": true, "release_gate": gate},
                "generation": g1,
                "epoch_id": ep,
                "request_id": 2,
            }),
        )
        .await;
        let ack = next_of_any(&mut ws1, &["command_ack", "command_reject"]).await;
        assert_eq!(ack["type"], "command_ack", "y4(a) start failed: {ack}");
        let aid = ack["action_id"].as_u64().unwrap();
        wait_until(Duration::from_secs(2), || {
            sup.hub().latest().is_some_and(|s| s.obs.policy.as_ref().is_some_and(|p| p.action_id == aid && p.steps >= 1))
        })
        .await;
        drop(ws1); // abrupt disconnect!
        wait_until(Duration::from_secs(2), || {
            sup.hub().latest().is_some_and(|s| s.obs.policy.as_ref().is_some_and(|p| p.action_id == aid && !p.active))
        })
        .await;
        let snap = sup.hub().latest().unwrap();
        assert_eq!(
            snap.obs.policy.as_ref().and_then(|p| p.stop_reason.as_deref()),
            Some("LEASE_LOST"),
            "disconnect mid-run must record LEASE_LOST"
        );
        wait_until(Duration::from_secs(1), || handle.is_policy_closed(aid)).await;
        let q_at_loss = snap.obs.commanded_positions[0];
        bus.clear_log();
        tokio::time::sleep(Duration::from_millis(80)).await;
        for f in bus.tx_log() {
            if f.id == 0x101 && f.data.len() >= 4 {
                let p = decode_pos_f32(&f);
                assert!(
                    (p - q_at_loss).abs() < 0.03,
                    "no old-gen policy progress after disconnect (got {p} vs {q_at_loss})"
                );
            }
        }
    }

    // (b) Explicit release + (c) Preemption + (d) Expiry + > MAX_STREAMS (10) sequential runs!
    let mut ws2 = connect_ws(addr).await;
    let g2 = acquire(&mut ws2, "y4-c2", true, 1).await;
    let ep2 = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;
    arm_via_ws(&mut ws2, g2, ep2, 1).await;
    send(
        &mut ws2,
        json!({
            "type": "command",
            "path": "/api/v1/action/policy/start",
            "body": {"policy_id": "p_lease", "mode": "shadow", "fps": 20.0, "max_seconds": 5.0},
            "generation": g2,
            "epoch_id": ep2,
            "request_id": 2,
        }),
    )
    .await;
    let ack2 = next_of_any(&mut ws2, &["command_ack", "command_reject"]).await;
    assert_eq!(ack2["type"], "command_ack", "y4(b) start failed: {ack2}");
    let aid2 = ack2["action_id"].as_u64().unwrap();
    send(&mut ws2, json!({"type": "release_lease", "generation": g2, "seq": 2})).await;
    let _ = next_of(&mut ws2, "lease_released").await;
    wait_until(Duration::from_secs(2), || {
        sup.hub().latest().is_some_and(|s| s.obs.policy.as_ref().is_some_and(|p| p.action_id == aid2 && !p.active))
    })
    .await;
    assert_eq!(
        sup.hub().latest().unwrap().obs.policy.as_ref().and_then(|p| p.stop_reason.as_deref()),
        Some("LEASE_LOST"),
        "explicit release mid-run must record LEASE_LOST (not masked as STOPPED)"
    );
    wait_until(Duration::from_secs(1), || handle.is_policy_closed(aid2)).await;

    // Sequential slot reclamation (> MAX_STREAMS = 8 runs):
    // Start 10 sequential short Shadow runs (renewing lease heartbeat each iteration) to verify no STREAMS_FULL!
    let mut ws3 = connect_ws(addr).await;
    let g3 = acquire(&mut ws3, "y4-reclaim", true, 1).await;
    let ep3 = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;
    for i in 1..=10u64 {
        send(&mut ws3, json!({"type": "heartbeat", "generation": g3, "epoch_id": ep3})).await;
        send(
            &mut ws3,
            json!({
                "type": "command",
                "path": "/api/v1/action/policy/start",
                "body": {"policy_id": "p_lease", "mode": "shadow", "fps": 20.0, "max_seconds": 1.0},
                "generation": g3,
                "epoch_id": ep3,
                "request_id": i * 2,
            }),
        )
        .await;
        let a = next_of_any(&mut ws3, &["command_ack", "command_reject"]).await;
        assert_eq!(a["type"], "command_ack", "y4 reclaim start {i} failed: {a}");
        let id = a["action_id"].as_u64().unwrap();
        send(
            &mut ws3,
            json!({
                "type": "command",
                "path": "/api/v1/action/policy/stop",
                "body": {},
                "generation": g3,
                "epoch_id": ep3,
                "request_id": i * 2 + 1,
            }),
        )
        .await;
        let stop_r = next_of_any(&mut ws3, &["command_ack", "command_reject"]).await;
        assert_eq!(stop_r["type"], "command_ack", "y4 reclaim stop {i} failed: {stop_r}");
        wait_until(Duration::from_secs(1), || handle.is_policy_closed(id)).await;
    }

    // Expiry mid-run: renew heartbeat, start Shadow, and stop sending heartbeats (> 1200 ms lease_timeout_ms)
    send(&mut ws3, json!({"type": "heartbeat", "generation": g3, "epoch_id": ep3})).await;
    send(
        &mut ws3,
        json!({
            "type": "command",
            "path": "/api/v1/action/policy/start",
            "body": {"policy_id": "p_lease", "mode": "shadow", "fps": 20.0, "max_seconds": 5.0},
            "generation": g3,
            "epoch_id": ep3,
            "request_id": 100,
        }),
    )
    .await;
    let ack_exp = next_of_any(&mut ws3, &["command_ack", "command_reject"]).await;
    assert_eq!(ack_exp["type"], "command_ack", "y4 expiry start failed: {ack_exp}");
    let aid_exp = ack_exp["action_id"].as_u64().unwrap();
    wait_until(Duration::from_secs(4), || {
        sup.hub().latest().is_some_and(|s| s.obs.policy.as_ref().is_some_and(|p| p.action_id == aid_exp && !p.active))
    })
    .await;
    let exp_reason = sup
        .hub()
        .latest()
        .unwrap()
        .obs
        .policy
        .as_ref()
        .and_then(|p| p.stop_reason);
    assert!(
        matches!(exp_reason.as_deref(), Some("LEASE_LOST") | Some("WATCHDOG_TIMEOUT")),
        "expected LEASE_LOST or WATCHDOG_TIMEOUT on lease expiry, got {exp_reason:?}"
    );
    wait_until(Duration::from_secs(1), || handle.is_policy_closed(aid_exp)).await;

    handle.stop_and_join();
}

// ============================================================================
// 5. REST off-lock with detached writer held, VideoEmbedding HTTP + ticket check,
//    completion-time operator lock, and supervised qualification ledger append
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn y5_rest_off_lock_embedding_ticket_operator_lock_and_qualification_ledger() {
    ensure_tracing_subscriber();
    let dir = tempdir().unwrap();
    let ledger_path = dir.path().join("qual_ledger.jsonl");
    let sandbox = Arc::new(SandboxState::new(dir.path().to_path_buf()));
    let cfg = ArmConfig {
        lease_timeout_ms: 10_000,
        ..ArmConfig::rebot_dm()
    };
    let (sup, _bus) = make_fakebus_supervisor(cfg, false);
    {
        let mut inner = sup.inner().write().await;
        inner.policy_ledger_path = Some(ledger_path.clone());
    }
    let auth = OperatorAuth::enforced("1234", 600);
    sup.set_operator_auth_enforced(true).await;

    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(10)))
        .await
        .unwrap();
    wait_until(Duration::from_secs(2), || sup.hub().latest().is_some()).await;
    let addr = serve(sup.clone(), sandbox.clone(), auth.clone()).await;

    // Hold detached legacy SupervisorLock write guard across REST & WS operations!
    let _detached_writer = sup.inner().write().await;

    // (a) REST POST /api/v1/action/policy/start without operator token -> 403 OPERATOR_REQUIRED
    //     WITHOUT touching SupervisorLock!
    let client = reqwest::Client::new();
    let r_unauth = client
        .post(format!("http://{addr}/api/v1/action/policy/start"))
        .json(&json!({"policy_id": "x", "mode": "live"}))
        .send()
        .await
        .unwrap();
    assert_eq!(r_unauth.status(), 403);
    let body_unauth: Value = r_unauth.json().await.unwrap();
    assert_eq!(body_unauth["code"], "OPERATOR_REQUIRED");

    // Unlock operator session
    let unlock: Value = client
        .post(format!("http://{addr}/api/v1/auth/unlock"))
        .json(&json!({"pin": "1234"}))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let op_token = unlock["token"].as_str().expect("operator token").to_string();

    // REST POST /api/v1/action/policy/stop with operator token -> 409 NO_LEASE/LEASE_REQUIRED (0 lock!)
    let r_stop = client
        .post(format!("http://{addr}/api/v1/action/policy/stop"))
        .header("Authorization", format!("Bearer {op_token}"))
        .json(&json!({}))
        .send()
        .await
        .unwrap();
    assert_eq!(r_stop.status(), 409);

    // (b) VideoEmbedding HTTP routes on OWNER_EXECUTOR (/api/v1/vision/clock-handshake & /embedding)
    let enc_sha = "a".repeat(64);
    let pre_sha = "b".repeat(64);
    let _emb_dig = write_dm_embedding_rmlp_pack(
        dir.path(),
        "p_emb",
        dm_bias_from_home(0.02, 0.0),
        4,
        1,
        &enc_sha,
        &pre_sha,
    );
    let hs_gen = confirm_clock_handshake(&client, addr, &op_token, 1).await;
    let now_ms = armd::policy_runtime::vision::monotonic_now_ms() as i64;

    let ing = client
        .post(format!("http://{addr}/api/v1/vision/embedding"))
        .header("Authorization", format!("Bearer {op_token}"))
        .json(&json!({
            "handshake_generation": hs_gen,
            "producer_epoch": 1,
            "frame_id": 1,
            "captured_at_ms": now_ms,
            "oldest_captured_at_ms": now_ms,
            "embedding": [0.1, 0.2, 0.3, 0.4],
            "encoder_artifact_sha256": enc_sha,
            "preprocess_config_sha256": pre_sha
        }))
        .send()
        .await
        .unwrap();
    let ing_status = ing.status();
    let ing_body = ing.text().await.unwrap();
    assert_eq!(ing_status, 200, "embedding ingest failed: {ing_body}");

    // (c) WS with Bearer operator token: first run VideoEmbedding live policy (`p_emb`) to prove
    //     end-to-end embedding ingest + InferenceTicket::verify + submit_policy + epoch change stop,
    //     then SupervisedFirstRun live run appends qualification ledger OFF-THREAD!
    let (scene_id, tool_id) = sandbox.active_scene_and_tool().await;
    let mut gate_emb = make_release_gate_json(
        &_emb_dig,
        "supervised_first_run",
        20,
        0.25,
        &scene_id,
        &tool_id,
        EMPTY_CALIBRATION_DIGEST,
    );
    gate_emb["encoder_artifact_sha256"] = json!(enc_sha);
    gate_emb["preprocess_config_sha256"] = json!(pre_sha);
    let dig_qual = write_dm_rmlp_pack(
        dir.path(),
        "p_qual",
        dm_bias_from_home(0.03, 0.0),
        json!({}),
    );
    let gate_qual = make_release_gate_json(
        &dig_qual,
        "supervised_first_run",
        20,
        0.25,
        &scene_id,
        &tool_id,
        EMPTY_CALIBRATION_DIGEST,
    );
    let mut ws_req = tokio_tungstenite::tungstenite::client::IntoClientRequest::into_client_request(
        format!("ws://{addr}/api/v1/ws"),
    )
    .unwrap();
    ws_req.headers_mut().insert(
        "Authorization",
        format!("Bearer {op_token}").parse().unwrap(),
    );
    let (mut ws, _) = connect_async(ws_req).await.unwrap();
    let g = acquire(&mut ws, "y5-op", false, 1).await;
    let ep = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;
    arm_via_ws(&mut ws, g, ep, 1).await;

    // Run `p_emb` with fresh embedding; verify live steps execute, then invalidate producer epoch
    // via a fresh clock handshake so `InferenceTicket` / stale guard stops the run cleanly.
    let now_ms2 = armd::policy_runtime::vision::monotonic_now_ms() as i64;
    let ing2 = client
        .post(format!("http://{addr}/api/v1/vision/embedding"))
        .header("Authorization", format!("Bearer {op_token}"))
        .json(&json!({
            "handshake_generation": hs_gen,
            "producer_epoch": 1,
            "frame_id": 2,
            "captured_at_ms": now_ms2,
            "oldest_captured_at_ms": now_ms2,
            "embedding": [0.1, 0.2, 0.3, 0.4],
            "encoder_artifact_sha256": enc_sha,
            "preprocess_config_sha256": pre_sha
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(ing2.status(), 200);
    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/action/policy/start",
            "body": {
                "policy_id": "p_emb",
                "mode": "live",
                "fps": 20.0,
                "max_seconds": 1.5,
                "i_am_at_the_arm": true,
                "release_gate": gate_emb
            },
            "generation": g,
            "epoch_id": ep,
            "request_id": 20,
        }),
    )
    .await;
    let ack_emb = next_of_any(&mut ws, &["command_ack", "command_reject"]).await;
    assert_eq!(ack_emb["type"], "command_ack", "p_emb start rejected: {ack_emb}");
    let aid_emb = ack_emb["action_id"].as_u64().unwrap();
    wait_until(Duration::from_secs(2), || {
        sup.hub().latest().is_some_and(|s| {
            s.obs.policy.as_ref().is_some_and(|p| {
                p.action_id == aid_emb && p.active && p.steps >= 1 && p.vision_frame_id == Some(2)
            })
        })
    })
    .await;
    // Switch producer_epoch -> clears embedding buffer & invalidates InferenceTicket -> VISION_STALE stop.
    let _hs_gen2 = confirm_clock_handshake(&client, addr, &op_token, 2).await;
    wait_until(Duration::from_secs(2), || {
        sup.hub().latest().is_some_and(|s| {
            s.obs.policy.as_ref().is_some_and(|p| p.action_id == aid_emb && !p.active)
        })
    })
    .await;
    let emb_stop_reason = sup
        .hub()
        .latest()
        .unwrap()
        .obs
        .policy
        .as_ref()
        .and_then(|p| p.stop_reason);
    assert!(
        matches!(
            emb_stop_reason.as_deref(),
            Some("HANDSHAKE_EXPIRED") | Some("VISION_STALE") | Some("VISION_TICKET_INVALID")
        ),
        "expected HANDSHAKE_EXPIRED / VISION_STALE / VISION_TICKET_INVALID after producer_epoch switch, got {emb_stop_reason:?}"
    );
    // Let braking decelerate commanded velocities back to 0 before starting `p_qual`.
    tokio::time::sleep(Duration::from_millis(120)).await;

    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/action/policy/start",
            "body": {
                "policy_id": "p_qual",
                "mode": "live",
                "fps": 20.0,
                "max_seconds": 0.30,
                "i_am_at_the_arm": true,
                "release_gate": gate_qual
            },
            "generation": g,
            "epoch_id": ep,
            "request_id": 21,
        }),
    )
    .await;
    let ack_q = next_of_any(&mut ws, &["command_ack", "command_reject"]).await;
    assert_eq!(ack_q["type"], "command_ack", "p_qual start rejected: {ack_q}");
    let aid_q = ack_q["action_id"].as_u64().unwrap();
    wait_until(Duration::from_secs(3), || {
        ledger_path.exists()
            && fs::read_to_string(&ledger_path)
                .map(|s| s.contains(&format!("\"action_id\":{aid_q}")))
                .unwrap_or(false)
    })
    .await;
    let recs = armd::policy_runtime::qualification::load_records(&ledger_path).unwrap();
    let rec_q = recs
        .iter()
        .find(|r| r.action_id == aid_q)
        .expect("p_qual qualification record");
    assert!(
        rec_q.passed,
        "supervised MAX_SECONDS run must record passed=true: {rec_q:?}"
    );

    // Let `p_qual` braking decelerate commanded velocities back to 0 before starting the delayed load.
    wait_until(Duration::from_secs(2), || {
        sup.hub().latest().is_some_and(|s| {
            s.obs.policy.as_ref().is_some_and(|p| p.action_id == aid_q && !p.active)
        })
    })
    .await;
    tokio::time::sleep(Duration::from_millis(120)).await;

    // (d) Completion-time operator lock during load delay -> BeginPolicy refused OPERATOR_REQUIRED!
    sandbox.set_policy_load_delay_ms(180);
    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/action/policy/start",
            "body": {
                "policy_id": "p_qual",
                "mode": "live",
                "fps": 20.0,
                "max_seconds": 0.50,
                "i_am_at_the_arm": true,
                "release_gate": gate_qual
            },
            "generation": g,
            "epoch_id": ep,
            "request_id": 22,
        }),
    )
    .await;
    tokio::time::sleep(Duration::from_millis(40)).await;
    let lock_res = client
        .post(format!("http://{addr}/api/v1/auth/lock"))
        .header("Authorization", format!("Bearer {op_token}"))
        .send()
        .await
        .unwrap();
    assert_eq!(lock_res.status(), 200);
    let rej_lock = next_of(&mut ws, "command_reject").await;
    assert_eq!(
        rej_lock["code"], "OPERATOR_REQUIRED",
        "locking operator session during policy load must refuse completion: {rej_lock}"
    );

    assert_eq!(
        sup.inner().control_thread_attempts(),
        0,
        "armd-control must record 0 SupervisorLock attempts"
    );
    drop(_detached_writer);
    handle.stop_and_join();
}

// ============================================================================
// 6. Zero-alloc (TimedAlloc) & zero-SupervisorLock proof on armd-control across
//    full Shadow + Live + Revoke + Stop + Busy + E-stop + Disconnect lifecycle
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn y6_executor_policy_zero_alloc_and_zero_supervisor_lock_proof() {
    ensure_tracing_subscriber();
    let dir = tempdir().unwrap();
    let sandbox = Arc::new(SandboxState::new(dir.path().to_path_buf()));
    let cfg = ArmConfig {
        lease_timeout_ms: 10_000,
        ..ArmConfig::rebot_dm()
    };
    let sup = Arc::new(ArmSupervisor::new(cfg));
    sup.set_operator_auth_enforced(false).await;

    let mut exec_cfg = ExecutorConfig::for_tests(Duration::from_millis(10));
    exec_cfg.alloc_timing = true;
    let delay_hook: Arc<Mutex<Option<Duration>>> = Arc::new(Mutex::new(None));
    let handle = sup
        .start_executor_with_hooks(
            exec_cfg,
            PersistTestHooks {
                release_delay_once: Some(delay_hook.clone()),
                ..Default::default()
            },
        )
        .await
        .unwrap();
    wait_until(Duration::from_secs(2), || sup.hub().latest().is_some()).await;

    let addr = serve(sup.clone(), sandbox.clone(), OperatorAuth::disabled()).await;

    // Hold detached legacy SupervisorLock write guard across the ENTIRE policy lifecycle!
    let _detached_writer = sup.inner().write().await;

    write_dm_rmlp_pack(
        dir.path(),
        "p_zero",
        dm_bias_from_home(0.05, 0.0),
        json!({}),
    );

    let mut ws = connect_ws(addr).await;
    let gen = acquire(&mut ws, "y6-zero", false, 1).await;
    let ep = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;
    arm_via_ws(&mut ws, gen, ep, 1).await;

    // Warm up 2 releases before recording baseline alloc counters (so startup prewarm is settled).
    let base_rel = handle.stats().completed_releases.load(std::sync::atomic::Ordering::Acquire) + 2;
    wait_until(Duration::from_secs(1), || {
        handle.stats().completed_releases.load(std::sync::atomic::Ordering::Acquire) >= base_rel
    })
    .await;
    let rel_allocs_before = handle.stats().release_allocs.snapshot();
    let wake_allocs_before = handle.stats().wake_allocs.snapshot();

    // 1. Shadow run -> MAX_SECONDS
    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/action/policy/start",
            "body": {"policy_id": "p_zero", "mode": "shadow", "fps": 20.0, "max_seconds": 0.25},
            "generation": gen,
            "epoch_id": ep,
            "request_id": 2,
        }),
    )
    .await;
    let ack_s = next_of(&mut ws, "command_ack").await;
    let aid_s = ack_s["action_id"].as_u64().unwrap();
    wait_until(Duration::from_secs(2), || {
        sup.hub().latest().is_some_and(|s| s.obs.policy.as_ref().is_some_and(|p| p.action_id == aid_s && !p.active))
    })
    .await;

    // 2. Live run + steady steps + Submitted::Busy + revocation + WS policy/stop
    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/action/policy/start",
            "body": {"policy_id": "p_zero", "mode": "live", "fps": 25.0, "max_seconds": 5.0},
            "generation": gen,
            "epoch_id": ep,
            "request_id": 3,
        }),
    )
    .await;
    let ack_l = next_of(&mut ws, "command_ack").await;
    let aid_l = ack_l["action_id"].as_u64().unwrap();
    wait_until(Duration::from_secs(2), || {
        sup.hub().latest().is_some_and(|s| s.obs.policy.as_ref().is_some_and(|p| p.action_id == aid_l && p.steps >= 3))
    })
    .await;
    // Pause a release briefly while runner submits -> provokes Submitted::Busy!
    *delay_hook.lock().unwrap() = Some(Duration::from_millis(85));
    tokio::time::sleep(Duration::from_millis(120)).await;
    // Revoke authorization mid-run, wait 2 releases, then explicit stop.
    sup.rt_policy_authorization().revoke();
    let seq_h = sup.hub().published_seq() + 3;
    wait_until(Duration::from_secs(2), || sup.hub().published_seq() >= seq_h).await;
    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/action/policy/stop",
            "body": {},
            "generation": gen,
            "epoch_id": ep,
            "request_id": 4,
        }),
    )
    .await;
    let _ = next_of(&mut ws, "command_ack").await;
    wait_until(Duration::from_secs(1), || handle.is_policy_closed(aid_l)).await;

    // 3. Live run + E-stop + disconnect
    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/action/policy/start",
            "body": {"policy_id": "p_zero", "mode": "live", "fps": 20.0, "max_seconds": 5.0},
            "generation": gen,
            "epoch_id": ep,
            "request_id": 5,
        }),
    )
    .await;
    let ack_e = next_of(&mut ws, "command_ack").await;
    let aid_e = ack_e["action_id"].as_u64().unwrap();
    wait_until(Duration::from_secs(2), || {
        sup.hub().latest().is_some_and(|s| s.obs.policy.as_ref().is_some_and(|p| p.action_id == aid_e && p.steps >= 1))
    })
    .await;
    let _ = sup.trigger_e_stop_checked().await.unwrap();
    wait_until(Duration::from_secs(2), || {
        sup.hub().latest().is_some_and(|s| s.obs.policy.as_ref().is_some_and(|p| p.action_id == aid_e && !p.active))
    })
    .await;
    drop(ws);
    let seq_end = sup.hub().published_seq() + 3;
    wait_until(Duration::from_secs(1), || sup.hub().published_seq() >= seq_end).await;

    let rel_allocs_after = handle.stats().release_allocs.snapshot();
    let wake_allocs_after = handle.stats().wake_allocs.snapshot();
    let rel_delta = armd::diagnostics::alloc::AllocCounts {
        alloc: rel_allocs_after.alloc.saturating_sub(rel_allocs_before.alloc),
        alloc_zeroed: rel_allocs_after
            .alloc_zeroed
            .saturating_sub(rel_allocs_before.alloc_zeroed),
        realloc: rel_allocs_after
            .realloc
            .saturating_sub(rel_allocs_before.realloc),
        dealloc: rel_allocs_after
            .dealloc
            .saturating_sub(rel_allocs_before.dealloc),
    };
    let wake_delta = armd::diagnostics::alloc::AllocCounts {
        alloc: wake_allocs_after.alloc.saturating_sub(wake_allocs_before.alloc),
        alloc_zeroed: wake_allocs_after
            .alloc_zeroed
            .saturating_sub(wake_allocs_before.alloc_zeroed),
        realloc: wake_allocs_after
            .realloc
            .saturating_sub(wake_allocs_before.realloc),
        dealloc: wake_allocs_after
            .dealloc
            .saturating_sub(wake_allocs_before.dealloc),
    };
    assert_eq!(
        rel_delta.total(),
        0,
        "armd-control whole releases must perform ZERO heap alloc/realloc/free across policy lifecycle: {rel_delta:?}"
    );
    assert_eq!(
        wake_delta.total(),
        0,
        "armd-control sub-release wakes must perform ZERO heap alloc/realloc/free: {wake_delta:?}"
    );
    assert_eq!(
        sup.inner().control_thread_attempts(),
        0,
        "armd-control must perform ZERO SupervisorLock attempts"
    );

    drop(_detached_writer);
    handle.stop_and_join();
}

// ============================================================================
// 7. Independent-review regression coverage (Blocker 1 + Major 4 end-to-end):
//    - Stopping a supervised live run records `passed=false / STOPPED` (never
//      synthesizes a false pass) and blocks `production_release` (`QUALIFICATION_REQUIRED`)
//    - Stopping during a delayed policy load cancels promptly and frees backend permits
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn y7_review_regressions_qualification_stop_and_load_cancel() {
    ensure_tracing_subscriber();
    let dir = tempdir().unwrap();
    let ledger_path = dir.path().join("qual_y7.jsonl");
    let sandbox = Arc::new(SandboxState::new(dir.path().to_path_buf()));
    let cfg = ArmConfig {
        lease_timeout_ms: 10_000,
        ..ArmConfig::rebot_dm()
    };
    let (sup, _bus) = make_fakebus_supervisor(cfg, false);
    {
        let mut inner = sup.inner().write().await;
        inner.policy_ledger_path = Some(ledger_path.clone());
    }
    let auth = OperatorAuth::enforced("1234", 600);
    sup.set_operator_auth_enforced(true).await;

    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(10)))
        .await
        .unwrap();
    wait_until(Duration::from_secs(2), || sup.hub().latest().is_some()).await;
    let addr = serve(sup.clone(), sandbox.clone(), auth).await;

    let client = reqwest::Client::new();
    let unlock: Value = client
        .post(format!("http://{addr}/api/v1/auth/unlock"))
        .json(&json!({"pin": "1234"}))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let op_token = unlock["token"].as_str().expect("operator token").to_string();

    let mut ws_req = tokio_tungstenite::tungstenite::client::IntoClientRequest::into_client_request(
        format!("ws://{addr}/api/v1/ws"),
    )
    .unwrap();
    ws_req.headers_mut().insert(
        "Authorization",
        format!("Bearer {op_token}").parse().unwrap(),
    );
    let (mut ws, _) = connect_async(ws_req).await.unwrap();
    let g = acquire(&mut ws, "y7-client", false, 1).await;
    let ep = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;
    arm_via_ws(&mut ws, g, ep, 1).await;

    let (scene_id, tool_id) = sandbox.active_scene_and_tool().await;
    let dig = write_dm_rmlp_pack(
        dir.path(),
        "p_y7",
        dm_bias_from_home(0.03, 0.0),
        json!({}),
    );
    let gate_sup = make_release_gate_json(
        &dig,
        "supervised_first_run",
        20,
        0.25,
        &scene_id,
        &tool_id,
        EMPTY_CALIBRATION_DIGEST,
    );
    let gate_prod = make_release_gate_json(
        &dig,
        "production_release",
        20,
        0.25,
        &scene_id,
        &tool_id,
        EMPTY_CALIBRATION_DIGEST,
    );

    // Part A (Blocker 1): Start a SupervisedFirstRun live policy, stop it mid-run, and verify:
    //  1. Ledger records `passed == false` with `stop_reason == "STOPPED"` (never optimistic pass).
    //  2. Subsequent `production_release` start is refused with `QUALIFICATION_REQUIRED`.
    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/action/policy/start",
            "body": {
                "policy_id": "p_y7",
                "mode": "live",
                "fps": 20.0,
                "max_seconds": 5.0,
                "i_am_at_the_arm": true,
                "release_gate": gate_sup
            },
            "generation": g,
            "epoch_id": ep,
            "request_id": 10,
        }),
    )
    .await;
    let ack = next_of_any(&mut ws, &["command_ack", "command_reject"]).await;
    assert_eq!(ack["type"], "command_ack", "y7 supervised start failed: {ack}");
    let aid = ack["action_id"].as_u64().unwrap();
    wait_until(Duration::from_secs(2), || {
        sup.hub().latest().is_some_and(|s| {
            s.obs
                .policy
                .as_ref()
                .is_some_and(|p| p.action_id == aid && p.active && p.steps >= 1)
        })
    })
    .await;

    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/action/policy/stop",
            "body": {},
            "generation": g,
            "epoch_id": ep,
            "request_id": 11,
        }),
    )
    .await;
    let stop_ack = next_of(&mut ws, "command_ack").await;
    assert_eq!(stop_ack["stopped"], true, "{stop_ack}");

    wait_until(Duration::from_secs(3), || {
        ledger_path.exists()
            && fs::read_to_string(&ledger_path)
                .map(|s| s.contains(&format!("\"action_id\":{aid}")))
                .unwrap_or(false)
    })
    .await;
    let recs = armd::policy_runtime::qualification::load_records(&ledger_path).unwrap();
    let rec = recs
        .iter()
        .find(|r| r.action_id == aid)
        .expect("stopped supervised run ledger record");
    assert!(
        !rec.passed,
        "stopped supervised run must record passed=false: {rec:?}"
    );
    assert_eq!(
        rec.stop_reason, "STOPPED",
        "stopped supervised run must preserve immutable owner stop_reason: {rec:?}"
    );

    // ProductionRelease MUST be refused `QUALIFICATION_REQUIRED`!
    tokio::time::sleep(Duration::from_millis(120)).await;
    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/action/policy/start",
            "body": {
                "policy_id": "p_y7",
                "mode": "live",
                "fps": 20.0,
                "max_seconds": 0.5,
                "i_am_at_the_arm": true,
                "release_gate": gate_prod
            },
            "generation": g,
            "epoch_id": ep,
            "request_id": 12,
        }),
    )
    .await;
    let prod_rej = next_of(&mut ws, "command_reject").await;
    assert_eq!(
        prod_rej["code"], "QUALIFICATION_REQUIRED",
        "production_release after stopped supervised run must fail closed: {prod_rej}"
    );

    // Part B (Major 4): Stop cancels an in-flight delayed blocking policy load promptly and
    // releases its backend permit so a subsequent start succeeds cleanly.
    sandbox.set_policy_load_delay_ms(600);
    let t0 = Instant::now();
    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/action/policy/start",
            "body": {
                "policy_id": "p_y7",
                "mode": "shadow",
                "fps": 20.0,
                "max_seconds": 0.5
            },
            "generation": g,
            "epoch_id": ep,
            "request_id": 13,
        }),
    )
    .await;
    tokio::time::sleep(Duration::from_millis(30)).await;
    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/action/policy/stop",
            "body": {},
            "generation": g,
            "epoch_id": ep,
            "request_id": 14,
        }),
    )
    .await;
    let mut got_load_reject = false;
    let mut got_stop_ack = false;
    while !(got_load_reject && got_stop_ack) {
        let msg = next_of_any(&mut ws, &["command_ack", "command_reject"]).await;
        if msg["request_id"] == 13 {
            assert_eq!(msg["type"], "command_reject", "cancelled load must reject: {msg}");
            assert_eq!(msg["code"], "STOPPED", "cancelled load must return STOPPED: {msg}");
            got_load_reject = true;
        } else if msg["request_id"] == 14 {
            assert_eq!(msg["type"], "command_ack", "policy/stop must ack: {msg}");
            got_stop_ack = true;
        }
    }
    let elapsed = t0.elapsed();
    assert!(
        elapsed < Duration::from_millis(400),
        "stopping a 600ms delayed load must cancel promptly (took {elapsed:?})"
    );

    // Permit freed -> immediate subsequent Shadow start succeeds!
    sandbox.set_policy_load_delay_ms(0);
    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/action/policy/start",
            "body": {
                "policy_id": "p_y7",
                "mode": "shadow",
                "fps": 20.0,
                "max_seconds": 0.25
            },
            "generation": g,
            "epoch_id": ep,
            "request_id": 15,
        }),
    )
    .await;
    let ack_after = next_of_any(&mut ws, &["command_ack", "command_reject"]).await;
    assert_eq!(
        ack_after["type"], "command_ack",
        "start after cancelled load must succeed once permit is released: {ack_after}"
    );
    let aid_after = ack_after["action_id"].as_u64().unwrap();
    wait_until(Duration::from_secs(2), || {
        sup.hub().latest().is_some_and(|s| {
            s.obs
                .policy
                .as_ref()
                .is_some_and(|p| p.action_id == aid_after && !p.active)
        })
    })
    .await;

    handle.stop_and_join();
}

// ============================================================================
// 8. Action chunking on OWNER_EXECUTOR:
//    - Open-loop horizon > 1000 ms refused at WS policy/start (`INVALID_POLICY`)
//    - Chunked RMLP (`chunk_size=4, replan_every=2, temporal_ensemble_k=0.5`) runs
//      Live on FakeBus with ZERO RT alloc/realloc/free, ZERO SupervisorLock attempts,
//      verified CAN TX motion, and J7 speed cap preserved
// ============================================================================

fn write_dm_chunked_rmlp_pack(
    sandbox_root: &Path,
    policy_id: &str,
    chunk_biases: &[[f32; 7]],
    extra_manifest: Value,
) -> String {
    let dir = sandbox_root.join("policies").join(policy_id);
    fs::create_dir_all(&dir).unwrap();
    let joint_names = [
        "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "gripper",
    ];
    let chunk_size = chunk_biases.len();
    let out_dim = chunk_size * 7;
    let mut flat_bias = Vec::with_capacity(out_dim);
    for step in chunk_biases {
        flat_bias.extend_from_slice(step);
    }
    let layers = vec![MlpLayerSpec::new(
        7,
        out_dim,
        Activation::None,
        vec![0.0; 7 * out_dim],
        flat_bias,
    )];
    let rmlp = encode_rmlp_bytes(&layers).unwrap();
    let mut manifest = json!({
        "format": "rebot-onnx-policy/v1",
        "name": policy_id,
        "unit_mode": "rad",
        "action_dim": 7,
        "action_space": "joint_positions",
        "joint_names": joint_names,
        "model_file": "model.rmlp",
        "outputs": {
            "action": {
                "name": "action",
                "shape": [1, out_dim]
            }
        }
    });
    if let (Some(m), Some(e)) = (manifest.as_object_mut(), extra_manifest.as_object()) {
        for (k, v) in e {
            m.insert(k.clone(), v.clone());
        }
    }
    let pj = serde_json::to_vec_pretty(&manifest).unwrap();
    fs::write(dir.join("policy.json"), &pj).unwrap();
    fs::write(dir.join("model.rmlp"), &rmlp).unwrap();
    compute_release_digest(&pj, &rmlp, "model.rmlp")
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn y8_executor_action_chunking_horizon_live_tx_and_zero_alloc_proof() {
    ensure_tracing_subscriber();
    let dir = tempdir().unwrap();
    let sandbox = Arc::new(SandboxState::new(dir.path().to_path_buf()));
    let cfg = ArmConfig {
        lease_timeout_ms: 10_000,
        ..ArmConfig::rebot_dm()
    };
    let (sup, bus) = make_fakebus_supervisor(cfg, true);
    sup.set_operator_auth_enforced(false).await;

    let mut exec_cfg = ExecutorConfig::for_tests(Duration::from_millis(10));
    exec_cfg.alloc_timing = true;
    let handle = sup.start_executor(exec_cfg).await.unwrap();
    assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);
    wait_until(Duration::from_secs(2), || sup.hub().latest().is_some()).await;

    let addr = serve(sup.clone(), sandbox.clone(), OperatorAuth::disabled()).await;

    // Hold detached legacy SupervisorLock write guard across the entire chunked policy lifecycle!
    let _detached_writer = sup.inner().write().await;

    let mut ws = connect_ws(addr).await;
    let gen = acquire(&mut ws, "y8-chunk", false, 1).await;
    let ep = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;
    arm_via_ws(&mut ws, gen, ep, 1).await;

    let settle_seq = sup.hub().published_seq() + 4;
    wait_until(Duration::from_secs(1), || sup.hub().published_seq() >= settle_seq).await;
    let cur = sup.hub().latest().unwrap().obs.commanded_positions.clone();

    // (a) Open-loop horizon > 1000 ms (replan_every=24 at 20 fps -> 1200 ms) MUST be refused
    //     at WS `/api/v1/action/policy/start` on OWNER_EXECUTOR with 422 `INVALID_POLICY`.
    let mut base_q = [0.0f32; 7];
    base_q.copy_from_slice(&cur);
    let over_horizon_biases = vec![base_q; 24];
    write_dm_chunked_rmlp_pack(
        dir.path(),
        "p_chunk_over_horizon",
        &over_horizon_biases,
        json!({
            "chunking": {
                "chunk_size": 24,
                "replan_every": 24
            }
        }),
    );
    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/action/policy/start",
            "body": {
                "policy_id": "p_chunk_over_horizon",
                "mode": "shadow",
                "fps": 20.0,
                "max_seconds": 0.5
            },
            "generation": gen,
            "epoch_id": ep,
            "request_id": 2,
        }),
    )
    .await;
    let rej_h = next_of_any(&mut ws, &["command_ack", "command_reject"]).await;
    assert_eq!(rej_h["type"], "command_reject", "horizon > 1000 ms must be rejected: {rej_h}");
    assert_eq!(rej_h["status"], 422, "{rej_h}");
    assert_eq!(rej_h["code"], "INVALID_POLICY", "{rej_h}");

    // (b) Live chunked RMLP (`chunk_size=4, replan_every=2, temporal_ensemble_k=0.5`) with
    //     valid supervised release gate: proves CAN TX advances J1/J7 under zero-alloc RT.
    let mut step0 = base_q;
    let mut step1 = base_q;
    let mut step2 = base_q;
    let mut step3 = base_q;
    // Distinct per-slot targets: slot 0 only moves +0.01 rad so exceeding +0.04 rad on CAN TX
    // proves cached / ensembled later slots (not repeated slot 0) reached the motor bus.
    step0[0] += 0.01;
    step1[0] += 0.06;
    step2[0] += 0.10;
    step3[0] += 0.13;
    step0[6] = (base_q[6] + 0.06).min(0.0);
    step1[6] = (base_q[6] + 0.08).min(0.0);
    step2[6] = (base_q[6] + 0.10).min(0.0);
    step3[6] = (base_q[6] + 0.10).min(0.0);

    let dig = write_dm_chunked_rmlp_pack(
        dir.path(),
        "p_chunk_live",
        &[step0, step1, step2, step3],
        json!({
            "chunking": {
                "chunk_size": 4,
                "replan_every": 2,
                "temporal_ensemble_k": 0.5
            }
        }),
    );
    let (scene_id, tool_id) = sandbox.active_scene_and_tool().await;
    let gate = make_release_gate_json(
        &dig,
        "supervised_first_run",
        20,
        0.25,
        &scene_id,
        &tool_id,
        EMPTY_CALIBRATION_DIGEST,
    );

    // Baseline RT alloc counters after warmup releases.
    let base_rel = handle.stats().completed_releases.load(std::sync::atomic::Ordering::Acquire) + 2;
    wait_until(Duration::from_secs(1), || {
        handle.stats().completed_releases.load(std::sync::atomic::Ordering::Acquire) >= base_rel
    })
    .await;
    let rel_allocs_before = handle.stats().release_allocs.snapshot();
    let wake_allocs_before = handle.stats().wake_allocs.snapshot();

    bus.clear_log();
    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/action/policy/start",
            "body": {
                "policy_id": "p_chunk_live",
                "mode": "live",
                "fps": 20.0,
                "max_seconds": 0.35,
                "i_am_at_the_arm": true,
                "release_gate": gate
            },
            "generation": gen,
            "epoch_id": ep,
            "request_id": 3,
        }),
    )
    .await;
    let ack = next_of_any(&mut ws, &["command_ack", "command_reject"]).await;
    assert_eq!(ack["type"], "command_ack", "live chunked policy start failed: {ack}");
    let aid = ack["action_id"].as_u64().expect("chunked action_id");
    wait_until(Duration::from_secs(3), || {
        sup.hub().latest().is_some_and(|s| {
            s.obs
                .policy
                .as_ref()
                .is_some_and(|p| p.action_id == aid && !p.active)
        })
    })
    .await;
    let snap_done = sup.hub().latest().unwrap();
    let pol_done = snap_done.obs.policy.as_ref().unwrap();
    assert_eq!(pol_done.stop_reason.as_deref(), Some("MAX_SECONDS"), "{pol_done:?}");
    assert!(pol_done.steps >= 3, "expected >= 3 live chunked steps, got {}", pol_done.steps);

    let final_target = pol_done
        .last_shadow_target
        .as_ref()
        .expect("live chunked run must record committed target");
    assert!(
        final_target[0] > base_q[0] + 0.05,
        "chunked live policy must emit later/ensembled slots past slot 0 (+0.01): final_target[0]={}, base={}",
        final_target[0],
        base_q[0]
    );

    let max_j1_cmd = bus
        .tx_log()
        .iter()
        .filter(|f| f.id == 0x101 && f.data.len() >= 4)
        .map(decode_pos_f32)
        .fold(f32::NEG_INFINITY, f32::max);
    assert!(
        max_j1_cmd > base_q[0] + 0.025,
        "chunked live policy on executor must advance J1 CAN TX past slot 0 (+0.01) into later/ensembled chunk slots (max_j1_cmd={max_j1_cmd}, base={})",
        base_q[0]
    );

    let j7_vel_raws: Vec<u16> = bus
        .tx_log()
        .iter()
        .filter(|f| f.id == 0x307 && f.data.len() >= 6)
        .map(decode_j7_pf_vel_raw)
        .collect();
    assert!(!j7_vel_raws.is_empty(), "chunked live policy must emit J7 PosForce frames");
    let max_j7_vel_raw = *j7_vel_raws.iter().max().unwrap();
    assert!(
        max_j7_vel_raw <= 250,
        "J7 PosForce velocity raw must remain capped under release gate max_speed_scale=0.25 (got {max_j7_vel_raw})"
    );

    let rel_allocs_after = handle.stats().release_allocs.snapshot();
    let wake_allocs_after = handle.stats().wake_allocs.snapshot();
    let rel_delta = armd::diagnostics::alloc::AllocCounts {
        alloc: rel_allocs_after.alloc.saturating_sub(rel_allocs_before.alloc),
        alloc_zeroed: rel_allocs_after
            .alloc_zeroed
            .saturating_sub(rel_allocs_before.alloc_zeroed),
        realloc: rel_allocs_after
            .realloc
            .saturating_sub(rel_allocs_before.realloc),
        dealloc: rel_allocs_after
            .dealloc
            .saturating_sub(rel_allocs_before.dealloc),
    };
    let wake_delta = armd::diagnostics::alloc::AllocCounts {
        alloc: wake_allocs_after.alloc.saturating_sub(wake_allocs_before.alloc),
        alloc_zeroed: wake_allocs_after
            .alloc_zeroed
            .saturating_sub(wake_allocs_before.alloc_zeroed),
        realloc: wake_allocs_after
            .realloc
            .saturating_sub(wake_allocs_before.realloc),
        dealloc: wake_allocs_after
            .dealloc
            .saturating_sub(wake_allocs_before.dealloc),
    };
    assert_eq!(
        rel_delta.total(),
        0,
        "armd-control whole releases must perform ZERO heap alloc/realloc/free for chunked live policy: {rel_delta:?}"
    );
    assert_eq!(
        wake_delta.total(),
        0,
        "armd-control sub-release wakes must perform ZERO heap alloc/realloc/free for chunked live policy: {wake_delta:?}"
    );
    assert_eq!(
        sup.inner().control_thread_attempts(),
        0,
        "armd-control must perform ZERO SupervisorLock attempts during chunked live policy"
    );

    drop(_detached_writer);
    handle.stop_and_join();
}
