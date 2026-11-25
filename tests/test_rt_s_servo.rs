//! Stage 3 PR S (C1) integration, FakeBus TX, J7 PosForce, zero-alloc, and lock-proof test suite:
//! - Authorize committed lease generation with `authorize_servo_stream` (including after E-stop clear)
//! - Revoke at the owner on release, preemption, expiry, and disconnect BEFORE target consumption
//! - Dispatch `close_servo(old_generation)` off-thread and reject late submissions from that generation
//! - Route WS `servo`, `cartesian_joint_stream`, implicit `positions` shorthand, and `/api/v1/motion/servo`
//!   through `TargetPort::submit_servo`, preserving ingress time, sequence rules, and reply correlation
//! - Keep REST `/api/v1/motion/servo` lease restriction and legacy direct path
//! - Buffered target across disconnect -> preemption -> fresh target from replacement owner: zero old-gen output
//! - J7 PosForce servo frames set J7 intent stamped from admission and go through `check_pos_force_gripper_fresh`
//! - `armd-control`: zero allocation (`TimedAlloc`) and zero `SupervisorLock` attempts.

mod common;
mod mit_support;

#[global_allocator]
static ALLOC: armd::diagnostics::alloc::TimedAlloc = armd::diagnostics::alloc::TimedAlloc;

use std::net::SocketAddr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

use futures_util::StreamExt;
use serde_json::{json, Value};
use tokio::net::TcpListener;
use tokio_tungstenite::{connect_async, tungstenite::Message};
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

use armd::api::create_router;
use armd::auth::OperatorAuth;
use armd::driver::{ArmDriver, DamiaoArmDriver};
use armd::protocol::damiao::DamiaoLimits;
use armd::rt_core::executor::{ExecutorConfig, PersistTestHooks};
use armd::supervisor::OWNER_EXECUTOR;
use armd::transport::{CanFrame, MockClock};
use armd::{ArmConfig, ArmSupervisor, SafetyState};
use common::{next_of as next_of_any, send, TestSocket};
use mit_support::FakeBus;

async fn next_of(ws: &mut TestSocket, kind: &str) -> Value {
    next_of_any(ws, &[kind]).await
}

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
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    assert!(pred(), "condition not satisfied within {timeout:?}");
}

async fn serve(sup: Arc<ArmSupervisor>) -> SocketAddr {
    sup.set_operator_auth_enforced(false).await;
    let router = create_router(sup, OperatorAuth::disabled());
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        axum::serve(listener, router).await.unwrap();
    });
    addr
}

async fn connect_ws(addr: SocketAddr) -> TestSocket {
    connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap().0
}

async fn next_non_telemetry_within(ws: &mut TestSocket, dur: Duration) -> Option<Value> {
    let deadline = tokio::time::Instant::now() + dur;
    loop {
        let rem = deadline.saturating_duration_since(tokio::time::Instant::now());
        if rem.is_zero() {
            return None;
        }
        match tokio::time::timeout(rem, ws.next()).await {
            Ok(Some(Ok(Message::Text(text)))) => {
                if let Ok(v) = serde_json::from_str::<Value>(&text) {
                    if v.get("type").is_some() {
                        return Some(v);
                    }
                }
            }
            Ok(Some(Ok(_))) => continue,
            _ => return None,
        }
    }
}

async fn acquire(ws: &mut TestSocket, client_id: &str, force: bool, seq: u64) -> u64 {
    let mut frame = json!({"type": "acquire_lease", "client_id": client_id, "seq": seq});
    if force {
        frame["force"] = json!(true);
    }
    send(ws, frame).await;
    let rep = next_of(ws, "lease_granted").await;
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
    let ack = next_of(ws, "command_ack").await;
    assert_eq!(ack["request_id"], req_id, "{ack}");
    assert_eq!(ack["is_torqued"], true, "{ack}");
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

// ============================================================================
// 1. Spec core scenario: buffered target across disconnect, then preemption,
//    followed by a fresh target from the replacement owner; ZERO old-gen output.
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn buffered_target_across_disconnect_and_preemption_then_replacement_owner_zero_old_gen_output() {
    let cfg = ArmConfig {
        lease_timeout_ms: 5_000,
        ..ArmConfig::rebot_dm()
    };
    let (sup, bus) = make_fakebus_supervisor(cfg, false);
    let addr = serve(sup.clone()).await;

    let delay: Arc<Mutex<Option<Duration>>> = Arc::new(Mutex::new(None));
    let handle = sup
        .start_executor_with_hooks(
            ExecutorConfig::for_tests(Duration::from_millis(5)),
            PersistTestHooks {
                release_delay_once: Some(delay.clone()),
                ..Default::default()
            },
        )
        .await
        .unwrap();
    assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);
    wait_until(Duration::from_secs(2), || sup.hub().latest().is_some()).await;

    // --- Phase A: Buffered servo target across DISCONNECT ---
    let mut ws1 = connect_ws(addr).await;
    let gen1 = acquire(&mut ws1, "owner-1", false, 1).await;
    let ep1 = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;
    arm_via_ws(&mut ws1, gen1, ep1, 1).await;
    // Let a few armed-hold releases settle and establish baseline q0.
    let seq_settled = sup.hub().published_seq() + 4;
    wait_until(Duration::from_secs(1), || sup.hub().published_seq() >= seq_settled).await;
    let q0 = sup.fresh_snapshot(None).await.unwrap().obs.commanded_positions[0];

    // Pause one release on armd-control so ws1's servo target sits buffered in the mailbox.
    *delay.lock().unwrap() = Some(Duration::from_millis(70));
    wait_until(Duration::from_secs(1), || delay.lock().unwrap().is_none()).await;
    bus.clear_log();

    let mut q_old1 = vec![0.0f32; 7];
    q_old1[0] = q0 + 0.55;
    send(
        &mut ws1,
        json!({
            "type": "servo",
            "generation": gen1,
            "epoch_id": ep1,
            "sequence_id": 1,
            "positions": q_old1,
        }),
    )
    .await;
    // Give ws1's worker a brief moment to push the target into the triple buffer while RT is paused.
    tokio::time::sleep(Duration::from_millis(12)).await;
    // Disconnect ws1 BEFORE the paused release unblocks.
    drop(ws1);

    // Wait for disconnect fencing, off-thread `close_servo(gen1)`, and several RT releases.
    wait_until(Duration::from_secs(2), || {
        handle.is_servo_closed(gen1) && !sup.hub().latest().unwrap().owner.is_lease_active_at(Instant::now())
    })
    .await;
    let seq_after_disc = sup.hub().published_seq() + 5;
    wait_until(Duration::from_secs(1), || sup.hub().published_seq() >= seq_after_disc).await;

    let snap_after_disc = sup.fresh_snapshot(None).await.unwrap();
    assert!(
        (snap_after_disc.obs.commanded_positions[0] - q0).abs() < 1e-3,
        "disconnect leaked old-gen target into commanded_positions: got {}, baseline {q0}",
        snap_after_disc.obs.commanded_positions[0]
    );
    let tx_disc = bus.tx_log();
    for f in tx_disc.iter().filter(|f| f.id == 0x101) {
        let p = decode_pos_f32(f);
        assert!(
            (p - q0).abs() < 1e-3,
            "disconnect leaked old-gen J1 CAN TX frame: pos {p} != baseline {q0}"
        );
    }

    // --- Phase B: Buffered servo target across PREEMPTION ---
    let mut ws2 = connect_ws(addr).await;
    let gen2 = acquire(&mut ws2, "owner-2", false, 1).await;
    assert!(gen2 > gen1);
    let ep2 = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;
    let mut ws3 = connect_ws(addr).await;

    // Pause one release so ws2's servo target and ws3's force preemption meet in the same release
    // (Step 3 LeasePreempt MUST revoke gen2 BEFORE Step 4 consumes ws2's buffered target!).
    *delay.lock().unwrap() = Some(Duration::from_millis(70));
    wait_until(Duration::from_secs(1), || delay.lock().unwrap().is_none()).await;
    bus.clear_log();

    let mut q_old2 = vec![0.0f32; 7];
    q_old2[0] = q0 - 0.55;
    send(
        &mut ws2,
        json!({
            "type": "servo",
            "generation": gen2,
            "epoch_id": ep2,
            "sequence_id": 1,
            "positions": q_old2,
        }),
    )
    .await;
    tokio::time::sleep(Duration::from_millis(10)).await;

    // Replacement owner preempts while the paused release is still in flight.
    send(
        &mut ws3,
        json!({"type": "acquire_lease", "client_id": "owner-3", "force": true, "seq": 1}),
    )
    .await;
    let granted3 = next_of(&mut ws3, "lease_granted").await;
    let gen3 = granted3["generation"].as_u64().unwrap();
    assert!(gen3 > gen2);

    // Evictee ws2 receives `lease_lost(reason: "preempted")`.
    let lost2 = next_of(&mut ws2, "lease_lost").await;
    assert_eq!(lost2["generation"], gen2);
    assert_eq!(lost2["reason"], "preempted");

    wait_until(Duration::from_secs(2), || handle.is_servo_closed(gen2)).await;
    let seq_after_preempt = sup.hub().published_seq() + 5;
    wait_until(Duration::from_secs(1), || sup.hub().published_seq() >= seq_after_preempt).await;

    let snap_after_preempt = sup.fresh_snapshot(None).await.unwrap();
    assert!(
        (snap_after_preempt.obs.commanded_positions[0] - q0).abs() < 1e-3,
        "preemption leaked old-gen target into commanded_positions: got {}, baseline {q0}",
        snap_after_preempt.obs.commanded_positions[0]
    );
    let tx_preempt = bus.tx_log();
    for f in tx_preempt.iter().filter(|f| f.id == 0x101) {
        let p = decode_pos_f32(f);
        assert!(
            (p - q0).abs() < 1e-3,
            "preemption leaked old-gen J1 CAN TX frame: pos {p} != baseline {q0}"
        );
    }

    // Late submission from displaced owner ws2 is rejected immediately off-thread.
    send(
        &mut ws2,
        json!({
            "type": "servo",
            "generation": gen2,
            "epoch_id": ep2,
            "sequence_id": 2,
            "positions": q_old2,
        }),
    )
    .await;
    let late_lost = next_of(&mut ws2, "lease_lost").await;
    assert_eq!(late_lost["generation"], gen2);
    assert_eq!(late_lost["reason"], "preempted");

    // --- Phase C: Fresh target from replacement owner moves the arm ---
    let ep3 = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;
    bus.clear_log();
    let mut q_new = vec![0.0f32; 7];
    q_new[0] = q0 + 0.18;
    send(
        &mut ws3,
        json!({
            "type": "servo",
            "generation": gen3,
            "epoch_id": ep3,
            "sequence_id": 1,
            "positions": q_new,
        }),
    )
    .await;

    wait_until(Duration::from_secs(2), || {
        sup.hub()
            .latest()
            .is_some_and(|s| s.obs.commanded_positions[0] > q0 + 0.05)
    })
    .await;
    let tx_new = bus.tx_log();
    assert!(
        tx_new
            .iter()
            .filter(|f| f.id == 0x101)
            .any(|f| decode_pos_f32(f) > q0 + 0.05),
        "replacement owner's fresh servo target must produce J1 CAN TX motion toward +0.18"
    );

    drop(ws2);
    drop(ws3);
    handle.stop_and_join();
}

// ============================================================================
// 2. All 4 WS servo forms, sequence rules, reply correlation, ingress TTL,
//    and REST /api/v1/motion/servo lease restriction across both owners.
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn all_four_ws_servo_forms_sequence_rules_reply_correlation_and_rest_restriction() {
    for executor_mode in [false, true] {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig {
            lease_timeout_ms: 5_000,
            ..ArmConfig::rebot_dm()
        }));
        let addr = serve(sup.clone()).await;
        let delay: Arc<Mutex<Option<Duration>>> = Arc::new(Mutex::new(None));
        let legacy_stop = Arc::new(AtomicBool::new(false));
        let handle = if executor_mode {
            let h = sup
                .start_executor_with_hooks(
                    ExecutorConfig::for_tests(Duration::from_millis(5)),
                    PersistTestHooks {
                        release_delay_once: Some(delay.clone()),
                        ..Default::default()
                    },
                )
                .await
                .unwrap();
            wait_until(Duration::from_secs(2), || sup.hub().published_seq() >= 2).await;
            let _ = sup.fresh_snapshot(None).await.unwrap();
            Some(h)
        } else {
            let sup_tick = sup.clone();
            let stop_flag = legacy_stop.clone();
            tokio::spawn(async move {
                while !stop_flag.load(Ordering::Acquire) {
                    sup_tick.tick(0.01).await;
                    tokio::time::sleep(Duration::from_millis(10)).await;
                }
            });
            None
        };

        // REST POST /api/v1/motion/servo MUST remain restricted on both owners.
        let http = reqwest::Client::new();
        let rest_resp = http
            .post(format!("http://{addr}/api/v1/motion/servo"))
            .json(&json!({"targets": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}))
            .send()
            .await
            .unwrap();
        let rest_status = rest_resp.status();
        let rest_body: Value = rest_resp.json().await.unwrap();
        assert_eq!(rest_status, reqwest::StatusCode::CONFLICT, "executor={executor_mode} {rest_body}");
        assert_eq!(rest_body["code"], "LEASE_REQUIRED", "executor={executor_mode} {rest_body}");

        let mut ws = connect_ws(addr).await;

        // Pre-lease validation precedence: explicit & cartesian -> missing_generation; implicit -> lease_lost(0).
        send(&mut ws, json!({"type": "servo", "positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "sequence_id": 1})).await;
        assert_eq!(next_of(&mut ws, "servo_reject").await["reason"], "missing_generation");
        send(&mut ws, json!({"mode": "cartesian_joint_stream", "positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "sequence_id": 1})).await;
        assert_eq!(next_of(&mut ws, "servo_reject").await["reason"], "missing_generation");
        send(&mut ws, json!({"positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "sequence_id": 1})).await;
        let ll0 = next_of(&mut ws, "lease_lost").await;
        assert_eq!(ll0["generation"], 0);
        assert!(ll0.get("reason").is_none());

        let gen = acquire(&mut ws, "ctrl", false, 1).await;
        let ep = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;

        // Missing sequence_id.
        send(&mut ws, json!({"type": "servo", "generation": gen, "epoch_id": ep, "positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]})).await;
        assert_eq!(next_of(&mut ws, "servo_reject").await["reason"], "missing_sequence_id");

        // Stale epoch -> epoch_sync.
        send(&mut ws, json!({"type": "servo", "generation": gen, "epoch_id": 999_999, "sequence_id": 1, "positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]})).await;
        assert_eq!(next_of(&mut ws, "epoch_sync").await["epoch_id"], ep);

        // Disarmed streaming servo (seq=1) -> async NOT_ARMED servo_reject AND advances sequence!
        send(&mut ws, json!({"type": "servo", "generation": gen, "epoch_id": ep, "sequence_id": 1, "positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]})).await;
        let disarmed_rej = next_of(&mut ws, "servo_reject").await;
        assert_eq!(disarmed_rej["code"], "NOT_ARMED", "executor={executor_mode} {disarmed_rej}");
        assert_eq!(disarmed_rej["status"], 412, "executor={executor_mode} {disarmed_rej}");

        // Arm via WS.
        arm_via_ws(&mut ws, gen, ep, 1).await;

        // Repeating seq=1 fails with invalid_sequence_id (proving seq=1 advanced on NOT_ARMED).
        send(&mut ws, json!({"type": "servo", "generation": gen, "epoch_id": ep, "sequence_id": 1, "positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]})).await;
        assert_eq!(next_of(&mut ws, "servo_reject").await["reason"], "invalid_sequence_id");

        // Form 1: explicit `type: "servo"` (seq=2).
        send(&mut ws, json!({"type": "servo", "generation": gen, "epoch_id": ep, "sequence_id": 2, "positions": [0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]})).await;
        assert!(next_non_telemetry_within(&mut ws, Duration::from_millis(60)).await.is_none());
        wait_until(Duration::from_secs(1), || {
            sup.hub().latest().is_some_and(|s| s.obs.commanded_positions[0] > 0.02)
        })
        .await;

        // Form 2: `mode: "cartesian_joint_stream"` (seq=3).
        send(&mut ws, json!({"mode": "cartesian_joint_stream", "generation": gen, "epoch_id": ep, "sequence_id": 3, "positions": [0.08, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]})).await;
        assert!(next_non_telemetry_within(&mut ws, Duration::from_millis(60)).await.is_none());
        wait_until(Duration::from_secs(1), || {
            sup.hub().latest().is_some_and(|s| s.obs.commanded_positions[0] > 0.06)
        })
        .await;

        // Form 3: implicit `positions` shorthand (seq=4).
        send(&mut ws, json!({"generation": gen, "epoch_id": ep, "sequence_id": 4, "positions": [0.12, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]})).await;
        assert!(next_non_telemetry_within(&mut ws, Duration::from_millis(60)).await.is_none());
        wait_until(Duration::from_secs(1), || {
            sup.hub().latest().is_some_and(|s| s.obs.commanded_positions[0] > 0.10)
        })
        .await;

        // Form 4: WS command `/api/v1/motion/servo` with `request_id = 2` -> correlated `command_ack`.
        send(
            &mut ws,
            json!({
                "type": "command",
                "path": "/api/v1/motion/servo",
                "body": {"targets": [0.16, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                "generation": gen,
                "epoch_id": ep,
                "request_id": 2,
            }),
        )
        .await;
        let cmd_ack = next_of(&mut ws, "command_ack").await;
        assert_eq!(cmd_ack["request_id"], 2, "executor={executor_mode} {cmd_ack}");
        assert_eq!(cmd_ack["status"], "accepted", "executor={executor_mode} {cmd_ack}");
        assert_eq!(cmd_ack["safety_state"], "Moving", "executor={executor_mode} {cmd_ack}");
        assert_eq!(cmd_ack["is_torqued"], true, "executor={executor_mode} {cmd_ack}");
        assert_eq!(cmd_ack["control_mode"], "position", "executor={executor_mode} {cmd_ack}");
        wait_until(Duration::from_secs(1), || {
            sup.hub().latest().is_some_and(|s| s.obs.commanded_positions[0] > 0.14)
        })
        .await;

        // `/api/v1/motion/servo` must NOT disturb streaming `sequence_id` (seq=5 ok, jump 2000 rejected).
        send(&mut ws, json!({"type": "servo", "generation": gen, "epoch_id": ep, "sequence_id": 5, "positions": [0.16, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]})).await;
        assert!(next_non_telemetry_within(&mut ws, Duration::from_millis(50)).await.is_none());
        send(&mut ws, json!({"type": "servo", "generation": gen, "epoch_id": ep, "sequence_id": 2000, "positions": [0.16, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]})).await;
        assert_eq!(next_of(&mut ws, "servo_reject").await["reason"], "invalid_sequence_id");

        // `/api/v1/motion/servo` error correlation: duplicate request_id, stale epoch, soft limit.
        send(&mut ws, json!({"type": "command", "path": "/api/v1/motion/servo", "body": {"targets": [0.16, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}, "generation": gen, "epoch_id": ep, "request_id": 2})).await;
        let dup_rej = next_of(&mut ws, "command_reject").await;
        assert_eq!(dup_rej["request_id"], 2);
        assert_eq!(dup_rej["code"], "INVALID_COMMAND");

        send(&mut ws, json!({"type": "command", "path": "/api/v1/motion/servo", "body": {"targets": [0.16, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}, "generation": gen, "epoch_id": 999_999, "request_id": 3})).await;
        let ep_rej = next_of(&mut ws, "command_reject").await;
        assert_eq!(ep_rej["request_id"], 3);
        assert_eq!(ep_rej["code"], "EPOCH_MISMATCH");

        send(&mut ws, json!({"type": "command", "path": "/api/v1/motion/servo", "body": {"targets": [99.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}, "generation": gen, "epoch_id": ep, "request_id": 4})).await;
        let lim_rej = next_of(&mut ws, "command_reject").await;
        assert_eq!(lim_rej["request_id"], 4);
        assert_eq!(lim_rej["code"], "SOFT_LIMIT_EXCEEDED");

        // Executor-only: ingress timestamp preserved across delay > SERVO_TARGET_TTL (200 ms) -> DEADLINE_EXPIRED.
        if executor_mode {
            let before_ttl = sup.fresh_snapshot(None).await.unwrap().obs.commanded_positions[0];
            *delay.lock().unwrap() = Some(Duration::from_millis(240));
            wait_until(Duration::from_secs(1), || delay.lock().unwrap().is_none()).await;
            send(
                &mut ws,
                json!({
                    "type": "command",
                    "path": "/api/v1/motion/servo",
                    "body": {"targets": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                    "generation": gen,
                    "epoch_id": ep,
                    "request_id": 5,
                }),
            )
            .await;
            let ttl_rej = next_of(&mut ws, "command_reject").await;
            assert_eq!(ttl_rej["request_id"], 5, "{ttl_rej}");
            assert_eq!(ttl_rej["code"], "DEADLINE_EXPIRED", "{ttl_rej}");
            let after_ttl = sup.fresh_snapshot(None).await.unwrap().obs.commanded_positions[0];
            assert!((after_ttl - before_ttl).abs() < 1e-2, "expired frame must not move arm");
        }

        drop(ws);
        legacy_stop.store(true, Ordering::Release);
        if let Some(h) = handle {
            h.stop_and_join();
        }
    }
}

// ============================================================================
// 3. Release & expiry revoke before target intake, > MAX_STREAMS close_servo
//    slot reclamation, and E-stop acquire -> clear_fault authorization.
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn release_expiry_close_servo_slot_reclamation_and_estop_clear_auth() {
    // 3a. Release & > MAX_STREAMS (12 sequential leases) + E-stop clear auth.
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig {
            lease_timeout_ms: 5_000,
            ..ArmConfig::rebot_dm()
        }));
        let addr = serve(sup.clone()).await;
        let delay: Arc<Mutex<Option<Duration>>> = Arc::new(Mutex::new(None));
        let handle = sup
            .start_executor_with_hooks(
                ExecutorConfig::for_tests(Duration::from_millis(5)),
                PersistTestHooks {
                    release_delay_once: Some(delay.clone()),
                    ..Default::default()
                },
            )
            .await
            .unwrap();
        wait_until(Duration::from_secs(2), || sup.hub().latest().is_some()).await;

        let mut ws = connect_ws(addr).await;
        let gen1 = acquire(&mut ws, "rel-ctrl", false, 1).await;
        let ep1 = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;
        arm_via_ws(&mut ws, gen1, ep1, 1).await;
        let q0 = sup.fresh_snapshot(None).await.unwrap().obs.commanded_positions[0];

        // Buffer a large servo target and release the lease while one release is paused:
        // Step 3 LeaseRelease revokes BEFORE Step 4 consumes the target.
        *delay.lock().unwrap() = Some(Duration::from_millis(70));
        wait_until(Duration::from_secs(1), || delay.lock().unwrap().is_none()).await;
        send(&mut ws, json!({"type": "servo", "generation": gen1, "epoch_id": ep1, "sequence_id": 1, "positions": [q0 + 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]})).await;
        send(&mut ws, json!({"type": "release_lease", "generation": gen1})).await;
        let rel = next_of(&mut ws, "lease_released").await;
        assert_eq!(rel["success"], true);
        wait_until(Duration::from_secs(1), || handle.is_servo_closed(gen1)).await;
        let seq_done = sup.hub().published_seq() + 4;
        wait_until(Duration::from_secs(1), || sup.hub().published_seq() >= seq_done).await;
        assert!(
            (sup.fresh_snapshot(None).await.unwrap().obs.commanded_positions[0] - q0).abs() < 1e-3,
            "buffered target across release_lease must produce zero output"
        );

        // 12 sequential lease acquire -> servo -> release cycles (> MAX_STREAMS = 8): zero STREAMS_FULL.
        let mut req_id = 10u64;
        for cycle in 1..=12u64 {
            let g = acquire(&mut ws, &format!("cycle-{cycle}"), false, cycle + 1).await;
            let ep = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;
            req_id += 1;
            let target = 0.01 * (cycle as f32);
            send(
                &mut ws,
                json!({
                    "type": "command",
                    "path": "/api/v1/motion/servo",
                    "body": {"targets": [target, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                    "generation": g,
                    "epoch_id": ep,
                    "request_id": req_id,
                }),
            )
            .await;
            let ack = next_of(&mut ws, "command_ack").await;
            assert_eq!(ack["request_id"], req_id, "cycle {cycle} must not hit STREAMS_FULL: {ack}");
            send(&mut ws, json!({"type": "release_lease", "generation": g})).await;
            assert_eq!(next_of(&mut ws, "lease_released").await["success"], true);
            wait_until(Duration::from_secs(1), || handle.is_servo_closed(g)).await;
        }

        // Acquire lease while E-stop is latched -> clear_fault -> torque_on -> servo succeeds!
        let (_st, _ep_e) = sup.trigger_e_stop_checked().await.unwrap();
        let g_estop = acquire(&mut ws, "estop-ctrl", false, 99).await;
        let ep_e = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;
        req_id += 1;
        send(
            &mut ws,
            json!({
                "type": "command",
                "path": "/api/v1/safety/clear_fault",
                "body": {},
                "generation": g_estop,
                "epoch_id": ep_e,
                "request_id": req_id,
            }),
        )
        .await;
        let clr_ack = next_of(&mut ws, "command_ack").await;
        assert_eq!(clr_ack["request_id"], req_id);
        let ep_clear = clr_ack["epoch_id"].as_u64().unwrap();
        req_id += 1;
        arm_via_ws(&mut ws, g_estop, ep_clear, req_id).await;
        req_id += 1;
        send(
            &mut ws,
            json!({
                "type": "command",
                "path": "/api/v1/motion/servo",
                "body": {"targets": [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                "generation": g_estop,
                "epoch_id": ep_clear,
                "request_id": req_id,
            }),
        )
        .await;
        let post_estop_ack = next_of(&mut ws, "command_ack").await;
        assert_eq!(post_estop_ack["request_id"], req_id, "{post_estop_ack}");
        assert_eq!(post_estop_ack["status"], "accepted");

        drop(ws);
        handle.stop_and_join();
    }

    // 3b. Lease expiry revokes before target intake and dispatches `close_servo(old_gen)` off-thread.
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig {
            lease_timeout_ms: 140,
            ..ArmConfig::rebot_dm()
        }));
        let addr = serve(sup.clone()).await;
        let delay: Arc<Mutex<Option<Duration>>> = Arc::new(Mutex::new(None));
        let handle = sup
            .start_executor_with_hooks(
                ExecutorConfig::for_tests(Duration::from_millis(5)),
                PersistTestHooks {
                    release_delay_once: Some(delay.clone()),
                    ..Default::default()
                },
            )
            .await
            .unwrap();
        wait_until(Duration::from_secs(2), || sup.hub().latest().is_some()).await;

        let mut ws = connect_ws(addr).await;
        let gen = acquire(&mut ws, "exp-ctrl", false, 1).await;
        let ep = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;
        arm_via_ws(&mut ws, gen, ep, 1).await;
        let q0 = sup.fresh_snapshot(None).await.unwrap().obs.commanded_positions[0];

        // Pause one release past lease_timeout (140 ms), buffer a target ~50ms before expiry
        // (so its 200ms TTL is STILL valid when the release unblocks, while the lease IS expired).
        *delay.lock().unwrap() = Some(Duration::from_millis(160));
        wait_until(Duration::from_secs(1), || delay.lock().unwrap().is_none()).await;
        tokio::time::sleep(Duration::from_millis(60)).await;
        send(&mut ws, json!({"type": "servo", "generation": gen, "epoch_id": ep, "sequence_id": 1, "positions": [q0 + 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]})).await;

        // Expiry revokes BEFORE target consumption and dispatches `close_servo(gen)` off-thread.
        wait_until(Duration::from_secs(2), || handle.is_servo_closed(gen)).await;
        let seq_done = sup.hub().published_seq() + 4;
        wait_until(Duration::from_secs(1), || sup.hub().published_seq() >= seq_done).await;
        assert!(
            (sup.fresh_snapshot(None).await.unwrap().obs.commanded_positions[0] - q0).abs() < 1e-3,
            "buffered target across expiry must produce zero output"
        );
        // Late submission after expiry is rejected off-thread with lease_lost(expired).
        send(&mut ws, json!({"type": "servo", "generation": gen, "epoch_id": ep, "sequence_id": 2, "positions": [q0 + 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]})).await;
        let ll = next_of(&mut ws, "lease_lost").await;
        assert_eq!(ll["generation"], gen);
        assert_eq!(ll["reason"], "expired");

        drop(ws);
        handle.stop_and_join();
    }
}

// ============================================================================
// 4. J7 PosForce servo frames: admission-stamped intent only + stale J7 gate.
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn j7_pos_force_servo_intent_admission_stamp_and_stale_freshness_gate() {
    let cfg = ArmConfig {
        lease_timeout_ms: 5_000,
        ..ArmConfig::rebot_dm()
    };
    let limits = dm_limits();
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&limits, clock.clone());
    bus.with(|s| {
        s.motors[6].set_reg_u32(
            armd::protocol::damiao::RID_CTRL_MODE,
            armd::protocol::damiao::CTRL_MODE_POS_FORCE,
        );
        s.motors[6].q = -3.0;
        s.immediate_replies = true;
        s.follow_commands = true;
        s.push_all_feedback();
    });
    let grip = armd::driver::gripper_drive_from_profile(&cfg).expect("pos-force gripper drive");
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap()
        .with_gripper_drive(grip)
        .unwrap()
        .with_tx_budget(cfg.damiao_tx_budget());
    drv.read_joint_states().unwrap();
    drv.read_joint_states().unwrap();
    drv.check_mapping_ranges();

    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
    let addr = serve(sup.clone()).await;

    let delay: Arc<Mutex<Option<Duration>>> = Arc::new(Mutex::new(None));
    let handle = sup
        .start_executor_with_hooks(
            ExecutorConfig::for_tests(Duration::from_millis(5)),
            PersistTestHooks {
                release_delay_once: Some(delay.clone()),
                ..Default::default()
            },
        )
        .await
        .unwrap();
    wait_until(Duration::from_secs(2), || sup.hub().latest().is_some()).await;

    let mut ws = connect_ws(addr).await;
    let gen = acquire(&mut ws, "j7-ctrl", false, 1).await;
    let ep = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;
    arm_via_ws(&mut ws, gen, ep, 1).await;
    bus.with(|s| s.push_all_feedback());
    let seq_arm = sup.hub().published_seq() + 4;
    wait_until(Duration::from_secs(1), || sup.hub().published_seq() >= seq_arm).await;

    // 4a. Fresh J7 PosForce servo moves J7 via admission-stamped J7 intent (0x307 frames).
    bus.clear_log();
    let mut q_j7 = sup.fresh_snapshot(None).await.unwrap().obs.commanded_positions.clone();
    q_j7[6] = -2.0;
    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/motion/servo",
            "body": {"targets": q_j7},
            "generation": gen,
            "epoch_id": ep,
            "request_id": 2,
        }),
    )
    .await;
    let ack = next_of(&mut ws, "command_ack").await;
    assert_eq!(ack["request_id"], 2, "{ack}");
    wait_until(Duration::from_secs(2), || {
        bus.with(|s| s.push_all_feedback());
        bus.tx_log()
            .iter()
            .filter(|f| f.id == 0x307)
            .any(|f| decode_pos_f32(f) > -2.9 && decode_j7_pf_vel_raw(f) > 0)
    })
    .await;

    // 4b. Urgent Stop while a J7 servo frame is buffered: ZERO stale J7 target (-0.5) on wire.
    *delay.lock().unwrap() = Some(Duration::from_millis(60));
    wait_until(Duration::from_secs(1), || delay.lock().unwrap().is_none()).await;
    bus.clear_log();
    let mut q_stop = sup.fresh_snapshot(None).await.unwrap().obs.commanded_positions.clone();
    q_stop[6] = -0.5;
    send(
        &mut ws,
        json!({
            "type": "servo",
            "generation": gen,
            "epoch_id": ep,
            "sequence_id": 1,
            "positions": q_stop,
        }),
    )
    .await;
    tokio::time::sleep(Duration::from_millis(10)).await;
    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/motion/stop",
            "body": {},
            "generation": gen,
            "epoch_id": ep,
            "request_id": 3,
        }),
    )
    .await;
    let stop_ack = next_of(&mut ws, "command_ack").await;
    assert_eq!(stop_ack["request_id"], 3, "{stop_ack}");
    let seq_stop = sup.hub().published_seq() + 4;
    wait_until(Duration::from_secs(1), || sup.hub().published_seq() >= seq_stop).await;
    for f in bus.tx_log().iter().filter(|f| f.id == 0x307) {
        let p = decode_pos_f32(f);
        assert!(
            p < -1.0,
            "stopped J7 servo target (-0.5) leaked onto wire: 0x307 pos={p}"
        );
    }

    // 4c. Stale J7 PosForce (> 100 ms, < 250 ms fault): servo moving J1 + J7 routes through
    //     `check_pos_force_gripper_fresh_static`: J1 moves while J7 stays frozen (`vel_raw == 0`),
    //     and after J7 recovers, zero stale J7 target (0.0) reaches the wire!
    let snap_fresh = sup.fresh_snapshot(None).await.unwrap();
    let frozen_j7 = snap_fresh.obs.commanded_positions[6];
    let j1_before = snap_fresh.obs.commanded_positions[0];
    bus.with(|s| {
        s.immediate_replies = false;
        s.motors[6].q = frozen_j7;
        s.motors[6].mute_feedback = true;
        for i in 0..6 {
            s.push_feedback(i);
        }
    });
    // Advance mock driver clock by 120 ms (> 100 ms J7 stale threshold, < 250 ms fault).
    clock.advance(Duration::from_millis(120));
    bus.with(|s| {
        for i in 0..6 {
            s.push_feedback(i);
        }
    });
    let seq_stale = sup.hub().published_seq() + 2;
    wait_until(Duration::from_secs(1), || sup.hub().published_seq() >= seq_stale).await;

    bus.clear_log();
    let mut q_stale = snap_fresh.obs.commanded_positions.clone();
    q_stale[0] = j1_before + 0.08;
    q_stale[6] = 0.0; // Attempt to move J7 while stale!
    send(
        &mut ws,
        json!({
            "type": "command",
            "path": "/api/v1/motion/servo",
            "body": {"targets": q_stale},
            "generation": gen,
            "epoch_id": ep,
            "request_id": 4,
        }),
    )
    .await;
    let stale_ack = next_of(&mut ws, "command_ack").await;
    assert_eq!(stale_ack["request_id"], 4, "{stale_ack}");
    bus.with(|s| {
        for i in 0..6 {
            s.push_feedback(i);
        }
    });
    let seq_after_stale = sup.hub().published_seq() + 3;
    wait_until(Duration::from_secs(1), || sup.hub().published_seq() >= seq_after_stale).await;
    let stale_j7_frames: Vec<_> = bus.tx_log().into_iter().filter(|f| f.id == 0x307).collect();
    assert!(!stale_j7_frames.is_empty(), "expected frozen 0x307 frames while J7 stale");
    for f in &stale_j7_frames {
        assert_eq!(decode_j7_pf_vel_raw(f), 0, "stale J7 must emit freeze (vel_raw == 0)");
        assert!(
            (decode_pos_f32(f) - frozen_j7).abs() < 0.05,
            "stale J7 target leaked on wire: {} vs frozen {frozen_j7}",
            decode_pos_f32(f)
        );
    }

    // Recover J7 feedback: ZERO stale J7 target (0.0) may leak after recovery!
    bus.clear_log();
    bus.with(|s| {
        s.immediate_replies = true;
        s.motors[6].mute_feedback = false;
        s.motors[6].q = frozen_j7;
        s.push_all_feedback();
    });
    let seq_rec = sup.hub().published_seq() + 4;
    wait_until(Duration::from_secs(1), || {
        bus.with(|s| s.push_all_feedback());
        sup.hub().published_seq() >= seq_rec
    })
    .await;
    let rec_j7_frames: Vec<_> = bus.tx_log().into_iter().filter(|f| f.id == 0x307).collect();
    assert!(!rec_j7_frames.is_empty(), "expected 0x307 frames after J7 recovery");
    for f in &rec_j7_frames {
        assert!(
            (decode_pos_f32(f) - frozen_j7).abs() < 0.05,
            "stale-admitted J7 target 0.0 leaked after recovery: {}",
            decode_pos_f32(f)
        );
    }

    drop(ws);
    handle.stop_and_join();
}

// ============================================================================
// 5. Zero-allocation (`TimedAlloc`) and zero `SupervisorLock` proof on
//    `armd-control` across WS servo streaming, `/api/v1/motion/servo`,
//    disconnect, preemption, and replacement owner while legacy lock is held.
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_servo_zero_alloc_and_zero_supervisor_lock_proof() {
    ensure_tracing_subscriber();
    let sup = Arc::new(ArmSupervisor::new(ArmConfig {
        lease_timeout_ms: 5_000,
        ..ArmConfig::rebot_dm()
    }));
    let addr = serve(sup.clone()).await;

    let mut ecfg = ExecutorConfig::for_tests(Duration::from_millis(5));
    ecfg.alloc_timing = true;
    let handle = sup.start_executor(ecfg).await.unwrap();
    assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);
    wait_until(Duration::from_secs(2), || sup.hub().latest().is_some()).await;

    // Hold the detached legacy `SupervisorLock` write guard continuously across the entire
    // servo + disconnect + preemption + replacement sequence to prove zero lock touches on RT!
    let _legacy_write_hold = sup.inner().write().await;

    let rel_before = handle.stats().release_allocs.snapshot();
    let wake_before = handle.stats().wake_allocs.snapshot();
    let lock_before = sup.inner().control_thread_attempts();

    // Owner 1: acquire -> arm -> burst of 15 streaming servo frames -> /motion/servo -> disconnect.
    let mut ws1 = connect_ws(addr).await;
    let gen1 = acquire(&mut ws1, "alloc-1", false, 1).await;
    let ep1 = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;
    arm_via_ws(&mut ws1, gen1, ep1, 1).await;
    for s in 1..=15u64 {
        let q = 0.004 * (s as f32);
        send(
            &mut ws1,
            json!({
                "type": "servo",
                "generation": gen1,
                "epoch_id": ep1,
                "sequence_id": s,
                "positions": [q, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            }),
        )
        .await;
    }
    send(
        &mut ws1,
        json!({
            "type": "command",
            "path": "/api/v1/motion/servo",
            "body": {"targets": [0.08, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
            "generation": gen1,
            "epoch_id": ep1,
            "request_id": 2,
        }),
    )
    .await;
    assert_eq!(next_of(&mut ws1, "command_ack").await["request_id"], 2);
    drop(ws1);
    wait_until(Duration::from_secs(2), || handle.is_servo_closed(gen1)).await;

    // Owner 2: acquire -> stream 10 frames -> preempted by Owner 3 -> Owner 3 streams & moves.
    let mut ws2 = connect_ws(addr).await;
    let gen2 = acquire(&mut ws2, "alloc-2", false, 1).await;
    let ep2 = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;
    for s in 1..=10u64 {
        send(
            &mut ws2,
            json!({
                "mode": "cartesian_joint_stream",
                "generation": gen2,
                "epoch_id": ep2,
                "sequence_id": s,
                "positions": [0.08 + 0.002 * (s as f32), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            }),
        )
        .await;
    }
    let mut ws3 = connect_ws(addr).await;
    let gen3 = acquire(&mut ws3, "alloc-3", true, 1).await;
    assert!(gen3 > gen2);
    assert_eq!(next_of(&mut ws2, "lease_lost").await["reason"], "preempted");
    wait_until(Duration::from_secs(2), || handle.is_servo_closed(gen2)).await;

    // Wait out handover settling if ws2 was actively moving at preemption, then move with ws3.
    tokio::time::sleep(Duration::from_millis(320)).await;
    let ep3 = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;
    let q_before_ws3 = sup.fresh_snapshot(None).await.unwrap().obs.commanded_positions[0];
    send(
        &mut ws3,
        json!({
            "type": "command",
            "path": "/api/v1/motion/servo",
            "body": {"targets": [q_before_ws3 + 0.08, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
            "generation": gen3,
            "epoch_id": ep3,
            "request_id": 1,
        }),
    )
    .await;
    assert_eq!(next_of(&mut ws3, "command_ack").await["request_id"], 1);
    for s in 1..=12u64 {
        send(
            &mut ws3,
            json!({
                "type": "servo",
                "generation": gen3,
                "epoch_id": ep3,
                "sequence_id": s,
                "positions": [q_before_ws3 + 0.08, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            }),
        )
        .await;
        tokio::time::sleep(Duration::from_millis(15)).await;
    }
    wait_until(Duration::from_secs(2), || {
        sup.hub()
            .latest()
            .is_some_and(|s| s.obs.commanded_positions[0] > q_before_ws3 + 0.04)
    })
    .await;

    let rel_delta = handle.stats().release_allocs.snapshot().delta_since(rel_before);
    let wake_delta = handle.stats().wake_allocs.snapshot().delta_since(wake_before);
    let lock_after = sup.inner().control_thread_attempts();

    assert_eq!(
        lock_after - lock_before,
        0,
        "armd-control must perform ZERO SupervisorLock attempts during servo/disconnect/preempt"
    );
    assert_eq!(
        rel_delta.total(),
        0,
        "armd-control must perform ZERO heap ops in whole releases during servo lifecycle: {rel_delta:?}"
    );
    assert_eq!(
        wake_delta.total(),
        0,
        "armd-control must perform ZERO heap ops in sub-release wakes during servo lifecycle: {wake_delta:?}"
    );
    assert_eq!(
        sup.fresh_snapshot(None).await.unwrap().obs.safety_state,
        SafetyState::Moving
    );

    drop(ws2);
    drop(ws3);
    drop(_legacy_write_hold);
    handle.stop_and_join();
}
