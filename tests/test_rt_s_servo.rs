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
        tokio::time::sleep(Duration::from_millis(1)).await;
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
        // Keep snapshot `valid_until` (2 / control_frequency_hz = 100 ms) wider than the
        // one-shot 60 ms release pause so ws3's `preempt_lease_ws` hits the fresh snapshot
        // immediately and queues `LeasePreempt` into the EXACT same paused release as ws2's
        // buffered servo frame.
        control_frequency_hz: 20,
        ..ArmConfig::rebot_dm()
    };
    let (sup, bus) = make_fakebus_supervisor(cfg, false);
    let addr = serve(sup.clone()).await;

    let delay: Arc<Mutex<Option<Duration>>> = Arc::new(Mutex::new(None));
    let handle = sup
        .start_executor_with_hooks(
            ExecutorConfig::for_tests(Duration::from_millis(10)),
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
    let sub_before_disc = handle.servo_submitted_count();
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
    // Deterministically prove ws1's target entered the triple-buffer mailbox while RT is paused.
    wait_until(Duration::from_secs(1), || {
        handle.servo_submitted_count() > sub_before_disc
    })
    .await;
    // Disconnect ws1 BEFORE the paused release unblocks.
    drop(ws1);

    // Wait for disconnect fencing, off-thread `close_servo(gen1)`, and several RT releases.
    wait_until(Duration::from_secs(2), || {
        handle.is_servo_closed(gen1) && !sup.hub().latest().unwrap().owner.is_lease_active_at(Instant::now())
    })
    .await;
    let seq_after_disc = sup.hub().published_seq() + 5;
    wait_until(Duration::from_secs(1), || sup.hub().published_seq() >= seq_after_disc).await;

    assert!(
        handle.servo_lease_refused_count(gen1) >= 1,
        "expected owner-side lease refusal for buffered gen1 target across disconnect"
    );
    let snap_after_disc = sup.fresh_snapshot(None).await.unwrap();
    assert!(
        (snap_after_disc.obs.commanded_positions[0] - q0).abs() < 1e-3,
        "disconnect leaked old-gen target into commanded_positions: got {}, baseline {q0}",
        snap_after_disc.obs.commanded_positions[0]
    );
    let tx_disc = bus.tx_log();
    let j1_tx_disc: Vec<_> = tx_disc.iter().filter(|f| f.id == 0x101).collect();
    assert!(
        !j1_tx_disc.is_empty(),
        "expected nonempty J1 hold CAN TX frames after disconnect"
    );
    for f in j1_tx_disc {
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
    // Wait until ws3 has completed server-side `register_ws_connection` and entered its read loop
    // (proven by receiving its first server message) BEFORE pausing armd-control!
    let _ = tokio::time::timeout(Duration::from_secs(2), ws3.next())
        .await
        .expect("ws3 initial server message")
        .expect("ws3 stream open")
        .expect("ws3 frame ok");

    // Pause one release so ws2's servo target and ws3's force preemption meet in the same release
    // (Step 3 LeasePreempt MUST revoke gen2 BEFORE Step 4 consumes ws2's buffered target!).
    *delay.lock().unwrap() = Some(Duration::from_millis(60));
    wait_until(Duration::from_secs(1), || delay.lock().unwrap().is_none()).await;
    bus.clear_log();

    let mut q_old2 = vec![0.0f32; 7];
    q_old2[0] = q0 - 0.55;
    let sub_before_preempt = handle.servo_submitted_count();
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
    wait_until(Duration::from_secs(1), || {
        handle.servo_submitted_count() > sub_before_preempt
    })
    .await;

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

    assert!(
        handle.servo_lease_refused_count(gen2) >= 1,
        "expected owner-side lease refusal for buffered gen2 target across preemption"
    );
    let snap_after_preempt = sup.fresh_snapshot(None).await.unwrap();
    assert!(
        (snap_after_preempt.obs.commanded_positions[0] - q0).abs() < 1e-3,
        "preemption leaked old-gen target into commanded_positions: got {}, baseline {q0}",
        snap_after_preempt.obs.commanded_positions[0]
    );
    let tx_preempt = bus.tx_log();
    let j1_tx_preempt: Vec<_> = tx_preempt.iter().filter(|f| f.id == 0x101).collect();
    assert!(
        !j1_tx_preempt.is_empty(),
        "expected nonempty J1 hold CAN TX frames after preemption"
    );
    for f in j1_tx_preempt {
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
    // Out-of-soft-limit refusal on the RT path must also allocate zero heap memory.
    send(
        &mut ws1,
        json!({
            "type": "command",
            "path": "/api/v1/motion/servo",
            "body": {"targets": [99.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
            "generation": gen1,
            "epoch_id": ep1,
            "request_id": 3,
        }),
    )
    .await;
    let oob_rej = next_of(&mut ws1, "command_reject").await;
    assert_eq!(oob_rej["request_id"], 3);
    assert_eq!(oob_rej["status"], 400);
    assert_eq!(oob_rej["code"], "SOFT_LIMIT_EXCEEDED");
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

// ============================================================================
// 6. Round 2 B1: Lease fence at the TX choke point (BOTH Legacy and Executor):
//    Close the client socket right between the output check (`mit.rs:892`) and
//    driver TX -> ZERO old-gen motion frames on CAN (J1-J6 + J7 PosForce),
//    stationary holds continue, and Disable frames remain exempt.
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn b1_lease_fence_at_tx_choke_point_both_owners_and_disable_exempt() {
    use armd::supervisor::OWNER_LEGACY;
    use std::sync::atomic::AtomicU64;

    for use_executor in [false, true] {
        let cfg = ArmConfig {
            lease_timeout_ms: 5_000,
            control_frequency_hz: 50,
            ..ArmConfig::rebot_dm()
        };
        let (sup, bus) = make_fakebus_supervisor(cfg, true);

        let arm_hook = Arc::new(AtomicBool::new(false));
        let hook_fired = Arc::new(AtomicBool::new(false));
        let expected_gen = Arc::new(AtomicU64::new(0));
        let close_tx_slot: Arc<Mutex<Option<tokio::sync::oneshot::Sender<()>>>> =
            Arc::new(Mutex::new(None));

        {
            let arm_hook_cb = Arc::clone(&arm_hook);
            let hook_fired_cb = Arc::clone(&hook_fired);
            let expected_gen_cb = Arc::clone(&expected_gen);
            let close_tx_cb = Arc::clone(&close_tx_slot);
            let sup_cb = Arc::clone(&sup);
            sup.inner().write().await.post_stop_check_hook = Some(Arc::new(move || {
                if !arm_hook_cb.load(Ordering::Acquire) {
                    return;
                }
                if hook_fired_cb.swap(true, Ordering::SeqCst) {
                    return;
                }
                if let Some(tx) = close_tx_cb.lock().unwrap().take() {
                    let _ = tx.send(());
                }
                let target_gen = expected_gen_cb.load(Ordering::Acquire);
                let deadline = Instant::now() + Duration::from_secs(1);
                while sup_cb.atomic_fenced_lease_gen() < target_gen && Instant::now() < deadline {
                    std::thread::sleep(Duration::from_millis(1));
                }
            }));
        }

        let addr = serve(sup.clone()).await;
        let delay: Arc<Mutex<Option<Duration>>> = Arc::new(Mutex::new(None));
        let exec_handle = if use_executor {
            let h = sup
                .start_executor_with_hooks(
                    ExecutorConfig::for_tests(Duration::from_millis(10)),
                    PersistTestHooks {
                        release_delay_once: Some(delay.clone()),
                        ..Default::default()
                    },
                )
                .await
                .unwrap();
            assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);
            wait_until(Duration::from_secs(2), || sup.hub().latest().is_some()).await;
            Some(h)
        } else {
            assert_eq!(sup.owner_mode(), OWNER_LEGACY);
            sup.control_tick(0.02).await;
            None
        };

        let mut ws1 = connect_ws(addr).await;
        let gen1 = acquire(&mut ws1, "b1-owner", false, 1).await;
        expected_gen.store(gen1, Ordering::Release);
        let ep1 = if use_executor {
            sup.fresh_snapshot(None).await.unwrap().obs.epoch_id
        } else {
            sup.get_observation().await.epoch_id
        };
        arm_via_ws(&mut ws1, gen1, ep1, 1).await;

        if use_executor {
            let seq = sup.hub().published_seq() + 4;
            wait_until(Duration::from_secs(1), || sup.hub().published_seq() >= seq).await;
        } else {
            for _ in 0..4 {
                sup.control_tick(0.02).await;
            }
        }

        let baseline = if use_executor {
            sup.fresh_snapshot(None)
                .await
                .unwrap()
                .obs
                .commanded_positions
                .clone()
        } else {
            sup.get_observation().await.commanded_positions
        };
        let q0 = baseline[0];
        let j7_0 = baseline[6];

        // Queue a moving servo target for BOTH J1 (`q0 + 0.45`) and J7 (`0.0`, away from `-3.0`).
        let mut q_move = baseline.clone();
        q_move[0] = q0 + 0.45;
        q_move[6] = 0.0;

        if use_executor {
            *delay.lock().unwrap() = Some(Duration::from_millis(60));
            wait_until(Duration::from_secs(1), || delay.lock().unwrap().is_none()).await;
        }
        let sub_before = exec_handle.as_ref().map_or(0, |h| h.servo_submitted_count());
        send(
            &mut ws1,
            json!({
                "type": "servo",
                "generation": gen1,
                "epoch_id": ep1,
                "sequence_id": 1,
                "positions": q_move,
            }),
        )
        .await;
        if let Some(ref h) = exec_handle {
            wait_until(Duration::from_secs(1), || h.servo_submitted_count() > sub_before).await;
        } else {
            wait_until(Duration::from_secs(1), || {
                sup.inner()
                    .try_read()
                    .ok()
                    .and_then(|g| g.servo_target.as_ref().map(|v| (v[0] - q_move[0]).abs() < 1e-3))
                    .unwrap_or(false)
            })
            .await;
        }

        // Hand `ws1` to a dedicated closer task triggered INSIDE `post_stop_check_hook`.
        let (close_tx, close_rx) = tokio::sync::oneshot::channel::<()>();
        *close_tx_slot.lock().unwrap() = Some(close_tx);
        let closer_task = tokio::spawn(async move {
            let _ = close_rx.await;
            drop(ws1);
        });

        // Arm the hook right before the moving tick runs and clear bus log.
        bus.clear_log();
        arm_hook.store(true, Ordering::Release);

        if use_executor {
            wait_until(Duration::from_secs(2), || {
                hook_fired.load(Ordering::Acquire)
                    && !sup
                        .hub()
                        .latest()
                        .unwrap()
                        .owner
                        .is_lease_active_at(Instant::now())
            })
            .await;
            let seq_after = sup.hub().published_seq() + 5;
            wait_until(Duration::from_secs(1), || sup.hub().published_seq() >= seq_after).await;
        } else {
            // Tick 1 hits `post_stop_check_hook` between `mit.rs:892` and TX: old-gen motion MUST
            // be blocked at the driver TX choke point!
            sup.control_tick(0.02).await;
            assert!(
                hook_fired.load(Ordering::Acquire),
                "Legacy post_stop_check_hook must fire"
            );
            // Subsequent ticks apply the queued disconnect, continue emitting stationary holds,
            // and accumulate K=3 spaced rest-pose samples across >= 25 ms before torque-off.
            for _ in 0..5 {
                tokio::time::sleep(Duration::from_millis(12)).await;
                sup.control_tick(0.02).await;
            }
        }
        let _ = closer_task.await;

        assert!(
            sup.atomic_fenced_lease_gen() >= gen1,
            "use_executor={use_executor}: real WS disconnect must fence gen1"
        );
        let log = bus.tx_log();
        let j1_frames: Vec<_> = log.iter().filter(|f| f.id == 0x101).collect();
        let j7_frames: Vec<_> = log.iter().filter(|f| f.id == 0x307).collect();
        assert!(
            !j1_frames.is_empty(),
            "use_executor={use_executor}: stationary J1 holds must continue after lease loss"
        );
        assert!(
            !j7_frames.is_empty(),
            "use_executor={use_executor}: stationary J7 holds must continue after lease loss"
        );
        for f in &j1_frames {
            let p = decode_pos_f32(f);
            assert!(
                (p - q0).abs() < 1e-3,
                "use_executor={use_executor}: old-gen J1 motion leaked on CAN! pos={p} vs baseline {q0}"
            );
        }
        for f in &j7_frames {
            let p = decode_pos_f32(f);
            let vel = decode_j7_pf_vel_raw(f);
            assert_eq!(
                vel, 0,
                "use_executor={use_executor}: old-gen J7 target leaked on CAN (vel_raw={vel} != 0)"
            );
            assert!(
                (p - j7_0).abs() < 0.05,
                "use_executor={use_executor}: old-gen J7 pos leaked on CAN! pos={p} vs baseline {j7_0}"
            );
        }

        // Verify Disable frames (`TxKind::Disable`) remain EXEMPT after lease loss.
        let mut ws_dis = connect_ws(addr).await;
        let gen_dis = acquire(&mut ws_dis, "b1-disable", false, 1).await;
        let ep_dis = if use_executor {
            sup.fresh_snapshot(None).await.unwrap().obs.epoch_id
        } else {
            sup.get_observation().await.epoch_id
        };
        bus.clear_log();
        send(
            &mut ws_dis,
            json!({
                "type": "command",
                "path": "/api/v1/torque",
                "body": {"enable": false},
                "generation": gen_dis,
                "epoch_id": ep_dis,
                "request_id": 2,
            }),
        )
        .await;
        let dis_ack = next_of(&mut ws_dis, "command_ack").await;
        if use_executor {
            assert_eq!(dis_ack["is_torqued"], false, "{dis_ack}");
        } else {
            // Legacy non-blocking rest-gate wait (R12 B1) may defer across K=3 ticks when the
            // prior control tick just transmitted a hold frame; advance legacy ticks to complete.
            if dis_ack["is_torqued"] != false {
                for _ in 0..10 {
                    tokio::time::sleep(Duration::from_millis(12)).await;
                    sup.control_tick(0.02).await;
                    if !sup.get_observation().await.is_torqued {
                        break;
                    }
                }
            }
            assert!(
                !sup.get_observation().await.is_torqued,
                "legacy owner must complete torque-off after K=3 rest-pose ticks; ack={dis_ack}"
            );
        }
        let disable_frames: Vec<_> = bus
            .tx_log()
            .into_iter()
            .filter(|f| f.data == [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
            .collect();
        assert!(
            !disable_frames.is_empty(),
            "use_executor={use_executor}: Disable frames must transmit after lease loss"
        );

        drop(ws_dis);
        if let Some(h) = exec_handle {
            h.stop_and_join();
        }
    }
}

#[test]
fn b1_pending_tx_queue_drops_fenced_lease_targets_and_allows_hold_and_disable() {
    use armd::driver::{ArmDriverError, JointTarget, TxBatchOutcome, TxBudget};
    use armd::rt_core::urgent::{EstopGate, LeaseGate};
    use std::sync::atomic::AtomicU64;

    let limits = dm_limits();
    let bus = FakeBus::new(&limits);
    bus.with(|s| {
        s.immediate_replies = true;
        s.follow_commands = true;
        s.push_all_feedback();
    });
    let current_lease = Arc::new(AtomicU64::new(7));
    let fenced_lease = Arc::new(AtomicU64::new(0));
    let pending_disc = Arc::new(AtomicU64::new(0));
    let gate = EstopGate::new(Arc::new(AtomicU64::new(0)), Arc::new(AtomicU64::new(0)))
        .with_lease_gate(LeaseGate::new(
            Arc::clone(&current_lease),
            Arc::clone(&fenced_lease),
            Arc::clone(&pending_disc),
        ));

    // Enable under valid max_frames=8, then tighten to 2 frames so a 7-joint batch transmits 2
    // frames and queues 5 in `pending_tx`.
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap()
        .with_tx_budget(TxBudget {
            max_frames: 8,
            max_duration: Duration::from_secs(1),
            frame_max_age: Duration::from_secs(1),
        });
    drv.set_estop_gate(gate);
    drv.read_joint_states().unwrap();
    drv.read_joint_states().unwrap();
    drv.set_torque(true).unwrap();
    drv.set_tx_budget(TxBudget {
        max_frames: 2,
        max_duration: Duration::from_secs(1),
        frame_max_age: Duration::from_secs(1),
    });

    // Queue a deferred motion batch under lease_gen = 7.
    let moving: Vec<JointTarget> = (0..7)
        .map(|i| {
            let mut t = JointTarget::position(0.4 + 0.01 * (i as f32));
            t.velocity = Some(0.5);
            t
        })
        .collect();
    drv.begin_tx_tick();
    drv.set_output_lease_gen(7);
    let first = drv.write_joint_targets_bounded(&moving).unwrap();
    drv.set_output_lease_gen(0);
    assert!(
        matches!(first, TxBatchOutcome::Deferred { pending } if pending > 0),
        "expected deferred pending_tx entries, got {first:?}"
    );

    // Fence lease_gen = 7 lock-free BEFORE draining pending_tx!
    fenced_lease.store(7, Ordering::SeqCst);
    current_lease.store(0, Ordering::SeqCst);
    bus.clear_log();
    drv.begin_tx_tick();
    let drained = drv.drain_pending_tx().unwrap();
    assert!(
        matches!(drained, TxBatchOutcome::Complete),
        "fenced pending_tx entries should drop cleanly, got {drained:?}"
    );
    assert!(
        bus.tx_log().is_empty(),
        "ZERO old-gen deferred target frames may transmit after lease fence: {:?}",
        bus.tx_log()
    );

    // A direct new Target with old lease_gen = 7 is also rejected with `LeaseFenced` and 0 CAN TX.
    drv.begin_tx_tick();
    drv.set_output_lease_gen(7);
    let err = drv.write_joint_targets_bounded(&moving).unwrap_err();
    drv.set_output_lease_gen(0);
    assert!(
        matches!(err, ArmDriverError::LeaseFenced),
        "expected ArmDriverError::LeaseFenced, got {err:?}"
    );
    assert!(bus.tx_log().is_empty(), "fenced write_joint_targets_bounded must emit 0 CAN frames");

    // Exempt: Hold (`write_estop_hold_bounded`) still transmits after lease loss.
    let hold: Vec<JointTarget> = (0..7).map(|_| JointTarget::position(0.0)).collect();
    drv.begin_tx_tick();
    let hold_res = drv.write_estop_hold_bounded(&hold).unwrap();
    assert!(
        matches!(hold_res, TxBatchOutcome::Complete | TxBatchOutcome::Deferred { .. }),
        "hold must remain exempt: {hold_res:?}"
    );
    assert!(
        !bus.tx_log().is_empty(),
        "Hold frames must transmit on CAN even after lease loss"
    );

    // Exempt: Disable (`set_torque(false)` -> `TxKind::DisableRequested`) still transmits after
    // lease loss once the rest-pose gate (K=3 causal rest samples) is satisfied.
    let mut rest_positions = [0.0f32; armd::rt_core::mailbox::MAX_DOF];
    let mut arm_joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
    for j in 0..6 {
        arm_joints[j] = j;
        rest_positions[j] = 0.0;
    }
    drv.set_stopped_disable_gate(armd::driver::StoppedDisableGate {
        safety_stopped: Arc::new(AtomicBool::new(false)),
        has_resting_pose: true,
        arm_joints,
        arm_joints_len: 6,
        rest_positions,
        rest_pose_tolerance_rad: 0.05,
    });
    for _ in 0..3 {
        std::thread::sleep(Duration::from_millis(12));
        bus.with(|s| {
            for m in &mut s.motors {
                m.q = 0.0;
                m.v = 0.0;
            }
            s.push_all_feedback();
        });
        drv.read_joint_states().unwrap();
    }
    bus.clear_log();
    drv.set_torque(false).unwrap();
    let disable_frames: Vec<_> = bus
        .tx_log()
        .into_iter()
        .filter(|f| f.data == [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
        .collect();
    for id in 1u32..=7 {
        assert!(
            disable_frames.iter().any(|f| f.id == id),
            "motor {id} must receive Disable frame after lease loss"
        );
    }
}

// ============================================================================
// 7. Round 2 B2 + M1: Live executor zero-alloc & zero-lock proof with 32 end
//    points + 4 keep-out boxes + 16 link capsules (>256 slots), workspace
//    refusals on intake & step, Step 6 publish, and off-thread C config commit.
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn b2_m1_executor_large_workspace_32_endpoints_keepout_refusal_and_config_commit_zero_alloc() {
    use armd::model::{KeepoutBox, LinkCapsule};

    ensure_tracing_subscriber();
    let mut cfg = ArmConfig {
        lease_timeout_ms: 5_000,
        ..ArmConfig::rebot_dm()
    };
    let base_pose = common::INSPECT.to_vec();
    let mut q_mid = base_pose.clone();
    q_mid[0] += 0.15;
    let mut q_far = base_pose.clone();
    q_far[0] += 0.30;
    let p_mid = cfg.arm_model.fk_f32(&q_mid).expect("fk q_mid").p;

    {
        let ws = cfg.workspace.as_mut().unwrap();
        let mut idx = 0usize;
        while ws.check_points_end_m.len() < 32 {
            let x = -0.02 - 0.004 * (idx as f32);
            let y = if idx.is_multiple_of(2) { 0.01 } else { -0.01 };
            ws.check_points_end_m.push([x, y, 0.0]);
            idx += 1;
        }
        let mut extra = 0usize;
        while ws.link_capsules.len() < armd::model::MAX_LINK_CAPSULES {
            ws.link_capsules.push(LinkCapsule {
                id: format!("exec_cap_{extra}"),
                frame: "link6".into(),
                a_m: [0.0, 0.0, 0.015],
                b_m: [0.0, 0.0, 0.022],
                radius_m: 0.012,
            });
            extra += 1;
        }
        ws.keepout_boxes = vec![
            KeepoutBox {
                name: "kb_a".into(),
                min_m: [-0.58, -0.58, 0.62],
                max_m: [-0.45, -0.45, 0.75],
            },
            KeepoutBox {
                name: "kb_b".into(),
                min_m: [0.45, -0.58, 0.62],
                max_m: [0.58, -0.45, 0.75],
            },
            KeepoutBox {
                name: "kb_c".into(),
                min_m: [-0.58, 0.45, 0.62],
                max_m: [-0.45, 0.58, 0.75],
            },
            KeepoutBox {
                name: "kb_mid_exec".into(),
                min_m: [
                    (p_mid[0] - 0.02) as f32,
                    (p_mid[1] - 0.02) as f32,
                    (p_mid[2] - 0.02) as f32,
                ],
                max_m: [
                    (p_mid[0] + 0.02) as f32,
                    (p_mid[1] + 0.02) as f32,
                    (p_mid[2] + 0.02) as f32,
                ],
            },
        ];
        ws.validate().expect("legal >256-slot workspace");
    }

    let sup = Arc::new(ArmSupervisor::new(cfg));
    {
        let mut inner = sup.inner().write().await;
        inner.commanded_positions.clone_from(&base_pose);
        inner.measured_positions.clone_from(&base_pose);
        inner.target_positions.clone_from(&base_pose);
    }
    let addr = serve(sup.clone()).await;

    let mut ecfg = ExecutorConfig::for_tests(Duration::from_millis(5));
    ecfg.alloc_timing = true;
    let handle = sup.start_executor(ecfg).await.unwrap();
    wait_until(Duration::from_secs(2), || sup.hub().latest().is_some()).await;
    let _legacy_write_hold = sup.inner().write().await;

    let rel_before = handle.stats().release_allocs.snapshot();
    let wake_before = handle.stats().wake_allocs.snapshot();
    let lock_before = sup.inner().control_thread_attempts();

    let mut ws1 = connect_ws(addr).await;
    let gen1 = acquire(&mut ws1, "ws-alloc", false, 1).await;
    let ep1 = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;
    arm_via_ws(&mut ws1, gen1, ep1, 1).await;

    // 1. Valid streaming servo on >256-slot workspace.
    for s in 1..=5u64 {
        let mut q_step = base_pose.clone();
        q_step[0] += 0.004 * (s as f32);
        send(
            &mut ws1,
            json!({
                "type": "servo",
                "generation": gen1,
                "epoch_id": ep1,
                "sequence_id": s,
                "positions": q_step,
            }),
        )
        .await;
    }
    let seq1 = sup.hub().published_seq() + 4;
    wait_until(Duration::from_secs(1), || sup.hub().published_seq() >= seq1).await;

    // 2. Intake workspace refusal (`WORKSPACE_VIOLATION`) on >256-slot workspace.
    send(
        &mut ws1,
        json!({
            "type": "command",
            "path": "/api/v1/motion/servo",
            "body": {"targets": q_mid},
            "generation": gen1,
            "epoch_id": ep1,
            "request_id": 2,
        }),
    )
    .await;
    let ws_rej = next_of(&mut ws1, "command_reject").await;
    assert_eq!(ws_rej["request_id"], 2, "{ws_rej}");
    assert_eq!(ws_rej["code"], "WORKSPACE_VIOLATION", "{ws_rej}");

    // 3. In-motion keep-out sweep refusal (`enter_servo_keepout_hold_rt`) + Step 6 publish!
    // Stream `q_far` continuously so servo motion-frame watchdog stays fresh while sweeping across `kb_mid_exec`.
    let mut saw_ws_violation = false;
    for s in 6..120u64 {
        send(
            &mut ws1,
            json!({
                "type": "servo",
                "generation": gen1,
                "epoch_id": ep1,
                "sequence_id": s,
                "positions": q_far,
            }),
        )
        .await;
        tokio::time::sleep(Duration::from_millis(15)).await;
        if sup
            .hub()
            .latest()
            .is_some_and(|snap| snap.obs.workspace_violation.is_some())
        {
            saw_ws_violation = true;
            break;
        }
    }
    assert!(
        saw_ws_violation,
        "sweeping across kb_mid_exec must trigger enter_servo_keepout_hold_rt and publish workspace_violation"
    );

    // 4. Off-thread C config commit adding a 5th keepout box, followed by another servo & refusal.
    send(
        &mut ws1,
        json!({
            "type": "command",
            "path": "/api/v1/workspace/keepout",
            "body": {
                "boxes": [{
                    "name": "kb_runtime_5",
                    "min_m": [0.45, 0.45, 0.62],
                    "max_m": [0.58, 0.58, 0.75]
                }]
            },
            "generation": gen1,
            "epoch_id": ep1,
            "request_id": 4,
        }),
    )
    .await;
    let cfg_ack = next_of(&mut ws1, "command_ack").await;
    assert_eq!(cfg_ack["request_id"], 4, "{cfg_ack}");

    send(
        &mut ws1,
        json!({
            "type": "command",
            "path": "/api/v1/motion/servo",
            "body": {"targets": q_mid},
            "generation": gen1,
            "epoch_id": ep1,
            "request_id": 5,
        }),
    )
    .await;
    let ws_rej2 = next_of(&mut ws1, "command_reject").await;
    assert_eq!(ws_rej2["code"], "WORKSPACE_VIOLATION", "{ws_rej2}");

    let rel_delta = handle.stats().release_allocs.snapshot().delta_since(rel_before);
    let wake_delta = handle.stats().wake_allocs.snapshot().delta_since(wake_before);
    let lock_after = sup.inner().control_thread_attempts();

    assert_eq!(lock_after - lock_before, 0);
    assert_eq!(
        rel_delta.total(),
        0,
        "armd-control must perform 0 heap ops across >256-slot workspace servo/refusal/config commit: {rel_delta:?}"
    );
    assert_eq!(
        wake_delta.total(),
        0,
        "armd-control must perform 0 wake heap ops across >256-slot workspace: {wake_delta:?}"
    );

    drop(ws1);
    drop(_legacy_write_hold);
    handle.stop_and_join();
}

// ============================================================================
// 8. Round 2 M2: Disconnect handling MUST NOT block when the servo dispatcher
//    mutex is held; fence + Disconnect apply immediately while mutex is held.
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn m2_disconnect_does_not_block_when_servo_dispatcher_mutex_is_held() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig {
        lease_timeout_ms: 5_000,
        ..ArmConfig::rebot_dm()
    }));
    let addr = serve(sup.clone()).await;
    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .unwrap();
    wait_until(Duration::from_secs(2), || sup.hub().latest().is_some()).await;

    let mut ws1 = connect_ws(addr).await;
    let gen1 = acquire(&mut ws1, "m2-owner", false, 1).await;
    let ep1 = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;
    arm_via_ws(&mut ws1, gen1, ep1, 1).await;
    let sub_before = handle.servo_submitted_count();
    send(
        &mut ws1,
        json!({
            "type": "servo",
            "generation": gen1,
            "epoch_id": ep1,
            "sequence_id": 1,
            "positions": [0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }),
    )
    .await;
    wait_until(Duration::from_secs(1), || handle.servo_submitted_count() > sub_before).await;

    // Hold the servo dispatcher mutex (`ServoTargetDispatcher.inner`) continuously across disconnect!
    let dispatcher_guard = handle.target_writer().lock().unwrap();
    let t0 = Instant::now();
    drop(ws1);

    // Real socket disconnect MUST fence `gen1` AND enqueue + apply `Disconnect` on `armd-control`
    // while `dispatcher_guard` is STILL held (proving teardown never blocked on the mutex!).
    wait_until(Duration::from_secs(1), || {
        sup.atomic_fenced_lease_gen() >= gen1
            && !sup
                .hub()
                .latest()
                .unwrap()
                .owner
                .is_lease_active_at(Instant::now())
    })
    .await;
    let disc_elapsed = t0.elapsed();
    assert!(
        disc_elapsed < Duration::from_millis(500),
        "disconnect with held dispatcher mutex took too long: {disc_elapsed:?}"
    );

    // Direct call to `fence_ws_connection_immediate` while mutex is held must return immediately.
    let t_direct = Instant::now();
    let dummy_conn = armd::rt_core::requests::lease::ConnectionHandle::from_ws_id(9999).unwrap();
    sup.fence_ws_connection_immediate(dummy_conn, gen1);
    assert!(
        t_direct.elapsed() < Duration::from_millis(50),
        "fence_ws_connection_immediate blocked on dispatcher mutex: {:?}",
        t_direct.elapsed()
    );

    // Release the mutex: `armd-dispatch` / `is_servo_closed` immediately syncs `close_servo(gen1)`.
    drop(dispatcher_guard);
    wait_until(Duration::from_secs(2), || handle.is_servo_closed(gen1)).await;

    handle.stop_and_join();
}

// ============================================================================
// 9. Round 2 M3: Streaming servo frame preempted by Stop MUST emit `servo_reject`
//    with `reason: "stopped"` and identical correlation on BOTH owners.
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn m3_streaming_servo_frame_preempted_by_stop_emits_stopped_servo_reject_both_owners() {
    use armd::supervisor::OWNER_LEGACY;

    let mut replies = Vec::new();
    for use_executor in [false, true] {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig {
            lease_timeout_ms: 5_000,
            control_frequency_hz: 50,
            ..ArmConfig::rebot_dm()
        }));
        let addr = serve(sup.clone()).await;
        let delay: Arc<Mutex<Option<Duration>>> = Arc::new(Mutex::new(None));
        let exec_handle = if use_executor {
            let h = sup
                .start_executor_with_hooks(
                    ExecutorConfig::for_tests(Duration::from_millis(10)),
                    PersistTestHooks {
                        release_delay_once: Some(delay.clone()),
                        ..Default::default()
                    },
                )
                .await
                .unwrap();
            assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);
            wait_until(Duration::from_secs(2), || sup.hub().latest().is_some()).await;
            Some(h)
        } else {
            assert_eq!(sup.owner_mode(), OWNER_LEGACY);
            sup.control_tick(0.02).await;
            None
        };

        let mut ws = connect_ws(addr).await;
        let gen = acquire(&mut ws, "m3-owner", false, 1).await;
        let ep = if use_executor {
            sup.fresh_snapshot(None).await.unwrap().obs.epoch_id
        } else {
            sup.get_observation().await.epoch_id
        };
        arm_via_ws(&mut ws, gen, ep, 1).await;
        let ep = if use_executor {
            sup.fresh_snapshot(None).await.unwrap().obs.epoch_id
        } else {
            sup.get_observation().await.epoch_id
        };
        let q0 = if use_executor {
            sup.fresh_snapshot(None).await.unwrap().obs.commanded_positions[0]
        } else {
            sup.get_observation().await.commanded_positions[0]
        };

        if let Some(ref h) = exec_handle {
            // Executor: pause one release so the streaming servo frame sits buffered in mailbox,
            // then send Stop so Step 1 STOP preempts Step 4 target consumption!
            *delay.lock().unwrap() = Some(Duration::from_millis(80));
            wait_until(Duration::from_secs(1), || delay.lock().unwrap().is_none()).await;
            let sub_before = h.servo_submitted_count();
            send(
                &mut ws,
                json!({
                    "type": "servo",
                    "generation": gen,
                    "epoch_id": ep,
                    "sequence_id": 1,
                    "positions": [q0 + 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                }),
            )
            .await;
            wait_until(Duration::from_secs(1), || h.servo_submitted_count() > sub_before).await;
            send(
                &mut ws,
                json!({
                    "type": "command",
                    "request_id": 10,
                    "generation": gen,
                    "epoch_id": ep,
                    "path": "/api/v1/motion/stop",
                    "body": {},
                }),
            )
            .await;
        } else {
            // Legacy: hold the ordinary WS worker right before `inner.write()`, send streaming
            // servo, apply Stop, then release the worker so `check_admission` refuses STOPPED!
            let conn_id = sup
                .inner()
                .read()
                .await
                .active_lease
                .as_ref()
                .unwrap()
                .client_id
                .clone();
            armd::api::ws::set_test_ws_worker_before_lock_paused(&conn_id, true);
            send(
                &mut ws,
                json!({
                    "type": "servo",
                    "generation": gen,
                    "epoch_id": ep,
                    "sequence_id": 1,
                    "positions": [q0 + 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                }),
            )
            .await;
            wait_until(Duration::from_secs(1), || {
                armd::api::ws::test_ws_worker_before_lock_blocked_count(&conn_id) >= 1
            })
            .await;
            send(
                &mut ws,
                json!({
                    "type": "command",
                    "request_id": 10,
                    "generation": gen,
                    "epoch_id": ep,
                    "path": "/api/v1/motion/stop",
                    "body": {},
                }),
            )
            .await;
            let stop_ack = next_of(&mut ws, "command_ack").await;
            assert_eq!(stop_ack["request_id"], 10, "{stop_ack}");
            armd::api::ws::set_test_ws_worker_before_lock_paused(&conn_id, false);
        }

        // Collect `servo_reject` (and `command_ack` for Stop on executor, in either arrival order).
        let mut saw_servo_reject: Option<Value> = None;
        let mut saw_stop_ack = !use_executor;
        let deadline = Instant::now() + Duration::from_secs(2);
        while (saw_servo_reject.is_none() || !saw_stop_ack) && Instant::now() < deadline {
            if let Some(msg) = next_non_telemetry_within(&mut ws, Duration::from_millis(500)).await {
                match msg.get("type").and_then(Value::as_str) {
                    Some("servo_reject") => saw_servo_reject = Some(msg),
                    Some("command_ack") if msg["request_id"] == 10 => saw_stop_ack = true,
                    _ => {}
                }
            }
        }
        assert!(saw_stop_ack, "use_executor={use_executor}: expected stop command_ack");
        let rej = saw_servo_reject.expect("expected servo_reject for Stop-preempted streaming frame");
        assert_eq!(rej["type"], "servo_reject", "use_executor={use_executor}: {rej}");
        assert_eq!(rej["reason"], "stopped", "use_executor={use_executor}: {rej}");
        assert_eq!(rej["code"], "STOPPED", "use_executor={use_executor}: {rej}");
        assert_eq!(rej["error"], "STOPPED", "use_executor={use_executor}: {rej}");
        assert_eq!(rej["status"], 409, "use_executor={use_executor}: {rej}");
        assert_eq!(rej["epoch_id"], ep, "use_executor={use_executor}: {rej}");

        let q_after = if use_executor {
            let seq = sup.hub().published_seq() + 3;
            wait_until(Duration::from_secs(1), || sup.hub().published_seq() >= seq).await;
            sup.fresh_snapshot(None).await.unwrap().obs.commanded_positions[0]
        } else {
            sup.control_tick(0.02).await;
            sup.get_observation().await.commanded_positions[0]
        };
        assert!(
            (q_after - q0).abs() < 1e-4,
            "use_executor={use_executor}: Stop-preempted servo moved arm: {q_after} vs {q0}"
        );

        replies.push(rej);
        drop(ws);
        if let Some(h) = exec_handle {
            h.stop_and_join();
        }
    }

    // Legacy and Executor `servo_reject` payloads must match field-for-field!
    assert_eq!(
        replies[0], replies[1],
        "Legacy vs Executor Stop-preempted servo_reject mismatch:\nlegacy={}\nexecutor={}",
        replies[0], replies[1]
    );
}
