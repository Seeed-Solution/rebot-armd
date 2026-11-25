//! P-F1 proof suite: per-connection executor macro & prepared-action replies.
//!
//! Covers:
//! 1. Mock WS replay on `OWNER_EXECUTOR`: success (prepared + macro + handover + observer),
//!    reject (validation/lease/epoch/record/unmigrated jog), cancel, and stop.
//! 2. Races: disconnect/reconnect + slot reuse, per-connection capacity saturation without HOL
//!    blocking, concurrent multi-thread pump, callback re-entry + deferred shutdown, ACK
//!    abandonment wire redaction, and broker malformed-reply recovery.
//! 3. `DamiaoArmDriver<FakeBus>` stop case over live WS on `OWNER_EXECUTOR`.
//! 4. Strictly zero RT allocator operations (`TimedAlloc`) from admission through RT execution,
//!    terminal delivery, retirement, connection close, and slot reuse.

mod common;
mod mit_support;

#[global_allocator]
static ALLOC: armd::diagnostics::alloc::TimedAlloc = armd::diagnostics::alloc::TimedAlloc;

use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

use futures_util::StreamExt;
use serde_json::{json, Value};
use tokio::net::TcpListener;
use tokio_tungstenite::connect_async;

use armd::diagnostics::alloc::AllocCounts;
use armd::driver::{
    ArmDriver, ArmDriverError, DamiaoArmDriver, DriverCapabilities, EnableState, JointTarget,
    TorqueOutcome, TorqueProgress,
};
use armd::model::JointState;
use armd::protocol::damiao::DamiaoLimits;
use armd::rt_core::broker::{
    ActionId, AdmitError, Authority, BootId, Capacities, ClientId, DeliveryDecision, Outcome,
    Reply, RequestId, Submit,
};
use armd::rt_core::executor::{
    ExecutorClock, ExecutorConfig, PersistTestHooks, RtExecutorHandle,
};
use armd::rt_core::ids::ArmId;
use armd::rt_core::ops::{ChildSpec, ChildTicket, PlanSlot};
use armd::rt_core::reply_router::{
    ExternalDestinationPolicy, ExternalSubmitError, WsWireAckTracker,
};
use armd::rt_core::requests::actions::{
    prepare_child_joint_slot, prepare_gripper, prepare_move_to, ActionSnapshot, ActionSubmit,
    BeginMacroSpec, FinishMacroSpec, MacroKind, PreparedActionKind, DEFAULT_ACTION_DEADLINE,
    DEFAULT_FINISH_DEADLINE,
};
use armd::rt_core::requests::lease::ConnectionHandle;
use armd::supervisor::{GripperBaselineCmd, OWNER_EXECUTOR};
use armd::{create_router, ArmConfig, ArmSupervisor, OperatorAuth, SafetyState, SupervisorInner};
use common::{controller, rpc, send, ws_reply, TestSocket, INSPECT};
use mit_support::FakeBus;

fn dm_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm()
        .damiao_ranges
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect()
}

async fn wait_action_terminal_ws(
    ws: &mut TestSocket,
    action_id: u64,
    timeout: Duration,
) -> Value {
    tokio::time::timeout(timeout, async {
        loop {
            let text = ws.next().await.unwrap().unwrap().into_text().unwrap();
            let v: Value = serde_json::from_str(&text).unwrap();
            if v.get("type").is_none() {
                if let Some(a) = v.get("action").filter(|a| !a.is_null()) {
                    if a["id"].as_u64() == Some(action_id) && a["state"] != "running" {
                        return a.clone();
                    }
                }
            }
        }
    })
    .await
    .unwrap_or_else(|_| panic!("timed out waiting for WS terminal of action {action_id}"))
}

async fn send_cmd_and_expect_ack_before_terminal(
    ws: &mut TestSocket,
    req_id: u64,
    generation: u64,
    epoch: u64,
    path: &str,
    body: Value,
) -> (Value, Value) {
    send(
        ws,
        json!({
            "type": "command",
            "request_id": req_id,
            "generation": generation,
            "epoch_id": epoch,
            "path": path,
            "body": body,
        }),
    )
    .await;

    let mut ack: Option<Value> = None;
    let mut ack_action_id: Option<u64> = None;
    let mut seen_early_terminals: Vec<u64> = Vec::new();

    let terminal = tokio::time::timeout(Duration::from_secs(15), async {
        loop {
            let text = ws.next().await.unwrap().unwrap().into_text().unwrap();
            let v: Value = serde_json::from_str(&text).unwrap();
            if v.get("type").is_some() {
                if v["request_id"].as_u64() == Some(req_id) {
                    assert_eq!(v["type"], json!("command_ack"), "expected command_ack for {path}, got {v}");
                    let aid = v["action_id"].as_u64().expect("command_ack must include action_id");
                    assert!(
                        !seen_early_terminals.contains(&aid),
                        "INVARIANT VIOLATION: terminal for action {aid} arrived BEFORE command_ack on {path}"
                    );
                    ack_action_id = Some(aid);
                    ack = Some(v);
                }
            } else if let Some(a) = v.get("action").filter(|a| !a.is_null()) {
                if a["state"] != "running" {
                    if let Some(id) = a["id"].as_u64() {
                        if ack_action_id == Some(id) {
                            return a.clone();
                        }
                        if ack_action_id.is_none() {
                            seen_early_terminals.push(id);
                        }
                    }
                }
            }
        }
    })
    .await
    .unwrap_or_else(|_| panic!("timed out waiting for ACK+terminal on {path}"));

    (ack.unwrap(), terminal)
}

async fn wait_pub_ready(sup: &ArmSupervisor) {
    let dl = Instant::now() + Duration::from_secs(3);
    while sup
        .hub()
        .latest()
        .is_none_or(|s| s.rt.owner.boot.is_zero())
    {
        assert!(Instant::now() < dl, "executor did not publish initial snapshot");
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
}

// ============================================================================
// 1. Mock WS replay on OWNER_EXECUTOR: success, reject, cancel, stop, handover
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn pf1_mock_ws_replay_success_reject_cancel_stop_and_handover() {
    let cfg = ArmConfig {
        collision: None,
        lease_timeout_ms: 60_000,
        ..ArmConfig::rebot_dm()
    };
    let sup = Arc::new(ArmSupervisor::new(cfg));
    {
        let mut inner = sup.inner().write().await;
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        inner.tick(0.02);
    }
    sup.start_dispatcher().unwrap();
    let router = create_router(sup.clone(), OperatorAuth::disabled());
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let srv = tokio::spawn(async move {
        let _ = axum::serve(listener, router).await;
    });

    // Connect BOTH controller and observer BEFORE Legacy -> Executor handover!
    let (mut ws_ctrl, gen_ctrl) = controller(addr).await;
    let (mut ws_obs, _) = connect_async(format!("ws://{addr}/api/v1/ws"))
        .await
        .unwrap();
    let ep0 = sup.get_observation().await.epoch_id;
    let tq_ack = rpc(&mut ws_ctrl, gen_ctrl, ep0, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(tq_ack["type"], "command_ack", "{tq_ack}");
    // Save a pose & a 1-step wait sequence while still in Legacy so we test move_to_pose + run_sequence.
    let ep1 = sup.get_observation().await.epoch_id;
    let p_ack = rpc(
        &mut ws_ctrl,
        gen_ctrl,
        ep1,
        "/api/v1/poses/save",
        json!({"name": "pf1_pose", "positions": INSPECT}),
    )
    .await;
    assert_eq!(p_ack["type"], "command_ack", "{p_ack}");
    let s_ack = rpc(
        &mut ws_ctrl,
        gen_ctrl,
        ep1,
        "/api/v1/sequences/save",
        json!({"name": "pf1_seq", "steps": [{"type": "wait", "s": 0.05}]}),
    )
    .await;
    assert_eq!(s_ack["type"], "command_ack", "{s_ack}");
    let seq_id = s_ack["sequence"]["id"]
        .as_str()
        .or_else(|| s_ack["id"].as_str())
        .map(str::to_string)
        .unwrap_or_else(|| {
            // Read saved sequence id from hub prep_env if not echoed on ack.
            "pf1_seq".to_string()
        });

    // Handover to RT Executor while ws_ctrl and ws_obs remain connected!
    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .unwrap();
    assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);
    wait_pub_ready(&sup).await;
    let seq_id = sup
        .hub()
        .prep_env()
        .and_then(|e| e.sequences.file.sequences.first().map(|s| s.id.clone()))
        .unwrap_or(seq_id);

    let ep = sup.hub().latest().unwrap().obs.epoch_id;
    let snap = ActionSnapshot::capture(
        &sup,
        sup.capture_admission_stamp(),
        Instant::now() + DEFAULT_ACTION_DEADLINE,
    )
    .await
    .unwrap();
    let tcp0 = armd::motion::start_pose(&snap.plan_input_in(armd::motion::GoalFrame::Tcp).unwrap()).p;

    // (a) Prepared actions over handover socket: move_to (linear), move_relative, gripper, move_to_pose.
    let mut req_id = 100u64;

    let (ack_m1, term_m1) = send_cmd_and_expect_ack_before_terminal(
        &mut ws_ctrl,
        req_id,
        gen_ctrl,
        ep,
        "/api/v1/action/move_to",
        json!({"position": [tcp0[0] + 0.02, tcp0[1], tcp0[2]], "speed_scale": 1.0}),
    )
    .await;
    req_id += 1;
    assert_eq!(ack_m1["status"], "accepted");
    assert!(ack_m1["expected_duration_s"].as_f64().unwrap_or(0.0) > 0.0);
    assert_eq!(term_m1["state"], "done", "{term_m1}");
    let obs_term_m1 = wait_action_terminal_ws(
        &mut ws_obs,
        ack_m1["action_id"].as_u64().unwrap(),
        Duration::from_secs(5),
    )
    .await;
    assert_eq!(obs_term_m1["state"], "done");

    let (ack_rel, term_rel) = send_cmd_and_expect_ack_before_terminal(
        &mut ws_ctrl,
        req_id,
        gen_ctrl,
        ep,
        "/api/v1/action/move_relative",
        json!({"delta": [-0.02, 0.0, 0.0], "frame": "base", "speed_scale": 1.0}),
    )
    .await;
    req_id += 1;
    assert_eq!(term_rel["state"], "done", "{term_rel}");
    let _ = wait_action_terminal_ws(
        &mut ws_obs,
        ack_rel["action_id"].as_u64().unwrap(),
        Duration::from_secs(5),
    )
    .await;

    let (ack_g, term_g) = send_cmd_and_expect_ack_before_terminal(
        &mut ws_ctrl,
        req_id,
        gen_ctrl,
        ep,
        "/api/v1/action/gripper",
        json!({"command": "open", "speed": 2.0}),
    )
    .await;
    req_id += 1;
    assert_eq!(term_g["state"], "done", "{term_g}");
    let _ = wait_action_terminal_ws(
        &mut ws_obs,
        ack_g["action_id"].as_u64().unwrap(),
        Duration::from_secs(5),
    )
    .await;

    let (ack_pose, term_pose) = send_cmd_and_expect_ack_before_terminal(
        &mut ws_ctrl,
        req_id,
        gen_ctrl,
        ep,
        "/api/v1/action/move_to_pose",
        json!({"name": "pf1_pose", "speed_scale": 1.0}),
    )
    .await;
    req_id += 1;
    assert_eq!(term_pose["state"], "done", "{term_pose}");
    let _ = wait_action_terminal_ws(
        &mut ws_obs,
        ack_pose["action_id"].as_u64().unwrap(),
        Duration::from_secs(5),
    )
    .await;

    // (b) Macro actions over WS on OWNER_EXECUTOR: place + run_sequence + dry_run grasp!
    let (ack_place, term_place) = send_cmd_and_expect_ack_before_terminal(
        &mut ws_ctrl,
        req_id,
        gen_ctrl,
        ep,
        "/api/v1/action/place",
        json!({"position": [tcp0[0] + 0.02, tcp0[1], tcp0[2]], "approach_height_m": 0.02, "release": false, "retreat": false}),
    )
    .await;
    req_id += 1;
    assert_eq!(term_place["state"], "done", "{term_place}");
    assert_eq!(term_place["kind"], "place");
    let _ = wait_action_terminal_ws(
        &mut ws_obs,
        ack_place["action_id"].as_u64().unwrap(),
        Duration::from_secs(5),
    )
    .await;

    let (ack_seq, term_seq) = send_cmd_and_expect_ack_before_terminal(
        &mut ws_ctrl,
        req_id,
        gen_ctrl,
        ep,
        "/api/v1/action/run_sequence",
        json!({"id": seq_id}),
    )
    .await;
    req_id += 1;
    assert_eq!(term_seq["state"], "done", "{term_seq}");
    let _ = wait_action_terminal_ws(
        &mut ws_obs,
        ack_seq["action_id"].as_u64().unwrap(),
        Duration::from_secs(5),
    )
    .await;

    // Also prove grasp (both dry_run AND live admission!) + calib_hover + measure_multiview over WS
    // route through executor admission (never 503 EXECUTOR_ROUTING_PENDING) with exact error codes.
    let grasp_body = json!({
        "label": "cube",
        "box_norm": [0.46, 0.86, 0.54, 0.94],
        "strategy": "legacy",
        "retreat": false
    });
    let mut grasp_dry_body = grasp_body.clone();
    grasp_dry_body["dry_run"] = json!(true);
    let grasp_dry = ws_reply(
        &mut ws_ctrl,
        json!({
            "type": "command",
            "request_id": req_id,
            "generation": gen_ctrl,
            "epoch_id": ep,
            "path": "/api/v1/action/grasp",
            "body": grasp_dry_body
        }),
        &["command_ack"],
    )
    .await;
    req_id += 1;
    assert_eq!(grasp_dry["status"], "accepted", "{grasp_dry}");
    assert_eq!(grasp_dry["reachable"], json!(true), "{grasp_dry}");

    // Live (non-dry-run) WS adaptive grasp reaching executor admission -> ACK strictly before terminal!
    let (ack_grasp, term_grasp) = send_cmd_and_expect_ack_before_terminal(
        &mut ws_ctrl,
        req_id,
        gen_ctrl,
        ep,
        "/api/v1/action/grasp",
        grasp_body,
    )
    .await;
    req_id += 1;
    assert_eq!(ack_grasp["status"], "accepted", "{ack_grasp}");
    assert_eq!(term_grasp["kind"], "grasp", "{term_grasp}");
    assert!(
        matches!(term_grasp["state"].as_str(), Some("done" | "failed")),
        "live WS grasp must reach terminal state: {term_grasp}"
    );
    let _ = wait_action_terminal_ws(
        &mut ws_obs,
        ack_grasp["action_id"].as_u64().unwrap(),
        Duration::from_secs(5),
    )
    .await;

    let ch_reply = ws_reply(
        &mut ws_ctrl,
        json!({
            "type": "command",
            "request_id": req_id,
            "generation": gen_ctrl,
            "epoch_id": ep,
            "path": "/api/v1/action/calib_hover",
            "body": {"resume": 123}
        }),
        &["command_reject"],
    )
    .await;
    req_id += 1;
    assert_eq!(
        (ch_reply["status"].as_u64(), ch_reply["code"].as_str()),
        (Some(400), Some("INVALID_COMMAND")),
        "calib_hover invalid resume must reach prepare_from_snapshot validation: {ch_reply}"
    );

    let mm_reply = ws_reply(
        &mut ws_ctrl,
        json!({
            "type": "command",
            "request_id": req_id,
            "generation": gen_ctrl,
            "epoch_id": ep,
            "path": "/api/v1/action/measure_multiview",
            "body": {"views": 123}
        }),
        &["command_reject"],
    )
    .await;
    req_id += 1;
    assert_eq!(
        (mm_reply["status"].as_u64(), mm_reply["code"].as_str()),
        (Some(400), Some("INVALID_COMMAND")),
        "measure_multiview invalid body must reach prepare_from_snapshot validation: {mm_reply}"
    );

    // (c) Rejections: bad epoch, bad body, non-owner lease, duplicate req_id, recording seq, unmigrated jog 503.
    let bad_ep = ws_reply(
        &mut ws_ctrl,
        json!({"type": "command", "request_id": req_id, "generation": gen_ctrl, "epoch_id": ep + 99, "path": "/api/v1/action/move_to", "body": {"position": [tcp0[0], tcp0[1], tcp0[2]]}}),
        &["command_reject"],
    )
    .await;
    assert_eq!((bad_ep["status"].as_u64(), bad_ep["code"].as_str()), (Some(409), Some("EPOCH_MISMATCH")), "{bad_ep}");

    let bad_body = ws_reply(
        &mut ws_ctrl,
        json!({"type": "command", "request_id": req_id, "generation": gen_ctrl, "epoch_id": ep, "path": "/api/v1/action/place", "body": 42}),
        &["command_reject"],
    )
    .await;
    assert_eq!((bad_body["status"].as_u64(), bad_body["code"].as_str()), (Some(400), Some("INVALID_COMMAND")), "{bad_body}");

    // Duplicate request_id rejected with 400 INVALID_COMMAND.
    let dup_req = ws_reply(
        &mut ws_ctrl,
        json!({"type": "command", "request_id": req_id, "generation": gen_ctrl, "epoch_id": ep, "path": "/api/v1/action/place", "body": {}}),
        &["command_reject"],
    )
    .await;
    assert_eq!((dup_req["status"].as_u64(), dup_req["code"].as_str()), (Some(400), Some("INVALID_COMMAND")), "{dup_req}");
    req_id += 1;

    // Non-owner socket receives 409 LEASE_LOST (NOT 503).
    let no_lease = ws_reply(
        &mut ws_obs,
        json!({"type": "command", "request_id": 1, "generation": gen_ctrl, "epoch_id": ep, "path": "/api/v1/action/move_to", "body": {"position": [tcp0[0], tcp0[1], tcp0[2]]}}),
        &["command_reject"],
    )
    .await;
    assert_eq!((no_lease["status"].as_u64(), no_lease["code"].as_str()), (Some(409), Some("LEASE_LOST")), "{no_lease}");

    // Recording sequence on executor refused upfront (never 503 EXECUTOR_ROUTING_PENDING).
    let rec_rej = ws_reply(
        &mut ws_ctrl,
        json!({"type": "command", "request_id": req_id, "generation": gen_ctrl, "epoch_id": ep, "path": "/api/v1/action/run_sequence", "body": {"id": seq_id, "record": {"dataset": "pf1"}}}),
        &["command_reject"],
    )
    .await;
    req_id += 1;
    assert_ne!(rec_rej["code"].as_str(), Some("EXECUTOR_ROUTING_PENDING"), "{rec_rej}");

    // Unmigrated motion jog stays at 503 EXECUTOR_ROUTING_PENDING.
    let jog_rej = ws_reply(
        &mut ws_ctrl,
        json!({"type": "command", "request_id": req_id, "generation": gen_ctrl, "epoch_id": ep, "path": "/api/v1/motion/jog", "body": {"target_positions": INSPECT}}),
        &["command_reject"],
    )
    .await;
    req_id += 1;
    assert_eq!((jog_rej["status"].as_u64(), jog_rej["code"].as_str()), (Some(503), Some("EXECUTOR_ROUTING_PENDING")), "{jog_rej}");

    // (d) Cancel/Stop mid-action: slow move_to -> ACK -> urgent Stop -> cancelled terminal!
    let slow_ack = ws_reply(
        &mut ws_ctrl,
        json!({"type": "command", "request_id": req_id, "generation": gen_ctrl, "epoch_id": ep, "path": "/api/v1/action/move_to", "body": {"position": [tcp0[0] + 0.04, tcp0[1], tcp0[2]], "speed_scale": 0.1}}),
        &["command_ack"],
    )
    .await;
    req_id += 1;
    let slow_aid = slow_ack["action_id"].as_u64().unwrap();
    let stop_ack = ws_reply(
        &mut ws_ctrl,
        json!({"type": "command", "request_id": req_id, "generation": gen_ctrl, "epoch_id": ep, "path": "/api/v1/motion/stop", "body": {}}),
        &["command_ack"],
    )
    .await;
    assert_eq!(stop_ack["type"], "command_ack", "{stop_ack}");
    let stop_term = wait_action_terminal_ws(&mut ws_ctrl, slow_aid, Duration::from_secs(5)).await;
    assert_eq!(stop_term["state"], "cancelled", "{stop_term}");

    // Verify all router reservations recovered to 0 and zero invariant anomalies.
    tokio::time::sleep(Duration::from_millis(30)).await;
    handle.pump_broker_replies();
    let st = handle.reply_router_stats();
    assert_eq!(st.global_reserved, 0, "leaked router reservations: {st:?}");
    assert_eq!(st.mapping_mismatches, 0, "{st:?}");
    assert_eq!(st.protocol_anomalies, 0, "{st:?}");
    assert!(st.terminals_delivered >= 7, "expected >= 7 delivered broker terminals, got {st:?}");
    assert!(!handle.has_external_reply_sink(), "WS path must NEVER attach global dummy sink");

    let _ = ws_ctrl.close(None).await;
    let _ = ws_obs.close(None).await;
    srv.abort();
    handle.stop_and_join();
}

// ============================================================================
// 2. Races: disconnect/reconnect + slot reuse + capacity saturation
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn pf1_races_disconnect_reconnect_slot_reuse_and_capacity_saturation() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig {
        collision: None,
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    }));
    sup.set_lease_enabled(false).await;
    {
        let mut inner = sup.inner().write().await;
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::enable_torque(a).unwrap();
        inner.tick(0.02);
    }
    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .unwrap();
    wait_pub_ready(&sup).await;

    // --- Part A: Disconnect/reconnect + slot reuse ---
    sup.register_ws_connection(101).await.unwrap();
    let c101_replies = Arc::new(Mutex::new(Vec::<(ClientId, Reply)>::new()));
    let c101_out = Arc::clone(&c101_replies);
    let tok_101 = handle
        .register_connection_reply_sink(
            ClientId(101),
            Arc::new(move |c, r| {
                c101_out.lock().unwrap().push((c, r));
                DeliveryDecision::Delivered
            }),
            Some(4),
        )
        .unwrap();

    let mut adm1 = sup.capture_admission_stamp();
    adm1.connection_id = 101;
    let snap1 = ActionSnapshot::capture(&sup, adm1, Instant::now() + DEFAULT_ACTION_DEADLINE)
        .await
        .unwrap();
    let p0 = armd::motion::start_pose(&snap1.plan_input_in(armd::motion::GoalFrame::Tcp).unwrap()).p;
    let slow_move = prepare_move_to(
        &snap1,
        &json!({"position": [p0[0] + 0.02, p0[1], p0[2]], "speed_scale": 0.8}),
    )
    .unwrap();
    let slot1 = slow_move.to_plan_slot(None).unwrap();
    let write1 = |dst: &mut PlanSlot| *dst = slot1.clone();
    let (_req1, mut ack_rx1, _guard1) = handle
        .submit_external_action(
            ClientId(101),
            1,
            snap1.owner.authority,
            Instant::now() + DEFAULT_ACTION_DEADLINE,
            ActionSubmit::InstallPrepared {
                write: &write1,
                kind: PreparedActionKind::MoveToLinear,
                prepared_boot: handle.boot(),
                admission_request_id: adm1.request_id,
                check_workspace_revision: true,
                poses_revision: None,
                calibration_revision: None,
                gripper_calib_revision: None,
                power_budget_revision: Some(snap1.env.power_budget_revision),
            },
            ExternalDestinationPolicy::ConnectionOnly,
            false,
            None,
        )
        .unwrap();
    // Wait for ACK so the slow motion is actively running on RT.
    let ack1 = loop {
        handle.pump_broker_replies();
        if let Ok(r) = ack_rx1.try_recv() {
            break r.unwrap();
        }
        tokio::time::sleep(Duration::from_millis(2)).await;
    };
    let aid1 = match ack1 {
        Reply::Ack { action: Some(id), .. } => id,
        other => panic!("expected Ack with ActionId, got {other:?}"),
    };

    // Disconnect ClientId(101) while action `aid1` is still running!
    handle
        .disconnect_connection_async(ConnectionHandle::from_ws_id(101).unwrap())
        .await
        .unwrap();

    // New admission for closed ClientId(101) MUST fail immediately with Disconnected!
    let err_closed = handle.submit_external_action(
        ClientId(101),
        2,
        snap1.owner.authority,
        Instant::now() + DEFAULT_ACTION_DEADLINE,
        ActionSubmit::BeginMacro(BeginMacroSpec::new(MacroKind::Place, handle.boot())),
        ExternalDestinationPolicy::ConnectionOnly,
        false,
        None,
    );
    assert!(
        matches!(err_closed, Err(ExternalSubmitError::Admit(AdmitError::Disconnected))),
        "closed connection must block new admissions: {err_closed:?}"
    );

    // Immediately register ClientId(102) reusing the EXACT same slot_idx with strictly larger slot_gen!
    sup.register_ws_connection(102).await.unwrap();
    let c102_replies = Arc::new(Mutex::new(Vec::<(ClientId, Reply)>::new()));
    let c102_out = Arc::clone(&c102_replies);
    let tok_102 = handle
        .register_connection_reply_sink(
            ClientId(102),
            Arc::new(move |c, r| {
                c102_out.lock().unwrap().push((c, r));
                DeliveryDecision::Delivered
            }),
            Some(4),
        )
        .unwrap();
    assert_eq!(tok_102.slot_idx, tok_101.slot_idx, "new connection must reuse freed slot_idx");
    assert!(tok_102.slot_gen > tok_101.slot_gen, "reused slot must carry strictly larger slot_gen");

    // Wait for c101's aid1 to physically finish AFTER c102 has already reused tok_101.slot_idx!
    // c102 MUST NOT receive aid1's terminal.
    let aid1_done_dl = Instant::now() + Duration::from_secs(5);
    while Instant::now() < aid1_done_dl {
        handle.pump_broker_replies();
        let finished = sup
            .hub()
            .latest()
            .and_then(|s| s.obs.action.as_ref().map(|a| a.id == aid1.0 && a.finished_at.is_some()))
            .unwrap_or(false);
        if finished {
            break;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    handle.pump_broker_replies();
    assert!(
        c102_replies.lock().unwrap().is_empty(),
        "reused slot c102 MUST NOT receive c101's terminal: {:?}",
        *c102_replies.lock().unwrap()
    );

    // Submit ClientId(102)'s own gripper action and verify it receives ONLY aid2's terminal.
    let mut adm2 = sup.capture_admission_stamp();
    adm2.connection_id = 102;
    let snap2 = ActionSnapshot::capture(&sup, adm2, Instant::now() + DEFAULT_ACTION_DEADLINE)
        .await
        .unwrap();
    let g_prep = prepare_gripper(
        &snap2,
        &json!({"command": "open", "speed": 2.0}),
        GripperBaselineCmd::None,
    )
    .unwrap();
    let g_slot = g_prep.to_plan_slot(None).unwrap();
    let write_g = |dst: &mut PlanSlot| *dst = g_slot.clone();
    let (req2, mut ack_rx2, _guard2) = handle
        .submit_external_action(
            ClientId(102),
            1,
            snap2.owner.authority,
            Instant::now() + DEFAULT_ACTION_DEADLINE,
            ActionSubmit::InstallPrepared {
                write: &write_g,
                kind: PreparedActionKind::Gripper,
                prepared_boot: handle.boot(),
                admission_request_id: adm2.request_id,
                check_workspace_revision: false,
                poses_revision: None,
                calibration_revision: None,
                gripper_calib_revision: g_prep.stamp.gripper_calib_revision,
                power_budget_revision: None,
            },
            ExternalDestinationPolicy::ConnectionOnly,
            false,
            None,
        )
        .unwrap();
    let ack2 = loop {
        handle.pump_broker_replies();
        if let Ok(r) = ack_rx2.try_recv() {
            break r.unwrap();
        }
        tokio::time::sleep(Duration::from_millis(2)).await;
    };
    let aid2 = match ack2 {
        Reply::Ack { action: Some(id), .. } => id,
        other => panic!("expected Ack for c102, got {other:?}"),
    };
    let dl = Instant::now() + Duration::from_secs(5);
    while Instant::now() < dl {
        handle.pump_broker_replies();
        if c102_replies
            .lock()
            .unwrap()
            .iter()
            .any(|(_, r)| matches!(r, Reply::Terminal { request, action, .. } if *request == req2 && *action == aid2))
        {
            break;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    let got_102 = c102_replies.lock().unwrap().clone();
    assert_eq!(got_102.len(), 1, "reused slot must receive ONLY its own terminal, got {got_102:?}");
    assert!(
        matches!(got_102[0], (ClientId(102), Reply::Terminal { request, action, .. }) if request == req2 && action == aid2 && action != aid1),
        "reused slot received wrong reply: {:?}",
        got_102[0]
    );

    // --- Part B: Capacity saturation (per-connection cap = 2, no HOL blocking) ---
    sup.register_ws_connection(201).await.unwrap();
    sup.register_ws_connection(202).await.unwrap();
    let bp_201 = Arc::new(AtomicBool::new(true));
    let c201_replies = Arc::new(Mutex::new(Vec::<Reply>::new()));
    let c202_replies = Arc::new(Mutex::new(Vec::<Reply>::new()));
    {
        let bp = Arc::clone(&bp_201);
        let out = Arc::clone(&c201_replies);
        handle
            .register_connection_reply_sink(
                ClientId(201),
                Arc::new(move |_, r| {
                    if bp.load(Ordering::Acquire) {
                        DeliveryDecision::Backpressured
                    } else {
                        out.lock().unwrap().push(r);
                        DeliveryDecision::Delivered
                    }
                }),
                Some(2),
            )
            .unwrap();
    }
    {
        let out = Arc::clone(&c202_replies);
        handle
            .register_connection_reply_sink(
                ClientId(202),
                Arc::new(move |_, r| {
                    out.lock().unwrap().push(r);
                    DeliveryDecision::Delivered
                }),
                Some(4),
            )
            .unwrap();
    }

    // Fill BOTH of ClientId(201)'s 2 per-connection slots with backpressured terminals.
    let mut c201_expected: Vec<(RequestId, ActionId)> = Vec::new();
    for seq in 1..=2u64 {
        let mut adm = sup.capture_admission_stamp();
        adm.connection_id = 201;
        let snap = ActionSnapshot::capture(&sup, adm, Instant::now() + DEFAULT_ACTION_DEADLINE)
            .await
            .unwrap();
        let mut spec = BeginMacroSpec::new(MacroKind::Place, handle.boot());
        spec.admission_request_id = adm.request_id;
        let (req, mut rx, _g) = handle
            .submit_external_action(
                ClientId(201),
                seq,
                snap.owner.authority,
                Instant::now() + DEFAULT_ACTION_DEADLINE,
                ActionSubmit::BeginMacro(spec),
                ExternalDestinationPolicy::ConnectionOnly,
                false,
                None,
            )
            .unwrap();
        let aid = loop {
            handle.pump_broker_replies();
            if let Ok(Ok(Reply::Ack { action: Some(id), .. })) = rx.try_recv() {
                break id;
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        };
        // Internal FinishMacro RPC must NOT be blocked by external saturation!
        let mut fin_auth = snap.owner.authority;
        fin_auth.parent_generation = aid.0;
        let fin = FinishMacroSpec::done(aid, handle.boot());
        let (_fr, mut frx) = handle
            .submit_internal_action(
                ClientId(201),
                seq + 10,
                fin_auth,
                Instant::now() + DEFAULT_FINISH_DEADLINE,
                ActionSubmit::FinishMacro(fin),
            )
            .unwrap();
        let fin_reply = loop {
            handle.pump_broker_replies();
            if let Ok(r) = frx.try_recv() {
                break r;
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        };
        assert!(matches!(fin_reply, Reply::Ack { .. }), "FinishMacro refused: {fin_reply:?}");
        c201_expected.push((req, aid));
    }

    // 3rd external submit for saturated ClientId(201) MUST fail boundedly with RepliesFull!
    let mut adm_sat = sup.capture_admission_stamp();
    adm_sat.connection_id = 201;
    let snap_sat = ActionSnapshot::capture(&sup, adm_sat, Instant::now() + DEFAULT_ACTION_DEADLINE)
        .await
        .unwrap();
    let sat_res = handle.submit_external_action(
        ClientId(201),
        3,
        snap_sat.owner.authority,
        Instant::now() + DEFAULT_ACTION_DEADLINE,
        ActionSubmit::BeginMacro(BeginMacroSpec::new(MacroKind::Place, handle.boot())),
        ExternalDestinationPolicy::ConnectionOnly,
        false,
        None,
    );
    assert!(
        matches!(sat_res, Err(ExternalSubmitError::Admit(AdmitError::RepliesFull))),
        "expected RepliesFull on saturated connection 201, got {sat_res:?}"
    );

    // Unrelated ClientId(202) MUST NOT be head-of-line blocked by ClientId(201)'s backpressure!
    let mut adm_202 = sup.capture_admission_stamp();
    adm_202.connection_id = 202;
    let snap_202 = ActionSnapshot::capture(&sup, adm_202, Instant::now() + DEFAULT_ACTION_DEADLINE)
        .await
        .unwrap();
    let mut spec_202 = BeginMacroSpec::new(MacroKind::Place, handle.boot());
    spec_202.admission_request_id = adm_202.request_id;
    let (req_202, mut rx_202, _g_202) = handle
        .submit_external_action(
            ClientId(202),
            1,
            snap_202.owner.authority,
            Instant::now() + DEFAULT_ACTION_DEADLINE,
            ActionSubmit::BeginMacro(spec_202),
            ExternalDestinationPolicy::ConnectionOnly,
            false,
            None,
        )
        .unwrap();
    let aid_202 = loop {
        handle.pump_broker_replies();
        if let Ok(Ok(Reply::Ack { action: Some(id), .. })) = rx_202.try_recv() {
            break id;
        }
        tokio::time::sleep(Duration::from_millis(2)).await;
    };
    let mut fin_auth_202 = snap_202.owner.authority;
    fin_auth_202.parent_generation = aid_202.0;
    let (_, mut frx_202) = handle
        .submit_internal_action(
            ClientId(202),
            2,
            fin_auth_202,
            Instant::now() + DEFAULT_FINISH_DEADLINE,
            ActionSubmit::FinishMacro(FinishMacroSpec::done(aid_202, handle.boot())),
        )
        .unwrap();
    let dl202 = Instant::now() + Duration::from_secs(3);
    while Instant::now() < dl202 {
        handle.pump_broker_replies();
        let _ = frx_202.try_recv();
        if !c202_replies.lock().unwrap().is_empty() {
            break;
        }
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
    assert_eq!(c202_replies.lock().unwrap().len(), 1);
    assert!(matches!(
        c202_replies.lock().unwrap()[0],
        Reply::Terminal { request, action, outcome: Outcome::Completed, .. }
            if request == req_202 && action == aid_202
    ));

    // Release backpressure on 201 -> exact FIFO delivery of both terminals & global_reserved == 0!
    bp_201.store(false, Ordering::Release);
    handle.pump_broker_replies();
    let got_201 = c201_replies.lock().unwrap().clone();
    assert_eq!(got_201.len(), 2, "expected 2 FIFO terminals for 201, got {got_201:?}");
    for (idx, (exp_req, exp_aid)) in c201_expected.iter().enumerate() {
        assert!(
            matches!(got_201[idx], Reply::Terminal { request, action, outcome: Outcome::Completed, .. } if request == *exp_req && action == *exp_aid),
            "FIFO mismatch at {idx}: {:?}",
            got_201[idx]
        );
    }
    assert_eq!(handle.reply_router_stats().global_reserved, 0);

    handle.stop_and_join();
}

// ============================================================================
// 3. Races: concurrent pump + callback re-entry + deferred shutdown + malformed broker
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn pf1_races_concurrent_pump_order_mapping_reentry_and_malformed_recovery() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig {
        collision: None,
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    }));
    sup.set_lease_enabled(false).await;
    {
        let mut inner = sup.inner().write().await;
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::enable_torque(a).unwrap();
        inner.tick(0.02);
    }
    sup.start_dispatcher().unwrap();
    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .unwrap();
    wait_pub_ready(&sup).await;

    // Register 2 connections with timestamped event logs asserting ACK-before-terminal & exact mapping.
    let event_seq = Arc::new(AtomicU64::new(1));
    let events: Arc<Mutex<Vec<(u64, &'static str, ClientId, RequestId, ActionId)>>> =
        Arc::new(Mutex::new(Vec::new()));
    for cid in [301u64, 302u64] {
        sup.register_ws_connection(cid).await.unwrap();
        let ev = Arc::clone(&events);
        let seq = Arc::clone(&event_seq);
        let h_cb = handle.clone();
        handle
            .register_connection_reply_sink(
                ClientId(cid),
                Arc::new(move |client, reply| {
                    // Exercise same-thread re-entrant pump inside sink callback (must never deadlock!).
                    let _ = h_cb.pump_broker_replies();
                    if let Reply::Terminal { request, action, .. } = reply {
                        let s = seq.fetch_add(1, Ordering::SeqCst);
                        ev.lock().unwrap().push((s, "terminal", client, request, action));
                    }
                    DeliveryDecision::Delivered
                }),
                Some(16),
            )
            .unwrap();
    }

    // Spawn 6 concurrent pump threads hammering `pump_broker_replies()` alongside `armd-dispatch`!
    let stop_pumpers = Arc::new(AtomicBool::new(false));
    let mut pumpers = Vec::new();
    for _ in 0..6 {
        let h = handle.clone();
        let stop = Arc::clone(&stop_pumpers);
        pumpers.push(std::thread::spawn(move || {
            while !stop.load(Ordering::Acquire) {
                let _ = h.pump_broker_replies();
                std::thread::yield_now();
            }
        }));
    }

    let mut expected_map: HashMap<RequestId, (ClientId, ActionId)> = HashMap::new();
    for i in 0..12u64 {
        let cid = if i % 2 == 0 { 301 } else { 302 };
        let mut adm = sup.capture_admission_stamp();
        adm.connection_id = cid;
        let snap = ActionSnapshot::capture(&sup, adm, Instant::now() + DEFAULT_ACTION_DEADLINE)
            .await
            .unwrap();
        let mut spec = BeginMacroSpec::new(MacroKind::Place, handle.boot());
        spec.admission_request_id = adm.request_id;
        let (req, rx, _guard) = handle
            .submit_external_action(
                ClientId(cid),
                i + 1,
                snap.owner.authority,
                Instant::now() + DEFAULT_ACTION_DEADLINE,
                ActionSubmit::BeginMacro(spec),
                ExternalDestinationPolicy::ConnectionOnly,
                false,
                None,
            )
            .unwrap();
        let ack = tokio::time::timeout(Duration::from_secs(5), rx)
            .await
            .expect("ACK timeout under concurrent pump")
            .unwrap()
            .unwrap();
        let aid = match ack {
            Reply::Ack {
                request,
                action: Some(id),
                ..
            } => {
                assert_eq!(request, req);
                let s = event_seq.fetch_add(1, Ordering::SeqCst);
                events
                    .lock()
                    .unwrap()
                    .push((s, "ack", ClientId(cid), req, id));
                id
            }
            other => panic!("unexpected ACK: {other:?}"),
        };
        expected_map.insert(req, (ClientId(cid), aid));

        // Finish macro internally so its terminal fires cleanly.
        let mut fin_auth = snap.owner.authority;
        fin_auth.parent_generation = aid.0;
        let (_, frx) = handle
            .submit_internal_action(
                ClientId(cid),
                100 + i,
                fin_auth,
                Instant::now() + DEFAULT_FINISH_DEADLINE,
                ActionSubmit::FinishMacro(FinishMacroSpec::done(aid, handle.boot())),
            )
            .unwrap();
        let fin_reply = tokio::time::timeout(Duration::from_secs(5), frx)
            .await
            .expect("FinishMacro ACK timeout")
            .unwrap();
        assert!(matches!(fin_reply, Reply::Ack { .. }), "FinishMacro refused: {fin_reply:?}");
    }

    // Wait for all 12 terminals to arrive.
    let dl = Instant::now() + Duration::from_secs(5);
    while Instant::now() < dl {
        let term_count = events
            .lock()
            .unwrap()
            .iter()
            .filter(|e| e.1 == "terminal")
            .count();
        if term_count >= 12 {
            break;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    stop_pumpers.store(true, Ordering::Release);
    for t in pumpers {
        t.join().unwrap();
    }

    // Verify invariants: <= 1 terminal per admitted request, ACK before terminal, exact mapping.
    let evs = events.lock().unwrap().clone();
    let mut ack_seq: HashMap<RequestId, u64> = HashMap::new();
    let mut term_seen: HashSet<RequestId> = HashSet::new();
    for (seq, kind, client, req, aid) in evs {
        let (exp_client, exp_aid) = expected_map[&req];
        assert_eq!(client, exp_client, "wrong client for {req:?}");
        assert_eq!(aid, exp_aid, "request<->action mismatch for {req:?}");
        if kind == "ack" {
            assert!(ack_seq.insert(req, seq).is_none(), "duplicate ACK for {req:?}");
        } else {
            assert!(term_seen.insert(req), "DUPLICATE terminal for {req:?}");
            let a_seq = *ack_seq.get(&req).expect("terminal recorded before ACK");
            assert!(a_seq < seq, "terminal ({seq}) preceded ACK ({a_seq}) for {req:?}");
        }
    }
    assert_eq!(term_seen.len(), 12, "all 12 admitted requests must deliver 1 terminal");
    let stats = handle.reply_router_stats();
    assert_eq!(stats.global_reserved, 0, "{stats:?}");
    assert_eq!(stats.mapping_mismatches, 0, "{stats:?}");
    assert_eq!(stats.protocol_anomalies, 0, "{stats:?}");

    // Verify WsWireAckTracker::redact_obs redacts abandoned, pending-barrier, AND unknown-abandoned
    // terminals in BOTH `obs.action` and `obs.previous_action` (Blocker 1).
    let tracker = WsWireAckTracker::new();
    let mk_term = |id: u64| -> armd::motion::ActionStatus {
        serde_json::from_value(json!({
            "id": id,
            "kind": "move_to",
            "tag": null,
            "state": "done",
            "code": null,
            "message": null,
            "progress": 1.0,
            "started_at": 0.5,
            "finished_at": 1.0,
            "holding": null,
            "gripper_gap_rad": null,
            "settled": true
        }))
        .unwrap()
    };
    tracker.mark_abandoned(42);
    let mut obs = sup.hub().latest().unwrap().obs.clone();
    obs.action = Some(mk_term(42));
    obs.previous_action = Some(mk_term(42));
    tracker.redact_obs(&mut obs);
    assert!(obs.action.is_none(), "abandoned terminal in obs.action must be redacted");
    assert!(obs.previous_action.is_none(), "abandoned terminal in obs.previous_action must be redacted");

    // Pending ACK barrier redacts any newly-finished terminal (`id > floor`) in BOTH fields!
    tracker.enter_ack_barrier(50);
    obs.action = Some(mk_term(51));
    obs.previous_action = Some(mk_term(52));
    tracker.redact_obs(&mut obs);
    assert!(obs.action.is_none(), "pending-barrier terminal in obs.action must be redacted");
    assert!(obs.previous_action.is_none(), "pending-barrier terminal in obs.previous_action must be redacted");
    // Older pre-barrier terminal (`id <= 50`) stays visible.
    obs.action = Some(mk_term(50));
    tracker.redact_obs(&mut obs);
    assert!(obs.action.is_some(), "pre-barrier terminal must remain visible");
    // Unknown-ID abandonment keeps hiding `id > 50` even after leaving barrier until resolved!
    tracker.begin_unknown_abandoned();
    tracker.leave_ack_barrier();
    obs.action = Some(mk_term(53));
    obs.previous_action = Some(mk_term(54));
    tracker.redact_obs(&mut obs);
    assert!(obs.action.is_none(), "unknown-abandoned terminal must stay hidden");
    assert!(obs.previous_action.is_none(), "unknown-abandoned previous_action must stay hidden");
    tracker.resolve_unknown_abandoned(Some(53));
    obs.action = Some(mk_term(53));
    obs.previous_action = Some(mk_term(54));
    tracker.redact_obs(&mut obs);
    assert!(obs.action.is_none(), "resolved abandoned id 53 stays redacted");
    assert!(obs.previous_action.is_some(), "unrelated id 54 becomes visible once resolved");

    // Queued wire ACK (`unwritten_wire_acks`) keeps hiding terminal even AFTER `leave_ack_barrier()`
    // until `writer_task` itself finishes writing `command_ack` to the wire (R2-B1)!
    tracker.enter_ack_barrier(60);
    tracker.mark_ack_queued_for_wire(61);
    tracker.leave_ack_barrier();
    obs.action = Some(mk_term(61));
    obs.previous_action = Some(mk_term(61));
    tracker.redact_obs(&mut obs);
    assert!(obs.action.is_none(), "queued unwritten ACK must redact obs.action even after barrier drop");
    assert!(obs.previous_action.is_none(), "queued unwritten ACK must redact obs.previous_action");
    tracker.mark_wire_ack_written(61);
    obs.action = Some(mk_term(61));
    tracker.redact_obs(&mut obs);
    assert!(obs.action.is_some(), "after writer writes ACK, terminal becomes visible");

    // Verify Broker::try_poll recovers reservations on early Terminal & malformed Ack{action:None}.
    {
        let (mut b, mut rt) = armd::rt_core::broker::channel(
            ArmId::LOCAL,
            BootId(1),
            Capacities {
                commands: 4,
                plans: 2,
                replies: 8,
            },
            PlanSlot::default,
        );
        let dl_b = Instant::now() + Duration::from_secs(5);
        let r1 = b
            .submit(
                ClientId(1),
                1,
                Authority::default(),
                dl_b,
                Submit::Action(ActionSubmit::BeginMacro(BeginMacroSpec::new(
                    MacroKind::Place,
                    BootId(1),
                ))),
            )
            .unwrap();
        assert_eq!(b.reserved(), 2);
        // Inject an early Terminal before ACK directly via emit_reply -> Broker recovers both credits!
        rt.emit_reply(Reply::Terminal {
            request: r1,
            action: ActionId(9),
            outcome: Outcome::Completed,
            children: 0,
        });
        let mut got = Vec::new();
        b.try_poll(|_, rep| {
            got.push(rep);
            DeliveryDecision::Delivered
        });
        assert_eq!(b.reserved(), 0, "early terminal must recover broker credits");
        assert_eq!(b.outstanding(), 0);
        assert!(matches!(
            got.as_slice(),
            [Reply::Refused { request, code: "PROTOCOL_ERROR" }] if *request == r1
        ));

        // Inject a malformed Ack { action: None } for a starts_action() command -> recovers both credits!
        let r2 = b
            .submit(
                ClientId(1),
                2,
                Authority::default(),
                dl_b,
                Submit::Action(ActionSubmit::BeginMacro(BeginMacroSpec::new(
                    MacroKind::Place,
                    BootId(1),
                ))),
            )
            .unwrap();
        rt.emit_reply(Reply::Ack {
            request: r2,
            commit_seq: 1,
            action: None,
        });
        b.try_poll(|_, _| DeliveryDecision::Delivered);
        assert_eq!(b.reserved(), 0, "malformed Ack(None) must recover both broker credits");
        assert_eq!(b.outstanding(), 0);
    }

    handle.stop_and_join();
}

// ============================================================================
// 4. FakeBus stop case over live WS on OWNER_EXECUTOR
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn pf1_fakebus_stop_case_delivers_ack_before_terminal_and_reclaims_credits() {
    let bus = FakeBus::new(&dm_limits());
    bus.with(|s| {
        s.immediate_replies = true;
        for (i, &q) in INSPECT.iter().enumerate() {
            s.motors[i].q = q;
        }
        s.push_all_feedback();
    });
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    drv.read_joint_states().unwrap();
    drv.check_mapping_ranges();

    let sup = Arc::new(
        ArmSupervisor::new(ArmConfig {
            collision: None,
            lease_timeout_ms: 60_000,
            ..ArmConfig::rebot_dm()
        })
        .with_driver(Box::new(drv)),
    );
    {
        let mut inner = sup.inner().write().await;
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::enable_torque(a).unwrap();
        inner.tick(0.02);
    }
    sup.start_dispatcher().unwrap();
    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .unwrap();
    wait_pub_ready(&sup).await;

    let router = create_router(sup.clone(), OperatorAuth::disabled());
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let srv = tokio::spawn(async move {
        let _ = axum::serve(listener, router).await;
    });

    let (mut ws, gen) = controller(addr).await;
    let ep = sup.hub().latest().unwrap().obs.epoch_id;
    let snap = ActionSnapshot::capture(
        &sup,
        sup.capture_admission_stamp(),
        Instant::now() + DEFAULT_ACTION_DEADLINE,
    )
    .await
    .unwrap();
    let tcp0 = armd::motion::start_pose(&snap.plan_input_in(armd::motion::GoalFrame::Tcp).unwrap()).p;

    // Start a slow WS move_to over DamiaoArmDriver<FakeBus>, verify ACK arrives first, then Stop!
    let ack = ws_reply(
        &mut ws,
        json!({
            "type": "command",
            "request_id": 1,
            "generation": gen,
            "epoch_id": ep,
            "path": "/api/v1/action/move_to",
            "body": {"position": [tcp0[0] + 0.04, tcp0[1], tcp0[2]], "speed_scale": 0.1}
        }),
        &["command_ack"],
    )
    .await;
    let aid = ack["action_id"].as_u64().expect("FakeBus WS move_to must return action_id");
    let tx_before_stop = bus.with(|s| s.tx_log.len());
    tokio::time::sleep(Duration::from_millis(25)).await;
    assert!(bus.with(|s| s.tx_log.len()) > tx_before_stop, "FakeBus must receive motion TX frames");

    let stop_ack = ws_reply(
        &mut ws,
        json!({
            "type": "command",
            "request_id": 2,
            "generation": gen,
            "epoch_id": ep,
            "path": "/api/v1/motion/stop",
            "body": {}
        }),
        &["command_ack"],
    )
    .await;
    assert_eq!(stop_ack["type"], "command_ack", "{stop_ack}");

    let term = wait_action_terminal_ws(&mut ws, aid, Duration::from_secs(5)).await;
    assert_eq!(term["state"], "cancelled", "FakeBus stop terminal: {term}");
    assert_eq!(sup.hub().latest().unwrap().obs.safety_state, SafetyState::Armed);

    tokio::time::sleep(Duration::from_millis(30)).await;
    handle.pump_broker_replies();
    let st = handle.reply_router_stats();
    assert_eq!(st.global_reserved, 0, "FakeBus stop leaked router reservations: {st:?}");
    assert_eq!(st.mapping_mismatches, 0);
    assert_eq!(st.protocol_anomalies, 0);

    let _ = ws.close(None).await;
    srv.abort();
    handle.stop_and_join();
}

// ============================================================================
// 5. Zero RT allocator operations from admission to retirement & slot reuse
// ============================================================================

struct ScriptedClock {
    base: Instant,
    state: Mutex<ClockState>,
    cv: Condvar,
}

struct ClockState {
    now_ns: u64,
    limit_ns: u64,
    waiting_for_ns: Option<u64>,
}

impl ScriptedClock {
    fn new() -> Arc<Self> {
        Arc::new(Self {
            base: Instant::now(),
            state: Mutex::new(ClockState {
                now_ns: 0,
                limit_ns: 0,
                waiting_for_ns: None,
            }),
            cv: Condvar::new(),
        })
    }

    fn run_steps(&self, period_ns: u64, steps: u64) {
        let mut s = self.state.lock().unwrap();
        let target = s.limit_ns + period_ns * steps;
        s.limit_ns = target;
        let expect_park = target + period_ns;
        self.cv.notify_all();
        let deadline = Instant::now() + Duration::from_secs(5);
        while s.waiting_for_ns != Some(expect_park) {
            let rem = deadline.saturating_duration_since(Instant::now());
            assert!(!rem.is_zero(), "timed out waiting for scripted clock park");
            let (next, _) = self.cv.wait_timeout(s, rem).unwrap();
            s = next;
        }
    }
}

impl ExecutorClock for ScriptedClock {
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
        while !stop.load(Ordering::Acquire) && s.now_ns < target_ns && s.limit_ns < target_ns {
            s.waiting_for_ns = Some(target_ns);
            self.cv.notify_all();
            s = self.cv.wait(s).unwrap();
        }
        s.waiting_for_ns = None;
        if !stop.load(Ordering::Acquire) && s.now_ns < target_ns {
            s.now_ns = target_ns;
        }
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

struct ZeroAllocDriver {
    dof: usize,
    q: Vec<f32>,
    enabled: bool,
    rx_at: Vec<Option<Instant>>,
    seqs: Vec<u64>,
    clock: Arc<ScriptedClock>,
}

impl ZeroAllocDriver {
    fn new(init: &[f32], clock: Arc<ScriptedClock>) -> Self {
        let dof = init.len();
        let now = clock.now();
        Self {
            dof,
            q: init.to_vec(),
            enabled: true,
            rx_at: vec![Some(now); dof],
            seqs: vec![1; dof],
            clock,
        }
    }
}

impl ArmDriver for ZeroAllocDriver {
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
        for t in &self.rx_at {
            out.push(t.map(|t| now.saturating_duration_since(t)));
        }
    }
    fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
        let now = self.clock.now();
        self.rx_at.len() == dof
            && self.rx_at.iter().all(|t| t.is_some_and(|t| now.saturating_duration_since(t) <= max_age))
    }
    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
        let now = self.clock.now();
        out.clear();
        for t in &self.rx_at {
            out.push(t.map(|t| now.saturating_duration_since(t).as_millis() as u64));
        }
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
    fn read_joint_states_budgeted(&mut self, out: &mut Vec<JointState>, _dl: Instant, _max: u16) -> Result<(u16, bool), ArmDriverError> {
        self.read_joint_states_into(out)?;
        Ok((self.dof as u16, false))
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        for (i, t) in targets.iter().enumerate().take(self.dof) {
            self.q[i] = t.position;
        }
        Ok(())
    }
    fn write_joint_targets_bounded(
        &mut self,
        targets: &[JointTarget],
    ) -> Result<armd::driver::TxBatchOutcome, ArmDriverError> {
        self.write_joint_targets(targets)?;
        Ok(armd::driver::TxBatchOutcome::Complete)
    }
    fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
        self.enabled = enable;
        Ok(())
    }
    fn begin_torque_budgeted(&mut self, enable: bool, _max: u16) -> Result<TorqueProgress, ArmDriverError> {
        self.enabled = enable;
        Ok(TorqueProgress::Done(TorqueOutcome::Confirmed))
    }
    fn poll_torque_budgeted(&mut self, _dl: Instant, _max: u16) -> Result<TorqueProgress, ArmDriverError> {
        Ok(TorqueProgress::Done(TorqueOutcome::Confirmed))
    }
    fn abort_enable(&mut self) {}
    fn torque_transition_pending(&self) -> bool { false }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn pf1_zero_rt_allocs_from_admission_to_retirement_and_slot_reuse() {
    let clock = ScriptedClock::new();
    let drv = ZeroAllocDriver::new(&INSPECT, clock.clone());
    let sup = Arc::new(
        ArmSupervisor::new(ArmConfig {
            collision: None,
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        })
        .with_driver(Box::new(drv)),
    );
    sup.set_lease_enabled(false).await;
    {
        let mut inner = sup.inner().write().await;
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::enable_torque(a).unwrap();
        inner.register_live_connection(501).unwrap();
        inner.register_live_connection(502).unwrap();
        inner.tick(0.02);
    }

    let period_ns = 20_000_000u64;
    let mut cfg = ExecutorConfig::for_tests(Duration::from_nanos(period_ns));
    cfg.alloc_timing = true;
    let handle: RtExecutorHandle = sup
        .start_executor_with_hooks(
            cfg,
            PersistTestHooks {
                clock: Some(clock.clone()),
                ..Default::default()
            },
        )
        .await
        .unwrap();
    clock.run_steps(period_ns, 3);
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);

    let term_count = Arc::new(AtomicUsize::new(0));
    let tc = Arc::clone(&term_count);
    let tok501 = handle
        .register_connection_reply_sink(
            ClientId(501),
            Arc::new(move |_, r| {
                if matches!(r, Reply::Terminal { .. }) {
                    tc.fetch_add(1, Ordering::SeqCst);
                }
                DeliveryDecision::Delivered
            }),
            Some(8),
        )
        .unwrap();

    // 1. External InstallPrepared -> RT execution -> Terminal -> retirement.
    let s0 = sup.hub().latest().unwrap();
    let env0 = sup.hub().prep_env().unwrap();
    let mut adm = sup.capture_admission_stamp();
    adm.connection_id = 501;
    let snap = ActionSnapshot::from_published(&s0, env0, adm, clock.now() + DEFAULT_ACTION_DEADLINE).unwrap();
    let g_prep = prepare_gripper(&snap, &json!({"command": "open", "speed": 2.0}), GripperBaselineCmd::None).unwrap();
    let g_slot = g_prep.to_plan_slot(None).unwrap();
    let wg = |dst: &mut PlanSlot| *dst = g_slot.clone();
    let (_, mut rx1, _) = handle
        .submit_external_action(
            ClientId(501),
            1,
            snap.owner.authority,
            clock.now() + DEFAULT_ACTION_DEADLINE,
            ActionSubmit::InstallPrepared {
                write: &wg,
                kind: PreparedActionKind::Gripper,
                prepared_boot: handle.boot(),
                admission_request_id: adm.request_id,
                check_workspace_revision: false,
                poses_revision: None,
                calibration_revision: None,
                gripper_calib_revision: g_prep.stamp.gripper_calib_revision,
                power_budget_revision: None,
            },
            ExternalDestinationPolicy::ConnectionOnly,
            false,
            None,
        )
        .unwrap();
    let max_steps = ((g_prep.expected_duration_s / 0.02).ceil() as usize).saturating_add(120);
    for _ in 0..max_steps {
        clock.run_steps(period_ns, 1);
        handle.pump_broker_replies();
        if term_count.load(Ordering::SeqCst) >= 1 {
            break;
        }
    }
    assert!(matches!(rx1.try_recv(), Ok(Ok(Reply::Ack { .. }))));
    assert_eq!(term_count.load(Ordering::SeqCst), 1);

    // 2. External BeginMacro -> Internal InstallChild -> Internal FinishMacro -> Terminal -> retirement.
    let mut adm2 = sup.capture_admission_stamp();
    adm2.connection_id = 501;
    let snap2 = ActionSnapshot::from_published(
        &sup.hub().latest().unwrap(),
        sup.hub().prep_env().unwrap(),
        adm2,
        clock.now() + DEFAULT_ACTION_DEADLINE,
    )
    .unwrap();
    let mut bspec = BeginMacroSpec::new(MacroKind::Sequence, handle.boot());
    bspec.admission_request_id = adm2.request_id;
    let (_, mut rx_bm, _) = handle
        .submit_external_action(
            ClientId(501),
            2,
            snap2.owner.authority,
            clock.now() + DEFAULT_ACTION_DEADLINE,
            ActionSubmit::BeginMacro(bspec),
            ExternalDestinationPolicy::ConnectionOnly,
            false,
            None,
        )
        .unwrap();
    clock.run_steps(period_ns, 2);
    handle.pump_broker_replies();
    let parent_id = match rx_bm.try_recv() {
        Ok(Ok(Reply::Ack { action: Some(id), .. })) => id,
        other => panic!("BeginMacro ACK failed: {other:?}"),
    };

    let mut tgt = snap2.commanded_positions.clone();
    tgt[0] += 0.02;
    let max_vel = vec![1.0f32; tgt.len()];
    let child_slot = prepare_child_joint_slot(
        &snap2.commanded_positions,
        &tgt,
        &max_vel,
        0.1,
        None,
        ChildSpec {
            ticket: ChildTicket {
                parent: parent_id,
                child_seq: 1,
            },
            timeout_s: 5.0,
            timeout_code: "TIMEOUT",
            contact: None,
        },
    )
    .unwrap();
    let mut parent_auth = snap2.owner.authority;
    parent_auth.parent_generation = parent_id.0;
    let wc = |dst: &mut PlanSlot| *dst = child_slot.clone();
    let (_, mut rx_ch) = handle
        .submit_internal_action(
            ClientId(501),
            3,
            parent_auth,
            clock.now() + DEFAULT_ACTION_DEADLINE,
            ActionSubmit::InstallChild {
                write: &wc,
                prepared_boot: handle.boot(),
                check_workspace_revision: true,
                calibration_revision: None,
                grasp_setup_revision: None,
                gripper_calib_revision: None,
                power_budget_revision: None,
            },
        )
        .unwrap();
    for _ in 0..40 {
        clock.run_steps(period_ns, 1);
        handle.pump_broker_replies();
    }
    assert!(matches!(rx_ch.try_recv(), Ok(Reply::Ack { .. })));

    let (_, mut rx_fin) = handle
        .submit_internal_action(
            ClientId(501),
            4,
            parent_auth,
            clock.now() + DEFAULT_FINISH_DEADLINE,
            ActionSubmit::FinishMacro(FinishMacroSpec::done(parent_id, handle.boot())),
        )
        .unwrap();
    for _ in 0..10 {
        clock.run_steps(period_ns, 1);
        handle.pump_broker_replies();
        if term_count.load(Ordering::SeqCst) >= 2 {
            break;
        }
    }
    assert!(matches!(rx_fin.try_recv(), Ok(Reply::Ack { .. })));
    assert_eq!(term_count.load(Ordering::SeqCst), 2);

    // 3. Disconnect 501 concurrently while advancing ScriptedClock (no single-thread wait deadlock!),
    //    reuse slot for 502, and verify strictly 0 RT allocs across the entire run!
    let h_disc = handle.clone();
    let disc_thr = std::thread::spawn(move || {
        h_disc.disconnect_connection_sync(ConnectionHandle::from_ws_id(501).unwrap())
    });
    let mut disc_done = false;
    for _ in 0..30 {
        clock.run_steps(period_ns, 1);
        handle.pump_broker_replies();
        if disc_thr.is_finished() {
            disc_done = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    assert!(disc_done, "disconnect_connection_sync must complete boundedly as clock advances");
    let disc_res = disc_thr.join().unwrap();
    assert!(disc_res.is_ok(), "disconnect_connection_sync failed: {disc_res:?}");

    let tok502 = handle
        .register_connection_reply_sink(
            ClientId(502),
            Arc::new(|_, _| DeliveryDecision::Delivered),
            Some(8),
        )
        .unwrap();
    assert_eq!(tok502.slot_idx, tok501.slot_idx);
    assert!(tok502.slot_gen > tok501.slot_gen);
    clock.run_steps(period_ns, 3);
    handle.pump_broker_replies();

    assert_eq!(
        handle.stats().total_allocs(),
        AllocCounts::ZERO,
        "B2 violation: RT thread allocated during P-F1 admission->terminal->retirement->slot-reuse"
    );
    assert_eq!(handle.reply_router_stats().global_reserved, 0);

    handle.stop_and_join();
}

// ============================================================================
// 6. Callback re-entry stop_and_join, emergency teardown bound, and Open rollback
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn pf1_reentry_shutdown_emergency_teardown_and_open_rollback() {
    // Part A: Same-thread callback re-entry calling stop_and_join() must defer finalization without deadlock.
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig {
            collision: None,
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        }));
        sup.set_lease_enabled(false).await;
        {
            let mut inner = sup.inner().write().await;
            inner.commanded_positions = INSPECT.to_vec();
            inner.measured_positions = INSPECT.to_vec();
            inner.target_positions = INSPECT.to_vec();
            let s = inner.current_admission_stamp();
            let a = inner.check_admission(s).unwrap();
            SupervisorInner::enable_torque(a).unwrap();
            inner.tick(0.02);
        }
        let handle = sup
            .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
            .await
            .unwrap();
        wait_pub_ready(&sup).await;

        sup.register_ws_connection(601).await.unwrap();
        let stopped_inside_cb = Arc::new(AtomicBool::new(false));
        let flag = Arc::clone(&stopped_inside_cb);
        let h_cb = handle.clone();
        handle
            .register_connection_reply_sink(
                ClientId(601),
                Arc::new(move |_, r| {
                    if matches!(r, Reply::Terminal { .. }) && !flag.swap(true, Ordering::SeqCst) {
                        // Re-entrant stop_and_join on the exact thread holding SerialDrainGate!
                        h_cb.stop_and_join();
                    }
                    DeliveryDecision::Delivered
                }),
                Some(4),
            )
            .unwrap();

        let mut adm = sup.capture_admission_stamp();
        adm.connection_id = 601;
        let snap = ActionSnapshot::capture(&sup, adm, Instant::now() + DEFAULT_ACTION_DEADLINE)
            .await
            .unwrap();
        let mut spec = BeginMacroSpec::new(MacroKind::Place, handle.boot());
        spec.admission_request_id = adm.request_id;
        let (_, mut rx, _g) = handle
            .submit_external_action(
                ClientId(601),
                1,
                snap.owner.authority,
                Instant::now() + DEFAULT_ACTION_DEADLINE,
                ActionSubmit::BeginMacro(spec),
                ExternalDestinationPolicy::ConnectionOnly,
                false,
                None,
            )
            .unwrap();
        let aid = loop {
            handle.pump_broker_replies();
            if let Ok(Ok(Reply::Ack { action: Some(id), .. })) = rx.try_recv() {
                break id;
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        };
        let mut fin_auth = snap.owner.authority;
        fin_auth.parent_generation = aid.0;
        let (_, mut frx) = handle
            .submit_internal_action(
                ClientId(601),
                2,
                fin_auth,
                Instant::now() + DEFAULT_FINISH_DEADLINE,
                ActionSubmit::FinishMacro(FinishMacroSpec::done(aid, handle.boot())),
            )
            .unwrap();
        let dl = Instant::now() + Duration::from_secs(5);
        while Instant::now() < dl && !stopped_inside_cb.load(Ordering::Acquire) {
            handle.pump_broker_replies();
            let _ = frx.try_recv();
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        assert!(stopped_inside_cb.load(Ordering::Acquire), "callback must fire and stop without deadlocking");
        assert_eq!(handle.reply_router_stats().global_reserved, 0);
    }

    // Part B: Emergency teardown with a blocked callback on another thread must return boundedly!
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig {
            collision: None,
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        }));
        sup.set_lease_enabled(false).await;
        {
            let mut inner = sup.inner().write().await;
            inner.commanded_positions = INSPECT.to_vec();
            inner.measured_positions = INSPECT.to_vec();
            inner.target_positions = INSPECT.to_vec();
            let s = inner.current_admission_stamp();
            let a = inner.check_admission(s).unwrap();
            SupervisorInner::enable_torque(a).unwrap();
            inner.tick(0.02);
        }
        let handle = sup
            .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
            .await
            .unwrap();
        wait_pub_ready(&sup).await;

        sup.register_ws_connection(701).await.unwrap();
        let entered = Arc::new(AtomicBool::new(false));
        let release = Arc::new(AtomicBool::new(false));
        let ent = Arc::clone(&entered);
        let rel = Arc::clone(&release);
        handle
            .register_connection_reply_sink(
                ClientId(701),
                Arc::new(move |_, r| {
                    if matches!(r, Reply::Terminal { .. }) {
                        ent.store(true, Ordering::Release);
                        while !rel.load(Ordering::Acquire) {
                            std::thread::sleep(Duration::from_millis(5));
                        }
                    }
                    DeliveryDecision::Delivered
                }),
                Some(4),
            )
            .unwrap();

        let mut adm = sup.capture_admission_stamp();
        adm.connection_id = 701;
        let snap = ActionSnapshot::capture(&sup, adm, Instant::now() + DEFAULT_ACTION_DEADLINE)
            .await
            .unwrap();
        let mut spec = BeginMacroSpec::new(MacroKind::Place, handle.boot());
        spec.admission_request_id = adm.request_id;
        let (_, mut rx, _g) = handle
            .submit_external_action(
                ClientId(701),
                1,
                snap.owner.authority,
                Instant::now() + DEFAULT_ACTION_DEADLINE,
                ActionSubmit::BeginMacro(spec),
                ExternalDestinationPolicy::ConnectionOnly,
                false,
                None,
            )
            .unwrap();
        let aid = loop {
            handle.pump_broker_replies();
            if let Ok(Ok(Reply::Ack { action: Some(id), .. })) = rx.try_recv() {
                break id;
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        };
        let mut fin_auth = snap.owner.authority;
        fin_auth.parent_generation = aid.0;
        let (_, _) = handle
            .submit_internal_action(
                ClientId(701),
                2,
                fin_auth,
                Instant::now() + DEFAULT_FINISH_DEADLINE,
                ActionSubmit::FinishMacro(FinishMacroSpec::done(aid, handle.boot())),
            )
            .unwrap();

        let h_pump = handle.clone();
        let ent_wait = Arc::clone(&entered);
        let pump_thr = std::thread::spawn(move || {
            let dl = Instant::now() + Duration::from_secs(3);
            while !ent_wait.load(Ordering::Acquire) && Instant::now() < dl {
                h_pump.pump_broker_replies();
                std::thread::sleep(Duration::from_millis(2));
            }
        });
        let wait_dl = Instant::now() + Duration::from_secs(3);
        while !entered.load(Ordering::Acquire) && Instant::now() < wait_dl {
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        assert!(entered.load(Ordering::Acquire), "sink callback must be blocked in-flight");

        let t0 = Instant::now();
        let _ = handle.emergency_disable_and_join();
        let elapsed = t0.elapsed();
        assert!(
            elapsed < Duration::from_secs(2),
            "emergency teardown must remain bounded while sink callback is blocked, took {elapsed:?}"
        );
        release.store(true, Ordering::Release);
        pump_thr.join().unwrap();
        // Wait boundedly for whichever thread (`pump_thr` or `armd-dispatch`) owned the callback
        // to complete `reconcile_in_flight` and release the in-flight reservation credit.
        let rec_dl = Instant::now() + Duration::from_secs(2);
        while Instant::now() < rec_dl && handle.reply_router_stats().global_reserved != 0 {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        assert_eq!(handle.reply_router_stats().global_reserved, 0);
    }

    // Part C: Cancelled open_connection_with_reply_sink_async rolls back router & RT slots cleanly.
    {
        let clock = ScriptedClock::new();
        let drv = ZeroAllocDriver::new(&INSPECT, clock.clone());
        let sup = Arc::new(
            ArmSupervisor::new(ArmConfig {
                collision: None,
                lease_timeout_ms: 600_000,
                ..ArmConfig::rebot_dm()
            })
            .with_driver(Box::new(drv)),
        );
        sup.set_lease_enabled(false).await;
        {
            let mut inner = sup.inner().write().await;
            inner.commanded_positions = INSPECT.to_vec();
            inner.measured_positions = INSPECT.to_vec();
            inner.target_positions = INSPECT.to_vec();
            inner.tick(0.02);
        }
        let period_ns = 20_000_000u64;
        let handle = sup
            .start_executor_with_hooks(
                ExecutorConfig::for_tests(Duration::from_nanos(period_ns)),
                PersistTestHooks {
                    clock: Some(clock.clone()),
                    ..Default::default()
                },
            )
            .await
            .unwrap();
        clock.run_steps(period_ns, 2);

        // Start open_connection_with_reply_sink_async while RT clock is parked, then cancel it!
        let fut = handle.open_connection_with_reply_sink_async(
            801,
            Arc::new(|_, _| DeliveryDecision::Delivered),
        );
        let timed = tokio::time::timeout(Duration::from_millis(20), fut).await;
        assert!(timed.is_err(), "open future should still be awaiting RT while clock is parked");
        // Drop of `fut` triggers OpenRollbackGuard -> router slot removed & disconnect enqueued!
        assert!(
            !handle.has_connection_reply_sink(ClientId(801)),
            "cancelled open must immediately roll back router slot"
        );
        // Step RT so queued Open + rolled-back Disconnect both execute on RT.
        clock.run_steps(period_ns, 4);
        handle.pump_broker_replies();
        // Prove the original RT slot for 801 was actually freed by filling ALL MAX_LIVE_CONNECTIONS
        // slots cleanly (802..=801+MAX_LIVE_CONNECTIONS); if 801 had leaked on RT, the 64th open
        // would fail with TOO_MANY_CONNECTIONS!
        for offset in 1..=(armd::supervisor::MAX_LIVE_CONNECTIONS as u64) {
            let cid = 801 + offset;
            let h_open = handle.clone();
            let open_task = tokio::spawn(async move {
                h_open
                    .open_connection_with_reply_sink_async(
                        cid,
                        Arc::new(|_, _| DeliveryDecision::Delivered),
                    )
                    .await
            });
            for _ in 0..10 {
                clock.run_steps(period_ns, 1);
                handle.pump_broker_replies();
                if open_task.is_finished() {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
            assert_eq!(
                open_task.await.unwrap().unwrap(),
                Ok(()),
                "slot {offset} failed to open — cancelled Open(801) leaked an RT slot"
            );
            assert!(handle.has_connection_reply_sink(ClientId(cid)));
        }
        handle.stop_and_join();
    }

    // Part D (Blocker 3): Emergency teardown with an IDLE gate + queued blocking sink must NOT
    // invoke the blocking sink and must return well within the emergency bound!
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig {
            collision: None,
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        }));
        sup.set_lease_enabled(false).await;
        {
            let mut inner = sup.inner().write().await;
            inner.commanded_positions = INSPECT.to_vec();
            inner.measured_positions = INSPECT.to_vec();
            inner.target_positions = INSPECT.to_vec();
            let s = inner.current_admission_stamp();
            let a = inner.check_admission(s).unwrap();
            SupervisorInner::enable_torque(a).unwrap();
            inner.tick(0.02);
        }
        let handle = sup
            .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
            .await
            .unwrap();
        wait_pub_ready(&sup).await;

        sup.register_ws_connection(901).await.unwrap();
        let caller_tid = std::thread::current().id();
        let in_emergency = Arc::new(AtomicBool::new(false));
        let cb_invoked_on_emergency_caller = Arc::new(AtomicBool::new(false));
        let em_flag = Arc::clone(&in_emergency);
        let invoked = Arc::clone(&cb_invoked_on_emergency_caller);
        handle
            .register_connection_reply_sink(
                ClientId(901),
                Arc::new(move |_, _| {
                    if em_flag.load(Ordering::Acquire) && std::thread::current().id() == caller_tid {
                        invoked.store(true, Ordering::Release);
                        std::thread::sleep(Duration::from_secs(5));
                    }
                    // Keep terminal queued in `slot.pending` while releasing `SerialDrainGate`!
                    DeliveryDecision::Backpressured
                }),
                Some(4),
            )
            .unwrap();

        let mut adm = sup.capture_admission_stamp();
        adm.connection_id = 901;
        let snap = ActionSnapshot::capture(&sup, adm, Instant::now() + DEFAULT_ACTION_DEADLINE)
            .await
            .unwrap();
        let mut spec = BeginMacroSpec::new(MacroKind::Place, handle.boot());
        spec.admission_request_id = adm.request_id;
        let (_, mut rx, _g) = handle
            .submit_external_action(
                ClientId(901),
                1,
                snap.owner.authority,
                Instant::now() + DEFAULT_ACTION_DEADLINE,
                ActionSubmit::BeginMacro(spec),
                ExternalDestinationPolicy::ConnectionOnly,
                false,
                None,
            )
            .unwrap();
        let aid = loop {
            handle.pump_broker_replies();
            if let Ok(Ok(Reply::Ack { action: Some(id), .. })) = rx.try_recv() {
                break id;
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        };
        let mut fin_auth = snap.owner.authority;
        fin_auth.parent_generation = aid.0;
        let _ = handle
            .submit_internal_action(
                ClientId(901),
                2,
                fin_auth,
                Instant::now() + DEFAULT_FINISH_DEADLINE,
                ActionSubmit::FinishMacro(FinishMacroSpec::done(aid, handle.boot())),
            )
            .unwrap();
        // Wait until the Terminal is queued in `slot.pending` and `SerialDrainGate` is IDLE!
        let q_dl = Instant::now() + Duration::from_secs(3);
        while Instant::now() < q_dl
            && handle.reply_router_stats().pending_external_deliveries == 0
        {
            handle.pump_broker_replies();
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        assert_eq!(handle.reply_router_stats().pending_external_deliveries, 1);

        in_emergency.store(true, Ordering::Release);
        let t0 = Instant::now();
        let _ = handle.emergency_disable_and_join();
        let elapsed = t0.elapsed();
        assert!(
            elapsed < Duration::from_secs(2),
            "emergency teardown with idle gate must stay bounded, took {elapsed:?}"
        );
        assert!(
            !cb_invoked_on_emergency_caller.load(Ordering::Acquire),
            "emergency teardown MUST NOT invoke sink callbacks on the emergency caller"
        );
        let rec_dl = Instant::now() + Duration::from_secs(2);
        while Instant::now() < rec_dl && handle.reply_router_stats().global_reserved != 0 {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        assert_eq!(handle.reply_router_stats().global_reserved, 0);
    }

    // Part E (Major 1) & Part F (Major 2): Callback panic unwinds cleanly without wedging
    // SerialDrainGate, AND callback calling disconnect_connection_sync fails fast without deadlock!
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig {
            collision: None,
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        }));
        sup.set_lease_enabled(false).await;
        {
            let mut inner = sup.inner().write().await;
            inner.commanded_positions = INSPECT.to_vec();
            inner.measured_positions = INSPECT.to_vec();
            inner.target_positions = INSPECT.to_vec();
            let s = inner.current_admission_stamp();
            let a = inner.check_admission(s).unwrap();
            SupervisorInner::enable_torque(a).unwrap();
            inner.tick(0.02);
        }
        let handle = sup
            .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
            .await
            .unwrap();
        wait_pub_ready(&sup).await;

        // Major 2: re-entrant disconnect_connection_sync inside sink callback fails fast!
        sup.register_ws_connection(951).await.unwrap();
        let reentrant_err = Arc::new(Mutex::new(None));
        let re_out = Arc::clone(&reentrant_err);
        let h_cb = handle.clone();
        handle
            .register_connection_reply_sink(
                ClientId(951),
                Arc::new(move |_, r| {
                    if matches!(r, Reply::Terminal { .. }) {
                        let res = h_cb.disconnect_connection_sync(
                            ConnectionHandle::from_ws_id(951).unwrap(),
                        );
                        *re_out.lock().unwrap() = Some(res);
                    }
                    DeliveryDecision::Delivered
                }),
                Some(4),
            )
            .unwrap();
        let mut adm1 = sup.capture_admission_stamp();
        adm1.connection_id = 951;
        let snap1 = ActionSnapshot::capture(&sup, adm1, Instant::now() + DEFAULT_ACTION_DEADLINE)
            .await
            .unwrap();
        let mut spec1 = BeginMacroSpec::new(MacroKind::Place, handle.boot());
        spec1.admission_request_id = adm1.request_id;
        let (_, mut rx1, _g1) = handle
            .submit_external_action(
                ClientId(951),
                1,
                snap1.owner.authority,
                Instant::now() + DEFAULT_ACTION_DEADLINE,
                ActionSubmit::BeginMacro(spec1),
                ExternalDestinationPolicy::ConnectionOnly,
                false,
                None,
            )
            .unwrap();
        let aid1 = loop {
            handle.pump_broker_replies();
            if let Ok(Ok(Reply::Ack { action: Some(id), .. })) = rx1.try_recv() {
                break id;
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        };
        let mut fa1 = snap1.owner.authority;
        fa1.parent_generation = aid1.0;
        let _ = handle
            .submit_internal_action(
                ClientId(951),
                2,
                fa1,
                Instant::now() + DEFAULT_FINISH_DEADLINE,
                ActionSubmit::FinishMacro(FinishMacroSpec::done(aid1, handle.boot())),
            )
            .unwrap();
        let dl1 = Instant::now() + Duration::from_secs(3);
        while Instant::now() < dl1 && reentrant_err.lock().unwrap().is_none() {
            handle.pump_broker_replies();
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        assert!(
            matches!(
                *reentrant_err.lock().unwrap(),
                Some(Err(armd::rt_core::requests::lease::ConnLifecycleError::ReentrantCallback))
            ),
            "re-entrant disconnect_connection_sync must fail fast with ReentrantCallback, got {:?}",
            *reentrant_err.lock().unwrap()
        );
        assert!(!handle.has_connection_reply_sink(ClientId(951)));

        // Major 1: panicking sink callback releases SerialDrainGate and recovers credits!
        sup.register_ws_connection(952).await.unwrap();
        let panic_caller_tid = std::thread::current().id();
        handle
            .register_connection_reply_sink(
                ClientId(952),
                Arc::new(move |_, r| {
                    if matches!(r, Reply::Terminal { .. }) {
                        if std::thread::current().id() != panic_caller_tid {
                            return DeliveryDecision::Backpressured;
                        }
                        panic!("simulated sink callback panic");
                    }
                    DeliveryDecision::Delivered
                }),
                Some(4),
            )
            .unwrap();
        let mut adm2 = sup.capture_admission_stamp();
        adm2.connection_id = 952;
        let snap2 = ActionSnapshot::capture(&sup, adm2, Instant::now() + DEFAULT_ACTION_DEADLINE)
            .await
            .unwrap();
        let mut spec2 = BeginMacroSpec::new(MacroKind::Place, handle.boot());
        spec2.admission_request_id = adm2.request_id;
        let (_, mut rx2, _g2) = handle
            .submit_external_action(
                ClientId(952),
                1,
                snap2.owner.authority,
                Instant::now() + DEFAULT_ACTION_DEADLINE,
                ActionSubmit::BeginMacro(spec2),
                ExternalDestinationPolicy::ConnectionOnly,
                false,
                None,
            )
            .unwrap();
        let aid2 = loop {
            handle.pump_broker_replies();
            if let Ok(Ok(Reply::Ack { action: Some(id), .. })) = rx2.try_recv() {
                break id;
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        };
        let mut fa2 = snap2.owner.authority;
        fa2.parent_generation = aid2.0;
        let _ = handle
            .submit_internal_action(
                ClientId(952),
                2,
                fa2,
                Instant::now() + DEFAULT_FINISH_DEADLINE,
                ActionSubmit::FinishMacro(FinishMacroSpec::done(aid2, handle.boot())),
            )
            .unwrap();
        let mut panicked = false;
        let dl2 = Instant::now() + Duration::from_secs(3);
        while Instant::now() < dl2 && !panicked {
            let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                handle.pump_broker_replies();
            }));
            if res.is_err() {
                panicked = true;
                break;
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        assert!(panicked, "expected sink callback panic");
        // Gate MUST NOT be wedged and reservation credits MUST have been recovered!
        let _ = handle.pump_broker_replies();
        assert_eq!(handle.reply_router_stats().global_reserved, 0);
        handle.stop_and_join();
    }

    // Part G (R2-B2): Sink calls stop_and_join() re-entrantly, returns Backpressured, then panics
    // when deferred normal shutdown retries the callback -> SerialDrainGuard unwinds cleanly,
    // completes callback-free deferred retirement, releases the gate, and does not wedge!
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig {
            collision: None,
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        }));
        sup.set_lease_enabled(false).await;
        {
            let mut inner = sup.inner().write().await;
            inner.commanded_positions = INSPECT.to_vec();
            inner.measured_positions = INSPECT.to_vec();
            inner.target_positions = INSPECT.to_vec();
            let s = inner.current_admission_stamp();
            let a = inner.check_admission(s).unwrap();
            SupervisorInner::enable_torque(a).unwrap();
            inner.tick(0.02);
        }
        let handle = sup
            .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
            .await
            .unwrap();
        wait_pub_ready(&sup).await;

        sup.register_ws_connection(961).await.unwrap();
        let g_caller_tid = std::thread::current().id();
        let calls = Arc::new(AtomicUsize::new(0));
        let calls_cb = Arc::clone(&calls);
        let h_cb = handle.clone();
        handle
            .register_connection_reply_sink(
                ClientId(961),
                Arc::new(move |_, r| {
                    if matches!(r, Reply::Terminal { .. }) {
                        if std::thread::current().id() != g_caller_tid {
                            return DeliveryDecision::Backpressured;
                        }
                        let n = calls_cb.fetch_add(1, Ordering::SeqCst);
                        if n == 0 {
                            h_cb.stop_and_join();
                            return DeliveryDecision::Backpressured;
                        }
                        panic!("simulated panic during deferred normal shutdown retry");
                    }
                    DeliveryDecision::Delivered
                }),
                Some(4),
            )
            .unwrap();

        let mut adm = sup.capture_admission_stamp();
        adm.connection_id = 961;
        let snap = ActionSnapshot::capture(&sup, adm, Instant::now() + DEFAULT_ACTION_DEADLINE)
            .await
            .unwrap();
        let mut spec = BeginMacroSpec::new(MacroKind::Place, handle.boot());
        spec.admission_request_id = adm.request_id;
        let (_, mut rx, _g) = handle
            .submit_external_action(
                ClientId(961),
                1,
                snap.owner.authority,
                Instant::now() + DEFAULT_ACTION_DEADLINE,
                ActionSubmit::BeginMacro(spec),
                ExternalDestinationPolicy::ConnectionOnly,
                false,
                None,
            )
            .unwrap();
        let aid = loop {
            handle.pump_broker_replies();
            if let Ok(Ok(Reply::Ack { action: Some(id), .. })) = rx.try_recv() {
                break id;
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        };
        let mut fa = snap.owner.authority;
        fa.parent_generation = aid.0;
        let _ = handle
            .submit_internal_action(
                ClientId(961),
                2,
                fa,
                Instant::now() + DEFAULT_FINISH_DEADLINE,
                ActionSubmit::FinishMacro(FinishMacroSpec::done(aid, handle.boot())),
            )
            .unwrap();

        let mut panicked = false;
        let dl = Instant::now() + Duration::from_secs(3);
        while Instant::now() < dl && !panicked {
            let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                handle.pump_broker_replies();
            }));
            if res.is_err() {
                panicked = true;
                break;
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        assert!(panicked, "expected deferred-shutdown retry callback panic");
        assert!(calls.load(Ordering::SeqCst) >= 2);
        // Gate MUST NOT be wedged and credits MUST be 0!
        assert_eq!(handle.reply_router_stats().global_reserved, 0);
        let _ = handle.pump_broker_replies();
        handle.stop_and_join();
    }
}
