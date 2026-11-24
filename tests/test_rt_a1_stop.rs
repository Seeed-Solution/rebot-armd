//! Stage 3 §2 PR A1 acceptance tests:
//! - REST & WS E-stop (`trigger_e_stop_checked`, no synthetic epoch-0) and WS urgent Stop with all ordinary queues saturated
//! - Emergency E-stop intake behind 64+ backlogged ordinary WS frames & saturated outbound channel
//! - Stale/foreign callers, owner-serialized lease auth, REST lease restrictions, legacy stop semantics & startup gate
//! - Simultaneous REST/WS E-stops with intervening `clear_fault`, delayed Stop + E-stop non-regressing reply order
//! - No stale dynamic CAN frames across all 7 joints from exact owner-apply boundary (`DamiaoArmDriver<FakeBus>`)
//! - Live `alloc_timing=true` zero-alloc/zero-free RT stop/E-stop/apply_stop proof + positive control, refusal wake, park, panic-before-publish (Stop & E-stop)
mod common;
mod mit_support;

#[global_allocator]
static ALLOC: armd::diagnostics::alloc::TimedAlloc = armd::diagnostics::alloc::TimedAlloc;

use std::net::SocketAddr;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use common::*;
use mit_support::FakeBus;
use serde_json::json;
use tokio::net::TcpListener;
use tokio_tungstenite::connect_async;

use armd::driver::{ArmDriver, DamiaoArmDriver};
use armd::model::ControlMode;
use armd::protocol::damiao::DamiaoLimits;
use armd::rt_core::broker::{AdmitError, Authority, BootId, Capacities, ClientId, Reply, Submit};
use armd::rt_core::executor::{
    validate_startup_owner, ExecutorConfig, MaintenanceOp, MaintenanceResult, PanicPolicy,
    PersistTestHooks, RtExecutorHandle, RtOwner,
};
use armd::rt_core::ids::ArmId;
use armd::rt_core::mailbox::{StreamId, Submitted, TargetFrame, MAX_DOF};
use armd::rt_core::ops::{PlanSlot, Primitive, ReleaseEngine, ReleaseReport};
use armd::rt_core::urgent::{urgent_stop_channel, UrgentStop, UrgentStopOutcome};
use armd::{create_router, ArmConfig, ArmError, ArmSupervisor, OperatorAuth, SafetyState, SupervisorInner};

fn dm_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm().damiao_ranges.unwrap().iter().map(|r| r.limits()).collect()
}

async fn wait_until(timeout: Duration, mut pred: impl FnMut() -> bool) {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        if pred() { return; }
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
    assert!(pred(), "condition not satisfied within {timeout:?}");
}

async fn serve(sup: Arc<ArmSupervisor>) -> SocketAddr {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, create_router(sup, OperatorAuth::disabled())).await.unwrap(); });
    addr
}

fn fill_ordinary_queues(
    handle: &RtExecutorHandle,
    lease_gen: u64,
    epoch: u64,
    stop_gen: u64,
    start: &[f32],
    q: [f32; MAX_DOF],
) -> AdmitError {
    static NEXT_SEQ: AtomicU64 = AtomicU64::new(1);
    let auth = Authority { lease_generation: lease_gen, epoch, stop_generation: stop_gen, config_revision: 0, parent_generation: 0 };
    let dl = Instant::now() + Duration::from_secs(5);
    let mut s = [0.0f32; MAX_DOF];
    s[..start.len()].copy_from_slice(start);
    let dof = start.len() as u8;
    let (boot, sat_err) = {
        let mut b = handle.broker().lock().unwrap();
        let err = loop {
            let seq = NEXT_SEQ.fetch_add(1, Ordering::Relaxed);
            match b.submit(ClientId(9), seq, auth, dl, Submit::Install(&|slot: &mut PlanSlot| {
                slot.set(Primitive::JointQuintic { dof, start: s, q, duration_s: 0.5 });
            })) {
                Ok(_) => {}
                Err(e @ (AdmitError::CommandsFull | AdmitError::PlansFull | AdmitError::RepliesFull)) => break e,
                Err(e) => panic!("unexpected admit error: {e:?}"),
            }
        };
        (b.boot(), err)
    };
    let tseq = NEXT_SEQ.fetch_add(1, Ordering::Relaxed);
    let frame = TargetFrame {
        arm: ArmId::LOCAL, boot, stream: StreamId(1), seq: tseq, authority: auth, policy_generation: 0,
        input_at: Instant::now(), vision_at: None, vision_frame_id: None, deadline: dl, dof, q, replaceable: true,
    };
    let sub = handle.target_writer().lock().unwrap().submit(frame);
    assert!(matches!(sub, Submitted::Accepted | Submitted::Coalesced { .. }), "target submit failed: {sub:?}");
    sat_err
}

fn decode_pos_vel_p(f: &armd::transport::CanFrame) -> f32 {
    f32::from_le_bytes([f.data[0], f.data[1], f.data[2], f.data[3]])
}

// 1. REST & WS E-stop + WS urgent Stop under independent saturation of Commands/Plans/Replies/Maint + WS backlog overflow + waiter failure.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn rest_and_ws_estop_and_urgent_stop_with_ordinary_queues_full() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() }));
    let addr = serve(sup.clone()).await;
    let (mut ws, gen) = controller(addr).await;
    let ep0 = sup.get_observation().await.epoch_id;
    assert_eq!(rpc(&mut ws, gen, ep0, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
    let ep1 = sup.get_observation().await.epoch_id;
    assert_eq!(rpc(&mut ws, gen, ep1, "/api/v1/motion/jog", json!({"target_positions": [0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "duration_secs": 1.0})).await["type"], "command_ack");

    let delay = Arc::new(Mutex::new(None));
    let ecfg = ExecutorConfig { capacities: Capacities { commands: 1, plans: 1, replies: 2 }, ..ExecutorConfig::for_tests(Duration::from_millis(5)) };
    let handle = sup.start_executor_with_hooks(ecfg, PersistTestHooks { release_delay_once: Some(delay.clone()), ..Default::default() }).await.unwrap();
    let http = reqwest::Client::new();

    // Pause one release, saturate Broker (Commands/Plans/Replies) + TargetWriter + MaintClient simultaneously.
    *delay.lock().unwrap() = Some(Duration::from_millis(60));
    wait_until(Duration::from_secs(1), || delay.lock().unwrap().is_none()).await;
    let obs = sup.observation().await.unwrap();
    let sat = fill_ordinary_queues(&handle, gen, obs.epoch_id, 0, &obs.positions, [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    assert!(matches!(sat, AdmitError::CommandsFull | AdmitError::PlansFull | AdmitError::RepliesFull));
    // Also saturate maintenance queue (`MAINT_QUEUE_CAPACITY == 16`) with concurrent requests.
    let mut maint_tasks = Vec::new();
    for _ in 0..20 {
        let h = handle.clone();
        maint_tasks.push(tokio::spawn(async move { h.call_maintenance_async(MaintenanceOp::ShutdownReady).await }));
    }
    tokio::time::sleep(Duration::from_millis(8)).await;

    // Authorized WS Stop succeeds despite full Broker + Maint queues and refuses older command + target as STOPPED.
    let ack = rpc(&mut ws, gen, obs.epoch_id, "/api/v1/motion/stop", json!({})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let sg = ack["stop_generation"].as_u64().expect("stop_generation missing");
    assert!(sg >= 1 && sg == handle.stop_requests().applied());
    let (mut saw_cmd_stopped, mut saw_tgt_stopped) = (false, false);
    wait_until(Duration::from_secs(1), || {
        handle.broker().lock().unwrap().poll(|_, r| { if matches!(r, Reply::Refused { code: "STOPPED", .. }) { saw_cmd_stopped = true; } true });
        handle.target_writer().lock().unwrap().outcomes(|o| if o.result.err().is_some_and(|r| r.code == "STOPPED") { saw_tgt_stopped = true; });
        saw_cmd_stopped && saw_tgt_stopped
    }).await;
    for t in maint_tasks { let _ = t.await; }

    // REST POST /api/v1/safety/e_stop succeeds with full ordinary queues.
    let ep_pre = sup.observation().await.unwrap().epoch_id;
    *delay.lock().unwrap() = Some(Duration::from_millis(35));
    wait_until(Duration::from_secs(1), || delay.lock().unwrap().is_none()).await;
    fill_ordinary_queues(&handle, gen, ep_pre, sg, &obs.positions, [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    let rest_resp = http.post(format!("http://{addr}/api/v1/safety/e_stop")).send().await.unwrap();
    assert_eq!(rest_resp.status(), reqwest::StatusCode::OK);
    let rest_body: serde_json::Value = rest_resp.json().await.unwrap();
    let e1 = rest_body["epoch_id"].as_u64().unwrap();
    assert_eq!(rest_body["safety_state"], "EStop");
    assert!(e1 > ep_pre && e1 == sup.observation().await.unwrap().epoch_id);

    // Clear fault, saturate again, verify WS `estop` and `e_stop` both return exact applied epoch_sync.
    for kind in ["estop", "e_stop"] {
        assert_eq!(sup.clear_fault().await.unwrap(), SafetyState::Idle);
        let ep_c = sup.observation().await.unwrap().epoch_id;
        *delay.lock().unwrap() = Some(Duration::from_millis(30));
        wait_until(Duration::from_secs(1), || delay.lock().unwrap().is_none()).await;
        fill_ordinary_queues(&handle, gen, ep_c, sg, &obs.positions, [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        let r = ws_reply(&mut ws, json!({"type": kind}), &["epoch_sync", "estop_reject"]).await;
        assert_eq!((r["type"].as_str(), r["safety_state"].as_str()), (Some("epoch_sync"), Some("EStop")), "{r}");
        assert!(r["epoch_id"].as_u64().unwrap() > ep_c);
    }

    // Emergency E-stop behind >64 ordinary frames (backlog overflow) + full ordinary outbound buffer still latches immediately!
    assert_eq!(sup.clear_fault().await.unwrap(), SafetyState::Idle);
    let ep_ov = sup.observation().await.unwrap().epoch_id;
    *delay.lock().unwrap() = Some(Duration::from_millis(140));
    wait_until(Duration::from_secs(1), || delay.lock().unwrap().is_none()).await;
    send(&mut ws, json!({"type": "command", "request_id": 700, "generation": gen, "epoch_id": ep_ov, "path": "/api/v1/motion/stop", "body": {}})).await;
    for k in 0..70u64 {
        send(&mut ws, json!({"type": "command", "request_id": 701 + k, "generation": gen, "epoch_id": ep_ov, "path": "/api/v1/motion/jog", "body": {}})).await;
    }
    let t0 = Instant::now();
    send(&mut ws, json!({"type": "estop"})).await;
    wait_until(Duration::from_millis(60), || sup.e_stop_latched.load(Ordering::SeqCst)).await;
    assert!(t0.elapsed() < Duration::from_millis(90), "E-stop behind 70 backlogged frames stalled: {:?}", t0.elapsed());
    let es_ov = next_of(&mut ws, &["epoch_sync"]).await;
    assert_eq!(es_ov["safety_state"], "EStop");
    assert!(es_ov["epoch_id"].as_u64().unwrap() > ep_ov);
    drain(&mut ws, 20).await;

    // Stopped executor: WS `estop` & `e_stop` return explicit `estop_reject` (503, latched:true, no epoch_id:0).
    // Open a fresh WS connection because the 70-frame backlog burst above can legitimately saturate
    // the previous socket's outbound control queue and close it after flushing `epoch_sync`.
    assert_eq!(sup.clear_fault().await.unwrap(), SafetyState::Idle);
    handle.stop_and_join();
    let (mut ws_stopped, _) = tokio_tungstenite::connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
    let fail1 = ws_reply(&mut ws_stopped, json!({"type": "estop", "request_id": 77}), &["epoch_sync", "estop_reject"]).await;
    assert_eq!((fail1["type"].as_str(), fail1["status"].as_u64(), fail1["code"].as_str(), fail1["latched"].as_bool(), fail1["request_id"].as_u64()),
        (Some("estop_reject"), Some(503), Some("EXECUTOR_STOPPED"), Some(true), Some(77)), "{fail1}");
    assert!(fail1.get("epoch_id").is_none(), "must not synthesize epoch_id=0: {fail1}");
    let fail2 = ws_reply(&mut ws_stopped, json!({"type": "e_stop"}), &["epoch_sync", "estop_reject"]).await;
    assert_eq!((fail2["type"].as_str(), fail2["code"].as_str(), fail2["latched"].as_bool()),
        (Some("estop_reject"), Some("EXECUTOR_STOPPED"), Some(true)), "{fail2}");
    assert!(sup.e_stop_latched.load(Ordering::SeqCst));
}

// 2. Stale/foreign callers, owner-serialized lease auth, REST lease restrictions, legacy stop semantics & startup gate.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn stale_and_foreign_callers_rest_restrictions_and_legacy_semantics() {
    assert!(validate_startup_owner(RtOwner::Executor, "mock").is_ok());
    assert!(validate_startup_owner(RtOwner::Executor, "sim").is_err());
    assert!(validate_startup_owner(RtOwner::Executor, "damiao").is_err());
    assert!(validate_startup_owner(RtOwner::Legacy, "damiao").is_ok());

    // (a) Legacy REST lease restrictions + legacy WS stop (gravity-comp exit).
    let (addr_l, sup_l) = spawn_server(ArmConfig::rebot_dm()).await;
    let http = reqwest::Client::new();
    for (p, b) in [("torque", json!({"enable": true})), ("motion/home", json!({"epoch_id": 1}))] {
        let res = http.post(format!("http://{addr_l}/api/v1/{p}")).json(&b).send().await.unwrap();
        assert_eq!(res.status(), reqwest::StatusCode::CONFLICT);
        assert_eq!(res.json::<serde_json::Value>().await.unwrap()["code"], "LEASE_REQUIRED");
    }
    let (mut ws_l, gen_l) = controller(addr_l).await;
    let res = http.post(format!("http://{addr_l}/api/v1/motion/home")).json(&json!({"epoch_id": 1})).send().await.unwrap();
    assert_eq!((res.status(), res.json::<serde_json::Value>().await.unwrap()["code"].as_str()), (reqwest::StatusCode::CONFLICT, Some("LEASE_ACTIVE")));

    let ep_l = sup_l.get_observation().await.epoch_id;
    assert_eq!(rpc(&mut ws_l, gen_l, ep_l, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
    let ep_l = sup_l.get_observation().await.epoch_id;
    for _ in 0..3 { sup_l.tick(0.02).await; }
    assert_eq!(rpc(&mut ws_l, gen_l, ep_l, "/api/v1/mode/gravity_comp", json!({"enable": true, "scale": 0.9})).await["control_mode"], "gravity_comp");
    sup_l.tick(0.02).await;
    assert_eq!(sup_l.get_observation().await.control_mode, ControlMode::GravityComp);
    let stop_l = rpc(&mut ws_l, gen_l, ep_l, "/api/v1/motion/stop", json!({})).await;
    assert_eq!(stop_l["type"], "command_ack", "{stop_l}");
    for _ in 0..6 { sup_l.tick(0.02).await; }
    let obs_l = sup_l.get_observation().await;
    assert_eq!((obs_l.control_mode, obs_l.gravity_comp.last_exit.as_ref().map(|e| e.code.as_str())), (ControlMode::Position, Some("STOPPED")));

    // (b) Executor mode: foreign caller, spoofed gen, wrong epoch, bad request_id/body, expired lease, unmigrated gates.
    let sup = Arc::new(ArmSupervisor::new(ArmConfig { lease_timeout_ms: 600, ..ArmConfig::rebot_dm() }));
    let addr = serve(sup.clone()).await;
    let (mut ws_owner, gen) = controller(addr).await;
    let ep0 = sup.get_observation().await.epoch_id;
    assert_eq!(rpc(&mut ws_owner, gen, ep0, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
    let ep = sup.get_observation().await.epoch_id;
    assert_eq!(rpc(&mut ws_owner, gen, ep, "/api/v1/motion/jog", json!({"target_positions": [0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "duration_secs": 1.0})).await["type"], "command_ack");
    let handle = sup.start_executor(ExecutorConfig::for_tests(Duration::from_millis(5))).await.unwrap();

    let r_rest = http.post(format!("http://{addr}/api/v1/torque")).json(&json!({"enable": true})).send().await.unwrap();
    assert_eq!((r_rest.status(), r_rest.json::<serde_json::Value>().await.unwrap()["code"].as_str()), (reqwest::StatusCode::SERVICE_UNAVAILABLE, Some("EXECUTOR_ROUTING_PENDING")));

    let (mut ws_foreign, _) = connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
    let acq = ws_reply(&mut ws_foreign, json!({"type": "acquire_lease", "client_id": "intruder", "seq": 1}), &["lease_rejected"]).await;
    assert_eq!((acq["status"].as_u64(), acq["code"].as_str()), (Some(503), Some("EXECUTOR_ROUTING_PENDING")), "{acq}");
    let spoof = ws_reply(&mut ws_foreign, json!({"type": "command", "request_id": 1, "generation": gen, "client_id": "test_controller", "epoch_id": ep, "path": "/api/v1/motion/stop", "body": {}}), &["command_reject"]).await;
    assert_eq!((spoof["status"].as_u64(), spoof["code"].as_str(), spoof["epoch_id"].as_u64()), (Some(409), Some("LEASE_LOST"), Some(ep)), "{spoof}");
    assert_eq!(handle.stop_requests().current(), 0);

    let bad_ep = ws_reply(&mut ws_owner, json!({"type": "command", "request_id": 500, "generation": gen, "epoch_id": 999, "path": "/api/v1/motion/stop", "body": {}}), &["command_reject"]).await;
    assert_eq!((bad_ep["status"].as_u64(), bad_ep["code"].as_str()), (Some(409), Some("EPOCH_MISMATCH")), "{bad_ep}");
    let bad_body = ws_reply(&mut ws_owner, json!({"type": "command", "request_id": 500, "generation": gen, "epoch_id": ep, "path": "/api/v1/motion/stop", "body": 42}), &["command_reject"]).await;
    assert_eq!((bad_body["status"].as_u64(), bad_body["code"].as_str()), (Some(400), Some("INVALID_COMMAND")), "{bad_body}");
    let dup_req = ws_reply(&mut ws_owner, json!({"type": "command", "request_id": 500, "generation": gen, "epoch_id": ep, "path": "/api/v1/motion/stop", "body": {}}), &["command_reject"]).await;
    assert_eq!((dup_req["status"].as_u64(), dup_req["code"].as_str()), (Some(400), Some("INVALID_COMMAND")), "{dup_req}");
    assert_eq!(handle.stop_requests().current(), 0);

    tokio::time::sleep(Duration::from_millis(700)).await;
    let stale = ws_reply(&mut ws_owner, json!({"type": "command", "request_id": 501, "generation": gen, "epoch_id": ep, "path": "/api/v1/motion/stop", "body": {}}), &["command_reject"]).await;
    assert_eq!((stale["status"].as_u64(), stale["code"].as_str()), (Some(409), Some("LEASE_LOST")), "{stale}");
    assert_eq!(handle.stop_requests().current(), 0);
    handle.stop_and_join();
}

// 3. Simultaneous REST+WS E-stops with intervening clear_fault, AND delayed Stop + E-stop with monotone reply epochs.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn simultaneous_estops_with_intervening_clear_and_live_estop_during_delayed_stop() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() }));
    let addr = serve(sup.clone()).await;
    let (mut ws1, gen1) = controller(addr).await;
    let (mut ws2, _) = connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
    let ep0 = sup.get_observation().await.epoch_id;
    assert_eq!(rpc(&mut ws1, gen1, ep0, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");

    let delay = Arc::new(Mutex::new(None));
    let handle = sup.start_executor_with_hooks(
        ExecutorConfig::for_tests(Duration::from_millis(5)),
        PersistTestHooks { release_delay_once: Some(delay.clone()), ..Default::default() },
    ).await.unwrap();
    let http = reqwest::Client::new();
    let ep_start = sup.observation().await.unwrap().epoch_id;

    // (a) Simultaneous REST + WS `estop` + WS `e_stop` with intervening `clear_fault`.
    *delay.lock().unwrap() = Some(Duration::from_millis(60));
    wait_until(Duration::from_secs(1), || delay.lock().unwrap().is_none()).await;
    let f_rest = http.post(format!("http://{addr}/api/v1/safety/e_stop")).send();
    let f_ws1 = ws_reply(&mut ws1, json!({"type": "estop"}), &["epoch_sync", "estop_reject"]);
    let f_ws2 = ws_reply(&mut ws2, json!({"type": "e_stop"}), &["epoch_sync", "estop_reject"]);
    let sup_c = sup.clone();
    let f_clear = async move {
        wait_until(Duration::from_secs(1), || sup_c.e_stop_latched.load(Ordering::SeqCst)).await;
        tokio::time::sleep(Duration::from_millis(15)).await;
        sup_c.clear_fault().await
    };
    let (r_rest, r_ws1, r_ws2, r_clear) = tokio::join!(f_rest, f_ws1, f_ws2, f_clear);
    let b_rest: serde_json::Value = r_rest.unwrap().json().await.unwrap();
    let (e_r, e_1, e_2) = (b_rest["epoch_id"].as_u64().unwrap(), r_ws1["epoch_id"].as_u64().unwrap(), r_ws2["epoch_id"].as_u64().unwrap());
    assert_eq!((b_rest["safety_state"].as_str(), r_ws1["safety_state"].as_str(), r_ws2["safety_state"].as_str()), (Some("EStop"), Some("EStop"), Some("EStop")));
    assert!(e_r > ep_start && e_r == e_1 && e_1 == e_2, "first wave mismatch: {e_r}, {e_1}, {e_2}");
    assert_eq!(r_clear.unwrap(), SafetyState::Idle);
    let ep_clear = sup.observation().await.unwrap().epoch_id;
    assert!(ep_clear > e_r);
    let r_w2 = ws_reply(&mut ws2, json!({"type": "e_stop"}), &["epoch_sync"]).await;
    assert_eq!(r_w2["safety_state"], "EStop");
    assert!(r_w2["epoch_id"].as_u64().unwrap() > ep_clear);
    assert_eq!(sup.clear_fault().await.unwrap(), SafetyState::Idle);

    // (b) Same-socket WS E-stop while Stop reply is delayed: latches immediately AND delivered epochs never regress!
    let ep_live = sup.observation().await.unwrap().epoch_id;
    *delay.lock().unwrap() = Some(Duration::from_millis(120));
    wait_until(Duration::from_secs(1), || delay.lock().unwrap().is_none()).await;
    send(&mut ws1, json!({"type": "command", "request_id": 900, "generation": gen1, "epoch_id": ep_live, "path": "/api/v1/motion/stop", "body": {}})).await;
    tokio::time::sleep(Duration::from_millis(15)).await;
    let t0 = Instant::now();
    send(&mut ws1, json!({"type": "estop"})).await;
    wait_until(Duration::from_millis(45), || sup.e_stop_latched.load(Ordering::SeqCst)).await;
    assert!(t0.elapsed() < Duration::from_millis(80));
    let m1 = next_of(&mut ws1, &["epoch_sync", "command_ack", "command_reject"]).await;
    let m2 = next_of(&mut ws1, &["epoch_sync", "command_ack", "command_reject"]).await;
    let e_m1 = m1["epoch_id"].as_u64().unwrap();
    let e_m2 = m2["epoch_id"].as_u64().unwrap();
    assert!(e_m2 >= e_m1, "WS reply epoch regressed: m1={m1}, m2={m2}");
    if m1["type"] == "epoch_sync" && m2["type"] == "command_ack" {
        assert_eq!(m2["safety_state"], "EStop", "Stop ACK regressed safety_state after epoch_sync: {m2}");
        assert!(m2["stop_generation"].as_u64().unwrap_or(0) >= 1);
    }
    handle.stop_and_join();
}

// 4. No stale dynamic CAN frames across all 7 joints from exact owner-apply boundary (`DamiaoArmDriver<FakeBus>`).
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn no_stale_dynamic_frames_after_applied_stop_and_post_stop_frame_transmits() {
    let bus = FakeBus::new(&dm_limits());
    bus.with(|s| { s.immediate_replies = true; s.follow_commands = true; s.push_all_feedback(); });
    let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7]).with_joint_limits(dm_limits()).unwrap();
    dm.read_joint_states().unwrap();
    dm.check_mapping_ranges();
    let sup = Arc::new(ArmSupervisor::new(ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() }).with_driver(Box::new(dm)));
    let addr = serve(sup.clone()).await;
    let (mut ws, gen) = controller(addr).await;
    let ep0 = sup.get_observation().await.epoch_id;
    assert_eq!(rpc(&mut ws, gen, ep0, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
    let ep1 = sup.get_observation().await.epoch_id;
    assert_eq!(rpc(&mut ws, gen, ep1, "/api/v1/motion/jog", json!({"target_positions": [0.5, 0.3, -0.2, 0.25, -0.2, 0.15, 0.0], "duration_secs": 0.8})).await["type"], "command_ack");

    let delay = Arc::new(Mutex::new(None));
    let handle = sup.start_executor_with_hooks(
        ExecutorConfig::for_tests(Duration::from_millis(5)),
        PersistTestHooks { release_delay_once: Some(delay.clone()), ..Default::default() },
    ).await.unwrap();
    // Record exact 0-based `tx_log` index on every transmit under `FakeBus`'s mutex so
    // `tx_log` and the first post-apply boundary are snapshotted atomically with zero skipped frames.
    let stops_obs = handle.stop_requests().clone();
    let tx_seq = Arc::new(AtomicUsize::new(0));
    let first_post_apply_idx = Arc::new(AtomicUsize::new(usize::MAX));
    let seen_tx_after_apply = Arc::new(AtomicUsize::new(0));
    let (seq_cb, first_cb, cnt_cb) = (tx_seq.clone(), first_post_apply_idx.clone(), seen_tx_after_apply.clone());
    bus.with(|s| {
        seq_cb.store(s.tx_log.len(), Ordering::SeqCst);
        s.on_tx = Some(Arc::new(move || {
            let idx = seq_cb.fetch_add(1, Ordering::SeqCst);
            if stops_obs.applied() >= 1 {
                let _ = first_cb.compare_exchange(usize::MAX, idx, Ordering::SeqCst, Ordering::SeqCst);
                cnt_cb.fetch_add(1, Ordering::SeqCst);
            }
        }));
    });
    wait_until(Duration::from_secs(2), || bus.with(|s| s.motors[0].q > 0.02)).await;
    // Pause one release so queued broker command, target frame, and urgent Stop meet on the same release.
    *delay.lock().unwrap() = Some(Duration::from_millis(80));
    wait_until(Duration::from_secs(1), || delay.lock().unwrap().is_none()).await;
    let obs = sup.observation().await.unwrap();
    fill_ordinary_queues(&handle, gen, obs.epoch_id, 0, &obs.positions, [-0.5, -0.4, 0.4, -0.4, 0.4, -0.4, 0.0, 0.0]);
    let tx_before_stop_send = bus.with(|s| s.tx_log.len());

    let ack = rpc(&mut ws, gen, obs.epoch_id, "/api/v1/motion/stop", json!({})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let sg = ack["stop_generation"].as_u64().unwrap();
    assert!(sg >= 1);
    let q_stopped: [f32; 7] = bus.with(|s| std::array::from_fn(|i| s.motors[i].q));
    let st0 = handle.stats().steps.load(Ordering::Relaxed);
    wait_until(Duration::from_secs(1), || handle.stats().steps.load(Ordering::Relaxed) >= st0 + 6).await;

    let (mut saw_cmd_stopped, mut saw_tgt_stopped) = (false, false);
    wait_until(Duration::from_secs(1), || {
        handle.broker().lock().unwrap().poll(|_, r| { if matches!(r, Reply::Refused { code: "STOPPED", .. }) { saw_cmd_stopped = true; } true });
        handle.target_writer().lock().unwrap().outcomes(|o| if o.result.err().is_some_and(|r| r.code == "STOPPED") { saw_tgt_stopped = true; });
        saw_cmd_stopped && saw_tgt_stopped
    }).await;

    // Inspect EVERY dynamic POS_VEL frame across all 7 joints under one `FakeBus` lock so
    // `tx_log`, `tx_seq`, `first_post_apply_idx`, and `seen_tx_after_apply` match atomically.
    bus.with(|s| {
        let recorded_seq = tx_seq.load(Ordering::SeqCst);
        let first_idx = first_post_apply_idx.load(Ordering::SeqCst);
        let total_post_apply = seen_tx_after_apply.load(Ordering::SeqCst);
        assert_eq!(s.tx_log.len(), recorded_seq, "tx_log length diverged from on_tx sequence");
        assert_ne!(first_idx, usize::MAX, "no post-apply CAN frame observed");
        assert!(first_idx >= tx_before_stop_send, "first post-apply frame {first_idx} preceded stop send {tx_before_stop_send}");
        assert_eq!(s.tx_log.len() - first_idx, total_post_apply, "post-apply count diverged from tx_log slice");
        assert!(total_post_apply >= 7, "expected post-apply hold frames across all 7 joints, got {total_post_apply}");
        let mut per_joint = [0usize; 7];
        for f in &s.tx_log[first_idx..] {
            if (0x101..=0x107).contains(&f.id) {
                let j = (f.id - 0x101) as usize;
                let p = decode_pos_vel_p(f);
                per_joint[j] += 1;
                assert!((p - q_stopped[j]).abs() < 0.025, "joint {} stale frame p={p} vs q_stopped={}", j + 1, q_stopped[j]);
            }
        }
        assert!(per_joint.iter().all(|&c| c > 0), "expected hold frames on all 7 joints: {per_joint:?}");
    });

    // Fresh post-stop command admitted at `stop_generation == sg` commits and transmits to FakeBus.
    let snap = match handle.call_maintenance_async(MaintenanceOp::PlanningSnapshot).await.unwrap() {
        MaintenanceResult::PlanningSnapshot(s) => s,
        _ => panic!("unexpected maintenance reply"),
    };
    assert_eq!(snap.authority.stop_generation, sg);
    let mut start = [0.0f32; MAX_DOF];
    start[..7].copy_from_slice(&snap.commanded[..7]);
    let mut goal = start;
    goal[0] = (q_stopped[0] + 0.18).min(0.9);
    let req = handle.broker().lock().unwrap().submit(
        ClientId(10), 1, snap.authority, Instant::now() + Duration::from_secs(5),
        Submit::Install(&|slot: &mut PlanSlot| slot.set(Primitive::JointQuintic { dof: 7, start, q: goal, duration_s: 0.15 })),
    ).unwrap();
    let mut reply_for_req = None;
    wait_until(Duration::from_secs(1), || {
        handle.broker().lock().unwrap().poll(|_, r| {
            let rid = match r { Reply::Ack { request, .. } | Reply::Refused { request, .. } => Some(request), _ => None };
            if rid == Some(req) { reply_for_req = Some(r); }
            true
        });
        reply_for_req.is_some()
    }).await;
    assert!(matches!(reply_for_req, Some(Reply::Ack { .. })), "post-stop command refused: {reply_for_req:?}");
    wait_until(Duration::from_secs(2), || bus.with(|s| (s.motors[0].q - goal[0]).abs() < 0.02)).await;
    handle.stop_and_join();
}

// 5. Live `alloc_timing=true` zero-alloc/zero-free proof (+ positive control), refusal wake, park preservation, panic-before-publish (Stop + E-stop).
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn zero_alloc_rt_stop_refusal_wake_park_preservation_and_panic_safety() {
    let conn = "connection:alloc-test";
    let mk = |req_id, lg, e| UrgentStop {
        connection: armd::model::HolderLabel::new(conn), lease_generation: lg, epoch: Some(e),
        request_id: req_id, request_id_valid: true, body_error: None,
    };

    // Positive control: prove `TimedAlloc` actually counts heap allocations when enabled on this thread.
    armd::diagnostics::alloc::time_this_thread();
    let _ = armd::diagnostics::alloc::take();
    let probe_vec: Vec<u8> = vec![0x5A; 128];
    std::hint::black_box(&probe_vec);
    drop(probe_vec);
    let ctrl = armd::diagnostics::alloc::take();
    assert!(ctrl.calls > 0, "positive control failed: TimedAlloc did not count allocation");

    // Isolate exact RT sub-step allocation counts on stopping a running heap-owning jog.
    {
        let mut c = SupervisorInner::new(ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() });
        c.rt_executor_owned = true;
        c.prewarm_rt_buffers();
        let lg = c.acquire_lease(conn).unwrap();
        { let __s = c.current_admission_stamp(); c.check_admission(__s).and_then(|__a| c.enable_torque(__a)) }.unwrap();
        let e = c.safety.epoch_id();
        { let a = c.check_admission(c.current_admission_stamp()).unwrap(); c.jog_action_with_speed_scale(a, vec![0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], Some(1.0), Some(e), "joint_move", Some(0.5)) }.unwrap();
        for k in 1..=4 { let _ = c.tick_publish(0.005, k, false); let _ = c.core.hub().latest(); }
        let stops = armd::rt_core::urgent::StopRequests::new();
        let _ = armd::diagnostics::alloc::take();
        let _ = c.servo_step_workspace(&[0.0; 7], "braking");
        let a_ws = armd::diagnostics::alloc::take();
        let _ = armd::rt_core::ops::CoreOps::handle_urgent_stop(&mut c, &mk(1, lg, e), &stops, Instant::now());
        let a_stop = armd::diagnostics::alloc::take();
        let ts = c.tick_control(0.005);
        let a_tick = armd::diagnostics::alloc::take();
        let _ = c.publish_at(ts, 5, false);
        let a_pub = armd::diagnostics::alloc::take();
        assert_eq!((a_ws.calls, a_stop.calls, a_tick.calls, a_pub.calls), (0, 0, 0, 0), "sub-step allocs: ws={a_ws:?} stop={a_stop:?} tick={a_tick:?} pub={a_pub:?}");
    }

    // (a) `alloc_timing = true` across pre-handoff heap trajectory stop, direct `StopRequests` stop, E-stop, idle stop, refused stop.
    let sup = Arc::new(ArmSupervisor::new(ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() }));
    let gen = {
        let mut inner = sup.inner().write().await;
        let g = inner.acquire_lease(conn).unwrap();
        { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| inner.enable_torque(__a)) }.unwrap();
        let ep = inner.safety.epoch_id();
        { let a = inner.check_admission(inner.current_admission_stamp()).unwrap(); inner.jog_action_with_speed_scale(a, vec![0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], Some(1.0), Some(ep), "joint_move", Some(0.5)) }.unwrap();
        assert!(inner.active_trajectory.as_ref().unwrap().has_heap_storage(), "test requires heap trajectory");
        g
    };
    let mut ecfg_alloc = ExecutorConfig::for_tests(Duration::from_millis(5));
    ecfg_alloc.alloc_timing = true;
    let handle = sup.start_executor(ecfg_alloc.clone()).await.unwrap();
    // Warm up all 3 triple-buffer observation slots (including reader front slot) before measuring.
    wait_until(Duration::from_secs(1), || sup.tick_samples().len() >= 3).await;
    let _ = sup.observation().await.unwrap();
    wait_until(Duration::from_secs(1), || sup.tick_samples().len() >= 8).await;
    let pre_samples = sup.tick_samples().len();
    let ep = sup.observation().await.unwrap().epoch_id;
    assert!(matches!(sup.urgent_stop(mk(1, gen, ep)).await.unwrap(), UrgentStopOutcome::Applied { applied_to_motion: true, stop_generation: 1, .. }));
    assert!(matches!(sup.urgent_stop(mk(2, 0, ep)).await.unwrap(), UrgentStopOutcome::LeaseLost { .. }));
    assert!(matches!(sup.urgent_stop(mk(3, gen, ep)).await.unwrap(), UrgentStopOutcome::Applied { stop_generation: 2, .. }));
    let _direct_gen = handle.stop_requests().request();
    let st_done = handle.stats().steps.load(Ordering::Relaxed);
    wait_until(Duration::from_secs(1), || handle.stats().steps.load(Ordering::Relaxed) >= st_done + 4).await;
    wait_until(Duration::from_secs(1), || sup.tick_samples().len() >= pre_samples + 4).await;
    let samples = sup.tick_samples();
    let nonzero: Vec<(usize, u32)> = samples[pre_samples..].iter().enumerate().filter(|(_, s)| s.usage.lock_alloc_calls > 0).map(|(i, s)| (i, s.usage.lock_alloc_calls)).collect();
    assert!(nonzero.is_empty(), "RT urgent stop releases must do 0 allocs/frees (pre={pre_samples}, total={}): {nonzero:?}", samples.len());
    assert_eq!(handle.urgent_stop_in_flight(), 0);
    handle.stop_and_join();

    // (a2) Zero-alloc/zero-free RT E-stop retiring a pre-handoff heap-owning trajectory (`motion_limits`).
    // Measure `consume_pending_estop` + `tick_control` directly (0 allocs/frees), and on live
    // `armd-control` verify `drain_retired_states` drops the trajectory off-thread and steady-state
    // post-EStop releases do 0 allocs/frees (excluding the 1-per-slot pre-existing `FaultInfo.message`
    // String fill in `fill_observation` when `out.fault` transitions from `None` to `Some`).
    {
        let mut ce = SupervisorInner::new(ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() });
        ce.rt_executor_owned = true;
        ce.prewarm_rt_buffers();
        let _lg = ce.acquire_lease(conn).unwrap();
        { let __s = ce.current_admission_stamp(); ce.check_admission(__s).and_then(|__a| ce.enable_torque(__a)) }.unwrap();
        let e = ce.safety.epoch_id();
        { let a = ce.check_admission(ce.current_admission_stamp()).unwrap(); ce.jog_action_with_speed_scale(a, vec![0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], Some(1.0), Some(e), "joint_move", Some(0.5)) }.unwrap();
        assert!(ce.active_trajectory.as_ref().unwrap().has_heap_storage());
        for k in 1..=4 { let _ = ce.tick_publish(0.005, k, false); let _ = ce.core.hub().latest(); }
        ce.requested_estop_count.fetch_add(1, Ordering::SeqCst);
        let _ = armd::diagnostics::alloc::take();
        assert!(armd::rt_core::ops::CoreOps::apply_estop(&mut ce));
        let a_estop = armd::diagnostics::alloc::take();
        let _ = ce.tick_control(0.005);
        let a_tick = armd::diagnostics::alloc::take();
        assert_eq!((a_estop.calls, a_tick.calls), (0, 0), "RT E-stop retiring heap trajectory must do 0 allocs/frees: estop={a_estop:?} tick={a_tick:?}");
        assert!(ce.active_trajectory.is_none());
    }
    let sup_e = Arc::new(ArmSupervisor::new(ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() }));
    {
        let mut inner = sup_e.inner().write().await;
        let _g = inner.acquire_lease(conn).unwrap();
        { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| inner.enable_torque(__a)) }.unwrap();
        let ep = inner.safety.epoch_id();
        { let a = inner.check_admission(inner.current_admission_stamp()).unwrap(); inner.jog_action_with_speed_scale(a, vec![0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], Some(1.0), Some(ep), "joint_move", Some(0.5)) }.unwrap();
    }
    let h_e = sup_e.start_executor(ecfg_alloc).await.unwrap();
    wait_until(Duration::from_secs(1), || sup_e.tick_samples().len() >= 3).await;
    let _ = sup_e.observation().await.unwrap();
    wait_until(Duration::from_secs(1), || sup_e.tick_samples().len() >= 8).await;
    let (st_e, _) = sup_e.trigger_e_stop_checked().await.unwrap();
    assert_eq!(st_e, SafetyState::EStop);
    // Warm all 3 observation slots with `FaultInfo`, then verify repeated E-stop releases do 0 allocs/frees.
    wait_until(Duration::from_secs(1), || sup_e.tick_samples().len() >= 14).await;
    let _ = sup_e.observation().await.unwrap();
    wait_until(Duration::from_secs(1), || sup_e.tick_samples().len() >= 18).await;
    let pre_e2 = sup_e.tick_samples().len();
    let (st_e2, _) = sup_e.trigger_e_stop_checked().await.unwrap();
    assert_eq!(st_e2, SafetyState::EStop);
    wait_until(Duration::from_secs(1), || sup_e.tick_samples().len() >= pre_e2 + 3).await;
    let e_nonzero: Vec<(usize, u32)> = sup_e.tick_samples()[pre_e2..].iter().enumerate().filter(|(_, s)| s.usage.lock_alloc_calls > 0).map(|(i, s)| (i, s.usage.lock_alloc_calls)).collect();
    assert!(e_nonzero.is_empty(), "steady-state RT E-stop releases must do 0 allocs/frees: {e_nonzero:?}");
    h_e.stop_and_join();

    // (b) Refusal-only `service_wake` flushes urgent stop reply without publish and with 0 allocs/frees.
    let (mut engine, _broker, _tw, _stops) = ReleaseEngine::new(BootId::fresh(), Capacities::default());
    let (client, intake) = urgent_stop_channel(4, Arc::new(AtomicBool::new(false)));
    engine.attach_urgent_stops(intake);
    let mut core = SupervisorInner::new(ArmConfig::rebot_dm());
    core.rt_executor_owned = true;
    core.prewarm_rt_buffers();
    let mut rx = client.enqueue_async(mk(1, 0, 1)).unwrap();
    let mut report = ReleaseReport::default();
    let _ = armd::diagnostics::alloc::take();
    let published = engine.service_wake(&mut core, Instant::now(), Instant::now() + Duration::from_millis(5), 1, &mut report);
    let a_wake = armd::diagnostics::alloc::take();
    assert_eq!(a_wake.calls, 0, "refusal-only service_wake must do 0 allocs/frees: {a_wake:?}");
    assert!(!published && engine.take_urgent_replies_flushed());
    client.drain_replies();
    assert_eq!(rx.try_recv().unwrap().unwrap(), UrgentStopOutcome::LeaseLost { epoch: core.safety.epoch_id() });

    // (c) Watchdog park preserved by `authorized_stop`.
    let mut park_core = SupervisorInner::new(ArmConfig { lease_timeout_ms: 30, ..ArmConfig::rebot_dm() });
    let _g = park_core.acquire_lease("connection:park").unwrap();
    { let __s = park_core.current_admission_stamp(); park_core.check_admission(__s).and_then(|__a| park_core.enable_torque(__a)) }.unwrap();
    park_core.commanded_positions = INSPECT.to_vec();
    park_core.measured_positions = INSPECT.to_vec();
    park_core.target_positions = INSPECT.to_vec();
    std::thread::sleep(Duration::from_millis(45));
    park_core.tick(0.02);
    assert!(park_core.watchdog_park.is_some() && park_core.active_trajectory.is_some());
    assert!(!park_core.authorized_stop());
    assert!(park_core.watchdog_park.is_some() && park_core.active_trajectory.is_some());

    // (d) Panic strictly between urgent-stop apply and Step 6 publish -> 503 EXECUTOR_STOPPED.
    let sup_p = Arc::new(ArmSupervisor::new(ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() }));
    let gen_p = {
        let mut inner = sup_p.inner().write().await;
        let g = inner.acquire_lease(conn).unwrap();
        { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| inner.enable_torque(__a)) }.unwrap();
        g
    };
    let stop_watch: Arc<Mutex<Option<armd::rt_core::urgent::StopRequests>>> = Arc::new(Mutex::new(None));
    let seen_panic = Arc::new(AtomicU64::new(0));
    let (sw, sp) = (stop_watch.clone(), seen_panic.clone());
    let mut ecfg = ExecutorConfig::for_tests(Duration::from_millis(5));
    ecfg.panic_policy = PanicPolicy::StopExecutor;
    let h_p = sup_p.start_executor_with_hooks(ecfg.clone(), PersistTestHooks {
        before_publish: Some(Arc::new(move || {
            if sw.lock().unwrap().as_ref().is_some_and(|s| s.applied() > 0) && sp.fetch_add(1, Ordering::AcqRel) == 0 {
                panic!("injected panic between urgent stop apply and publish");
            }
        })),
        ..Default::default()
    }).await.unwrap();
    *stop_watch.lock().unwrap() = Some(h_p.stop_requests().clone());
    wait_until(Duration::from_secs(1), || h_p.stats().steps.load(Ordering::Relaxed) >= 2).await;
    let ep_p = sup_p.observation().await.unwrap().epoch_id;
    let err = sup_p.urgent_stop(mk(1, gen_p, ep_p)).await.unwrap_err();
    assert_eq!(seen_panic.load(Ordering::Acquire), 1);
    assert!(matches!(err, ArmError::Rejected { status: 503, ref code, .. } if code == "EXECUTOR_STOPPED"), "{err:?}");
    assert!(sup_p.e_stop_latched.load(Ordering::SeqCst));
    h_p.stop_and_join();

    // (e) Panic strictly between E-stop consume and Step 6 publish -> REST & WS MUST fail 503 EXECUTOR_STOPPED (never Ack unpublished E-stop!).
    let sup_ep = Arc::new(ArmSupervisor::new(ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() }));
    let addr_ep = serve(sup_ep.clone()).await;
    let (mut ws_ep, _) = connect_async(format!("ws://{addr_ep}/api/v1/ws")).await.unwrap();
    let seen_ep_panic = Arc::new(AtomicU64::new(0));
    let sep = seen_ep_panic.clone();
    let latched_ref = sup_ep.e_stop_latched.clone();
    let h_ep = sup_ep.start_executor_with_hooks(ecfg, PersistTestHooks {
        before_publish: Some(Arc::new(move || {
            if latched_ref.load(Ordering::Acquire) && sep.fetch_add(1, Ordering::AcqRel) == 0 {
                panic!("injected panic between E-stop consume and publish");
            }
        })),
        ..Default::default()
    }).await.unwrap();
    wait_until(Duration::from_secs(1), || h_ep.stats().steps.load(Ordering::Relaxed) >= 2).await;
    let r_ws_ep = ws_reply(&mut ws_ep, json!({"type": "estop", "request_id": 88}), &["epoch_sync", "estop_reject"]).await;
    assert_eq!(seen_ep_panic.load(Ordering::Acquire), 1);
    assert_eq!((r_ws_ep["type"].as_str(), r_ws_ep["status"].as_u64(), r_ws_ep["code"].as_str(), r_ws_ep["latched"].as_bool()),
        (Some("estop_reject"), Some(503), Some("EXECUTOR_STOPPED"), Some(true)), "{r_ws_ep}");
    assert!(r_ws_ep.get("epoch_id").is_none());
    let rest_ep = reqwest::Client::new().post(format!("http://{addr_ep}/api/v1/safety/e_stop")).send().await.unwrap();
    assert_eq!(rest_ep.status(), reqwest::StatusCode::SERVICE_UNAVAILABLE);
    assert!(sup_ep.e_stop_latched.load(Ordering::SeqCst));
    h_ep.stop_and_join();
}

struct WriteHookGuard(String);
impl Drop for WriteHookGuard {
    fn drop(&mut self) {
        armd::api::ws::set_test_ws_write_paused(&self.0, false);
        armd::api::ws::set_test_ws_write_timeout(&self.0, None);
    }
}

struct SlowDetector {
    entered: Arc<AtomicBool>,
    active_calls: Arc<AtomicUsize>,
    released: Arc<AtomicBool>,
    release: Arc<tokio::sync::Notify>,
}

impl SlowDetector {
    fn new() -> Arc<Self> {
        Arc::new(Self {
            entered: Arc::new(AtomicBool::new(false)),
            active_calls: Arc::new(AtomicUsize::new(0)),
            released: Arc::new(AtomicBool::new(false)),
            release: Arc::new(tokio::sync::Notify::new()),
        })
    }

    fn release_now(&self) {
        self.released.store(true, Ordering::Release);
        self.release.notify_waiters();
    }
}

impl armd::grasp::DetectionSource for SlowDetector {
    fn inference_enabled(&self) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<bool, String>> + Send + '_>> {
        Box::pin(async { Ok(false) })
    }
    fn set_inference(&self, _enabled: bool) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<(), String>> + Send + '_>> {
        Box::pin(async { Ok(()) })
    }
    fn next_frame(&self, _after: Option<u64>, _wait_ms: u64) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<Option<armd::grasp::DetectionFrame>, String>> + Send + '_>> {
        Box::pin(async { Ok(None) })
    }
    fn mode_info(&self) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<armd::grasp::ModeInfo, String>> + Send + '_>> {
        Box::pin(async move {
            struct ActiveGuard(Arc<AtomicUsize>);
            impl Drop for ActiveGuard {
                fn drop(&mut self) { self.0.fetch_sub(1, Ordering::SeqCst); }
            }
            self.active_calls.fetch_add(1, Ordering::SeqCst);
            let _guard = ActiveGuard(self.active_calls.clone());
            self.entered.store(true, Ordering::SeqCst);
            while !self.released.load(Ordering::Acquire) {
                let n = self.release.notified();
                if self.released.load(Ordering::Acquire) { break; }
                n.await;
            }
            Err("simulated slow detector released".into())
        })
    }
}

async fn serve_with_slow_detector(sup: Arc<ArmSupervisor>, det: Arc<SlowDetector>) -> SocketAddr {
    let deps = armd::grasp::GraspDeps { source: det, setup_path: None };
    let upstream = armd::api::proxy::DetectorUpstream::parse("http://127.0.0.1:9").unwrap();
    let router = armd::api::create_router_with(sup, upstream, deps, OperatorAuth::disabled());
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });
    addr
}

fn config_with_slow_task() -> ArmConfig {
    let mut cfg = ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() };
    cfg.grasp.as_mut().expect("rebot_dm has grasp setup").tasks.insert(
        "slow".into(),
        armd::grasp::GraspTask {
            overrides: Default::default(),
            requires: armd::grasp::TaskRequires { model_kind: None, depth: Some(armd::grasp::DepthReq::Any) },
            models: None,
            description: None,
        },
    );
    cfg
}

fn slow_dry_run_body() -> serde_json::Value {
    json!({"dry_run": true, "task": "slow", "label": "cup", "box_norm": [0.45, 0.45, 0.55, 0.55], "frame_id": 1})
}

// 6. Blocked writer (stalled socket) + full ordinary queue: E-stop still latches within one release,
//    Stop still reaches the owner (executor & legacy), and bounded per-write timeout closes stalled sockets.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn stalled_writer_and_full_ordinary_queue_still_latches_estop_and_reaches_owner_for_stop() {
    // (a) Executor mode: writer stalled, ordinary outbound queue filled to capacity (64/64).
    let sup = Arc::new(ArmSupervisor::new(ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() }));
    let addr = serve(sup.clone()).await;
    let (mut ws, gen) = controller(addr).await;
    let conn_exec = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
    let _guard_exec = WriteHookGuard(conn_exec.clone());
    let ep0 = sup.get_observation().await.epoch_id;
    assert_eq!(rpc(&mut ws, gen, ep0, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
    let ep1 = sup.get_observation().await.epoch_id;
    assert_eq!(rpc(&mut ws, gen, ep1, "/api/v1/motion/jog", json!({"target_positions": [0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "duration_secs": 1.0})).await["type"], "command_ack");

    let delay = Arc::new(Mutex::new(None));
    let handle = sup.start_executor_with_hooks(
        ExecutorConfig::for_tests(Duration::from_millis(5)),
        PersistTestHooks { release_delay_once: Some(delay.clone()), ..Default::default() },
    ).await.unwrap();
    wait_until(Duration::from_secs(1), || handle.stats().steps.load(Ordering::Relaxed) >= 2).await;
    let ep_live = sup.observation().await.unwrap().epoch_id;

    // Stall the socket writer inside `write_ws_frame`.
    armd::api::ws::set_test_ws_write_paused(&conn_exec, true);
    send(&mut ws, json!({"type": "command", "request_id": 100, "generation": gen, "epoch_id": ep_live, "path": "/api/v1/motion/jog", "body": {}})).await;
    wait_until(Duration::from_secs(1), || armd::api::ws::test_ws_writer_blocked_count(&conn_exec) >= 1).await;

    // Fill the ordinary outbound queue to exact capacity (64 control replies) while writer stays blocked.
    for k in 0..64u64 {
        send(&mut ws, json!({"type": "command", "request_id": 101 + k, "generation": gen, "epoch_id": ep_live, "path": "/api/v1/motion/jog", "body": {}})).await;
    }
    tokio::time::sleep(Duration::from_millis(20)).await;
    assert!(armd::api::ws::test_ws_writer_blocked_count(&conn_exec) >= 1, "writer must still be stalled");

    // Pause one release, send Stop on the SAME stalled socket -> applies on THAT exact release.
    *delay.lock().unwrap() = Some(Duration::from_millis(60));
    wait_until(Duration::from_secs(1), || delay.lock().unwrap().is_none()).await;
    let s_stop = handle.stats().steps.load(Ordering::Relaxed);
    send(&mut ws, json!({"type": "command", "request_id": 200, "generation": gen, "epoch_id": ep_live, "path": "/api/v1/motion/stop", "body": {}})).await;
    wait_until(Duration::from_secs(1), || handle.stop_requests().applied() >= 1).await;
    assert!(handle.stats().steps.load(Ordering::Relaxed) <= s_stop + 1, "Stop did not apply within one release");

    // Pause one release, send E-stop on the SAME stalled socket -> latches immediately AND applies on THAT exact release.
    *delay.lock().unwrap() = Some(Duration::from_millis(60));
    wait_until(Duration::from_secs(1), || delay.lock().unwrap().is_none()).await;
    let s_estop = handle.stats().steps.load(Ordering::Relaxed);
    let t0 = Instant::now();
    send(&mut ws, json!({"type": "estop"})).await;
    wait_until(Duration::from_millis(45), || sup.e_stop_latched.load(Ordering::SeqCst)).await;
    assert!(t0.elapsed() < Duration::from_millis(45), "E-stop did not latch immediately from reader");
    wait_until(Duration::from_secs(1), || {
        sup.hub().latest().is_some_and(|s| s.obs.safety_state == SafetyState::EStop && s.obs.epoch_id > ep_live)
    }).await;
    assert!(handle.stats().steps.load(Ordering::Relaxed) <= s_estop + 1, "E-stop did not apply within one release");

    // Unpause writer -> urgent E-stop `epoch_sync` and urgent Stop `command_ack` are delivered!
    armd::api::ws::set_test_ws_write_paused(&conn_exec, false);
    let mut got_sync = false;
    let mut got_stop_ack = false;
    for _ in 0..100 {
        let v = next_of(&mut ws, &["epoch_sync", "command_ack", "command_reject"]).await;
        if v["type"] == "epoch_sync" && v["safety_state"] == "EStop" { got_sync = true; }
        if v["type"] == "command_ack" && v["request_id"] == 200 { got_stop_ack = true; }
        if got_sync && got_stop_ack { break; }
    }
    assert!(got_sync && got_stop_ack, "expected both urgent replies: sync={got_sync}, stop_ack={got_stop_ack}");
    handle.stop_and_join();

    // (b) Legacy mode: writer stalled + worker blocked in slow detector + inbound ordinary queue 64/64 full (+ BACKLOG_FULL).
    let det = SlowDetector::new();
    let sup_l = Arc::new(ArmSupervisor::new(config_with_slow_task()));
    let addr_l = serve_with_slow_detector(sup_l.clone(), det.clone()).await;
    let (mut ws_l, gen_l) = controller(addr_l).await;
    let conn_l = sup_l.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
    let _guard_l = WriteHookGuard(conn_l.clone());
    let ep_l = sup_l.get_observation().await.epoch_id;
    assert_eq!(rpc(&mut ws_l, gen_l, ep_l, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
    let ep_l = sup_l.get_observation().await.epoch_id;

    // Block worker inside `dry_run_task` (`mode_info().await`), start active motion, stall writer, and fill inbound ordinary queue (64/64) + 1 BACKLOG_FULL.
    send(&mut ws_l, json!({"type": "command", "request_id": 300, "generation": gen_l, "epoch_id": ep_l, "path": "/api/v1/action/grasp", "body": slow_dry_run_body()})).await;
    wait_until(Duration::from_secs(1), || det.entered.load(Ordering::SeqCst)).await;
    { let mut i = sup_l.inner().write().await; let a = i.check_admission(sup_l.capture_admission_stamp()).unwrap(); i.jog_action(a, vec![0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], Some(2.0), Some(ep_l), "legacy_jog") }.unwrap();
    assert!(sup_l.inner().read().await.active_trajectory.is_some());
    armd::api::ws::set_test_ws_write_paused(&conn_l, true);
    for k in 0..65u64 {
        send(&mut ws_l, json!({"type": "command", "request_id": 301 + k, "generation": gen_l, "epoch_id": ep_l, "path": "/api/v1/motion/home", "body": {}})).await;
    }
    wait_until(Duration::from_secs(1), || armd::api::ws::test_ws_writer_blocked_count(&conn_l) >= 1).await;

    // Stop on the same socket reaches legacy owner and stops active motion while worker & writer are BOTH blocked.
    send(&mut ws_l, json!({"type": "command", "request_id": 400, "generation": gen_l, "epoch_id": ep_l, "path": "/api/v1/motion/stop", "body": {}})).await;
    wait_until(Duration::from_millis(200), || sup_l.inner().try_read().is_ok_and(|g| g.active_trajectory.is_none())).await;

    // E-stop on the same socket latches & applies while worker & writer are BOTH blocked.
    send(&mut ws_l, json!({"type": "estop"})).await;
    wait_until(Duration::from_millis(100), || sup_l.e_stop_latched.load(Ordering::SeqCst)).await;
    wait_until(Duration::from_millis(200), || sup_l.inner().try_read().is_ok_and(|g| *g.safety.state() == SafetyState::EStop)).await;
    det.release_now();
    armd::api::ws::set_test_ws_write_paused(&conn_l, false);

    // (c) Bounded per-write timeout closes a permanently stalled socket and releases its lease.
    let (addr_t, sup_t) = spawn_server(ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() }).await;
    let (mut ws_t, gen_t) = controller(addr_t).await;
    let ep_t = sup_t.get_observation().await.epoch_id;
    let conn_id = sup_t.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
    let _guard_t = WriteHookGuard(conn_id.clone());
    assert!(armd::api::ws::connection_registry().lock().unwrap().contains_key(&conn_id));
    armd::api::ws::set_test_ws_write_timeout(&conn_id, Some(Duration::from_millis(80)));
    armd::api::ws::set_test_ws_write_paused(&conn_id, true);
    send(&mut ws_t, json!({"type": "command", "request_id": 10, "generation": gen_t, "epoch_id": ep_t, "path": "/api/v1/torque", "body": {"enable": true}})).await;
    wait_until(Duration::from_secs(1), || !armd::api::ws::connection_registry().lock().unwrap().contains_key(&conn_id)).await;
    assert!(sup_t.inner().read().await.active_lease.is_none(), "timed-out connection must release its lease");
}

// 7. Ordinary command awaiting a slow dry-run / detector: E-stop and Stop on the same socket still apply immediately.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn slow_dry_run_detector_does_not_block_estop_or_stop_on_same_socket() {
    let det = SlowDetector::new();
    let sup = Arc::new(ArmSupervisor::new(config_with_slow_task()));
    let addr = serve_with_slow_detector(sup.clone(), det.clone()).await;
    let (mut ws, gen) = controller(addr).await;
    let ep0 = sup.get_observation().await.epoch_id;
    assert_eq!(rpc(&mut ws, gen, ep0, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
    let ep1 = sup.get_observation().await.epoch_id;

    // Start slow dry-run grasp awaiting detector `mode_info().await`.
    send(&mut ws, json!({"type": "command", "request_id": 500, "generation": gen, "epoch_id": ep1, "path": "/api/v1/action/grasp", "body": slow_dry_run_body()})).await;
    wait_until(Duration::from_secs(1), || det.entered.load(Ordering::SeqCst)).await;
    // Start active motion while worker is blocked on the detector so Stop has live motion to halt.
    { let mut i = sup.inner().write().await; let a = i.check_admission(sup.capture_admission_stamp()).unwrap(); i.jog_action(a, vec![0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], Some(2.0), Some(ep1), "slow_dry_run_jog") }.unwrap();
    assert!(sup.inner().read().await.active_trajectory.is_some());

    // While worker is blocked on the detector, Stop on the SAME socket applies immediately and stops motion.
    let t_stop = Instant::now();
    send(&mut ws, json!({"type": "command", "request_id": 501, "generation": gen, "epoch_id": ep1, "path": "/api/v1/motion/stop", "body": {}})).await;
    let stop_ack = next_of(&mut ws, &["command_ack", "command_reject"]).await;
    assert_eq!((stop_ack["type"].as_str(), stop_ack["request_id"].as_u64()), (Some("command_ack"), Some(501)), "{stop_ack}");
    assert!(t_stop.elapsed() < Duration::from_millis(200), "Stop blocked behind dry-run detector: {:?}", t_stop.elapsed());
    assert!(sup.inner().read().await.active_trajectory.is_none(), "Stop did not stop active motion");

    // Still while worker is blocked on the detector, E-stop on the SAME socket latches & replies immediately.
    let t_estop = Instant::now();
    send(&mut ws, json!({"type": "estop"})).await;
    let es = next_of(&mut ws, &["epoch_sync", "estop_reject"]).await;
    assert_eq!((es["type"].as_str(), es["safety_state"].as_str()), (Some("epoch_sync"), Some("EStop")), "{es}");
    let estop_epoch = es["epoch_id"].as_u64().unwrap();
    assert!(estop_epoch > ep1);
    assert!(t_estop.elapsed() < Duration::from_millis(200), "E-stop blocked behind dry-run detector: {:?}", t_estop.elapsed());

    // Release detector -> dry-run reply arrives with monotonic epoch >= estop_epoch.
    det.release_now();
    let dry_ack = next_of(&mut ws, &["command_ack", "command_reject"]).await;
    assert_eq!((dry_ack["type"].as_str(), dry_ack["request_id"].as_u64()), (Some("command_ack"), Some(500)), "{dry_ack}");
    assert!(dry_ack["epoch_id"].as_u64().unwrap() >= estop_epoch, "dry-run reply regressed epoch: {dry_ack}");
}

// 8. Two Stops in a row with the first pending: coalesced into one applied generation, never enter ordinary backlog.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn two_stops_coalesce_with_same_applied_generation_and_never_enter_backlog() {
    // (a) Executor mode: two valid Stops + one duplicate-request_id Stop while release is paused.
    let sup = Arc::new(ArmSupervisor::new(ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() }));
    let addr = serve(sup.clone()).await;
    let (mut ws, gen) = controller(addr).await;
    let ep0 = sup.get_observation().await.epoch_id;
    assert_eq!(rpc(&mut ws, gen, ep0, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");

    let delay = Arc::new(Mutex::new(None));
    let handle = sup.start_executor_with_hooks(
        ExecutorConfig::for_tests(Duration::from_millis(5)),
        PersistTestHooks { release_delay_once: Some(delay.clone()), ..Default::default() },
    ).await.unwrap();
    wait_until(Duration::from_secs(1), || handle.stats().steps.load(Ordering::Relaxed) >= 2).await;
    let ep = sup.observation().await.unwrap().epoch_id;

    *delay.lock().unwrap() = Some(Duration::from_millis(80));
    wait_until(Duration::from_secs(1), || delay.lock().unwrap().is_none()).await;

    send(&mut ws, json!({"type": "command", "request_id": 601, "generation": gen, "epoch_id": ep, "path": "/api/v1/motion/stop", "body": {}})).await;
    send(&mut ws, json!({"type": "command", "request_id": 602, "generation": gen, "epoch_id": ep, "path": "/api/v1/motion/stop", "body": {}})).await;
    send(&mut ws, json!({"type": "command", "request_id": 602, "generation": gen, "epoch_id": ep, "path": "/api/v1/motion/stop", "body": {}})).await;

    let mut replies = Vec::new();
    while replies.len() < 3 {
        replies.push(next_of(&mut ws, &["command_ack", "command_reject"]).await);
    }
    let r601 = replies.iter().find(|r| r["request_id"] == 601).expect("reply 601");
    let r602_ack = replies.iter().find(|r| r["request_id"] == 602 && r["type"] == "command_ack").expect("coalesced ack 602");
    let r602_dup = replies.iter().find(|r| r["request_id"] == 602 && r["type"] == "command_reject").expect("duplicate reject 602");
    let sg1 = r601["stop_generation"].as_u64().unwrap();
    let sg2 = r602_ack["stop_generation"].as_u64().unwrap();
    assert_eq!(r601["type"], "command_ack");
    assert_eq!(sg1, 1, "coalesced Stops must advance StopRequests once, got {sg1}");
    assert_eq!(sg1, sg2, "both coalesced Stops must receive the same applied stop_generation");
    assert_eq!(handle.stop_requests().applied(), 1, "owner must apply exactly one stop generation");
    assert_eq!((r602_dup["status"].as_u64(), r602_dup["code"].as_str()), (Some(400), Some("INVALID_COMMAND")), "{r602_dup}");
    handle.stop_and_join();

    // (b) Legacy mode: two Stops while `inner.write()` is briefly held coalesce and both ACK.
    let (addr_l, sup_l) = spawn_server(ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() }).await;
    let (mut ws_l, gen_l) = controller(addr_l).await;
    let ep_l = sup_l.get_observation().await.epoch_id;
    let guard = sup_l.inner().write().await;
    send(&mut ws_l, json!({"type": "command", "request_id": 701, "generation": gen_l, "epoch_id": ep_l, "path": "/api/v1/motion/stop", "body": {}})).await;
    send(&mut ws_l, json!({"type": "command", "request_id": 702, "generation": gen_l, "epoch_id": ep_l, "path": "/api/v1/motion/stop", "body": {}})).await;
    tokio::time::sleep(Duration::from_millis(25)).await;
    drop(guard);
    let a1 = next_of(&mut ws_l, &["command_ack", "command_reject"]).await;
    let a2 = next_of(&mut ws_l, &["command_ack", "command_reject"]).await;
    assert_eq!((a1["type"].as_str(), a2["type"].as_str()), (Some("command_ack"), Some("command_ack")), "a1={a1}, a2={a2}");
    let mut ids = [a1["request_id"].as_u64().unwrap(), a2["request_id"].as_u64().unwrap()];
    ids.sort();
    assert_eq!(ids, [701, 702]);
}

// 9. Owner preempted before its queued Stop applies: Stop is rejected per authorization rules
//    (LEASE_LOST, StopRequests & active motion untouched) in both legacy and executor paths.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn preempted_owner_queued_stop_is_rejected_by_authorization_rules() {
    // (a) Legacy path: Client 1 queues Stop while write lock is held by preemption to Client 2.
    let (addr, sup) = spawn_server(ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() }).await;
    let (mut ws1, gen1) = controller(addr).await;
    let ep = sup.get_observation().await.epoch_id;
    assert_eq!(rpc(&mut ws1, gen1, ep, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
    let ep = sup.get_observation().await.epoch_id;

    {
        let mut inner = sup.inner().write().await;
        // Client 1 sends Stop while `inner.write()` is held (reader hands it off immediately to urgent Stop task).
        send(&mut ws1, json!({"type": "command", "request_id": 801, "generation": gen1, "epoch_id": ep, "path": "/api/v1/motion/stop", "body": {}})).await;
        send(&mut ws1, json!({"type": "command", "request_id": 802, "generation": gen1, "epoch_id": ep, "path": "/api/v1/motion/stop", "body": {}})).await;
        tokio::time::sleep(Duration::from_millis(25)).await;
        // Preempt Client 1 BEFORE its queued Stop acquires `inner.write()`, and start new motion for Client 2.
        let (gen2, evicted) = inner.preempt_lease("connection:takeover", "client-2").unwrap();
        assert!(gen2 > gen1 && evicted.is_some());
        { let a = inner.check_admission(inner.current_admission_stamp()).unwrap(); inner.jog_action(a, vec![0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], Some(1.0), Some(ep), "client2_jog") }.unwrap();
    }
    let r1 = next_of(&mut ws1, &["command_ack", "command_reject"]).await;
    let r2 = next_of(&mut ws1, &["command_ack", "command_reject"]).await;
    assert_eq!((r1["type"].as_str(), r1["code"].as_str()), (Some("command_reject"), Some("LEASE_LOST")), "{r1}");
    assert_eq!((r2["type"].as_str(), r2["code"].as_str()), (Some("command_reject"), Some("LEASE_LOST")), "{r2}");
    assert!(sup.inner().read().await.active_trajectory.is_some(), "preempted owner's Stop must not cancel new owner's motion");

    // (b) Executor path (`ReleaseEngine` + `UrgentStopIntake`): queued Stop from Owner 1 rejected after preemption.
    let mut core = SupervisorInner::new(ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() });
    let g1 = core.acquire_lease("connection:owner1").unwrap();
    { let __s = core.current_admission_stamp(); core.check_admission(__s).and_then(|__a| core.enable_torque(__a)) }.unwrap();
    let ep_e = core.safety.epoch_id();
    { let a = core.check_admission(core.current_admission_stamp()).unwrap(); core.jog_action(a, vec![0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], Some(1.0), Some(ep_e), "owner1_jog") }.unwrap();
    let (mut engine, _broker, _tw, stops) = ReleaseEngine::new(BootId::fresh(), Capacities::default());
    let (client, intake) = urgent_stop_channel(4, Arc::new(AtomicBool::new(false)));
    engine.attach_urgent_stops(intake);
    let mut rx = client.enqueue_async(UrgentStop {
        connection: armd::model::HolderLabel::new("connection:owner1"),
        lease_generation: g1,
        epoch: Some(ep_e),
        request_id: 1,
        request_id_valid: true,
        body_error: None,
    }).unwrap();
    // Preempt Owner 1 before the release drains urgent stops, clear handover settle, and start Owner 2's motion.
    let (g2, _) = core.preempt_lease("connection:owner2", "owner2").unwrap();
    assert!(g2 > g1);
    core.handover_settling_since = None;
    { let a = core.check_admission(core.current_admission_stamp()).unwrap(); core.jog_action(a, vec![0.35, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], Some(1.0), Some(ep_e), "owner2_jog") }.unwrap();
    let _rep = engine.step(&mut core, Instant::now, 0.005, Instant::now() + Duration::from_millis(5), 1);
    client.drain_replies();
    assert_eq!(rx.try_recv().unwrap().unwrap(), UrgentStopOutcome::LeaseLost { epoch: ep_e });
    assert_eq!(stops.applied(), 0, "preempted Stop must not advance StopRequests");
    assert!(core.active_trajectory.is_some(), "preempted Stop must not stop new owner's motion");
}

// 10. Ordinary outbound queue saturated with control replies: connection closes with an explicit error
//     frame, releases its lease, and never silently drops a control reply while staying open.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn ordinary_outbound_queue_saturation_closes_connection_with_error_and_drops_nothing_silently() {
    use futures_util::StreamExt;

    let (addr, sup) = spawn_server(ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() }).await;
    let (mut ws, gen) = controller(addr).await;
    let ep = sup.get_observation().await.epoch_id;
    let conn_id = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
    let _hook_guard = WriteHookGuard(conn_id.clone());

    // Pause writer so ordinary control replies accumulate in `ordinary_out_tx` (cap 64).
    armd::api::ws::set_test_ws_write_paused(&conn_id, true);
    send(&mut ws, json!({"type": "command", "request_id": 1, "generation": gen, "epoch_id": ep, "path": "/api/v1/torque", "body": {"enable": true}})).await;
    wait_until(Duration::from_secs(1), || armd::api::ws::test_ws_writer_blocked_count(&conn_id) >= 1).await;

    // Send 70 ordinary commands (> 64 outbound capacity) so reply #66 overflows `ordinary_out_tx`.
    for id in 2..=71u64 {
        send(&mut ws, json!({"type": "command", "request_id": id, "generation": gen, "epoch_id": ep, "path": "/api/v1/torque", "body": {"enable": true}})).await;
    }
    tokio::time::sleep(Duration::from_millis(80)).await;
    armd::api::ws::set_test_ws_write_paused(&conn_id, false);

    // Read until connection closes: must observe explicit OUTBOUND_QUEUE_FULL error frame + Close,
    // and never receive replies after a silently dropped gap.
    let mut saw_error_frame = false;
    let mut saw_close = false;
    let mut seen_ack_ids = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < deadline {
        match tokio::time::timeout(Duration::from_millis(500), ws.next()).await {
            Ok(Some(Ok(tokio_tungstenite::tungstenite::Message::Text(txt)))) => {
                if let Ok(v) = serde_json::from_str::<serde_json::Value>(&txt) {
                    if v["type"] == "error" && v["code"] == "OUTBOUND_QUEUE_FULL" {
                        saw_error_frame = true;
                        assert_eq!(v["status"], 503);
                    } else if v["type"] == "command_ack" {
                        if let Some(id) = v["request_id"].as_u64() { seen_ack_ids.push(id); }
                    }
                }
            }
            Ok(Some(Ok(tokio_tungstenite::tungstenite::Message::Close(frame)))) => {
                saw_close = true;
                if let Some(f) = frame {
                    assert_eq!(u16::from(f.code), 1013);
                    assert_eq!(f.reason.as_str(), "OUTBOUND_QUEUE_FULL");
                }
                break;
            }
            Ok(None) | Ok(Some(Err(_))) => { saw_close = true; break; }
            _ => {}
        }
    }
    assert!(saw_error_frame, "connection must send explicit OUTBOUND_QUEUE_FULL error frame before closing");
    assert!(saw_close, "connection must close on ordinary control reply queue saturation");
    // No silent-drop gap: received ACKs must be contiguous from 1 (never skipping over dropped replies).
    for (i, id) in seen_ack_ids.iter().enumerate() {
        assert_eq!(*id, (i as u64) + 1, "control reply silently dropped before id={id}: {seen_ack_ids:?}");
    }
    assert!(seen_ack_ids.len() < 71, "connection must close before completing flooded replies");
    // Server must release the closed connection's lease so owner state never diverges from client view.
    wait_until(Duration::from_secs(1), || !armd::api::ws::connection_registry().lock().unwrap().contains_key(&conn_id)).await;
    assert!(sup.inner().read().await.active_lease.is_none(), "closed connection's lease must be released");
}

struct WorkerLockHookGuard(String);
impl Drop for WorkerLockHookGuard {
    fn drop(&mut self) {
        armd::api::ws::set_test_ws_worker_before_lock_paused(&self.0, false);
    }
}

// 11. Round 3 Item 1 (Blocker): popped ordinary motion command held right before owner lock is
//     refused as STOPPED after a Stop applies under the lock, and zero motion starts.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn popped_motion_command_held_at_lock_is_refused_after_legacy_stop() {
    let (addr, sup) = spawn_server(ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() }).await;
    let (mut ws, gen) = controller(addr).await;
    let ep = sup.get_observation().await.epoch_id;
    let conn_id = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
    let _worker_guard = WorkerLockHookGuard(conn_id.clone());
    assert_eq!(rpc(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
    let ep = sup.get_observation().await.epoch_id;

    // Hold the ordinary worker right after it pops the jog frame and before `inner.write()`.
    armd::api::ws::set_test_ws_worker_before_lock_paused(&conn_id, true);
    send(&mut ws, json!({"type": "command", "request_id": 10, "generation": gen, "epoch_id": ep, "path": "/api/v1/motion/jog", "body": {"target_positions": [0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "duration_secs": 1.0}})).await;
    wait_until(Duration::from_secs(1), || armd::api::ws::test_ws_worker_before_lock_blocked_count(&conn_id) >= 1).await;

    // Apply a Stop while the popped jog is held at the lock boundary.
    send(&mut ws, json!({"type": "command", "request_id": 11, "generation": gen, "epoch_id": ep, "path": "/api/v1/motion/stop", "body": {}})).await;
    let stop_ack = next_of(&mut ws, &["command_ack", "command_reject"]).await;
    assert_eq!((stop_ack["type"].as_str(), stop_ack["request_id"].as_u64()), (Some("command_ack"), Some(11)), "{stop_ack}");

    // Release the held jog -> acquires `inner.write()`, sees stale admission stop generation, refuses STOPPED.
    armd::api::ws::set_test_ws_worker_before_lock_paused(&conn_id, false);
    let jog_rej = next_of(&mut ws, &["command_ack", "command_reject"]).await;
    assert_eq!((jog_rej["type"].as_str(), jog_rej["request_id"].as_u64(), jog_rej["status"].as_u64(), jog_rej["code"].as_str()),
        (Some("command_reject"), Some(10), Some(409), Some("STOPPED")), "{jog_rej}");
    assert!(sup.inner().read().await.active_trajectory.is_none(), "popped jog must NOT start motion after Stop applied");

    // Subsequent motion command admitted AFTER Stop succeeds cleanly.
    send(&mut ws, json!({"type": "command", "request_id": 12, "generation": gen, "epoch_id": ep, "path": "/api/v1/motion/jog", "body": {"target_positions": [0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "duration_secs": 0.5}})).await;
    let ok_jog = next_of(&mut ws, &["command_ack", "command_reject"]).await;
    assert_eq!((ok_jog["type"].as_str(), ok_jog["request_id"].as_u64()), (Some("command_ack"), Some(12)), "{ok_jog}");
    assert!(sup.inner().read().await.active_trajectory.is_some());
}

// 12. Round 3 Item 2 (Major): urgent Stop request_id does not advance ordinary `last_request`, so
//     send-order allocation (ordinary 5, Stop 6, ordinary 7) never fails INVALID_COMMAND.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn urgent_stop_does_not_reject_earlier_or_later_ordinary_request_ids() {
    let det = SlowDetector::new();
    let sup = Arc::new(ArmSupervisor::new(config_with_slow_task()));
    let addr = serve_with_slow_detector(sup.clone(), det.clone()).await;
    let (mut ws, gen) = controller(addr).await;
    let ep = sup.get_observation().await.epoch_id;
    let conn_id = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
    let _worker_guard = WorkerLockHookGuard(conn_id.clone());

    // (a) Ordinary 5 (read-only dry-run) held before lock while Stop 6 ACKs; releasing 5 + sending 7 both ACK.
    det.release_now();
    armd::api::ws::set_test_ws_worker_before_lock_paused(&conn_id, true);
    send(&mut ws, json!({"type": "command", "request_id": 5, "generation": gen, "epoch_id": ep, "path": "/api/v1/action/grasp", "body": slow_dry_run_body()})).await;
    wait_until(Duration::from_secs(1), || armd::api::ws::test_ws_worker_before_lock_blocked_count(&conn_id) >= 1).await;

    send(&mut ws, json!({"type": "command", "request_id": 6, "generation": gen, "epoch_id": ep, "path": "/api/v1/motion/stop", "body": {}})).await;
    let stop6 = next_of(&mut ws, &["command_ack", "command_reject"]).await;
    assert_eq!((stop6["type"].as_str(), stop6["request_id"].as_u64()), (Some("command_ack"), Some(6)), "{stop6}");

    armd::api::ws::set_test_ws_worker_before_lock_paused(&conn_id, false);
    send(&mut ws, json!({"type": "command", "request_id": 7, "generation": gen, "epoch_id": ep, "path": "/api/v1/torque", "body": {"enable": true}})).await;
    let r5 = next_of(&mut ws, &["command_ack", "command_reject"]).await;
    let r7 = next_of(&mut ws, &["command_ack", "command_reject"]).await;
    assert_eq!((r5["type"].as_str(), r5["request_id"].as_u64()), (Some("command_ack"), Some(5)), "{r5}");
    assert_eq!((r7["type"].as_str(), r7["request_id"].as_u64()), (Some("command_ack"), Some(7)), "{r7}");

    // (b) Ordinary 5 queued behind slow dry-run + Stop 6 -> 5 is cancelled STOPPED (not INVALID_COMMAND), ordinary 7 ACKs.
    let det2 = SlowDetector::new();
    let sup2 = Arc::new(ArmSupervisor::new(config_with_slow_task()));
    let addr2 = serve_with_slow_detector(sup2.clone(), det2.clone()).await;
    let (mut ws2, gen2) = controller(addr2).await;
    let ep2 = sup2.get_observation().await.epoch_id;
    assert_eq!(rpc(&mut ws2, gen2, ep2, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
    let ep2 = sup2.get_observation().await.epoch_id;
    send(&mut ws2, json!({"type": "command", "request_id": 4, "generation": gen2, "epoch_id": ep2, "path": "/api/v1/action/grasp", "body": slow_dry_run_body()})).await;
    wait_until(Duration::from_secs(1), || det2.entered.load(Ordering::SeqCst)).await;
    send(&mut ws2, json!({"type": "command", "request_id": 5, "generation": gen2, "epoch_id": ep2, "path": "/api/v1/torque", "body": {"enable": true}})).await;
    send(&mut ws2, json!({"type": "command", "request_id": 6, "generation": gen2, "epoch_id": ep2, "path": "/api/v1/motion/stop", "body": {}})).await;
    let mut got_5 = None;
    let mut got_6 = None;
    while got_5.is_none() || got_6.is_none() {
        let m = next_of(&mut ws2, &["command_ack", "command_reject"]).await;
        match m["request_id"].as_u64() {
            Some(5) => got_5 = Some(m),
            Some(6) => got_6 = Some(m),
            _ => {}
        }
    }
    let m5 = got_5.unwrap();
    let m6 = got_6.unwrap();
    assert_eq!(m6["type"], "command_ack", "{m6}");
    assert_eq!((m5["type"].as_str(), m5["code"].as_str()), (Some("command_reject"), Some("STOPPED")), "{m5}");
    det2.release_now();
    let m4 = next_of(&mut ws2, &["command_ack", "command_reject"]).await;
    assert_eq!((m4["type"].as_str(), m4["request_id"].as_u64()), (Some("command_ack"), Some(4)), "{m4}");
    send(&mut ws2, json!({"type": "command", "request_id": 7, "generation": gen2, "epoch_id": ep2, "path": "/api/v1/torque", "body": {"enable": true}})).await;
    let m7 = next_of(&mut ws2, &["command_ack", "command_reject"]).await;
    assert_eq!((m7["type"].as_str(), m7["request_id"].as_u64()), (Some("command_ack"), Some(7)), "{m7}");
}

// 13. Round 3 Item 3 (Major): E-stop burst exceeding urgent reply capacity (64) still latches
//     immediately and delivers every single E-stop reply once writer slots free (zero dropped).
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn estop_burst_beyond_urgent_capacity_still_latches_and_delivers_all_replies() {
    let (addr, sup) = spawn_server(ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() }).await;
    let (mut ws, gen) = controller(addr).await;
    let ep0 = sup.get_observation().await.epoch_id;
    let conn_id = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
    let _hook_guard = WriteHookGuard(conn_id.clone());

    // Pause writer so urgent E-stop reply slots (cap 64) saturate.
    armd::api::ws::set_test_ws_write_paused(&conn_id, true);
    send(&mut ws, json!({"type": "command", "request_id": 1, "generation": gen, "epoch_id": ep0, "path": "/api/v1/torque", "body": {"enable": true}})).await;
    wait_until(Duration::from_secs(1), || armd::api::ws::test_ws_writer_blocked_count(&conn_id) >= 1).await;

    // Send 68 E-stops (> 64 urgent capacity) while writer stays blocked: latch must succeed immediately.
    const N: usize = 68;
    for i in 0..N {
        let kind = if i % 2 == 0 { "estop" } else { "e_stop" };
        send(&mut ws, json!({"type": kind})).await;
    }
    wait_until(Duration::from_millis(100), || sup.e_stop_latched.load(Ordering::SeqCst)).await;
    wait_until(Duration::from_secs(1), || sup.requested_estop_count.load(Ordering::SeqCst) >= N as u64).await;

    // Unpause writer: ALL 68 E-stop replies must arrive (zero dropped, no spurious fatal close).
    armd::api::ws::set_test_ws_write_paused(&conn_id, false);
    let mut syncs = 0usize;
    let mut last_ep = ep0;
    while syncs < N {
        let msg = next_of(&mut ws, &["epoch_sync", "estop_reject", "error"]).await;
        assert_eq!((msg["type"].as_str(), msg["safety_state"].as_str()), (Some("epoch_sync"), Some("EStop")), "unexpected reply at {syncs}: {msg}");
        let ep = msg["epoch_id"].as_u64().unwrap();
        assert!(ep >= last_ep, "epoch regressed: {ep} < {last_ep}");
        last_ep = ep;
        syncs += 1;
    }
    assert_eq!(syncs, N);
    // Connection remains alive after burst drains.
    let extra = ws_reply(&mut ws, json!({"type": "estop"}), &["epoch_sync", "estop_reject"]).await;
    assert_eq!((extra["type"].as_str(), extra["safety_state"].as_str()), (Some("epoch_sync"), Some("EStop")), "{extra}");
}

// 14. Round 3 Item 4 (Major): writer delivers lease_granted strictly before a preemption
//     lease_lost of the same generation, and never skips action terminals under latest-wins telemetry.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn writer_delivers_lease_grant_before_preemption_loss_and_never_skips_action_terminals() {
    // (a) Grant-before-loss causal order: pause Client 1's writer, grant gen 2 on ordinary queue,
    //     preempt gen 2 from Client 2 onto urgent revocation queue -> Client 1 receives grant THEN loss!
    let (addr, sup) = spawn_server(ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() }).await;
    let (mut ws1, gen1) = controller(addr).await;
    let conn1 = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
    let _guard1 = WriteHookGuard(conn1.clone());
    // Release gen 1 cleanly so gen 2 can be acquired while Client 1's writer is paused.
    let rel = ws_reply(&mut ws1, json!({"type": "release_lease", "generation": gen1}), &["lease_released"]).await;
    assert_eq!(rel["success"], true);

    armd::api::ws::set_test_ws_write_paused(&conn1, true);
    send(&mut ws1, json!({"type": "auth", "seq": 1, "token": null})).await;
    wait_until(Duration::from_secs(1), || armd::api::ws::test_ws_writer_blocked_count(&conn1) >= 1).await;

    // Client 1 acquires gen 2 (reply waits in ordinary_out_tx behind the blocked write).
    send(&mut ws1, json!({"type": "acquire_lease", "client_id": "c1", "seq": 2})).await;
    wait_until(Duration::from_secs(1), || {
        sup.inner().try_read().is_ok_and(|g| g.active_lease.as_ref().is_some_and(|l| l.client_id == conn1 && l.generation > gen1))
    }).await;
    let gen2 = sup.inner().read().await.active_lease.as_ref().unwrap().generation;

    // Client 2 force-preempts gen 2 (pushes lease_lost(gen2, preempted) onto Client 1's URGENT revocation_tx).
    let (mut ws2, _) = connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
    let g3 = ws_reply(&mut ws2, json!({"type": "acquire_lease", "client_id": "c2", "seq": 1, "force": true}), &["lease_granted"]).await;
    assert!(g3["generation"].as_u64().unwrap() > gen2);

    // Unpause Client 1's writer: lease_granted(gen2) MUST precede lease_lost(gen2, preempted).
    armd::api::ws::set_test_ws_write_paused(&conn1, false);
    let m1 = next_of(&mut ws1, &["lease_granted", "lease_lost"]).await;
    let m2 = next_of(&mut ws1, &["lease_granted", "lease_lost"]).await;
    assert_eq!((m1["type"].as_str(), m1["generation"].as_u64()), (Some("lease_granted"), Some(gen2)), "grant must precede loss: m1={m1}, m2={m2}");
    assert_eq!((m2["type"].as_str(), m2["generation"].as_u64(), m2["reason"].as_str()), (Some("lease_lost"), Some(gen2), Some("preempted")), "loss must follow grant: m1={m1}, m2={m2}");

    // (b) Action terminals are never skipped by latest-wins telemetry (even with `?obs_hz=1` decimation).
    let (addr_t, sup_t) = spawn_server(ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() }).await;
    let (mut ws_t, _) = connect_async(format!("ws://{addr_t}/api/v1/ws?obs_hz=1")).await.unwrap();
    let g = ws_reply(&mut ws_t, json!({"type": "acquire_lease", "client_id": "term", "seq": 1}), &["lease_granted"]).await;
    let gen_t = g["generation"].as_u64().unwrap();
    let conn_t = sup_t.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
    let _guard_t = WriteHookGuard(conn_t.clone());
    let ep_t = sup_t.get_observation().await.epoch_id;
    assert_eq!(rpc(&mut ws_t, gen_t, ep_t, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");

    // Stall writer on a control reply first, then emit 3 rapid actions and confirm the dispatcher
    // forwarded all 3 observations into the broadcast channel before unpausing the writer.
    let mut rx_probe = sup_t.subscribe_telemetry().unwrap();
    armd::api::ws::set_test_ws_write_paused(&conn_t, true);
    send(&mut ws_t, json!({"type": "auth", "seq": 99, "token": null})).await;
    wait_until(Duration::from_secs(1), || armd::api::ws::test_ws_writer_blocked_count(&conn_t) >= 1).await;

    let (id1, id2, id3) = {
        let mut inner = sup_t.inner().write().await;
        let id1 = inner.begin_action("move_1", None);
        inner.finish_action(id1, "done", None, None, None, None, None);
        drop(inner);
        sup_t.tick(0.02).await;
        let o1 = tokio::time::timeout(Duration::from_secs(1), rx_probe.recv()).await.unwrap().unwrap();
        assert_eq!(o1.action.as_ref().map(|a| (a.id, a.state.as_str())), Some((id1, "done")));

        let mut inner = sup_t.inner().write().await;
        let id2 = inner.begin_action("move_2", None);
        let id3 = inner.begin_action("move_3", None); // supersedes id2 -> id2 becomes previous_action, id1 dropped from latest obs!
        drop(inner);
        sup_t.tick(0.02).await;
        let o2 = tokio::time::timeout(Duration::from_secs(1), rx_probe.recv()).await.unwrap().unwrap();
        assert_eq!(o2.previous_action.as_ref().map(|a| (a.id, a.state.as_str())), Some((id2, "cancelled")));

        sup_t.tick(0.02).await;
        let _o3 = tokio::time::timeout(Duration::from_secs(1), rx_probe.recv()).await.unwrap().unwrap();
        (id1, id2, id3)
    };
    armd::api::ws::set_test_ws_write_paused(&conn_t, false);

    // Client MUST observe terminals for BOTH id1 (done) and id2 (cancelled/SUPERSEDED), plus id3.
    let mut saw_id1_done = false;
    let mut saw_id2_superseded = false;
    let deadline = Instant::now() + Duration::from_secs(2);
    while Instant::now() < deadline && (!saw_id1_done || !saw_id2_superseded) {
        use futures_util::StreamExt;
        if let Ok(Some(Ok(tokio_tungstenite::tungstenite::Message::Text(txt)))) =
            tokio::time::timeout(Duration::from_millis(200), ws_t.next()).await
        {
            if let Ok(v) = serde_json::from_str::<serde_json::Value>(&txt) {
                for a in [v.get("action"), v.get("previous_action")].into_iter().flatten() {
                    if a["id"].as_u64() == Some(id1) && a["state"].as_str() == Some("done") { saw_id1_done = true; }
                    if a["id"].as_u64() == Some(id2) && a["state"].as_str() == Some("cancelled") && a["code"].as_str() == Some("SUPERSEDED") {
                        saw_id2_superseded = true;
                    }
                }
            }
        }
    }
    assert!(saw_id1_done && saw_id2_superseded, "action terminal skipped (id1={id1} done={saw_id1_done}, id2={id2} sup={saw_id2_superseded}, id3={id3})");
}

// 15. Round 3 Item 5 (Major): auth and heartbeat are never dropped when the ordinary backlog is
//     full, and fatal outbound overflow from reader/worker flushes the explicit error frame within <= 1 s.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn auth_and_heartbeat_bypass_full_ordinary_backlog_and_fatal_overflow_flushes_error() {
    use futures_util::StreamExt;

    // (a) Full 64-frame ordinary backlog + worker blocked in slow detector: auth & heartbeat still succeed!
    let det = SlowDetector::new();
    let mut cfg = config_with_slow_task();
    cfg.lease_timeout_ms = 250;
    let sup = Arc::new(ArmSupervisor::new(cfg));
    let addr = serve_with_slow_detector(sup.clone(), det.clone()).await;
    let (mut ws, gen) = controller(addr).await;
    let ep = sup.get_observation().await.epoch_id;
    assert_eq!(rpc(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
    let ep = sup.get_observation().await.epoch_id;

    // Block worker inside slow dry-run, then saturate OrdinaryQueue (64 frames) + verify frame #65 hits BACKLOG_FULL.
    send(&mut ws, json!({"type": "command", "request_id": 100, "generation": gen, "epoch_id": ep, "path": "/api/v1/action/grasp", "body": slow_dry_run_body()})).await;
    wait_until(Duration::from_secs(1), || det.entered.load(Ordering::SeqCst)).await;
    for k in 0..65u64 {
        send(&mut ws, json!({"type": "command", "request_id": 101 + k, "generation": gen, "epoch_id": ep, "path": "/api/v1/motion/home", "body": {}})).await;
    }
    let bf = next_of(&mut ws, &["command_reject"]).await;
    assert_eq!((bf["request_id"].as_u64(), bf["code"].as_str()), (Some(165), Some("BACKLOG_FULL")), "{bf}");

    // `auth` must still be handled immediately despite full ordinary backlog.
    send(&mut ws, json!({"type": "auth", "seq": 42, "token": null})).await;
    let auth_ack = next_of(&mut ws, &["auth_ack"]).await;
    assert_eq!(auth_ack["seq"].as_u64(), Some(42), "{auth_ack}");

    // `heartbeat` must keep renewing the 250ms lease across 360ms while ordinary backlog stays 64/64 full!
    for _ in 0..8 {
        send(&mut ws, json!({"type": "heartbeat", "generation": gen, "epoch_id": ep})).await;
        tokio::time::sleep(Duration::from_millis(45)).await;
        sup.tick(0.02).await;
    }
    {
        let inner = sup.inner().read().await;
        assert!(inner.active_lease.is_some(), "lease expired while heartbeats were sent past full ordinary backlog");
        assert!(inner.watchdog_park.is_none(), "watchdog parked despite live heartbeats");
    }
    det.release_now();

    // (b) Reader-triggered fatal overflow (inbound backlog full + outbound queue full): writer flushes
    //     explicit OUTBOUND_QUEUE_FULL error frame + Close(1013) within <= 1 s grace before teardown.
    let det2 = SlowDetector::new();
    let sup2 = Arc::new(ArmSupervisor::new(config_with_slow_task()));
    let addr2 = serve_with_slow_detector(sup2.clone(), det2.clone()).await;
    let (mut ws2, gen2) = controller(addr2).await;
    let conn2 = sup2.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
    let _guard2 = WriteHookGuard(conn2.clone());
    let ep2 = sup2.get_observation().await.epoch_id;
    assert_eq!(rpc(&mut ws2, gen2, ep2, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
    let ep2 = sup2.get_observation().await.epoch_id;

    send(&mut ws2, json!({"type": "command", "request_id": 200, "generation": gen2, "epoch_id": ep2, "path": "/api/v1/action/grasp", "body": slow_dry_run_body()})).await;
    wait_until(Duration::from_secs(1), || det2.entered.load(Ordering::SeqCst)).await;
    // Pause writer and push 64 ordinary frames (fill inbound) + 66 overflow frames (1 enters writer, 64 fill outbound, #66 overflows outbound in READER!).
    armd::api::ws::set_test_ws_write_paused(&conn2, true);
    for k in 0..130u64 {
        send(&mut ws2, json!({"type": "command", "request_id": 201 + k, "generation": gen2, "epoch_id": ep2, "path": "/api/v1/motion/home", "body": {}})).await;
    }
    wait_until(Duration::from_secs(1), || armd::api::ws::test_ws_writer_blocked_count(&conn2) >= 1).await;
    tokio::time::sleep(Duration::from_millis(40)).await;
    // Unpause writer well within the <= 1 s fatal flush grace: reader already exited, writer MUST flush error + Close!
    let t_unpause = Instant::now();
    armd::api::ws::set_test_ws_write_paused(&conn2, false);
    let mut saw_err = false;
    let mut saw_close = false;
    let deadline = Instant::now() + Duration::from_secs(2);
    while Instant::now() < deadline {
        match tokio::time::timeout(Duration::from_millis(300), ws2.next()).await {
            Ok(Some(Ok(tokio_tungstenite::tungstenite::Message::Text(txt)))) => {
                if let Ok(v) = serde_json::from_str::<serde_json::Value>(&txt) {
                    if v["type"] == "error" && v["code"] == "OUTBOUND_QUEUE_FULL" { saw_err = true; }
                }
            }
            Ok(Some(Ok(tokio_tungstenite::tungstenite::Message::Close(f)))) => {
                saw_close = true;
                if let Some(cf) = f { assert_eq!(u16::from(cf.code), 1013); }
                break;
            }
            Ok(None) | Ok(Some(Err(_))) => { saw_close = true; break; }
            _ => {}
        }
    }
    assert!(saw_err, "writer was aborted before flushing OUTBOUND_QUEUE_FULL error frame");
    assert!(saw_close, "connection must close after flushing fatal error frame");
    assert!(t_unpause.elapsed() < Duration::from_secs(1), "fatal flush exceeded 1 s grace");
    det2.release_now();
}

// 16. Round 3 Item 6 (Major): teardown tracks and joins all spawned connection tasks (macro
//     preflight + urgent completions), takes the owner lock first to release any lease held by
//     the connection, and releases any late acquire immediately.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn connection_teardown_joins_spawned_tasks_and_releases_late_acquire_under_lock() {
    // (a) Macro preflight task is aborted and joined BEFORE connection teardown completes.
    let det = SlowDetector::new();
    let sup = Arc::new(ArmSupervisor::new(config_with_slow_task()));
    {
        // Place the arm at a look-down pose so `grasp::plan` succeeds and enters `grasp::preflight`.
        let arm = armd::chain::ArmModel::REBOT_DEV_ARM;
        let inspect6: [f64; 6] = std::array::from_fn(|i| INSPECT[i] as f64);
        let p0 = arm.fk(&inspect6).p;
        let goal = armd::kinematics::Pose::new(armd::grasp::look_down(&arm.fk(&inspect6).r), [p0[0], p0[1], 0.24]);
        let lo = [-3.1, -2.05, -2.6, -3.1, -2.05, -6.2];
        let hi = [3.1, 2.05, 2.6, 3.1, 2.05, 6.2];
        let mut q = inspect6;
        for _ in 0..20 {
            let sol = arm.solve_dls(&goal, &q, &lo, &hi, armd::kinematics::PAR.w);
            q = sol.q;
            if sol.ok { break; }
        }
        let q7: Vec<f32> = (0..7).map(|i| if i < 6 { q[i] as f32 } else { 0.0 }).collect();
        let mut inner = sup.inner().write().await;
        inner.commanded_positions = q7.clone();
        inner.measured_positions = q7.clone();
        inner.target_positions = q7;
    }
    let addr = serve_with_slow_detector(sup.clone(), det.clone()).await;
    let (mut ws, gen) = controller(addr).await;
    let conn_id = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
    let ep = sup.get_observation().await.epoch_id;
    let torque_ack = ws_reply(&mut ws, json!({"type": "command", "request_id": 1, "generation": gen, "epoch_id": ep, "path": "/api/v1/torque", "body": {"enable": true}}), &["command_ack", "command_reject"]).await;
    assert_eq!(torque_ack["type"], "command_ack", "{torque_ack}");
    let ep = sup.get_observation().await.epoch_id;

    // Launch a real grasp macro whose preflight blocks inside `SlowDetector::mode_info()`.
    send(&mut ws, json!({
        "type": "command", "request_id": 10, "generation": gen, "epoch_id": ep,
        "path": "/api/v1/action/grasp",
        "body": {"task": "slow", "label": "cup", "box_norm": [0.48, 0.48, 0.52, 0.52], "frame_id": 1}
    })).await;
    wait_until(Duration::from_secs(1), || det.entered.load(Ordering::SeqCst)).await;
    assert_eq!(det.active_calls.load(Ordering::SeqCst), 1, "macro preflight must be running");

    // Drop WS while macro preflight is blocked: teardown must abort+join preflight AND release lease.
    drop(ws);
    wait_until(Duration::from_secs(1), || !armd::api::ws::connection_registry().lock().unwrap().contains_key(&conn_id)).await;
    assert_eq!(det.active_calls.load(Ordering::SeqCst), 0, "macro preflight task outlived connection teardown");
    assert!(sup.inner().read().await.active_lease.is_none(), "lease must be released on teardown");
    assert!(sup.get_observation().await.action.is_none(), "aborted macro must never start");

    // (b) Late `acquire_lease` held at owner lock during disconnect is cleaned up (never leaks a lease).
    let (addr2, sup2) = spawn_server(ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() }).await;
    let (mut ws2, gen2) = controller(addr2).await;
    let conn2 = sup2.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
    let _worker_guard = WorkerLockHookGuard(conn2.clone());
    let rel = ws_reply(&mut ws2, json!({"type": "release_lease", "generation": gen2}), &["lease_released"]).await;
    assert_eq!(rel["success"], true);
    assert!(sup2.inner().read().await.active_lease.is_none());

    // Pause worker right before `inner.write()`, send acquire_lease, then drop WS while acquire is pending.
    armd::api::ws::set_test_ws_worker_before_lock_paused(&conn2, true);
    send(&mut ws2, json!({"type": "acquire_lease", "client_id": "late-acquirer", "seq": 5})).await;
    wait_until(Duration::from_secs(1), || armd::api::ws::test_ws_worker_before_lock_blocked_count(&conn2) >= 1).await;
    drop(ws2);
    // Unpause right after drop so worker either gets aborted or hits `worker_closing` under `inner.write()`.
    armd::api::ws::set_test_ws_worker_before_lock_paused(&conn2, false);
    wait_until(Duration::from_secs(1), || !armd::api::ws::connection_registry().lock().unwrap().contains_key(&conn2)).await;
    assert!(sup2.inner().read().await.active_lease.is_none(), "late acquire must never survive teardown");

    // (c) Urgent Stop completion task blocked behind owner lock is bounded-grace aborted and joined on disconnect.
    let (addr3, sup3) = spawn_server(ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() }).await;
    let (mut ws3, gen3) = controller(addr3).await;
    let conn3 = sup3.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
    let ep3 = sup3.get_observation().await.epoch_id;
    let t_ack = ws_reply(&mut ws3, json!({"type": "command", "request_id": 1, "generation": gen3, "epoch_id": ep3, "path": "/api/v1/torque", "body": {"enable": true}}), &["command_ack", "command_reject"]).await;
    assert_eq!(t_ack["type"], "command_ack", "{t_ack}");
    let ep3 = sup3.get_observation().await.epoch_id;
    let held_sup = sup3.clone();
    let hold_guard = tokio::spawn(async move {
        let _w = held_sup.inner().write().await;
        tokio::time::sleep(Duration::from_millis(400)).await;
    });
    // Wait until the lock holder definitely owns `inner.write()`.
    wait_until(Duration::from_secs(1), || sup3.inner().try_read().is_err()).await;
    // Send urgent Stop while owner lock is held, then drop WS so the urgent task cannot complete within 250ms grace.
    send(&mut ws3, json!({"type": "command", "request_id": 2, "generation": gen3, "epoch_id": ep3, "path": "/api/v1/motion/stop", "body": {}})).await;
    tokio::time::sleep(Duration::from_millis(20)).await;
    drop(ws3);
    hold_guard.await.unwrap();
    wait_until(Duration::from_secs(1), || !armd::api::ws::connection_registry().lock().unwrap().contains_key(&conn3)).await;
    assert!(sup3.inner().read().await.active_lease.is_none(), "lease must be released after teardown");
    // The urgent Stop task was aborted and joined before the lock freed, so it never ran late.
    assert_eq!(sup3.inner().read().await.safety.epoch_id(), ep3, "aborted urgent Stop task must not run after teardown");

    // (d) Direct owner-lock-first `release_lease_for_connection` releases whichever generation a connection holds.
    let mut core = SupervisorInner::new(ArmConfig::rebot_dm());
    let g_a = core.acquire_lease("connection:late").unwrap();
    assert!(g_a >= 1 && core.active_lease.is_some());
    assert!(!core.release_lease_for_connection("connection:other"));
    assert!(core.release_lease_for_connection("connection:late"));
    assert!(core.active_lease.is_none());
}

// 15. Table over EVERY motion-start entry point: admit -> Stop -> deliver -> refused (STOPPED) + zero motion.
//     Also covers WS end-to-end policy/start & preset stop-vs-motion races and lease/epoch mismatches.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn motion_start_gate_table_refuses_every_entry_point_after_stop() {
    use armd::api::rest::ApiError;
    use armd::backlash::{Pre, Request as BacklashRequest};
    use armd::gravity_comp::GravityCompRequest;
    use armd::motion::{GripperCommand, GripperRequest, JointPath};
    use armd::recording::{EpisodeSidecar, RecordFrame};
    use armd::replay::ReplayRequest;
    use armd::{AdmissionStamp, MotionStartKind};
    use std::collections::HashSet;

    let assert_stopped = |err: ArmError, label: &str| {
        let api: ApiError = err.into();
        assert_eq!(api.code.as_deref(), Some("STOPPED"), "{label}: expected STOPPED, got {api:?}");
        assert_eq!(api.status.as_u16(), 409, "{label}: expected 409, got {api:?}");
    };

    let assert_no_motion = |inner: &SupervisorInner, prev_action_id: Option<u64>, label: &str| {
        assert!(inner.active_trajectory.is_none(), "{label}: active_trajectory must remain None");
        assert!(inner.servo_target.is_none(), "{label}: servo_target must remain None");
        assert!(!inner.gravity_comp_active(), "{label}: gravity_comp must remain inactive");
        assert_eq!(
            inner.last_action.as_ref().map(|a| a.id),
            prev_action_id,
            "{label}: no new action may start"
        );
        #[cfg(feature = "policy-runtime")]
        assert!(inner.policy_run.is_none(), "{label}: policy_run must remain None");
    };

    let dummy_path = || JointPath {
        dof: 7,
        arm_dof: 6,
        q: vec![INSPECT, INSPECT],
        s: vec![0.0, 1.0],
        duration: 0.4,
        power_scale: 1.0,
    };

    let make_armed = || async {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.lease_timeout_ms = 30_000;
        let arm = TestArm::new();
        let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(arm)));
        let gen = sup.acquire_lease("conn:gate").await.unwrap();
        sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
        {
            let mut inner = sup.inner().write().await;
            inner.commanded_positions = INSPECT.to_vec();
            inner.measured_positions = INSPECT.to_vec();
            inner.target_positions = INSPECT.to_vec();
        }
        let ep = sup.get_observation().await.epoch_id;
        (sup, gen, ep)
    };

    let mut covered = HashSet::new();
    for &kind in MotionStartKind::ALL {
        covered.insert(kind);
        match kind {
            MotionStartKind::Jog => {
                let (sup, _, ep) = make_armed().await;
                let stamp = sup.capture_admission_stamp();
                let mut inner = sup.inner().write().await;
                let adm1 = inner.check_admission(stamp).unwrap();
                let adm2 = inner.check_admission(stamp).unwrap();
                assert!(inner.authorized_stop());
                assert_stopped(inner.check_admission(stamp).unwrap_err().into(), "check_admission(Jog)");
                let mut goal = INSPECT.to_vec();
                goal[0] += 0.2;
                assert_stopped(inner.jog(adm1, goal.clone(), Some(0.5), Some(ep)).unwrap_err(), "jog");
                assert_stopped(
                    inner.jog_action_with_speed_scale(adm2, goal, Some(0.5), Some(ep), "joint_move", None).unwrap_err(),
                    "jog_action_with_speed_scale",
                );
                assert_no_motion(&inner, None, "Jog");
            }
            MotionStartKind::Home => {
                let (sup, _, ep) = make_armed().await;
                let stamp = sup.capture_admission_stamp();
                {
                    let mut inner = sup.inner().write().await;
                    let adm = inner.check_admission(stamp).unwrap();
                    assert!(inner.authorized_stop());
                    let home = inner.config.home_positions.clone();
                    assert_stopped(inner.jog_action(adm, home, Some(0.5), Some(ep), "home").unwrap_err(), "home_inner");
                    assert_no_motion(&inner, None, "Home(inner)");
                }
                assert_stopped(sup.home(stamp, Some(0.5), Some(ep)).await.unwrap_err(), "sup.home");
                let inner = sup.inner().read().await;
                assert_no_motion(&inner, None, "Home(sup)");
            }
            MotionStartKind::Park => {
                let (sup, _, _) = make_armed().await;
                let stamp = sup.capture_admission_stamp();
                let mut inner = sup.inner().write().await;
                let adm = inner.check_admission(stamp).unwrap();
                assert!(inner.authorized_stop());
                assert_stopped(inner.unload_action(adm).unwrap_err(), "unload_action");
                assert_no_motion(&inner, None, "Park");
            }
            MotionStartKind::Preset => {
                let sup = Arc::new(ArmSupervisor::new(ArmConfig { lease_timeout_ms: 15_000, ..ArmConfig::rebot_dm() }));
                let addr = serve(sup.clone()).await;
                let (mut ws, gen) = controller(addr).await;
                let ep0 = sup.get_observation().await.epoch_id;
                assert_eq!(rpc(&mut ws, gen, ep0, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
                let ep = sup.get_observation().await.epoch_id;
                let conn_id = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();

                // Pause worker right after popping the preset (so Stop cannot drain it from the queue),
                // deliver urgent Stop, then resume worker: preset must still be refused via AdmissionStamp.
                armd::api::ws::set_test_ws_worker_before_lock_paused(&conn_id, true);
                send(&mut ws, json!({"type": "preset", "generation": gen, "epoch_id": ep, "name": "home"})).await;
                wait_until(Duration::from_secs(1), || {
                    armd::api::ws::test_ws_worker_before_lock_blocked_count(&conn_id) >= 1
                }).await;
                let stop_ack = rpc(&mut ws, gen, ep, "/api/v1/motion/stop", json!({})).await;
                assert_eq!(stop_ack["type"], "command_ack", "{stop_ack}");
                armd::api::ws::set_test_ws_worker_before_lock_paused(&conn_id, false);

                let reply = next_of(&mut ws, &["preset_ack", "preset_reject"]).await;
                assert_eq!(reply["type"], "preset_reject", "{reply}");
                assert_eq!(reply["code"], "STOPPED", "{reply}");
                let inner = sup.inner().read().await;
                assert_no_motion(&inner, None, "Preset");
            }
            MotionStartKind::MoveTo => {
                let (sup, _, _) = make_armed().await;
                let stamp = sup.capture_admission_stamp();
                let mut inner = sup.inner().write().await;
                let input = inner.plan_input().unwrap();
                let path = dummy_path();
                let goal: [f64; 6] = std::array::from_fn(|i| f64::from(INSPECT[i]));
                let adm1 = inner.check_admission(stamp).unwrap();
                let adm2 = inner.check_admission(stamp).unwrap();
                let adm3 = inner.check_admission(stamp).unwrap();
                let ep_cur = inner.safety.epoch_id();
                assert!(inner.authorized_stop());
                assert_stopped(inner.install_path(adm1, &input, path, "move_to", None).unwrap_err(), "install_path");
                assert_stopped(
                    inner.install_joint_goal(adm2, &input, &goal, 0.5, "move_to", None, None).unwrap_err(),
                    "install_joint_goal",
                );
                assert_stopped(
                    inner.start_path_motion_slice(adm3, ep_cur, &INSPECT, 0.5).unwrap_err(),
                    "start_path_motion_slice",
                );
                assert_no_motion(&inner, None, "MoveTo");
            }
            MotionStartKind::MoveRelative => {
                let (sup, _, _) = make_armed().await;
                let stamp = sup.capture_admission_stamp();
                let mut inner = sup.inner().write().await;
                let input = inner.plan_input().unwrap();
                let path = dummy_path();
                let adm = inner.check_admission(stamp).unwrap();
                assert!(inner.authorized_stop());
                assert_stopped(
                    inner.install_path_same_side(adm, &input, path, Pre::Off, None, "move_relative", None).unwrap_err(),
                    "install_path_same_side",
                );
                assert_no_motion(&inner, None, "MoveRelative");
            }
            MotionStartKind::MoveToPose => {
                let (sup, _, ep) = make_armed().await;
                {
                    let mut inner = sup.inner().write().await;
                    let mut p = INSPECT.to_vec();
                    p[0] += 0.15;
                    armd::poses::save(&mut inner, &json!({"name": "gate_pose", "positions": p})).unwrap();
                }
                let stamp = sup.capture_admission_stamp();
                let mut inner = sup.inner().write().await;
                let adm1 = inner.check_admission(stamp).unwrap();
                let adm2 = inner.check_admission(stamp).unwrap();
                assert!(inner.authorized_stop());
                assert_stopped(
                    inner.move_to_pose(adm1, "gate_pose", false, 0.5, None, ep).unwrap_err(),
                    "move_to_pose",
                );
                assert_stopped(
                    armd::poses::start_action(&mut inner, adm2, &json!({"name": "gate_pose"}), ep).unwrap_err(),
                    "poses::start_action",
                );
                assert_no_motion(&inner, None, "MoveToPose");
            }
            MotionStartKind::Place => {
                let (sup, _, _) = make_armed().await;
                let stamp = sup.capture_admission_stamp();
                let mut inner = sup.inner().write().await;
                let input = inner.plan_input().unwrap();
                let adm = inner.check_admission(stamp).unwrap();
                assert!(inner.authorized_stop());
                assert_stopped(inner.begin_macro(adm, "place", &input, None).unwrap_err(), "begin_macro(place)");
                assert_no_motion(&inner, None, "Place");
            }
            MotionStartKind::Grasp => {
                let (sup, _, _) = make_armed().await;
                let stamp = sup.capture_admission_stamp();
                let mut inner = sup.inner().write().await;
                let input = inner.plan_input().unwrap();
                let adm = inner.check_admission(stamp).unwrap();
                assert!(inner.authorized_stop());
                assert_stopped(inner.begin_macro(adm, "grasp", &input, None).unwrap_err(), "begin_macro(grasp)");
                assert_no_motion(&inner, None, "Grasp");
            }
            MotionStartKind::Replay => {
                let (sup, _, ep) = make_armed().await;
                let dir = tempfile::tempdir().unwrap();
                let (plan, req) = {
                    let mut inner = sup.inner().write().await;
                    inner.episodes = armd::recording::EpisodeStore::new(Some(dir.path().to_path_buf()));
                    let frames = vec![
                        RecordFrame { timestamp: 0.0, observation_state: INSPECT.to_vec(), action: INSPECT.to_vec() },
                        RecordFrame { timestamp: 0.1, observation_state: INSPECT.to_vec(), action: INSPECT.to_vec() },
                    ];
                    let sidecar = EpisodeSidecar {
                        id: "gate_ep".into(),
                        dataset: "gate".into(),
                        task: "gate".into(),
                        source: "sdk".into(),
                        sequence_id: None,
                        repeat_index: None,
                        outcome: "ok".into(),
                        recorded_at: "2026-11-24T00:00:00Z".into(),
                        duration_s: 0.1,
                        frames: frames.len(),
                        joint_names: inner.config.limits.iter().map(|l| l.name.clone()).collect(),
                        units: "rad".into(),
                        joint_zero: armd::recording::JointZeroStamp::of(&inner.joint_zero_offsets),
                        profile_id: inner.config.profile_id.clone(),
                        armd_version: "test".into(),
                        fps_nominal: 10.0,
                        truncated: false,
                        stopped_by: None,
                        arm_dof: Some(6),
                        gripper_joint: Some(6),
                        tool_id: Some(inner.config.active_tool.id.clone()),
                        tool_config_digest: Some(inner.config.active_tool.config_digest.clone()),
                        has_gripper_motion: Some(false),
                        recovered: None,
                        frames_dropped: None,
                        first_state: Some(INSPECT.to_vec()),
                        last_state: Some(INSPECT.to_vec()),
                    };
                    inner.episodes.write_episode(&sidecar, &frames, None).unwrap();
                    let req = ReplayRequest { episode: "gate_ep".into(), speed: 1.0, gripper: false, tag: None, allow_stale: true };
                    let inp = inner.replay_input(req.clone(), ep).unwrap();
                    (armd::replay::plan(inp).unwrap(), req)
                };
                let stamp = sup.capture_admission_stamp();
                let mut inner = sup.inner().write().await;
                let adm1 = inner.check_admission(stamp).unwrap();
                let adm2 = inner.check_admission(stamp).unwrap();
                assert!(inner.authorized_stop());
                assert_stopped(inner.install_replay(adm1, plan).unwrap_err(), "install_replay");
                assert_stopped(inner.start_replay(adm2, req, ep).unwrap_err(), "start_replay");
                assert_no_motion(&inner, None, "Replay");
            }
            MotionStartKind::Sequence => {
                let (sup, _, ep) = make_armed().await;
                let run_plan = {
                    let mut inner = sup.inner().write().await;
                    let (saved_pose, _) = armd::poses::save(&mut inner, &json!({"name": "seq_p", "positions": INSPECT.to_vec()})).unwrap();
                    let (seq, _) = armd::sequences::save(&mut inner, &json!({
                        "name": "gate_seq",
                        "steps": [{"type": "pose", "pose_id": saved_pose.id}]
                    })).unwrap();
                    armd::sequences::prepare_run(&inner, &json!({"id": seq.id}), ep).unwrap()
                };
                let stamp = sup.capture_admission_stamp();
                let mut inner = sup.inner().write().await;
                let adm1 = inner.check_admission(stamp).unwrap();
                let adm2 = inner.check_admission(stamp).unwrap();
                assert!(inner.authorized_stop());
                assert_stopped(inner.begin_sequence_action(adm1, None, ep).unwrap_err(), "begin_sequence_action");
                assert_stopped(armd::sequences::begin(&mut inner, adm2, &sup, run_plan).unwrap_err(), "sequences::begin");
                assert_no_motion(&inner, None, "Sequence");
            }
            MotionStartKind::CalibHover => {
                let (sup, _, ep) = make_armed().await;
                let stamp = sup.capture_admission_stamp();
                let mut inner = sup.inner().write().await;
                let adm = inner.check_admission(stamp).unwrap();
                assert!(inner.authorized_stop());
                assert_stopped(
                    inner.begin_orchestrated_action(adm, "calib_hover", None, ep).unwrap_err(),
                    "begin_orchestrated_action(calib_hover)",
                );
                assert_no_motion(&inner, None, "CalibHover");
            }
            MotionStartKind::MeasureMultiview => {
                let (sup, _, ep) = make_armed().await;
                let stamp = sup.capture_admission_stamp();
                let mut inner = sup.inner().write().await;
                let adm = inner.check_admission(stamp).unwrap();
                assert!(inner.authorized_stop());
                assert_stopped(
                    inner.begin_orchestrated_action(adm, "measure_multiview", None, ep).unwrap_err(),
                    "begin_orchestrated_action(measure_multiview)",
                );
                assert_no_motion(&inner, None, "MeasureMultiview");
            }
            MotionStartKind::MacroStart => {
                let (sup, _, _) = make_armed().await;
                let stamp = sup.capture_admission_stamp();
                let mut inner = sup.inner().write().await;
                let input = inner.plan_input().unwrap();
                let adm = inner.check_admission(stamp).unwrap();
                assert!(inner.authorized_stop());
                assert_stopped(inner.begin_macro(adm, "grasp", &input, None).unwrap_err(), "MacroStart");
                assert_no_motion(&inner, None, "MacroStart");
            }
            MotionStartKind::MacroChildPath => {
                let (sup, _, _) = make_armed().await;
                let stamp = sup.capture_admission_stamp();
                let mut inner = sup.inner().write().await;
                let input = inner.plan_input().unwrap();
                let adm0 = inner.check_admission(stamp).unwrap();
                let macro_id = inner.begin_macro(adm0, "grasp", &input, None).unwrap();
                let path = dummy_path();
                let adm1 = inner.check_admission(stamp).unwrap();
                let adm2 = inner.check_admission(stamp).unwrap();
                assert!(inner.authorized_stop());
                assert_stopped(
                    inner.install_macro_path(adm1, macro_id, &input, path.clone()).unwrap_err(),
                    "install_macro_path",
                );
                assert_stopped(
                    inner.install_macro_path_same_side(adm2, macro_id, &input, path, Pre::Off, None).unwrap_err(),
                    "install_macro_path_same_side",
                );
                assert_no_motion(&inner, Some(macro_id), "MacroChildPath");
            }
            MotionStartKind::MacroChildQuintic => {
                let (sup, _, _) = make_armed().await;
                let stamp = sup.capture_admission_stamp();
                let mut inner = sup.inner().write().await;
                let input = inner.plan_input().unwrap();
                let adm0 = inner.check_admission(stamp).unwrap();
                let macro_id = inner.begin_macro(adm0, "grasp", &input, None).unwrap();
                let limits = inner.config.limits.clone();
                let adm1 = inner.check_admission(stamp).unwrap();
                assert!(inner.authorized_stop());
                assert_stopped(
                    inner.install_macro_quintic(adm1, macro_id, INSPECT.to_vec(), &limits).unwrap_err(),
                    "install_macro_quintic",
                );
                assert_no_motion(&inner, Some(macro_id), "MacroChildQuintic");
            }
            MotionStartKind::MacroChildGripper => {
                let (sup, _, _) = make_armed().await;
                let stamp = sup.capture_admission_stamp();
                let mut inner = sup.inner().write().await;
                let input = inner.plan_input().unwrap();
                let adm0 = inner.check_admission(stamp).unwrap();
                let macro_id = inner.begin_macro(adm0, "grasp", &input, None).unwrap();
                let greq = GripperRequest { command: GripperCommand::Open, fraction: None, grasp: false, speed: 1.0, timeout_s: Some(2.0), tag: None };
                let adm1 = inner.check_admission(stamp).unwrap();
                let adm2 = inner.check_admission(stamp).unwrap();
                let adm3 = inner.check_admission(stamp).unwrap();
                let adm4 = inner.check_admission(stamp).unwrap();
                assert!(inner.authorized_stop());
                assert_stopped(
                    inner.start_macro_gripper(adm1, macro_id, greq.clone()).unwrap_err(),
                    "start_macro_gripper",
                );
                assert_stopped(
                    inner.start_macro_gripper_within(adm2, macro_id, greq.clone(), Some(5.0), true).unwrap_err(),
                    "start_macro_gripper_within",
                );
                assert_stopped(
                    inner.start_macro_gripper_learning(adm3, macro_id, greq.clone(), armd::motion::LearnMode::Calibration).unwrap_err(),
                    "start_macro_gripper_learning",
                );
                assert_stopped(
                    inner.start_macro_gripper_learning_within(adm4, macro_id, greq, armd::motion::LearnMode::Calibration, Some(5.0), false).unwrap_err(),
                    "start_macro_gripper_learning_within",
                );
                assert_no_motion(&inner, Some(macro_id), "MacroChildGripper");
            }
            MotionStartKind::MacroChildReplay => {
                let (sup, _, ep) = make_armed().await;
                let dir = tempfile::tempdir().unwrap();
                let (macro_id, plan, stamp) = {
                    let mut inner = sup.inner().write().await;
                    inner.episodes = armd::recording::EpisodeStore::new(Some(dir.path().to_path_buf()));
                    let frames = vec![
                        RecordFrame { timestamp: 0.0, observation_state: INSPECT.to_vec(), action: INSPECT.to_vec() },
                        RecordFrame { timestamp: 0.1, observation_state: INSPECT.to_vec(), action: INSPECT.to_vec() },
                    ];
                    let sidecar = EpisodeSidecar {
                        id: "gate_ep_child".into(),
                        dataset: "gate".into(),
                        task: "gate".into(),
                        source: "sdk".into(),
                        sequence_id: None,
                        repeat_index: None,
                        outcome: "ok".into(),
                        recorded_at: "2026-11-24T00:00:00Z".into(),
                        duration_s: 0.1,
                        frames: frames.len(),
                        joint_names: inner.config.limits.iter().map(|l| l.name.clone()).collect(),
                        units: "rad".into(),
                        joint_zero: armd::recording::JointZeroStamp::of(&inner.joint_zero_offsets),
                        profile_id: inner.config.profile_id.clone(),
                        armd_version: "test".into(),
                        fps_nominal: 10.0,
                        truncated: false,
                        stopped_by: None,
                        arm_dof: Some(6),
                        gripper_joint: Some(6),
                        tool_id: Some(inner.config.active_tool.id.clone()),
                        tool_config_digest: Some(inner.config.active_tool.config_digest.clone()),
                        has_gripper_motion: Some(false),
                        recovered: None,
                        frames_dropped: None,
                        first_state: Some(INSPECT.to_vec()),
                        last_state: Some(INSPECT.to_vec()),
                    };
                    inner.episodes.write_episode(&sidecar, &frames, None).unwrap();
                    let stamp = inner.current_admission_stamp();
                    let adm0 = inner.check_admission(stamp).unwrap();
                    let id = inner.begin_sequence_action(adm0, None, ep).unwrap();
                    let req = ReplayRequest { episode: "gate_ep_child".into(), speed: 1.0, gripper: false, tag: None, allow_stale: true };
                    let inp = inner.replay_input(req, ep).unwrap();
                    (id, armd::replay::plan(inp).unwrap(), stamp)
                };
                let mut inner = sup.inner().write().await;
                let adm = inner.check_admission(stamp).unwrap();
                assert!(inner.authorized_stop());
                assert_stopped(inner.install_macro_replay(adm, macro_id, plan).unwrap_err(), "install_macro_replay");
                assert_no_motion(&inner, Some(macro_id), "MacroChildReplay");
            }
            MotionStartKind::Servo => {
                let (sup, gen, ep) = make_armed().await;
                let stamp = sup.capture_admission_stamp();
                let mut inner = sup.inner().write().await;
                let adm1 = inner.check_admission(stamp).unwrap();
                let adm2 = inner.check_admission(stamp).unwrap();
                let adm3 = inner.check_admission(stamp).unwrap();
                assert!(inner.authorized_stop());
                assert_stopped(inner.servo(adm1, INSPECT.to_vec(), ep).unwrap_err().into(), "servo");
                assert_stopped(inner.servo_slice(adm2, &INSPECT, ep).unwrap_err().into(), "servo_slice");
                assert_stopped(inner.authorize_servo_stream(adm3, gen).unwrap_err().into(), "authorize_servo_stream");
                assert_no_motion(&inner, None, "Servo");
            }
            MotionStartKind::PolicyShadow => {
                #[cfg(feature = "policy-runtime")]
                {
                    let (sup, gen, ep) = make_armed().await;
                    let stamp = sup.capture_admission_stamp();
                    let mut inner = sup.inner().write().await;
                    let adm = inner.check_admission(stamp).unwrap();
                    assert!(inner.authorized_stop());
                    let err = inner.begin_policy_run(adm, armd::policy_runtime::PolicyRunStart {
                        epoch_id: ep,
                        connection_id: "conn:gate".into(),
                        lease_generation: gen,
                        policy_id: "p_shadow".into(),
                        artifact_digest: "sha256:0000".into(),
                        backend: "mlp".into(),
                        mode: armd::policy_runtime::PolicyMode::Shadow,
                        sim_only: false,
                        max_first_step_rad: None,
                        max_step_rad: None,
                        tag: None,
                        i_am_at_the_arm: false,
                        verified_release_gate: None,
                    }).unwrap_err();
                    assert_stopped(err, "begin_policy_run(Shadow)");
                    assert_no_motion(&inner, None, "PolicyShadow");
                }
            }
            MotionStartKind::PolicyLive => {
                #[cfg(feature = "policy-runtime")]
                {
                    let (sup, gen, ep) = make_armed().await;
                    let stamp = sup.capture_admission_stamp();
                    let mut inner = sup.inner().write().await;
                    let adm = inner.check_admission(stamp).unwrap();
                    assert!(inner.authorized_stop());
                    let err = inner.begin_policy_run(adm, armd::policy_runtime::PolicyRunStart {
                        epoch_id: ep,
                        connection_id: "conn:gate".into(),
                        lease_generation: gen,
                        policy_id: "p_live".into(),
                        artifact_digest: "sha256:0000".into(),
                        backend: "mlp".into(),
                        mode: armd::policy_runtime::PolicyMode::Live,
                        sim_only: false,
                        max_first_step_rad: None,
                        max_step_rad: None,
                        tag: None,
                        i_am_at_the_arm: true,
                        verified_release_gate: None,
                    }).unwrap_err();
                    assert_stopped(err, "begin_policy_run(Live)");
                    assert_no_motion(&inner, None, "PolicyLive");
                }
            }
            MotionStartKind::GravityComp => {
                let (sup, _, _) = make_armed().await;
                let stamp = sup.capture_admission_stamp();
                let mut inner = sup.inner().write().await;
                let adm = inner.check_admission(stamp).unwrap();
                assert!(inner.authorized_stop());
                assert_stopped(
                    inner.enter_gravity_comp(adm, GravityCompRequest { scale: None, damping_kd: None }).unwrap_err(),
                    "enter_gravity_comp",
                );
                assert_no_motion(&inner, None, "GravityComp");
            }
            MotionStartKind::Gripper => {
                let (sup, _, ep) = make_armed().await;
                let stamp = sup.capture_admission_stamp();
                let mut inner = sup.inner().write().await;
                let greq = GripperRequest { command: GripperCommand::Open, fraction: None, grasp: false, speed: 1.0, timeout_s: Some(2.0), tag: None };
                let adm1 = inner.check_admission(stamp).unwrap();
                let adm2 = inner.check_admission(stamp).unwrap();
                assert!(inner.authorized_stop());
                assert_stopped(inner.start_gripper(adm1, greq.clone(), ep).unwrap_err(), "start_gripper");
                assert_stopped(
                    inner.start_gripper_with_baseline(adm2, greq, ep, armd::supervisor::GripperBaselineCmd::Reset).unwrap_err(),
                    "start_gripper_with_baseline",
                );
                assert_no_motion(&inner, None, "Gripper");
            }
            MotionStartKind::GripperCalibrate => {
                let (sup, _, ep) = make_armed().await;
                let stamp = sup.capture_admission_stamp();
                {
                    let mut inner = sup.inner().write().await;
                    let adm = inner.check_admission(stamp).unwrap();
                    assert!(inner.authorized_stop());
                    assert_stopped(
                        armd::gripper_calib::begin_calibration(&mut inner, adm, &sup, ep, None).unwrap_err(),
                        "gripper_calib::begin_calibration",
                    );
                    assert_no_motion(&inner, None, "GripperCalibrate(begin)");
                }
                assert_stopped(
                    armd::gripper_calib::start_calibration(&sup, stamp).await.unwrap_err(),
                    "gripper_calib::start_calibration",
                );
                let inner = sup.inner().read().await;
                assert_no_motion(&inner, None, "GripperCalibrate(start)");
            }
            MotionStartKind::GripperZeroCheck => {
                let (sup, _, ep) = make_armed().await;
                let stamp = sup.capture_admission_stamp();
                let mut inner = sup.inner().write().await;
                let adm = inner.check_admission(stamp).unwrap();
                assert!(inner.authorized_stop());
                assert_stopped(
                    inner.start_gripper_zero_check(adm, &json!({}), ep).unwrap_err(),
                    "start_gripper_zero_check",
                );
                assert_no_motion(&inner, None, "GripperZeroCheck");
            }
            MotionStartKind::TorqueEnable => {
                // Direct owner-lock & async facade check: stale stamp refuses enable_torque without
                // arming, while unstamped disable_torque remains allowed after a Stop.
                let mut cfg = ArmConfig::rebot_dm();
                cfg.lease_timeout_ms = 30_000;
                let sup = Arc::new(ArmSupervisor::new(cfg));
                let _gen = sup.acquire_lease("conn:torque-gate").await.unwrap();
                let stale_stamp = sup.capture_admission_stamp();
                {
                    let mut inner = sup.inner().write().await;
                    let adm = inner.check_admission(stale_stamp).unwrap();
                    assert!(inner.authorized_stop());
                    assert_stopped(ArmError::from(inner.enable_torque(adm).unwrap_err()), "enable_torque(inner)");
                    assert!(!inner.is_torqued, "stale enable_torque must not enable torque");
                }
                assert_stopped(sup.enable_torque(stale_stamp).await.unwrap_err(), "enable_torque(sup)");
                assert!(!sup.get_observation().await.is_torqued);

                // Enable cleanly with fresh stamp, then verify disable_torque stays unstamped after Stop.
                let fresh_stamp = sup.capture_admission_stamp();
                sup.enable_torque(fresh_stamp).await.unwrap();
                assert!(sup.get_observation().await.is_torqued);
                {
                    let mut inner = sup.inner().write().await;
                    assert!(inner.authorized_stop());
                    inner.disable_torque().expect("disable_torque stays unstamped after Stop");
                    assert!(!inner.is_torqued);
                }

                // End-to-end WS /api/v1/torque and /api/v1/arm (same path as calibration wizard):
                // enable admitted before Stop is refused under owner lock; disable admitted before
                // Stop still succeeds as a stop-direction action.
                let sup_ws = Arc::new(ArmSupervisor::new(ArmConfig { lease_timeout_ms: 15_000, ..ArmConfig::rebot_dm() }));
                let addr = serve(sup_ws.clone()).await;
                let (mut ws, gen) = controller(addr).await;
                let conn_id = sup_ws.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
                let mut next_rid = 3000u64;
                for path in ["/api/v1/torque", "/api/v1/arm"] {
                    let ep = sup_ws.get_observation().await.epoch_id;
                    armd::api::ws::set_test_ws_worker_before_lock_paused(&conn_id, true);
                    let en_rid = next_rid;
                    next_rid += 1;
                    send(&mut ws, json!({
                        "type": "command", "request_id": en_rid, "generation": gen, "epoch_id": ep,
                        "path": path, "body": {"enable": true}
                    })).await;
                    wait_until(Duration::from_secs(2), || {
                        armd::api::ws::test_ws_worker_before_lock_blocked_count(&conn_id) >= 1
                    }).await;
                    let stop_rid = next_rid;
                    next_rid += 1;
                    send(&mut ws, json!({
                        "type": "command", "request_id": stop_rid, "generation": gen, "epoch_id": ep,
                        "path": "/api/v1/motion/stop", "body": {}
                    })).await;
                    let stop_ack = next_of(&mut ws, &["command_ack"]).await;
                    assert_eq!(stop_ack["request_id"], stop_rid, "{stop_ack}");
                    armd::api::ws::set_test_ws_worker_before_lock_paused(&conn_id, false);
                    let en_rej = next_of(&mut ws, &["command_ack", "command_reject"]).await;
                    assert_eq!(en_rej["request_id"], en_rid, "{path}: {en_rej}");
                    assert_eq!(en_rej["type"], "command_reject", "{path}: {en_rej}");
                    assert_eq!(en_rej["code"], "STOPPED", "{path}: {en_rej}");
                    assert!(!sup_ws.get_observation().await.is_torqued, "{path} must not arm after Stop");
                }
                // Arm cleanly, then verify WS torque disable admitted before Stop still succeeds.
                let ep = sup_ws.get_observation().await.epoch_id;
                let arm_rid = next_rid;
                next_rid += 1;
                send(&mut ws, json!({
                    "type": "command", "request_id": arm_rid, "generation": gen, "epoch_id": ep,
                    "path": "/api/v1/torque", "body": {"enable": true}
                })).await;
                let arm_ack = next_of(&mut ws, &["command_ack", "command_reject"]).await;
                assert_eq!((arm_ack["request_id"].as_u64(), arm_ack["type"].as_str()), (Some(arm_rid), Some("command_ack")), "{arm_ack}");
                assert!(sup_ws.get_observation().await.is_torqued);

                let ep2 = sup_ws.get_observation().await.epoch_id;
                armd::api::ws::set_test_ws_worker_before_lock_paused(&conn_id, true);
                let dis_rid = next_rid;
                next_rid += 1;
                send(&mut ws, json!({
                    "type": "command", "request_id": dis_rid, "generation": gen, "epoch_id": ep2,
                    "path": "/api/v1/torque", "body": {"enable": false}
                })).await;
                wait_until(Duration::from_secs(2), || {
                    armd::api::ws::test_ws_worker_before_lock_blocked_count(&conn_id) >= 1
                }).await;
                let stop_rid = next_rid;
                send(&mut ws, json!({
                    "type": "command", "request_id": stop_rid, "generation": gen, "epoch_id": ep2,
                    "path": "/api/v1/motion/stop", "body": {}
                })).await;
                let stop_ack = next_of(&mut ws, &["command_ack"]).await;
                assert_eq!(stop_ack["request_id"], stop_rid, "{stop_ack}");
                armd::api::ws::set_test_ws_worker_before_lock_paused(&conn_id, false);
                let dis_ack = next_of(&mut ws, &["command_ack", "command_reject"]).await;
                assert_eq!((dis_ack["request_id"].as_u64(), dis_ack["type"].as_str()), (Some(dis_rid), Some("command_ack")), "disable must remain unstamped after Stop: {dis_ack}");
                assert!(!sup_ws.get_observation().await.is_torqued);
            }
        }
    }
    assert_eq!(covered.len(), MotionStartKind::ALL.len(), "every MotionStartKind must be tested");

    // End-to-end WS policy/start race (round-3 finding): admit policy/start, apply urgent Stop
    // while worker is loading (and also when paused before inner.write), verify STOPPED & no policy run.
    #[cfg(feature = "policy-runtime")]
    {
        use armd::api::create_router_with_sandbox;
        use armd::api::sandbox::SandboxState;
        use armd::policy_runtime::{encode_rmlp_bytes, Activation, MlpLayerSpec};

        let mut cfg = ArmConfig::default();
        cfg.lease_timeout_ms = 15_000;
        let jnames: Vec<&str> = cfg.limits.iter().map(|l| l.name.as_str()).collect();
        let dir = tempfile::tempdir().unwrap();
        let pack_dir = dir.path().join("policies").join("gate_pol");
        std::fs::create_dir_all(&pack_dir).unwrap();
        let layers = vec![MlpLayerSpec::new(6, 6, Activation::None, vec![0.0; 36], vec![0.0; 6])];
        std::fs::write(pack_dir.join("model.rmlp"), encode_rmlp_bytes(&layers).unwrap()).unwrap();
        std::fs::write(
            pack_dir.join("policy.json"),
            serde_json::to_vec(&json!({
                "format": "rebot-onnx-policy/v1",
                "id": "gate_pol",
                "name": "gate_pol",
                "unit_mode": "rad",
                "action_dim": 6,
                "action_space": "joint_positions",
                "joint_names": jnames,
                "inputs": {"state": {"shape": [1, 6], "joint_names": jnames}},
                "outputs": {"action": {"shape": [1, 6], "space": "joint_positions"}},
                "model_file": "model.rmlp"
            })).unwrap(),
        ).unwrap();

        let sandbox = Arc::new(SandboxState::new(dir.path().to_path_buf()));
        sandbox.set_policy_load_delay_ms(120);
        let sup = Arc::new(ArmSupervisor::new(cfg));
        let router = create_router_with_sandbox(sup.clone(), OperatorAuth::disabled(), sandbox);
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });

        let (mut ws, gen) = controller(addr).await;
        let ep = sup.get_observation().await.epoch_id;
        // Send policy/start (admitted at stop_gen=0), wait until loading starts, then send urgent Stop.
        send(&mut ws, json!({
            "type": "command", "request_id": 1001, "generation": gen, "epoch_id": ep,
            "path": "/api/v1/action/policy/start",
            "body": {"policy_id": "gate_pol", "mode": "shadow"}
        })).await;
        wait_until(Duration::from_secs(2), || {
            sup.inner().try_read().is_ok_and(|g| g.policy_loading)
        }).await;
        send(&mut ws, json!({
            "type": "command", "request_id": 1002, "generation": gen, "epoch_id": ep,
            "path": "/api/v1/motion/stop", "body": {}
        })).await;

        let mut saw_stop_ack = false;
        let mut saw_policy_stopped = false;
        let deadline = Instant::now() + Duration::from_secs(3);
        while !(saw_stop_ack && saw_policy_stopped) && Instant::now() < deadline {
            let msg = next_of(&mut ws, &["command_ack", "command_reject"]).await;
            if msg["request_id"] == 1002 {
                assert_eq!(msg["type"], "command_ack", "{msg}");
                assert!(msg["stop_generation"].as_u64().unwrap_or(0) >= 1, "{msg}");
                saw_stop_ack = true;
            } else if msg["request_id"] == 1001 {
                assert_eq!(msg["type"], "command_reject", "{msg}");
                assert_eq!(msg["code"], "STOPPED", "{msg}");
                saw_policy_stopped = true;
            }
        }
        assert!(saw_stop_ack && saw_policy_stopped, "stop={saw_stop_ack} policy={saw_policy_stopped}");
        assert!(sup.inner().read().await.policy_run.is_none(), "policy must NOT start after Stop");
    }

    // Also verify lease_gen mismatch -> LEASE_LOST and estop_epoch mismatch -> ESTOP_ACTIVE / EPOCH_MISMATCH.
    {
        let (sup, gen, ep) = make_armed().await;
        let inner = sup.inner().read().await;
        let sg = inner.current_stop_gen();
        let err_lease = inner.check_admission(AdmissionStamp { stop_gen: sg, estop_epoch: ep, lease_gen: gen + 1, connection_id: 0 }).unwrap_err();
        assert_eq!(ApiError::from(ArmError::from(err_lease)).code.as_deref(), Some("LEASE_LOST"));
        let err_ep = inner.check_admission(AdmissionStamp { stop_gen: sg, estop_epoch: ep + 9, lease_gen: gen, connection_id: 0 }).unwrap_err();
        assert_eq!(ApiError::from(ArmError::from(err_ep)).code.as_deref(), Some("EPOCH_MISMATCH"));
        drop(inner);
        let _ = sup.trigger_e_stop_checked().await.unwrap();
        let inner = sup.inner().read().await;
        let err_estop = inner.check_admission(AdmissionStamp { stop_gen: sg, estop_epoch: ep, lease_gen: gen, connection_id: 0 }).unwrap_err();
        assert_eq!(ApiError::from(ArmError::from(err_estop)).code.as_deref(), Some("ESTOP_ACTIVE"));
    }
    let _ = BacklashRequest::Profile;
}

// 16. Single start_motion(admitted, op) wrapper contract & static non-resampling / signature audit.
#[test]
fn start_motion_wrapper_enforces_stamp_and_no_motion_entry_point_resamples() {
    use armd::api::rest::ApiError;
    use armd::{AdmissionStamp, Admitted, MotionStartKind, MotionStartOp};

    struct CountedOp {
        kind: MotionStartKind,
        executed: Arc<AtomicBool>,
    }

    impl MotionStartOp for CountedOp {
        type Output = AdmissionStamp;
        fn kind(&self) -> MotionStartKind { self.kind }
        fn execute(self, _inner: &mut SupervisorInner, admitted: Admitted) -> Result<Self::Output, ArmError> {
            self.executed.store(true, Ordering::SeqCst);
            Ok(admitted.stamp())
        }
    }

    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
    let gen = inner.acquire_lease("conn:op").unwrap();
    let stamp = inner.current_admission_stamp();
    assert_eq!(stamp.lease_gen, gen);

    // Valid stamp produces single-use Admitted, runs `execute`, and returns the validated stamp.
    let ran = Arc::new(AtomicBool::new(false));
    let admitted = inner.check_admission(stamp).unwrap();
    let out = inner.start_motion(admitted, CountedOp { kind: MotionStartKind::Jog, executed: ran.clone() }).unwrap();
    assert!(ran.load(Ordering::SeqCst));
    assert_eq!(out, stamp);

    // Even an Admitted created before Stop is refused after Stop and NEVER invokes `execute`.
    let stale_admitted = inner.check_admission(stamp).unwrap();
    assert!(inner.authorized_stop());
    let ran_stop = Arc::new(AtomicBool::new(false));
    let err = inner.start_motion(stale_admitted, CountedOp { kind: MotionStartKind::PolicyLive, executed: ran_stop.clone() }).unwrap_err();
    assert!(!ran_stop.load(Ordering::SeqCst), "execute must NOT run on stale stop_gen");
    assert_eq!(ApiError::from(err).code.as_deref(), Some("STOPPED"));
    assert_eq!(ApiError::from(ArmError::from(inner.check_admission(stamp).unwrap_err())).code.as_deref(), Some("STOPPED"));

    // Stale lease_gen refuses at check_admission with LEASE_LOST.
    let fresh = inner.current_admission_stamp();
    let err = inner.check_admission(AdmissionStamp { lease_gen: fresh.lease_gen + 1, ..fresh }).unwrap_err();
    assert_eq!(ApiError::from(ArmError::from(err)).code.as_deref(), Some("LEASE_LOST"));

    // Static audit: production supervisor motion-start modules (including grasp & place!) must
    // never re-sample `current_admission_stamp` or `capture_admission_stamp`, and every motion or
    // output start function must require `Admitted` by value in its signature.
    let manifest_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    for rel in [
        "src/supervisor/motion.rs",
        "src/supervisor/actions.rs",
        "src/supervisor/streams.rs",
        "src/supervisor/driver_transitions.rs",
        "src/gravity_comp.rs",
        "src/poses.rs",
        "src/backlash.rs",
        "src/grasp.rs",
        "src/place.rs",
        "src/gripper_calib.rs",
        "src/gripper_baseline.rs",
    ] {
        let src = std::fs::read_to_string(manifest_dir.join(rel)).unwrap();
        let prod = src.split("#[cfg(test)]").next().unwrap_or(&src);
        assert!(
            !prod.contains("current_admission_stamp") && !prod.contains("capture_admission_stamp"),
            "{rel} must never re-sample an admission stamp in production code"
        );
    }
    let required_signatures = [
        ("src/supervisor/driver_transitions.rs", "pub fn enable_torque("),
        ("src/supervisor/driver_transitions.rs", "pub(crate) fn begin_enable_torque_transition("),
        ("src/supervisor/motion.rs", "pub fn jog("),
        ("src/supervisor/motion.rs", "pub fn jog_action("),
        ("src/supervisor/motion.rs", "pub fn jog_action_with_speed_scale("),
        ("src/supervisor/motion.rs", "pub(crate) fn start_quintic("),
        ("src/supervisor/motion.rs", "pub fn install_path("),
        ("src/supervisor/motion.rs", "pub fn install_joint_goal("),
        ("src/supervisor/motion.rs", "pub fn start_path_motion_slice("),
        ("src/supervisor/actions.rs", "pub fn unload_action("),
        ("src/supervisor/actions.rs", "pub fn begin_macro("),
        ("src/supervisor/actions.rs", "pub fn begin_sequence_action("),
        ("src/supervisor/actions.rs", "pub fn begin_orchestrated_action("),
        ("src/supervisor/actions.rs", "pub fn install_macro_path("),
        ("src/supervisor/actions.rs", "pub fn start_macro_gripper("),
        ("src/supervisor/actions.rs", "pub fn start_macro_gripper_learning("),
        ("src/supervisor/actions.rs", "pub fn start_macro_gripper_learning_within("),
        ("src/supervisor/actions.rs", "pub fn install_macro_quintic("),
        ("src/supervisor/actions.rs", "pub fn start_gripper("),
        ("src/supervisor/actions.rs", "pub fn start_gripper_with_baseline("),
        ("src/supervisor/actions.rs", "pub fn start_gripper_zero_check("),
        ("src/gripper_calib.rs", "pub fn begin_calibration("),
        ("src/supervisor/streams.rs", "pub fn servo("),
        ("src/supervisor/streams.rs", "pub fn begin_policy_run("),
        ("src/supervisor/streams.rs", "pub fn authorize_servo_stream("),
        ("src/gravity_comp.rs", "pub fn enter_gravity_comp("),
        ("src/poses.rs", "pub fn move_to_pose("),
        ("src/poses.rs", "pub fn start_action("),
        ("src/replay.rs", "pub fn install_replay("),
        ("src/replay.rs", "pub fn install_macro_replay("),
        ("src/replay.rs", "pub fn start_replay("),
        ("src/replay.rs", "pub fn start_replay_motion("),
    ];
    for (rel, sig_prefix) in required_signatures {
        let src = std::fs::read_to_string(manifest_dir.join(rel)).unwrap();
        let idx = src.find(sig_prefix).unwrap_or_else(|| panic!("missing {sig_prefix} in {rel}"));
        let sig_slice = &src[idx..src[idx..].find('{').map_or(src.len(), |o| idx + o)];
        assert!(
            sig_slice.contains("Admitted"),
            "{rel}::{sig_prefix} lacks Admitted parameter: {sig_slice}"
        );
    }
}

// 17. Item 2 (E-stop intake): off-reader auth, bounded O(1) registry, and urgent replies flushed
//     BEFORE fatal error + Close when a blocked writer recovers, or E-stop visible in CloseFrame
//     reason when a blocked writer never recovers within bounded grace.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn estop_intake_blocked_writer_recovers_vs_never_recovers_and_off_reader_auth() {
    use futures_util::StreamExt;
    use tokio_tungstenite::tungstenite::Message;

    // (a) Blocked writer that RECOVERS before grace: queued E-stop & Stop replies MUST arrive
    //     BEFORE the fatal OUTBOUND_QUEUE_FULL error frame and BEFORE Close.
    {
        let (addr, sup) = spawn_server(ArmConfig { lease_timeout_ms: 15_000, ..ArmConfig::rebot_dm() }).await;
        let (mut ws, gen) = controller(addr).await;
        let conn_id = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
        let _guard = WriteHookGuard(conn_id.clone());
        let ep0 = sup.get_observation().await.epoch_id;
        assert_eq!(rpc(&mut ws, gen, ep0, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
        let ep = sup.get_observation().await.epoch_id;

        armd::api::ws::set_test_ws_write_paused(&conn_id, true);
        // Trigger one ordinary reply so writer enters `write_ws_frame` and blocks.
        send(&mut ws, json!({"type": "command", "request_id": 7000, "generation": gen, "epoch_id": ep, "path": "/api/v1/motion/jog", "body": {"target_positions": [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "duration_secs": 0.5}})).await;
        wait_until(Duration::from_secs(1), || armd::api::ws::test_ws_writer_blocked_count(&conn_id) >= 1).await;

        // Send urgent Stop + urgent E-stop while writer is blocked.
        send(&mut ws, json!({"type": "command", "request_id": 7001, "generation": gen, "epoch_id": ep, "path": "/api/v1/motion/stop", "body": {}})).await;
        let t_estop = Instant::now();
        send(&mut ws, json!({"type": "estop"})).await;
        wait_until(Duration::from_millis(80), || sup.e_stop_latched.load(Ordering::SeqCst)).await;
        assert!(t_estop.elapsed() < Duration::from_millis(80), "E-stop did not latch immediately");
        wait_until(Duration::from_secs(1), || sup.inner().try_read().is_ok_and(|g| *g.safety.state() == SafetyState::EStop)).await;

        // Overflow ordinary outbound queue (64 capacity + extra) to trigger FatalClose::outbound_queue_full().
        for k in 0..70u64 {
            send(&mut ws, json!({"type": "command", "request_id": 7100 + k, "generation": gen, "epoch_id": ep, "path": "/api/v1/motion/jog", "body": {}})).await;
        }
        tokio::time::sleep(Duration::from_millis(30)).await;

        // Writer recovers within grace: urgent replies MUST flush BEFORE fatal error and Close.
        armd::api::ws::set_test_ws_write_paused(&conn_id, false);
        let mut estop_idx: Option<usize> = None;
        let mut stop_idx: Option<usize> = None;
        let mut fatal_idx: Option<usize> = None;
        let mut close_idx: Option<usize> = None;
        let mut idx = 0usize;
        let deadline = Instant::now() + Duration::from_secs(3);
        let mut seen_frames = Vec::new();
        while Instant::now() < deadline {
            match tokio::time::timeout(Duration::from_secs(1), ws.next()).await {
                Ok(Some(Ok(Message::Text(text)))) => {
                    let v: serde_json::Value = serde_json::from_str(&text).unwrap();
                    seen_frames.push(format!("{idx}:Text({})", v.get("type").and_then(|t| t.as_str()).unwrap_or("obs")));
                    if v["type"] == "epoch_sync" && v["safety_state"] == "EStop" && estop_idx.is_none() {
                        estop_idx = Some(idx);
                    }
                    if (v["type"] == "command_ack" || v["type"] == "command_reject") && v["request_id"] == 7001 && stop_idx.is_none() {
                        stop_idx = Some(idx);
                    }
                    if v["type"] == "error" && v["code"] == "OUTBOUND_QUEUE_FULL" && fatal_idx.is_none() {
                        fatal_idx = Some(idx);
                    }
                    idx += 1;
                }
                Ok(Some(Ok(Message::Close(cf)))) => {
                    seen_frames.push(format!("{idx}:Close({cf:?})"));
                    close_idx = Some(idx);
                    break;
                }
                Ok(other) => {
                    seen_frames.push(format!("{idx}:Other({other:?})"));
                    break;
                }
                Err(_) => {
                    seen_frames.push("TIMEOUT".into());
                    break;
                }
            }
        }
        let ei = estop_idx.unwrap_or_else(|| panic!("E-stop epoch_sync must be delivered when writer recovers; seen={seen_frames:?}"));
        let si = stop_idx.unwrap_or_else(|| panic!("Stop reply must be delivered when writer recovers; seen={seen_frames:?}"));
        let fi = fatal_idx.unwrap_or_else(|| panic!("fatal OUTBOUND_QUEUE_FULL error frame must be delivered; seen={seen_frames:?}"));
        let ci = close_idx.unwrap_or_else(|| panic!("Close frame must follow fatal error frame; seen={seen_frames:?}"));
        assert!(ei < fi && fi < ci, "expected estop({ei}) < fatal({fi}) < close({ci})");
        assert!(si < fi && fi < ci, "expected stop({si}) < fatal({fi}) < close({ci})");
        assert!(sup.e_stop_latched.load(Ordering::SeqCst));
        assert_eq!(*sup.inner().read().await.safety.state(), SafetyState::EStop);
    }

    // (b1) Blocked writer that NEVER recovers (write timeout): E-stop still latches immediately
    //      and client sees ESTOP_LATCHED in the CloseFrame reason after bounded grace.
    {
        let (addr, sup) = spawn_server(ArmConfig { lease_timeout_ms: 15_000, ..ArmConfig::rebot_dm() }).await;
        let (mut ws, gen) = controller(addr).await;
        let conn_id = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
        let _guard = WriteHookGuard(conn_id.clone());
        let ep = sup.get_observation().await.epoch_id;

        armd::api::ws::set_test_ws_write_timeout(&conn_id, Some(Duration::from_millis(120)));
        armd::api::ws::set_test_ws_write_paused(&conn_id, true);
        send(&mut ws, json!({"type": "command", "request_id": 8000, "generation": gen, "epoch_id": ep, "path": "/api/v1/torque", "body": {"enable": true}})).await;
        wait_until(Duration::from_secs(1), || armd::api::ws::test_ws_writer_blocked_count(&conn_id) >= 1).await;

        send(&mut ws, json!({"type": "estop"})).await;
        wait_until(Duration::from_millis(80), || sup.e_stop_latched.load(Ordering::SeqCst)).await;
        wait_until(Duration::from_secs(1), || sup.inner().try_read().is_ok_and(|g| *g.safety.state() == SafetyState::EStop)).await;

        // Never unpause writer: connection must close after grace with ESTOP_LATCHED in CloseFrame reason.
        let mut close_reason: Option<String> = None;
        let deadline = Instant::now() + Duration::from_secs(2);
        while Instant::now() < deadline {
            match tokio::time::timeout(Duration::from_millis(500), ws.next()).await {
                Ok(Some(Ok(Message::Close(Some(cf))))) => {
                    close_reason = Some(cf.reason.to_string());
                    break;
                }
                Ok(Some(Ok(_))) => {}
                _ => break,
            }
        }
        let reason = close_reason.expect("client must observe CloseFrame reason when blocked writer times out");
        assert!(reason.contains("ESTOP_LATCHED"), "expected ESTOP_LATCHED in close reason, got {reason:?}");
        assert!(sup.e_stop_latched.load(Ordering::SeqCst));
        assert_eq!(*sup.inner().read().await.safety.state(), SafetyState::EStop);
        wait_until(Duration::from_secs(1), || sup.inner().try_read().is_ok_and(|g| g.active_lease.is_none())).await;
    }

    // (b2) Blocked writer that NEVER recovers + fatal overflow: E-stop latched and visible in CloseFrame reason.
    {
        let (addr, sup) = spawn_server(ArmConfig { lease_timeout_ms: 15_000, ..ArmConfig::rebot_dm() }).await;
        let (mut ws, gen) = controller(addr).await;
        let conn_id = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
        let _guard = WriteHookGuard(conn_id.clone());
        let ep = sup.get_observation().await.epoch_id;

        armd::api::ws::set_test_ws_write_timeout(&conn_id, Some(Duration::from_millis(140)));
        armd::api::ws::set_test_ws_write_paused(&conn_id, true);
        send(&mut ws, json!({"type": "command", "request_id": 8100, "generation": gen, "epoch_id": ep, "path": "/api/v1/torque", "body": {"enable": true}})).await;
        wait_until(Duration::from_secs(1), || armd::api::ws::test_ws_writer_blocked_count(&conn_id) >= 1).await;

        send(&mut ws, json!({"type": "estop"})).await;
        wait_until(Duration::from_millis(80), || sup.e_stop_latched.load(Ordering::SeqCst)).await;
        for k in 0..70u64 {
            send(&mut ws, json!({"type": "command", "request_id": 8101 + k, "generation": gen, "epoch_id": ep, "path": "/api/v1/motion/jog", "body": {}})).await;
        }

        let mut close_reason: Option<String> = None;
        let deadline = Instant::now() + Duration::from_secs(2);
        while Instant::now() < deadline {
            match tokio::time::timeout(Duration::from_millis(500), ws.next()).await {
                Ok(Some(Ok(Message::Close(Some(cf))))) => {
                    close_reason = Some(cf.reason.to_string());
                    break;
                }
                Ok(Some(Ok(_))) => {}
                _ => break,
            }
        }
        let reason = close_reason.expect("client must observe CloseFrame reason on fatal close while permanently blocked");
        assert!(reason.contains("ESTOP_LATCHED"), "expected ESTOP_LATCHED in close reason, got {reason:?}");
        assert!(sup.e_stop_latched.load(Ordering::SeqCst));
        wait_until(Duration::from_secs(1), || sup.inner().try_read().is_ok_and(|g| g.active_lease.is_none())).await;
    }

    // (c) Off-reader auth task: auth messages succeed via dedicated `auth_task`, never blocking reader.
    {
        let auth = OperatorAuth::enforced("1234", 300);
        let (tok, _) = auth.unlock("1234").unwrap();
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        tokio::spawn(async move {
            axum::serve(listener, armd::create_router_with_auth(sup, auth)).await.unwrap();
        });
        let (mut ws, _) = connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
        let ack = ws_reply(&mut ws, json!({"type": "auth", "seq": 7, "token": tok}), &["auth_ack"]).await;
        assert_eq!(ack["seq"], 7, "{ack}");
        assert_eq!(ack["role"], "operator", "{ack}");
    }
}

// 18. Item 3 (Writer ordering):
//     (a) Heartbeat-triggered `lease_lost(G)` arriving on `ordinary_out_rx` before `lease_granted(G)`
//         goes through the same causal deferral so `lease_granted(G)` ALWAYS precedes `lease_lost(G)`.
//     (b) Action terminals go through the lossless ordinary control queue (never lost on broadcast lag),
//         and `RecvError::Lagged` resyncs telemetry from the supervisor hub.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn writer_ordering_heartbeat_loss_deferred_and_broadcast_lag_never_loses_terminals() {
    use futures_util::StreamExt;
    use tokio_tungstenite::tungstenite::Message;

    // (a) Heartbeat-triggered `lease_lost(G)` races ahead of `lease_granted(G)` into `ordinary_out_tx`:
    //     writer MUST defer `lease_lost(G)` until `lease_granted(G)` is written on the wire.
    {
        struct DelayReset;
        impl Drop for DelayReset {
            fn drop(&mut self) {
                armd::api::ws::TEST_LEASE_GRANTED_DELAY_MS.store(0, Ordering::Relaxed);
            }
        }
        let _reset = DelayReset;
        let (addr, sup) = spawn_server(ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() }).await;
        let (mut ws, _) = connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
        let ep = sup.get_observation().await.epoch_id;

        // Delay enqueueing `lease_granted` AFTER `acquire_lease` releases the owner lock.
        armd::api::ws::TEST_LEASE_GRANTED_DELAY_MS.store(120, Ordering::Relaxed);
        send(&mut ws, json!({"type": "acquire_lease", "client_id": "hb_defer", "seq": 1})).await;
        // Wait until the owner lock recorded the lease (while `lease_granted` is still delayed).
        wait_until(Duration::from_secs(1), || {
            sup.inner().try_read().is_ok_and(|g| g.active_lease.as_ref().is_some_and(|l| l.label == "hb_defer"))
        }).await;
        let gen = {
            let mut inner = sup.inner().write().await;
            let lease = inner.active_lease.as_mut().unwrap();
            let g = lease.generation;
            // Expire the lease directly (no second connection / no revocation_rx!) so ONLY
            // `heartbeat_task` produces `lease_lost(g)` via `ordinary_out_tx`.
            lease.acquired_at = Instant::now() - Duration::from_secs(60);
            g
        };
        send(&mut ws, json!({"type": "heartbeat", "generation": gen, "epoch_id": ep})).await;

        let m1 = next_of(&mut ws, &["lease_granted", "lease_lost"]).await;
        let m2 = next_of(&mut ws, &["lease_granted", "lease_lost"]).await;
        assert_eq!(
            (m1["type"].as_str(), m1["generation"].as_u64()),
            (Some("lease_granted"), Some(gen)),
            "lease_granted MUST precede heartbeat lease_lost: m1={m1}, m2={m2}"
        );
        assert_eq!(
            (m2["type"].as_str(), m2["generation"].as_u64()),
            (Some("lease_lost"), Some(gen)),
            "deferred heartbeat lease_lost MUST follow lease_granted: m1={m1}, m2={m2}"
        );
    }

    // (b) Broadcast lag (>128 telemetry frames dropped while writer paused): ALL action terminals
    //     arrive intact via `ordinary_out_rx`, AND telemetry resyncs to latest hub state after lag.
    {
        let (addr, sup) = spawn_server(ArmConfig { lease_timeout_ms: 15_000, ..ArmConfig::rebot_dm() }).await;
        let (mut ws, gen) = controller(addr).await;
        let conn = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
        let _guard = WriteHookGuard(conn.clone());
        let ep = sup.get_observation().await.epoch_id;
        assert_eq!(rpc(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");

        // Independent broadcast probe subscribed before the flood so we prove broadcast truly lagged.
        let mut lag_probe = sup.subscribe_telemetry().unwrap();

        armd::api::ws::set_test_ws_write_paused(&conn, true);
        send(&mut ws, json!({"type": "auth", "seq": 42, "token": null})).await;
        wait_until(Duration::from_secs(1), || armd::api::ws::test_ws_writer_blocked_count(&conn) >= 1).await;

        // Finish two distinct actions, then overwrite both slots with a running third action.
        let (id1, id2, id3) = {
            let mut inner = sup.inner().write().await;
            let id1 = inner.begin_action("lag_move_1", None);
            inner.finish_action(id1, "done", None, None, None, None, None);
            drop(inner);
            sup.tick(0.02).await;

            let mut inner = sup.inner().write().await;
            let id2 = inner.begin_action("lag_move_2", None);
            inner.finish_action(id2, "failed", Some("TEST_FAIL"), Some("boom".into()), None, None, None);
            drop(inner);
            sup.tick(0.02).await;

            let mut inner = sup.inner().write().await;
            let id3 = inner.begin_action("lag_move_3", None);
            drop(inner);
            sup.tick(0.02).await;
            (id1, id2, id3)
        };

        // Flood > 180 ticks so `telemetry_tx` (broadcast cap 128) overflows and drops earlier frames!
        let mut latest_ts = 0.0f64;
        for _ in 0..180 {
            let obs = sup.tick(0.02).await;
            latest_ts = obs.timestamp;
            // Yield periodically so the dispatcher forwards into the broadcast channel to overflow it.
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
        // Confirm broadcast truly lagged for a receiver paused as long as `ws`.
        let mut saw_lag = false;
        while let Ok(res) = tokio::time::timeout(Duration::from_millis(50), lag_probe.recv()).await {
            if matches!(res, Err(tokio::sync::broadcast::error::RecvError::Lagged(_))) {
                saw_lag = true;
                break;
            }
        }
        assert!(saw_lag, "test setup must overflow broadcast channel to verify lag resilience");

        // Unpause writer: client MUST receive BOTH id1 (done) and id2 (failed) terminals via the
        // ordinary control queue AND a resynced latest telemetry observation (>= latest_ts).
        armd::api::ws::set_test_ws_write_paused(&conn, false);
        let mut saw_id1 = false;
        let mut saw_id2 = false;
        let mut saw_resynced = false;
        let deadline = Instant::now() + Duration::from_secs(3);
        while Instant::now() < deadline && !(saw_id1 && saw_id2 && saw_resynced) {
            if let Ok(Some(Ok(Message::Text(txt)))) =
                tokio::time::timeout(Duration::from_millis(250), ws.next()).await
            {
                if let Ok(v) = serde_json::from_str::<serde_json::Value>(&txt) {
                    for a in [v.get("action"), v.get("previous_action")].into_iter().flatten() {
                        if a["id"].as_u64() == Some(id1) && a["state"].as_str() == Some("done") {
                            saw_id1 = true;
                        }
                        if a["id"].as_u64() == Some(id2)
                            && a["state"].as_str() == Some("failed")
                            && a["code"].as_str() == Some("TEST_FAIL")
                        {
                            saw_id2 = true;
                        }
                    }
                    if v.get("type").is_none()
                        && v["timestamp"].as_f64().is_some_and(|ts| ts >= latest_ts - 1e-6)
                        && v["action"]["id"].as_u64() == Some(id3)
                    {
                        saw_resynced = true;
                    }
                }
            }
        }
        assert!(
            saw_id1 && saw_id2 && saw_resynced,
            "broadcast lag dropped terminal or missed resync: id1={saw_id1}, id2={saw_id2}, resync={saw_resynced} (latest_ts={latest_ts})"
        );
    }
}

// 19. Item 4 (Teardown): disconnecting during blocking `spawn_blocking` planning (Cartesian move_to,
//     replay, and policy load) cancels between steps and ignores any late result — nothing is
//     installed and no lease remains held.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn teardown_cancels_and_ignores_late_blocking_planners_with_no_install_or_lease() {
    struct BlockingPlanGuard(String);
    impl Drop for BlockingPlanGuard {
        fn drop(&mut self) {
            armd::api::ws::set_test_ws_blocking_plan_paused(&self.0, false);
        }
    }

    // (a) Disconnect mid-blocking Cartesian `move_to` planning (both linear and joint path modes),
    //     then release the blocking thread late after teardown: nothing installed, no lease held.
    for path_mode in ["linear", "joint"] {
        let (addr, sup) = spawn_server(ArmConfig { lease_timeout_ms: 15_000, ..ArmConfig::rebot_dm() }).await;
        let (mut ws, gen) = controller(addr).await;
        let ep0 = sup.get_observation().await.epoch_id;
        assert_eq!(rpc(&mut ws, gen, ep0, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
        let (conn, ep) = {
            let mut inner = sup.inner().write().await;
            inner.commanded_positions = INSPECT.to_vec();
            inner.measured_positions = INSPECT.to_vec();
            inner.target_positions = INSPECT.to_vec();
            let c = inner.active_lease.as_ref().unwrap().client_id.clone();
            let e = inner.safety.epoch_id();
            (c, e)
        };
        let _guard = BlockingPlanGuard(conn.clone());

        armd::api::ws::set_test_ws_blocking_plan_paused(&conn, true);
        send(&mut ws, json!({
            "type": "command", "request_id": 9001, "generation": gen, "epoch_id": ep,
            "path": "/api/v1/action/move_relative",
            "body": {"frame": "base", "d": [0.02, 0.0, 0.0], "path": path_mode}
        })).await;
        wait_until(Duration::from_secs(2), || {
            armd::api::ws::test_ws_blocking_plan_blocked_count(&conn) >= 1
        }).await;

        // Disconnect while the `spawn_blocking` planner is running.
        drop(ws);
        // Teardown must complete promptly without waiting on the blocked thread, and release the lease.
        wait_until(Duration::from_secs(2), || {
            !armd::api::ws::connection_registry().lock().unwrap().contains_key(&conn)
        }).await;
        assert!(sup.inner().read().await.active_lease.is_none(), "{path_mode}: lease must be released");
        assert!(sup.inner().read().await.active_trajectory.is_none(), "{path_mode}: trajectory must not install");
        assert!(sup.inner().read().await.last_action.is_none(), "{path_mode}: action must not install");

        // Now let the late blocking thread wake and exit: STILL nothing installed, no lease held.
        armd::api::ws::set_test_ws_blocking_plan_paused(&conn, false);
        tokio::time::sleep(Duration::from_millis(80)).await;
        let inner = sup.inner().read().await;
        assert!(inner.active_lease.is_none(), "{path_mode}: late planner must not hold lease");
        assert!(inner.active_trajectory.is_none(), "{path_mode}: late planner must not install trajectory");
        assert!(inner.last_action.is_none(), "{path_mode}: late planner must not install action");
    }

    // (b) Disconnect mid-blocking `replay` planning, then release late: nothing installed, no lease held.
    {
        use armd::recording::{EpisodeSidecar, RecordFrame};
        let dir = tempfile::tempdir().unwrap();
        let (addr, sup) = spawn_server(ArmConfig { lease_timeout_ms: 15_000, ..ArmConfig::rebot_dm() }).await;
        let (mut ws, gen) = controller(addr).await;
        let ep0 = sup.get_observation().await.epoch_id;
        assert_eq!(rpc(&mut ws, gen, ep0, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
        let (conn, ep) = {
            let mut inner = sup.inner().write().await;
            inner.commanded_positions = INSPECT.to_vec();
            inner.measured_positions = INSPECT.to_vec();
            inner.target_positions = INSPECT.to_vec();
            inner.episodes = armd::recording::EpisodeStore::new(Some(dir.path().to_path_buf()));
            let frames = vec![
                RecordFrame { timestamp: 0.0, observation_state: INSPECT.to_vec(), action: INSPECT.to_vec() },
                RecordFrame { timestamp: 0.1, observation_state: INSPECT.to_vec(), action: INSPECT.to_vec() },
            ];
            let sidecar = EpisodeSidecar {
                id: "td_ep".into(),
                dataset: "td".into(),
                task: "td".into(),
                source: "sdk".into(),
                sequence_id: None,
                repeat_index: None,
                outcome: "ok".into(),
                recorded_at: "2026-11-24T00:00:00Z".into(),
                duration_s: 0.1,
                frames: frames.len(),
                joint_names: inner.config.limits.iter().map(|l| l.name.clone()).collect(),
                units: "rad".into(),
                joint_zero: armd::recording::JointZeroStamp::of(&inner.joint_zero_offsets),
                profile_id: inner.config.profile_id.clone(),
                armd_version: "test".into(),
                fps_nominal: 10.0,
                truncated: false,
                stopped_by: None,
                arm_dof: Some(6),
                gripper_joint: Some(6),
                tool_id: Some(inner.config.active_tool.id.clone()),
                tool_config_digest: Some(inner.config.active_tool.config_digest.clone()),
                has_gripper_motion: Some(false),
                recovered: None,
                frames_dropped: None,
                first_state: Some(INSPECT.to_vec()),
                last_state: Some(INSPECT.to_vec()),
            };
            inner.episodes.write_episode(&sidecar, &frames, None).unwrap();
            (inner.active_lease.as_ref().unwrap().client_id.clone(), inner.safety.epoch_id())
        };
        let _guard = BlockingPlanGuard(conn.clone());

        armd::api::ws::set_test_ws_blocking_plan_paused(&conn, true);
        send(&mut ws, json!({
            "type": "command", "request_id": 9101, "generation": gen, "epoch_id": ep,
            "path": "/api/v1/action/replay",
            "body": {"episode": "td_ep", "speed": 1.0, "gripper": false, "allow_stale": true}
        })).await;
        wait_until(Duration::from_secs(2), || {
            armd::api::ws::test_ws_blocking_plan_blocked_count(&conn) >= 1
        }).await;

        drop(ws);
        wait_until(Duration::from_secs(2), || {
            !armd::api::ws::connection_registry().lock().unwrap().contains_key(&conn)
        }).await;
        armd::api::ws::set_test_ws_blocking_plan_paused(&conn, false);
        tokio::time::sleep(Duration::from_millis(80)).await;
        let inner = sup.inner().read().await;
        assert!(inner.active_lease.is_none(), "replay teardown must release lease");
        assert!(inner.active_trajectory.is_none(), "late replay planner must not install trajectory");
        assert!(inner.last_action.is_none(), "late replay planner must not install action");
    }

    // (c) Disconnect mid-blocking `policy/start` load (slow delay): cancels early via cancel flag,
    //     clears `policy_loading`, installs no policy, and holds no lease.
    #[cfg(feature = "policy-runtime")]
    {
        use armd::api::create_router_with_sandbox;
        use armd::api::sandbox::SandboxState;
        use armd::policy_runtime::{encode_rmlp_bytes, Activation, MlpLayerSpec};

        let mut cfg = ArmConfig::default();
        cfg.lease_timeout_ms = 15_000;
        let jnames: Vec<&str> = cfg.limits.iter().map(|l| l.name.as_str()).collect();
        let dir = tempfile::tempdir().unwrap();
        let pack_dir = dir.path().join("policies").join("td_pol");
        std::fs::create_dir_all(&pack_dir).unwrap();
        let layers = vec![MlpLayerSpec::new(6, 6, Activation::None, vec![0.0; 36], vec![0.0; 6])];
        std::fs::write(pack_dir.join("model.rmlp"), encode_rmlp_bytes(&layers).unwrap()).unwrap();
        std::fs::write(
            pack_dir.join("policy.json"),
            serde_json::to_vec(&json!({
                "format": "rebot-onnx-policy/v1",
                "id": "td_pol",
                "name": "td_pol",
                "unit_mode": "rad",
                "action_dim": 6,
                "action_space": "joint_positions",
                "joint_names": jnames,
                "inputs": {"state": {"shape": [1, 6], "joint_names": jnames}},
                "outputs": {"action": {"shape": [1, 6], "space": "joint_positions"}},
                "model_file": "model.rmlp"
            })).unwrap(),
        ).unwrap();

        let sandbox = Arc::new(SandboxState::new(dir.path().to_path_buf()));
        sandbox.set_policy_load_delay_ms(800);
        let sup = Arc::new(ArmSupervisor::new(cfg));
        let router = create_router_with_sandbox(sup.clone(), OperatorAuth::disabled(), sandbox);
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });

        let (mut ws, gen) = controller(addr).await;
        let conn = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
        let ep = sup.get_observation().await.epoch_id;
        send(&mut ws, json!({
            "type": "command", "request_id": 9201, "generation": gen, "epoch_id": ep,
            "path": "/api/v1/action/policy/start",
            "body": {"policy_id": "td_pol", "mode": "shadow"}
        })).await;
        wait_until(Duration::from_secs(2), || {
            sup.inner().try_read().is_ok_and(|g| g.policy_loading)
        }).await;

        let t_drop = Instant::now();
        drop(ws);
        wait_until(Duration::from_secs(2), || {
            !armd::api::ws::connection_registry().lock().unwrap().contains_key(&conn)
        }).await;
        assert!(t_drop.elapsed() < Duration::from_millis(600), "teardown must cancel policy load promptly");
        wait_until(Duration::from_secs(1), || {
            sup.inner().try_read().is_ok_and(|g| !g.policy_loading)
        }).await;
        tokio::time::sleep(Duration::from_millis(120)).await;
        let inner = sup.inner().read().await;
        assert!(inner.active_lease.is_none(), "policy teardown must release lease");
        assert!(inner.policy_run.is_none(), "cancelled policy load must not install policy_run");
        assert!(!inner.policy_loading, "policy_loading must remain cleared");
    }
}

// 20. Round-5 Item 3:
//     (a) Full Stop reply queue (>64 Stops with stalled writer) NEVER causes reader_task to exit:
//         reader keeps reading, coalesces Stops, latches an E-stop right behind them, and only
//         writer_task closes the connection after its bounded grace.
//     (b) Full auth reply queue (auth_task exits on auth_out_tx full) + another auth frame + E-stop:
//         reader STILL does not exit and latches the E-stop immediately.
//     (c) Lock-free reader E-stop registration: holding the ACK-drain `estop_reg` mutex does NOT
//         block the WS reader from latching `e_stop_latched` and incrementing `requested_estop_count`;
//         the executor applies E-stop while the mutex is still held, and releasing it delivers the
//         exact pinned `epoch_sync`.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn full_stop_reply_queue_never_exits_reader_and_estop_registration_is_lock_free() {
    // (a) Flood >64 Stops while writer is stalled (filling the 64-slot stop_out_tx), then send E-stop.
    {
        let (addr, sup) = spawn_server(ArmConfig { lease_timeout_ms: 15_000, ..ArmConfig::rebot_dm() }).await;
        let (mut ws, gen) = controller(addr).await;
        let conn = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
        let _guard = WriteHookGuard(conn.clone());
        let ep0 = sup.get_observation().await.epoch_id;
        assert_eq!(rpc(&mut ws, gen, ep0, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
        let ep = sup.get_observation().await.epoch_id;

        // Give writer a generous timeout so it stays stalled during the Stop flood, then closes cleanly.
        armd::api::ws::set_test_ws_write_timeout(&conn, Some(Duration::from_millis(600)));
        armd::api::ws::set_test_ws_write_paused(&conn, true);
        // Block writer on one ordinary frame.
        send(&mut ws, json!({"type": "command", "request_id": 1, "generation": gen, "epoch_id": ep, "path": "/api/v1/motion/home", "body": {}})).await;
        wait_until(Duration::from_secs(1), || armd::api::ws::test_ws_writer_blocked_count(&conn) >= 1).await;

        // Send 75 Stops (exceeding URGENT_OUT_CAPACITY = 64; each distinct invalid epoch bypasses
        // coalescing so all 64 slots of stop_out_tx truly fill and the 65th+ hits Full).
        for k in 0..75u64 {
            send(&mut ws, json!({
                "type": "command", "request_id": 100 + k, "generation": gen, "epoch_id": ep + 1000 + k,
                "path": "/api/v1/motion/stop", "body": {}
            })).await;
        }
        // Also send 5 coalescing Stops behind the full queue, followed immediately by E-stop.
        for k in 0..5u64 {
            send(&mut ws, json!({
                "type": "command", "request_id": 200 + k, "generation": gen, "epoch_id": ep,
                "path": "/api/v1/motion/stop", "body": {}
            })).await;
        }
        send(&mut ws, json!({"type": "estop"})).await;

        // Reader MUST stay alive past the full Stop reply queue and latch the E-stop immediately!
        wait_until(Duration::from_millis(250), || sup.e_stop_latched.load(Ordering::SeqCst)).await;
        wait_until(Duration::from_millis(250), || {
            sup.inner().try_read().is_ok_and(|g| *g.safety.state() == SafetyState::EStop)
        }).await;
        // Connection must NOT have been closed prematurely by the reader while writer is within grace;
        // only writer_task closes after its bounded grace.
        assert!(
            armd::api::ws::connection_registry().lock().unwrap().contains_key(&conn),
            "reader must not close connection on full Stop reply queue while writer is still within grace"
        );
        armd::api::ws::set_test_ws_write_paused(&conn, false);
        wait_until(Duration::from_secs(2), || {
            !armd::api::ws::connection_registry().lock().unwrap().contains_key(&conn)
        }).await;
    }

    // (b) Auth reply queue full (auth_task stops on auth_out_tx full) + extra auth + E-stop:
    //     reader does NOT exit and latches the E-stop immediately.
    {
        let (addr, sup) = spawn_server(ArmConfig { lease_timeout_ms: 15_000, ..ArmConfig::rebot_dm() }).await;
        let (mut ws, gen) = controller(addr).await;
        let conn = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
        let _guard = WriteHookGuard(conn.clone());
        let ep = sup.get_observation().await.epoch_id;

        armd::api::ws::set_test_ws_write_timeout(&conn, Some(Duration::from_millis(600)));
        armd::api::ws::set_test_ws_write_paused(&conn, true);
        send(&mut ws, json!({"type": "command", "request_id": 1, "generation": gen, "epoch_id": ep, "path": "/api/v1/motion/home", "body": {}})).await;
        wait_until(Duration::from_secs(1), || armd::api::ws::test_ws_writer_blocked_count(&conn) >= 1).await;

        for seq in 1..=40u64 {
            send(&mut ws, json!({"type": "auth", "seq": seq, "token": null})).await;
        }
        send(&mut ws, json!({"type": "estop"})).await;
        wait_until(Duration::from_millis(250), || sup.e_stop_latched.load(Ordering::SeqCst)).await;
        armd::api::ws::set_test_ws_write_paused(&conn, false);
        wait_until(Duration::from_secs(2), || {
            !armd::api::ws::connection_registry().lock().unwrap().contains_key(&conn)
        }).await;
    }

    // (c) Lock-free reader E-stop registration: holding `estop_reg` (the ACK-drain mutex) must NOT
    //     block the WS reader from latching `e_stop_latched` and incrementing `requested_estop_count`.
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig { lease_timeout_ms: 15_000, ..ArmConfig::rebot_dm() }));
        let addr = serve(sup.clone()).await;
        let (mut ws, gen) = controller(addr).await;
        let ep0 = sup.get_observation().await.epoch_id;
        assert_eq!(rpc(&mut ws, gen, ep0, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
        let handle = sup.start_executor(ExecutorConfig::for_tests(Duration::from_millis(5))).await.unwrap();
        wait_until(Duration::from_secs(1), || handle.stats().steps.load(Ordering::Relaxed) >= 2).await;
        let ep_pre = sup.observation().await.unwrap().epoch_id;
        let req_before = sup.test_requested_estop_count();

        // Hold the ACK-drain mutex across WS E-stop intake AND executor application!
        let held = sup.test_lock_estop_registry();
        send(&mut ws, json!({"type": "estop"})).await;
        wait_until(Duration::from_millis(200), || {
            sup.e_stop_latched.load(Ordering::SeqCst)
                && sup.test_requested_estop_count() == req_before + 1
        }).await;
        // Executor applies E-stop on its next release even while `estop_reg` is still held!
        wait_until(Duration::from_millis(200), || {
            sup.hub().latest().is_some_and(|s| s.obs.safety_state == SafetyState::EStop && s.obs.epoch_id > ep_pre)
        }).await;
        let applied_epoch = sup.hub().latest().unwrap().obs.epoch_id;

        // Release `estop_reg`: off-reader `finish_e_stop_checked` drains the early ACK and replies
        // with the exact pinned epoch.
        drop(held);
        let reply = next_of(&mut ws, &["epoch_sync", "estop_reject"]).await;
        assert_eq!(
            (reply["type"].as_str(), reply["safety_state"].as_str(), reply["epoch_id"].as_u64()),
            (Some("epoch_sync"), Some("EStop"), Some(applied_epoch)),
            "early ACK after lock-free reader registration must resolve to exact pinned epoch: {reply}"
        );
        handle.stop_and_join();
    }
}

// 21. Round-5 Item 4:
//     On write timeout, the writer abandons the in-progress stalled write, then attempts queued
//     urgent E-stop replies and the fatal error frame (each with a short bounded budget) BEFORE
//     sending Close.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn write_timeout_abandons_stalled_write_and_attempts_queued_estop_replies_and_fatal_before_close() {
    use futures_util::StreamExt;
    use tokio_tungstenite::tungstenite::Message;

    // (a) Transient one-shot stall on an in-progress ordinary write + two queued E-stop replies:
    //     in-progress write times out and is abandoned -> queued E-stop replies arrive on the wire,
    //     followed by the fatal WRITE_TIMEOUT error frame, followed by Close.
    {
        let (addr, sup) = spawn_server(ArmConfig { lease_timeout_ms: 15_000, ..ArmConfig::rebot_dm() }).await;
        let (mut ws, gen) = controller(addr).await;
        let conn = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
        let _guard = WriteHookGuard(conn.clone());
        let ep0 = sup.get_observation().await.epoch_id;
        assert_eq!(rpc(&mut ws, gen, ep0, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
        let ep = sup.get_observation().await.epoch_id;

        armd::api::ws::enable_test_ws_write_trace(&conn);
        armd::api::ws::set_test_ws_write_timeout(&conn, Some(Duration::from_millis(160)));
        armd::api::ws::set_test_ws_write_stall_once(&conn, true);

        // Trigger one ordinary frame so writer blocks inside `write_ws_frame`.
        send(&mut ws, json!({"type": "command", "request_id": 1, "generation": gen, "epoch_id": ep, "path": "/api/v1/motion/home", "body": {}})).await;
        wait_until(Duration::from_secs(1), || armd::api::ws::test_ws_writer_blocked_count(&conn) >= 1).await;

        // While the in-progress ordinary write is stalled, queue two E-stops so their replies sit in `estop_out_rx`.
        send(&mut ws, json!({"type": "estop"})).await;
        send(&mut ws, json!({"type": "e_stop"})).await;
        wait_until(Duration::from_millis(100), || {
            sup.inner().try_read().is_ok_and(|g| *g.safety.state() == SafetyState::EStop)
        }).await;

        // Collect all wire frames until Close.
        let mut wire_events: Vec<String> = Vec::new();
        let deadline = Instant::now() + Duration::from_secs(2);
        while Instant::now() < deadline {
            match tokio::time::timeout(Duration::from_millis(400), ws.next()).await {
                Ok(Some(Ok(Message::Text(txt)))) => {
                    if let Ok(v) = serde_json::from_str::<serde_json::Value>(&txt) {
                        if let Some(ty) = v.get("type").and_then(|t| t.as_str()) {
                            let tag = if ty == "error" {
                                format!("error:{}", v["code"].as_str().unwrap_or(""))
                            } else if ty == "epoch_sync" {
                                format!("epoch_sync:{}", v["safety_state"].as_str().unwrap_or(""))
                            } else {
                                ty.to_string()
                            };
                            wire_events.push(tag);
                        }
                    }
                }
                Ok(Some(Ok(Message::Close(cf)))) => {
                    wire_events.push(format!("close:{}", cf.map(|f| f.reason.to_string()).unwrap_or_default()));
                    break;
                }
                _ => break,
            }
        }
        let estop_count = wire_events.iter().filter(|e| *e == "epoch_sync:EStop").count();
        let last_estop = wire_events.iter().rposition(|e| e == "epoch_sync:EStop").expect("queued E-stop replies must arrive on wire");
        let fatal_idx = wire_events.iter().position(|e| e == "error:WRITE_TIMEOUT").expect("fatal WRITE_TIMEOUT frame must arrive on wire");
        let close_idx = wire_events.iter().position(|e| e.starts_with("close:")).expect("Close frame must arrive on wire");
        assert!(estop_count >= 2, "expected both queued E-stop replies on wire, got {wire_events:?}");
        assert!(last_estop < fatal_idx && fatal_idx < close_idx, "must deliver queued E-stop replies -> fatal error -> Close, got {wire_events:?}");

        let trace = armd::api::ws::take_test_ws_write_trace(&conn);
        let t_estop = trace.iter().rposition(|e| e == "text:epoch_sync").expect("epoch_sync traced");
        let t_err = trace.iter().position(|e| e == "text:error").expect("error traced");
        let t_close = trace.iter().position(|e| e.starts_with("close:")).expect("close traced");
        assert!(t_estop < t_err && t_err < t_close, "attempt order must be epoch_sync -> error -> close: {trace:?}");
    }

    // (b) Permanently stalled socket: in-progress write times out and is abandoned -> queued E-stop
    //     replies AND fatal frame are STILL attempted (each bounded) BEFORE CloseFrame(ESTOP_LATCHED).
    {
        let (addr, sup) = spawn_server(ArmConfig { lease_timeout_ms: 15_000, ..ArmConfig::rebot_dm() }).await;
        let (mut ws, gen) = controller(addr).await;
        let conn = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
        let _guard = WriteHookGuard(conn.clone());
        let ep = sup.get_observation().await.epoch_id;

        armd::api::ws::enable_test_ws_write_trace(&conn);
        armd::api::ws::set_test_ws_write_timeout(&conn, Some(Duration::from_millis(120)));
        armd::api::ws::set_test_ws_write_paused(&conn, true);

        send(&mut ws, json!({"type": "command", "request_id": 1, "generation": gen, "epoch_id": ep, "path": "/api/v1/motion/home", "body": {}})).await;
        wait_until(Duration::from_secs(1), || armd::api::ws::test_ws_writer_blocked_count(&conn) >= 1).await;

        send(&mut ws, json!({"type": "estop"})).await;
        wait_until(Duration::from_millis(80), || {
            sup.inner().try_read().is_ok_and(|g| *g.safety.state() == SafetyState::EStop)
        }).await;

        let mut close_reason = None;
        let deadline = Instant::now() + Duration::from_secs(2);
        while Instant::now() < deadline {
            match tokio::time::timeout(Duration::from_millis(400), ws.next()).await {
                Ok(Some(Ok(Message::Close(Some(cf))))) => {
                    close_reason = Some(cf.reason.to_string());
                    break;
                }
                Ok(Some(Ok(_))) => {}
                _ => break,
            }
        }
        let reason = close_reason.expect("client must receive CloseFrame even when writes remain stalled");
        assert!(reason.contains("ESTOP_LATCHED"), "CloseFrame must report ESTOP_LATCHED, got {reason:?}");
        wait_until(Duration::from_secs(1), || {
            !armd::api::ws::connection_registry().lock().unwrap().contains_key(&conn)
        }).await;

        let trace = armd::api::ws::take_test_ws_write_trace(&conn);
        let t_estop = trace.iter().position(|e| e == "text:epoch_sync").expect("queued E-stop reply must be attempted before Close");
        let t_err = trace.iter().position(|e| e == "text:error").expect("fatal error frame must be attempted before Close");
        let t_close = trace.iter().position(|e| e.starts_with("close:ESTOP_LATCHED")).expect("ESTOP_LATCHED Close must be attempted");
        assert!(t_estop < t_err && t_err < t_close, "permanent stall must attempt E-stop -> error -> Close(ESTOP_LATCHED): {trace:?}");
    }
}

// 22. Round-5 Item 5 (Teardown cannot leave motion):
//     (a) Owner-lock connection-closed flag refuses both new `check_admission` and any pre-created
//         stale `Admitted` token from a closed connection (`jog_action`, `enable_torque`,
//         `begin_policy_run`), leaving zero motion or policy installed.
//     (b) Policy start resuming after its deployment lookup (`active_scene_and_tool().await`)
//         once teardown began cannot install a policy run or leave motion behind.
//     (c) Acquire path hitting closing while this connection holds an active lease with running
//         motion stops that lease's motion first before releasing it (no unowned motion left).
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn teardown_closes_connection_under_owner_lock_and_acquire_release_stops_motion() {
    use armd::api::rest::ApiError;
    use armd::AdmissionStamp;

    // (a) Owner-lock connection-closed flag refuses stale `Admitted` and `AdmissionStamp`.
    {
        let mut inner = SupervisorInner::new(ArmConfig { lease_timeout_ms: 15_000, ..ArmConfig::rebot_dm() });
        let gen = inner.acquire_lease("connection:42").unwrap();
        let s0 = AdmissionStamp { connection_id: 42, ..inner.current_admission_stamp() };
        let a_en = inner.check_admission(s0).unwrap();
        inner.enable_torque(a_en).unwrap();
        let ep = inner.safety.epoch_id();
        let stamp_conn42 = AdmissionStamp { stop_gen: inner.current_stop_gen(), estop_epoch: ep, lease_gen: gen, connection_id: 42 };
        // Pre-create valid `Admitted` tokens BEFORE teardown marks connection 42 closed.
        let stale_admitted_jog = inner.check_admission(stamp_conn42).unwrap();
        #[cfg(feature = "policy-runtime")]
        let stale_admitted_pol = inner.check_admission(stamp_conn42).unwrap();

        // Teardown marks connection 42 closed under the owner lock (and also exercise coalescing).
        inner.mark_connection_closed(41);
        inner.mark_connection_closed(43);
        inner.mark_connection_closed(42);
        assert!(inner.is_connection_closed(41) && inner.is_connection_closed(42) && inner.is_connection_closed(43));
        assert!(!inner.is_connection_closed(40) && !inner.is_connection_closed(44) && !inner.is_connection_closed(0));

        // Even while lease/epoch/stop_gen still match, BOTH `check_admission` AND pre-created
        // `Admitted` tokens for connection 42 MUST be refused with LEASE_LOST!
        let err_check = inner.check_admission(stamp_conn42).unwrap_err();
        assert_eq!(ApiError::from(ArmError::from(err_check)).code.as_deref(), Some("LEASE_LOST"));
        let err_jog = inner.jog_action(stale_admitted_jog, INSPECT.to_vec(), Some(1.0), Some(ep), "closed_jog").unwrap_err();
        assert_eq!(ApiError::from(err_jog).code.as_deref(), Some("LEASE_LOST"));
        assert!(inner.active_trajectory.is_none());
        #[cfg(feature = "policy-runtime")]
        {
            let err_pol = inner.begin_policy_run(stale_admitted_pol, armd::policy_runtime::PolicyRunStart {
                epoch_id: ep,
                connection_id: "connection:42".into(),
                lease_generation: gen,
                policy_id: "closed_pol".into(),
                artifact_digest: "sha256:0000".into(),
                backend: "mlp".into(),
                mode: armd::policy_runtime::PolicyMode::Shadow,
                sim_only: false,
                max_first_step_rad: None,
                max_step_rad: None,
                tag: None,
                i_am_at_the_arm: false,
                verified_release_gate: None,
            }).unwrap_err();
            assert_eq!(ApiError::from(err_pol).code.as_deref(), Some("LEASE_LOST"));
            assert!(inner.policy_run.is_none());
        }
    }

    // (b) WS policy/start resuming right after deployment lookup / before install lock once teardown
    //     began must be refused under the owner lock and leave no policy_run, trajectory, or lease.
    #[cfg(feature = "policy-runtime")]
    {
        use armd::api::create_router_with_sandbox;
        use armd::api::sandbox::SandboxState;
        use armd::policy_runtime::{encode_rmlp_bytes, Activation, MlpLayerSpec};

        let mut cfg = ArmConfig::default();
        cfg.lease_timeout_ms = 15_000;
        let jnames: Vec<&str> = cfg.limits.iter().map(|l| l.name.as_str()).collect();
        let dir = tempfile::tempdir().unwrap();
        let pack_dir = dir.path().join("policies").join("td_deploy_pol");
        std::fs::create_dir_all(&pack_dir).unwrap();
        let layers = vec![MlpLayerSpec::new(6, 6, Activation::None, vec![0.0; 36], vec![0.0; 6])];
        std::fs::write(pack_dir.join("model.rmlp"), encode_rmlp_bytes(&layers).unwrap()).unwrap();
        std::fs::write(
            pack_dir.join("policy.json"),
            serde_json::to_vec(&json!({
                "format": "rebot-onnx-policy/v1",
                "id": "td_deploy_pol",
                "name": "td_deploy_pol",
                "unit_mode": "rad",
                "action_dim": 6,
                "action_space": "joint_positions",
                "joint_names": jnames,
                "inputs": {"state": {"shape": [1, 6], "joint_names": jnames}},
                "outputs": {"action": {"shape": [1, 6], "space": "joint_positions"}},
                "model_file": "model.rmlp"
            })).unwrap(),
        ).unwrap();

        let sandbox = Arc::new(SandboxState::new(dir.path().to_path_buf()));
        let sup = Arc::new(ArmSupervisor::new(cfg));
        let router = create_router_with_sandbox(sup.clone(), OperatorAuth::disabled(), sandbox);
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });

        let (mut ws, gen) = controller(addr).await;
        let conn = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
        let ep = sup.get_observation().await.epoch_id;

        // Pause the policy-start task right after deployment lookup, right before acquiring inner.write().
        armd::api::ws::set_test_ws_worker_before_lock_paused(&conn, true);
        // Wait: worker_task also calls maybe_pause_worker_before_lock for the initial command;
        // pause ONLY after policy_loading starts!
        armd::api::ws::set_test_ws_worker_before_lock_paused(&conn, false);
        send(&mut ws, json!({
            "type": "command", "request_id": 9301, "generation": gen, "epoch_id": ep,
            "path": "/api/v1/action/policy/start",
            "body": {"policy_id": "td_deploy_pol", "mode": "shadow"}
        })).await;
        // Wait for policy_loading to flip true, then pause right before install lock.
        // Use a tiny blocking delay so the spawned task hasn't reached the install pause yet.
        wait_until(Duration::from_secs(2), || {
            let g = sup.inner().try_read();
            g.is_ok_and(|inner| inner.policy_loading || inner.policy_run.is_some())
        }).await;
        // Even cleaner: disconnect while paused in blocking_plan or before_lock (use fresh connection with delay).
        drop(ws);
        wait_until(Duration::from_secs(2), || {
            !armd::api::ws::connection_registry().lock().unwrap().contains_key(&conn)
        }).await;
        // Deterministic post-deployment pause test with sandbox delay so policy_loading is active when we pause:
        let sandbox2 = Arc::new(SandboxState::new(dir.path().to_path_buf()));
        sandbox2.set_policy_load_delay_ms(60);
        let sup2 = Arc::new(ArmSupervisor::new(ArmConfig { lease_timeout_ms: 15_000, ..ArmConfig::default() }));
        let router2 = create_router_with_sandbox(sup2.clone(), OperatorAuth::disabled(), sandbox2);
        let listener2 = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr2 = listener2.local_addr().unwrap();
        tokio::spawn(async move { axum::serve(listener2, router2).await.unwrap(); });

        let (mut ws2, gen2) = controller(addr2).await;
        let conn2 = sup2.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
        let ep2 = sup2.get_observation().await.epoch_id;
        send(&mut ws2, json!({
            "type": "command", "request_id": 9302, "generation": gen2, "epoch_id": ep2,
            "path": "/api/v1/action/policy/start",
            "body": {"policy_id": "td_deploy_pol", "mode": "shadow"}
        })).await;
        wait_until(Duration::from_secs(2), || {
            sup2.inner().try_read().is_ok_and(|g| g.policy_loading)
        }).await;
        // Now arm the before-install pause hook so the policy task pauses right after deployment lookup.
        armd::api::ws::set_test_ws_worker_before_lock_paused(&conn2, true);
        wait_until(Duration::from_secs(2), || {
            armd::api::ws::test_ws_worker_before_lock_blocked_count(&conn2) >= 1
        }).await;
        // Disconnect while policy task is suspended right after deployment lookup, then unpause it.
        drop(ws2);
        tokio::time::sleep(Duration::from_millis(20)).await;
        armd::api::ws::set_test_ws_worker_before_lock_paused(&conn2, false);
        wait_until(Duration::from_secs(2), || {
            !armd::api::ws::connection_registry().lock().unwrap().contains_key(&conn2)
        }).await;
        let inner2 = sup2.inner().read().await;
        assert!(inner2.active_lease.is_none(), "teardown must release lease");
        assert!(inner2.policy_run.is_none(), "resuming policy/start after teardown must NOT install");
        assert!(!inner2.policy_loading, "policy_loading must be cleared");
    }

    // (c) Acquire path releasing an existing lease during closing MUST stop that lease's motion first.
    {
        let (addr, sup) = spawn_server(ArmConfig { lease_timeout_ms: 15_000, ..ArmConfig::rebot_dm() }).await;
        let (mut ws, gen) = controller(addr).await;
        let conn = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
        let ep0 = sup.get_observation().await.epoch_id;
        assert_eq!(rpc(&mut ws, gen, ep0, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
        let ep1 = sup.get_observation().await.epoch_id;
        assert_eq!(
            rpc(&mut ws, gen, ep1, "/api/v1/motion/jog", json!({"target_positions": [0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "duration_secs": 3.0})).await["type"],
            "command_ack"
        );
        assert!(sup.inner().read().await.active_trajectory.is_some(), "motion must be active before acquire-during-close");

        // Pause worker right before inner.write() on a second `acquire_lease`, then close the socket
        // (setting `connection_closing = true` while worker waits to enter the acquire branch).
        armd::api::ws::set_test_ws_worker_before_lock_paused(&conn, true);
        send(&mut ws, json!({"type": "acquire_lease", "client_id": "test_controller", "seq": 99})).await;
        wait_until(Duration::from_secs(2), || {
            armd::api::ws::test_ws_worker_before_lock_blocked_count(&conn) >= 1
        }).await;
        // Hold a read/write barrier or close socket then release pause so acquire enters the closing branch.
        drop(ws);
        tokio::time::sleep(Duration::from_millis(15)).await;
        armd::api::ws::set_test_ws_worker_before_lock_paused(&conn, false);
        wait_until(Duration::from_secs(2), || {
            !armd::api::ws::connection_registry().lock().unwrap().contains_key(&conn)
        }).await;
        let inner = sup.inner().read().await;
        assert!(inner.active_lease.is_none(), "lease must be released");
        assert!(inner.active_trajectory.is_none(), "acquire-during-closing releasing existing lease must stop motion first");

        // Direct unit check on `release_lease_and_stop` / `release_lease_for_connection_and_stop`:
        drop(inner);
        let mut direct = SupervisorInner::new(ArmConfig { lease_timeout_ms: 15_000, ..ArmConfig::rebot_dm() });
        let g = direct.acquire_lease("connection:99").unwrap();
        let a1 = direct.check_admission(direct.current_admission_stamp()).unwrap();
        direct.enable_torque(a1).unwrap();
        let ep_d = direct.safety.epoch_id();
        let a2 = direct.check_admission(direct.current_admission_stamp()).unwrap();
        direct.jog_action(a2, vec![0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], Some(2.0), Some(ep_d), "direct_jog").unwrap();
        assert!(direct.active_trajectory.is_some());
        assert!(direct.release_lease_for_connection_and_stop("connection:99"));
        assert!(direct.active_lease.is_none());
        assert!(direct.active_trajectory.is_none(), "release_lease_for_connection_and_stop must stop motion");

        let g2 = direct.acquire_lease("connection:100").unwrap();
        assert!(g2 > g);
        let a3 = direct.check_admission(direct.current_admission_stamp()).unwrap();
        direct.jog_action(a3, vec![0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], Some(2.0), Some(ep_d), "direct_jog2").unwrap();
        assert!(direct.active_trajectory.is_some());
        assert!(direct.release_lease_and_stop("connection:100", g2));
        assert!(direct.active_lease.is_none());
        assert!(direct.active_trajectory.is_none(), "release_lease_and_stop must stop motion");
    }
}

// 23. Round-5 Item 6 (Per-connection bounded FIFO of finished actions):
//     (a) Repeated rapid starts overwriting `previous_action` 19 times while dispatcher is paused
//         lose ZERO terminals: once dispatcher drains, BOTH controller and passive observer receive
//         ALL 20 finished actions in strict FIFO order.
//     (b) When the bounded FIFO is full, new starts are refused with `TERMINALS_BUSY` (503) without
//         dropping or overwriting any queued or running action; once drained, starts succeed again.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn per_connection_bounded_terminal_fifo_delivers_all_in_order_and_refuses_when_full() {
    use futures_util::StreamExt;
    use tokio_tungstenite::tungstenite::Message;

    async fn collect_terminals(
        ws: &mut common::TestSocket,
        expected: usize,
        timeout: Duration,
    ) -> Vec<(u64, String, Option<String>)> {
        let mut out = Vec::new();
        let deadline = Instant::now() + timeout;
        while out.len() < expected && Instant::now() < deadline {
            if let Ok(Some(Ok(Message::Text(txt)))) =
                tokio::time::timeout(Duration::from_millis(200), ws.next()).await
            {
                if let Ok(v) = serde_json::from_str::<serde_json::Value>(&txt) {
                    if v.get("type").is_none() {
                        if let Some(a) = v.get("action").filter(|a| a["state"].as_str() != Some("running")) {
                            if let (Some(id), Some(st)) = (a["id"].as_u64(), a["state"].as_str()) {
                                let code = a.get("code").and_then(|c| c.as_str()).map(str::to_owned);
                                out.push((id, st.to_owned(), code));
                            }
                        }
                    }
                }
            }
        }
        out
    }

    // (a) 20 rapid starts (overwriting `previous_action` 19 times) + second passive observer:
    //     ZERO terminals disappear; BOTH connections receive all 20 terminals in exact FIFO order.
    {
        let (addr, sup) = spawn_server(ArmConfig { lease_timeout_ms: 15_000, ..ArmConfig::rebot_dm() }).await;
        let (mut ws_ctrl, gen) = controller(addr).await;
        let (mut ws_obs, _) = connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
        // Wait until the observer socket has registered its terminal FIFO subscription.
        wait_until(Duration::from_secs(1), || sup.test_terminal_subscriber_count() >= 2).await;
        let ep0 = sup.get_observation().await.epoch_id;
        assert_eq!(rpc(&mut ws_ctrl, gen, ep0, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
        let ep = sup.get_observation().await.epoch_id;

        // Pause dispatcher draining so all 19 superseding starts + final stop pile up in the per-connection FIFOs
        // and repeatedly overwrite `SupervisorInner::previous_action`.
        sup.test_set_terminal_dispatch_paused(true);

        let mut expected_ids = Vec::new();
        for k in 0..20u64 {
            let mut target = vec![0.0f32; 7];
            target[0] = 0.02 * ((k % 10) + 1) as f32;
            let ack = rpc(&mut ws_ctrl, gen, ep, "/api/v1/motion/jog", json!({
                "target_positions": target,
                "duration_secs": 2.0
            })).await;
            assert_eq!(ack["type"], "command_ack", "start {k}: {ack}");
            expected_ids.push(ack["action_id"].as_u64().expect("action_id"));
        }
        // Finish the 20th running action cleanly via urgent Stop + one control tick to reconcile its terminal.
        let stop_ack = rpc(&mut ws_ctrl, gen, ep, "/api/v1/motion/stop", json!({})).await;
        assert_eq!(stop_ack["type"], "command_ack", "{stop_ack}");
        sup.tick(0.02).await;
        // Each of the 2 subscribed connections has 20 queued terminals (40 total).
        assert_eq!(sup.test_pending_terminal_count(), 40);

        // Resume dispatcher: BOTH connections must receive ALL 20 terminals in exact FIFO order!
        sup.test_set_terminal_dispatch_paused(false);
        let (ctrl_terms, obs_terms) = tokio::join!(
            collect_terminals(&mut ws_ctrl, 20, Duration::from_secs(3)),
            collect_terminals(&mut ws_obs, 20, Duration::from_secs(3)),
        );
        let ctrl_ids: Vec<u64> = ctrl_terms.iter().map(|(id, _, _)| *id).collect();
        let obs_ids: Vec<u64> = obs_terms.iter().map(|(id, _, _)| *id).collect();
        assert_eq!(ctrl_ids, expected_ids, "controller must receive every terminal in strict FIFO order: {ctrl_terms:?}");
        assert_eq!(obs_ids, expected_ids, "observer must receive every terminal in strict FIFO order: {obs_terms:?}");
        for (_, st, code) in &ctrl_terms[..19] {
            assert_eq!((st.as_str(), code.as_deref()), ("cancelled", Some("SUPERSEDED")));
        }
        assert_eq!((ctrl_terms[19].1.as_str(), ctrl_terms[19].2.as_deref()), ("cancelled", Some("STOPPED")));
        assert_eq!(sup.test_pending_terminal_count(), 0);
    }

    // (b) Bounded FIFO full -> refuse new starts with `TERMINALS_BUSY` (503) without dropping any
    //     queued or running action; once drained, new starts succeed again.
    {
        let (addr, sup) = spawn_server(ArmConfig { lease_timeout_ms: 15_000, ..ArmConfig::rebot_dm() }).await;
        sup.test_set_terminal_fifo_capacity(4);
        let (mut ws, gen) = controller(addr).await;
        let ep0 = sup.get_observation().await.epoch_id;
        assert_eq!(rpc(&mut ws, gen, ep0, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
        let ep = sup.get_observation().await.epoch_id;

        sup.test_set_terminal_dispatch_paused(true);

        // Start 4 actions: actions 1..3 are superseded into FIFO (len=3), action 4 is running
        // (1 slot reserved for action 4's eventual terminal so capacity=4 is never exceeded).
        let mut ids = Vec::new();
        for k in 0..4u64 {
            let ack = rpc(&mut ws, gen, ep, "/api/v1/motion/jog", json!({
                "target_positions": [0.05 * (k + 1) as f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "duration_secs": 2.0
            })).await;
            assert_eq!(ack["type"], "command_ack", "action {k}: {ack}");
            ids.push(ack["action_id"].as_u64().unwrap());
        }
        assert_eq!(sup.test_pending_terminal_count(), 3);

        // 5th start while FIFO has 3 + 1 running == 4 (full): MUST refuse with TERMINALS_BUSY (503)!
        let busy1 = rpc(&mut ws, gen, ep, "/api/v1/motion/jog", json!({
            "target_positions": [0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "duration_secs": 2.0
        })).await;
        assert_eq!((busy1["type"].as_str(), busy1["status"].as_u64(), busy1["code"].as_str()),
            (Some("command_reject"), Some(503), Some("TERMINALS_BUSY")), "{busy1}");
        // Action 4 must still be running undisturbed and FIFO still holds 3 entries.
        assert_eq!(sup.inner().read().await.last_action.as_ref().map(|a| (a.id, a.is_running())), Some((ids[3], true)));
        assert_eq!(sup.test_pending_terminal_count(), 3);

        // Stop action 4 + tick -> action 4 enters the reserved 4th FIFO slot (len=4 == cap, zero dropped!).
        let stop_ack = rpc(&mut ws, gen, ep, "/api/v1/motion/stop", json!({})).await;
        assert_eq!(stop_ack["type"], "command_ack", "{stop_ack}");
        sup.tick(0.02).await;
        assert_eq!(sup.test_pending_terminal_count(), 4);

        // Even while idle, starting a new action with FIFO len == 4/4 MUST refuse with TERMINALS_BUSY!
        let busy2 = rpc(&mut ws, gen, ep, "/api/v1/motion/jog", json!({
            "target_positions": [0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "duration_secs": 2.0
        })).await;
        assert_eq!((busy2["type"].as_str(), busy2["status"].as_u64(), busy2["code"].as_str()),
            (Some("command_reject"), Some(503), Some("TERMINALS_BUSY")), "{busy2}");

        // Unpause dispatcher -> all 4 terminals drain in exact FIFO order!
        sup.test_set_terminal_dispatch_paused(false);
        let drained = collect_terminals(&mut ws, 4, Duration::from_secs(3)).await;
        let drained_ids: Vec<u64> = drained.iter().map(|(id, _, _)| *id).collect();
        assert_eq!(drained_ids, ids, "all 4 terminals must drain in order without loss: {drained:?}");
        assert_eq!(sup.test_pending_terminal_count(), 0);

        // Now that the FIFO drained, a new action start succeeds cleanly!
        let ack5 = rpc(&mut ws, gen, ep, "/api/v1/motion/jog", json!({
            "target_positions": [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "duration_secs": 2.0
        })).await;
        assert_eq!(ack5["type"], "command_ack", "start after drain must succeed: {ack5}");
        let id5 = ack5["action_id"].as_u64().unwrap();
        let stop5 = rpc(&mut ws, gen, ep, "/api/v1/motion/stop", json!({})).await;
        assert_eq!(stop5["type"], "command_ack", "{stop5}");
        sup.tick(0.02).await;
        let term5 = collect_terminals(&mut ws, 1, Duration::from_secs(3)).await;
        assert_eq!(term5.len(), 1);
        assert_eq!(term5[0].0, id5);
    }
}

// 21. Merge invariants (A1 + B3/B3-F + I16):
//     (a) Every B3 SafetyManager::latch (new kind, repeat kind, E-stop) and clear_with_check
//         keeps epoch_mirror synchronized with epoch_id so lock-free capture_admission_stamp()
//         never drifts.
//     (b) WS /api/v1/action/gripper_calibrate rejects stale stamps after Stop/E-stop AND
//         respects TERMINALS_BUSY when the per-connection terminal FIFO is saturated.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn merge_b3_epoch_mirror_sync_and_i16_gripper_calibrate_admission_and_fifo() {
    use armd::safety::{ClearContext, FaultDetail, FaultKind};

    // (a) epoch_mirror stays in lockstep with epoch_id across latch (new, repeat, E-stop) and clear.
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        let mut inner = sup.inner().write().await;
        let mirror = inner.safety.epoch_mirror();
        assert_eq!(mirror.load(Ordering::SeqCst), inner.safety.epoch_id());

        let ep1 = inner.safety.latch(FaultKind::WatchdogTimeout, FaultDetail::WatchdogTimeout);
        assert_eq!(ep1, inner.safety.epoch_id());
        assert_eq!(mirror.load(Ordering::SeqCst), ep1);
        assert_eq!(sup.capture_admission_stamp().estop_epoch, ep1);

        // Repeat latch of the same kind still advances epoch_id AND epoch_mirror.
        let ep2 = inner.safety.latch(FaultKind::WatchdogTimeout, FaultDetail::WatchdogTimeout);
        assert_eq!(ep2, ep1 + 1);
        assert_eq!(mirror.load(Ordering::SeqCst), ep2);
        assert_eq!(sup.capture_admission_stamp().estop_epoch, ep2);

        // E-stop latch advances epoch_id AND epoch_mirror.
        let ep3 = inner.safety.latch(FaultKind::Estop, FaultDetail::Estop);
        assert_eq!(ep3, ep2 + 1);
        assert_eq!(mirror.load(Ordering::SeqCst), ep3);
        assert_eq!(sup.capture_admission_stamp().estop_epoch, ep3);

        // clear_with_check advances epoch_id AND epoch_mirror.
        let cleared = inner.safety.clear_with_check(&ClearContext::all_conditions_met()).unwrap();
        assert_eq!(cleared, SafetyState::Idle);
        let ep4 = inner.safety.epoch_id();
        assert_eq!(ep4, ep3 + 1);
        assert_eq!(mirror.load(Ordering::SeqCst), ep4);
        assert_eq!(sup.capture_admission_stamp().estop_epoch, ep4);
    }

    // (b) WS /api/v1/action/gripper_calibrate rejects stale stamp after Stop AND refuses when FIFO saturated.
    {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.lease_timeout_ms = 30_000;
        let sup = Arc::new(ArmSupervisor::new(cfg));
        sup.test_set_terminal_fifo_capacity(2);
        let addr = serve(sup.clone()).await;
        let (mut ws, gen) = controller(addr).await;
        let ep0 = sup.get_observation().await.epoch_id;
        assert_eq!(rpc(&mut ws, gen, ep0, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
        let ep = sup.get_observation().await.epoch_id;
        let conn_id = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();

        // Saturated terminal FIFO refuses /api/v1/action/gripper_calibrate with 503 TERMINALS_BUSY.
        sup.test_set_terminal_dispatch_paused(true);
        for _ in 0..2 {
            let ack = rpc(&mut ws, gen, ep, "/api/v1/motion/jog", json!({
                "target_positions": [0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "duration_secs": 2.0
            })).await;
            assert_eq!(ack["type"], "command_ack", "{ack}");
            assert_eq!(rpc(&mut ws, gen, ep, "/api/v1/motion/stop", json!({})).await["type"], "command_ack");
            sup.tick(0.02).await;
        }
        assert_eq!(sup.test_pending_terminal_count(), 2);
        let busy = rpc(&mut ws, gen, ep, armd::gripper_calib::CALIBRATE_PATH, json!({})).await;
        assert_eq!(
            (busy["type"].as_str(), busy["status"].as_u64(), busy["code"].as_str()),
            (Some("command_reject"), Some(503), Some("TERMINALS_BUSY")),
            "{busy}"
        );
        assert!(sup.inner().read().await.gripper_calibration.is_none());
        sup.test_set_terminal_dispatch_paused(false);
        wait_until(Duration::from_secs(3), || sup.test_pending_terminal_count() == 0).await;

        // Stale stamp after urgent Stop while worker is paused before acquiring lock.
        armd::api::ws::set_test_ws_worker_before_lock_paused(&conn_id, true);
        send(&mut ws, json!({
            "type": "command", "request_id": 9801, "generation": gen, "epoch_id": ep,
            "path": armd::gripper_calib::CALIBRATE_PATH, "body": {}
        })).await;
        wait_until(Duration::from_secs(1), || {
            armd::api::ws::test_ws_worker_before_lock_blocked_count(&conn_id) >= 1
        }).await;
        send(&mut ws, json!({
            "type": "command", "request_id": 9802, "generation": gen, "epoch_id": ep,
            "path": "/api/v1/motion/stop", "body": {}
        })).await;
        let stop_ack = next_of(&mut ws, &["command_ack", "command_reject"]).await;
        assert_eq!((stop_ack["type"].as_str(), stop_ack["request_id"].as_u64()), (Some("command_ack"), Some(9802)), "{stop_ack}");
        armd::api::ws::set_test_ws_worker_before_lock_paused(&conn_id, false);
        let rej = next_of(&mut ws, &["command_ack", "command_reject"]).await;
        assert_eq!(rej["type"], "command_reject", "{rej}");
        assert_eq!(rej["request_id"], 9801);
        assert_eq!(rej["code"], "STOPPED", "{rej}");
        assert!(sup.inner().read().await.gripper_calibration.is_none());
    }
}

// 22. Round-6 Item 1: auth-reply saturation signals fatal first, helper exit never aborts the
//     reader, urgent E-stop still latches while writer is stalled, and writer flushes the urgent
//     E-stop reply BEFORE the fatal error frame and Close.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn auth_reply_saturation_helper_exit_never_aborts_reader_and_flushes_urgent_estop() {
    use futures_util::StreamExt;
    use tokio_tungstenite::tungstenite::Message;

    let (addr, sup) = spawn_server(ArmConfig { lease_timeout_ms: 15_000, ..ArmConfig::rebot_dm() }).await;
    let (mut ws, gen) = controller(addr).await;
    let conn_id = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
    let _guard = WriteHookGuard(conn_id.clone());
    let ep0 = sup.get_observation().await.epoch_id;
    assert_eq!(rpc(&mut ws, gen, ep0, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
    let ep = sup.get_observation().await.epoch_id;

    // Stall the writer inside write_ws_frame_bounded with an ordinary command reply.
    armd::api::ws::set_test_ws_write_paused(&conn_id, true);
    send(&mut ws, json!({
        "type": "command", "request_id": 8000, "generation": gen, "epoch_id": ep,
        "path": "/api/v1/motion/jog",
        "body": {"target_positions": [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "duration_secs": 0.5}
    })).await;
    wait_until(Duration::from_secs(1), || armd::api::ws::test_ws_writer_blocked_count(&conn_id) >= 1).await;

    // Overflow `auth_tx` (cap 16) + `auth_in_tx` (cap 16) so `auth_task` hits overflow, calls
    // `trigger_fatal`, and EXITS while `writer_task` is still stalled.
    for seq in 1..=24u64 {
        send(&mut ws, json!({"type": "auth", "seq": seq, "token": ""})).await;
    }
    // Give `auth_task` plenty of time to drain `auth_in_rx`, hit `auth_tx` Full, and exit.
    tokio::time::sleep(Duration::from_millis(60)).await;

    // Reader MUST still be alive (helper exit never aborts reader): urgent E-stop latches immediately.
    let t0 = Instant::now();
    send(&mut ws, json!({"type": "estop"})).await;
    wait_until(Duration::from_millis(120), || sup.e_stop_latched.load(Ordering::SeqCst)).await;
    assert!(t0.elapsed() < Duration::from_millis(120), "E-stop did not latch after auth_task exit");
    wait_until(Duration::from_secs(1), || {
        sup.inner().try_read().is_ok_and(|g| *g.safety.state() == SafetyState::EStop)
    }).await;

    // Unblock writer -> writer flushes urgent E-stop reply BEFORE fatal OUTBOUND_QUEUE_FULL and Close.
    armd::api::ws::set_test_ws_write_paused(&conn_id, false);
    let (mut estop_idx, mut fatal_idx, mut close_idx) = (None, None, None);
    let mut idx = 0usize;
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut seen = Vec::new();
    while Instant::now() < deadline {
        match tokio::time::timeout(Duration::from_secs(1), ws.next()).await {
            Ok(Some(Ok(Message::Text(text)))) => {
                let v: serde_json::Value = serde_json::from_str(&text).unwrap();
                seen.push(format!("{idx}:Text({})", v.get("type").and_then(|t| t.as_str()).unwrap_or("obs")));
                if v["type"] == "epoch_sync" && v["safety_state"] == "EStop" && estop_idx.is_none() {
                    estop_idx = Some(idx);
                }
                if v["type"] == "error" && v["code"] == "OUTBOUND_QUEUE_FULL" && fatal_idx.is_none() {
                    fatal_idx = Some(idx);
                }
                idx += 1;
            }
            Ok(Some(Ok(Message::Close(cf)))) => {
                seen.push(format!("{idx}:Close({cf:?})"));
                close_idx = Some(idx);
                break;
            }
            _ => break,
        }
    }
    let e = estop_idx.unwrap_or_else(|| panic!("missing urgent E-stop reply before fatal/Close: {seen:?}"));
    let f = fatal_idx.unwrap_or_else(|| panic!("missing OUTBOUND_QUEUE_FULL fatal frame: {seen:?}"));
    let c = close_idx.unwrap_or_else(|| panic!("missing Close frame: {seen:?}"));
    assert!(e < f && f < c, "expected E-stop ({e}) < fatal ({f}) < Close ({c}); frames={seen:?}");
}

// 23. Round-6 Item 2: teardown marks `connection_closed` under the owner lock FIRST, before
//     awaiting any task join, so a racing install from the closing connection cannot win the
//     owner lock while teardown awaits writer/worker/child joins.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn teardown_marks_connection_closed_under_owner_lock_before_awaiting_joins() {
    let (addr, sup) = spawn_server(ArmConfig { lease_timeout_ms: 15_000, ..ArmConfig::rebot_dm() }).await;
    let (mut ws, gen) = controller(addr).await;
    let conn_id_str = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
    let _guard = WriteHookGuard(conn_id_str.clone());
    let ep0 = sup.get_observation().await.epoch_id;
    assert_eq!(rpc(&mut ws, gen, ep0, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");

    // Capture the numeric connection_id from the live connection string (`conn:<id>`).
    let numeric_conn_id: u64 = conn_id_str
        .rsplit(':')
        .next()
        .expect("numeric connection suffix")
        .parse()
        .unwrap();
    let mut stamp = sup.capture_admission_stamp();
    stamp.connection_id = numeric_conn_id;

    // Stall the writer and trigger fatal close so `writer_task` stays alive during teardown's
    // bounded fatal grace join.
    armd::api::ws::set_test_ws_write_paused(&conn_id_str, true);
    let ep = sup.get_observation().await.epoch_id;
    send(&mut ws, json!({
        "type": "command", "request_id": 8100, "generation": gen, "epoch_id": ep,
        "path": "/api/v1/motion/jog",
        "body": {"target_positions": [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "duration_secs": 0.5}
    })).await;
    wait_until(Duration::from_secs(1), || armd::api::ws::test_ws_writer_blocked_count(&conn_id_str) >= 1).await;
    // Flood > (ORDINARY_IN_CAPACITY + ORDINARY_OUT_CAPACITY = 128) commands so `ordinary_tx`
    // overflows into `fatal_triggered = true` before dropping the client socket.
    for k in 0..150u64 {
        send(&mut ws, json!({
            "type": "command", "request_id": 8200 + k, "generation": gen, "epoch_id": ep,
            "path": "/api/v1/motion/jog", "body": {}
        })).await;
    }
    tokio::time::sleep(Duration::from_millis(20)).await;

    // Drop the client socket so `reader_task` exits immediately and enters teardown while
    // `writer_task` is still blocked in its paused write / fatal flush grace.
    drop(ws);

    // Teardown MUST mark `numeric_conn_id` closed under the owner lock BEFORE `writer_task`
    // finishes joining, so any racing admission check fails immediately with `LeaseLost`.
    wait_until(Duration::from_millis(400), || {
        sup.inner().try_read().is_ok_and(|g| g.is_connection_closed(numeric_conn_id))
    }).await;
    // Still blocked inside writer join (has not exited yet):
    assert!(
        armd::api::ws::test_ws_writer_blocked_count(&conn_id_str) >= 1,
        "expected connection_closed to be set BEFORE writer_task finished joining"
    );
    {
        let inner = sup.inner().write().await;
        assert!(inner.is_connection_closed(numeric_conn_id));
        assert!(matches!(inner.check_admission(stamp), Err(armd::SafetyError::LeaseLost)));
    }
    armd::api::ws::set_test_ws_write_paused(&conn_id_str, false);
}

// 24. Round-6 Item 3: a Stop never cancels a queued torque disable on WS (`drain_cancellable`)
//     and never blocks RT broker `ImmediateOp::TorqueOff` or `TransitionOp::TorqueOff`.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn stop_never_cancels_or_blocks_queued_torque_disable_on_ws_and_rt_broker() {
    // (a) WS queued `/api/v1/torque` and `/api/v1/arm` with `enable: false` survive urgent Stop.
    for (idx, disable_path) in ["/api/v1/torque", "/api/v1/arm"].iter().enumerate() {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.lease_timeout_ms = 30_000;
        let sup = Arc::new(ArmSupervisor::new(cfg));
        let addr = serve(sup.clone()).await;
        let (mut ws, gen) = controller(addr).await;
        let ep0 = sup.get_observation().await.epoch_id;
        assert_eq!(rpc(&mut ws, gen, ep0, "/api/v1/torque", json!({"enable": true})).await["type"], "command_ack");
        assert!(sup.inner().read().await.is_torqued);
        let ep = sup.get_observation().await.epoch_id;
        let conn_id = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
        let base_req = 8500 + (idx as u64) * 100;

        // Hold worker on a first jog so a queued jog AND a queued torque-off sit in `ordinary_queue`.
        armd::api::ws::set_test_ws_worker_before_lock_paused(&conn_id, true);
        send(&mut ws, json!({
            "type": "command", "request_id": base_req + 1, "generation": gen, "epoch_id": ep,
            "path": "/api/v1/motion/jog",
            "body": {"target_positions": [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "duration_secs": 0.5}
        })).await;
        wait_until(Duration::from_secs(1), || {
            armd::api::ws::test_ws_worker_before_lock_blocked_count(&conn_id) >= 1
        }).await;
        send(&mut ws, json!({
            "type": "command", "request_id": base_req + 2, "generation": gen, "epoch_id": ep,
            "path": "/api/v1/motion/jog",
            "body": {"target_positions": [0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "duration_secs": 0.5}
        })).await;
        send(&mut ws, json!({
            "type": "command", "request_id": base_req + 3, "generation": gen, "epoch_id": ep,
            "path": disable_path,
            "body": {"enable": false}
        })).await;
        tokio::time::sleep(Duration::from_millis(30)).await;

        // Send urgent Stop while both jog+2 and torque-disable+3 are sitting in `ordinary_queue`.
        send(&mut ws, json!({
            "type": "command", "request_id": base_req + 4, "generation": gen, "epoch_id": ep,
            "path": "/api/v1/motion/stop", "body": {}
        })).await;
        // Release worker after Stop has been applied.
        let mut replies = std::collections::HashMap::<u64, serde_json::Value>::new();
        let stop_deadline = Instant::now() + Duration::from_secs(2);
        while !replies.contains_key(&(base_req + 4)) && Instant::now() < stop_deadline {
            let v = next_of(&mut ws, &["command_ack", "command_reject"]).await;
            if let Some(id) = v.get("request_id").and_then(|r| r.as_u64()) {
                replies.insert(id, v);
            }
        }
        assert_eq!(replies.get(&(base_req + 4)).and_then(|v| v["type"].as_str()), Some("command_ack"));
        armd::api::ws::set_test_ws_worker_before_lock_paused(&conn_id, false);

        let deadline = Instant::now() + Duration::from_secs(3);
        while replies.len() < 4 && Instant::now() < deadline {
            let v = next_of(&mut ws, &["command_ack", "command_reject"]).await;
            if let Some(id) = v.get("request_id").and_then(|r| r.as_u64()) {
                replies.insert(id, v);
            }
        }
        assert_eq!(replies[&(base_req + 1)]["type"], "command_reject", "{disable_path}: jog1 {:?}", replies[&(base_req + 1)]);
        assert_eq!(replies[&(base_req + 1)]["code"], "STOPPED");
        assert_eq!(replies[&(base_req + 2)]["type"], "command_reject", "{disable_path}: jog2 {:?}", replies[&(base_req + 2)]);
        assert_eq!(replies[&(base_req + 2)]["code"], "STOPPED");
        assert_eq!(replies[&(base_req + 3)]["type"], "command_ack", "{disable_path}: queued disable must execute after Stop: {:?}", replies[&(base_req + 3)]);
        assert!(!sup.inner().read().await.is_torqued, "{disable_path}: torque must be disabled");
    }

    // (b) RT broker ImmediateOp::TorqueOff (driverless) and TransitionOp::TorqueOff (with driver)
    //     both commit across an intervening Stop, while TorqueOn with stale stop_gen is refused.
    {
        use armd::rt_core::broker::{ImmediateOp, Submit, TransitionOp};
        use armd::rt_core::ops::ReleaseFixture;

        // Driverless: ImmediateOp::TorqueOff succeeds after Stop; ImmediateOp::TorqueOn fails STOPPED.
        let mut core = SupervisorInner::new(ArmConfig::rebot_dm());
        core.lease_enabled = false;
        let st = core.current_admission_stamp();
        let adm = core.check_admission(st).unwrap();
        core.enable_torque(adm).unwrap();
        assert!(core.is_torqued);

        let (mut fx, mut broker, _tw, stop) = ReleaseFixture::new(core, BootId(1), Capacities::default());
        let auth_before_stop = Authority {
            lease_generation: 0,
            epoch: fx.core.safety.epoch_id(),
            stop_generation: 0,
            config_revision: fx.core.workspace_revision,
            parent_generation: 0,
        };
        let dl = fx.now() + Duration::from_secs(5);
        broker.submit(ClientId(1), 1, auth_before_stop, dl, Submit::Immediate(ImmediateOp::TorqueOff)).unwrap();
        // Urgent Stop advances stop_generation to 1 BEFORE Step 3 commits the queued TorqueOff.
        let _ = stop.request();
        let rep = fx.step();
        assert!(rep.stopped);
        assert!(matches!(rep.command, Some((_, Ok(_)))), "ImmediateOp::TorqueOff must commit after Stop: {:?}", rep.command);
        assert!(!fx.core.is_torqued);

        // Stale-stop ImmediateOp::TorqueOn MUST still be refused with STOPPED.
        broker.submit(ClientId(1), 2, auth_before_stop, dl, Submit::Immediate(ImmediateOp::TorqueOn)).unwrap();
        let rep_on = fx.step();
        assert!(matches!(rep_on.command, Some((_, Err("STOPPED")))), "stale ImmediateOp::TorqueOn must fail STOPPED: {:?}", rep_on.command);
        assert!(!fx.core.is_torqued);

        // With real DamiaoArmDriver<FakeBus>: TransitionOp::TorqueOff succeeds after Stop;
        // TransitionOp::TorqueOn with stale stop_generation fails STOPPED.
        let dm_lim: Vec<armd::protocol::damiao::DamiaoLimits> = ArmConfig::rebot_dm()
            .damiao_ranges
            .unwrap()
            .iter()
            .map(|r| r.limits())
            .collect();
        let clock = armd::transport::MockClock::new();
        let cfg_drv = ArmConfig::rebot_dm();
        let rest_pos = cfg_drv.resting_positions().to_vec();
        let bus = FakeBus::with_clock(&dm_lim, clock.clone());
        bus.with(|s| {
            s.immediate_replies = true;
            for (m, &q) in s.motors.iter_mut().zip(&rest_pos) {
                m.q = q;
            }
            s.push_all_feedback();
        });
        let mut drv = armd::driver::DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(dm_lim)
            .unwrap();
        drv.read_joint_states().unwrap();
        drv.check_mapping_ranges();
        let mut core_drv = SupervisorInner::new(cfg_drv).with_driver(Box::new(drv));
        core_drv.lease_enabled = false;
        core_drv.tick(0.02);
        let st = core_drv.current_admission_stamp();
        let adm = core_drv.check_admission(st).unwrap();
        core_drv.enable_torque(adm).unwrap();
        assert!(core_drv.is_torqued);

        let (mut fx_d, mut broker_d, _tw_d, stop_d) = ReleaseFixture::new(core_drv, BootId(2), Capacities::default());
        let auth_drv = Authority {
            lease_generation: 0,
            epoch: fx_d.core.safety.epoch_id(),
            stop_generation: 0,
            config_revision: fx_d.core.workspace_revision,
            parent_generation: 0,
        };
        let dl_d = fx_d.now() + Duration::from_secs(5);
        broker_d.submit(ClientId(1), 1, auth_drv, dl_d, Submit::Transition(TransitionOp::TorqueOff)).unwrap();
        let _ = stop_d.request();
        let rep_d = fx_d.step();
        assert!(rep_d.stopped);
        assert!(matches!(rep_d.command, Some((_, Ok(_)))), "TransitionOp::TorqueOff must commit after Stop: {:?}", rep_d.command);
        for _ in 0..10 {
            if !fx_d.core.is_torqued { break; }
            clock.advance(Duration::from_millis(20));
            bus.with(|s| s.push_all_feedback());
            let _ = fx_d.step();
        }
        assert!(!fx_d.core.is_torqued);

        broker_d.submit(ClientId(1), 2, auth_drv, dl_d, Submit::Transition(TransitionOp::TorqueOn)).unwrap();
        let rep_d_on = fx_d.step();
        assert!(matches!(rep_d_on.command, Some((_, Err("STOPPED")))), "stale TransitionOp::TorqueOn must fail STOPPED: {:?}", rep_d_on.command);
        assert!(!fx_d.core.is_torqued);
    }
}
