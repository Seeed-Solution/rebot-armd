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
    assert_eq!(sup.clear_fault().await.unwrap(), SafetyState::Idle);
    handle.stop_and_join();
    let fail1 = ws_reply(&mut ws, json!({"type": "estop", "request_id": 77}), &["epoch_sync", "estop_reject"]).await;
    assert_eq!((fail1["type"].as_str(), fail1["status"].as_u64(), fail1["code"].as_str(), fail1["latched"].as_bool(), fail1["request_id"].as_u64()),
        (Some("estop_reject"), Some(503), Some("EXECUTOR_STOPPED"), Some(true), Some(77)), "{fail1}");
    assert!(fail1.get("epoch_id").is_none(), "must not synthesize epoch_id=0: {fail1}");
    let fail2 = ws_reply(&mut ws, json!({"type": "e_stop"}), &["epoch_sync", "estop_reject"]).await;
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
    let sup = Arc::new(ArmSupervisor::new(ArmConfig { lease_timeout_ms: 180, ..ArmConfig::rebot_dm() }));
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

    tokio::time::sleep(Duration::from_millis(240)).await;
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
        c.set_torque(true).unwrap();
        let e = c.safety.epoch_id();
        c.jog_action_with_speed_scale(vec![0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], Some(1.0), Some(e), "joint_move", Some(0.5)).unwrap();
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
        inner.set_torque(true).unwrap();
        let ep = inner.safety.epoch_id();
        inner.jog_action_with_speed_scale(vec![0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], Some(1.0), Some(ep), "joint_move", Some(0.5)).unwrap();
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
        ce.set_torque(true).unwrap();
        let e = ce.safety.epoch_id();
        ce.jog_action_with_speed_scale(vec![0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], Some(1.0), Some(e), "joint_move", Some(0.5)).unwrap();
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
        inner.set_torque(true).unwrap();
        let ep = inner.safety.epoch_id();
        inner.jog_action_with_speed_scale(vec![0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], Some(1.0), Some(ep), "joint_move", Some(0.5)).unwrap();
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
    park_core.set_torque(true).unwrap();
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
        inner.set_torque(true).unwrap();
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
    released: Arc<AtomicBool>,
    release: Arc<tokio::sync::Notify>,
}

impl SlowDetector {
    fn new() -> Arc<Self> {
        Arc::new(Self {
            entered: Arc::new(AtomicBool::new(false)),
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
    sup_l.inner().write().await.jog_action(vec![0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], Some(2.0), Some(ep_l), "legacy_jog").unwrap();
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
    sup.inner().write().await.jog_action(vec![0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], Some(2.0), Some(ep1), "slow_dry_run_jog").unwrap();
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
        inner.jog_action(vec![0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], Some(1.0), Some(ep), "client2_jog").unwrap();
    }
    let r1 = next_of(&mut ws1, &["command_ack", "command_reject"]).await;
    let r2 = next_of(&mut ws1, &["command_ack", "command_reject"]).await;
    assert_eq!((r1["type"].as_str(), r1["code"].as_str()), (Some("command_reject"), Some("LEASE_LOST")), "{r1}");
    assert_eq!((r2["type"].as_str(), r2["code"].as_str()), (Some("command_reject"), Some("LEASE_LOST")), "{r2}");
    assert!(sup.inner().read().await.active_trajectory.is_some(), "preempted owner's Stop must not cancel new owner's motion");

    // (b) Executor path (`ReleaseEngine` + `UrgentStopIntake`): queued Stop from Owner 1 rejected after preemption.
    let mut core = SupervisorInner::new(ArmConfig { lease_timeout_ms: 5_000, ..ArmConfig::rebot_dm() });
    let g1 = core.acquire_lease("connection:owner1").unwrap();
    core.set_torque(true).unwrap();
    let ep_e = core.safety.epoch_id();
    core.jog_action(vec![0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], Some(1.0), Some(ep_e), "owner1_jog").unwrap();
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
    core.jog_action(vec![0.35, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], Some(1.0), Some(ep_e), "owner2_jog").unwrap();
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
