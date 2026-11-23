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
    // Count every CAN transmit observed once the owner advances `stops.applied() >= 1`.
    let stops_obs = handle.stop_requests().clone();
    let seen_tx_after_apply = Arc::new(AtomicUsize::new(0));
    let tx_cnt = seen_tx_after_apply.clone();
    bus.with(|s| {
        s.on_tx = Some(Arc::new(move || {
            if stops_obs.applied() >= 1 {
                tx_cnt.fetch_add(1, Ordering::Relaxed);
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

    // Inspect EVERY dynamic POS_VEL frame across all 7 joints from the exact first post-apply transmit onward.
    let total_post_apply = seen_tx_after_apply.load(Ordering::Relaxed);
    assert!(total_post_apply >= 7, "expected post-apply hold frames across all 7 joints, got {total_post_apply}");
    bus.with(|s| {
        let start_idx = s.tx_log.len().saturating_sub(total_post_apply).max(tx_before_stop_send);
        let mut per_joint = [0usize; 7];
        for f in &s.tx_log[start_idx..] {
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
