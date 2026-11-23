//! Stage 3 §2 PR A1 acceptance tests:
//! - REST & WS E-stop (`trigger_e_stop_checked`, no synthetic epoch-0) and WS urgent Stop with ordinary queues full
//! - Stale/foreign callers, owner-serialized lease auth, REST lease restrictions, legacy stop semantics & startup gate
//! - Simultaneous REST/WS E-stops with intervening `clear_fault`, and WS E-stop while Stop reply is delayed
//! - No stale dynamic CAN frames after applied stop (`DamiaoArmDriver<FakeBus>`), fresh post-stop motion transmits
//! - Zero-alloc/zero-free RT stop proof (inline + heap-retiring), refusal-only `service_wake`, park preservation, panic-before-publish
mod common;
mod mit_support;

#[global_allocator]
static ALLOC: armd::diagnostics::alloc::TimedAlloc = armd::diagnostics::alloc::TimedAlloc;

use std::net::SocketAddr;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
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
    validate_startup_owner, ExecutorConfig, PanicPolicy, PersistTestHooks, RtExecutorHandle, RtOwner,
};
use armd::rt_core::ids::ArmId;
use armd::rt_core::mailbox::{StreamId, TargetFrame, MAX_DOF};
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

fn fill_ordinary_queues(handle: &RtExecutorHandle, lease_gen: u64, epoch: u64, stop_gen: u64, start: &[f32], q: [f32; MAX_DOF]) {
    static NEXT_SEQ: AtomicU64 = AtomicU64::new(1);
    let auth = Authority { lease_generation: lease_gen, epoch, stop_generation: stop_gen, config_revision: 0, parent_generation: 0 };
    let dl = Instant::now() + Duration::from_secs(5);
    let mut s = [0.0f32; MAX_DOF];
    s[..start.len()].copy_from_slice(start);
    let dof = start.len() as u8;
    let boot = {
        let mut b = handle.broker().lock().unwrap();
        loop {
            let seq = NEXT_SEQ.fetch_add(1, Ordering::Relaxed);
            let res = b.submit(ClientId(9), seq, auth, dl, Submit::Install(&|slot: &mut PlanSlot| {
                slot.set(Primitive::JointQuintic { dof, start: s, q, duration_s: 0.5 });
            }));
            match res {
                Ok(_) => {}
                Err(AdmitError::CommandsFull | AdmitError::PlansFull | AdmitError::RepliesFull) => break,
                Err(e) => panic!("unexpected admit error: {e:?}"),
            }
        }
        b.boot()
    };
    let tseq = NEXT_SEQ.fetch_add(1, Ordering::Relaxed);
    let frame = TargetFrame {
        arm: ArmId::LOCAL, boot, stream: StreamId(1), seq: tseq, authority: auth, policy_generation: 0,
        input_at: Instant::now(), vision_at: None, vision_frame_id: None, deadline: dl, dof, q, replaceable: true,
    };
    let _ = handle.target_writer().lock().unwrap().submit(frame);
}

fn decode_pos_vel_p(f: &armd::transport::CanFrame) -> f32 {
    f32::from_le_bytes([f.data[0], f.data[1], f.data[2], f.data[3]])
}

// 1. REST & WS E-stop + WS urgent Stop when ordinary Broker/Target queues are full, plus waiter failure.
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
    let ecfg = ExecutorConfig { capacities: Capacities { commands: 1, plans: 1, replies: 4 }, ..ExecutorConfig::for_tests(Duration::from_millis(5)) };
    let handle = sup.start_executor_with_hooks(ecfg, PersistTestHooks { release_delay_once: Some(delay.clone()), ..Default::default() }).await.unwrap();
    let http = reqwest::Client::new();

    *delay.lock().unwrap() = Some(Duration::from_millis(45));
    wait_until(Duration::from_secs(1), || delay.lock().unwrap().is_none()).await;
    let obs = sup.observation().await.unwrap();
    fill_ordinary_queues(&handle, gen, obs.epoch_id, 0, &obs.positions, [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    let saturated = handle.broker().lock().unwrap().submit(
        ClientId(9), 99_999,
        Authority { lease_generation: gen, epoch: obs.epoch_id, stop_generation: 0, config_revision: 0, parent_generation: 0 },
        Instant::now() + Duration::from_secs(5),
        Submit::Install(&|slot: &mut PlanSlot| slot.set(Primitive::JointQuintic { dof: 7, start: [0.0; MAX_DOF], q: [0.1; MAX_DOF], duration_s: 0.2 })),
    );
    assert!(matches!(saturated, Err(AdmitError::CommandsFull | AdmitError::PlansFull)), "{saturated:?}");

    // Authorized WS Stop succeeds despite full ordinary queues and refuses older queued command + target as STOPPED.
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

    // REST POST /api/v1/safety/e_stop succeeds with full ordinary queues.
    let ep_pre = sup.observation().await.unwrap().epoch_id;
    *delay.lock().unwrap() = Some(Duration::from_millis(30));
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
        *delay.lock().unwrap() = Some(Duration::from_millis(25));
        wait_until(Duration::from_secs(1), || delay.lock().unwrap().is_none()).await;
        fill_ordinary_queues(&handle, gen, ep_c, sg, &obs.positions, [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        let r = ws_reply(&mut ws, json!({"type": kind}), &["epoch_sync", "estop_reject"]).await;
        assert_eq!((r["type"].as_str(), r["safety_state"].as_str()), (Some("epoch_sync"), Some("EStop")), "{r}");
        assert!(r["epoch_id"].as_u64().unwrap() > ep_c);
    }

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

    // Owner validation precedence: wrong epoch does NOT consume request_id; invalid body DOES consume request_id.
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

// 3. Simultaneous REST+WS E-stops with intervening clear_fault, AND WS E-stop while Stop reply is delayed.
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

    // (a) Simultaneous REST + WS `estop` + WS `e_stop` with intervening `clear_fault` queued in the same wave.
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

    // (b) Same-socket WS E-stop while Stop reply is delayed: E-stop latches immediately without blocking on Stop ACK.
    let ep_live = sup.observation().await.unwrap().epoch_id;
    *delay.lock().unwrap() = Some(Duration::from_millis(120));
    wait_until(Duration::from_secs(1), || delay.lock().unwrap().is_none()).await;
    send(&mut ws1, json!({"type": "command", "request_id": 900, "generation": gen1, "epoch_id": ep_live, "path": "/api/v1/motion/stop", "body": {}})).await;
    tokio::time::sleep(Duration::from_millis(15)).await;
    let t0 = Instant::now();
    send(&mut ws1, json!({"type": "estop"})).await;
    wait_until(Duration::from_millis(45), || sup.e_stop_latched.load(Ordering::SeqCst)).await;
    assert!(t0.elapsed() < Duration::from_millis(80), "E-stop delayed behind Stop: {:?}", t0.elapsed());
    let m1 = next_of(&mut ws1, &["epoch_sync", "command_ack", "command_reject"]).await;
    let m2 = next_of(&mut ws1, &["epoch_sync", "command_ack", "command_reject"]).await;
    let types = [m1["type"].as_str().unwrap(), m2["type"].as_str().unwrap()];
    assert!(types.contains(&"epoch_sync") && (types.contains(&"command_ack") || types.contains(&"command_reject")), "{m1}, {m2}");
    handle.stop_and_join();
}

// 4. No stale dynamic frames after applied stop (`DamiaoArmDriver<FakeBus>`) and fresh post-stop motion transmits.
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
    assert_eq!(rpc(&mut ws, gen, ep1, "/api/v1/motion/jog", json!({"target_positions": [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "duration_secs": 0.8})).await["type"], "command_ack");

    let handle = sup.start_executor(ExecutorConfig::for_tests(Duration::from_millis(5))).await.unwrap();
    wait_until(Duration::from_secs(2), || bus.with(|s| s.motors[0].q > 0.02)).await;
    let obs = sup.observation().await.unwrap();
    fill_ordinary_queues(&handle, gen, obs.epoch_id, 0, &obs.positions, [-0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);

    let ack = rpc(&mut ws, gen, obs.epoch_id, "/api/v1/motion/stop", json!({})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let sg = ack["stop_generation"].as_u64().unwrap();
    assert!(sg >= 1);
    let (q_stopped, tx_after_stop) = bus.with(|s| (s.motors[0].q, s.tx_log.len()));
    let st0 = handle.stats().steps.load(Ordering::Relaxed);
    wait_until(Duration::from_secs(1), || handle.stats().steps.load(Ordering::Relaxed) >= st0 + 6).await;

    // Assert queued older Broker command & TargetWriter frame are refused STOPPED, and every
    // post-stop joint-1 dynamic POS_VEL frame holds `q_stopped` (zero stale frames).
    let (mut saw_cmd_stopped, mut saw_tgt_stopped) = (false, false);
    wait_until(Duration::from_secs(1), || {
        handle.broker().lock().unwrap().poll(|_, r| { if matches!(r, Reply::Refused { code: "STOPPED", .. }) { saw_cmd_stopped = true; } true });
        handle.target_writer().lock().unwrap().outcomes(|o| if o.result.err().is_some_and(|r| r.code == "STOPPED") { saw_tgt_stopped = true; });
        saw_cmd_stopped && saw_tgt_stopped
    }).await;
    let post_stop_j1: Vec<f32> = bus.with(|s| s.tx_log[tx_after_stop..].iter().filter(|f| f.id == 0x101).map(decode_pos_vel_p).collect());
    assert!(!post_stop_j1.is_empty(), "expected hold frames after stop");
    for &p in &post_stop_j1 {
        assert!((p - q_stopped).abs() < 0.02, "stale dynamic frame p={p} diverged from q_stopped={q_stopped}");
    }

    // Fresh post-stop command admitted at `stop_generation == sg` commits and transmits to FakeBus.
    let snap = match handle.call_maintenance_async(armd::rt_core::executor::MaintenanceOp::PlanningSnapshot).await.unwrap() {
        armd::rt_core::executor::MaintenanceResult::PlanningSnapshot(s) => s,
        _ => panic!("unexpected maintenance reply"),
    };
    assert_eq!(snap.authority.stop_generation, sg);
    let mut start = [0.0f32; MAX_DOF];
    start[..7].copy_from_slice(&snap.commanded[..7]);
    let mut goal = start;
    goal[0] = (q_stopped + 0.18).min(0.9);
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

// 5. Zero-alloc/zero-free RT stop proof, refusal-only service_wake flush, watchdog park preservation, panic-before-publish.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn zero_alloc_rt_stop_refusal_wake_park_preservation_and_panic_safety() {
    let conn = "connection:alloc-test";
    let mk = |req_id, lg, e| UrgentStop {
        connection: armd::model::HolderLabel::new(conn), lease_generation: lg, epoch: Some(e),
        request_id: req_id, request_id_valid: true, body_error: None,
    };

    // (a) Zero-alloc/zero-free RT stop across pre-handoff heap-owning trajectory (`motion_limits`), idle stop, and refused stop.
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
    let handle = sup.start_executor(ExecutorConfig::for_tests(Duration::from_millis(5))).await.unwrap();
    wait_until(Duration::from_secs(1), || handle.stats().steps.load(Ordering::Relaxed) >= 5).await;
    let pre_samples = sup.tick_samples().len();
    let ep = sup.observation().await.unwrap().epoch_id;
    assert!(matches!(sup.urgent_stop(mk(1, gen, ep)).await.unwrap(), UrgentStopOutcome::Applied { applied_to_motion: true, stop_generation: 1, .. }));
    assert!(matches!(sup.urgent_stop(mk(2, 0, ep)).await.unwrap(), UrgentStopOutcome::LeaseLost { .. }));
    assert!(matches!(sup.urgent_stop(mk(3, gen, ep)).await.unwrap(), UrgentStopOutcome::Applied { stop_generation: 2, .. }));
    let st_done = handle.stats().steps.load(Ordering::Relaxed);
    wait_until(Duration::from_secs(1), || handle.stats().steps.load(Ordering::Relaxed) >= st_done + 3).await;
    let max_allocs = sup.tick_samples()[pre_samples..].iter().map(|s| s.usage.lock_alloc_calls).max().unwrap_or(0);
    assert_eq!(max_allocs, 0, "RT urgent stop (including heap trajectory retirement) must do 0 allocs/frees");
    assert_eq!(handle.urgent_stop_in_flight(), 0);
    handle.stop_and_join();

    // (b) Refusal-only `service_wake` flushes urgent stop reply without requiring a state publish.
    let (mut engine, _broker, _tw, _stops) = ReleaseEngine::new(BootId::fresh(), Capacities::default());
    let (client, intake) = urgent_stop_channel(4, Arc::new(AtomicBool::new(false)));
    engine.attach_urgent_stops(intake);
    let mut core = SupervisorInner::new(ArmConfig::rebot_dm());
    let mut rx = client.enqueue_async(mk(1, 0, 1)).unwrap();
    let mut report = ReleaseReport::default();
    let published = engine.service_wake(&mut core, Instant::now(), Instant::now() + Duration::from_millis(5), 1, &mut report);
    assert!(!published && engine.take_urgent_replies_flushed());
    client.drain_replies();
    assert_eq!(rx.try_recv().unwrap().unwrap(), UrgentStopOutcome::LeaseLost { epoch: core.safety.epoch_id() });

    // (c) Watchdog park preserved by `authorized_stop` (returns false, keeps active park trajectory).
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

    // (d) Panic strictly between urgent-stop apply and Step 6 publish converts unpublished Stop to 503 EXECUTOR_STOPPED.
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
    let h_p = sup_p.start_executor_with_hooks(ecfg, PersistTestHooks {
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
}
