//! Operator authentication, permission matrix, expected_revision, and atomic
//! persist-before-commit integration tests (P1a).
mod common;

use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;

use serde_json::{json, Value};
use tokio::net::TcpListener;
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::client::IntoClientRequest;
use tokio_tungstenite::tungstenite::http::header::{AUTHORIZATION, COOKIE};

use armd::api::create_router_with_auth;
use armd::model::ArmConfig;
use armd::safety::{FaultCode, FaultReason, SafetyState};
use armd::{ArmSupervisor, OperatorAuth};
use common::{INSPECT, TestArm, TestSocket};

async fn spawn_auth_server(
    supervisor: Arc<ArmSupervisor>,
    auth: OperatorAuth,
) -> SocketAddr {
    supervisor.set_operator_auth_enforced(auth.is_enforced()).await;
    let router = create_router_with_auth(supervisor.clone(), auth);
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });
    let ticker = supervisor.clone();
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(20));
        loop {
            interval.tick().await;
            ticker.tick(0.02).await;
        }
    });
    addr
}

async fn connect_ws_with_header(addr: SocketAddr, header_name: &str, header_val: Option<&str>) -> TestSocket {
    let mut req = format!("ws://{addr}/api/v1/ws").into_client_request().unwrap();
    if let Some(val) = header_val {
        let name = axum::http::HeaderName::from_bytes(header_name.as_bytes()).unwrap();
        let hv = axum::http::HeaderValue::from_str(val).unwrap();
        req.headers_mut().insert(name, hv);
    }
    let (ws, _) = connect_async(req).await.unwrap();
    ws
}

async fn acquire(ws: &mut TestSocket, client_id: &str, force: bool) -> Value {
    common::ws_reply(
        ws,
        json!({"type": "acquire_lease", "client_id": client_id, "force": force, "seq": 1}),
        &["lease_granted", "lease_rejected"],
    )
    .await
}

fn cube(name: &str, c: [f64; 3]) -> Value {
    json!({
        "name": name,
        "min_m": [c[0] - 0.02, c[1] - 0.02, c[2] - 0.02],
        "max_m": [c[0] + 0.02, c[1] + 0.02, c[2] + 0.02],
    })
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn auth_endpoints_token_cookie_lock_and_origin() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let auth = OperatorAuth::enforced("123456", 1800);
    let addr = spawn_auth_server(sup.clone(), auth).await;
    let client = reqwest::Client::new();
    let base = format!("http://{addr}");

    // Initial unauthenticated status
    let st: Value = client.get(format!("{base}/api/v1/auth/status")).send().await.unwrap().json().await.unwrap();
    assert_eq!(st, json!({"auth_enforced": true, "role": "program", "expires_in_s": null}));

    // safety/describe reports operator_auth_enforced: true and preserves 8 top-level keys
    let desc: Value = client.get(format!("{base}/api/v1/safety/describe")).send().await.unwrap().json().await.unwrap();
    assert_eq!(desc["hardware"]["operator_auth_enforced"], true);
    assert_eq!(desc.as_object().unwrap().len(), 8);

    // Cross-origin POST /api/v1/auth/unlock rejected by same_origin
    let cross = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .header("Origin", "http://evil.example")
        .json(&json!({"pin": "123456"}))
        .send()
        .await
        .unwrap();
    assert_eq!(cross.status(), 403);

    // Invalid PIN rejected with 403 INVALID_PIN / invalid_pin
    let bad = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .json(&json!({"pin": "000000"}))
        .send()
        .await
        .unwrap();
    assert_eq!(bad.status(), 403);
    let bad_body: Value = bad.json().await.unwrap();
    assert_eq!(bad_body["code"], "INVALID_PIN");
    assert_eq!(bad_body["error"], "invalid_pin");

    // Valid PIN issues 64-char hex token + HttpOnly SameSite=Strict cookie
    let ok = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .json(&json!({"pin": "123456"}))
        .send()
        .await
        .unwrap();
    assert_eq!(ok.status(), 200);
    let cookie_hdr = ok.headers().get("set-cookie").unwrap().to_str().unwrap().to_string();
    assert!(cookie_hdr.contains("HttpOnly"), "{cookie_hdr}");
    assert!(cookie_hdr.contains("SameSite=Strict"), "{cookie_hdr}");
    assert!(cookie_hdr.contains("Max-Age=1800"), "{cookie_hdr}");
    let ok_body: Value = ok.json().await.unwrap();
    assert_eq!(ok_body["role"], "operator");
    assert_eq!(ok_body["expires_in_s"], 1800);
    let token = ok_body["token"].as_str().unwrap().to_string();
    assert_eq!(token.len(), 64);
    assert!(token.bytes().all(|b| b.is_ascii_hexdigit()));
    assert!(cookie_hdr.contains(&format!("rebot_op={token}")));

    // Status via Bearer and via Cookie both return operator
    let st_bearer: Value = client
        .get(format!("{base}/api/v1/auth/status"))
        .header(AUTHORIZATION.as_str(), format!("Bearer {token}"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(st_bearer["role"], "operator");
    assert!(st_bearer["expires_in_s"].as_u64().unwrap() > 0);

    let st_cookie: Value = client
        .get(format!("{base}/api/v1/auth/status"))
        .header(COOKIE.as_str(), format!("rebot_op={token}"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(st_cookie["role"], "operator");

    // Bearer lock revokes token and does NOT emit Set-Cookie
    let lock_resp = client
        .post(format!("{base}/api/v1/auth/lock"))
        .header(AUTHORIZATION.as_str(), format!("Bearer {token}"))
        .send()
        .await
        .unwrap();
    assert_eq!(lock_resp.status(), 200);
    assert!(lock_resp.headers().get("set-cookie").is_none());
    let lock_body: Value = lock_resp.json().await.unwrap();
    assert_eq!(lock_body["role"], "program");

    // Cookie-only lock revokes cookie session and clears cookie with Max-Age=0
    let ok2: Value = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .json(&json!({"pin": "123456"}))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let cookie_token = ok2["token"].as_str().unwrap().to_string();
    let cookie_lock_resp = client
        .post(format!("{base}/api/v1/auth/lock"))
        .header(COOKIE.as_str(), format!("rebot_op={cookie_token}"))
        .send()
        .await
        .unwrap();
    assert_eq!(cookie_lock_resp.status(), 200);
    let lock_cookie = cookie_lock_resp.headers().get("set-cookie").unwrap().to_str().unwrap().to_string();
    assert!(lock_cookie.contains("Max-Age=0"), "{lock_cookie}");
    let st_cookie_after: Value = client
        .get(format!("{base}/api/v1/auth/status"))
        .header(COOKIE.as_str(), format!("rebot_op={cookie_token}"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(st_cookie_after["role"], "program");

    // After lock, token is no longer operator
    let st_after: Value = client
        .get(format!("{base}/api/v1/auth/status"))
        .header(AUTHORIZATION.as_str(), format!("Bearer {token}"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(st_after["role"], "program");
    assert!(st_after["expires_in_s"].is_null());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn permission_matrix_program_vs_operator_and_live_revocation() {
    let dir = tempfile::tempdir().unwrap();
    let keepout_path = dir.path().join("keepout.json");
    let collision_path = dir.path().join("collision.json");
    let calib_path = dir.path().join("gripper_calib.json");

    let arm = TestArm::new();
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(arm)));
    armd::keepout::load_into(&sup, &keepout_path).await.unwrap();
    armd::collision::load_into(&sup, &collision_path).await;
    armd::gripper_calib::load_into(&sup, &calib_path).await;
    {
        let mut inner = sup.inner().write().await;
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        inner.measured_positions[6] = -0.15;
        inner.commanded_positions[6] = -0.15;
        inner.target_positions[6] = -0.15;
    }

    let auth = OperatorAuth::enforced("123456", 1800);
    let addr = spawn_auth_server(sup.clone(), auth).await;
    let client = reqwest::Client::new();
    let base = format!("http://{addr}");

    // 1. Program WS (even with localhost, User-Agent, client_id="operator") stays program
    let mut prog_ws = connect_ws_with_header(addr, "User-Agent", Some("rebot-operator/1.0")).await;
    let granted = acquire(&mut prog_ws, "operator", false).await;
    assert_eq!(granted["type"], "lease_granted");
    let prog_gen = granted["generation"].as_u64().unwrap();

    // Second unauthenticated WS cannot force preempt
    let mut attacker_ws = connect_ws_with_header(addr, "User-Agent", Some("cockpit")).await;
    let preempt_rej = acquire(&mut attacker_ws, "cockpit", true).await;
    assert_eq!(preempt_rej["type"], "lease_rejected");
    assert_eq!(preempt_rej["code"], "OPERATOR_REQUIRED");
    assert_eq!(preempt_rej["error"], "operator_required");
    // Original lease holder still owns lease
    assert!(sup.inner().read().await.active_lease.as_ref().unwrap().generation == prog_gen);

    let epoch = sup.get_observation().await.epoch_id;

    // Privileged commands denied for program, state MUST remain unchanged
    let r = common::rpc(&mut prog_ws, prog_gen, epoch, "/api/v1/workspace/keepout", json!({"boxes": [cube("b1", [0.3, 0.3, 0.3])]})).await;
    assert_eq!((r["code"].as_str(), r["error"].as_str(), r["status"].as_u64()), (Some("OPERATOR_REQUIRED"), Some("operator_required"), Some(403)), "{r}");
    let ko: Value = client.get(format!("{base}/api/v1/workspace/keepout")).send().await.unwrap().json().await.unwrap();
    assert_eq!(ko["boxes"].as_array().unwrap().len(), 0);
    assert_eq!(ko["revision"], 1);

    let r = common::rpc(&mut prog_ws, prog_gen, epoch, "/api/v1/safety/collision", json!({"enabled": true, "sensitivity": "high"})).await;
    assert_eq!(r["code"], "OPERATOR_REQUIRED");
    let col: Value = client.get(format!("{base}/api/v1/safety/collision")).send().await.unwrap().json().await.unwrap();
    assert_eq!(col["enabled"], false);
    assert_eq!(col["revision"], 1);

    let r = common::rpc(&mut prog_ws, prog_gen, epoch, "/api/v1/mode/gravity_comp", json!({"enable": true})).await;
    assert_eq!(r["code"], "OPERATOR_REQUIRED");
    assert!(sup.inner().read().await.gravity_comp.is_none());

    let r = common::rpc(&mut prog_ws, prog_gen, epoch, "/api/v1/gripper/calibration", json!({
        "jaw_opening": {"points": [[0.0, 0.0], [-5.0, 0.08]], "calibrated": true},
        "offset_rad": 0.02
    })).await;
    assert_eq!(r["code"], "OPERATOR_REQUIRED");
    let gc: Value = client.get(format!("{base}/api/v1/gripper/calibration")).send().await.unwrap().json().await.unwrap();
    assert_eq!(gc["revision"], 1);
    assert!(gc["offset_rad"].is_null());

    // Torque on IS allowed for program
    let r = common::rpc(&mut prog_ws, prog_gen, epoch, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(r["type"], "command_ack", "{r}");

    // gripper_zero_check(store_offset=true) denied for program
    let r = common::rpc(&mut prog_ws, prog_gen, epoch, "/api/v1/action/gripper_zero_check", json!({"store_offset": true, "threshold_rad": 0.1})).await;
    assert_eq!(r["code"], "OPERATOR_REQUIRED");
    assert!(sup.inner().read().await.last_action.is_none());

    // calibration/apply, calibration/restore, grasp/setup denied for program
    let r = common::rpc(&mut prog_ws, prog_gen, epoch, "/api/v1/calibration/apply", json!({"dry_run": true})).await;
    assert_eq!(r["code"], "OPERATOR_REQUIRED");
    let r = common::rpc(&mut prog_ws, prog_gen, epoch, "/api/v1/calibration/restore", json!({"backup": "x"})).await;
    assert_eq!(r["code"], "OPERATOR_REQUIRED");
    let r = common::rpc(&mut prog_ws, prog_gen, epoch, "/api/v1/grasp/setup", json!({"table_z_m": 0.1})).await;
    assert_eq!(r["code"], "OPERATOR_REQUIRED");

    // HTTP POST privileged endpoints also return 403 OPERATOR_REQUIRED for program
    let http_gc = client.post(format!("{base}/api/v1/mode/gravity_comp")).json(&json!({"enable": true})).send().await.unwrap();
    assert_eq!(http_gc.status(), 403);
    let http_zc = client.post(format!("{base}/api/v1/action/gripper_zero_check")).json(&json!({"store_offset": true})).send().await.unwrap();
    assert_eq!(http_zc.status(), 403);

    // Program is DENIED poses/save and poses/delete (Operator only), while motion/stop and gripper_zero_check(store_offset=false) remain allowed
    let r = common::rpc(&mut prog_ws, prog_gen, epoch, "/api/v1/poses/save", json!({"name": "scan", "expected_revision": 1})).await;
    assert_eq!(r["code"], "OPERATOR_REQUIRED", "{r}");
    let r = common::rpc(&mut prog_ws, prog_gen, epoch, "/api/v1/poses/delete", json!({"name": "scan", "expected_revision": 1})).await;
    assert_eq!(r["code"], "OPERATOR_REQUIRED", "{r}");
    let r = common::rpc(&mut prog_ws, prog_gen, epoch, "/api/v1/motion/stop", json!({})).await;
    assert_eq!(r["type"], "command_ack", "{r}");

    // Program CAN clear recoverable fault (WatchdogTimeout) with matching epoch_id
    {
        let mut inner = sup.inner().write().await;
        inner.safety.trigger_fault(FaultReason::WatchdogTimeout);
    }
    let w_epoch = sup.get_observation().await.epoch_id;
    let r = common::rpc(&mut prog_ws, prog_gen, w_epoch, "/api/v1/safety/clear_fault", json!({})).await;
    assert_eq!(r["type"], "command_ack", "{r}");
    assert_eq!(sup.get_observation().await.safety_state, SafetyState::Idle);

    // Program CANNOT clear non-recoverable Collision fault
    {
        let mut inner = sup.inner().write().await;
        inner.safety.trigger_fault_coded(FaultCode::Collision, FaultReason::Other("too many trips".into()));
    }
    let c_epoch = sup.get_observation().await.epoch_id;
    let r = common::rpc(&mut prog_ws, prog_gen, c_epoch, "/api/v1/safety/clear_fault", json!({})).await;
    assert_eq!(r["code"], "OPERATOR_REQUIRED", "{r}");
    assert!(matches!(sup.get_observation().await.safety_state, SafetyState::Fault(_)));

    // 2. Unlock operator and connect via Cookie AND Bearer
    let unlock: Value = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .json(&json!({"pin": "123456"}))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let op_token = unlock["token"].as_str().unwrap().to_string();

    // Cookie WS force-preempts program WS!
    let mut op_ws = connect_ws_with_header(addr, "Cookie", Some(&format!("rebot_op={op_token}"))).await;
    let granted_op = acquire(&mut op_ws, "cockpit_op", true).await;
    assert_eq!(granted_op["type"], "lease_granted", "{granted_op}");
    let op_gen = granted_op["generation"].as_u64().unwrap();

    // Operator clears Collision fault -> Idle
    let cur_epoch = sup.get_observation().await.epoch_id;
    let r = common::rpc(&mut op_ws, op_gen, cur_epoch, "/api/v1/safety/clear_fault", json!({})).await;
    assert_eq!(r["type"], "command_ack", "{r}");
    assert_eq!(sup.get_observation().await.safety_state, SafetyState::Idle);

    // EStop (no lease needed via REST) cannot be cleared by program, CAN be cleared by operator
    let es_resp: Value = client.post(format!("{base}/api/v1/safety/e_stop")).send().await.unwrap().json().await.unwrap();
    assert_eq!(es_resp["safety_state"], "EStop");
    let e_epoch = sup.get_observation().await.epoch_id;
    let rej_preempt = acquire(&mut prog_ws, "prog2", true).await;
    assert_eq!(rej_preempt["code"], "OPERATOR_REQUIRED");
    // Operator releases lease so program can acquire normally to test EStop clear denial
    let rel = common::ws_reply(&mut op_ws, json!({"type": "release_lease", "generation": op_gen}), &["lease_released"]).await;
    assert_eq!(rel["success"], true);
    let granted_p2 = acquire(&mut prog_ws, "prog2", false).await;
    let p2_gen = granted_p2["generation"].as_u64().unwrap();
    let r = common::rpc(&mut prog_ws, p2_gen, e_epoch, "/api/v1/safety/clear_fault", json!({})).await;
    assert_eq!(r["code"], "OPERATOR_REQUIRED", "{r}");
    assert_eq!(sup.get_observation().await.safety_state, SafetyState::EStop);

    // Operator force-preempts (after cooldown) and clears EStop -> Idle
    tokio::time::sleep(Duration::from_millis(3100)).await;
    let granted_op2 = acquire(&mut op_ws, "cockpit_op", true).await;
    assert_eq!(granted_op2["type"], "lease_granted", "{granted_op2}");
    let op_gen = granted_op2["generation"].as_u64().unwrap();
    let r = common::rpc(&mut op_ws, op_gen, e_epoch, "/api/v1/safety/clear_fault", json!({})).await;
    assert_eq!(r["type"], "command_ack", "{r}");
    assert_eq!(sup.get_observation().await.safety_state, SafetyState::Idle);

    let cur_epoch = sup.get_observation().await.epoch_id;
    // Operator can write keepout, collision, gripper_calib
    let ack_k = common::rpc(&mut op_ws, op_gen, cur_epoch, "/api/v1/workspace/keepout", json!({"boxes": [cube("tray", [0.3, 0.3, 0.3])], "expected_revision": 1})).await;
    assert_eq!(ack_k["type"], "command_ack", "{ack_k}");
    assert_eq!(ack_k["persisted"], true);
    assert_eq!(ack_k["revision"], 2);

    let ack_c = common::rpc(&mut op_ws, op_gen, cur_epoch, "/api/v1/safety/collision", json!({"enabled": true, "sensitivity": "high", "expected_revision": 1})).await;
    assert_eq!(ack_c["type"], "command_ack", "{ack_c}");
    assert_eq!(ack_c["persisted"], true);
    assert_eq!(ack_c["revision"], 2);

    let ack_g = common::rpc(&mut op_ws, op_gen, cur_epoch, "/api/v1/gripper/calibration", json!({
        "jaw_opening": {"points": [[0.0, 0.0], [-5.7, 0.08]], "calibrated": true},
        "offset_rad": 0.02,
        "expected_revision": 1
    })).await;
    assert_eq!(ack_g["type"], "command_ack", "{ack_g}");
    assert_eq!(ack_g["persisted"], true);
    assert_eq!(ack_g["revision"], 2);

    // 3. Live revocation via POST /api/v1/auth/lock immediately revokes already-open op_ws!
    let lock_res = client
        .post(format!("{base}/api/v1/auth/lock"))
        .header(COOKIE.as_str(), format!("rebot_op={op_token}"))
        .send()
        .await
        .unwrap();
    assert_eq!(lock_res.status(), 200);

    // Wait a tick? No, lock is synchronous — immediately try a privileged op on the ALREADY-OPEN op_ws!
    let rej_after_lock = common::rpc(&mut op_ws, op_gen, cur_epoch, "/api/v1/safety/collision", json!({"enabled": false})).await;
    assert_eq!(rej_after_lock["type"], "command_reject", "{rej_after_lock}");
    assert_eq!(rej_after_lock["code"], "OPERATOR_REQUIRED");
    // Collision stays enabled at revision 2
    let col_after: Value = client.get(format!("{base}/api/v1/safety/collision")).send().await.unwrap().json().await.unwrap();
    assert_eq!(col_after["enabled"], true);
    assert_eq!(col_after["revision"], 2);

    // Also verify Bearer WS works after a fresh unlock
    let unlock2: Value = client.post(format!("{base}/api/v1/auth/unlock")).json(&json!({"pin": "123456"})).send().await.unwrap().json().await.unwrap();
    let tok2 = unlock2["token"].as_str().unwrap();
    let mut bearer_ws = connect_ws_with_header(addr, "Authorization", Some(&format!("Bearer {tok2}"))).await;
    tokio::time::sleep(Duration::from_millis(3100)).await; // wait past preemption cooldown
    let granted_b = acquire(&mut bearer_ws, "bearer_op", true).await;
    assert_eq!(granted_b["type"], "lease_granted", "{granted_b}");
    let b_gen = granted_b["generation"].as_u64().unwrap();
    let ack_b = common::rpc(&mut bearer_ws, b_gen, cur_epoch, "/api/v1/safety/collision", json!({"enabled": false, "expected_revision": 2})).await;
    assert_eq!(ack_b["type"], "command_ack", "{ack_b}");
    assert_eq!(ack_b["revision"], 3);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn expected_revision_mismatch_and_persist_failed_preserve_memory() {
    let dir = tempfile::tempdir().unwrap();
    // Point keepout and collision paths inside a regular file so write_atomic fails reliably (ENOTDIR)
    let blocker_file = dir.path().join("not_a_dir");
    std::fs::write(&blocker_file, b"x").unwrap();
    let bad_keepout = blocker_file.join("keepout.json");
    let bad_collision = blocker_file.join("collision.json");
    let bad_calib = blocker_file.join("gripper_calib.json");

    let arm = TestArm::new();
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(arm)));
    {
        let mut inner = sup.inner().write().await;
        inner.keepout.path = Some(bad_keepout);
        inner.collision.path = Some(bad_collision);
        inner.gripper_calib_path = Some(bad_calib);
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
    }

    let auth = OperatorAuth::enforced("123456", 1800);
    let addr = spawn_auth_server(sup.clone(), auth).await;
    let client = reqwest::Client::new();
    let base = format!("http://{addr}");

    let unlock: Value = client.post(format!("{base}/api/v1/auth/unlock")).json(&json!({"pin": "123456"})).send().await.unwrap().json().await.unwrap();
    let tok = unlock["token"].as_str().unwrap();
    let mut ws = connect_ws_with_header(addr, "Authorization", Some(&format!("Bearer {tok}"))).await;
    let granted = acquire(&mut ws, "op", false).await;
    let gen = granted["generation"].as_u64().unwrap();
    let epoch = sup.get_observation().await.epoch_id;

    // 1. Stale expected_revision rejected BEFORE disk write
    let stale_k = common::rpc(&mut ws, gen, epoch, "/api/v1/workspace/keepout", json!({"boxes": [cube("t", [0.3, 0.3, 0.3])], "expected_revision": 99})).await;
    assert_eq!(stale_k["type"], "command_reject");
    assert_eq!(stale_k["code"], "POLICY_REVISION_MISMATCH");
    assert_eq!(stale_k["error"], "policy_revision_mismatch");
    assert_eq!(stale_k["current_revision"], 1);

    let stale_c = common::rpc(&mut ws, gen, epoch, "/api/v1/safety/collision", json!({"enabled": true, "expected_revision": 5})).await;
    assert_eq!(stale_c["code"], "POLICY_REVISION_MISMATCH");
    assert_eq!(stale_c["error"], "policy_revision_mismatch");
    assert_eq!(stale_c["current_revision"], 1);

    let stale_g = common::rpc(&mut ws, gen, epoch, "/api/v1/gripper/calibration", json!({
        "jaw_opening": {"points": [[0.0, 0.0], [-5.0, 0.08]], "calibrated": true},
        "expected_revision": 3
    })).await;
    assert_eq!(stale_g["code"], "POLICY_REVISION_MISMATCH");
    assert_eq!(stale_g["error"], "policy_revision_mismatch");
    assert_eq!(stale_g["current_revision"], 1);

    // 2. Disk persist failure returns PERSIST_FAILED and leaves in-memory state + revision UNCHANGED
    let fail_k = common::rpc(&mut ws, gen, epoch, "/api/v1/workspace/keepout", json!({"boxes": [cube("t", [0.3, 0.3, 0.3])], "expected_revision": 1})).await;
    assert_eq!(fail_k["type"], "command_reject", "{fail_k}");
    assert_eq!(fail_k["status"], 500);
    assert_eq!(fail_k["code"], "PERSIST_FAILED");
    assert_eq!(fail_k["error"], "persist_failed");
    let ko: Value = client.get(format!("{base}/api/v1/workspace/keepout")).send().await.unwrap().json().await.unwrap();
    assert_eq!(ko["boxes"].as_array().unwrap().len(), 0);
    assert_eq!(ko["source"], "profile");
    assert_eq!(ko["revision"], 1);

    let fail_c = common::rpc(&mut ws, gen, epoch, "/api/v1/safety/collision", json!({"enabled": true, "sensitivity": "high", "expected_revision": 1})).await;
    assert_eq!(fail_c["type"], "command_reject", "{fail_c}");
    assert_eq!(fail_c["code"], "PERSIST_FAILED");
    assert_eq!(fail_c["error"], "persist_failed");
    let col: Value = client.get(format!("{base}/api/v1/safety/collision")).send().await.unwrap().json().await.unwrap();
    assert_eq!(col["enabled"], false);
    assert_eq!(col["sensitivity"], "medium");
    assert_eq!(col["revision"], 1);

    let fail_g = common::rpc(&mut ws, gen, epoch, "/api/v1/gripper/calibration", json!({
        "jaw_opening": {"points": [[0.0, 0.0], [-5.7, 0.08]], "calibrated": true},
        "offset_rad": 0.02,
        "expected_revision": 1
    })).await;
    assert_eq!(fail_g["type"], "command_reject", "{fail_g}");
    assert_eq!(fail_g["code"], "PERSIST_FAILED");
    assert_eq!(fail_g["error"], "persist_failed");
    let gc: Value = client.get(format!("{base}/api/v1/gripper/calibration")).send().await.unwrap().json().await.unwrap();
    assert!(gc["offset_rad"].is_null());
    assert_eq!(gc["revision"], 1);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn ttl_expiration_revokes_open_ws_operator_privilege() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let auth = OperatorAuth::enforced("123456", 1);
    let addr = spawn_auth_server(sup.clone(), auth).await;
    let client = reqwest::Client::new();
    let base = format!("http://{addr}");

    let unlock: Value = client.post(format!("{base}/api/v1/auth/unlock")).json(&json!({"pin": "123456"})).send().await.unwrap().json().await.unwrap();
    let tok = unlock["token"].as_str().unwrap();
    let mut ws = connect_ws_with_header(addr, "Authorization", Some(&format!("Bearer {tok}"))).await;
    let granted = acquire(&mut ws, "op", false).await;
    let gen = granted["generation"].as_u64().unwrap();
    let epoch = sup.get_observation().await.epoch_id;

    // Wait past 1s TTL
    tokio::time::sleep(Duration::from_millis(1150)).await;
    let rej = common::rpc(&mut ws, gen, epoch, "/api/v1/workspace/keepout", json!({"boxes": []})).await;
    assert_eq!(rej["type"], "command_reject", "{rej}");
    assert_eq!(rej["code"], "OPERATOR_REQUIRED");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn zero_check_store_offset_persists_before_commit_and_disabled_auth_checks_revision() {
    let dir = tempfile::tempdir().unwrap();
    let blocker = dir.path().join("file_blocker");
    std::fs::write(&blocker, b"x").unwrap();
    let bad_calib = blocker.join("gripper_calib.json");
    let good_calib = dir.path().join("gripper_calib.json");

    // Test with OperatorAuth::disabled(): expected_revision & atomic persist still active!
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    {
        let mut inner = sup.inner().write().await;
        inner.gripper_calib_path = Some(bad_calib);
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        inner.measured_positions[6] = -0.15;
        inner.commanded_positions[6] = -0.15;
        inner.target_positions[6] = -0.15;
        inner.set_mock_gripper_stall_offset(Some(0.05));
    }
    let addr = spawn_auth_server(sup.clone(), OperatorAuth::disabled()).await;
    let client = reqwest::Client::new();
    let base = format!("http://{addr}");

    let desc: Value = client.get(format!("{base}/api/v1/safety/describe")).send().await.unwrap().json().await.unwrap();
    assert_eq!(desc["hardware"]["operator_auth_enforced"], false);

    let mut ws = connect_ws_with_header(addr, "User-Agent", Some("prog")).await;
    let g = acquire(&mut ws, "prog", false).await["generation"].as_u64().unwrap();
    let epoch = sup.get_observation().await.epoch_id;
    let ack = common::rpc(&mut ws, g, epoch, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");

    // Stale expected_revision rejected even when OperatorAuth is disabled
    let stale = common::rpc(&mut ws, g, epoch, "/api/v1/gripper/calibration", json!({
        "jaw_opening": {"points": [[0.0, 0.0], [-5.0, 0.08]], "calibrated": true},
        "expected_revision": 42
    })).await;
    assert_eq!(stale["code"], "POLICY_REVISION_MISMATCH");

    // Zero check store_offset=true with unwritable path -> action fails with PERSIST_FAILED, policy/revision unchanged
    let z_ack = common::rpc(&mut ws, g, epoch, "/api/v1/action/gripper_zero_check", json!({"threshold_rad": 0.1, "store_offset": true})).await;
    assert_eq!(z_ack["type"], "command_ack", "{z_ack}");
    let aid = z_ack["action_id"].as_u64().unwrap();
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    let failed_act = loop {
        common::send(&mut ws, json!({"type": "heartbeat", "generation": g, "epoch_id": epoch})).await;
        if let Some(a) = sup.get_observation().await.action.filter(|a| a.id == aid && !a.is_running()) {
            break a;
        }
        assert!(std::time::Instant::now() < deadline, "zero_check did not finish");
        tokio::time::sleep(Duration::from_millis(20)).await;
    };
    assert_eq!(failed_act.state, "failed", "{failed_act:?}");
    assert_eq!(failed_act.code.as_deref(), Some("PERSIST_FAILED"));
    let gc_after_fail = sup.gripper_calibration_response().await;
    assert!(gc_after_fail.offset_rad.is_none());
    assert_eq!(gc_after_fail.revision, 1);

    // Now switch to writable path and verify zero_check(store_offset=true) commits & bumps revision
    {
        let mut inner = sup.inner().write().await;
        inner.gripper_calib_path = Some(good_calib.clone());
        inner.measured_positions[6] = -0.15;
        inner.commanded_positions[6] = -0.15;
        inner.target_positions[6] = -0.15;
    }
    let z2 = common::rpc(&mut ws, g, epoch, "/api/v1/action/gripper_zero_check", json!({"threshold_rad": 0.1, "store_offset": true})).await;
    let aid2 = z2["action_id"].as_u64().unwrap();
    let deadline2 = std::time::Instant::now() + Duration::from_secs(5);
    let done_act = loop {
        common::send(&mut ws, json!({"type": "heartbeat", "generation": g, "epoch_id": epoch})).await;
        if let Some(a) = sup.get_observation().await.action.filter(|a| a.id == aid2 && !a.is_running()) {
            break a;
        }
        assert!(std::time::Instant::now() < deadline2, "zero_check 2 did not finish");
        tokio::time::sleep(Duration::from_millis(20)).await;
    };
    assert_eq!(done_act.state, "done", "{done_act:?}");
    assert_eq!(done_act.durability_confirmed, Some(true), "{done_act:?}");
    assert!(done_act.durability_warning.is_none(), "{done_act:?}");
    assert!(sup.inner().read().await.zero_check_persisting.is_none());
    assert!(good_calib.exists());
    let gc_after_ok = sup.gripper_calibration_response().await;
    assert!((gc_after_ok.offset_rad.unwrap() - 0.05).abs() < 1e-2);
    assert_eq!(gc_after_ok.revision, 2);
}

static WRITE_DELAY_SERIAL: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn keepout_motion_between_stage_and_publish_leaves_disk_and_memory_unchanged() {
    let _serial = WRITE_DELAY_SERIAL.lock().await;
    let dir = tempfile::tempdir().unwrap();
    let keepout_path = dir.path().join("keepout.json");
    let old_box = cube("old_box", [0.3, 0.3, 0.3]);
    let new_box = cube("new_box", [0.35, 0.35, 0.35]);
    std::fs::write(
        &keepout_path,
        serde_json::to_string_pretty(&json!({"boxes": [old_box]})).unwrap(),
    )
    .unwrap();

    let arm = TestArm::new();
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(arm)));
    armd::keepout::load_into(&sup, &keepout_path).await.unwrap();
    {
        let mut inner = sup.inner().write().await;
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
    }

    let auth = OperatorAuth::enforced("123456", 1800);
    let addr = spawn_auth_server(sup.clone(), auth).await;
    let client = reqwest::Client::new();
    let base = format!("http://{addr}");

    let unlock: Value = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .json(&json!({"pin": "123456"}))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let tok = unlock["token"].as_str().unwrap();
    let mut ws = connect_ws_with_header(addr, "Authorization", Some(&format!("Bearer {tok}"))).await;
    let g = acquire(&mut ws, "op", false).await["generation"].as_u64().unwrap();
    let epoch = sup.get_observation().await.epoch_id;
    let ack = common::rpc(&mut ws, g, epoch, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");

    // Slow down staging so we can start motion after stage begins and before publish runs.
    armd::persist::TEST_WRITE_DELAY_MS.store(250, std::sync::atomic::Ordering::Relaxed);
    common::send(
        &mut ws,
        json!({
            "type": "command",
            "request_id": 500_001,
            "generation": g,
            "epoch_id": epoch,
            "path": "/api/v1/workspace/keepout",
            "body": {"boxes": [new_box], "expected_revision": 1}
        }),
    )
    .await;

    // Wait briefly so the config write task acquires cw_lock and enters StagedWrite::stage,
    // then start a motion (jog) before publish acquires sup.inner().write().
    tokio::time::sleep(Duration::from_millis(50)).await;
    let mut target = INSPECT.to_vec();
    target[0] += 0.2;
    let jog_reply = common::ws_reply(
        &mut ws,
        json!({
            "type": "command",
            "request_id": 500_002,
            "generation": g,
            "epoch_id": epoch,
            "path": "/api/v1/motion/jog",
            "body": {"target_positions": target}
        }),
        &["command_ack", "command_reject"],
    )
    .await;
    assert_eq!(jog_reply["type"], "command_ack", "{jog_reply}");

    // Now read the delayed keepout reply (request_id 500_001).
    use futures_util::StreamExt;
    let reject = tokio::time::timeout(Duration::from_secs(5), async {
        loop {
            let text = ws.next().await.unwrap().unwrap().into_text().unwrap();
            let msg: Value = serde_json::from_str(&text).unwrap();
            if msg.get("request_id") == Some(&json!(500_001)) {
                break msg;
            }
        }
    })
    .await
    .unwrap();
    armd::persist::TEST_WRITE_DELAY_MS.store(0, std::sync::atomic::Ordering::Relaxed);

    assert_eq!(reject["type"], "command_reject", "{reject}");
    assert_eq!(reject["code"], "MOTION_BUSY", "{reject}");

    // Memory, official file on disk, and a fresh reload must all keep ONLY old_box.
    let ko_mem: Value = client.get(format!("{base}/api/v1/workspace/keepout")).send().await.unwrap().json().await.unwrap();
    assert_eq!(ko_mem["revision"], 1);
    assert_eq!(ko_mem["boxes"].as_array().unwrap().len(), 1);
    assert_eq!(ko_mem["boxes"][0]["name"], "old_box");

    let disk: Value = serde_json::from_str(&std::fs::read_to_string(&keepout_path).unwrap()).unwrap();
    assert_eq!(disk["boxes"].as_array().unwrap().len(), 1);
    assert_eq!(disk["boxes"][0]["name"], "old_box");

    // No temp files left behind
    let files: Vec<String> = std::fs::read_dir(dir.path())
        .unwrap()
        .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
        .collect();
    assert_eq!(files, ["keepout.json"]);

    let fresh_sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    armd::keepout::load_into(&fresh_sup, &keepout_path).await.unwrap();
    let fresh_state = fresh_sup.inner().read().await.keepout_state();
    assert_eq!(fresh_state.boxes.len(), 1);
    assert_eq!(fresh_state.boxes[0].name, "old_box");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn zero_check_cancelled_before_persist_finishes_leaves_disk_and_memory_unchanged() {
    let _serial = WRITE_DELAY_SERIAL.lock().await;
    let dir = tempfile::tempdir().unwrap();
    let calib_path = dir.path().join("gripper_calib.json");
    std::fs::write(
        &calib_path,
        serde_json::to_string_pretty(&json!({"offset_rad": 0.01})).unwrap(),
    )
    .unwrap();

    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    armd::gripper_calib::load_into(&sup, &calib_path).await;
    {
        let mut inner = sup.inner().write().await;
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        inner.measured_positions[6] = -0.15;
        inner.commanded_positions[6] = -0.15;
        inner.target_positions[6] = -0.15;
        inner.set_mock_gripper_stall_offset(Some(0.05));
    }

    let addr = spawn_auth_server(sup.clone(), OperatorAuth::disabled()).await;
    let mut ws = connect_ws_with_header(addr, "User-Agent", Some("prog")).await;
    let g = acquire(&mut ws, "prog", false).await["generation"].as_u64().unwrap();
    let epoch = sup.get_observation().await.epoch_id;
    let ack = common::rpc(&mut ws, g, epoch, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");

    armd::persist::TEST_WRITE_DELAY_MS.store(250, std::sync::atomic::Ordering::Relaxed);
    let z_ack = common::rpc(
        &mut ws,
        g,
        epoch,
        "/api/v1/action/gripper_zero_check",
        json!({"threshold_rad": 0.1, "store_offset": true}),
    )
    .await;
    assert_eq!(z_ack["type"], "command_ack", "{z_ack}");
    let aid = z_ack["action_id"].as_u64().unwrap();

    // Wait until zero_check enters background persist (zero_check_persisting == Some(aid))
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    loop {
        common::send(&mut ws, json!({"type": "heartbeat", "generation": g, "epoch_id": epoch})).await;
        if sup.inner().read().await.zero_check_persisting == Some(aid) {
            break;
        }
        assert!(std::time::Instant::now() < deadline, "zero_check did not enter persisting state");
        tokio::time::sleep(Duration::from_millis(20)).await;
    }

    // Cancel/stop before background persist finishes
    let stop_ack = common::rpc(&mut ws, g, epoch, "/api/v1/motion/stop", json!({})).await;
    assert_eq!(stop_ack["type"], "command_ack", "{stop_ack}");

    // Wait past the persist delay so the background task completes and aborts its staged write
    tokio::time::sleep(Duration::from_millis(350)).await;
    armd::persist::TEST_WRITE_DELAY_MS.store(0, std::sync::atomic::Ordering::Relaxed);

    let act = sup.get_observation().await.action.unwrap();
    assert_eq!(act.id, aid);
    assert_eq!(act.state, "cancelled", "{act:?}");

    // Official file, in-memory offset, and revision are ALL unchanged (still 0.01, rev 1)
    let disk: Value = serde_json::from_str(&std::fs::read_to_string(&calib_path).unwrap()).unwrap();
    assert!((disk["offset_rad"].as_f64().unwrap() - 0.01).abs() < 1e-6, "{disk}");
    let gc = sup.gripper_calibration_response().await;
    assert!((gc.offset_rad.unwrap() - 0.01).abs() < 1e-6, "{gc:?}");
    assert_eq!(gc.revision, 1);
    let files: Vec<String> = std::fs::read_dir(dir.path())
        .unwrap()
        .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
        .collect();
    assert_eq!(files, ["gripper_calib.json"]);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn unlock_rate_limit_secure_cookie_and_clear_fault_pending_estop_epoch() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let auth = OperatorAuth::enforced("123456", 1800).with_trust_forwarded_proto(true);
    let addr = spawn_auth_server(sup.clone(), auth).await;
    let client = reqwest::Client::new();
    let base = format!("http://{addr}");

    // 1. Five bad PINs trigger 429 RATE_LIMITED
    for i in 0..5 {
        let r = client
            .post(format!("{base}/api/v1/auth/unlock"))
            .json(&json!({"pin": "000000"}))
            .send()
            .await
            .unwrap();
        assert_eq!(r.status(), 403, "attempt {i}");
    }
    // Even the correct PIN is rejected with 429 while locked out
    let limited = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .json(&json!({"pin": "123456"}))
        .send()
        .await
        .unwrap();
    assert_eq!(limited.status(), 429);
    let lim_body: Value = limited.json().await.unwrap();
    assert_eq!(lim_body["code"], "RATE_LIMITED");
    assert_eq!(lim_body["error"], "rate_limited");
    assert_eq!(lim_body["retry_after_s"], 1);

    // After 1s lockout expires, valid PIN succeeds and resets counter
    tokio::time::sleep(Duration::from_millis(1050)).await;
    let ok = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .header("X-Forwarded-Proto", "https")
        .json(&json!({"pin": "123456"}))
        .send()
        .await
        .unwrap();
    assert_eq!(ok.status(), 200);
    let set_cookie = ok.headers().get("set-cookie").unwrap().to_str().unwrap().to_string();
    assert!(set_cookie.contains("; Secure"), "{set_cookie}");
    let tok = ok.json::<Value>().await.unwrap()["token"].as_str().unwrap().to_string();

    // Cookie lock under Forwarded: proto=https also appends ; Secure
    let lock_https = client
        .post(format!("{base}/api/v1/auth/lock"))
        .header(COOKIE.as_str(), format!("rebot_op={tok}"))
        .header("Forwarded", "for=192.0.2.1;proto=https")
        .send()
        .await
        .unwrap();
    assert_eq!(lock_https.status(), 200);
    let lock_cookie = lock_https.headers().get("set-cookie").unwrap().to_str().unwrap().to_string();
    assert!(lock_cookie.contains("; Secure"), "{lock_cookie}");

    // Verify failure counter was reset (a single bad PIN returns 403, not 429)
    let after_reset_bad = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .json(&json!({"pin": "000000"}))
        .send()
        .await
        .unwrap();
    assert_eq!(after_reset_bad.status(), 403);

    // 2. MINOR-3: clear_fault with pending estop while already in Fault (epoch unchanged) -> OPERATOR_REQUIRED, not bogus EPOCH_MISMATCH
    let mut prog_ws = connect_ws_with_header(addr, "User-Agent", Some("prog")).await;
    let g = acquire(&mut prog_ws, "prog", false).await["generation"].as_u64().unwrap();
    {
        let mut inner = sup.inner().write().await;
        inner.safety.trigger_fault(FaultReason::WatchdogTimeout);
        // Simulate concurrent EStop counter bump while Fault is already active (consume_pending_estop keeps Fault epoch).
        inner.requested_estop_count.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
    }
    let epoch = sup.get_observation().await.epoch_id;
    let rej = common::rpc(&mut prog_ws, g, epoch, "/api/v1/safety/clear_fault", json!({})).await;
    assert_eq!(rej["type"], "command_reject", "{rej}");
    assert_eq!(rej["code"], "OPERATOR_REQUIRED", "{rej}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn gripper_calib_driver_rejection_between_stage_and_publish_preserves_disk_and_memory() {
    use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
    use armd::model::JointState;

    let _serial = WRITE_DELAY_SERIAL.lock().await;

    struct DynamicGripperLimitArm {
        inner: TestArm,
        j7_hw_limit: Arc<std::sync::Mutex<Option<(f32, f32)>>>,
    }
    impl ArmDriver for DynamicGripperLimitArm {
        fn capabilities(&self) -> DriverCapabilities { self.inner.capabilities() }
        fn enable_state(&self) -> EnableState { self.inner.enable_state() }
        fn is_feedback_valid(&self) -> bool { self.inner.is_feedback_valid() }
        fn is_enabled(&self) -> bool { self.inner.is_enabled() }
        fn feedback_ages(&self) -> Vec<Option<Duration>> { self.inner.feedback_ages() }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> { self.inner.read_joint_states() }
        fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> { self.inner.write_joint_targets(targets) }
        fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> { self.inner.set_torque(enabled) }
        fn position_limits(&self, joint_index: usize) -> Option<(f32, f32)> {
            if joint_index == 6 { *self.j7_hw_limit.lock().unwrap() } else { None }
        }
    }

    let dir = tempfile::tempdir().unwrap();
    let calib_path = dir.path().join("gripper_calib.json");
    std::fs::write(
        &calib_path,
        serde_json::to_string_pretty(&json!({"offset_rad": 0.0})).unwrap(),
    )
    .unwrap();

    let j7_hw_limit = Arc::new(std::sync::Mutex::new(None));
    let drv = DynamicGripperLimitArm { inner: TestArm::new(), j7_hw_limit: j7_hw_limit.clone() };
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
    armd::gripper_calib::load_into(&sup, &calib_path).await;

    let addr = spawn_auth_server(sup.clone(), OperatorAuth::disabled()).await;
    let mut ws = connect_ws_with_header(addr, "User-Agent", Some("op")).await;
    let g = acquire(&mut ws, "op", false).await["generation"].as_u64().unwrap();
    let epoch = sup.get_observation().await.epoch_id;

    armd::persist::TEST_WRITE_DELAY_MS.store(250, std::sync::atomic::Ordering::Relaxed);
    common::send(
        &mut ws,
        json!({
            "type": "command",
            "request_id": 600_001,
            "generation": g,
            "epoch_id": epoch,
            "path": "/api/v1/gripper/calibration",
            "body": {
                "jaw_opening": {"points": [[0.0, 0.0], [-5.0, 0.08]], "calibrated": true},
                "offset_rad": 0.02,
                "expected_revision": 1
            }
        }),
    )
    .await;

    // Wait until stage has begun, then tighten J7 hardware limits so closed_rad = 0.02 > 0.0 fails reapply_driver_limits() in precheck!
    tokio::time::sleep(Duration::from_millis(50)).await;
    *j7_hw_limit.lock().unwrap() = Some((-6.0, 0.0));

    use futures_util::StreamExt;
    let reply = tokio::time::timeout(Duration::from_secs(5), async {
        loop {
            let text = ws.next().await.unwrap().unwrap().into_text().unwrap();
            let msg: Value = serde_json::from_str(&text).unwrap();
            if msg.get("request_id") == Some(&json!(600_001)) {
                break msg;
            }
        }
    })
    .await
    .unwrap();
    armd::persist::TEST_WRITE_DELAY_MS.store(0, std::sync::atomic::Ordering::Relaxed);

    assert_eq!(reply["type"], "command_reject", "{reply}");
    assert_eq!(reply["code"], "INVALID_COMMAND", "{reply}");

    // Official file on disk, in-memory offset, and revision are ALL untouched
    let disk: Value = serde_json::from_str(&std::fs::read_to_string(&calib_path).unwrap()).unwrap();
    assert_eq!(disk["offset_rad"], 0.0);
    let gc = sup.gripper_calibration_response().await;
    assert_eq!(gc.offset_rad, Some(0.0));
    assert_eq!(gc.revision, 1);
    let files: Vec<String> = std::fs::read_dir(dir.path())
        .unwrap()
        .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
        .collect();
    assert_eq!(files, ["gripper_calib.json"]);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn http_write_routes_and_detection_ws_proxy_permission_matrix() {
    use armd::api::create_router_with_detector;
    use armd::api::proxy::{DetectionWsUpstream, DetectorUpstream};
    use futures_util::{SinkExt, StreamExt};
    use std::sync::Mutex;
    use tokio::io::{AsyncReadExt, AsyncWriteExt};
    use tokio_tungstenite::tungstenite::Message as TungMsg;

    let dir = tempfile::tempdir().unwrap();
    let episodes_dir = dir.path().join("episodes");
    std::fs::create_dir_all(&episodes_dir).unwrap();
    // Seed an existing episode file (<id>.parquet) to verify DELETE while locked has zero disk effect
    let ep_file = episodes_dir.join("ep_existing.parquet");
    std::fs::write(&ep_file, b"PAR1dummy").unwrap();

    // 1. Mock detector HTTP upstream recording all received requests
    let http_seen: Arc<Mutex<Vec<(String, String, String)>>> = Arc::new(Mutex::new(Vec::new()));
    let http_listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let http_addr = http_listener.local_addr().unwrap();
    let http_seen_srv = http_seen.clone();
    tokio::spawn(async move {
        loop {
            let Ok((mut sock, _)) = http_listener.accept().await else { break; };
            let log = http_seen_srv.clone();
            tokio::spawn(async move {
                let mut raw = Vec::new();
                let mut buf = [0u8; 4096];
                let (head, mut body) = loop {
                    let n = sock.read(&mut buf).await.unwrap_or(0);
                    if n == 0 { return; }
                    raw.extend_from_slice(&buf[..n]);
                    if let Some(i) = raw.windows(4).position(|w| w == b"\r\n\r\n") {
                        break (String::from_utf8(raw[..i].to_vec()).unwrap(), raw[i + 4..].to_vec());
                    }
                };
                let len = head
                    .lines()
                    .find_map(|l| l.to_ascii_lowercase().strip_prefix("content-length:").map(|v| v.trim().parse::<usize>().unwrap()))
                    .unwrap_or(0);
                while body.len() < len {
                    let n = sock.read(&mut buf).await.unwrap_or(0);
                    if n == 0 { break; }
                    body.extend_from_slice(&buf[..n]);
                }
                let mut rl = head.lines().next().unwrap().split(' ');
                let method = rl.next().unwrap().to_string();
                let target = rl.next().unwrap().split('?').next().unwrap().to_string();
                let body_s = String::from_utf8_lossy(&body).into_owned();
                log.lock().unwrap().push((method.clone(), target.clone(), body_s));
                let resp_body = match (method.as_str(), target.as_str()) {
                    ("GET", "/api/inference") | ("POST", "/api/inference") => r#"{"enabled":true,"fps":10}"#,
                    ("POST", "/api/trigger") => r#"{"api_version":2,"success":true,"detections":[],"count":0,"markers":[]}"#,
                    ("GET", "/api/mask_focus") | ("POST", "/api/mask_focus") => r#"{"api_version":2,"mask_focus":null}"#,
                    ("GET", "/api/mode") | ("POST", "/api/mode") => r#"{"api_version":2,"mode":"yolo","enabled":true}"#,
                    ("POST", "/api/subscribe") => r#"{"id":"s1","ttl_ms":5000}"#,
                    ("POST", "/api/subscribe/s1/keepalive") => r#"{"ok":true}"#,
                    ("DELETE", "/api/subscribe/s1") => r#"{"deleted":true}"#,
                    ("GET", "/api/subscriptions") => r#"{"subscriptions":[]}"#,
                    _ => r#"{"ok":true}"#,
                };
                let resp = format!(
                    "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
                    resp_body.len(),
                    resp_body
                );
                let _ = sock.write_all(resp.as_bytes()).await;
            });
        }
    });

    // 2. Mock detector WS upstream recording any client frames forwarded to it
    let ws_seen: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let ws_listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let ws_addr = ws_listener.local_addr().unwrap();
    let ws_seen_srv = ws_seen.clone();
    tokio::spawn(async move {
        loop {
            let Ok((stream, _)) = ws_listener.accept().await else { break; };
            let ws_seen = ws_seen_srv.clone();
            tokio::spawn(async move {
                let mut ws = tokio_tungstenite::accept_async(stream).await.unwrap();
                let _ = ws.send(TungMsg::Text(r#"{"frame":1}"#.into())).await;
                let _ = ws.send(TungMsg::Ping(vec![1, 2, 3].into())).await;
                while let Some(Ok(msg)) = ws.next().await {
                    match msg {
                        TungMsg::Text(t) => ws_seen.lock().unwrap().push(format!("TEXT:{t}")),
                        TungMsg::Binary(b) => ws_seen.lock().unwrap().push(format!("BIN:{}", b.len())),
                        TungMsg::Close(_) => break,
                        _ => {}
                    }
                }
            });
        }
    });

    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(TestArm::new())));
    sup.set_episodes_dir(Some(episodes_dir.clone())).await;
    let auth = OperatorAuth::enforced("123456", 1800);
    sup.set_operator_auth_enforced(true).await;

    let det_up = DetectorUpstream::parse(&format!("http://{http_addr}")).unwrap();
    let ws_up = DetectionWsUpstream(format!("ws://{ws_addr}/results"));
    let router = create_router_with_detector(sup.clone(), det_up, auth).layer(axum::Extension(ws_up));
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });

    let client = reqwest::Client::new();
    let base = format!("http://{addr}");

    // Snapshot sandbox state & catalog before locked attempts
    let sb_state_before: Value = client.get(format!("{base}/api/v1/sandbox/state")).send().await.unwrap().json().await.unwrap();
    let sb_cat_before: Value = client.get(format!("{base}/api/v1/sandbox/catalog")).send().await.unwrap().json().await.unwrap();

    // A. Read-only GET routes work without auth
    for path in [
        "/api/v1/auth/status",
        "/api/v1/observation",
        "/api/v1/safety/describe",
        "/api/v1/poses",
        "/api/v1/grasp/setup",
        "/api/v1/dataset/episodes",
        "/api/v1/sandbox/state",
        "/api/v1/sandbox/catalog",
        "/api/v1/detection/inference",
        "/api/v1/detection/mask_focus",
        "/api/v1/vision/mode",
        "/api/v1/vision/subscriptions",
    ] {
        let r = client.get(format!("{base}{path}")).send().await.unwrap();
        assert_eq!(r.status(), 200, "GET {path}");
    }
    let get_http_count = http_seen.lock().unwrap().len();

    // B. POST /api/v1/action/grasp dry_run:true succeeds without auth or lease; non-dry_run is rejected (lease required)
    let dry = client
        .post(format!("{base}/api/v1/action/grasp"))
        .json(&json!({"dry_run": true, "box_norm": [0.45, 0.45, 0.55, 0.55]}))
        .send()
        .await
        .unwrap();
    assert_eq!(dry.status(), 200, "grasp dry_run=true must not require auth or lease");
    let non_dry = client
        .post(format!("{base}/api/v1/action/grasp"))
        .json(&json!({"dry_run": false, "box_norm": [0.45, 0.45, 0.55, 0.55]}))
        .send()
        .await
        .unwrap();
    assert_eq!(non_dry.status(), 409, "grasp dry_run=false must require WS lease");
    let non_dry_body: Value = non_dry.json().await.unwrap();
    assert_eq!(non_dry_body["code"], "LEASE_REQUIRED", "{non_dry_body}");

    // C. When LOCKED, all newly guarded HTTP write/delete routes return 403 OPERATOR_REQUIRED with zero side effects
    let locked_posts: &[(&str, Value)] = &[
        ("/api/v1/sandbox/scenes", json!({"id": "unauth_scene", "files": []})),
        ("/api/v1/sandbox/tools", json!({"id": "unauth_tool", "files": []})),
        ("/api/v1/sandbox/policies", json!({"id": "unauth_pol", "files": []})),
        ("/api/v1/sandbox/activate", json!({"mode": "wasm", "scene": "pcb_suction"})),
        ("/api/v1/sandbox/reset", json!({})),
        ("/api/v1/sandbox/io", json!({"suction": true})),
        ("/api/v1/sandbox/telemetry", json!({"seq": 1})),
        ("/api/v1/dataset/record/start", json!({"episode_id": "ep_unauth"})),
        ("/api/v1/dataset/record/save", json!({})),
        ("/api/v1/dataset/record/discard", json!({})),
        ("/api/v1/detection/inference", json!({"enabled": true})),
        ("/api/v1/detection/trigger", json!({"pipelines": ["yolo"]})),
        ("/api/v1/detection/mask_focus", json!({"enabled": true})),
        ("/api/v1/vision/mode", json!({"mode": "yolo", "enabled": true})),
        ("/api/v1/vision/subscribe", json!({"client": "unauth"})),
        ("/api/v1/vision/subscribe/s1/keepalive", json!({})),
    ];
    for (path, body) in locked_posts {
        let r = client.post(format!("{base}{path}")).json(body).send().await.unwrap();
        assert_eq!(r.status(), 403, "locked POST {path}");
        let v: Value = r.json().await.unwrap();
        assert_eq!(v["code"], "OPERATOR_REQUIRED", "POST {path}: {v}");
    }
    for path in ["/api/v1/dataset/episodes/ep_existing", "/api/v1/vision/subscribe/s1"] {
        let r = client.delete(format!("{base}{path}")).send().await.unwrap();
        assert_eq!(r.status(), 403, "locked DELETE {path}");
        let v: Value = r.json().await.unwrap();
        assert_eq!(v["code"], "OPERATOR_REQUIRED", "DELETE {path}: {v}");
    }

    // Verify ZERO side effects from locked calls
    let sb_state_after: Value = client.get(format!("{base}/api/v1/sandbox/state")).send().await.unwrap().json().await.unwrap();
    let sb_cat_after: Value = client.get(format!("{base}/api/v1/sandbox/catalog")).send().await.unwrap().json().await.unwrap();
    assert_eq!(sb_state_after["mode"], sb_state_before["mode"]);
    assert_eq!(sb_state_after["telemetry"], sb_state_before["telemetry"]);
    assert_eq!(sb_cat_after, sb_cat_before);
    assert!(ep_file.exists(), "episode must survive locked DELETE");
    assert_eq!(sup.inner().read().await.recorder.state(), &armd::recording::EpisodeState::Idle);
    // Zero upstream POST/DELETE requests reached detector upstream
    assert_eq!(
        http_seen.lock().unwrap().len(),
        get_http_count,
        "locked POST/DELETE reached upstream: {:?}",
        *http_seen.lock().unwrap()
    );

    // D. Detection WS proxy (/api/v1/detection/ws and /results): read-only subscription works, client write frames NEVER reach upstream
    for ws_path in ["/api/v1/detection/ws", "/results"] {
        let (mut cli_ws, _) = connect_async(format!("ws://{addr}{ws_path}")).await.unwrap();
        let first = tokio::time::timeout(Duration::from_secs(2), cli_ws.next()).await.unwrap().unwrap().unwrap();
        assert_eq!(first.into_text().unwrap(), r#"{"frame":1}"#);
        cli_ws.send(TungMsg::Text(r#"{"cmd":"set_mode","mode":"off"}"#.into())).await.unwrap();
        cli_ws.send(TungMsg::Binary(vec![0xDE, 0xAD, 0xBE, 0xEF].into())).await.unwrap();
        cli_ws.send(TungMsg::Ping(vec![9].into())).await.unwrap();
        tokio::time::sleep(Duration::from_millis(60)).await;
        let _ = cli_ws.close(None).await;
    }
    tokio::time::sleep(Duration::from_millis(50)).await;
    assert!(
        ws_seen.lock().unwrap().is_empty(),
        "read-only detection WS proxy forwarded client write frames: {:?}",
        *ws_seen.lock().unwrap()
    );

    // E. Unlock as Operator and verify the same write/delete endpoints succeed cleanly
    let unlock: Value = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .json(&json!({"pin": "123456"}))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let tok = unlock["token"].as_str().unwrap().to_string();
    let auth_hdr = format!("Bearer {tok}");

    for (path, body) in [
        ("/api/v1/sandbox/reset", json!({})),
        ("/api/v1/sandbox/io", json!({"suction": true})),
        ("/api/v1/sandbox/telemetry", json!({"seq": 42})),
        ("/api/v1/detection/inference", json!({"enabled": true})),
        ("/api/v1/detection/trigger", json!({"pipelines": ["yolo"]})),
        ("/api/v1/detection/mask_focus", json!({"enabled": true})),
        ("/api/v1/vision/mode", json!({"mode": "yolo", "enabled": true})),
        ("/api/v1/vision/subscribe", json!({"client": "op"})),
        ("/api/v1/vision/subscribe/s1/keepalive", json!({})),
        ("/api/v1/dataset/record/start", json!({"episode_id": "ep_op"})),
        ("/api/v1/dataset/record/discard", json!({})),
    ] {
        let r = client
            .post(format!("{base}{path}"))
            .header(AUTHORIZATION.as_str(), &auth_hdr)
            .json(&body)
            .send()
            .await
            .unwrap();
        assert_eq!(r.status(), 200, "unlocked POST {path}");
    }
    for path in ["/api/v1/vision/subscribe/s1", "/api/v1/dataset/episodes/ep_existing"] {
        let r = client
            .delete(format!("{base}{path}"))
            .header(AUTHORIZATION.as_str(), &auth_hdr)
            .send()
            .await
            .unwrap();
        assert_eq!(r.status(), 200, "unlocked DELETE {path}");
    }
    assert!(!ep_file.exists(), "unlocked DELETE removed episode");

    // F. POST /api/v1/safety/e_stop remains unauthenticated and lease-free
    let es = client.post(format!("{base}/api/v1/safety/e_stop")).send().await.unwrap();
    assert_eq!(es.status(), 200);
    assert_eq!(es.json::<Value>().await.unwrap()["safety_state"], "EStop");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn poses_grasp_setup_and_calibration_revision_and_atomic_persist() {
    let _serial = WRITE_DELAY_SERIAL.lock().await;
    let dir = tempfile::tempdir().unwrap();
    let keepout_path = dir.path().join("keepout.json");
    let collision_path = dir.path().join("collision.json");
    let calib_path = dir.path().join("gripper_calib.json");
    let poses_path = dir.path().join("poses.json");
    let setup_path = dir.path().join("grasp_setup.json");

    // 1. Calibration startup invalid-file reset uses StagedWrite and bumps grasp_setup_revision
    std::fs::write(&setup_path, r#"{"table_z_m": 0.05, "camera_tilt_deg": -20.0}"#).unwrap();
    let bad_cal = dir.path().join("bad_calibration.json");
    std::fs::write(&bad_cal, r#"{"camera_mount":{"q_end_cam":[9,0,0,0],"offset_m":[0,0,0]}}"#).unwrap();
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(TestArm::new())));
    sup.tick(0.02).await;
    armd::grasp::load_into(&sup, &setup_path).await;
    armd::calibration::load_into(&sup, &bad_cal, Some(&setup_path)).await.unwrap();
    // Startup reset wrote profile defaults back to setup_path and incremented grasp_setup_revision (1 -> 2)
    assert_eq!(sup.inner().read().await.grasp_setup_revision, 2);
    assert!(setup_path.exists());

    armd::keepout::load_into(&sup, &keepout_path).await.unwrap();
    armd::collision::load_into(&sup, &collision_path).await;
    armd::gripper_calib::load_into(&sup, &calib_path).await;
    {
        let mut inner = sup.inner().write().await;
        inner.poses.path = Some(poses_path.clone());
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
    }

    let vision_state = Arc::new(std::sync::Mutex::new((
        1u64,
        json!({"fx": 824.0, "fy": 824.0, "cx": 320.0, "cy": 180.0, "k1": 0.0, "k2": 0.0, "p1": 0.0, "p2": 0.0, "k3": 0.0, "width": 640, "height": 360, "marker_size": 0.05}),
    )));
    let v_get = vision_state.clone();
    let v_put = vision_state.clone();
    let vision_listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let vision_addr = vision_listener.local_addr().unwrap();
    let vision_app = axum::Router::new().route(
        "/api/calibration/intrinsics",
        axum::routing::get(move || {
            let v_get = v_get.clone();
            async move {
                let st = v_get.lock().unwrap();
                let mut obj = st.1.as_object().unwrap().clone();
                obj.insert("revision".into(), json!(st.0));
                obj.insert("api_version".into(), json!(2));
                axum::Json(Value::Object(obj))
            }
        })
        .put(move |axum::Json(body): axum::Json<Value>| {
            let v_put = v_put.clone();
            async move {
                use axum::response::IntoResponse;
                let mut st = v_put.lock().unwrap();
                if body.get("expected_revision").and_then(Value::as_u64) != Some(st.0) {
                    return (
                        axum::http::StatusCode::CONFLICT,
                        axum::Json(json!({"error": "REVISION_MISMATCH", "revision": st.0})),
                    )
                        .into_response();
                }
                st.0 += 1;
                let obj = st.1.as_object_mut().unwrap();
                for k in ["fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3", "width", "height", "marker_size"] {
                    if let Some(v) = body.get(k) {
                        obj.insert(k.into(), v.clone());
                    }
                }
                let mut out = obj.clone();
                out.insert("revision".into(), json!(st.0));
                out.insert("api_version".into(), json!(2));
                axum::Json(Value::Object(out)).into_response()
            }
        }),
    );
    tokio::spawn(async move { axum::serve(vision_listener, vision_app).await.unwrap(); });

    let auth = OperatorAuth::enforced("123456", 1800);
    sup.set_operator_auth_enforced(true).await;
    let det_up = armd::api::proxy::DetectorUpstream::parse(&format!("http://{vision_addr}")).unwrap();
    let grasp_deps = armd::grasp::GraspDeps::http(det_up.clone(), Some(setup_path.clone()));
    let cal_deps = armd::calibration::CalibrationDeps::new(
        dir.path().join("calibration.json"),
        Some(setup_path.clone()),
        det_up.clone(),
    );
    let router = armd::api::create_router_calibration(
        sup.clone(),
        det_up,
        grasp_deps,
        armd::diagnostics::DiagnosticsState::global(),
        cal_deps,
        auth,
    );
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });
    let client = reqwest::Client::new();
    let base = format!("http://{addr}");

    // GET endpoints return latest revision
    let g_poses: Value = client.get(format!("{base}/api/v1/poses")).send().await.unwrap().json().await.unwrap();
    assert_eq!(g_poses["revision"], 1);
    let g_grasp: Value = client.get(format!("{base}/api/v1/grasp/setup")).send().await.unwrap().json().await.unwrap();
    assert_eq!(g_grasp["revision"], 2);

    // Unlock and acquire lease
    let unlock: Value = client.post(format!("{base}/api/v1/auth/unlock")).json(&json!({"pin": "123456"})).send().await.unwrap().json().await.unwrap();
    let tok = unlock["token"].as_str().unwrap().to_string();
    let mut ws = connect_ws_with_header(addr, "Authorization", Some(&format!("Bearer {tok}"))).await;
    let g = acquire(&mut ws, "op", false).await["generation"].as_u64().unwrap();
    let epoch = sup.get_observation().await.epoch_id;

    // 2. When auth.is_enforced(), omitting expected_revision MUST fail with 400 INVALID_COMMAND on all 6 privileged writes
    for (path, body) in [
        ("/api/v1/workspace/keepout", json!({"boxes": []})),
        ("/api/v1/safety/collision", json!({"enabled": false})),
        ("/api/v1/gripper/calibration", json!({"offset_rad": 0.01})),
        ("/api/v1/poses/save", json!({"name": "scan"})),
        ("/api/v1/poses/delete", json!({"name": "scan"})),
        ("/api/v1/grasp/setup", json!({"table_z_m": 0.02})),
    ] {
        let rej = common::rpc(&mut ws, g, epoch, path, body).await;
        assert_eq!(rej["type"], "command_reject", "{path}: {rej}");
        assert_eq!(rej["status"], 400, "{path}: {rej}");
        assert_eq!(rej["code"], "INVALID_COMMAND", "{path}: {rej}");
    }

    // 3. Stale expected_revision -> 409 POLICY_REVISION_MISMATCH preserving current_revision
    let stale_p = common::rpc(&mut ws, g, epoch, "/api/v1/poses/save", json!({"name": "scan", "expected_revision": 99})).await;
    assert_eq!(stale_p["status"], 409);
    assert_eq!(stale_p["code"], "POLICY_REVISION_MISMATCH");
    assert_eq!(stale_p["current_revision"], 1);

    let stale_g = common::rpc(&mut ws, g, epoch, "/api/v1/grasp/setup", json!({"table_z_m": 0.02, "expected_revision": 99})).await;
    assert_eq!(stale_g["status"], 409);
    assert_eq!(stale_g["code"], "POLICY_REVISION_MISMATCH");
    assert_eq!(stale_g["current_revision"], 2);

    // 4. Valid poses/save, poses/delete, grasp/setup bump revisions and persist atomically
    let ack_ps = common::rpc(&mut ws, g, epoch, "/api/v1/poses/save", json!({"name": "scan", "expected_revision": 1})).await;
    assert_eq!(ack_ps["type"], "command_ack", "{ack_ps}");
    assert_eq!(ack_ps["revision"], 2);
    assert_eq!(ack_ps["persisted"], true);
    let g_poses2: Value = client.get(format!("{base}/api/v1/poses")).send().await.unwrap().json().await.unwrap();
    assert_eq!(g_poses2["revision"], 2);
    assert_eq!(g_poses2["poses"].as_array().unwrap().len(), 1);

    let ack_pd = common::rpc(&mut ws, g, epoch, "/api/v1/poses/delete", json!({"name": "scan", "expected_revision": 2})).await;
    assert_eq!(ack_pd["type"], "command_ack", "{ack_pd}");
    assert_eq!(ack_pd["revision"], 3);
    assert_eq!(ack_pd["persisted"], true);

    let ack_gs = common::rpc(&mut ws, g, epoch, "/api/v1/grasp/setup", json!({"table_z_m": 0.03, "expected_revision": 2})).await;
    assert_eq!(ack_gs["type"], "command_ack", "{ack_gs}");
    assert_eq!(ack_gs["revision"], 3);
    assert_eq!(ack_gs["persisted"], true);
    let g_grasp3: Value = client.get(format!("{base}/api/v1/grasp/setup")).send().await.unwrap().json().await.unwrap();
    assert_eq!(g_grasp3["revision"], 3);
    assert!((g_grasp3["setup"]["table_z_m"].as_f64().unwrap() - 0.03).abs() < 1e-6);

    // 5. PERSIST_FAILED before rename preserves disk, memory, and revision (blocker on poses_path + unreachable setup_path)
    let blocker = dir.path().join("blocker_file");
    std::fs::write(&blocker, b"x").unwrap();
    {
        let mut inner = sup.inner().write().await;
        inner.poses.path = Some(blocker.join("poses.json"));
    }
    let fail_ps = common::rpc(&mut ws, g, epoch, "/api/v1/poses/save", json!({"name": "bad", "expected_revision": 3})).await;
    assert_eq!(fail_ps["code"], "PERSIST_FAILED");
    let g_poses_after_fail: Value = client.get(format!("{base}/api/v1/poses")).send().await.unwrap().json().await.unwrap();
    assert_eq!(g_poses_after_fail["revision"], 3);
    assert_eq!(g_poses_after_fail["poses"].as_array().unwrap().len(), 0);

    // For grasp/setup PERSIST_FAILED: spawn an auth router whose GraspDeps.setup_path is under blocker
    let sup_gs_fail = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(TestArm::new())));
    sup_gs_fail.set_operator_auth_enforced(true).await;
    let auth_gs = OperatorAuth::enforced("123456", 1800);
    let det_gs = armd::api::proxy::DetectorUpstream::from_env();
    let router_gs = armd::api::create_router_with(
        sup_gs_fail.clone(),
        det_gs.clone(),
        armd::grasp::GraspDeps::http(det_gs, Some(blocker.join("grasp_setup.json"))),
        auth_gs,
    );
    let l_gs = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr_gs = l_gs.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(l_gs, router_gs).await.unwrap(); });
    let tok_gs = client.post(format!("http://{addr_gs}/api/v1/auth/unlock")).json(&json!({"pin": "123456"})).send().await.unwrap().json::<Value>().await.unwrap()["token"].as_str().unwrap().to_string();
    let mut ws_gs = connect_ws_with_header(addr_gs, "Authorization", Some(&format!("Bearer {tok_gs}"))).await;
    let g_gs = acquire(&mut ws_gs, "op_gs", false).await["generation"].as_u64().unwrap();
    let ep_gs = sup_gs_fail.get_observation().await.epoch_id;
    let fail_gs = common::rpc(&mut ws_gs, g_gs, ep_gs, "/api/v1/grasp/setup", json!({"table_z_m": 0.09, "expected_revision": 1})).await;
    assert_eq!(fail_gs["code"], "PERSIST_FAILED", "{fail_gs}");
    let g_grasp_after_fail: Value = client.get(format!("http://{addr_gs}/api/v1/grasp/setup")).send().await.unwrap().json().await.unwrap();
    assert_eq!(g_grasp_after_fail["revision"], 1);

    // Restore writable poses path for post-stage lock re-checks
    {
        let mut inner = sup.inner().write().await;
        inner.poses.path = Some(poses_path.clone());
    }

    // 6. Post-stage re-check: auth/lock during poses/save staging -> OPERATOR_REQUIRED, zero mutation, zero temp files
    use futures_util::StreamExt;
    armd::persist::TEST_WRITE_DELAY_MS.store(250, std::sync::atomic::Ordering::Relaxed);
    common::send(&mut ws, json!({
        "type": "command", "request_id": 700_001, "generation": g, "epoch_id": epoch,
        "path": "/api/v1/poses/save", "body": {"name": "during_lock", "expected_revision": 3}
    })).await;
    tokio::time::sleep(Duration::from_millis(50)).await;
    let lock_res = client.post(format!("{base}/api/v1/auth/lock")).header(AUTHORIZATION.as_str(), format!("Bearer {tok}")).send().await.unwrap();
    assert_eq!(lock_res.status(), 200);
    let rej_lock = tokio::time::timeout(Duration::from_secs(5), async {
        loop {
            let msg: Value = serde_json::from_str(&ws.next().await.unwrap().unwrap().into_text().unwrap()).unwrap();
            if msg.get("request_id") == Some(&json!(700_001)) { break msg; }
        }
    }).await.unwrap();
    armd::persist::TEST_WRITE_DELAY_MS.store(0, std::sync::atomic::Ordering::Relaxed);
    assert_eq!(rej_lock["code"], "OPERATOR_REQUIRED", "{rej_lock}");
    let g_poses_lock: Value = client.get(format!("{base}/api/v1/poses")).send().await.unwrap().json().await.unwrap();
    assert_eq!(g_poses_lock["revision"], 3);
    assert_eq!(g_poses_lock["poses"].as_array().unwrap().len(), 0);

    // Re-unlock & test post-stage lease release during grasp/setup staging -> LEASE_LOST
    let tok2 = client.post(format!("{base}/api/v1/auth/unlock")).json(&json!({"pin": "123456"})).send().await.unwrap().json::<Value>().await.unwrap()["token"].as_str().unwrap().to_string();
    let mut ws2 = connect_ws_with_header(addr, "Authorization", Some(&format!("Bearer {tok2}"))).await;
    let g2 = acquire(&mut ws2, "op2", true).await["generation"].as_u64().unwrap();
    armd::persist::TEST_WRITE_DELAY_MS.store(250, std::sync::atomic::Ordering::Relaxed);
    common::send(&mut ws2, json!({
        "type": "command", "request_id": 700_002, "generation": g2, "epoch_id": epoch,
        "path": "/api/v1/grasp/setup", "body": {"table_z_m": 0.07, "expected_revision": 3}
    })).await;
    tokio::time::sleep(Duration::from_millis(50)).await;
    let _ = common::ws_reply(&mut ws2, json!({"type": "release_lease", "generation": g2}), &["lease_released"]).await;
    let rej_lease = tokio::time::timeout(Duration::from_secs(5), async {
        loop {
            let msg: Value = serde_json::from_str(&ws2.next().await.unwrap().unwrap().into_text().unwrap()).unwrap();
            if msg.get("request_id") == Some(&json!(700_002)) { break msg; }
        }
    }).await.unwrap();
    armd::persist::TEST_WRITE_DELAY_MS.store(0, std::sync::atomic::Ordering::Relaxed);
    assert_eq!(rej_lease["code"], "LEASE_LOST", "{rej_lease}");
    let g_grasp_lease: Value = client.get(format!("{base}/api/v1/grasp/setup")).send().await.unwrap().json().await.unwrap();
    assert_eq!(g_grasp_lease["revision"], 3);
    assert!((g_grasp_lease["setup"]["table_z_m"].as_f64().unwrap() - 0.03).abs() < 1e-6);

    // 7. Calibration apply & restore via WS increment grasp_setup_revision and write atomically
    let mut g3_ws = connect_ws_with_header(addr, "Authorization", Some(&format!("Bearer {tok2}"))).await;
    let granted3 = acquire(&mut g3_ws, "op3", true).await;
    assert_eq!(granted3["type"], "lease_granted", "{granted3}");
    let g3 = granted3["generation"].as_u64().unwrap();
    let cur_epoch = sup.get_observation().await.epoch_id;
    let q_tilt = armd::kinematics::r_to_quat(&armd::chain::ArmModel::REBOT_DEV_ARM.cam_mount(false, 0.0, -15.0));
    let apply_body = json!({
        "intrinsics": {"fx": 831.5, "fy": 829.0, "cx": 318.2, "cy": 183.4, "k1": 0.05, "k2": -0.1, "p1": 0.0, "p2": 0.0, "k3": 0.0, "width": 640, "height": 360, "marker_size": 0.05},
        "camera_mount": {"offset_m": [-0.0728, 0.0012, 0.0655], "q_end_cam": q_tilt},
        "table_z_m": 0.015,
        "dry_run": false
    });
    let apply_out = common::rpc(&mut g3_ws, g3, cur_epoch, "/api/v1/calibration/apply", apply_body).await;
    assert_eq!(apply_out["type"], "command_ack", "{apply_out}");
    assert_eq!(apply_out["grasp_revision"], 4, "{apply_out}");
    let backup_name = std::path::Path::new(apply_out["backup"].as_str().unwrap())
        .file_name().unwrap().to_string_lossy().into_owned();
    let g_grasp4: Value = client.get(format!("{base}/api/v1/grasp/setup")).send().await.unwrap().json().await.unwrap();
    assert_eq!(g_grasp4["revision"], 4);

    let restore_out = common::rpc(&mut g3_ws, g3, cur_epoch, "/api/v1/calibration/restore", json!({"backup": backup_name})).await;
    assert_eq!(restore_out["type"], "command_ack", "{restore_out}");
    assert_eq!(restore_out["grasp_revision"], 5, "{restore_out}");
    let g_grasp5: Value = client.get(format!("{base}/api/v1/grasp/setup")).send().await.unwrap().json().await.unwrap();
    assert_eq!(g_grasp5["revision"], 5);
    assert!((g_grasp5["setup"]["table_z_m"].as_f64().unwrap() - 0.03).abs() < 1e-6);

    // Post-stage epoch re-check: trigger EStop during collision staging -> EPOCH_MISMATCH, zero mutation
    armd::persist::TEST_WRITE_DELAY_MS.store(250, std::sync::atomic::Ordering::Relaxed);
    common::send(&mut g3_ws, json!({
        "type": "command", "request_id": 700_003, "generation": g3, "epoch_id": cur_epoch,
        "path": "/api/v1/safety/collision", "body": {"enabled": true, "expected_revision": 1}
    })).await;
    tokio::time::sleep(Duration::from_millis(50)).await;
    let es = client.post(format!("{base}/api/v1/safety/e_stop")).send().await.unwrap();
    assert_eq!(es.status(), 200);
    let rej_epoch = tokio::time::timeout(Duration::from_secs(5), async {
        loop {
            let msg: Value = serde_json::from_str(&g3_ws.next().await.unwrap().unwrap().into_text().unwrap()).unwrap();
            if msg.get("request_id") == Some(&json!(700_003)) { break msg; }
        }
    }).await.unwrap();
    armd::persist::TEST_WRITE_DELAY_MS.store(0, std::sync::atomic::Ordering::Relaxed);
    assert_eq!(rej_epoch["code"], "EPOCH_MISMATCH", "{rej_epoch}");
    let col_after: Value = client.get(format!("{base}/api/v1/safety/collision")).send().await.unwrap().json().await.unwrap();
    assert_eq!(col_after["enabled"], false);
    assert_eq!(col_after["revision"], 1);

    // 8. When !auth.is_enforced() (mock/sim), omitting expected_revision remains backward-compatible
    let sup_sim = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(TestArm::new())));
    {
        let mut inner = sup_sim.inner().write().await;
        inner.poses.path = Some(dir.path().join("sim_poses.json"));
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
    }
    let det_sim = armd::api::proxy::DetectorUpstream::from_env();
    let router_sim = armd::api::create_router_with(
        sup_sim.clone(),
        det_sim.clone(),
        armd::grasp::GraspDeps::http(det_sim, Some(dir.path().join("sim_setup.json"))),
        OperatorAuth::disabled(),
    );
    let l_sim = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr_sim = l_sim.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(l_sim, router_sim).await.unwrap(); });
    let mut ws_sim = connect_ws_with_header(addr_sim, "User-Agent", Some("sim")).await;
    let g_sim = acquire(&mut ws_sim, "sim", false).await["generation"].as_u64().unwrap();
    let ep_sim = sup_sim.get_observation().await.epoch_id;
    let sim_ps = common::rpc(&mut ws_sim, g_sim, ep_sim, "/api/v1/poses/save", json!({"name": "p1"})).await;
    assert_eq!(sim_ps["type"], "command_ack", "{sim_ps}");
    assert_eq!(sim_ps["revision"], 2);
    let sim_gs = common::rpc(&mut ws_sim, g_sim, ep_sim, "/api/v1/grasp/setup", json!({"table_z_m": 0.04})).await;
    assert_eq!(sim_gs["type"], "command_ack", "{sim_gs}");
    assert_eq!(sim_gs["revision"], 2);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn delayed_zero_check_and_calibration_recheck_operator_session_and_lease_expiry() {
    let _serial = WRITE_DELAY_SERIAL.lock().await;
    let dir = tempfile::tempdir().unwrap();
    let calib_path = dir.path().join("gripper_calib.json");
    let setup_path = dir.path().join("grasp_setup.json");
    std::fs::write(&calib_path, serde_json::to_string_pretty(&json!({"offset_rad": 0.01})).unwrap()).unwrap();

    // A. Locking operator session while gripper_zero_check(store_offset=true) is staging -> OPERATOR_REQUIRED, zero mutation
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    armd::gripper_calib::load_into(&sup, &calib_path).await;
    {
        let mut inner = sup.inner().write().await;
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        inner.measured_positions[6] = -0.15;
        inner.commanded_positions[6] = -0.15;
        inner.target_positions[6] = -0.15;
        inner.set_mock_gripper_stall_offset(Some(0.05));
    }

    let vision_delay = Arc::new(std::sync::atomic::AtomicU64::new(0));
    let vd = vision_delay.clone();
    let vision_state = Arc::new(std::sync::Mutex::new((
        1u64,
        json!({"fx": 824.0, "fy": 824.0, "cx": 320.0, "cy": 180.0, "k1": 0.0, "k2": 0.0, "p1": 0.0, "p2": 0.0, "k3": 0.0, "width": 640, "height": 360, "marker_size": 0.05}),
    )));
    let v_get = vision_state.clone();
    let v_put = vision_state.clone();
    let vision_listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let vision_addr = vision_listener.local_addr().unwrap();
    let vision_app = axum::Router::new().route(
        "/api/calibration/intrinsics",
        axum::routing::get(move || {
            let v_get = v_get.clone();
            async move {
                let st = v_get.lock().unwrap();
                let mut obj = st.1.as_object().unwrap().clone();
                obj.insert("revision".into(), json!(st.0));
                obj.insert("api_version".into(), json!(2));
                axum::Json(Value::Object(obj))
            }
        })
        .put(move |axum::Json(body): axum::Json<Value>| {
            let v_put = v_put.clone();
            let vd = vd.clone();
            async move {
                use axum::response::IntoResponse;
                let ms = vd.load(std::sync::atomic::Ordering::Relaxed);
                if ms > 0 {
                    tokio::time::sleep(Duration::from_millis(ms)).await;
                }
                let mut st = v_put.lock().unwrap();
                if body.get("expected_revision").and_then(Value::as_u64) != Some(st.0) {
                    return (
                        axum::http::StatusCode::CONFLICT,
                        axum::Json(json!({"error": "REVISION_MISMATCH", "revision": st.0})),
                    )
                        .into_response();
                }
                st.0 += 1;
                let obj = st.1.as_object_mut().unwrap();
                for k in ["fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3", "width", "height", "marker_size"] {
                    if let Some(v) = body.get(k) {
                        obj.insert(k.into(), v.clone());
                    }
                }
                let mut out = obj.clone();
                out.insert("revision".into(), json!(st.0));
                out.insert("api_version".into(), json!(2));
                axum::Json(Value::Object(out)).into_response()
            }
        }),
    );
    tokio::spawn(async move { axum::serve(vision_listener, vision_app).await.unwrap(); });

    let auth = OperatorAuth::enforced("123456", 1800);
    sup.set_operator_auth_enforced(true).await;
    let det_up = armd::api::proxy::DetectorUpstream::parse(&format!("http://{vision_addr}")).unwrap();
    let grasp_deps = armd::grasp::GraspDeps::http(det_up.clone(), Some(setup_path.clone()));
    let cal_deps = armd::calibration::CalibrationDeps::new(
        dir.path().join("calibration.json"),
        Some(setup_path.clone()),
        det_up.clone(),
    );
    let router = armd::api::create_router_calibration(
        sup.clone(),
        det_up,
        grasp_deps,
        armd::diagnostics::DiagnosticsState::global(),
        cal_deps,
        auth,
    );
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });
    let ticker = sup.clone();
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(20));
        loop {
            interval.tick().await;
            ticker.tick(0.02).await;
        }
    });

    let client = reqwest::Client::new();
    let base = format!("http://{addr}");
    let tok = client.post(format!("{base}/api/v1/auth/unlock")).json(&json!({"pin": "123456"})).send().await.unwrap().json::<Value>().await.unwrap()["token"].as_str().unwrap().to_string();
    let mut ws = connect_ws_with_header(addr, "Authorization", Some(&format!("Bearer {tok}"))).await;
    let g = acquire(&mut ws, "op_zc", false).await["generation"].as_u64().unwrap();
    let epoch = sup.get_observation().await.epoch_id;
    let ack = common::rpc(&mut ws, g, epoch, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");

    armd::persist::TEST_WRITE_DELAY_MS.store(250, std::sync::atomic::Ordering::Relaxed);
    let z_ack = common::rpc(&mut ws, g, epoch, "/api/v1/action/gripper_zero_check", json!({"threshold_rad": 0.1, "store_offset": true})).await;
    assert_eq!(z_ack["type"], "command_ack", "{z_ack}");
    let aid = z_ack["action_id"].as_u64().unwrap();

    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    loop {
        common::send(&mut ws, json!({"type": "heartbeat", "generation": g, "epoch_id": epoch})).await;
        if sup.inner().read().await.zero_check_persisting == Some(aid) {
            break;
        }
        assert!(std::time::Instant::now() < deadline, "zero_check did not enter persisting state");
        tokio::time::sleep(Duration::from_millis(20)).await;
    }

    // Lock operator session during staging -> publish MUST be aborted with OPERATOR_REQUIRED
    let lock_res = client.post(format!("{base}/api/v1/auth/lock")).header(AUTHORIZATION.as_str(), format!("Bearer {tok}")).send().await.unwrap();
    assert_eq!(lock_res.status(), 200);
    tokio::time::sleep(Duration::from_millis(350)).await;
    armd::persist::TEST_WRITE_DELAY_MS.store(0, std::sync::atomic::Ordering::Relaxed);

    let act = sup.get_observation().await.action.unwrap();
    assert_eq!(act.id, aid);
    assert_eq!(act.state, "failed", "{act:?}");
    assert_eq!(act.code.as_deref(), Some("OPERATOR_REQUIRED"), "{act:?}");
    let disk: Value = serde_json::from_str(&std::fs::read_to_string(&calib_path).unwrap()).unwrap();
    assert!((disk["offset_rad"].as_f64().unwrap() - 0.01).abs() < 1e-6);
    assert_eq!(sup.gripper_calibration_response().await.revision, 1);

    // B. Locking operator session while calibration/apply is running -> OPERATOR_REQUIRED before local disk writes
    let tok2 = client.post(format!("{base}/api/v1/auth/unlock")).json(&json!({"pin": "123456"})).send().await.unwrap().json::<Value>().await.unwrap()["token"].as_str().unwrap().to_string();
    let mut ws2 = connect_ws_with_header(addr, "Authorization", Some(&format!("Bearer {tok2}"))).await;
    let g2 = acquire(&mut ws2, "op_cal", true).await["generation"].as_u64().unwrap();
    let ep2 = sup.get_observation().await.epoch_id;
    vision_delay.store(250, std::sync::atomic::Ordering::Relaxed);
    let q_tilt = armd::kinematics::r_to_quat(&armd::chain::ArmModel::REBOT_DEV_ARM.cam_mount(false, 0.0, -15.0));
    common::send(&mut ws2, json!({
        "type": "command", "request_id": 800_001, "generation": g2, "epoch_id": ep2,
        "path": "/api/v1/calibration/apply",
        "body": {
            "intrinsics": {"fx": 831.5, "fy": 829.0, "cx": 318.2, "cy": 183.4, "k1": 0.05, "k2": -0.1, "p1": 0.0, "p2": 0.0, "k3": 0.0, "width": 640, "height": 360, "marker_size": 0.05},
            "camera_mount": {"offset_m": [-0.0728, 0.0012, 0.0655], "q_end_cam": q_tilt},
            "table_z_m": 0.015,
            "dry_run": false
        }
    })).await;
    tokio::time::sleep(Duration::from_millis(60)).await;
    let lock_res2 = client.post(format!("{base}/api/v1/auth/lock")).header(AUTHORIZATION.as_str(), format!("Bearer {tok2}")).send().await.unwrap();
    assert_eq!(lock_res2.status(), 200);
    use futures_util::StreamExt;
    let cal_rej = tokio::time::timeout(Duration::from_secs(5), async {
        loop {
            let msg: Value = serde_json::from_str(&ws2.next().await.unwrap().unwrap().into_text().unwrap()).unwrap();
            if msg.get("request_id") == Some(&json!(800_001)) { break msg; }
        }
    }).await.unwrap();
    vision_delay.store(0, std::sync::atomic::Ordering::Relaxed);
    assert_eq!(cal_rej["type"], "command_reject", "{cal_rej}");
    assert_eq!(cal_rej["code"], "OPERATOR_REQUIRED", "{cal_rej}");
    assert!(!dir.path().join("calibration.json").exists());
    assert_eq!(sup.inner().read().await.grasp_setup_revision, 1);

    // C. Lease expiration during zero_check staging (even if active_lease has not yet been swept by a tick) -> LEASE_LOST
    let mut cfg_short = ArmConfig::rebot_dm();
    cfg_short.lease_timeout_ms = 200;
    let sup_exp = Arc::new(ArmSupervisor::new(cfg_short));
    armd::gripper_calib::load_into(&sup_exp, &calib_path).await;
    let aid_exp = {
        let mut inner = sup_exp.inner().write().await;
        inner.set_torque(true).unwrap();
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        inner.measured_positions[6] = -0.15;
        inner.commanded_positions[6] = -0.15;
        inner.target_positions[6] = -0.15;
        inner.set_mock_gripper_stall_offset(Some(0.05));
        inner.acquire_lease("conn_exp").unwrap();
        let ep = inner.safety.epoch_id();
        let (id, _) = inner.start_gripper_zero_check(&json!({"threshold_rad": 0.1, "store_offset": true}), ep).unwrap();
        id
    };
    // Tick until zero_check reaches persisting, then wait past 200ms lease_timeout without renewing lease
    armd::persist::TEST_WRITE_DELAY_MS.store(300, std::sync::atomic::Ordering::Relaxed);
    for _ in 0..100 {
        sup_exp.tick(0.02).await;
        if sup_exp.inner().read().await.zero_check_persisting == Some(aid_exp) {
            break;
        }
    }
    assert_eq!(sup_exp.inner().read().await.zero_check_persisting, Some(aid_exp));
    tokio::time::sleep(Duration::from_millis(400)).await;
    armd::persist::TEST_WRITE_DELAY_MS.store(0, std::sync::atomic::Ordering::Relaxed);
    let act_exp = sup_exp.get_observation().await.action.unwrap();
    assert_eq!(act_exp.id, aid_exp);
    assert_eq!(act_exp.state, "cancelled", "{act_exp:?}");
    assert_eq!(act_exp.code.as_deref(), Some("LEASE_LOST"), "{act_exp:?}");
    let disk_exp: Value = serde_json::from_str(&std::fs::read_to_string(&calib_path).unwrap()).unwrap();
    assert!((disk_exp["offset_rad"].as_f64().unwrap() - 0.01).abs() < 1e-6);
    assert_eq!(sup_exp.gripper_calibration_response().await.revision, 1);
}


/// `{"type":"auth"}` hands an unlock token to an open socket: the lease stays, operator commands
/// pass; `token: null` drops back to program; an unknown token gives role program.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn ws_auth_message_switches_the_operator_session_without_reconnecting() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let auth = OperatorAuth::enforced("123456", 1800);
    let addr = spawn_auth_server(sup.clone(), auth).await;
    let mut ws = connect_ws_with_header(addr, "Authorization", None).await;
    let granted = acquire(&mut ws, "cockpit", false).await;
    let gen = granted["generation"].as_u64().unwrap();
    let epoch = sup.get_observation().await.epoch_id;
    let rej = common::rpc(&mut ws, gen, epoch, "/api/v1/workspace/keepout", json!({"boxes": []})).await;
    assert_eq!(rej["code"], "OPERATOR_REQUIRED", "{rej}");

    let unlock: Value = reqwest::Client::new().post(format!("http://{addr}/api/v1/auth/unlock"))
        .json(&json!({"pin": "123456"})).send().await.unwrap().json().await.unwrap();
    let tok = unlock["token"].as_str().unwrap();
    let ack = common::ws_reply(&mut ws, json!({"type": "auth", "seq": 7, "token": tok}), &["auth_ack"]).await;
    assert_eq!(ack["role"], "operator", "{ack}");
    assert_eq!(ack["seq"], 7);
    assert_eq!(ack["auth_enforced"], true);
    // past the operator check (the write itself then wants expected_revision, nothing is stored)
    let ok = common::rpc(&mut ws, gen, epoch, "/api/v1/workspace/keepout", json!({"boxes": []})).await;
    assert_eq!(ok["code"], "INVALID_COMMAND", "same socket, same lease: {ok}");
    assert!(ok["message"].as_str().unwrap().contains("expected_revision"), "{ok}");
    assert_eq!(sup.inner().read().await.active_lease.as_ref().unwrap().generation, gen);

    let ack = common::ws_reply(&mut ws, json!({"type": "auth", "token": null}), &["auth_ack"]).await;
    assert_eq!(ack["role"], "program", "{ack}");
    let rej = common::rpc(&mut ws, gen, epoch, "/api/v1/workspace/keepout", json!({"boxes": []})).await;
    assert_eq!(rej["code"], "OPERATOR_REQUIRED", "{rej}");
    let ack = common::ws_reply(&mut ws, json!({"type": "auth", "token": "00".repeat(32)}), &["auth_ack"]).await;
    assert_eq!(ack["role"], "program", "{ack}");
}
