//! Integration tests for P0-3: scheme-aware Origin validation, Bearer session isolation,
//! explicit WS auth (`?auth=explicit` + `auth_ack(seq, role)`), and pose `content_sha256`.
mod common;

use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;

use axum::http::header::{AUTHORIZATION, COOKIE, HOST, ORIGIN, SET_COOKIE};
use axum::http::HeaderMap;
use serde_json::{json, Value};
use tokio::net::TcpListener;
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::client::IntoClientRequest;

use armd::api::create_router_with_auth;
use armd::auth::{parse_host_header, parse_origin, validate_origin_headers};
use armd::model::ArmConfig;
use armd::poses::compute_pose_content_sha256;
use armd::safety::SafetyState;
use armd::{ArmSupervisor, OperatorAuth};
use common::{TestSocket, INSPECT};

async fn spawn_auth_server(supervisor: Arc<ArmSupervisor>, auth: OperatorAuth) -> SocketAddr {
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

async fn connect_ws(addr: SocketAddr, path_and_query: &str, headers: &[(&str, &str)]) -> TestSocket {
    let mut req = format!("ws://{addr}{path_and_query}").into_client_request().unwrap();
    for (k, v) in headers {
        let name = axum::http::HeaderName::from_bytes(k.as_bytes()).unwrap();
        let hv = axum::http::HeaderValue::from_str(v).unwrap();
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

fn extract_rebot_cookie(set_cookie_hdr: &str) -> String {
    let first = set_cookie_hdr.split(';').next().unwrap().trim();
    first.strip_prefix("rebot_op=").unwrap().to_string()
}

#[test]
fn origin_parser_normalizes_ports_case_and_ipv6() {
    assert_eq!(
        parse_origin("HTTP://LocalHost"),
        parse_host_header("localhost:80", "http")
    );
    assert_eq!(
        parse_origin("http://example.com:80"),
        parse_host_header("EXAMPLE.COM", "http")
    );
    assert_eq!(
        parse_origin("https://example.com:443"),
        parse_host_header("example.com", "https")
    );
    assert_eq!(
        parse_origin("http://[2001:0db8::1]:8765"),
        parse_host_header("[2001:db8::1]:8765", "http")
    );
    assert_eq!(
        parse_origin("http://[::1]"),
        parse_host_header("[0:0::1]:80", "http")
    );

    // Malformed / null / non-http(s) / userinfo / path origins are rejected
    for bad in [
        "null",
        "NULL",
        "",
        "ws://127.0.0.1:8765",
        "file://localhost",
        "http://user@127.0.0.1:8765",
        "http://127.0.0.1:8765/path",
        "http://127.0.0.1:8765?a=1",
        "http://127.0.0.1:8765#frag",
        "http://127.0.0.1:0",
        "http://127.0.0.1:99999",
        "http://[::1",
        "http://::1:8765",
    ] {
        assert!(parse_origin(bad).is_none(), "expected None for {bad}");
    }

    // Wrong scheme with same host+port is rejected
    let mut hdrs = HeaderMap::new();
    hdrs.insert(HOST, "127.0.0.1:8765".parse().unwrap());
    hdrs.insert(ORIGIN, "https://127.0.0.1:8765".parse().unwrap());
    assert!(!validate_origin_headers(&hdrs, None));

    // Overridden via configured public origin accepts https and rejects http for that host
    assert!(validate_origin_headers(&hdrs, Some("https://127.0.0.1:8765")));
    hdrs.insert(ORIGIN, "http://127.0.0.1:8765".parse().unwrap());
    assert!(!validate_origin_headers(&hdrs, Some("https://127.0.0.1:8765")));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn scheme_aware_origin_validation_on_http_and_ws_upgrade() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let auth = OperatorAuth::enforced("123456", 1800);
    let addr = spawn_auth_server(sup.clone(), auth).await;
    let client = reqwest::Client::new();
    let base = format!("http://{addr}");

    // Same host+port, wrong scheme (https:// against http://) -> 403 BEFORE handler runs
    let bad_scheme_estop = client
        .post(format!("{base}/api/v1/safety/e_stop"))
        .header("Origin", format!("https://{addr}"))
        .send()
        .await
        .unwrap();
    assert_eq!(bad_scheme_estop.status(), 403);
    assert_eq!(sup.get_observation().await.safety_state, SafetyState::Idle);

    let bad_scheme_unlock = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .header("Origin", format!("https://{addr}"))
        .json(&json!({"pin": "123456"}))
        .send()
        .await
        .unwrap();
    assert_eq!(bad_scheme_unlock.status(), 403);

    // Other mutating HTTP verbs (DELETE, PUT, PATCH) with wrong scheme or null Origin -> 403
    for method in [reqwest::Method::DELETE, reqwest::Method::PUT, reqwest::Method::PATCH] {
        let r = client
            .request(method.clone(), format!("{base}/api/v1/dataset/episodes/ep1"))
            .header("Origin", format!("https://{addr}"))
            .send()
            .await
            .unwrap();
        assert_eq!(r.status(), 403, "{method}");
    }

    // Origin: null -> 403 on HTTP POST and WS Upgrade
    let null_origin = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .header("Origin", "null")
        .json(&json!({"pin": "123456"}))
        .send()
        .await
        .unwrap();
    assert_eq!(null_origin.status(), 403);

    let mut ws_wrong_scheme = format!("ws://{addr}/api/v1/ws").into_client_request().unwrap();
    ws_wrong_scheme
        .headers_mut()
        .insert("Origin", format!("https://{addr}").parse().unwrap());
    let err = connect_async(ws_wrong_scheme).await.unwrap_err();
    match err {
        tokio_tungstenite::tungstenite::Error::Http(resp) => assert_eq!(resp.status(), 403),
        other => panic!("expected HTTP 403 rejection on WS upgrade, got {other:?}"),
    }

    let mut ws_null_origin = format!("ws://{addr}/api/v1/ws").into_client_request().unwrap();
    ws_null_origin.headers_mut().insert("Origin", "null".parse().unwrap());
    let err_null = connect_async(ws_null_origin).await.unwrap_err();
    match err_null {
        tokio_tungstenite::tungstenite::Error::Http(resp) => assert_eq!(resp.status(), 403),
        other => panic!("expected HTTP 403 rejection on WS upgrade, got {other:?}"),
    }

    // Matching http:// Origin succeeds on both HTTP POST and WS Upgrade
    let ok_http = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .header("Origin", format!("http://{addr}"))
        .json(&json!({"pin": "123456", "set_cookie": false}))
        .send()
        .await
        .unwrap();
    assert_eq!(ok_http.status(), 200);

    let mut ws_ok = format!("ws://{addr}/api/v1/ws").into_client_request().unwrap();
    ws_ok
        .headers_mut()
        .insert("Origin", format!("http://{addr}").parse().unwrap());
    assert!(connect_async(ws_ok).await.is_ok());

    // Absent Origin (CLI / Python SDK / curl) continues to succeed
    let no_origin = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .json(&json!({"pin": "123456", "set_cookie": false}))
        .send()
        .await
        .unwrap();
    assert_eq!(no_origin.status(), 200);

    // Explicit public origin override accepts https:// and rejects http:// for the same host:port
    let sup_https = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let https_addr = listener.local_addr().unwrap();
    let auth_https = OperatorAuth::enforced("123456", 1800)
        .with_public_origin(Some(format!("https://{https_addr}")));
    let router_https = create_router_with_auth(sup_https, auth_https);
    tokio::spawn(async move { axum::serve(listener, router_https).await.unwrap(); });

    let ok_pub = client
        .post(format!("http://{https_addr}/api/v1/auth/unlock"))
        .header("Origin", format!("https://{https_addr}"))
        .json(&json!({"pin": "123456", "set_cookie": false}))
        .send()
        .await
        .unwrap();
    assert_eq!(ok_pub.status(), 200);

    let rej_http = client
        .post(format!("http://{https_addr}/api/v1/auth/unlock"))
        .header("Origin", format!("http://{https_addr}"))
        .json(&json!({"pin": "123456", "set_cookie": false}))
        .send()
        .await
        .unwrap();
    assert_eq!(rej_http.status(), 403);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn unlock_set_cookie_false_and_bearer_isolation_from_cookie() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let auth = OperatorAuth::enforced("123456", 1800);
    let addr = spawn_auth_server(sup.clone(), auth).await;
    let client = reqwest::Client::new();
    let base = format!("http://{addr}");

    // 1. Default unlock (set_cookie omitted) emits Set-Cookie
    let cockpit_unlock = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .json(&json!({"pin": "123456"}))
        .send()
        .await
        .unwrap();
    assert_eq!(cockpit_unlock.status(), 200);
    let set_cookie_hdr = cockpit_unlock
        .headers()
        .get(SET_COOKIE)
        .expect("default unlock must emit Set-Cookie")
        .to_str()
        .unwrap()
        .to_string();
    let cockpit_tok = extract_rebot_cookie(&set_cookie_hdr);
    let cockpit_body: Value = cockpit_unlock.json().await.unwrap();
    assert_eq!(cockpit_body["token"], cockpit_tok);

    // 2. unlock with set_cookie: false returns token JSON and emits NO Set-Cookie
    let bearer_unlock = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .json(&json!({"pin": "123456", "set_cookie": false}))
        .send()
        .await
        .unwrap();
    assert_eq!(bearer_unlock.status(), 200);
    assert!(
        bearer_unlock.headers().get(SET_COOKIE).is_none(),
        "set_cookie: false must not emit Set-Cookie"
    );
    let bearer_body: Value = bearer_unlock.json().await.unwrap();
    assert_eq!(bearer_body["role"], "operator");
    let bearer_tok = bearer_body["token"].as_str().unwrap().to_string();
    assert_ne!(bearer_tok, cockpit_tok);

    // 3. Invalid Bearer + valid Cockpit Cookie MUST be rejected and NEVER fall back to Cookie
    let st_bad: Value = client
        .get(format!("{base}/api/v1/auth/status"))
        .header(AUTHORIZATION, "Bearer bad_token_value")
        .header(COOKIE, format!("rebot_op={cockpit_tok}"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(st_bad["role"], "program");
    assert!(st_bad["expires_in_s"].is_null());

    let priv_bad = client
        .post(format!("{base}/api/v1/mode/gravity_comp"))
        .header(AUTHORIZATION, "Bearer bad_token_value")
        .header(COOKIE, format!("rebot_op={cockpit_tok}"))
        .json(&json!({"enable": true}))
        .send()
        .await
        .unwrap();
    assert_eq!(priv_bad.status(), 403);

    // Empty Bearer + valid Cockpit Cookie also MUST NOT fall back to Cookie
    let priv_empty = client
        .post(format!("{base}/api/v1/mode/gravity_comp"))
        .header(AUTHORIZATION, "Bearer ")
        .header(COOKIE, format!("rebot_op={cockpit_tok}"))
        .json(&json!({"enable": true}))
        .send()
        .await
        .unwrap();
    assert_eq!(priv_empty.status(), 403);

    // Middleware-guarded route (`require_operator`) with bad Bearer + valid Cookie also rejects 403
    let mw_bad = client
        .post(format!("{base}/api/v1/sandbox/reset"))
        .header(AUTHORIZATION, "Bearer bad_token_value")
        .header(COOKIE, format!("rebot_op={cockpit_tok}"))
        .send()
        .await
        .unwrap();
    assert_eq!(mw_bad.status(), 403);

    // 4. POST /api/v1/auth/lock with Authorization: Bearer <t1> + Cookie <t2>:
    //    revokes ONLY <t1>, emits NO Set-Cookie, and leaves <t2> valid!
    let lock_bearer = client
        .post(format!("{base}/api/v1/auth/lock"))
        .header(AUTHORIZATION, format!("Bearer {bearer_tok}"))
        .header(COOKIE, format!("rebot_op={cockpit_tok}"))
        .send()
        .await
        .unwrap();
    assert_eq!(lock_bearer.status(), 200);
    assert!(
        lock_bearer.headers().get(SET_COOKIE).is_none(),
        "Bearer lock must NOT emit Set-Cookie"
    );
    let lock_body: Value = lock_bearer.json().await.unwrap();
    assert_eq!(lock_body["role"], "program");

    // <t1> is revoked
    let st_t1: Value = client
        .get(format!("{base}/api/v1/auth/status"))
        .header(AUTHORIZATION, format!("Bearer {bearer_tok}"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(st_t1["role"], "program");

    // <t2> (Cockpit Cookie) is STILL valid
    let st_t2: Value = client
        .get(format!("{base}/api/v1/auth/status"))
        .header(COOKIE, format!("rebot_op={cockpit_tok}"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(st_t2["role"], "operator");

    // Cockpit Cookie lock (no Authorization header) DOES emit Set-Cookie Max-Age=0 and revokes <t2>
    let lock_cookie = client
        .post(format!("{base}/api/v1/auth/lock"))
        .header(COOKIE, format!("rebot_op={cockpit_tok}"))
        .send()
        .await
        .unwrap();
    assert_eq!(lock_cookie.status(), 200);
    let clear_hdr = lock_cookie.headers().get(SET_COOKIE).unwrap().to_str().unwrap();
    assert!(clear_hdr.contains("Max-Age=0"), "{clear_hdr}");

    let st_t2_after: Value = client
        .get(format!("{base}/api/v1/auth/status"))
        .header(COOKIE, format!("rebot_op={cockpit_tok}"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(st_t2_after["role"], "program");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn ws_explicit_auth_ignores_upgrade_cookie_and_promotes_on_auth_frame() {
    let dir = tempfile::tempdir().unwrap();
    let poses_path = dir.path().join("poses.json");
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    armd::poses::load_into(&sup, &poses_path).await;
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

    let cockpit_tok = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .json(&json!({"pin": "123456"}))
        .send()
        .await
        .unwrap()
        .json::<Value>()
        .await
        .unwrap()["token"]
        .as_str()
        .unwrap()
        .to_string();

    let bearer_tok = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .json(&json!({"pin": "123456", "set_cookie": false}))
        .send()
        .await
        .unwrap()
        .json::<Value>()
        .await
        .unwrap()["token"]
        .as_str()
        .unwrap()
        .to_string();

    // Connect with ?auth=explicit AND a valid Cockpit Cookie on the Upgrade request.
    let mut ws = connect_ws(
        addr,
        "/api/v1/ws?obs_hz=10&auth=explicit",
        &[("Cookie", &format!("rebot_op={cockpit_tok}"))],
    )
    .await;
    let granted = acquire(&mut ws, "playground", false).await;
    let gen = granted["generation"].as_u64().unwrap();
    let epoch = sup.get_observation().await.epoch_id;

    // Must start unprivileged (must NOT borrow Cockpit Upgrade Cookie)
    let rej_before = common::rpc(
        &mut ws,
        gen,
        epoch,
        "/api/v1/poses/save",
        json!({"name": "scan", "expected_revision": 1}),
    )
    .await;
    assert_eq!(rej_before["code"], "OPERATOR_REQUIRED", "{rej_before}");

    // Bad auth frame -> auth_ack with matching seq and role="program"
    let bad_ack = common::ws_reply(
        &mut ws,
        json!({"type": "auth", "seq": 11, "token": "wrong_token"}),
        &["auth_ack"],
    )
    .await;
    assert_eq!(bad_ack["seq"], 11);
    assert_eq!(bad_ack["role"], "program");
    assert_eq!(bad_ack["auth_enforced"], true);

    // Valid auth frame -> auth_ack with matching seq and role="operator"
    let ok_ack = common::ws_reply(
        &mut ws,
        json!({"type": "auth", "seq": 12, "token": bearer_tok}),
        &["auth_ack"],
    )
    .await;
    assert_eq!(ok_ack["seq"], 12);
    assert_eq!(ok_ack["role"], "operator");
    assert_eq!(ok_ack["auth_enforced"], true);
    assert!(ok_ack["expires_in_s"].as_u64().unwrap() > 0);

    // Now privileged commands succeed on the same connection
    let ok_save = common::rpc(
        &mut ws,
        gen,
        epoch,
        "/api/v1/poses/save",
        json!({"name": "scan", "expected_revision": 1}),
    )
    .await;
    assert_eq!(ok_save["type"], "command_ack", "{ok_save}");

    // Default Cockpit WS (without ?auth=explicit) still inherits Upgrade Cookie
    let mut cockpit_ws = connect_ws(
        addr,
        "/api/v1/ws",
        &[("Cookie", &format!("rebot_op={cockpit_tok}"))],
    )
    .await;
    let c_granted = acquire(&mut cockpit_ws, "cockpit", true).await;
    let c_gen = c_granted["generation"].as_u64().unwrap();
    let c_save = common::rpc(
        &mut cockpit_ws,
        c_gen,
        epoch,
        "/api/v1/poses/save",
        json!({"name": "cockpit_pose", "expected_revision": 2}),
    )
    .await;
    assert_eq!(c_save["type"], "command_ack", "{c_save}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn pose_content_sha256_matches_across_ws_ack_rest_and_canonical_hash() {
    // Hardcoded SHA-256 test vector + canonical -0.0 == +0.0 check
    let jnames = vec!["j1".to_string(), "j2".to_string()];
    assert_eq!(
        compute_pose_content_sha256("01ARZ3NDEKTSV4RRFFQ69G5FAV", &jnames, &[-0.0, 1.25]),
        "6f2b329a46614fc796cd9142ded2475949212ba066e73df6e9f880b36db8080e"
    );
    assert_eq!(
        compute_pose_content_sha256("id1", &jnames, &[-0.0, 1.25]),
        compute_pose_content_sha256("id1", &jnames, &[0.0, 1.25]),
    );

    let dir = tempfile::tempdir().unwrap();
    let poses_path = dir.path().join("poses.json");
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    armd::poses::load_into(&sup, &poses_path).await;
    let auth = OperatorAuth::enforced("123456", 1800);
    let addr = spawn_auth_server(sup.clone(), auth).await;
    let client = reqwest::Client::new();
    let base = format!("http://{addr}");

    let tok = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .json(&json!({"pin": "123456", "set_cookie": false}))
        .send()
        .await
        .unwrap()
        .json::<Value>()
        .await
        .unwrap()["token"]
        .as_str()
        .unwrap()
        .to_string();

    let mut ws = connect_ws(addr, "/api/v1/ws?auth=explicit", &[]).await;
    let ack_auth = common::ws_reply(
        &mut ws,
        json!({"type": "auth", "seq": 1, "token": tok}),
        &["auth_ack"],
    )
    .await;
    assert_eq!(ack_auth["role"], "operator");

    let g = acquire(&mut ws, "pose_tester", false).await["generation"]
        .as_u64()
        .unwrap();
    let epoch = sup.get_observation().await.epoch_id;

    let pos1 = vec![0.1_f32, -0.5, -1.2, 1.1, 0.2, 0.9, -0.3];
    let ack1 = common::rpc(
        &mut ws,
        g,
        epoch,
        "/api/v1/poses/save",
        json!({"name": "pick_pose", "positions": pos1, "expected_revision": 1}),
    )
    .await;
    assert_eq!(ack1["type"], "command_ack", "{ack1}");

    let ack_sha1 = ack1["content_sha256"].as_str().unwrap().to_string();
    assert_eq!(ack_sha1.len(), 64);
    assert!(ack_sha1.bytes().all(|b| matches!(b, b'0'..=b'9' | b'a'..=b'f')));
    assert_eq!(ack1["pose"]["content_sha256"], ack_sha1);
    let pose_id = ack1["pose_id"].as_str().unwrap().to_string();
    assert_eq!(ack1["pose"]["pose_id"], pose_id);
    assert_eq!(ack1["pose"]["id"], pose_id);
    assert!(ack1["stale"].is_null());

    let joint_names: Vec<String> = ack1["joint_names"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_str().unwrap().to_string())
        .collect();
    let joints_f64: Vec<f64> = pos1.iter().map(|&v| f64::from(v)).collect();
    assert_eq!(
        ack_sha1,
        compute_pose_content_sha256(&pose_id, &joint_names, &joints_f64)
    );

    // GET /api/v1/poses matches WS ack identically
    let rest1: Value = client
        .get(format!("{base}/api/v1/poses"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let entry1 = &rest1["poses"][0];
    assert_eq!(entry1["pose_id"], pose_id);
    assert_eq!(entry1["content_sha256"], ack_sha1);
    assert_eq!(entry1["joint_names"], ack1["joint_names"]);
    assert!(entry1["stale"].is_null());

    // Modifying a joint value changes content_sha256 in both WS ack and GET /api/v1/poses
    let mut pos2 = pos1.clone();
    pos2[2] = -1.25;
    let ack2 = common::rpc(
        &mut ws,
        g,
        epoch,
        "/api/v1/poses/save",
        json!({"name": "pick_pose", "positions": pos2, "expected_revision": 2}),
    )
    .await;
    assert_eq!(ack2["type"], "command_ack", "{ack2}");
    let ack_sha2 = ack2["content_sha256"].as_str().unwrap().to_string();
    assert_eq!(ack_sha2.len(), 64);
    assert_ne!(ack_sha2, ack_sha1);
    assert_eq!(ack2["pose_id"], pose_id);

    let rest2: Value = client
        .get(format!("{base}/api/v1/poses"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(rest2["poses"][0]["content_sha256"], ack_sha2);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn ws_explicit_auth_with_pin_disabled_still_gates_until_auth_frame() {
    let dir = tempfile::tempdir().unwrap();
    let poses_path = dir.path().join("poses.json");
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    armd::poses::load_into(&sup, &poses_path).await;
    {
        let mut inner = sup.inner().write().await;
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
    }
    let addr = spawn_auth_server(sup.clone(), OperatorAuth::disabled()).await;

    let mut ws = connect_ws(addr, "/api/v1/ws?obs_hz=10&auth=explicit", &[]).await;
    let granted = acquire(&mut ws, "playground_no_pin", false).await;
    let gen = granted["generation"].as_u64().unwrap();
    let epoch = sup.get_observation().await.epoch_id;

    // Even with PIN enforcement disabled, ?auth=explicit starts unprivileged until `auth` frame completes
    let rej_pose = common::rpc(
        &mut ws,
        gen,
        epoch,
        "/api/v1/poses/save",
        json!({"name": "scan"}),
    )
    .await;
    assert_eq!(rej_pose["code"], "OPERATOR_REQUIRED", "{rej_pose}");

    let rej_rec = common::rpc(
        &mut ws,
        gen,
        epoch,
        "/api/v1/record/start",
        json!({}),
    )
    .await;
    assert_eq!(rej_rec["code"], "OPERATOR_REQUIRED", "{rej_rec}");

    // Complete explicit auth handshake
    let ack = common::ws_reply(
        &mut ws,
        json!({"type": "auth", "seq": 9, "token": null}),
        &["auth_ack"],
    )
    .await;
    assert_eq!(ack["seq"], 9);
    assert_eq!(ack["role"], "operator");
    assert_eq!(ack["auth_enforced"], false);

    // Operator command now succeeds
    let ok_pose = common::rpc(
        &mut ws,
        gen,
        epoch,
        "/api/v1/poses/save",
        json!({"name": "scan"}),
    )
    .await;
    assert_eq!(ok_pose["type"], "command_ack", "{ok_pose}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn secure_cookie_ignores_untrusted_forwarded_headers_on_plain_http() {
    let client = reqwest::Client::new();

    // 1. Default plain HTTP: spoofed X-Forwarded-Proto / Forwarded MUST NOT emit `; Secure`
    let sup_plain = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let addr_plain = spawn_auth_server(sup_plain, OperatorAuth::enforced("123456", 1800)).await;
    let unlock_spoof = client
        .post(format!("http://{addr_plain}/api/v1/auth/unlock"))
        .header("X-Forwarded-Proto", "https")
        .header("Forwarded", "proto=https")
        .json(&json!({"pin": "123456"}))
        .send()
        .await
        .unwrap();
    assert_eq!(unlock_spoof.status(), 200);
    let cookie_spoof = unlock_spoof.headers().get(SET_COOKIE).unwrap().to_str().unwrap().to_string();
    assert!(!cookie_spoof.contains("; Secure"), "plain HTTP must ignore spoofed forwarded headers: {cookie_spoof}");
    let tok = extract_rebot_cookie(&cookie_spoof);

    let lock_spoof = client
        .post(format!("http://{addr_plain}/api/v1/auth/lock"))
        .header(COOKIE, format!("rebot_op={tok}"))
        .header("X-Forwarded-Proto", "https")
        .header("Forwarded", "proto=https")
        .send()
        .await
        .unwrap();
    assert_eq!(lock_spoof.status(), 200);
    let lock_cookie_spoof = lock_spoof.headers().get(SET_COOKIE).unwrap().to_str().unwrap().to_string();
    assert!(!lock_cookie_spoof.contains("; Secure"), "plain HTTP lock must ignore spoofed forwarded headers: {lock_cookie_spoof}");

    // 2. HTTPS public_origin DOES emit `; Secure` on unlock and cookie lock
    let sup_pub = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let pub_addr = listener.local_addr().unwrap();
    let auth_pub = OperatorAuth::enforced("123456", 1800)
        .with_public_origin(Some(format!("https://{pub_addr}")));
    let router_pub = create_router_with_auth(sup_pub, auth_pub);
    tokio::spawn(async move { axum::serve(listener, router_pub).await.unwrap(); });

    let unlock_pub = client
        .post(format!("http://{pub_addr}/api/v1/auth/unlock"))
        .header("Origin", format!("https://{pub_addr}"))
        .json(&json!({"pin": "123456"}))
        .send()
        .await
        .unwrap();
    assert_eq!(unlock_pub.status(), 200);
    let cookie_pub = unlock_pub.headers().get(SET_COOKIE).unwrap().to_str().unwrap().to_string();
    assert!(cookie_pub.contains("; Secure"), "HTTPS public_origin must emit ; Secure: {cookie_pub}");
    let tok_pub = extract_rebot_cookie(&cookie_pub);

    let lock_pub = client
        .post(format!("http://{pub_addr}/api/v1/auth/lock"))
        .header("Origin", format!("https://{pub_addr}"))
        .header(COOKIE, format!("rebot_op={tok_pub}"))
        .send()
        .await
        .unwrap();
    assert_eq!(lock_pub.status(), 200);
    let lock_cookie_pub = lock_pub.headers().get(SET_COOKIE).unwrap().to_str().unwrap().to_string();
    assert!(lock_cookie_pub.contains("; Secure"), "HTTPS public_origin lock must emit ; Secure: {lock_cookie_pub}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn async_config_commit_observes_midflight_ws_auth_demotion() {
    let dir = tempfile::tempdir().unwrap();
    let poses_path = dir.path().join("poses.json");
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    armd::poses::load_into(&sup, &poses_path).await;
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

    let tok = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .json(&json!({"pin": "123456", "set_cookie": false}))
        .send()
        .await
        .unwrap()
        .json::<Value>()
        .await
        .unwrap()["token"]
        .as_str()
        .unwrap()
        .to_string();

    let mut ws = connect_ws(addr, "/api/v1/ws?auth=explicit", &[]).await;
    let ack1 = common::ws_reply(
        &mut ws,
        json!({"type": "auth", "seq": 1, "token": tok}),
        &["auth_ack"],
    )
    .await;
    assert_eq!(ack1["role"], "operator");

    let g = acquire(&mut ws, "demote_tester", false).await["generation"]
        .as_u64()
        .unwrap();
    let epoch = sup.get_observation().await.epoch_id;

    // Delay staging so we can demote the socket via an `auth` frame before commit
    armd::persist::TEST_WRITE_DELAY_MS.store(250, std::sync::atomic::Ordering::Relaxed);
    common::send(
        &mut ws,
        json!({
            "type": "command",
            "request_id": 880_001,
            "generation": g,
            "epoch_id": epoch,
            "path": "/api/v1/poses/save",
            "body": {"name": "midflight_pose", "expected_revision": 1}
        }),
    )
    .await;

    tokio::time::sleep(Duration::from_millis(50)).await;
    // Demote ONLY this socket's WsAuth state (token itself stays valid on the server!)
    let ack2 = common::ws_reply(
        &mut ws,
        json!({"type": "auth", "seq": 2, "token": null}),
        &["auth_ack"],
    )
    .await;
    assert_eq!(ack2["seq"], 2);
    assert_eq!(ack2["role"], "program");

    let reject = common::next_of(&mut ws, &["command_ack", "command_reject"]).await;
    armd::persist::TEST_WRITE_DELAY_MS.store(0, std::sync::atomic::Ordering::Relaxed);
    assert_eq!(reject["request_id"], 880_001);
    assert_eq!(reject["type"], "command_reject", "{reject}");
    assert_eq!(reject["code"], "OPERATOR_REQUIRED", "{reject}");

    // Nothing was committed to memory or disk
    let rest: Value = client
        .get(format!("{base}/api/v1/poses"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(rest["revision"], 1);
    assert_eq!(rest["poses"].as_array().unwrap().len(), 0);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn public_origin_rejects_arbitrary_host_match_fallback_and_allows_loopback_proxy() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let auth = OperatorAuth::enforced("123456", 1800)
        .with_public_origin(Some("https://robot.example.com".to_string()));
    let addr = spawn_auth_server(sup.clone(), auth).await;
    let client = reqwest::Client::new();
    let base = format!("http://{addr}");
    let port = addr.port();

    // 1. Accepts configured public_origin on HTTP POST and WS upgrade
    let ok_pub = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .header(HOST, "robot.example.com")
        .header(ORIGIN, "https://robot.example.com")
        .json(&json!({"pin": "123456", "set_cookie": false}))
        .send()
        .await
        .unwrap();
    assert_eq!(ok_pub.status(), 200);

    let mut ws_pub = format!("ws://{addr}/api/v1/ws").into_client_request().unwrap();
    ws_pub.headers_mut().insert(HOST, "robot.example.com".parse().unwrap());
    ws_pub.headers_mut().insert(ORIGIN, "https://robot.example.com".parse().unwrap());
    assert!(connect_async(ws_pub).await.is_ok());

    // 2. Accepts local loopback console proxy (Host: 127.0.0.1:<port>, Origin: http://127.0.0.1:<port>)
    let ok_loopback = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .header(HOST, format!("127.0.0.1:{port}"))
        .header(ORIGIN, format!("http://127.0.0.1:{port}"))
        .json(&json!({"pin": "123456", "set_cookie": false}))
        .send()
        .await
        .unwrap();
    assert_eq!(ok_loopback.status(), 200);

    let mut ws_loopback = format!("ws://{addr}/api/v1/ws").into_client_request().unwrap();
    ws_loopback.headers_mut().insert(HOST, format!("127.0.0.1:{port}").parse().unwrap());
    ws_loopback.headers_mut().insert(ORIGIN, format!("http://127.0.0.1:{port}").parse().unwrap());
    assert!(connect_async(ws_loopback).await.is_ok());

    // 3. Rejects arbitrary Host-match fallback (evil.example) and non-approved device IP (192.168.42.1)
    for (host_val, origin_val) in [
        (format!("evil.example:{port}"), format!("http://evil.example:{port}")),
        (format!("192.168.42.1:{port}"), format!("http://192.168.42.1:{port}")),
    ] {
        let rej_http = client
            .post(format!("{base}/api/v1/auth/unlock"))
            .header(HOST, &host_val)
            .header(ORIGIN, &origin_val)
            .json(&json!({"pin": "123456", "set_cookie": false}))
            .send()
            .await
            .unwrap();
        assert_eq!(rej_http.status(), 403, "HTTP POST must reject {origin_val}");

        let mut ws_rej = format!("ws://{addr}/api/v1/ws").into_client_request().unwrap();
        ws_rej.headers_mut().insert(HOST, host_val.parse().unwrap());
        ws_rej.headers_mut().insert(ORIGIN, origin_val.parse().unwrap());
        let err = connect_async(ws_rej).await.unwrap_err();
        match err {
            tokio_tungstenite::tungstenite::Error::Http(resp) => {
                assert_eq!(resp.status(), 403, "WS upgrade must reject {origin_val}");
            }
            other => panic!("expected HTTP 403 for {origin_val}, got {other:?}"),
        }
    }

    // 4. Bare scheme-only public origin ("https" / "http") fails closed (no arbitrary Host-match fallback)
    let mut bare_hdrs = HeaderMap::new();
    bare_hdrs.insert(HOST, "evil.example:8765".parse().unwrap());
    bare_hdrs.insert(ORIGIN, "https://evil.example:8765".parse().unwrap());
    assert!(!validate_origin_headers(&bare_hdrs, Some("https")));
    bare_hdrs.insert(ORIGIN, "http://evil.example:8765".parse().unwrap());
    assert!(!validate_origin_headers(&bare_hdrs, Some("http")));

    // 5. Credential-free native request (no Origin header, no bearer/cookie) to operator-gated endpoint -> 403 OPERATOR_REQUIRED
    let no_cred = client
        .post(format!("{base}/api/v1/sandbox/reset"))
        .send()
        .await
        .unwrap();
    assert_eq!(no_cred.status(), 403);
    let no_cred_body: Value = no_cred.json().await.unwrap();
    assert_eq!(no_cred_body["code"], "OPERATOR_REQUIRED");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn default_device_mode_and_explicit_allowed_origins() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let auth = OperatorAuth::enforced("123456", 1800);
    let addr = spawn_auth_server(sup.clone(), auth).await;
    let client = reqwest::Client::new();
    let base = format!("http://{addr}");
    let port = addr.port();

    // 1. Default device mode accepts direct device IP same-origin on HTTP POST and WS upgrade
    let ok_dev = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .header(HOST, "192.168.42.1:8765")
        .header(ORIGIN, "http://192.168.42.1:8765")
        .json(&json!({"pin": "123456", "set_cookie": false}))
        .send()
        .await
        .unwrap();
    assert_eq!(ok_dev.status(), 200);

    let mut ws_dev = format!("ws://{addr}/api/v1/ws").into_client_request().unwrap();
    ws_dev.headers_mut().insert(HOST, "192.168.42.1:8765".parse().unwrap());
    ws_dev.headers_mut().insert(ORIGIN, "http://192.168.42.1:8765".parse().unwrap());
    assert!(connect_async(ws_dev).await.is_ok());

    // 2. Default device mode accepts reCamera console proxy (127.0.0.1:<port>) on HTTP POST and WS upgrade
    let ok_proxy = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .header(HOST, format!("127.0.0.1:{port}"))
        .header(ORIGIN, format!("http://127.0.0.1:{port}"))
        .json(&json!({"pin": "123456", "set_cookie": false}))
        .send()
        .await
        .unwrap();
    assert_eq!(ok_proxy.status(), 200);

    let mut ws_proxy = format!("ws://{addr}/api/v1/ws").into_client_request().unwrap();
    ws_proxy.headers_mut().insert(HOST, format!("127.0.0.1:{port}").parse().unwrap());
    ws_proxy.headers_mut().insert(ORIGIN, format!("http://127.0.0.1:{port}").parse().unwrap());
    assert!(connect_async(ws_proxy).await.is_ok());

    // 3. Default device mode rejects cross-origin and wrong-scheme with 403
    for (host_val, origin_val) in [
        ("192.168.42.1:8765", "http://evil.example:8765"),
        ("192.168.42.1:8765", "https://192.168.42.1:8765"),
    ] {
        let rej = client
            .post(format!("{base}/api/v1/auth/unlock"))
            .header(HOST, host_val)
            .header(ORIGIN, origin_val)
            .json(&json!({"pin": "123456", "set_cookie": false}))
            .send()
            .await
            .unwrap();
        assert_eq!(rej.status(), 403, "expected 403 for {origin_val}");

        let mut ws_rej = format!("ws://{addr}/api/v1/ws").into_client_request().unwrap();
        ws_rej.headers_mut().insert(HOST, host_val.parse().unwrap());
        ws_rej.headers_mut().insert(ORIGIN, origin_val.parse().unwrap());
        let err = connect_async(ws_rej).await.unwrap_err();
        match err {
            tokio_tungstenite::tungstenite::Error::Http(resp) => assert_eq!(resp.status(), 403),
            other => panic!("expected HTTP 403 for {origin_val}, got {other:?}"),
        }
    }

    // 4. Explicitly approved allowed_origins works with public_origin, unless shadowing public_origin scheme
    let sup_allowed = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let auth_allowed = OperatorAuth::enforced("123456", 1800)
        .with_public_origin(Some("https://robot.example.com".to_string()))
        .with_allowed_origins(vec![
            parse_origin("http://192.168.42.1:8765").unwrap(),
            parse_origin("http://robot.example.com:443").unwrap(),
            parse_origin("http://robot.example.com").unwrap(),
        ]);
    let addr_allowed = spawn_auth_server(sup_allowed, auth_allowed).await;
    let ok_lan = client
        .post(format!("http://{addr_allowed}/api/v1/auth/unlock"))
        .header(HOST, "192.168.42.1:8765")
        .header(ORIGIN, "http://192.168.42.1:8765")
        .json(&json!({"pin": "123456", "set_cookie": false}))
        .send()
        .await
        .unwrap();
    assert_eq!(ok_lan.status(), 200);

    for shadow_origin in ["http://robot.example.com", "http://robot.example.com:443"] {
        let rej_shadow = client
            .post(format!("http://{addr_allowed}/api/v1/auth/unlock"))
            .header(HOST, "robot.example.com")
            .header(ORIGIN, shadow_origin)
            .json(&json!({"pin": "123456", "set_cookie": false}))
            .send()
            .await
            .unwrap();
        assert_eq!(rej_shadow.status(), 403, "shadowing origin {shadow_origin} must be rejected");
    }
}
