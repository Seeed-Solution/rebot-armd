//! OperatorAuth classification of the rc9 commands (teach v1, motor / software zero,
//! calibration sessions, depth measurement) and the two-phase writes of sequences.json and
//! the pose rename.
mod common;

use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;

use serde_json::{json, Value};
use tokio::net::TcpListener;
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::client::IntoClientRequest;

use armd::api::create_router_with_auth;
use armd::model::ArmConfig;
use armd::{ArmSupervisor, OperatorAuth};
use common::{TestSocket, INSPECT};

async fn spawn(sup: Arc<ArmSupervisor>, auth: OperatorAuth) -> SocketAddr {
    sup.set_operator_auth_enforced(auth.is_enforced()).await;
    let router = create_router_with_auth(sup.clone(), auth);
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });
    let ticker = sup.clone();
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(20));
        loop { interval.tick().await; ticker.tick(0.02).await; }
    });
    addr
}

async fn ws(addr: SocketAddr, token: Option<&str>) -> TestSocket {
    let mut req = format!("ws://{addr}/api/v1/ws").into_client_request().unwrap();
    if let Some(t) = token {
        req.headers_mut().insert("authorization", format!("Bearer {t}").parse().unwrap());
    }
    connect_async(req).await.unwrap().0
}

async fn lease(ws: &mut TestSocket, id: &str) -> u64 {
    let r = common::ws_reply(ws, json!({"type": "acquire_lease", "client_id": id}), &["lease_granted", "lease_rejected"]).await;
    r["generation"].as_u64().unwrap_or_else(|| panic!("{r}"))
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn rc9_commands_are_classified_in_the_operator_matrix() {
    let dir = tempfile::tempdir().unwrap();
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    armd::poses::load_into(&sup, &dir.path().join("poses.json")).await;
    armd::sequences::load_into(&sup, &dir.path().join("sequences.json")).await;
    {
        let mut inner = sup.inner().write().await;
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
    }
    let addr = spawn(sup.clone(), OperatorAuth::enforced("123456", 1800)).await;
    let client = reqwest::Client::new();
    let base = format!("http://{addr}");

    // program (no operator session), holding the lease
    let mut prog = ws(addr, None).await;
    let g = lease(&mut prog, "program").await;
    let epoch = sup.get_observation().await.epoch_id;
    for (path, body) in [
        ("/api/v1/poses/rename", json!({"id": "01ARZ3NDEKTSV4RRFFQ69G5FAV", "name": "x", "expected_revision": 1})),
        ("/api/v1/sequences/save", json!({"name": "s", "steps": [{"type": "wait", "s": 1.0}]})),
        ("/api/v1/sequences/delete", json!({"id": "01ARZ3NDEKTSV4RRFFQ69G5FAV"})),
        ("/api/v1/calibration/motor_zero", json!({"joints": [1], "dry_run": true})),
        ("/api/v1/calibration/joint_zero/restore", json!({"backup": "x"})),
        ("/api/v1/action/run_sequence", json!({"id": "01ARZ3NDEKTSV4RRFFQ69G5FAV", "record": {"dataset": "default"}})),
        // lease-free commands (episode store, recorder, depth measurement)
        ("/api/v1/record/start", json!({})),
        ("/api/v1/record/stop", json!({"save": true})),
        ("/api/v1/episodes/update", json!({"id": "01ARZ3NDEKTSV4RRFFQ69G5FAV", "task": "t"})),
        ("/api/v1/episodes/delete", json!({"id": "01ARZ3NDEKTSV4RRFFQ69G5FAV"})),
        ("/api/v1/datasets/create", json!({"name": "d1"})),
        ("/api/v1/datasets/delete", json!({"name": "d1"})),
        ("/api/v1/grasp/measure", json!({})),
    ] {
        let r = common::rpc(&mut prog, g, epoch, path, body).await;
        assert_eq!((r["type"].as_str(), r["code"].as_str()), (Some("command_reject"), Some("OPERATOR_REQUIRED")), "{path}: {r}");
    }
    // a run without recording is a lease action (not operator); it fails on the unknown id
    let r = common::rpc(&mut prog, g, epoch, "/api/v1/action/run_sequence", json!({"id": "01ARZ3NDEKTSV4RRFFQ69G5FAV"})).await;
    assert_ne!(r["code"], "OPERATOR_REQUIRED", "{r}");
    // the owner can always end a recording without saving it
    let r = common::rpc(&mut prog, g, epoch, "/api/v1/record/stop", json!({"save": false})).await;
    assert_ne!(r["code"], "OPERATOR_REQUIRED", "{r}");
    assert!(sup.inner().read().await.sequences.file.sequences.is_empty());
    assert!(!dir.path().join("sequences.json").exists());

    // REST: the GETs stay open, the writes need an operator
    for path in ["/api/v1/episodes", "/api/v1/datasets", "/api/v1/sequences", "/api/v1/calibration/sessions", "/api/v1/poses"] {
        let r = client.get(format!("{base}{path}")).send().await.unwrap();
        assert_ne!(r.status(), 403, "GET {path}");
    }
    for path in ["/api/v1/episodes/import", "/api/v1/calibration/sessions", "/api/v1/grasp/measure"] {
        let r = client.post(format!("{base}{path}")).header("content-type", "application/json").body("{}").send().await.unwrap();
        assert_eq!(r.status(), 403, "POST {path}");
        let v: Value = r.json().await.unwrap();
        assert_eq!(v["code"], "OPERATOR_REQUIRED", "POST {path}: {v}");
    }
    let r = client.delete(format!("{base}/api/v1/calibration/sessions/x")).send().await.unwrap();
    assert_eq!(r.status(), 403);
    drop(prog);
    tokio::time::sleep(Duration::from_millis(100)).await;

    // operator: sequences/save and delete are two-phase writes with a store revision
    let unlock: Value = client.post(format!("{base}/api/v1/auth/unlock")).json(&json!({"pin": "123456"})).send().await.unwrap().json().await.unwrap();
    let token = unlock["token"].as_str().unwrap().to_string();
    let mut op = ws(addr, Some(&token)).await;
    let g = lease(&mut op, "cockpit").await;
    let epoch = sup.get_observation().await.epoch_id;
    let r = common::rpc(&mut op, g, epoch, "/api/v1/sequences/save", json!({"name": "s", "steps": [{"type": "wait", "s": 1.0}]})).await;
    assert_eq!(r["type"], "command_ack", "{r}");
    assert_eq!((r["persisted"].as_bool(), r["revision"].as_u64()), (Some(true), Some(1)), "{r}");
    let id = r["sequence"]["id"].as_str().unwrap().to_string();
    let file: Value = serde_json::from_str(&std::fs::read_to_string(dir.path().join("sequences.json")).unwrap()).unwrap();
    assert_eq!(file["sequences"][0]["id"], json!(id));
    let list: Value = client.get(format!("{base}/api/v1/sequences")).send().await.unwrap().json().await.unwrap();
    assert_eq!(list["revision"], 1);
    // a stale store revision is refused and changes nothing
    let r = common::rpc(&mut op, g, epoch, "/api/v1/sequences/delete", json!({"id": id, "expected_revision": 0})).await;
    assert_eq!(r["code"], "POLICY_REVISION_MISMATCH", "{r}");
    assert_eq!(sup.inner().read().await.sequences.file.sequences.len(), 1);
    let r = common::rpc(&mut op, g, epoch, "/api/v1/sequences/delete", json!({"id": id, "expected_revision": 1})).await;
    assert_eq!((r["type"].as_str(), r["revision"].as_u64()), (Some("command_ack"), Some(2)), "{r}");
    let file: Value = serde_json::from_str(&std::fs::read_to_string(dir.path().join("sequences.json")).unwrap()).unwrap();
    assert_eq!(file["sequences"], json!([]));

    // pose rename: expected_revision required when enforced, then a two-phase write
    let r = common::rpc(&mut op, g, epoch, "/api/v1/poses/save", json!({"name": "scan", "expected_revision": 1})).await;
    assert_eq!(r["type"], "command_ack", "{r}");
    let pid = r["pose"]["id"].as_str().unwrap().to_string();
    let rev = r["revision"].as_u64().unwrap();
    let r = common::rpc(&mut op, g, epoch, "/api/v1/poses/rename", json!({"id": pid, "name": "scan2"})).await;
    assert_eq!(r["code"], "INVALID_COMMAND", "expected_revision is required: {r}");
    let r = common::rpc(&mut op, g, epoch, "/api/v1/poses/rename", json!({"id": pid, "name": "scan2", "expected_revision": rev})).await;
    assert_eq!((r["type"].as_str(), r["pose"]["id"].as_str(), r["pose"]["name"].as_str()), (Some("command_ack"), Some(pid.as_str()), Some("scan2")), "{r}");
    assert_eq!(r["revision"].as_u64(), Some(rev + 1));
    let file: Value = serde_json::from_str(&std::fs::read_to_string(dir.path().join("poses.json")).unwrap()).unwrap();
    assert_eq!(file["poses"][0]["name"], "scan2");
}
