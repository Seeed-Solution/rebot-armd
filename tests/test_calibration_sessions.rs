//! Calibration sessions API (`/api/v1/calibration/sessions`, src/calibration_sessions.rs):
//! store, list, get, overwrite, delete, the cap of 10 and the 8 MB body limit.

use std::sync::Arc;

use armd::calibration::CalibrationDeps;
use armd::grasp::GraspDeps;
use armd::{ArmConfig, ArmSupervisor};
use serde_json::{json, Value};

struct Rig {
    base: String,
    http: reqwest::Client,
    dir: tempfile::TempDir,
}

async fn rig() -> Rig {
    let dir = tempfile::tempdir().unwrap();
    let up = armd::api::proxy::DetectorUpstream::parse("http://127.0.0.1:9").unwrap();
    let mut deps = CalibrationDeps::new(dir.path().join("calibration.json"), None, up.clone());
    deps.sessions_dir = dir.path().join("sessions");
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let router = armd::api::create_router_calibration(sup, up.clone(), GraspDeps::http(up, None), armd::diagnostics::DiagnosticsState::global(), deps, armd::auth::OperatorAuth::disabled());
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });
    Rig { base: format!("http://{addr}/api/v1/calibration/sessions"), http: reqwest::Client::new(), dir }
}

fn session(kind: &str, n_views: usize) -> Value {
    let views: Vec<Value> = (0..n_views).map(|i| json!({"markers": [{"id": 0, "corners": [[1.0, 2.0], [3.0, 4.0], [5.0, 6.0], [7.0, 8.0]]}],
        "q": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6], "raw": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6], "ds": i % 2})).collect();
    json!({"kind": kind, "created_at": "2026-11-19T10:00:00.000Z", "board": {"markerSide": 0.025}, "views": views,
        "summary": {"heldout_mm": [2.6, 7.7]}})
}

async fn post(r: &Rig, body: &Value) -> (u16, Value) {
    let res = r.http.post(&r.base).json(body).send().await.unwrap();
    let status = res.status().as_u16();
    (status, res.json().await.unwrap_or(Value::Null))
}

async fn list(r: &Rig) -> Vec<Value> {
    let v: Value = r.http.get(&r.base).send().await.unwrap().json().await.unwrap();
    v["sessions"].as_array().unwrap().clone()
}

#[tokio::test]
async fn store_list_get_overwrite_delete() {
    let r = rig().await;
    assert!(list(&r).await.is_empty(), "no sessions before the first store");
    let (s, a) = post(&r, &session("fine", 4)).await;
    assert_eq!(s, 200, "{a}");
    let id = a["id"].as_str().unwrap().to_string();
    assert!(armd::recording::is_ulid(&id), "new id is a ULID: {id}");
    assert_eq!(a["kind"], "fine");
    assert_eq!(a["views"], 4);
    assert_eq!(a["created_at"], "2026-11-19T10:00:00.000Z");
    assert!(a["size"].as_u64().unwrap() > 100);
    assert!(a.get("stored_seq").is_none(), "internal ordering key not exposed");

    let l = list(&r).await;
    assert_eq!(l.len(), 1);
    assert_eq!(l[0]["id"], id.as_str());
    assert_eq!(l[0]["summary"]["heldout_mm"][1], 7.7);

    let got: Value = r.http.get(format!("{}/{id}", r.base)).send().await.unwrap().json().await.unwrap();
    assert_eq!(got["id"], id.as_str());
    assert_eq!(got["views"].as_array().unwrap().len(), 4);
    assert_eq!(got["views"][1]["q"][5], 0.6);
    assert_eq!(got["board"]["markerSide"], 0.025);

    // overwrite: same id, more views, still one session
    let mut b = session("fine", 9);
    b["id"] = json!(id);
    b["levels"] = json!([{"level": "L0", "ab": 2.85, "ba": 3.33}]);
    let (s, a2) = post(&r, &b).await;
    assert_eq!(s, 200, "{a2}");
    assert_eq!(a2["id"], id.as_str());
    let l = list(&r).await;
    assert_eq!(l.len(), 1, "overwrite keeps one session");
    assert_eq!(l[0]["views"], 9);
    let got: Value = r.http.get(format!("{}/{id}", r.base)).send().await.unwrap().json().await.unwrap();
    assert_eq!(got["levels"][0]["level"], "L0");

    // a second session is listed first
    let (_, c) = post(&r, &session("camera", 2)).await;
    let l = list(&r).await;
    assert_eq!(l.iter().map(|m| m["id"].as_str().unwrap().to_string()).collect::<Vec<_>>(), vec![c["id"].as_str().unwrap().to_string(), id.clone()]);
    assert_eq!(l[0]["kind"], "camera");

    let res = r.http.delete(format!("{}/{id}", r.base)).send().await.unwrap();
    assert_eq!(res.status().as_u16(), 200);
    assert_eq!(r.http.get(format!("{}/{id}", r.base)).send().await.unwrap().status().as_u16(), 404);
    assert_eq!(r.http.delete(format!("{}/{id}", r.base)).send().await.unwrap().status().as_u16(), 404);
    assert_eq!(list(&r).await.len(), 1);
    // no temp files left behind
    let names: Vec<String> = std::fs::read_dir(r.dir.path().join("sessions")).unwrap().flatten().map(|e| e.file_name().to_string_lossy().into_owned()).collect();
    assert!(names.iter().all(|n| !n.starts_with('.')), "{names:?}");
    assert_eq!(names.len(), 2, "data + meta of the remaining session: {names:?}");
}

#[tokio::test]
async fn keeps_the_newest_ten() {
    let r = rig().await;
    let mut ids = Vec::new();
    for _ in 0..12 { let (s, a) = post(&r, &session("camera", 1)).await; assert_eq!(s, 200); ids.push(a["id"].as_str().unwrap().to_string()); }
    let l: Vec<String> = list(&r).await.iter().map(|m| m["id"].as_str().unwrap().to_string()).collect();
    assert_eq!(l.len(), 10);
    let mut newest: Vec<String> = ids[2..].to_vec();
    newest.reverse();
    assert_eq!(l, newest, "newest first, the two oldest removed");
    for old in &ids[..2] { assert_eq!(r.http.get(format!("{}/{old}", r.base)).send().await.unwrap().status().as_u16(), 404); }
    // overwriting the oldest kept session makes it the newest; the cap still holds
    let mut b = session("camera", 3);
    b["id"] = json!(ids[2]);
    post(&r, &b).await;
    let (_, n) = post(&r, &session("fine", 1)).await;
    let l: Vec<String> = list(&r).await.iter().map(|m| m["id"].as_str().unwrap().to_string()).collect();
    assert_eq!(l.len(), 10);
    assert_eq!(l[0], n["id"].as_str().unwrap());
    assert_eq!(l[1], ids[2], "the overwritten session moved to the front");
    assert!(!l.contains(&ids[3]), "the next oldest was removed");
    assert_eq!(std::fs::read_dir(r.dir.path().join("sessions")).unwrap().count(), 20);
}

#[tokio::test]
async fn refuses_bad_bodies_and_the_size_limit() {
    let r = rig().await;
    for (body, why) in [
        (json!({"kind": "other", "views": []}), "kind"),
        (json!({"kind": "fine"}), "views"),
        (json!({"kind": "fine", "views": [], "id": "../x"}), "id"),
        (json!({"kind": "fine", "views": [], "created_at": 5}), "created_at"),
        (json!([1, 2]), "object"),
    ] {
        let (s, v) = post(&r, &body).await;
        assert_eq!(s, 400, "{why}: {v}");
        assert_eq!(v["code"], "INVALID");
    }
    let res = r.http.post(&r.base).header("content-type", "application/json").body("{not json").send().await.unwrap();
    assert_eq!(res.status().as_u16(), 400);
    // just under the limit is stored, over it is refused with 413
    let pad = |n: usize| { let mut b = json!({"kind": "fine", "views": [], "pad": ""}); let base = b.to_string().len(); b["pad"] = json!("x".repeat(n - base)); b };
    let ok = pad(armd::calibration_sessions::MAX_BODY_BYTES);
    assert_eq!(ok.to_string().len(), armd::calibration_sessions::MAX_BODY_BYTES);
    let (s, v) = post(&r, &ok).await;
    assert_eq!(s, 200, "{v}");
    let res = r.http.post(&r.base).json(&pad(armd::calibration_sessions::MAX_BODY_BYTES + 1)).send().await.unwrap();
    assert_eq!(res.status().as_u16(), 413);
    assert_eq!(list(&r).await.len(), 1);
    assert_eq!(r.http.get(format!("{}/bad.id", r.base)).send().await.unwrap().status().as_u16(), 400);
    // cross-origin writes are refused like every other non-GET route
    let res = r.http.post(&r.base).header("origin", "http://evil.example").json(&session("fine", 1)).send().await.unwrap();
    assert_eq!(res.status().as_u16(), 403);
}
