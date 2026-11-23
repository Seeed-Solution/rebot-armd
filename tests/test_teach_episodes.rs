//! Teach v1 episode store through the supervisor and the API: ULID episodes with sidecars,
//! REST listing / download, lease-free WS commands, disk guard, length cap, boot recovery,
//! replay of a new-store episode, tick timing while recording.
mod common;

use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

use armd::model::ArmConfig;
use armd::ArmSupervisor;
use serde_json::{json, Value};
use tokio_tungstenite::connect_async;

fn profile() -> ArmConfig { ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() } }

static REQ: AtomicU64 = AtomicU64::new(1);

/// A WS command without a lease (teach lease-free paths).
async fn free_cmd(ws: &mut common::TestSocket, path: &str, body: Value) -> Value {
    common::ws_reply(ws, json!({"type":"command", "request_id": REQ.fetch_add(1, Ordering::Relaxed), "path": path, "body": body}),
        &["command_ack", "command_reject"]).await
}

async fn get(addr: std::net::SocketAddr, path: &str) -> (u16, Value) {
    let r = reqwest::get(format!("http://{addr}{path}")).await.unwrap();
    let status = r.status().as_u16();
    (status, r.json().await.unwrap_or(Value::Null))
}

/// Record `ticks` control ticks through the legacy REST routes; returns the episode id.
async fn record_rest(addr: std::net::SocketAddr, sup: &ArmSupervisor, ticks: usize) -> String {
    let client = reqwest::Client::new();
    let r: Value = client.post(format!("http://{addr}/api/v1/dataset/record/start")).json(&json!({})).send().await.unwrap().json().await.unwrap();
    let id = r["episode_id"].as_str().unwrap().to_string();
    for _ in 0..ticks { sup.tick(0.02).await; }
    let r: Value = client.post(format!("http://{addr}/api/v1/dataset/record/save")).send().await.unwrap().json().await.unwrap();
    assert_eq!(r["status"], "saved", "{r}");
    id
}

#[tokio::test]
async fn ulid_episode_sidecar_listing_download_and_lease_free_commands() {
    let tmp = tempfile::tempdir().unwrap();
    let (addr, sup) = common::spawn_server(profile()).await;
    sup.set_episodes_dir(Some(tmp.path().to_path_buf())).await;
    let id = record_rest(addr, &sup, 30).await;
    assert!(armd::recording::is_ulid(&id), "{id}");
    let sc: Value = serde_json::from_str(&std::fs::read_to_string(tmp.path().join(format!("{id}.json"))).unwrap()).unwrap();
    for key in ["id", "dataset", "task", "source", "outcome", "recorded_at", "duration_s", "frames", "joint_names", "units",
                "joint_zero", "profile_id", "armd_version", "fps_nominal"] {
        assert!(sc.get(key).is_some(), "sidecar lacks {key}: {sc}");
    }
    assert_eq!((sc["units"].as_str(), sc["dataset"].as_str(), sc["outcome"].as_str(), sc["frames"].as_u64()), (Some("rad"), Some("default"), Some("ok"), Some(30)));
    assert_eq!(sc["joint_zero"]["offsets_rad"].as_array().unwrap().len(), 7);
    assert!(!tmp.path().join(format!("{id}.meta.json")).exists());

    let (s, list) = get(addr, "/api/v1/episodes").await;
    assert_eq!(s, 200);
    assert_eq!(list["episodes"][0]["id"], id.as_str());
    assert_eq!(list["episodes"][0]["stale"], false);
    let (s, one) = get(addr, &format!("/api/v1/episodes/{id}")).await;
    assert_eq!((s, one["frames"].as_u64()), (200, Some(30)));
    let (s, missing) = get(addr, "/api/v1/episodes/01ARZ3NDEKTSV4RRFFQ69G5FAV").await;
    assert_eq!((s, missing["code"].as_str()), (404, Some("EPISODE_NOT_FOUND")));
    let data = reqwest::get(format!("http://{addr}/api/v1/episodes/{id}/data")).await.unwrap();
    assert_eq!(data.headers()["content-type"], "application/vnd.apache.parquet");
    let bytes = data.bytes().await.unwrap();
    let dl = tmp.path().join("dl.bin");
    std::fs::write(&dl, &bytes).unwrap();
    assert_eq!(armd::read_frames_from_parquet(&dl).unwrap().len(), 30);
    std::fs::remove_file(&dl).unwrap();
    // the legacy listing shows the same episode
    let (_, legacy) = get(addr, "/api/v1/dataset/episodes").await;
    assert_eq!((legacy["episodes"][0]["id"].as_str(), legacy["episodes"][0]["frames"].as_u64()), (Some(id.as_str()), Some(30)));

    // lease-free WS commands (this connection holds no lease)
    let (mut ws, _) = connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
    let r = free_cmd(&mut ws, "/api/v1/datasets/create", json!({"name": "抓取 demo"})).await;
    assert_eq!(r["type"], "command_ack", "{r}");
    assert_eq!(free_cmd(&mut ws, "/api/v1/datasets/create", json!({"name": "抓取 demo"})).await["code"], "DATASET_EXISTS");
    assert_eq!(free_cmd(&mut ws, "/api/v1/datasets/create", json!({"name": "a/b"})).await["code"], "INVALID_COMMAND");
    let r = free_cmd(&mut ws, "/api/v1/episodes/update", json!({"id": id, "task": "pick the cube", "dataset": "抓取 demo"})).await;
    assert_eq!((r["type"].as_str(), r["episode"]["dataset"].as_str(), r["episode"]["task"].as_str()), (Some("command_ack"), Some("抓取 demo"), Some("pick the cube")), "{r}");
    assert_eq!(free_cmd(&mut ws, "/api/v1/episodes/update", json!({"id": id, "dataset": "nope"})).await["code"], "DATASET_NOT_FOUND");
    let (_, ds) = get(addr, "/api/v1/datasets").await;
    let demo = ds["datasets"].as_array().unwrap().iter().find(|d| d["name"] == "抓取 demo").unwrap().clone();
    assert_eq!(demo["episodes"], 1);
    assert!(demo["bytes"].as_u64().unwrap() > 0 && demo["duration_s"].as_f64().unwrap() >= 0.0);
    let (_, filtered) = get(addr, &format!("/api/v1/episodes?dataset={}", urlencode("抓取 demo"))).await;
    assert_eq!(filtered["episodes"].as_array().unwrap().len(), 1);
    let (_, filtered) = get(addr, "/api/v1/episodes?dataset=default").await;
    assert!(filtered["episodes"].as_array().unwrap().is_empty());

    assert_eq!(free_cmd(&mut ws, "/api/v1/datasets/delete", json!({"name": "抓取 demo"})).await["code"], "DATASET_NOT_EMPTY");
    let id2 = record_rest(addr, &sup, 5).await;
    let r = free_cmd(&mut ws, "/api/v1/episodes/delete", json!({"id": id2})).await;
    assert_eq!((r["type"].as_str(), r["deleted"].as_str()), (Some("command_ack"), Some(id2.as_str())), "{r}");
    assert!(!tmp.path().join(format!("{id2}.parquet")).exists() && !tmp.path().join(format!("{id2}.json")).exists());
    assert_eq!(free_cmd(&mut ws, "/api/v1/episodes/delete", json!({"id": id2})).await["code"], "EPISODE_NOT_FOUND");
    let r = free_cmd(&mut ws, "/api/v1/datasets/delete", json!({"name": "抓取 demo", "force": true})).await;
    assert_eq!(r["deleted_episodes"], json!([id]), "{r}");
    let (_, list) = get(addr, "/api/v1/episodes").await;
    assert!(list["episodes"].as_array().unwrap().is_empty());
}

fn urlencode(s: &str) -> String {
    s.bytes().map(|b| if b.is_ascii_alphanumeric() { (b as char).to_string() } else { format!("%{b:02X}") }).collect()
}

#[tokio::test]
async fn disk_guard_refuses_the_start_and_stops_a_recording() {
    let tmp = tempfile::tempdir().unwrap();
    let (addr, sup) = common::spawn_server(profile()).await;
    sup.set_episodes_dir(Some(tmp.path().to_path_buf())).await;
    let store = sup.episode_store().await;
    store.set_free_bytes_override(Some(150 * 1024 * 1024));
    let r = reqwest::Client::new().post(format!("http://{addr}/api/v1/dataset/record/start")).json(&json!({})).send().await.unwrap();
    assert_eq!(r.status().as_u16(), 507);
    let body: Value = r.json().await.unwrap();
    assert_eq!(body["code"], "RECORD_NO_SPACE", "{body}");

    store.set_free_bytes_override(Some(300 * 1024 * 1024));
    sup.start_recording(None, None).await.unwrap();
    for _ in 0..10 { sup.tick(0.02).await; }
    store.set_free_bytes_override(Some(90 * 1024 * 1024));
    // the monitor stops it (saved) within its free-space period
    let id = wait_saved(&sup, Duration::from_secs(3)).await;
    let sc = sup.episode_store().await.get(&id).unwrap();
    assert_eq!((sc.stopped_by.as_deref(), sc.truncated, sc.frames), (Some("low_space"), false, 10));
}

/// Waits until the recorder is idle and returns the newest episode id.
async fn wait_saved(sup: &ArmSupervisor, limit: Duration) -> String {
    let until = std::time::Instant::now() + limit;
    loop {
        assert!(std::time::Instant::now() < until, "recording did not stop");
        if sup.inner().read().await.recording.is_none() && sup.get_recording_state().await.0 != armd::EpisodeState::Finalizing { break; }
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    let st = sup.episode_store().await;
    st.list_episodes(None, Some(1), None, &[]).first().map(|e| e.sidecar.id.clone()).expect("an episode")
}

#[tokio::test]
async fn length_cap_stops_and_saves_truncated() {
    let tmp = tempfile::tempdir().unwrap();
    let sup = ArmSupervisor::new(profile());
    sup.set_episodes_dir(Some(tmp.path().to_path_buf())).await;
    sup.inner().write().await.max_episode_s = 0.1; // 5 ticks at 50 Hz (+1)
    sup.start_recording(None, None).await.unwrap();
    for _ in 0..40 { sup.tick(0.02).await; }
    assert_eq!(sup.get_recording_state().await.1, 6, "the tick stops appending at the cap");
    let id = wait_saved(&sup, Duration::from_secs(3)).await;
    let sc = sup.episode_store().await.get(&id).unwrap();
    assert_eq!((sc.truncated, sc.stopped_by.as_deref(), sc.outcome.as_str(), sc.frames), (true, Some("max_duration"), "ok", 6));
    // a new recording can start
    sup.start_recording(None, None).await.unwrap();
}

#[tokio::test]
async fn crash_leftovers_are_recovered_at_boot() {
    let tmp = tempfile::tempdir().unwrap();
    let dir = tmp.path();
    {
        let sup = ArmSupervisor::new(profile());
        sup.set_episodes_dir(Some(dir.to_path_buf())).await;
        sup.start_recording(None, None).await.unwrap();
        for _ in 0..8 { sup.tick(0.02).await; }
        sup.save_recording().await.unwrap();
    }
    let id = std::fs::read_dir(dir).unwrap().filter_map(|e| e.ok()).map(|e| e.path())
        .find(|p| p.extension().is_some_and(|x| x == "json")).unwrap().file_stem().unwrap().to_string_lossy().into_owned();
    // crash after the parquet rename: no sidecar; crash during a write: temp file
    std::fs::remove_file(dir.join(format!("{id}.json"))).unwrap();
    let t = armd::recording::new_ulid();
    std::fs::write(dir.join(format!("{t}.parquet.tmp")), b"partial").unwrap();
    // a legacy recording (before the store) is adopted
    let frames: Vec<armd::RecordFrame> = (0..4).map(|k| armd::RecordFrame { timestamp: k as f64 * 0.02, observation_state: common::INSPECT.to_vec(), action: common::INSPECT.to_vec() }).collect();
    armd::write_frames_to_parquet(&dir.join("episode_20261101_101010.parquet"), &frames).unwrap();

    let sup = ArmSupervisor::new(profile());
    sup.set_episodes_dir(Some(dir.to_path_buf())).await;
    let report = sup.ensure_episodes_loaded().await.unwrap().unwrap();
    println!("boot recovery: {report:?}");
    assert_eq!(report.sidecars_rebuilt, vec![id.clone()]);
    assert_eq!(report.temps_removed, vec![format!("{t}.parquet.tmp")]);
    assert_eq!(report.legacy_adopted, vec!["episode_20261101_101010".to_string()]);
    let st = sup.episode_store().await;
    assert_eq!(st.get(&id).unwrap().frames, 8);
    assert_eq!(st.get("episode_20261101_101010").unwrap().recovered.as_deref(), Some("legacy"));
    assert!(dir.join(format!("{id}.json")).exists() && !dir.join(format!("{t}.parquet.tmp")).exists());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn new_store_episode_replays_by_id_and_stale_needs_allow_stale() {
    let tmp = tempfile::tempdir().unwrap();
    let mut inner = common::sim_at(profile(), &common::INSPECT);
    inner.episodes = armd::recording::EpisodeStore::new(Some(tmp.path().to_path_buf()));
    let sup = ArmSupervisor::new(profile());
    *sup.inner().write().await = inner;
    sup.start_recording(None, None).await.unwrap();
    // a small joint move while recording
    let epoch = sup.get_observation().await.epoch_id;
    let mut target = common::INSPECT.to_vec();
    target[0] += 0.1;
    sup.inner().write().await.jog_action(target, Some(1.0), Some(epoch), "joint_move").unwrap();
    for _ in 0..70 {
        sup.tick(0.02).await;
        let mut i = sup.inner().write().await;
        i.measured_positions = i.commanded_positions.clone();
        // recorded timestamps come from the wall clock (start_time); advance it by the tick so the
        // episode keeps the motion's speed. Without this, 70 ticks in a few ms recorded the jog
        // far above the speed limit and the replay was refused with REPLAY_TOO_FAST in ~30 % of runs.
        i.start_time = i.start_time.checked_sub(Duration::from_millis(20)).unwrap();
    }
    let path = sup.save_recording().await.unwrap();
    let id = path.file_stem().unwrap().to_string_lossy().into_owned();
    assert!(armd::recording::is_ulid(&id));
    let req = |allow_stale| armd::replay::ReplayRequest { episode: id.clone(), speed: 1.0, gripper: false, tag: None, allow_stale };
    let (aid, _) = sup.inner().write().await.start_replay(req(false), epoch).unwrap();
    let mut done = None;
    for _ in 0..400 {
        sup.tick(0.02).await;
        let mut i = sup.inner().write().await;
        i.measured_positions = i.commanded_positions.clone();
        if let Some(a) = i.last_action.clone().filter(|a| a.id == aid && !a.is_running()) { done = Some(a); break; }
    }
    let a = done.expect("replay finished");
    assert_eq!((a.state.as_str(), a.kind.as_str(), a.episode.as_deref()), ("done", "replay", Some(id.as_str())), "{a:?}");

    // a joint zero change of J1 by 1 deg makes the episode stale
    {
        let mut i = sup.inner().write().await;
        i.joint_zero_offsets[0] = 1f32.to_radians();
    }
    let err = sup.inner().write().await.start_replay(req(false), epoch).unwrap_err();
    let api = armd::api::rest::ApiError::from(err);
    assert_eq!(api.code.as_deref(), Some("DATA_STALE"));
    let st = sup.episode_store().await;
    assert!(st.view(&id, &sup.inner().read().await.joint_zero_offsets).unwrap().stale);
    sup.inner().write().await.start_replay(req(true), epoch).unwrap();
}

/// The control tick with a recording running does what it did before teach v1 (append one
/// frame); saving a long episode happens off the lock. Prints the timing numbers.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn tick_timing_while_recording_and_saving() {
    let tmp = tempfile::tempdir().unwrap();
    let sup = std::sync::Arc::new(ArmSupervisor::new(profile()));
    sup.set_episodes_dir(Some(tmp.path().to_path_buf())).await;
    for _ in 0..1000 { sup.tick(0.02).await; }
    let idle = sup.tick_stats();
    sup.start_recording(None, None).await.unwrap();
    for _ in 0..1000 { sup.tick(0.02).await; }
    let rec = sup.tick_stats();
    // a 600 s episode at 50 Hz is 30000 frames; record that many, then save while ticking
    for _ in 0..29000 { sup.tick(0.02).await; }
    let saver = { let s = sup.clone(); tokio::spawn(async move { s.save_recording().await }) };
    let mut during = 0;
    while !saver.is_finished() { sup.tick(0.02).await; during += 1; tokio::task::yield_now().await; }
    saver.await.unwrap().unwrap();
    let saving = sup.tick_stats();
    println!("TICK idle: tick p50/p99/max {}/{}/{} us, lock {}/{}/{} us", idle.tick_us.p50, idle.tick_us.p99, idle.tick_us.max, idle.lock_us.p50, idle.lock_us.p99, idle.lock_us.max);
    println!("TICK recording: tick p50/p99/max {}/{}/{} us, lock {}/{}/{} us", rec.tick_us.p50, rec.tick_us.p99, rec.tick_us.max, rec.lock_us.p50, rec.lock_us.p99, rec.lock_us.max);
    println!("TICK last 1000 incl. {during} ticks during a 30000-frame save: tick p50/p99/max {}/{}/{} us, lock {}/{}/{} us",
        saving.tick_us.p50, saving.tick_us.p99, saving.tick_us.max, saving.lock_us.p50, saving.lock_us.p99, saving.lock_us.max);
    assert!(rec.lock_us.p99 < 2000 && saving.lock_us.p99 < 5000, "{rec:?} {saving:?}");
}

// ---- recorder ownership (separate from the motion lease) ----

async fn connect(addr: std::net::SocketAddr) -> common::TestSocket {
    connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap().0
}

#[tokio::test]
async fn recorder_ownership_other_clients_cannot_stop_and_disconnect_discards() {
    let tmp = tempfile::tempdir().unwrap();
    let (addr, sup) = common::spawn_server(profile()).await;
    sup.set_episodes_dir(Some(tmp.path().to_path_buf())).await;
    // a lease holder (teleop) and two passive recorders without lease
    let (mut lease_ws, _generation) = common::controller(addr).await;
    let mut a = connect(addr).await;
    let mut b = connect(addr).await;
    // no client_id (none in the message, no lease on this connection): refused
    let r = free_cmd(&mut a, "/api/v1/record/start", json!({"dataset": "default", "task": "wipe", "source": "hand_guide"})).await;
    assert_eq!(r["code"], "INVALID_COMMAND", "{r}");
    let r = free_cmd(&mut b, "/api/v1/record/start", json!({"dataset": "missing"})).await;
    assert_eq!(r["code"], "INVALID_COMMAND", "{r}");
    let start = |ws: &'static str| json!({"type":"command", "request_id": REQ.fetch_add(1, Ordering::Relaxed), "client_id": ws,
        "path": "/api/v1/record/start", "body": {"task": "wipe", "source": "hand_guide"}});
    let r = common::ws_reply(&mut a, start("page-a"), &["command_ack", "command_reject"]).await;
    assert_eq!((r["type"].as_str(), r["owner"].as_str()), (Some("command_ack"), Some("page-a")), "{r}");
    let rid = r["recording_id"].as_str().unwrap().to_string();
    assert!(armd::recording::is_ulid(&rid));
    let (_, list) = get(addr, "/api/v1/episodes").await;
    let rec = &list["recording"];
    assert_eq!((rec["recording_id"].as_str(), rec["owner"].as_str(), rec["dataset"].as_str(), rec["task"].as_str()),
        (Some(rid.as_str()), Some("page-a"), Some("default"), Some("wipe")), "{rec}");
    assert!(rec["started_at"].as_str().is_some_and(|t| t.ends_with('Z')), "{rec}");
    // a second recording: RECORDER_BUSY with the owner
    let r = common::ws_reply(&mut b, start("page-b"), &["command_ack", "command_reject"]).await;
    assert_eq!((r["code"].as_str(), r["owner"].as_str()), (Some("RECORDER_BUSY"), Some("page-a")), "{r}");
    // the legacy REST routes and another client cannot stop it
    let res = reqwest::Client::new().post(format!("http://{addr}/api/v1/dataset/record/save")).send().await.unwrap();
    assert_eq!(res.status().as_u16(), 409);
    assert_eq!(res.json::<Value>().await.unwrap()["code"], "RECORDER_NOT_OWNER");
    let stop = |client: &'static str, save: bool| json!({"type":"command", "request_id": REQ.fetch_add(1, Ordering::Relaxed), "client_id": client,
        "path": "/api/v1/record/stop", "body": {"save": save}});
    let r = common::ws_reply(&mut b, stop("page-b", true), &["command_ack", "command_reject"]).await;
    assert_eq!((r["code"].as_str(), r["owner"].as_str()), (Some("RECORDER_NOT_OWNER"), Some("page-a")), "{r}");
    // the lease holder neither (ownership is not the lease)
    let r = common::ws_reply(&mut lease_ws, stop("test_controller", true), &["command_ack", "command_reject"]).await;
    assert_eq!(r["code"], "RECORDER_NOT_OWNER", "{r}");
    for _ in 0..10 { sup.tick(0.02).await; }
    let r = common::ws_reply(&mut a, stop("page-a", true), &["command_ack", "command_reject"]).await;
    assert_eq!((r["type"].as_str(), r["episode"]["id"].as_str(), r["episode"]["source"].as_str(), r["episode"]["task"].as_str()),
        (Some("command_ack"), Some(rid.as_str()), Some("hand_guide"), Some("wipe")), "{r}");
    assert_eq!(r["episode"]["frames"], 10);

    // owner disconnect: discarded after 5 s
    let r = common::ws_reply(&mut a, start("page-a"), &["command_ack", "command_reject"]).await;
    let rid2 = r["recording_id"].as_str().unwrap().to_string();
    for _ in 0..5 { sup.tick(0.02).await; }
    drop(a);
    tokio::time::sleep(Duration::from_millis(4000)).await;
    assert!(sup.inner().read().await.recording.as_ref().is_some_and(|r| r.id == rid2 && r.owner_lost_at.is_some()), "kept for 5 s");
    tokio::time::sleep(Duration::from_millis(1600)).await;
    assert!(sup.inner().read().await.recording.is_none(), "discarded 5 s after the owner left");
    assert!(!tmp.path().join(format!("{rid2}.parquet")).exists());
    assert_eq!(sup.get_recording_state().await.0, armd::EpisodeState::Discarded);

    // a reconnect with the same client id within 5 s takes the recording back
    let mut c = connect(addr).await;
    let r = common::ws_reply(&mut c, start("page-c"), &["command_ack", "command_reject"]).await;
    let rid3 = r["recording_id"].as_str().unwrap().to_string();
    for _ in 0..5 { sup.tick(0.02).await; }
    drop(c);
    tokio::time::sleep(Duration::from_millis(500)).await;
    let mut c2 = connect(addr).await;
    let r = common::ws_reply(&mut c2, stop("page-c", true), &["command_ack", "command_reject"]).await;
    assert_eq!(r["episode"]["id"].as_str(), Some(rid3.as_str()), "{r}");
}

// ---- import ----

/// multipart/form-data body with a `parquet` and a `sidecar` part.
fn multipart(parquet: &[u8], sidecar: &Value) -> (String, Vec<u8>) {
    let boundary = "teachboundary7MA4YWxkTrZu0gW";
    let mut body = Vec::new();
    body.extend_from_slice(format!("--{boundary}\r\nContent-Disposition: form-data; name=\"sidecar\"; filename=\"e.json\"\r\nContent-Type: application/json\r\n\r\n").as_bytes());
    body.extend_from_slice(sidecar.to_string().as_bytes());
    body.extend_from_slice(format!("\r\n--{boundary}\r\nContent-Disposition: form-data; name=\"parquet\"; filename=\"e.parquet\"\r\nContent-Type: application/octet-stream\r\n\r\n").as_bytes());
    body.extend_from_slice(parquet);
    body.extend_from_slice(format!("\r\n--{boundary}--\r\n").as_bytes());
    (format!("multipart/form-data; boundary={boundary}"), body)
}

async fn import(addr: std::net::SocketAddr, parquet: &[u8], sidecar: &Value) -> (u16, Value) {
    let (ct, body) = multipart(parquet, sidecar);
    let r = reqwest::Client::new().post(format!("http://{addr}/api/v1/episodes/import")).header("content-type", ct).body(body).send().await.unwrap();
    let status = r.status().as_u16();
    (status, r.json().await.unwrap_or(Value::Null))
}

#[tokio::test]
async fn import_validates_units_joint_order_limits_and_zero() {
    let tmp = tempfile::tempdir().unwrap();
    let (addr, sup) = common::spawn_server(profile()).await;
    sup.set_episodes_dir(Some(tmp.path().to_path_buf())).await;
    let names: Vec<String> = sup.inner().read().await.joint_names();
    let file = tmp.path().join("src.bin");
    let write = |q: &[f32]| {
        let frames: Vec<armd::RecordFrame> = (0..50).map(|k| {
            let mut s = q.to_vec();
            s[0] += k as f32 * 0.002;
            armd::RecordFrame { timestamp: 100.0 + k as f64 * 0.02, observation_state: s.clone(), action: s }
        }).collect();
        armd::write_frames_to_parquet(&file, &frames).unwrap();
        std::fs::read(&file).unwrap()
    };
    let rad = write(&common::INSPECT);
    let good = json!({"units": "rad", "joint_names": names, "dataset": "pc", "task": "pour",
        "joint_zero": {"offsets_rad": vec![0.0; 7], "rev": "abc"}, "recorded_at": "2026-11-18T10:00:00Z"});

    // degrees: refused by the units field, and by the limits when labelled rad
    let mut deg = good.clone();
    deg["units"] = json!("deg");
    let (s, r) = import(addr, &rad, &deg).await;
    assert_eq!((s, r["code"].as_str(), r["reason"].as_str()), (422, Some("IMPORT_INVALID"), Some("units")), "{r}");
    let deg_data = write(&common::INSPECT.map(f32::to_degrees));
    let (_, r) = import(addr, &deg_data, &good).await;
    assert_eq!(r["reason"], "soft_limit", "{r}");
    // wrong joint order
    let mut order = good.clone();
    let mut swapped = names.clone();
    swapped.swap(1, 2);
    order["joint_names"] = json!(swapped);
    let (_, r) = import(addr, &rad, &order).await;
    assert_eq!(r["reason"], "joint_order", "{r}");
    // not a parquet / not JSON
    let (_, r) = import(addr, b"PAR1 nope", &good).await;
    assert_eq!(r["reason"], "schema", "{r}");
    assert!(sup.episode_store().await.list_episodes(None, None, None, &[]).is_empty(), "nothing stored");

    // accepted: new id, source import, dataset created, not stale
    let (s, r) = import(addr, &rad, &good).await;
    assert_eq!(s, 200, "{r}");
    let id = r["id"].as_str().unwrap().to_string();
    assert!(armd::recording::is_ulid(&id) && r["stale"] == false);
    let (_, one) = get(addr, &format!("/api/v1/episodes/{id}")).await;
    assert_eq!((one["source"].as_str(), one["dataset"].as_str(), one["task"].as_str(), one["frames"].as_u64(), one["stale"].as_bool()),
        (Some("import"), Some("pc"), Some("pour"), Some(50), Some(false)));
    assert!(sup.episode_store().await.has_dataset("pc"));
    // another joint zero: stored, stale
    let mut z = good.clone();
    z["joint_zero"] = json!({"offsets_rad": [0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0], "rev": "old"});
    let (s, r) = import(addr, &rad, &z).await;
    assert_eq!((s, r["stale"].as_bool()), (200, Some(true)), "{r}");
    let (_, one) = get(addr, &format!("/api/v1/episodes/{}", r["id"].as_str().unwrap())).await;
    assert_eq!(one["stale"], true);
}

/// The monitor stops only the recording it watched: a stop for a recording that was
/// replaced meanwhile is refused under the write guard and the new recording keeps running.
#[tokio::test]
async fn a_monitor_stop_never_touches_a_replacement_recording() {
    let tmp = tempfile::tempdir().unwrap();
    let sup = ArmSupervisor::new(profile());
    sup.set_episodes_dir(Some(tmp.path().to_path_buf())).await;
    sup.start_recording(None, None).await.unwrap();
    for _ in 0..3 { sup.tick(0.02).await; }
    let current = sup.inner().read().await.recording.as_ref().map(|r| r.id.clone()).unwrap();
    let stale = armd::teach::Stopper::Monitor { reason: "max_duration", recording: format!("{current}-replaced") };
    let e = sup.stop_recording_as(stale, true, None, None).await.expect_err("a stale monitor stop");
    assert!(format!("{e:?}").contains("RECORDING_REPLACED"), "{e:?}");
    assert_eq!(sup.inner().read().await.recording.as_ref().map(|r| r.id.clone()), Some(current.clone()), "the running recording was touched");
    let own = armd::teach::Stopper::Monitor { reason: "max_duration", recording: current };
    assert!(sup.stop_recording_as(own, true, None, None).await.is_ok());
    assert!(sup.inner().read().await.recording.is_none());
}
