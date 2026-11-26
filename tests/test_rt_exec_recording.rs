//! Adapter 2 dual-owner parity and RT proof suite:
//! - Same teach recording / episodes / datasets / import / delete / run_sequence(record) scenarios
//!   verified identically on BOTH `Owner::Legacy` and `Owner::Executor`.
//! - Dedicated RT zero-allocation, zero-SupervisorLock, zero-forbidden-lock, stream saturation,
//!   delayed-writer, idempotent publish, and config-change interleaving proofs.
mod common;

#[global_allocator]
static ALLOC: armd::diagnostics::alloc::TimedAlloc = armd::diagnostics::alloc::TimedAlloc;

use std::net::SocketAddr;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use armd::diagnostics::alloc::AllocCounts;
use armd::model::ArmConfig;
use armd::motion::ActionStatus;
use armd::recording::{EpisodeSidecar, EpisodeState, JointZeroStamp, RecordFrame};
use armd::rt_core::executor::{ExecutorConfig, PanicPolicy, RtExecutorHandle};
use armd::rt_core::recorder::RECORDER_STREAM_CAPACITY;
use armd::{create_router, create_router_with_auth, ArmSupervisor, OperatorAuth};
use serde_json::{json, Value};
use tokio::net::TcpListener;
use tokio_tungstenite::connect_async;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Owner {
    Legacy,
    Executor,
}

impl Owner {
    const BOTH: [Self; 2] = [Self::Legacy, Self::Executor];
}

fn profile() -> ArmConfig {
    ArmConfig {
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    }
}

static REQ: AtomicU64 = AtomicU64::new(1);

async fn free_cmd(ws: &mut common::TestSocket, path: &str, body: Value) -> Value {
    common::ws_reply(
        ws,
        json!({
            "type": "command",
            "request_id": REQ.fetch_add(1, Ordering::Relaxed),
            "path": path,
            "body": body,
        }),
        &["command_ack", "command_reject"],
    )
    .await
}

async fn client_cmd(
    ws: &mut common::TestSocket,
    client_id: &str,
    path: &str,
    body: Value,
) -> Value {
    common::ws_reply(
        ws,
        json!({
            "type": "command",
            "request_id": REQ.fetch_add(1, Ordering::Relaxed),
            "client_id": client_id,
            "path": path,
            "body": body,
        }),
        &["command_ack", "command_reject"],
    )
    .await
}

async fn get_json(addr: SocketAddr, path: &str) -> (u16, Value) {
    let r = reqwest::get(format!("http://{addr}{path}")).await.unwrap();
    let status = r.status().as_u16();
    (status, r.json().await.unwrap_or(Value::Null))
}

fn multipart(parquet: &[u8], sidecar: &Value) -> (String, Vec<u8>) {
    let boundary = "execrecboundary7MA4YWxkTrZu0gW";
    let mut body = Vec::new();
    body.extend_from_slice(
        format!(
            "--{boundary}\r\nContent-Disposition: form-data; name=\"sidecar\"; filename=\"e.json\"\r\nContent-Type: application/json\r\n\r\n"
        )
        .as_bytes(),
    );
    body.extend_from_slice(sidecar.to_string().as_bytes());
    body.extend_from_slice(
        format!(
            "\r\n--{boundary}\r\nContent-Disposition: form-data; name=\"parquet\"; filename=\"e.parquet\"\r\nContent-Type: application/octet-stream\r\n\r\n"
        )
        .as_bytes(),
    );
    body.extend_from_slice(parquet);
    body.extend_from_slice(format!("\r\n--{boundary}--\r\n").as_bytes());
    (format!("multipart/form-data; boundary={boundary}"), body)
}

struct DualRig {
    owner: Owner,
    addr: SocketAddr,
    sup: Arc<ArmSupervisor>,
    _tmp: tempfile::TempDir,
    exec: Option<RtExecutorHandle>,
    stop_ticker: Arc<AtomicBool>,
}

impl DualRig {
    async fn new(owner: Owner, period: Duration, auth: Option<OperatorAuth>) -> Self {
        let tmp = tempfile::tempdir().unwrap();
        let sup = Arc::new(ArmSupervisor::new(profile()));
        sup.set_episodes_dir(Some(tmp.path().to_path_buf())).await;
        if let Some(ref a) = auth {
            sup.set_operator_auth_enforced(a.is_enforced()).await;
        }
        let stop_ticker = Arc::new(AtomicBool::new(false));
        let exec = match owner {
            Owner::Legacy => {
                let s = sup.clone();
                let stop = stop_ticker.clone();
                tokio::spawn(async move {
                    let mut interval = tokio::time::interval(period);
                    while !stop.load(Ordering::Acquire) {
                        interval.tick().await;
                        s.tick(0.02).await;
                    }
                });
                let dl = Instant::now() + Duration::from_secs(3);
                while sup.hub().latest().is_none() {
                    assert!(Instant::now() < dl, "legacy ticker failed to publish");
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
                None
            }
            Owner::Executor => {
                let mut ecfg = ExecutorConfig::for_tests(period);
                ecfg.alloc_timing = true;
                ecfg.panic_policy = PanicPolicy::StopExecutor;
                let h = sup.start_executor(ecfg).await.unwrap();
                let dl = Instant::now() + Duration::from_secs(3);
                while sup.hub().latest().is_none() {
                    assert!(Instant::now() < dl, "executor failed to publish");
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
                Some(h)
            }
        };
        let router = match auth {
            Some(a) => create_router_with_auth(sup.clone(), a),
            None => create_router(sup.clone(), OperatorAuth::disabled()),
        };
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        tokio::spawn(async move {
            let _ = axum::serve(listener, router).await;
        });
        Self {
            owner,
            addr,
            sup,
            _tmp: tmp,
            exec,
            stop_ticker,
        }
    }

    async fn wait_frames(&self, min_frames: usize) {
        let dl = Instant::now() + Duration::from_secs(5);
        while self.sup.get_recording_state().await.1 < min_frames {
            assert!(
                Instant::now() < dl,
                "{:?}: timed out waiting for {min_frames} frames (got {:?})",
                self.owner,
                self.sup.get_recording_state().await
            );
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    }

    async fn wait_idle(&self, limit: Duration) {
        let dl = Instant::now() + limit;
        loop {
            assert!(
                Instant::now() < dl,
                "{:?}: recorder did not become idle (state={:?})",
                self.owner,
                self.sup.get_recording_state().await
            );
            let status = self.sup.recording_status().await;
            let (state, _) = self.sup.get_recording_state().await;
            if status.is_null() && !matches!(state, EpisodeState::Recording | EpisodeState::Finalizing) {
                break;
            }
            tokio::time::sleep(Duration::from_millis(15)).await;
        }
    }

    fn shutdown(mut self) {
        self.stop_ticker.store(true, Ordering::Release);
        if let Some(h) = self.exec.take() {
            h.stop_and_join();
        }
    }
}

impl Drop for DualRig {
    fn drop(&mut self) {
        self.stop_ticker.store(true, Ordering::Release);
        if let Some(h) = self.exec.take() {
            h.stop_and_join();
        }
    }
}

async fn wait_action_done(sup: &ArmSupervisor, id: u64, timeout_s: f64) -> ActionStatus {
    let dl = Instant::now() + Duration::from_secs_f64(timeout_s);
    loop {
        assert!(Instant::now() < dl, "action {id} did not finish");
        let obs = sup.get_observation().await;
        if let Some(a) = obs.action.filter(|a| a.id == id && !a.is_running()) {
            return a;
        }
        if let Some(a) = obs.previous_action.filter(|a| a.id == id && !a.is_running()) {
            return a;
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

// ============================================================================
// 1. Dual-owner parity: REST & WS recording, datasets, episodes, import, delete, auth
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_rest_and_ws_recording_episodes_datasets_import_delete() {
    let mut summaries = Vec::new();
    for owner in Owner::BOTH {
        let rig = DualRig::new(owner, Duration::from_millis(8), None).await;
        let http = reqwest::Client::new();
        let (mut ws_a, _) = connect_async(format!("ws://{}/api/v1/ws", rig.addr))
            .await
            .unwrap();
        let (mut ws_b, _) = connect_async(format!("ws://{}/api/v1/ws", rig.addr))
            .await
            .unwrap();

        // Dataset create + duplicate + invalid
        let ds_ok = free_cmd(&mut ws_a, "/api/v1/datasets/create", json!({"name": "demo_ds", "description": "dual"})).await;
        assert_eq!(ds_ok["type"], "command_ack", "{owner:?}: {ds_ok}");
        let ds_dup = free_cmd(&mut ws_a, "/api/v1/datasets/create", json!({"name": "demo_ds"})).await;
        assert_eq!(ds_dup["code"], "DATASET_EXISTS", "{owner:?}: {ds_dup}");
        let ds_bad = free_cmd(&mut ws_a, "/api/v1/datasets/create", json!({"name": "bad/name"})).await;
        assert_eq!(ds_bad["code"], "INVALID_COMMAND", "{owner:?}: {ds_bad}");

        // REST recording: start -> wait frames -> save -> duplicate id rejected -> discard path
        let rest_start: Value = http
            .post(format!("http://{}/api/v1/dataset/record/start", rig.addr))
            .json(&json!({"episode_id": "rest_ep_1"}))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!((rest_start["status"].as_str(), rest_start["episode_id"].as_str()), (Some("recording_started"), Some("rest_ep_1")), "{owner:?}: {rest_start}");
        rig.wait_frames(8).await;
        let rest_save: Value = http
            .post(format!("http://{}/api/v1/dataset/record/save", rig.addr))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!((rest_save["status"].as_str(), rest_save["episode_id"].as_str()), (Some("saved"), Some("rest_ep_1")), "{owner:?}: {rest_save}");
        assert!(rest_save["frame_count"].as_u64().unwrap() >= 8);

        // Duplicate episode_id fails with 409 EPISODE_EXISTS
        let dup_res = http
            .post(format!("http://{}/api/v1/dataset/record/start", rig.addr))
            .json(&json!({"episode_id": "rest_ep_1"}))
            .send()
            .await
            .unwrap();
        assert_eq!(dup_res.status().as_u16(), 409);
        assert_eq!(dup_res.json::<Value>().await.unwrap()["code"], "EPISODE_EXISTS");

        // REST discard
        let _ = http
            .post(format!("http://{}/api/v1/dataset/record/start", rig.addr))
            .json(&json!({}))
            .send()
            .await
            .unwrap();
        rig.wait_frames(3).await;
        let disc: Value = http
            .post(format!("http://{}/api/v1/dataset/record/discard", rig.addr))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        assert_eq!(disc["status"], "discarded", "{owner:?}: {disc}");

        // WS record/start -> RECORDER_BUSY & RECORDER_NOT_OWNER & dataset-delete RECORDER_BUSY -> stop(save)
        let missing_ds = client_cmd(&mut ws_a, "rec_a", "/api/v1/record/start", json!({"dataset": "no_such_ds"})).await;
        assert_eq!(missing_ds["code"], "DATASET_NOT_FOUND", "{owner:?}: {missing_ds}");

        let ws_start = client_cmd(
            &mut ws_a,
            "rec_a",
            "/api/v1/record/start",
            json!({"dataset": "demo_ds", "task": "pick cube", "source": "hand_guide"}),
        )
        .await;
        assert_eq!((ws_start["type"].as_str(), ws_start["owner"].as_str()), (Some("command_ack"), Some("rec_a")), "{owner:?}: {ws_start}");
        let rid = ws_start["recording_id"].as_str().unwrap().to_string();
        assert!(armd::recording::is_ulid(&rid));

        // GET /api/v1/episodes shows active recording
        let (st_ep, ep_list) = get_json(rig.addr, "/api/v1/episodes").await;
        assert_eq!(st_ep, 200);
        assert_eq!(ep_list["recording"]["recording_id"], rid);
        assert_eq!(ep_list["recording"]["owner"], "rec_a");
        assert_eq!(ep_list["recording"]["dataset"], "demo_ds");

        // Concurrent start -> 409 RECORDER_BUSY with owner + recording_id
        let busy = client_cmd(&mut ws_b, "rec_b", "/api/v1/record/start", json!({})).await;
        assert_eq!(
            (busy["status"].as_u64(), busy["code"].as_str(), busy["owner"].as_str(), busy["recording_id"].as_str()),
            (Some(409), Some("RECORDER_BUSY"), Some("rec_a"), Some(rid.as_str())),
            "{owner:?}: {busy}"
        );

        // Other WS and REST cannot stop
        let not_owner_ws = client_cmd(&mut ws_b, "rec_b", "/api/v1/record/stop", json!({"save": true})).await;
        assert_eq!(
            (not_owner_ws["status"].as_u64(), not_owner_ws["code"].as_str(), not_owner_ws["owner"].as_str()),
            (Some(409), Some("RECORDER_NOT_OWNER"), Some("rec_a")),
            "{owner:?}: {not_owner_ws}"
        );
        let not_owner_rest = http
            .post(format!("http://{}/api/v1/dataset/record/save", rig.addr))
            .send()
            .await
            .unwrap();
        assert_eq!(not_owner_rest.status().as_u16(), 409);
        assert_eq!(not_owner_rest.json::<Value>().await.unwrap()["code"], "RECORDER_NOT_OWNER");

        // Deleting target dataset while recording -> 409 RECORDER_BUSY (no owner field)
        let ds_busy = free_cmd(&mut ws_b, "/api/v1/datasets/delete", json!({"name": "demo_ds", "force": true})).await;
        assert_eq!((ds_busy["status"].as_u64(), ds_busy["code"].as_str()), (Some(409), Some("RECORDER_BUSY")), "{owner:?}: {ds_busy}");
        assert!(ds_busy.get("owner").is_none(), "dataset delete busy must omit owner: {ds_busy}");

        rig.wait_frames(10).await;
        let ws_stop = client_cmd(&mut ws_a, "rec_a", "/api/v1/record/stop", json!({"save": true, "outcome": "ok"})).await;
        assert_eq!(
            (ws_stop["type"].as_str(), ws_stop["episode"]["id"].as_str(), ws_stop["episode"]["dataset"].as_str(), ws_stop["episode"]["task"].as_str(), ws_stop["episode"]["source"].as_str()),
            (Some("command_ack"), Some(rid.as_str()), Some("demo_ds"), Some("pick cube"), Some("hand_guide")),
            "{owner:?}: {ws_stop}"
        );
        assert!(ws_stop["episode"]["frames"].as_u64().unwrap() >= 10);

        // REST GET /api/v1/episodes/{id}, /data (Parquet + KV), /dataset/episodes, /datasets
        let (st_one, one) = get_json(rig.addr, &format!("/api/v1/episodes/{rid}")).await;
        assert_eq!((st_one, one["stale"].as_bool()), (200, Some(false)));
        let (st_miss, miss) = get_json(rig.addr, "/api/v1/episodes/01ARZ3NDEKTSV4RRFFQ69G5FAV").await;
        assert_eq!((st_miss, miss["code"].as_str()), (404, Some("EPISODE_NOT_FOUND")));

        let data_resp = reqwest::get(format!("http://{}/api/v1/episodes/{rid}/data", rig.addr)).await.unwrap();
        assert_eq!(data_resp.status().as_u16(), 200);
        assert_eq!(data_resp.headers()["content-type"], "application/vnd.apache.parquet");
        let parquet_bytes = data_resp.bytes().await.unwrap();
        let kv = armd::recording::read_parquet_kv_from_bytes(parquet_bytes.clone()).unwrap();
        assert!(kv.contains_key("rebot.episode"), "{owner:?}: missing rebot.episode footer KV");
        let frames = armd::recording::read_frames_from_bytes(parquet_bytes.clone()).unwrap();
        assert_eq!(frames.len() as u64, ws_stop["episode"]["frames"].as_u64().unwrap());

        let (st_leg, leg_list) = get_json(rig.addr, "/api/v1/dataset/episodes").await;
        assert_eq!(st_leg, 200);
        assert_eq!(leg_list["episodes"].as_array().unwrap().len(), 2);

        let (st_ds, ds_list) = get_json(rig.addr, "/api/v1/datasets").await;
        assert_eq!(st_ds, 200);
        assert!(ds_list["datasets"].as_array().unwrap().iter().any(|d| d["name"] == "demo_ds" && d["episodes"] == 1));

        // REST POST /api/v1/episodes/import using exported parquet + sidecar (round-trip parity!)
        let import_sc = json!({
            "units": "rad",
            "joint_names": one["joint_names"],
            "dataset": "imported_ds",
            "task": "imported_task",
            "joint_zero": one["joint_zero"],
            "recorded_at": one["recorded_at"],
            "profile_id": one["profile_id"],
            "tool_id": one["tool_id"],
            "tool_config_digest": one["tool_config_digest"],
            "has_gripper_motion": one["has_gripper_motion"],
        });
        let (ct, body) = multipart(&parquet_bytes, &import_sc);
        let imp_resp = http
            .post(format!("http://{}/api/v1/episodes/import", rig.addr))
            .header("content-type", ct)
            .body(body)
            .send()
            .await
            .unwrap();
        assert_eq!(imp_resp.status().as_u16(), 200);
        let imp_val: Value = imp_resp.json().await.unwrap();
        let imp_id = imp_val["id"].as_str().unwrap().to_string();
        assert_eq!(imp_val["stale"], false);
        assert_eq!(imp_val["episode"]["source"], "import");

        // WS episodes/update + episodes/delete + datasets/delete + REST DELETE /api/v1/dataset/episodes/{id}
        let upd = free_cmd(&mut ws_a, "/api/v1/episodes/update", json!({"id": imp_id, "task": "renamed"})).await;
        assert_eq!((upd["type"].as_str(), upd["episode"]["task"].as_str()), (Some("command_ack"), Some("renamed")));
        let del_rest = http
            .delete(format!("http://{}/api/v1/dataset/episodes/rest_ep_1", rig.addr))
            .send()
            .await
            .unwrap();
        assert_eq!(del_rest.status().as_u16(), 200);
        assert_eq!(del_rest.json::<Value>().await.unwrap()["status"], "deleted");

        let ds_nonempty = free_cmd(&mut ws_a, "/api/v1/datasets/delete", json!({"name": "demo_ds"})).await;
        assert_eq!(ds_nonempty["code"], "DATASET_NOT_EMPTY");
        let ds_force = free_cmd(&mut ws_a, "/api/v1/datasets/delete", json!({"name": "demo_ds", "force": true})).await;
        assert_eq!(ds_force["deleted_episodes"], json!([rid]));
        let ep_del = free_cmd(&mut ws_a, "/api/v1/episodes/delete", json!({"id": imp_id})).await;
        assert_eq!((ep_del["type"].as_str(), ep_del["deleted"].as_str()), (Some("command_ack"), Some(imp_id.as_str())));

        if owner == Owner::Executor {
            assert_eq!(rig.sup.inner().control_thread_attempts(), 0, "0 SupervisorLock on armd-control");
            assert_eq!(rig.exec.as_ref().unwrap().stats().total_allocs(), AllocCounts::ZERO, "0 RT allocs");
        }

        summaries.push(json!({
            "ds_dup": ds_dup["code"],
            "ds_bad": ds_bad["code"],
            "missing_ds": missing_ds["code"],
            "busy_code": busy["code"],
            "busy_owner": busy["owner"],
            "not_owner_code": not_owner_ws["code"],
            "ds_busy_code": ds_busy["code"],
            "ds_nonempty": ds_nonempty["code"],
            "imp_source": imp_val["episode"]["source"],
            "imp_stale": imp_val["stale"],
        }));
        rig.shutdown();
    }
    assert_eq!(summaries[0], summaries[1], "Legacy and Executor responses must match identically");
}

// ============================================================================
// 2. Dual-owner parity: passive survival across lease/STOP/E-stop, 5s disconnect & reclaim
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_recorder_ownership_disconnect_grace_reclaim_and_passive_survival() {
    for owner in Owner::BOTH {
        let rig = DualRig::new(owner, Duration::from_millis(8), None).await;
        let http = reqwest::Client::new();
        let (mut ctrl_ws, gen) = common::controller(rig.addr).await;
        let (mut rec_ws, _) = connect_async(format!("ws://{}/api/v1/ws", rig.addr)).await.unwrap();

        let start = client_cmd(&mut rec_ws, "passive_a", "/api/v1/record/start", json!({"task": "survive"})).await;
        assert_eq!(start["type"], "command_ack", "{owner:?}: {start}");
        let rid = start["recording_id"].as_str().unwrap().to_string();
        rig.wait_frames(4).await;

        // Motion stop and E-stop must NOT revoke passive recording!
        let ep = rig.sup.get_observation().await.epoch_id;
        let stop_ack = common::rpc(&mut ctrl_ws, gen, ep, "/api/v1/motion/stop", json!({})).await;
        assert_eq!(stop_ack["type"], "command_ack", "{owner:?}: {stop_ack}");
        let estop_resp = http
            .post(format!("http://{}/api/v1/safety/e_stop", rig.addr))
            .send()
            .await
            .unwrap();
        assert_eq!(estop_resp.status().as_u16(), 200);

        // Lease release by another client must NOT revoke passive recording!
        let rel = common::ws_reply(&mut ctrl_ws, json!({"type": "release_lease", "generation": gen}), &["lease_released"]).await;
        assert_eq!(rel["success"], true);

        let frames_after_events = rig.sup.get_recording_state().await.1;
        rig.wait_frames(frames_after_events + 4).await;
        assert_eq!(rig.sup.recording_status().await["recording_id"], rid, "{owner:?}: passive recording must survive stop/estop/lease release");

        // Disconnect and reconnect with same client_id within 5 s -> reclaims and saves cleanly!
        drop(rec_ws);
        tokio::time::sleep(Duration::from_millis(400)).await;
        assert_eq!(rig.sup.recording_status().await["owner_connected"], false, "{owner:?}");
        let (mut rec_ws2, _) = connect_async(format!("ws://{}/api/v1/ws", rig.addr)).await.unwrap();
        let saved = client_cmd(&mut rec_ws2, "passive_a", "/api/v1/record/stop", json!({"save": true})).await;
        assert_eq!((saved["type"].as_str(), saved["episode"]["id"].as_str()), (Some("command_ack"), Some(rid.as_str())), "{owner:?}: {saved}");

        // Disconnect without reconnect -> discarded after 5 s grace period
        let start2 = client_cmd(&mut rec_ws2, "passive_b", "/api/v1/record/start", json!({})).await;
        assert_eq!(start2["type"], "command_ack");
        let rid2 = start2["recording_id"].as_str().unwrap().to_string();
        rig.wait_frames(3).await;
        drop(rec_ws2);
        tokio::time::sleep(Duration::from_millis(3500)).await;
        assert_eq!(rig.sup.recording_status().await["recording_id"], rid2, "{owner:?}: must stay alive before 5 s");
        tokio::time::sleep(Duration::from_millis(2200)).await;
        assert!(rig.sup.recording_status().await.is_null(), "{owner:?}: must discard after 5 s");
        assert_eq!(rig.sup.get_recording_state().await.0, EpisodeState::Discarded, "{owner:?}");
        assert!(rig.sup.episode_store().await.get(&rid2).is_none());

        if owner == Owner::Executor {
            assert_eq!(rig.sup.inner().control_thread_attempts(), 0);
            assert_eq!(rig.exec.as_ref().unwrap().stats().total_allocs(), AllocCounts::ZERO);
        }
        rig.shutdown();
    }
}

// ============================================================================
// 3. Dual-owner parity: disk guard + length cap (truncated save)
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_disk_guard_and_length_cap() {
    for owner in Owner::BOTH {
        let rig = DualRig::new(owner, Duration::from_millis(8), None).await;
        let store = rig.sup.episode_store().await;

        // Start refused below START_MIN_FREE_BYTES (507 RECORD_NO_SPACE)
        store.set_free_bytes_override(Some(150 * 1024 * 1024));
        let err = rig.sup.start_recording(None, None).await.unwrap_err();
        let api = armd::api::rest::ApiError::from(err);
        assert_eq!((api.status.as_u16(), api.code.as_deref()), (507, Some("RECORD_NO_SPACE")), "{owner:?}");

        // Low-space monitor save
        store.set_free_bytes_override(Some(300 * 1024 * 1024));
        rig.sup.start_recording(None, None).await.unwrap();
        rig.wait_frames(6).await;
        store.set_free_bytes_override(Some(90 * 1024 * 1024));
        rig.wait_idle(Duration::from_secs(4)).await;
        let eps = store.list_episodes(None, Some(1), None, &[]);
        assert_eq!(eps.len(), 1, "{owner:?}");
        let sc = &eps[0].sidecar;
        assert_eq!((sc.stopped_by.as_deref(), sc.truncated), (Some("low_space"), false), "{owner:?}: {sc:?}");
        assert!(sc.frames >= 6);

        // Length cap (0.1 s at 50 Hz -> ceil(5.0) + 1 = 6 frames, truncated=true, stopped_by=max_duration)
        store.set_free_bytes_override(Some(500 * 1024 * 1024));
        rig.sup.set_max_episode_s(0.1).await;
        rig.sup.start_recording(None, None).await.unwrap();
        rig.wait_idle(Duration::from_secs(4)).await;
        let eps2 = store.list_episodes(None, Some(1), None, &[]);
        let sc2 = &eps2[0].sidecar;
        assert_eq!(
            (sc2.truncated, sc2.stopped_by.as_deref(), sc2.outcome.as_str(), sc2.frames),
            (true, Some("max_duration"), "ok", 6),
            "{owner:?}: {sc2:?}"
        );

        // Immediate restart works cleanly after cap stop
        rig.sup.set_max_episode_s(600.0).await;
        rig.sup.start_recording(None, None).await.unwrap();
        rig.sup.discard_recording().await.unwrap();

        if owner == Owner::Executor {
            assert_eq!(rig.sup.inner().control_thread_attempts(), 0);
            assert_eq!(rig.exec.as_ref().unwrap().stats().total_allocs(), AllocCounts::ZERO);
        }
        rig.shutdown();
    }
}

// ============================================================================
// 4. Dual-owner parity: run_sequence with record={...} (multi-repeat + STOP discard)
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_run_sequence_with_record_repeats_and_stop_discard() {
    for owner in Owner::BOTH {
        let rig = DualRig::new(owner, Duration::from_millis(4), None).await;
        let (mut ws, gen) = common::controller(rig.addr).await;
        let ep = rig.sup.get_observation().await.epoch_id;
        assert_eq!(
            common::rpc(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await["type"],
            "command_ack"
        );

        // Save a short 2-step sequence (wait + wait) so it runs fast & deterministically on both owners
        let seq_save = common::rpc(
            &mut ws,
            gen,
            ep,
            "/api/v1/sequences/save",
            json!({
                "name": "dual_rec_seq",
                "steps": [
                    {"type": "wait", "s": 0.08},
                    {"type": "wait", "s": 0.08}
                ],
                "expected_revision": 0
            }),
        )
        .await;
        assert_eq!(seq_save["type"], "command_ack", "{owner:?}: {seq_save}");
        let sid = seq_save["sequence"]["id"].as_str().unwrap().to_string();

        // Missing dataset refused upfront
        let bad_ds = common::rpc(
            &mut ws,
            gen,
            ep,
            "/api/v1/action/run_sequence",
            json!({"id": sid, "record": {"dataset": "nope"}}),
        )
        .await;
        assert_eq!(bad_ds["code"], "DATASET_NOT_FOUND", "{owner:?}: {bad_ds}");

        // Busy recorder refused upfront with owner
        rig.sup.start_recording(None, None).await.unwrap();
        let busy_seq = common::rpc(
            &mut ws,
            gen,
            ep,
            "/api/v1/action/run_sequence",
            json!({"id": sid, "record": {"dataset": "default"}}),
        )
        .await;
        assert_eq!(
            (busy_seq["code"].as_str(), busy_seq["owner"].as_str()),
            (Some("RECORDER_BUSY"), Some("legacy-rest")),
            "{owner:?}: {busy_seq}"
        );
        rig.sup.discard_recording().await.unwrap();

        // Run 2 repeats with record -> saves exactly 2 episodes with repeat_index 0 and 1
        let ack = common::rpc(
            &mut ws,
            gen,
            ep,
            "/api/v1/action/run_sequence",
            json!({"id": sid, "repeat": 2, "record": {"dataset": "default", "task": "pick"}}),
        )
        .await;
        assert_eq!(ack["type"], "command_ack", "{owner:?}: {ack}");
        let aid = ack["action_id"].as_u64().unwrap();
        let done = wait_action_done(&rig.sup, aid, 15.0).await;
        assert_eq!(done.state.as_str(), "done", "{owner:?}: {done:?}");
        assert_eq!(done.episodes.len(), 2, "{owner:?}: {done:?}");
        let st = rig.sup.episode_store().await;
        for (r, eid) in done.episodes.iter().enumerate() {
            let sc = st.get(eid).expect("saved episode in catalog");
            assert_eq!(
                (sc.source.as_str(), sc.sequence_id.as_deref(), sc.repeat_index, sc.task.as_str(), sc.outcome.as_str()),
                ("sequence", Some(sid.as_str()), Some(r as u32), "pick", "ok"),
                "{owner:?}: {sc:?}"
            );
            assert!(sc.frames >= 5, "{owner:?}: {sc:?}");
        }
        assert!(rig.sup.recording_status().await.is_null());

        // Stop during repeat 2 of 3 -> repeat 1 kept, repeat 2 discarded!
        let seq2_save = common::rpc(
            &mut ws,
            gen,
            ep,
            "/api/v1/sequences/save",
            json!({
                "name": "dual_stop_seq",
                "steps": [
                    {"type": "wait", "s": 0.06},
                    {"type": "wait", "s": 0.50}
                ],
                "expected_revision": seq_save["revision"]
            }),
        )
        .await;
        assert_eq!(seq2_save["type"], "command_ack", "{owner:?}: {seq2_save}");
        let sid2 = seq2_save["sequence"]["id"].as_str().unwrap().to_string();

        let ack2 = common::rpc(
            &mut ws,
            gen,
            ep,
            "/api/v1/action/run_sequence",
            json!({"id": sid2, "repeat": 3, "record": {"dataset": "default"}}),
        )
        .await;
        assert_eq!(ack2["type"], "command_ack", "{owner:?}: {ack2}");
        let aid2 = ack2["action_id"].as_u64().unwrap();

        let dl = Instant::now() + Duration::from_secs(15);
        loop {
            assert!(Instant::now() < dl, "{owner:?}: never reached repeat 1 step 1");
            let obs = rig.sup.get_observation().await;
            if let Some(a) = obs.action.as_ref().filter(|a| a.id == aid2) {
                if a.repeat_index == Some(1) && a.step_index == Some(1) {
                    // External caller cannot stop the sequence's recording
                    let foreign_err = rig
                        .sup
                        .stop_recording_as(
                            armd::teach::Stopper::Owner(armd::teach::RecorderOwner::LegacyRest),
                            true,
                            None,
                            None,
                        )
                        .await
                        .unwrap_err();
                    assert_eq!(
                        armd::api::rest::ApiError::from(foreign_err).code.as_deref(),
                        Some("RECORDER_NOT_OWNER")
                    );
                    let stop_r = common::rpc(&mut ws, gen, ep, "/api/v1/motion/stop", json!({})).await;
                    assert_eq!(stop_r["type"], "command_ack", "{owner:?}: {stop_r}");
                    break;
                }
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        let done2 = wait_action_done(&rig.sup, aid2, 10.0).await;
        assert_eq!(
            (done2.state.as_str(), done2.code.as_deref(), done2.episodes.len()),
            ("cancelled", Some("STOPPED"), 1),
            "{owner:?}: {done2:?}"
        );
        assert!(done2.failed_step.is_some(), "{owner:?}: {done2:?}");
        rig.wait_idle(Duration::from_secs(3)).await;
        let of_run: Vec<_> = st
            .list_episodes(None, None, None, &[])
            .into_iter()
            .filter(|e| e.sidecar.sequence_id.as_deref() == Some(sid2.as_str()))
            .collect();
        assert_eq!(of_run.len(), 1, "{owner:?}: only the completed repeat may be published");
        assert_eq!(of_run[0].sidecar.repeat_index, Some(0));
        assert_eq!(rig.sup.get_recording_state().await.0, EpisodeState::Discarded);

        // Executor-only deterministic race proof (Blocker 2): STOP arrives immediately after RT
        // commits repeat 0 while the off-thread parquet writer is held open. `SequenceCommitGuard`
        // must hold sequence terminal delivery until publication completes so the terminal still
        // carries the RT-committed repeat episode ID.
        if owner == Owner::Executor {
            let (entered_rx, release_tx) = st.arm_test_publish_pause();
            let ack3 = common::rpc(
                &mut ws,
                gen,
                ep,
                "/api/v1/action/run_sequence",
                json!({"id": sid2, "repeat": 2, "record": {"dataset": "default"}}),
            )
            .await;
            assert_eq!(ack3["type"], "command_ack", "executor delayed-writer run: {ack3}");
            let aid3 = ack3["action_id"].as_u64().unwrap();

            // Wait until repeat 0 has passed RT stop fence and entered Phase 2 parquet publish.
            let paused_ep_id = tokio::task::spawn_blocking(move || {
                entered_rx
                    .recv_timeout(Duration::from_secs(15))
                    .expect("repeat 0 never entered publish pause")
            })
            .await
            .unwrap();

            // Issue STOP while repeat 0's parquet write is still held open!
            let stop_r3 = common::rpc(&mut ws, gen, ep, "/api/v1/motion/stop", json!({})).await;
            assert_eq!(stop_r3["type"], "command_ack", "stop during delayed writer: {stop_r3}");

            // Hold writer briefly and assert observation does NOT leak a premature terminal missing `paused_ep_id`.
            tokio::time::sleep(Duration::from_millis(60)).await;
            let obs_held = rig.sup.get_observation().await;
            if let Some(a) = obs_held.action.as_ref().filter(|a| a.id == aid3) {
                assert!(
                    a.is_running() || a.episodes.contains(&paused_ep_id),
                    "sequence terminal leaked before RT-committed episode published: {a:?}"
                );
            }
            let _ = release_tx.send(());

            let done3 = wait_action_done(&rig.sup, aid3, 10.0).await;
            assert_eq!(
                (done3.state.as_str(), done3.code.as_deref()),
                ("cancelled", Some("STOPPED")),
                "delayed writer STOP terminal: {done3:?}"
            );
            assert_eq!(
                done3.episodes,
                vec![paused_ep_id.clone()],
                "RT-committed repeat episode must never be omitted from terminal: {done3:?}"
            );
            assert!(
                st.get(&paused_ep_id).is_some(),
                "committed repeat episode must exist in store"
            );
            rig.wait_idle(Duration::from_secs(3)).await;

            assert_eq!(rig.sup.inner().control_thread_attempts(), 0);
            assert_eq!(rig.exec.as_ref().unwrap().stats().total_allocs(), AllocCounts::ZERO);
        }
        rig.shutdown();
    }
}

// ============================================================================
// 5. Executor: idempotent publish, delayed writer, dataset pin, config interleaving, auth
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_idempotent_publish_delayed_writer_config_interleaving_and_auth() {
    let rig = DualRig::new(
        Owner::Executor,
        Duration::from_millis(8),
        Some(OperatorAuth::enforced("123456", 1800)),
    )
    .await;
    let store = rig.sup.episode_store().await;

    // (a) Operator auth matrix on executor WS/REST: unauthenticated blocked, discard allowed
    let (mut ws, _) = connect_async(format!("ws://{}/api/v1/ws", rig.addr)).await.unwrap();
    let r_unauth = free_cmd(&mut ws, "/api/v1/datasets/create", json!({"name": "auth_ds"})).await;
    assert_eq!((r_unauth["status"].as_u64(), r_unauth["code"].as_str()), (Some(403), Some("OPERATOR_REQUIRED")));
    let r_disc_unauth = free_cmd(&mut ws, "/api/v1/record/stop", json!({"save": false})).await;
    assert_ne!(r_disc_unauth["code"].as_str(), Some("OPERATOR_REQUIRED"), "record/stop save=false must bypass operator requirement");

    let http = reqwest::Client::new();
    let rest_unauth = http
        .post(format!("http://{}/api/v1/dataset/record/start", rig.addr))
        .json(&json!({}))
        .send()
        .await
        .unwrap();
    assert_eq!(rest_unauth.status().as_u16(), 403);

    // (b) Idempotent publish_complete_episode by (id, commit_token) + real post-parquet-rename
    //     recovery + custom-writer missing-file refusal + capacity slot reservation + strict Exists
    let frames = vec![
        RecordFrame { timestamp: 0.0, observation_state: vec![0.0; 7], action: vec![0.0; 7] },
        RecordFrame { timestamp: 0.02, observation_state: vec![0.1; 7], action: vec![0.1; 7] },
    ];
    let sc = EpisodeSidecar {
        id: "idem_ep_1".into(),
        dataset: "default".into(),
        task: "idem".into(),
        source: "sdk".into(),
        sequence_id: None,
        repeat_index: None,
        outcome: "ok".into(),
        recorded_at: "2026-11-26T00:00:00.000Z".into(),
        duration_s: 0.02,
        frames: 2,
        joint_names: vec!["j1".into(), "j2".into(), "j3".into(), "j4".into(), "j5".into(), "j6".into(), "j7".into()],
        units: "rad".into(),
        joint_zero: JointZeroStamp::of(&[0.0; 7]),
        profile_id: Some("rebot-dm".into()),
        armd_version: env!("CARGO_PKG_VERSION").into(),
        fps_nominal: 50.0,
        truncated: false,
        stopped_by: None,
        arm_dof: Some(6),
        gripper_joint: Some(6),
        recovered: None,
        first_state: Some(vec![0.0; 7]),
        last_state: Some(vec![0.1; 7]),
        tool_id: Some("stock".into()),
        tool_config_digest: Some("d".into()),
        has_gripper_motion: Some(false),
        frames_dropped: None,
    };
    assert!(store.publish_complete_episode(&sc, &[], None, Some(42), true).is_err(), "empty episode must be rejected");
    let p1 = store.publish_complete_episode(&sc, &frames, None, Some(42), true).unwrap();
    let p2 = store.publish_complete_episode(&sc, &frames, None, Some(42), true).unwrap();
    assert_eq!(p1, p2, "same (id, commit_token) retry must be idempotent");
    let err_other = store.publish_complete_episode(&sc, &frames, None, Some(99), true).unwrap_err();
    assert_eq!(err_other.status_code(), (409, "EPISODE_EXISTS"), "separate operation reusing id must fail");
    let err_none = store.publish_complete_episode(&sc, &frames, None, None, true).unwrap_err();
    assert_eq!(err_none.status_code(), (409, "EPISODE_EXISTS"));

    // Custom writer returning Ok(()) without creating parquet must fail closed (Minor 1).
    let mut sc_noop = sc.clone();
    sc_noop.id = "idem_ep_noop_writer".into();
    let noop_err = store
        .publish_complete_episode(
            &sc_noop,
            &frames,
            Some(Box::new(|_p, _f| Ok(()))),
            Some(55),
            true,
        )
        .unwrap_err();
    assert_eq!(noop_err.status_code(), (500, "EPISODES_IO_ERROR"));
    assert!(store.get(&sc_noop.id).is_none());

    // Real post-parquet-rename recovery: writer creates parquet file on disk then returns a fault
    // before sidecar/catalog commit; different token fails EPISODE_EXISTS; same token recovers!
    let mut sc_rec = sc.clone();
    sc_rec.id = "idem_ep_post_rename".into();
    let first_fail = store.publish_complete_episode(
        &sc_rec,
        &frames,
        Some(Box::new(|p, f| {
            armd::recording::write_episode_parquet(p, f, None, &[])?;
            Err(armd::recording::ParquetRecordingError::Io(std::io::Error::other(
                "simulated fault after parquet rename",
            )))
        })),
        Some(77),
        true,
    );
    assert!(first_fail.is_err(), "first attempt must fail after parquet rename");
    assert!(store.path_of(&sc_rec.id).unwrap().is_file(), "parquet file must exist on disk");
    assert!(store.get(&sc_rec.id).is_none(), "catalog must not expose incomplete episode before sidecar");
    let wrong_tok = store
        .publish_complete_episode(&sc_rec, &frames, None, Some(88), true)
        .unwrap_err();
    assert_eq!(wrong_tok.status_code(), (409, "EPISODE_EXISTS"));
    // Same token with altered sidecar metadata must fail closed (Major 5).
    let mut sc_rec_altered = sc_rec.clone();
    sc_rec_altered.task = "tampered_task".into();
    let mismatch_meta = store
        .publish_complete_episode(&sc_rec_altered, &frames, None, Some(77), true)
        .unwrap_err();
    assert_eq!(mismatch_meta.status_code(), (409, "EPISODE_EXISTS"));
    // In-progress publication claim blocks raw deletion and replay adoption (Blocker 2).
    assert_eq!(
        store.delete_files(&sc_rec.id).unwrap_err().status_code(),
        (404, "EPISODE_NOT_FOUND")
    );
    assert_eq!(
        store
            .get_or_adopt(&sc_rec.id, &[0.0; 7], &sc_rec.joint_names, sc_rec.profile_id.as_deref(), 50.0)
            .unwrap_err()
            .status_code(),
        (404, "EPISODE_NOT_FOUND")
    );
    let recovered_path = store
        .publish_complete_episode(&sc_rec, &frames, None, Some(77), true)
        .expect("retry with same (commit_token, sidecar) must complete post-rename recovery");
    assert!(recovered_path.is_file());
    assert!(store.get(&sc_rec.id).is_some(), "recovered episode must be cataloged");

    // Capacity slot reservation consumes atomically into in_progress_tokens without double-counting (Major 4/6).
    let (_, cur_files) = store.usage().unwrap();
    let max_two_more = cur_files + 2;
    let mut slot1 = store
        .reserve_capacity_slot_with(u64::MAX, max_two_more)
        .expect("slot 1 at cap-2 must reserve");
    let (pub_entered_rx, pub_release_tx) = store.arm_test_publish_pause();
    let store_p = store.clone();
    let mut sc_cap = sc.clone();
    sc_cap.id = "idem_ep_cap_slot".into();
    let frames_p = frames.clone();
    let pub_handle = tokio::task::spawn_blocking(move || {
        store_p.publish_complete_episode_with_slot(
            &sc_cap,
            &frames_p,
            None,
            Some(91),
            false,
            Some(&mut slot1),
        )
    });
    let _ = tokio::task::spawn_blocking(move || pub_entered_rx.recv_timeout(Duration::from_secs(5)).unwrap())
        .await
        .unwrap();
    // While episode 1 is mid-publication, exactly 1 slot remains: slot 2 MUST succeed, slot 3 MUST fail 507!
    let slot2 = store
        .reserve_capacity_slot_with(u64::MAX, max_two_more)
        .expect("slot 2 must succeed (slot 1 must not double-count)");
    let slot3_err = store
        .reserve_capacity_slot_with(u64::MAX, max_two_more)
        .unwrap_err();
    assert_eq!(slot3_err.status_code(), (507, "EPISODES_FULL"));
    let _ = pub_release_tx.send(());
    pub_handle.await.unwrap().unwrap();
    drop(slot2);

    // Force-delete membership revalidation, single-owner guard, and episode move guard (Blocker 3 + Major 3).
    store.create_dataset("del_guard_ds", "").unwrap();
    store.create_dataset("del_other_ds", "").unwrap();
    let mut sc_dg = sc.clone();
    sc_dg.id = "dg_ep_1".into();
    sc_dg.dataset = "del_guard_ds".into();
    store.publish_complete_episode(&sc_dg, &frames, None, Some(102), true).unwrap();
    let seq_rev = std::sync::atomic::AtomicU64::new(0);
    // Stale membership list (missing `dg_ep_1`) must trigger SequenceRevisionChanged retry instead of deleting!
    assert!(matches!(
        store.take_dataset_if_unpinned_and_rev_matches("del_guard_ds", &[], &seq_rev, 0),
        armd::recording::store::TakeDatasetOutcome::SequenceRevisionChanged
    ));
    let (taken_dg, dg_guard) = match store.take_dataset_if_unpinned_and_rev_matches(
        "del_guard_ds",
        &["dg_ep_1".to_string()],
        &seq_rev,
        0,
    ) {
        armd::recording::store::TakeDatasetOutcome::Removed(ids, g) => (ids, g),
        other => panic!("expected Removed, got {other:?}"),
    };
    // Concurrent second force-delete, episode move into deleting dataset, replay adoption,
    // and ID reuse via new_episode MUST all be blocked while deletion claim is active (B2/B3/M3).
    assert!(matches!(
        store.take_dataset_if_unpinned_and_rev_matches("del_guard_ds", &[], &seq_rev, 0),
        armd::recording::store::TakeDatasetOutcome::DatasetRecordingBusy
    ));
    assert_eq!(
        store.update(&sc_rec.id, None, Some("del_guard_ds")).unwrap_err().status_code(),
        (409, "RECORDER_BUSY")
    );
    assert_eq!(
        store
            .get_or_adopt("dg_ep_1", &[0.0; 7], &sc_dg.joint_names, sc_dg.profile_id.as_deref(), 50.0)
            .unwrap_err()
            .status_code(),
        (404, "EPISODE_NOT_FOUND")
    );
    assert_eq!(
        store.new_episode(Some("dg_ep_1"), None).unwrap_err().status_code(),
        (409, "EPISODE_EXISTS")
    );
    // Unclaimed raw delete_files MUST NOT steal or unlink a claimed episode, and a wrong claim
    // token MUST NOT unlink it either (B2/B3).
    assert_eq!(
        store.delete_files("dg_ep_1").unwrap_err().status_code(),
        (404, "EPISODE_NOT_FOUND")
    );
    assert!(store.path_of("dg_ep_1").unwrap().is_file());
    store.delete_files_with_claim("dg_ep_1", u64::MAX).unwrap();
    assert!(store.path_of("dg_ep_1").unwrap().is_file(), "stale claim token must not unlink");
    store.delete_dataset_taken_files("del_guard_ds", &taken_dg, dg_guard).unwrap();
    assert!(!store.has_dataset("del_guard_ds"));

    // Abandoned single-episode DELETE claim guard marks claim retryable so a repeat DELETE rotates
    // the token and finishes cleanup without stranding the episode ID (R6 Major).
    let mut sc_aband = sc.clone();
    sc_aband.id = "aband_del_ep".into();
    store.publish_complete_episode(&sc_aband, &frames, None, Some(103), true).unwrap();
    let abandoned_guard = rig
        .sup
        .take_episode_for_delete_claim("aband_del_ep")
        .await
        .unwrap()
        .expect("initial take must return claim guard");
    let old_tok = abandoned_guard.token();
    drop(abandoned_guard); // simulate async request cancellation before unlink
    let retry_guard = rig
        .sup
        .take_episode_for_delete_claim("aband_del_ep")
        .await
        .unwrap()
        .expect("repeat take after cancelled delete must rotate claim");
    assert_ne!(retry_guard.token(), old_tok);
    store.delete_files_with_claim("aband_del_ep", old_tok).unwrap();
    assert!(store.path_of("aband_del_ep").unwrap().is_file(), "old token must not unlink");
    store.delete_files_with_claim_guard("aband_del_ep", retry_guard).unwrap();
    assert!(!store.path_of("aband_del_ep").unwrap().exists());

    // (c) Delayed parquet writer on executor: RT keeps ticking with 0 allocs/locks, dataset stays
    //     pinned during Finalizing, RECORDER_BUSY blocks concurrent start, AND force-delete does
    //     not remove existing catalog episodes when dataset is pinned (Major 7).
    store.create_dataset("pin_ds", "").unwrap();
    let mut sc_existing = sc.clone();
    sc_existing.id = "pin_ds_existing_ep".into();
    sc_existing.dataset = "pin_ds".into();
    store
        .publish_complete_episode(&sc_existing, &frames, None, Some(101), true)
        .unwrap();
    rig.sup
        .start_recording_as(
            armd::teach::RecorderOwner::LegacyRest,
            armd::teach::RecordOptions {
                dataset: "pin_ds".into(),
                ..Default::default()
            },
            None,
            None,
        )
        .await
        .unwrap();
    rig.wait_frames(8).await;

    let (entered_tx, entered_rx) = std::sync::mpsc::sync_channel::<()>(1);
    let (release_tx, release_rx) = std::sync::mpsc::sync_channel::<()>(1);
    let sup_c = rig.sup.clone();
    let saver = tokio::spawn(async move {
        sup_c
            .save_recording_with(move |path, frames| {
                let _ = entered_tx.send(());
                let _ = release_rx.recv();
                armd::recording::write_episode_parquet(path, frames, None, &[])
            })
            .await
    });
    tokio::task::spawn_blocking(move || entered_rx.recv_timeout(Duration::from_secs(5)).unwrap())
        .await
        .unwrap();
    // While finalizing: new start -> RECORDER_BUSY, dataset force-delete -> RECORDER_BUSY without removing catalog!
    let busy_err = rig.sup.start_recording(None, None).await.unwrap_err();
    assert_eq!(armd::api::rest::ApiError::from(busy_err).code.as_deref(), Some("RECORDER_BUSY"));
    let ds_take_err = rig
        .sup
        .take_dataset_episodes_for_delete("pin_ds")
        .await
        .unwrap_err();
    assert_eq!(armd::api::rest::ApiError::from(ds_take_err).code.as_deref(), Some("RECORDER_BUSY"));
    assert!(
        store.contains("pin_ds_existing_ep"),
        "failed force-delete while pinned must preserve existing catalog entries"
    );
    let ds_pin_err = store.delete_dataset("pin_ds", true).unwrap_err();
    assert_eq!(ds_pin_err.status_code(), (409, "RECORDER_BUSY"));

    let _ = release_tx.send(());
    saver.await.unwrap().unwrap();
    assert_eq!(rig.sup.get_recording_state().await.0, EpisodeState::Committed);

    // (d) Config-change interleaving: commit a PR C pose config write while a recording is active
    //     on executor -> verifies coherent metadata binding, exact DOF sample width, and 0 RT allocs.
    let (mut op_ws, _) = connect_async(format!("ws://{}/api/v1/ws", rig.addr)).await.unwrap();
    let unlock: Value = http
        .post(format!("http://{}/api/v1/auth/unlock", rig.addr))
        .json(&json!({"pin": "123456"}))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let token = unlock["token"].as_str().unwrap();
    let auth_ack = common::ws_reply(&mut op_ws, json!({"type": "auth", "token": token}), &["auth_ack"]).await;
    assert_eq!(auth_ack["role"], "operator");
    let lease_ack = common::ws_reply(&mut op_ws, json!({"type": "acquire_lease", "client_id": "cfg_interleave"}), &["lease_granted"]).await;
    let gen = lease_ack["generation"].as_u64().unwrap();
    let ep = rig.sup.get_observation().await.epoch_id;

    rig.sup.start_recording(None, None).await.unwrap();
    rig.wait_frames(5).await;
    let (_, poses_val) = get_json(rig.addr, "/api/v1/poses").await;
    let poses_rev = poses_val["revision"].as_u64().unwrap();
    let pose_ack = common::rpc(
        &mut op_ws,
        gen,
        ep,
        "/api/v1/poses/save",
        json!({"name": "mid_rec_pose", "positions": common::INSPECT.to_vec(), "expected_revision": poses_rev}),
    )
    .await;
    assert_eq!(pose_ack["type"], "command_ack", "config write during recording must succeed: {pose_ack}");
    rig.wait_frames(10).await;
    let saved_path = rig.sup.save_recording().await.unwrap();
    let saved_id = saved_path.file_stem().unwrap().to_string_lossy().to_string();
    let (st_v, view_val) = get_json(rig.addr, &format!("/api/v1/episodes/{saved_id}")).await;
    assert_eq!(st_v, 200);
    assert_eq!(view_val["joint_zero"]["offsets_rad"][0].as_f64().unwrap(), 0.0);
    assert_eq!(view_val["stale"], false);
    assert_eq!(view_val["joint_names"].as_array().unwrap().len(), 7);
    let saved_frames = armd::read_frames_from_parquet(&saved_path).unwrap();
    assert!(saved_frames.len() >= 10);
    assert!(saved_frames.iter().all(|f| f.observation_state.len() == 7 && f.action.len() == 7));

    // Verify stale detection via read_services on an imported episode with a different joint_zero
    let pq_bytes = std::fs::read(&saved_path).unwrap();
    let stale_sc = json!({
        "units": "rad",
        "joint_names": view_val["joint_names"],
        "dataset": "default",
        "task": "stale_check",
        "joint_zero": {"offsets_rad": [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "rev": "old"},
        "recorded_at": view_val["recorded_at"],
        "profile_id": view_val["profile_id"],
        "tool_id": view_val["tool_id"],
        "tool_config_digest": view_val["tool_config_digest"],
        "has_gripper_motion": view_val["has_gripper_motion"],
    });
    let (ct, body) = multipart(&pq_bytes, &stale_sc);
    let imp_stale: Value = http
        .post(format!("http://{}/api/v1/episodes/import", rig.addr))
        .header("authorization", format!("Bearer {token}"))
        .header("content-type", ct)
        .body(body)
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(imp_stale["stale"], true, "{imp_stale}");
    let stale_id = imp_stale["id"].as_str().unwrap();
    let (st_stale, stale_view) = get_json(rig.addr, &format!("/api/v1/episodes/{stale_id}")).await;
    assert_eq!((st_stale, stale_view["stale"].as_bool()), (200, Some(true)));

    assert_eq!(rig.sup.inner().control_thread_attempts(), 0);
    assert_eq!(rig.exec.as_ref().unwrap().stats().total_allocs(), AllocCounts::ZERO);
    rig.shutdown();
}

// ============================================================================
// 6. RT proof: 0 alloc, 0 SupervisorLock, 0 forbidden locks, empty save, saturation, restart
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_recorder_rt_zero_alloc_zero_lock_saturation_and_teardown_proof() {
    // 0) Empty-episode save refusal on RT: long period ensures 0 samples pushed when save arrives;
    //    RT must refuse EMPTY_EPISODE while keeping the session live so subsequent frames save!
    {
        let slow_rig = DualRig::new(Owner::Executor, Duration::from_millis(80), None).await;
        let slow_exec = slow_rig.exec.as_ref().unwrap().clone();
        let slow_shared = slow_exec.recorder_shared();
        // Start right after a tick publish so the next tick is ~80 ms away.
        slow_rig.sup.start_recording(None, None).await.unwrap();
        if slow_shared.live_pushed() == 0 {
            let empty_err = slow_rig.sup.save_recording().await.unwrap_err();
            assert_eq!(
                armd::api::rest::ApiError::from(empty_err).code.as_deref(),
                Some("EMPTY_EPISODE"),
                "empty save must be refused with EMPTY_EPISODE"
            );
            assert_eq!(
                slow_rig.sup.get_recording_state().await.0,
                EpisodeState::Recording,
                "empty save refusal must leave session live"
            );
        }
        slow_rig.wait_frames(2).await;
        let saved = slow_rig.sup.save_recording().await.unwrap();
        assert!(saved.is_file());
        slow_rig.shutdown();
        assert_eq!(slow_exec.stats().total_allocs(), AllocCounts::ZERO);
    }

    let rig = DualRig::new(Owner::Executor, Duration::from_millis(2), None).await;
    let exec = rig.exec.as_ref().unwrap().clone();
    let assembly = exec.recorder_assembly();
    let shared = exec.recorder_shared();

    // 1) Hold RecorderAssembly mutex to force stream ring overflow (> RECORDER_STREAM_CAPACITY)
    //    while armd-control ticks: proves drop-newest with 0 RT alloc and 0 lock attempts!
    rig.sup.start_recording(None, None).await.unwrap();
    let guard = assembly.lock_for_test();
    let dl = Instant::now() + Duration::from_secs(5);
    while shared.live_dropped.load(Ordering::Acquire) < 20 {
        assert!(
            Instant::now() < dl,
            "expected stream overflow drops while assembly locked (pushed={}, dropped={})",
            shared.live_pushed(),
            shared.live_dropped.load(Ordering::Acquire)
        );
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    drop(guard);

    let saved_path = rig.sup.save_recording().await.unwrap();
    let saved_id = saved_path.file_stem().unwrap().to_string_lossy().to_string();
    let sc = rig.sup.episode_store().await.get(&saved_id).unwrap();
    assert!(sc.frames >= RECORDER_STREAM_CAPACITY, "expected >= {RECORDER_STREAM_CAPACITY} frames, got {}", sc.frames);
    assert!(sc.frames_dropped.unwrap_or(0) >= 20, "expected dropped samples recorded on sidecar: {sc:?}");

    // 2) Immediate restart after saturation saves a clean episode with 0 dropped frames
    rig.sup.start_recording(None, None).await.unwrap();
    rig.wait_frames(15).await;
    let p2 = rig.sup.save_recording().await.unwrap();
    let id2 = p2.file_stem().unwrap().to_string_lossy().to_string();
    let sc2 = rig.sup.episode_store().await.get(&id2).unwrap();
    assert!(sc2.frames >= 15);
    assert_eq!(sc2.frames_dropped, None, "clean session after saturation must report 0 drops: {sc2:?}");

    // 3) Leave an active recording open across ordinary executor teardown -> must revoke
    //    SessionSlot::Recorder with a frozen close fence (Major 1), fail closed on post-exit
    //    store lookup with 0 SupervisorLock fallback (Major 11), and keep 0 RT allocs/locks!
    rig.sup.start_recording(None, None).await.unwrap();
    rig.wait_frames(4).await;
    let active_op_id = shared.live_op_id.load(Ordering::Acquire);
    let active_sid = armd::rt_core::session::SessionId(shared.live_session.load(Ordering::Acquire));
    assert!(active_op_id != 0 && active_sid.0 != 0);

    let sup_after = rig.sup.clone();
    rig.shutdown();

    let closed_fence = shared
        .load_closed_fence_for(active_op_id, active_sid)
        .expect("ordinary executor teardown must freeze a close fence for active recorder");
    assert!(closed_fence.revoked, "teardown close fence must be marked revoked");
    assert_eq!(shared.live_session.load(Ordering::Acquire), 0);

    // Post-exit store lookup must fail closed (503 CONFIG_UNAVAILABLE) without touching SupervisorLock.
    let post_exit_err = sup_after.try_episode_store().await.unwrap_err();
    assert_eq!(
        armd::api::rest::ApiError::from(post_exit_err).code.as_deref(),
        Some("CONFIG_UNAVAILABLE")
    );

    assert_eq!(sup_after.inner().control_thread_attempts(), 0, "0 SupervisorLock attempts on armd-control (including teardown)");
    assert_eq!(assembly.control_thread_lock_attempts(), 0, "0 RecorderAssembly lock attempts on armd-control");
    assert_eq!(sup_after.executor_recorder().control_thread_lock_attempts(), 0, "0 ExecutorRecorder lock attempts on armd-control");
    assert_eq!(exec.stats().total_allocs(), AllocCounts::ZERO, "0 RT heap allocs/frees across start/sample/overflow/save/restart/teardown");
}
