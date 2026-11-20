//! Calibration hover job (T20 acceptance) on the simulated follower with a mock rebot-vision:
//! the bounded sequence, refused points skipped, observations with discarded frames, request
//! bounds, a disconnect during the descent (lease watchdog parks, observations kept, resume by
//! job id) and a stop during the capture.
mod common;

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use armd::calibration::CalibrationDeps;
use armd::grasp::GraspDeps;
use armd::{ArmConfig, ArmSupervisor};
use serde_json::{json, Value};

const LEASE_MS: u64 = 400;
const DOWN: [f64; 3] = [0.0, std::f64::consts::FRAC_PI_2, 0.0];

struct Rig {
    ws: common::TestSocket,
    generation: u64,
    epoch: u64,
    sup: Arc<ArmSupervisor>,
    addr: std::net::SocketAddr,
    triggers: Arc<AtomicU64>,
    _dir: tempfile::TempDir,
}

async fn mock_vision(triggers: Arc<AtomicU64>) -> String {
    let app = axum::Router::new().route("/api/trigger", axum::routing::post(move || {
        let t = triggers.clone();
        async move {
            let n = t.fetch_add(1, Ordering::SeqCst) + 1;
            tokio::time::sleep(Duration::from_millis(30)).await;
            axum::Json(json!({"success": true, "frame_id": 100 + n, "width": 640, "height": 360,
                "markers": [{"id": 5, "corners": [[300.0, 200.0], [340.0, 200.0], [340.0, 160.0], [300.0, 160.0]]},
                            {"id": 40, "corners": [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]}]}))
        }
    }));
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let url = format!("http://{}", listener.local_addr().unwrap());
    tokio::spawn(async move { axum::serve(listener, app).await.unwrap(); });
    url
}

impl Rig {
    async fn new() -> Self {
        let dir = tempfile::tempdir().unwrap();
        let triggers = Arc::new(AtomicU64::new(0));
        let url = mock_vision(triggers.clone()).await;
        let up = armd::api::proxy::DetectorUpstream::parse(&url).unwrap();
        let mut deps = CalibrationDeps::new(dir.path().join("calibration.json"), Some(dir.path().join("grasp_setup.json")), up.clone());
        deps.backup_dir = dir.path().join("backups");
        let sup = Arc::new(ArmSupervisor::new(ArmConfig { lease_timeout_ms: LEASE_MS, ..ArmConfig::rebot_dm() }));
        let router = armd::api::create_router_calibration(sup.clone(), up.clone(), GraspDeps::http(up, None),
            armd::diagnostics::DiagnosticsState::global(), deps, armd::OperatorAuth::disabled());
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });
        let ticker = sup.clone();
        tokio::spawn(async move {
            let mut interval = tokio::time::interval(Duration::from_millis(20));
            loop { interval.tick().await; ticker.tick(0.02).await; }
        });
        let (ws, generation, epoch) = Self::control(addr, &sup).await;
        Rig { ws, generation, epoch, sup, addr, triggers, _dir: dir }
    }

    /// A controller with torque on at the inspection pose.
    async fn control(addr: std::net::SocketAddr, sup: &ArmSupervisor) -> (common::TestSocket, u64, u64) {
        let (mut ws, generation) = common::controller(addr).await;
        let epoch = sup.get_observation().await.epoch_id;
        let torqued = sup.inner().read().await.is_torqued;
        if !torqued {
            let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/torque", json!({"enable": true})).await;
            assert_eq!(ack["type"], "command_ack", "{ack}");
            let mut inner = sup.inner().write().await;
            inner.commanded_positions = common::INSPECT.to_vec();
            inner.measured_positions = common::INSPECT.to_vec();
            inner.target_positions = common::INSPECT.to_vec();
        }
        let epoch = sup.get_observation().await.epoch_id;
        (ws, generation, epoch)
    }

    async fn rpc(&mut self, path: &str, body: Value) -> Value { common::rpc(&mut self.ws, self.generation, self.epoch, path, body).await }

    async fn heartbeat(&mut self) { common::send(&mut self.ws, json!({"type":"heartbeat", "generation": self.generation, "epoch_id": self.epoch})).await; }

    async fn job(&self, id: &str) -> Value {
        reqwest::get(format!("http://{}/api/v1/calibration/hover/{id}", self.addr)).await.unwrap().json().await.unwrap()
    }

    /// Polls the job (heartbeats keep the lease while `keep_lease`) until `done(job)`.
    async fn wait_job(&mut self, id: &str, max_s: f64, keep_lease: bool, done: impl Fn(&Value) -> bool) -> Value {
        let until = Instant::now() + Duration::from_secs_f64(max_s);
        loop {
            let j = self.job(id).await;
            if done(&j) { return j; }
            assert!(Instant::now() < until, "job {id} timed out: {j}");
            if keep_lease { self.heartbeat().await; common::drain(&mut self.ws, 5).await; }
            tokio::time::sleep(Duration::from_millis(80)).await;
        }
    }
}

fn point(id: &str, x: f64, y: f64) -> Value {
    json!({"id": id, "approach_m": [x, y, 0.08], "hover_m": [x, y, 0.06]})
}

fn body(points: Vec<Value>) -> Value {
    json!({"candidate_fingerprint": "cafe0123", "points": points, "rpy_rad": DOWN, "approach_speed_mps": 0.05, "descent_speed_mps": 0.015, "settle_s": 2.5})
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn hover_job_visits_points_skips_refusals_and_records_fresh_frames() {
    let mut r = Rig::new().await;
    // request bounds
    for (k, v) in [("descent_speed_mps", json!(0.02)), ("settle_s", json!(1.0)), ("discard", json!(1)), ("frames", json!(9))] {
        let mut b = body(vec![point("a", 0.3, 0.0)]);
        b[k] = v;
        let rej = r.rpc(armd::calib_hover::PATH, b).await;
        assert_eq!((rej["type"].as_str(), rej["code"].as_str()), (Some("command_reject"), Some("INVALID_COMMAND")), "{k}: {rej}");
    }
    let rej = r.rpc(armd::calib_hover::PATH, body(vec![json!({"id": "a", "approach_m": [0.3, 0.0, 0.08], "hover_m": [0.31, 0.0, 0.06]})])).await;
    assert_eq!(rej["code"], "INVALID_COMMAND", "descent not vertical: {rej}");
    let rej = r.rpc(armd::calib_hover::PATH, body(vec![json!({"id": "a", "approach_m": [0.3, 0.0, 0.03], "hover_m": [0.3, 0.0, 0.002]})])).await;
    assert_eq!(rej["code"], "WORKSPACE_VIOLATION", "hover below floor + 5 mm: {rej}");
    let rej = r.rpc(armd::calib_hover::PATH, body((0..9).map(|i| point(&format!("p{i}"), 0.3, 0.0)).collect())).await;
    assert_eq!(rej["code"], "INVALID_COMMAND", "too many points: {rej}");

    let ack = r.rpc(armd::calib_hover::PATH, body(vec![point("a", 0.30, 0.03), point("far", 0.75, 0.0), point("b", 0.28, -0.03)])).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let job_id = ack["job_id"].as_str().unwrap().to_string();
    let job = r.wait_job(&job_id, 90.0, true, |j| j["state"] != "running").await;
    assert_eq!(job["state"], "done", "{job}");
    let pts = job["points"].as_array().unwrap();
    assert_eq!(pts[1]["state"], "skipped", "{job}");
    assert!(["IK_UNREACHABLE", "WORKSPACE_VIOLATION", "IK_BRANCH_FLIP"].contains(&pts[1]["code"].as_str().unwrap()), "{}", pts[1]);
    for p in [&pts[0], &pts[2]] {
        assert_eq!(p["state"], "done", "{p}");
        let obs = p["observations"].as_array().unwrap();
        assert_eq!(obs.len(), 3, "{p}");
        assert_eq!(p["discarded"].as_array().unwrap().len(), 2);
        let ids: Vec<u64> = obs.iter().map(|o| o["frame_id"].as_u64().unwrap()).collect();
        assert!(ids.windows(2).all(|w| w[1] > w[0]) && ids[0] > p["discarded"][1].as_u64().unwrap(), "fresh frames after the discarded ones: {ids:?}");
        assert_eq!(obs[0]["markers"].as_array().unwrap().len(), 1, "only board ids 0..11");
        // the fingertip hovered at the target (sim follows exactly)
        let end = p["settled_end_m"].as_array().unwrap();
        let want = p["hover_m"].as_array().unwrap();
        for k in 0..3 { assert!((end[k].as_f64().unwrap() - want[k].as_f64().unwrap()).abs() < 0.002, "{p}"); }
    }
    assert!(r.triggers.load(Ordering::SeqCst) >= 10);
    let action = r.sup.inner().read().await.last_action.clone().unwrap();
    assert_eq!((action.kind.as_str(), action.state.as_str(), action.tag.as_deref()), ("calib_hover", "done", Some(job_id.as_str())));
    // the job file is on the device
    let list: Value = reqwest::get(format!("http://{}/api/v1/calibration/hover", r.addr)).await.unwrap().json().await.unwrap();
    assert_eq!(list["jobs"][0]["id"], job_id.as_str(), "{list}");
    assert_eq!(list["jobs"][0]["done"], 2);
    assert_eq!(job["end"]["is_torqued"], true, "the arm holds at the last approach point: {job}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn disconnect_during_descent_parks_keeps_observations_and_resumes() {
    let mut r = Rig::new().await;
    let ack = r.rpc(armd::calib_hover::PATH, body(vec![point("a", 0.30, 0.03), point("b", 0.28, -0.03)])).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let job_id = ack["job_id"].as_str().unwrap().to_string();
    // wait until the second point descends
    let sup = r.sup.clone();
    let until = Instant::now() + Duration::from_secs(60);
    loop {
        let phase = sup.inner().read().await.last_action.as_ref().and_then(|a| a.phase.clone());
        if phase.as_deref() == Some("descend:b") { break; }
        assert!(Instant::now() < until, "no descent of b (phase {phase:?})");
        r.heartbeat().await;
        common::drain(&mut r.ws, 5).await;
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
    // the browser is gone: no heartbeat, the socket closes
    let _ = r.ws.close(None).await;
    let job = r.wait_job(&job_id, 10.0, false, |j| j["state"] != "running").await;
    assert_eq!(job["state"], "cancelled", "{job}");
    assert_eq!(job["code"], "LEASE_LOST", "{job}");
    assert_eq!(job["points"][0]["state"], "done");
    assert_eq!(job["points"][0]["observations"].as_array().unwrap().len(), 3, "completed observations stay");
    assert_ne!(job["points"][1]["state"], "done");
    // the lease watchdog parks the arm: torque off in the end, visible to a reconnecting client
    let parked = r.wait_job(&job_id, 40.0, false, |j| j["arm"]["is_torqued"] == false).await;
    assert_eq!(parked["arm"]["is_torqued"], false, "{parked}");
    // reconnect and resume by job id: only point b runs again
    let (ws, generation, epoch) = Rig::control(r.addr, &r.sup).await;
    r.ws = ws; r.generation = generation; r.epoch = epoch;
    let ack = r.rpc(armd::calib_hover::PATH, json!({"resume": job_id})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    assert_eq!(ack["job_id"], job_id.as_str());
    let first_frames = job["points"][0]["observations"].clone();
    let job = r.wait_job(&job_id, 60.0, true, |j| j["state"] != "running").await;
    assert_eq!(job["state"], "done", "{job}");
    assert_eq!(job["runs"], 2);
    assert_eq!(job["points"][0]["observations"], first_frames, "point a is not repeated");
    assert_eq!(job["points"][1]["observations"].as_array().unwrap().len(), 3);
    let rej = r.rpc(armd::calib_hover::PATH, json!({"resume": job_id})).await;
    assert_eq!(rej["code"], "INVALID_COMMAND", "nothing left: {rej}");
    let rej = r.rpc(armd::calib_hover::PATH, json!({"resume": "hv-nope"})).await;
    assert_eq!(rej["code"], "JOB_NOT_FOUND", "{rej}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn stop_during_capture_ends_the_run() {
    let mut r = Rig::new().await;
    let ack = r.rpc(armd::calib_hover::PATH, body(vec![point("a", 0.30, 0.0)])).await;
    let job_id = ack["job_id"].as_str().unwrap().to_string();
    let sup = r.sup.clone();
    let until = Instant::now() + Duration::from_secs(60);
    loop {
        let phase = sup.inner().read().await.last_action.as_ref().and_then(|a| a.phase.clone());
        if phase.as_deref() == Some("capture:a") { break; }
        assert!(Instant::now() < until, "no capture (phase {phase:?})");
        r.heartbeat().await;
        common::drain(&mut r.ws, 5).await;
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
    let stop = r.rpc("/api/v1/motion/stop", json!({})).await;
    assert_eq!(stop["type"], "command_ack", "{stop}");
    let job = r.wait_job(&job_id, 10.0, true, |j| j["state"] != "running").await;
    assert_eq!((job["state"].as_str(), job["code"].as_str()), (Some("cancelled"), Some("STOPPED")), "{job}");
    assert_ne!(job["points"][0]["state"], "done");
    assert_eq!(job["end"]["is_torqued"], true, "a stop holds the arm (torque on)");
}
