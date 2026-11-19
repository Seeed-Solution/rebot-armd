//! Teach v1: pose ids, IN_USE, sequences and the run_sequence action on the mock follower.
mod common;

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use armd::model::ArmConfig;
use armd::sequences::{Sequence, Step};
use armd::ArmSupervisor;
use common::INSPECT;
use serde_json::{json, Value};

fn profile() -> ArmConfig { ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() } }

static REQ: AtomicU64 = AtomicU64::new(1_000_000);

async fn free_cmd(ws: &mut common::TestSocket, path: &str, body: Value) -> Value {
    common::ws_reply(ws, json!({"type":"command", "request_id": REQ.fetch_add(1, Ordering::Relaxed), "path": path, "body": body}),
        &["command_ack", "command_reject"]).await
}

fn seq(id: &str, name: &str, steps: Vec<Step>) -> Sequence {
    Sequence { id: id.into(), name: name.into(), description: String::new(), steps, created_at: String::new(), updated_at: String::new(), rev: 1 }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn pose_ids_are_stable_and_referenced_items_are_in_use() {
    let tmp = tempfile::tempdir().unwrap();
    let supervisor = Arc::new(ArmSupervisor::new(profile()));
    supervisor.set_episodes_dir(Some(tmp.path().join("episodes"))).await;
    let (mut ws, generation, epoch, sup, addr) = common::torqued_rig(supervisor, &INSPECT).await;

    let a = common::rpc(&mut ws, generation, epoch, "/api/v1/poses/save", json!({"name": "a"})).await;
    let id = a["pose"]["id"].as_str().unwrap().to_string();
    assert!(armd::recording::is_ulid(&id), "{a}");
    // saving the same name keeps the id; a rename by id keeps it too
    let again = common::rpc(&mut ws, generation, epoch, "/api/v1/poses/save", json!({"name": "a"})).await;
    assert_eq!(again["pose"]["id"], id.as_str());
    let renamed = common::rpc(&mut ws, generation, epoch, "/api/v1/poses/save", json!({"name": "a2", "id": id})).await;
    assert_eq!((renamed["pose"]["id"].as_str(), renamed["pose"]["name"].as_str()), (Some(id.as_str()), Some("a2")), "{renamed}");
    assert_eq!(sup.inner().read().await.poses.list.poses.len(), 1);
    // poses/rename keeps the id
    let r = common::rpc(&mut ws, generation, epoch, "/api/v1/poses/rename", json!({"id": id, "name": "a3"})).await;
    assert_eq!((r["pose"]["id"].as_str(), r["pose"]["name"].as_str()), (Some(id.as_str()), Some("a3")), "{r}");
    let r = common::rpc(&mut ws, generation, epoch, "/api/v1/poses/rename", json!({"id": id, "name": "a2"})).await;
    assert_eq!(r["pose"]["name"], "a2", "{r}");
    let r = common::rpc(&mut ws, generation, epoch, "/api/v1/poses/rename", json!({"id": "01ARZ3NDEKTSV4RRFFQ69G5FAV", "name": "z"})).await;
    assert_eq!(r["code"], "POSE_NOT_FOUND", "{r}");
    let listed: Value = reqwest::get(format!("http://{addr}/api/v1/poses")).await.unwrap().json().await.unwrap();
    assert_eq!(listed["poses"][0]["id"], id.as_str());
    assert!(listed["poses"][0].get("stale").is_some_and(Value::is_null), "{listed}");
    let b = common::rpc(&mut ws, generation, epoch, "/api/v1/poses/save", json!({"name": "b"})).await;
    let r = common::rpc(&mut ws, generation, epoch, "/api/v1/poses/rename", json!({"id": id, "name": "b"})).await;
    assert_eq!(r["code"], "POSE_EXISTS", "{r}");
    let r = common::rpc(&mut ws, generation, epoch, "/api/v1/poses/save", json!({"name": "b", "id": id})).await;
    assert_eq!(r["code"], "POSE_EXISTS", "{r}");
    let r = common::rpc(&mut ws, generation, epoch, "/api/v1/poses/save", json!({"name": "c", "id": "01ARZ3NDEKTSV4RRFFQ69G5FAV"})).await;
    assert_eq!(r["code"], "POSE_NOT_FOUND", "{r}");

    // an episode
    sup.start_recording(None, None).await.unwrap();
    tokio::time::sleep(Duration::from_millis(200)).await;
    let path = sup.save_recording().await.unwrap();
    let ep = path.file_stem().unwrap().to_string_lossy().into_owned();

    // a sequence referencing pose a2 and the episode
    let sid = armd::recording::new_ulid();
    sup.inner().write().await.sequences.file.sequences.push(seq(&sid, "s", vec![
        Step::Pose { pose_id: id.clone(), speed_scale: 0.5, gripper: Default::default() },
        Step::Replay { episode_id: ep.clone(), speed: 0.5, gripper: true },
    ]));
    let r = common::rpc(&mut ws, generation, epoch, "/api/v1/poses/delete", json!({"name": "a2"})).await;
    assert_eq!((r["code"].as_str(), r["sequences"].clone()), (Some("IN_USE"), json!([sid])), "{r}");
    let (mut free, _) = tokio_tungstenite::connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
    let r = free_cmd(&mut free, "/api/v1/episodes/delete", json!({"id": ep})).await;
    assert_eq!((r["code"].as_str(), r["sequences"].clone()), (Some("IN_USE"), json!([sid])), "{r}");
    let r = free_cmd(&mut free, "/api/v1/datasets/delete", json!({"name": "default", "force": true})).await;
    assert_eq!(r["code"], "IN_USE", "{r}");
    let res = reqwest::Client::new().delete(format!("http://{addr}/api/v1/dataset/episodes/{ep}")).send().await.unwrap();
    assert_eq!(res.status().as_u16(), 409);
    assert!(sup.episode_store().await.contains(&ep), "still listed");
    // pose b is free
    let r = common::rpc(&mut ws, generation, epoch, "/api/v1/poses/delete", json!({"name": b["pose"]["name"]})).await;
    assert_eq!(r["type"], "command_ack", "{r}");
    // once the reference is gone both can be deleted
    sup.inner().write().await.sequences.file.sequences.clear();
    assert_eq!(common::rpc(&mut ws, generation, epoch, "/api/v1/poses/delete", json!({"name": "a2"})).await["type"], "command_ack");
    assert_eq!(free_cmd(&mut free, "/api/v1/episodes/delete", json!({"id": ep})).await["type"], "command_ack");
}

// ---- sequences and run_sequence on the mock follower ----

const A: [f32; 7] = [0.3, -1.0, -1.4, 1.3, 0.1, 0.2, 0.0];
const B: [f32; 7] = [-0.2, -1.2, -1.5, 1.4, -0.1, 0.0, 0.0];
const C: [f32; 7] = [0.5, -1.1, -1.5, 1.4, 0.0, 0.1, 0.0];

struct Rig {
    ws: common::TestSocket,
    generation: u64,
    epoch: u64,
    sup: Arc<ArmSupervisor>,
    addr: std::net::SocketAddr,
    _tmp: tempfile::TempDir,
    a: String,
    b: String,
}

impl Rig {
    async fn new() -> Self {
        let tmp = tempfile::tempdir().unwrap();
        let supervisor = Arc::new(ArmSupervisor::new(profile()));
        supervisor.set_episodes_dir(Some(tmp.path().join("episodes"))).await;
        armd::sequences::load_into(&supervisor, &tmp.path().join("sequences.json")).await;
        let (ws, generation, epoch, sup, addr) = common::torqued_rig(supervisor, &INSPECT).await;
        let mut rig = Rig { ws, generation, epoch, sup, addr, _tmp: tmp, a: String::new(), b: String::new() };
        rig.a = rig.rpc("/api/v1/poses/save", json!({"name": "A", "positions": A})).await["pose"]["id"].as_str().unwrap().to_string();
        rig.b = rig.rpc("/api/v1/poses/save", json!({"name": "B", "positions": B})).await["pose"]["id"].as_str().unwrap().to_string();
        rig
    }

    async fn rpc(&mut self, path: &str, body: Value) -> Value {
        common::rpc(&mut self.ws, self.generation, self.epoch, path, body).await
    }

    async fn save(&mut self, name: &str, steps: Value) -> String {
        let r = self.rpc("/api/v1/sequences/save", json!({"name": name, "steps": steps})).await;
        assert_eq!(r["type"], "command_ack", "{r}");
        r["sequence"]["id"].as_str().unwrap().to_string()
    }

    async fn run(&mut self, body: Value) -> (u64, f64) {
        let r = self.rpc("/api/v1/action/run_sequence", body).await;
        assert_eq!(r["type"], "command_ack", "{r}");
        (r["action_id"].as_u64().unwrap(), r["expected_duration_s"].as_f64().unwrap())
    }

    /// Polls the action until it ends; returns it and the (repeat, step) pairs seen in order.
    async fn wait(&mut self, id: u64, max_s: f64, mut each: impl FnMut(&armd::motion::ActionStatus)) -> (armd::motion::ActionStatus, Vec<(u32, usize)>) {
        let until = Instant::now() + Duration::from_secs_f64(max_s);
        let mut seen: Vec<(u32, usize)> = Vec::new();
        loop {
            assert!(Instant::now() < until, "action {id} did not end in {max_s} s");
            common::drain(&mut self.ws, 2).await;
            let a = self.sup.inner().read().await.last_action.clone().filter(|a| a.id == id).expect("the action");
            if let (Some(r), Some(k)) = (a.repeat_index, a.step_index) {
                if seen.last() != Some(&(r, k)) { seen.push((r, k)); }
            }
            each(&a);
            if !a.is_running() { return (a, seen); }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }

    async fn commanded(&self) -> Vec<f32> { self.sup.inner().read().await.commanded_positions.clone() }

    /// An episode in the store: J1 from INSPECT by `dj1` over `secs`.
    async fn episode(&self, dj1: f32, secs: f64) -> String {
        let st = self.sup.episode_store().await;
        self.sup.ensure_episodes_loaded().await.unwrap();
        let n = (secs / 0.02) as usize;
        let frames: Vec<armd::RecordFrame> = (0..=n).map(|k| {
            let u = k as f32 / n as f32;
            let mut q = INSPECT.to_vec();
            q[0] += dj1 * u * u * (3.0 - 2.0 * u);
            armd::RecordFrame { timestamp: k as f64 * 0.02, observation_state: q.clone(), action: q }
        }).collect();
        let inner = self.sup.inner().read().await;
        let id = armd::recording::new_ulid();
        let sc = armd::recording::EpisodeSidecar {
            id: id.clone(), dataset: "default".into(), task: "test".into(), source: "sdk".into(), sequence_id: None, repeat_index: None,
            outcome: "ok".into(), recorded_at: "2026-11-19T00:00:00.000Z".into(), duration_s: secs, frames: frames.len(), joint_names: inner.joint_names(),
            units: "rad".into(), joint_zero: armd::recording::JointZeroStamp::of(&inner.joint_zero_offsets), profile_id: inner.config.profile_id.clone(),
            armd_version: "test".into(), fps_nominal: 50.0, truncated: false, stopped_by: None, arm_dof: Some(inner.config.arm_dof()),
            gripper_joint: inner.config.gripper_index(), recovered: None,
            first_state: Some(frames[0].observation_state.clone()), last_state: Some(frames[frames.len() - 1].observation_state.clone()),
        };
        drop(inner);
        tokio::task::spawn_blocking(move || st.write_episode(&sc, &frames, None)).await.unwrap().unwrap();
        id
    }
}

fn close(a: &[f32], b: &[f32], joints: usize) -> bool { a.iter().zip(b).take(joints).all(|(x, y)| (x - y).abs() < 2e-3) }

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn save_validates_and_steps_run_in_order() {
    let mut rig = Rig::new().await;
    let (a, b) = (rig.a.clone(), rig.b.clone());
    // validation
    let r = rig.rpc("/api/v1/sequences/save", json!({"name": "s", "steps": [{"type": "pose", "pose_id": "01ARZ3NDEKTSV4RRFFQ69G5FAV"}]})).await;
    assert_eq!((r["code"].as_str(), r["step"].as_u64()), (Some("SEQUENCE_REF_MISSING"), Some(0)), "{r}");
    let r = rig.rpc("/api/v1/sequences/save", json!({"name": "s", "steps": [{"type": "wait", "s": 1}, {"type": "replay", "episode_id": "nope"}]})).await;
    assert_eq!((r["code"].as_str(), r["step"].as_u64()), (Some("SEQUENCE_REF_MISSING"), Some(1)), "{r}");
    for bad in [json!([{"type": "pose", "pose_id": a, "speed_scale": 1.5}]), json!([{"type": "wait", "s": 61}]),
                json!([{"type": "gripper", "action": "move"}]), json!([{"type": "jump"}]), json!([{"type": "wait", "s": 1, "x": 2}])] {
        let r = rig.rpc("/api/v1/sequences/save", json!({"name": "s", "steps": bad})).await;
        assert_eq!(r["code"], "INVALID_COMMAND", "{bad}: {r}");
    }
    assert_eq!(rig.rpc("/api/v1/sequences/save", json!({"name": "a/b", "steps": []})).await["code"], "INVALID_COMMAND");
    let steps = json!([
        {"type": "pose", "pose_id": a, "speed_scale": 1.0},
        {"type": "gripper", "action": "move", "fraction": 0.5},
        {"type": "wait", "s": 0.2},
        {"type": "pose", "pose_id": b, "speed_scale": 0.8, "gripper": "pose"},
        {"type": "gripper", "action": "open"},
    ]);
    let r = rig.rpc("/api/v1/sequences/save", json!({"name": "取放 1", "description": "d", "steps": steps})).await;
    assert_eq!((r["type"].as_str(), r["sequence"]["rev"].as_u64()), (Some("command_ack"), Some(1)), "{r}");
    let sid = r["sequence"]["id"].as_str().unwrap().to_string();
    assert_eq!(rig.rpc("/api/v1/sequences/save", json!({"name": "取放 1", "steps": []})).await["code"], "SEQUENCE_EXISTS");
    let r = rig.rpc("/api/v1/sequences/save", json!({"id": sid, "name": "取放 1", "steps": steps, "rev": 7})).await;
    assert_eq!((r["code"].as_str(), r["rev"].as_u64()), (Some("SEQUENCE_CONFLICT"), Some(1)), "{r}");
    let r = rig.rpc("/api/v1/sequences/save", json!({"id": sid, "name": "取放 1", "steps": steps, "rev": 1})).await;
    assert_eq!(r["sequence"]["rev"], 2, "{r}");
    let list: Value = reqwest::get(format!("http://{}/api/v1/sequences", rig.addr)).await.unwrap().json().await.unwrap();
    assert_eq!((list["sequences"][0]["id"].as_str(), list["sequences"][0]["steps"].as_array().map(Vec::len)), (Some(sid.as_str()), Some(5)));
    assert_eq!(list["sequences"][0]["stale_steps"], json!([]));
    let one: Value = reqwest::get(format!("http://{}/api/v1/sequences/{sid}", rig.addr)).await.unwrap().json().await.unwrap();
    assert_eq!(one["name"], "取放 1");

    let t0 = Instant::now();
    let (id, expected) = rig.run(json!({"id": sid, "tag": "t1"})).await;
    let mut kinds = Vec::new();
    let (a_done, seen) = rig.wait(id, 30.0, |a| {
        if let Some(t) = &a.step_type { if kinds.last() != Some(t) { kinds.push(t.clone()); } }
    }).await;
    let took = t0.elapsed().as_secs_f64();
    println!("sequence: {took:.2} s, expected bound {expected:.2} s, steps {seen:?}");
    assert_eq!((a_done.state.as_str(), a_done.kind.as_str(), a_done.tag.as_deref(), a_done.failed_step), ("done", "sequence", Some("t1"), None), "{a_done:?}");
    assert_eq!(a_done.sequence_id.as_deref(), Some(sid.as_str()));
    assert_eq!(seen, vec![(0, 0), (0, 1), (0, 2), (0, 3), (0, 4)]);
    assert_eq!(kinds, vec!["pose", "gripper", "wait", "pose", "gripper"]);
    assert!(took <= expected, "{took} > {expected}");
    let q = rig.commanded().await;
    assert!(close(&q, &B, 6), "{q:?}");
    // single step / from_step
    let (id, _) = rig.run(json!({"id": sid, "from_step": 0, "single_step": true})).await;
    let (done, seen) = rig.wait(id, 20.0, |_| {}).await;
    assert_eq!((done.state.as_str(), seen), ("done", vec![(0, 0)]));
    assert!(close(&rig.commanded().await, &A, 6));
    assert_eq!(rig.rpc("/api/v1/action/run_sequence", json!({"id": sid, "from_step": 5})).await["code"], "INVALID_COMMAND");
    assert_eq!(rig.rpc("/api/v1/action/run_sequence", json!({"id": sid, "repeat": 101})).await["code"], "INVALID_COMMAND");
    // delete
    assert_eq!(rig.rpc("/api/v1/sequences/delete", json!({"id": sid})).await["type"], "command_ack");
    assert_eq!(rig.rpc("/api/v1/action/run_sequence", json!({"id": sid})).await["code"], "SEQUENCE_NOT_FOUND");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn stop_lease_loss_calibration_change_and_planning_refusal_end_the_run() {
    let mut rig = Rig::new().await;
    let a = rig.a.clone();
    let sid = rig.save("waits", json!([{"type": "wait", "s": 5}, {"type": "pose", "pose_id": a}])).await;
    // stop during the wait
    let (id, _) = rig.run(json!({"id": sid})).await;
    tokio::time::sleep(Duration::from_millis(300)).await;
    let t0 = Instant::now();
    assert_eq!(rig.rpc("/api/v1/motion/stop", json!({})).await["type"], "command_ack");
    let (done, _) = rig.wait(id, 2.0, |_| {}).await;
    assert_eq!((done.state.as_str(), done.code.as_deref(), done.step_type.as_deref()), ("cancelled", Some("STOPPED"), Some("wait")), "{done:?}");
    assert!(t0.elapsed() < Duration::from_millis(500));
    // the orchestrator releases the pins when it sees the end (next poll)
    tokio::time::sleep(Duration::from_millis(100)).await;
    assert!(rig.sup.inner().read().await.sequences.pins.is_none(), "pins released");
    assert_eq!(rig.sup.inner().read().await.last_action.as_ref().unwrap().failed_step, Some(0));
    // a joint zero / kinematics change during the wait: the next motion step is refused
    rig.epoch = rig.sup.get_observation().await.epoch_id;
    let sid2 = rig.save("short wait", json!([{"type": "wait", "s": 0.4}, {"type": "pose", "pose_id": a}])).await;
    let (id, _) = rig.run(json!({"id": sid2})).await;
    tokio::time::sleep(Duration::from_millis(100)).await;
    rig.sup.inner().write().await.calibration_revision += 1;
    let (done, _) = rig.wait(id, 3.0, |_| {}).await;
    assert_eq!((done.state.as_str(), done.code.as_deref(), done.step_index, done.failed_step), ("failed", Some("CALIBRATION_CHANGED"), Some(1), Some(1)), "{done:?}");
    // planning refusal (a replay step too fast to play) ends the run at that step
    let fast = rig.episode(1.0, 0.1).await;
    let sid3 = rig.save("too fast", json!([{"type": "pose", "pose_id": a}, {"type": "replay", "episode_id": fast, "speed": 1.0}, {"type": "wait", "s": 0.1}])).await;
    let (id, _) = rig.run(json!({"id": sid3})).await;
    let (done, seen) = rig.wait(id, 20.0, |_| {}).await;
    assert_eq!((done.state.as_str(), done.code.as_deref(), done.failed_step), ("failed", Some("REPLAY_TOO_FAST"), Some(1)), "{done:?}");
    assert_eq!(seen, vec![(0, 0), (0, 1)]);
    assert!(close(&rig.commanded().await, &A, 6));
    // lease loss during the wait
    let (id, _) = rig.run(json!({"id": sid})).await;
    tokio::time::sleep(Duration::from_millis(300)).await;
    common::send(&mut rig.ws, json!({"type": "release_lease", "generation": rig.generation, "epoch_id": rig.epoch})).await;
    let (done, _) = rig.wait(id, 2.0, |_| {}).await;
    assert_eq!((done.state.as_str(), done.code.as_deref()), ("cancelled", Some("LEASE_LOST")), "{done:?}");
    tokio::time::sleep(Duration::from_millis(100)).await;
    assert_eq!(rig.sup.inner().read().await.last_action.as_ref().unwrap().failed_step, Some(0), "set by the orchestrator once it sees the end");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn run_pins_pose_revisions_and_refuses_stale_data() {
    let mut rig = Rig::new().await;
    let a = rig.a.clone();
    let sid = rig.save("pinned", json!([{"type": "wait", "s": 0.6}, {"type": "pose", "pose_id": a}])).await;
    let (id, _) = rig.run(json!({"id": sid})).await;
    tokio::time::sleep(Duration::from_millis(150)).await;
    // the pose is edited during the run: this run keeps the revision it started with
    let r = rig.rpc("/api/v1/poses/save", json!({"name": "A", "positions": C})).await;
    assert_eq!(r["pose"]["id"], a.as_str());
    // a pinned pose cannot be deleted even after the sequence dropped it
    let (done, _) = rig.wait(id, 20.0, |_| {}).await;
    assert_eq!(done.state, "done", "{done:?}");
    assert!(close(&rig.commanded().await, &A, 6), "pinned: {:?}", rig.commanded().await);
    let (id, _) = rig.run(json!({"id": sid})).await;
    rig.wait(id, 20.0, |_| {}).await;
    assert!(close(&rig.commanded().await, &C, 6), "next run uses the new revision");

    // a pose taught with another joint zero: DATA_STALE unless allow_stale
    {
        let mut inner = rig.sup.inner().write().await;
        let p = inner.poses.list.poses.iter_mut().find(|p| p.id == a).unwrap();
        p.stale = Some(armd::calibration::CalibrationStale { reason: "software zero of joints [1]".into(), at: "2026-11-19T00:00:00Z".into() });
    }
    let r = rig.rpc("/api/v1/action/run_sequence", json!({"id": sid})).await;
    assert_eq!((r["code"].as_str(), r["steps"].clone()), (Some("DATA_STALE"), json!([1])), "{r}");
    let list: Value = reqwest::get(format!("http://{}/api/v1/sequences", rig.addr)).await.unwrap().json().await.unwrap();
    assert_eq!(list["sequences"][0]["stale_steps"], json!([1]));
    let (id, _) = rig.run(json!({"id": sid, "allow_stale": true})).await;
    assert_eq!(rig.wait(id, 20.0, |_| {}).await.0.state, "done");
    // an episode recorded with another joint zero
    rig.sup.inner().write().await.poses.list.poses.iter_mut().for_each(|p| p.stale = None);
    let ep = rig.episode(0.1, 1.0).await;
    let sid2 = rig.save("replay", json!([{"type": "replay", "episode_id": ep, "speed": 1.0}])).await;
    rig.sup.inner().write().await.joint_zero_offsets[1] = 0.05;
    let r = rig.rpc("/api/v1/action/run_sequence", json!({"id": sid2})).await;
    assert_eq!((r["code"].as_str(), r["steps"].clone()), (Some("DATA_STALE"), json!([0])), "{r}");
    rig.sup.inner().write().await.joint_zero_offsets[1] = 0.0;
    let (id, _) = rig.run(json!({"id": sid2})).await;
    let (done, _) = rig.wait(id, 20.0, |_| {}).await;
    assert_eq!(done.state, "done", "{done:?}");
    assert!((rig.commanded().await[0] - (INSPECT[0] + 0.1)).abs() < 2e-3);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn record_saves_one_episode_per_repeat_and_discards_an_aborted_repeat() {
    let mut rig = Rig::new().await;
    let (a, b) = (rig.a.clone(), rig.b.clone());
    let ep = rig.episode(0.15, 0.6).await;
    let sid = rig.save("rec", json!([{"type": "pose", "pose_id": a}, {"type": "replay", "episode_id": ep, "speed": 1.0, "gripper": false},
                                     {"type": "pose", "pose_id": b}])).await;
    let before = rig.sup.episode_store().await.list_episodes(None, None, None, &[]).len();
    let t0 = Instant::now();
    let (id, expected) = rig.run(json!({"id": sid, "repeat": 2, "record": {"dataset": "default", "task": "pick"}})).await;
    let (done, seen) = rig.wait(id, 60.0, |_| {}).await;
    let took = t0.elapsed().as_secs_f64();
    println!("recorded run: {took:.2} s, expected bound {expected:.1} s, seen {seen:?}");
    assert!(took <= expected, "{took} > {expected}");
    assert_eq!(done.state, "done", "{done:?}");
    assert_eq!(seen, vec![(0, 0), (0, 1), (0, 2), (1, 0), (1, 1), (1, 2)]);
    assert_eq!(done.episodes.len(), 2);
    let st = rig.sup.episode_store().await;
    for (r, eid) in done.episodes.iter().enumerate() {
        let sc = st.get(eid).unwrap();
        assert_eq!((sc.source.as_str(), sc.sequence_id.as_deref(), sc.repeat_index, sc.task.as_str(), sc.outcome.as_str()),
            ("sequence", Some(sid.as_str()), Some(r as u32), "pick", "ok"));
        assert!(sc.frames > 20 && sc.duration_s > 0.5, "{sc:?}");
    }
    assert_eq!(st.list_episodes(None, None, None, &[]).len(), before + 2);
    assert!(rig.sup.inner().read().await.recording.is_none());

    // the recorder is busy: refused at the start
    rig.sup.start_recording(None, None).await.unwrap();
    let r = rig.rpc("/api/v1/action/run_sequence", json!({"id": sid, "record": {"dataset": "default"}})).await;
    assert_eq!((r["code"].as_str(), r["owner"].as_str()), (Some("RECORDER_BUSY"), Some("legacy-rest")), "{r}");
    // ... and a sequence's recording cannot be stopped by others
    rig.sup.discard_recording().await.unwrap();

    // stop during the second repeat: its episode is discarded, the first is kept
    let sid2 = rig.save("rec wait", json!([{"type": "pose", "pose_id": a}, {"type": "wait", "s": 1.0}])).await;
    let (id, _) = rig.run(json!({"id": sid2, "repeat": 3, "record": {"dataset": "default"}})).await;
    let mut stopped = false;
    let until = Instant::now() + Duration::from_secs(20);
    while !stopped {
        assert!(Instant::now() < until);
        let a = rig.sup.inner().read().await.last_action.clone().unwrap();
        if a.repeat_index == Some(1) && a.step_type.as_deref() == Some("wait") {
            let r = rig.sup.stop_recording_as(armd::teach::Stopper::Owner(armd::teach::RecorderOwner::LegacyRest), true, None, None).await;
            assert!(r.is_err(), "legacy cannot stop the sequence's recording");
            assert_eq!(rig.rpc("/api/v1/motion/stop", json!({})).await["type"], "command_ack");
            stopped = true;
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    let (done, _) = rig.wait(id, 3.0, |_| {}).await;
    assert_eq!((done.state.as_str(), done.code.as_deref(), done.episodes.len()), ("cancelled", Some("STOPPED"), 1), "{done:?}");
    tokio::time::sleep(Duration::from_millis(200)).await;
    let st = rig.sup.episode_store().await;
    let of_run: Vec<_> = st.list_episodes(None, None, None, &[]).into_iter().filter(|e| e.sidecar.sequence_id.as_deref() == Some(sid2.as_str())).collect();
    assert_eq!(of_run.len(), 1, "only the finished repeat");
    assert_eq!(of_run[0].sidecar.repeat_index, Some(0));
    assert!(rig.sup.inner().read().await.recording.is_none(), "the aborted repeat's recording is gone");
    assert_eq!(rig.sup.get_recording_state().await.0, armd::EpisodeState::Discarded);
}
