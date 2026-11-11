//! Named poses: store, persistence, WS save/delete and the move_to_pose action.

mod common;

use std::sync::Arc;
use std::time::{Duration, Instant};
use serde_json::json;

use armd::model::{ArmConfig, KeepoutBox};
use armd::poses::{NamedPose, PoseList};
use armd::{kinematics, ArmSupervisor};

const INSPECT: [f32; 7] = [0.0, -1.167, -1.565, 1.435, 0.0, std::f32::consts::FRAC_PI_2, 0.0];
const READY: [f32; 7] = [0.3, -0.9, -1.3, 1.2, 0.1, 1.2, -1.0];

async fn wait_action(ws: &mut common::TestSocket, supervisor: &ArmSupervisor, generation: u64, epoch: u64, id: u64) -> armd::motion::ActionStatus {
    let deadline = Instant::now() + Duration::from_secs(15);
    let mut last_hb = Instant::now();
    loop {
        assert!(Instant::now() < deadline, "action {id} not finished");
        if last_hb.elapsed() > Duration::from_millis(200) {
            common::send(ws, json!({"type":"heartbeat", "generation":generation, "epoch_id":epoch})).await;
            last_hb = Instant::now();
        }
        common::drain(ws, 5).await;
        if let Some(a) = supervisor.get_observation().await.action.filter(|a| a.id == id && !a.is_running()) { return a; }
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
}

#[tokio::test]
async fn poses_file_round_trips_and_corrupt_file_is_not_overwritten_on_start() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("poses.json");
    let list = PoseList { poses: vec![NamedPose { name: "bin_a".into(), positions: READY.to_vec(), tcp: None, saved_at_ms: 42 }] };
    armd::persist::write_atomic(&path, &serde_json::to_string(&list).unwrap()).unwrap();
    let supervisor = ArmSupervisor::new(ArmConfig::rebot_dm());
    armd::poses::load_into(&supervisor, &path).await;
    assert_eq!(supervisor.inner().read().await.poses.list, list);

    for corrupt in ["{\"poses\": [", "{\"poses\": [{\"name\": \"bad name\", \"positions\": [0,0,0,0,0,0,0], \"tcp\": null, \"saved_at_ms\": 0}]}",
                    "{\"poses\": [{\"name\": \"short\", \"positions\": [0,0], \"tcp\": null, \"saved_at_ms\": 0}]}"] {
        std::fs::write(&path, corrupt).unwrap();
        let supervisor = ArmSupervisor::new(ArmConfig::rebot_dm());
        armd::poses::load_into(&supervisor, &path).await;
        assert!(supervisor.inner().read().await.poses.list.poses.is_empty(), "{corrupt}");
        // The invalid file is moved aside before anything can overwrite it.
        supervisor.tick(0.02).await;
        assert!(!path.exists(), "{corrupt}");
        let bad: Vec<_> = std::fs::read_dir(dir.path()).unwrap().map(|e| e.unwrap().path())
            .filter(|p| p.file_name().unwrap().to_string_lossy().starts_with("poses.json.bad-")).collect();
        assert!(bad.iter().any(|p| std::fs::read_to_string(p).unwrap() == corrupt), "{bad:?}");
        for p in bad { std::fs::remove_file(p).unwrap(); }
        // The next successful save replaces it.
        let write = {
            let mut inner = supervisor.inner().write().await;
            armd::poses::save(&mut inner, &json!({"name": "home2", "positions": INSPECT})).unwrap().1.unwrap()
        };
        write.run().await.unwrap();
        let saved: PoseList = serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
        assert_eq!(saved.poses.len(), 1);
        assert_eq!(saved.poses[0].name, "home2");
        assert!(saved.poses[0].tcp.is_some(), "kinematics profile stores the TCP");
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn ws_save_list_move_to_pose_and_delete() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("poses.json");
    let supervisor = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    armd::poses::load_into(&supervisor, &path).await;
    let (mut ws, generation, epoch, supervisor, addr) = common::torqued_rig(supervisor, &INSPECT).await;
    let rpc = |path: &'static str, body: serde_json::Value| (path, body);

    // Save the measured pose (default) and an explicit one.
    let (p, b) = rpc("/api/v1/poses/save", json!({"name": "inspect"}));
    let ack = common::rpc(&mut ws, generation, epoch, p, b).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    assert!(ack.get("action_id").is_none());
    assert_eq!(ack["persisted"], true);
    let saved: Vec<f32> = serde_json::from_value(ack["pose"]["positions"].clone()).unwrap();
    assert!(saved.iter().zip(INSPECT).all(|(a, b)| (a - b).abs() < 1e-6));
    let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/poses/save", json!({"name": "ready", "positions": READY})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let tcp = &ack["pose"]["tcp"];
    let fk = kinematics::fk(&kinematics::q6(&READY));
    assert!((tcp["position"][0].as_f64().unwrap() - fk.p[0]).abs() < 1e-9);

    // Validation.
    for (body, code) in [
        (json!({"name": "bad name"}), "INVALID_COMMAND"),
        (json!({"name": "x", "positions": [0.0, 0.0]}), "INVALID_COMMAND"),
        (json!({"name": "x", "positions": [9.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}), "INVALID_COMMAND"),
    ] {
        let r = common::rpc(&mut ws, generation, epoch, "/api/v1/poses/save", body.clone()).await;
        assert_eq!(r["code"], code, "{body}: {r}");
    }

    let list: serde_json::Value = reqwest::get(format!("http://{addr}/api/v1/poses")).await.unwrap().json().await.unwrap();
    let names: Vec<&str> = list["poses"].as_array().unwrap().iter().map(|p| p["name"].as_str().unwrap()).collect();
    assert_eq!(names, ["inspect", "ready"]);
    assert!(list["poses"][0]["saved_at_ms"].as_u64().unwrap() > 1_600_000_000_000);
    let file: PoseList = serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    assert_eq!(file.poses.len(), 2);

    // Move to "ready" without the gripper: J7 keeps its command.
    let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/action/move_to_pose", json!({"name": "ready", "speed_scale": 1.0, "tag": "t1"})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let id = ack["action_id"].as_u64().unwrap();
    // A second action while it runs is refused (action preconditions).
    let busy = common::rpc(&mut ws, generation, epoch, "/api/v1/action/move_to_pose", json!({"name": "inspect"})).await;
    assert_eq!(busy["code"], "MOTION_BUSY", "{busy}");
    let a = wait_action(&mut ws, &supervisor, generation, epoch, id).await;
    assert_eq!((a.kind.as_str(), a.state.as_str(), a.tag.as_deref()), ("move_to_pose", "done", Some("t1")), "{a:?}");
    let q = supervisor.inner().read().await.commanded_positions.clone();
    assert!(q[..6].iter().zip(&READY[..6]).all(|(a, b)| (a - b).abs() < 1e-3), "{q:?}");
    assert!((q[6] - INSPECT[6]).abs() < 1e-6, "gripper untouched");

    // Half speed takes longer than full speed for the same move.
    let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/action/move_to_pose", json!({"name": "inspect"})).await;
    let full = ack["expected_duration_s"].as_f64().unwrap();
    let id = ack["action_id"].as_u64().unwrap();
    wait_action(&mut ws, &supervisor, generation, epoch, id).await;
    let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/action/move_to_pose", json!({"name": "ready", "speed_scale": 0.5, "include_gripper": true})).await;
    let half = ack["expected_duration_s"].as_f64().unwrap();
    assert!(half > full * 1.5, "speed_scale 0.5: {full} -> {half}");
    let id = ack["action_id"].as_u64().unwrap();
    wait_action(&mut ws, &supervisor, generation, epoch, id).await;
    assert!((supervisor.inner().read().await.commanded_positions[6] - READY[6]).abs() < 1e-3, "include_gripper moves J7");

    // Errors.
    for (body, code) in [
        (json!({"name": "nope"}), "POSE_NOT_FOUND"),
        (json!({"name": "ready", "speed_scale": 0.05}), "INVALID_COMMAND"),
        (json!({"name": "ready", "include_gripper": "yes"}), "INVALID_COMMAND"),
        (json!({}), "INVALID_COMMAND"),
    ] {
        let r = common::rpc(&mut ws, generation, epoch, "/api/v1/action/move_to_pose", body.clone()).await;
        assert_eq!(r["code"], code, "{body}: {r}");
    }
    let r = common::rpc(&mut ws, generation, epoch, "/api/v1/action/move_to_pose", json!({"name": "nope"})).await;
    assert_eq!(r["status"], 404);

    // Keep-out at the target refuses the move.
    let c = kinematics::fk(&kinematics::q6(&INSPECT)).p;
    let b = KeepoutBox { name: "fixture".into(), min_m: c.map(|v| (v - 0.01) as f32), max_m: c.map(|v| (v + 0.01) as f32) };
    let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/workspace/keepout", json!({"boxes": [b]})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let r = common::rpc(&mut ws, generation, epoch, "/api/v1/action/move_to_pose", json!({"name": "inspect"})).await;
    assert_eq!(r["code"], "WORKSPACE_VIOLATION", "{r}");

    // Delete.
    let ack = common::rpc(&mut ws, generation, epoch, "/api/v1/poses/delete", json!({"name": "inspect"})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let r = common::rpc(&mut ws, generation, epoch, "/api/v1/poses/delete", json!({"name": "inspect"})).await;
    assert_eq!((r["code"].as_str(), r["status"].as_u64()), (Some("POSE_NOT_FOUND"), Some(404)), "{r}");
    let file: PoseList = serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    assert_eq!(file.poses.iter().map(|p| p.name.as_str()).collect::<Vec<_>>(), ["ready"]);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn pose_commands_need_the_lease() {
    let (addr, _supervisor) = common::spawn_server(ArmConfig::rebot_dm()).await;
    let (mut ws, _) = tokio_tungstenite::connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
    for path in ["/api/v1/poses/save", "/api/v1/poses/delete", "/api/v1/action/move_to_pose", "/api/v1/workspace/keepout"] {
        let r = common::rpc(&mut ws, 0, 1, path, json!({"name": "a"})).await;
        assert_eq!(r["code"], "LEASE_LOST", "{path}: {r}");
    }
    let rest = reqwest::Client::new().post(format!("http://{addr}/api/v1/action/move_to_pose")).json(&json!({"name": "a"})).send().await.unwrap();
    assert!(!rest.status().is_success());
}

#[tokio::test]
async fn at_most_64_poses() {
    let supervisor = ArmSupervisor::new(ArmConfig::rebot_dm());
    let mut inner = supervisor.inner().write().await;
    for i in 0..64 {
        armd::poses::save(&mut inner, &json!({"name": format!("p{i}"), "positions": INSPECT})).unwrap();
    }
    assert!(armd::poses::save(&mut inner, &json!({"name": "p64", "positions": INSPECT})).is_err());
    // Overwriting an existing name is allowed at the limit.
    assert!(armd::poses::save(&mut inner, &json!({"name": "p3", "positions": READY})).is_ok());
    assert_eq!(inner.poses.list.poses.len(), 64);
}
