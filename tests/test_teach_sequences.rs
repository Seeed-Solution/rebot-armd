//! Teach v1: pose ids, IN_USE, sequences and the run_sequence action on the mock follower.
mod common;

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

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
    let b = common::rpc(&mut ws, generation, epoch, "/api/v1/poses/save", json!({"name": "b"})).await;
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
