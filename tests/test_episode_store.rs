//! Episode directory (`src/recording/store.rs`): recording paths, size cap, REST listing.
mod common;

use armd::api::rest::ApiError;
use armd::model::ArmConfig;
use serde_json::json;

fn profile() -> ArmConfig { ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() } }

fn code_of(e: armd::ArmError) -> (u16, String, String) {
    let msg = e.to_string();
    let api = ApiError::from(e);
    (api.status.as_u16(), api.code.unwrap_or_default(), msg)
}

#[tokio::test]
async fn rest_recording_rejects_paths_outside_the_directory() {
    let tmp = tempfile::tempdir().unwrap();
    let (addr, supervisor) = common::spawn_server(profile()).await;
    supervisor.set_episodes_dir(Some(tmp.path().join("episodes"))).await;
    let client = reqwest::Client::new();
    let post = |body: serde_json::Value| {
        let client = client.clone();
        async move {
            let res = client.post(format!("http://{addr}/api/v1/dataset/record/start")).json(&body).send().await.unwrap();
            let status = res.status().as_u16();
            (status, res.json::<serde_json::Value>().await.unwrap())
        }
    };
    for (body, code) in [
        (json!({"output_path": "../../etc/x.parquet"}), "INVALID_OUTPUT_PATH"),
        (json!({"output_path": tmp.path().join("x.parquet")}), "INVALID_OUTPUT_PATH"),
        (json!({"output_path": "/tmp/x.parquet"}), "INVALID_OUTPUT_PATH"),
        (json!({"episode_id": "a/../b"}), "INVALID_EPISODE_ID"),
        (json!({"episode_id": "x".repeat(65)}), "INVALID_EPISODE_ID"),
    ] {
        let (status, r) = post(body.clone()).await;
        assert_eq!((status, r["code"].as_str()), (400, Some(code)), "{body}: {r}");
    }
    let (status, r) = post(json!({"episode_id": "ok_1"})).await;
    assert_eq!(status, 200, "{r}");
    assert_eq!(r["episode_id"], "ok_1");
    supervisor.tick(0.02).await;
    supervisor.tick(0.02).await;
    let r: serde_json::Value = client.post(format!("http://{addr}/api/v1/dataset/record/save")).send().await.unwrap().json().await.unwrap();
    assert_eq!((r["status"].as_str(), r["episode_id"].as_str()), (Some("saved"), Some("ok_1")), "{r}");
    assert!(tmp.path().join("episodes/ok_1.parquet").exists());
    // the same id again is refused
    let (status, r) = post(json!({"episode_id": "ok_1"})).await;
    assert_eq!((status, r["code"].as_str()), (409, Some("EPISODE_EXISTS")));
    // storage off
    supervisor.set_episodes_dir(None).await;
    let (status, r) = post(json!({})).await;
    assert_eq!((status, r["code"].as_str()), (409, Some("EPISODES_DISABLED")));
    let list: serde_json::Value = client.get(format!("http://{addr}/api/v1/dataset/episodes")).send().await.unwrap().json().await.unwrap();
    assert_eq!(list["enabled"], false);
}

#[tokio::test]
async fn save_refused_when_the_directory_is_full() {
    let tmp = tempfile::tempdir().unwrap();
    let (_addr, supervisor) = common::spawn_server(profile()).await;
    supervisor.set_episodes_dir(Some(tmp.path().to_path_buf())).await;
    supervisor.start_recording(Some("r".into()), None).await.unwrap();
    supervisor.tick(0.02).await;
    // fill the directory to the file cap after the start
    for k in 0..armd::recording::store::MAX_EPISODE_FILES { std::fs::write(tmp.path().join(format!("f{k}.parquet")), b"").unwrap(); }
    let err = supervisor.save_recording().await.unwrap_err();
    let (status, code, msg) = code_of(err);
    println!("full: {msg}");
    assert_eq!((status, code.as_str()), (507, "EPISODES_FULL"));
    // still recording: frees space, then saves
    assert_eq!(supervisor.get_recording_state().await.0, armd::EpisodeState::Recording);
    std::fs::remove_file(tmp.path().join("f0.parquet")).unwrap();
    supervisor.save_recording().await.unwrap();
    assert!(tmp.path().join("r.parquet").exists());
}
