//! Gravity compensation over the WS lease: `/api/v1/mode/gravity_comp`, MOTION_BUSY for
//! motion commands, gripper and recording still accepted, REST stub (simulated arm).
mod common;
use common::*;
use serde_json::json;

use armd::model::ControlMode;
use armd::ArmConfig;

async fn ticks(sup: &armd::ArmSupervisor, n: usize) -> armd::ArmObservation {
    let mut obs = sup.get_observation().await;
    for _ in 0..n { obs = sup.tick(0.02).await; }
    obs
}

#[tokio::test]
async fn ws_mode_command_gates_motion_and_keeps_gripper_and_recording() {
    let (addr, sup) = spawn_server(ArmConfig::rebot_dm()).await;
    let (mut ws, generation) = controller(addr).await;
    let epoch = sup.get_observation().await.epoch_id;

    // entry is refused before the arm is enabled
    let r = rpc(&mut ws, generation, epoch, "/api/v1/mode/gravity_comp", json!({"enable": true})).await;
    assert_eq!((r["type"].as_str(), r["code"].as_str(), r["status"].as_u64()), (Some("command_reject"), Some("NOT_ARMED"), Some(412)), "{r}");
    let r = rpc(&mut ws, generation, epoch, "/api/v1/mode/gravity_comp", json!({})).await;
    assert_eq!(r["code"], "INVALID_COMMAND");

    let r = rpc(&mut ws, generation, epoch, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(r["type"], "command_ack", "{r}");
    let epoch = r["epoch_id"].as_u64().unwrap();
    ticks(&sup, 3).await;
    let r = rpc(&mut ws, generation, epoch, "/api/v1/mode/gravity_comp", json!({"enable": true, "scale": 1.6})).await;
    assert_eq!(r["code"], "INVALID_COMMAND", "scale outside [0.5, 1.2]: {r}");
    let r = rpc(&mut ws, generation, epoch, "/api/v1/mode/gravity_comp", json!({"enable": true, "scale": 0.9, "damping_kd": 0.5})).await;
    assert_eq!((r["type"].as_str(), r["control_mode"].as_str()), (Some("command_ack"), Some("gravity_comp")), "{r}");
    let obs = ticks(&sup, 2).await;
    assert_eq!(obs.control_mode, ControlMode::GravityComp);
    assert_eq!(obs.gravity_comp.phase.as_deref(), Some("ramp_in"));
    assert_eq!(obs.gravity_comp.scale, 0.9);
    assert_eq!(obs.gravity_comp.damping_kd, vec![0.5; 6]);
    let pos = obs.positions.clone();

    // motion is busy
    servo(&mut ws, generation, epoch, &pos).await;
    let reject = servo_reject_within(&mut ws, 500).await.expect("servo frame must be rejected");
    assert_eq!(reject["code"], "MOTION_BUSY", "{reject}");
    for (path, body) in [
        ("/api/v1/motion/jog", json!({"target_positions": pos})),
        ("/api/v1/motion/home", json!({})),
        ("/api/v1/motion/unload", json!({})),
        ("/api/v1/torque", json!({"enable": false})),
        ("/api/v1/action/move_relative", json!({"frame": "base", "delta": [0.0, 0.0, 0.02]})),
        ("/api/v1/action/grasp", json!({"label": "cup", "box_norm": [0.4, 0.4, 0.6, 0.6]})),
    ] {
        let r = rpc(&mut ws, generation, epoch, path, body).await;
        assert_eq!((r["type"].as_str(), r["code"].as_str()), (Some("command_reject"), Some("MOTION_BUSY")), "{path}: {r}");
    }
    let r = preset(&mut ws, generation, epoch, "home", None).await;
    assert_eq!((r["type"].as_str(), r["code"].as_str()), (Some("preset_reject"), Some("MOTION_BUSY")), "{r}");

    // gripper and recording are accepted
    let r = rpc(&mut ws, generation, epoch, "/api/v1/action/gripper", json!({"command": "open"})).await;
    assert_eq!(r["type"], "command_ack", "{r}");
    let client = reqwest::Client::new();
    let dir = tempfile::tempdir().unwrap();
    let res = client.post(format!("http://{addr}/api/v1/dataset/record/start"))
        .json(&json!({"output_path": dir.path().join("e.parquet")})).send().await.unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::OK);
    // REST mode route does not actuate
    let res = client.post(format!("http://{addr}/api/v1/mode/gravity_comp")).json(&json!({"enable": false})).send().await.unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::CONFLICT);
    let body: serde_json::Value = res.json().await.unwrap();
    assert!(matches!(body["code"].as_str(), Some("LEASE_ACTIVE" | "LEASE_REQUIRED")), "{body}");
    let obs = ticks(&sup, 10).await;
    assert_eq!(obs.control_mode, ControlMode::GravityComp, "still on after the rejections");

    // exit
    let r = rpc(&mut ws, generation, epoch, "/api/v1/mode/gravity_comp", json!({"enable": false})).await;
    assert_eq!(r["type"], "command_ack", "{r}");
    let obs = ticks(&sup, 6).await;
    assert_eq!(obs.control_mode, ControlMode::Position);
    assert_eq!(obs.gravity_comp.last_exit.as_ref().unwrap().code, "REQUESTED");
    let res = client.post(format!("http://{addr}/api/v1/dataset/record/save")).send().await.unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::OK);

    // stop and E-stop also exit
    let mut obs = ticks(&sup, 1).await;
    for _ in 0..800 {
        if obs.action.as_ref().is_none_or(|a| a.state != "running") && obs.safety_state == armd::SafetyState::Armed { break; }
        obs = ticks(&sup, 1).await; // let the gripper action finish
    }
    let r = rpc(&mut ws, generation, obs.epoch_id, "/api/v1/mode/gravity_comp", json!({"enable": true})).await;
    assert_eq!(r["type"], "command_ack", "{r}");
    let r = rpc(&mut ws, generation, obs.epoch_id, "/api/v1/motion/stop", json!({})).await;
    assert_eq!(r["type"], "command_ack");
    let obs = ticks(&sup, 6).await;
    assert_eq!((obs.control_mode, obs.gravity_comp.last_exit.unwrap().code.as_str()), (ControlMode::Position, "STOPPED"));
    let r = rpc(&mut ws, generation, obs.epoch_id, "/api/v1/mode/gravity_comp", json!({"enable": true})).await;
    assert_eq!(r["type"], "command_ack", "{r}");
    send(&mut ws, json!({"type": "estop"})).await;
    next_of(&mut ws, &["epoch_sync"]).await;
    let obs = ticks(&sup, 6).await;
    assert_eq!((obs.control_mode, obs.gravity_comp.last_exit.unwrap().code.as_str()), (ControlMode::Position, "ESTOP"));
    assert!(obs.is_torqued);
}

#[tokio::test]
async fn observation_reports_support_and_blocker() {
    let (addr, _sup) = spawn_server(ArmConfig::rebot_dm()).await;
    let obs: serde_json::Value = reqwest::get(format!("http://{addr}/api/v1/observation")).await.unwrap().json().await.unwrap();
    assert_eq!(obs["control_mode"], "position");
    assert_eq!(obs["gravity_comp"]["supported"], true);
    assert_eq!(obs["gravity_comp"]["blocker"]["code"], "NOT_ARMED");
    let (addr, _sup) = spawn_server(ArmConfig::default()).await;
    let obs: serde_json::Value = reqwest::get(format!("http://{addr}/api/v1/observation")).await.unwrap().json().await.unwrap();
    assert_eq!(obs["gravity_comp"]["supported"], false);
    assert_eq!(obs["gravity_comp"]["blocker"]["code"], "UNSUPPORTED");
}
