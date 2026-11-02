use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;
use futures_util::StreamExt;
use tempfile::tempdir;
use tokio::net::TcpListener;
use tokio_tungstenite::connect_async;

use armd::{
    create_router, ArmConfig, ArmObservation, ArmSupervisor, JointLimit, ObservationSchema,
    SafetyState,
};
use armd::api::rest::{
    HeartbeatRequest, HeartbeatResponse, JogRequest, MotionResponse,
    RecordActionResponse, SafetyResponse, ServoRequest, StartRecordRequest, TorqueRequest,
    TorqueResponse,
};

async fn spawn_test_server() -> (SocketAddr, Arc<ArmSupervisor>) {
    let mut config = ArmConfig::default();
    config.dof = 2;
    config.joint_names = vec!["joint_1".to_string(), "joint_2".to_string()];
    config.limits = vec![
        JointLimit::new("joint_1", -1.0, 1.0, 2.0, 5.0),
        JointLimit::new("joint_2", -2.0, 2.0, 2.0, 5.0),
    ];
    config.home_positions = vec![0.0, 0.0];
    config.lease_timeout_ms = 1000;

    let supervisor = Arc::new(ArmSupervisor::new(config));
    let router = create_router(supervisor.clone());

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();

    tokio::spawn(async move {
        axum::serve(listener, router).await.unwrap();
    });

    (addr, supervisor)
}

#[tokio::test]
async fn test_rest_api_observation_and_schema() {
    let (addr, _) = spawn_test_server().await;
    let client = reqwest::Client::new();

    // 1. GET /api/v1/observation
    let res = client
        .get(format!("http://{}/api/v1/observation", addr))
        .send()
        .await
        .unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::OK);
    let obs: ArmObservation = res.json().await.unwrap();
    assert_eq!(obs.positions.len(), 2);
    assert_eq!(obs.safety_state, SafetyState::Idle);
    assert_eq!(obs.epoch_id, 1);
    assert!(!obs.is_torqued);

    // 2. GET /api/v1/observation/schema
    let res = client
        .get(format!("http://{}/api/v1/observation/schema", addr))
        .send()
        .await
        .unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::OK);
    let schema: ObservationSchema = res.json().await.unwrap();
    assert_eq!(schema.dof, 2);
    assert_eq!(schema.joint_names, vec!["joint_1", "joint_2"]);
    assert_eq!(schema.limits.len(), 2);
}

#[tokio::test]
async fn test_rest_api_torque_and_motion() {
    let (addr, _) = spawn_test_server().await;
    let client = reqwest::Client::new();

    // 1. Try jog before torque enabled -> Precondition Failed (NotArmed)
    let jog_req = JogRequest {
        target_positions: vec![0.5, 0.5],
        duration_secs: Some(1.0),
        epoch_id: Some(1),
    };
    let res = client
        .post(format!("http://{}/api/v1/motion/jog", addr))
        .json(&jog_req)
        .send()
        .await
        .unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::PRECONDITION_FAILED);

    // 2. Enable torque -> POST /api/v1/torque
    let torque_req = TorqueRequest { enable: true };
    let res = client
        .post(format!("http://{}/api/v1/torque", addr))
        .json(&torque_req)
        .send()
        .await
        .unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::OK);
    let torque_res: TorqueResponse = res.json().await.unwrap();
    assert!(torque_res.is_torqued);
    assert_eq!(torque_res.safety_state, SafetyState::Armed);

    // 3. Jog with valid target -> Success
    let res = client
        .post(format!("http://{}/api/v1/motion/jog", addr))
        .json(&jog_req)
        .send()
        .await
        .unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::OK);
    let motion_res: MotionResponse = res.json().await.unwrap();
    assert_eq!(motion_res.status, "accepted");
    assert_eq!(motion_res.epoch_id, 1);

    // 4. Jog exceeding soft limit (joint_1 max is 1.0, sending 2.5) -> Bad Request
    let bad_jog = JogRequest {
        target_positions: vec![2.5, 0.0],
        duration_secs: Some(1.0),
        epoch_id: Some(1),
    };
    let res = client
        .post(format!("http://{}/api/v1/motion/jog", addr))
        .json(&bad_jog)
        .send()
        .await
        .unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::BAD_REQUEST);

    // 5. Jog with mismatched epoch -> Conflict (409)
    let stale_jog = JogRequest {
        target_positions: vec![0.2, 0.2],
        duration_secs: Some(1.0),
        epoch_id: Some(999), // wrong epoch
    };
    let res = client
        .post(format!("http://{}/api/v1/motion/jog", addr))
        .json(&stale_jog)
        .send()
        .await
        .unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::CONFLICT);

    // 6. Home command -> Success
    let home_req = armd::api::rest::HomeRequest {
        duration_secs: Some(0.5),
        epoch_id: Some(1),
    };
    let res = client
        .post(format!("http://{}/api/v1/motion/home", addr))
        .json(&home_req)
        .send()
        .await
        .unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::OK);
}

#[tokio::test]
async fn test_rest_api_safety_estop_lifecycle() {
    let (addr, _) = spawn_test_server().await;
    let client = reqwest::Client::new();

    // Arm arm first
    client
        .post(format!("http://{}/api/v1/torque", addr))
        .json(&TorqueRequest { enable: true })
        .send()
        .await
        .unwrap();

    // 1. POST /api/v1/safety/e_stop
    let res = client
        .post(format!("http://{}/api/v1/safety/e_stop", addr))
        .send()
        .await
        .unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::OK);
    let estop_res: SafetyResponse = res.json().await.unwrap();
    assert_eq!(estop_res.safety_state, SafetyState::EStop);
    assert_eq!(estop_res.epoch_id, 2);

    // 2. Motion while in EStop -> Conflict (409)
    let jog_req = JogRequest {
        target_positions: vec![0.1, 0.1],
        duration_secs: Some(1.0),
        epoch_id: Some(2),
    };
    let res = client
        .post(format!("http://{}/api/v1/motion/jog", addr))
        .json(&jog_req)
        .send()
        .await
        .unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::CONFLICT);

    // 3. POST /api/v1/safety/clear_fault
    let res = client
        .post(format!("http://{}/api/v1/safety/clear_fault", addr))
        .send()
        .await
        .unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::OK);
    let clear_res: SafetyResponse = res.json().await.unwrap();
    assert_eq!(clear_res.safety_state, SafetyState::Idle);
    assert_eq!(clear_res.epoch_id, 3);
}

#[tokio::test]
async fn test_rest_api_dataset_recording() {
    let (addr, supervisor) = spawn_test_server().await;
    let client = reqwest::Client::new();
    let temp_dir = tempdir().unwrap();
    let episode_path = temp_dir.path().join("api_ep.parquet");

    // 1. Start recording
    let start_req = StartRecordRequest {
        episode_id: Some("test_api_ep".to_string()),
        output_path: Some(episode_path.to_string_lossy().to_string()),
    };
    let res = client
        .post(format!("http://{}/api/v1/dataset/record/start", addr))
        .json(&start_req)
        .send()
        .await
        .unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::OK);
    let start_res: RecordActionResponse = res.json().await.unwrap();
    assert_eq!(start_res.status, "recording_started");

    // Tick supervisor so frames are recorded
    supervisor.tick(0.02).await;
    supervisor.tick(0.02).await;
    supervisor.tick(0.02).await;

    // 2. Save recording
    let res = client
        .post(format!("http://{}/api/v1/dataset/record/save", addr))
        .send()
        .await
        .unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::OK);
    let save_res: RecordActionResponse = res.json().await.unwrap();
    assert_eq!(save_res.status, "saved");
    assert_eq!(save_res.frame_count, Some(3));
    assert!(episode_path.exists());

    // 3. Start another and discard
    let discard_path = temp_dir.path().join("discarded.parquet");
    let start_req2 = StartRecordRequest {
        episode_id: Some("ep_discard".to_string()),
        output_path: Some(discard_path.to_string_lossy().to_string()),
    };
    client
        .post(format!("http://{}/api/v1/dataset/record/start", addr))
        .json(&start_req2)
        .send()
        .await
        .unwrap();
    supervisor.tick(0.02).await;

    let res = client
        .post(format!("http://{}/api/v1/dataset/record/discard", addr))
        .send()
        .await
        .unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::OK);
    let discard_res: RecordActionResponse = res.json().await.unwrap();
    assert_eq!(discard_res.status, "discarded");
    assert!(!discard_path.exists());
}

#[tokio::test]
async fn test_websocket_telemetry_stream() {
    let (addr, supervisor) = spawn_test_server().await;

    let ws_url = format!("ws://{}/api/v1/ws", addr);
    let (ws_stream, _) = connect_async(&ws_url).await.expect("Failed to connect WS");
    let (_, mut read) = ws_stream.split();

    // Trigger ticks on supervisor which broadcasts telemetry
    supervisor.tick(0.02).await;

    // Read message from WS
    let msg = tokio::time::timeout(Duration::from_secs(2), read.next())
        .await
        .expect("Timeout waiting for WS message")
        .expect("Stream closed prematurely")
        .expect("Error reading WS message");

    let text = msg.into_text().expect("Expected text message");
    let obs: ArmObservation = serde_json::from_str(&text).expect("Failed to parse observation JSON");
    assert_eq!(obs.positions.len(), 2);
    assert_eq!(obs.epoch_id, 1);
}

#[tokio::test]
async fn test_rest_api_heartbeat_and_servo() {
    let (addr, supervisor) = spawn_test_server().await;
    let client = reqwest::Client::new();

    // 1. Valid heartbeat -> 200 OK
    let hb_req = HeartbeatRequest {
        client_id: "teleop_controller".to_string(),
        epoch_id: 1,
    };
    let res = client
        .post(format!("http://{}/api/v1/heartbeat", addr))
        .json(&hb_req)
        .send()
        .await
        .unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::OK);
    let hb_res: HeartbeatResponse = res.json().await.unwrap();
    assert_eq!(hb_res.status, "ok");
    assert_eq!(hb_res.epoch_id, 1);

    // 2. Epoch mismatch heartbeat -> 409 Conflict
    let hb_bad = HeartbeatRequest {
        client_id: "teleop_controller".to_string(),
        epoch_id: 999,
    };
    let res = client
        .post(format!("http://{}/api/v1/heartbeat", addr))
        .json(&hb_bad)
        .send()
        .await
        .unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::CONFLICT);

    // 3. Servo before arming -> 412 Precondition Failed (NotArmed)
    let servo_req = ServoRequest {
        targets: vec![0.1, -0.2],
        epoch_id: Some(1),
    };
    let res = client
        .post(format!("http://{}/api/v1/motion/servo", addr))
        .json(&servo_req)
        .send()
        .await
        .unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::PRECONDITION_FAILED);

    // 4. Arm arm
    supervisor.set_torque(true).await.unwrap();

    // 5. Valid servo command -> 200 OK
    let res = client
        .post(format!("http://{}/api/v1/motion/servo", addr))
        .json(&servo_req)
        .send()
        .await
        .unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::OK);
    let motion_res: MotionResponse = res.json().await.unwrap();
    assert_eq!(motion_res.status, "accepted");
    assert_eq!(motion_res.epoch_id, 1);

    // Check that observation targets updated
    let obs = supervisor.get_observation().await;
    assert!((obs.positions[0] - 0.0).abs() < 0.2);

    // 6. Servo with soft limit exceeded -> 400 Bad Request
    let servo_bad = ServoRequest {
        targets: vec![50.0, 0.0],
        epoch_id: Some(1),
    };
    let res = client
        .post(format!("http://{}/api/v1/motion/servo", addr))
        .json(&servo_bad)
        .send()
        .await
        .unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::BAD_REQUEST);

    // 7. WebSocket bi-directional heartbeat test
    use futures_util::SinkExt;
    let ws_url = format!("ws://{}/api/v1/ws", addr);
    let (mut ws_stream, _) = connect_async(&ws_url).await.expect("Failed to connect WS");

    // Send heartbeat text frame over WS
    let ws_hb = serde_json::json!({
        "type": "heartbeat",
        "client_id": "ws_client",
        "epoch_id": 1
    });
    ws_stream
        .send(tokio_tungstenite::tungstenite::Message::Text(ws_hb.to_string().into()))
        .await
        .unwrap();

    // Give WS handler task a moment to process the message
    tokio::time::sleep(Duration::from_millis(50)).await;

    // Tick supervisor and ensure state is operational
    let obs = supervisor.tick(0.02).await;
    assert!(obs.safety_state.is_operational());
}
