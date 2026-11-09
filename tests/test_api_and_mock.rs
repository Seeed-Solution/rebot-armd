use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;
use futures_util::{SinkExt, StreamExt};
use tempfile::tempdir;
use tokio::net::TcpListener;
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::Message;

use armd::{
    create_router, ArmConfig, ArmObservation, ArmSupervisor, JointLimit, ObservationSchema,
    SafetyState,
};
use armd::api::rest::{
    JogRequest, RecordActionResponse, SafetyResponse, ServoRequest, StartRecordRequest,
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


type TestSocket = tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>>;
async fn ws_reply(ws: &mut TestSocket, message: serde_json::Value, expected: &[&str]) -> serde_json::Value {
    ws.send(Message::Text(message.to_string().into())).await.unwrap();
    tokio::time::timeout(Duration::from_secs(2), async {
        loop {
            let text = ws.next().await.unwrap().unwrap().into_text().unwrap();
            let value: serde_json::Value = serde_json::from_str(&text).unwrap();
            if expected.iter().any(|kind| value["type"] == *kind) { return value; }
        }
    }).await.expect("Timed out waiting for control response")
}
async fn controller(addr: SocketAddr) -> (TestSocket, u64) {
    let (mut ws, _) = connect_async(format!("ws://{}/api/v1/ws", addr)).await.unwrap();
    let reply = ws_reply(&mut ws, serde_json::json!({"type":"acquire_lease", "client_id":"test_controller"}), &["lease_granted"]).await;
    (ws, reply["generation"].as_u64().unwrap())
}
async fn rpc(ws: &mut TestSocket, generation: u64, epoch: u64, path: &str, body: serde_json::Value) -> serde_json::Value {
    static REQUEST: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1);
    ws_reply(ws, serde_json::json!({"type":"command", "request_id":REQUEST.fetch_add(1, std::sync::atomic::Ordering::Relaxed), "generation":generation, "epoch_id":epoch, "path":path, "body":body}), &["command_ack", "command_reject"]).await
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
    let (addr, supervisor) = spawn_test_server().await;
    let client = reqwest::Client::new();
    // HTTP actuator requests cannot bypass connection ownership, even before a lease exists.
    for (path, body) in [
        ("torque", serde_json::json!({"enable":true})),
        ("motion/jog", serde_json::json!({"target_positions":[0.1,0.1],"epoch_id":1})),
        ("motion/home", serde_json::json!({"epoch_id":1})),
        ("motion/unload", serde_json::json!({})),
        ("safety/clear_fault", serde_json::json!({})),
    ] {
        let response = client.post(format!("http://{}/api/v1/{}",addr,path)).json(&body).send().await.unwrap();
        assert_eq!(response.status(), reqwest::StatusCode::CONFLICT);
        assert_eq!(response.json::<serde_json::Value>().await.unwrap()["code"], "LEASE_REQUIRED");
    }
    let (mut ws,generation) = controller(addr).await;
    let jog = serde_json::json!({"target_positions":[0.5,0.5],"duration_secs":1.0});
    assert_eq!(rpc(&mut ws,generation,1,"/api/v1/motion/jog",jog.clone()).await["type"],"command_reject");
    assert_eq!(rpc(&mut ws,generation,1,"/api/v1/torque",serde_json::json!({"enable":true})).await["type"],"command_ack");
    assert!(supervisor.get_observation().await.is_torqued);
    assert_eq!(rpc(&mut ws,generation,1,"/api/v1/motion/jog",jog).await["type"],"command_ack");
    assert_eq!(rpc(&mut ws,generation,1,"/api/v1/motion/jog",serde_json::json!({"target_positions":[2.5,0.0]})).await["type"],"command_reject");
    assert_eq!(rpc(&mut ws,generation,999,"/api/v1/motion/home",serde_json::json!({})).await["code"],"EPOCH_MISMATCH");
    assert_eq!(rpc(&mut ws,generation,1,"/api/v1/motion/home",serde_json::json!({})).await["type"],"command_ack");
    assert_eq!(rpc(&mut ws,generation,1,"/api/v1/motion/unload",serde_json::json!({})).await["type"],"command_ack");
}

#[tokio::test]
async fn test_rest_api_safety_estop_lifecycle() {
    let (addr,supervisor) = spawn_test_server().await;
    let (mut ws,generation) = controller(addr).await;
    assert_eq!(rpc(&mut ws,generation,1,"/api/v1/torque",serde_json::json!({"enable":true})).await["type"],"command_ack");
    let response = reqwest::Client::new().post(format!("http://{}/api/v1/safety/e_stop",addr)).send().await.unwrap();
    assert_eq!(response.status(), reqwest::StatusCode::OK);
    let stopped: SafetyResponse = response.json().await.unwrap();
    assert_eq!(stopped.safety_state,SafetyState::EStop);
    assert_eq!(rpc(&mut ws,generation,stopped.epoch_id,"/api/v1/motion/jog",serde_json::json!({"target_positions":[0.1,0.1]})).await["type"],"command_reject");
    assert_eq!(rpc(&mut ws,generation,stopped.epoch_id,"/api/v1/torque",serde_json::json!({"enable":false})).await["type"],"command_reject");
    let clear = rpc(&mut ws,generation,stopped.epoch_id,"/api/v1/safety/clear_fault",serde_json::json!({})).await;
    assert_eq!(clear["type"],"command_ack");
    assert_eq!(clear["safety_state"],"Idle");
    assert_eq!(rpc(&mut ws,generation,stopped.epoch_id,"/api/v1/torque",serde_json::json!({"enable":true})).await["code"],"EPOCH_MISMATCH");
    let epoch = supervisor.get_observation().await.epoch_id;
    assert_eq!(rpc(&mut ws,generation,epoch,"/api/v1/torque",serde_json::json!({"enable":true})).await["type"],"command_ack");
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
    let (addr,supervisor) = spawn_test_server().await;
    let client = reqwest::Client::new();
    let response = client.post(format!("http://{}/api/v1/heartbeat",addr)).json(&serde_json::json!({"client_id":"test_controller","epoch_id":1})).send().await.unwrap();
    assert_eq!(response.status(),reqwest::StatusCode::CONFLICT);
    let (mut ws,generation) = controller(addr).await;
    let servo = serde_json::json!({"targets":[0.1,-0.2]});
    assert_eq!(rpc(&mut ws,generation,1,"/api/v1/motion/servo",servo.clone()).await["type"],"command_reject");
    supervisor.set_torque(true).await.unwrap();
    assert_eq!(rpc(&mut ws,generation,1,"/api/v1/motion/servo",servo).await["type"],"command_ack");
    assert_eq!(rpc(&mut ws,generation,1,"/api/v1/motion/servo",serde_json::json!({"targets":[50.0,0.0]})).await["type"],"command_reject");
    let before = supervisor.inner().read().await.last_heartbeat;
    ws.send(Message::Text(serde_json::json!({"type":"heartbeat","generation":generation,"epoch_id":1}).to_string().into())).await.unwrap();
    // The following acknowledged command is an ordering barrier for the heartbeat.
    assert_eq!(rpc(&mut ws,generation,1,"/api/v1/motion/stop",serde_json::json!({})).await["type"],"command_ack");
    assert!(supervisor.inner().read().await.last_heartbeat > before);
    assert!(supervisor.inner().read().await.servo_target.is_none());
}

#[tokio::test]
async fn test_websocket_lease_handshake_generation_and_http_conflict() {
    use futures_util::{SinkExt, StreamExt};

    let (addr, supervisor) = spawn_test_server().await;
    supervisor.set_torque(true).await.unwrap();

    let ws_url = format!("ws://{}/api/v1/ws", addr);

    // 1. Client 1 connects and acquires lease
    let (mut ws1, _) = connect_async(&ws_url).await.expect("Failed to connect ws1");
    let req1 = serde_json::json!({
        "type": "acquire_lease",
        "client_id": "client_1"
    });
    ws1.send(tokio_tungstenite::tungstenite::Message::Text(req1.to_string().into())).await.unwrap();

    // Read messages until lease_granted is received
    let mut gen1 = None;
    for _ in 0..10 {
        if let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws1.next().await {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|t| t.as_str()) == Some("lease_granted") {
                    assert_eq!(val["client_id"], "client_1");
                    gen1 = val["generation"].as_u64();
                    break;
                }
            }
        }
    }
    let gen1 = gen1.expect("Client 1 should receive lease_granted");
    assert_eq!(gen1, 1);

    // 2. Client 2 attempts to acquire lease while Client 1 is active -> preemption intercepted (lease_rejected)
    let (mut ws2, _) = connect_async(&ws_url).await.expect("Failed to connect ws2");
    let req2 = serde_json::json!({
        "type": "acquire_lease",
        "client_id": "client_2"
    });
    ws2.send(tokio_tungstenite::tungstenite::Message::Text(req2.to_string().into())).await.unwrap();

    let mut rejected = false;
    for _ in 0..10 {
        if let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws2.next().await {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|t| t.as_str()) == Some("lease_rejected") {
                    assert!(val.get("owner").is_none());
                    assert!(val.get("generation").is_none());
                    rejected = true;
                    break;
                }
            }
        }
    }
    assert!(rejected, "Client 2 acquire_lease should be rejected due to active Client 1 lease");

    // 3. HTTP arbitration: /motion/servo and /motion/jog return 409 Conflict while lease is active
    let client = reqwest::Client::new();
    let servo_req = ServoRequest {
        targets: vec![0.1, 0.1],
        epoch_id: Some(1),
    };
    let res = client
        .post(format!("http://{}/api/v1/motion/servo", addr))
        .json(&servo_req)
        .send()
        .await
        .unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::CONFLICT, "HTTP servo must return 409 when lease is active");

    let jog_req = JogRequest {
        target_positions: vec![0.2, 0.2],
        duration_secs: Some(1.0),
        epoch_id: Some(1),
    };
    let res = client
        .post(format!("http://{}/api/v1/motion/jog", addr))
        .json(&jog_req)
        .send()
        .await
        .unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::CONFLICT, "HTTP jog must return 409 when lease is active");

    // 4. Client 1 sends valid servo frame with generation, sequence_id, and epoch_id
    let servo_frame = serde_json::json!({
        "type": "servo",
        "client_id": "client_1",
        "generation": gen1,
        "sequence_id": 1,
        "epoch_id": 1,
        "positions": [0.10, -0.10]
    });
    ws1.send(tokio_tungstenite::tungstenite::Message::Text(servo_frame.to_string().into())).await.unwrap();
    tokio::time::sleep(Duration::from_millis(50)).await;

    // Tick supervisor and ensure arm moved towards target
    supervisor.tick(0.02).await;
    let obs = supervisor.get_observation().await;
    assert!(obs.positions[0] > 0.0, "Position should advance towards servo target");

    // 5. Client 1 sends frame with wrong generation -> silently dropped
    let pos_before = obs.positions[0];
    let bad_gen_frame = serde_json::json!({
        "type": "servo",
        "client_id": "client_1",
        "generation": 999, // wrong generation
        "sequence_id": 2,
        "epoch_id": 1,
        "positions": [-0.5, 0.5]
    });
    ws1.send(tokio_tungstenite::tungstenite::Message::Text(bad_gen_frame.to_string().into())).await.unwrap();
    tokio::time::sleep(Duration::from_millis(50)).await;

    // Tick supervisor: position should not have changed towards -0.5
    supervisor.tick(0.02).await;
    let obs_after = supervisor.get_observation().await;
    assert!(obs_after.positions[0] >= pos_before, "Frame with invalid generation must be dropped");

    // 6. Release Client 1 lease explicitly
    let rel_req = serde_json::json!({
        "type": "release_lease",
        "client_id": "client_1",
        "generation": gen1
    });
    ws1.send(tokio_tungstenite::tungstenite::Message::Text(rel_req.to_string().into())).await.unwrap();
    tokio::time::sleep(Duration::from_millis(50)).await;
    drop(ws1);

    // 7. Client 2 can now acquire lease successfully
    let req2_retry = serde_json::json!({
        "type": "acquire_lease",
        "client_id": "client_2"
    });
    ws2.send(tokio_tungstenite::tungstenite::Message::Text(req2_retry.to_string().into())).await.unwrap();

    let mut gen2 = None;
    for _ in 0..10 {
        if let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws2.next().await {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|t| t.as_str()) == Some("lease_granted") {
                    assert_eq!(val["client_id"], "client_2");
                    gen2 = val["generation"].as_u64();
                    break;
                }
            }
        }
    }
    let gen2 = gen2.expect("Client 2 should acquire lease after Client 1 released");
    assert!(gen2 > gen1, "Generation counter must increment");
}

#[tokio::test]
async fn test_websocket_servo_rejects_missing_generation() {
    use futures_util::{SinkExt, StreamExt};

    let (addr, supervisor) = spawn_test_server().await;
    supervisor.set_torque(true).await.unwrap();

    let ws_url = format!("ws://{}/api/v1/ws", addr);
    let (mut ws, _) = connect_async(&ws_url).await.expect("Failed to connect ws");

    // Send servo frame without generation field
    let missing_gen_frame = serde_json::json!({
        "type": "servo",
        "client_id": "client_1",
        "sequence_id": 1,
        "epoch_id": 1,
        "positions": [0.5, -0.5]
    });
    ws.send(tokio_tungstenite::tungstenite::Message::Text(missing_gen_frame.to_string().into()))
        .await
        .unwrap();

    // Expect servo_reject message with reason "missing_generation"
    let mut rejected = false;
    for _ in 0..10 {
        if let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws.next().await {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|t| t.as_str()) == Some("servo_reject") {
                    assert_eq!(val["reason"], "missing_generation");
                    rejected = true;
                    break;
                }
            }
        }
    }
    assert!(rejected, "WebSocket servo frame without generation must be rejected with missing_generation");

    // Also verify that arm positions did not move towards the commanded target
    supervisor.tick(0.02).await;
    let obs = supervisor.get_observation().await;
    assert_eq!(obs.positions, vec![0.0, 0.0], "Arm positions must remain untouched when servo frame is rejected");
}

#[tokio::test]
async fn test_ws_rejects_missing_or_invalid_sequence_id() {
    use futures_util::{SinkExt, StreamExt};

    let (addr, supervisor) = spawn_test_server().await;
    supervisor.set_torque(true).await.unwrap();

    let ws_url = format!("ws://{}/api/v1/ws", addr);
    let (mut ws, _) = connect_async(&ws_url).await.expect("Failed to connect ws");

    // Acquire lease
    let lease_req = serde_json::json!({
        "type": "acquire_lease",
        "client_id": "test_client"
    });
    ws.send(tokio_tungstenite::tungstenite::Message::Text(lease_req.to_string().into()))
        .await
        .unwrap();

    let mut gen = None;
    for _ in 0..10 {
        if let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws.next().await {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|t| t.as_str()) == Some("lease_granted") {
                    gen = val["generation"].as_u64();
                    break;
                }
            }
        }
    }
    let gen = gen.expect("Failed to acquire lease");

    // 1. Missing sequence_id: must reject with reason "missing_sequence_id"
    let missing_seq_frame = serde_json::json!({
        "type": "servo",
        "client_id": "test_client",
        "generation": gen,
        "epoch_id": 1,
        "positions": [0.5, -0.5]
    });
    ws.send(tokio_tungstenite::tungstenite::Message::Text(missing_seq_frame.to_string().into()))
        .await
        .unwrap();

    let mut rejected_missing = false;
    for _ in 0..10 {
        if let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws.next().await {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|t| t.as_str()) == Some("servo_reject") {
                    assert_eq!(val["reason"], "missing_sequence_id");
                    rejected_missing = true;
                    break;
                }
            }
        }
    }
    assert!(rejected_missing, "Servo frame without sequence_id must be rejected");

    // 2. Valid sequence_id: 100
    let valid_frame = serde_json::json!({
        "type": "servo",
        "client_id": "test_client",
        "generation": gen,
        "sequence_id": 100,
        "epoch_id": 1,
        "positions": [0.2, -0.2]
    });
    ws.send(tokio_tungstenite::tungstenite::Message::Text(valid_frame.to_string().into()))
        .await
        .unwrap();

    // 3. Duplicate sequence_id: 100 (delta == 0) -> must reject
    let dup_frame = serde_json::json!({
        "type": "servo",
        "client_id": "test_client",
        "generation": gen,
        "sequence_id": 100,
        "epoch_id": 1,
        "positions": [0.3, -0.3]
    });
    ws.send(tokio_tungstenite::tungstenite::Message::Text(dup_frame.to_string().into()))
        .await
        .unwrap();

    let mut rejected_dup = false;
    for _ in 0..10 {
        if let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws.next().await {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|t| t.as_str()) == Some("servo_reject") {
                    assert_eq!(val["reason"], "invalid_sequence_id");
                    rejected_dup = true;
                    break;
                }
            }
        }
    }
    assert!(rejected_dup, "Duplicate sequence_id must be rejected");

    // 4. Backward sequence_id: 50 (delta = 50.wrapping_sub(100) > 1000) -> must reject
    let back_frame = serde_json::json!({
        "type": "servo",
        "client_id": "test_client",
        "generation": gen,
        "sequence_id": 50,
        "epoch_id": 1,
        "positions": [0.4, -0.4]
    });
    ws.send(tokio_tungstenite::tungstenite::Message::Text(back_frame.to_string().into()))
        .await
        .unwrap();

    let mut rejected_back = false;
    for _ in 0..10 {
        if let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws.next().await {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|t| t.as_str()) == Some("servo_reject") {
                    assert_eq!(val["reason"], "invalid_sequence_id");
                    rejected_back = true;
                    break;
                }
            }
        }
    }
    assert!(rejected_back, "Backward sequence_id must be rejected");

    // 5. Jumped sequence_id: 2000 (delta = 2000 - 100 = 1900 > 1000) -> must reject
    let jump_frame = serde_json::json!({
        "type": "servo",
        "client_id": "test_client",
        "generation": gen,
        "sequence_id": 2000,
        "epoch_id": 1,
        "positions": [0.4, -0.4]
    });
    ws.send(tokio_tungstenite::tungstenite::Message::Text(jump_frame.to_string().into()))
        .await
        .unwrap();

    let mut rejected_jump = false;
    for _ in 0..10 {
        if let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws.next().await {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|t| t.as_str()) == Some("servo_reject") {
                    assert_eq!(val["reason"], "invalid_sequence_id");
                    rejected_jump = true;
                    break;
                }
            }
        }
    }
    assert!(rejected_jump, "Unreasonably jumped sequence_id (>1000) must be rejected");
}

#[tokio::test]
async fn test_ws_connection_drop_ensures_lease_release() {
    use futures_util::{SinkExt, StreamExt};

    let (addr, supervisor) = spawn_test_server().await;
    supervisor.set_torque(true).await.unwrap();

    let ws_url = format!("ws://{}/api/v1/ws", addr);

    // 1. Client 1 connects and acquires lease
    let (mut ws1, _) = connect_async(&ws_url).await.expect("Failed to connect ws1");
    let req1 = serde_json::json!({
        "type": "acquire_lease",
        "client_id": "client_drop_test"
    });
    ws1.send(tokio_tungstenite::tungstenite::Message::Text(req1.to_string().into()))
        .await
        .unwrap();

    let mut gen1 = None;
    for _ in 0..10 {
        if let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws1.next().await {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|t| t.as_str()) == Some("lease_granted") {
                    gen1 = val["generation"].as_u64();
                    break;
                }
            }
        }
    }
    assert!(gen1.is_some(), "Client 1 must acquire lease");
    assert!(supervisor.is_lease_active().await, "Supervisor lease must be active");

    // 2. Abruptly drop client 1 WebSocket without calling release_lease
    drop(ws1);

    // Allow top-level handle_socket to clean up the scoped lease
    let mut released = false;
    for _ in 0..20 {
        tokio::time::sleep(Duration::from_millis(25)).await;
        if !supervisor.is_lease_active().await {
            released = true;
            break;
        }
    }
    assert!(released, "Dropping WS connection must automatically release the lease");

    // 3. New client can now immediately acquire lease
    let (mut ws2, _) = connect_async(&ws_url).await.expect("Failed to connect ws2");
    let req2 = serde_json::json!({
        "type": "acquire_lease",
        "client_id": "client_successor"
    });
    ws2.send(tokio_tungstenite::tungstenite::Message::Text(req2.to_string().into()))
        .await
        .unwrap();

    let mut gen2 = None;
    for _ in 0..10 {
        if let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws2.next().await {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|t| t.as_str()) == Some("lease_granted") {
                    gen2 = val["generation"].as_u64();
                    break;
                }
            }
        }
    }
    assert!(gen2.is_some(), "Successor client must be able to acquire lease cleanly");
}

#[tokio::test]
async fn test_rest_api_airborne_torque_off_rejected_409() {
    let (addr,supervisor) = spawn_test_server().await;
    let (mut ws,generation) = controller(addr).await;
    assert_eq!(rpc(&mut ws,generation,1,"/api/v1/torque",serde_json::json!({"enable":true})).await["type"],"command_ack");
    assert_eq!(rpc(&mut ws,generation,1,"/api/v1/motion/jog",serde_json::json!({"target_positions":[0.5,-0.5]})).await["type"],"command_ack");
    for _ in 0..25 { supervisor.tick(0.02).await; }
    assert!(supervisor.get_observation().await.positions[0] > 0.1);
    let rejection = rpc(&mut ws,generation,1,"/api/v1/torque",serde_json::json!({"enable":false})).await;
    assert_eq!(rejection["type"],"command_reject");
    assert_eq!(rejection["status"], 409);
    assert_eq!(rejection["code"], "ARM_NOT_AT_RESTING_POSE");
    assert_eq!(rejection["error"], "ArmNotAtRestingPose");
    assert!(rejection["message"].as_str().unwrap().contains("resting pose"));
    assert!(supervisor.get_observation().await.is_torqued);
}

#[tokio::test]
async fn test_websocket_preset_ack_and_reject() {
    use futures_util::{SinkExt, StreamExt};

    let (addr, supervisor) = spawn_test_server().await;
    supervisor.set_torque(true).await.unwrap();

    let ws_url = format!("ws://{}/api/v1/ws", addr);
    let (mut ws, _) = connect_async(&ws_url).await.expect("Failed to connect ws");

    let obs = supervisor.get_observation().await;
    let current_epoch = obs.epoch_id;

    // 1. Sending preset without holding lease -> must receive lease_lost
    let unauthed_preset = serde_json::json!({
        "type": "preset",
        "name": "home",
        "client_id": "client_test",
        "generation": 9999,
        "epoch_id": current_epoch
    });
    ws.send(tokio_tungstenite::tungstenite::Message::Text(unauthed_preset.to_string().into()))
        .await
        .unwrap();

    let mut got_lease_lost = false;
    for _ in 0..10 {
        if let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws.next().await {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|t| t.as_str()) == Some("lease_lost") {
                    got_lease_lost = true;
                    break;
                }
            }
        }
    }
    assert!(got_lease_lost, "Must receive lease_lost when preset is sent without valid lease");

    // 2. Acquire lease
    let lease_req = serde_json::json!({
        "type": "acquire_lease",
        "client_id": "client_test"
    });
    ws.send(tokio_tungstenite::tungstenite::Message::Text(lease_req.to_string().into()))
        .await
        .unwrap();

    let mut gen = None;
    for _ in 0..10 {
        if let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws.next().await {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|t| t.as_str()) == Some("lease_granted") {
                    gen = val["generation"].as_u64();
                    break;
                }
            }
        }
    }
    let gen = gen.expect("Failed to acquire lease");

    // 3. Epoch mismatch -> must receive epoch_sync
    let mismatch_preset = serde_json::json!({
        "type": "preset",
        "name": "home",
        "client_id": "client_test",
        "generation": gen,
        "epoch_id": current_epoch + 999
    });
    ws.send(tokio_tungstenite::tungstenite::Message::Text(mismatch_preset.to_string().into()))
        .await
        .unwrap();

    let mut got_epoch_sync = false;
    for _ in 0..10 {
        if let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws.next().await {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|t| t.as_str()) == Some("epoch_sync") {
                    assert_eq!(val["epoch_id"], current_epoch);
                    got_epoch_sync = true;
                    break;
                }
            }
        }
    }
    assert!(got_epoch_sync, "Must receive epoch_sync on epoch mismatch");

    // 4. Preset "home" -> receives preset_ack
    let home_preset = serde_json::json!({
        "type": "preset",
        "name": "home",
        "client_id": "client_test",
        "generation": gen,
        "epoch_id": current_epoch
    });
    ws.send(tokio_tungstenite::tungstenite::Message::Text(home_preset.to_string().into()))
        .await
        .unwrap();

    let mut got_home_ack = false;
    for _ in 0..10 {
        if let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws.next().await {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|t| t.as_str()) == Some("preset_ack") {
                    assert_eq!(val["name"], "home");
                    got_home_ack = true;
                    break;
                }
            }
        }
    }
    assert!(got_home_ack, "Must receive preset_ack for preset 'home'");

    // 5. Preset "inspect" with positions -> receives preset_ack
    let inspect_preset = serde_json::json!({
        "type": "preset",
        "name": "inspect",
        "positions": [0.1, -0.1],
        "client_id": "client_test",
        "generation": gen,
        "epoch_id": current_epoch
    });
    ws.send(tokio_tungstenite::tungstenite::Message::Text(inspect_preset.to_string().into()))
        .await
        .unwrap();

    let mut got_inspect_ack = false;
    for _ in 0..10 {
        if let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws.next().await {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|t| t.as_str()) == Some("preset_ack") {
                    assert_eq!(val["name"], "inspect");
                    got_inspect_ack = true;
                    break;
                }
            }
        }
    }
    assert!(got_inspect_ack, "Must receive preset_ack for preset 'inspect'");

    // 6. Preset "inspect" with out-of-limits positions -> receives preset_reject
    let bad_preset = serde_json::json!({
        "type": "preset",
        "name": "inspect",
        "positions": [999.0, -999.0],
        "client_id": "client_test",
        "generation": gen,
        "epoch_id": current_epoch
    });
    ws.send(tokio_tungstenite::tungstenite::Message::Text(bad_preset.to_string().into()))
        .await
        .unwrap();

    let mut got_reject = false;
    for _ in 0..10 {
        if let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws.next().await {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|t| t.as_str()) == Some("preset_reject") {
                    assert_eq!(val["name"], "inspect");
                    got_reject = true;
                    break;
                }
            }
        }
    }
    assert!(got_reject, "Must receive preset_reject for out-of-limits preset");

    // 7. Preset "park" -> receives preset_ack
    let park_preset = serde_json::json!({
        "type": "preset",
        "name": "park",
        "client_id": "client_test",
        "generation": gen,
        "epoch_id": current_epoch
    });
    ws.send(tokio_tungstenite::tungstenite::Message::Text(park_preset.to_string().into()))
        .await
        .unwrap();

    let mut got_park_ack = false;
    for _ in 0..10 {
        if let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws.next().await {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|t| t.as_str()) == Some("preset_ack") {
                    assert_eq!(val["name"], "park");
                    got_park_ack = true;
                    break;
                }
            }
        }
    }
    assert!(got_park_ack, "Must receive preset_ack for preset 'park'");

    // 8. Trigger E-stop, then send preset "park" -> receives preset_reject
    supervisor.trigger_e_stop().await;
    let estop_obs = supervisor.get_observation().await;

    let park_in_estop = serde_json::json!({
        "type": "preset",
        "name": "park",
        "client_id": "client_test",
        "generation": gen,
        "epoch_id": estop_obs.epoch_id
    });
    ws.send(tokio_tungstenite::tungstenite::Message::Text(park_in_estop.to_string().into()))
        .await
        .unwrap();

    let mut got_estop_reject = false;
    for _ in 0..10 {
        if let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws.next().await {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|t| t.as_str()) == Some("preset_reject") {
                    assert_eq!(val["name"], "park");
                    got_estop_reject = true;
                    break;
                }
            }
        }
    }
    assert!(got_estop_reject, "Must receive preset_reject for preset while stopped");
}

#[tokio::test]
async fn test_websocket_empty_client_id_rejected_as_lease_lost() {
    use futures_util::{SinkExt, StreamExt};

    let (addr, supervisor) = spawn_test_server().await;
    supervisor.set_torque(true).await.unwrap();

    let ws_url = format!("ws://{}/api/v1/ws", addr);

    // 1. Client 1 acquires a valid lease
    let (mut ws1, _) = connect_async(&ws_url).await.expect("Failed to connect ws1");
    let req1 = serde_json::json!({
        "type": "acquire_lease",
        "client_id": "valid_owner"
    });
    ws1.send(tokio_tungstenite::tungstenite::Message::Text(req1.to_string().into())).await.unwrap();

    let mut gen = None;
    for _ in 0..10 {
        if let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws1.next().await {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|t| t.as_str()) == Some("lease_granted") {
                    gen = val["generation"].as_u64();
                    break;
                }
            }
        }
    }
    let gen = gen.expect("Failed to acquire lease");

    let obs = supervisor.get_observation().await;
    let current_epoch = obs.epoch_id;

    // 2. Client 2 connects without acquiring lease and sends preset with client_id: ""
    let (mut ws2, _) = connect_async(&ws_url).await.expect("Failed to connect ws2");
    let empty_cid_preset = serde_json::json!({
        "type": "preset",
        "name": "home",
        "client_id": "",
        "generation": gen,
        "epoch_id": current_epoch
    });
    ws2.send(tokio_tungstenite::tungstenite::Message::Text(empty_cid_preset.to_string().into())).await.unwrap();

    let mut preset_lease_lost = false;
    for _ in 0..10 {
        if let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws2.next().await {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|t| t.as_str()) == Some("lease_lost") {
                    preset_lease_lost = true;
                    break;
                }
            }
        }
    }
    assert!(preset_lease_lost, "Preset with empty client_id must be rejected with lease_lost");

    // 3. Client 2 sends servo with client_id: ""
    let empty_cid_servo = serde_json::json!({
        "type": "servo",
        "client_id": "",
        "generation": gen,
        "sequence_id": 1,
        "epoch_id": current_epoch,
        "positions": [0.1, -0.1]
    });
    ws2.send(tokio_tungstenite::tungstenite::Message::Text(empty_cid_servo.to_string().into())).await.unwrap();

    let mut servo_lease_lost = false;
    for _ in 0..10 {
        if let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws2.next().await {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|t| t.as_str()) == Some("lease_lost") {
                    servo_lease_lost = true;
                    break;
                }
            }
        }
    }
    assert!(servo_lease_lost, "Servo with empty client_id must be rejected with lease_lost");
}

#[tokio::test]
async fn test_websocket_servo_rejected_during_preset_trajectory() {
    use futures_util::{SinkExt, StreamExt};

    let (addr, supervisor) = spawn_test_server().await;
    supervisor.set_torque(true).await.unwrap();

    let ws_url = format!("ws://{}/api/v1/ws", addr);
    let (mut ws, _) = connect_async(&ws_url).await.expect("Failed to connect ws");

    // 1. Acquire lease
    let lease_req = serde_json::json!({
        "type": "acquire_lease",
        "client_id": "test_client"
    });
    ws.send(tokio_tungstenite::tungstenite::Message::Text(lease_req.to_string().into())).await.unwrap();

    let mut gen = None;
    for _ in 0..10 {
        if let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws.next().await {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|t| t.as_str()) == Some("lease_granted") {
                    gen = val["generation"].as_u64();
                    break;
                }
            }
        }
    }
    let gen = gen.expect("Failed to acquire lease");

    let obs = supervisor.get_observation().await;
    let current_epoch = obs.epoch_id;

    // 2. Start a preset trajectory to target [0.5, -0.5]
    let preset_cmd = serde_json::json!({
        "type": "preset",
        "name": "target_pose",
        "positions": [0.5, -0.5],
        "client_id": "test_client",
        "generation": gen,
        "epoch_id": current_epoch
    });
    ws.send(tokio_tungstenite::tungstenite::Message::Text(preset_cmd.to_string().into())).await.unwrap();

    let mut got_ack = false;
    for _ in 0..10 {
        if let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws.next().await {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|t| t.as_str()) == Some("preset_ack") {
                    got_ack = true;
                    break;
                }
            }
        }
    }
    assert!(got_ack, "Preset must be accepted with preset_ack");
    assert!(supervisor.has_active_trajectory().await, "Trajectory must be active");

    // 3. Send servo frame while preset trajectory is executing
    let servo_cmd = serde_json::json!({
        "type": "servo",
        "client_id": "test_client",
        "generation": gen,
        "sequence_id": 1,
        "epoch_id": current_epoch,
        "positions": [0.1, -0.1]
    });
    ws.send(tokio_tungstenite::tungstenite::Message::Text(servo_cmd.to_string().into())).await.unwrap();

    let mut got_servo_reject = false;
    for _ in 0..10 {
        if let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws.next().await {
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|t| t.as_str()) == Some("servo_reject") {
                    got_servo_reject = true;
                    break;
                }
            }
        }
    }
    assert!(got_servo_reject, "Servo during active trajectory must be rejected with servo_reject");

    // 4. Verify trajectory was NOT truncated / cancelled
    assert!(supervisor.has_active_trajectory().await, "Trajectory must remain active and not truncated");
}

#[tokio::test]
async fn test_proxy_routes_and_passive_time_sync() {
    let (addr, supervisor) = spawn_test_server().await;
    let _ = supervisor;
    let client = reqwest::Client::new();

    // 1. Passive time sync call (no crash, safe bounds)
    armd::api::rest::try_sync_system_time(Some(1_730_000_000_000));
    armd::api::rest::try_sync_system_time(None);
    armd::api::rest::try_sync_system_time(Some(999)); // rejected out of bounds

    // 2. The go2rtc proxy routes are gone.
    let res = client.post(format!("http://{}/api/webrtc?src=live0", addr)).send().await.unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::NOT_FOUND);
    for path in &["/api/stream.mp4?src=live0", "/api/stream.mjpeg?src=live0", "/api/ws?src=live0"] {
        let res = client.get(format!("http://{}{}", addr, path)).send().await.unwrap();
        assert_eq!(res.status(), reqwest::StatusCode::NOT_FOUND, "{path}");
    }

    // 3. WS proxy for detection WS (/api/v1/detection/ws & /results)
    for path in &["/api/v1/detection/ws", "/results"] {
        let ws_url = format!("ws://{}{}", addr, path);
        if let Ok((mut ws_stream, _)) = connect_async(&ws_url).await {
            // Verify bidirectional framing / clean handshake without hanging
            let _ = ws_stream.send(Message::Ping(vec![1, 2, 3].into())).await;
            let _ = tokio::time::timeout(Duration::from_millis(150), ws_stream.next()).await;
            let _ = ws_stream.close(None).await;
        }
    }
}

#[tokio::test]
async fn connection_identity_cannot_be_copied_and_expired_heartbeat_cannot_revive() {
    let (addr, supervisor) = spawn_test_server().await;
    let (mut owner, generation) = controller(addr).await;
    let (mut stranger, _) = connect_async(format!("ws://{}/api/v1/ws", addr)).await.unwrap();
    let denied = ws_reply(&mut stranger, serde_json::json!({"type":"acquire_lease", "client_id":"test_controller"}), &["lease_rejected"]).await;
    assert!(denied.get("generation").is_none());
    assert!(denied.get("owner").is_none());
    let heartbeat_before = supervisor.inner().read().await.last_heartbeat;
    let fake = serde_json::json!({"type":"heartbeat","client_id":"test_controller","generation":generation,"epoch_id":1});
    assert_eq!(ws_reply(&mut stranger,fake,&["lease_lost"]).await["type"],"lease_lost");
    assert_eq!(supervisor.inner().read().await.last_heartbeat,heartbeat_before);
    assert_eq!(rpc(&mut stranger,generation,1,"/api/v1/torque",serde_json::json!({"enable":true})).await["code"],"LEASE_LOST");
    assert!(!supervisor.get_observation().await.is_torqued);
    let released = ws_reply(&mut stranger,serde_json::json!({"type":"release_lease","client_id":"test_controller","generation":generation}),&["lease_released"]).await;
    assert_eq!(released["success"],false);
    assert!(supervisor.is_lease_active().await);
    {
        let mut inner = supervisor.inner().write().await;
        inner.active_lease.as_mut().unwrap().acquired_at = std::time::Instant::now() - Duration::from_secs(2);
    }
    let old = supervisor.inner().read().await.active_lease.as_ref().unwrap().acquired_at;
    assert_eq!(ws_reply(&mut owner,serde_json::json!({"type":"heartbeat","generation":generation,"epoch_id":1}),&["lease_lost"]).await["type"],"lease_lost");
    assert_eq!(supervisor.inner().read().await.active_lease.as_ref().unwrap().acquired_at,old);
}

#[tokio::test]
async fn lease_replacement_before_command_lock_prevents_actuation() {
    let (addr,supervisor) = spawn_test_server().await;
    let (mut owner,generation) = controller(addr).await;
    let mut inner = supervisor.inner().write().await;
    owner.send(Message::Text(serde_json::json!({"type":"command","request_id":1,"generation":generation,"epoch_id":1,"path":"/api/v1/torque","body":{"enable":true}}).to_string().into())).await.unwrap();
    let identity = inner.active_lease.as_ref().unwrap().client_id.clone();
    assert!(inner.release_lease(&identity,generation));
    inner.acquire_lease("replacement").unwrap();
    drop(inner);
    let response = tokio::time::timeout(Duration::from_secs(2),owner.next()).await.unwrap().unwrap().unwrap();
    let response:serde_json::Value = serde_json::from_str(&response.into_text().unwrap()).unwrap();
    assert_eq!(response["type"],"command_reject");
    assert_eq!(response["code"],"LEASE_LOST");
    assert!(!supervisor.get_observation().await.is_torqued);
}

#[tokio::test]
async fn cross_origin_control_is_rejected_but_same_origin_estop_remains_available() {
    use tokio_tungstenite::tungstenite::client::IntoClientRequest;
    let (addr,supervisor) = spawn_test_server().await;
    let client=reqwest::Client::new();
    let response=client.post(format!("http://{}/api/v1/safety/e_stop",addr)).header("Origin","http://evil.invalid").send().await.unwrap();
    assert_eq!(response.status(),reqwest::StatusCode::FORBIDDEN);
    assert_eq!(supervisor.get_observation().await.safety_state,SafetyState::Idle);
    let mut request=format!("ws://{}/api/v1/ws",addr).into_client_request().unwrap();
    request.headers_mut().insert("Origin","http://evil.invalid".parse().unwrap());
    assert!(connect_async(request).await.is_err());
    let response=client.post(format!("http://{}/api/v1/safety/e_stop",addr)).header("Origin",format!("http://{}",addr)).send().await.unwrap();
    assert_eq!(response.status(),reqwest::StatusCode::OK);
    assert!(response.headers().get("access-control-allow-origin").is_none());
    assert_eq!(supervisor.get_observation().await.safety_state,SafetyState::EStop);
}

#[tokio::test]
async fn repeated_rpc_is_rejected_and_lease_handover_cancels_old_trajectory() {
    let (addr,supervisor)=spawn_test_server().await;
    let (mut ws,generation)=controller(addr).await;
    let command=serde_json::json!({"type":"command","request_id":1,"generation":generation,"epoch_id":1,"path":"/api/v1/torque","body":{"enable":true}});
    assert_eq!(ws_reply(&mut ws,command.clone(), &["command_ack"]).await["type"],"command_ack");
    let duplicate=ws_reply(&mut ws,command,&["command_reject"]).await;
    assert_eq!(duplicate["request_id"],1);
    assert_eq!(duplicate["code"],"INVALID_COMMAND");
    supervisor.jog(vec![0.5,-0.5],Some(1.0),Some(1)).await.unwrap();
    let mut inner=supervisor.inner().write().await;
    assert!(inner.active_trajectory.is_some());
    inner.active_lease.as_mut().unwrap().acquired_at=std::time::Instant::now()-Duration::from_secs(2);
    inner.acquire_lease("next_controller").unwrap();
    assert!(inner.active_trajectory.is_none());
    assert!(inner.servo_target.is_none());
}

struct UnloadCapabilityDriver {
    capabilities: armd::driver::DriverCapabilities,
}
impl armd::driver::ArmDriver for UnloadCapabilityDriver {
    fn capabilities(&self) -> armd::driver::DriverCapabilities { self.capabilities }
    fn enable_state(&self) -> armd::driver::EnableState {
        if self.capabilities.enable_feedback { armd::driver::EnableState::Enabled }
        else { armd::driver::EnableState::Unknown }
    }
    fn is_feedback_valid(&self) -> bool { true }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); 2] }
    fn read_joint_states(&mut self) -> Result<Vec<armd::model::JointState>, armd::driver::ArmDriverError> {
        Ok(vec![armd::model::JointState::default(); 2])
    }
    fn write_joint_targets(&mut self, _: &[armd::driver::JointTarget]) -> Result<(), armd::driver::ArmDriverError> { Ok(()) }
    fn set_torque(&mut self, _: bool) -> Result<(), armd::driver::ArmDriverError> { Ok(()) }
}

#[tokio::test]
async fn unsupported_automatic_unload_rejects_without_motion_but_home_remains_available() {
    use armd::driver::DriverCapabilities;
    let full = DriverCapabilities { velocity_feedback: true, torque_feedback: true,
        temperature_feedback: true, torque_feedforward: false, enable_feedback: true };
    for missing in ["velocity", "torque", "enable", "profile"] {
        let (addr, supervisor) = spawn_test_server().await;
        let mut capabilities = full;
        match missing {
            "velocity" => capabilities.velocity_feedback = false,
            "torque" => capabilities.torque_feedback = false,
            "enable" => capabilities.enable_feedback = false,
            _ => {},
        }
        supervisor.set_driver(Box::new(UnloadCapabilityDriver { capabilities })).await;
        {
            let mut inner = supervisor.inner().write().await;
            inner.config.resting_pose = if missing == "profile" { None } else { Some(armd::model::RestingPosePolicy {
                position_tolerances: vec![0.08; 2], max_total_deviation: 0.16, max_velocity: 0.015, joint_max_velocities: vec![],
                support_torque_limits: vec![(1, 1.0)], feedback_max_age_ms: 100,
            }) };
            // Arrange an operational controller with a queued target, so a rejected
            // unload must not secretly cancel or replace an existing command.
            inner.safety.set_torque(true).unwrap();
            inner.is_torqued = true;
            inner.commanded_positions = vec![0.2, -0.2];
            inner.target_positions = vec![0.25, -0.25];
            inner.servo_target = Some(vec![0.25, -0.25]);
            assert!(!inner.supports_automatic_unload(), "missing {missing}");
            inner.apply_stop_policy(armd::safety::StopPolicy::NormalUnload);
            assert_eq!(inner.target_positions, vec![0.25, -0.25]);
            assert_eq!(inner.servo_target, Some(vec![0.25, -0.25]));
            assert!(!inner.pending_normal_unload && inner.active_trajectory.is_none());
        }
        let (mut ws, generation) = controller(addr).await;
        let response = rpc(&mut ws, generation, 1, "/api/v1/motion/unload", serde_json::json!({})).await;
        assert_eq!(response["type"], "command_reject");
        assert_eq!(response["code"], "UNSUPPORTED", "missing {missing}");
        assert_eq!(response["status"], 422);
        for name in ["park", "unload"] {
            let response = ws_reply(&mut ws, serde_json::json!({"type":"preset", "name":name, "generation":generation, "epoch_id":1}), &["preset_reject"]).await;
            assert_eq!(response["code"], "UNSUPPORTED");
        }
        {
            let inner = supervisor.inner().read().await;
            assert!(!inner.pending_normal_unload && inner.active_trajectory.is_none());
            assert_eq!(inner.commanded_positions, vec![0.2, -0.2]);
            assert_eq!(inner.target_positions, vec![0.25, -0.25]);
            assert_eq!(inner.servo_target, Some(vec![0.25, -0.25]));
        }
        assert_eq!(rpc(&mut ws, generation, 1, "/api/v1/motion/home", serde_json::json!({})).await["type"], "command_ack");
        assert!(supervisor.has_active_trajectory().await);
    }

    let (_, supervisor) = spawn_test_server().await;
    assert!(supervisor.inner().read().await.supports_automatic_unload(), "Mock remains supported without a physical resting profile");
    supervisor.set_driver(Box::new(UnloadCapabilityDriver { capabilities: full })).await;
    let mut inner = supervisor.inner().write().await;
    inner.config.resting_pose = Some(armd::model::RestingPosePolicy {
        position_tolerances: vec![0.08; 2], max_total_deviation: 0.16, max_velocity: 0.015, joint_max_velocities: vec![],
        support_torque_limits: vec![(1, 1.0)], feedback_max_age_ms: 100,
    });
    inner.safety.set_torque(true).unwrap();
    inner.is_torqued = true;
    inner.measured_positions = vec![0.5, -0.5];
    inner.commanded_positions = vec![0.5, -0.5];
    assert!(inner.supports_automatic_unload(), "Capability is independent of current pose");
    inner.apply_stop_policy(armd::safety::StopPolicy::NormalUnload);
    assert!(inner.pending_normal_unload && inner.active_trajectory.is_some());
}

#[tokio::test]
async fn websocket_business_rejections_preserve_status_code_and_message() {
    let (addr, _) = spawn_test_server().await;
    let (mut ws, generation) = controller(addr).await;
    for (path, body, message) in [
        ("/api/v1/torque", serde_json::json!({"enable":"yes"}), "enable must be boolean"),
        ("/api/v1/motion/jog", serde_json::json!({"target_positions":true}), "Malformed positions array"),
        ("/api/v1/motion/home", serde_json::json!({"duration_secs":"slow"}), "duration_secs must be numeric"),
        ("/api/v1/not_a_command", serde_json::json!({}), "Unknown actuator command"),
    ] {
        let response = rpc(&mut ws, generation, 1, path, body).await;
        assert_eq!(response["type"], "command_reject");
        assert_eq!(response["status"], 400);
        assert_eq!(response["code"], "INVALID_COMMAND");
        assert_eq!(response["message"], message);
    }
    let not_armed = rpc(&mut ws, generation, 1, "/api/v1/motion/home", serde_json::json!({})).await;
    assert_eq!(not_armed["status"], 412);
    assert_eq!(not_armed["code"], "NOT_ARMED");
}

#[tokio::test]
async fn test_jmuxer_asset_served_as_javascript() {
    let (addr, _) = spawn_test_server().await;
    let res = reqwest::get(format!("http://{}/assets/jmuxer.min.js", addr)).await.unwrap();
    assert_eq!(res.status(), reqwest::StatusCode::OK);
    let ct = res.headers().get(reqwest::header::CONTENT_TYPE).unwrap().to_str().unwrap().to_string();
    assert!(ct.starts_with("application/javascript"), "content-type: {ct}");
    let cc = res.headers().get(reqwest::header::CACHE_CONTROL).unwrap().to_str().unwrap().to_string();
    assert!(cc.contains("max-age="), "cache-control: {cc}");
    let body = res.text().await.unwrap();
    assert!(body.contains("JMuxer 2.1.0"));
    assert!(body.contains("global.JMuxer = factory"));
}
