//! Shared WS helpers for the integration tests (same pattern as tests/test_api_and_mock.rs).
#![allow(dead_code)]
use std::net::SocketAddr;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;
use futures_util::{SinkExt, StreamExt};
use tokio::net::TcpListener;
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::Message;

use armd::{create_router, ArmConfig, ArmSupervisor};

pub type TestSocket = tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>>;

pub async fn spawn_server(config: ArmConfig) -> (SocketAddr, Arc<ArmSupervisor>) {
    let supervisor = Arc::new(ArmSupervisor::new(config));
    let router = create_router(supervisor.clone());
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });
    (addr, supervisor)
}

pub async fn send(ws: &mut TestSocket, message: serde_json::Value) {
    ws.send(Message::Text(message.to_string().into())).await.unwrap();
}

/// Next message whose `type` is one of `expected` (telemetry is skipped).
pub async fn next_of(ws: &mut TestSocket, expected: &[&str]) -> serde_json::Value {
    tokio::time::timeout(Duration::from_secs(3), async {
        loop {
            let text = ws.next().await.unwrap().unwrap().into_text().unwrap();
            let value: serde_json::Value = serde_json::from_str(&text).unwrap();
            if expected.iter().any(|kind| value["type"] == *kind) { return value; }
        }
    }).await.expect("Timed out waiting for control response")
}

pub async fn ws_reply(ws: &mut TestSocket, message: serde_json::Value, expected: &[&str]) -> serde_json::Value {
    send(ws, message).await;
    next_of(ws, expected).await
}

pub async fn controller(addr: SocketAddr) -> (TestSocket, u64) {
    let (mut ws, _) = connect_async(format!("ws://{}/api/v1/ws", addr)).await.unwrap();
    let reply = ws_reply(&mut ws, serde_json::json!({"type":"acquire_lease", "client_id":"test_controller"}), &["lease_granted"]).await;
    (ws, reply["generation"].as_u64().unwrap())
}

static REQUEST: AtomicU64 = AtomicU64::new(1);
static SEQUENCE: AtomicU64 = AtomicU64::new(1);

pub async fn rpc(ws: &mut TestSocket, generation: u64, epoch: u64, path: &str, body: serde_json::Value) -> serde_json::Value {
    ws_reply(ws, serde_json::json!({"type":"command", "request_id":REQUEST.fetch_add(1, Ordering::Relaxed), "generation":generation, "epoch_id":epoch, "path":path, "body":body}), &["command_ack", "command_reject"]).await
}

pub async fn preset(ws: &mut TestSocket, generation: u64, epoch: u64, name: &str, positions: Option<Vec<f32>>) -> serde_json::Value {
    ws_reply(ws, serde_json::json!({"type":"preset", "generation":generation, "epoch_id":epoch, "name":name, "positions":positions}), &["preset_ack", "preset_reject"]).await
}

/// Sends one servo frame; servo frames are acknowledged only on rejection.
pub async fn servo(ws: &mut TestSocket, generation: u64, epoch: u64, positions: &[f32]) {
    send(ws, serde_json::json!({"type":"servo", "generation":generation, "epoch_id":epoch,
        "sequence_id":SEQUENCE.fetch_add(1, Ordering::Relaxed), "positions":positions})).await;
}

/// Collects messages for `ms` and returns the first `servo_reject`, if any.
pub async fn servo_reject_within(ws: &mut TestSocket, ms: u64) -> Option<serde_json::Value> {
    tokio::time::timeout(Duration::from_millis(ms), next_of(ws, &["servo_reject"])).await.ok()
}

/// Reads and drops whatever is already queued (telemetry), waiting at most `ms` per message.
pub async fn drain(ws: &mut TestSocket, ms: u64) {
    while let Ok(Some(Ok(_))) = tokio::time::timeout(Duration::from_millis(ms), ws.next()).await {}
}

/// Server with a 50 Hz ticker, a lease holder and torque on, commanded/measured at `q`.
/// Returns (socket, generation, epoch, supervisor, addr).
pub async fn torqued_rig(supervisor: Arc<ArmSupervisor>, q: &[f32]) -> (TestSocket, u64, u64, Arc<ArmSupervisor>, SocketAddr) {
    let router = create_router(supervisor.clone());
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });
    let ticker = supervisor.clone();
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(20));
        loop { interval.tick().await; ticker.tick(0.02).await; }
    });
    let (mut ws, generation) = controller(addr).await;
    let epoch = supervisor.get_observation().await.epoch_id;
    let ack = rpc(&mut ws, generation, epoch, "/api/v1/torque", serde_json::json!({"enable": true})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    {
        let mut inner = supervisor.inner().write().await;
        inner.commanded_positions = q.to_vec();
        inner.measured_positions = q.to_vec();
        inner.target_positions = q.to_vec();
    }
    let epoch = supervisor.get_observation().await.epoch_id;
    (ws, generation, epoch, supervisor, addr)
}
