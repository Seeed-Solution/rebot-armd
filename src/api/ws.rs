use std::sync::Arc;
use axum::{
    extract::{
        ws::{Message, WebSocket, WebSocketUpgrade},
        State,
    },
    response::IntoResponse,
};
use futures_util::{SinkExt, StreamExt};
use tracing::{debug, warn};
use crate::supervisor::ArmSupervisor;

/// GET /api/v1/ws - WebSocket telemetry stream
pub async fn ws_handler(
    ws: WebSocketUpgrade,
    State(supervisor): State<Arc<ArmSupervisor>>,
) -> impl IntoResponse {
    ws.on_upgrade(move |socket| handle_socket(socket, supervisor))
}

async fn handle_socket(socket: WebSocket, supervisor: Arc<ArmSupervisor>) {
    let (mut sender, mut receiver) = socket.split();
    let mut telemetry_rx = supervisor.subscribe_telemetry();

    // Task for streaming observations to client
    let mut send_task = tokio::spawn(async move {
        while let Ok(obs) = telemetry_rx.recv().await {
            match serde_json::to_string(&obs) {
                Ok(json_str) => {
                    if sender.send(Message::Text(json_str.into())).await.is_err() {
                        // Client disconnected
                        break;
                    }
                }
                Err(e) => {
                    warn!("Failed to serialize ArmObservation: {}", e);
                }
            }
        }
    });

    // Task for reading incoming client messages (e.g. ping/pong, heartbeat, close)
    let supervisor_recv = supervisor.clone();
    let mut recv_task = tokio::spawn(async move {
        while let Some(Ok(msg)) = receiver.next().await {
            match msg {
                Message::Close(_) => break,
                Message::Ping(_) => {
                    // Axum automatically handles Ping/Pong under the hood
                }
                Message::Text(text) => {
                    if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                        if val.get("type").and_then(|t| t.as_str()) == Some("heartbeat") {
                            let client_id = val
                                .get("client_id")
                                .and_then(|c| c.as_str())
                                .unwrap_or("ws_client");
                            let epoch_id = val.get("epoch_id").and_then(|e| e.as_u64());
                            if let Some(epoch) = epoch_id {
                                if let Err(e) = supervisor_recv.heartbeat(client_id, epoch).await {
                                    warn!("WebSocket heartbeat rejected: {}", e);
                                }
                            } else {
                                warn!("WebSocket heartbeat rejected: missing epoch_id");
                            }
                        }
                    }
                }
                _ => {}
            }
        }
    });

    // Wait until either sending or receiving terminates
    tokio::select! {
        _ = (&mut send_task) => {
            recv_task.abort();
        }
        _ = (&mut recv_task) => {
            send_task.abort();
        }
    }
    debug!("WebSocket client session ended");
}
