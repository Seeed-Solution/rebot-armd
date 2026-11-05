use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
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
    let (out_tx, mut out_rx) = tokio::sync::mpsc::channel::<Message>(64);

    let connection_client_id = Arc::new(Mutex::new(String::from("ws_client")));
    let connection_lease_gen = Arc::new(AtomicU64::new(0));

    // Task for streaming observations and outgoing messages to client
    let mut send_task = tokio::spawn(async move {
        loop {
            tokio::select! {
                Some(msg) = out_rx.recv() => {
                    if sender.send(msg).await.is_err() {
                        break;
                    }
                }
                res = telemetry_rx.recv() => {
                    match res {
                        Ok(obs) => {
                            match serde_json::to_string(&obs) {
                                Ok(json_str) => {
                                    if sender.send(Message::Text(json_str.into())).await.is_err() {
                                        break;
                                    }
                                }
                                Err(e) => {
                                    warn!("Failed to serialize ArmObservation: {}", e);
                                }
                            }
                        }
                        Err(tokio::sync::broadcast::error::RecvError::Lagged(skipped)) => {
                            debug!("Telemetry receiver lagged, skipped {} frames", skipped);
                            continue;
                        }
                        Err(tokio::sync::broadcast::error::RecvError::Closed) => {
                            break;
                        }
                    }
                }
                else => break,
            }
        }
    });

    // Task for reading incoming client messages (e.g. ping/pong, heartbeat, lease, servo)
    let supervisor_recv = supervisor.clone();
    let out_tx_recv = out_tx.clone();
    let conn_client_id_recv = connection_client_id.clone();
    let conn_lease_gen_recv = connection_lease_gen.clone();
    let mut recv_task = tokio::spawn(async move {
        let mut conn_client_id: Option<String> = None;
        let mut conn_last_sequence_id: Option<u64> = None;

        while let Some(Ok(msg)) = receiver.next().await {
            match msg {
                Message::Close(_) => break,
                Message::Ping(_) => {
                    // Axum automatically handles Ping/Pong under the hood
                }
                Message::Text(text) => {
                    if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                        let msg_type = val.get("type").and_then(|t| t.as_str());
                        let mode = val.get("mode").and_then(|m| m.as_str());

                        if msg_type == Some("acquire_lease") {
                            let client_id = val
                                .get("client_id")
                                .and_then(|c| c.as_str())
                                .unwrap_or("ws_client");
                            match supervisor_recv.acquire_lease(client_id).await {
                                Ok(gen) => {
                                    conn_client_id = Some(client_id.to_string());
                                    if let Ok(mut cid) = conn_client_id_recv.lock() {
                                        *cid = client_id.to_string();
                                    }
                                    conn_lease_gen_recv.store(gen, Ordering::SeqCst);
                                    conn_last_sequence_id = None;
                                    let grant_msg = serde_json::json!({
                                        "type": "lease_granted",
                                        "client_id": client_id,
                                        "generation": gen,
                                    });
                                    let _ = out_tx_recv.try_send(Message::Text(grant_msg.to_string().into()));
                                }
                                Err((owner, owner_gen)) => {
                                    let rej_msg = serde_json::json!({
                                        "type": "lease_rejected",
                                        "client_id": client_id,
                                        "owner": owner,
                                        "generation": owner_gen,
                                    });
                                    let _ = out_tx_recv.try_send(Message::Text(rej_msg.to_string().into()));
                                }
                            }
                        } else if msg_type == Some("release_lease") {
                            let client_id = val.get("client_id").and_then(|c| c.as_str()).unwrap_or("");
                            let gen = val.get("generation").and_then(|g| g.as_u64()).unwrap_or(0);
                            let released = supervisor_recv.release_lease(client_id, gen).await;
                            if released && conn_lease_gen_recv.load(Ordering::SeqCst) == gen {
                                conn_lease_gen_recv.store(0, Ordering::SeqCst);
                                conn_client_id = None;
                            }
                            let rel_msg = serde_json::json!({
                                "type": "lease_released",
                                "client_id": client_id,
                                "generation": gen,
                                "success": released,
                            });
                            let _ = out_tx_recv.try_send(Message::Text(rel_msg.to_string().into()));
                        } else if msg_type == Some("e_stop") || msg_type == Some("estop") {
                            let (safety_state, epoch_id) = supervisor_recv.trigger_e_stop().await;
                            let sync_msg = serde_json::json!({
                                "type": "epoch_sync",
                                "epoch_id": epoch_id,
                                "safety_state": safety_state,
                            });
                            let _ = out_tx_recv.try_send(Message::Text(sync_msg.to_string().into()));
                        } else if msg_type == Some("heartbeat") {
                            let client_id = val
                                .get("client_id")
                                .and_then(|c| c.as_str())
                                .or(conn_client_id.as_deref())
                                .unwrap_or("ws_client");
                            let epoch_id = val.get("epoch_id").and_then(|e| e.as_u64());
                            if let Some(epoch) = epoch_id {
                                if let Err(e) = supervisor_recv.heartbeat(client_id, epoch).await {
                                    debug!("WebSocket heartbeat rejected: {}", e);
                                    let obs = supervisor_recv.get_observation().await;
                                    let sync_msg = serde_json::json!({
                                        "type": "epoch_sync",
                                        "epoch_id": obs.epoch_id,
                                        "safety_state": obs.safety_state,
                                    });
                                    let _ = out_tx_recv.try_send(Message::Text(sync_msg.to_string().into()));
                                }
                            } else {
                                debug!("WebSocket heartbeat: missing epoch_id");
                            }
                        } else if msg_type == Some("preset") {
                            let client_id = val
                                .get("client_id")
                                .and_then(|c| c.as_str())
                                .or(conn_client_id.as_deref())
                                .unwrap_or("");
                            let gen = val
                                .get("generation")
                                .and_then(|g| g.as_u64())
                                .unwrap_or_else(|| conn_lease_gen_recv.load(Ordering::SeqCst));

                            if client_id.is_empty() || !supervisor_recv.validate_servo_lease(client_id, gen).await {
                                warn!("WebSocket preset rejected: lease lost or generation mismatch (gen={})", gen);
                                let lost = serde_json::json!({
                                    "type": "lease_lost"
                                });
                                let _ = out_tx_recv.try_send(Message::Text(lost.to_string().into()));
                                continue;
                            }

                            let epoch_id = val.get("epoch_id").and_then(|e| e.as_u64());
                            let obs = supervisor_recv.get_observation().await;
                            if epoch_id != Some(obs.epoch_id) {
                                let sync_msg = serde_json::json!({
                                    "type": "epoch_sync",
                                    "epoch_id": obs.epoch_id,
                                    "safety_state": obs.safety_state,
                                });
                                let _ = out_tx_recv.try_send(Message::Text(sync_msg.to_string().into()));
                                continue;
                            }
                            let target_epoch = epoch_id;

                            let name = val.get("name").and_then(|n| n.as_str()).unwrap_or("");
                            match name {
                                "home" => {
                                    match supervisor_recv.home(None, target_epoch).await {
                                        Ok(epoch) => {
                                            let ack = serde_json::json!({
                                                "type": "preset_ack",
                                                "name": name,
                                                "epoch_id": epoch,
                                            });
                                            let _ = out_tx_recv.try_send(Message::Text(ack.to_string().into()));
                                        }
                                        Err(e) => {
                                            let rej = serde_json::json!({
                                                "type": "preset_reject",
                                                "name": name,
                                                "error": e.to_string(),
                                                "epoch_id": obs.epoch_id,
                                            });
                                            let _ = out_tx_recv.try_send(Message::Text(rej.to_string().into()));
                                        }
                                    }
                                }
                                "park" | "unload" => {
                                    if obs.safety_state.is_stopped() {
                                        let rej = serde_json::json!({
                                            "type": "preset_reject",
                                            "name": name,
                                            "error": "Cannot execute preset while stopped or in fault state",
                                            "epoch_id": obs.epoch_id,
                                        });
                                        let _ = out_tx_recv.try_send(Message::Text(rej.to_string().into()));
                                    } else {
                                        supervisor_recv.apply_stop_policy(crate::safety::StopPolicy::NormalUnload).await;
                                        let ack = serde_json::json!({
                                            "type": "preset_ack",
                                            "name": name,
                                            "epoch_id": obs.epoch_id,
                                        });
                                        let _ = out_tx_recv.try_send(Message::Text(ack.to_string().into()));
                                    }
                                }
                                _ => {
                                    let maybe_targets: Option<Vec<f32>> = val
                                        .get("positions")
                                        .and_then(|p| p.as_array())
                                        .and_then(|arr| {
                                            arr.iter()
                                                .map(|v| v.as_f64().filter(|f| f.is_finite()).map(|f| f as f32))
                                                .collect()
                                        });
                                    match maybe_targets {
                                        Some(targets) if !targets.is_empty() => {
                                            match supervisor_recv.jog(targets, None, target_epoch).await {
                                                Ok(epoch) => {
                                                    let ack = serde_json::json!({
                                                        "type": "preset_ack",
                                                        "name": name,
                                                        "epoch_id": epoch,
                                                    });
                                                    let _ = out_tx_recv.try_send(Message::Text(ack.to_string().into()));
                                                }
                                                Err(e) => {
                                                    let rej = serde_json::json!({
                                                        "type": "preset_reject",
                                                        "name": name,
                                                        "error": e.to_string(),
                                                        "epoch_id": obs.epoch_id,
                                                    });
                                                    let _ = out_tx_recv.try_send(Message::Text(rej.to_string().into()));
                                                }
                                            }
                                        }
                                        _ => {
                                            let rej = serde_json::json!({
                                                "type": "preset_reject",
                                                "name": name,
                                                "error": "Missing or malformed positions for preset",
                                                "epoch_id": obs.epoch_id,
                                            });
                                            let _ = out_tx_recv.try_send(Message::Text(rej.to_string().into()));
                                        }
                                    }
                                }
                            }
                        } else if mode == Some("cartesian_joint_stream") || msg_type == Some("servo") || val.get("positions").is_some() {
                            // 1. Generation validation: generation is strictly required
                            let gen = match val.get("generation").and_then(|g| g.as_u64()) {
                                Some(g) => g,
                                None => {
                                    warn!("WebSocket servo frame rejected: missing generation");
                                    let rej = serde_json::json!({
                                        "type": "servo_reject",
                                        "reason": "missing_generation"
                                    });
                                    let _ = out_tx_recv.try_send(Message::Text(rej.to_string().into()));
                                    continue;
                                }
                            };

                            let client_id = val
                                .get("client_id")
                                .and_then(|c| c.as_str())
                                .or(conn_client_id.as_deref())
                                .unwrap_or("");
                            if client_id.is_empty() || !supervisor_recv.validate_servo_lease(client_id, gen).await {
                                warn!("WebSocket servo frame rejected: lease lost or generation mismatch (gen={})", gen);
                                let lost = serde_json::json!({
                                    "type": "lease_lost"
                                });
                                let _ = out_tx_recv.try_send(Message::Text(lost.to_string().into()));
                                continue;
                            }

                            // 2. Monotonic sequence_id validation with wrapping safety window
                            let seq = match val.get("sequence_id").and_then(|s| s.as_u64()) {
                                Some(s) => s,
                                None => {
                                    warn!("WebSocket servo frame rejected: missing sequence_id");
                                    let rej = serde_json::json!({
                                        "type": "servo_reject",
                                        "reason": "missing_sequence_id"
                                    });
                                    let _ = out_tx_recv.try_send(Message::Text(rej.to_string().into()));
                                    continue;
                                }
                            };

                            if let Some(last_seq) = conn_last_sequence_id {
                                let delta = seq.wrapping_sub(last_seq);
                                if delta == 0 || delta > 1000 {
                                    debug!("WebSocket servo frame rejected: invalid sequence_id (seq={}, last_seq={}, delta={})", seq, last_seq, delta);
                                    let rej = serde_json::json!({
                                        "type": "servo_reject",
                                        "reason": "invalid_sequence_id",
                                        "sequence_id": seq,
                                        "last_sequence_id": last_seq,
                                        "delta": delta
                                    });
                                    let _ = out_tx_recv.try_send(Message::Text(rej.to_string().into()));
                                    continue;
                                }
                            }
                            conn_last_sequence_id = Some(seq);

                            // 3. Epoch verification: must match current epoch_id, otherwise send epoch_sync
                            let epoch_id = val.get("epoch_id").and_then(|e| e.as_u64());
                            let obs = supervisor_recv.get_observation().await;
                            match epoch_id {
                                Some(epoch) if epoch == obs.epoch_id => {
                                    if let Some(pos_val) = val.get("positions").and_then(|p| p.as_array()) {
                                        let maybe_targets: Option<Vec<f32>> = pos_val
                                            .iter()
                                            .map(|v| v.as_f64().filter(|f| f.is_finite()).map(|f| f as f32))
                                            .collect();
                                        match maybe_targets {
                                            Some(targets) if !targets.is_empty() => {
                                                if let Err(e) = supervisor_recv.servo(targets, epoch).await {
                                                    debug!("WebSocket servo command rejected: {}", e);
                                                    let rej = serde_json::json!({
                                                        "type": "servo_reject",
                                                        "error": e.to_string(),
                                                        "epoch_id": obs.epoch_id
                                                    });
                                                    let _ = out_tx_recv.try_send(Message::Text(rej.to_string().into()));
                                                }
                                            }
                                            _ => {
                                                debug!("WebSocket servo command contains invalid/non-numeric positions");
                                                let rej = serde_json::json!({
                                                    "type": "servo_reject",
                                                    "error": "Malformed positions array: non-numeric or non-finite value",
                                                    "epoch_id": obs.epoch_id
                                                });
                                                let _ = out_tx_recv.try_send(Message::Text(rej.to_string().into()));
                                            }
                                        }
                                    }
                                }
                                Some(_) => {
                                    // Epoch mismatch: notify client of current state
                                    let sync_msg = serde_json::json!({
                                        "type": "epoch_sync",
                                        "epoch_id": obs.epoch_id,
                                        "safety_state": obs.safety_state,
                                    });
                                    let _ = out_tx_recv.try_send(Message::Text(sync_msg.to_string().into()));
                                }
                                None => {
                                    debug!("WebSocket servo: missing epoch_id");
                                }
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

    let gen = connection_lease_gen.load(Ordering::SeqCst);
    if gen > 0 {
        let client_id = connection_client_id.lock().map(|cid| cid.clone()).unwrap_or_else(|_| "ws_client".to_string());
        supervisor.release_lease(&client_id, gen).await;
        debug!("WebSocket connection termination: released lease for client={}, generation={}", client_id, gen);
    }
    debug!("WebSocket client session ended");
}
