use axum::{
    extract::ws::{CloseFrame, Message as WsMessage, WebSocket, WebSocketUpgrade},
    response::Response,
};
use futures_util::{SinkExt, StreamExt};
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::protocol::CloseFrame as TungCloseFrame;
use tokio_tungstenite::tungstenite::Message as TungMessage;

/// Generic bidirectional WebSocket proxy between Axum client and upstream server
async fn bridge_websocket(client_ws: WebSocketUpgrade, upstream_url: String) -> Response {
    client_ws.on_upgrade(move |mut client_socket: WebSocket| async move {
        let connect_timeout = std::time::Duration::from_secs(3);
        let connect_res = tokio::time::timeout(connect_timeout, connect_async(&upstream_url)).await;
        match connect_res {
            Ok(Ok((upstream_socket, _))) => {
                let (mut client_tx, mut client_rx) = client_socket.split();
                let (mut upstream_tx, mut upstream_rx) = upstream_socket.split();

                let client_to_upstream = async {
                    while let Some(msg_res) = client_rx.next().await {
                        match msg_res {
                            Ok(WsMessage::Text(t)) => {
                                if upstream_tx.send(TungMessage::Text(t.as_str().into())).await.is_err() {
                                    break;
                                }
                            }
                            Ok(WsMessage::Binary(b)) => {
                                if upstream_tx.send(TungMessage::Binary(b.into())).await.is_err() {
                                    break;
                                }
                            }
                            Ok(WsMessage::Ping(p)) => {
                                if upstream_tx.send(TungMessage::Ping(p.into())).await.is_err() {
                                    break;
                                }
                            }
                            Ok(WsMessage::Pong(p)) => {
                                if upstream_tx.send(TungMessage::Pong(p.into())).await.is_err() {
                                    break;
                                }
                            }
                            Ok(WsMessage::Close(c)) => {
                                let _ = upstream_tx
                                    .send(TungMessage::Close(c.map(|f| TungCloseFrame {
                                        code: f.code.into(),
                                        reason: f.reason.as_str().into(),
                                    })))
                                    .await;
                                break;
                            }
                            Err(_) => break,
                        }
                    }
                };

                let upstream_to_client = async {
                    while let Some(msg_res) = upstream_rx.next().await {
                        match msg_res {
                            Ok(TungMessage::Text(t)) => {
                                if client_tx.send(WsMessage::Text(t.as_str().into())).await.is_err() {
                                    break;
                                }
                            }
                            Ok(TungMessage::Binary(b)) => {
                                if client_tx.send(WsMessage::Binary(b.into())).await.is_err() {
                                    break;
                                }
                            }
                            Ok(TungMessage::Ping(p)) => {
                                if client_tx.send(WsMessage::Ping(p.into())).await.is_err() {
                                    break;
                                }
                            }
                            Ok(TungMessage::Pong(p)) => {
                                if client_tx.send(WsMessage::Pong(p.into())).await.is_err() {
                                    break;
                                }
                            }
                            Ok(TungMessage::Close(c)) => {
                                let _ = client_tx
                                    .send(WsMessage::Close(c.map(|f| CloseFrame {
                                        code: f.code.into(),
                                        reason: f.reason.as_str().into(),
                                    })))
                                    .await;
                                break;
                            }
                            Ok(TungMessage::Frame(_)) => continue,
                            Err(_) => break,
                        }
                    }
                };

                tokio::select! {
                    _ = client_to_upstream => {},
                    _ = upstream_to_client => {},
                }
            }
            Err(elapsed) => {
                tracing::debug!("Timeout connecting to upstream {}: {}", upstream_url, elapsed);
                let _ = client_socket
                    .send(WsMessage::Close(Some(CloseFrame {
                        code: 1001,
                        reason: "upstream connection timed out".into(),
                    })))
                    .await;
            }
            Ok(Err(e)) => {
                tracing::debug!("Failed to connect to upstream {}: {}", upstream_url, e);
                // Send standard graceful close frame (1001 Going Away) to avoid console errors
                let _ = client_socket
                    .send(WsMessage::Close(Some(CloseFrame {
                        code: 1001,
                        reason: "upstream unavailable".into(),
                    })))
                    .await;
            }
        }
    })
}

/// GET /api/v1/detection/ws and /results (YOLO detection WebSocket stream on Port 8001)
pub async fn proxy_detection_ws(ws: WebSocketUpgrade) -> Response {
    bridge_websocket(ws, "ws://127.0.0.1:8001/results".to_string()).await
}
