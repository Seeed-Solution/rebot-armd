use std::sync::LazyLock;
use axum::{
    body::Body,
    extract::{
        ws::{CloseFrame, Message as WsMessage, WebSocket, WebSocketUpgrade},
        Request,
    },
    http::{header, HeaderMap, StatusCode},
    response::{IntoResponse, Response},
};
use futures_util::{SinkExt, StreamExt};
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::protocol::CloseFrame as TungCloseFrame;
use tokio_tungstenite::tungstenite::Message as TungMessage;

static HTTP_CLIENT: LazyLock<reqwest::Client> = LazyLock::new(|| {
    reqwest::Client::builder()
        .connect_timeout(std::time::Duration::from_secs(3))
        .build()
        .unwrap_or_default()
});

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

/// GET /api/ws (go2rtc MSE/WebRTC WebSocket stream on Port 1984)
pub async fn proxy_go2rtc_ws(ws: WebSocketUpgrade, req: Request) -> Response {
    let query = req.uri().query().unwrap_or("");
    let upstream_url = if query.is_empty() {
        "ws://127.0.0.1:1984/api/ws".to_string()
    } else {
        format!("ws://127.0.0.1:1984/api/ws?{}", query)
    };
    bridge_websocket(ws, upstream_url).await
}

/// POST /api/webrtc (WHEP WebRTC SDP handshake to go2rtc on Port 1984)
pub async fn proxy_go2rtc_webrtc(req: Request) -> impl IntoResponse {
    let uri = req.uri();
    let query = uri.query().map_or("".to_string(), |q| format!("?{}", q));
    let upstream_url = format!("http://127.0.0.1:1984/api/webrtc{}", query);

    let (parts, body) = req.into_parts();
    let body_bytes = match axum::body::to_bytes(body, 1024 * 1024).await {
        Ok(b) => b,
        Err(e) => {
            return (
                StatusCode::BAD_REQUEST,
                format!("Failed to read request body: {}", e),
            )
                .into_response()
        }
    };

    let mut req_builder = HTTP_CLIENT.post(&upstream_url).body(body_bytes);
    for (k, v) in &parts.headers {
        if k != header::TRANSFER_ENCODING && k != header::CONNECTION && k != header::HOST {
            req_builder = req_builder.header(k, v);
        }
    }

    match req_builder.send().await {
        Ok(resp) => {
            let status = StatusCode::from_u16(resp.status().as_u16()).unwrap_or(StatusCode::OK);
            let mut res_headers = HeaderMap::new();
            for (key, val) in resp.headers() {
                if key != header::TRANSFER_ENCODING && key != header::CONNECTION {
                    res_headers.insert(key.clone(), val.clone());
                }
            }
            match resp.bytes().await {
                Ok(bytes) => (status, res_headers, bytes).into_response(),
                Err(e) => (
                    StatusCode::BAD_GATEWAY,
                    format!("Failed to read response from go2rtc: {}", e),
                )
                    .into_response(),
            }
        }
        Err(e) => (
            StatusCode::BAD_GATEWAY,
            format!("go2rtc service unavailable: {}", e),
        )
            .into_response(),
    }
}

/// GET /api/stream.mp4 and /api/stream.mjpeg (Streaming video from go2rtc on Port 1984)
pub async fn proxy_go2rtc_stream(req: Request) -> impl IntoResponse {
    let uri = req.uri();
    let path = uri.path();
    let query = uri.query().map_or("".to_string(), |q| format!("?{}", q));
    let upstream_url = format!("http://127.0.0.1:1984{}{}", path, query);

    match HTTP_CLIENT.get(&upstream_url).send().await {
        Ok(resp) => {
            let status = StatusCode::from_u16(resp.status().as_u16()).unwrap_or(StatusCode::OK);
            let mut res_headers = HeaderMap::new();
            for (key, val) in resp.headers() {
                if key != header::TRANSFER_ENCODING && key != header::CONNECTION {
                    res_headers.insert(key.clone(), val.clone());
                }
            }
            // Use streaming Body::from_stream to guarantee zero memory buffering on SG2002
            let stream = resp.bytes_stream();
            let body = Body::from_stream(stream);
            (status, res_headers, body).into_response()
        }
        Err(e) => (
            StatusCode::BAD_GATEWAY,
            format!("go2rtc stream unavailable: {}", e),
        )
            .into_response(),
    }
}
