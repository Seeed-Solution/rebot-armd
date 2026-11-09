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
                                if upstream_tx.send(TungMessage::Binary(b)).await.is_err() {
                                    break;
                                }
                            }
                            Ok(WsMessage::Ping(p)) => {
                                if upstream_tx.send(TungMessage::Ping(p)).await.is_err() {
                                    break;
                                }
                            }
                            Ok(WsMessage::Pong(p)) => {
                                if upstream_tx.send(TungMessage::Pong(p)).await.is_err() {
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
                                if client_tx.send(WsMessage::Binary(b)).await.is_err() {
                                    break;
                                }
                            }
                            Ok(TungMessage::Ping(p)) => {
                                if client_tx.send(WsMessage::Ping(p)).await.is_err() {
                                    break;
                                }
                            }
                            Ok(TungMessage::Pong(p)) => {
                                if client_tx.send(WsMessage::Pong(p)).await.is_err() {
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

// ---------------------------------------------------------------------------
// Detector HTTP control API (yolo-detector on 127.0.0.1:8090)
//
// Minimal HTTP/1.1 client over tokio TcpStream: the upstream is a loopback JSON
// service, and hyper's client features are not enabled in the release build.
// ---------------------------------------------------------------------------

use axum::{
    body::Bytes,
    extract::{Extension, Path, RawQuery},
    http::{header, HeaderValue, Method, StatusCode},
    response::IntoResponse,
};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tokio::io::{AsyncReadExt, AsyncWriteExt};

/// Default detector control API base URL. Override with `ARMD_DETECTOR_URL`.
pub const DEFAULT_DETECTOR_URL: &str = "http://127.0.0.1:8090";
/// Upstream caps `wait_ms` at this value.
const MAX_WAIT_MS: u64 = 5000;
/// Added on top of the long-poll wait for the upstream round trip.
const TIMEOUT_MARGIN: Duration = Duration::from_secs(2);
const MAX_RESPONSE_BYTES: usize = 1 << 20;

/// Upstream detector control API location (`http://host[:port][/base]`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DetectorUpstream {
    host: String,
    port: u16,
    base_path: String,
}

impl DetectorUpstream {
    pub fn parse(url: &str) -> Result<Self, String> {
        let rest = url
            .strip_prefix("http://")
            .ok_or_else(|| format!("detector URL must start with http://: {url}"))?;
        let (authority, path) = match rest.find('/') {
            Some(i) => (&rest[..i], rest[i..].trim_end_matches('/')),
            None => (rest, ""),
        };
        let (host, port) = match authority.rsplit_once(':') {
            Some((h, p)) => (h, p.parse::<u16>().map_err(|_| format!("invalid port in detector URL: {url}"))?),
            None => (authority, 80),
        };
        if host.is_empty() || host.contains(|c: char| c.is_whitespace() || c.is_control()) {
            return Err(format!("invalid host in detector URL: {url}"));
        }
        Ok(Self { host: host.to_string(), port, base_path: path.to_string() })
    }

    /// `ARMD_DETECTOR_URL` if set and valid, else [`DEFAULT_DETECTOR_URL`].
    pub fn from_env() -> Self {
        match std::env::var("ARMD_DETECTOR_URL") {
            Ok(url) => Self::parse(&url).unwrap_or_else(|e| {
                tracing::warn!("{e}; using {DEFAULT_DETECTOR_URL}");
                Self::parse(DEFAULT_DETECTOR_URL).unwrap()
            }),
            Err(_) => Self::parse(DEFAULT_DETECTOR_URL).unwrap(),
        }
    }
}

struct UpstreamResponse {
    status: StatusCode,
    content_type: Option<HeaderValue>,
    body: Vec<u8>,
}

/// Raw HTTP/1.1 request bytes. A request with a body always carries `Content-Type: application/json`
/// (the detector rejects other content types on POST /api/trigger and /api/inference).
fn build_request(up: &DetectorUpstream, method: &Method, path_and_query: &str, body: Option<&[u8]>) -> Vec<u8> {
    let mut req = format!(
        "{method} {}{path_and_query} HTTP/1.1\r\nHost: {}:{}\r\nAccept: application/json\r\nConnection: close\r\n",
        up.base_path, up.host, up.port
    );
    if let Some(b) = body {
        req.push_str(&format!("Content-Type: application/json\r\nContent-Length: {}\r\n", b.len()));
    }
    req.push_str("\r\n");
    let mut out = req.into_bytes();
    if let Some(b) = body {
        out.extend_from_slice(b);
    }
    out
}

/// Empty or whitespace-only client bodies become `{}` so the upstream always gets valid JSON.
fn json_body(body: Bytes) -> Bytes {
    if body.iter().all(|b| b.is_ascii_whitespace()) { Bytes::from_static(b"{}") } else { body }
}

async fn upstream_request(
    up: &DetectorUpstream,
    method: Method,
    path_and_query: &str,
    body: Option<&[u8]>,
) -> Result<UpstreamResponse, String> {
    let mut stream = tokio::net::TcpStream::connect((up.host.as_str(), up.port))
        .await
        .map_err(|e| format!("connect: {e}"))?;
    let out = build_request(up, &method, path_and_query, body);
    stream.write_all(&out).await.map_err(|e| format!("write: {e}"))?;

    let mut raw = Vec::new();
    let mut buf = [0u8; 8192];
    loop {
        let n = stream.read(&mut buf).await.map_err(|e| format!("read: {e}"))?;
        if n == 0 {
            break;
        }
        raw.extend_from_slice(&buf[..n]);
        if raw.len() > MAX_RESPONSE_BYTES {
            return Err("response too large".into());
        }
        // Stop once a Content-Length body is complete, in case the upstream keeps the socket open.
        if let Some(parsed) = parse_response(&raw, false) {
            return Ok(parsed);
        }
    }
    parse_response(&raw, true).ok_or_else(|| "malformed upstream response".into())
}

/// Parses a complete response. With `eof == false` it returns `Some` only when the body
/// length is known (Content-Length or a terminated chunked body) and fully received.
fn parse_response(raw: &[u8], eof: bool) -> Option<UpstreamResponse> {
    let head_end = raw.windows(4).position(|w| w == b"\r\n\r\n")?;
    let head = std::str::from_utf8(&raw[..head_end]).ok()?;
    let rest = &raw[head_end + 4..];
    let mut lines = head.split("\r\n");
    let status_line = lines.next()?;
    let mut parts = status_line.splitn(3, ' ');
    if !parts.next()?.starts_with("HTTP/1.") {
        return None;
    }
    let status = StatusCode::from_u16(parts.next()?.parse().ok()?).ok()?;
    let (mut content_length, mut chunked, mut content_type) = (None, false, None);
    for line in lines {
        let Some((k, v)) = line.split_once(':') else { continue };
        let v = v.trim();
        if k.eq_ignore_ascii_case("content-length") {
            content_length = v.parse::<usize>().ok();
        } else if k.eq_ignore_ascii_case("transfer-encoding") {
            chunked = v.to_ascii_lowercase().contains("chunked");
        } else if k.eq_ignore_ascii_case("content-type") {
            content_type = HeaderValue::from_str(v).ok();
        }
    }
    let body = if chunked {
        decode_chunked(rest)?
    } else if let Some(len) = content_length {
        if rest.len() < len {
            return None;
        }
        rest[..len].to_vec()
    } else if eof {
        rest.to_vec()
    } else {
        return None;
    };
    Some(UpstreamResponse { status, content_type, body })
}

/// Decodes a chunked body; `None` until the terminating zero-size chunk has arrived.
fn decode_chunked(mut data: &[u8]) -> Option<Vec<u8>> {
    let mut out = Vec::new();
    loop {
        let line_end = data.windows(2).position(|w| w == b"\r\n")?;
        let size_str = std::str::from_utf8(&data[..line_end]).ok()?;
        let size = usize::from_str_radix(size_str.split(';').next()?.trim(), 16).ok()?;
        data = &data[line_end + 2..];
        if size == 0 {
            return Some(out);
        }
        if data.len() < size + 2 {
            return None;
        }
        out.extend_from_slice(&data[..size]);
        data = &data[size + 2..];
    }
}

/// One upstream exchange with a deadline; `None` (logged) when unreachable or timed out.
async fn fetch(
    up: &DetectorUpstream,
    method: Method,
    path_and_query: &str,
    body: Option<&[u8]>,
    timeout: Duration,
) -> Option<UpstreamResponse> {
    match tokio::time::timeout(timeout, upstream_request(up, method, path_and_query, body)).await {
        Ok(Ok(r)) => Some(r),
        Ok(Err(e)) => {
            tracing::debug!("detector upstream {path_and_query} failed: {e}");
            None
        }
        Err(_) => {
            tracing::debug!("detector upstream {path_and_query} timed out after {timeout:?}");
            None
        }
    }
}

fn passthrough(r: UpstreamResponse) -> Response {
    let ct = r.content_type.unwrap_or_else(|| HeaderValue::from_static("application/json"));
    (r.status, [(header::CONTENT_TYPE, ct), (header::CACHE_CONTROL, HeaderValue::from_static("no-store"))], r.body)
        .into_response()
}

async fn forward(
    up: &DetectorUpstream,
    method: Method,
    path_and_query: String,
    body: Option<Bytes>,
    timeout: Duration,
) -> Response {
    match fetch(up, method, &path_and_query, body.as_deref(), timeout).await {
        Some(r) => passthrough(r),
        None => detector_unreachable(),
    }
}

/// One JSON request to the detector for armd's own use (the grasp orchestrator).
/// Non-2xx statuses and non-JSON bodies are errors.
pub(crate) async fn request_json(
    up: &DetectorUpstream,
    method: &str,
    path_and_query: &str,
    body: Option<&[u8]>,
    timeout: Duration,
) -> Result<serde_json::Value, String> {
    let method = Method::from_bytes(method.as_bytes()).map_err(|e| e.to_string())?;
    let r = tokio::time::timeout(timeout, upstream_request(up, method, path_and_query, body))
        .await
        .map_err(|_| "detector timed out".to_string())??;
    if !r.status.is_success() {
        return Err(format!("detector returned {}", r.status));
    }
    serde_json::from_slice(&r.body).map_err(|e| format!("detector returned invalid JSON: {e}"))
}

/// As [`request_json`], but a 404 (provider without the route) is `Ok(None)`.
pub(crate) async fn request_json_opt(
    up: &DetectorUpstream,
    method: &str,
    path_and_query: &str,
    body: Option<&[u8]>,
    timeout: Duration,
) -> Result<Option<serde_json::Value>, String> {
    let m = Method::from_bytes(method.as_bytes()).map_err(|e| e.to_string())?;
    let r = tokio::time::timeout(timeout, upstream_request(up, m, path_and_query, body))
        .await
        .map_err(|_| "detector timed out".to_string())??;
    if r.status == StatusCode::NOT_FOUND {
        return Ok(None);
    }
    if !r.status.is_success() {
        let detail = serde_json::from_slice::<serde_json::Value>(&r.body)
            .ok()
            .and_then(|v| v.get("error").and_then(|e| e.as_str()).map(str::to_string))
            .unwrap_or_default();
        return Err(format!("detector returned {} {detail}", r.status).trim_end().to_string());
    }
    serde_json::from_slice(&r.body).map(Some).map_err(|e| format!("detector returned invalid JSON: {e}"))
}

fn detector_unreachable() -> Response {
    (StatusCode::BAD_GATEWAY, axum::Json(serde_json::json!({"error": "detector unreachable"}))).into_response()
}

/// `wait_ms` from a raw query string, clamped to the upstream cap.
fn wait_ms_from_query(query: Option<&str>) -> u64 {
    wait_ms_or(query, 0)
}

/// `wait_ms` from a raw query string (`default` if absent or invalid), clamped to the upstream cap.
fn wait_ms_or(query: Option<&str>, default: u64) -> u64 {
    query
        .into_iter()
        .flat_map(|q| q.split('&'))
        .filter_map(|kv| kv.split_once('='))
        .find(|(k, _)| *k == "wait_ms")
        .and_then(|(_, v)| v.parse::<u64>().ok())
        .unwrap_or(default)
        .min(MAX_WAIT_MS)
}

/// GET /api/v1/detection/inference -> upstream GET /api/inference
pub async fn get_detection_inference(Extension(up): Extension<DetectorUpstream>) -> Response {
    forward(&up, Method::GET, "/api/inference".into(), None, TIMEOUT_MARGIN).await
}

/// POST /api/v1/detection/inference -> upstream POST /api/inference (body passed through)
pub async fn post_detection_inference(Extension(up): Extension<DetectorUpstream>, body: Bytes) -> Response {
    forward(&up, Method::POST, "/api/inference".into(), Some(json_body(body)), TIMEOUT_MARGIN).await
}

/// GET /api/v1/detection/latest -> upstream GET /api/results/latest (query passed through)
pub async fn get_detection_latest(
    Extension(up): Extension<DetectorUpstream>,
    RawQuery(query): RawQuery,
) -> Response {
    let timeout = Duration::from_millis(wait_ms_from_query(query.as_deref())) + TIMEOUT_MARGIN;
    let path = match &query {
        Some(q) if !q.is_empty() => format!("/api/results/latest?{q}"),
        _ => "/api/results/latest".to_string(),
    };
    forward(&up, Method::GET, path, None, timeout).await
}

/// Upstream `/api/trigger` waits this long when the query has no `wait_ms`.
const TRIGGER_DEFAULT_WAIT_MS: u64 = 2500;

/// POST /api/v1/detection/trigger -> upstream POST /api/trigger (query and JSON body passed
/// through; perception API 2 reads `{"pipelines": [...]}` from the body). Single-shot inference
/// on a frame newer than the call; does not change any continuous inference. Detector builds
/// without the route answer 404, which is passed through.
pub async fn post_detection_trigger(
    Extension(up): Extension<DetectorUpstream>,
    RawQuery(query): RawQuery,
    body: Bytes,
) -> Response {
    let wait = trigger_body_wait_ms(&body).unwrap_or_else(|| wait_ms_or(query.as_deref(), TRIGGER_DEFAULT_WAIT_MS));
    let timeout = Duration::from_millis(wait) + TIMEOUT_MARGIN;
    let path = match &query {
        Some(q) if !q.is_empty() => format!("/api/trigger?{q}"),
        _ => "/api/trigger".to_string(),
    };
    forward(&up, Method::POST, path, Some(json_body(body)), timeout).await
}

/// `wait_ms` from a trigger's JSON body, clamped to the upstream cap (the body wins over the query).
fn trigger_body_wait_ms(body: &[u8]) -> Option<u64> {
    serde_json::from_slice::<serde_json::Value>(body).ok()?.get("wait_ms")?.as_u64().map(|w| w.min(MAX_WAIT_MS))
}

// ---------------------------------------------------------------------------
// Vision subscriptions (perception API 2): continuous inference per caller
// ---------------------------------------------------------------------------

/// Subscription ids are short tokens from the provider (`s12`); anything else is not forwarded.
fn valid_subscription_id(id: &str) -> bool {
    !id.is_empty() && id.len() <= 32 && id.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_')
}

fn bad_subscription_id() -> Response {
    (StatusCode::BAD_REQUEST, axum::Json(serde_json::json!({"error": "invalid subscription id"}))).into_response()
}

/// POST /api/v1/vision/subscribe -> upstream POST /api/subscribe
/// (body `{"client", "pipelines", "fps"?, "ttl_ms"?, "id"?}` passed through)
pub async fn post_vision_subscribe(Extension(up): Extension<DetectorUpstream>, body: Bytes) -> Response {
    forward(&up, Method::POST, "/api/subscribe".into(), Some(json_body(body)), TIMEOUT_MARGIN).await
}

/// POST /api/v1/vision/subscribe/{id}/keepalive -> upstream POST /api/subscribe/{id}/keepalive
pub async fn post_vision_keepalive(Extension(up): Extension<DetectorUpstream>, Path(id): Path<String>) -> Response {
    if !valid_subscription_id(&id) {
        return bad_subscription_id();
    }
    forward(&up, Method::POST, format!("/api/subscribe/{id}/keepalive"), Some(json_body(Bytes::new())), TIMEOUT_MARGIN).await
}

/// DELETE /api/v1/vision/subscribe/{id} -> upstream DELETE /api/subscribe/{id}
pub async fn delete_vision_subscribe(Extension(up): Extension<DetectorUpstream>, Path(id): Path<String>) -> Response {
    if !valid_subscription_id(&id) {
        return bad_subscription_id();
    }
    forward(&up, Method::DELETE, format!("/api/subscribe/{id}"), None, TIMEOUT_MARGIN).await
}

/// GET /api/v1/vision/subscriptions -> upstream GET /api/subscriptions
pub async fn get_vision_subscriptions(Extension(up): Extension<DetectorUpstream>) -> Response {
    forward(&up, Method::GET, "/api/subscriptions".into(), None, TIMEOUT_MARGIN).await
}

// ---------------------------------------------------------------------------
// Vision mode (rebot-vision `/api/mode`): off | yolo | aruco | both
// ---------------------------------------------------------------------------

/// Last `/api/mode` body seen from the provider, for `capabilities.vision`.
#[derive(Clone, Default)]
pub struct VisionCache(Arc<Mutex<Option<(Instant, serde_json::Value)>>>);

/// The schema reuses a cached mode body this young instead of asking the provider.
const VISION_CACHE_MAX_AGE: Duration = Duration::from_secs(2);
/// Schema requests wait at most this long for the provider.
const VISION_SCHEMA_TIMEOUT: Duration = Duration::from_millis(500);

impl VisionCache {
    fn store(&self, body: &[u8]) {
        if let Ok(v) = serde_json::from_slice::<serde_json::Value>(body) {
            if v.get("mode").and_then(|m| m.as_str()).is_some() {
                *self.0.lock().unwrap_or_else(|e| e.into_inner()) = Some((Instant::now(), v));
            }
        }
    }
    fn clear(&self) {
        *self.0.lock().unwrap_or_else(|e| e.into_inner()) = None;
    }
    fn fresh(&self) -> Option<serde_json::Value> {
        let g = self.0.lock().unwrap_or_else(|e| e.into_inner());
        g.as_ref().filter(|(t, _)| t.elapsed() < VISION_CACHE_MAX_AGE).map(|(_, v)| v.clone())
    }

    /// Provider mode body: a fresh cached one, else `GET /api/mode` (short timeout).
    /// `None` when the provider is unreachable or has no mode API (lite, older detectors).
    pub async fn status(&self, up: &DetectorUpstream) -> Option<serde_json::Value> {
        if let Some(v) = self.fresh() {
            return Some(v);
        }
        match fetch(up, Method::GET, "/api/mode", None, VISION_SCHEMA_TIMEOUT).await {
            Some(r) if r.status.is_success() => {
                self.store(&r.body);
                self.fresh()
            }
            _ => {
                self.clear();
                None
            }
        }
    }
}

/// Fills the provider-reported fields of `capabilities.vision` from a `/api/mode` body.
pub fn apply_vision_status(caps: &mut crate::model::VisionCapabilities, v: &serde_json::Value) {
    caps.mode = v.get("mode").and_then(|m| m.as_str()).map(str::to_string);
    caps.modes = v
        .get("modes")
        .and_then(|m| m.as_array())
        .map(|a| a.iter().filter_map(|x| x.as_str().map(str::to_string)).collect())
        .unwrap_or_default();
    caps.api_version = v.get("api_version").and_then(|n| n.as_u64()).and_then(|n| u32::try_from(n).ok());
    caps.trigger = v.get("trigger").and_then(|b| b.as_bool());
    caps.yolo_available = v.get("available").and_then(|a| a.get("yolo")).and_then(|b| b.as_bool());
    caps.per_request_pipelines = v.get("per_request_pipelines").and_then(|b| b.as_bool()) == Some(true);
    caps.subscriptions = v.get("subscriptions").and_then(|b| b.as_bool()) == Some(true);
}

async fn forward_mode(up: &DetectorUpstream, cache: &VisionCache, method: Method, body: Option<Bytes>) -> Response {
    match fetch(up, method, "/api/mode", body.as_deref(), TIMEOUT_MARGIN).await {
        Some(r) => {
            if r.status.is_success() {
                cache.store(&r.body);
            }
            passthrough(r)
        }
        None => {
            cache.clear();
            detector_unreachable()
        }
    }
}

/// GET /api/v1/vision/mode -> upstream GET /api/mode
pub async fn get_vision_mode(Extension(up): Extension<DetectorUpstream>, Extension(cache): Extension<VisionCache>) -> Response {
    forward_mode(&up, &cache, Method::GET, None).await
}

/// POST /api/v1/vision/mode -> upstream POST /api/mode (body `{"mode"?, "enabled"?, "fps"?}` passed through)
pub async fn post_vision_mode(
    Extension(up): Extension<DetectorUpstream>,
    Extension(cache): Extension<VisionCache>,
    body: Bytes,
) -> Response {
    forward_mode(&up, &cache, Method::POST, Some(json_body(body))).await
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_detector_urls() {
        let u = DetectorUpstream::parse("http://127.0.0.1:8090").unwrap();
        assert_eq!((u.host.as_str(), u.port, u.base_path.as_str()), ("127.0.0.1", 8090, ""));
        let u = DetectorUpstream::parse("http://det.local/base/").unwrap();
        assert_eq!((u.host.as_str(), u.port, u.base_path.as_str()), ("det.local", 80, "/base"));
        assert!(DetectorUpstream::parse("https://x").is_err());
        assert!(DetectorUpstream::parse("http://x:notaport").is_err());
    }

    #[test]
    fn clamps_wait_ms() {
        assert_eq!(wait_ms_from_query(None), 0);
        assert_eq!(wait_ms_from_query(Some("after=3&wait_ms=3000")), 3000);
        assert_eq!(wait_ms_from_query(Some("wait_ms=99999")), 5000);
        assert_eq!(wait_ms_from_query(Some("wait_ms=abc")), 0);
        assert_eq!(wait_ms_or(None, TRIGGER_DEFAULT_WAIT_MS), 2500);
        assert_eq!(wait_ms_or(Some("wait_ms=9000"), TRIGGER_DEFAULT_WAIT_MS), 5000);
        assert_eq!(wait_ms_or(Some("wait_ms=800"), TRIGGER_DEFAULT_WAIT_MS), 800);
    }

    #[test]
    fn post_bodies_are_json_with_content_type() {
        let up = DetectorUpstream::parse("http://127.0.0.1:8090").unwrap();
        assert_eq!(json_body(Bytes::new()), Bytes::from_static(b"{}"));
        assert_eq!(json_body(Bytes::from_static(b" \n")), Bytes::from_static(b"{}"));
        assert_eq!(json_body(Bytes::from_static(b"{\"enabled\":true}")), Bytes::from_static(b"{\"enabled\":true}"));
        let r = String::from_utf8(build_request(&up, &Method::POST, "/api/trigger", Some(b"{}"))).unwrap();
        assert!(r.contains("Content-Type: application/json\r\n"));
        assert!(r.contains("Content-Length: 2\r\n"));
        assert!(r.ends_with("\r\n\r\n{}"));
        let g = String::from_utf8(build_request(&up, &Method::GET, "/api/inference", None)).unwrap();
        assert!(!g.contains("Content-Type"));
    }

    #[test]
    fn trigger_wait_from_body_and_subscription_ids() {
        assert_eq!(trigger_body_wait_ms(br#"{"pipelines":["yolo"],"wait_ms":800}"#), Some(800));
        assert_eq!(trigger_body_wait_ms(br#"{"wait_ms":90000}"#), Some(5000));
        assert_eq!(trigger_body_wait_ms(br#"{"pipelines":["aruco"]}"#), None);
        assert_eq!(trigger_body_wait_ms(b""), None);
        assert!(valid_subscription_id("s12") && valid_subscription_id("a-b_C9"));
        for id in ["", "s1/keepalive", "..", "s 1", "s1?x=1", &"x".repeat(33)] {
            assert!(!valid_subscription_id(id), "{id:?}");
        }
    }

    #[test]
    fn parses_chunked_and_partial_responses() {
        let raw = b"HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n4\r\n{\"a\"\r\n3\r\n:1}\r\n0\r\n\r\n";
        let r = parse_response(raw, false).unwrap();
        assert_eq!(r.body, b"{\"a\":1}");
        assert!(parse_response(b"HTTP/1.1 200 OK\r\nContent-Length: 10\r\n\r\n{}", false).is_none());
        assert!(parse_response(b"HTTP/1.1 200 OK\r\n\r\n{}", false).is_none());
        assert_eq!(parse_response(b"HTTP/1.1 400 Bad\r\n\r\n{}", true).unwrap().status, StatusCode::BAD_REQUEST);
    }
}
