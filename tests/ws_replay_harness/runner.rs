//! Live replay recorder & executor driving `armd --follower mock` (or an in-process router)
//! against scripted WS + REST owner-event sessions.
//!
//! Records every inbound control frame and unseen action terminal at the exact point it arrives
//! on the wire (never buffering/reordering terminals across ACKs and never discarding unexpected
//! frames during waits or teardown).

use std::collections::{HashMap, HashSet};
use std::io::{BufRead, BufReader};
use std::net::SocketAddr;
use std::path::Path;
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

use futures_util::{SinkExt, StreamExt};
use serde_json::{json, Value};
use tokio::net::TcpStream;
use tokio_tungstenite::tungstenite::client::IntoClientRequest;
use tokio_tungstenite::tungstenite::protocol::frame::coding::CloseCode;
use tokio_tungstenite::tungstenite::protocol::CloseFrame;
use tokio_tungstenite::tungstenite::Message;
use tokio_tungstenite::{connect_async, MaybeTlsStream, WebSocketStream};

use super::comparator::{CloseOutcome, RecordedEvent, ScriptStep, SessionSetup, SessionTranscript};
use super::seed::{materialize_seed, FORBIDDEN_PORTS};

type WsConn = WebSocketStream<MaybeTlsStream<TcpStream>>;

struct ChildGuard(Child);

impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

struct ClientState {
    ws: WsConn,
    /// Action IDs whose terminal observation has already been recorded in `events` (used only
    /// to suppress duplicate `previous_action` snapshots repeated on subsequent 50 Hz ticks).
    recorded_terminals: HashSet<u64>,
}

#[derive(Default)]
struct RuntimeVars {
    gens: HashMap<String, u64>,
    epoch: u64,
    token: Option<String>,
    pose_id: Option<String>,
    sequence_id: Option<String>,
    recording_id: Option<String>,
    episode_id: Option<String>,
    actions: HashMap<String, u64>,
    action_expected_s: HashMap<u64, f64>,
    last_action_id: Option<u64>,
}

impl RuntimeVars {
    fn update_from_frame(&mut self, client: &str, frame: &Value, capture_action: Option<&str>) {
        if frame.get("type").and_then(Value::as_str) == Some("lease_granted") {
            if let Some(g) = frame.get("generation").and_then(Value::as_u64) {
                self.gens.insert(client.to_string(), g);
            }
        }
        if let Some(ep) = frame.get("epoch_id").and_then(Value::as_u64) {
            self.epoch = ep;
        }
        if let Some(aid) = frame.get("action_id").and_then(Value::as_u64) {
            self.last_action_id = Some(aid);
            if let Some(exp) = frame.get("expected_duration_s").and_then(Value::as_f64) {
                if exp.is_finite() && exp >= 0.0 {
                    self.action_expected_s.insert(aid, exp);
                }
            }
            if let Some(alias) = capture_action {
                self.actions.insert(alias.to_string(), aid);
            }
        }
        if let Some(pid) = frame.get("pose_id").and_then(Value::as_str) {
            self.pose_id = Some(pid.to_string());
        } else if let Some(pid) = frame
            .get("pose")
            .and_then(|p| p.get("id"))
            .and_then(Value::as_str)
        {
            self.pose_id = Some(pid.to_string());
        }
        if let Some(sid) = frame
            .get("sequence")
            .and_then(|s| s.get("id"))
            .and_then(Value::as_str)
        {
            self.sequence_id = Some(sid.to_string());
        }
        if let Some(rid) = frame.get("recording_id").and_then(Value::as_str) {
            self.recording_id = Some(rid.to_string());
        }
        if let Some(eid) = frame
            .get("episode")
            .and_then(|e| e.get("id"))
            .and_then(Value::as_str)
        {
            self.episode_id = Some(eid.to_string());
        }
    }

    fn resolve_str(&self, s: &str) -> Value {
        if s == "{{epoch}}" {
            return Value::from(self.epoch);
        }
        if let Some(client) = s.strip_prefix("{{gen:").and_then(|r| r.strip_suffix("}}")) {
            return Value::from(self.gens.get(client).copied().unwrap_or(0));
        }
        if s == "{{token}}" {
            return Value::String(self.token.clone().unwrap_or_default());
        }
        if s == "{{pose_id}}" {
            return Value::String(self.pose_id.clone().unwrap_or_default());
        }
        if s == "{{sequence_id}}" {
            return Value::String(self.sequence_id.clone().unwrap_or_default());
        }
        if s == "{{recording_id}}" {
            return Value::String(self.recording_id.clone().unwrap_or_default());
        }
        if s == "{{episode_id}}" {
            return Value::String(self.episode_id.clone().unwrap_or_default());
        }
        if s.contains("{{token}}") {
            return Value::String(s.replace("{{token}}", self.token.as_deref().unwrap_or("")));
        }
        Value::String(s.to_string())
    }

    fn resolve_value(&self, v: &Value) -> Value {
        match v {
            Value::String(s) => self.resolve_str(s),
            Value::Array(arr) => Value::Array(arr.iter().map(|x| self.resolve_value(x)).collect()),
            Value::Object(map) => {
                let mut out = serde_json::Map::new();
                for (k, val) in map {
                    out.insert(k.clone(), self.resolve_value(val));
                }
                Value::Object(out)
            }
            other => other.clone(),
        }
    }
}

async fn fetch_epoch(http: &reqwest::Client, base_url: &str) -> u64 {
    let obs: Value = http
        .get(format!("{base_url}/api/v1/observation"))
        .send()
        .await
        .expect("GET /api/v1/observation")
        .json()
        .await
        .expect("parse observation JSON");
    obs["epoch_id"].as_u64().unwrap_or(1)
}

/// Extracts any unseen terminal action(s) from an observation frame and appends them directly
/// into `events` in exact receive order.
fn record_unseen_terminals(
    client_name: &str,
    v: &Value,
    recorded_terminals: &mut HashSet<u64>,
    vars: &mut RuntimeVars,
    events: &mut Vec<RecordedEvent>,
) {
    for key in ["action", "previous_action"] {
        let Some(la) = v.get(key).filter(|x| x.is_object()) else {
            continue;
        };
        let id = la.get("id").and_then(Value::as_u64).unwrap_or(0);
        let state = la.get("state").and_then(Value::as_str).unwrap_or("");
        if id > 0 && matches!(state, "done" | "failed" | "cancelled") && recorded_terminals.insert(id)
        {
            let epoch_id = v.get("epoch_id").and_then(Value::as_u64).unwrap_or(0);
            if epoch_id > 0 {
                vars.epoch = epoch_id;
            }
            events.push(RecordedEvent::ActionTerminal {
                client: client_name.to_string(),
                action: la.clone(),
                safety_state: v.get("safety_state").cloned().unwrap_or(Value::Null),
                epoch_id,
                is_torqued: v.get("is_torqued").and_then(Value::as_bool).unwrap_or(false),
                control_mode: v
                    .get("control_mode")
                    .and_then(Value::as_str)
                    .unwrap_or("")
                    .to_string(),
            });
        }
    }
}

/// Reads until the next control frame (`type.is_some()`), recording any unseen action terminals
/// that arrive BEFORE the control frame immediately in wire receive order.
async fn read_next_control(
    client_name: &str,
    client: &mut ClientState,
    vars: &mut RuntimeVars,
    events: &mut Vec<RecordedEvent>,
) -> Value {
    tokio::time::timeout(Duration::from_secs(6), async {
        loop {
            let msg = client
                .ws
                .next()
                .await
                .expect("WS stream ended before control frame")
                .expect("WS read error");
            match msg {
                Message::Text(txt) => {
                    let v: Value = serde_json::from_str(&txt).expect("valid server JSON");
                    if v.get("type").is_some() {
                        return v;
                    }
                    record_unseen_terminals(
                        client_name,
                        &v,
                        &mut client.recorded_terminals,
                        vars,
                        events,
                    );
                }
                Message::Ping(_) | Message::Pong(_) => continue,
                Message::Close(cf) => {
                    panic!("unexpected WS Close while waiting for control: {cf:?}")
                }
                _ => continue,
            }
        }
    })
    .await
    .expect("timed out waiting for control frame")
}

/// Waits until `target_action_id` has been recorded as an `ActionTerminal`.
/// Any control frames or other unseen terminals arriving in the meantime are recorded verbatim
/// in wire receive order (never discarded).
async fn wait_for_action_terminal(
    client_name: &str,
    client: &mut ClientState,
    target_action_id: u64,
    vars: &mut RuntimeVars,
    events: &mut Vec<RecordedEvent>,
) {
    if client.recorded_terminals.contains(&target_action_id) {
        return;
    }
    let timeout = vars
        .action_expected_s
        .get(&target_action_id)
        .copied()
        .map_or(Duration::from_secs(10), |exp| {
            Duration::from_secs_f64((exp * 2.0 + 5.0).max(10.0))
        });
    tokio::time::timeout(timeout, async {
        loop {
            let msg = client
                .ws
                .next()
                .await
                .expect("WS stream ended before action terminal")
                .expect("WS read error");
            match msg {
                Message::Text(txt) => {
                    let v: Value = serde_json::from_str(&txt).expect("valid server JSON");
                    if v.get("type").is_some() {
                        vars.update_from_frame(client_name, &v, None);
                        events.push(RecordedEvent::ServerFrame {
                            client: client_name.to_string(),
                            frame: v,
                        });
                        continue;
                    }
                    record_unseen_terminals(
                        client_name,
                        &v,
                        &mut client.recorded_terminals,
                        vars,
                        events,
                    );
                    if client.recorded_terminals.contains(&target_action_id) {
                        return;
                    }
                }
                Message::Ping(_) | Message::Pong(_) => continue,
                Message::Close(cf) => {
                    panic!("unexpected WS Close while waiting for action {target_action_id}: {cf:?}")
                }
                _ => continue,
            }
        }
    })
    .await
    .unwrap_or_else(|_| {
        panic!("timed out after {timeout:?} waiting for terminal of action {target_action_id}")
    });
}

/// Quantize a joint-position array from a telemetry frame to integer milliradians (`1e-3 rad`).
pub fn quantize_positions_mrad(v: &Value) -> Vec<i32> {
    v.get("positions")
        .and_then(Value::as_array)
        .map(|arr| {
            arr.iter()
                .filter_map(Value::as_f64)
                .map(|r| (r * 1000.0).round() as i32)
                .collect()
        })
        .unwrap_or_default()
}

/// Extract `127.0.0.1:<port>` from armd's startup log line (`armd API server listening on http://...`).
fn parse_listening_addr(line: &str) -> Option<SocketAddr> {
    let marker = "armd API server listening on http://";
    let rest = line.split(marker).nth(1)?;
    // Strip any trailing ANSI escape sequence or whitespace.
    let token: String = rest
        .chars()
        .take_while(|c| c.is_ascii_digit() || matches!(c, '.' | ':' | '[' | ']'))
        .collect();
    token.parse::<SocketAddr>().ok()
}

/// Run a replay session script against an `armd` binary spawned with `--follower mock` and
/// seeded state, returning the newly recorded [`SessionTranscript`].
pub async fn execute_binary_session(
    binary: &Path,
    id: &str,
    family: &str,
    description: &str,
    setup: &SessionSetup,
    steps: &[ScriptStep],
) -> SessionTranscript {
    let seed = materialize_seed(setup).await;

    // Let the kernel assign an ephemeral port (`--port 0`) directly inside `armd` so concurrent
    // sessions and test processes never race on bind-drop-rebind probe sockets.
    let mut cmd = Command::new(binary);
    cmd.args([
        "--follower",
        "mock",
        "--config",
        seed.profile_path.to_str().unwrap(),
        "--port",
        "0",
        "--episodes-dir",
        seed.episodes_dir.to_str().unwrap(),
        "--log-dir",
        "off",
        "--blackbox-dir",
        "off",
    ]);
    if let Some(pin) = setup.operator_pin.as_deref() {
        cmd.args(["--operator-pin", pin]);
    }
    cmd.env("ARMD_BIND_HOST", "127.0.0.1");
    cmd.env_remove("PORT");
    cmd.env_remove("RUST_LOG");
    cmd.env_remove("ARMD_OPERATOR_PIN");
    cmd.env_remove("ARMD_TOOL");
    for (k, v) in &seed.env {
        cmd.env(k, v);
    }
    cmd.stdout(Stdio::piped()).stderr(Stdio::null());
    let mut spawned = cmd.spawn().unwrap_or_else(|e| {
        panic!("failed to spawn {} for session {id}: {e}", binary.display())
    });
    let stdout = spawned.stdout.take().expect("piped stdout from armd");
    let child = ChildGuard(spawned);

    let (addr_tx, addr_rx) = tokio::sync::oneshot::channel::<SocketAddr>();
    let drain_handle = std::thread::spawn(move || {
        let mut tx = Some(addr_tx);
        for line in BufReader::new(stdout).lines().map_while(Result::ok) {
            if let Some(sender) = tx.as_ref() {
                if let Some(addr) = parse_listening_addr(&line) {
                    let _ = sender;
                    if let Some(sender) = tx.take() {
                        let _ = sender.send(addr);
                    }
                }
            }
        }
    });

    let bound_addr = tokio::time::timeout(Duration::from_secs(6), addr_rx)
        .await
        .unwrap_or_else(|_| panic!("timed out waiting for armd --port 0 listening addr in session {id}"))
        .unwrap_or_else(|_| panic!("armd exited before reporting listening addr in session {id}"));
    assert!(
        !FORBIDDEN_PORTS.contains(&bound_addr.port()),
        "armd bound forbidden port {} in session {id}",
        bound_addr.port()
    );
    let base_url = format!("http://{bound_addr}");
    let ws_base = format!("ws://{bound_addr}/api/v1/ws");

    let http = reqwest::Client::new();
    let deadline = Instant::now() + Duration::from_secs(6);
    loop {
        if let Ok(r) = http.get(format!("{base_url}/api/v1/observation")).send().await {
            if r.status().is_success() {
                break;
            }
        }
        assert!(
            Instant::now() < deadline,
            "armd mock server failed to start on {bound_addr} for session {id}"
        );
        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    let events = run_steps_against(&http, &base_url, &ws_base, steps).await;
    drop(child);
    let _ = drain_handle.join();

    SessionTranscript {
        id: id.to_string(),
        family: family.to_string(),
        description: description.to_string(),
        setup: setup.clone(),
        steps: steps.to_vec(),
        events,
    }
}

/// Shared step executor usable against both spawned `armd` binaries and in-process routers.
pub async fn run_steps_against(
    http: &reqwest::Client,
    base_url: &str,
    ws_base: &str,
    steps: &[ScriptStep],
) -> Vec<RecordedEvent> {
    let mut clients: HashMap<String, ClientState> = HashMap::new();
    let mut vars = RuntimeVars {
        epoch: fetch_epoch(http, base_url).await,
        ..Default::default()
    };
    let mut events = Vec::new();

    for step in steps {
        match step {
            ScriptStep::Connect {
                client,
                query,
                auth_header,
                expect_http_status,
            } => {
                let url = if query.is_empty() {
                    ws_base.to_string()
                } else {
                    format!("{ws_base}?{query}")
                };
                let mut req = url.into_client_request().unwrap();
                if let Some(auth_val) = auth_header {
                    let resolved = vars.resolve_str(auth_val);
                    req.headers_mut().insert(
                        "authorization",
                        resolved.as_str().unwrap_or("").parse().unwrap(),
                    );
                }
                match connect_async(req).await {
                    Ok((ws, resp)) => {
                        let status = resp.status().as_u16();
                        if let Some(exp) = expect_http_status {
                            assert_eq!(status, *exp, "unexpected WS upgrade status");
                        }
                        events.push(RecordedEvent::UpgradeStatus {
                            client: client.clone(),
                            status,
                        });
                        clients.insert(
                            client.clone(),
                            ClientState {
                                ws,
                                recorded_terminals: HashSet::new(),
                            },
                        );
                    }
                    Err(tokio_tungstenite::tungstenite::Error::Http(resp)) => {
                        let status = resp.status().as_u16();
                        if let Some(exp) = expect_http_status {
                            assert_eq!(status, *exp, "unexpected WS HTTP reject status");
                        } else {
                            panic!("unexpected WS upgrade failure {status} for {client}");
                        }
                        events.push(RecordedEvent::UpgradeStatus {
                            client: client.clone(),
                            status,
                        });
                    }
                    Err(e) => panic!("WS connect error for {client}: {e}"),
                }
            }
            ScriptStep::Send { client, frame } => {
                let resolved = vars.resolve_value(frame);
                let cs = clients.get_mut(client).expect("unknown client");
                cs.ws
                    .send(Message::Text(resolved.to_string().into()))
                    .await
                    .expect("WS send");
                events.push(RecordedEvent::ClientFrame {
                    client: client.clone(),
                    frame: resolved,
                });
            }
            ScriptStep::SendRaw { client, text } => {
                let cs = clients.get_mut(client).expect("unknown client");
                cs.ws
                    .send(Message::Text(text.clone().into()))
                    .await
                    .expect("WS send raw");
                events.push(RecordedEvent::ClientRaw {
                    client: client.clone(),
                    text: text.clone(),
                });
            }
            ScriptStep::SendCloseFrame {
                client,
                code,
                reason,
            } => {
                let cs = clients.get_mut(client).expect("unknown client");
                cs.ws
                    .send(Message::Close(Some(CloseFrame {
                        code: CloseCode::from(*code),
                        reason: reason.clone().into(),
                    })))
                    .await
                    .expect("WS send close frame");
                events.push(RecordedEvent::ClientCloseFrame {
                    client: client.clone(),
                    code: *code,
                    reason: reason.clone(),
                });
            }
            ScriptStep::ExpectControl {
                client,
                count,
                capture_action,
            } => {
                let cs = clients.get_mut(client).expect("unknown client");
                for i in 0..*count {
                    let frame = read_next_control(client, cs, &mut vars, &mut events).await;
                    let alias = if i + 1 == *count {
                        capture_action.as_deref()
                    } else {
                        None
                    };
                    vars.update_from_frame(client, &frame, alias);
                    events.push(RecordedEvent::ServerFrame {
                        client: client.clone(),
                        frame,
                    });
                }
            }
            ScriptStep::ExpectTerminal {
                client,
                action_alias,
            } => {
                let target_id = match action_alias {
                    Some(alias) => *vars
                        .actions
                        .get(alias)
                        .unwrap_or_else(|| panic!("unknown action alias {alias}")),
                    None => vars.last_action_id.expect("no action_id captured yet"),
                };
                let cs = clients.get_mut(client).expect("unknown client");
                wait_for_action_terminal(client, cs, target_id, &mut vars, &mut events).await;
            }
            ScriptStep::ExpectNoControl { client, wait_ms } => {
                let cs = clients.get_mut(client).expect("unknown client");
                let deadline = tokio::time::Instant::now() + Duration::from_millis(*wait_ms);
                loop {
                    let rem = deadline.saturating_duration_since(tokio::time::Instant::now());
                    if rem.is_zero() {
                        break;
                    }
                    match tokio::time::timeout(rem, cs.ws.next()).await {
                        Err(_) => break,
                        Ok(Some(Ok(Message::Text(txt)))) => {
                            let v: Value = serde_json::from_str(&txt).expect("valid server JSON");
                            if v.get("type").is_some() {
                                panic!("expected no control frame on {client}, got: {v}");
                            }
                            record_unseen_terminals(
                                client,
                                &v,
                                &mut cs.recorded_terminals,
                                &mut vars,
                                &mut events,
                            );
                        }
                        Ok(Some(Ok(Message::Ping(_) | Message::Pong(_)))) => continue,
                        Ok(Some(Ok(other))) => {
                            panic!("unexpected non-text frame on {client}: {other:?}");
                        }
                        Ok(other) => {
                            panic!("WS closed unexpectedly during ExpectNoControl: {other:?}")
                        }
                    }
                }
                events.push(RecordedEvent::NoReply {
                    client: client.clone(),
                });
            }
            ScriptStep::ExpectClose { client } => {
                let cs = clients.get_mut(client).expect("unknown client");
                let close = tokio::time::timeout(Duration::from_secs(5), async {
                    loop {
                        match cs.ws.next().await {
                            Some(Ok(Message::Text(txt))) => {
                                let v: Value =
                                    serde_json::from_str(&txt).expect("valid server JSON");
                                if v.get("type").is_some() {
                                    vars.update_from_frame(client, &v, None);
                                    events.push(RecordedEvent::ServerFrame {
                                        client: client.clone(),
                                        frame: v,
                                    });
                                } else {
                                    record_unseen_terminals(
                                        client,
                                        &v,
                                        &mut cs.recorded_terminals,
                                        &mut vars,
                                        &mut events,
                                    );
                                }
                            }
                            Some(Ok(Message::Close(cf))) => {
                                return CloseOutcome::Frame {
                                    code: cf.as_ref().map(|c| c.code.into()),
                                    reason: cf.as_ref().map(|c| c.reason.to_string()),
                                };
                            }
                            Some(Ok(Message::Ping(_) | Message::Pong(_) | Message::Binary(_))) => {
                                continue
                            }
                            Some(Ok(Message::Frame(_))) => continue,
                            None => return CloseOutcome::Eof,
                            Some(Err(_)) => return CloseOutcome::TransportError,
                        }
                    }
                })
                .await
                .expect("timed out waiting for WS Close");
                events.push(RecordedEvent::ServerClose {
                    client: client.clone(),
                    close,
                });
                clients.remove(client);
            }
            ScriptStep::CloseClient { client } => {
                if let Some(mut cs) = clients.remove(client) {
                    // Drain any already-buffered unread control frames, unseen terminals, or
                    // unexpected server Close frames before closing so nothing is silently dropped.
                    let drain_deadline =
                        tokio::time::Instant::now() + Duration::from_millis(25);
                    let mut unexpected_close: Option<CloseOutcome> = None;
                    loop {
                        let rem = drain_deadline
                            .saturating_duration_since(tokio::time::Instant::now());
                        if rem.is_zero() {
                            break;
                        }
                        match tokio::time::timeout(rem, cs.ws.next()).await {
                            Ok(Some(Ok(Message::Text(txt)))) => {
                                if let Ok(v) = serde_json::from_str::<Value>(&txt) {
                                    if v.get("type").is_some() {
                                        events.push(RecordedEvent::ServerFrame {
                                            client: client.clone(),
                                            frame: v,
                                        });
                                    } else {
                                        record_unseen_terminals(
                                            client,
                                            &v,
                                            &mut cs.recorded_terminals,
                                            &mut vars,
                                            &mut events,
                                        );
                                    }
                                }
                            }
                            Ok(Some(Ok(Message::Close(cf)))) => {
                                unexpected_close = Some(CloseOutcome::Frame {
                                    code: cf.as_ref().map(|c| c.code.into()),
                                    reason: cf.as_ref().map(|c| c.reason.to_string()),
                                });
                                break;
                            }
                            Ok(Some(Ok(Message::Ping(_) | Message::Pong(_)))) => continue,
                            Ok(None) => {
                                unexpected_close = Some(CloseOutcome::Eof);
                                break;
                            }
                            Ok(Some(Err(_))) => {
                                unexpected_close = Some(CloseOutcome::TransportError);
                                break;
                            }
                            _ => break,
                        }
                    }
                    if let Some(close) = unexpected_close {
                        events.push(RecordedEvent::ServerClose {
                            client: client.clone(),
                            close,
                        });
                    }
                    let _ = cs.ws.close(None).await;
                    let _ = tokio::time::timeout(Duration::from_millis(200), async {
                        while cs.ws.next().await.is_some() {}
                    })
                    .await;
                }
                events.push(RecordedEvent::ClientClosed {
                    client: client.clone(),
                });
            }
            ScriptStep::Rest {
                method,
                path,
                body,
                auth_header,
                capture_token,
                record_body,
            } => {
                let resolved_path = vars
                    .resolve_str(path)
                    .as_str()
                    .unwrap_or(path)
                    .to_string();
                let url = format!("{base_url}{resolved_path}");
                let mut req = match method.as_str() {
                    "GET" => http.get(&url),
                    "POST" => http.post(&url),
                    "DELETE" => http.delete(&url),
                    other => panic!("unsupported REST method {other}"),
                };
                if let Some(auth_val) = auth_header {
                    let resolved = vars.resolve_str(auth_val);
                    req = req.header("authorization", resolved.as_str().unwrap_or(""));
                }
                if let Some(b) = body {
                    req = req.json(&vars.resolve_value(b));
                }
                let resp = req.send().await.expect("REST request");
                let status = resp.status().as_u16();
                let parsed: Value = resp.json().await.unwrap_or(Value::Null);
                if *capture_token {
                    if let Some(tok) = parsed.get("token").and_then(Value::as_str) {
                        vars.token = Some(tok.to_string());
                    }
                }
                if let Some(ep) = parsed.get("epoch_id").and_then(Value::as_u64) {
                    vars.epoch = ep;
                }
                let recorded_body = if *record_body {
                    parsed
                } else {
                    json!({"present": !parsed.is_null()})
                };
                events.push(RecordedEvent::RestResponse {
                    method: method.clone(),
                    path: resolved_path,
                    status,
                    body: recorded_body,
                });
            }
            ScriptStep::SleepMs { ms } => {
                tokio::time::sleep(Duration::from_millis(*ms)).await;
            }
            ScriptStep::CollectTelemetry {
                client,
                duration_ms,
                min_count,
                max_count,
            } => {
                let cs = clients.get_mut(client).expect("unknown client");
                let deadline = tokio::time::Instant::now() + Duration::from_millis(*duration_ms);
                let mut count = 0usize;
                let mut ts_count = 0usize;
                let mut last_ts: Option<f64> = None;
                let mut strictly_increasing = true;
                let mut first_shape: Option<(usize, bool, String, Vec<i32>, Value, u64)> = None;
                let mut all_samples_consistent = true;
                loop {
                    let rem = deadline.saturating_duration_since(tokio::time::Instant::now());
                    if rem.is_zero() {
                        break;
                    }
                    match tokio::time::timeout(rem, cs.ws.next()).await {
                        Err(_) => break,
                        Ok(Some(Ok(Message::Text(txt)))) => {
                            let v: Value = serde_json::from_str(&txt).expect("valid server JSON");
                            if v.get("type").is_some() {
                                vars.update_from_frame(client, &v, None);
                                events.push(RecordedEvent::ServerFrame {
                                    client: client.clone(),
                                    frame: v,
                                });
                                continue;
                            }
                            record_unseen_terminals(
                                client,
                                &v,
                                &mut cs.recorded_terminals,
                                &mut vars,
                                &mut events,
                            );
                            count += 1;
                            if let Some(ts) = v.get("timestamp").and_then(Value::as_f64) {
                                ts_count += 1;
                                if last_ts.is_some_and(|prev| ts <= prev) {
                                    strictly_increasing = false;
                                }
                                last_ts = Some(ts);
                            }
                            let qpos = quantize_positions_mrad(&v);
                            let sample_shape = (
                                qpos.len(),
                                v.get("is_torqued").and_then(Value::as_bool).unwrap_or(false),
                                v.get("control_mode")
                                    .and_then(Value::as_str)
                                    .unwrap_or("")
                                    .to_string(),
                                qpos,
                                v.get("safety_state").cloned().unwrap_or(Value::Null),
                                v.get("epoch_id").and_then(Value::as_u64).unwrap_or(0),
                            );
                            match &first_shape {
                                None => first_shape = Some(sample_shape),
                                Some(expected_shape) => {
                                    if *expected_shape != sample_shape {
                                        all_samples_consistent = false;
                                    }
                                }
                            }
                        }
                        Ok(Some(Ok(_))) => continue,
                        Ok(_) => break,
                    }
                }
                let in_range = count >= *min_count && count <= *max_count;
                assert!(
                    in_range,
                    "telemetry count {count} outside [{min_count}, {max_count}] on {client}"
                );
                let has_all_timestamps = count >= 2 && ts_count == count;
                let monotonic = has_all_timestamps && strictly_increasing;
                let (dof, is_torqued, control_mode, positions_mrad, safety_state, epoch_id) =
                    first_shape.unwrap_or((0, false, String::new(), Vec::new(), Value::Null, 0));
                events.push(RecordedEvent::TelemetrySummary {
                    client: client.clone(),
                    in_range,
                    has_all_timestamps,
                    monotonic,
                    all_samples_consistent,
                    dof,
                    is_torqued,
                    control_mode,
                    positions_mrad,
                    safety_state,
                    epoch_id,
                });
            }
            ScriptStep::SyncEpoch => {
                vars.epoch = fetch_epoch(http, base_url).await;
            }
            ScriptStep::CheckState => {
                let obs: Value = http
                    .get(format!("{base_url}/api/v1/observation"))
                    .send()
                    .await
                    .expect("GET /api/v1/observation")
                    .json()
                    .await
                    .expect("parse observation JSON");
                let epoch_id = obs["epoch_id"].as_u64().unwrap_or(0);
                if epoch_id > 0 {
                    vars.epoch = epoch_id;
                }
                let active_action = obs
                    .get("action")
                    .and_then(|a| a.get("state"))
                    .and_then(Value::as_str)
                    .map(str::to_string);
                events.push(RecordedEvent::StateSnapshot {
                    safety_state: obs.get("safety_state").cloned().unwrap_or(Value::Null),
                    is_torqued: obs
                        .get("is_torqued")
                        .and_then(Value::as_bool)
                        .unwrap_or(false),
                    control_mode: obs
                        .get("control_mode")
                        .and_then(Value::as_str)
                        .unwrap_or("")
                        .to_string(),
                    epoch_id,
                    active_action,
                });
            }
        }
    }

    events
}
