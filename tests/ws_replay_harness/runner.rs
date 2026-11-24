//! WS replay session recorder/replayer against `armd --follower mock` binaries and
//! in-process controlled-clock / injected owner-event harnesses.

use std::collections::{HashMap, HashSet};
use std::path::Path;
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

use futures_util::{SinkExt, StreamExt};
use serde_json::{json, Value};
use tokio::net::TcpStream;
use tokio_tungstenite::tungstenite::client::IntoClientRequest;
use tokio_tungstenite::tungstenite::Message;
use tokio_tungstenite::{connect_async, MaybeTlsStream, WebSocketStream};

use super::comparator::{CloseInfo, RecordedEvent, ScriptStep, SessionSetup, SessionTranscript};
use super::seed::{materialize_seed, reserve_mock_port};

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
    seen_terminals: HashSet<(u64, String)>,
}

struct RuntimeVars {
    gens: HashMap<String, u64>,
    epoch: u64,
    token: Option<String>,
    pose_id: Option<String>,
    sequence_id: Option<String>,
    recording_id: Option<String>,
    episode_id: Option<String>,
    actions: HashMap<String, u64>,
    last_action_id: Option<u64>,
}

impl Default for RuntimeVars {
    fn default() -> Self {
        Self {
            gens: HashMap::new(),
            epoch: 1,
            token: None,
            pose_id: None,
            sequence_id: None,
            recording_id: None,
            episode_id: None,
            actions: HashMap::new(),
            last_action_id: None,
        }
    }
}

impl RuntimeVars {
    fn update_from_frame(&mut self, client: &str, v: &Value, capture_action: Option<&str>) {
        if let Some(ep) = v.get("epoch_id").and_then(Value::as_u64) {
            self.epoch = ep;
        }
        if v.get("type").and_then(Value::as_str) == Some("lease_granted") {
            if let Some(g) = v.get("generation").and_then(Value::as_u64) {
                self.gens.insert(client.to_string(), g);
            }
        }
        if let Some(aid) = v.get("action_id").and_then(Value::as_u64) {
            self.last_action_id = Some(aid);
            if let Some(alias) = capture_action {
                self.actions.insert(alias.to_string(), aid);
            }
        }
        if let Some(pid) = v.get("pose_id").and_then(Value::as_str) {
            self.pose_id = Some(pid.to_string());
        }
        if let Some(sid) = v
            .get("sequence")
            .and_then(|s| s.get("id"))
            .and_then(Value::as_str)
        {
            self.sequence_id = Some(sid.to_string());
        }
        if let Some(rid) = v.get("recording_id").and_then(Value::as_str) {
            self.recording_id = Some(rid.to_string());
        }
        if let Some(eid) = v
            .get("episode")
            .and_then(|e| e.get("id"))
            .and_then(Value::as_str)
        {
            self.episode_id = Some(eid.to_string());
        }
    }

    fn resolve_str(&self, s: &str) -> Value {
        if s == "{{epoch}}" {
            return json!(self.epoch);
        }
        if let Some(c) = s.strip_prefix("{{gen:").and_then(|r| r.strip_suffix("}}")) {
            return json!(self.gens.get(c).copied().unwrap_or(0));
        }
        if s == "{{token}}" {
            return json!(self.token.as_deref().unwrap_or(""));
        }
        if s == "{{pose_id}}" {
            return json!(self.pose_id.as_deref().unwrap_or(""));
        }
        if s == "{{sequence_id}}" {
            return json!(self.sequence_id.as_deref().unwrap_or(""));
        }
        if s == "{{recording_id}}" {
            return json!(self.recording_id.as_deref().unwrap_or(""));
        }
        if s == "{{episode_id}}" {
            return json!(self.episode_id.as_deref().unwrap_or(""));
        }
        if s.contains("{{token}}") {
            return Value::String(s.replace("{{token}}", self.token.as_deref().unwrap_or("")));
        }
        if s.contains("{{episode_id}}") {
            return Value::String(
                s.replace("{{episode_id}}", self.episode_id.as_deref().unwrap_or("")),
            );
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

async fn read_next_control(client: &mut ClientState) -> Value {
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
                }
                Message::Ping(_) | Message::Pong(_) => continue,
                Message::Close(cf) => panic!("unexpected WS Close while waiting for control: {cf:?}"),
                _ => continue,
            }
        }
    })
    .await
    .expect("timed out waiting for control frame")
}

async fn read_action_terminal(client: &mut ClientState, target_action_id: u64) -> RecordedEvent {
    tokio::time::timeout(Duration::from_secs(10), async {
        loop {
            let msg = client
                .ws
                .next()
                .await
                .expect("WS stream ended before action terminal")
                .expect("WS read error");
            let Message::Text(txt) = msg else { continue };
            let v: Value = serde_json::from_str(&txt).expect("valid server JSON");
            if v.get("type").is_some() {
                continue;
            }
            let Some(la) = v.get("last_action").filter(|x| x.is_object()) else {
                continue;
            };
            let id = la.get("id").and_then(Value::as_u64).unwrap_or(0);
            let state = la.get("state").and_then(Value::as_str).unwrap_or("");
            if id == target_action_id && matches!(state, "done" | "failed" | "cancelled") {
                client.seen_terminals.insert((id, state.to_string()));
                return RecordedEvent::ActionTerminal {
                    client: String::new(),
                    action_id: id,
                    kind: la.get("kind").and_then(Value::as_str).unwrap_or("").to_string(),
                    state: state.to_string(),
                    code: la.get("code").and_then(Value::as_str).map(str::to_string),
                    safety_state: v.get("safety_state").cloned().unwrap_or(Value::Null),
                    epoch_id: v.get("epoch_id").and_then(Value::as_u64).unwrap_or(0),
                    is_torqued: v.get("is_torqued").and_then(Value::as_bool).unwrap_or(false),
                    control_mode: v
                        .get("control_mode")
                        .and_then(Value::as_str)
                        .unwrap_or("")
                        .to_string(),
                };
            }
        }
    })
    .await
    .unwrap_or_else(|_| panic!("timed out waiting for terminal of action {target_action_id}"))
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
    let port = reserve_mock_port();
    let base_url = format!("http://127.0.0.1:{port}");
    let ws_base = format!("ws://127.0.0.1:{port}/api/v1/ws");

    let mut cmd = Command::new(binary);
    cmd.args([
        "--follower",
        "mock",
        "--config",
        seed.profile_path.to_str().unwrap(),
        "--port",
        &port.to_string(),
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
    cmd.env_remove("PORT");
    cmd.env_remove("ARMD_OPERATOR_PIN");
    cmd.env_remove("ARMD_TOOL");
    for (k, v) in &seed.env {
        cmd.env(k, v);
    }
    cmd.stdout(Stdio::null()).stderr(Stdio::null());
    let child = ChildGuard(cmd.spawn().unwrap_or_else(|e| {
        panic!("failed to spawn {} for session {id}: {e}", binary.display())
    }));

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
            "armd mock server failed to start on port {port} for session {id}"
        );
        tokio::time::sleep(Duration::from_millis(15)).await;
    }

    let events = run_steps_against(&http, &base_url, &ws_base, steps).await;
    drop(child);

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
                                seen_terminals: HashSet::new(),
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
            ScriptStep::ExpectControl {
                client,
                count,
                capture_action,
            } => {
                let cs = clients.get_mut(client).expect("unknown client");
                for i in 0..*count {
                    let frame = read_next_control(cs).await;
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
                let mut ev = read_action_terminal(cs, target_id).await;
                if let RecordedEvent::ActionTerminal {
                    client: ref mut c,
                    epoch_id,
                    ..
                } = ev
                {
                    *c = client.clone();
                    if epoch_id > 0 {
                        vars.epoch = epoch_id;
                    }
                }
                events.push(ev);
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
                        }
                        Ok(Some(Ok(Message::Ping(_) | Message::Pong(_)))) => continue,
                        Ok(Some(Ok(other))) => {
                            panic!("unexpected non-text frame on {client}: {other:?}");
                        }
                        Ok(other) => panic!("WS closed unexpectedly during ExpectNoControl: {other:?}"),
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
                            Some(Ok(Message::Close(cf))) => {
                                return CloseInfo {
                                    code: cf.as_ref().map(|c| c.code.into()),
                                    reason: cf.as_ref().map(|c| c.reason.to_string()),
                                };
                            }
                            None => return CloseInfo { code: None, reason: None },
                            Some(Ok(_)) => continue,
                            Some(Err(_)) => return CloseInfo { code: None, reason: None },
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
                let mut last_ts: Option<f64> = None;
                let mut monotonic = true;
                let mut dof = 0usize;
                let mut safety_state = Value::Null;
                let mut epoch_id = 0u64;
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
                                continue;
                            }
                            count += 1;
                            if let Some(ts) = v.get("timestamp").and_then(Value::as_f64) {
                                if last_ts.is_some_and(|prev| ts < prev) {
                                    monotonic = false;
                                }
                                last_ts = Some(ts);
                            }
                            if let Some(arr) = v.get("positions").and_then(Value::as_array) {
                                dof = arr.len();
                            }
                            safety_state = v.get("safety_state").cloned().unwrap_or(Value::Null);
                            epoch_id = v.get("epoch_id").and_then(Value::as_u64).unwrap_or(0);
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
                events.push(RecordedEvent::TelemetrySummary {
                    client: client.clone(),
                    in_range,
                    monotonic,
                    dof,
                    safety_state,
                    epoch_id,
                });
            }
            ScriptStep::SyncEpoch => {
                vars.epoch = fetch_epoch(http, base_url).await;
            }
        }
    }

    events
}
