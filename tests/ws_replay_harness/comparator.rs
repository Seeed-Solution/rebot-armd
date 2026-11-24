//! Transcript schema and equivalence-preserving comparator for WS replay parity.
//!
//! Normalizes ONLY genuinely nondeterministic values (wall timestamps, ephemeral durations,
//! opaque generated IDs, epochs, lease generations, stop generations, action IDs) using
//! bijective first-appearance tables so equality and ordering relationships are strictly
//! preserved. Frame kinds, payload keys, status/error codes, messages, ordering, and close
//! codes/reasons are compared verbatim.

use std::collections::{BTreeMap, HashMap};
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SessionSetup {
    /// Profile preset: `"rebot_dm"` (default), `"rebot_dm_short_lease"`, `"default_6dof"`.
    #[serde(default = "default_profile")]
    pub profile: String,
    /// Optional operator PIN (`--operator-pin`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub operator_pin: Option<String>,
    #[serde(default)]
    pub seed_poses: bool,
    #[serde(default)]
    pub seed_episode: bool,
    #[serde(default)]
    pub seed_policy: bool,
    #[serde(default)]
    pub seed_calibration: bool,
    #[serde(default)]
    pub mock_detector: bool,
}

impl Default for SessionSetup {
    fn default() -> Self {
        Self {
            profile: default_profile(),
            operator_pin: None,
            seed_poses: false,
            seed_episode: false,
            seed_policy: false,
            seed_calibration: false,
            mock_detector: false,
        }
    }
}

fn default_profile() -> String {
    "rebot_dm".to_string()
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "op", rename_all = "snake_case")]
pub enum ScriptStep {
    /// Open a WS connection (or assert an HTTP error status on upgrade failure).
    Connect {
        client: String,
        #[serde(default, skip_serializing_if = "String::is_empty")]
        query: String,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        auth_header: Option<String>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        expect_http_status: Option<u16>,
    },
    /// Send a raw text frame (JSON object or arbitrary malformed text).
    /// Supports runtime placeholders: `{{gen:c0}}`, `{{epoch}}`, `{{token}}`,
    /// `{{pose_id}}`, `{{sequence_id}}`, `{{recording_id}}`, `{{episode_id}}`.
    Send {
        client: String,
        frame: Value,
    },
    /// Send a raw non-JSON text frame.
    SendRaw {
        client: String,
        text: String,
    },
    /// Wait for `count` control frames (messages with a `"type"` field) on `client`.
    ExpectControl {
        client: String,
        #[serde(default = "one")]
        count: usize,
        /// Optional alias name to capture `action_id` from the last received frame.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        capture_action: Option<String>,
    },
    /// Wait for a terminal observation for a previously captured action (or latest action_id).
    ExpectTerminal {
        client: String,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        action_alias: Option<String>,
    },
    /// Assert no control frame arrives on `client` within `wait_ms`.
    ExpectNoControl {
        client: String,
        wait_ms: u64,
    },
    /// Wait for server-initiated WS Close on `client`.
    ExpectClose {
        client: String,
    },
    /// Client cleanly closes its WS connection.
    CloseClient {
        client: String,
    },
    /// Inject an owner-side event or probe via HTTP REST.
    Rest {
        method: String,
        path: String,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        body: Option<Value>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        auth_header: Option<String>,
        #[serde(default)]
        capture_token: bool,
        /// Whether to record the full response body (default true; false records status + keys).
        #[serde(default = "yes")]
        record_body: bool,
    },
    /// Controlled wait (e.g. lease timeout expiry or recording window).
    SleepMs {
        ms: u64,
    },
    /// Collect telemetry observations for `duration_ms` and record rate/shape summary.
    CollectTelemetry {
        client: String,
        duration_ms: u64,
        min_count: usize,
        max_count: usize,
    },
    /// Refresh `{{epoch}}` from `GET /api/v1/observation` without recording a REST event.
    SyncEpoch,
}

fn one() -> usize {
    1
}
fn yes() -> bool {
    true
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CloseInfo {
    pub code: Option<u16>,
    pub reason: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "ev", rename_all = "snake_case")]
pub enum RecordedEvent {
    UpgradeStatus {
        client: String,
        status: u16,
    },
    ClientFrame {
        client: String,
        frame: Value,
    },
    ClientRaw {
        client: String,
        text: String,
    },
    ServerFrame {
        client: String,
        frame: Value,
    },
    ActionTerminal {
        client: String,
        action_id: u64,
        kind: String,
        state: String,
        code: Option<String>,
        safety_state: Value,
        epoch_id: u64,
        is_torqued: bool,
        control_mode: String,
    },
    NoReply {
        client: String,
    },
    ServerClose {
        client: String,
        close: CloseInfo,
    },
    ClientClosed {
        client: String,
    },
    RestResponse {
        method: String,
        path: String,
        status: u16,
        body: Value,
    },
    TelemetrySummary {
        client: String,
        in_range: bool,
        monotonic: bool,
        dof: usize,
        safety_state: Value,
        epoch_id: u64,
    },
    OwnerEvent {
        label: String,
        detail: Value,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SessionTranscript {
    pub id: String,
    pub family: String,
    pub description: String,
    pub setup: SessionSetup,
    pub steps: Vec<ScriptStep>,
    pub events: Vec<RecordedEvent>,
}

impl SessionTranscript {
    /// Serializes with one compact JSON line per step and event so transcripts are readable,
    /// diff-friendly, and concise.
    pub fn to_compact_json(&self) -> Result<String, serde_json::Error> {
        let mut out = String::new();
        out.push_str("{\n");
        out.push_str(&format!("  \"id\": {},\n", serde_json::to_string(&self.id)?));
        out.push_str(&format!("  \"family\": {},\n", serde_json::to_string(&self.family)?));
        out.push_str(&format!("  \"description\": {},\n", serde_json::to_string(&self.description)?));
        out.push_str(&format!("  \"setup\": {},\n", serde_json::to_string(&self.setup)?));
        out.push_str("  \"steps\": [\n");
        for (i, step) in self.steps.iter().enumerate() {
            let comma = if i + 1 < self.steps.len() { "," } else { "" };
            out.push_str(&format!("    {}{comma}\n", serde_json::to_string(step)?));
        }
        out.push_str("  ],\n");
        out.push_str("  \"events\": [\n");
        for (i, ev) in self.events.iter().enumerate() {
            let comma = if i + 1 < self.events.len() { "," } else { "" };
            out.push_str(&format!("    {}{comma}\n", serde_json::to_string(ev)?));
        }
        out.push_str("  ]\n}\n");
        Ok(out)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TranscriptDiff {
    pub event_index: usize,
    pub path: String,
    pub expected: String,
    pub actual: String,
}

#[derive(Debug, Default)]
struct Normalizer {
    epochs: HashMap<u64, String>,
    epoch_seq: Vec<u64>,
    generations: HashMap<u64, String>,
    action_ids: HashMap<u64, String>,
    stop_gens: HashMap<u64, String>,
    opaque_ids: HashMap<String, String>,
}

const FIXED_SEED_IDS: &[&str] = &[
    "01J00000000000000000000001",
    "01J00000000000000000000002",
    "01J00000000000000000000003",
    "shadow_demo",
];

impl Normalizer {
    fn norm_epoch(&mut self, ep: u64) -> String {
        if let Some(prev) = self.epoch_seq.last().copied() {
            if prev != ep {
                self.epoch_seq.push(ep);
            }
        } else {
            self.epoch_seq.push(ep);
        }
        let next = self.epochs.len();
        self.epochs
            .entry(ep)
            .or_insert_with(|| format!("$epoch_{next}"))
            .clone()
    }

    fn norm_gen(&mut self, g: u64) -> Value {
        if g == 0 {
            return Value::from(0u64);
        }
        let next = self.generations.len() + 1;
        Value::String(
            self.generations
                .entry(g)
                .or_insert_with(|| format!("$gen_{next}"))
                .clone(),
        )
    }

    fn norm_action_id(&mut self, id: u64) -> String {
        let next = self.action_ids.len() + 1;
        self.action_ids
            .entry(id)
            .or_insert_with(|| format!("$action_{next}"))
            .clone()
    }

    fn norm_stop_gen(&mut self, sg: u64) -> Value {
        if sg == 0 {
            return Value::from(0u64);
        }
        let next = self.stop_gens.len() + 1;
        Value::String(
            self.stop_gens
                .entry(sg)
                .or_insert_with(|| format!("$stop_gen_{next}"))
                .clone(),
        )
    }

    fn norm_opaque(&mut self, s: &str) -> String {
        if FIXED_SEED_IDS.contains(&s) {
            return s.to_string();
        }
        let next = self.opaque_ids.len() + 1;
        self.opaque_ids
            .entry(s.to_string())
            .or_insert_with(|| format!("$id_{next}"))
            .clone()
    }

    fn epoch_order_signature(&self) -> Vec<i8> {
        self.epoch_seq
            .windows(2)
            .map(|w| match w[1].cmp(&w[0]) {
                std::cmp::Ordering::Less => -1,
                std::cmp::Ordering::Equal => 0,
                std::cmp::Ordering::Greater => 1,
            })
            .collect()
    }

    fn normalize_value(&mut self, key: Option<&str>, parent_key: Option<&str>, val: &Value) -> Value {
        if let Some(k) = key {
            match k {
                "epoch_id" => {
                    if let Some(n) = val.as_u64() {
                        return Value::String(self.norm_epoch(n));
                    }
                }
                "generation" => {
                    if let Some(n) = val.as_u64() {
                        return self.norm_gen(n);
                    }
                }
                "action_id" => {
                    if let Some(n) = val.as_u64() {
                        return Value::String(self.norm_action_id(n));
                    }
                }
                "stop_generation" => {
                    if let Some(n) = val.as_u64() {
                        return self.norm_stop_gen(n);
                    }
                }
                "pose_id" | "recording_id" | "job_id" | "token" | "backup" | "content_sha256"
                | "deleted" => {
                    if let Some(s) = val.as_str() {
                        return Value::String(self.norm_opaque(s));
                    }
                }
                "expected_duration_s" => {
                    if let Some(n) = val.as_f64() {
                        if n.is_finite() {
                            return Value::from((n * 100.0).round() / 100.0);
                        }
                    }
                }
                "id" if matches!(parent_key, Some("pose" | "sequence" | "episode" | "body")) => {
                    if let Some(s) = val.as_str() {
                        return Value::String(self.norm_opaque(s));
                    }
                }
                "timestamp" | "started_at" | "finished_at" | "recorded_at" | "created_at"
                | "updated_at" | "saved_at_ms" | "ts" => {
                    if val.is_number() {
                        return Value::String("$ts_num".into());
                    }
                    if val.as_str().is_some_and(|s| !s.is_empty()) {
                        return Value::String("$ts_str".into());
                    }
                }
                "expires_in_s" | "max_fps" => {
                    if val.as_f64().is_some_and(|n| n.is_finite() && n > 0.0) {
                        return Value::String("$positive_f64".into());
                    }
                }
                "remaining_ms" => {
                    if val.as_u64().is_some_and(|n| n > 0) {
                        return Value::String("$positive_u64".into());
                    }
                }
                "warmup_infer_us" => {
                    if let Some(obj) = val.as_object() {
                        let mut out = Map::new();
                        for (wk, wv) in obj {
                            if wk == "n" || wk == "samples" || wk == "count" {
                                out.insert(wk.clone(), wv.clone());
                            } else if wv.is_number() {
                                out.insert(wk.clone(), Value::String("$timing_us".into()));
                            } else {
                                out.insert(wk.clone(), wv.clone());
                            }
                        }
                        return Value::Object(out);
                    }
                }
                "duration_s" | "frames" | "bytes" if parent_key == Some("episode") => {
                    if let Some(n) = val.as_f64() {
                        if n.is_finite() && n >= 0.0 {
                            return Value::String("$nonneg_num".into());
                        }
                    }
                }
                _ => {}
            }
        }
        match val {
            Value::Object(map) => {
                // Use BTreeMap so key iteration order is deterministic when populating tables.
                let sorted: BTreeMap<&String, &Value> = map.iter().collect();
                let mut out = Map::new();
                for (k, v) in sorted {
                    let nv = self.normalize_value(Some(k.as_str()), key, v);
                    out.insert(k.clone(), nv);
                }
                Value::Object(out)
            }
            Value::Array(arr) => Value::Array(
                arr.iter()
                    .map(|item| self.normalize_value(None, key, item))
                    .collect(),
            ),
            other => other.clone(),
        }
    }

    fn normalize_event(&mut self, ev: &RecordedEvent) -> Value {
        match ev {
            RecordedEvent::UpgradeStatus { client, status } => serde_json::json!({
                "ev": "upgrade_status",
                "client": client,
                "status": status,
            }),
            RecordedEvent::ClientFrame { client, frame } => serde_json::json!({
                "ev": "client_frame",
                "client": client,
                "frame": self.normalize_value(None, None, frame),
            }),
            RecordedEvent::ClientRaw { client, text } => serde_json::json!({
                "ev": "client_raw",
                "client": client,
                "text": text,
            }),
            RecordedEvent::ServerFrame { client, frame } => serde_json::json!({
                "ev": "server_frame",
                "client": client,
                "frame": self.normalize_value(None, None, frame),
            }),
            RecordedEvent::ActionTerminal {
                client,
                action_id,
                kind,
                state,
                code,
                safety_state,
                epoch_id,
                is_torqued,
                control_mode,
            } => {
                // On completion or STOPPED cancellation tick, `Moving` -> `Armed` can straddle
                // the terminal vs next 50 Hz tick; normalize only those two operational states
                // while keeping Fault/EStop/Idle and non-STOPPED cancellations strict.
                let operational_tick_boundary = state == "done"
                    || (state == "cancelled" && code.as_deref() == Some("STOPPED"));
                let norm_safety = if operational_tick_boundary
                    && matches!(safety_state.as_str(), Some("Moving" | "Armed"))
                {
                    Value::String("$operational".into())
                } else {
                    safety_state.clone()
                };
                serde_json::json!({
                    "ev": "action_terminal",
                    "client": client,
                    "action_id": self.norm_action_id(*action_id),
                    "kind": kind,
                    "state": state,
                    "code": code,
                    "safety_state": norm_safety,
                    "epoch_id": self.norm_epoch(*epoch_id),
                    "is_torqued": is_torqued,
                    "control_mode": control_mode,
                })
            }
            RecordedEvent::NoReply { client } => serde_json::json!({
                "ev": "no_reply",
                "client": client,
            }),
            RecordedEvent::ServerClose { client, close } => serde_json::json!({
                "ev": "server_close",
                "client": client,
                "code": close.code,
                "reason": close.reason,
            }),
            RecordedEvent::ClientClosed { client } => serde_json::json!({
                "ev": "client_closed",
                "client": client,
            }),
            RecordedEvent::RestResponse {
                method,
                path,
                status,
                body,
            } => serde_json::json!({
                "ev": "rest_response",
                "method": method,
                "path": path,
                "status": status,
                "body": self.normalize_value(None, None, body),
            }),
            RecordedEvent::TelemetrySummary {
                client,
                in_range,
                monotonic,
                dof,
                safety_state,
                epoch_id,
            } => serde_json::json!({
                "ev": "telemetry_summary",
                "client": client,
                "in_range": in_range,
                "monotonic": monotonic,
                "dof": dof,
                "safety_state": safety_state,
                "epoch_id": self.norm_epoch(*epoch_id),
            }),
            RecordedEvent::OwnerEvent { label, detail } => serde_json::json!({
                "ev": "owner_event",
                "label": label,
                "detail": self.normalize_value(None, None, detail),
            }),
        }
    }
}

fn diff_values(
    event_index: usize,
    path: &str,
    expected: &Value,
    actual: &Value,
    out: &mut Vec<TranscriptDiff>,
) {
    if expected == actual {
        return;
    }
    match (expected, actual) {
        (Value::Object(em), Value::Object(am)) => {
            let mut keys: Vec<&String> = em.keys().chain(am.keys()).collect();
            keys.sort();
            keys.dedup();
            for k in keys {
                let child = if path.is_empty() {
                    k.clone()
                } else {
                    format!("{path}.{k}")
                };
                match (em.get(k), am.get(k)) {
                    (Some(ev), Some(av)) => diff_values(event_index, &child, ev, av, out),
                    (Some(ev), None) => out.push(TranscriptDiff {
                        event_index,
                        path: child,
                        expected: ev.to_string(),
                        actual: "<missing>".into(),
                    }),
                    (None, Some(av)) => out.push(TranscriptDiff {
                        event_index,
                        path: child,
                        expected: "<missing>".into(),
                        actual: av.to_string(),
                    }),
                    (None, None) => {}
                }
            }
        }
        (Value::Array(ea), Value::Array(aa)) => {
            let max_len = ea.len().max(aa.len());
            for i in 0..max_len {
                let child = format!("{path}[{i}]");
                match (ea.get(i), aa.get(i)) {
                    (Some(ev), Some(av)) => diff_values(event_index, &child, ev, av, out),
                    (Some(ev), None) => out.push(TranscriptDiff {
                        event_index,
                        path: child,
                        expected: ev.to_string(),
                        actual: "<missing>".into(),
                    }),
                    (None, Some(av)) => out.push(TranscriptDiff {
                        event_index,
                        path: child,
                        expected: "<missing>".into(),
                        actual: av.to_string(),
                    }),
                    (None, None) => {}
                }
            }
        }
        _ => out.push(TranscriptDiff {
            event_index,
            path: path.to_string(),
            expected: expected.to_string(),
            actual: actual.to_string(),
        }),
    }
}

/// Compare two recorded transcripts strictly after equivalence-preserving normalization.
/// Returns an empty vector iff both transcripts are identical in normalized form and epoch order.
pub fn compare_transcripts(
    expected: &SessionTranscript,
    actual: &SessionTranscript,
) -> Vec<TranscriptDiff> {
    let mut norm_exp = Normalizer::default();
    let mut norm_act = Normalizer::default();
    let exp_events: Vec<Value> = expected
        .events
        .iter()
        .map(|ev| norm_exp.normalize_event(ev))
        .collect();
    let act_events: Vec<Value> = actual
        .events
        .iter()
        .map(|ev| norm_act.normalize_event(ev))
        .collect();

    let mut diffs = Vec::new();
    let max_len = exp_events.len().max(act_events.len());
    for i in 0..max_len {
        match (exp_events.get(i), act_events.get(i)) {
            (Some(e), Some(a)) => diff_values(i, "", e, a, &mut diffs),
            (Some(e), None) => diffs.push(TranscriptDiff {
                event_index: i,
                path: "<event>".into(),
                expected: e.to_string(),
                actual: "<missing>".into(),
            }),
            (None, Some(a)) => diffs.push(TranscriptDiff {
                event_index: i,
                path: "<event>".into(),
                expected: "<missing>".into(),
                actual: a.to_string(),
            }),
            (None, None) => {}
        }
    }

    let exp_order = norm_exp.epoch_order_signature();
    let act_order = norm_act.epoch_order_signature();
    if exp_order != act_order {
        diffs.push(TranscriptDiff {
            event_index: max_len,
            path: "<epoch_order>".into(),
            expected: format!("{exp_order:?}"),
            actual: format!("{act_order:?}"),
        });
    }

    diffs
}
