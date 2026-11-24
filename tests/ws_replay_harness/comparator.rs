//! Transcript schema and equivalence-preserving comparator for WS replay parity.
//!
//! Normalizes ONLY genuinely nondeterministic values (wall timestamps, scoped ephemeral
//! timings/cooldowns, opaque generated IDs, epochs, lease generations, stop generations,
//! action IDs) into a type-safe [`NormValue`] domain so wire JSON strings can never collide
//! with normalized numeric sentinels. Preserves bijective equality AND total relative order
//! (`a < b <=> f(a) < f(b)`) across every numeric identifier domain.

use std::collections::{BTreeMap, HashMap};
use serde::{Deserialize, Serialize};
use serde_json::Value;

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
    /// Send an explicit WebSocket Close control frame without tearing down the TCP socket yet.
    SendCloseFrame {
        client: String,
        code: u16,
        #[serde(default)]
        reason: String,
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
    /// Wait for server-initiated WS Close (recording any preceding control frames in arrival order).
    ExpectClose {
        client: String,
    },
    /// Client drains any unexpected unread frames briefly and closes its WS connection.
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
        /// Whether to record the full response body (default true; false records status + presence).
        #[serde(default = "yes")]
        record_body: bool,
    },
    /// Controlled wait (e.g. lease timeout expiry or recording window).
    SleepMs {
        ms: u64,
    },
    /// Collect telemetry observations for `duration_ms` and record rate/payload summary.
    CollectTelemetry {
        client: String,
        duration_ms: u64,
        min_count: usize,
        max_count: usize,
    },
    /// Refresh `{{epoch}}` from `GET /api/v1/observation` without recording a REST event.
    SyncEpoch,
    /// Read-only `GET /api/v1/observation` probe recording steady-state safety/action fields
    /// without sending any mutating command.
    CheckState,
}

fn one() -> usize {
    1
}
fn yes() -> bool {
    true
}

/// Exact WebSocket close outcome distinguishing a Close handshake frame from bare EOF or reset.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(tag = "outcome", rename_all = "snake_case")]
pub enum CloseOutcome {
    Frame {
        code: Option<u16>,
        reason: Option<String>,
    },
    Eof,
    TransportError,
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
    ClientCloseFrame {
        client: String,
        code: u16,
        reason: String,
    },
    ServerFrame {
        client: String,
        frame: Value,
    },
    ActionTerminal {
        client: String,
        action: Value,
        safety_state: Value,
        epoch_id: u64,
        is_torqued: bool,
        control_mode: String,
    },
    StateSnapshot {
        safety_state: Value,
        is_torqued: bool,
        control_mode: String,
        epoch_id: u64,
        active_action: Option<String>,
    },
    NoReply {
        client: String,
    },
    ServerClose {
        client: String,
        close: CloseOutcome,
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
        has_all_timestamps: bool,
        monotonic: bool,
        all_samples_consistent: bool,
        dof: usize,
        is_torqued: bool,
        control_mode: String,
        positions_mrad: Vec<i32>,
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

/// Type-safe normalized comparison domain. Normalized sentinels live in distinct enum variants
/// so no literal wire JSON string (e.g. `"$gen_1"`) can ever compare equal to a numeric token.
#[derive(Debug, Clone, PartialEq, Eq)]
enum NormValue {
    Null,
    Bool(bool),
    I64(i64),
    U64(u64),
    /// Canonical finite f64 bits (`-0.0` canonicalized to `0.0`).
    F64Bits(u64),
    Str(String),
    Array(Vec<NormValue>),
    Object(BTreeMap<String, NormValue>),
    NormEpoch(usize),
    NormGen(usize),
    NormActionId(usize),
    NormStopGen(usize),
    NormOpaqueId(usize),
    NormTsNum,
    NormTsStr,
    NormTimingUs,
    NormPositiveF64,
    NormPositiveU64,
    NormNonNegF64,
    NormPartialProgress,
    NormOperationalState,
}

impl NormValue {
    fn from_f64(n: f64) -> Self {
        let canon = if n == 0.0 { 0.0 } else { n };
        Self::F64Bits(canon.to_bits())
    }

    fn describe(&self) -> String {
        match self {
            Self::Null => "null".into(),
            Self::Bool(b) => b.to_string(),
            Self::I64(i) => i.to_string(),
            Self::U64(u) => u.to_string(),
            Self::F64Bits(bits) => format!("{}", f64::from_bits(*bits)),
            Self::Str(s) => format!("{s:?}"),
            Self::NormEpoch(n) => format!("<epoch#{n}>"),
            Self::NormGen(n) => format!("<gen#{n}>"),
            Self::NormActionId(n) => format!("<action#{n}>"),
            Self::NormStopGen(n) => format!("<stop_gen#{n}>"),
            Self::NormOpaqueId(n) => format!("<opaque#{n}>"),
            Self::NormTsNum => "<ts:num>".into(),
            Self::NormTsStr => "<ts:str>".into(),
            Self::NormTimingUs => "<timing:us>".into(),
            Self::NormPositiveF64 => "<pos:f64>".into(),
            Self::NormPositiveU64 => "<pos:u64>".into(),
            Self::NormNonNegF64 => "<nonneg:f64>".into(),
            Self::NormPartialProgress => "<progress:partial>".into(),
            Self::NormOperationalState => "<state:operational>".into(),
            Self::Array(a) => format!(
                "[{}]",
                a.iter().map(Self::describe).collect::<Vec<_>>().join(", ")
            ),
            Self::Object(m) => format!(
                "{{{}}}",
                m.iter()
                    .map(|(k, v)| format!("{k}: {}", v.describe()))
                    .collect::<Vec<_>>()
                    .join(", ")
            ),
        }
    }
}

/// First-appearance bijective table plus raw-value list for total relative-order checking.
#[derive(Debug, Default)]
struct NumericDomain {
    by_value: HashMap<u64, usize>,
    first_seen: Vec<u64>,
}

impl NumericDomain {
    fn intern(&mut self, v: u64) -> usize {
        if let Some(&id) = self.by_value.get(&v) {
            return id;
        }
        let id = self.first_seen.len() + 1;
        self.first_seen.push(v);
        self.by_value.insert(v, id);
        id
    }

    /// Returns the sorted rank (`0..k`) of each unique value in first-appearance order.
    /// Two sequences have identical rank vectors iff the first-appearance bijection is strictly
    /// order-preserving (`a < b <=> map(a) < map(b)` across ALL pairs, not just adjacent ones).
    fn rank_signature(&self) -> Vec<usize> {
        let mut sorted = self.first_seen.clone();
        sorted.sort_unstable();
        self.first_seen
            .iter()
            .map(|v| sorted.binary_search(v).unwrap())
            .collect()
    }
}

#[derive(Debug, Default)]
struct Normalizer {
    epochs: NumericDomain,
    generations: NumericDomain,
    action_ids: NumericDomain,
    stop_gens: NumericDomain,
    opaque_ids: HashMap<String, usize>,
}

const FIXED_SEED_IDS: &[&str] = &[
    "01J00000000000000000000001",
    "01J00000000000000000000002",
    "01J00000000000000000000003",
    "shadow_demo",
];

#[derive(Debug, Clone, Copy, Default)]
struct ObjectContext {
    is_command_ack: bool,
    is_policy_start_ack: bool,
    is_preemption_cooldown: bool,
}

impl ObjectContext {
    fn from_map(map: &serde_json::Map<String, Value>) -> Self {
        let msg_type = map.get("type").and_then(Value::as_str);
        let is_command_ack = msg_type == Some("command_ack");
        let is_policy_start_ack =
            is_command_ack && map.contains_key("policy_id") && map.contains_key("warmup_infer_us");
        let is_preemption_cooldown = msg_type == Some("lease_rejected")
            && map.get("error").and_then(Value::as_str) == Some("preemption_cooldown");
        Self {
            is_command_ack,
            is_policy_start_ack,
            is_preemption_cooldown,
        }
    }
}

impl Normalizer {
    fn norm_epoch(&mut self, ep: u64) -> NormValue {
        if ep == 0 {
            NormValue::U64(0)
        } else {
            NormValue::NormEpoch(self.epochs.intern(ep))
        }
    }

    fn norm_gen(&mut self, g: u64) -> NormValue {
        if g == 0 {
            NormValue::U64(0)
        } else {
            NormValue::NormGen(self.generations.intern(g))
        }
    }

    fn norm_action_id(&mut self, id: u64) -> NormValue {
        if id == 0 {
            NormValue::U64(0)
        } else {
            NormValue::NormActionId(self.action_ids.intern(id))
        }
    }

    fn norm_stop_gen(&mut self, sg: u64) -> NormValue {
        if sg == 0 {
            NormValue::U64(0)
        } else {
            NormValue::NormStopGen(self.stop_gens.intern(sg))
        }
    }

    fn norm_opaque(&mut self, s: &str) -> NormValue {
        if FIXED_SEED_IDS.contains(&s) {
            return NormValue::Str(s.to_string());
        }
        let next = self.opaque_ids.len() + 1;
        let id = *self.opaque_ids.entry(s.to_string()).or_insert(next);
        NormValue::NormOpaqueId(id)
    }

    fn normalize_plain_value(&mut self, val: &Value) -> NormValue {
        match val {
            Value::Null => NormValue::Null,
            Value::Bool(b) => NormValue::Bool(*b),
            Value::Number(n) => {
                if let Some(u) = n.as_u64() {
                    NormValue::U64(u)
                } else if let Some(i) = n.as_i64() {
                    NormValue::I64(i)
                } else if let Some(f) = n.as_f64() {
                    NormValue::from_f64(f)
                } else {
                    NormValue::Str(n.to_string())
                }
            }
            Value::String(s) => NormValue::Str(s.clone()),
            Value::Array(arr) => NormValue::Array(
                arr.iter()
                    .map(|item| self.normalize_value(None, None, ObjectContext::default(), item))
                    .collect(),
            ),
            Value::Object(map) => {
                let ctx = ObjectContext::from_map(map);
                let sorted: BTreeMap<&String, &Value> = map.iter().collect();
                let mut out = BTreeMap::new();
                for (k, v) in sorted {
                    let nv = self.normalize_value(Some(k.as_str()), None, ctx, v);
                    out.insert(k.clone(), nv);
                }
                NormValue::Object(out)
            }
        }
    }

    fn normalize_value(
        &mut self,
        key: Option<&str>,
        parent_key: Option<&str>,
        parent_ctx: ObjectContext,
        val: &Value,
    ) -> NormValue {
        if let Some(k) = key {
            match k {
                "epoch_id" => {
                    if let Some(n) = val.as_u64() {
                        return self.norm_epoch(n);
                    }
                }
                "generation" => {
                    if let Some(n) = val.as_u64() {
                        return self.norm_gen(n);
                    }
                }
                "action_id" => {
                    if let Some(n) = val.as_u64() {
                        return self.norm_action_id(n);
                    }
                }
                "stop_generation" => {
                    if let Some(n) = val.as_u64() {
                        return self.norm_stop_gen(n);
                    }
                }
                "pose_id" | "sequence_id" | "recording_id" | "job_id" | "token" | "backup"
                | "content_sha256" | "deleted" => {
                    if let Some(s) = val.as_str() {
                        return self.norm_opaque(s);
                    }
                }
                "gripper_gap_rad" if parent_key == Some("action") => {
                    // Mock gripper settling jitters by ~1e-6 rad; quantize to 1 mrad so real gap
                    // changes (>= 0.001 rad) still fail.
                    if let Some(n) = val.as_f64() {
                        if n.is_finite() {
                            return NormValue::from_f64((n * 1000.0).round() / 1000.0);
                        }
                    }
                }
                "phase_ms" if parent_key == Some("action") => {
                    if let Some(obj) = val.as_object() {
                        let mut out = BTreeMap::new();
                        for (pk, pv) in obj {
                            if pv.as_u64().is_some_and(|n| n > 0) {
                                out.insert(pk.clone(), NormValue::NormPositiveU64);
                            } else {
                                out.insert(pk.clone(), self.normalize_plain_value(pv));
                            }
                        }
                        return NormValue::Object(out);
                    }
                }
                "infer_us" if parent_key == Some("policy_stats") => {
                    if let Some(obj) = val.as_object() {
                        let mut out = BTreeMap::new();
                        for (wk, wv) in obj {
                            if wk == "n" || wk == "samples" || wk == "count" {
                                out.insert(wk.clone(), self.normalize_plain_value(wv));
                            } else if wv.is_number() {
                                out.insert(wk.clone(), NormValue::NormTimingUs);
                            } else {
                                out.insert(wk.clone(), self.normalize_plain_value(wv));
                            }
                        }
                        return NormValue::Object(out);
                    }
                }
                "steps"
                    if parent_key == Some("policy_stats")
                        && val.as_u64().is_some_and(|n| n > 0) =>
                {
                    return NormValue::NormPositiveU64;
                }
                "expected_duration_s" if parent_ctx.is_command_ack => {
                    // Mock joint settling varies by ~3 us on Cartesian planners; quantize to 1 ms
                    // strictly on `command_ack` so any >= 1 ms drift fails.
                    if let Some(n) = val.as_f64() {
                        if n.is_finite() {
                            return NormValue::from_f64((n * 1000.0).round() / 1000.0);
                        }
                    }
                }
                "id" if matches!(parent_key, Some("pose" | "sequence" | "episode" | "body")) => {
                    if let Some(s) = val.as_str() {
                        return self.norm_opaque(s);
                    }
                }
                "timestamp" | "started_at" | "finished_at" | "recorded_at" | "created_at"
                | "updated_at" | "saved_at_ms" | "ts" => {
                    if val.is_number() {
                        return NormValue::NormTsNum;
                    }
                    if val.as_str().is_some_and(|s| !s.is_empty()) {
                        return NormValue::NormTsStr;
                    }
                }
                "max_fps" if parent_ctx.is_policy_start_ack => {
                    if val.as_f64().is_some_and(|n| n.is_finite() && n > 0.0) {
                        return NormValue::NormPositiveF64;
                    }
                }
                "remaining_ms" if parent_ctx.is_preemption_cooldown => {
                    if val.as_u64().is_some_and(|n| n > 0) {
                        return NormValue::NormPositiveU64;
                    }
                }
                "warmup_infer_us" if parent_ctx.is_policy_start_ack => {
                    if let Some(obj) = val.as_object() {
                        let mut out = BTreeMap::new();
                        for (wk, wv) in obj {
                            if wk == "n" || wk == "samples" || wk == "count" {
                                out.insert(wk.clone(), self.normalize_plain_value(wv));
                            } else if wv.is_number() {
                                out.insert(wk.clone(), NormValue::NormTimingUs);
                            } else {
                                out.insert(wk.clone(), self.normalize_plain_value(wv));
                            }
                        }
                        return NormValue::Object(out);
                    }
                }
                "duration_s" if parent_key == Some("episode") => {
                    if let Some(n) = val.as_f64() {
                        if n.is_finite() && n >= 0.0 {
                            return NormValue::NormNonNegF64;
                        }
                    }
                }
                "frames" | "bytes"
                    if parent_key == Some("episode") && val.as_u64().is_some_and(|n| n > 0) =>
                {
                    // Preserve integer JSON type and reject zero counts on recorded episodes.
                    return NormValue::NormPositiveU64;
                }
                _ => {}
            }
        }
        match val {
            Value::Object(map) => {
                let ctx = ObjectContext::from_map(map);
                let sorted: BTreeMap<&String, &Value> = map.iter().collect();
                let mut out = BTreeMap::new();
                for (k, v) in sorted {
                    let nv = self.normalize_value(Some(k.as_str()), key, ctx, v);
                    out.insert(k.clone(), nv);
                }
                NormValue::Object(out)
            }
            Value::Array(arr) => NormValue::Array(
                arr.iter()
                    .map(|item| self.normalize_value(None, key, parent_ctx, item))
                    .collect(),
            ),
            other => self.normalize_plain_value(other),
        }
    }

    fn normalize_event(&mut self, ev: &RecordedEvent) -> NormValue {
        let mut m = BTreeMap::new();
        match ev {
            RecordedEvent::UpgradeStatus { client, status } => {
                m.insert("ev".into(), NormValue::Str("upgrade_status".into()));
                m.insert("client".into(), NormValue::Str(client.clone()));
                m.insert("status".into(), NormValue::U64(u64::from(*status)));
            }
            RecordedEvent::ClientFrame { client, frame } => {
                m.insert("ev".into(), NormValue::Str("client_frame".into()));
                m.insert("client".into(), NormValue::Str(client.clone()));
                m.insert(
                    "frame".into(),
                    self.normalize_value(None, None, ObjectContext::default(), frame),
                );
            }
            RecordedEvent::ClientRaw { client, text } => {
                m.insert("ev".into(), NormValue::Str("client_raw".into()));
                m.insert("client".into(), NormValue::Str(client.clone()));
                m.insert("text".into(), NormValue::Str(text.clone()));
            }
            RecordedEvent::ClientCloseFrame {
                client,
                code,
                reason,
            } => {
                m.insert("ev".into(), NormValue::Str("client_close_frame".into()));
                m.insert("client".into(), NormValue::Str(client.clone()));
                m.insert("code".into(), NormValue::U64(u64::from(*code)));
                m.insert("reason".into(), NormValue::Str(reason.clone()));
            }
            RecordedEvent::ServerFrame { client, frame } => {
                m.insert("ev".into(), NormValue::Str("server_frame".into()));
                m.insert("client".into(), NormValue::Str(client.clone()));
                m.insert(
                    "frame".into(),
                    self.normalize_value(None, None, ObjectContext::default(), frame),
                );
            }
            RecordedEvent::ActionTerminal {
                client,
                action,
                safety_state,
                epoch_id,
                is_torqued,
                control_mode,
            } => {
                let state = action.get("state").and_then(Value::as_str).unwrap_or("");
                let code = action.get("code").and_then(Value::as_str);
                // On completion or operational cancellation (`STOPPED`, `SUPERSEDED`, `LEASE_LOST`),
                // `Moving` -> `Armed` can straddle a single 50 Hz tick boundary; normalize only
                // those two operational states while keeping Fault/EStop/Idle and fault/ESTOP
                // cancellations strict. Post-stop steady-state `Armed` is verified verbatim by
                // read-only `StateSnapshot` probes.
                let operational_tick_boundary = state == "done"
                    || (state == "cancelled"
                        && matches!(
                            code,
                            Some("STOPPED" | "SUPERSEDED" | "LEASE_LOST")
                        ));
                let norm_safety = if operational_tick_boundary
                    && matches!(safety_state.as_str(), Some("Moving" | "Armed"))
                {
                    NormValue::NormOperationalState
                } else {
                    self.normalize_plain_value(safety_state)
                };
                let norm_action = match action {
                    Value::Object(map) => {
                        let sorted: BTreeMap<&String, &Value> = map.iter().collect();
                        let mut out = BTreeMap::new();
                        for (k, v) in sorted {
                            let nv = match k.as_str() {
                                "id" => match v.as_u64() {
                                    Some(id) => self.norm_action_id(id),
                                    None => self.normalize_plain_value(v),
                                },
                                // Partial progress on cancelled/failed actions varies with
                                // interruption wall time; keep `done` progress exact (1.0).
                                "progress" if state != "done" => {
                                    if v.as_f64().is_some_and(|p| p.is_finite() && (0.0..=1.0).contains(&p)) {
                                        NormValue::NormPartialProgress
                                    } else {
                                        self.normalize_plain_value(v)
                                    }
                                }
                                _ => self.normalize_value(
                                    Some(k.as_str()),
                                    Some("action"),
                                    ObjectContext::default(),
                                    v,
                                ),
                            };
                            out.insert(k.clone(), nv);
                        }
                        NormValue::Object(out)
                    }
                    other => self.normalize_plain_value(other),
                };
                m.insert("ev".into(), NormValue::Str("action_terminal".into()));
                m.insert("client".into(), NormValue::Str(client.clone()));
                m.insert("action".into(), norm_action);
                m.insert("safety_state".into(), norm_safety);
                m.insert("epoch_id".into(), self.norm_epoch(*epoch_id));
                m.insert("is_torqued".into(), NormValue::Bool(*is_torqued));
                m.insert("control_mode".into(), NormValue::Str(control_mode.clone()));
            }
            RecordedEvent::StateSnapshot {
                safety_state,
                is_torqued,
                control_mode,
                epoch_id,
                active_action,
            } => {
                m.insert("ev".into(), NormValue::Str("state_snapshot".into()));
                m.insert(
                    "safety_state".into(),
                    self.normalize_plain_value(safety_state),
                );
                m.insert("is_torqued".into(), NormValue::Bool(*is_torqued));
                m.insert("control_mode".into(), NormValue::Str(control_mode.clone()));
                m.insert("epoch_id".into(), self.norm_epoch(*epoch_id));
                m.insert(
                    "active_action".into(),
                    active_action
                        .as_ref()
                        .map_or(NormValue::Null, |s| NormValue::Str(s.clone())),
                );
            }
            RecordedEvent::NoReply { client } => {
                m.insert("ev".into(), NormValue::Str("no_reply".into()));
                m.insert("client".into(), NormValue::Str(client.clone()));
            }
            RecordedEvent::ServerClose { client, close } => {
                m.insert("ev".into(), NormValue::Str("server_close".into()));
                m.insert("client".into(), NormValue::Str(client.clone()));
                match close {
                    CloseOutcome::Frame { code, reason } => {
                        m.insert("outcome".into(), NormValue::Str("frame".into()));
                        m.insert(
                            "code".into(),
                            code.map_or(NormValue::Null, |c| NormValue::U64(u64::from(c))),
                        );
                        m.insert(
                            "reason".into(),
                            reason
                                .as_ref()
                                .map_or(NormValue::Null, |r| NormValue::Str(r.clone())),
                        );
                    }
                    CloseOutcome::Eof => {
                        m.insert("outcome".into(), NormValue::Str("eof".into()));
                    }
                    CloseOutcome::TransportError => {
                        m.insert("outcome".into(), NormValue::Str("transport_error".into()));
                    }
                }
            }
            RecordedEvent::ClientClosed { client } => {
                m.insert("ev".into(), NormValue::Str("client_closed".into()));
                m.insert("client".into(), NormValue::Str(client.clone()));
            }
            RecordedEvent::RestResponse {
                method,
                path,
                status,
                body,
            } => {
                m.insert("ev".into(), NormValue::Str("rest_response".into()));
                m.insert("method".into(), NormValue::Str(method.clone()));
                m.insert("path".into(), NormValue::Str(path.clone()));
                m.insert("status".into(), NormValue::U64(u64::from(*status)));
                m.insert(
                    "body".into(),
                    self.normalize_value(None, None, ObjectContext::default(), body),
                );
            }
            RecordedEvent::TelemetrySummary {
                client,
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
            } => {
                m.insert("ev".into(), NormValue::Str("telemetry_summary".into()));
                m.insert("client".into(), NormValue::Str(client.clone()));
                m.insert("in_range".into(), NormValue::Bool(*in_range));
                m.insert(
                    "has_all_timestamps".into(),
                    NormValue::Bool(*has_all_timestamps),
                );
                m.insert("monotonic".into(), NormValue::Bool(*monotonic));
                m.insert(
                    "all_samples_consistent".into(),
                    NormValue::Bool(*all_samples_consistent),
                );
                m.insert("dof".into(), NormValue::U64(*dof as u64));
                m.insert("is_torqued".into(), NormValue::Bool(*is_torqued));
                m.insert("control_mode".into(), NormValue::Str(control_mode.clone()));
                m.insert(
                    "positions_mrad".into(),
                    NormValue::Array(
                        positions_mrad
                            .iter()
                            .map(|&p| NormValue::I64(i64::from(p)))
                            .collect(),
                    ),
                );
                m.insert(
                    "safety_state".into(),
                    self.normalize_plain_value(safety_state),
                );
                m.insert("epoch_id".into(), self.norm_epoch(*epoch_id));
            }
            RecordedEvent::OwnerEvent { label, detail } => {
                m.insert("ev".into(), NormValue::Str("owner_event".into()));
                m.insert("label".into(), NormValue::Str(label.clone()));
                m.insert(
                    "detail".into(),
                    self.normalize_value(None, None, ObjectContext::default(), detail),
                );
            }
        }
        NormValue::Object(m)
    }
}

fn diff_values(
    idx: usize,
    path: &str,
    expected: &NormValue,
    actual: &NormValue,
    out: &mut Vec<TranscriptDiff>,
) {
    if expected == actual {
        return;
    }
    match (expected, actual) {
        (NormValue::Object(em), NormValue::Object(am)) => {
            for (k, ev) in em {
                let p = if path.is_empty() {
                    k.clone()
                } else {
                    format!("{path}.{k}")
                };
                match am.get(k) {
                    Some(av) => diff_values(idx, &p, ev, av, out),
                    None => out.push(TranscriptDiff {
                        event_index: idx,
                        path: p,
                        expected: ev.describe(),
                        actual: "<missing>".into(),
                    }),
                }
            }
            for (k, av) in am {
                if !em.contains_key(k) {
                    let p = if path.is_empty() {
                        k.clone()
                    } else {
                        format!("{path}.{k}")
                    };
                    out.push(TranscriptDiff {
                        event_index: idx,
                        path: p,
                        expected: "<missing>".into(),
                        actual: av.describe(),
                    });
                }
            }
        }
        (NormValue::Array(ea), NormValue::Array(aa)) if ea.len() == aa.len() => {
            for (i, (ev, av)) in ea.iter().zip(aa.iter()).enumerate() {
                diff_values(idx, &format!("{path}[{i}]"), ev, av, out);
            }
        }
        _ => out.push(TranscriptDiff {
            event_index: idx,
            path: path.to_string(),
            expected: expected.describe(),
            actual: actual.describe(),
        }),
    }
}

/// Equivalence-preserving comparison of two transcripts.
/// Returns an empty `Vec` iff every event kind, payload, error code, ordering, close outcome,
/// bijective ID relation, and total numeric order relation across epochs/generations/actions/stops match.
pub fn compare_transcripts(
    expected: &SessionTranscript,
    actual: &SessionTranscript,
) -> Vec<TranscriptDiff> {
    let mut norm_e = Normalizer::default();
    let mut norm_a = Normalizer::default();
    let mut diffs = Vec::new();

    let common = expected.events.len().min(actual.events.len());
    for i in 0..common {
        let ve = norm_e.normalize_event(&expected.events[i]);
        let va = norm_a.normalize_event(&actual.events[i]);
        diff_values(i, "", &ve, &va, &mut diffs);
    }
    if expected.events.len() != actual.events.len() {
        diffs.push(TranscriptDiff {
            event_index: common,
            path: "<events.len>".into(),
            expected: expected.events.len().to_string(),
            actual: actual.events.len().to_string(),
        });
    }
    for (name, re, ra) in [
        (
            "<epoch_total_order>",
            norm_e.epochs.rank_signature(),
            norm_a.epochs.rank_signature(),
        ),
        (
            "<generation_total_order>",
            norm_e.generations.rank_signature(),
            norm_a.generations.rank_signature(),
        ),
        (
            "<action_id_total_order>",
            norm_e.action_ids.rank_signature(),
            norm_a.action_ids.rank_signature(),
        ),
        (
            "<stop_generation_total_order>",
            norm_e.stop_gens.rank_signature(),
            norm_a.stop_gens.rank_signature(),
        ),
    ] {
        if re != ra {
            diffs.push(TranscriptDiff {
                event_index: common,
                path: name.into(),
                expected: format!("{re:?}"),
                actual: format!("{ra:?}"),
            });
        }
    }
    diffs
}
