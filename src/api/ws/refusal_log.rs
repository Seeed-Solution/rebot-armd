//! Log of refusal replies (`command_reject`, `preset_reject`, `servo_reject`) the writer task
//! sends to a WebSocket client.
//!
//! The writer calls [`note`] with the structured reply right before it serializes it, in the
//! ordinary writer branch and in the urgent branches (including the urgent drain at shutdown).
//! The call is independent of the outbound limiter, and it runs on the connection's writer task,
//! never on an RT thread. Bytes on the wire are not changed.
//!
//! Per connection and per canonical code: the first refusal is logged at once; further ones inside
//! [`REFUSAL_LOG_INTERVAL`] are counted, and the next line after the interval carries the count.

use std::collections::HashMap;
use std::sync::{LazyLock, Mutex};
use std::time::{Duration, Instant};

use serde_json::Value;

/// Minimum time between two log lines for one code on one connection.
pub const REFUSAL_LOG_INTERVAL: Duration = Duration::from_secs(5);
/// Codes tracked per connection; further codes share the key `OTHER`.
const MAX_CODES_PER_CONNECTION: usize = 64;
/// Log target of the lines, for filters and tests.
pub const REFUSAL_LOG_TARGET: &str = "armd::ws_refusal";

/// One line to log.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RefusalLine {
    /// `command_reject`, `preset_reject` or `servo_reject`.
    pub kind: String,
    /// Canonical code: `code`, else `error`, else `reason` of the reply.
    pub code: String,
    /// HTTP-style status of the reply, when it has one.
    pub status: Option<u64>,
    /// `request_id` when it is an unsigned integer.
    pub request_id: Option<u64>,
    /// Refusals with this code since the previous line, not logged.
    pub suppressed: u64,
}

impl RefusalLine {
    /// `STOPPED` (a message cancelled by a stop the client asked for) is logged at DEBUG.
    pub fn is_debug(&self) -> bool {
        self.code == "STOPPED"
    }
}

/// Rate-limit state of one connection.
#[derive(Debug, Default)]
pub struct RefusalLog {
    codes: HashMap<String, (Instant, u64)>,
}

impl RefusalLog {
    /// The line to log for `reply` at `now`, or `None` when it is not a refusal or is suppressed.
    pub fn observe(&mut self, now: Instant, reply: &Value) -> Option<RefusalLine> {
        let kind = reply.get("type")?.as_str()?;
        if !matches!(kind, "command_reject" | "preset_reject" | "servo_reject") {
            return None;
        }
        let code = ["code", "error", "reason"]
            .iter()
            .find_map(|k| reply.get(k).and_then(Value::as_str).filter(|s| !s.is_empty()))
            .unwrap_or("UNKNOWN");
        let key = if self.codes.contains_key(code) || self.codes.len() < MAX_CODES_PER_CONNECTION { code } else { "OTHER" };
        let suppressed = match self.codes.get_mut(key) {
            None => {
                self.codes.insert(key.to_owned(), (now, 0));
                0
            }
            Some((last, count)) => {
                if now.saturating_duration_since(*last) < REFUSAL_LOG_INTERVAL {
                    *count += 1;
                    return None;
                }
                let n = *count;
                *last = now;
                *count = 0;
                n
            }
        };
        Some(RefusalLine {
            kind: kind.to_owned(),
            code: key.to_owned(),
            status: reply.get("status").and_then(Value::as_u64),
            request_id: reply.get("request_id").and_then(Value::as_u64),
            suppressed,
        })
    }
}

static REGISTRY: LazyLock<Mutex<HashMap<String, RefusalLog>>> = LazyLock::new(|| Mutex::new(HashMap::new()));

/// Drops the connection's state when its writer task ends.
pub struct RefusalLogGuard(String);

impl RefusalLogGuard {
    pub fn new(connection: &str) -> Self {
        RefusalLogGuard(connection.to_owned())
    }
}

impl Drop for RefusalLogGuard {
    fn drop(&mut self) {
        REGISTRY.lock().unwrap_or_else(|e| e.into_inner()).remove(&self.0);
    }
}

/// Logs `reply` for `connection` when it is a refusal and not suppressed.
pub fn note(connection: &str, reply: &Value) {
    note_at(connection, reply, Instant::now());
}

#[doc(hidden)]
pub fn note_at(connection: &str, reply: &Value, now: Instant) {
    if reply.get("type").and_then(Value::as_str).is_none_or(|t| !t.ends_with("_reject")) {
        return;
    }
    let line = REGISTRY
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .entry(connection.to_owned())
        .or_default()
        .observe(now, reply);
    if let Some(l) = line {
        emit(connection, &l);
    }
}

fn emit(connection: &str, l: &RefusalLine) {
    let status = l.status.map_or_else(|| "-".to_owned(), |s| s.to_string());
    let request_id = l.request_id.map_or_else(|| "-".to_owned(), |s| s.to_string());
    macro_rules! line {
        ($lvl:ident) => {
            tracing::$lvl!(
                target: REFUSAL_LOG_TARGET,
                connection = connection, reply_type = l.kind.as_str(), code = l.code.as_str(),
                status = status.as_str(), request_id = request_id.as_str(), suppressed = l.suppressed,
                "WS {} refused on {}: code={} status={} request_id={} suppressed_since_last={}",
                l.kind, connection, l.code, status, request_id, l.suppressed
            )
        };
    }
    if l.is_debug() { line!(debug) } else { line!(warn) }
}
