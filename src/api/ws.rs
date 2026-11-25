use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, LazyLock, Mutex};
use std::time::{Duration, Instant};
use axum::{extract::{ws::{CloseFrame, Message, WebSocket, WebSocketUpgrade}, State}, response::IntoResponse};
use futures_util::{SinkExt, StreamExt};
use serde_json::{json, Value};
use tokio::sync::mpsc;
use crate::supervisor::{ArmError, ArmSupervisor, SupervisorInner};
use crate::motion::PlanInput;
use crate::safety::SafetyError;
use crate::api::rest::ApiError;
use axum::http::StatusCode;

// ws leaf modules (armd 1.6 RT-core foundation)
mod lease;
#[allow(unused_imports)]
use lease::*;
mod motion;
#[allow(unused_imports)]
use motion::*;
mod actions;
#[allow(unused_imports)]
use actions::*;
mod policy;
#[allow(unused_imports)]
use policy::*;
mod calibration;
#[allow(unused_imports)]
use calibration::*;
mod config;
#[allow(unused_imports)]
use config::*;
mod servo;
#[allow(unused_imports)]
use servo::*;
mod dispatch;
use dispatch::{
    AdmissionClass, AuthRequirement, DispatchHandler, FrameKind, LeaseFreeCtx, LeaseRequirement,
    OwnerDispatchCtx, ReplyPolicy, UrgentEStopCtx, UrgentStopCtx,
};

/// Initialises the WS dispatch tables ahead of the first connection (see
/// `dispatch::prewarm_tables`).
pub fn prewarm_dispatch_tables() {
    dispatch::prewarm_tables();
}

pub type ConnectionRegistry = Arc<Mutex<HashMap<String, mpsc::Sender<Message>>>>;

static CONNECTION_REGISTRY: LazyLock<ConnectionRegistry> =
    LazyLock::new(|| Arc::new(Mutex::new(HashMap::new())));

type FatalCloserEntry = (mpsc::Sender<FatalClose>, Arc<AtomicBool>, Arc<tokio::sync::Notify>);

static CONNECTION_CLOSERS: LazyLock<Mutex<HashMap<String, FatalCloserEntry>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));

pub fn connection_registry() -> ConnectionRegistry {
    CONNECTION_REGISTRY.clone()
}

/// Delivers a post-publication `lease_lost(reason: "preempted")` notice to an evicted WS socket
/// (called by `armd-dispatch` in executor mode and by `worker_task` in legacy mode).
pub(crate) fn notify_evicted_connection(evicted_conn: &str, evicted_gen: u64) {
    lease::notify_evicted(evicted_conn, evicted_gen);
}

/// Test hook: pause (ms) between stamping a delayed command ack with the current epoch and
/// enqueueing it. 0 (no pause) outside tests.
pub static TEST_ACK_STAMP_DELAY_MS: AtomicU64 = AtomicU64::new(0);

/// Test hook: pause (ms) after `acquire_lease` releases the owner lock and before enqueueing
/// `lease_granted` onto `ordinary_out_tx`, allowing heartbeat-triggered `lease_lost` to race ahead.
pub static TEST_LEASE_GRANTED_DELAY_MS: AtomicU64 = AtomicU64::new(0);

/// Test hook: pause (ms) after a deferred action install (`complete_pending_plan` /
/// `complete_pending_replay` / `complete_macro_check`) releases the owner lock and before
/// enqueueing its `command_ack` onto `ordinary_out_tx`, allowing a concurrent Stop/terminal to
/// race against the ACK gate. 0 (no pause) outside tests.
#[doc(hidden)]
pub static TEST_DEFERRED_ACK_DELAY_MS: AtomicU64 = AtomicU64::new(0);

static TEST_WS_WRITE_HOOKS_ACTIVE: AtomicBool = AtomicBool::new(false);
static TEST_WS_WRITE_PAUSED_CONNS: LazyLock<Mutex<HashMap<String, usize>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));
static TEST_WS_WRITE_STALL_ONCE_CONNS: LazyLock<Mutex<HashSet<String>>> =
    LazyLock::new(|| Mutex::new(HashSet::new()));
static TEST_WS_WRITE_TRACE_CONNS: LazyLock<Mutex<HashMap<String, Vec<String>>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));
static TEST_WS_WRITE_TIMEOUT_CONNS: LazyLock<Mutex<HashMap<String, Duration>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));
static TEST_WS_WRITE_UNPAUSE: LazyLock<tokio::sync::Notify> = LazyLock::new(tokio::sync::Notify::new);

static TEST_WS_WORKER_BEFORE_LOCK_ACTIVE: AtomicBool = AtomicBool::new(false);
static TEST_WS_WORKER_BEFORE_LOCK_CONNS: LazyLock<Mutex<HashMap<String, usize>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));
static TEST_WS_WORKER_BEFORE_LOCK_UNPAUSE: LazyLock<tokio::sync::Notify> =
    LazyLock::new(tokio::sync::Notify::new);

static TEST_WS_BLOCKING_PLAN_ACTIVE: AtomicBool = AtomicBool::new(false);
static TEST_WS_BLOCKING_PLAN_STATE: LazyLock<(Mutex<HashMap<String, usize>>, std::sync::Condvar)> =
    LazyLock::new(|| (Mutex::new(HashMap::new()), std::sync::Condvar::new()));

/// Test hook: pause or unpause `spawn_blocking` planners for a specific connection ID so tests
/// can disconnect deterministically mid-planning and then release a late blocking result.
pub fn set_test_ws_blocking_plan_paused(connection_id: &str, paused: bool) {
    let (lock, cvar) = &*TEST_WS_BLOCKING_PLAN_STATE;
    {
        let mut map = lock.lock().unwrap_or_else(|e| e.into_inner());
        if paused {
            map.entry(connection_id.to_owned()).or_insert(0);
        } else {
            map.remove(connection_id);
        }
        TEST_WS_BLOCKING_PLAN_ACTIVE.store(!map.is_empty(), Ordering::Release);
    }
    if !paused {
        cvar.notify_all();
    }
}

/// Test hook: number of `spawn_blocking` planner threads currently paused for `connection_id`.
pub fn test_ws_blocking_plan_blocked_count(connection_id: &str) -> usize {
    TEST_WS_BLOCKING_PLAN_STATE
        .0
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .get(connection_id)
        .copied()
        .unwrap_or(0)
}

fn maybe_pause_blocking_plan(connection_id: &str) {
    if !TEST_WS_BLOCKING_PLAN_ACTIVE.load(Ordering::Acquire) {
        return;
    }
    let (lock, cvar) = &*TEST_WS_BLOCKING_PLAN_STATE;
    let mut map = lock.lock().unwrap_or_else(|e| e.into_inner());
    if !map.contains_key(connection_id) {
        return;
    }
    if let Some(c) = map.get_mut(connection_id) {
        *c += 1;
    }
    while map.contains_key(connection_id) {
        map = cvar.wait(map).unwrap_or_else(|e| e.into_inner());
    }
    if let Some(c) = map.get_mut(connection_id) {
        *c = c.saturating_sub(1);
    }
}

fn sleep_cancellable_ms(delay_ms: u64, cancel: &AtomicBool) -> bool {
    let deadline = std::time::Instant::now() + Duration::from_millis(delay_ms);
    while std::time::Instant::now() < deadline {
        if cancel.load(Ordering::Acquire) {
            return false;
        }
        let rem = deadline.saturating_duration_since(std::time::Instant::now());
        std::thread::sleep(rem.min(Duration::from_millis(5)));
    }
    !cancel.load(Ordering::Acquire)
}

/// Test hook: pause or unpause the ordinary worker right after popping a frame and before
/// acquiring `supervisor.inner().write()` for a specific connection ID.
pub fn set_test_ws_worker_before_lock_paused(connection_id: &str, paused: bool) {
    {
        let mut map = TEST_WS_WORKER_BEFORE_LOCK_CONNS.lock().unwrap_or_else(|e| e.into_inner());
        if paused {
            map.entry(connection_id.to_owned()).or_insert(0);
        } else {
            map.remove(connection_id);
        }
        TEST_WS_WORKER_BEFORE_LOCK_ACTIVE.store(!map.is_empty(), Ordering::Release);
    }
    if !paused {
        TEST_WS_WORKER_BEFORE_LOCK_UNPAUSE.notify_waiters();
    }
}

/// Test hook: number of ordinary workers currently paused before acquiring `inner.write()` for `connection_id`.
pub fn test_ws_worker_before_lock_blocked_count(connection_id: &str) -> usize {
    TEST_WS_WORKER_BEFORE_LOCK_CONNS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .get(connection_id)
        .copied()
        .unwrap_or(0)
}

async fn maybe_pause_worker_before_lock(connection_id: &str) {
    if !TEST_WS_WORKER_BEFORE_LOCK_ACTIVE.load(Ordering::Acquire) {
        return;
    }
    let is_paused = || {
        TEST_WS_WORKER_BEFORE_LOCK_CONNS
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .contains_key(connection_id)
    };
    if !is_paused() {
        return;
    }
    struct WorkerBlockedGuard(String);
    impl Drop for WorkerBlockedGuard {
        fn drop(&mut self) {
            let mut map = TEST_WS_WORKER_BEFORE_LOCK_CONNS.lock().unwrap_or_else(|e| e.into_inner());
            if let Some(c) = map.get_mut(&self.0) {
                *c = c.saturating_sub(1);
            }
        }
    }
    {
        let mut map = TEST_WS_WORKER_BEFORE_LOCK_CONNS.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(c) = map.get_mut(connection_id) {
            *c += 1;
        }
    }
    let _guard = WorkerBlockedGuard(connection_id.to_owned());
    while is_paused() {
        let notified = TEST_WS_WORKER_BEFORE_LOCK_UNPAUSE.notified();
        if !is_paused() {
            break;
        }
        notified.await;
    }
}

fn refresh_write_hooks_active() {
    let paused = !TEST_WS_WRITE_PAUSED_CONNS.lock().unwrap_or_else(|e| e.into_inner()).is_empty();
    let stall_once = !TEST_WS_WRITE_STALL_ONCE_CONNS.lock().unwrap_or_else(|e| e.into_inner()).is_empty();
    let traced = !TEST_WS_WRITE_TRACE_CONNS.lock().unwrap_or_else(|e| e.into_inner()).is_empty();
    let timed = !TEST_WS_WRITE_TIMEOUT_CONNS.lock().unwrap_or_else(|e| e.into_inner()).is_empty();
    TEST_WS_WRITE_HOOKS_ACTIVE.store(paused || stall_once || traced || timed, Ordering::Release);
}

/// Test hook: stall ONLY the next outbound WebSocket write on `connection_id` until its
/// bounded timeout abandons it; subsequent writes proceed normally.
#[doc(hidden)]
pub fn set_test_ws_write_stall_once(connection_id: &str, enabled: bool) {
    {
        let mut set = TEST_WS_WRITE_STALL_ONCE_CONNS.lock().unwrap_or_else(|e| e.into_inner());
        if enabled {
            set.insert(connection_id.to_owned());
        } else {
            set.remove(connection_id);
        }
    }
    refresh_write_hooks_active();
}

/// Test hook: record attempted outbound WebSocket frame kinds (`text:<type>`, `close:<reason>`)
/// for `connection_id` in exact attempt order.
#[doc(hidden)]
pub fn enable_test_ws_write_trace(connection_id: &str) {
    {
        let mut map = TEST_WS_WRITE_TRACE_CONNS.lock().unwrap_or_else(|e| e.into_inner());
        map.insert(connection_id.to_owned(), Vec::new());
    }
    refresh_write_hooks_active();
}

/// Test hook: take and stop recording attempted outbound frame labels for `connection_id`.
#[doc(hidden)]
pub fn take_test_ws_write_trace(connection_id: &str) -> Vec<String> {
    let trace = {
        let mut map = TEST_WS_WRITE_TRACE_CONNS.lock().unwrap_or_else(|e| e.into_inner());
        map.remove(connection_id).unwrap_or_default()
    };
    refresh_write_hooks_active();
    trace
}

fn record_test_ws_write_attempt(connection_id: &str, msg: &Message) {
    let mut map = TEST_WS_WRITE_TRACE_CONNS.lock().unwrap_or_else(|e| e.into_inner());
    let Some(log) = map.get_mut(connection_id) else { return; };
    let label = match msg {
        Message::Text(txt) => {
            let ty = serde_json::from_str::<Value>(txt)
                .ok()
                .and_then(|v| v.get("type").and_then(Value::as_str).map(str::to_owned))
                .unwrap_or_else(|| "observation".to_owned());
            format!("text:{ty}")
        }
        Message::Close(Some(cf)) => format!("close:{}", cf.reason),
        Message::Close(None) => "close:".to_owned(),
        _ => "other".to_owned(),
    };
    log.push(label);
}

/// Test hook: pause or unpause outbound WebSocket writes for a single connection ID.
pub fn set_test_ws_write_paused(connection_id: &str, paused: bool) {
    {
        let mut map = TEST_WS_WRITE_PAUSED_CONNS.lock().unwrap_or_else(|e| e.into_inner());
        if paused {
            map.entry(connection_id.to_owned()).or_insert(0);
        } else {
            map.remove(connection_id);
        }
    }
    refresh_write_hooks_active();
    if !paused {
        TEST_WS_WRITE_UNPAUSE.notify_waiters();
    }
}

/// Test hook: override the bounded per-write timeout for a single connection ID.
pub fn set_test_ws_write_timeout(connection_id: &str, timeout: Option<Duration>) {
    {
        let mut map = TEST_WS_WRITE_TIMEOUT_CONNS.lock().unwrap_or_else(|e| e.into_inner());
        match timeout {
            Some(dur) => { map.insert(connection_id.to_owned(), dur); }
            None => { map.remove(connection_id); }
        }
    }
    refresh_write_hooks_active();
}

/// Test hook: number of writer tasks currently blocked in a simulated write stall for `connection_id`.
pub fn test_ws_writer_blocked_count(connection_id: &str) -> usize {
    TEST_WS_WRITE_PAUSED_CONNS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .get(connection_id)
        .copied()
        .unwrap_or(0)
}

async fn test_ack_stamp_delay() {
    let ms = TEST_ACK_STAMP_DELAY_MS.load(Ordering::Relaxed);
    if ms > 0 { tokio::time::sleep(Duration::from_millis(ms)).await; }
}

fn ws_write_timeout(connection_id: &str) -> Duration {
    if TEST_WS_WRITE_HOOKS_ACTIVE.load(Ordering::Acquire) {
        if let Some(dur) = TEST_WS_WRITE_TIMEOUT_CONNS
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(connection_id)
            .copied()
        {
            return dur;
        }
    }
    WS_WRITE_TIMEOUT
}

/// `?obs_hz=N`: send at most N observations per second on this connection (clamped to
/// 1..control_frequency_hz). Absent: every tick. Control replies are never decimated.
pub async fn ws_handler(ws: WebSocketUpgrade, State(supervisor): State<Arc<ArmSupervisor>>,
                        axum::extract::Query(query): axum::extract::Query<HashMap<String, String>>,
                        headers: axum::http::HeaderMap,
                        axum::Extension(deps): axum::Extension<crate::grasp::GraspDeps>,
                        axum::Extension(calib): axum::Extension<crate::calibration::CalibrationDeps>,
                        axum::Extension(sandbox): axum::Extension<Arc<crate::api::sandbox::SandboxState>>,
                        axum::Extension(auth): axum::Extension<crate::auth::OperatorAuth>) -> axum::response::Response {
    let obs_rate = match query.get("obs_hz") {
        None => None,
        Some(v) => {
            let control_hz = match supervisor.control_frequency_hz_checked().await {
                Ok(hz) => hz.max(1),
                Err(err) => return ApiError::from(err).into_response(),
            };
            match v.trim().parse::<u64>() {
                Ok(n) => Some(ObsRate { hz: n.clamp(1, u64::from(control_hz)) as f64, control_hz: f64::from(control_hz) }),
                Err(_) => return (StatusCode::BAD_REQUEST, "obs_hz must be an integer >= 0 (0 and 1 mean 1 Hz; values above control_frequency_hz are clamped)").into_response(),
            }
        }
    };
    let explicit_auth = query
        .get("auth")
        .is_some_and(|v| v.trim().eq_ignore_ascii_case("explicit"));
    let ws_token = if explicit_auth {
        None
    } else {
        crate::auth::OperatorAuth::extract_token(&headers)
    };
    ws.on_upgrade(move |socket| handle_socket(socket, supervisor, deps, calib, sandbox, obs_rate, auth, ws_token, explicit_auth)).into_response()
}

/// Per-connection observation rate limit.
#[derive(Debug, Clone, Copy)]
struct ObsRate { hz: f64, control_hz: f64 }

impl ObsRate {
    /// Whether an observation stamped `timestamp` is sent, given the last one sent.
    /// Half a control period of tolerance keeps tick jitter from skipping a slot.
    fn due(&self, last_sent: Option<f64>, timestamp: f64) -> bool {
        last_sent.is_none_or(|t| timestamp - t >= 1.0 / self.hz - 0.5 / self.control_hz)
    }
}

fn invalid_command(message: &str) -> ApiError {
    ApiError(StatusCode::BAD_REQUEST, message.into(), "INVALID_COMMAND".into())
}

fn rejection(error: ApiError, kind: &str, epoch: u64) -> Value {
    let mut v = json!({"type":kind, "status":error.status.as_u16(), "code":error.code,
        "error":error.error, "message":error.message, "safety_state":error.safety_state, "epoch_id":epoch});
    if let Some(rev) = error.current_revision {
        v["current_revision"] = json!(rev);
    }
    if let Some(d) = error.workspace_violation {
        if let Some(c) = d.constraint_id { v["constraint_id"] = json!(c); }
        if let Some(g) = d.geometry_id { v["geometry_id"] = json!(g); }
        if let Some(p) = d.violation_phase { v["violation_phase"] = json!(p); }
        if let Some(r) = d.reason { v["reason"] = json!(r); }
    }
    if let Some(Value::Object(m)) = error.details.map(|d| *d) { for (k, val) in m { v[k] = val; } }
    v
}

fn positions(value: Option<&Value>) -> Result<Vec<f32>, ApiError> {
    let values = value.and_then(Value::as_array).ok_or_else(|| invalid_command("Malformed positions array"))?;
    values.iter().map(|v| v.as_f64().map(|n| n as f32).filter(|n| n.is_finite())
        .ok_or_else(|| ApiError::from(ArmError::Safety(SafetyError::InvalidValue)))).collect()
}

/// Action started by a command: (action id, expected duration in s).
type Started = Option<(u64, f64)>;

/// Bounded inbound ordinary message queue capacity per WS connection.
const ORDINARY_IN_CAPACITY: usize = 64;
/// Bounded outbound ordinary control reply queue capacity per WS connection.
const ORDINARY_OUT_CAPACITY: usize = 64;
/// Bounded urgent outbound capacity for E-stop and Stop replies (reserved at admission).
const URGENT_OUT_CAPACITY: usize = 64;
/// Bounded revocation/preemption notice capacity per WS connection.
const REVOCATION_OUT_CAPACITY: usize = 8;
/// Bounded reserved outbound capacity for `auth_ack` replies (isolated from ordinary backlog).
const AUTH_OUT_CAPACITY: usize = 16;
/// Default bounded per-write timeout on a WS socket; on timeout, the connection closes.
const WS_WRITE_TIMEOUT: Duration = Duration::from_secs(5);
/// Maximum bounded grace (<= 1 s) to let the writer flush an explicit fatal error + Close frame.
const WS_FATAL_FLUSH_TIMEOUT: Duration = Duration::from_secs(1);
/// Maximum bounded grace to join connection-scoped child tasks before aborting and joining them.
const WS_TEARDOWN_JOIN_TIMEOUT: Duration = Duration::from_millis(250);

/// Handled by the ordinary worker while a [`MacroCheck`] runs; motion commands stay queued in
/// order within the bounded ordinary queue. (E-stop, `/api/v1/motion/stop`, `auth`, and
/// `heartbeat` never enter the ordinary queue.)
fn runs_during_macro_check(value: &Value) -> bool {
    dispatch::classify(value)
        .1
        .is_some_and(|e| e.admission == AdmissionClass::OrdinarySession)
}

/// The reply to a message cancelled by a stop / e-stop before it ran (`code` STOPPED /
/// ESTOP_ACTIVE). None for messages that are not acknowledged.
/// Preserves the legacy cancellation precedence (`command` > `preset` > `servo::is_servo_frame`),
/// including mixed-field lease frames carrying `positions` or `mode: "cartesian_joint_stream"`.
fn cancelled_reply(value: &Value, code: &str, epoch: Option<u64>) -> Option<Value> {
    let raw_kind = value.get("type").and_then(Value::as_str).unwrap_or("");
    let mut reply = json!({"status":409, "code":code, "error":code, "message":"Cancelled by a stop before it began"});
    if let Some(ep) = epoch {
        reply["epoch_id"] = json!(ep);
    }
    if raw_kind == "command" {
        reply["type"] = json!("command_reject");
        reply["request_id"] = value.get("request_id").cloned().unwrap_or(Value::Null);
    } else if raw_kind == "preset" {
        reply["type"] = json!("preset_reject");
        reply["name"] = value.get("name").cloned().unwrap_or(Value::Null);
    } else if servo::is_servo_frame(raw_kind, value) {
        reply["type"] = json!("servo_reject");
        reply["reason"] = json!("stopped");
    } else {
        return None;
    }
    Some(reply)
}

pub(super) fn validate_command_body_common(body: &Value) -> Result<(), &'static str> {
    if !body.is_object() { return Err("Command body must be an object"); }
    if body.get("duration_secs").filter(|v| !v.is_null()).is_some_and(|d| d.as_f64().is_none()) {
        return Err("duration_secs must be numeric");
    }
    Ok(())
}

fn ws_status_reject(value: &Value, status: u16, code: &str, msg: &str, servo_reason: &str) -> Option<Value> {
    let (kind, entry) = dispatch::classify(value);
    match (kind, entry.map(|e| e.reply)) {
        (FrameKind::AcquireLease, Some(ReplyPolicy::LeaseReply)) => {
            let mut r = json!({"type":"lease_rejected", "status":status, "code":code, "error":code, "message":msg});
            if let Some(s) = value.get("seq").and_then(Value::as_u64) { r["seq"] = json!(s); }
            Some(r)
        }
        (FrameKind::ReleaseLease, Some(ReplyPolicy::LeaseReply)) => {
            Some(json!({"type":"lease_released", "success":false, "status":status, "code":code, "error":code, "message":msg}))
        }
        (FrameKind::Command, _) => {
            Some(json!({"type":"command_reject", "request_id":value.get("request_id"), "status":status, "code":code, "error":code, "message":msg}))
        }
        (FrameKind::Preset, Some(ReplyPolicy::PresetReply)) => {
            Some(json!({"type":"preset_reject", "name":value.get("name"), "status":status, "code":code, "error":code, "message":msg}))
        }
        (FrameKind::Servo | FrameKind::ImplicitServo, Some(ReplyPolicy::ServoRejectOnly)) => {
            Some(json!({"type":"servo_reject", "reason":servo_reason, "status":status, "code":code, "error":code, "message":msg}))
        }
        _ => None,
    }
}

fn executor_routing_reject(value: &Value, owner_mode: u8) -> Option<Value> {
    let (code, msg) = if owner_mode == crate::supervisor::OWNER_EXECUTOR {
        ("EXECUTOR_ROUTING_PENDING", "WebSocket message is not yet routed through the RT executor")
    } else {
        ("EXECUTOR_UNAVAILABLE", "RT executor ownership is transitioning or unavailable")
    };
    ws_status_reject(value, 503, code, msg, "executor_routing_pending")
}

fn backlog_full_reject(value: &Value) -> Option<Value> {
    ws_status_reject(value, 503, "BACKLOG_FULL", "WebSocket ordinary message backlog is full", "backlog_full")
}

#[derive(Debug, Clone)]
struct FatalClose {
    code: &'static str,
    message: &'static str,
}

impl FatalClose {
    fn outbound_queue_full() -> Self {
        Self {
            code: "OUTBOUND_QUEUE_FULL",
            message: "WebSocket outbound control reply queue saturated; closing connection",
        }
    }

    fn urgent_queue_full() -> Self {
        Self {
            code: "URGENT_QUEUE_FULL",
            message: "WebSocket urgent reply queue saturated; closing connection",
        }
    }

    fn write_timeout() -> Self {
        Self {
            code: "WRITE_TIMEOUT",
            message: "WebSocket write timed out; closing connection",
        }
    }

    fn error_value(&self, estop_latched: bool) -> Value {
        let mut v = json!({
            "type": "error",
            "status": 503,
            "code": self.code,
            "error": self.code,
            "message": self.message,
        });
        if estop_latched {
            v["estop_latched"] = json!(true);
        }
        v
    }

    fn close_frame(&self, estop_latched: bool) -> CloseFrame {
        let reason = if estop_latched {
            format!("ESTOP_LATCHED:{}", self.code)
        } else {
            self.code.to_string()
        };
        CloseFrame {
            code: 1013,
            reason: reason.into(),
        }
    }
}

struct InFlightUrgentGuard {
    counter: Arc<AtomicUsize>,
    notify: Arc<tokio::sync::Notify>,
}

impl Drop for InFlightUrgentGuard {
    fn drop(&mut self) {
        self.counter.fetch_sub(1, Ordering::SeqCst);
        self.notify.notify_waiters();
    }
}

#[derive(Clone)]
struct WsWriterHandle {
    estop_tx: mpsc::Sender<Value>,
    stop_tx: mpsc::Sender<Value>,
    auth_tx: mpsc::Sender<Value>,
    ordinary_tx: mpsc::Sender<Value>,
    fatal_tx: mpsc::Sender<FatalClose>,
    fatal_triggered: Arc<AtomicBool>,
    fatal_notify: Arc<tokio::sync::Notify>,
    estop_admitted_count: Arc<AtomicU64>,
    in_flight_estops: Arc<AtomicUsize>,
    in_flight_stops: Arc<AtomicUsize>,
    urgent_done_notify: Arc<tokio::sync::Notify>,
}

impl WsWriterHandle {
    fn trigger_fatal(&self, fatal: FatalClose) {
        self.fatal_triggered.store(true, Ordering::Release);
        self.fatal_notify.notify_waiters();
        let _ = self.fatal_tx.try_send(fatal);
    }

    fn is_fatal(&self) -> bool {
        self.fatal_triggered.load(Ordering::Acquire)
    }

    /// True only once `writer_task` has dropped its receivers and closed the outbound pipeline.
    fn is_writer_closed(&self) -> bool {
        self.estop_tx.is_closed()
    }

    fn mark_estop_admitted(&self) {
        self.estop_admitted_count.fetch_add(1, Ordering::SeqCst);
    }

    fn enter_in_flight_estop(&self) -> InFlightUrgentGuard {
        self.in_flight_estops.fetch_add(1, Ordering::SeqCst);
        InFlightUrgentGuard {
            counter: self.in_flight_estops.clone(),
            notify: self.urgent_done_notify.clone(),
        }
    }

    fn enter_in_flight_stop(&self) -> InFlightUrgentGuard {
        self.in_flight_stops.fetch_add(1, Ordering::SeqCst);
        InFlightUrgentGuard {
            counter: self.in_flight_stops.clone(),
            notify: self.urgent_done_notify.clone(),
        }
    }

    /// `auth_ack` send from the off-reader auth task via its dedicated reserved queue.
    /// On overflow, signals the shared fatal close path (`fatal_triggered = true` before sending
    /// `FatalClose`) so `writer_task` owns the urgent flush and close without ever aborting the reader.
    async fn send_auth(&self, reply: Value) -> bool {
        if self.is_fatal() {
            return false;
        }
        match self.auth_tx.try_send(reply) {
            Ok(()) => true,
            Err(mpsc::error::TrySendError::Full(reply)) => {
                tokio::task::yield_now().await;
                if self.is_fatal() {
                    return false;
                }
                match self.auth_tx.try_send(reply) {
                    Ok(()) => true,
                    Err(mpsc::error::TrySendError::Full(_)) => {
                        self.trigger_fatal(FatalClose::outbound_queue_full());
                        false
                    }
                    Err(mpsc::error::TrySendError::Closed(_)) => false,
                }
            }
            Err(mpsc::error::TrySendError::Closed(_)) => false,
        }
    }

    /// Non-blocking ordinary control reply send (reader / immediate reject). Never drops
    /// silently: if the ordinary queue is full, triggers connection close with an error frame.
    fn try_send_ordinary_control(&self, reply: Value) -> bool {
        if self.is_fatal() {
            return false;
        }
        match self.ordinary_tx.try_send(reply) {
            Ok(()) => true,
            Err(mpsc::error::TrySendError::Full(_)) => {
                self.trigger_fatal(FatalClose::outbound_queue_full());
                false
            }
            Err(mpsc::error::TrySendError::Closed(_)) => false,
        }
    }

    /// Ordinary control reply send from async worker / cancellation tasks. Tries non-blocking
    /// first; yields once if the writer task simply hasn't been polled yet in a tight burst,
    /// then closes the connection with an explicit error frame if still full. Never drops.
    async fn send_ordinary_control(&self, reply: Value) -> bool {
        if self.is_fatal() {
            return false;
        }
        match self.ordinary_tx.try_send(reply) {
            Ok(()) => true,
            Err(mpsc::error::TrySendError::Full(reply)) => {
                tokio::task::yield_now().await;
                if self.is_fatal() {
                    return false;
                }
                match self.ordinary_tx.try_send(reply) {
                    Ok(()) => true,
                    Err(mpsc::error::TrySendError::Full(_)) => {
                        self.trigger_fatal(FatalClose::outbound_queue_full());
                        false
                    }
                    Err(mpsc::error::TrySendError::Closed(_)) => false,
                }
            }
            Err(mpsc::error::TrySendError::Closed(_)) => false,
        }
    }

    fn try_reserve_estop(&self) -> Result<mpsc::OwnedPermit<Value>, mpsc::error::TrySendError<()>> {
        self.estop_tx.clone().try_reserve_owned().map_err(|e| match e {
            mpsc::error::TrySendError::Full(_) => mpsc::error::TrySendError::Full(()),
            mpsc::error::TrySendError::Closed(_) => mpsc::error::TrySendError::Closed(()),
        })
    }

    /// Fallback delivery for an E-stop reply whose initial non-blocking urgent reservation hit
    /// capacity: waits for the next freed urgent slot so an E-stop reply is never dropped.
    async fn send_estop_blocking(&self, reply: Value) -> bool {
        self.estop_tx.send(reply).await.is_ok()
    }

    fn try_reserve_stop(&self) -> Result<mpsc::OwnedPermit<Value>, mpsc::error::TrySendError<()>> {
        self.stop_tx.clone().try_reserve_owned().map_err(|e| match e {
            mpsc::error::TrySendError::Full(_) => mpsc::error::TrySendError::Full(()),
            mpsc::error::TrySendError::Closed(_) => mpsc::error::TrySendError::Closed(()),
        })
    }

    fn try_send_stop_best_effort(&self, reply: Value) -> bool {
        self.stop_tx.try_send(reply).is_ok()
    }
}

/// Per-connection RAII gate pausing `subscribe_action_terminals` delivery for this connection
/// while `worker_task` executes an immediate owner command and enqueues its reply on `ordinary_tx`.
/// Returning `false` from the subscriber callback keeps pending terminals at the head of
/// `TerminalBroadcaster`'s bounded FIFO (preserving capacity accounting and holding back newer
/// telemetry broadcasts) until this guard drops and wakes `armd-dispatch`.
pub(super) struct CommandReplyGateGuard {
    active: Arc<AtomicBool>,
    hub: Arc<crate::rt_core::publish::StateHub>,
}

impl CommandReplyGateGuard {
    pub(super) fn enter(active: &Arc<AtomicBool>, hub: &Arc<crate::rt_core::publish::StateHub>) -> Self {
        active.store(true, Ordering::Release);
        Self {
            active: Arc::clone(active),
            hub: Arc::clone(hub),
        }
    }
}

impl Drop for CommandReplyGateGuard {
    fn drop(&mut self) {
        self.active.store(false, Ordering::Release);
        self.hub.wake();
    }
}

/// Single-convergence-point wire epoch guard in the writer task: ensures outbound control
/// frames (`command_ack`, `command_reject`, delayed acks, etc.) never regress `epoch_id` or
/// `safety_state` behind an already-emitted newer `epoch_sync` or control frame.
fn enforce_monotonic_epoch(last_wire_epoch: &mut Option<(u64, Option<Value>)>, reply: &mut Value) {
    let Some(obj) = reply.as_object_mut() else { return; };
    let is_sync = obj.get("type").and_then(Value::as_str) == Some("epoch_sync");
    if let Some(ep) = obj.get("epoch_id").and_then(Value::as_u64) {
        let st = obj.get("safety_state").cloned();
        if let Some((prev_ep, prev_st)) = last_wire_epoch.as_ref() {
            if ep < *prev_ep {
                obj.insert("epoch_id".into(), json!(prev_ep));
                if obj.contains_key("safety_state") {
                    if let Some(s) = prev_st {
                        obj.insert("safety_state".into(), s.clone());
                    }
                }
                return;
            }
            if ep == *prev_ep && is_sync && st.is_some() {
                *last_wire_epoch = Some((ep, st));
                return;
            }
        }
        if is_sync || last_wire_epoch.as_ref().is_none_or(|(p, _)| ep > *p) {
            let next_st = st.or_else(|| last_wire_epoch.as_ref().and_then(|(_, s)| s.clone()));
            *last_wire_epoch = Some((ep, next_st));
        }
    }
}

fn extract_lease_loss_gen_value(v: &Value) -> Option<u64> {
    if v.get("type").and_then(Value::as_str) == Some("lease_lost") {
        v.get("generation").and_then(Value::as_u64)
    } else {
        None
    }
}

fn extract_lease_loss_gen(msg: &Message) -> Option<u64> {
    let Message::Text(t) = msg else { return None; };
    let v: Value = serde_json::from_str(t).ok()?;
    extract_lease_loss_gen_value(&v)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum WriteErr {
    Timeout,
    Closed,
}

async fn write_ws_frame(
    connection_id: &str,
    sender: &mut futures_util::stream::SplitSink<WebSocket, Message>,
    msg: Message,
    timeout_dur: Duration,
) -> Result<(), WriteErr> {
    tokio::time::timeout(timeout_dur, async {
        if TEST_WS_WRITE_HOOKS_ACTIVE.load(Ordering::Acquire) {
            record_test_ws_write_attempt(connection_id, &msg);
            let stall_once = {
                let mut set = TEST_WS_WRITE_STALL_ONCE_CONNS.lock().unwrap_or_else(|e| e.into_inner());
                set.remove(connection_id)
            };
            if stall_once {
                refresh_write_hooks_active();
                struct OnceBlockedGuard(String);
                impl Drop for OnceBlockedGuard {
                    fn drop(&mut self) {
                        let mut map = TEST_WS_WRITE_PAUSED_CONNS.lock().unwrap_or_else(|e| e.into_inner());
                        if let Some(c) = map.get_mut(&self.0) {
                            *c = c.saturating_sub(1);
                            if *c == 0 {
                                map.remove(&self.0);
                            }
                        }
                        drop(map);
                        refresh_write_hooks_active();
                    }
                }
                {
                    let mut map = TEST_WS_WRITE_PAUSED_CONNS.lock().unwrap_or_else(|e| e.into_inner());
                    *map.entry(connection_id.to_owned()).or_insert(0) += 1;
                }
                let _guard = OnceBlockedGuard(connection_id.to_owned());
                std::future::pending::<()>().await;
            }
            let is_paused = || {
                TEST_WS_WRITE_PAUSED_CONNS
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .contains_key(connection_id)
            };
            if is_paused() {
                struct BlockedGuard(String);
                impl Drop for BlockedGuard {
                    fn drop(&mut self) {
                        let mut map = TEST_WS_WRITE_PAUSED_CONNS.lock().unwrap_or_else(|e| e.into_inner());
                        if let Some(c) = map.get_mut(&self.0) {
                            *c = c.saturating_sub(1);
                        }
                    }
                }
                {
                    let mut map = TEST_WS_WRITE_PAUSED_CONNS.lock().unwrap_or_else(|e| e.into_inner());
                    if let Some(c) = map.get_mut(connection_id) {
                        *c += 1;
                    }
                }
                let _guard = BlockedGuard(connection_id.to_owned());
                while is_paused() {
                    let notified = TEST_WS_WRITE_UNPAUSE.notified();
                    if !is_paused() {
                        break;
                    }
                    notified.await;
                }
            }
        }
        sender.send(msg).await
    })
    .await
    .map_err(|_| WriteErr::Timeout)?
    .map_err(|_| WriteErr::Closed)
}

async fn write_ws_frame_bounded(
    connection_id: &str,
    sender: &mut futures_util::stream::SplitSink<WebSocket, Message>,
    msg: Message,
    timeout_dur: Duration,
    fatal_triggered: &AtomicBool,
    fatal_notify: &tokio::sync::Notify,
) -> Result<(), WriteErr> {
    let write_fut = write_ws_frame(connection_id, sender, msg, timeout_dur);
    // Keep fatal abandonment grace short so abandoning a stalled in-progress write leaves ample
    // bounded budget to attempt queued urgent replies, the fatal frame, and Close.
    let fatal_grace = timeout_dur
        .min(Duration::from_millis(120))
        .max(Duration::from_millis(20));
    tokio::select! {
        res = write_fut => res,
        _ = async {
            if !fatal_triggered.load(Ordering::Acquire) {
                fatal_notify.notified().await;
            }
            tokio::time::sleep(fatal_grace).await;
        } => Err(WriteErr::Timeout),
    }
}

async fn send_best_effort_close(
    connection_id: &str,
    sender: &mut futures_util::stream::SplitSink<WebSocket, Message>,
    frame: CloseFrame,
) {
    let msg = Message::Close(Some(frame));
    if TEST_WS_WRITE_HOOKS_ACTIVE.load(Ordering::Acquire) {
        record_test_ws_write_attempt(connection_id, &msg);
    }
    let _ = tokio::time::timeout(Duration::from_millis(50), async {
        let _ = sender.send(msg).await;
        let _ = sender.flush().await;
        let _ = sender.close().await;
    })
    .await;
}

#[allow(clippy::too_many_arguments)]
async fn flush_urgent_then_fatal_and_close(
    connection_id: &str,
    sender: &mut futures_util::stream::SplitSink<WebSocket, Message>,
    fatal: &FatalClose,
    fatal_triggered: &AtomicBool,
    fatal_notify: &tokio::sync::Notify,
    last_wire_epoch: &mut Option<(u64, Option<Value>)>,
    estop_out_rx: &mut mpsc::Receiver<Value>,
    stop_out_rx: &mut mpsc::Receiver<Value>,
    estop_admitted_count: &AtomicU64,
    estop_replies_delivered: &mut u64,
    in_flight_estops: &AtomicUsize,
    in_flight_stops: &AtomicUsize,
    urgent_done_notify: &tokio::sync::Notify,
) {
    // Always publish the fatal flag before starting the urgent flush and close.
    fatal_triggered.store(true, Ordering::Release);
    fatal_notify.notify_waiters();
    let total_grace = ws_write_timeout(connection_id).min(WS_FATAL_FLUSH_TIMEOUT);
    // Give each urgent reply and the fatal frame its own short bounded budget so a single stalled
    // write never starves subsequent urgent replies, the fatal error frame, or Close.
    let per_frame_budget = (total_grace / 4).clamp(Duration::from_millis(20), Duration::from_millis(80));
    let fatal_budget = per_frame_budget.min(total_grace.max(Duration::from_millis(20)));
    let urgent_budget = total_grace.saturating_sub(fatal_budget).max(per_frame_budget);
    let urgent_deadline = tokio::time::Instant::now() + urgent_budget;
    let urgent_rem = || urgent_deadline.saturating_duration_since(tokio::time::Instant::now());
    let mut socket_closed = false;

    // 1. Attempt all already-queued urgent E-stop and Stop replies BEFORE fatal error + Close.
    //    Always check the remaining urgent budget BEFORE dequeuing a reply.
    while !socket_closed {
        let r = urgent_rem();
        if r.is_zero() {
            break;
        }
        if let Ok(mut val) = estop_out_rx.try_recv() {
            enforce_monotonic_epoch(last_wire_epoch, &mut val);
            match write_ws_frame(connection_id, sender, Message::Text(val.to_string().into()), per_frame_budget.min(r)).await {
                Ok(()) => { *estop_replies_delivered += 1; }
                Err(WriteErr::Timeout) => {}
                Err(WriteErr::Closed) => { socket_closed = true; }
            }
        } else if let Ok(mut val) = stop_out_rx.try_recv() {
            enforce_monotonic_epoch(last_wire_epoch, &mut val);
            if matches!(
                write_ws_frame(connection_id, sender, Message::Text(val.to_string().into()), per_frame_budget.min(r)).await,
                Err(WriteErr::Closed)
            ) {
                socket_closed = true;
            }
        } else {
            break;
        }
    }

    // 2. If urgent E-stop/Stop replies admitted before close are still in-flight, wait within
    //    the bounded urgent budget so they are attempted BEFORE the fatal error frame and Close.
    //    Check the deadline arm first under `biased;` so an expired budget never dequeues a reply.
    while !socket_closed
        && (in_flight_estops.load(Ordering::SeqCst) > 0 || in_flight_stops.load(Ordering::SeqCst) > 0)
    {
        if urgent_rem().is_zero() {
            break;
        }
        tokio::select! {
            biased;
            _ = tokio::time::sleep_until(urgent_deadline) => { break; }
            msg = estop_out_rx.recv() => match msg {
                Some(mut val) => {
                    let r2 = urgent_rem().max(Duration::from_millis(1));
                    enforce_monotonic_epoch(last_wire_epoch, &mut val);
                    match write_ws_frame(connection_id, sender, Message::Text(val.to_string().into()), per_frame_budget.min(r2)).await {
                        Ok(()) => { *estop_replies_delivered += 1; }
                        Err(WriteErr::Timeout) => {}
                        Err(WriteErr::Closed) => { socket_closed = true; }
                    }
                }
                None => break,
            },
            msg = stop_out_rx.recv() => match msg {
                Some(mut val) => {
                    let r2 = urgent_rem().max(Duration::from_millis(1));
                    enforce_monotonic_epoch(last_wire_epoch, &mut val);
                    if matches!(
                        write_ws_frame(connection_id, sender, Message::Text(val.to_string().into()), per_frame_budget.min(r2)).await,
                        Err(WriteErr::Closed)
                    ) {
                        socket_closed = true;
                    }
                }
                None => break,
            },
            _ = urgent_done_notify.notified() => {}
        }
    }

    // Drain any final urgent reply pushed right before the in-flight guard dropped (check budget
    // BEFORE dequeuing).
    while !socket_closed {
        let r = urgent_rem();
        if r.is_zero() {
            break;
        }
        if let Ok(mut val) = estop_out_rx.try_recv() {
            enforce_monotonic_epoch(last_wire_epoch, &mut val);
            match write_ws_frame(connection_id, sender, Message::Text(val.to_string().into()), per_frame_budget.min(r)).await {
                Ok(()) => { *estop_replies_delivered += 1; }
                Err(WriteErr::Timeout) => {}
                Err(WriteErr::Closed) => { socket_closed = true; }
            }
        } else if let Ok(mut val) = stop_out_rx.try_recv() {
            enforce_monotonic_epoch(last_wire_epoch, &mut val);
            if matches!(
                write_ws_frame(connection_id, sender, Message::Text(val.to_string().into()), per_frame_budget.min(r)).await,
                Err(WriteErr::Closed)
            ) {
                socket_closed = true;
            }
        } else {
            break;
        }
    }

    let undelivered_estop = estop_admitted_count.load(Ordering::SeqCst) > *estop_replies_delivered;
    let close_frame = fatal.close_frame(undelivered_estop);

    // 3. Always attempt the fatal error frame with its own short bounded budget BEFORE Close
    //    (unless the underlying socket is already closed).
    if !socket_closed
        && matches!(
            write_ws_frame(
                connection_id,
                sender,
                Message::Text(fatal.error_value(undelivered_estop).to_string().into()),
                fatal_budget,
            )
            .await,
            Err(WriteErr::Closed)
        )
    {
        socket_closed = true;
    }

    // 4. Finally send Close (trying normal bounded write first, then best-effort direct Close
    //    so any undelivered E-stop remains visible in the CloseFrame reason).
    if !socket_closed
        && write_ws_frame(connection_id, sender, Message::Close(Some(close_frame.clone())), fatal_budget)
            .await
            .is_ok()
    {
        let _ = tokio::time::timeout(Duration::from_millis(50), async {
            let _ = sender.flush().await;
            let _ = sender.close().await;
        })
        .await;
        return;
    }
    send_best_effort_close(connection_id, sender, close_frame).await;
}

/// Dedicated latest-wins heartbeat handoff isolated from the bounded ordinary queue and slow worker tasks.
#[derive(Clone, Default)]
struct HeartbeatSlot {
    latest: Arc<Mutex<(Option<Value>, bool)>>,
    notify: Arc<tokio::sync::Notify>,
}

impl HeartbeatSlot {
    fn push_latest(&self, value: Value) {
        {
            let mut g = self.latest.lock().unwrap_or_else(|e| e.into_inner());
            if g.1 { return; }
            g.0 = Some(value);
        }
        self.notify.notify_one();
    }

    async fn pop(&self) -> Option<Value> {
        loop {
            let notified = self.notify.notified();
            {
                let mut g = self.latest.lock().unwrap_or_else(|e| e.into_inner());
                if let Some(v) = g.0.take() {
                    return Some(v);
                }
                if g.1 {
                    return None;
                }
            }
            notified.await;
        }
    }

    fn close(&self) {
        {
            let mut g = self.latest.lock().unwrap_or_else(|e| e.into_inner());
            g.1 = true;
        }
        self.notify.notify_waiters();
    }
}

struct ConnectionTasksInner {
    handles: Vec<tokio::task::JoinHandle<()>>,
    closed: bool,
}

/// Tracks every spawned child task belonging to one WS connection so teardown waits with a
/// bounded timeout, aborts any remaining tasks, and joins every handle before returning.
#[derive(Clone)]
struct ConnectionTasks {
    inner: Arc<Mutex<ConnectionTasksInner>>,
}

impl ConnectionTasks {
    fn new() -> Self {
        Self {
            inner: Arc::new(Mutex::new(ConnectionTasksInner {
                handles: Vec::new(),
                closed: false,
            })),
        }
    }

    fn spawn<F>(&self, fut: F)
    where
        F: std::future::Future<Output = ()> + Send + 'static,
    {
        let _ = self.spawn_abortable(fut);
    }

    fn spawn_abortable<F>(&self, fut: F) -> tokio::task::AbortHandle
    where
        F: std::future::Future<Output = ()> + Send + 'static,
    {
        let handle = tokio::spawn(fut);
        let abort = handle.abort_handle();
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        if g.closed {
            abort.abort();
            g.handles.push(handle);
            return abort;
        }
        g.handles.retain(|h| !h.is_finished());
        g.handles.push(handle);
        abort
    }

    async fn shutdown_and_join(&self, grace: Duration) {
        let mut handles = {
            let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
            g.closed = true;
            std::mem::take(&mut g.handles)
        };
        let deadline = tokio::time::Instant::now() + grace;
        for h in &mut handles {
            if h.is_finished() {
                let _ = h.await;
                continue;
            }
            let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
            if remaining.is_zero() || tokio::time::timeout(remaining, &mut *h).await.is_err() {
                h.abort();
                let _ = h.await;
            }
        }
        // Drain any task registered concurrently right before `closed = true` was observed.
        loop {
            let extra = {
                let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
                std::mem::take(&mut g.handles)
            };
            if extra.is_empty() {
                break;
            }
            for h in extra {
                h.abort();
                let _ = h.await;
            }
        }
    }
}

struct QueuedOrdinary {
    value: Value,
    stamp: crate::supervisor::AdmissionStamp,
    received_at: Instant,
}

struct OrdinaryQueueInner {
    items: VecDeque<QueuedOrdinary>,
    closed: bool,
}

#[derive(Clone)]
struct OrdinaryQueue {
    inner: Arc<Mutex<OrdinaryQueueInner>>,
    notify: Arc<tokio::sync::Notify>,
}

impl OrdinaryQueue {
    fn new() -> Self {
        Self {
            inner: Arc::new(Mutex::new(OrdinaryQueueInner {
                items: VecDeque::with_capacity(ORDINARY_IN_CAPACITY),
                closed: false,
            })),
            notify: Arc::new(tokio::sync::Notify::new()),
        }
    }

    fn try_push(
        &self,
        value: Value,
        stamp: crate::supervisor::AdmissionStamp,
        received_at: Instant,
    ) -> Result<(), Value> {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        if g.closed || g.items.len() >= ORDINARY_IN_CAPACITY {
            return Err(value);
        }
        g.items.push_back(QueuedOrdinary {
            value,
            stamp,
            received_at,
        });
        drop(g);
        self.notify.notify_one();
        Ok(())
    }

    async fn pop(&self, macro_active: bool) -> Option<QueuedOrdinary> {
        loop {
            let notified = self.notify.notified();
            {
                let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
                let idx = if !macro_active {
                    (!g.items.is_empty()).then_some(0)
                } else {
                    g.items.iter().position(|item| runs_during_macro_check(&item.value))
                };
                if let Some(i) = idx {
                    return g.items.remove(i);
                }
                if g.closed {
                    return None;
                }
            }
            notified.await;
        }
    }

    /// Drains cancellable motion/RPC items (`command`, `preset`, `servo`) while preserving
    /// non-cancellable session/lease frames (`acquire_lease`, `release_lease`, `auth`, `heartbeat`).
    /// Preserves the legacy cancellation predicate (`command` | `preset` | `servo::is_servo_frame`)
    /// so mixed-field frames retain exact baseline cancellation parity.
    /// Builds JSON cancellation payloads outside the sync lock so the reader critical section stays O(1).
    fn drain_cancellable(&self, code: &'static str, epoch: Option<u64>) -> Vec<Value> {
        let mut to_cancel = Vec::new();
        {
            let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
            let n = g.items.len();
            for _ in 0..n {
                if let Some(item) = g.items.pop_front() {
                    let raw_kind = item.value.get("type").and_then(Value::as_str).unwrap_or("");
                    let (_, entry) = dispatch::classify(&item.value);
                    let exempt_disable = code == "STOPPED"
                        && entry.is_some_and(|e| e.admission == AdmissionClass::OrdinaryTorqueOrStopExempt)
                        && lease::is_torque_disable_command(&item.value);
                    let cancellable = !exempt_disable
                        && (matches!(raw_kind, "command" | "preset")
                            || servo::is_servo_frame(raw_kind, &item.value));
                    if cancellable {
                        to_cancel.push(item.value);
                    } else {
                        g.items.push_back(item);
                    }
                }
            }
        }
        to_cancel
            .into_iter()
            .filter_map(|v| cancelled_reply(&v, code, epoch))
            .collect()
    }

    #[cfg(test)]
    fn len(&self) -> usize {
        self.inner.lock().unwrap_or_else(|e| e.into_inner()).items.len()
    }

    fn close(&self) {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        g.closed = true;
        drop(g);
        self.notify.notify_waiters();
    }
}

#[derive(Debug, Clone, Copy)]
struct CancelSignal {
    gen: u64,
    code: &'static str,
    epoch: Option<u64>,
}

struct CoalescedStopWaiter {
    raw_request_id: Value,
    parsed_request_id: Option<u64>,
    request_id_valid: bool,
    body_error: Option<&'static str>,
    permit: Option<mpsc::OwnedPermit<Value>>,
}

struct ActiveStopBatch {
    leader_id: u64,
    lease_generation: u64,
    epoch: Option<u64>,
    pending_max_request_id: Option<u64>,
    waiters: Vec<CoalescedStopWaiter>,
}

#[derive(Default)]
struct InFlightStopState {
    next_leader_id: u64,
    active: Option<ActiveStopBatch>,
}

fn last_request_get(lr: &Mutex<Option<u64>>) -> Option<u64> {
    *lr.lock().unwrap_or_else(|e| e.into_inner())
}

fn last_request_advance(lr: &Mutex<Option<u64>>, id: Option<u64>) {
    let Some(new_id) = id else { return; };
    let mut g = lr.lock().unwrap_or_else(|e| e.into_inner());
    *g = Some(g.map_or(new_id, |prev| prev.max(new_id)));
}

fn format_urgent_stop_outcome(
    raw_request_id: &Value,
    parsed_request_id: Option<u64>,
    last_request: &Mutex<Option<u64>>,
    outcome: Result<&crate::rt_core::urgent::UrgentStopOutcome, &ApiError>,
) -> Value {
    use crate::rt_core::urgent::{UrgentStopError, UrgentStopOutcome};
    let reject_api = |api: &ApiError| {
        json!({"type":"command_reject", "request_id":raw_request_id, "status":api.status.as_u16(), "code":api.code, "error":api.error, "message":api.message})
    };
    match outcome {
        Ok(UrgentStopOutcome::Applied { stop_generation, epoch, safety_state, is_torqued, control_mode_gravity_comp, .. }) => {
            last_request_advance(last_request, parsed_request_id);
            let mode = if *control_mode_gravity_comp { "gravity_comp" } else { "position" };
            let mut ack = json!({"type":"command_ack", "request_id":raw_request_id, "status":"accepted", "epoch_id":epoch, "safety_state":&**safety_state, "is_torqued":is_torqued, "control_mode":mode});
            if *stop_generation > 0 { ack["stop_generation"] = json!(stop_generation); }
            ack
        }
        Ok(UrgentStopOutcome::LeaseLost { epoch }) => json!({
            "type":"command_reject", "request_id":raw_request_id, "status":409, "code":"LEASE_LOST",
            "message":"This connection does not own an active lease", "error":"LEASE_LOST", "epoch_id":epoch
        }),
        Ok(UrgentStopOutcome::EpochMismatch { epoch }) => json!({
            "type":"command_reject", "request_id":raw_request_id, "status":409, "code":"EPOCH_MISMATCH",
            "message":"Command epoch is no longer current", "error":"EPOCH_MISMATCH", "epoch_id":epoch
        }),
        Ok(UrgentStopOutcome::InvalidRequestId) => json!({
            "type":"command_reject", "request_id":raw_request_id, "status":400, "code":"INVALID_COMMAND",
            "message":"request_id must be present and strictly increase", "error":"INVALID_COMMAND"
        }),
        Ok(UrgentStopOutcome::InvalidBody { message, epoch }) => {
            last_request_advance(last_request, parsed_request_id);
            let mut reply = rejection(invalid_command(message), "command_reject", *epoch);
            reply["request_id"] = raw_request_id.clone();
            reply
        }
        Ok(UrgentStopOutcome::ExecutorStopped) => {
            let api = ApiError::from(ArmError::from(UrgentStopError::Stopped));
            reject_api(&api)
        }
        Err(api) => reject_api(api),
    }
}

#[allow(clippy::too_many_arguments)]
async fn deliver_urgent_stop_batch(
    waiters: Vec<CoalescedStopWaiter>,
    stop_res: Result<crate::rt_core::urgent::UrgentStopOutcome, ArmError>,
    pre_drained: Option<Vec<Value>>,
    last_request: &Mutex<Option<u64>>,
    cancel_tx: &tokio::sync::watch::Sender<Option<CancelSignal>>,
    ordinary_queue: &OrdinaryQueue,
    writer: &WsWriterHandle,
) {
    use crate::rt_core::urgent::UrgentStopOutcome;
    let stop_res: Result<UrgentStopOutcome, ApiError> = stop_res.map_err(ApiError::from);
    let applied = match &stop_res {
        Ok(UrgentStopOutcome::Applied { epoch, stop_generation, .. }) => Some((*epoch, *stop_generation)),
        _ => None,
    };
    let cancelled = if let Some(drained) = pre_drained {
        // Legacy path already advanced owner `stop_gen`, signalled `cancel_tx`, and drained
        // `ordinary_queue` atomically under `inner.write()`.
        drained
    } else if let Some((epoch, stop_gen)) = applied {
        let _ = cancel_tx.send(Some(CancelSignal { gen: stop_gen, code: "STOPPED", epoch: Some(epoch) }));
        ordinary_queue.drain_cancellable("STOPPED", Some(epoch))
    } else {
        Vec::new()
    };
    for waiter in waiters {
        let invalid_req = UrgentStopOutcome::InvalidRequestId;
        let invalid_body;
        let per_waiter = match &stop_res {
            Ok(UrgentStopOutcome::Applied { .. }) if !waiter.request_id_valid => Ok(&invalid_req),
            Ok(UrgentStopOutcome::Applied { epoch, .. }) if waiter.body_error.is_some() => {
                invalid_body = UrgentStopOutcome::InvalidBody {
                    message: waiter.body_error.expect("checked above"),
                    epoch: *epoch,
                };
                Ok(&invalid_body)
            }
            Ok(other) => Ok(other),
            Err(api) => Err(api),
        };
        let reply = format_urgent_stop_outcome(
            &waiter.raw_request_id,
            waiter.parsed_request_id,
            last_request,
            per_waiter,
        );
        match waiter.permit {
            Some(permit) => { permit.send(reply); }
            None => { let _ = writer.try_send_stop_best_effort(reply); }
        }
    }
    for reply in cancelled {
        if !writer.send_ordinary_control(reply).await {
            break;
        }
    }
}

pub(super) struct AckSnapshot {
    pub(super) epoch_id: u64,
    pub(super) safety_state: crate::safety::SafetyState,
    pub(super) is_torqued: bool,
    pub(super) control_mode: &'static str,
}

impl AckSnapshot {
    pub(super) fn from_inner(inner: &SupervisorInner) -> Self {
        Self {
            epoch_id: inner.safety.epoch_id(),
            safety_state: inner.safety.state().clone(),
            is_torqued: inner.is_torqued,
            control_mode: if inner.gravity_comp_active() { "gravity_comp" } else { "position" },
        }
    }
}

pub(super) fn command_ack_from_snap(snap: &AckSnapshot, request_id: &Value, started: Started) -> Value {
    let mut ack = json!({
        "type": "command_ack",
        "request_id": request_id,
        "status": "accepted",
        "epoch_id": snap.epoch_id,
        "safety_state": &snap.safety_state,
        "is_torqued": snap.is_torqued,
        "control_mode": snap.control_mode,
    });
    if let Some((id, duration)) = started {
        ack["action_id"] = json!(id);
        ack["expected_duration_s"] = json!(duration);
    }
    ack
}

struct WsAuthState {
    auth: crate::auth::OperatorAuth,
    token: Option<String>,
    explicit_auth: bool,
    explicit_authed: bool,
}

#[derive(Clone)]
struct WsAuth(Arc<std::sync::RwLock<WsAuthState>>);

impl WsAuth {
    fn new(auth: crate::auth::OperatorAuth, token: Option<String>, explicit_auth: bool) -> Self {
        Self(Arc::new(std::sync::RwLock::new(WsAuthState {
            auth,
            token,
            explicit_auth,
            explicit_authed: false,
        })))
    }

    fn is_operator(&self) -> bool {
        let s = self.0.read().unwrap_or_else(|e| e.into_inner());
        if s.explicit_auth && !s.explicit_authed {
            return false;
        }
        !s.auth.is_enforced() || s.auth.is_operator(s.token.as_deref())
    }

    fn operator_grant(&self) -> crate::auth::OperatorGrant {
        let s = self.0.read().unwrap_or_else(|e| e.into_inner());
        if s.explicit_auth && !s.explicit_authed {
            return crate::auth::OperatorGrant::DENIED;
        }
        s.auth.operator_grant(s.token.as_deref())
    }

    fn is_enforced(&self) -> bool {
        self.0.read().unwrap_or_else(|e| e.into_inner()).auth.is_enforced()
    }

    fn auth_and_token(&self) -> (crate::auth::OperatorAuth, Option<String>) {
        let s = self.0.read().unwrap_or_else(|e| e.into_inner());
        (s.auth.clone(), s.token.clone())
    }

    fn update_token(&self, token: Option<String>) -> crate::auth::AuthStatus {
        let mut s = self.0.write().unwrap_or_else(|e| e.into_inner());
        s.token = token;
        let status = s.auth.status(s.token.as_deref());
        if s.explicit_auth {
            s.explicit_authed = !s.auth.is_enforced() || status.role == crate::auth::Role::Operator;
        }
        status
    }
}

/// Bound `auth` lane handler executed off-reader by `auth_task` via `DISPATCH_TABLE`.
fn handle_auth_frame(ws_auth: &WsAuth, value: &Value) -> Value {
    let next_token = value
        .get("token")
        .and_then(Value::as_str)
        .filter(|t| !t.is_empty())
        .map(str::to_string);
    let status = ws_auth.update_token(next_token);
    json!({
        "type": "auth_ack",
        "seq": value.get("seq"),
        "auth_enforced": status.auth_enforced,
        "role": status.role,
        "expires_in_s": status.expires_in_s
    })
}

/// Bound urgent E-stop reader transition handler (`estop` / `e_stop`).
/// Reserves urgent reply capacity before latching, always latches in O(1) via lock-free atomics,
/// and returns `true` iff the writer estop channel is already closed.
fn handle_urgent_estop_frame(ctx: &mut UrgentEStopCtx<'_>) -> bool {
    ctx.reader_writer.mark_estop_admitted();
    let estop_guard = ctx.reader_writer.enter_in_flight_estop();
    let reserved = ctx.reader_writer.try_reserve_estop();
    let closed = matches!(reserved, Err(mpsc::error::TrySendError::Closed(())));
    let ticket = ctx.reader_supervisor.begin_e_stop_checked();
    let req_id = ctx.value.get("request_id").cloned();
    let sup = ctx.reader_supervisor.clone();
    let cancel_tx = ctx.reader_cancel_tx.clone();
    let writer_c = ctx.reader_writer.clone();
    ctx.reader_tasks.spawn(async move {
        let _estop_guard = estop_guard;
        let (reply, cancel_epoch) = match sup.finish_e_stop_checked(ticket).await {
            Ok((state, epoch)) => (
                json!({"type":"epoch_sync", "epoch_id":epoch, "safety_state":state}),
                Some(epoch),
            ),
            Err(err) => {
                let api: ApiError = err.into();
                let mut r = json!({"type":"estop_reject", "status":api.status.as_u16(), "code":api.code, "error":api.error, "message":api.message, "latched":true});
                if let Some(id) = req_id {
                    r["request_id"] = id;
                }
                (r, sup.hub().latest().map(|s| s.obs.epoch_id))
            }
        };
        match reserved {
            Ok(permit) => {
                permit.send(reply);
            }
            Err(mpsc::error::TrySendError::Full(())) => {
                let _ = writer_c.send_estop_blocking(reply).await;
            }
            Err(mpsc::error::TrySendError::Closed(())) => {}
        }
        let _ = cancel_tx.send(Some(CancelSignal {
            gen: sup.stop_generation(),
            code: "ESTOP_ACTIVE",
            epoch: cancel_epoch,
        }));
    });
    closed
}

/// Bound urgent Stop reader transition handler (`/api/v1/motion/stop`).
/// Non-blocking handoff with coalescing; returns `true` iff the writer stop channel is closed.
fn handle_urgent_stop_frame(ctx: &mut UrgentStopCtx<'_>) -> bool {
    let permit = match ctx.reader_writer.try_reserve_stop() {
        Ok(p) => Some(p),
        Err(mpsc::error::TrySendError::Full(())) => {
            ctx.reader_writer.trigger_fatal(FatalClose::urgent_queue_full());
            None
        }
        Err(mpsc::error::TrySendError::Closed(())) => return true,
    };
    let local_gen = ctx.reader_generation.load(Ordering::SeqCst);
    let supplied_gen = ctx.value.get("generation").and_then(Value::as_u64).unwrap_or(0);
    let label_ok = {
        let lbl = ctx
            .reader_client_label
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        ctx.value
            .get("client_id")
            .is_none_or(|id| id.as_str() == lbl.as_deref())
    };
    let conn_label = crate::model::HolderLabel::try_new(ctx.reader_connection);
    let effective_gen = if conn_label.is_some()
        && local_gen != 0
        && supplied_gen == local_gen
        && label_ok
    {
        local_gen
    } else {
        0
    };
    let raw_request_id = ctx.value.get("request_id").cloned().unwrap_or(Value::Null);
    let parsed_request_id = ctx.value.get("request_id").and_then(Value::as_u64);
    let epoch = ctx.value.get("epoch_id").and_then(Value::as_u64);
    let body_error = validate_command_body_common(&ctx.value["body"]).err();

    let dispatch = {
        let last_req = last_request_get(ctx.reader_last_request);
        let mut st = ctx
            .reader_in_flight_stop
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        let high_water = match (
            last_req,
            st.active.as_ref().and_then(|a| a.pending_max_request_id),
        ) {
            (Some(a), Some(b)) => Some(a.max(b)),
            (a, b) => a.or(b),
        };
        let request_id_valid =
            parsed_request_id.is_some_and(|id| high_water.is_none_or(|last| id > last));
        let waiter = CoalescedStopWaiter {
            raw_request_id,
            parsed_request_id,
            request_id_valid,
            body_error,
            permit,
        };
        if let Some(active) = st.active.as_mut() {
            if active.lease_generation == effective_gen && active.epoch == epoch {
                if request_id_valid {
                    active.pending_max_request_id =
                        match (active.pending_max_request_id, parsed_request_id) {
                            (Some(a), Some(b)) => Some(a.max(b)),
                            (a, b) => a.or(b),
                        };
                }
                if waiter.permit.is_some() || active.waiters.len() < URGENT_OUT_CAPACITY {
                    active.waiters.push(waiter);
                }
                None
            } else {
                let stop = crate::rt_core::urgent::UrgentStop {
                    connection: conn_label.unwrap_or_default(),
                    lease_generation: effective_gen,
                    epoch,
                    request_id: parsed_request_id.unwrap_or(0),
                    request_id_valid,
                    body_error,
                };
                Some((None, stop, Some(waiter)))
            }
        } else {
            let stop = crate::rt_core::urgent::UrgentStop {
                connection: conn_label.unwrap_or_default(),
                lease_generation: effective_gen,
                epoch,
                request_id: parsed_request_id.unwrap_or(0),
                request_id_valid,
                body_error,
            };
            let can_lead = effective_gen != 0 && request_id_valid && body_error.is_none();
            if can_lead {
                st.next_leader_id += 1;
                let leader_id = st.next_leader_id;
                st.active = Some(ActiveStopBatch {
                    leader_id,
                    lease_generation: effective_gen,
                    epoch,
                    pending_max_request_id: parsed_request_id,
                    waiters: vec![waiter],
                });
                Some((Some(leader_id), stop, None))
            } else {
                Some((None, stop, Some(waiter)))
            }
        }
    };

    if let Some((leader_id, stop, single_waiter)) = dispatch {
        let take_waiters = |state: &Mutex<InFlightStopState>,
                            lid: Option<u64>,
                            single: Option<CoalescedStopWaiter>|
         -> Vec<CoalescedStopWaiter> {
            if let Some(id) = lid {
                let mut st = state.lock().unwrap_or_else(|e| e.into_inner());
                if st.active.as_ref().is_some_and(|a| a.leader_id == id) {
                    return st.active.take().map(|b| b.waiters).unwrap_or_default();
                }
                Vec::new()
            } else {
                single.into_iter().collect()
            }
        };
        match ctx.reader_supervisor.try_enqueue_executor_urgent_stop(stop) {
            Ok(Some((client, mut rx))) => {
                let stop_guard = ctx.reader_writer.enter_in_flight_stop();
                let in_flight_c = ctx.reader_in_flight_stop.clone();
                let last_req_c = ctx.reader_last_request.clone();
                let cancel_tx_c = ctx.reader_cancel_tx.clone();
                let ord_q_c = ctx.reader_ordinary_queue.clone();
                let writer_c = ctx.reader_writer.clone();
                ctx.reader_tasks.spawn(async move {
                    let _stop_guard = stop_guard;
                    let stop_res = client.await_ticket(&mut rx).await.map_err(ArmError::from);
                    let waiters = take_waiters(&in_flight_c, leader_id, single_waiter);
                    deliver_urgent_stop_batch(
                        waiters,
                        stop_res,
                        None,
                        &last_req_c,
                        &cancel_tx_c,
                        &ord_q_c,
                        &writer_c,
                    )
                    .await;
                });
            }
            Ok(None) => {
                let stop_guard = ctx.reader_writer.enter_in_flight_stop();
                let sup_c = ctx.reader_supervisor.clone();
                let in_flight_c = ctx.reader_in_flight_stop.clone();
                let last_req_c = ctx.reader_last_request.clone();
                let cancel_tx_c = ctx.reader_cancel_tx.clone();
                let ord_q_c = ctx.reader_ordinary_queue.clone();
                let writer_c = ctx.reader_writer.clone();
                ctx.reader_tasks.spawn(async move {
                    let _stop_guard = stop_guard;
                    let mut pre_drained: Option<Vec<Value>> = None;
                    let stop_res = sup_c
                        .urgent_stop_with_legacy_hook(stop, |applied_epoch, next_gen| {
                            // Owner `stop_gen` was already advanced under `inner.write()`;
                            // notify macro cancellation and drain the ordinary queue atomically.
                            let _ = cancel_tx_c.send(Some(CancelSignal {
                                gen: next_gen,
                                code: "STOPPED",
                                epoch: Some(applied_epoch),
                            }));
                            pre_drained =
                                Some(ord_q_c.drain_cancellable("STOPPED", Some(applied_epoch)));
                        })
                        .await;
                    let waiters = take_waiters(&in_flight_c, leader_id, single_waiter);
                    deliver_urgent_stop_batch(
                        waiters,
                        stop_res,
                        pre_drained,
                        &last_req_c,
                        &cancel_tx_c,
                        &ord_q_c,
                        &writer_c,
                    )
                    .await;
                });
            }
            Err(err) => {
                let api = ApiError::from(err);
                let waiters = take_waiters(ctx.reader_in_flight_stop, leader_id, single_waiter);
                for w in waiters {
                    let reply = format_urgent_stop_outcome(
                        &w.raw_request_id,
                        w.parsed_request_id,
                        ctx.reader_last_request,
                        Err(&api),
                    );
                    match w.permit {
                        Some(permit) => {
                            permit.send(reply);
                        }
                        None => {
                            let _ = ctx.reader_writer.try_send_stop_best_effort(reply);
                        }
                    }
                }
            }
        }
    }
    false
}

/// `command_ack` carrying the fields of `reply`.
fn fields_ack(request_id: &Value, reply: Value) -> Value {
    let mut ack = json!({"type":"command_ack", "request_id":request_id, "status":"accepted"});
    match reply {
        Value::Object(fields) => for (k, v) in fields { ack[k] = v; },
        other => ack["result"] = other,
    }
    ack
}

pub(super) fn command_reject_at(epoch_id: u64, request_id: &Value, error: ApiError) -> Value {
    let mut reply = rejection(error, "command_reject", epoch_id);
    reply["request_id"] = request_id.clone();
    reply
}

async fn handle_socket(socket: WebSocket, supervisor: Arc<ArmSupervisor>, deps: crate::grasp::GraspDeps, calib: crate::calibration::CalibrationDeps, sandbox: Arc<crate::api::sandbox::SandboxState>, obs_rate: Option<ObsRate>, auth: crate::auth::OperatorAuth, ws_token: Option<String>, explicit_auth: bool) {
    #[cfg(not(feature = "policy-runtime"))]
    let _ = &sandbox;
    let (mut sender, mut receiver) = socket.split();
    let mut telemetry = match supervisor.subscribe_telemetry() {
        Ok(rx) => rx,
        Err(e) => {
            tracing::error!("WebSocket refused: {e}");
            let _ = sender.close().await;
            return;
        }
    };

    // Two-tier writer queues:
    // - Urgent: revocation/preemption notices, E-stop replies, urgent Stop replies (reserved at admission),
    //   plus dedicated reserved `auth_ack` capacity isolated from ordinary backlog.
    // - Ordinary: control replies + action terminals (never dropped silently; overflow closes connection with error)
    //   and latest-wins non-terminal telemetry.
    let (revocation_tx, mut revocation_rx) = mpsc::channel::<Message>(REVOCATION_OUT_CAPACITY);
    let (estop_out_tx, mut estop_out_rx) = mpsc::channel::<Value>(URGENT_OUT_CAPACITY);
    let (stop_out_tx, mut stop_out_rx) = mpsc::channel::<Value>(URGENT_OUT_CAPACITY);
    let (auth_out_tx, mut auth_out_rx) = mpsc::channel::<Value>(AUTH_OUT_CAPACITY);
    let (auth_in_tx, mut auth_in_rx) = mpsc::channel::<Value>(AUTH_OUT_CAPACITY);
    let (ordinary_out_tx, mut ordinary_out_rx) = mpsc::channel::<Value>(ORDINARY_OUT_CAPACITY);
    let (fatal_tx, mut fatal_rx) = mpsc::channel::<FatalClose>(1);
    let fatal_triggered = Arc::new(AtomicBool::new(false));
    let fatal_notify = Arc::new(tokio::sync::Notify::new());
    let estop_admitted_count = Arc::new(AtomicU64::new(0));
    let in_flight_estops = Arc::new(AtomicUsize::new(0));
    let in_flight_stops = Arc::new(AtomicUsize::new(0));
    let urgent_done_notify = Arc::new(tokio::sync::Notify::new());
    let writer = WsWriterHandle {
        estop_tx: estop_out_tx,
        stop_tx: stop_out_tx,
        auth_tx: auth_out_tx,
        ordinary_tx: ordinary_out_tx,
        fatal_tx: fatal_tx.clone(),
        fatal_triggered: fatal_triggered.clone(),
        fatal_notify: fatal_notify.clone(),
        estop_admitted_count: estop_admitted_count.clone(),
        in_flight_estops: in_flight_estops.clone(),
        in_flight_stops: in_flight_stops.clone(),
        urgent_done_notify: urgent_done_notify.clone(),
    };

    // Reserve one off-thread WS connection slot (`<= MAX_LIVE_CONNECTIONS = 64`) and allocate a
    // strictly monotonic connection ID WITHOUT awaiting the RT executor:
    // - Rejected upgrades never consume an ID (no permanent holes in `ClosedConnections`).
    // - The cap applies uniformly to every retained socket, including degraded E-stop sockets.
    // - `reader_task` starts immediately so urgent E-stop can latch even if the executor is stalled.
    let conn_handle = match supervisor.try_claim_ws_slot() {
        Ok(h) => h,
        Err(err) => {
            let api = ApiError::from(err);
            let err_frame = json!({
                "type": "error",
                "status": api.status.as_u16(),
                "code": api.code,
                "error": api.error,
                "message": api.message,
            });
            let close = CloseFrame {
                code: 1013,
                reason: api
                    .code
                    .as_deref()
                    .unwrap_or("CONNECTION_REJECTED")
                    .to_owned()
                    .into(),
            };
            let _ = tokio::time::timeout(Duration::from_millis(100), async {
                let _ = sender.send(Message::Text(err_frame.to_string().into())).await;
                let _ = sender.send(Message::Close(Some(close))).await;
                let _ = sender.flush().await;
                let _ = sender.close().await;
            })
            .await;
            return;
        }
    };
    let conn_id = conn_handle.id();
    let connection = conn_handle.as_str().to_owned();
    let connection_generation = Arc::new(AtomicU64::new(0));

    struct WsSlotGuard {
        supervisor: Arc<ArmSupervisor>,
        conn: crate::rt_core::requests::lease::ConnectionHandle,
        generation: Arc<AtomicU64>,
        disarmed: bool,
    }
    impl Drop for WsSlotGuard {
        fn drop(&mut self) {
            if !self.disarmed {
                let closing_gen = self.generation.swap(0, Ordering::SeqCst);
                self.supervisor
                    .fence_ws_connection_immediate(self.conn, closing_gen);
                self.supervisor.finish_ws_connection_detached(self.conn);
            }
        }
    }
    let mut ws_slot_guard = WsSlotGuard {
        supervisor: supervisor.clone(),
        conn: conn_handle,
        generation: connection_generation.clone(),
        disarmed: false,
    };

    // Register this admitted slot with the active owner concurrently so `reader_task` is live
    // immediately for urgent E-stop while `worker_task` and `heartbeat_task` gate ordinary/lease
    // traffic until owner registration settles (`Some(true)` registered, `Some(false)` stopped/degraded).
    let (reg_tx, reg_rx) = tokio::sync::watch::channel::<Option<bool>>(None);
    let reg_supervisor = supervisor.clone();
    let mut reg_task = tokio::spawn(async move {
        let ok = reg_supervisor
            .register_ws_connection_for_slot(conn_id)
            .await
            .is_ok();
        let _ = reg_tx.send(Some(ok));
    });

    let client_label: Arc<Mutex<Option<String>>> = Arc::new(Mutex::new(None));
    // Urgent Stop request_id ordering is tracked independently of ordinary RPCs so an urgent Stop
    // overtaking queued ordinary requests never causes earlier ordinary IDs to fail INVALID_COMMAND.
    let last_urgent_stop_request: Arc<Mutex<Option<u64>>> = Arc::new(Mutex::new(None));
    let in_flight_stop = Arc::new(Mutex::new(InFlightStopState::default()));
    let ordinary_queue = OrdinaryQueue::new();
    let heartbeat_slot = HeartbeatSlot::default();
    let ws_auth = WsAuth::new(auth, ws_token, explicit_auth);
    let tasks = ConnectionTasks::new();
    let connection_closing = Arc::new(AtomicBool::new(false));
    let (cancel_tx, mut cancel_rx) = tokio::sync::watch::channel::<Option<CancelSignal>>(None);

    #[cfg(feature = "policy-runtime")]
    let ws_closed = Arc::new(AtomicBool::new(false));
    #[cfg(feature = "policy-runtime")]
    struct WsClosedGuard(Arc<AtomicBool>);
    #[cfg(feature = "policy-runtime")]
    impl Drop for WsClosedGuard {
        fn drop(&mut self) {
            self.0.store(true, Ordering::SeqCst);
        }
    }
    #[cfg(feature = "policy-runtime")]
    let _ws_closed_guard = WsClosedGuard(ws_closed.clone());

    {
        let mut reg = CONNECTION_REGISTRY.lock().unwrap_or_else(|e| e.into_inner());
        reg.insert(connection.clone(), revocation_tx);
    }
    {
        let mut closers = CONNECTION_CLOSERS.lock().unwrap_or_else(|e| e.into_inner());
        closers.insert(connection.clone(), (fatal_tx, fatal_triggered.clone(), fatal_notify.clone()));
    }

    struct ConnectionGuard(String);
    impl Drop for ConnectionGuard {
        fn drop(&mut self) {
            let mut reg = CONNECTION_REGISTRY.lock().unwrap_or_else(|e| e.into_inner());
            reg.remove(&self.0);
            let mut closers = CONNECTION_CLOSERS.lock().unwrap_or_else(|e| e.into_inner());
            closers.remove(&self.0);
        }
    }
    let _conn_guard = ConnectionGuard(connection.clone());

    // Route unseen action terminals into this connection's lossless ordinary control queue
    // (never dropped by telemetry broadcast lag; overflow closes the connection with an error).
    // Returning `false` while `command_reply_pending` is set keeps terminals at the head of
    // `TerminalBroadcaster`'s bounded FIFO (and gates telemetry broadcast) until the in-flight
    // command's own reply is enqueued on `ordinary_tx`.
    let command_reply_pending = Arc::new(AtomicBool::new(false));
    let term_writer = writer.clone();
    let term_pending = Arc::clone(&command_reply_pending);
    let _terminal_sub = supervisor.subscribe_action_terminals(move |obs| {
        if term_pending.load(Ordering::Acquire) {
            return false;
        }
        let Ok(val) = serde_json::to_value(obs) else { return true; };
        term_writer.try_send_ordinary_control(val)
    });

    // Task 3: Writer task with urgent-first priority, unified grant-before-loss deferral across
    // ALL lease_lost sources (revocation + heartbeat + worker), monotonic wire epoch guard,
    // lossless action-terminal delivery via `ordinary_out_rx`, telemetry resync on broadcast lag,
    // explicit error close on control queue saturation (<= 1 s grace, flushing queued urgent
    // replies BEFORE fatal error + Close), and bounded write timeout.
    let writer_connection = connection.clone();
    let writer_supervisor = supervisor.clone();
    let writer_fatal_triggered = fatal_triggered.clone();
    let writer_fatal_notify = fatal_notify.clone();
    let writer_estop_admitted = estop_admitted_count.clone();
    let writer_in_flight_estops = in_flight_estops.clone();
    let writer_in_flight_stops = in_flight_stops.clone();
    let writer_urgent_notify = urgent_done_notify.clone();
    let mut writer_task = tokio::spawn(async move {
        let mut last_sent: Option<f64> = None;
        let mut last_wire_epoch: Option<(u64, Option<Value>)> = None;
        let mut max_granted_gen: u64 = 0;
        let mut estop_replies_delivered: u64 = 0;
        let mut deferred_losses: VecDeque<(u64, Message)> = VecDeque::new();
        let mut pending_latest_telemetry: Option<crate::model::ArmObservation> = None;
        let (mut rev_closed, mut estop_closed, mut stop_closed, mut auth_closed, mut ord_closed) =
            (false, false, false, false, false);
        loop {
            if rev_closed && estop_closed && stop_closed && auth_closed && ord_closed {
                break;
            }
            tokio::select! {
                biased;
                msg = revocation_rx.recv(), if !rev_closed => match msg {
                    Some(msg) => {
                        if let Some(loss_gen) = extract_lease_loss_gen(&msg).filter(|&g| g > max_granted_gen) {
                            // Causal order: hold any lease_lost until its lease_granted is written.
                            if deferred_losses.len() >= ORDINARY_OUT_CAPACITY {
                                flush_urgent_then_fatal_and_close(
                                    &writer_connection,
                                    &mut sender,
                                    &FatalClose::outbound_queue_full(),
                                    &writer_fatal_triggered,
                                    &writer_fatal_notify,
                                    &mut last_wire_epoch,
                                    &mut estop_out_rx,
                                    &mut stop_out_rx,
                                    &writer_estop_admitted,
                                    &mut estop_replies_delivered,
                                    &writer_in_flight_estops,
                                    &writer_in_flight_stops,
                                    &writer_urgent_notify,
                                )
                                .await;
                                break;
                            }
                            deferred_losses.push_back((loss_gen, msg));
                        } else if let Err(e) = write_ws_frame_bounded(&writer_connection, &mut sender, msg, ws_write_timeout(&writer_connection), &writer_fatal_triggered, &writer_fatal_notify).await {
                            if e == WriteErr::Timeout {
                                let fatal = fatal_rx.try_recv().unwrap_or_else(|_| FatalClose::write_timeout());
                                flush_urgent_then_fatal_and_close(
                                    &writer_connection, &mut sender, &fatal, &writer_fatal_triggered, &writer_fatal_notify, &mut last_wire_epoch,
                                    &mut estop_out_rx, &mut stop_out_rx, &writer_estop_admitted,
                                    &mut estop_replies_delivered, &writer_in_flight_estops,
                                    &writer_in_flight_stops, &writer_urgent_notify,
                                ).await;
                            }
                            break;
                        }
                    }
                    None => { rev_closed = true; }
                },
                reply = estop_out_rx.recv(), if !estop_closed => match reply {
                    Some(mut val) => {
                        enforce_monotonic_epoch(&mut last_wire_epoch, &mut val);
                        match write_ws_frame_bounded(&writer_connection, &mut sender, Message::Text(val.to_string().into()), ws_write_timeout(&writer_connection), &writer_fatal_triggered, &writer_fatal_notify).await {
                            Ok(()) => { estop_replies_delivered += 1; }
                            Err(e) => {
                                if e == WriteErr::Timeout {
                                    let fatal = fatal_rx.try_recv().unwrap_or_else(|_| FatalClose::write_timeout());
                                    flush_urgent_then_fatal_and_close(
                                        &writer_connection, &mut sender, &fatal, &writer_fatal_triggered, &writer_fatal_notify, &mut last_wire_epoch,
                                        &mut estop_out_rx, &mut stop_out_rx, &writer_estop_admitted,
                                        &mut estop_replies_delivered, &writer_in_flight_estops,
                                        &writer_in_flight_stops, &writer_urgent_notify,
                                    ).await;
                                }
                                break;
                            }
                        }
                    }
                    None => { estop_closed = true; }
                },
                reply = stop_out_rx.recv(), if !stop_closed => match reply {
                    Some(mut val) => {
                        enforce_monotonic_epoch(&mut last_wire_epoch, &mut val);
                        if let Err(e) = write_ws_frame_bounded(&writer_connection, &mut sender, Message::Text(val.to_string().into()), ws_write_timeout(&writer_connection), &writer_fatal_triggered, &writer_fatal_notify).await {
                            if e == WriteErr::Timeout {
                                let fatal = fatal_rx.try_recv().unwrap_or_else(|_| FatalClose::write_timeout());
                                flush_urgent_then_fatal_and_close(
                                    &writer_connection, &mut sender, &fatal, &writer_fatal_triggered, &writer_fatal_notify, &mut last_wire_epoch,
                                    &mut estop_out_rx, &mut stop_out_rx, &writer_estop_admitted,
                                    &mut estop_replies_delivered, &writer_in_flight_estops,
                                    &writer_in_flight_stops, &writer_urgent_notify,
                                ).await;
                            }
                            break;
                        }
                    }
                    None => { stop_closed = true; }
                },
                fatal = fatal_rx.recv() => match fatal {
                    Some(fatal) => {
                        flush_urgent_then_fatal_and_close(
                            &writer_connection,
                            &mut sender,
                            &fatal,
                            &writer_fatal_triggered,
                            &writer_fatal_notify,
                            &mut last_wire_epoch,
                            &mut estop_out_rx,
                            &mut stop_out_rx,
                            &writer_estop_admitted,
                            &mut estop_replies_delivered,
                            &writer_in_flight_estops,
                            &writer_in_flight_stops,
                            &writer_urgent_notify,
                        )
                        .await;
                        break;
                    }
                    None => break,
                },
                reply = auth_out_rx.recv(), if !auth_closed => match reply {
                    Some(val) => {
                        if let Err(e) = write_ws_frame_bounded(&writer_connection, &mut sender, Message::Text(val.to_string().into()), ws_write_timeout(&writer_connection), &writer_fatal_triggered, &writer_fatal_notify).await {
                            if e == WriteErr::Timeout {
                                let fatal = fatal_rx.try_recv().unwrap_or_else(|_| FatalClose::write_timeout());
                                flush_urgent_then_fatal_and_close(
                                    &writer_connection, &mut sender, &fatal, &writer_fatal_triggered, &writer_fatal_notify, &mut last_wire_epoch,
                                    &mut estop_out_rx, &mut stop_out_rx, &writer_estop_admitted,
                                    &mut estop_replies_delivered, &writer_in_flight_estops,
                                    &writer_in_flight_stops, &writer_urgent_notify,
                                ).await;
                            }
                            break;
                        }
                    }
                    None => { auth_closed = true; }
                },
                reply = ordinary_out_rx.recv(), if !ord_closed => match reply {
                    Some(mut val) => {
                        // Unified grant-before-loss deferral: losses from heartbeat_task or
                        // worker_task follow the exact same causal rule as revocation_rx.
                        if let Some(loss_gen) = extract_lease_loss_gen_value(&val).filter(|&g| g > max_granted_gen) {
                            if deferred_losses.len() >= ORDINARY_OUT_CAPACITY {
                                flush_urgent_then_fatal_and_close(
                                    &writer_connection,
                                    &mut sender,
                                    &FatalClose::outbound_queue_full(),
                                    &writer_fatal_triggered,
                                    &writer_fatal_notify,
                                    &mut last_wire_epoch,
                                    &mut estop_out_rx,
                                    &mut stop_out_rx,
                                    &writer_estop_admitted,
                                    &mut estop_replies_delivered,
                                    &writer_in_flight_estops,
                                    &writer_in_flight_stops,
                                    &writer_urgent_notify,
                                )
                                .await;
                                break;
                            }
                            deferred_losses.push_back((loss_gen, Message::Text(val.to_string().into())));
                            continue;
                        }
                        let is_control_msg = val.get("type").is_some();
                        let granted_gen = if val.get("type").and_then(Value::as_str) == Some("lease_granted") {
                            val.get("generation").and_then(Value::as_u64)
                        } else {
                            None
                        };
                        if is_control_msg {
                            enforce_monotonic_epoch(&mut last_wire_epoch, &mut val);
                        }
                        if let Err(e) = write_ws_frame_bounded(&writer_connection, &mut sender, Message::Text(val.to_string().into()), ws_write_timeout(&writer_connection), &writer_fatal_triggered, &writer_fatal_notify).await {
                            if e == WriteErr::Timeout {
                                let fatal = fatal_rx.try_recv().unwrap_or_else(|_| FatalClose::write_timeout());
                                flush_urgent_then_fatal_and_close(
                                    &writer_connection, &mut sender, &fatal, &writer_fatal_triggered, &writer_fatal_notify, &mut last_wire_epoch,
                                    &mut estop_out_rx, &mut stop_out_rx, &writer_estop_admitted,
                                    &mut estop_replies_delivered, &writer_in_flight_estops,
                                    &writer_in_flight_stops, &writer_urgent_notify,
                                ).await;
                            }
                            break;
                        }
                        if let Some(g) = granted_gen {
                            max_granted_gen = max_granted_gen.max(g);
                            let mut write_failed = None;
                            let n = deferred_losses.len();
                            for _ in 0..n {
                                if let Some((lg, lmsg)) = deferred_losses.pop_front() {
                                    if lg <= max_granted_gen {
                                        if let Err(e) = write_ws_frame_bounded(&writer_connection, &mut sender, lmsg, ws_write_timeout(&writer_connection), &writer_fatal_triggered, &writer_fatal_notify).await {
                                            write_failed = Some(e);
                                            break;
                                        }
                                    } else {
                                        deferred_losses.push_back((lg, lmsg));
                                    }
                                }
                            }
                            if let Some(e) = write_failed {
                                if e == WriteErr::Timeout {
                                    let fatal = fatal_rx.try_recv().unwrap_or_else(|_| FatalClose::write_timeout());
                                    flush_urgent_then_fatal_and_close(
                                        &writer_connection, &mut sender, &fatal, &writer_fatal_triggered, &writer_fatal_notify, &mut last_wire_epoch,
                                        &mut estop_out_rx, &mut stop_out_rx, &writer_estop_admitted,
                                        &mut estop_replies_delivered, &writer_in_flight_estops,
                                        &writer_in_flight_stops, &writer_urgent_notify,
                                    ).await;
                                }
                                break;
                            }
                        }
                    }
                    None => { ord_closed = true; }
                },
                _ = std::future::ready(()), if pending_latest_telemetry.is_some() => {
                    if let Some(obs) = pending_latest_telemetry.take() {
                        if let Some(rate) = obs_rate {
                            if !rate.due(last_sent, obs.timestamp) { continue; }
                            last_sent = Some(obs.timestamp);
                        }
                        if let Ok(text) = serde_json::to_string(&obs) {
                            if let Err(e) = write_ws_frame_bounded(&writer_connection, &mut sender, Message::Text(text.into()), ws_write_timeout(&writer_connection), &writer_fatal_triggered, &writer_fatal_notify).await {
                                if e == WriteErr::Timeout {
                                    let fatal = fatal_rx.try_recv().unwrap_or_else(|_| FatalClose::write_timeout());
                                    flush_urgent_then_fatal_and_close(
                                        &writer_connection, &mut sender, &fatal, &writer_fatal_triggered, &writer_fatal_notify, &mut last_wire_epoch,
                                        &mut estop_out_rx, &mut stop_out_rx, &writer_estop_admitted,
                                        &mut estop_replies_delivered, &writer_in_flight_estops,
                                        &writer_in_flight_stops, &writer_urgent_notify,
                                    ).await;
                                }
                                break;
                            }
                        }
                    }
                },
                observation = telemetry.recv() => match observation {
                    Ok(first) => {
                        pending_latest_telemetry = Some(first);
                        loop {
                            match telemetry.try_recv() {
                                Ok(newer) => pending_latest_telemetry = Some(newer),
                                Err(tokio::sync::broadcast::error::TryRecvError::Lagged(_)) => {
                                    if let Some(s) = writer_supervisor.hub().latest() {
                                        pending_latest_telemetry = Some(s.obs.clone());
                                    }
                                }
                                Err(_) => break,
                            }
                        }
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => {
                        // Action terminals already went through `ordinary_out_rx`; resync
                        // latest telemetry snapshot from the supervisor hub.
                        if let Some(s) = writer_supervisor.hub().latest() {
                            pending_latest_telemetry = Some(s.obs.clone());
                        }
                    }
                    Err(_) => break,
                }
            }
        }
    });

    // Dedicated off-reader auth verification task so `OperatorAuth::status` session-table
    // pruning never stalls urgent E-stop/Stop frame classification in `reader_task`.
    let auth_ws_auth = ws_auth.clone();
    let auth_writer = writer.clone();
    let auth_task = tokio::spawn(async move {
        let Some(DispatchHandler::Auth(auth_handler)) =
            dispatch::lookup(FrameKind::Auth, None).map(|e| e.handler)
        else {
            unreachable!("FrameKind::Auth must bind DispatchHandler::Auth");
        };
        while let Some(value) = auth_in_rx.recv().await {
            let reply = auth_handler(&auth_ws_auth, &value);
            if !auth_writer.send_auth(reply).await {
                break;
            }
        }
    });

    // Dedicated latest-wins heartbeat processor isolated from ordinary backlog and slow worker IO.
    let hb_supervisor = supervisor.clone();
    let hb_conn_handle = conn_handle;
    let hb_generation = connection_generation.clone();
    let hb_client_label = client_label.clone();
    let hb_slot = heartbeat_slot.clone();
    let hb_writer = writer.clone();
    let mut hb_reg_rx = reg_rx.clone();
    let heartbeat_task = tokio::spawn(async move {
        let Some(DispatchHandler::Heartbeat(hb_handler)) =
            dispatch::lookup(FrameKind::Heartbeat, None).map(|e| e.handler)
        else {
            unreachable!("FrameKind::Heartbeat must bind DispatchHandler::Heartbeat");
        };
        while let Some(value) = hb_slot.pop().await {
            if hb_reg_rx.borrow().is_none() {
                let _ = hb_reg_rx.wait_for(|v| v.is_some()).await;
            }
            let reply = hb_handler(
                &hb_supervisor,
                &value,
                hb_conn_handle,
                &hb_generation,
                &hb_client_label,
            )
            .await;
            if let Some(reply) = reply {
                if !hb_writer.send_ordinary_control(reply).await {
                    break;
                }
            }
        }
    });

    // Task 1: Reader task. Only reads frames and classifies them; never awaits planning, IO,
    // detector, supervisor lock, or channel capacity.
    let reader_supervisor = supervisor.clone();
    let reader_conn_id = conn_id;
    let reader_connection = connection.clone();
    let reader_generation = connection_generation.clone();
    let reader_client_label = client_label.clone();
    let reader_last_request = last_urgent_stop_request.clone();
    let reader_in_flight_stop = in_flight_stop.clone();
    let reader_ordinary_queue = ordinary_queue.clone();
    let reader_heartbeat_slot = heartbeat_slot.clone();
    let reader_auth_in_tx = auth_in_tx;
    let reader_tasks = tasks.clone();
    let reader_cancel_tx = cancel_tx.clone();
    let reader_writer = writer.clone();
    let mut reader_task = tokio::spawn(async move {
        loop {
            let Some(Ok(message)) = receiver.next().await else { break; };
            let Message::Text(text) = message else {
                if matches!(message, Message::Close(_)) { break; }
                continue;
            };
            let Ok(value) = serde_json::from_str::<Value>(&text) else { continue; };
            let (_, entry) = dispatch::classify(&value);
            let admission = entry.map_or(AdmissionClass::Ordinary, |e| e.admission);

            let is_estop = admission == AdmissionClass::UrgentEStop;
            let is_stop = admission == AdmissionClass::UrgentStop;

            // When fatal close is already in progress, keep draining inbound bytes so closing the
            // TCP fd does not emit RST over unread flood traffic; still latch urgent E-stop and
            // coalesce/apply urgent Stop until the writer finishes its bounded close.
            if reader_writer.is_fatal() && !is_estop && !is_stop {
                continue;
            }

            // `auth` is classified in O(1) on the reader and verified off-reader by `auth_task`
            // so `OperatorAuth::sessions` table scans never stall urgent E-stop/Stop intake.
            if admission == AdmissionClass::Auth {
                match reader_auth_in_tx.try_send(value) {
                    Ok(()) => {}
                    Err(mpsc::error::TrySendError::Full(_)) => {
                        reader_writer.trigger_fatal(FatalClose::outbound_queue_full());
                    }
                    Err(mpsc::error::TrySendError::Closed(_)) => {
                        // Never exit reader just because auth_task stopped on reply-queue capacity;
                        // only exit if writer_task itself has already closed.
                        if reader_writer.is_writer_closed() {
                            break;
                        }
                    }
                }
                continue;
            }

            // `heartbeat` uses its own latest-wins slot so an ordinary backlog or slow worker
            // task never drops lease renewals.
            if admission == AdmissionClass::Heartbeat {
                reader_heartbeat_slot.push_latest(value);
                continue;
            }

            // Urgent E-stop (both aliases): dispatched through bound table handler.
            if let Some(DispatchHandler::UrgentEStop(handler)) = entry.map(|e| e.handler) {
                let mut ctx = UrgentEStopCtx {
                    value: &value,
                    reader_supervisor: &reader_supervisor,
                    reader_cancel_tx: &reader_cancel_tx,
                    reader_writer: &reader_writer,
                    reader_tasks: &reader_tasks,
                };
                if handler(&mut ctx) {
                    break;
                }
                continue;
            }

            // Urgent Stop (`/api/v1/motion/stop`): dispatched through bound table handler.
            if let Some(DispatchHandler::UrgentStop(handler)) = entry.map(|e| e.handler) {
                let mut ctx = UrgentStopCtx {
                    value: &value,
                    reader_supervisor: &reader_supervisor,
                    reader_connection: &reader_connection,
                    reader_generation: &reader_generation,
                    reader_client_label: &reader_client_label,
                    reader_last_request: &reader_last_request,
                    reader_in_flight_stop: &reader_in_flight_stop,
                    reader_ordinary_queue: &reader_ordinary_queue,
                    reader_cancel_tx: &reader_cancel_tx,
                    reader_writer: &reader_writer,
                    reader_tasks: &reader_tasks,
                };
                if handler(&mut ctx) {
                    break;
                }
                continue;
            }

            // Capture AdmissionStamp and ingress timestamp ONCE in the reader before queuing
            // (never re-sampled downstream).
            let received_at = Instant::now();
            let stamp = crate::supervisor::AdmissionStamp {
                stop_gen: reader_supervisor.stop_generation(),
                estop_epoch: value
                    .get("epoch_id")
                    .and_then(Value::as_u64)
                    .unwrap_or_else(|| reader_supervisor.estop_epoch()),
                lease_gen: value
                    .get("generation")
                    .and_then(Value::as_u64)
                    .unwrap_or_else(|| reader_generation.load(Ordering::SeqCst)),
                connection_id: reader_conn_id,
                request_id: crate::supervisor::next_admission_request_id(),
            };
            // Every other frame goes onto the connection's bounded ordinary queue (non-blocking).
            if let Err(value) = reader_ordinary_queue.try_push(value, stamp, received_at) {
                if let Some(reply) = backlog_full_reject(&value) {
                    reader_writer.try_send_ordinary_control(reply);
                    if reader_writer.is_writer_closed() {
                        break;
                    }
                }
            }
        }
    });

    // Task 2: Ordinary worker task. Processes ordinary commands one at a time, in order, and
    // may await planning, replay, dry-run, and the detector without blocking reader/urgent paths.
    let recv_supervisor = supervisor.clone();
    let recv_conn_id = conn_id;
    let recv_conn_handle = conn_handle;
    let recv_connection = connection.clone();
    let recv_generation = connection_generation.clone();
    let recv_client_label = client_label.clone();
    let worker_ordinary_queue = ordinary_queue.clone();
    let worker_writer = writer.clone();
    let worker_command_reply_pending = Arc::clone(&command_reply_pending);
    let worker_tasks = tasks.clone();
    let worker_closing = connection_closing.clone();
    let recv_deps = deps;
    let recv_calib = calib;
    #[cfg(feature = "policy-runtime")]
    let recv_sandbox = sandbox;
    #[cfg(feature = "policy-runtime")]
    let worker_ws_closed = ws_closed.clone();
    let worker_ws_auth = ws_auth.clone();
    let mut worker_reg_rx = reg_rx;
    let worker_task = tokio::spawn(async move {
        // the operator session of this connection: from the upgrade request, updated by `auth` messages in reader
        let ws_auth = worker_ws_auth;
        let mut last_sequence: Option<u64> = None;
        let mut streaming_servo = crate::supervisor::targets::ServoStream::new();
        let mut cmd_servo = crate::supervisor::targets::ServoStream::new();
        let servo_reply_sink: Arc<dyn Fn(Value) + Send + Sync> = {
            let w = worker_writer.clone();
            Arc::new(move |reply| {
                let _ = w.try_send_ordinary_control(reply);
            })
        };
        let mut last_ordinary_request: Option<u64> = None;
        // Delayed acks (sent after a state-file write) come back here and are stamped with
        // the current epoch and enqueued; the writer task enforces wire epoch monotonicity.
        let (ack_tx, mut ack_rx) = mpsc::channel::<Value>(64);
        #[cfg(feature = "policy-runtime")]
        let pending_policy_loads = Arc::new(AtomicUsize::new(0));
        let mut macro_check: Option<MacroCheck> = None;
        loop {
            let macro_active = macro_check.is_some();
            let queued = tokio::select! {
                biased;
                Ok(()) = cancel_rx.changed() => {
                    let sig = *cancel_rx.borrow_and_update();
                    if let Some(sig) = sig {
                        if macro_check.as_ref().is_some_and(|c| sig.code == "ESTOP_ACTIVE" || sig.gen > c.stamp.stop_gen) {
                            let check = macro_check.take().expect("checked above");
                            check.abort.0.abort();
                            let mut r = json!({"type":"command_reject", "request_id":check.request_id, "status":409, "code":sig.code, "error":sig.code, "message":"Stopped before the action began"});
                            if let Some(ep) = sig.epoch { r["epoch_id"] = json!(ep); }
                            if !worker_writer.send_ordinary_control(r).await { break; }
                            let mut failed = false;
                            for queued_reply in worker_ordinary_queue.drain_cancellable(sig.code, sig.epoch) {
                                if !worker_writer.send_ordinary_control(queued_reply).await {
                                    failed = true;
                                    break;
                                }
                            }
                            if failed { break; }
                        }
                    }
                    continue;
                }
                Some(mut reply) = ack_rx.recv() => {
                    if let Ok(s) = recv_supervisor.fresh_snapshot(None).await {
                        reply["epoch_id"] = json!(s.obs.epoch_id);
                        reply["safety_state"] = json!(s.obs.safety_state);
                        reply["is_torqued"] = json!(s.obs.is_torqued);
                    }
                    test_ack_stamp_delay().await;
                    if !worker_writer.send_ordinary_control(reply).await { break; }
                    continue;
                }
                joined = async { (&mut macro_check.as_mut().expect("guarded by the precondition").rx).await }, if macro_active => {
                    let MacroCheck { request_id, generation, stamp, .. } = macro_check.take().expect("checked above");
                    let Some((reply, macro_gate)) = actions::complete_macro_check(
                        request_id,
                        generation,
                        stamp,
                        joined,
                        &recv_supervisor,
                        &recv_connection,
                        &recv_generation,
                        &recv_deps,
                        &worker_closing,
                        &worker_command_reply_pending,
                    ).await else { break; };
                    if reply.get("type").and_then(Value::as_str) == Some("command_ack") {
                        let delay_ms = TEST_DEFERRED_ACK_DELAY_MS.load(Ordering::Relaxed);
                        if delay_ms > 0 {
                            tokio::time::sleep(Duration::from_millis(delay_ms)).await;
                        }
                    }
                    let sent = worker_writer.send_ordinary_control(reply).await;
                    drop(macro_gate);
                    if !sent { break; }
                    continue;
                }
                item = worker_ordinary_queue.pop(macro_active) => match item {
                    Some(q) => q,
                    None => break,
                }
            };
            let QueuedOrdinary {
                value,
                stamp,
                received_at,
            } = queued;
            if worker_reg_rx.borrow().is_none() {
                let _ = worker_reg_rx.wait_for(|v| v.is_some()).await;
            }
            let raw_kind = value.get("type").and_then(Value::as_str).unwrap_or("");
            let (frame_kind, entry) = dispatch::classify(&value);
            // Unified table-bound async session handlers (`acquire_lease`, `release_lease`)
            // dispatched strictly through `entry.handler` (single authority in `dispatch.rs`).
            let owner_mode = recv_supervisor.owner_mode();
            if let Some(DispatchHandler::SessionAsync(handler)) = entry.map(|e| e.handler) {
                if matches!(
                    owner_mode,
                    crate::supervisor::OWNER_LEGACY | crate::supervisor::OWNER_EXECUTOR
                ) {
                    maybe_pause_worker_before_lock(&recv_connection).await;
                    let (response, evicted_notify) = handler(dispatch::SessionDispatchCtx {
                        supervisor: &recv_supervisor,
                        value: &value,
                        conn_handle: recv_conn_handle,
                        recv_generation: &recv_generation,
                        recv_client_label: &recv_client_label,
                        worker_closing: &worker_closing,
                        ws_auth: &ws_auth,
                        last_sequence: &mut last_sequence,
                    })
                    .await;
                    if let Some((evicted_conn, evicted_gen)) = evicted_notify {
                        lease::notify_evicted(&evicted_conn, evicted_gen);
                    }
                    if let Some(response) = response {
                        if response.get("type").and_then(Value::as_str) == Some("lease_granted") {
                            if let Some(g) = response.get("generation").and_then(Value::as_u64) {
                                streaming_servo.lease_granted(g);
                                cmd_servo.lease_granted(g);
                            }
                            let delay_ms = TEST_LEASE_GRANTED_DELAY_MS.load(Ordering::Relaxed);
                            if delay_ms > 0 {
                                tokio::time::sleep(Duration::from_millis(delay_ms)).await;
                            }
                        }
                        if !worker_writer.send_ordinary_control(response).await {
                            break;
                        }
                    }
                    continue;
                }
            }
            // Unified table-bound async command handlers (`/api/v1/torque`, `/api/v1/arm`,
            // `/api/v1/safety/clear_fault`) routed identically across legacy and RT executor.
            // Both `entry.lease` and `entry.auth` are enforced directly from the dispatch table row.
            if let Some((lease_req, auth_req, DispatchHandler::CommandAsync(handler))) =
                entry.map(|e| (e.lease, e.auth, e.handler))
            {
                if matches!(
                    owner_mode,
                    crate::supervisor::OWNER_LEGACY | crate::supervisor::OWNER_EXECUTOR
                ) {
                    let local_generation = match lease_req {
                        LeaseRequirement::Required => {
                            let Some(gen) = lease::check_local_command_lease(
                                &value,
                                &recv_generation,
                                &recv_client_label,
                            ) else {
                                let req_id_val = value.get("request_id").unwrap_or(&Value::Null);
                                let epoch_id = lease::current_reject_epoch(&recv_supervisor).await;
                                let response =
                                    lease::format_command_lease_lost(req_id_val, epoch_id);
                                if !worker_writer.send_ordinary_control(response).await {
                                    break;
                                }
                                continue;
                            };
                            gen
                        }
                        LeaseRequirement::None => 0,
                        _ => unreachable!("validate_entry restricts CommandAsync lease policy"),
                    };
                    let auth_policy = auth_req
                        .owner_auth_policy()
                        .expect("validate_entry restricts CommandAsync auth policy");
                    let operator = ws_auth.operator_grant();
                    maybe_pause_worker_before_lock(&recv_connection).await;
                    let response = handler(dispatch::CommandAsyncCtx {
                        supervisor: &recv_supervisor,
                        value: &value,
                        stamp,
                        conn_handle: recv_conn_handle,
                        local_generation,
                        auth_policy,
                        operator,
                        last_ordinary_request: &mut last_ordinary_request,
                    })
                    .await;
                    if !worker_writer.send_ordinary_control(response).await {
                        break;
                    }
                    continue;
                }
            }
            // Fail closed before any legacy lock/reader path (including depth/teach) whenever
            // `ArmCore` is not owned by the legacy supervisor lock.
            if owner_mode != crate::supervisor::OWNER_LEGACY {
                if owner_mode == crate::supervisor::OWNER_EXECUTOR {
                    if let Some(opt_reply) = servo::handle_executor_ws_servo(
                        frame_kind,
                        &value,
                        stamp,
                        received_at,
                        recv_conn_id,
                        &recv_connection,
                        &recv_generation,
                        &recv_client_label,
                        &mut streaming_servo,
                        &mut cmd_servo,
                        &mut last_sequence,
                        &mut last_ordinary_request,
                        &recv_supervisor,
                        &servo_reply_sink,
                    )
                    .await
                    {
                        if let Some(reply) = opt_reply {
                            if !worker_writer.send_ordinary_control(reply).await {
                                break;
                            }
                        }
                        continue;
                    }
                    if let Some(opt_reply) = config::handle_executor_ws_command(
                        frame_kind,
                        entry,
                        &value,
                        stamp,
                        recv_conn_id,
                        &recv_connection,
                        &recv_generation,
                        &recv_client_label,
                        &mut last_ordinary_request,
                        &ws_auth,
                        &recv_supervisor,
                        &recv_deps,
                        &recv_calib,
                        &ack_tx,
                        &worker_tasks,
                    )
                    .await
                    {
                        if let Some(reply) = opt_reply {
                            if !worker_writer.send_ordinary_control(reply).await {
                                break;
                            }
                        }
                        continue;
                    }
                }
                if let Some(reply) = executor_routing_reject(&value, owner_mode) {
                    if !worker_writer.send_ordinary_control(reply).await { break; }
                }
                continue;
            }
            // Pre-lock lease-free commands (depth measurement, teach episode/recorder store).
            // Request-ID ordering and `entry.auth` are enforced uniformly by the dispatcher before
            // invoking the bound handler.
            if let Some((auth_req, DispatchHandler::LeaseFree(handler))) =
                entry.map(|e| (e.auth, e.handler))
            {
                let request_id = value.get("request_id").and_then(Value::as_u64);
                if request_id.is_none()
                    || last_ordinary_request.is_some_and(|last| request_id.unwrap() <= last)
                {
                    let reply = json!({"type":"command_reject", "request_id":value.get("request_id"), "status":400, "code":"INVALID_COMMAND", "message":"request_id must be present and strictly increase", "error":"INVALID_COMMAND"});
                    if !worker_writer.send_ordinary_control(reply).await {
                        break;
                    }
                    continue;
                }
                last_ordinary_request = request_id;
                let path = value.get("path").and_then(Value::as_str).unwrap_or("");
                if let Some(error) = auth_req.check_lease_free(&ws_auth, path, &value["body"]) {
                    let reject_epoch = recv_supervisor
                        .fresh_snapshot(None)
                        .await
                        .map_or_else(|_| recv_supervisor.estop_epoch(), |s| s.obs.epoch_id);
                    let mut reply = rejection(error, "command_reject", reject_epoch);
                    reply["request_id"] = value["request_id"].clone();
                    if !worker_writer.send_ordinary_control(reply).await {
                        break;
                    }
                    continue;
                }
                handler(LeaseFreeCtx {
                    value,
                    recv_supervisor: &recv_supervisor,
                    recv_deps: &recv_deps,
                    recv_connection: &recv_connection,
                    recv_client_label: &recv_client_label,
                    ack_tx: &ack_tx,
                    worker_tasks: &worker_tasks,
                });
                continue;
            }
            let mut response: Option<Value>;
            let mut response_sent = false;
            let evicted_notify: Option<(String, u64)> = None;
            let mut action_pending = actions::ActionPending {
                plan: None,
                macro_action: None,
                replay: None,
                #[cfg(feature = "policy-runtime")]
                policy: None,
                dry_run_grasp: None,
            };
            let mut pending_motion: Option<motion::PendingMotion> = None;
            let mut pending_config_write: Option<config::PendingConfigJob> = None;
            let mut pending_calibration: Option<(Value, CalibJob, u64, u64, bool)> = None;
            maybe_pause_worker_before_lock(&recv_connection).await;
            let mut reply_gate_guard;
            let lock1_reply: Option<dispatch::OwnerReply> = {
                let mut inner = recv_supervisor.inner().write().await;
                if inner.detached {
                    drop(inner);
                    if let Some(reply) = executor_routing_reject(&value, recv_supervisor.owner_mode()) {
                        if !worker_writer.send_ordinary_control(reply).await { break; }
                    }
                    continue;
                }
                reply_gate_guard = Some(CommandReplyGateGuard::enter(
                    &worker_command_reply_pending,
                    recv_supervisor.hub(),
                ));
                macro_rules! call_owner {
                    ($handler:expr, $path:expr, $epoch:expr, $gen:expr) => {{
                        let mut ctx = OwnerDispatchCtx {
                            inner: &mut inner,
                            stamp,
                            received_at,
                            path: $path,
                            value: &value,
                            epoch: $epoch,
                            local_generation: $gen,
                            ws_auth: &ws_auth,
                            recv_supervisor: &recv_supervisor,
                            recv_deps: &recv_deps,
                            recv_calib: &recv_calib,
                            last_sequence: &mut last_sequence,
                            action_pending: &mut action_pending,
                            pending_motion: &mut pending_motion,
                            pending_config_write: &mut pending_config_write,
                            pending_calibration: &mut pending_calibration,
                        };
                        ($handler)(&mut ctx)
                    }};
                }
                let (local_generation, authorized) = lease::check_lease_authority(
                    &mut inner,
                    raw_kind,
                    &value,
                    &recv_connection,
                    &recv_generation,
                    &recv_client_label,
                );
                let is_dry_run = entry.is_some_and(|e| e.lease == LeaseRequirement::RequiredExceptDryRun)
                    && value.get("body").and_then(|b| b.get("dry_run")).and_then(Value::as_bool) == Some(true);
                let missing_gen = (entry.is_some_and(|e| e.lease == LeaseRequirement::RequiredExplicitGeneration)
                    && value.get("generation").and_then(Value::as_u64).is_none())
                    || servo::is_missing_generation(raw_kind, &value);
                if missing_gen {
                    Some(dispatch::OwnerReply::ServoRejectReason("missing_generation"))
                } else if !authorized && !is_dry_run {
                    let reason = if raw_kind != "command" && raw_kind != "release_lease" && local_generation != 0 {
                        Some(inner.lease_loss_reason(&recv_connection, local_generation))
                    } else {
                        None
                    };
                    Some(dispatch::OwnerReply::LeaseLost {
                        epoch: inner.safety.epoch_id(),
                        local_generation,
                        reason,
                    })
                } else {
                    inner.consume_pending_estop();
                    let epoch = inner.safety.epoch_id();
                    if !is_dry_run && value.get("epoch_id").and_then(Value::as_u64) != Some(epoch) {
                        Some(dispatch::OwnerReply::EpochMismatch {
                            epoch,
                            safety_state: inner.safety.state().clone(),
                        })
                    } else if frame_kind == FrameKind::Command {
                        let request_id = value.get("request_id").and_then(Value::as_u64);
                        if request_id.is_none() || last_ordinary_request.is_some_and(|last| request_id.unwrap() <= last) {
                            Some(dispatch::OwnerReply::InvalidRequestId)
                        } else {
                            last_ordinary_request = request_id;
                            let path = value.get("path").and_then(Value::as_str).unwrap_or("");
                            let auth_req = entry.map_or(AuthRequirement::None, |e| e.auth);
                            if let Some(auth_err) = auth_req.check_owner(&ws_auth, &mut inner, path, &value["body"], epoch) {
                                Some(dispatch::OwnerReply::CommandReject {
                                    epoch: inner.safety.epoch_id(),
                                    error: auth_err,
                                })
                            } else if let Some(code) = calibration_busy_code(&inner, path) {
                                Some(dispatch::OwnerReply::CommandReject {
                                    epoch: inner.safety.epoch_id(),
                                    error: ApiError(
                                        StatusCode::CONFLICT,
                                        "a calibration apply, restore or motor zero is running".into(),
                                        code.into(),
                                    ),
                                })
                            } else if let Some(DispatchHandler::Owner(handler)) = entry.map(|e| e.handler) {
                                call_owner!(handler, path, epoch, local_generation)
                            } else if path.starts_with("/api/v1/action/") {
                                let err = if !value["body"].is_object() {
                                    invalid_command("Command body must be an object")
                                } else {
                                    invalid_command("Unknown action")
                                };
                                Some(dispatch::OwnerReply::CommandReject {
                                    epoch: inner.safety.epoch_id(),
                                    error: err,
                                })
                            } else {
                                let error = match validate_command_body_common(&value["body"]) {
                                    Ok(()) => invalid_command("Unknown actuator command"),
                                    Err(msg) => invalid_command(msg),
                                };
                                Some(dispatch::OwnerReply::CommandReject {
                                    epoch: inner.safety.epoch_id(),
                                    error,
                                })
                            }
                        }
                    } else if let Some(DispatchHandler::Owner(handler)) = entry.map(|e| e.handler) {
                        call_owner!(handler, "", epoch, local_generation)
                    } else {
                        None
                    }
                }
            };
            // Write guard is now released; format every immediate WS reply strictly off-lock and
            // enqueue it onto `ordinary_tx` while `reply_gate_guard` stays active so any terminal
            // caused by this command cannot overtake its ACK on the wire.
            response = lock1_reply.map(|r| {
                r.into_value(
                    raw_kind,
                    frame_kind,
                    &value,
                    #[cfg(feature = "policy-runtime")]
                    &pending_policy_loads,
                )
            });
            if evicted_notify.is_none()
                && response
                    .as_ref()
                    .is_some_and(|r| r.get("type").and_then(Value::as_str) != Some("lease_granted"))
                && !worker_writer.is_fatal()
            {
                if let Some(reply) = response.take() {
                    match worker_writer.ordinary_tx.try_send(reply) {
                        Ok(()) => {
                            response_sent = true;
                        }
                        Err(mpsc::error::TrySendError::Full(reply)) => {
                            response = Some(reply);
                        }
                        Err(mpsc::error::TrySendError::Closed(_)) => {
                            drop(reply_gate_guard.take());
                            break;
                        }
                    }
                }
            }
            if response.is_none() {
                drop(reply_gate_guard.take());
            }

            let allows_motion_deferred = entry.is_some_and(|e| matches!(e.reply, ReplyPolicy::CommandDeferred | ReplyPolicy::PresetReply));
            if allows_motion_deferred {
                if let Some(pending) = pending_motion.take() {
                    let Some((next, gate)) = motion::complete_pending_motion(
                        pending,
                        stamp,
                        &recv_supervisor,
                        &recv_connection,
                        &recv_generation,
                        &worker_closing,
                        &worker_command_reply_pending,
                        &worker_tasks,
                    ).await else { break; };
                    reply_gate_guard = gate;
                    response = next;
                }
            }

            let allows_deferred = entry.is_some_and(|e| e.reply == ReplyPolicy::CommandDeferred);
            if allows_deferred {
                if let Some((plan, generation)) = action_pending.plan.take() {
                    let Some((next, gate)) = actions::complete_pending_plan(
                        plan,
                        generation,
                        stamp,
                        &recv_supervisor,
                        &recv_connection,
                        &recv_generation,
                        &worker_closing,
                        &worker_command_reply_pending,
                    ).await else { break; };
                    reply_gate_guard = gate;
                    response = next;
                }

                if let Some((request_id, pending, generation)) = action_pending.macro_action.take() {
                    macro_check = Some(actions::spawn_macro_check(
                        request_id,
                        pending,
                        generation,
                        stamp,
                        &recv_deps,
                        &worker_closing,
                        &recv_connection,
                        &worker_tasks,
                    ));
                }

                if let Some((request_id, input, generation)) = action_pending.replay.take() {
                    let Some((next, gate)) = actions::complete_pending_replay(
                        request_id,
                        input,
                        generation,
                        stamp,
                        &recv_supervisor,
                        &recv_connection,
                        &recv_generation,
                        &worker_closing,
                        &worker_command_reply_pending,
                    ).await else { break; };
                    reply_gate_guard = gate;
                    response = next;
                }
                #[cfg(feature = "policy-runtime")]
                if let Some((request_id, pending, generation, loading_guard)) = action_pending.policy.take() {
                    policy::spawn_policy_start(
                        request_id,
                        pending,
                        generation,
                        loading_guard,
                        recv_conn_id,
                        &recv_supervisor,
                        &recv_sandbox,
                        &recv_deps,
                        &recv_connection,
                        &recv_generation,
                        &ack_tx,
                        &worker_ws_closed,
                        &pending_policy_loads,
                        &ws_auth,
                        &worker_tasks,
                    );
                }
                if let Some((request_id, body)) = action_pending.dry_run_grasp.take() {
                    response = Some(actions::complete_dry_run_grasp(request_id, body, &recv_supervisor, &recv_deps).await);
                }
                // Calibration apply / restore: its own tracked task (HTTP to rebot-vision, file writes);
                // this loop goes on (heartbeats, stop) and the reply comes back through ack_tx.
                if let Some((request_id, job, generation, epoch, busy)) = pending_calibration.take() {
                    calibration::spawn_calibration_job(
                        request_id,
                        job,
                        generation,
                        epoch,
                        busy,
                        &recv_supervisor,
                        &recv_calib,
                        &recv_connection,
                        &recv_generation,
                        &ack_tx,
                        &ws_auth,
                        &worker_tasks,
                    );
                }
                // Atomic two-phase persist-before-commit config writes (PR C / A8, off-lock prepare + requests::config).
                if let Some(job) = pending_config_write.take() {
                    response = config::complete_or_spawn_config_write(
                        job,
                        recv_conn_id,
                        &recv_supervisor,
                        &recv_deps,
                        &ack_tx,
                        &ws_auth,
                        &recv_connection,
                        &recv_generation,
                        &worker_tasks,
                    )
                    .await;
                }
            } else {
                #[cfg(feature = "policy-runtime")]
                let has_policy = action_pending.policy.is_some();
                #[cfg(not(feature = "policy-runtime"))]
                let has_policy = false;
                let unexpected_deferred = action_pending.plan.is_some()
                    || action_pending.macro_action.is_some()
                    || action_pending.replay.is_some()
                    || has_policy
                    || action_pending.dry_run_grasp.is_some()
                    || pending_motion.is_some()
                    || pending_calibration.is_some()
                    || pending_config_write.is_some();
                debug_assert!(
                    !unexpected_deferred,
                    "non-CommandDeferred entry populated a deferred async slot"
                );
                if unexpected_deferred
                    || (entry.is_some_and(|e| e.reply == ReplyPolicy::CommandImmediate)
                        && response.is_none()
                        && !response_sent)
                {
                    tracing::error!(
                        "WS dispatch contract violation: immediate entry {:?} produced no immediate reply or populated deferred slot",
                        entry.map(|e| e.key)
                    );
                    response = Some(json!({
                        "type": "command_reject",
                        "request_id": value.get("request_id"),
                        "status": 500,
                        "code": "INTERNAL_ERROR",
                        "error": "INTERNAL_ERROR",
                        "message": "Command handler violated immediate reply policy"
                    }));
                }
            }
            // Control replies are never silently dropped: saturation closes the connection with an error.
            if let Some(response) = response {
                if response.get("type").and_then(Value::as_str) == Some("lease_granted") {
                    let delay_ms = TEST_LEASE_GRANTED_DELAY_MS.load(Ordering::Relaxed);
                    if delay_ms > 0 {
                        tokio::time::sleep(Duration::from_millis(delay_ms)).await;
                    }
                } else if reply_gate_guard.is_some()
                    && response.get("type").and_then(Value::as_str) == Some("command_ack")
                {
                    let delay_ms = TEST_DEFERRED_ACK_DELAY_MS.load(Ordering::Relaxed);
                    if delay_ms > 0 {
                        tokio::time::sleep(Duration::from_millis(delay_ms)).await;
                    }
                }
                if !worker_writer.send_ordinary_control(response).await {
                    drop(reply_gate_guard.take());
                    break;
                }
            }
            drop(reply_gate_guard.take());
        }
    });
    // Only the writer (fatal close after urgent flush, or socket error) and reader (client
    // EOF / Close frame) drive connection teardown. A helper task (`worker_task`,
    // `heartbeat_task`, `auth_task`) exiting on reply-queue overflow signals `trigger_fatal`
    // and NEVER aborts `reader_task` or triggers early teardown.
    enum FirstExit { Writer, Reader }
    let first_exit = tokio::select! {
        _ = &mut writer_task => FirstExit::Writer,
        _ = &mut reader_task => FirstExit::Reader,
    };
    // M1: The VERY FIRST action on socket close is a non-blocking owner-side fence of this
    // connection's lease generation BEFORE any task abort, join, or cleanup, so zero bus output
    // from that generation can escape while teardown runs.
    let closing_gen = connection_generation.swap(0, Ordering::SeqCst);
    supervisor.fence_ws_connection_immediate(conn_handle, closing_gen);

    connection_closing.store(true, Ordering::SeqCst);
    #[cfg(feature = "policy-runtime")]
    ws_closed.store(true, Ordering::SeqCst);
    ordinary_queue.close();
    heartbeat_slot.close();
    reg_task.abort();
    let _ = (&mut reg_task).await;

    // Stop and JOIN all primary tasks so none outlives teardown; give writer <= 1 s grace on fatal overflow.
    worker_task.abort();
    let _ = worker_task.await;
    if let Some(handle) = supervisor.executor_handle() {
        let _ = handle.try_unregister_servo_connection(conn_id);
    }
    heartbeat_task.abort();
    let _ = heartbeat_task.await;
    auth_task.abort();
    let _ = auth_task.await;
    let fatal_close = fatal_triggered.load(Ordering::Acquire);
    if !fatal_close && !matches!(first_exit, FirstExit::Reader) {
        reader_task.abort();
        let _ = (&mut reader_task).await;
    }
    if !matches!(first_exit, FirstExit::Writer) {
        if fatal_close {
            let grace = ws_write_timeout(&connection).min(WS_FATAL_FLUSH_TIMEOUT)
                + Duration::from_millis(250);
            if tokio::time::timeout(grace, &mut writer_task).await.is_err() {
                writer_task.abort();
                let _ = writer_task.await;
            }
        } else {
            writer_task.abort();
            let _ = writer_task.await;
        }
    }
    if fatal_close && !matches!(first_exit, FirstExit::Reader) {
        // Keep draining unread inbound bytes briefly after sending Close+FIN so TCP close does
        // not emit RST over flooded client buffers and wipe the flushed urgent/fatal frames.
        if tokio::time::timeout(Duration::from_millis(50), &mut reader_task).await.is_err() {
            reader_task.abort();
            let _ = reader_task.await;
        }
    }
    // Wait up to bounded grace for tracked child tasks (urgent E-stop/Stop, macro preflight, etc.),
    // then abort any remaining and join EVERY handle before releasing the owner lease.
    tasks.shutdown_and_join(WS_TEARDOWN_JOIN_TIMEOUT).await;

    if supervisor.owner_mode() == crate::supervisor::OWNER_LEGACY {
        supervisor.recorder_connection_closed(&connection).await;
    }
    if !ws_slot_guard.disarmed {
        supervisor.finish_ws_connection(conn_handle).await;
        ws_slot_guard.disarmed = true;
    }
    connection_generation.store(0, Ordering::SeqCst);
}
