use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, LazyLock, Mutex};
use std::time::Duration;
use axum::{extract::{ws::{CloseFrame, Message, WebSocket, WebSocketUpgrade}, State}, response::IntoResponse};
use futures_util::{SinkExt, StreamExt};
use serde_json::{json, Value};
use tokio::sync::mpsc;
use crate::supervisor::{ArmError, ArmSupervisor, SupervisorInner};
use crate::motion::{self, PlanInput};
use crate::safety::SafetyError;
use crate::api::rest::ApiError;
use axum::http::StatusCode;

// ws leaf modules (armd 1.6 RT-core foundation)
mod lease;
#[allow(unused_imports)]
use lease::*;
mod motion_cmds;
#[allow(unused_imports)]
use motion_cmds::*;
mod policy;
#[allow(unused_imports)]
use policy::*;
mod calibration;
#[allow(unused_imports)]
use calibration::*;
mod config;
#[allow(unused_imports)]
use config::*;

pub type ConnectionRegistry = Arc<Mutex<HashMap<String, mpsc::Sender<Message>>>>;

static CONNECTION_REGISTRY: LazyLock<ConnectionRegistry> =
    LazyLock::new(|| Arc::new(Mutex::new(HashMap::new())));

type FatalCloserEntry = (mpsc::Sender<FatalClose>, Arc<AtomicBool>, Arc<tokio::sync::Notify>);

static CONNECTION_CLOSERS: LazyLock<Mutex<HashMap<String, FatalCloserEntry>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));

pub fn connection_registry() -> ConnectionRegistry {
    CONNECTION_REGISTRY.clone()
}

static NEXT_CONNECTION: AtomicU64 = AtomicU64::new(1);

/// Test hook: pause (ms) between stamping a delayed command ack with the current epoch and
/// enqueueing it. 0 (no pause) outside tests.
pub static TEST_ACK_STAMP_DELAY_MS: AtomicU64 = AtomicU64::new(0);

/// Test hook: pause (ms) after `acquire_lease` releases the owner lock and before enqueueing
/// `lease_granted` onto `ordinary_out_tx`, allowing heartbeat-triggered `lease_lost` to race ahead.
pub static TEST_LEASE_GRANTED_DELAY_MS: AtomicU64 = AtomicU64::new(0);

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
    let control_hz = supervisor.control_frequency_hz().max(1);
    let obs_rate = match query.get("obs_hz") {
        None => None,
        Some(v) => match v.trim().parse::<u64>() {
            Ok(n) => Some(ObsRate { hz: n.clamp(1, u64::from(control_hz)) as f64, control_hz: f64::from(control_hz) }),
            Err(_) => return (StatusCode::BAD_REQUEST, "obs_hz must be an integer >= 0 (0 and 1 mean 1 Hz; values above control_frequency_hz are clamped)").into_response(),
        },
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

enum ActionStart {
    Started(u64, f64),
    Plan(PendingPlan),
    Macro(PendingMacro),
    Replay(crate::replay::ReplayInput),
    DryRunGrasp(Value),
    Sequence(Box<crate::sequences::RunPlan>),
    CalibHover(Box<crate::calib_hover::HoverPlan>),
    /// `gripper_calibrate` (epoch, tag): begun with the supervisor handle at the call site.
    GripperCalibrate(u64, Option<String>),
    MeasureMultiview(Box<crate::measure_multiview::Plan>),
    #[cfg(feature = "policy-runtime")]
    Policy(Box<PendingPolicyStart>),
    #[cfg(feature = "policy-runtime")]
    PolicyStopped(bool),
}

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
fn runs_during_macro_check(kind: &str) -> bool {
    matches!(kind, "acquire_lease" | "release_lease")
}

/// The reply to a message cancelled by a stop / e-stop before it ran (`code` STOPPED /
/// ESTOP_ACTIVE). None for messages that are not acknowledged.
fn cancelled_reply(value: &Value, code: &str, epoch: Option<u64>) -> Option<Value> {
    let kind = value.get("type").and_then(Value::as_str).unwrap_or("");
    let mut reply = json!({"status":409, "code":code, "error":code, "message":"Cancelled by a stop before it began"});
    if let Some(ep) = epoch {
        reply["epoch_id"] = json!(ep);
    }
    if kind == "command" {
        reply["type"] = json!("command_reject");
        reply["request_id"] = value.get("request_id").cloned().unwrap_or(Value::Null);
    } else if kind == "preset" {
        reply["type"] = json!("preset_reject");
        reply["name"] = value.get("name").cloned().unwrap_or(Value::Null);
    } else if kind == "servo" || value.get("positions").is_some() || value.get("mode").and_then(Value::as_str) == Some("cartesian_joint_stream") {
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

fn ws_status_reject(kind: &str, value: &Value, status: u16, code: &str, msg: &str, servo_reason: &str) -> Option<Value> {
    if kind == "acquire_lease" {
        let mut r = json!({"type":"lease_rejected", "status":status, "code":code, "error":code, "message":msg});
        if let Some(s) = value.get("seq").and_then(Value::as_u64) { r["seq"] = json!(s); }
        Some(r)
    } else if kind == "release_lease" {
        Some(json!({"type":"lease_released", "success":false, "status":status, "code":code, "error":code, "message":msg}))
    } else if kind == "command" {
        Some(json!({"type":"command_reject", "request_id":value.get("request_id"), "status":status, "code":code, "error":code, "message":msg}))
    } else if kind == "preset" {
        Some(json!({"type":"preset_reject", "name":value.get("name"), "status":status, "code":code, "error":code, "message":msg}))
    } else if kind == "servo" || value.get("mode").and_then(Value::as_str) == Some("cartesian_joint_stream") || value.get("positions").is_some() {
        Some(json!({"type":"servo_reject", "reason":servo_reason, "status":status, "code":code, "error":code, "message":msg}))
    } else {
        None
    }
}

fn executor_routing_reject(kind: &str, value: &Value, owner_mode: u8) -> Option<Value> {
    let (code, msg) = if owner_mode == crate::supervisor::OWNER_EXECUTOR {
        ("EXECUTOR_ROUTING_PENDING", "WebSocket message is not yet routed through the RT executor")
    } else {
        ("EXECUTOR_UNAVAILABLE", "RT executor ownership is transitioning or unavailable")
    };
    ws_status_reject(kind, value, 503, code, msg, "executor_routing_pending")
}

fn backlog_full_reject(kind: &str, value: &Value) -> Option<Value> {
    ws_status_reject(kind, value, 503, "BACKLOG_FULL", "WebSocket ordinary message backlog is full", "backlog_full")
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

    fn try_push(&self, value: Value, stamp: crate::supervisor::AdmissionStamp) -> Result<(), Value> {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        if g.closed || g.items.len() >= ORDINARY_IN_CAPACITY {
            return Err(value);
        }
        g.items.push_back(QueuedOrdinary { value, stamp });
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
                    g.items.iter().position(|item| {
                        let kind = item.value.get("type").and_then(Value::as_str).unwrap_or("");
                        runs_during_macro_check(kind)
                    })
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
    /// Builds JSON cancellation payloads outside the sync lock so the reader critical section stays O(1).
    fn drain_cancellable(&self, code: &'static str, epoch: Option<u64>) -> Vec<Value> {
        let mut to_cancel = Vec::new();
        {
            let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
            let n = g.items.len();
            for _ in 0..n {
                if let Some(item) = g.items.pop_front() {
                    let kind = item.value.get("type").and_then(Value::as_str).unwrap_or("");
                    let exempt_disable = code == "STOPPED" && is_torque_disable_command(&item.value);
                    let cancellable = !exempt_disable
                        && (matches!(kind, "command" | "preset" | "servo")
                            || item.value.get("positions").is_some()
                            || item.value.get("mode").and_then(Value::as_str) == Some("cartesian_joint_stream"));
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

// Under the write lock, after lease / epoch / request_id checks.
#[allow(clippy::too_many_arguments)]
fn prepare_action(
    inner: &mut SupervisorInner,
    stamp: crate::supervisor::AdmissionStamp,
    path: &str,
    body: &Value,
    epoch: u64,
    request_id: &Value,
    auth: &crate::auth::OperatorAuth,
    ws_token: Option<&str>,
) -> Result<ActionStart, ApiError> {
    if !body.is_object() { return Err(invalid_command("Command body must be an object")); }
    let (kind, parsed) = match path {
        "/api/v1/action/move_to" => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            ("move_to", motion::parse_move_to(body))
        }
        "/api/v1/action/move_relative" => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            ("move_relative", motion::parse_move_relative(body))
        }
        "/api/v1/action/gripper" => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            let mut request = motion::parse_gripper(body).map_err(ArmError::from)?;
            if let Some(scale) = motion::num(body, "speed_scale").map_err(ArmError::from)? {
                if !(0.05..=1.0).contains(&scale) {
                    return Err(ArmError::from(motion::MotionError::InvalidCommand(
                        "speed_scale must be in [0.05, 1.0]".into(),
                    ))
                    .into());
                }
                if inner.config.active_tool.commissioning_required && scale > 0.2 + 1e-6 {
                    return Err(ArmError::Safety(SafetyError::CommissioningRequired(format!(
                        "gripper (speed_scale {scale:.2} > 0.2)"
                    )))
                    .into());
                }
                if body.get("speed").is_none_or(Value::is_null) {
                    request.speed = (2.0 * scale).clamp(0.01, 2.0);
                }
            }
            if body.get("baseline").is_some_and(|v| !v.is_null()) {
                return Err(invalid_command("baseline is not a gripper action field; calibrate with /api/v1/action/gripper_calibrate"));
            }
            let admitted = inner.check_admission(stamp).map_err(ArmError::from)?;
            let (id, duration) = SupervisorInner::start_gripper(admitted, request, epoch)?;
            return Ok(ActionStart::Started(id, duration));
        }
        "/api/v1/action/gripper_zero_check" => {
            let admitted = inner.check_admission(stamp).map_err(ArmError::from)?;
            let (id, duration) = SupervisorInner::start_gripper_zero_check_with_auth(
                admitted,
                body,
                epoch,
                Some((auth.clone(), ws_token.map(str::to_string))),
            )?;
            return Ok(ActionStart::Started(id, duration));
        }
        "/api/v1/action/grasp" => {
            if body.get("dry_run").and_then(Value::as_bool) == Some(true) {
                return Ok(ActionStart::DryRunGrasp(body.clone()));
            }
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            return Ok(ActionStart::Macro(PendingMacro::Grasp(crate::grasp::prepare(inner, body, epoch)?)));
        }
        "/api/v1/action/place" => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            return Ok(ActionStart::Macro(PendingMacro::Place(crate::place::prepare(inner, body, epoch)?)));
        }
        "/api/v1/action/replay" => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            let request = crate::replay::parse_replay(body).map_err(ArmError::from)?;
            return Ok(ActionStart::Replay(inner.replay_input(request, epoch)?));
        }
        "/api/v1/action/move_to_pose" => {
            let admitted = inner.check_admission(stamp).map_err(ArmError::from)?;
            let (id, duration) = crate::poses::start_action(admitted, body, epoch)?;
            return Ok(ActionStart::Started(id, duration));
        }
        "/api/v1/action/run_sequence" => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            return Ok(ActionStart::Sequence(Box::new(crate::sequences::prepare_run(inner, body, epoch)?)));
        }
        crate::calib_hover::PATH => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            return Ok(ActionStart::CalibHover(Box::new(crate::calib_hover::prepare(inner, body, epoch)?)));
        }
        crate::gripper_calib::CALIBRATE_PATH => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            let tag = motion::tag(body).map_err(ArmError::from)?;
            return Ok(ActionStart::GripperCalibrate(epoch, tag));
        }
        crate::measure_multiview::PATH => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            return Ok(ActionStart::MeasureMultiview(Box::new(crate::measure_multiview::prepare(inner, body, epoch)?)));
        }
        #[cfg(feature = "policy-runtime")]
        "/api/v1/action/policy/start" => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            let request = crate::policy_runtime::parse_policy_start_request(body).map_err(ArmError::from)?;
            let _ = crate::policy_runtime::compute_policy_deadline(request.max_seconds).map_err(ArmError::from)?;
            inner.check_policy_start_preconditions_with(
                request.mode,
                request.sim_only,
                request.i_am_at_the_arm,
                epoch,
            )?;
            return Ok(ActionStart::Policy(Box::new(PendingPolicyStart {
                request,
                config: inner.config.clone(),
                epoch,
                stamp,
                policy_live_allowed: inner.policy_live_allowed(),
            })));
        }
        #[cfg(feature = "policy-runtime")]
        "/api/v1/action/policy/stop" => {
            inner.advance_stop_generation();
            let stopped = inner.stop_policy_run(None, "STOPPED");
            return Ok(ActionStart::PolicyStopped(stopped));
        }
        _ => return Err(invalid_command("Unknown action")),
    };
    let (mut spec, mut speed, tag) = parsed.map_err(ArmError::from)?;
    let _ = &mut spec;
    // move_to / move_relative keep their straight line to the goal; `precise: true` (profile
    // `applies_to` move_to_precise) or `approach: "same_side"` end it with the short segment
    let precise = motion::flag(body, "precise", false).map_err(ArmError::from)?;
    let request = crate::backlash::parse_request(body).map_err(ArmError::from)?;
    let path_mode = motion::parse_path_mode(body).map_err(ArmError::from)?;
    let same_side = precise || request != crate::backlash::Request::Profile;
    if path_mode == motion::PathMode::Joint && body.get("speed").is_some_and(|v| !v.is_null()) {
        return Err(invalid_command("speed applies to path \"linear\"; path \"joint\" takes speed_scale (0.1..1.0)"));
    }
    let linear_speed_scale = if path_mode == motion::PathMode::Linear {
        if let Some(s) = motion::num(body, "speed_scale").map_err(ArmError::from)? {
            if !(0.05..=1.0).contains(&s) {
                return Err(invalid_command("speed_scale must be in [0.05, 1.0]"));
            }
            if body.get("speed").is_none_or(Value::is_null) {
                speed.linear = (motion::MAX_LINEAR_MPS * s).min(motion::MAX_LINEAR_MPS);
                speed.angular = (motion::MAX_ANGULAR_RPS * s).min(motion::MAX_ANGULAR_RPS);
            }
            Some(s)
        } else {
            None
        }
    } else {
        None
    };
    let approach = if same_side && path_mode == motion::PathMode::Linear { inner.approach_plan("move_to_precise", request) } else { Err(None) };
    let input = inner.plan_input_tcp()?;
    let (goal, opts) = motion::resolve_goal_opts(&input, &spec);
    let joint = match path_mode {
        motion::PathMode::Linear => {
            if inner.config.active_tool.commissioning_required {
                let scale_ok = linear_speed_scale.is_none_or(|s| s <= 0.2 + 1e-6);
                let speed_ok = speed.linear <= 0.2 * motion::MAX_LINEAR_MPS + 1e-6
                    && speed.angular <= 0.2 * motion::MAX_ANGULAR_RPS + 1e-6;
                if !scale_ok || !speed_ok {
                    return Err(ArmError::Safety(SafetyError::CommissioningRequired(format!(
                        "{kind} (requires speed_scale <= 0.2 or speed <= {:.3} m/s / {:.2} rad/s)",
                        0.2 * motion::MAX_LINEAR_MPS,
                        0.2 * motion::MAX_ANGULAR_RPS
                    )))
                    .into());
                }
            }
            None
        }
        motion::PathMode::Joint => {
            let speed_scale = crate::poses::speed_scale(body)?;
            if inner.config.active_tool.commissioning_required && speed_scale > 0.2 + 1e-6 {
                return Err(ArmError::Safety(SafetyError::CommissioningRequired(format!(
                    "{kind} (speed_scale {speed_scale:.2} > 0.2)"
                )))
                .into());
            }
            Some(JointGoal {
                speed_scale,
                same_side: same_side.then_some(request),
                snapshot: inner.joint_goal_snapshot(speed_scale),
            })
        }
    };
    let seeds = motion::goal_seeds(&input, &goal, &inner.goal_seed_poses());
    Ok(ActionStart::Plan(PendingPlan { request_id: request_id.clone(), kind, input, goal, speed, tag, opts, approach, joint, seeds }))
}

fn command_ack(inner: &SupervisorInner, request_id: &Value, started: Started) -> Value {
    let control_mode = if inner.gravity_comp_active() { "gravity_comp" } else { "position" };
    let mut ack = json!({"type":"command_ack", "request_id":request_id, "status":"accepted", "epoch_id":inner.safety.epoch_id(), "safety_state":inner.safety.state(), "is_torqued":inner.is_torqued, "control_mode":control_mode});
    if let Some((id, duration)) = started {
        ack["action_id"] = json!(id);
        ack["expected_duration_s"] = json!(duration);
    }
    ack
}

fn pose_ack_fields(inner: &SupervisorInner, pose: &crate::poses::NamedPose, revision: u64) -> Vec<(&'static str, Value)> {
    let default_joint_names = inner
        .poses
        .list
        .joint_names
        .clone()
        .unwrap_or_else(|| inner.joint_names());
    let pose_val = crate::poses::pose_response_value(pose, &default_joint_names);
    let pose_id = pose_val.get("pose_id").cloned().unwrap_or_else(|| json!(pose.id));
    let content_sha256 = pose_val.get("content_sha256").cloned().unwrap_or(Value::Null);
    let joint_names = pose_val.get("joint_names").cloned().unwrap_or_else(|| json!(default_joint_names));
    let stale = pose_val.get("stale").cloned().unwrap_or(Value::Null);
    vec![
        ("pose", pose_val),
        ("pose_id", pose_id),
        ("content_sha256", content_sha256),
        ("joint_names", joint_names),
        ("stale", stale),
        ("revision", json!(revision)),
    ]
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

/// Check privileged-operations authorization under the supervisor write lock.
/// Returns `Some(ApiError)` if rejected, `None` if allowed.
fn check_operator_privilege(
    ws_auth: &WsAuth,
    inner: &mut SupervisorInner,
    path: &str,
    body: &Value,
    epoch: u64,
) -> Option<ApiError> {
    if path == "/api/v1/safety/clear_fault"
        && inner.consume_pending_estop()
        && inner.safety.epoch_id() != epoch
    {
        return Some(ArmError::Safety(SafetyError::EpochMismatch {
            expected: inner.safety.epoch_id(),
            received: epoch,
        }).into());
    }
    if ws_auth.is_operator() {
        return None;
    }
    match path {
        "/api/v1/workspace/keepout"
        | "/api/v1/safety/collision"
        | "/api/v1/mode/gravity_comp"
        | "/api/v1/gripper/calibration"
        | "/api/v1/calibration/apply"
        | "/api/v1/calibration/restore"
        | "/api/v1/calibration/motor_zero"
        | "/api/v1/calibration/joint_zero/restore"
        | crate::mit::TIMEOUT_PATH
        | crate::mit::QUALIFICATION_PATH
        | "/api/v1/poses/save"
        | "/api/v1/poses/rename"
        | "/api/v1/poses/delete"
        | "/api/v1/sequences/save"
        | "/api/v1/sequences/delete"
        | "/api/v1/grasp/setup"
        | "/api/v1/tool/commissioning/confirm" => Some(ApiError::operator_required(path)),
        // moves the arm and triggers the camera (like /api/v1/detection/trigger)
        crate::measure_multiview::PATH => Some(ApiError::operator_required(path)),
        // a sequence run that records writes episodes (like /api/v1/dataset/record/*)
        "/api/v1/action/run_sequence"
            if body.get("record").is_some_and(|r| !r.is_null()) =>
        {
            Some(ApiError::operator_required("run_sequence (record)"))
        }
        // learns the free-motion torque of the jaw (stored on the device)
        crate::gripper_calib::CALIBRATE_PATH => Some(ApiError::operator_required("gripper_calibrate")),
        "/api/v1/action/gripper_zero_check"
            if body.get("store_offset").and_then(Value::as_bool) == Some(true) =>
        {
            Some(ApiError::operator_required("gripper_zero_check (store_offset=true)"))
        }
        // a policy run on a real (or unknown) driver, in any mode; mock / sim drivers stay open.
        // policy/stop needs no session, like /api/v1/motion/stop. Rechecked after loading.
        #[cfg(feature = "policy-runtime")]
        "/api/v1/action/policy/start" if !inner.policy_live_allowed() => {
            Some(ApiError::operator_required("policy/start (real arm)"))
        }
        "/api/v1/safety/clear_fault" => {
            if inner.consume_pending_estop() && inner.safety.epoch_id() != epoch {
                return Some(ArmError::Safety(SafetyError::EpochMismatch {
                    expected: inner.safety.epoch_id(),
                    received: epoch,
                }).into());
            }
            if inner.e_stop_latched.load(Ordering::SeqCst) {
                return Some(ApiError::operator_required("clear_fault (EStop latched)"));
            }
            if (inner.safety.state().is_stopped() || inner.safety.has_latched_fault())
                && !inner.safety.recoverable()
            {
                return Some(ApiError::operator_required("clear_fault (non-recoverable fault or EStop)"));
            }
            None
        }
        _ => None,
    }
}

/// OperatorAuth for the WS commands handled without a lease (teach v1 episode store and
/// recorder, depth measurement). Every one of them writes the episode store or triggers the
/// camera, so it needs an operator session when auth is enforced, like
/// `/api/v1/dataset/record/*` and `/api/v1/detection/trigger`; the owner can always end a
/// recording without saving it (`record/stop {save: false}`).
fn lease_free_operator_check(ws_auth: &WsAuth, path: &str, body: &Value) -> Option<ApiError> {
    if ws_auth.is_operator() { return None; }
    if path == "/api/v1/record/stop" && body.get("save").and_then(Value::as_bool) == Some(false) { return None; }
    Some(ApiError::operator_required(path))
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

fn command_reject(inner: &SupervisorInner, request_id: &Value, error: ApiError) -> Value {
    let mut reply = rejection(error, "command_reject", inner.safety.epoch_id());
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

    // This identity never comes from a client payload and is never disclosed.
    let conn_id = NEXT_CONNECTION.fetch_add(1, Ordering::Relaxed);
    let connection = format!("connection:{conn_id}");
    let connection_generation = Arc::new(AtomicU64::new(0));
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
    let term_writer = writer.clone();
    let _terminal_sub = supervisor.subscribe_action_terminals(move |obs| {
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
        while let Some(value) = auth_in_rx.recv().await {
            let next_token = value.get("token").and_then(Value::as_str).filter(|t| !t.is_empty()).map(str::to_string);
            let status = auth_ws_auth.update_token(next_token);
            let reply = json!({
                "type": "auth_ack",
                "seq": value.get("seq"),
                "auth_enforced": status.auth_enforced,
                "role": status.role,
                "expires_in_s": status.expires_in_s
            });
            if !auth_writer.send_auth(reply).await {
                break;
            }
        }
    });

    // Dedicated latest-wins heartbeat processor isolated from ordinary backlog and slow worker IO.
    let hb_supervisor = supervisor.clone();
    let hb_connection = connection.clone();
    let hb_generation = connection_generation.clone();
    let hb_client_label = client_label.clone();
    let hb_slot = heartbeat_slot.clone();
    let hb_writer = writer.clone();
    let heartbeat_task = tokio::spawn(async move {
        while let Some(value) = hb_slot.pop().await {
            if hb_supervisor.owner_mode() != crate::supervisor::OWNER_LEGACY {
                continue;
            }
            let reply = {
                let mut inner = hb_supervisor.inner().write().await;
                lease::handle_heartbeat_task_step(
                    &mut inner,
                    &value,
                    &hb_connection,
                    &hb_generation,
                    &hb_client_label,
                )
            };
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
            let kind = value.get("type").and_then(Value::as_str).unwrap_or("");

            let is_estop = kind == "estop" || kind == "e_stop";
            let is_stop = kind == "command"
                && value.get("path").and_then(Value::as_str) == Some("/api/v1/motion/stop");

            // When fatal close is already in progress, keep draining inbound bytes so closing the
            // TCP fd does not emit RST over unread flood traffic; still latch urgent E-stop and
            // coalesce/apply urgent Stop until the writer finishes its bounded close.
            if reader_writer.is_fatal() && !is_estop && !is_stop {
                continue;
            }

            // `auth` is classified in O(1) on the reader and verified off-reader by `auth_task`
            // so `OperatorAuth::sessions` table scans never stall urgent E-stop/Stop intake.
            if kind == "auth" {
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
            if kind == "heartbeat" {
                reader_heartbeat_slot.push_latest(value);
                continue;
            }

            // Urgent E-stop (both aliases): reserve urgent reply capacity BEFORE latching, ALWAYS
            // latch immediately in O(1) via lock-free atomics (no ACK-drain mutex, no waiter scan),
            // and never exit reader because of reply-queue capacity.
            if is_estop {
                reader_writer.mark_estop_admitted();
                let estop_guard = reader_writer.enter_in_flight_estop();
                let reserved = reader_writer.try_reserve_estop();
                let closed = matches!(reserved, Err(mpsc::error::TrySendError::Closed(())));
                let ticket = reader_supervisor.begin_e_stop_checked();
                let req_id = value.get("request_id").cloned();
                let sup = reader_supervisor.clone();
                let cancel_tx = reader_cancel_tx.clone();
                let writer_c = reader_writer.clone();
                reader_tasks.spawn(async move {
                    let _estop_guard = estop_guard;
                    let (reply, cancel_epoch) = match sup.finish_e_stop_checked(ticket).await {
                        Ok((state, epoch)) => (
                            json!({"type":"epoch_sync", "epoch_id":epoch, "safety_state":state}),
                            Some(epoch),
                        ),
                        Err(err) => {
                            let api: ApiError = err.into();
                            let mut r = json!({"type":"estop_reject", "status":api.status.as_u16(), "code":api.code, "error":api.error, "message":api.message, "latched":true});
                            if let Some(id) = req_id { r["request_id"] = id; }
                            (r, sup.hub().latest().map(|s| s.obs.epoch_id))
                        }
                    };
                    match reserved {
                        Ok(permit) => { permit.send(reply); }
                        Err(mpsc::error::TrySendError::Full(())) => {
                            let _ = writer_c.send_estop_blocking(reply).await;
                        }
                        Err(mpsc::error::TrySendError::Closed(())) => {}
                    }
                    let _ = cancel_tx.send(Some(CancelSignal { gen: sup.stop_generation(), code: "ESTOP_ACTIVE", epoch: cancel_epoch }));
                });
                if closed { break; }
                continue;
            }

            // Urgent Stop (`/api/v1/motion/stop`): non-blocking handoff; coalesces into an already
            // pending Stop from this connection with matching lease/epoch; never enters ordinary
            // queue and NEVER exits the reader on reply-queue capacity (only writer closes after
            // its bounded grace, while reader keeps reading and latching any E-stop behind it).
            if is_stop {
                let permit = match reader_writer.try_reserve_stop() {
                    Ok(p) => Some(p),
                    Err(mpsc::error::TrySendError::Full(())) => {
                        reader_writer.trigger_fatal(FatalClose::urgent_queue_full());
                        None
                    }
                    Err(mpsc::error::TrySendError::Closed(())) => break,
                };
                let local_gen = reader_generation.load(Ordering::SeqCst);
                let supplied_gen = value.get("generation").and_then(Value::as_u64).unwrap_or(0);
                let label_ok = {
                    let lbl = reader_client_label.lock().unwrap_or_else(|e| e.into_inner());
                    value.get("client_id").is_none_or(|id| id.as_str() == lbl.as_deref())
                };
                let conn_label = crate::model::HolderLabel::try_new(&reader_connection);
                let effective_gen = if conn_label.is_some() && local_gen != 0 && supplied_gen == local_gen && label_ok { local_gen } else { 0 };
                let raw_request_id = value.get("request_id").cloned().unwrap_or(Value::Null);
                let parsed_request_id = value.get("request_id").and_then(Value::as_u64);
                let epoch = value.get("epoch_id").and_then(Value::as_u64);
                let body_error = validate_command_body_common(&value["body"]).err();

                let dispatch = {
                    let last_req = last_request_get(&reader_last_request);
                    let mut st = reader_in_flight_stop.lock().unwrap_or_else(|e| e.into_inner());
                    let high_water = match (last_req, st.active.as_ref().and_then(|a| a.pending_max_request_id)) {
                        (Some(a), Some(b)) => Some(a.max(b)),
                        (a, b) => a.or(b),
                    };
                    let request_id_valid = parsed_request_id.is_some_and(|id| high_water.is_none_or(|last| id > last));
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
                                active.pending_max_request_id = match (active.pending_max_request_id, parsed_request_id) {
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
                    let take_waiters = |state: &Mutex<InFlightStopState>, lid: Option<u64>, single: Option<CoalescedStopWaiter>| -> Vec<CoalescedStopWaiter> {
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
                    match reader_supervisor.try_enqueue_executor_urgent_stop(stop) {
                        Ok(Some((client, mut rx))) => {
                            let stop_guard = reader_writer.enter_in_flight_stop();
                            let in_flight_c = reader_in_flight_stop.clone();
                            let last_req_c = reader_last_request.clone();
                            let cancel_tx_c = reader_cancel_tx.clone();
                            let ord_q_c = reader_ordinary_queue.clone();
                            let writer_c = reader_writer.clone();
                            reader_tasks.spawn(async move {
                                let _stop_guard = stop_guard;
                                let stop_res = client.await_ticket(&mut rx).await.map_err(ArmError::from);
                                let waiters = take_waiters(&in_flight_c, leader_id, single_waiter);
                                deliver_urgent_stop_batch(
                                    waiters, stop_res, None, &last_req_c, &cancel_tx_c, &ord_q_c, &writer_c,
                                ).await;
                            });
                        }
                        Ok(None) => {
                            let stop_guard = reader_writer.enter_in_flight_stop();
                            let sup_c = reader_supervisor.clone();
                            let in_flight_c = reader_in_flight_stop.clone();
                            let last_req_c = reader_last_request.clone();
                            let cancel_tx_c = reader_cancel_tx.clone();
                            let ord_q_c = reader_ordinary_queue.clone();
                            let writer_c = reader_writer.clone();
                            reader_tasks.spawn(async move {
                                let _stop_guard = stop_guard;
                                let mut pre_drained: Option<Vec<Value>> = None;
                                let stop_res = sup_c.urgent_stop_with_legacy_hook(stop, |applied_epoch, next_gen| {
                                    // Owner `stop_gen` was already advanced under `inner.write()`;
                                    // notify macro cancellation and drain the ordinary queue atomically.
                                    let _ = cancel_tx_c.send(Some(CancelSignal {
                                        gen: next_gen,
                                        code: "STOPPED",
                                        epoch: Some(applied_epoch),
                                    }));
                                    pre_drained = Some(ord_q_c.drain_cancellable("STOPPED", Some(applied_epoch)));
                                }).await;
                                let waiters = take_waiters(&in_flight_c, leader_id, single_waiter);
                                deliver_urgent_stop_batch(
                                    waiters, stop_res, pre_drained, &last_req_c, &cancel_tx_c, &ord_q_c, &writer_c,
                                ).await;
                            });
                        }
                        Err(err) => {
                            let api = ApiError::from(err);
                            let waiters = take_waiters(&reader_in_flight_stop, leader_id, single_waiter);
                            for w in waiters {
                                let reply = format_urgent_stop_outcome(
                                    &w.raw_request_id, w.parsed_request_id, &reader_last_request, Err(&api),
                                );
                                match w.permit {
                                    Some(permit) => { permit.send(reply); }
                                    None => { let _ = reader_writer.try_send_stop_best_effort(reply); }
                                }
                            }
                        }
                    }
                }
                continue;
            }

            // Capture AdmissionStamp ONCE in the reader before queuing (never re-sampled downstream).
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
            if let Err(value) = reader_ordinary_queue.try_push(value, stamp) {
                let kind = value.get("type").and_then(Value::as_str).unwrap_or("");
                if let Some(reply) = backlog_full_reject(kind, &value) {
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
    let recv_connection = connection.clone();
    let recv_generation = connection_generation.clone();
    let recv_client_label = client_label.clone();
    let worker_ordinary_queue = ordinary_queue.clone();
    let worker_writer = writer.clone();
    let worker_tasks = tasks.clone();
    let worker_closing = connection_closing.clone();
    let recv_deps = deps;
    let recv_calib = calib;
    #[cfg(feature = "policy-runtime")]
    let recv_sandbox = sandbox;
    #[cfg(feature = "policy-runtime")]
    let worker_ws_closed = ws_closed.clone();
    let worker_ws_auth = ws_auth.clone();
    let worker_task = tokio::spawn(async move {
        // the operator session of this connection: from the upgrade request, updated by `auth` messages in reader
        let ws_auth = worker_ws_auth;
        let mut last_sequence: Option<u64> = None;
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
                    let stamped_from_legacy = if recv_supervisor.owner_mode() == crate::supervisor::OWNER_LEGACY {
                        let inner = recv_supervisor.inner().read().await;
                        if !inner.detached {
                            reply["epoch_id"] = json!(inner.safety.epoch_id());
                            reply["safety_state"] = json!(inner.safety.state());
                            reply["is_torqued"] = json!(inner.is_torqued);
                            true
                        } else {
                            false
                        }
                    } else {
                        false
                    };
                    if !stamped_from_legacy {
                        if let Some(s) = recv_supervisor.hub().latest() {
                            reply["epoch_id"] = json!(s.obs.epoch_id);
                            reply["safety_state"] = json!(s.obs.safety_state);
                            reply["is_torqued"] = json!(s.obs.is_torqued);
                        }
                    }
                    test_ack_stamp_delay().await;
                    if !worker_writer.send_ordinary_control(reply).await { break; }
                    continue;
                }
                joined = async { (&mut macro_check.as_mut().expect("guarded by the precondition").rx).await }, if macro_active => {
                    let MacroCheck { request_id, generation, stamp, .. } = macro_check.take().expect("checked above");
                    let owner_transferred = || {
                        let mut r = json!({"type":"command_reject", "request_id":request_id, "status":503, "code":"OWNER_TRANSFERRED", "error":"OWNER_TRANSFERRED", "message":"Ownership transferred to RT executor while planning macro"});
                        if let Some(s) = recv_supervisor.hub().latest() { r["epoch_id"] = json!(s.obs.epoch_id); }
                        r
                    };
                    let reply = if worker_closing.load(Ordering::SeqCst) {
                        break;
                    } else if recv_supervisor.owner_mode() != crate::supervisor::OWNER_LEGACY {
                        owner_transferred()
                    } else {
                        let mut inner = recv_supervisor.inner().write().await;
                        if worker_closing.load(Ordering::SeqCst) {
                            drop(inner);
                            break;
                        } else if inner.detached {
                            drop(inner);
                            owner_transferred()
                        } else {
                            if let Err(err) = inner.preflight_admission(stamp) {
                                command_reject(&inner, &request_id, ArmError::from(err).into())
                            } else {
                                match joined {
                                    Err(e) => command_reject(&inner, &request_id, ArmError::Internal(format!("planner task failed: {e}")).into()),
                                    Ok(Err(error)) => command_reject(&inner, &request_id, error.into()),
                                    Ok(Ok(planned)) => {
                                        if recv_generation.load(Ordering::SeqCst) != generation || !inner.validate_servo_lease(&recv_connection, generation) {
                                            json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"LEASE_LOST", "message":"This connection lost the lease while planning", "error":"LEASE_LOST", "epoch_id":inner.safety.epoch_id()})
                                        } else {
                                            match inner.check_admission(stamp).map_err(ArmError::from).and_then(|admitted| planned.start(admitted, &recv_supervisor, &recv_deps)) {
                                                Ok(started) => command_ack(&inner, &request_id, Some(started)),
                                                Err(error) => command_reject(&inner, &request_id, error.into()),
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    };
                    if !worker_writer.send_ordinary_control(reply).await { break; }
                    continue;
                }
                item = worker_ordinary_queue.pop(macro_active) => match item {
                    Some(q) => q,
                    None => break,
                }
            };
            let QueuedOrdinary { value, stamp } = queued;
            let kind = value.get("type").and_then(Value::as_str).unwrap_or("");
            // Fail closed before any legacy lock/reader path (including depth/teach) whenever
            // `ArmCore` is not owned by the legacy supervisor lock.
            let owner_mode = recv_supervisor.owner_mode();
            if owner_mode != crate::supervisor::OWNER_LEGACY {
                if let Some(reply) = executor_routing_reject(kind, &value, owner_mode) {
                    if !worker_writer.send_ordinary_control(reply).await { break; }
                }
                continue;
            }
            // depth measurement: read-only, no motion, no lease
            if kind == "command" && value.get("path").and_then(Value::as_str) == Some(crate::depth_measure::PATH) {
                let request_id = value.get("request_id").and_then(Value::as_u64);
                if request_id.is_none() || last_ordinary_request.is_some_and(|last| request_id.unwrap() <= last) {
                    let reply = json!({"type":"command_reject", "request_id":value.get("request_id"), "status":400, "code":"INVALID_COMMAND", "message":"request_id must be present and strictly increase", "error":"INVALID_COMMAND"});
                    if !worker_writer.send_ordinary_control(reply).await { break; }
                    continue;
                }
                last_ordinary_request = request_id;
                if let Some(e) = lease_free_operator_check(&ws_auth, crate::depth_measure::PATH, &value["body"]) {
                    let mut reply = rejection(e, "command_reject", recv_supervisor.inner().read().await.safety.epoch_id());
                    reply["request_id"] = value["request_id"].clone();
                    if !worker_writer.send_ordinary_control(reply).await { break; }
                    continue;
                }
                let (sup, source, tx) = (recv_supervisor.clone(), recv_deps.source.clone(), ack_tx.clone());
                worker_tasks.spawn(async move {
                    let result = crate::depth_measure::measure(&sup, source.as_ref(), &value["body"]).await;
                    let _ = tx.send(crate::teach::reply(&value["request_id"], result)).await;
                });
                continue;
            }
            // teach commands without a motion lease (episode store; recorder ownership)
            if kind == "command" && crate::teach::is_lease_free(value.get("path").and_then(Value::as_str).unwrap_or("")) {
                let request_id = value.get("request_id").and_then(Value::as_u64);
                if request_id.is_none() || last_ordinary_request.is_some_and(|last| request_id.unwrap() <= last) {
                    let reply = json!({"type":"command_reject", "request_id":value.get("request_id"), "status":400, "code":"INVALID_COMMAND", "message":"request_id must be present and strictly increase", "error":"INVALID_COMMAND"});
                    if !worker_writer.send_ordinary_control(reply).await { break; }
                    continue;
                }
                last_ordinary_request = request_id;
                if let Some(e) = lease_free_operator_check(&ws_auth, value["path"].as_str().unwrap_or(""), &value["body"]) {
                    let mut reply = rejection(e, "command_reject", recv_supervisor.inner().read().await.safety.epoch_id());
                    reply["request_id"] = value["request_id"].clone();
                    if !worker_writer.send_ordinary_control(reply).await { break; }
                    continue;
                }
                let caller_label = recv_client_label.lock().unwrap_or_else(|e| e.into_inner()).clone();
                let caller = crate::teach::Caller {
                    connection: recv_connection.clone(),
                    client_id: value.get("client_id").and_then(Value::as_str).map(str::to_string).or(caller_label),
                };
                let (sup, tx) = (recv_supervisor.clone(), ack_tx.clone());
                worker_tasks.spawn(async move {
                    let path = value["path"].as_str().unwrap_or("").to_string();
                    let result = crate::teach::lease_free(&sup, &caller, &path, &value["body"]).await;
                    let _ = tx.send(crate::teach::reply(&value["request_id"], result)).await;
                });
                continue;
            }
            let mut response: Option<Value>;
            let mut evicted_notify: Option<(String, u64)> = None;
            let mut pending_plan: Option<(PendingPlan, u64)> = None;
            let mut pending_macro: Option<(Value, PendingMacro, u64)> = None;
            let mut pending_replay: Option<(Value, crate::replay::ReplayInput, u64)> = None;
            #[cfg(feature = "policy-runtime")]
            let mut pending_policy: Option<(Value, PendingPolicyStart, u64, PolicyLoadingGuard)> = None;
            let mut pending_dry_run_grasp: Option<(Value, Value)> = None;
            let mut pending_config_write: Option<(Value, PreparedConfigWrite, u64, u64)> = None;
            let mut pending_calibration: Option<(Value, CalibJob, u64, u64, bool)> = None;
            maybe_pause_worker_before_lock(&recv_connection).await;
            {
                let mut inner = recv_supervisor.inner().write().await;
                if inner.detached {
                    drop(inner);
                    if let Some(reply) = executor_routing_reject(kind, &value, recv_supervisor.owner_mode()) {
                        if !worker_writer.send_ordinary_control(reply).await { break; }
                    }
                    continue;
                }
                if kind == "acquire_lease" {
                    let (acq_resp, acq_evicted) = lease::handle_acquire_lease(
                        &mut inner,
                        &value,
                        recv_conn_id,
                        &recv_connection,
                        &recv_generation,
                        &recv_client_label,
                        &worker_closing,
                        &ws_auth,
                        &mut last_sequence,
                    );
                    response = acq_resp;
                    evicted_notify = acq_evicted;
                    } else {
                        let (local_generation, authorized) = lease::check_lease_authority(
                            &mut inner,
                            kind,
                            &value,
                            &recv_connection,
                            &recv_generation,
                            &recv_client_label,
                        );
                        let is_dry_run = kind == "command" && value.get("path").and_then(Value::as_str) == Some("/api/v1/action/grasp") && value.get("body").and_then(|b| b.get("dry_run")).and_then(Value::as_bool) == Some(true);
                        if (kind == "servo" || value.get("mode").and_then(Value::as_str) == Some("cartesian_joint_stream")) && value.get("generation").and_then(Value::as_u64).is_none() {
                            response = Some(json!({"type":"servo_reject", "reason":"missing_generation"}));
                        } else if !authorized && !is_dry_run {
                            response = Some(lease::lease_loss_reply(&inner, kind, &value, &recv_connection, local_generation));
                        } else if kind == "release_lease" {
                            response = Some(lease::handle_release_lease(
                                &mut inner,
                                &recv_connection,
                                local_generation,
                                &recv_generation,
                                &recv_client_label,
                            ));
                        } else {
                            inner.consume_pending_estop();
                            let epoch = inner.safety.epoch_id();
                            if !is_dry_run && value.get("epoch_id").and_then(Value::as_u64) != Some(epoch) {
                                response = Some(if kind == "command" {
                                    json!({"type":"command_reject", "request_id":value.get("request_id"), "status":409, "code":"EPOCH_MISMATCH", "message":"Command epoch is no longer current", "error":"EPOCH_MISMATCH", "epoch_id":epoch})
                                } else { json!({"type":"epoch_sync", "epoch_id":epoch, "safety_state":inner.safety.state()}) });
                            } else if kind == "heartbeat" {
                                response = lease::handle_heartbeat_inline(&mut inner, &recv_connection, epoch);
                            } else if kind == "command" {
                                let request_id = value.get("request_id").and_then(Value::as_u64);
                                if request_id.is_none() || last_ordinary_request.is_some_and(|last| request_id.unwrap() <= last) {
                                    response = Some(json!({"type":"command_reject", "request_id":value.get("request_id"), "status":400, "code":"INVALID_COMMAND", "message":"request_id must be present and strictly increase", "error":"INVALID_COMMAND"}));
                                } else {
                                    last_ordinary_request = request_id;
                                    let path = value.get("path").and_then(Value::as_str).unwrap_or("");
                                    if let Some(auth_err) = check_operator_privilege(&ws_auth, &mut inner, path, &value["body"], epoch) {
                                        response = Some(command_reject(&inner, &value["request_id"], auth_err));
                                    } else if let Some(code) = calibration_busy_code(&inner, path) {
                                        response = Some(command_reject(&inner, &value["request_id"],
                                            ApiError(StatusCode::CONFLICT, "a calibration apply, restore or motor zero is running".into(), code.into())));
                                    } else if path == "/api/v1/calibration/apply" || path == "/api/v1/calibration/restore" || path == crate::motor_zero::PATH
                                        || path == crate::motor_zero::RESTORE_PATH || path == crate::mit::TIMEOUT_PATH || path == crate::mit::QUALIFICATION_PATH {
                                        response = match calibration_job(&mut inner, path, &value["body"]) {
                                            Ok(CalibStart::Job(job, busy)) => { pending_calibration = Some((value["request_id"].clone(), job, local_generation, epoch, busy)); None }
                                            Ok(CalibStart::Reply(reply)) => Some(fields_ack(&value["request_id"], reply)),
                                            Err(e) => Some(calibration_reject(&value["request_id"], &e)),
                                        };
                                    } else if path.starts_with("/api/v1/action/") {
                                        let (auth_snap, token_snap) = ws_auth.auth_and_token();
                                        response = match prepare_action(&mut inner, stamp, path, &value["body"], epoch, &value["request_id"], &auth_snap, token_snap.as_deref()) {
                                            Ok(ActionStart::Started(id, duration)) => Some(command_ack(&inner, &value["request_id"], Some((id, duration)))),
                                            Ok(ActionStart::Plan(plan)) => { pending_plan = Some((plan, local_generation)); None }
                                            Ok(ActionStart::Macro(m)) => { pending_macro = Some((value["request_id"].clone(), m, local_generation)); None }
                                            Ok(ActionStart::Replay(input)) => { pending_replay = Some((value["request_id"].clone(), input, local_generation)); None }
                                            Ok(ActionStart::DryRunGrasp(body)) => { pending_dry_run_grasp = Some((value["request_id"].clone(), body)); None }
                                            Ok(ActionStart::Sequence(plan)) => Some(match inner.check_admission(stamp).map_err(ArmError::from).and_then(|admitted| crate::sequences::begin(admitted, &recv_supervisor, *plan)) {
                                                Ok(started) => command_ack(&inner, &value["request_id"], Some(started)),
                                                Err(error) => command_reject(&inner, &value["request_id"], error.into()),
                                            }),
                                            Ok(ActionStart::CalibHover(plan)) => Some(match inner.check_admission(stamp).map_err(ArmError::from).and_then(|admitted| crate::calib_hover::begin(admitted, &recv_supervisor, *plan,
                                                recv_calib.upstream.clone(), crate::calib_hover::jobs_dir(&recv_calib.path))) {
                                                Ok((id, duration, job_id)) => {
                                                    let mut ack = command_ack(&inner, &value["request_id"], Some((id, duration)));
                                                    ack["job_id"] = json!(job_id);
                                                    ack
                                                }
                                                Err(error) => command_reject(&inner, &value["request_id"], error.into()),
                                            }),
                                            Ok(ActionStart::GripperCalibrate(epoch, tag)) => Some(match inner.check_admission(stamp).map_err(ArmError::from).and_then(|admitted| crate::gripper_calib::begin_calibration(admitted, &recv_supervisor, epoch, tag)) {
                                                Ok(started) => command_ack(&inner, &value["request_id"], Some(started)),
                                                Err(error) => command_reject(&inner, &value["request_id"], error.into()),
                                            }),
                                            Ok(ActionStart::MeasureMultiview(plan)) => Some(match inner.check_admission(stamp).map_err(ArmError::from).and_then(|admitted| crate::measure_multiview::begin(admitted, &recv_supervisor, *plan, recv_deps.source.clone())) {
                                                Ok(started) => command_ack(&inner, &value["request_id"], Some(started)),
                                                Err(error) => command_reject(&inner, &value["request_id"], error.into()),
                                            }),
                                            #[cfg(feature = "policy-runtime")]
                                            Ok(ActionStart::Policy(p)) => {
                                                inner.policy_loading = true;
                                                let loading_guard = PolicyLoadingGuard::new(recv_supervisor.clone());
                                                pending_policy = Some((value["request_id"].clone(), *p, local_generation, loading_guard));
                                                None
                                            }
                                            #[cfg(feature = "policy-runtime")]
                                            Ok(ActionStart::PolicyStopped(stopped)) => {
                                                let had_pending = pending_policy_loads.load(Ordering::SeqCst) > 0;
                                                let mut ack = command_ack(&inner, &value["request_id"], None);
                                                ack["stopped"] = json!(stopped || had_pending);
                                                Some(ack)
                                            }
                                            Err(error) => Some(command_reject(&inner, &value["request_id"], error)),
                                        };
                                    } else if let Some(result) = PreparedConfigWrite::prepare(
                                        &mut inner,
                                        path,
                                        &value["body"],
                                        recv_deps.setup_path.as_ref(),
                                        ws_auth.is_enforced(),
                                    ) {
                                        match result {
                                            Ok(prepared) if !prepared.has_write() => {
                                                response = Some(match prepared.commit(&mut inner) {
                                                    Ok(fields) => {
                                                        let mut ack = command_ack(&inner, &value["request_id"], None);
                                                        for (key, field) in fields { ack[key] = field; }
                                                        ack
                                                    }
                                                    Err(error) => command_reject(&inner, &value["request_id"], error.into()),
                                                });
                                            }
                                            Ok(prepared) => {
                                                pending_config_write = Some((value["request_id"].clone(), prepared, local_generation, epoch));
                                                response = None;
                                            }
                                            Err(error) => {
                                                response = Some(command_reject(&inner, &value["request_id"], error.into()));
                                            }
                                        }
                                    } else {
                                        let result = execute_command(&mut inner, stamp, path, &value["body"], epoch);
                                        response = Some(match result {
                                            Ok(started) => command_ack(&inner, &value["request_id"], started),
                                            Err(error) => command_reject(&inner, &value["request_id"], error),
                                        });
                                    }
                                }
                            } else if kind == "preset" && inner.calibration_busy {
                                response = Some(json!({"type":"preset_reject", "name":value.get("name"), "status":409, "code":"MOTION_BUSY", "error":"MOTION_BUSY",
                                    "message":"a camera calibration apply or restore is running", "epoch_id":epoch}));
                            } else if kind == "preset" {
                                let name = value.get("name").and_then(Value::as_str).unwrap_or("");
                                let (path, body) = match name {
                                    "home" => ("/api/v1/motion/home", json!({})),
                                    "park" | "unload" => ("/api/v1/motion/unload", json!({})),
                                    _ => {
                                        let mut j = json!({"target_positions":value.get("positions")});
                                        if let Some(s) = value.get("speed_scale") {
                                            j["speed_scale"] = s.clone();
                                        }
                                        ("/api/v1/motion/jog", j)
                                    }
                                };
                                response = Some(match execute_command(&mut inner, stamp, path, &body, epoch) {
                                    Ok(started) => {
                                        let mut ack = json!({"type":"preset_ack", "name":name, "epoch_id":inner.safety.epoch_id()});
                                        if let Some((id, _)) = started { ack["action_id"] = json!(id); }
                                        ack
                                    }
                                    Err(error) => {
                                        let mut reply = rejection(error, "preset_reject", inner.safety.epoch_id());
                                        reply["name"] = json!(name);
                                        reply
                                    },
                                });
                            } else if kind == "servo" || value.get("mode").and_then(Value::as_str) == Some("cartesian_joint_stream") || value.get("positions").is_some() {
                                match value.get("sequence_id").and_then(Value::as_u64) {
                                    _ if inner.calibration_busy => response = Some(json!({"type":"servo_reject", "reason":"calibration_busy"})),
                                    None => response = Some(json!({"type":"servo_reject", "reason":"missing_sequence_id"})),
                                    Some(sequence) if last_sequence.is_some_and(|last| { let delta = sequence.wrapping_sub(last); delta == 0 || delta > 1000 }) => {
                                        response = Some(json!({"type":"servo_reject", "reason":"invalid_sequence_id"}));
                                    }
                                    Some(sequence) => {
                                        last_sequence = Some(sequence);
                                        response = match positions(value.get("positions")).and_then(|targets| {
                                            let admitted = inner.check_admission(stamp).map_err(|error| ApiError::from(ArmError::from(error)))?;
                                            SupervisorInner::servo(admitted, targets, epoch).map_err(|error| ApiError::from(ArmError::from(error)))
                                        }) {
                                            Ok(()) => None,
                                            Err(error) => {
                                                let stopped = error.code.as_deref() == Some("STOPPED");
                                                let mut r = rejection(error, "servo_reject", epoch);
                                                if stopped { r["reason"] = json!("stopped"); }
                                                Some(r)
                                            }
                                        };
                                    }
                                }
                            } else { response = None; }
                        }
                    }
                }

                // Cartesian plan: IK runs on a blocking thread without the supervisor lock,
                // so the control tick is never blocked by it; cancellable between steps and
                // late results after teardown are ignored before installing anything.
                if let Some((plan, generation)) = pending_plan.take() {
                    let PendingPlan { request_id, kind, input, goal, speed, tag, opts, approach, joint, seeds } = plan;
                    if let Some(joint) = joint {
                        // path "joint": IK of the goal only (several seeds) without the lock,
                        // then the move_to_pose quintic, checked under the lock
                        // ... and the quintic's plan, power fit and swept-workspace check, which
                        // the install reuses when the arm and the workspace did not change
                        let JointGoal { speed_scale, same_side, snapshot } = joint;
                        let cancel = worker_closing.clone();
                        let plan_conn = recv_connection.clone();
                        let solved = tokio::task::spawn_blocking(move || {
                            maybe_pause_blocking_plan(&plan_conn);
                            if cancel.load(Ordering::Acquire) {
                                return None;
                            }
                            let q = motion::solve_goal_joints(&input, &goal, opts, &seeds);
                            if cancel.load(Ordering::Acquire) {
                                return None;
                            }
                            let prepared = match (&q, same_side) {
                                (Ok(q), None) => crate::supervisor::prepare_quintic(&input, &snapshot, q),
                                _ => None,
                            };
                            if cancel.load(Ordering::Acquire) {
                                return None;
                            }
                            Some((input, q.map(|q| (q, prepared))))
                        }).await;
                        if worker_closing.load(Ordering::SeqCst) {
                            break;
                        }
                        let mut inner = recv_supervisor.inner().write().await;
                        if worker_closing.load(Ordering::SeqCst) {
                            break;
                        }
                        if inner.detached {
                            let mut r = json!({"type":"command_reject", "request_id":request_id, "status":503, "code":"OWNER_TRANSFERRED", "error":"OWNER_TRANSFERRED", "message":"Ownership transferred to RT executor while planning"});
                            if let Some(s) = recv_supervisor.hub().latest() { r["epoch_id"] = json!(s.obs.epoch_id); }
                            response = Some(r);
                        } else {
                            if let Some(hook) = inner.before_install_hook.take() { hook(&mut inner); }
                            if let Err(err) = inner.preflight_admission(stamp) {
                                response = Some(command_reject(&inner, &request_id, ArmError::from(err).into()));
                            } else {
                                response = match solved {
                                    Ok(None) => None,
                                    Err(e) => Some(command_reject(&inner, &request_id, ArmError::Internal(format!("planner task failed: {e}")).into())),
                                    Ok(Some((_, Err(error)))) => Some(command_reject(&inner, &request_id, ArmError::from(error).into())),
                                    Ok(Some((input, Ok((q, prepared))))) => Some(
                                        if recv_generation.load(Ordering::SeqCst) != generation || !inner.validate_servo_lease(&recv_connection, generation) {
                                            json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"LEASE_LOST", "message":"This connection lost the lease while planning", "error":"LEASE_LOST", "epoch_id":inner.safety.epoch_id()})
                                        } else {
                                            match inner.check_admission(stamp).map_err(ArmError::from).and_then(|admitted| SupervisorInner::install_joint_goal_prepared(admitted, &input, &q, speed_scale, kind, tag, same_side, prepared)) {
                                                Ok(started) => command_ack(&inner, &request_id, Some(started)),
                                                Err(error) => command_reject(&inner, &request_id, error.into()),
                                            }
                                        }
                                    ),
                                };
                            }
                        }
                    } else {
                        let cancel = worker_closing.clone();
                        let plan_conn = recv_connection.clone();
                        let planned = tokio::task::spawn_blocking(move || {
                            maybe_pause_blocking_plan(&plan_conn);
                            if cancel.load(Ordering::Acquire) {
                                return None;
                            }
                            let result = motion::plan_cartesian_opts(&input, goal, speed, opts);
                            if cancel.load(Ordering::Acquire) {
                                return None;
                            }
                            let pre = match (&approach, &result) {
                                (Ok(plan), Ok(direct)) => crate::backlash::plan_cartesian_pre(&input, direct, plan, None, speed, opts),
                                (Err(Some(r)), _) => crate::backlash::Pre::Direct((*r).to_string()),
                                _ => crate::backlash::Pre::Off,
                            };
                            if cancel.load(Ordering::Acquire) {
                                return None;
                            }
                            // a branch flip: would the joint-space move reach the goal (hint)?
                            let joint_goal = match &result {
                                Err(motion::MotionError::IkBranchFlip { .. }) => motion::solve_goal_joints(&input, &goal, opts, &seeds).ok(),
                                _ => None,
                            };
                            if cancel.load(Ordering::Acquire) {
                                return None;
                            }
                            Some((input, result.map(|path| (path, pre, approach.ok())).map_err(|e| (e, joint_goal))))
                        }).await;
                        if worker_closing.load(Ordering::SeqCst) {
                            break;
                        }
                        let mut inner = recv_supervisor.inner().write().await;
                        if worker_closing.load(Ordering::SeqCst) {
                            break;
                        }
                        if inner.detached {
                            let mut r = json!({"type":"command_reject", "request_id":request_id, "status":503, "code":"OWNER_TRANSFERRED", "error":"OWNER_TRANSFERRED", "message":"Ownership transferred to RT executor while planning"});
                            if let Some(s) = recv_supervisor.hub().latest() { r["epoch_id"] = json!(s.obs.epoch_id); }
                            response = Some(r);
                        } else {
                            if let Some(hook) = inner.before_install_hook.take() { hook(&mut inner); }
                            if let Err(err) = inner.preflight_admission(stamp) {
                                response = Some(command_reject(&inner, &request_id, ArmError::from(err).into()));
                            } else {
                                response = match planned {
                                    Ok(None) => None,
                                    Err(e) => Some(command_reject(&inner, &request_id, ArmError::Internal(format!("planner task failed: {e}")).into())),
                                    Ok(Some((_, Err((error, joint_goal))))) => {
                                        let mut api: ApiError = ArmError::from(error).into();
                                        if joint_goal.is_some_and(|q| inner.joint_goal_feasible(&q)) { api.message.push_str(motion::JOINT_PATH_HINT); }
                                        Some(command_reject(&inner, &request_id, api))
                                    }
                                    Ok(Some((input, Ok((path, pre, approach))))) => Some(
                                        if recv_generation.load(Ordering::SeqCst) != generation || !inner.validate_servo_lease(&recv_connection, generation) {
                                            json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"LEASE_LOST", "message":"This connection lost the lease while planning", "error":"LEASE_LOST", "epoch_id":inner.safety.epoch_id()})
                                        } else {
                                            match inner.check_admission(stamp).map_err(ArmError::from).and_then(|admitted| SupervisorInner::install_path_same_side(admitted, &input, path, pre, approach.as_ref(), kind, tag)) {
                                                Ok(started) => command_ack(&inner, &request_id, Some(started)),
                                                Err(error) => command_reject(&inner, &request_id, error.into()),
                                            }
                                        }
                                    ),
                                };
                            }
                        }
                    }
                }

                // Macro actions (grasp, place): the first motions are planned without the lock
                // (feasibility check), then the action begins and the orchestrator task takes over.
                // The check runs in its own task; its result is taken by the select! above
                // (lease re-check, then begin), while later ordinary commands stay in the bounded queue.
                if let Some((request_id, pending, generation)) = pending_macro.take() {
                    let source = recv_deps.source.clone();
                    let cancel = worker_closing.clone();
                    let plan_conn = recv_connection.clone();
                    let (res_tx, res_rx) = tokio::sync::oneshot::channel();
                    let abort = worker_tasks.spawn_abortable(async move {
                        let planned_res = async {
                            let cancel_blk = cancel.clone();
                            let mut planned = tokio::task::spawn_blocking(move || {
                                maybe_pause_blocking_plan(&plan_conn);
                                if cancel_blk.load(Ordering::Acquire) {
                                    return Err(ArmError::Safety(SafetyError::Stopped));
                                }
                                let out = pending.plan()?;
                                if cancel_blk.load(Ordering::Acquire) {
                                    return Err(ArmError::Safety(SafetyError::Stopped));
                                }
                                Ok(out)
                            }).await
                                .map_err(|e| ArmError::Internal(format!("planner task failed: {e}")))??;
                            if cancel.load(Ordering::Acquire) {
                                return Err(ArmError::Safety(SafetyError::Stopped));
                            }
                            // grasp task requirements: checked without the lock, before the action begins
                            if let PlannedMacro::Grasp(g) = &mut planned { crate::grasp::preflight(source.as_ref(), g).await?; }
                            if cancel.load(Ordering::Acquire) {
                                return Err(ArmError::Safety(SafetyError::Stopped));
                            }
                            Ok(planned)
                        }.await;
                        if !cancel.load(Ordering::Acquire) {
                            let _ = res_tx.send(planned_res);
                        }
                    });
                    macro_check = Some(MacroCheck {
                        request_id,
                        generation,
                        stamp,
                        abort: motion_cmds::AbortOnDrop(abort),
                        rx: res_rx,
                    });
                }
                // Replay: the episode is read and checked without the lock; cancellable and late
                // results after teardown are ignored before installing anything.
                if let Some((request_id, input, generation)) = pending_replay.take() {
                    let cancel = worker_closing.clone();
                    let plan_conn = recv_connection.clone();
                    let planned = tokio::task::spawn_blocking(move || {
                        maybe_pause_blocking_plan(&plan_conn);
                        if cancel.load(Ordering::Acquire) {
                            return None;
                        }
                        let out = crate::replay::plan(input);
                        if cancel.load(Ordering::Acquire) {
                            return None;
                        }
                        Some(out)
                    }).await;
                    if worker_closing.load(Ordering::SeqCst) {
                        break;
                    }
                    let mut inner = recv_supervisor.inner().write().await;
                    if worker_closing.load(Ordering::SeqCst) {
                        break;
                    }
                    if inner.detached {
                        let mut r = json!({"type":"command_reject", "request_id":request_id, "status":503, "code":"OWNER_TRANSFERRED", "error":"OWNER_TRANSFERRED", "message":"Ownership transferred to RT executor while checking replay"});
                        if let Some(s) = recv_supervisor.hub().latest() { r["epoch_id"] = json!(s.obs.epoch_id); }
                        response = Some(r);
                    } else if let Err(err) = inner.preflight_admission(stamp) {
                        response = Some(command_reject(&inner, &request_id, ArmError::from(err).into()));
                    } else {
                        response = match planned {
                            Ok(None) => None,
                            Err(e) => Some(command_reject(&inner, &request_id, ArmError::Internal(format!("replay check task failed: {e}")).into())),
                            Ok(Some(Err(error))) => Some(command_reject(&inner, &request_id, error.into())),
                            Ok(Some(Ok(plan))) => Some(
                                if recv_generation.load(Ordering::SeqCst) != generation || !inner.validate_servo_lease(&recv_connection, generation) {
                                    json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"LEASE_LOST", "message":"This connection lost the lease while the episode was checked", "error":"LEASE_LOST", "epoch_id":inner.safety.epoch_id()})
                                } else {
                                    match inner.check_admission(stamp).map_err(ArmError::from).and_then(|admitted| SupervisorInner::install_replay(admitted, plan)) {
                                        Ok(started) => command_ack(&inner, &request_id, Some(started)),
                                        Err(error) => command_reject(&inner, &request_id, error.into()),
                                    }
                                }
                            ),
                        };
                    }
                }
                #[cfg(feature = "policy-runtime")]
                if let Some((request_id, pending, generation, loading_guard)) = pending_policy.take() {
                    let sup = recv_supervisor.clone();
                    let sandbox_c = recv_sandbox.clone();
                    let vision_src = recv_deps.source.clone();
                    let conn = recv_connection.clone();
                    let gen_now = recv_generation.clone();
                    let tx = ack_tx.clone();
                    let ws_closed_c = worker_ws_closed.clone();
                    let policy_stamp = pending.stamp;
                    let pending_loads_c = pending_policy_loads.clone();
                    pending_loads_c.fetch_add(1, Ordering::SeqCst);
                    struct PendingLoadGuard(Arc<AtomicUsize>);
                    impl Drop for PendingLoadGuard {
                        fn drop(&mut self) {
                            self.0.fetch_sub(1, Ordering::SeqCst);
                        }
                    }
                    let load_guard = PendingLoadGuard(pending_loads_c);
                    let epoch_id = pending.epoch;
                    let policy_id = pending.request.policy_id.clone();
                    let config = pending.config;
                    let request_mode = pending.request.mode;
                    let request_sim_only = pending.request.sim_only;
                    let request_fps = pending.request.fps;
                    let inline_release_gate = pending.request.release_gate;
                    let policy_live_allowed = pending.policy_live_allowed;
                    let i_am_at_the_arm = pending.request.i_am_at_the_arm;
                    let ws_auth_c = ws_auth.clone();
                    worker_tasks.spawn(async move {
                        let _load_guard = load_guard;
                        let mut loading_guard = loading_guard;
                        let policy_id_load = policy_id.clone();
                        let deployment = if request_mode == crate::policy_runtime::PolicyMode::Live && !policy_live_allowed {
                            let (scene_id, tool_id) = sandbox_c.active_scene_and_tool().await;
                            let calibration_digest = {
                                let inner = sup.inner().read().await;
                                inner.active_calibration_digest()
                            };
                            Some(crate::policy_runtime::ActiveDeploymentContext {
                                scene_id,
                                tool_id,
                                calibration_digest,
                            })
                        } else {
                            None
                        };
                        let sandbox_load = sandbox_c.clone();
                        let cancel_policy = ws_closed_c.clone();
                        let plan_conn = conn.clone();
                        let prepared = tokio::task::spawn_blocking(move || {
                            maybe_pause_blocking_plan(&plan_conn);
                            let delay_ms = sandbox_load.policy_load_delay_ms();
                            if delay_ms > 0 && !sleep_cancellable_ms(delay_ms, &cancel_policy) {
                                return Err(ArmError::Safety(SafetyError::Stopped));
                            }
                            if cancel_policy.load(Ordering::Acquire) {
                                return Err(ArmError::Safety(SafetyError::Stopped));
                            }
                            let dir = sandbox_load.policy_dir(&policy_id_load).ok_or_else(|| ArmError::Rejected {
                                status: 404,
                                code: "POLICY_NOT_FOUND".into(),
                                message: format!("Policy pack '{policy_id_load}' not found"),
                            })?;
                            let mut prepared_policy = crate::policy_runtime::prepare_policy(&dir, &config).map_err(ArmError::from)?;
                            if cancel_policy.load(Ordering::Acquire) {
                                return Err(ArmError::Safety(SafetyError::Stopped));
                            }
                            let infer_delay_ms = sandbox_load.policy_infer_delay_ms();
                            if infer_delay_ms > 0 {
                                prepared_policy = prepared_policy.with_infer_delay(std::time::Duration::from_millis(infer_delay_ms));
                            }
                            let effective_fps = request_fps
                                .or(prepared_policy.contract().fps)
                                .unwrap_or(crate::policy_runtime::DEFAULT_POLICY_FPS);
                            let effective_fps = crate::policy_runtime::validate_policy_fps(request_mode, effective_fps)
                                .map_err(ArmError::from)?;
                            let verified_release_gate = if let Some(ref deployment_ctx) = deployment {
                                let raw_gate = match inline_release_gate {
                                    Some(v) => Some(v),
                                    None => crate::policy_runtime::load_release_gate_json(&dir).map_err(ArmError::from)?,
                                };
                                match raw_gate {
                                    Some(raw) => Some(
                                        crate::policy_runtime::ReleaseGate::parse_and_verify(
                                            &raw,
                                            prepared_policy.artifact_digest(),
                                            &config,
                                            prepared_policy.contract(),
                                            effective_fps,
                                            deployment_ctx,
                                        )
                                        .map_err(ArmError::from)?,
                                    ),
                                    None => None,
                                }
                            } else {
                                None
                            };
                            if cancel_policy.load(Ordering::Acquire) {
                                return Err(ArmError::Safety(SafetyError::Stopped));
                            }
                            // speed gate: measured here, off the supervisor lock and the control tick
                            let warmup = prepared_policy
                                .warmup(crate::policy_runtime::WARMUP_INFERENCES, crate::policy_runtime::WARMUP_MAX_TOTAL)
                                .map_err(ArmError::from)?;
                            if cancel_policy.load(Ordering::Acquire) {
                                return Err(ArmError::Safety(SafetyError::Stopped));
                            }
                            let speed = crate::policy_runtime::SpeedGateReport::new(warmup, effective_fps);
                            Ok::<_, ArmError>((prepared_policy, effective_fps, verified_release_gate, speed))
                        }).await;

                        if ws_closed_c.load(Ordering::SeqCst) {
                            let mut inner = sup.inner().write().await;
                            loading_guard.clear(&mut inner);
                            return;
                        }

                        let reply = match prepared {
                            Err(e) => {
                                let mut inner = sup.inner().write().await;
                                loading_guard.clear(&mut inner);
                                command_reject(&inner, &request_id, ArmError::Internal(format!("policy load task failed: {e}")).into())
                            }
                            Ok(Err(error)) => {
                                let mut inner = sup.inner().write().await;
                                loading_guard.clear(&mut inner);
                                command_reject(&inner, &request_id, error.into())
                            }
                            Ok(Ok((prepared_policy, fps, verified_release_gate, speed))) => {
                                let deadline_res = crate::policy_runtime::compute_policy_deadline(pending.request.max_seconds)
                                    .map_err(ArmError::from);
                                // Fetch deployment context BEFORE taking the owner lock so NO `.await`
                                // ever happens while holding `inner` and connection-closed is checked
                                // atomically under the same lock as install.
                                let active_scene_tool = if verified_release_gate.is_some() {
                                    Some(sandbox_c.active_scene_and_tool().await)
                                } else {
                                    None
                                };
                                maybe_pause_worker_before_lock(&conn).await;

                                let mut inner = sup.inner().write().await;
                                loading_guard.clear(&mut inner);
                                if ws_closed_c.load(Ordering::SeqCst) || inner.is_connection_closed(recv_conn_id) {
                                    return;
                                }
                                if gen_now.load(Ordering::SeqCst) != generation || !inner.validate_servo_lease(&conn, generation) {
                                    json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"LEASE_LOST", "message":"This connection lost the lease while loading policy", "error":"LEASE_LOST", "epoch_id":inner.safety.epoch_id()})
                                } else if inner.safety.epoch_id() != epoch_id {
                                    json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"EPOCH_MISMATCH", "message":"Epoch changed while loading policy", "error":"EPOCH_MISMATCH", "epoch_id":inner.safety.epoch_id()})
                                } else if !inner.policy_live_allowed() && !ws_auth_c.is_operator() {
                                    // the operator session may have expired or been locked while loading
                                    command_reject(&inner, &request_id, ApiError::operator_required("policy/start (real arm)"))
                                } else if let Err(err) = speed.check(request_mode) {
                                    let mut reply = command_reject(&inner, &request_id, err.into());
                                    for (k, v) in speed.ack_fields() { reply[k] = v; }
                                    reply
                                } else {
                                    match deadline_res {
                                        Err(err) => command_reject(&inner, &request_id, err.into()),
                                        Ok(deadline) => {
                                            let deployment_check = if let Some(ref gate) = verified_release_gate {
                                                let (cur_scene, cur_tool) = active_scene_tool.unwrap_or_default();
                                                let cur_ctx = crate::policy_runtime::ActiveDeploymentContext {
                                                    scene_id: cur_scene,
                                                    tool_id: cur_tool,
                                                    calibration_digest: inner.active_calibration_digest(),
                                                };
                                                gate.verify_deployment(&cur_ctx).map_err(ArmError::from)
                                            } else {
                                                Ok(())
                                            };
                                            if let Err(err) = deployment_check {
                                                command_reject(&inner, &request_id, err.into())
                                            } else {
                                                let start_req = crate::policy_runtime::PolicyRunStart {
                                                    epoch_id,
                                                    connection_id: conn,
                                                    lease_generation: generation,
                                                    policy_id: policy_id.clone(),
                                                    artifact_digest: prepared_policy.artifact_digest().to_string(),
                                                    backend: prepared_policy.backend_name().to_string(),
                                                    mode: request_mode,
                                                    sim_only: request_sim_only,
                                                    max_first_step_rad: prepared_policy.contract().max_first_step_rad,
                                                    max_step_rad: prepared_policy.contract().max_step_rad,
                                                    tag: pending.request.tag,
                                                    i_am_at_the_arm,
                                                    verified_release_gate,
                                                };
                                                match inner.check_admission(policy_stamp).map_err(ArmError::from).and_then(|admitted| SupervisorInner::begin_policy_run(admitted, start_req)) {
                                                Ok(action_id) => {
                                                    let digest = prepared_policy.artifact_digest().to_string();
                                                    let backend = prepared_policy.backend_name().to_string();
                                                    let mode_str = pending.request.mode.as_str().to_string();
                                                    let vision_source = prepared_policy
                                                        .contract()
                                                        .vision
                                                        .as_ref()
                                                        .map(|_| {
                                                            Arc::new(crate::policy_runtime::DetectionSourceAdapter(
                                                                vision_src.clone(),
                                                            ))
                                                                as Arc<dyn crate::policy_runtime::PolicyVisionSource>
                                                        });
                                                    crate::policy_runtime::spawn_policy_runner(
                                                        sup.clone(),
                                                        action_id,
                                                        prepared_policy,
                                                        fps,
                                                        deadline,
                                                        vision_source,
                                                        Some(sandbox_c.clone()),
                                                    );
                                                    let mut ack = command_ack(&inner, &request_id, Some((action_id, pending.request.max_seconds.unwrap_or(0.0))));
                                                    ack["policy_id"] = json!(policy_id);
                                                    ack["artifact_digest"] = json!(digest);
                                                    ack["backend"] = json!(backend);
                                                    ack["mode"] = json!(mode_str);
                                                    for (k, v) in speed.ack_fields() { ack[k] = v; }
                                                    ack
                                                }
                                                Err(error) => command_reject(&inner, &request_id, error.into()),
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        };
                        let _ = tx.send(reply).await;
                    });
                }
                if let Some((request_id, body)) = pending_dry_run_grasp.take() {
                    let dry = crate::grasp::dry_run_task(&recv_supervisor, recv_deps.source.as_ref(), &body).await;
                    let mut ack = {
                        let inner = recv_supervisor.inner().read().await;
                        if !inner.detached {
                            command_ack(&inner, &request_id, None)
                        } else if let Some(s) = recv_supervisor.hub().latest() {
                            let control_mode = if s.obs.control_mode == crate::model::ControlMode::GravityComp { "gravity_comp" } else { "position" };
                            json!({"type":"command_ack", "request_id":request_id, "status":"accepted", "epoch_id":s.obs.epoch_id, "safety_state":s.obs.safety_state, "is_torqued":s.obs.is_torqued, "control_mode":control_mode})
                        } else {
                            command_ack(&inner, &request_id, None)
                        }
                    };
                    for (k, v) in dry.fields() { ack[k.as_str()] = v; }
                    response = Some(ack);
                }
                // Calibration apply / restore: its own tracked task (HTTP to rebot-vision, file writes);
                // this loop goes on (heartbeats, stop) and the reply comes back through ack_tx.
                if let Some((request_id, job, generation, epoch, busy)) = pending_calibration.take() {
                    let (sup, calib, conn, gen_now, tx) = (recv_supervisor.clone(), recv_calib.clone(), recv_connection.clone(), recv_generation.clone(), ack_tx.clone());
                    let ws_auth_c = ws_auth.clone();
                    let tasks_c = worker_tasks.clone();
                    worker_tasks.spawn(async move {
                        let check: crate::calibration::LeaseCheck = Box::new(move |inner: &mut SupervisorInner| {
                            if !ws_auth_c.is_operator() {
                                return Err(crate::calibration::CalibError::new(403, "OPERATOR_REQUIRED", "Operator session expired or locked during calibration"));
                            }
                            if gen_now.load(Ordering::SeqCst) != generation || !inner.validate_servo_lease(&conn, generation) {
                                return Err(crate::calibration::CalibError::new(409, "LEASE_LOST", "This connection lost the lease during the calibration"));
                            }
                            if inner.safety.epoch_id() != epoch {
                                return Err(crate::calibration::CalibError::new(409, "EPOCH_MISMATCH", "The epoch changed during the calibration (stop or e-stop)"));
                            }
                            Ok(())
                        });
                        // the busy flag is released by the guard, also when the job panics
                        let guard = busy.then(|| crate::calibration::BusyGuard::new(sup.clone()));
                        let job_sup = sup.clone();
                        let (job_tx, job_rx) = tokio::sync::oneshot::channel();
                        let job_abort = tasks_c.spawn_abortable(async move {
                            let res = match job {
                                CalibJob::Apply(req) => crate::calibration::apply(&job_sup, &calib, *req, &check).await,
                                CalibJob::Restore(body) => crate::calibration::restore(&job_sup, &calib, &body, &check).await,
                                CalibJob::MotorZero(req) => crate::motor_zero::run(&job_sup, &calib, req, &check).await,
                                CalibJob::JointZeroRestore(name) => crate::motor_zero::restore(&job_sup, &calib, name, &check).await,
                                CalibJob::MitQualification(file) => crate::mit::store_qualification(&job_sup, &calib, *file, &check).await,
                            };
                            let _ = job_tx.send(res);
                        });
                        let _abort_inner = motion_cmds::AbortOnDrop(job_abort);
                        let result = job_rx.await.unwrap_or_else(|e| {
                            tracing::error!("calibration task failed: {e}");
                            Err(crate::calibration::CalibError::new(500, "INTERNAL", format!("calibration task failed: {e}; check GET /api/v1/calibration and the backups")))
                        });
                        if let Some(g) = guard { g.release().await; }
                        let reply = match result {
                            Ok(v) => fields_ack(&request_id, v),
                            Err(e) => calibration_reject(&request_id, &e),
                        };
                        let _ = tx.send(reply).await;
                    });
                }
                // Atomic two-phase persist-before-commit config writes (track B, `config::persist_and_activate`).
                if let Some((request_id, prepared, generation, epoch)) = pending_config_write.take() {
                    let sup = recv_supervisor.clone();
                    let tx = ack_tx.clone();
                    let (ws_auth_c, conn_c, gen_now) = (ws_auth.clone(), recv_connection.clone(), recv_generation.clone());
                    worker_tasks.spawn(async move {
                        let reply = persist_and_activate(sup, request_id, prepared, generation, epoch, ws_auth_c, conn_c, gen_now).await;
                        let _ = tx.send(reply).await;
                    });
                }
                if let Some((evicted_conn, evicted_gen)) = evicted_notify {
                    lease::notify_evicted(evicted_conn, evicted_gen);
                }
            // Control replies are never silently dropped: saturation closes the connection with an error.
            if let Some(response) = response {
                if response.get("type").and_then(Value::as_str) == Some("lease_granted") {
                    let delay_ms = TEST_LEASE_GRANTED_DELAY_MS.load(Ordering::Relaxed);
                    if delay_ms > 0 {
                        tokio::time::sleep(Duration::from_millis(delay_ms)).await;
                    }
                }
                if !worker_writer.send_ordinary_control(response).await { break; }
            }
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
    // Mark connection closing and record `connection_closed` under the owner lock FIRST,
    // before awaiting ANY task join, so no resuming async install (policy/start after deployment
    // lookup, planning, macro) or stale `Admitted` token from this connection can win the owner
    // lock while teardown waits on worker/heartbeat/auth/writer/reader/child joins.
    connection_closing.store(true, Ordering::SeqCst);
    #[cfg(feature = "policy-runtime")]
    ws_closed.store(true, Ordering::SeqCst);
    ordinary_queue.close();
    heartbeat_slot.close();
    if supervisor.owner_mode() == crate::supervisor::OWNER_LEGACY {
        let mut inner = supervisor.inner().write().await;
        if !inner.detached {
            inner.mark_connection_closed(conn_id);
        }
    }

    // Stop and JOIN all primary tasks so none outlives teardown; give writer <= 1 s grace on fatal overflow.
    worker_task.abort();
    let _ = worker_task.await;
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
        // Take the owner lock FIRST, keep connection marked closed, and stop motion before
        // releasing whichever lease this connection currently holds.
        let mut inner = supervisor.inner().write().await;
        if !inner.detached {
            inner.mark_connection_closed(conn_id);
            let _ = inner.release_lease_for_connection_and_stop(&connection);
        }
        connection_generation.store(0, Ordering::SeqCst);
    }
}
