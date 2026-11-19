use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, LazyLock, Mutex};
use axum::{extract::{ws::{Message, WebSocket, WebSocketUpgrade}, State}, response::IntoResponse};
use futures_util::{SinkExt, StreamExt};
use serde_json::{json, Value};
use tokio::sync::mpsc;
use crate::supervisor::{ArmError, ArmSupervisor, SupervisorInner};
use crate::motion::{self, PlanInput};
use crate::safety::SafetyError;
use crate::api::rest::ApiError;
use axum::http::StatusCode;

pub type ConnectionRegistry = Arc<Mutex<HashMap<String, mpsc::Sender<Message>>>>;

static CONNECTION_REGISTRY: LazyLock<ConnectionRegistry> =
    LazyLock::new(|| Arc::new(Mutex::new(HashMap::new())));

pub fn connection_registry() -> ConnectionRegistry {
    CONNECTION_REGISTRY.clone()
}

static NEXT_CONNECTION: AtomicU64 = AtomicU64::new(1);

/// Test hook: pause (ms) between stamping a delayed command ack with the current epoch and
/// enqueueing it. 0 (no pause) outside tests.
pub static TEST_ACK_STAMP_DELAY_MS: AtomicU64 = AtomicU64::new(0);

async fn test_ack_stamp_delay() {
    let ms = TEST_ACK_STAMP_DELAY_MS.load(Ordering::Relaxed);
    if ms > 0 { tokio::time::sleep(std::time::Duration::from_millis(ms)).await; }
}

/// `?obs_hz=N`: send at most N observations per second on this connection (clamped to
/// 1..control_frequency_hz). Absent: every tick. Control replies are never decimated.
pub async fn ws_handler(ws: WebSocketUpgrade, State(supervisor): State<Arc<ArmSupervisor>>,
                        axum::extract::Query(query): axum::extract::Query<HashMap<String, String>>,
                        axum::Extension(deps): axum::Extension<crate::grasp::GraspDeps>,
                        axum::Extension(calib): axum::Extension<crate::calibration::CalibrationDeps>) -> axum::response::Response {
    let control_hz = supervisor.inner().read().await.config.control_frequency_hz.max(1);
    let obs_rate = match query.get("obs_hz") {
        None => None,
        Some(v) => match v.trim().parse::<u64>() {
            Ok(n) => Some(ObsRate { hz: n.clamp(1, u64::from(control_hz)) as f64, control_hz: f64::from(control_hz) }),
            Err(_) => return (StatusCode::BAD_REQUEST, "obs_hz must be an integer >= 0 (0 and 1 mean 1 Hz; values above control_frequency_hz are clamped)").into_response(),
        },
    };
    ws.on_upgrade(move |socket| handle_socket(socket, supervisor, deps, calib, obs_rate)).into_response()
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
    if let Some(d) = error.workspace_violation {
        if let Some(c) = d.constraint_id { v["constraint_id"] = json!(c); }
        if let Some(g) = d.geometry_id { v["geometry_id"] = json!(g); }
        if let Some(p) = d.violation_phase { v["violation_phase"] = json!(p); }
        if let Some(r) = d.reason { v["reason"] = json!(r); }
    }
    v
}

fn positions(value: Option<&Value>) -> Result<Vec<f32>, ApiError> {
    let values = value.and_then(Value::as_array).ok_or_else(|| invalid_command("Malformed positions array"))?;
    values.iter().map(|v| v.as_f64().map(|n| n as f32).filter(|n| n.is_finite())
        .ok_or_else(|| ApiError::from(ArmError::Safety(SafetyError::InvalidValue)))).collect()
}

/// Action started by a command: (action id, expected duration in s).
type Started = Option<(u64, f64)>;

// The caller holds the same write lock used to verify lease and epoch.
fn execute_command(inner: &mut SupervisorInner, path: &str, body: &Value, epoch: u64) -> Result<Started, ApiError> {
    if !body.is_object() { return Err(invalid_command("Command body must be an object")); }
    if let Some(duration) = body.get("duration_secs").filter(|v| !v.is_null()) {
        if duration.as_f64().is_none() { return Err(invalid_command("duration_secs must be numeric")); }
    }
    match path {
        "/api/v1/torque" | "/api/v1/arm" => {
            let enabled = body.get("enable").and_then(Value::as_bool)
                .ok_or_else(|| invalid_command("enable must be boolean"))?;
            inner.set_torque(enabled).map_err(ArmError::from)?;
        }
        "/api/v1/safety/clear_fault" => { inner.clear_fault().map_err(ArmError::from)?; }
        "/api/v1/motion/servo" => { inner.servo(positions(body.get("targets"))?, epoch).map_err(ArmError::from)?; }
        "/api/v1/motion/jog" => {
            let (_, id, duration) = inner.jog_action(positions(body.get("target_positions"))?, body.get("duration_secs").and_then(Value::as_f64), Some(epoch), "joint_move")?;
            return Ok(Some((id, duration)));
        }
        "/api/v1/motion/home" => {
            let (_, id, duration) = inner.jog_action(inner.config.home_positions.clone(), body.get("duration_secs").and_then(Value::as_f64), Some(epoch), "home")?;
            return Ok(Some((id, duration)));
        }
        "/api/v1/motion/unload" => {
            inner.gravity_comp_motion_busy().map_err(ArmError::from)?;
            inner.collision_hold_check().map_err(ArmError::from)?;
            if inner.handover_settling_since.is_some() {
                return Err(ArmError::Safety(SafetyError::HandoverSettling).into());
            }
            if !inner.supports_automatic_unload() {
                return Err(ArmError::Unsupported("Automatic unload requires a resting-pose profile and velocity, torque and enable-state feedback; no motion was started".into()).into());
            }
            if !inner.safety.state().is_operational() {
                return Err(ArmError::Safety(SafetyError::NotArmed(inner.safety.state().clone())).into());
            }
            let id = inner.unload_action()?;
            let duration = inner.active_trajectory.as_ref().map_or(0.0, |t| t.duration()) + 0.8;
            return Ok(id.map(|id| (id, duration)));
        }
        "/api/v1/motion/stop" => {
            inner.exit_gravity_comp("STOPPED", "Stop command");
            inner.stop_motion();
            inner.request_macro_stop();
        }
        "/api/v1/mode/gravity_comp" => {
            let enable = body.get("enable").and_then(Value::as_bool)
                .ok_or_else(|| invalid_command("enable must be boolean"))?;
            if enable {
                inner.enter_gravity_comp(gravity_comp_request(body)?)?;
            } else {
                inner.exit_gravity_comp("REQUESTED", "Exit requested");
            }
        }
        _ => return Err(invalid_command("Unknown actuator command")),
    }
    Ok(None)
}

/// `scale` (number) and `damping_kd` (number for all six arm joints, or six numbers).
fn gravity_comp_request(body: &Value) -> Result<crate::gravity_comp::GravityCompRequest, ApiError> {
    let scale = match body.get("scale") {
        None | Some(Value::Null) => None,
        Some(v) => Some(v.as_f64().filter(|x| x.is_finite()).ok_or_else(|| invalid_command("scale must be a number"))? as f32),
    };
    let damping_kd = match body.get("damping_kd") {
        None | Some(Value::Null) => None,
        Some(Value::Number(n)) => Some(vec![n.as_f64().unwrap_or(f64::NAN) as f32; 6]),
        Some(Value::Array(a)) => Some(a.iter().map(|v| v.as_f64().map_or(f32::NAN, |x| x as f32)).collect()),
        Some(_) => return Err(invalid_command("damping_kd must be a number or an array of 6 numbers")),
    };
    Ok(crate::gravity_comp::GravityCompRequest { scale, damping_kd })
}

/// A Cartesian action waiting for its plan (computed outside the lock).
struct PendingPlan {
    request_id: Value,
    kind: &'static str,
    input: PlanInput,
    goal: crate::kinematics::Pose,
    speed: crate::motion::Speed,
    tag: Option<String>,
    yaw_free: bool,
}

enum ActionStart { Started(u64, f64), Plan(PendingPlan), Macro(PendingMacro), Replay(crate::replay::ReplayInput), DryRunGrasp(Value) }

/// A macro action (crate::macro_action) checked under the lock, planned without it.
enum PendingMacro { Grasp(crate::grasp::PendingGrasp), Place(crate::place::PendingPlace) }

#[allow(clippy::large_enum_variant)]
enum PlannedMacro { Grasp(crate::grasp::PlannedGrasp), Place(crate::place::PlannedPlace) }

impl PendingMacro {
    fn plan(self) -> Result<PlannedMacro, ArmError> {
        match self {
            Self::Grasp(g) => crate::grasp::plan(g).map(PlannedMacro::Grasp),
            Self::Place(p) => crate::place::plan(p).map(PlannedMacro::Place),
        }
    }
}

impl PlannedMacro {
    /// Begin the action and spawn its orchestrator. Returns (action id, expected duration).
    fn start(self, inner: &mut SupervisorInner, supervisor: &Arc<ArmSupervisor>, deps: &crate::grasp::GraspDeps) -> Result<(u64, f64), ArmError> {
        match self {
            Self::Grasp(planned) => crate::grasp::begin(inner, supervisor, deps.source.clone(), planned),
            Self::Place(planned) => {
                let id = inner.begin_macro("place", &planned.input, planned.request.tag.clone())?;
                if planned.orientation_levelled {
                    inner.update_action(id, |a| {
                        a.orientation = Some("levelled".into());
                    });
                }
                let duration = crate::place::expected_duration(&planned);
                crate::place::spawn(supervisor.clone(), id, planned);
                Ok((id, duration))
            }
        }
    }
}

/// A macro action being planned and checked (grasp preflight: model switch, up to ~7 s) in a
/// task of its own, so the receive loop still reads stop / e-stop on the connection meanwhile.
struct MacroCheck {
    request_id: Value,
    generation: u64,
    task: tokio::task::JoinHandle<Result<PlannedMacro, ArmError>>,
}

/// Messages queued while a [`MacroCheck`] runs, at most; then the socket is not read until
/// the check ends.
const MACRO_CHECK_BACKLOG: usize = 64;

/// Handled while a [`MacroCheck`] runs; every other message waits for its reply, in order.
/// None of these can start motion; lease changes are caught by the re-check after the check.
fn runs_during_macro_check(kind: &str, value: &Value) -> bool {
    matches!(kind, "estop" | "e_stop" | "heartbeat" | "acquire_lease" | "release_lease")
        || (kind == "command" && value.get("path").and_then(Value::as_str) == Some("/api/v1/motion/stop"))
}

/// The reply to a message cancelled by a stop / e-stop before it ran (`code` STOPPED /
/// ESTOP_ACTIVE). None for messages that are not acknowledged.
fn cancelled_reply(value: &Value, code: &str, epoch: u64) -> Option<Value> {
    let kind = value.get("type").and_then(Value::as_str).unwrap_or("");
    let mut reply = json!({"status":409, "code":code, "error":code, "message":"Cancelled by a stop before it began", "epoch_id":epoch});
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

// Under the write lock, after lease / epoch / request_id checks.
fn prepare_action(inner: &mut SupervisorInner, path: &str, body: &Value, epoch: u64, request_id: &Value) -> Result<ActionStart, ApiError> {
    if !body.is_object() { return Err(invalid_command("Command body must be an object")); }
    let (kind, parsed) = match path {
        "/api/v1/action/move_to" => ("move_to", motion::parse_move_to(body)),
        "/api/v1/action/move_relative" => ("move_relative", motion::parse_move_relative(body)),
        "/api/v1/action/gripper" => {
            let request = motion::parse_gripper(body).map_err(ArmError::from)?;
            let (id, duration) = inner.start_gripper(request, epoch)?;
            return Ok(ActionStart::Started(id, duration));
        }
        "/api/v1/action/gripper_zero_check" => {
            let (id, duration) = inner.start_gripper_zero_check(body, epoch)?;
            return Ok(ActionStart::Started(id, duration));
        }
        "/api/v1/action/grasp" => {
            if body.get("dry_run").and_then(Value::as_bool) == Some(true) {
                return Ok(ActionStart::DryRunGrasp(body.clone()));
            }
            return Ok(ActionStart::Macro(PendingMacro::Grasp(crate::grasp::prepare(inner, body, epoch)?)));
        }
        "/api/v1/action/place" => return Ok(ActionStart::Macro(PendingMacro::Place(crate::place::prepare(inner, body, epoch)?))),
        "/api/v1/action/replay" => {
            let request = crate::replay::parse_replay(body).map_err(ArmError::from)?;
            return Ok(ActionStart::Replay(inner.replay_input(request, epoch)?));
        }
        "/api/v1/action/move_to_pose" => {
            let (id, duration) = crate::poses::start_action(inner, body, epoch)?;
            return Ok(ActionStart::Started(id, duration));
        }
        _ => return Err(invalid_command("Unknown action")),
    };
    let (spec, speed, tag) = parsed.map_err(ArmError::from)?;
    let input = inner.plan_input()?;
    let (goal, yaw_free) = motion::resolve_goal_ex(&input, &spec);
    Ok(ActionStart::Plan(PendingPlan { request_id: request_id.clone(), kind, input, goal, speed, tag, yaw_free }))
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

/// Ack fields and the state-file write of a state-changing command.
type StateReply = (Vec<(&'static str, Value)>, Option<crate::persist::PendingWrite>);

/// Commands that change persisted state (grasp setup, poses, collision setting, keep-out
/// boxes). None for any other path.
fn state_command(inner: &mut SupervisorInner, path: &str, body: &Value, setup_path: Option<&std::path::PathBuf>) -> Option<Result<StateReply, ArmError>> {
    Some(match path {
        "/api/v1/grasp/setup" => crate::grasp::update_setup(inner, body).map(|setup| {
            let write = setup_path.cloned().map(|p| crate::persist::PendingWrite::new(p, serde_json::to_string_pretty(&setup).unwrap_or_default()));
            (vec![("setup", json!(setup))], write)
        }),
        "/api/v1/poses/save" => crate::poses::save(inner, body).map(|(pose, write)| (vec![("pose", json!(pose))], write)),
        "/api/v1/poses/delete" => crate::poses::delete(inner, body).map(|write| (vec![], write)),
        "/api/v1/safety/collision" => crate::collision::update(inner, body).map(|(config, write)| (vec![("collision", json!(config))], write)),
        "/api/v1/workspace/keepout" => crate::keepout::update(inner, body)
            .map(|(state, write)| (vec![("boxes", json!(state.boxes)), ("source", json!(state.source))], write)),
        "/api/v1/gripper/calibration" => crate::gripper_calib::update(inner, body).map(|(policy, write)| {
            (vec![
                ("policy", json!(policy)),
                ("closed_rad", json!(policy.closed_rad)),
                ("open_rad", json!(policy.open_rad)),
                ("jaw_opening", json!(policy.jaw_opening)),
            ], write)
        }),
        _ => return None,
    })
}

/// A calibration command checked under the lock.
enum CalibJob { Apply(Box<crate::calibration::ApplyRequest>), Restore(Value) }

/// Paths refused while a calibration transaction runs: motion (MOTION_BUSY) and configuration
/// writes (CONFIG_BUSY). Stop, torque, clear_fault and everything else go through.
fn calibration_busy_code(inner: &SupervisorInner, path: &str) -> Option<&'static str> {
    if !inner.calibration_busy || path == "/api/v1/motion/stop" { return None; }
    if path.starts_with("/api/v1/action/") || path.starts_with("/api/v1/motion/") || path.starts_with("/api/v1/mode/") { return Some("MOTION_BUSY"); }
    if path.starts_with("/api/v1/calibration/") || matches!(path, "/api/v1/grasp/setup" | "/api/v1/poses/save" | "/api/v1/poses/delete"
        | "/api/v1/safety/collision" | "/api/v1/workspace/keepout" | "/api/v1/gripper/calibration") { return Some("CONFIG_BUSY"); }
    None
}

/// Under the lock: parse and check a calibration command; a real apply / restore marks the
/// transaction running (`calibration_busy`, cleared by its task). Returns (job, busy set).
fn calibration_job(inner: &mut SupervisorInner, path: &str, body: &Value) -> Result<(CalibJob, bool), crate::calibration::CalibError> {
    crate::calibration::begin_checks(inner)?;
    let job = if path == "/api/v1/calibration/apply" {
        CalibJob::Apply(Box::new(crate::calibration::parse_apply(body)?))
    } else {
        if !body.is_object() { return Err(crate::calibration::CalibError::new(400, "INVALID_COMMAND", "Command body must be an object")); }
        CalibJob::Restore(body.clone())
    };
    let busy = !matches!(&job, CalibJob::Apply(r) if r.dry_run);
    if busy { inner.calibration_busy = true; }
    Ok((job, busy))
}

fn calibration_reject(request_id: &Value, e: &crate::calibration::CalibError) -> Value {
    let mut reply = json!({"type":"command_reject", "request_id":request_id});
    for (k, v) in e.reply_fields() { reply[k] = v; }
    reply
}

fn command_reject(inner: &SupervisorInner, request_id: &Value, error: ApiError) -> Value {
    let mut reply = rejection(error, "command_reject", inner.safety.epoch_id());
    reply["request_id"] = request_id.clone();
    reply
}

async fn handle_socket(socket: WebSocket, supervisor: Arc<ArmSupervisor>, deps: crate::grasp::GraspDeps, calib: crate::calibration::CalibrationDeps, obs_rate: Option<ObsRate>) {
    let (mut sender, mut receiver) = socket.split();
    let mut telemetry = supervisor.subscribe_telemetry();
    let (out_tx, mut out_rx) = tokio::sync::mpsc::channel::<Message>(64);
    // This identity never comes from a client payload and is never disclosed.
    let connection = format!("connection:{}", NEXT_CONNECTION.fetch_add(1, Ordering::Relaxed));
    let connection_generation = Arc::new(AtomicU64::new(0));

    {
        let mut reg = CONNECTION_REGISTRY.lock().unwrap_or_else(|e| e.into_inner());
        reg.insert(connection.clone(), out_tx.clone());
    }

    struct ConnectionGuard(String);
    impl Drop for ConnectionGuard {
        fn drop(&mut self) {
            let mut reg = CONNECTION_REGISTRY.lock().unwrap_or_else(|e| e.into_inner());
            reg.remove(&self.0);
        }
    }
    let _conn_guard = ConnectionGuard(connection.clone());

    let mut send_task = tokio::spawn(async move {
        let mut last_sent: Option<f64> = None;
        loop {
            tokio::select! {
                biased;
                message = out_rx.recv() => match message {
                    Some(message) => if sender.send(message).await.is_err() { break; },
                    None => break,
                },
                observation = telemetry.recv() => match observation {
                    Ok(obs) => {
                        if let Some(rate) = obs_rate {
                            if !rate.due(last_sent, obs.timestamp) { continue; }
                            last_sent = Some(obs.timestamp);
                        }
                        if let Ok(text) = serde_json::to_string(&obs) {
                            if sender.send(Message::Text(text.into())).await.is_err() { break; }
                        }
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => {},
                    Err(_) => break,
                }
            }
        }
    });
    let recv_supervisor = supervisor.clone();
    let recv_connection = connection.clone();
    let recv_generation = connection_generation.clone();
    let recv_deps = deps;
    let recv_calib = calib;
    let mut recv_task = tokio::spawn(async move {
        let mut client_label: Option<String> = None;
        let mut last_sequence: Option<u64> = None;
        let mut last_request: Option<u64> = None;
        // Delayed acks (sent after a state-file write) come back here and are stamped with
        // the current epoch and enqueued by this loop, in order with its other replies
        // (epoch_sync, rejects): the epochs a client receives never go back.
        let (ack_tx, mut ack_rx) = mpsc::channel::<Value>(64);
        let mut macro_check: Option<MacroCheck> = None;
        let mut backlog: std::collections::VecDeque<Message> = std::collections::VecDeque::new();
        loop {
            let message = match backlog.pop_front() {
                Some(queued) if macro_check.is_none() => queued,
                queued => {
                    if let Some(queued) = queued { backlog.push_front(queued); }
                    tokio::select! {
                        biased;
                        Some(mut reply) = ack_rx.recv() => {
                            {
                                let inner = recv_supervisor.inner().read().await;
                                reply["epoch_id"] = json!(inner.safety.epoch_id());
                                reply["safety_state"] = json!(inner.safety.state());
                                reply["is_torqued"] = json!(inner.is_torqued);
                            }
                            test_ack_stamp_delay().await;
                            if out_tx.send(Message::Text(reply.to_string().into())).await.is_err() { break; }
                            continue;
                        }
                        joined = async { (&mut macro_check.as_mut().expect("guarded by the precondition").task).await }, if macro_check.is_some() => {
                            let MacroCheck { request_id, generation, .. } = macro_check.take().expect("checked above");
                            let mut inner = recv_supervisor.inner().write().await;
                            let reply = match joined {
                                Err(e) => command_reject(&inner, &request_id, ArmError::Internal(format!("planner task failed: {e}")).into()),
                                Ok(Err(error)) => command_reject(&inner, &request_id, error.into()),
                                Ok(Ok(planned)) => {
                                    if recv_generation.load(Ordering::SeqCst) != generation || !inner.validate_servo_lease(&recv_connection, generation) {
                                        json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"LEASE_LOST", "message":"This connection lost the lease while planning", "error":"LEASE_LOST", "epoch_id":inner.safety.epoch_id()})
                                    } else {
                                        match planned.start(&mut inner, &recv_supervisor, &recv_deps) {
                                            Ok(started) => command_ack(&inner, &request_id, Some(started)),
                                            Err(error) => command_reject(&inner, &request_id, error.into()),
                                        }
                                    }
                                }
                            };
                            drop(inner);
                            if out_tx.send(Message::Text(reply.to_string().into())).await.is_err() { break; }
                            continue;
                        }
                        message = receiver.next(), if backlog.len() < MACRO_CHECK_BACKLOG => match message {
                            Some(Ok(message)) => message,
                            _ => break,
                        },
                    }
                }
            };
            let Message::Text(text) = message else {
                if matches!(message, Message::Close(_)) { break; }
                continue;
            };
            let Ok(value) = serde_json::from_str::<Value>(&text) else { continue; };
            let kind = value.get("type").and_then(Value::as_str).unwrap_or("");
            if macro_check.is_some() && !runs_during_macro_check(kind, &value) {
                backlog.push_back(Message::Text(text));
                continue;
            }
            let mut response: Option<Value>;
            let mut evicted_notify: Option<(String, u64)> = None;
            let mut pending_plan: Option<(PendingPlan, u64)> = None;
            let mut pending_macro: Option<(Value, PendingMacro, u64)> = None;
            let mut pending_replay: Option<(Value, crate::replay::ReplayInput, u64)> = None;
            let mut pending_dry_run_grasp: Option<(Value, Value)> = None;
            let mut pending_write: Option<crate::persist::PendingWrite> = None;
            let mut pending_calibration: Option<(Value, CalibJob, u64, u64, bool)> = None;
            if kind == "estop" || kind == "e_stop" {
                let (state, epoch) = recv_supervisor.trigger_e_stop().await;
                response = Some(json!({"type":"epoch_sync", "epoch_id":epoch, "safety_state":state}));
            } else {
                {
                    let mut inner = recv_supervisor.inner().write().await;
                    if kind == "acquire_lease" {
                        let label = value.get("client_id").and_then(Value::as_str).unwrap_or("");
                        let seq = value.get("seq").and_then(Value::as_u64);
                        let force_val = value.get("force");

                        if label.is_empty() || label.len() > 128 {
                            let mut reply = json!({"type":"lease_rejected", "error":"invalid_client_id"});
                            if let Some(s) = seq { reply["seq"] = json!(s); }
                            response = Some(reply);
                        } else {
                            match force_val {
                                Some(v) if !v.is_boolean() && !v.is_null() => {
                                    let mut reply = json!({
                                        "type": "lease_rejected",
                                        "status": 400,
                                        "error": "invalid_argument",
                                        "message": "force must be a boolean"
                                    });
                                    if let Some(s) = seq { reply["seq"] = json!(s); }
                                    response = Some(reply);
                                }
                                _ => {
                                    let force = force_val.and_then(Value::as_bool).unwrap_or(false);
                                    if force {
                                        match inner.preempt_lease(&recv_connection, label) {
                                            Ok((generation, evicted)) => {
                                                let previous = recv_generation.swap(generation, Ordering::SeqCst);
                                                client_label = Some(label.to_owned());
                                                if previous != generation { last_sequence = None; }
                                                let mut reply = json!({
                                                    "type": "lease_granted",
                                                    "client_id": label,
                                                    "generation": generation
                                                });
                                                if let Some(s) = seq { reply["seq"] = json!(s); }
                                                response = Some(reply);
                                                evicted_notify = evicted;
                                            }
                                            Err(crate::supervisor::PreemptError::Cooldown(remaining)) => {
                                                let mut reply = json!({
                                                    "type": "lease_rejected",
                                                    "error": "preemption_cooldown",
                                                    "remaining_ms": remaining
                                                });
                                                if let Some(s) = seq { reply["seq"] = json!(s); }
                                                response = Some(reply);
                                            }
                                        }
                                    } else {
                                        match inner.acquire_lease(&recv_connection) {
                                            Ok(generation) => {
                                                inner.set_lease_label(&recv_connection, generation, label);
                                                let previous = recv_generation.swap(generation, Ordering::SeqCst);
                                                client_label = Some(label.to_owned());
                                                if previous != generation { last_sequence = None; }
                                                let mut reply = json!({
                                                    "type": "lease_granted",
                                                    "client_id": label,
                                                    "generation": generation
                                                });
                                                if let Some(s) = seq { reply["seq"] = json!(s); }
                                                response = Some(reply);
                                            }
                                            Err(_) => {
                                                let mut reply = json!({"type":"lease_rejected", "error":"controller_busy"});
                                                if let Some(s) = seq { reply["seq"] = json!(s); }
                                                response = Some(reply);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    } else {
                        let local_generation = recv_generation.load(Ordering::SeqCst);
                        let generation = value.get("generation").and_then(Value::as_u64).unwrap_or(0);
                        let label_matches = value.get("client_id").is_none_or(|id| id.as_str() == client_label.as_deref());
                        // Heartbeat from legacy clients may omit generation; connection still must own it.
                        let supplied_generation_ok = generation == local_generation || (kind == "heartbeat" && generation == 0);
                        let authorized = local_generation != 0 && supplied_generation_ok && label_matches
                            && inner.validate_servo_lease(&recv_connection, local_generation);
                        let is_dry_run = kind == "command" && value.get("path").and_then(Value::as_str) == Some("/api/v1/action/grasp") && value.get("body").and_then(|b| b.get("dry_run")).and_then(Value::as_bool) == Some(true);
                        if (kind == "servo" || value.get("mode").and_then(Value::as_str) == Some("cartesian_joint_stream")) && value.get("generation").and_then(Value::as_u64).is_none() {
                            response = Some(json!({"type":"servo_reject", "reason":"missing_generation"}));
                        } else if !authorized && !is_dry_run {
                            response = Some(if kind == "command" {
                                json!({"type":"command_reject", "request_id":value.get("request_id"), "status":409, "code":"LEASE_LOST", "message":"This connection does not own an active lease", "error":"LEASE_LOST", "epoch_id":inner.safety.epoch_id()})
                            } else if kind == "release_lease" {
                                json!({"type":"lease_released", "success":false})
                            } else if local_generation == 0 {
                                json!({"type":"lease_lost", "generation":0})
                            } else {
                                json!({"type":"lease_lost", "generation":local_generation, "reason":inner.lease_loss_reason(&recv_connection, local_generation)})
                            });
                        } else if kind == "release_lease" {
                            let released = inner.release_lease(&recv_connection, local_generation);
                            recv_generation.store(0, Ordering::SeqCst);
                            client_label = None;
                            inner.stop_motion();
                            response = Some(json!({"type":"lease_released", "success":released}));
                        } else {
                            let epoch = inner.safety.epoch_id();
                            if !is_dry_run && value.get("epoch_id").and_then(Value::as_u64) != Some(epoch) {
                                response = Some(if kind == "command" {
                                    json!({"type":"command_reject", "request_id":value.get("request_id"), "status":409, "code":"EPOCH_MISMATCH", "message":"Command epoch is no longer current", "error":"EPOCH_MISMATCH", "epoch_id":epoch})
                                } else { json!({"type":"epoch_sync", "epoch_id":epoch, "safety_state":inner.safety.state()}) });
                            } else if kind == "heartbeat" {
                                let _ = inner.heartbeat(&recv_connection, epoch);
                                response = None;
                            } else if kind == "command" {
                                let request_id = value.get("request_id").and_then(Value::as_u64);
                                if request_id.is_none() || last_request.is_some_and(|last| request_id.unwrap() <= last) {
                                    response = Some(json!({"type":"command_reject", "request_id":value.get("request_id"), "status":400, "code":"INVALID_COMMAND", "message":"request_id must be present and strictly increase", "error":"INVALID_COMMAND"}));
                                } else {
                                    last_request = request_id;
                                    let path = value.get("path").and_then(Value::as_str).unwrap_or("");
                                    if let Some(code) = calibration_busy_code(&inner, path) {
                                        response = Some(command_reject(&inner, &value["request_id"],
                                            ApiError(StatusCode::CONFLICT, "a camera calibration apply or restore is running".into(), code.into())));
                                    } else if path == "/api/v1/calibration/apply" || path == "/api/v1/calibration/restore" {
                                        response = match calibration_job(&mut inner, path, &value["body"]) {
                                            Ok((job, busy)) => { pending_calibration = Some((value["request_id"].clone(), job, local_generation, epoch, busy)); None }
                                            Err(e) => Some(calibration_reject(&value["request_id"], &e)),
                                        };
                                    } else if path.starts_with("/api/v1/action/") {
                                        response = match prepare_action(&mut inner, path, &value["body"], epoch, &value["request_id"]) {
                                            Ok(ActionStart::Started(id, duration)) => Some(command_ack(&inner, &value["request_id"], Some((id, duration)))),
                                            Ok(ActionStart::Plan(plan)) => { pending_plan = Some((plan, local_generation)); None }
                                            Ok(ActionStart::Macro(m)) => { pending_macro = Some((value["request_id"].clone(), m, local_generation)); None }
                                            Ok(ActionStart::Replay(input)) => { pending_replay = Some((value["request_id"].clone(), input, local_generation)); None }
                                            Ok(ActionStart::DryRunGrasp(body)) => { pending_dry_run_grasp = Some((value["request_id"].clone(), body)); None }
                                            Err(error) => Some(command_reject(&inner, &value["request_id"], error)),
                                        };
                                    } else if let Some(result) = state_command(&mut inner, path, &value["body"], recv_deps.setup_path.as_ref()) {
                                        response = Some(match result {
                                            Ok((fields, write)) => {
                                                let mut ack = command_ack(&inner, &value["request_id"], None);
                                                for (key, field) in fields { ack[key] = field; }
                                                pending_write = write;
                                                ack
                                            }
                                            Err(error) => command_reject(&inner, &value["request_id"], error.into()),
                                        });
                                    } else {
                                        let result = execute_command(&mut inner, path, &value["body"], epoch);
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
                                    _ => ("/api/v1/motion/jog", json!({"target_positions":value.get("positions")})),
                                };
                                response = Some(match execute_command(&mut inner, path, &body, epoch) {
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
                                        response = match positions(value.get("positions")).and_then(|targets| inner.servo(targets, epoch).map_err(|error| ApiError::from(ArmError::from(error)))) {
                                            Ok(()) => None,
                                            Err(error) => Some(rejection(error, "servo_reject", epoch)),
                                        };
                                    }
                                }
                            } else { response = None; }
                        }
                    }
                }

                // Cartesian plan: IK runs on a blocking thread without the supervisor lock,
                // so the control tick is never blocked by it; install re-checks everything.
                if let Some((plan, generation)) = pending_plan.take() {
                    let PendingPlan { request_id, kind, input, goal, speed, tag, yaw_free } = plan;
                    let planned = tokio::task::spawn_blocking(move || {
                        let result = motion::plan_cartesian_ex(&input, goal, speed, yaw_free);
                        (input, result)
                    }).await;
                    let mut inner = recv_supervisor.inner().write().await;
                    if let Some(hook) = inner.before_install_hook.take() { hook(&mut inner); }
                    response = Some(match planned {
                        Err(e) => command_reject(&inner, &request_id, ArmError::Internal(format!("planner task failed: {e}")).into()),
                        Ok((_, Err(error))) => command_reject(&inner, &request_id, ArmError::from(error).into()),
                        Ok((input, Ok(path))) => {
                            if recv_generation.load(Ordering::SeqCst) != generation || !inner.validate_servo_lease(&recv_connection, generation) {
                                json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"LEASE_LOST", "message":"This connection lost the lease while planning", "error":"LEASE_LOST", "epoch_id":inner.safety.epoch_id()})
                            } else {
                                match inner.install_path(&input, path, kind, tag) {
                                    Ok(started) => command_ack(&inner, &request_id, Some(started)),
                                    Err(error) => command_reject(&inner, &request_id, error.into()),
                                }
                            }
                        }
                    });
                }

                // Macro actions (grasp, place): the first motions are planned without the lock
                // (feasibility check), then the action begins and the orchestrator task takes over.
                // The check runs in its own task; its result is taken by the select! above
                // (lease re-check, then begin), and later messages wait in `backlog`.
                if let Some((request_id, pending, generation)) = pending_macro.take() {
                    let source = recv_deps.source.clone();
                    let task = tokio::spawn(async move {
                        let mut planned = tokio::task::spawn_blocking(move || pending.plan()).await
                            .map_err(|e| ArmError::Internal(format!("planner task failed: {e}")))??;
                        // grasp task requirements: checked without the lock, before the action begins
                        if let PlannedMacro::Grasp(g) = &mut planned { crate::grasp::preflight(source.as_ref(), g).await?; }
                        Ok(planned)
                    });
                    macro_check = Some(MacroCheck { request_id, generation, task });
                }
                // Replay: the episode is read and checked without the lock; install re-checks.
                if let Some((request_id, input, generation)) = pending_replay.take() {
                    let planned = tokio::task::spawn_blocking(move || crate::replay::plan(input)).await;
                    let mut inner = recv_supervisor.inner().write().await;
                    response = Some(match planned {
                        Err(e) => command_reject(&inner, &request_id, ArmError::Internal(format!("replay check task failed: {e}")).into()),
                        Ok(Err(error)) => command_reject(&inner, &request_id, error.into()),
                        Ok(Ok(plan)) => {
                            if recv_generation.load(Ordering::SeqCst) != generation || !inner.validate_servo_lease(&recv_connection, generation) {
                                json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"LEASE_LOST", "message":"This connection lost the lease while the episode was checked", "error":"LEASE_LOST", "epoch_id":inner.safety.epoch_id()})
                            } else {
                                match inner.install_replay(plan) {
                                    Ok(started) => command_ack(&inner, &request_id, Some(started)),
                                    Err(error) => command_reject(&inner, &request_id, error.into()),
                                }
                            }
                        }
                    });
                }
                if let Some((request_id, body)) = pending_dry_run_grasp.take() {
                    let dry = crate::grasp::dry_run_task(&recv_supervisor, recv_deps.source.as_ref(), &body).await;
                    let inner = recv_supervisor.inner().read().await;
                    let mut ack = command_ack(&inner, &request_id, None);
                    for (k, v) in dry.fields() { ack[k.as_str()] = v; }
                    response = Some(ack);
                }
                // Calibration apply / restore: its own task (HTTP to rebot-vision, file writes);
                // this loop goes on (heartbeats, stop) and the reply comes back through ack_tx.
                if let Some((request_id, job, generation, epoch, busy)) = pending_calibration.take() {
                    let (sup, calib, conn, gen_now, tx) = (recv_supervisor.clone(), recv_calib.clone(), recv_connection.clone(), recv_generation.clone(), ack_tx.clone());
                    tokio::spawn(async move {
                        let check: crate::calibration::LeaseCheck = Box::new(move |inner: &mut SupervisorInner| {
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
                        let job_task = tokio::spawn(async move {
                            match job {
                                CalibJob::Apply(req) => crate::calibration::apply(&job_sup, &calib, *req, &check).await,
                                CalibJob::Restore(body) => crate::calibration::restore(&job_sup, &calib, &body, &check).await,
                            }
                        });
                        let result = job_task.await.unwrap_or_else(|e| {
                            tracing::error!("calibration task failed: {e}");
                            Err(crate::calibration::CalibError::new(500, "INTERNAL", format!("calibration task failed: {e}; check GET /api/v1/calibration and the backups")))
                        });
                        if let Some(g) = guard { g.release().await; }
                        let reply = match result {
                            Ok(Value::Object(fields)) => {
                                let mut ack = json!({"type":"command_ack", "request_id":request_id, "status":"accepted"});
                                for (k, v) in fields { ack[k] = v; }
                                ack
                            }
                            Ok(other) => json!({"type":"command_ack", "request_id":request_id, "status":"accepted", "result":other}),
                            Err(e) => calibration_reject(&request_id, &e),
                        };
                        let _ = tx.send(reply).await;
                    });
                }
                // State files are written by the single persist writer; this loop does not wait
                // for the disk: the ack (with `persisted`) is sent when the write finished.
                if let Some(write) = pending_write.take() {
                    let reply = response.take();
                    let tx = ack_tx.clone();
                    tokio::spawn(async move {
                        let result = write.run().await;
                        let Some(mut reply) = reply else { return; };
                        match result {
                            Ok(()) => reply["persisted"] = json!(true),
                            Err(e) => { tracing::warn!("State file not saved: {e}"); reply["persisted"] = json!(false); reply["persist_error"] = json!(e); }
                        }
                        // the ack can follow later replies: the receive loop stamps it with the
                        // state at send time, so a client never rolls its epoch back
                        let _ = tx.send(reply).await;
                    });
                }
                if let Some((evicted_conn, evicted_gen)) = evicted_notify {
                    let sender_opt = {
                        let reg = CONNECTION_REGISTRY.lock().unwrap_or_else(|e| e.into_inner());
                        reg.get(&evicted_conn).cloned()
                    };
                    if let Some(sender) = sender_opt {
                        let lost = json!({
                            "type": "lease_lost",
                            "generation": evicted_gen,
                            "reason": "preempted"
                        });
                        if let Err(e) = sender.try_send(Message::Text(lost.to_string().into())) {
                            tracing::warn!("Failed to deliver eviction notice to {}: {}", evicted_conn, e);
                        }
                    }
                }
            }
            // A stop / e-stop during a macro check cancels it and every message queued behind
            // it: none of them begins after the stop.
            let stop_code = if kind == "estop" || kind == "e_stop" {
                Some("ESTOP_ACTIVE")
            } else if kind == "command" && value.get("path").and_then(Value::as_str) == Some("/api/v1/motion/stop")
                && response.as_ref().is_some_and(|r| r["type"] == "command_ack") {
                Some("STOPPED")
            } else { None };
            if let (Some(code), Some(check)) = (stop_code, macro_check.as_ref()) {
                check.task.abort();
                let epoch = recv_supervisor.inner().read().await.safety.epoch_id();
                let mut cancelled = vec![json!({"type":"command_reject", "request_id":check.request_id, "status":409, "code":code, "error":code, "message":"Stopped before the action began", "epoch_id":epoch})];
                macro_check = None;
                cancelled.extend(backlog.drain(..).filter_map(|queued| match queued {
                    Message::Text(t) => serde_json::from_str::<Value>(&t).ok().and_then(|v| cancelled_reply(&v, code, epoch)),
                    _ => None,
                }));
                let mut closed = false;
                for reply in cancelled {
                    if out_tx.send(Message::Text(reply.to_string().into())).await.is_err() { closed = true; break; }
                }
                if closed { break; }
            }
            // Do not silently lose control acknowledgements behind telemetry.
            if let Some(response) = response {
                if out_tx.send(Message::Text(response.to_string().into())).await.is_err() { break; }
            }
        }
    });
    tokio::select! {
        _ = &mut send_task => recv_task.abort(),
        _ = &mut recv_task => send_task.abort(),
    }
    let generation = connection_generation.load(Ordering::SeqCst);
    let mut inner = supervisor.inner().write().await;
    if generation != 0 && inner.release_lease(&connection, generation) {
        inner.stop_motion();
    }
}
