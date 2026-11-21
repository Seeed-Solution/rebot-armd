use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
#[cfg(feature = "policy-runtime")]
use std::sync::atomic::{AtomicBool, AtomicUsize};
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
                        headers: axum::http::HeaderMap,
                        axum::Extension(deps): axum::Extension<crate::grasp::GraspDeps>,
                        axum::Extension(calib): axum::Extension<crate::calibration::CalibrationDeps>,
                        axum::Extension(sandbox): axum::Extension<Arc<crate::api::sandbox::SandboxState>>,
                        axum::Extension(auth): axum::Extension<crate::auth::OperatorAuth>) -> axum::response::Response {
    let control_hz = supervisor.inner().read().await.config.control_frequency_hz.max(1);
    let obs_rate = match query.get("obs_hz") {
        None => None,
        Some(v) => match v.trim().parse::<u64>() {
            Ok(n) => Some(ObsRate { hz: n.clamp(1, u64::from(control_hz)) as f64, control_hz: f64::from(control_hz) }),
            Err(_) => return (StatusCode::BAD_REQUEST, "obs_hz must be an integer >= 0 (0 and 1 mean 1 Hz; values above control_frequency_hz are clamped)").into_response(),
        },
    };
    let ws_token = crate::auth::OperatorAuth::extract_token(&headers);
    ws.on_upgrade(move |socket| handle_socket(socket, supervisor, deps, calib, sandbox, obs_rate, auth, ws_token)).into_response()
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
            if !inner.safety.state().is_operational() && !inner.holding_after_clear() {
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
    opts: crate::motion::PathOpts,
    /// Same-side approach (`precise: true` / `approach`, src/backlash.rs): settings or the
    /// reason for a direct motion (None = off).
    approach: Result<crate::backlash::Plan, Option<&'static str>>,
}

#[cfg(feature = "policy-runtime")]
struct PendingPolicyStart {
    request: crate::policy_runtime::PolicyStartRequest,
    config: crate::model::ArmConfig,
    epoch: u64,
    policy_live_allowed: bool,
}

#[cfg(feature = "policy-runtime")]
struct PolicyLoadingGuard(Option<Arc<ArmSupervisor>>);

#[cfg(feature = "policy-runtime")]
impl PolicyLoadingGuard {
    fn new(sup: Arc<ArmSupervisor>) -> Self {
        Self(Some(sup))
    }

    fn clear(&mut self, inner: &mut SupervisorInner) {
        if self.0.take().is_some() {
            inner.policy_loading = false;
        }
    }
}

#[cfg(feature = "policy-runtime")]
impl Drop for PolicyLoadingGuard {
    fn drop(&mut self) {
        let Some(sup) = self.0.take() else {
            return;
        };
        if let Ok(mut inner) = sup.inner().try_write() {
            inner.policy_loading = false;
            return;
        }
        match tokio::runtime::Handle::try_current() {
            Ok(h) => {
                h.spawn(async move {
                    sup.inner().write().await.policy_loading = false;
                });
            }
            Err(_) => sup.inner().blocking_write().policy_loading = false,
        }
        tracing::warn!("policy loading ended without explicit clear; cleared policy_loading");
    }
}

enum ActionStart {
    Started(u64, f64),
    Plan(PendingPlan),
    Macro(PendingMacro),
    Replay(crate::replay::ReplayInput),
    DryRunGrasp(Value),
    Sequence(Box<crate::sequences::RunPlan>),
    CalibHover(Box<crate::calib_hover::HoverPlan>),
    #[cfg(feature = "policy-runtime")]
    Policy(Box<PendingPolicyStart>),
    #[cfg(feature = "policy-runtime")]
    PolicyStopped(bool),
}

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
                let duration = crate::place::expected_duration(&planned) + inner.approach_estimate_s("place_final", planned.request.approach);
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
    matches!(kind, "estop" | "e_stop" | "heartbeat" | "acquire_lease" | "release_lease" | "auth")
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
fn prepare_action(
    inner: &mut SupervisorInner,
    path: &str,
    body: &Value,
    epoch: u64,
    request_id: &Value,
    auth: &crate::auth::OperatorAuth,
    ws_token: Option<&str>,
) -> Result<ActionStart, ApiError> {
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
            let (id, duration) = inner.start_gripper_zero_check_with_auth(
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
        "/api/v1/action/run_sequence" => return Ok(ActionStart::Sequence(Box::new(crate::sequences::prepare_run(inner, body, epoch)?))),
        crate::calib_hover::PATH => return Ok(ActionStart::CalibHover(Box::new(crate::calib_hover::prepare(inner, body, epoch)?))),
        #[cfg(feature = "policy-runtime")]
        "/api/v1/action/policy/start" => {
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
                policy_live_allowed: inner.policy_live_allowed(),
            })));
        }
        #[cfg(feature = "policy-runtime")]
        "/api/v1/action/policy/stop" => {
            let stopped = inner.stop_policy_run(None, "STOPPED");
            return Ok(ActionStart::PolicyStopped(stopped));
        }
        _ => return Err(invalid_command("Unknown action")),
    };
    let (spec, speed, tag) = parsed.map_err(ArmError::from)?;
    // move_to / move_relative keep their straight line to the goal; `precise: true` (profile
    // `applies_to` move_to_precise) or `approach: "same_side"` end it with the short segment
    let precise = motion::flag(body, "precise", false).map_err(ArmError::from)?;
    let request = crate::backlash::parse_request(body).map_err(ArmError::from)?;
    let approach = if precise || request != crate::backlash::Request::Profile { inner.approach_plan("move_to_precise", request) } else { Err(None) };
    let input = inner.plan_input()?;
    let (goal, opts) = motion::resolve_goal_opts(&input, &spec);
    Ok(ActionStart::Plan(PendingPlan { request_id: request_id.clone(), kind, input, goal, speed, tag, opts, approach }))
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

/// Atomic persist-before-commit config write (Step 1 prepared under lock, Step 2 staged to
/// disk without lock under `config_write_lock`, Step 3 prechecked + published + committed under lock).
enum PreparedConfigWrite {
    Keepout(crate::keepout::PreparedKeepout),
    Collision(crate::collision::PreparedCollision),
    GripperCalib(Box<crate::gripper_calib::PreparedGripperCalib>),
    PosesSave(crate::poses::PreparedPosesSave),
    PosesDelete(crate::poses::PreparedPosesDelete),
    PosesRename(crate::poses::PreparedPosesSave),
    Sequences(crate::sequences::PreparedSequences),
    GraspSetup(Box<crate::grasp::PreparedGraspSetup>),
}

impl PreparedConfigWrite {
    fn prepare(
        inner: &mut SupervisorInner,
        path: &str,
        body: &Value,
        setup_path: Option<&std::path::PathBuf>,
        require_expected_revision: bool,
    ) -> Option<Result<Self, ArmError>> {
        Some(match path {
            "/api/v1/workspace/keepout" => crate::keepout::prepare_update_with(inner, body, require_expected_revision).map(Self::Keepout),
            "/api/v1/safety/collision" => crate::collision::prepare_update_with(inner, body, require_expected_revision).map(Self::Collision),
            "/api/v1/gripper/calibration" => crate::gripper_calib::prepare_update_with(inner, body, require_expected_revision).map(|p| Self::GripperCalib(Box::new(p))),
            "/api/v1/poses/save" => crate::poses::prepare_save_with(inner, body, require_expected_revision).map(Self::PosesSave),
            "/api/v1/poses/delete" => crate::poses::prepare_delete_with(inner, body, require_expected_revision).map(Self::PosesDelete),
            "/api/v1/poses/rename" => crate::poses::prepare_rename_with(inner, body, require_expected_revision).map(Self::PosesRename),
            "/api/v1/sequences/save" => crate::sequences::prepare_save(inner, body).map(Self::Sequences),
            "/api/v1/sequences/delete" => crate::sequences::prepare_delete(inner, body).map(Self::Sequences),
            "/api/v1/grasp/setup" => crate::grasp::prepare_setup_update(inner, body, setup_path, require_expected_revision).map(|p| Self::GraspSetup(Box::new(p))),
            _ => return None,
        })
    }

    fn path(&self) -> &'static str {
        match self {
            Self::Keepout(_) => "/api/v1/workspace/keepout",
            Self::Collision(_) => "/api/v1/safety/collision",
            Self::GripperCalib(_) => "/api/v1/gripper/calibration",
            Self::PosesSave(_) => "/api/v1/poses/save",
            Self::PosesDelete(_) => "/api/v1/poses/delete",
            Self::PosesRename(_) => "/api/v1/poses/rename",
            Self::Sequences(p) => if p.sequence.is_some() { "/api/v1/sequences/save" } else { "/api/v1/sequences/delete" },
            Self::GraspSetup(_) => "/api/v1/grasp/setup",
        }
    }

    fn has_write(&self) -> bool {
        match self {
            Self::Keepout(p) => p.write.is_some(),
            Self::Collision(p) => p.write.is_some(),
            Self::GripperCalib(p) => p.write.is_some(),
            Self::PosesSave(p) => p.write.is_some(),
            Self::PosesDelete(p) => p.write.is_some(),
            Self::PosesRename(p) => p.write.is_some(),
            Self::Sequences(p) => p.write.is_some(),
            Self::GraspSetup(p) => p.write.is_some(),
        }
    }

    fn take_write(&mut self) -> Option<crate::persist::PendingWrite> {
        match self {
            Self::Keepout(p) => p.write.take(),
            Self::Collision(p) => p.write.take(),
            Self::GripperCalib(p) => p.write.take(),
            Self::PosesSave(p) => p.write.take(),
            Self::PosesDelete(p) => p.write.take(),
            Self::PosesRename(p) => p.write.take(),
            Self::Sequences(p) => p.write.take(),
            Self::GraspSetup(p) => p.write.take(),
        }
    }

    fn precheck(&mut self, inner: &mut SupervisorInner) -> Option<ArmError> {
        match self {
            Self::Keepout(p) => {
                if inner.keepout.revision != p.base_revision {
                    return Some(ArmError::PolicyRevisionMismatch { expected: p.base_revision, current: inner.keepout.revision });
                }
                if inner.calibration_busy {
                    return Some(ArmError::Rejected { status: 409, code: "CONFIG_BUSY".into(), message: "a camera calibration apply or restore is running".into() });
                }
                if inner.workspace_policy().is_none() {
                    return Some(crate::motion::MotionError::Unsupported(
                        "Keep-out boxes need a profile with kinematics and a workspace".into(),
                    ).into());
                }
                if let Some(reason) = inner.workspace_change_blocker() {
                    return Some(crate::motion::MotionError::MotionBusy(format!("keep-out boxes cannot change while {reason}")).into());
                }
                None
            }
            Self::Collision(p) => {
                if inner.collision.revision != p.base_revision {
                    return Some(ArmError::PolicyRevisionMismatch { expected: p.base_revision, current: inner.collision.revision });
                }
                if inner.calibration_busy {
                    return Some(ArmError::Rejected { status: 409, code: "CONFIG_BUSY".into(), message: "a camera calibration apply or restore is running".into() });
                }
                if p.enabled == Some(true) && !inner.collision_available() {
                    return Some(crate::motion::MotionError::Unsupported(
                        "The collision guard needs a profile gravity model and a driver that reports torque".into(),
                    ).into());
                }
                None
            }
            Self::GripperCalib(ref mut p) => crate::gripper_calib::precheck_update(inner, p).err(),
            Self::PosesSave(p) => crate::poses::precheck_save(inner, p).err(),
            Self::PosesDelete(p) => crate::poses::precheck_delete(inner, p).err(),
            Self::PosesRename(p) => crate::poses::precheck_save(inner, p).err(),
            Self::Sequences(p) => crate::sequences::precheck(inner, p).err(),
            Self::GraspSetup(p) => crate::grasp::precheck_setup_update(inner, p).err(),
        }
    }

    fn commit(self, inner: &mut SupervisorInner) -> Result<Vec<(&'static str, Value)>, ArmError> {
        match self {
            Self::Keepout(p) => {
                let state = crate::keepout::commit_update(inner, p)?;
                Ok(vec![
                    ("boxes", json!(state.boxes)),
                    ("source", json!(state.source)),
                    ("revision", json!(state.revision)),
                ])
            }
            Self::Collision(p) => {
                let cfg = crate::collision::commit_update(inner, p)?;
                Ok(vec![
                    ("collision", json!(cfg)),
                    ("revision", json!(cfg.revision)),
                ])
            }
            Self::GripperCalib(p) => {
                let (policy, revision) = crate::gripper_calib::commit_update(inner, *p)?;
                Ok(vec![
                    ("policy", json!(policy)),
                    ("closed_rad", json!(policy.closed_rad)),
                    ("open_rad", json!(policy.open_rad)),
                    ("jaw_opening", json!(policy.jaw_opening)),
                    ("revision", json!(revision)),
                ])
            }
            Self::PosesSave(p) => {
                let (pose, revision) = crate::poses::commit_save(inner, p)?;
                Ok(vec![
                    ("pose", json!(pose)),
                    ("revision", json!(revision)),
                ])
            }
            Self::PosesDelete(p) => {
                let revision = crate::poses::commit_delete(inner, p)?;
                Ok(vec![
                    ("revision", json!(revision)),
                ])
            }
            Self::PosesRename(p) => {
                let (pose, revision) = crate::poses::commit_save(inner, p)?;
                Ok(vec![
                    ("pose", json!(pose)),
                    ("revision", json!(revision)),
                ])
            }
            Self::Sequences(p) => {
                let (seq, revision) = crate::sequences::commit(inner, p)?;
                let mut fields = vec![("revision", json!(revision))];
                if let Some(seq) = seq { fields.push(("sequence", json!(seq))); }
                Ok(fields)
            }
            Self::GraspSetup(p) => {
                let (setup, revision) = crate::grasp::commit_setup_update(inner, *p)?;
                Ok(vec![
                    ("setup", json!(setup)),
                    ("revision", json!(revision)),
                ])
            }
        }
    }
}

/// Check privileged-operations authorization under the supervisor write lock.
/// Returns `Some(ApiError)` if rejected, `None` if allowed.
fn check_operator_privilege(
    auth: &crate::auth::OperatorAuth,
    ws_token: Option<&str>,
    inner: &mut SupervisorInner,
    path: &str,
    body: &Value,
    epoch: u64,
) -> Option<ApiError> {
    if !auth.is_enforced() || auth.is_operator(ws_token) {
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
        | "/api/v1/poses/save"
        | "/api/v1/poses/rename"
        | "/api/v1/poses/delete"
        | "/api/v1/sequences/save"
        | "/api/v1/sequences/delete"
        | "/api/v1/grasp/setup" => Some(ApiError::operator_required(path)),
        // a sequence run that records writes episodes (like /api/v1/dataset/record/*)
        "/api/v1/action/run_sequence"
            if body.get("record").is_some_and(|r| !r.is_null()) =>
        {
            Some(ApiError::operator_required("run_sequence (record)"))
        }
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
            if let Some(fault) = inner.safety.fault() {
                if !fault.recoverable || fault.code == crate::safety::FaultCode::Collision {
                    return Some(ApiError::operator_required("clear_fault (non-recoverable fault or EStop)"));
                }
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
fn lease_free_operator_check(auth: &crate::auth::OperatorAuth, ws_token: Option<&str>, path: &str, body: &Value) -> Option<ApiError> {
    if !auth.is_enforced() || auth.is_operator(ws_token) { return None; }
    if path == "/api/v1/record/stop" && body.get("save").and_then(Value::as_bool) == Some(false) { return None; }
    Some(ApiError::operator_required(path))
}

/// A calibration command checked under the lock.
enum CalibJob { Apply(Box<crate::calibration::ApplyRequest>), Restore(Value), MotorZero(crate::motor_zero::MotorZeroRequest), JointZeroRestore(String) }

/// What a checked calibration command does: a task (job, busy flag set) or a reply under the
/// lock (motor zero dry run).
enum CalibStart { Job(CalibJob, bool), Reply(Value) }

/// Paths refused while a calibration transaction runs: motion (MOTION_BUSY) and configuration
/// writes (CONFIG_BUSY). Stop, torque, clear_fault and everything else go through.
fn calibration_busy_code(inner: &SupervisorInner, path: &str) -> Option<&'static str> {
    if !inner.calibration_busy || path == "/api/v1/motion/stop" { return None; }
    if inner.motor_zero_busy && matches!(path, "/api/v1/torque" | "/api/v1/arm" | "/api/v1/safety/clear_fault") { return Some("MOTION_BUSY"); }
    if path.starts_with("/api/v1/action/") || path.starts_with("/api/v1/motion/") || path.starts_with("/api/v1/mode/") { return Some("MOTION_BUSY"); }
    if path.starts_with("/api/v1/calibration/") || matches!(path, "/api/v1/grasp/setup" | "/api/v1/poses/save" | "/api/v1/poses/rename" | "/api/v1/poses/delete"
        | "/api/v1/sequences/save" | "/api/v1/sequences/delete"
        | "/api/v1/safety/collision" | "/api/v1/workspace/keepout" | "/api/v1/gripper/calibration") { return Some("CONFIG_BUSY"); }
    None
}

/// Under the lock: parse and check a calibration command; a real apply / restore marks the
/// transaction running (`calibration_busy`, cleared by its task). Returns (job, busy set).
fn calibration_job(inner: &mut SupervisorInner, path: &str, body: &Value) -> Result<CalibStart, crate::calibration::CalibError> {
    if path == crate::motor_zero::PATH {
        let (req, dry) = crate::motor_zero::check(inner, body)?;
        if let Some(reply) = dry { return Ok(CalibStart::Reply(reply)); }
        inner.calibration_busy = true;
        inner.motor_zero_busy = true;
        return Ok(CalibStart::Job(CalibJob::MotorZero(req), true));
    }
    if path == crate::motor_zero::RESTORE_PATH {
        let name = crate::motor_zero::check_restore(inner, body)?;
        inner.calibration_busy = true;
        inner.motor_zero_busy = true;
        return Ok(CalibStart::Job(CalibJob::JointZeroRestore(name), true));
    }
    crate::calibration::begin_checks(inner)?;
    let job = if path == "/api/v1/calibration/apply" {
        let req = crate::calibration::parse_apply(body)?;
        if !req.dry_run && req.kinematics.is_some() { crate::calibration::kinematics_armed(inner)?; }
        CalibJob::Apply(Box::new(req))
    } else {
        if !body.is_object() { return Err(crate::calibration::CalibError::new(400, "INVALID_COMMAND", "Command body must be an object")); }
        CalibJob::Restore(body.clone())
    };
    let busy = !matches!(&job, CalibJob::Apply(r) if r.dry_run);
    if busy { inner.calibration_busy = true; }
    Ok(CalibStart::Job(job, busy))
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

async fn handle_socket(socket: WebSocket, supervisor: Arc<ArmSupervisor>, deps: crate::grasp::GraspDeps, calib: crate::calibration::CalibrationDeps, sandbox: Arc<crate::api::sandbox::SandboxState>, obs_rate: Option<ObsRate>, auth: crate::auth::OperatorAuth, ws_token: Option<String>) {
    #[cfg(not(feature = "policy-runtime"))]
    let _ = &sandbox;
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
    #[cfg(feature = "policy-runtime")]
    let recv_sandbox = sandbox;
    let mut recv_task = tokio::spawn(async move {
        // the operator session of this connection: from the upgrade request, replaced by `auth` messages
        let mut ws_token = ws_token;
        let mut client_label: Option<String> = None;
        let mut last_sequence: Option<u64> = None;
        let mut last_request: Option<u64> = None;
        // Delayed acks (sent after a state-file write) come back here and are stamped with
        // the current epoch and enqueued by this loop, in order with its other replies
        // (epoch_sync, rejects): the epochs a client receives never go back.
        let (ack_tx, mut ack_rx) = mpsc::channel::<Value>(64);
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
        #[cfg(feature = "policy-runtime")]
        let policy_stop_gen = Arc::new(AtomicU64::new(0));
        #[cfg(feature = "policy-runtime")]
        let pending_policy_loads = Arc::new(AtomicUsize::new(0));
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
            // `{"type":"auth","token":<unlock token|null>}`: use this operator session on the open socket
            // (a browser unlocks over REST; reconnecting would drop the lease and park a torqued arm).
            // The token is checked on every privileged use, as from the upgrade request.
            if kind == "auth" {
                ws_token = value.get("token").and_then(Value::as_str).filter(|t| !t.is_empty()).map(str::to_string);
                let status = auth.status(ws_token.as_deref());
                let reply = json!({"type":"auth_ack", "seq":value.get("seq"), "auth_enforced":status.auth_enforced,
                    "role":status.role, "expires_in_s":status.expires_in_s});
                if out_tx.send(Message::Text(reply.to_string().into())).await.is_err() { break; }
                continue;
            }
            // depth measurement: read-only, no motion, no lease
            if kind == "command" && value.get("path").and_then(Value::as_str) == Some(crate::depth_measure::PATH) {
                let request_id = value.get("request_id").and_then(Value::as_u64);
                if request_id.is_none() || last_request.is_some_and(|last| request_id.unwrap() <= last) {
                    let reply = json!({"type":"command_reject", "request_id":value.get("request_id"), "status":400, "code":"INVALID_COMMAND", "message":"request_id must be present and strictly increase", "error":"INVALID_COMMAND"});
                    if out_tx.send(Message::Text(reply.to_string().into())).await.is_err() { break; }
                    continue;
                }
                last_request = request_id;
                if let Some(e) = lease_free_operator_check(&auth, ws_token.as_deref(), crate::depth_measure::PATH, &value["body"]) {
                    let mut reply = rejection(e, "command_reject", recv_supervisor.inner().read().await.safety.epoch_id());
                    reply["request_id"] = value["request_id"].clone();
                    if out_tx.send(Message::Text(reply.to_string().into())).await.is_err() { break; }
                    continue;
                }
                let (sup, source, tx) = (recv_supervisor.clone(), recv_deps.source.clone(), ack_tx.clone());
                tokio::spawn(async move {
                    let result = crate::depth_measure::measure(&sup, source.as_ref(), &value["body"]).await;
                    let _ = tx.send(crate::teach::reply(&value["request_id"], result)).await;
                });
                continue;
            }
            // teach commands without a motion lease (episode store; recorder ownership)
            if kind == "command" && crate::teach::is_lease_free(value.get("path").and_then(Value::as_str).unwrap_or("")) {
                let request_id = value.get("request_id").and_then(Value::as_u64);
                if request_id.is_none() || last_request.is_some_and(|last| request_id.unwrap() <= last) {
                    let reply = json!({"type":"command_reject", "request_id":value.get("request_id"), "status":400, "code":"INVALID_COMMAND", "message":"request_id must be present and strictly increase", "error":"INVALID_COMMAND"});
                    if out_tx.send(Message::Text(reply.to_string().into())).await.is_err() { break; }
                    continue;
                }
                last_request = request_id;
                if let Some(e) = lease_free_operator_check(&auth, ws_token.as_deref(), value["path"].as_str().unwrap_or(""), &value["body"]) {
                    let mut reply = rejection(e, "command_reject", recv_supervisor.inner().read().await.safety.epoch_id());
                    reply["request_id"] = value["request_id"].clone();
                    if out_tx.send(Message::Text(reply.to_string().into())).await.is_err() { break; }
                    continue;
                }
                let caller = crate::teach::Caller {
                    connection: recv_connection.clone(),
                    client_id: value.get("client_id").and_then(Value::as_str).map(str::to_string).or_else(|| client_label.clone()),
                };
                let (sup, tx) = (recv_supervisor.clone(), ack_tx.clone());
                tokio::spawn(async move {
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
                                        if auth.is_enforced() && !auth.is_operator(ws_token.as_deref()) {
                                            let mut reply = json!({
                                                "type": "lease_rejected",
                                                "status": 403,
                                                "code": "OPERATOR_REQUIRED",
                                                "error": "operator_required",
                                                "message": "Operator unlock is required for force lease preemption"
                                            });
                                            if let Some(s) = seq { reply["seq"] = json!(s); }
                                            response = Some(reply);
                                        } else {
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
                                    if let Some(auth_err) = check_operator_privilege(&auth, ws_token.as_deref(), &mut inner, path, &value["body"], epoch) {
                                        response = Some(command_reject(&inner, &value["request_id"], auth_err));
                                    } else if let Some(code) = calibration_busy_code(&inner, path) {
                                        response = Some(command_reject(&inner, &value["request_id"],
                                            ApiError(StatusCode::CONFLICT, "a calibration apply, restore or motor zero is running".into(), code.into())));
                                    } else if path == "/api/v1/calibration/apply" || path == "/api/v1/calibration/restore" || path == crate::motor_zero::PATH
                                        || path == crate::motor_zero::RESTORE_PATH {
                                        response = match calibration_job(&mut inner, path, &value["body"]) {
                                            Ok(CalibStart::Job(job, busy)) => { pending_calibration = Some((value["request_id"].clone(), job, local_generation, epoch, busy)); None }
                                            Ok(CalibStart::Reply(reply)) => Some(fields_ack(&value["request_id"], reply)),
                                            Err(e) => Some(calibration_reject(&value["request_id"], &e)),
                                        };
                                    } else if path.starts_with("/api/v1/action/") {
                                        response = match prepare_action(&mut inner, path, &value["body"], epoch, &value["request_id"], &auth, ws_token.as_deref()) {
                                            Ok(ActionStart::Started(id, duration)) => Some(command_ack(&inner, &value["request_id"], Some((id, duration)))),
                                            Ok(ActionStart::Plan(plan)) => { pending_plan = Some((plan, local_generation)); None }
                                            Ok(ActionStart::Macro(m)) => { pending_macro = Some((value["request_id"].clone(), m, local_generation)); None }
                                            Ok(ActionStart::Replay(input)) => { pending_replay = Some((value["request_id"].clone(), input, local_generation)); None }
                                            Ok(ActionStart::DryRunGrasp(body)) => { pending_dry_run_grasp = Some((value["request_id"].clone(), body)); None }
                                            Ok(ActionStart::Sequence(plan)) => Some(match crate::sequences::begin(&mut inner, &recv_supervisor, *plan) {
                                                Ok(started) => command_ack(&inner, &value["request_id"], Some(started)),
                                                Err(error) => command_reject(&inner, &value["request_id"], error.into()),
                                            }),
                                            Ok(ActionStart::CalibHover(plan)) => Some(match crate::calib_hover::begin(&mut inner, &recv_supervisor, *plan,
                                                recv_calib.upstream.clone(), crate::calib_hover::jobs_dir(&recv_calib.path)) {
                                                Ok((id, duration, job_id)) => {
                                                    let mut ack = command_ack(&inner, &value["request_id"], Some((id, duration)));
                                                    ack["job_id"] = json!(job_id);
                                                    ack
                                                }
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
                                                policy_stop_gen.fetch_add(1, Ordering::SeqCst);
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
                                        auth.is_enforced(),
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
                                        let result = execute_command(&mut inner, path, &value["body"], epoch);
                                        #[cfg(feature = "policy-runtime")]
                                        if path == "/api/v1/motion/stop" && result.is_ok() {
                                            policy_stop_gen.fetch_add(1, Ordering::SeqCst);
                                        }
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
                    let PendingPlan { request_id, kind, input, goal, speed, tag, opts, approach } = plan;
                    let planned = tokio::task::spawn_blocking(move || {
                        let result = motion::plan_cartesian_opts(&input, goal, speed, opts);
                        let pre = match (&approach, &result) {
                            (Ok(plan), Ok(direct)) => crate::backlash::plan_cartesian_pre(&input, direct, plan, None, speed, opts),
                            (Err(Some(r)), _) => crate::backlash::Pre::Direct((*r).to_string()),
                            _ => crate::backlash::Pre::Off,
                        };
                        (input, result.map(|path| (path, pre, approach.ok())))
                    }).await;
                    let mut inner = recv_supervisor.inner().write().await;
                    if let Some(hook) = inner.before_install_hook.take() { hook(&mut inner); }
                    response = Some(match planned {
                        Err(e) => command_reject(&inner, &request_id, ArmError::Internal(format!("planner task failed: {e}")).into()),
                        Ok((_, Err(error))) => command_reject(&inner, &request_id, ArmError::from(error).into()),
                        Ok((input, Ok((path, pre, approach)))) => {
                            if recv_generation.load(Ordering::SeqCst) != generation || !inner.validate_servo_lease(&recv_connection, generation) {
                                json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"LEASE_LOST", "message":"This connection lost the lease while planning", "error":"LEASE_LOST", "epoch_id":inner.safety.epoch_id()})
                            } else {
                                match inner.install_path_same_side(&input, path, pre, approach.as_ref(), kind, tag) {
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
                #[cfg(feature = "policy-runtime")]
                if let Some((request_id, pending, generation, loading_guard)) = pending_policy.take() {
                    let sup = recv_supervisor.clone();
                    let sandbox_c = recv_sandbox.clone();
                    let vision_src = recv_deps.source.clone();
                    let conn = recv_connection.clone();
                    let gen_now = recv_generation.clone();
                    let tx = ack_tx.clone();
                    let ws_closed_c = ws_closed.clone();
                    let stop_gen_c = policy_stop_gen.clone();
                    let my_stop_gen = stop_gen_c.load(Ordering::SeqCst);
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
                    let auth_c = auth.clone();
                    let ws_token_c = ws_token.clone();
                    tokio::spawn(async move {
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
                        let prepared = tokio::task::spawn_blocking(move || {
                            let delay_ms = sandbox_load.policy_load_delay_ms();
                            if delay_ms > 0 {
                                std::thread::sleep(std::time::Duration::from_millis(delay_ms));
                            }
                            let dir = sandbox_load.policy_dir(&policy_id_load).ok_or_else(|| ArmError::Rejected {
                                status: 404,
                                code: "POLICY_NOT_FOUND".into(),
                                message: format!("Policy pack '{policy_id_load}' not found"),
                            })?;
                            let mut prepared_policy = crate::policy_runtime::prepare_policy(&dir, &config).map_err(ArmError::from)?;
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
                            // speed gate: measured here, off the supervisor lock and the control tick
                            let warmup = prepared_policy
                                .warmup(crate::policy_runtime::WARMUP_INFERENCES, crate::policy_runtime::WARMUP_MAX_TOTAL)
                                .map_err(ArmError::from)?;
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

                                let mut inner = sup.inner().write().await;
                                loading_guard.clear(&mut inner);
                                if ws_closed_c.load(Ordering::SeqCst) {
                                    return;
                                }
                                if gen_now.load(Ordering::SeqCst) != generation || !inner.validate_servo_lease(&conn, generation) {
                                    json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"LEASE_LOST", "message":"This connection lost the lease while loading policy", "error":"LEASE_LOST", "epoch_id":inner.safety.epoch_id()})
                                } else if inner.safety.epoch_id() != epoch_id {
                                    json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"EPOCH_MISMATCH", "message":"Epoch changed while loading policy", "error":"EPOCH_MISMATCH", "epoch_id":inner.safety.epoch_id()})
                                } else if stop_gen_c.load(Ordering::SeqCst) != my_stop_gen {
                                    json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"STOPPED", "message":"Stopped while loading policy", "error":"STOPPED", "epoch_id":inner.safety.epoch_id()})
                                } else if auth_c.is_enforced() && !inner.policy_live_allowed() && !auth_c.is_operator(ws_token_c.as_deref()) {
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
                                                let (cur_scene, cur_tool) = sandbox_c.active_scene_and_tool().await;
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
                                                match inner.begin_policy_run(start_req) {
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
                    let inner = recv_supervisor.inner().read().await;
                    let mut ack = command_ack(&inner, &request_id, None);
                    for (k, v) in dry.fields() { ack[k.as_str()] = v; }
                    response = Some(ack);
                }
                // Calibration apply / restore: its own task (HTTP to rebot-vision, file writes);
                // this loop goes on (heartbeats, stop) and the reply comes back through ack_tx.
                if let Some((request_id, job, generation, epoch, busy)) = pending_calibration.take() {
                    let (sup, calib, conn, gen_now, tx) = (recv_supervisor.clone(), recv_calib.clone(), recv_connection.clone(), recv_generation.clone(), ack_tx.clone());
                    let auth_c = auth.clone();
                    let token_c = ws_token.clone();
                    tokio::spawn(async move {
                        let check: crate::calibration::LeaseCheck = Box::new(move |inner: &mut SupervisorInner| {
                            if auth_c.is_enforced() && !auth_c.is_operator(token_c.as_deref()) {
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
                        let job_task = tokio::spawn(async move {
                            match job {
                                CalibJob::Apply(req) => crate::calibration::apply(&job_sup, &calib, *req, &check).await,
                                CalibJob::Restore(body) => crate::calibration::restore(&job_sup, &calib, &body, &check).await,
                                CalibJob::MotorZero(req) => crate::motor_zero::run(&job_sup, &calib, req, &check).await,
                                CalibJob::JointZeroRestore(name) => crate::motor_zero::restore(&job_sup, &calib, name, &check).await,
                            }
                        });
                        let result = job_task.await.unwrap_or_else(|e| {
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
                // Atomic two-phase persist-before-commit config writes (keepout, collision, gripper_calibration, poses, grasp_setup).
                // Phase 1 (stage) runs outside sup.inner() lock; Phase 2 (auth + lease + epoch + precheck + publish + commit) runs under sup.inner() write lock.
                if let Some((request_id, mut prepared, generation, epoch)) = pending_config_write.take() {
                    let sup = recv_supervisor.clone();
                    let tx = ack_tx.clone();
                    let auth_c = auth.clone();
                    let token_c = ws_token.clone();
                    let conn_c = recv_connection.clone();
                    let gen_now = recv_generation.clone();
                    let op_path = prepared.path();
                    tokio::spawn(async move {
                        let cw_lock = sup.config_write_lock();
                        let _cw = cw_lock.lock().await;
                        let write = prepared.take_write().expect("checked has_write");
                        let staged_res = tokio::task::spawn_blocking(move || {
                            crate::persist::StagedWrite::stage(write.path, write.text)
                        }).await;
                        let reply = match staged_res {
                            Err(e) => {
                                tracing::warn!("State file stage task failed: {e}");
                                let inner = sup.inner().read().await;
                                command_reject(&inner, &request_id, ApiError::persist_failed(format!("Failed to persist configuration: {e}")))
                            }
                            Ok(Err(e)) => {
                                tracing::warn!("State file not staged: {e}");
                                let inner = sup.inner().read().await;
                                command_reject(&inner, &request_id, ApiError::persist_failed(format!("Failed to persist configuration: {e}")))
                            }
                            Ok(Ok(staged)) => {
                                let (reply_or_ack, dir_sync_opt) = {
                                    let mut inner = sup.inner().write().await;
                                    if auth_c.is_enforced() && !auth_c.is_operator(token_c.as_deref()) {
                                        staged.abort();
                                        (command_reject(&inner, &request_id, ApiError::operator_required(op_path)), None)
                                    } else if gen_now.load(Ordering::SeqCst) != generation || !inner.validate_servo_lease(&conn_c, generation) {
                                        staged.abort();
                                        (json!({
                                            "type": "command_reject",
                                            "request_id": request_id,
                                            "status": 409,
                                            "code": "LEASE_LOST",
                                            "message": "This connection lost the lease while persisting configuration",
                                            "error": "LEASE_LOST",
                                            "epoch_id": inner.safety.epoch_id()
                                        }), None)
                                    } else if inner.safety.epoch_id() != epoch {
                                        staged.abort();
                                        (json!({
                                            "type": "command_reject",
                                            "request_id": request_id,
                                            "status": 409,
                                            "code": "EPOCH_MISMATCH",
                                            "message": "Command epoch is no longer current",
                                            "error": "EPOCH_MISMATCH",
                                            "epoch_id": inner.safety.epoch_id()
                                        }), None)
                                    } else if let Some(err) = prepared.precheck(&mut inner) {
                                        staged.abort();
                                        (command_reject(&inner, &request_id, err.into()), None)
                                    } else {
                                        match staged.rename_only() {
                                            Err(e) => {
                                                tracing::warn!("State file not published: {e}");
                                                (command_reject(&inner, &request_id, ApiError::persist_failed(format!("Failed to persist configuration: {e}"))), None)
                                            }
                                            Ok(dir_sync) => match prepared.commit(&mut inner) {
                                                Ok(fields) => {
                                                    let mut ack = command_ack(&inner, &request_id, None);
                                                    for (key, field) in fields { ack[key] = field; }
                                                    ack["persisted"] = json!(true);
                                                    (ack, Some(dir_sync))
                                                }
                                                Err(err) => (command_reject(&inner, &request_id, err.into()), None),
                                            }
                                        }
                                    }
                                };
                                match dir_sync_opt {
                                    Some(dir_sync) => {
                                        let mut ack = reply_or_ack;
                                        let outcome = tokio::task::spawn_blocking(move || dir_sync.sync_parent_dir())
                                            .await
                                            .unwrap_or_else(|e| crate::persist::PublishOutcome::PublishedDurabilityUnconfirmed(e.to_string()));
                                        match outcome {
                                            crate::persist::PublishOutcome::Published => {
                                                ack["durability_confirmed"] = json!(true);
                                            }
                                            crate::persist::PublishOutcome::PublishedDurabilityUnconfirmed(warn_msg) => {
                                                ack["durability_confirmed"] = json!(false);
                                                ack["durability_warning"] = json!(warn_msg);
                                            }
                                        }
                                        ack
                                    }
                                    None => reply_or_ack,
                                }
                            }
                        };
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
    supervisor.recorder_connection_closed(&connection).await;
    let generation = connection_generation.load(Ordering::SeqCst);
    let mut inner = supervisor.inner().write().await;
    if generation != 0 && inner.release_lease(&connection, generation) {
        inner.stop_motion();
    }
}
