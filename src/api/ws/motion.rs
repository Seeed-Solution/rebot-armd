//! Motion commands of the WS command path: jog, home, park/unload, presets, and gravity comp
//! (family M). Jog, home, park, and presets prepare off the supervisor lock and install under a
//! short lock via `rt_core::requests::motion` with zero in-lock planning fallback.

use std::time::Instant;

#[allow(unused_imports)]
use super::*;
use super::dispatch::{
    AdmissionClass, AuthRequirement, DispatchEntry, DispatchHandler, DispatchKey, FrameKind,
    LeaseRequirement, OwnerDispatchCtx, ReplyPolicy,
};
use crate::rt_core::requests::motion as req;

const MAX_START_DRIFT_RETRIES: usize = 5;
const PARK_CONTINUATION_TIMEOUT: Duration = Duration::from_secs(15);

pub(super) const ENTRIES: &[DispatchEntry] = &[
    DispatchEntry {
        key: DispatchKey::frame(FrameKind::Preset),
        handler: DispatchHandler::Owner(dispatch_preset),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::PresetReply,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/motion/jog"),
        handler: DispatchHandler::Owner(dispatch_motion_command),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandDeferred,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/motion/home"),
        handler: DispatchHandler::Owner(dispatch_motion_command),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandDeferred,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/motion/unload"),
        handler: DispatchHandler::Owner(dispatch_motion_command),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandDeferred,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/mode/gravity_comp"),
        handler: DispatchHandler::Owner(dispatch_motion_command),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::Operator(None),
        reply: ReplyPolicy::CommandImmediate,
    },
];

pub(super) enum MotionReplyKind {
    Command { request_id: Value },
    Preset { name: String },
}

pub(super) struct PendingMotion {
    reply_kind: MotionReplyKind,
    request: req::MotionRequest,
    snapshot: req::MotionPlanSnapshot,
    epoch: u64,
    generation: u64,
}

fn format_motion_ack(inner: &SupervisorInner, kind: &MotionReplyKind, started: Started) -> Value {
    match kind {
        MotionReplyKind::Command { request_id } => command_ack(inner, request_id, started),
        MotionReplyKind::Preset { name } => {
            let mut ack = json!({"type":"preset_ack", "name":name, "epoch_id":inner.safety.epoch_id()});
            if let Some((id, _)) = started { ack["action_id"] = json!(id); }
            ack
        }
    }
}

fn format_motion_reject(inner: &SupervisorInner, kind: &MotionReplyKind, error: ApiError) -> Value {
    match kind {
        MotionReplyKind::Command { request_id } => command_reject(inner, request_id, error),
        MotionReplyKind::Preset { name } => {
            let mut reply = rejection(error, "preset_reject", inner.safety.epoch_id());
            reply["name"] = json!(name);
            reply
        }
    }
}

fn format_status_reject(kind: &MotionReplyKind, status: u16, code: &str, message: &str, epoch: Option<u64>) -> Value {
    let mut r = match kind {
        MotionReplyKind::Command { request_id } => json!({
            "type":"command_reject", "request_id":request_id, "status":status, "code":code, "error":code, "message":message,
        }),
        MotionReplyKind::Preset { name } => json!({
            "type":"preset_reject", "name":name, "status":status, "code":code, "error":code, "message":message,
        }),
    };
    if let Some(ep) = epoch { r["epoch_id"] = json!(ep); }
    r
}

fn parse_motion_command(path: &str, body: &Value) -> Result<req::MotionRequest, ApiError> {
    validate_command_body_common(body).map_err(invalid_command)?;
    match path {
        "/api/v1/motion/jog" => {
            let speed_scale = crate::motion::num(body, "speed_scale").map_err(ArmError::from)?;
            let target = positions(body.get("target_positions"))?;
            let duration_secs = body.get("duration_secs").and_then(Value::as_f64);
            Ok(req::MotionRequest::jog(target, duration_secs, speed_scale))
        }
        "/api/v1/motion/home" => Ok(req::MotionRequest::home(body.get("duration_secs").and_then(Value::as_f64))),
        "/api/v1/motion/unload" => Ok(req::MotionRequest::park()),
        _ => Err(invalid_command("Unknown actuator command")),
    }
}

fn dispatch_preset(ctx: &mut OwnerDispatchCtx<'_>) -> Option<Value> {
    if ctx.inner.calibration_busy {
        return Some(json!({
            "type":"preset_reject", "name":ctx.value.get("name"), "status":409, "code":"MOTION_BUSY",
            "error":"MOTION_BUSY", "message":"a camera calibration apply or restore is running", "epoch_id":ctx.epoch,
        }));
    }
    let name = ctx.value.get("name").and_then(Value::as_str).unwrap_or("").to_string();
    let reply_kind = MotionReplyKind::Preset { name: name.clone() };
    let parsed = match name.as_str() {
        "home" => Ok(req::MotionRequest::home(None)),
        "park" | "unload" => Ok(req::MotionRequest::park()),
        _ => {
            let mut j = json!({"target_positions":ctx.value.get("positions")});
            if let Some(s) = ctx.value.get("speed_scale") { j["speed_scale"] = s.clone(); }
            parse_motion_command("/api/v1/motion/jog", &j)
        }
    };
    let request = match parsed {
        Ok(r) => r,
        Err(e) => return Some(format_motion_reject(ctx.inner, &reply_kind, e)),
    };
    match req::preflight_motion_request(ctx.inner, ctx.stamp, &request, Some(ctx.epoch)) {
        Ok(snapshot) => {
            *ctx.pending_motion = Some(PendingMotion { reply_kind, request, snapshot, epoch: ctx.epoch, generation: ctx.local_generation });
            None
        }
        Err(e) => Some(format_motion_reject(ctx.inner, &reply_kind, e.into())),
    }
}

fn dispatch_motion_command(ctx: &mut OwnerDispatchCtx<'_>) -> Option<Value> {
    if ctx.path == "/api/v1/mode/gravity_comp" {
        let result = (|| -> Result<Started, ApiError> {
            validate_command_body_common(&ctx.value["body"]).map_err(invalid_command)?;
            let enable = ctx.value["body"].get("enable").and_then(Value::as_bool)
                .ok_or_else(|| invalid_command("enable must be boolean"))?;
            if enable {
                let g = gravity_comp_request(&ctx.value["body"])?;
                let admitted = ctx.inner.check_admission(ctx.stamp).map_err(ArmError::from)?;
                SupervisorInner::enter_gravity_comp(admitted, g)?;
            } else {
                ctx.inner.exit_gravity_comp("REQUESTED", "Exit requested");
            }
            Ok(None)
        })();
        return Some(match result {
            Ok(started) => command_ack(ctx.inner, &ctx.value["request_id"], started),
            Err(error) => command_reject(ctx.inner, &ctx.value["request_id"], error),
        });
    }
    let reply_kind = MotionReplyKind::Command { request_id: ctx.value["request_id"].clone() };
    let request = match parse_motion_command(ctx.path, &ctx.value["body"]) {
        Ok(r) => r,
        Err(e) => return Some(format_motion_reject(ctx.inner, &reply_kind, e)),
    };
    match req::preflight_motion_request(ctx.inner, ctx.stamp, &request, Some(ctx.epoch)) {
        Ok(snapshot) => {
            *ctx.pending_motion = Some(PendingMotion { reply_kind, request, snapshot, epoch: ctx.epoch, generation: ctx.local_generation });
            None
        }
        Err(e) => Some(format_motion_reject(ctx.inner, &reply_kind, e.into())),
    }
}

/// Complete an off-lock jog/home/park/preset preparation and install under a short write lock.
/// Retries off-lock on `StartDrifted` only (bounded), with zero in-lock planning fallback.
#[allow(clippy::too_many_arguments)]
pub(super) async fn complete_pending_motion(
    pending: PendingMotion,
    stamp: crate::supervisor::AdmissionStamp,
    recv_supervisor: &Arc<ArmSupervisor>,
    recv_connection: &str,
    recv_generation: &AtomicU64,
    worker_closing: &Arc<AtomicBool>,
    worker_tasks: &ConnectionTasks,
) -> Option<Value> {
    let PendingMotion { reply_kind, request, mut snapshot, epoch, generation } = pending;
    for attempt in 0..MAX_START_DRIFT_RETRIES {
        let snap_for_plan = snapshot.clone();
        let req_for_plan = request.clone();
        let cancel = worker_closing.clone();
        let plan_conn = recv_connection.to_string();
        let planned = tokio::task::spawn_blocking(move || {
            maybe_pause_blocking_plan(&plan_conn);
            if cancel.load(Ordering::Acquire) { return None; }
            let res = req::prepare_motion(&snap_for_plan, &req_for_plan);
            if cancel.load(Ordering::Acquire) { return None; }
            Some(res)
        })
        .await;
        if worker_closing.load(Ordering::SeqCst) { return None; }
        let prepared = match planned {
            Ok(None) => return None,
            Ok(Some(res)) => res,
            Err(e) => {
                let inner = recv_supervisor.inner().read().await;
                return Some(format_motion_reject(&inner, &reply_kind, ArmError::Internal(format!("motion planner failed: {e}")).into()));
            }
        };

        let mut inner = recv_supervisor.inner().write().await;
        if worker_closing.load(Ordering::SeqCst) { return None; }
        if inner.detached {
            let ep = recv_supervisor.hub().latest().map(|s| s.obs.epoch_id);
            return Some(format_status_reject(&reply_kind, 503, "OWNER_TRANSFERRED", "Ownership transferred to RT executor while planning", ep));
        }
        if let Some(hook) = inner.before_install_hook.take() { hook(&mut inner); }
        if recv_generation.load(Ordering::SeqCst) != generation || !inner.validate_servo_lease(recv_connection, generation) {
            return Some(format_status_reject(&reply_kind, 409, "LEASE_LOST", "This connection lost the lease while planning", Some(inner.safety.epoch_id())));
        }
        if let Err(err) = inner.preflight_admission(stamp) {
            return Some(format_motion_reject(&inner, &reply_kind, ArmError::from(err).into()));
        }
        let plan = match prepared {
            Ok(p) => p,
            Err(e) => return Some(format_motion_reject(&inner, &reply_kind, e.into())),
        };
        let installed = match request.kind {
            req::MotionKind::Jog => req::install_jog(&mut inner, stamp, &plan.slot, plan.heap_traj, Some(epoch), false),
            req::MotionKind::Home => req::install_home(&mut inner, stamp, &plan.slot, plan.heap_traj, Some(epoch), false),
            req::MotionKind::Park => req::install_park(&mut inner, stamp, &plan.slot, plan.heap_traj, Some(epoch), false),
        };
        match installed {
            Ok(req::MotionInstalled::Started { action_id, expected_duration_s, .. }) => {
                return Some(format_motion_ack(&inner, &reply_kind, Some((action_id, expected_duration_s))));
            }
            Ok(req::MotionInstalled::DeferredContinuation { action_id, expected_duration_s, .. }) => {
                let notify = Arc::clone(&inner.park_continuation_notify);
                let ack = format_motion_ack(&inner, &reply_kind, Some((action_id, expected_duration_s)));
                drop(inner);
                spawn_park_continuation(
                    action_id, epoch, stamp.stop_gen, recv_connection.to_string(), notify, recv_supervisor.clone(), worker_closing.clone(), worker_tasks,
                );
                return Some(ack);
            }
            Err(req::MotionInstallError::StartDrifted) if attempt + 1 < MAX_START_DRIFT_RETRIES => {
                let Some(next) = req::MotionPlanSnapshot::refresh_for_drift_retry(&inner, snapshot.config) else {
                    return Some(format_motion_reject(&inner, &reply_kind, req::MotionInstallError::PlanStale.into_arm_error(&inner, request.action_kind, Some(epoch)).into()));
                };
                snapshot = next;
                continue;
            }
            Err(err) => {
                return Some(format_motion_reject(&inner, &reply_kind, err.into_arm_error(&inner, request.action_kind, Some(epoch)).into()));
            }
        }
    }
    let inner = recv_supervisor.inner().read().await;
    Some(format_motion_reject(&inner, &reply_kind, req::MotionInstallError::PlanStale.into_arm_error(&inner, request.action_kind, Some(epoch)).into()))
}

#[allow(clippy::too_many_arguments)]
fn spawn_park_continuation(
    action_id: u64,
    epoch: u64,
    request_stop_gen: u64,
    connection_id: String,
    notify: Arc<tokio::sync::Notify>,
    supervisor: Arc<ArmSupervisor>,
    closing: Arc<AtomicBool>,
    worker_tasks: &ConnectionTasks,
) {
    worker_tasks.spawn(async move {
        let deadline = Instant::now() + PARK_CONTINUATION_TIMEOUT;
        let mut retries = 0usize;
        loop {
            if closing.load(Ordering::Acquire) || Instant::now() >= deadline {
                let mut inner = supervisor.inner().write().await;
                if !inner.detached {
                    req::fail_park_continuation(&mut inner, action_id, "UNLOAD_FAILED", "Park continuation timed out or disconnected");
                }
                return;
            }
            let notified = notify.notified();
            let ready_snapshot = {
                let inner = supervisor.inner().read().await;
                if inner.detached || inner.current_stop_gen() != request_stop_gen { return; }
                let Some(cont) = inner.park_continuation.filter(|c| c.action_id == action_id) else { return; };
                if cont.phase == crate::supervisor::ParkContinuationPhase::ReadyForPlan {
                    Some(req::MotionPlanSnapshot::from_supervisor(&inner))
                } else {
                    None
                }
            };
            let Some(snap) = ready_snapshot else {
                let _ = tokio::time::timeout(Duration::from_millis(50), notified).await;
                continue;
            };
            let cancel = closing.clone();
            let plan_conn = connection_id.clone();
            let planned = tokio::task::spawn_blocking(move || {
                maybe_pause_blocking_plan(&plan_conn);
                if cancel.load(Ordering::Acquire) { return None; }
                Some(req::prepare_motion(&snap, &req::MotionRequest::park()))
            }).await;
            let mut inner = supervisor.inner().write().await;
            if inner.detached { return; }
            if inner.current_stop_gen() != request_stop_gen
                || !inner.park_continuation.is_some_and(|c| c.action_id == action_id)
            {
                return;
            }
            if closing.load(Ordering::Acquire) || Instant::now() >= deadline {
                req::fail_park_continuation(&mut inner, action_id, "UNLOAD_FAILED", "Park continuation timed out or disconnected before install");
                return;
            }
            let plan = match planned {
                Ok(Some(Ok(p))) => p,
                Ok(Some(Err(e))) => {
                    req::fail_park_continuation(&mut inner, action_id, crate::rt_core::ops::static_code(e), "Park planning failed after continuation");
                    return;
                }
                _ => {
                    req::fail_park_continuation(&mut inner, action_id, "UNLOAD_FAILED", "Park continuation planner aborted");
                    return;
                }
            };
            match req::install_park_continuation(&mut inner, action_id, &plan.slot, plan.heap_traj, Some(epoch), false) {
                Ok(_) => return,
                Err(req::MotionInstallError::StartDrifted) if retries + 1 < MAX_START_DRIFT_RETRIES => {
                    retries += 1;
                    continue;
                }
                Err(e) => {
                    req::fail_park_continuation(&mut inner, action_id, e.code(), "Park continuation failed validation");
                    return;
                }
            }
        }
    });
}

/// `scale` (number) and `damping_kd` (number for all six arm joints, or six numbers).
pub(super) fn gravity_comp_request(body: &Value) -> Result<crate::gravity_comp::GravityCompRequest, ApiError> {
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
