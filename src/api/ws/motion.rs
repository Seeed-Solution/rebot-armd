//! Motion commands of the WS command path: jog, home, park/unload, presets, and gravity comp
//! (family M). Jog, home, park, and presets prepare off the supervisor lock and install under a
//! short lock via `rt_core::requests::motion` with zero in-lock planning fallback.

use std::time::Instant;

#[allow(unused_imports)]
use super::*;
use super::dispatch::{
    AdmissionClass, AuthRequirement, DispatchEntry, DispatchHandler, DispatchKey, FrameKind,
    LeaseRequirement, OwnerDispatchCtx, OwnerReply, ReplyPolicy,
};
use crate::rt_core::requests::motion as req;

const MAX_START_DRIFT_RETRIES: usize = 5;

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

fn format_motion_ack_from_snap(snap: &AckSnapshot, kind: &MotionReplyKind, started: Started) -> Value {
    match kind {
        MotionReplyKind::Command { request_id } => command_ack_from_snap(snap, request_id, started),
        MotionReplyKind::Preset { name } => {
            let mut ack = json!({"type":"preset_ack", "name":name, "epoch_id":snap.epoch_id});
            if let Some((id, _)) = started { ack["action_id"] = json!(id); }
            ack
        }
    }
}

fn format_motion_reject_at(epoch_id: u64, kind: &MotionReplyKind, error: ApiError) -> Value {
    match kind {
        MotionReplyKind::Command { request_id } => command_reject_at(epoch_id, request_id, error),
        MotionReplyKind::Preset { name } => {
            let mut reply = rejection(error, "preset_reject", epoch_id);
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

fn dispatch_preset(ctx: &mut OwnerDispatchCtx<'_>) -> Option<OwnerReply> {
    if ctx.inner.calibration_busy {
        return Some(OwnerReply::PresetCalibrationBusy { epoch: ctx.epoch });
    }
    let name = ctx.value.get("name").and_then(Value::as_str).unwrap_or("").to_string();
    let parsed = match name.as_str() {
        "home" => Ok(req::MotionRequest::home(None)),
        "park" | "unload" => Ok(req::MotionRequest::park()),
        _ => (|| -> Result<req::MotionRequest, ApiError> {
            let speed_scale = crate::motion::num(ctx.value, "speed_scale").map_err(ArmError::from)?;
            let target = positions(ctx.value.get("positions"))?;
            Ok(req::MotionRequest::jog(target, None, speed_scale))
        })(),
    };
    let request = match parsed {
        Ok(r) => r,
        Err(error) => {
            return Some(OwnerReply::PresetReject {
                epoch: ctx.inner.safety.epoch_id(),
                name,
                error,
            });
        }
    };
    match req::preflight_motion_request_at(ctx.inner, ctx.stamp, &request, Some(ctx.epoch), ctx.received_at) {
        Ok(snapshot) => {
            let reply_kind = MotionReplyKind::Preset { name };
            *ctx.pending_motion = Some(PendingMotion { reply_kind, request, snapshot, epoch: ctx.epoch, generation: ctx.local_generation });
            None
        }
        Err(e) => Some(OwnerReply::PresetReject {
            epoch: ctx.inner.safety.epoch_id(),
            name,
            error: e.into(),
        }),
    }
}

fn dispatch_motion_command(ctx: &mut OwnerDispatchCtx<'_>) -> Option<OwnerReply> {
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
            Ok(started) => OwnerReply::CommandAck {
                snap: AckSnapshot::from_inner(ctx.inner),
                started,
            },
            Err(error) => OwnerReply::CommandReject {
                epoch: ctx.inner.safety.epoch_id(),
                error,
            },
        });
    }
    let request = match parse_motion_command(ctx.path, &ctx.value["body"]) {
        Ok(r) => r,
        Err(error) => {
            return Some(OwnerReply::CommandReject {
                epoch: ctx.inner.safety.epoch_id(),
                error,
            });
        }
    };
    match req::preflight_motion_request_at(ctx.inner, ctx.stamp, &request, Some(ctx.epoch), ctx.received_at) {
        Ok(snapshot) => {
            let reply_kind = MotionReplyKind::Command { request_id: ctx.value["request_id"].clone() };
            *ctx.pending_motion = Some(PendingMotion { reply_kind, request, snapshot, epoch: ctx.epoch, generation: ctx.local_generation });
            None
        }
        Err(e) => Some(OwnerReply::CommandReject {
            epoch: ctx.inner.safety.epoch_id(),
            error: e.into(),
        }),
    }
}

/// Complete an off-lock jog/home/park/preset preparation and install under a short write lock.
/// Retries off-lock on `StartDrifted` only (bounded), with zero in-lock planning fallback.
/// Releases the write guard BEFORE formatting any JSON reply while keeping [`CommandReplyGateGuard`]
/// active until the caller enqueues the reply on `ordinary_tx` so terminals cannot overtake ACK.
#[allow(clippy::too_many_arguments)]
pub(super) async fn complete_pending_motion(
    pending: PendingMotion,
    stamp: crate::supervisor::AdmissionStamp,
    recv_supervisor: &Arc<ArmSupervisor>,
    recv_connection: &str,
    recv_generation: &AtomicU64,
    worker_closing: &Arc<AtomicBool>,
    command_reply_pending: &Arc<AtomicBool>,
    worker_tasks: &ConnectionTasks,
) -> Option<(Option<Value>, Option<CommandReplyGateGuard>)> {
    enum InstallStep {
        Closed,
        Detached,
        LeaseLost { epoch: u64 },
        Reject { epoch: u64, error: ApiError },
        Ack { snap: AckSnapshot, started: Started },
        DeferredParkAck {
            snap: AckSnapshot,
            started: Started,
            action_id: u64,
            deadline: Instant,
            notify: Arc<tokio::sync::Notify>,
        },
        RetryDrift(req::MotionPlanSnapshot),
    }

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
                let epoch_id = recv_supervisor.inner().read().await.safety.epoch_id();
                let reply = format_motion_reject_at(epoch_id, &reply_kind, ArmError::Internal(format!("motion planner failed: {e}")).into());
                return Some((Some(reply), None));
            }
        };

        // Keep per-connection reply gate closed from BEFORE acquiring the install lock until
        // the caller enqueues the ACK/reply on `ordinary_tx` so no terminal can overtake ACK.
        let gate = CommandReplyGateGuard::enter(command_reply_pending, recv_supervisor.hub());
        let step = {
            let mut inner = recv_supervisor.inner().write().await;
            if worker_closing.load(Ordering::SeqCst) {
                InstallStep::Closed
            } else if inner.detached {
                InstallStep::Detached
            } else {
                if let Some(hook) = inner.before_install_hook.take() { hook(&mut inner); }
                if recv_generation.load(Ordering::SeqCst) != generation || !inner.validate_servo_lease(recv_connection, generation) {
                    InstallStep::LeaseLost { epoch: inner.safety.epoch_id() }
                } else {
                    match prepared {
                        Err(e) => {
                            let epoch_id = inner.safety.epoch_id();
                            let err = if Instant::now() > snapshot.authority.deadline {
                                req::MotionInstallError::DeadlineExpired.into_arm_error(&inner, request.action_kind, Some(epoch))
                            } else if let Err(err) = inner.preflight_admission(stamp) {
                                ArmError::from(err)
                            } else {
                                e
                            };
                            InstallStep::Reject { epoch: epoch_id, error: err.into() }
                        }
                        Ok(plan) => {
                            let installed = match request.kind {
                                req::MotionKind::Jog => req::install_jog(&mut inner, stamp, &plan.slot, plan.heap_traj, Some(epoch), false),
                                req::MotionKind::Home => req::install_home(&mut inner, stamp, &plan.slot, plan.heap_traj, Some(epoch), false),
                                req::MotionKind::Park => req::install_park(&mut inner, stamp, &plan.slot, plan.heap_traj, Some(epoch), false),
                            };
                            match installed {
                                Ok(req::MotionInstalled::Started { action_id, expected_duration_s, .. }) => {
                                    InstallStep::Ack {
                                        snap: AckSnapshot::from_inner(&inner),
                                        started: Some((action_id, expected_duration_s)),
                                    }
                                }
                                Ok(req::MotionInstalled::DeferredContinuation { action_id, expected_duration_s, .. }) => {
                                    let notify = Arc::clone(&inner.park_continuation_notify);
                                    let deadline = inner.park_continuation.map_or(snapshot.authority.deadline, |c| c.deadline);
                                    InstallStep::DeferredParkAck {
                                        snap: AckSnapshot::from_inner(&inner),
                                        started: Some((action_id, expected_duration_s)),
                                        action_id,
                                        deadline,
                                        notify,
                                    }
                                }
                                Err(req::MotionInstallError::StartDrifted) if attempt + 1 < MAX_START_DRIFT_RETRIES => {
                                    if Instant::now() > snapshot.authority.deadline {
                                        InstallStep::Reject {
                                            epoch: inner.safety.epoch_id(),
                                            error: req::MotionInstallError::DeadlineExpired.into_arm_error(&inner, request.action_kind, Some(epoch)).into(),
                                        }
                                    } else if let Some(next) = req::MotionPlanSnapshot::refresh_for_drift_retry(&inner, &snapshot) {
                                        InstallStep::RetryDrift(next)
                                    } else {
                                        InstallStep::Reject {
                                            epoch: inner.safety.epoch_id(),
                                            error: req::MotionInstallError::PlanStale.into_arm_error(&inner, request.action_kind, Some(epoch)).into(),
                                        }
                                    }
                                }
                                Err(err) => InstallStep::Reject {
                                    epoch: inner.safety.epoch_id(),
                                    error: err.into_arm_error(&inner, request.action_kind, Some(epoch)).into(),
                                },
                            }
                        }
                    }
                }
            }
        };
        // Write guard is now released; build every JSON reply strictly off-lock.
        match step {
            InstallStep::Closed => return None,
            InstallStep::Detached => {
                drop(gate);
                let ep = recv_supervisor.hub().latest().map(|s| s.obs.epoch_id);
                let reply = format_status_reject(&reply_kind, 503, "OWNER_TRANSFERRED", "Ownership transferred to RT executor while planning", ep);
                return Some((Some(reply), None));
            }
            InstallStep::LeaseLost { epoch: epoch_id } => {
                drop(gate);
                let reply = format_status_reject(&reply_kind, 409, "LEASE_LOST", "This connection lost the lease while planning", Some(epoch_id));
                return Some((Some(reply), None));
            }
            InstallStep::Reject { epoch: epoch_id, error } => {
                drop(gate);
                let reply = format_motion_reject_at(epoch_id, &reply_kind, error);
                return Some((Some(reply), None));
            }
            InstallStep::Ack { snap, started } => {
                let ack = format_motion_ack_from_snap(&snap, &reply_kind, started);
                return Some((Some(ack), Some(gate)));
            }
            InstallStep::DeferredParkAck { snap, started, action_id, deadline, notify } => {
                spawn_park_continuation(
                    action_id, epoch, stamp.stop_gen, deadline, recv_connection.to_string(), notify, recv_supervisor.clone(), worker_closing.clone(), worker_tasks,
                );
                let ack = format_motion_ack_from_snap(&snap, &reply_kind, started);
                return Some((Some(ack), Some(gate)));
            }
            InstallStep::RetryDrift(next) => {
                drop(gate);
                snapshot = next;
                continue;
            }
        }
    }
    let (epoch_id, err) = {
        let inner = recv_supervisor.inner().read().await;
        (inner.safety.epoch_id(), req::MotionInstallError::PlanStale.into_arm_error(&inner, request.action_kind, Some(epoch)))
    };
    Some((Some(format_motion_reject_at(epoch_id, &reply_kind, err.into())), None))
}

#[allow(clippy::too_many_arguments)]
fn spawn_park_continuation(
    action_id: u64,
    epoch: u64,
    request_stop_gen: u64,
    deadline: Instant,
    connection_id: String,
    notify: Arc<tokio::sync::Notify>,
    supervisor: Arc<ArmSupervisor>,
    closing: Arc<AtomicBool>,
    worker_tasks: &ConnectionTasks,
) {
    worker_tasks.spawn(async move {
        let mut retries = 0usize;
        loop {
            let disconnected = closing.load(Ordering::Acquire);
            let expired = Instant::now() > deadline;
            if disconnected || expired {
                let mut inner = supervisor.inner().write().await;
                if !inner.detached {
                    let (code, text) = if expired {
                        ("DEADLINE_EXPIRED", "Park continuation exceeded original request deadline")
                    } else {
                        ("UNLOAD_FAILED", "Park continuation disconnected")
                    };
                    req::fail_park_continuation(&mut inner, action_id, code, text);
                }
                return;
            }
            let notified = notify.notified();
            let ready_snapshot = {
                let inner = supervisor.inner().read().await;
                if inner.detached || inner.current_stop_gen() != request_stop_gen { return; }
                let Some(cont) = inner.park_continuation.filter(|c| c.action_id == action_id) else { return; };
                if cont.phase == crate::supervisor::ParkContinuationPhase::ReadyForPlan {
                    let mut snap = req::MotionPlanSnapshot::from_supervisor(&inner);
                    snap.authority.stop_generation = request_stop_gen;
                    snap.authority.deadline = cont.deadline;
                    Some(snap)
                } else {
                    None
                }
            };
            let Some(snap) = ready_snapshot else {
                let wait = deadline.saturating_duration_since(Instant::now()).min(Duration::from_millis(50));
                if !wait.is_zero() {
                    let _ = tokio::time::timeout(wait, notified).await;
                }
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
            let disconnected = closing.load(Ordering::Acquire);
            let expired = Instant::now() > deadline;
            if disconnected || expired {
                let (code, text) = if expired {
                    ("DEADLINE_EXPIRED", "Park continuation exceeded original request deadline before install")
                } else {
                    ("UNLOAD_FAILED", "Park continuation disconnected before install")
                };
                req::fail_park_continuation(&mut inner, action_id, code, text);
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
