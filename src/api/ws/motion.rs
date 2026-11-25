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
    command_path: &'static str,
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
            *ctx.pending_motion = Some(PendingMotion { command_path: "preset", reply_kind, request, snapshot, epoch: ctx.epoch, generation: ctx.local_generation });
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
    let command_path: &'static str = match ctx.path {
        "/api/v1/motion/jog" => "/api/v1/motion/jog",
        "/api/v1/motion/home" => "/api/v1/motion/home",
        "/api/v1/motion/unload" => "/api/v1/motion/unload",
        _ => "command",
    };
    match req::preflight_motion_request_at(ctx.inner, ctx.stamp, &request, Some(ctx.epoch), ctx.received_at) {
        Ok(snapshot) => {
            let reply_kind = MotionReplyKind::Command { request_id: ctx.value["request_id"].clone() };
            *ctx.pending_motion = Some(PendingMotion { command_path, reply_kind, request, snapshot, epoch: ctx.epoch, generation: ctx.local_generation });
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

    let PendingMotion { command_path, reply_kind, request, mut snapshot, epoch, generation } = pending;
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
            inner.set_command(command_path);
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
                                    inner.set_action_id(action_id);
                                    InstallStep::Ack {
                                        snap: AckSnapshot::from_inner(&inner),
                                        started: Some((action_id, expected_duration_s)),
                                    }
                                }
                                Ok(req::MotionInstalled::DeferredContinuation { action_id, expected_duration_s, .. }) => {
                                    inner.set_action_id(action_id);
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
                inner.set_command("/api/v1/motion/unload");
                inner.set_action_id(action_id);
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
            inner.set_command("/api/v1/motion/unload");
            inner.set_action_id(action_id);
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

enum ExecutorMotionSubmitOutcome {
    Ack {
        action_id: u64,
        expected_duration_s: f64,
        ack_guard: crate::rt_core::reply_router::AckCommitGuard,
    },
    StartDrifted,
    Reject {
        epoch: u64,
        error: ArmError,
    },
}

async fn submit_prepared_motion_executor_ws(
    sup: &Arc<ArmSupervisor>,
    conn_id: u64,
    kind: req::MotionKind,
    plan: &req::PreparedMotionPlan,
    authority: crate::rt_core::broker::Authority,
    deadline: Instant,
    wire_tracker: Arc<crate::rt_core::reply_router::WsWireAckTracker>,
) -> ExecutorMotionSubmitOutcome {
    use crate::rt_core::broker::{AdmitError, ClientId, Reply, Submit};
    use crate::rt_core::ops::PlanSlot;
    use crate::rt_core::reply_router::{ExternalDestinationPolicy, ExternalSubmitError};
    use crate::rt_core::requests::actions::ActionRefusal;

    let cur_epoch = || {
        sup.hub()
            .latest()
            .map_or(authority.epoch, |s| s.rt.owner.authority.epoch)
    };
    let Some(handle) = sup.executor_handle() else {
        return ExecutorMotionSubmitOutcome::Reject {
            epoch: cur_epoch(),
            error: ArmError::Rejected {
                status: 503,
                code: "EXECUTOR_UNAVAILABLE".into(),
                message: "RT executor is not active".into(),
            },
        };
    };

    let client = ClientId(conn_id);
    let submit_deadline = deadline.max(Instant::now() + Duration::from_millis(50));
    let retry_until = Instant::now() + Duration::from_millis(50);

    let (_req_id, mut rx, ack_guard) = loop {
        let write = |dst: &mut PlanSlot| match kind {
            req::MotionKind::Jog => req::stage_jog(dst, &plan.slot),
            req::MotionKind::Home => req::stage_home(dst, &plan.slot),
            req::MotionKind::Park => req::stage_park(dst, &plan.slot),
        };
        let sub = match kind {
            req::MotionKind::Jog => Submit::Install(&write),
            req::MotionKind::Home => Submit::InstallHome(&write),
            req::MotionKind::Park => Submit::InstallPark(&write),
        };
        match handle.submit_external_motion(
            client,
            0,
            authority,
            submit_deadline,
            sub,
            ExternalDestinationPolicy::ConnectionOnly,
            true,
            Some(Arc::clone(&wire_tracker)),
        ) {
            Ok(triple) => break triple,
            Err(ExternalSubmitError::NoDestination) => {
                return ExecutorMotionSubmitOutcome::Reject {
                    epoch: cur_epoch(),
                    error: ArmError::Rejected {
                        status: 503,
                        code: "EXECUTOR_ROUTING_PENDING".into(),
                        message: "RT executor has no per-connection reply route bound".into(),
                    },
                };
            }
            Err(ExternalSubmitError::Stopped) => {
                return ExecutorMotionSubmitOutcome::Reject {
                    epoch: cur_epoch(),
                    error: ArmError::Rejected {
                        status: 503,
                        code: "EXECUTOR_UNAVAILABLE".into(),
                        message: "RT executor stopped before admitting motion".into(),
                    },
                };
            }
            Err(ExternalSubmitError::NotAnExternalAction) => {
                return ExecutorMotionSubmitOutcome::Reject {
                    epoch: cur_epoch(),
                    error: ArmError::Internal("motion command rejected as non-external".into()),
                };
            }
            Err(ExternalSubmitError::Admit(AdmitError::Disconnected)) => {
                return ExecutorMotionSubmitOutcome::Reject {
                    epoch: cur_epoch(),
                    error: ActionRefusal::new("LEASE_LOST").into_arm_error_with_epoch(authority.epoch),
                };
            }
            Err(ExternalSubmitError::Admit(AdmitError::RepliesFull)) => {
                handle.pump_broker_replies();
                if Instant::now() >= retry_until || Instant::now() > submit_deadline {
                    return ExecutorMotionSubmitOutcome::Reject {
                        epoch: cur_epoch(),
                        error: ActionRefusal::new("TERMINALS_BUSY")
                            .into_arm_error_with_epoch(authority.epoch),
                    };
                }
                tokio::time::sleep(Duration::from_millis(2)).await;
            }
            Err(ExternalSubmitError::Admit(AdmitError::CommandsFull | AdmitError::PlansFull)) => {
                handle.pump_broker_replies();
                let now = Instant::now();
                if now > submit_deadline {
                    return ExecutorMotionSubmitOutcome::Reject {
                        epoch: cur_epoch(),
                        error: ActionRefusal::new("DEADLINE_EXPIRED")
                            .into_arm_error_with_epoch(authority.epoch),
                    };
                }
                if now >= retry_until {
                    return ExecutorMotionSubmitOutcome::Reject {
                        epoch: cur_epoch(),
                        error: ArmError::Rejected {
                            status: 503,
                            code: "MOTION_BUSY".into(),
                            message: "RT executor command or plan queue is saturated".into(),
                        },
                    };
                }
                tokio::time::sleep(Duration::from_millis(2)).await;
            }
        }
    };

    let wait_limit = submit_deadline + Duration::from_secs(2);
    loop {
        handle.pump_broker_replies();
        let remaining = wait_limit.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            drop(ack_guard);
            return ExecutorMotionSubmitOutcome::Reject {
                epoch: cur_epoch(),
                error: ActionRefusal::new("DEADLINE_EXPIRED")
                    .into_arm_error_with_epoch(authority.epoch),
            };
        }
        let step = remaining.min(Duration::from_millis(2));
        match tokio::time::timeout(step, &mut rx).await {
            Ok(Ok(Ok(Reply::Ack { action: Some(id), .. }))) => {
                return ExecutorMotionSubmitOutcome::Ack {
                    action_id: id.0,
                    expected_duration_s: plan.slot.duration,
                    ack_guard,
                };
            }
            Ok(Ok(Ok(Reply::Ack { action: None, .. }))) => {
                drop(ack_guard);
                return ExecutorMotionSubmitOutcome::Reject {
                    epoch: cur_epoch(),
                    error: ArmError::Internal("motion install ACK omitted ActionId".into()),
                };
            }
            Ok(Ok(Ok(Reply::Refused { code: "START_DRIFTED", .. }))) => {
                drop(ack_guard);
                return ExecutorMotionSubmitOutcome::StartDrifted;
            }
            Ok(Ok(Ok(Reply::Refused { code, .. }))) => {
                drop(ack_guard);
                let ep = cur_epoch();
                return ExecutorMotionSubmitOutcome::Reject {
                    epoch: ep,
                    error: ActionRefusal::new(code).into_arm_error_with_epoch(ep),
                };
            }
            Ok(Ok(Ok(Reply::Terminal { .. } | Reply::Lease { .. }))) => {
                drop(ack_guard);
                return ExecutorMotionSubmitOutcome::Reject {
                    epoch: cur_epoch(),
                    error: ArmError::Internal("unexpected reply variant for motion install".into()),
                };
            }
            Ok(Ok(Err(_))) | Ok(Err(_)) => {
                drop(ack_guard);
                return ExecutorMotionSubmitOutcome::Reject {
                    epoch: cur_epoch(),
                    error: ArmError::Rejected {
                        status: 503,
                        code: "EXECUTOR_UNAVAILABLE".into(),
                        message: "RT executor stopped before acknowledging motion".into(),
                    },
                };
            }
            Err(_) => {}
        }
    }
}

/// Executor-owner WebSocket router for family M prepared motion commands:
/// - `/api/v1/motion/jog`, `/api/v1/motion/home`, `/api/v1/motion/unload`
/// - `preset` frames (`home`, `park`/`unload`, or explicit joint positions)
///
/// Returns `None` for `/api/v1/mode/gravity_comp` (unmigrated) and all non-M frames.
#[allow(clippy::too_many_arguments)]
pub(super) async fn handle_executor_ws_motion(
    frame_kind: FrameKind,
    entry: Option<&'static DispatchEntry>,
    value: &Value,
    stamp: crate::supervisor::AdmissionStamp,
    recv_conn_id: u64,
    recv_connection: &str,
    recv_generation: &Arc<AtomicU64>,
    recv_client_label: &Mutex<Option<String>>,
    last_ordinary_request: &mut Option<u64>,
    recv_supervisor: &Arc<ArmSupervisor>,
    worker_closing: &Arc<AtomicBool>,
    command_reply_pending: &Arc<AtomicBool>,
    wire_ack_tracker: &Arc<crate::rt_core::reply_router::WsWireAckTracker>,
) -> Option<actions::ExecutorActionDispatch> {
    let reply_kind = match frame_kind {
        FrameKind::Command => {
            let path = entry?.key.path?;
            if !matches!(
                path,
                "/api/v1/motion/jog" | "/api/v1/motion/home" | "/api/v1/motion/unload"
            ) {
                return None;
            }
            MotionReplyKind::Command {
                request_id: value.get("request_id").cloned().unwrap_or(Value::Null),
            }
        }
        FrameKind::Preset => MotionReplyKind::Preset {
            name: value
                .get("name")
                .and_then(Value::as_str)
                .unwrap_or("")
                .to_string(),
        },
        _ => return None,
    };

    let epoch = recv_supervisor
        .hub()
        .latest()
        .map_or_else(|| recv_supervisor.estop_epoch(), |s| s.obs.epoch_id);
    let immediate = |reply: Value| {
        Some(actions::ExecutorActionDispatch::Reply {
            reply,
            gate: None,
            ack_guard: None,
        })
    };

    // 1. Lease authority check (off-lock).
    let local_generation = recv_generation.load(Ordering::SeqCst);
    let supplied_generation = value.get("generation").and_then(Value::as_u64).unwrap_or(0);
    let label_matches = {
        let lbl = recv_client_label.lock().unwrap_or_else(|e| e.into_inner());
        value.get("client_id").is_none_or(|id| id.as_str() == lbl.as_deref())
    };
    if local_generation == 0 || supplied_generation != local_generation || !label_matches {
        return immediate(format_status_reject(
            &reply_kind,
            409,
            "LEASE_LOST",
            "This connection does not hold the active lease",
            Some(epoch),
        ));
    }

    // 2. Epoch check against live published state.
    if value.get("epoch_id").and_then(Value::as_u64) != Some(epoch) {
        return immediate(format_status_reject(
            &reply_kind,
            409,
            "EPOCH_MISMATCH",
            "Command epoch is no longer current",
            Some(epoch),
        ));
    }

    // 3. Monotonic request_id + object body check for Command frames.
    if frame_kind == FrameKind::Command {
        let request_id = value.get("request_id").and_then(Value::as_u64);
        if request_id.is_none() || last_ordinary_request.is_some_and(|last| request_id.unwrap() <= last)
        {
            return immediate(format_status_reject(
                &reply_kind,
                400,
                "INVALID_COMMAND",
                "request_id must be present and strictly increase",
                None,
            ));
        }
        *last_ordinary_request = request_id;
        if !value["body"].is_object() {
            return immediate(format_motion_reject_at(
                epoch,
                &reply_kind,
                invalid_command("Command body must be an object"),
            ));
        }
    }

    // 4. Parse request.
    let request = match frame_kind {
        FrameKind::Command => {
            let path = entry?.key.path?;
            match parse_motion_command(path, &value["body"]) {
                Ok(r) => r,
                Err(error) => return immediate(format_motion_reject_at(epoch, &reply_kind, error)),
            }
        }
        FrameKind::Preset => {
            let name = match &reply_kind {
                MotionReplyKind::Preset { name } => name.as_str(),
                _ => unreachable!(),
            };
            let parsed = match name {
                "home" => Ok(req::MotionRequest::home(None)),
                "park" | "unload" => Ok(req::MotionRequest::park()),
                _ => (|| -> Result<req::MotionRequest, ApiError> {
                    let speed_scale = crate::motion::num(value, "speed_scale").map_err(ArmError::from)?;
                    let target = positions(value.get("positions"))?;
                    Ok(req::MotionRequest::jog(target, None, speed_scale))
                })(),
            };
            match parsed {
                Ok(r) => r,
                Err(error) => return immediate(format_motion_reject_at(epoch, &reply_kind, error)),
            }
        }
        _ => unreachable!(),
    };

    let deadline = Instant::now() + req::DEFAULT_MOTION_PLAN_DEADLINE;
    for attempt in 0..MAX_START_DRIFT_RETRIES {
        let Some(latest) = recv_supervisor.hub().latest() else {
            return immediate(format_status_reject(
                &reply_kind,
                503,
                "EXECUTOR_UNAVAILABLE",
                "RT executor has not published an observation snapshot yet",
                Some(epoch),
            ));
        };
        let Some(motion_cfg) = recv_supervisor.hub().motion_config() else {
            return immediate(format_status_reject(
                &reply_kind,
                503,
                "EXECUTOR_UNAVAILABLE",
                "RT executor motion config snapshot is unavailable",
                Some(latest.obs.epoch_id),
            ));
        };
        if latest.rt.calibration_busy {
            return immediate(format_status_reject(
                &reply_kind,
                409,
                "CALIBRATION_BUSY",
                "Calibration is in progress",
                Some(latest.obs.epoch_id),
            ));
        }
        let Some(mut plan_snap) = req::MotionPlanSnapshot::from_published(&latest, motion_cfg) else {
            return immediate(format_motion_reject_at(
                latest.obs.epoch_id,
                &reply_kind,
                ArmError::Motion(crate::motion::MotionError::PlanStale(
                    "published state revision is updating; retry".into(),
                ))
                .into(),
            ));
        };
        plan_snap.authority.epoch = epoch;
        plan_snap.authority.lease_generation = local_generation;
        plan_snap.authority.stop_generation = stamp.stop_gen;
        plan_snap.authority.deadline = deadline;

        let config_revision = plan_snap.workspace_revision;
        let req_for_plan = request.clone();
        let cancel = Arc::clone(worker_closing);
        let plan_conn = recv_connection.to_string();
        let planned = tokio::task::spawn_blocking(move || {
            maybe_pause_blocking_plan(&plan_conn);
            if cancel.load(Ordering::Acquire) {
                return None;
            }
            let res = req::prepare_motion(&plan_snap, &req_for_plan);
            if cancel.load(Ordering::Acquire) {
                return None;
            }
            Some((plan_snap.authority, res))
        })
        .await;

        if worker_closing.load(Ordering::SeqCst) {
            return Some(actions::ExecutorActionDispatch::Closed);
        }
        let cur_epoch = recv_supervisor
            .hub()
            .latest()
            .map_or(epoch, |s| s.obs.epoch_id);
        if recv_supervisor.owner_mode() != crate::supervisor::OWNER_EXECUTOR {
            return immediate(format_status_reject(
                &reply_kind,
                503,
                "EXECUTOR_UNAVAILABLE",
                "RT executor is no longer active",
                Some(cur_epoch),
            ));
        }
        let (plan_auth, prepared) = match planned {
            Ok(None) => return Some(actions::ExecutorActionDispatch::Closed),
            Err(e) => {
                return immediate(format_motion_reject_at(
                    cur_epoch,
                    &reply_kind,
                    ArmError::Internal(format!("motion planner failed: {e}")).into(),
                ));
            }
            Ok(Some((auth, res))) => (auth, res),
        };
        if recv_generation.load(Ordering::SeqCst) != local_generation {
            return immediate(format_status_reject(
                &reply_kind,
                409,
                "LEASE_LOST",
                "This connection lost the lease while planning",
                Some(cur_epoch),
            ));
        }
        let plan = match prepared {
            Ok(p) => p,
            Err(e) => {
                return immediate(format_motion_reject_at(cur_epoch, &reply_kind, e.into()));
            }
        };

        let broker_auth = crate::rt_core::broker::Authority {
            lease_generation: local_generation,
            epoch: plan_auth.epoch,
            stop_generation: stamp.stop_gen,
            config_revision,
            parent_generation: 0,
        };
        let gate = CommandReplyGateGuard::enter_with_tracker(
            command_reply_pending,
            recv_supervisor.hub(),
            wire_ack_tracker,
        );
        match submit_prepared_motion_executor_ws(
            recv_supervisor,
            recv_conn_id,
            request.kind,
            &plan,
            broker_auth,
            deadline,
            Arc::clone(wire_ack_tracker),
        )
        .await
        {
            ExecutorMotionSubmitOutcome::Ack {
                action_id,
                expected_duration_s,
                ack_guard,
            } => {
                let ack_epoch = recv_supervisor
                    .hub()
                    .latest()
                    .map_or(cur_epoch, |s| s.obs.epoch_id);
                let reply = match &reply_kind {
                    MotionReplyKind::Command { request_id } => json!({
                        "type": "command_ack",
                        "request_id": request_id,
                        "epoch_id": ack_epoch,
                        "action_id": action_id,
                        "expected_duration_s": expected_duration_s,
                    }),
                    MotionReplyKind::Preset { name } => json!({
                        "type": "preset_ack",
                        "name": name,
                        "epoch_id": ack_epoch,
                        "action_id": action_id,
                    }),
                };
                return Some(actions::ExecutorActionDispatch::Reply {
                    reply,
                    gate: Some(gate),
                    ack_guard: Some(ack_guard),
                });
            }
            ExecutorMotionSubmitOutcome::StartDrifted
                if attempt + 1 < MAX_START_DRIFT_RETRIES && Instant::now() <= deadline =>
            {
                drop(gate);
                continue;
            }
            ExecutorMotionSubmitOutcome::StartDrifted => {
                drop(gate);
                let ep = recv_supervisor
                    .hub()
                    .latest()
                    .map_or(cur_epoch, |s| s.obs.epoch_id);
                return immediate(format_motion_reject_at(
                    ep,
                    &reply_kind,
                    ArmError::Motion(crate::motion::MotionError::PlanStale(
                        "arm position drifted while planning".into(),
                    ))
                    .into(),
                ));
            }
            ExecutorMotionSubmitOutcome::Reject { epoch: ep, error } => {
                drop(gate);
                return immediate(format_motion_reject_at(ep, &reply_kind, error.into()));
            }
        }
    }

    let ep = recv_supervisor
        .hub()
        .latest()
        .map_or(epoch, |s| s.obs.epoch_id);
    immediate(format_motion_reject_at(
        ep,
        &reply_kind,
        ArmError::Motion(crate::motion::MotionError::PlanStale(
            "arm position drifted across planning retries".into(),
        ))
        .into(),
    ))
}
