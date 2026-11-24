//! Calibration commands of the WS command path: job selection, busy codes and rejections (PR C / A8).
//!
//! Both `RtOwner::Legacy` and `RtOwner::Executor` route calibration commits through
//! [`crate::rt_core::requests::config`] with A1 [`AdmissionStamp`] authority rechecks.
//! Hardware motor-zero / MIT operations that require synchronous CAN bus I/O fail closed
//! with `503 EXECUTOR_CAPABILITY_UNSUPPORTED` on `RtOwner::Executor`.

#[allow(unused_imports)]
use super::*;
use super::dispatch::{
    AdmissionClass, AuthRequirement, DispatchEntry, DispatchHandler, DispatchKey,
    LeaseRequirement, OwnerDispatchCtx, ReplyPolicy,
};
use crate::model::HolderLabel;
use crate::rt_core::requests::config::{CalibBeginCheck, ConfigReply, ConfigRequest};
use crate::supervisor::AdmissionStamp;

pub(super) const ENTRIES: &[DispatchEntry] = &[
    DispatchEntry {
        key: DispatchKey::command("/api/v1/calibration/apply"),
        handler: DispatchHandler::Owner(dispatch_calibration_entry),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::Operator(None),
        reply: ReplyPolicy::CommandDeferred,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/calibration/restore"),
        handler: DispatchHandler::Owner(dispatch_calibration_entry),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::Operator(None),
        reply: ReplyPolicy::CommandDeferred,
    },
    DispatchEntry {
        key: DispatchKey::command(crate::motor_zero::PATH),
        handler: DispatchHandler::Owner(dispatch_calibration_entry),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::Operator(None),
        reply: ReplyPolicy::CommandDeferred,
    },
    DispatchEntry {
        key: DispatchKey::command(crate::motor_zero::RESTORE_PATH),
        handler: DispatchHandler::Owner(dispatch_calibration_entry),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::Operator(None),
        reply: ReplyPolicy::CommandDeferred,
    },
    DispatchEntry {
        key: DispatchKey::command(crate::mit::TIMEOUT_PATH),
        handler: DispatchHandler::Owner(dispatch_calibration_entry),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::Operator(None),
        reply: ReplyPolicy::CommandImmediate,
    },
    DispatchEntry {
        key: DispatchKey::command(crate::mit::QUALIFICATION_PATH),
        handler: DispatchHandler::Owner(dispatch_calibration_entry),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::Operator(None),
        reply: ReplyPolicy::CommandDeferred,
    },
];

/// Returns true if `path` is one of the WS calibration / motor-zero / MIT endpoints.
pub(super) fn is_calibration_path(path: &str) -> bool {
    ENTRIES.iter().any(|e| e.key.path == Some(path))
}

fn dispatch_calibration_entry(ctx: &mut OwnerDispatchCtx<'_>) -> Option<Value> {
    dispatch_calibration_command(
        ctx.inner,
        ctx.path,
        ctx.value,
        ctx.local_generation,
        ctx.epoch,
        ctx.pending_calibration,
    )
}

/// A calibration command checked under the lock or off-thread on the Executor.
pub(super) enum CalibJob {
    Apply(Box<crate::calibration::ApplyRequest>),
    Restore(Value),
    MotorZero(crate::motor_zero::MotorZeroRequest),
    JointZeroRestore(String),
    MitQualification(Box<crate::calibration::CalibrationFile>),
}

/// What a checked calibration command does: a task (job, busy flag set) or a reply
/// immediately (motor zero dry run, MIT timeout).
pub(super) enum CalibStart {
    Job(CalibJob, bool),
    Reply(Value),
}

/// Paths refused while a calibration transaction runs: motion (MOTION_BUSY) and configuration
/// writes (CONFIG_BUSY). Stop, torque, clear_fault and everything else go through.
pub(super) fn calibration_busy_code(inner: &SupervisorInner, path: &str) -> Option<&'static str> {
    if !inner.calibration_busy || path == "/api/v1/motion/stop" {
        return None;
    }
    if inner.motor_zero_busy && matches!(path, "/api/v1/torque" | "/api/v1/arm" | "/api/v1/safety/clear_fault") {
        return Some("MOTION_BUSY");
    }
    if path.starts_with("/api/v1/action/")
        || path.starts_with("/api/v1/motion/")
        || path.starts_with("/api/v1/mode/")
    {
        return Some("MOTION_BUSY");
    }
    if path.starts_with("/api/v1/calibration/")
        || matches!(
            path,
            "/api/v1/grasp/setup"
                | "/api/v1/poses/save"
                | "/api/v1/poses/rename"
                | "/api/v1/poses/delete"
                | "/api/v1/sequences/save"
                | "/api/v1/sequences/delete"
                | "/api/v1/safety/collision"
                | "/api/v1/workspace/keepout"
                | "/api/v1/gripper/calibration"
        )
    {
        return Some("CONFIG_BUSY");
    }
    None
}

/// Under the lock (Legacy): parse and check a calibration command; a real apply / restore marks the
/// transaction running (`calibration_busy`, cleared by its task). Returns (job, busy set).
pub(super) fn calibration_job(
    inner: &mut SupervisorInner,
    path: &str,
    body: &Value,
) -> Result<CalibStart, crate::calibration::CalibError> {
    if path == crate::motor_zero::PATH {
        let (req, dry) = crate::motor_zero::check(inner, body)?;
        if let Some(reply) = dry {
            return Ok(CalibStart::Reply(reply));
        }
        let token = crate::rt_core::requests::config::next_calib_job_token();
        inner.calibration_busy = true;
        inner.calibration_busy_token = Some(token);
        inner.motor_zero_busy = true;
        return Ok(CalibStart::Job(CalibJob::MotorZero(req), true));
    }
    if path == crate::mit::TIMEOUT_PATH {
        // register reads / RAM write of a few motors with torque off, under the lock
        return inner.mit_timeout_command(body).map(CalibStart::Reply);
    }
    if path == crate::mit::QUALIFICATION_PATH {
        let file = inner.mit_qualification_check(body)?;
        let token = crate::rt_core::requests::config::next_calib_job_token();
        inner.calibration_busy = true;
        inner.calibration_busy_token = Some(token);
        return Ok(CalibStart::Job(CalibJob::MitQualification(Box::new(file)), true));
    }
    if path == crate::motor_zero::RESTORE_PATH {
        let name = crate::motor_zero::check_restore(inner, body)?;
        let token = crate::rt_core::requests::config::next_calib_job_token();
        inner.calibration_busy = true;
        inner.calibration_busy_token = Some(token);
        inner.motor_zero_busy = true;
        return Ok(CalibStart::Job(CalibJob::JointZeroRestore(name), true));
    }
    crate::calibration::begin_checks(inner)?;
    let job = if path == "/api/v1/calibration/apply" {
        let req = crate::calibration::parse_apply(body)?;
        if !req.dry_run && req.kinematics.is_some() {
            crate::calibration::kinematics_armed(inner)?;
        }
        CalibJob::Apply(Box::new(req))
    } else {
        if !body.is_object() {
            return Err(crate::calibration::CalibError::new(
                400,
                "INVALID_COMMAND",
                "Command body must be an object",
            ));
        }
        CalibJob::Restore(body.clone())
    };
    let busy = !matches!(&job, CalibJob::Apply(r) if r.dry_run);
    if busy {
        let token = crate::rt_core::requests::config::next_calib_job_token();
        inner.calibration_busy = true;
        inner.calibration_busy_token = Some(token);
    }
    Ok(CalibStart::Job(job, busy))
}

pub(super) fn calibration_reject(request_id: &Value, e: &crate::calibration::CalibError) -> Value {
    let mut reply = json!({"type":"command_reject", "request_id":request_id});
    for (k, v) in e.reply_fields() {
        reply[k] = v;
    }
    reply
}

/// Dispatch a calibration command under the supervisor write lock (Legacy owner).
#[allow(clippy::too_many_arguments)]
pub(super) fn dispatch_calibration_command(
    inner: &mut SupervisorInner,
    path: &str,
    value: &Value,
    local_generation: u64,
    epoch: u64,
    pending_calibration: &mut Option<(Value, CalibJob, u64, u64, bool)>,
) -> Option<Value> {
    match calibration_job(inner, path, &value["body"]) {
        Ok(CalibStart::Job(job, busy)) => {
            *pending_calibration = Some((value["request_id"].clone(), job, local_generation, epoch, busy));
            None
        }
        Ok(CalibStart::Reply(reply)) => Some(fields_ack(&value["request_id"], reply)),
        Err(e) => Some(calibration_reject(&value["request_id"], &e)),
    }
}

/// Dispatch a calibration / motor-zero / MIT WS command on `RtOwner::Executor` without
/// touching `SupervisorLock`.
#[allow(clippy::too_many_arguments)]
pub(super) async fn dispatch_executor_calibration_command(
    path: &str,
    value: &Value,
    stamp: AdmissionStamp,
    conn_id: u64,
    local_generation: u64,
    epoch: u64,
    recv_supervisor: &Arc<ArmSupervisor>,
    recv_calib: &crate::calibration::CalibrationDeps,
    recv_connection: &str,
    recv_generation: &Arc<AtomicU64>,
    ack_tx: &mpsc::Sender<Value>,
    ws_auth: &WsAuth,
    worker_tasks: &ConnectionTasks,
) -> Option<Value> {
    let request_id = value["request_id"].clone();
    let body = &value["body"];

    if path == crate::mit::TIMEOUT_PATH || path == crate::mit::QUALIFICATION_PATH {
        let err = crate::calibration::CalibError::new(
            503,
            "EXECUTOR_CAPABILITY_UNSUPPORTED",
            "MIT register I/O and qualification are not supported on the RT executor",
        );
        return Some(calibration_reject(&request_id, &err));
    }

    if path == crate::motor_zero::PATH {
        let pub_cfg = match recv_supervisor.published_config().await {
            Ok(c) => c,
            Err(e) => {
                return Some(calibration_reject(
                    &request_id,
                    &crate::calibration::CalibError::new(503, "CONFIG_UNAVAILABLE", e.to_string()),
                ));
            }
        };
        let Some(snap) = recv_supervisor.hub().latest() else {
            return Some(calibration_reject(
                &request_id,
                &crate::calibration::CalibError::new(
                    503,
                    "OBSERVATION_UNAVAILABLE",
                    "No published observation available",
                ),
            ));
        };
        let (req, dry) = match crate::motor_zero::check_executor(&pub_cfg, &snap.obs, body) {
            Ok(v) => v,
            Err(e) => return Some(calibration_reject(&request_id, &e)),
        };
        if let Some(reply) = dry {
            return Some(fields_ack(&request_id, reply));
        }
        let guard = match acquire_calibration_busy_owner(recv_supervisor, true, None).await {
            Ok(g) => g,
            Err(e) => return Some(calibration_reject(&request_id, &e)),
        };
        spawn_calibration_job_with_guard(
            request_id,
            CalibJob::MotorZero(req),
            local_generation,
            epoch,
            Some(guard),
            stamp,
            conn_id,
            recv_supervisor,
            recv_calib,
            recv_connection,
            recv_generation,
            ack_tx,
            ws_auth,
            worker_tasks,
        );
        return None;
    }

    if path == crate::motor_zero::RESTORE_PATH {
        let pub_cfg = match recv_supervisor.published_config().await {
            Ok(c) => c,
            Err(e) => {
                return Some(calibration_reject(
                    &request_id,
                    &crate::calibration::CalibError::new(503, "CONFIG_UNAVAILABLE", e.to_string()),
                ));
            }
        };
        let Some(snap) = recv_supervisor.hub().latest() else {
            return Some(calibration_reject(
                &request_id,
                &crate::calibration::CalibError::new(
                    503,
                    "OBSERVATION_UNAVAILABLE",
                    "No published observation available",
                ),
            ));
        };
        let name = match crate::motor_zero::check_restore_executor(&pub_cfg, &snap.obs, body) {
            Ok(n) => n,
            Err(e) => return Some(calibration_reject(&request_id, &e)),
        };
        let guard = match acquire_calibration_busy_owner(recv_supervisor, true, None).await {
            Ok(g) => g,
            Err(e) => return Some(calibration_reject(&request_id, &e)),
        };
        spawn_calibration_job_with_guard(
            request_id,
            CalibJob::JointZeroRestore(name),
            local_generation,
            epoch,
            Some(guard),
            stamp,
            conn_id,
            recv_supervisor,
            recv_calib,
            recv_connection,
            recv_generation,
            ack_tx,
            ws_auth,
            worker_tasks,
        );
        return None;
    }

    // `/api/v1/calibration/apply` or `/api/v1/calibration/restore`
    let (job, busy, check_kin_armed) = if path == "/api/v1/calibration/apply" {
        let req = match crate::calibration::parse_apply(body) {
            Ok(r) => r,
            Err(e) => return Some(calibration_reject(&request_id, &e)),
        };
        let busy = !req.dry_run;
        let check_kin = busy && req.kinematics.is_some();
        (CalibJob::Apply(Box::new(req)), busy, check_kin)
    } else {
        if !body.is_object() {
            let e = crate::calibration::CalibError::new(
                400,
                "INVALID_COMMAND",
                "Command body must be an object",
            );
            return Some(calibration_reject(&request_id, &e));
        }
        (CalibJob::Restore(body.clone()), true, false)
    };

    let guard = if busy {
        match acquire_calibration_busy_owner(
            recv_supervisor,
            false,
            Some(CalibBeginCheck {
                check_grasp: true,
                check_kinematics_armed: check_kin_armed,
            }),
        )
        .await
        {
            Ok(g) => Some(g),
            Err(e) => return Some(calibration_reject(&request_id, &e)),
        }
    } else {
        // Dry-run still requires grasp setup & non-busy/idle checks via PublishedConfig.
        let pub_cfg = match recv_supervisor.published_config().await {
            Ok(c) => c,
            Err(e) => {
                return Some(calibration_reject(
                    &request_id,
                    &crate::calibration::CalibError::new(503, "CONFIG_UNAVAILABLE", e.to_string()),
                ));
            }
        };
        if pub_cfg.calibration_busy {
            return Some(calibration_reject(
                &request_id,
                &crate::calibration::CalibError::new(
                    409,
                    "CONFIG_BUSY",
                    "a calibration apply or restore is running",
                ),
            ));
        }
        if pub_cfg.grasp_setup().is_none() {
            return Some(calibration_reject(
                &request_id,
                &crate::calibration::CalibError::new(400, "UNSUPPORTED", "This profile has no grasp setup"),
            ));
        }
        None
    };

    spawn_calibration_job_with_guard(
        request_id,
        job,
        local_generation,
        epoch,
        guard,
        stamp,
        conn_id,
        recv_supervisor,
        recv_calib,
        recv_connection,
        recv_generation,
        ack_tx,
        ws_auth,
        worker_tasks,
    );
    None
}

async fn acquire_calibration_busy_owner(
    sup: &Arc<ArmSupervisor>,
    motor_zero: bool,
    check_begin: Option<CalibBeginCheck>,
) -> Result<crate::calibration::BusyGuard, crate::calibration::CalibError> {
    let job_token = crate::rt_core::requests::config::next_calib_job_token();
    let sup_w = Arc::clone(sup);
    let (tx, rx) = tokio::sync::oneshot::channel();
    // Run acquisition inside a detached task so caller cancellation between enqueue and reply
    // still constructs and drops `BusyGuard` cleanly, never stranding `calibration_busy`.
    tokio::spawn(async move {
        let res = match sup_w
            .dispatch_config_request(ConfigRequest::SetCalibrationBusy {
                busy: true,
                motor_zero,
                job_token,
                check_begin,
            })
            .await
        {
            Ok(ConfigReply::CalibrationBusySet(Ok(_))) => {
                sup_w
                    .published_config_hub()
                    .update(|cfg| cfg.calibration_busy = true);
                Ok(crate::calibration::BusyGuard::new_with_token(sup_w, job_token))
            }
            Ok(ConfigReply::CalibrationBusySet(Err(e))) => Err(e.to_calib_error()),
            Err(e) => Err(e.to_calib_error()),
            _ => Err(crate::calibration::CalibError::new(
                500,
                "INTERNAL",
                "unexpected calibration busy reply",
            )),
        };
        let _ = tx.send(res);
    });
    match rx.await {
        Ok(res) => res,
        Err(_) => Err(crate::calibration::CalibError::new(
            500,
            "INTERNAL",
            "calibration busy acquisition task aborted",
        )),
    }
}

/// Spawn a tracked calibration job with owner-phase lease/auth/epoch checks and completion reply.
#[allow(clippy::too_many_arguments)]
pub(super) fn spawn_calibration_job(
    request_id: Value,
    job: CalibJob,
    generation: u64,
    epoch: u64,
    busy: bool,
    stamp: AdmissionStamp,
    conn_id: u64,
    recv_supervisor: &Arc<ArmSupervisor>,
    recv_calib: &crate::calibration::CalibrationDeps,
    recv_connection: &str,
    recv_generation: &Arc<AtomicU64>,
    ack_tx: &mpsc::Sender<Value>,
    ws_auth: &WsAuth,
    worker_tasks: &ConnectionTasks,
) {
    let guard = busy.then(|| crate::calibration::BusyGuard::new(recv_supervisor.clone()));
    spawn_calibration_job_with_guard(
        request_id,
        job,
        generation,
        epoch,
        guard,
        stamp,
        conn_id,
        recv_supervisor,
        recv_calib,
        recv_connection,
        recv_generation,
        ack_tx,
        ws_auth,
        worker_tasks,
    );
}

#[allow(clippy::too_many_arguments)]
fn spawn_calibration_job_with_guard(
    request_id: Value,
    job: CalibJob,
    generation: u64,
    epoch: u64,
    guard: Option<crate::calibration::BusyGuard>,
    mut stamp: AdmissionStamp,
    conn_id: u64,
    recv_supervisor: &Arc<ArmSupervisor>,
    recv_calib: &crate::calibration::CalibrationDeps,
    recv_connection: &str,
    recv_generation: &Arc<AtomicU64>,
    ack_tx: &mpsc::Sender<Value>,
    ws_auth: &WsAuth,
    worker_tasks: &ConnectionTasks,
) {
    stamp.connection_id = conn_id;
    stamp.lease_gen = generation;
    stamp.estop_epoch = epoch;
    let job_token = guard.as_ref().map(|g| g.job_token()).filter(|&t| t != 0);
    let ingress_deadline = std::time::Instant::now() + crate::supervisor::config::CONFIG_PROPOSAL_BUDGET;

    let (sup, calib, conn, gen_now, tx) = (
        recv_supervisor.clone(),
        recv_calib.clone(),
        recv_connection.to_string(),
        recv_generation.clone(),
        ack_tx.clone(),
    );
    let ws_auth_c = ws_auth.clone();
    let ws_auth_off = ws_auth.clone();
    let gen_now_off = recv_generation.clone();
    let conn_holder = HolderLabel::new(recv_connection);
    let tasks_c = worker_tasks.clone();
    worker_tasks.spawn(async move {
        let check: crate::calibration::LeaseCheck = Box::new(move |inner: &mut SupervisorInner| {
            if !ws_auth_c.is_operator() {
                return Err(crate::calibration::CalibError::new(
                    403,
                    "OPERATOR_REQUIRED",
                    "Operator session expired or locked during calibration",
                ));
            }
            if gen_now.load(Ordering::SeqCst) != generation
                || !inner.validate_servo_lease(&conn, generation)
            {
                return Err(crate::calibration::CalibError::new(
                    409,
                    "LEASE_LOST",
                    "This connection lost the lease during the calibration",
                ));
            }
            if inner.safety.epoch_id() != epoch {
                return Err(crate::calibration::CalibError::new(
                    409,
                    "EPOCH_MISMATCH",
                    "The epoch changed during the calibration (stop or e-stop)",
                ));
            }
            Ok(())
        });
        let auth = crate::calibration::CalibCommitAuth {
            admission: stamp,
            lease_holder: Some((conn_holder, generation)),
            off_thread_check: Arc::new(move || {
                if !ws_auth_off.is_operator() {
                    return Err((
                        "OPERATOR_REQUIRED",
                        crate::calibration::CalibError::new(
                            403,
                            "OPERATOR_REQUIRED",
                            "Operator session expired or locked during calibration",
                        ),
                    ));
                }
                if gen_now_off.load(Ordering::SeqCst) != generation {
                    return Err((
                        "LEASE_LOST",
                        crate::calibration::CalibError::new(
                            409,
                            "LEASE_LOST",
                            "This connection lost the lease during the calibration",
                        ),
                    ));
                }
                Ok(())
            }),
            job_token,
            deadline: ingress_deadline,
        };
        let job_sup = sup.clone();
        let (job_tx, job_rx) = tokio::sync::oneshot::channel();
        let job_abort = tasks_c.spawn_abortable(async move {
            let res = match job {
                CalibJob::Apply(req) => {
                    crate::calibration::apply_with_auth(&job_sup, &calib, *req, &check, &auth).await
                }
                CalibJob::Restore(body) => {
                    crate::calibration::restore_with_auth(&job_sup, &calib, &body, &check, &auth).await
                }
                CalibJob::MotorZero(req) => {
                    crate::motor_zero::run_with_auth(&job_sup, &calib, req, &check, &auth).await
                }
                CalibJob::JointZeroRestore(name) => {
                    crate::motor_zero::restore_with_auth(&job_sup, &calib, name, &check, &auth).await
                }
                CalibJob::MitQualification(file) => {
                    crate::mit::store_qualification(&job_sup, &calib, *file, &check).await
                }
            };
            let _ = job_tx.send(res);
        });
        let _abort_inner = actions::AbortOnDrop(job_abort);
        let result = job_rx.await.unwrap_or_else(|e| {
            tracing::error!("calibration task failed: {e}");
            Err(crate::calibration::CalibError::new(
                500,
                "INTERNAL",
                format!("calibration task failed: {e}; check GET /api/v1/calibration and the backups"),
            ))
        });
        if let Some(g) = guard {
            g.release().await;
        }
        let reply = match result {
            Ok(v) => fields_ack(&request_id, v),
            Err(e) => calibration_reject(&request_id, &e),
        };
        let _ = tx.send(reply).await;
    });
}
