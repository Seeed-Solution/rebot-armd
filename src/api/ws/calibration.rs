//! Calibration commands of the WS command path: job selection, busy codes and rejections (track E).
//!
//! Moved from `api/ws.rs` without changes (armd 1.6 RT-core foundation); moved items, their
//! fields and inherent methods are `pub(super)` so the connection loop keeps using them.

#[allow(unused_imports)]
use super::*;
use super::dispatch::{
    AdmissionClass, AuthRequirement, DispatchEntry, DispatchHandler, DispatchKey,
    LeaseRequirement, OwnerDispatchCtx, OwnerReply, ReplyPolicy,
};

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

/// Returns true if `path` is one of the WS calibration / motor-zero / joint-zero / MIT endpoints.
pub(super) fn is_calibration_path(path: &str) -> bool {
    ENTRIES.iter().any(|e| e.key.path == Some(path))
}

/// Returns true if `path` (and its request body) is an out-of-scope hardware commissioning
/// operation that must still fail closed with `409 EXECUTOR_UNSUPPORTED` on the RT executor.
pub(super) async fn is_executor_calibration_unsupported(
    path: &str,
    body: &Value,
    sup: &Arc<ArmSupervisor>,
) -> bool {
    if path == crate::mit::TIMEOUT_PATH || path == crate::mit::QUALIFICATION_PATH {
        return true;
    }
    if path == crate::motor_zero::RESTORE_PATH {
        return sup
            .published_config()
            .await
            .map_or(true, |cfg| cfg.has_driver);
    }
    if path == crate::motor_zero::PATH {
        let Ok(pub_cfg) = sup.published_config().await else {
            return true;
        };
        if pub_cfg.has_driver {
            return true;
        }
        return match body.get("method") {
            Some(Value::String(m)) if m == "motor" => true,
            Some(Value::String(m)) if m == "software" => false,
            None | Some(Value::Null) => pub_cfg.calibration_tools().motor_zero.motor,
            // Invalid method values (non-string or unknown string) fall through to software
            // validation so they return the exact legacy `400 INVALID_COMMAND` reply.
            Some(_) => false,
        };
    }
    false
}

/// Formats a synchronous WS ingress calibration error from the RT owner, preserving the exact
/// legacy ingress lease-loss reply (`This connection does not own an active lease` + `epoch_id`)
/// when the connection's local generation was stale against an expired owner lease.
fn ingress_calib_reject(
    request_id: &Value,
    epoch: u64,
    err: crate::rt_core::requests::config::ConfigError,
) -> Value {
    use crate::rt_core::requests::config::ConfigError;
    match err {
        ConfigError::LeaseLost | ConfigError::ConnectionClosed => json!({
            "type": "command_reject",
            "request_id": request_id,
            "status": 409,
            "code": "LEASE_LOST",
            "error": "LEASE_LOST",
            "message": "This connection does not own an active lease",
            "epoch_id": epoch,
        }),
        other => calibration_reject(request_id, &crate::calibration::calib_error_from_config(other)),
    }
}

/// Forward a detached calibration worker's `JoinHandle<Value>` to `ack_tx`, ensuring a worker
/// panic or join failure still produces the legacy deferred `500 INTERNAL` reply while the WS
/// connection stays open.
fn forward_deferred_calib_job(
    worker_tasks: &ConnectionTasks,
    job: tokio::task::JoinHandle<Value>,
    request_id: Value,
    ack_tx: mpsc::Sender<Value>,
) {
    worker_tasks.spawn(async move {
        let reply = match job.await {
            Ok(reply) => reply,
            Err(e) => {
                tracing::error!("executor calibration task failed: {e}");
                calibration_reject(
                    &request_id,
                    &crate::calibration::CalibError::new(
                        500,
                        "INTERNAL",
                        "calibration task failed: channel closed; check GET /api/v1/calibration and the backups",
                    ),
                )
            }
        };
        let _ = ack_tx.send(reply).await;
    });
}

/// Executor-mode WS handler for in-scope persistent calibration commands:
/// `/api/v1/calibration/apply`, `/api/v1/calibration/restore`,
/// software `/api/v1/calibration/motor_zero`, and `/api/v1/calibration/joint_zero/restore`.
///
/// Preserves the exact legacy synchronous-reject / immediate-dry-run vs deferred `ack_tx`
/// split so wire replies remain byte-identical to legacy for the same inputs.
#[allow(clippy::too_many_arguments)]
pub(super) async fn handle_executor_calibration_ws(
    path: &str,
    request_id: Value,
    body: Value,
    generation: u64,
    epoch: u64,
    mut stamp: crate::supervisor::AdmissionStamp,
    conn_id: u64,
    recv_connection: &str,
    recv_generation: &Arc<AtomicU64>,
    ws_auth: &WsAuth,
    recv_supervisor: &Arc<ArmSupervisor>,
    recv_calib: &crate::calibration::CalibrationDeps,
    ack_tx: &mpsc::Sender<Value>,
    worker_tasks: &ConnectionTasks,
) -> Option<Value> {
    // Preserve `stamp.stop_gen` captured once at WS ingress (`ws.rs:2462`) so a Stop arriving
    // while this command was queued is never overtaken.
    stamp.connection_id = conn_id;
    stamp.lease_gen = generation;
    stamp.estop_epoch = epoch;

    let ws_auth_c = ws_auth.clone();
    let gen_now_c = recv_generation.clone();
    let auth_check: Arc<
        dyn Fn() -> Result<(), (&'static str, crate::supervisor::ArmError)> + Send + Sync,
    > = Arc::new(move || {
        if !ws_auth_c.is_operator() {
            return Err((
                "OPERATOR_REQUIRED",
                crate::supervisor::ArmError::Rejected {
                    status: 403,
                    code: "OPERATOR_REQUIRED".into(),
                    message: "Operator session expired or locked during calibration".into(),
                },
            ));
        }
        if gen_now_c.load(Ordering::SeqCst) != generation {
            return Err((
                "LEASE_LOST",
                crate::supervisor::ArmError::Rejected {
                    status: 409,
                    code: "LEASE_LOST".into(),
                    message: "This connection lost the lease during the calibration".into(),
                },
            ));
        }
        Ok(())
    });
    let mut auth = crate::calibration::ExecutorCalibAuth {
        admission: stamp,
        lease_holder: Some((crate::model::HolderLabel::new(recv_connection), generation)),
        operator_grant: ws_auth.operator_grant(),
        auth_check,
        txn_id: None,
    };

    let cfg_auth = crate::rt_core::requests::config::ConfigAuthContext {
        admission: auth.admission,
        lease_holder: auth.lease_holder,
    };

    if path == crate::motor_zero::PATH || path == crate::motor_zero::RESTORE_PATH {
        // Match legacy `ws.rs:2902` precedence authoritatively on RT: validate live owner lease,
        // epoch, stop_gen, and operator grant BEFORE checking `calibration_busy` and BEFORE body parsing.
        match recv_supervisor
            .dispatch_config_request(
                crate::rt_core::requests::config::ConfigRequest::BeginCalibrationTxn {
                    txn_id: 0,
                    auth: cfg_auth,
                    operator_grant: auth.operator_grant,
                    expected_epoch: auth.admission.estop_epoch,
                    expected_stop_gen: auth.admission.stop_gen,
                    kind: crate::rt_core::requests::config::CalibrationTxnKind::IngressBusyCheck,
                },
            )
            .await
        {
            Ok(crate::rt_core::requests::config::ConfigReply::CalibrationTxnBegan(Ok(_))) => {}
            Ok(crate::rt_core::requests::config::ConfigReply::CalibrationTxnBegan(Err(e))) => {
                return Some(ingress_calib_reject(&request_id, epoch, e));
            }
            Err(e) => {
                return Some(ingress_calib_reject(&request_id, epoch, e));
            }
            _ => {
                return Some(calibration_reject(
                    &request_id,
                    &crate::calibration::CalibError::new(
                        500,
                        "INTERNAL",
                        "unexpected ingress check reply",
                    ),
                ));
            }
        }
    }

    if path == crate::motor_zero::PATH {
        match crate::motor_zero::check_software_executor(recv_supervisor, &body, &auth).await {
            Ok(crate::motor_zero::ExecutorZeroStart::DryRunReply(reply)) => {
                return Some(fields_ack(&request_id, reply));
            }
            Ok(crate::motor_zero::ExecutorZeroStart::Run(req, pre_guard)) => {
                auth.txn_id = Some(pre_guard.txn_id());
                let (sup, calib, tx) = (recv_supervisor.clone(), recv_calib.clone(), ack_tx.clone());
                let req_id_job = request_id.clone();
                let job = tokio::spawn(async move {
                    match crate::motor_zero::run_software_executor_with_guard(
                        &sup,
                        &calib,
                        req,
                        auth,
                        Some(pre_guard),
                    )
                    .await
                    {
                        Ok(v) => fields_ack(&req_id_job, v),
                        Err(e) => calibration_reject(&req_id_job, &e),
                    }
                });
                forward_deferred_calib_job(worker_tasks, job, request_id, tx);
                return None;
            }
            Err(e) => {
                if e.code == "LEASE_LOST" {
                    return Some(ingress_calib_reject(
                        &request_id,
                        epoch,
                        crate::rt_core::requests::config::ConfigError::LeaseLost,
                    ));
                }
                return Some(calibration_reject(&request_id, &e));
            }
        }
    }

    if path == crate::motor_zero::RESTORE_PATH {
        match crate::motor_zero::check_restore_executor(recv_supervisor, &body, &auth).await {
            Ok((name, pre_guard)) => {
                auth.txn_id = Some(pre_guard.txn_id());
                let (sup, calib, tx) = (recv_supervisor.clone(), recv_calib.clone(), ack_tx.clone());
                let req_id_job = request_id.clone();
                let job = tokio::spawn(async move {
                    match crate::motor_zero::restore_executor_with_guard(
                        &sup,
                        &calib,
                        name,
                        auth,
                        Some(pre_guard),
                    )
                    .await
                    {
                        Ok(v) => fields_ack(&req_id_job, v),
                        Err(e) => calibration_reject(&req_id_job, &e),
                    }
                });
                forward_deferred_calib_job(worker_tasks, job, request_id, tx);
                return None;
            }
            Err(e) => {
                if e.code == "LEASE_LOST" {
                    return Some(ingress_calib_reject(
                        &request_id,
                        epoch,
                        crate::rt_core::requests::config::ConfigError::LeaseLost,
                    ));
                }
                return Some(calibration_reject(&request_id, &e));
            }
        }
    }

    // `/api/v1/calibration/apply` and `/api/v1/calibration/restore`:
    // Run authoritative RT `CameraPreflight` BEFORE body parsing to match legacy `begin_checks`
    // -> `parse_apply` / object check -> `kinematics_armed` precedence exactly.
    match recv_supervisor
        .dispatch_config_request(
            crate::rt_core::requests::config::ConfigRequest::BeginCalibrationTxn {
                txn_id: 0,
                auth: cfg_auth,
                operator_grant: auth.operator_grant,
                expected_epoch: auth.admission.estop_epoch,
                expected_stop_gen: auth.admission.stop_gen,
                kind: crate::rt_core::requests::config::CalibrationTxnKind::CameraPreflight,
            },
        )
        .await
    {
        Ok(crate::rt_core::requests::config::ConfigReply::CalibrationTxnBegan(Ok(_))) => {}
        Ok(crate::rt_core::requests::config::ConfigReply::CalibrationTxnBegan(Err(e))) => {
            return Some(ingress_calib_reject(&request_id, epoch, e));
        }
        Err(e) => {
            return Some(ingress_calib_reject(&request_id, epoch, e));
        }
        _ => {
            return Some(calibration_reject(
                &request_id,
                &crate::calibration::CalibError::new(500, "INTERNAL", "unexpected preflight reply"),
            ));
        }
    }

    if path == "/api/v1/calibration/apply" {
        let req = match crate::calibration::parse_apply(&body) {
            Ok(r) => r,
            Err(e) => return Some(calibration_reject(&request_id, &e)),
        };
        let txn_id = crate::rt_core::requests::config::next_calibration_txn_id();
        let mut pre_guard = (!req.dry_run)
            .then(|| crate::calibration::ExecutorTxnGuard::new(recv_supervisor.clone(), txn_id));
        match recv_supervisor
            .dispatch_config_request(
                crate::rt_core::requests::config::ConfigRequest::BeginCalibrationTxn {
                    txn_id,
                    auth: cfg_auth,
                    operator_grant: auth.operator_grant,
                    expected_epoch: auth.admission.estop_epoch,
                    expected_stop_gen: auth.admission.stop_gen,
                    kind: crate::rt_core::requests::config::CalibrationTxnKind::CameraApply {
                        require_kinematics_disarmed: !req.dry_run && req.kinematics.is_some(),
                        dry_run: req.dry_run,
                    },
                },
            )
            .await
        {
            Ok(crate::rt_core::requests::config::ConfigReply::CalibrationTxnBegan(Ok(_))) => {
                if !req.dry_run {
                    recv_supervisor.published_config_hub().update(|cfg| {
                        cfg.calibration_busy = true;
                        cfg.calibration_txn_id = Some(txn_id);
                    });
                    auth.txn_id = Some(txn_id);
                }
            }
            Ok(crate::rt_core::requests::config::ConfigReply::CalibrationTxnBegan(Err(e))) => {
                if let Some(g) = pre_guard.take() {
                    let _ = g.disarm();
                }
                return Some(ingress_calib_reject(&request_id, epoch, e));
            }
            Err(e) => {
                if let Some(g) = pre_guard.take() {
                    g.release().await;
                }
                return Some(ingress_calib_reject(&request_id, epoch, e));
            }
            _ => {
                if let Some(g) = pre_guard.take() {
                    g.release().await;
                }
                return Some(calibration_reject(
                    &request_id,
                    &crate::calibration::CalibError::new(500, "INTERNAL", "unexpected begin reply"),
                ));
            }
        }
        let (sup, calib, tx) = (recv_supervisor.clone(), recv_calib.clone(), ack_tx.clone());
        let req_id_job = request_id.clone();
        let job = tokio::spawn(async move {
            match crate::calibration::apply_executor_with_guard(
                &sup,
                &calib,
                req,
                auth,
                pre_guard,
            )
            .await
            {
                Ok(v) => fields_ack(&req_id_job, v),
                Err(e) => calibration_reject(&req_id_job, &e),
            }
        });
        forward_deferred_calib_job(worker_tasks, job, request_id, tx);
        return None;
    }

    // `/api/v1/calibration/restore`
    if !body.is_object() {
        return Some(calibration_reject(
            &request_id,
            &crate::calibration::CalibError::new(
                400,
                "INVALID_COMMAND",
                "Command body must be an object",
            ),
        ));
    }
    let txn_id = crate::rt_core::requests::config::next_calibration_txn_id();
    let pre_guard = crate::calibration::ExecutorTxnGuard::new(recv_supervisor.clone(), txn_id);
    match recv_supervisor
        .dispatch_config_request(
            crate::rt_core::requests::config::ConfigRequest::BeginCalibrationTxn {
                txn_id,
                auth: cfg_auth,
                operator_grant: auth.operator_grant,
                expected_epoch: auth.admission.estop_epoch,
                expected_stop_gen: auth.admission.stop_gen,
                kind: crate::rt_core::requests::config::CalibrationTxnKind::CameraRestore,
            },
        )
        .await
    {
        Ok(crate::rt_core::requests::config::ConfigReply::CalibrationTxnBegan(Ok(_))) => {
            recv_supervisor.published_config_hub().update(|cfg| {
                cfg.calibration_busy = true;
                cfg.calibration_txn_id = Some(txn_id);
            });
            auth.txn_id = Some(txn_id);
        }
        Ok(crate::rt_core::requests::config::ConfigReply::CalibrationTxnBegan(Err(e))) => {
            let _ = pre_guard.disarm();
            return Some(ingress_calib_reject(&request_id, epoch, e));
        }
        Err(e) => {
            pre_guard.release().await;
            return Some(ingress_calib_reject(&request_id, epoch, e));
        }
        _ => {
            pre_guard.release().await;
            return Some(calibration_reject(
                &request_id,
                &crate::calibration::CalibError::new(500, "INTERNAL", "unexpected begin reply"),
            ));
        }
    }
    let (sup, calib, tx) = (recv_supervisor.clone(), recv_calib.clone(), ack_tx.clone());
    let req_id_job = request_id.clone();
    let job = tokio::spawn(async move {
        match crate::calibration::restore_executor_with_guard(
            &sup,
            &calib,
            &body,
            auth,
            Some(pre_guard),
        )
        .await
        {
            Ok(v) => fields_ack(&req_id_job, v),
            Err(e) => calibration_reject(&req_id_job, &e),
        }
    });
    forward_deferred_calib_job(worker_tasks, job, request_id, tx);
    None
}

/// Fail-closed capability refusal for calibration, motor-zero, joint-zero, and MIT endpoints
/// while the RT executor owns the arm (never touches disk or live state).
pub(super) fn dispatch_executor_calibration_command(path: &str, value: &Value, epoch: u64) -> Value {
    json!({
        "type": "command_reject",
        "request_id": value["request_id"],
        "status": 409,
        "code": "EXECUTOR_UNSUPPORTED",
        "error": "EXECUTOR_UNSUPPORTED",
        "message": format!("{path} is not available while the RT executor owns the arm; use the legacy owner"),
        "epoch_id": epoch,
    })
}

fn dispatch_calibration_entry(ctx: &mut OwnerDispatchCtx<'_>) -> Option<OwnerReply> {
    dispatch_calibration_command(
        ctx.inner,
        ctx.path,
        ctx.value,
        ctx.local_generation,
        ctx.epoch,
        ctx.pending_calibration,
    )
}

/// A calibration command checked under the lock.
pub(super) enum CalibJob { Apply(Box<crate::calibration::ApplyRequest>), Restore(Value), MotorZero(crate::motor_zero::MotorZeroRequest), JointZeroRestore(String),
    MitQualification(Box<crate::calibration::CalibrationFile>) }

/// What a checked calibration command does: a task (job, busy flag set) or a reply under the
/// lock (motor zero dry run).
pub(super) enum CalibStart { Job(CalibJob, bool), Reply(Value) }

/// Paths refused while a calibration transaction runs: motion (MOTION_BUSY) and configuration
/// writes (CONFIG_BUSY). Stop, torque, clear_fault and everything else go through.
pub(super) fn calibration_busy_code(inner: &SupervisorInner, path: &str) -> Option<&'static str> {
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
pub(super) fn calibration_job(inner: &mut SupervisorInner, path: &str, body: &Value) -> Result<CalibStart, crate::calibration::CalibError> {
    if path == crate::motor_zero::PATH {
        let (req, dry) = crate::motor_zero::check(inner, body)?;
        if let Some(reply) = dry { return Ok(CalibStart::Reply(reply)); }
        inner.calibration_busy = true;
        inner.motor_zero_busy = true;
        return Ok(CalibStart::Job(CalibJob::MotorZero(req), true));
    }
    if path == crate::mit::TIMEOUT_PATH {
        // register reads / RAM write of a few motors with torque off, under the lock
        return inner.mit_timeout_command(body).map(CalibStart::Reply);
    }
    if path == crate::mit::QUALIFICATION_PATH {
        let file = inner.mit_qualification_check(body)?;
        inner.calibration_busy = true;
        return Ok(CalibStart::Job(CalibJob::MitQualification(Box::new(file)), true));
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

pub(super) fn calibration_reject(request_id: &Value, e: &crate::calibration::CalibError) -> Value {
    let mut reply = json!({"type":"command_reject", "request_id":request_id});
    for (k, v) in e.reply_fields() { reply[k] = v; }
    reply
}

/// Dispatch a calibration command under the supervisor write lock.
#[allow(clippy::too_many_arguments)]
pub(super) fn dispatch_calibration_command(
    inner: &mut SupervisorInner,
    path: &str,
    value: &Value,
    local_generation: u64,
    epoch: u64,
    pending_calibration: &mut Option<(Value, CalibJob, u64, u64, bool)>,
) -> Option<OwnerReply> {
    match calibration_job(inner, path, &value["body"]) {
        Ok(CalibStart::Job(job, busy)) => {
            *pending_calibration = Some((value["request_id"].clone(), job, local_generation, epoch, busy));
            None
        }
        Ok(CalibStart::Reply(reply)) => Some(OwnerReply::FieldsAck(reply)),
        Err(e) => Some(OwnerReply::CalibReject(e)),
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
    recv_supervisor: &Arc<ArmSupervisor>,
    recv_calib: &crate::calibration::CalibrationDeps,
    recv_connection: &str,
    recv_generation: &Arc<AtomicU64>,
    ack_tx: &mpsc::Sender<Value>,
    ws_auth: &WsAuth,
    worker_tasks: &ConnectionTasks,
) {
    let (sup, calib, conn, gen_now, tx) = (
        recv_supervisor.clone(),
        recv_calib.clone(),
        recv_connection.to_string(),
        recv_generation.clone(),
        ack_tx.clone(),
    );
    let ws_auth_c = ws_auth.clone();
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
