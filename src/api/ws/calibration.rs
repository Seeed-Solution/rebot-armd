//! Calibration commands of the WS command path: job selection, busy codes and rejections (track E).
//!
//! Moved from `api/ws.rs` without changes (armd 1.6 RT-core foundation); moved items, their
//! fields and inherent methods are `pub(super)` so the connection loop keeps using them.

#[allow(unused_imports)]
use super::*;

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

pub(super) fn is_calibration_path(path: &str) -> bool {
    path == "/api/v1/calibration/apply"
        || path == "/api/v1/calibration/restore"
        || path == crate::motor_zero::PATH
        || path == crate::motor_zero::RESTORE_PATH
        || path == crate::mit::TIMEOUT_PATH
        || path == crate::mit::QUALIFICATION_PATH
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
