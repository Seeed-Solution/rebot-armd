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
