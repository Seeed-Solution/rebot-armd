//! Servo stream classification, generation/sequence validation, and servo handlers (family S).
//!
//! Moved from `api/ws.rs` and `api/ws/motion.rs` without behaviour changes (A1b PR 1).

#[allow(unused_imports)]
use super::*;
use super::dispatch::{
    AdmissionClass, AuthRequirement, DispatchEntry, DispatchHandler, DispatchKey, FrameKind,
    LeaseRequirement, OwnerDispatchCtx, ReplyPolicy,
};

pub(super) const ENTRIES: &[DispatchEntry] = &[
    DispatchEntry {
        key: DispatchKey::frame(FrameKind::Servo),
        handler: DispatchHandler::Owner(dispatch_servo_frame),
        admission: AdmissionClass::OrdinaryServo,
        lease: LeaseRequirement::RequiredExplicitGeneration,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::ServoRejectOnly,
    },
    DispatchEntry {
        key: DispatchKey::frame(FrameKind::ImplicitServo),
        handler: DispatchHandler::Owner(dispatch_servo_frame),
        admission: AdmissionClass::OrdinaryServo,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::ServoRejectOnly,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/motion/servo"),
        handler: DispatchHandler::Owner(dispatch_servo_command),
        admission: AdmissionClass::OrdinaryServo,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandImmediate,
    },
];

fn dispatch_servo_frame(ctx: &mut OwnerDispatchCtx<'_>) -> Option<Value> {
    handle_servo_frame(ctx.inner, ctx.stamp, ctx.value, ctx.epoch, ctx.last_sequence)
}

fn dispatch_servo_command(ctx: &mut OwnerDispatchCtx<'_>) -> Option<Value> {
    let body = &ctx.value["body"];
    let result = validate_command_body_common(body)
        .map_err(invalid_command)
        .and_then(|()| execute_servo_command(ctx.inner, ctx.stamp, body, ctx.epoch));
    Some(match result {
        Ok(started) => command_ack(ctx.inner, &ctx.value["request_id"], started),
        Err(error) => command_reject(ctx.inner, &ctx.value["request_id"], error),
    })
}

/// Explicit `type: "servo"` or `mode: "cartesian_joint_stream"` frames that require an explicit
/// `generation` field before falling through to generic lease checks.
pub(super) fn is_explicit_or_cartesian_servo(kind: &str, value: &Value) -> bool {
    kind == "servo"
        || value.get("mode").and_then(Value::as_str) == Some("cartesian_joint_stream")
}

/// Legacy servo-frame predicate used by Stop/E-stop queue cancellation (`drain_cancellable` and
/// `cancelled_reply`), matching explicit servo, `cartesian_joint_stream`, or any frame carrying
/// a top-level `positions` key.
pub(super) fn is_servo_frame(kind: &str, value: &Value) -> bool {
    is_explicit_or_cartesian_servo(kind, value) || value.get("positions").is_some()
}

/// Pre-lease missing-generation check for explicit / cartesian-stream servo frames.
pub(super) fn missing_generation_reject(kind: &str, value: &Value) -> Option<Value> {
    if is_explicit_or_cartesian_servo(kind, value)
        && value.get("generation").and_then(Value::as_u64).is_none()
    {
        Some(json!({"type":"servo_reject", "reason":"missing_generation"}))
    } else {
        None
    }
}

/// Execute `/api/v1/motion/servo` command under the supervisor write lock.
pub(super) fn execute_servo_command(
    inner: &mut SupervisorInner,
    stamp: crate::supervisor::AdmissionStamp,
    body: &Value,
    epoch: u64,
) -> Result<Started, ApiError> {
    let targets = positions(body.get("targets"))?;
    let admitted = inner.check_admission(stamp).map_err(ArmError::from)?;
    SupervisorInner::servo(admitted, targets, epoch).map_err(ArmError::from)?;
    Ok(None)
}

/// Handle a streaming servo frame (explicit, `cartesian_joint_stream`, or implicit `positions`)
/// under the supervisor write lock. Returns `None` on acceptance or `Some(servo_reject)` on error.
pub(super) fn handle_servo_frame(
    inner: &mut SupervisorInner,
    stamp: crate::supervisor::AdmissionStamp,
    value: &Value,
    epoch: u64,
    last_sequence: &mut Option<u64>,
) -> Option<Value> {
    match value.get("sequence_id").and_then(Value::as_u64) {
        _ if inner.calibration_busy => {
            Some(json!({"type":"servo_reject", "reason":"calibration_busy"}))
        }
        None => Some(json!({"type":"servo_reject", "reason":"missing_sequence_id"})),
        Some(sequence)
            if last_sequence.is_some_and(|last| {
                let delta = sequence.wrapping_sub(last);
                delta == 0 || delta > 1000
            }) =>
        {
            Some(json!({"type":"servo_reject", "reason":"invalid_sequence_id"}))
        }
        Some(sequence) => {
            *last_sequence = Some(sequence);
            match positions(value.get("positions")).and_then(|targets| {
                let admitted = inner
                    .check_admission(stamp)
                    .map_err(|error| ApiError::from(ArmError::from(error)))?;
                SupervisorInner::servo(admitted, targets, epoch)
                    .map_err(|error| ApiError::from(ArmError::from(error)))
            }) {
                Ok(()) => None,
                Err(error) => {
                    let stopped = error.code.as_deref() == Some("STOPPED");
                    let mut r = rejection(error, "servo_reject", epoch);
                    if stopped {
                        r["reason"] = json!("stopped");
                    }
                    Some(r)
                }
            }
        }
    }
}
