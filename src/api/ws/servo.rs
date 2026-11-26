//! Servo stream classification, generation/sequence validation, and servo handlers (family S).
//!
//! Moved from `api/ws.rs` and `api/ws/motion.rs` without behaviour changes (A1b PR 1).

#[allow(unused_imports)]
use super::*;
use super::dispatch::{
    AdmissionClass, AuthRequirement, DispatchEntry, DispatchHandler, DispatchKey, FrameKind,
    LeaseRequirement, OwnerDispatchCtx, OwnerReply, ReplyPolicy,
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

fn dispatch_servo_frame(ctx: &mut OwnerDispatchCtx<'_>) -> Option<OwnerReply> {
    handle_servo_frame(ctx.inner, ctx.stamp, ctx.value, ctx.epoch, ctx.last_sequence)
}

fn dispatch_servo_command(ctx: &mut OwnerDispatchCtx<'_>) -> Option<OwnerReply> {
    let body = &ctx.value["body"];
    let result = validate_command_body_common(body)
        .map_err(invalid_command)
        .and_then(|()| execute_servo_command(ctx.inner, ctx.stamp, body, ctx.epoch));
    Some(match result {
        Ok(started) => OwnerReply::CommandAck {
            snap: AckSnapshot::from_inner(ctx.inner),
            started,
        },
        Err(error) => OwnerReply::CommandReject {
            epoch: ctx.inner.safety.epoch_id(),
            error,
        },
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
pub(super) fn is_missing_generation(kind: &str, value: &Value) -> bool {
    is_explicit_or_cartesian_servo(kind, value)
        && value.get("generation").and_then(Value::as_u64).is_none()
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
/// under the supervisor write lock. Returns `None` on acceptance or `Some(OwnerReply)` on error
/// (formatted into JSON strictly off-lock).
pub(super) fn handle_servo_frame(
    inner: &mut SupervisorInner,
    stamp: crate::supervisor::AdmissionStamp,
    value: &Value,
    epoch: u64,
    last_sequence: &mut Option<u64>,
) -> Option<OwnerReply> {
    match value.get("sequence_id").and_then(Value::as_u64) {
        _ if inner.calibration_busy => Some(OwnerReply::ServoRejectReason("calibration_busy")),
        None => Some(OwnerReply::ServoRejectReason("missing_sequence_id")),
        Some(sequence)
            if last_sequence.is_some_and(|last| {
                let delta = sequence.wrapping_sub(last);
                delta == 0 || delta > 1000
            }) =>
        {
            Some(OwnerReply::ServoRejectReason("invalid_sequence_id"))
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
                Err(error) => Some(OwnerReply::ServoRejectApi { epoch, error }),
            }
        }
    }
}

fn format_servo_command_refusal(
    request_id: &Value,
    refusal: &crate::rt_core::mailbox::Refusal,
    epoch_id: u64,
) -> Value {
    match refusal.code {
        "LEASE_LOST" | "NO_LEASE" | "STREAM_NOT_AUTHORIZED" => {
            lease::format_command_lease_lost(request_id, epoch_id)
        }
        "EPOCH_MISMATCH" => lease::format_command_epoch_mismatch(request_id, epoch_id),
        "CALIBRATION_BUSY" => lease::format_command_reject_api(
            request_id,
            ApiError(
                StatusCode::CONFLICT,
                "a calibration apply, restore or motor zero is running".into(),
                "MOTION_BUSY".into(),
            ),
            epoch_id,
        ),
        "STOPPED" => json!({
            "type": "command_reject",
            "request_id": request_id,
            "status": 409,
            "code": "STOPPED",
            "error": "STOPPED",
            "message": "Stopped before the action began",
            "epoch_id": epoch_id
        }),
        code => {
            let (status, error, safety_state) = crate::supervisor::targets::reject_fields(code);
            let mut reply = json!({
                "type": "command_reject",
                "request_id": request_id,
                "status": status,
                "code": code,
                "error": error,
                "message": "servo target refused when the control loop applied it",
                "safety_state": safety_state,
                "epoch_id": epoch_id,
            });
            if let crate::rt_core::mailbox::RefusalDetail::CurrentRevision(rev) = refusal.detail {
                reply["current_revision"] = json!(rev);
            } else {
                crate::chain::attach_workspace_box_edge_fields(&mut reply, refusal.detail, "servo_target");
            }
            reply
        }
    }
}

fn translate_immediate_submit_to_command_reject(
    request_id: &Value,
    reply: Value,
    epoch_id: u64,
) -> Value {
    let ty = reply.get("type").and_then(Value::as_str).unwrap_or("");
    if ty == "lease_lost" {
        return lease::format_command_lease_lost(request_id, epoch_id);
    }
    if ty == "epoch_sync" {
        let ep = reply
            .get("epoch_id")
            .and_then(Value::as_u64)
            .unwrap_or(epoch_id);
        return lease::format_command_epoch_mismatch(request_id, ep);
    }
    if reply.get("reason").and_then(Value::as_str) == Some("calibration_busy") {
        return lease::format_command_reject_api(
            request_id,
            ApiError(
                StatusCode::CONFLICT,
                "a calibration apply, restore or motor zero is running".into(),
                "MOTION_BUSY".into(),
            ),
            epoch_id,
        );
    }
    let code = reply
        .get("code")
        .and_then(Value::as_str)
        .unwrap_or("SAFETY_ERROR");
    let (default_status, default_error, default_safety_state) =
        crate::supervisor::targets::reject_fields(code);
    let status = reply
        .get("status")
        .and_then(Value::as_u64)
        .unwrap_or(u64::from(default_status));
    let error = reply
        .get("error")
        .and_then(Value::as_str)
        .unwrap_or(default_error);
    let message = reply
        .get("message")
        .and_then(Value::as_str)
        .unwrap_or("servo target refused");
    let safety_state = reply
        .get("safety_state")
        .cloned()
        .unwrap_or_else(|| json!(default_safety_state));
    let ep = reply
        .get("epoch_id")
        .and_then(Value::as_u64)
        .unwrap_or(epoch_id);
    json!({
        "type": "command_reject",
        "request_id": request_id,
        "status": status,
        "code": code,
        "error": error,
        "message": message,
        "safety_state": safety_state,
        "epoch_id": ep,
    })
}

/// Routes WS servo frames (`type: "servo"`, `mode: "cartesian_joint_stream"`, implicit `positions`)
/// and WS `/api/v1/motion/servo` commands through `TargetPort::submit_servo` when `ArmCore` is
/// owned by the RT executor (`OWNER_EXECUTOR`).
///
/// Returns `None` for non-servo messages (falling through to other executor handlers), or
/// `Some(optional_immediate_reply)` when handled.
#[allow(clippy::too_many_arguments)]
pub(super) async fn handle_executor_ws_servo(
    frame_kind: FrameKind,
    value: &Value,
    stamp: crate::supervisor::AdmissionStamp,
    received_at: Instant,
    conn_id: u64,
    recv_connection: &str,
    recv_generation: &AtomicU64,
    recv_client_label: &Mutex<Option<String>>,
    streaming_servo: &mut crate::supervisor::targets::ServoStream,
    cmd_servo: &mut crate::supervisor::targets::ServoStream,
    last_sequence: &mut Option<u64>,
    last_ordinary_request: &mut Option<u64>,
    supervisor: &ArmSupervisor,
    servo_reply_sink: &Arc<dyn Fn(Value) + Send + Sync>,
) -> Option<Option<Value>> {
    use crate::rt_core::executor::{ServoCommandOutcome, ServoCommandSubmitError};
    use crate::supervisor::targets::ServoMessage;

    let is_servo_cmd = frame_kind == FrameKind::Command
        && value.get("path").and_then(Value::as_str) == Some("/api/v1/motion/servo");
    let is_streaming = matches!(frame_kind, FrameKind::Servo | FrameKind::ImplicitServo);
    if !is_streaming && !is_servo_cmd {
        return None;
    }

    // 1. Explicit `servo` / `cartesian_joint_stream` require an explicit `generation` field
    //    before lease or snapshot checks (exact legacy precedence).
    if frame_kind == FrameKind::Servo && value.get("generation").and_then(Value::as_u64).is_none() {
        return Some(Some(json!({"type":"servo_reject", "reason":"missing_generation"})));
    }

    let Some(handle) = supervisor.executor_handle() else {
        return Some(executor_routing_reject(value, crate::supervisor::OWNER_FAILED));
    };
    let mut snap = match supervisor.latest_executor_snapshot().await {
        Ok(s) => s,
        Err(_) => return Some(executor_routing_reject(value, crate::supervisor::OWNER_FAILED)),
    };
    if supervisor.is_estop_latched() && snap.obs.safety_state != crate::safety::SafetyState::EStop {
        if let Ok(fresh) = supervisor.fresh_snapshot(None).await {
            snap = fresh;
        }
    }

    // 2. Lease authority check against local generation, client label, published owner snapshot,
    //    and off-thread closed-generation fence.
    let local_generation = recv_generation.load(Ordering::SeqCst);
    let supplied_generation = value.get("generation").and_then(Value::as_u64).unwrap_or(0);
    let label_matches = {
        let lbl = recv_client_label.lock().unwrap_or_else(|e| e.into_inner());
        value
            .get("client_id")
            .is_none_or(|id| id.as_str() == lbl.as_deref())
    };
    let now = Instant::now();
    let owner_matches = snap.owner.is_lease_active_at(now)
        && snap.owner.lease_generation == local_generation
        && (snap.owner.lease_owner_conn_id == conn_id
            || (snap.owner.lease_owner_conn_id == 0
                && snap
                    .owner
                    .lease_owner
                    .as_ref()
                    .is_some_and(|o| o.as_str() == recv_connection)));
    let authorized = local_generation != 0
        && supplied_generation == local_generation
        && label_matches
        && owner_matches
        && !handle.is_servo_closed(local_generation);

    let req_id_val = value.get("request_id").unwrap_or(&Value::Null);
    if !authorized {
        if is_servo_cmd {
            return Some(Some(lease::format_command_lease_lost(
                req_id_val,
                snap.obs.epoch_id,
            )));
        }
        return Some(Some(if local_generation == 0 {
            json!({"type":"lease_lost", "generation":0})
        } else {
            json!({
                "type": "lease_lost",
                "generation": local_generation,
                "reason": snap.owner.lease_loss_reason(recv_connection, local_generation),
            })
        }));
    }

    // 3. Epoch check (matching legacy `ws.rs:2792`).
    let epoch = snap.obs.epoch_id;
    if value.get("epoch_id").and_then(Value::as_u64) != Some(epoch) {
        if is_servo_cmd {
            return Some(Some(lease::format_command_epoch_mismatch(req_id_val, epoch)));
        }
        return Some(Some(json!({
            "type": "epoch_sync",
            "epoch_id": epoch,
            "safety_state": snap.obs.safety_state,
        })));
    }

    // Preserve ingress stop generation from reader's `AdmissionStamp` (never re-sample).
    let authority = crate::rt_core::broker::Authority {
        lease_generation: local_generation,
        epoch,
        stop_generation: stamp.stop_gen,
        config_revision: snap.owner.config_revision,
        parent_generation: 0,
    };

    if is_streaming {
        if streaming_servo.last_sequence() != *last_sequence {
            streaming_servo.sync_from(local_generation, *last_sequence);
        } else {
            streaming_servo.lease_granted(local_generation);
        }
        let seq_opt = value.get("sequence_id").and_then(Value::as_u64);
        match positions(value.get("positions")) {
            Ok(targets) => {
                let msg = ServoMessage {
                    sequence_id: seq_opt,
                    positions: &targets,
                    authority,
                    received_at,
                    calibration_busy: snap.owner.calibration_busy,
                };
                match handle.submit_servo_stream(
                    conn_id,
                    recv_connection,
                    servo_reply_sink,
                    streaming_servo,
                    &msg,
                ) {
                    Ok(mut immediate) => {
                        *last_sequence = streaming_servo.last_sequence();
                        if let Some(ref mut reply) = immediate {
                            if reply.get("type").and_then(Value::as_str) == Some("lease_lost")
                                && local_generation != 0
                                && reply.get("reason").is_none()
                            {
                                reply["reason"] = json!(snap
                                    .owner
                                    .lease_loss_reason(recv_connection, local_generation));
                            }
                        }
                        Some(immediate)
                    }
                    Err(()) => Some(executor_routing_reject(
                        value,
                        crate::supervisor::OWNER_FAILED,
                    )),
                }
            }
            Err(api_err) => {
                // Preserve exact legacy precedence: closed lease -> calibration_busy ->
                // sequence_id checks (advancing sequence once validated) -> positions error.
                if handle.is_servo_closed(local_generation) {
                    return Some(Some(json!({
                        "type": "lease_lost",
                        "generation": local_generation,
                        "reason": snap.owner.lease_loss_reason(recv_connection, local_generation),
                    })));
                }
                if snap.owner.calibration_busy {
                    return Some(Some(json!({"type":"servo_reject", "reason":"calibration_busy"})));
                }
                let Some(sequence) = seq_opt else {
                    return Some(Some(json!({"type":"servo_reject", "reason":"missing_sequence_id"})));
                };
                if streaming_servo.last_sequence().is_some_and(|last| {
                    let delta = sequence.wrapping_sub(last);
                    delta == 0 || delta > 1000
                }) {
                    return Some(Some(json!({"type":"servo_reject", "reason":"invalid_sequence_id"})));
                }
                streaming_servo.advance_sequence(sequence);
                *last_sequence = streaming_servo.last_sequence();
                Some(Some(rejection(api_err, "servo_reject", epoch)))
            }
        }
    } else {
        // WS command `/api/v1/motion/servo`: validate request_id, calibration_busy, body, targets,
        // submit via `TargetPort::submit_servo`, and await its correlated RT outcome.
        let parsed_request_id = value.get("request_id").and_then(Value::as_u64);
        if parsed_request_id.is_none()
            || last_ordinary_request.is_some_and(|last| parsed_request_id.unwrap() <= last)
        {
            return Some(Some(lease::format_command_invalid_request_id(req_id_val)));
        }
        *last_ordinary_request = parsed_request_id;

        if snap.owner.calibration_busy {
            return Some(Some(lease::format_command_reject_api(
                req_id_val,
                ApiError(
                    StatusCode::CONFLICT,
                    "a calibration apply, restore or motor zero is running".into(),
                    "MOTION_BUSY".into(),
                ),
                epoch,
            )));
        }
        let body = &value["body"];
        if let Err(msg) = validate_command_body_common(body) {
            return Some(Some(lease::format_command_reject_api(
                req_id_val,
                invalid_command(msg),
                epoch,
            )));
        }
        let targets = match positions(body.get("targets")) {
            Ok(t) => t,
            Err(err) => {
                return Some(Some(lease::format_command_reject_api(
                    req_id_val,
                    err,
                    epoch,
                )));
            }
        };

        cmd_servo.lease_granted(local_generation);
        let cmd_seq = cmd_servo
            .last_sequence()
            .map_or(1, |last| last.wrapping_add(1));
        let msg = ServoMessage {
            sequence_id: Some(cmd_seq),
            positions: &targets,
            authority,
            received_at,
            calibration_busy: snap.owner.calibration_busy,
        };
        let waiter = match handle.submit_servo_command(cmd_servo, &msg) {
            Ok(w) => w,
            Err(ServoCommandSubmitError::Immediate(reply)) => {
                return Some(Some(translate_immediate_submit_to_command_reject(
                    req_id_val, reply, epoch,
                )));
            }
            Err(ServoCommandSubmitError::Unavailable) => {
                return Some(executor_routing_reject(
                    value,
                    crate::supervisor::OWNER_FAILED,
                ));
            }
        };
        let reply = match waiter.wait().await {
            ServoCommandOutcome::Applied {
                result: Ok(()),
                post,
            } => {
                let post = post.unwrap_or(snap);
                lease::format_command_ack_state(
                    req_id_val,
                    post.obs.epoch_id,
                    &post.obs.safety_state,
                    post.obs.is_torqued,
                    post.owner.gravity_comp_active,
                )
            }
            ServoCommandOutcome::Applied {
                result: Err(refusal),
                post,
            } => {
                let post_epoch = post.as_ref().map_or(epoch, |s| s.obs.epoch_id);
                format_servo_command_refusal(req_id_val, &refusal, post_epoch)
            }
            ServoCommandOutcome::Closed => {
                let post_epoch = supervisor
                    .hub()
                    .latest()
                    .map_or(epoch, |s| s.obs.epoch_id);
                lease::format_command_lease_lost(req_id_val, post_epoch)
            }
            ServoCommandOutcome::Coalesced => {
                let post_epoch = supervisor
                    .hub()
                    .latest()
                    .map_or(epoch, |s| s.obs.epoch_id);
                json!({
                    "type": "command_reject",
                    "request_id": req_id_val,
                    "status": 409,
                    "code": "SUPERSEDED",
                    "error": "SUPERSEDED",
                    "message": "Servo command was superseded before the control loop applied it",
                    "epoch_id": post_epoch,
                })
            }
            ServoCommandOutcome::Stopped => {
                return Some(executor_routing_reject(
                    value,
                    crate::supervisor::OWNER_FAILED,
                ));
            }
        };
        Some(Some(reply))
    }
}
