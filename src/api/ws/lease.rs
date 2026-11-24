//! Lease lifecycle, preemption, heartbeat, loss reasons, and torque/arm/clear_fault routing (family L).
//!
//! Moved from `api/ws.rs` and `api/ws/motion_cmds.rs` without behaviour changes (A1b PR 1).

#[allow(unused_imports)]
use super::*;
use super::dispatch::{
    AdmissionClass, AuthRequirement, DispatchEntry, DispatchHandler, DispatchKey, FrameKind,
    LeaseRequirement, OwnerDispatchCtx, ReplyPolicy,
};

pub(super) const ENTRIES: &[DispatchEntry] = &[
    DispatchEntry {
        key: DispatchKey::frame(FrameKind::Heartbeat),
        handler: DispatchHandler::Owner(dispatch_heartbeat_inline),
        admission: AdmissionClass::Heartbeat,
        lease: LeaseRequirement::RequiredAllowZeroGeneration,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::SilentOrLossOrEpochSync,
    },
    DispatchEntry {
        key: DispatchKey::frame(FrameKind::AcquireLease),
        handler: DispatchHandler::Owner(dispatch_acquire_lease),
        admission: AdmissionClass::OrdinarySession,
        lease: LeaseRequirement::None,
        auth: AuthRequirement::Conditional,
        reply: ReplyPolicy::LeaseReply,
    },
    DispatchEntry {
        key: DispatchKey::frame(FrameKind::ReleaseLease),
        handler: DispatchHandler::Owner(dispatch_release_lease),
        admission: AdmissionClass::OrdinarySession,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::LeaseReply,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/torque"),
        handler: DispatchHandler::Owner(dispatch_torque),
        admission: AdmissionClass::OrdinaryTorqueOrStopExempt,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandImmediate,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/arm"),
        handler: DispatchHandler::Owner(dispatch_torque),
        admission: AdmissionClass::OrdinaryTorqueOrStopExempt,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandImmediate,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/safety/clear_fault"),
        handler: DispatchHandler::Owner(dispatch_clear_fault),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::Conditional,
        reply: ReplyPolicy::CommandImmediate,
    },
];

fn dispatch_acquire_lease(ctx: &mut OwnerDispatchCtx<'_>) -> Option<Value> {
    let (resp, evicted) = handle_acquire_lease(
        ctx.inner,
        ctx.value,
        ctx.recv_conn_id,
        ctx.recv_connection,
        ctx.recv_generation,
        ctx.recv_client_label,
        ctx.worker_closing,
        ctx.ws_auth,
        ctx.last_sequence,
    );
    *ctx.evicted_notify = evicted;
    resp
}

fn dispatch_release_lease(ctx: &mut OwnerDispatchCtx<'_>) -> Option<Value> {
    Some(handle_release_lease(
        ctx.inner,
        ctx.recv_connection,
        ctx.local_generation,
        ctx.recv_generation,
        ctx.recv_client_label,
    ))
}

fn dispatch_heartbeat_inline(ctx: &mut OwnerDispatchCtx<'_>) -> Option<Value> {
    handle_heartbeat_inline(ctx.inner, ctx.recv_connection, ctx.epoch)
}

fn dispatch_torque(ctx: &mut OwnerDispatchCtx<'_>) -> Option<Value> {
    let body = &ctx.value["body"];
    let result = validate_command_body_common(body)
        .map_err(invalid_command)
        .and_then(|()| execute_torque_command(ctx.inner, ctx.stamp, body));
    Some(match result {
        Ok(started) => command_ack(ctx.inner, &ctx.value["request_id"], started),
        Err(error) => command_reject(ctx.inner, &ctx.value["request_id"], error),
    })
}

fn dispatch_clear_fault(ctx: &mut OwnerDispatchCtx<'_>) -> Option<Value> {
    let body = &ctx.value["body"];
    let result = validate_command_body_common(body)
        .map_err(invalid_command)
        .and_then(|()| execute_clear_fault(ctx.inner, body, ctx.epoch));
    Some(match result {
        Ok(started) => command_ack(ctx.inner, &ctx.value["request_id"], started),
        Err(error) => command_reject(ctx.inner, &ctx.value["request_id"], error),
    })
}

/// Torque disable (`/api/v1/torque` or `/api/v1/arm` with `enable: false`) is a stop-direction
/// action: an ordinary Stop must never cancel or drain it from the queue.
pub(super) fn is_torque_disable_command(value: &Value) -> bool {
    value.get("type").and_then(Value::as_str) == Some("command")
        && matches!(
            value.get("path").and_then(Value::as_str),
            Some("/api/v1/torque" | "/api/v1/arm")
        )
        && value
            .get("body")
            .and_then(|b| b.get("enable"))
            .and_then(Value::as_bool)
            == Some(false)
}

/// Execute `/api/v1/torque` or `/api/v1/arm` under the supervisor write lock.
pub(super) fn execute_torque_command(
    inner: &mut SupervisorInner,
    stamp: crate::supervisor::AdmissionStamp,
    body: &Value,
) -> Result<Started, ApiError> {
    let enabled = body
        .get("enable")
        .and_then(Value::as_bool)
        .ok_or_else(|| invalid_command("enable must be boolean"))?;
    if enabled {
        let admitted = inner.check_admission(stamp).map_err(ArmError::from)?;
        SupervisorInner::enable_torque(admitted).map_err(ArmError::from)?;
    } else {
        inner.disable_torque().map_err(ArmError::from)?;
    }
    Ok(None)
}

/// Execute `/api/v1/safety/clear_fault` under the supervisor write lock.
pub(super) fn execute_clear_fault(
    inner: &mut SupervisorInner,
    body: &Value,
    epoch: u64,
) -> Result<Started, ApiError> {
    // {"mit_recovery": "torque_off_all"}: the operator accepts the MIT expiry recovery
    // (torque off on every joint); any other value is refused
    let mit_recovery = match body.get("mit_recovery") {
        None | Some(Value::Null) => false,
        Some(v) if v.as_str() == Some(crate::mit::RECOVERY_CONFIRM_VALUE) => true,
        Some(_) => {
            return Err(invalid_command(&format!(
                "mit_recovery must be \"{}\"",
                crate::mit::RECOVERY_CONFIRM_VALUE
            )))
        }
    };
    inner.clear_fault_confirmed_for_epoch(mit_recovery, Some(epoch))?;
    Ok(None)
}

/// Process one latest-wins heartbeat in `heartbeat_task` under the supervisor write lock.
pub(super) fn handle_heartbeat_task_step(
    inner: &mut SupervisorInner,
    value: &Value,
    hb_connection: &str,
    hb_generation: &AtomicU64,
    hb_client_label: &Mutex<Option<String>>,
) -> Option<Value> {
    if inner.detached {
        None
    } else {
        let local_generation = hb_generation.load(Ordering::SeqCst);
        let generation = value.get("generation").and_then(Value::as_u64).unwrap_or(0);
        let label_matches = {
            let lbl = hb_client_label.lock().unwrap_or_else(|e| e.into_inner());
            value.get("client_id").is_none_or(|id| id.as_str() == lbl.as_deref())
        };
        let supplied_generation_ok = generation == local_generation || generation == 0;
        let authorized = local_generation != 0
            && supplied_generation_ok
            && label_matches
            && inner.validate_servo_lease(hb_connection, local_generation);
        if !authorized {
            Some(if local_generation == 0 {
                json!({"type":"lease_lost", "generation":0})
            } else {
                json!({"type":"lease_lost", "generation":local_generation, "reason":inner.lease_loss_reason(hb_connection, local_generation)})
            })
        } else {
            let epoch = inner.safety.epoch_id();
            if value.get("epoch_id").and_then(Value::as_u64) != Some(epoch) {
                Some(json!({"type":"epoch_sync", "epoch_id":epoch, "safety_state":inner.safety.state()}))
            } else {
                let _ = inner.heartbeat(hb_connection, epoch);
                None
            }
        }
    }
}

/// Handle `acquire_lease` (normal and `force: true` preemption) under the supervisor write lock.
#[allow(clippy::too_many_arguments)]
pub(super) fn handle_acquire_lease(
    inner: &mut SupervisorInner,
    value: &Value,
    recv_conn_id: u64,
    recv_connection: &str,
    recv_generation: &AtomicU64,
    recv_client_label: &Mutex<Option<String>>,
    worker_closing: &AtomicBool,
    ws_auth: &WsAuth,
    last_sequence: &mut Option<u64>,
) -> (Option<Value>, Option<(String, u64)>) {
    let mut evicted_notify: Option<(String, u64)> = None;
    let response;
    if worker_closing.load(Ordering::SeqCst) || inner.is_connection_closed(recv_conn_id) {
        // Teardown already started while waiting for the owner lock: stop any
        // existing lease's motion first, then release it; never acquire or evict.
        let _ = inner.release_lease_for_connection_and_stop(recv_connection);
        recv_generation.store(0, Ordering::SeqCst);
        response = None;
    } else {
        let label = value.get("client_id").and_then(Value::as_str).unwrap_or("");
        let seq = value.get("seq").and_then(Value::as_u64);
        let force_val = value.get("force");

        if label.is_empty() || label.len() > 128 {
            let mut reply = json!({"type":"lease_rejected", "error":"invalid_client_id"});
            if let Some(s) = seq {
                reply["seq"] = json!(s);
            }
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
                    if let Some(s) = seq {
                        reply["seq"] = json!(s);
                    }
                    response = Some(reply);
                }
                _ => {
                    let force = force_val.and_then(Value::as_bool).unwrap_or(false);
                    if force {
                        if !ws_auth.is_operator() {
                            let mut reply = json!({
                                "type": "lease_rejected",
                                "status": 403,
                                "code": "OPERATOR_REQUIRED",
                                "error": "operator_required",
                                "message": "Operator unlock is required for force lease preemption"
                            });
                            if let Some(s) = seq {
                                reply["seq"] = json!(s);
                            }
                            response = Some(reply);
                        } else {
                            match inner.preempt_lease(recv_connection, label) {
                                Ok((generation, evicted)) => {
                                    if worker_closing.load(Ordering::SeqCst)
                                        || inner.is_connection_closed(recv_conn_id)
                                    {
                                        // Late acquire completing after teardown started: stop motion first, then release.
                                        let _ = inner.release_lease_and_stop(recv_connection, generation);
                                        recv_generation.store(0, Ordering::SeqCst);
                                        response = None;
                                    } else {
                                        let previous = recv_generation.swap(generation, Ordering::SeqCst);
                                        *recv_client_label.lock().unwrap_or_else(|e| e.into_inner()) =
                                            Some(label.to_owned());
                                        if previous != generation {
                                            *last_sequence = None;
                                        }
                                        let mut reply = json!({
                                            "type": "lease_granted",
                                            "client_id": label,
                                            "generation": generation
                                        });
                                        if let Some(s) = seq {
                                            reply["seq"] = json!(s);
                                        }
                                        response = Some(reply);
                                        evicted_notify = evicted;
                                    }
                                }
                                Err(crate::supervisor::PreemptError::Cooldown(remaining)) => {
                                    let mut reply = json!({
                                        "type": "lease_rejected",
                                        "error": "preemption_cooldown",
                                        "remaining_ms": remaining
                                    });
                                    if let Some(s) = seq {
                                        reply["seq"] = json!(s);
                                    }
                                    response = Some(reply);
                                }
                                Err(crate::supervisor::PreemptError::OwnerTransferred) => {
                                    let mut reply = json!({
                                        "type": "lease_rejected",
                                        "error": "owner_transferred",
                                        "code": "OWNER_TRANSFERRED"
                                    });
                                    if let Some(s) = seq {
                                        reply["seq"] = json!(s);
                                    }
                                    response = Some(reply);
                                }
                            }
                        }
                    } else {
                        match inner.acquire_lease(recv_connection) {
                            Ok(generation) => {
                                if worker_closing.load(Ordering::SeqCst)
                                    || inner.is_connection_closed(recv_conn_id)
                                {
                                    // Late acquire completing after teardown started: stop motion first, then release.
                                    let _ = inner.release_lease_and_stop(recv_connection, generation);
                                    recv_generation.store(0, Ordering::SeqCst);
                                    response = None;
                                } else {
                                    inner.set_lease_label(recv_connection, generation, label);
                                    let previous = recv_generation.swap(generation, Ordering::SeqCst);
                                    *recv_client_label.lock().unwrap_or_else(|e| e.into_inner()) =
                                        Some(label.to_owned());
                                    if previous != generation {
                                        *last_sequence = None;
                                    }
                                    let mut reply = json!({
                                        "type": "lease_granted",
                                        "client_id": label,
                                        "generation": generation
                                    });
                                    if let Some(s) = seq {
                                        reply["seq"] = json!(s);
                                    }
                                    response = Some(reply);
                                }
                            }
                            Err((code, 0)) if code == "OWNER_TRANSFERRED" => {
                                let mut reply = json!({
                                    "type": "lease_rejected",
                                    "error": "owner_transferred",
                                    "code": "OWNER_TRANSFERRED"
                                });
                                if let Some(s) = seq {
                                    reply["seq"] = json!(s);
                                }
                                response = Some(reply);
                            }
                            Err(_) => {
                                let mut reply =
                                    json!({"type":"lease_rejected", "error":"controller_busy"});
                                if let Some(s) = seq {
                                    reply["seq"] = json!(s);
                                }
                                response = Some(reply);
                            }
                        }
                    }
                }
            }
        }
    }
    (response, evicted_notify)
}

/// Check whether this connection currently holds the lease matching the inbound frame.
/// Returns `(local_generation, authorized)`.
pub(super) fn check_lease_authority(
    inner: &mut SupervisorInner,
    kind: &str,
    value: &Value,
    recv_connection: &str,
    recv_generation: &AtomicU64,
    recv_client_label: &Mutex<Option<String>>,
) -> (u64, bool) {
    let local_generation = recv_generation.load(Ordering::SeqCst);
    let generation = value.get("generation").and_then(Value::as_u64).unwrap_or(0);
    let label_matches = {
        let lbl = recv_client_label.lock().unwrap_or_else(|e| e.into_inner());
        value.get("client_id").is_none_or(|id| id.as_str() == lbl.as_deref())
    };
    // Heartbeat from legacy clients may omit generation; connection still must own it.
    let supplied_generation_ok =
        generation == local_generation || (kind == "heartbeat" && generation == 0);
    let authorized = local_generation != 0
        && supplied_generation_ok
        && label_matches
        && inner.validate_servo_lease(recv_connection, local_generation);
    (local_generation, authorized)
}

/// Format the lease-loss reply for an unauthorized ordinary frame.
pub(super) fn lease_loss_reply(
    inner: &SupervisorInner,
    kind: &str,
    value: &Value,
    recv_connection: &str,
    local_generation: u64,
) -> Value {
    if kind == "command" {
        json!({"type":"command_reject", "request_id":value.get("request_id"), "status":409, "code":"LEASE_LOST", "message":"This connection does not own an active lease", "error":"LEASE_LOST", "epoch_id":inner.safety.epoch_id()})
    } else if kind == "release_lease" {
        json!({"type":"lease_released", "success":false})
    } else if local_generation == 0 {
        json!({"type":"lease_lost", "generation":0})
    } else {
        json!({"type":"lease_lost", "generation":local_generation, "reason":inner.lease_loss_reason(recv_connection, local_generation)})
    }
}

/// Handle `release_lease` under the supervisor write lock.
pub(super) fn handle_release_lease(
    inner: &mut SupervisorInner,
    recv_connection: &str,
    local_generation: u64,
    recv_generation: &AtomicU64,
    recv_client_label: &Mutex<Option<String>>,
) -> Value {
    let released = inner.release_lease_and_stop(recv_connection, local_generation);
    recv_generation.store(0, Ordering::SeqCst);
    *recv_client_label.lock().unwrap_or_else(|e| e.into_inner()) = None;
    json!({"type":"lease_released", "success":released})
}

/// Handle an inline heartbeat under the supervisor write lock.
pub(super) fn handle_heartbeat_inline(
    inner: &mut SupervisorInner,
    recv_connection: &str,
    epoch: u64,
) -> Option<Value> {
    let _ = inner.heartbeat(recv_connection, epoch);
    None
}

/// Deliver a `lease_lost` (`reason: "preempted"`) notice to an evicted connection via the registry.
pub(super) fn notify_evicted(evicted_conn: String, evicted_gen: u64) {
    let (sender_opt, closer_opt) = {
        let reg = CONNECTION_REGISTRY.lock().unwrap_or_else(|e| e.into_inner());
        let closers = CONNECTION_CLOSERS.lock().unwrap_or_else(|e| e.into_inner());
        (reg.get(&evicted_conn).cloned(), closers.get(&evicted_conn).cloned())
    };
    if let Some(sender) = sender_opt {
        let lost = json!({
            "type": "lease_lost",
            "generation": evicted_gen,
            "reason": "preempted"
        });
        match sender.try_send(Message::Text(lost.to_string().into())) {
            Ok(()) => {}
            Err(mpsc::error::TrySendError::Full(_)) => {
                tracing::warn!(
                    "Urgent revocation queue full for {}; closing evicted connection",
                    evicted_conn
                );
                if let Some((closer, fatal_flag, notify)) = closer_opt {
                    fatal_flag.store(true, Ordering::Release);
                    notify.notify_waiters();
                    let _ = closer.try_send(FatalClose::urgent_queue_full());
                }
            }
            Err(mpsc::error::TrySendError::Closed(_)) => {}
        }
    }
}
