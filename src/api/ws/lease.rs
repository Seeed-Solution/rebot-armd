//! Lease lifecycle, preemption, heartbeat, loss reasons, and torque/arm/clear_fault routing (family L).
//!
//! Moved from `api/ws.rs` and `api/ws/motion_cmds.rs` without behaviour changes (A1b PR 1).

#[allow(unused_imports)]
use super::*;
use super::dispatch::{
    AdmissionClass, AuthRequirement, CommandAsyncCtx, CommandAsyncFuture, DispatchEntry,
    DispatchHandler, DispatchKey, FrameKind, LeaseRequirement, ReplyPolicy,
};

pub(super) const ENTRIES: &[DispatchEntry] = &[
    DispatchEntry {
        key: DispatchKey::frame(FrameKind::Heartbeat),
        handler: DispatchHandler::Heartbeat(dispatch_heartbeat_ws),
        admission: AdmissionClass::Heartbeat,
        lease: LeaseRequirement::RequiredAllowZeroGeneration,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::SilentOrLossOrEpochSync,
    },
    DispatchEntry {
        key: DispatchKey::frame(FrameKind::AcquireLease),
        handler: DispatchHandler::SessionAsync(dispatch_acquire_lease_async),
        admission: AdmissionClass::OrdinarySession,
        lease: LeaseRequirement::None,
        auth: AuthRequirement::InHandler,
        reply: ReplyPolicy::LeaseReply,
    },
    DispatchEntry {
        key: DispatchKey::frame(FrameKind::ReleaseLease),
        handler: DispatchHandler::SessionAsync(dispatch_release_lease_async),
        admission: AdmissionClass::OrdinarySession,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::LeaseReply,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/torque"),
        handler: DispatchHandler::CommandAsync(dispatch_torque_async),
        admission: AdmissionClass::OrdinaryTorqueOrStopExempt,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandImmediate,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/arm"),
        handler: DispatchHandler::CommandAsync(dispatch_torque_async),
        admission: AdmissionClass::OrdinaryTorqueOrStopExempt,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandImmediate,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/safety/clear_fault"),
        handler: DispatchHandler::CommandAsync(dispatch_clear_fault_async),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::Conditional(clear_fault_auth_check),
        reply: ReplyPolicy::CommandImmediate,
    },
];

fn clear_fault_auth_check(
    ws_auth: &WsAuth,
    inner: &mut SupervisorInner,
    _path: &str,
    _body: &Value,
    epoch: u64,
) -> Option<ApiError> {
    if inner.consume_pending_estop() && inner.safety.epoch_id() != epoch {
        return Some(
            ArmError::Safety(SafetyError::EpochMismatch {
                expected: inner.safety.epoch_id(),
                received: epoch,
            })
            .into(),
        );
    }
    if ws_auth.is_operator() {
        return None;
    }
    // Re-check pending E-stop after `ws_auth.is_operator()` (preserving baseline parity when an
    // atomic E-stop latches concurrently during session-table verification).
    if inner.consume_pending_estop() && inner.safety.epoch_id() != epoch {
        return Some(
            ArmError::Safety(SafetyError::EpochMismatch {
                expected: inner.safety.epoch_id(),
                received: epoch,
            })
            .into(),
        );
    }
    if inner.e_stop_latched.load(Ordering::SeqCst) {
        return Some(ApiError::operator_required("clear_fault (EStop latched)"));
    }
    if (inner.safety.state().is_stopped() || inner.safety.has_latched_fault())
        && !inner.safety.recoverable()
    {
        return Some(ApiError::operator_required(
            "clear_fault (non-recoverable fault or EStop)",
        ));
    }
    None
}

fn dispatch_acquire_lease_async<'a>(
    ctx: super::dispatch::SessionDispatchCtx<'a>,
) -> super::dispatch::SessionFuture<'a> {
    Box::pin(async move {
        handle_acquire_lease_unified(
            ctx.supervisor,
            ctx.value,
            ctx.conn_handle,
            ctx.recv_generation,
            ctx.recv_client_label,
            ctx.worker_closing,
            ctx.ws_auth,
            ctx.last_sequence,
        )
        .await
    })
}

fn dispatch_release_lease_async<'a>(
    ctx: super::dispatch::SessionDispatchCtx<'a>,
) -> super::dispatch::SessionFuture<'a> {
    Box::pin(async move {
        (
            handle_release_lease_unified(
                ctx.supervisor,
                ctx.value,
                ctx.conn_handle,
                ctx.recv_generation,
                ctx.recv_client_label,
            )
            .await,
            None,
        )
    })
}

async fn current_reject_epoch(supervisor: &ArmSupervisor) -> u64 {
    supervisor
        .fresh_snapshot(None)
        .await
        .map_or_else(|_| supervisor.estop_epoch(), |s| s.obs.epoch_id)
}

fn format_command_lease_lost(request_id: &Value, epoch_id: u64) -> Value {
    json!({
        "type": "command_reject",
        "request_id": request_id,
        "status": 409,
        "code": "LEASE_LOST",
        "message": "This connection does not own an active lease",
        "error": "LEASE_LOST",
        "epoch_id": epoch_id
    })
}

fn format_command_epoch_mismatch(request_id: &Value, epoch_id: u64) -> Value {
    json!({
        "type": "command_reject",
        "request_id": request_id,
        "status": 409,
        "code": "EPOCH_MISMATCH",
        "message": "Command epoch is no longer current",
        "error": "EPOCH_MISMATCH",
        "epoch_id": epoch_id
    })
}

fn format_command_invalid_request_id(request_id: &Value) -> Value {
    json!({
        "type": "command_reject",
        "request_id": request_id,
        "status": 400,
        "code": "INVALID_COMMAND",
        "message": "request_id must be present and strictly increase",
        "error": "INVALID_COMMAND"
    })
}

fn format_command_ack_state(
    request_id: &Value,
    epoch_id: u64,
    safety_state: &crate::safety::SafetyState,
    is_torqued: bool,
    control_mode_gravity_comp: bool,
) -> Value {
    let control_mode = if control_mode_gravity_comp {
        "gravity_comp"
    } else {
        "position"
    };
    json!({
        "type": "command_ack",
        "request_id": request_id,
        "status": "accepted",
        "epoch_id": epoch_id,
        "safety_state": safety_state,
        "is_torqued": is_torqued,
        "control_mode": control_mode
    })
}

fn format_command_reject_api(request_id: &Value, error: ApiError, epoch_id: u64) -> Value {
    let mut reply = rejection(error, "command_reject", epoch_id);
    reply["request_id"] = request_id.clone();
    reply
}

fn check_local_command_lease(
    value: &Value,
    recv_generation: &AtomicU64,
    recv_client_label: &Mutex<Option<String>>,
) -> Option<u64> {
    let local_generation = recv_generation.load(Ordering::SeqCst);
    let supplied_generation = value.get("generation").and_then(Value::as_u64).unwrap_or(0);
    let label_matches = {
        let lbl = recv_client_label.lock().unwrap_or_else(|e| e.into_inner());
        value
            .get("client_id")
            .is_none_or(|id| id.as_str() == lbl.as_deref())
    };
    if local_generation != 0 && supplied_generation == local_generation && label_matches {
        Some(local_generation)
    } else {
        None
    }
}

/// Table-bound unified async handler for `/api/v1/torque` and `/api/v1/arm`.
fn dispatch_torque_async<'a>(ctx: CommandAsyncCtx<'a>) -> CommandAsyncFuture<'a> {
    Box::pin(async move {
        use crate::rt_core::requests::lease::TorqueWsOutcome;
        let req_id_val = ctx.value.get("request_id").unwrap_or(&Value::Null);
        let Some(local_generation) =
            check_local_command_lease(ctx.value, ctx.recv_generation, ctx.recv_client_label)
        else {
            return format_command_lease_lost(req_id_val, current_reject_epoch(ctx.supervisor).await);
        };
        let supplied_epoch = ctx.value.get("epoch_id").and_then(Value::as_u64);
        let parsed_request_id = ctx.value.get("request_id").and_then(Value::as_u64);
        let request_id_valid = parsed_request_id
            .is_some_and(|id| ctx.last_ordinary_request.is_none_or(|last| id > last));
        let body = &ctx.value["body"];
        let enable_res = validate_command_body_common(body).and_then(|()| {
            body.get("enable")
                .and_then(Value::as_bool)
                .ok_or("enable must be boolean")
        });

        match ctx
            .supervisor
            .torque_ws(
                ctx.conn_handle,
                local_generation,
                supplied_epoch,
                request_id_valid,
                parsed_request_id.unwrap_or(0),
                enable_res,
                ctx.stamp,
            )
            .await
        {
            Ok(outcome) => {
                if outcome.consumes_request_id() {
                    *ctx.last_ordinary_request = parsed_request_id;
                }
                match outcome {
                    TorqueWsOutcome::Accepted {
                        epoch_id,
                        safety_state,
                        is_torqued,
                        control_mode_gravity_comp,
                    } => format_command_ack_state(
                        req_id_val,
                        epoch_id,
                        &safety_state,
                        is_torqued,
                        control_mode_gravity_comp,
                    ),
                    TorqueWsOutcome::LeaseLost { epoch_id } => {
                        format_command_lease_lost(req_id_val, epoch_id)
                    }
                    TorqueWsOutcome::EpochMismatch { epoch_id } => {
                        format_command_epoch_mismatch(req_id_val, epoch_id)
                    }
                    TorqueWsOutcome::InvalidRequestId => {
                        format_command_invalid_request_id(req_id_val)
                    }
                    TorqueWsOutcome::InvalidBody { message, epoch_id } => {
                        format_command_reject_api(req_id_val, invalid_command(message), epoch_id)
                    }
                    TorqueWsOutcome::Rejected { error, epoch_id } => {
                        format_command_reject_api(req_id_val, error, epoch_id)
                    }
                }
            }
            Err(err) => {
                let epoch_id = current_reject_epoch(ctx.supervisor).await;
                format_command_reject_api(req_id_val, err.into(), epoch_id)
            }
        }
    })
}

/// Table-bound unified async handler for `/api/v1/safety/clear_fault`.
fn dispatch_clear_fault_async<'a>(ctx: CommandAsyncCtx<'a>) -> CommandAsyncFuture<'a> {
    Box::pin(async move {
        use crate::rt_core::requests::lease::LeaseClearFaultOutcome;
        let req_id_val = ctx.value.get("request_id").unwrap_or(&Value::Null);
        let Some(local_generation) =
            check_local_command_lease(ctx.value, ctx.recv_generation, ctx.recv_client_label)
        else {
            return format_command_lease_lost(req_id_val, current_reject_epoch(ctx.supervisor).await);
        };
        let supplied_epoch = ctx.value.get("epoch_id").and_then(Value::as_u64);
        let parsed_request_id = ctx.value.get("request_id").and_then(Value::as_u64);
        let request_id_valid = parsed_request_id
            .is_some_and(|id| ctx.last_ordinary_request.is_none_or(|last| id > last));
        let body = &ctx.value["body"];
        let (body_error, mit_recovery) = match validate_command_body_common(body) {
            Err(msg) => (Some(msg), false),
            Ok(()) => match body.get("mit_recovery") {
                None | Some(Value::Null) => (None, false),
                Some(v) if v.as_str() == Some(crate::mit::RECOVERY_CONFIRM_VALUE) => (None, true),
                Some(_) => (Some("mit_recovery must be \"torque_off_all\""), false),
            },
        };
        let operator = ctx.ws_auth.operator_grant();

        match ctx
            .supervisor
            .clear_fault_ws(
                ctx.conn_handle,
                local_generation,
                supplied_epoch,
                request_id_valid,
                body_error,
                mit_recovery,
                operator,
            )
            .await
        {
            Ok(Ok(outcome)) => {
                if outcome.consumes_request_id() {
                    *ctx.last_ordinary_request = parsed_request_id;
                }
                match outcome {
                    LeaseClearFaultOutcome::Cleared {
                        safety_state,
                        epoch_id,
                        is_torqued,
                        control_mode_gravity_comp,
                    } => format_command_ack_state(
                        req_id_val,
                        epoch_id,
                        &safety_state,
                        is_torqued,
                        control_mode_gravity_comp,
                    ),
                    LeaseClearFaultOutcome::LeaseLost { epoch_id } => {
                        format_command_lease_lost(req_id_val, epoch_id)
                    }
                    LeaseClearFaultOutcome::EpochMismatch {
                        expected,
                        received,
                        consumed_request_id,
                    } => {
                        if consumed_request_id {
                            let err: ApiError = ArmError::Safety(SafetyError::EpochMismatch {
                                expected,
                                received,
                            })
                            .into();
                            format_command_reject_api(req_id_val, err, expected)
                        } else {
                            format_command_epoch_mismatch(req_id_val, expected)
                        }
                    }
                    LeaseClearFaultOutcome::InvalidRequestId => {
                        format_command_invalid_request_id(req_id_val)
                    }
                    LeaseClearFaultOutcome::OperatorRequired {
                        estop_latched,
                        epoch_id,
                    } => {
                        let op_label = if estop_latched {
                            "clear_fault (EStop latched)"
                        } else {
                            "clear_fault (non-recoverable fault or EStop)"
                        };
                        format_command_reject_api(
                            req_id_val,
                            ApiError::operator_required(op_label),
                            epoch_id,
                        )
                    }
                    LeaseClearFaultOutcome::MotionBusy { epoch_id } => format_command_reject_api(
                        req_id_val,
                        ApiError(
                            StatusCode::CONFLICT,
                            "a calibration apply, restore or motor zero is running".into(),
                            "MOTION_BUSY".into(),
                        ),
                        epoch_id,
                    ),
                    LeaseClearFaultOutcome::InvalidBody { message, epoch_id } => {
                        format_command_reject_api(req_id_val, invalid_command(message), epoch_id)
                    }
                    LeaseClearFaultOutcome::ClearFailed { err, epoch_id } => {
                        let api: ApiError = ArmError::Safety(SafetyError::from(err)).into();
                        format_command_reject_api(req_id_val, api, epoch_id)
                    }
                }
            }
            Ok(Err((arm_err, epoch_id))) => {
                *ctx.last_ordinary_request = parsed_request_id;
                format_command_reject_api(req_id_val, arm_err.into(), epoch_id)
            }
            Err(err) => {
                let epoch_id = current_reject_epoch(ctx.supervisor).await;
                format_command_reject_api(req_id_val, err.into(), epoch_id)
            }
        }
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

/// Table-bound unified WS `heartbeat` handler invoked by `heartbeat_task`.
fn dispatch_heartbeat_ws<'a>(
    supervisor: &'a ArmSupervisor,
    value: &'a Value,
    conn_handle: crate::rt_core::requests::lease::ConnectionHandle,
    hb_generation: &'a AtomicU64,
    hb_client_label: &'a Mutex<Option<String>>,
) -> super::dispatch::HeartbeatFuture<'a> {
    Box::pin(handle_heartbeat_ws_async(
        supervisor,
        value,
        conn_handle,
        hb_generation,
        hb_client_label,
    ))
}

/// Unified owner-routed WS `heartbeat` handler shared by both legacy and RT executor owners.
///
/// Off-lock preparation runs from a published snapshot via [`ArmSupervisor::heartbeat_ws`],
/// preserving the connection-bound omission rule (`generation == 0` allowed when connection owns
/// a valid lease), `epoch_sync`, and `lease_lost` wire format (omitting `"reason"` at gen 0).
pub(super) async fn handle_heartbeat_ws_async(
    supervisor: &ArmSupervisor,
    value: &Value,
    conn_handle: crate::rt_core::requests::lease::ConnectionHandle,
    hb_generation: &AtomicU64,
    hb_client_label: &Mutex<Option<String>>,
) -> Option<Value> {
    use crate::rt_core::requests::lease::HeartbeatInstallOutcome;
    let local_generation = hb_generation.load(Ordering::SeqCst);
    let supplied_generation = value.get("generation").and_then(Value::as_u64).unwrap_or(0);
    let local_label = hb_client_label
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone();
    let supplied_client_id = value.get("client_id").map(Value::as_str);
    let supplied_epoch = value.get("epoch_id").and_then(Value::as_u64);
    match supervisor
        .heartbeat_ws(
            conn_handle,
            local_generation,
            supplied_generation,
            local_label.as_deref(),
            supplied_client_id,
            supplied_epoch,
        )
        .await
    {
        Ok(HeartbeatInstallOutcome::Renewed) => None,
        Ok(HeartbeatInstallOutcome::LeaseLost { generation, reason }) => Some(match reason {
            Some(r) => json!({"type":"lease_lost", "generation":generation, "reason":r}),
            None => json!({"type":"lease_lost", "generation":generation}),
        }),
        Ok(HeartbeatInstallOutcome::EpochSync { epoch, safety_state }) => {
            Some(json!({"type":"epoch_sync", "epoch_id":epoch, "safety_state":safety_state}))
        }
        Err(_) => None,
    }
}

/// Unified owner-routed WS `acquire_lease` (`force == false` and `force == true` preemption)
/// shared by both legacy and RT executor owners.
#[allow(clippy::too_many_arguments)]
pub(super) async fn handle_acquire_lease_unified(
    supervisor: &ArmSupervisor,
    value: &Value,
    conn_handle: crate::rt_core::requests::lease::ConnectionHandle,
    recv_generation: &AtomicU64,
    recv_client_label: &Mutex<Option<String>>,
    worker_closing: &AtomicBool,
    ws_auth: &WsAuth,
    last_sequence: &mut Option<u64>,
) -> (Option<Value>, Option<(String, u64)>) {
    use crate::rt_core::requests::lease::LeaseOutcome;
    if worker_closing.load(Ordering::SeqCst) {
        let _ = supervisor.disconnect_ws_connection(conn_handle).await;
        recv_generation.store(0, Ordering::SeqCst);
        return (None, None);
    }
    let label = value.get("client_id").and_then(Value::as_str).unwrap_or("");
    let seq = value.get("seq").and_then(Value::as_u64);
    let force_val = value.get("force");

    let with_seq = |mut reply: Value| -> Value {
        if let Some(s) = seq {
            reply["seq"] = json!(s);
        }
        reply
    };

    if label.is_empty() || label.len() > 128 {
        return (
            Some(with_seq(json!({"type":"lease_rejected", "error":"invalid_client_id"}))),
            None,
        );
    }
    match force_val {
        Some(v) if !v.is_boolean() && !v.is_null() => {
            return (
                Some(with_seq(json!({
                    "type": "lease_rejected",
                    "status": 400,
                    "error": "invalid_argument",
                    "message": "force must be a boolean"
                }))),
                None,
            );
        }
        _ => {}
    }

    let force = force_val.and_then(Value::as_bool).unwrap_or(false);
    let client_seq = seq.unwrap_or(0);
    let outcome = if force {
        let grant = ws_auth.operator_grant();
        supervisor
            .preempt_lease_ws(conn_handle, label, grant, client_seq)
            .await
    } else {
        supervisor
            .acquire_lease_ws(conn_handle, label, client_seq)
            .await
    };

    match outcome {
        Ok(LeaseOutcome::Granted {
            generation,
            evicted_connection,
            evicted_generation,
        }) => {
            // In executor mode, committed evictions are delivered post-publication by
            // `armd-dispatch` via `eviction_rx` independently of this requester's lifetime;
            // in legacy mode, StateHub was published under the write lock before returning.
            let evicted_notify = if supervisor.owner_mode() == crate::supervisor::OWNER_LEGACY {
                evicted_connection.map(|c| (c.as_str().to_owned(), evicted_generation))
            } else {
                None
            };
            if worker_closing.load(Ordering::SeqCst) {
                let _ = supervisor.disconnect_ws_connection(conn_handle).await;
                recv_generation.store(0, Ordering::SeqCst);
                return (None, evicted_notify);
            }
            let previous = recv_generation.swap(generation, Ordering::SeqCst);
            *recv_client_label.lock().unwrap_or_else(|e| e.into_inner()) = Some(label.to_owned());
            if previous != generation {
                *last_sequence = None;
            }
            let reply = with_seq(json!({
                "type": "lease_granted",
                "client_id": label,
                "generation": generation
            }));
            (Some(reply), evicted_notify)
        }
        Ok(LeaseOutcome::Busy { .. }) => (
            Some(with_seq(json!({"type":"lease_rejected", "error":"controller_busy"}))),
            None,
        ),
        Ok(LeaseOutcome::Cooldown { remaining_ms }) => (
            Some(with_seq(json!({
                "type": "lease_rejected",
                "error": "preemption_cooldown",
                "remaining_ms": remaining_ms
            }))),
            None,
        ),
        Ok(LeaseOutcome::OperatorRequired) => (
            Some(with_seq(json!({
                "type": "lease_rejected",
                "status": 403,
                "code": "OPERATOR_REQUIRED",
                "error": "operator_required",
                "message": "Operator unlock is required for force lease preemption"
            }))),
            None,
        ),
        Ok(LeaseOutcome::Released { .. } | LeaseOutcome::Disconnected { .. }) => (None, None),
        Err(ArmError::Rejected { code, .. }) if code == "DISCONNECTED" => {
            let _ = supervisor.disconnect_ws_connection(conn_handle).await;
            recv_generation.store(0, Ordering::SeqCst);
            (None, None)
        }
        Err(ArmError::Rejected { code, .. }) if code == "OWNER_TRANSFERRED" => (
            Some(with_seq(json!({
                "type": "lease_rejected",
                "error": "owner_transferred",
                "code": "OWNER_TRANSFERRED"
            }))),
            None,
        ),
        Err(err) => {
            let api = ApiError::from(err);
            (
                Some(with_seq(json!({
                    "type": "lease_rejected",
                    "status": api.status.as_u16(),
                    "code": api.code,
                    "error": api.error,
                    "message": api.message
                }))),
                None,
            )
        }
    }
}

/// Unified owner-routed WS `release_lease` shared by both legacy and RT executor owners.
pub(super) async fn handle_release_lease_unified(
    supervisor: &ArmSupervisor,
    value: &Value,
    conn_handle: crate::rt_core::requests::lease::ConnectionHandle,
    recv_generation: &AtomicU64,
    recv_client_label: &Mutex<Option<String>>,
) -> Option<Value> {
    use crate::rt_core::requests::lease::LeaseOutcome;
    let local_generation = recv_generation.load(Ordering::SeqCst);
    let supplied_generation = value.get("generation").and_then(Value::as_u64).unwrap_or(0);
    let label_matches = {
        let lbl = recv_client_label.lock().unwrap_or_else(|e| e.into_inner());
        value
            .get("client_id")
            .is_none_or(|id| id.as_str() == lbl.as_deref())
    };
    if local_generation == 0 || supplied_generation != local_generation || !label_matches {
        return Some(json!({"type":"lease_released", "success":false}));
    }
    match supervisor
        .release_lease_ws(conn_handle, local_generation)
        .await
    {
        Ok(LeaseOutcome::Released { released, .. }) => {
            if released {
                let _ = recv_generation.compare_exchange(
                    local_generation,
                    0,
                    Ordering::SeqCst,
                    Ordering::SeqCst,
                );
                *recv_client_label.lock().unwrap_or_else(|e| e.into_inner()) = None;
            }
            Some(json!({"type":"lease_released", "success":released}))
        }
        Ok(_) => Some(json!({"type":"lease_released", "success":false})),
        Err(ArmError::Rejected { code, .. }) if code == "DISCONNECTED" => {
            Some(json!({"type":"lease_released", "success":false}))
        }
        Err(err) => {
            let api = ApiError::from(err);
            Some(json!({
                "type": "lease_released",
                "success": false,
                "status": api.status.as_u16(),
                "code": api.code,
                "error": api.error,
                "message": api.message
            }))
        }
    }
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

/// Deliver a `lease_lost` (`reason: "preempted"`) notice to an evicted connection via the registry.
pub(super) fn notify_evicted(evicted_conn: &str, evicted_gen: u64) {
    let (sender_opt, closer_opt) = {
        let reg = CONNECTION_REGISTRY.lock().unwrap_or_else(|e| e.into_inner());
        let closers = CONNECTION_CLOSERS.lock().unwrap_or_else(|e| e.into_inner());
        (reg.get(evicted_conn).cloned(), closers.get(evicted_conn).cloned())
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
