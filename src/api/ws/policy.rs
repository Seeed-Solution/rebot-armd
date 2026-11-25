//! Policy start of the WS command path: the pending start and the loading guard (track D).
//!
//! Moved from `api/ws.rs` without changes (armd 1.6 RT-core foundation); moved items, their
//! fields and inherent methods are `pub(super)` so the connection loop keeps using them.

#[allow(unused_imports)]
use super::*;
#[cfg(feature = "policy-runtime")]
use super::dispatch::{
    AdmissionClass, AuthRequirement, DispatchHandler, DispatchKey, LeaseRequirement,
    OwnerDispatchCtx, ReplyPolicy,
};
use super::dispatch::DispatchEntry;

#[cfg(feature = "policy-runtime")]
pub(super) const ENTRIES: &[DispatchEntry] = &[
    DispatchEntry {
        key: DispatchKey::command("/api/v1/action/policy/start"),
        handler: DispatchHandler::Owner(dispatch_policy_start),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::Conditional(policy_start_auth_check),
        reply: ReplyPolicy::CommandDeferred,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/action/policy/stop"),
        handler: DispatchHandler::Owner(dispatch_policy_stop),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandImmediate,
    },
];

#[cfg(feature = "policy-runtime")]
fn policy_start_auth_check(
    ws_auth: &WsAuth,
    inner: &mut SupervisorInner,
    _path: &str,
    _body: &Value,
    _epoch: u64,
) -> Option<ApiError> {
    if !ws_auth.is_operator() && !inner.policy_live_allowed() {
        Some(ApiError::operator_required("policy/start (real arm)"))
    } else {
        None
    }
}

#[cfg(not(feature = "policy-runtime"))]
pub(super) const ENTRIES: &[DispatchEntry] = &[];

#[cfg(feature = "policy-runtime")]
fn dispatch_policy_start(ctx: &mut OwnerDispatchCtx<'_>) -> Option<Value> {
    let body = &ctx.value["body"];
    if !body.is_object() {
        return Some(command_reject(
            ctx.inner,
            &ctx.value["request_id"],
            invalid_command("Command body must be an object"),
        ));
    }
    match prepare_policy_start(ctx.inner, ctx.stamp, body, ctx.epoch) {
        Ok(pending) => {
            ctx.inner.policy_loading = true;
            let loading_guard = PolicyLoadingGuard::new(ctx.recv_supervisor.clone());
            ctx.action_pending.policy = Some((
                ctx.value["request_id"].clone(),
                pending,
                ctx.local_generation,
                loading_guard,
            ));
            None
        }
        Err(error) => Some(command_reject(ctx.inner, &ctx.value["request_id"], error)),
    }
}

#[cfg(feature = "policy-runtime")]
fn dispatch_policy_stop(ctx: &mut OwnerDispatchCtx<'_>) -> Option<Value> {
    let body = &ctx.value["body"];
    if !body.is_object() {
        return Some(command_reject(
            ctx.inner,
            &ctx.value["request_id"],
            invalid_command("Command body must be an object"),
        ));
    }
    let stopped = prepare_policy_stop(ctx.inner);
    Some(format_policy_stop_ack(
        ctx.inner,
        &ctx.value["request_id"],
        stopped,
        ctx.pending_policy_loads,
    ))
}

#[cfg(feature = "policy-runtime")]
pub(super) struct PendingPolicyStart {
    pub(super) request: crate::policy_runtime::PolicyStartRequest,
    pub(super) config: crate::model::ArmConfig,
    pub(super) epoch: u64,
    pub(super) stamp: crate::supervisor::AdmissionStamp,
    pub(super) policy_live_allowed: bool,
}

#[cfg(feature = "policy-runtime")]
pub(super) struct PolicyLoadingGuard(Option<Arc<ArmSupervisor>>);

#[cfg(feature = "policy-runtime")]
impl PolicyLoadingGuard {
    pub(super) fn new(sup: Arc<ArmSupervisor>) -> Self {
        Self(Some(sup))
    }

    pub(super) fn clear(&mut self, inner: &mut SupervisorInner) {
        if self.0.take().is_some() && !inner.detached {
            inner.policy_loading = false;
        }
    }
}

#[cfg(feature = "policy-runtime")]
impl Drop for PolicyLoadingGuard {
    fn drop(&mut self) {
        let Some(sup) = self.0.take() else {
            return;
        };
        if let Ok(mut inner) = sup.inner().try_write() {
            if !inner.detached {
                inner.policy_loading = false;
            }
            return;
        }
        match tokio::runtime::Handle::try_current() {
            Ok(h) => {
                h.spawn(async move {
                    let mut inner = sup.inner().write().await;
                    if !inner.detached {
                        inner.policy_loading = false;
                    }
                });
            }
            Err(_) => {
                let mut inner = sup.inner().blocking_write();
                if !inner.detached {
                    inner.policy_loading = false;
                }
            }
        }
        tracing::warn!("policy loading ended without explicit clear; cleared policy_loading");
    }
}

#[cfg(feature = "policy-runtime")]
pub(super) fn prepare_policy_start(
    inner: &mut SupervisorInner,
    stamp: crate::supervisor::AdmissionStamp,
    body: &Value,
    epoch: u64,
) -> Result<PendingPolicyStart, ApiError> {
    inner.preflight_admission(stamp).map_err(ArmError::from)?;
    inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
    let request = crate::policy_runtime::parse_policy_start_request(body).map_err(ArmError::from)?;
    let _ = crate::policy_runtime::compute_policy_deadline(request.max_seconds)
        .map_err(ArmError::from)?;
    inner.check_policy_start_preconditions_with(
        request.mode,
        request.sim_only,
        request.i_am_at_the_arm,
        epoch,
    )?;
    Ok(PendingPolicyStart {
        request,
        config: inner.config.clone(),
        epoch,
        stamp,
        policy_live_allowed: inner.policy_live_allowed(),
    })
}

#[cfg(feature = "policy-runtime")]
pub(super) fn prepare_policy_stop(inner: &mut SupervisorInner) -> bool {
    inner.advance_stop_generation();
    inner.stop_policy_run(None, "STOPPED")
}

#[cfg(feature = "policy-runtime")]
pub(super) fn format_policy_stop_ack(
    inner: &SupervisorInner,
    request_id: &Value,
    stopped: bool,
    pending_policy_loads: &AtomicUsize,
) -> Value {
    let had_pending = pending_policy_loads.load(Ordering::SeqCst) > 0;
    let mut ack = command_ack(inner, request_id, None);
    ack["stopped"] = json!(stopped || had_pending);
    ack
}

/// Spawn off-lock policy preparation (artifact load, release gate, warmup speed gate) and
/// owner-locked launch (`begin_policy_run` + `spawn_policy_runner`), sending the reply on `ack_tx`.
#[cfg(feature = "policy-runtime")]
#[allow(clippy::too_many_arguments)]
pub(super) fn spawn_policy_start(
    request_id: Value,
    pending: PendingPolicyStart,
    generation: u64,
    loading_guard: PolicyLoadingGuard,
    recv_conn_id: u64,
    recv_supervisor: &Arc<ArmSupervisor>,
    recv_sandbox: &Arc<crate::api::sandbox::SandboxState>,
    recv_deps: &crate::grasp::GraspDeps,
    recv_connection: &str,
    recv_generation: &Arc<AtomicU64>,
    ack_tx: &mpsc::Sender<Value>,
    worker_ws_closed: &Arc<AtomicBool>,
    pending_policy_loads: &Arc<AtomicUsize>,
    ws_auth: &WsAuth,
    worker_tasks: &ConnectionTasks,
) {
    let sup = recv_supervisor.clone();
    let sandbox_c = recv_sandbox.clone();
    let vision_src = recv_deps.source.clone();
    let conn = recv_connection.to_string();
    let gen_now = recv_generation.clone();
    let tx = ack_tx.clone();
    let ws_closed_c = worker_ws_closed.clone();
    let policy_stamp = pending.stamp;
    let pending_loads_c = pending_policy_loads.clone();
    pending_loads_c.fetch_add(1, Ordering::SeqCst);
    struct PendingLoadGuard(Arc<AtomicUsize>);
    impl Drop for PendingLoadGuard {
        fn drop(&mut self) {
            self.0.fetch_sub(1, Ordering::SeqCst);
        }
    }
    let load_guard = PendingLoadGuard(pending_loads_c);
    let epoch_id = pending.epoch;
    let policy_id = pending.request.policy_id.clone();
    let config = pending.config;
    let request_mode = pending.request.mode;
    let request_sim_only = pending.request.sim_only;
    let request_fps = pending.request.fps;
    let inline_release_gate = pending.request.release_gate;
    let policy_live_allowed = pending.policy_live_allowed;
    let i_am_at_the_arm = pending.request.i_am_at_the_arm;
    let ws_auth_c = ws_auth.clone();
    worker_tasks.spawn(async move {
        let _load_guard = load_guard;
        let mut loading_guard = loading_guard;
        let policy_id_load = policy_id.clone();
        let deployment =
            if request_mode == crate::policy_runtime::PolicyMode::Live && !policy_live_allowed {
                let (scene_id, tool_id) = sandbox_c.active_scene_and_tool().await;
                let calibration_digest = {
                    let inner = sup.inner().read().await;
                    inner.active_calibration_digest()
                };
                Some(crate::policy_runtime::ActiveDeploymentContext {
                    scene_id,
                    tool_id,
                    calibration_digest,
                })
            } else {
                None
            };
        let sandbox_load = sandbox_c.clone();
        let cancel_policy = ws_closed_c.clone();
        let plan_conn = conn.clone();
        let prepared = tokio::task::spawn_blocking(move || {
            maybe_pause_blocking_plan(&plan_conn);
            let delay_ms = sandbox_load.policy_load_delay_ms();
            if delay_ms > 0 && !sleep_cancellable_ms(delay_ms, &cancel_policy) {
                return Err(ArmError::Safety(SafetyError::Stopped));
            }
            if cancel_policy.load(Ordering::Acquire) {
                return Err(ArmError::Safety(SafetyError::Stopped));
            }
            let dir = sandbox_load.policy_dir(&policy_id_load).ok_or_else(|| ArmError::Rejected {
                status: 404,
                code: "POLICY_NOT_FOUND".into(),
                message: format!("Policy pack '{policy_id_load}' not found"),
            })?;
            let mut prepared_policy =
                crate::policy_runtime::prepare_policy(&dir, &config).map_err(ArmError::from)?;
            if cancel_policy.load(Ordering::Acquire) {
                return Err(ArmError::Safety(SafetyError::Stopped));
            }
            let infer_delay_ms = sandbox_load.policy_infer_delay_ms();
            if infer_delay_ms > 0 {
                prepared_policy = prepared_policy
                    .with_infer_delay(std::time::Duration::from_millis(infer_delay_ms));
            }
            let effective_fps = request_fps
                .or(prepared_policy.contract().fps)
                .unwrap_or(crate::policy_runtime::DEFAULT_POLICY_FPS);
            let effective_fps =
                crate::policy_runtime::validate_policy_fps(request_mode, effective_fps)
                    .map_err(ArmError::from)?;
            let verified_release_gate = if let Some(ref deployment_ctx) = deployment {
                let raw_gate = match inline_release_gate {
                    Some(v) => Some(v),
                    None => crate::policy_runtime::load_release_gate_json(&dir)
                        .map_err(ArmError::from)?,
                };
                match raw_gate {
                    Some(raw) => Some(
                        crate::policy_runtime::ReleaseGate::parse_and_verify(
                            &raw,
                            prepared_policy.artifact_digest(),
                            &config,
                            prepared_policy.contract(),
                            effective_fps,
                            deployment_ctx,
                        )
                        .map_err(ArmError::from)?,
                    ),
                    None => None,
                }
            } else {
                None
            };
            if cancel_policy.load(Ordering::Acquire) {
                return Err(ArmError::Safety(SafetyError::Stopped));
            }
            // speed gate: measured here, off the supervisor lock and the control tick
            let warmup = prepared_policy
                .warmup(
                    crate::policy_runtime::WARMUP_INFERENCES,
                    crate::policy_runtime::WARMUP_MAX_TOTAL,
                )
                .map_err(ArmError::from)?;
            if cancel_policy.load(Ordering::Acquire) {
                return Err(ArmError::Safety(SafetyError::Stopped));
            }
            let speed = crate::policy_runtime::SpeedGateReport::new(warmup, effective_fps);
            Ok::<_, ArmError>((prepared_policy, effective_fps, verified_release_gate, speed))
        })
        .await;

        if ws_closed_c.load(Ordering::SeqCst) {
            let mut inner = sup.inner().write().await;
            loading_guard.clear(&mut inner);
            return;
        }

        let reply = match prepared {
            Err(e) => {
                let mut inner = sup.inner().write().await;
                loading_guard.clear(&mut inner);
                command_reject(
                    &inner,
                    &request_id,
                    ArmError::Internal(format!("policy load task failed: {e}")).into(),
                )
            }
            Ok(Err(error)) => {
                let mut inner = sup.inner().write().await;
                loading_guard.clear(&mut inner);
                command_reject(&inner, &request_id, error.into())
            }
            Ok(Ok((prepared_policy, fps, verified_release_gate, speed))) => {
                let deadline_res =
                    crate::policy_runtime::compute_policy_deadline(pending.request.max_seconds)
                        .map_err(ArmError::from);
                // Fetch deployment context BEFORE taking the owner lock so NO `.await`
                // ever happens while holding `inner` and connection-closed is checked
                // atomically under the same lock as install.
                let active_scene_tool = if verified_release_gate.is_some() {
                    Some(sandbox_c.active_scene_and_tool().await)
                } else {
                    None
                };
                maybe_pause_worker_before_lock(&conn).await;

                let mut inner = sup.inner().write().await;
                loading_guard.clear(&mut inner);
                if ws_closed_c.load(Ordering::SeqCst) || inner.is_connection_closed(recv_conn_id) {
                    return;
                }
                if gen_now.load(Ordering::SeqCst) != generation
                    || !inner.validate_servo_lease(&conn, generation)
                {
                    json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"LEASE_LOST", "message":"This connection lost the lease while loading policy", "error":"LEASE_LOST", "epoch_id":inner.safety.epoch_id()})
                } else if inner.safety.epoch_id() != epoch_id {
                    json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"EPOCH_MISMATCH", "message":"Epoch changed while loading policy", "error":"EPOCH_MISMATCH", "epoch_id":inner.safety.epoch_id()})
                } else if !inner.policy_live_allowed() && !ws_auth_c.is_operator() {
                    // the operator session may have expired or been locked while loading
                    command_reject(
                        &inner,
                        &request_id,
                        ApiError::operator_required("policy/start (real arm)"),
                    )
                } else if let Err(err) = speed.check(request_mode) {
                    let mut reply = command_reject(&inner, &request_id, err.into());
                    for (k, v) in speed.ack_fields() {
                        reply[k] = v;
                    }
                    reply
                } else {
                    match deadline_res {
                        Err(err) => command_reject(&inner, &request_id, err.into()),
                        Ok(deadline) => {
                            let deployment_check = if let Some(ref gate) = verified_release_gate {
                                let (cur_scene, cur_tool) = active_scene_tool.unwrap_or_default();
                                let cur_ctx = crate::policy_runtime::ActiveDeploymentContext {
                                    scene_id: cur_scene,
                                    tool_id: cur_tool,
                                    calibration_digest: inner.active_calibration_digest(),
                                };
                                gate.verify_deployment(&cur_ctx).map_err(ArmError::from)
                            } else {
                                Ok(())
                            };
                            if let Err(err) = deployment_check {
                                command_reject(&inner, &request_id, err.into())
                            } else {
                                let start_req = crate::policy_runtime::PolicyRunStart {
                                    epoch_id,
                                    connection_id: conn,
                                    lease_generation: generation,
                                    policy_id: policy_id.clone(),
                                    artifact_digest: prepared_policy.artifact_digest().to_string(),
                                    backend: prepared_policy.backend_name().to_string(),
                                    mode: request_mode,
                                    sim_only: request_sim_only,
                                    max_first_step_rad: prepared_policy
                                        .contract()
                                        .max_first_step_rad,
                                    max_step_rad: prepared_policy.contract().max_step_rad,
                                    tag: pending.request.tag,
                                    i_am_at_the_arm,
                                    verified_release_gate,
                                };
                                match inner
                                    .check_admission(policy_stamp)
                                    .map_err(ArmError::from)
                                    .and_then(|admitted| {
                                        SupervisorInner::begin_policy_run(admitted, start_req)
                                    }) {
                                    Ok(action_id) => {
                                        let digest = prepared_policy.artifact_digest().to_string();
                                        let backend = prepared_policy.backend_name().to_string();
                                        let mode_str = pending.request.mode.as_str().to_string();
                                        let vision_source = prepared_policy
                                            .contract()
                                            .vision
                                            .as_ref()
                                            .map(|_| {
                                                Arc::new(
                                                    crate::policy_runtime::DetectionSourceAdapter(
                                                        vision_src.clone(),
                                                    ),
                                                )
                                                    as Arc<
                                                        dyn crate::policy_runtime::PolicyVisionSource,
                                                    >
                                            });
                                        crate::policy_runtime::spawn_policy_runner(
                                            sup.clone(),
                                            action_id,
                                            prepared_policy,
                                            fps,
                                            deadline,
                                            vision_source,
                                            Some(sandbox_c.clone()),
                                        );
                                        let mut ack = command_ack(
                                            &inner,
                                            &request_id,
                                            Some((
                                                action_id,
                                                pending.request.max_seconds.unwrap_or(0.0),
                                            )),
                                        );
                                        ack["policy_id"] = json!(policy_id);
                                        ack["artifact_digest"] = json!(digest);
                                        ack["backend"] = json!(backend);
                                        ack["mode"] = json!(mode_str);
                                        for (k, v) in speed.ack_fields() {
                                            ack[k] = v;
                                        }
                                        ack
                                    }
                                    Err(error) => command_reject(&inner, &request_id, error.into()),
                                }
                            }
                        }
                    }
                }
            }
        };
        let _ = tx.send(reply).await;
    });
}

#[cfg(feature = "policy-runtime")]
fn off_lock_policy_ack(sup: &ArmSupervisor, request_id: &Value, started: Started) -> Value {
    let snap = sup.hub().latest();
    let epoch_id = snap
        .as_ref()
        .map_or_else(|| sup.estop_epoch().max(1), |s| s.obs.epoch_id);
    let safety_state = snap
        .as_ref()
        .map_or_else(|| json!("Idle"), |s| json!(s.obs.safety_state));
    let is_torqued = snap.as_ref().is_some_and(|s| s.owner.is_torqued);
    let control_mode = if snap.as_ref().is_some_and(|s| s.owner.gravity_comp_active) {
        "gravity_comp"
    } else {
        "position"
    };
    let mut ack = json!({
        "type": "command_ack",
        "request_id": request_id,
        "status": "accepted",
        "epoch_id": epoch_id,
        "safety_state": safety_state,
        "is_torqued": is_torqued,
        "control_mode": control_mode,
    });
    if let Some((id, duration)) = started {
        ack["action_id"] = json!(id);
        ack["expected_duration_s"] = json!(duration);
    }
    ack
}

/// Owner-routed WS handler for `/api/v1/action/policy/start` and `/api/v1/action/policy/stop`
/// on `OWNER_EXECUTOR` (zero `SupervisorLock` reads/writes; zero RT allocations/frees/file I/O).
///
/// Returns `None` for any non-policy frame so `ws.rs` falls through untouched.
#[cfg(feature = "policy-runtime")]
#[allow(clippy::too_many_arguments)]
pub(super) async fn handle_executor_ws_policy(
    frame_kind: FrameKind,
    entry: Option<&'static DispatchEntry>,
    value: &Value,
    stamp: crate::supervisor::AdmissionStamp,
    recv_conn_id: u64,
    recv_connection: &str,
    recv_generation: &Arc<AtomicU64>,
    recv_client_label: &std::sync::Mutex<Option<String>>,
    last_ordinary_request: &mut Option<u64>,
    ws_auth: &WsAuth,
    recv_supervisor: &Arc<ArmSupervisor>,
    recv_sandbox: &Arc<crate::api::sandbox::SandboxState>,
    recv_deps: &crate::grasp::GraspDeps,
    ack_tx: &mpsc::Sender<Value>,
    worker_ws_closed: &Arc<AtomicBool>,
    pending_policy_loads: &Arc<AtomicUsize>,
    worker_tasks: &ConnectionTasks,
) -> Option<Option<Value>> {
    use crate::model::HolderLabel;
    use crate::rt_core::broker::{ActionId, AdmitError, Authority, ClientId, Reply};
    use crate::rt_core::executor::BrokerDispatchError;
    use crate::rt_core::requests::lease::ConnectionHandle;
    use crate::rt_core::requests::policy::{
        policy_code_to_api_error, BeginPolicySpec, EndPolicySpec, PolicyCommand,
        ReservePolicyLoadSpec,
    };

    if frame_kind != FrameKind::Command {
        return None;
    }
    let path = entry?.key.path?;
    let is_start = path == "/api/v1/action/policy/start";
    let is_stop = path == "/api/v1/action/policy/stop";
    if !is_start && !is_stop {
        return None;
    }

    let request_id_val = value.get("request_id").cloned().unwrap_or(Value::Null);
    let epoch = config::current_epoch_off_lock(recv_supervisor, 1);

    // 1. Lease authority check (off-lock, identical rules to legacy & PR C).
    let local_generation = recv_generation.load(Ordering::SeqCst);
    let supplied_generation = value.get("generation").and_then(Value::as_u64).unwrap_or(0);
    let label_matches = {
        let lbl = recv_client_label.lock().unwrap_or_else(|e| e.into_inner());
        value
            .get("client_id")
            .is_none_or(|id| id.as_str() == lbl.as_deref())
    };
    if local_generation == 0 || supplied_generation != local_generation || !label_matches {
        return Some(Some(json!({
            "type": "command_reject",
            "request_id": request_id_val,
            "status": 409,
            "code": "LEASE_LOST",
            "error": "LEASE_LOST",
            "message": "This connection does not hold the active lease",
            "epoch_id": epoch
        })));
    }

    // 2. Epoch check.
    if value.get("epoch_id").and_then(Value::as_u64) != Some(epoch) {
        return Some(Some(json!({
            "type": "command_reject",
            "request_id": request_id_val,
            "status": 409,
            "code": "EPOCH_MISMATCH",
            "message": "Command epoch is no longer current",
            "error": "EPOCH_MISMATCH",
            "epoch_id": epoch
        })));
    }

    // 3. Monotonic request_id check.
    let request_id = value.get("request_id").and_then(Value::as_u64);
    if request_id.is_none() || last_ordinary_request.is_some_and(|last| request_id.unwrap() <= last)
    {
        return Some(Some(json!({
            "type": "command_reject",
            "request_id": value.get("request_id"),
            "status": 400,
            "code": "INVALID_COMMAND",
            "message": "request_id must be present and strictly increase",
            "error": "INVALID_COMMAND"
        })));
    }
    *last_ordinary_request = request_id;
    let client_seq = request_id.unwrap_or(0);

    let body = &value["body"];
    if !body.is_object() {
        return Some(Some(config::off_lock_command_reject(
            &request_id_val,
            epoch,
            invalid_command("Command body must be an object"),
        )));
    }

    let Some(handle) = recv_supervisor.executor_handle() else {
        return Some(Some(config::off_lock_command_reject(
            &request_id_val,
            epoch,
            ApiError(
                axum::http::StatusCode::SERVICE_UNAVAILABLE,
                "RT executor is unavailable".into(),
                "EXECUTOR_UNAVAILABLE".into(),
            ),
        )));
    };
    let Some(conn_handle) = ConnectionHandle::try_new(recv_conn_id, recv_connection) else {
        return Some(Some(config::off_lock_command_reject(
            &request_id_val,
            epoch,
            policy_code_to_api_error("LEASE_LOST", epoch),
        )));
    };

    if is_stop {
        let had_pending = pending_policy_loads.load(Ordering::SeqCst) > 0;
        let authority = Authority {
            lease_generation: local_generation,
            epoch,
            stop_generation: recv_supervisor.current_stop_gen(),
            config_revision: 0,
            parent_generation: 0,
        };
        let cmd = PolicyCommand::EndPolicy(EndPolicySpec::Stop {
            connection: conn_handle,
            lease_generation: local_generation,
        });
        let res = handle
            .submit_policy_command_async(
                ClientId(recv_conn_id),
                client_seq,
                authority,
                Instant::now() + std::time::Duration::from_secs(2),
                cmd,
                |_| {},
            )
            .await;
        let post_epoch = config::current_epoch_off_lock(recv_supervisor, epoch);
        return Some(Some(match res {
            Ok(Reply::Ack { action, .. }) => {
                let mut ack = off_lock_policy_ack(recv_supervisor, &request_id_val, None);
                ack["stopped"] = json!(action.is_some() || had_pending);
                ack
            }
            Ok(Reply::Refused { code, .. }) => config::off_lock_command_reject(
                &request_id_val,
                post_epoch,
                policy_code_to_api_error(code, post_epoch),
            ),
            Ok(Reply::Lease { .. } | Reply::Terminal { .. }) | Err(BrokerDispatchError::Stopped) => {
                config::off_lock_command_reject(
                    &request_id_val,
                    post_epoch,
                    ApiError(
                        axum::http::StatusCode::SERVICE_UNAVAILABLE,
                        "RT executor stopped".into(),
                        "EXECUTOR_UNAVAILABLE".into(),
                    ),
                )
            }
            Err(BrokerDispatchError::Admit(AdmitError::Disconnected)) => {
                config::off_lock_command_reject(
                    &request_id_val,
                    post_epoch,
                    policy_code_to_api_error("LEASE_LOST", post_epoch),
                )
            }
            Err(BrokerDispatchError::Admit(_)) => config::off_lock_command_reject(
                &request_id_val,
                post_epoch,
                policy_code_to_api_error("MOTION_BUSY", post_epoch),
            ),
        }));
    }

    // `/api/v1/action/policy/start`:
    let Some(pub_cfg) = recv_supervisor.published_config_hub().latest() else {
        return Some(Some(config::off_lock_command_reject(
            &request_id_val,
            epoch,
            ApiError(
                axum::http::StatusCode::SERVICE_UNAVAILABLE,
                "Published configuration is unavailable".into(),
                "EXECUTOR_UNAVAILABLE".into(),
            ),
        )));
    };
    if !pub_cfg.policy_live_allowed && !ws_auth.is_operator() {
        return Some(Some(config::off_lock_command_reject(
            &request_id_val,
            epoch,
            ApiError::operator_required("policy/start (real arm)"),
        )));
    }
    let request = match crate::policy_runtime::parse_policy_start_request(body) {
        Ok(r) => r,
        Err(e) => {
            return Some(Some(config::off_lock_command_reject(
                &request_id_val,
                epoch,
                ArmError::from(e).into(),
            )));
        }
    };
    if let Err(e) = crate::policy_runtime::compute_policy_deadline(request.max_seconds) {
        return Some(Some(config::off_lock_command_reject(
            &request_id_val,
            epoch,
            ArmError::from(e).into(),
        )));
    }

    let token_guard = match recv_supervisor.policy_cancel_slots().claim_slot() {
        Ok(g) => g,
        Err(code) => {
            return Some(Some(config::off_lock_command_reject(
                &request_id_val,
                epoch,
                policy_code_to_api_error(code, epoch),
            )));
        }
    };

    let reserve_authority = Authority {
        lease_generation: local_generation,
        epoch,
        stop_generation: recv_supervisor.current_stop_gen(),
        config_revision: 0,
        parent_generation: 0,
    };
    let prepared_boot = handle.boot();
    let reserve_spec = ReservePolicyLoadSpec {
        load_token: token_guard.token(),
        cancel_slot: token_guard.slot(),
        admission_request_id: stamp.request_id,
        prepared_boot,
        connection: conn_handle,
        mode: request.mode,
        sim_only: request.sim_only,
        i_am_at_the_arm: request.i_am_at_the_arm,
        operator: ws_auth.operator_grant(),
        reservation_expires_at: Instant::now() + std::time::Duration::from_secs(30),
    };
    let reserve_res = handle
        .submit_policy_command_async(
            ClientId(recv_conn_id),
            client_seq,
            reserve_authority,
            Instant::now() + std::time::Duration::from_secs(2),
            PolicyCommand::ReserveLoad(reserve_spec),
            |req_id| token_guard.record_admitted_req(req_id.0),
        )
        .await;
    let post_epoch = config::current_epoch_off_lock(recv_supervisor, epoch);
    match reserve_res {
        Ok(Reply::Ack { .. }) => {}
        Ok(Reply::Refused { code, .. }) => {
            return Some(Some(config::off_lock_command_reject(
                &request_id_val,
                post_epoch,
                policy_code_to_api_error(code, post_epoch),
            )));
        }
        Ok(Reply::Lease { .. } | Reply::Terminal { .. }) | Err(BrokerDispatchError::Stopped) => {
            return Some(Some(config::off_lock_command_reject(
                &request_id_val,
                post_epoch,
                ApiError(
                    axum::http::StatusCode::SERVICE_UNAVAILABLE,
                    "RT executor stopped".into(),
                    "EXECUTOR_UNAVAILABLE".into(),
                ),
            )));
        }
        Err(BrokerDispatchError::Admit(AdmitError::Disconnected)) => {
            return Some(Some(config::off_lock_command_reject(
                &request_id_val,
                post_epoch,
                policy_code_to_api_error("LEASE_LOST", post_epoch),
            )));
        }
        Err(BrokerDispatchError::Admit(_)) => {
            return Some(Some(config::off_lock_command_reject(
                &request_id_val,
                post_epoch,
                policy_code_to_api_error("MOTION_BUSY", post_epoch),
            )));
        }
    }

    // Spawn off-thread artifact load, release-gate + qualification check, warmup speed gate,
    // `BeginPolicy` commit, and drop-safe `PolicyTokenGuard` handoff into the executor runner.
    let sup = recv_supervisor.clone();
    let sandbox_c = recv_sandbox.clone();
    let vision_src = recv_deps.source.clone();
    let conn = recv_connection.to_string();
    let gen_now = recv_generation.clone();
    let tx = ack_tx.clone();
    let ws_closed_c = worker_ws_closed.clone();
    let pending_loads_c = pending_policy_loads.clone();
    pending_loads_c.fetch_add(1, Ordering::SeqCst);
    struct PendingLoadGuard(Arc<AtomicUsize>);
    impl Drop for PendingLoadGuard {
        fn drop(&mut self) {
            self.0.fetch_sub(1, Ordering::SeqCst);
        }
    }
    let load_guard = PendingLoadGuard(pending_loads_c);
    let ws_auth_c = ws_auth.clone();
    let config_snap = pub_cfg.config.clone();
    let policy_live_allowed = pub_cfg.policy_live_allowed;
    let has_driver = pub_cfg.has_driver;
    let initial_cal_digest = pub_cfg.active_calibration_digest();
    let ledger_path = pub_cfg.policy_ledger_path.clone();
    let admission_request_id = stamp.request_id;

    worker_tasks.spawn(async move {
        let _load_guard = load_guard;
        // Guard remains armed across load/warmup/`BeginPolicy` until handed into the runner.
        let token_guard = token_guard;
        let policy_id = request.policy_id.clone();
        let request_mode = request.mode;
        let request_sim_only = request.sim_only;
        let request_fps = request.fps;
        let inline_release_gate = request.release_gate;
        let i_am_at_the_arm = request.i_am_at_the_arm;
        let max_seconds = request.max_seconds;

        let deployment = if request_mode == crate::policy_runtime::PolicyMode::Live
            && !policy_live_allowed
        {
            let (scene_id, tool_id) = sandbox_c.active_scene_and_tool().await;
            Some(crate::policy_runtime::ActiveDeploymentContext {
                scene_id,
                tool_id,
                calibration_digest: initial_cal_digest,
            })
        } else {
            None
        };

        let sandbox_load = sandbox_c.clone();
        let cancel_policy = ws_closed_c.clone();
        let cancel_slots_load = sup.policy_cancel_slots().clone();
        let cancel_slot_idx = token_guard.slot();
        let cancel_token_val = token_guard.token();
        let stop_requests_load = handle.stop_requests().clone();
        let initial_stop_gen = reserve_authority.stop_generation;
        let qual_coord_load = sup.policy_qualification().clone();
        let plan_conn = conn.clone();
        let policy_id_load = policy_id.clone();
        let backend_permit = match sup.policy_backend_permits().clone().try_acquire_owned() {
            Ok(p) => p,
            Err(_) => {
                let cur_ep = config::current_epoch_off_lock(&sup, epoch);
                let _ = tx
                    .send(config::off_lock_command_reject(
                        &request_id_val,
                        cur_ep,
                        policy_code_to_api_error("MOTION_BUSY", cur_ep),
                    ))
                    .await;
                return;
            }
        };
        let prepared = tokio::task::spawn_blocking(move || {
            let _backend_permit = backend_permit;
            let is_cancelled = || {
                cancel_policy.load(Ordering::Acquire)
                    || cancel_slots_load.is_cancelled(cancel_slot_idx, cancel_token_val)
                    || stop_requests_load.current() != initial_stop_gen
            };
            maybe_pause_blocking_plan(&plan_conn);
            let delay_ms = sandbox_load.policy_load_delay_ms();
            if delay_ms > 0 {
                let until = Instant::now() + std::time::Duration::from_millis(delay_ms);
                while Instant::now() < until {
                    if is_cancelled() {
                        return Err(ArmError::Safety(SafetyError::Stopped));
                    }
                    std::thread::sleep(std::time::Duration::from_millis(5));
                }
            }
            if is_cancelled() {
                return Err(ArmError::Safety(SafetyError::Stopped));
            }
            let dir = sandbox_load.policy_dir(&policy_id_load).ok_or_else(|| {
                ArmError::Rejected {
                    status: 404,
                    code: "POLICY_NOT_FOUND".into(),
                    message: format!("Policy pack '{policy_id_load}' not found"),
                }
            })?;
            let mut prepared_policy = crate::policy_runtime::prepare_policy(&dir, &config_snap)
                .map_err(ArmError::from)?;
            if is_cancelled() {
                return Err(ArmError::Safety(SafetyError::Stopped));
            }
            let infer_delay_ms = sandbox_load.policy_infer_delay_ms();
            if infer_delay_ms > 0 {
                prepared_policy = prepared_policy
                    .with_infer_delay(std::time::Duration::from_millis(infer_delay_ms));
            }
            let effective_fps = request_fps
                .or(prepared_policy.contract().fps)
                .unwrap_or(crate::policy_runtime::DEFAULT_POLICY_FPS);
            let effective_fps =
                crate::policy_runtime::validate_policy_fps(request_mode, effective_fps)
                    .map_err(ArmError::from)?;

            let verified_release_gate = if let Some(ref deployment_ctx) = deployment {
                let raw_gate = match inline_release_gate {
                    Some(v) => Some(v),
                    None => crate::policy_runtime::load_release_gate_json(&dir)
                        .map_err(ArmError::from)?,
                };
                match raw_gate {
                    Some(raw) => Some(
                        crate::policy_runtime::ReleaseGate::parse_and_verify(
                            &raw,
                            prepared_policy.artifact_digest(),
                            &config_snap,
                            prepared_policy.contract(),
                            effective_fps,
                            deployment_ctx,
                        )
                        .map_err(ArmError::from)?,
                    ),
                    None => None,
                }
            } else {
                None
            };

            let mut production_qualified_verified = false;
            if let Some(ref gate) = verified_release_gate {
                if let Some(ref emb) = prepared_policy.contract().video_embedding {
                    crate::policy_runtime::runner::verify_embedding_release_gate(gate, emb)
                        .map_err(|msg| ArmError::Rejected {
                            status: 403,
                            code: "RELEASE_GATE_INVALID".into(),
                            message: msg,
                        })?;
                }
                if gate.authorization_tier()
                    == crate::policy_runtime::AuthorizationTier::ProductionRelease
                {
                    let Some(ref path) = ledger_path else {
                        return Err(ArmError::Rejected {
                            status: 403,
                            code: "QUALIFICATION_REQUIRED".into(),
                            message: "production_release requires a device qualification ledger ($ARMD_POLICY_LEDGER or /userdata/rebot-teleop/policy_qualification.jsonl)".into(),
                        });
                    };
                    qual_coord_load.check_production_qualified_locked(path, gate)?;
                    production_qualified_verified = true;
                }
            }

            if is_cancelled() {
                return Err(ArmError::Safety(SafetyError::Stopped));
            }
            let warmup = prepared_policy
                .warmup(
                    crate::policy_runtime::WARMUP_INFERENCES,
                    crate::policy_runtime::WARMUP_MAX_TOTAL,
                )
                .map_err(ArmError::from)?;
            if is_cancelled() {
                return Err(ArmError::Safety(SafetyError::Stopped));
            }
            let speed = crate::policy_runtime::SpeedGateReport::new(warmup, effective_fps);
            Ok::<_, ArmError>((
                prepared_policy,
                effective_fps,
                verified_release_gate,
                production_qualified_verified,
                speed,
            ))
        })
        .await;

        if ws_closed_c.load(Ordering::SeqCst) {
            return;
        }
        let cur_epoch = config::current_epoch_off_lock(&sup, epoch);

        let reply = match prepared {
            Err(e) => config::off_lock_command_reject(
                &request_id_val,
                cur_epoch,
                ArmError::Internal(format!("policy load task failed: {e}")).into(),
            ),
            Ok(Err(error)) => {
                config::off_lock_command_reject(&request_id_val, cur_epoch, error.into())
            }
            Ok(Ok((
                prepared_policy,
                fps,
                verified_release_gate,
                production_qualified_verified,
                speed,
            ))) => {
                let deadline_res =
                    crate::policy_runtime::compute_policy_deadline(max_seconds)
                        .map_err(ArmError::from);
                let active_scene_tool = if verified_release_gate.is_some() {
                    Some(sandbox_c.active_scene_and_tool().await)
                } else {
                    None
                };
                maybe_pause_worker_before_lock(&conn).await;

                if ws_closed_c.load(Ordering::SeqCst) {
                    return;
                }
                let latest_epoch = config::current_epoch_off_lock(&sup, epoch);
                let Some(pub_cfg2) = sup.published_config_hub().latest() else {
                    let _ = tx
                        .send(config::off_lock_command_reject(
                            &request_id_val,
                            latest_epoch,
                            ApiError(
                                axum::http::StatusCode::SERVICE_UNAVAILABLE,
                                "Published configuration is unavailable".into(),
                                "EXECUTOR_UNAVAILABLE".into(),
                            ),
                        ))
                        .await;
                    return;
                };

                if gen_now.load(Ordering::SeqCst) != local_generation {
                    json!({
                        "type": "command_reject",
                        "request_id": request_id_val,
                        "status": 409,
                        "code": "LEASE_LOST",
                        "message": "This connection lost the lease while loading policy",
                        "error": "LEASE_LOST",
                        "epoch_id": latest_epoch
                    })
                } else if latest_epoch != epoch {
                    json!({
                        "type": "command_reject",
                        "request_id": request_id_val,
                        "status": 409,
                        "code": "EPOCH_MISMATCH",
                        "message": "Epoch changed while loading policy",
                        "error": "EPOCH_MISMATCH",
                        "epoch_id": latest_epoch
                    })
                } else if !pub_cfg2.policy_live_allowed && !ws_auth_c.is_operator() {
                    config::off_lock_command_reject(
                        &request_id_val,
                        latest_epoch,
                        ApiError::operator_required("policy/start (real arm)"),
                    )
                } else if let Err(err) = speed.check(request_mode) {
                    let mut reply = config::off_lock_command_reject(
                        &request_id_val,
                        latest_epoch,
                        err.into(),
                    );
                    for (k, v) in speed.ack_fields() {
                        reply[k] = v;
                    }
                    reply
                } else {
                    match deadline_res {
                        Err(err) => config::off_lock_command_reject(
                            &request_id_val,
                            latest_epoch,
                            err.into(),
                        ),
                        Ok(deadline) => {
                            let deployment_check = if let Some(ref gate) = verified_release_gate {
                                let (cur_scene, cur_tool) = active_scene_tool.unwrap_or_default();
                                let cur_ctx = crate::policy_runtime::ActiveDeploymentContext {
                                    scene_id: cur_scene,
                                    tool_id: cur_tool,
                                    calibration_digest: pub_cfg2.active_calibration_digest(),
                                };
                                gate.verify_deployment(&cur_ctx).map_err(ArmError::from)
                            } else {
                                Ok(())
                            };
                            if let Err(err) = deployment_check {
                                config::off_lock_command_reject(
                                    &request_id_val,
                                    latest_epoch,
                                    err.into(),
                                )
                            } else {
                                let (max_vision_age_ms, vision_stale_hold) = match (
                                    &prepared_policy.contract().video_embedding,
                                    &prepared_policy.contract().vision,
                                ) {
                                    (Some(emb), _) => (
                                        Some(emb.effective_staleness_ms_for_fps(fps)),
                                        emb.on_stale == crate::policy_runtime::VisionStalePolicy::Hold,
                                    ),
                                    (None, Some(spec)) => (
                                        Some(spec.max_staleness_ms),
                                        spec.on_stale == crate::policy_runtime::VisionStalePolicy::Hold,
                                    ),
                                    (None, None) => (None, false),
                                };
                                let (
                                    has_verified_release_gate,
                                    gate_policy_sha256,
                                    gate_calibration_digest,
                                    gate_max_speed_scale,
                                    gate_is_production_release,
                                ) = match verified_release_gate.as_ref() {
                                    Some(g) => (
                                        true,
                                        HolderLabel::new(g.policy_sha256()),
                                        HolderLabel::new(g.calibration_digest()),
                                        g.max_speed_scale(),
                                        g.authorization_tier()
                                            == crate::policy_runtime::AuthorizationTier::ProductionRelease,
                                    ),
                                    None => (
                                        false,
                                        HolderLabel::new(""),
                                        HolderLabel::new(""),
                                        1.0,
                                        false,
                                    ),
                                };
                                let begin_spec = BeginPolicySpec {
                                    load_token: token_guard.token(),
                                    cancel_slot: token_guard.slot(),
                                    admission_request_id,
                                    prepared_boot,
                                    connection: conn_handle,
                                    operator: ws_auth_c.operator_grant(),
                                    calibration_revision: pub_cfg2.calibration_revision,
                                    policy_id: HolderLabel::new(&policy_id),
                                    artifact_digest: HolderLabel::new(
                                        prepared_policy.artifact_digest(),
                                    ),
                                    backend: HolderLabel::new(prepared_policy.backend_name()),
                                    mode: request_mode,
                                    sim_only: request_sim_only,
                                    i_am_at_the_arm,
                                    max_first_step_rad: prepared_policy
                                        .contract()
                                        .max_first_step_rad
                                        .unwrap_or(crate::policy_runtime::DEFAULT_MAX_FIRST_STEP_RAD),
                                    max_step_rad: prepared_policy.contract().max_step_rad,
                                    max_input_age_ms: crate::policy_runtime::max_policy_input_age_ms(
                                        fps,
                                    ),
                                    max_vision_age_ms,
                                    vision_stale_hold,
                                    period_us: crate::policy_runtime::period_us_for_fps(fps),
                                    deadline,
                                    has_verified_release_gate,
                                    gate_policy_sha256,
                                    gate_calibration_digest,
                                    gate_max_speed_scale,
                                    gate_is_production_release,
                                    production_qualified_verified,
                                };
                                let begin_res = handle
                                    .submit_policy_command_async(
                                        ClientId(recv_conn_id),
                                        client_seq.wrapping_add(1),
                                        reserve_authority,
                                        Instant::now() + std::time::Duration::from_secs(2),
                                        PolicyCommand::BeginPolicy(begin_spec),
                                        |req_id| token_guard.record_admitted_req(req_id.0),
                                    )
                                    .await;
                                let post_ep = config::current_epoch_off_lock(&sup, latest_epoch);
                                match begin_res {
                                    Ok(Reply::Ack {
                                        action: Some(ActionId(action_id)),
                                        ..
                                    }) => {
                                        let digest =
                                            prepared_policy.artifact_digest().to_string();
                                        let backend = prepared_policy.backend_name().to_string();
                                        let mode_str = request_mode.as_str().to_string();
                                        let vision_source = prepared_policy
                                            .contract()
                                            .vision
                                            .as_ref()
                                            .map(|_| {
                                                Arc::new(
                                                    crate::policy_runtime::DetectionSourceAdapter(
                                                        vision_src.clone(),
                                                    ),
                                                )
                                                    as Arc<
                                                        dyn crate::policy_runtime::PolicyVisionSource,
                                                    >
                                            });
                                        // Register pending qualification fence BEFORE spawning runner
                                        // or sending WS ACK so no subsequent `production_release`
                                        // start can slip in before this supervised run's ledger write.
                                        let pending_qual_guard = if request_mode
                                            == crate::policy_runtime::PolicyMode::Live
                                            && !pub_cfg2.policy_live_allowed
                                        {
                                            verified_release_gate
                                                .as_ref()
                                                .filter(|g| {
                                                    g.authorization_tier()
                                                        == crate::policy_runtime::AuthorizationTier::SupervisedFirstRun
                                                })
                                                .map(|g| {
                                                    sup.policy_qualification()
                                                        .register_pending(action_id, g)
                                                })
                                        } else {
                                            None
                                        };
                                        // Hand ownership of `token_guard` and `pending_qual_guard`
                                        // directly into the runner BEFORE sending the WS ACK.
                                        crate::policy_runtime::spawn_executor_policy_runner(
                                            crate::policy_runtime::ExecutorPolicyRunParams {
                                                supervisor: sup.clone(),
                                                handle: handle.clone(),
                                                client: ClientId(recv_conn_id),
                                                action_id,
                                                lease_generation: local_generation,
                                                epoch_id: epoch,
                                                mode: request_mode,
                                                prepared: prepared_policy,
                                                fps,
                                                deadline,
                                                vision_source,
                                                sandbox: Some(sandbox_c.clone()),
                                                verified_release_gate,
                                                policy_live_allowed: pub_cfg2.policy_live_allowed,
                                                has_driver,
                                                token_guard,
                                                pending_qual_guard,
                                            },
                                        );
                                        let mut ack = off_lock_policy_ack(
                                            &sup,
                                            &request_id_val,
                                            Some((action_id, max_seconds.unwrap_or(0.0))),
                                        );
                                        ack["policy_id"] = json!(policy_id);
                                        ack["artifact_digest"] = json!(digest);
                                        ack["backend"] = json!(backend);
                                        ack["mode"] = json!(mode_str);
                                        for (k, v) in speed.ack_fields() {
                                            ack[k] = v;
                                        }
                                        ack
                                    }
                                    Ok(Reply::Refused { code, .. }) => {
                                        config::off_lock_command_reject(
                                            &request_id_val,
                                            post_ep,
                                            policy_code_to_api_error(code, post_ep),
                                        )
                                    }
                                    Ok(_) | Err(BrokerDispatchError::Stopped) => {
                                        config::off_lock_command_reject(
                                            &request_id_val,
                                            post_ep,
                                            ApiError(
                                                axum::http::StatusCode::SERVICE_UNAVAILABLE,
                                                "RT executor stopped".into(),
                                                "EXECUTOR_UNAVAILABLE".into(),
                                            ),
                                        )
                                    }
                                    Err(BrokerDispatchError::Admit(AdmitError::Disconnected)) => {
                                        config::off_lock_command_reject(
                                            &request_id_val,
                                            post_ep,
                                            policy_code_to_api_error("LEASE_LOST", post_ep),
                                        )
                                    }
                                    Err(BrokerDispatchError::Admit(_)) => {
                                        config::off_lock_command_reject(
                                            &request_id_val,
                                            post_ep,
                                            policy_code_to_api_error("MOTION_BUSY", post_ep),
                                        )
                                    }
                                }
                            }
                        }
                    }
                }
            }
        };
        let _ = tx.send(reply).await;
    });
    Some(None)
}
