//! Policy start of the WS command path: the pending start and the loading guard (track D).
//!
//! Moved from `api/ws.rs` without changes (armd 1.6 RT-core foundation); moved items, their
//! fields and inherent methods are `pub(super)` so the connection loop keeps using them.

#[allow(unused_imports)]
use super::*;
#[cfg(feature = "policy-runtime")]
use super::dispatch::{
    AdmissionClass, AuthRequirement, DispatchHandler, DispatchKey, LeaseRequirement,
    OwnerDispatchCtx, OwnerReply, ReplyPolicy,
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
fn dispatch_policy_start(ctx: &mut OwnerDispatchCtx<'_>) -> Option<OwnerReply> {
    let body = &ctx.value["body"];
    if !body.is_object() {
        return Some(OwnerReply::CommandReject {
            epoch: ctx.inner.safety.epoch_id(),
            error: invalid_command("Command body must be an object"),
        });
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
        Err(error) => Some(OwnerReply::CommandReject {
            epoch: ctx.inner.safety.epoch_id(),
            error,
        }),
    }
}

#[cfg(feature = "policy-runtime")]
fn dispatch_policy_stop(ctx: &mut OwnerDispatchCtx<'_>) -> Option<OwnerReply> {
    let body = &ctx.value["body"];
    if !body.is_object() {
        return Some(OwnerReply::CommandReject {
            epoch: ctx.inner.safety.epoch_id(),
            error: invalid_command("Command body must be an object"),
        });
    }
    let stopped = prepare_policy_stop(ctx.inner);
    Some(OwnerReply::PolicyStopAck {
        snap: AckSnapshot::from_inner(ctx.inner),
        stopped,
    })
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

        enum PolicyStartOutcome {
            Closed,
            Reject { epoch: u64, error: ApiError },
            LeaseLost { epoch: u64 },
            EpochMismatch { epoch: u64 },
            RejectWithSpeed {
                epoch: u64,
                error: ApiError,
                speed: crate::policy_runtime::SpeedGateReport,
            },
            Started {
                snap: AckSnapshot,
                action_id: u64,
                prepared_policy: crate::policy_runtime::PreparedPolicy,
                fps: f64,
                deadline: Option<std::time::Instant>,
                speed: crate::policy_runtime::SpeedGateReport,
            },
        }

        let outcome = match prepared {
            Err(e) => {
                let epoch = {
                    let mut inner = sup.inner().write().await;
                    loading_guard.clear(&mut inner);
                    inner.safety.epoch_id()
                };
                PolicyStartOutcome::Reject {
                    epoch,
                    error: ArmError::Internal(format!("policy load task failed: {e}")).into(),
                }
            }
            Ok(Err(error)) => {
                let epoch = {
                    let mut inner = sup.inner().write().await;
                    loading_guard.clear(&mut inner);
                    inner.safety.epoch_id()
                };
                PolicyStartOutcome::Reject {
                    epoch,
                    error: error.into(),
                }
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
                    PolicyStartOutcome::Closed
                } else if gen_now.load(Ordering::SeqCst) != generation
                    || !inner.validate_servo_lease(&conn, generation)
                {
                    PolicyStartOutcome::LeaseLost {
                        epoch: inner.safety.epoch_id(),
                    }
                } else if inner.safety.epoch_id() != epoch_id {
                    PolicyStartOutcome::EpochMismatch {
                        epoch: inner.safety.epoch_id(),
                    }
                } else if !inner.policy_live_allowed() && !ws_auth_c.is_operator() {
                    // the operator session may have expired or been locked while loading
                    PolicyStartOutcome::Reject {
                        epoch: inner.safety.epoch_id(),
                        error: ApiError::operator_required("policy/start (real arm)"),
                    }
                } else if let Err(err) = speed.check(request_mode) {
                    PolicyStartOutcome::RejectWithSpeed {
                        epoch: inner.safety.epoch_id(),
                        error: err.into(),
                        speed,
                    }
                } else {
                    match deadline_res {
                        Err(err) => PolicyStartOutcome::Reject {
                            epoch: inner.safety.epoch_id(),
                            error: err.into(),
                        },
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
                                PolicyStartOutcome::Reject {
                                    epoch: inner.safety.epoch_id(),
                                    error: err.into(),
                                }
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
                                    Ok(action_id) => PolicyStartOutcome::Started {
                                        snap: AckSnapshot::from_inner(&inner),
                                        action_id,
                                        prepared_policy,
                                        fps,
                                        deadline,
                                        speed,
                                    },
                                    Err(error) => PolicyStartOutcome::Reject {
                                        epoch: inner.safety.epoch_id(),
                                        error: error.into(),
                                    },
                                }
                            }
                        }
                    }
                }
            }
        };
        // Write guard is now released; spawn runner and build every JSON reply strictly off-lock.
        let reply = match outcome {
            PolicyStartOutcome::Closed => return,
            PolicyStartOutcome::Reject { epoch, error } => {
                command_reject_at(epoch, &request_id, error)
            }
            PolicyStartOutcome::LeaseLost { epoch } => {
                json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"LEASE_LOST", "message":"This connection lost the lease while loading policy", "error":"LEASE_LOST", "epoch_id":epoch})
            }
            PolicyStartOutcome::EpochMismatch { epoch } => {
                json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"EPOCH_MISMATCH", "message":"Epoch changed while loading policy", "error":"EPOCH_MISMATCH", "epoch_id":epoch})
            }
            PolicyStartOutcome::RejectWithSpeed { epoch, error, speed } => {
                let mut reply = command_reject_at(epoch, &request_id, error);
                for (k, v) in speed.ack_fields() {
                    reply[k] = v;
                }
                reply
            }
            PolicyStartOutcome::Started {
                snap,
                action_id,
                prepared_policy,
                fps,
                deadline,
                speed,
            } => {
                let digest = prepared_policy.artifact_digest().to_string();
                let backend = prepared_policy.backend_name().to_string();
                let mode_str = pending.request.mode.as_str().to_string();
                let vision_source = prepared_policy
                    .contract()
                    .vision
                    .as_ref()
                    .map(|_| {
                        Arc::new(crate::policy_runtime::DetectionSourceAdapter(
                            vision_src.clone(),
                        )) as Arc<dyn crate::policy_runtime::PolicyVisionSource>
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
                let mut ack = command_ack_from_snap(
                    &snap,
                    &request_id,
                    Some((action_id, pending.request.max_seconds.unwrap_or(0.0))),
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
        };
        let _ = tx.send(reply).await;
    });
}
