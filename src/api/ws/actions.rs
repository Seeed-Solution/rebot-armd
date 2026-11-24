//! Prepared actions, Cartesian/joint/replay installs, macros, grasp dry-run, depth measurement,
//! and lease-free teach routing (family P).
//!
//! Moved from `api/ws.rs` and `api/ws/motion.rs` without behaviour changes (A1b PR 1).

#[allow(unused_imports)]
use super::*;
#[allow(dead_code)]
type _PlanInputImportKeep = PlanInput;
use super::dispatch::{
    AdmissionClass, AuthRequirement, DispatchEntry, DispatchHandler, DispatchKey, LeaseFreeCtx,
    LeaseRequirement, OwnerDispatchCtx, ReplyPolicy,
};

pub(super) const ENTRIES: &[DispatchEntry] = &[
    DispatchEntry {
        key: DispatchKey::command("/api/v1/action/move_to"),
        handler: DispatchHandler::Owner(dispatch_action_entry),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandDeferred,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/action/move_relative"),
        handler: DispatchHandler::Owner(dispatch_action_entry),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandDeferred,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/action/gripper"),
        handler: DispatchHandler::Owner(dispatch_action_entry),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandDeferred,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/action/gripper_zero_check"),
        handler: DispatchHandler::Owner(dispatch_action_entry),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::Conditional(gripper_zero_check_auth_check),
        reply: ReplyPolicy::CommandImmediate,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/action/grasp"),
        handler: DispatchHandler::Owner(dispatch_action_entry),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::RequiredExceptDryRun,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandDeferred,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/action/place"),
        handler: DispatchHandler::Owner(dispatch_action_entry),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandDeferred,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/action/replay"),
        handler: DispatchHandler::Owner(dispatch_action_entry),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandDeferred,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/action/move_to_pose"),
        handler: DispatchHandler::Owner(dispatch_action_entry),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandDeferred,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/action/run_sequence"),
        handler: DispatchHandler::Owner(dispatch_action_entry),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::Conditional(run_sequence_auth_check),
        reply: ReplyPolicy::CommandImmediate,
    },
    DispatchEntry {
        key: DispatchKey::command(crate::calib_hover::PATH),
        handler: DispatchHandler::Owner(dispatch_action_entry),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandImmediate,
    },
    DispatchEntry {
        key: DispatchKey::command(crate::gripper_calib::CALIBRATE_PATH),
        handler: DispatchHandler::Owner(dispatch_action_entry),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::Operator(Some("gripper_calibrate")),
        reply: ReplyPolicy::CommandImmediate,
    },
    DispatchEntry {
        key: DispatchKey::command(crate::measure_multiview::PATH),
        handler: DispatchHandler::Owner(dispatch_action_entry),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::Operator(None),
        reply: ReplyPolicy::CommandImmediate,
    },
    DispatchEntry {
        key: DispatchKey::command(crate::depth_measure::PATH),
        handler: DispatchHandler::LeaseFree(dispatch_depth_measure),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::None,
        auth: AuthRequirement::Operator(None),
        reply: ReplyPolicy::CommandDeferred,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/record/start"),
        handler: DispatchHandler::LeaseFree(dispatch_teach_lease_free),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::None,
        auth: AuthRequirement::Operator(None),
        reply: ReplyPolicy::CommandDeferred,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/record/stop"),
        handler: DispatchHandler::LeaseFree(dispatch_teach_lease_free),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::None,
        auth: AuthRequirement::ConditionalLeaseFree(record_stop_auth_check),
        reply: ReplyPolicy::CommandDeferred,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/episodes/update"),
        handler: DispatchHandler::LeaseFree(dispatch_teach_lease_free),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::None,
        auth: AuthRequirement::Operator(None),
        reply: ReplyPolicy::CommandDeferred,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/episodes/delete"),
        handler: DispatchHandler::LeaseFree(dispatch_teach_lease_free),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::None,
        auth: AuthRequirement::Operator(None),
        reply: ReplyPolicy::CommandDeferred,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/datasets/create"),
        handler: DispatchHandler::LeaseFree(dispatch_teach_lease_free),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::None,
        auth: AuthRequirement::Operator(None),
        reply: ReplyPolicy::CommandDeferred,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/datasets/delete"),
        handler: DispatchHandler::LeaseFree(dispatch_teach_lease_free),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::None,
        auth: AuthRequirement::Operator(None),
        reply: ReplyPolicy::CommandDeferred,
    },
];

fn run_sequence_auth_check(
    ws_auth: &WsAuth,
    _inner: &mut SupervisorInner,
    _path: &str,
    body: &Value,
    _epoch: u64,
) -> Option<ApiError> {
    if !ws_auth.is_operator() && body.get("record").is_some_and(|r| !r.is_null()) {
        Some(ApiError::operator_required("run_sequence (record)"))
    } else {
        None
    }
}

fn gripper_zero_check_auth_check(
    ws_auth: &WsAuth,
    _inner: &mut SupervisorInner,
    _path: &str,
    body: &Value,
    _epoch: u64,
) -> Option<ApiError> {
    if !ws_auth.is_operator()
        && body.get("store_offset").and_then(Value::as_bool) == Some(true)
    {
        Some(ApiError::operator_required(
            "gripper_zero_check (store_offset=true)",
        ))
    } else {
        None
    }
}

fn record_stop_auth_check(ws_auth: &WsAuth, path: &str, body: &Value) -> Option<ApiError> {
    if ws_auth.is_operator() {
        return None;
    }
    if body.get("save").and_then(Value::as_bool) == Some(false) {
        return None;
    }
    Some(ApiError::operator_required(path))
}

fn dispatch_action_entry(ctx: &mut OwnerDispatchCtx<'_>) -> Option<Value> {
    dispatch_action_command(
        ctx.inner,
        ctx.stamp,
        ctx.path,
        ctx.value,
        ctx.epoch,
        ctx.local_generation,
        ctx.ws_auth,
        ctx.recv_supervisor,
        ctx.recv_deps,
        ctx.recv_calib,
        ctx.action_pending,
    )
}

fn dispatch_depth_measure(ctx: LeaseFreeCtx<'_>) {
    let (sup, source, tx) = (
        ctx.recv_supervisor.clone(),
        ctx.recv_deps.source.clone(),
        ctx.ack_tx.clone(),
    );
    let value = ctx.value;
    ctx.worker_tasks.spawn(async move {
        let result = crate::depth_measure::measure(&sup, source.as_ref(), &value["body"]).await;
        let _ = tx.send(crate::teach::reply(&value["request_id"], result)).await;
    });
}

fn dispatch_teach_lease_free(ctx: LeaseFreeCtx<'_>) {
    let caller_label = ctx
        .recv_client_label
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone();
    let caller = crate::teach::Caller {
        connection: ctx.recv_connection.to_string(),
        client_id: ctx
            .value
            .get("client_id")
            .and_then(Value::as_str)
            .map(str::to_string)
            .or(caller_label),
    };
    let (sup, tx) = (ctx.recv_supervisor.clone(), ctx.ack_tx.clone());
    let value = ctx.value;
    ctx.worker_tasks.spawn(async move {
        let path = value["path"].as_str().unwrap_or("").to_string();
        let result = crate::teach::lease_free(&sup, &caller, &path, &value["body"]).await;
        let _ = tx.send(crate::teach::reply(&value["request_id"], result)).await;
    });
}

pub(super) enum ActionStart {
    Started(u64, f64),
    Plan(PendingPlan),
    Macro(PendingMacro),
    DryRunGrasp(Value),
    Sequence(Box<crate::sequences::RunPlan>),
    CalibHover(Box<crate::calib_hover::HoverPlan>),
    /// `gripper_calibrate` (epoch, tag): begun with the supervisor handle at the call site.
    GripperCalibrate(u64, Option<String>),
    MeasureMultiview(Box<crate::measure_multiview::Plan>),
}

/// Prepared A6 action kind routed through snapshot capture -> off-lock `prepare_*` ->
/// single owner-side `install_*`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum PendingPlanKind {
    MoveTo,
    MoveRelative,
    MoveToPose,
    Gripper,
    Replay,
}

/// A prepared action waiting for off-lock planning from a captured [`ActionSnapshot`].
pub(super) struct PendingPlan {
    pub(super) request_id: Value,
    pub(super) kind: PendingPlanKind,
    pub(super) snap: crate::rt_core::requests::actions::ActionSnapshot,
    pub(super) body: Value,
}

/// A macro action (crate::macro_action) checked under the lock, planned without it.
pub(super) enum PendingMacro {
    Grasp(crate::grasp::PendingGrasp),
    Place(crate::place::PendingPlace),
}

#[allow(clippy::large_enum_variant)]
pub(super) enum PlannedMacro {
    Grasp(crate::grasp::PlannedGrasp),
    Place(crate::place::PlannedPlace),
}

impl PendingMacro {
    pub(super) fn plan(self) -> Result<PlannedMacro, ArmError> {
        match self {
            Self::Grasp(g) => crate::grasp::plan(g).map(PlannedMacro::Grasp),
            Self::Place(p) => crate::place::plan(p).map(PlannedMacro::Place),
        }
    }
}

impl PlannedMacro {
    /// Begin the action and spawn its orchestrator. Returns (action id, expected duration).
    pub(super) fn start(
        self,
        inner: &mut SupervisorInner,
        stamp: crate::supervisor::AdmissionStamp,
        supervisor: &Arc<ArmSupervisor>,
        deps: &crate::grasp::GraspDeps,
    ) -> Result<(u64, f64), ArmError> {
        match self {
            Self::Grasp(planned) => {
                crate::grasp::begin_with_stamp(inner, stamp, supervisor, deps.source.clone(), planned)
            }
            Self::Place(planned) => crate::place::begin_with_stamp(inner, stamp, supervisor, planned),
        }
    }
}

/// Aborts a tracked child task handle when dropped.
pub(super) struct AbortOnDrop(pub(super) tokio::task::AbortHandle);

impl Drop for AbortOnDrop {
    fn drop(&mut self) {
        self.0.abort();
    }
}

/// A macro action being planned and checked (grasp preflight: model switch, up to ~7 s) in a
/// tracked connection task, aborted automatically on drop so preflight never outlives the socket.
pub(super) struct MacroCheck {
    pub(super) request_id: Value,
    pub(super) generation: u64,
    pub(super) stamp: crate::supervisor::AdmissionStamp,
    pub(super) abort: AbortOnDrop,
    pub(super) rx: tokio::sync::oneshot::Receiver<Result<PlannedMacro, ArmError>>,
}

// Under the write lock, after lease / epoch / request_id checks.
#[allow(clippy::too_many_arguments)]
pub(super) fn prepare_action(
    inner: &mut SupervisorInner,
    stamp: crate::supervisor::AdmissionStamp,
    path: &str,
    body: &Value,
    epoch: u64,
    request_id: &Value,
    auth: &crate::auth::OperatorAuth,
    ws_token: Option<&str>,
) -> Result<ActionStart, ApiError> {
    if !body.is_object() {
        return Err(invalid_command("Command body must be an object"));
    }
    match path {
        "/api/v1/action/move_to"
        | "/api/v1/action/move_relative"
        | "/api/v1/action/gripper"
        | "/api/v1/action/move_to_pose"
        | "/api/v1/action/replay" => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            let kind = match path {
                "/api/v1/action/move_to" => PendingPlanKind::MoveTo,
                "/api/v1/action/move_relative" => PendingPlanKind::MoveRelative,
                "/api/v1/action/gripper" => PendingPlanKind::Gripper,
                "/api/v1/action/move_to_pose" => PendingPlanKind::MoveToPose,
                "/api/v1/action/replay" => PendingPlanKind::Replay,
                _ => unreachable!(),
            };
            let snap = crate::rt_core::requests::actions::ActionSnapshot::from_inner(
                inner,
                stamp,
                std::time::Instant::now()
                    + crate::rt_core::requests::actions::DEFAULT_ACTION_DEADLINE,
            );
            Ok(ActionStart::Plan(PendingPlan {
                request_id: request_id.clone(),
                kind,
                snap,
                body: body.clone(),
            }))
        }
        "/api/v1/action/gripper_zero_check" => {
            let admitted = inner.check_admission(stamp).map_err(ArmError::from)?;
            let (id, duration) = SupervisorInner::start_gripper_zero_check_with_auth(
                admitted,
                body,
                epoch,
                Some((auth.clone(), ws_token.map(str::to_string))),
            )?;
            Ok(ActionStart::Started(id, duration))
        }
        "/api/v1/action/grasp" => {
            if body.get("dry_run").and_then(Value::as_bool) == Some(true) {
                return Ok(ActionStart::DryRunGrasp(body.clone()));
            }
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            Ok(ActionStart::Macro(PendingMacro::Grasp(
                crate::grasp::prepare(inner, body, epoch)?,
            )))
        }
        "/api/v1/action/place" => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            Ok(ActionStart::Macro(PendingMacro::Place(
                crate::place::prepare(inner, body, epoch)?,
            )))
        }
        "/api/v1/action/run_sequence" => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            Ok(ActionStart::Sequence(Box::new(
                crate::sequences::prepare_run(inner, body, epoch)?,
            )))
        }
        crate::calib_hover::PATH => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            Ok(ActionStart::CalibHover(Box::new(
                crate::calib_hover::prepare(inner, body, epoch)?,
            )))
        }
        crate::gripper_calib::CALIBRATE_PATH => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            let tag = crate::motion::tag(body).map_err(ArmError::from)?;
            Ok(ActionStart::GripperCalibrate(epoch, tag))
        }
        crate::measure_multiview::PATH => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            Ok(ActionStart::MeasureMultiview(Box::new(
                crate::measure_multiview::prepare(inner, body, epoch)?,
            )))
        }
        _ => Err(invalid_command("Unknown action")),
    }
}

/// Pending asynchronous action work produced by [`dispatch_action_command`].
pub(super) struct ActionPending {
    pub(super) plan: Option<(PendingPlan, u64)>,
    pub(super) macro_action: Option<(Value, PendingMacro, u64)>,
    pub(super) replay: Option<(Value, crate::replay::ReplayInput, u64)>,
    #[cfg(feature = "policy-runtime")]
    pub(super) policy: Option<(Value, PendingPolicyStart, u64, PolicyLoadingGuard)>,
    pub(super) dry_run_grasp: Option<(Value, Value)>,
}

/// Dispatch an `/api/v1/action/*` command under the supervisor write lock.
#[allow(clippy::too_many_arguments)]
pub(super) fn dispatch_action_command(
    inner: &mut SupervisorInner,
    stamp: crate::supervisor::AdmissionStamp,
    path: &str,
    value: &Value,
    epoch: u64,
    local_generation: u64,
    ws_auth: &WsAuth,
    recv_supervisor: &Arc<ArmSupervisor>,
    recv_deps: &crate::grasp::GraspDeps,
    recv_calib: &crate::calibration::CalibrationDeps,
    pending: &mut ActionPending,
) -> Option<Value> {
    let (auth_snap, token_snap) = ws_auth.auth_and_token();
    match prepare_action(
        inner,
        stamp,
        path,
        &value["body"],
        epoch,
        &value["request_id"],
        &auth_snap,
        token_snap.as_deref(),
    ) {
        Ok(ActionStart::Started(id, duration)) => {
            Some(command_ack(inner, &value["request_id"], Some((id, duration))))
        }
        Ok(ActionStart::Plan(plan)) => {
            pending.plan = Some((plan, local_generation));
            None
        }
        Ok(ActionStart::Macro(m)) => {
            pending.macro_action = Some((value["request_id"].clone(), m, local_generation));
            None
        }
        Ok(ActionStart::DryRunGrasp(body)) => {
            pending.dry_run_grasp = Some((value["request_id"].clone(), body));
            None
        }
        Ok(ActionStart::Sequence(plan)) => Some(
            match crate::sequences::begin_with_stamp(inner, stamp, recv_supervisor, *plan) {
                Ok(started) => command_ack(inner, &value["request_id"], Some(started)),
                Err(error) => command_reject(inner, &value["request_id"], error.into()),
            },
        ),
        Ok(ActionStart::CalibHover(plan)) => Some(
            match inner
                .check_admission(stamp)
                .map_err(ArmError::from)
                .and_then(|admitted| {
                    crate::calib_hover::begin(
                        admitted,
                        recv_supervisor,
                        *plan,
                        recv_calib.upstream.clone(),
                        crate::calib_hover::jobs_dir(&recv_calib.path),
                    )
                }) {
                Ok((id, duration, job_id)) => {
                    let mut ack = command_ack(inner, &value["request_id"], Some((id, duration)));
                    ack["job_id"] = json!(job_id);
                    ack
                }
                Err(error) => command_reject(inner, &value["request_id"], error.into()),
            },
        ),
        Ok(ActionStart::GripperCalibrate(epoch, tag)) => Some(
            match inner
                .check_admission(stamp)
                .map_err(ArmError::from)
                .and_then(|admitted| {
                    crate::gripper_calib::begin_calibration(admitted, recv_supervisor, epoch, tag)
                }) {
                Ok(started) => command_ack(inner, &value["request_id"], Some(started)),
                Err(error) => command_reject(inner, &value["request_id"], error.into()),
            },
        ),
        Ok(ActionStart::MeasureMultiview(plan)) => Some(
            match inner
                .check_admission(stamp)
                .map_err(ArmError::from)
                .and_then(|admitted| {
                    crate::measure_multiview::begin(
                        admitted,
                        recv_supervisor,
                        *plan,
                        recv_deps.source.clone(),
                    )
                }) {
                Ok(started) => command_ack(inner, &value["request_id"], Some(started)),
                Err(error) => command_reject(inner, &value["request_id"], error.into()),
            },
        ),
        Err(error) => Some(command_reject(inner, &value["request_id"], error)),
    }
}

/// Complete a joined `MacroCheck` (returns `None` when connection is closing).
#[allow(clippy::too_many_arguments)]
pub(super) async fn complete_macro_check(
    request_id: Value,
    generation: u64,
    stamp: crate::supervisor::AdmissionStamp,
    joined: Result<Result<PlannedMacro, ArmError>, tokio::sync::oneshot::error::RecvError>,
    recv_supervisor: &Arc<ArmSupervisor>,
    recv_connection: &str,
    recv_generation: &AtomicU64,
    recv_deps: &crate::grasp::GraspDeps,
    worker_closing: &AtomicBool,
) -> Option<Value> {
    let owner_transferred = || {
        let mut r = json!({"type":"command_reject", "request_id":request_id, "status":503, "code":"OWNER_TRANSFERRED", "error":"OWNER_TRANSFERRED", "message":"Ownership transferred to RT executor while planning macro"});
        if let Some(s) = recv_supervisor.hub().latest() {
            r["epoch_id"] = json!(s.obs.epoch_id);
        }
        r
    };
    if worker_closing.load(Ordering::SeqCst) {
        return None;
    }
    if recv_supervisor.owner_mode() != crate::supervisor::OWNER_LEGACY {
        return Some(owner_transferred());
    }
    let mut inner = recv_supervisor.inner().write().await;
    if worker_closing.load(Ordering::SeqCst) {
        drop(inner);
        return None;
    }
    if inner.detached {
        drop(inner);
        return Some(owner_transferred());
    }
    Some(if let Err(err) = inner.preflight_admission(stamp) {
        command_reject(&inner, &request_id, ArmError::from(err).into())
    } else {
        match joined {
            Err(e) => command_reject(
                &inner,
                &request_id,
                ArmError::Internal(format!("planner task failed: {e}")).into(),
            ),
            Ok(Err(error)) => command_reject(&inner, &request_id, error.into()),
            Ok(Ok(planned)) => {
                if recv_generation.load(Ordering::SeqCst) != generation
                    || !inner.validate_servo_lease(recv_connection, generation)
                {
                    json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"LEASE_LOST", "message":"This connection lost the lease while planning", "error":"LEASE_LOST", "epoch_id":inner.safety.epoch_id()})
                } else {
                    match planned.start(&mut inner, stamp, recv_supervisor, recv_deps) {
                        Ok(started) => command_ack(&inner, &request_id, Some(started)),
                        Err(error) => command_reject(&inner, &request_id, error.into()),
                    }
                }
            }
        }
    })
}

#[allow(clippy::large_enum_variant)]
enum PreparedWsAction {
    Cartesian(crate::rt_core::requests::actions::PreparedCartesianAction),
    Joint(crate::rt_core::requests::actions::PreparedJointAction),
    Gripper(crate::rt_core::requests::actions::PreparedGripperAction),
    Replay(crate::rt_core::requests::actions::PreparedReplayAction),
}

/// Complete any prepared A6 action (`move_to`, `move_relative`, `move_to_pose`, `gripper`,
/// `replay`) outside the lock via `rt_core::requests::actions::prepare_*`, then install under
/// a short write lock via the single owner-side `install_*` function.
/// Returns `None` if `worker_closing` requests loop termination; `Some(resp)` otherwise.
#[allow(clippy::too_many_arguments)]
pub(super) async fn complete_pending_plan(
    plan: PendingPlan,
    generation: u64,
    stamp: crate::supervisor::AdmissionStamp,
    recv_supervisor: &Arc<ArmSupervisor>,
    recv_connection: &str,
    recv_generation: &AtomicU64,
    worker_closing: &Arc<AtomicBool>,
) -> Option<Option<Value>> {
    use crate::rt_core::requests::actions as req;
    let PendingPlan {
        request_id,
        kind,
        snap,
        body,
    } = plan;
    let is_replay = kind == PendingPlanKind::Replay;
    let cancel = worker_closing.clone();
    let plan_conn = recv_connection.to_string();
    let prepared = tokio::task::spawn_blocking(move || {
        maybe_pause_blocking_plan(&plan_conn);
        if cancel.load(Ordering::Acquire) {
            return None;
        }
        let res: Result<PreparedWsAction, (ArmError, bool)> = match kind {
            PendingPlanKind::MoveTo => req::prepare_move_with_branch_hint(&snap, &body, false)
                .map(|m| match m {
                    req::PreparedMoveAction::Cartesian(c) => PreparedWsAction::Cartesian(c),
                    req::PreparedMoveAction::Joint(j) => PreparedWsAction::Joint(j),
                }),
            PendingPlanKind::MoveRelative => req::prepare_move_with_branch_hint(&snap, &body, true)
                .map(|m| match m {
                    req::PreparedMoveAction::Cartesian(c) => PreparedWsAction::Cartesian(c),
                    req::PreparedMoveAction::Joint(j) => PreparedWsAction::Joint(j),
                }),
            PendingPlanKind::MoveToPose => req::prepare_saved_pose(&snap, &body)
                .map(PreparedWsAction::Joint)
                .map_err(|e| (e, false)),
            PendingPlanKind::Gripper => {
                req::prepare_gripper(&snap, &body, crate::supervisor::GripperBaselineCmd::None)
                    .map(PreparedWsAction::Gripper)
                    .map_err(|e| (e, false))
            }
            PendingPlanKind::Replay => req::prepare_replay(&snap, &body)
                .map(PreparedWsAction::Replay)
                .map_err(|e| (e, false)),
        };
        if cancel.load(Ordering::Acquire) {
            return None;
        }
        Some(res)
    })
    .await;
    if worker_closing.load(Ordering::SeqCst) {
        return None;
    }
    let mut inner = recv_supervisor.inner().write().await;
    if worker_closing.load(Ordering::SeqCst) {
        return None;
    }
    if inner.detached {
        let msg = if is_replay {
            "Ownership transferred to RT executor while checking replay"
        } else {
            "Ownership transferred to RT executor while planning"
        };
        let mut r = json!({"type":"command_reject", "request_id":request_id, "status":503, "code":"OWNER_TRANSFERRED", "error":"OWNER_TRANSFERRED", "message":msg});
        if let Some(s) = recv_supervisor.hub().latest() {
            r["epoch_id"] = json!(s.obs.epoch_id);
        }
        return Some(Some(r));
    }
    if let Some(hook) = inner.before_install_hook.take() {
        hook(&mut inner);
    }
    if let Err(err) = inner.preflight_admission(stamp) {
        return Some(Some(command_reject(
            &inner,
            &request_id,
            ArmError::from(err).into(),
        )));
    }
    Some(match prepared {
        Ok(None) => None,
        Err(e) => {
            let label = if is_replay {
                "replay check task failed"
            } else {
                "planner task failed"
            };
            Some(command_reject(
                &inner,
                &request_id,
                ArmError::Internal(format!("{label}: {e}")).into(),
            ))
        }
        Ok(Some(Err((error, joint_hint)))) => {
            let mut api: ApiError = error.into();
            if joint_hint {
                api.message.push_str(crate::motion::JOINT_PATH_HINT);
            }
            Some(command_reject(&inner, &request_id, api))
        }
        Ok(Some(Ok(prepared))) => Some(
            if recv_generation.load(Ordering::SeqCst) != generation
                || !inner.validate_servo_lease(recv_connection, generation)
            {
                let msg = if is_replay {
                    "This connection lost the lease while the episode was checked"
                } else {
                    "This connection lost the lease while planning"
                };
                json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"LEASE_LOST", "message":msg, "error":"LEASE_LOST", "epoch_id":inner.safety.epoch_id()})
            } else {
                let ctx = req::OwnerCheckCtx::for_legacy(&inner);
                let res = match prepared {
                    PreparedWsAction::Cartesian(c) => {
                        let s = c.stamp;
                        req::install_move_cartesian(
                            &mut inner,
                            &s,
                            &ctx,
                            req::CartesianInstallPayload::Legacy(Box::new(c)),
                        )
                    }
                    PreparedWsAction::Joint(j) => {
                        let s = j.stamp;
                        req::install_move_joint(
                            &mut inner,
                            &s,
                            &ctx,
                            req::JointInstallPayload::Legacy(Box::new(j)),
                        )
                    }
                    PreparedWsAction::Gripper(g) => {
                        let s = g.stamp;
                        req::install_gripper(
                            &mut inner,
                            &s,
                            &ctx,
                            req::GripperInstallPayload::Legacy(Box::new(g)),
                        )
                    }
                    PreparedWsAction::Replay(r) => {
                        let s = r.stamp;
                        req::install_replay(
                            &mut inner,
                            &s,
                            &ctx,
                            req::ReplayInstallPayload::Legacy(Box::new(r)),
                        )
                    }
                };
                match res {
                    Ok(started) => command_ack(&inner, &request_id, Some(started)),
                    Err(error) => command_reject(&inner, &request_id, error.into()),
                }
            },
        ),
    })
}

/// Spawn a tracked macro planning/preflight task and return its [`MacroCheck`] handle.
#[allow(clippy::too_many_arguments)]
pub(super) fn spawn_macro_check(
    request_id: Value,
    pending: PendingMacro,
    generation: u64,
    stamp: crate::supervisor::AdmissionStamp,
    recv_deps: &crate::grasp::GraspDeps,
    worker_closing: &Arc<AtomicBool>,
    recv_connection: &str,
    worker_tasks: &ConnectionTasks,
) -> MacroCheck {
    let source = recv_deps.source.clone();
    let cancel = worker_closing.clone();
    let plan_conn = recv_connection.to_string();
    let (res_tx, res_rx) = tokio::sync::oneshot::channel();
    let abort = worker_tasks.spawn_abortable(async move {
        let planned_res = async {
            let cancel_blk = cancel.clone();
            let mut planned = tokio::task::spawn_blocking(move || {
                maybe_pause_blocking_plan(&plan_conn);
                if cancel_blk.load(Ordering::Acquire) {
                    return Err(ArmError::Safety(SafetyError::Stopped));
                }
                let out = pending.plan()?;
                if cancel_blk.load(Ordering::Acquire) {
                    return Err(ArmError::Safety(SafetyError::Stopped));
                }
                Ok(out)
            })
            .await
            .map_err(|e| ArmError::Internal(format!("planner task failed: {e}")))??;
            if cancel.load(Ordering::Acquire) {
                return Err(ArmError::Safety(SafetyError::Stopped));
            }
            // grasp task requirements: checked without the lock, before the action begins
            if let PlannedMacro::Grasp(g) = &mut planned {
                crate::grasp::preflight(source.as_ref(), g).await?;
            }
            if cancel.load(Ordering::Acquire) {
                return Err(ArmError::Safety(SafetyError::Stopped));
            }
            Ok(planned)
        }
        .await;
        if !cancel.load(Ordering::Acquire) {
            let _ = res_tx.send(planned_res);
        }
    });
    MacroCheck {
        request_id,
        generation,
        stamp,
        abort: AbortOnDrop(abort),
        rx: res_rx,
    }
}

/// Complete a replay check outside the lock and install under the lock.
/// Returns `None` if `worker_closing` requests loop termination; `Some(resp)` otherwise.
#[allow(clippy::too_many_arguments)]
pub(super) async fn complete_pending_replay(
    request_id: Value,
    input: crate::replay::ReplayInput,
    generation: u64,
    stamp: crate::supervisor::AdmissionStamp,
    recv_supervisor: &Arc<ArmSupervisor>,
    recv_connection: &str,
    recv_generation: &AtomicU64,
    worker_closing: &Arc<AtomicBool>,
) -> Option<Option<Value>> {
    let cancel = worker_closing.clone();
    let plan_conn = recv_connection.to_string();
    let planned = tokio::task::spawn_blocking(move || {
        maybe_pause_blocking_plan(&plan_conn);
        if cancel.load(Ordering::Acquire) {
            return None;
        }
        let out = crate::replay::plan(input);
        if cancel.load(Ordering::Acquire) {
            return None;
        }
        Some(out)
    })
    .await;
    if worker_closing.load(Ordering::SeqCst) {
        return None;
    }
    let mut inner = recv_supervisor.inner().write().await;
    if worker_closing.load(Ordering::SeqCst) {
        return None;
    }
    if inner.detached {
        let mut r = json!({"type":"command_reject", "request_id":request_id, "status":503, "code":"OWNER_TRANSFERRED", "error":"OWNER_TRANSFERRED", "message":"Ownership transferred to RT executor while checking replay"});
        if let Some(s) = recv_supervisor.hub().latest() {
            r["epoch_id"] = json!(s.obs.epoch_id);
        }
        Some(Some(r))
    } else if let Err(err) = inner.preflight_admission(stamp) {
        Some(Some(command_reject(
            &inner,
            &request_id,
            ArmError::from(err).into(),
        )))
    } else {
        Some(match planned {
            Ok(None) => None,
            Err(e) => Some(command_reject(
                &inner,
                &request_id,
                ArmError::Internal(format!("replay check task failed: {e}")).into(),
            )),
            Ok(Some(Err(error))) => Some(command_reject(&inner, &request_id, error.into())),
            Ok(Some(Ok(plan))) => Some(
                if recv_generation.load(Ordering::SeqCst) != generation
                    || !inner.validate_servo_lease(recv_connection, generation)
                {
                    json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"LEASE_LOST", "message":"This connection lost the lease while the episode was checked", "error":"LEASE_LOST", "epoch_id":inner.safety.epoch_id()})
                } else {
                    match inner
                        .check_admission(stamp)
                        .map_err(ArmError::from)
                        .and_then(|admitted| SupervisorInner::install_replay(admitted, plan))
                    {
                        Ok(started) => command_ack(&inner, &request_id, Some(started)),
                        Err(error) => command_reject(&inner, &request_id, error.into()),
                    }
                },
            ),
        })
    }
}

/// Complete a dry-run grasp check outside the lock.
pub(super) async fn complete_dry_run_grasp(
    request_id: Value,
    body: Value,
    recv_supervisor: &Arc<ArmSupervisor>,
    recv_deps: &crate::grasp::GraspDeps,
) -> Value {
    let dry = crate::grasp::dry_run_task(recv_supervisor, recv_deps.source.as_ref(), &body).await;
    let mut ack = {
        let inner = recv_supervisor.inner().read().await;
        if !inner.detached {
            command_ack(&inner, &request_id, None)
        } else if let Some(s) = recv_supervisor.hub().latest() {
            let control_mode = if s.obs.control_mode == crate::model::ControlMode::GravityComp {
                "gravity_comp"
            } else {
                "position"
            };
            json!({"type":"command_ack", "request_id":request_id, "status":"accepted", "epoch_id":s.obs.epoch_id, "safety_state":s.obs.safety_state, "is_torqued":s.obs.is_torqued, "control_mode":control_mode})
        } else {
            command_ack(&inner, &request_id, None)
        }
    };
    for (k, v) in dry.fields() {
        ack[k.as_str()] = v;
    }
    ack
}
