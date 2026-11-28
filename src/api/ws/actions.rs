//! Prepared actions, Cartesian/joint/replay installs, macros, grasp dry-run, depth measurement,
//! and lease-free teach routing (family P).
//!
//! Moved from `api/ws.rs` and `api/ws/motion.rs` without behaviour changes (A1b PR 1).

#[allow(unused_imports)]
use super::*;
#[allow(dead_code)]
type _PlanInputImportKeep = PlanInput;
use super::dispatch::{
    AdmissionClass, AuthRequirement, DispatchEntry, DispatchHandler, DispatchKey, ExecutorWsRoute,
    LeaseFreeCtx, LeaseRequirement, OwnerDispatchCtx, OwnerReply, ReplyPolicy,
};
use crate::api::routes::{FaultAvailability, RouteOwnership};

pub(super) const ENTRIES: &[DispatchEntry] = &[
    DispatchEntry {
        key: DispatchKey::command("/api/v1/action/move_to"),
        handler: DispatchHandler::Owner(dispatch_action_entry),
        handler_name: "ws::actions::dispatch_action_entry",
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandDeferred,
        ownership: RouteOwnership::ExecutorRequest,
        fault: FaultAvailability::Fenced,
        executor_route: ExecutorWsRoute::Action,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/action/move_relative"),
        handler: DispatchHandler::Owner(dispatch_action_entry),
        handler_name: "ws::actions::dispatch_action_entry",
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandDeferred,
        ownership: RouteOwnership::ExecutorRequest,
        fault: FaultAvailability::Fenced,
        executor_route: ExecutorWsRoute::Action,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/action/gripper"),
        handler: DispatchHandler::Owner(dispatch_action_entry),
        handler_name: "ws::actions::dispatch_action_entry",
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandDeferred,
        ownership: RouteOwnership::ExecutorRequest,
        fault: FaultAvailability::Fenced,
        executor_route: ExecutorWsRoute::Action,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/action/gripper_zero_check"),
        handler: DispatchHandler::Owner(dispatch_action_entry),
        handler_name: "ws::actions::dispatch_action_entry",
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::Conditional(gripper_zero_check_auth_check),
        reply: ReplyPolicy::CommandImmediate,
        ownership: RouteOwnership::ExecutorRequest,
        fault: FaultAvailability::Fenced,
        executor_route: ExecutorWsRoute::Action,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/action/grasp"),
        handler: DispatchHandler::Owner(dispatch_action_entry),
        handler_name: "ws::actions::dispatch_action_entry",
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::RequiredExceptDryRun,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandDeferred,
        ownership: RouteOwnership::ExecutorRequest,
        fault: FaultAvailability::Fenced,
        executor_route: ExecutorWsRoute::Action,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/action/place"),
        handler: DispatchHandler::Owner(dispatch_action_entry),
        handler_name: "ws::actions::dispatch_action_entry",
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandDeferred,
        ownership: RouteOwnership::ExecutorRequest,
        fault: FaultAvailability::Fenced,
        executor_route: ExecutorWsRoute::Action,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/action/replay"),
        handler: DispatchHandler::Owner(dispatch_action_entry),
        handler_name: "ws::actions::dispatch_action_entry",
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandDeferred,
        ownership: RouteOwnership::ExecutorRequest,
        fault: FaultAvailability::Fenced,
        executor_route: ExecutorWsRoute::Action,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/action/move_to_pose"),
        handler: DispatchHandler::Owner(dispatch_action_entry),
        handler_name: "ws::actions::dispatch_action_entry",
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandDeferred,
        ownership: RouteOwnership::ExecutorRequest,
        fault: FaultAvailability::Fenced,
        executor_route: ExecutorWsRoute::Action,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/action/run_sequence"),
        handler: DispatchHandler::Owner(dispatch_action_entry),
        handler_name: "ws::actions::dispatch_action_entry",
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::Conditional(run_sequence_auth_check),
        reply: ReplyPolicy::CommandImmediate,
        ownership: RouteOwnership::ExecutorRequest,
        fault: FaultAvailability::Fenced,
        executor_route: ExecutorWsRoute::Action,
    },
    DispatchEntry {
        key: DispatchKey::command(crate::calib_hover::PATH),
        handler: DispatchHandler::Owner(dispatch_action_entry),
        handler_name: "ws::actions::dispatch_action_entry",
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandImmediate,
        ownership: RouteOwnership::ExecutorRequest,
        fault: FaultAvailability::Fenced,
        executor_route: ExecutorWsRoute::Action,
    },
    DispatchEntry {
        key: DispatchKey::command(crate::gripper_calib::CALIBRATE_PATH),
        handler: DispatchHandler::Owner(dispatch_action_entry),
        handler_name: "ws::actions::dispatch_action_entry",
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::Operator(Some("gripper_calibrate")),
        reply: ReplyPolicy::CommandImmediate,
        ownership: RouteOwnership::LegacyOnly,
        fault: FaultAvailability::Fenced,
        executor_route: ExecutorWsRoute::None,
    },
    DispatchEntry {
        key: DispatchKey::command(crate::measure_multiview::PATH),
        handler: DispatchHandler::Owner(dispatch_action_entry),
        handler_name: "ws::actions::dispatch_action_entry",
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::Operator(None),
        reply: ReplyPolicy::CommandImmediate,
        ownership: RouteOwnership::ExecutorRequest,
        fault: FaultAvailability::Fenced,
        executor_route: ExecutorWsRoute::Action,
    },
    DispatchEntry {
        key: DispatchKey::command(crate::depth_measure::PATH),
        handler: DispatchHandler::LeaseFree(dispatch_depth_measure),
        handler_name: "ws::actions::dispatch_depth_measure",
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::None,
        auth: AuthRequirement::Operator(None),
        reply: ReplyPolicy::CommandDeferred,
        ownership: RouteOwnership::ExecutorRequest,
        fault: FaultAvailability::Fenced,
        executor_route: ExecutorWsRoute::Unified,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/record/start"),
        handler: DispatchHandler::LeaseFree(dispatch_teach_lease_free),
        handler_name: "ws::actions::dispatch_teach_lease_free",
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::None,
        auth: AuthRequirement::Operator(None),
        reply: ReplyPolicy::CommandDeferred,
        ownership: RouteOwnership::ExecutorRequest,
        fault: FaultAvailability::Fenced,
        executor_route: ExecutorWsRoute::LeaseFree,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/record/stop"),
        handler: DispatchHandler::LeaseFree(dispatch_teach_lease_free),
        handler_name: "ws::actions::dispatch_teach_lease_free",
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::None,
        auth: AuthRequirement::ConditionalLeaseFree(record_stop_auth_check),
        reply: ReplyPolicy::CommandDeferred,
        ownership: RouteOwnership::ExecutorRequest,
        fault: FaultAvailability::Fenced,
        executor_route: ExecutorWsRoute::LeaseFree,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/episodes/update"),
        handler: DispatchHandler::LeaseFree(dispatch_teach_lease_free),
        handler_name: "ws::actions::dispatch_teach_lease_free",
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::None,
        auth: AuthRequirement::Operator(None),
        reply: ReplyPolicy::CommandDeferred,
        ownership: RouteOwnership::ExecutorRequest,
        fault: FaultAvailability::Fenced,
        executor_route: ExecutorWsRoute::LeaseFree,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/episodes/delete"),
        handler: DispatchHandler::LeaseFree(dispatch_teach_lease_free),
        handler_name: "ws::actions::dispatch_teach_lease_free",
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::None,
        auth: AuthRequirement::Operator(None),
        reply: ReplyPolicy::CommandDeferred,
        ownership: RouteOwnership::ExecutorRequest,
        fault: FaultAvailability::Fenced,
        executor_route: ExecutorWsRoute::LeaseFree,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/datasets/create"),
        handler: DispatchHandler::LeaseFree(dispatch_teach_lease_free),
        handler_name: "ws::actions::dispatch_teach_lease_free",
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::None,
        auth: AuthRequirement::Operator(None),
        reply: ReplyPolicy::CommandDeferred,
        ownership: RouteOwnership::ExecutorRequest,
        fault: FaultAvailability::Fenced,
        executor_route: ExecutorWsRoute::LeaseFree,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/datasets/delete"),
        handler: DispatchHandler::LeaseFree(dispatch_teach_lease_free),
        handler_name: "ws::actions::dispatch_teach_lease_free",
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::None,
        auth: AuthRequirement::Operator(None),
        reply: ReplyPolicy::CommandDeferred,
        ownership: RouteOwnership::ExecutorRequest,
        fault: FaultAvailability::Fenced,
        executor_route: ExecutorWsRoute::LeaseFree,
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

fn dispatch_action_entry(ctx: &mut OwnerDispatchCtx<'_>) -> Option<OwnerReply> {
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
    Plan(Box<PendingPlan>),
    Macro(Box<PendingMacro>),
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

impl PendingPlanKind {
    pub(super) const fn command_path(self) -> &'static str {
        match self {
            Self::MoveTo => "/api/v1/action/move_to",
            Self::MoveRelative => "/api/v1/action/move_relative",
            Self::MoveToPose => "/api/v1/action/move_to_pose",
            Self::Gripper => "/api/v1/action/gripper",
            Self::Replay => "/api/v1/action/replay",
        }
    }
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
    pub(super) const fn command_path(&self) -> &'static str {
        match self {
            Self::Grasp(_) => "/api/v1/action/grasp",
            Self::Place(_) => "/api/v1/action/place",
        }
    }

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
    pub(super) command_path: &'static str,
    pub(super) generation: u64,
    pub(super) stamp: crate::supervisor::AdmissionStamp,
    /// True when admitted on `OWNER_EXECUTOR` (0 `SupervisorLock`, commits via `begin_executor`).
    pub(super) executor: bool,
    pub(super) abort: AbortOnDrop,
    pub(super) rx: tokio::sync::oneshot::Receiver<Result<PlannedMacro, ArmError>>,
}

/// Dispatch result for Executor-owner `/api/v1/action/*`, `calib_hover`, and `measure_multiview` WS commands.
pub(super) enum ExecutorActionDispatch {
    Reply {
        reply: Value,
        gate: Option<CommandReplyGateGuard>,
        ack_guard: Option<crate::rt_core::reply_router::AckCommitGuard>,
    },
    Macro(MacroCheck),
    Closed,
}

fn off_lock_action_reject(request_id: &Value, epoch: u64, error: ApiError) -> Value {
    let mut r = rejection(error, "command_reject", epoch);
    r["request_id"] = request_id.clone();
    r
}

fn off_lock_action_ack(
    supervisor: &ArmSupervisor,
    request_id: &Value,
    started: Started,
    fallback_epoch: u64,
) -> Value {
    let (epoch_id, safety_state, is_torqued, control_mode) =
        if let Some(s) = supervisor.hub().latest() {
            let cm = if s.obs.control_mode == crate::model::ControlMode::GravityComp {
                "gravity_comp"
            } else {
                "position"
            };
            (s.obs.epoch_id, json!(s.obs.safety_state), s.obs.is_torqued, cm)
        } else {
            (fallback_epoch, json!("unknown"), false, "position")
        };
    let mut v = json!({
        "type": "command_ack",
        "request_id": request_id,
        "status": "accepted",
        "epoch_id": epoch_id,
        "safety_state": safety_state,
        "is_torqued": is_torqued,
        "control_mode": control_mode,
    });
    if let Some((id, duration)) = started {
        v["action_id"] = json!(id);
        v["expected_duration_s"] = json!(duration);
    }
    v
}

async fn submit_prepared_executor_ws(
    sup: &Arc<ArmSupervisor>,
    conn_id: u64,
    prepared: PreparedWsAction,
    wire_tracker: Arc<crate::rt_core::reply_router::WsWireAckTracker>,
) -> Result<((u64, f64), crate::rt_core::reply_router::AckCommitGuard), ArmError> {
    use crate::rt_core::broker::{AdmitError, ClientId, Reply};
    use crate::rt_core::ops::PlanSlot;
    use crate::rt_core::reply_router::{ExternalDestinationPolicy, ExternalSubmitError};
    use crate::rt_core::requests::actions::{
        ActionRefusal, ActionSubmit, PreparedActionKind, DEFAULT_ACTION_DEADLINE,
    };

    let handle = sup.executor_handle().ok_or_else(|| ArmError::Rejected {
        status: 503,
        code: "EXECUTOR_UNAVAILABLE".into(),
        message: "RT executor is not active".into(),
    })?;
    // Ownership of `persist_ctx_id` stays with this guard ONLY until `submit_external_action`
    // succeeds; once admitted to the broker, `BrokerDispatcher` owns refusal reclamation and
    // `armd-control` / `armd-persist` owns post-ACK reclamation so dropping this WS future
    // mid-await can never delete a live action's context.
    struct ZeroCheckPreSubmitGuard {
        handle: crate::rt_core::executor::RtExecutorHandle,
        ctx_id: u64,
    }
    impl ZeroCheckPreSubmitGuard {
        fn disarm(&mut self) {
            self.ctx_id = 0;
        }
    }
    impl Drop for ZeroCheckPreSubmitGuard {
        fn drop(&mut self) {
            if self.ctx_id != 0 {
                self.handle.remove_zero_check_context(self.ctx_id);
            }
        }
    }
    let persist_ctx_id = match &prepared {
        PreparedWsAction::GripperZeroCheck { persist_ctx_id, .. } => *persist_ctx_id,
        _ => 0,
    };
    let mut zc_guard = ZeroCheckPreSubmitGuard {
        handle: handle.clone(),
        ctx_id: persist_ctx_id,
    };
    let (
        slot,
        kind,
        stamp,
        check_workspace_revision,
        poses_revision,
        calibration_revision,
        gripper_calib_revision,
        power_budget_revision,
        expected_duration_s,
    ) = match prepared {
        PreparedWsAction::Cartesian(c) => {
            let slot = c
                .to_plan_slot(None)
                .map_err(|code| ActionRefusal::new(code).into_arm_error_with_epoch(c.stamp.admission.estop_epoch))?;
            (
                slot,
                c.kind,
                c.stamp,
                true,
                None,
                c.stamp.calibration_revision,
                c.stamp.gripper_calib_revision,
                c.stamp.power_budget_revision,
                c.expected_duration_s,
            )
        }
        PreparedWsAction::Joint(j) => {
            let slot = j
                .to_plan_slot(None)
                .map_err(|code| ActionRefusal::new(code).into_arm_error_with_epoch(j.stamp.admission.estop_epoch))?;
            (
                slot,
                j.kind,
                j.stamp,
                true,
                j.stamp.poses_revision,
                j.stamp.calibration_revision,
                j.stamp.gripper_calib_revision,
                j.stamp.power_budget_revision,
                j.expected_duration_s,
            )
        }
        PreparedWsAction::Gripper(g) => {
            let slot = g
                .to_plan_slot(None)
                .map_err(|code| ActionRefusal::new(code).into_arm_error_with_epoch(g.stamp.admission.estop_epoch))?;
            (
                slot,
                PreparedActionKind::Gripper,
                g.stamp,
                false,
                None,
                g.stamp.calibration_revision,
                g.stamp.gripper_calib_revision,
                g.stamp.power_budget_revision,
                g.expected_duration_s,
            )
        }
        PreparedWsAction::GripperZeroCheck {
            prepared: gz,
            persist_ctx_id,
        } => {
            let slot = gz.to_plan_slot(persist_ctx_id).map_err(|code| {
                ActionRefusal::new(code).into_arm_error_with_epoch(gz.stamp.admission.estop_epoch)
            })?;
            (
                slot,
                PreparedActionKind::GripperZeroCheck,
                gz.stamp,
                false,
                None,
                gz.stamp.calibration_revision,
                gz.stamp.gripper_calib_revision,
                gz.stamp.power_budget_revision,
                gz.expected_duration_s,
            )
        }
        PreparedWsAction::Replay(r) => {
            let slot = r
                .to_plan_slot(None)
                .map_err(|code| ActionRefusal::new(code).into_arm_error_with_epoch(r.stamp.admission.estop_epoch))?;
            (
                slot,
                PreparedActionKind::Replay,
                r.stamp,
                true,
                None,
                r.stamp.calibration_revision,
                r.stamp.gripper_calib_revision,
                r.stamp.power_budget_revision,
                r.expected_duration_s,
            )
        }
    };

    let client = ClientId(conn_id);
    let auth = stamp.broker_authority();
    let submit_deadline = stamp.deadline.max(Instant::now() + DEFAULT_ACTION_DEADLINE);
    let prepared_boot = stamp.boot;
    let admission_request_id = stamp.admission.request_id;
    let retry_until = Instant::now() + Duration::from_millis(50);

    let (_req_id, mut rx, ack_guard) = loop {
        let write = |dst: &mut PlanSlot| *dst = slot.clone();
        match handle.submit_external_action_with_zero_check_ctx(
            client,
            0,
            auth,
            submit_deadline,
            ActionSubmit::InstallPrepared {
                write: &write,
                kind,
                prepared_boot,
                admission_request_id,
                check_workspace_revision,
                poses_revision,
                calibration_revision,
                gripper_calib_revision,
                power_budget_revision,
            },
            ExternalDestinationPolicy::ConnectionOnly,
            true,
            Some(Arc::clone(&wire_tracker)),
            persist_ctx_id,
        ) {
            Ok(triple) => {
                // Ownership transferred atomically to BrokerDispatcher -> RT owner / PR C.
                zc_guard.disarm();
                break triple;
            }
            Err(ExternalSubmitError::NoDestination) => {
                return Err(ArmError::Rejected {
                    status: 503,
                    code: "EXECUTOR_ROUTING_PENDING".into(),
                    message: "RT executor has no per-connection reply route bound".into(),
                });
            }
            Err(ExternalSubmitError::Stopped) => {
                return Err(ArmError::Rejected {
                    status: 503,
                    code: "EXECUTOR_UNAVAILABLE".into(),
                    message: "RT executor stopped before admitting prepared action".into(),
                });
            }
            Err(ExternalSubmitError::NotAnExternalAction) => {
                return Err(ArmError::Internal("InstallPrepared rejected as non-external".into()));
            }
            Err(ExternalSubmitError::Admit(AdmitError::Disconnected)) => {
                return Err(ActionRefusal::new("LEASE_LOST").into_arm_error_with_epoch(auth.epoch));
            }
            Err(ExternalSubmitError::Admit(AdmitError::RepliesFull)) => {
                handle.pump_broker_replies();
                if Instant::now() >= retry_until || Instant::now() > submit_deadline {
                    return Err(ActionRefusal::new("TERMINALS_BUSY")
                        .into_arm_error_with_epoch(auth.epoch));
                }
                tokio::time::sleep(Duration::from_millis(2)).await;
            }
            Err(ExternalSubmitError::Admit(AdmitError::CommandsFull | AdmitError::PlansFull)) => {
                handle.pump_broker_replies();
                let now = Instant::now();
                if now > submit_deadline {
                    return Err(ActionRefusal::new("DEADLINE_EXPIRED")
                        .into_arm_error_with_epoch(auth.epoch));
                }
                if now >= retry_until {
                    return Err(ArmError::Rejected {
                        status: 503,
                        code: "MOTION_BUSY".into(),
                        message: "RT executor command or plan queue is saturated".into(),
                    });
                }
                tokio::time::sleep(Duration::from_millis(2)).await;
            }
        }
    };

    let wait_limit = submit_deadline + Duration::from_secs(2);
    loop {
        handle.pump_broker_replies();
        let remaining = wait_limit.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            drop(ack_guard);
            return Err(ActionRefusal::new("DEADLINE_EXPIRED").into_arm_error_with_epoch(auth.epoch));
        }
        let step = remaining.min(Duration::from_millis(2));
        match tokio::time::timeout(step, &mut rx).await {
            Ok(Ok(Ok(Reply::Ack { action: Some(id), .. }))) => {
                return Ok(((id.0, expected_duration_s), ack_guard));
            }
            Ok(Ok(Ok(Reply::Ack { action: None, .. }))) => {
                drop(ack_guard);
                return Err(ArmError::Internal("InstallPrepared ACK omitted ActionId".into()));
            }
            Ok(Ok(Ok(Reply::Refused { code, detail, .. }))) => {
                drop(ack_guard);
                let latest = sup.hub().latest();
                let cur_epoch = latest
                    .as_ref()
                    .map_or(auth.epoch, |s| s.rt.owner.authority.epoch);
                let ws_detail = (code == "WORKSPACE_VIOLATION")
                    .then(|| latest.as_ref().and_then(|s| s.obs.workspace_violation.clone()))
                    .flatten();
                let lims = sup.hub().prep_env().map(|e| e.config.limits.clone());
                return Err(ActionRefusal::new(code)
                    .into_arm_error_with_workspace(cur_epoch, detail, lims.as_deref(), ws_detail));
            }
            Ok(Ok(Ok(Reply::Terminal { .. } | Reply::Lease { .. }))) => {
                drop(ack_guard);
                return Err(ArmError::Internal("unexpected reply variant for InstallPrepared".into()));
            }
            Ok(Ok(Err(_))) | Ok(Err(_)) => {
                drop(ack_guard);
                return Err(ArmError::Rejected {
                    status: 503,
                    code: "EXECUTOR_UNAVAILABLE".into(),
                    message: "RT executor stopped before acknowledging prepared action".into(),
                });
            }
            Err(_) => {}
        }
    }
}

/// Executor-owner WebSocket router for family P prepared actions and macro actions (P-F1):
/// - Prepared actions: `/api/v1/action/move_to`, `move_relative`, `move_to_pose`, `gripper`, `replay`
/// - Macro actions: `/api/v1/action/grasp` (`dry_run: true`, live `adaptive` and `legacy`),
///   `/api/v1/action/place`, `/api/v1/action/run_sequence`, `/api/v1/calib/hover`,
///   `/api/v1/measure/multiview`
///
/// Every external action binds its terminal destination atomically to this connection's per-connection
/// reply slot (`ExternalDestinationPolicy::ConnectionOnly`) and enforces the wire-ACK enqueue barrier
/// via `CommandReplyGateGuard` + `AckCommitGuard`.
///
/// Returns `None` for any non-P path (leaving M's `/api/v1/motion/jog`, C's calibration, S, Y untouched).
#[allow(clippy::too_many_arguments)]
pub(super) async fn handle_executor_ws_action(
    frame_kind: FrameKind,
    entry: Option<&'static DispatchEntry>,
    value: &Value,
    stamp: crate::supervisor::AdmissionStamp,
    recv_conn_id: u64,
    recv_connection: &str,
    recv_generation: &Arc<AtomicU64>,
    recv_client_label: &Mutex<Option<String>>,
    last_ordinary_request: &mut Option<u64>,
    ws_auth: &WsAuth,
    recv_supervisor: &Arc<ArmSupervisor>,
    recv_deps: &crate::grasp::GraspDeps,
    recv_calib: &crate::calibration::CalibrationDeps,
    worker_closing: &Arc<AtomicBool>,
    command_reply_pending: &Arc<AtomicBool>,
    wire_ack_tracker: &Arc<crate::rt_core::reply_router::WsWireAckTracker>,
    worker_tasks: &ConnectionTasks,
) -> Option<ExecutorActionDispatch> {
    if frame_kind != FrameKind::Command {
        return None;
    }
    let entry = entry?;
    let path = entry.key.path?;
    let is_prepared = matches!(
        path,
        "/api/v1/action/move_to"
            | "/api/v1/action/move_relative"
            | "/api/v1/action/move_to_pose"
            | "/api/v1/action/gripper"
            | "/api/v1/action/replay"
    );
    let is_gripper_zero_check = path == "/api/v1/action/gripper_zero_check";
    let is_macro = path == "/api/v1/action/grasp"
        || path == "/api/v1/action/place"
        || path == "/api/v1/action/run_sequence"
        || path == crate::calib_hover::PATH
        || path == crate::measure_multiview::PATH;
    if !is_prepared && !is_gripper_zero_check && !is_macro {
        return None;
    }

    let body = &value["body"];
    let request_id_val = value.get("request_id").cloned().unwrap_or(Value::Null);
    let epoch = recv_supervisor
        .hub()
        .latest()
        .map_or_else(|| recv_supervisor.estop_epoch(), |s| s.obs.epoch_id);

    let immediate = |reply: Value| {
        Some(ExecutorActionDispatch::Reply {
            reply,
            gate: None,
            ack_guard: None,
        })
    };

    // Lease-free dry-run grasp stays on its planning-result contract.
    if path == "/api/v1/action/grasp" && body.get("dry_run").and_then(Value::as_bool) == Some(true) {
        let request_id = value.get("request_id").and_then(Value::as_u64);
        if request_id.is_none() || last_ordinary_request.is_some_and(|last| request_id.unwrap() <= last) {
            return immediate(json!({
                "type": "command_reject",
                "request_id": request_id_val,
                "status": 400,
                "code": "INVALID_COMMAND",
                "message": "request_id must be present and strictly increase",
                "error": "INVALID_COMMAND"
            }));
        }
        *last_ordinary_request = request_id;
        let dry = crate::grasp::dry_run_task(recv_supervisor, recv_deps.source.as_ref(), body).await;
        let mut ack = off_lock_action_ack(recv_supervisor, &request_id_val, None, epoch);
        for (k, v) in dry.fields() {
            ack[k.as_str()] = v;
        }
        return immediate(ack);
    }

    // 1. Lease authority check (off-lock).
    let local_generation = recv_generation.load(Ordering::SeqCst);
    let supplied_generation = value.get("generation").and_then(Value::as_u64).unwrap_or(0);
    let label_matches = {
        let lbl = recv_client_label.lock().unwrap_or_else(|e| e.into_inner());
        value.get("client_id").is_none_or(|id| id.as_str() == lbl.as_deref())
    };
    if local_generation == 0 || supplied_generation != local_generation || !label_matches {
        return immediate(json!({
            "type": "command_reject",
            "request_id": request_id_val,
            "status": 409,
            "code": "LEASE_LOST",
            "error": "LEASE_LOST",
            "message": "This connection does not hold the active lease",
            "epoch_id": epoch
        }));
    }

    // 2. Epoch check against live published state + atomic E-stop counter.
    if value.get("epoch_id").and_then(Value::as_u64) != Some(epoch) {
        return immediate(json!({
            "type": "command_reject",
            "request_id": request_id_val,
            "status": 409,
            "code": "EPOCH_MISMATCH",
            "message": "Command epoch is no longer current",
            "error": "EPOCH_MISMATCH",
            "epoch_id": epoch
        }));
    }

    // 3. Monotonic request_id check.
    let request_id = value.get("request_id").and_then(Value::as_u64);
    if request_id.is_none() || last_ordinary_request.is_some_and(|last| request_id.unwrap() <= last) {
        return immediate(json!({
            "type": "command_reject",
            "request_id": request_id_val,
            "status": 400,
            "code": "INVALID_COMMAND",
            "message": "request_id must be present and strictly increase",
            "error": "INVALID_COMMAND"
        }));
    }
    *last_ordinary_request = request_id;

    // 4. Operator auth check from dispatch table.
    match entry.auth {
        AuthRequirement::Operator(label) => {
            if !ws_auth.is_operator() {
                return immediate(off_lock_action_reject(
                    &request_id_val,
                    epoch,
                    ApiError::operator_required(label.unwrap_or(path)),
                ));
            }
        }
        AuthRequirement::Conditional(_)
            if path == "/api/v1/action/run_sequence"
                && !ws_auth.is_operator()
                && body.get("record").is_some_and(|r| !r.is_null()) =>
        {
            return immediate(off_lock_action_reject(
                &request_id_val,
                epoch,
                ApiError::operator_required("run_sequence (record)"),
            ));
        }
        AuthRequirement::Conditional(_)
            if path == "/api/v1/action/gripper_zero_check"
                && !ws_auth.is_operator()
                && body.get("store_offset").and_then(Value::as_bool) == Some(true) =>
        {
            return immediate(off_lock_action_reject(
                &request_id_val,
                epoch,
                ApiError::operator_required("gripper_zero_check (store_offset=true)"),
            ));
        }
        _ => {}
    }

    if !body.is_object() {
        return immediate(off_lock_action_reject(
            &request_id_val,
            epoch,
            invalid_command("Command body must be an object"),
        ));
    }

    // 5. Capture original connection admission stamp (preserving single-use A1 `stamp.request_id`).
    let mut ws_stamp = stamp;
    ws_stamp.connection_id = recv_conn_id;
    ws_stamp.lease_gen = local_generation;
    ws_stamp.estop_epoch = epoch;
    let deadline = Instant::now() + crate::rt_core::requests::actions::DEFAULT_ACTION_DEADLINE;

    if is_gripper_zero_check {
        let svc = match recv_supervisor
            .read_services(crate::rt_core::publish::ReadRequirements {
                require_within_valid_until: false,
                require_prep_env: true,
                ..crate::rt_core::publish::ReadRequirements::identity()
            })
            .await
        {
            Ok(s) => s,
            Err(e) => return immediate(off_lock_action_reject(&request_id_val, epoch, e.into())),
        };
        let pub_cfg = match recv_supervisor.published_config().await {
            Ok(c) => c,
            Err(e) => return immediate(off_lock_action_reject(&request_id_val, epoch, e.into())),
        };
        let operator_grant = ws_auth.operator_grant();
        let operator_auth = Some(ws_auth.auth_and_token());
        let (prepared, persist_ctx) = match crate::rt_core::requests::actions::prepare_gripper_zero_check(
            &svc,
            &pub_cfg,
            ws_stamp,
            deadline,
            body,
            operator_grant,
            operator_auth,
        ) {
            Ok(pair) => pair,
            Err(e) => return immediate(off_lock_action_reject(&request_id_val, epoch, e.into())),
        };
        let Some(handle) = recv_supervisor.executor_handle() else {
            return immediate(off_lock_action_reject(
                &request_id_val,
                epoch,
                ArmError::Rejected {
                    status: 503,
                    code: "EXECUTOR_UNAVAILABLE".into(),
                    message: "RT executor is not active".into(),
                }
                .into(),
            ));
        };
        let persist_ctx_id = if let Some(ctx) = persist_ctx {
            match handle.stage_zero_check_context(ctx) {
                Ok(id) => id,
                Err(e) => return immediate(off_lock_action_reject(&request_id_val, epoch, e.into())),
            }
        } else {
            0
        };
        let cur_epoch = recv_supervisor
            .hub()
            .latest()
            .map_or(epoch, |s| s.obs.epoch_id);
        let gate = CommandReplyGateGuard::enter_with_tracker(
            command_reply_pending,
            recv_supervisor.hub(),
            wire_ack_tracker,
        );
        match submit_prepared_executor_ws(
            recv_supervisor,
            recv_conn_id,
            PreparedWsAction::GripperZeroCheck {
                prepared,
                persist_ctx_id,
            },
            Arc::clone(wire_ack_tracker),
        )
        .await
        {
            Ok((started, ack_guard)) => {
                let reply = off_lock_action_ack(recv_supervisor, &request_id_val, Some(started), cur_epoch);
                return Some(ExecutorActionDispatch::Reply {
                    reply,
                    gate: Some(gate),
                    ack_guard: Some(ack_guard),
                });
            }
            Err(error) => {
                drop(gate);
                let post_ep = recv_supervisor
                    .hub()
                    .latest()
                    .map_or(cur_epoch, |s| s.obs.epoch_id);
                return immediate(off_lock_action_reject(&request_id_val, post_ep, error.into()));
            }
        }
    }

    let snap = match crate::rt_core::requests::actions::ActionSnapshot::capture(
        recv_supervisor,
        ws_stamp,
        deadline,
    )
    .await
    {
        Ok(s) => s,
        Err(e) => {
            return immediate(off_lock_action_reject(&request_id_val, epoch, e.into()));
        }
    };

    // 6. Dispatch prepared action vs macro action.
    if is_prepared {
        use crate::rt_core::requests::actions as req;
        let kind = match path {
            "/api/v1/action/move_to" => PendingPlanKind::MoveTo,
            "/api/v1/action/move_relative" => PendingPlanKind::MoveRelative,
            "/api/v1/action/move_to_pose" => PendingPlanKind::MoveToPose,
            "/api/v1/action/gripper" => PendingPlanKind::Gripper,
            "/api/v1/action/replay" => PendingPlanKind::Replay,
            _ => unreachable!(),
        };
        let is_replay = kind == PendingPlanKind::Replay;
        let body_owned = body.clone();
        let cancel = Arc::clone(worker_closing);
        let plan_conn = recv_connection.to_string();
        let prepared = tokio::task::spawn_blocking(move || {
            maybe_pause_blocking_plan(&plan_conn);
            if cancel.load(Ordering::Acquire) {
                return None;
            }
            let res: Result<PreparedWsAction, (ArmError, bool)> = match kind {
                PendingPlanKind::MoveTo => {
                    req::prepare_move_with_branch_hint(&snap, &body_owned, false).map(|m| match m {
                        req::PreparedMoveAction::Cartesian(c) => PreparedWsAction::Cartesian(c),
                        req::PreparedMoveAction::Joint(j) => PreparedWsAction::Joint(j),
                    })
                }
                PendingPlanKind::MoveRelative => {
                    req::prepare_move_with_branch_hint(&snap, &body_owned, true).map(|m| match m {
                        req::PreparedMoveAction::Cartesian(c) => PreparedWsAction::Cartesian(c),
                        req::PreparedMoveAction::Joint(j) => PreparedWsAction::Joint(j),
                    })
                }
                PendingPlanKind::MoveToPose => req::prepare_saved_pose(&snap, &body_owned)
                    .map(PreparedWsAction::Joint)
                    .map_err(|e| (e, false)),
                PendingPlanKind::Gripper => {
                    req::prepare_gripper(&snap, &body_owned, crate::supervisor::GripperBaselineCmd::None)
                        .map(PreparedWsAction::Gripper)
                        .map_err(|e| (e, false))
                }
                PendingPlanKind::Replay => req::prepare_replay(&snap, &body_owned)
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
            return Some(ExecutorActionDispatch::Closed);
        }
        let cur_epoch = recv_supervisor
            .hub()
            .latest()
            .map_or(epoch, |s| s.obs.epoch_id);
        if recv_supervisor.owner_mode() != crate::supervisor::OWNER_EXECUTOR {
            return immediate(off_lock_action_reject(
                &request_id_val,
                cur_epoch,
                ArmError::Rejected {
                    status: 503,
                    code: "EXECUTOR_UNAVAILABLE".into(),
                    message: "RT executor is no longer active".into(),
                }
                .into(),
            ));
        }
        let prepared = match prepared {
            Ok(None) => return Some(ExecutorActionDispatch::Closed),
            Err(e) => {
                let label = if is_replay {
                    "replay check task failed"
                } else {
                    "planner task failed"
                };
                return immediate(off_lock_action_reject(
                    &request_id_val,
                    cur_epoch,
                    ArmError::Internal(format!("{label}: {e}")).into(),
                ));
            }
            Ok(Some(Err((error, joint_hint)))) => {
                let mut api: ApiError = error.into();
                if joint_hint {
                    api.message.push_str(crate::motion::JOINT_PATH_HINT);
                }
                return immediate(off_lock_action_reject(&request_id_val, cur_epoch, api));
            }
            Ok(Some(Ok(p))) => p,
        };
        if recv_generation.load(Ordering::SeqCst) != local_generation {
            let msg = if is_replay {
                "This connection lost the lease while the episode was checked"
            } else {
                "This connection lost the lease while planning"
            };
            return immediate(json!({
                "type": "command_reject",
                "request_id": request_id_val,
                "status": 409,
                "code": "LEASE_LOST",
                "message": msg,
                "error": "LEASE_LOST",
                "epoch_id": cur_epoch
            }));
        }

        let gate = CommandReplyGateGuard::enter_with_tracker(
            command_reply_pending,
            recv_supervisor.hub(),
            wire_ack_tracker,
        );
        match submit_prepared_executor_ws(
            recv_supervisor,
            recv_conn_id,
            prepared,
            Arc::clone(wire_ack_tracker),
        )
        .await
        {
            Ok((started, ack_guard)) => {
                let reply = off_lock_action_ack(recv_supervisor, &request_id_val, Some(started), cur_epoch);
                return Some(ExecutorActionDispatch::Reply {
                    reply,
                    gate: Some(gate),
                    ack_guard: Some(ack_guard),
                });
            }
            Err(error) => {
                drop(gate);
                let post_ep = recv_supervisor
                    .hub()
                    .latest()
                    .map_or(cur_epoch, |s| s.obs.epoch_id);
                return immediate(off_lock_action_reject(&request_id_val, post_ep, error.into()));
            }
        }
    }

    match path {
        "/api/v1/action/grasp" => {
            let pending = match crate::grasp::prepare_from_snapshot(&snap, body, epoch) {
                Ok(p) => p,
                Err(e) => return immediate(off_lock_action_reject(&request_id_val, epoch, e.into())),
            };
            let mut check = spawn_macro_check(
                request_id_val,
                PendingMacro::Grasp(pending),
                local_generation,
                ws_stamp,
                recv_deps,
                worker_closing,
                recv_connection,
                worker_tasks,
            );
            check.executor = true;
            Some(ExecutorActionDispatch::Macro(check))
        }
        "/api/v1/action/place" => {
            let pending = match crate::place::prepare_from_snapshot(&snap, body, epoch) {
                Ok(p) => p,
                Err(e) => return immediate(off_lock_action_reject(&request_id_val, epoch, e.into())),
            };
            let mut check = spawn_macro_check(
                request_id_val,
                PendingMacro::Place(pending),
                local_generation,
                ws_stamp,
                recv_deps,
                worker_closing,
                recv_connection,
                worker_tasks,
            );
            check.executor = true;
            Some(ExecutorActionDispatch::Macro(check))
        }
        "/api/v1/action/run_sequence" => {
            let plan = match crate::sequences::prepare_run_from_snapshot_with_recorder(
                recv_supervisor,
                &snap,
                body,
                epoch,
            )
            .await
            {
                Ok(p) => p,
                Err(e) => return immediate(off_lock_action_reject(&request_id_val, epoch, e.into())),
            };
            let gate = CommandReplyGateGuard::enter_with_tracker(
                command_reply_pending,
                recv_supervisor.hub(),
                wire_ack_tracker,
            );
            match crate::sequences::begin_executor_ws(
                recv_supervisor,
                ws_stamp,
                plan,
                Arc::clone(wire_ack_tracker),
            )
            .await
            {
                Ok((started, ack_guard)) => {
                    let reply = off_lock_action_ack(recv_supervisor, &request_id_val, Some(started), epoch);
                    Some(ExecutorActionDispatch::Reply {
                        reply,
                        gate: Some(gate),
                        ack_guard: Some(ack_guard),
                    })
                }
                Err(error) => {
                    drop(gate);
                    let cur_epoch = recv_supervisor.hub().latest().map_or(epoch, |s| s.obs.epoch_id);
                    immediate(off_lock_action_reject(&request_id_val, cur_epoch, error.into()))
                }
            }
        }
        p if p == crate::calib_hover::PATH => {
            let existing_jobs = recv_supervisor.hover_jobs_snapshot().await;
            let plan = match crate::calib_hover::prepare_from_snapshot(&snap, &existing_jobs, body, epoch) {
                Ok(p) => p,
                Err(e) => return immediate(off_lock_action_reject(&request_id_val, epoch, e.into())),
            };
            let gate = CommandReplyGateGuard::enter_with_tracker(
                command_reply_pending,
                recv_supervisor.hub(),
                wire_ack_tracker,
            );
            match crate::calib_hover::begin_executor_ws(
                recv_supervisor,
                ws_stamp,
                plan,
                recv_calib.upstream.clone(),
                crate::calib_hover::jobs_dir(&recv_calib.path),
                Arc::clone(wire_ack_tracker),
            )
            .await
            {
                Ok(((id, duration, job_id), ack_guard)) => {
                    let mut reply = off_lock_action_ack(
                        recv_supervisor,
                        &request_id_val,
                        Some((id, duration)),
                        epoch,
                    );
                    reply["job_id"] = json!(job_id);
                    Some(ExecutorActionDispatch::Reply {
                        reply,
                        gate: Some(gate),
                        ack_guard: Some(ack_guard),
                    })
                }
                Err(error) => {
                    drop(gate);
                    let cur_epoch = recv_supervisor.hub().latest().map_or(epoch, |s| s.obs.epoch_id);
                    immediate(off_lock_action_reject(&request_id_val, cur_epoch, error.into()))
                }
            }
        }
        p if p == crate::measure_multiview::PATH => {
            let plan = match crate::measure_multiview::prepare_from_snapshot(&snap, body, epoch) {
                Ok(p) => p,
                Err(e) => return immediate(off_lock_action_reject(&request_id_val, epoch, e.into())),
            };
            let gate = CommandReplyGateGuard::enter_with_tracker(
                command_reply_pending,
                recv_supervisor.hub(),
                wire_ack_tracker,
            );
            match crate::measure_multiview::begin_executor_ws(
                recv_supervisor,
                ws_stamp,
                plan,
                recv_deps.source.clone(),
                Arc::clone(wire_ack_tracker),
            )
            .await
            {
                Ok((started, ack_guard)) => {
                    let reply = off_lock_action_ack(recv_supervisor, &request_id_val, Some(started), epoch);
                    Some(ExecutorActionDispatch::Reply {
                        reply,
                        gate: Some(gate),
                        ack_guard: Some(ack_guard),
                    })
                }
                Err(error) => {
                    drop(gate);
                    let cur_epoch = recv_supervisor.hub().latest().map_or(epoch, |s| s.obs.epoch_id);
                    immediate(off_lock_action_reject(&request_id_val, cur_epoch, error.into()))
                }
            }
        }
        _ => None,
    }
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
            Ok(ActionStart::Plan(Box::new(PendingPlan {
                request_id: request_id.clone(),
                kind,
                snap,
                body: body.clone(),
            })))
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
            Ok(ActionStart::Macro(Box::new(PendingMacro::Grasp(
                crate::grasp::prepare(inner, body, epoch)?,
            ))))
        }
        "/api/v1/action/place" => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            Ok(ActionStart::Macro(Box::new(PendingMacro::Place(
                crate::place::prepare(inner, body, epoch)?,
            ))))
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
) -> Option<OwnerReply> {
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
        Ok(ActionStart::Started(id, duration)) => Some(OwnerReply::CommandAck {
            snap: AckSnapshot::from_inner(inner),
            started: Some((id, duration)),
        }),
        Ok(ActionStart::Plan(plan)) => {
            pending.plan = Some((*plan, local_generation));
            None
        }
        Ok(ActionStart::Macro(m)) => {
            pending.macro_action = Some((value["request_id"].clone(), *m, local_generation));
            None
        }
        Ok(ActionStart::DryRunGrasp(body)) => {
            pending.dry_run_grasp = Some((value["request_id"].clone(), body));
            None
        }
        Ok(ActionStart::Sequence(plan)) => Some(
            match crate::sequences::begin_with_stamp(inner, stamp, recv_supervisor, *plan) {
                Ok(started) => OwnerReply::CommandAck {
                    snap: AckSnapshot::from_inner(inner),
                    started: Some(started),
                },
                Err(error) => OwnerReply::CommandReject {
                    epoch: inner.safety.epoch_id(),
                    error: error.into(),
                },
            },
        ),
        Ok(ActionStart::CalibHover(plan)) => Some(
            match crate::calib_hover::begin_with_stamp(
                inner,
                stamp,
                recv_supervisor,
                *plan,
                recv_calib.upstream.clone(),
                crate::calib_hover::jobs_dir(&recv_calib.path),
            ) {
                Ok((id, duration, job_id)) => OwnerReply::CommandAckJob {
                    snap: AckSnapshot::from_inner(inner),
                    started: Some((id, duration)),
                    job_id,
                },
                Err(error) => OwnerReply::CommandReject {
                    epoch: inner.safety.epoch_id(),
                    error: error.into(),
                },
            },
        ),
        Ok(ActionStart::GripperCalibrate(epoch, tag)) => Some(
            match inner
                .check_admission(stamp)
                .map_err(ArmError::from)
                .and_then(|admitted| {
                    crate::gripper_calib::begin_calibration(admitted, recv_supervisor, epoch, tag)
                }) {
                Ok(started) => OwnerReply::CommandAck {
                    snap: AckSnapshot::from_inner(inner),
                    started: Some(started),
                },
                Err(error) => OwnerReply::CommandReject {
                    epoch: inner.safety.epoch_id(),
                    error: error.into(),
                },
            },
        ),
        Ok(ActionStart::MeasureMultiview(plan)) => Some(
            match crate::measure_multiview::begin_with_stamp(
                inner,
                stamp,
                recv_supervisor,
                *plan,
                recv_deps.source.clone(),
            ) {
                Ok(started) => OwnerReply::CommandAck {
                    snap: AckSnapshot::from_inner(inner),
                    started: Some(started),
                },
                Err(error) => OwnerReply::CommandReject {
                    epoch: inner.safety.epoch_id(),
                    error: error.into(),
                },
            },
        ),
        Err(error) => Some(OwnerReply::CommandReject {
            epoch: inner.safety.epoch_id(),
            error,
        }),
    }
}

/// Complete a joined `MacroCheck` (returns `None` when connection is closing).
#[allow(clippy::too_many_arguments)]
pub(super) async fn complete_macro_check(
    request_id: Value,
    command_path: &'static str,
    generation: u64,
    stamp: crate::supervisor::AdmissionStamp,
    executor: bool,
    joined: Result<Result<PlannedMacro, ArmError>, tokio::sync::oneshot::error::RecvError>,
    recv_supervisor: &Arc<ArmSupervisor>,
    recv_connection: &str,
    recv_generation: &AtomicU64,
    recv_deps: &crate::grasp::GraspDeps,
    worker_closing: &AtomicBool,
    command_reply_pending: &Arc<AtomicBool>,
    wire_ack_tracker: &Arc<crate::rt_core::reply_router::WsWireAckTracker>,
) -> Option<(
    Value,
    Option<CommandReplyGateGuard>,
    Option<crate::rt_core::reply_router::AckCommitGuard>,
)> {
    if worker_closing.load(Ordering::SeqCst) {
        return None;
    }
    if executor {
        let epoch = recv_supervisor
            .hub()
            .latest()
            .map_or_else(|| recv_supervisor.estop_epoch(), |s| s.obs.epoch_id);
        if recv_supervisor.owner_mode() != crate::supervisor::OWNER_EXECUTOR {
            return Some((
                off_lock_action_reject(
                    &request_id,
                    epoch,
                    ArmError::Rejected {
                        status: 503,
                        code: "EXECUTOR_UNAVAILABLE".into(),
                        message: "RT executor is no longer active".into(),
                    }
                    .into(),
                ),
                None,
                None,
            ));
        }
        if recv_generation.load(Ordering::SeqCst) != generation {
            return Some((
                json!({
                    "type": "command_reject",
                    "request_id": request_id,
                    "status": 409,
                    "code": "LEASE_LOST",
                    "message": "This connection lost the lease while planning",
                    "error": "LEASE_LOST",
                    "epoch_id": epoch
                }),
                None,
                None,
            ));
        }
        let planned = match joined {
            Err(e) => {
                return Some((
                    off_lock_action_reject(
                        &request_id,
                        epoch,
                        ArmError::Internal(format!("planner task failed: {e}")).into(),
                    ),
                    None,
                    None,
                ));
            }
            Ok(Err(error)) => {
                return Some((off_lock_action_reject(&request_id, epoch, error.into()), None, None));
            }
            Ok(Ok(p)) => p,
        };
        let gate = CommandReplyGateGuard::enter_with_tracker(
            command_reply_pending,
            recv_supervisor.hub(),
            wire_ack_tracker,
        );
        let begin_res = match planned {
            PlannedMacro::Grasp(g) => {
                crate::grasp::begin_executor_ws(
                    recv_supervisor,
                    stamp,
                    recv_deps.source.clone(),
                    g,
                    Arc::clone(wire_ack_tracker),
                )
                .await
            }
            PlannedMacro::Place(p) => {
                crate::place::begin_executor_ws(
                    recv_supervisor,
                    stamp,
                    p,
                    Arc::clone(wire_ack_tracker),
                )
                .await
            }
        };
        match begin_res {
            Ok((started, ack_guard)) => {
                let reply = off_lock_action_ack(recv_supervisor, &request_id, Some(started), epoch);
                return Some((reply, Some(gate), Some(ack_guard)));
            }
            Err(error) => {
                drop(gate);
                let cur_epoch = recv_supervisor
                    .hub()
                    .latest()
                    .map_or(epoch, |s| s.obs.epoch_id);
                return Some((
                    off_lock_action_reject(&request_id, cur_epoch, error.into()),
                    None,
                    None,
                ));
            }
        }
    }
    let owner_transferred = || {
        let mut r = json!({"type":"command_reject", "request_id":request_id, "status":503, "code":"OWNER_TRANSFERRED", "error":"OWNER_TRANSFERRED", "message":"Ownership transferred to RT executor while planning macro"});
        if let Some(s) = recv_supervisor.hub().latest() {
            r["epoch_id"] = json!(s.obs.epoch_id);
        }
        r
    };
    if recv_supervisor.owner_mode() != crate::supervisor::OWNER_LEGACY {
        return Some((owner_transferred(), None, None));
    }
    enum MacroFinishOutcome {
        Closed,
        Detached,
        LeaseLost { epoch: u64 },
        Ack { snap: AckSnapshot, started: Started },
        Reject { epoch: u64, error: ApiError },
    }
    let gate = CommandReplyGateGuard::enter(command_reply_pending, recv_supervisor.hub());
    let outcome = {
        let mut inner = recv_supervisor.inner().write().await;
        inner.set_command(command_path);
        if worker_closing.load(Ordering::SeqCst) {
            MacroFinishOutcome::Closed
        } else if inner.detached {
            MacroFinishOutcome::Detached
        } else if let Err(err) = inner.preflight_admission(stamp) {
            MacroFinishOutcome::Reject {
                epoch: inner.safety.epoch_id(),
                error: ArmError::from(err).into(),
            }
        } else {
            match joined {
                Err(e) => MacroFinishOutcome::Reject {
                    epoch: inner.safety.epoch_id(),
                    error: ArmError::Internal(format!("planner task failed: {e}")).into(),
                },
                Ok(Err(error)) => MacroFinishOutcome::Reject {
                    epoch: inner.safety.epoch_id(),
                    error: error.into(),
                },
                Ok(Ok(planned)) => {
                    if recv_generation.load(Ordering::SeqCst) != generation
                        || !inner.validate_servo_lease(recv_connection, generation)
                    {
                        MacroFinishOutcome::LeaseLost {
                            epoch: inner.safety.epoch_id(),
                        }
                    } else {
                        match planned.start(&mut inner, stamp, recv_supervisor, recv_deps) {
                            Ok(started) => {
                                inner.set_action_id(started.0);
                                MacroFinishOutcome::Ack {
                                    snap: AckSnapshot::from_inner(&inner),
                                    started: Some(started),
                                }
                            }
                            Err(error) => MacroFinishOutcome::Reject {
                                epoch: inner.safety.epoch_id(),
                                error: error.into(),
                            },
                        }
                    }
                }
            }
        }
    };
    // Write guard is now released; build every JSON reply strictly off-lock while keeping `gate`
    // active for ACKs so action terminals cannot overtake the ACK on the wire.
    match outcome {
        MacroFinishOutcome::Closed => None,
        MacroFinishOutcome::Detached => {
            drop(gate);
            Some((owner_transferred(), None, None))
        }
        MacroFinishOutcome::LeaseLost { epoch } => {
            drop(gate);
            let reply = json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"LEASE_LOST", "message":"This connection lost the lease while planning", "error":"LEASE_LOST", "epoch_id":epoch});
            Some((reply, None, None))
        }
        MacroFinishOutcome::Reject { epoch, error } => {
            drop(gate);
            Some((command_reject_at(epoch, &request_id, error), None, None))
        }
        MacroFinishOutcome::Ack { snap, started } => {
            Some((command_ack_from_snap(&snap, &request_id, started), Some(gate), None))
        }
    }
}

#[allow(clippy::large_enum_variant)]
enum PreparedWsAction {
    Cartesian(crate::rt_core::requests::actions::PreparedCartesianAction),
    Joint(crate::rt_core::requests::actions::PreparedJointAction),
    Gripper(crate::rt_core::requests::actions::PreparedGripperAction),
    GripperZeroCheck {
        prepared: crate::rt_core::requests::actions::PreparedGripperZeroCheck,
        persist_ctx_id: u64,
    },
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
    command_reply_pending: &Arc<AtomicBool>,
) -> Option<(Option<Value>, Option<CommandReplyGateGuard>)> {
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
    enum PlanInstallOutcome {
        Closed,
        Detached,
        NoReply,
        LeaseLost { epoch: u64 },
        Ack { snap: AckSnapshot, started: Started },
        Reject { epoch: u64, error: ApiError },
    }
    // M2: Keep the per-connection reply gate closed from BEFORE acquiring the install lock until
    // the caller enqueues `command_ack` onto `ordinary_tx` so terminals (e.g. from an immediate
    // Stop) can never overtake the ACK for gripper / move_to_pose / move_to / replay.
    let gate = CommandReplyGateGuard::enter(command_reply_pending, recv_supervisor.hub());
    let outcome = {
        let mut inner = recv_supervisor.inner().write().await;
        inner.set_command(kind.command_path());
        if worker_closing.load(Ordering::SeqCst) {
            PlanInstallOutcome::Closed
        } else if inner.detached {
            PlanInstallOutcome::Detached
        } else {
            if let Some(hook) = inner.before_install_hook.take() {
                hook(&mut inner);
            }
            if let Err(err) = inner.preflight_admission(stamp) {
                PlanInstallOutcome::Reject {
                    epoch: inner.safety.epoch_id(),
                    error: ArmError::from(err).into(),
                }
            } else {
                match prepared {
                    Ok(None) => PlanInstallOutcome::NoReply,
                    Err(e) => {
                        let label = if is_replay {
                            "replay check task failed"
                        } else {
                            "planner task failed"
                        };
                        PlanInstallOutcome::Reject {
                            epoch: inner.safety.epoch_id(),
                            error: ArmError::Internal(format!("{label}: {e}")).into(),
                        }
                    }
                    Ok(Some(Err((error, joint_hint)))) => {
                        let mut api: ApiError = error.into();
                        if joint_hint {
                            api.message.push_str(crate::motion::JOINT_PATH_HINT);
                        }
                        PlanInstallOutcome::Reject {
                            epoch: inner.safety.epoch_id(),
                            error: api,
                        }
                    }
                    Ok(Some(Ok(prepared))) => {
                        if recv_generation.load(Ordering::SeqCst) != generation
                            || !inner.validate_servo_lease(recv_connection, generation)
                        {
                            PlanInstallOutcome::LeaseLost {
                                epoch: inner.safety.epoch_id(),
                            }
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
                                PreparedWsAction::GripperZeroCheck { .. } => {
                                    unreachable!("legacy gripper_zero_check does not use PendingPlan")
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
                                Ok(started) => {
                                    inner.set_action_id(started.0);
                                    PlanInstallOutcome::Ack {
                                        snap: AckSnapshot::from_inner(&inner),
                                        started: Some(started),
                                    }
                                }
                                Err(error) => PlanInstallOutcome::Reject {
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
    // Build every WS reply strictly AFTER the write guard is released.
    match outcome {
        PlanInstallOutcome::Closed => None,
        PlanInstallOutcome::NoReply => Some((None, None)),
        PlanInstallOutcome::Detached => {
            drop(gate);
            let msg = if is_replay {
                "Ownership transferred to RT executor while checking replay"
            } else {
                "Ownership transferred to RT executor while planning"
            };
            let mut r = json!({"type":"command_reject", "request_id":request_id, "status":503, "code":"OWNER_TRANSFERRED", "error":"OWNER_TRANSFERRED", "message":msg});
            if let Some(s) = recv_supervisor.hub().latest() {
                r["epoch_id"] = json!(s.obs.epoch_id);
            }
            Some((Some(r), None))
        }
        PlanInstallOutcome::LeaseLost { epoch } => {
            drop(gate);
            let msg = if is_replay {
                "This connection lost the lease while the episode was checked"
            } else {
                "This connection lost the lease while planning"
            };
            let r = json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"LEASE_LOST", "message":msg, "error":"LEASE_LOST", "epoch_id":epoch});
            Some((Some(r), None))
        }
        PlanInstallOutcome::Reject { epoch, error } => {
            drop(gate);
            Some((Some(command_reject_at(epoch, &request_id, error)), None))
        }
        PlanInstallOutcome::Ack { snap, started } => {
            let reply = command_ack_from_snap(&snap, &request_id, started);
            Some((Some(reply), Some(gate)))
        }
    }
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
    let command_path = pending.command_path();
    let source = recv_deps.source.clone();
    let cancel = worker_closing.clone();
    let plan_conn = recv_connection.to_string();
    let (res_tx, res_rx) = tokio::sync::oneshot::channel();
    let abort = worker_tasks.spawn_abortable(async move {
        let planned_res = async {
            let cancel_blk = cancel.clone();
            let mut pending = pending;
            if let PendingMacro::Grasp(g) = &mut pending {
                g.set_cancel(cancel_blk.clone());
            }
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
        command_path,
        generation,
        stamp,
        executor: false,
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
    command_reply_pending: &Arc<AtomicBool>,
) -> Option<(Option<Value>, Option<CommandReplyGateGuard>)> {
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
    enum ReplayInstallOutcome {
        Closed,
        Detached,
        NoReply,
        LeaseLost { epoch: u64 },
        Ack { snap: AckSnapshot, started: Started },
        Reject { epoch: u64, error: ApiError },
    }
    let gate = CommandReplyGateGuard::enter(command_reply_pending, recv_supervisor.hub());
    let outcome = {
        let mut inner = recv_supervisor.inner().write().await;
        inner.set_command("/api/v1/action/replay");
        if worker_closing.load(Ordering::SeqCst) {
            ReplayInstallOutcome::Closed
        } else if inner.detached {
            ReplayInstallOutcome::Detached
        } else if let Err(err) = inner.preflight_admission(stamp) {
            ReplayInstallOutcome::Reject {
                epoch: inner.safety.epoch_id(),
                error: ArmError::from(err).into(),
            }
        } else {
            match planned {
                Ok(None) => ReplayInstallOutcome::NoReply,
                Err(e) => ReplayInstallOutcome::Reject {
                    epoch: inner.safety.epoch_id(),
                    error: ArmError::Internal(format!("replay check task failed: {e}")).into(),
                },
                Ok(Some(Err(error))) => ReplayInstallOutcome::Reject {
                    epoch: inner.safety.epoch_id(),
                    error: error.into(),
                },
                Ok(Some(Ok(plan))) => {
                    if recv_generation.load(Ordering::SeqCst) != generation
                        || !inner.validate_servo_lease(recv_connection, generation)
                    {
                        ReplayInstallOutcome::LeaseLost {
                            epoch: inner.safety.epoch_id(),
                        }
                    } else {
                        match inner
                            .check_admission(stamp)
                            .map_err(ArmError::from)
                            .and_then(|admitted| SupervisorInner::install_replay(admitted, plan))
                        {
                            Ok(started) => {
                                inner.set_action_id(started.0);
                                ReplayInstallOutcome::Ack {
                                    snap: AckSnapshot::from_inner(&inner),
                                    started: Some(started),
                                }
                            }
                            Err(error) => ReplayInstallOutcome::Reject {
                                epoch: inner.safety.epoch_id(),
                                error: error.into(),
                            },
                        }
                    }
                }
            }
        }
    };
    match outcome {
        ReplayInstallOutcome::Closed => None,
        ReplayInstallOutcome::NoReply => Some((None, None)),
        ReplayInstallOutcome::Detached => {
            drop(gate);
            let mut r = json!({"type":"command_reject", "request_id":request_id, "status":503, "code":"OWNER_TRANSFERRED", "error":"OWNER_TRANSFERRED", "message":"Ownership transferred to RT executor while checking replay"});
            if let Some(s) = recv_supervisor.hub().latest() {
                r["epoch_id"] = json!(s.obs.epoch_id);
            }
            Some((Some(r), None))
        }
        ReplayInstallOutcome::LeaseLost { epoch } => {
            drop(gate);
            let r = json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"LEASE_LOST", "message":"This connection lost the lease while the episode was checked", "error":"LEASE_LOST", "epoch_id":epoch});
            Some((Some(r), None))
        }
        ReplayInstallOutcome::Reject { epoch, error } => {
            drop(gate);
            Some((Some(command_reject_at(epoch, &request_id, error)), None))
        }
        ReplayInstallOutcome::Ack { snap, started } => {
            let reply = command_ack_from_snap(&snap, &request_id, started);
            Some((Some(reply), Some(gate)))
        }
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
    let snap = {
        let inner = recv_supervisor.inner().read().await;
        if !inner.detached {
            AckSnapshot::from_inner(&inner)
        } else if let Some(s) = recv_supervisor.hub().latest() {
            AckSnapshot {
                epoch_id: s.obs.epoch_id,
                safety_state: s.obs.safety_state.clone(),
                is_torqued: s.obs.is_torqued,
                control_mode: if s.obs.control_mode == crate::model::ControlMode::GravityComp {
                    "gravity_comp"
                } else {
                    "position"
                },
            }
        } else {
            AckSnapshot::from_inner(&inner)
        }
    };
    let mut ack = command_ack_from_snap(&snap, &request_id, None);
    for (k, v) in dry.fields() {
        ack[k.as_str()] = v;
    }
    ack
}
