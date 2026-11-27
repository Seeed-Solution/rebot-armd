use std::sync::Arc;
use std::time::{Duration, Instant};

use serde_json::Value;

use crate::api::rest::ApiError;
use crate::supervisor::{ArmError, ArmSupervisor, SupervisorInner};

use super::{expected_duration, plan, preflight, prepare, run, DetectionSource, PlannedGrasp};

/// Build the A7 [`BeginMacroSpec`] for a planned `grasp` macro (shared by Legacy and RT Executor).
/// For `strategy: "adaptive"`, builds P's [`crate::grasp_select::SelectedGraspMacro`] directly
/// from the authoritative [`crate::grasp_select::AdaptiveRoute`].
pub fn begin_macro_spec(
    planned: &PlannedGrasp,
    prepared_boot: crate::rt_core::broker::BootId,
    dof: usize,
    gripper_idx: Option<usize>,
    deadline: Instant,
) -> Result<crate::rt_core::requests::actions::BeginMacroSpec, ArmError> {
    if let Some(pa) = &planned.adaptive {
        let m = pa
            .route
            .into_selected_macro(
                prepared_boot,
                &pa.start_positions,
                deadline,
                planned.request.tag.as_deref(),
                planned.request.task.as_deref(),
                pa.revisions,
                0,
            )
            .map_err(|code| ArmError::Detailed {
                status: 400,
                code: code.into(),
                message: format!("[begin] failed to construct adaptive BeginMacroSpec: {code}"),
                details: Box::new(serde_json::json!({
                    "failing_phase": "begin",
                    "outcome": "infeasible",
                })),
            })?;
        return Ok(m.begin);
    }
    use crate::rt_core::requests::actions::{BeginMacroSpec, MacroKind};
    let mut start = vec![0.0f32; dof];
    let arm_dof = planned.input.arm.arm_dof().min(dof).min(6);
    for (i, dst) in start.iter_mut().take(arm_dof).enumerate() {
        *dst = planned.input.q0[i] as f32;
    }
    if let Some(g) = gripper_idx.filter(|&g| g < dof) {
        start[g] = planned.input.j7;
    }
    let mut spec = BeginMacroSpec::new(MacroKind::Grasp, prepared_boot);
    if let Ok(s) = spec.with_start(&start) {
        spec = s;
    }
    spec.check_workspace_revision = true;
    spec.macro_deadline = Some(deadline);
    spec.timeout_code = crate::safety::BoundedText::new("GRASP_TIMEOUT");
    if let Some(tag) = planned.request.tag.as_deref() {
        spec.tag = crate::model::HolderLabel::new(tag);
    }
    if let Some(task) = planned.request.task.as_deref() {
        spec.task = crate::model::HolderLabel::new(task);
    }
    Ok(spec)
}

/// Begin a planned grasp under a short write lock through the single owner-side
/// [`crate::rt_core::requests::actions::install_begin_macro`] function and spawn its orchestrator.
/// Returns (action id, expected duration).
pub fn begin_with_stamp(
    inner: &mut SupervisorInner,
    stamp: crate::supervisor::AdmissionStamp,
    supervisor: &Arc<ArmSupervisor>,
    source: Arc<dyn DetectionSource>,
    planned: PlannedGrasp,
) -> Result<(u64, f64), ArmError> {
    use crate::rt_core::requests::actions::{
        install_begin_macro, ActionStamp, OwnerCheckCtx, DEFAULT_ACTION_DEADLINE,
    };
    let deadline = Instant::now() + Duration::from_secs_f64(planned.request.setup.timeout_s);
    let spec = begin_macro_spec(
        &planned,
        inner.boot_id,
        inner.config.dof,
        inner.config.gripper_index(),
        deadline,
    )?;
    let adaptive_rev = planned.adaptive.as_ref().map(|pa| pa.revisions);
    let action_stamp = ActionStamp {
        arm: inner.core.id(),
        boot: inner.boot_id,
        admission: stamp,
        config_revision: adaptive_rev.map_or(planned.input.workspace_revision, |r| r.workspace_revision),
        calibration_revision: adaptive_rev.map(|r| r.calibration_revision),
        poses_revision: None,
        sequences_revision: None,
        grasp_setup_revision: adaptive_rev.map(|r| r.grasp_setup_revision),
        gripper_calib_revision: adaptive_rev.map(|r| r.gripper_calib_revision),
        power_budget_revision: adaptive_rev.map(|r| r.power_budget_revision),
        parent_generation: 0,
        deadline: Instant::now() + DEFAULT_ACTION_DEADLINE,
    };
    let vision_guard = if planned.mode.as_ref().is_some_and(Result::is_err) {
        crate::vision_detector::vision_action_in_flight()
    } else {
        crate::vision_detector::try_begin_vision_action()?
    };
    let owner_ctx = OwnerCheckCtx::for_legacy(inner);
    let id = install_begin_macro(
        inner,
        &action_stamp,
        &owner_ctx,
        &spec,
        planned.request.tag.clone(),
        planned.request.task.clone(),
    )
    .map_err(|r| r.into_arm_error(inner))?;
    let duration = if planned.adaptive.is_some() {
        expected_duration(&planned)
    } else {
        expected_duration(&planned)
            + inner.approach_estimate_s("grasp_final", planned.request.approach)
    };
    spawn_with_deadline_guarded(supervisor.clone(), source, id, planned, stamp, deadline, vision_guard);
    Ok((id, duration))
}

/// Begin a planned grasp under the write lock: the action record (with its `task`) and the
/// orchestrator. Returns (action id, expected duration).
pub fn begin(admitted: crate::supervisor::Admitted<'_>, supervisor: &Arc<ArmSupervisor>, source: Arc<dyn DetectionSource>, planned: PlannedGrasp) -> Result<(u64, f64), ArmError> {
    let deadline = Instant::now() + Duration::from_secs_f64(planned.request.setup.timeout_s);
    let (inner, proof) = admitted.consume()?;
    let vision_guard = if planned.mode.as_ref().is_some_and(Result::is_err) {
        crate::vision_detector::vision_action_in_flight()
    } else {
        crate::vision_detector::try_begin_vision_action()?
    };
    if planned.adaptive.is_some() {
        inner.j7_admitted_stamp = None;
    }
    let stamp = proof.stamp();
    let spec = begin_macro_spec(
        &planned,
        inner.boot_id,
        inner.config.dof,
        inner.config.gripper_index(),
        deadline,
    )?;
    let duration = if planned.adaptive.is_some() {
        expected_duration(&planned)
    } else {
        expected_duration(&planned) + inner.approach_estimate_s("grasp_final", planned.request.approach)
    };
    let ws_rev = planned
        .adaptive
        .as_ref()
        .map_or(planned.input.workspace_revision, |pa| pa.revisions.workspace_revision);
    let id = crate::rt_core::requests::actions::install_begin_macro_with_proof(
        inner,
        &proof,
        &spec,
        Some(ws_rev),
        planned.request.tag.clone(),
        planned.request.task.clone(),
    )?;
    spawn_with_deadline_guarded(supervisor.clone(), source, id, planned, stamp, deadline, vision_guard);
    Ok((id, duration))
}

/// Start a grasp (tests, in-process agents): caller passes the `AdmissionStamp` captured at
/// admission (never re-sampled internally). Prepares, plans, runs task preflight, validates
/// `stamp` under the owner lock through [`begin_with_stamp`], and spawns the orchestrator.
/// Returns (action id, expected duration).
pub async fn start(supervisor: &Arc<ArmSupervisor>, stamp: crate::supervisor::AdmissionStamp, source: Arc<dyn DetectionSource>, body: &Value) -> Result<(u64, f64), ApiError> {
    let pending = {
        let inner = supervisor.inner().read().await;
        prepare(&inner, body, stamp.estop_epoch)?
    };
    let mut planned = tokio::task::spawn_blocking(move || plan(pending)).await
        .map_err(|e| ArmError::Internal(format!("planner task failed: {e}")))??;
    preflight(source.as_ref(), &mut planned).await?;
    let mut inner = supervisor.inner().write().await;
    Ok(begin_with_stamp(&mut inner, stamp, supervisor, source, planned)?)
}

/// Production broker-to-worker handoff for `grasp` on `OWNER_EXECUTOR` (A7 / P-F1):
/// submits `BeginMacro` carrying the caller's original `AdmissionStamp` and exact absolute
/// `deadline`, then spawns the orchestrator with the accepted parent `ActionId`, original
/// `stamp`, and exact `deadline` (0 `SupervisorLock`).
pub async fn begin_executor(
    supervisor: &Arc<ArmSupervisor>,
    stamp: crate::supervisor::AdmissionStamp,
    source: Arc<dyn DetectionSource>,
    planned: PlannedGrasp,
) -> Result<(u64, f64), ArmError> {
    let ((id, duration), guard) = begin_executor_inner(
        supervisor,
        stamp,
        source,
        planned,
        crate::rt_core::reply_router::ExternalDestinationPolicy::ConnectionOrGlobal,
        false,
        None,
    )
    .await?;
    let _ = guard.confirm_enqueued();
    Ok((id, duration))
}

/// WebSocket-bound `grasp` executor handoff returning the RAII [`AckCommitGuard`] so the WS
/// worker confirms wire-ACK enqueue strictly after `ordinary_tx` accepts `command_ack`.
pub(crate) async fn begin_executor_ws(
    supervisor: &Arc<ArmSupervisor>,
    stamp: crate::supervisor::AdmissionStamp,
    source: Arc<dyn DetectionSource>,
    planned: PlannedGrasp,
    wire_tracker: Arc<crate::rt_core::reply_router::WsWireAckTracker>,
) -> Result<((u64, f64), crate::rt_core::reply_router::AckCommitGuard), ArmError> {
    begin_executor_inner(
        supervisor,
        stamp,
        source,
        planned,
        crate::rt_core::reply_router::ExternalDestinationPolicy::ConnectionOnly,
        true,
        Some(wire_tracker),
    )
    .await
}

async fn begin_executor_inner(
    supervisor: &Arc<ArmSupervisor>,
    stamp: crate::supervisor::AdmissionStamp,
    source: Arc<dyn DetectionSource>,
    planned: PlannedGrasp,
    policy: crate::rt_core::reply_router::ExternalDestinationPolicy,
    require_ack_commit: bool,
    wire_tracker: Option<Arc<crate::rt_core::reply_router::WsWireAckTracker>>,
) -> Result<((u64, f64), crate::rt_core::reply_router::AckCommitGuard), ArmError> {
    use crate::rt_core::requests::actions::{ActionStamp, DEFAULT_ACTION_DEADLINE};
    let handle = supervisor.executor_handle().ok_or_else(|| ArmError::Rejected {
        status: 503,
        code: "EXECUTOR_UNAVAILABLE".into(),
        message: "RT executor is not active".into(),
    })?;
    let boot = handle.boot();
    let arm = supervisor
        .hub()
        .latest()
        .map_or(crate::rt_core::ids::ArmId::LOCAL, |s| s.rt.owner.arm);
    let now = Instant::now();
    let deadline = now + Duration::from_secs_f64(planned.request.setup.timeout_s);
    let (dof, gripper_idx) = supervisor
        .hub()
        .prep_env()
        .map_or((planned.input.limits.len(), planned.input.limits.len().checked_sub(1)), |e| {
            (e.config.dof, e.config.gripper_index())
        });
    let mut spec = begin_macro_spec(&planned, boot, dof, gripper_idx, deadline)?;
    spec.admission_request_id = stamp.request_id;
    let adaptive_rev = planned.adaptive.as_ref().map(|pa| pa.revisions);
    let action_stamp = ActionStamp {
        arm,
        boot,
        admission: stamp,
        config_revision: adaptive_rev.map_or(planned.input.workspace_revision, |r| r.workspace_revision),
        calibration_revision: adaptive_rev.map(|r| r.calibration_revision),
        poses_revision: None,
        sequences_revision: None,
        grasp_setup_revision: adaptive_rev.map(|r| r.grasp_setup_revision),
        gripper_calib_revision: adaptive_rev.map(|r| r.gripper_calib_revision),
        power_budget_revision: adaptive_rev.map(|r| r.power_budget_revision),
        parent_generation: 0,
        deadline: now + DEFAULT_ACTION_DEADLINE,
    };
    let vision_guard = if planned.mode.as_ref().is_some_and(Result::is_err) {
        crate::vision_detector::vision_action_in_flight()
    } else {
        crate::vision_detector::begin_vision_action(source.as_ref()).await?
    };
    let (id, _req_id, ack_guard) = crate::macro_action::begin_macro_executor_with_barrier(
        supervisor,
        &action_stamp,
        spec,
        action_stamp.deadline,
        policy,
        require_ack_commit,
        wire_tracker,
    )
    .await?;
    let duration = if planned.adaptive.is_some() {
        expected_duration(&planned)
    } else {
        let approach_extra = supervisor
            .hub()
            .prep_env()
            .and_then(|env| env.approach_plan("grasp_final", planned.request.approach).ok())
            .map_or(0.0, |p| {
                (1.875 * f64::from(p.policy.delta_rad()) / f64::from(p.policy.speed_rad_s))
                    .max(crate::backlash::MIN_SEGMENT_S)
                    + 0.5
            });
        expected_duration(&planned) + approach_extra
    };
    spawn_with_deadline_guarded(supervisor.clone(), source, id, planned, stamp, deadline, vision_guard);
    Ok(((id, duration), ack_guard))
}

/// Spawn the orchestrator for an action begun with `SupervisorInner::begin_macro(stamp, "grasp", ..)`.
pub fn spawn(supervisor: Arc<ArmSupervisor>, source: Arc<dyn DetectionSource>, id: u64, planned: PlannedGrasp, stamp: crate::supervisor::AdmissionStamp) {
    let deadline = Instant::now() + Duration::from_secs_f64(planned.request.setup.timeout_s);
    spawn_with_deadline(supervisor, source, id, planned, stamp, deadline);
}

/// Spawn the `grasp` orchestrator with an explicit pre-committed `deadline` (broker handoff).
pub fn spawn_with_deadline(
    supervisor: Arc<ArmSupervisor>,
    source: Arc<dyn DetectionSource>,
    id: u64,
    planned: PlannedGrasp,
    stamp: crate::supervisor::AdmissionStamp,
    deadline: Instant,
) {
    let vision_guard = crate::vision_detector::vision_action_in_flight();
    spawn_with_deadline_guarded(supervisor, source, id, planned, stamp, deadline, vision_guard);
}

fn spawn_with_deadline_guarded(
    supervisor: Arc<ArmSupervisor>,
    source: Arc<dyn DetectionSource>,
    id: u64,
    planned: PlannedGrasp,
    stamp: crate::supervisor::AdmissionStamp,
    deadline: Instant,
    vision_guard: crate::vision_detector::VisionActionGuard,
) {
    tokio::spawn(async move {
        let _vision_guard = vision_guard;
        run(supervisor, source, id, planned, stamp, deadline).await;
    });
}

