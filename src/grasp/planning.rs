use std::sync::Arc;

use serde_json::Value;

use crate::api::rest::ApiError;
use crate::kinematics::{self, Mat3, Pose, Vec3};
use crate::motion::{self, GoalFrame, MotionError, PlanInput, Speed};
use crate::supervisor::{ArmError, ArmSupervisor, SupervisorInner};

use super::{
    box_centre, camera_down_mode, camera_down_pose, envelope_with_contact, look_down,
    refuse_calibrated_request, table_violation, task_verdict, CamModel, DetectionSource,
    GraspSetup, HeightProbeMode, ModeInfo, TaskModels, TaskRequires, TABLE_CLEARANCE_M,
    TASK_UNSUPPORTED,
};

// ---- request ----

#[derive(Debug, Clone, PartialEq)]
pub struct GraspRequest {
    pub label: String,
    pub box_norm: [f64; 4],
    pub frame_id: Option<u64>,
    /// Effective setup: base, then the task's overrides, then the request's (no `tasks`).
    pub setup: GraspSetup,
    pub tag: Option<String>,
    /// Name of the task in `GraspSetup.tasks`, when the request named one.
    pub task: Option<String>,
    /// The task's perception requirements (empty without a task).
    pub requires: TaskRequires,
    /// The task's model switch (None without a task or when it names no models).
    pub models: Option<TaskModels>,
    /// Request field `approach` for the final descent (src/backlash.rs, `grasp_final`).
    pub approach: crate::backlash::Request,
}

/// Body of `/api/v1/action/grasp`: `{label, box_norm, frame_id?, ts_ms?, task?, overrides?, tag?}`.
/// The setup is `base`, then the overrides of `task`, then `overrides`.
pub fn parse_grasp(body: &Value, base: &GraspSetup) -> Result<GraspRequest, MotionError> {
    let bad = |m: &str| MotionError::InvalidCommand(m.to_string());
    let label = body.get("label").and_then(Value::as_str).filter(|l| !l.is_empty() && l.len() <= 64)
        .ok_or_else(|| bad("label must be a non-empty string of at most 64 bytes"))?.to_string();
    let b = body.get("box_norm").and_then(Value::as_array).filter(|b| b.len() == 4)
        .ok_or_else(|| bad("box_norm must be [x0, y0, x1, y1] in [0, 1]"))?;
    let mut box_norm = [0.0; 4];
    for (o, v) in box_norm.iter_mut().zip(b) {
        *o = v.as_f64().filter(|x| (0.0..=1.0).contains(x)).ok_or_else(|| bad("box_norm must be [x0, y0, x1, y1] in [0, 1]"))?;
    }
    if box_norm[2] <= box_norm[0] || box_norm[3] <= box_norm[1] { return Err(bad("box_norm needs x0 < x1 and y0 < y1")); }
    let frame_id = match body.get("frame_id") {
        None | Some(Value::Null) => None,
        Some(v) => Some(v.as_u64().ok_or_else(|| bad("frame_id must be a non-negative integer"))?),
    };
    if body.get("ts_ms").is_some_and(|v| !v.is_null() && !v.is_number()) { return Err(bad("ts_ms must be a number")); }
    let task = match body.get("task") {
        None | Some(Value::Null) => None,
        Some(Value::String(s)) => Some(s.clone()),
        _ => return Err(bad("task must be a string")),
    };
    let mut setup = base.without_tasks();
    let mut requires = TaskRequires::default();
    let mut models = None;
    if let Some(name) = &task {
        let t = base.tasks.get(name).ok_or_else(|| bad(&format!("unknown grasp task: {name}")))?;
        setup = setup.patched_request(&Value::Object(t.overrides.clone()))
            .map_err(|e| MotionError::InvalidCommand(format!("grasp task {name}: {e}")))?;
        requires = t.requires;
        models = t.models.clone();
    }
    match body.get("overrides") {
        None | Some(Value::Null) => {}
        Some(o) => setup = setup.patched_request(o).map_err(MotionError::InvalidCommand)?,
    }
    let tag = match body.get("tag") {
        None | Some(Value::Null) => None,
        Some(Value::String(s)) if s.len() <= 64 => Some(s.clone()),
        _ => return Err(bad("tag must be a string of at most 64 bytes")),
    };
    let approach = crate::backlash::parse_request(body)?;
    Ok(GraspRequest { label, box_norm, frame_id, setup, tag, task, requires, models, approach })
}

/// Adaptive grasp preparation state captured under the owner snapshot (`strategy: "adaptive"`).
pub struct PendingAdaptive {
    pub snapshot: crate::grasp_select::Snapshot,
    pub start_positions: Vec<f32>,
    pub revisions: crate::grasp_select::AdaptiveRevisions,
    pub gripper_policy: crate::model::GripperPolicy,
    pub gripper_limit: crate::model::JointLimit,
    pub close_speed_limit: Option<f32>,
    pub has_gravity_model: bool,
}

/// Adaptive grasp planned state (`strategy: "adaptive"`) carrying the single authoritative
/// [`crate::grasp_select::AdaptiveRoute`], start positions, and captured revisions.
pub struct PlannedAdaptive {
    pub route: crate::grasp_select::AdaptiveRoute,
    pub start_positions: Vec<f32>,
    pub revisions: crate::grasp_select::AdaptiveRevisions,
    pub expected_generation: Option<u64>,
}

/// A request checked under the lock, ready to be planned outside it.
pub struct PendingGrasp {
    pub input: PlanInput,
    /// Boxed: the setup makes the request large.
    pub request: Box<GraspRequest>,
    pub open_rad: f32,
    pub physical_contact_end_m: Option<[f64; 3]>,
    /// Populated only when `strategy: "adaptive"` is requested (`None` for legacy).
    pub adaptive: Option<Box<PendingAdaptive>>,
}

/// A grasp whose coarse approach (legacy) or whole adaptive route was found reachable.
pub struct PlannedGrasp {
    pub input: PlanInput,
    pub request: GraspRequest,
    /// Estimated target on the grasp plane (base frame).
    pub target: Vec3,
    pub approach_duration: f64,
    /// Camera-down acquisition ([`camera_down_mode`]).
    pub camera_down: bool,
    /// Estimated duration of the camera-down → tool-vertical move (0 without camera-down).
    pub transition_duration: f64,
    /// The provider's mode read by [`crate::grasp::preflight`] (a task with requirements or adaptive grasp);
    /// the orchestrator reads it itself when None.
    pub mode: Option<Result<ModeInfo, String>>,
    /// Populated only when `strategy: "adaptive"` is requested (`None` for legacy).
    pub adaptive: Option<Box<PlannedAdaptive>>,
}

fn adaptive_prep_err(phase: &'static str, err: ArmError) -> ArmError {
    if matches!(err, ArmError::Detailed { .. }) {
        return err;
    }
    let api = ApiError::from(err);
    let status = api.status.as_u16();
    let code = api.code.unwrap_or(api.error);
    let msg = if api.message.starts_with(&format!("[{phase}]")) {
        api.message
    } else {
        format!("[{phase}] {}", api.message)
    };
    ArmError::Detailed {
        status,
        code,
        message: msg,
        details: Box::new(serde_json::json!({
            "failing_phase": phase,
            "outcome": "infeasible",
        })),
    }
}

/// Under the lock: parse and check preconditions for both `strategy: "legacy"` (default) and
/// `strategy: "adaptive"` (phase B live execution).
pub fn prepare(inner: &SupervisorInner, body: &Value, epoch: u64) -> Result<PendingGrasp, ArmError> {
    let strategy = if body.is_object() {
        crate::grasp_select::parse_strategy(body)?
    } else {
        crate::grasp_select::Strategy::Legacy
    };
    let mut pending = match prepare_any(inner, body, epoch) {
        Ok(p) => p,
        Err(e) if strategy == crate::grasp_select::Strategy::Adaptive => {
            return Err(adaptive_prep_err("request", e));
        }
        Err(e) => return Err(e),
    };
    if strategy == crate::grasp_select::Strategy::Adaptive {
        let snap = crate::grasp_select::capture(inner, &pending, body)
            .map_err(|e| adaptive_prep_err("snapshot", e))?;
        let gripper_idx = inner
            .config
            .gripper_index()
            .ok_or_else(|| adaptive_prep_err("snapshot", ArmError::from(MotionError::Unsupported("Grasp needs a gripper joint".into()))))?;
        let gripper_policy = inner
            .config
            .gripper_policy
            .clone()
            .ok_or_else(|| adaptive_prep_err("snapshot", ArmError::from(MotionError::Unsupported("Grasp needs a gripper policy".into()))))?;
        let gripper_limit = inner
            .config
            .limits
            .get(gripper_idx)
            .cloned()
            .ok_or_else(|| adaptive_prep_err("snapshot", ArmError::from(MotionError::Unsupported("Grasp needs a gripper joint limit".into()))))?;
        let revisions = crate::grasp_select::AdaptiveRevisions {
            workspace_revision: inner.workspace_revision,
            calibration_revision: inner.calibration_revision,
            grasp_setup_revision: inner.grasp_setup_revision,
            gripper_calib_revision: inner.gripper_calib_revision,
            power_budget_revision: inner.power_budget_revision,
        };
        pending.adaptive = Some(Box::new(PendingAdaptive {
            snapshot: snap,
            start_positions: inner.commanded_positions.clone(),
            revisions,
            gripper_policy,
            gripper_limit,
            close_speed_limit: inner.gripper_close_speed_limit(),
            has_gravity_model: inner.config.gravity_model.is_some(),
        }));
    }
    Ok(pending)
}

/// [`prepare`] for any strategy (used by legacy and dry-run paths).
fn prepare_any(inner: &SupervisorInner, body: &Value, epoch: u64) -> Result<PendingGrasp, ArmError> {
    if !body.is_object() { return Err(MotionError::InvalidCommand("Command body must be an object".into()).into()); }
    inner.check_commissioning_for_ordinary_motion("grasp")?;
    let Some(base) = inner.grasp_setup() else {
        return Err(MotionError::Unsupported("Grasp needs a profile with kinematics, a gripper policy and a grasp setup".into()).into());
    };
    if inner.camera_calibrated() { refuse_calibrated_request(&base, body)?; }
    let Some(policy) = inner.config.gripper_policy.clone() else {
        return Err(MotionError::Unsupported("Grasp needs a gripper policy".into()).into());
    };
    let request = parse_grasp(body, &base)?;
    if epoch != inner.safety.epoch_id() {
        return Err(crate::safety::SafetyError::EpochMismatch { expected: inner.safety.epoch_id(), received: epoch }.into());
    }
    let input = inner.plan_input()?;
    Ok(PendingGrasp {
        input,
        request: Box::new(request),
        open_rad: policy.open_rad,
        physical_contact_end_m: inner.config.effective_physical_contact_end_m(),
        adaptive: None,
    })
}

fn prepare_from_env_parts(
    env: &crate::rt_core::requests::actions::ActionPrepEnv,
    commanded_positions: &[f32],
    input: PlanInput,
    body: &Value,
    strategy: crate::grasp_select::Strategy,
) -> Result<PendingGrasp, ArmError> {
    let wrap = |phase: &'static str, e: ArmError| {
        if strategy == crate::grasp_select::Strategy::Adaptive {
            adaptive_prep_err(phase, e)
        } else {
            e
        }
    };
    if !body.is_object() {
        return Err(wrap("request", MotionError::InvalidCommand("Command body must be an object".into()).into()));
    }
    if env.config.active_tool.commissioning_required {
        return Err(wrap(
            "request",
            crate::safety::SafetyError::CommissioningRequired(
                "grasp (blocked while active tool requires commissioning; use jog/move_to/move_relative/gripper at speed_scale <= 0.2)".into(),
            )
            .into(),
        ));
    }
    let Some(base) = env.grasp_setup.clone() else {
        return Err(wrap(
            "snapshot",
            MotionError::Unsupported(
                "Grasp needs a profile with kinematics, a gripper policy and a grasp setup".into(),
            )
            .into(),
        ));
    };
    if env.camera_calibrated {
        refuse_calibrated_request(&base, body).map_err(|e| wrap("request", e.into()))?;
    }
    let Some(policy) = env.config.gripper_policy.clone() else {
        return Err(wrap("snapshot", MotionError::Unsupported("Grasp needs a gripper policy".into()).into()));
    };
    let request = parse_grasp(body, &base).map_err(|e| wrap("request", e.into()))?;
    let mut pending = PendingGrasp {
        input,
        request: Box::new(request),
        open_rad: policy.open_rad,
        physical_contact_end_m: env.config.effective_physical_contact_end_m(),
        adaptive: None,
    };
    if strategy == crate::grasp_select::Strategy::Adaptive {
        let gsnap = crate::grasp_select::capture_from_env(env, &pending, body)
            .map_err(|e| adaptive_prep_err("snapshot", e))?;
        let gripper_idx = env
            .config
            .gripper_index()
            .ok_or_else(|| adaptive_prep_err("snapshot", ArmError::from(MotionError::Unsupported("Grasp needs a gripper joint".into()))))?;
        let gripper_limit = env
            .config
            .limits
            .get(gripper_idx)
            .cloned()
            .ok_or_else(|| adaptive_prep_err("snapshot", ArmError::from(MotionError::Unsupported("Grasp needs a gripper joint limit".into()))))?;
        let revisions = crate::grasp_select::AdaptiveRevisions {
            workspace_revision: env.workspace_revision,
            calibration_revision: env.calibration_revision,
            grasp_setup_revision: env.grasp_setup_revision,
            gripper_calib_revision: env.gripper_calib_revision,
            power_budget_revision: env.power_budget_revision,
        };
        pending.adaptive = Some(Box::new(PendingAdaptive {
            snapshot: gsnap,
            start_positions: commanded_positions.to_vec(),
            revisions,
            gripper_policy: policy,
            gripper_limit,
            close_speed_limit: env.gripper_close_speed_limit,
            has_gravity_model: env.config.gravity_model.is_some(),
        }));
    }
    Ok(pending)
}

/// Prepare a `grasp` action from a published [`ActionSnapshot`] (0 `SupervisorLock` on
/// `OWNER_EXECUTOR`) for both `strategy: "legacy"` and `strategy: "adaptive"`.
pub fn prepare_from_snapshot(
    snap: &crate::rt_core::requests::actions::ActionSnapshot,
    body: &Value,
    epoch: u64,
) -> Result<PendingGrasp, ArmError> {
    if !body.is_object() {
        return Err(MotionError::InvalidCommand("Command body must be an object".into()).into());
    }
    let strategy = crate::grasp_select::parse_strategy(body)?;
    if epoch != snap.owner.authority.epoch {
        return Err(crate::safety::SafetyError::EpochMismatch {
            expected: snap.owner.authority.epoch,
            received: epoch,
        }
        .into());
    }
    let input = snap.plan_input_in(GoalFrame::EndLink)?;
    prepare_from_env_parts(&snap.env, &snap.commanded_positions, input, body, strategy)
}

/// Lease-free read-only preparation from a [`crate::rt_core::requests::actions::ReadOnlySnapshot`]
/// (0 `SupervisorLock` on `OWNER_EXECUTOR`, used by `dry_run_plan`).
fn prepare_from_read_only(
    ro: &crate::rt_core::requests::actions::ReadOnlySnapshot,
    body: &Value,
    strategy: crate::grasp_select::Strategy,
) -> Result<PendingGrasp, ArmError> {
    let input = ro.plan_input_in(GoalFrame::EndLink)?;
    prepare_from_env_parts(&ro.env, &ro.commanded_positions, input, body, strategy)
}

/// TCP height of the grasp plane (where box centres are assumed to lie).
pub(super) fn grasp_plane(setup: &GraspSetup) -> f64 { setup.table_z_m + setup.grasp_height_m }

/// Look-down approach pose above `target`.
pub(super) fn approach_pose(r_now: &Mat3, target: &Vec3, setup: &GraspSetup) -> Pose {
    Pose::new(look_down(r_now), [target[0], target[1], setup.table_z_m + setup.approach_height_m])
}

/// Outside the lock: estimate the target from the box-centre ray and plan the approach
/// (legacy) or select & preflight the whole adaptive route (`strategy: "adaptive"`).
pub fn plan(pending: PendingGrasp) -> Result<PlannedGrasp, ArmError> {
    let PendingGrasp { input, request, open_rad, physical_contact_end_m, adaptive } = pending;
    if let Some(pa) = adaptive {
        let sel = crate::grasp_select::select_adaptive_route(
            &pa.snapshot,
            &pa.gripper_policy,
            &pa.gripper_limit,
            pa.close_speed_limit,
            pa.has_gravity_model,
        );
        if let Some(route) = sel.route.filter(|_| sel.selection.outcome == crate::grasp_select::Outcome::Selected) {
            let target = [route.target_centroid[0], route.target_centroid[1], route.plan.grasp.p[2]];
            let approach_duration = route.budget.pregrasp_s;
            return Ok(PlannedGrasp {
                input,
                request: *request,
                target,
                approach_duration,
                camera_down: pa.snapshot.camera_down,
                transition_duration: 0.0,
                mode: None,
                adaptive: Some(Box::new(PlannedAdaptive {
                    route,
                    start_positions: pa.start_positions,
                    revisions: pa.revisions,
                    expected_generation: None,
                })),
            });
        }
        let code = sel.selection.code.unwrap_or_else(|| "NO_CANDIDATE".into());
        let phase = sel.failing_phase.unwrap_or("selection");
        let base_msg = sel.selection.message.unwrap_or_else(|| "adaptive grasp infeasible".into());
        // Preserve dry-run/live parity for checked-only hints: only emit the legacy hint when
        // the legacy route actually passes on this exact snapshot.
        let checked_hint = sel.checked_hint.or_else(|| {
            let legacy_pending = PendingGrasp {
                input,
                request,
                open_rad,
                physical_contact_end_m,
                adaptive: None,
            };
            plan(legacy_pending)
                .ok()
                .map(|_| "legacy strategy passed dry_run for this target".to_string())
        });
        let message = match &checked_hint {
            Some(hint) => format!("[{phase}] {base_msg}; hint: {hint}"),
            None => format!("[{phase}] {base_msg}"),
        };
        let mut details = serde_json::Map::new();
        details.insert("failing_phase".into(), serde_json::json!(phase));
        details.insert("outcome".into(), serde_json::json!(sel.selection.outcome.name()));
        if let Some(hint) = checked_hint {
            details.insert("hint".into(), serde_json::json!(hint));
        }
        return Err(ArmError::Detailed {
            status: 422,
            code,
            message,
            details: Box::new(Value::Object(details)),
        });
    }
    let request = *request;
    let setup = &request.setup;
    let pose_end = input.arm.fk(&input.q0);
    let pose_tcp = motion::end_to_tcp(&pose_end, &input.tcp_offset);
    let cam = CamModel::assumed(&input.arm, setup, &input.cam);
    let centre = cam.to_px(box_centre(&request.box_norm));
    let target = cam.plane_point(&pose_end, centre, grasp_plane(setup)).ok_or_else(|| ArmError::from(MotionError::InvalidCommand(
        "the box centre ray does not meet the table plane in front of the camera".into())))?;
    let camera_down = camera_down_mode(setup, &input.cam).map_err(MotionError::InvalidCommand)?;
    let pts = envelope_with_contact(input.ws.as_ref(), &input.cam, &cam.rec, physical_contact_end_m);
    let is_5dof = input.arm.arm_dof() == 5;
    let r_ref_p = if camera_down { &pose_end.p } else { &pose_tcp.p };
    let r_ref = if is_5dof {
        motion::apply_auto_yaw(r_ref_p, &target, pose_end.r)
    } else {
        pose_end.r
    };
    // the envelope stays above the table on the approach and (camera-down) the transition
    let (goal, tool) = if camera_down {
        let goal = camera_down_pose(&r_ref, &target, setup, &cam, &pts);
        let tool = approach_pose(&goal.r, &target, setup);
        let tool_end = motion::tcp_to_end(&tool, &input.tcp_offset);
        if let Some(e) = table_violation(&goal, &tool_end, &pts, setup.table_z_m) { return Err(e.into()); }
        (goal, tool)
    } else {
        let goal = approach_pose(&r_ref, &target, setup);
        (goal, goal)
    };
    let goal_end = if camera_down { goal } else { motion::tcp_to_end(&goal, &input.tcp_offset) };
    if let Some(e) = table_violation(&pose_end, &goal_end, &pts, setup.table_z_m) { return Err(e.into()); }
    let mut cur = input.clone();
    cur.j7 = open_rad;
    cur.goal_frame = if camera_down { GoalFrame::EndLink } else { GoalFrame::Tcp };
    let path = motion::plan_cartesian_ex(&cur, goal, Speed { linear: setup.approach_speed_mps, angular: motion::DEFAULT_ANGULAR_RPS }, is_5dof)?;
    crate::backlash::check_table_clear(&input.arm, &pts, setup.table_z_m + TABLE_CLEARANCE_M, path.q.iter().map(|q| q.as_slice()))
        .map_err(|v| ArmError::from(v.into_motion_error()))?;
    let last = path.last();
    cur.q0 = std::array::from_fn(|i| f64::from(last[i]));
    cur.goal_frame = GoalFrame::Tcp;
    let transition_duration = if camera_down {
        let trans_path = motion::plan_cartesian_ex(&cur, tool, Speed { linear: setup.approach_speed_mps, angular: motion::DEFAULT_ANGULAR_RPS }, is_5dof)?;
        crate::backlash::check_table_clear(&input.arm, &pts, setup.table_z_m + TABLE_CLEARANCE_M, trans_path.q.iter().map(|q| q.as_slice()))
            .map_err(|v| ArmError::from(v.into_motion_error()))?;
        let last = trans_path.last();
        cur.q0 = std::array::from_fn(|i| f64::from(last[i]));
        transition_estimate(&goal, &target, setup)
    } else {
        0.0
    };
    let grasp_pose = Pose::new(tool.r, [target[0], target[1], setup.table_z_m + setup.grasp_height_m]);
    let _descend_path = motion::plan_cartesian_ex(&cur, grasp_pose, Speed { linear: setup.descend_speed_mps, angular: motion::DEFAULT_ANGULAR_RPS }, is_5dof)?;
    Ok(PlannedGrasp { input, request, target, approach_duration: path.duration, camera_down, transition_duration, mode: None, adaptive: None })
}

/// Duration of the transition from the camera-down pose `from` to the tool-vertical pose
/// above `target`, from the planner's Cartesian speed bound (quintic peak 1.875 / T, +10 %);
/// joint speed limits can make it longer.
pub fn transition_estimate(from: &Pose, target: &Vec3, setup: &GraspSetup) -> f64 {
    let to = approach_pose(&from.r, target, setup);
    let length = kinematics::norm(&kinematics::sub(&to.p, &from.p));
    let theta = kinematics::angle_between(&to.r, &from.r);
    1.1 * 1.875 * (length / setup.approach_speed_mps).max(theta / motion::DEFAULT_ANGULAR_RPS)
}

/// Rough total duration for the ack.
pub fn expected_duration(planned: &PlannedGrasp) -> f64 {
    if let Some(pa) = &planned.adaptive {
        return pa.route.budget.nominal_total_s;
    }
    let s = &planned.request.setup;
    let descend = (s.approach_height_m - s.grasp_height_m) * 1.875 / s.descend_speed_mps;
    let lift = (s.lift_height_m - s.grasp_height_m) * 1.875 / s.descend_speed_mps;
    let mut probe = if s.height_probe_m > 0.0 { 2.0 * s.height_probe_m * 1.875 / s.approach_speed_mps + 1.5 } else { 0.0 };
    // lateral: B/2 + B + B/2 of motion and two frame sets, on top of a possible up fallback
    if s.height_probe_m > 0.0 && s.height_probe_mode == HeightProbeMode::Lateral { probe += 2.0 * 1.875 * s.lateral_probe_m / s.approach_speed_mps + 1.5; }
    let recentre = if s.recentre_height_m > 0.0 && !planned.camera_down { 3.0 } else { 0.0 };
    let lift_check = s.post_lift_settle_ms as f64 / 1000.0;
    let base = lift_check + 6.5 + planned.approach_duration + 4.0 * 1.5 + descend + 2.0 + lift + probe + recentre;
    // camera-down: the transition move and its re-check frame
    if planned.camera_down { base + planned.transition_duration + 1.5 } else { base }
}

/// Result of a grasp dry run.
#[derive(Debug, Clone, PartialEq)]
pub struct DryRun {
    pub reachable: bool,
    pub code: Option<String>,
    pub message: Option<String>,
    /// Requests with a task: whether the provider serves its requirements (true when it has
    /// none or the provider cannot be read) and why not (`needs model switch`, supported, when
    /// the task's models differ from the resident ones).
    pub task: Option<(bool, Option<String>)>,
    /// `strategy: "adaptive"`: the selection reply ([`crate::grasp_select::Selection::to_json`])
    /// plus `legacy` (the legacy verdict on the same request).
    pub adaptive: Option<Value>,
}

impl DryRun {
    /// The reply fields: `reachable`, `code`, `message`, and with a task `task_supported`,
    /// `task_reason`.
    pub fn fields(&self) -> serde_json::Map<String, Value> {
        let mut m = serde_json::Map::new();
        m.insert("reachable".into(), serde_json::json!(self.reachable));
        m.insert("code".into(), serde_json::json!(self.code));
        m.insert("message".into(), serde_json::json!(self.message));
        if let Some((supported, reason)) = &self.task {
            m.insert("task_supported".into(), serde_json::json!(supported));
            m.insert("task_reason".into(), serde_json::json!(reason));
        }
        if let Some(Value::Object(a)) = &self.adaptive {
            for (k, v) in a { m.insert(k.clone(), v.clone()); }
        }
        m
    }
}

/// Dry-run reachability check: prepare + plan only, no macro started, no orchestrator spawned.
/// Returns (reachable, code, message). The task requirements are not checked (see
/// [`dry_run_task`]).
pub async fn dry_run(supervisor: &Arc<ArmSupervisor>, body: &Value) -> (bool, Option<String>, Option<String>) {
    let (reachable, code, message, _, _) = dry_run_plan(supervisor, body).await;
    (reachable, code, message)
}

/// [`dry_run`] plus the task check ([`task_verdict`]): one `/api/mode` reading when the
/// request's task has requirements or models, none otherwise; never a model switch. The task
/// result does not change `reachable`.
pub async fn dry_run_task(supervisor: &Arc<ArmSupervisor>, source: &dyn DetectionSource, body: &Value) -> DryRun {
    let (mut reachable, mut code, mut message, request, mut adaptive) = dry_run_plan(supervisor, body).await;
    let is_adaptive = adaptive.is_some();
    let task = match request {
        Some((Some(name), requires, models)) => Some(if requires.is_empty() && models.is_none() {
            (true, None)
        } else {
            match source.mode_info().await {
                Ok(m) => task_verdict(&name, &requires, models.as_ref(), &m),
                Err(e) if is_adaptive => (false, Some(format!("vision provider unreachable: {e}"))),
                Err(_) => (true, None),
            }
        }),
        _ => None,
    };
    if is_adaptive {
        if let Some((false, ref reason)) = task {
            let is_unreach = reason.as_deref().is_some_and(|r| r.starts_with("vision provider unreachable:"));
            let err_code = if is_unreach { "DETECTION_UNAVAILABLE" } else { TASK_UNSUPPORTED };
            let phase = if is_unreach { "vision_preflight" } else { "task" };
            reachable = false;
            code = Some(err_code.to_string());
            message = reason.clone();
            if let Some(Value::Object(ref mut obj)) = adaptive {
                obj.insert("outcome".into(), serde_json::json!("infeasible"));
                obj.insert("failing_phase".into(), serde_json::json!(phase));
            }
        }
    }
    DryRun { reachable, code, message, task, adaptive }
}

/// Dry-run verdict: (reachable, code, message, (task, requires, models) when the request parsed,
/// the adaptive selection reply when `strategy` is `"adaptive"`).
///
/// Read-only planning/selection path (acquires only a read lock on `SupervisorInner` on Legacy,
/// or a lock-free [`crate::rt_core::requests::actions::ReadOnlySnapshot`] on `OWNER_EXECUTOR`,
/// and runs off-lock in `spawn_blocking`): never starts motion or actuator output, so no
/// `Admitted` token is required.
type DryRunPlan = (bool, Option<String>, Option<String>, Option<(Option<String>, TaskRequires, Option<TaskModels>)>, Option<Value>);

async fn dry_run_plan(supervisor: &Arc<ArmSupervisor>, body: &Value) -> DryRunPlan {
    let rejected = |e: ArmError| {
        let api = ApiError::from(e);
        (false, api.code.or(Some(api.error)), Some(api.message))
    };
    let strategy = match crate::grasp_select::parse_strategy(body) {
        Ok(s) => s,
        Err(e) => { let (r, c, m) = rejected(e.into()); return (r, c, m, None, None); }
    };
    let pending_res: Result<(PendingGrasp, Option<Box<PendingAdaptive>>), ArmError> =
        if supervisor.owner_mode() == crate::supervisor::OWNER_EXECUTOR {
            // Lease-free read-only snapshot on RT Executor (0 SupervisorLock, no AdmissionStamp).
            match crate::rt_core::requests::actions::ReadOnlySnapshot::capture(supervisor).await {
                Ok(ro) => prepare_from_read_only(&ro, body, strategy).map(|mut p| {
                    let pa = p.adaptive.take();
                    (p, pa)
                }),
                Err(e) => Err(e),
            }
        } else {
            let inner = supervisor.inner().read().await;
            prepare_any(&inner, body, inner.safety.epoch_id()).and_then(|p| {
                let pa = match strategy {
                    crate::grasp_select::Strategy::Adaptive => {
                        let snap = crate::grasp_select::capture(&inner, &p, body)?;
                        let g_idx = inner.config.gripper_index().ok_or_else(|| {
                            ArmError::from(MotionError::Unsupported("Grasp needs a gripper joint".into()))
                        })?;
                        let gripper_policy = inner.config.gripper_policy.clone().ok_or_else(|| {
                            ArmError::from(MotionError::Unsupported("Grasp needs a gripper policy".into()))
                        })?;
                        let gripper_limit = inner.config.limits.get(g_idx).cloned().ok_or_else(|| {
                            ArmError::from(MotionError::Unsupported("Grasp needs a gripper joint limit".into()))
                        })?;
                        let revisions = crate::grasp_select::AdaptiveRevisions {
                            workspace_revision: inner.workspace_revision,
                            calibration_revision: inner.calibration_revision,
                            grasp_setup_revision: inner.grasp_setup_revision,
                            gripper_calib_revision: inner.gripper_calib_revision,
                            power_budget_revision: inner.power_budget_revision,
                        };
                        Some(Box::new(PendingAdaptive {
                            snapshot: snap,
                            start_positions: inner.commanded_positions.clone(),
                            revisions,
                            gripper_policy,
                            gripper_limit,
                            close_speed_limit: inner.gripper_close_speed_limit(),
                            has_gravity_model: inner.config.gravity_model.is_some(),
                        }))
                    }
                    crate::grasp_select::Strategy::Legacy => None,
                };
                Ok((p, pa))
            })
        };
    match pending_res {
        Ok((pending, pa)) => {
            let task = Some((pending.request.task.clone(), pending.request.requires, pending.request.models.clone()));
            let joined = tokio::task::spawn_blocking(move || {
                let legacy = plan(pending).map(|_| ());
                let adaptive_sel = pa.map(|pa| {
                    let sel = crate::grasp_select::select_adaptive_route(
                        &pa.snapshot,
                        &pa.gripper_policy,
                        &pa.gripper_limit,
                        pa.close_speed_limit,
                        pa.has_gravity_model,
                    );
                    (pa.snapshot, sel)
                });
                (legacy, adaptive_sel)
            }).await;
            match joined {
                Ok((legacy, None)) => {
                    let (reachable, code, message) = match legacy { Ok(()) => (true, None, None), Err(e) => rejected(e) };
                    (reachable, code, message, task, None)
                }
                Ok((legacy, Some((snap, sel)))) => {
                    let (lr, lc, lm) = match legacy { Ok(()) => (true, None, None), Err(e) => rejected(e) };
                    let mut v = sel.to_json(&snap, Some((lr, lc.as_deref(), lm.as_deref())));
                    v["legacy"] = serde_json::json!({"reachable": lr, "code": lc, "message": lm});
                    let reachable = sel.reachable();
                    let code = sel.selection.code;
                    let message = sel.selection.message;
                    (reachable, code, message, task, Some(v))
                }
                Err(e) => (false, Some("INTERNAL".into()), Some(format!("planner task failed: {e}")), task, None),
            }
        }
        Err(e) => {
            let failing_phase = match &e {
                ArmError::Detailed { details, .. } => details
                    .get("failing_phase")
                    .and_then(Value::as_str)
                    .map(str::to_string),
                _ => None,
            };
            let (reachable, code, message) = rejected(e);
            let adaptive = (strategy == crate::grasp_select::Strategy::Adaptive).then(|| {
                serde_json::json!({
                    "strategy": "adaptive",
                    "outcome": "infeasible",
                    "failing_phase": failing_phase.as_deref().unwrap_or("request"),
                })
            });
            (reachable, code, message, None, adaptive)
        }
    }
}
