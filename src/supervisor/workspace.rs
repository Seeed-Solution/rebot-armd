//! Workspace policy checks, conservative keep-out sweeps, and bounded RT evaluators.

#[allow(unused_imports)]
use super::*;

/// [`SupervisorInner::keepout_sweep`] with an explicit arm model (planning outside the lock).
pub fn keepout_sweep_with(
    model: &crate::chain::ArmModel,
    ws: &crate::model::WorkspacePolicy,
    cam: &crate::model::CameraMount,
    at: &dyn Fn(f64) -> Vec<f32>,
    travel: &dyn Fn(f64, f64) -> Vec<f64>,
    knots: &[f64],
    budget: usize,
) -> Result<Vec<Vec<f32>>, SafetyError> {
    keepout_sweep_with_tcp_tol(model, ws, cam, [0.0; 3], crate::kinematics::WORKSPACE_EPS, at, travel, knots, budget)
}

#[allow(clippy::too_many_arguments)]
/// The commanded joints `start` with the arm joints replaced by `q` (gripper and extra joints kept).
pub(crate) fn joint_goal_target_from(start: &[f32], q: &[f64; 6], arm_dof: usize, gripper_idx: Option<usize>) -> Vec<f32> {
    let mut target = start.to_vec();
    for (i, t) in target.iter_mut().enumerate().take(arm_dof.min(6)) {
        if Some(i) != gripper_idx { *t = q[i] as f32; }
    }
    target
}

/// Supervisor state a `path: "joint"` quintic is planned from ([`SupervisorInner::joint_goal_snapshot`]).
#[derive(Debug, Clone)]
pub struct JointGoalSnapshot {
    pub start: Vec<f32>,
    pub measured: Vec<f32>,
    pub v0: Vec<f32>,
    pub limits: Vec<crate::model::JointLimit>,
    pub arm_dof: usize,
    pub gripper_index: Option<usize>,
}

/// A joint quintic planned, power-fitted and workspace-checked without the supervisor lock
/// ([`prepare_quintic`]), with the inputs it was made from.
#[derive(Debug, Clone)]
pub struct PreparedQuintic {
    pub(crate) start: Vec<f32>,
    pub(crate) measured: Vec<f32>,
    pub(crate) v0: Vec<f32>,
    pub(crate) target: Vec<f32>,
    pub(crate) limits: Vec<crate::model::JointLimit>,
    pub(crate) workspace_revision: u64,
    pub(crate) tcp_offset: [f64; 3],
    pub(crate) power: Option<crate::power::PowerBudget>,
    pub(crate) static_restricted: bool,
    pub(crate) checked_workspace: bool,
    pub(crate) traj: MultiJointQuinticTrajectory,
    pub(crate) power_scale: Option<f64>,
}

/// The work of [`SupervisorInner::start_quintic`] that needs no lock, for the joints `q`:
/// quintic, power fit and swept-workspace check, from `input` and `snap`. None when any of
/// them fails: the install then repeats them under the lock and returns their error.
pub fn prepare_quintic(input: &PlanInput, snap: &JointGoalSnapshot, q: &[f64; 6]) -> Option<PreparedQuintic> {
    let target = joint_goal_target_from(&snap.start, q, snap.arm_dof, snap.gripper_index);
    let zeros = vec![0.0; target.len()];
    let traj = TrajectoryPlanner::plan(&snap.start, &snap.v0, &target, &zeros, None, &snap.limits).ok()?;
    let (traj, power_scale) = crate::power::power_fit_quintic_with(
        input.power.as_ref(), &input.arm, input.arm.arm_dof(), traj, &snap.start, &snap.v0, &target, &snap.limits, true, input.static_restricted).ok()?;
    if let Some(ws) = input.ws.as_ref() {
        let knots: Vec<f64> = (0..=20).map(|k| traj.duration * k as f64 / 20.0).collect();
        let at = |t: f64| traj.sample(t).positions;
        let travel = |t0: f64, t1: f64| traj.travel_bound(t0, t1);
        let eps = crate::kinematics::WORKSPACE_EPS;
        let samples = keepout_sweep_with_tcp_measured(&input.arm, ws, &input.cam, input.tcp_offset, eps, Some(&snap.measured), &at, &travel, &knots, MAX_KEEPOUT_EVALS_PER_PLAN).ok()?;
        check_path_with_tcp_measured(&input.arm, ws, &input.cam, input.tcp_offset, eps, Some(&snap.measured), &snap.start, samples.iter().map(Vec::as_slice)).ok()?;
    }
    Some(PreparedQuintic {
        start: snap.start.clone(),
        measured: snap.measured.clone(),
        v0: snap.v0.clone(),
        target,
        limits: snap.limits.clone(),
        workspace_revision: input.workspace_revision,
        tcp_offset: input.tcp_offset,
        power: input.power.clone(),
        static_restricted: input.static_restricted,
        checked_workspace: input.ws.is_some(),
        traj,
        power_scale,
    })
}

#[allow(clippy::too_many_arguments)]
pub fn keepout_sweep_with_tcp_tol(
    model: &crate::chain::ArmModel,
    ws: &crate::model::WorkspacePolicy,
    cam: &crate::model::CameraMount,
    tcp_offset: [f64; 3],
    escape_tol: f64,
    at: &dyn Fn(f64) -> Vec<f32>,
    travel: &dyn Fn(f64, f64) -> Vec<f64>,
    knots: &[f64],
    budget: usize,
) -> Result<Vec<Vec<f32>>, SafetyError> {
    keepout_sweep_with_tcp_measured(model, ws, cam, tcp_offset, escape_tol, None, at, travel, knots, budget)
}

#[allow(clippy::too_many_arguments)]
pub fn keepout_sweep_with_tcp_measured(
    model: &crate::chain::ArmModel,
    ws: &crate::model::WorkspacePolicy,
    cam: &crate::model::CameraMount,
    tcp_offset: [f64; 3],
    escape_tol: f64,
    measured: Option<&[f32]>,
    at: &dyn Fn(f64) -> Vec<f32>,
    travel: &dyn Fn(f64, f64) -> Vec<f64>,
    knots: &[f64],
    budget: usize,
) -> Result<Vec<Vec<f32>>, SafetyError> {
    crate::chain::WorkspaceEvaluator::new_with_tcp(model, ws, cam, tcp_offset)
        .with_escape_tol(escape_tol)
        .sweep_knots_from_measured(measured, at, travel, knots, budget, "sweep")
        .map_err(SafetyError::from_workspace)
}

/// Workspace rule of [`SupervisorInner::check_workspace_path`] with an explicit arm model:
/// reject a sample that worsens or newly intrudes on any `(geometry_id, constraint_id)`
/// relative to `start` (per-constraint monotonic escape + terminal return rule).
pub fn check_path_with<'a>(
    model: &crate::chain::ArmModel,
    ws: &crate::model::WorkspacePolicy,
    cam: &crate::model::CameraMount,
    start: &[f32],
    samples: impl IntoIterator<Item = &'a [f32]>,
) -> Result<(), SafetyError> {
    check_path_with_tcp_measured(model, ws, cam, [0.0; 3], crate::kinematics::WORKSPACE_EPS, None, start, samples)
}

pub fn check_path_with_tcp_tol<'a>(
    model: &crate::chain::ArmModel,
    ws: &crate::model::WorkspacePolicy,
    cam: &crate::model::CameraMount,
    tcp_offset: [f64; 3],
    escape_tol: f64,
    start: &[f32],
    samples: impl IntoIterator<Item = &'a [f32]>,
) -> Result<(), SafetyError> {
    check_path_with_tcp_measured(model, ws, cam, tcp_offset, escape_tol, None, start, samples)
}

#[allow(clippy::too_many_arguments)]
pub fn check_path_with_tcp_measured<'a>(
    model: &crate::chain::ArmModel,
    ws: &crate::model::WorkspacePolicy,
    cam: &crate::model::CameraMount,
    tcp_offset: [f64; 3],
    escape_tol: f64,
    measured: Option<&[f32]>,
    start: &[f32],
    samples: impl IntoIterator<Item = &'a [f32]>,
) -> Result<(), SafetyError> {
    let eval = crate::chain::WorkspaceEvaluator::new_with_tcp(model, ws, cam, tcp_offset)
        .with_escape_tol(escape_tol);
    let (start_depths, mut min_depths, start_outside, mut clr, mut has_samples) =
        if let Some(m) = crate::chain::WorkspaceEvaluator::valid_distinct_measured(measured, start) {
            let mut cm = eval.alloc_buf();
            eval.eval_into(m, &mut cm, "plan").map_err(SafetyError::from_workspace)?;
            let sd = eval.init_min_depths(&cm);
            let mut md = sd.clone();
            let so = crate::chain::WorkspaceEvaluator::any_outside(sd.as_slice());
            let mut rem = MAX_KEEPOUT_EVALS_PER_PLAN;
            let c_start = eval
                .sweep_linear_step_with_start_outside(
                    m, start, &cm, &mut md, &mut rem, MAX_KEEPOUT_EVALS_PER_PLAN, "plan", "sweep", so,
                )
                .map_err(|(_, f)| SafetyError::from_workspace(f))?;
            (sd, md, so, c_start, true)
        } else {
            let mut c0 = eval.alloc_buf();
            eval.eval_into(start, &mut c0, "plan").map_err(SafetyError::from_workspace)?;
            let sd = eval.init_min_depths(&c0);
            let so = crate::chain::WorkspaceEvaluator::any_outside(sd.as_slice());
            let clr = eval.alloc_buf();
            (sd.clone(), sd, so, clr, false)
        };
    for q in samples {
        eval.eval_into(q, &mut clr, "plan").map_err(SafetyError::from_workspace)?;
        eval.check_and_advance_with_start_outside(&clr, &mut min_depths, "plan", start_outside)
            .map_err(SafetyError::from_workspace)?;
        has_samples = true;
    }
    if has_samples {
        eval.check_terminal_progress(start_depths.as_slice(), clr.as_slice(), "plan")
            .map_err(SafetyError::from_workspace)?;
    }
    Ok(())
}

fn describe_workspace_slot_bounded(
    eval: &crate::chain::WorkspaceEvaluator<'_>,
    ws: &crate::model::WorkspacePolicy,
    cam: &crate::model::CameraMount,
    tcp_offset: [f64; 3],
    slot: usize,
    clr: f64,
) -> crate::safety::WorkspaceSupportViolation {
    use crate::kinematics as k;
    use crate::safety::{BoundedText, WorkspaceSupportViolation as W};
    use core::fmt::Write as _;

    let depth = -clr;
    if slot >= eval.self_collision_slot_start {
        let pi = slot - eval.self_collision_slot_start;
        let (id_a, id_b) = ws.self_collision_pairs.get(pi).map_or(("?", "?"), |[a, b]| (a.as_str(), b.as_str()));
        let (mut cid, mut gid, mut limit) = (BoundedText::empty(), BoundedText::empty(), BoundedText::empty());
        let _ = write!(&mut cid, "self_collision:{id_a}:{id_b}");
        let _ = write!(&mut gid, "capsule:{id_a}");
        let _ = write!(&mut limit, "self_collision '{id_a}' vs '{id_b}'");
        let ia = ws.link_capsules.iter().position(|c| c.id == id_a);
        let ib = ws.link_capsules.iter().position(|c| c.id == id_b);
        let point = match (ia, ib) { (Some(a), Some(b)) if a != b => eval.num_end_pts + a, _ => 0 };
        let bound = f64::from(ws.self_collision_min_clearance_m);
        return W::Constraint { limit, point, cid, gid, depth, value: bound + clr, bound };
    }
    if eval.has_reach && slot >= eval.num_geoms * eval.slots_per_geom {
        let reach_idx = slot - eval.num_geoms * eval.slots_per_geom;
        let bound = f64::from(ws.max_reach_m.unwrap_or(0.0));
        let (point, gid) = if reach_idx == 0 {
            (0, BoundedText::new("end_origin"))
        } else {
            let tcp_f32 = tcp_offset.map(|v| v as f32);
            let pt = ws.check_points_end_m.iter()
                .position(|c| (c[0] - tcp_f32[0]).abs() <= 1e-6 && (c[1] - tcp_f32[1]).abs() <= 1e-6 && (c[2] - tcp_f32[2]).abs() <= 1e-6)
                .map_or(0, |i| i + 1);
            (pt, BoundedText::new("tcp_origin"))
        };
        return W::Constraint { limit: BoundedText::new("reach"), point, cid: BoundedText::new("max_reach"), gid, depth, value: bound - clr, bound };
    }
    let (gi, ci) = (slot / eval.slots_per_geom, slot % eval.slots_per_geom);
    let mut gid = BoundedText::empty();
    if gi == 0 {
        gid.push_str("end_origin");
    } else if gi < eval.num_end_pts {
        if cam.offset_m.is_some() && gi + 1 == eval.num_end_pts { gid.push_str("camera_mount"); }
        else { let _ = write!(&mut gid, "end_point:{gi}"); }
    } else {
        let _ = write!(&mut gid, "capsule:{}", ws.link_capsules[gi - eval.num_end_pts].id);
    }
    let (mut limit, mut cid) = (BoundedText::empty(), BoundedText::empty());
    let (value, bound) = if ci == 0 {
        limit.push_str("floor"); cid.push_str("floor_z");
        let b = f64::from(ws.floor_z_m); (b + clr, b)
    } else if ci <= 3 {
        let ax = ci - 1;
        limit.push_str(k::BOX_LIMITS[ax][0]); cid.push_str(k::BOX_CONSTRAINT_IDS[ax][0]);
        let b = f64::from(ws.box_min_m[ax]); (b + clr, b)
    } else if ci <= 6 {
        let ax = ci - 4;
        limit.push_str(k::BOX_LIMITS[ax][1]); cid.push_str(k::BOX_CONSTRAINT_IDS[ax][1]);
        let b = f64::from(ws.box_max_m[ax]); (b - clr, b)
    } else {
        let name = ws.keepout_boxes.get(ci - 7).map_or("?", |b| b.name.as_str());
        let _ = write!(&mut limit, "keepout '{name}'"); let _ = write!(&mut cid, "keepout:{name}");
        (depth, 0.0)
    };
    W::Constraint { limit, point: gi, cid, gid, depth, value, bound }
}

impl SupervisorInner {
    #[allow(clippy::too_many_arguments)]
    pub fn keepout_sweep_with_tcp_measured(
        model: &crate::chain::ArmModel,
        ws: &crate::model::WorkspacePolicy,
        cam: &crate::model::CameraMount,
        tcp_offset: [f64; 3],
        escape_tol: f64,
        measured: Option<&[f32]>,
        at: &dyn Fn(f64) -> Vec<f32>,
        travel: &dyn Fn(f64, f64) -> Vec<f64>,
        knots: &[f64],
        budget: usize,
    ) -> Result<Vec<Vec<f32>>, SafetyError> {
        keepout_sweep_with_tcp_measured(model, ws, cam, tcp_offset, escape_tol, measured, at, travel, knots, budget)
    }

    #[allow(clippy::too_many_arguments)]
    pub fn check_path_with_tcp_measured<'a>(
        model: &crate::chain::ArmModel,
        ws: &crate::model::WorkspacePolicy,
        cam: &crate::model::CameraMount,
        tcp_offset: [f64; 3],
        escape_tol: f64,
        measured: Option<&[f32]>,
        start: &[f32],
        samples: impl IntoIterator<Item = &'a [f32]>,
    ) -> Result<(), SafetyError> {
        check_path_with_tcp_measured(model, ws, cam, tcp_offset, escape_tol, measured, start, samples)
    }

    /// Clears `self.workspace_violation`, recycling its preallocated detail into
    /// `self.workspace_violation_spare` (0 alloc / 0 free on `armd-control`).
    pub fn clear_workspace_violation_rt(&mut self) {
        if let Some(old) = self.workspace_violation.take() {
            if self.workspace_violation_spare.is_none() {
                self.workspace_violation_spare = Some(old);
            }
        }
    }

    /// Clears `self.servo_ws_min_depths`, recycling any heap `SlotBuf` back into
    /// `self.ws_eval_bufs.min_depths_spare` so `armd-control` never frees it.
    /// Retains running minima while still torqued, operational, and outside the workspace at
    /// the same `workspace_revision` so settled/idle stream frames cannot ratchet deeper.
    pub(crate) fn clear_servo_ws_min_depths_rt(&mut self) {
        if let Some((rev, ref buf)) = self.servo_ws_min_depths {
            let arm_motion = self.active_trajectory.as_ref().is_some_and(|t| {
                !matches!(t.reference, crate::motion::Reference::Gripper(_))
            });
            if rev == self.workspace_revision
                && self.is_torqued
                && self.safety.state().is_operational()
                && !arm_motion
                && crate::chain::WorkspaceEvaluator::any_outside(buf.as_slice())
            {
                return;
            }
        }
        self.force_clear_servo_ws_min_depths_rt();
    }

    /// Unconditionally clears and recycles `self.servo_ws_min_depths` (used when installing a
    /// planned motion, disabling/faulting, or resetting workspace configuration).
    pub(crate) fn force_clear_servo_ws_min_depths_rt(&mut self) {
        if let Some((_, old)) = self.servo_ws_min_depths.take() {
            if old.is_heap() {
                if self.ws_eval_bufs.min_depths_spare.is_none() {
                    self.ws_eval_bufs.min_depths_spare = Some(old);
                } else {
                    self.push_retired_stop(crate::rt_core::urgent::RetiredStopPayloads {
                        servo_ws_min_depths: Some((0, old)),
                        ..Default::default()
                    });
                }
            }
        }
    }

    /// Pre-sizes `ws_eval_bufs` (including overflow slot and capsule buffers for workspaces
    /// exceeding 256 slots) off the RT thread so RT checks allocate zero heap memory.
    pub fn prepare_workspace_eval_bufs(&mut self) {
        let mut bufs = std::mem::take(&mut self.ws_eval_bufs);
        if let Some((ws, cam)) = self.workspace_policy() {
            bufs.prepare_for(&self.config.arm_model, ws, cam, self.config.tcp_offset);
        }
        self.tracking.install_ws_depth_spares_from_eval_bufs(&mut bufs, None);
        self.ws_eval_bufs = bufs;
        // Single-buffer workspace spare invariant: at most one heap `SlotBuf` exists across
        // `(self.servo_ws_min_depths, self.ws_eval_bufs.min_depths_spare)`, so
        // `clear_servo_ws_min_depths_rt` always recycles directly into `min_depths_spare` with
        // 0 frees even when retirement queues are full.
        if self.servo_ws_min_depths.as_ref().is_some_and(|(_, b)| b.is_heap()) {
            self.ws_eval_bufs.min_depths_spare = None;
        }
        if self.workspace_violation.is_none() && self.workspace_violation_spare.is_none() {
            self.workspace_violation_spare =
                Some(crate::kinematics::WorkspaceViolationDetail::preallocated_rt());
        }
    }

    /// Joint limits used for planning and servo stepping: `config.limits` with
    /// `max_velocity` capped at `config.planned_max_velocity` for every joint
    /// except the 7-DOF gripper (index 6).
    /// `speed_cap.joint_velocity_scale` then scales the result (same joints).
    pub fn planning_limits(&self) -> Vec<crate::model::JointLimit> {
        let mut limits = self.config.limits.clone();
        let scale = self.config.speed_cap.and_then(|c| c.joint_velocity_scale).unwrap_or(1.0);
        let gripper_idx = self.config.gripper_index();
        for (i, limit) in limits.iter_mut().enumerate() {
            if Some(i) != gripper_idx {
                if let Some(cap) = self.config.planned_max_velocity {
                    limit.max_velocity = limit.max_velocity.min(cap);
                }
                limit.max_velocity *= scale;
            }
        }
        limits
    }

    /// Conservative keep-out check of the joint path `at(t)` over `knots` (ascending).
    ///
    /// Every check point (end_link origin, `check_points_end_m`, camera offset) moves along
    /// an interval `[t0, t1]` at most `L = sum_j travel_j * reach_j`, with `travel` an upper
    /// bound on each joint's travel and `reach_j` the largest distance from joint `j`'s axis to
    /// a check point ([`ArmModel::joint_reach`](crate::chain::ArmModel::joint_reach)). A
    /// point at Euclidean distances `d0`, `d1` from a box at the interval ends cannot enter
    /// it when `d0 + d1 > L`. An interval where that fails for some point and box is split at
    /// its midpoint; one with `L` at most [`WORKSPACE_EPS`](crate::kinematics::WORKSPACE_EPS)
    /// is accepted (the depth rule's tolerance). A sample with a point deeper than the
    /// tolerance in a box is WORKSPACE_VIOLATION; so is needing more than `budget` added
    /// evaluations. Boxes a point is already inside at `at(knots[0])` are left to the
    /// sample rule (a move out of a box is allowed). Returns the evaluated samples in order.
    pub fn keepout_sweep(
        &self,
        ws: &crate::model::WorkspacePolicy,
        cam: &crate::model::CameraMount,
        at: &dyn Fn(f64) -> Vec<f32>,
        travel: &dyn Fn(f64, f64) -> Vec<f64>,
        knots: &[f64],
        budget: usize,
    ) -> Result<Vec<Vec<f32>>, SafetyError> {
        keepout_sweep_with_tcp_tol(&self.config.arm_model, ws, cam, self.config.tcp_offset, crate::kinematics::WORKSPACE_EPS, at, travel, knots, budget)
    }

    pub fn keepout_sweep_with_tol(
        &self,
        ws: &crate::model::WorkspacePolicy,
        cam: &crate::model::CameraMount,
        escape_tol: f64,
        at: &dyn Fn(f64) -> Vec<f32>,
        travel: &dyn Fn(f64, f64) -> Vec<f64>,
        knots: &[f64],
        budget: usize,
    ) -> Result<Vec<Vec<f32>>, SafetyError> {
        keepout_sweep_with_tcp_tol(&self.config.arm_model, ws, cam, self.config.tcp_offset, escape_tol, at, travel, knots, budget)
    }

    /// Workspace policy and camera mount when the Cartesian check applies
    /// (workspace and kinematics configured, 7-DOF arm).
    pub fn workspace_policy(&self) -> Option<(&crate::model::WorkspacePolicy, &crate::model::CameraMount)> {
        match (&self.config.workspace, self.config.kinematics) {
            (Some(ws), Some(_)) if self.config.dof >= self.config.arm_dof() => Some((ws, &self.config.camera_mount)),
            _ => None,
        }
    }

    /// Reject a move whose samples leave the workspace further than `start` already is.
    pub fn check_workspace_path<'a>(
        &self,
        start: &[f32],
        samples: impl IntoIterator<Item = &'a [f32]>,
    ) -> Result<(), SafetyError> {
        let Some((ws, cam)) = self.workspace_policy() else { return Ok(()); };
        self.check_path_against(ws, cam, start, samples)
    }

    /// [`check_workspace_path`](Self::check_workspace_path) with explicit escape jitter tolerance.
    pub fn check_workspace_path_with_tol<'a>(
        &self,
        start: &[f32],
        samples: impl IntoIterator<Item = &'a [f32]>,
        escape_tol: f64,
    ) -> Result<(), SafetyError> {
        let Some((ws, cam)) = self.workspace_policy() else { return Ok(()); };
        check_path_with_tcp_measured(&self.config.arm_model, ws, cam, self.config.tcp_offset, escape_tol, None, start, samples)
    }

    /// [`check_workspace_path`](Self::check_workspace_path) evaluating from `measured` baseline
    /// when it differs from `start`.
    pub fn check_workspace_path_from_measured<'a>(
        &self,
        start: &[f32],
        measured: Option<&[f32]>,
        samples: impl IntoIterator<Item = &'a [f32]>,
    ) -> Result<(), SafetyError> {
        let Some((ws, cam)) = self.workspace_policy() else { return Ok(()); };
        check_path_with_tcp_measured(
            &self.config.arm_model, ws, cam, self.config.tcp_offset, crate::kinematics::WORKSPACE_EPS, measured, start, samples,
        )
    }

    /// [`check_workspace_path`](Self::check_workspace_path) against the keep-out boxes only
    /// (no floor / box / reach): the part of the rule park may not bypass.
    pub fn check_keepout_path<'a>(
        &self,
        start: &[f32],
        samples: impl IntoIterator<Item = &'a [f32]>,
    ) -> Result<(), SafetyError> {
        let Some((ws, cam)) = self.workspace_policy() else { return Ok(()); };
        if ws.keepout_boxes.is_empty() { return Ok(()); }
        let only = ws.keepout_only();
        self.check_path_against(&only, cam, start, samples)
    }

    fn check_path_against<'a>(
        &self,
        ws: &crate::model::WorkspacePolicy,
        cam: &crate::model::CameraMount,
        start: &[f32],
        samples: impl IntoIterator<Item = &'a [f32]>,
    ) -> Result<(), SafetyError> {
        check_path_with_tcp_tol(&self.config.arm_model, ws, cam, self.config.tcp_offset, crate::kinematics::WORKSPACE_EPS, start, samples)
    }

    /// The planner's 21 verification points of a trajectory.
    pub fn trajectory_samples(traj: &MultiJointQuinticTrajectory) -> Vec<Vec<f32>> {
        Self::uniform_samples(traj, 20)
    }

    fn uniform_samples(traj: &MultiJointQuinticTrajectory, n: usize) -> Vec<Vec<f32>> {
        (0..=n).map(|k| traj.sample(traj.duration * k as f64 / n as f64).positions).collect()
    }

    /// Workspace verification points of a joint-space trajectory: the 21 planner samples
    /// verified against the full workspace (floor, outer box, reach, keep-out boxes,
    /// link capsules) via the conservative sweep check
    /// ([`keepout_sweep`](Self::keepout_sweep), at most [`MAX_KEEPOUT_EVALS_PER_PLAN`] added
    /// FK evaluations; a path that needs more, or that violates, is WORKSPACE_VIOLATION).
    pub fn workspace_samples(&self, traj: &MultiJointQuinticTrajectory) -> Result<Vec<Vec<f32>>, SafetyError> {
        self.workspace_samples_with_budget(traj, MAX_KEEPOUT_EVALS_PER_PLAN)
    }

    /// [`workspace_samples`](Self::workspace_samples) with `budget` added FK evaluations.
    pub fn workspace_samples_with_budget(&self, traj: &MultiJointQuinticTrajectory, budget: usize) -> Result<Vec<Vec<f32>>, SafetyError> {
        self.workspace_samples_with_budget_tol(traj, budget, crate::kinematics::WORKSPACE_EPS)
    }

    /// [`workspace_samples`](Self::workspace_samples) with explicit escape jitter tolerance.
    pub fn workspace_samples_with_tol(&self, traj: &MultiJointQuinticTrajectory, escape_tol: f64) -> Result<Vec<Vec<f32>>, SafetyError> {
        self.workspace_samples_with_budget_tol(traj, MAX_KEEPOUT_EVALS_PER_PLAN, escape_tol)
    }

    /// [`workspace_samples`](Self::workspace_samples) evaluating from `measured` baseline when it
    /// differs from `traj(0)`.
    pub fn workspace_samples_from_measured(
        &self,
        traj: &MultiJointQuinticTrajectory,
        measured: Option<&[f32]>,
    ) -> Result<Vec<Vec<f32>>, SafetyError> {
        let Some((ws, cam)) = self.workspace_policy()
            else { return Ok(Self::trajectory_samples(traj)); };
        let knots: Vec<f64> = (0..=20).map(|k| traj.duration * k as f64 / 20.0).collect();
        let at = |t: f64| traj.sample(t).positions;
        let travel = |t0: f64, t1: f64| traj.travel_bound(t0, t1);
        keepout_sweep_with_tcp_measured(
            &self.config.arm_model, ws, cam, self.config.tcp_offset, crate::kinematics::WORKSPACE_EPS,
            measured, &at, &travel, &knots, MAX_KEEPOUT_EVALS_PER_PLAN,
        )
    }

    fn workspace_samples_with_budget_tol(&self, traj: &MultiJointQuinticTrajectory, budget: usize, escape_tol: f64) -> Result<Vec<Vec<f32>>, SafetyError> {
        let Some((ws, cam)) = self.workspace_policy()
            else { return Ok(Self::trajectory_samples(traj)); };
        let knots: Vec<f64> = (0..=20).map(|k| traj.duration * k as f64 / 20.0).collect();
        let at = |t: f64| traj.sample(t).positions;
        let travel = |t0: f64, t1: f64| traj.travel_bound(t0, t1);
        self.keepout_sweep_with_tol(ws, cam, escape_tol, &at, &travel, &knots, budget)
    }

    /// Keepout-only sweep samples.
    pub fn keepout_samples(&self, traj: &MultiJointQuinticTrajectory) -> Result<Vec<Vec<f32>>, SafetyError> {
        let Some((ws, cam)) = self.workspace_policy().filter(|(ws, _)| !ws.keepout_boxes.is_empty())
            else { return Ok(Self::trajectory_samples(traj)); };
        let only = ws.keepout_only();
        let knots: Vec<f64> = (0..=20).map(|k| traj.duration * k as f64 / 20.0).collect();
        let at = |t: f64| traj.sample(t).positions;
        let travel = |t0: f64, t1: f64| traj.travel_bound(t0, t1);
        self.keepout_sweep(&only, cam, &at, &travel, &knots, MAX_KEEPOUT_EVALS_PER_PLAN)
    }

    /// Evaluate the active workspace policy (including `tcp_offset`, keep-outs, floor,
    /// max reach, and link capsules) at `self.measured_positions`.
    pub(super) fn measured_workspace_violation(&self, phase: &str) -> Option<String> {
        let (ws, cam) = self.workspace_policy()?;
        let eval = crate::chain::WorkspaceEvaluator::new_with_tcp(
            &self.config.arm_model,
            ws,
            cam,
            self.config.tcp_offset,
        );
        let mut buf = eval.alloc_buf();
        match eval.eval_into(&self.measured_positions, &mut buf, phase) {
            Err(f) => Some(format!(
                "measured pose workspace evaluation failed ({}): {}",
                f.limit,
                f.detail.reason.as_deref().unwrap_or("invalid joint positions")
            )),
            Ok(()) => {
                if let Some((slot, &clr)) = buf
                    .as_slice()
                    .iter()
                    .enumerate()
                    .find(|(_, clr)| !clr.is_finite() || **clr < -crate::kinematics::WORKSPACE_EPS)
                {
                    let f = eval.describe_failure_with_context(slot, clr, -clr, 0.0, phase, true);
                    let cid = f.detail.constraint_id.as_deref().unwrap_or(f.limit.as_str());
                    let detail = f.detail.reason.as_deref().unwrap_or(f.limit.as_str());
                    Some(format!("measured pose violates workspace ({cid}, point {}): {detail}", f.point))
                } else {
                    None
                }
            }
        }
    }

    /// Zero-allocation RT quintic sweep returning static error codes on both acceptance and
    /// refusal; fails closed without allocating on oversized/undersized inputs on RT.
    pub(crate) fn check_workspace_quintic_inline_rt(
        &mut self,
        traj: &MultiJointQuinticTrajectory,
        check_measured: bool,
    ) -> Result<(), &'static str> {
        if self.workspace_policy().is_none() {
            return Ok(());
        }
        if traj.polynomials.len() > crate::trajectory::INLINE_POLYS
            || self.config.arm_model.arm_dof() > crate::chain::MAX_CHAIN_JOINTS
        {
            if self.rt_executor_owned {
                return Err("WORKSPACE_VIOLATION");
            }
            let meas = check_measured.then_some(self.measured_positions.as_slice());
            let samples = self.workspace_samples_from_measured(traj, meas).map_err(|_| "WORKSPACE_VIOLATION")?;
            return self
                .check_workspace_path_from_measured(&self.commanded_positions, meas, samples.iter().map(Vec::as_slice))
                .map_err(|_| "WORKSPACE_VIOLATION");
        }
        let mut bufs = std::mem::take(&mut self.ws_eval_bufs);
        let result = (|| {
            let Some((ws, cam)) = self.workspace_policy() else { return Ok(()); };
            if !bufs.can_eval_rt(&self.config.arm_model, ws, cam, self.config.tcp_offset, false) {
                if self.rt_executor_owned {
                    return Err("WORKSPACE_VIOLATION");
                }
                bufs.prepare_for(&self.config.arm_model, ws, cam, self.config.tcp_offset);
            }
            let eval = crate::chain::WorkspaceEvaluator::new_with_tcp_in(
                &self.config.arm_model, ws, cam, self.config.tcp_offset, &mut bufs,
            )
            .with_escape_tol(crate::kinematics::WORKSPACE_EPS);
            let meas = check_measured.then_some(self.measured_positions.as_slice());
            let checked = eval
                .check_quintic_inline_rt(meas, traj, MAX_KEEPOUT_EVALS_PER_PLAN, "sweep", &mut bufs)
                .map_err(|_| "WORKSPACE_VIOLATION");
            eval.recycle(&mut bufs);
            checked
        })();
        self.ws_eval_bufs = bufs;
        result
    }

    pub(super) fn measured_workspace_violation_bounded(
        &self,
        bufs: &mut crate::chain::EvalBufs,
    ) -> Option<crate::safety::WorkspaceSupportViolation> {
        self.workspace_violation_for_positions_bounded(&self.measured_positions, bufs)
    }

    pub(super) fn workspace_violation_for_positions_bounded(
        &self,
        positions: &[f32],
        bufs: &mut crate::chain::EvalBufs,
    ) -> Option<crate::safety::WorkspaceSupportViolation> {
        use crate::safety::{BoundedText, WorkspaceSupportViolation};
        let (ws, cam) = self.workspace_policy()?;
        if self.config.arm_model.frames_f32(positions).is_none() {
            return Some(WorkspaceSupportViolation::EvalFailed {
                limit: BoundedText::new("invalid"),
                reason: BoundedText::new("non-finite or incomplete joint positions"),
            });
        }
        if ws.self_collision_enabled && !ws.link_capsules.is_empty() && !ws.self_collision_pairs.is_empty()
            && (!ws.self_collision_min_clearance_m.is_finite() || ws.self_collision_min_clearance_m < 0.0)
        {
            return Some(WorkspaceSupportViolation::EvalFailed {
                limit: BoundedText::new("invalid"),
                reason: BoundedText::new("invalid self_collision_min_clearance_m"),
            });
        }
        if let Some(cap) = ws.link_capsules.iter().find(|c| self.config.arm_model.resolve_frame(&c.frame).is_none()) {
            let mut reason = BoundedText::<128>::empty();
            let _ = core::fmt::write(&mut reason, format_args!("unknown frame '{}' for capsule '{}'", cap.frame, cap.id));
            return Some(WorkspaceSupportViolation::EvalFailed {
                limit: BoundedText::new("invalid"),
                reason,
            });
        }
        if self.rt_executor_owned
            && !bufs.can_eval_rt(&self.config.arm_model, ws, cam, self.config.tcp_offset, false)
        {
            return Some(WorkspaceSupportViolation::EvalFailed {
                limit: BoundedText::new("invalid"),
                reason: BoundedText::new("workspace scratch undersized"),
            });
        }
        let eval = crate::chain::WorkspaceEvaluator::new_with_tcp_in(
            &self.config.arm_model, ws, cam, self.config.tcp_offset, bufs,
        );
        let mut buf = eval.alloc_buf_in(bufs);
        let violation = match eval.eval_into_in(positions, &mut buf, &mut bufs.world_caps, "clear_fault") {
            Err(f) => Some(WorkspaceSupportViolation::EvalFailed {
                limit: BoundedText::new(&f.limit),
                reason: BoundedText::new(f.detail.reason.as_deref().unwrap_or("invalid joint positions")),
            }),
            Ok(()) => buf.as_slice().iter().enumerate()
                .find(|(_, clr)| !clr.is_finite() || **clr < -crate::kinematics::WORKSPACE_EPS)
                .map(|(slot, &clr)| describe_workspace_slot_bounded(&eval, ws, cam, self.config.tcp_offset, slot, clr)),
        };
        crate::chain::WorkspaceEvaluator::recycle_buf(buf, bufs);
        eval.recycle(bufs);
        violation
    }
}
