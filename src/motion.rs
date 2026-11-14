//! High-level actions: Cartesian path planning (pure, runs outside the supervisor lock) and
//! the references the supervisor executes for them (spec §2).
//!
//! - [`plan_cartesian`] turns a goal pose into a joint path: straight line in position,
//!   slerp in rotation, one DLS IK solve per sample, single quintic time scaling.
//! - [`PathTrajectory`] executes such a path with lag-aware time scaling.
//! - [`GripperMotion`] runs open / close / move of the 7-DOF gripper with grasp detection.
use std::collections::BTreeMap;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use thiserror::Error;

use crate::kinematics::{self, Pose};
use crate::model::{CameraMount, GripperPolicy, HoldingMode, JointLimit, WorkspacePolicy};
use crate::trajectory::MultiJointQuinticTrajectory;

#[derive(Debug, Clone, PartialEq, Error)]
pub enum MotionError {
    #[error("IK unreachable at s={s:.3} (position error {ep:.4} m, rotation error {er:.4} rad)")]
    IkUnreachable { s: f64, ep: f64, er: f64 },
    #[error("IK branch flip at s={s:.3}: joint {joint} jumps {dq:.3} rad between samples (limit 0.10)")]
    IkBranchFlip { s: f64, joint: usize, dq: f64 },
    #[error("Path leaves the safe workspace at s={s:.3}: check point {point} {limit} {value:.3} m beyond bound {bound:.3} m")]
    WorkspaceViolation { s: f64, point: usize, limit: String, value: f64, bound: f64 },
    #[error("Motion busy: {0}")]
    MotionBusy(String),
    #[error("Plan is stale: {0}")]
    PlanStale(String),
    #[error("{0}")]
    InvalidCommand(String),
    #[error("{0}")]
    Unsupported(String),
    /// Power budget planning (`src/power_plan.rs`): even the slowest allowed speed exceeds it.
    #[error("Power budget: {0}")]
    PowerBudget(String),
}

impl MotionError {
    pub fn code(&self) -> &'static str {
        match self {
            Self::IkUnreachable { .. } => "IK_UNREACHABLE",
            Self::IkBranchFlip { .. } => "IK_BRANCH_FLIP",
            Self::WorkspaceViolation { .. } => "WORKSPACE_VIOLATION",
            Self::MotionBusy(_) => "MOTION_BUSY",
            Self::PlanStale(_) => "PLAN_STALE",
            Self::InvalidCommand(_) => "INVALID_COMMAND",
            Self::Unsupported(_) => "UNSUPPORTED",
            Self::PowerBudget(_) => "POWER_BUDGET",
        }
    }
}

// ---- speed ----

pub const DEFAULT_LINEAR_MPS: f64 = 0.05;
pub const MAX_LINEAR_MPS: f64 = 0.15;
pub const DEFAULT_ANGULAR_RPS: f64 = 0.5;
pub const MAX_ANGULAR_RPS: f64 = 1.0;
/// Joint acceleration cap for action paths (rad/s²), applied as min(limit, this).
pub const ACTION_MAX_ACCELERATION: f64 = 3.0;
/// Largest joint change between consecutive path samples before the path counts as a branch flip.
pub const MAX_SAMPLE_JUMP_RAD: f64 = 0.10;
pub const SAMPLE_SPACING_M: f64 = 0.005;
pub const SAMPLE_SPACING_RAD: f64 = 2.0 * std::f64::consts::PI / 180.0;
pub const MAX_SAMPLES: usize = 400;
/// IK search limits are the joint limits shrunk by this margin (cockpit `CFG.limitMargin`).
pub const IK_LIMIT_MARGIN: f64 = 0.02;
/// Auto-level threshold (cockpit `CFG.view.levelCos`).
pub fn level_cos() -> f64 { (15.0f64).to_radians().cos() }

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Speed {
    pub linear: f64,
    pub angular: f64,
}

impl Default for Speed {
    fn default() -> Self { Self { linear: DEFAULT_LINEAR_MPS, angular: DEFAULT_ANGULAR_RPS } }
}

// ---- planning ----

/// Snapshot of the supervisor state a plan is made from.
#[derive(Debug, Clone)]
pub struct PlanInput {
    pub q0: [f64; 6],
    pub j7: f32,
    /// `planning_limits()` (speed-capped) for all 7 joints.
    pub limits: Vec<JointLimit>,
    /// Profile speed cap; Cartesian speeds are min(request, cap).
    pub speed_cap: Option<crate::model::SpeedCap>,
    pub ws: Option<WorkspacePolicy>,
    pub cam: CameraMount,
    /// Kinematic model of the profile (built-in or URDF chain).
    pub arm: crate::chain::ArmModel,
    pub epoch: u64,
    /// `SupervisorInner::workspace_revision` the plan was checked against.
    pub workspace_revision: u64,
    /// Power budget whose `planning` section time-scales the path; None = none.
    pub power: Option<crate::power::PowerBudget>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct JointPath {
    pub dof: usize,
    /// Samples, the first one is the start.
    pub q: Vec<[f32; 7]>,
    /// Normalised path parameter of each sample (0..=1).
    pub s: Vec<f64>,
    pub duration: f64,
    /// Power-budget time scale applied to `duration` (1 = none, or no budget).
    pub power_scale: f64,
}

impl JointPath {
    pub fn last(&self) -> [f32; 7] { *self.q.last().expect("path has samples") }

    pub fn last_slice(&self) -> &[f32] { &self.q.last().expect("path has samples")[..self.dof] }

    /// Joint positions at path time `t` (quintic time law of [`PathTrajectory`]).
    pub fn position_at_time(&self, t: f64) -> [f32; 7] {
        let u = (t / self.duration).clamp(0.0, 1.0);
        let s = u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
        if self.s.len() < 2 { return self.q[0]; }
        let b = self.s.partition_point(|x| *x <= s).clamp(1, self.s.len() - 1);
        let a = b - 1;
        let f = ((s - self.s[a]) / (self.s[b] - self.s[a])).clamp(0.0, 1.0) as f32;
        std::array::from_fn(|i| self.q[a][i] + (self.q[b][i] - self.q[a][i]) * f)
    }

    /// Fit the path into the power budget's planning limit (`src/power_plan.rs`): the
    /// duration grows by 1 / scale.
    pub fn fit_power(&mut self, budget: &crate::power::PowerBudget, arm: &crate::chain::ArmModel) -> Result<crate::power_plan::PlanScale, MotionError> {
        let p = budget.planning;
        let n = crate::power_plan::Samples::count(&p, self.duration);
        let arm_dof = arm.arm_dof();
        let samples = crate::power_plan::Samples::from_positions(self.duration, n, arm_dof, |t| {
            let q = self.position_at_time(t);
            std::array::from_fn(|i| if i < arm_dof { f64::from(q[i]) } else { 0.0 })
        });
        let fit = crate::power_plan::plan_scale(budget, arm, &samples)?;
        self.duration /= fit.scale;
        self.power_scale = fit.scale;
        Ok(fit)
    }
}

/// Plan a straight-line (position) / slerp (rotation) path from `fk(q0)` to `goal`.
pub fn plan_cartesian(input: &PlanInput, goal: Pose, speed: Speed) -> Result<JointPath, MotionError> {
    plan_cartesian_ex(input, goal, speed, false)
}

/// Plan a Cartesian path from `fk(q0)` to `goal`, with optional `yaw_free` for 5-DOF arms.
pub fn plan_cartesian_ex(input: &PlanInput, goal: Pose, speed: Speed, yaw_free: bool) -> Result<JointPath, MotionError> {
    let arm_dof = input.arm.arm_dof();
    if input.limits.len() < arm_dof + 1 {
        return Err(MotionError::Unsupported(format!("Cartesian actions need at least {} joints", arm_dof + 1)));
    }
    if !goal.p.iter().chain(goal.r.iter().flatten()).all(|v| v.is_finite()) {
        return Err(MotionError::InvalidCommand("goal pose must be finite".into()));
    }
    if !(speed.linear > 0.0 && speed.linear <= MAX_LINEAR_MPS + 1e-12 && speed.angular > 0.0 && speed.angular <= MAX_ANGULAR_RPS + 1e-12) {
        return Err(MotionError::InvalidCommand(format!(
            "speed must be 0 < linear <= {MAX_LINEAR_MPS} m/s and 0 < angular <= {MAX_ANGULAR_RPS} rad/s")));
    }
    let speed = match input.speed_cap {
        Some(cap) => Speed {
            linear: cap.linear_mps.map_or(speed.linear, |c| speed.linear.min(f64::from(c))),
            angular: cap.angular_rps.map_or(speed.angular, |c| speed.angular.min(f64::from(c))),
        },
        None => speed,
    };
    let start = input.arm.fk(&input.q0);
    let dp = kinematics::sub(&goal.p, &start.p);
    let length = kinematics::norm(&dp);
    let w_rel = kinematics::log_v(&kinematics::mul_t(&goal.r, &start.r));
    let theta = kinematics::norm(&w_rel);
    let raw = ((length / SAMPLE_SPACING_M).ceil()).max((theta / SAMPLE_SPACING_RAD).ceil()).max(1.0);
    if raw > MAX_SAMPLES as f64 {
        return Err(MotionError::InvalidCommand(format!(
            "path too long: {length:.3} m / {:.1} deg needs {raw} samples (max {MAX_SAMPLES})", theta.to_degrees())));
    }
    let n = raw as usize;

    let lo: [f64; 6] = std::array::from_fn(|i| {
        if i < input.limits.len() {
            f64::from(input.limits[i].min_position) + IK_LIMIT_MARGIN
        } else {
            -std::f64::consts::PI
        }
    });
    let hi: [f64; 6] = std::array::from_fn(|i| {
        if i < input.limits.len() {
            f64::from(input.limits[i].max_position) - IK_LIMIT_MARGIN
        } else {
            std::f64::consts::PI
        }
    });
    let start_q32: [f32; 7] = std::array::from_fn(|i| if i < arm_dof { input.q0[i] as f32 } else if i == arm_dof { input.j7 } else { 0.0 });
    let v_start = input.ws.as_ref().map(|ws| input.arm.workspace_violation(&start_q32, ws, &input.cam));

    let mut q_samples = Vec::with_capacity(n + 1);
    let mut s_samples = Vec::with_capacity(n + 1);
    q_samples.push(start_q32);
    s_samples.push(0.0);
    let mut prev = input.q0;
    for k in 1..=n {
        let s = k as f64 / n as f64;
        let p = [start.p[0] + dp[0] * s, start.p[1] + dp[1] * s, start.p[2] + dp[2] * s];
        let r = if k == n { goal.r } else {
            kinematics::mat_mul(&kinematics::exp_r(&[w_rel[0] * s, w_rel[1] * s, w_rel[2] * s]), &start.r)
        };
        let sol = input.arm.solve_dls_ex(&Pose::new(r, p), &prev, &lo, &hi, kinematics::PAR.w, yaw_free);
        if !sol.ok {
            return Err(MotionError::IkUnreachable { s, ep: sol.ep, er: sol.er });
        }
        if let Some((joint, dq)) = (0..arm_dof).map(|i| (i, (sol.q[i] - prev[i]).abs())).max_by(|a, b| a.1.total_cmp(&b.1)) {
            if dq > MAX_SAMPLE_JUMP_RAD {
                return Err(MotionError::IkBranchFlip { s, joint: joint + 1, dq });
            }
        }
        let q32: [f32; 7] = std::array::from_fn(|i| if i < arm_dof { sol.q[i] as f32 } else if i == arm_dof { input.j7 } else { 0.0 });
        if let (Some(ws), Some(v0)) = (input.ws.as_ref(), v_start.as_ref()) {
            let v = input.arm.workspace_violation(&q32, ws, &input.cam);
            if kinematics::workspace_rejects(&v, v0) {
                let w = v.worst.expect("violation has a worst point");
                return Err(MotionError::WorkspaceViolation { s, point: w.point, limit: w.label(ws), value: w.value, bound: w.bound });
            }
        }
        q_samples.push(q32);
        s_samples.push(s);
        prev = sol.q;
    }

    // Single quintic s(t): peak s' = 1.875 / T, peak s'' = 5.7735 / T².
    let mut duration: f64 = 0.0;
    for seg in 1..q_samples.len() {
        let ds = s_samples[seg] - s_samples[seg - 1];
        for j in 0..arm_dof {
            let slope = f64::from((q_samples[seg][j] - q_samples[seg - 1][j]).abs()) / ds;
            let vcap = f64::from(input.limits[j].max_velocity);
            let acap = f64::from(input.limits[j].max_acceleration).min(ACTION_MAX_ACCELERATION);
            if vcap > 0.0 { duration = duration.max(1.875 * slope / vcap); }
            if acap > 0.0 { duration = duration.max((5.7735 * slope / acap).sqrt()); }
        }
    }
    duration = duration.max(1.875 * length / speed.linear).max(1.875 * theta / speed.angular);
    let duration = (duration * 1.1).max(0.2);
    let dof = input.limits.len();
    let mut path = JointPath { dof, q: q_samples, s: s_samples, duration, power_scale: 1.0 };
    if let Some(b) = input.power.as_ref().filter(|b| b.enabled && b.planning.enabled) {
        let t0 = std::time::Instant::now();
        let fit = path.fit_power(b, &input.arm)?;
        tracing::debug!("Power planning: scale {:.3}, peak {:.2} A (full speed {:.2} A, limit {:.2} A), {} samples, {} µs",
            fit.scale, fit.peak_a, fit.peak_full_a, fit.limit_a, fit.samples, t0.elapsed().as_micros());
    }
    Ok(path)
}

// ---- goal requests ----

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum RelFrame { Base, Tool, Camera }

#[derive(Debug, Clone, PartialEq)]
pub enum GoalSpec {
    /// Absolute end_link pose; `r = None` keeps the current orientation.
    Absolute { p: [f64; 3], r: Option<kinematics::Mat3> },
    AbsoluteEx { p: [f64; 3], r: Option<kinematics::Mat3>, auto_yaw: bool },
    Relative { frame: RelFrame, d: [f64; 3], dyaw: f64, dpitch: f64, droll: f64, level: bool, auto_yaw: Option<bool> },
}

/// Resolve a goal request against the start pose `fk(q0)`.
pub fn resolve_goal(input: &PlanInput, spec: &GoalSpec) -> Pose {
    resolve_goal_ex(input, spec).0
}

/// Apply auto-yaw rotation about world Z for 5-DOF arms based on radial azimuth change.
pub fn apply_auto_yaw(start_p: &[f64; 3], goal_p: &[f64; 3], mut target_r: kinematics::Mat3) -> kinematics::Mat3 {
    let r_start = (start_p[0] * start_p[0] + start_p[1] * start_p[1]).sqrt();
    let r_target = (goal_p[0] * goal_p[0] + goal_p[1] * goal_p[1]).sqrt();
    if r_start >= 0.03 && r_target >= 0.03 {
        let dpsi = goal_p[1].atan2(goal_p[0]) - start_p[1].atan2(start_p[0]);
        let rz = kinematics::axis_r(&[0.0, 0.0, 1.0], dpsi);
        target_r = kinematics::mat_mul(&rz, &target_r);
    }
    target_r
}

pub fn align_auto_yaw(goal_p: &[f64; 3], target_r: kinematics::Mat3) -> kinematics::Mat3 {
    let r_target = (goal_p[0] * goal_p[0] + goal_p[1] * goal_p[1]).sqrt();
    if r_target >= 0.03 && !crate::chain::is_tool_vertical(&target_r) {
        let psi_goal = goal_p[1].atan2(goal_p[0]);
        let psi_cur = target_r[1][0].atan2(target_r[0][0]);
        let rz = kinematics::axis_r(&[0.0, 0.0, 1.0], psi_goal - psi_cur);
        kinematics::mat_mul(&rz, &target_r)
    } else {
        target_r
    }
}

/// Resolve a goal request against the start pose `fk(q0)`, returning `(pose, yaw_free)`.
pub fn resolve_goal_ex(input: &PlanInput, spec: &GoalSpec) -> (Pose, bool) {
    let start = input.arm.fk(&input.q0);
    match spec {
        GoalSpec::Absolute { p, r } => {
            let auto_yaw = input.arm.arm_dof() == 5 && r.is_none();
            let mut target_r = r.unwrap_or(start.r);
            if auto_yaw {
                target_r = apply_auto_yaw(&start.p, p, target_r);
            }
            (Pose::new(target_r, *p), auto_yaw)
        }
        GoalSpec::AbsoluteEx { p, r, auto_yaw } => {
            let mut target_r = r.unwrap_or(start.r);
            if *auto_yaw && input.arm.arm_dof() == 5 {
                target_r = match r {
                    None => apply_auto_yaw(&start.p, p, start.r),
                    Some(explicit_r) => align_auto_yaw(p, *explicit_r),
                };
            }
            (Pose::new(target_r, *p), *auto_yaw)
        }
        GoalSpec::Relative { frame, d, dyaw, dpitch, droll, level, auto_yaw } => {
            let x_tool = kinematics::column(&start.r, 0);
            let mut pose = match frame {
                RelFrame::Base => {
                    let p = kinematics::add(&start.p, d);
                    let mut r = start.r;
                    if *droll != 0.0 { r = kinematics::mat_mul(&kinematics::axis_r(&x_tool, *droll), &r); }
                    if *dpitch != 0.0 {
                        // horizontal axis unit(Z × x_tool); positive = tool tips down
                        let axis = kinematics::cross(&[0.0, 0.0, 1.0], &kinematics::column(&r, 0));
                        if kinematics::norm(&axis) > 1e-9 {
                            r = kinematics::mat_mul(&kinematics::axis_r(&kinematics::unit(&axis), *dpitch), &r);
                        }
                    }
                    if *dyaw != 0.0 { r = kinematics::mat_mul(&kinematics::axis_r(&[0.0, 0.0, 1.0], *dyaw), &r); }
                    Pose::new(r, p)
                }
                RelFrame::Tool => {
                    let p = kinematics::add(&start.p, &kinematics::mat_vec(&start.r, d));
                    // intrinsic rotations in end_link axes: x = roll, y = pitch, z = yaw
                    let local = kinematics::mat_mul(
                        &kinematics::mat_mul(&kinematics::axis_r(&[0.0, 0.0, 1.0], *dyaw), &kinematics::axis_r(&[0.0, 1.0, 0.0], *dpitch)),
                        &kinematics::axis_r(&[1.0, 0.0, 0.0], *droll));
                    Pose::new(kinematics::mat_mul(&start.r, &local), p)
                }
                RelFrame::Camera => {
                    let rec = input.arm.cam_mount(input.cam.flip, f64::from(input.cam.rot_deg), f64::from(input.cam.tilt_deg));
                    let rc = kinematics::cam_rotation(&start, &rec);
                    let p = kinematics::add(&start.p, &kinematics::mat_vec(&rc, d));
                    let mut r = start.r;
                    // signs as VIK.integrate: look down (+dpitch) = -dpitch about x_cam,
                    // look right (+dyaw) = -dyaw about world Z
                    if *dpitch != 0.0 { r = kinematics::mat_mul(&kinematics::axis_r(&kinematics::column(&rc, 0), -dpitch), &r); }
                    if *dyaw != 0.0 { r = kinematics::mat_mul(&kinematics::axis_r(&[0.0, 0.0, 1.0], -dyaw), &r); }
                    if *droll != 0.0 {
                        let z_cam = kinematics::column(&kinematics::mat_mul(&r, &rec), 2);
                        r = kinematics::mat_mul(&kinematics::axis_r(&z_cam, *droll), &r);
                    }
                    let goal = Pose::new(r, p);
                    if *level { kinematics::level(&goal, &rec, level_cos()) } else { goal }
                }
            };
            let is_5dof = input.arm.arm_dof() == 5;
            let auto_yaw = match auto_yaw {
                Some(b) => *b && is_5dof,
                None => is_5dof && *dyaw == 0.0 && *dpitch == 0.0 && *droll == 0.0 && (*frame != RelFrame::Camera || !*level),
            };
            if auto_yaw {
                pose.r = apply_auto_yaw(&start.p, &pose.p, pose.r);
                (pose, true)
            } else {
                (pose, false)
            }
        }
    }
}

pub fn num(body: &Value, key: &str) -> Result<Option<f64>, MotionError> {
    match body.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(v) => v.as_f64().filter(|x| x.is_finite()).map(Some)
            .ok_or_else(|| MotionError::InvalidCommand(format!("{key} must be a finite number"))),
    }
}

/// Optional boolean `key`; `default` when absent or null.
pub fn flag(body: &Value, key: &str, default: bool) -> Result<bool, MotionError> {
    match body.get(key) {
        None | Some(Value::Null) => Ok(default),
        Some(v) => v.as_bool().ok_or_else(|| MotionError::InvalidCommand(format!("{key} must be a boolean"))),
    }
}

fn vec_n<const N: usize>(v: &Value, key: &str) -> Result<[f64; N], MotionError> {
    let arr = v.as_array().filter(|a| a.len() == N)
        .ok_or_else(|| MotionError::InvalidCommand(format!("{key} must be an array of {N} numbers")))?;
    let mut out = [0.0; N];
    for (o, x) in out.iter_mut().zip(arr) {
        *o = x.as_f64().filter(|v| v.is_finite()).ok_or_else(|| MotionError::InvalidCommand(format!("{key} must be finite numbers")))?;
    }
    Ok(out)
}

pub fn tag(body: &Value) -> Result<Option<String>, MotionError> {
    match body.get("tag") {
        None | Some(Value::Null) => Ok(None),
        Some(Value::String(s)) if s.len() <= 64 => Ok(Some(s.clone())),
        _ => Err(MotionError::InvalidCommand("tag must be a string of at most 64 bytes".into())),
    }
}

/// `speed: {linear, angular}`, defaults 0.05 m/s / 0.5 rad/s, max 0.15 / 1.0.
pub fn parse_speed(body: &Value) -> Result<Speed, MotionError> {
    let mut speed = Speed::default();
    match body.get("speed") {
        None | Some(Value::Null) => {}
        Some(v @ Value::Object(_)) => {
            if let Some(l) = num(v, "linear")? { speed.linear = l; }
            if let Some(a) = num(v, "angular")? { speed.angular = a; }
        }
        Some(_) => return Err(MotionError::InvalidCommand("speed must be an object {linear, angular}".into())),
    }
    if !(speed.linear > 0.0 && speed.linear <= MAX_LINEAR_MPS) {
        return Err(MotionError::InvalidCommand(format!("speed.linear must be in (0, {MAX_LINEAR_MPS}] m/s")));
    }
    if !(speed.angular > 0.0 && speed.angular <= MAX_ANGULAR_RPS) {
        return Err(MotionError::InvalidCommand(format!("speed.angular must be in (0, {MAX_ANGULAR_RPS}] rad/s")));
    }
    Ok(speed)
}

/// Body of `/api/v1/action/move_to`.
pub fn parse_move_to(body: &Value) -> Result<(GoalSpec, Speed, Option<String>), MotionError> {
    let p = parse_position(body)?;
    let r = parse_orientation(body)?;
    let auto_yaw = match body.get("auto_yaw") {
        None | Some(Value::Null) => None,
        Some(Value::Bool(b)) => Some(*b),
        Some(_) => return Err(MotionError::InvalidCommand("auto_yaw must be a boolean".into())),
    };
    let spec = match auto_yaw {
        Some(ay) => GoalSpec::AbsoluteEx { p, r, auto_yaw: ay },
        None => GoalSpec::Absolute { p, r },
    };
    Ok((spec, parse_speed(body)?, tag(body)?))
}

/// `position: [x, y, z]` (finite numbers).
pub fn parse_position(body: &Value) -> Result<[f64; 3], MotionError> {
    vec_n::<3>(body.get("position").unwrap_or(&Value::Null), "position")
}

/// `orientation: {"rpy": [r, p, y]} | {"quat": [w, x, y, z]} | null` (absent = None).
pub fn parse_orientation(body: &Value) -> Result<Option<kinematics::Mat3>, MotionError> {
    Ok(match body.get("orientation") {
        None | Some(Value::Null) => None,
        Some(o) => match (o.get("rpy"), o.get("quat")) {
            (Some(rpy), None) => { let v = vec_n::<3>(rpy, "orientation.rpy")?; Some(kinematics::rpy_to_r(v[0], v[1], v[2])) }
            (None, Some(q)) => {
                let v = vec_n::<4>(q, "orientation.quat")?;
                if (v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]).sqrt() < 1e-6 {
                    return Err(MotionError::InvalidCommand("orientation.quat must be non-zero".into()));
                }
                Some(kinematics::quat_to_r(&v))
            }
            _ => return Err(MotionError::InvalidCommand("orientation must be {\"rpy\":[r,p,y]} or {\"quat\":[w,x,y,z]} or null".into())),
        },
    })
}

/// Body of `/api/v1/action/move_relative`.
pub fn parse_move_relative(body: &Value) -> Result<(GoalSpec, Speed, Option<String>), MotionError> {
    let frame = match body.get("frame").and_then(Value::as_str) {
        Some("base") => RelFrame::Base,
        Some("tool") => RelFrame::Tool,
        Some("camera") => RelFrame::Camera,
        _ => return Err(MotionError::InvalidCommand("frame must be \"base\", \"tool\" or \"camera\"".into())),
    };
    let d = match body.get("d") { None | Some(Value::Null) => [0.0; 3], Some(v) => vec_n::<3>(v, "d")? };
    let dyaw = num(body, "dyaw")?.unwrap_or(0.0);
    let dpitch = num(body, "dpitch")?.unwrap_or(0.0);
    let droll = num(body, "droll")?.unwrap_or(0.0);
    let level = match body.get("level") {
        None | Some(Value::Null) => frame == RelFrame::Camera,
        Some(Value::Bool(b)) => *b,
        Some(_) => return Err(MotionError::InvalidCommand("level must be boolean".into())),
    };
    if level && frame != RelFrame::Camera {
        return Err(MotionError::InvalidCommand("level applies to the camera frame only".into()));
    }
    let auto_yaw = match body.get("auto_yaw") {
        None | Some(Value::Null) => None,
        Some(Value::Bool(b)) => Some(*b),
        Some(_) => return Err(MotionError::InvalidCommand("auto_yaw must be boolean".into())),
    };
    Ok((GoalSpec::Relative { frame, d, dyaw, dpitch, droll, level, auto_yaw }, parse_speed(body)?, tag(body)?))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GripperCommand { Open, Close, Move, Seek }

#[derive(Debug, Clone, PartialEq)]
pub struct GripperRequest {
    pub command: GripperCommand,
    /// 0 = closed, 1 = open (move only).
    pub fraction: Option<f64>,
    pub grasp: bool,
    /// J7 speed (rad/s), <= 2.0.
    pub speed: f64,
    /// Deadline (s). None: `max(GRIPPER_DEFAULT_TIMEOUT_S, expected duration + GRIPPER_TIMEOUT_MARGIN_S)`.
    pub timeout_s: Option<f64>,
    pub tag: Option<String>,
}

/// Shortest default gripper deadline (s).
pub const GRIPPER_DEFAULT_TIMEOUT_S: f64 = 6.0;
/// Added to the expected duration for the default gripper deadline (s).
pub const GRIPPER_TIMEOUT_MARGIN_S: f64 = 2.0;

/// Body of `/api/v1/action/gripper`.
pub fn parse_gripper(body: &Value) -> Result<GripperRequest, MotionError> {
    if body.get("width_m").is_some_and(|v| !v.is_null()) {
        return Err(MotionError::Unsupported("width_m is not supported until the jaw angle-to-width mapping is measured; use fraction".into()));
    }
    let command = match body.get("command").and_then(Value::as_str) {
        Some("open") => GripperCommand::Open,
        Some("close") => GripperCommand::Close,
        Some("move") => GripperCommand::Move,
        _ => return Err(MotionError::InvalidCommand("command must be \"open\", \"close\" or \"move\"".into())),
    };
    let fraction = num(body, "fraction")?;
    match (command, fraction) {
        (GripperCommand::Move, None) => return Err(MotionError::InvalidCommand("move needs fraction in [0, 1]".into())),
        (GripperCommand::Move, Some(f)) if !(0.0..=1.0).contains(&f) => return Err(MotionError::InvalidCommand("fraction must be in [0, 1]".into())),
        (GripperCommand::Open | GripperCommand::Close, Some(_)) => return Err(MotionError::InvalidCommand("fraction applies to move only".into())),
        _ => {}
    }
    let grasp = match body.get("grasp") {
        None | Some(Value::Null) => command == GripperCommand::Close,
        Some(Value::Bool(b)) => *b,
        Some(_) => return Err(MotionError::InvalidCommand("grasp must be boolean".into())),
    };
    if grasp && command != GripperCommand::Close {
        return Err(MotionError::InvalidCommand("grasp applies to close only".into()));
    }
    let speed = num(body, "speed")?.unwrap_or(1.0);
    if !(speed > 0.0 && speed <= 2.0) {
        return Err(MotionError::InvalidCommand("speed must be in (0, 2.0] rad/s".into()));
    }
    let timeout_s = num(body, "timeout_s")?;
    if timeout_s.is_some_and(|t| !(t > 0.0 && t <= 30.0)) {
        return Err(MotionError::InvalidCommand("timeout_s must be in (0, 30] s".into()));
    }
    Ok(GripperRequest { command, fraction, grasp, speed, timeout_s, tag: tag(body)? })
}

// ---- execution ----

/// Measured / commanded state handed to a reference each tick.
pub struct StepCtx<'a> {
    pub dt: f64,
    /// Power governor speed scale (1 = none): references advance their time at dt x scale.
    pub time_scale: f64,
    pub measured: &'a [f32],
    pub measured_velocities: &'a [f32],
    pub commanded: &'a [f32],
    pub commanded_velocities: &'a [f32],
    pub torques: &'a [f32],
    pub loads: &'a [f32],
    pub feedback_fresh: bool,
    pub torque_feedback: bool,
    pub load_feedback: bool,
    pub gripper_stalled: bool,
    pub gripper_stall_sign: f32,
}

#[derive(Debug, Clone, PartialEq)]
pub enum StepStatus {
    Running,
    Done { settled: Option<bool>, holding: Option<bool>, gripper_gap_rad: Option<f32> },
    Failed { code: &'static str, message: String },
}

#[derive(Debug, Clone, PartialEq)]
pub struct StepOut {
    pub positions: Vec<f32>,
    pub velocities: Vec<f32>,
    pub status: StepStatus,
    /// 0..1
    pub progress: f32,
    /// Clear the gripper stall latch (direction reversal).
    pub clear_gripper_stall: bool,
}

/// Lag-aware time scaling band (rad): full speed below LAG_FULL, stop above LAG_STOP.
pub const LAG_FULL: f64 = 0.38;
pub const LAG_STOP: f64 = 0.45;
pub const STALL_TIMEOUT_S: f64 = 1.0;
pub const SETTLE_POS_RAD: f64 = 0.05;
pub const SETTLE_VEL: f64 = 0.05;
/// An open / move gripper action that stops within this distance of its target counts as done.
pub const GRIPPER_STOP_SHORT_RAD: f32 = 0.25;
pub const SETTLE_HOLD_S: f64 = 0.2;
pub const SETTLE_MAX_S: f64 = 2.0;

/// Settle rule shared by every arm action: max_{i<6}|cmd−meas| < 0.05 and |v| < 0.05 for 200 ms.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Settle { pub ok_for: f64, pub elapsed: f64 }

impl Settle {
    /// Some(settled) when finished.
    pub fn step(&mut self, dt: f64, commanded: &[f32], measured: &[f32], velocities: &[f32]) -> Option<bool> {
        self.elapsed += dt;
        let n = commanded.len().min(measured.len()).min(6);
        let still = (0..n).all(|i| (commanded[i] - measured[i]).abs() < SETTLE_POS_RAD as f32
            && velocities.get(i).is_some_and(|v| v.abs() < SETTLE_VEL as f32));
        if still { self.ok_for += dt; } else { self.ok_for = 0.0; }
        if self.ok_for >= SETTLE_HOLD_S - 1e-9 { Some(true) }
        else if self.elapsed >= SETTLE_MAX_S { Some(false) }
        else { None }
    }
}

#[derive(Debug, Clone)]
pub struct PathTrajectory {
    pub path: JointPath,
    /// Path time, advanced at `dt · rate`.
    pub elapsed: f64,
    pub stalled_for: f64,
    pub settle: Option<Settle>,
    index: usize,
}

impl PathTrajectory {
    pub fn new(path: JointPath) -> Self { Self { path, elapsed: 0.0, stalled_for: 0.0, settle: None, index: 0 } }

    pub fn duration(&self) -> f64 { self.path.duration }

    fn position_at(&mut self, s: f64) -> [f32; 7] {
        let ss = &self.path.s;
        while self.index + 1 < ss.len() - 1 && ss[self.index + 1] <= s { self.index += 1; }
        let (a, b) = (self.index, (self.index + 1).min(ss.len() - 1));
        if a == b { return self.path.q[a]; }
        let f = ((s - ss[a]) / (ss[b] - ss[a])).clamp(0.0, 1.0) as f32;
        std::array::from_fn(|i| self.path.q[a][i] + (self.path.q[b][i] - self.path.q[a][i]) * f)
    }

    pub fn step(&mut self, ctx: &StepCtx) -> StepOut {
        let dof = ctx.commanded.len();
        let last = self.path.last()[..dof].to_vec();
        if let Some(settle) = self.settle.as_mut() {
            let status = match settle.step(ctx.dt, ctx.commanded, ctx.measured, ctx.measured_velocities) {
                Some(settled) => StepStatus::Done { settled: Some(settled), holding: None, gripper_gap_rad: None },
                None => StepStatus::Running,
            };
            return StepOut { positions: last, velocities: vec![0.0; dof], status, progress: 1.0, clear_gripper_stall: false };
        }
        let arm_joints = dof.saturating_sub(1);
        let lag = (0..arm_joints).map(|i| f64::from((ctx.commanded[i] - ctx.measured[i]).abs())).fold(0.0, f64::max);
        let lag = if lag.is_finite() { lag } else { f64::INFINITY };
        let rate = ((LAG_STOP - lag) / (LAG_STOP - LAG_FULL)).clamp(0.0, 1.0);
        if rate == 0.0 {
            self.stalled_for += ctx.dt;
            if self.stalled_for >= STALL_TIMEOUT_S - 1e-9 {
                return StepOut {
                    positions: ctx.commanded.to_vec(), velocities: vec![0.0; dof],
                    status: StepStatus::Failed { code: "TRACKING_STALL", message: format!(
                        "Arm did not follow the path for {STALL_TIMEOUT_S} s (lag {lag:.3} rad); motion stopped") },
                    progress: (self.elapsed / self.path.duration) as f32, clear_gripper_stall: false,
                };
            }
        } else {
            self.stalled_for = 0.0;
        }
        self.elapsed = (self.elapsed + ctx.dt * rate * ctx.time_scale).min(self.path.duration);
        let u = self.elapsed / self.path.duration;
        let s = u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
        let q = self.position_at(s);
        let velocities: Vec<f32> = (0..dof).map(|i| ((q[i] - ctx.commanded[i]) as f64 / ctx.dt) as f32).collect();
        if self.elapsed >= self.path.duration {
            self.settle = Some(Settle::default());
            return StepOut { positions: last, velocities: vec![0.0; dof], status: StepStatus::Running, progress: 1.0, clear_gripper_stall: false };
        }
        StepOut { positions: q[..dof].to_vec(), velocities, status: StepStatus::Running, progress: u as f32, clear_gripper_stall: false }
    }
}

#[derive(Debug, Clone, PartialEq)]
enum GripPhase {
    Moving,
    /// Squeezing an object: hold target, samples of |τ_gripper| and |load_gripper|.
    Squeeze { hold: f32, elapsed: f64, taus: Vec<f32>, loads: Vec<f32> },
}

#[derive(Debug, Clone)]
pub struct GripperMotion {
    pub request: GripperRequest,
    pub policy: GripperPolicy,
    pub limit: JointLimit,
    pub target: f32,
    pub start: f32,
    pub gripper_index: usize,
    pub elapsed: f64,
    /// Deadline (s): the request's timeout_s, else the default rule.
    pub timeout_s: f64,
    still_for: f64,
    moved: bool,
    phase: GripPhase,
}

impl GripperMotion {
    pub fn new(request: GripperRequest, policy: GripperPolicy, limit: JointLimit, start: f32, gripper_index: usize) -> Self {
        let target = match request.command {
            GripperCommand::Open => policy.open_rad,
            GripperCommand::Close => policy.closed_rad,
            GripperCommand::Move => policy.closed_rad + request.fraction.unwrap_or(0.0) as f32 * (policy.open_rad - policy.closed_rad),
            GripperCommand::Seek => {
                let dir = if policy.closed_rad >= policy.open_rad { 1.0 } else { -1.0 };
                policy.closed_rad + dir * 0.6
            }
        }.clamp(limit.min_position, limit.max_position);
        let limit = JointLimit { max_velocity: request.speed as f32, ..limit };
        let mut m = Self { request, policy, limit, target, start, gripper_index, elapsed: 0.0, timeout_s: 0.0, still_for: 0.0, moved: false, phase: GripPhase::Moving };
        m.timeout_s = m.request.timeout_s
            .unwrap_or_else(|| GRIPPER_DEFAULT_TIMEOUT_S.max(m.expected_duration() + GRIPPER_TIMEOUT_MARGIN_S));
        m
    }

    /// Expected duration (s) for the travel at the requested speed, plus settle.
    pub fn expected_duration(&self) -> f64 {
        f64::from((self.target - self.start).abs()) / self.request.speed + 0.5 + self.policy.settle_ms as f64 / 1000.0
    }

    fn close_dir(&self) -> f32 { if self.policy.closed_rad >= self.policy.open_rad { 1.0 } else { -1.0 } }

    pub fn step(&mut self, ctx: &StepCtx) -> StepOut {
        self.elapsed += ctx.dt;
        let mut positions: Vec<f32> = ctx.commanded.to_vec();
        let mut velocities = vec![0.0f32; positions.len()];
        let idx = self.gripper_index;
        let (q_g, v_gc) = (ctx.commanded.get(idx).copied().unwrap_or(0.0), ctx.commanded_velocities.get(idx).copied().unwrap_or(0.0));
        let (m_g, mv_g) = (ctx.measured.get(idx).copied().unwrap_or(0.0), ctx.measured_velocities.get(idx).copied().unwrap_or(0.0));
        let tau_g = ctx.torques.get(idx).copied().unwrap_or(0.0);
        // A latch set while gripper pushed the other way (e.g. the squeeze of a grasp, or the push that
        // lasts while the command unwinds from the squeeze hold) does not block travel away from
        // it: release it on every tick, not only the first. Only a stall in the travel direction holds.
        let clear = ctx.gripper_stalled && (self.target - q_g) * ctx.gripper_stall_sign < -1e-4;
        let stalled = ctx.gripper_stalled && !clear;
        let progress_of = |q: f32| -> f32 {
            let span = (self.target - self.start).abs();
            if span < 1e-6 { 1.0 } else { (1.0 - (self.target - q).abs() / span).clamp(0.0, 1.0) }
        };
        let done = |holding: Option<bool>, gap: Option<f32>| StepStatus::Done { settled: None, holding, gripper_gap_rad: gap };

        if let GripPhase::Squeeze { hold, elapsed, taus, loads } = &mut self.phase {
            *elapsed += ctx.dt;
            taus.push(tau_g.abs());
            loads.push(ctx.loads.get(idx).copied().unwrap_or(0.0).abs());
            if idx < positions.len() { positions[idx] = *hold; }
            let status = if *elapsed * 1000.0 >= self.policy.settle_ms as f64 - 1e-6 {
                let mut sorted = taus.clone();
                sorted.sort_by(|a, b| a.total_cmp(b));
                let tau = sorted[sorted.len() / 2];
                let mut sorted_loads = loads.clone();
                sorted_loads.sort_by(|a, b| a.total_cmp(b));
                let median_abs_load = if sorted_loads.is_empty() { 0.0 } else { sorted_loads[sorted_loads.len() / 2] };
                let gap = (self.policy.closed_rad - m_g) * self.close_dir();
                let holding = match self.policy.holding_mode {
                    HoldingMode::TorqueAndGap => ctx.torque_feedback && gap >= self.policy.holding_gap_rad && tau >= self.policy.holding_torque_nm,
                    HoldingMode::PositionGap => {
                        ctx.feedback_fresh
                            && (self.moved || (self.start - m_g).abs() < 0.05)
                            && mv_g.abs() < SETTLE_VEL as f32
                            && gap >= self.policy.holding_gap_rad
                    }
                    HoldingMode::LoadAndGap => {
                        let req = self.policy.holding_load_fraction.unwrap_or(0.15);
                        ctx.feedback_fresh
                            && ctx.load_feedback
                            && gap >= self.policy.holding_gap_rad
                            && median_abs_load >= req
                    }
                };
                done(Some(holding), Some(gap))
            } else { StepStatus::Running };
            return StepOut { positions, velocities, status, progress: 1.0, clear_gripper_stall: clear };
        }

        if self.elapsed > self.timeout_s {
            if idx < positions.len() { positions[idx] = m_g; }
            return StepOut { positions, velocities, progress: progress_of(m_g), clear_gripper_stall: clear,
                status: StepStatus::Failed { code: "GRIPPER_TIMEOUT", message: format!(
                    "Gripper did not finish within {:.1} s (at {m_g:.3} rad, target {:.3} rad)", self.timeout_s, self.target) } };
        }

        // Step gripper toward the target unless the stall latch holds it.
        if !stalled {
            match crate::supervisor::SupervisorInner::bounded_servo_step(q_g, self.target, v_gc, ctx.dt, &self.limit) {
                Some((q, v, _)) => {
                    if idx < positions.len() { positions[idx] = q; }
                    if idx < velocities.len() { velocities[idx] = v; }
                }
                None => {
                    if idx < positions.len() { positions[idx] = q_g; }
                }
            }
        }
        if (m_g - self.start).abs() > 0.05 { self.moved = true; }
        let cmd_at_target = idx < positions.len() && (positions[idx] - self.target).abs() < 1e-4;
        if mv_g.abs() < SETTLE_VEL as f32 && (self.moved || cmd_at_target) { self.still_for += ctx.dt; } else { self.still_for = 0.0; }
        let still = self.still_for >= SETTLE_HOLD_S - 1e-9;

        let grasping = self.request.command == GripperCommand::Close && self.request.grasp;
        let status = if grasping {
            if stalled || still {
                let dir = self.close_dir();
                let squeezed = m_g + dir * self.policy.squeeze_rad;
                let hold = if dir > 0.0 { squeezed.min(self.policy.closed_rad) } else { squeezed.max(self.policy.closed_rad) }
                    .clamp(self.limit.min_position, self.limit.max_position);
                if idx < positions.len() { positions[idx] = hold; }
                if idx < velocities.len() { velocities[idx] = 0.0; }
                self.phase = GripPhase::Squeeze { hold, elapsed: 0.0, taus: Vec::new(), loads: Vec::new() };
                StepStatus::Running
            } else { StepStatus::Running }
        } else if stalled {
            if idx < positions.len() { positions[idx] = q_g; }
            if idx < velocities.len() { velocities[idx] = 0.0; }
            done(None, Some((self.policy.closed_rad - m_g) * self.close_dir()))
        } else if self.request.command != GripperCommand::Seek && still && ((m_g - self.target).abs() < SETTLE_POS_RAD as f32
            || (cmd_at_target && (m_g - self.target).abs() < GRIPPER_STOP_SHORT_RAD)) {
            // the jaw stopped close to the target: the real gripper opens to about -5.61 against a
            // -5.70 command (mechanical stop / POS_VEL offset); stopping farther away is a jam -> timeout
            done(None, Some((self.policy.closed_rad - m_g) * self.close_dir()))
        } else { StepStatus::Running };
        StepOut { positions, velocities, status, progress: progress_of(m_g), clear_gripper_stall: clear }
    }
}

/// What the supervisor steps each tick while a trajectory is active.
#[derive(Debug, Clone)]
pub enum Reference {
    Quintic(MultiJointQuinticTrajectory),
    Path(PathTrajectory),
    Gripper(GripperMotion),
    /// Episode replay (`crate::replay`).
    Replay(Box<crate::replay::ReplayMotion>),
}

#[derive(Debug, Clone)]
pub struct ActiveTrajectory {
    pub action_id: Option<u64>,
    pub reference: Reference,
}

impl ActiveTrajectory {
    pub fn quintic(traj: MultiJointQuinticTrajectory, action_id: Option<u64>) -> Self {
        Self { action_id, reference: Reference::Quintic(traj) }
    }
    /// Planned duration (s); for a gripper motion the expected duration.
    pub fn duration(&self) -> f64 {
        match &self.reference {
            Reference::Quintic(t) => t.duration,
            Reference::Path(p) => p.duration(),
            Reference::Gripper(g) => g.expected_duration(),
            Reference::Replay(r) => r.expected_duration(),
        }
    }
    pub fn as_quintic(&self) -> Option<&MultiJointQuinticTrajectory> {
        match &self.reference { Reference::Quintic(t) => Some(t), _ => None }
    }
}

/// Status of the most recent action, published in every observation.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ActionStatus {
    pub id: u64,
    /// move_to | move_relative | gripper | grasp | place | joint_move | home | park | move_to_pose | replay
    pub kind: String,
    pub tag: Option<String>,
    /// running | done | failed | cancelled
    pub state: String,
    pub code: Option<String>,
    pub message: Option<String>,
    pub progress: f32,
    pub started_at: f64,
    pub finished_at: Option<f64>,
    pub holding: Option<bool>,
    pub gripper_gap_rad: Option<f32>,
    pub settled: Option<bool>,
    /// Grasp only: open | approach | centring | transition | align | descend | close | release | lift | done.
    #[serde(default)]
    pub phase: Option<String>,
    /// Grasp centring: corrections made so far and the latest pixel error.
    #[serde(default)]
    pub iteration: Option<u32>,
    #[serde(default)]
    pub error_px: Option<f32>,
    /// Grasp camera-down acquisition: target box centre minus `grip_pixel_norm` (px) in a
    /// frame after the transition; diagnostic, absent when not measured.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub final_offset_px: Option<[f32; 2]>,
    /// Grasp camera-down acquisition: TCP height above the table (m) used instead of
    /// `approach_height_m`, raised for table clearance; absent when not raised.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub acquire_height_m: Option<f32>,
    /// Power-budget time scale of the planned motion (1 = as planned, < 1 slowed to fit);
    /// absent without a power budget or for motions it does not plan (gripper).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub power_scale: Option<f32>,
    /// Replay only: episode id.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub episode: Option<String>,
    /// Replay only: recording time reached (s); 0 during the approach (`phase` approach |
    /// replay | settle).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub t_s: Option<f32>,
    /// Grasp J6 alignment: `turned:box` / `turned:angle` (roll in `align_turn_rad`),
    /// `kept:box` / `kept:angle` (no turn needed), `skipped:<reason>` or `off`; absent before the phase.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub align: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub align_turn_rad: Option<f32>,
    /// Grasp J6 alignment: angle dispersion across centring frames (deg).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub angle_spread_deg: Option<f32>,
    /// Place orientation override (`levelled` when tilt was reduced for a spill-safe object).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub orientation: Option<String>,
    /// Grasp: measured cross-section width of the target on the table plane (m).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub grasp_width_m: Option<f32>,
    /// Grasp height probe: `scale` (up probe: object top from the box size change), `parallax`
    /// (lateral probe: object top from the box-centre shift), `object_height_m` above the table,
    /// or `setup` (fallback to `grasp_height_m`, why in `height_reason`). `height_reason` also
    /// carries `lateral_fallback:<why>` when the lateral probe gave way to the up probe.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub height_source: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub height_reason: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub object_height_m: Option<f32>,
    /// Grasp: TCP height above the table at which the jaws close (m).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub close_height_m: Option<f32>,
    /// Grasp height probe: the upward move used (m), absent when the probe did not move.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub probe_dz_m: Option<f32>,
    /// Grasp depth anchoring source (`metric_table`, `oblique_ransac`, `two_height_fk`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub depth_anchor: Option<String>,
    /// Grasp: measured optical depth of the target foreground (m).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub object_depth_m: Option<f32>,
    /// Grasp: measured object length on the table plane (m).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub object_length_m: Option<f32>,
    /// Grasp: true when depth inference confirms the jaw descent corridor is clear.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub jaw_clear: Option<bool>,
    /// Grasp centring with the target's box cut by the image border: `shifted` (centred uncut
    /// at an aim pixel inside the image, `border_shift_px` from the grip pixel / image centre,
    /// then moved by the model's offset between the two) or `cut` (still cut: centred on the
    /// cut box); absent when the box was not cut.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub border: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub border_shift_px: Option<[f32; 2]>,
    /// Grasp height probe that ran: `up` or `lateral` (absent when the probe was skipped).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub probe_mode: Option<String>,
    /// Lateral probe: measured lens displacement along the image x axis between the two poses
    /// (m, FK of the measured joints) and its difference from `lateral_probe_m` (mm).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lateral_baseline_m: Option<f32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lateral_baseline_err_mm: Option<f32>,
    /// Grasp mid-height re-centre (look-down): `done`, `unconverged` or `skipped:<reason>`,
    /// with the corrections made.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub recentre: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub recentre_iterations: Option<u32>,
    /// Grasp descent: largest |residual torque| of J2, J3 (N·m; absent without residuals) and
    /// largest |commanded − measured| of J1..J6 (rad) while the descent ran.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub descend_peak_residual_nm: Option<[f32; 2]>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub descend_peak_tracking_rad: Option<f32>,
    /// Grasp descent stopped on contact (`descend_contact_stop_nm`) at this TCP height above
    /// the table (m).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub descend_contact: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub descend_contact_height_m: Option<f32>,
    /// Grasp holding re-check after the lift: `held`, `regripped` (grip torque gone, a re-grip
    /// stopped on the object) or `dropped`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub post_lift: Option<String>,
    /// Grasp width-based pre-open: `done`, `off` or `skipped:<reason>`; absent before the phase.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pre_open: Option<String>,
    /// Grasp pre-open opening width (m) before descend; absent when pre-open did not run.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pre_open_m: Option<f32>,
    /// Grasp opening width (m) from measured J7 after the close.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub held_width_m: Option<f32>,
    /// Gripper zero check: measured J7 position at the stall stop (rad).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stop_rad: Option<f32>,
    /// Gripper zero check: stop_rad minus base closed_rad (rad).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub offset_rad: Option<f32>,
    /// Gripper zero check: true when |offset_rad| <= threshold_rad.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pass: Option<bool>,
    /// Accumulated runtime per macro phase (ms).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub phase_ms: BTreeMap<String, u32>,
    /// Execution diagnostics for each arm sub-motion of the macro action.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub motions: Vec<MacroMotionDiag>,
}

/// Execution diagnostics for one sub-motion in a macro action (grasp, place).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct MacroMotionDiag {
    pub phase: String,
    pub ms: u32,
    pub track_mm: f32,
    pub track_deg: f32,
    pub goal_mm: f32,
    pub goal_deg: f32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub contact: Option<bool>,
}

impl ActionStatus {
    pub fn is_running(&self) -> bool { self.state == "running" }
}
