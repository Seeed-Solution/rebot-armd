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
use crate::model::{CameraMount, JointLimit, WorkspacePolicy};
#[cfg(not(target_arch = "wasm32"))]
use crate::model::{GripperPolicy, HoldingMode};
use crate::trajectory::MultiJointQuinticTrajectory;

#[derive(Debug, Clone, PartialEq, Error)]
pub enum MotionError {
    /// `yaw_off_deg`: largest [`outward_hint`] over the start and the samples up to `s`.
    #[error("IK unreachable at s={s:.3} (position error {ep:.4} m, rotation error {er:.4} rad){}", yaw_hint(*.yaw_off_deg))]
    IkUnreachable { s: f64, ep: f64, er: f64, yaw_off_deg: Option<f64> },
    #[error("IK branch flip at s={s:.3}: joint {joint} jumps {dq:.3} rad between samples (limit 0.10){}", yaw_hint(*.yaw_off_deg))]
    IkBranchFlip { s: f64, joint: usize, dq: f64, yaw_off_deg: Option<f64> },
    /// `path: "joint"`: no IK seed reached the goal pose within the joint limits (best of `seeds`).
    #[error("No IK solution for the goal pose within the joint limits (best of {seeds} seeds: position error {ep:.4} m, rotation error {er:.4} rad)")]
    IkGoalUnreachable { ep: f64, er: f64, seeds: usize },
    #[error("Path leaves the safe workspace at s={s:.3}: check point {point} {limit} {value:.3} m beyond bound {bound:.3} m")]
    WorkspaceViolation {
        s: f64,
        point: usize,
        limit: String,
        value: f64,
        bound: f64,
        detail: Box<kinematics::WorkspaceViolationDetail>,
    },
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
            Self::IkUnreachable { .. } | Self::IkGoalUnreachable { .. } => "IK_UNREACHABLE",
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

/// Message suffix of an IK failure whose sample yaw is off the outward direction.
fn yaw_hint(yaw_off_deg: Option<f64>) -> String {
    match yaw_off_deg {
        Some(d) => format!("; yaw differs from the outward direction by {d:.0}°; try yaw: \"outward\""),
        None => String::new(),
    }
}

// ---- outward yaw ----

/// `yaw: "outward"` is refused when the path passes closer than this to the base z axis
/// (atan2(y, x) is undefined there); same radius as the 5-DOF auto_yaw.
pub const OUTWARD_MIN_RADIUS_M: f64 = 0.03;
/// IK failures on arms with 6+ arm joints get the `yaw: "outward"` hint when the yaw of the start
/// or of a sample up to the failing one differs from atan2(y, x) by more than this (the largest
/// such difference is reported).
pub const OUTWARD_HINT_DEG: f64 = 15.0;

/// Orientation options of a Cartesian path.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PathOpts {
    /// 5-DOF auto_yaw: the IK leaves yaw free.
    pub yaw_free: bool,
    /// `yaw: "outward"`: every sample's yaw is atan2(y, x) of its position plus the start's
    /// offset from it, which goes to 0 along the path (see [`plan_cartesian_opts`]).
    pub yaw_outward: bool,
}

impl From<bool> for PathOpts {
    fn from(yaw_free: bool) -> Self { Self { yaw_free, yaw_outward: false } }
}

/// `r` with its rpy yaw (heading of the end_link x axis, atan2(R[1][0], R[0][0])) replaced by
/// atan2(y, x) of `p`: Rz(atan2(y, x) − yaw) · r, which keeps roll and pitch of
/// R = Rz(yaw)·Ry(pitch)·Rx(roll). With the tool x axis vertical
/// ([`crate::chain::is_tool_vertical`], pitch ±90°) yaw and roll are one rotation and `r` is
/// returned unchanged.
pub fn with_outward_yaw(p: &[f64; 3], r: &kinematics::Mat3) -> kinematics::Mat3 {
    if crate::chain::is_tool_vertical(r) { return *r; }
    let dyaw = p[1].atan2(p[0]) - r[1][0].atan2(r[0][0]);
    kinematics::mat_mul(&kinematics::axis_r(&[0.0, 0.0, 1.0], dyaw), r)
}

/// Distance of the segment a → b, projected on the xy plane, from the base z axis.
fn segment_axis_distance(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 1e-18 { (-(a[0] * dx + a[1] * dy) / len2).clamp(0.0, 1.0) } else { 0.0 };
    (a[0] + dx * t).hypot(a[1] + dy * t)
}

/// Signed angle about base z from the xy direction of `a` to that of `b`.
fn azimuth_delta(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    (a[0] * b[1] - a[1] * b[0]).atan2(a[0] * b[0] + a[1] * b[1])
}

fn wrap_pi(a: f64) -> f64 {
    let a = (a + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI);
    a - std::f64::consts::PI
}

/// |yaw − atan2(y, x)| in degrees when it exceeds [`OUTWARD_HINT_DEG`] at a sample of a path
/// planned without `yaw: "outward"` on an arm with 6+ arm joints; None otherwise (5-DOF, near the
/// base axis, tool vertical, or yaw close to outward).
pub fn outward_hint(arm_dof: usize, opts: PathOpts, p: &[f64; 3], r: &kinematics::Mat3) -> Option<f64> {
    if arm_dof < 6 || opts.yaw_outward || p[0].hypot(p[1]) < OUTWARD_MIN_RADIUS_M || crate::chain::is_tool_vertical(r) {
        return None;
    }
    let off = wrap_pi(r[1][0].atan2(r[0][0]) - p[1].atan2(p[0])).abs().to_degrees();
    (off > OUTWARD_HINT_DEG).then_some(off)
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
/// Maximum added FK evaluations during Cartesian path workspace sweep.
pub const MAX_CARTESIAN_SWEEP_EVALS: usize = 4000;
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

/// Frame of Cartesian planning goals and `start_pose`: raw `end_link` vs active `tcp`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GoalFrame {
    #[default]
    EndLink,
    Tcp,
}

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
    /// Active tool TCP offset in `end_link` frame (m).
    pub tcp_offset: [f64; 3],
    /// Coordinate frame of `goal` and `resolve_goal*` outputs (`EndLink` or `Tcp`).
    pub goal_frame: GoalFrame,
}

/// Convert an `end_link` pose into the TCP frame (`p_tcp = p_end + R_end * tcp_offset`).
pub fn end_to_tcp(end: &Pose, tcp_offset: &[f64; 3]) -> Pose {
    if *tcp_offset == [0.0; 3] {
        return *end;
    }
    Pose::new(
        end.r,
        kinematics::add(&end.p, &kinematics::mat_vec(&end.r, tcp_offset)),
    )
}

/// Convert a TCP pose into the `end_link` frame (`p_end = p_tcp - R_tcp * tcp_offset`).
pub fn tcp_to_end(tcp: &Pose, tcp_offset: &[f64; 3]) -> Pose {
    if *tcp_offset == [0.0; 3] {
        return *tcp;
    }
    Pose::new(
        tcp.r,
        kinematics::sub(&tcp.p, &kinematics::mat_vec(&tcp.r, tcp_offset)),
    )
}

/// Convert a raw `end_link` pose into `input.goal_frame`.
pub fn end_to_frame(end: &Pose, input: &PlanInput) -> Pose {
    match input.goal_frame {
        GoalFrame::EndLink => *end,
        GoalFrame::Tcp => end_to_tcp(end, &input.tcp_offset),
    }
}

/// Convert a pose expressed in `input.goal_frame` into `end_link`.
pub fn goal_to_end(goal: &Pose, input: &PlanInput) -> Pose {
    match input.goal_frame {
        GoalFrame::EndLink => *goal,
        GoalFrame::Tcp => tcp_to_end(goal, &input.tcp_offset),
    }
}

/// Start pose of `input.q0` in `input.goal_frame`.
pub fn start_pose(input: &PlanInput) -> Pose {
    end_to_frame(&input.arm.fk(&input.q0), input)
}

thread_local! {
    /// DLS iterations of the Cartesian planners on this thread (see [`ik_iterations`]).
    static IK_ITERATIONS: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
}

/// Total DLS iterations run by [`plan_cartesian_opts`] and [`solve_goal_joints`] on the calling
/// thread since it started; callers take differences to count the work of one plan.
pub fn ik_iterations() -> u64 { IK_ITERATIONS.with(std::cell::Cell::get) }

fn solve_ik_in_frame(
    input: &PlanInput,
    goal_in_frame: &Pose,
    q_seed: &[f64; 6],
    lo: &[f64; 6],
    hi: &[f64; 6],
    yaw_free: bool,
) -> kinematics::IkResult {
    let sol = solve_ik_in_frame_uncounted(input, goal_in_frame, q_seed, lo, hi, yaw_free);
    IK_ITERATIONS.with(|c| c.set(c.get() + sol.iterations as u64));
    sol
}

fn solve_ik_in_frame_uncounted(
    input: &PlanInput,
    goal_in_frame: &Pose,
    q_seed: &[f64; 6],
    lo: &[f64; 6],
    hi: &[f64; 6],
    yaw_free: bool,
) -> kinematics::IkResult {
    if input.goal_frame == GoalFrame::EndLink || input.tcp_offset == [0.0; 3] {
        let target_end = goal_to_end(goal_in_frame, input);
        return input
            .arm
            .solve_dls_ex(&target_end, q_seed, lo, hi, kinematics::PAR.w, yaw_free);
    }
    if !yaw_free {
        let target_end = tcp_to_end(goal_in_frame, &input.tcp_offset);
        let mut sol = input
            .arm
            .solve_dls_ex(&target_end, q_seed, lo, hi, kinematics::PAR.w, false);
        let ach_tcp = end_to_tcp(&input.arm.fk(&sol.q), &input.tcp_offset);
        let tcp_ep = kinematics::norm(&kinematics::sub(&goal_in_frame.p, &ach_tcp.p));
        sol.ep = sol.ep.max(tcp_ep);
        sol.ok = sol.ok && tcp_ep <= kinematics::PAR.tol_pos;
        return sol;
    }
    // 5-DOF auto_yaw: yaw is coupled to J1, so refine target_end using achieved R_end(q)
    // and verify the achieved TCP residual <= tol_pos.
    let mut r_est = goal_in_frame.r;
    let mut q_cur = *q_seed;
    let mut best_sol: Option<kinematics::IkResult> = None;
    let mut total_iters = 0usize;
    for _ in 0..6 {
        let target_end = Pose::new(
            goal_in_frame.r,
            kinematics::sub(
                &goal_in_frame.p,
                &kinematics::mat_vec(&r_est, &input.tcp_offset),
            ),
        );
        let mut sol = input
            .arm
            .solve_dls_ex(&target_end, &q_cur, lo, hi, kinematics::PAR.w, true);
        total_iters += sol.iterations;
        let ach_end = input.arm.fk(&sol.q);
        let ach_tcp = end_to_tcp(&ach_end, &input.tcp_offset);
        let tcp_ep = kinematics::norm(&kinematics::sub(&goal_in_frame.p, &ach_tcp.p));
        sol.ep = sol.ep.max(tcp_ep);
        sol.ok = sol.ok && tcp_ep <= kinematics::PAR.tol_pos;
        sol.iterations = total_iters;
        if sol.ok {
            return sol;
        }
        q_cur = sol.q;
        r_est = ach_end.r;
        best_sol = Some(sol);
    }
    best_sol.expect("at least one IK iteration")
}

#[derive(Debug, Clone, PartialEq)]
pub struct JointPath {
    pub dof: usize,
    pub arm_dof: usize,
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
    plan_cartesian_opts(input, goal, speed, PathOpts { yaw_free, yaw_outward: false })
}

/// Plan a Cartesian path from `fk(q0)` to `goal` with [`PathOpts`].
///
/// With `yaw_outward` the orientation is interpolated in the outward frame: with
/// ψ(s) = atan2(y, x) of sample s (unwrapped along the line), R(s) = Rz(ψ(s)) · Rrel(s), where
/// Rrel slerps from Rz(−ψ(0)) · R_start to Rz(−ψ(1)) · R_goal. A start already at the outward yaw
/// keeps its yaw at atan2(y, x) of every sample. A path passing within
/// [`OUTWARD_MIN_RADIUS_M`] of the base z axis is `INVALID_COMMAND`.
pub fn plan_cartesian_opts(input: &PlanInput, goal: Pose, speed: Speed, opts: PathOpts) -> Result<JointPath, MotionError> {
    let yaw_free = opts.yaw_free;
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
    let start = start_pose(input);
    let dp = kinematics::sub(&goal.p, &start.p);
    let length = kinematics::norm(&dp);
    // outward: (ψ(0), Δψ over the path, Rrel(0)); rotations below are then in the outward frame
    let outward = if opts.yaw_outward {
        let d = segment_axis_distance(&start.p, &goal.p);
        if d < OUTWARD_MIN_RADIUS_M {
            return Err(MotionError::InvalidCommand(format!(
                "yaw \"outward\" is undefined near the base z axis: the path passes {d:.3} m from it (minimum {OUTWARD_MIN_RADIUS_M} m)")));
        }
        let psi0 = start.p[1].atan2(start.p[0]);
        Some((psi0, azimuth_delta(&start.p, &goal.p), kinematics::mat_mul(&kinematics::axis_r(&[0.0, 0.0, 1.0], -psi0), &start.r)))
    } else { None };
    let (r_from, r_to) = match outward {
        Some((psi0, dpsi, rel0)) => (rel0, kinematics::mat_mul(&kinematics::axis_r(&[0.0, 0.0, 1.0], -(psi0 + dpsi)), &goal.r)),
        None => (start.r, goal.r),
    };
    let w_rel = kinematics::log_v(&kinematics::mul_t(&r_to, &r_from));
    let theta = kinematics::norm(&w_rel) + outward.map_or(0.0, |(_, dpsi, _)| dpsi.abs());
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
    let ws_eval = input.ws.as_ref().map(|ws| crate::chain::WorkspaceEvaluator::new_with_tcp(&input.arm, ws, &input.cam, input.tcp_offset));
    let mut ws_state = match &ws_eval {
        Some(ev) => {
            let mut c0 = ev.alloc_buf();
            ev.eval_into(&start_q32, &mut c0, "plan").map_err(|f| MotionError::WorkspaceViolation {
                s: 0.0,
                point: f.point,
                limit: f.limit,
                value: f.value,
                bound: f.bound,
                detail: f.detail,
            })?;
            let min_d = ev.init_min_depths(&c0);
            Some((c0, min_d, MAX_CARTESIAN_SWEEP_EVALS))
        }
        None => None,
    };

    let mut q_samples = Vec::with_capacity(n + 1);
    let mut s_samples = Vec::with_capacity(n + 1);
    q_samples.push(start_q32);
    s_samples.push(0.0);
    let mut prev = input.q0;
    let mut prev_q32 = start_q32;
    // largest yaw offset from outward over the start and the samples so far (IK failure hint)
    let mut yaw_off_deg = outward_hint(arm_dof, opts, &start.p, &start.r);
    for k in 1..=n {
        let s = k as f64 / n as f64;
        let p = [start.p[0] + dp[0] * s, start.p[1] + dp[1] * s, start.p[2] + dp[2] * s];
        let r = if k == n { goal.r } else {
            let r_s = kinematics::mat_mul(&kinematics::exp_r(&[w_rel[0] * s, w_rel[1] * s, w_rel[2] * s]), &r_from);
            match outward {
                Some((psi0, _, _)) => kinematics::mat_mul(&kinematics::axis_r(&[0.0, 0.0, 1.0], psi0 + azimuth_delta(&start.p, &p)), &r_s),
                None => r_s,
            }
        };
        if let Some(off) = outward_hint(arm_dof, opts, &p, &r) {
            yaw_off_deg = Some(yaw_off_deg.map_or(off, |m| m.max(off)));
        }
        let sol = solve_ik_in_frame(input, &Pose::new(r, p), &prev, &lo, &hi, yaw_free);
        if !sol.ok {
            return Err(MotionError::IkUnreachable { s, ep: sol.ep, er: sol.er, yaw_off_deg });
        }
        if let Some((joint, dq)) = (0..arm_dof).map(|i| (i, (sol.q[i] - prev[i]).abs())).max_by(|a, b| a.1.total_cmp(&b.1)) {
            if dq > MAX_SAMPLE_JUMP_RAD {
                return Err(MotionError::IkBranchFlip { s, joint: joint + 1, dq, yaw_off_deg });
            }
        }
        let q32: [f32; 7] = std::array::from_fn(|i| if i < arm_dof { sol.q[i] as f32 } else if i == arm_dof { input.j7 } else { 0.0 });
        if let (Some(ev), Some((c_prev, min_d, rem))) = (&ws_eval, &mut ws_state) {
            match ev.sweep_linear_step(
                &prev_q32,
                &q32,
                c_prev,
                min_d,
                rem,
                MAX_CARTESIAN_SWEEP_EVALS,
                "plan",
                "sweep",
            ) {
                Ok(c_next) => *c_prev = c_next,
                Err((t_seg, f)) => {
                    let s_fail = ((k - 1) as f64 + t_seg) / n as f64;
                    return Err(MotionError::WorkspaceViolation {
                        s: s_fail,
                        point: f.point,
                        limit: f.limit,
                        value: f.value,
                        bound: f.bound,
                        detail: f.detail,
                    });
                }
            }
        }
        q_samples.push(q32);
        s_samples.push(s);
        prev = sol.q;
        prev_q32 = q32;
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
    let arm_dof = input.arm.arm_dof();
    let mut path = JointPath { dof, arm_dof, q: q_samples, s: s_samples, duration, power_scale: 1.0 };
    if let Some(b) = input.power.as_ref().filter(|b| b.enabled && b.planning.enabled) {
        #[cfg(not(target_arch = "wasm32"))]
        let t0 = std::time::Instant::now();
        #[cfg(not(target_arch = "wasm32"))]
        let fit = path.fit_power(b, &input.arm)?;
        #[cfg(target_arch = "wasm32")]
        let _fit = path.fit_power(b, &input.arm)?;
        #[cfg(not(target_arch = "wasm32"))]
        tracing::debug!("Power planning: scale {:.3}, peak {:.2} A (full speed {:.2} A, limit {:.2} A), {} samples, {} µs",
            fit.scale, fit.peak_a, fit.peak_full_a, fit.limit_a, fit.samples, t0.elapsed().as_micros());
    }
    Ok(path)
}

// ---- joint-space path to a Cartesian goal (`path: "joint"`) ----

/// Path of `move_to` / `move_relative`: `"linear"` (default) plans the straight line of
/// [`plan_cartesian_opts`]; `"joint"` solves the IK for the goal only and moves with the
/// joint-space quintic of `move_to_pose` (the TCP path is then not a straight line).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PathMode { #[default] Linear, Joint }

/// Optional `path`: `"linear"` | `"joint"`; absent or null → linear.
pub fn parse_path_mode(body: &Value) -> Result<PathMode, MotionError> {
    match body.get("path") {
        None | Some(Value::Null) => Ok(PathMode::Linear),
        Some(Value::String(s)) if s == "linear" => Ok(PathMode::Linear),
        Some(Value::String(s)) if s == "joint" => Ok(PathMode::Joint),
        Some(_) => Err(MotionError::InvalidCommand("path must be \"linear\", \"joint\" or null".into())),
    }
}

/// Appended to the message of a straight-line `IK_BRANCH_FLIP` when the joint-space move to the
/// same goal has an IK solution and passes the swept workspace check.
pub const JOINT_PATH_HINT: &str = "; a joint-space move reaches it: try path: \"joint\"";

/// DLS rounds of [`solve_goal_joints`] per seed ([`kinematics::PAR`] iterations each).
pub const GOAL_IK_ROUNDS: usize = 40;

/// The seeds of [`solve_goal_joints`] besides the current joints: the current joints with J1
/// turned by the azimuth change from the current TCP to the goal, and each of `extra` (home,
/// rest) with J1 turned by the azimuth change from its own TCP to the goal.
pub fn goal_seeds(input: &PlanInput, goal: &Pose, extra: &[[f64; 6]]) -> Vec<[f64; 6]> {
    let turned = |q: &[f64; 6]| -> [f64; 6] {
        let p = end_to_frame(&input.arm.fk(q), input).p;
        let mut s = *q;
        if p[0].hypot(p[1]) >= OUTWARD_MIN_RADIUS_M && goal.p[0].hypot(goal.p[1]) >= OUTWARD_MIN_RADIUS_M {
            s[0] = wrap_pi(q[0] + azimuth_delta(&p, &goal.p));
        }
        s
    };
    let mut seeds = vec![input.q0, turned(&input.q0)];
    for q in extra { seeds.push(*q); seeds.push(turned(q)); }
    seeds
}

/// IK of `goal` for a joint-space move: solved from each of `seeds` (the first one should be the
/// current joints, see [`goal_seeds`]); a solution counts when it reaches the goal within the IK
/// tolerance, lies within the joint limits and (with a workspace policy) passes the workspace
/// rule at the goal. Of those, the one with the smallest Σ |q − q0| / max_velocity over the arm
/// joints wins. None: the workspace violation of the first solution within the joint limits
/// that failed the workspace rule, else [`MotionError::IkGoalUnreachable`] with the best errors seen.
pub fn solve_goal_joints(input: &PlanInput, goal: &Pose, opts: PathOpts, seeds: &[[f64; 6]]) -> Result<[f64; 6], MotionError> {
    let arm_dof = input.arm.arm_dof();
    if input.limits.len() < arm_dof + 1 {
        return Err(MotionError::Unsupported(format!("Cartesian actions need at least {} joints", arm_dof + 1)));
    }
    if !goal.p.iter().chain(goal.r.iter().flatten()).all(|v| v.is_finite()) {
        return Err(MotionError::InvalidCommand("goal pose must be finite".into()));
    }
    if opts.yaw_outward && goal.p[0].hypot(goal.p[1]) < OUTWARD_MIN_RADIUS_M {
        return Err(MotionError::InvalidCommand(format!(
            "yaw \"outward\" is undefined near the base z axis: the goal is {:.3} m from it (minimum {OUTWARD_MIN_RADIUS_M} m)",
            goal.p[0].hypot(goal.p[1]))));
    }
    let lim = |i: usize| input.limits.get(i).filter(|_| i < arm_dof);
    let lo: [f64; 6] = std::array::from_fn(|i| lim(i).map_or(-std::f64::consts::PI, |l| f64::from(l.min_position) + IK_LIMIT_MARGIN));
    let hi: [f64; 6] = std::array::from_fn(|i| lim(i).map_or(std::f64::consts::PI, |l| f64::from(l.max_position) - IK_LIMIT_MARGIN));
    // workspace rule at the goal as for a path sample: no constraint deeper than at the start
    let ws_eval = input.ws.as_ref().map(|ws| crate::chain::WorkspaceEvaluator::new_with_tcp(&input.arm, ws, &input.cam, input.tcp_offset));
    let mut ws_state = match &ws_eval {
        Some(ev) => {
            let start_q32: [f32; 7] = std::array::from_fn(|i| if i < arm_dof { input.q0[i] as f32 } else if i == arm_dof { input.j7 } else { 0.0 });
            let mut c0 = ev.alloc_buf();
            ev.eval_into(&start_q32, &mut c0, "plan").map_err(|f| MotionError::WorkspaceViolation {
                s: 0.0, point: f.point, limit: f.limit, value: f.value, bound: f.bound, detail: f.detail })?;
            Some((ev.init_min_depths(&c0), c0))
        }
        None => None,
    };
    let cost = |q: &[f64; 6]| -> f64 {
        (0..arm_dof).map(|i| {
            let v = f64::from(input.limits[i].max_velocity);
            (q[i] - input.q0[i]).abs() / if v > 0.0 { v } else { 1.0 }
        }).sum()
    };
    let mut best: Option<([f64; 6], f64)> = None;
    let (mut best_ep, mut best_er) = (f64::INFINITY, f64::INFINITY);
    let mut ws_failure: Option<MotionError> = None;
    for seed in seeds {
        // the seed itself within the search limits (a seed past them would widen them)
        let mut q: [f64; 6] = std::array::from_fn(|i| if i < arm_dof { seed[i].clamp(lo[i], hi[i]) } else { seed[i] });
        let mut sol = solve_ik_in_frame(input, goal, &q, &lo, &hi, opts.yaw_free);
        for _ in 1..GOAL_IK_ROUNDS {
            if sol.ok { break; }
            let before = sol.ep + kinematics::PAR.w * sol.er;
            q = sol.q;
            let next = solve_ik_in_frame(input, goal, &q, &lo, &hi, opts.yaw_free);
            let stalled = next.ep + kinematics::PAR.w * next.er > before - 1e-9;
            sol = next;
            if stalled && !sol.ok { break; }
        }
        if sol.ep + sol.er < best_ep + best_er { best_ep = sol.ep; best_er = sol.er; }
        if !sol.ok { continue; }
        let within = (0..arm_dof).all(|i| {
            let l = &input.limits[i];
            sol.q[i].is_finite() && sol.q[i] >= f64::from(l.min_position) && sol.q[i] <= f64::from(l.max_position)
        });
        if !within { continue; }
        if let (Some(ev), Some((min_d, buf))) = (&ws_eval, ws_state.as_mut()) {
            let q32: [f32; 7] = std::array::from_fn(|i| if i < arm_dof { sol.q[i] as f32 } else if i == arm_dof { input.j7 } else { 0.0 });
            if let Err(f) = ev.eval_into(&q32, buf, "plan").and_then(|()| ev.check_readonly(buf, min_d, "plan")) {
                ws_failure.get_or_insert(MotionError::WorkspaceViolation { s: 1.0, point: f.point, limit: f.limit, value: f.value, bound: f.bound, detail: f.detail });
                continue;
            }
        }
        let c = cost(&sol.q);
        if best.is_none_or(|(_, bc)| c < bc) { best = Some((sol.q, c)); }
    }
    match (best, ws_failure) {
        (Some((q, _)), _) => Ok(q),
        (None, Some(e)) => Err(e),
        (None, None) => Err(MotionError::IkGoalUnreachable { ep: best_ep, er: best_er, seeds: seeds.len() }),
    }
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
    /// `yaw: "outward"`: the inner goal with its rpy yaw replaced by atan2(y, x) of the goal
    /// position (roll and pitch kept), and a path whose yaw follows atan2(y, x).
    YawOutward(Box<GoalSpec>),
}

/// Resolve a goal request against the start pose `fk(q0)`.
pub fn resolve_goal(input: &PlanInput, spec: &GoalSpec) -> Pose {
    resolve_goal_ex(input, spec).0
}

/// Apply auto-yaw rotation about world Z for 5-DOF arms based on radial azimuth change.
pub fn apply_auto_yaw(start_p: &[f64; 3], goal_p: &[f64; 3], mut target_r: kinematics::Mat3) -> kinematics::Mat3 {
    let r_start = (start_p[0] * start_p[0] + start_p[1] * start_p[1]).sqrt();
    let r_target = (goal_p[0] * goal_p[0] + goal_p[1] * goal_p[1]).sqrt();
    if r_start >= 0.03 && r_target >= 0.03 && !crate::chain::is_tool_vertical(&target_r) {
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
    let (pose, opts) = resolve_goal_opts(input, spec);
    (pose, opts.yaw_free)
}

/// Resolve a goal request against the start pose `fk(q0)`, returning the pose and the path options
/// for [`plan_cartesian_opts`].
pub fn resolve_goal_opts(input: &PlanInput, spec: &GoalSpec) -> (Pose, PathOpts) {
    if let GoalSpec::YawOutward(inner) = spec {
        let (mut pose, opts) = resolve_goal_opts(input, inner);
        pose.r = with_outward_yaw(&pose.p, &pose.r);
        return (pose, PathOpts { yaw_outward: true, ..opts });
    }
    let (pose, yaw_free) = resolve_goal_base(input, spec);
    (pose, yaw_free.into())
}

fn resolve_goal_base(input: &PlanInput, spec: &GoalSpec) -> (Pose, bool) {
    let start_end = input.arm.fk(&input.q0);
    let start = end_to_frame(&start_end, input);
    match spec {
        GoalSpec::YawOutward(_) => unreachable!("handled by resolve_goal_opts"),
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
                    let rec = input.arm.r_end_cam(&input.cam, f64::from(input.cam.tilt_deg));
                    let rc = kinematics::cam_rotation(&start_end, &rec);
                    let p = kinematics::add(&start.p, &kinematics::mat_vec(&rc, d));
                    let mut r = start_end.r;
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
    Ok((outward_spec(spec, parse_yaw_mode(body)?), parse_speed(body)?, tag(body)?))
}

/// Optional `yaw`: `"outward"` → true; absent or null → false.
pub fn parse_yaw_mode(body: &Value) -> Result<bool, MotionError> {
    match body.get("yaw") {
        None | Some(Value::Null) => Ok(false),
        Some(Value::String(s)) if s == "outward" => Ok(true),
        Some(_) => Err(MotionError::InvalidCommand("yaw must be \"outward\" or null".into())),
    }
}

fn outward_spec(spec: GoalSpec, outward: bool) -> GoalSpec {
    if outward { GoalSpec::YawOutward(Box::new(spec)) } else { spec }
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
    let outward = parse_yaw_mode(body)?;
    if outward && dyaw != 0.0 {
        return Err(MotionError::InvalidCommand("dyaw cannot be combined with yaw \"outward\"".into()));
    }
    Ok((outward_spec(GoalSpec::Relative { frame, d, dyaw, dpitch, droll, level, auto_yaw }, outward), parse_speed(body)?, tag(body)?))
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
    /// The stall latch was set by the gripper torque cap (`gripper_policy.max_close_torque_nm`).
    pub gripper_force_capped: bool,
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
/// Grasp contact: consecutive ticks of torque rise or following error.
pub const GRIPPER_CONTACT_TICKS: u32 = 2;
/// Grasp contact from the following error: command at least this far ahead of the jaw (rad) ...
pub const GRIPPER_CONTACT_LAG_RAD: f32 = 0.30;
/// ... while the jaw closes slower than this fraction of the requested speed.
pub const GRIPPER_CONTACT_SLOW_FRACTION: f32 = 0.3;
/// J7 speed (rad/s) from contact to the contact stop.
pub const GRIPPER_CONTACT_SPEED: f32 = 0.3;
/// A grasp squeeze above the gripper torque cap is reduced to aim at this fraction of the cap.
pub const SQUEEZE_CAP_FRACTION: f32 = 0.8;
/// Squeeze after a torque cap trip: press in from the measured position, reaching squeeze_rad
/// in half the squeeze window (settle_ms), until |τ| reaches this fraction of the cap (or squeeze_rad past the measured position).
pub const SQUEEZE_RAMP_FRACTION: f32 = 0.6;
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
        let arm_joints = self.path.arm_dof.min(dof);
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

#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, PartialEq)]
enum GripPhase {
    Moving,
    /// Squeezing an object: hold target, samples of |τ_gripper| and |load_gripper|.
    /// `ramp`: the squeeze started at the measured position after a torque cap trip and
    /// presses in (squeeze_rad over half the window) until the torque reaches SQUEEZE_RAMP_FRACTION of the cap.
    Squeeze { hold: f32, elapsed: f64, taus: Vec<f32>, loads: Vec<f32>, ramp: bool },
}

#[cfg(not(target_arch = "wasm32"))]
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
    /// Grasp: measured J7 where contact was detected; the command then stops at
    /// `contact + squeeze_rad` (never past it) and closes at GRIPPER_CONTACT_SPEED. Cleared
    /// again when the contact gave way (`contact_yielded`).
    pub contact_rad: Option<f32>,
    contact_ticks: u32,
}

#[cfg(not(target_arch = "wasm32"))]
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
        let mut m = Self { request, policy, limit, target, start, gripper_index, elapsed: 0.0, timeout_s: 0.0, still_for: 0.0, moved: false, phase: GripPhase::Moving, contact_rad: None, contact_ticks: 0 };
        m.timeout_s = m.request.timeout_s
            .unwrap_or_else(|| GRIPPER_DEFAULT_TIMEOUT_S.max(m.expected_duration() + GRIPPER_TIMEOUT_MARGIN_S));
        m
    }

    /// Expected duration (s) for the travel at the requested speed, plus settle.
    pub fn expected_duration(&self) -> f64 {
        f64::from((self.target - self.start).abs()) / self.request.speed + 0.5 + self.policy.settle_ms as f64 / 1000.0
    }

    /// Grasp: contact + squeeze_rad toward closed (not past closed_rad), once contact is known.
    pub fn contact_stop(&self) -> Option<f32> {
        let dir = self.close_dir();
        self.contact_rad.map(|c| {
            let stop = c + dir * self.policy.squeeze_rad;
            if (stop - self.policy.closed_rad) * dir > 0.0 { self.policy.closed_rad } else { stop }
        })
    }

    fn close_dir(&self) -> f32 { if self.policy.closed_rad >= self.policy.open_rad { 1.0 } else { -1.0 } }

    /// Grasp: the jaw (measured `m_g`) went on past the contact by at least half of squeeze_rad
    /// and the torque is below contact_torque_nm (holding_torque_nm when unset): the object gave
    /// way. False without a contact or when the stop is already closed_rad.
    fn contact_yielded(&self, m_g: f32, tau_g: f32, torque_feedback: bool) -> bool {
        let (Some(contact), Some(stop)) = (self.contact_rad, self.contact_stop()) else { return false };
        let dir = self.close_dir();
        let low_torque = !torque_feedback
            || tau_g.abs() < self.policy.contact_torque_nm.unwrap_or(self.policy.holding_torque_nm);
        (self.policy.closed_rad - stop) * dir > 1e-4
            && (m_g - contact) * dir >= 0.5 * self.policy.squeeze_rad
            && low_torque
    }

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

        let dir = self.close_dir();
        let stop = self.contact_stop();
        let active_target = stop.unwrap_or(self.target);
        let closing_phase = matches!(self.phase, GripPhase::Squeeze { .. }) || (active_target - q_g) * dir > 1e-4;
        if self.policy.max_close_torque_nm.is_some() && closing_phase && (!ctx.torque_feedback || !ctx.feedback_fresh) {
            let hold = if m_g.is_finite() { m_g } else { q_g };
            if idx < positions.len() { positions[idx] = hold; }
            if idx < velocities.len() { velocities[idx] = 0.0; }
            return StepOut {
                positions,
                velocities,
                progress: progress_of(hold),
                clear_gripper_stall: false,
                status: StepStatus::Failed {
                    code: "FEEDBACK_UNAVAILABLE",
                    message: "Gripper close stopped: torque feedback unavailable or stale while max_close_torque_nm is configured".into(),
                },
            };
        }
        let force_cap = self.policy.max_close_torque_nm.filter(|_| ctx.torque_feedback && ctx.feedback_fresh);
        let squeeze_rad = self.policy.squeeze_rad;
        let ramp_rate = 2.0 * f64::from(squeeze_rad) / (self.policy.settle_ms as f64 / 1000.0);
        let contact_stop = self.contact_stop();
        let yielded = self.contact_yielded(m_g, tau_g, ctx.torque_feedback);
        let mut rearm = false;
        if let GripPhase::Squeeze { hold, elapsed, taus, loads, ramp } = &mut self.phase {
            *elapsed += ctx.dt;
            taus.push(tau_g.abs());
            loads.push(ctx.loads.get(idx).copied().unwrap_or(0.0).abs());
            // torque cap: back the squeeze off toward the measured position so the grip
            // torque settles near SQUEEZE_CAP_FRACTION of the cap (position-proportional grip)
            if let Some(cap) = force_cap {
                let press = (*hold - m_g) * dir;
                if tau_g.abs() > cap {
                    if press > 0.0 { *hold = m_g + dir * press * (SQUEEZE_CAP_FRACTION * cap / tau_g.abs()); }
                    *ramp = false;
                } else if *ramp && tau_g.abs() < SQUEEZE_RAMP_FRACTION * cap && press < squeeze_rad {
                    *hold += dir * ((ramp_rate * ctx.dt) as f32).min(squeeze_rad - press);
                }
            }
            if idx < positions.len() { positions[idx] = *hold; }
            let window_done = *elapsed * 1000.0 >= self.policy.settle_ms as f64 - 1e-6;
            // The contact gave way: the squeeze, limited to the contact stop, ends with the jaw
            // past the contact and little torque, so the object turned or slid in the jaws (a box
            // closed on across its corners squares up) and the hold would grip with about 0 N·m.
            rearm = window_done && yielded && contact_stop.is_some_and(|stop| (*hold - stop).abs() < 1e-4);
        }
        if rearm {
            // forget the contact and close on at GRIPPER_CONTACT_SPEED from the measured jaw until
            // the next torque rise or following error sets a new contact stop
            self.phase = GripPhase::Moving;
            self.contact_rad = None;
            self.contact_ticks = 0;
            self.still_for = 0.0;
            if idx < positions.len() { positions[idx] = m_g; }
            return StepOut { positions, velocities, status: StepStatus::Running, progress: progress_of(m_g), clear_gripper_stall: true };
        }
        if let GripPhase::Squeeze { elapsed, taus, loads, .. } = &mut self.phase {
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

        let grasping = self.request.command == GripperCommand::Close && self.request.grasp;
        // Grasp contact: torque rise (contact_torque_nm, once the jaw has moved) or following
        // error (command ahead while the jaw slows down) for GRIPPER_CONTACT_TICKS ticks.
        if grasping && self.contact_rad.is_none() && ctx.feedback_fresh && !stalled {
            let torque_hit = ctx.torque_feedback && self.moved
                && self.policy.contact_torque_nm.is_some_and(|c| tau_g.abs() >= c);
            let lag_hit = (q_g - m_g) * dir >= GRIPPER_CONTACT_LAG_RAD
                && mv_g * dir < GRIPPER_CONTACT_SLOW_FRACTION * self.request.speed as f32;
            self.contact_ticks = if torque_hit || lag_hit { self.contact_ticks + 1 } else { 0 };
            if self.contact_ticks >= GRIPPER_CONTACT_TICKS {
                self.contact_rad = Some(m_g);
                self.limit.max_velocity = self.limit.max_velocity.min(GRIPPER_CONTACT_SPEED);
            }
        }
        let stop = self.contact_stop();
        let target = stop.unwrap_or(self.target);
        // Step gripper toward the target unless the stall latch holds it.
        if !stalled && stop.is_some_and(|stop| (q_g - stop) * dir > 0.0) {
            // the command ran ahead of the contact stop: take it back there
            if idx < positions.len() { positions[idx] = target; }
            if idx < velocities.len() { velocities[idx] = 0.0; }
        } else if !stalled {
            match crate::supervisor::SupervisorInner::bounded_servo_step(q_g, target, v_gc, ctx.dt, &self.limit) {
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
        let cmd_at_target = idx < positions.len() && (positions[idx] - target).abs() < 1e-4;
        if mv_g.abs() < SETTLE_VEL as f32 && (self.moved || cmd_at_target) { self.still_for += ctx.dt; } else { self.still_for = 0.0; }
        let still = self.still_for >= SETTLE_HOLD_S - 1e-9;

        let status = if grasping {
            if stalled || still {
                // after a torque cap trip the jaw already presses harder than the cap: hold the
                // measured position (the latch's hold), no squeeze past it
                let capped = stalled && ctx.gripper_force_capped;
                let squeeze = if capped { 0.0 } else { self.policy.squeeze_rad };
                let squeezed = m_g + dir * squeeze;
                // never past the contact stop
                let squeezed = self.contact_stop().map_or(squeezed, |stop| if (squeezed - stop) * dir > 0.0 { stop } else { squeezed });
                let hold = if dir > 0.0 { squeezed.min(self.policy.closed_rad) } else { squeezed.max(self.policy.closed_rad) }
                    .clamp(self.limit.min_position, self.limit.max_position);
                if idx < positions.len() { positions[idx] = hold; }
                if idx < velocities.len() { velocities[idx] = 0.0; }
                let ramp = capped && self.policy.max_close_torque_nm.is_some();
                self.phase = GripPhase::Squeeze { hold, elapsed: 0.0, taus: Vec::new(), loads: Vec::new(), ramp };
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
    #[cfg(not(target_arch = "wasm32"))]
    Gripper(Box<GripperMotion>),
    /// Episode replay (`crate::replay`).
    #[cfg(not(target_arch = "wasm32"))]
    Replay(Box<crate::replay::ReplayMotion>),
}

#[derive(Debug, Clone)]
pub struct ActiveTrajectory {
    pub action_id: Option<u64>,
    pub reference: Reference,
    /// Same-side approach (`src/backlash.rs`): the short joint-space segment that replaces
    /// `reference` when it reaches its end (a quintic or a path, before its settle).
    pub then: Option<MultiJointQuinticTrajectory>,
    /// Progress mapping while a two-leg motion runs: (seconds before this leg, total seconds).
    pub span: Option<(f64, f64)>,
    /// Per-joint velocity/acceleration limits used when planning this trajectory (e.g. after `speed_scale`).
    pub motion_limits: Option<Vec<JointLimit>>,
}

impl ActiveTrajectory {
    pub fn new(action_id: Option<u64>, reference: Reference) -> Self {
        Self { action_id, reference, then: None, span: None, motion_limits: None }
    }
    pub fn quintic(traj: MultiJointQuinticTrajectory, action_id: Option<u64>) -> Self {
        Self::new(action_id, Reference::Quintic(traj))
    }
    pub fn with_motion_limits(mut self, limits: Vec<JointLimit>) -> Self {
        self.motion_limits = Some(limits);
        self
    }
    /// `reference` followed by the short segment `then` (same-side approach).
    pub fn with_approach(action_id: Option<u64>, reference: Reference, then: MultiJointQuinticTrajectory) -> Self {
        let mut a = Self::new(action_id, reference);
        let total = a.duration() + then.duration;
        a.span = Some((0.0, total));
        a.then = Some(then);
        a
    }
    /// Action progress of this leg's `progress` (0..1) over the whole motion.
    pub fn overall_progress(&self, progress: f32) -> f32 {
        match self.span {
            Some((before, total)) if total > 0.0 => ((before + f64::from(progress) * self.duration()) / total).clamp(0.0, 1.0) as f32,
            _ => progress,
        }
    }
    /// Replace an ended first leg by the short segment; true when it did.
    pub fn start_then(&mut self) -> bool {
        let Some(then) = self.then.take() else { return false };
        let before = self.duration();
        let total = self.span.map_or(before + then.duration, |(_, t)| t);
        self.reference = Reference::Quintic(then);
        self.span = Some((before, total));
        true
    }
    /// The first leg of a two-leg motion reached its end.
    pub fn first_leg_ended(&self) -> bool {
        self.then.is_some() && (matches!(&self.reference, Reference::Quintic(t) if t.is_finished())
            || matches!(&self.reference, Reference::Path(p) if p.settle.is_some()))
    }
    /// Planned duration (s); for a gripper motion the expected duration.
    pub fn duration(&self) -> f64 {
        match &self.reference {
            Reference::Quintic(t) => t.duration,
            Reference::Path(p) => p.duration(),
            #[cfg(not(target_arch = "wasm32"))]
            Reference::Gripper(g) => g.expected_duration(),
            #[cfg(not(target_arch = "wasm32"))]
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
    /// move_to | move_relative | gripper | grasp | place | joint_move | home | park | move_to_pose | replay | sequence
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
    /// Grasp: the request's task (`GraspSetup.tasks`); absent without one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub task: Option<String>,
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
    /// Sequence only (`run_sequence`): the sequence, the running step (index in the sequence)
    /// and its type, the repeat (0-based) and the number of repeats.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sequence_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub step_index: Option<usize>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub step_type: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub repeat_index: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub repeats: Option<u32>,
    /// Sequence that ended before finishing: index of the step that was running.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub failed_step: Option<usize>,
    /// Sequence with `record`: the episodes saved so far (one per finished repeat).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub episodes: Vec<String>,
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
    /// Grasp: every evaluated height source in evaluation order (`height_sources`,
    /// `height_record`), with its value, reject reason or veto code, and whether it decided.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub height_estimates: Vec<EstimateEntry>,
    /// Grasp: every evaluated J6 yaw source (`yaw_sources`, `yaw_record`); value = the object's
    /// long side in the image (deg).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub yaw_estimates: Vec<EstimateEntry>,
    /// Grasp: every evaluated width source (`width_sources`, `width_record`); value = width (m).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub width_estimates: Vec<EstimateEntry>,
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
    /// Final approach of move_to_pose / grasp / place / precise move_to (`src/backlash.rs`):
    /// `same_side` or `direct`; absent when the profile has the approach off for this motion
    /// and the request did not ask for it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub approach: Option<String>,
    /// Why the motion approached directly (`request`, `contact`, a failed check of the
    /// pre-target or the short segment).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub approach_reason: Option<String>,
    /// same_side: direction of each arm joint on the short segment (+1 = moving in +, -1 = in -,
    /// 0 = not listed).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub approach_signs: Option<Vec<i8>>,
    /// same_side, grasp / place: joints whose sign the from-above rule flipped.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub approach_flipped: Option<Vec<String>>,
    /// Accumulated runtime per macro phase (ms).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub phase_ms: BTreeMap<String, u32>,
    /// Execution diagnostics for each arm sub-motion of the macro action.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub motions: Vec<MacroMotionDiag>,
    /// Structured workspace violation details when failed with `WORKSPACE_VIOLATION`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub constraint_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub geometry_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub violation_phase: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
    /// Persistence durability status when an action updates persisted state (`gripper_zero_check`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub durability_confirmed: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub durability_warning: Option<String>,
    /// Per-step timing of a `policy` action, while running and in the final result.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub policy_stats: Option<PolicyStats>,
    /// `measure_multiview`: the measurement (`method`, `objects`, `table_check`, `views`, `why`;
    /// src/measure_multiview.rs), set when the action ends after its captures.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub measurement: Option<serde_json::Value>,
}

/// `ActionStatus::policy_stats`: inferences done (`steps`), steps whose inference + commit
/// took longer than the period (`overruns`), periods without a committed target because the
/// previous inference had not finished (`skipped`), inference time over the last 256 steps,
/// and the run's period.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(default)]
pub struct PolicyStats {
    pub steps: u64,
    pub overruns: u64,
    pub skipped: u64,
    pub infer_us: PolicyInferUs,
    pub period_us: u64,
}

/// Inference time percentiles in microseconds.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(default)]
pub struct PolicyInferUs {
    pub p50: u64,
    pub p99: u64,
    pub max: u64,
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

/// One evaluated source, reported in `ActionStatus.height_estimates` / `yaw_estimates` /
/// `width_estimates`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EstimateEntry {
    /// Height: `depth:<anchor>`, `depth`, `scale`, `parallax`, `probe`, `prior`.
    /// Yaw: `mask`, `depth`, `box`. Width: `mask`, `depth`.
    pub source: String,
    /// The value the source measured, also when rejected or vetoed: height above the table (m),
    /// yaw = the object's long side in the image (deg, [-90, 90)), width across the jaws (m).
    pub value: Option<f32>,
    pub used: bool,
    pub record_only: bool,
    /// Reject reason or veto code.
    pub reason: Option<String>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kinematics;
    use crate::model::ArmConfig;

    fn inspect_input_with_tcp(tcp_offset: [f64; 3]) -> PlanInput {
        let mut config = ArmConfig::rebot_dm();
        config.tcp_offset = tcp_offset;
        config.rebuild_effective_check_points().unwrap();
        let q0 = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0];
        PlanInput {
            epoch: 1,
            workspace_revision: 0,
            q0,
            j7: 0.0,
            limits: config.limits.clone(),
            speed_cap: config.speed_cap,
            arm: config.arm_model.clone(),
            ws: config.workspace.clone(),
            cam: config.camera_mount.clone(),
            power: None,
            tcp_offset,
            goal_frame: GoalFrame::Tcp,
        }
    }

    #[test]
    fn stationary_tcp_pitch_rotation_keeps_tcp_position_invariant() {
        let tcp_offset = [0.04, 0.0, 0.0];
        let input = inspect_input_with_tcp(tcp_offset);
        let start_tcp = start_pose(&input);
        let start_end = input.arm.fk(&input.q0);
        let dr = kinematics::rpy_to_r(0.0, 0.25, 0.0);
        let goal_tcp = Pose::new(kinematics::mat_mul(&start_tcp.r, &dr), start_tcp.p);
        let path = plan_cartesian_opts(
            &input,
            goal_tcp,
            Speed::default(),
            PathOpts::default(),
        )
        .expect("stationary TCP pitch rotation must succeed");
        assert!(path.q.len() >= 3);
        for (i, q_f32) in path.q.iter().enumerate() {
            let q: [f64; 6] = std::array::from_fn(|j| f64::from(q_f32[j]));
            let p_tcp = end_to_tcp(&input.arm.fk(&q), &tcp_offset).p;
            let err = kinematics::norm(&kinematics::sub(&p_tcp, &start_tcp.p));
            assert!(
                err < kinematics::PAR.tol_pos,
                "sample {i} TCP drifted by {err:.6} m >= tol_pos"
            );
        }
        let final_q: [f64; 6] = std::array::from_fn(|j| f64::from(path.last()[j]));
        let end_disp = kinematics::norm(&kinematics::sub(&input.arm.fk(&final_q).p, &start_end.p));
        assert!(
            end_disp > 0.005,
            "end_link must orbit around TCP pivot, moved only {end_disp:.5} m"
        );
    }

    #[test]
    fn move_relative_camera_frame_keeps_raw_end_link_camera_axes() {
        let mut input = inspect_input_with_tcp([0.05, 0.0, 0.0]);
        input.cam.rot_deg = 90.0;
        input.cam.tilt_deg = 0.0;
        let start_end = input.arm.fk(&input.q0);
        let start_tcp = end_to_tcp(&start_end, &input.tcp_offset);
        let rec = input.arm.r_end_cam(&input.cam, f64::from(input.cam.tilt_deg));
        let rc = kinematics::cam_rotation(&start_end, &rec);
        let dz_cam = 0.02;
        let spec = GoalSpec::Relative {
            frame: RelFrame::Camera,
            d: [0.0, 0.0, dz_cam],
            dyaw: 0.0,
            dpitch: 0.0,
            droll: 0.0,
            level: false,
            auto_yaw: None,
        };
        let (goal, opts) = resolve_goal_opts(&input, &spec);
        let path = plan_cartesian_opts(&input, goal, Speed::default(), opts)
            .expect("camera-frame relative move with TCP offset must succeed");
        let final_q: [f64; 6] = std::array::from_fn(|j| f64::from(path.last()[j]));
        let actual_delta = kinematics::sub(
            &end_to_tcp(&input.arm.fk(&final_q), &input.tcp_offset).p,
            &start_tcp.p,
        );
        let z_cam = kinematics::column(&rc, 2);
        let expected_delta = [z_cam[0] * dz_cam, z_cam[1] * dz_cam, z_cam[2] * dz_cam];
        let diff = kinematics::norm(&kinematics::sub(&actual_delta, &expected_delta));
        assert!(diff < 1e-3, "camera-axis displacement diff={diff:.6}");
    }

    #[test]
    fn max_reach_checks_tcp_offset_when_nonzero() {
        let mut input = inspect_input_with_tcp([0.05, 0.0, 0.0]);
        // At q = [0; 6] (HOME), end_link +X points radially forward (+X_base), so tcp_offset increases reach.
        input.q0 = [0.0; 6];
        let end_pose = input.arm.fk(&input.q0);
        let end_r = kinematics::shoulder_reach_from(&end_pose.p, &kinematics::SHOULDER_RZ);
        let tcp_p = end_to_tcp(&end_pose, &input.tcp_offset).p;
        let tcp_r = kinematics::shoulder_reach_from(&tcp_p, &kinematics::SHOULDER_RZ);
        assert!(tcp_r > end_r + 0.04, "end_r={end_r:.4}, tcp_r={tcp_r:.4}");
        let mut ws = input.ws.unwrap();
        ws.max_reach_m = Some(((end_r + tcp_r) * 0.5) as f32);
        let viol = kinematics::workspace_violation_pose_at_tcp(
            &end_pose,
            &ws,
            &input.cam,
            kinematics::SHOULDER_RZ,
            &input.tcp_offset,
        );
        let w = viol.worst.expect("tcp_origin must trigger reach violation");
        assert_eq!(w.limit, "reach");
        assert!((w.value - tcp_r).abs() < 1e-6);
    }
}
