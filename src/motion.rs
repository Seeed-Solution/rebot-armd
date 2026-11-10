//! High-level actions: Cartesian path planning (pure, runs outside the supervisor lock) and
//! the references the supervisor executes for them (spec §2).
//!
//! - [`plan_cartesian`] turns a goal pose into a joint path: straight line in position,
//!   slerp in rotation, one DLS IK solve per sample, single quintic time scaling.
//! - [`PathTrajectory`] executes such a path with lag-aware time scaling.
//! - [`GripperMotion`] runs open / close / move of the 7-DOF gripper with grasp detection.
use serde::{Deserialize, Serialize};
use serde_json::Value;
use thiserror::Error;

use crate::kinematics::{self, Pose};
use crate::model::{CameraMount, GripperPolicy, JointLimit, WorkspacePolicy};
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
    pub epoch: u64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct JointPath {
    /// Samples, the first one is the start.
    pub q: Vec<[f32; 7]>,
    /// Normalised path parameter of each sample (0..=1).
    pub s: Vec<f64>,
    pub duration: f64,
}

impl JointPath {
    pub fn last(&self) -> [f32; 7] { *self.q.last().expect("path has samples") }
}

/// Plan a straight-line (position) / slerp (rotation) path from `fk(q0)` to `goal`.
pub fn plan_cartesian(input: &PlanInput, goal: Pose, speed: Speed) -> Result<JointPath, MotionError> {
    if input.limits.len() < 7 {
        return Err(MotionError::Unsupported("Cartesian actions need a 7-DOF arm".into()));
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
    let start = kinematics::fk(&input.q0);
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

    let lo: [f64; 6] = std::array::from_fn(|i| f64::from(input.limits[i].min_position) + IK_LIMIT_MARGIN);
    let hi: [f64; 6] = std::array::from_fn(|i| f64::from(input.limits[i].max_position) - IK_LIMIT_MARGIN);
    let start_q32: [f32; 7] = std::array::from_fn(|i| if i < 6 { input.q0[i] as f32 } else { input.j7 });
    let v_start = input.ws.as_ref().map(|ws| kinematics::workspace_violation(&start_q32, ws, &input.cam));

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
        let sol = kinematics::solve_dls(&Pose::new(r, p), &prev, &lo, &hi, kinematics::PAR.w);
        if !sol.ok {
            return Err(MotionError::IkUnreachable { s, ep: sol.ep, er: sol.er });
        }
        if let Some((joint, dq)) = (0..6).map(|i| (i, (sol.q[i] - prev[i]).abs())).max_by(|a, b| a.1.total_cmp(&b.1)) {
            if dq > MAX_SAMPLE_JUMP_RAD {
                return Err(MotionError::IkBranchFlip { s, joint: joint + 1, dq });
            }
        }
        let q32: [f32; 7] = std::array::from_fn(|i| if i < 6 { sol.q[i] as f32 } else { input.j7 });
        if let (Some(ws), Some(v0)) = (input.ws.as_ref(), v_start.as_ref()) {
            let v = kinematics::workspace_violation(&q32, ws, &input.cam);
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
        for j in 0..6 {
            let slope = f64::from((q_samples[seg][j] - q_samples[seg - 1][j]).abs()) / ds;
            let vcap = f64::from(input.limits[j].max_velocity);
            let acap = f64::from(input.limits[j].max_acceleration).min(ACTION_MAX_ACCELERATION);
            if vcap > 0.0 { duration = duration.max(1.875 * slope / vcap); }
            if acap > 0.0 { duration = duration.max((5.7735 * slope / acap).sqrt()); }
        }
    }
    duration = duration.max(1.875 * length / speed.linear).max(1.875 * theta / speed.angular);
    let duration = (duration * 1.1).max(0.2);
    Ok(JointPath { q: q_samples, s: s_samples, duration })
}

// ---- goal requests ----

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum RelFrame { Base, Tool, Camera }

#[derive(Debug, Clone, PartialEq)]
pub enum GoalSpec {
    /// Absolute end_link pose; `r = None` keeps the current orientation.
    Absolute { p: [f64; 3], r: Option<kinematics::Mat3> },
    Relative { frame: RelFrame, d: [f64; 3], dyaw: f64, dpitch: f64, droll: f64, level: bool },
}

/// Resolve a goal request against the start pose `fk(q0)`.
pub fn resolve_goal(input: &PlanInput, spec: &GoalSpec) -> Pose {
    let start = kinematics::fk(&input.q0);
    match spec {
        GoalSpec::Absolute { p, r } => Pose::new(r.unwrap_or(start.r), *p),
        GoalSpec::Relative { frame, d, dyaw, dpitch, droll, level } => {
            let x_tool = kinematics::column(&start.r, 0);
            match frame {
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
                    let rec = kinematics::cam_mount(input.cam.flip, f64::from(input.cam.rot_deg), f64::from(input.cam.tilt_deg));
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
    Ok((GoalSpec::Absolute { p, r: parse_orientation(body)? }, parse_speed(body)?, tag(body)?))
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
    Ok((GoalSpec::Relative { frame, d, dyaw, dpitch, droll, level }, parse_speed(body)?, tag(body)?))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GripperCommand { Open, Close, Move }

#[derive(Debug, Clone, PartialEq)]
pub struct GripperRequest {
    pub command: GripperCommand,
    /// 0 = closed, 1 = open (move only).
    pub fraction: Option<f64>,
    pub grasp: bool,
    /// J7 speed (rad/s), <= 2.0.
    pub speed: f64,
    pub timeout_s: f64,
    pub tag: Option<String>,
}

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
    let timeout_s = num(body, "timeout_s")?.unwrap_or(6.0);
    if !(timeout_s > 0.0 && timeout_s <= 30.0) {
        return Err(MotionError::InvalidCommand("timeout_s must be in (0, 30] s".into()));
    }
    Ok(GripperRequest { command, fraction, grasp, speed, timeout_s, tag: tag(body)? })
}

// ---- execution ----

/// Measured / commanded state handed to a reference each tick.
pub struct StepCtx<'a> {
    pub dt: f64,
    pub measured: &'a [f32],
    pub measured_velocities: &'a [f32],
    pub commanded: &'a [f32],
    pub commanded_velocities: &'a [f32],
    pub torques: &'a [f32],
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
        let last = self.path.last().to_vec();
        if let Some(settle) = self.settle.as_mut() {
            let status = match settle.step(ctx.dt, ctx.commanded, ctx.measured, ctx.measured_velocities) {
                Some(settled) => StepStatus::Done { settled: Some(settled), holding: None, gripper_gap_rad: None },
                None => StepStatus::Running,
            };
            return StepOut { positions: last, velocities: vec![0.0; 7], status, progress: 1.0, clear_gripper_stall: false };
        }
        let lag = (0..6).map(|i| f64::from((ctx.commanded[i] - ctx.measured[i]).abs())).fold(0.0, f64::max);
        let lag = if lag.is_finite() { lag } else { f64::INFINITY };
        let rate = ((LAG_STOP - lag) / (LAG_STOP - LAG_FULL)).clamp(0.0, 1.0);
        if rate == 0.0 {
            self.stalled_for += ctx.dt;
            if self.stalled_for >= STALL_TIMEOUT_S - 1e-9 {
                return StepOut {
                    positions: ctx.commanded.to_vec(), velocities: vec![0.0; 7],
                    status: StepStatus::Failed { code: "TRACKING_STALL", message: format!(
                        "Arm did not follow the path for {STALL_TIMEOUT_S} s (lag {lag:.3} rad); motion stopped") },
                    progress: (self.elapsed / self.path.duration) as f32, clear_gripper_stall: false,
                };
            }
        } else {
            self.stalled_for = 0.0;
        }
        self.elapsed = (self.elapsed + ctx.dt * rate).min(self.path.duration);
        let u = self.elapsed / self.path.duration;
        let s = u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
        let q = self.position_at(s);
        let velocities: Vec<f32> = (0..7).map(|i| ((q[i] - ctx.commanded[i]) as f64 / ctx.dt) as f32).collect();
        if self.elapsed >= self.path.duration {
            self.settle = Some(Settle::default());
            return StepOut { positions: last, velocities: vec![0.0; 7], status: StepStatus::Running, progress: 1.0, clear_gripper_stall: false };
        }
        StepOut { positions: q.to_vec(), velocities, status: StepStatus::Running, progress: u as f32, clear_gripper_stall: false }
    }
}

#[derive(Debug, Clone, PartialEq)]
enum GripPhase {
    Moving,
    /// Squeezing an object: hold target, samples of |τ7|.
    Squeeze { hold: f32, elapsed: f64, taus: Vec<f32> },
}

#[derive(Debug, Clone)]
pub struct GripperMotion {
    pub request: GripperRequest,
    pub policy: GripperPolicy,
    pub limit: JointLimit,
    pub target: f32,
    pub start: f32,
    pub elapsed: f64,
    still_for: f64,
    moved: bool,
    first: bool,
    phase: GripPhase,
}

impl GripperMotion {
    pub fn new(request: GripperRequest, policy: GripperPolicy, limit: JointLimit, start: f32) -> Self {
        let target = match request.command {
            GripperCommand::Open => policy.open_rad,
            GripperCommand::Close => policy.closed_rad,
            GripperCommand::Move => policy.closed_rad + request.fraction.unwrap_or(0.0) as f32 * (policy.open_rad - policy.closed_rad),
        }.clamp(limit.min_position, limit.max_position);
        let limit = JointLimit { max_velocity: request.speed as f32, ..limit };
        Self { request, policy, limit, target, start, elapsed: 0.0, still_for: 0.0, moved: false, first: true, phase: GripPhase::Moving }
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
        let (q7, v7c) = (ctx.commanded[6], ctx.commanded_velocities[6]);
        let (m7, mv7) = (ctx.measured[6], ctx.measured_velocities.get(6).copied().unwrap_or(0.0));
        let tau7 = ctx.torques.get(6).copied().unwrap_or(0.0);
        // direction reversal releases a latched stall (same rule as servo / jog)
        let clear = self.first && ctx.gripper_stalled && {
            let delta = self.target - q7;
            delta * tau7 < -1e-4 || delta * ctx.gripper_stall_sign < -1e-4
        };
        let stalled = ctx.gripper_stalled && !clear;
        self.first = false;
        let progress_of = |q: f32| -> f32 {
            let span = (self.target - self.start).abs();
            if span < 1e-6 { 1.0 } else { (1.0 - (self.target - q).abs() / span).clamp(0.0, 1.0) }
        };
        let done = |holding: Option<bool>, gap: Option<f32>| StepStatus::Done { settled: None, holding, gripper_gap_rad: gap };

        if let GripPhase::Squeeze { hold, elapsed, taus } = &mut self.phase {
            *elapsed += ctx.dt;
            taus.push(tau7.abs());
            positions[6] = *hold;
            let status = if *elapsed * 1000.0 >= self.policy.settle_ms as f64 - 1e-6 {
                let mut sorted = taus.clone();
                sorted.sort_by(|a, b| a.total_cmp(b));
                let tau = sorted[sorted.len() / 2];
                let gap = (self.policy.closed_rad - m7) * self.close_dir();
                let holding = gap >= self.policy.holding_gap_rad && tau >= self.policy.holding_torque_nm;
                done(Some(holding), Some(gap))
            } else { StepStatus::Running };
            return StepOut { positions, velocities, status, progress: 1.0, clear_gripper_stall: clear };
        }

        if self.elapsed > self.request.timeout_s {
            positions[6] = m7;
            return StepOut { positions, velocities, progress: progress_of(m7), clear_gripper_stall: clear,
                status: StepStatus::Failed { code: "GRIPPER_TIMEOUT", message: format!(
                    "Gripper did not finish within {:.1} s (at {m7:.3} rad, target {:.3} rad)", self.request.timeout_s, self.target) } };
        }

        // Step J7 toward the target unless the stall latch holds it.
        if !stalled {
            match crate::supervisor::SupervisorInner::bounded_servo_step(q7, self.target, v7c, ctx.dt, &self.limit) {
                Some((q, v, _)) => { positions[6] = q; velocities[6] = v; }
                None => { positions[6] = q7; }
            }
        }
        if (m7 - self.start).abs() > 0.05 { self.moved = true; }
        let cmd_at_target = (positions[6] - self.target).abs() < 1e-4;
        if mv7.abs() < SETTLE_VEL as f32 && (self.moved || cmd_at_target) { self.still_for += ctx.dt; } else { self.still_for = 0.0; }
        let still = self.still_for >= SETTLE_HOLD_S - 1e-9;

        let grasping = self.request.command == GripperCommand::Close && self.request.grasp;
        let status = if grasping {
            if stalled || still {
                let dir = self.close_dir();
                let squeezed = m7 + dir * self.policy.squeeze_rad;
                let hold = if dir > 0.0 { squeezed.min(self.policy.closed_rad) } else { squeezed.max(self.policy.closed_rad) }
                    .clamp(self.limit.min_position, self.limit.max_position);
                positions[6] = hold;
                velocities[6] = 0.0;
                self.phase = GripPhase::Squeeze { hold, elapsed: 0.0, taus: Vec::new() };
                StepStatus::Running
            } else { StepStatus::Running }
        } else if stalled {
            positions[6] = q7;
            velocities[6] = 0.0;
            done(None, Some((self.policy.closed_rad - m7) * self.close_dir()))
        } else if still && ((m7 - self.target).abs() < SETTLE_POS_RAD as f32
            || (cmd_at_target && (m7 - self.target).abs() < GRIPPER_STOP_SHORT_RAD)) {
            // the jaw stopped close to the target: the real gripper opens to about -5.61 against a
            // -5.70 command (mechanical stop / POS_VEL offset); stopping farther away is a jam -> timeout
            done(None, Some((self.policy.closed_rad - m7) * self.close_dir()))
        } else { StepStatus::Running };
        StepOut { positions, velocities, status, progress: progress_of(m7), clear_gripper_stall: clear }
    }
}

/// What the supervisor steps each tick while a trajectory is active.
#[derive(Debug, Clone)]
pub enum Reference {
    Quintic(MultiJointQuinticTrajectory),
    Path(PathTrajectory),
    Gripper(GripperMotion),
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
    /// move_to | move_relative | gripper | grasp | place | joint_move | home | park | move_to_pose
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
    /// Grasp only: open | approach | centring | align | descend | close | release | lift | done.
    #[serde(default)]
    pub phase: Option<String>,
    /// Grasp centring: corrections made so far and the latest pixel error.
    #[serde(default)]
    pub iteration: Option<u32>,
    #[serde(default)]
    pub error_px: Option<f32>,
}

impl ActionStatus {
    pub fn is_running(&self) -> bool { self.state == "running" }
}
