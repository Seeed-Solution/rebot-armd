//! Same-side (backlash-free) final approach (profile section `backlash_approach`).
//!
//! A joint with gear backlash stops at a slightly different place depending on the direction it
//! came from (B601-DM, 2026-11-18: J1 0.28-0.30 deg seen by the camera, 0.01-0.02 deg by the
//! encoder; about 1.6 mm at the base pose between reaching it from above and from below). With
//! the approach on, a motion ends with a short joint-space segment `q_pre -> q_target` at a low
//! speed, where `q_pre = q_target - delta * s` per listed joint:
//!
//! - `s = +1` (the joint arrives moving in +), `s = -1` when `q_target - delta` is below the
//!   joint's soft limit (the joint arrives moving in -);
//! - grasp / place final descent: a joint whose offset alone puts the TCP (end_link origin = jaw
//!   tips) lower than at the target gets its sign flipped, so the short segment never rises
//!   towards the target from below ([`keep_from_above`]). The tool's bounding-box corners are not
//!   part of this rule (any wrist turn lowers one corner of the box, so the rule would refuse
//!   every tool-down pose); they keep the table clearance of the descent instead
//!   ([`DescentRules::table`]).
//!
//! The fine calibration capture of the cockpit uses the same rule (its pre-pose follows
//! `schema.backlash_approach`), so calibration and runtime see the same backlash side.
//! Directions are armd joint (motor) directions, not URDF model directions.
use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::model::JointLimit;
use crate::motion::MotionError;

/// Motions the approach can apply to (`applies_to`).
pub const USES: [&str; 4] = ["move_to_pose", "grasp_final", "place_final", "move_to_precise"];
/// Default pre-target offset (deg): above the measured 0.3 deg J1 backlash.
pub const DEFAULT_DELTA_DEG: f64 = 0.5;
pub const DELTA_RANGE_DEG: (f64, f64) = (0.05, 5.0);
/// Default joint speed cap of the short segment (rad/s).
pub const DEFAULT_SPEED_RAD_S: f32 = 0.05;
pub const SPEED_RANGE_RAD_S: (f32, f32) = (0.005, 0.5);
/// Shortest duration of the short segment (s).
pub const MIN_SEGMENT_S: f64 = 0.5;
/// A tool point counts as lower than at the target below this margin (m).
pub const BELOW_TOL_M: f64 = 1e-4;
/// Largest deviation (rad) between a Cartesian leg's end and `q_pre` that still counts as
/// reaching the pre-target, as a fraction of `delta`.
pub const PRE_MATCH_FRACTION: f32 = 0.25;

fn default_true() -> bool { true }
fn default_delta() -> f64 { DEFAULT_DELTA_DEG }
fn default_speed() -> f32 { DEFAULT_SPEED_RAD_S }
fn default_uses() -> Vec<String> { USES.iter().map(|s| s.to_string()).collect() }

/// Profile section `backlash_approach`; published as `schema.backlash_approach`.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct BacklashApproach {
    #[serde(default = "default_true")]
    pub enabled: bool,
    /// Pre-target offset per joint (deg).
    #[serde(default = "default_delta")]
    pub delta_deg: f64,
    /// Joint names; absent = every arm joint (not the gripper).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub joints: Option<Vec<String>>,
    /// Subset of [`USES`]; absent = all of them. `move_to_precise` applies to `move_to` /
    /// `move_relative` with `precise: true` only.
    #[serde(default = "default_uses")]
    pub applies_to: Vec<String>,
    /// Joint speed cap of the short segment (rad/s).
    #[serde(default = "default_speed")]
    pub speed_rad_s: f32,
}

impl Default for BacklashApproach {
    fn default() -> Self {
        Self { enabled: true, delta_deg: DEFAULT_DELTA_DEG, joints: None, applies_to: default_uses(), speed_rad_s: DEFAULT_SPEED_RAD_S }
    }
}

impl BacklashApproach {
    pub fn validate(&self, joint_names: &[String], arm_joints: &[usize]) -> Result<(), String> {
        if !self.delta_deg.is_finite() || self.delta_deg < DELTA_RANGE_DEG.0 || self.delta_deg > DELTA_RANGE_DEG.1 {
            return Err(format!("backlash_approach.delta_deg must be in [{}, {}]", DELTA_RANGE_DEG.0, DELTA_RANGE_DEG.1));
        }
        if !self.speed_rad_s.is_finite() || self.speed_rad_s < SPEED_RANGE_RAD_S.0 || self.speed_rad_s > SPEED_RANGE_RAD_S.1 {
            return Err(format!("backlash_approach.speed_rad_s must be in [{}, {}]", SPEED_RANGE_RAD_S.0, SPEED_RANGE_RAD_S.1));
        }
        if let Some(u) = self.applies_to.iter().find(|u| !USES.contains(&u.as_str())) {
            return Err(format!("backlash_approach.applies_to: unknown entry '{u}' (one of {})", USES.join(", ")));
        }
        if let Some(names) = &self.joints {
            for n in names {
                match joint_names.iter().position(|j| j == n) {
                    Some(i) if arm_joints.contains(&i) => {}
                    Some(_) => return Err(format!("backlash_approach.joints: '{n}' is not an arm joint")),
                    None => return Err(format!("backlash_approach.joints: unknown joint '{n}'")),
                }
            }
        }
        Ok(())
    }

    pub fn delta_rad(&self) -> f32 { self.delta_deg.to_radians() as f32 }

    /// The listed joints as armd joint indices (sorted), every arm joint when `joints` is absent.
    pub fn joint_indices(&self, joint_names: &[String], arm_joints: &[usize]) -> Vec<usize> {
        let mut v: Vec<usize> = match &self.joints {
            None => arm_joints.to_vec(),
            Some(names) => names.iter().filter_map(|n| joint_names.iter().position(|j| j == n)).filter(|i| arm_joints.contains(i)).collect(),
        };
        v.sort_unstable();
        v.dedup();
        v
    }
}

/// Request field `approach`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Request {
    /// Absent: the profile decides.
    Profile,
    SameSide,
    Direct,
}

pub fn parse_request(body: &Value) -> Result<Request, MotionError> {
    match body.get("approach") {
        None | Some(Value::Null) => Ok(Request::Profile),
        Some(Value::String(s)) if s == "same_side" => Ok(Request::SameSide),
        Some(Value::String(s)) if s == "direct" => Ok(Request::Direct),
        Some(_) => Err(MotionError::InvalidCommand("approach must be \"same_side\" or \"direct\"".into())),
    }
}

/// Settings to use for motion `use_` (one of [`USES`]) under the request field `approach`.
pub fn resolve(policy: Option<&BacklashApproach>, use_: &str, request: Request) -> Resolved {
    match request {
        Request::Direct => Resolved::Direct(Some("request")),
        Request::SameSide => Resolved::SameSide(policy.cloned().map(|p| BacklashApproach { enabled: true, ..p }).unwrap_or_default()),
        Request::Profile => match policy {
            Some(p) if p.enabled && p.applies_to.iter().any(|u| u == use_) => Resolved::SameSide(p.clone()),
            _ => Resolved::Direct(None),
        },
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum Resolved {
    SameSide(BacklashApproach),
    /// Direct; Some(reason) is recorded in the action (`approach: "direct"`), None leaves the
    /// action record as without the feature.
    Direct(Option<&'static str>),
}

/// Pre-target of `target` (armd joint angles): `q_pre[j] = target[j] - delta * s[j]` for the
/// listed joints, `s[j] = +1`, or -1 when `target[j] - delta` is below the joint's lower soft
/// limit. Returns (q_pre, s per joint (0 = not listed)). Err when the joint range is narrower
/// than delta on both sides.
pub fn pre_target(target: &[f32], joints: &[usize], delta: f32, limits: &[JointLimit]) -> Result<(Vec<f32>, Vec<i8>), String> {
    let mut pre = target.to_vec();
    let mut signs = vec![0i8; target.len()];
    for &j in joints {
        let (Some(l), Some(&t)) = (limits.get(j), target.get(j)) else { continue };
        let s: i8 = if t - delta >= l.min_position { 1 } else if t + delta <= l.max_position { -1 } else {
            return Err(format!("joint {} has no room for the pre-target", l.name));
        };
        pre[j] = t - delta * f32::from(s);
        signs[j] = s;
    }
    Ok((pre, signs))
}

/// Grasp / place final descent: flip the sign of every listed joint whose offset alone puts a
/// tool point lower than at the target (`lowest(q)` = lowest tool point z, None = no model),
/// then require the combined pre-target not lower either. Returns the flipped joints; Err(reason)
/// when a flip leaves the joint range or the combined pre-target is still lower.
pub fn keep_from_above(target: &[f32], pre: &mut [f32], signs: &mut [i8], delta: f32, limits: &[JointLimit],
    lowest: impl Fn(&[f32]) -> Option<f64>) -> Result<Vec<usize>, String> {
    let Some(z_target) = lowest(target) else { return Ok(Vec::new()) };
    let mut flipped = Vec::new();
    for j in 0..signs.len() {
        if signs[j] == 0 { continue; }
        let mut q = target.to_vec();
        q[j] = pre[j];
        if lowest(&q).is_some_and(|z| z < z_target - BELOW_TOL_M) {
            let s = -signs[j];
            let qj = target[j] - delta * f32::from(s);
            let l = &limits[j];
            if qj < l.min_position || qj > l.max_position {
                return Err(format!("from_below: {} cannot approach from above within its limits", l.name));
            }
            q[j] = qj;
            if lowest(&q).is_some_and(|z| z < z_target - BELOW_TOL_M) {
                return Err(format!("from_below: {} lowers the tool either way", l.name));
            }
            signs[j] = s;
            pre[j] = qj;
            flipped.push(j);
        }
    }
    if lowest(pre).is_some_and(|z| z < z_target - BELOW_TOL_M) {
        return Err("from_below: the pre-target is lower than the target".into());
    }
    Ok(flipped)
}

/// The short segment's planning limits: `limits` with the listed joints' speed capped at
/// `speed_rad_s`.
pub fn segment_limits(limits: &[JointLimit], joints: &[usize], speed_rad_s: f32) -> Vec<JointLimit> {
    let mut l = limits.to_vec();
    for &j in joints { if let Some(x) = l.get_mut(j) { x.max_velocity = x.max_velocity.min(speed_rad_s); } }
    l
}

/// Did a leg that should end at `pre` end there (every joint within [`PRE_MATCH_FRACTION`] x
/// delta)? Then the short segment `end -> target` moves every listed joint in direction `s`.
pub fn reached_pre(end: &[f32], pre: &[f32], delta: f32) -> bool {
    end.iter().zip(pre).all(|(a, b)| (a - b).abs() <= delta * PRE_MATCH_FRACTION)
}

/// Action-record fields of a same-side or a direct-with-reason motion.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Outcome {
    /// `same_side` | `direct`
    pub approach: Option<String>,
    pub reason: Option<String>,
    /// Arm joints' approach direction (+1 / -1, 0 = not listed); same_side only.
    pub signs: Option<Vec<i8>>,
    /// Joint names flipped by the from-above rule.
    pub flipped: Option<Vec<String>>,
}

impl Outcome {
    pub fn direct(reason: impl Into<String>) -> Self { Self { approach: Some("direct".into()), reason: Some(reason.into()), ..Default::default() } }
    pub fn same_side(signs: &[i8], arm_dof: usize, flipped: Vec<String>) -> Self {
        Self { approach: Some("same_side".into()), reason: None, signs: Some(signs[..arm_dof.min(signs.len())].to_vec()),
            flipped: (!flipped.is_empty()).then_some(flipped) }
    }
    pub fn apply(&self, a: &mut crate::motion::ActionStatus) {
        a.approach = self.approach.clone();
        a.approach_reason = self.reason.clone();
        a.approach_signs = self.signs.clone();
        a.approach_flipped = self.flipped.clone();
    }
}

/// Lowest z (base frame) of `points` (end_link frame) at armd angles `q`.
pub fn lowest_z(arm: &crate::chain::ArmModel, points: &[crate::kinematics::Vec3], q: &[f32]) -> Option<f64> {
    let pose = arm.fk_f32(q)?;
    Some(points.iter().map(|p| pose.transform_point(p)[2]).fold(f64::INFINITY, f64::min))
}

/// The joint-space segment `pre -> target` (a straight line in joint space) keeps every point at
/// or above its height at the target (20 samples).
pub fn segment_from_above(target: &[f32], pre: &[f32], lowest: impl Fn(&[f32]) -> Option<f64>) -> Result<(), String> {
    let Some(z_target) = lowest(target) else { return Ok(()) };
    for k in 0..20 {
        let f = k as f32 / 20.0;
        let q: Vec<f32> = pre.iter().zip(target).map(|(a, b)| a + (b - a) * f).collect();
        if lowest(&q).is_some_and(|z| z < z_target - BELOW_TOL_M) {
            return Err("from_below: the short segment passes below the target height".into());
        }
    }
    Ok(())
}

/// Grasp / place descent rules for the leg to the pre-target and the short segment.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct DescentRules {
    /// Points (end_link frame) kept no lower than at the target ([`keep_from_above`]): the TCP.
    pub from_above: Vec<crate::kinematics::Vec3>,
    /// Table clearance: (points in the end_link frame, bound z). A point starting above the bound
    /// stays above it, one starting below it does not go lower (the grasp table rule).
    pub table: Option<(Vec<crate::kinematics::Vec3>, f64)>,
}

impl DescentRules {
    /// The grasp / place default: the `end_link` origin from above, no table rule.
    pub fn tcp() -> Self { Self { from_above: vec![[0.0; 3]], table: None } }

    /// Protect an explicit local point in `end_link` frame (`tcp_offset` for TCP actions).
    pub fn tcp_at(point_end_m: [f64; 3]) -> Self { Self { from_above: vec![point_end_m], table: None } }
}

/// Detailed diagnostic for a table-clearance violation.
#[derive(Debug, Clone, PartialEq)]
pub struct TableClearViolation {
    pub sample_index: usize,
    pub total_samples: usize,
    pub point: usize,
    pub value_m: f64,
    pub bound_m: f64,
}

impl TableClearViolation {
    pub fn s(&self) -> f64 {
        if self.total_samples <= 1 {
            1.0
        } else {
            self.sample_index as f64 / (self.total_samples - 1) as f64
        }
    }

    pub fn message(&self) -> String {
        format!(
            "table: sample {}/{} tool point {} z={:.4} m below clearance bound {:.4} m",
            self.sample_index, self.total_samples, self.point, self.value_m, self.bound_m
        )
    }

    pub fn into_motion_error(self) -> MotionError {
        let s = self.s();
        let reason = self.message();
        MotionError::WorkspaceViolation {
            s,
            point: self.point,
            limit: "table_z_m + 0.005".into(),
            value: self.value_m,
            bound: self.bound_m,
            detail: Box::new(crate::kinematics::WorkspaceViolationDetail {
                constraint_id: Some("table_z".into()),
                geometry_id: None,
                violation_phase: Some("plan".into()),
                reason: Some(reason),
            }),
        }
    }
}

/// Check the table rule of [`DescentRules::table`] along joint-space samples `qs`, returning full
/// violation diagnostics (sample index, tool point index, measured z, and effective bound).
pub fn check_table_clear<'a>(
    arm: &crate::chain::ArmModel,
    points: &[crate::kinematics::Vec3],
    bound: f64,
    qs: impl IntoIterator<Item = &'a [f32]>,
) -> Result<(), TableClearViolation> {
    let samples: Vec<&'a [f32]> = qs.into_iter().collect();
    let total_samples = samples.len();
    let Some(start) = samples.first().and_then(|q| arm.fk_f32(q)) else {
        return Ok(());
    };
    let floor: Vec<f64> = points.iter().map(|p| start.transform_point(p)[2].min(bound)).collect();
    for (sample_index, q) in samples.iter().enumerate().skip(1) {
        let Some(pose) = arm.fk_f32(q) else { continue };
        for (point, (p, f)) in points.iter().zip(&floor).enumerate() {
            let z = pose.transform_point(p)[2];
            if z < f - 1e-6 {
                return Err(TableClearViolation {
                    sample_index,
                    total_samples,
                    point,
                    value_m: z,
                    bound_m: *f,
                });
            }
        }
    }
    Ok(())
}

/// The table rule of [`DescentRules::table`] along joint-space samples `qs` (the first is the start).
pub fn table_clear<Q: AsRef<[f32]>>(arm: &crate::chain::ArmModel, points: &[crate::kinematics::Vec3], bound: f64, qs: &[Q]) -> Result<(), String> {
    table_clear_iter(arm, points, bound, qs.iter().map(AsRef::as_ref))
}

/// Iterator variant of [`table_clear`].
pub fn table_clear_iter<'a>(arm: &crate::chain::ArmModel, points: &[crate::kinematics::Vec3], bound: f64, qs: impl IntoIterator<Item = &'a [f32]>) -> Result<(), String> {
    check_table_clear(arm, points, bound, qs).map_err(|v| v.message())
}

/// Same-side settings of one motion, taken under the supervisor lock.
#[derive(Debug, Clone, PartialEq)]
pub struct Plan {
    pub policy: BacklashApproach,
    /// Listed joints (armd indices).
    pub joints: Vec<usize>,
    /// Planning limits of the short segment.
    pub seg_limits: Vec<JointLimit>,
    pub joint_names: Vec<String>,
    pub arm_dof: usize,
}

/// A Cartesian motion's leg to the pre-target, planned outside the lock.
#[derive(Debug, Clone)]
pub struct CartesianPre {
    pub path: crate::motion::JointPath,
    /// End of the direct path (armd angles, all joints).
    pub target: Vec<f32>,
    pub signs: Vec<i8>,
    pub flipped: Vec<usize>,
}

/// What a Cartesian motion does at its end.
#[derive(Debug, Clone)]
pub enum Pre {
    /// The approach does not apply (no record fields).
    Off,
    /// Direct, with the reason recorded.
    Direct(String),
    Ready(Box<CartesianPre>),
}

/// Outside the lock: from the `direct` path (to the goal), the pre-target of its end and the
/// Cartesian path to the pre-target's pose. `rules`: the grasp / place descent rules.
pub fn plan_cartesian_pre(input: &crate::motion::PlanInput, direct: &crate::motion::JointPath, plan: &Plan,
    rules: Option<&DescentRules>, speed: crate::motion::Speed, opts: impl Into<crate::motion::PathOpts>) -> Pre {
    match plan_cartesian_pre_inner(input, direct, plan, rules, speed, opts.into()) {
        Ok(p) => Pre::Ready(Box::new(p)),
        Err(r) => Pre::Direct(r),
    }
}

fn plan_cartesian_pre_inner(input: &crate::motion::PlanInput, direct: &crate::motion::JointPath, plan: &Plan,
    rules: Option<&DescentRules>, speed: crate::motion::Speed, opts: crate::motion::PathOpts) -> Result<CartesianPre, String> {
    let target = direct.last_slice().to_vec();
    let delta = plan.policy.delta_rad();
    let (mut pre, mut signs) = pre_target(&target, &plan.joints, delta, &input.limits)?;
    let mut flipped = Vec::new();
    if let Some(points) = rules.map(|r| r.from_above.as_slice()).filter(|p| !p.is_empty()) {
        let lowest = |q: &[f32]| lowest_z(&input.arm, points, q);
        flipped = keep_from_above(&target, &mut pre, &mut signs, delta, &input.limits, lowest)?;
        segment_from_above(&target, &pre, lowest)?;
    }
    let end = input.arm.fk_f32(&pre).ok_or_else(|| "pre_target: no kinematic model".to_string())?;
    let goal = crate::motion::end_to_frame(&end, input);
    let path = crate::motion::plan_cartesian_opts(input, goal, speed, opts).map_err(|e| format!("pre_target: {}", e.code()))?;
    if !reached_pre(path.last_slice(), &pre, delta) {
        return Err("pre_target: the path to the pre-target ends elsewhere (IK)".into());
    }
    if let Some((points, bound)) = rules.and_then(|r| r.table.as_ref()) {
        // the leg (its samples) and the segment (21 joint-space samples) from the start state
        let end = path.last_slice().to_vec();
        let mut qs: Vec<Vec<f32>> = path.q.iter().map(|q| q[..end.len()].to_vec()).collect();
        qs.extend((1..=20).map(|k| end.iter().zip(&target).map(|(a, b)| a + (b - a) * k as f32 / 20.0).collect()));
        table_clear(&input.arm, points, *bound, &qs)?;
    }
    Ok(CartesianPre { path, target, signs, flipped })
}

#[cfg(not(target_arch = "wasm32"))]
mod runtime {
    use super::*;
    use crate::motion::{ActiveTrajectory, JointPath};
    use crate::supervisor::{ArmError, SupervisorInner};
    use crate::trajectory::{MultiJointQuinticTrajectory, TrajectoryPlanner};

    fn code(e: ArmError) -> String {
        let api = crate::api::rest::ApiError::from(e);
        api.code.unwrap_or(api.error)
    }

    impl SupervisorInner {
        /// Same-side settings for motion `use_` (one of [`USES`]): Ok(plan), or Err(reason) for a
        /// direct motion (None = the approach does not apply: no record fields).
        pub fn approach_plan(&self, use_: &str, request: Request) -> Result<Plan, Option<&'static str>> {
            let policy = match resolve(self.config.backlash_approach.as_ref(), use_, request) {
                Resolved::Direct(r) => return Err(r),
                Resolved::SameSide(p) => p,
            };
            let arm = self.config.arm_joint_indices();
            let joints = policy.joint_indices(&self.config.joint_names, &arm);
            let planning = self.planning_limits();
            let mut seg_limits = segment_limits(&planning, &joints, policy.speed_rad_s);
            if self.config.active_tool.commissioning_required {
                let gripper_idx = self.config.gripper_index();
                for (i, (seg, base)) in seg_limits.iter_mut().zip(&planning).enumerate() {
                    if Some(i) != gripper_idx {
                        seg.max_velocity = seg.max_velocity.min(base.max_velocity * 0.2);
                    }
                }
            }
            Ok(Plan { policy, joints, seg_limits, joint_names: self.config.joint_names.clone(), arm_dof: arm.len() })
        }

        /// The short segment `start -> target` with every check of a joint move: no torque /
        /// power hold in force, soft limits, power budget, workspace and keep-out sweep.
        /// Returns it (power-fitted) and its power scale; Err(reason).
        fn approach_segment(&self, start: &[f32], target: &[f32], seg_limits: &[JointLimit]) -> Result<(MultiJointQuinticTrajectory, Option<f64>), String> {
            if self.torque_guard.hold.is_some() || self.power.gov.hold.is_some() { return Err("segment: a torque or power hold is in force".into()); }
            let epoch = Some(self.safety.epoch_id());
            self.safety.validate_motion_command(epoch, start, &self.config.limits).map_err(|e| format!("pre_target: {}", code(e.into())))?;
            self.safety.validate_motion_command(epoch, target, &self.config.limits).map_err(|e| format!("segment: {}", code(e.into())))?;
            let zeros = vec![0.0; start.len()];
            let traj = TrajectoryPlanner::plan(start, &zeros, target, &zeros, Some(MIN_SEGMENT_S), seg_limits)
                .map_err(|e| format!("segment: {e}"))?;
            let (traj, scale) = self.power_fit_quintic(traj, start, &zeros, target, seg_limits, true).map_err(|e| format!("segment: {}", code(e)))?;
            let samples = self.workspace_samples(&traj).map_err(|e| format!("segment: {}", code(e.into())))?;
            self.check_workspace_path(start, samples.iter().map(Vec::as_slice)).map_err(|e| format!("segment: {}", code(e.into())))?;
            Ok((traj, scale))
        }

        fn record_approach(&mut self, id: u64, outcome: &Outcome) {
            if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) { outcome.apply(a); }
        }

        fn outcome_of(plan: &Plan, signs: &[i8], flipped: &[usize]) -> Outcome {
            Outcome::same_side(signs, plan.arm_dof, flipped.iter().map(|&j| plan.joint_names.get(j).cloned().unwrap_or_else(|| j.to_string())).collect())
        }

        /// [`start_quintic`](Self::start_quintic) with the same-side approach of `use_`: a quintic
        /// to the pre-target, then the short segment to `target`. Falls back to the direct quintic
        /// (reason in the action record) when the pre-target or the segment fails a check.
        /// Returns (epoch, action id, planned duration of both legs).
        #[allow(clippy::too_many_arguments)]
        pub(crate) fn start_quintic_same_side(&mut self, target: Vec<f32>, epoch_id: Option<u64>, kind: &str, tag: Option<String>,
            limits: &[JointLimit], use_: &str, request: Request) -> Result<(u64, u64, f64), ArmError> {
            let plan = match self.approach_plan(use_, request) {
                Ok(p) => p,
                Err(None) => return self.start_quintic(target, None, epoch_id, kind, tag, limits),
                Err(Some(r)) => return self.start_quintic_direct(target, epoch_id, kind, tag, limits, r.to_string()),
            };
            let delta = plan.policy.delta_rad();
            let (pre, signs) = match pre_target(&target, &plan.joints, delta, &self.config.limits) {
                Ok(x) => x,
                Err(r) => return self.start_quintic_direct(target, epoch_id, kind, tag, limits, format!("pre_target: {r}")),
            };
            let mut seg_limits = plan.seg_limits.clone();
            for (seg, lim) in seg_limits.iter_mut().zip(limits) {
                seg.max_velocity = seg.max_velocity.min(lim.max_velocity);
            }
            let (seg, seg_scale) = match self.approach_segment(&pre, &target, &seg_limits) {
                Ok(x) => x,
                Err(r) => return self.start_quintic_direct(target, epoch_id, kind, tag, limits, r),
            };
            let (epoch, id, d1) = match self.start_quintic(pre, None, epoch_id, kind, tag.clone(), limits) {
                Ok(x) => x,
                Err(e) => return self.start_quintic_direct(target, epoch_id, kind, tag, limits, format!("pre_target: {}", code(e))),
            };
            let d2 = seg.duration;
            if let Some(active) = self.active_trajectory.take() {
                self.active_trajectory = Some(ActiveTrajectory::with_approach(active.action_id, active.reference, seg));
            }
            self.target_positions = target;
            if let Some(s) = seg_scale { self.note_action_power_scale(id, s); }
            self.record_approach(id, &Self::outcome_of(&plan, &signs, &[]));
            if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) { a.phase = Some("move".into()); }
            Ok((epoch, id, d1 + d2))
        }

        fn start_quintic_direct(&mut self, target: Vec<f32>, epoch_id: Option<u64>, kind: &str, tag: Option<String>,
            limits: &[JointLimit], reason: String) -> Result<(u64, u64, f64), ArmError> {
            let started = self.start_quintic(target, None, epoch_id, kind, tag, limits)?;
            self.record_approach(started.1, &Outcome::direct(reason));
            Ok(started)
        }

        /// [`install_path`](Self::install_path) with the same-side approach: `pre` from
        /// [`plan_cartesian_pre`]; the direct path when it is not ready or the short segment
        /// fails a check. Returns (action id, expected duration).
        pub fn install_path_same_side(&mut self, input: &crate::motion::PlanInput, direct: JointPath, pre: Pre, plan: Option<&Plan>,
            kind: &str, tag: Option<String>) -> Result<(u64, f64), ArmError> {
            self.install_path_same_side_with_rules(input, direct, pre, plan, None, kind, tag)
        }

        /// [`install_path_same_side`](Self::install_path_same_side) with `rules` enforced on the
        /// actually selected path (`cp.path` + `seg`, or `direct` on any fallback).
        #[allow(clippy::too_many_arguments)]
        pub fn install_path_same_side_with_rules(&mut self, input: &crate::motion::PlanInput, direct: JointPath, pre: Pre, plan: Option<&Plan>,
            rules: Option<&DescentRules>, kind: &str, tag: Option<String>) -> Result<(u64, f64), ArmError> {
            let check_direct_table = |path: &JointPath| -> Result<(), ArmError> {
                if let Some((points, bound)) = rules.and_then(|r| r.table.as_ref()) {
                    check_table_clear(&input.arm, points, *bound, path.q.iter().map(|q| q.as_slice()))
                        .map_err(|v| ArmError::from(v.into_motion_error()))?;
                }
                Ok(())
            };
            let (cp, plan) = match (pre, plan) {
                (Pre::Ready(cp), Some(plan)) => (cp, plan),
                (Pre::Direct(r), _) => {
                    check_direct_table(&direct)?;
                    let started = self.install_path(input, direct, kind, tag)?;
                    self.record_approach(started.0, &Outcome::direct(r));
                    return Ok(started);
                }
                _ => {
                    check_direct_table(&direct)?;
                    return self.install_path(input, direct, kind, tag);
                }
            };
            let (seg, seg_scale) = match self.approach_segment(cp.path.last_slice(), &cp.target, &plan.seg_limits) {
                Ok(x) => x,
                Err(r) => return self.install_path_same_side_with_rules(input, direct, Pre::Direct(r), None, rules, kind, tag),
            };
            if let Some((points, bound)) = rules.and_then(|r| r.table.as_ref()) {
                let seg_qs = match self.workspace_samples(&seg) {
                    Ok(s) => s,
                    Err(e) => return self.install_path_same_side_with_rules(input, direct, Pre::Direct(format!("segment: {}", code(e.into()))), None, rules, kind, tag),
                };
                if let Err(r) = table_clear_iter(&input.arm, points, *bound, cp.path.q.iter().map(|q| q.as_slice()).chain(seg_qs.iter().map(Vec::as_slice))) {
                    return self.install_path_same_side_with_rules(input, direct, Pre::Direct(r), None, rules, kind, tag);
                }
            }
            let target = cp.target.clone();
            let (id, d1) = match self.install_path(input, cp.path, kind, tag.clone()) {
                Ok(x) => x,
                Err(e) => return self.install_path_same_side_with_rules(input, direct, Pre::Direct(format!("pre_target: {}", code(e))), None, rules, kind, tag),
            };
            let d2 = seg.duration;
            if let Some(active) = self.active_trajectory.take() {
                self.active_trajectory = Some(ActiveTrajectory::with_approach(active.action_id, active.reference, seg));
            }
            self.target_positions = target;
            if let Some(s) = seg_scale { self.note_action_power_scale(id, s); }
            self.record_approach(id, &Self::outcome_of(plan, &cp.signs, &cp.flipped));
            if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) { a.phase = Some("move".into()); }
            Ok((id, d1 + d2))
        }

        /// [`install_macro_path`](Self::install_macro_path) with the same-side approach (grasp /
        /// place final descent). Records the outcome on action `id`. Returns the expected duration.
        pub fn install_macro_path_same_side(&mut self, id: u64, input: &crate::motion::PlanInput, direct: JointPath, pre: Pre,
            plan: Option<&Plan>) -> Result<f64, ArmError> {
            self.install_macro_path_same_side_with_rules(id, input, direct, pre, plan, None)
        }

        /// [`install_macro_path_same_side`](Self::install_macro_path_same_side) with `rules`
        /// enforced on the actually selected path (`cp.path` + `seg`, or `direct` on any fallback).
        pub fn install_macro_path_same_side_with_rules(&mut self, id: u64, input: &crate::motion::PlanInput, direct: JointPath, pre: Pre,
            plan: Option<&Plan>, rules: Option<&DescentRules>) -> Result<f64, ArmError> {
            let check_direct_table = |path: &JointPath| -> Result<(), ArmError> {
                if let Some((points, bound)) = rules.and_then(|r| r.table.as_ref()) {
                    check_table_clear(&input.arm, points, *bound, path.q.iter().map(|q| q.as_slice()))
                        .map_err(|v| ArmError::from(v.into_motion_error()))?;
                }
                Ok(())
            };
            let (cp, plan) = match (pre, plan) {
                (Pre::Ready(cp), Some(plan)) => (cp, plan),
                (Pre::Direct(r), _) => {
                    check_direct_table(&direct)?;
                    let d = self.install_macro_path(id, input, direct)?;
                    self.record_approach(id, &Outcome::direct(r));
                    return Ok(d);
                }
                _ => {
                    check_direct_table(&direct)?;
                    return self.install_macro_path(id, input, direct);
                }
            };
            let (seg, seg_scale) = match self.approach_segment(cp.path.last_slice(), &cp.target, &plan.seg_limits) {
                Ok(x) => x,
                Err(r) => return self.install_macro_path_same_side_with_rules(id, input, direct, Pre::Direct(r), None, rules),
            };
            if let Some((points, bound)) = rules.and_then(|r| r.table.as_ref()) {
                let seg_qs = match self.workspace_samples(&seg) {
                    Ok(s) => s,
                    Err(e) => return self.install_macro_path_same_side_with_rules(id, input, direct, Pre::Direct(format!("segment: {}", code(e.into()))), None, rules),
                };
                if let Err(r) = table_clear_iter(&input.arm, points, *bound, cp.path.q.iter().map(|q| q.as_slice()).chain(seg_qs.iter().map(Vec::as_slice))) {
                    return self.install_macro_path_same_side_with_rules(id, input, direct, Pre::Direct(r), None, rules);
                }
            }
            let target = cp.target.clone();
            let d1 = match self.install_macro_path(id, input, cp.path) {
                Ok(d) => d,
                Err(e) => return self.install_macro_path_same_side_with_rules(id, input, direct, Pre::Direct(format!("pre_target: {}", code(e))), None, rules),
            };
            let d2 = seg.duration;
            if let Some(active) = self.active_trajectory.take() {
                self.active_trajectory = Some(ActiveTrajectory::with_approach(active.action_id, active.reference, seg));
            }
            self.target_positions = target;
            if let Some(s) = seg_scale { self.note_action_power_scale(id, s); }
            self.record_approach(id, &Self::outcome_of(plan, &cp.signs, &cp.flipped));
            // the short segment settles before the sub-motion counts as done
            Ok(d1 + d2 + crate::motion::SETTLE_MAX_S)
        }

        /// Expected extra duration (s) of the short segment for motion `use_` (0 when the
        /// approach does not apply); for the ack of grasp / place.
        pub fn approach_estimate_s(&self, use_: &str, request: Request) -> f64 {
            match self.approach_plan(use_, request) {
                Ok(p) => (1.875 * f64::from(p.policy.delta_rad()) / f64::from(p.policy.speed_rad_s)).max(MIN_SEGMENT_S) + 0.5,
                Err(_) => 0.0,
            }
        }

        /// The same-side approach of action `id` did not run to its end (contact stop): the
        /// record says direct with `reason`.
        pub fn approach_interrupted(&mut self, id: u64, reason: &str) {
            if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id && a.approach.as_deref() == Some("same_side")) {
                Outcome::direct(reason).apply(a);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lim(lo: f32, hi: f32) -> JointLimit { JointLimit::new("j", lo, hi, 1.0, 1.0) }

    #[test]
    fn pre_target_flips_at_the_lower_limit() {
        let limits = vec![lim(-1.0, 1.0), lim(-1.0, 1.0), lim(-0.005, 1.0)];
        let (pre, s) = pre_target(&[0.2, -0.995, 0.0], &[0, 1, 2], 0.01, &limits).unwrap();
        assert_eq!(s, vec![1, -1, -1]);
        assert!((pre[0] - 0.19).abs() < 1e-6 && (pre[1] + 0.985).abs() < 1e-6 && (pre[2] - 0.01).abs() < 1e-6, "{pre:?}");
        assert!(pre_target(&[0.0], &[0], 0.01, &[lim(-0.005, 0.005)]).is_err());
    }

    #[test]
    fn resolve_rules() {
        let p = BacklashApproach { applies_to: vec!["move_to_pose".into()], ..Default::default() };
        assert!(matches!(resolve(Some(&p), "move_to_pose", Request::Profile), Resolved::SameSide(_)));
        assert_eq!(resolve(Some(&p), "grasp_final", Request::Profile), Resolved::Direct(None));
        assert_eq!(resolve(Some(&p), "move_to_pose", Request::Direct), Resolved::Direct(Some("request")));
        assert!(matches!(resolve(None, "grasp_final", Request::SameSide), Resolved::SameSide(_)));
        let off = BacklashApproach { enabled: false, ..Default::default() };
        assert_eq!(resolve(Some(&off), "move_to_pose", Request::Profile), Resolved::Direct(None));
    }

    #[test]
    fn keep_from_above_flips_the_lowering_joint() {
        // z = q0 - q1: lowering q0 lowers the tool, lowering q1 raises it
        let lowest = |q: &[f32]| Some(f64::from(q[0] - q[1]));
        let limits = vec![lim(-1.0, 1.0), lim(-1.0, 1.0)];
        let target = [0.0, 0.0];
        let (mut pre, mut s) = pre_target(&target, &[0, 1], 0.01, &limits).unwrap();
        let flipped = keep_from_above(&target, &mut pre, &mut s, 0.01, &limits, lowest).unwrap();
        assert_eq!(flipped, vec![0]);
        assert_eq!(s, vec![-1, 1]);
        assert!(lowest(&pre).unwrap() >= lowest(&target).unwrap());
        // no room above: refused
        let limits = vec![lim(-1.0, 0.005), lim(-1.0, 1.0)];
        let (mut pre, mut s) = pre_target(&target, &[0, 1], 0.01, &limits).unwrap();
        assert!(keep_from_above(&target, &mut pre, &mut s, 0.01, &limits, lowest).is_err());
    }

    #[test]
    fn validate_rejects_bad_sections() {
        let names: Vec<String> = ["joint_1", "joint_2", "gripper"].iter().map(|s| s.to_string()).collect();
        let arm = [0, 1];
        assert!(BacklashApproach::default().validate(&names, &arm).is_ok());
        assert!(BacklashApproach { delta_deg: 0.0, ..Default::default() }.validate(&names, &arm).is_err());
        assert!(BacklashApproach { applies_to: vec!["home".into()], ..Default::default() }.validate(&names, &arm).is_err());
        assert!(BacklashApproach { joints: Some(vec!["gripper".into()]), ..Default::default() }.validate(&names, &arm).is_err());
        let p = BacklashApproach { joints: Some(vec!["joint_2".into()]), ..Default::default() };
        assert_eq!(p.joint_indices(&names, &arm), vec![1]);
        let parsed: BacklashApproach = serde_json::from_str("{}").unwrap();
        assert_eq!(parsed, BacklashApproach::default());
    }

    #[test]
    fn r3_minor1_check_table_clear_reports_actual_sample_point_value_and_bound() {
        let arm = crate::chain::ArmModel::REBOT_DEV_ARM;
        // Sample 0 is well above z=0; sample 1 dips below bound at the second envelope point.
        let q_high = [0.0f32, -0.4, -0.4, 0.2, 0.0, 0.0, 0.0];
        let q_low = [0.0f32, 0.8, 0.2, 0.8, 0.0, 0.0, 0.0];
        let end_low = arm.fk_f32(&q_low).unwrap();
        let bound = 0.05;
        // Point 0 stays safely above bound; point 1 sits far below end_link (-0.30 in z in base/end frame).
        let pt_safe = [0.0, 0.0, 0.50];
        let mut r_inv = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                r_inv[i][j] = end_low.r[j][i];
            }
        }
        let world_low = [0.0, 0.0, -0.10];
        let pt_violating = crate::kinematics::mat_vec(&r_inv, &crate::kinematics::sub(&world_low, &end_low.p));
        let points = [pt_safe, pt_violating];

        let qs: [&[f32]; 3] = [&q_high, &q_low, &q_high];
        let err = check_table_clear(&arm, &points, bound, qs).unwrap_err();
        assert_eq!(err.sample_index, 1);
        assert_eq!(err.total_samples, 3);
        assert_eq!(err.point, 1);
        assert!((err.value_m - (-0.10)).abs() < 1e-6, "got value_m={}", err.value_m);
        assert!((err.bound_m - bound).abs() < 1e-9, "got bound_m={}", err.bound_m);
        let motion_err = err.into_motion_error();
        match motion_err {
            crate::motion::MotionError::WorkspaceViolation { s, point, value, bound: b, .. } => {
                assert!((s - 0.5).abs() < 1e-9, "s={s}");
                assert_eq!(point, 1);
                assert!((value - (-0.10)).abs() < 1e-6, "value={value}");
                assert!((b - bound).abs() < 1e-9, "bound={b}");
            }
            other => panic!("expected WorkspaceViolation, got {other:?}"),
        }
    }
}
