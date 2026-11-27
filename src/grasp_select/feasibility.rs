//! Feasibility of a hypothesis: seeded endpoint IK as a screen, then the connected trajectory
//! (acquisition, transition, pre-open, insertion, closure, separation, loaded lift, empty
//! withdrawal) through the motion planner's Cartesian path check (seeded IK per sample,
//! branch-jump rejection, swept workspace evaluator), the grasp table clearance and a static
//! payload torque check.

use super::candidates::{Hypothesis, Rejection};
use super::geometry as g;
use super::snapshot::{Snapshot, Target};
use crate::grasp::{self, TABLE_CLEARANCE_M};
use crate::kinematics::{self, Pose, Vec3, PAR};
use crate::motion::{self, GoalFrame, JointPath, MotionError, PathOpts, PlanInput, Speed};

/// Rise from the contact before the loaded lift, straight up (m).
pub const SEPARATION_M: f64 = 0.01;
/// DLS rounds per seed of the endpoint screen ([`PAR`] iterations each).
pub const ENDPOINT_ROUNDS: usize = 8;
/// Characteristic length scaling the angular rows of the TCP Jacobian (m).
pub const JACOBIAN_CHAR_LENGTH_M: f64 = 0.1;
/// Payload bound assumed when the request gives none (kg).
pub const ASSUMED_PAYLOAD_KG: f64 = 0.15;
/// Maximum per-joint deviation (rad) allowed for a planned path endpoint to remain on the
/// selected IK branch (~20 deg: well above IK tolerances, well below a branch flip).
pub const BRANCH_CONSISTENCY_TOL_RAD: f64 = 0.35;
/// Maximum early-contact distance (m) before the nominal grasp pose that is still accepted
/// as target contact (clamped to `0.5 * insertion_m`); earlier contact is classified as an
/// obstruction (`COLLISION`).
pub const CONTACT_WINDOW_M: f64 = 0.015;
/// Maximum lateral TCP error (m) perpendicular to `insertion_dir` at an accepted contact pose.
pub const CONTACT_LATERAL_TOL_M: f64 = 0.006;
/// Maximum TCP orientation error (rad, 3 deg) at an accepted contact pose.
pub const CONTACT_ROT_TOL_RAD: f64 = 3.0 * std::f64::consts::PI / 180.0;
/// Maximum TCP translation between recursively subdivided carried-object checks (m).
pub const VOLUME_SUBDIV_POS_M: f64 = 0.004;
/// Maximum TCP rotation between recursively subdivided carried-object checks (rad, 1 deg).
pub const VOLUME_SUBDIV_ROT_RAD: f64 = 1.0 * std::f64::consts::PI / 180.0;
/// Conservative margin added to the carried-object OBB half-extents to cover sub-interval
/// nonlinear FK excursions plus `START_TOLERANCE` (m).
pub const VOLUME_RESIDUAL_MARGIN_M: f64 = 0.004;
/// Maximum binary subdivision depth per joint-path segment for carried-object checks.
pub const VOLUME_MAX_SUBDIV_DEPTH: u8 = 10;
/// Joint-space step (rad) for swept-joint pre-grasp fallback path sampling.
pub const SWEPT_JOINT_STEP_RAD: f64 = 0.04;

/// Work counters of one selection.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Work {
    pub ik_iterations: u64,
    pub endpoint_solves: usize,
    pub path_checks: usize,
    pub path_samples: usize,
}

/// Error from [`endpoint_ik`]: either generic IK non-convergence or a named planning-limit refusal.
#[derive(Debug, Clone)]
pub enum EndpointIkError {
    Unreachable { ep: f64, er: f64 },
    PlanningLimit(MotionError),
}

impl EndpointIkError {
    pub fn into_rejection(self, label: &str) -> Rejection {
        match self {
            Self::PlanningLimit(e) => motion_rejection("endpoint_ik", &e),
            Self::Unreachable { ep, er } => Rejection::new(
                "endpoint_ik",
                "IK_UNREACHABLE",
                format!("{label}: best IK error {:.4} m / {:.2} deg", ep, er.to_degrees()),
            )
            .with(ep, PAR.tol_pos),
        }
    }
}

/// IK of the TCP pose `goal` from each of `seeds` (first success wins): Err([`EndpointIkError`]).
pub fn endpoint_ik(input: &PlanInput, goal: &Pose, seeds: &[[f64; 6]], work: &mut Work) -> Result<[f64; 6], EndpointIkError> {
    let arm_dof = input.arm.arm_dof();
    let arm_scope = crate::model::PlanningScope::first_n(arm_dof);
    if let Err(e) = motion::check_start_planning_limits_f64(&input.q0, &input.limits, arm_scope, Some(0.0)) {
        return Err(EndpointIkError::PlanningLimit(e));
    }
    let (lo, hi) = motion::ik_search_bounds(&input.limits, arm_dof).map_err(EndpointIkError::PlanningLimit)?;
    let end = motion::tcp_to_end(goal, &input.tcp_offset);
    let mut best = (f64::INFINITY, f64::INFINITY);
    for seed in seeds {
        work.endpoint_solves += 1;
        let mut q: [f64; 6] = std::array::from_fn(|i| seed[i].clamp(lo[i], hi[i]));
        let mut prev = f64::INFINITY;
        for _ in 0..ENDPOINT_ROUNDS {
            let sol = input.arm.solve_dls_ex(&end, &q, &lo, &hi, PAR.w, false);
            work.ik_iterations += sol.iterations as u64;
            if sol.ep + sol.er < best.0 + best.1 { best = (sol.ep, sol.er); }
            q = sol.q;
            if sol.ok {
                if motion::check_planning_limits_f64(&q, &input.limits, arm_scope, None).is_ok() {
                    return Ok(q);
                }
                break;
            }
            let obj = sol.ep + PAR.w * sol.er;
            if obj > prev - 1e-9 { break; }
            prev = obj;
        }
    }
    let has_custom = input.limits.iter().take(arm_dof).any(crate::model::JointLimit::has_custom_planning_limits);
    if has_custom {
        let (hard_lo, hard_hi) = motion::hard_ik_search_bounds(&input.limits, arm_dof);
        for seed in seeds {
            let mut q: [f64; 6] = std::array::from_fn(|i| seed[i].clamp(hard_lo[i], hard_hi[i]));
            let mut prev = f64::INFINITY;
            for _ in 0..ENDPOINT_ROUNDS {
                let sol = input.arm.solve_dls_ex(&end, &q, &hard_lo, &hard_hi, PAR.w, false);
                work.ik_iterations += sol.iterations as u64;
                q = sol.q;
                if sol.ok {
                    let hard_ok = (0..arm_dof).all(|i| {
                        let l = &input.limits[i];
                        q[i].is_finite() && q[i] >= f64::from(l.min_position) && q[i] <= f64::from(l.max_position)
                    });
                    if hard_ok {
                        if let Err(e) = motion::check_planning_limits_f64(&q, &input.limits, arm_scope, None) {
                            return Err(EndpointIkError::PlanningLimit(e));
                        }
                    }
                    break;
                }
                let obj = sol.ep + PAR.w * sol.er;
                if obj > prev - 1e-9 { break; }
                prev = obj;
            }
        }
    }
    Err(EndpointIkError::Unreachable { ep: best.0, er: best.1 })
}

/// A path phase's rejection from a planner error.
pub fn motion_rejection(phase: &'static str, e: &MotionError) -> Rejection {
    let mut r = Rejection::new(phase, e.code(), e.to_string());
    match e {
        MotionError::IkUnreachable { s, ep, .. } => { r.value = Some(*ep); r.bound = Some(PAR.tol_pos); r.s = Some(*s); }
        MotionError::PlanningLimitExceeded { value, min, max, s, .. } => {
            r.value = value.is_finite().then_some(*value);
            r.bound = if value.is_finite() && *value > *max { Some(*max) } else { Some(*min) };
            r.s = *s;
            r.planning_limit = true;
        }
        MotionError::IkBranchFlip { s, dq, .. } => { r.value = Some(*dq); r.bound = Some(motion::MAX_SAMPLE_JUMP_RAD); r.s = Some(*s); }
        MotionError::WorkspaceViolation { s, value, bound, detail, .. } => {
            r.value = Some(*value); r.bound = Some(*bound); r.s = Some(*s);
            r.geometry = detail.geometry_id.clone().or(detail.constraint_id.clone());
        }
        _ => {}
    }
    r
}

/// Planner input at joints `q` with gripper `j7`, goals in the TCP frame.
pub fn input_at(base: &PlanInput, q: [f64; 6], j7: f32) -> PlanInput {
    let mut i = base.clone();
    i.q0 = q;
    i.j7 = j7;
    i.goal_frame = GoalFrame::Tcp;
    i
}

fn q6(q: &[f32; 7]) -> [f64; 6] { std::array::from_fn(|i| f64::from(q[i])) }

/// One straight Cartesian phase from `q` to the TCP `goal`, checked by the motion planner and
/// for table clearance of the envelope `pts`.
pub fn phase(s: &Snapshot, phase: &'static str, q: [f64; 6], j7: f32, goal: Pose, speed: f64, work: &mut Work) -> Result<JointPath, Rejection> {
    let input = input_at(&s.input, q, j7);
    let before = motion::ik_iterations();
    work.path_checks += 1;
    let res = motion::plan_cartesian_opts(&input, goal, Speed { linear: speed, angular: motion::DEFAULT_ANGULAR_RPS }, PathOpts::default());
    work.ik_iterations += motion::ik_iterations() - before;
    let path = res.map_err(|e| motion_rejection(phase, &e))?;
    work.path_samples += path.q.len();
    grasp_table_clear(s, phase, &path)?;
    Ok(path)
}

/// Name of envelope point `i` of [`grasp::envelope_with_contact`]: the TCP, the workspace
/// check points, then (with a lens offset or body size) the lens and the 8 camera body corners,
/// then the physical contact point.
pub fn envelope_point_name(s: &Snapshot, i: usize) -> String {
    let n_ws = s.input.ws.as_ref().map_or(0, |w| w.check_points_end_m.len());
    let cam = s.input.cam.offset_m.is_some() || s.input.cam.body_size_m.is_some();
    match i {
        0 => "tcp".into(),
        i if i <= n_ws => format!("gripper_envelope:{}", i - 1),
        i if cam && i == n_ws + 1 => "camera_lens".into(),
        i if cam && i <= n_ws + 9 => format!("camera_body:{}", i - n_ws - 2),
        _ => "contact_point".into(),
    }
}

/// The grasp's table rule ([`crate::backlash::check_table_clear`]) on a planned path.
pub fn grasp_table_clear(s: &Snapshot, phase: &'static str, path: &JointPath) -> Result<(), Rejection> {
    crate::backlash::check_table_clear(&s.input.arm, &s.envelope, s.setup.table_z_m + TABLE_CLEARANCE_M, path.q.iter().map(|q| q.as_slice()))
        .map_err(|v| {
            let what = envelope_point_name(s, v.point);
            let sv = v.sample_index as f64 / (v.total_samples.max(2) - 1) as f64;
            let mut r = Rejection::new(phase, "WORKSPACE_VIOLATION", format!("{what} at z {:.4} m, below the table bound {:.4} m (s={sv:.2})",
                v.value_m, v.bound_m)).with(v.value_m - v.bound_m, 0.0);
            r.geometry = Some(what);
            r.s = Some(sv);
            r
        })
}

/// The acquisition prefix shared by every candidate: the legacy approach of
/// [`grasp::plan`] (camera-down or look-down above the box-centre target). Returns the
/// joints at its end, the end pose and the path.
pub struct Acquisition { pub q_end: [f64; 6], pub pose_end: Pose, pub path: JointPath, pub target: Vec3 }

pub fn acquisition(s: &Snapshot, work: &mut Work) -> Result<Acquisition, Rejection> {
    let setup = &s.setup;
    let pose_end = s.input.arm.fk(&s.input.q0);
    let centre = s.cam.to_px(grasp::box_centre(&s.box_norm));
    let target = s.cam.plane_point(&pose_end, centre, setup.table_z_m + setup.grasp_height_m)
        .ok_or_else(|| Rejection::new("acquisition", "INVALID_COMMAND", "the box centre ray does not meet the table plane in front of the camera"))?;
    let approach = |r: &kinematics::Mat3| Pose::new(grasp::look_down(r), [target[0], target[1], setup.table_z_m + setup.approach_height_m]);
    let (goal, frame) = if s.camera_down {
        let goal = grasp::camera_down_pose(&pose_end.r, &target, setup, &s.cam, &s.envelope);
        let tool_end = motion::tcp_to_end(&approach(&goal.r), &s.input.tcp_offset);
        if let Some(e) = grasp::table_violation(&goal, &tool_end, &s.envelope, setup.table_z_m) { return Err(motion_rejection("acquisition", &e)); }
        (goal, GoalFrame::EndLink)
    } else {
        (approach(&pose_end.r), GoalFrame::Tcp)
    };
    let goal_end = if frame == GoalFrame::EndLink { goal } else { motion::tcp_to_end(&goal, &s.input.tcp_offset) };
    if let Some(e) = grasp::table_violation(&pose_end, &goal_end, &s.envelope, setup.table_z_m) { return Err(motion_rejection("acquisition", &e)); }
    let mut input = input_at(&s.input, s.input.q0, s.open_rad);
    input.goal_frame = frame;
    let before = motion::ik_iterations();
    work.path_checks += 1;
    let res = motion::plan_cartesian_opts(&input, goal, Speed { linear: setup.approach_speed_mps, angular: motion::DEFAULT_ANGULAR_RPS }, PathOpts::default());
    work.ik_iterations += motion::ik_iterations() - before;
    let path = res.map_err(|e| motion_rejection("acquisition", &e))?;
    work.path_samples += path.q.len();
    grasp_table_clear(s, "acquisition", &path)?;
    let q_end = q6(&path.last());
    Ok(Acquisition { q_end, pose_end: s.input.arm.fk(&q_end), path, target })
}

/// Margins along the checked paths.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Margins {
    /// Smallest distance to a joint limit over the half range, 0..1.
    pub joint_limit: f64,
    /// Smallest singular value of the TCP Jacobian (angular rows × [`JACOBIAN_CHAR_LENGTH_M`]), m/rad.
    pub sigma_min: f64,
    /// Smallest table clearance of the envelope and fingers at the contact (m).
    pub table_clearance_m: f64,
    /// Largest payload (kg) the derated holding torques allow along the loaded phases; None
    /// without torque limits.
    pub payload_capacity_kg: Option<f64>,
}

/// Smallest normalized joint-limit margin of `q` (against effective planning limits).
pub fn joint_margin(input: &PlanInput, q: &[f64; 6]) -> f64 {
    (0..input.arm.arm_dof()).map(|i| {
        let l = &input.limits[i];
        let Some((lo32, hi32)) = l.effective_planning_range() else { return 0.0; };
        let (lo, hi) = (f64::from(lo32), f64::from(hi32));
        ((q[i] - lo).min(hi - q[i]) / ((hi - lo) / 2.0)).clamp(0.0, 1.0)
    }).fold(1.0, f64::min)
}

/// Geometric TCP Jacobian columns `[v; w]` of the arm joints at `q`.
fn tcp_jacobian(input: &PlanInput, q: &[f64; 6], point_end: &Vec3) -> Option<Vec<[f64; 6]>> {
    let q32: [f32; 6] = std::array::from_fn(|i| q[i] as f32);
    let f = input.arm.frames_f32(&q32)?;
    let p = f.end.transform_point(point_end);
    Some((0..f.n).map(|j| {
        let v = kinematics::cross(&f.axes[j], &kinematics::sub(&p, &f.origins[j]));
        let a = f.axes[j];
        [v[0], v[1], v[2], a[0], a[1], a[2]]
    }).collect())
}

/// Smallest singular value of the TCP Jacobian (angular rows scaled by [`JACOBIAN_CHAR_LENGTH_M`]).
pub fn sigma_min(input: &PlanInput, q: &[f64; 6]) -> f64 {
    let Some(j) = tcp_jacobian(input, q, &input.tcp_offset) else { return 0.0 };
    let n = j.len();
    let l = JACOBIAN_CHAR_LENGTH_M;
    let col = |c: usize| -> [f64; 6] { let v = j[c]; [v[0], v[1], v[2], v[3] * l, v[4] * l, v[5] * l] };
    let mut m = vec![0.0; n * n];
    for a in 0..n { for b in 0..n {
        let (x, y) = (col(a), col(b));
        m[a * n + b] = (0..6).map(|k| x[k] * y[k]).sum();
    }}
    g::sym_eigenvalues(&mut m, n).into_iter().fold(f64::INFINITY, f64::min).max(0.0).sqrt()
}

/// Joint holding torque per kg of payload at the base-frame point `p` (end_link frames at `q`).
fn payload_torque_per_kg_arm(arm: &crate::chain::ArmModel, q: &[f64; 6], p: &Vec3) -> Option<Vec<f64>> {
    let q32: [f32; 6] = std::array::from_fn(|i| q[i] as f32);
    let f = arm.frames_f32(&q32)?;
    // the motor holds the weight: τ = Jᵥᵀ (m g ẑ)
    Some((0..f.n).map(|j| kinematics::cross(&f.axes[j], &kinematics::sub(p, &f.origins[j]))[2] * crate::chain::G).collect())
}

#[cfg(test)]
fn payload_torque_per_kg(input: &PlanInput, q: &[f64; 6], p: &Vec3) -> Option<Vec<f64>> {
    payload_torque_per_kg_arm(&input.arm, q, p)
}

/// Largest payload the torque limits allow over the loaded samples `qs`, with the payload's
/// centre of mass at `com_tcp` (TCP frame). Negative when the arm alone exceeds a limit.
pub fn payload_capacity_arm(
    arm_model: &crate::chain::ArmModel,
    tcp_offset: &Vec3,
    torque_limits: Option<&[f64]>,
    qs: &[[f64; 6]],
    com_tcp: &Vec3,
) -> Option<(f64, usize)> {
    let lim = torque_limits?;
    let mut cap = f64::INFINITY;
    let mut joint = 0;
    for q in qs {
        let q32: [f32; 7] = std::array::from_fn(|i| if i < 6 { q[i] as f32 } else { 0.0 });
        let arm = arm_model.gravity_torques(&q32).ok()?;
        let tcp = motion::end_to_tcp(&arm_model.fk(q), tcp_offset);
        let p = tcp.transform_point(com_tcp);
        let per = payload_torque_per_kg_arm(arm_model, q, &p)?;
        for i in 0..6.min(lim.len()) {
            let t_arm = f64::from(arm[i]);
            let room = if t_arm.abs() > lim[i] {
                -1.0
            } else if per[i].abs() < 1e-9 {
                f64::INFINITY
            } else if per[i] * t_arm >= 0.0 {
                // |t_arm + m per| <= lim for m in [0, cap]
                (lim[i] - t_arm.abs()) / per[i].abs()
            } else {
                (lim[i] + t_arm.abs()) / per[i].abs()
            };
            if room < cap { cap = room; joint = i; }
        }
    }
    Some((cap, joint))
}

/// Largest payload the torque limits allow over the loaded samples `qs`, with the payload's
/// centre of mass at `com_tcp` (TCP frame). Negative when the arm alone exceeds a limit.
pub fn payload_capacity(s: &Snapshot, qs: &[[f64; 6]], com_tcp: &Vec3) -> Option<(f64, usize)> {
    payload_capacity_arm(
        &s.input.arm,
        &s.input.tcp_offset,
        s.torque_limits.as_ref().map(|(l, _)| l.as_slice()),
        qs,
        com_tcp,
    )
}

/// Result of the path checks of one hypothesis.
#[derive(Debug, Clone)]
pub struct Checked {
    pub q_pre: [f64; 6],
    pub q_grasp: [f64; 6],
    pub q_lift: [f64; 6],
    pub separation: Pose,
    pub lift: Pose,
    pub duration_s: f64,
    pub joint_travel_rad: f64,
    pub margins: Margins,
    pub phases: Vec<&'static str>,
    /// The candidate passes only with no more than the assumed payload bound.
    pub payload_assumed: bool,
    pub insufficient_evidence: Option<String>,
}

fn travel(p: &JointPath) -> f64 {
    p.q.windows(2).map(|w| (0..p.arm_dof).map(|i| f64::from((w[1][i] - w[0][i]).abs())).sum::<f64>()).sum()
}

/// Full path check of `hy` from the acquisition's end.
pub fn check(s: &Snapshot, t: &Target, acq: &Acquisition, hy: &Hypothesis, work: &mut Work) -> Result<Checked, Rejection> {
    let st = &s.setup;
    let pre = hy.pregrasp_pose();
    let grasp_pose = hy.grasp_pose();
    let mut phases = vec!["acquisition"];
    let transition = phase(s, "transition", acq.q_end, s.open_rad, pre, st.approach_speed_mps, work)?;
    phases.push("transition");
    let q_pre = q6(&transition.last());
    // pre-open happens at the pre-grasp pose: its finger volumes were checked against the
    // object, the envelope against the table along the transition
    phases.push("pre_open");
    let insertion = phase(s, "insertion", q_pre, s.open_rad, grasp_pose, st.descend_speed_mps, work)?;
    phases.push("insertion");
    let q_grasp = q6(&insertion.last());
    phases.push("closure");
    let sep = Pose::new(hy.r, kinematics::add(&hy.g, &[0.0, 0.0, SEPARATION_M]));
    let separation = phase(s, "separation", q_grasp, s.closed_rad, sep, st.descend_speed_mps, work)?;
    phases.push("separation");
    let q_sep = q6(&separation.last());
    let lift_z = (st.table_z_m + st.lift_height_m).max(sep.p[2]);
    let lift = Pose::new(hy.r, [hy.g[0], hy.g[1], lift_z]);
    let (lift_path, q_lift) = if lift_z > sep.p[2] + 1e-4 {
        let p = phase(s, "lift", q_sep, s.closed_rad, lift, st.descend_speed_mps, work)?;
        let q = q6(&p.last());
        (Some(p), q)
    } else { (None, q_sep) };
    // the carried object stays clear of keep-out boxes during the lift
    let rt = kinematics::transpose(&hy.r);
    let obj_tcp: Vec<Vec3> = t.poly.iter().flat_map(|v| [t.table_z + 0.002, t.table_z + t.height[1]].map(|z|
        kinematics::mat_vec(&rt, &kinematics::sub(&[v[0], v[1], z], &hy.g)))).collect();
    if let (Some(ws), Some(lp)) = (s.input.ws.as_ref(), lift_path.as_ref()) {
        for q in &lp.q {
            let tcp = motion::end_to_tcp(&s.input.arm.fk(&q6(q)), &s.input.tcp_offset);
            for (k, b) in ws.keepout_boxes.iter().enumerate() {
                if let Some(p) = obj_tcp.iter().map(|o| tcp.transform_point(o)).find(|p| (0..3).all(|i| p[i] >= f64::from(b.min_m[i]) && p[i] <= f64::from(b.max_m[i]))) {
                    return Err(Rejection::new("lift", "WORKSPACE_VIOLATION", format!("the carried object enters keep-out box {k} ({}) at ({:.3}, {:.3}, {:.3})", b.name, p[0], p[1], p[2])));
                }
            }
        }
    }
    phases.push("lift");
    // payload: static holding torque at the contact and along separation and lift
    let mut loaded: Vec<[f64; 6]> = separation.q.iter().map(q6).collect();
    if let Some(lp) = &lift_path { loaded.extend(lp.q.iter().map(q6)); }
    let com = [t.centroid[0], t.centroid[1], t.table_z + (t.height[0] + t.height[1]) / 4.0];
    let com_tcp = kinematics::mat_vec(&rt, &kinematics::sub(&com, &hy.g));
    let capacity = payload_capacity(s, &loaded, &com_tcp);
    let mut payload_assumed = false;
    let mut insufficient = None;
    if let Some((cap, joint)) = capacity {
        let (bound, given) = match s.evidence.payload_kg { Some(m) => (m, true), None => (ASSUMED_PAYLOAD_KG, false) };
        if cap < 0.0 {
            return Err(Rejection::new("lift", "LIFT_TORQUE", format!("J{} holding torque exceeds its derated limit without payload", joint + 1)).with(cap, 0.0));
        }
        if cap < bound {
            if given {
                return Err(Rejection::new("lift", "LIFT_TORQUE", format!("J{} allows {cap:.3} kg of payload, the bound is {bound:.3} kg", joint + 1)).with(cap, bound));
            }
            insufficient = Some(format!("J{} allows {cap:.3} kg; the payload bound is unknown (assumed {bound:.2} kg)", joint + 1));
        }
        payload_assumed = !given;
    }
    // empty withdrawal after a missed grasp: back out along -a with the jaws closed
    let withdrawal = phase(s, "withdrawal", q_grasp, s.closed_rad, pre, st.descend_speed_mps, work)?;
    phases.push("withdrawal");
    let paths: Vec<&JointPath> = [Some(&transition), Some(&insertion), Some(&separation), lift_path.as_ref()].into_iter().flatten().collect();
    let mut jm = 1.0f64;
    let mut sig = f64::INFINITY;
    for p in paths.iter().copied().chain(std::iter::once(&withdrawal)) {
        for (k, q) in p.q.iter().enumerate() {
            let q = q6(q);
            jm = jm.min(joint_margin(&s.input, &q));
            if k % 3 == 0 || k + 1 == p.q.len() { sig = sig.min(sigma_min(&s.input, &q)); }
        }
    }
    let duration_s = paths.iter().map(|p| p.duration).sum::<f64>();
    let joint_travel_rad = paths.iter().map(|p| travel(p)).sum::<f64>();
    Ok(Checked {
        q_pre, q_grasp, q_lift, separation: sep, lift, duration_s, joint_travel_rad,
        margins: Margins { joint_limit: jm, sigma_min: sig, table_clearance_m: hy.table_clearance_m, payload_capacity_kg: capacity.map(|c| c.0) },
        phases, payload_assumed, insufficient_evidence: insufficient,
    })
}

// ---- Phase B conservative geometry, carried-object SAT, payload envelope & swept-joint paths ----

/// Conservative TCP-attached oriented bounding box (OBB) of the carried object.
///
/// Storing the object's own principal-axis vertical-aligned orientation (`axes_tcp = R_grasp^T * R_obj`)
/// avoids the large downward phantom-corner over-approximation that a tilted TCP-frame AABB
/// introduces for oblique grasps of elongated objects resting flat on the table, while still
/// transforming exactly under rigid SE(3) contact deviations (`transformed_by_relative_pose`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CarriedObjectBox {
    pub center_tcp: Vec3,
    pub axes_tcp: kinematics::Mat3,
    pub half: Vec3,
}

impl CarriedObjectBox {
    /// Axis-aligned convenience constructor in TCP coordinates (used in tests).
    #[cfg(test)]
    pub fn from_tcp_aabb(min_tcp: Vec3, max_tcp: Vec3) -> Self {
        Self {
            center_tcp: std::array::from_fn(|i| 0.5 * (min_tcp[i] + max_tcp[i])),
            axes_tcp: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            half: std::array::from_fn(|i| (0.5 * (max_tcp[i] - min_tcp[i])).max(0.0)),
        }
    }

    /// Conservative object-aligned OBB attached to `grasp_pose` enclosing the target's full
    /// low-to-high projection frustum hull (`poly_lo` at `table_z`, `poly_hi` at
    /// `table_z + height[1] + TABLE_ERROR_M`, and `poly` at both planes), horizontally inflated
    /// by `hard_error_m`. Does NOT exclude the bottom 2 mm at the table.
    pub fn from_target_grasp(t: &Target, grasp_pose: &Pose) -> Self {
        let u2 = g::unit2(t.axis);
        let u = if g::norm2(u2) > 1e-6 { [u2[0], u2[1], 0.0] } else { [1.0, 0.0, 0.0] };
        let v = [-u[1], u[0], 0.0];
        let z_ax = [0.0, 0.0, 1.0];
        // World-to-object rotation matrix R_obj (columns = u, v, z_ax)
        let r_obj: kinematics::Mat3 = [
            [u[0], v[0], z_ax[0]],
            [u[1], v[1], z_ax[1]],
            [u[2], v[2], z_ax[2]],
        ];
        let z0 = t.table_z;
        let z1 = t.table_z + t.height[1] + super::snapshot::TABLE_ERROR_M;
        let e = t.hard_error_m.max(0.0);
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        let mut include = |pt: [f64; 2], z: f64| {
            let pu = pt[0] * u[0] + pt[1] * u[1];
            let pv = pt[0] * v[0] + pt[1] * v[1];
            lo[0] = lo[0].min(pu - e);
            hi[0] = hi[0].max(pu + e);
            lo[1] = lo[1].min(pv - e);
            hi[1] = hi[1].max(pv + e);
            lo[2] = lo[2].min(z);
            hi[2] = hi[2].max(z);
        };
        for &pt in &t.poly_lo {
            include(pt, z0);
        }
        for &pt in &t.poly_hi {
            include(pt, z1);
        }
        for &pt in &t.poly {
            include(pt, z0);
            include(pt, z1);
        }
        let c_obj: Vec3 = std::array::from_fn(|i| 0.5 * (lo[i] + hi[i]));
        let half: Vec3 = std::array::from_fn(|i| (0.5 * (hi[i] - lo[i])).max(0.0));
        let c_w = kinematics::mat_vec(&r_obj, &c_obj);
        let rt = kinematics::transpose(&grasp_pose.r);
        Self {
            center_tcp: kinematics::mat_vec(&rt, &kinematics::sub(&c_w, &grasp_pose.p)),
            axes_tcp: kinematics::mat_mul(&rt, &r_obj),
            half,
        }
    }

    /// Exact SE(3) rigid transform of this OBB by
    /// `t_actual_to_nominal = actual_contact_tcp^{-1} * nominal_grasp_pose`.
    pub fn transformed_by_relative_pose(&self, t_actual_to_nominal: &Pose) -> Self {
        Self {
            center_tcp: t_actual_to_nominal.transform_point(&self.center_tcp),
            axes_tcp: kinematics::mat_mul(&t_actual_to_nominal.r, &self.axes_tcp),
            half: self.half,
        }
    }

    /// Expands this OBB to conservatively enclose all accepted contact poses within
    /// `x in [0, contact_window_m]`, `y, z in [-lateral_tol_m, +lateral_tol_m]`, and any
    /// contact orientation deviation up to `rot_tol_rad`.
    pub fn expanded_by_contact_envelope(
        &self,
        contact_window_m: f64,
        lateral_tol_m: f64,
        rot_tol_rad: f64,
    ) -> Self {
        let half_x = 0.5 * contact_window_m.max(0.0);
        let lat = lateral_tol_m.max(0.0);
        // Shift centre forward along TCP +x by half_x, and project the translation box
        // `[-half_x..+half_x, -lat..+lat, -lat..+lat]` onto each OBB local axis.
        let mut center_tcp = self.center_tcp;
        center_tcp[0] += half_x;
        let mut half = self.half;
        for j in 0..3 {
            let ext_j = self.axes_tcp[0][j].abs() * half_x
                + self.axes_tcp[1][j].abs() * lat
                + self.axes_tcp[2][j].abs() * lat;
            half[j] += ext_j;
        }
        let shifted = Self {
            center_tcp,
            axes_tcp: self.axes_tcp,
            half,
        };
        let rot_arc = shifted.max_corner_radius() * 2.0 * (0.5 * rot_tol_rad.max(0.0)).sin();
        shifted.inflated(rot_arc)
    }

    /// Expands this OBB symmetrically by `margin` on every local axis.
    pub fn inflated(&self, margin: f64) -> Self {
        let m = margin.max(0.0);
        Self {
            center_tcp: self.center_tcp,
            axes_tcp: self.axes_tcp,
            half: self.half.map(|h| h + m),
        }
    }

    /// All 8 corners of the OBB in TCP coordinates (exact extrema for affine payload torque).
    pub fn corners_tcp(&self) -> [Vec3; 8] {
        let mut out = [[0.0; 3]; 8];
        let mut idx = 0;
        for sx in [-1.0, 1.0] {
            for sy in [-1.0, 1.0] {
                for sz in [-1.0, 1.0] {
                    out[idx] = std::array::from_fn(|i| {
                        self.center_tcp[i]
                            + sx * self.half[0] * self.axes_tcp[i][0]
                            + sy * self.half[1] * self.axes_tcp[i][1]
                            + sz * self.half[2] * self.axes_tcp[i][2]
                    });
                    idx += 1;
                }
            }
        }
        out
    }

    /// Largest Euclidean distance (m) of any OBB corner from the TCP origin.
    pub fn max_corner_radius(&self) -> f64 {
        self.corners_tcp()
            .iter()
            .map(kinematics::norm)
            .fold(0.0, f64::max)
    }
}

/// Exact 15-axis Separating Axis Theorem (SAT) overlap test between a base-frame OBB
/// (`center`, column-axis orthonormal `axes`, `half`) and an axis-aligned box
/// `[aabb_min, aabb_max]`. Returns `true` iff the two closed volumes intersect.
pub fn obb_intersects_aabb(
    center: &Vec3,
    axes: &kinematics::Mat3,
    half: &Vec3,
    aabb_min: &Vec3,
    aabb_max: &Vec3,
) -> bool {
    let b_c: Vec3 = std::array::from_fn(|i| 0.5 * (aabb_min[i] + aabb_max[i]));
    let b_h: Vec3 = std::array::from_fn(|i| (0.5 * (aabb_max[i] - aabb_min[i])).max(0.0));
    let t = kinematics::sub(center, &b_c);
    // Rotation matrix R[i][j] = world_i . obb_axis_j = axes[i][j] (columns are OBB axes).
    let eps = 1e-9;
    let mut abs_r = [[0.0f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            abs_r[i][j] = axes[i][j].abs() + eps;
        }
    }
    // 1) World axes e0, e1, e2
    for i in 0..3 {
        let ra = b_h[i];
        let rb = half[0] * abs_r[i][0] + half[1] * abs_r[i][1] + half[2] * abs_r[i][2];
        if t[i].abs() > ra + rb { return false; }
    }
    // 2) OBB local axes u0, u1, u2
    for j in 0..3 {
        let proj_t = (t[0] * axes[0][j] + t[1] * axes[1][j] + t[2] * axes[2][j]).abs();
        let ra = b_h[0] * abs_r[0][j] + b_h[1] * abs_r[1][j] + b_h[2] * abs_r[2][j];
        let rb = half[j];
        if proj_t > ra + rb { return false; }
    }
    // 3) 9 cross-product axes e_i x u_j
    for i in 0..3 {
        let i1 = (i + 1) % 3;
        let i2 = (i + 2) % 3;
        for j in 0..3 {
            let j1 = (j + 1) % 3;
            let j2 = (j + 2) % 3;
            let proj_t = (t[i2] * axes[i1][j] - t[i1] * axes[i2][j]).abs();
            let ra = b_h[i1] * abs_r[i2][j] + b_h[i2] * abs_r[i1][j];
            let rb = half[j1] * abs_r[i][j2] + half[j2] * abs_r[i][j1];
            if proj_t > ra + rb { return false; }
        }
    }
    true
}

/// World-space pose and conservative AABB of a TCP-attached OBB at TCP pose `tcp`.
fn obb_world_frame_and_aabb(
    tcp: &Pose,
    obj_box: &CarriedObjectBox,
    half: &Vec3,
) -> (Vec3, kinematics::Mat3, Vec3, Vec3) {
    let c_w = tcp.transform_point(&obj_box.center_tcp);
    let axes_w = kinematics::mat_mul(&tcp.r, &obj_box.axes_tcp);
    let ext: Vec3 = std::array::from_fn(|i| {
        axes_w[i][0].abs() * half[0]
            + axes_w[i][1].abs() * half[1]
            + axes_w[i][2].abs() * half[2]
    });
    (c_w, axes_w, kinematics::sub(&c_w, &ext), kinematics::add(&c_w, &ext))
}

/// Conservative swept volume check for the carried object along `path` (covers BOTH `separation`
/// and `lift`), using recursive joint-interval subdivision bounded by actual worst-case corner
/// displacement (`dp + r_max * 2 * sin(dr / 2) <= VOLUME_RESIDUAL_MARGIN_M`) across the commanded
/// linear joint interpolation (always evaluating the interval midpoint `q_m`), exact 15-axis
/// OBB-vs-AABB SAT against all `keepout_boxes`, workspace bounds, and validated workspace floor.
pub fn check_carried_object_volume(
    arm: &crate::chain::ArmModel,
    tcp_offset: &Vec3,
    ws: &crate::model::WorkspacePolicy,
    table_z_m: f64,
    obj_box: &CarriedObjectBox,
    path: &JointPath,
    phase: &'static str,
) -> Result<(), Rejection> {
    if path.q.is_empty() {
        return Ok(());
    }
    let half_nom = obj_box.half;
    let half_inflated = half_nom.map(|h| h + VOLUME_RESIDUAL_MARGIN_M);
    let r_max = obj_box.max_corner_radius() + VOLUME_RESIDUAL_MARGIN_M;
    let tcp_of = |q: &[f64; 6]| motion::end_to_tcp(&arm.fk(q), tcp_offset);
    let corner_disp = |ta: &Pose, tb: &Pose| -> f64 {
        let dp = kinematics::norm(&kinematics::sub(&tb.p, &ta.p));
        let dr = kinematics::angle_between(&ta.r, &tb.r);
        dp + r_max * 2.0 * (0.5 * dr).sin()
    };
    let q0 = q6(&path.q[0]);
    let tcp0 = tcp_of(&q0);
    let (_, _, wmin0_nom, _) = obb_world_frame_and_aabb(&tcp0, obj_box, &half_nom);
    let ws_floor = f64::from(ws.floor_z_m);
    // Strict configured floor enforcement: only grant a bounded resting-contact allowance when
    // the validated table sits at or above `ws.floor_z_m` (within TABLE_ERROR_M), and bound that
    // allowance strictly to the contact-window/table-error expansion below `table_z_m` so a
    // genuinely below-floor starting envelope can never pass.
    let resting_allowance = if table_z_m >= ws_floor - super::snapshot::TABLE_ERROR_M {
        let max_below_table = CONTACT_WINDOW_M + super::snapshot::TABLE_ERROR_M + CONTACT_LATERAL_TOL_M + 0.015;
        wmin0_nom[2].max(table_z_m - max_below_table)
    } else {
        ws_floor
    };
    let floor_bound = ws_floor.min(resting_allowance) - 0.001;
    let ws_min: Vec3 = ws.box_min_m.map(f64::from);
    let ws_max: Vec3 = ws.box_max_m.map(f64::from);

    let check_pose = |tcp: &Pose, s_param: f64| -> Result<(), Rejection> {
        let (c_w, axes_w, wmin_inf, wmax_inf) = obb_world_frame_and_aabb(tcp, obj_box, &half_inflated);
        let (_, _, wmin_nom, _) = obb_world_frame_and_aabb(tcp, obj_box, &half_nom);
        for (k, b) in ws.keepout_boxes.iter().enumerate() {
            let bmin = b.min_m.map(f64::from);
            let bmax = b.max_m.map(f64::from);
            if obb_intersects_aabb(&c_w, &axes_w, &half_inflated, &bmin, &bmax) {
                let mut r = Rejection::new(
                    phase,
                    "WORKSPACE_VIOLATION",
                    format!(
                        "carried object volume intersects keep-out box {k} ({}) at s={s_param:.2}",
                        b.name
                    ),
                );
                r.geometry = Some(format!("carried_object:keepout:{k}"));
                r.s = Some(s_param);
                return Err(r);
            }
        }
        for axis in 0..3 {
            if wmin_inf[axis] < ws_min[axis] {
                let mut r = Rejection::new(
                    phase,
                    "WORKSPACE_VIOLATION",
                    format!(
                        "carried object volume leaves workspace min axis {axis} ({:.4} < {:.4} m) at s={s_param:.2}",
                        wmin_inf[axis], ws_min[axis]
                    ),
                )
                .with(wmin_inf[axis], ws_min[axis]);
                r.geometry = Some("carried_object:workspace_min".into());
                r.s = Some(s_param);
                return Err(r);
            }
            if wmax_inf[axis] > ws_max[axis] {
                let mut r = Rejection::new(
                    phase,
                    "WORKSPACE_VIOLATION",
                    format!(
                        "carried object volume leaves workspace max axis {axis} ({:.4} > {:.4} m) at s={s_param:.2}",
                        wmax_inf[axis], ws_max[axis]
                    ),
                )
                .with(wmax_inf[axis], ws_max[axis]);
                r.geometry = Some("carried_object:workspace_max".into());
                r.s = Some(s_param);
                return Err(r);
            }
        }
        if wmin_nom[2] < floor_bound {
            let mut r = Rejection::new(
                phase,
                "WORKSPACE_VIOLATION",
                format!(
                    "carried object volume drops below floor ({:.4} < {:.4} m) at s={s_param:.2}",
                    wmin_nom[2], floor_bound
                ),
            )
            .with(wmin_nom[2], floor_bound);
            r.geometry = Some("carried_object:floor".into());
            r.s = Some(s_param);
            return Err(r);
        }
        Ok(())
    };

    check_pose(&tcp0, 0.0)?;
    let total = path.q.len().saturating_sub(1).max(1) as f64;
    for k in 0..path.q.len().saturating_sub(1) {
        let qa = q6(&path.q[k]);
        let qb = q6(&path.q[k + 1]);
        let sa = path.s.get(k).copied().unwrap_or(k as f64 / total);
        let sb = path.s.get(k + 1).copied().unwrap_or((k + 1) as f64 / total);
        let ta = tcp_of(&qa);
        let tb = tcp_of(&qb);
        let mut stack: Vec<([f64; 6], Pose, f64, [f64; 6], Pose, f64, u8)> =
            vec![(qa, ta, sa, qb, tb, sb, 0)];
        while let Some((q_l, t_l, s_l, q_r, t_r, s_r, depth)) = stack.pop() {
            // Always sample the midpoint of the commanded linear joint interpolation so curved
            // intermediate excursions cannot hide behind matching endpoints.
            let q_m: [f64; 6] = std::array::from_fn(|i| 0.5 * (q_l[i] + q_r[i]));
            let t_m = tcp_of(&q_m);
            let s_m = 0.5 * (s_l + s_r);
            check_pose(&t_m, s_m)?;
            let d_lm = corner_disp(&t_l, &t_m);
            let d_mr = corner_disp(&t_m, &t_r);
            let d_lr = corner_disp(&t_l, &t_r);
            if d_lm <= VOLUME_RESIDUAL_MARGIN_M
                && d_mr <= VOLUME_RESIDUAL_MARGIN_M
                && d_lr <= VOLUME_RESIDUAL_MARGIN_M * 1.5
            {
                check_pose(&t_r, s_r)?;
                continue;
            }
            if depth >= VOLUME_MAX_SUBDIV_DEPTH {
                // Fail closed unless both half-interval corner displacements stay strictly inside
                // the residual margin (never silently accepting large corner arcs).
                if d_lm > VOLUME_RESIDUAL_MARGIN_M || d_mr > VOLUME_RESIDUAL_MARGIN_M {
                    let mut r = Rejection::new(
                        phase,
                        "WORKSPACE_VIOLATION",
                        format!(
                            "carried object sweep failed to converge on segment s={s_l:.2}..{s_r:.2} (corner_disp={:.4}/{:.4} m > {:.4} m)",
                            d_lm, d_mr, VOLUME_RESIDUAL_MARGIN_M
                        ),
                    );
                    r.geometry = Some("carried_object:subdiv".into());
                    r.s = Some(s_m);
                    return Err(r);
                }
                check_pose(&t_r, s_r)?;
                continue;
            }
            stack.push((q_m, t_m, s_m, q_r, t_r, s_r, depth + 1));
            stack.push((q_l, t_l, s_l, q_m, t_m, s_m, depth + 1));
        }
    }
    Ok(())
}

/// Exact global worst-case payload capacity (kg) over the entire CoM uncertainty volume
/// `obj_box` across all loaded joint samples `qs` (arm-only inputs, shared by preflight and live).
pub fn payload_capacity_box_arm(
    arm: &crate::chain::ArmModel,
    tcp_offset: &Vec3,
    torque_limits: Option<&[f64]>,
    qs: &[[f64; 6]],
    obj_box: &CarriedObjectBox,
) -> Option<(f64, usize)> {
    let corners = obj_box.corners_tcp();
    let mut worst: Option<(f64, usize)> = None;
    for corner in &corners {
        let (cap, joint) = payload_capacity_arm(arm, tcp_offset, torque_limits, qs, corner)?;
        if worst.is_none_or(|(w, _)| cap < w) {
            worst = Some((cap, joint));
        }
    }
    worst
}

/// Exact global worst-case payload capacity (kg) over the entire CoM uncertainty volume
/// `obj_box` across all loaded joint samples `qs`.
pub fn payload_capacity_box(
    s: &Snapshot,
    qs: &[[f64; 6]],
    obj_box: &CarriedObjectBox,
) -> Option<(f64, usize)> {
    payload_capacity_box_arm(
        &s.input.arm,
        &s.input.tcp_offset,
        s.torque_limits.as_ref().map(|(l, _)| l.as_slice()),
        qs,
        obj_box,
    )
}

/// Parameters for the single converged loaded-leg validator shared across nominal preflight,
/// guarded-contact preflight alternatives, and live adaptive `separation` / `lift` replanning.
pub struct LoadedLegParams<'a> {
    pub arm: &'a crate::chain::ArmModel,
    pub tcp_offset: &'a Vec3,
    pub ws: Option<&'a crate::model::WorkspacePolicy>,
    pub envelope: &'a [Vec3],
    pub table_z_m: f64,
    pub phase_timeout_s: f64,
    pub obj_box: &'a CarriedObjectBox,
    pub payload_kg: Option<f64>,
    pub torque_limits: Option<&'a [f64]>,
    pub has_gravity_model: bool,
    /// If true (live execution), an insufficient-evidence torque bound fails closed immediately.
    pub fail_on_insufficient_evidence: bool,
}

#[derive(Debug, Clone)]
pub struct LoadedLegVerdict {
    pub payload_capacity: Option<(f64, usize)>,
    pub insufficient_evidence: Option<String>,
}

/// Single authoritative validator for ANY loaded adaptive leg (`separation` or `lift`):
/// enforces per-child duration+settle timeout, grasp table clearance, swept carried-object OBB
/// SAT + workspace floor, AND 8-corner payload holding torque.
pub fn validate_loaded_leg(
    params: &LoadedLegParams<'_>,
    path: &JointPath,
    phase: &'static str,
) -> Result<LoadedLegVerdict, Rejection> {
    if path.duration + 0.5 > params.phase_timeout_s {
        return Err(Rejection::new(
            phase,
            "GRASP_TIMEOUT",
            format!(
                "{phase} duration {:.2} s + settle exceeds phase_timeout_s {:.2} s",
                path.duration, params.phase_timeout_s
            ),
        )
        .with(path.duration + 0.5, params.phase_timeout_s));
    }
    crate::backlash::check_table_clear(
        params.arm,
        params.envelope,
        params.table_z_m + TABLE_CLEARANCE_M,
        path.q.iter().map(|q| q.as_slice()),
    )
    .map_err(|v| motion_rejection(phase, &v.into_motion_error()))?;
    if let Some(ws) = params.ws {
        check_carried_object_volume(
            params.arm,
            params.tcp_offset,
            ws,
            params.table_z_m,
            params.obj_box,
            path,
            phase,
        )?;
    }
    let qs: Vec<[f64; 6]> = path.q.iter().map(q6).collect();
    let capacity = payload_capacity_box_arm(
        params.arm,
        params.tcp_offset,
        params.torque_limits,
        &qs,
        params.obj_box,
    );
    let mut insufficient = None;
    match capacity {
        Some((cap, joint)) => {
            let (bound, given) = match params.payload_kg {
                Some(m) => (m, true),
                None => (ASSUMED_PAYLOAD_KG, false),
            };
            if cap < 0.0 {
                return Err(Rejection::new(
                    "lift",
                    "LIFT_TORQUE",
                    format!("J{} holding torque exceeds its derated limit without payload", joint + 1),
                )
                .with(cap, 0.0));
            }
            if cap < bound {
                if given {
                    return Err(Rejection::new(
                        "lift",
                        "LIFT_TORQUE",
                        format!(
                            "J{} allows {cap:.3} kg of payload across CoM envelope, bound is {bound:.3} kg",
                            joint + 1
                        ),
                    )
                    .with(cap, bound));
                }
                let msg = format!(
                    "J{} allows {cap:.3} kg across CoM envelope; payload bound is unknown (assumed {bound:.2} kg)",
                    joint + 1
                );
                if params.fail_on_insufficient_evidence {
                    return Err(Rejection::new("lift", "LIFT_TORQUE", msg).with(cap, bound));
                }
                insufficient = Some(msg);
            }
        }
        None if params.has_gravity_model => {
            let msg = "gravity model is configured without derated torque limits; payload capacity cannot be bounded".to_string();
            if params.fail_on_insufficient_evidence {
                return Err(Rejection::new("lift", "INSUFFICIENT_EVIDENCE", msg));
            }
            insufficient = Some(msg);
        }
        None => {}
    }
    Ok(LoadedLegVerdict {
        payload_capacity: capacity,
        insufficient_evidence: insufficient,
    })
}

/// Largest per-arm-joint absolute difference (rad) between two joint vectors.
pub fn max_joint_diff(a: &[f64; 6], b: &[f64; 6], arm_dof: usize) -> f64 {
    (0..arm_dof.min(6))
        .map(|i| (a[i] - b[i]).abs())
        .fold(0.0, f64::max)
}

/// Plan a swept joint-space trajectory from `input.q0` to `q_goal` as a verbatim [`JointPath`]
/// with J7 strictly locked to `input.j7` at every sample.
///
/// Because [`motion::PathTrajectory`] evaluates `q(s)` along the quintic time scaling
/// `s(u) = 10u^3 - 15u^4 + 6u^5`, linear-in-`s` joint knots execute the exact zero-velocity
/// boundary multi-joint quintic trajectory verbatim with zero owner-side reconstruction.
/// Sweeps workspace, keep-out boxes, self-collision capsules, joint limits, power budget, and
/// grasp table clearance off-lock.
pub fn plan_swept_joint_path(
    input: &PlanInput,
    q_goal: [f64; 6],
    envelope: &[Vec3],
    table_bound_z: f64,
) -> Result<JointPath, MotionError> {
    let arm_dof = input.arm.arm_dof();
    if input.limits.len() < arm_dof + 1 {
        return Err(MotionError::Unsupported(format!(
            "Swept joint path needs at least {} joints",
            arm_dof + 1
        )));
    }
    let arm_scope = crate::model::PlanningScope::first_n(arm_dof);
    // Check configured planning bounds on start and BOTH effective planning bounds on goal
    // BEFORE the search-margin guard, so planning-cap violations surface IK_UNREACHABLE naming
    // the joint and bound while unconfigured sides preserve the original IK_LIMIT_MARGIN guard.
    motion::check_start_planning_limits_f64(&input.q0, &input.limits, arm_scope, Some(0.0))?;
    motion::check_planning_limits_f64(&q_goal, &input.limits, arm_scope, Some(1.0))?;
    // Preserve the original IK_LIMIT_MARGIN guard on every omitted (non-custom) side
    // independently of `ik_search_bounds`'s narrow-interval fallback.
    for i in 0..arm_dof {
        let l = &input.limits[i];
        let (pmin, pmax) = l.effective_planning_range().expect("checked above");
        let lo_i = if l.planning_min_position.is_some() {
            f64::from(pmin)
        } else {
            f64::from(l.min_position) + motion::IK_LIMIT_MARGIN
        };
        let hi_i = if l.planning_max_position.is_some() {
            f64::from(pmax)
        } else {
            f64::from(l.max_position) - motion::IK_LIMIT_MARGIN
        };
        if !q_goal[i].is_finite() || q_goal[i] < lo_i - 1e-6 || q_goal[i] > hi_i + 1e-6 {
            return Err(MotionError::InvalidCommand(format!(
                "swept joint goal J{} ({:.4} rad) outside safe limits [{:.4}, {:.4}]",
                i + 1,
                q_goal[i],
                lo_i,
                hi_i
            )));
        }
    }
    let max_dq = max_joint_diff(&input.q0, &q_goal, arm_dof);
    let n = ((max_dq / SWEPT_JOINT_STEP_RAD).ceil() as usize).clamp(2, motion::MAX_SAMPLES);
    let start_q32: [f32; 7] = std::array::from_fn(|i| {
        if i < arm_dof {
            input.q0[i] as f32
        } else if i == arm_dof {
            input.j7
        } else {
            0.0
        }
    });
    motion::check_start_planning_limits_f32(&start_q32, &input.limits, arm_scope, Some(0.0))?;
    let ws_eval = input.ws.as_ref().map(|ws| {
        crate::chain::WorkspaceEvaluator::new_with_tcp(&input.arm, ws, &input.cam, input.tcp_offset)
    });
    let mut ws_state = match &ws_eval {
        Some(ev) => {
            let mut c0 = ev.alloc_buf();
            ev.eval_into(&start_q32, &mut c0, "plan").map_err(|f| {
                MotionError::WorkspaceViolation {
                    s: 0.0,
                    point: f.point,
                    limit: f.limit,
                    value: f.value,
                    bound: f.bound,
                    detail: f.detail,
                }
            })?;
            let min_d = ev.init_min_depths(&c0);
            let start_outside = crate::chain::WorkspaceEvaluator::any_outside(min_d.as_slice());
            Some((c0, min_d, motion::MAX_CARTESIAN_SWEEP_EVALS, start_outside))
        }
        None => None,
    };

    let mut q_samples = Vec::with_capacity(n + 1);
    let mut s_samples = Vec::with_capacity(n + 1);
    q_samples.push(start_q32);
    s_samples.push(0.0);
    let mut prev_q32 = start_q32;
    for k in 1..=n {
        let s = k as f64 / n as f64;
        let q32: [f32; 7] = std::array::from_fn(|i| {
            if i < arm_dof {
                let (a, b) = (input.q0[i], q_goal[i]);
                let v = (a + (b - a) * s).clamp(a.min(b), a.max(b)) as f32;
                let (a32, b32) = (a as f32, b as f32);
                v.clamp(a32.min(b32), a32.max(b32))
            } else if i == arm_dof {
                input.j7
            } else {
                0.0
            }
        });
        motion::check_planning_limits_f32(&q32, &input.limits, arm_scope, Some(s))?;
        if let (Some(ev), Some((c_prev, min_d, rem, start_outside))) = (&ws_eval, &mut ws_state) {
            match ev.sweep_linear_step_with_start_outside(
                &prev_q32,
                &q32,
                c_prev,
                min_d,
                rem,
                motion::MAX_CARTESIAN_SWEEP_EVALS,
                "plan",
                "sweep",
                *start_outside,
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
        prev_q32 = q32;
    }

    crate::backlash::check_table_clear(
        &input.arm,
        envelope,
        table_bound_z,
        q_samples.iter().map(|q| q.as_slice()),
    )
    .map_err(|v| v.into_motion_error())?;

    let mut duration: f64 = 0.0;
    for j in 0..arm_dof {
        let dq = (q_goal[j] - input.q0[j]).abs();
        let vcap = f64::from(input.limits[j].max_velocity);
        let acap = f64::from(input.limits[j].max_acceleration).min(motion::ACTION_MAX_ACCELERATION);
        if vcap > 0.0 {
            duration = duration.max(1.875 * dq / vcap);
        }
        if acap > 0.0 {
            duration = duration.max((5.7735 * dq / acap).sqrt());
        }
    }
    let duration = (duration * 1.1).max(0.2);
    let dof = input.limits.len();
    let mut path = JointPath {
        dof,
        arm_dof,
        q: q_samples,
        s: s_samples,
        duration,
        power_scale: 1.0,
    };
    if let Some(b) = input.power.as_ref().filter(|b| b.enabled && b.planning.enabled) {
        let _ = path.fit_power(b, &input.arm)?;
    }
    Ok(path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::ArmConfig;

    fn input() -> PlanInput {
        let c = ArmConfig::rebot_dm();
        PlanInput {
            q0: [0.0, -1.167, -1.565, 1.435, 0.0, 0.0], j7: 0.0, limits: c.limits.clone(), speed_cap: None, ws: c.workspace.clone(),
            cam: c.camera_mount.clone(), arm: c.arm_model.clone(), epoch: 0, workspace_revision: 0, power: None, static_restricted: false, tcp_offset: c.tcp_offset,
            goal_frame: GoalFrame::Tcp,
        }
    }

    #[test]
    fn endpoint_ik_reaches_fk_of_a_known_pose() {
        let i = input();
        let q = [0.3, -0.9, -1.2, 1.0, 0.2, 0.1];
        let goal = motion::end_to_tcp(&i.arm.fk(&q), &i.tcp_offset);
        let mut w = Work::default();
        let sol = endpoint_ik(&i, &goal, &[i.q0], &mut w).expect("reachable");
        let got = motion::end_to_tcp(&i.arm.fk(&sol), &i.tcp_offset);
        assert!(kinematics::norm(&kinematics::sub(&got.p, &goal.p)) <= PAR.tol_pos);
        assert!(w.ik_iterations > 0);
        assert!(endpoint_ik(&i, &Pose::new(goal.r, [2.0, 0.0, 0.0]), &[i.q0], &mut w).is_err());
    }

    #[test]
    fn margins_and_payload_torque_signs() {
        let i = input();
        let q = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0];
        let m = joint_margin(&i, &q);
        assert!(m > 0.0 && m <= 1.0);
        let s = sigma_min(&i, &q);
        assert!(s > 1e-3, "{s}");
        // a payload in front of the base loads J2 the same way the arm's own weight does
        let q32: [f32; 7] = std::array::from_fn(|k| if k < 6 { q[k] as f32 } else { 0.0 });
        let arm = i.arm.gravity_torques(&q32).unwrap();
        let tcp = motion::end_to_tcp(&i.arm.fk(&q), &i.tcp_offset);
        let per = payload_torque_per_kg(&i, &q, &tcp.p).unwrap();
        assert!(f64::from(arm[1]) * per[1] > 0.0, "arm {} payload {}", arm[1], per[1]);
    }

    #[test]
    fn obb_sat_catches_thin_box_piercing_face_with_all_corners_outside() {
        // Object cube [-0.02, 0.02]^3 centered at (0.30, 0.0, 0.05), rotated 45 deg about z.
        let r = kinematics::axis_r(&[0.0, 0.0, 1.0], 45f64.to_radians());
        let center = [0.30, 0.0, 0.05];
        let half = [0.02, 0.02, 0.02];
        // A thin vertical rod piercing straight through the center: ALL 8 object corners lie
        // far outside this 2 mm x 2 mm rod, yet the rod intersects the object interior & faces!
        let rod_min = [0.299, -0.001, 0.0];
        let rod_max = [0.301, 0.001, 0.10];
        assert!(obb_intersects_aabb(&center, &r, &half, &rod_min, &rod_max));
        // Shifting the rod past the rotated corner radius (0.02 * sqrt(2) ~ 0.0283 m) clears it.
        let clear_min = [0.335, -0.001, 0.0];
        let clear_max = [0.337, 0.001, 0.10];
        assert!(!obb_intersects_aabb(&center, &r, &half, &clear_min, &clear_max));
    }

    #[test]
    fn swept_joint_path_locks_j7_and_respects_keepout() {
        let mut i = input();
        i.j7 = -2.34;
        let q_goal = [0.15, -1.05, -1.45, 1.35, 0.1, 0.05];
        let env = vec![[0.0, 0.0, 0.0]];
        let path = plan_swept_joint_path(&i, q_goal, &env, -0.5).expect("clear swept joint path");
        assert!(path.q.len() >= 2);
        assert!(path.q.iter().all(|q| (q[6] - i.j7).abs() == 0.0), "J7 must stay locked");
        // Put a keep-out box at the mid-path TCP position -> swept joint path must reject!
        let mid_q = q6(&path.q[path.q.len() / 2]);
        let mid_tcp = motion::end_to_tcp(&i.arm.fk(&mid_q), &i.tcp_offset).p;
        i.ws.as_mut().unwrap().keepout_boxes.push(crate::model::KeepoutBox {
            name: "mid_block".into(),
            min_m: mid_tcp.map(|v| (v - 0.01) as f32),
            max_m: mid_tcp.map(|v| (v + 0.01) as f32),
        });
        let err = plan_swept_joint_path(&i, q_goal, &env, -0.5).unwrap_err();
        assert_eq!(err.code(), "WORKSPACE_VIOLATION");
    }

    #[test]
    fn carried_object_box_includes_floor_enforces_below_floor_and_se3_rotation() {
        let mut i = input();
        let ws = i.ws.as_mut().unwrap();
        ws.floor_z_m = 0.05;
        let q = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0];
        let q32: [f32; 7] = std::array::from_fn(|k| if k < 6 { q[k] as f32 } else { 0.0 });
        let path = JointPath {
            dof: 7,
            arm_dof: 6,
            q: vec![q32, q32],
            s: vec![0.0, 1.0],
            duration: 0.3,
            power_scale: 1.0,
        };
        let tcp = motion::end_to_tcp(&i.arm.fk(&q), &i.tcp_offset);
        // Box sitting at z = 0.00 (50 mm below floor_z_m = 0.05) with table_z_m = 0.00 MUST FAIL!
        let base_box = CarriedObjectBox::from_tcp_aabb([-0.02, -0.02, -0.02], [0.02, 0.02, 0.02]);
        // Shift box in TCP frame so its world z is ~0.0 while tcp.p[2] ~ 0.15
        let rt = kinematics::transpose(&tcp.r);
        let shift_tcp = kinematics::mat_vec(&rt, &[0.0, 0.0, 0.0 - tcp.p[2]]);
        let low_box = CarriedObjectBox {
            center_tcp: shift_tcp,
            ..base_box
        };
        let err = check_carried_object_volume(&i.arm, &i.tcp_offset, ws, 0.0, &low_box, &path, "separation")
            .unwrap_err();
        assert_eq!(err.code, "WORKSPACE_VIOLATION");
        assert_eq!(err.geometry.as_deref(), Some("carried_object:floor"));

        // SE(3) relative rotation must rotate exact OBB corners (45 deg rotation about z expands XY)
        let rot45 = Pose::new(kinematics::axis_r(&[0.0, 0.0, 1.0], 45f64.to_radians()), [0.0; 3]);
        let rotated = base_box.transformed_by_relative_pose(&rot45);
        let max_x = rotated.corners_tcp().iter().map(|c| c[0]).fold(f64::NEG_INFINITY, f64::max);
        let max_y = rotated.corners_tcp().iter().map(|c| c[1]).fold(f64::NEG_INFINITY, f64::max);
        assert!(max_x > 0.027 && max_y > 0.027, "max_x={max_x} max_y={max_y}");
    }

    #[test]
    fn validate_loaded_leg_enforces_torque_and_phase_timeout() {
        let i = input();
        let q = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0];
        let q32: [f32; 7] = std::array::from_fn(|k| if k < 6 { q[k] as f32 } else { 0.0 });
        let path = JointPath {
            dof: 7,
            arm_dof: 6,
            q: vec![q32, q32],
            s: vec![0.0, 1.0],
            duration: 1.0,
            power_scale: 1.0,
        };
        let obj = CarriedObjectBox::from_tcp_aabb([-0.02, -0.02, -0.02], [0.02, 0.02, 0.02]);
        let env = vec![[0.0, 0.0, 0.0]];
        // Phase timeout violation (1.0 + 0.5 > 1.2)
        let params_to = LoadedLegParams {
            arm: &i.arm,
            tcp_offset: &i.tcp_offset,
            ws: i.ws.as_ref(),
            envelope: &env,
            table_z_m: -0.5,
            phase_timeout_s: 1.2,
            obj_box: &obj,
            payload_kg: Some(0.1),
            torque_limits: Some(&[10.0; 6]),
            has_gravity_model: true,
            fail_on_insufficient_evidence: true,
        };
        let err = validate_loaded_leg(&params_to, &path, "separation").unwrap_err();
        assert_eq!(err.code, "GRASP_TIMEOUT");

        // Tight torque limit violation with 2.0 kg payload -> LIFT_TORQUE
        let params_tq = LoadedLegParams {
            phase_timeout_s: 10.0,
            payload_kg: Some(2.0),
            torque_limits: Some(&[3.0; 6]),
            ..params_to
        };
        let err = validate_loaded_leg(&params_tq, &path, "lift").unwrap_err();
        assert_eq!(err.code, "LIFT_TORQUE");
    }
}
