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
use crate::motion::{self, GoalFrame, JointPath, MotionError, PathOpts, PlanInput, Speed, IK_LIMIT_MARGIN};

/// Rise from the contact before the loaded lift, straight up (m).
pub const SEPARATION_M: f64 = 0.01;
/// DLS rounds per seed of the endpoint screen ([`PAR`] iterations each).
pub const ENDPOINT_ROUNDS: usize = 8;
/// Characteristic length scaling the angular rows of the TCP Jacobian (m).
pub const JACOBIAN_CHAR_LENGTH_M: f64 = 0.1;
/// Payload bound assumed when the request gives none (kg).
pub const ASSUMED_PAYLOAD_KG: f64 = 0.15;

/// Work counters of one selection.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Work {
    pub ik_iterations: u64,
    pub endpoint_solves: usize,
    pub path_checks: usize,
    pub path_samples: usize,
}

fn limits(input: &PlanInput) -> ([f64; 6], [f64; 6]) {
    let n = input.arm.arm_dof();
    let lim = |i: usize| input.limits.get(i).filter(|_| i < n);
    (std::array::from_fn(|i| lim(i).map_or(-std::f64::consts::PI, |l| f64::from(l.min_position) + IK_LIMIT_MARGIN)),
     std::array::from_fn(|i| lim(i).map_or(std::f64::consts::PI, |l| f64::from(l.max_position) - IK_LIMIT_MARGIN)))
}

/// IK of the TCP pose `goal` from each of `seeds` (first success wins): Err(position error,
/// rotation error) of the best attempt.
pub fn endpoint_ik(input: &PlanInput, goal: &Pose, seeds: &[[f64; 6]], work: &mut Work) -> Result<[f64; 6], (f64, f64)> {
    let (lo, hi) = limits(input);
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
            if sol.ok { return Ok(q); }
            let obj = sol.ep + PAR.w * sol.er;
            if obj > prev - 1e-9 { break; }
            prev = obj;
        }
    }
    Err(best)
}

/// A path phase's rejection from a planner error.
pub fn motion_rejection(phase: &'static str, e: &MotionError) -> Rejection {
    let mut r = Rejection::new(phase, e.code(), e.to_string());
    match e {
        MotionError::IkUnreachable { s, ep, .. } => { r.value = Some(*ep); r.bound = Some(PAR.tol_pos); r.s = Some(*s); }
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

/// Smallest normalized joint-limit margin of `q`.
pub fn joint_margin(input: &PlanInput, q: &[f64; 6]) -> f64 {
    (0..input.arm.arm_dof()).map(|i| {
        let l = &input.limits[i];
        let (lo, hi) = (f64::from(l.min_position), f64::from(l.max_position));
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
fn payload_torque_per_kg(input: &PlanInput, q: &[f64; 6], p: &Vec3) -> Option<Vec<f64>> {
    let q32: [f32; 6] = std::array::from_fn(|i| q[i] as f32);
    let f = input.arm.frames_f32(&q32)?;
    // the motor holds the weight: τ = Jᵥᵀ (m g ẑ)
    Some((0..f.n).map(|j| kinematics::cross(&f.axes[j], &kinematics::sub(p, &f.origins[j]))[2] * crate::chain::G).collect())
}

/// Largest payload the torque limits allow over the loaded samples `qs`, with the payload's
/// centre of mass at `com_tcp` (TCP frame). Negative when the arm alone exceeds a limit.
pub fn payload_capacity(s: &Snapshot, qs: &[[f64; 6]], com_tcp: &Vec3) -> Option<(f64, usize)> {
    let (lim, _) = s.torque_limits.as_ref()?;
    let mut cap = f64::INFINITY;
    let mut joint = 0;
    for q in qs {
        let q32: [f32; 7] = std::array::from_fn(|i| if i < 6 { q[i] as f32 } else { 0.0 });
        let arm = s.input.arm.gravity_torques(&q32).ok()?;
        let tcp = motion::end_to_tcp(&s.input.arm.fk(q), &s.input.tcp_offset);
        let p = tcp.transform_point(com_tcp);
        let per = payload_torque_per_kg(&s.input, q, &p)?;
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::ArmConfig;

    fn input() -> PlanInput {
        let c = ArmConfig::rebot_dm();
        PlanInput {
            q0: [0.0, -1.167, -1.565, 1.435, 0.0, 0.0], j7: 0.0, limits: c.limits.clone(), speed_cap: None, ws: c.workspace.clone(),
            cam: c.camera_mount.clone(), arm: c.arm_model.clone(), epoch: 0, workspace_revision: 0, power: None, tcp_offset: c.tcp_offset,
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
}
