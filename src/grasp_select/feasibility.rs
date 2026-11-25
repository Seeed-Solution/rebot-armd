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
pub const VOLUME_MAX_SUBDIV_DEPTH: u8 = 6;
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

// ---- Phase B conservative geometry, carried-object SAT, payload envelope & swept-joint paths ----

/// Conservative TCP-frame bounding box of the carried object (including horizontal
/// `hard_error_m`, vertical `TABLE_ERROR_M`, and optional contact-window offset).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CarriedObjectBox {
    pub min_tcp: Vec3,
    pub max_tcp: Vec3,
}

impl CarriedObjectBox {
    /// Conservative TCP-frame AABB enclosing the target's uncertainty-inflated footprint prism
    /// from `table_z + 0.002` to `table_z + height[1] + TABLE_ERROR_M`.
    pub fn from_target_grasp(t: &Target, grasp_pose: &Pose) -> Self {
        let rt = kinematics::transpose(&grasp_pose.r);
        let z0 = t.table_z + 0.002;
        let z1 = t.table_z + t.height[1] + super::snapshot::TABLE_ERROR_M;
        let e = t.hard_error_m.max(0.0);
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for v in &t.poly {
            for dx in [-e, e] {
                for dy in [-e, e] {
                    for z in [z0, z1] {
                        let p = kinematics::mat_vec(&rt, &kinematics::sub(&[v[0] + dx, v[1] + dy, z], &grasp_pose.p));
                        for i in 0..3 {
                            lo[i] = lo[i].min(p[i]);
                            hi[i] = hi[i].max(p[i]);
                        }
                    }
                }
            }
        }
        Self { min_tcp: lo, max_tcp: hi }
    }

    /// Expands this box to conservatively cover an accepted early-contact offset `offset_tcp`
    /// (object shifted relative to TCP when jaws stop up to `contact_window_m` early along `-a`).
    pub fn expanded_by_tcp_offset(&self, offset_tcp: &Vec3) -> Self {
        Self {
            min_tcp: std::array::from_fn(|i| self.min_tcp[i] + offset_tcp[i].min(0.0)),
            max_tcp: std::array::from_fn(|i| self.max_tcp[i] + offset_tcp[i].max(0.0)),
        }
    }

    /// Expands this box symmetrically by `margin` on every axis.
    pub fn inflated(&self, margin: f64) -> Self {
        let m = margin.max(0.0);
        Self {
            min_tcp: self.min_tcp.map(|v| v - m),
            max_tcp: self.max_tcp.map(|v| v + m),
        }
    }

    pub fn center_and_half(&self) -> (Vec3, Vec3) {
        let c = std::array::from_fn(|i| 0.5 * (self.min_tcp[i] + self.max_tcp[i]));
        let h = std::array::from_fn(|i| (0.5 * (self.max_tcp[i] - self.min_tcp[i])).max(0.0));
        (c, h)
    }

    /// All 8 corners of the box in TCP coordinates (exact extrema for affine payload torque).
    pub fn corners_tcp(&self) -> [Vec3; 8] {
        let (a, b) = (self.min_tcp, self.max_tcp);
        [
            [a[0], a[1], a[2]], [b[0], a[1], a[2]], [a[0], b[1], a[2]], [b[0], b[1], a[2]],
            [a[0], a[1], b[2]], [b[0], a[1], b[2]], [a[0], b[1], b[2]], [b[0], b[1], b[2]],
        ]
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

/// World-space conservative AABB of a TCP-frame OBB `(center_tcp, half_tcp)` at TCP pose `tcp`.
fn obb_world_aabb(tcp: &Pose, center_tcp: &Vec3, half_tcp: &Vec3) -> (Vec3, Vec3, Vec3) {
    let c_w = tcp.transform_point(center_tcp);
    let ext: Vec3 = std::array::from_fn(|i| {
        tcp.r[i][0].abs() * half_tcp[0]
            + tcp.r[i][1].abs() * half_tcp[1]
            + tcp.r[i][2].abs() * half_tcp[2]
    });
    (c_w, kinematics::sub(&c_w, &ext), kinematics::add(&c_w, &ext))
}

/// Conservative swept volume check for the carried object along `path` (covers BOTH `separation`
/// and `lift`), using recursive joint-interval subdivision (`VOLUME_SUBDIV_POS_M` /
/// `VOLUME_SUBDIV_ROT_RAD`), residual margin `VOLUME_RESIDUAL_MARGIN_M`, exact 15-axis
/// OBB-vs-AABB SAT against all `keepout_boxes`, and world-AABB checks against workspace bounds
/// and floor.
pub fn check_carried_object_volume(
    arm: &crate::chain::ArmModel,
    tcp_offset: &Vec3,
    ws: &crate::model::WorkspacePolicy,
    obj_box: &CarriedObjectBox,
    path: &JointPath,
    phase: &'static str,
) -> Result<(), Rejection> {
    if path.q.is_empty() {
        return Ok(());
    }
    let (center_tcp, half_nom) = obj_box.center_and_half();
    let half_inflated = half_nom.map(|h| h + VOLUME_RESIDUAL_MARGIN_M);
    let tcp_of = |q: &[f64; 6]| motion::end_to_tcp(&arm.fk(q), tcp_offset);
    let q0 = q6(&path.q[0]);
    let tcp0 = tcp_of(&q0);
    let (_, wmin0_nom, _) = obb_world_aabb(&tcp0, &center_tcp, &half_nom);
    // Floor bound: never drop below floor_z_m (or below the resting start height when the object
    // starts right on a table at z = floor_z_m, avoiding a false positive from the downward 4 mm
    // residual margin while the object rests on the table at s = 0).
    let floor_bound = (f64::from(ws.floor_z_m)).min(wmin0_nom[2]) - 0.001;
    let ws_min: Vec3 = ws.box_min_m.map(f64::from);
    let ws_max: Vec3 = ws.box_max_m.map(f64::from);

    let check_pose = |tcp: &Pose, s_param: f64| -> Result<(), Rejection> {
        let (c_w, wmin_inf, wmax_inf) = obb_world_aabb(tcp, &center_tcp, &half_inflated);
        let (_, wmin_nom, _) = obb_world_aabb(tcp, &center_tcp, &half_nom);
        for (k, b) in ws.keepout_boxes.iter().enumerate() {
            let bmin = b.min_m.map(f64::from);
            let bmax = b.max_m.map(f64::from);
            if obb_intersects_aabb(&c_w, &tcp.r, &half_inflated, &bmin, &bmax) {
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
            let dp = kinematics::norm(&kinematics::sub(&t_r.p, &t_l.p));
            let dr = kinematics::angle_between(&t_l.r, &t_r.r);
            if dp <= VOLUME_SUBDIV_POS_M && dr <= VOLUME_SUBDIV_ROT_RAD {
                check_pose(&t_r, s_r)?;
                continue;
            }
            if depth >= VOLUME_MAX_SUBDIV_DEPTH {
                // Fail closed if the interval still exceeds the conservative residual margin.
                if dp > VOLUME_RESIDUAL_MARGIN_M * 2.0 || dr > VOLUME_SUBDIV_ROT_RAD * 4.0 {
                    let mut r = Rejection::new(
                        phase,
                        "WORKSPACE_VIOLATION",
                        format!(
                            "carried object sweep failed to converge on segment s={s_l:.2}..{s_r:.2} (dp={dp:.4} m, dr={:.2} deg)",
                            dr.to_degrees()
                        ),
                    );
                    r.geometry = Some("carried_object:subdiv".into());
                    r.s = Some(0.5 * (s_l + s_r));
                    return Err(r);
                }
                check_pose(&t_r, s_r)?;
                continue;
            }
            let q_m: [f64; 6] = std::array::from_fn(|i| 0.5 * (q_l[i] + q_r[i]));
            let t_m = tcp_of(&q_m);
            let s_m = 0.5 * (s_l + s_r);
            check_pose(&t_m, s_m)?;
            stack.push((q_m, t_m, s_m, q_r, t_r, s_r, depth + 1));
            stack.push((q_l, t_l, s_l, q_m, t_m, s_m, depth + 1));
        }
    }
    Ok(())
}

/// Exact global worst-case payload capacity (kg) over the entire CoM uncertainty volume
/// `obj_box` across all loaded joint samples `qs`. Because holding torque `tau = Jv^T (m g z)`
/// is affine in CoM position `p`, the minimum over the 8 corners of `obj_box` equals the exact
/// global minimum over the entire convex box.
pub fn payload_capacity_box(
    s: &Snapshot,
    qs: &[[f64; 6]],
    obj_box: &CarriedObjectBox,
) -> Option<(f64, usize)> {
    let corners = obj_box.corners_tcp();
    let mut worst: Option<(f64, usize)> = None;
    for corner in &corners {
        let (cap, joint) = payload_capacity(s, qs, corner)?;
        if worst.is_none_or(|(w, _)| cap < w) {
            worst = Some((cap, joint));
        }
    }
    worst
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
    let (lo, hi) = limits(input);
    for i in 0..arm_dof {
        if !q_goal[i].is_finite() || q_goal[i] < lo[i] - 1e-6 || q_goal[i] > hi[i] + 1e-6 {
            return Err(MotionError::InvalidCommand(format!(
                "swept joint goal J{} ({:.4} rad) outside safe limits [{:.4}, {:.4}]",
                i + 1,
                q_goal[i],
                lo[i],
                hi[i]
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
            Some((c0, min_d, motion::MAX_CARTESIAN_SWEEP_EVALS))
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
                (input.q0[i] + (q_goal[i] - input.q0[i]) * s) as f32
            } else if i == arm_dof {
                input.j7
            } else {
                0.0
            }
        });
        if let (Some(ev), Some((c_prev, min_d, rem))) = (&ws_eval, &mut ws_state) {
            match ev.sweep_linear_step(
                &prev_q32,
                &q32,
                c_prev,
                min_d,
                rem,
                motion::MAX_CARTESIAN_SWEEP_EVALS,
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
}
