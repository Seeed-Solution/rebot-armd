//! Kinematic chain built from a URDF at startup: forward kinematics, geometric Jacobian,
//! damped-least-squares IK and gravity torques from the URDF inertials.
//!
//! The chain is the path `base_link -> tip_link` of the URDF. Its revolute / continuous
//! joints are the arm joints, in path order; entry `i` of [`ChainSpec::joints`] is armd joint
//! `i`. Fixed joints on the path are folded into the neighbouring transforms. Links that hang
//! off the path (gripper fingers, sensors) count for gravity at joint position 0.
//!
//! Joint mapping: URDF angle = `sign * armd angle + offset_rad`.
//!
//! The built-in B601-DM model (`kinematics.rs`, `gravity.rs`) stays the default; this module
//! is used when the profile names a URDF (`kinematics` / `gravity_model` = `Urdf`).
//! `tests/test_chain.rs` checks that a chain built from the DM URDF reproduces the built-in
//! model.
use std::sync::Arc;

use serde::{Deserialize, Serialize};

use crate::kinematics::{self as k, IkResult, Mat3, Pose, Vec3, IDENTITY, PAR};
use crate::urdf::{JointType, Urdf};

/// Most arm joints a chain may have.
pub const MAX_CHAIN_JOINTS: usize = 8;

/// Gravitational acceleration (m/s², along -z of the base frame), as `gravity.rs`.
pub const G: f64 = 9.81;

/// Profile section `chain`: which part of the URDF is the arm and how its joints map to armd.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ChainSpec {
    pub base_link: String,
    pub tip_link: String,
    /// One entry per arm joint, armd joint order (= URDF path order).
    pub joints: Vec<ChainJointSpec>,
}

fn one() -> f64 { 1.0 }

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ChainJointSpec {
    /// URDF joint name.
    pub urdf_joint: String,
    /// +1 or -1.
    #[serde(default = "one")]
    pub sign: f64,
    /// URDF angle at armd angle 0 (rad).
    #[serde(default)]
    pub offset_rad: f64,
}

/// A rigid transform (rotation, translation).
type Tf = (Mat3, Vec3);

fn tf_mul(a: &Tf, b: &Tf) -> Tf { (k::mat_mul(&a.0, &b.0), k::add(&a.1, &k::mat_vec(&a.0, &b.1))) }

#[derive(Debug, Clone, PartialEq)]
pub struct KinematicChain {
    /// URDF robot name.
    pub robot: String,
    pub base_link: String,
    pub tip_link: String,
    pub joint_names: Vec<String>,
    /// Transform from the previous joint's (rotated) frame, or the base, to joint `j`'s origin.
    pre_r: Vec<Mat3>,
    pre_p: Vec<Vec3>,
    /// Unit joint axis in the joint frame.
    axis: Vec<Vec3>,
    sign: Vec<f64>,
    offset: Vec<f64>,
    /// Last joint's frame -> tip_link.
    tip: Tf,
    /// Lumped mass (kg) and centre of mass of each segment; segment 0 is fixed to the base,
    /// segment `j + 1` moves with joint `j` (expressed in that joint's rotated frame).
    seg_mass: Vec<f64>,
    seg_com: Vec<Vec3>,
    /// URDF `<limit>` of each arm joint (URDF angles), when given.
    pub urdf_limits: Vec<Option<(f64, f64)>>,
}

/// World frames of a chain for one joint vector.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChainFrames {
    pub n: usize,
    /// T_base_tip.
    pub end: Pose,
    /// d(pose)/d(armd angle) direction of each joint: world axis times the joint sign.
    pub axes: [Vec3; MAX_CHAIN_JOINTS],
    pub origins: [Vec3; MAX_CHAIN_JOINTS],
    /// Rotation of joint `j`'s frame after its rotation (segment `j + 1`).
    pub seg_r: [Mat3; MAX_CHAIN_JOINTS],
}

/// IK result for an `n`-joint chain (armd angles in `q[..n]`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChainIk {
    pub ok: bool,
    pub q: [f64; MAX_CHAIN_JOINTS],
    pub ep: f64,
    pub er: f64,
    pub iterations: usize,
}

impl KinematicChain {
    /// Build the chain `spec` from a parsed URDF.
    pub fn from_urdf(urdf: &Urdf, spec: &ChainSpec) -> Result<Self, String> {
        if urdf.link(&spec.base_link).is_none() { return Err(format!("chain base_link '{}' is not in the URDF", spec.base_link)); }
        if urdf.link(&spec.tip_link).is_none() { return Err(format!("chain tip_link '{}' is not in the URDF", spec.tip_link)); }
        // Path tip -> base.
        let mut path = Vec::new();
        let mut cur = spec.tip_link.as_str();
        while cur != spec.base_link {
            let j = urdf.parent_joint(cur)
                .ok_or_else(|| format!("tip_link '{}' is not below base_link '{}'", spec.tip_link, spec.base_link))?;
            if path.len() > urdf.joints.len() { return Err("URDF joint graph has a cycle".into()); }
            path.push(j);
            cur = &j.parent;
        }
        path.reverse();
        let movable: Vec<_> = path.iter().filter(|j| j.kind != JointType::Fixed).collect();
        if let Some(j) = movable.iter().find(|j| !matches!(j.kind, JointType::Revolute | JointType::Continuous)) {
            return Err(format!("joint '{}' on the chain is {:?}; only revolute, continuous and fixed joints are supported", j.name, j.kind));
        }
        let names: Vec<&str> = movable.iter().map(|j| j.name.as_str()).collect();
        let listed: Vec<&str> = spec.joints.iter().map(|j| j.urdf_joint.as_str()).collect();
        if names != listed {
            return Err(format!("chain joints {listed:?} must be the movable joints from base_link to tip_link in order: {names:?}"));
        }
        if names.is_empty() || names.len() > MAX_CHAIN_JOINTS {
            return Err(format!("a chain needs 1..={MAX_CHAIN_JOINTS} joints, has {}", names.len()));
        }
        for j in &spec.joints {
            if j.sign != 1.0 && j.sign != -1.0 { return Err(format!("joint '{}': sign must be 1 or -1", j.urdf_joint)); }
            if !j.offset_rad.is_finite() { return Err(format!("joint '{}': offset_rad must be finite", j.urdf_joint)); }
        }

        let origin_tf = |j: &crate::urdf::Joint| -> Tf { (k::rpy_to_r(j.rpy[0], j.rpy[1], j.rpy[2]), j.xyz) };
        let (mut pre_r, mut pre_p, mut axis) = (Vec::new(), Vec::new(), Vec::new());
        let mut acc: Tf = (IDENTITY, [0.0; 3]);
        for j in &path {
            let t = tf_mul(&acc, &origin_tf(j));
            if j.kind == JointType::Fixed {
                acc = t;
            } else {
                let n = k::norm(&j.axis);
                if n.is_nan() || n <= 1e-9 { return Err(format!("joint '{}': zero axis", j.name)); }
                pre_r.push(t.0);
                pre_p.push(t.1);
                axis.push([j.axis[0] / n, j.axis[1] / n, j.axis[2] / n]);
                acc = (IDENTITY, [0.0; 3]);
            }
        }
        let tip = acc;

        // Mass of every link below base_link, lumped per segment.
        let n = names.len();
        let mut m_sum = vec![0.0f64; n + 1];
        let mut mc_sum = vec![[0.0f64; 3]; n + 1];
        let mut stack: Vec<(&str, usize, Tf)> = vec![(spec.base_link.as_str(), 0, (IDENTITY, [0.0; 3]))];
        let mut visited = 0usize;
        while let Some((link, seg, t)) = stack.pop() {
            visited += 1;
            if visited > urdf.links.len() { return Err("URDF link graph has a cycle".into()); }
            if let Some(i) = urdf.link(link).and_then(|l| l.inertial.as_ref()) {
                let c = k::add(&t.1, &k::mat_vec(&t.0, &i.com));
                m_sum[seg] += i.mass;
                for a in 0..3 { mc_sum[seg][a] += i.mass * c[a]; }
            }
            for jt in urdf.child_joints(link) {
                match names.iter().position(|nm| *nm == jt.name) {
                    Some(idx) => stack.push((&jt.child, idx + 1, (IDENTITY, [0.0; 3]))),
                    None => stack.push((&jt.child, seg, tf_mul(&t, &origin_tf(jt)))),
                }
            }
        }
        let seg_com = m_sum.iter().zip(&mc_sum)
            .map(|(m, mc)| if *m > 0.0 { [mc[0] / m, mc[1] / m, mc[2] / m] } else { [0.0; 3] })
            .collect();

        Ok(Self {
            robot: urdf.name.clone(),
            base_link: spec.base_link.clone(),
            tip_link: spec.tip_link.clone(),
            joint_names: names.iter().map(|s| s.to_string()).collect(),
            pre_r, pre_p, axis,
            sign: spec.joints.iter().map(|j| j.sign).collect(),
            offset: spec.joints.iter().map(|j| j.offset_rad).collect(),
            tip,
            seg_mass: m_sum,
            seg_com,
            urdf_limits: movable.iter().map(|j| j.limit).collect(),
        })
    }

    /// Parse `text` and build the chain.
    pub fn from_urdf_str(text: &str, spec: &ChainSpec) -> Result<Self, String> {
        Self::from_urdf(&crate::urdf::parse(text)?, spec)
    }

    pub fn dof(&self) -> usize { self.axis.len() }

    /// Total mass that moves with joint 0 (kg); 0 means the URDF has no usable inertials.
    pub fn moving_mass(&self) -> f64 { self.seg_mass[1..].iter().sum() }

    /// World frames for armd angles `q` (at least [`Self::dof`] entries).
    pub fn frames(&self, q: &[f64]) -> ChainFrames {
        let n = self.dof();
        let mut r = IDENTITY;
        let mut p = [0.0; 3];
        let mut axes = [[0.0; 3]; MAX_CHAIN_JOINTS];
        let mut origins = [[0.0; 3]; MAX_CHAIN_JOINTS];
        let mut seg_r = [IDENTITY; MAX_CHAIN_JOINTS];
        for j in 0..n {
            p = k::add(&p, &k::mat_vec(&r, &self.pre_p[j]));
            r = k::mat_mul(&r, &self.pre_r[j]);
            let a = k::mat_vec(&r, &self.axis[j]);
            let s = self.sign[j];
            axes[j] = [a[0] * s, a[1] * s, a[2] * s];
            origins[j] = p;
            r = k::mat_mul(&r, &k::axis_r(&self.axis[j], s * q[j] + self.offset[j]));
            seg_r[j] = r;
        }
        p = k::add(&p, &k::mat_vec(&r, &self.tip.1));
        r = k::mat_mul(&r, &self.tip.0);
        ChainFrames { n, end: Pose { r, p }, axes, origins, seg_r }
    }

    pub fn fk(&self, q: &[f64]) -> Pose { self.frames(q).end }

    /// Geometric Jacobian at the tip: column `j` = [a_j × (p - o_j); a_j] (per armd radian).
    pub fn jacobian(&self, q: &[f64]) -> Vec<[f64; 6]> {
        let f = self.frames(q);
        (0..f.n).map(|j| {
            let v = k::cross(&f.axes[j], &k::sub(&f.end.p, &f.origins[j]));
            let a = f.axes[j];
            [v[0], v[1], v[2], a[0], a[1], a[2]]
        }).collect()
    }

    /// Damped-least-squares IK, the algorithm of [`k::solve_dls_with`] for `n` joints:
    /// dq = Jᵀ (J Jᵀ + λ² I)⁻¹ e with the 6x6 system solved by Cholesky. `q0`, `lo`, `hi`
    /// need at least `n` entries.
    pub fn solve_dls(&self, goal: &Pose, q0: &[f64], lo: &[f64], hi: &[f64], w: f64, iters: usize) -> ChainIk {
        let n = self.dof();
        let mut lo_w = [0.0; MAX_CHAIN_JOINTS];
        let mut hi_w = [0.0; MAX_CHAIN_JOINTS];
        let mut q = [0.0; MAX_CHAIN_JOINTS];
        for i in 0..n {
            lo_w[i] = lo[i].min(q0[i]);
            hi_w[i] = hi[i].max(q0[i]);
            q[i] = q0[i];
        }
        let q_start = q;
        let (rg, pg) = (&goal.r, &goal.p);
        let mut f = self.frames(&q);
        let mut e = k::pose_err_end(&f.end, rg, pg, w);
        let mut lam = PAR.lam_min;
        let mut it = 0;
        if !(e.ep < f64::INFINITY && e.er < f64::INFINITY) {
            return ChainIk { ok: false, q: q_start, ep: f64::INFINITY, er: f64::INFINITY, iterations: it };
        }
        while it < iters && !(e.ep <= PAR.tol_pos / 10.0 && e.er <= PAR.tol_rot / 10.0) {
            it += 1;
            let mut jac = [[0.0f64; 6]; MAX_CHAIN_JOINTS];
            for j in 0..n {
                let a = f.axes[j];
                let v = k::cross(&a, &k::sub(&f.end.p, &f.origins[j]));
                jac[j] = [v[0], v[1], v[2], w * a[0], w * a[1], w * a[2]];
            }
            let mut m = [0.0f64; 36];
            for r in 0..6 {
                for c in r..6 {
                    let mut s = 0.0;
                    for col in &jac[..n] { s += col[r] * col[c]; }
                    m[r * 6 + c] = s;
                    m[c * 6 + r] = s;
                }
            }
            for r in 0..6 { m[r * 6 + r] += lam * lam; }
            let Some(y) = k::chol6(&m, &e.e) else { break; };
            let mut dq = [0.0f64; MAX_CHAIN_JOINTS];
            for (j, col) in jac[..n].iter().enumerate() {
                let mut s = 0.0;
                for r in 0..6 { s += col[r] * y[r]; }
                dq[j] = s;
            }
            let big = dq[..n].iter().fold(0.0f64, |m, v| m.max(v.abs()));
            let kk = if big > PAR.max_step { PAR.max_step / big } else { 1.0 };
            let mut qn = [0.0; MAX_CHAIN_JOINTS];
            for i in 0..n { qn[i] = hi_w[i].min(lo_w[i].max(q[i] + kk * dq[i])); }
            let fn_ = self.frames(&qn);
            let en = k::pose_err_end(&fn_.end, rg, pg, w);
            if en.ep + w * en.er < e.ep + w * e.er {
                q = qn; f = fn_; e = en; lam = PAR.lam_min;
            } else if lam < PAR.lam_max {
                lam = PAR.lam_max;
            } else {
                q = qn; f = fn_; e = en;
            }
        }
        ChainIk { ok: e.ep <= PAR.tol_pos && e.er <= PAR.tol_rot, q, ep: e.ep, er: e.er, iterations: it }
    }

    /// Holding torques (N·m, armd joint direction) against gravity at armd angles `q`.
    pub fn gravity_torques(&self, q: &[f64]) -> [f64; MAX_CHAIN_JOINTS] {
        let f = self.frames(q);
        let n = f.n;
        let mut wc = [[0.0f64; 3]; MAX_CHAIN_JOINTS];
        for s in 0..n {
            wc[s] = k::add(&f.origins[s], &k::mat_vec(&f.seg_r[s], &self.seg_com[s + 1]));
        }
        let mut tau = [0.0f64; MAX_CHAIN_JOINTS];
        for kj in 0..n {
            let a = &f.axes[kj];
            let o = &f.origins[kj];
            let mut t = 0.0;
            for s in kj..n {
                let mg = self.seg_mass[s + 1] * G;
                if mg == 0.0 { continue; }
                let r = k::sub(&wc[s], o);
                // holding torque = -(r × m g ẑ(-1)) = [r_y m g, -r_x m g, 0]
                t += a[0] * (r[1] * mg) - a[1] * (r[0] * mg);
            }
            tau[kj] = t;
        }
        tau
    }

    /// (radius, height) of the second joint's axis in the arm plane at q = 0, the origin of
    /// the workspace `max_reach_m` check (see `kinematics::shoulder_reach`). The radius is
    /// measured along the horizontal direction perpendicular to that axis, sign taken so it
    /// points to +x.
    pub fn shoulder_rz(&self) -> [f64; 2] {
        let f = self.frames(&[0.0; MAX_CHAIN_JOINTS]);
        let j = usize::min(1, f.n - 1);
        let (o, a) = (f.origins[j], f.axes[j]);
        let ah = (a[0] * a[0] + a[1] * a[1]).sqrt();
        let radius = if ah < 1e-9 {
            (o[0] * o[0] + o[1] * o[1]).sqrt()
        } else {
            let mut e = [-a[1] / ah, a[0] / ah];
            if e[0] < 0.0 || (e[0] == 0.0 && e[1] < 0.0) { e = [-e[0], -e[1]]; }
            o[0] * e[0] + o[1] * e[1]
        };
        [radius, o[2]]
    }
}

/// The arm model the profile selects: the built-in B601-DM code or a URDF chain.
#[derive(Debug, Clone, Default)]
pub enum ArmModel {
    #[default]
    RebotDevArm,
    Chain(Arc<KinematicChain>),
}

impl ArmModel {
    /// Number of arm joints (armd joints `0..arm_dof`).
    pub fn arm_dof(&self) -> usize {
        match self { Self::RebotDevArm => 6, Self::Chain(c) => c.dof() }
    }

    pub fn chain(&self) -> Option<&KinematicChain> {
        match self { Self::RebotDevArm => None, Self::Chain(c) => Some(c) }
    }

    /// Tip pose for armd angles `q` (at least `arm_dof` entries).
    pub fn fk(&self, q: &[f64]) -> Pose {
        match self {
            Self::RebotDevArm => k::fk(&std::array::from_fn(|i| q[i])),
            Self::Chain(c) => c.fk(q),
        }
    }

    /// FK of an f32 joint vector; None when it is shorter than `arm_dof` or not finite.
    pub fn fk_f32(&self, q: &[f32]) -> Option<Pose> {
        let n = self.arm_dof();
        if q.len() < n || !q[..n].iter().all(|v| v.is_finite()) { return None; }
        let mut q64 = [0.0; MAX_CHAIN_JOINTS];
        for (o, v) in q64.iter_mut().zip(&q[..n]) { *o = f64::from(*v); }
        Some(self.fk(&q64))
    }

    /// 6-joint DLS IK (the Cartesian actions). A chain with another joint count returns
    /// `ok: false`.
    pub fn solve_dls(&self, goal: &Pose, q0: &[f64; 6], lo: &[f64; 6], hi: &[f64; 6], w: f64) -> IkResult {
        match self {
            Self::RebotDevArm => k::solve_dls(goal, q0, lo, hi, w),
            Self::Chain(c) if c.dof() == 6 => {
                let r = c.solve_dls(goal, q0, lo, hi, w, PAR.iters);
                IkResult { ok: r.ok, q: std::array::from_fn(|i| r.q[i]), ep: r.ep, er: r.er, iterations: r.iterations }
            }
            Self::Chain(_) => IkResult { ok: false, q: *q0, ep: f64::INFINITY, er: f64::INFINITY, iterations: 0 },
        }
    }

    /// Gravity holding torques of the arm joints (entries past `arm_dof` are 0).
    pub fn gravity_torques(&self, q: &[f32]) -> Result<[f32; MAX_CHAIN_JOINTS], crate::gravity::GravityError> {
        let n = self.arm_dof();
        if q.len() < n {
            return Err(crate::gravity::GravityError::NonFiniteAngle(q.len(), f32::NAN));
        }
        if let Some((i, v)) = q[..n].iter().enumerate().find(|(_, v)| !v.is_finite()) {
            return Err(crate::gravity::GravityError::NonFiniteAngle(i, *v));
        }
        let mut out = [0.0f32; MAX_CHAIN_JOINTS];
        match self {
            Self::RebotDevArm => {
                let tau = crate::gravity::compute_gravity_torques(&std::array::from_fn(|i| q[i]))?;
                out[..6].copy_from_slice(&tau);
            }
            Self::Chain(c) => {
                let mut q64 = [0.0; MAX_CHAIN_JOINTS];
                for (o, v) in q64.iter_mut().zip(&q[..n]) { *o = f64::from(*v); }
                let tau = c.gravity_torques(&q64);
                for i in 0..n { out[i] = tau[i] as f32; }
            }
        }
        Ok(out)
    }

    /// For each arm joint, an upper bound on the distance from its axis to any point fixed
    /// to the tip at an offset of at most `max_offset` m: the link lengths from the joint
    /// to the tip plus the offset (configuration independent).
    pub fn joint_reach(&self, max_offset: f64) -> [f64; MAX_CHAIN_JOINTS] {
        let n = self.arm_dof();
        let (origins, end): (Vec<Vec3>, Vec3) = match self {
            Self::RebotDevArm => { let f = k::frames(&[0.0; 6]); (f.origins.to_vec(), f.end.p) }
            Self::Chain(c) => { let f = c.frames(&[0.0; MAX_CHAIN_JOINTS]); (f.origins[..n].to_vec(), f.end.p) }
        };
        let mut reach = [0.0; MAX_CHAIN_JOINTS];
        let mut acc = max_offset + k::norm(&k::sub(&end, &origins[n - 1]));
        for j in (0..n).rev() {
            reach[j] = acc;
            if j > 0 { acc += k::norm(&k::sub(&origins[j], &origins[j - 1])); }
        }
        reach
    }

    pub fn shoulder_rz(&self) -> [f64; 2] {
        match self { Self::RebotDevArm => k::SHOULDER_RZ, Self::Chain(c) => c.shoulder_rz() }
    }

    /// Camera mount R_tip_cam (`kinematics::cam_mount`): for a chain, the wrist axis is the
    /// second-to-last joint seen at last joint = pi/2.
    pub fn cam_mount(&self, flip: bool, rot_deg: f64, tilt_deg: f64) -> Mat3 {
        match self {
            Self::RebotDevArm => k::cam_mount(flip, rot_deg, tilt_deg),
            Self::Chain(c) => {
                let n = c.dof();
                let mut q = [0.0; MAX_CHAIN_JOINTS];
                q[n - 1] = std::f64::consts::PI / 2.0;
                let f = c.frames(&q);
                let a = f.axes[n.saturating_sub(2)];
                k::cam_mount_from(&f.end.r, &a, flip, rot_deg, tilt_deg)
            }
        }
    }

    /// Workspace excess for armd joint positions `q` (`kinematics::workspace_violation`).
    pub fn workspace_violation(&self, q: &[f32], ws: &crate::model::WorkspacePolicy, cam: &crate::model::CameraMount) -> k::Violation {
        match self {
            Self::RebotDevArm => k::workspace_violation(q, ws, cam),
            Self::Chain(_) => match self.fk_f32(q) {
                Some(pose) => k::workspace_violation_pose_at(&pose, ws, cam, self.shoulder_rz()),
                None => k::Violation::invalid(),
            },
        }
    }
}
