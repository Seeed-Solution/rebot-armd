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
//! The built-in B601-DM model (`kinematics.rs`, `gravity.rs`) stays the default ([`ArmModel`]
//! maps motor to model angles there with the profile's `model_offsets_rad`); this module
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
    #[serde(default)]
    pub tip_rpy_rad: [f64; 3],
    #[serde(default)]
    pub canonical_end_frame: bool,
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

/// m (|c|² E − c cᵀ): inertia of a point mass `m` at `c` about the origin.
fn parallel_axis(m: f64, c: &Vec3) -> Mat3 {
    let c2 = k::dot(c, c);
    std::array::from_fn(|a| std::array::from_fn(|b| m * (if a == b { c2 } else { 0.0 } - c[a] * c[b])))
}

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
    pub(crate) sign: Vec<f64>,
    pub(crate) offset: Vec<f64>,
    /// Last joint's frame -> tip_link.
    tip: Tf,
    /// Fixed rotation from tip_link to canonical end_link.
    pub tip_rot: Mat3,
    pub canonical_end_frame: bool,
    /// Lumped mass (kg) and centre of mass of each segment; segment 0 is fixed to the base,
    /// segment `j + 1` moves with joint `j` (expressed in that joint's rotated frame).
    seg_mass: Vec<f64>,
    seg_com: Vec<Vec3>,
    /// Inertia tensor of each segment about its centre of mass, in the segment frame
    /// (URDF `<inertia>` of its links, parallel-axis lumped). Used by `src/dynamics.rs`.
    seg_inertia: Vec<Mat3>,
    /// URDF `<limit>` of each arm joint (URDF angles), when given.
    pub urdf_limits: Vec<Option<(f64, f64)>>,
    /// Every link reachable from `base_link`: `(link_name, segment_idx, rel_tf_in_segment)`.
    /// `segment_idx == 0` is `base_link`, `segment_idx == j + 1` moves with joint `j`.
    pub(crate) link_frames: Vec<(String, usize, Tf)>,
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

/// Check whether tool X axis is vertical (pointing straight up or down).
pub fn is_tool_vertical(r: &k::Mat3) -> bool {
    (1.0 - k::column(r, 0)[2].abs()) < 1e-3
}

impl KinematicChain {
    pub fn is_tool_vertical(r: &k::Mat3) -> bool {
        is_tool_vertical(r)
    }

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
        // second moments about the segment origin
        let mut j_sum = vec![[[0.0f64; 3]; 3]; n + 1];
        let mut stack: Vec<(&str, usize, Tf, bool)> = vec![(spec.base_link.as_str(), 0, (IDENTITY, [0.0; 3]), false)];
        let mut link_frames: Vec<(String, usize, Tf)> = Vec::new();
        let mut visited = 0usize;
        while let Some((link, seg, t, crossed_movable)) = stack.pop() {
            visited += 1;
            if visited > urdf.links.len() { return Err("URDF link graph has a cycle".into()); }
            if !crossed_movable {
                link_frames.push((link.to_string(), seg, t));
            }
            if let Some(i) = urdf.link(link).and_then(|l| l.inertial.as_ref()) {
                let c = k::add(&t.1, &k::mat_vec(&t.0, &i.com));
                m_sum[seg] += i.mass;
                for a in 0..3 { mc_sum[seg][a] += i.mass * c[a]; }
                let [ixx, ixy, ixz, iyy, iyz, izz] = i.inertia;
                let r = k::mat_mul(&t.0, &i.rot);
                let rotated = k::mat_mul(&k::mat_mul(&r, &[[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]), &k::transpose(&r));
                let shift = parallel_axis(i.mass, &c);
                for a in 0..3 { for b in 0..3 { j_sum[seg][a][b] += rotated[a][b] + shift[a][b]; } }
            }
            for jt in urdf.child_joints(link) {
                match names.iter().position(|nm| *nm == jt.name) {
                    Some(idx) => stack.push((&jt.child, idx + 1, (IDENTITY, [0.0; 3]), crossed_movable)),
                    None => {
                        let child_crossed = crossed_movable || jt.kind != JointType::Fixed;
                        stack.push((&jt.child, seg, tf_mul(&t, &origin_tf(jt)), child_crossed));
                    }
                }
            }
        }
        let seg_com: Vec<Vec3> = m_sum.iter().zip(&mc_sum)
            .map(|(m, mc)| if *m > 0.0 { [mc[0] / m, mc[1] / m, mc[2] / m] } else { [0.0; 3] })
            .collect();
        let seg_inertia = (0..=n).map(|sg| {
            let back = parallel_axis(m_sum[sg], &seg_com[sg]);
            std::array::from_fn(|a| std::array::from_fn(|b| j_sum[sg][a][b] - back[a][b]))
        }).collect();

        let tip_rot = k::rpy_to_r(spec.tip_rpy_rad[0], spec.tip_rpy_rad[1], spec.tip_rpy_rad[2]);

        Ok(Self {
            robot: urdf.name.clone(),
            base_link: spec.base_link.clone(),
            tip_link: spec.tip_link.clone(),
            joint_names: names.iter().map(|s| s.to_string()).collect(),
            pre_r, pre_p, axis,
            sign: spec.joints.iter().map(|j| j.sign).collect(),
            offset: spec.joints.iter().map(|j| j.offset_rad).collect(),
            tip,
            tip_rot,
            canonical_end_frame: spec.canonical_end_frame,
            seg_mass: m_sum,
            seg_com,
            seg_inertia,
            urdf_limits: movable.iter().map(|j| j.limit).collect(),
            link_frames,
        })
    }

    /// Parse `text` and build the chain.
    pub fn from_urdf_str(text: &str, spec: &ChainSpec) -> Result<Self, String> {
        Self::from_urdf(&crate::urdf::parse(text)?, spec)
    }

    pub fn dof(&self) -> usize { self.axis.len() }

    /// Mass (kg), centre of mass and inertia about it (segment frame) of the segment that
    /// moves with joint `j`.
    pub fn segment(&self, j: usize) -> (f64, Vec3, Mat3) { (self.seg_mass[j + 1], self.seg_com[j + 1], self.seg_inertia[j + 1]) }

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
        r = k::mat_mul(&r, &self.tip_rot);
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
        self.solve_dls_ex(goal, q0, lo, hi, w, iters, false)
    }

    /// Damped-least-squares IK for `n` joints, with optional `yaw_free` (auto_yaw) for 5-DOF chains.
    pub fn solve_dls_ex(&self, goal: &Pose, q0: &[f64], lo: &[f64], hi: &[f64], w: f64, iters: usize, yaw_free: bool) -> ChainIk {
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

        let r_xy = (goal.p[0] * goal.p[0] + goal.p[1] * goal.p[1]).sqrt();
        let use_auto_yaw = yaw_free && n == 5 && r_xy >= 0.03;

        let (free_yaw, z_weight) = if use_auto_yaw {
            let rg_x_z = k::column(rg, 0)[2];
            let perp = (1.0 - rg_x_z * rg_x_z).max(0.0).sqrt();
            if perp <= 0.045 {
                (0.0, 1.0)
            } else if perp >= 0.14 {
                (1.0, 0.0)
            } else {
                let t = (perp - 0.045) / (0.14 - 0.045);
                let fy = t * t * (3.0 - 2.0 * t);
                (fy, 1.0 - fy)
            }
        } else {
            (0.0, 1.0)
        };

        let align_rot = |r_curr: &Mat3| -> Mat3 {
            if free_yaw > 0.0 {
                let m = k::mat_mul(rg, &k::transpose(r_curr));
                let a = m[0][0] + m[1][1];
                let b = m[0][1] - m[1][0];
                let dpsi = b.atan2(a) * free_yaw;
                k::mat_mul(&k::axis_r(&[0.0, 0.0, 1.0], dpsi), rg)
            } else {
                *rg
            }
        };

        let mut r_eff = align_rot(&f.end.r);
        let mut e = k::pose_err_end(&f.end, &r_eff, pg, w);
        e.e[5] *= z_weight;
        let er_eff = if w > 1e-9 {
            (e.e[3] * e.e[3] + e.e[4] * e.e[4] + e.e[5] * e.e[5]).sqrt() / w
        } else {
            e.er
        };
        e.er = er_eff;
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
                let az = w * a[2] * z_weight;
                jac[j] = [v[0], v[1], v[2], w * a[0], w * a[1], az];
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
            let r_eff_n = align_rot(&fn_.end.r);
            let mut en = k::pose_err_end(&fn_.end, &r_eff_n, pg, w);
            en.e[5] *= z_weight;
            let en_er_eff = if w > 1e-9 {
                (en.e[3] * en.e[3] + en.e[4] * en.e[4] + en.e[5] * en.e[5]).sqrt() / w
            } else {
                en.er
            };
            en.er = en_er_eff;
            if en.ep + w * en.er < e.ep + w * e.er {
                q = qn; f = fn_; e = en; r_eff = r_eff_n; lam = PAR.lam_min;
            } else if lam < PAR.lam_max {
                lam = PAR.lam_max;
            } else {
                q = qn; f = fn_; e = en; r_eff = r_eff_n;
            }
        }
        let _ = r_eff;
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

/// Mapping of one armd joint to its model (URDF) joint: model angle = `sign * armd angle +
/// offset_rad` (the [`ChainJointSpec`] convention). Published in the observation schema as
/// `model_joint_map`.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct JointMap {
    pub sign: f64,
    pub offset_rad: f64,
}

/// The arm model the profile selects: the built-in B601-DM code with its motor -> model
/// offsets (model angle = motor angle + offset, profile `model_offsets_rad`, default
/// [`k::B601DM_MODEL_OFFSETS_RAD`]) and an optional kinematic correction (calibration.json
/// `kinematics`, [`k::frames_corrected`]), or a URDF chain (sign / offset per joint in the
/// chain spec; no correction).
#[derive(Debug, Clone)]
pub enum ArmModel {
    RebotDevArm([f64; 6], Option<Arc<k::PreparedCorrection>>),
    Chain(Arc<KinematicChain>),
}

impl Default for ArmModel {
    fn default() -> Self { Self::REBOT_DEV_ARM }
}

impl ArmModel {
    /// The built-in model with the measured B601-DM offsets.
    pub const REBOT_DEV_ARM: Self = Self::RebotDevArm(k::B601DM_MODEL_OFFSETS_RAD, None);

    /// The built-in model with the motor -> model offsets `off`, uncorrected.
    pub fn rebot_dev_arm(off: [f64; 6]) -> Self { Self::RebotDevArm(off, None) }

    /// The kinematic correction in effect (built-in model only).
    pub fn correction(&self) -> Option<&k::Correction> {
        match self { Self::RebotDevArm(_, c) => c.as_deref().map(|p| &p.joints), Self::Chain(_) => None }
    }

    /// This model with the kinematic correction `corr` (None or all zero = uncorrected).
    /// Err for a URDF chain (v1 corrects the built-in model only) and for values outside the
    /// bounds ([`k::validate_correction`]).
    pub fn with_correction(&self, corr: Option<&k::Correction>) -> Result<Self, String> {
        let corr = corr.filter(|c| c.iter().any(|j| !j.is_zero()));
        match self {
            Self::RebotDevArm(off, _) => {
                if let Some(c) = corr { k::validate_correction(c)?; }
                Ok(Self::RebotDevArm(*off, corr.map(|c| Arc::new(k::PreparedCorrection::new(c)))))
            }
            Self::Chain(_) if corr.is_none() => Ok(self.clone()),
            Self::Chain(_) => Err("a kinematic correction needs the built-in RebotDevArm model; this profile uses a URDF chain".into()),
        }
    }

    /// armd -> model mapping of each arm joint.
    pub fn joint_map(&self) -> Vec<JointMap> {
        match self {
            Self::RebotDevArm(off, _) => off.iter().map(|o| JointMap { sign: 1.0, offset_rad: *o }).collect(),
            Self::Chain(c) => c.sign.iter().zip(&c.offset).map(|(s, o)| JointMap { sign: *s, offset_rad: *o }).collect(),
        }
    }

    /// Model angles of the built-in model for armd angles `q` (at least 6 entries).
    fn builtin_q(off: &[f64; 6], q: &[f64]) -> [f64; 6] { std::array::from_fn(|i| q[i] + off[i]) }

    /// Number of arm joints (armd joints `0..arm_dof`).
    pub fn arm_dof(&self) -> usize {
        match self { Self::RebotDevArm(..) => 6, Self::Chain(c) => c.dof() }
    }

    pub fn chain(&self) -> Option<&KinematicChain> {
        match self { Self::RebotDevArm(..) => None, Self::Chain(c) => Some(c) }
    }

    /// Tip pose for armd angles `q` (at least `arm_dof` entries).
    pub fn fk(&self, q: &[f64]) -> Pose {
        match self {
            Self::RebotDevArm(off, c) => k::fk_prepared(&Self::builtin_q(off, q), c.as_deref()),
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

    /// All link frames for an f32 joint vector in a single FK pass.
    pub fn frames_f32(&self, q: &[f32]) -> Option<ChainFrames> {
        let n = self.arm_dof();
        if q.len() < n || !q[..n].iter().all(|v| v.is_finite()) { return None; }
        let mut q64 = [0.0; MAX_CHAIN_JOINTS];
        for (o, v) in q64.iter_mut().zip(&q[..n]) { *o = f64::from(*v); }
        Some(match self {
            Self::RebotDevArm(off, c) => {
                // the live model: the kinematic correction when one is installed
                let mq = Self::builtin_q(off, &q64);
                let f = k::frames_prepared(&mq, c.as_deref());
                let mut axes = [[0.0; 3]; MAX_CHAIN_JOINTS];
                let mut origins = [[0.0; 3]; MAX_CHAIN_JOINTS];
                let mut seg_r = [k::IDENTITY; MAX_CHAIN_JOINTS];
                axes[..6].copy_from_slice(&f.axes);
                origins[..6].copy_from_slice(&f.origins);
                seg_r[..6].copy_from_slice(&f.seg_r);
                ChainFrames { n: 6, end: f.end, axes, origins, seg_r }
            }
            Self::Chain(c) => c.frames(&q64),
        })
    }

    /// DLS IK (the Cartesian actions).
    pub fn solve_dls(&self, goal: &Pose, q0: &[f64; 6], lo: &[f64; 6], hi: &[f64; 6], w: f64) -> IkResult {
        self.solve_dls_ex(goal, q0, lo, hi, w, false)
    }

    /// DLS IK with optional `yaw_free` for 5-DOF chains.
    pub fn solve_dls_ex(&self, goal: &Pose, q0: &[f64; 6], lo: &[f64; 6], hi: &[f64; 6], w: f64, yaw_free: bool) -> IkResult {
        match self {
            Self::RebotDevArm(off, c) => {
                let m = |q: &[f64; 6]| Self::builtin_q(off, q);
                let r = k::solve_dls_prepared(goal, &m(q0), &m(lo), &m(hi), w, &PAR, c.as_deref());
                IkResult { q: std::array::from_fn(|i| r.q[i] - off[i]), ..r }
            }
            Self::Chain(c) if c.dof() == 6 => {
                let r = c.solve_dls(goal, q0, lo, hi, w, PAR.iters);
                IkResult { ok: r.ok, q: std::array::from_fn(|i| r.q[i]), ep: r.ep, er: r.er, iterations: r.iterations }
            }
            Self::Chain(c) if c.dof() == 5 => {
                let iters = 35;
                let r = c.solve_dls_ex(goal, q0, lo, hi, w, iters, yaw_free);
                let mut q = *q0;
                q[..5].copy_from_slice(&r.q[..5]);
                IkResult { ok: r.ok, q, ep: r.ep, er: r.er, iterations: r.iterations }
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
            // the correction is geometric (mm, < 1 deg); the gravity model stays the URDF's
            Self::RebotDevArm(off, _) => {
                // offsets are f64; the model angle in f32 loses < 1e-7 rad
                let tau = crate::gravity::compute_gravity_torques(&std::array::from_fn(|i| (f64::from(q[i]) + off[i]) as f32))?;
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
            Self::RebotDevArm(off, c) => { let f = k::frames_prepared(off, c.as_deref()); (f.origins.to_vec(), f.end.p) }
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
        // the reach origin is a policy constant; the correction does not move it
        match self { Self::RebotDevArm(..) => k::SHOULDER_RZ, Self::Chain(c) => c.shoulder_rz() }
    }

    /// Camera mount R_tip_cam (`kinematics::cam_mount`): the wrist axis (second-to-last joint)
    /// seen in the tip frame at the rest pose (built-in model: URDF angles 0; URDF chain: armd
    /// angles 0, the official B601 convention where the rest pose reads 0). For a canonical end frame, computes R_end_cam directly.
    /// The nominal (uncorrected) chain defines this mount: the kinematic correction does not
    /// change the legacy mount or the tilt derived from `q_end_cam`.
    pub fn cam_mount(&self, flip: bool, rot_deg: f64, tilt_deg: f64) -> Mat3 {
        match self {
            Self::RebotDevArm(..) => k::cam_mount(flip, rot_deg, tilt_deg),
            Self::Chain(c) if c.canonical_end_frame => {
                let end_r = k::IDENTITY;
                let wrist_axis = [0.0, 0.0, -1.0];
                k::cam_mount_from(&end_r, &wrist_axis, flip, rot_deg, tilt_deg)
            }
            Self::Chain(c) => {
                let n = c.dof();
                let f = c.frames(&[0.0; MAX_CHAIN_JOINTS]);
                let a = f.axes[n.saturating_sub(2)];
                k::cam_mount_from(&f.end.r, &a, flip, rot_deg, tilt_deg)
            }
        }
    }

    /// R_end_cam of `mount`: the calibrated `q_end_cam` when set, else
    /// [`Self::cam_mount`] from flip / rot_deg and `tilt_deg` (the grasp setup's tilt for grasps).
    pub fn r_end_cam(&self, mount: &crate::model::CameraMount, tilt_deg: f64) -> Mat3 {
        match &mount.q_end_cam {
            Some(q) => k::quat_to_r(q),
            None => self.cam_mount(mount.flip, f64::from(mount.rot_deg), tilt_deg),
        }
    }

    /// The tilt (deg) `mount` stands for: `tilt_deg`, or the tilt derived from `q_end_cam`
    /// ([`k::derived_tilt_deg`]).
    pub fn mount_tilt_deg(&self, mount: &crate::model::CameraMount) -> f64 {
        match &mount.q_end_cam {
            Some(q) => k::derived_tilt_deg(&k::quat_to_r(q), &self.cam_mount(mount.flip, 0.0, 0.0)),
            None => f64::from(mount.tilt_deg),
        }
    }

    /// Workspace excess for armd joint positions `q` (`kinematics::workspace_violation`).
    pub fn workspace_violation(&self, q: &[f32], ws: &crate::model::WorkspacePolicy, cam: &crate::model::CameraMount) -> k::Violation {
        let Some(f) = self.frames_f32(q) else { return k::Violation::invalid(); };
        let mut v = k::workspace_violation_pose_at(&f.end, ws, cam, self.shoulder_rz());
        if !ws.link_capsules.is_empty() {
            let floor = f64::from(ws.floor_z_m);
            let lo = ws.box_min_m.map(f64::from);
            let hi = ws.box_max_m.map(f64::from);
            let base_pt = 1 + ws.check_points_end_m.len() + usize::from(cam.offset_m.is_some());
            let mut note = |excess: f64, point: usize, limit: &'static str, value: f64, bound: f64, keepout: Option<usize>| {
                if excess > 0.0 {
                    v.total += excess;
                    if v.worst.is_none_or(|w| excess > (w.value - w.bound).abs()) {
                        v.worst = Some(k::ViolationDetail { point, limit, value, bound, keepout });
                    }
                }
            };
            let mut world_caps = Vec::with_capacity(ws.link_capsules.len());
            for (ci, cap) in ws.link_capsules.iter().enumerate() {
                let Some(fr) = self.resolve_frame(&cap.frame) else { return k::Violation::invalid(); };
                let pose = fr.pose(&f);
                let a = pose.transform_point(&cap.a_m.map(f64::from));
                let b = pose.transform_point(&cap.b_m.map(f64::from));
                let r = f64::from(cap.radius_m);
                world_caps.push((a, b, r));
                if !matches!(fr, FrameRef::BaseRel(_, _)) {
                    let pt = base_pt + ci;
                    let z_min = a[2].min(b[2]) - r;
                    note(floor - z_min, pt, "floor", z_min, floor, None);
                    for axis in 0..3 {
                        let c_min = a[axis].min(b[axis]) - r;
                        let c_max = a[axis].max(b[axis]) + r;
                        note(lo[axis] - c_min, pt, k::BOX_LIMITS[axis][0], c_min, lo[axis], None);
                        note(c_max - hi[axis], pt, k::BOX_LIMITS[axis][1], c_max, hi[axis], None);
                    }
                    for (bi, kb) in ws.keepout_boxes.iter().enumerate() {
                        let clr = k::capsule_aabb_clearance(&a, &b, r, &kb.min_m.map(f64::from), &kb.max_m.map(f64::from));
                        note(-clr, pt, "keepout", -clr, 0.0, Some(bi));
                    }
                }
            }
            if ws.self_collision_enabled {
                let min_clr = f64::from(ws.self_collision_min_clearance_m);
                if !min_clr.is_finite() || min_clr < 0.0 {
                    return k::Violation::invalid();
                }
                for [id_a, id_b] in &ws.self_collision_pairs {
                    let Some(ia) = ws.link_capsules.iter().position(|c| c.id == *id_a) else {
                        return k::Violation::invalid();
                    };
                    let Some(ib) = ws.link_capsules.iter().position(|c| c.id == *id_b) else {
                        return k::Violation::invalid();
                    };
                    if ia == ib {
                        return k::Violation::invalid();
                    }
                    let (a0, a1, ra) = world_caps[ia];
                    let (b0, b1, rb) = world_caps[ib];
                    let dist = seg_seg_dist(&a0, &a1, &b0, &b1);
                    let clr = dist - (ra + rb) - min_clr;
                    note(-clr, base_pt + ia, "self_collision", min_clr + clr, min_clr, None);
                }
            }
        }
        v
    }
}

/// Shortest Euclidean distance between 3D line segments `[p1, q1]` and `[p2, q2]`.
pub fn seg_seg_dist(p1: &Vec3, q1: &Vec3, p2: &Vec3, q2: &Vec3) -> f64 {
    const LEN2_EPS: f64 = 1e-18;
    const PARALLEL_EPS: f64 = 1e-12;
    let d1 = k::sub(q1, p1);
    let d2 = k::sub(q2, p2);
    let r = k::sub(p1, p2);
    let a = k::dot(&d1, &d1);
    let e = k::dot(&d2, &d2);
    let pt_seg_dist = |pt: &Vec3, seg_a: &Vec3, seg_d: &Vec3, seg_len2: f64| -> f64 {
        let ap = k::sub(pt, seg_a);
        let proj = (k::dot(&ap, seg_d) / seg_len2).clamp(0.0, 1.0);
        k::norm(&[ap[0] - seg_d[0] * proj, ap[1] - seg_d[1] * proj, ap[2] - seg_d[2] * proj])
    };
    if a <= LEN2_EPS && e <= LEN2_EPS {
        return k::norm(&r);
    }
    if a <= LEN2_EPS {
        return pt_seg_dist(p1, p2, &d2, e);
    }
    if e <= LEN2_EPS {
        return pt_seg_dist(p2, p1, &d1, a);
    }
    let b = k::dot(&d1, &d2);
    let denom = a * e - b * b;
    if denom <= PARALLEL_EPS * a * e {
        return pt_seg_dist(p1, p2, &d2, e)
            .min(pt_seg_dist(q1, p2, &d2, e))
            .min(pt_seg_dist(p2, p1, &d1, a))
            .min(pt_seg_dist(q2, p1, &d1, a));
    }
    let c = k::dot(&d1, &r);
    let f = k::dot(&d2, &r);
    let mut s = ((b * f - c * e) / denom).clamp(0.0, 1.0);
    let mut t = (b * s + f) / e;
    if t < 0.0 {
        t = 0.0;
        s = (-c / a).clamp(0.0, 1.0);
    } else if t > 1.0 {
        t = 1.0;
        s = ((b - c) / a).clamp(0.0, 1.0);
    }
    let c1 = k::add(p1, &[d1[0] * s, d1[1] * s, d1[2] * s]);
    let c2 = k::add(p2, &[d2[0] * t, d2[1] * t, d2[2] * t]);
    k::norm(&k::sub(&c1, &c2))
}

/// Resolved kinematic frame for a `LinkCapsule`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FrameRef {
    BaseRel(Mat3, Vec3),
    /// `seg` is 0-based joint index (`0..arm_dof`), with relative transform in that segment's frame.
    SegRel(usize, Mat3, Vec3),
    End,
}

impl FrameRef {
    pub fn pose(&self, f: &ChainFrames) -> Pose {
        match *self {
            Self::BaseRel(r, p) => Pose::new(r, p),
            Self::SegRel(j, r, p) => Pose::new(
                k::mat_mul(&f.seg_r[j], &r),
                k::add(&f.origins[j], &k::mat_vec(&f.seg_r[j], &p)),
            ),
            Self::End => f.end,
        }
    }

    /// Number of proximal arm joints (`0..=arm_dof`) that move this frame.
    fn active_joints(&self, arm_dof: usize) -> usize {
        match *self {
            Self::BaseRel(_, _) => 0,
            Self::SegRel(j, _, _) => (j + 1).min(arm_dof),
            Self::End => arm_dof,
        }
    }
}

fn pair_relative_reach(
    arm_dof: usize,
    fr_a: Option<FrameRef>,
    reach_a: &[f64; MAX_CHAIN_JOINTS],
    fr_b: Option<FrameRef>,
    reach_b: &[f64; MAX_CHAIN_JOINTS],
) -> [f64; MAX_CHAIN_JOINTS] {
    let (Some(fa), Some(fb)) = (fr_a, fr_b) else {
        return [f64::INFINITY; MAX_CHAIN_JOINTS];
    };
    let sa = fa.active_joints(arm_dof);
    let sb = fb.active_joints(arm_dof);
    let mut out = [0.0; MAX_CHAIN_JOINTS];
    match sa.cmp(&sb) {
        std::cmp::Ordering::Less => {
            out[sa..sb].copy_from_slice(&reach_b[sa..sb]);
        }
        std::cmp::Ordering::Greater => {
            out[sb..sa].copy_from_slice(&reach_a[sb..sa]);
        }
        std::cmp::Ordering::Equal => {}
    }
    out
}

impl ArmModel {
    /// Resolve a link / kinematic frame / joint name to a `FrameRef`.
    pub fn resolve_frame(&self, name: &str) -> Option<FrameRef> {
        let n = self.arm_dof();
        if name == "end_link" {
            return Some(FrameRef::End);
        }
        if name == "base_link" {
            return Some(FrameRef::BaseRel(k::IDENTITY, [0.0; 3]));
        }
        // Built-in semantic frame names
        let semantic = match name {
            "upper_arm" if n >= 2 => Some(1),
            "elbow" if n >= 3 => Some(2),
            "forearm" if n >= 6 => Some(3),
            "forearm" if n >= 3 => Some(2),
            "wrist" if n >= 5 => Some(4),
            _ => None,
        };
        if let Some(j) = semantic {
            return Some(FrameRef::SegRel(j, k::IDENTITY, [0.0; 3]));
        }
        // 1-based joint index aliases: joint_1..joint_N, joint1..jointN, j1..jN
        let j_suffix = name
            .strip_prefix("joint_")
            .or_else(|| name.strip_prefix("joint"))
            .or_else(|| name.strip_prefix('j'));
        if let Some(s) = j_suffix {
            if let Ok(idx) = s.parse::<usize>() {
                if (1..=n).contains(&idx) {
                    return Some(FrameRef::SegRel(idx - 1, k::IDENTITY, [0.0; 3]));
                }
            }
        }
        match self {
            Self::RebotDevArm(..) => {
                if let Some(s) = name.strip_prefix("link") {
                    if let Ok(idx) = s.parse::<usize>() {
                        if (1..=6).contains(&idx) {
                            return Some(FrameRef::SegRel(idx - 1, k::IDENTITY, [0.0; 3]));
                        }
                    }
                }
                None
            }
            Self::Chain(c) => {
                if let Some((_, seg, tf)) = c.link_frames.iter().find(|(ln, _, _)| ln == name) {
                    return Some(if *seg == 0 {
                        FrameRef::BaseRel(tf.0, tf.1)
                    } else {
                        FrameRef::SegRel(*seg - 1, tf.0, tf.1)
                    });
                }
                if let Some(idx) = c.joint_names.iter().position(|jn| jn == name) {
                    return Some(FrameRef::SegRel(idx, k::IDENTITY, [0.0; 3]));
                }
                None
            }
        }
    }

    /// Upper bound on Cartesian motion per joint for a point at local distance `max_offset`
    /// inside `frame`.
    pub fn joint_reach_for_frame(&self, frame: FrameRef, max_offset: f64) -> [f64; MAX_CHAIN_JOINTS] {
        let n = self.arm_dof();
        match frame {
            FrameRef::BaseRel(_, _) => [0.0; MAX_CHAIN_JOINTS],
            FrameRef::End => self.joint_reach(max_offset),
            FrameRef::SegRel(s, _, rel_p) => {
                let s = s.min(n.saturating_sub(1));
                let origins: [Vec3; MAX_CHAIN_JOINTS] = match self {
                    Self::RebotDevArm(off, c) => {
                        let f = k::frames_prepared(off, c.as_deref());
                        let mut arr = [[0.0; 3]; MAX_CHAIN_JOINTS];
                        arr[..6].copy_from_slice(&f.origins);
                        arr
                    }
                    Self::Chain(c) => c.frames(&[0.0; MAX_CHAIN_JOINTS]).origins,
                };
                let mut reach = [0.0; MAX_CHAIN_JOINTS];
                let mut acc = max_offset + k::norm(&rel_p);
                for j in (0..=s).rev() {
                    reach[j] = acc;
                    if j > 0 {
                        acc += k::norm(&k::sub(&origins[j], &origins[j - 1]));
                    }
                }
                reach
            }
        }
    }
}

/// Failure produced by [`WorkspaceChecker`].
#[derive(Debug, Clone, PartialEq)]
pub struct WorkspaceFailure {
    pub point: usize,
    pub limit: String,
    pub value: f64,
    pub bound: f64,
    pub detail: Box<k::WorkspaceViolationDetail>,
}

impl WorkspaceFailure {
    pub fn invalid(phase: &str, reason: impl Into<String>) -> Self {
        Self {
            point: 0,
            limit: "invalid".into(),
            value: f64::NAN,
            bound: 0.0,
            detail: Box::new(k::WorkspaceViolationDetail {
                constraint_id: Some("invalid".into()),
                geometry_id: Some("end_origin".into()),
                violation_phase: Some(phase.into()),
                reason: Some(reason.into()),
            }),
        }
    }

    pub fn with_phase(mut self, phase: &str) -> Self {
        self.detail.violation_phase = Some(phase.into());
        self
    }
}

/// Precomputed per-constraint workspace evaluator (points + optional link capsules + optional self-collision pairs).
#[derive(Debug, Clone)]
pub struct WorkspaceEvaluator<'a> {
    pub model: &'a ArmModel,
    pub ws: &'a crate::model::WorkspacePolicy,
    pub cam: &'a crate::model::CameraMount,
    shoulder: [f64; 2],
    floor: f64,
    box_lo: [f64; 3],
    box_hi: [f64; 3],
    keepouts: Vec<([f64; 3], [f64; 3])>,
    end_locals: Vec<[f64; 3]>,
    has_cam: bool,
    capsules: Vec<(Option<FrameRef>, [f64; 3], [f64; 3], f64)>,
    self_pairs: Vec<Option<(usize, usize, [f64; MAX_CHAIN_JOINTS])>>,
    self_collision_min_clearance: f64,
    pub end_reach: [f64; MAX_CHAIN_JOINTS],
    pub cap_reaches: Vec<[f64; MAX_CHAIN_JOINTS]>,
    pub slots_per_geom: usize,
    pub num_end_pts: usize,
    pub num_geoms: usize,
    pub has_reach: bool,
    pub self_collision_slot_start: usize,
    pub total_slots: usize,
}

impl<'a> WorkspaceEvaluator<'a> {
    pub fn new(
        model: &'a ArmModel,
        ws: &'a crate::model::WorkspacePolicy,
        cam: &'a crate::model::CameraMount,
    ) -> Self {
        let mut end_locals = Vec::with_capacity(1 + ws.check_points_end_m.len() + 1);
        end_locals.push([0.0; 3]);
        for p in &ws.check_points_end_m {
            end_locals.push(p.map(f64::from));
        }
        let has_cam = cam.offset_m.is_some();
        if let Some(o) = cam.offset_m {
            end_locals.push(o.map(f64::from));
        }
        let max_end_off = end_locals.iter().map(k::norm).fold(0.0, f64::max);
        let end_reach = model.joint_reach(max_end_off);

        let mut capsules = Vec::with_capacity(ws.link_capsules.len());
        let mut cap_reaches = Vec::with_capacity(ws.link_capsules.len());
        for c in &ws.link_capsules {
            let fr = model.resolve_frame(&c.frame);
            let a = c.a_m.map(f64::from);
            let b = c.b_m.map(f64::from);
            let max_off = k::norm(&a).max(k::norm(&b));
            let reach = match fr {
                Some(fr) => model.joint_reach_for_frame(fr, max_off),
                None => [f64::INFINITY; MAX_CHAIN_JOINTS],
            };
            capsules.push((fr, a, b, f64::from(c.radius_m)));
            cap_reaches.push(reach);
        }

        let arm_dof = model.arm_dof();
        let mut self_pairs = Vec::new();
        if ws.self_collision_enabled && !ws.link_capsules.is_empty() {
            self_pairs.reserve(ws.self_collision_pairs.len());
            for [id_a, id_b] in &ws.self_collision_pairs {
                let ia = ws.link_capsules.iter().position(|c| c.id == *id_a);
                let ib = ws.link_capsules.iter().position(|c| c.id == *id_b);
                let resolved = match (ia, ib) {
                    (Some(a), Some(b)) if a != b => {
                        let reach = pair_relative_reach(
                            arm_dof,
                            capsules[a].0,
                            &cap_reaches[a],
                            capsules[b].0,
                            &cap_reaches[b],
                        );
                        Some((a, b, reach))
                    }
                    _ => None,
                };
                self_pairs.push(resolved);
            }
        }

        let keepouts: Vec<([f64; 3], [f64; 3])> = ws
            .keepout_boxes
            .iter()
            .map(|b| (b.min_m.map(f64::from), b.max_m.map(f64::from)))
            .collect();

        let num_end_pts = end_locals.len();
        let num_geoms = num_end_pts + capsules.len();
        let slots_per_geom = 7 + keepouts.len();
        let has_reach = ws.max_reach_m.is_some();
        let self_collision_slot_start = num_geoms * slots_per_geom + usize::from(has_reach);
        let total_slots = self_collision_slot_start + self_pairs.len();

        Self {
            model,
            ws,
            cam,
            shoulder: model.shoulder_rz(),
            floor: f64::from(ws.floor_z_m),
            box_lo: ws.box_min_m.map(f64::from),
            box_hi: ws.box_max_m.map(f64::from),
            keepouts,
            end_locals,
            has_cam,
            capsules,
            self_pairs,
            self_collision_min_clearance: f64::from(ws.self_collision_min_clearance_m),
            end_reach,
            cap_reaches,
            slots_per_geom,
            num_end_pts,
            num_geoms,
            has_reach,
            self_collision_slot_start,
            total_slots,
        }
    }

    pub fn alloc_buf(&self) -> k::SlotBuf {
        k::SlotBuf::zeros(self.total_slots)
    }

    /// Geometry index for slot `slot`.
    #[inline]
    pub fn slot_geom(&self, slot: usize) -> usize {
        if slot < self.num_geoms * self.slots_per_geom {
            slot / self.slots_per_geom
        } else {
            0 // max_reach belongs to end_origin (geometry 0)
        }
    }

    /// Motion bound of geometry `g` for joint travel `tr`.
    #[inline]
    pub fn geom_bound(&self, g: usize, tr: &[f64], arm_dof: usize) -> f64 {
        let reach = if g < self.num_end_pts {
            &self.end_reach
        } else {
            &self.cap_reaches[g - self.num_end_pts]
        };
        tr.iter().take(arm_dof).zip(reach).map(|(d, r)| d * r).sum()
    }

    /// Evaluate clearances of all `(geometry, constraint)` slots at `q` into `out`.
    /// Returns `Err` if FK fails or a capsule references an unknown frame.
    pub fn eval_into(&self, q: &[f32], out: &mut k::SlotBuf, phase: &str) -> Result<(), WorkspaceFailure> {
        let Some(f) = self.model.frames_f32(q) else {
            return Err(WorkspaceFailure::invalid(phase, "non-finite or incomplete joint positions"));
        };
        let buf = out.as_mut_slice();
        for (gi, local) in self.end_locals.iter().enumerate() {
            let p = f.end.transform_point(local);
            let base = gi * self.slots_per_geom;
            buf[base] = p[2] - self.floor;
            for axis in 0..3 {
                buf[base + 1 + axis] = p[axis] - self.box_lo[axis];
                buf[base + 4 + axis] = self.box_hi[axis] - p[axis];
            }
            for (bi, (kb_lo, kb_hi)) in self.keepouts.iter().enumerate() {
                let depth = (0..3)
                    .map(|axis| (p[axis] - kb_lo[axis]).min(kb_hi[axis] - p[axis]))
                    .fold(f64::INFINITY, f64::min);
                buf[base + 7 + bi] = -depth;
            }
        }
        let mut stack_caps = [([0.0f64; 3], [0.0f64; 3], 0.0f64); crate::model::MAX_LINK_CAPSULES];
        let mut heap_caps = Vec::new();
        let world_caps: &mut [([f64; 3], [f64; 3], f64)] = if self.capsules.len() <= stack_caps.len() {
            &mut stack_caps[..self.capsules.len()]
        } else {
            heap_caps.resize(self.capsules.len(), ([0.0; 3], [0.0; 3], 0.0));
            &mut heap_caps
        };
        for (ci, (fr_opt, a_local, b_local, radius)) in self.capsules.iter().enumerate() {
            let Some(fr) = fr_opt else {
                let cap = &self.ws.link_capsules[ci];
                return Err(WorkspaceFailure {
                    point: self.num_end_pts + ci,
                    limit: "invalid".into(),
                    value: f64::NAN,
                    bound: 0.0,
                    detail: Box::new(k::WorkspaceViolationDetail {
                        constraint_id: Some("invalid".into()),
                        geometry_id: Some(format!("capsule:{}", cap.id)),
                        violation_phase: Some(phase.into()),
                        reason: Some(format!("unknown frame '{}' for capsule '{}'", cap.frame, cap.id)),
                    }),
                });
            };
            let pose = fr.pose(&f);
            let a = pose.transform_point(a_local);
            let b = pose.transform_point(b_local);
            world_caps[ci] = (a, b, *radius);
            let base = (self.num_end_pts + ci) * self.slots_per_geom;
            if matches!(fr, FrameRef::BaseRel(_, _)) {
                buf[base..base + self.slots_per_geom].fill(0.0);
            } else {
                buf[base] = a[2].min(b[2]) - *radius - self.floor;
                for axis in 0..3 {
                    buf[base + 1 + axis] = (a[axis].min(b[axis]) - *radius) - self.box_lo[axis];
                    buf[base + 4 + axis] = self.box_hi[axis] - (a[axis].max(b[axis]) + *radius);
                }
                for (bi, (kb_lo, kb_hi)) in self.keepouts.iter().enumerate() {
                    buf[base + 7 + bi] = k::capsule_aabb_clearance(&a, &b, *radius, kb_lo, kb_hi);
                }
            }
        }
        if let Some(reach) = self.ws.max_reach_m {
            let p0 = f.end.p;
            let d = k::shoulder_reach_from(&p0, &self.shoulder);
            buf[self.num_geoms * self.slots_per_geom] = f64::from(reach) - d;
        }
        if !self.self_pairs.is_empty() {
            if !self.self_collision_min_clearance.is_finite() || self.self_collision_min_clearance < 0.0 {
                return Err(WorkspaceFailure::invalid(phase, "invalid self_collision_min_clearance_m"));
            }
            for (pi, pair) in self.self_pairs.iter().enumerate() {
                let Some(&(ia, ib, _)) = pair.as_ref() else {
                    let msg = self.ws.self_collision_pairs.get(pi).map_or_else(
                        || "invalid self-collision pair".to_string(),
                        |[a, b]| format!("unknown or invalid self_collision pair ['{a}', '{b}']"),
                    );
                    return Err(WorkspaceFailure::invalid(phase, msg));
                };
                let (a0, a1, ra) = world_caps[ia];
                let (b0, b1, rb) = world_caps[ib];
                let dist = seg_seg_dist(&a0, &a1, &b0, &b1);
                buf[self.self_collision_slot_start + pi] = dist - (ra + rb) - self.self_collision_min_clearance;
            }
        }
        Ok(())
    }

    pub fn describe_failure(
        &self,
        slot: usize,
        clr: f64,
        depth: f64,
        min_d: f64,
        phase: &str,
    ) -> WorkspaceFailure {
        let reason_for = |cid: &str, gid: &str| -> String {
            if !depth.is_finite() {
                format!("invalid workspace check for {cid} ({gid})")
            } else if min_d <= k::WORKSPACE_EPS {
                format!("new intrusion into {cid} by {depth:.3}m (geometry {gid})")
            } else {
                format!("deepened {cid} intrusion from {min_d:.3}m to {depth:.3}m (geometry {gid})")
            }
        };
        if slot >= self.self_collision_slot_start {
            let pi = slot - self.self_collision_slot_start;
            let (id_a, id_b) = self
                .ws
                .self_collision_pairs
                .get(pi)
                .map_or(("?", "?"), |[a, b]| (a.as_str(), b.as_str()));
            let cid = format!("self_collision:{id_a}:{id_b}");
            let gid = format!("capsule:{id_a}");
            let point = self
                .self_pairs
                .get(pi)
                .and_then(|p| p.as_ref())
                .map_or(0, |(ia, _, _)| self.num_end_pts + ia);
            let reason = reason_for(&cid, &gid);
            return WorkspaceFailure {
                point,
                limit: format!("self_collision '{id_a}' vs '{id_b}'"),
                value: self.self_collision_min_clearance + clr,
                bound: self.self_collision_min_clearance,
                detail: Box::new(k::WorkspaceViolationDetail {
                    constraint_id: Some(cid),
                    geometry_id: Some(gid),
                    violation_phase: Some(phase.into()),
                    reason: Some(reason),
                }),
            };
        }
        if self.has_reach && slot == self.num_geoms * self.slots_per_geom {
            let bound = f64::from(self.ws.max_reach_m.unwrap_or(0.0));
            let cid = "max_reach".to_string();
            let gid = "end_origin".to_string();
            return WorkspaceFailure {
                point: 0,
                limit: "reach".into(),
                value: bound - clr,
                bound,
                detail: Box::new(k::WorkspaceViolationDetail {
                    reason: Some(reason_for(&cid, &gid)),
                    constraint_id: Some(cid),
                    geometry_id: Some(gid),
                    violation_phase: Some(phase.into()),
                }),
            };
        }
        let gi = slot / self.slots_per_geom;
        let ci = slot % self.slots_per_geom;
        let gid = if gi == 0 {
            "end_origin".to_string()
        } else if gi < self.num_end_pts {
            if self.has_cam && gi + 1 == self.num_end_pts {
                "camera_mount".to_string()
            } else {
                format!("end_point:{gi}")
            }
        } else {
            format!("capsule:{}", self.ws.link_capsules[gi - self.num_end_pts].id)
        };
        let (limit, cid, value, bound) = if ci == 0 {
            ("floor".to_string(), "floor_z".to_string(), self.floor + clr, self.floor)
        } else if ci <= 3 {
            let ax = ci - 1;
            (
                k::BOX_LIMITS[ax][0].to_string(),
                k::BOX_CONSTRAINT_IDS[ax][0].to_string(),
                self.box_lo[ax] + clr,
                self.box_lo[ax],
            )
        } else if ci <= 6 {
            let ax = ci - 4;
            (
                k::BOX_LIMITS[ax][1].to_string(),
                k::BOX_CONSTRAINT_IDS[ax][1].to_string(),
                self.box_hi[ax] - clr,
                self.box_hi[ax],
            )
        } else {
            let bi = ci - 7;
            let name = self.ws.keepout_boxes.get(bi).map_or("?", |b| b.name.as_str());
            (
                format!("keepout '{name}'"),
                format!("keepout:{name}"),
                depth,
                0.0,
            )
        };
        let reason = reason_for(&cid, &gid);
        WorkspaceFailure {
            point: gi,
            limit,
            value,
            bound,
            detail: Box::new(k::WorkspaceViolationDetail {
                constraint_id: Some(cid),
                geometry_id: Some(gid),
                violation_phase: Some(phase.into()),
                reason: Some(reason),
            }),
        }
    }

    /// Initialize `min_depths` from `start` clearances.
    pub fn init_min_depths(&self, clr: &k::SlotBuf) -> k::SlotBuf {
        let mut min_depths = self.alloc_buf();
        for (dst, &c) in min_depths.as_mut_slice().iter_mut().zip(clr.as_slice()) {
            *dst = if c.is_nan() { f64::NAN } else { (-c).max(0.0) };
        }
        min_depths
    }

    /// Check `clr` against `min_depths`, returning the first failing constraint or updating `min_depths`.
    pub fn check_and_advance(
        &self,
        clr: &k::SlotBuf,
        min_depths: &mut k::SlotBuf,
        phase: &str,
    ) -> Result<(), WorkspaceFailure> {
        let cs = clr.as_slice();
        if let Some((slot, depth, min_d, _allowed)) = k::first_workspace_rejection(cs, min_depths.as_slice()) {
            return Err(self.describe_failure(slot, cs[slot], depth, min_d, phase));
        }
        k::advance_min_depths(cs, min_depths.as_mut_slice());
        Ok(())
    }

    /// Check `clr` against `min_depths` without mutating `min_depths`.
    pub fn check_readonly(
        &self,
        clr: &k::SlotBuf,
        min_depths: &k::SlotBuf,
        phase: &str,
    ) -> Result<(), WorkspaceFailure> {
        let cs = clr.as_slice();
        if let Some((slot, depth, min_d, _allowed)) = k::first_workspace_rejection(cs, min_depths.as_slice()) {
            return Err(self.describe_failure(slot, cs[slot], depth, min_d, phase));
        }
        Ok(())
    }

    /// Conservative check of whether interval `[0, 1]` with endpoint clearances `c0, c1` and
    /// per-joint travel `tr` can exceed `min_depths` on any slot.
    /// Returns `Ok(Some((slot, d_max, min_d)))` for the first slot that cannot be proven safe.
    fn interval_needs_split(
        &self,
        c0: &k::SlotBuf,
        c1: &k::SlotBuf,
        min_depths: &k::SlotBuf,
        tr: &[f64],
        arm_dof: usize,
    ) -> Result<Option<(usize, f64, f64)>, WorkspaceFailure> {
        let mut geom_bounds = [0.0f64; 34];
        let mut heap_bounds = Vec::new();
        let bounds: &mut [f64] = if self.num_geoms <= geom_bounds.len() {
            &mut geom_bounds[..self.num_geoms]
        } else {
            heap_bounds.resize(self.num_geoms, 0.0);
            &mut heap_bounds
        };
        for (g, b) in bounds.iter_mut().enumerate() {
            let v = self.geom_bound(g, tr, arm_dof);
            if !v.is_finite() {
                return Err(WorkspaceFailure::invalid("sweep", "non-finite joint travel bound"));
            }
            *b = v;
        }
        let s0 = c0.as_slice();
        let s1 = c1.as_slice();
        let md = min_depths.as_slice();
        for slot in 0..self.total_slots {
            let l_g = if slot >= self.self_collision_slot_start {
                let pi = slot - self.self_collision_slot_start;
                let Some((_, _, reach)) = &self.self_pairs[pi] else {
                    return Err(WorkspaceFailure::invalid("sweep", "invalid self-collision pair"));
                };
                let v: f64 = tr.iter().take(arm_dof).zip(reach).map(|(d, r)| d * r).sum();
                if !v.is_finite() {
                    return Err(WorkspaceFailure::invalid("sweep", "non-finite joint travel bound"));
                }
                v
            } else {
                bounds[self.slot_geom(slot)]
            };
            if l_g <= 0.0 {
                continue;
            }
            let min_d = md[slot];
            let allowed = if min_d <= k::WORKSPACE_EPS {
                k::WORKSPACE_EPS
            } else {
                min_d + k::WORKSPACE_EPS
            };
            let d_max = 0.5 * (l_g - s0[slot] - s1[slot]);
            if d_max > allowed {
                return Ok(Some((slot, d_max, min_d)));
            }
        }
        Ok(None)
    }

    fn budget_failure(&self, max_budget: usize, phase: &str) -> WorkspaceFailure {
        WorkspaceFailure {
            point: 0,
            limit: format!("keepout check: the path needs more than {max_budget} FK evaluations to clear the workspace; bound"),
            value: 0.0,
            bound: k::WORKSPACE_EPS,
            detail: Box::new(k::WorkspaceViolationDetail {
                constraint_id: Some("sweep_budget".into()),
                geometry_id: Some("end_origin".into()),
                violation_phase: Some(phase.into()),
                reason: Some(format!("workspace sweep exceeded {max_budget} FK evaluations")),
            }),
        }
    }

    /// Verify a linear joint-space step `a -> b`, given `a`'s clearances `c_a` and running
    /// `min_depths`. Zero heap allocations when no subdivision is needed.
    /// Returns `(c_b)` on success and advances `min_depths` through `b`.
    pub fn sweep_linear_step(
        &self,
        a: &[f32],
        b: &[f32],
        c_a: &k::SlotBuf,
        min_depths: &mut k::SlotBuf,
        remaining_budget: &mut usize,
        max_budget: usize,
        endpoint_phase: &str,
        sweep_phase: &str,
    ) -> Result<k::SlotBuf, (f64, WorkspaceFailure)> {
        let mut c_b = self.alloc_buf();
        self.eval_into(b, &mut c_b, endpoint_phase).map_err(|e| (1.0, e))?;
        self.check_readonly(&c_b, min_depths, endpoint_phase).map_err(|e| (1.0, e))?;

        let dof = a.len().min(b.len()).min(MAX_CHAIN_JOINTS);
        let arm_dof = self.model.arm_dof().min(dof);
        let mut full_tr = [0.0f64; MAX_CHAIN_JOINTS];
        for j in 0..arm_dof {
            full_tr[j] = f64::from((b[j] - a[j]).abs());
        }
        if self
            .interval_needs_split(c_a, &c_b, min_depths, &full_tr[..arm_dof], arm_dof)
            .map_err(|e| (1.0, e.with_phase(sweep_phase)))?
            .is_none()
        {
            self.check_and_advance(&c_b, min_depths, endpoint_phase).map_err(|e| (1.0, e))?;
            return Ok(c_b);
        }

        let mut stack: Vec<(f64, k::SlotBuf, f64, k::SlotBuf)> = Vec::new();
        stack.push((0.0, c_a.clone(), 1.0, c_b.clone()));
        let mut qm_buf = [0.0f32; MAX_CHAIN_JOINTS];
        let mut tr_buf = [0.0f64; MAX_CHAIN_JOINTS];
        while let Some((t0, c0, t1, c1)) = stack.pop() {
            let dt = t1 - t0;
            for j in 0..arm_dof {
                tr_buf[j] = full_tr[j] * dt;
            }
            let failing = self
                .interval_needs_split(&c0, &c1, min_depths, &tr_buf[..arm_dof], arm_dof)
                .map_err(|e| (t0, e.with_phase(sweep_phase)))?;
            let Some((slot, d_max, min_d)) = failing else {
                let ph = if (t1 - 1.0).abs() < 1e-12 { endpoint_phase } else { sweep_phase };
                self.check_and_advance(&c1, min_depths, ph).map_err(|e| (t1, e))?;
                continue;
            };
            if dt <= 1e-9 {
                return Err((t0, self.describe_failure(slot, -d_max, d_max, min_d, sweep_phase)));
            }
            if *remaining_budget == 0 {
                return Err((t0, self.budget_failure(max_budget, sweep_phase)));
            }
            *remaining_budget -= 1;
            let tm = 0.5 * (t0 + t1);
            let tm32 = tm as f32;
            for j in 0..dof {
                qm_buf[j] = a[j] + tm32 * (b[j] - a[j]);
            }
            let mut cm = self.alloc_buf();
            self.eval_into(&qm_buf[..dof], &mut cm, sweep_phase).map_err(|e| (tm, e))?;
            self.check_readonly(&cm, min_depths, sweep_phase).map_err(|e| (tm, e))?;
            stack.push((tm, cm.clone(), t1, c1));
            stack.push((t0, c0, tm, cm));
        }
        Ok(c_b)
    }

    /// Chronological adaptive sweep over `knots` with path sampler `at(t)` and joint travel
    /// bound `travel(t0, t1)`. Returns all evaluated samples in ascending time order.
    pub fn sweep_knots(
        &self,
        at: &dyn Fn(f64) -> Vec<f32>,
        travel: &dyn Fn(f64, f64) -> Vec<f64>,
        knots: &[f64],
        budget: usize,
        phase: &str,
    ) -> Result<Vec<Vec<f32>>, WorkspaceFailure> {
        let Some(&t_start) = knots.first() else {
            return Err(WorkspaceFailure::invalid(phase, "empty knot list"));
        };
        let arm_dof = self.model.arm_dof();
        let first = at(t_start);
        let mut c_prev = self.alloc_buf();
        self.eval_into(&first, &mut c_prev, phase)?;
        let mut min_depths = self.init_min_depths(&c_prev);
        let mut samples = vec![first];
        let mut t_prev = t_start;
        let mut evals = 0usize;
        let mut stack: Vec<(f64, k::SlotBuf, f64, k::SlotBuf, Option<Vec<f32>>)> = Vec::new();

        for &t_next in &knots[1..] {
            let q_next = at(t_next);
            let mut c_next = self.alloc_buf();
            self.eval_into(&q_next, &mut c_next, phase)?;
            self.check_readonly(&c_next, &min_depths, phase)?;
            stack.push((t_prev, c_prev, t_next, c_next.clone(), Some(q_next)));
            while let Some((t0, c0, t1, c1, q1_opt)) = stack.pop() {
                let tr = travel(t0, t1);
                if tr.len() < arm_dof {
                    return Err(WorkspaceFailure::invalid(phase, "short travel bound"));
                }
                let Some((slot, d_max, min_d)) = self.interval_needs_split(&c0, &c1, &min_depths, &tr, arm_dof)? else {
                    self.check_and_advance(&c1, &mut min_depths, phase)?;
                    if let Some(q1) = q1_opt {
                        samples.push(q1);
                    }
                    continue;
                };
                if t1 - t0 <= 1e-9 {
                    return Err(self.describe_failure(slot, -d_max, d_max, min_d, phase));
                }
                if evals >= budget {
                    return Err(self.budget_failure(budget, phase));
                }
                evals += 1;
                let tm = 0.5 * (t0 + t1);
                let qm = at(tm);
                let mut cm = self.alloc_buf();
                self.eval_into(&qm, &mut cm, phase)?;
                self.check_readonly(&cm, &min_depths, phase)?;
                stack.push((tm, cm.clone(), t1, c1, q1_opt));
                stack.push((t0, c0, tm, cm, Some(qm)));
            }
            t_prev = t_next;
            c_prev = c_next;
        }
        Ok(samples)
    }
}
