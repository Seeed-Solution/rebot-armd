//! URDF forward kinematics and 6-joint damped-least-squares IK for the reBot DevArm
//! (assets/reBot-DevArm_gripper.urdf, base_link -> joint1..joint6 -> end_joint -> end_link).
//!
//! A faithful f64 port of the cockpit's `VIK` block (src/api/index.html, `/* VIEW-IK-BEGIN */`)
//! and the `CALM` rotation helpers it uses; `tests/test_kinematics.rs` cross-checks it against
//! values the JS produces (`tests/fixtures/vik_samples.json`).
//!
//! Conventions
//! - Joint angles are URDF model angles J1..J6 (rad). armd treats motor angle = model angle;
//!   the cockpit's `?iksign` / `?ikoff` URL overrides are not applied here.
//! - A [`Pose`] is T_base_end_link (CALM.fk convention, end_link x = tool axis).
//! - Camera frame is OpenCV (x right, y down, z optical axis = end_link x at zero tilt).
use std::sync::LazyLock;

use crate::gravity::{mat3_mul, mat3_vec3_mul, AXES, ROT, TRANS};

pub type Mat3 = [[f64; 3]; 3];
pub type Vec3 = [f64; 3];

pub const IDENTITY: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// end_joint origin (URDF xyz 0 0 0.15539).
pub const END_TRANS: Vec3 = [0.0, 0.0, 0.15539];
/// end_joint origin rpy (URDF 0 -1.5708 3.1415), fixed-axis R = Rz(y) Ry(p) Rx(r).
// The URDF values are rounded; keeping them as written keeps FK equal to the URDF.
#[allow(clippy::approx_constant)]
pub const END_RPY: Vec3 = [0.0, -1.5708, 3.1415];
static END_ROT: LazyLock<Mat3> = LazyLock::new(|| rpy_to_r(END_RPY[0], END_RPY[1], END_RPY[2]));

/// J2 axis in the arm plane at J1 = 0 as (radius, height) in m (cockpit `IK` S0); the
/// workspace reach is measured from this axis, as `IK.reach` does.
pub const SHOULDER_RZ: [f64; 2] = [0.0200, 0.1402];

/// DLS parameters, identical to `VIK.PAR`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IkParams {
    pub w: f64,
    pub lam_min: f64,
    pub lam_max: f64,
    pub iters: usize,
    pub tol_pos: f64,
    pub tol_rot: f64,
    pub max_step: f64,
}

pub const PAR: IkParams = IkParams {
    w: 0.2,
    lam_min: 0.01,
    lam_max: 0.05,
    iters: 10,
    tol_pos: 1e-3,
    tol_rot: 0.5 * std::f64::consts::PI / 180.0,
    max_step: 0.5,
};

/// T_base_end_link.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    pub r: Mat3,
    pub p: Vec3,
}

impl Pose {
    pub fn new(r: Mat3, p: Vec3) -> Self { Self { r, p } }
    /// Point given in end_link coordinates, expressed in base_link.
    pub fn transform_point(&self, v: &Vec3) -> Vec3 { add(&self.p, &mat3_vec3_mul(&self.r, v)) }
}

/// End pose plus world joint axes and origins of J1..J6 (`VIK.frames`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Frames {
    pub end: Pose,
    pub axes: [Vec3; 6],
    pub origins: [Vec3; 6],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IkResult {
    /// Within `tol_pos` / `tol_rot` of the goal.
    pub ok: bool,
    pub q: [f64; 6],
    /// Position error (m).
    pub ep: f64,
    /// Rotation error (rad).
    pub er: f64,
    pub iterations: usize,
}

// ---- small vector / matrix helpers (same arithmetic order as the JS) ----

#[inline]
pub fn add(a: &Vec3, b: &Vec3) -> Vec3 { [a[0] + b[0], a[1] + b[1], a[2] + b[2]] }
#[inline]
pub fn sub(a: &Vec3, b: &Vec3) -> Vec3 { [a[0] - b[0], a[1] - b[1], a[2] - b[2]] }
#[inline]
pub fn cross(a: &Vec3, b: &Vec3) -> Vec3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
#[inline]
pub fn dot(a: &Vec3, b: &Vec3) -> f64 { a[0] * b[0] + a[1] * b[1] + a[2] * b[2] }
#[inline]
pub fn norm(a: &Vec3) -> f64 { hypot3(a[0], a[1], a[2]) }
#[inline]
fn hypot3(x: f64, y: f64, z: f64) -> f64 { (x * x + y * y + z * z).sqrt() }
/// Unit vector; a zero vector is returned unchanged (JS `n || 1`).
pub fn unit(a: &Vec3) -> Vec3 {
    let n = norm(a);
    let n = if n == 0.0 || n.is_nan() { 1.0 } else { n };
    [a[0] / n, a[1] / n, a[2] / n]
}
pub fn mat_mul(a: &Mat3, b: &Mat3) -> Mat3 { mat3_mul(a, b) }
pub fn mat_vec(m: &Mat3, v: &Vec3) -> Vec3 { mat3_vec3_mul(m, v) }
pub fn transpose(m: &Mat3) -> Mat3 {
    [[m[0][0], m[1][0], m[2][0]], [m[0][1], m[1][1], m[2][1]], [m[0][2], m[1][2], m[2][2]]]
}
/// A * Bᵀ (`VIK.mtR`).
pub fn mul_t(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut r = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            r[i][j] = a[i][0] * b[j][0] + a[i][1] * b[j][1] + a[i][2] * b[j][2];
        }
    }
    r
}
pub fn column(m: &Mat3, c: usize) -> Vec3 { [m[0][c], m[1][c], m[2][c]] }
pub fn from_columns(x: &Vec3, y: &Vec3, z: &Vec3) -> Mat3 {
    [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]]
}

/// Rodrigues rotation about a unit axis (`CALM.axisR`; the axis is not normalised).
pub fn axis_r(k: &Vec3, a: f64) -> Mat3 {
    let (c, s) = (a.cos(), a.sin());
    let cc = 1.0 - c;
    let [x, y, z] = *k;
    [
        [c + x * x * cc, x * y * cc - z * s, x * z * cc + y * s],
        [y * x * cc + z * s, c + y * y * cc, y * z * cc - x * s],
        [z * x * cc - y * s, z * y * cc + x * s, c + z * z * cc],
    ]
}

/// URDF fixed-axis rpy: R = Rz(yaw) Ry(pitch) Rx(roll), radians (`CALM.rpyR`).
pub fn rpy_to_r(r: f64, p: f64, y: f64) -> Mat3 {
    let (cr, sr, cp, sp, cy, sy) = (r.cos(), r.sin(), p.cos(), p.sin(), y.cos(), y.sin());
    [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
}

/// Inverse of [`rpy_to_r`], radians [roll, pitch, yaw] (`CALM.rpyDeg` without the degree factor).
pub fn r_to_rpy(r: &Mat3) -> Vec3 {
    [r[2][1].atan2(r[2][2]), (-r[2][0]).clamp(-1.0, 1.0).asin(), r[1][0].atan2(r[0][0])]
}

/// Rotation matrix -> unit quaternion [w, x, y, z] with w >= 0 (Shepperd, `CALM.quat`).
pub fn r_to_quat(r: &Mat3) -> [f64; 4] {
    let s0 = r[0][0] + r[1][1] + r[2][2];
    let q = if s0 > 0.0 {
        let s = (s0 + 1.0).sqrt() * 2.0;
        [s / 4.0, (r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s]
    } else if r[0][0] > r[1][1] && r[0][0] > r[2][2] {
        let s = (1.0 + r[0][0] - r[1][1] - r[2][2]).sqrt() * 2.0;
        [(r[2][1] - r[1][2]) / s, s / 4.0, (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s]
    } else if r[1][1] > r[2][2] {
        let s = (1.0 + r[1][1] - r[0][0] - r[2][2]).sqrt() * 2.0;
        [(r[0][2] - r[2][0]) / s, (r[0][1] + r[1][0]) / s, s / 4.0, (r[1][2] + r[2][1]) / s]
    } else {
        let s = (1.0 + r[2][2] - r[0][0] - r[1][1]).sqrt() * 2.0;
        [(r[1][0] - r[0][1]) / s, (r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s, s / 4.0]
    };
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt() * if q[0] < 0.0 { -1.0 } else { 1.0 };
    [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
}

/// Unit quaternion [w, x, y, z] (normalised here) -> rotation matrix (`CALM.quatR`).
pub fn quat_to_r(q: &[f64; 4]) -> Mat3 {
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    let n = if n == 0.0 || n.is_nan() { 1.0 } else { n };
    let (w, x, y, z) = (q[0] / n, q[1] / n, q[2] / n, q[3] / n);
    [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]
}

/// Rotation matrix -> rotation vector (axis * angle, rad), via the quaternion (`CALM.logR`).
pub fn log_r(r: &Mat3) -> Vec3 {
    let [w, x, y, z] = r_to_quat(r);
    let s = hypot3(x, y, z);
    if s < 1e-12 {
        return [2.0 * x, 2.0 * y, 2.0 * z];
    }
    let th = 2.0 * s.atan2(w);
    [x / s * th, y / s * th, z / s * th]
}

/// Rotation vector -> rotation matrix (`CALM.expR` / `VIK.expv`).
pub fn exp_r(v: &Vec3) -> Mat3 {
    let th = norm(v);
    if th < 1e-15 {
        return IDENTITY;
    }
    axis_r(&[v[0] / th, v[1] / th, v[2] / th], th)
}

/// Rotation vector of R with the fast path away from pi (`VIK.logv`).
pub fn log_v(r: &Mat3) -> Vec3 {
    let c = (r[0][0] + r[1][1] + r[2][2] - 1.0) / 2.0;
    if c > 0.999999 {
        return [(r[2][1] - r[1][2]) / 2.0, (r[0][2] - r[2][0]) / 2.0, (r[1][0] - r[0][1]) / 2.0];
    }
    if c < -0.99 {
        return log_r(r);
    }
    let th = c.acos();
    let k = th / (2.0 * th.sin());
    [(r[2][1] - r[1][2]) * k, (r[0][2] - r[2][0]) * k, (r[1][0] - r[0][1]) * k]
}

/// Rotation angle between two rotations (rad).
pub fn angle_between(a: &Mat3, b: &Mat3) -> f64 { norm(&log_v(&mul_t(b, a))) }

// ---- chain ----

/// End pose and world joint axes / origins for q (model J1..J6), `VIK.frames`.
pub fn frames(q: &[f64; 6]) -> Frames {
    let mut r = IDENTITY;
    let mut p = [0.0; 3];
    let mut axes = [[0.0; 3]; 6];
    let mut origins = [[0.0; 3]; 6];
    for j in 0..6 {
        let (link_p, link_r, s) = (&TRANS[j + 1], &ROT[j + 1], AXES[j + 1][2]);
        let d = mat3_vec3_mul(&r, link_p);
        p = add(&p, &d);
        r = mat3_mul(&r, link_r);
        axes[j] = [r[0][2] * s, r[1][2] * s, r[2][2] * s];
        origins[j] = p;
        let th = s * q[j];
        let (c, sn) = (th.cos(), th.sin());
        r = [
            [r[0][0] * c + r[0][1] * sn, -r[0][0] * sn + r[0][1] * c, r[0][2]],
            [r[1][0] * c + r[1][1] * sn, -r[1][0] * sn + r[1][1] * c, r[1][2]],
            [r[2][0] * c + r[2][1] * sn, -r[2][0] * sn + r[2][1] * c, r[2][2]],
        ];
    }
    let d = mat3_vec3_mul(&r, &END_TRANS);
    p = add(&p, &d);
    r = mat3_mul(&r, &END_ROT);
    Frames { end: Pose { r, p }, axes, origins }
}

pub fn fk(q: &[f64; 6]) -> Pose { frames(q).end }

/// f32 joint vector (at least 6 entries) -> model angles.
pub fn q6(q: &[f32]) -> [f64; 6] {
    let mut out = [0.0; 6];
    for (o, v) in out.iter_mut().zip(q) { *o = f64::from(*v); }
    out
}

struct PoseErr { e: [f64; 6], ep: f64, er: f64 }

fn pose_err(f: &Frames, rg: &Mat3, pg: &Vec3, w: f64) -> PoseErr {
    let dp = sub(pg, &f.end.p);
    let dr = log_v(&mul_t(rg, &f.end.r));
    PoseErr { e: [dp[0], dp[1], dp[2], w * dr[0], w * dr[1], w * dr[2]], ep: norm(&dp), er: norm(&dr) }
}

/// 6x6 SPD solve (Cholesky), row-major; None when not positive definite (`chol6`).
fn chol6(a: &[f64; 36], b: &[f64; 6]) -> Option<[f64; 6]> {
    let mut l = [0.0f64; 36];
    for i in 0..6 {
        for j in 0..=i {
            let mut s = a[i * 6 + j];
            for k in 0..j { s -= l[i * 6 + k] * l[j * 6 + k]; }
            if i == j {
                // Also rejects NaN.
                #[allow(clippy::neg_cmp_op_on_partial_ord)]
                if !(s > 1e-18) { return None; }
                l[i * 6 + i] = s.sqrt();
            } else {
                l[i * 6 + j] = s / l[j * 6 + j];
            }
        }
    }
    let mut y = [0.0f64; 6];
    for i in 0..6 {
        let mut s = b[i];
        for k in 0..i { s -= l[i * 6 + k] * y[k]; }
        y[i] = s / l[i * 6 + i];
    }
    for i in (0..6).rev() {
        let mut s = y[i];
        for k in (i + 1)..6 { s -= l[k * 6 + i] * y[k]; }
        y[i] = s / l[i * 6 + i];
    }
    Some(y)
}

/// Damped-least-squares IK (`VIK.solve`, same PAR). `lo` / `hi` are model limits, widened to
/// contain `q0` so a joint already past its margin may stay but not go further. Never panics
/// on bad targets; returns `ok: false` instead.
pub fn solve_dls(goal: &Pose, q0: &[f64; 6], lo: &[f64; 6], hi: &[f64; 6], w: f64) -> IkResult {
    solve_dls_with(goal, q0, lo, hi, w, PAR.iters)
}

pub fn solve_dls_with(goal: &Pose, q0: &[f64; 6], lo: &[f64; 6], hi: &[f64; 6], w: f64, iters: usize) -> IkResult {
    let mut lo_w = [0.0; 6];
    let mut hi_w = [0.0; 6];
    for i in 0..6 {
        lo_w[i] = lo[i].min(q0[i]);
        hi_w[i] = hi[i].max(q0[i]);
    }
    let (rg, pg) = (&goal.r, &goal.p);
    let mut q = *q0;
    let mut f = frames(&q);
    let mut e = pose_err(&f, rg, pg, w);
    let mut lam = PAR.lam_min;
    let mut it = 0;
    if !(e.ep < f64::INFINITY && e.er < f64::INFINITY) {
        return IkResult { ok: false, q: *q0, ep: f64::INFINITY, er: f64::INFINITY, iterations: it };
    }
    while it < iters && !(e.ep <= PAR.tol_pos / 10.0 && e.er <= PAR.tol_rot / 10.0) {
        it += 1;
        // geometric Jacobian, angular rows scaled by w (column per joint)
        let mut jac = [[0.0f64; 6]; 6];
        for j in 0..6 {
            let a = f.axes[j];
            let v = cross(&a, &sub(&f.end.p, &f.origins[j]));
            jac[j] = [v[0], v[1], v[2], w * a[0], w * a[1], w * a[2]];
        }
        let mut m = [0.0f64; 36];
        for r in 0..6 {
            for c in r..6 {
                let mut s = 0.0;
                for col in &jac { s += col[r] * col[c]; }
                m[r * 6 + c] = s;
                m[c * 6 + r] = s;
            }
        }
        for r in 0..6 { m[r * 6 + r] += lam * lam; }
        let Some(y) = chol6(&m, &e.e) else { break; };
        let mut dq = [0.0f64; 6];
        for (j, col) in jac.iter().enumerate() {
            let mut s = 0.0;
            for r in 0..6 { s += col[r] * y[r]; }
            dq[j] = s;
        }
        let big = dq.iter().fold(0.0f64, |m, v| m.max(v.abs()));
        let k = if big > PAR.max_step { PAR.max_step / big } else { 1.0 };
        let mut qn = [0.0; 6];
        for i in 0..6 { qn[i] = hi_w[i].min(lo_w[i].max(q[i] + k * dq[i])); }
        let fn_ = frames(&qn);
        let en = pose_err(&fn_, rg, pg, w);
        if en.ep + w * en.er < e.ep + w * e.er {
            q = qn; f = fn_; e = en; lam = PAR.lam_min;
        } else if lam < PAR.lam_max {
            lam = PAR.lam_max; // adaptive damping: retry the step with heavier damping
        } else {
            q = qn; f = fn_; e = en; // still worse with lam_max: take it (clamped) and continue
        }
    }
    IkResult { ok: e.ep <= PAR.tol_pos && e.er <= PAR.tol_rot, q, ep: e.ep, er: e.er, iterations: it }
}

// ---- camera ----

/// Camera mount R_end_cam (`VIK.camMount`, then · Rx(tilt)): z_cam = end_link x; y_cam = flip ·
/// J5 axis seen in end_link at q6 = pi/2 (x part removed); x_cam = y_cam × z_cam; then `rot_deg`
/// about the optical axis. `tilt_deg` (new, not in the cockpit) rotates about the resulting x_cam
/// (right-handed: positive tilt turns the optical axis toward -y_cam, i.e. up in the image).
pub fn cam_mount(flip: bool, rot_deg: f64, tilt_deg: f64) -> Mat3 {
    let f = frames(&[0.0, 0.0, 0.0, 0.0, 0.0, std::f64::consts::PI / 2.0]);
    let a_end = mat3_vec3_mul(&transpose(&f.end.r), &f.axes[4]);
    let sgn = if flip { -1.0 } else { 1.0 };
    let y = unit(&[0.0, a_end[1] * sgn, a_end[2] * sgn]);
    let z = [1.0, 0.0, 0.0];
    let x = cross(&y, &z);
    let mut r = from_columns(&x, &y, &z);
    if rot_deg != 0.0 {
        r = mat3_mul(&r, &axis_r(&[0.0, 0.0, 1.0], rot_deg.to_radians()));
    }
    if tilt_deg != 0.0 {
        r = mat3_mul(&r, &axis_r(&[1.0, 0.0, 0.0], tilt_deg.to_radians()));
    }
    r
}

/// Camera rotation in the base frame: R_base_cam = R_base_end · R_end_cam.
pub fn cam_rotation(pose: &Pose, rec: &Mat3) -> Mat3 { mat3_mul(&pose.r, rec) }

/// Auto-level (`VIK.level`): roll about the optical axis so x_cam is horizontal and y_cam
/// points down; skipped when the optical axis is within acos(level_cos) of vertical.
pub fn level(pose: &Pose, rec: &Mat3, level_cos: f64) -> Pose {
    let rc = cam_rotation(pose, rec);
    let z = column(&rc, 2);
    if z[2].abs() >= level_cos {
        return *pose;
    }
    let x = unit(&cross(&z, &[0.0, 0.0, 1.0]));
    let y = cross(&z, &x);
    let rc = from_columns(&x, &y, &z);
    Pose { r: mul_t(&rc, rec), p: pose.p }
}

/// Distance from the shoulder (J2) axis to a base-frame point, as cockpit `IK.reach`.
pub fn shoulder_reach(p: &Vec3) -> f64 {
    let r = (p[0] * p[0] + p[1] * p[1]).sqrt();
    ((r - SHOULDER_RZ[0]).powi(2) + (p[2] - SHOULDER_RZ[1]).powi(2)).sqrt()
}

// ---- Cartesian workspace check ----

/// The largest single excess of a workspace check.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ViolationDetail {
    /// 0 = end_link origin, 1..=n = `check_points_end_m[i - 1]`, n + 1 = camera (`offset_m`).
    pub point: usize,
    /// `floor`, `box_x_min`, `box_x_max`, ..., `box_z_max` or `reach`.
    pub limit: &'static str,
    /// Coordinate (m) of the point along the violated limit (reach: distance to the shoulder axis;
    /// keepout: depth inside the box, bound 0).
    pub value: f64,
    pub bound: f64,
    /// Index into `keepout_boxes` when `limit` is `keepout`.
    pub keepout: Option<usize>,
}

impl ViolationDetail {
    /// `limit`, with the box name for a keep-out box (`keepout 'name'`).
    pub fn label(&self, ws: &crate::model::WorkspacePolicy) -> String {
        match self.keepout {
            Some(i) => format!("keepout '{}'", ws.keepout_boxes.get(i).map_or("?", |b| b.name.as_str())),
            None => self.limit.to_string(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Violation {
    /// Sum of the positive excesses over all points and limits (m).
    pub total: f64,
    pub worst: Option<ViolationDetail>,
}

/// Tolerance used by the workspace rule (m).
pub const WORKSPACE_EPS: f64 = 1e-4;

const BOX_LIMITS: [[&str; 2]; 3] = [["box_x_min", "box_x_max"], ["box_y_min", "box_y_max"], ["box_z_min", "box_z_max"]];

/// Workspace excess for joint positions `q` (at least J1..J6, model = motor angles).
pub fn workspace_violation(q: &[f32], ws: &crate::model::WorkspacePolicy, cam: &crate::model::CameraMount) -> Violation {
    if q.len() < 6 || q[..6].iter().any(|v| !v.is_finite()) {
        return Violation { total: f64::INFINITY, worst: Some(ViolationDetail { point: 0, limit: "invalid", value: f64::NAN, bound: 0.0, keepout: None }) };
    }
    workspace_violation_pose(&fk(&q6(q)), ws, cam)
}

pub fn workspace_violation_pose(pose: &Pose, ws: &crate::model::WorkspacePolicy, cam: &crate::model::CameraMount) -> Violation {
    let mut v = Violation::default();
    let mut note = |excess: f64, point: usize, limit: &'static str, value: f64, bound: f64, keepout: Option<usize>| {
        if excess > 0.0 {
            v.total += excess;
            if v.worst.is_none_or(|w| excess > (w.value - w.bound).abs()) {
                v.worst = Some(ViolationDetail { point, limit, value, bound, keepout });
            }
        }
    };
    let floor = f64::from(ws.floor_z_m);
    let lo = ws.box_min_m.map(f64::from);
    let hi = ws.box_max_m.map(f64::from);
    let n = ws.check_points_end_m.len();
    let points = std::iter::once((0, [0.0f32; 3]))
        .chain(ws.check_points_end_m.iter().enumerate().map(|(i, p)| (i + 1, *p)))
        .chain(cam.offset_m.map(|o| (n + 1, o)));
    for (idx, local) in points {
        let p = pose.transform_point(&local.map(f64::from));
        note(floor - p[2], idx, "floor", p[2], floor, None);
        for k in 0..3 {
            note(lo[k] - p[k], idx, BOX_LIMITS[k][0], p[k], lo[k], None);
            note(p[k] - hi[k], idx, BOX_LIMITS[k][1], p[k], hi[k], None);
        }
        if idx == 0 {
            if let Some(reach) = ws.max_reach_m {
                let d = shoulder_reach(&p);
                note(d - f64::from(reach), 0, "reach", d, f64::from(reach), None);
            }
        }
        // Keep-out box: excess = depth inside (distance to the nearest face).
        for (b, kb) in ws.keepout_boxes.iter().enumerate() {
            let depth = (0..3).map(|k| (p[k] - f64::from(kb.min_m[k])).min(f64::from(kb.max_m[k]) - p[k]))
                .fold(f64::INFINITY, f64::min);
            note(depth, idx, "keepout", depth, 0.0, Some(b));
        }
    }
    v
}

/// The workspace rule: a target is rejected when it violates and is worse than the start.
/// A move from an already-violating pose that does not make the violation worse is allowed.
pub fn workspace_rejects(target: &Violation, start: &Violation) -> bool {
    target.total > WORKSPACE_EPS && target.total > start.total + WORKSPACE_EPS
}
