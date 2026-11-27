use crate::kinematics::{self, Mat3, Pose, Vec3};
use crate::model::{CameraMount, WorkspacePolicy};
use crate::motion::MotionError;

use super::{approach_pose, AcquireCameraDown, GraspSetup, CAMERA_DOWN_AUTO_TILT_DEG};

// ---- camera geometry ----

/// Pinhole camera rigidly mounted on end_link.
#[derive(Debug, Clone, PartialEq)]
pub struct CamModel {
    /// R_end_cam (`kinematics::cam_mount`).
    pub rec: Mat3,
    /// Optical centre in end_link (m).
    pub offset: Vec3,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub size: [f64; 2],
}

impl CamModel {
    /// The controller's model: mount flip / rot from the config and tilt from the setup, or the
    /// calibrated `q_end_cam`; intrinsics (fx, fy, principal point, default the image centre)
    /// from the setup; optical centre at `camera_mount.offset_m` (end_link origin when unset).
    /// Lens distortion is not modelled.
    pub fn assumed(arm: &crate::chain::ArmModel, setup: &GraspSetup, mount: &CameraMount) -> Self {
        Self {
            rec: arm.r_end_cam(mount, setup.camera_tilt_deg),
            offset: mount.offset_m.map(|o| o.map(f64::from)).unwrap_or([0.0; 3]),
            fx: setup.fx_px, fy: setup.fy_px,
            cx: setup.cx_px.unwrap_or(setup.image_size_px[0] / 2.0),
            cy: setup.cy_px.unwrap_or(setup.image_size_px[1] / 2.0),
            size: setup.image_size_px,
        }
    }

    pub fn origin(&self, pose: &Pose) -> Vec3 { pose.transform_point(&self.offset) }

    pub fn rotation(&self, pose: &Pose) -> Mat3 { kinematics::cam_rotation(pose, &self.rec) }

    /// Base-frame point where the ray through pixel `px` meets the plane z = `plane_z`.
    pub fn plane_point(&self, pose: &Pose, px: [f64; 2], plane_z: f64) -> Option<Vec3> {
        let o = self.origin(pose);
        let d = kinematics::mat_vec(&self.rotation(pose), &kinematics::unit(&[(px[0] - self.cx) / self.fx, (px[1] - self.cy) / self.fy, 1.0]));
        if d[2] > -0.05 { return None; }
        let t = (plane_z - o[2]) / d[2];
        if !(t > 0.0 && t < 2.0) { return None; }
        Some([o[0] + d[0] * t, o[1] + d[1] * t, plane_z])
    }

    /// Pixel of a base-frame point; None behind the camera.
    pub fn project(&self, pose: &Pose, p: &Vec3) -> Option<[f64; 2]> {
        let pc = kinematics::mat_vec(&kinematics::transpose(&self.rotation(pose)), &kinematics::sub(p, &self.origin(pose)));
        if pc[2] <= 1e-3 { return None; }
        Some([self.fx * pc[0] / pc[2] + self.cx, self.fy * pc[1] / pc[2] + self.cy])
    }

    pub fn to_px(&self, n: [f64; 2]) -> [f64; 2] { [n[0] * self.size[0], n[1] * self.size[1]] }
    pub fn to_norm(&self, p: [f64; 2]) -> [f64; 2] { [p[0] / self.size[0], p[1] / self.size[1]] }

    /// Base-frame direction per unit of optical depth Z_cam: R_world_cam * [(px[0]-cx)/fx, (px[1]-cy)/fy, 1.0].
    pub fn ray_world_per_z(&self, pose: &Pose, px: [f64; 2]) -> Vec3 {
        let r_world_cam = self.rotation(pose);
        let p_cam = [(px[0] - self.cx) / self.fx, (px[1] - self.cy) / self.fy, 1.0];
        kinematics::mat_vec(&r_world_cam, &p_cam)
    }

    /// Optical depth Z_cam where the ray through pixel `px` meets the plane z = `table_z_m`.
    pub fn table_optical_depth(&self, pose: &Pose, px: [f64; 2], table_z_m: f64) -> Option<f64> {
        let o = self.origin(pose);
        let ray = self.ray_world_per_z(pose, px);
        if ray[2] >= -0.15 { return None; }
        let z_cam = (table_z_m - o[2]) / ray[2];
        if (0.08..=2.5).contains(&z_cam) {
            Some(z_cam)
        } else {
            None
        }
    }
}


/// Rotate `r` minimally so the tool axis (end_link x) points straight down.
pub fn look_down(r: &Mat3) -> Mat3 {
    rotate_down(r, kinematics::column(r, 0), kinematics::column(r, 1))
}

/// Rotate `r` minimally so the base-frame direction `v` points straight down; when `v` points
/// straight up, turn by pi about `flip`.
fn rotate_down(r: &Mat3, v: Vec3, flip: Vec3) -> Mat3 {
    let down = [0.0, 0.0, -1.0];
    let axis = kinematics::cross(&v, &down);
    let (s, c) = (kinematics::norm(&axis), kinematics::dot(&v, &down));
    if s < 1e-9 {
        if c > 0.0 { return *r; }
        return kinematics::mat_mul(&kinematics::axis_r(&flip, std::f64::consts::PI), r);
    }
    kinematics::mat_mul(&kinematics::axis_r(&kinematics::unit(&axis), s.atan2(c)), r)
}

/// Rotate `r` minimally so the optical axis (`rec` column 2, in end_link) points straight
/// down. From a tool-vertical `r` this pitches the tool by the mount tilt about x_cam, so the
/// heading is kept and `look_down` of the result is `r` again.
pub fn camera_down(r: &Mat3, rec: &Mat3) -> Mat3 {
    rotate_down(r, kinematics::mat_vec(r, &kinematics::column(rec, 2)), kinematics::mat_vec(r, &kinematics::column(rec, 0)))
}

/// Whether a grasp with `setup` on `mount` acquires camera-down (`acquire_camera_down`).
/// `on` needs the lens centre (`camera_mount.offset_m`): the camera is placed with it.
pub fn camera_down_mode(setup: &GraspSetup, mount: &CameraMount) -> Result<bool, String> {
    match setup.acquire_camera_down {
        AcquireCameraDown::Off => Ok(false),
        AcquireCameraDown::Auto => Ok(setup.camera_tilt_deg.abs() > CAMERA_DOWN_AUTO_TILT_DEG && mount.offset_m.is_some()),
        AcquireCameraDown::On if mount.offset_m.is_none() =>
            Err("acquire_camera_down \"on\" needs camera_mount.offset_m (the lens centre in end_link)".into()),
        AcquireCameraDown::On => Ok(true),
    }
}

/// Clearance between every envelope point and the table on every grasp sub-motion (m). One
/// bound for every point, the TCP (jaw tips) included: `grasp_height_m` is validated to be at
/// least this, so the tool-vertical jaw tips at the grasp height always satisfy it, and a point
/// below the jaw-tip plane needs `grasp_height_m` to leave this much under it.
pub const TABLE_CLEARANCE_M: f64 = 0.005;

/// End_link points kept above the table on every grasp sub-motion: the TCP, the workspace
/// check points and, when `camera_mount.offset_m` or `body_size_m` is set, the lens centre
/// (`offset_m`, else the end_link origin as [`CamModel::assumed`] takes it) and the 8 corners
/// of the camera body box (`body_size_m` or the default: front face centred on the lens, `rec` axes).
pub fn envelope(ws: Option<&WorkspacePolicy>, mount: &CameraMount, rec: &Mat3) -> Vec<Vec3> {
    let mut pts = vec![[0.0; 3]];
    pts.extend(ws.iter().flat_map(|w| w.check_points_end_m.iter().map(|p| p.map(f64::from))));
    let lens = mount.offset_m.or(mount.body_size_m.map(|_| [0.0; 3]));
    if let Some(o) = lens.map(|o| o.map(f64::from)) {
        pts.push(o);
        let [w, h, d] = mount.body_size().map(f64::from);
        for sx in [-0.5, 0.5] { for sy in [-0.5, 0.5] { for z in [0.0, -d] {
            pts.push(kinematics::add(&o, &kinematics::mat_vec(rec, &[sx * w, sy * h, z])));
        }}}
    }
    pts
}

/// [`envelope`] additionally ensuring any declared `physical_contact_end_m` is included
/// (deduplicated within 1e-6 m) in table-clearance checks.
pub fn envelope_with_contact(
    ws: Option<&WorkspacePolicy>,
    mount: &CameraMount,
    rec: &Mat3,
    physical_contact_end_m: Option<[f64; 3]>,
) -> Vec<Vec3> {
    let mut pts = envelope(ws, mount, rec);
    if let Some(pc) = physical_contact_end_m {
        if !pts.iter().any(|e| {
            (e[0] - pc[0]).abs() <= 1e-6
                && (e[1] - pc[1]).abs() <= 1e-6
                && (e[2] - pc[2]).abs() <= 1e-6
        }) {
            pts.push(pc);
        }
    }
    pts
}

/// Samples of the straight Cartesian path `from` → `to` as `motion::plan_cartesian`
/// interpolates it (linear position, rotation along the geodesic): (s, pose), s in (0, 1].
pub(super) fn path_samples(from: &Pose, to: &Pose) -> Vec<(f64, Pose)> {
    let dp = kinematics::sub(&to.p, &from.p);
    let w = kinematics::log_v(&kinematics::mul_t(&to.r, &from.r));
    let n = (kinematics::norm(&dp) / 0.005).max(kinematics::norm(&w) / 0.02).ceil().clamp(1.0, 2000.0) as usize;
    (1..=n).map(|k| {
        let s = k as f64 / n as f64;
        let r = if k == n { to.r } else { kinematics::mat_mul(&kinematics::exp_r(&w.map(|v| v * s)), &from.r) };
        (s, Pose::new(r, kinematics::add(&from.p, &dp.map(|v| v * s))))
    }).collect()
}

/// First sample of the path `from` → `to` where an envelope point is below
/// `table_z + TABLE_CLEARANCE_M`, as a workspace violation (s, point index, z, bound). Each
/// point is checked against its own floor: a point that starts below the bound may move as
/// long as it does not go lower than its start height; a point that starts above it must stay
/// above the bound.
pub fn table_violation(from: &Pose, to: &Pose, pts: &[Vec3], table_z: f64) -> Option<MotionError> {
    let bound = table_z + TABLE_CLEARANCE_M;
    let floor: Vec<f64> = pts.iter().map(|p| from.transform_point(p)[2].min(bound)).collect();
    path_samples(from, to).into_iter().find_map(|(s, pose)| {
        pts.iter().zip(&floor).enumerate()
            .map(|(i, (p, f))| (i, pose.transform_point(p)[2], *f))
            .filter(|&(_, z, f)| z < f - 1e-6)
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(point, z, _)| MotionError::WorkspaceViolation {
                s,
                point,
                limit: "table_z_m + 0.005".into(),
                value: z,
                bound,
                detail: Box::new(kinematics::WorkspaceViolationDetail {
                    constraint_id: Some("table_z".into()),
                    geometry_id: Some(if point == 0 { "end_origin".into() } else { format!("end_point:{point}") }),
                    violation_phase: Some("plan".into()),
                    reason: Some(format!("new intrusion below table clearance {bound:.3}m at z={z:.3}m")),
                }),
            })
    })
}

/// TCP height of the camera-down acquisition at orientation `r_cd`: `table_z_m +
/// approach_height_m`, raised when needed so the envelope stays `TABLE_CLEARANCE_M` above the
/// table at `r_cd` (approach end, centring) and along the transition to the tool-vertical
/// pose at approach height (height linear, rotation along the geodesic).
pub fn acquisition_z(r_cd: &Mat3, setup: &GraspSetup, pts: &[Vec3]) -> f64 {
    let z_end = setup.table_z_m + setup.approach_height_m;
    let bound = setup.table_z_m + TABLE_CLEARANCE_M;
    let w = kinematics::log_v(&kinematics::mul_t(&look_down(r_cd), r_cd));
    // at s the TCP is at z0 (1 − s) + z_end s and must be at least bound − lowest(s), with
    // lowest(s) the lowest envelope point relative to the TCP (<= 0, the TCP counts)
    (0..50).map(|k| {
        let s = k as f64 / 50.0;
        let r = kinematics::mat_mul(&kinematics::exp_r(&w.map(|v| v * s)), r_cd);
        let lowest = pts.iter().map(|p| kinematics::mat_vec(&r, p)[2]).fold(0.0, f64::min);
        (bound - lowest - z_end * s) / (1.0 - s)
    }).fold(z_end, f64::max)
}

/// Camera-down approach pose: optical axis straight down ([`camera_down`] of the
/// tool-vertical orientation, heading kept), optical centre above `target`, TCP at
/// [`acquisition_z`] (`table_z_m + approach_height_m` unless the envelope `pts` needs more).
pub fn camera_down_pose(r_now: &Mat3, target: &Vec3, setup: &GraspSetup, cam: &CamModel, pts: &[Vec3]) -> Pose {
    let r = camera_down(&look_down(r_now), &cam.rec);
    let off = kinematics::mat_vec(&r, &cam.offset);
    Pose::new(r, [target[0] - off[0], target[1] - off[1], acquisition_z(&r, setup, pts)])
}

/// Camera-down → tool-vertical transition from the centred `pose`: the ray through pixel `px`
/// meets the plane z = `plane` (grasp plane, or the object's top from the height probe) at the
/// target; the goal is the tool-vertical pose with the TCP above it at approach height. None
/// when the ray misses the plane.
pub fn transition_pose(pose: &Pose, px: [f64; 2], plane: f64, setup: &GraspSetup, cam: &CamModel) -> Option<(Vec3, Pose)> {
    let t = cam.plane_point(pose, px, plane)?;
    Some((t, approach_pose(&pose.r, &t, setup)))
}

