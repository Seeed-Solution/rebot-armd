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
fn path_samples(from: &Pose, to: &Pose) -> Vec<(f64, Pose)> {
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


// Private-seam unit tests (`path_samples`, `approach_pose`, `grasp_plane`); move to tests/grasp/ once S6 adds `feature = "testing"`.
#[cfg(test)]
mod tests {
    use super::*;
    use crate::grasp::{grasp_plane, transition_estimate};
    use crate::motion;

    fn v8() -> (GraspSetup, CameraMount, CamModel) {
        let setup = GraspSetup { camera_tilt_deg: -38.0, ..Default::default() };
        let mount = CameraMount { rot_deg: 0.0, tilt_deg: -38.0, offset_m: Some([-0.0731, 0.0005, 0.066]), ..Default::default() };
        let cam = CamModel::assumed(&crate::chain::ArmModel::REBOT_DEV_ARM, &setup, &mount);
        (setup, mount, cam)
    }

    fn inspect_r() -> Mat3 { crate::chain::ArmModel::REBOT_DEV_ARM.fk(&[0.0, -1.167, -1.565, 1.435, 0.0, 0.0]).r }

    fn close(a: &Vec3, b: &Vec3, tol: f64) -> bool { kinematics::norm(&kinematics::sub(a, b)) < tol }

    #[test]
    fn acquisition_height_keeps_the_pitched_envelope_clear() {
        // rot 90, tilt -38: pitch about end_link z, the axis along which the kit check points
        // are 92.5 mm out
        let setup = GraspSetup { camera_tilt_deg: -38.0, table_z_m: 0.10, approach_height_m: 0.035, ..Default::default() };
        let mount = CameraMount { rot_deg: 90.0, tilt_deg: -38.0, offset_m: Some([-0.0731, 0.0005, 0.066]), ..Default::default() };
        let cam = CamModel::assumed(&crate::chain::ArmModel::REBOT_DEV_ARM, &setup, &mount);
        let pts = envelope(crate::model::ArmConfig::rebot_dm().workspace.as_ref(), &mount, &cam.rec);
        assert_eq!(pts.len(), 18, "TCP, 8 check points, lens centre, 8 body corners");
        let target = [0.28, 0.0, setup.table_z_m + setup.grasp_height_m];
        let low = camera_down_pose(&inspect_r(), &target, &setup, &cam, &[]);
        assert!((low.p[2] - 0.135).abs() < 1e-12, "TCP only: approach height");
        let bound = setup.table_z_m + TABLE_CLEARANCE_M;
        let min_z = |pose: &Pose| pts.iter().map(|p| pose.transform_point(p)[2]).fold(f64::INFINITY, f64::min);
        assert!(min_z(&low) < setup.table_z_m - 0.02, "unraised: {:.4}", min_z(&low));
        let pose = camera_down_pose(&inspect_r(), &target, &setup, &cam, &pts);
        assert!(pose.p[2] > low.p[2] + 0.02 && (min_z(&pose) - bound).abs() < 1e-3, "raised to the bound: {:.4}", min_z(&pose));
        let tool = approach_pose(&pose.r, &target, &setup);
        assert!(path_samples(&pose, &tool).iter().all(|(_, p)| min_z(p) >= bound - 1e-3), "transition clear");
        assert!(table_violation(&pose, &tool, &pts, setup.table_z_m).is_none());
        let from = Pose::new(pose.r, [pose.p[0], pose.p[1], pose.p[2] + 0.1]);
        assert!(table_violation(&from, &low, &pts, setup.table_z_m).is_some(), "a path into the table is reported");
        // a path from below the bound may move sideways but not lower
        let side = Pose::new(low.r, [low.p[0] + 0.01, low.p[1], low.p[2]]);
        assert!(table_violation(&low, &side, &pts, setup.table_z_m).is_none());
    }

    #[test]
    fn below_clearance_exception_is_per_point() {
        // two points 0.1 m either side of the TCP along x, TCP 4 mm above the table. A roll
        // about y from -5.73 to +5.16 deg swaps which point is lowest: -x starts lowest at
        // -6 mm (below the table), +x starts at 14 mm and ends at -5 mm, through the table but
        // above the lowest start height. Only -x may sit below the bound, +x must stay above it.
        let table_z = 0.0;
        let pts = [[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [-0.1, 0.0, 0.0]];
        let roll = |deg: f64| kinematics::exp_r(&[0.0, deg.to_radians(), 0.0]);
        let a = (0.01f64 / 0.1).asin().to_degrees();
        let from = Pose::new(roll(-a), [0.0, 0.0, 0.004]);
        let to = Pose::new(roll(0.9 * a), [0.0, 0.0, 0.004]);
        let z = |pose: &Pose, i: usize| pose.transform_point(&pts[i])[2];
        let bound = table_z + TABLE_CLEARANCE_M;
        assert!(z(&from, 2) < table_z && z(&from, 1) > bound, "-x starts lowest, below the table; +x above the bound");
        assert!(z(&to, 1) < table_z && z(&to, 1) > z(&from, 2) + 1e-4, "+x ends in the table, above the lowest start");
        assert!(path_samples(&from, &to).iter().all(|(_, p)| (0..3).all(|i| z(p, i) >= z(&from, 2) - 1e-6)),
            "no sample goes below the lowest start height, so a single-minimum check accepts the path");
        match table_violation(&from, &to, &pts, table_z) {
            Some(MotionError::WorkspaceViolation { point, value, .. }) => assert!(point == 1 && value < bound, "{point} {value}"),
            other => panic!("expected a violation of point 1, got {other:?}"),
        }
        // the lowest point may stay where it started
        assert!(table_violation(&from, &from, &pts, table_z).is_none());
    }

    #[test]
    fn transition_goes_tool_vertical_above_the_centred_target() {
        let (setup, _, cam) = v8();
        let r_now = inspect_r();
        let target = [0.30, 0.03, setup.table_z_m + setup.grasp_height_m];
        let pose = camera_down_pose(&r_now, &target, &setup, &cam, &[]);
        let (t, goal) = transition_pose(&pose, [cam.cx, cam.cy], grasp_plane(&setup), &setup, &cam).unwrap();
        assert!(close(&t, &target, 1e-9), "{t:?}");
        assert!(close(&goal.p, &[target[0], target[1], setup.table_z_m + setup.approach_height_m], 1e-9), "{:?}", goal.p);
        assert!(kinematics::column(&goal.r, 0)[2] < -1.0 + 1e-12, "tool axis down");
        assert!(kinematics::angle_between(&goal.r, &look_down(&r_now)) < 1e-9, "heading of the approach kept");
        // a box centre off the image centre maps to its own table point
        let off_px = [cam.cx + 30.0, cam.cy - 20.0];
        let (t2, _) = transition_pose(&pose, off_px, grasp_plane(&setup), &setup, &cam).unwrap();
        let px = cam.project(&pose, &t2).unwrap();
        assert!((px[0] - off_px[0]).abs() < 1e-6 && (px[1] - off_px[1]).abs() < 1e-6);
        // expected duration of the move: rotation of 38 deg at 0.5 rad/s dominates here
        let est = transition_estimate(&pose, &target, &setup);
        let rot = 1.1 * 1.875 * 38f64.to_radians() / motion::DEFAULT_ANGULAR_RPS;
        assert!(est >= rot - 1e-9 && est < rot + 3.0, "{est} vs {rot}");
    }
}
