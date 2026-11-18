use std::borrow::Cow;
use serde::{Deserialize, Serialize};
use crate::safety::SafetyState;
/// Hardware enable state confirmed by fresh telemetry, not by a successful write.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum EnableState {
    Enabled,
    Disabled,
    #[default]
    Unknown,
}

/// Telemetry and command features implemented by a driver in its active mode.
/// A false feedback flag means the numeric placeholder is not a measurement.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct DriverCapabilities {
    pub velocity_feedback: bool,
    pub torque_feedback: bool,
    pub temperature_feedback: bool,
    pub torque_feedforward: bool,
    pub enable_feedback: bool,
    #[serde(default)]
    pub load_feedback: bool,
}

/// Outcome of a mapping-range check.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RangeCheckStatus {
    /// Every motor answered and its ranges equal the configured ones.
    Passed,
    /// At least one motor reports a different range: arming is refused.
    Mismatch,
    /// At least one motor did not answer: the configured ranges are used unverified.
    Unverified,
}

/// Registers read from one motor by the range check.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointRangeReading {
    /// 1-based joint number.
    pub joint: usize,
    pub motor_id: u16,
    pub status: RangeCheckStatus,
    /// CTRL_MODE register (1 = MIT, 2 = POS_VEL); None without a reply.
    pub ctrl_mode: Option<u32>,
    /// PMAX / VMAX / TMAX read from the motor; None without a reply.
    pub reported: Option<[f32; 3]>,
    /// PMAX / VMAX / TMAX the driver decodes and encodes this joint with.
    pub configured: [f32; 3],
}

/// Startup check of the motors' mapping ranges (published as `driver_range_check`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DriverRangeCheck {
    pub status: RangeCheckStatus,
    pub message: String,
    pub joints: Vec<JointRangeReading>,
    /// Time the check took (ms).
    pub duration_ms: u64,
}



/// Limits for a single joint
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct JointLimit {
    pub name: String,
    pub min_position: f32,
    pub max_position: f32,
    pub max_velocity: f32,
    pub max_acceleration: f32,
}

impl JointLimit {
    pub fn new(
        name: impl Into<String>,
        min_position: f32,
        max_position: f32,
        max_velocity: f32,
        max_acceleration: f32,
    ) -> Self {
        Self {
            name: name.into(),
            min_position,
            max_position,
            max_velocity,
            max_acceleration,
        }
    }
}

/// Dynamic state of a single joint
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct JointState {
    pub position: f32,
    pub velocity: f32,
    pub torque: f32,
    pub temperature: f32,
}

impl Default for JointState {
    fn default() -> Self {
        Self {
            position: 0.0,
            velocity: 0.0,
            torque: 0.0,
            temperature: 25.0,
        }
    }
}

/// Hardware-specific evidence required before automatic torque release.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RestingPosePolicy {
    /// Optional resting joint target (rad, length `dof`) used by `park` / `NormalUnload` instead of `home_positions`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub positions: Option<Vec<f32>>,
    pub position_tolerances: Vec<f32>,
    pub max_total_deviation: f32,
    /// |velocity| limit (rad/s) for every joint without an entry in `joint_max_velocities`.
    pub max_velocity: f32,
    /// Per-joint |velocity| limits (rad/s), joint 1 first; joints past the end use
    /// `max_velocity`.
    #[serde(default)]
    pub joint_max_velocities: Vec<f32>,
    pub support_torque_limits: Vec<(usize, f32)>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub support_load_limits: Vec<(usize, f32)>,
    pub feedback_max_age_ms: u64,
}

impl RestingPosePolicy {
    /// The velocity and support-torque limits were tuned on readings decoded with the old
    /// single range (VMAX 30, TMAX 18). In real units (J1-J3 VMAX 10, TMAX 28) the same
    /// physical limits are 0.015 / 3 rad/s on J1-J3 and 1.5 / 4.5 N·m × 28 / 18 on J2 / J3.
    pub fn rebot_dm() -> Self {
        Self {
            positions: None,
            position_tolerances: vec![0.15, 0.08, 0.08, 0.08, 0.08, 0.08],
            max_total_deviation: 0.22,
            max_velocity: 0.015,
            joint_max_velocities: vec![0.005, 0.005, 0.005, 0.015, 0.015, 0.015, 0.015],
            support_torque_limits: vec![(1, 2.333), (2, 7.0)],
            support_load_limits: Vec::new(),
            feedback_max_age_ms: 100,
        }
    }

    /// |velocity| limit for joint `i` (0-based).
    pub fn max_velocity_of(&self, i: usize) -> f32 {
        self.joint_max_velocities.get(i).copied().unwrap_or(self.max_velocity)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FollowingErrorPolicy {
    pub max_error_rad: f32,
    pub persistence_ms: u64,
}
impl Default for FollowingErrorPolicy {
    fn default() -> Self { Self { max_error_rad: 0.5, persistence_ms: 500 } }
}

/// Gravity model: the built-in B601-DM model or the inertials of the profile URDF
/// (`urdf` + `chain`, src/chain.rs).
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum GravityModel { RebotDevArm, Urdf }

/// Kinematic model used for the Cartesian workspace check and Cartesian actions: the
/// built-in B601-DM model or the profile URDF chain (`urdf` + `chain`, src/chain.rs).
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum KinematicsModel { RebotDevArm, Urdf }

/// Server-side Cartesian safety box, checked on the end_link pose (src/kinematics.rs).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct WorkspacePolicy {
    /// No check point may go below this height (m, base frame).
    pub floor_z_m: f32,
    pub box_min_m: [f32; 3],
    pub box_max_m: [f32; 3],
    /// Maximum distance from the shoulder (J2) axis to the end_link origin (m).
    #[serde(default)]
    pub max_reach_m: Option<f32>,
    /// Points in end_link coordinates (m) that must stay inside the box and above the floor.
    pub check_points_end_m: Vec<[f32; 3]>,
    /// Boxes (base frame) no check point may enter. A runtime list
    /// (`/api/v1/workspace/keepout`) replaces this one.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub keepout_boxes: Vec<KeepoutBox>,
}

/// Most keep-out boxes a profile or the runtime list may hold.
pub const MAX_KEEPOUT_BOXES: usize = 16;

/// Axis-aligned keep-out box in the base frame (m).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct KeepoutBox {
    pub name: String,
    pub min_m: [f32; 3],
    pub max_m: [f32; 3],
}

impl KeepoutBox {
    /// Validate a box list: at most [`MAX_KEEPOUT_BOXES`], names 1..=64 bytes, finite, min < max per axis.
    pub fn validate_list(boxes: &[KeepoutBox]) -> Result<(), String> {
        if boxes.len() > MAX_KEEPOUT_BOXES {
            return Err(format!("at most {MAX_KEEPOUT_BOXES} keep-out boxes"));
        }
        for b in boxes {
            if b.name.is_empty() || b.name.len() > 64 {
                return Err("keep-out box name must be 1..64 bytes".into());
            }
            if !b.min_m.iter().chain(&b.max_m).all(|v| v.is_finite())
                || b.min_m.iter().zip(&b.max_m).any(|(lo, hi)| lo >= hi) {
                return Err(format!("keep-out box '{}': finite values and min_m < max_m on every axis", b.name));
            }
        }
        Ok(())
    }
}

impl WorkspacePolicy {
    pub fn rebot_dm() -> Self {
        // Corners of the URDF gripper envelope in end_link: X[-0.160, 0], Y[+-0.0925], Z[+-0.0347].
        let mut corners = Vec::with_capacity(8);
        for x in [-0.160f32, 0.0] {
            for y in [-0.0925f32, 0.0925] {
                for z in [-0.0347f32, 0.0347] { corners.push([x, y, z]); }
            }
        }
        Self {
            floor_z_m: 0.0,
            box_min_m: [-0.60, -0.60, -1.0],
            box_max_m: [0.60, 0.60, 0.80],
            max_reach_m: Some(0.52),
            check_points_end_m: corners,
            keepout_boxes: Vec::new(),
        }
    }

    pub fn validate(&self) -> Result<(), String> {
        let finite3 = |v: &[f32; 3]| v.iter().all(|x| x.is_finite());
        if !self.floor_z_m.is_finite() || !finite3(&self.box_min_m) || !finite3(&self.box_max_m)
            || self.box_min_m.iter().zip(&self.box_max_m).any(|(lo, hi)| lo >= hi)
            || self.max_reach_m.is_some_and(|r| !r.is_finite() || r <= 0.0)
            || self.check_points_end_m.is_empty() || self.check_points_end_m.len() > 16
            || !self.check_points_end_m.iter().all(finite3) {
            return Err("Invalid workspace policy: finite values, box_min < box_max, 1..16 check points".into());
        }
        KeepoutBox::validate_list(&self.keepout_boxes)
    }
}

/// Camera mount relative to end_link (see `kinematics::cam_mount`). Defaults match the
/// cockpit defaults (`CFG.view.camFlip = false`, `camRotDeg = 0`); `tilt_deg` 0 means the
/// optical axis is end_link x.
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct CameraMount {
    pub flip: bool,
    pub rot_deg: f32,
    pub tilt_deg: f32,
    /// Camera position in end_link (m). When set it is added to the workspace check points.
    /// No default: the real position needs measuring.
    pub offset_m: Option<[f32; 3]>,
    /// Camera body (housing and holder) as a box [width along x_cam, height along y_cam,
    /// depth behind the lens along −z_cam] (m), lens at the centre of its front face. Kept
    /// above the table on every grasp sub-motion; None = [`CAMERA_BODY_DEFAULT_M`].
    #[serde(skip_serializing_if = "Option::is_none")]
    pub body_size_m: Option<[f32; 3]>,
    /// Calibrated R_end_cam as a unit quaternion [w, x, y, z] (camera calibration wizard).
    /// When set it replaces `flip` / `rot_deg` / `tilt_deg` for every consumer;
    /// `tilt_deg` is then kept as the derived tilt ([`crate::kinematics::derived_tilt_deg`]).
    /// Must be unit within [`Q_UNIT_TOL`]; it is normalised when loaded.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub q_end_cam: Option<[f64; 4]>,
}

/// Tolerance on |q| - 1 of `camera_mount.q_end_cam`.
pub const Q_UNIT_TOL: f64 = 1e-3;

/// `q` normalised, or why it is not a unit quaternion within [`Q_UNIT_TOL`].
pub fn unit_quaternion(q: &[f64; 4]) -> Result<[f64; 4], String> {
    let n = q.iter().map(|v| v * v).sum::<f64>().sqrt();
    if !q.iter().all(|v| v.is_finite()) || (n - 1.0).abs() > Q_UNIT_TOL {
        return Err(format!("camera_mount.q_end_cam must be a unit quaternion [w, x, y, z] (|q| = 1 within {Q_UNIT_TOL})"));
    }
    Ok(q.map(|v| v / n))
}

/// reCamera housing 40 x 40 x 36.5 mm in the v8 holder (reBot-DevArm
/// hardware/camera-mounts/recamera-mounts/v8/generate_recamera_mount.py: 0.3 mm clearance,
/// 3 mm walls, 7 mm ears each side, 3 mm support below and top bar above, 3.5 mm rear lip):
/// 54.6 x 46.6 x 40 mm, rounded up.
pub const CAMERA_BODY_DEFAULT_M: [f32; 3] = [0.055, 0.047, 0.040];

impl CameraMount {
    /// The body box size in use.
    pub fn body_size(&self) -> [f32; 3] { self.body_size_m.unwrap_or(CAMERA_BODY_DEFAULT_M) }

    /// Check `q_end_cam` and normalise it in place.
    pub fn normalise_q(&mut self) -> Result<(), String> {
        if let Some(q) = &self.q_end_cam { self.q_end_cam = Some(unit_quaternion(q)?); }
        Ok(())
    }
}

/// Mapping between J7 angle (rad) and jaw opening (m).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct JawOpening {
    /// [[j7_rad, opening_m], ...]
    pub points: Vec<[f32; 2]>,
    pub calibrated: bool,
}

impl JawOpening {
    pub const SIM_DEFAULT_POINTS: [[f32; 2]; 2] = [[0.0, 0.0], [-5.7, 0.085]];

    pub fn sim_default() -> Self {
        Self {
            points: Self::SIM_DEFAULT_POINTS.to_vec(),
            calibrated: false,
        }
    }

    /// Validate: piecewise-linear, monotonic, >= 2 points, inside J7 limits, openings in [0, 0.2] m.
    pub fn validate(&self, j7_limit: Option<&JointLimit>) -> Result<(), String> {
        if self.points.len() < 2 {
            return Err("jaw_opening requires at least 2 points".into());
        }
        for (i, p) in self.points.iter().enumerate() {
            let [j7, w] = *p;
            if !j7.is_finite() || !w.is_finite() {
                return Err(format!("jaw_opening point {i} values must be finite"));
            }
            if !(0.0..=0.2).contains(&w) {
                return Err(format!("jaw_opening opening {w} must be in [0, 0.2] m"));
            }
            if let Some(lim) = j7_limit {
                if j7 < lim.min_position - 1e-4 || j7 > lim.max_position + 1e-4 {
                    return Err(format!("jaw_opening j7 {j7} outside J7 limits [{}, {}]", lim.min_position, lim.max_position));
                }
            }
        }
        let d_j7_0 = self.points[1][0] - self.points[0][0];
        let d_w_0 = self.points[1][1] - self.points[0][1];
        if d_j7_0.abs() < 1e-6 || d_w_0.abs() < 1e-6 {
            return Err("jaw_opening points must be strictly monotonic".into());
        }
        for w in self.points.windows(2) {
            let dj = w[1][0] - w[0][0];
            let dw = w[1][1] - w[0][1];
            if dj.abs() < 1e-6 || dw.abs() < 1e-6 {
                return Err("jaw_opening points must be strictly monotonic".into());
            }
            if (dj > 0.0) != (d_j7_0 > 0.0) || (dw > 0.0) != (d_w_0 > 0.0) {
                return Err("jaw_opening points must be strictly monotonic".into());
            }
        }
        Ok(())
    }

    /// Piecewise linear lookup of jaw opening (m) for joint J7 (rad).
    pub fn opening_m(&self, j7_rad: f32) -> f32 {
        if self.points.is_empty() { return 0.0; }
        if self.points.len() == 1 { return self.points[0][1]; }
        let p_first = self.points[0];
        let p_last = self.points[self.points.len() - 1];
        let dir = p_last[0] - p_first[0];
        if dir > 0.0 {
            if j7_rad <= p_first[0] { return p_first[1]; }
            if j7_rad >= p_last[0] { return p_last[1]; }
        } else {
            if j7_rad >= p_first[0] { return p_first[1]; }
            if j7_rad <= p_last[0] { return p_last[1]; }
        }
        for w in self.points.windows(2) {
            let (p0, p1) = (w[0], w[1]);
            let span = p1[0] - p0[0];
            if span.abs() > 1e-6 {
                let t = (j7_rad - p0[0]) / span;
                if (0.0..=1.0).contains(&t) {
                    return p0[1] + t * (p1[1] - p0[1]);
                }
            }
        }
        p_last[1]
    }

    /// Inverse lookup: target J7 (rad) for a given jaw opening (m).
    pub fn j7_for(&self, opening: f32) -> f32 {
        if self.points.is_empty() { return 0.0; }
        if self.points.len() == 1 { return self.points[0][0]; }
        let p_first = self.points[0];
        let p_last = self.points[self.points.len() - 1];
        let dir = p_last[1] - p_first[1];
        if dir > 0.0 {
            if opening <= p_first[1] { return p_first[0]; }
            if opening >= p_last[1] { return p_last[0]; }
        } else {
            if opening >= p_first[1] { return p_first[0]; }
            if opening <= p_last[1] { return p_last[0]; }
        }
        for w in self.points.windows(2) {
            let (p0, p1) = (w[0], w[1]);
            let span = p1[1] - p0[1];
            if span.abs() > 1e-6 {
                let t = (opening - p0[1]) / span;
                if (0.0..=1.0).contains(&t) {
                    return p0[0] + t * (p1[0] - p0[0]);
                }
            }
        }
        p_last[0]
    }
}

fn default_true() -> bool {
    true
}

fn is_true(b: &bool) -> bool {
    *b
}

fn is_default_holding_mode(m: &HoldingMode) -> bool {
    matches!(m, HoldingMode::TorqueAndGap)
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HoldingMode {
    #[default]
    TorqueAndGap,
    PositionGap,
    LoadAndGap,
}

pub type HoldingDetection = HoldingMode;

/// Gripper action parameters (`/api/v1/action/gripper`).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct GripperPolicy {
    /// J7 at 0 % open (rad).
    pub closed_rad: f32,
    /// J7 at 100 % open (rad).
    pub open_rad: f32,
    /// A grasp holds an object when the jaw stopped at least this far from closed (rad) ...
    pub holding_gap_rad: f32,
    /// ... and the median |tau7| over the squeeze window is at least this (N·m).
    pub holding_torque_nm: f32,
    /// Position offset past the contact point that produces the grip force under POS_VEL (rad).
    pub squeeze_rad: f32,
    /// Squeeze window before the holding verdict (ms).
    pub settle_ms: u64,
    /// Angle-to-opening calibration mapping; absent by default (uses built-in sim model).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub jaw_opening: Option<JawOpening>,
    #[serde(default = "default_true", skip_serializing_if = "is_true")]
    pub zero_check_supported: bool,
    #[serde(default, skip_serializing_if = "is_default_holding_mode")]
    pub holding_mode: HoldingMode,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub holding_load_fraction: Option<f32>,
}

impl GripperPolicy {
    /// Thresholds from the MIT-mode project; holding torque and squeeze need verification in POS_VEL.
    /// holding_torque_nm 0.0667 is the former 0.12 read with TMAX 18, converted to the
    /// J7 motor's real TMAX 10 (0.12 × 10 / 18).
    pub fn rebot_dm() -> Self {
        Self {
            closed_rad: 0.0,
            open_rad: -5.7,
            holding_gap_rad: 0.30,
            holding_torque_nm: 0.0667,
            squeeze_rad: 0.15,
            settle_ms: 300,
            jaw_opening: None,
            zero_check_supported: true,
            holding_mode: HoldingMode::TorqueAndGap,
            holding_load_fraction: None,
        }
    }

    /// Jaw opening model (or the built-in uncalibrated sim default).
    pub fn jaw_opening(&self) -> Cow<'_, JawOpening> {
        match &self.jaw_opening {
            Some(jo) => Cow::Borrowed(jo),
            None => Cow::Owned(JawOpening::sim_default()),
        }
    }

    pub fn opening_m(&self, j7_rad: f32) -> f32 {
        self.jaw_opening().opening_m(j7_rad)
    }

    pub fn j7_for(&self, opening: f32) -> f32 {
        self.jaw_opening().j7_for(opening)
    }

    pub fn is_calibrated(&self) -> bool {
        self.jaw_opening.as_ref().is_some_and(|jo| jo.calibrated)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn jaw_opening_interpolation_and_clamping() {
        let jo = JawOpening::sim_default();
        assert!((jo.opening_m(0.0) - 0.0).abs() < 1e-6);
        assert!((jo.opening_m(-5.7) - 0.085).abs() < 1e-6);
        assert!((jo.opening_m(-2.85) - 0.0425).abs() < 1e-6);

        // Clamping outside bounds
        assert!((jo.opening_m(1.0) - 0.0).abs() < 1e-6);
        assert!((jo.opening_m(-10.0) - 0.085).abs() < 1e-6);

        // Inverse lookup
        assert!((jo.j7_for(0.0) - 0.0).abs() < 1e-6);
        assert!((jo.j7_for(0.085) - (-5.7)).abs() < 1e-6);
        let j7_55 = jo.j7_for(0.055);
        let expected_j7_55 = -5.7 * (0.055 / 0.085);
        assert!((j7_55 - expected_j7_55).abs() < 1e-5, "got {j7_55}, expected {expected_j7_55}");

        // Inverse clamping
        assert!((jo.j7_for(-0.1) - 0.0).abs() < 1e-6);
        assert!((jo.j7_for(0.2) - (-5.7)).abs() < 1e-6);

        // Multi-point piecewise linear
        let multi = JawOpening {
            points: vec![[0.0, 0.0], [-2.0, 0.030], [-5.7, 0.085]],
            calibrated: true,
        };
        assert!((multi.opening_m(-1.0) - 0.015).abs() < 1e-5);
        assert!((multi.j7_for(0.015) - (-1.0)).abs() < 1e-5);
        assert!((multi.opening_m(-3.85) - 0.0575).abs() < 1e-5);
    }

    #[test]
    fn jaw_opening_validation() {
        let lim = JointLimit::new("j7", -6.0, 6.0, 5.0, 10.0);

        // Default passes
        assert!(JawOpening::sim_default().validate(Some(&lim)).is_ok());

        // Less than 2 points fails
        let one_point = JawOpening { points: vec![[0.0, 0.0]], calibrated: false };
        assert!(one_point.validate(Some(&lim)).is_err());

        // Non-finite values fail
        let non_fin = JawOpening { points: vec![[0.0, 0.0], [f32::NAN, 0.085]], calibrated: false };
        assert!(non_fin.validate(Some(&lim)).is_err());

        // Opening out of [0, 0.2] fails
        let neg_w = JawOpening { points: vec![[0.0, -0.01], [-5.7, 0.085]], calibrated: false };
        assert!(neg_w.validate(Some(&lim)).is_err());
        let big_w = JawOpening { points: vec![[0.0, 0.0], [-5.7, 0.25]], calibrated: false };
        assert!(big_w.validate(Some(&lim)).is_err());

        // Outside J7 limits fails
        let out_j7 = JawOpening { points: vec![[0.0, 0.0], [-7.0, 0.085]], calibrated: false };
        assert!(out_j7.validate(Some(&lim)).is_err());

        // Non-monotonic fails
        let non_mono_j7 = JawOpening {
            points: vec![[0.0, 0.0], [-3.0, 0.040], [-2.0, 0.085]],
            calibrated: false,
        };
        assert!(non_mono_j7.validate(Some(&lim)).is_err());

        let non_mono_w = JawOpening {
            points: vec![[0.0, 0.0], [-2.0, 0.050], [-5.7, 0.030]],
            calibrated: false,
        };
        assert!(non_mono_w.validate(Some(&lim)).is_err());
    }
}

/// Gravity compensation ("hand-guiding") parameters. J1-J6 run in MIT mode with kp = 0,
/// damping `damping_kd` and the URDF gravity torque times `scale` as feed-forward; the
/// gripper stays position-controlled. Values are software defaults for the B601-DM and
/// have not been verified on hardware. Arrays have one entry per arm joint (J1-J6).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct GravityCompPolicy {
    /// Time over which the model feed-forward is faded in (and the hold stiffness out), s.
    pub ramp_in_s: f32,
    /// Default multiplier on the model gravity torque.
    pub scale: f32,
    /// Allowed range for a per-session `scale` override.
    pub scale_range: [f32; 2],
    /// Joint damping while floating (N·m·s/rad).
    pub damping_kd: Vec<f32>,
    /// Upper bound for a per-session damping override (N·m·s/rad).
    pub damping_kd_max: f32,
    /// Per-joint clamp on the feed-forward torque (N·m).
    pub torque_limits_nm: Vec<f32>,
    /// A joint faster than this (rad/s) ...
    pub max_speed_rad_s: f32,
    /// Optional per-joint overspeed limits (rad/s) for J1..J6; empty uses `max_speed_rad_s` for all.
    /// The light wrist joints move faster than 1.5 rad/s under normal hand guiding.
    #[serde(default)]
    pub joint_max_speed_rad_s: Vec<f32>,
    /// Optional per-joint limits (rad/s) for J1..J6 that apply instead of the limits above
    /// while a joint moves against the model gravity torque (lifted by hand) or carries less
    /// than [`GRAVITY_DIRECTION_MIN_NM`] of it. Empty: the limits above apply in every
    /// direction. A joint moving with gravity (falling) always uses the limits above.
    #[serde(default)]
    pub lift_max_speed_rad_s: Vec<f32>,
    /// ... for this many consecutive ticks ends the mode with GRAVITY_COMP_OVERSPEED.
    pub overspeed_ticks: u32,
    /// MIT stiffness used while entering (faded out) and while exiting (N·m/rad).
    pub hold_kp: Vec<f32>,
    /// MIT damping used with `hold_kp` (N·m·s/rad).
    pub hold_kd: Vec<f32>,
    /// MIT stiff hold at the captured pose before switching back to POS_VEL (ms).
    pub exit_hold_ms: u64,
    /// Entry requires every arm joint slower than this (rad/s).
    pub entry_max_velocity: f32,
    /// Feedback older than this ends the mode (ms).
    pub feedback_max_age_ms: u64,
    /// The mode ends when a joint comes this close to its soft limit (rad).
    pub joint_limit_margin_rad: f32,
    /// During the ramp-in the arm should stay where it was: a joint moving further than
    /// this from the entry pose ends the mode with GRAVITY_COMP_SAG (rad).
    pub ramp_max_deviation_rad: f32,
}

/// A joint whose model gravity torque is below this (N·m) has no falling direction.
pub const GRAVITY_DIRECTION_MIN_NM: f32 = 0.3;

impl GravityCompPolicy {
    /// Overspeed limit for arm joint `i` (0-based, J1..J6).
    pub fn speed_limit(&self, i: usize) -> f32 {
        self.joint_max_speed_rad_s.get(i).copied().unwrap_or(self.max_speed_rad_s)
    }

    /// Overspeed limit for arm joint `i` moving at `velocity` (signed, rad/s) with the model
    /// gravity torque `gravity_nm` (the torque that holds the joint; `None` when unknown).
    pub fn directional_speed_limit(&self, i: usize, velocity: f32, gravity_nm: Option<f32>) -> f32 {
        let (Some(&lift), Some(g)) = (self.lift_max_speed_rad_s.get(i), gravity_nm) else {
            return self.speed_limit(i);
        };
        // Falling: the holding torque opposes the motion.
        let falling = g.abs() >= GRAVITY_DIRECTION_MIN_NM && velocity * g < 0.0;
        if falling { self.speed_limit(i) } else { lift.max(self.speed_limit(i)) }
    }
}

impl GravityCompPolicy {
    /// B601-DM defaults. The torque limits sit above the model torque over the usual
    /// workspace (URDF maxima on a joint grid: J2 16.3, J3 7.8, J4 2.2, J5 0.8 N·m), so
    /// J2 is not fully supported when the arm is stretched out horizontally.
    pub fn rebot_dm() -> Self {
        Self {
            ramp_in_s: 1.0,
            scale: 1.0,
            scale_range: [0.5, 1.2],
            damping_kd: vec![0.4, 0.6, 0.5, 0.35, 0.15, 0.1],
            damping_kd_max: 2.0,
            torque_limits_nm: vec![2.0, 12.0, 8.5, 3.0, 1.5, 0.5],
            max_speed_rad_s: 1.5,
            // measured 2026-11-09: a hand flick of the wrist reached 2.5 rad/s on J4
            joint_max_speed_rad_s: vec![1.5, 1.5, 1.5, 3.0, 3.0, 3.0],
            // measured 2026-11-12: lifting the arm by hand reached 1.59 and 2.16 rad/s on J3
            lift_max_speed_rad_s: vec![3.0, 3.0, 3.0, 3.0, 3.0, 3.0],
            overspeed_ticks: 3,
            hold_kp: vec![30.0, 30.0, 30.0, 15.0, 10.0, 8.0],
            hold_kd: vec![1.0, 1.0, 1.0, 0.5, 0.3, 0.2],
            exit_hold_ms: 100,
            entry_max_velocity: 0.05,
            feedback_max_age_ms: 100,
            joint_limit_margin_rad: 0.05,
            ramp_max_deviation_rad: 0.15,
        }
    }

    fn validate(&self) -> Result<(), String> {
        let six = |v: &Vec<f32>| v.len() == 6 && v.iter().all(|x| x.is_finite() && *x >= 0.0);
        let pos = |x: f32| x.is_finite() && x > 0.0;
        let [lo, hi] = self.scale_range;
        if !pos(self.ramp_in_s) || self.ramp_in_s > 10.0
            || !(lo.is_finite() && hi.is_finite() && lo > 0.0 && lo <= hi && hi <= 2.0)
            || !(lo..=hi).contains(&self.scale)
            || !six(&self.damping_kd) || !pos(self.damping_kd_max) || self.damping_kd_max > 5.0
            || self.damping_kd.iter().any(|k| *k > self.damping_kd_max)
            || !six(&self.torque_limits_nm) || self.torque_limits_nm.iter().any(|t| *t > 18.0)
            || !pos(self.max_speed_rad_s) || self.overspeed_ticks == 0
            || !(self.joint_max_speed_rad_s.is_empty() || (six(&self.joint_max_speed_rad_s) && self.joint_max_speed_rad_s.iter().all(|v| pos(*v))))
            || !(self.lift_max_speed_rad_s.is_empty() || (six(&self.lift_max_speed_rad_s) && self.lift_max_speed_rad_s.iter().all(|v| pos(*v))))
            || !six(&self.hold_kp) || self.hold_kp.iter().any(|k| *k > 500.0)
            || !six(&self.hold_kd) || self.hold_kd.iter().any(|k| *k > 5.0)
            || self.exit_hold_ms > 2000 || !pos(self.entry_max_velocity) || self.feedback_max_age_ms == 0
            || !self.joint_limit_margin_rad.is_finite() || self.joint_limit_margin_rad < 0.0
            || !pos(self.ramp_max_deviation_rad) {
            return Err("Invalid gravity_comp policy (6 values per array, kp <= 500, kd <= 5, torque <= 18 N·m, scale inside scale_range)".into());
        }
        Ok(())
    }
}

/// Damiao MIT / feedback mapping range of one motor: registers PMAX (21), VMAX (22)
/// and TMAX (23). Feedback is decoded and MIT frames are encoded with these, so they
/// must equal what the motor reports.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct DamiaoRange {
    /// rad
    pub p_max: f32,
    /// rad/s
    pub v_max: f32,
    /// N·m
    pub t_max: f32,
}

impl DamiaoRange {
    /// B601-DM J1-J3, read from the motors 2026-11-09.
    pub const REBOT_DM_J1_J3: Self = Self { p_max: 12.5, v_max: 10.0, t_max: 28.0 };
    /// B601-DM J4-J7 (J7 = gripper), read from the motors 2026-11-09.
    pub const REBOT_DM_J4_J7: Self = Self { p_max: 12.5, v_max: 30.0, t_max: 10.0 };

    #[cfg(not(target_arch = "wasm32"))]
    pub fn limits(&self) -> crate::protocol::damiao::DamiaoLimits {
        crate::protocol::damiao::DamiaoLimits::symmetric(self.p_max, self.v_max, self.t_max)
    }
}

/// Estimated Damiao POS_VEL lag gain: tracking error ~= v / POSVEL_LAG_GAIN rad at
/// v rad/s. Estimate from the 2026-11 tracking-fault analysis; needs verification
/// on hardware (record |commanded - measured| during presets to fit it).
pub const POSVEL_LAG_GAIN: f32 = 1.5; // measured 2026-11-09: J3 trailed 0.458 rad at a 0.7 rad/s jog

/// Version of the arm profile contract (the `ArmConfig` JSON read by `--config`).
/// A profile without `profile_version` is version 1; armd refuses newer versions.
pub const ARM_PROFILE_VERSION: u32 = 1;

fn default_profile_version() -> u32 {
    1
}

/// Default for [`ArmConfig::servo_max_lead_rad`].
pub const DEFAULT_SERVO_MAX_LEAD_RAD: f32 = 0.2;

fn default_servo_max_lead_rad() -> f32 {
    DEFAULT_SERVO_MAX_LEAD_RAD
}

/// Upper bound on planned motion speed (`speed_cap` in the profile). Each entry is
/// applied as min() with the request and with `planned_max_velocity`; absent entries
/// do not cap. Not changeable at runtime.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct SpeedCap {
    /// Cartesian actions: linear speed (m/s).
    pub linear_mps: Option<f32>,
    /// Cartesian actions: angular speed (rad/s).
    pub angular_rps: Option<f32>,
    /// Factor (0, 1] on the planned joint speed cap of every joint except the 7-DOF gripper.
    pub joint_velocity_scale: Option<f32>,
}

impl SpeedCap {
    fn validate(&self) -> Result<(), String> {
        let positive = |v: Option<f32>| v.is_none_or(|x| x.is_finite() && x > 0.0);
        if !positive(self.linear_mps) || !positive(self.angular_rps) || !positive(self.joint_velocity_scale)
            || self.joint_velocity_scale.is_some_and(|x| x > 1.0) {
            return Err("speed_cap: linear_mps and angular_rps positive finite, joint_velocity_scale in (0, 1]".into());
        }
        Ok(())
    }
}

/// Robotic arm hardware/control configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ArmConfig {
    /// Optional profile identifier (e.g. "rebot_b601dm", "rebot_b601rs", "so_arm101").
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub profile_id: Option<String>,
    /// Arm profile contract version; missing in older profiles, which are version 1.
    #[serde(default = "default_profile_version")]
    pub profile_version: u32,
    pub dof: usize,
    pub joint_names: Vec<String>,
    pub limits: Vec<JointLimit>,
    pub home_positions: Vec<f32>,
    pub lease_timeout_ms: u64,
    pub max_temperature: f32,
    pub control_frequency_hz: u32,
    /// None forbids automatic hardware torque release without a calibrated profile.
    #[serde(default)]
    pub resting_pose: Option<RestingPosePolicy>,
    #[serde(default)]
    pub following_error: FollowingErrorPolicy,
    /// Soft jerk preference; braking/position constraints take precedence.
    #[serde(default)]
    pub max_jerk: Option<f32>,
    #[serde(default)]
    pub gravity_model: Option<GravityModel>,
    /// Joint speed cap (rad/s) for planned motion (jog, home, presets, park).
    /// Applies to every joint except the 7-DOF gripper (index 6).
    /// Keeps POS_VEL lag below the following-error bound. None = no extra cap.
    #[serde(default)]
    pub planned_max_velocity: Option<f32>,
    /// Per-joint speed cap (rad/s) for servo streaming, one entry per joint (`null` =
    /// no cap); the servo step uses min(entry, joint limit). The 7-DOF gripper is
    /// always exempt. None keeps the joint limits.
    #[serde(default)]
    pub servo_max_velocity: Option<Vec<Option<f32>>>,
    /// Servo streaming: the commanded position of every joint except the 7-DOF gripper
    /// stays within this distance of the measured position in the direction of travel
    /// (torque on, fresh feedback). Must be below `following_error.max_error_rad`.
    #[serde(default = "default_servo_max_lead_rad")]
    pub servo_max_lead_rad: f32,
    /// Kinematic model; None disables the workspace check and Cartesian actions.
    #[serde(default)]
    pub kinematics: Option<KinematicsModel>,
    /// Cartesian safety box; None disables the workspace check.
    #[serde(default)]
    pub workspace: Option<WorkspacePolicy>,
    #[serde(default)]
    pub camera_mount: CameraMount,
    /// Gripper action parameters; None disables `/api/v1/action/gripper`.
    #[serde(default)]
    pub gripper_policy: Option<GripperPolicy>,
    /// Tap-to-grasp setup; None disables `/api/v1/action/grasp`.
    #[serde(default)]
    #[cfg(not(target_arch = "wasm32"))]
    pub grasp: Option<crate::grasp::GraspSetup>,
    /// Gravity compensation (hand-guiding); None means unsupported.
    #[serde(default)]
    pub gravity_comp: Option<GravityCompPolicy>,
    /// Damiao mapping range per joint (one per DOF). None keeps the single default
    /// range ±12.5 rad, ±30 rad/s, ±18 N·m for every motor.
    #[serde(default)]
    pub damiao_ranges: Option<Vec<DamiaoRange>>,
    /// Speed cap for every planned motion (actions, jog, home, park); None = no cap.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub speed_cap: Option<SpeedCap>,
    /// Collision guard (`src/collision.rs`); None = defaults, guard off.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub collision: Option<crate::collision::CollisionPolicy>,
    /// Torque-saturation guard (`src/torque_guard.rs`); None = off. Needs `damiao_ranges`
    /// (the per-joint TMAX) when enabled.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub torque_guard: Option<crate::torque_guard::TorqueGuardPolicy>,
    /// Supply power budget (`src/power.rs`): supply-current estimate from motor feedback;
    /// None = off.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub power_budget: Option<crate::power::PowerBudget>,
    /// Facts about the installed hardware published by `/api/v1/safety/describe`;
    /// None = no wired emergency stop and the default notes.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub hardware: Option<HardwareInfo>,
    /// URDF file for `kinematics` / `gravity_model` = `Urdf`, relative to the profile file's
    /// directory (or absolute). Read once at startup by [`ArmConfig::load_arm_model`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub urdf: Option<String>,
    /// Expected sha256 (hex) of the `urdf` file; startup fails on a mismatch.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub urdf_sha256: Option<String>,
    /// Arm chain inside `urdf` and its joint mapping (src/chain.rs).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub chain: Option<crate::chain::ChainSpec>,
    /// Built-in model only: motor -> model (URDF) offset per arm joint J1..J6, model angle =
    /// motor angle + offset (rad). Absent = [`crate::kinematics::B601DM_MODEL_OFFSETS_RAD`].
    /// URDF chains set `chain.joints[].offset_rad` instead.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub model_offsets_rad: Option<Vec<f64>>,
    /// Gripper joint index. Absent: the joint after the arm joints when `dof` has one more
    /// joint than the arm model (index 6 for the B601-DM), otherwise none.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gripper_joint: Option<usize>,
    /// Feetech STS3215 bus and joint calibration settings.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[cfg(not(target_arch = "wasm32"))]
    pub feetech: Option<crate::protocol::feetech::FeetechConfig>,
    /// Arm model built from the fields above (not part of the profile JSON).
    #[serde(skip)]
    pub arm_model: crate::chain::ArmModel,
}

/// Profile field `hardware`.
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq)]
pub struct HardwareInfo {
    /// A wired emergency stop that removes motor power is installed.
    #[serde(default)]
    pub physical_estop: bool,
    /// Replaces the default notes when set.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub notes: Option<Vec<String>>,
}

impl Default for ArmConfig {
    #[allow(clippy::approx_constant)] // joint limits use the rounded values from the motor profile
    fn default() -> Self {
        let dof = 6;
        let joint_names = vec![
            "joint_1".to_string(),
            "joint_2".to_string(),
            "joint_3".to_string(),
            "joint_4".to_string(),
            "joint_5".to_string(),
            "joint_6".to_string(),
        ];

        let limits = vec![
            JointLimit::new("joint_1", -3.14159, 3.14159, 2.0, 5.0),
            JointLimit::new("joint_2", -2.0944, 2.0944, 2.0, 5.0),
            JointLimit::new("joint_3", -2.61799, 2.61799, 2.0, 5.0),
            JointLimit::new("joint_4", -3.14159, 3.14159, 3.0, 8.0),
            JointLimit::new("joint_5", -2.0944, 2.0944, 3.0, 8.0),
            JointLimit::new("joint_6", -6.28318, 6.28318, 4.0, 10.0),
        ];

        let home_positions = vec![0.0; dof];

        Self {
            profile_id: None,
            profile_version: ARM_PROFILE_VERSION,
            dof,
            joint_names,
            limits,
            home_positions,
            lease_timeout_ms: 2500,
            max_temperature: 70.0,
            control_frequency_hz: 50,
            resting_pose: None,
            following_error: FollowingErrorPolicy::default(),
            max_jerk: None,
            gravity_model: None,
            planned_max_velocity: None,
            servo_max_velocity: None,
            servo_max_lead_rad: DEFAULT_SERVO_MAX_LEAD_RAD,
            kinematics: None,
            workspace: None,
            camera_mount: CameraMount::default(),
            gripper_policy: None,
            #[cfg(not(target_arch = "wasm32"))] grasp: None,
            gravity_comp: None,
            damiao_ranges: None,
            speed_cap: None,
            collision: None,
            torque_guard: None,
            power_budget: None,
            hardware: None,
            urdf: None,
            urdf_sha256: None,
            chain: None,
            model_offsets_rad: None,
            gripper_joint: None,
            #[cfg(not(target_arch = "wasm32"))] feetech: None,
            arm_model: crate::chain::ArmModel::REBOT_DEV_ARM,
        }
    }
}

impl ArmConfig {
    /// Number of arm joints of the kinematic / gravity model (6 for the built-in model).
    pub fn arm_dof(&self) -> usize { self.arm_model.arm_dof() }

    /// Target joint positions for `NormalUnload` (`park`) and resting-pose checks.
    pub fn resting_positions(&self) -> &[f32] {
        self.resting_pose
            .as_ref()
            .and_then(|p| p.positions.as_deref())
            .unwrap_or(&self.home_positions)
    }

    /// Joints the resting-pose position check must cover: every joint before a trailing
    /// gripper (all joints when the gripper is not the last one or there is none).
    pub fn resting_arm_joints(&self) -> usize {
        match self.gripper_index() {
            Some(g) if g + 1 == self.dof => g,
            _ => self.dof,
        }
    }

    /// Index of the gripper joint, if the profile has one.
    pub fn gripper_index(&self) -> Option<usize> {
        self.gripper_joint.or_else(|| (self.dof > self.arm_dof()).then_some(self.arm_dof()))
    }

    /// The joint layout the Cartesian action planner, gravity compensation and the gripper
    /// actions use: 6 arm joints followed by the gripper at index 6.
    pub fn six_plus_gripper(&self) -> bool {
        self.dof == 7 && self.arm_dof() == 6 && self.gripper_index() == Some(6)
    }

    /// Whether the configuration supports Cartesian actions with a trailing gripper
    /// (5 or 6 arm joints + gripper at index arm_dof).
    pub fn has_cartesian_with_gripper(&self) -> bool {
        matches!(self.arm_dof(), 5 | 6) && self.gripper_index() == Some(self.arm_dof()) && self.dof == self.arm_dof() + 1
    }

    /// Profile uses the URDF chain for kinematics or gravity.
    pub fn uses_urdf(&self) -> bool {
        self.kinematics == Some(KinematicsModel::Urdf) || self.gravity_model == Some(GravityModel::Urdf)
    }

    /// Read a profile JSON file and build its arm model (`urdf` relative to the file).
    /// The result still needs [`Self::validate`].
    #[cfg(not(target_arch = "wasm32"))]
    pub fn from_profile_file(path: impl AsRef<std::path::Path>) -> Result<Self, String> {
        let path = path.as_ref();
        let file = std::fs::File::open(path).map_err(|e| format!("profile {}: {e}", path.display()))?;
        let mut config: Self = serde_json::from_reader(std::io::BufReader::new(file))
            .map_err(|e| format!("profile {}: {e}", path.display()))?;
        config.load_arm_model(path.parent())?;
        Ok(config)
    }

    /// Build [`Self::arm_model`] from `urdf` + `chain` when the profile uses `Urdf`;
    /// `profile_dir` resolves a relative `urdf` path. Call before [`Self::validate`].
    #[cfg(not(target_arch = "wasm32"))]
    pub fn load_arm_model(&mut self, profile_dir: Option<&std::path::Path>) -> Result<(), String> {
        if !self.uses_urdf() {
            if self.urdf.is_some() || self.chain.is_some() {
                return Err("urdf / chain are set but neither kinematics nor gravity_model is \"Urdf\"".into());
            }
            self.arm_model = crate::chain::ArmModel::rebot_dev_arm(self.builtin_model_offsets()?);
            return Ok(());
        }
        let (Some(rel), Some(spec)) = (&self.urdf, &self.chain) else {
            return Err("kinematics / gravity_model \"Urdf\" needs the urdf and chain fields".into());
        };
        if self.model_offsets_rad.is_some() {
            return Err("model_offsets_rad is for the built-in model; a URDF chain sets chain.joints[].offset_rad".into());
        }
        let path = match profile_dir {
            Some(dir) if std::path::Path::new(rel).is_relative() => dir.join(rel),
            _ => std::path::PathBuf::from(rel),
        };
        let bytes = std::fs::read(&path).map_err(|e| format!("urdf {}: {e}", path.display()))?;
        if let Some(expected) = &self.urdf_sha256 {
            let actual = crate::gravity::compute_sha256(&bytes);
            if !actual.eq_ignore_ascii_case(expected) {
                return Err(format!("urdf {}: sha256 {actual} does not match urdf_sha256 {expected}", path.display()));
            }
        }
        let text = std::str::from_utf8(&bytes).map_err(|e| format!("urdf {}: {e}", path.display()))?;
        let chain = crate::chain::KinematicChain::from_urdf_str(text, spec)
            .map_err(|e| format!("urdf {}: {e}", path.display()))?;
        if self.gravity_model == Some(GravityModel::Urdf) && chain.moving_mass() <= 0.0 {
            return Err(format!("urdf {}: gravity_model \"Urdf\" but the chain links have no mass", path.display()));
        }
        self.arm_model = crate::chain::ArmModel::Chain(std::sync::Arc::new(chain));
        Ok(())
    }

    /// Build the arm model without filesystem access. Checks the same model mapping,
    /// optional URDF checksum and mass requirements as native profile loading.
    pub fn init_arm_model_in_memory(&mut self, urdf_xml: Option<&str>) -> Result<(), String> {
        if !self.uses_urdf() {
            if self.urdf.is_some() || self.chain.is_some() {
                return Err("urdf / chain are set but neither kinematics nor gravity_model is \"Urdf\"".into());
            }
            self.arm_model = crate::chain::ArmModel::rebot_dev_arm(self.builtin_model_offsets()?);
            return Ok(());
        }
        let text = urdf_xml.ok_or("URDF XML is required for this profile")?;
        let spec = self.chain.as_ref().ok_or("URDF profile requires chain")?;
        if self.model_offsets_rad.is_some() {
            return Err("model_offsets_rad is for the built-in model; a URDF chain sets chain.joints[].offset_rad".into());
        }
        if let Some(expected) = &self.urdf_sha256 {
            let actual = crate::gravity::compute_sha256(text.as_bytes());
            if !actual.eq_ignore_ascii_case(expected) {
                return Err(format!("urdf: sha256 {actual} does not match urdf_sha256 {expected}"));
            }
        }
        let chain = crate::chain::KinematicChain::from_urdf_str(text, spec)
            .map_err(|e| format!("urdf: {e}"))?;
        if self.gravity_model == Some(GravityModel::Urdf) && chain.moving_mass() <= 0.0 {
            return Err("urdf: gravity_model \"Urdf\" but the chain links have no mass".into());
        }
        self.arm_model = crate::chain::ArmModel::Chain(std::sync::Arc::new(chain));
        Ok(())
    }

    /// `model_offsets_rad`, or the B601-DM default when absent.
    fn builtin_model_offsets(&self) -> Result<[f64; 6], String> {
        match &self.model_offsets_rad {
            None => Ok(crate::kinematics::B601DM_MODEL_OFFSETS_RAD),
            Some(v) if v.len() == 6 && v.iter().all(|o| o.is_finite() && o.abs() <= std::f64::consts::TAU) => {
                Ok(std::array::from_fn(|i| v[i]))
            }
            Some(_) => Err("model_offsets_rad needs 6 finite values in [-2pi, 2pi] (J1..J6)".into()),
        }
    }

    /// Validate configuration invariants and auto-normalize joint names
    pub fn validate(&mut self) -> Result<(), String> {
        if self.profile_version == 0 || self.profile_version > ARM_PROFILE_VERSION {
            return Err(format!(
                "profile_version {} is not supported (this armd reads 1..={ARM_PROFILE_VERSION})",
                self.profile_version
            ));
        }
        if self.camera_mount.body_size_m.is_some_and(|b| !b.iter().all(|v| v.is_finite() && *v > 0.0 && *v <= 0.2)) {
            return Err("camera_mount.body_size_m must be three sizes in (0, 0.2] m".into());
        }
        if self.dof == 0 {
            return Err("DOF cannot be zero".into());
        }
        if self.limits.len() != self.dof {
            return Err(format!("Limits length {} does not match DOF {}", self.limits.len(), self.dof));
        }
        if self.home_positions.len() != self.dof {
            return Err(format!("Home positions length {} does not match DOF {}", self.home_positions.len(), self.dof));
        }
        if self.joint_names.len() != self.dof {
            self.joint_names.resize_with(self.dof, || "joint".to_string());
        }
        if self.control_frequency_hz == 0 || self.control_frequency_hz > 1000
            || !self.max_temperature.is_finite() || self.max_temperature <= 5.0
            || !self.following_error.max_error_rad.is_finite() || self.following_error.max_error_rad <= 0.0
            || self.following_error.persistence_ms == 0
            || self.max_jerk.is_some_and(|j| !j.is_finite() || j <= 0.0) {
            return Err("Invalid control, temperature, following-error or jerk configuration".into());
        }
        if let Some(cap) = self.planned_max_velocity {
            if !cap.is_finite() || cap <= 0.0 {
                return Err("planned_max_velocity must be positive finite".into());
            }
            if cap / POSVEL_LAG_GAIN >= self.following_error.max_error_rad {
                return Err(format!(
                    "planned_max_velocity {cap} rad/s implies an expected lag {:.3} rad at or above the following-error bound {} rad",
                    cap / POSVEL_LAG_GAIN, self.following_error.max_error_rad));
            }
        }
        if let Some(cap) = &self.speed_cap { cap.validate()?; }
        if let Some(c) = &self.collision { c.validate()?; }
        if let Some(t) = &self.torque_guard {
            t.validate()?;
            if t.enabled && self.damiao_ranges.is_none() {
                return Err("torque_guard needs damiao_ranges (the per-joint torque mapping range TMAX)".into());
            }
        }
        if let Some(p) = &self.power_budget { p.validate(self.dof)?; }
        if let Some(caps) = &self.servo_max_velocity {
            if caps.len() != self.dof || caps.iter().flatten().any(|v| !v.is_finite() || *v <= 0.0) {
                return Err(format!("servo_max_velocity needs {} entries, each null or positive finite", self.dof));
            }
        }
        if !self.servo_max_lead_rad.is_finite() || self.servo_max_lead_rad <= 0.0
            || self.servo_max_lead_rad >= self.following_error.max_error_rad {
            return Err(format!(
                "servo_max_lead_rad {} must be positive and below the following-error bound {} rad",
                self.servo_max_lead_rad, self.following_error.max_error_rad));
        }
        let builtin = self.kinematics == Some(KinematicsModel::RebotDevArm) || self.gravity_model == Some(GravityModel::RebotDevArm);
        if builtin && self.uses_urdf() {
            return Err("kinematics and gravity_model must both be RebotDevArm or both Urdf".into());
        }
        if self.uses_urdf() {
            if self.model_offsets_rad.is_some() {
                return Err("model_offsets_rad is for the built-in model; a URDF chain sets chain.joints[].offset_rad".into());
            }
        } else {
            // the built-in model needs no file; keep it in step with the profile offsets (and
            // keep a kinematic correction installed by the calibration)
            let corr = self.arm_model.correction().copied();
            self.arm_model = crate::chain::ArmModel::rebot_dev_arm(self.builtin_model_offsets()?).with_correction(corr.as_ref())?;
        }
        if self.uses_urdf() != self.arm_model.chain().is_some() {
            return Err("kinematics / gravity_model \"Urdf\" needs a loaded URDF chain (ArmConfig::load_arm_model)".into());
        }
        if self.kinematics == Some(KinematicsModel::RebotDevArm) && self.dof != 7 {
            return Err("kinematics requires dof == 7 (6 arm joints + gripper)".into());
        }
        if let Some(g) = self.gripper_joint {
            if g >= self.dof || g < self.arm_dof() {
                return Err(format!("gripper_joint {g} must be an index in {}..{} (after the arm joints)", self.arm_dof(), self.dof));
            }
        }
        if self.uses_urdf() && (self.dof < self.arm_dof() || self.dof > self.arm_dof() + 1) {
            return Err(format!("the URDF chain has {} joints; dof must be {} or {} (with a gripper)", self.arm_dof(), self.arm_dof(), self.arm_dof() + 1));
        }
        if self.kinematics.is_some() || self.gripper_joint.is_some() {
            let valid = (self.gripper_index().is_none() && self.dof == self.arm_dof())
                || (self.gripper_index() == Some(self.arm_dof()) && self.dof == self.arm_dof() + 1);
            if !valid {
                return Err(format!(
                    "Invalid joint layout: arm has {} joints (dof {}), expected either {} arm joints (no gripper) or {} arm joints + gripper at index {}",
                    self.arm_dof(), self.dof, self.arm_dof(), self.arm_dof(), self.arm_dof()
                ));
            }
        }
        if let Some(ws) = &self.workspace { ws.validate()?; }
        let cam = &self.camera_mount;
        if !cam.rot_deg.is_finite() || !cam.tilt_deg.is_finite()
            || cam.offset_m.is_some_and(|o| o.iter().any(|v| !v.is_finite())) {
            return Err("Invalid camera mount".into());
        }
        if let Some(q) = &cam.q_end_cam { unit_quaternion(q)?; }
        if let Some(g) = &self.gripper_policy {
            let lim = self.gripper_index().and_then(|idx| self.limits.get(idx));
            let in_lim = |v: f32| lim.is_some_and(|l| v >= l.min_position && v <= l.max_position);
            if ![g.closed_rad, g.open_rad, g.holding_gap_rad, g.holding_torque_nm, g.squeeze_rad].iter().all(|v| v.is_finite())
                || !in_lim(g.closed_rad) || !in_lim(g.open_rad) || (g.closed_rad - g.open_rad).abs() < 0.1
                || g.holding_gap_rad < 0.0 || g.holding_torque_nm < 0.0
                || !(0.0..=0.5).contains(&g.squeeze_rad) || g.settle_ms == 0 || g.settle_ms > 5000 {
                return Err("Invalid gripper policy (open/closed inside the gripper joint limits, 0 <= squeeze_rad <= 0.5)".into());
            }
            if let Some(frac) = g.holding_load_fraction {
                if !frac.is_finite() || !(0.0..=1.0).contains(&frac) {
                    return Err("Invalid gripper policy: holding_load_fraction must be in 0.0..=1.0".into());
                }
            }
            if g.holding_mode == HoldingMode::LoadAndGap && g.holding_load_fraction.unwrap_or(0.0) <= 0.0 {
                return Err("Invalid gripper policy: LoadAndGap holding mode requires holding_load_fraction > 0.0".into());
            }
            if let Some(jo) = &g.jaw_opening {
                jo.validate(lim).map_err(|e| format!("Invalid gripper policy: {e}"))?;
            }
        }
        if let Some(ranges) = &self.damiao_ranges {
            let ok = |x: f32| x.is_finite() && x > 0.0;
            if ranges.len() != self.dof || !ranges.iter().all(|r| ok(r.p_max) && ok(r.v_max) && ok(r.t_max)) {
                return Err(format!("damiao_ranges needs {} entries with positive finite p_max / v_max / t_max", self.dof));
            }
        }
        if let Some(gc) = &self.gravity_comp {
            if !self.six_plus_gripper() || self.gravity_model.is_none() {
                return Err("gravity_comp requires dof == 7 and a gravity_model".into());
            }
            gc.validate()?;
            if let Some(ranges) = &self.damiao_ranges {
                if let Some(i) = (0..6).find(|&i| gc.torque_limits_nm[i] > ranges[i].t_max) {
                    return Err(format!("gravity_comp torque_limits_nm[{i}] exceeds the joint's TMAX {}", ranges[i].t_max));
                }
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        if let Some(g) = &self.grasp { g.validate().map_err(|e| format!("Invalid grasp setup: {e}"))?; }
        if let Some(policy) = &self.resting_pose {
            if policy.position_tolerances.len() < self.resting_arm_joints() {
                return Err(format!("resting_pose.position_tolerances needs a value for each of the {} arm joints",
                                   self.resting_arm_joints()));
            }
            if policy.position_tolerances.is_empty() || policy.position_tolerances.len() > self.dof
                || policy.position_tolerances.iter().any(|v| !v.is_finite() || *v <= 0.0)
                || !policy.max_total_deviation.is_finite() || policy.max_total_deviation <= 0.0
                || !policy.max_velocity.is_finite() || policy.max_velocity <= 0.0
                || policy.joint_max_velocities.len() > self.dof
                || policy.joint_max_velocities.iter().any(|v| !v.is_finite() || *v <= 0.0)
                || policy.feedback_max_age_ms == 0
                || policy.support_torque_limits.iter().any(|(i, t)| *i >= self.dof || !t.is_finite() || *t < 0.0)
                || policy.support_load_limits.iter().any(|(i, l)| *i >= self.dof || !l.is_finite() || *l < 0.0) {
                return Err("Invalid resting-pose policy".into());
            }
            if let Some(pos) = &policy.positions {
                if pos.len() != self.dof
                    || !pos.iter().zip(&self.limits).all(|(p, l)| p.is_finite() && *p >= l.min_position && *p <= l.max_position)
                {
                    return Err("Invalid resting-pose positions: must match dof, be finite, and stay within joint limits".into());
                }
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        if let Some(ft) = &self.feetech {
            if !(1..=1000).contains(&ft.max_torque_raw) {
                return Err(format!("feetech.max_torque_raw must be 1..=1000, got {}", ft.max_torque_raw));
            }
            if !(1..=1000).contains(&ft.protection_current_raw) {
                return Err(format!("feetech.protection_current_raw must be 1..=1000, got {}", ft.protection_current_raw));
            }
            if let Some(ot) = ft.overload_torque_raw {
                if !(1..=100).contains(&ot) {
                    return Err(format!("feetech.overload_torque_raw must be 1..=100, got {}", ot));
                }
                if ft.joints.is_empty() && self.arm_dof() > 0 && ot < 50 {
                    return Err(format!("feetech.overload_torque_raw must be >= 50 for arm joints, got {}", ot));
                }
            }
            if !ft.joints.is_empty() {
                if ft.joints.len() != self.dof {
                    return Err(format!(
                        "feetech.joints length {} does not match DOF {}",
                        ft.joints.len(), self.dof
                    ));
                }
                let mut ids = std::collections::HashSet::new();
                for (i, j) in ft.joints.iter().enumerate() {
                    if j.id == 0 || j.id >= 0xFE {
                        return Err(format!("Feetech servo ID {} must be in 1..=253", j.id));
                    }
                    if !ids.insert(j.id) {
                        return Err(format!(
                            "Duplicate Feetech servo ID {} at joint index {}",
                            j.id, i
                        ));
                    }
                    if j.zero_step > 4095 {
                        return Err(format!("Feetech joint {} zero_step must be <= 4095, got {}", i, j.zero_step));
                    }
                    if !(-2047..=2047).contains(&j.homing_offset_steps) {
                        return Err(format!("Feetech joint {} homing_offset_steps abs must be <= 2047, got {}", i, j.homing_offset_steps));
                    }
                    if let Some(exp) = j.expected_homing_offset_steps {
                        if !(-2047..=2047).contains(&exp) {
                            return Err(format!("Feetech joint {} expected_homing_offset_steps abs must be <= 2047, got {}", i, exp));
                        }
                    }
                    if let Some(mt) = j.max_torque_raw {
                        if !(1..=1000).contains(&mt) {
                            return Err(format!("Feetech joint {} max_torque_raw must be 1..=1000, got {}", i, mt));
                        }
                    }
                    if let Some(pc) = j.protection_current_raw {
                        if !(1..=1000).contains(&pc) {
                            return Err(format!("Feetech joint {} protection_current_raw must be 1..=1000, got {}", i, pc));
                        }
                    }
                    if let Some(ot) = j.overload_torque_raw.or(ft.overload_torque_raw) {
                        if !(1..=100).contains(&ot) {
                            return Err(format!("Feetech joint {} overload_torque_raw must be 1..=100, got {}", i, ot));
                        }
                        if i < self.arm_dof() && ot < 50 {
                            return Err(format!("Feetech arm joint {} overload_torque_raw must be >= 50, got {}", i, ot));
                        }
                    }
                    let (eff_min, eff_max) = j.effective_range_steps();
                    if eff_min >= eff_max || eff_max > 4095 {
                        return Err(format!(
                            "Invalid Feetech step range [{}, {}] for servo ID {} at joint {}",
                            eff_min, eff_max, j.id, i
                        ));
                    }
                }
            }
        }
        for (i, lim) in self.limits.iter().enumerate() {
            if !lim.min_position.is_finite() || !lim.max_position.is_finite() || lim.min_position >= lim.max_position {
                return Err(format!("Joint {} min position >= max position", i));
            }
            if lim.max_velocity <= 0.0 || !lim.max_velocity.is_finite() || lim.max_acceleration <= 0.0 || !lim.max_acceleration.is_finite() {
                return Err(format!("Joint {} max_velocity must be positive finite", i));
            }
        }
        if self.home_positions.iter().zip(&self.limits).any(|(p, l)| !p.is_finite() || *p < l.min_position || *p > l.max_position) {
            return Err("Home positions must be finite and within joint limits".into());
        }
        Ok(())
    }

    /// Joint limits for servo stepping: `max_velocity` capped at `servo_max_velocity`
    /// per joint, except the 7-DOF gripper (index 6).
    pub fn servo_limits(&self) -> Vec<JointLimit> {
        let mut limits = self.limits.clone();
        if let Some(caps) = &self.servo_max_velocity {
            let gripper_idx = self.gripper_index();
            for (i, (limit, cap)) in limits.iter_mut().zip(caps).enumerate() {
                if let (Some(cap), false) = (cap, Some(i) == gripper_idx) {
                    limit.max_velocity = limit.max_velocity.min(*cap);
                }
            }
        }
        limits
    }

    /// Factory for reBot Arm B601-DM with 6 arm joints + 1 gripper joint (7 DOF total)
    #[allow(clippy::approx_constant)] // joint limits use the rounded values from the motor profile
    pub fn rebot_dm() -> Self {
        let dof = 7;
        let joint_names = vec![
            "joint_1".to_string(),
            "joint_2".to_string(),
            "joint_3".to_string(),
            "joint_4".to_string(),
            "joint_5".to_string(),
            "joint_6".to_string(),
            "gripper".to_string(),
        ];

        let limits = vec![
            JointLimit::new("joint_1", -3.14159, 3.14159, 2.0, 5.0),
            JointLimit::new("joint_2", -2.0944, 2.0944, 2.0, 5.0),
            JointLimit::new("joint_3", -2.61799, 2.61799, 2.0, 5.0),
            JointLimit::new("joint_4", -3.14159, 3.14159, 3.0, 8.0),
            JointLimit::new("joint_5", -2.0944, 2.0944, 3.0, 8.0),
            JointLimit::new("joint_6", -6.28318, 6.28318, 4.0, 10.0),
            // Measured on hardware 2026-11-06: closed +0.03 rad, mechanical full open -5.87 rad.
            JointLimit::new("gripper", -5.75, 0.10, 2.0, 5.0),
        ];

        let home_positions = vec![0.0, 0.0, 0.0, 0.0, 0.0, 1.5707963, 0.0];

        Self {
            profile_id: None,
            profile_version: ARM_PROFILE_VERSION,
            dof,
            joint_names,
            limits,
            home_positions,
            lease_timeout_ms: 2500,
            max_temperature: 70.0,
            control_frequency_hz: 50,
            resting_pose: Some(RestingPosePolicy::rebot_dm()),
            following_error: FollowingErrorPolicy::default(),
            max_jerk: Some(50.0),
            gravity_model: Some(GravityModel::RebotDevArm),
            planned_max_velocity: Some(0.45),
            // J1-J3 measured trackable at 0.45 rad/s; wrist joints follow faster.
            servo_max_velocity: Some(vec![Some(0.45), Some(0.45), Some(0.45), Some(1.5), Some(1.5), Some(1.5), None]),
            servo_max_lead_rad: DEFAULT_SERVO_MAX_LEAD_RAD,
            kinematics: Some(KinematicsModel::RebotDevArm),
            workspace: Some(WorkspacePolicy::rebot_dm()),
            camera_mount: CameraMount::default(),
            gripper_policy: Some(GripperPolicy::rebot_dm()),
            #[cfg(not(target_arch = "wasm32"))] grasp: Some(crate::grasp::GraspSetup::default()),
            gravity_comp: Some(GravityCompPolicy::rebot_dm()),
            damiao_ranges: Some(vec![
                DamiaoRange::REBOT_DM_J1_J3, DamiaoRange::REBOT_DM_J1_J3, DamiaoRange::REBOT_DM_J1_J3,
                DamiaoRange::REBOT_DM_J4_J7, DamiaoRange::REBOT_DM_J4_J7, DamiaoRange::REBOT_DM_J4_J7,
                DamiaoRange::REBOT_DM_J4_J7,
            ]),
            speed_cap: None,
            // rebot-sysid on the real arm (B601-DM, 2026-11-11, ±0.08 rad, one session at two
            // poses): off until validated at more poses (kits/recamera-b601dm/README.md)
            collision: Some(crate::collision::CollisionPolicy {
                enabled: false,
                thresholds_nm: [2.0, 1.19, 2.82, 1.43, 0.87, 0.28],
                // J2 and J3 rest on their supports at the folded pose, and J4 shows ~2.1 N·m
                // residual there (0.4 N·m once lifted), above its 1.43 N·m threshold
                // (real arm, 2026-11-12)
                rest_exempt_joints: Some([false, true, true, true, false, false]),
                ..Default::default()
            }),
            // 60 % of TMAX for 100 ms on J1-J6: 16.8 N·m on J1-J3, 6.0 N·m on J4-J6. Chosen
            // 2026-11-12 after the J4 stall incident for the Mean Well LRS-150-24 in use
            // (24 V 6.5 A, overload hiccup measured at ~8.5 A); 0.85 is the value intended
            // with the recommended LRS-350-24 (14.6 A).
            torque_guard: Some(crate::torque_guard::TorqueGuardPolicy { ratio: 0.60, ..Default::default() }),
            // Mean Well LRS-150-24 in use: 6.5 A rated, overload hiccup measured at ~8.5 A in a
            // teardown. For the recommended LRS-350-24 set supply_rated_a 14.6 and
            // supply_trip_a ~18 (its datasheet overload range is 110-140 % of rated; not
            // measured). Motor parameters are datasheet values (src/power.rs).
            power_budget: Some(crate::power::PowerBudget::rebot_dm()),
            hardware: None,
            urdf: None,
            urdf_sha256: None,
            chain: None,
            model_offsets_rad: Some(crate::kinematics::B601DM_MODEL_OFFSETS_RAD.to_vec()),
            gripper_joint: None,
            #[cfg(not(target_arch = "wasm32"))] feetech: None,
            arm_model: crate::chain::ArmModel::REBOT_DEV_ARM,
        }
    }
}

/// Cartesian TCP pose and orientation of the end link in the base frame.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct TcpState {
    pub xyz_m: [f32; 3],
    pub rpy_rad: [f32; 3],
    pub quat_wxyz: [f32; 4],
}

/// Gripper state observation.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct GripperObs {
    pub joint: usize,
    pub rad: f32,
    pub fraction: f32,
    pub opening_m: Option<f32>,
    pub stalled: bool,
}

/// Full arm telemetry / observation
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ArmObservation {
    pub timestamp: f64,
    pub positions: Vec<f32>,
    pub velocities: Vec<f32>,
    pub torques: Vec<f32>,
    pub temperatures: Vec<f32>,
    pub is_torqued: bool,
    pub safety_state: SafetyState,
    pub epoch_id: u64,
    #[serde(default)]
    pub gravity_compensated: bool,
    #[serde(default)]
    pub gravity_torques: Vec<f32>,
    #[serde(default)]
    pub driver_capabilities: DriverCapabilities,
    #[serde(default)]
    pub enable_state: EnableState,
    #[serde(default)]
    pub feedback_age_ms: Vec<Option<u64>>,
    #[serde(default)]
    pub disable_pending: bool,
    #[serde(default)]
    pub feedback_valid: bool,
    /// Reference positions sent to the motors this tick (rad).
    #[serde(default)]
    pub commanded_positions: Vec<f32>,
    /// Most recent action (move_to, move_relative, gripper, joint_move, home, park).
    #[serde(default)]
    pub action: Option<crate::motion::ActionStatus>,
    /// The action a newer one replaced, published for 2 s after the replacement.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub previous_action: Option<crate::motion::ActionStatus>,
    /// `position` (POS_VEL hold / motion) or `gravity_comp` (hand-guiding, any phase).
    #[serde(default)]
    pub control_mode: ControlMode,
    #[serde(default)]
    pub gravity_comp: GravityCompObservation,
    /// Startup check of the motors' mapping ranges (Damiao); null when the driver has
    /// no such check or it has not run.
    #[serde(default)]
    pub driver_range_check: Option<DriverRangeCheck>,
    /// `client_id` label of the client holding the control lease (empty when it sent
    /// none); null when no lease is held.
    #[serde(default)]
    pub lease_holder: Option<String>,
    /// Limit that is shaping servo motion right now (speed / lead cap, joint limit,
    /// workspace, blocked hold); null when none applies. See docs/control_safety_contract.md.
    #[serde(default)]
    pub motion_limit: Option<MotionLimit>,
    /// Per joint: the motor's reported fault code, or null. Empty when the driver
    /// reports no motor state (Damiao state codes 8-14).
    #[serde(default)]
    pub motor_errors: Vec<Option<MotorError>>,
    /// Structured cause while the safety state is Fault or EStop; null otherwise.
    #[serde(default)]
    pub fault: Option<crate::safety::FaultInfo>,
    /// Measured torque minus model gravity torque for J1..J6 (N·m); null without a
    /// gravity model or valid torque feedback. Observation only, no fault is raised from it.
    #[serde(default)]
    pub residual_torques: Option<[f32; 6]>,
    /// Collision guard state; null while the guard is disabled or unavailable.
    #[serde(default)]
    pub collision: Option<crate::collision::CollisionObservation>,
    /// Estimated supply current and the power governor state (`src/power.rs`); null when the
    /// profile has no enabled `power_budget`. An estimate from motor feedback, not a measurement.
    #[serde(default)]
    pub power: Option<crate::power::PowerObservation>,
    /// Gripper zero check state (`src/gripper_calib.rs`); null without a gripper policy or < 7 DOF.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gripper_zero: Option<GripperZeroObservation>,
    /// Cartesian TCP pose and orientation of the end link in the base frame; null when kinematics is unavailable.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tcp: Option<TcpState>,
    /// Gripper joint state; null without a gripper joint and policy.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gripper: Option<GripperObs>,
    /// Normalized servo load fraction per joint when `driver_capabilities.load_feedback` is true; omitted otherwise.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub loads: Option<Vec<f32>>,
}

/// Gripper zero state published in `observation.gripper_zero`.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct GripperZeroObservation {
    /// `ok`, `out_of_range`, `checked`, or `calib_invalid`.
    pub state: String,
    pub j7: f32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub offset_rad: Option<f32>,
}

/// A motor fault code reported in the motor's feedback (`observation.motor_errors`).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct MotorError {
    pub code: u8,
    pub name: String,
}

/// Why armd is shaping or refusing the requested motion (`observation.motion_limit`).
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum MotionLimitReason {
    /// The servo speed cap (`servo_max_velocity`) bounds the commanded velocity.
    Speed,
    /// The servo lead cap (`servo_max_lead_rad`) holds the command back.
    Lead,
    /// A servo target at or beyond a joint limit.
    JointLimit,
    /// A servo frame rejected by the Cartesian workspace check.
    Workspace,
    /// The stream ended in a hold: a joint could not follow (no Fault).
    Blocked,
    /// Torque guard: a joint's |torque| stayed at or above its threshold; motion stopped and
    /// the joint is held (no Fault). Targets pushing further in are clamped.
    Torque,
    /// Power governor (`src/power.rs`): the estimated supply current is above the soft
    /// fraction (motion slowed) or a power hold is in force (`observation.power.hold`).
    Power,
}

impl MotionLimitReason {
    /// Higher wins when several limits apply at once.
    pub fn priority(self) -> u8 {
        match self {
            Self::Speed => 0,
            Self::Lead => 1,
            Self::Workspace => 2,
            Self::JointLimit => 3,
            Self::Blocked => 4,
            Self::Power => 5,
            Self::Torque => 6,
        }
    }
}

/// `observation.motion_limit`.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct MotionLimit {
    pub reason: MotionLimitReason,
    /// 0-based joint index; null when the limit is not tied to one joint (workspace).
    pub joint: Option<usize>,
    /// Observation clock (`timestamp`, in ms) when this limit started.
    pub since_ms: u64,
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ControlMode {
    #[default]
    Position,
    GravityComp,
}

/// A code with a human-readable message (entry blocker or exit reason).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ModeReason {
    pub code: String,
    pub message: String,
}

/// Why the last gravity-compensation session ended.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct GravityCompExit {
    pub code: String,
    pub message: String,
    /// Observation timestamp of the exit (s).
    pub at: f64,
}

/// Gravity-compensation state published with every observation.
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq)]
pub struct GravityCompObservation {
    /// The profile and driver support the mode.
    pub supported: bool,
    /// Why entering is refused right now (None when allowed or already active).
    pub blocker: Option<ModeReason>,
    /// `ramp_in`, `active`, `exiting` or `exit_failed`; None in position mode.
    pub phase: Option<String>,
    /// Feed-forward fade-in progress, 0..1.
    pub ramp: f32,
    pub scale: f32,
    pub damping_kd: Vec<f32>,
    /// Feed-forward torque sent to J1-J6 this tick (N·m).
    pub tau_ff: Vec<f32>,
    pub last_exit: Option<GravityCompExit>,
}

/// Field description in the observation schema
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SchemaField {
    pub name: String,
    pub dtype: String,
    pub shape: Vec<usize>,
    pub unit: Option<String>,
}

/// Version of the capability summary in the observation schema (`capabilities`).
pub const CAPABILITIES_VERSION: u32 = 9;

/// Vision features armd offers.
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(default)]
pub struct VisionCapabilities {
    /// armd proxies a detector's API under `/api/v1/detection/*` (upstream
    /// `ARMD_DETECTOR_URL`). Whether a detector is reachable is not checked here.
    pub detection_proxy: bool,
    /// armd proxies the provider's mode API at this route (`GET`/`POST`). Present from
    /// capabilities version 2; whether the provider has it shows in `modes`.
    pub mode_endpoint: String,
    /// Modes the provider reports (`off`, `yolo`, `aruco`, `both` for rebot-vision);
    /// empty when the provider is unreachable or has no mode API.
    pub modes: Vec<String>,
    /// The provider's current base mode, when it reports one.
    pub mode: Option<String>,
    /// The provider's perception API version (`api_version`), when it reports one.
    pub api_version: Option<u32>,
    /// The provider runs `/api/trigger` in every mode, when it reports so.
    pub trigger: Option<bool>,
    /// Whether the provider loaded its YOLO model, when it reports so.
    pub yolo_available: Option<bool>,
    /// The provider runs the pipelines a trigger names (`{"pipelines": [...]}`, perception
    /// API 2). Present from capabilities version 3; false when the provider does not report it.
    pub per_request_pipelines: bool,
    /// The provider serves continuous inference per caller through `/api/v1/vision/subscribe`
    /// (perception API 2). Present from capabilities version 3.
    pub subscriptions: bool,
}

/// armd's route for the vision mode API.
pub const VISION_MODE_ENDPOINT: &str = "/api/v1/vision/mode";

/// What this armd instance supports, derived from the profile and the driver, so clients
/// can adapt without probing routes. Missing in schemas from older armd versions.
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(default)]
pub struct Capabilities {
    /// [`CAPABILITIES_VERSION`]; 0 when the schema has no `capabilities` object.
    pub version: u32,
    /// `profile_version` of the loaded arm profile.
    pub profile_version: u32,
    /// Cartesian actions (`/api/v1/action/move_*`): 7-DOF profile with a kinematics model.
    pub cartesian: bool,
    /// Cartesian workspace check: kinematics model and workspace box.
    pub workspace_check: bool,
    /// `/api/v1/action/gripper`: gripper policy on a profile with a 7th (gripper) joint.
    pub gripper: bool,
    /// `/api/v1/action/grasp`: 7-DOF, kinematics, gripper policy and grasp setup.
    pub grasp: bool,
    /// Grasp tasks (`GraspSetup.tasks`, request `task`, `TASK_UNSUPPORTED`); same value as `grasp`.
    pub grasp_tasks: bool,
    /// Gravity-compensation mode; same value as `observation.gravity_comp.supported`.
    pub gravity_comp: bool,
    /// The profile sets per-joint Damiao mapping ranges (`damiao_ranges`); the startup
    /// range check result is `observation.driver_range_check`.
    pub damiao_ranges: bool,
    /// armd bounds servo streams itself (speed and lead caps, blocked hold) and publishes
    /// `observation.motion_limit`. Present from capabilities version 4.
    pub servo_guard: bool,
    /// Read-only `/api/v1/diagnostics/boots` and `/api/v1/diagnostics/blackbox` exist
    /// (their `enabled` says whether this armd writes the files). Present from
    /// capabilities version 5.
    pub diagnostics: bool,
    /// `/api/v1/ws?obs_hz=N` limits the observation rate per connection. Present from
    /// capabilities version 6, like the fields below.
    pub obs_rate: bool,
    /// `observation.fault` carries `{code, message, recoverable}` in Fault / EStop.
    pub fault_codes: bool,
    /// Named poses: `GET /api/v1/poses`, WS `/api/v1/poses/save|delete`, action `move_to_pose`.
    pub poses: bool,
    /// Keep-out boxes (`workspace.keepout_boxes`, `/api/v1/workspace/keepout`); needs the
    /// workspace check.
    pub keepout: bool,
    /// The profile `speed_cap` section is honoured and published as `schema.speed_cap`.
    pub speed_cap: bool,
    /// `observation.residual_torques` can be non-null (profile gravity model); it stays null
    /// without torque feedback.
    pub residual_torques: bool,
    /// `GET /api/v1/diagnostics/tick`.
    pub tick_stats: bool,
    /// `/api/v1/action/place`: 7-DOF, kinematics and gripper policy.
    pub place: bool,
    /// Collision guard (`collision` profile section, `/api/v1/safety/collision`,
    /// `observation.collision`): profile gravity model and a driver that reports torque.
    pub collision_guard: bool,
    /// `GET /api/v1/safety/describe`: limits, stops and faults generated from the running config.
    pub safety_describe: bool,
    /// Episode replay (`/api/v1/action/replay`) and episode management
    /// (`GET /api/v1/dataset/episodes`, `DELETE /api/v1/dataset/episodes/{id}`). Present from
    /// capabilities version 7. Whether storage is on shows in the listing's `enabled`.
    pub replay: bool,
    /// Torque-saturation guard (`torque_guard` profile section, `schema.torque_guard`,
    /// `motion_limit.reason` = `torque`, action code `TORQUE_LIMIT`): enabled in the profile
    /// with `damiao_ranges`, and a driver that reports torque. Present from capabilities
    /// version 8.
    pub torque_guard: bool,
    /// Supply power budget (`power_budget` profile section, `observation.power`,
    /// `schema.power_budget`, `GET /api/v1/diagnostics/power`): enabled in the profile and a
    /// driver that reports torque. Present from capabilities version 9.
    pub power_budget: bool,
    #[serde(default)]
    pub auto_yaw: bool,
    /// Camera calibration wizard: `GET /api/v1/calibration`, WS `/api/v1/calibration/apply`
    /// and `/api/v1/calibration/restore`, `camera_mount.q_end_cam`; same value as `grasp`.
    #[serde(default)]
    pub calibration_apply: bool,
    pub vision: VisionCapabilities,
}

impl Capabilities {
    /// Capabilities that follow from the profile alone. `gravity_comp` also depends on
    /// the driver and the URDF check; this sets it from the profile only.
    pub fn from_config(config: &ArmConfig) -> Self {
        let arm_with_gripper = config.has_cartesian_with_gripper();
        let kin = config.kinematics.is_some();
        let six_arm = config.arm_dof() == 6 && matches!(config.dof, 6 | 7);
        let gripper = config.gripper_index().is_some() && config.gripper_policy.is_some();
        #[cfg(not(target_arch = "wasm32"))]
        let grasp = arm_with_gripper && kin && config.gripper_policy.is_some() && config.grasp.is_some();
        #[cfg(target_arch = "wasm32")]
        let grasp = false;
        Self {
            version: CAPABILITIES_VERSION,
            profile_version: config.profile_version,
            cartesian: arm_with_gripper && kin,
            workspace_check: kin && config.workspace.is_some(),
            gripper,
            grasp,
            grasp_tasks: grasp,
            gravity_comp: config.six_plus_gripper() && config.gravity_model.is_some() && config.gravity_comp.is_some(),
            damiao_ranges: config.damiao_ranges.is_some(),
            servo_guard: true,
            diagnostics: true,
            obs_rate: true,
            fault_codes: true,
            poses: true,
            keepout: kin && config.workspace.is_some(),
            speed_cap: true,
            residual_torques: config.gravity_model.is_some() && six_arm,
            tick_stats: true,
            place: arm_with_gripper && kin && config.gripper_policy.is_some(),
            // The driver half is applied by ArmSupervisor::get_schema.
            collision_guard: config.gravity_model.is_some() && six_arm,
            safety_describe: true,
            replay: true,
            // The driver half is applied by ArmSupervisor::get_schema.
            torque_guard: crate::torque_guard::TorqueGuardInfo::from_config(config).is_some(),
            // The driver half is applied by ArmSupervisor::get_schema.
            power_budget: config.power_budget.as_ref().is_some_and(|p| p.enabled),
            auto_yaw: false,
            calibration_apply: grasp,
            vision: VisionCapabilities { detection_proxy: true, mode_endpoint: VISION_MODE_ENDPOINT.to_string(), ..Default::default() },
        }
    }
}

/// Observation schema for clients
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObservationSchema {
    #[serde(default)]
    pub profile_id: Option<String>,
    #[serde(default)]
    pub arm_dof: usize,
    #[serde(default)]
    pub gripper_joint: Option<usize>,
    #[serde(default)]
    pub orientation_dof: usize,
    pub dof: usize,
    pub joint_names: Vec<String>,
    pub fields: Vec<SchemaField>,
    pub limits: Vec<JointLimit>,
    #[serde(default)]
    pub workspace: Option<WorkspacePolicy>,
    #[serde(default)]
    pub camera_mount: CameraMount,
    #[serde(default)]
    pub kinematics: Option<KinematicsModel>,
    #[serde(default)]
    pub capabilities: Capabilities,
    #[serde(default)]
    pub gripper_policy: Option<GripperPolicy>,
    /// Effective servo-streaming speed cap per joint (rad/s): min(`servo_max_velocity`,
    /// joint limit); the joint limit where no cap applies. Empty in older schemas.
    #[serde(default)]
    pub servo_max_velocity: Vec<f32>,
    /// Servo lead cap (rad); null in older schemas.
    #[serde(default)]
    pub servo_max_lead_rad: Option<f32>,
    /// Profile speed cap on planned motion; null when the profile sets none.
    #[serde(default)]
    pub speed_cap: Option<SpeedCap>,
    /// Profile home pose (rad, one value per joint); null in older schemas.
    #[serde(default)]
    pub home_positions: Option<Vec<f32>>,
    /// Profile resting-pose policy (tolerances, stillness and support-torque limits); null
    /// when the profile has none and in older schemas.
    #[serde(default)]
    pub resting_pose: Option<RestingPosePolicy>,
    /// Torque-saturation guard policy; null when the profile has none (and in older schemas).
    #[serde(default)]
    pub torque_guard: Option<crate::torque_guard::TorqueGuardInfo>,
    /// armd joint -> kinematic model (URDF) joint, one entry per arm joint: model angle =
    /// `sign * position + offset_rad`. Every pose, FK and IK of the API uses this mapping.
    /// Empty in older schemas (those assumed model = motor for J1..J6).
    #[serde(default)]
    pub model_joint_map: Vec<crate::chain::JointMap>,
    /// Supply power budget in force; null when the profile has none (and in older schemas).
    #[serde(default)]
    pub power_budget: Option<crate::power::PowerBudget>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[cfg(not(target_arch = "wasm32"))]
    pub feetech_joints: Option<Vec<crate::protocol::feetech::FeetechJointCalib>>,
}

impl ObservationSchema {
    pub fn from_config(config: &ArmConfig) -> Self {
        let orientation_dof = if config.kinematics.is_some() {
            if config.arm_dof() >= 6 {
                3
            } else if config.arm_dof() == 5 {
                2
            } else {
                0
            }
        } else {
            0
        };
        Self {
            profile_id: config.profile_id.clone(),
            arm_dof: config.arm_dof(),
            gripper_joint: config.gripper_index(),
            orientation_dof,
            dof: config.dof,
            joint_names: config.joint_names.clone(),
            fields: vec![
                SchemaField {
                    name: "timestamp".to_string(),
                    dtype: "float64".to_string(),
                    shape: vec![1],
                    unit: Some("seconds".to_string()),
                },
                SchemaField {
                    name: "positions".to_string(),
                    dtype: "float32".to_string(),
                    shape: vec![config.dof],
                    unit: Some("radians".to_string()),
                },
                SchemaField {
                    name: "commanded_positions".to_string(),
                    dtype: "float32".to_string(),
                    shape: vec![config.dof],
                    unit: Some("radians".to_string()),
                },
                SchemaField {
                    name: "velocities".to_string(),
                    dtype: "float32".to_string(),
                    shape: vec![config.dof],
                    unit: Some("rad/s".to_string()),
                },
                SchemaField {
                    name: "torques".to_string(),
                    dtype: "float32".to_string(),
                    shape: vec![config.dof],
                    unit: Some("Nm".to_string()),
                },
                SchemaField {
                    name: "temperatures".to_string(),
                    dtype: "float32".to_string(),
                    shape: vec![config.dof],
                    unit: Some("celsius".to_string()),
                },
                SchemaField {
                    name: "is_torqued".to_string(),
                    dtype: "bool".to_string(),
                    shape: vec![1],
                    unit: None,
                },
                SchemaField {
                    name: "epoch_id".to_string(),
                    dtype: "uint64".to_string(),
                    shape: vec![1],
                    unit: None,
                },
                SchemaField {
                    name: "gravity_compensated".to_string(),
                    dtype: "bool".to_string(),
                    shape: vec![1],
                    unit: None,
                },
                SchemaField {
                    name: "gravity_torques".to_string(),
                    dtype: "float32".to_string(),
                    shape: vec![config.arm_dof()],
                    unit: Some("Nm".to_string()),
                },
            ],
            limits: config.limits.clone(),
            workspace: config.workspace.clone(),
            camera_mount: config.camera_mount.clone(),
            kinematics: config.kinematics,
            capabilities: Capabilities::from_config(config),
            gripper_policy: config.gripper_policy.clone(),
            servo_max_velocity: config.servo_limits().iter().map(|l| l.max_velocity).collect(),
            servo_max_lead_rad: Some(config.servo_max_lead_rad),
            speed_cap: config.speed_cap,
            home_positions: Some(config.home_positions.clone()),
            resting_pose: config.resting_pose.clone(),
            torque_guard: crate::torque_guard::TorqueGuardInfo::from_config(config),
            model_joint_map: config.arm_model.joint_map(),
            power_budget: config.power_budget.clone().filter(|p| p.enabled),
            #[cfg(not(target_arch = "wasm32"))]
            feetech_joints: config.feetech.as_ref().map(|f| f.joints.clone()),

        }
    }
}

#[cfg(test)]
mod in_memory_model_tests {
    use super::*;

    #[test]
    fn built_in_in_memory_model_uses_motor_offsets() {
        let mut config = ArmConfig::rebot_dm();
        config.model_offsets_rad = None;
        config.init_arm_model_in_memory(None).unwrap();
        let map = config.arm_model.joint_map();
        assert_eq!(map[5].offset_rad, -std::f64::consts::FRAC_PI_2);
        let default_pose = ArmConfig::rebot_dm().arm_model.fk(&[0.0; 6]);
        assert_eq!(config.arm_model.fk(&[0.0; 6]), default_pose);
    }

    #[test]
    fn urdf_in_memory_validates_checksum_and_preserves_model_on_failure() {
        let mut config: ArmConfig = serde_json::from_str(include_str!("../profiles/so_arm101.json")).unwrap();
        let xml = include_str!("../assets/so101_new_calib.urdf");
        config.init_arm_model_in_memory(Some(xml)).unwrap();
        assert_eq!(config.arm_dof(), 5);
        let before = config.arm_model.fk(&[0.0; 6]);
        config.urdf_sha256 = Some("0".repeat(64));
        assert!(config.init_arm_model_in_memory(Some(xml)).unwrap_err().contains("sha256"));
        assert_eq!(config.arm_model.fk(&[0.0; 6]), before);
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn native_and_in_memory_profile_models_are_identical() {
        let profile = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("profiles/so_arm101.json");
        let native = ArmConfig::from_profile_file(profile).unwrap();
        let mut memory: ArmConfig = serde_json::from_str(include_str!("../profiles/so_arm101.json")).unwrap();
        memory.init_arm_model_in_memory(Some(include_str!("../assets/so101_new_calib.urdf"))).unwrap();
        assert_eq!(native.arm_model.joint_map(), memory.arm_model.joint_map());
        assert_eq!(native.arm_model.chain(), memory.arm_model.chain());
        assert_eq!(native.arm_model.fk(&[0.2, -0.3, 0.4, 0.1, -0.2, 0.0]),
            memory.arm_model.fk(&[0.2, -0.3, 0.4, 0.1, -0.2, 0.0]));
    }
}
