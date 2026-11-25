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
#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub struct DriverRangeCheck {
    pub status: RangeCheckStatus,
    pub message: String,
    pub joints: Vec<JointRangeReading>,
    /// Time the check took (ms).
    pub duration_ms: u64,
}

crate::fieldwise_clone!(DriverRangeCheck { status, message, joints, duration_ms });



/// `Clone` whose `clone_from` copies field by field, so `String` / `Vec` / `Option` fields
/// reuse their memory: the control tick fills its published observation in place without
/// allocating while the shape does not change (armd 1.6 RT core). Every field must be
/// listed (a missing one does not compile).
#[macro_export]
macro_rules! fieldwise_clone {
    ($t:ident { $($f:ident),* $(,)? }) => {
        impl Clone for $t {
            fn clone(&self) -> Self { Self { $($f: self.$f.clone()),* } }
            fn clone_from(&mut self, src: &Self) { $(self.$f.clone_from(&src.$f);)* }
        }
    };
}

/// Maximum total joint count supported inline across motion planning, trajectories,
/// power budgeting, and stationary settle tracking: an 8-joint URDF chain plus 1 gripper.
pub const MAX_MOTION_DOF: usize = crate::chain::MAX_CHAIN_JOINTS + 1;

/// Up to [`ArmJoints::CAP`] joint indices without a heap allocation (the arm joints of a
/// profile, [`ArmConfig::arm_joints`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ArmJoints {
    idx: [usize; Self::CAP],
    len: usize,
}

impl ArmJoints {
    pub const CAP: usize = 16;

    pub fn collect_from(it: impl IntoIterator<Item = usize>) -> Self {
        let mut s = Self { idx: [0; Self::CAP], len: 0 };
        for i in it {
            assert!(s.len < Self::CAP, "more than {} arm joints", Self::CAP);
            s.idx[s.len] = i;
            s.len += 1;
        }
        s
    }
}

impl std::ops::Deref for ArmJoints {
    type Target = [usize];
    fn deref(&self) -> &[usize] { &self.idx[..self.len] }
}

/// Limits for a single joint
#[derive(Debug, Serialize, Deserialize, PartialEq)]
pub struct JointLimit {
    pub name: String,
    pub min_position: f32,
    pub max_position: f32,
    pub max_velocity: f32,
    pub max_acceleration: f32,
}

crate::fieldwise_clone!(JointLimit { name, min_position, max_position, max_velocity, max_acceleration });

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
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
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

fn is_zero_f32(v: &f32) -> bool { *v == 0.0 }

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
    /// Optional collision capsules attached to arm links / kinematic frames.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub link_capsules: Vec<LinkCapsule>,
    /// Enable link-link self-collision checking across `self_collision_pairs`.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub self_collision_enabled: bool,
    /// Minimum required clearance (m) between self-collision capsule pairs (default 0.0).
    #[serde(default, skip_serializing_if = "is_zero_f32")]
    pub self_collision_min_clearance_m: f32,
    /// Non-adjacent `link_capsules` id pairs checked for self-collision.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub self_collision_pairs: Vec<[String; 2]>,
}

/// Most keep-out boxes a profile or the runtime list may hold.
pub const MAX_KEEPOUT_BOXES: usize = 16;

/// Most link capsules a workspace policy may define.
pub const MAX_LINK_CAPSULES: usize = 16;

/// Most self-collision capsule pairs a workspace policy may define.
pub const MAX_SELF_COLLISION_PAIRS: usize = 64;

/// Capsule attached to a link/kinematic frame for workspace and keep-out checks.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct LinkCapsule {
    pub id: String,
    pub frame: String,
    pub a_m: [f32; 3],
    pub b_m: [f32; 3],
    pub radius_m: f32,
}

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
            // Link capsules enclosing sim/mujoco/models/rebot_b601dm_scene.xml collision boxes + ~4-5 mm margin,
            // plus base_link pedestal + base_plate and link6 wrist housing for non-adjacent self-collision checks:
            // - base_link: pedestal z>=0.014 of the URDF base_link mesh (max_d 53.2 mm <= r=0.055; z 0.040..0.070 keeps
            //   the dome 0.125 m high: the real pedestal ends at z 0.0877, a segment top at 0.080 put the dome 46 mm above it)
            // - base_plate: mounting flange [-0.07,0.07]x[-0.10,0.10]x[0,0.014] (max_d 83.2 mm <= r=0.084)
            // - link2: box pos [-0.132, 0, -0.0316], size [0.1605, 0.0285, 0.0337] (half-diag 44.1 mm) -> r=0.050
            // - link3: box pos [0.1222, -0.03, -0.0311], size [0.1489, 0.0525, 0.0401] (half-diag 66.1 mm) -> r=0.071
            // - link4: box pos [0.0449, -0.03, -0.03], size [0.0729, 0.058, 0.0334] (half-diag 66.9 mm) -> r=0.072
            // - link5: fitted to the URDF link5 mesh (max_d 44.3 mm <= r=0.050, 5.7 mm margin); the MuJoCo box (half-diag
            //   52.0 mm) overstates the part. With r=0.057 / base top 0.080 the folded rest pose (HOME, J 0) sat on the
            //   base_link-link5 bound under the live kinematic correction (T21); now >= 17 mm at rest, >= 5 mm within +-2 deg.
            // - link6: box pos [0, 0, 0.0242], size [0.0385, 0.0318, 0.029] (max corner dist 51.6 mm) -> r=0.052
            link_capsules: vec![
                LinkCapsule {
                    id: "base_link".into(),
                    frame: "base_link".into(),
                    a_m: [0.0, 0.0, 0.040],
                    b_m: [0.0, 0.0, 0.070],
                    radius_m: 0.055,
                },
                LinkCapsule {
                    id: "base_plate".into(),
                    frame: "base_link".into(),
                    a_m: [0.0, -0.05, 0.007],
                    b_m: [0.0, 0.05, 0.007],
                    radius_m: 0.084,
                },
                LinkCapsule {
                    id: "link2".into(),
                    frame: "link2".into(),
                    a_m: [0.020, 0.0, -0.0316],
                    b_m: [-0.284, 0.0, -0.0316],
                    radius_m: 0.050,
                },
                LinkCapsule {
                    id: "link3".into(),
                    frame: "link3".into(),
                    a_m: [-0.015, -0.030, -0.0311],
                    b_m: [0.260, -0.030, -0.0311],
                    radius_m: 0.071,
                },
                LinkCapsule {
                    id: "link4".into(),
                    frame: "link4".into(),
                    a_m: [-0.015, -0.030, -0.030],
                    b_m: [0.105, -0.030, -0.030],
                    radius_m: 0.072,
                },
                LinkCapsule {
                    id: "link5".into(),
                    frame: "link5".into(),
                    a_m: [-0.0028, 0.0, 0.010],
                    b_m: [-0.0028, 0.0, 0.060],
                    radius_m: 0.050,
                },
                LinkCapsule {
                    id: "link6".into(),
                    frame: "link6".into(),
                    a_m: [0.0, 0.0, 0.008],
                    b_m: [0.0, 0.0, 0.04],
                    radius_m: 0.052,
                },
            ],
            self_collision_enabled: true,
            self_collision_min_clearance_m: 0.0,
            self_collision_pairs: vec![
                ["base_link".into(), "link4".into()],
                ["base_link".into(), "link5".into()],
                ["base_link".into(), "link6".into()],
                ["base_plate".into(), "link4".into()],
                ["base_plate".into(), "link5".into()],
                ["base_plate".into(), "link6".into()],
            ],
        }
    }

    /// Whether self-collision checking is actively configured with non-empty valid pairs.
    pub fn self_collision_covered(&self) -> bool {
        self.self_collision_enabled
            && !self.self_collision_pairs.is_empty()
            && self.self_collision_pairs.iter().all(|[a, b]| {
                a != b
                    && self.link_capsules.iter().any(|c| c.id == *a)
                    && self.link_capsules.iter().any(|c| c.id == *b)
            })
    }

    /// Copy with `floor_z`, outer `box`, and `max_reach` disabled (used by `park` so it
    /// can escape outer workspace violations while still enforcing `keepout_boxes`).
    pub fn keepout_only(&self) -> Self {
        Self {
            floor_z_m: f32::NEG_INFINITY,
            box_min_m: [f32::NEG_INFINITY; 3],
            box_max_m: [f32::INFINITY; 3],
            max_reach_m: None,
            check_points_end_m: self.check_points_end_m.clone(),
            keepout_boxes: self.keepout_boxes.clone(),
            link_capsules: self.link_capsules.clone(),
            self_collision_enabled: self.self_collision_enabled,
            self_collision_min_clearance_m: self.self_collision_min_clearance_m,
            self_collision_pairs: self.self_collision_pairs.clone(),
        }
    }

    pub fn validate(&self) -> Result<(), String> {
        let finite3 = |v: &[f32; 3]| v.iter().all(|x| x.is_finite());
        let valid_cp = |v: &[f32; 3]| {
            finite3(v)
                && (f64::from(v[0]).powi(2) + f64::from(v[1]).powi(2) + f64::from(v[2]).powi(2))
                    .sqrt()
                    <= crate::tool::MAX_CHECK_POINT_NORM_M
        };
        if !self.floor_z_m.is_finite() || !finite3(&self.box_min_m) || !finite3(&self.box_max_m)
            || self.box_min_m.iter().zip(&self.box_max_m).any(|(lo, hi)| lo >= hi)
            || self.max_reach_m.is_some_and(|r| !r.is_finite() || r <= 0.0)
            || self.check_points_end_m.is_empty() || self.check_points_end_m.len() > 32
            || !self.check_points_end_m.iter().all(valid_cp) {
            return Err("Invalid workspace policy: finite values, box_min < box_max, 1..32 check points within 0.50 m".into());
        }
        if self.link_capsules.len() > MAX_LINK_CAPSULES {
            return Err(format!("Invalid workspace policy: at most {MAX_LINK_CAPSULES} link_capsules"));
        }
        let mut ids = std::collections::HashSet::with_capacity(self.link_capsules.len());
        for (i, c) in self.link_capsules.iter().enumerate() {
            if c.id.is_empty() || c.id.len() > 64 {
                return Err(format!("Invalid workspace policy: link_capsules[{i}] id must be 1..64 bytes"));
            }
            if !ids.insert(c.id.as_str()) {
                return Err(format!("Invalid workspace policy: duplicate link_capsule id '{}'", c.id));
            }
            if c.frame.is_empty() {
                return Err(format!("Invalid workspace policy: link_capsules[{i}] ('{}') frame must not be empty", c.id));
            }
            if !finite3(&c.a_m) || !finite3(&c.b_m) || !c.radius_m.is_finite() || c.radius_m < 0.0 {
                return Err(format!("Invalid workspace policy: link_capsules[{i}] ('{}') requires finite endpoints and radius_m >= 0", c.id));
            }
        }
        if !self.self_collision_min_clearance_m.is_finite() || self.self_collision_min_clearance_m < 0.0 {
            return Err("Invalid workspace policy: self_collision_min_clearance_m must be finite and >= 0".into());
        }
        if self.self_collision_pairs.len() > MAX_SELF_COLLISION_PAIRS {
            return Err(format!("Invalid workspace policy: at most {MAX_SELF_COLLISION_PAIRS} self_collision_pairs"));
        }
        let mut pair_set = std::collections::HashSet::with_capacity(self.self_collision_pairs.len());
        for (i, [a, b]) in self.self_collision_pairs.iter().enumerate() {
            if !ids.contains(a.as_str()) {
                return Err(format!("Invalid workspace policy: self_collision_pairs[{i}] references unknown link_capsule id '{a}'"));
            }
            if !ids.contains(b.as_str()) {
                return Err(format!("Invalid workspace policy: self_collision_pairs[{i}] references unknown link_capsule id '{b}'"));
            }
            if a == b {
                return Err(format!("Invalid workspace policy: self_collision_pairs[{i}] cannot pair '{a}' with itself"));
            }
            let key = if a.as_str() <= b.as_str() { (a.as_str(), b.as_str()) } else { (b.as_str(), a.as_str()) };
            if !pair_set.insert(key) {
                return Err(format!("Invalid workspace policy: duplicate self_collision_pairs[{i}] ('{a}', '{b}')"));
            }
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

fn is_zero_tcp(v: &[f64; 3]) -> bool {
    *v == [0.0; 3]
}

fn is_stock_active_tool(t: &crate::tool::ActiveToolInfo) -> bool {
    t.is_stock && t.id == crate::tool::STOCK_TOOL_ID
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
    /// Absolute gripper torque cap while closing (N·m): |τ| above it on fresh feedback for
    /// [`crate::supervisor::GRIPPER_FORCE_CAP_TICKS`] ticks latches the stall whatever the jaw
    /// speed and holds the measured position. Absent: no cap (drivers without torque feedback).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_close_torque_nm: Option<f32>,
    /// Grasp contact (close with grasp detection): |τ| at or above this (N·m) for 2 ticks while
    /// closing marks the contact point; the jaw then closes slowly to contact + squeeze_rad and
    /// never further. Absent: contact from the following error only.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub contact_torque_nm: Option<f32>,
    /// Bound on how far the gripper command may run ahead of the measured jaw in the closing
    /// direction (rad), torque on and fresh feedback, so a blocked jaw cannot build a large
    /// position error. Absent: no bound.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_lead_rad: Option<f32>,
    /// Contact regulator of servo streams and live policies (rad/s; needs `contact_torque_nm`
    /// and torque feedback). From the contact (|τ| at or above `contact_torque_nm`, or at or
    /// above the stall threshold with the jaw still, while the stream closes) the J7 command
    /// may run ahead of the last written command by at most this speed while |τ| is below
    /// `contact_torque_nm`; at or above it the lead over the measured jaw is scaled by
    /// `contact_torque_nm / |τ|`, which holds the grip near `contact_torque_nm` whatever the
    /// object stiffness. Ends when the jaw opens 0.05 rad past the contact point, when the jaw
    /// closes on 0.1 rad with |τ| below half of `contact_torque_nm` (the object gave way), or
    /// when the stream ends. Gripper actions (grasp) keep their own contact handling. Absent: off.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stream_contact_speed_rad_s: Option<f32>,
    /// J7 speed (rad/s) of closing gripper steps that armd plans itself: grasp, place and
    /// sequence closes and moves, and every closing stroke of `gripper_calibrate`. Absent:
    /// [`DEFAULT_CLOSE_SPEED_RAD_S`]. A gripper action keeps its own `speed`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub close_speed_rad_s: Option<f32>,
}

/// Default `gripper_policy.close_speed_rad_s` (rad/s): the grasp contact speed
/// (`motion::GRIPPER_CONTACT_SPEED`).
pub const DEFAULT_CLOSE_SPEED_RAD_S: f32 = 0.3;

impl GripperPolicy {
    /// `close_speed_rad_s` or [`DEFAULT_CLOSE_SPEED_RAD_S`] (rad/s).
    pub fn close_speed(&self) -> f64 { f64::from(self.close_speed_rad_s.unwrap_or(DEFAULT_CLOSE_SPEED_RAD_S)) }

    /// Zero-allocation copy of all scalar policy fields with `jaw_opening: None`
    /// (used on the RT thread when constructing [`crate::motion::GripperMotion`], which never
    /// reads `jaw_opening`).
    pub fn clone_without_jaw_opening(&self) -> Self {
        Self {
            closed_rad: self.closed_rad,
            open_rad: self.open_rad,
            holding_gap_rad: self.holding_gap_rad,
            holding_torque_nm: self.holding_torque_nm,
            squeeze_rad: self.squeeze_rad,
            settle_ms: self.settle_ms,
            jaw_opening: None,
            zero_check_supported: self.zero_check_supported,
            holding_mode: self.holding_mode,
            holding_load_fraction: self.holding_load_fraction,
            max_close_torque_nm: self.max_close_torque_nm,
            contact_torque_nm: self.contact_torque_nm,
            max_lead_rad: self.max_lead_rad,
            stream_contact_speed_rad_s: self.stream_contact_speed_rad_s,
            close_speed_rad_s: self.close_speed_rad_s,
        }
    }

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
            // Office B601-DM 2026-11-20 (rc10, toy banana, black box 5 Hz): free close at
            // ~1 rad/s reads 0.30-0.45 N·m, the first contact sample 0.85 N·m; the torque then
            // rose to 5.0 N·m while the jaw kept moving at ~1 rad/s and a finger broke.
            // Cap 1.0 N·m: 2.2x the free-close peak, a fifth of the torque at the break.
            max_close_torque_nm: Some(1.0),
            // same log: 1.3x the free-close peak, below the first contact sample (0.85)
            contact_torque_nm: Some(0.6),
            // free close at 1 rad/s lags 0.21-0.23 rad behind the command (same log)
            max_lead_rad: Some(0.35),
            // the grasp contact speed (motion::GRIPPER_CONTACT_SPEED): one 20 ms tick adds
            // 0.006 rad of lead, 0.18 N·m on a 30 N·m/rad contact
            stream_contact_speed_rad_s: Some(0.3),
            close_speed_rad_s: None,
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

/// Unified post-merge validation for an effective [`GripperPolicy`]: checks policy bounds,
/// `jaw_opening` monotonicity, direction (`open_rad` wider than `closed_rad`), full angular span
/// coverage `[min(closed, open), max(closed, open)]`, and reachable open width `>= max_grasp_width_m`.
pub fn validate_effective_gripper_policy(
    g: &GripperPolicy,
    lim: Option<&JointLimit>,
    max_grasp_width_m: Option<f64>,
) -> Result<(), String> {
    validate_effective_gripper_policy_with_mode(g, lim, max_grasp_width_m, true, true)
}

/// Like [`validate_effective_gripper_policy`], optionally skipping J7 limit checks on
/// `jaw_opening` points (after a calibration zero-offset shift clamped only `open_rad`).
pub fn validate_effective_gripper_policy_ex(
    g: &GripperPolicy,
    lim: Option<&JointLimit>,
    max_grasp_width_m: Option<f64>,
    check_jaw_limits: bool,
) -> Result<(), String> {
    validate_effective_gripper_policy_with_mode(g, lim, max_grasp_width_m, check_jaw_limits, true)
}

/// Like [`validate_effective_gripper_policy_ex`], with explicit `allow_sim_default` gate:
/// when `false` and `max_grasp_width_m > 0`, missing `jaw_opening` is rejected instead of falling
/// back to [`JawOpening::sim_default()`].
pub fn validate_effective_gripper_policy_with_mode(
    g: &GripperPolicy,
    lim: Option<&JointLimit>,
    max_grasp_width_m: Option<f64>,
    check_jaw_limits: bool,
    allow_sim_default: bool,
) -> Result<(), String> {
    let in_lim = |v: f32| lim.is_none_or(|l| v >= l.min_position && v <= l.max_position);
    if ![g.closed_rad, g.open_rad, g.holding_gap_rad, g.holding_torque_nm, g.squeeze_rad]
        .iter()
        .all(|v| v.is_finite())
        || !in_lim(g.closed_rad)
        || !in_lim(g.open_rad)
        || (g.closed_rad - g.open_rad).abs() < 0.1
        || g.holding_gap_rad < 0.0
        || g.holding_torque_nm < 0.0
        || !(0.0..=0.5).contains(&g.squeeze_rad)
        || g.settle_ms == 0
        || g.settle_ms > 5000
    {
        return Err(
            "Invalid gripper policy (open/closed inside the gripper joint limits, 0 <= squeeze_rad <= 0.5)"
                .into(),
        );
    }
    if let Some(frac) = g.holding_load_fraction {
        if !frac.is_finite() || !(0.0..=1.0).contains(&frac) {
            return Err("Invalid gripper policy: holding_load_fraction must be in 0.0..=1.0".into());
        }
    }
    let positive = |v: Option<f32>| v.is_none_or(|v| v.is_finite() && v > 0.0);
    if !positive(g.max_close_torque_nm)
        || !positive(g.contact_torque_nm)
        || !positive(g.max_lead_rad)
        || !positive(g.stream_contact_speed_rad_s)
    {
        return Err("Invalid gripper policy: max_close_torque_nm, contact_torque_nm, max_lead_rad and stream_contact_speed_rad_s must be positive".into());
    }
    if g.close_speed_rad_s.is_some_and(|v| !(v.is_finite() && v > 0.0 && v <= 2.0)) {
        return Err("Invalid gripper policy: close_speed_rad_s must be in (0, 2] rad/s".into());
    }
    if let (Some(cap), Some(contact)) = (g.max_close_torque_nm, g.contact_torque_nm) {
        if contact >= cap {
            return Err(
                "Invalid gripper policy: contact_torque_nm must be below max_close_torque_nm".into(),
            );
        }
    }
    if g.holding_mode == HoldingMode::LoadAndGap && g.holding_load_fraction.unwrap_or(0.0) <= 0.0 {
        return Err(
            "Invalid gripper policy: LoadAndGap holding mode requires holding_load_fraction > 0.0"
                .into(),
        );
    }
    if let Some(jo) = &g.jaw_opening {
        jo.validate(if check_jaw_limits { lim } else { None })
            .map_err(|e| format!("Invalid gripper policy: {e}"))?;
        let w_closed = jo.opening_m(g.closed_rad);
        let w_open = jo.opening_m(g.open_rad);
        if w_open <= w_closed + 1e-6 {
            return Err(format!(
                "Invalid gripper policy: jaw_opening at open_rad ({w_open:.4} m) must be wider than at closed_rad ({w_closed:.4} m)"
            ));
        }
        let first = jo.points.first().expect("validated >= 2 points");
        let last = jo.points.last().expect("validated >= 2 points");
        let slope_dir = (last[1] - first[1]) / (last[0] - first[0]);
        if slope_dir * (g.open_rad - g.closed_rad) <= 0.0 {
            return Err(
                "Invalid gripper policy: jaw_opening direction opposes closed_rad -> open_rad"
                    .into(),
            );
        }
        let jo_min = jo.points.iter().map(|p| p[0]).fold(f32::INFINITY, f32::min);
        let jo_max = jo.points.iter().map(|p| p[0]).fold(f32::NEG_INFINITY, f32::max);
        let pol_min = g.closed_rad.min(g.open_rad);
        let pol_max = g.closed_rad.max(g.open_rad);
        if jo_min > pol_min + 1e-4 || jo_max < pol_max - 1e-4 {
            return Err(format!(
                "Invalid gripper policy: jaw_opening span [{jo_min:.4}, {jo_max:.4}] does not cover policy span [{pol_min:.4}, {pol_max:.4}]"
            ));
        }
    }
    if let Some(max_w) = max_grasp_width_m.filter(|w| *w > 0.0) {
        if !allow_sim_default && g.jaw_opening.is_none() {
            return Err(format!(
                "Invalid gripper policy: jaw_opening calibration curve is required on physical/non-stock tools to verify max_grasp_width_m {max_w:.4} m"
            ));
        }
        let w_open = g.opening_m(g.open_rad);
        if f64::from(w_open) + 1e-6 < max_w {
            return Err(format!(
                "Invalid gripper policy: reachable open width {w_open:.4} m is below max_grasp_width_m {max_w:.4} m"
            ));
        }
    }
    Ok(())
}

/// How a grasp setup's `max_grasp_width_m` relates to the gripper's jaw curve. Never an
/// error: persisted setups and gripper calibrations stay valid whatever the result.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GraspWidthCheck {
    /// `max_grasp_width_m` is 0 (width check off).
    Off,
    /// The jaw curve opens at least `max_grasp_width_m`.
    Verified { curve_max_m: f64 },
    /// No jaw curve on a physical follower: `max_grasp_width_m` is unverified and used as set.
    Unverified,
    /// The jaw curve opens less than `max_grasp_width_m`: grasps use `curve_max_m`.
    Clamped { curve_max_m: f64 },
}

impl GraspWidthCheck {
    /// Status name for the API (`off`, `verified`, `unverified`, `clamped`).
    pub fn status(&self) -> &'static str {
        match self {
            Self::Off => "off",
            Self::Verified { .. } => "verified",
            Self::Unverified => "unverified",
            Self::Clamped { .. } => "clamped",
        }
    }

    /// The max grasp width grasps use for a setup value `max_w`.
    pub fn effective_max_m(&self, max_w: f64) -> f64 {
        match self {
            Self::Clamped { curve_max_m } => max_w.min(*curve_max_m),
            _ => max_w,
        }
    }

    /// The curve's open width (None without a curve or with the check off).
    pub fn curve_max_m(&self) -> Option<f64> {
        match self {
            Self::Verified { curve_max_m } | Self::Clamped { curve_max_m } => Some(*curve_max_m),
            _ => None,
        }
    }
}

/// Compare `max_w` with the open width of `g`'s jaw curve. `require_real_curve` (physical
/// follower): a missing curve gives [`GraspWidthCheck::Unverified`] instead of comparing against
/// `JawOpening::sim_default()`.
pub fn check_grasp_width(g: &GripperPolicy, max_w: f64, require_real_curve: bool) -> GraspWidthCheck {
    if max_w <= 0.0 {
        return GraspWidthCheck::Off;
    }
    if require_real_curve && g.jaw_opening.is_none() {
        return GraspWidthCheck::Unverified;
    }
    let curve_max_m = f64::from(g.opening_m(g.open_rad));
    if curve_max_m + 1e-6 < max_w {
        GraspWidthCheck::Clamped { curve_max_m }
    } else {
        GraspWidthCheck::Verified { curve_max_m }
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

/// How the Damiao driver commands the gripper joint (`ArmConfig::damiao_gripper`).
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum DamiaoGripperMode {
    /// Position-force (CTRL_MODE 4): every frame carries `current_ceiling_raw`.
    PosForce,
    /// Legacy POS_VEL (CTRL_MODE 2): no motor current cap.
    PosVel,
}

/// Damiao gripper drive: base mode, motor current ceiling and closing speed.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct DamiaoGripperDrive {
    pub mode: DamiaoGripperMode,
    /// Current ceiling of every position-force frame, in 1/10000 of the motor's maximum phase
    /// current (1..=10000). Its torque equivalent is a property of the motor and the gripper
    /// and must be commissioned on the arm (docs/gripper_pos_force.md).
    pub current_ceiling_raw: u16,
    /// Speed limit of closing frames (rad/s).
    pub close_speed_rad_s: f32,
    /// The ceiling's torque equivalent was measured (commissioning); false: uncalibrated
    /// (estimate or default, not externally commissioned).
    #[serde(default)]
    pub calibrated: bool,
    /// Measured torque equivalent of the ceiling (N·m), from commissioning.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ceiling_torque_nm: Option<f32>,
    /// `external` (independent force / torque reference) or `feedback_equivalent` (the motor's
    /// own torque feedback only).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub torque_reference: Option<String>,
    /// Maximum time (ms) J7 position-force feedback may remain stale (> 100 ms) before latching
    /// recoverable `J7_FEEDBACK_STALE` (default 250 ms; allowed range 200..=450 ms, i.e.
    /// >= 2 x 100 ms freshness bound and strictly < 500 ms `BUS_LOSS_TIMEOUT`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feedback_stale_fault_ms: Option<u64>,
}

impl DamiaoGripperDrive {
    /// Default bounded staleness limit before latching recoverable `J7_FEEDBACK_STALE` (ms).
    pub const DEFAULT_FEEDBACK_STALE_FAULT_MS: u64 = 250;
    /// Minimum configurable staleness fault limit (2 x 100 ms `POS_FORCE_FEEDBACK_MAX_AGE`).
    pub const MIN_FEEDBACK_STALE_FAULT_MS: u64 = 200;
    /// Maximum configurable staleness fault limit (strictly below 500 ms `BUS_LOSS_TIMEOUT`).
    pub const MAX_FEEDBACK_STALE_FAULT_MS: u64 = 450;

    /// Feedback-equivalent torque ratio (N·m per raw current unit) measured from office B601-DM J7
    /// stall plateaus on 2026-11-23 (raw 220 -> 0.42 N·m, raw 300 -> 0.55–0.57 N·m).
    pub const FEEDBACK_EQUIVALENT_NM_PER_RAW: f32 = 0.0019;

    /// B601-DM J7 default: position-force with raw 460 (4.6 % of maximum phase current),
    /// `calibrated: false`. On 2026-11-23 (office B601-DM, build 1.6.0-dev8j7 / 027e46c),
    /// stalled feedback-torque plateaus measured 0.42 N·m at raw 220 and 0.55-0.57 N·m at raw 300
    /// (~0.0019 N·m/raw, no external reference), making raw 460 a feedback-equivalent ~0.87 N·m
    /// estimate above the 0.79 N·m free-motion peak near full open, not an external calibration.
    pub fn rebot_dm() -> Self {
        Self { mode: DamiaoGripperMode::PosForce, current_ceiling_raw: 460, close_speed_rad_s: 0.30,
            calibrated: false, ceiling_torque_nm: None, torque_reference: None,
            feedback_stale_fault_ms: None }
    }

    /// Resolved bounded staleness limit (ms) for J7 position-force feedback.
    pub fn resolved_feedback_stale_fault_ms(&self) -> u64 {
        self.feedback_stale_fault_ms.unwrap_or(Self::DEFAULT_FEEDBACK_STALE_FAULT_MS)
    }

    /// Legacy profile: J7 in POS_VEL, no motor current cap.
    pub fn legacy_pos_vel() -> Self {
        Self { mode: DamiaoGripperMode::PosVel, ..Self::rebot_dm() }
    }

    /// Wire speed quantization from `f64` (0.01 rad/s units, floored in f64 so wire speed never
    /// exceeds `speed_rad_s`; non-finite -> 0). Single authority shared by protocol encoding,
    /// config validation, driver checks and gripper motion planning.
    pub fn quantize_vel_raw_f64(speed_rad_s: f64) -> u16 {
        if !speed_rad_s.is_finite() { return 0; }
        (speed_rad_s.abs() * 100.0).floor().clamp(0.0, f64::from(u16::MAX)) as u16
    }

    /// Wire speed quantization from `f32` (delegates to [`Self::quantize_vel_raw_f64`]).
    pub fn quantize_vel_raw(speed_rad_s: f32) -> u16 {
        Self::quantize_vel_raw_f64(f64::from(speed_rad_s))
    }

    /// Smallest `f32` that encodes back to `raw` (preventing double-flooring when the planned
    /// trajectory velocity is re-encoded by `pos_force_vel_raw`).
    pub fn wire_speed_f32(raw: u16) -> f32 {
        if raw == 0 { return 0.0; }
        let v = (f64::from(raw) / 100.0) as f32;
        if Self::quantize_vel_raw(v) < raw { v.next_up() } else { v }
    }

    /// Largest `f32` not exceeding `raw * 0.01` rad/s: used for trajectory stepping so
    /// `bounded_servo_step` never advances the target faster than the wire speed `raw * 0.01`.
    pub fn step_speed_f32(raw: u16) -> f32 {
        if raw == 0 { return 0.0; }
        let exact = f64::from(raw) / 100.0;
        let v = exact as f32;
        if f64::from(v) > exact { v.next_down() } else { v }
    }

    pub fn validate(&self) -> Result<(), String> {
        if !(1..=10_000).contains(&self.current_ceiling_raw) {
            return Err("damiao_gripper.current_ceiling_raw must be 1..=10000".into());
        }
        if !(self.close_speed_rad_s.is_finite() && self.close_speed_rad_s > 0.0) {
            return Err("damiao_gripper.close_speed_rad_s must be positive".into());
        }
        if self.mode == DamiaoGripperMode::PosForce && Self::quantize_vel_raw(self.close_speed_rad_s) == 0 {
            return Err("damiao_gripper.close_speed_rad_s encodes to 0 on the wire (must quantize to >= 0.01 rad/s)".into());
        }
        if self.calibrated && !self.ceiling_torque_nm.is_some_and(|t| t.is_finite() && t > 0.0) {
            return Err("damiao_gripper: calibrated needs a positive ceiling_torque_nm".into());
        }
        if let Some(r) = &self.torque_reference {
            if r != "external" && r != "feedback_equivalent" {
                return Err("damiao_gripper.torque_reference must be external or feedback_equivalent".into());
            }
        }
        if let Some(ms) = self.feedback_stale_fault_ms {
            if !(Self::MIN_FEEDBACK_STALE_FAULT_MS..=Self::MAX_FEEDBACK_STALE_FAULT_MS).contains(&ms) {
                return Err(format!(
                    "damiao_gripper.feedback_stale_fault_ms must be {}..={} ms, got {ms}",
                    Self::MIN_FEEDBACK_STALE_FAULT_MS,
                    Self::MAX_FEEDBACK_STALE_FAULT_MS
                ));
            }
        }
        Ok(())
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

/// Default for [`ArmConfig::rest_pose_tolerance_rad`]: maximum per-joint deviation from
/// the profile's rest pose that permits torque disable while stopped (E-stop or Fault).
pub const DEFAULT_REST_POSE_TOLERANCE_RAD: f32 = 0.05;

fn default_rest_pose_tolerance_rad() -> f32 {
    DEFAULT_REST_POSE_TOLERANCE_RAD
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

pub use crate::motor_mode::{
    ControlMode as MotorControlMode, ImpedanceLimits, JointControlModeConfig, ModeLimits,
    PositionForceLimits, PositionLimits, PositionVelocityLimits, ResolvedJointMode, VendorBinding,
    VendorCapability, VENDOR_CAPABILITIES,
};
pub use crate::tracking::{JointTrackingConfig, JointTrackingJoint, TrackingStrategy};

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
    /// Home / park target (joint angles, rad). Derived from `zero_reference` when the profile
    /// has one (then it may be omitted; a conflicting value is rejected).
    #[serde(default)]
    pub home_positions: Vec<f32>,
    pub lease_timeout_ms: u64,
    pub max_temperature: f32,
    pub control_frequency_hz: u32,
    /// None forbids automatic hardware torque release without a calibrated profile.
    #[serde(default)]
    pub resting_pose: Option<RestingPosePolicy>,
    /// Per-joint tolerance (rad) from the profile's rest pose for allowing torque disable while
    /// stopped (E-stop or Fault). Checked on arm joints (excluding the gripper) against fresh
    /// feedback (<= 100 ms). Default: 0.05 rad.
    #[serde(default = "default_rest_pose_tolerance_rad")]
    pub rest_pose_tolerance_rad: f32,
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
    /// Default detector models of the detection kinds (`POST /api/v1/vision/detector`); None =
    /// the built-in rebot-vision names ([`crate::vision_detector::DetectorDefaults::default`]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[cfg(not(target_arch = "wasm32"))]
    pub vision_detectors: Option<crate::vision_detector::DetectorDefaults>,
    /// Gravity compensation (hand-guiding); None means unsupported.
    #[serde(default)]
    pub gravity_comp: Option<GravityCompPolicy>,
    /// Joint tracking compensation (`src/tracking.rs`); None = legacy output path.
    /// A block that is not enabled never rejects the profile (`deserialize_profile_block`).
    #[serde(default, skip_serializing_if = "Option::is_none", deserialize_with = "crate::tracking::deserialize_profile_block")]
    pub joint_tracking: Option<JointTrackingConfig>,
    /// Damiao mapping range per joint (one per DOF). None keeps the single default
    /// range ±12.5 rad, ±30 rad/s, ±18 N·m for every motor.
    #[serde(default)]
    pub damiao_ranges: Option<Vec<DamiaoRange>>,
    /// Damiao gripper drive (ignored without a gripper joint); None: the gripper uses the arm's
    /// POS_VEL base mode without being reported (profiles from before this field).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub damiao_gripper: Option<DamiaoGripperDrive>,
    /// Optional per-joint control-mode declarations with typed limits (length `dof`, each entry
    /// `null` to translate from existing follower/profile settings). Omitting this field keeps
    /// existing behaviour identically.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub control_mode: Option<Vec<Option<JointControlModeConfig>>>,
    /// Per-tick Damiao CAN TX start-cutoff budget (ms); None uses `DAMIAO_TX_MAX_DURATION` (10 ms).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tx_budget_ms: Option<u64>,
    /// Maximum consecutive control ticks with a pending CAN TX deferral before latching
    /// `CAN_TX_STALLED`. None defaults to `ceil(45 ms / period)` (3 ticks at 50 Hz).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tx_stall_ticks: Option<u32>,
    /// Speed cap for every planned motion (actions, jog, home, park); None = no cap.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub speed_cap: Option<SpeedCap>,
    /// Same-side (backlash-free) final approach (`src/backlash.rs`); None = direct motions.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub backlash_approach: Option<crate::backlash::BacklashApproach>,
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
    /// Documentation of the zero pose (every motor at 0) for the motor zero tool; None = no
    /// description (the 3D view still renders the pose).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub zero_pose: Option<ZeroPoseDoc>,
    /// The physical reference pose of the joint zero (the folded rest pose) in URDF angles;
    /// `home_positions` (and so the park target) follow from it, and the zero tools set the
    /// joint zero so that the arm reads it in this pose (src/joint_zero.rs).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub zero_reference: Option<ZeroReference>,
    /// The tool the fingertip calibration measures (calibration wizard, src/calibration.rs):
    /// its id and the height of the sticker edge above the tool's lowest point; None = no
    /// known tool (the acceptance then reports sticker-edge heights and sets no table height).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub calibration_tool: Option<CalibrationToolSpec>,
    /// Active tool/gripper identity (stock `"default"` when no `--tool` overlay is applied).
    #[serde(default, skip_deserializing, skip_serializing_if = "is_stock_active_tool")]
    pub active_tool: crate::tool::ActiveToolInfo,
    /// Tool TCP offset in the `end_link` frame (m).
    #[serde(default, skip_serializing_if = "is_zero_tcp")]
    pub tcp_offset: [f64; 3],
    /// Layer 1 of the collision envelope: robot/rail check points in `end_link` frame (m).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub base_check_points_end_m: Vec<[f32; 3]>,
    /// Layer 3 of the collision envelope: permanent tool check points in `end_link` frame (m).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub permanent_tool_check_points_end_m: Vec<[f32; 3]>,
    /// Layer 4 of the collision envelope: transient payload check points in `end_link` frame (m).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub transient_payload_check_points_end_m: Vec<[f32; 3]>,
    /// Physical resting contact point in `end_link` frame (m) for table datum derivation.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub physical_contact_end_m: Option<[f64; 3]>,
    /// Jaw contact opening gap when closed (m).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub contact_opening_m: Option<f32>,
    /// Whether active tool dynamics are qualified for gravity compensation / feedforward.
    #[serde(default = "default_true", skip_deserializing, skip_serializing_if = "is_true")]
    pub dynamics_qualified: bool,
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
            rest_pose_tolerance_rad: DEFAULT_REST_POSE_TOLERANCE_RAD,
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
            #[cfg(not(target_arch = "wasm32"))] vision_detectors: None,
            gravity_comp: None,
            joint_tracking: None,
            damiao_ranges: None,
            damiao_gripper: None,
            control_mode: None,
            tx_budget_ms: None,
            tx_stall_ticks: None,
            speed_cap: None,
            backlash_approach: None,
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
            zero_pose: None,
            zero_reference: None,
            calibration_tool: None,
            active_tool: crate::tool::ActiveToolInfo::default(),
            tcp_offset: [0.0; 3],
            base_check_points_end_m: Vec::new(),
            permanent_tool_check_points_end_m: Vec::new(),
            transient_payload_check_points_end_m: Vec::new(),
            physical_contact_end_m: None,
            contact_opening_m: None,
            dynamics_qualified: true,
            arm_model: crate::chain::ArmModel::REBOT_DEV_ARM,
        }
    }
}

impl ArmConfig {
    /// Minimum (and default) consecutive deferred control ticks before latching `CAN_TX_STALLED`:
    /// `ceil(frame_max_age / period) + 1` with `frame_max_age = 45 ms` (`DAMIAO_TX_FRAME_MAX_AGE`),
    /// ensuring a single deferral never latches before a queued frame can reach 45 ms age
    /// (e.g. 2 at 10 Hz, 3 at 25 Hz, 4 at 50 Hz, 6 at 100 Hz).
    pub fn min_tx_stall_ticks(&self) -> u32 {
        let hz = u64::from(self.control_frequency_hz.max(1));
        let ceil_age_ticks = (45u64.saturating_mul(hz).saturating_add(999)) / 1000;
        (ceil_age_ticks + 1) as u32
    }

    /// Consecutive ticks with a pending CAN TX deferral before latching `CAN_TX_STALLED`.
    /// Defaults to [`Self::min_tx_stall_ticks`] (`ceil(45 ms / period) + 1`: 4 ticks at 50 Hz,
    /// 2 ticks at 10 Hz). Values below the minimum are rejected by [`Self::validate`] and clamped
    /// at runtime as defense-in-depth.
    pub fn resolved_tx_stall_ticks(&self) -> u32 {
        let min_ticks = self.min_tx_stall_ticks();
        self.tx_stall_ticks.map_or(min_ticks, |ticks| ticks.max(min_ticks))
    }

    /// Effective Damiao per-tick CAN TX start-cutoff (ms): explicit `tx_budget_ms` when set,
    /// otherwise the default `10` ms (`DAMIAO_TX_MAX_DURATION`).
    pub fn resolved_tx_budget_ms(&self) -> u64 {
        self.tx_budget_ms.unwrap_or(10)
    }

    /// Resolved bounded J7 position-force staleness fault limit (ms), clamped as defense-in-depth
    /// to `[MIN_FEEDBACK_STALE_FAULT_MS, MAX_FEEDBACK_STALE_FAULT_MS]` (`200..=450` ms, default
    /// `250` ms).
    pub fn resolved_j7_stale_fault_ms(&self) -> u64 {
        self.damiao_gripper
            .as_ref()
            .map_or(
                DamiaoGripperDrive::DEFAULT_FEEDBACK_STALE_FAULT_MS,
                DamiaoGripperDrive::resolved_feedback_stale_fault_ms,
            )
            .clamp(
                DamiaoGripperDrive::MIN_FEEDBACK_STALE_FAULT_MS,
                DamiaoGripperDrive::MAX_FEEDBACK_STALE_FAULT_MS,
            )
    }

    /// Damiao per-tick CAN TX budget derived from this profile (`tx_budget_ms` overrides
    /// `DAMIAO_TX_MAX_DURATION`; `max_frames = 16`, `frame_max_age = 45 ms`).
    #[cfg(not(target_arch = "wasm32"))]
    pub fn damiao_tx_budget(&self) -> crate::driver::TxBudget {
        crate::driver::TxBudget {
            max_duration: std::time::Duration::from_millis(self.resolved_tx_budget_ms()),
            ..crate::driver::TxBudget::default()
        }
    }

    /// Convert an `end_link` pose into the active TCP pose (`p_tcp = p_end + R_end * tcp_offset`).
    pub fn end_to_tcp(&self, end: &crate::kinematics::Pose) -> crate::kinematics::Pose {
        if self.tcp_offset == [0.0; 3] {
            return *end;
        }
        crate::kinematics::Pose::new(
            end.r,
            crate::kinematics::add(&end.p, &crate::kinematics::mat_vec(&end.r, &self.tcp_offset)),
        )
    }

    /// Convert a TCP pose into the `end_link` pose (`p_end = p_tcp - R_tcp * tcp_offset`).
    pub fn tcp_to_end(&self, tcp: &crate::kinematics::Pose) -> crate::kinematics::Pose {
        if self.tcp_offset == [0.0; 3] {
            return *tcp;
        }
        crate::kinematics::Pose::new(
            tcp.r,
            crate::kinematics::sub(&tcp.p, &crate::kinematics::mat_vec(&tcp.r, &self.tcp_offset)),
        )
    }

    /// TCP pose from model angles `q`.
    pub fn tcp_pose(&self, q: &[f64; 6]) -> crate::kinematics::Pose {
        self.end_to_tcp(&self.arm_model.fk(q))
    }

    /// TCP pose from armd motor angles `q`.
    pub fn tcp_pose_f32(&self, q: &[f32]) -> Option<crate::kinematics::Pose> {
        self.arm_model.fk_f32(q).map(|end| self.end_to_tcp(&end))
    }

    /// Physical contact point in `end_link` frame for table datum derivation
    /// (`physical_contact_end_m` when set, else `[0, 0, 0]` for stock grippers).
    pub fn effective_physical_contact_end_m(&self) -> Option<[f64; 3]> {
        self.physical_contact_end_m
            .or_else(|| self.active_tool.is_stock.then_some([0.0; 3]))
    }

    /// Collect the deduplicated tool/workspace envelope check points in `end_link` frame (m):
    /// `base_check_points_end_m` (or `workspace.check_points_end_m`), `tcp_offset` (if nonzero),
    /// `physical_contact_end_m` (if non-stock or nonzero), `permanent_tool_check_points_end_m`,
    /// and `transient_payload_check_points_end_m`.
    pub fn effective_tool_envelope_points_end_m(&self) -> Vec<[f32; 3]> {
        let base_slice = if !self.base_check_points_end_m.is_empty() {
            self.base_check_points_end_m.as_slice()
        } else {
            self.workspace
                .as_ref()
                .map(|w| w.check_points_end_m.as_slice())
                .unwrap_or(&[])
        };
        let mut effective: Vec<[f32; 3]> = Vec::new();
        let mut push_unique = |pt: [f32; 3]| {
            if !effective.iter().any(|e| {
                (e[0] - pt[0]).abs() <= 1e-6
                    && (e[1] - pt[1]).abs() <= 1e-6
                    && (e[2] - pt[2]).abs() <= 1e-6
            }) {
                effective.push(pt);
            }
        };
        for &p in base_slice {
            push_unique(p);
        }
        if self.tcp_offset != [0.0; 3] {
            push_unique([
                self.tcp_offset[0] as f32,
                self.tcp_offset[1] as f32,
                self.tcp_offset[2] as f32,
            ]);
        }
        if let Some(pc) = self.physical_contact_end_m {
            if !self.active_tool.is_stock || pc != [0.0; 3] {
                push_unique([pc[0] as f32, pc[1] as f32, pc[2] as f32]);
            }
        }
        for &p in &self.permanent_tool_check_points_end_m {
            push_unique(p);
        }
        for &p in &self.transient_payload_check_points_end_m {
            push_unique(p);
        }
        effective
    }

    /// Recompute `workspace.check_points_end_m` as the deduplicated 4-layer union of
    /// `base_check_points_end_m`, `tcp_offset` (if nonzero), `physical_contact_end_m`,
    /// `permanent_tool_check_points_end_m`, and `transient_payload_check_points_end_m`.
    pub fn rebuild_effective_check_points(&mut self) -> Result<(), String> {
        crate::tool::validate_tcp_offset(&self.tcp_offset)?;
        if let Some(pc) = &self.physical_contact_end_m {
            crate::tool::validate_physical_contact_end_m(pc, "physical_contact_end_m")?;
        }
        let has_pc_layer = self
            .physical_contact_end_m
            .is_some_and(|p| !self.active_tool.is_stock || p != [0.0; 3]);
        let no_extra_layers = self.tcp_offset == [0.0; 3]
            && !has_pc_layer
            && self.permanent_tool_check_points_end_m.is_empty()
            && self.transient_payload_check_points_end_m.is_empty();
        if self.base_check_points_end_m.is_empty() && !no_extra_layers {
            if let Some(ws) = &self.workspace {
                self.base_check_points_end_m = ws.check_points_end_m.clone();
            }
        }
        let ws_base = (self.base_check_points_end_m.is_empty() && no_extra_layers)
            .then(|| self.workspace.as_ref().map(|w| w.check_points_end_m.as_slice()))
            .flatten()
            .unwrap_or(&[]);
        for (label, pts) in [
            ("base_check_points_end_m", if self.base_check_points_end_m.is_empty() { ws_base } else { self.base_check_points_end_m.as_slice() }),
            ("permanent_tool_check_points_end_m", self.permanent_tool_check_points_end_m.as_slice()),
            ("transient_payload_check_points_end_m", self.transient_payload_check_points_end_m.as_slice()),
        ] {
            for (i, p) in pts.iter().enumerate() {
                if !p.iter().all(|v| v.is_finite()) {
                    return Err(format!("{label}[{i}] must be finite"));
                }
                let norm = (f64::from(p[0]).powi(2)
                    + f64::from(p[1]).powi(2)
                    + f64::from(p[2]).powi(2))
                .sqrt();
                if norm > crate::tool::MAX_CHECK_POINT_NORM_M {
                    return Err(format!(
                        "{label}[{i}] norm {norm:.4} m exceeds limit {:.2} m",
                        crate::tool::MAX_CHECK_POINT_NORM_M
                    ));
                }
            }
        }
        if self.base_check_points_end_m.is_empty() && no_extra_layers {
            return Ok(());
        }
        let effective = self.effective_tool_envelope_points_end_m();
        if effective.len() > crate::tool::MAX_EFFECTIVE_CHECK_POINTS {
            return Err(format!(
                "effective check_points_end_m count {} exceeds limit {}",
                effective.len(),
                crate::tool::MAX_EFFECTIVE_CHECK_POINTS
            ));
        }
        if let Some(ws) = self.workspace.as_mut() {
            if effective.is_empty() {
                return Err("effective check_points_end_m cannot be empty".into());
            }
            ws.check_points_end_m = effective;
        }
        Ok(())
    }
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

    /// Arm joints (armd indices below `dof`, not the gripper, at most `arm_dof` of them).
    pub fn arm_joint_indices(&self) -> Vec<usize> {
        let g = self.gripper_index();
        (0..self.dof).filter(|i| Some(*i) != g).take(self.arm_dof()).collect()
    }

    /// [`Self::arm_joint_indices`] without a heap allocation (the control tick's form).
    pub fn arm_joints(&self) -> ArmJoints {
        let g = self.gripper_index();
        ArmJoints::collect_from((0..self.dof).filter(|i| Some(*i) != g).take(self.arm_dof()))
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
        crate::tool::validate_tcp_offset(&config.tcp_offset)?;
        if let Some(pc) = &config.physical_contact_end_m {
            crate::tool::validate_physical_contact_end_m(pc, "physical_contact_end_m")?;
        }
        if let Some(co) = config.contact_opening_m {
            crate::tool::validate_contact_opening_m(f64::from(co), "contact_opening_m")?;
        }
        config.active_tool = crate::tool::stock_active_tool_info(&config);
        config.dynamics_qualified = true;
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

    /// Joint angles of `zero_reference` (None without one): URDF angle -> joint angle through the
    /// model map (joint = (urdf - offset) / sign) for the arm joints, other joints as given.
    pub fn zero_reference_home(&self) -> Result<Option<Vec<f32>>, String> {
        let Some(r) = &self.zero_reference else { return Ok(None); };
        if r.positions.len() != self.dof || !r.positions.iter().all(|v| v.is_finite()) {
            return Err(format!("zero_reference.positions needs {} finite values", self.dof));
        }
        let map: Vec<crate::chain::JointMap> = if self.uses_urdf() {
            self.arm_model.joint_map()
        } else {
            self.builtin_model_offsets()?.iter().map(|o| crate::chain::JointMap { sign: 1.0, offset_rad: *o }).collect()
        };
        Ok(Some(r.positions.iter().enumerate().map(|(i, u)| match map.get(i) {
            Some(m) if m.sign != 0.0 => ((u - m.offset_rad) / m.sign) as f32,
            _ => *u as f32,
        }).collect()))
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
        if self.dof > MAX_MOTION_DOF {
            return Err(format!(
                "DOF {} exceeds maximum supported {MAX_MOTION_DOF} (up to {} arm joints + gripper)",
                self.dof,
                crate::chain::MAX_CHAIN_JOINTS
            ));
        }
        if let Some(t) = &self.calibration_tool { t.validate()?; }
        if self.limits.len() != self.dof {
            return Err(format!("Limits length {} does not match DOF {}", self.limits.len(), self.dof));
        }
        if let Some(home) = self.zero_reference_home()? {
            let conflict = self.home_positions.len() == self.dof
                && self.home_positions.iter().zip(&home).any(|(a, b)| (a - b).abs() > 1e-6);
            if conflict {
                return Err(format!("home_positions {:?} conflict with zero_reference (joint angles {home:?}); omit home_positions", self.home_positions));
            }
            self.home_positions = home;
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
        {
            let period_ms = (1000 / u64::from(self.control_frequency_hz.max(1))).max(1);
            let half_period_ms = period_ms / 2;
            let max_ms = half_period_ms.min(45);
            if max_ms == 0 {
                return Err(format!(
                    "control_frequency_hz ({}) has period {period_ms} ms; half control period is < 1 ms for tx_budget_ms",
                    self.control_frequency_hz
                ));
            }
            let ms = self.resolved_tx_budget_ms();
            if ms == 0 || ms > max_ms {
                return Err(format!(
                    "effective tx_budget_ms ({ms} ms) must be in 1..={max_ms} (tx_budget_ms <= period/2 = {half_period_ms} ms at {} Hz, frame max age 45 ms)",
                    self.control_frequency_hz
                ));
            }
        }
        {
            let min_stall_ticks = self.min_tx_stall_ticks();
            if let Some(ticks) = self.tx_stall_ticks {
                if ticks < min_stall_ticks {
                    return Err(format!(
                        "tx_stall_ticks ({ticks}) must be >= {min_stall_ticks} (ceil(45 ms / period) + 1 at {} Hz)",
                        self.control_frequency_hz
                    ));
                }
            }
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
        if let Some(b) = &self.backlash_approach { b.validate(&self.joint_names, &self.arm_joint_indices())?; }
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
        if !self.rest_pose_tolerance_rad.is_finite() || self.rest_pose_tolerance_rad <= 0.0 {
            return Err(format!(
                "rest_pose_tolerance_rad {} must be finite and positive",
                self.rest_pose_tolerance_rad
            ));
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
        if let Some(jt) = &self.joint_tracking {
            let checked = jt.validate(self.arm_dof(), self.dof, self.gripper_index());
            #[cfg(not(target_arch = "wasm32"))]
            let checked = checked.and_then(|()| crate::mit::validate_profile(self, jt));
            match checked {
                Ok(()) => {}
                Err(e) if jt.enabled => return Err(e),
                // A disabled block is never evaluated: like a profile without one (before joint
                // tracking existed), it does not reject the profile.
                Err(e) => tracing::warn!("joint_tracking is disabled and invalid (ignored): {e}"),
            }
        }
        if let Some(ws) = &self.workspace {
            ws.validate()?;
            for (i, c) in ws.link_capsules.iter().enumerate() {
                if self.arm_model.resolve_frame(&c.frame).is_none() {
                    return Err(format!(
                        "Invalid workspace policy: link_capsules[{i}] ('{}') unknown frame '{}'",
                        c.id, c.frame
                    ));
                }
            }
        }
        let cam = &self.camera_mount;
        if !cam.rot_deg.is_finite() || !cam.tilt_deg.is_finite()
            || cam.offset_m.is_some_and(|o| o.iter().any(|v| !v.is_finite())) {
            return Err("Invalid camera mount".into());
        }
        if let Some(q) = &cam.q_end_cam { unit_quaternion(q)?; }
        if let Some(g) = &self.gripper_policy {
            let lim = self.gripper_index().and_then(|idx| self.limits.get(idx));
            if lim.is_none() {
                return Err("Invalid gripper policy (open/closed inside the gripper joint limits, 0 <= squeeze_rad <= 0.5)".into());
            }
            #[cfg(not(target_arch = "wasm32"))]
            let max_w = self.grasp.as_ref().map(|gr| gr.max_grasp_width_m);
            #[cfg(target_arch = "wasm32")]
            let max_w = crate::tool::baseline_grasp_limits(self).0;
            validate_effective_gripper_policy(g, lim, max_w)?;
        }
        // without a gripper joint the block is ignored (gripper_drive_from_profile)
        if let Some(g) = &self.damiao_gripper { g.validate()?; }
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
        #[cfg(not(target_arch = "wasm32"))]
        if let Some(d) = &self.vision_detectors { d.validate()?; }
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
        if let (Some(g), Some(dg)) = (self.gripper_index(), &self.damiao_gripper) {
            if dg.mode == DamiaoGripperMode::PosForce {
                if self.limits.get(g).is_some_and(|l| DamiaoGripperDrive::quantize_vel_raw(l.max_velocity) == 0) {
                    return Err(format!(
                        "Joint {g} max_velocity encodes to 0 on the pos-force wire (must quantize to >= 0.01 rad/s)"
                    ));
                }
                if let Some(gp) = &self.gripper_policy {
                    if DamiaoGripperDrive::quantize_vel_raw_f64(gp.close_speed()) == 0 {
                        return Err(
                            "gripper_policy.close_speed_rad_s encodes to 0 on the pos-force wire (must quantize to >= 0.01 rad/s)".into()
                        );
                    }
                }
            }
        }
        if self.home_positions.iter().zip(&self.limits).any(|(p, l)| !p.is_finite() || *p < l.min_position || *p > l.max_position) {
            return Err("Home positions must be finite and within joint limits".into());
        }
        crate::tool::validate_tcp_offset(&self.tcp_offset)?;
        if let Some(pc) = &self.physical_contact_end_m {
            crate::tool::validate_physical_contact_end_m(pc, "physical_contact_end_m")?;
        }
        if let Some(co) = self.contact_opening_m {
            crate::tool::validate_contact_opening_m(f64::from(co), "contact_opening_m")?;
        }
        if self.workspace.is_some()
            || !self.base_check_points_end_m.is_empty()
            || !self.permanent_tool_check_points_end_m.is_empty()
            || !self.transient_payload_check_points_end_m.is_empty()
            || self.tcp_offset != [0.0; 3]
            || self.physical_contact_end_m.is_some()
        {
            self.rebuild_effective_check_points()?;
        }
        if (self.active_tool.id == crate::tool::STOCK_TOOL_ID) != self.active_tool.is_stock {
            return Err("active_tool id 'default' is reserved exclusively for stock configuration".into());
        }
        if !crate::tool::is_valid_tool_kind(&self.active_tool.kind) {
            return Err(format!("active_tool.kind '{}' is invalid", self.active_tool.kind));
        }
        if self.active_tool.is_stock {
            if self.tcp_offset != [0.0; 3]
                || !self.permanent_tool_check_points_end_m.is_empty()
                || self.active_tool.commissioning_required
                || self.active_tool.tcp_offset_m != [0.0; 3]
                || self.active_tool.kind != "gripper"
            {
                return Err("stock active_tool cannot carry non-zero tcp_offset, permanent tool check points, commissioning requirement, or non-gripper kind".into());
            }
            self.active_tool = crate::tool::stock_active_tool_info(self);
            self.dynamics_qualified = true;
        } else {
            if !valid_tool_id(&self.active_tool.id)
                || self.active_tool.schema_version != 1
                || !self.active_tool.config_digest.starts_with("v1:")
                || self.active_tool.config_digest.len() <= 3
                || self.active_tool.tcp_offset_m != self.tcp_offset
            {
                return Err("non-stock active_tool identity must have schema_version=1, valid tool id, 'v1:' config_digest, and matching tcp_offset_m".into());
            }
        }
        self.validate_control_mode()?;
        Ok(())
    }

    fn has_feetech_config(&self) -> bool {
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.feetech.is_some()
        }
        #[cfg(target_arch = "wasm32")]
        {
            false
        }
    }

    fn feetech_joint_step_range(&self, joint: usize) -> Option<[u16; 2]> {
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.feetech.as_ref().and_then(|ft| ft.joints.get(joint)).map(|jc| {
                let (lo, hi) = jc.effective_range_steps();
                [lo, hi]
            })
        }
        #[cfg(target_arch = "wasm32")]
        {
            let _ = joint;
            None
        }
    }

    fn feetech_joint_explicit_step_bounds(&self, joint: usize) -> Option<(Option<u16>, Option<u16>)> {
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.feetech
                .as_ref()
                .and_then(|ft| ft.joints.get(joint))
                .map(|jc| (jc.range_min_steps, jc.range_max_steps))
        }
        #[cfg(target_arch = "wasm32")]
        {
            let _ = joint;
            None
        }
    }

    fn enabled_mit_tracking_joint(&self, joint: usize) -> bool {
        let Some(jt) = self.joint_tracking.as_ref().filter(|jt| jt.enabled) else {
            return false;
        };
        let arm = self.arm_joint_indices();
        let slots = jt.mit_joint_slots();
        arm.iter()
            .enumerate()
            .any(|(k, &j)| j == joint && slots.contains(&k))
    }

    fn joint_requires_runtime_impedance(&self, joint: usize) -> bool {
        let gc_arm = self.gravity_comp.is_some() && self.arm_joint_indices().contains(&joint);
        gc_arm || self.enabled_mit_tracking_joint(joint)
    }

    /// Compatibility reader for the Damiao gripper drive: returns `damiao_gripper` when set
    /// (with a gripper joint), or translates an explicit `control_mode[gripper]` `PositionForce`
    /// entry when `damiao_gripper` is omitted.
    pub fn effective_damiao_gripper(&self) -> Option<DamiaoGripperDrive> {
        let g = self.gripper_index()?;
        if let Some(dg) = &self.damiao_gripper {
            return Some(dg.clone());
        }
        let entry = self
            .control_mode
            .as_ref()
            .and_then(|entries| entries.get(g))
            .and_then(|e| e.as_ref())?;
        match (&entry.mode, &entry.limits) {
            (MotorControlMode::PositionForce, Some(ModeLimits::PositionForce(pf))) => {
                Some(pf.to_damiao_gripper())
            }
            _ => None,
        }
    }

    /// Infer the hardware vendor binding from profile sections when no CLI `--follower` binding
    /// is supplied (e.g. `--follower mock` / `--follower sim` or standalone profile inspection).
    pub fn infer_vendor_binding(&self) -> Option<VendorBinding> {
        if let Some(entries) = &self.control_mode {
            if let Some(v) = entries.iter().flatten().find_map(|e| e.vendor) {
                return Some(v);
            }
        }
        if self.has_feetech_config()
            || self
                .profile_id
                .as_deref()
                .is_some_and(|id| id.starts_with("so_arm"))
        {
            return Some(VendorBinding::Feetech);
        }
        if self.damiao_ranges.is_some()
            || self.damiao_gripper.is_some()
            || self.kinematics == Some(KinematicsModel::RebotDevArm)
            || self.gravity_model == Some(GravityModel::RebotDevArm)
            || self.profile_id.as_deref() == Some("rebot_b601dm")
        {
            return Some(VendorBinding::Damiao);
        }
        if self.profile_id.as_deref() == Some("rebot_b601_rs") {
            return Some(VendorBinding::RobStride);
        }
        None
    }

    fn validate_vendor_profile_sections(&self, vendor: VendorBinding) -> Result<(), String> {
        if vendor != VendorBinding::Damiao
            && (self.damiao_ranges.is_some() || self.damiao_gripper.is_some())
        {
            return Err(format!(
                "vendor '{vendor}' conflicts with damiao_ranges / damiao_gripper in profile"
            ));
        }
        if vendor != VendorBinding::Feetech && self.has_feetech_config() {
            return Err(format!(
                "vendor '{vendor}' conflicts with feetech section in profile"
            ));
        }
        Ok(())
    }

    fn active_impedance_wire_range(
        &self,
        vendor: VendorBinding,
        joint: usize,
    ) -> Option<(f32, f32, f32)> {
        match vendor {
            VendorBinding::Damiao => Some(
                self.damiao_ranges
                    .as_ref()
                    .and_then(|rs| rs.get(joint))
                    .map_or(
                        (
                            crate::motor_mode::DAMIAO_DEFAULT_P_MAX,
                            crate::motor_mode::DAMIAO_DEFAULT_V_MAX,
                            crate::motor_mode::DAMIAO_DEFAULT_T_MAX,
                        ),
                        |r| (r.p_max, r.v_max, r.t_max),
                    ),
            ),
            VendorBinding::RobStride => Some((
                crate::motor_mode::ROBSTRIDE_DEFAULT_P_MAX,
                crate::motor_mode::ROBSTRIDE_DEFAULT_V_MAX,
                crate::motor_mode::ROBSTRIDE_DEFAULT_T_MAX,
            )),
            VendorBinding::Feetech | VendorBinding::FashionStar => None,
        }
    }

    fn runtime_impedance_limits_for(
        &self,
        vendor: VendorBinding,
        joint: usize,
        runtime_modes: &[MotorControlMode],
    ) -> Option<ImpedanceLimits> {
        if !runtime_modes.contains(&MotorControlMode::Impedance) {
            return None;
        }
        let (p_max, v_max, t_max) = self.active_impedance_wire_range(vendor, joint)?;
        Some(ImpedanceLimits {
            p_max,
            v_max,
            t_max,
            kp_max: crate::motor_mode::IMPEDANCE_WIRE_KP_MAX,
            kd_max: crate::motor_mode::IMPEDANCE_WIRE_KD_MAX,
            effort_limit_nm: None,
        })
    }

    fn validate_impedance_policy_compatibility(
        &self,
        joint: usize,
        imp: &ImpedanceLimits,
    ) -> Result<(), String> {
        let eff_cap = imp.effective_effort_limit_nm();
        let arm_indices = self.arm_joint_indices();
        if let Some(gc) = &self.gravity_comp {
            if arm_indices.contains(&joint) {
                if let Some(&t_lim) = gc.torque_limits_nm.get(joint) {
                    if t_lim > eff_cap {
                        return Err(format!(
                            "control_mode[{joint}] impedance effective effort limit ({eff_cap}) is below gravity_comp torque_limits_nm[{joint}] ({t_lim})"
                        ));
                    }
                }
                if let Some(&kp_lim) = gc.hold_kp.get(joint) {
                    if kp_lim > imp.kp_max {
                        return Err(format!(
                            "control_mode[{joint}] impedance kp_max ({}) is below gravity_comp hold_kp[{joint}] ({kp_lim})",
                            imp.kp_max
                        ));
                    }
                }
                if let Some(&kd_lim) = gc.hold_kd.get(joint) {
                    if kd_lim > imp.kd_max {
                        return Err(format!(
                            "control_mode[{joint}] impedance kd_max ({}) is below gravity_comp hold_kd[{joint}] ({kd_lim})",
                            imp.kd_max
                        ));
                    }
                }
            }
        }
        if let Some(jt) = self.joint_tracking.as_ref().filter(|jt| jt.enabled) {
            if let Some(slot) = arm_indices.iter().position(|&j| j == joint) {
                if let Some(jcfg) = jt.joints.get(slot) {
                    if jcfg.strategy == crate::tracking::TrackingStrategy::Mit {
                        if jcfg.effort_limit_nm > eff_cap {
                            return Err(format!(
                                "control_mode[{joint}] impedance effective effort limit ({eff_cap}) is below joint_tracking.joints[{slot}].effort_limit_nm ({})",
                                jcfg.effort_limit_nm
                            ));
                        }
                        if jcfg.kp > imp.kp_max {
                            return Err(format!(
                                "control_mode[{joint}] impedance kp_max ({}) is below joint_tracking.joints[{slot}].kp ({})",
                                imp.kp_max, jcfg.kp
                            ));
                        }
                        if jcfg.kd > imp.kd_max {
                            return Err(format!(
                                "control_mode[{joint}] impedance kd_max ({}) is below joint_tracking.joints[{slot}].kd ({})",
                                imp.kd_max, jcfg.kd
                            ));
                        }
                        if let Some(hkp) = jcfg.hold_kp {
                            if hkp > imp.kp_max {
                                return Err(format!(
                                    "control_mode[{joint}] impedance kp_max ({}) is below joint_tracking.joints[{slot}].hold_kp ({hkp})",
                                    imp.kp_max
                                ));
                            }
                        }
                        if let Some(hkd) = jcfg.hold_kd {
                            if hkd > imp.kd_max {
                                return Err(format!(
                                    "control_mode[{joint}] impedance kd_max ({}) is below joint_tracking.joints[{slot}].hold_kd ({hkd})",
                                    imp.kd_max
                                ));
                            }
                        }
                    }
                }
            }
        }
        Ok(())
    }

    /// Single-authority validator for one **explicit** `control_mode[joint]` declaration bound
    /// to a concrete [`VendorBinding`]. Omitted slots keep legacy acceptance rules untouched.
    fn validate_explicit_joint_for_vendor(
        &self,
        vendor: VendorBinding,
        joint: usize,
        mode: MotorControlMode,
        limits: &ModeLimits,
        runtime_modes: &[MotorControlMode],
    ) -> Result<(), String> {
        if !vendor.supports_mode(mode) {
            return Err(format!(
                "control_mode[{joint}]: vendor '{vendor}' does not support control mode '{mode}'"
            ));
        }
        if limits.mode() != mode {
            return Err(format!(
                "control_mode[{joint}]: limits for '{}' do not match mode '{mode}'",
                limits.mode()
            ));
        }
        limits
            .validate()
            .map_err(|e| format!("control_mode[{joint}]: {e}"))?;

        for &rt in runtime_modes {
            if !vendor.supports_runtime_switch(mode, rt) {
                return Err(format!(
                    "control_mode[{joint}]: vendor '{vendor}' does not support runtime switch '{mode}' -> '{rt}'"
                ));
            }
        }

        // Joint-role capability checks for Damiao: POS_FORCE is gripper-only; MIT / runtime
        // Impedance transitions are arm-joint-only (never on the gripper joint).
        if vendor == VendorBinding::Damiao {
            let is_gripper = Some(joint) == self.gripper_index();
            if mode == MotorControlMode::PositionForce && !is_gripper {
                return Err(format!(
                    "control_mode[{joint}]: Damiao position_force is only supported on the gripper joint"
                ));
            }
            if is_gripper
                && (mode == MotorControlMode::Impedance
                    || runtime_modes.contains(&MotorControlMode::Impedance))
            {
                return Err(format!(
                    "control_mode[{joint}]: Damiao impedance / runtime MIT transition is only supported on arm joints, not the gripper joint"
                ));
            }
        }

        let jl = self
            .limits
            .get(joint)
            .ok_or_else(|| format!("control_mode[{joint}]: missing joint limits entry"))?;

        match (vendor, limits) {
            (VendorBinding::Damiao, ModeLimits::PositionVelocity(pv)) => {
                let (wire_p_max, wire_v_max, _) = self
                    .active_impedance_wire_range(VendorBinding::Damiao, joint)
                    .unwrap();
                if pv.max_velocity_rad_s > wire_v_max {
                    return Err(format!(
                        "control_mode[{joint}] position_velocity.max_velocity_rad_s ({}) exceeds Damiao wire v_max ({wire_v_max})",
                        pv.max_velocity_rad_s
                    ));
                }
                if jl.min_position < -wire_p_max || jl.max_position > wire_p_max {
                    return Err(format!(
                        "control_mode[{joint}]: joint position range [{}, {}] exceeds Damiao wire p_max +/-{wire_p_max}",
                        jl.min_position, jl.max_position
                    ));
                }
            }
            (VendorBinding::Damiao, ModeLimits::Impedance(imp)) => {
                let (wire_p, wire_v, wire_t) = self
                    .active_impedance_wire_range(VendorBinding::Damiao, joint)
                    .unwrap();
                if (wire_p - imp.p_max).abs() > 1e-5
                    || (wire_v - imp.v_max).abs() > 1e-5
                    || (wire_t - imp.t_max).abs() > 1e-5
                {
                    return Err(format!(
                        "control_mode[{joint}] impedance wire range ({}/{}/{}) conflicts with active Damiao wire mapping ({wire_p}/{wire_v}/{wire_t})",
                        imp.p_max, imp.v_max, imp.t_max
                    ));
                }
                if jl.min_position < -imp.p_max || jl.max_position > imp.p_max {
                    return Err(format!(
                        "control_mode[{joint}]: joint position range [{}, {}] exceeds impedance wire p_max +/-{}",
                        jl.min_position, jl.max_position, imp.p_max
                    ));
                }
                self.validate_impedance_policy_compatibility(joint, imp)?;
            }
            (VendorBinding::Damiao, ModeLimits::PositionForce(_pf)) => {
                let (wire_p_max, _, _) = self
                    .active_impedance_wire_range(VendorBinding::Damiao, joint)
                    .unwrap();
                if jl.min_position < -wire_p_max || jl.max_position > wire_p_max {
                    return Err(format!(
                        "control_mode[{joint}]: joint position range [{}, {}] exceeds Damiao wire p_max +/-{wire_p_max}",
                        jl.min_position, jl.max_position
                    ));
                }
                if DamiaoGripperDrive::quantize_vel_raw(jl.max_velocity) == 0 {
                    return Err(format!(
                        "control_mode[{joint}]: joint max_velocity encodes to 0 on the pos-force wire (must quantize to >= 0.01 rad/s)"
                    ));
                }
                let gp = self.gripper_policy.as_ref().ok_or_else(|| {
                    format!(
                        "control_mode[{joint}] (position_force) on gripper joint requires gripper_policy (closed_rad / open_rad)"
                    )
                })?;
                if DamiaoGripperDrive::quantize_vel_raw_f64(gp.close_speed()) == 0 {
                    return Err(
                        "gripper_policy.close_speed_rad_s encodes to 0 on the pos-force wire (must quantize to >= 0.01 rad/s)"
                            .into(),
                    );
                }
            }
            (VendorBinding::Feetech, ModeLimits::Position(pos)) => {
                if pos.interval_ms.is_some() || pos.power_limit.is_some() {
                    return Err(format!(
                        "control_mode[{joint}]: Feetech position mode does not support interval_ms / power_limit"
                    ));
                }
                if let Some([lo, hi]) = pos.step_range {
                    if let Some((exp_min, exp_max)) = self.feetech_joint_explicit_step_bounds(joint)
                    {
                        if exp_min.is_some_and(|m| m != lo) || exp_max.is_some_and(|m| m != hi) {
                            return Err(format!(
                                "control_mode[{joint}] position.step_range [{lo}, {hi}] conflicts with feetech.joints[{joint}]"
                            ));
                        }
                    }
                }
            }
            (VendorBinding::RobStride, ModeLimits::Impedance(imp)) => {
                let (wire_p, wire_v, wire_t) = self
                    .active_impedance_wire_range(VendorBinding::RobStride, joint)
                    .unwrap();
                if (wire_p - imp.p_max).abs() > 1e-5
                    || (wire_v - imp.v_max).abs() > 1e-5
                    || (wire_t - imp.t_max).abs() > 1e-5
                {
                    return Err(format!(
                        "control_mode[{joint}] impedance wire range ({}/{}/{}) conflicts with active RobStride wire mapping ({wire_p}/{wire_v}/{wire_t})",
                        imp.p_max, imp.v_max, imp.t_max
                    ));
                }
                if jl.min_position < -imp.p_max || jl.max_position > imp.p_max {
                    return Err(format!(
                        "control_mode[{joint}]: joint position range [{}, {}] exceeds RobStride wire p_max +/-{}",
                        jl.min_position, jl.max_position, imp.p_max
                    ));
                }
                self.validate_impedance_policy_compatibility(joint, imp)?;
            }
            (VendorBinding::FashionStar, ModeLimits::Position(pos)) => {
                if pos.step_range.is_some() {
                    return Err(format!(
                        "control_mode[{joint}]: FashionStar position mode uses 0.1 deg angle commands and does not accept step_range"
                    ));
                }
                let max_rad = crate::motor_mode::FASHIONSTAR_MAX_ANGLE_RAD;
                if jl.min_position < -max_rad || jl.max_position > max_rad {
                    return Err(format!(
                        "control_mode[{joint}]: joint position range [{}, {}] exceeds FashionStar wire angle span +/-{max_rad:.3} rad",
                        jl.min_position, jl.max_position
                    ));
                }
            }
            _ => {
                return Err(format!(
                    "control_mode[{joint}]: unsupported vendor/mode pair ('{vendor}', '{mode}')"
                ));
            }
        }
        Ok(())
    }

    /// Vendor-agnostic shape and cross-section checks for `control_mode`.
    /// Returns the single explicit `VendorBinding` if any joint declared one.
    fn validate_control_mode_shape(&self) -> Result<Option<VendorBinding>, String> {
        let Some(entries) = &self.control_mode else {
            return Ok(None);
        };
        if entries.len() != self.dof {
            return Err(format!(
                "control_mode length {} does not match DOF {}",
                entries.len(),
                self.dof
            ));
        }
        if self.limits.len() != self.dof {
            return Err(format!(
                "Limits length {} does not match DOF {}",
                self.limits.len(),
                self.dof
            ));
        }
        let mut explicit_vendor: Option<VendorBinding> = None;
        for (i, slot) in entries.iter().enumerate() {
            let Some(entry) = slot else {
                continue;
            };
            if let Some(v) = entry.vendor {
                if let Some(first) = explicit_vendor {
                    if first != v {
                        return Err(format!(
                            "control_mode[{i}] vendor '{v}' conflicts with earlier vendor '{first}'"
                        ));
                    }
                } else {
                    explicit_vendor = Some(v);
                }
            }

            let resolved_limits = match &entry.limits {
                None if entry.mode == MotorControlMode::Position => {
                    ModeLimits::Position(PositionLimits::default())
                }
                None => {
                    return Err(format!(
                        "control_mode[{i}] ({}): missing mode-specific limits",
                        entry.mode
                    ));
                }
                Some(lim) => {
                    if lim.mode() != entry.mode {
                        return Err(format!(
                            "control_mode[{i}]: limits for '{}' do not match mode '{}'",
                            lim.mode(),
                            entry.mode
                        ));
                    }
                    lim.validate()
                        .map_err(|e| format!("control_mode[{i}]: {e}"))?;
                    lim.clone()
                }
            };

            if let Some(rt_modes) = &entry.runtime_modes {
                for (k, &m) in rt_modes.iter().enumerate() {
                    if m == entry.mode {
                        return Err(format!(
                            "control_mode[{i}]: runtime_modes must not repeat base mode '{m}'"
                        ));
                    }
                    if rt_modes[..k].contains(&m) {
                        return Err(format!(
                            "control_mode[{i}]: duplicate runtime_mode '{m}'"
                        ));
                    }
                }
            }

            // Mode-level shape checks independent of vendor.
            let jl = &self.limits[i];
            match &resolved_limits {
                ModeLimits::Position(_) | ModeLimits::PositionVelocity(_) => {}
                ModeLimits::Impedance(imp) => {
                    if jl.min_position < -imp.p_max || jl.max_position > imp.p_max {
                        return Err(format!(
                            "control_mode[{i}]: joint position range [{}, {}] exceeds impedance wire p_max +/-{}",
                            jl.min_position, jl.max_position, imp.p_max
                        ));
                    }
                    self.validate_impedance_policy_compatibility(i, imp)?;
                }
                ModeLimits::PositionForce(_) => {
                    if DamiaoGripperDrive::quantize_vel_raw(jl.max_velocity) == 0 {
                        return Err(format!(
                            "control_mode[{i}]: joint max_velocity encodes to 0 on the pos-force wire (must quantize to >= 0.01 rad/s)"
                        ));
                    }
                    if Some(i) == self.gripper_index() {
                        let gp = self.gripper_policy.as_ref().ok_or_else(|| {
                            format!(
                                "control_mode[{i}] (position_force) on gripper joint requires gripper_policy (closed_rad / open_rad)"
                            )
                        })?;
                        if DamiaoGripperDrive::quantize_vel_raw_f64(gp.close_speed()) == 0 {
                            return Err(
                                "gripper_policy.close_speed_rad_s encodes to 0 on the pos-force wire (must quantize to >= 0.01 rad/s)"
                                    .into(),
                            );
                        }
                    }
                }
            }

            // Conflict check with legacy `damiao_gripper` block on the gripper joint.
            if Some(i) == self.gripper_index() {
                if let Some(dg) = &self.damiao_gripper {
                    let expected = match dg.mode {
                        DamiaoGripperMode::PosForce => MotorControlMode::PositionForce,
                        DamiaoGripperMode::PosVel => MotorControlMode::PositionVelocity,
                    };
                    if entry.mode != expected {
                        return Err(format!(
                            "control_mode[{i}] ('{}') conflicts with damiao_gripper.mode ('{expected}')",
                            entry.mode
                        ));
                    }
                    if let ModeLimits::PositionForce(pf) = &resolved_limits {
                        let opt_f32_eq = |a: Option<f32>, b: Option<f32>| match (a, b) {
                            (None, None) => true,
                            (Some(x), Some(y)) => (x - y).abs() <= 1e-5,
                            _ => false,
                        };
                        if pf.current_ceiling_raw != dg.current_ceiling_raw
                            || (pf.close_speed_rad_s - dg.close_speed_rad_s).abs() > 1e-5
                            || pf.calibrated != dg.calibrated
                            || !opt_f32_eq(pf.ceiling_torque_nm, dg.ceiling_torque_nm)
                            || pf.torque_reference != dg.torque_reference
                            || pf.feedback_stale_fault_ms != dg.feedback_stale_fault_ms
                        {
                            return Err(format!(
                                "control_mode[{i}] position_force limits conflict with damiao_gripper"
                            ));
                        }
                    }
                }
            }

            // Conflict check with runtime MIT transitions (`gravity_comp` / enabled MIT `joint_tracking`).
            if self.joint_requires_runtime_impedance(i) && entry.mode != MotorControlMode::Impedance
            {
                if let Some(rt) = &entry.runtime_modes {
                    if !rt.contains(&MotorControlMode::Impedance) {
                        return Err(format!(
                            "control_mode[{i}] forbids runtime impedance required by gravity_comp / MIT joint_tracking"
                        ));
                    }
                }
            }
        }
        Ok(explicit_vendor)
    }

    /// Validate optional per-joint `control_mode` declarations.
    ///
    /// Runs vendor-agnostic shape checks and, whenever a vendor binding is explicitly declared
    /// or inferred from profile sections, delegates directly to [`Self::resolve_control_modes`]
    /// so `validate()` and follower resolution share one authoritative rule set.
    pub fn validate_control_mode(&self) -> Result<(), String> {
        let explicit_vendor = self.validate_control_mode_shape()?;
        if self.control_mode.is_none() {
            return Ok(());
        }
        if let Some(vendor) = explicit_vendor.or_else(|| self.infer_vendor_binding()) {
            let _ = self.resolve_control_modes(vendor)?;
        }
        Ok(())
    }

    /// Translate one omitted joint slot from legacy profile settings for `vendor`, preserving
    /// legacy acceptance rules.
    fn resolve_omitted_joint(
        &self,
        vendor: VendorBinding,
        joint: usize,
    ) -> Result<ResolvedJointMode, String> {
        let gripper_idx = self.gripper_index();
        let (mode, limits, runtime_modes) = match vendor {
            VendorBinding::Damiao => {
                if Some(joint) == gripper_idx {
                    if let Some(dg) = &self.damiao_gripper {
                        match dg.mode {
                            DamiaoGripperMode::PosForce => {
                                let gp = self.gripper_policy.as_ref().ok_or_else(|| {
                                    "damiao_gripper pos_force needs gripper_policy (closed_rad / open_rad)"
                                        .to_string()
                                })?;
                                if DamiaoGripperDrive::quantize_vel_raw_f64(gp.close_speed()) == 0 {
                                    return Err(
                                        "gripper_policy.close_speed_rad_s encodes to 0 on the pos-force wire (must quantize to >= 0.01 rad/s)"
                                            .into(),
                                    );
                                }
                                (
                                    MotorControlMode::PositionForce,
                                    ModeLimits::PositionForce(
                                        PositionForceLimits::from_damiao_gripper(dg),
                                    ),
                                    Vec::new(),
                                )
                            }
                            DamiaoGripperMode::PosVel => (
                                MotorControlMode::PositionVelocity,
                                ModeLimits::PositionVelocity(
                                    PositionVelocityLimits::damiao_default(),
                                ),
                                Vec::new(),
                            ),
                        }
                    } else {
                        (
                            MotorControlMode::PositionVelocity,
                            ModeLimits::PositionVelocity(PositionVelocityLimits::damiao_default()),
                            Vec::new(),
                        )
                    }
                } else {
                    let rt = if self.joint_requires_runtime_impedance(joint) {
                        vec![MotorControlMode::Impedance]
                    } else {
                        Vec::new()
                    };
                    (
                        MotorControlMode::PositionVelocity,
                        ModeLimits::PositionVelocity(PositionVelocityLimits::damiao_default()),
                        rt,
                    )
                }
            }
            VendorBinding::Feetech => (
                MotorControlMode::Position,
                ModeLimits::Position(PositionLimits {
                    step_range: self.feetech_joint_step_range(joint),
                    ..PositionLimits::default()
                }),
                Vec::new(),
            ),
            VendorBinding::RobStride => (
                MotorControlMode::Impedance,
                ModeLimits::Impedance(ImpedanceLimits::robstride_default()),
                Vec::new(),
            ),
            VendorBinding::FashionStar => (
                MotorControlMode::Position,
                ModeLimits::Position(PositionLimits {
                    interval_ms: Some(crate::motor_mode::FASHIONSTAR_DEFAULT_INTERVAL_MS),
                    power_limit: Some(crate::motor_mode::FASHIONSTAR_DEFAULT_POWER),
                    ..PositionLimits::default()
                }),
                Vec::new(),
            ),
        };
        let runtime_impedance_limits =
            self.runtime_impedance_limits_for(vendor, joint, &runtime_modes);
        Ok(ResolvedJointMode {
            joint,
            vendor: Some(vendor),
            mode,
            limits,
            runtime_modes,
            runtime_impedance_limits,
        })
    }

    /// Resolve every joint's effective control mode, typed limits, and permitted runtime
    /// transitions for a concrete hardware [`VendorBinding`].
    ///
    /// Omitted `control_mode` entries translate to today's exact behaviour and acceptance rules
    /// (including `damiao_gripper` compatibility and existing runtime MIT transitions).
    /// Explicit entries are checked by the single-authority vendor validator.
    pub fn resolve_control_modes(
        &self,
        vendor: VendorBinding,
    ) -> Result<Vec<ResolvedJointMode>, String> {
        let explicit_vendor = self.validate_control_mode_shape()?;
        if let Some(v) = explicit_vendor {
            if v != vendor {
                return Err(format!(
                    "control_mode vendor '{v}' conflicts with follower vendor '{vendor}'"
                ));
            }
        }
        self.validate_vendor_profile_sections(vendor)?;
        if self.limits.len() != self.dof {
            return Err(format!(
                "Limits length {} does not match DOF {}",
                self.limits.len(),
                self.dof
            ));
        }

        let mut out = Vec::with_capacity(self.dof);

        for i in 0..self.dof {
            let explicit = self
                .control_mode
                .as_ref()
                .and_then(|v| v.get(i))
                .and_then(|e| e.as_ref());

            if let Some(entry) = explicit {
                let lim = match (&entry.limits, vendor) {
                    (Some(ModeLimits::Position(p)), VendorBinding::Feetech) => {
                        let mut merged = p.clone();
                        if merged.step_range.is_none() {
                            merged.step_range = self.feetech_joint_step_range(i);
                        }
                        ModeLimits::Position(merged)
                    }
                    (Some(ModeLimits::Position(p)), VendorBinding::FashionStar) => {
                        let mut merged = p.clone();
                        if merged.interval_ms.is_none() {
                            merged.interval_ms =
                                Some(crate::motor_mode::FASHIONSTAR_DEFAULT_INTERVAL_MS);
                        }
                        if merged.power_limit.is_none() {
                            merged.power_limit =
                                Some(crate::motor_mode::FASHIONSTAR_DEFAULT_POWER);
                        }
                        ModeLimits::Position(merged)
                    }
                    (Some(l), _) => l.clone(),
                    (None, VendorBinding::Feetech) if entry.mode == MotorControlMode::Position => {
                        ModeLimits::Position(PositionLimits {
                            step_range: self.feetech_joint_step_range(i),
                            ..PositionLimits::default()
                        })
                    }
                    (None, VendorBinding::FashionStar)
                        if entry.mode == MotorControlMode::Position =>
                    {
                        ModeLimits::Position(PositionLimits {
                            interval_ms: Some(crate::motor_mode::FASHIONSTAR_DEFAULT_INTERVAL_MS),
                            power_limit: Some(crate::motor_mode::FASHIONSTAR_DEFAULT_POWER),
                            ..PositionLimits::default()
                        })
                    }
                    (None, _) if entry.mode == MotorControlMode::Position => {
                        ModeLimits::Position(PositionLimits::default())
                    }
                    (None, _) => {
                        return Err(format!(
                            "control_mode[{i}] ({}): missing mode-specific limits",
                            entry.mode
                        ));
                    }
                };
                let rt = match &entry.runtime_modes {
                    Some(rt) => rt.clone(),
                    None => {
                        if vendor == VendorBinding::Damiao
                            && entry.mode == MotorControlMode::PositionVelocity
                            && self.joint_requires_runtime_impedance(i)
                        {
                            vec![MotorControlMode::Impedance]
                        } else {
                            Vec::new()
                        }
                    }
                };
                self.validate_explicit_joint_for_vendor(vendor, i, entry.mode, &lim, &rt)?;
                if self.joint_requires_runtime_impedance(i)
                    && entry.mode != MotorControlMode::Impedance
                    && !rt.contains(&MotorControlMode::Impedance)
                {
                    return Err(format!(
                        "control_mode[{i}]: vendor '{vendor}' / mode '{}' cannot satisfy runtime impedance required by gravity_comp / MIT joint_tracking",
                        entry.mode
                    ));
                }
                let runtime_impedance_limits =
                    self.runtime_impedance_limits_for(vendor, i, &rt);
                out.push(ResolvedJointMode {
                    joint: i,
                    vendor: Some(vendor),
                    mode: entry.mode,
                    limits: lim,
                    runtime_modes: rt,
                    runtime_impedance_limits,
                });
            } else {
                out.push(self.resolve_omitted_joint(vendor, i)?);
            }
        }

        Ok(out)
    }

    /// Resolve per-joint control modes for a CLI `--follower` name (`rebot-dm`, `rebot-rs`,
    /// `so-arm101`, `so-arm100`, `fashionstar`, `star-arm102`, `sim`, `mock`).
    pub fn resolve_control_modes_for_follower(
        &self,
        follower: &str,
    ) -> Result<Vec<ResolvedJointMode>, String> {
        if let Some(vendor) = VendorBinding::from_follower_name(follower) {
            return self.resolve_control_modes(vendor);
        }
        if matches!(follower, "sim" | "mock") {
            let explicit_vendor = self.validate_control_mode_shape()?;
            if let Some(vendor) = explicit_vendor.or_else(|| self.infer_vendor_binding()) {
                return self.resolve_control_modes(vendor);
            }
            let mut out = Vec::with_capacity(self.dof);
            for i in 0..self.dof {
                let explicit = self
                    .control_mode
                    .as_ref()
                    .and_then(|v| v.get(i))
                    .and_then(|e| e.as_ref());
                if let Some(entry) = explicit {
                    let lim = match &entry.limits {
                        Some(l) => l.clone(),
                        None => ModeLimits::Position(PositionLimits::default()),
                    };
                    out.push(ResolvedJointMode {
                        joint: i,
                        vendor: entry.vendor,
                        mode: entry.mode,
                        limits: lim,
                        runtime_modes: entry.runtime_modes.clone().unwrap_or_default(),
                        runtime_impedance_limits: None,
                    });
                } else {
                    out.push(ResolvedJointMode {
                        joint: i,
                        vendor: None,
                        mode: MotorControlMode::Position,
                        limits: ModeLimits::Position(PositionLimits::default()),
                        runtime_modes: Vec::new(),
                        runtime_impedance_limits: None,
                    });
                }
            }
            return Ok(out);
        }
        Err(format!("Unknown follower model: {follower}"))
    }

    /// Step-1 startup guard called before opening any hardware transport (`src/main.rs`).
    ///
    /// Resolves per-joint control modes for `follower` and, when a real hardware follower has
    /// explicit `control_mode` entries, verifies that every declared mode, limit set, and
    /// runtime transition matches what the unmigrated Step-1 driver path will actually apply.
    /// Fails closed if an explicit declaration would be silently ignored on hardware (e.g.
    /// removing `damiao_gripper` and declaring J7 `position_force` solely in `control_mode`).
    pub fn validate_step1_driver_parity(
        &self,
        follower: &str,
    ) -> Result<Vec<ResolvedJointMode>, String> {
        let declared = self.resolve_control_modes_for_follower(follower)?;
        let Some(vendor) = VendorBinding::from_follower_name(follower) else {
            return Ok(declared);
        };
        let has_explicit = self
            .control_mode
            .as_ref()
            .is_some_and(|entries| entries.iter().any(Option::is_some));
        if !has_explicit {
            return Ok(declared);
        }
        for (i, resolved) in declared.iter().enumerate() {
            let active = self.resolve_omitted_joint(vendor, i)?;
            if resolved != &active {
                return Err(format!(
                    "control_mode[{i}] diverges from Step-1 driver path for follower '{follower}' (declared '{}', active driver applies '{}'); Step 1 has not migrated src/driver.rs TX paths yet — keep legacy profile fields matching or omit control_mode until Step 2",
                    resolved.mode, active.mode
                ));
            }
        }
        Ok(declared)
    }

    /// Joint limits for servo stepping: `max_velocity` capped at `servo_max_velocity`
    /// per joint, except the 7-DOF gripper (index 6).
    pub fn servo_limits(&self) -> Vec<JointLimit> {
        let mut limits = Vec::with_capacity(self.limits.len());
        self.servo_limits_into(&mut limits);
        limits
    }

    /// [`Self::servo_limits`] into `limits`, reusing its memory.
    pub fn servo_limits_into(&self, limits: &mut Vec<JointLimit>) {
        limits.truncate(self.limits.len());
        let have = limits.len();
        for (d, s) in limits.iter_mut().zip(&self.limits) { d.clone_from(s); }
        limits.extend(self.limits[have..].iter().cloned());
        if let Some(caps) = &self.servo_max_velocity {
            let gripper_idx = self.gripper_index();
            for (i, (limit, cap)) in limits.iter_mut().zip(caps).enumerate() {
                if let (Some(cap), false) = (cap, Some(i) == gripper_idx) {
                    limit.max_velocity = limit.max_velocity.min(*cap);
                }
            }
        }
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

        // the official convention: the folded rest pose is URDF 0 on every joint, gripper closed
        let zero_reference = ZeroReference::rebot_b601();
        let home_positions = zero_reference.positions.iter().map(|v| *v as f32).collect();

        Self {
            profile_id: Some("rebot_b601dm".to_string()),
            profile_version: ARM_PROFILE_VERSION,
            dof,
            joint_names,
            limits,
            home_positions,
            lease_timeout_ms: 2500,
            max_temperature: 70.0,
            control_frequency_hz: 50,
            resting_pose: Some(RestingPosePolicy::rebot_dm()),
            rest_pose_tolerance_rad: DEFAULT_REST_POSE_TOLERANCE_RAD,
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
            #[cfg(not(target_arch = "wasm32"))] vision_detectors: None,
            gravity_comp: Some(GravityCompPolicy::rebot_dm()),
            joint_tracking: Some(JointTrackingConfig::rebot_dm()),
            damiao_ranges: Some(vec![
                DamiaoRange::REBOT_DM_J1_J3, DamiaoRange::REBOT_DM_J1_J3, DamiaoRange::REBOT_DM_J1_J3,
                DamiaoRange::REBOT_DM_J4_J7, DamiaoRange::REBOT_DM_J4_J7, DamiaoRange::REBOT_DM_J4_J7,
                DamiaoRange::REBOT_DM_J4_J7,
            ]),
            damiao_gripper: Some(DamiaoGripperDrive::rebot_dm()),
            control_mode: None,
            tx_budget_ms: Some(10),
            tx_stall_ticks: None,
            speed_cap: None,
            // J1 backlash 0.28-0.30 deg (camera, real arm 2026-11-18): every saved-pose move and the
            // final grasp / place descent end from the same side (src/backlash.rs)
            backlash_approach: Some(crate::backlash::BacklashApproach::default()),
            // Free-motion residuals on the real arm (B601-DM, 2026-11-21): 2786 samples over
            // pose-to-pose moves at speed 0.3/0.6, ±0.3 rad on every joint in 1 s and 2 s,
            // linear moves at 0.04/0.08 m/s and J6 sweeps, outside the folded rest pose. Max
            // |residual| J1 1.31, J2 1.75, J3 2.61, J4 0.41, J5 0.26, J6 0.29 N·m: the 2026-11-11
            // values for J2 (1.19) and J6 (0.28) were exceeded by normal motion. Threshold =
            // max(previous, 1.5 x measured max). Still off by default: contact detection is not
            // validated with real contacts and faster motions (kits/recamera-b601dm/README.md)
            collision: Some(crate::collision::CollisionPolicy {
                enabled: false,
                thresholds_nm: [2.0, 2.65, 3.9, 1.43, 0.87, 0.45],
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
            zero_pose: Some(ZeroPoseDoc::rebot_b601()),
            zero_reference: Some(zero_reference),
            calibration_tool: Some(CalibrationToolSpec::b601dm_gripper()),
            active_tool: crate::tool::ActiveToolInfo::default(),
            tcp_offset: [0.0; 3],
            base_check_points_end_m: Vec::new(),
            permanent_tool_check_points_end_m: Vec::new(),
            transient_payload_check_points_end_m: Vec::new(),
            physical_contact_end_m: None,
            contact_opening_m: None,
            dynamics_qualified: true,
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
    /// The stall latch was set by the torque cap (`gripper_policy.max_close_torque_nm`) and holds.
    #[serde(default)]
    pub force_limited: bool,
    /// Torque cap trips since start.
    #[serde(default)]
    pub force_limit_trips: u64,
    /// Learned free-motion torque of the jaw (`crate::gripper_baseline`); absent from older armd.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub baseline: Option<GripperBaselineObs>,
}

/// `observation.gripper.baseline`: state of the learned free-motion torque baseline.
/// Stored inline (`HolderLabel`) so `fill_observation` does zero heap allocations per tick.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct GripperBaselineObs {
    /// `"unlearned"`, `"learning"` or `"learned"`.
    pub state: HolderLabel,
    /// Unloaded moves learned per direction.
    pub close_moves: u64,
    pub open_moves: u64,
    /// Moves with a position bin rejected as an outlier (not learned).
    pub rejected_moves: u64,
    /// Last grasp contact verdict: torque, threshold, baseline and margin (|τ| − baseline).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub last_contact: Option<crate::gripper_baseline::ContactInfo>,
}

/// A short label stored inline (at most [`HolderLabel::CAPACITY`] bytes): a lease holder's
/// label (the limit `acquire_lease` enforces), a policy stop reason, a gravity-compensation
/// phase. Copying, setting and clearing it never allocate or free.
/// Serializes as a JSON string; a longer string is cut at a character boundary.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct HolderLabel {
    len: u8,
    buf: [u8; HolderLabel::CAPACITY],
}

impl HolderLabel {
    pub const CAPACITY: usize = 128;

    /// `s`, cut at the last character boundary within [`Self::CAPACITY`] bytes.
    pub fn new(s: &str) -> Self {
        let mut n = s.len().min(Self::CAPACITY);
        while !s.is_char_boundary(n) { n -= 1; }
        let mut buf = [0u8; Self::CAPACITY];
        buf[..n].copy_from_slice(&s.as_bytes()[..n]);
        Self { len: n as u8, buf }
    }

    /// Exact inline copy when `s` fits within [`Self::CAPACITY`] bytes; `None` if longer
    /// (fail-closed for identity comparisons where truncation could alias two IDs).
    pub fn try_new(s: &str) -> Option<Self> {
        if s.len() > Self::CAPACITY {
            return None;
        }
        let mut buf = [0u8; Self::CAPACITY];
        buf[..s.len()].copy_from_slice(s.as_bytes());
        Some(Self { len: s.len() as u8, buf })
    }

    pub fn as_str(&self) -> &str {
        // SAFETY-free: `new` copies a prefix ending at a char boundary of a `str`
        std::str::from_utf8(&self.buf[..usize::from(self.len)]).unwrap_or_default()
    }
}

impl Default for HolderLabel {
    fn default() -> Self { Self { len: 0, buf: [0; Self::CAPACITY] } }
}

impl std::ops::Deref for HolderLabel {
    type Target = str;
    fn deref(&self) -> &str { self.as_str() }
}

impl From<&str> for HolderLabel {
    fn from(s: &str) -> Self { Self::new(s) }
}

impl From<String> for HolderLabel {
    fn from(s: String) -> Self { Self::new(&s) }
}

impl From<&String> for HolderLabel {
    fn from(s: &String) -> Self { Self::new(s) }
}

impl AsRef<str> for HolderLabel {
    fn as_ref(&self) -> &str { self.as_str() }
}

impl std::fmt::Debug for HolderLabel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result { std::fmt::Debug::fmt(self.as_str(), f) }
}

impl std::fmt::Display for HolderLabel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result { f.write_str(self.as_str()) }
}

impl PartialEq<str> for HolderLabel {
    fn eq(&self, other: &str) -> bool { self.as_str() == other }
}

impl PartialEq<&str> for HolderLabel {
    fn eq(&self, other: &&str) -> bool { self.as_str() == *other }
}

impl PartialEq<HolderLabel> for &str {
    fn eq(&self, other: &HolderLabel) -> bool { *self == other.as_str() }
}

impl PartialEq<String> for HolderLabel {
    fn eq(&self, other: &String) -> bool { self.as_str() == other.as_str() }
}

impl PartialEq<HolderLabel> for String {
    fn eq(&self, other: &HolderLabel) -> bool { self.as_str() == other.as_str() }
}

impl Serialize for HolderLabel {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> { s.serialize_str(self.as_str()) }
}

impl<'de> Deserialize<'de> for HolderLabel {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let s = String::deserialize(d)?;
        Ok(Self::new(&s))
    }
}

/// Reusable heap `String` buffer parked on an [`ArmObservation`] slot while `safety_state`
/// does not own a string (`Idle`, `Armed`, `EStop`, `ThermalTrip`, `BusLoss`, `WatchdogTimeout`).
/// Ignored by `PartialEq` and `serde`; cloned empty for off-thread snapshots so only RT
/// triple-buffer slots carry preallocated capacity.
#[derive(Debug, Default)]
#[cfg_attr(target_arch = "wasm32", allow(dead_code))]
pub(crate) struct SafetyTextSpare(String);

impl PartialEq for SafetyTextSpare {
    fn eq(&self, _other: &Self) -> bool { true }
}

impl Clone for SafetyTextSpare {
    fn clone(&self) -> Self { Self(String::new()) }
    fn clone_from(&mut self, _source: &Self) {}
}

/// Reusable preallocated [`crate::kinematics::WorkspaceViolationDetail`] parked on an
/// [`ArmObservation`] slot while `workspace_violation` is `None`, so RT `fill_observation`
/// transitions perform 0 heap allocations or frees. Ignored by `PartialEq` and `serde`.
#[derive(Debug, Default)]
#[cfg_attr(target_arch = "wasm32", allow(dead_code))]
pub(crate) struct WorkspaceViolationSpare(Option<crate::kinematics::WorkspaceViolationDetail>);

impl PartialEq for WorkspaceViolationSpare {
    fn eq(&self, _other: &Self) -> bool { true }
}

impl Clone for WorkspaceViolationSpare {
    fn clone(&self) -> Self { Self(None) }
    fn clone_from(&mut self, _source: &Self) {}
}

/// Full arm telemetry / observation
#[derive(Debug, Serialize, Deserialize, PartialEq)]
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
    /// none); null when no lease is held. Stored inline ([`HolderLabel`]): the tick sets and
    /// clears it without touching the allocator.
    #[serde(default)]
    pub lease_holder: Option<HolderLabel>,
    /// Limit that is shaping servo motion right now (speed / lead cap, joint limit,
    /// workspace, blocked hold); null when none applies. See docs/control_safety_contract.md.
    #[serde(default)]
    pub motion_limit: Option<MotionLimit>,
    /// Structured workspace violation details when `motion_limit.reason == Workspace`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub workspace_violation: Option<crate::kinematics::WorkspaceViolationDetail>,
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
    /// Active or most recent policy-runtime run; omitted when none has run or feature disabled.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub policy: Option<PolicyObservation>,
    /// Reason the lease watchdog held position instead of starting a park (for example
    /// `"configuration reconciliation active"` or `"configuration hold active"`),
    /// cleared on fault clear or subsequent park; omitted when null.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub watchdog_park_blocked_reason: Option<HolderLabel>,
    /// Monotonic count of deferred safety log events dropped on overflow (never reset); omitted while zero.
    #[serde(default, skip_serializing_if = "is_zero_u64")]
    pub safety_log_dropped: u64,
    /// Preallocated string buffer moved into/out of `safety_state` (`FaultReason::Other` /
    /// `JointLimitExceeded`) so RT `fill_observation` transitions perform 0 heap allocations or drops.
    #[serde(skip)]
    pub(crate) safety_text_spare: SafetyTextSpare,
    /// Preallocated [`crate::kinematics::WorkspaceViolationDetail`] moved into/out of
    /// `workspace_violation` so RT `fill_observation` transitions perform 0 heap ops.
    #[serde(skip)]
    pub(crate) workspace_violation_spare: WorkspaceViolationSpare,
}

fn is_zero_u64(v: &u64) -> bool { *v == 0 }

impl ArmObservation {
    /// An observation with every field empty (filled by `SupervisorInner::fill_observation`).
    pub fn blank() -> Self {
        Self {
            timestamp: 0.0, positions: Vec::new(), velocities: Vec::new(), torques: Vec::new(), temperatures: Vec::new(),
            is_torqued: false, safety_state: SafetyState::Idle, epoch_id: 0, gravity_compensated: false,
            gravity_torques: Vec::new(), driver_capabilities: DriverCapabilities::default(), enable_state: EnableState::default(),
            feedback_age_ms: Vec::new(), disable_pending: false, feedback_valid: false, commanded_positions: Vec::new(),
            action: None, previous_action: None, control_mode: ControlMode::default(), gravity_comp: GravityCompObservation::default(),
            driver_range_check: None, lease_holder: None, motion_limit: None, workspace_violation: None, motor_errors: Vec::new(),
            fault: None, residual_torques: None, collision: None, power: None, gripper_zero: None, tcp: None, gripper: None,
            loads: None, policy: None, watchdog_park_blocked_reason: None, safety_log_dropped: 0,
            safety_text_spare: SafetyTextSpare(String::new()),
            workspace_violation_spare: WorkspaceViolationSpare(None),
        }
    }

    /// Blank observation for an RT triple-buffer slot with preallocated `safety_state` text capacity.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn blank_rt() -> Self {
        let mut obs = Self::blank();
        obs.ensure_rt_safety_text_spare();
        obs
    }

    /// Ensures `safety_text_spare` (or the owned `String` already inside `safety_state`) holds at
    /// least `SAFETY_LOG_TEXT_CAPACITY` bytes so RT `set_safety_state` transitions never allocate
    /// or drop a spare buffer (needed after cloning an `ArmObservation`, because
    /// `SafetyTextSpare::clone` intentionally returns an empty `String`).
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn ensure_rt_safety_text_spare(&mut self) {
        let needed = crate::safety::SAFETY_LOG_TEXT_CAPACITY;
        match &mut self.safety_state {
            SafetyState::Fault(crate::safety::FaultReason::Other(s))
            | SafetyState::Fault(crate::safety::FaultReason::JointLimitExceeded { joint_name: s, .. }) => {
                if s.capacity() < needed {
                    s.reserve(needed - s.len());
                }
            }
            _ => {
                if self.safety_text_spare.0.capacity() < needed {
                    self.safety_text_spare.0.reserve(needed - self.safety_text_spare.0.len());
                }
            }
        }
        if self.gravity_comp.tau_ff.capacity() < 6 {
            self.gravity_comp.tau_ff.reserve(6 - self.gravity_comp.tau_ff.len());
        }
        if self.gravity_comp.damping_kd.capacity() < 6 {
            self.gravity_comp.damping_kd.reserve(6 - self.gravity_comp.damping_kd.len());
        }
        if self.workspace_violation.is_none() && self.workspace_violation_spare.0.is_none() {
            self.workspace_violation_spare.0 =
                Some(crate::kinematics::WorkspaceViolationDetail::preallocated_rt());
        }
    }

    /// Updates `self.workspace_violation` reusing preallocated buffers between
    /// `self.workspace_violation` and `self.workspace_violation_spare` (0 alloc / 0 free on RT).
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn set_workspace_violation_rt(
        &mut self,
        src: Option<&crate::kinematics::WorkspaceViolationDetail>,
    ) {
        if self.workspace_violation.as_ref() == src {
            return;
        }
        match src {
            Some(s) => {
                if let Some(dst) = self.workspace_violation.as_mut() {
                    dst.copy_from_rt(s);
                } else if let Some(mut spare) = self.workspace_violation_spare.0.take() {
                    spare.copy_from_rt(s);
                    self.workspace_violation = Some(spare);
                } else {
                    self.workspace_violation = Some(s.clone());
                }
            }
            None => {
                if let Some(old) = self.workspace_violation.take() {
                    if self.workspace_violation_spare.0.is_none() {
                        self.workspace_violation_spare.0 = Some(old);
                    }
                }
            }
        }
    }

    /// Updates `self.safety_state` to match `next`, recycling any owned `String` buffer between
    /// `self.safety_state` and `self.safety_text_spare` so transitions perform 0 heap ops on RT slots.
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn set_safety_state(&mut self, next: &SafetyState) {
        use crate::safety::FaultReason;
        if self.safety_state == *next {
            return;
        }
        let prev = std::mem::replace(&mut self.safety_state, SafetyState::Idle);
        let existing_buf = match prev {
            SafetyState::Fault(FaultReason::Other(s)) => Some(s),
            SafetyState::Fault(FaultReason::JointLimitExceeded { joint_name, .. }) => Some(joint_name),
            _ => None,
        };
        match next {
            SafetyState::Fault(FaultReason::Other(src)) => {
                let mut buf = existing_buf.unwrap_or_else(|| std::mem::take(&mut self.safety_text_spare.0));
                buf.clear();
                buf.push_str(src);
                self.safety_state = SafetyState::Fault(FaultReason::Other(buf));
            }
            SafetyState::Fault(FaultReason::JointLimitExceeded { joint_idx, joint_name, value, min, max }) => {
                let mut buf = existing_buf.unwrap_or_else(|| std::mem::take(&mut self.safety_text_spare.0));
                buf.clear();
                buf.push_str(joint_name);
                self.safety_state = SafetyState::Fault(FaultReason::JointLimitExceeded {
                    joint_idx: *joint_idx,
                    joint_name: buf,
                    value: *value,
                    min: *min,
                    max: *max,
                });
            }
            other => {
                if let Some(mut buf) = existing_buf {
                    buf.clear();
                    if self.safety_text_spare.0.capacity() == 0 {
                        self.safety_text_spare.0 = buf;
                    }
                }
                self.safety_state = match other {
                    SafetyState::Booting => SafetyState::Booting,
                    SafetyState::Idle => SafetyState::Idle,
                    SafetyState::Armed => SafetyState::Armed,
                    SafetyState::Moving => SafetyState::Moving,
                    SafetyState::EStop => SafetyState::EStop,
                    SafetyState::Fault(FaultReason::BusLoss) => SafetyState::Fault(FaultReason::BusLoss),
                    SafetyState::Fault(FaultReason::WatchdogTimeout) => SafetyState::Fault(FaultReason::WatchdogTimeout),
                    SafetyState::Fault(FaultReason::CanTxStalled) => SafetyState::Fault(FaultReason::CanTxStalled),
                    SafetyState::Fault(FaultReason::J7FeedbackStale) => SafetyState::Fault(FaultReason::J7FeedbackStale),
                    SafetyState::Fault(FaultReason::ThermalTrip { joint_idx, temperature, limit }) => {
                        SafetyState::Fault(FaultReason::ThermalTrip {
                            joint_idx: *joint_idx,
                            temperature: *temperature,
                            limit: *limit,
                        })
                    }
                    SafetyState::Fault(FaultReason::Other(_) | FaultReason::JointLimitExceeded { .. }) => unreachable!(),
                };
            }
        }
    }
}

crate::fieldwise_clone!(ArmObservation { timestamp, positions, velocities, torques, temperatures, is_torqued, safety_state, epoch_id, gravity_compensated, gravity_torques, driver_capabilities, enable_state, feedback_age_ms, disable_pending, feedback_valid, commanded_positions, action, previous_action, control_mode, gravity_comp, driver_range_check, lease_holder, motion_limit, workspace_violation, motor_errors, fault, residual_torques, collision, power, gripper_zero, tcp, gripper, loads, policy, watchdog_park_blocked_reason, safety_log_dropped, safety_text_spare, workspace_violation_spare });

/// Policy runtime execution state published in `observation.policy`.
#[derive(Debug, Default, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct PolicyObservation {
    pub active: bool,
    pub action_id: u64,
    pub policy_id: String,
    pub artifact_digest: String,
    pub backend: String,
    pub mode: String,
    pub steps: u64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_input_age_ms: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub last_vision_age_ms: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vision_frame_id: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_shadow_target: Option<Vec<f32>>,
    /// Inline (no allocation or free when the tick sets or clears it).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stop_reason: Option<HolderLabel>,
}

crate::fieldwise_clone!(PolicyObservation { active, action_id, policy_id, artifact_digest, backend, mode, steps, last_input_age_ms, last_vision_age_ms, vision_frame_id, last_shadow_target, stop_reason });

impl PolicyObservation {
    /// Preallocated empty `PolicyObservation` reused by `armd-control` spares (0 RT alloc/free).
    pub fn preallocated_rt() -> Self {
        Self {
            active: false,
            action_id: 0,
            policy_id: String::with_capacity(HolderLabel::CAPACITY),
            artifact_digest: String::with_capacity(HolderLabel::CAPACITY),
            backend: String::with_capacity(64),
            mode: String::with_capacity(16),
            steps: 0,
            last_input_age_ms: None,
            last_vision_age_ms: None,
            vision_frame_id: None,
            last_shadow_target: None,
            stop_reason: None,
        }
    }
}

/// Gripper zero state published in `observation.gripper_zero`.
#[derive(Debug, Serialize, Deserialize, PartialEq)]
pub struct GripperZeroObservation {
    /// `ok`, `out_of_range`, `checked`, or `calib_invalid`.
    pub state: String,
    pub j7: f32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub offset_rad: Option<f32>,
}

crate::fieldwise_clone!(GripperZeroObservation { state, j7, offset_rad });

/// A motor fault code reported in the motor's feedback (`observation.motor_errors`).
#[derive(Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct MotorError {
    pub code: u8,
    pub name: std::borrow::Cow<'static, str>,
}

crate::fieldwise_clone!(MotorError { code, name });

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
    /// Gripper torque cap (`gripper_policy.max_close_torque_nm`): the jaw stopped closing and
    /// holds its measured position. Targets closing further are held.
    GripperForce,
}

impl MotionLimitReason {
    /// Higher wins when several limits apply at once.
    pub fn priority(self) -> u8 {
        match self {
            Self::Speed => 0,
            Self::Lead => 1,
            Self::Workspace => 2,
            Self::JointLimit => 3,
            Self::Blocked | Self::GripperForce => 4,
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
/// Inline storage keeps observation publishing (`fill_observation`) allocation-free on the control thread.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct ModeReason {
    pub code: crate::safety::BoundedText<48>,
    pub message: crate::safety::BoundedText<256>,
}

/// Why the last gravity-compensation session ended.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct GravityCompExit {
    pub code: HolderLabel,
    pub message: HolderLabel,
    /// Observation timestamp of the exit (s).
    pub at: f64,
}

/// Gravity-compensation state published with every observation.
#[derive(Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct GravityCompObservation {
    /// The profile and driver support the mode.
    pub supported: bool,
    /// Why entering is refused right now (None when allowed or already active).
    pub blocker: Option<ModeReason>,
    /// `ramp_in`, `active`, `exiting` or `exit_failed`; None in position mode. Inline (no
    /// allocation or free when the tick sets or clears it).
    pub phase: Option<HolderLabel>,
    /// Feed-forward fade-in progress, 0..1.
    pub ramp: f32,
    pub scale: f32,
    pub damping_kd: Vec<f32>,
    /// Feed-forward torque sent to J1-J6 this tick (N·m).
    pub tau_ff: Vec<f32>,
    pub last_exit: Option<GravityCompExit>,
}

crate::fieldwise_clone!(GravityCompObservation { supported, blocker, phase, ramp, scale, damping_kd, tau_ff, last_exit });

/// Field description in the observation schema
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SchemaField {
    pub name: String,
    pub dtype: String,
    pub shape: Vec<usize>,
    pub unit: Option<String>,
}

/// Version of the capability summary in the observation schema (`capabilities`).
pub const CAPABILITIES_VERSION: u32 = 11;

/// Vision features armd offers.
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(default)]
pub struct VisionCapabilities {
    /// armd proxies a detector's API under `/api/v1/detection/*` (upstream
    /// `ARMD_DETECTOR_URL`). Whether a detector is reachable is not checked here.
    pub detection_proxy: bool,
    /// `POST /api/v1/vision/detector` (operator): switch the detector between the default
    /// `detect` and `segment` models (`vision_detectors` profile section). Whether the provider
    /// can switch models shows in `GET /api/v1/vision/mode` `models`.
    #[serde(default)]
    pub detector_switch: bool,
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
    /// Actions accept `approach: "same_side" | "direct"` (and `move_to` / `move_relative`
    /// `precise`); the profile settings are `schema.backlash_approach` (`src/backlash.rs`).
    #[serde(default)]
    pub backlash_approach: bool,
    #[serde(default)]
    pub auto_yaw: bool,
    /// `move_to`, `move_relative` and `place` accept `yaw: "outward"` (yaw = atan2(y, x) of the
    /// target, the path's yaw follows atan2(y, x); `motion::PathOpts`); same value as
    /// `cartesian`. Absent (false) from older armd.
    #[serde(default)]
    pub yaw_outward: bool,
    /// `move_to` and `move_relative` accept `path: "joint"` (IK of the goal, then the joint-space
    /// quintic of `move_to_pose` with `speed_scale`; `motion::PathMode`); same value as
    /// `cartesian`. Absent (false) from older armd.
    #[serde(default)]
    pub move_path_joint: bool,
    /// `POST /api/v1/grasp/measure` and the WS command of that path (no lease): one depth
    /// measurement at the current pose with the frame's table fit; same value as `grasp`.
    /// Absent (false) from older armd.
    #[serde(default)]
    pub grasp_measure: bool,
    /// Grasp dry run `strategy: "adaptive"` (pose selection over approach directions and
    /// contacts, `pose_preferences`, `evidence`, `budget_ms`): `grasp` on a 6-DOF arm. Absent
    /// (false) from older armd.
    #[serde(default)]
    pub grasp_select: bool,
    /// Live grasp execution with `strategy: "adaptive"` (`grasp_select` phase B): `grasp` on a
    /// 6-DOF arm. Absent (false) from older armd (phase A was dry-run only).
    #[serde(default)]
    pub grasp_select_exec: bool,
    /// Action `/api/v1/action/measure_multiview` (lease, operator session): motion-stereo object
    /// measurement from views around the object (`ActionStatus.measurement`); same value as `grasp`.
    /// Absent (false) from older armd.
    #[serde(default)]
    pub measure_multiview: bool,
    /// Camera calibration wizard: `GET /api/v1/calibration`, WS `/api/v1/calibration/apply`
    /// and `/api/v1/calibration/restore`, `camera_mount.q_end_cam`; same value as `grasp`.
    #[serde(default)]
    pub calibration_apply: bool,
    /// `/api/v1/calibration/apply` accepts `kinematics` (the built-in model with a grasp
    /// setup; URDF chains: false), `schema.kinematics_correction`.
    #[serde(default)]
    pub calibration_kinematics: bool,
    /// Self-service verification (T20): apply fields `tool` / `acceptance` / `table_z_source`,
    /// action `calib_hover` and `GET /api/v1/calibration/hover`; same value as `calibration_apply`
    /// on the built-in model.
    #[serde(default)]
    pub calibration_verify: bool,
    /// Calibration tools of this arm (driver + profile): which ones it supports, their joints and
    /// requirements. Filled in by the supervisor's schema (it depends on the driver); None in a
    /// schema built from the profile alone. Present from capabilities version 10.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub calibration_tools: Option<CalibrationTools>,
    /// Teach v1 (episode store, recorder ownership, sequences): `{version: 1, episodes,
    /// sequences}`; `episodes` is false while episode storage is off. Present from
    /// capabilities version 11; absent (null) from older armd.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub teach: Option<TeachCapabilities>,
    pub vision: VisionCapabilities,
    /// On-device policy runtime capabilities; omitted unless compiled with `policy-runtime`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub policy_runtime: Option<PolicyRuntimeCapabilities>,
}

/// Policy-runtime features compiled into this armd instance.
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(default)]
pub struct PolicyRuntimeCapabilities {
    pub backends: Vec<String>,
    pub inputs: Vec<String>,
    pub modes: Vec<String>,
    /// `speed_gate`: a live start is refused with `POLICY_TOO_SLOW` after a warm-up, and the
    /// start ACK carries `warmup_infer_us` / `too_slow`; `policy_stats`: per-step timing in the
    /// `policy` action status; `policy_delete`: `DELETE /api/v1/sandbox/policies/{id}`.
    pub features: Vec<String>,
}

impl PolicyRuntimeCapabilities {
    #[cfg(all(feature = "policy-runtime", not(target_arch = "wasm32")))]
    pub fn compiled_default() -> Self {
        let mut backends = vec!["mlp".to_string()];
        if cfg!(feature = "policy-tract") {
            backends.push("tract".to_string());
        }
        Self {
            backends,
            inputs: vec![
                "proprioception".to_string(),
                "detector_top1_box_norm_v1".to_string(),
            ],
            modes: vec!["shadow".to_string(), "live_sim".to_string()],
            features: ["speed_gate", "policy_stats", "policy_delete"].map(String::from).to_vec(),
        }
    }
}

/// `capabilities.teach`.
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(default)]
pub struct TeachCapabilities {
    pub version: u32,
    /// `GET /api/v1/episodes`, `/api/v1/datasets`, WS `/api/v1/record/*`, `/api/v1/episodes/*`,
    /// `/api/v1/datasets/*`, `POST /api/v1/episodes/import`.
    pub episodes: bool,
    /// `GET /api/v1/sequences`, WS `/api/v1/sequences/save|delete`, action `run_sequence`.
    pub sequences: bool,
}

/// Version of `capabilities.teach`.
pub const TEACH_VERSION: u32 = 1;

/// `capabilities.calibration_tools` (also in `GET /api/v1/calibration`). A client renders only
/// the tools with `supported: true`; an unsupported tool carries `reason`.
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(default)]
pub struct CalibrationTools {
    /// Camera calibration (intrinsics, hand-eye mount, table height): WS `/api/v1/calibration/apply`.
    pub camera: CalibrationToolInfo,
    /// Kinematic fine calibration (`kinematics` in the apply); `joints` = the joints the
    /// wizard identifies (1-based).
    pub kinematics: CalibrationToolInfo,
    /// Motor zero: WS `/api/v1/calibration/motor_zero`.
    pub motor_zero: MotorZeroTool,
    /// Gripper zero check (`/api/v1/action/gripper_zero_check`) and opening table
    /// (`/api/v1/gripper/calibration`).
    pub gripper: GripperCalibrationTool,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(default)]
pub struct CalibrationToolInfo {
    pub supported: bool,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub joints: Vec<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
}

impl CalibrationToolInfo {
    pub fn new(supported: bool, joints: Vec<usize>, reason: &str) -> Self {
        Self { supported, joints: if supported { joints } else { Vec::new() }, reason: (!supported).then(|| reason.to_string()) }
    }
}

/// What a driver can do for the motor zero tool ([`crate::driver::ArmDriver::motor_zero_support`]).
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(default)]
pub struct MotorZeroSupport {
    pub supported: bool,
    /// `set_current`: the current position of each joint becomes its zero.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub method: Option<String>,
    /// Where the new zero is kept: `motor_flash` (in the motor), `simulator` (the simulated
    /// encoder, until armd restarts), `process` (the mock follower).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub persist: Option<String>,
    /// The persistence above was checked on hardware (power cycle).
    pub persist_verified: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub notes: Vec<String>,
}

impl MotorZeroSupport {
    pub fn unsupported(reason: impl Into<String>) -> Self { Self { reason: Some(reason.into()), ..Default::default() } }

    pub fn set_current(persist: &str, persist_verified: bool, notes: Vec<String>) -> Self {
        Self { supported: true, method: Some("set_current".into()), persist: Some(persist.into()), persist_verified, reason: None, notes }
    }
}

/// Largest |reading| (deg) a joint may have for a motor zero without `force`.
pub const MOTOR_ZERO_MAX_OFFSET_DEG: u32 = 45;
/// |reading| (mrad) every zeroed joint must reach within 1 s after the set-zero command.
pub const MOTOR_ZERO_VERIFY_MRAD: u32 = 10;

/// `calibration_tools.motor_zero`.
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(default)]
pub struct MotorZeroTool {
    #[serde(flatten)]
    pub support: MotorZeroSupport,
    /// Joints that can be zeroed (1-based).
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub joints: Vec<usize>,
    /// Conditions of a real run: `lease`, `torque_off`, `no_action`, `no_fault`.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub requires: Vec<String>,
    /// [`MOTOR_ZERO_MAX_OFFSET_DEG`]: a joint farther from 0 needs `force`.
    pub max_offset_deg: u32,
    /// [`MOTOR_ZERO_VERIFY_MRAD`].
    pub verify_mrad: u32,
    /// Calibrations a zero invalidates: `kinematics` (removed), `camera`, `poses`,
    /// `gripper_calibration` (marked stale).
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub invalidates: Vec<String>,
    /// `method: "motor"` is available (the driver's set-zero command, `supported` above refers to
    /// the tool as a whole).
    pub motor: bool,
    /// `method: "software"` is available (the profile has a `zero_reference`): calibration.json
    /// `joint_zero`, every driver, reversible.
    pub software: bool,
    /// Available methods, the default first.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub methods: Vec<String>,
    /// The profile's description of the zero pose (all motors at 0).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub zero_pose: Option<ZeroPoseDoc>,
}

/// `calibration_tools.gripper`.
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(default)]
pub struct GripperCalibrationTool {
    pub supported: bool,
    /// `/api/v1/action/gripper_zero_check` is available (gripper policy `zero_check_supported`).
    pub zero_check: bool,
    /// `/api/v1/gripper/calibration` (opening table, offset) is available.
    pub calibration: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
}

/// Profile field `zero_pose`: what the arm looks like with every motor at 0 (documentation for
/// the motor zero tool; the 3D view renders the pose itself).
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(default)]
pub struct ZeroPoseDoc {
    /// The whole pose in one sentence.
    pub description: String,
    /// One entry per joint (J1.. in order; may be shorter than `dof`).
    pub joints: Vec<String>,
    /// How to hold the gripper while its zero is set.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gripper: Option<String>,
    /// Where the description comes from.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<String>,
}

impl ZeroPoseDoc {
    /// The B601-DM zero pose = its rest / home pose (official convention, every joint 0).
    pub fn rebot_b601() -> Self {
        Self {
            description: "The folded rest pose (the home and park pose): upper arm level pointing back from the shoulder, forearm folded back over it, gripper level pointing straight forward, jaws opening sideways, gripper fully closed (tool about 0.26 m in front of the base, 0.19 m high). Every joint reads 0 here after the zero.".into(),
            joints: vec![
                "J1 base: arm plane facing forward (+x)".into(),
                "J2 shoulder: upper arm level, pointing back".into(),
                "J3 elbow: forearm folded back over the upper arm".into(),
                "J4 wrist pitch: gripper level".into(),
                "J5 wrist yaw: gripper pointing straight forward".into(),
                "J6 wrist roll: as folded in the rest pose, jaws opening sideways, camera holder on top".into(),
            ],
            gripper: Some("Gripper fully closed".into()),
            source: Some("official convention (Seeed-Projects/reBotArm_control_py: URDF angle = motor angle, zero at the folded rest pose); built-in FK of assets/reBot-DevArm_gripper.urdf; gripper: wiki.seeedstudio.com/rebot_arm_b601_dm_lerobot (zero position = gripper fully closed)".into()),
        }
    }
}

/// Profile field `calibration_tool`: the tool whose fingertip the calibration wizard measures.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct CalibrationToolSpec {
    /// Tool identity (1..64 of A-Z a-z 0-9 _ - .); a stored fingertip measurement is bound to it.
    pub tool_id: String,
    /// Height (m) of the fingertip sticker's bottom edge above the tool's lowest point when
    /// the tool points down; None = unknown.
    #[serde(default)]
    pub edge_to_low_m: Option<f64>,
}

impl CalibrationToolSpec {
    /// B601-DM parallel gripper: sticker edge about 2.3 mm above the finger's lowest point
    /// (two caliper readings on the office unit, 2026-11-20; approximate).
    pub fn b601dm_gripper() -> Self { Self { tool_id: "b601dm_gripper".into(), edge_to_low_m: Some(0.0023) } }

    pub fn validate(&self) -> Result<(), String> {
        if !valid_tool_id(&self.tool_id) { return Err("calibration_tool.tool_id must be 1..64 of A-Z a-z 0-9 _ - .".into()); }
        if self.edge_to_low_m.is_some_and(|e| !e.is_finite() || !(-0.10..=0.10).contains(&e)) {
            return Err("calibration_tool.edge_to_low_m must be in [-0.10, 0.10] m".into());
        }
        Ok(())
    }
}

/// 1..64 of A-Z a-z 0-9 _ - .
pub fn valid_tool_id(id: &str) -> bool {
    !id.is_empty() && id.len() <= 64 && id.chars().all(|c| c.is_ascii_alphanumeric() || matches!(c, '_' | '-' | '.'))
}

/// Profile field `zero_reference`: the pose the joint zero refers to.
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ZeroReference {
    /// One value per joint: URDF angles (rad) for the arm joints, the joint value for others
    /// (gripper closed = 0 on the B601-DM).
    pub positions: Vec<f64>,
    /// Pose name (`rest`).
    #[serde(default = "default_zero_reference_name")]
    pub name: String,
}

fn default_zero_reference_name() -> String { "rest".into() }

impl ZeroReference {
    /// B601-DM: the folded rest pose, URDF 0 on J1..J6, gripper closed (0).
    pub fn rebot_b601() -> Self { Self { positions: vec![0.0; 7], name: "rest".into() } }
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
        #[cfg(all(feature = "policy-runtime", not(target_arch = "wasm32")))]
        let policy_runtime = Some(PolicyRuntimeCapabilities::compiled_default());
        #[cfg(not(all(feature = "policy-runtime", not(target_arch = "wasm32"))))]
        let policy_runtime = None;
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
            backlash_approach: true,
            auto_yaw: false,
            yaw_outward: arm_with_gripper && kin,
            move_path_joint: arm_with_gripper && kin,
            grasp_measure: grasp,
            grasp_select: grasp && config.arm_dof() == 6,
            grasp_select_exec: grasp && config.arm_dof() == 6,
            measure_multiview: grasp,
            calibration_apply: grasp,
            calibration_kinematics: grasp && config.arm_model.chain().is_none(),
            calibration_verify: grasp && config.arm_model.chain().is_none(),
            calibration_tools: None,
            #[cfg(not(target_arch = "wasm32"))]
            teach: Some(TeachCapabilities { version: TEACH_VERSION, episodes: true, sequences: true }),
            #[cfg(target_arch = "wasm32")]
            teach: None,
            vision: VisionCapabilities { detection_proxy: true, detector_switch: cfg!(not(target_arch = "wasm32")), mode_endpoint: VISION_MODE_ENDPOINT.to_string(), ..Default::default() },
            policy_runtime,
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
    /// Same-side final approach settings (`src/backlash.rs`); null = every motion approaches
    /// directly. The cockpit's fine calibration pre-pose follows `delta_deg` and the sign rule.
    #[serde(default)]
    pub backlash_approach: Option<crate::backlash::BacklashApproach>,
    /// Profile home pose (rad, one value per joint); null in older schemas.
    #[serde(default)]
    pub home_positions: Option<Vec<f32>>,
    /// Profile resting-pose policy (tolerances, stillness and support-torque limits); null
    /// when the profile has none and in older schemas.
    #[serde(default)]
    pub resting_pose: Option<RestingPosePolicy>,
    /// Per-joint tolerance (rad) around the rest pose for torque disable while stopped; null in older schemas.
    #[serde(default)]
    pub rest_pose_tolerance_rad: Option<f32>,
    /// Torque-saturation guard policy; null when the profile has none (and in older schemas).
    #[serde(default)]
    pub torque_guard: Option<crate::torque_guard::TorqueGuardInfo>,
    /// armd joint -> kinematic model (URDF) joint, one entry per arm joint: model angle =
    /// `sign * position + offset_rad`. Every pose, FK and IK of the API uses this mapping.
    /// Empty in older schemas (those assumed model = motor for J1..J6).
    #[serde(default)]
    pub model_joint_map: Vec<crate::chain::JointMap>,
    /// Kinematic correction of the built-in chain in effect (calibration.json `kinematics.joints`,
    /// J1..J6, applied after `model_joint_map`); null = the URDF chain. Absent in older schemas.
    #[serde(default)]
    pub kinematics_correction: Option<crate::kinematics::Correction>,
    /// Supply power budget in force; null when the profile has none (and in older schemas).
    #[serde(default)]
    pub power_budget: Option<crate::power::PowerBudget>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[cfg(not(target_arch = "wasm32"))]
    pub feetech_joints: Option<Vec<crate::protocol::feetech::FeetechJointCalib>>,
    /// The profile's `zero_reference` (the pose the joint zero refers to); absent without one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub zero_reference: Option<ZeroReference>,
    /// calibration.json `joint_zero` in effect (src/joint_zero.rs); null = none (offsets 0).
    #[serde(default)]
    pub joint_zero: Option<serde_json::Value>,
    /// Joint zero check against `zero_reference` (`{status: "ok" | "legacy_j6" | "unchecked", message}`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub zero_convention: Option<serde_json::Value>,
    /// Active tool/gripper identity (`schema_version`, `id`, `is_stock`, `config_digest`, ...).
    #[serde(default)]
    pub active_tool: crate::tool::ActiveToolInfo,
    /// Physical resting contact point in `end_link` frame (m) for table datum derivation.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub physical_contact_end_m: Option<[f64; 3]>,
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
            backlash_approach: config.backlash_approach.clone(),
            home_positions: Some(config.home_positions.clone()),
            resting_pose: config.resting_pose.clone(),
            rest_pose_tolerance_rad: Some(config.rest_pose_tolerance_rad),
            torque_guard: crate::torque_guard::TorqueGuardInfo::from_config(config),
            model_joint_map: config.arm_model.joint_map(),
            kinematics_correction: config.arm_model.correction().copied(),
            power_budget: config.power_budget.clone().filter(|p| p.enabled),
            #[cfg(not(target_arch = "wasm32"))]
            feetech_joints: config.feetech.as_ref().map(|f| f.joints.clone()),
            zero_reference: config.zero_reference.clone(),
            joint_zero: None,
            zero_convention: None,
            active_tool: if config.active_tool.is_stock {
                crate::tool::stock_active_tool_info(config)
            } else {
                config.active_tool.clone()
            },
            physical_contact_end_m: config.effective_physical_contact_end_m(),
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
        assert_eq!(map[5].offset_rad, 0.0);
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
