use serde::{Deserialize, Serialize};
use crate::safety::SafetyState;
use crate::driver::{DriverCapabilities, DriverRangeCheck, EnableState};

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
    pub position_tolerances: Vec<f32>,
    pub max_total_deviation: f32,
    /// |velocity| limit (rad/s) for every joint without an entry in `joint_max_velocities`.
    pub max_velocity: f32,
    /// Per-joint |velocity| limits (rad/s), joint 1 first; joints past the end use
    /// `max_velocity`.
    #[serde(default)]
    pub joint_max_velocities: Vec<f32>,
    pub support_torque_limits: Vec<(usize, f32)>,
    pub feedback_max_age_ms: u64,
}

impl RestingPosePolicy {
    /// The velocity and support-torque limits were tuned on readings decoded with the old
    /// single range (VMAX 30, TMAX 18). In real units (J1-J3 VMAX 10, TMAX 28) the same
    /// physical limits are 0.015 / 3 rad/s on J1-J3 and 1.5 / 4.5 N·m × 28 / 18 on J2 / J3.
    pub fn rebot_dm() -> Self {
        Self {
            position_tolerances: vec![0.15, 0.08, 0.08, 0.08, 0.08, 0.08],
            max_total_deviation: 0.22,
            max_velocity: 0.015,
            joint_max_velocities: vec![0.005, 0.005, 0.005, 0.015, 0.015, 0.015, 0.015],
            support_torque_limits: vec![(1, 2.333), (2, 7.0)],
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

    fn validate(&self) -> Result<(), String> {
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
    /// above the table during camera-down grasp acquisition; None = [`CAMERA_BODY_DEFAULT_M`].
    #[serde(skip_serializing_if = "Option::is_none")]
    pub body_size_m: Option<[f32; 3]>,
}

/// reCamera housing 40 x 40 x 36.5 mm in the v8 holder (reBot-DevArm
/// hardware/camera-mounts/recamera-mounts/v8/generate_recamera_mount.py: 0.3 mm clearance,
/// 3 mm walls, 7 mm ears each side, 3 mm support below and top bar above, 3.5 mm rear lip):
/// 54.6 x 46.6 x 40 mm, rounded up.
pub const CAMERA_BODY_DEFAULT_M: [f32; 3] = [0.055, 0.047, 0.040];

impl CameraMount {
    /// The body box size in use.
    pub fn body_size(&self) -> [f32; 3] { self.body_size_m.unwrap_or(CAMERA_BODY_DEFAULT_M) }
}

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
}

impl GripperPolicy {
    /// Thresholds from the MIT-mode project; holding torque and squeeze need verification in POS_VEL.
    /// holding_torque_nm 0.0667 is the former 0.12 read with TMAX 18, converted to the
    /// J7 motor's real TMAX 10 (0.12 × 10 / 18).
    pub fn rebot_dm() -> Self {
        Self { closed_rad: 0.0, open_rad: -5.7, holding_gap_rad: 0.30, holding_torque_nm: 0.0667, squeeze_rad: 0.15, settle_ms: 300 }
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

impl GravityCompPolicy {
    /// Overspeed limit for arm joint `i` (0-based, J1..J6).
    pub fn speed_limit(&self, i: usize) -> f32 {
        self.joint_max_speed_rad_s.get(i).copied().unwrap_or(self.max_speed_rad_s)
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
    /// Gripper joint index. Absent: the joint after the arm joints when `dof` has one more
    /// joint than the arm model (index 6 for the B601-DM), otherwise none.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gripper_joint: Option<usize>,
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
            grasp: None,
            gravity_comp: None,
            damiao_ranges: None,
            speed_cap: None,
            collision: None,
            hardware: None,
            urdf: None,
            urdf_sha256: None,
            chain: None,
            gripper_joint: None,
            arm_model: crate::chain::ArmModel::RebotDevArm,
        }
    }
}

impl ArmConfig {
    /// Number of arm joints of the kinematic / gravity model (6 for the built-in model).
    pub fn arm_dof(&self) -> usize { self.arm_model.arm_dof() }

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

    /// Profile uses the URDF chain for kinematics or gravity.
    pub fn uses_urdf(&self) -> bool {
        self.kinematics == Some(KinematicsModel::Urdf) || self.gravity_model == Some(GravityModel::Urdf)
    }

    /// Read a profile JSON file and build its arm model (`urdf` relative to the file).
    /// The result still needs [`Self::validate`].
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
    pub fn load_arm_model(&mut self, profile_dir: Option<&std::path::Path>) -> Result<(), String> {
        if !self.uses_urdf() {
            if self.urdf.is_some() || self.chain.is_some() {
                return Err("urdf / chain are set but neither kinematics nor gravity_model is \"Urdf\"".into());
            }
            self.arm_model = crate::chain::ArmModel::RebotDevArm;
            return Ok(());
        }
        let (Some(rel), Some(spec)) = (&self.urdf, &self.chain) else {
            return Err("kinematics / gravity_model \"Urdf\" needs the urdf and chain fields".into());
        };
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
        if let Some(ws) = &self.workspace { ws.validate()?; }
        let cam = &self.camera_mount;
        if !cam.rot_deg.is_finite() || !cam.tilt_deg.is_finite()
            || cam.offset_m.is_some_and(|o| o.iter().any(|v| !v.is_finite())) {
            return Err("Invalid camera mount".into());
        }
        if let Some(g) = &self.gripper_policy {
            let lim = if self.dof >= 7 { Some(&self.limits[6]) } else { None };
            let in_lim = |v: f32| lim.is_some_and(|l| v >= l.min_position && v <= l.max_position);
            if ![g.closed_rad, g.open_rad, g.holding_gap_rad, g.holding_torque_nm, g.squeeze_rad].iter().all(|v| v.is_finite())
                || !in_lim(g.closed_rad) || !in_lim(g.open_rad) || (g.closed_rad - g.open_rad).abs() < 0.1
                || g.holding_gap_rad < 0.0 || g.holding_torque_nm < 0.0
                || !(0.0..=0.5).contains(&g.squeeze_rad) || g.settle_ms == 0 || g.settle_ms > 5000 {
                return Err("Invalid gripper policy (needs dof 7, open/closed inside the J7 limits, 0 <= squeeze_rad <= 0.5)".into());
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
                || policy.support_torque_limits.iter().any(|(i, t)| *i >= self.dof || !t.is_finite() || *t < 0.0) {
                return Err("Invalid resting-pose policy".into());
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
            grasp: Some(crate::grasp::GraspSetup::default()),
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
                ..Default::default()
            }),
            hardware: None,
            urdf: None,
            urdf_sha256: None,
            chain: None,
            gripper_joint: None,
            arm_model: crate::chain::ArmModel::RebotDevArm,
        }
    }
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
pub const CAPABILITIES_VERSION: u32 = 6;

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
    pub vision: VisionCapabilities,
}

impl Capabilities {
    /// Capabilities that follow from the profile alone. `gravity_comp` also depends on
    /// the driver and the URDF check; this sets it from the profile only.
    pub fn from_config(config: &ArmConfig) -> Self {
        let seven = config.six_plus_gripper();
        let kin = config.kinematics.is_some();
        let six_arm = config.arm_dof() == 6 && matches!(config.dof, 6 | 7);
        let gripper = config.dof >= 7 && config.gripper_policy.is_some();
        Self {
            version: CAPABILITIES_VERSION,
            profile_version: config.profile_version,
            cartesian: seven && kin,
            workspace_check: kin && config.workspace.is_some(),
            gripper,
            grasp: seven && kin && config.gripper_policy.is_some() && config.grasp.is_some(),
            gravity_comp: seven && config.gravity_model.is_some() && config.gravity_comp.is_some(),
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
            place: seven && kin && config.gripper_policy.is_some(),
            // The driver half is applied by ArmSupervisor::get_schema.
            collision_guard: config.gravity_model.is_some() && six_arm,
            safety_describe: true,
            vision: VisionCapabilities { detection_proxy: true, mode_endpoint: VISION_MODE_ENDPOINT.to_string(), ..Default::default() },
        }
    }
}

/// Observation schema for clients
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObservationSchema {
    pub dof: usize,
    pub joint_names: Vec<String>,
    pub fields: Vec<SchemaField>,
    pub limits: Vec<JointLimit>,
    /// Cartesian safety box enforced by the server (None = not checked).
    #[serde(default)]
    pub workspace: Option<WorkspacePolicy>,
    #[serde(default)]
    pub camera_mount: CameraMount,
    #[serde(default)]
    pub kinematics: Option<KinematicsModel>,
    #[serde(default)]
    pub capabilities: Capabilities,
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
}

impl ObservationSchema {
    pub fn from_config(config: &ArmConfig) -> Self {
        Self {
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
                    shape: vec![6],
                    unit: Some("Nm".to_string()),
                },
            ],
            limits: config.limits.clone(),
            workspace: config.workspace.clone(),
            camera_mount: config.camera_mount.clone(),
            kinematics: config.kinematics,
            capabilities: Capabilities::from_config(config),
            servo_max_velocity: config.servo_limits().iter().map(|l| l.max_velocity).collect(),
            servo_max_lead_rad: Some(config.servo_max_lead_rad),
            speed_cap: config.speed_cap,
            home_positions: Some(config.home_positions.clone()),
            resting_pose: config.resting_pose.clone(),
        }
    }
}
