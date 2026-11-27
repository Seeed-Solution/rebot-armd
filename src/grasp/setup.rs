use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::api::proxy::DetectorUpstream;
use crate::kinematics;
use crate::motion::{self, MotionError};
use crate::supervisor::{ArmError, ArmSupervisor, SupervisorInner};

use super::{valid_task_name, DetectionSource, GraspTask, HttpDetectionSource, TABLE_CLEARANCE_M};

/// Default location of the persisted setup. Override with `ARMD_GRASP_SETUP`.
pub const DEFAULT_SETUP_PATH: &str = "/userdata/rebot-teleop/grasp_setup.json";

// ---- setup ----

/// Grasp setup (`ArmConfig.grasp`). Runtime changes are persisted to a JSON file; every
/// field can also be overridden for one request (`overrides`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct GraspSetup {
    /// Table surface height in the base frame (m).
    pub table_z_m: f64,
    /// TCP (end_link origin = jaw tips) height above the table for approach and centring (m).
    pub approach_height_m: f64,
    /// TCP height above the table when the jaws close (m).
    pub grasp_height_m: f64,
    /// TCP height above the table after the lift (m).
    pub lift_height_m: f64,
    /// Normalised image point [x, y] where the table point under the jaw tips appears at
    /// approach height, looking down.
    pub grip_pixel_norm: [f64; 2],
    /// Camera tilt about x_cam (deg). Mirrors `camera_mount.tilt_deg`; setting it updates the mount.
    pub camera_tilt_deg: f64,
    /// Rough intrinsics used to turn pixel errors into metric steps (placeholders).
    pub image_size_px: [f64; 2],
    pub fx_px: f64,
    pub fy_px: f64,
    /// Principal point (px at `image_size_px`); absent = the image centre.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cx_px: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cy_px: Option<f64>,
    /// Centring is done when the box centre is within this many pixels of the grip pixel ...
    pub centre_tol_px: f64,
    /// ... for this many consecutive frames.
    pub stable_frames: u32,
    /// Frames per pose whose median centre decides the next step.
    pub median_frames: u32,
    /// Centring moves before CENTRING_FAILED.
    pub max_iterations: u32,
    /// Fraction of the estimated metric error applied per centring step.
    pub servo_gain: f64,
    /// Largest centring step (m).
    pub max_step_m: f64,
    /// Same target if the centre is within this many pixels of the prediction or IoU > match_iou.
    pub match_dist_px: f64,
    pub match_iou: f64,
    /// Gate for the first match after the coarse approach (pixels from the prediction).
    pub acquire_dist_px: f64,
    /// TARGET_LOST after this long (s) or this many fresh frames without the target.
    pub lost_timeout_s: f64,
    pub lost_frames: u32,
    /// A frame counts only if captured this long (s) after the motion settled.
    pub settle_delay_s: f64,
    /// Rotate J6 by 90 deg when the box's long / short side ratio is at least this.
    pub align_min_aspect: f64,
    pub align_j6: bool,
    /// Detections with `angle_deg` and `angle_conf` at least this decide the J6 roll from the
    /// angle (long side across the jaws) instead of the box aspect.
    pub align_min_angle_conf: f64,
    /// Angle dispersion threshold (deg) above which J6 alignment from angle is skipped (0 disables).
    pub align_max_angle_spread_deg: f64,
    /// Linear speeds (m/s) for the approach / centring and for descend / lift.
    pub approach_speed_mps: f64,
    pub descend_speed_mps: f64,
    /// Limit per motion phase and for the whole grasp (s).
    pub phase_timeout_s: f64,
    pub timeout_s: f64,
    /// Camera-down acquisition: approach and centre with the optical axis vertical, then
    /// re-orient to the tool-vertical pose above the target. `auto` = on when
    /// |`camera_tilt_deg`| > [`CAMERA_DOWN_AUTO_TILT_DEG`] and `camera_mount.offset_m` is set.
    pub acquire_camera_down: AcquireCameraDown,
    /// Height probe after the centring: move straight up this far (m; [`crate::grasp::PROBE_FALLBACK_DZ_M`]
    /// when that pose is unreachable), estimate the object's top from the change of the box
    /// size and move back; 0 disables the probe in either `height_probe_mode`.
    pub height_probe_m: f64,
    /// Height probe motion: `up` (default, box scale change over `height_probe_m`) or
    /// `lateral` (box-centre parallax over a horizontal baseline of `lateral_probe_m` along the
    /// image x axis; falls back to `up` when the shifted box would reach the image border or a
    /// lateral pose is unreachable).
    pub height_probe_mode: HeightProbeMode,
    /// Lateral probe baseline (m): the tool moves to −B/2 and +B/2 from the centring pose.
    pub lateral_probe_m: f64,
    /// With an estimated top height h, the jaws close at clamp(h · this, `grasp_height_m`,
    /// `max_grasp_height_m`) above the table.
    pub height_grasp_fraction: f64,
    pub max_grasp_height_m: f64,
    /// Look-down: descend first to this TCP height above the table and centre once more
    /// (at most `recentre_max_iterations` corrections) before the rest of the descent; 0 disables.
    pub recentre_height_m: f64,
    pub recentre_max_iterations: u32,
    /// Stop the descent and close where the arm is when |residual torque| of J2 or J3 exceeds
    /// this (N·m); None = off (peaks are only logged).
    pub descend_contact_stop_nm: Option<f64>,
    /// After the lift: wait this long (ms), then check `holding` again on the measured J7
    /// position and torque (DROPPED when the object is gone).
    pub post_lift_settle_ms: u64,
    /// Camera-down re-check after the transition: TARGET_DISPLACED when the target's box
    /// centre is more than this many pixels from where the model projects it.
    pub recheck_tol_px: f64,
    /// Maximum target cross-section width (m) for the jaws; 0 disables the check.
    pub max_grasp_width_m: f64,
    /// Detected labels treated as spill-safe (placed upright with tilt at most `spill_max_tilt_deg`).
    pub spill_safe_labels: Vec<String>,
    /// Maximum allowed tilt (deg) when placing a spill-safe object.
    pub spill_max_tilt_deg: f64,
    /// Width-based pre-open before descend: `auto` (default, only when calibrated), `on` (also uncalibrated), `off`.
    pub pre_open: PreOpen,
    /// Margin (m) added to the measured width for the pre-open opening.
    pub pre_open_margin_m: f64,
    /// Object height sources that decide, in this order: `depth`, `probe` (`height_probe_mode`),
    /// `prior` (`height_priors`); the first accepted height is used, `grasp_height_m` is the last
    /// fallback (`height_source` `setup`).
    pub height_sources: Vec<HeightSource>,
    /// Sources evaluated and reported in `height_estimates` without deciding (`depth`, `prior`);
    /// those also in `height_sources` decide instead.
    pub height_record: Vec<HeightSource>,
    /// Object height above the table (m) per detected label (case-insensitive), for `prior`.
    pub height_priors: BTreeMap<String, f64>,
    /// J6 yaw sources that decide, in this order: `mask` (the detector's `angle_deg`),
    /// `depth` (`depth_angle_deg` of the depth frame), `box` (the box aspect). The first that
    /// accepts decides the roll; an angle source whose frames disagree (`angle_spread`), whose
    /// long side does not meet the plane (`ray`) or that does not fit J6 (`j6_limits`) ends
    /// the alignment without a turn.
    pub yaw_sources: Vec<YawSource>,
    /// Yaw sources evaluated and reported in `yaw_estimates` without deciding; those also in
    /// `yaw_sources` decide instead.
    pub yaw_record: Vec<YawSource>,
    /// Grasp width sources that decide, in this order: `mask` (the detector's `grasp_width_px`
    /// on the grasp plane), `depth` (the depth measurement's width). The first that accepts is
    /// `grasp_width_m`.
    pub width_sources: Vec<WidthSource>,
    /// Width sources evaluated and reported in `width_estimates` without deciding.
    pub width_record: Vec<WidthSource>,
    /// Named grasp tasks: stored overrides plus perception requirements, selected per request
    /// with `task`. Changed only as a whole through the setup (`/api/v1/grasp/setup`, the
    /// setup file); request and task `overrides` cannot contain it.
    pub tasks: BTreeMap<String, GraspTask>,
}


/// Object height source (`height_sources`, `height_record`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HeightSource {
    Depth,
    Probe,
    Prior,
}

/// J6 yaw source (`yaw_sources`, `yaw_record`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum YawSource {
    Mask,
    Depth,
    Box,
}

impl YawSource {
    pub fn name(self) -> &'static str {
        match self { Self::Mask => "mask", Self::Depth => "depth", Self::Box => "box" }
    }
}

/// Grasp width source (`width_sources`, `width_record`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WidthSource {
    Mask,
    Depth,
}

impl WidthSource {
    pub fn name(self) -> &'static str {
        match self { Self::Mask => "mask", Self::Depth => "depth" }
    }
}

/// `pre_open` setting.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum PreOpen {
    #[default]
    Auto,
    On,
    Off,
}

/// `height_probe_mode` setting.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum HeightProbeMode {
    #[default]
    Up,
    Lateral,
}

/// `acquire_camera_down` setting.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum AcquireCameraDown {
    #[default]
    Auto,
    On,
    Off,
}

/// `acquire_camera_down: auto` turns camera-down acquisition on above this mount tilt (deg).
pub const CAMERA_DOWN_AUTO_TILT_DEG: f64 = 5.0;

impl Default for GraspSetup {
    fn default() -> Self {
        Self {
            table_z_m: 0.0,
            approach_height_m: 0.15,
            grasp_height_m: 0.015,
            lift_height_m: 0.12,
            grip_pixel_norm: [0.5, 0.5],
            camera_tilt_deg: 0.0,
            image_size_px: [640.0, 360.0],
            fx_px: 600.0,
            fy_px: 600.0,
            cx_px: None,
            cy_px: None,
            centre_tol_px: 10.0,
            stable_frames: 3,
            median_frames: 3,
            max_iterations: 10,
            servo_gain: 0.6,
            max_step_m: 0.03,
            match_dist_px: 35.0,
            match_iou: 0.45,
            acquire_dist_px: 160.0,
            lost_timeout_s: 1.0,
            lost_frames: 5,
            settle_delay_s: 0.1,
            align_min_aspect: 1.2,
            align_j6: true,
            align_min_angle_conf: 0.3,
            align_max_angle_spread_deg: 15.0,
            approach_speed_mps: 0.05,
            descend_speed_mps: 0.03,
            // covers a gripper close at close_speed_rad_s over the whole stroke (25.6 s deadline)
            phase_timeout_s: 30.0,
            timeout_s: 120.0,
            acquire_camera_down: AcquireCameraDown::Auto,
            recheck_tol_px: 40.0,
            height_probe_m: 0.10,
            height_probe_mode: HeightProbeMode::Up,
            lateral_probe_m: 0.04,
            height_grasp_fraction: 0.5,
            max_grasp_height_m: 0.06,
            recentre_height_m: 0.0,
            recentre_max_iterations: 3,
            descend_contact_stop_nm: None,
            post_lift_settle_ms: 300,
            max_grasp_width_m: 0.075,
            spill_safe_labels: vec!["cup".into(), "bottle".into(), "wine glass".into(), "bowl".into(), "vase".into()],
            spill_max_tilt_deg: 8.0,
            pre_open: PreOpen::Auto,
            pre_open_margin_m: 0.015,
            height_sources: vec![HeightSource::Probe],
            height_record: vec![HeightSource::Depth],
            height_priors: BTreeMap::new(),
            yaw_sources: vec![YawSource::Mask, YawSource::Box],
            yaw_record: vec![YawSource::Depth],
            width_sources: vec![WidthSource::Mask, WidthSource::Depth],
            width_record: Vec::new(),
            tasks: BTreeMap::new(),
        }
    }
}

impl GraspSetup {
    pub fn validate(&self) -> Result<(), String> {
        let fin = |v: f64| v.is_finite();
        let all = [self.table_z_m, self.approach_height_m, self.grasp_height_m, self.lift_height_m,
            self.grip_pixel_norm[0], self.grip_pixel_norm[1], self.camera_tilt_deg, self.image_size_px[0],
            self.image_size_px[1], self.fx_px, self.fy_px, self.centre_tol_px, self.servo_gain, self.max_step_m,
            self.match_dist_px, self.match_iou, self.acquire_dist_px, self.lost_timeout_s, self.settle_delay_s,
            self.align_min_aspect, self.align_min_angle_conf, self.align_max_angle_spread_deg, self.approach_speed_mps, self.descend_speed_mps, self.phase_timeout_s, self.timeout_s, self.recheck_tol_px,
            self.height_probe_m, self.lateral_probe_m, self.height_grasp_fraction, self.max_grasp_height_m, self.recentre_height_m, self.max_grasp_width_m, self.spill_max_tilt_deg, self.pre_open_margin_m];
        if !all.iter().all(|v| fin(*v)) { return Err("grasp setup values must be finite".into()); }
        let check = |ok: bool, msg: &str| if ok { Ok(()) } else { Err(msg.to_string()) };
        check((-0.5..=0.5).contains(&self.table_z_m), "table_z_m must be in [-0.5, 0.5] m")?;
        // >= TABLE_CLEARANCE_M: the jaw tips (TCP) at the grasp height are within the clearance rule
        check(self.grasp_height_m >= TABLE_CLEARANCE_M && self.grasp_height_m <= 0.2, "grasp_height_m must be in [0.005, 0.2] m")?;
        check(self.approach_height_m >= self.grasp_height_m + 0.02 && self.approach_height_m <= 0.4,
            "approach_height_m must be at least grasp_height_m + 0.02 and at most 0.4 m")?;
        check(self.lift_height_m >= self.grasp_height_m + 0.02 && self.lift_height_m <= 0.4,
            "lift_height_m must be at least grasp_height_m + 0.02 and at most 0.4 m")?;
        check(self.grip_pixel_norm.iter().all(|v| (0.0..=1.0).contains(v)), "grip_pixel_norm must be in [0, 1]")?;
        check(self.camera_tilt_deg.abs() <= 60.0, "camera_tilt_deg must be in [-60, 60]")?;
        check(self.image_size_px.iter().all(|v| *v >= 16.0 && *v <= 8192.0), "image_size_px must be in [16, 8192]")?;
        check(self.fx_px > 10.0 && self.fy_px > 10.0 && self.fx_px < 1e5 && self.fy_px < 1e5, "fx_px / fy_px must be in (10, 1e5)")?;
        check(self.cx_px.is_none_or(|c| c.is_finite() && c > 0.0 && c < self.image_size_px[0])
            && self.cy_px.is_none_or(|c| c.is_finite() && c > 0.0 && c < self.image_size_px[1]),
            "cx_px / cy_px must be null or inside the image (0, image_size_px)")?;
        check(self.centre_tol_px > 0.0 && self.centre_tol_px <= 100.0, "centre_tol_px must be in (0, 100]")?;
        check((1..=20).contains(&self.stable_frames) && (1..=20).contains(&self.median_frames), "stable_frames / median_frames must be 1..20")?;
        check((1..=30).contains(&self.max_iterations), "max_iterations must be 1..30")?;
        check(self.servo_gain > 0.0 && self.servo_gain <= 1.0, "servo_gain must be in (0, 1]")?;
        check(self.max_step_m > 0.0 && self.max_step_m <= 0.05, "max_step_m must be in (0, 0.05] m")?;
        check(self.match_dist_px > 0.0 && (0.0..1.0).contains(&self.match_iou) && self.acquire_dist_px >= self.match_dist_px,
            "match_dist_px > 0, match_iou in [0, 1), acquire_dist_px >= match_dist_px")?;
        check(self.lost_timeout_s > 0.0 && self.lost_timeout_s <= 10.0 && (1..=100).contains(&self.lost_frames),
            "lost_timeout_s must be in (0, 10] s and lost_frames 1..100")?;
        check((0.0..=2.0).contains(&self.settle_delay_s), "settle_delay_s must be in [0, 2] s")?;
        check(self.recheck_tol_px > 0.0 && self.recheck_tol_px <= 500.0, "recheck_tol_px must be in (0, 500]")?;
        check((0.0..=0.1).contains(&self.height_probe_m), "height_probe_m must be in [0, 0.1] m")?;
        check((0.01..=0.08).contains(&self.lateral_probe_m), "lateral_probe_m must be in [0.01, 0.08] m")?;
        check((0.0..=1.0).contains(&self.height_grasp_fraction), "height_grasp_fraction must be in [0, 1]")?;
        check(self.max_grasp_height_m >= self.grasp_height_m && self.max_grasp_height_m <= 0.2,
            "max_grasp_height_m must be in [grasp_height_m, 0.2] m")?;
        check((0.0..=0.4).contains(&self.recentre_height_m), "recentre_height_m must be in [0, 0.4] m")?;
        check(self.post_lift_settle_ms <= 5000, "post_lift_settle_ms must be at most 5000")?;
        check((1..=30).contains(&self.recentre_max_iterations), "recentre_max_iterations must be 1..30")?;
        check(self.descend_contact_stop_nm.is_none_or(|v| v.is_finite() && v > 0.0 && v <= 50.0), "descend_contact_stop_nm must be null or in (0, 50] N·m")?;
        check(self.align_min_aspect >= 1.0 && self.align_min_aspect <= 10.0, "align_min_aspect must be in [1, 10]")?;
        check((0.0..=1.0).contains(&self.align_min_angle_conf), "align_min_angle_conf must be in [0, 1]")?;
        check((0.0..=90.0).contains(&self.align_max_angle_spread_deg), "align_max_angle_spread_deg must be in [0, 90]")?;
        check((0.0..=0.2).contains(&self.max_grasp_width_m), "max_grasp_width_m must be in [0, 0.2] m")?;
        check((1.0..=30.0).contains(&self.spill_max_tilt_deg), "spill_max_tilt_deg must be in [1, 30]")?;
        check((0.005..=0.05).contains(&self.pre_open_margin_m), "pre_open_margin_m must be in [0.005, 0.05] m")?;
        let unique = |l: &[HeightSource]| l.iter().enumerate().all(|(i, x)| !l[..i].contains(x));
        check(unique(&self.height_sources) && unique(&self.height_record), "height_sources / height_record must not repeat a source")?;
        check(!self.height_record.contains(&HeightSource::Probe), "height_record allows only depth and prior")?;
        let unique_yaw = |l: &[YawSource]| l.iter().enumerate().all(|(i, x)| !l[..i].contains(x));
        check(unique_yaw(&self.yaw_sources) && unique_yaw(&self.yaw_record), "yaw_sources / yaw_record must not repeat a source")?;
        let unique_width = |l: &[WidthSource]| l.iter().enumerate().all(|(i, x)| !l[..i].contains(x));
        check(unique_width(&self.width_sources) && unique_width(&self.width_record), "width_sources / width_record must not repeat a source")?;
        check(self.height_priors.values().all(|h| h.is_finite() && *h > 0.0 && *h <= 0.5), "height_priors must be in (0, 0.5] m")?;
        let labels: Vec<String> = self.height_priors.keys().map(|k| k.to_ascii_lowercase()).collect();
        check(labels.iter().enumerate().all(|(i, k)| !labels[..i].contains(k)), "height_priors labels must be unique ignoring case")?;
        let speed_ok = |v: f64| v > 0.0 && v <= motion::MAX_LINEAR_MPS;
        check(speed_ok(self.approach_speed_mps) && speed_ok(self.descend_speed_mps), "speeds must be in (0, 0.15] m/s")?;
        check(self.phase_timeout_s > 0.0 && self.phase_timeout_s <= 120.0 && self.timeout_s >= self.phase_timeout_s && self.timeout_s <= 600.0,
            "phase_timeout_s in (0, 120] s, timeout_s in [phase_timeout_s, 600] s")?;
        if !self.tasks.is_empty() {
            // every stored task is valid against this setup (its task-free copy)
            let base = self.without_tasks();
            for (name, task) in &self.tasks {
                check(valid_task_name(name), &format!("grasp task name {name:?} must be 1..32 characters of a-z, 0-9, _ and -"))?;
                check(task.description.as_ref().is_none_or(|d| d.len() <= 200), &format!("grasp task {name}: description must be at most 200 bytes"))?;
                check(!task.overrides.contains_key("tasks"), &format!("grasp task {name}: overrides cannot contain tasks"))?;
                if let Some(m) = &task.models { m.validate().map_err(|e| format!("grasp task {name}: {e}"))?; }
                base.apply_fields(&task.overrides).and_then(|s| s.validate()).map_err(|e| format!("grasp task {name}: {e}"))?;
            }
        }
        Ok(())
    }

    /// This setup with an empty task registry.
    pub fn without_tasks(&self) -> Self {
        let mut s = self.clone();
        s.tasks.clear();
        s
    }

    /// Set the fields of `obj` (unknown keys rejected) without validating the result.
    fn apply_fields(&self, obj: &serde_json::Map<String, Value>) -> Result<Self, String> {
        let mut base = serde_json::to_value(self).map_err(|e| e.to_string())?;
        let fields = base.as_object_mut().expect("setup serialises to an object");
        for (k, v) in obj {
            if !fields.contains_key(k) && !OPTIONAL_FIELDS.contains(&k.as_str()) { return Err(format!("unknown grasp setup field: {k}")); }
            fields.insert(k.clone(), v.clone());
        }
        serde_json::from_value(base).map_err(|e| format!("invalid grasp setup: {e}"))
    }

    /// Configuration patch (setup API, setup file): apply a JSON object of field changes;
    /// unknown keys and invalid results are rejected. `tasks` replaces the whole task map
    /// (`{}` deletes all tasks).
    pub fn patched(&self, patch: &Value) -> Result<Self, String> {
        let Some(obj) = patch.as_object() else { return Err("grasp setup must be a JSON object".into()); };
        let out = self.apply_fields(obj)?;
        out.validate()?;
        Ok(out)
    }

    /// Request patch (`overrides` of one grasp) on a task-free setup: like [`Self::patched`],
    /// but `tasks` is rejected.
    pub fn patched_request(&self, patch: &Value) -> Result<Self, String> {
        let Some(obj) = patch.as_object() else { return Err("grasp setup must be a JSON object".into()); };
        if obj.contains_key("tasks") { return Err("tasks cannot be overridden per request".into()); }
        self.patched(patch)
    }
}

/// Setup fields left out of the JSON while unset (`null` in a patch unsets them).
const OPTIONAL_FIELDS: [&str; 2] = ["cx_px", "cy_px"];

/// Setup file path: `ARMD_GRASP_SETUP` or [`DEFAULT_SETUP_PATH`].
pub fn setup_path_from_env() -> PathBuf {
    std::env::var_os("ARMD_GRASP_SETUP").map(PathBuf::from).unwrap_or_else(|| PathBuf::from(DEFAULT_SETUP_PATH))
}

/// Result of reading the setup file.
#[derive(Debug, Clone, PartialEq)]
pub enum LoadOutcome { Loaded, Missing, Invalid(String) }

/// Read the persisted setup on top of `base`. A missing file keeps `base`; an unreadable,
/// malformed or invalid file keeps `base` and reports why (the file is left untouched).
pub fn load_setup(path: &Path, base: &GraspSetup) -> (GraspSetup, LoadOutcome) {
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return (base.clone(), LoadOutcome::Missing),
        Err(e) => return (base.clone(), LoadOutcome::Invalid(format!("read {}: {e}", path.display()))),
    };
    let value: Value = match serde_json::from_str(&text) {
        Ok(v) => v,
        Err(e) => return (base.clone(), LoadOutcome::Invalid(format!("parse {}: {e}", path.display()))),
    };
    match base.patched(&value) {
        Ok(s) => (s, LoadOutcome::Loaded),
        Err(e) => (base.clone(), LoadOutcome::Invalid(format!("{}: {e}", path.display()))),
    }
}

/// Serialize `setup` stamped with the active tool's `(tool_id, tool_config_digest)`.
pub fn serialize_setup_for_tool(setup: &GraspSetup, active_tool: &crate::tool::ActiveToolInfo) -> String {
    let mut val = serde_json::to_value(setup).unwrap_or_default();
    if let Some(obj) = val.as_object_mut() {
        obj.insert("tool_id".into(), serde_json::json!(active_tool.id));
        obj.insert("tool_config_digest".into(), serde_json::json!(active_tool.config_digest));
    }
    serde_json::to_string_pretty(&val).unwrap_or_default()
}

/// Read the persisted setup on top of `base`, verifying `(tool_id, tool_config_digest)` against
/// `active_tool` (legacy unkeyed records apply only when `active_tool.is_stock`).
pub fn load_setup_for_tool(path: &Path, base: &GraspSetup, active_tool: &crate::tool::ActiveToolInfo) -> (GraspSetup, LoadOutcome) {
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return (base.clone(), LoadOutcome::Missing),
        Err(e) => return (base.clone(), LoadOutcome::Invalid(format!("read {}: {e}", path.display()))),
    };
    let mut value: Value = match serde_json::from_str(&text) {
        Ok(v) => v,
        Err(e) => return (base.clone(), LoadOutcome::Invalid(format!("parse {}: {e}", path.display()))),
    };
    if let Some(obj) = value.as_object_mut() {
        let tid_raw = obj.remove("tool_id");
        let tdig_raw = obj.remove("tool_config_digest");
        let tid = match tid_raw {
            None | Some(Value::Null) => None,
            Some(Value::String(s)) => Some(s),
            Some(_) => return (base.clone(), LoadOutcome::Invalid(format!("{}: tool_id must be a string", path.display()))),
        };
        let tdig = match tdig_raw {
            None | Some(Value::Null) => None,
            Some(Value::String(s)) => Some(s),
            Some(_) => return (base.clone(), LoadOutcome::Invalid(format!("{}: tool_config_digest must be a string", path.display()))),
        };
        if !active_tool.matches_record(tid.as_deref(), tdig.as_deref()) {
            return (
                base.clone(),
                LoadOutcome::Invalid(format!(
                    "{}: grasp setup belongs to tool {:?}/{:?}, not active tool {}/{}",
                    path.display(), tid, tdig, active_tool.id, active_tool.config_digest
                )),
            );
        }
    }
    match base.patched(&value) {
        Ok(s) => (s, LoadOutcome::Loaded),
        Err(e) => (base.clone(), LoadOutcome::Invalid(format!("{}: {e}", path.display()))),
    }
}

/// Load the persisted setup into the supervisor at startup (logs the outcome).
pub async fn load_into(supervisor: &ArmSupervisor, path: &Path) {
    let mut inner = supervisor.inner().write().await;
    let Some(base) = inner.grasp_setup() else { return; };
    let (setup, outcome) = load_setup_for_tool(path, &base, &inner.config.active_tool);
    let mut needs_camera_sync = false;
    match &outcome {
        LoadOutcome::Loaded => {
            tracing::info!("Grasp setup loaded from {}", path.display());
            log_grasp_width_check(&inner, setup.max_grasp_width_m);
        }
        LoadOutcome::Missing => {
            tracing::info!("No grasp setup at {}; using defaults", path.display());
            needs_camera_sync = true;
        }
        LoadOutcome::Invalid(e) => {
            tracing::warn!("Ignoring grasp setup file: {e}; using defaults");
            needs_camera_sync = true;
        }
    }
    inner.grasp_setup_needs_camera_sync = needs_camera_sync;
    inner.set_grasp_setup(setup);
}

/// Warn when `max_w` cannot be verified (no jaw curve on a physical follower) or exceeds the jaw
/// curve's open width (grasps are clamped to it). The setup stays in effect either way.
pub(crate) fn log_grasp_width_check(inner: &SupervisorInner, max_w: f64) {
    match inner.grasp_width_check(max_w) {
        Some(crate::model::GraspWidthCheck::Unverified) => tracing::warn!(
            "Grasp setup max_grasp_width_m {max_w:.4} m is unverified: no gripper jaw_opening calibration curve"
        ),
        Some(crate::model::GraspWidthCheck::Clamped { curve_max_m }) => tracing::warn!(
            "Grasp setup max_grasp_width_m {max_w:.4} m exceeds the jaw curve's open width {curve_max_m:.4} m; grasps use {curve_max_m:.4} m"
        ),
        _ => {}
    }
}

/// `GET /api/v1/grasp/setup` `width_check`: `max_grasp_width_m` against the jaw curve
/// (`status` off / verified / unverified / clamped, `jaw_curve` present / missing).
pub fn grasp_width_check_json(inner: &SupervisorInner, max_w: f64) -> Value {
    match inner.grasp_width_check(max_w) {
        Some(c) => {
            let has_curve = inner.config.gripper_policy.as_ref().is_some_and(|g| g.jaw_opening.is_some());
            serde_json::json!({
                "status": c.status(),
                "jaw_curve": if has_curve { "present" } else { "missing" },
                "max_grasp_width_m": max_w,
                "effective_max_grasp_width_m": c.effective_max_m(max_w),
                "curve_max_opening_m": c.curve_max_m(),
            })
        }
        None => Value::Null,
    }
}

/// Published-config counterpart of [`grasp_width_check_json`] (zero `SupervisorLock`).
pub fn grasp_width_check_json_from_published(
    pub_cfg: &crate::rt_core::requests::config::PublishedConfig,
    max_w: f64,
) -> Value {
    match pub_cfg.grasp_width_check(max_w) {
        Some(c) => {
            let has_curve = pub_cfg.config.gripper_policy.as_ref().is_some_and(|g| g.jaw_opening.is_some());
            serde_json::json!({
                "status": c.status(),
                "jaw_curve": if has_curve { "present" } else { "missing" },
                "max_grasp_width_m": max_w,
                "effective_max_grasp_width_m": c.effective_max_m(max_w),
                "curve_max_opening_m": c.curve_max_m(),
            })
        }
        None => Value::Null,
    }
}

/// What the WS / REST handlers need for grasps: the detection source and the setup file.
#[derive(Clone)]
pub struct GraspDeps {
    pub source: Arc<dyn DetectionSource>,
    /// None = runtime setup changes are not persisted.
    pub setup_path: Option<PathBuf>,
}

impl GraspDeps {
    pub fn http(upstream: DetectorUpstream, setup_path: Option<PathBuf>) -> Self {
        Self { source: Arc::new(HttpDetectionSource { upstream, image_size_px: GraspSetup::default().image_size_px }), setup_path }
    }
}

/// Validated grasp setup update prepared without mutating `SupervisorInner`.
#[derive(Debug, Clone)]
pub struct PreparedGraspSetup {
    pub base_revision: u64,
    pub setup: GraspSetup,
    pub write: Option<crate::persist::PendingWrite>,
}

/// Step 1 of atomic persist-before-commit for `/api/v1/grasp/setup`.
pub fn prepare_setup_update(
    inner: &SupervisorInner,
    body: &Value,
    setup_path: Option<&PathBuf>,
    require_expected_revision: bool,
) -> Result<PreparedGraspSetup, ArmError> {
    let Some(base) = inner.grasp_setup() else {
        return Err(MotionError::Unsupported("This profile has no grasp setup".into()).into());
    };
    let Some(obj) = body.as_object() else {
        return Err(MotionError::InvalidCommand("grasp setup must be a JSON object".into()).into());
    };
    if let Some(exp) = crate::persist::parse_expected_revision(body, require_expected_revision)
        .map_err(MotionError::InvalidCommand)?
    {
        if exp != inner.grasp_setup_revision {
            return Err(ArmError::PolicyRevisionMismatch {
                expected: exp,
                current: inner.grasp_setup_revision,
            });
        }
    }
    let mut patch = obj.clone();
    patch.remove("expected_revision");
    match patch.remove("table_z_from_tcp") {
        None | Some(Value::Null) | Some(Value::Bool(false)) => {}
        Some(Value::Bool(true)) => {
            if inner.config.kinematics.is_none() {
                return Err(MotionError::Unsupported("table_z_from_tcp requires kinematics".into()).into());
            }
            inner.check_table_capture_stillness()?;
            let Some(end_pose) = inner.config.arm_model.fk_f32(&inner.measured_positions) else {
                return Err(MotionError::InvalidCommand("measured positions are not a valid joint vector".into()).into());
            };
            let contact_end = inner
                .config
                .effective_physical_contact_end_m()
                .ok_or_else(|| {
                    MotionError::InvalidCommand(
                        "table_z_from_tcp requires a known physical_contact_end_m for the active tool".into(),
                    )
                })?;
            // Tool +X axis must point vertically down (-Z_base) within 5 degrees.
            let tool_x = kinematics::column(&end_pose.r, 0);
            let tilt_deg = (-tool_x[2]).clamp(-1.0, 1.0).acos().to_degrees();
            if tilt_deg > 5.0 {
                return Err(MotionError::InvalidCommand(format!(
                    "table_z_from_tcp requires the tool within 5 deg of vertical (current tilt {tilt_deg:.1} deg)"
                )).into());
            }
            if let (Some(idx), Some(policy)) = (inner.config.gripper_index(), inner.config.gripper_policy.as_ref()) {
                let j7 = inner.measured_positions.get(idx).copied().ok_or_else(|| {
                    MotionError::InvalidCommand("measured gripper position unavailable".into())
                })?;
                let span = (policy.open_rad - policy.closed_rad).abs();
                let frac = if span > 1e-6 { ((j7 - policy.closed_rad) / (policy.open_rad - policy.closed_rad)).clamp(0.0, 1.0) } else { 0.0 };
                if frac > 0.05 && (j7 - policy.closed_rad).abs() > 0.20 {
                    return Err(MotionError::InvalidCommand(format!(
                        "table_z_from_tcp requires the gripper closed (measured {j7:.3} rad, closed {:.3} rad)",
                        policy.closed_rad
                    )).into());
                }
            }
            let contact_world = end_pose.transform_point(&contact_end);
            let z = contact_world[2];
            patch.insert("table_z_m".into(), serde_json::json!((z * 1e4).round() / 1e4));
        }
        Some(_) => return Err(MotionError::InvalidCommand("table_z_from_tcp must be boolean".into()).into()),
    }
    if inner.camera_calibrated() {
        refuse_calibrated_fields(&base, &patch, "")?;
    }
    let width_patched = patch.contains_key("max_grasp_width_m");
    let setup = base.patched(&Value::Object(patch)).map_err(MotionError::InvalidCommand)?;
    // a new max_grasp_width_m beyond the jaw curve is refused; other edits keep a stored width
    // (grasps are clamped to the curve), and without a curve the width stays unverified
    match inner.grasp_width_check(setup.max_grasp_width_m) {
        Some(crate::model::GraspWidthCheck::Clamped { curve_max_m }) if width_patched => {
            return Err(MotionError::InvalidCommand(format!(
                "max_grasp_width_m {:.4} m exceeds the jaw curve's open width {curve_max_m:.4} m",
                setup.max_grasp_width_m
            )).into());
        }
        _ => log_grasp_width_check(inner, setup.max_grasp_width_m),
    }
    // the camera tilt is workspace geometry (camera check point): it cannot change under an
    // accepted motion; other setup fields can
    if setup.camera_tilt_deg as f32 != inner.config.camera_mount.tilt_deg {
        if let Some(reason) = inner.workspace_change_blocker() {
            return Err(MotionError::MotionBusy(format!("camera_tilt_deg cannot change while {reason}")).into());
        }
    }
    let write = setup_path.cloned().map(|p| {
        crate::persist::PendingWrite::new(p, serialize_setup_for_tool(&setup, &inner.config.active_tool))
    });
    Ok(PreparedGraspSetup {
        base_revision: inner.grasp_setup_revision,
        setup,
        write,
    })
}

/// Step 2.5 of atomic persist-before-commit (under supervisor lock, before publish).
pub fn precheck_setup_update(inner: &SupervisorInner, prepared: &PreparedGraspSetup) -> Result<(), ArmError> {
    if inner.grasp_setup_revision != prepared.base_revision {
        return Err(ArmError::PolicyRevisionMismatch {
            expected: prepared.base_revision,
            current: inner.grasp_setup_revision,
        });
    }
    if inner.calibration_busy {
        return Err(ArmError::Rejected {
            status: 409,
            code: "CONFIG_BUSY".into(),
            message: "a camera calibration apply or restore is running".into(),
        });
    }
    if prepared.setup.camera_tilt_deg as f32 != inner.config.camera_mount.tilt_deg {
        if let Some(reason) = inner.workspace_change_blocker() {
            return Err(MotionError::MotionBusy(format!("camera_tilt_deg cannot change while {reason}")).into());
        }
    }
    Ok(())
}

/// Step 3 of atomic persist-before-commit: re-verify revision and commit.
pub fn commit_setup_update(
    inner: &mut SupervisorInner,
    prepared: PreparedGraspSetup,
) -> Result<(GraspSetup, u64), ArmError> {
    if inner.grasp_setup_revision != prepared.base_revision {
        return Err(ArmError::PolicyRevisionMismatch {
            expected: prepared.base_revision,
            current: inner.grasp_setup_revision,
        });
    }
    inner.set_grasp_setup(prepared.setup.clone());
    inner.grasp_setup_revision += 1;
    Ok((prepared.setup, inner.grasp_setup_revision))
}

/// WS command `/api/v1/grasp/setup`: apply a patch (and `table_z_from_tcp: true`, which sets
/// `table_z_m` to the current commanded TCP height). Returns the new setup.
pub fn update_setup(inner: &mut SupervisorInner, body: &Value) -> Result<GraspSetup, ArmError> {
    let prepared = prepare_setup_update(inner, body, None, false)?;
    let (setup, _) = commit_setup_update(inner, prepared)?;
    Ok(setup)
}

/// Grasp setup fields the camera calibration sets (`/api/v1/calibration/apply`). While the
/// mount is calibrated, setup writes, task overrides and request overrides that change them
/// are refused; `grip_pixel_norm` and `table_z_m` stay writable.
pub const CALIBRATED_FIELDS: [&str; 6] = ["camera_tilt_deg", "fx_px", "fy_px", "cx_px", "cy_px", "image_size_px"];

fn same_json(a: &Value, b: &Value) -> bool {
    match (a, b) {
        (Value::Number(x), Value::Number(y)) => x.as_f64().zip(y.as_f64()).is_some_and(|(x, y)| (x - y).abs() <= 1e-9 * x.abs().max(1.0)),
        (Value::Array(x), Value::Array(y)) => x.len() == y.len() && x.iter().zip(y).all(|(a, b)| same_json(a, b)),
        _ => a == b,
    }
}

/// INVALID_COMMAND naming the first calibrated field `patch` changes against `current`, and
/// any calibrated field in the `overrides` of a task in `patch.tasks`. `what` prefixes the field.
pub fn refuse_calibrated_fields(current: &GraspSetup, patch: &serde_json::Map<String, Value>, what: &str) -> Result<(), MotionError> {
    let cur = serde_json::to_value(current).unwrap_or(Value::Null);
    let refuse = |f: &str| MotionError::InvalidCommand(format!("{what}{f}: camera mount is calibrated; restore or recalibrate"));
    for f in CALIBRATED_FIELDS {
        if let Some(v) = patch.get(f) {
            if !cur.get(f).is_some_and(|c| same_json(c, v)) { return Err(refuse(f)); }
        }
    }
    if let Some(tasks) = patch.get("tasks").and_then(Value::as_object) {
        for (name, task) in tasks {
            if let Some(o) = task.get("overrides").and_then(Value::as_object) {
                if let Some(f) = CALIBRATED_FIELDS.iter().find(|f| o.contains_key(**f)) {
                    return Err(MotionError::InvalidCommand(format!("{what}grasp task {name}: {f}: camera mount is calibrated; restore or recalibrate")));
                }
            }
        }
    }
    Ok(())
}

/// [`refuse_calibrated_fields`] for a grasp request: its `overrides` and its task's stored overrides.
pub(super) fn refuse_calibrated_request(base: &GraspSetup, body: &Value) -> Result<(), MotionError> {
    if let Some(name) = body.get("task").and_then(Value::as_str) {
        if let Some(t) = base.tasks.get(name) {
            refuse_calibrated_fields(base, &t.overrides, &format!("grasp task {name}: "))?;
        }
    }
    if let Some(o) = body.get("overrides").and_then(Value::as_object) { refuse_calibrated_fields(base, o, "overrides.")?; }
    Ok(())
}
