//! Tap-to-grasp action (`/api/v1/action/grasp`, docs/agent_action_api.md).
//!
//! The user picks a detection box; an async orchestrator (outside the supervisor lock and
//! the control tick) runs: open → coarse approach (box-centre ray hits the table plane) →
//! closed-loop image centring → J6 alignment → descend → close → lift. Every motion is a
//! sub-motion of the one `grasp` action record, installed under the lock only while the
//! grasp is still running, so stop / E-stop / lease loss / supersede end it between or
//! during motions without leaving a half-issued motion.
//!
//! The closed loop drives the target's box centre onto `grip_pixel_norm` (where the table
//! point under the jaw tips appears at approach height). Pixel errors become metric steps
//! through rough pinhole intrinsics and the FK-known camera height, scaled by a
//! conservative gain, so wrong intrinsics or mount angles slow convergence but do not
//! break it.
//!
//! Camera-down acquisition (`acquire_camera_down`, for mounts pitched toward the jaws such
//! as the v8 holder): approach with the optical axis vertical and the camera centre above
//! the estimate, centre the box on the image centre, then one Cartesian move to the
//! tool-vertical pose above the table point of the centred ray (`transition`).
//!
//! Table clearance ([`TABLE_CLEARANCE_M`], [`envelope`]): every sub-motion, in both modes,
//! keeps the TCP, the gripper check points and the camera (lens, body box) above
//! `table_z_m + 0.005` along its path. The approach (and the camera-down transition) is
//! checked before the ack; every sub-motion again before it is installed.
use std::collections::BTreeMap;
use std::future::Future;
use std::path::{Path, PathBuf};
use std::pin::Pin;
use std::sync::Arc;
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::api::proxy::DetectorUpstream;
use crate::api::rest::ApiError;
use crate::kinematics::{self, Mat3, Pose, Vec3};
use crate::model::{CameraMount, HoldingMode, WorkspacePolicy};
use crate::estimate::{EstimateChain, EstimateEntry, SourceResult};
use crate::macro_action::{fail, ContactGuard, MacroCtx, Stop, SubDone};
use crate::motion::{self, GoalFrame, GoalSpec, GripperCommand, MotionError, PlanInput, RelFrame, Speed};
use crate::supervisor::{ArmError, ArmSupervisor, SupervisorInner};

pub type BoxFuture<'a, T> = Pin<Box<dyn Future<Output = T> + Send + 'a>>;

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
    /// Height probe after the centring: move straight up this far (m; [`PROBE_FALLBACK_DZ_M`]
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

/// A named grasp task (`GraspSetup.tasks`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GraspTask {
    /// Grasp setup fields applied on top of the setup (before the request's `overrides`).
    #[serde(default)]
    pub overrides: serde_json::Map<String, Value>,
    /// What the vision provider must offer; checked before the grasp begins.
    #[serde(default, skip_serializing_if = "TaskRequires::is_empty")]
    pub requires: TaskRequires,
    /// Models the provider loads for the task (`POST /api/models` before the grasp begins).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub models: Option<TaskModels>,
    /// Free text, at most 200 bytes.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
}

/// Perception requirements of a grasp task, compared with the provider's `GET /api/mode`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct TaskRequires {
    /// `model.kind` of the loaded YOLO model (`detect` | `segment`); exact match.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub model_kind: Option<ModelKind>,
    /// Depth pipeline: `any` (`available.depth`) or `metric` (also `depth_model.metric`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub depth: Option<DepthReq>,
}

/// The provider's model slots a task sets (names of the provider catalogue, `GET /api/mode`
/// `models.catalogue`). Per slot: absent = keep, `null` = unload, a name = load.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TaskModels {
    #[serde(default, deserialize_with = "some_or_null", skip_serializing_if = "Option::is_none")]
    pub detector: Option<Option<String>>,
    #[serde(default, deserialize_with = "some_or_null", skip_serializing_if = "Option::is_none")]
    pub depth: Option<Option<String>>,
}

/// A present field: `null` -> `Some(None)` (an absent one stays `None` through `default`).
fn some_or_null<'de, D: serde::Deserializer<'de>>(d: D) -> Result<Option<Option<String>>, D::Error> {
    Option::<String>::deserialize(d).map(Some)
}

impl TaskModels {
    fn slots(&self) -> [(&'static str, &Option<Option<String>>); 2] { [("detector", &self.detector), ("depth", &self.depth)] }

    fn validate(&self) -> Result<(), String> {
        if self.detector.is_none() && self.depth.is_none() { return Err("models needs detector or depth".into()); }
        for (slot, v) in self.slots() {
            if let Some(Some(name)) = v {
                if name.is_empty() || name.len() > 64 { return Err(format!("models.{slot} must be null or a name of 1..64 bytes")); }
            }
        }
        Ok(())
    }

    /// The resident set after the switch (the provider's `resident`, with the slots this names).
    fn after(&self, resident: &ResidentModels) -> ResidentModels {
        ResidentModels {
            detector: self.detector.clone().unwrap_or_else(|| resident.detector.clone()),
            depth: self.depth.clone().unwrap_or_else(|| resident.depth.clone()),
        }
    }

    /// `POST /api/models` body when the provider's resident set differs (every slot this names;
    /// a provider without `models` in its mode always differs).
    pub fn switch_body(&self, mode: &ModeInfo) -> Option<Value> {
        if mode.models.as_ref().is_some_and(|m| self.after(&m.resident) == m.resident) { return None; }
        let mut body = serde_json::Map::new();
        for (slot, v) in self.slots() {
            if let Some(name) = v { body.insert(slot.into(), serde_json::json!(name)); }
        }
        Some(Value::Object(body))
    }
}

/// `requires.model_kind`, compared exactly with the provider's `model.kind` (rebot-vision:
/// `detect`, `segment` for an instance segmentation model, `obb` for an oriented box model).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ModelKind { Detect, Segment, Obb }

impl ModelKind {
    pub fn name(self) -> &'static str {
        match self { Self::Detect => "detect", Self::Segment => "segment", Self::Obb => "obb" }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DepthReq { Any, Metric }

impl TaskRequires {
    pub fn is_empty(&self) -> bool { self.model_kind.is_none() && self.depth.is_none() }

    /// `Err(reason)` when the provider (`mode`) cannot serve the requirements of task `task`.
    pub fn check(&self, task: &str, mode: &ModeInfo) -> Result<(), String> {
        if let Some(kind) = self.model_kind {
            if mode.reported && !mode.yolo_available {
                return Err(format!("task {task} needs model_kind {}, provider has no detector loaded", kind.name()));
            }
            match mode.model_kind.as_deref().filter(|_| mode.reported) {
                None => return Err(format!("task {task} needs model_kind {}, provider does not report model.kind", kind.name())),
                Some(have) if have != kind.name() => return Err(format!("task {task} needs model_kind {}, provider has {have}", kind.name())),
                Some(_) => {}
            }
        }
        if let Some(depth) = self.depth {
            let want = match depth { DepthReq::Any => "any", DepthReq::Metric => "metric" };
            if !mode.reported {
                return Err(format!("task {task} needs depth {want}, provider does not report depth"));
            }
            if !mode.depth_available {
                return Err(format!("task {task} needs depth {want}, provider has no depth pipeline"));
            }
            if depth == DepthReq::Metric && !mode.depth_metric {
                return Err(format!("task {task} needs depth metric, provider depth model is not metric"));
            }
        }
        Ok(())
    }
}

/// Task names: 1..=32 of `[a-z0-9_-]`.
fn valid_task_name(name: &str) -> bool {
    (1..=32).contains(&name.len()) && name.bytes().all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'_' || b == b'-')
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
fn refuse_calibrated_request(base: &GraspSetup, body: &Value) -> Result<(), MotionError> {
    if let Some(name) = body.get("task").and_then(Value::as_str) {
        if let Some(t) = base.tasks.get(name) {
            refuse_calibrated_fields(base, &t.overrides, &format!("grasp task {name}: "))?;
        }
    }
    if let Some(o) = body.get("overrides").and_then(Value::as_object) { refuse_calibrated_fields(base, o, "overrides.")?; }
    Ok(())
}

// ---- detections ----

#[derive(Debug, Clone, PartialEq)]
pub struct Detection {
    pub label: String,
    pub score: f64,
    /// `true` when the upstream raw score was finite and in `[0.0, 1.0]` before clamping.
    pub raw_score_valid: bool,
    /// Normalised [x0, y0, x1, y1].
    pub box_norm: [f64; 4],
    /// Orientation of the object's long side in the image (deg, from +x toward +y, y down,
    /// wrapped to [-90, 90)) and its confidence [0, 1], when the detector reports them.
    pub angle_deg: Option<f64>,
    pub angle_conf: Option<f64>,
    pub grasp_width_px: Option<f64>,
    pub mask_focus: bool,
    pub disp_fg: Option<f64>,
    pub disp_bg: Option<f64>,
    pub depth_conf: Option<f64>,
    pub length_px: Option<f64>,
    pub jaw_clear: Option<bool>,
    /// The depth pipeline's own long-side angle (deg, as `angle_deg`), its confidence and
    /// cross-section width (px), when the depth pipeline ran (rebot-vision `depth_angle_deg`,
    /// `depth_angle_conf`, `depth_width_px`). Never mixed into `angle_deg` / `grasp_width_px`.
    pub depth_angle_deg: Option<f64>,
    pub depth_angle_conf: Option<f64>,
    pub depth_width_px: Option<f64>,
    /// Exterior contour of the segmentation mask (rebot-vision `contour`, [x, y] normalised to the
    /// visible image like `box_norm`, at most [`MAX_CONTOUR_VERTICES`]): None when absent,
    /// Some(Err(reason)) when malformed (that detection's contour only is unusable).
    pub contour_norm: Option<Result<Vec<[f64; 2]>, String>>,
}

/// Most vertices a detection contour may have.
pub const MAX_CONTOUR_VERTICES: usize = 128;

/// A detection's `contour` value: [[x, y], ...] normalised (pixels when the box was in pixels:
/// divided by `size`), 3..=[`MAX_CONTOUR_VERTICES`] finite vertices within [-0.01, 1.01] (clamped
/// to [0, 1]) and a non-zero area.
pub fn parse_contour(v: &Value, pixel: bool, size: [f64; 2]) -> Result<Vec<[f64; 2]>, String> {
    let arr = v.as_array().ok_or("contour_not_an_array")?;
    if arr.len() > MAX_CONTOUR_VERTICES { return Err("contour_too_many_vertices".into()); }
    let (sx, sy) = if pixel { (size[0], size[1]) } else { (1.0, 1.0) };
    let mut out = Vec::with_capacity(arr.len());
    for p in arr {
        let q = p.as_array().filter(|q| q.len() == 2).ok_or("contour_vertex_not_a_pair")?;
        let (Some(x), Some(y)) = (q[0].as_f64(), q[1].as_f64()) else { return Err("contour_vertex_not_numeric".into()) };
        let (x, y) = (x / sx, y / sy);
        if !(x.is_finite() && y.is_finite()) || !(-0.01..=1.01).contains(&x) || !(-0.01..=1.01).contains(&y) {
            return Err("contour_vertex_out_of_range".into());
        }
        out.push([x.clamp(0.0, 1.0), y.clamp(0.0, 1.0)]);
    }
    crate::mview_stereo::footprint::validate_polygon(&out, MAX_CONTOUR_VERTICES).map_err(str::to_string)?;
    Ok(out)
}

#[derive(Debug, Clone, PartialEq)]
pub struct DepthFrameInfo {
    pub metric_model: bool,
    pub table_samples: Vec<[f64; 3]>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct MaskFocus {
    pub label: Option<String>,
    pub box_norm: [f64; 4],
    pub ttl_ms: u64,
}

#[derive(Debug, Clone)]
pub struct DetectionFrame {
    pub frame_id: u64,
    /// Capture time on this host's clock (response time − `age_ms`).
    pub captured_at: Instant,
    /// Conservative capture time for policy runtime staleness checks (`Some` when upstream
    /// reported a valid non-negative finite `age_ms` and subtraction succeeded, penalized for
    /// HTTP response latency beyond any server long-poll `wait_ms`).
    pub policy_captured_at: Option<Instant>,
    pub image_size_px: [f64; 2],
    pub detections: Vec<Detection>,
    pub depth: Option<DepthFrameInfo>,
    /// `model_generation`: the provider's model set the frame ran with, when reported.
    pub model_generation: Option<u64>,
}

/// A provider's vision mode (`GET /api/mode` of rebot-vision).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VisionMode {
    /// `off`, `yolo`, `aruco` or `both`.
    pub mode: String,
    /// Continuous inference on.
    pub enabled: bool,
}

/// Format descriptor advertised in `GET /api/mode` `gray_frames` for loopback peers.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GrayFormatDescriptor {
    pub encoding: String,
    pub format: String,
    pub width: usize,
    pub height: usize,
    pub stride: usize,
    pub ring: usize,
    pub exposure_qualified: bool,
    pub frame_size: Option<[usize; 2]>,
    pub crop: Option<[usize; 4]>,
    pub luma: Option<String>,
    pub luma_range: Option<String>,
}

impl GrayFormatDescriptor {
    /// `gray_frames` of a `GET /api/mode` body: `Ok(None)` when not advertised, `Err` when
    /// advertised but malformed (a wrong type, arity or a missing required field), which must not
    /// be read as "absent".
    fn parse_advertised(v: &Value) -> Result<Option<Self>, String> {
        let Some(g) = v.get("gray_frames") else { return Ok(None) };
        Self::parse(g).map(Some).ok_or_else(|| format!("malformed gray_frames descriptor: {g}"))
    }

    fn parse(g: &Value) -> Option<Self> {
        let g = g.as_object()?;
        let parse_u_array = |k: &str, len: usize| -> Option<Vec<usize>> {
            let a = g.get(k)?.as_array()?;
            if a.len() != len { return None; }
            a.iter().map(|x| usize::try_from(x.as_u64()?).ok()).collect()
        };
        let frame_size = match g.get("frame_size") {
            None => None,
            Some(_) => {
                let fs = parse_u_array("frame_size", 2)?;
                Some([fs[0], fs[1]])
            }
        };
        let crop = match g.get("crop") {
            None => None,
            Some(_) => {
                let c = parse_u_array("crop", 4)?;
                Some([c[0], c[1], c[2], c[3]])
            }
        };
        let luma = match g.get("luma") {
            None => None,
            Some(v) => Some(v.as_str()?.to_string()),
        };
        let luma_range = match g.get("luma_range") {
            None => None,
            Some(v) => Some(v.as_str()?.to_string()),
        };
        Some(Self {
            encoding: g.get("encoding")?.as_str()?.to_string(),
            format: g.get("format")?.as_str()?.to_string(),
            width: usize::try_from(g.get("width")?.as_u64()?).ok()?,
            height: usize::try_from(g.get("height")?.as_u64()?).ok()?,
            stride: usize::try_from(g.get("stride")?.as_u64()?).ok()?,
            ring: usize::try_from(g.get("ring")?.as_u64()?).ok()?,
            // optional (false when absent); any other type than bool makes the descriptor malformed
            exposure_qualified: match g.get("exposure_qualified") {
                None => false,
                Some(v) => v.as_bool()?,
            },
            frame_size,
            crop,
            luma,
            luma_range,
        })
    }

    /// True when the advertised descriptor matches the fixed v1 RGF1 contract.
    pub fn is_supported_v1(&self) -> bool {
        if self.encoding != "RGF1"
            || self.format != "gray8"
            || self.width != GRAY_V1_WIDTH
            || self.height != GRAY_V1_HEIGHT
            || self.stride != GRAY_V1_STRIDE
            || self.ring == 0
        {
            return false;
        }
        if let Some([fw, fh]) = self.frame_size {
            if fw == 0 || fh == 0 {
                return false;
            }
        }
        if let Some([cx, cy, cw, ch]) = self.crop {
            if cw != self.width || ch != self.height {
                return false;
            }
            if let Some([fw, fh]) = self.frame_size {
                if cx.checked_add(cw).is_none_or(|x| x > fw) || cy.checked_add(ch).is_none_or(|y| y > fh) {
                    return false;
                }
            }
        }
        if let Some(ref l) = self.luma {
            if l != GRAY_V1_LUMA {
                return false;
            }
        }
        if let Some(ref r) = self.luma_range {
            if r != GRAY_V1_LUMA_RANGE {
                return false;
            }
        }
        true
    }
}

/// One reading of the provider's `GET /api/mode`, shared by a grasp's task check and its
/// perception negotiation.
#[derive(Debug, Clone, PartialEq)]
pub struct ModeInfo {
    /// The provider has a mode API (false: `/api/mode` 404, an older detector).
    pub reported: bool,
    /// `per_request_pipelines` (perception API 2).
    pub per_request_pipelines: bool,
    /// `available.yolo` (true when not reported): a detector is loaded. An older provider
    /// reports `model.kind` `detect` with the slot empty, so the kind alone does not tell.
    pub yolo_available: bool,
    /// `available.aruco`.
    pub aruco_available: bool,
    /// `available.depth`.
    pub depth_available: bool,
    /// `available.gray` (loopback peers only on rebot-vision).
    pub gray_available: bool,
    /// `gray_frames` format descriptor: `Ok(None)` when not advertised, `Err` when malformed.
    pub gray_format: Result<Option<GrayFormatDescriptor>, String>,
    /// True when the upstream detector endpoint is on loopback (local device).
    pub upstream_loopback: bool,
    /// `depth_model.metric`.
    pub depth_metric: bool,
    /// `model.kind` (`detect` | `segment` | `obb`), when reported (null with the detector slot empty).
    pub model_kind: Option<String>,
    /// `mode` / `enabled` (`Ok(None)` without a mode API); used only without per-request
    /// pipelines.
    pub vision: Result<Option<VisionMode>, String>,
    /// `models` and `ion` (a provider that can switch models, `POST /api/models`).
    pub models: Option<ProviderModels>,
}

/// Names of the provider's loaded models (`models.resident`); None = slot empty.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ResidentModels {
    pub detector: Option<String>,
    pub depth: Option<String>,
}

/// The provider's model state (`GET /api/mode` `models`, `ion`).
#[derive(Debug, Clone, PartialEq)]
pub struct ProviderModels {
    /// +1 per executed swap; every result carries the one it ran with (`model_generation`).
    pub generation: u64,
    pub resident: ResidentModels,
    /// `swap.state`: `idle`, `pending`, `running` or `failed`.
    pub swap_state: String,
    pub swap_error: Option<String>,
    /// Catalogue name -> `ion_mb` (None when not reported).
    pub catalogue: BTreeMap<String, Option<f64>>,
    /// `ion` (`base_mb`, `total_mb`, `reserve_mb`), when reported.
    pub ion: Option<[f64; 3]>,
}

impl ProviderModels {
    fn parse(v: &Value) -> Option<Self> {
        let m = v.get("models")?;
        let name = |s: Option<&Value>| s.and_then(Value::as_str).map(str::to_string);
        let resident = m.get("resident");
        let swap = m.get("swap");
        let catalogue = m.get("catalogue").and_then(Value::as_array).map(|a| a.iter().filter_map(|e| {
            Some((e.get("name")?.as_str()?.to_string(), f(e, &["ion_mb"])))
        }).collect()).unwrap_or_default();
        let ion = v.get("ion").and_then(|i| Some([f(i, &["base_mb"])?, f(i, &["total_mb"])?, f(i, &["reserve_mb"])?]));
        Some(Self {
            generation: m.get("generation").and_then(Value::as_u64)?,
            resident: ResidentModels { detector: name(resident.and_then(|r| r.get("detector"))), depth: name(resident.and_then(|r| r.get("depth"))) },
            swap_state: name(swap.and_then(|s| s.get("state"))).unwrap_or_else(|| "idle".into()),
            swap_error: name(swap.and_then(|s| s.get("error"))),
            catalogue,
            ion,
        })
    }

    /// The ION budget check of `set` from the catalogue (`Err(reason)` when it does not fit);
    /// None when the provider does not report the catalogue's `ion_mb` or its `ion` budget.
    fn budget(&self, set: &ResidentModels) -> Option<Result<(), String>> {
        let [base, total, reserve] = self.ion?;
        let mut need = base + reserve;
        for name in [&set.detector, &set.depth].into_iter().flatten() {
            need += (*self.catalogue.get(name)?)?;
        }
        Some(if need <= total + 1e-9 { Ok(()) } else { Err(refusal_message("ION_BUDGET", Some(need), Some(total))) })
    }
}

/// A provider's refusal (`error`, with `need_mb` / `limit_mb` for `ION_BUDGET`).
fn refusal_message(error: &str, need: Option<f64>, limit: Option<f64>) -> String {
    match (need, limit) {
        (Some(n), Some(l)) => format!("{error} (need {n:.1} MB, limit {l:.1} MB)"),
        _ => error.to_string(),
    }
}

impl ModeInfo {
    /// Parse a `GET /api/mode` body (`None` = 404).
    pub fn parse(v: Option<&Value>) -> Self {
        let Some(v) = v else {
            return Self {
                reported: false, per_request_pipelines: false, yolo_available: true,
                aruco_available: false, depth_available: false, gray_available: false,
                gray_format: Ok(None), upstream_loopback: true, depth_metric: false,
                model_kind: None, vision: Ok(None), models: None,
            };
        };
        let vision = match v.get("mode").and_then(Value::as_str) {
            Some(mode) => Ok(Some(VisionMode { mode: mode.to_string(), enabled: v.get("enabled").and_then(Value::as_bool).unwrap_or(false) })),
            None => Err("vision mode missing".to_string()),
        };
        let avail = |k: &str| v.get("available").and_then(|a| a.get(k)).and_then(Value::as_bool);
        Self {
            reported: true,
            per_request_pipelines: v.get("per_request_pipelines").and_then(Value::as_bool) == Some(true),
            yolo_available: avail("yolo").unwrap_or(true),
            aruco_available: avail("aruco").unwrap_or(false),
            depth_available: avail("depth").unwrap_or(false),
            gray_available: avail("gray").unwrap_or(false),
            gray_format: GrayFormatDescriptor::parse_advertised(v),
            upstream_loopback: true,
            depth_metric: v.get("depth_model").and_then(|d| d.get("metric")).and_then(Value::as_bool).unwrap_or(false),
            model_kind: v.get("model").and_then(|m| m.get("kind")).and_then(Value::as_str).map(str::to_string),
            vision,
            models: ProviderModels::parse(v),
        }
    }

    /// From the separate queries of a source without its own [`DetectionSource::mode_info`]:
    /// a per-request query error is the error; depth/aruco/gray query errors read as unavailable;
    /// the vision mode is read only without per-request pipelines.
    pub async fn compose(source: &(impl DetectionSource + ?Sized)) -> Result<Self, String> {
        let per_request_pipelines = source.per_request_pipelines().await?;
        let aruco_available = source.aruco_available().await.unwrap_or(false);
        let depth_available = source.depth_available().await.unwrap_or(false);
        let gray_available = source.gray_available().await.unwrap_or(false);
        let vision = if per_request_pipelines { Ok(None) } else { source.vision_mode().await };
        Ok(Self {
            reported: per_request_pipelines || matches!(vision, Ok(Some(_))),
            per_request_pipelines, yolo_available: true, aruco_available, depth_available,
            gray_available, gray_format: Ok(None), upstream_loopback: true, depth_metric: false,
            model_kind: None, vision, models: None,
        })
    }
}

/// RGF1 raw gray frame wire constants (rebot-vision `GET /api/frame/gray`).
pub const GRAY_FRAME_MAGIC: &[u8; 4] = b"RGF1";
pub const GRAY_FRAME_ALIGN: usize = 8;
pub const GRAY_MAX_HEADER_BYTES: usize = 16384;
pub const GRAY_V1_WIDTH: usize = 640;
pub const GRAY_V1_HEIGHT: usize = 360;
pub const GRAY_V1_STRIDE: usize = 640;
pub const GRAY_V1_LUMA: &str = "(77*R+150*G+29*B)>>8";
pub const GRAY_V1_LUMA_RANGE: &str = "full_0_255";

/// One decoded raw gray frame from rebot-vision `GET /api/frame/gray`.
#[derive(Debug, Clone, PartialEq)]
pub struct GrayFrame {
    pub instance_id: String,
    pub frame_id: u64,
    pub img: crate::mview_stereo::Gray,
    pub frame_size: [usize; 2],
    pub crop: [usize; 4],
    pub ts_ms: u64,
    pub frame_mono_ms: u64,
    pub wait_mono_ms: u64,
    pub grab_mono_ms: u64,
    pub stored_mono_ms: u64,
    pub convert_ms: f64,
    /// HTTP transfer duration (ms), excluding RGF1 JSON parsing and pixel buffer copy.
    pub fetch_ms: f64,
    pub parse_ms: f64,
}

/// Trigger reply with provider instance identity and gray pipeline status.
#[derive(Debug, Clone)]
pub struct GrayTriggerResult {
    pub frame: DetectionFrame,
    pub markers: Vec<crate::mview_stereo::Marker>,
    pub instance_id: Option<String>,
    pub gray_ran: bool,
    pub gray_ms: Option<f64>,
}

/// Typed errors from fetching or parsing a gray frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GrayFrameError {
    /// 404 FRAME_NOT_STORED (retryable with a new bracket + trigger).
    NotStored(String),
    /// 410 FRAME_EXPIRED: evicted or instance mismatch (retryable with a new bracket + trigger).
    Expired(String),
    /// 503 GRAY_BUSY: concurrent transfer cap reached (retryable).
    Busy(String),
    /// 403 FORBIDDEN: non-loopback peer (non-retryable).
    Forbidden(String),
    /// Network or timeout error (retryable).
    Transport(String),
    /// Malformed RGF1, wrong media type, identity mismatch, or unsupported geometry (non-retryable).
    Protocol(String),
}

impl GrayFrameError {
    pub fn is_retryable(&self) -> bool {
        matches!(self, Self::NotStored(_) | Self::Expired(_) | Self::Busy(_) | Self::Transport(_))
    }

    pub fn retry_reason(&self) -> &'static str {
        match self {
            Self::NotStored(_) => "not_stored",
            Self::Expired(_) => "expired",
            Self::Busy(_) => "busy",
            Self::Forbidden(_) => "forbidden",
            Self::Transport(_) => "transport",
            Self::Protocol(_) => "protocol",
        }
    }
}

impl std::fmt::Display for GrayFrameError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NotStored(s) | Self::Expired(s) | Self::Busy(s) | Self::Forbidden(s)
            | Self::Transport(s) | Self::Protocol(s) => f.write_str(s),
        }
    }
}

fn is_octet_stream(content_type: Option<&str>) -> bool {
    let Some(ct) = content_type else { return false; };
    let media = ct.split(';').next().unwrap_or("").trim();
    media.eq_ignore_ascii_case("application/octet-stream")
}

fn valid_instance_token(s: &str) -> bool {
    !s.is_empty() && s.len() <= 64 && s.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_')
}

/// Strictly parse an `RGF1` gray frame body (`"RGF1"` + u32 LE `N` + `N` space-padded UTF-8 JSON
/// bytes aligned to 8 + exact `height * stride` pixel bytes). Validates bounds, alignment,
/// checked arithmetic, required v1 metadata (encoding, source dims, crop bounds, luma, timing),
/// fixed v1 geometry (`640x360`, `stride=640`), exact payload length, and expected identity
/// before allocating the pixel buffer.
pub fn parse_gray_frame(
    body: &[u8],
    expected_instance_id: Option<&str>,
    expected_frame_id: Option<u64>,
) -> Result<GrayFrame, GrayFrameError> {
    parse_gray_frame_with_format(body, expected_instance_id, expected_frame_id, None)
}

/// Strict [`parse_gray_frame`] that additionally checks the frame metadata against a negotiated
/// [`GrayFormatDescriptor`] from `GET /api/mode`.
pub fn parse_gray_frame_with_format(
    body: &[u8],
    expected_instance_id: Option<&str>,
    expected_frame_id: Option<u64>,
    expected_format: Option<&GrayFormatDescriptor>,
) -> Result<GrayFrame, GrayFrameError> {
    let t0 = Instant::now();
    let proto = |m: String| GrayFrameError::Protocol(m);
    if body.len() < 8 {
        return Err(proto(format!("RGF1 body truncated ({} bytes)", body.len())));
    }
    if &body[..4] != GRAY_FRAME_MAGIC {
        return Err(proto("RGF1 bad magic".into()));
    }
    let header_len = u32::from_le_bytes([body[4], body[5], body[6], body[7]]) as usize;
    if header_len == 0 || header_len > GRAY_MAX_HEADER_BYTES {
        return Err(proto(format!("RGF1 invalid header length {header_len} (max {GRAY_MAX_HEADER_BYTES})")));
    }
    let off = 8usize
        .checked_add(header_len)
        .ok_or_else(|| proto("RGF1 header offset overflow".into()))?;
    if !off.is_multiple_of(GRAY_FRAME_ALIGN) {
        return Err(proto(format!("RGF1 pixel offset {off} is not {GRAY_FRAME_ALIGN}-byte aligned")));
    }
    if off > body.len() {
        return Err(proto(format!("RGF1 header truncated (need {off} bytes, got {})", body.len())));
    }
    let hdr_bytes = &body[8..off];
    let hdr_str = std::str::from_utf8(hdr_bytes).map_err(|_| proto("RGF1 header is not valid UTF-8".into()))?;
    // Trailing padding must be ASCII spaces only (no arbitrary trailing junk).
    let trimmed = hdr_str.trim_end_matches(' ');
    if trimmed.is_empty() || trimmed.bytes().last() != Some(b'}') {
        return Err(proto("RGF1 header padding or JSON terminator invalid".into()));
    }
    let v: Value = serde_json::from_str(trimmed).map_err(|e| proto(format!("RGF1 header JSON invalid: {e}")))?;
    if !v.is_object() {
        return Err(proto("RGF1 header must be a JSON object".into()));
    }
    if v.get("api_version").and_then(Value::as_u64) != Some(2) {
        return Err(proto("RGF1 unsupported api_version (expected 2)".into()));
    }
    if v.get("encoding").and_then(Value::as_str) != Some("RGF1") {
        return Err(proto("RGF1 missing or unsupported encoding (expected RGF1)".into()));
    }
    if v.get("format").and_then(Value::as_str) != Some("gray8") {
        return Err(proto("RGF1 unsupported format (expected gray8)".into()));
    }
    if v.get("frame_format").and_then(Value::as_str) != Some("RGB888") {
        return Err(proto("RGF1 missing or unsupported frame_format (expected RGB888)".into()));
    }
    if v.get("luma").and_then(Value::as_str) != Some(GRAY_V1_LUMA) {
        return Err(proto("RGF1 missing or unsupported luma equation".into()));
    }
    if v.get("luma_range").and_then(Value::as_str) != Some(GRAY_V1_LUMA_RANGE) {
        return Err(proto("RGF1 missing or unsupported luma_range (expected full_0_255)".into()));
    }
    let instance_id = v
        .get("instance_id")
        .and_then(Value::as_str)
        .filter(|s| valid_instance_token(s))
        .ok_or_else(|| proto("RGF1 missing or invalid instance_id".into()))?
        .to_string();
    let frame_id = v
        .get("frame_id")
        .and_then(Value::as_u64)
        .ok_or_else(|| proto("RGF1 missing frame_id".into()))?;
    if let Some(exp) = expected_instance_id {
        if instance_id != exp {
            return Err(proto(format!("RGF1 instance_id mismatch: expected {exp}, got {instance_id}")));
        }
    }
    if let Some(exp) = expected_frame_id {
        if frame_id != exp {
            return Err(proto(format!("RGF1 frame_id mismatch: expected {exp}, got {frame_id}")));
        }
    }
    let req_u64 = |k: &str| -> Result<u64, GrayFrameError> {
        v.get(k).and_then(Value::as_u64).ok_or_else(|| proto(format!("RGF1 missing or invalid {k}")))
    };
    let req_usize = |k: &str| -> Result<usize, GrayFrameError> {
        let n = req_u64(k)?;
        usize::try_from(n).map_err(|_| proto(format!("RGF1 {k} overflow")))
    };
    let req_usize_array = |k: &str, len: usize| -> Result<Vec<usize>, GrayFrameError> {
        let arr = v.get(k).and_then(Value::as_array).ok_or_else(|| proto(format!("RGF1 missing {k}")))?;
        if arr.len() != len {
            return Err(proto(format!("RGF1 {k} must have {len} elements")));
        }
        arr.iter()
            .map(|el| {
                let n = el.as_u64().ok_or_else(|| proto(format!("RGF1 {k} element must be non-negative integer")))?;
                usize::try_from(n).map_err(|_| proto(format!("RGF1 {k} element overflow")))
            })
            .collect()
    };
    let width = req_usize("width")?;
    let height = req_usize("height")?;
    let stride = req_usize("stride")?;
    if width != GRAY_V1_WIDTH || height != GRAY_V1_HEIGHT || stride != GRAY_V1_STRIDE {
        return Err(proto(format!(
            "RGF1 unsupported geometry {width}x{height} stride {stride} (expected {GRAY_V1_WIDTH}x{GRAY_V1_HEIGHT} stride {GRAY_V1_STRIDE})"
        )));
    }
    let fs_vec = req_usize_array("frame_size", 2)?;
    let frame_size = [fs_vec[0], fs_vec[1]];
    if frame_size[0] == 0 || frame_size[1] == 0 {
        return Err(proto(format!("RGF1 invalid frame_size {}x{}", frame_size[0], frame_size[1])));
    }
    let crop_vec = req_usize_array("crop", 4)?;
    let crop = [crop_vec[0], crop_vec[1], crop_vec[2], crop_vec[3]];
    if crop[2] != width || crop[3] != height {
        return Err(proto(format!(
            "RGF1 crop size {}x{} does not match output {}x{}",
            crop[2], crop[3], width, height
        )));
    }
    if crop[0].checked_add(crop[2]).is_none_or(|x| x > frame_size[0])
        || crop[1].checked_add(crop[3]).is_none_or(|y| y > frame_size[1])
    {
        return Err(proto(format!(
            "RGF1 crop {:?} exceeds frame_size {:?}",
            crop, frame_size
        )));
    }
    if let Some(fmt) = expected_format {
        if fmt.encoding != "RGF1" || fmt.format != "gray8" {
            return Err(proto("RGF1 negotiated descriptor encoding/format incompatible".into()));
        }
        if width != fmt.width || height != fmt.height || stride != fmt.stride {
            return Err(proto(format!(
                "RGF1 geometry {width}x{height} stride {stride} does not match negotiated {}x{} stride {}",
                fmt.width, fmt.height, fmt.stride
            )));
        }
        if let Some(exp_fs) = fmt.frame_size {
            if frame_size != exp_fs {
                return Err(proto(format!(
                    "RGF1 frame_size {:?} does not match negotiated {:?}",
                    frame_size, exp_fs
                )));
            }
        }
        if let Some(exp_crop) = fmt.crop {
            if crop != exp_crop {
                return Err(proto(format!(
                    "RGF1 crop {:?} does not match negotiated {:?}",
                    crop, exp_crop
                )));
            }
        }
    }
    let _channel = req_u64("channel")?;
    let _model_generation = req_u64("model_generation")?;
    let ts_ms = req_u64("ts_ms")?;
    let frame_mono_ms = req_u64("frame_mono_ms")?;
    let wait_mono_ms = req_u64("wait_mono_ms")?;
    let grab_mono_ms = req_u64("grab_mono_ms")?;
    let stored_mono_ms = req_u64("stored_mono_ms")?;
    if stored_mono_ms < grab_mono_ms {
        return Err(proto(format!("RGF1 stored_mono_ms {stored_mono_ms} precedes grab_mono_ms {grab_mono_ms}")));
    }
    let convert_ms = v
        .get("convert_ms")
        .and_then(Value::as_f64)
        .filter(|x| x.is_finite() && *x >= 0.0)
        .ok_or_else(|| proto("RGF1 missing or invalid convert_ms".into()))?;

    let payload_len = height
        .checked_mul(stride)
        .ok_or_else(|| proto("RGF1 payload size overflow".into()))?;
    let total = off
        .checked_add(payload_len)
        .ok_or_else(|| proto("RGF1 total size overflow".into()))?;
    if body.len() != total {
        return Err(proto(format!(
            "RGF1 payload length mismatch: expected {total} bytes total, got {}",
            body.len()
        )));
    }
    let img = crate::mview_stereo::Gray::new(width, height, body[off..total].to_vec())
        .ok_or_else(|| proto("RGF1 pixel buffer allocation mismatch".into()))?;
    let parse_ms = t0.elapsed().as_secs_f64() * 1000.0;
    Ok(GrayFrame {
        instance_id,
        frame_id,
        img,
        frame_size,
        crop,
        ts_ms,
        frame_mono_ms,
        wait_mono_ms,
        grab_mono_ms,
        stored_mono_ms,
        convert_ms,
        fetch_ms: 0.0,
        parse_ms,
    })
}

/// The mode a grasp needs: YOLO, plus ArUco when it was running.
pub fn grasp_mode(current: &str) -> &'static str {
    match current {
        "aruco" | "both" => "both",
        _ => "yolo",
    }
}

/// Pipelines a grasp triggers per centring step. A marker target would add `"aruco"`.
pub const GRASP_PIPELINES: &[&str] = &["yolo"];
/// How long one centring trigger waits for a fresh frame.
pub(crate) const TRIGGER_WAIT_MS: u64 = 1000;

/// Where the orchestrator gets detections; tests inject a simulated camera.
pub trait DetectionSource: Send + Sync {
    /// The provider runs the pipelines a trigger names (perception API 2). The grasp then
    /// triggers [`GRASP_PIPELINES`] per centring step and leaves the vision mode alone.
    fn per_request_pipelines(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async { Ok(false) })
    }
    /// Whether depth inference is available on the provider (`GET /api/mode` available.depth).
    fn depth_available(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async { Ok(false) })
    }
    /// One inference running exactly `pipelines` on a frame newer than the call, waiting up to
    /// `wait_ms`. `Ok(None)` = no fresh frame in time.
    fn trigger(&self, pipelines: &'static [&'static str], wait_ms: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        self.trigger_with_focus(pipelines, wait_ms, None)
    }
    /// Whether the provider runs the ArUco pipeline (`GET /api/mode` available.aruco).
    fn aruco_available(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async { Ok(false) })
    }
    /// Whether raw gray frames are available (`GET /api/mode` available.gray, loopback only).
    fn gray_available(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async { Ok(false) })
    }
    /// [`DetectionSource::trigger`] that also returns the ArUco markers of the frame (pixels of
    /// the frame's `image_size_px`) when `pipelines` names `"aruco"`. Default: no markers.
    fn trigger_markers(&self, pipelines: &'static [&'static str], wait_ms: u64) -> BoxFuture<'_, Result<Option<(DetectionFrame, Vec<crate::mview_stereo::Marker>)>, String>> {
        Box::pin(async move { Ok(self.trigger(pipelines, wait_ms).await?.map(|f| (f, Vec::new()))) })
    }
    /// Trigger returning detections, optional markers, and gray frame identity (`instance_id`,
    /// `pipelines.gray.ran`). Default delegates to [`Self::trigger_markers`] without gray.
    fn trigger_gray(&self, pipelines: &'static [&'static str], wait_ms: u64) -> BoxFuture<'_, Result<Option<GrayTriggerResult>, String>> {
        Box::pin(async move {
            Ok(self.trigger_markers(pipelines, wait_ms).await?.map(|(frame, markers)| GrayTriggerResult {
                frame, markers, instance_id: None, gray_ran: false, gray_ms: None,
            }))
        })
    }
    /// Fetch the exact raw gray frame `(instance_id, frame_id)` from `GET /api/frame/gray`.
    fn gray_frame(&self, _instance_id: &str, _frame_id: u64) -> BoxFuture<'_, Result<GrayFrame, GrayFrameError>> {
        Box::pin(async { Err(GrayFrameError::Protocol("gray frames not supported by this detection source".to_string())) })
    }
    /// One inference with optional mask focus.
    fn trigger_with_focus(&self, _pipelines: &'static [&'static str], _wait_ms: u64, _focus: Option<&MaskFocus>) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        Box::pin(async { Err("per-request pipelines not supported".to_string()) })
    }
    /// Sets mask focus for subsequent frames until expiry or clears it (`ttl_ms` = 0).
    fn set_mask_focus(&self, _label: Option<String>, _box_norm: [f64; 4], _ttl_ms: u64) -> BoxFuture<'_, Result<(), String>> {
        Box::pin(async { Ok(()) })
    }
    fn inference_enabled(&self) -> BoxFuture<'_, Result<bool, String>>;
    fn set_inference(&self, enabled: bool) -> BoxFuture<'_, Result<(), String>>;
    /// The provider's vision mode; `Ok(None)` when it has no mode API (older detectors),
    /// in which case the grasp only switches inference on and off.
    fn vision_mode(&self) -> BoxFuture<'_, Result<Option<VisionMode>, String>> {
        Box::pin(async { Ok(None) })
    }
    /// Sets mode and continuous inference in one request.
    fn set_vision_mode(&self, _mode: &VisionMode) -> BoxFuture<'_, Result<(), String>> {
        Box::pin(async { Err("vision mode not supported".to_string()) })
    }
    /// `POST /api/models` with `body`: the reply's status and JSON body (`Err` = unreachable or
    /// timed out). Default: a provider without the route (404).
    fn set_models(&self, _body: &Value) -> BoxFuture<'_, Result<(u16, Value), String>> {
        Box::pin(async { Ok((404, Value::Null)) })
    }
    /// Everything a grasp reads from the provider's mode, once per grasp (one `GET /api/mode`
    /// for the HTTP source). `Err` = provider unreachable.
    fn mode_info(&self) -> BoxFuture<'_, Result<ModeInfo, String>> {
        Box::pin(ModeInfo::compose(self))
    }
    /// The next frame with `frame_id > after` (any frame when `after` is None), waiting up to
    /// `wait_ms`. `Ok(None)` = no new frame in time.
    fn next_frame(&self, after: Option<u64>, wait_ms: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>>;
    /// The current camera image as JPEG (rebot-vision `GET /snapshot.jpg`: the frame letterboxed
    /// into a square, used by the multi-view measurement). Default: not available.
    fn snapshot_jpeg(&self) -> BoxFuture<'_, Result<Vec<u8>, String>> {
        Box::pin(async { Err("camera snapshots not supported by this detection source".to_string()) })
    }
    /// Keep the camera gate open without inference for `ttl_ms` (rebot-vision camera hold:
    /// `POST /api/subscribe {"client", "pipelines": [], "ttl_ms"}`), so a trigger is served by the
    /// next camera frame. Calling it again with the same `client` refreshes the same hold.
    /// `Ok(Some(ticket))` = held (the id may change when the provider restarted: keep the
    /// latest); `Ok(None)` = the provider has no camera hold (older rebot-vision answers 400, a
    /// provider without subscriptions 404). Default: no camera hold.
    fn camera_hold(&self, _client: &str, _ttl_ms: u64) -> BoxFuture<'_, Result<Option<CameraHoldTicket>, String>> {
        Box::pin(async { Ok(None) })
    }
    /// End `client`'s camera hold `ticket` (`DELETE /api/subscribe/<id>?client=&instance_id=`):
    /// a rebot-vision with owner checks deletes it only when it is still this client's
    /// subscription of the same provider instance (after a restart the ids start again at s1).
    fn camera_release(&self, _ticket: &CameraHoldTicket, _client: &str) -> BoxFuture<'_, Result<(), String>> {
        Box::pin(async { Ok(()) })
    }
}

/// A camera hold taken by [`DetectionSource::camera_hold`]: the subscription id and the
/// provider instance that issued it (rebot-vision subscription bodies' `instance_id`; None from
/// a provider that does not report it).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CameraHoldTicket {
    pub id: String,
    pub instance_id: Option<String>,
}

/// The camera app's detection API (the same upstream armd proxies under /api/v1/detection).
pub struct HttpDetectionSource {
    pub upstream: DetectorUpstream,
    /// Size used for detections reported in pixels when the frame carries no size.
    pub image_size_px: [f64; 2],
}

impl HttpDetectionSource {
    /// A result envelope (`latest` or `trigger`) as a frame; `None` when it carries no new frame.
    fn frame(
        &self,
        v: &Value,
        after: Option<u64>,
        req_started: Instant,
        received: Instant,
        wait_ms: u64,
    ) -> Option<DetectionFrame> {
        if v.get("timeout").and_then(Value::as_bool) == Some(true) || v.get("success").and_then(Value::as_bool) == Some(false) {
            return None;
        }
        let frame_id = v.get("frame_id").and_then(Value::as_u64)?;
        if after.is_some_and(|a| frame_id <= a) { return None; }
        let age_dur = v
            .get("age_ms")
            .and_then(Value::as_f64)
            .filter(|a| a.is_finite() && *a >= 0.0)
            .and_then(|age_ms| Duration::try_from_secs_f64(age_ms / 1000.0).ok());
        let captured_at = age_dur
            .and_then(|dur| received.checked_sub(dur))
            .unwrap_or(received);
        let conservative_extra_delay = if after.is_none() || wait_ms == 0 {
            received.saturating_duration_since(req_started)
        } else {
            received
                .saturating_duration_since(req_started)
                .saturating_sub(Duration::from_millis(wait_ms))
        };
        let policy_captured_at = age_dur
            .and_then(|dur| dur.checked_add(conservative_extra_delay))
            .and_then(|total_delay| received.checked_sub(total_delay));
        let size = match (v.get("width").and_then(Value::as_f64), v.get("height").and_then(Value::as_f64)) {
            (Some(w), Some(h)) if w.is_finite() && h.is_finite() && w > 0.0 && h > 0.0 => [w, h],
            _ => self.image_size_px,
        };
        let depth = v.get("depth").map(|d| {
            let metric_model = d.get("metric_model").and_then(Value::as_bool).unwrap_or(false);
            let table_samples = d.get("table_samples").and_then(Value::as_array).map(|arr| {
                arr.iter().filter_map(|item| {
                    let s = item.as_array()?;
                    if s.len() >= 3 {
                        let u = s[0].as_f64()?;
                        let v = s[1].as_f64()?;
                        let disp = s[2].as_f64()?;
                        if u.is_finite() && v.is_finite() && disp.is_finite() {
                            return Some([u, v, disp]);
                        }
                    }
                    None
                }).collect::<Vec<[f64; 3]>>()
            }).unwrap_or_default();
            DepthFrameInfo { metric_model, table_samples }
        });
        let detections = parse_detections(v.get("detections").unwrap_or(&Value::Null), size);
        let model_generation = v.get("model_generation").and_then(Value::as_u64);
        Some(DetectionFrame { frame_id, captured_at, policy_captured_at, image_size_px: size, detections, depth, model_generation })
    }
}

/// Rescales a long-side angle (deg) and the width across it (px) from a frame scaled by
/// (rx, ry) to the camera model's image; returns the scale along the long side (for a length),
/// None without an angle (the width then scales isotropically).
fn rescale_angle_width(angle_deg: &mut Option<f64>, width_px: &mut Option<f64>, rx: f64, ry: f64, cam: &CamModel) -> Option<f64> {
    let isotropic = (rx * ry).sqrt();
    let Some(deg) = *angle_deg else {
        if let Some(w) = width_px.as_mut() {
            *w *= isotropic;
        }
        return None;
    };
    let (sin_t, cos_t) = deg.to_radians().sin_cos();
    let sl = (cos_t * rx).hypot(sin_t * ry);
    let theta_cam = (sin_t * ry).atan2(cos_t * rx);
    let deg_cam = (theta_cam.to_degrees() + 90.0).rem_euclid(180.0) - 90.0;
    *angle_deg = Some(deg_cam);
    let (sin_cam, cos_cam) = deg_cam.to_radians().sin_cos();
    let denom = (-sin_cam / cam.fx).hypot(cos_cam / cam.fy);
    let sw = if denom > 1e-12 {
        (-sin_t * rx / cam.fx).hypot(cos_t * ry / cam.fy) / denom
    } else {
        isotropic
    };
    if let Some(w) = width_px.as_mut() {
        *w *= sw;
    }
    Some(sl)
}

pub fn rescale_frame_to_cam(frame: &mut DetectionFrame, cam: &CamModel) {
    if !(frame.image_size_px[0] > 0.0 && frame.image_size_px[1] > 0.0 && cam.size[0] > 0.0 && cam.size[1] > 0.0) {
        return;
    }
    let rx = cam.size[0] / frame.image_size_px[0];
    let ry = cam.size[1] / frame.image_size_px[1];
    if (rx - 1.0).abs() <= 1e-6 && (ry - 1.0).abs() <= 1e-6 {
        frame.image_size_px = cam.size;
        return;
    }
    if let Some(ref mut d) = frame.depth {
        for sample in &mut d.table_samples {
            sample[0] *= rx;
            sample[1] *= ry;
        }
    }
    for det in &mut frame.detections {
        let isotropic = (rx * ry).sqrt();
        let sl_mask = rescale_angle_width(&mut det.angle_deg, &mut det.grasp_width_px, rx, ry, cam);
        let sl_depth = rescale_angle_width(&mut det.depth_angle_deg, &mut det.depth_width_px, rx, ry, cam);
        // the length follows the detector angle, else the depth angle (the angle the vision
        // pipeline measured it along)
        if let Some(l) = det.length_px.as_mut() {
            *l *= sl_mask.or(sl_depth).unwrap_or(isotropic);
        }
    }
    frame.image_size_px = cam.size;
}

/// A subscription id, instance id or client name armd puts into a query: 1..64 of [A-Za-z0-9_-].
fn hold_token_ok(s: &str) -> bool {
    !s.is_empty() && s.len() <= 64 && s.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-')
}

impl DetectionSource for HttpDetectionSource {
    fn camera_hold(&self, client: &str, ttl_ms: u64) -> BoxFuture<'_, Result<Option<CameraHoldTicket>, String>> {
        let body = serde_json::json!({"client": client, "pipelines": [], "ttl_ms": ttl_ms}).to_string();
        Box::pin(async move {
            let (status, v) = crate::api::proxy::request_status_json(&self.upstream, "POST", "/api/subscribe", Some(body.as_bytes()), Duration::from_secs(2)).await?;
            if status == axum::http::StatusCode::BAD_REQUEST || status == axum::http::StatusCode::NOT_FOUND { return Ok(None); }
            if !status.is_success() { return Err(format!("camera hold: detector returned {status}")); }
            // an older provider that took [] as an ordinary subscription does not report a hold
            if v.get("hold").and_then(Value::as_bool) != Some(true) { return Ok(None); }
            let id = v.get("id").and_then(Value::as_str).filter(|id| hold_token_ok(id)).ok_or_else(|| "camera hold: reply without a valid id".to_string())?;
            let instance_id = v.get("instance_id").and_then(Value::as_str).filter(|i| hold_token_ok(i)).map(str::to_string);
            Ok(Some(CameraHoldTicket { id: id.to_string(), instance_id }))
        })
    }
    fn camera_release(&self, ticket: &CameraHoldTicket, client: &str) -> BoxFuture<'_, Result<(), String>> {
        // an older rebot-vision ignores the query and deletes by id
        let mut path = format!("/api/subscribe/{}?client={client}", ticket.id);
        if let Some(i) = &ticket.instance_id { path.push_str("&instance_id="); path.push_str(i); }
        let (ok_client, id, client) = (hold_token_ok(client), ticket.id.clone(), client.to_string());
        Box::pin(async move {
            if !ok_client { return Err(format!("camera release: client {client:?} is not [A-Za-z0-9_-]{{1,64}}")); }
            let (status, v) = crate::api::proxy::request_status_json(&self.upstream, "DELETE", &path, None, Duration::from_secs(2)).await?;
            // 404: already expired; 409: no longer ours (another client's id, or a restarted
            // provider whose ids started again): nothing of this hold is left to end
            if status == axum::http::StatusCode::CONFLICT {
                let why = v.get("error").and_then(serde_json::Value::as_str).unwrap_or("409");
                tracing::info!("camera release {id}: not deleted ({why})");
            }
            if status.is_success() || status == axum::http::StatusCode::NOT_FOUND || status == axum::http::StatusCode::CONFLICT { Ok(()) } else { Err(format!("camera release: detector returned {status}")) }
        })
    }
    fn mode_info(&self) -> BoxFuture<'_, Result<ModeInfo, String>> {
        Box::pin(async move {
            let v = crate::api::proxy::request_json_opt(&self.upstream, "GET", "/api/mode", None, Duration::from_secs(2)).await?;
            let mut info = ModeInfo::parse(v.as_ref());
            info.upstream_loopback = self.upstream.is_loopback();
            Ok(info)
        })
    }
    fn set_models(&self, body: &Value) -> BoxFuture<'_, Result<(u16, Value), String>> {
        let body = body.to_string();
        Box::pin(async move {
            let (status, v) = crate::api::proxy::request_status_json(&self.upstream, "POST", "/api/models", Some(body.as_bytes()), Duration::from_secs(2)).await?;
            Ok((status.as_u16(), v))
        })
    }
    fn per_request_pipelines(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async move {
            let v = crate::api::proxy::request_json_opt(&self.upstream, "GET", "/api/mode", None, Duration::from_secs(2)).await?;
            Ok(v.and_then(|v| v.get("per_request_pipelines").and_then(Value::as_bool)) == Some(true))
        })
    }
    fn depth_available(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async move {
            let v = crate::api::proxy::request_json_opt(&self.upstream, "GET", "/api/mode", None, Duration::from_secs(2)).await?;
            Ok(v.as_ref()
                .and_then(|v| v.get("available"))
                .and_then(|a| a.get("depth"))
                .and_then(Value::as_bool)
                .unwrap_or(false))
        })
    }
    fn trigger(&self, pipelines: &'static [&'static str], wait_ms: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        self.trigger_with_focus(pipelines, wait_ms, None)
    }
    fn aruco_available(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async move {
            let v = crate::api::proxy::request_json_opt(&self.upstream, "GET", "/api/mode", None, Duration::from_secs(2)).await?;
            Ok(v.as_ref().and_then(|v| v.get("available")).and_then(|a| a.get("aruco")).and_then(Value::as_bool).unwrap_or(false))
        })
    }
    fn gray_available(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async move {
            if !self.upstream.is_loopback() {
                return Ok(false);
            }
            let v = crate::api::proxy::request_json_opt(&self.upstream, "GET", "/api/mode", None, Duration::from_secs(2)).await?;
            Ok(v.as_ref().and_then(|v| v.get("available")).and_then(|a| a.get("gray")).and_then(Value::as_bool).unwrap_or(false))
        })
    }
    fn trigger_markers(&self, pipelines: &'static [&'static str], wait_ms: u64) -> BoxFuture<'_, Result<Option<(DetectionFrame, Vec<crate::mview_stereo::Marker>)>, String>> {
        Box::pin(async move {
            let wait_ms = wait_ms.min(5000);
            let body = serde_json::json!({"pipelines": pipelines, "wait_ms": wait_ms}).to_string();
            let req_started = Instant::now();
            let v = crate::api::proxy::request_json(&self.upstream, "POST", "/api/trigger", Some(body.as_bytes()), Duration::from_millis(wait_ms + 2000)).await?;
            Ok(self.frame(&v, None, req_started, Instant::now(), wait_ms).map(|f| (f, parse_markers(v.get("markers").unwrap_or(&Value::Null)))))
        })
    }
    fn trigger_gray(&self, pipelines: &'static [&'static str], wait_ms: u64) -> BoxFuture<'_, Result<Option<GrayTriggerResult>, String>> {
        Box::pin(async move {
            let wait_ms = wait_ms.min(5000);
            let body = serde_json::json!({"pipelines": pipelines, "wait_ms": wait_ms}).to_string();
            let req_started = Instant::now();
            let v = crate::api::proxy::request_json(&self.upstream, "POST", "/api/trigger", Some(body.as_bytes()), Duration::from_millis(wait_ms + 2000)).await?;
            let Some(frame) = self.frame(&v, None, req_started, Instant::now(), wait_ms) else {
                return Ok(None);
            };
            let markers = parse_markers(v.get("markers").unwrap_or(&Value::Null));
            let instance_id = v.get("instance_id").and_then(Value::as_str).map(str::to_string);
            let gray_obj = v.get("pipelines").and_then(|p| p.get("gray"));
            let gray_ran = gray_obj.and_then(|g| g.get("ran")).and_then(Value::as_bool).unwrap_or(false);
            let gray_ms = gray_obj.and_then(|g| g.get("ms")).and_then(Value::as_f64).filter(|x| x.is_finite());
            Ok(Some(GrayTriggerResult { frame, markers, instance_id, gray_ran, gray_ms }))
        })
    }
    fn gray_frame(&self, instance_id: &str, frame_id: u64) -> BoxFuture<'_, Result<GrayFrame, GrayFrameError>> {
        let instance_id = instance_id.to_string();
        Box::pin(async move {
            if !valid_instance_token(&instance_id) {
                return Err(GrayFrameError::Protocol("invalid gray frame instance_id".into()));
            }
            let path = format!("/api/frame/gray?instance_id={instance_id}&frame_id={frame_id}");
            let t_fetch = Instant::now();
            let (status, content_type, body) = crate::api::proxy::request_status_bytes(
                &self.upstream,
                "GET",
                &path,
                Duration::from_secs(2),
            )
            .await
            .map_err(|e| GrayFrameError::Transport(format!("gray frame {frame_id}: {e}")))?;
            let fetch_ms = t_fetch.elapsed().as_secs_f64() * 1000.0;
            if status != axum::http::StatusCode::OK {
                let err_code = serde_json::from_slice::<Value>(&body)
                    .ok()
                    .and_then(|v| v.get("error").and_then(Value::as_str).map(str::to_string))
                    .unwrap_or_else(|| status.canonical_reason().unwrap_or("error").to_string());
                let msg = format!("gray frame {frame_id}: {} {err_code}", status.as_u16());
                return Err(match status {
                    axum::http::StatusCode::NOT_FOUND => GrayFrameError::NotStored(msg),
                    axum::http::StatusCode::GONE => GrayFrameError::Expired(msg),
                    axum::http::StatusCode::FORBIDDEN => GrayFrameError::Forbidden(msg),
                    axum::http::StatusCode::SERVICE_UNAVAILABLE => GrayFrameError::Busy(msg),
                    _ => GrayFrameError::Protocol(msg),
                });
            }
            if !is_octet_stream(content_type.as_deref()) {
                return Err(GrayFrameError::Protocol(format!(
                    "gray frame {frame_id}: unexpected Content-Type {:?} (expected application/octet-stream)",
                    content_type.as_deref().unwrap_or("<missing>")
                )));
            }
            let mut frame = parse_gray_frame(&body, Some(&instance_id), Some(frame_id))?;
            frame.fetch_ms = fetch_ms;
            Ok(frame)
        })
    }
    fn trigger_with_focus(&self, pipelines: &'static [&'static str], wait_ms: u64, focus: Option<&MaskFocus>) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        let focus = focus.cloned();
        Box::pin(async move {
            let wait_ms = wait_ms.min(5000);
            let mut obj = serde_json::json!({"pipelines": pipelines, "wait_ms": wait_ms});
            if let Some(ref f) = focus {
                let mut mf = serde_json::json!({
                    "box_norm": f.box_norm,
                    "ttl_ms": f.ttl_ms,
                });
                if let Some(ref l) = f.label {
                    mf["label"] = serde_json::Value::String(l.clone());
                }
                obj["mask_focus"] = mf;
            }
            let body = obj.to_string();
            let req_started = Instant::now();
            let v = match crate::api::proxy::request_json(&self.upstream, "POST", "/api/trigger", Some(body.as_bytes()), Duration::from_millis(wait_ms + 2000)).await {
                Ok(v) => v,
                // an older rebot-vision ignores the unknown field; retry only on a rejected body
                Err(e) if focus.is_some() && e.starts_with("detector returned 400") => {
                    tracing::debug!("Trigger with mask_focus failed ({e}), retrying without focus");
                    let fallback = serde_json::json!({"pipelines": pipelines, "wait_ms": wait_ms}).to_string();
                    crate::api::proxy::request_json(&self.upstream, "POST", "/api/trigger", Some(fallback.as_bytes()), Duration::from_millis(wait_ms + 2000)).await?
                }
                Err(e) => return Err(e),
            };
            Ok(self.frame(&v, None, req_started, Instant::now(), wait_ms))
        })
    }
    fn set_mask_focus(&self, label: Option<String>, box_norm: [f64; 4], ttl_ms: u64) -> BoxFuture<'_, Result<(), String>> {
        Box::pin(async move {
            let body = if ttl_ms == 0 {
                serde_json::json!({"ttl_ms": 0}).to_string()
            } else {
                let mut j = serde_json::json!({
                    "box_norm": box_norm,
                    "ttl_ms": ttl_ms,
                });
                if let Some(l) = label {
                    j["label"] = serde_json::Value::String(l);
                }
                j.to_string()
            };
            match crate::api::proxy::request_json_opt(&self.upstream, "POST", "/api/mask_focus", Some(body.as_bytes()), Duration::from_secs(2)).await {
                Ok(_) => Ok(()),
                Err(e) => {
                    tracing::debug!("POST /api/mask_focus failed ({e}), ignored");
                    Ok(())
                }
            }
        })
    }
    fn inference_enabled(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async move {
            let v = crate::api::proxy::request_json(&self.upstream, "GET", "/api/inference", None, Duration::from_secs(2)).await?;
            v.get("enabled").and_then(Value::as_bool).ok_or_else(|| "inference state missing".to_string())
        })
    }
    fn set_inference(&self, enabled: bool) -> BoxFuture<'_, Result<(), String>> {
        Box::pin(async move {
            let body = serde_json::json!({"enabled": enabled}).to_string();
            crate::api::proxy::request_json(&self.upstream, "POST", "/api/inference", Some(body.as_bytes()), Duration::from_secs(2)).await.map(|_| ())
        })
    }
    fn vision_mode(&self) -> BoxFuture<'_, Result<Option<VisionMode>, String>> {
        Box::pin(async move {
            let Some(v) = crate::api::proxy::request_json_opt(&self.upstream, "GET", "/api/mode", None, Duration::from_secs(2)).await? else {
                return Ok(None);
            };
            let mode = v.get("mode").and_then(Value::as_str).ok_or_else(|| "vision mode missing".to_string())?;
            Ok(Some(VisionMode { mode: mode.to_string(), enabled: v.get("enabled").and_then(Value::as_bool).unwrap_or(false) }))
        })
    }
    fn set_vision_mode(&self, m: &VisionMode) -> BoxFuture<'_, Result<(), String>> {
        let body = serde_json::json!({"mode": m.mode, "enabled": m.enabled}).to_string();
        Box::pin(async move {
            crate::api::proxy::request_json(&self.upstream, "POST", "/api/mode", Some(body.as_bytes()), Duration::from_secs(2)).await.map(|_| ())
        })
    }
    fn snapshot_jpeg(&self) -> BoxFuture<'_, Result<Vec<u8>, String>> {
        Box::pin(crate::api::proxy::fetch_snapshot(&self.upstream, Duration::from_secs(3)))
    }
    fn next_frame(&self, after: Option<u64>, wait_ms: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        Box::pin(async move {
            let wait_ms = wait_ms.min(5000);
            let path = format!("/api/results/latest?after={}&wait_ms={wait_ms}", after.unwrap_or(0));
            let req_started = Instant::now();
            let v = crate::api::proxy::request_json(&self.upstream, "GET", &path, None, Duration::from_millis(wait_ms + 2000)).await?;
            Ok(self.frame(&v, after, req_started, Instant::now(), wait_ms))
        })
    }
}

/// ArUco markers of a trigger reply (`markers`: `[{id, corners: [[x, y] x 4]}]`, pixels of the
/// frame, the format calib_hover reads from rebot-vision's `aruco` pipeline). Entries without an
/// integer id or four finite corners are skipped.
pub fn parse_markers(raw: &Value) -> Vec<crate::mview_stereo::Marker> {
    raw.as_array().map(|a| a.iter().filter_map(|m| {
        let id = m.get("id").and_then(Value::as_i64)?;
        let c = m.get("corners").and_then(Value::as_array).filter(|c| c.len() == 4)?;
        let mut corners = [[0.0; 2]; 4];
        for (k, p) in c.iter().enumerate() {
            let p = p.as_array()?;
            corners[k] = [p.first()?.as_f64().filter(|x| x.is_finite())?, p.get(1)?.as_f64().filter(|x| x.is_finite())?];
        }
        Some(crate::mview_stereo::Marker { id, corners })
    }).collect()).unwrap_or_default()
}

fn f(v: &Value, keys: &[&str]) -> Option<f64> {
    keys.iter().find_map(|k| v.get(*k).and_then(Value::as_f64)).filter(|x| x.is_finite())
}

/// Detector boxes → normalised boxes. Accepts `[cx, cy, w, h, score, label]` arrays and
/// objects with `box_norm`, or `format` `xyxy` / `xywh_center` / `xywh_top_left` (as the
/// cockpit's `parseDetections`), or centre `x`, `y`, `w`, `h` without `format` (rebot-vision
/// 1.2). Values above 1.05 are pixels of `size`. Objects may carry `angle_deg` (long side,
/// wrapped to [-90, 90)) and `angle_conf` (clamped to [0, 1]).
pub fn parse_detections(raw: &Value, size: [f64; 2]) -> Vec<Detection> {
    let Some(items) = raw.as_array() else { return Vec::new(); };
    let mut out = Vec::new();
    for item in items {
        let (label, raw_score_opt, cx, cy, w, h) = if let Some(a) = item.as_array().filter(|a| a.len() >= 5) {
            let n = |i: usize| a.get(i).and_then(Value::as_f64).filter(|x| x.is_finite());
            let (Some(cx), Some(cy), Some(w), Some(h), Some(s)) = (n(0), n(1), n(2), n(3), n(4)) else { continue };
            let label = a.get(5).map(|l| l.as_str().map(str::to_string).unwrap_or_else(|| l.to_string())).unwrap_or_else(|| "obj".into());
            (label, Some(s), cx, cy, w, h)
        } else if item.is_object() {
            let label = ["label", "name", "target", "class_name"].iter()
                .find_map(|k| item.get(*k).and_then(Value::as_str)).unwrap_or("obj").to_string();
            let raw_score_opt = f(item, &["score", "confidence"]);
            if let Some(b) = item.get("box_norm").and_then(Value::as_array).filter(|b| b.len() == 4) {
                let b: Vec<f64> = b.iter().filter_map(Value::as_f64).collect();
                if b.len() != 4 { continue; }
                (label, raw_score_opt, (b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0, b[2] - b[0], b[3] - b[1])
            } else {
                let arr = |k: &str| item.get(k).and_then(Value::as_array).filter(|b| b.len() >= 4)
                    .map(|b| b.iter().take(4).filter_map(Value::as_f64).collect::<Vec<_>>()).filter(|b| b.len() == 4);
                let bb = arr("bbox").or_else(|| arr("box"));
                match item.get("format").and_then(Value::as_str) {
                    Some("xyxy") => {
                        let (x0, y0, x1, y1) = match (&bb, f(item, &["x0", "x1"]), f(item, &["y0", "y1"])) {
                            (Some(b), _, _) => (b[0], b[1], b[2], b[3]),
                            _ => match (f(item, &["x1"]), f(item, &["x2"]), f(item, &["y1"]), f(item, &["y2"])) {
                                (Some(x1), Some(x2), Some(y1), Some(y2)) => (x1, y1, x2, y2),
                                _ => match (f(item, &["x0"]), f(item, &["x1"]), f(item, &["y0"]), f(item, &["y1"])) {
                                    (Some(a), Some(b), Some(c), Some(d)) => (a, c, b, d),
                                    _ => continue,
                                },
                            },
                        };
                        (label, raw_score_opt, (x0 + x1) / 2.0, (y0 + y1) / 2.0, x1 - x0, y1 - y0)
                    }
                    Some("xywh_center") => {
                        let (Some(cx), Some(cy), Some(w), Some(h)) = (f(item, &["cx", "x"]), f(item, &["cy", "y"]), f(item, &["w", "width"]), f(item, &["h", "height"])) else { continue };
                        (label, raw_score_opt, cx, cy, w, h)
                    }
                    Some("xywh_top_left") => {
                        let (x0, y0, w, h) = match &bb {
                            Some(b) => (b[0], b[1], b[2], b[3]),
                            None => match (f(item, &["x", "x0"]), f(item, &["y", "y0"]), f(item, &["w", "width"]), f(item, &["h", "height"])) {
                                (Some(a), Some(b), Some(c), Some(d)) => (a, b, c, d),
                                _ => continue,
                            },
                        };
                        (label, raw_score_opt, x0 + w / 2.0, y0 + h / 2.0, w, h)
                    }
                    // rebot-vision 1.2 (perception API 1) sent centre x, y, w, h without `format`
                    None if bb.is_none() => {
                        let (Some(cx), Some(cy), Some(w), Some(h)) = (f(item, &["cx", "x"]), f(item, &["cy", "y"]), f(item, &["w"]), f(item, &["h"])) else { continue };
                        (label, raw_score_opt, cx, cy, w, h)
                    }
                    _ => continue,
                }
            }
        } else { continue };
        let score = raw_score_opt.unwrap_or(0.0);
        if !(w > 0.0 && h > 0.0 && score.is_finite()) { continue; }
        let pixel = cx > 1.05 || cy > 1.05 || w > 1.05 || h > 1.05;
        let (sx, sy) = if pixel { (size[0], size[1]) } else { (1.0, 1.0) };
        let b = [(cx - w / 2.0) / sx, (cy - h / 2.0) / sy, (cx + w / 2.0) / sx, (cy + h / 2.0) / sy].map(|v| v.clamp(0.0, 1.0));
        if b[2] <= b[0] || b[3] <= b[1] { continue; }
        let angle_deg = item.is_object().then(|| f(item, &["angle_deg"])).flatten().map(|a| (a + 90.0).rem_euclid(180.0) - 90.0);
        let angle_conf = item.is_object().then(|| f(item, &["angle_conf"])).flatten().map(|c| c.clamp(0.0, 1.0));
        let grasp_width_px = item.is_object().then(|| f(item, &["grasp_width_px", "width_px"])).flatten().filter(|w| *w > 0.0);
        let mask_focus = item.is_object().then(|| item.get("mask_focus").and_then(Value::as_bool)).flatten().unwrap_or(false);
        let disp_fg = item.is_object().then(|| f(item, &["disp_fg"])).flatten();
        let disp_bg = item.is_object().then(|| f(item, &["disp_bg"])).flatten();
        let depth_conf = item.is_object().then(|| f(item, &["depth_conf"])).flatten().map(|c| c.clamp(0.0, 1.0));
        let length_px = item.is_object().then(|| f(item, &["length_px"])).flatten().filter(|l| *l > 0.0);
        let jaw_clear = item.is_object().then(|| item.get("jaw_clear").and_then(Value::as_bool)).flatten();
        let depth_angle_deg = item.is_object().then(|| f(item, &["depth_angle_deg"])).flatten().map(|a| (a + 90.0).rem_euclid(180.0) - 90.0);
        let depth_angle_conf = item.is_object().then(|| f(item, &["depth_angle_conf"])).flatten().map(|c| c.clamp(0.0, 1.0));
        let depth_width_px = item.is_object().then(|| f(item, &["depth_width_px"])).flatten().filter(|w| *w > 0.0);
        let raw_score_valid = raw_score_opt.is_some_and(|s| s.is_finite() && (0.0..=1.0).contains(&s));
        let contour_norm = item.get("contour").filter(|c| !c.is_null()).map(|c| parse_contour(c, pixel, size));
        out.push(Detection { label, score: score.clamp(0.0, 1.0), raw_score_valid, box_norm: b, angle_deg, angle_conf, grasp_width_px, mask_focus, disp_fg, disp_bg, depth_conf, length_px, jaw_clear,
            depth_angle_deg, depth_angle_conf, depth_width_px, contour_norm });
    }
    out
}

pub fn box_changed_px(a: &[f64; 4], b: &[f64; 4], size: [f64; 2]) -> bool {
    let dx0 = (a[0] - b[0]).abs() * size[0];
    let dy0 = (a[1] - b[1]).abs() * size[1];
    let dx1 = (a[2] - b[2]).abs() * size[0];
    let dy1 = (a[3] - b[3]).abs() * size[1];
    dx0 > 5.0 || dy0 > 5.0 || dx1 > 5.0 || dy1 > 5.0
}

pub fn box_centre(b: &[f64; 4]) -> [f64; 2] { [(b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0] }

pub fn iou(a: &[f64; 4], b: &[f64; 4]) -> f64 {
    let w = (a[2].min(b[2]) - a[0].max(b[0])).max(0.0);
    let h = (a[3].min(b[3]) - a[1].max(b[1])).max(0.0);
    let inter = w * h;
    let union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter;
    if union > 0.0 { inter / union } else { 0.0 }
}

fn median(v: &mut [f64]) -> f64 {
    v.sort_by(|a, b| a.total_cmp(b));
    let n = v.len();
    if n % 2 == 1 { v[n / 2] } else { (v[n / 2 - 1] + v[n / 2]) / 2.0 }
}

/// Follows the picked target across frames: same label, and centre within the gate of the
/// predicted centre or IoU above `match_iou` with the predicted box. While the predicted box
/// touches the image border the gate is at least `acquire_dist_px`: the centre of a cut box
/// moves by up to half the hidden part as the box comes into view.
#[derive(Debug, Clone)]
pub struct Tracker {
    pub label: String,
    /// Predicted box (normalised) of the target in the next frame.
    pub predicted: [f64; 4],
    pub gate_px: f64,
}

impl Tracker {
    pub fn find(&self, frame: &DetectionFrame, setup: &GraspSetup) -> Option<Detection> {
        let [w, h] = setup.image_size_px;
        let pc = box_centre(&self.predicted);
        let gate = if touches_border(&self.predicted, [w, h]) { self.gate_px.max(setup.acquire_dist_px) } else { self.gate_px };
        frame.detections.iter()
            .filter(|d| d.label.eq_ignore_ascii_case(&self.label))
            .filter_map(|d| {
                let c = box_centre(&d.box_norm);
                let dist = ((c[0] - pc[0]) * w).hypot((c[1] - pc[1]) * h);
                (dist <= gate || iou(&d.box_norm, &self.predicted) > setup.match_iou).then_some((dist, d))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, d)| d.clone())
    }

    /// Re-centre the predicted box on `centre_norm`, keeping its size.
    pub fn predict_at(&mut self, centre_norm: [f64; 2]) {
        let c = box_centre(&self.predicted);
        let (dx, dy) = (centre_norm[0] - c[0], centre_norm[1] - c[1]);
        self.predicted = [self.predicted[0] + dx, self.predicted[1] + dy, self.predicted[2] + dx, self.predicted[3] + dy];
    }
}

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

// ---- depth calibration & measurement ----

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DepthCalibration {
    pub a: f64,
    pub b: f64,
    pub source: &'static str,
}

/// Start-frame depth kept for the overhead measurement (`two_height_fk` needs both frames).
#[derive(Default)]
struct DepthCache {
    prior: Option<TableObs>,
    meas: Option<DepthMeasurement>,
    det0: Option<Detection>,
    /// Why `meas` is None.
    why: Option<&'static str>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct TableObs {
    pub inv_z_fk: f64,
    pub disp_table: f64,
    pub samples: Vec<(f64, f64, f64)>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct DepthMeasurement {
    pub height_m: f64,
    pub depth_m: f64,
    pub width_m: Option<f64>,
    pub length_m: Option<f64>,
    pub source: &'static str,
    pub jaw_clear: Option<bool>,
}

/// Table samples of a depth frame that calibrate the disparity: `(1 / z_fk, disp, z_fk)` per
/// sample with a positive finite disparity whose ray meets the table (`table_z_m`) at an
/// optical depth z_fk in [0.08, 2.5] m (FK pose + camera model).
pub fn table_fk_samples(cam: &CamModel, pose: &Pose, depth: &DepthFrameInfo, table_z_m: f64) -> Vec<(f64, f64, f64)> {
    depth.table_samples.iter().filter_map(|&[u, v, disp]| {
        if disp <= 1e-3 || !disp.is_finite() { return None; }
        cam.table_optical_depth(pose, [u, v], table_z_m).map(|z_fk| (1.0 / z_fk, disp, z_fk))
    }).collect()
}

pub fn extract_table_obs(
    cam: &CamModel,
    pose: &Pose,
    depth: &DepthFrameInfo,
    table_z_m: f64,
) -> Option<TableObs> {
    let mut samples: Vec<(f64, f64, f64)> = Vec::new();
    let mut inv_zs = Vec::new();
    let mut disps = Vec::new();
    for sample in &depth.table_samples {
        let [u, v, disp] = *sample;
        if disp > 1e-3 && disp.is_finite() {
            if let Some(z_fk) = cam.table_optical_depth(pose, [u, v], table_z_m) {
                samples.push((1.0 / z_fk, disp, z_fk));
                inv_zs.push(1.0 / z_fk);
                disps.push(disp);
            }
        }
    }
    if samples.len() < 6 { return None; }
    Some(TableObs {
        inv_z_fk: median(&mut inv_zs),
        disp_table: median(&mut disps),
        samples,
    })
}

fn fit_oblique_ransac(samples: &[(f64, f64, f64)]) -> Option<DepthCalibration> {
    let n = samples.len();
    if n < 8 { return None; }

    let mut best_inliers: Vec<usize> = Vec::new();
    let mut best_pair_a = 0.0;
    let mut best_pair_b = 0.0;
    let mut best_err_sum = f64::INFINITY;

    for i in 0..n {
        for j in (i + 1)..n {
            let (y_i, x_i, _) = samples[i];
            let (y_j, x_j, _) = samples[j];
            let dx = x_j - x_i;
            if dx.abs() < 1e-4 { continue; }
            let a = (y_j - y_i) / dx;
            let b = y_i - a * x_i;
            if a <= 1e-4 || !b.is_finite() { continue; }

            let mut inliers = Vec::new();
            let mut err_sum = 0.0;
            for (k, &(_, x_k, z_fk_k)) in samples.iter().enumerate() {
                let pred_inv_z = a * x_k + b;
                if pred_inv_z <= 1e-4 { continue; }
                let z_pred = 1.0 / pred_inv_z;
                let rel_err = (z_pred - z_fk_k).abs() / z_fk_k;
                if rel_err <= 0.08 {
                    inliers.push(k);
                    err_sum += rel_err;
                }
            }

            if inliers.len() > best_inliers.len()
                || (inliers.len() == best_inliers.len() && err_sum < best_err_sum)
            {
                best_inliers = inliers;
                best_pair_a = a;
                best_pair_b = b;
                best_err_sum = err_sum;
            }
        }
    }

    let min_inliers_req = 6.max((0.6 * n as f64).ceil() as usize);
    if best_inliers.len() < min_inliers_req {
        return None;
    }

    let mut a = best_pair_a;
    let mut b = best_pair_b;

    for _ in 0..4 {
        let mut residuals: Vec<f64> = samples.iter().map(|&(y, x, _)| (y - (a * x + b)).abs()).collect();
        let mad = median(&mut residuals).max(1e-4);
        let delta = 1.345 * mad;

        let mut sum_w = 0.0;
        let mut sum_wx = 0.0;
        let mut sum_wy = 0.0;
        let mut sum_wxx = 0.0;
        let mut sum_wxy = 0.0;

        for &(y, x, _) in samples {
            let r = (y - (a * x + b)).abs();
            let w = if r <= delta { 1.0 } else { delta / r };
            sum_w += w;
            sum_wx += w * x;
            sum_wy += w * y;
            sum_wxx += w * x * x;
            sum_wxy += w * x * y;
        }

        let denom = sum_w * sum_wxx - sum_wx * sum_wx;
        if denom.abs() < 1e-12 { break; }
        let new_a = (sum_w * sum_wxy - sum_wx * sum_wy) / denom;
        let new_b = (sum_wy - new_a * sum_wx) / sum_w;
        if new_a <= 1e-4 || !new_b.is_finite() { break; }
        a = new_a;
        b = new_b;
    }

    if a <= 1e-4 || !b.is_finite() {
        return None;
    }

    let mut final_inliers = 0;
    let mut sum_sq_rel_err = 0.0;
    for &(_, x_k, z_fk_k) in samples {
        let pred_inv_z = a * x_k + b;
        if pred_inv_z <= 1e-4 { continue; }
        let z_pred = 1.0 / pred_inv_z;
        let rel_err = (z_pred - z_fk_k).abs() / z_fk_k;
        if rel_err <= 0.08 {
            final_inliers += 1;
            sum_sq_rel_err += rel_err * rel_err;
        }
    }

    if final_inliers < min_inliers_req {
        return None;
    }

    let rel_rmse = (sum_sq_rel_err / final_inliers as f64).sqrt();
    if rel_rmse <= 0.06 {
        Some(DepthCalibration { a, b, source: "oblique_ransac" })
    } else {
        None
    }
}

pub fn calibrate_depth(
    cam: &CamModel,
    pose: &Pose,
    depth: Option<&DepthFrameInfo>,
    prior: Option<&TableObs>,
    table_z_m: f64,
) -> Option<DepthCalibration> {
    calibrate_depth_why(cam, pose, depth, prior, table_z_m).ok()
}

/// [`calibrate_depth`] with the reason it failed: `no_depth_frame` (the frame has no depth),
/// `too_few_samples` (metric: < 3 table samples; relative: < 8 and no two-height pair),
/// `table_contradicts_fk` (the table samples do not fit the FK table depths),
/// `no_oblique_spread` (relative: the table's inverse depth spans < 0.35 1/m),
/// `no_two_height_spread` / `two_height_fit` (relative with a start frame: the two views are
/// too close, or the combined fit fails).
pub fn calibrate_depth_why(
    cam: &CamModel,
    pose: &Pose,
    depth: Option<&DepthFrameInfo>,
    prior: Option<&TableObs>,
    table_z_m: f64,
) -> Result<DepthCalibration, &'static str> {
    let depth = depth.ok_or("no_depth_frame")?;

    let valid_samples = table_fk_samples(cam, pose, depth, table_z_m);

    if depth.metric_model {
        let a_prior = (600.0 / 640.0) / (cam.fx / cam.size[0].max(1.0));
        if valid_samples.len() >= 3 {
            let mut ratios: Vec<f64> = valid_samples.iter().map(|&(inv_z, disp, _)| inv_z / disp).collect();
            let a = median(&mut ratios);
            let b = 0.0;
            if (0.25..=4.0).contains(&a) {
                // Ensure samples agree with both the metric prior and the fitted table scale a.
                let mut prior_errs: Vec<f64> = valid_samples.iter().map(|&(_, disp, z_fk)| {
                    let z_pred = 1.0 / (a_prior * disp);
                    (z_pred - z_fk).abs() / z_fk
                }).collect();
                let mut fit_errs: Vec<f64> = valid_samples.iter().map(|&(_, disp, z_fk)| {
                    let z_pred = 1.0 / (a * disp);
                    (z_pred - z_fk).abs() / z_fk
                }).collect();
                if median(&mut prior_errs) <= 0.12 && median(&mut fit_errs) <= 0.12 {
                    return Ok(DepthCalibration { a, b, source: "metric_table" });
                }
            }
            return Err("table_contradicts_fk");
        }
        // Fewer than 3 table samples (table occluded): the nominal scale alone is not checked
        // against FK, so it must not replace the mechanical probe.
        return Err("too_few_samples");
    }

    let mut why = "too_few_samples";
    if valid_samples.len() >= 8 {
        let mut min_inv_z = f64::INFINITY;
        let mut max_inv_z = f64::NEG_INFINITY;
        for &(inv_z, _, _) in &valid_samples {
            if inv_z < min_inv_z { min_inv_z = inv_z; }
            if inv_z > max_inv_z { max_inv_z = inv_z; }
        }
        if max_inv_z - min_inv_z >= 0.35 {
            if let Some(cal) = fit_oblique_ransac(&valid_samples) {
                return Ok(cal);
            }
            why = "table_contradicts_fk";
        } else {
            why = "no_oblique_spread";
        }
    }

    if let Some(prior_obs) = prior {
        if !depth.metric_model && prior_obs.samples.len() >= 6 && valid_samples.len() >= 6 {
            let mut inv_zs: Vec<f64> = valid_samples.iter().map(|s| s.0).collect();
            let mut disps: Vec<f64> = valid_samples.iter().map(|s| s.1).collect();
            let inv_z_2 = median(&mut inv_zs);
            let d_2 = median(&mut disps);
            let d_inv_z = inv_z_2 - prior_obs.inv_z_fk;
            let d_disp = d_2 - prior_obs.disp_table;

            why = "no_two_height_spread";
            if d_inv_z.abs() >= 0.35 && d_disp.abs() >= 0.05 && (d_inv_z * d_disp > 0.0) {
                why = "two_height_fit";
                let mut combined = prior_obs.samples.clone();
                combined.extend_from_slice(&valid_samples);
                if let Some(cal) = fit_oblique_ransac(&combined) {
                    if (0.05..=20.0).contains(&cal.a) {
                        let check_inliers = |samples: &[(f64, f64, f64)]| -> bool {
                            let inliers = samples.iter().filter(|&&(_, x, z_fk)| {
                                let pred_inv_z = cal.a * x + cal.b;
                                if pred_inv_z <= 1e-4 { return false; }
                                let z_pred = 1.0 / pred_inv_z;
                                (z_pred - z_fk).abs() / z_fk <= 0.08
                            }).count();
                            inliers as f64 >= 0.60 * samples.len() as f64
                        };
                        if check_inliers(&prior_obs.samples) && check_inliers(&valid_samples) {
                            return Ok(DepthCalibration { a: cal.a, b: cal.b, source: "two_height_fk" });
                        }
                    }
                }
            }
        }
    }

    Err(why)
}

fn metric_table_contradicts_fk(cam: &CamModel, pose: &Pose, depth: Option<&DepthFrameInfo>, table_z_m: f64) -> bool {
    let Some(d) = depth else { return false; };
    if !d.metric_model { return false; }
    let valid_count = d.table_samples.iter().filter(|s| {
        let [u, v, disp] = **s;
        disp > 1e-3 && disp.is_finite() && cam.table_optical_depth(pose, [u, v], table_z_m).is_some()
    }).count();
    valid_count >= 3 && calibrate_depth(cam, pose, Some(d), None, table_z_m).is_none()
}

pub fn measure_detection_depth_with_cal(
    cam: &CamModel,
    pose: &Pose,
    det: &Detection,
    cal: &DepthCalibration,
) -> Option<DepthMeasurement> {
    measure_detection_depth_with_cal_why(cam, pose, det, cal).ok()
}

/// [`measure_detection_depth_with_cal`] with the reason it failed: `low_conf` (`depth_conf`
/// < 0.35), `no_disparity`, `bad_disparity` (foreground not nearer than the table, or depths
/// out of range), `too_flat` (< 2 mm above the table).
// `!(a > b)` also rejects NaN
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn measure_detection_depth_with_cal_why(
    cam: &CamModel,
    pose: &Pose,
    det: &Detection,
    cal: &DepthCalibration,
) -> Result<DepthMeasurement, &'static str> {
    if det.depth_conf.unwrap_or(0.0) < 0.35 { return Err("low_conf"); }
    let (Some(disp_fg), Some(disp_bg)) = (det.disp_fg, det.disp_bg) else { return Err("no_disparity"); };
    if !(disp_fg > disp_bg) { return Err("bad_disparity"); }
    let denom_fg = cal.a * disp_fg + cal.b;
    let denom_bg = cal.a * disp_bg + cal.b;
    if !(denom_fg > 0.1 && denom_bg > 0.1) { return Err("bad_disparity"); }
    let z_fg = 1.0 / denom_fg;
    let z_bg = 1.0 / denom_bg;
    if !( (0.06..=2.0).contains(&z_fg) && z_bg > z_fg ) { return Err("bad_disparity"); }

    let centre_px = cam.to_px(box_centre(&det.box_norm));
    let ray = cam.ray_world_per_z(pose, centre_px);
    let h_raw = (z_bg - z_fg) * ray[2].abs();
    if !(h_raw > 0.002) { return Err("too_flat"); }
    let h = h_raw.max(0.005);

    let depth_m = z_fg;
    // the detector's width and angle (a segmentation mask) when it reports them, else the
    // depth pipeline's own (a plain detection model): the same inputs as before rebot-vision
    // reported the depth values separately
    let (width_px, angle_deg) = if det.angle_deg.is_some() || det.grasp_width_px.is_some() {
        (det.grasp_width_px, det.angle_deg)
    } else {
        (det.depth_width_px, det.depth_angle_deg)
    };
    let (sin_t, cos_t) = angle_deg.map(|deg| deg.to_radians().sin_cos()).unwrap_or((0.0, 0.0));
    let f_geom = (cam.fx * cam.fy).sqrt();
    let width_m = width_px.filter(|w| *w > 0.0).map(|w| {
        if angle_deg.is_some() {
            w * z_fg * (-sin_t / cam.fx).hypot(cos_t / cam.fy)
        } else {
            w * z_fg / f_geom
        }
    });
    let length_m = det.length_px.filter(|l| *l > 0.0).map(|l| {
        if angle_deg.is_some() {
            l * z_fg * (cos_t / cam.fx).hypot(sin_t / cam.fy)
        } else {
            l * z_fg / f_geom
        }
    });
    Ok(DepthMeasurement {
        height_m: h,
        depth_m,
        width_m,
        length_m,
        source: cal.source,
        jaw_clear: det.jaw_clear,
    })
}

pub fn measure_detection_depth(
    cam: &CamModel,
    pose: &Pose,
    det: &Detection,
    depth: Option<&DepthFrameInfo>,
    prior: Option<&TableObs>,
    table_z_m: f64,
) -> Option<DepthMeasurement> {
    let cal = calibrate_depth(cam, pose, depth, prior, table_z_m)?;
    measure_detection_depth_with_cal(cam, pose, det, &cal)
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

// ---- request ----

#[derive(Debug, Clone, PartialEq)]
pub struct GraspRequest {
    pub label: String,
    pub box_norm: [f64; 4],
    pub frame_id: Option<u64>,
    /// Effective setup: base, then the task's overrides, then the request's (no `tasks`).
    pub setup: GraspSetup,
    pub tag: Option<String>,
    /// Name of the task in `GraspSetup.tasks`, when the request named one.
    pub task: Option<String>,
    /// The task's perception requirements (empty without a task).
    pub requires: TaskRequires,
    /// The task's model switch (None without a task or when it names no models).
    pub models: Option<TaskModels>,
    /// Request field `approach` for the final descent (src/backlash.rs, `grasp_final`).
    pub approach: crate::backlash::Request,
}

/// Body of `/api/v1/action/grasp`: `{label, box_norm, frame_id?, ts_ms?, task?, overrides?, tag?}`.
/// The setup is `base`, then the overrides of `task`, then `overrides`.
pub fn parse_grasp(body: &Value, base: &GraspSetup) -> Result<GraspRequest, MotionError> {
    let bad = |m: &str| MotionError::InvalidCommand(m.to_string());
    let label = body.get("label").and_then(Value::as_str).filter(|l| !l.is_empty() && l.len() <= 64)
        .ok_or_else(|| bad("label must be a non-empty string of at most 64 bytes"))?.to_string();
    let b = body.get("box_norm").and_then(Value::as_array).filter(|b| b.len() == 4)
        .ok_or_else(|| bad("box_norm must be [x0, y0, x1, y1] in [0, 1]"))?;
    let mut box_norm = [0.0; 4];
    for (o, v) in box_norm.iter_mut().zip(b) {
        *o = v.as_f64().filter(|x| (0.0..=1.0).contains(x)).ok_or_else(|| bad("box_norm must be [x0, y0, x1, y1] in [0, 1]"))?;
    }
    if box_norm[2] <= box_norm[0] || box_norm[3] <= box_norm[1] { return Err(bad("box_norm needs x0 < x1 and y0 < y1")); }
    let frame_id = match body.get("frame_id") {
        None | Some(Value::Null) => None,
        Some(v) => Some(v.as_u64().ok_or_else(|| bad("frame_id must be a non-negative integer"))?),
    };
    if body.get("ts_ms").is_some_and(|v| !v.is_null() && !v.is_number()) { return Err(bad("ts_ms must be a number")); }
    let task = match body.get("task") {
        None | Some(Value::Null) => None,
        Some(Value::String(s)) => Some(s.clone()),
        _ => return Err(bad("task must be a string")),
    };
    let mut setup = base.without_tasks();
    let mut requires = TaskRequires::default();
    let mut models = None;
    if let Some(name) = &task {
        let t = base.tasks.get(name).ok_or_else(|| bad(&format!("unknown grasp task: {name}")))?;
        setup = setup.patched_request(&Value::Object(t.overrides.clone()))
            .map_err(|e| MotionError::InvalidCommand(format!("grasp task {name}: {e}")))?;
        requires = t.requires;
        models = t.models.clone();
    }
    match body.get("overrides") {
        None | Some(Value::Null) => {}
        Some(o) => setup = setup.patched_request(o).map_err(MotionError::InvalidCommand)?,
    }
    let tag = match body.get("tag") {
        None | Some(Value::Null) => None,
        Some(Value::String(s)) if s.len() <= 64 => Some(s.clone()),
        _ => return Err(bad("tag must be a string of at most 64 bytes")),
    };
    let approach = crate::backlash::parse_request(body)?;
    Ok(GraspRequest { label, box_norm, frame_id, setup, tag, task, requires, models, approach })
}

/// A request checked under the lock, ready to be planned outside it.
pub struct PendingGrasp {
    pub input: PlanInput,
    /// Boxed: the setup makes the request large.
    pub request: Box<GraspRequest>,
    pub open_rad: f32,
    pub physical_contact_end_m: Option<[f64; 3]>,
}

/// A grasp whose coarse approach was found reachable.
pub struct PlannedGrasp {
    pub input: PlanInput,
    pub request: GraspRequest,
    /// Estimated target on the grasp plane (base frame).
    pub target: Vec3,
    pub approach_duration: f64,
    /// Camera-down acquisition ([`camera_down_mode`]).
    pub camera_down: bool,
    /// Estimated duration of the camera-down → tool-vertical move (0 without camera-down).
    pub transition_duration: f64,
    /// The provider's mode read by [`preflight`] (a task with requirements); the orchestrator
    /// reads it itself when None.
    pub mode: Option<Result<ModeInfo, String>>,
}

/// Under the lock: parse and check preconditions. `strategy: "adaptive"` is refused: it is a
/// dry-run strategy ([`crate::grasp_select`]).
pub fn prepare(inner: &SupervisorInner, body: &Value, epoch: u64) -> Result<PendingGrasp, ArmError> {
    if body.is_object() { crate::grasp_select::refuse_execution(body)?; }
    prepare_any(inner, body, epoch)
}

/// [`prepare`] for any strategy (the dry run).
fn prepare_any(inner: &SupervisorInner, body: &Value, epoch: u64) -> Result<PendingGrasp, ArmError> {
    if !body.is_object() { return Err(MotionError::InvalidCommand("Command body must be an object".into()).into()); }
    inner.check_commissioning_for_ordinary_motion("grasp")?;
    let Some(base) = inner.grasp_setup() else {
        return Err(MotionError::Unsupported("Grasp needs a profile with kinematics, a gripper policy and a grasp setup".into()).into());
    };
    if inner.camera_calibrated() { refuse_calibrated_request(&base, body)?; }
    let Some(policy) = inner.config.gripper_policy.clone() else {
        return Err(MotionError::Unsupported("Grasp needs a gripper policy".into()).into());
    };
    let request = parse_grasp(body, &base)?;
    if epoch != inner.safety.epoch_id() {
        return Err(crate::safety::SafetyError::EpochMismatch { expected: inner.safety.epoch_id(), received: epoch }.into());
    }
    let input = inner.plan_input()?;
    Ok(PendingGrasp {
        input,
        request: Box::new(request),
        open_rad: policy.open_rad,
        physical_contact_end_m: inner.config.effective_physical_contact_end_m(),
    })
}

/// TCP height of the grasp plane (where box centres are assumed to lie).
fn grasp_plane(setup: &GraspSetup) -> f64 { setup.table_z_m + setup.grasp_height_m }

/// Look-down approach pose above `target`.
fn approach_pose(r_now: &Mat3, target: &Vec3, setup: &GraspSetup) -> Pose {
    Pose::new(look_down(r_now), [target[0], target[1], setup.table_z_m + setup.approach_height_m])
}

/// Outside the lock: estimate the target from the box-centre ray and plan the approach
/// with the gripper open (feasibility check, so an unreachable target is rejected up front).
pub fn plan(pending: PendingGrasp) -> Result<PlannedGrasp, ArmError> {
    let PendingGrasp { input, request, open_rad, physical_contact_end_m } = pending;
    let request = *request;
    let setup = &request.setup;
    let pose_end = input.arm.fk(&input.q0);
    let pose_tcp = motion::end_to_tcp(&pose_end, &input.tcp_offset);
    let cam = CamModel::assumed(&input.arm, setup, &input.cam);
    let centre = cam.to_px(box_centre(&request.box_norm));
    let target = cam.plane_point(&pose_end, centre, grasp_plane(setup)).ok_or_else(|| ArmError::from(MotionError::InvalidCommand(
        "the box centre ray does not meet the table plane in front of the camera".into())))?;
    let camera_down = camera_down_mode(setup, &input.cam).map_err(MotionError::InvalidCommand)?;
    let pts = envelope_with_contact(input.ws.as_ref(), &input.cam, &cam.rec, physical_contact_end_m);
    let is_5dof = input.arm.arm_dof() == 5;
    let r_ref_p = if camera_down { &pose_end.p } else { &pose_tcp.p };
    let r_ref = if is_5dof {
        motion::apply_auto_yaw(r_ref_p, &target, pose_end.r)
    } else {
        pose_end.r
    };
    // the envelope stays above the table on the approach and (camera-down) the transition
    let (goal, tool) = if camera_down {
        let goal = camera_down_pose(&r_ref, &target, setup, &cam, &pts);
        let tool = approach_pose(&goal.r, &target, setup);
        let tool_end = motion::tcp_to_end(&tool, &input.tcp_offset);
        if let Some(e) = table_violation(&goal, &tool_end, &pts, setup.table_z_m) { return Err(e.into()); }
        (goal, tool)
    } else {
        let goal = approach_pose(&r_ref, &target, setup);
        (goal, goal)
    };
    let goal_end = if camera_down { goal } else { motion::tcp_to_end(&goal, &input.tcp_offset) };
    if let Some(e) = table_violation(&pose_end, &goal_end, &pts, setup.table_z_m) { return Err(e.into()); }
    let mut cur = input.clone();
    cur.j7 = open_rad;
    cur.goal_frame = if camera_down { GoalFrame::EndLink } else { GoalFrame::Tcp };
    let path = motion::plan_cartesian_ex(&cur, goal, Speed { linear: setup.approach_speed_mps, angular: motion::DEFAULT_ANGULAR_RPS }, is_5dof)?;
    crate::backlash::check_table_clear(&input.arm, &pts, setup.table_z_m + TABLE_CLEARANCE_M, path.q.iter().map(|q| q.as_slice()))
        .map_err(|v| ArmError::from(v.into_motion_error()))?;
    let last = path.last();
    cur.q0 = std::array::from_fn(|i| f64::from(last[i]));
    cur.goal_frame = GoalFrame::Tcp;
    let transition_duration = if camera_down {
        let trans_path = motion::plan_cartesian_ex(&cur, tool, Speed { linear: setup.approach_speed_mps, angular: motion::DEFAULT_ANGULAR_RPS }, is_5dof)?;
        crate::backlash::check_table_clear(&input.arm, &pts, setup.table_z_m + TABLE_CLEARANCE_M, trans_path.q.iter().map(|q| q.as_slice()))
            .map_err(|v| ArmError::from(v.into_motion_error()))?;
        let last = trans_path.last();
        cur.q0 = std::array::from_fn(|i| f64::from(last[i]));
        transition_estimate(&goal, &target, setup)
    } else {
        0.0
    };
    let grasp_pose = Pose::new(tool.r, [target[0], target[1], setup.table_z_m + setup.grasp_height_m]);
    let _descend_path = motion::plan_cartesian_ex(&cur, grasp_pose, Speed { linear: setup.descend_speed_mps, angular: motion::DEFAULT_ANGULAR_RPS }, is_5dof)?;
    Ok(PlannedGrasp { input, request, target, approach_duration: path.duration, camera_down, transition_duration, mode: None })
}

/// Duration of the transition from the camera-down pose `from` to the tool-vertical pose
/// above `target`, from the planner's Cartesian speed bound (quintic peak 1.875 / T, +10 %);
/// joint speed limits can make it longer.
pub fn transition_estimate(from: &Pose, target: &Vec3, setup: &GraspSetup) -> f64 {
    let to = approach_pose(&from.r, target, setup);
    let length = kinematics::norm(&kinematics::sub(&to.p, &from.p));
    let theta = kinematics::angle_between(&to.r, &from.r);
    1.1 * 1.875 * (length / setup.approach_speed_mps).max(theta / motion::DEFAULT_ANGULAR_RPS)
}

/// Rough total duration for the ack.
pub fn expected_duration(planned: &PlannedGrasp) -> f64 {
    let s = &planned.request.setup;
    let descend = (s.approach_height_m - s.grasp_height_m) * 1.875 / s.descend_speed_mps;
    let lift = (s.lift_height_m - s.grasp_height_m) * 1.875 / s.descend_speed_mps;
    let mut probe = if s.height_probe_m > 0.0 { 2.0 * s.height_probe_m * 1.875 / s.approach_speed_mps + 1.5 } else { 0.0 };
    // lateral: B/2 + B + B/2 of motion and two frame sets, on top of a possible up fallback
    if s.height_probe_m > 0.0 && s.height_probe_mode == HeightProbeMode::Lateral { probe += 2.0 * 1.875 * s.lateral_probe_m / s.approach_speed_mps + 1.5; }
    let recentre = if s.recentre_height_m > 0.0 && !planned.camera_down { 3.0 } else { 0.0 };
    let lift_check = s.post_lift_settle_ms as f64 / 1000.0;
    let base = lift_check + 6.5 + planned.approach_duration + 4.0 * 1.5 + descend + 2.0 + lift + probe + recentre;
    // camera-down: the transition move and its re-check frame
    if planned.camera_down { base + planned.transition_duration + 1.5 } else { base }
}

/// Code of a grasp rejected because the provider cannot serve the task's requirements.
pub const TASK_UNSUPPORTED: &str = "TASK_UNSUPPORTED";

fn task_unsupported(message: String) -> ArmError {
    ArmError::Rejected { status: 422, code: TASK_UNSUPPORTED.to_string(), message }
}

/// One model-changing preflight at a time (armd talks to one provider; its swap is one at a time).
static MODEL_SWITCH: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());
/// How long a preflight waits for the provider's swap (`/api/mode` polled).
const MODEL_SWITCH_TIMEOUT: Duration = Duration::from_secs(5);

/// Task check of a planned grasp, outside the supervisor lock and before the action begins.
/// Without requirements and models nothing is read. Otherwise one mode reading, kept in
/// `planned.mode` for the orchestrator; with `models` and a different resident set the provider
/// switches models first (serialized, [`switch_models`]) and the post-switch reading is kept. A
/// provider that cannot be read is not a task failure (the grasp then fails as without a task).
pub async fn preflight(source: &dyn DetectionSource, planned: &mut PlannedGrasp) -> Result<(), ArmError> {
    let request = &planned.request;
    let Some(task) = request.task.as_deref().filter(|_| !request.requires.is_empty() || request.models.is_some()) else { return Ok(()); };
    let _serial = if request.models.is_some() { Some(MODEL_SWITCH.lock().await) } else { None };
    let mut mode = source.mode_info().await;
    if let (Some(models), Ok(m)) = (&request.models, &mode) {
        if let Some(body) = models.switch_body(m) {
            mode = switch_models(source, &body, task).await?;
        }
    }
    let verdict = match &mode {
        Ok(m) => request.requires.check(task, m),
        Err(_) => Ok(()),
    };
    planned.mode = Some(mode);
    verdict.map_err(task_unsupported)
}

/// `POST /api/models`, then `/api/mode` polled until the provider published the returned
/// generation and its swap is idle (or failed). Refusals are TASK_UNSUPPORTED; an unreachable
/// provider or a swap that does not finish in time is the returned `Err` reading.
async fn switch_models(source: &dyn DetectionSource, body: &Value, task: &str) -> Result<Result<ModeInfo, String>, ArmError> {
    let refused = |why: String| task_unsupported(format!("task {task}: {why}"));
    let (status, reply) = match source.set_models(body).await {
        Ok(r) => r,
        Err(e) => return Ok(Err(format!("model switch failed: {e}"))),
    };
    let error = reply.get("error").and_then(Value::as_str).unwrap_or("refused");
    match status {
        200..=299 => {}
        404 => return Err(refused("provider cannot switch models".into())),
        409 if error == "SWAP_BUSY" => return Err(refused("model switch busy".into())),
        400..=499 => return Err(refused(format!("model switch refused: {}", refusal_message(error, f(&reply, &["need_mb"]), f(&reply, &["limit_mb"]))))),
        _ => return Ok(Err(format!("model switch failed: detector returned {status}"))),
    }
    let Some(generation) = reply.get("generation").and_then(Value::as_u64) else {
        return Ok(Err("model switch reply has no generation".into()));
    };
    let deadline = Instant::now() + MODEL_SWITCH_TIMEOUT;
    loop {
        let m = match source.mode_info().await {
            Ok(m) => m,
            Err(e) => return Ok(Err(e)),
        };
        if let Some(pm) = m.models.as_ref().filter(|pm| pm.generation >= generation && matches!(pm.swap_state.as_str(), "idle" | "failed")) {
            if pm.swap_state == "failed" {
                return Err(refused(format!("model switch failed: {}", pm.swap_error.as_deref().unwrap_or("unknown error"))));
            }
            return Ok(Ok(m));
        }
        if Instant::now() >= deadline {
            return Ok(Err(format!("model switch did not finish within {} s", MODEL_SWITCH_TIMEOUT.as_secs())));
        }
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
}

/// The dry-run task verdict on one mode reading (no switch): a budget refusal computed from the
/// provider's catalogue, else `needs model switch` (supported) when the resident set differs,
/// else the requirement check on the current set.
fn task_verdict(task: &str, requires: &TaskRequires, models: Option<&TaskModels>, mode: &ModeInfo) -> (bool, Option<String>) {
    if let Some(want) = models.filter(|w| w.switch_body(mode).is_some()) {
        let budget = mode.models.as_ref().and_then(|pm| pm.budget(&want.after(&pm.resident)));
        return match budget {
            Some(Err(why)) => (false, Some(format!("task {task}: model switch refused: {why}"))),
            _ => (true, Some("needs model switch".into())),
        };
    }
    match requires.check(task, mode) { Ok(()) => (true, None), Err(r) => (false, Some(r)) }
}

/// Result of a grasp dry run.
#[derive(Debug, Clone, PartialEq)]
pub struct DryRun {
    pub reachable: bool,
    pub code: Option<String>,
    pub message: Option<String>,
    /// Requests with a task: whether the provider serves its requirements (true when it has
    /// none or the provider cannot be read) and why not (`needs model switch`, supported, when
    /// the task's models differ from the resident ones).
    pub task: Option<(bool, Option<String>)>,
    /// `strategy: "adaptive"`: the selection reply ([`crate::grasp_select::Selection::to_json`])
    /// plus `legacy` (the legacy verdict on the same request).
    pub adaptive: Option<Value>,
}

impl DryRun {
    /// The reply fields: `reachable`, `code`, `message`, and with a task `task_supported`,
    /// `task_reason`.
    pub fn fields(&self) -> serde_json::Map<String, Value> {
        let mut m = serde_json::Map::new();
        m.insert("reachable".into(), serde_json::json!(self.reachable));
        m.insert("code".into(), serde_json::json!(self.code));
        m.insert("message".into(), serde_json::json!(self.message));
        if let Some((supported, reason)) = &self.task {
            m.insert("task_supported".into(), serde_json::json!(supported));
            m.insert("task_reason".into(), serde_json::json!(reason));
        }
        if let Some(Value::Object(a)) = &self.adaptive {
            for (k, v) in a { m.insert(k.clone(), v.clone()); }
        }
        m
    }
}

/// Dry-run reachability check: prepare + plan only, no macro started, no orchestrator spawned.
/// Returns (reachable, code, message). The task requirements are not checked (see
/// [`dry_run_task`]).
pub async fn dry_run(supervisor: &Arc<ArmSupervisor>, body: &Value) -> (bool, Option<String>, Option<String>) {
    let (reachable, code, message, _, _) = dry_run_plan(supervisor, body).await;
    (reachable, code, message)
}

/// [`dry_run`] plus the task check ([`task_verdict`]): one `/api/mode` reading when the
/// request's task has requirements or models, none otherwise; never a model switch. The task
/// result does not change `reachable`.
pub async fn dry_run_task(supervisor: &Arc<ArmSupervisor>, source: &dyn DetectionSource, body: &Value) -> DryRun {
    let (reachable, code, message, request, adaptive) = dry_run_plan(supervisor, body).await;
    let task = match request {
        Some((Some(name), requires, models)) => Some(if requires.is_empty() && models.is_none() {
            (true, None)
        } else {
            match source.mode_info().await {
                Ok(m) => task_verdict(&name, &requires, models.as_ref(), &m),
                Err(_) => (true, None),
            }
        }),
        _ => None,
    };
    DryRun { reachable, code, message, task, adaptive }
}

/// Dry-run verdict: (reachable, code, message, (task, requires, models) when the request parsed,
/// the adaptive selection reply when `strategy` is `"adaptive"`).
///
/// Read-only planning/selection path (acquires only a read lock on `SupervisorInner` and runs
/// off-lock in `spawn_blocking`): never starts motion or actuator output, so no `Admitted` token
/// is required; real execution of `strategy: "adaptive"` is unconditionally refused in [`prepare`].
type DryRunPlan = (bool, Option<String>, Option<String>, Option<(Option<String>, TaskRequires, Option<TaskModels>)>, Option<Value>);

async fn dry_run_plan(supervisor: &Arc<ArmSupervisor>, body: &Value) -> DryRunPlan {
    let rejected = |e: ArmError| {
        let api = ApiError::from(e);
        (false, api.code.or(Some(api.error)), Some(api.message))
    };
    let strategy = match crate::grasp_select::parse_strategy(body) {
        Ok(s) => s,
        Err(e) => { let (r, c, m) = rejected(e.into()); return (r, c, m, None, None); }
    };
    let pending_res = {
        let inner = supervisor.inner().read().await;
        prepare_any(&inner, body, inner.safety.epoch_id()).and_then(|p| {
            let snap = match strategy {
                crate::grasp_select::Strategy::Adaptive => Some(crate::grasp_select::capture(&inner, &p, body)?),
                crate::grasp_select::Strategy::Legacy => None,
            };
            Ok((p, snap))
        })
    };
    match pending_res {
        Ok((pending, snap)) => {
            let task = Some((pending.request.task.clone(), pending.request.requires, pending.request.models.clone()));
            let joined = tokio::task::spawn_blocking(move || {
                let legacy = plan(pending).map(|_| ());
                let selection = snap.map(|s| { let sel = crate::grasp_select::select(&s); let v = sel.to_json(&s); (sel.reachable(), sel.code, sel.message, v) });
                (legacy, selection)
            }).await;
            match joined {
                Ok((legacy, None)) => {
                    let (reachable, code, message) = match legacy { Ok(()) => (true, None, None), Err(e) => rejected(e) };
                    (reachable, code, message, task, None)
                }
                Ok((legacy, Some((reachable, code, message, mut v)))) => {
                    let (lr, lc, lm) = match legacy { Ok(()) => (true, None, None), Err(e) => rejected(e) };
                    v["legacy"] = serde_json::json!({"reachable": lr, "code": lc, "message": lm});
                    (reachable, code, message, task, Some(v))
                }
                Err(e) => (false, Some("INTERNAL".into()), Some(format!("planner task failed: {e}")), task, None),
            }
        }
        Err(e) => {
            let (reachable, code, message) = rejected(e);
            (reachable, code, message, None, None)
        }
    }
}

/// Begin a planned grasp under the write lock: the action record (with its `task`) and the
/// orchestrator. Returns (action id, expected duration).
pub fn begin(admitted: crate::supervisor::Admitted<'_>, supervisor: &Arc<ArmSupervisor>, source: Arc<dyn DetectionSource>, planned: PlannedGrasp) -> Result<(u64, f64), ArmError> {
    let (inner, proof) = admitted.consume()?;
    let stamp = proof.stamp();
    let id = inner.begin_macro_inner(&proof, "grasp", &planned.input, planned.request.tag.clone())?;
    if let Some(task) = &planned.request.task {
        inner.update_action(id, |a| a.task = Some(task.clone()));
    }
    let duration = expected_duration(&planned) + inner.approach_estimate_s("grasp_final", planned.request.approach);
    spawn(supervisor.clone(), source, id, planned, stamp);
    Ok((id, duration))
}

/// Start a grasp (tests, in-process agents): caller passes the `AdmissionStamp` captured at
/// admission (never re-sampled internally). Prepares, plans, runs task preflight, validates
/// `stamp` under the owner lock, and spawns the orchestrator. Returns (action id, expected duration).
pub async fn start(supervisor: &Arc<ArmSupervisor>, stamp: crate::supervisor::AdmissionStamp, source: Arc<dyn DetectionSource>, body: &Value) -> Result<(u64, f64), ApiError> {
    let pending = {
        let inner = supervisor.inner().read().await;
        prepare(&inner, body, stamp.estop_epoch)?
    };
    let mut planned = tokio::task::spawn_blocking(move || plan(pending)).await
        .map_err(|e| ArmError::Internal(format!("planner task failed: {e}")))??;
    preflight(source.as_ref(), &mut planned).await?;
    let mut inner = supervisor.inner().write().await;
    let admitted = inner.check_admission(stamp).map_err(ArmError::from)?;
    Ok(begin(admitted, supervisor, source, planned)?)
}

/// Spawn the orchestrator for an action begun with `SupervisorInner::begin_macro(stamp, "grasp", ..)`.
pub fn spawn(supervisor: Arc<ArmSupervisor>, source: Arc<dyn DetectionSource>, id: u64, planned: PlannedGrasp, stamp: crate::supervisor::AdmissionStamp) {
    tokio::spawn(run(supervisor, source, id, planned, stamp));
}

// ---- orchestrator ----

const POLL: Duration = crate::macro_action::POLL;

/// Grasp phases on top of the shared macro-action primitives ([`MacroCtx`], via `Deref`).
pub struct Orchestrator {
    pub ctx: MacroCtx,
    pub source: Arc<dyn DetectionSource>,
    pub setup: GraspSetup,
    pub cam: CamModel,
    /// Points kept above the table on every sub-motion ([`envelope`]).
    pub envelope: Vec<Vec3>,
    /// Active tool TCP offset in `end_link` frame (m).
    pub tcp_offset: [f64; 3],
    /// Each centring step triggers its own frame (perception API 2) instead of reading the
    /// continuous results.
    pub triggered: bool,
    /// `camera_mount.offset_m` is set: the lens position (height probe) is known.
    pub lens_known: bool,
    /// Perception API depth pipeline is available.
    pub depth_available: bool,
    /// The provider's model generation at the start: a frame from another one fails the grasp.
    pub generation: Option<u64>,
}

/// A guarded grasp insertion ([`Orchestrator::insert`]): the TCP goal, the linear speed and
/// the same-side approach request of its final segment.
#[derive(Debug, Clone, Copy)]
struct Insertion {
    goal_tcp: Vec3,
    speed_mps: f64,
    approach: crate::backlash::Request,
}

impl std::ops::Deref for Orchestrator {
    type Target = MacroCtx;
    fn deref(&self) -> &MacroCtx { &self.ctx }
}

impl Orchestrator {
    /// Every grasp sub-motion in `EndLink` frame: WORKSPACE_VIOLATION before anything is
    /// installed when its straight path takes the envelope below table clearance.
    async fn move_arm(&self, phase: &str, spec: GoalSpec, linear: f64) -> Result<SubDone, Stop> {
        self.move_arm_in(GoalFrame::EndLink, phase, spec, linear).await
    }

    async fn move_arm_in(&self, frame: GoalFrame, phase: &str, spec: GoalSpec, linear: f64) -> Result<SubDone, Stop> {
        self.move_arm_final_in(frame, phase, spec, linear, None).await
    }

    /// [`move_arm_in`](Self::move_arm_in) with an optional same-side final approach.
    async fn move_arm_final_in(&self, frame: GoalFrame, phase: &str, spec: GoalSpec, linear: f64, approach: Option<crate::macro_action::FinalApproach>) -> Result<SubDone, Stop> {
        let input = {
            let inner = self.sup.inner().read().await;
            if !inner.macro_alive(self.id) { return Err(Stop::Cancelled); }
            inner.plan_input_in(frame).map_err(crate::macro_action::arm_fail)?
        };
        let spec = if input.arm.arm_dof() == 5 {
            match spec {
                GoalSpec::Absolute { p, r } => GoalSpec::AbsoluteEx { p, r, auto_yaw: true },
                other => other,
            }
        } else {
            spec
        };
        let (goal, _yaw_free) = motion::resolve_goal_ex(&input, &spec);
        let goal_end = motion::goal_to_end(&goal, &input);
        if let Some(e) = table_violation(&input.arm.fk(&input.q0), &goal_end, &self.envelope, self.setup.table_z_m) {
            if let MotionError::WorkspaceViolation { ref detail, .. } = e {
                self.sup.inner().write().await.set_action_workspace_violation(self.id, (**detail).clone());
            }
            return Err(crate::macro_action::arm_fail(e.into()));
        }
        self.ctx.move_arm_staged_in_with_table(
            frame,
            phase,
            spec,
            Speed { linear, angular: motion::DEFAULT_ANGULAR_RPS },
            approach,
            Some((self.envelope.clone(), self.setup.table_z_m + TABLE_CLEARANCE_M)),
        ).await.map_err(|(stop, _)| stop)
    }

    /// The frame ran with the model set the grasp started with (or either side does not report it).
    fn same_models(&self, frame: &DetectionFrame) -> bool {
        !matches!((self.generation, frame.model_generation), (Some(a), Some(b)) if a != b)
    }

    async fn set_mask_focus(&self, label: Option<String>, box_norm: [f64; 4], ttl_ms: u64) {
        if let Err(e) = self.source.set_mask_focus(label, box_norm, ttl_ms).await {
            tracing::debug!("Mask focus request failed (ignored): {e}");
        }
    }

    /// Next fresh frame (captured after `since` + settle delay) containing the target.
    /// TARGET_LOST when: frames keep arriving but all predate the settle time, `lost_timeout_s`
    /// after max(now, settle time) (stale frames do not extend it); no frame arrives for
    /// max(3 s, `lost_timeout_s`); fresh frames miss the target `lost_frames` times or for
    /// `lost_timeout_s` from the call.
    async fn fresh_match(&self, tracker: &Tracker, since: Instant, after: &mut Option<u64>) -> Result<Detection, Stop> {
        let not_before = since + Duration::from_secs_f64(self.setup.settle_delay_s);
        let started = Instant::now();
        let observe_until = started.max(not_before) + Duration::from_secs_f64(self.setup.lost_timeout_s);
        let mut last_frame = Instant::now();
        let mut unmatched = 0u32;
        loop {
            self.check().await?;
            let source = self.source.clone();
            let a = *after;
            let triggered = self.triggered;
            let focus = MaskFocus {
                label: Some(tracker.label.clone()),
                box_norm: tracker.predicted,
                ttl_ms: 2000,
            };
            let get = async move {
                if triggered {
                    // the triggered frame is taken once the arm has settled
                    tokio::time::sleep(not_before.saturating_duration_since(Instant::now())).await;
                    source.trigger_with_focus(GRASP_PIPELINES, TRIGGER_WAIT_MS, Some(&focus)).await
                } else {
                    source.next_frame(a, 200).await
                }
            };
            let next = tokio::select! {
                r = get => r,
                _ = async { loop { tokio::time::sleep(POLL).await; if !self.alive().await { break; } } } => return Err(Stop::Cancelled),
            };
            let mut frame = match next {
                Ok(Some(frame)) => frame,
                Ok(None) => {
                    if last_frame.elapsed().as_secs_f64() > self.setup.lost_timeout_s.max(3.0) {
                        return Err(fail("TARGET_LOST", "No detection frames arrived"));
                    }
                    continue;
                }
                Err(e) => return Err(fail("DETECTION_UNAVAILABLE", format!("Detection source failed: {e}"))),
            };
            if !self.same_models(&frame) { return Err(fail("DETECTION_UNAVAILABLE", "model changed during grasp")); }
            rescale_frame_to_cam(&mut frame, &self.cam);
            *after = Some(frame.frame_id);
            last_frame = Instant::now();
            if frame.captured_at < not_before {
                if last_frame > observe_until {
                    return Err(fail("TARGET_LOST", format!("No frame captured after the motion settled within {:.1} s", self.setup.lost_timeout_s)));
                }
                continue;
            }
            if let Some(d) = tracker.find(&frame, &self.setup) { return Ok(d); }
            unmatched += 1;
            if unmatched >= self.setup.lost_frames || started.elapsed().as_secs_f64() > self.setup.lost_timeout_s {
                return Err(fail("TARGET_LOST", format!("Target '{}' not seen for {unmatched} frames / {:.1} s", tracker.label, started.elapsed().as_secs_f64())));
            }
        }
    }

    async fn run(&self, planned: &PlannedGrasp) -> Result<(bool, Option<f32>), Stop> {
        let s = &self.setup;
        let open = self.gripper_phase("open", 0.02, GripperCommand::Open).await?;
        let _ = open;

        // coarse approach: tool axis down, or (camera-down) optical axis down with the camera
        // centre above the estimate
        let camera_down = planned.camera_down;
        let pose0 = self.pose().await;
        let tcp0 = self.tcp_pose().await;
        // depth capture runs when depth is a height source or recorded (`height_record`), or
        // decides the yaw (`yaw_sources`); the depth yaw / width record and the depth width
        // source use it when it ran
        let depth = self.triggered && self.depth_available && self.lens_known
            && (s.height_sources.contains(&HeightSource::Depth) || s.height_record.contains(&HeightSource::Depth)
                || s.yaw_sources.contains(&YawSource::Depth));
        let depth_cache = if depth { self.depth_start_frame(planned, &pose0).await } else { DepthCache::default() };

        self.phase("approach", 0.1).await?;
        let target = planned.target;
        let is_5dof = self.sup.inner().read().await.config.arm_model.arm_dof() == 5;
        let r_ref_p = if camera_down { &pose0.p } else { &tcp0.p };
        let r_ref = if is_5dof { motion::apply_auto_yaw(r_ref_p, &target, pose0.r) } else { pose0.r };
        let goal = if camera_down { camera_down_pose(&r_ref, &target, s, &self.cam, &self.envelope) } else { approach_pose(&r_ref, &target, s) };
        if camera_down {
            let height = goal.p[2] - s.table_z_m;
            if height > s.approach_height_m + 1e-6 {
                tracing::info!("Grasp {}: camera-down acquisition at {height:.3} m (approach_height_m {:.3}) for table clearance", self.id, s.approach_height_m);
                self.sup.inner().write().await.set_action_acquire_height(self.id, height as f32);
            }
        }
        let approach_frame = if camera_down { GoalFrame::EndLink } else { GoalFrame::Tcp };
        let done = self.move_arm_in(approach_frame, "approach", GoalSpec::Absolute { p: goal.p, r: Some(goal.r) }, s.approach_speed_mps).await?;

        // closed-loop centring onto the grip pixel, or (camera-down) the image centre
        self.phase("centring", 0.3).await?;
        self.set_mask_focus(Some(planned.request.label.clone()), planned.request.box_norm, 2000).await;
        let grip_px = self.cam.to_px(s.grip_pixel_norm);
        let (aim_px, aim_name) = if camera_down { ([self.cam.cx, self.cam.cy], "the image centre") } else { (grip_px, "the grip point") };
        let plane = grasp_plane(s);
        let pose = self.pose().await;
        let predicted = self.cam.project(&pose, &target).map(|p| self.cam.to_norm(p)).unwrap_or(s.grip_pixel_norm);
        let mut tracker = Tracker { label: planned.request.label.clone(), predicted: planned.request.box_norm, gate_px: s.acquire_dist_px };
        tracker.predict_at(predicted);
        let mut after = None;
        let mut c = self.centre("centring", 0.3, &mut tracker, aim_px, aim_name, plane, done.at, &mut after, s.max_iterations, true).await?;
        // a box cut by the image border at the aim (look-down v5: the point under the jaws is
        // near the bottom edge) has its centre off the target's: centre the box uncut at an aim
        // inside the image; the later steps use those frames, and the `shift` phase moves the
        // target onto the aim
        let mut centred_at = aim_px;
        for attempt in 0..BORDER_SHIFTS {
            let Some(inner) = border_free_aim(&c.frames, centred_at, self.cam.size, s.centre_tol_px + 4.0, attempt) else { break };
            tracing::info!("Grasp {}: box cut by the image border at {aim_name}, centring at ({:.0}, {:.0}) px", self.id, inner[0], inner[1]);
            c = self.centre("centring", 0.3, &mut tracker, inner, "the border-free aim", plane, Instant::now(), &mut after, s.max_iterations, true).await?;
            centred_at = inner;
        }
        let shifted = centred_at != aim_px;
        let cut = c.frames.iter().any(|d| touches_border(&d.box_norm, self.cam.size));
        if shifted || cut {
            self.sup.inner().write().await.update_action(self.id, |a| {
                a.border = Some(if cut { "cut" } else { "shifted" }.into());
                a.border_shift_px = shifted.then(|| [(centred_at[0] - aim_px[0]) as f32, (centred_at[1] - aim_px[1]) as f32]);
            });
        }
        let last = c.last.clone();

        // the overhead depth trigger replaces `after` and `c.last` / `c.frames` with its frame
        // whenever it runs, also when depth is record-only (only the height selection ignores it)
        let (depth_meas, depth_why) = if depth {
            match self.depth_overhead(&planned.request.label, &mut c, &mut after, &pose0, depth_cache).await {
                Ok(m) => (Some(m), "no_measurement"),
                Err(why) => (None, why),
            }
        } else {
            (None, "unavailable")
        };
        let depth_res = depth_meas.as_ref().ok_or(depth_why);

        // object height: record-only sources first (they do not move the arm), then the
        // `height_sources` in order until one accepts; `grasp_height_m` when none does. Box
        // centres map onto the object's top plane, the jaws close part way up the object
        let label = &planned.request.label;
        let record: Vec<HeightSource> = s.height_record.iter().copied().filter(|x| !s.height_sources.contains(x)).collect();
        let mut chain = EstimateChain::new();
        for src in &record {
            let (name, value, r) = if *src == HeightSource::Depth { self.depth_source(depth, depth_res) } else { self.prior_source(label) };
            chain.offer(&name, value, r, true);
        }
        self.write_estimates(&chain).await;
        let mut reasons: Vec<String> = Vec::new();
        for src in &s.height_sources {
            if chain.done() { break; }
            // a prior does not override a measured too-tall result
            if *src == HeightSource::Prior && chain.vetoed() { continue; }
            let (name, value, r) = match src {
                HeightSource::Depth => self.depth_source(depth, depth_res),
                HeightSource::Prior => self.prior_source(label),
                // only a too-tall measurement is a result; stop / fault / other failures end the grasp
                HeightSource::Probe => match self.probe_height(&c, label, &mut after).await {
                    Ok(Ok((h, source))) => (source.to_string(), None, SourceResult::Accept(h)),
                    Ok(Err(reason)) => ("probe".to_string(), None, SourceResult::Reject(reason)),
                    Err(Stop::Fail(code, msg)) if code == "OBJECT_TOO_TALL" => ("probe".to_string(), None, SourceResult::Veto(Stop::Fail(code, msg))),
                    Err(stop) => return Err(stop),
                },
            };
            // `height_reason` keeps the reasons of the non-depth sources (depth's are in `height_estimates`)
            if let (SourceResult::Reject(reason), false) = (&r, *src == HeightSource::Depth) { reasons.push(reason.clone()); }
            chain.offer(&name, value, r, false);
            self.write_estimates(&chain).await;
        }
        let chosen = match chain.finish() { Ok((chosen, _)) => chosen, Err((stop, _)) => return Err(stop) };

        let mut close_h = s.grasp_height_m;
        let mut plane = plane;
        let depth_chosen = chosen.as_ref().is_some_and(|(_, source)| source.starts_with("depth"));
        match chosen {
            Some((h, source)) => {
                plane = s.table_z_m + s.grasp_height_m.max(h);
                close_h = (h * s.height_grasp_fraction).clamp(s.grasp_height_m, s.max_grasp_height_m);
                tracing::info!("Grasp {}: object top {h:.3} m above the table ({source}), closing at {close_h:.3} m", self.id);
                self.sup.inner().write().await.update_action(self.id, |a| {
                    a.height_source = Some(if depth_chosen { "depth".into() } else { source });
                    a.object_height_m = Some(h as f32);
                });
            }
            None => self.sup.inner().write().await.update_action(self.id, |a| {
                a.height_source = Some("setup".into());
                a.height_reason = (!reasons.is_empty()).then(|| reasons.join(","));
            }),
        }
        // depth measurement fields: when depth decided or is record-only (diagnostic)
        if let Some(meas) = depth_meas.as_ref().filter(|_| depth_chosen || record.contains(&HeightSource::Depth)) {
            self.sup.inner().write().await.update_action(self.id, |a| {
                a.depth_anchor = Some(meas.source.into());
                a.object_depth_m = Some(meas.depth_m as f32);
                a.object_length_m = meas.length_m.map(|l| l as f32);
                a.jaw_clear = meas.jaw_clear;
            });
        }
        self.sup.inner().write().await.update_action(self.id, |a| a.close_height_m = Some(close_h as f32));

        // look-down, centred at a border-free aim: one base move that puts the TCP over the box
        // centre's point on the plane z = `plane`, seen from the centring pose. The point under
        // the TCP on that plane is the grip pixel (the table point under the jaws) moved by the
        // model's parallax between the table and the plane (camera-down: the transition starts
        // from the box centre's ray anyway)
        if shifted && !camera_down {
            self.phase("shift", 0.5).await?;
            let mut xs: Vec<f64> = c.frames.iter().map(|d| self.cam.to_px(box_centre(&d.box_norm))[0]).collect();
            let mut ys: Vec<f64> = c.frames.iter().map(|d| self.cam.to_px(box_centre(&d.box_norm))[1]).collect();
            let centre = [median(&mut xs), median(&mut ys)];
            let tcp_c = motion::end_to_tcp(&c.pose, &self.tcp_offset);
            let under = |z: f64| self.cam.project(&c.pose, &[tcp_c.p[0], tcp_c.p[1], z]);
            let aim = match (under(plane), under(s.table_z_m)) {
                (Some(m), Some(m0)) => [aim_px[0] + m[0] - m0[0], aim_px[1] + m[1] - m0[1]],
                _ => aim_px,
            };
            let (Some(pt), Some(pg)) = (self.cam.plane_point(&c.pose, centre, plane), self.cam.plane_point(&c.pose, aim, plane)) else {
                return Err(fail("CENTRING_FAILED", "Border shift cannot be mapped onto the table plane"));
            };
            let d = [pt[0] - pg[0], pt[1] - pg[1], 0.0];
            self.move_arm_in(GoalFrame::Tcp, "shift", GoalSpec::Relative { frame: RelFrame::Base, d, dyaw: 0.0, dpitch: 0.0, droll: 0.0, level: false, auto_yaw: None }, s.approach_speed_mps).await?;
        }

        // camera-down: re-orient to the tool-vertical pose above the target's table point
        if camera_down {
            self.phase("transition", 0.5).await?;
            // the ray of the last centring box from the pose it was seen at (before the probe)
            let Some((t, goal)) = transition_pose(&c.pose, self.cam.to_px(box_centre(&last.box_norm)), plane, s, &self.cam) else {
                return Err(fail("CENTRING_FAILED", "The target ray does not meet the table plane"));
            };
            let done = self.move_arm_in(GoalFrame::Tcp, "transition", GoalSpec::Absolute { p: goal.p, r: Some(goal.r) }, s.approach_speed_mps).await?;
            // verify the target where the model expects it before descending
            self.recheck(&planned.request.label, &last, &t, grip_px, done.at, &mut after).await?;
        }

        // J6 alignment: close the jaws across the object's short side
        self.phase("align", 0.55).await?;
        let (turn, align) = if s.align_j6 { self.align_turn(&c, plane).await } else { (None, "off".to_string()) };
        self.sup.inner().write().await.update_action(self.id, |a| { a.align = Some(align.clone()); a.align_turn_rad = turn.map(|t| t as f32); });

        // grasp width: `width_record` sources first (reported only), then `width_sources` in
        // order until one accepts; one limit check on the chosen width. The width is measured
        // whatever the angle confidence: a disc or a square has a low angle_conf but a valid
        // width (any closing direction sees about the same width)
        let width_record: Vec<WidthSource> = s.width_record.iter().copied().filter(|x| !s.width_sources.contains(x)).collect();
        let mut wchain = EstimateChain::new();
        let width_of = |src: WidthSource| -> SourceResult {
            let r = match src {
                WidthSource::Mask => self.mask_width_m(&c, plane),
                WidthSource::Depth => match depth_res {
                    Ok(m) => m.width_m.ok_or("no_width"),
                    Err(why) => Err(if depth { why } else { "unavailable" }),
                },
            };
            match r { Ok(w) => SourceResult::Accept(w), Err(why) => SourceResult::Reject(why.into()) }
        };
        for src in &width_record {
            wchain.offer(src.name(), None, width_of(*src), true);
        }
        for src in &s.width_sources {
            if wchain.done() { break; }
            wchain.offer(src.name(), None, width_of(*src), false);
        }
        let (chosen_width, width_entries) = match wchain.finish() { Ok(r) => r, Err((_, e)) => (None, e) };
        let measured_width_m = chosen_width.map(|(w, _)| w);
        self.sup.inner().write().await.update_action(self.id, |a| {
            a.width_estimates = width_entries;
            if let Some(w) = measured_width_m { a.grasp_width_m = Some(w as f32); }
        });
        if let Some(w) = measured_width_m {
            let max_w = self.max_grasp_width().await;
            if max_w > 0.0 && w > max_w {
                return Err(fail("TOO_WIDE", format!("Object width {w:.3} m exceeds max_grasp_width_m {:.3} m", max_w)));
            }
        }

        if let Some(turn) = turn {
            self.move_arm_in(GoalFrame::Tcp, "align", GoalSpec::Relative { frame: RelFrame::Tool, d: [0.0; 3], dyaw: 0.0, dpitch: 0.0, droll: turn, level: false, auto_yaw: None }, s.approach_speed_mps).await?;
        }

        // width-based pre-open before descend
        let (pre_open_decision, pre_open_target) = {
            let inner = self.sup.inner().read().await;
            let policy = inner.config.gripper_policy.clone();
            drop(inner);
            match policy {
                None => (Some("skipped:no_policy".to_string()), None),
                Some(policy) => {
                    let calibrated = policy.is_calibrated();
                    if s.pre_open == PreOpen::Off {
                        (Some("off".to_string()), None)
                    } else if s.pre_open == PreOpen::Auto && !calibrated {
                        (Some("skipped:uncalibrated".to_string()), None)
                    } else if !matches!(align.as_str(), "turned:angle" | "kept:angle")
                        && frames_angle(&c.frames, s.align_min_angle_conf).is_some()
                    {
                        // an elongated mask (confident angle): its narrowest cross-section is the
                        // width across the jaws only once J6 is aligned to it; a round or square
                        // mask (no confident angle) has about the same width in every direction
                        (Some("skipped:not_aligned".to_string()), None)
                    } else if let Err(reason) = align_guard(&c.last, &c.frames, self.cam.size) {
                        (Some(format!("skipped:{reason}")), None)
                    } else if let Some(w) = measured_width_m {
                        let target_w = (w + s.pre_open_margin_m) as f32;
                        let target_j7 = policy.j7_for(target_w);
                        let full_open_w = policy.opening_m(policy.open_rad);
                        let at_or_beyond_open = target_w >= full_open_w
                            || (target_j7 - policy.open_rad).abs() < 1e-4
                            || if policy.open_rad > policy.closed_rad {
                                target_j7 >= policy.open_rad
                            } else {
                                target_j7 <= policy.open_rad
                            };
                        if at_or_beyond_open {
                            (Some("skipped:open_rad".to_string()), None)
                        } else {
                            let min_j7 = policy.closed_rad.min(policy.open_rad);
                            let max_j7 = policy.closed_rad.max(policy.open_rad);
                            let clamped_j7 = target_j7.clamp(min_j7, max_j7);
                            let span = policy.open_rad - policy.closed_rad;
                            let frac = if span.abs() < 1e-6 { 0.0 } else { ((clamped_j7 - policy.closed_rad) / span).clamp(0.0, 1.0) as f64 };
                            (None, Some((frac, target_w)))
                        }
                    } else {
                        (Some("skipped:no_width".to_string()), None)
                    }
                }
            }
        };

        if let Some((fraction, target_w)) = pre_open_target {
            self.phase("preopen", 0.6).await?;
            self.gripper_move("preopen", fraction).await?;
            self.sup.inner().write().await.update_action(self.id, |a| {
                a.pre_open = Some("done".into());
                a.pre_open_m = Some(target_w);
            });
        } else if let Some(reason) = pre_open_decision {
            self.sup.inner().write().await.update_action(self.id, |a| {
                a.pre_open = Some(reason);
            });
        }

        // descend (look-down: re-centre at mid height on the way), close, check
        self.phase("descend", 0.65).await?;
        if !camera_down {
            let (recentre, iterations) = self.recentre(&c, plane, close_h, &planned.request.label, &mut after).await?;
            self.sup.inner().write().await.update_action(self.id, |a| { a.recentre = Some(recentre); a.recentre_iterations = iterations; });
            self.phase("descend", 0.75).await?;
        }
        self.descend(close_h, planned.request.approach).await?;
        let closed = self.gripper_phase("close", 0.8, GripperCommand::Close).await?;
        let held_width = {
            let inner = self.sup.inner().read().await;
            let require_real = inner.requires_real_jaw_curve();
            inner.config.gripper_policy.as_ref().and_then(|p| {
                if require_real && p.jaw_opening.is_none() {
                    return None;
                }
                let idx = inner.config.gripper_index().unwrap_or(6);
                let m = inner.measured_positions.get(idx).copied().unwrap_or(0.0);
                Some(p.opening_m(m))
            })
        };
        if let Some(w) = held_width {
            self.sup.inner().write().await.update_action(self.id, |a| a.held_width_m = Some(w));
        }
        let holding = closed.holding == Some(true);
        let lift_to = |p: Vec3| GoalSpec::Absolute { p: [p[0], p[1], s.table_z_m + s.lift_height_m], r: None };
        if !holding {
            self.gripper_phase("release", 0.85, GripperCommand::Open).await?;
            self.phase("lift", 0.9).await?;
            let p = self.tcp_pose().await.p;
            self.move_arm_in(GoalFrame::Tcp, "lift", lift_to(p), s.descend_speed_mps).await?;
            return Err(fail("NOT_HOLDING", format!("Jaws closed without holding the object (gap {:.2} rad)", closed.gripper_gap_rad.unwrap_or(0.0))));
        }
        self.phase("lift", 0.9).await?;
        let p = self.tcp_pose().await.p;
        self.move_arm_in(GoalFrame::Tcp, "lift", lift_to(p), s.descend_speed_mps).await?;
        let r_lift = self.pose().await.r;
        let tool_z = kinematics::column(&r_lift, 2)[2];
        let tilt = (-tool_z).clamp(-1.0, 1.0).acos().to_degrees();
        tracing::info!("Grasp {}: lift tilt {:.2}°", self.id, tilt);
        // the object may slip out while lifting. Under position control the jaw settles on its
        // hold target and the grip torque fades when the object only shifted or turned in the
        // jaws, so a failed torque rule is decided by a re-grip: a close with grasp detection
        // stops on the object (holding) or runs to closed (dropped)
        let (held, gap) = self.holding_now().await?;
        if held {
            self.sup.inner().write().await.update_action(self.id, |a| a.post_lift = Some("held".into()));
            return Ok((true, closed.gripper_gap_rad));
        }
        tracing::info!("Grasp {}: grip torque gone after the lift (gap {gap:.2} rad), re-gripping", self.id);
        let regrip = self.gripper_phase("regrip", 0.95, GripperCommand::Close).await?;
        let gap = regrip.gripper_gap_rad.unwrap_or(gap);
        if regrip.holding == Some(true) {
            let held_width = {
                let inner = self.sup.inner().read().await;
                let require_real = inner.requires_real_jaw_curve();
                inner.config.gripper_policy.as_ref().and_then(|p| {
                    if require_real && p.jaw_opening.is_none() {
                        return None;
                    }
                    let idx = inner.config.gripper_index().unwrap_or(6);
                    let m = inner.measured_positions.get(idx).copied().unwrap_or(0.0);
                    Some(p.opening_m(m))
                })
            };
            if let Some(w) = held_width {
                self.sup.inner().write().await.update_action(self.id, |a| a.held_width_m = Some(w));
            }
            self.sup.inner().write().await.update_action(self.id, |a| a.post_lift = Some("regripped".into()));
            return Ok((true, Some(gap)));
        }
        self.sup.inner().write().await.update_action(self.id, |a| { a.post_lift = Some("dropped".into()); a.gripper_gap_rad = Some(gap); });
        Err(fail("DROPPED", format!("Object no longer held after the lift: the re-grip closed to {gap:.2} rad from closed")))
    }

    /// `holding` after `post_lift_settle_ms` from the measured gripper joint, with the gripper policy's rule
    /// (matching GripperMotion in motion.rs). Returns (holding, gap).
    async fn holding_now(&self) -> Result<(bool, f32), Stop> {
        let until = Instant::now() + Duration::from_millis(self.setup.post_lift_settle_ms);
        let mut taus = Vec::new();
        let mut loads = Vec::new();
        let mut vels = Vec::new();
        loop {
            self.check().await?;
            let (gap, tau, load, vel, policy, feedback_fresh, torque_feedback, load_feedback) = {
                let inner = self.sup.inner().read().await;
                let Some(policy) = inner.config.gripper_policy.clone() else {
                    return Err(fail("UNSUPPORTED", "No gripper policy"));
                };
                let g_idx = inner.config.gripper_index().unwrap_or(6);
                let dir = if policy.closed_rad >= policy.open_rad { 1.0 } else { -1.0 };
                let m_g = inner.measured_positions.get(g_idx).copied().unwrap_or(f32::NAN);
                let gap = (policy.closed_rad - m_g) * dir;
                let tau = inner.torques.get(g_idx).copied().unwrap_or(0.0).abs();
                let load = inner.loads.get(g_idx).copied().unwrap_or(0.0).abs();
                let vel = inner.measured_velocities.get(g_idx).copied().unwrap_or(0.0);

                let (torque_fb, load_fb) = if let Some(ref d) = inner.driver {
                    if let Ok(driver) = d.lock() {
                        let caps = driver.capabilities();
                        (caps.torque_feedback, caps.load_feedback)
                    } else {
                        (false, false)
                    }
                } else {
                    (true, !inner.loads.is_empty())
                };
                let fresh = if inner.driver.is_some() {
                    inner.hardware_feedback_fresh(Duration::from_millis(250))
                } else {
                    true
                };

                (gap, tau, load, vel, policy, fresh, torque_fb, load_fb)
            };
            taus.push(f64::from(tau));
            loads.push(f64::from(load));
            vels.push(f64::from(vel));

            if Instant::now() >= until {
                let tau_median = if taus.is_empty() { 0.0 } else { median(&mut taus) };
                let load_median = if loads.is_empty() { 0.0 } else { median(&mut loads) };
                let vel_median = if vels.is_empty() { 0.0 } else { median(&mut vels) };

                let holding = match policy.holding_mode {
                    HoldingMode::TorqueAndGap => {
                        torque_feedback
                            && gap >= policy.holding_gap_rad
                            && tau_median >= f64::from(policy.holding_torque_nm)
                    }
                    HoldingMode::PositionGap => {
                        feedback_fresh
                            && vel_median.abs() < motion::SETTLE_VEL
                            && gap >= policy.holding_gap_rad
                    }
                    HoldingMode::LoadAndGap => {
                        let req = policy.holding_load_fraction.unwrap_or(0.15) as f64;
                        feedback_fresh
                            && load_feedback
                            && gap >= policy.holding_gap_rad
                            && load_median >= req
                    }
                };
                return Ok((holding, gap));
            }
            tokio::time::sleep(POLL).await;
        }
    }

    /// After the camera-down transition, when the model projects the target `t` into the
    /// image: one fresh frame must show it (else TARGET_LOST / DETECTION_UNAVAILABLE), within
    /// `recheck_tol_px` of the projection (else TARGET_DISPLACED). Records the box centre minus
    /// `grip_px` as `final_offset_px`. Nothing is checked when the target is not expected in view.
    async fn recheck(&self, label: &str, last: &Detection, t: &Vec3, grip_px: [f64; 2], since: Instant, after: &mut Option<u64>) -> Result<(), Stop> {
        let pose = self.pose().await;
        let Some(p) = self.cam.project(&pose, t) else { return Ok(()) };
        if !(p[0] >= 0.0 && p[0] <= self.cam.size[0] && p[1] >= 0.0 && p[1] <= self.cam.size[1]) { return Ok(()); }
        let mut tracker = Tracker { label: label.to_string(), predicted: last.box_norm, gate_px: self.setup.acquire_dist_px };
        tracker.predict_at(self.cam.to_norm(p));
        let d = self.fresh_match(&tracker, since, after).await?;
        let c = self.cam.to_px(box_centre(&d.box_norm));
        self.sup.inner().write().await.set_action_final_offset(self.id, [(c[0] - grip_px[0]) as f32, (c[1] - grip_px[1]) as f32]);
        let moved = (c[0] - p[0]).hypot(c[1] - p[1]);
        if moved > self.setup.recheck_tol_px {
            return Err(fail("TARGET_DISPLACED", format!("Target {moved:.0} px from where it was expected after the transition (recheck_tol_px {:.0})", self.setup.recheck_tol_px)));
        }
        Ok(())
    }

    /// Closed-loop centring: drive the target's median box centre onto `aim_px` (pixel errors
    /// mapped onto the plane z = `plane`), at most `max_iter` corrections, else CENTRING_FAILED.
    /// `report`: publish `iteration` / `error_px` in the action status.
    #[allow(clippy::too_many_arguments)]
    async fn centre(&self, phase: &str, progress: f32, tracker: &mut Tracker, aim_px: [f64; 2], aim_name: &str, plane: f64,
                    mut since: Instant, after: &mut Option<u64>, max_iter: u32, report: bool) -> Result<Centred, Stop> {
        let s = &self.setup;
        let mut iteration = 0u32;
        let mut last_focus_box = tracker.predicted;
        loop {
            let pose = self.pose().await;
            let mut xs = Vec::new();
            let mut ys = Vec::new();
            let mut frames = Vec::new();
            let mut consecutive = 0u32;
            let decision = loop {
                let d = self.fresh_match(tracker, since, after).await?;
                tracker.gate_px = s.match_dist_px;
                tracker.predicted = d.box_norm;
                if self.triggered {
                    self.set_mask_focus(Some(tracker.label.clone()), d.box_norm, 2000).await;
                } else if box_changed_px(&d.box_norm, &last_focus_box, self.cam.size) {
                    self.set_mask_focus(Some(tracker.label.clone()), d.box_norm, 2000).await;
                    last_focus_box = d.box_norm;
                }
                let c = self.cam.to_px(box_centre(&d.box_norm));
                frames.push(d);
                xs.push(c[0]);
                ys.push(c[1]);
                let err = (c[0] - aim_px[0]).hypot(c[1] - aim_px[1]);
                if err <= s.centre_tol_px { consecutive += 1; } else { consecutive = 0; }
                let (it, e) = if report { (Some(iteration), Some(err as f32)) } else { (None, None) };
                self.sup.inner().write().await.set_action_phase(self.id, phase, progress, it, e);
                if consecutive >= s.stable_frames { break None; }
                let m = s.median_frames as usize;
                if xs.len() >= m {
                    let med = [median(&mut xs[xs.len() - m..].to_vec()), median(&mut ys[ys.len() - m..].to_vec())];
                    if (med[0] - aim_px[0]).hypot(med[1] - aim_px[1]) > s.centre_tol_px { break Some(med); }
                    if xs.len() >= m + s.stable_frames as usize { break None; }
                }
            };
            let Some(med) = decision else {
                let last = frames.last().cloned().expect("at least one matched frame");
                return Ok(Centred { last, frames, pose, iterations: iteration });
            };
            if iteration >= max_iter {
                let err = (med[0] - aim_px[0]).hypot(med[1] - aim_px[1]);
                return Err(fail("CENTRING_FAILED", format!("Target still {err:.0} px from {aim_name} after {iteration} corrections")));
            }
            iteration += 1;
            let (Some(pt), Some(pg)) = (self.cam.plane_point(&pose, med, plane), self.cam.plane_point(&pose, aim_px, plane)) else {
                return Err(fail("CENTRING_FAILED", "Pixel error cannot be mapped onto the table plane"));
            };
            let mut step = [(pt[0] - pg[0]) * s.servo_gain, (pt[1] - pg[1]) * s.servo_gain, 0.0];
            let len = step[0].hypot(step[1]);
            if len > s.max_step_m { step = [step[0] * s.max_step_m / len, step[1] * s.max_step_m / len, 0.0]; }
            // predict where the target lands after the step (same rough model)
            let moved = Pose::new(pose.r, kinematics::add(&pose.p, &step));
            if let Some(p) = self.cam.project(&moved, &pt) { tracker.predict_at(self.cam.to_norm(p)); }
            let done = self.move_arm(phase, GoalSpec::Relative { frame: RelFrame::Base, d: step, dyaw: 0.0, dpitch: 0.0, droll: 0.0, level: false, auto_yaw: None }, s.approach_speed_mps).await?;
            since = done.at;
        }
    }

    /// Start-frame depth: table samples (`two_height_fk` prior), the matched detection and its
    /// measurement at the start pose `pose0`.
    async fn depth_start_frame(&self, planned: &PlannedGrasp, pose0: &Pose) -> DepthCache {
        let s = &self.setup;
        let mut cache = DepthCache::default();
        let focus = MaskFocus {
            label: Some(planned.request.label.clone()),
            box_norm: planned.request.box_norm,
            ttl_ms: 2000,
        };
        // a frame of another model set is no depth frame (the next centring frame fails the grasp)
        if let Ok(Some(mut frame0)) = self.source.trigger_with_focus(&["yolo", "depth"], TRIGGER_WAIT_MS, Some(&focus)).await.map(|f| f.filter(|f| self.same_models(f))) {
            rescale_frame_to_cam(&mut frame0, &self.cam);
            if let Some(ref d) = frame0.depth {
                cache.prior = extract_table_obs(&self.cam, pose0, d, s.table_z_m);
                let cal0 = calibrate_depth_why(&self.cam, pose0, Some(d), None, s.table_z_m);
                let tracker0 = Tracker {
                    label: planned.request.label.clone(),
                    predicted: planned.request.box_norm,
                    gate_px: s.acquire_dist_px,
                };
                if let Some(det0) = tracker0.find(&frame0, s) {
                    cache.det0 = Some(det0.clone());
                    match cal0 {
                        Ok(ref cal) => match measure_detection_depth_with_cal_why(&self.cam, pose0, &det0, cal) {
                            Ok(m) => cache.meas = Some(m),
                            Err(why) => cache.why = Some(why),
                        },
                        Err(why) => cache.why = Some(why),
                    }
                } else {
                    cache.why = Some("target_lost");
                }
            } else {
                cache.why = Some("no_depth_frame");
            }
        } else {
            cache.why = Some("no_depth_frame");
        }
        cache
    }

    /// Overhead depth trigger at the centred pose: its frame becomes `after`, and its matched
    /// detection `c.last` (appended to `c.frames`). Returns the overhead measurement, else the
    /// start-frame one, else why there is none (the last failed attempt).
    async fn depth_overhead(&self, label: &str, c: &mut Centred, after: &mut Option<u64>, pose0: &Pose, cache: DepthCache) -> Result<DepthMeasurement, &'static str> {
        let s = &self.setup;
        let DepthCache { prior: cached_prior, meas: mut cached_meas, det0: cached_det0, why: cached_why } = cache;
        let mut depth_meas: Option<DepthMeasurement> = None;
        let mut why = cached_why.unwrap_or("no_depth_frame");
        let focus = MaskFocus {
            label: Some(label.to_string()),
            box_norm: c.last.box_norm,
            ttl_ms: 2000,
        };
        if let Ok(Some(mut overhead_frame)) = self.source.trigger_with_focus(&["yolo", "depth"], TRIGGER_WAIT_MS, Some(&focus)).await.map(|f| f.filter(|f| self.same_models(f))) {
            rescale_frame_to_cam(&mut overhead_frame, &self.cam);
            *after = Some(overhead_frame.frame_id);
            if metric_table_contradicts_fk(&self.cam, &c.pose, overhead_frame.depth.as_ref(), s.table_z_m) {
                tracing::debug!("Overhead table samples contradict FK; invalidating cached start-frame depth measurement");
                cached_meas = None;
                why = "table_contradicts_fk";
            }
            let tracker = Tracker {
                label: label.to_string(),
                predicted: c.last.box_norm,
                gate_px: s.match_dist_px,
            };
            if let Some(overhead_det) = tracker.find(&overhead_frame, s) {
                c.last = overhead_det.clone();
                c.frames.push(overhead_det.clone());

                match calibrate_depth_why(&self.cam, &c.pose, overhead_frame.depth.as_ref(), None, s.table_z_m)
                    .and_then(|cal| measure_detection_depth_with_cal_why(&self.cam, &c.pose, &overhead_det, &cal)) {
                    Ok(m) => depth_meas = Some(m),
                    Err(w) => why = w,
                }
                if depth_meas.is_none() {
                    depth_meas = cached_meas.clone();
                }
                if depth_meas.is_none() {
                    match calibrate_depth_why(&self.cam, &c.pose, overhead_frame.depth.as_ref(), cached_prior.as_ref(), s.table_z_m) {
                        Ok(cal) if cal.source == "two_height_fk" => {
                            if let Some(ref det0) = cached_det0 {
                                match measure_detection_depth_with_cal_why(&self.cam, &c.pose, &overhead_det, &cal) {
                                    Ok(m1) => match measure_detection_depth_with_cal_why(&self.cam, pose0, det0, &cal) {
                                        Ok(m0) => {
                                            if (m0.height_m - m1.height_m).abs() <= (0.20 * m1.height_m).max(0.008) {
                                                depth_meas = Some(m1);
                                            } else {
                                                tracing::debug!("two_height_fk rejected: height drifted between frames (m0={:.3}, m1={:.3})", m0.height_m, m1.height_m);
                                                why = "two_height_mismatch";
                                            }
                                        }
                                        Err(w) => why = w,
                                    },
                                    Err(w) => why = w,
                                }
                            }
                        }
                        Ok(cal) => match measure_detection_depth_with_cal_why(&self.cam, &c.pose, &overhead_det, &cal) {
                            Ok(m) => depth_meas = Some(m),
                            Err(w) => why = w,
                        },
                        Err(w) => why = w,
                    }
                }
            } else {
                why = "target_lost";
            }
        }
        depth_meas.or(cached_meas).ok_or(why)
    }

    /// Height source `depth` (entry name `depth:<anchor>`); `ran`: the depth capture ran.
    fn depth_source(&self, ran: bool, meas: Result<&DepthMeasurement, &'static str>) -> (String, Option<f64>, SourceResult) {
        let reject = |r: &str| ("depth".to_string(), None, SourceResult::Reject(r.into()));
        match meas {
            Ok(m) => (format!("depth:{}", m.source), Some(m.height_m), self.height_result(m.height_m)),
            Err(why) if ran => reject(why),
            Err(_) if !self.lens_known => reject("no_lens_centre"),
            Err(_) => reject("unavailable"),
        }
    }

    /// Height source `prior`: `height_priors` for the target's label (case-insensitive).
    fn prior_source(&self, label: &str) -> (String, Option<f64>, SourceResult) {
        match self.setup.height_priors.iter().find(|(k, _)| k.eq_ignore_ascii_case(label)) {
            Some((_, &h)) => ("prior".into(), Some(h), self.height_result(h)),
            None => ("prior".into(), None, SourceResult::Reject("no_prior".into())),
        }
    }

    fn height_result(&self, h: f64) -> SourceResult {
        match self.accept_height(h) {
            Ok(Ok(h)) => SourceResult::Accept(h),
            Ok(Err(reason)) => SourceResult::Reject(reason),
            Err(stop) => SourceResult::Veto(stop),
        }
    }

    async fn write_estimates(&self, chain: &EstimateChain) {
        let entries = chain.entries().to_vec();
        self.sup.inner().write().await.update_action(self.id, |a| a.height_estimates = entries);
    }

    /// Height probe after the centring (`height_probe_mode`): `Ok(Ok((h, source)))` with the
    /// object's top h above the table and `source` `scale` (up probe) or `parallax` (lateral
    /// probe); `Ok(Err(reason))` when the probe was skipped or its result is unreliable (the
    /// grasp goes on with `grasp_height_m`); OBJECT_TOO_TALL when h >= `approach_height_m` − 0.02.
    /// Both modes need the optical axis within 5 deg of vertical and the lens centre; the
    /// lateral probe gives way to the up probe (`lateral_fallback:<why>` in `height_reason`)
    /// when the shifted box would reach the image border, a lateral pose is unreachable, or the
    /// target was cut off or lost at a lateral pose.
    async fn probe_height(&self, c: &Centred, label: &str, after: &mut Option<u64>) -> Result<Result<(f64, &'static str), String>, Stop> {
        let s = &self.setup;
        let skip = |r: &str| Ok(Err(r.to_string()));
        if s.height_probe_m <= 0.0 { return skip("disabled"); }
        if !self.lens_known { return skip("no_lens_centre"); }
        if kinematics::column(&self.cam.rotation(&c.pose), 2)[2] > -(5f64.to_radians().cos()) { return skip("camera_not_vertical"); }
        if c.frames.iter().any(|d| touches_border(&d.box_norm, self.cam.size)) { return skip("border"); }
        let centre_px = self.cam.to_px(box_centre(&c.last.box_norm));
        let Some(est) = self.cam.plane_point(&c.pose, centre_px, grasp_plane(s)) else { return skip("ray"); };
        let mut fallback = None;
        if s.height_probe_mode == HeightProbeMode::Lateral {
            match self.probe_lateral(c, &est, label, after).await? {
                Lateral::Done(r) => return Ok(match r { Ok(h) => self.accept_height(h)?.map(|h| (h, "parallax")), Err(e) => Err(e) }),
                Lateral::Fallback(why) => {
                    tracing::info!("Grasp {}: lateral height probe falls back to up ({why})", self.id);
                    fallback = Some(format!("lateral_fallback:{why}"));
                }
            }
        }
        let up = match self.probe_up(c, &est, label, after).await? { Ok(h) => self.accept_height(h)?, Err(e) => Err(e) };
        Ok(match (up, fallback) {
            (Ok(h), Some(f)) => {
                self.sup.inner().write().await.update_action(self.id, |a| a.height_reason = Some(f));
                Ok((h, "scale"))
            }
            (Ok(h), None) => Ok((h, "scale")),
            (Err(e), Some(f)) => Err(format!("{f},{e}")),
            (Err(e), None) => Err(e),
        })
    }

    /// The acceptance rules shared by both probe modes: finite, not below the table, and
    /// OBJECT_TOO_TALL when h >= `approach_height_m` − 0.02.
    fn accept_height(&self, h: f64) -> Result<Result<f64, String>, Stop> {
        let s = &self.setup;
        if !h.is_finite() { return Ok(Err("not_finite".into())); }
        if h < 0.0 { return Ok(Err(format!("below_table:{h:.3}"))); }
        if h >= s.approach_height_m - 0.02 {
            return Err(fail("OBJECT_TOO_TALL", format!("Object top {h:.3} m above the table, approach_height_m {:.3} m leaves less than 0.02 m", s.approach_height_m)));
        }
        Ok(Ok(h))
    }

    /// Up probe: from the centred pose `c.pose` move straight up by `height_probe_m`, take
    /// `median_frames` fresh matched frames, and return to `c.pose`. The median box size
    /// s = sqrt(w·h) (px) of the centring frames (s_low) and of the raised frames (s_high) give
    /// the lens-to-top distance at the start pose D_low = dz · s_high / (s_low − s_high)
    /// ([`scale_height`]); top height = lens z − D_low − `table_z_m` (not yet accepted). Moving
    /// up keeps the jaws away from an object whose height is not known yet.
    async fn probe_up(&self, c: &Centred, est: &Vec3, label: &str, after: &mut Option<u64>) -> Result<Result<f64, String>, Stop> {
        let s = &self.setup;
        let pose = c.pose;
        let size = |d: &Detection| (((d.box_norm[2] - d.box_norm[0]) * self.cam.size[0]) * ((d.box_norm[3] - d.box_norm[1]) * self.cam.size[1])).sqrt();
        let low: Vec<f64> = c.frames.iter().map(size).collect();
        self.phase("probe", 0.45).await?;
        // height_probe_m, then the shorter fallback, when the planner rejects the raised pose
        let mut dzs = vec![s.height_probe_m];
        if s.height_probe_m > PROBE_FALLBACK_DZ_M + 1e-9 { dzs.push(PROBE_FALLBACK_DZ_M); }
        let mut moved = None;
        for dz in dzs {
            let up = Pose::new(pose.r, [pose.p[0], pose.p[1], pose.p[2] + dz]);
            if table_violation(&pose, &up, &self.envelope, s.table_z_m).is_some() { continue; }
            let spec = GoalSpec::Relative { frame: RelFrame::Base, d: [0.0, 0.0, dz], dyaw: 0.0, dpitch: 0.0, droll: 0.0, level: false, auto_yaw: None };
            match self.move_arm("probe", spec, s.approach_speed_mps).await {
                Ok(d) => { moved = Some((d, dz)); break; }
                // rejected by the planner before anything was installed: the arm is still at the start
                Err(Stop::Fail(code, _)) if code.starts_with("IK_") || code == "WORKSPACE_VIOLATION" => {}
                Err(e) => return Err(e),
            }
        }
        let Some((done, dz)) = moved else { return Ok(Err("probe_unreachable".into())); };
        self.sup.inner().write().await.update_action(self.id, |a| { a.probe_dz_m = Some(dz as f32); a.probe_mode = Some("up".into()); });
        let now = self.pose().await;
        let mut tracker = Tracker { label: label.to_string(), predicted: c.last.box_norm, gate_px: s.acquire_dist_px };
        if let Some(p) = self.cam.project(&now, est) { tracker.predict_at(self.cam.to_norm(p)); }
        let mut high = Vec::new();
        let mut reason = None;
        while high.len() < s.median_frames as usize {
            let d = match self.fresh_match(&tracker, done.at, after).await {
                Ok(d) => d,
                Err(Stop::Fail(code, _)) if code == "TARGET_LOST" => { reason = Some("target_lost".to_string()); break; }
                Err(e) => return Err(e),
            };
            if touches_border(&d.box_norm, self.cam.size) { reason = Some("border".to_string()); break; }
            tracker.gate_px = s.match_dist_px;
            tracker.predicted = d.box_norm;
            high.push(size(&d));
        }
        // back to the start pose: the later steps start from it
        self.move_arm("probe", GoalSpec::Absolute { p: pose.p, r: Some(pose.r) }, s.approach_speed_mps).await?;
        if let Some(r) = reason { return Ok(Err(r)); }
        let (z_low, z_high) = (self.cam.origin(&pose)[2], self.cam.origin(&now)[2]);
        Ok(scale_height(&low, &high, z_high - z_low).map(|d_low| z_low - d_low - s.table_z_m))
    }

    /// Lateral probe: from the centred pose `c.pose` move horizontally (orientation kept) to
    /// −B/2 and then +B/2 along the base-frame direction of the image x axis (B =
    /// `lateral_probe_m`), take `median_frames` fresh matched frames at each pose, and return to
    /// `c.pose`. Before moving: [`Lateral::Fallback`] `border` when the box shifted by the
    /// predicted parallax fx · B/2 / D0 (D0 = lens height above the grasp plane) comes within
    /// 2 px of the image border at either pose, `unreachable` when either pose fails the table
    /// clearance or the planner (IK, workspace), `edge` when no box feature is on the top face at
    /// both predicted (or, after the moves, seen) poses ([`top_face_u`]). The distance D = fx · B_meas / |du|
    /// ([`parallax_distance`]) uses the median u of a top-face feature at each pose
    /// ([`top_face_u`]: box centre or outer edge) and B_meas, the image-x
    /// component of the lens displacement between the poses from FK of the measured joints;
    /// top height = mean measured lens z − D − `table_z_m` (not yet accepted).
    pub async fn probe_lateral(&self, c: &Centred, est: &Vec3, label: &str, after: &mut Option<u64>) -> Result<Lateral, Stop> {
        if self.sup.inner().read().await.config.arm_model.arm_dof() == 5 {
            return Ok(Lateral::Fallback("unreachable"));
        }
        let s = &self.setup;
        let pose = c.pose;
        let x_cam = kinematics::column(&self.cam.rotation(&pose), 0);
        let n = x_cam[0].hypot(x_cam[1]);
        // within 5 deg of a vertical optical axis the image x axis is nearly horizontal
        if n < 0.5 { return Ok(Lateral::Fallback("direction")); }
        let half = s.lateral_probe_m / 2.0;
        let at = |k: f64| Pose::new(pose.r, [pose.p[0] + k * half * x_cam[0] / n, pose.p[1] + k * half * x_cam[1] / n, pose.p[2]]);
        let (neg, pos) = (at(-1.0), at(1.0));
        // predicted shift: a point at D0 moves by −fx · dx / D0 when the camera moves dx along x_cam
        let d0 = self.cam.origin(&pose)[2] - grasp_plane(s);
        if d0.is_nan() || d0 <= 0.01 { return Ok(Lateral::Fallback("border")); }
        let shift_norm = self.cam.fx * half * n / d0 / self.cam.size[0];
        let shifted = |b: &[f64; 4], du: f64| [b[0] + du, b[1], b[2] + du, b[3]];
        if c.frames.iter().any(|d| [shift_norm, -shift_norm].iter().any(|du| touches_border(&shifted(&d.box_norm, *du), self.cam.size))) {
            return Ok(Lateral::Fallback("border"));
        }
        // a box feature on the object's top face at both predicted poses
        let px = |b: [f64; 4]| [b[0] * self.cam.size[0], b[1] * self.cam.size[1], b[2] * self.cam.size[0], b[3] * self.cam.size[1]];
        let predicted = |du: f64| c.frames.iter().map(|d| px(shifted(&d.box_norm, du))).collect::<Vec<_>>();
        if top_face_u(&predicted(shift_norm), &predicted(-shift_norm), self.cam.cx).is_none() { return Ok(Lateral::Fallback("edge")); }
        if [(&pose, &neg), (&neg, &pos), (&pos, &pose)].iter().any(|(a, b)| table_violation(a, b, &self.envelope, s.table_z_m).is_some()) {
            return Ok(Lateral::Fallback("unreachable"));
        }
        {
            let mut input = self.sup.inner().read().await.plan_input().map_err(crate::macro_action::arm_fail)?;
            let speed = Speed { linear: s.approach_speed_mps, angular: motion::DEFAULT_ANGULAR_RPS };
            let is_5dof = input.arm.arm_dof() == 5;
            for goal in [neg, pos] {
                match motion::plan_cartesian_ex(&input, goal, speed, is_5dof) {
                    Ok(path) => { let q = path.last(); input.q0 = std::array::from_fn(|i| f64::from(q[i])); }
                    Err(_) => return Ok(Lateral::Fallback("unreachable")),
                }
            }
        }
        self.phase("probe", 0.45).await?;
        // per pose: boxes (px), measured lens centres, measured camera rotation
        let mut sides: Vec<(Vec<[f64; 4]>, Vec<Vec3>, Mat3)> = Vec::new();
        let mut reason = None;
        let mut moved = false;
        'poses: for goal in [neg, pos] {
            let done = match self.move_arm("probe", GoalSpec::Absolute { p: goal.p, r: Some(goal.r) }, s.approach_speed_mps).await {
                Ok(d) => d,
                // rejected by the planner before anything was installed
                Err(Stop::Fail(code, _)) if code.starts_with("IK_") || code == "WORKSPACE_VIOLATION" => { reason = Some("unreachable"); break; }
                Err(e) => return Err(e),
            };
            if !moved {
                moved = true;
                self.sup.inner().write().await.update_action(self.id, |a| a.probe_mode = Some("lateral".into()));
            }
            let mut tracker = Tracker { label: label.to_string(), predicted: c.last.box_norm, gate_px: s.acquire_dist_px };
            if let Some(p) = self.cam.project(&goal, est) { tracker.predict_at(self.cam.to_norm(p)); }
            let (mut boxes, mut lens, mut rot) = (Vec::new(), Vec::new(), None);
            while boxes.len() < s.median_frames as usize {
                let d = match self.fresh_match(&tracker, done.at, after).await {
                    Ok(d) => d,
                    Err(Stop::Fail(code, _)) if code == "TARGET_LOST" => { reason = Some("target_lost"); break 'poses; }
                    Err(e) => return Err(e),
                };
                if touches_border(&d.box_norm, self.cam.size) { reason = Some("border"); break 'poses; }
                tracker.gate_px = s.match_dist_px;
                tracker.predicted = d.box_norm;
                boxes.push(px(d.box_norm));
                let meas = {
                    let inner = self.sup.inner().read().await;
                    inner.config.arm_model.fk_f32(&inner.measured_positions)
                };
                let meas = meas.unwrap_or(goal);
                lens.push(self.cam.origin(&meas));
                rot.get_or_insert(self.cam.rotation(&meas));
            }
            sides.push((boxes, lens, rot.unwrap_or_else(|| self.cam.rotation(&goal))));
        }
        // back to the start pose: the later steps start from it
        if moved { self.move_arm("probe", GoalSpec::Absolute { p: pose.p, r: Some(pose.r) }, s.approach_speed_mps).await?; }
        if let Some(r) = reason { return Ok(Lateral::Fallback(r)); }
        let mean = |v: &[Vec3]| { let k = v.len().max(1) as f64; v.iter().fold([0.0; 3], |a, p| kinematics::add(&a, p)).map(|x| x / k) };
        let (l_neg, l_pos) = (mean(&sides[0].1), mean(&sides[1].1));
        let x_meas = kinematics::column(&sides[0].2, 0);
        let b_meas = kinematics::dot(&kinematics::sub(&l_pos, &l_neg), &x_meas);
        // commanded: the image-x component of the commanded lens displacement (B · n)
        let b_cmd = kinematics::dot(&kinematics::sub(&self.cam.origin(&pos), &self.cam.origin(&neg)), &x_cam);
        self.sup.inner().write().await.update_action(self.id, |a| {
            a.lateral_baseline_m = Some(b_meas as f32);
            a.lateral_baseline_err_mm = Some(((b_meas - b_cmd) * 1000.0) as f32);
        });
        tracing::info!("Grasp {}: lateral probe baseline {:.4} m measured, {:.4} m commanded", self.id, b_meas, b_cmd);
        let z = (l_neg[2] + l_pos[2]) / 2.0;
        let Some((u_neg, u_pos, feature)) = top_face_u(&sides[0].0, &sides[1].0, self.cam.cx) else { return Ok(Lateral::Fallback("edge")); };
        tracing::info!("Grasp {}: lateral probe parallax of the box {feature}", self.id);
        Ok(Lateral::Done(parallax_distance(&u_neg, &u_pos, self.cam.fx, b_meas).map(|d| z - d - s.table_z_m)))
    }

    /// Look-down mid-height re-centre: descend to `recentre_height_m`, then centre the target
    /// on the grip pixel for that height: the model's projection of the point under the TCP on
    /// the plane z = `plane`, plus the calibration offset (`grip_pixel_norm` minus the model's
    /// projection of that point at the centring pose). Returns the `recentre` status and the
    /// corrections; target lost, table clearance or an unusable aim skip it (the descent goes on).
    async fn recentre(&self, c: &Centred, plane: f64, close_h: f64, label: &str, after: &mut Option<u64>) -> Result<(String, Option<u32>), Stop> {
        let s = &self.setup;
        let skip = |r: &str| Ok((format!("skipped:{r}"), None));
        if s.recentre_height_m <= 0.0 { return skip("disabled"); }
        if !self.lens_known { return skip("no_lens_centre"); }
        let pose = self.pose().await;
        let pose_tcp = self.tcp_pose().await;
        let z = s.table_z_m + s.recentre_height_m;
        if s.recentre_height_m <= close_h + 0.02 || z >= pose_tcp.p[2] - 1e-3 { return skip("height"); }
        let grip_px = self.cam.to_px(s.grip_pixel_norm);
        let tcp_c = motion::end_to_tcp(&c.pose, &self.tcp_offset);
        let Some(m0) = self.cam.project(&c.pose, &[tcp_c.p[0], tcp_c.p[1], plane]) else { return skip("projection"); };
        let Some(t3) = self.cam.plane_point(&c.pose, self.cam.to_px(box_centre(&c.last.box_norm)), plane) else { return skip("projection"); };
        let goal = Pose::new(pose_tcp.r, [pose_tcp.p[0], pose_tcp.p[1], z]);
        let goal_end = motion::tcp_to_end(&goal, &self.tcp_offset);
        if table_violation(&pose, &goal_end, &self.envelope, s.table_z_m).is_some() { return skip("table_clearance"); }
        let done = self.move_arm_in(GoalFrame::Tcp, "descend", GoalSpec::Absolute { p: goal.p, r: None }, s.descend_speed_mps).await?;
        let now = self.pose().await;
        let now_tcp = self.tcp_pose().await;
        let Some(m1) = self.cam.project(&now, &[now_tcp.p[0], now_tcp.p[1], plane]) else { return skip("projection"); };
        let aim = [m1[0] + grip_px[0] - m0[0], m1[1] + grip_px[1] - m0[1]];
        if !(aim[0] >= 0.0 && aim[0] <= self.cam.size[0] && aim[1] >= 0.0 && aim[1] <= self.cam.size[1]) { return skip("aim_outside_image"); }
        self.phase("recentre", 0.7).await?;
        let mut tracker = Tracker { label: label.to_string(), predicted: c.last.box_norm, gate_px: s.acquire_dist_px };
        if let Some(p) = self.cam.project(&now, &t3) { tracker.predict_at(self.cam.to_norm(p)); }
        match self.centre("recentre", 0.7, &mut tracker, aim, "the grip point", plane, done.at, after, s.recentre_max_iterations, false).await {
            Ok(r) => Ok(("done".into(), Some(r.iterations))),
            Err(Stop::Fail(code, _)) if code == "TARGET_LOST" => skip("target_lost"),
            Err(Stop::Fail(code, _)) if code == "WORKSPACE_VIOLATION" => skip("table_clearance"),
            Err(Stop::Fail(code, _)) if code == "CENTRING_FAILED" => Ok(("unconverged".into(), Some(s.recentre_max_iterations))),
            Err(e) => Err(e),
        }
    }

    /// Straight down to `close_h` above the table: the [`Insertion`] along -z that ends at the
    /// close height, run by [`insert`](Self::insert). With the same-side approach
    /// (`grasp_final`) it ends with the short segment, never rising towards the target from below.
    async fn descend(&self, close_h: f64, approach: crate::backlash::Request) -> Result<(), Stop> {
        let s = &self.setup;
        let p = self.tcp_pose().await.p;
        self.insert(Insertion { goal_tcp: [p[0], p[1], s.table_z_m + close_h], speed_mps: s.descend_speed_mps, approach }).await
    }

    /// One guarded insertion (phase `descend`): a TCP-frame path to `goal_tcp` whose final
    /// segment is the same-side approach `grasp_final`, with the table clearance enforced on
    /// the installed path. The control tick runs its contact guard ([`ContactGuard`]): it
    /// records the peak |residual| of J2 / J3 and the peak tracking error, and with
    /// `descend_contact_stop_nm` set a J2 or J3 residual above it ends the insertion at the
    /// measured position (the short segment is then not run). The straight-down descent is the
    /// insertion along -z; another approach direction is another `goal_tcp` from its pre-grasp.
    async fn insert(&self, ins: Insertion) -> Result<(), Stop> {
        let s = &self.setup;
        // the TCP never rises to the object from below; the envelope keeps the table clearance
        let rules = crate::backlash::DescentRules { table: Some((self.envelope.clone(), s.table_z_m + TABLE_CLEARANCE_M)), ..crate::backlash::DescentRules::tcp_at(self.tcp_offset) };
        let final_approach = crate::macro_action::FinalApproach { use_: "grasp_final", request: ins.approach, rules };
        // J2 / J3 bounded by descend_contact_stop_nm (none: peaks only); a contact ends the descent done
        let lim = s.descend_contact_stop_nm.map_or(0.0, |l| l as f32);
        let mut max_residual_nm = [0.0; crate::rt_core::mailbox::MAX_DOF];
        max_residual_nm[1] = lim;
        max_residual_nm[2] = lim;
        let guard = ContactGuard { max_residual_nm, stop_code: None };
        self.sup.inner().write().await.guard_next_macro_child(self.id, Some(guard));
        let result = self.move_arm_final_in(GoalFrame::Tcp, "descend", GoalSpec::Absolute { p: ins.goal_tcp, r: None }, ins.speed_mps, Some(final_approach)).await;
        {
            let mut inner = self.sup.inner().write().await;
            // a guard not taken by a child (planning or install refused) does not stay armed
            inner.guard_next_macro_child(self.id, None);
            let watch = inner.macro_run.as_ref().filter(|m| m.id == self.id && m.child.guard == Some(guard)).and_then(|m| m.ticket())
                .and_then(|t| inner.macro_child_watch(t)).unwrap_or_default();
            let contact_h = watch.contact_joint.map(|_| watch.contact_tcp_z_m.unwrap_or(f64::NAN) - s.table_z_m);
            inner.update_action(self.id, |a| {
                a.descend_peak_residual_nm = watch.peak_residual_nm;
                a.descend_peak_tracking_rad = watch.peak_tracking_rad;
                if let Some(h) = contact_h { a.descend_contact = Some(true); a.descend_contact_height_m = Some(h as f32); }
            });
            if let Some(h) = contact_h {
                tracing::info!("Grasp {}: contact during the descent at {h:.3} m above the table", self.id);
                // a contact stop ends the descent before the short segment
                inner.approach_interrupted(self.id, "contact");
            }
        }
        result.map(|_| ())
    }

    /// J6 roll (about the tool axis) that closes the jaws across the object's short side, and
    /// the `align` status, from the yaw chain ([`yaw_chain`], `yaw_sources` default `mask`
    /// then `box`): the detector angle when the centring frames carry one with
    /// `angle_conf >= align_min_angle_conf`, else ±pi/2 from the box aspect. No turn when the
    /// last box is unreliable ([`align_guard`]) or no candidate fits J6's limits.
    async fn align_turn(&self, c: &Centred, plane: f64) -> (Option<f64>, String) {
        let pose = self.pose().await;
        let inner = self.sup.inner().read().await;
        let roll_idx = inner.config.arm_dof() - 1;
        let (q_roll, lim) = (f64::from(inner.commanded_positions[roll_idx]), inner.config.limits[roll_idx].clone());
        // a roll t about the tool axis moves the roll motor by sign * t
        let sign = inner.config.arm_model.joint_map().get(roll_idx).map_or(1.0, |m| m.sign);
        drop(inner);
        let room = |t: f64| { let q = q_roll + sign * t; q >= f64::from(lim.min_position) + 0.1 && q <= f64::from(lim.max_position) - 0.1 };
        // an image angle: the long side on the plane, seen from the pose of the centring frames
        let angle_turn = |alpha: f64| -> Result<Option<f64>, &'static str> {
            let px = self.cam.to_px(box_centre(&c.last.box_norm));
            let (a, b) = (alpha.to_radians().cos(), alpha.to_radians().sin());
            let (Some(p0), Some(p1)) = (self.cam.plane_point(&c.pose, px, plane), self.cam.plane_point(&c.pose, [px[0] + 20.0 * a, px[1] + 20.0 * b], plane)) else {
                return Err("ray");
            };
            let jaw = kinematics::mat_vec(&pose.r, &[0.0, 1.0, 0.0]);
            let t0 = roll_across_long_side(jaw, kinematics::sub(&p1, &p0));
            let pi = std::f64::consts::PI;
            let Some(t) = [t0, t0 + pi, t0 - pi].into_iter().filter(|t| room(*t)).min_by(|x, y| x.abs().total_cmp(&y.abs())) else {
                return Err("j6_limits");
            };
            Ok((t.abs() >= 1e-3).then_some(t))
        };
        // the box: ±pi/2 when the short side is along the jaw closing direction (end_link y)
        // seen in the image
        let box_turn = |short_along_x: bool| -> Option<f64> {
            let jaw = kinematics::mat_vec(&kinematics::transpose(&self.cam.rec), &[0.0, 1.0, 0.0]);
            let jaw_along_x = jaw[0].abs() >= jaw[1].abs();
            if jaw_along_x == short_along_x { return None; }
            let half = std::f64::consts::FRAC_PI_2;
            // roll about +x_tool; J6 turns with it when its axis is the tool axis
            Some([half, -half].into_iter().find(|t| room(*t)).unwrap_or(half))
        };
        let out = yaw_chain(&self.setup, &c.last, &c.frames, self.cam.size, &angle_turn, &box_turn);
        self.sup.inner().write().await.update_action(self.id, |a| {
            if let Some(spread) = out.angle_spread_deg { a.angle_spread_deg = Some(spread as f32); }
            a.yaw_estimates = out.entries.clone();
        });
        (out.turn, out.align)
    }

    /// Grasp width source `mask`: the median `grasp_width_px` of the centring frames across the
    /// mask angle (when it is confident and consistent; else along the jaw axis), on the plane.
    fn mask_width_m(&self, c: &Centred, plane: f64) -> Result<f64, &'static str> {
        let s = &self.setup;
        let Some(w_px) = frames_width_px(&c.frames, 0.0) else { return Err("no_width") };
        let accepted_angle = frames_angle(&c.frames, s.align_min_angle_conf)
            .filter(|&(_alpha, spread)| s.align_max_angle_spread_deg <= 0.0 || spread <= s.align_max_angle_spread_deg);
        let (u_x, u_y) = if let Some((alpha, _spread)) = accepted_angle {
            let closing_deg = alpha + 90.0;
            (closing_deg.to_radians().cos(), closing_deg.to_radians().sin())
        } else {
            let jaw = kinematics::mat_vec(&kinematics::transpose(&self.cam.rec), &[0.0, 1.0, 0.0]);
            let n = jaw[0].hypot(jaw[1]);
            if n > 1e-6 { (jaw[0] / n, jaw[1] / n) } else { (0.0, 1.0) }
        };
        let centre_px = self.cam.to_px(box_centre(&c.last.box_norm));
        let half_w = w_px / 2.0;
        let p0_px = [centre_px[0] - half_w * u_x, centre_px[1] - half_w * u_y];
        let p1_px = [centre_px[0] + half_w * u_x, centre_px[1] + half_w * u_y];
        match (self.cam.plane_point(&c.pose, p0_px, plane), self.cam.plane_point(&c.pose, p1_px, plane)) {
            (Some(p0), Some(p1)) => Ok(kinematics::norm(&kinematics::sub(&p1, &p0))),
            _ => Err("ray"),
        }
    }

    /// TOO_WIDE limit (m; 0 = off): the calibrated full opening minus `pre_open_margin_m`, at
    /// most `max_grasp_width_m` (when > 0); `max_grasp_width_m` without a calibrated policy.
    async fn max_grasp_width(&self) -> f64 {
        let s = &self.setup;
        let inner = self.sup.inner().read().await;
        match inner.config.gripper_policy.as_ref().filter(|p| p.is_calibrated()) {
            Some(p) => {
                let calib_limit = (p.opening_m(p.open_rad) as f64) - s.pre_open_margin_m;
                if s.max_grasp_width_m > 0.0 { s.max_grasp_width_m.min(calib_limit) } else { calib_limit }
            }
            // an uncalibrated (nominal) curve narrower than the setup clamps it
            None => inner.grasp_width_check(s.max_grasp_width_m)
                .map_or(s.max_grasp_width_m, |c| c.effective_max_m(s.max_grasp_width_m)),
        }
    }
}

/// Result of [`yaw_chain`]: the roll, the `align` status, the `yaw_estimates` entries and the
/// mask angle dispersion (`angle_spread_deg`, written only by a deciding `mask` source).
#[derive(Debug, Clone, PartialEq)]
pub struct YawOutcome {
    pub turn: Option<f64>,
    pub align: String,
    pub entries: Vec<EstimateEntry>,
    pub angle_spread_deg: Option<f64>,
}

/// Mean angle and dispersion of an angle source's frames (`mask`: `angle_deg` / `angle_conf`,
/// `depth`: `depth_angle_deg` / `depth_angle_conf`).
fn yaw_angle(src: YawSource, frames: &[Detection], min_conf: f64) -> Option<(f64, f64)> {
    match src {
        YawSource::Mask => frames_angle(frames, min_conf),
        YawSource::Depth => frames_angle_by(frames, min_conf, |d| (d.depth_angle_deg, d.depth_angle_conf)),
        YawSource::Box => None,
    }
}

/// The J6 yaw estimate chain (`yaw_sources`, `yaw_record`). Record-only sources are evaluated
/// first (value only, no turn). [`align_guard`] then decides for every source (`skipped:<why>`).
/// The deciding sources run in order: `mask` / `depth` without a confident angle pass to the
/// next source; with one, they decide: `skipped:angle_spread` (frames disagree), `skipped:ray`
/// / `skipped:j6_limits` (from `angle_turn`), else `kept:angle` / `turned:angle` (`mask`) or
/// `kept:depth` / `turned:depth` (`depth`). `box` passes on `aspect` (box ratio below
/// `align_min_aspect`), else `kept:box` / `turned:box` (`box_turn(short side along image x)`).
/// When every source passes: `skipped:<reason of the last one>`.
pub fn yaw_chain(
    s: &GraspSetup,
    last: &Detection,
    frames: &[Detection],
    size: [f64; 2],
    angle_turn: &dyn Fn(f64) -> Result<Option<f64>, &'static str>,
    box_turn: &dyn Fn(bool) -> Option<f64>,
) -> YawOutcome {
    let spread_ok = |spread: f64| s.align_max_angle_spread_deg <= 0.0 || spread <= s.align_max_angle_spread_deg;
    let b = last.box_norm;
    let (w, h) = ((b[2] - b[0]) * size[0], (b[3] - b[1]) * size[1]);
    let aspect_ok = w.max(h) / w.min(h).max(1e-6) >= s.align_min_aspect;
    // the box's long side in the image: along x (0 deg) or along y (-90 deg)
    let box_alpha = if w >= h { 0.0 } else { -90.0 };
    let mut chain = EstimateChain::new();
    let mut out = YawOutcome { turn: None, align: String::new(), entries: Vec::new(), angle_spread_deg: None };
    let finish = |chain: EstimateChain, mut out: YawOutcome, turn: Option<f64>, align: String| {
        out.turn = turn;
        out.align = align;
        out.entries = match chain.finish() { Ok((_, e)) | Err((_, e)) => e };
        out
    };
    for src in s.yaw_record.iter().filter(|x| !s.yaw_sources.contains(x)) {
        let r = match src {
            YawSource::Box if aspect_ok => (Some(box_alpha), SourceResult::Accept(box_alpha)),
            YawSource::Box => (Some(box_alpha), SourceResult::Reject("aspect".into())),
            _ => match yaw_angle(*src, frames, s.align_min_angle_conf) {
                Some((alpha, spread)) if spread_ok(spread) => (Some(alpha), SourceResult::Accept(alpha)),
                Some((alpha, _)) => (Some(alpha), SourceResult::Reject("angle_spread".into())),
                None => (None, SourceResult::Reject("no_angle".into())),
            },
        };
        chain.offer(src.name(), r.0, r.1, true);
    }
    if let Err(reason) = align_guard(last, frames, size) {
        return finish(chain, out, None, format!("skipped:{reason}"));
    }
    let mut passed = "no_source";
    for src in &s.yaw_sources {
        if *src == YawSource::Box {
            if !aspect_ok {
                chain.offer("box", Some(box_alpha), SourceResult::Reject("aspect".into()), false);
                passed = "aspect";
                continue;
            }
            chain.offer("box", Some(box_alpha), SourceResult::Accept(box_alpha), false);
            return match box_turn(w < h) {
                None => finish(chain, out, None, "kept:box".into()),
                Some(t) => finish(chain, out, Some(t), "turned:box".into()),
            };
        }
        let Some((alpha, spread)) = yaw_angle(*src, frames, s.align_min_angle_conf) else {
            chain.offer(src.name(), None, SourceResult::Reject("no_angle".into()), false);
            passed = "no_angle";
            continue;
        };
        if *src == YawSource::Mask { out.angle_spread_deg = Some(spread); }
        // a confident angle decides: a failure below ends the alignment without a turn
        if !spread_ok(spread) {
            chain.offer(src.name(), Some(alpha), SourceResult::Reject("angle_spread".into()), false);
            return finish(chain, out, None, "skipped:angle_spread".into());
        }
        let tag = if *src == YawSource::Mask { "angle" } else { "depth" };
        return match angle_turn(alpha) {
            Err(why) => {
                chain.offer(src.name(), Some(alpha), SourceResult::Reject(why.into()), false);
                finish(chain, out, None, format!("skipped:{why}"))
            }
            Ok(turn) => {
                chain.offer(src.name(), Some(alpha), SourceResult::Accept(alpha), false);
                let align = format!("{}:{tag}", if turn.is_some() { "turned" } else { "kept" });
                finish(chain, out, turn, align)
            }
        };
    }
    finish(chain, out, None, format!("skipped:{passed}"))
}

/// Lateral probe outcome: a height estimate (or why it is unreliable), or why the up probe runs instead.
#[derive(Debug, PartialEq)]
pub enum Lateral {
    Done(Result<f64, String>),
    Fallback(&'static str),
}

/// Matched frames of the last centring pose.
pub struct Centred {
    pub last: Detection,
    pub frames: Vec<Detection>,
    /// Commanded pose at which `frames` were taken.
    pub pose: Pose,
    pub iterations: u32,
}

/// Why the last box cannot decide the J6 roll: `border` when an edge is within 2 px of the
/// image border, `occluded` when its area differs by more than 40 % from the median area of
/// `frames` (the jaws or a hand cover part of it).
pub fn align_guard(last: &Detection, frames: &[Detection], size: [f64; 2]) -> Result<(), &'static str> {
    let b = last.box_norm;
    if touches_border(&b, size) { return Err("border"); }
    let area = |b: &[f64; 4]| (b[2] - b[0]) * (b[3] - b[1]);
    let mut areas: Vec<f64> = frames.iter().map(|d| area(&d.box_norm)).collect();
    if !areas.is_empty() {
        let m = median(&mut areas);
        if m > 0.0 && (area(&b) / m - 1.0).abs() > 0.4 { return Err("occluded"); }
    }
    Ok(())
}

/// Largest height uncertainty (m) the probe accepts: box size noise beyond it makes the
/// estimate fall back to `grasp_height_m`.
pub const HEIGHT_PROBE_TOL_M: f64 = 0.01;

/// Shorter upward probe tried when the `height_probe_m` pose is unreachable or outside the workspace.
pub const PROBE_FALLBACK_DZ_M: f64 = 0.06;

/// u (px) of a box feature that lies on the object's top face at both lateral-probe poses,
/// from the boxes [x0, y0, x1, y1] (px) at the −B/2 (`neg`) and +B/2 (`pos`) poses and the
/// principal point `cx`, with the feature's name. Looking down, the box edge on the far side of
/// `cx` is the top face's (a closer point projects farther from the principal point); an edge
/// on the near side of `cx` is the bottom of the far side face. So: the centre when every box
/// straddles `cx` (both edges on the top face), else the left edge when it is left of `cx` in
/// every box, else the right edge when it is right of `cx` in every box; None otherwise. With the
/// centre of a box seen off-centre the parallax measures a depth between top and bottom (MuJoCo
/// `axis_cube_lateral`, 0.04 m cube at u 0.3: 0.023 m from the centre).
pub fn top_face_u(neg: &[[f64; 4]], pos: &[[f64; 4]], cx: f64) -> Option<(Vec<f64>, Vec<f64>, &'static str)> {
    let all = || neg.iter().chain(pos);
    let pick = |f: fn(&[f64; 4]) -> f64| (neg.iter().map(f).collect(), pos.iter().map(f).collect());
    if all().all(|b| b[0] <= cx && b[2] >= cx) {
        let (n, p) = pick(|b| (b[0] + b[2]) / 2.0);
        return Some((n, p, "centre"));
    }
    if all().all(|b| b[0] <= cx) { let (n, p) = pick(|b| b[0]); return Some((n, p, "left edge")); }
    if all().all(|b| b[2] >= cx) { let (n, p) = pick(|b| b[2]); return Some((n, p, "right edge")); }
    None
}

/// Smallest box-centre shift (px) between the two lateral-probe poses the parallax estimate accepts.
pub const LATERAL_MIN_SHIFT_PX: f64 = 5.0;

/// Lens-to-top distance from the box-centre u (px) at the −B/2 pose (`u_neg`) and the +B/2
/// pose (`u_pos`) of the lateral probe ([`top_face_u`]) and the measured baseline along the image x axis
/// `baseline_m`: D = fx · |B| / |du|, du = median(u_pos) − median(u_neg). Err (fallback reason)
/// when |B| < 1 mm, |du| < [`LATERAL_MIN_SHIFT_PX`], or the frames' centre spread (half range
/// e of each set) makes D uncertain by more than [`HEIGHT_PROBE_TOL_M`]:
/// dD = D / |du| · √(e_neg² + e_pos²).
pub fn parallax_distance(u_neg: &[f64], u_pos: &[f64], fx: f64, baseline_m: f64) -> Result<f64, String> {
    let stats = |v: &[f64]| -> Option<(f64, f64)> {
        if v.is_empty() { return None; }
        let (lo, hi) = v.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), x| (a.min(*x), b.max(*x)));
        Some((median(&mut v.to_vec()), (hi - lo) / 2.0))
    };
    let (Some((m_neg, e1)), Some((m_pos, e2))) = (stats(u_neg), stats(u_pos)) else { return Err("no_frames".into()); };
    let b = baseline_m.abs();
    if b.is_nan() || b < 1e-3 { return Err(format!("parallax_baseline:{baseline_m:.4}")); }
    let du = (m_pos - m_neg).abs();
    if du.is_nan() || du < LATERAL_MIN_SHIFT_PX { return Err(format!("parallax_shift:{du:.1}")); }
    let d = fx * b / du;
    let unc = d / du * e1.hypot(e2);
    if unc.is_nan() || unc > HEIGHT_PROBE_TOL_M { return Err(format!("parallax_noise:{unc:.3}")); }
    Ok(d)
}

/// Lens-to-top distance at the low pose from box sizes (px) there (`low`) and after moving
/// `dz` farther away (`high`): with r = s_low / s_high (medians), D_low = dz · s_high /
/// (s_low − s_high) = dz / (r − 1). Err (fallback reason) when r < 1.03 or when the frames'
/// size spread (half range / median of each set) makes D_low uncertain by more than
/// [`HEIGHT_PROBE_TOL_M`]: dD = dz / (r − 1)² · dr, dr = r · √(e_low² + e_high²).
pub fn scale_height(low: &[f64], high: &[f64], dz: f64) -> Result<f64, String> {
    let stats = |v: &[f64]| -> Option<(f64, f64)> {
        if v.is_empty() { return None; }
        let m = median(&mut v.to_vec());
        let (lo, hi) = v.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), x| (a.min(*x), b.max(*x)));
        (m > 0.0).then(|| (m, (hi - lo) / 2.0 / m))
    };
    let (Some((s_low, e1)), Some((s_high, e2))) = (stats(low), stats(high)) else { return Err("no_frames".into()); };
    let r = s_low / s_high;
    if r.is_nan() || r < 1.03 { return Err(format!("scale_change:{r:.3}")); }
    let unc = dz / (r - 1.0).powi(2) * r * e1.hypot(e2);
    if unc.is_nan() || unc > HEIGHT_PROBE_TOL_M { return Err(format!("scale_noise:{unc:.3}")); }
    Ok(dz / (r - 1.0))
}

/// Attempts at a border-free aim ([`border_free_aim`]) after the centring on the grip pixel.
pub const BORDER_SHIFTS: u32 = 3;

/// Aim pixel at which a target whose box, centred on `aim` (px), is cut by the image border
/// would be seen whole: on each axis where the median box edge of `frames` is within 2 px of
/// one border, the estimated full extent (the larger of the visible extent and the box's
/// extent along the other axis when that one is uncut, times 1 + 0.25 · (`attempt` + 1)) is
/// placed `margin` px inside that border; other axes keep `aim`. None when no axis is cut, an
/// axis is cut on both sides, or the estimate does not fit in the image.
pub fn border_free_aim(frames: &[Detection], aim: [f64; 2], size: [f64; 2], margin: f64, attempt: u32) -> Option<[f64; 2]> {
    if frames.is_empty() { return None; }
    let edge = |k: usize| median(&mut frames.iter().map(|d| d.box_norm[k] * size[k % 2]).collect::<Vec<_>>());
    let (lo, hi) = ([edge(0), edge(1)], [edge(2), edge(3)]);
    let cut = |k: usize| (lo[k] <= 2.0, hi[k] >= size[k] - 2.0);
    let mut out = aim;
    let mut any = false;
    for k in 0..2 {
        let (low, high) = cut(k);
        if !low && !high { continue; }
        if low && high { return None; }
        let o = 1 - k;
        let other = if cut(o) == (false, false) { hi[o] - lo[o] } else { 0.0 };
        let est = (hi[k] - lo[k]).max(other) * (1.0 + 0.25 * f64::from(attempt + 1));
        if est + 2.0 * margin > size[k] { return None; }
        out[k] = if high { size[k] - margin - est / 2.0 } else { margin + est / 2.0 };
        any = true;
    }
    any.then_some(out)
}

/// A box edge within 2 px of the image border (the box may be cut off).
pub fn touches_border(b: &[f64; 4], size: [f64; 2]) -> bool {
    let [w, h] = size;
    b[0] * w <= 2.0 || b[1] * h <= 2.0 || b[2] * w >= w - 2.0 || b[3] * h >= h - 2.0
}

/// Mean long-side angle (deg, [-90, 90)) and dispersion (deg) of the frames with
/// `angle_conf >= min_conf`, averaged on doubled angles (an axis, not a direction); None without such frames.
pub fn frames_angle(frames: &[Detection], min_conf: f64) -> Option<(f64, f64)> {
    frames_angle_by(frames, min_conf, |d| (d.angle_deg, d.angle_conf))
}

/// [`frames_angle`] over the (angle, confidence) pair `pick` returns per frame.
pub fn frames_angle_by(frames: &[Detection], min_conf: f64, pick: impl Fn(&Detection) -> (Option<f64>, Option<f64>)) -> Option<(f64, f64)> {
    let (mut c, mut s, mut n) = (0.0, 0.0, 0);
    for d in frames {
        if let (Some(a), Some(conf)) = pick(d) {
            if conf >= min_conf { c += (2.0 * a.to_radians()).cos(); s += (2.0 * a.to_radians()).sin(); n += 1; }
        }
    }
    if n == 0 || c.hypot(s) < 1e-9 { return None; }
    let r = (c.hypot(s) / n as f64).clamp(1e-12, 1.0);
    let spread = if n <= 1 { 0.0 } else { ((-2.0 * r.ln()).sqrt() / 2.0).to_degrees() };
    let a = (s.atan2(c) / 2.0).to_degrees();
    let mean = (a + 90.0).rem_euclid(180.0) - 90.0;
    Some((mean, spread))
}

/// Median grasp width (px) of the frames with `angle_conf >= min_conf` and a positive, finite
/// `grasp_width_px`; None without such frames.
pub fn frames_width_px(frames: &[Detection], min_conf: f64) -> Option<f64> {
    let mut widths: Vec<f64> = frames
        .iter()
        .filter(|d| d.angle_conf.unwrap_or(0.0) >= min_conf)
        .filter_map(|d| d.grasp_width_px)
        .filter(|w| *w > 0.0 && w.is_finite())
        .collect();
    if widths.is_empty() {
        None
    } else {
        Some(median(&mut widths))
    }
}

/// Roll about the (vertical, down) tool axis, in [-pi/2, pi/2), that turns the jaw closing
/// direction `jaw` (base frame) perpendicular to the long side `long` (base frame, horizontal
/// parts used). Rolling by t about the downward tool axis turns `jaw` by -t about base z.
pub fn roll_across_long_side(jaw: Vec3, long: Vec3) -> f64 {
    let pi = std::f64::consts::PI;
    let t = jaw[1].atan2(jaw[0]) - long[1].atan2(long[0]) - pi / 2.0;
    (t + pi / 2.0).rem_euclid(pi) - pi / 2.0
}

async fn run(sup: Arc<ArmSupervisor>, source: Arc<dyn DetectionSource>, id: u64, mut planned: PlannedGrasp, stamp: crate::supervisor::AdmissionStamp) {
    let setup = planned.request.setup.clone();
    let cam = CamModel::assumed(&planned.input.arm, &setup, &planned.input.cam);
    let deadline = Instant::now() + Duration::from_secs_f64(setup.timeout_s);
    // one mode reading per grasp: the preflight's, else read here
    let mode = match planned.mode.take() {
        Some(m) => m,
        None => source.mode_info().await,
    };
    let per_request = mode.as_ref().map(|m| m.per_request_pipelines).map_err(Clone::clone);
    let triggered = per_request == Ok(true);
    let depth_available = mode.as_ref().is_ok_and(|m| m.depth_available);
    let generation = mode.as_ref().ok().and_then(|m| m.models.as_ref()).map(|pm| pm.generation);
    let vision = mode.map_or(Ok(None), |m| m.vision);
    let ctx = MacroCtx {
        sup: sup.clone(), id, stamp, deadline, phase_timeout_s: setup.phase_timeout_s, timeout_code: "GRASP_TIMEOUT",
        timeout_message: format!("Grasp did not finish within {:.0} s", setup.timeout_s),
    };
    let contact_end_m = sup.inner().read().await.config.effective_physical_contact_end_m();
    let envelope = envelope_with_contact(planned.input.ws.as_ref(), &planned.input.cam, &cam.rec, contact_end_m);
    let lens_known = planned.input.cam.offset_m.is_some();
    let orch = Orchestrator { ctx, source: source.clone(), setup, cam, envelope, tcp_offset: planned.input.tcp_offset, triggered, lens_known, depth_available, generation };
    let result = if let Err(e) = per_request {
        Err(fail("DETECTION_UNAVAILABLE", format!("Cannot reach the vision provider: {e}")))
    } else if triggered {
        // perception API 2: a ["yolo"] trigger per centring step, no mode change to restore
        orch.run(&planned).await
    } else { match vision {
        // perception API 1 (rebot-vision 1.2): YOLO (both when ArUco was running) with inference
        // on for the duration, then the previous mode
        Ok(Some(prev)) => {
            let want = VisionMode { mode: grasp_mode(&prev.mode).to_string(), enabled: true };
            if want == prev {
                orch.run(&planned).await
            } else {
                match source.set_vision_mode(&want).await {
                    Ok(()) => {
                        let r = orch.run(&planned).await;
                        if let Err(e) = source.set_vision_mode(&prev).await {
                            tracing::warn!("Grasp could not restore vision mode {} (enabled {}): {e}", prev.mode, prev.enabled);
                        }
                        r
                    }
                    Err(e) => Err(fail("DETECTION_UNAVAILABLE", format!("Cannot switch the camera to {}: {e}", want.mode))),
                }
            }
        }
        // older detectors: inference on for the duration
        Ok(None) => {
            let enabled_by_us = match source.inference_enabled().await {
                Ok(true) => Ok(false),
                Ok(false) => source.set_inference(true).await.map(|_| true),
                Err(e) => Err(e),
            };
            let r = match &enabled_by_us {
                Ok(_) => orch.run(&planned).await,
                Err(e) => Err(fail("DETECTION_UNAVAILABLE", format!("Cannot read the detection state: {e}"))),
            };
            if enabled_by_us == Ok(true) {
                if let Err(e) = source.set_inference(false).await { tracing::warn!("Grasp could not restore detection off: {e}"); }
            }
            r
        }
        Err(e) => Err(fail("DETECTION_UNAVAILABLE", format!("Cannot read the vision mode: {e}"))),
    } };
    let _ = source.set_mask_focus(None, [0.0; 4], 0).await;
    let mut inner = sup.inner().write().await;
    match result {
        Ok((holding, gap)) => {
            // a grasp stopped meanwhile must not overwrite newer held state
            if holding && inner.macro_alive(id) {
                let spill_safe = planned.request.setup.spill_safe_labels.iter().any(|s| s.eq_ignore_ascii_case(&planned.request.label));
                inner.held = Some(crate::supervisor::HeldObject {
                    label: planned.request.label.clone(),
                    spill_safe,
                });
            }
            inner.finish_macro(id, "done", None, None, Some(holding), gap);
        }
        Err(Stop::Fail(code, message)) => {
            let holding = if code == "NOT_HOLDING" || code == "DROPPED" { Some(false) } else { None };
            inner.finish_macro(id, "failed", Some(&code), Some(message), holding, None);
        }
        Err(Stop::Cancelled) => inner.finish_macro(id, "cancelled", None, None, None, None),
    }
    if let Some(a) = inner.last_action.as_ref().filter(|a| a.id == id) {
        let code = a.code.as_deref().unwrap_or(if a.state == "done" { "OK" } else { &a.state });
        let worst = a.motions.iter().max_by(|x, y| x.track_mm.total_cmp(&y.track_mm));
        tracing::info!("Grasp {id} {code}: phase_ms {:?}, worst motion {:?}", a.phase_ms, worst);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tasks_setup(tasks: Value) -> Result<GraspSetup, String> { GraspSetup::default().patched(&serde_json::json!({"tasks": tasks})) }

    #[test]
    fn task_registry_validation() {
        let ok = tasks_setup(serde_json::json!({"cup-2_a": {"description": "d", "requires": {"model_kind": "segment", "depth": "any"},
            "overrides": {"yaw_sources": ["mask"]}}, "x": {}})).unwrap();
        assert_eq!(ok.tasks["cup-2_a"].requires, TaskRequires { model_kind: Some(ModelKind::Segment), depth: Some(DepthReq::Any) });
        assert!(ok.tasks["x"].requires.is_empty() && ok.tasks["x"].overrides.is_empty());
        let obb = tasks_setup(serde_json::json!({"part": {"requires": {"model_kind": "obb"}}})).unwrap();
        assert_eq!(obb.tasks["part"].requires.model_kind, Some(ModelKind::Obb));
        assert_eq!(serde_json::to_value(&obb.tasks["part"]).unwrap()["requires"], serde_json::json!({"model_kind": "obb"}));
        let long = "a".repeat(33);
        for (tasks, msg) in [
            (serde_json::json!({"Cup": {}}), "name"),
            (serde_json::json!({"": {}}), "name"),
            (serde_json::json!({long: {}}), "name"),
            (serde_json::json!({"a b": {}}), "name"),
            (serde_json::json!({"a": {"description": "x".repeat(201)}}), "description"),
            (serde_json::json!({"a": {"overrides": {"tasks": {}}}}), "overrides cannot contain tasks"),
            (serde_json::json!({"a": {"overrides": {"bogus": 1}}}), "unknown grasp setup field: bogus"),
            (serde_json::json!({"a": {"overrides": {"grasp_height_m": 5.0}}}), "grasp_height_m"),
            (serde_json::json!({"a": {"overrides": {"yaw_sources": ["mask", "mask"]}}}), "repeat"),
            (serde_json::json!({"a": {"requires": {"model_kind": "pose"}}}), "invalid grasp setup"),
            (serde_json::json!({"a": {"requires": {"depth": "stereo"}}}), "invalid grasp setup"),
            (serde_json::json!({"a": {"requires": {"gpu": true}}}), "invalid grasp setup"),
            (serde_json::json!({"a": {"model": "x"}}), "invalid grasp setup"),
        ] {
            let e = tasks_setup(tasks.clone()).unwrap_err();
            assert!(e.contains(msg), "{tasks}: {e}");
        }
        // a task is checked against the setup it is stored with; a base change that breaks it is rejected
        let s = tasks_setup(serde_json::json!({"low": {"overrides": {"max_grasp_height_m": 0.05}}})).unwrap();
        assert!(GraspSetup::default().patched(&serde_json::json!({"grasp_height_m": 0.055})).is_ok());
        let e = s.patched(&serde_json::json!({"grasp_height_m": 0.055})).unwrap_err();
        assert!(e.starts_with("grasp task low: max_grasp_height_m"), "{e}");
        // replacing the map in the same patch fixes it; `{}` deletes all tasks
        assert!(s.patched(&serde_json::json!({"grasp_height_m": 0.055, "tasks": {}})).unwrap().tasks.is_empty());
        // tasks always serialise (first insertion through a patch of the default)
        assert_eq!(serde_json::to_value(GraspSetup::default()).unwrap()["tasks"], serde_json::json!({}));
        assert_eq!(serde_json::to_value(&s).unwrap()["tasks"], serde_json::json!({"low": {"overrides": {"max_grasp_height_m": 0.05}}}));
    }

    #[test]
    fn grasp_request_applies_base_then_task_then_overrides() {
        let base = tasks_setup(serde_json::json!({
            "t": {"requires": {"depth": "metric"}, "overrides": {"lift_height_m": 0.13, "approach_height_m": 0.2}},
            "low": {"overrides": {"max_grasp_height_m": 0.05}},
        })).unwrap();
        let body = |extra: Value| {
            let mut b = serde_json::json!({"label": "cup", "box_norm": [0.4, 0.4, 0.6, 0.6]});
            for (k, v) in extra.as_object().unwrap() { b[k] = v.clone(); }
            b
        };
        let r = parse_grasp(&body(serde_json::json!({"task": "t", "overrides": {"approach_height_m": 0.18}})), &base).unwrap();
        assert_eq!((r.setup.lift_height_m, r.setup.approach_height_m, r.setup.table_z_m), (0.13, 0.18, base.table_z_m), "request override wins");
        assert_eq!((r.task.as_deref(), r.requires.depth), (Some("t"), Some(DepthReq::Metric)));
        assert!(r.setup.tasks.is_empty(), "the effective setup carries no tasks");
        let r = parse_grasp(&body(serde_json::json!({})), &base).unwrap();
        assert_eq!((r.task, r.requires, r.setup), (None, TaskRequires::default(), base.without_tasks()));
        let err = |extra: Value| match parse_grasp(&body(extra), &base) { Err(MotionError::InvalidCommand(m)) => m, other => panic!("{other:?}") };
        assert_eq!(err(serde_json::json!({"task": "nope"})), "unknown grasp task: nope");
        assert_eq!(err(serde_json::json!({"task": ["t"]})), "task must be a string");
        assert_eq!(err(serde_json::json!({"overrides": {"tasks": {}}})), "tasks cannot be overridden per request");
        // only the final effective setup is validated: the request's grasp height against the task's max
        assert!(err(serde_json::json!({"task": "low", "overrides": {"grasp_height_m": 0.055}})).contains("max_grasp_height_m"));
        assert!(parse_grasp(&body(serde_json::json!({"overrides": {"grasp_height_m": 0.055}})), &base).is_ok(), "no task: the tasks do not constrain overrides");
    }

    #[test]
    fn mode_info_parse_and_task_requirement_check() {
        let legacy = ModeInfo::parse(None);
        assert!(!legacy.reported && legacy.vision == Ok(None));
        let full = ModeInfo::parse(Some(&serde_json::json!({"mode": "yolo", "enabled": true, "per_request_pipelines": true,
            "model": {"kind": "segment"}, "available": {"depth": true}, "depth_model": {"metric": true}})));
        assert_eq!(full, ModeInfo { reported: true, per_request_pipelines: true, yolo_available: true, aruco_available: false, depth_available: true, gray_available: false, gray_format: Ok(None), upstream_loopback: true, depth_metric: true,
            model_kind: Some("segment".into()), vision: Ok(Some(VisionMode { mode: "yolo".into(), enabled: true })), models: None });
        // available.depth is read independently of depth_model.metric
        let depth_off = ModeInfo::parse(Some(&serde_json::json!({"available": {"depth": false}, "depth_model": {"metric": true}})));
        assert!(!depth_off.depth_available && depth_off.depth_metric && depth_off.vision.is_err());
        let relative = ModeInfo { depth_metric: false, model_kind: Some("detect".into()), ..full.clone() };
        let req = |k: Option<ModelKind>, d: Option<DepthReq>| TaskRequires { model_kind: k, depth: d };
        assert_eq!(req(None, None).check("t", &legacy), Ok(()));
        assert_eq!(req(Some(ModelKind::Segment), Some(DepthReq::Metric)).check("t", &full), Ok(()));
        assert_eq!(req(Some(ModelKind::Segment), None).check("t", &relative).unwrap_err(), "task t needs model_kind segment, provider has detect");
        assert_eq!(req(Some(ModelKind::Detect), None).check("t", &relative), Ok(()));
        assert_eq!(req(None, Some(DepthReq::Any)).check("t", &relative), Ok(()));
        assert_eq!(req(None, Some(DepthReq::Metric)).check("t", &relative).unwrap_err(), "task t needs depth metric, provider depth model is not metric");
        assert_eq!(req(None, Some(DepthReq::Any)).check("t", &depth_off).unwrap_err(), "task t needs depth any, provider has no depth pipeline");
        assert_eq!(req(Some(ModelKind::Detect), None).check("t", &ModeInfo { model_kind: None, ..full.clone() }).unwrap_err(),
            "task t needs model_kind detect, provider does not report model.kind");
        assert_eq!(req(Some(ModelKind::Detect), None).check("t", &legacy).unwrap_err(), "task t needs model_kind detect, provider does not report model.kind");
        assert_eq!(req(None, Some(DepthReq::Any)).check("t", &legacy).unwrap_err(), "task t needs depth any, provider does not report depth");
        // detector slot empty: an older provider still reports model.kind "detect", a newer one null
        let mode = |kind: Value| ModeInfo::parse(Some(&serde_json::json!({"mode": "aruco", "per_request_pipelines": true,
            "available": {"yolo": false, "aruco": true, "depth": true}, "model": {"name": "", "kind": kind}})));
        for empty in [mode(serde_json::json!("detect")), mode(Value::Null)] {
            assert!(!empty.yolo_available);
            assert_eq!(req(Some(ModelKind::Detect), None).check("t", &empty).unwrap_err(), "task t needs model_kind detect, provider has no detector loaded");
            assert_eq!(req(None, Some(DepthReq::Any)).check("t", &empty), Ok(()), "depth alone needs no detector");
        }
        assert!(full.yolo_available, "available.yolo not reported: a detector is assumed");
        // obb: an oriented box model satisfies obb only; segment stays the segmentation model
        let obb = ModeInfo { model_kind: Some("obb".into()), ..full.clone() };
        assert_eq!(req(Some(ModelKind::Obb), None).check("t", &obb), Ok(()));
        assert_eq!(req(Some(ModelKind::Segment), None).check("t", &obb).unwrap_err(), "task t needs model_kind segment, provider has obb");
        assert_eq!(req(Some(ModelKind::Obb), None).check("t", &full).unwrap_err(), "task t needs model_kind obb, provider has segment");
    }

    #[test]
    fn task_models_are_tri_state_per_slot() {
        let s = tasks_setup(serde_json::json!({
            "a": {"models": {"detector": "seg"}}, "b": {"models": {"depth": null}}, "c": {"models": {"detector": null, "depth": "zd"}}})).unwrap();
        let m = |t: &str| s.tasks[t].models.clone().unwrap();
        assert_eq!(m("a"), TaskModels { detector: Some(Some("seg".into())), depth: None });
        assert_eq!(m("b"), TaskModels { detector: None, depth: Some(None) });
        assert_eq!(m("c"), TaskModels { detector: Some(None), depth: Some(Some("zd".into())) });
        // round trip: absent stays absent, null stays null
        assert_eq!(serde_json::to_value(&s.tasks["b"]).unwrap(), serde_json::json!({"overrides": {}, "models": {"depth": null}}));
        for bad in [serde_json::json!({"models": {}}), serde_json::json!({"models": {"detector": ""}}),
                    serde_json::json!({"models": {"detector": "x".repeat(65)}}), serde_json::json!({"models": {"gpu": "x"}}),
                    serde_json::json!({"models": {"detector": 3}})] {
            assert!(tasks_setup(serde_json::json!({"a": bad.clone()})).is_err(), "{bad}");
        }
        // the switch body names every slot of the task when the resident set differs
        let mode = |det: Option<&str>, depth: Option<&str>| ModeInfo::parse(Some(&serde_json::json!({"mode": "yolo",
            "models": {"generation": 3, "swap": {"state": "idle", "generation": 3}, "resident": {"detector": det, "depth": depth},
                       "catalogue": [{"name": "seg", "kind": "segment", "ion_mb": 19.5}, {"name": "zd", "kind": "depth", "ion_mb": 20.5},
                                     {"name": "det", "kind": "detect"}]},
            "ion": {"base_mb": 21.5, "used_mb": 41.0, "total_mb": 62.9, "reserve_mb": 8.0}})));
        assert_eq!(m("a").switch_body(&mode(Some("seg"), Some("zd"))), None);
        assert_eq!(m("a").switch_body(&mode(Some("det"), None)), Some(serde_json::json!({"detector": "seg"})));
        assert_eq!(m("c").switch_body(&mode(Some("det"), None)), Some(serde_json::json!({"detector": null, "depth": "zd"})));
        assert_eq!(m("b").switch_body(&mode(Some("det"), None)), None);
        assert!(m("b").switch_body(&ModeInfo::parse(Some(&serde_json::json!({"mode": "yolo"})))).is_some(), "no models: always switch");
        // provider state and the catalogue budget
        let pm = mode(Some("det"), None).models.unwrap();
        assert_eq!((pm.generation, pm.swap_state.as_str(), pm.ion), (3, "idle", Some([21.5, 62.9, 8.0])));
        assert_eq!(pm.catalogue.get("det"), Some(&None));
        let set = |d: &str, z: Option<&str>| ResidentModels { detector: Some(d.into()), depth: z.map(str::to_string) };
        assert_eq!(pm.budget(&set("seg", None)), Some(Ok(())));
        assert_eq!(pm.budget(&set("seg", Some("zd"))), Some(Err("ION_BUDGET (need 69.5 MB, limit 62.9 MB)".into())));
        assert_eq!(pm.budget(&set("det", Some("zd"))), None, "det has no ion_mb");
        let req = TaskRequires { model_kind: Some(ModelKind::Segment), depth: None };
        assert_eq!(task_verdict("a", &req, Some(&m("a")), &mode(Some("det"), Some("zd"))),
                   (false, Some("task a: model switch refused: ION_BUDGET (need 69.5 MB, limit 62.9 MB)".into())));
        assert_eq!(task_verdict("a", &req, Some(&m("a")), &mode(Some("det"), None)), (true, Some("needs model switch".into())));
        assert!(!task_verdict("a", &req, Some(&m("a")), &mode(Some("seg"), None)).0, "model.kind not reported");
    }

    #[test]
    fn patch_rejects_unknown_and_invalid_fields() {
        let s = GraspSetup::default();
        assert!(s.validate().is_ok());
        assert!(s.patched(&serde_json::json!({"nope": 1})).is_err());
        assert!(s.patched(&serde_json::json!({"grasp_height_m": 0.5})).is_err());
        let p = s.patched(&serde_json::json!({"table_z_m": 0.02, "grip_pixel_norm": [0.4, 0.7]})).unwrap();
        assert_eq!((p.table_z_m, p.grip_pixel_norm), (0.02, [0.4, 0.7]));
    }

    #[test]
    fn height_source_settings_default_validate_and_patch() {
        let d = GraspSetup::default();
        assert_eq!((d.height_sources.as_slice(), d.height_record.as_slice()), ([HeightSource::Probe].as_slice(), [HeightSource::Depth].as_slice()));
        assert!(d.height_priors.is_empty());
        // a setup file without the keys gets the defaults
        let old: GraspSetup = serde_json::from_value(serde_json::json!({"table_z_m": 0.01})).unwrap();
        assert_eq!((old.height_sources, old.height_record, old.height_priors), (d.height_sources.clone(), d.height_record.clone(), BTreeMap::new()));
        let p = d.patched(&serde_json::json!({"height_sources": ["depth", "probe", "prior"], "height_record": ["prior"], "height_priors": {"cup": 0.09}})).unwrap();
        assert_eq!(p.height_sources, [HeightSource::Depth, HeightSource::Probe, HeightSource::Prior]);
        assert_eq!(p.height_priors.get("cup"), Some(&0.09));
        // the default height_record may name a deciding source: it decides instead
        assert!(d.patched(&serde_json::json!({"height_sources": ["depth", "probe"]})).is_ok());
        for bad in [
            serde_json::json!({"height_sources": ["probe", "probe"]}),
            serde_json::json!({"height_record": ["depth", "depth"]}),
            serde_json::json!({"height_record": ["probe"]}),
            serde_json::json!({"height_sources": ["scale"]}),
            serde_json::json!({"height_priors": {"cup": 0.0}}),
            serde_json::json!({"height_priors": {"cup": 0.6}}),
            serde_json::json!({"height_priors": {"cup": 0.05, "Cup": 0.06}}),
        ] {
            assert!(d.patched(&bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn parses_obb_v2_detections() {
        let raw = serde_json::json!([{
            "class_id": 39,
            "format": "xywh_center",
            "class_name": "bottle",
            "confidence": 0.88,
            "x": 0.45,
            "y": 0.55,
            "w": 0.12,
            "h": 0.24,
            "obb_corners": [
                [0.40, 0.45],
                [0.50, 0.45],
                [0.50, 0.65],
                [0.40, 0.65]
            ],
            "angle_deg": -25.5,
            "angle_conf": 0.85,
            "grasp_width_px": 28.5,
            "length_px": 72.0
        }]);
        let d = parse_detections(&raw, [640.0, 360.0]);
        assert_eq!(d.len(), 1);
        let det = &d[0];
        assert_eq!(det.label, "bottle");
        assert!((det.score - 0.88).abs() < 1e-6);
        assert!((det.box_norm[0] - 0.39).abs() < 1e-6);
        assert!((det.box_norm[1] - 0.43).abs() < 1e-6);
        assert!((det.box_norm[2] - 0.51).abs() < 1e-6);
        assert!((det.box_norm[3] - 0.67).abs() < 1e-6);
        assert_eq!(det.angle_deg, Some(-25.5));
        assert_eq!(det.angle_conf, Some(0.85));
        assert_eq!(det.grasp_width_px, Some(28.5));
        assert_eq!(det.length_px, Some(72.0));
    }

    #[test]
    fn parses_detector_formats() {
        let raw = serde_json::json!([
            [320.0, 180.0, 64.0, 36.0, 0.9, "cup"],
            {"label": "bottle", "score": 0.8, "format": "xyxy", "bbox": [0.1, 0.2, 0.3, 0.4]},
            {"label": "x", "score": 0.5, "bbox": [1, 2, 3, 4]},
            {"label": "mouse", "score": 0.7, "box_norm": [0.5, 0.5, 0.6, 0.7]},
            {"class_id": 39, "class_name": "bottle", "confidence": 0.71, "x": 0.42, "y": 0.55, "w": 0.08, "h": 0.2}
        ]);
        let d = parse_detections(&raw, [640.0, 360.0]);
        assert_eq!(d.len(), 4, "{d:?}");
        assert_eq!(d[3].label, "bottle", "rebot-vision 1.2 detections carry no format");
        assert!(d[3].box_norm.iter().zip([0.38, 0.45, 0.46, 0.65]).all(|(a, b)| (a - b).abs() < 1e-9), "{:?}", d[3].box_norm);
        assert_eq!(d[0].label, "cup");
        assert!((d[0].box_norm[0] - 0.45).abs() < 1e-9 && (d[0].box_norm[3] - 0.55).abs() < 1e-9);
        assert!(d[1].box_norm.iter().zip([0.1, 0.2, 0.3, 0.4]).all(|(a, b)| (a - b).abs() < 1e-9));
        assert_eq!(d[2].label, "mouse");
    }

    fn det(b: [f64; 4], angle: Option<(f64, f64)>) -> Detection {
        Detection { label: "cup".into(), score: 0.9, raw_score_valid: true, box_norm: b, angle_deg: angle.map(|a| a.0), angle_conf: angle.map(|a| a.1), grasp_width_px: None, mask_focus: false, disp_fg: None, disp_bg: None, depth_conf: None, length_px: None, jaw_clear: None, depth_angle_deg: None, depth_angle_conf: None, depth_width_px: None, contour_norm: None }
    }

    #[test]
    fn test_box_changed_and_iou() {
        let size = [640.0, 360.0];
        let b0 = [0.1, 0.1, 0.3, 0.3];
        // small shift <= 5 px (4 px in x: 4/640 = 0.00625)
        let b_small = [0.1 + 4.0 / 640.0, 0.1, 0.3 + 4.0 / 640.0, 0.3];
        assert!(!box_changed_px(&b0, &b_small, size));
        // large shift > 5 px (6 px in x: 6/640 = 0.009375)
        let b_large = [0.1 + 6.0 / 640.0, 0.1, 0.3 + 6.0 / 640.0, 0.3];
        assert!(box_changed_px(&b0, &b_large, size));

        // IoU matching unit test
        let b1 = [0.0, 0.0, 1.0, 1.0];
        let b2 = [0.5, 0.0, 1.5, 1.0];
        // intersection: 0.5 * 1.0 = 0.5; union: 1.0 + 1.0 - 0.5 = 1.5 -> iou = 1/3
        assert!((iou(&b1, &b2) - 1.0 / 3.0).abs() < 1e-6);
        let b_disjoint = [2.0, 2.0, 3.0, 3.0];
        assert_eq!(iou(&b1, &b_disjoint), 0.0);
    }

    #[test]
    fn align_guard_skips_border_and_occluded_boxes() {
        let size = [640.0, 360.0];
        let b = [0.3, 0.3, 0.5, 0.6];
        let frames = vec![det(b, None), det([0.31, 0.3, 0.51, 0.6], None), det(b, None)];
        assert_eq!(align_guard(&det(b, None), &frames, size), Ok(()));
        // an edge within 2 px of the border (left, bottom)
        assert_eq!(align_guard(&det([0.002, 0.3, 0.2, 0.6], None), &frames, size), Err("border"));
        assert_eq!(align_guard(&det([0.3, 0.3, 0.5, 0.995], None), &frames, size), Err("border"));
        // the last box shrank to half its height (jaws in front of it): area -50 %
        let shrunk = det([0.3, 0.3, 0.5, 0.45], None);
        let mut with_last = frames.clone();
        with_last.push(shrunk.clone());
        assert_eq!(align_guard(&shrunk, &with_last, size), Err("occluded"));
        // -30 % passes
        assert_eq!(align_guard(&det([0.3, 0.3, 0.5, 0.51], None), &frames, size), Ok(()));
    }

    #[test]
    fn border_free_aim_moves_a_cut_axis_inside() {
        let size = [640.0, 360.0];
        let n = |b: [f64; 4]| det([b[0] / 640.0, b[1] / 360.0, b[2] / 640.0, b[3] / 360.0], None);
        // uncut: no new aim
        assert_eq!(border_free_aim(&[n([280.0, 200.0, 360.0, 280.0])], [320.0, 240.0], size, 14.0, 0), None);
        // cut at the bottom, 80 px wide: v so that 1.25 x 80 px fits 14 px above the bottom, u kept
        let a = border_free_aim(&[n([280.0, 320.0, 360.0, 360.0])], [320.0, 340.0], size, 14.0, 0).unwrap();
        assert_eq!(a[0], 320.0);
        assert!((a[1] - (360.0 - 14.0 - 50.0)).abs() < 1e-9, "{a:?}");
        // the estimate grows with the attempt; the visible extent counts when it is the larger
        let b = border_free_aim(&[n([280.0, 200.0, 360.0, 360.0])], [320.0, 340.0], size, 14.0, 1).unwrap();
        assert!((b[1] - (360.0 - 14.0 - 160.0 * 1.5 / 2.0)).abs() < 1e-9, "{b:?}");
        // cut at the left: u moves right
        let c = border_free_aim(&[n([0.0, 150.0, 40.0, 210.0])], [10.0, 180.0], size, 14.0, 0).unwrap();
        assert!((c[0] - (14.0 + 75.0 / 2.0)).abs() < 1e-9 && c[1] == 180.0, "{c:?}");
        // cut on both sides of an axis, or too large to fit: none
        assert_eq!(border_free_aim(&[n([100.0, 0.0, 200.0, 360.0])], [150.0, 180.0], size, 14.0, 0), None);
        assert_eq!(border_free_aim(&[n([100.0, 200.0, 400.0, 360.0])], [250.0, 340.0], size, 14.0, 0), None);
        // the median edge decides: one cut frame of three does not
        let frames = [n([280.0, 200.0, 360.0, 300.0]), n([280.0, 200.0, 360.0, 301.0]), n([280.0, 200.0, 360.0, 359.0])];
        assert_eq!(border_free_aim(&frames, [320.0, 250.0], size, 14.0, 0), None);
    }

    #[test]
    fn tracker_widens_the_gate_for_a_box_at_the_border() {
        let setup = GraspSetup::default();
        let frame = |b: [f64; 4]| {
            let now = Instant::now();
            DetectionFrame { frame_id: 1, captured_at: now, policy_captured_at: Some(now), image_size_px: [640.0, 360.0], detections: vec![Detection { label: "cube".into(), ..det(b, None) }], depth: None, model_generation: None }
        };
        // predicted cut box (v 0.9..1.0) after a step; the box seen whole is 96 px higher
        let seen = frame([0.4, 300.0 / 360.0 - 0.3, 0.5, 300.0 / 360.0]);
        let cut = Tracker { label: "cube".into(), predicted: [0.4, 0.9, 0.5, 1.0], gate_px: setup.match_dist_px };
        assert!(cut.find(&seen, &setup).is_some());
        let uncut = Tracker { predicted: [0.4, 0.8, 0.5, 0.99], ..cut };
        assert!(uncut.find(&seen, &setup).is_none());
    }

    #[test]
    fn scale_height_rejects_noisy_sizes() {
        // lens 0.13 m above the top, 0.04 m up: sizes scale by 0.13 / 0.17
        let (s_low, s_high) = (100.0, 100.0 * 0.13 / 0.17);
        let d = scale_height(&[s_low; 3], &[s_high; 3], 0.04).unwrap();
        assert!((d - 0.13).abs() < 1e-9, "{d}");
        // 5 % size jitter: several cm of height uncertainty
        let e = scale_height(&[s_low * 0.95, s_low, s_low * 1.05], &[s_high * 1.05, s_high * 0.95, s_high], 0.04).unwrap_err();
        assert!(e.starts_with("scale_noise:"), "{e}");
        // 0.3 % jitter passes
        assert!(scale_height(&[s_low * 0.997, s_low, s_low * 1.003], &[s_high, s_high * 1.003, s_high], 0.04).is_ok());
        assert!(scale_height(&[s_low], &[s_low / 1.01], 0.04).unwrap_err().starts_with("scale_change:"));
    }

    #[test]
    fn parallax_distance_and_its_noise_gate() {
        // lens 0.13 m above the top, 0.04 m baseline, fx 600: the centre moves by 600 · 0.04 / 0.13 px
        let du = 600.0 * 0.04 / 0.13;
        let (un, up) = (320.0 + du / 2.0, 320.0 - du / 2.0);
        let d = parallax_distance(&[un; 3], &[up; 3], 600.0, 0.04).unwrap();
        assert!((d - 0.13).abs() < 1e-9, "{d}");
        // either direction of the shift and of the baseline gives the same distance
        assert!((parallax_distance(&[up; 3], &[un; 3], 600.0, -0.04).unwrap() - 0.13).abs() < 1e-9);
        // 3 px centre jitter: dD = 0.13 / 185 · √(3² + 3²) ≈ 3 mm, accepted
        assert!(parallax_distance(&[un - 3.0, un, un + 3.0], &[up + 3.0, up, up - 3.0], 600.0, 0.04).is_ok());
        // 10 px jitter on a 1 cm baseline (46 px shift): dD ≈ 40 mm, rejected
        let du1 = 600.0 * 0.01 / 0.13;
        let e = parallax_distance(&[320.0 + du1 / 2.0 - 10.0, 320.0 + du1 / 2.0 + 10.0], &[320.0 - du1 / 2.0 - 10.0, 320.0 - du1 / 2.0 + 10.0], 600.0, 0.01).unwrap_err();
        assert!(e.starts_with("parallax_noise:"), "{e}");
        assert!(parallax_distance(&[320.0], &[318.0], 600.0, 0.04).unwrap_err().starts_with("parallax_shift:"));
        assert!(parallax_distance(&[un], &[up], 600.0, 0.0005).unwrap_err().starts_with("parallax_baseline:"));
        assert_eq!(parallax_distance(&[], &[up], 600.0, 0.04).unwrap_err(), "no_frames");
    }

    #[test]
    fn top_face_feature_for_the_parallax() {
        let cx = 320.0;
        // straddling the principal point at both poses: the centre
        let (n, p, f) = top_face_u(&[[250.0, 0.0, 410.0, 1.0]], &[[200.0, 0.0, 360.0, 1.0]], cx).unwrap();
        assert_eq!((n, p, f), (vec![330.0], vec![280.0], "centre"));
        // left of it at one pose: the left edge (top face) in both
        let (n, p, f) = top_face_u(&[[200.0, 0.0, 330.0, 1.0]], &[[100.0, 0.0, 250.0, 1.0]], cx).unwrap();
        assert_eq!((n, p, f), (vec![200.0], vec![100.0], "left edge"));
        let (n, p, f) = top_face_u(&[[400.0, 0.0, 500.0, 1.0]], &[[300.0, 0.0, 420.0, 1.0]], cx).unwrap();
        assert_eq!((n, p, f), (vec![500.0], vec![420.0], "right edge"));
        // entirely right at one pose and entirely left at the other: no common top-face edge
        assert!(top_face_u(&[[330.0, 0.0, 380.0, 1.0]], &[[250.0, 0.0, 300.0, 1.0]], cx).is_none());
    }

    /// The former downward probe's estimator (lens-to-top distance at the start pose from the
    /// start sizes and the sizes `dz` closer), kept here only for the comparison below.
    fn down_probe_distance(start: &[f64], near: &[f64], dz: f64) -> f64 {
        let r = median(&mut near.to_vec()) / median(&mut start.to_vec());
        dz * r / (r - 1.0)
    }

    /// Same synthetic noisy sizes (Gaussian size jitter, 3 frames per pose) for the up probe
    /// (0.06 / 0.08 / 0.10 m) and the former down probe (0.04 m): distance errors without the
    /// noise gate, and how many estimates the gate accepts.
    #[test]
    fn up_and_down_probe_estimates_under_size_noise() {
        let mut seed = 12345u64;
        let mut gauss = || {
            let mut u = || { seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407); ((seed >> 11) as f64 + 0.5) / (1u64 << 53) as f64 };
            let (a, b) = (u(), u());
            (-2.0 * a.ln()).sqrt() * (2.0 * std::f64::consts::PI * b).cos()
        };
        let n = 2000;
        let stat = |v: &mut Vec<f64>| { v.sort_by(|a, b| a.total_cmp(b)); (v.iter().sum::<f64>() / v.len() as f64, v[v.len() / 2], v[v.len() - 1]) };
        println!("probe     D (m)  jitter  mean / median / max error (mm)  accepted");
        for d in [0.13, 0.20] {
            for jitter in [0.01, 0.03, 0.05] {
                let mut medians = Vec::new();
                for (dir, dz) in [("down", 0.04), ("up", 0.06), ("up", 0.08), ("up", 0.10)] {
                    let (mut err, mut ok) = (Vec::new(), 0);
                    for _ in 0..n {
                        let mut frames = |dist: f64| -> Vec<f64> { (0..3).map(|_| 100.0 / dist * (1.0 + jitter * gauss())).collect() };
                        let start = frames(d);
                        let (est, accepted) = if dir == "down" {
                            let near = frames(d - dz);
                            (down_probe_distance(&start, &near, dz), scale_height(&near, &start, dz).is_ok())
                        } else {
                            let high = frames(d + dz);
                            (dz / (median(&mut start.clone()) / median(&mut high.clone()) - 1.0), scale_height(&start, &high, dz).is_ok())
                        };
                        err.push((est - d).abs());
                        if accepted { ok += 1; }
                    }
                    let (m, med, max) = stat(&mut err);
                    medians.push(med);
                    println!("{dir:>4} {dz:.2}  {d:.2}   {:>3.0} %   {:>6.1} / {:>6.1} / {:>7.1}            {ok:>4}/{n}", jitter * 100.0, m * 1e3, med * 1e3, max * 1e3);
                }
                // a longer raise reduces the error of the up probe
                assert!(medians[3] < medians[1], "{medians:?}");
            }
        }
    }

    #[test]
    fn detector_angle_parsing_and_doubled_mean() {
        let raw = serde_json::json!([{"label": "pen", "score": 0.9, "box_norm": [0.1, 0.1, 0.3, 0.2], "angle_deg": 120.0, "angle_conf": 1.4},
                                     {"label": "pen", "score": 0.9, "box_norm": [0.1, 0.1, 0.3, 0.2]}]);
        let d = parse_detections(&raw, [640.0, 360.0]);
        assert_eq!((d[0].angle_deg, d[0].angle_conf), (Some(-60.0), Some(1.0)));
        assert_eq!((d[1].angle_deg, d[1].angle_conf), (None, None));
        // 89 and -89 are 2 deg apart: mean at the wrap, not 0
        let fr = [det([0.0; 4], Some((89.0, 0.9))), det([0.0; 4], Some((-89.0, 0.9))), det([0.0; 4], Some((10.0, 0.1)))];
        let (a, spread) = frames_angle(&fr, 0.3).unwrap();
        assert!(a.abs() > 89.9, "{a}");
        assert!(spread < 2.0, "spread for 89 and -89 should be small: {spread}");
        assert_eq!(frames_angle(&fr[2..], 0.3), None, "below align_min_angle_conf");

        // dispersion units: identical -> 0, ±40° -> ~40°, one frame -> 0
        let fr_ident = [det([0.0; 4], Some((30.0, 0.9))), det([0.0; 4], Some((30.0, 0.9)))];
        let (a_id, sp_id) = frames_angle(&fr_ident, 0.3).unwrap();
        assert!((a_id - 30.0).abs() < 1e-6);
        assert!(sp_id.abs() < 1e-6);

        let fr_pm = [det([0.0; 4], Some((40.0, 0.9))), det([0.0; 4], Some((-40.0, 0.9)))];
        let (a_pm, sp_pm) = frames_angle(&fr_pm, 0.3).unwrap();
        assert!(a_pm.abs() < 1e-6);
        assert!((sp_pm - 53.6).abs() < 1.0, "spread was {sp_pm}");

        let fr_single = [det([0.0; 4], Some((40.0, 0.9)))];
        let (_, sp_single) = frames_angle(&fr_single, 0.3).unwrap();
        assert_eq!(sp_single, 0.0);
        // the roll turns the jaw by -t about z
        let t = roll_across_long_side([0.0, 1.0, 0.0], [1.0, 0.0, 0.0]);
        assert!(t.abs() < 1e-12, "jaw already across a long side along x: {t}");
        let t = roll_across_long_side([1.0, 0.0, 0.0], [30f64.to_radians().cos(), 30f64.to_radians().sin(), 0.0]);
        assert!((t - (-120f64 + 180.0).to_radians()).abs() < 1e-9, "{}", t.to_degrees());
        assert!(GraspSetup::default().patched(&serde_json::json!({"align_min_angle_conf": 1.5})).is_err());
    }

    #[test]
    #[allow(clippy::approx_constant)]
    fn ray_and_projection_round_trip() {
        let setup = GraspSetup { camera_tilt_deg: 20.0, ..Default::default() };
        let cam = CamModel::assumed(&crate::chain::ArmModel::REBOT_DEV_ARM, &setup, &CameraMount::default());
        let pose = Pose::new(look_down(&inspect_r()), [0.3, 0.05, 0.2]);
        assert!(kinematics::column(&pose.r, 0)[2] < -0.999999);
        let p = cam.plane_point(&pose, [300.0, 200.0], 0.01).unwrap();
        let px = cam.project(&pose, &p).unwrap();
        assert!((px[0] - 300.0).abs() < 1e-6 && (px[1] - 200.0).abs() < 1e-6, "{px:?}");
        // a ray pointing above the horizon never meets the table (tool axis up, optical axis
        // 20 deg off it)
        let up = Pose::new(kinematics::axis_r(&[0.0, 1.0, 0.0], -std::f64::consts::FRAC_PI_2), pose.p);
        assert!(cam.plane_point(&up, [320.0, 180.0], 0.0).is_none());
    }

    /// v8 holder: rot 0, tilt -38, lens centre behind and above the jaw tips.
    fn v8() -> (GraspSetup, CameraMount, CamModel) {
        let setup = GraspSetup { camera_tilt_deg: -38.0, ..Default::default() };
        let mount = CameraMount { rot_deg: 0.0, tilt_deg: -38.0, offset_m: Some([-0.0731, 0.0005, 0.066]), ..Default::default() };
        let cam = CamModel::assumed(&crate::chain::ArmModel::REBOT_DEV_ARM, &setup, &mount);
        (setup, mount, cam)
    }

    /// End rotation at the inspect preset (joint angles, J6 at its rest angle 0).
    fn inspect_r() -> Mat3 { crate::chain::ArmModel::REBOT_DEV_ARM.fk(&[0.0, -1.167, -1.565, 1.435, 0.0, 0.0]).r }

    fn close(a: &Vec3, b: &Vec3, tol: f64) -> bool { kinematics::norm(&kinematics::sub(a, b)) < tol }

    #[test]
    fn camera_down_mode_follows_the_setting_tilt_and_lens_centre() {
        let (setup, mount, _) = v8();
        assert_eq!(camera_down_mode(&setup, &mount), Ok(true), "auto, tilted, lens centre known");
        let no_offset = CameraMount { offset_m: None, ..mount.clone() };
        assert_eq!(camera_down_mode(&setup, &no_offset), Ok(false), "auto without a lens centre");
        for tilt in [0.0, 4.9, -5.0] {
            assert_eq!(camera_down_mode(&GraspSetup { camera_tilt_deg: tilt, ..setup.clone() }, &mount), Ok(false), "auto at tilt {tilt}");
        }
        assert_eq!(camera_down_mode(&GraspSetup { camera_tilt_deg: -5.1, ..setup.clone() }, &mount), Ok(true));
        let off = GraspSetup { acquire_camera_down: AcquireCameraDown::Off, ..setup.clone() };
        assert_eq!(camera_down_mode(&off, &mount), Ok(false));
        let on = GraspSetup { acquire_camera_down: AcquireCameraDown::On, camera_tilt_deg: 0.0, ..setup.clone() };
        assert_eq!(camera_down_mode(&on, &mount), Ok(true));
        assert!(camera_down_mode(&on, &no_offset).is_err(), "on needs the lens centre");
        // setup field: lowercase strings, default auto, other values rejected
        let d = GraspSetup::default();
        assert_eq!(serde_json::to_value(&d).unwrap()["acquire_camera_down"], "auto");
        assert_eq!(d.patched(&serde_json::json!({"acquire_camera_down": "on"})).unwrap().acquire_camera_down, AcquireCameraDown::On);
        assert_eq!((d.height_probe_mode, d.lateral_probe_m), (HeightProbeMode::Up, 0.04));
        assert_eq!(d.patched(&serde_json::json!({"height_probe_mode": "lateral"})).unwrap().height_probe_mode, HeightProbeMode::Lateral);
        assert!(d.patched(&serde_json::json!({"height_probe_mode": "sideways"})).is_err());
        for (b, ok) in [(0.009, false), (0.01, true), (0.08, true), (0.081, false)] {
            assert_eq!(d.patched(&serde_json::json!({"lateral_probe_m": b})).is_ok(), ok, "{b}");
        }
        assert!(d.patched(&serde_json::json!({"recheck_tol_px": 0.0})).is_err() && d.patched(&serde_json::json!({"recheck_tol_px": 501.0})).is_err());
        assert_eq!(d.patched(&serde_json::json!({"recheck_tol_px": 25.0})).unwrap().recheck_tol_px, 25.0);
        for bad in [serde_json::json!("yes"), serde_json::json!(true), serde_json::json!(1)] {
            assert!(d.patched(&serde_json::json!({"acquire_camera_down": bad})).is_err(), "{bad}");
        }
    }

    #[test]
    fn camera_down_pose_puts_the_optical_axis_down_over_the_target() {
        let (setup, _, cam) = v8();
        let target = [0.28, -0.04, setup.table_z_m + setup.grasp_height_m];
        let r_now = inspect_r();
        let pose = camera_down_pose(&r_now, &target, &setup, &cam, &[]);
        let rc = cam.rotation(&pose);
        assert!(kinematics::column(&rc, 2)[2] < -1.0 + 1e-12, "optical axis down: {:?}", kinematics::column(&rc, 2));
        let o = cam.origin(&pose);
        assert!((o[0] - target[0]).abs() < 1e-12 && (o[1] - target[1]).abs() < 1e-12, "camera centre over the target: {o:?}");
        assert!((pose.p[2] - (setup.table_z_m + setup.approach_height_m)).abs() < 1e-12, "TCP at approach height");
        // the tool is pitched by the mount tilt about x_cam; heading (x_cam) kept
        let tool_from_down = kinematics::column(&pose.r, 0)[2].clamp(-1.0, 1.0).acos();
        assert!((std::f64::consts::PI - tool_from_down - 38f64.to_radians()).abs() < 1e-9, "tool pitched by 38 deg");
        let v = look_down(&r_now);
        let x_cam_v = kinematics::mat_vec(&v, &kinematics::column(&cam.rec, 0));
        assert!(close(&kinematics::column(&rc, 0), &x_cam_v, 1e-9), "pitch about x_cam");
        assert!(kinematics::angle_between(&look_down(&pose.r), &v) < 1e-9, "look_down undoes it");
        // the image centre ray meets the plane straight below the camera
        let hit = cam.plane_point(&pose, [cam.cx, cam.cy], target[2]).unwrap();
        assert!(close(&hit, &target, 1e-9), "{hit:?}");
        // tilt 0: the optical axis already is the tool axis, the orientation is look_down's
        let flat = CamModel::assumed(&crate::chain::ArmModel::REBOT_DEV_ARM, &GraspSetup::default(), &CameraMount::default());
        assert_eq!(camera_down(&v, &flat.rec), v);
    }

    #[test]
    fn tilted_ray_meets_the_table_ahead_by_the_tilt() {
        // tool vertical, camera tilted 38 deg: the image centre ray meets the plane h·tan(38)
        // from the point below the lens centre
        let (setup, _, cam) = v8();
        let pose = Pose::new(look_down(&inspect_r()), [0.25, 0.0, 0.15]);
        let o = cam.origin(&pose);
        let plane = setup.table_z_m + setup.grasp_height_m;
        let hit = cam.plane_point(&pose, [cam.cx, cam.cy], plane).unwrap();
        let ahead = (hit[0] - o[0]).hypot(hit[1] - o[1]);
        assert!((ahead - (o[2] - plane) * 38f64.to_radians().tan()).abs() < 1e-9, "ahead {ahead}");
        let px = cam.project(&pose, &hit).unwrap();
        assert!((px[0] - cam.cx).abs() < 1e-6 && (px[1] - cam.cy).abs() < 1e-6);
        // the table point under the jaw tips is outside the image at approach height (v < 0)
        let under = cam.project(&pose, &[pose.p[0], pose.p[1], setup.table_z_m]).map(|p| cam.to_norm(p));
        assert!(under.is_none_or(|n| !(0.0..=1.0).contains(&n[1])), "{under:?}");
    }

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
    fn envelope_has_the_camera_when_a_lens_or_body_is_set() {
        let ws = crate::model::ArmConfig::rebot_dm().workspace;
        let rec = kinematics::cam_mount(false, 0.0, -38.0);
        let n = |mount: CameraMount| envelope(ws.as_ref(), &mount, &rec).len();
        assert_eq!(n(CameraMount::default()), 9, "TCP and 8 check points");
        assert_eq!(n(CameraMount { offset_m: Some([-0.06, 0.0, 0.04]), ..Default::default() }), 18);
        // a body size without a lens centre: the lens at the end_link origin, as CamModel assumes
        let pts = envelope(ws.as_ref(), &CameraMount { body_size_m: Some([0.05, 0.05, 0.04]), ..Default::default() }, &rec);
        assert_eq!((pts.len(), pts[9]), (18, [0.0; 3]));
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

    #[test]
    fn test_parse_detections_grasp_width_px() {
        let raw = serde_json::json!([
            {"label": "can", "score": 0.9, "box_norm": [0.1, 0.1, 0.3, 0.3], "grasp_width_px": 42.5},
            {"label": "cup", "score": 0.8, "box_norm": [0.2, 0.2, 0.4, 0.4], "width_px": 35.0},
            {"label": "box", "score": 0.7, "box_norm": [0.3, 0.3, 0.5, 0.5], "grasp_width_px": 0.0},
            {"label": "pen", "score": 0.6, "box_norm": [0.4, 0.4, 0.6, 0.6], "width_px": -5.0},
            {"label": "mug", "score": 0.5, "box_norm": [0.5, 0.5, 0.7, 0.7]},
            [320.0, 180.0, 64.0, 36.0, 0.9, "arr_obj"]
        ]);
        let d = parse_detections(&raw, [640.0, 360.0]);
        assert_eq!(d.len(), 6);
        assert_eq!(d[0].grasp_width_px, Some(42.5));
        assert_eq!(d[1].grasp_width_px, Some(35.0));
        assert_eq!(d[2].grasp_width_px, None, "zero width rejected");
        assert_eq!(d[3].grasp_width_px, None, "negative width rejected");
        assert_eq!(d[4].grasp_width_px, None, "absent width");
        assert_eq!(d[5].grasp_width_px, None, "array format detection has no grasp_width_px");
    }

    #[test]
    fn test_frames_width_px() {
        let mut d1 = det([0.0; 4], Some((0.0, 0.8)));
        d1.grasp_width_px = Some(40.0);

        let mut d2 = det([0.0; 4], Some((0.0, 0.9)));
        d2.grasp_width_px = Some(50.0);

        let mut d3 = det([0.0; 4], Some((0.0, 0.7)));
        d3.grasp_width_px = Some(45.0);

        // Low confidence detection
        let mut d_low = det([0.0; 4], Some((0.0, 0.2)));
        d_low.grasp_width_px = Some(100.0);

        // No width detection
        let d_nowidth = det([0.0; 4], Some((0.0, 0.9)));

        // Non-positive width
        let mut d_neg = det([0.0; 4], Some((0.0, 0.9)));
        d_neg.grasp_width_px = Some(-10.0);

        let frames = vec![d1, d2, d3, d_low, d_nowidth, d_neg];

        // min_conf = 0.5 filters out d_low. Valid widths: [40, 50, 45]. Median of [40, 45, 50] is 45.0.
        let w = frames_width_px(&frames, 0.5);
        assert_eq!(w, Some(45.0));

        // Even count: only d1 and d2 (min_conf = 0.75). Widths: [40, 50]. Median is 45.0.
        let w_even = frames_width_px(&frames, 0.75);
        assert_eq!(w_even, Some(45.0));

        // min_conf too high: no frames qualify
        assert_eq!(frames_width_px(&frames, 0.95), None);

        // Empty slice
        assert_eq!(frames_width_px(&[], 0.5), None);
    }

    #[test]
    fn test_plane_point_width_conversion() {
        let setup = GraspSetup { camera_tilt_deg: 0.0, fx_px: 600.0, fy_px: 600.0, image_size_px: [640.0, 360.0], ..Default::default() };
        let cam = CamModel::assumed(&crate::chain::ArmModel::REBOT_DEV_ARM, &setup, &CameraMount::default());
        let pose = Pose::new(look_down(&inspect_r()), [0.3, 0.0, 0.20]);
        let plane_z = 0.0;

        let centre_px = [320.0, 180.0];
        let w_px = 60.0;
        let closing_deg: f64 = 90.0;
        let (u_x, u_y) = (closing_deg.to_radians().cos(), closing_deg.to_radians().sin());
        let half_w = w_px / 2.0;
        let p0_px = [centre_px[0] - half_w * u_x, centre_px[1] - half_w * u_y];
        let p1_px = [centre_px[0] + half_w * u_x, centre_px[1] + half_w * u_y];

        let p0 = cam.plane_point(&pose, p0_px, plane_z).expect("p0 meets plane");
        let p1 = cam.plane_point(&pose, p1_px, plane_z).expect("p1 meets plane");
        let width_m = kinematics::norm(&kinematics::sub(&p1, &p0));

        // Projected width: dx = w_px * height / fx = 60.0 * 0.20 / 600.0 = 0.02 m (20 mm)
        assert!((width_m - 0.02).abs() < 1e-4, "expected 0.02 m, got {width_m}");
    }

    #[test]
    fn test_parse_detections_depth_fields() {
        let raw = serde_json::json!([
            {
                "label": "cup",
                "score": 0.9,
                "box_norm": [0.1, 0.1, 0.3, 0.3],
                "disp_fg": 1.25,
                "disp_bg": 0.85,
                "depth_conf": 0.92,
                "length_px": 55.0,
                "jaw_clear": true,
            },
            {
                "label": "bottle",
                "score": 0.8,
                "box_norm": [0.2, 0.2, 0.4, 0.4],
                "disp_fg": 0.0,
                "disp_bg": -0.5,
                "depth_conf": 1.5,
                "length_px": -10.0,
                "jaw_clear": false,
            }
        ]);
        let d = parse_detections(&raw, [640.0, 360.0]);
        assert_eq!(d.len(), 2);
        assert_eq!(d[0].disp_fg, Some(1.25));
        assert_eq!(d[0].disp_bg, Some(0.85));
        assert_eq!(d[0].depth_conf, Some(0.92));
        assert_eq!(d[0].length_px, Some(55.0));
        assert_eq!(d[0].jaw_clear, Some(true));

        assert_eq!(d[1].disp_fg, Some(0.0));
        assert_eq!(d[1].disp_bg, Some(-0.5));
        assert_eq!(d[1].depth_conf, Some(1.0), "clamped to 1.0");
        assert_eq!(d[1].length_px, None, "negative length rejected");
        assert_eq!(d[1].jaw_clear, Some(false));
    }

    #[test]
    fn test_camera_optical_depth_and_ray() {
        let setup = GraspSetup { camera_tilt_deg: 0.0, fx_px: 600.0, fy_px: 600.0, image_size_px: [640.0, 360.0], ..Default::default() };
        let cam = CamModel::assumed(&crate::chain::ArmModel::REBOT_DEV_ARM, &setup, &CameraMount::default());
        let pose = Pose::new(look_down(&inspect_r()), [0.3, 0.0, 0.20]);
        let table_z = 0.0;

        // Image centre ray straight down: optical axis aligned with -z
        let ray = cam.ray_world_per_z(&pose, [320.0, 180.0]);
        assert!((ray[0]).abs() < 1e-6 && (ray[1]).abs() < 1e-6);
        assert!((ray[2] - (-1.0)).abs() < 1e-6, "ray[2] should be -1.0 for vertical down camera");

        // Optical depth: z_cam = (0.0 - 0.20) / (-1.0) = 0.20 m
        let z_opt = cam.table_optical_depth(&pose, [320.0, 180.0], table_z).expect("optical depth valid");
        assert!((z_opt - 0.20).abs() < 1e-6, "expected 0.20 m, got {z_opt}");

        // Off-centre pixel: orthogonal depth to table plane is the same (0.20 m)
        let z_off = cam.table_optical_depth(&pose, [420.0, 180.0], table_z).expect("off-centre valid");
        assert!((z_off - 0.20).abs() < 1e-6, "expected 0.20 m, got {z_off}");
    }

    #[test]
    fn test_depth_calibration_and_measurement() {
        let setup = GraspSetup { camera_tilt_deg: 0.0, fx_px: 600.0, fy_px: 600.0, image_size_px: [640.0, 360.0], ..Default::default() };
        let cam = CamModel::assumed(&crate::chain::ArmModel::REBOT_DEV_ARM, &setup, &CameraMount::default());
        let pose = Pose::new(look_down(&inspect_r()), [0.3, 0.0, 0.20]);
        let table_z = 0.0;

        // 1. Metric model with visible table
        let z_table = 0.20;
        let disp_table = 1.0 / z_table; // 5.0
        let samples = vec![
            [320.0, 180.0, disp_table],
            [300.0, 160.0, disp_table * 1.01],
            [340.0, 200.0, disp_table * 0.99],
        ];
        let depth_info = DepthFrameInfo { metric_model: true, table_samples: samples };
        let cal = calibrate_depth(&cam, &pose, Some(&depth_info), None, table_z).expect("metric_table calibrated");
        assert_eq!(cal.source, "metric_table");
        assert!((cal.a - 1.0).abs() < 0.05);
        assert_eq!(cal.b, 0.0);

        // Object on table: top height 0.04 m -> z_fg = 0.16 m -> disp_fg = 1/0.16 = 6.25, disp_bg = 1/0.20 = 5.0
        let det = Detection {
            label: "cube".into(),
            score: 0.9,
            raw_score_valid: true,
            box_norm: [0.45, 0.45, 0.55, 0.55],
            angle_deg: None,
            angle_conf: None,
            grasp_width_px: Some(30.0),
            mask_focus: false,
            disp_fg: Some(6.25),
            disp_bg: Some(5.0),
            depth_conf: Some(0.95),
            length_px: Some(50.0),
            jaw_clear: Some(true),
            depth_angle_deg: None,
            depth_angle_conf: None,
            depth_width_px: None, contour_norm: None,
        };
        let meas = measure_detection_depth(&cam, &pose, &det, Some(&depth_info), None, table_z).expect("measurement valid");
        assert!((meas.height_m - 0.04).abs() < 0.005, "height error: {}", (meas.height_m - 0.04).abs());
        assert_eq!(meas.source, "metric_table");
        assert_eq!(meas.jaw_clear, Some(true));

        // 2. Table occluded (0 samples): no calibration, the grasp probes instead
        let occluded_depth = DepthFrameInfo { metric_model: true, table_samples: Vec::new() };
        assert!(calibrate_depth(&cam, &pose, Some(&occluded_depth), None, table_z).is_none(), "occluded table must not calibrate");

        // 3. Metric model with table samples contradicting FK returns None
        let conflicting_samples = vec![
            [320.0, 180.0, disp_table * 2.0],
            [300.0, 160.0, disp_table],
            [340.0, 200.0, disp_table * 2.0],
            [360.0, 220.0, disp_table],
        ];
        let conflicting_depth = DepthFrameInfo { metric_model: true, table_samples: conflicting_samples };
        assert!(calibrate_depth(&cam, &pose, Some(&conflicting_depth), None, table_z).is_none(), "contradicting table samples should return None");
    }

    #[test]
    fn test_anisotropic_camera_sizing() {
        let setup = GraspSetup { camera_tilt_deg: 0.0, fx_px: 600.0, fy_px: 400.0, image_size_px: [640.0, 360.0], ..Default::default() };
        let cam = CamModel::assumed(&crate::chain::ArmModel::REBOT_DEV_ARM, &setup, &CameraMount::default());
        let pose = Pose::new(look_down(&inspect_r()), [0.3, 0.0, 0.20]);
        let z_fg = 0.16;
        let cal = DepthCalibration { a: 1.0, b: 0.0, source: "metric_table" };
        let det = Detection {
            label: "obj".into(),
            score: 0.9,
            raw_score_valid: true,
            box_norm: [0.45, 0.45, 0.55, 0.55],
            angle_deg: Some(90.0),
            angle_conf: Some(0.9),
            grasp_width_px: Some(30.0),
            mask_focus: false,
            disp_fg: Some(1.0 / z_fg),
            disp_bg: Some(1.0 / 0.20),
            depth_conf: Some(0.95),
            length_px: Some(50.0),
            jaw_clear: Some(true),
            depth_angle_deg: None,
            depth_angle_conf: None,
            depth_width_px: None, contour_norm: None,
        };
        let meas = measure_detection_depth_with_cal(&cam, &pose, &det, &cal).expect("valid measurement");
        let expected_w = 30.0 * z_fg / 600.0;
        let expected_l = 50.0 * z_fg / 400.0;
        assert!((meas.width_m.unwrap() - expected_w).abs() < 1e-6);
        assert!((meas.length_m.unwrap() - expected_l).abs() < 1e-6);
    }

    #[test]
    fn test_http_detection_source_rescaling() {
        let src = HttpDetectionSource {
            upstream: DetectorUpstream::parse("http://127.0.0.1:8090").unwrap(),
            image_size_px: [640.0, 360.0],
        };
        let raw = serde_json::json!({
            "frame_id": 1,
            "width": 1280,
            "height": 720,
            "depth": {
                "metric_model": true,
                "table_samples": [[100.0, 200.0, 0.85]],
            },
            "detections": [{
                "label": "cup",
                "score": 0.9,
                "format": "xywh_center",
                "x": 640.0,
                "y": 360.0,
                "w": 100.0,
                "h": 100.0,
                "length_px": 80.0,
                "grasp_width_px": 40.0,
            }]
        });
        let now = Instant::now();
        let mut frame = src.frame(&raw, None, now, now, 0).expect("frame parsed");
        assert_eq!(frame.image_size_px, [1280.0, 720.0]);
        assert_eq!(frame.depth.as_ref().unwrap().table_samples[0], [100.0, 200.0, 0.85]);
        assert_eq!(frame.detections[0].length_px, Some(80.0));
        assert_eq!(frame.detections[0].grasp_width_px, Some(40.0));

        let cam_iso = CamModel {
            rec: crate::kinematics::IDENTITY,
            offset: [0.0; 3],
            fx: 900.0,
            fy: 900.0,
            cx: 480.0,
            cy: 270.0,
            size: [960.0, 540.0],
        };
        rescale_frame_to_cam(&mut frame, &cam_iso);
        assert_eq!(frame.image_size_px, [960.0, 540.0]);
        assert_eq!(frame.depth.as_ref().unwrap().table_samples[0], [75.0, 150.0, 0.85]);
        assert_eq!(frame.detections[0].length_px, Some(60.0));
        assert_eq!(frame.detections[0].grasp_width_px, Some(30.0));

        let cal = DepthCalibration { a: 1.0, b: 0.0, source: "test" };
        let pose = Pose::new(crate::kinematics::IDENTITY, [0.0; 3]);
        let mut det_iso = frame.detections[0].clone();
        det_iso.depth_conf = Some(0.9);
        det_iso.disp_fg = Some(1.0);
        det_iso.disp_bg = Some(0.5);
        let meas_iso = measure_detection_depth_with_cal(&cam_iso, &pose, &det_iso, &cal).expect("measured");
        let expected_l_iso = 80.0 / 1200.0;
        let expected_w_iso = 40.0 / 1200.0;
        assert!((meas_iso.length_m.unwrap() - expected_l_iso).abs() < 1e-9);
        assert!((meas_iso.width_m.unwrap() - expected_w_iso).abs() < 1e-9);

        // Anisotropic rescaling:
        let now_aniso = Instant::now();
        let mut frame_aniso = DetectionFrame {
            frame_id: 2,
            captured_at: now_aniso,
            policy_captured_at: Some(now_aniso),
            image_size_px: [640.0, 360.0],
            detections: vec![Detection {
                label: "cup".into(),
                score: 0.9,
                raw_score_valid: true,
                box_norm: [0.3, 0.3, 0.7, 0.7],
                angle_deg: Some(45.0),
                angle_conf: Some(0.9),
                grasp_width_px: Some(120.0),
                mask_focus: false,
                disp_fg: Some(1.0),
                disp_bg: Some(0.5),
                depth_conf: Some(0.9),
                length_px: Some(240.0),
                jaw_clear: Some(true),
                depth_angle_deg: None,
                depth_angle_conf: None,
                depth_width_px: None, contour_norm: None,
            }],
            depth: None,
            model_generation: None,
        };
        let cam_aniso = CamModel {
            rec: crate::kinematics::IDENTITY,
            offset: [0.0; 3],
            fx: 1200.0,
            fy: 1800.0,
            cx: 640.0,
            cy: 540.0,
            size: [1280.0, 1080.0],
        };
        rescale_frame_to_cam(&mut frame_aniso, &cam_aniso);
        assert_eq!(frame_aniso.image_size_px, [1280.0, 1080.0]);

        let meas_aniso = measure_detection_depth_with_cal(&cam_aniso, &pose, &frame_aniso.detections[0], &cal).expect("measured");
        let expected_l_aniso = 240.0 / 600.0;
        let expected_w_aniso = 120.0 / 600.0;
        assert!((meas_aniso.length_m.unwrap() - expected_l_aniso).abs() < 1e-9);
        assert!((meas_aniso.width_m.unwrap() - expected_w_aniso).abs() < 1e-9);
    }

    #[test]
    fn yaw_and_width_source_settings_default_validate_and_patch() {
        let d = GraspSetup::default();
        assert_eq!(d.yaw_sources, [YawSource::Mask, YawSource::Box]);
        assert_eq!(d.yaw_record, [YawSource::Depth]);
        assert_eq!(d.width_sources, [WidthSource::Mask, WidthSource::Depth]);
        assert!(d.width_record.is_empty());
        // a setup file without the keys loads the defaults
        let old: GraspSetup = serde_json::from_value(serde_json::json!({"table_z_m": 0.01})).unwrap();
        assert_eq!((old.yaw_sources, old.yaw_record, old.width_sources, old.width_record),
            (d.yaw_sources.clone(), d.yaw_record.clone(), d.width_sources.clone(), d.width_record.clone()));
        let p = d.patched(&serde_json::json!({"yaw_sources": ["depth", "box"], "yaw_record": ["mask"], "width_sources": ["depth"], "width_record": ["mask"]})).unwrap();
        assert_eq!(p.yaw_sources, [YawSource::Depth, YawSource::Box]);
        assert_eq!(p.width_record, [WidthSource::Mask]);
        // the default yaw_record may name a deciding source: it decides instead
        assert!(d.patched(&serde_json::json!({"yaw_sources": ["depth", "mask", "box"]})).is_ok());
        for bad in [
            serde_json::json!({"yaw_sources": ["mask", "mask"]}),
            serde_json::json!({"yaw_record": ["depth", "depth"]}),
            serde_json::json!({"width_sources": ["mask", "mask"]}),
            serde_json::json!({"width_record": ["depth", "depth"]}),
            serde_json::json!({"yaw_sources": ["probe"]}),
            serde_json::json!({"width_sources": ["box"]}),
        ] {
            assert!(d.patched(&bad).is_err(), "{bad} accepted");
        }
    }

    /// A centred, elongated box (aspect 2) with mask and / or depth angles per frame.
    fn yaw_frames(mask: &[f64], depth: &[f64]) -> (Detection, Vec<Detection>) {
        let n = mask.len().max(depth.len()).max(1);
        let frames: Vec<Detection> = (0..n).map(|i| Detection {
            angle_deg: mask.get(i).copied(), angle_conf: mask.get(i).map(|_| 0.9),
            depth_angle_deg: depth.get(i).copied(), depth_angle_conf: depth.get(i).map(|_| 0.8),
            ..det([0.4, 0.45, 0.6, 0.55], None)
        }).collect();
        (frames[n - 1].clone(), frames)
    }

    const SIZE: [f64; 2] = [640.0, 360.0];

    fn turn_by(t: f64) -> impl Fn(f64) -> Result<Option<f64>, &'static str> { move |_| Ok(Some(t)) }

    fn summary(o: &YawOutcome) -> Vec<(&str, bool, bool, Option<&str>)> {
        o.entries.iter().map(|e| (e.source.as_str(), e.used, e.record_only, e.reason.as_deref())).collect()
    }

    #[test]
    fn yaw_chain_default_mask_decides_and_depth_is_recorded() {
        let s = GraspSetup::default();
        let (last, frames) = yaw_frames(&[30.0], &[-40.0]);
        let o = yaw_chain(&s, &last, &frames, SIZE, &turn_by(0.4), &|_| Some(1.0));
        assert_eq!((o.turn, o.align.as_str()), (Some(0.4), "turned:angle"));
        assert_eq!(summary(&o), [("depth", false, true, None), ("mask", true, false, None)]);
        assert_eq!(o.entries[0].value, Some(-40.0));
        assert_eq!(o.entries[1].value, Some(30.0));
        assert_eq!(o.angle_spread_deg, Some(0.0));
        // a turn below 1e-3 rad keeps J6
        let o = yaw_chain(&s, &last, &frames, SIZE, &|_| Ok(None), &|_| Some(1.0));
        assert_eq!((o.turn, o.align.as_str()), (None, "kept:angle"));
    }

    #[test]
    fn yaw_chain_depth_first_decides_with_both_present() {
        let s = GraspSetup { yaw_sources: vec![YawSource::Depth, YawSource::Mask, YawSource::Box], ..GraspSetup::default() };
        let (last, frames) = yaw_frames(&[30.0], &[-40.0]);
        let seen = std::cell::Cell::new(f64::NAN);
        let angle_turn = |alpha: f64| { seen.set(alpha); Ok(Some(0.2)) };
        let o = yaw_chain(&s, &last, &frames, SIZE, &angle_turn, &|_| Some(1.0));
        assert_eq!((o.turn, o.align.as_str()), (Some(0.2), "turned:depth"));
        assert_eq!(seen.get(), -40.0);
        // depth is a deciding source here (not recorded twice), the mask is not evaluated
        assert_eq!(summary(&o), [("depth", true, false, None)]);
        // the depth source never writes angle_spread_deg
        assert_eq!(o.angle_spread_deg, None);
        let o = yaw_chain(&s, &last, &frames, SIZE, &|_| Ok(None), &|_| Some(1.0));
        assert_eq!(o.align, "kept:depth");
    }

    #[test]
    fn yaw_chain_record_only_depth_never_turns_and_box_decides_without_a_mask() {
        let s = GraspSetup::default();
        // a plain detection model: no mask angle, a depth angle
        let (last, frames) = yaw_frames(&[], &[-40.0]);
        let angle_turn = |_: f64| -> Result<Option<f64>, &'static str> { panic!("no angle source decides") };
        let o = yaw_chain(&s, &last, &frames, SIZE, &angle_turn, &|short_x| { assert!(!short_x); Some(1.5) });
        assert_eq!((o.turn, o.align.as_str()), (Some(1.5), "turned:box"));
        assert_eq!(summary(&o), [("depth", false, true, None), ("mask", false, false, Some("no_angle")), ("box", true, false, None)]);
        assert_eq!(o.entries[0].value, Some(-40.0));
        assert_eq!(o.entries[2].value, Some(0.0), "box long side along image x");
        assert_eq!(o.angle_spread_deg, None);
        let o = yaw_chain(&s, &last, &frames, SIZE, &angle_turn, &|_| None);
        assert_eq!((o.turn, o.align.as_str()), (None, "kept:box"));
        // a square box: the aspect rejects, nothing else: skipped:aspect as before the chain
        let sq = Detection { box_norm: [0.45, 0.4, 0.55, 0.578], ..last.clone() };
        let o = yaw_chain(&s, &sq, std::slice::from_ref(&sq), SIZE, &angle_turn, &|_| Some(1.0));
        assert_eq!((o.turn, o.align.as_str()), (None, "skipped:aspect"));
        assert_eq!(o.entries.last().unwrap().reason.as_deref(), Some("aspect"));
    }

    #[test]
    fn yaw_chain_spread_gate_is_terminal_per_angle_source() {
        // mask frames 0 / 60 deg disagree: skipped:angle_spread, the box does not decide
        let s = GraspSetup::default();
        let (last, frames) = yaw_frames(&[0.0, 60.0], &[]);
        let o = yaw_chain(&s, &last, &frames, SIZE, &turn_by(0.3), &|_| Some(1.0));
        assert_eq!((o.turn, o.align.as_str()), (None, "skipped:angle_spread"));
        assert!(o.angle_spread_deg.is_some_and(|a| a > 15.0));
        assert_eq!(summary(&o).last().unwrap(), &("mask", false, false, Some("angle_spread")));
        // the same for a deciding depth source, which does not write angle_spread_deg
        let s = GraspSetup { yaw_sources: vec![YawSource::Depth, YawSource::Box], yaw_record: vec![], ..GraspSetup::default() };
        let (last, frames) = yaw_frames(&[], &[0.0, 60.0]);
        let o = yaw_chain(&s, &last, &frames, SIZE, &turn_by(0.3), &|_| Some(1.0));
        assert_eq!((o.turn, o.align.as_str(), o.angle_spread_deg), (None, "skipped:angle_spread", None));
        // record-only: the spread only rejects the entry
        let s = GraspSetup::default();
        let (last, frames) = yaw_frames(&[10.0, 10.0], &[0.0, 60.0]);
        let o = yaw_chain(&s, &last, &frames, SIZE, &turn_by(0.3), &|_| Some(1.0));
        assert_eq!(o.align, "turned:angle");
        assert_eq!(summary(&o)[0], ("depth", false, true, Some("angle_spread")));
        // spread 0 disables the gate
        let s = GraspSetup { align_max_angle_spread_deg: 0.0, ..GraspSetup::default() };
        let (last, frames) = yaw_frames(&[0.0, 60.0], &[]);
        assert_eq!(yaw_chain(&s, &last, &frames, SIZE, &turn_by(0.3), &|_| Some(1.0)).align, "turned:angle");
    }

    #[test]
    fn yaw_chain_guard_ray_and_limits_end_the_alignment() {
        let s = GraspSetup::default();
        let (last, frames) = yaw_frames(&[30.0], &[-40.0]);
        // the box at the image border: skipped:border before any source, the record is kept
        let cut = Detection { box_norm: [0.0, 0.45, 0.2, 0.55], ..last.clone() };
        let o = yaw_chain(&s, &cut, std::slice::from_ref(&cut), SIZE, &turn_by(0.3), &|_| Some(1.0));
        assert_eq!((o.turn, o.align.as_str()), (None, "skipped:border"));
        assert_eq!(summary(&o), [("depth", false, true, None)]);
        for why in ["ray", "j6_limits"] {
            let o = yaw_chain(&s, &last, &frames, SIZE, &|_| Err(why), &|_| Some(1.0));
            assert_eq!((o.turn, o.align), (None, format!("skipped:{why}")));
            assert_eq!(o.entries.last().unwrap().reason.as_deref(), Some(why));
        }
        // no deciding source
        let s = GraspSetup { yaw_sources: vec![], ..GraspSetup::default() };
        assert_eq!(yaw_chain(&s, &last, &frames, SIZE, &turn_by(0.3), &|_| Some(1.0)).align, "skipped:no_source");
    }

    #[test]
    fn depth_angle_fields_parse_and_rescale_like_the_mask_angle() {
        let raw = serde_json::json!([{"label": "block", "score": 0.9, "box_norm": [0.1, 0.1, 0.3, 0.2],
            "depth_angle_deg": 120.0, "depth_angle_conf": 1.5, "depth_width_px": 20.0},
            {"label": "block", "score": 0.9, "box_norm": [0.1, 0.1, 0.3, 0.2], "depth_width_px": -1.0}]);
        let d = parse_detections(&raw, [640.0, 360.0]);
        assert_eq!((d[0].depth_angle_deg, d[0].depth_angle_conf, d[0].depth_width_px), (Some(-60.0), Some(1.0), Some(20.0)));
        assert_eq!((d[0].angle_deg, d[0].grasp_width_px), (None, None), "depth values stay separate");
        assert_eq!((d[1].depth_angle_deg, d[1].depth_width_px), (None, None));
        // the same rescale as angle_deg / grasp_width_px
        let setup = GraspSetup { fx_px: 600.0, fy_px: 600.0, image_size_px: [640.0, 360.0], ..Default::default() };
        let cam = CamModel::assumed(&crate::chain::ArmModel::REBOT_DEV_ARM, &setup, &CameraMount::default());
        let mk = |mask: bool| Detection {
            angle_deg: mask.then_some(30.0), grasp_width_px: mask.then_some(40.0), length_px: Some(80.0),
            depth_angle_deg: (!mask).then_some(30.0), depth_width_px: (!mask).then_some(40.0),
            ..det([0.4, 0.4, 0.6, 0.6], None)
        };
        let frame = |d: Detection| {
            let now = Instant::now();
            DetectionFrame { frame_id: 1, captured_at: now, policy_captured_at: Some(now), image_size_px: [1280.0, 540.0], detections: vec![d], depth: None, model_generation: None }
        };
        let (mut fm, mut fd) = (frame(mk(true)), frame(mk(false)));
        rescale_frame_to_cam(&mut fm, &cam);
        rescale_frame_to_cam(&mut fd, &cam);
        let (m, dd) = (&fm.detections[0], &fd.detections[0]);
        assert_eq!(m.angle_deg, dd.depth_angle_deg);
        assert_eq!(m.grasp_width_px, dd.depth_width_px);
        assert_eq!(m.length_px, dd.length_px, "the length follows the depth angle without a mask angle");
        assert!((m.angle_deg.unwrap() - 30.0).abs() > 1.0, "anisotropic rescale changed the angle");
    }

    #[test]
    fn depth_measurement_width_from_the_detector_else_the_depth_pipeline() {
        let setup = GraspSetup { camera_tilt_deg: 0.0, fx_px: 600.0, fy_px: 600.0, image_size_px: [640.0, 360.0], ..Default::default() };
        let cam = CamModel::assumed(&crate::chain::ArmModel::REBOT_DEV_ARM, &setup, &CameraMount::default());
        let pose = Pose::new(look_down(&inspect_r()), [0.3, 0.0, 0.20]);
        let cal = DepthCalibration { a: 1.0, b: 0.0, source: "metric_table" };
        let base = Detection { disp_fg: Some(6.25), disp_bg: Some(5.0), depth_conf: Some(0.95), length_px: Some(50.0), ..det([0.45, 0.45, 0.55, 0.55], None) };
        // a detection model: depth PCA width 30 px at 90 deg -> the numbers of grasp_width_px / angle_deg
        let old = Detection { angle_deg: Some(90.0), grasp_width_px: Some(30.0), ..base.clone() };
        let new = Detection { depth_angle_deg: Some(90.0), depth_width_px: Some(30.0), ..base.clone() };
        let (mo, mn) = (measure_detection_depth_with_cal(&cam, &pose, &old, &cal).unwrap(), measure_detection_depth_with_cal(&cam, &pose, &new, &cal).unwrap());
        assert_eq!((mo.width_m, mo.length_m), (mn.width_m, mn.length_m));
        assert!(mn.width_m.is_some());
        // a segmentation model: the mask width decides the measurement's width, as before
        let seg = Detection { grasp_width_px: Some(20.0), angle_deg: Some(90.0), ..new.clone() };
        let ms = measure_detection_depth_with_cal(&cam, &pose, &seg, &cal).unwrap();
        let only_mask = Detection { depth_angle_deg: None, depth_width_px: None, ..seg.clone() };
        assert_eq!(ms, measure_detection_depth_with_cal(&cam, &pose, &only_mask, &cal).unwrap());
    }

    #[test]
    fn depth_calibration_and_measurement_report_why_they_fail() {
        let setup = GraspSetup { camera_tilt_deg: 0.0, fx_px: 600.0, fy_px: 600.0, image_size_px: [640.0, 360.0], ..Default::default() };
        let cam = CamModel::assumed(&crate::chain::ArmModel::REBOT_DEV_ARM, &setup, &CameraMount::default());
        let pose = Pose::new(look_down(&inspect_r()), [0.3, 0.0, 0.20]);
        let why = |d: Option<&DepthFrameInfo>| calibrate_depth_why(&cam, &pose, d, None, 0.0).err();
        assert_eq!(why(None), Some("no_depth_frame"));
        let metric = |samples: Vec<[f64; 3]>| DepthFrameInfo { metric_model: true, table_samples: samples };
        assert_eq!(why(Some(&metric(vec![]))), Some("too_few_samples"));
        assert_eq!(why(Some(&metric(vec![[320.0, 180.0, 10.0], [300.0, 160.0, 5.0], [340.0, 200.0, 2.5]]))), Some("table_contradicts_fk"));
        // metric 50 % off the prior (the simulator's --depth-scale 1.5): contradicts FK
        assert_eq!(why(Some(&metric(vec![[320.0, 180.0, 5.0 / 1.5], [300.0, 160.0, 5.0 / 1.5], [340.0, 200.0, 5.0 / 1.5]]))), Some("table_contradicts_fk"));
        assert_eq!(why(Some(&metric(vec![[320.0, 180.0, 5.0], [300.0, 160.0, 5.0], [340.0, 200.0, 5.0]]))), None);
        // relative: a flat look-down table spans no inverse depth
        let grid: Vec<[f64; 3]> = (0..12).map(|i| [100.0 + 40.0 * i as f64, 100.0 + 15.0 * i as f64, 2.0]).collect();
        let rel = |samples: Vec<[f64; 3]>| DepthFrameInfo { metric_model: false, table_samples: samples };
        assert_eq!(why(Some(&rel(grid.clone()))), Some("no_oblique_spread"));
        assert_eq!(why(Some(&rel(grid[..5].to_vec()))), Some("too_few_samples"));
        // the boolean API is unchanged
        assert!(calibrate_depth(&cam, &pose, Some(&rel(grid)), None, 0.0).is_none());

        let cal = DepthCalibration { a: 1.0, b: 0.0, source: "metric_table" };
        let ok = Detection { disp_fg: Some(6.25), disp_bg: Some(5.0), depth_conf: Some(0.95), ..det([0.45, 0.45, 0.55, 0.55], None) };
        let m = |d: Detection| measure_detection_depth_with_cal_why(&cam, &pose, &d, &cal).err();
        assert_eq!(m(ok.clone()), None);
        assert_eq!(m(Detection { depth_conf: Some(0.2), ..ok.clone() }), Some("low_conf"));
        assert_eq!(m(Detection { disp_fg: None, ..ok.clone() }), Some("no_disparity"));
        assert_eq!(m(Detection { disp_fg: Some(4.0), ..ok.clone() }), Some("bad_disparity"));
        assert_eq!(m(Detection { disp_fg: Some(5.001), ..ok.clone() }), Some("too_flat"));
    }

    struct DummyDetectionSource;
    impl DetectionSource for DummyDetectionSource {
        fn inference_enabled(&self) -> BoxFuture<'_, Result<bool, String>> { Box::pin(async { Ok(true) }) }
        fn set_inference(&self, _enabled: bool) -> BoxFuture<'_, Result<(), String>> { Box::pin(async { Ok(()) }) }
        fn next_frame(&self, _after: Option<u64>, _wait_ms: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> { Box::pin(async { Ok(None) }) }
    }

    #[tokio::test]
    async fn test_orchestrator_holding_now_load_and_gap_on_6dof() {
        let config = crate::model::ArmConfig::from_profile_file("profiles/so_arm101.json").expect("loads");
        let policy = config.gripper_policy.clone().expect("gripper_policy");

        let mut setup = GraspSetup::default();
        setup.post_lift_settle_ms = 10;

        let sup = Arc::new(ArmSupervisor::new(config));
        let (id, stamp) = {
            let mut inner = sup.inner().write().await;
            inner.lease_enabled = false;
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).unwrap();
            SupervisorInner::enable_torque(admitted).unwrap();
            inner.commanded_positions = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.5];
            inner.measured_positions = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.5];
            inner.loads = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.35];
            let stamp = inner.current_admission_stamp();
            let plan_in = inner.plan_input().unwrap();
            let admitted = inner.check_admission(stamp).unwrap();
            (SupervisorInner::begin_macro(admitted, "grasp", &plan_in, None).unwrap(), stamp)
        };

        let source = Arc::new(DummyDetectionSource);
        let cam = CamModel::assumed(&sup.inner().read().await.config.arm_model, &setup, &CameraMount::default());
        let ctx = MacroCtx {
            sup: sup.clone(),
            id,
            stamp,
            deadline: Instant::now() + Duration::from_secs(10),
            phase_timeout_s: setup.phase_timeout_s,
            timeout_code: "GRASP_TIMEOUT",
            timeout_message: "timeout".into(),
        };
        let orch = Orchestrator {
            ctx,
            source,
            setup: setup.clone(),
            cam,
            envelope: vec![],
            tcp_offset: [0.0; 3],
            triggered: false,
            lens_known: false,
            depth_available: false,
            generation: None,
        };

        let (holding, gap) = orch.holding_now().await.expect("holding_now should succeed");
        assert!(holding);
        assert!((gap - 0.5).abs() < 1e-4);

        // Test held_width calculation on 6-DOF
        let held_width = {
            let inner = sup.inner().read().await;
            inner.config.gripper_policy.as_ref().map(|p| {
                let idx = inner.config.gripper_index().unwrap_or(6);
                let m = inner.measured_positions.get(idx).copied().unwrap_or(0.0);
                p.opening_m(m)
            })
        };
        assert_eq!(held_width, Some(policy.opening_m(0.5)));
    }

    #[test]
    fn markers_of_a_trigger_reply() {
        let v = serde_json::json!([
            {"id": 3, "corners": [[10.0, 20.0], [30.0, 20.0], [30.0, 40.0], [10.0, 40.0]]},
            {"id": 4, "corners": [[1, 2], [3, 4], [5, 6]]},
            {"corners": [[1, 2], [3, 4], [5, 6], [7, 8]]},
            {"id": 5, "corners": [[1, 2], [3, 4], [5, 6], [7, "x"]]},
        ]);
        let m = parse_markers(&v);
        assert_eq!(m.len(), 1);
        assert_eq!((m[0].id, m[0].corners[2]), (3, [30.0, 40.0]));
        assert!(parse_markers(&Value::Null).is_empty());
    }

    fn build_rgf1(header: &Value, pixels: &[u8]) -> Vec<u8> {
        let mut js = header.to_string().into_bytes();
        while !(8 + js.len()).is_multiple_of(GRAY_FRAME_ALIGN) {
            js.push(b' ');
        }
        let mut out = Vec::with_capacity(8 + js.len() + pixels.len());
        out.extend_from_slice(GRAY_FRAME_MAGIC);
        out.extend_from_slice(&(js.len() as u32).to_le_bytes());
        out.extend_from_slice(&js);
        out.extend_from_slice(pixels);
        out
    }

    #[test]
    fn parse_gray_frame_decodes_cpp_golden_and_rejects_malformed() {
        // 1. Independently decode the C++-generated golden RGF1 fixture.
        let golden = include_bytes!("../tests/fixtures/gray_frame_golden.rgf1");
        let g = parse_gray_frame(golden, Some("rv-golden-0001"), Some(7)).expect("golden RGF1");
        assert_eq!(g.instance_id, "rv-golden-0001");
        assert_eq!(g.frame_id, 7);
        assert_eq!((g.img.w, g.img.h), (640, 360));
        assert_eq!(g.ts_ms, 1763791234567);
        assert_eq!((g.frame_mono_ms, g.wait_mono_ms, g.grab_mono_ms, g.stored_mono_ms), (123450, 123401, 123452, 123459));
        assert!((g.convert_ms - 4.9).abs() < 0.05);
        for y in 0..360usize {
            for x in 0..640usize {
                let expect = ((x * 3 + y * 5 + 17) & 0xFF) as u8;
                assert_eq!(g.img.px[y * 640 + x], expect, "pixel ({x},{y})");
            }
        }

        // 2. Identity mismatches
        assert!(matches!(parse_gray_frame(golden, Some("rv-other"), Some(7)), Err(GrayFrameError::Protocol(_))));
        assert!(matches!(parse_gray_frame(golden, Some("rv-golden-0001"), Some(8)), Err(GrayFrameError::Protocol(_))));

        // 3. Truncated / bad magic / extra payload / oversized / unaligned / bad padding
        assert!(matches!(parse_gray_frame(&golden[..6], None, None), Err(GrayFrameError::Protocol(_))));
        let mut bad_magic = golden.to_vec();
        bad_magic[0] = b'X';
        assert!(matches!(parse_gray_frame(&bad_magic, None, None), Err(GrayFrameError::Protocol(_))));
        assert!(matches!(parse_gray_frame(&golden[..golden.len() - 1], None, None), Err(GrayFrameError::Protocol(_))));
        let mut extra = golden.to_vec();
        extra.push(0);
        assert!(matches!(parse_gray_frame(&extra, None, None), Err(GrayFrameError::Protocol(_))));

        let mut oversized = Vec::from(&b"RGF1"[..]);
        oversized.extend_from_slice(&((GRAY_MAX_HEADER_BYTES as u32) + 8).to_le_bytes());
        assert!(matches!(parse_gray_frame(&oversized, None, None), Err(GrayFrameError::Protocol(_))));

        let mut unaligned = Vec::from(&b"RGF1"[..]);
        unaligned.extend_from_slice(&5u32.to_le_bytes());
        unaligned.extend_from_slice(b"{}   ");
        assert!(matches!(parse_gray_frame(&unaligned, None, None), Err(GrayFrameError::Protocol(_))));

        let px = vec![0x42u8; 640 * 360];
        let base_hdr = || serde_json::json!({
            "api_version": 2, "encoding": "RGF1", "format": "gray8",
            "instance_id": "rv-test1", "frame_id": 1,
            "width": 640, "height": 360, "stride": 640,
            "frame_size": [640, 640], "frame_format": "RGB888",
            "crop": [0, 140, 640, 360], "channel": 0,
            "luma": GRAY_V1_LUMA, "luma_range": GRAY_V1_LUMA_RANGE,
            "ts_ms": 1763791234567u64, "frame_mono_ms": 100u64,
            "wait_mono_ms": 95u64, "grab_mono_ms": 101u64, "stored_mono_ms": 105u64,
            "convert_ms": 4.2, "model_generation": 0u64,
        });
        assert!(parse_gray_frame(&build_rgf1(&base_hdr(), &px), Some("rv-test1"), Some(1)).is_ok());

        // Non-space trailing byte inside padded header
        let mut bad_pad = build_rgf1(&base_hdr(), &px);
        let hlen = u32::from_le_bytes([bad_pad[4], bad_pad[5], bad_pad[6], bad_pad[7]]) as usize;
        bad_pad[8 + hlen - 1] = b'\t';
        assert!(matches!(parse_gray_frame(&bad_pad, None, None), Err(GrayFrameError::Protocol(_))));

        // Missing required v1 metadata fields must all be rejected
        for missing_key in [
            "encoding", "format", "frame_format", "frame_size", "crop", "channel",
            "luma", "luma_range", "ts_ms", "frame_mono_ms", "wait_mono_ms",
            "grab_mono_ms", "stored_mono_ms", "convert_ms", "model_generation",
        ] {
            let mut h = base_hdr();
            h.as_object_mut().unwrap().remove(missing_key);
            assert!(
                matches!(parse_gray_frame(&build_rgf1(&h, &px), None, None), Err(GrayFrameError::Protocol(_))),
                "missing {missing_key} must fail"
            );
        }

        // Incompatible metadata, crop out-of-bounds/mismatch, negative timing, bad stride/dims
        for mut h in [
            { let mut h = base_hdr(); h["encoding"] = serde_json::json!("RGF2"); h },
            { let mut h = base_hdr(); h["frame_format"] = serde_json::json!("BGR888"); h },
            { let mut h = base_hdr(); h["luma"] = serde_json::json!("limited"); h },
            { let mut h = base_hdr(); h["luma_range"] = serde_json::json!("limited_16_235"); h },
            { let mut h = base_hdr(); h["frame_size"] = serde_json::json!([0, 640]); h },
            { let mut h = base_hdr(); h["crop"] = serde_json::json!([0, 300, 640, 360]); h }, // 300+360 > 640
            { let mut h = base_hdr(); h["crop"] = serde_json::json!([0, 140, 640, 350]); h }, // ch != height
            { let mut h = base_hdr(); h["crop"] = serde_json::json!([0, 140, 640]); h },      // wrong arity
            { let mut h = base_hdr(); h["convert_ms"] = serde_json::json!(-1.0); h },
            { let mut h = base_hdr(); h["stored_mono_ms"] = serde_json::json!(-5); h },
            { let mut h = base_hdr(); h["stored_mono_ms"] = serde_json::json!(100u64); h }, // stored before grab (101)
            { let mut h = base_hdr(); h["stride"] = serde_json::json!(648); h },
            { let mut h = base_hdr(); h["width"] = serde_json::json!(320); h["stride"] = serde_json::json!(320); h },
            { let mut h = base_hdr(); h["height"] = serde_json::json!(u64::MAX); h },
            { let mut h = base_hdr(); h["api_version"] = serde_json::json!(1); h },
            { let mut h = base_hdr(); h["format"] = serde_json::json!("rgb888"); h },
        ] {
            let body = build_rgf1(&h, &px);
            assert!(matches!(parse_gray_frame(&body, None, None), Err(GrayFrameError::Protocol(_))), "{h}");
            h["width"] = serde_json::json!(0);
            assert!(matches!(parse_gray_frame(&build_rgf1(&h, &[]), None, None), Err(GrayFrameError::Protocol(_))));
        }

        // Negotiated format mismatch (frame_size / crop differs from negotiated /api/mode descriptor)
        let negotiated = GrayFormatDescriptor {
            encoding: "RGF1".into(), format: "gray8".into(),
            width: 640, height: 360, stride: 640, ring: 4, exposure_qualified: false,
            frame_size: Some([640, 640]), crop: Some([0, 140, 640, 360]),
            luma: Some(GRAY_V1_LUMA.into()), luma_range: Some(GRAY_V1_LUMA_RANGE.into()),
        };
        assert!(negotiated.is_supported_v1());
        assert!(parse_gray_frame_with_format(&build_rgf1(&base_hdr(), &px), Some("rv-test1"), Some(1), Some(&negotiated)).is_ok());
        let mut diff_crop = base_hdr();
        diff_crop["crop"] = serde_json::json!([0, 100, 640, 360]);
        assert!(matches!(
            parse_gray_frame_with_format(&build_rgf1(&diff_crop, &px), None, None, Some(&negotiated)),
            Err(GrayFrameError::Protocol(m)) if m.contains("crop")
        ));
        let mut diff_fs = base_hdr();
        diff_fs["frame_size"] = serde_json::json!([1280, 720]);
        assert!(matches!(
            parse_gray_frame_with_format(&build_rgf1(&diff_fs, &px), None, None, Some(&negotiated)),
            Err(GrayFrameError::Protocol(m)) if m.contains("frame_size")
        ));
    }

    #[test]
    fn mode_info_parses_gray_and_aruco_and_upstream_loopback() {
        let m = ModeInfo::parse(Some(&serde_json::json!({
            "mode": "yolo", "enabled": false, "per_request_pipelines": true,
            "available": {"yolo": true, "aruco": true, "depth": false, "gray": true},
            "gray_frames": {
                "encoding": "RGF1", "format": "gray8", "width": 640, "height": 360, "stride": 640,
                "frame_size": [640, 640], "crop": [0, 140, 640, 360],
                "luma": GRAY_V1_LUMA, "luma_range": GRAY_V1_LUMA_RANGE,
                "ring": 4, "exposure_qualified": false
            }
        })));
        assert!(m.per_request_pipelines && m.aruco_available && m.gray_available);
        assert_eq!(m.gray_format, Ok(Some(GrayFormatDescriptor {
            encoding: "RGF1".into(), format: "gray8".into(), width: 640, height: 360, stride: 640, ring: 4, exposure_qualified: false,
            frame_size: Some([640, 640]), crop: Some([0, 140, 640, 360]),
            luma: Some(GRAY_V1_LUMA.into()), luma_range: Some(GRAY_V1_LUMA_RANGE.into()),
        })));
        assert!(m.gray_format.as_ref().unwrap().as_ref().unwrap().is_supported_v1());
        let old = ModeInfo::parse(Some(&serde_json::json!({"mode": "yolo", "per_request_pipelines": true, "available": {"yolo": true, "aruco": true}})));
        assert!(old.aruco_available && !old.gray_available && old.gray_format == Ok(None));
        // An advertised but malformed descriptor is an error, not "absent".
        let good = serde_json::json!({
            "encoding": "RGF1", "format": "gray8", "width": 640, "height": 360, "stride": 640,
            "frame_size": [640, 640], "crop": [0, 140, 640, 360], "ring": 4
        });
        for (key, bad) in [
            ("width", serde_json::json!("640")), ("crop", serde_json::json!([0, 140, 640])),
            ("frame_size", serde_json::json!(null)), ("luma", serde_json::json!(1)), ("ring", serde_json::json!(-1)),
            ("exposure_qualified", serde_json::json!("yes")), ("exposure_qualified", serde_json::json!(1)),
            ("luma_range", serde_json::json!(null)),
        ] {
            let mut g = good.clone();
            g[key] = bad;
            let m = ModeInfo::parse(Some(&serde_json::json!({"mode": "yolo", "per_request_pipelines": true,
                "available": {"gray": true}, "gray_frames": g})));
            assert!(m.gray_format.is_err(), "{key}: {:?}", m.gray_format);
        }
        let mut no_enc = good.clone();
        no_enc.as_object_mut().unwrap().remove("encoding");
        for g in [no_enc, serde_json::json!(null), serde_json::json!([1, 2])] {
            let m = ModeInfo::parse(Some(&serde_json::json!({"mode": "yolo", "per_request_pipelines": true,
                "available": {"gray": true}, "gray_frames": g})));
            assert!(m.gray_format.is_err(), "{:?}", m.gray_format);
        }

        for lo in ["http://127.0.0.1:8090", "http://127.0.0.2:8090", "http://localhost:8090", "http://[::1]:8090"] {
            assert!(DetectorUpstream::parse(lo).unwrap().is_loopback(), "{lo}");
        }
        for rem in ["http://192.168.1.50:8090", "http://10.0.0.2:8090", "http://recamera.local:8090"] {
            assert!(!DetectorUpstream::parse(rem).unwrap().is_loopback(), "{rem}");
        }
    }

    #[test]
    fn table_z_from_tcp_requires_vertical_closed_known_contact_and_measured_fk() {
        let mut cfg = crate::ArmConfig::rebot_dm();
        cfg.active_tool.is_stock = false;
        cfg.active_tool.id = "custom_tool".into();
        cfg.active_tool.config_digest = "v1:test".into();
        cfg.physical_contact_end_m = None;
        let mut sup = SupervisorInner::new(cfg);

        // 1. Missing physical_contact_end_m -> rejected
        let err = prepare_setup_update(&sup, &serde_json::json!({"table_z_from_tcp": true}), None, false)
            .unwrap_err();
        assert!(format!("{err:?}").contains("physical_contact_end_m"));

        // 2. Vertical pose (+X_end pointing -Z_base) with open gripper -> rejected
        sup.config.physical_contact_end_m = Some([0.0288, 0.0, 0.0]);
        let target_r = kinematics::rpy_to_r(0.0, std::f64::consts::FRAC_PI_2, 0.0);
        let q_seed = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0];
        let p_end = sup.config.arm_model.fk(&q_seed).p;
        let (lo, hi): ([f64; 6], [f64; 6]) = (
            std::array::from_fn(|i| f64::from(sup.config.limits[i].min_position)),
            std::array::from_fn(|i| f64::from(sup.config.limits[i].max_position)),
        );
        let sol = sup.config.arm_model.solve_dls(
            &kinematics::Pose::new(target_r, p_end),
            &q_seed,
            &lo,
            &hi,
            kinematics::PAR.w,
        );
        assert!(sol.ok);
        let q_vert: Vec<f32> = sol.q.iter().map(|&v| v as f32).chain([-3.0]).collect();
        sup.measured_positions = q_vert.clone();
        sup.commanded_positions = vec![0.0; 7];
        let err = prepare_setup_update(&sup, &serde_json::json!({"table_z_from_tcp": true}), None, false)
            .unwrap_err();
        assert!(format!("{err:?}").contains("closed"));

        // 3. Tilted > 5 deg with closed gripper -> rejected
        sup.measured_positions = vec![0.0, -1.167, -1.565, 1.0, 0.0, 0.0, 0.0];
        let err = prepare_setup_update(&sup, &serde_json::json!({"table_z_from_tcp": true}), None, false)
            .unwrap_err();
        assert!(format!("{err:?}").contains("5 deg"));

        // 4. Vertical + closed: uses measured_positions FK + physical_contact_end_m (not commanded_positions)
        let mut q_ok = q_vert;
        q_ok[6] = 0.0;
        sup.measured_positions = q_ok.clone();
        let meas_end = sup.config.arm_model.fk_f32(&q_ok).unwrap();
        let expected_z = (meas_end.transform_point(&[0.0288, 0.0, 0.0])[2] * 1e4).round() / 1e4;
        let prep = prepare_setup_update(&sup, &serde_json::json!({"table_z_from_tcp": true}), None, false)
            .expect("valid vertical closed pose must compute table_z_m");
        assert!((prep.setup.table_z_m - expected_z).abs() < 1e-6);
    }
}
