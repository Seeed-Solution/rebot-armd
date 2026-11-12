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
use crate::model::{CameraMount, WorkspacePolicy};
use crate::macro_action::{fail, MacroCtx, Stop, SubDone};
use crate::motion::{self, GoalSpec, GripperCommand, MotionError, PlanInput, RelFrame, Speed};
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
    /// size and move back; 0 disables.
    pub height_probe_m: f64,
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
            approach_speed_mps: 0.05,
            descend_speed_mps: 0.03,
            phase_timeout_s: 25.0,
            timeout_s: 120.0,
            acquire_camera_down: AcquireCameraDown::Auto,
            recheck_tol_px: 40.0,
            height_probe_m: 0.10,
            height_grasp_fraction: 0.5,
            max_grasp_height_m: 0.06,
            recentre_height_m: 0.0,
            recentre_max_iterations: 3,
            descend_contact_stop_nm: None,
            post_lift_settle_ms: 300,
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
            self.align_min_aspect, self.align_min_angle_conf, self.approach_speed_mps, self.descend_speed_mps, self.phase_timeout_s, self.timeout_s, self.recheck_tol_px,
            self.height_probe_m, self.height_grasp_fraction, self.max_grasp_height_m, self.recentre_height_m];
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
        check((0.0..=1.0).contains(&self.height_grasp_fraction), "height_grasp_fraction must be in [0, 1]")?;
        check(self.max_grasp_height_m >= self.grasp_height_m && self.max_grasp_height_m <= 0.2,
            "max_grasp_height_m must be in [grasp_height_m, 0.2] m")?;
        check((0.0..=0.4).contains(&self.recentre_height_m), "recentre_height_m must be in [0, 0.4] m")?;
        check(self.post_lift_settle_ms <= 5000, "post_lift_settle_ms must be at most 5000")?;
        check((1..=30).contains(&self.recentre_max_iterations), "recentre_max_iterations must be 1..30")?;
        check(self.descend_contact_stop_nm.is_none_or(|v| v.is_finite() && v > 0.0 && v <= 50.0), "descend_contact_stop_nm must be null or in (0, 50] N·m")?;
        check(self.align_min_aspect >= 1.0 && self.align_min_aspect <= 10.0, "align_min_aspect must be in [1, 10]")?;
        check((0.0..=1.0).contains(&self.align_min_angle_conf), "align_min_angle_conf must be in [0, 1]")?;
        let speed_ok = |v: f64| v > 0.0 && v <= motion::MAX_LINEAR_MPS;
        check(speed_ok(self.approach_speed_mps) && speed_ok(self.descend_speed_mps), "speeds must be in (0, 0.15] m/s")?;
        check(self.phase_timeout_s > 0.0 && self.phase_timeout_s <= 120.0 && self.timeout_s >= self.phase_timeout_s && self.timeout_s <= 600.0,
            "phase_timeout_s in (0, 120] s, timeout_s in [phase_timeout_s, 600] s")?;
        Ok(())
    }

    /// Apply a JSON object of field overrides; unknown keys and invalid results are rejected.
    pub fn patched(&self, patch: &Value) -> Result<Self, String> {
        let Some(obj) = patch.as_object() else { return Err("grasp setup must be a JSON object".into()); };
        let mut base = serde_json::to_value(self).map_err(|e| e.to_string())?;
        let fields = base.as_object_mut().expect("setup serialises to an object");
        for (k, v) in obj {
            if !fields.contains_key(k) { return Err(format!("unknown grasp setup field: {k}")); }
            fields.insert(k.clone(), v.clone());
        }
        let out: Self = serde_json::from_value(base).map_err(|e| format!("invalid grasp setup: {e}"))?;
        out.validate()?;
        Ok(out)
    }
}

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

/// Load the persisted setup into the supervisor at startup (logs the outcome).
pub async fn load_into(supervisor: &ArmSupervisor, path: &Path) {
    let mut inner = supervisor.inner().write().await;
    let Some(base) = inner.grasp_setup() else { return; };
    let (setup, outcome) = load_setup(path, &base);
    match &outcome {
        LoadOutcome::Loaded => tracing::info!("Grasp setup loaded from {}", path.display()),
        LoadOutcome::Missing => tracing::info!("No grasp setup at {}; using defaults", path.display()),
        LoadOutcome::Invalid(e) => tracing::warn!("Ignoring grasp setup file: {e}; using defaults"),
    }
    inner.set_grasp_setup(setup);
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

/// WS command `/api/v1/grasp/setup`: apply a patch (and `table_z_from_tcp: true`, which sets
/// `table_z_m` to the current commanded TCP height). Returns the new setup.
pub fn update_setup(inner: &mut SupervisorInner, body: &Value) -> Result<GraspSetup, ArmError> {
    let Some(base) = inner.grasp_setup() else {
        return Err(MotionError::Unsupported("This profile has no grasp setup".into()).into());
    };
    let Some(obj) = body.as_object() else { return Err(MotionError::InvalidCommand("grasp setup must be a JSON object".into()).into()); };
    let mut patch = obj.clone();
    match patch.remove("table_z_from_tcp") {
        None | Some(Value::Null) | Some(Value::Bool(false)) => {}
        Some(Value::Bool(true)) => {
            let Some(pose) = inner.config.arm_model.fk_f32(&inner.commanded_positions) else {
                return Err(MotionError::InvalidCommand("commanded positions are not a valid joint vector".into()).into());
            };
            let z = pose.p[2];
            patch.insert("table_z_m".into(), serde_json::json!((z * 1e4).round() / 1e4));
        }
        Some(_) => return Err(MotionError::InvalidCommand("table_z_from_tcp must be boolean".into()).into()),
    }
    let setup = base.patched(&Value::Object(patch)).map_err(MotionError::InvalidCommand)?;
    // the camera tilt is workspace geometry (camera check point): it cannot change under an
    // accepted motion; other setup fields can
    if setup.camera_tilt_deg as f32 != inner.config.camera_mount.tilt_deg {
        if let Some(reason) = inner.workspace_change_blocker() {
            return Err(MotionError::MotionBusy(format!("camera_tilt_deg cannot change while {reason}")).into());
        }
    }
    inner.set_grasp_setup(setup.clone());
    Ok(setup)
}

// ---- detections ----

#[derive(Debug, Clone, PartialEq)]
pub struct Detection {
    pub label: String,
    pub score: f64,
    /// Normalised [x0, y0, x1, y1].
    pub box_norm: [f64; 4],
    /// Orientation of the object's long side in the image (deg, from +x toward +y, y down,
    /// wrapped to [-90, 90)) and its confidence [0, 1], when the detector reports them.
    pub angle_deg: Option<f64>,
    pub angle_conf: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct DetectionFrame {
    pub frame_id: u64,
    /// Capture time on this host's clock (response time − `age_ms`).
    pub captured_at: Instant,
    pub detections: Vec<Detection>,
}

/// A provider's vision mode (`GET /api/mode` of rebot-vision).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VisionMode {
    /// `off`, `yolo`, `aruco` or `both`.
    pub mode: String,
    /// Continuous inference on.
    pub enabled: bool,
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
const TRIGGER_WAIT_MS: u64 = 1000;

/// Where the orchestrator gets detections; tests inject a simulated camera.
pub trait DetectionSource: Send + Sync {
    /// The provider runs the pipelines a trigger names (perception API 2). The grasp then
    /// triggers [`GRASP_PIPELINES`] per centring step and leaves the vision mode alone.
    fn per_request_pipelines(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async { Ok(false) })
    }
    /// One inference running exactly `pipelines` on a frame newer than the call, waiting up to
    /// `wait_ms`. `Ok(None)` = no fresh frame in time.
    fn trigger(&self, _pipelines: &'static [&'static str], _wait_ms: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        Box::pin(async { Err("per-request pipelines not supported".to_string()) })
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
    /// The next frame with `frame_id > after` (any frame when `after` is None), waiting up to
    /// `wait_ms`. `Ok(None)` = no new frame in time.
    fn next_frame(&self, after: Option<u64>, wait_ms: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>>;
}

/// The camera app's detection API (the same upstream armd proxies under /api/v1/detection).
pub struct HttpDetectionSource {
    pub upstream: DetectorUpstream,
    /// Size used for detections reported in pixels when the frame carries no size.
    pub image_size_px: [f64; 2],
}

impl HttpDetectionSource {
    /// A result envelope (`latest` or `trigger`) as a frame; `None` when it carries no new frame.
    fn frame(&self, v: &Value, after: Option<u64>, received: Instant) -> Option<DetectionFrame> {
        if v.get("timeout").and_then(Value::as_bool) == Some(true) || v.get("success").and_then(Value::as_bool) == Some(false) {
            return None;
        }
        let frame_id = v.get("frame_id").and_then(Value::as_u64)?;
        if after.is_some_and(|a| frame_id <= a) { return None; }
        let age = v.get("age_ms").and_then(Value::as_f64).filter(|a| a.is_finite() && *a >= 0.0).unwrap_or(0.0);
        let captured_at = received.checked_sub(Duration::from_secs_f64(age / 1000.0)).unwrap_or(received);
        let size = match (v.get("width").and_then(Value::as_f64), v.get("height").and_then(Value::as_f64)) {
            (Some(w), Some(h)) if w > 0.0 && h > 0.0 => [w, h],
            _ => self.image_size_px,
        };
        Some(DetectionFrame { frame_id, captured_at, detections: parse_detections(v.get("detections").unwrap_or(&Value::Null), size) })
    }
}

impl DetectionSource for HttpDetectionSource {
    fn per_request_pipelines(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async move {
            let v = crate::api::proxy::request_json_opt(&self.upstream, "GET", "/api/mode", None, Duration::from_secs(2)).await?;
            Ok(v.and_then(|v| v.get("per_request_pipelines").and_then(Value::as_bool)) == Some(true))
        })
    }
    fn trigger(&self, pipelines: &'static [&'static str], wait_ms: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        Box::pin(async move {
            let wait_ms = wait_ms.min(5000);
            let body = serde_json::json!({"pipelines": pipelines, "wait_ms": wait_ms}).to_string();
            let v = crate::api::proxy::request_json(&self.upstream, "POST", "/api/trigger", Some(body.as_bytes()), Duration::from_millis(wait_ms + 2000)).await?;
            Ok(self.frame(&v, None, Instant::now()))
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
    fn next_frame(&self, after: Option<u64>, wait_ms: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        Box::pin(async move {
            let wait_ms = wait_ms.min(5000);
            let path = format!("/api/results/latest?after={}&wait_ms={wait_ms}", after.unwrap_or(0));
            let v = crate::api::proxy::request_json(&self.upstream, "GET", &path, None, Duration::from_millis(wait_ms + 2000)).await?;
            Ok(self.frame(&v, after, Instant::now()))
        })
    }
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
        let (label, score, cx, cy, w, h) = if let Some(a) = item.as_array().filter(|a| a.len() >= 5) {
            let n = |i: usize| a.get(i).and_then(Value::as_f64).filter(|x| x.is_finite());
            let (Some(cx), Some(cy), Some(w), Some(h), Some(s)) = (n(0), n(1), n(2), n(3), n(4)) else { continue };
            let label = a.get(5).map(|l| l.as_str().map(str::to_string).unwrap_or_else(|| l.to_string())).unwrap_or_else(|| "obj".into());
            (label, s, cx, cy, w, h)
        } else if item.is_object() {
            let label = ["label", "name", "target", "class_name"].iter()
                .find_map(|k| item.get(*k).and_then(Value::as_str)).unwrap_or("obj").to_string();
            let score = f(item, &["score", "confidence"]).unwrap_or(0.0);
            if let Some(b) = item.get("box_norm").and_then(Value::as_array).filter(|b| b.len() == 4) {
                let b: Vec<f64> = b.iter().filter_map(Value::as_f64).collect();
                if b.len() != 4 { continue; }
                (label, score, (b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0, b[2] - b[0], b[3] - b[1])
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
                        (label, score, (x0 + x1) / 2.0, (y0 + y1) / 2.0, x1 - x0, y1 - y0)
                    }
                    Some("xywh_center") => {
                        let (Some(cx), Some(cy), Some(w), Some(h)) = (f(item, &["cx", "x"]), f(item, &["cy", "y"]), f(item, &["w", "width"]), f(item, &["h", "height"])) else { continue };
                        (label, score, cx, cy, w, h)
                    }
                    Some("xywh_top_left") => {
                        let (x0, y0, w, h) = match &bb {
                            Some(b) => (b[0], b[1], b[2], b[3]),
                            None => match (f(item, &["x", "x0"]), f(item, &["y", "y0"]), f(item, &["w", "width"]), f(item, &["h", "height"])) {
                                (Some(a), Some(b), Some(c), Some(d)) => (a, b, c, d),
                                _ => continue,
                            },
                        };
                        (label, score, x0 + w / 2.0, y0 + h / 2.0, w, h)
                    }
                    // rebot-vision 1.2 (perception API 1) sent centre x, y, w, h without `format`
                    None if bb.is_none() => {
                        let (Some(cx), Some(cy), Some(w), Some(h)) = (f(item, &["cx", "x"]), f(item, &["cy", "y"]), f(item, &["w"]), f(item, &["h"])) else { continue };
                        (label, score, cx, cy, w, h)
                    }
                    _ => continue,
                }
            }
        } else { continue };
        if !(w > 0.0 && h > 0.0 && score.is_finite()) { continue; }
        let pixel = cx > 1.05 || cy > 1.05 || w > 1.05 || h > 1.05;
        let (sx, sy) = if pixel { (size[0], size[1]) } else { (1.0, 1.0) };
        let b = [(cx - w / 2.0) / sx, (cy - h / 2.0) / sy, (cx + w / 2.0) / sx, (cy + h / 2.0) / sy].map(|v| v.clamp(0.0, 1.0));
        if b[2] <= b[0] || b[3] <= b[1] { continue; }
        let angle_deg = item.is_object().then(|| f(item, &["angle_deg"])).flatten().map(|a| (a + 90.0).rem_euclid(180.0) - 90.0);
        let angle_conf = item.is_object().then(|| f(item, &["angle_conf"])).flatten().map(|c| c.clamp(0.0, 1.0));
        out.push(Detection { label, score: score.clamp(0.0, 1.0), box_norm: b, angle_deg, angle_conf });
    }
    out
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
/// predicted centre or IoU above `match_iou` with the predicted box.
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
        frame.detections.iter()
            .filter(|d| d.label.eq_ignore_ascii_case(&self.label))
            .filter_map(|d| {
                let c = box_centre(&d.box_norm);
                let dist = ((c[0] - pc[0]) * w).hypot((c[1] - pc[1]) * h);
                (dist <= self.gate_px || iou(&d.box_norm, &self.predicted) > setup.match_iou).then_some((dist, d))
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
    /// The controller's model: mount flip / rot from the config, tilt and rough intrinsics from
    /// the setup, optical centre at `camera_mount.offset_m` (end_link origin when unset).
    pub fn assumed(arm: &crate::chain::ArmModel, setup: &GraspSetup, mount: &CameraMount) -> Self {
        Self {
            rec: arm.cam_mount(mount.flip, f64::from(mount.rot_deg), setup.camera_tilt_deg),
            offset: mount.offset_m.map(|o| o.map(f64::from)).unwrap_or([0.0; 3]),
            fx: setup.fx_px, fy: setup.fy_px,
            cx: setup.image_size_px[0] / 2.0, cy: setup.image_size_px[1] / 2.0,
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
            .map(|(point, z, _)| MotionError::WorkspaceViolation { s, point, limit: "table_z_m + 0.005".into(), value: z, bound })
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
    pub setup: GraspSetup,
    pub tag: Option<String>,
}

/// Body of `/api/v1/action/grasp`: `{label, box_norm, frame_id?, ts_ms?, overrides?, tag?}`.
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
    let setup = match body.get("overrides") {
        None | Some(Value::Null) => base.clone(),
        Some(o) => base.patched(o).map_err(MotionError::InvalidCommand)?,
    };
    let tag = match body.get("tag") {
        None | Some(Value::Null) => None,
        Some(Value::String(s)) if s.len() <= 64 => Some(s.clone()),
        _ => return Err(bad("tag must be a string of at most 64 bytes")),
    };
    Ok(GraspRequest { label, box_norm, frame_id, setup, tag })
}

/// A request checked under the lock, ready to be planned outside it.
pub struct PendingGrasp {
    pub input: PlanInput,
    /// Boxed: the setup makes the request large.
    pub request: Box<GraspRequest>,
    pub open_rad: f32,
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
}

/// Under the lock: parse and check preconditions.
pub fn prepare(inner: &SupervisorInner, body: &Value, epoch: u64) -> Result<PendingGrasp, ArmError> {
    if !body.is_object() { return Err(MotionError::InvalidCommand("Command body must be an object".into()).into()); }
    let Some(base) = inner.grasp_setup() else {
        return Err(MotionError::Unsupported("Grasp needs a 7-DOF profile with kinematics, a gripper policy and a grasp setup".into()).into());
    };
    let Some(policy) = inner.config.gripper_policy.clone() else {
        return Err(MotionError::Unsupported("Grasp needs a gripper policy".into()).into());
    };
    let request = parse_grasp(body, &base)?;
    if epoch != inner.safety.epoch_id() {
        return Err(crate::safety::SafetyError::EpochMismatch { expected: inner.safety.epoch_id(), received: epoch }.into());
    }
    let input = inner.plan_input()?;
    Ok(PendingGrasp { input, request: Box::new(request), open_rad: policy.open_rad })
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
    let PendingGrasp { input, request, open_rad } = pending;
    let request = *request;
    let setup = &request.setup;
    let pose = input.arm.fk(&input.q0);
    let cam = CamModel::assumed(&input.arm, setup, &input.cam);
    let centre = cam.to_px(box_centre(&request.box_norm));
    let target = cam.plane_point(&pose, centre, grasp_plane(setup)).ok_or_else(|| ArmError::from(MotionError::InvalidCommand(
        "the box centre ray does not meet the table plane in front of the camera".into())))?;
    let camera_down = camera_down_mode(setup, &input.cam).map_err(MotionError::InvalidCommand)?;
    let pts = envelope(input.ws.as_ref(), &input.cam, &cam.rec);
    // the envelope stays above the table on the approach and (camera-down) the transition
    let goal = if camera_down {
        let goal = camera_down_pose(&pose.r, &target, setup, &cam, &pts);
        let tool = approach_pose(&goal.r, &target, setup);
        if let Some(e) = table_violation(&goal, &tool, &pts, setup.table_z_m) { return Err(e.into()); }
        goal
    } else {
        approach_pose(&pose.r, &target, setup)
    };
    if let Some(e) = table_violation(&pose, &goal, &pts, setup.table_z_m) { return Err(e.into()); }
    let mut open_input = input.clone();
    open_input.j7 = open_rad;
    let path = motion::plan_cartesian(&open_input, goal, Speed { linear: setup.approach_speed_mps, angular: motion::DEFAULT_ANGULAR_RPS })?;
    let transition_duration = if camera_down { transition_estimate(&goal, &target, setup) } else { 0.0 };
    Ok(PlannedGrasp { input, request, target, approach_duration: path.duration, camera_down, transition_duration })
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
    let probe = if s.height_probe_m > 0.0 { 2.0 * s.height_probe_m * 1.875 / s.approach_speed_mps + 1.5 } else { 0.0 };
    let recentre = if s.recentre_height_m > 0.0 && !planned.camera_down { 3.0 } else { 0.0 };
    let lift_check = s.post_lift_settle_ms as f64 / 1000.0;
    let base = lift_check + 6.5 + planned.approach_duration + 4.0 * 1.5 + descend + 2.0 + lift + probe + recentre;
    // camera-down: the transition move and its re-check frame
    if planned.camera_down { base + planned.transition_duration + 1.5 } else { base }
}

/// Start a grasp without a WS lease check (tests, in-process agents): prepare, plan, begin
/// and spawn the orchestrator. Returns (action id, expected duration).
pub async fn start(supervisor: &Arc<ArmSupervisor>, source: Arc<dyn DetectionSource>, body: &Value) -> Result<(u64, f64), ApiError> {
    let pending = {
        let inner = supervisor.inner().read().await;
        prepare(&inner, body, inner.safety.epoch_id())?
    };
    let planned = tokio::task::spawn_blocking(move || plan(pending)).await
        .map_err(|e| ArmError::Internal(format!("planner task failed: {e}")))??;
    let id = supervisor.inner().write().await.begin_macro("grasp", &planned.input, planned.request.tag.clone())?;
    let duration = expected_duration(&planned);
    spawn(supervisor.clone(), source, id, planned);
    Ok((id, duration))
}

/// Spawn the orchestrator for an action begun with `SupervisorInner::begin_macro("grasp", ..)`.
pub fn spawn(supervisor: Arc<ArmSupervisor>, source: Arc<dyn DetectionSource>, id: u64, planned: PlannedGrasp) {
    tokio::spawn(run(supervisor, source, id, planned));
}

// ---- orchestrator ----

const POLL: Duration = crate::macro_action::POLL;

/// Grasp phases on top of the shared macro-action primitives ([`MacroCtx`], via `Deref`).
struct Orchestrator {
    ctx: MacroCtx,
    source: Arc<dyn DetectionSource>,
    setup: GraspSetup,
    cam: CamModel,
    /// Points kept above the table on every sub-motion ([`envelope`]).
    envelope: Vec<Vec3>,
    /// Each centring step triggers its own frame (perception API 2) instead of reading the
    /// continuous results.
    triggered: bool,
    /// `camera_mount.offset_m` is set: the lens position (height probe) is known.
    lens_known: bool,
}

impl std::ops::Deref for Orchestrator {
    type Target = MacroCtx;
    fn deref(&self) -> &MacroCtx { &self.ctx }
}

impl Orchestrator {
    /// Every grasp sub-motion: WORKSPACE_VIOLATION before anything is installed when its
    /// straight path from the commanded pose takes the envelope below the table clearance.
    async fn move_arm(&self, phase: &str, spec: GoalSpec, linear: f64) -> Result<SubDone, Stop> {
        let input = {
            let inner = self.sup.inner().read().await;
            if !inner.macro_alive(self.id) { return Err(Stop::Cancelled); }
            inner.plan_input().map_err(crate::macro_action::arm_fail)?
        };
        let goal = motion::resolve_goal(&input, &spec);
        if let Some(e) = table_violation(&input.arm.fk(&input.q0), &goal, &self.envelope, self.setup.table_z_m) {
            return Err(crate::macro_action::arm_fail(e.into()));
        }
        self.ctx.move_arm(phase, spec, Speed { linear, angular: motion::DEFAULT_ANGULAR_RPS }).await
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
            let get = async move {
                if triggered {
                    // the triggered frame is taken once the arm has settled
                    tokio::time::sleep(not_before.saturating_duration_since(Instant::now())).await;
                    source.trigger(GRASP_PIPELINES, TRIGGER_WAIT_MS).await
                } else {
                    source.next_frame(a, 200).await
                }
            };
            let next = tokio::select! {
                r = get => r,
                _ = async { loop { tokio::time::sleep(POLL).await; if !self.alive().await { break; } } } => return Err(Stop::Cancelled),
            };
            let frame = match next {
                Ok(Some(frame)) => frame,
                Ok(None) => {
                    if last_frame.elapsed().as_secs_f64() > self.setup.lost_timeout_s.max(3.0) {
                        return Err(fail("TARGET_LOST", "No detection frames arrived"));
                    }
                    continue;
                }
                Err(e) => return Err(fail("DETECTION_UNAVAILABLE", format!("Detection source failed: {e}"))),
            };
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
        self.phase("approach", 0.1).await?;
        let pose0 = self.pose().await;
        let target = planned.target;
        let goal = if camera_down { camera_down_pose(&pose0.r, &target, s, &self.cam, &self.envelope) } else { approach_pose(&pose0.r, &target, s) };
        if camera_down {
            let height = goal.p[2] - s.table_z_m;
            if height > s.approach_height_m + 1e-6 {
                tracing::info!("Grasp {}: camera-down acquisition at {height:.3} m (approach_height_m {:.3}) for table clearance", self.id, s.approach_height_m);
                self.sup.inner().write().await.set_action_acquire_height(self.id, height as f32);
            }
        }
        let done = self.move_arm("approach", GoalSpec::Absolute { p: goal.p, r: Some(goal.r) }, s.approach_speed_mps).await?;

        // closed-loop centring onto the grip pixel, or (camera-down) the image centre
        self.phase("centring", 0.3).await?;
        let grip_px = self.cam.to_px(s.grip_pixel_norm);
        let (aim_px, aim_name) = if camera_down { ([self.cam.cx, self.cam.cy], "the image centre") } else { (grip_px, "the grip point") };
        let plane = grasp_plane(s);
        let pose = self.pose().await;
        let predicted = self.cam.project(&pose, &target).map(|p| self.cam.to_norm(p)).unwrap_or(s.grip_pixel_norm);
        let mut tracker = Tracker { label: planned.request.label.clone(), predicted: planned.request.box_norm, gate_px: s.acquire_dist_px };
        tracker.predict_at(predicted);
        let mut after = None;
        let c = self.centre("centring", 0.3, &mut tracker, aim_px, aim_name, plane, done.at, &mut after, s.max_iterations, true).await?;
        let last = c.last.clone();

        // object height from the box scale change over a short straight-down probe: box
        // centres map onto the object's top plane, the jaws close part way up the object
        let mut close_h = s.grasp_height_m;
        let mut plane = plane;
        match self.probe_height(&c, &planned.request.label, &mut after).await? {
            Ok(h) => {
                plane = s.table_z_m + s.grasp_height_m.max(h);
                close_h = (h * s.height_grasp_fraction).clamp(s.grasp_height_m, s.max_grasp_height_m);
                tracing::info!("Grasp {}: object top {h:.3} m above the table (scale), closing at {close_h:.3} m", self.id);
                self.sup.inner().write().await.update_action(self.id, |a| {
                    a.height_source = Some("scale".into());
                    a.object_height_m = Some(h as f32);
                });
            }
            Err(reason) => self.sup.inner().write().await.update_action(self.id, |a| {
                a.height_source = Some("setup".into());
                a.height_reason = Some(reason);
            }),
        }
        self.sup.inner().write().await.update_action(self.id, |a| a.close_height_m = Some(close_h as f32));

        // camera-down: re-orient to the tool-vertical pose above the target's table point
        if camera_down {
            self.phase("transition", 0.5).await?;
            // the ray of the last centring box from the pose it was seen at (before the probe)
            let Some((t, goal)) = transition_pose(&c.pose, self.cam.to_px(box_centre(&last.box_norm)), plane, s, &self.cam) else {
                return Err(fail("CENTRING_FAILED", "The target ray does not meet the table plane"));
            };
            let done = self.move_arm("transition", GoalSpec::Absolute { p: goal.p, r: Some(goal.r) }, s.approach_speed_mps).await?;
            // verify the target where the model expects it before descending
            self.recheck(&planned.request.label, &last, &t, grip_px, done.at, &mut after).await?;
        }

        // J6 alignment: close the jaws across the object's short side
        self.phase("align", 0.55).await?;
        let (turn, align) = if s.align_j6 { self.align_turn(&c, plane).await } else { (None, "off".to_string()) };
        self.sup.inner().write().await.update_action(self.id, |a| { a.align = Some(align); a.align_turn_rad = turn.map(|t| t as f32); });
        if let Some(turn) = turn {
            self.move_arm("align", GoalSpec::Relative { frame: RelFrame::Tool, d: [0.0; 3], dyaw: 0.0, dpitch: 0.0, droll: turn, level: false }, s.approach_speed_mps).await?;
        }

        // descend (look-down: re-centre at mid height on the way), close, check
        self.phase("descend", 0.65).await?;
        if !camera_down {
            let (recentre, iterations) = self.recentre(&c, plane, close_h, &planned.request.label, &mut after).await?;
            self.sup.inner().write().await.update_action(self.id, |a| { a.recentre = Some(recentre); a.recentre_iterations = iterations; });
            self.phase("descend", 0.75).await?;
        }
        self.descend(close_h).await?;
        let closed = self.gripper_phase("close", 0.8, GripperCommand::Close).await?;
        let holding = closed.holding == Some(true);
        let lift_to = |p: Vec3| GoalSpec::Absolute { p: [p[0], p[1], s.table_z_m + s.lift_height_m], r: None };
        if !holding {
            self.gripper_phase("release", 0.85, GripperCommand::Open).await?;
            self.phase("lift", 0.9).await?;
            let p = self.pose().await.p;
            self.move_arm("lift", lift_to(p), s.descend_speed_mps).await?;
            return Err(fail("NOT_HOLDING", format!("Jaws closed without holding the object (gap {:.2} rad)", closed.gripper_gap_rad.unwrap_or(0.0))));
        }
        self.phase("lift", 0.9).await?;
        let p = self.pose().await.p;
        self.move_arm("lift", lift_to(p), s.descend_speed_mps).await?;
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
            self.sup.inner().write().await.update_action(self.id, |a| a.post_lift = Some("regripped".into()));
            return Ok((true, Some(gap)));
        }
        self.sup.inner().write().await.update_action(self.id, |a| { a.post_lift = Some("dropped".into()); a.gripper_gap_rad = Some(gap); });
        Err(fail("DROPPED", format!("Object no longer held after the lift: the re-grip closed to {gap:.2} rad from closed")))
    }

    /// `holding` after `post_lift_settle_ms` from the measured J7, with the gripper policy's rule
    /// (as the close: gap from closed >= `holding_gap_rad` and median |J7 torque| over the window
    /// >= `holding_torque_nm`). Returns (holding, gap).
    async fn holding_now(&self) -> Result<(bool, f32), Stop> {
        let until = Instant::now() + Duration::from_millis(self.setup.post_lift_settle_ms);
        let mut taus = Vec::new();
        loop {
            self.check().await?;
            let (gap, tau, policy) = {
                let inner = self.sup.inner().read().await;
                let Some(policy) = inner.config.gripper_policy.clone() else { return Err(fail("UNSUPPORTED", "No gripper policy")); };
                let dir = if policy.closed_rad >= policy.open_rad { 1.0 } else { -1.0 };
                let m7 = inner.measured_positions.get(6).copied().unwrap_or(f32::NAN);
                ((policy.closed_rad - m7) * dir, inner.torques.get(6).copied().unwrap_or(0.0).abs(), policy)
            };
            taus.push(f64::from(tau));
            if Instant::now() >= until {
                let tau = median(&mut taus);
                return Ok((gap >= policy.holding_gap_rad && tau >= f64::from(policy.holding_torque_nm), gap));
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
            let done = self.move_arm(phase, GoalSpec::Relative { frame: RelFrame::Base, d: step, dyaw: 0.0, dpitch: 0.0, droll: 0.0, level: false }, s.approach_speed_mps).await?;
            since = done.at;
        }
    }

    /// Height probe: from the centred pose `c.pose` move straight up by `height_probe_m`, take
    /// `median_frames` fresh matched frames, and return to `c.pose`. The median box size
    /// s = sqrt(w·h) (px) of the centring frames (s_low) and of the raised frames (s_high) give
    /// the lens-to-top distance at the start pose D_low = dz · s_high / (s_low − s_high)
    /// ([`scale_height`]); top height = lens z − D_low − `table_z_m`. Moving up keeps the jaws
    /// away from an object whose height is not known yet. `Ok(Ok(h))` on success; `Ok(Err(reason))`
    /// when the probe was skipped or its result is unreliable (the grasp goes on with
    /// `grasp_height_m`); OBJECT_TOO_TALL when h >= `approach_height_m` − 0.02.
    async fn probe_height(&self, c: &Centred, label: &str, after: &mut Option<u64>) -> Result<Result<f64, String>, Stop> {
        let s = &self.setup;
        let skip = |r: &str| Ok(Err(r.to_string()));
        if s.height_probe_m <= 0.0 { return skip("disabled"); }
        if !self.lens_known { return skip("no_lens_centre"); }
        let pose = c.pose;
        if kinematics::column(&self.cam.rotation(&pose), 2)[2] > -(5f64.to_radians().cos()) { return skip("camera_not_vertical"); }
        if c.frames.iter().any(|d| touches_border(&d.box_norm, self.cam.size)) { return skip("border"); }
        let centre_px = self.cam.to_px(box_centre(&c.last.box_norm));
        let Some(est) = self.cam.plane_point(&pose, centre_px, grasp_plane(s)) else { return skip("ray"); };
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
            let spec = GoalSpec::Relative { frame: RelFrame::Base, d: [0.0, 0.0, dz], dyaw: 0.0, dpitch: 0.0, droll: 0.0, level: false };
            match self.move_arm("probe", spec, s.approach_speed_mps).await {
                Ok(d) => { moved = Some((d, dz)); break; }
                // rejected by the planner before anything was installed: the arm is still at the start
                Err(Stop::Fail(code, _)) if code.starts_with("IK_") || code == "WORKSPACE_VIOLATION" => {}
                Err(e) => return Err(e),
            }
        }
        let Some((done, dz)) = moved else { return skip("probe_unreachable"); };
        self.sup.inner().write().await.update_action(self.id, |a| a.probe_dz_m = Some(dz as f32));
        let now = self.pose().await;
        let mut tracker = Tracker { label: label.to_string(), predicted: c.last.box_norm, gate_px: s.acquire_dist_px };
        if let Some(p) = self.cam.project(&now, &est) { tracker.predict_at(self.cam.to_norm(p)); }
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
        let h = match scale_height(&low, &high, z_high - z_low) {
            Ok(d_low) => z_low - d_low - s.table_z_m,
            Err(reason) => return Ok(Err(reason)),
        };
        if !h.is_finite() { return skip("not_finite"); }
        if h < 0.0 { return Ok(Err(format!("below_table:{h:.3}"))); }
        if h >= s.approach_height_m - 0.02 {
            return Err(fail("OBJECT_TOO_TALL", format!("Object top {h:.3} m above the table, approach_height_m {:.3} m leaves less than 0.02 m", s.approach_height_m)));
        }
        Ok(Ok(h))
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
        let z = s.table_z_m + s.recentre_height_m;
        if s.recentre_height_m <= close_h + 0.02 || z >= pose.p[2] - 1e-3 { return skip("height"); }
        let grip_px = self.cam.to_px(s.grip_pixel_norm);
        let Some(m0) = self.cam.project(&c.pose, &[c.pose.p[0], c.pose.p[1], plane]) else { return skip("projection"); };
        let Some(t3) = self.cam.plane_point(&c.pose, self.cam.to_px(box_centre(&c.last.box_norm)), plane) else { return skip("projection"); };
        let goal = Pose::new(pose.r, [pose.p[0], pose.p[1], z]);
        if table_violation(&pose, &goal, &self.envelope, s.table_z_m).is_some() { return skip("table_clearance"); }
        let done = self.move_arm("descend", GoalSpec::Absolute { p: goal.p, r: None }, s.descend_speed_mps).await?;
        let now = self.pose().await;
        let Some(m1) = self.cam.project(&now, &[now.p[0], now.p[1], plane]) else { return skip("projection"); };
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

    /// Straight down to `close_h` above the table. Each poll while it runs records the peak
    /// |residual| of J2 / J3 and the peak J1..J6 tracking error; with `descend_contact_stop_nm`
    /// set, a J2 or J3 residual above it ends the descent at the measured position.
    async fn descend(&self, close_h: f64) -> Result<(), Stop> {
        let s = &self.setup;
        let p = self.pose().await.p;
        let mv = self.move_arm("descend", GoalSpec::Absolute { p: [p[0], p[1], s.table_z_m + close_h], r: None }, s.descend_speed_mps);
        tokio::pin!(mv);
        let (mut peak_res, mut peak_trk, mut contact): (Option<[f32; 2]>, Option<f32>, Option<f64>) = (None, None, None);
        let result = loop {
            tokio::select! {
                r = &mut mv => break r,
                _ = tokio::time::sleep(POLL) => {
                    let over = {
                        let inner = self.sup.inner().read().await;
                        if !inner.active_trajectory.as_ref().is_some_and(|t| t.action_id == Some(self.id)) { continue; }
                        let trk = (0..6).map(|i| (inner.commanded_positions[i] - inner.measured_positions[i]).abs()).fold(0.0f32, f32::max);
                        peak_trk = Some(peak_trk.unwrap_or(0.0).max(trk));
                        let Some(r) = inner.residual_torques else { continue };
                        let pk = peak_res.get_or_insert([0.0; 2]);
                        pk[0] = pk[0].max(r[1].abs());
                        pk[1] = pk[1].max(r[2].abs());
                        s.descend_contact_stop_nm.is_some_and(|lim| f64::from(r[1].abs()) > lim || f64::from(r[2].abs()) > lim)
                    };
                    if over && contact.is_none() {
                        let mut inner = self.sup.inner().write().await;
                        if inner.end_macro_motion_at_measured(self.id) {
                            let z = inner.config.arm_model.fk_f32(&inner.measured_positions).map(|p| p.p[2]).unwrap_or(f64::NAN);
                            contact = Some(z - s.table_z_m);
                            tracing::info!("Grasp {}: contact during the descent at {:.3} m above the table", self.id, z - s.table_z_m);
                        }
                    }
                }
            }
        };
        self.sup.inner().write().await.update_action(self.id, |a| {
            a.descend_peak_residual_nm = peak_res;
            a.descend_peak_tracking_rad = peak_trk;
            if let Some(h) = contact { a.descend_contact = Some(true); a.descend_contact_height_m = Some(h as f32); }
        });
        result.map(|_| ())
    }

    /// J6 roll (about the tool axis) that closes the jaws across the object's short side, and
    /// the `align` status. From the detector angle when the centring frames carry one with
    /// `angle_conf >= align_min_angle_conf`, else ±pi/2 from the box aspect. No turn when the
    /// last box is unreliable ([`align_guard`]) or no candidate fits J6's limits.
    async fn align_turn(&self, c: &Centred, plane: f64) -> (Option<f64>, String) {
        if let Err(reason) = align_guard(&c.last, &c.frames, self.cam.size) { return (None, format!("skipped:{reason}")); }
        let pose = self.pose().await;
        let inner = self.sup.inner().read().await;
        let (q6, lim) = (f64::from(inner.commanded_positions[5]), inner.config.limits[5].clone());
        drop(inner);
        let room = |t: f64| q6 + t >= f64::from(lim.min_position) + 0.1 && q6 + t <= f64::from(lim.max_position) - 0.1;
        if let Some(alpha) = frames_angle(&c.frames, self.setup.align_min_angle_conf) {
            // the long side on the plane, seen from the pose of the centring frames
            let px = self.cam.to_px(box_centre(&c.last.box_norm));
            let (a, b) = (alpha.to_radians().cos(), alpha.to_radians().sin());
            let (Some(p0), Some(p1)) = (self.cam.plane_point(&c.pose, px, plane), self.cam.plane_point(&c.pose, [px[0] + 20.0 * a, px[1] + 20.0 * b], plane)) else {
                return (None, "skipped:ray".into());
            };
            let jaw = kinematics::mat_vec(&pose.r, &[0.0, 1.0, 0.0]);
            let t0 = roll_across_long_side(jaw, kinematics::sub(&p1, &p0));
            let pi = std::f64::consts::PI;
            let Some(t) = [t0, t0 + pi, t0 - pi].into_iter().filter(|t| room(*t)).min_by(|x, y| x.abs().total_cmp(&y.abs())) else {
                return (None, "skipped:j6_limits".into());
            };
            return if t.abs() < 1e-3 { (None, "kept:angle".into()) } else { (Some(t), "turned:angle".into()) };
        }
        let b = c.last.box_norm;
        let (w, h) = ((b[2] - b[0]) * self.cam.size[0], (b[3] - b[1]) * self.cam.size[1]);
        if w.max(h) / w.min(h).max(1e-6) < self.setup.align_min_aspect { return (None, "skipped:aspect".into()); }
        // jaw closing direction (end_link y) seen in the image
        let jaw = kinematics::mat_vec(&kinematics::transpose(&self.cam.rec), &[0.0, 1.0, 0.0]);
        let jaw_along_x = jaw[0].abs() >= jaw[1].abs();
        let short_along_x = w < h;
        if jaw_along_x == short_along_x { return (None, "kept:box".into()); }
        let half = std::f64::consts::FRAC_PI_2;
        // roll about +x_tool; J6 turns with it when its axis is the tool axis
        let t = [half, -half].into_iter().find(|t| room(*t)).unwrap_or(half);
        (Some(t), "turned:box".into())
    }
}

/// Matched frames of the last centring pose.
struct Centred {
    last: Detection,
    frames: Vec<Detection>,
    /// Commanded pose at which `frames` were taken.
    pose: Pose,
    iterations: u32,
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

/// A box edge within 2 px of the image border (the box may be cut off).
pub fn touches_border(b: &[f64; 4], size: [f64; 2]) -> bool {
    let [w, h] = size;
    b[0] * w <= 2.0 || b[1] * h <= 2.0 || b[2] * w >= w - 2.0 || b[3] * h >= h - 2.0
}

/// Mean long-side angle (deg, [-90, 90)) of the frames with `angle_conf >= min_conf`, averaged
/// on doubled angles (an axis, not a direction); None without such frames.
pub fn frames_angle(frames: &[Detection], min_conf: f64) -> Option<f64> {
    let (mut c, mut s, mut n) = (0.0, 0.0, 0);
    for d in frames {
        if let (Some(a), Some(conf)) = (d.angle_deg, d.angle_conf) {
            if conf >= min_conf { c += (2.0 * a.to_radians()).cos(); s += (2.0 * a.to_radians()).sin(); n += 1; }
        }
    }
    if n == 0 || c.hypot(s) < 1e-9 { return None; }
    let a = (s.atan2(c) / 2.0).to_degrees();
    Some((a + 90.0).rem_euclid(180.0) - 90.0)
}

/// Roll about the (vertical, down) tool axis, in [-pi/2, pi/2), that turns the jaw closing
/// direction `jaw` (base frame) perpendicular to the long side `long` (base frame, horizontal
/// parts used). Rolling by t about the downward tool axis turns `jaw` by -t about base z.
pub fn roll_across_long_side(jaw: Vec3, long: Vec3) -> f64 {
    let pi = std::f64::consts::PI;
    let t = jaw[1].atan2(jaw[0]) - long[1].atan2(long[0]) - pi / 2.0;
    (t + pi / 2.0).rem_euclid(pi) - pi / 2.0
}

async fn run(sup: Arc<ArmSupervisor>, source: Arc<dyn DetectionSource>, id: u64, planned: PlannedGrasp) {
    let setup = planned.request.setup.clone();
    let cam = CamModel::assumed(&planned.input.arm, &setup, &planned.input.cam);
    let deadline = Instant::now() + Duration::from_secs_f64(setup.timeout_s);
    let per_request = source.per_request_pipelines().await;
    let triggered = per_request == Ok(true);
    let ctx = MacroCtx {
        sup: sup.clone(), id, deadline, phase_timeout_s: setup.phase_timeout_s, timeout_code: "GRASP_TIMEOUT",
        timeout_message: format!("Grasp did not finish within {:.0} s", setup.timeout_s),
    };
    let envelope = envelope(planned.input.ws.as_ref(), &planned.input.cam, &cam.rec);
    let lens_known = planned.input.cam.offset_m.is_some();
    let orch = Orchestrator { ctx, source: source.clone(), setup, cam, envelope, triggered, lens_known };
    let result = if let Err(e) = per_request {
        Err(fail("DETECTION_UNAVAILABLE", format!("Cannot reach the vision provider: {e}")))
    } else if triggered {
        // perception API 2: a ["yolo"] trigger per centring step, no mode change to restore
        orch.run(&planned).await
    } else { match source.vision_mode().await {
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
    let mut inner = sup.inner().write().await;
    match result {
        Ok((holding, gap)) => inner.finish_macro(id, "done", None, None, Some(holding), gap),
        Err(Stop::Fail(code, message)) => {
            tracing::info!("Grasp {id} failed: {code} {message}");
            let holding = if code == "NOT_HOLDING" || code == "DROPPED" { Some(false) } else { None };
            inner.finish_macro(id, "failed", Some(&code), Some(message), holding, None);
        }
        Err(Stop::Cancelled) => inner.finish_macro(id, "cancelled", None, None, None, None),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
        Detection { label: "cup".into(), score: 0.9, box_norm: b, angle_deg: angle.map(|a| a.0), angle_conf: angle.map(|a| a.1) }
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
        let a = frames_angle(&fr, 0.3).unwrap();
        assert!(a.abs() > 89.9, "{a}");
        assert_eq!(frames_angle(&fr[2..], 0.3), None, "below align_min_angle_conf");
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
        let cam = CamModel::assumed(&crate::chain::ArmModel::RebotDevArm, &setup, &CameraMount::default());
        let pose = Pose::new(look_down(&kinematics::fk(&[0.0, -1.167, -1.565, 1.435, 0.0, 1.5708]).r), [0.3, 0.05, 0.2]);
        assert!(kinematics::column(&pose.r, 0)[2] < -0.999999);
        let p = cam.plane_point(&pose, [300.0, 200.0], 0.01).unwrap();
        let px = cam.project(&pose, &p).unwrap();
        assert!((px[0] - 300.0).abs() < 1e-6 && (px[1] - 200.0).abs() < 1e-6, "{px:?}");
        // a ray pointing above the horizon never meets the table
        let up = Pose::new(kinematics::axis_r(&[0.0, 1.0, 0.0], std::f64::consts::PI), pose.p);
        assert!(cam.plane_point(&up, [320.0, 180.0], 0.0).is_none());
    }

    /// v8 holder: rot 90, tilt -38, lens centre behind and above the jaw tips.
    fn v8() -> (GraspSetup, CameraMount, CamModel) {
        let setup = GraspSetup { camera_tilt_deg: -38.0, ..Default::default() };
        let mount = CameraMount { rot_deg: 90.0, tilt_deg: -38.0, offset_m: Some([-0.0731, 0.0005, 0.066]), ..Default::default() };
        let cam = CamModel::assumed(&crate::chain::ArmModel::RebotDevArm, &setup, &mount);
        (setup, mount, cam)
    }

    fn inspect_r() -> Mat3 { kinematics::fk(&[0.0, -1.167, -1.565, 1.435, 0.0, std::f64::consts::FRAC_PI_2]).r }

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
        let flat = CamModel::assumed(&crate::chain::ArmModel::RebotDevArm, &GraspSetup::default(), &CameraMount::default());
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
        // rot 0, tilt -38: pitch about the axis along which the kit check points are 92.5 mm out
        let setup = GraspSetup { camera_tilt_deg: -38.0, table_z_m: 0.10, approach_height_m: 0.035, ..Default::default() };
        let mount = CameraMount { tilt_deg: -38.0, offset_m: Some([-0.0731, 0.0005, 0.066]), ..Default::default() };
        let cam = CamModel::assumed(&crate::chain::ArmModel::RebotDevArm, &setup, &mount);
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
}
