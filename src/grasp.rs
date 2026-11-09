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
use crate::model::CameraMount;
use crate::motion::{self, GoalSpec, GripperCommand, GripperRequest, MotionError, PlanInput, RelFrame, Speed};
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
    /// Linear speeds (m/s) for the approach / centring and for descend / lift.
    pub approach_speed_mps: f64,
    pub descend_speed_mps: f64,
    /// Limit per motion phase and for the whole grasp (s).
    pub phase_timeout_s: f64,
    pub timeout_s: f64,
}

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
            approach_speed_mps: 0.05,
            descend_speed_mps: 0.03,
            phase_timeout_s: 25.0,
            timeout_s: 120.0,
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
            self.align_min_aspect, self.approach_speed_mps, self.descend_speed_mps, self.phase_timeout_s, self.timeout_s];
        if !all.iter().all(|v| fin(*v)) { return Err("grasp setup values must be finite".into()); }
        let check = |ok: bool, msg: &str| if ok { Ok(()) } else { Err(msg.to_string()) };
        check((-0.5..=0.5).contains(&self.table_z_m), "table_z_m must be in [-0.5, 0.5] m")?;
        check(self.grasp_height_m >= 0.005 && self.grasp_height_m <= 0.2, "grasp_height_m must be in [0.005, 0.2] m")?;
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
        check(self.align_min_aspect >= 1.0 && self.align_min_aspect <= 10.0, "align_min_aspect must be in [1, 10]")?;
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

/// Write the setup atomically (temp file + rename), creating the directory.
pub fn save_setup(path: &Path, setup: &GraspSetup) -> std::io::Result<()> {
    if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty()) { std::fs::create_dir_all(dir)?; }
    let text = serde_json::to_string_pretty(setup).map_err(std::io::Error::other)?;
    let tmp = path.with_extension("json.tmp");
    std::fs::write(&tmp, text)?;
    std::fs::rename(&tmp, path)
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
            let z = kinematics::fk(&kinematics::q6(&inner.commanded_positions)).p[2];
            patch.insert("table_z_m".into(), serde_json::json!((z * 1e4).round() / 1e4));
        }
        Some(_) => return Err(MotionError::InvalidCommand("table_z_from_tcp must be boolean".into()).into()),
    }
    let setup = base.patched(&Value::Object(patch)).map_err(MotionError::InvalidCommand)?;
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
/// cockpit's `parseDetections`). Values above 1.05 are pixels of `size`.
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
                    _ => continue,
                }
            }
        } else { continue };
        if !(w > 0.0 && h > 0.0 && score.is_finite()) { continue; }
        let pixel = cx > 1.05 || cy > 1.05 || w > 1.05 || h > 1.05;
        let (sx, sy) = if pixel { (size[0], size[1]) } else { (1.0, 1.0) };
        let b = [(cx - w / 2.0) / sx, (cy - h / 2.0) / sy, (cx + w / 2.0) / sx, (cy + h / 2.0) / sy].map(|v| v.clamp(0.0, 1.0));
        if b[2] <= b[0] || b[3] <= b[1] { continue; }
        out.push(Detection { label, score: score.clamp(0.0, 1.0), box_norm: b });
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
    pub fn assumed(setup: &GraspSetup, mount: &CameraMount) -> Self {
        Self {
            rec: kinematics::cam_mount(mount.flip, f64::from(mount.rot_deg), setup.camera_tilt_deg),
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
    let x = kinematics::column(r, 0);
    let down = [0.0, 0.0, -1.0];
    let axis = kinematics::cross(&x, &down);
    let (s, c) = (kinematics::norm(&axis), kinematics::dot(&x, &down));
    if s < 1e-9 {
        if c > 0.0 { return *r; }
        return kinematics::mat_mul(&kinematics::axis_r(&kinematics::column(r, 1), std::f64::consts::PI), r);
    }
    kinematics::mat_mul(&kinematics::axis_r(&kinematics::unit(&axis), s.atan2(c)), r)
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
    pub request: GraspRequest,
    pub open_rad: f32,
}

/// A grasp whose coarse approach was found reachable.
pub struct PlannedGrasp {
    pub input: PlanInput,
    pub request: GraspRequest,
    /// Estimated target on the grasp plane (base frame).
    pub target: Vec3,
    pub approach_duration: f64,
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
    Ok(PendingGrasp { input, request, open_rad: policy.open_rad })
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
    let setup = &request.setup;
    let pose = kinematics::fk(&input.q0);
    let cam = CamModel::assumed(setup, &input.cam);
    let centre = cam.to_px(box_centre(&request.box_norm));
    let target = cam.plane_point(&pose, centre, grasp_plane(setup)).ok_or_else(|| ArmError::from(MotionError::InvalidCommand(
        "the box centre ray does not meet the table plane in front of the camera".into())))?;
    let mut open_input = input.clone();
    open_input.j7 = open_rad;
    let path = motion::plan_cartesian(&open_input, approach_pose(&pose.r, &target, setup), Speed { linear: setup.approach_speed_mps, angular: motion::DEFAULT_ANGULAR_RPS })?;
    Ok(PlannedGrasp { input, request, target, approach_duration: path.duration })
}

/// Rough total duration for the ack.
pub fn expected_duration(planned: &PlannedGrasp) -> f64 {
    let s = &planned.request.setup;
    let descend = (s.approach_height_m - s.grasp_height_m) * 1.875 / s.descend_speed_mps;
    let lift = (s.lift_height_m - s.grasp_height_m) * 1.875 / s.descend_speed_mps;
    6.5 + planned.approach_duration + 4.0 * 1.5 + descend + 2.0 + lift
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
    let id = supervisor.inner().write().await.begin_grasp(&planned.input, planned.request.tag.clone())?;
    let duration = expected_duration(&planned);
    spawn(supervisor.clone(), source, id, planned);
    Ok((id, duration))
}

/// Spawn the orchestrator for an action begun with `SupervisorInner::begin_grasp`.
pub fn spawn(supervisor: Arc<ArmSupervisor>, source: Arc<dyn DetectionSource>, id: u64, planned: PlannedGrasp) {
    tokio::spawn(run(supervisor, source, id, planned));
}

// ---- orchestrator ----

/// How a phase ended early.
#[derive(Debug, Clone, PartialEq)]
enum Stop {
    /// The action already ended (stop, E-stop, lease, supersede): exit without touching it.
    Cancelled,
    Fail(String, String),
}

fn fail(code: &str, message: impl Into<String>) -> Stop { Stop::Fail(code.into(), message.into()) }

fn arm_fail(e: ArmError) -> Stop {
    let api = ApiError::from(e);
    Stop::Fail(api.code.unwrap_or_else(|| api.error.clone()), api.message)
}

/// Result of a finished sub-motion.
#[derive(Debug, Clone, PartialEq)]
pub struct SubDone {
    pub settled: Option<bool>,
    pub holding: Option<bool>,
    pub gripper_gap_rad: Option<f32>,
    pub at: Instant,
}

#[derive(Debug, Clone, PartialEq)]
pub enum SubState {
    Idle,
    Running,
    Done(SubDone),
    Failed { code: String, message: String },
}

/// Supervisor-side record of a running grasp.
#[derive(Debug, Clone, PartialEq)]
pub struct GraspRun {
    pub id: u64,
    pub epoch: u64,
    pub sub: SubState,
    /// `/motion/stop` arrived while no sub-motion was running.
    pub stop_requested: bool,
}

const POLL: Duration = Duration::from_millis(20);

struct Orchestrator {
    sup: Arc<ArmSupervisor>,
    source: Arc<dyn DetectionSource>,
    id: u64,
    setup: GraspSetup,
    cam: CamModel,
    deadline: Instant,
    /// Each centring step triggers its own frame (perception API 2) instead of reading the
    /// continuous results.
    triggered: bool,
}

impl Orchestrator {
    async fn alive(&self) -> bool { self.sup.inner().read().await.grasp_alive(self.id) }

    async fn check(&self) -> Result<(), Stop> {
        if !self.alive().await { return Err(Stop::Cancelled); }
        if Instant::now() > self.deadline { return Err(fail("GRASP_TIMEOUT", format!("Grasp did not finish within {:.0} s", self.setup.timeout_s))); }
        Ok(())
    }

    async fn phase(&self, phase: &str, progress: f32) -> Result<(), Stop> {
        self.check().await?;
        self.sup.inner().write().await.set_grasp_phase(self.id, phase, progress, None, None);
        Ok(())
    }

    async fn pose(&self) -> Pose { kinematics::fk(&kinematics::q6(&self.sup.inner().read().await.commanded_positions)) }

    /// Wait for the installed sub-motion; stop it on a timeout.
    async fn wait_sub(&self, phase: &str, limit_s: f64) -> Result<SubDone, Stop> {
        let until = Instant::now() + Duration::from_secs_f64(limit_s.min(self.setup.phase_timeout_s));
        loop {
            tokio::time::sleep(POLL).await;
            let mut inner = self.sup.inner().write().await;
            if !inner.grasp_alive(self.id) { return Err(Stop::Cancelled); }
            match inner.grasp_sub(self.id) {
                Some(SubState::Done(d)) => return Ok(d),
                Some(SubState::Failed { code, message }) => return Err(Stop::Fail(code, message)),
                _ => {}
            }
            let now = Instant::now();
            if now > until || now > self.deadline {
                inner.stop_motion();
                return Err(fail("GRASP_TIMEOUT", format!("Phase {phase} did not finish in time")));
            }
        }
    }

    async fn move_arm(&self, phase: &str, spec: GoalSpec, linear: f64) -> Result<SubDone, Stop> {
        let input = {
            let inner = self.sup.inner().read().await;
            if !inner.grasp_alive(self.id) { return Err(Stop::Cancelled); }
            inner.plan_input().map_err(arm_fail)?
        };
        let goal = motion::resolve_goal(&input, &spec);
        let speed = Speed { linear, angular: motion::DEFAULT_ANGULAR_RPS };
        let (input, planned) = tokio::task::spawn_blocking(move || { let p = motion::plan_cartesian(&input, goal, speed); (input, p) }).await
            .map_err(|e| fail("INTERNAL", format!("planner task failed: {e}")))?;
        let path = planned.map_err(|e| arm_fail(e.into()))?;
        let limit = {
            let mut inner = self.sup.inner().write().await;
            if !inner.grasp_alive(self.id) { return Err(Stop::Cancelled); }
            inner.install_grasp_path(self.id, &input, path).map_err(arm_fail)?
        };
        self.wait_sub(phase, limit * 2.0 + 5.0).await
    }

    async fn gripper(&self, phase: &str, command: GripperCommand) -> Result<SubDone, Stop> {
        let request = GripperRequest { command, fraction: None, grasp: command == GripperCommand::Close, speed: 1.0, timeout_s: 8.0, tag: None };
        let limit = {
            let mut inner = self.sup.inner().write().await;
            if !inner.grasp_alive(self.id) { return Err(Stop::Cancelled); }
            inner.start_grasp_gripper(self.id, request).map_err(arm_fail)?
        };
        self.wait_sub(phase, limit + 10.0).await
    }

    /// Next fresh frame (captured after `since` + settle delay) containing the target.
    async fn fresh_match(&self, tracker: &Tracker, since: Instant, after: &mut Option<u64>) -> Result<Detection, Stop> {
        let not_before = since + Duration::from_secs_f64(self.setup.settle_delay_s);
        let started = Instant::now();
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
            if frame.captured_at < not_before { continue; }
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

        // coarse approach, looking down
        self.phase("approach", 0.1).await?;
        let pose0 = self.pose().await;
        let target = planned.target;
        let goal = approach_pose(&pose0.r, &target, s);
        let done = self.move_arm("approach", GoalSpec::Absolute { p: goal.p, r: Some(goal.r) }, s.approach_speed_mps).await?;

        // closed-loop centring
        self.phase("centring", 0.3).await?;
        let grip_px = self.cam.to_px(s.grip_pixel_norm);
        let plane = grasp_plane(s);
        let pose = self.pose().await;
        let predicted = self.cam.project(&pose, &target).map(|p| self.cam.to_norm(p)).unwrap_or(s.grip_pixel_norm);
        let mut tracker = Tracker { label: planned.request.label.clone(), predicted: planned.request.box_norm, gate_px: s.acquire_dist_px };
        tracker.predict_at(predicted);
        let mut since = done.at;
        let mut after = None;
        let mut iteration = 0u32;
        let last = loop {
            let pose = self.pose().await;
            let mut xs = Vec::new();
            let mut ys = Vec::new();
            let mut consecutive = 0u32;
            let mut last_det;
            let decision = loop {
                let d = self.fresh_match(&tracker, since, &mut after).await?;
                tracker.gate_px = s.match_dist_px;
                tracker.predicted = d.box_norm;
                let c = self.cam.to_px(box_centre(&d.box_norm));
                last_det = d;
                xs.push(c[0]);
                ys.push(c[1]);
                let err = (c[0] - grip_px[0]).hypot(c[1] - grip_px[1]);
                if err <= s.centre_tol_px { consecutive += 1; } else { consecutive = 0; }
                self.sup.inner().write().await.set_grasp_phase(self.id, "centring", 0.3, Some(iteration), Some(err as f32));
                if consecutive >= s.stable_frames { break None; }
                let m = s.median_frames as usize;
                if xs.len() >= m {
                    let med = [median(&mut xs[xs.len() - m..].to_vec()), median(&mut ys[ys.len() - m..].to_vec())];
                    if (med[0] - grip_px[0]).hypot(med[1] - grip_px[1]) > s.centre_tol_px { break Some(med); }
                    if xs.len() >= m + s.stable_frames as usize { break None; }
                }
            };
            let Some(med) = decision else { break last_det; };
            if iteration >= s.max_iterations {
                let err = (med[0] - grip_px[0]).hypot(med[1] - grip_px[1]);
                return Err(fail("CENTRING_FAILED", format!("Target still {err:.0} px from the grip point after {iteration} corrections")));
            }
            iteration += 1;
            let (Some(pt), Some(pg)) = (self.cam.plane_point(&pose, med, plane), self.cam.plane_point(&pose, grip_px, plane)) else {
                return Err(fail("CENTRING_FAILED", "Pixel error cannot be mapped onto the table plane"));
            };
            let mut step = [(pt[0] - pg[0]) * s.servo_gain, (pt[1] - pg[1]) * s.servo_gain, 0.0];
            let len = step[0].hypot(step[1]);
            if len > s.max_step_m { step = [step[0] * s.max_step_m / len, step[1] * s.max_step_m / len, 0.0]; }
            // predict where the target lands after the step (same rough model)
            let moved = Pose::new(pose.r, kinematics::add(&pose.p, &step));
            if let Some(p) = self.cam.project(&moved, &pt) { tracker.predict_at(self.cam.to_norm(p)); }
            let done = self.move_arm("centring", GoalSpec::Relative { frame: RelFrame::Base, d: step, dyaw: 0.0, dpitch: 0.0, droll: 0.0, level: false }, s.approach_speed_mps).await?;
            since = done.at;
        };

        // J6 alignment: close the jaws across the box's short side
        self.phase("align", 0.55).await?;
        if s.align_j6 {
            if let Some(turn) = self.align_turn(&last).await {
                self.move_arm("align", GoalSpec::Relative { frame: RelFrame::Tool, d: [0.0; 3], dyaw: 0.0, dpitch: 0.0, droll: turn, level: false }, s.approach_speed_mps).await?;
            }
        }

        // descend, close, check
        self.phase("descend", 0.65).await?;
        let p = self.pose().await.p;
        self.move_arm("descend", GoalSpec::Absolute { p: [p[0], p[1], s.table_z_m + s.grasp_height_m], r: None }, s.descend_speed_mps).await?;
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
        Ok((true, closed.gripper_gap_rad))
    }

    async fn gripper_phase(&self, phase: &str, progress: f32, command: GripperCommand) -> Result<SubDone, Stop> {
        self.phase(phase, progress).await?;
        self.gripper(phase, command).await
    }

    /// ±pi/2 about the tool axis when the jaws would close across the long side.
    async fn align_turn(&self, det: &Detection) -> Option<f64> {
        let b = det.box_norm;
        let (w, h) = ((b[2] - b[0]) * self.cam.size[0], (b[3] - b[1]) * self.cam.size[1]);
        if w.max(h) / w.min(h).max(1e-6) < self.setup.align_min_aspect { return None; }
        // jaw closing direction (end_link y) seen in the image
        let jaw = kinematics::mat_vec(&kinematics::transpose(&self.cam.rec), &[0.0, 1.0, 0.0]);
        let jaw_along_x = jaw[0].abs() >= jaw[1].abs();
        let short_along_x = w < h;
        if jaw_along_x == short_along_x { return None; }
        let inner = self.sup.inner().read().await;
        let (q6, lim) = (f64::from(inner.commanded_positions[5]), &inner.config.limits[5]);
        let room = |t: f64| q6 + t >= f64::from(lim.min_position) + 0.1 && q6 + t <= f64::from(lim.max_position) - 0.1;
        let half = std::f64::consts::FRAC_PI_2;
        // roll about +x_tool; J6 turns with it when its axis is the tool axis
        [half, -half].into_iter().find(|t| room(*t)).or(Some(half))
    }
}

async fn run(sup: Arc<ArmSupervisor>, source: Arc<dyn DetectionSource>, id: u64, planned: PlannedGrasp) {
    let setup = planned.request.setup.clone();
    let cam = CamModel::assumed(&setup, &planned.input.cam);
    let deadline = Instant::now() + Duration::from_secs_f64(setup.timeout_s);
    let per_request = source.per_request_pipelines().await;
    let triggered = per_request == Ok(true);
    let orch = Orchestrator { sup: sup.clone(), source: source.clone(), id, deadline, setup, cam, triggered };
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
        Ok((holding, gap)) => inner.finish_grasp(id, "done", None, None, Some(holding), gap),
        Err(Stop::Fail(code, message)) => {
            tracing::info!("Grasp {id} failed: {code} {message}");
            let holding = if code == "NOT_HOLDING" { Some(false) } else { None };
            inner.finish_grasp(id, "failed", Some(&code), Some(message), holding, None);
        }
        Err(Stop::Cancelled) => inner.finish_grasp(id, "cancelled", None, None, None, None),
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
            {"label": "mouse", "score": 0.7, "box_norm": [0.5, 0.5, 0.6, 0.7]}
        ]);
        let d = parse_detections(&raw, [640.0, 360.0]);
        assert_eq!(d.len(), 3, "{d:?}");
        assert_eq!(d[0].label, "cup");
        assert!((d[0].box_norm[0] - 0.45).abs() < 1e-9 && (d[0].box_norm[3] - 0.55).abs() < 1e-9);
        assert!(d[1].box_norm.iter().zip([0.1, 0.2, 0.3, 0.4]).all(|(a, b)| (a - b).abs() < 1e-9));
        assert_eq!(d[2].label, "mouse");
    }

    #[test]
    #[allow(clippy::approx_constant)]
    fn ray_and_projection_round_trip() {
        let setup = GraspSetup { camera_tilt_deg: 20.0, ..Default::default() };
        let cam = CamModel::assumed(&setup, &CameraMount::default());
        let pose = Pose::new(look_down(&kinematics::fk(&[0.0, -1.167, -1.565, 1.435, 0.0, 1.5708]).r), [0.3, 0.05, 0.2]);
        assert!(kinematics::column(&pose.r, 0)[2] < -0.999999);
        let p = cam.plane_point(&pose, [300.0, 200.0], 0.01).unwrap();
        let px = cam.project(&pose, &p).unwrap();
        assert!((px[0] - 300.0).abs() < 1e-6 && (px[1] - 200.0).abs() < 1e-6, "{px:?}");
        // a ray pointing above the horizon never meets the table
        let up = Pose::new(kinematics::axis_r(&[0.0, 1.0, 0.0], std::f64::consts::PI), pose.p);
        assert!(cam.plane_point(&up, [320.0, 180.0], 0.0).is_none());
    }
}
