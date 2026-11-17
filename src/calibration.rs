//! Camera calibration persistence and the apply / restore transaction (calibration wizard).
//!
//! `calibration.json` (`ARMD_CALIBRATION`, default [`DEFAULT_PATH`]) holds the calibrated
//! camera mount (`q_end_cam`, `offset_m`), a copy of the intrinsics written to rebot-vision,
//! the table height and the solver residuals. It is loaded at startup after the grasp setup and
//! wins for `camera_mount` and `camera_tilt_deg`.
//!
//! WS `/api/v1/calibration/apply` writes three places in order: rebot-vision's intrinsics
//! (`PUT /api/calibration/intrinsics` through armd's detector URL), `calibration.json` plus the
//! live camera mount, and the grasp setup (live and `grasp_setup.json`). A backup of all three
//! is taken first; a failing step reverts the earlier ones. WS `/api/v1/calibration/restore`
//! puts a backup back in the same order. While a transaction runs, motion commands
//! (MOTION_BUSY) and other configuration writes (CONFIG_BUSY) are refused; heartbeats go on.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use axum::http::StatusCode;
use serde::{Deserialize, Serialize};
use serde_json::{json, Map, Value};

use crate::api::proxy::DetectorUpstream;
use crate::grasp::{CamModel, GraspSetup};
use crate::model::CameraMount;
use crate::supervisor::{ArmSupervisor, SupervisorInner};

pub const DEFAULT_PATH: &str = "/userdata/rebot-teleop/calibration.json";
/// rebot-vision intrinsics route (GET and PUT).
pub const VISION_INTRINSICS_PATH: &str = "/api/calibration/intrinsics";
/// The image size every calibrated value refers to (rebot-vision's content image).
pub const IMAGE_SIZE_PX: [f64; 2] = [640.0, 360.0];
/// Timeout of one rebot-vision request ([`CalibrationDeps::vision_timeout`] default).
pub const VISION_TIMEOUT: Duration = Duration::from_secs(5);
const INTRINSIC_KEYS: [&str; 9] = ["fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3"];

/// `calibration.json`.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct CalibrationFile {
    /// flip / rot_deg as the profile, tilt_deg derived, offset_m and q_end_cam calibrated.
    pub camera_mount: CameraMount,
    /// Copy of the intrinsics written to rebot-vision (fx..k3, width, height, marker_size).
    pub intrinsics: Option<Value>,
    pub table_z_m: Option<f64>,
    /// UTC time of the apply (RFC 3339), or the solver's time when the client sent it.
    pub solved_at: Option<String>,
    /// Residuals reported by the solver (passed through).
    pub rms: Option<Value>,
}

impl CalibrationFile {
    pub fn validate(&self) -> Result<(), String> {
        let m = &self.camera_mount;
        let Some(q) = &m.q_end_cam else { return Err("calibration.json: camera_mount.q_end_cam is required".into()); };
        crate::model::unit_quaternion(q)?;
        if !m.offset_m.is_some_and(|o| o.iter().all(|v| v.is_finite() && v.abs() <= 0.3)) {
            return Err("calibration.json: camera_mount.offset_m must be three values within 0.3 m".into());
        }
        if self.table_z_m.is_some_and(|z| !z.is_finite() || z.abs() > 0.5) {
            return Err("calibration.json: table_z_m must be in [-0.5, 0.5] m".into());
        }
        Ok(())
    }
}

/// Files and upstream of the calibration commands.
#[derive(Clone, Debug)]
pub struct CalibrationDeps {
    /// `calibration.json`.
    pub path: PathBuf,
    /// Backup root (`ARMD_CALIBRATION_BACKUPS`, default `calibration-backups/` next to `path`).
    pub backup_dir: PathBuf,
    /// `grasp_setup.json` (None = the grasp setup is not persisted).
    pub setup_path: Option<PathBuf>,
    /// rebot-vision (armd's detector URL).
    pub upstream: DetectorUpstream,
    /// Timeout of one rebot-vision request (default [`VISION_TIMEOUT`]).
    pub vision_timeout: Duration,
}

/// `ARMD_CALIBRATION` or [`DEFAULT_PATH`].
pub fn path_from_env() -> PathBuf { crate::persist::path_from_env("ARMD_CALIBRATION", DEFAULT_PATH) }

impl CalibrationDeps {
    pub fn new(path: PathBuf, setup_path: Option<PathBuf>, upstream: DetectorUpstream) -> Self {
        let backup_dir = path.parent().unwrap_or(Path::new(".")).join("calibration-backups");
        Self { path, backup_dir, setup_path, upstream, vision_timeout: VISION_TIMEOUT }
    }

    /// Paths from `ARMD_CALIBRATION` / `ARMD_CALIBRATION_BACKUPS`.
    pub fn from_env(setup_path: Option<PathBuf>, upstream: DetectorUpstream) -> Self {
        let mut d = Self::new(path_from_env(), setup_path, upstream);
        if let Some(b) = std::env::var_os("ARMD_CALIBRATION_BACKUPS") { d.backup_dir = PathBuf::from(b); }
        d
    }
}

/// The live camera mount for a calibration: the profile mount's flip / rot_deg / body size with
/// the calibrated offset and quaternion, tilt_deg derived from the quaternion.
fn calibrated_mount(inner: &SupervisorInner, offset: [f32; 3], q: [f64; 4]) -> CameraMount {
    let mut m = CameraMount { offset_m: Some(offset), q_end_cam: Some(q), ..inner.profile_camera_mount.clone() };
    m.tilt_deg = inner.config.arm_model.mount_tilt_deg(&m) as f32;
    m
}

/// Read `calibration.json` at startup (after the grasp setup) and install its camera mount.
/// A malformed or invalid file is moved aside and the profile mount stays.
pub async fn load_into(supervisor: &ArmSupervisor, path: &Path) {
    let loaded = crate::persist::load::<CalibrationFile>(path, CalibrationFile::validate);
    let mut inner = supervisor.inner().write().await;
    match loaded {
        crate::persist::Loaded::Loaded(mut file) => {
            let q = crate::model::unit_quaternion(&file.camera_mount.q_end_cam.unwrap_or([1.0, 0.0, 0.0, 0.0])).unwrap_or([1.0, 0.0, 0.0, 0.0]);
            let mount = calibrated_mount(&inner, file.camera_mount.offset_m.unwrap_or([0.0; 3]), q);
            file.camera_mount = mount.clone();
            tracing::info!("Camera calibration loaded from {}: q_end_cam {:?}, offset_m {:?}, derived tilt {:.2} deg (overrides the profile mount and the grasp setup's camera_tilt_deg)",
                path.display(), mount.q_end_cam, mount.offset_m, mount.tilt_deg);
            inner.set_camera_mount(mount);
            inner.calibration = Some(file);
        }
        crate::persist::Loaded::Missing => tracing::info!("No camera calibration at {}; using the profile camera mount", path.display()),
        crate::persist::Loaded::Invalid(e) => tracing::warn!("Ignoring camera calibration file: {e}; using the profile camera mount"),
    }
}

// ---- errors ----

/// A failed calibration command: HTTP-like status, code, message, the step that failed and
/// extra reply fields (`state`, `backup`).
#[derive(Debug, Clone)]
pub struct CalibError {
    pub status: u16,
    pub code: String,
    pub message: String,
    pub step: Option<&'static str>,
    pub extra: Map<String, Value>,
}

impl CalibError {
    pub fn new(status: u16, code: &str, message: impl Into<String>) -> Self {
        Self { status, code: code.into(), message: message.into(), step: None, extra: Map::new() }
    }
    fn at(mut self, step: &'static str) -> Self { self.step = Some(step); self }
    fn invalid(message: impl Into<String>) -> Self { Self::new(400, "INVALID_COMMAND", message) }

    /// Fields of a `command_reject`.
    pub fn reply_fields(&self) -> Map<String, Value> {
        let mut m = self.extra.clone();
        m.insert("status".into(), json!(self.status));
        m.insert("code".into(), json!(self.code));
        m.insert("error".into(), json!(self.code));
        m.insert("message".into(), json!(self.message));
        if let Some(s) = self.step { m.insert("step".into(), json!(s)); }
        m
    }
}

impl From<crate::supervisor::ArmError> for CalibError {
    fn from(e: crate::supervisor::ArmError) -> Self {
        let api: crate::api::rest::ApiError = e.into();
        Self::new(api.status.as_u16(), api.code.as_deref().unwrap_or(&api.error), api.message)
    }
}

// ---- request ----

/// Validated intrinsics (W7 rules): fx..k3, width 640, height 360, marker_size optional.
pub fn validate_intrinsics(v: &Value) -> Result<Map<String, Value>, String> {
    let o = v.as_object().ok_or("intrinsics must be an object")?;
    let num = |k: &str| o.get(k).and_then(Value::as_f64).filter(|x| x.is_finite()).ok_or(format!("intrinsics.{k} must be a finite number"));
    let mut out = Map::new();
    for k in INTRINSIC_KEYS { out.insert(k.into(), json!(num(k)?)); }
    let (w, h) = (num("width")?, num("height")?);
    if w != IMAGE_SIZE_PX[0] || h != IMAGE_SIZE_PX[1] { return Err("intrinsics.width / height must be 640 x 360".into()); }
    let g = |k: &str| out[k].as_f64().unwrap_or(f64::NAN);
    for k in ["fx", "fy"] { if !(100.0..=5000.0).contains(&g(k)) { return Err(format!("intrinsics.{k} must be in [100, 5000]")); } }
    if !(g("cx") > 0.0 && g("cx") < w) { return Err("intrinsics.cx must be in (0, width)".into()); }
    if !(g("cy") > 0.0 && g("cy") < h) { return Err("intrinsics.cy must be in (0, height)".into()); }
    for k in ["k1", "k2", "k3"] { if g(k).abs() > 10.0 { return Err(format!("intrinsics.{k} must be in [-10, 10]")); } }
    for k in ["p1", "p2"] { if g(k).abs() > 1.0 { return Err(format!("intrinsics.{k} must be in [-1, 1]")); } }
    out.insert("width".into(), json!(640));
    out.insert("height".into(), json!(360));
    if let Some(ms) = o.get("marker_size").filter(|v| !v.is_null()) {
        let ms = ms.as_f64().filter(|x| x.is_finite() && *x > 0.0 && *x <= 1.0).ok_or("intrinsics.marker_size must be in (0, 1] m")?;
        out.insert("marker_size".into(), json!(ms));
    }
    Ok(out)
}

/// Body of `/api/v1/calibration/apply`.
#[derive(Debug, Clone)]
pub struct ApplyRequest {
    pub intrinsics: Map<String, Value>,
    pub offset_m: [f32; 3],
    pub q_end_cam: [f64; 4],
    pub table_z_m: f64,
    pub rms: Option<Value>,
    pub solved_at: Option<String>,
    pub dry_run: bool,
}

pub fn parse_apply(body: &Value) -> Result<ApplyRequest, CalibError> {
    let o = body.as_object().ok_or_else(|| CalibError::invalid("Command body must be an object"))?;
    for k in o.keys() {
        if !["intrinsics", "camera_mount", "table_z_m", "rms", "solved_at", "dry_run"].contains(&k.as_str()) {
            return Err(CalibError::invalid(format!("unknown field: {k}")));
        }
    }
    let intrinsics = validate_intrinsics(o.get("intrinsics").unwrap_or(&Value::Null)).map_err(CalibError::invalid)?;
    let cm = o.get("camera_mount").and_then(Value::as_object).ok_or_else(|| CalibError::invalid("camera_mount must be {offset_m, q_end_cam}"))?;
    let arr = |k: &str, n: usize| -> Result<Vec<f64>, CalibError> {
        let a = cm.get(k).and_then(Value::as_array).filter(|a| a.len() == n)
            .ok_or_else(|| CalibError::invalid(format!("camera_mount.{k} must be an array of {n} numbers")))?;
        a.iter().map(|v| v.as_f64().filter(|x| x.is_finite()).ok_or_else(|| CalibError::invalid(format!("camera_mount.{k} must be finite numbers")))).collect()
    };
    let off = arr("offset_m", 3)?;
    if off.iter().any(|v| v.abs() > 0.3) { return Err(CalibError::invalid("camera_mount.offset_m must be within 0.3 m of end_link")); }
    let q = arr("q_end_cam", 4)?;
    let q_end_cam = crate::model::unit_quaternion(&[q[0], q[1], q[2], q[3]]).map_err(CalibError::invalid)?;
    let table_z_m = o.get("table_z_m").and_then(Value::as_f64).filter(|z| z.is_finite() && z.abs() <= 0.5)
        .ok_or_else(|| CalibError::invalid("table_z_m must be a number in [-0.5, 0.5] m"))?;
    let dry_run = match o.get("dry_run") {
        None | Some(Value::Null) => false,
        Some(Value::Bool(b)) => *b,
        Some(_) => return Err(CalibError::invalid("dry_run must be boolean")),
    };
    let solved_at = match o.get("solved_at") {
        None | Some(Value::Null) => None,
        Some(Value::String(s)) if s.len() <= 64 => Some(s.clone()),
        Some(_) => return Err(CalibError::invalid("solved_at must be a string of at most 64 bytes")),
    };
    Ok(ApplyRequest {
        intrinsics,
        offset_m: [off[0] as f32, off[1] as f32, off[2] as f32],
        q_end_cam,
        table_z_m,
        rms: o.get("rms").filter(|v| !v.is_null()).cloned(),
        solved_at,
        dry_run,
    })
}

// ---- derived values ----

/// X3: the grip pixel (normalised) of a camera model: the projection of the point under the
/// TCP on the grasp plane (`table_z_m + grasp_height_m`) seen from the look-down approach pose
/// (tool axis end_link x vertical, TCP at `table_z_m + approach_height_m`). Independent of the
/// yaw about the vertical. None when the point is behind the camera.
pub fn grip_pixel_norm(cam: &CamModel, setup: &GraspSetup) -> Option<[f64; 2]> {
    let pose = crate::kinematics::Pose::new(crate::grasp::look_down(&crate::kinematics::IDENTITY), [0.0, 0.0, setup.table_z_m + setup.approach_height_m]);
    let px = cam.project(&pose, &[0.0, 0.0, setup.table_z_m + setup.grasp_height_m])?;
    let n = cam.to_norm(px);
    Some([(n[0] * 1e4).round() / 1e4, (n[1] * 1e4).round() / 1e4])
}

/// The grasp setup fields the calibration sets.
pub fn grasp_fields(s: &GraspSetup) -> Value {
    json!({"fx_px": s.fx_px, "fy_px": s.fy_px, "cx_px": s.cx_px, "cy_px": s.cy_px, "image_size_px": s.image_size_px,
        "table_z_m": s.table_z_m, "camera_tilt_deg": s.camera_tilt_deg, "grip_pixel_norm": s.grip_pixel_norm})
}

/// State of armd that a transaction changes (and a backup holds).
#[derive(Debug, Clone, Serialize, Deserialize)]
struct ArmdState {
    camera_mount: CameraMount,
    calibration: Option<CalibrationFile>,
    grasp_setup: Option<GraspSetup>,
    /// Apply: the grip pixel of the new camera model ([`grip_pixel_norm`]), also when it is
    /// outside the image (then `grip_pixel_norm` is left as it was).
    #[serde(skip)]
    grip_pixel_model: Option<[f64; 2]>,
}

fn armd_state(inner: &SupervisorInner) -> ArmdState {
    ArmdState { camera_mount: inner.config.camera_mount.clone(), calibration: inner.calibration.clone(), grasp_setup: inner.config.grasp.clone(), grip_pixel_model: None }
}

fn mount_summary(m: &CameraMount) -> Value {
    json!({"flip": m.flip, "rot_deg": m.rot_deg, "tilt_deg": m.tilt_deg, "offset_m": m.offset_m, "q_end_cam": m.q_end_cam})
}

/// The state an apply produces, computed under the lock.
fn after_state(inner: &SupervisorInner, req: &ApplyRequest) -> Result<ArmdState, CalibError> {
    let Some(base) = inner.config.grasp.clone() else {
        return Err(CalibError::new(400, "UNSUPPORTED", "This profile has no grasp setup"));
    };
    let mount = calibrated_mount(inner, req.offset_m, req.q_end_cam);
    let g = |k: &str| req.intrinsics[k].as_f64().unwrap_or(f64::NAN);
    let mut setup = GraspSetup {
        fx_px: g("fx"), fy_px: g("fy"), cx_px: Some(g("cx")), cy_px: Some(g("cy")), image_size_px: IMAGE_SIZE_PX,
        table_z_m: req.table_z_m, camera_tilt_deg: f64::from(mount.tilt_deg), ..base
    };
    let cam = CamModel::assumed(&inner.config.arm_model, &setup, &mount);
    // outside the image (a mount tilted toward the jaws, e.g. v8 at approach height): the grip
    // pixel cannot be stored; camera-down acquisition does not use it, so it stays as it was
    let grip_pixel_model = grip_pixel_norm(&cam, &setup);
    if let Some(p) = grip_pixel_model.filter(|p| p.iter().all(|v| (0.0..=1.0).contains(v))) { setup.grip_pixel_norm = p; }
    setup.validate().map_err(|e| CalibError::invalid(format!("grasp setup after the calibration: {e}")))?;
    let file = CalibrationFile {
        camera_mount: mount.clone(),
        intrinsics: Some(Value::Object(req.intrinsics.clone())),
        table_z_m: Some(req.table_z_m),
        solved_at: Some(req.solved_at.clone().unwrap_or_else(|| chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Secs, true))),
        rms: req.rms.clone(),
    };
    Ok(ArmdState { camera_mount: mount, calibration: Some(file), grasp_setup: Some(setup), grip_pixel_model })
}

// ---- rebot-vision ----

/// `GET /api/calibration/intrinsics`; 404 = VISION_CALIBRATION_UNSUPPORTED.
async fn vision_get(deps: &CalibrationDeps) -> Result<Value, CalibError> {
    match crate::api::proxy::request_status_json(&deps.upstream, "GET", VISION_INTRINSICS_PATH, None, deps.vision_timeout).await {
        Ok((StatusCode::NOT_FOUND, _)) => Err(CalibError::new(409, "VISION_CALIBRATION_UNSUPPORTED",
            "rebot-vision has no /api/calibration/intrinsics; update rebot-teleop")),
        Ok((s, v)) if s.is_success() && v.is_object() => Ok(v),
        Ok((s, v)) => Err(CalibError::new(502, "VISION_ERROR", format!("rebot-vision GET intrinsics returned {s}: {v}"))),
        Err(e) => Err(CalibError::new(502, "VISION_UNREACHABLE", format!("rebot-vision: {e}"))),
    }
}

/// `PUT /api/calibration/intrinsics` with `values` (fx..k3, width, height, marker_size?) at
/// `expected_revision`. Returns the reply body.
async fn vision_put(deps: &CalibrationDeps, values: &Map<String, Value>, expected_revision: u64) -> Result<Value, CalibError> {
    let mut body = Map::new();
    for k in INTRINSIC_KEYS.iter().chain(&["width", "height", "marker_size"]) {
        if let Some(v) = values.get(*k).filter(|v| !v.is_null()) { body.insert((*k).into(), v.clone()); }
    }
    body.insert("expected_revision".into(), json!(expected_revision));
    let bytes = serde_json::to_vec(&Value::Object(body)).unwrap_or_default();
    match crate::api::proxy::request_status_json(&deps.upstream, "PUT", VISION_INTRINSICS_PATH, Some(&bytes), deps.vision_timeout).await {
        Ok((StatusCode::NOT_FOUND, _)) => Err(CalibError::new(409, "VISION_CALIBRATION_UNSUPPORTED", "rebot-vision has no /api/calibration/intrinsics")),
        Ok((s, v)) if s.is_success() => Ok(v),
        Ok((s, v)) => {
            let code = v.get("code").or_else(|| v.get("error")).and_then(Value::as_str).filter(|c| c.chars().all(|c| c.is_ascii_uppercase() || c == '_')).unwrap_or("VISION_ERROR");
            Err(CalibError::new(502, code, format!("rebot-vision PUT intrinsics returned {s}: {v}")))
        }
        Err(e) => Err(CalibError::new(502, "VISION_UNREACHABLE", format!("rebot-vision: {e}"))),
    }
}

fn revision(v: &Value) -> u64 { v.get("revision").and_then(Value::as_u64).unwrap_or(0) }

/// The intrinsics in `values` (fx..k3, width, height; null / missing skipped) equal those in `got`.
fn intrinsics_match(got: &Value, values: &Map<String, Value>) -> bool {
    INTRINSIC_KEYS.iter().chain(&["width", "height"]).all(|k| match values.get(*k).filter(|v| !v.is_null()).and_then(Value::as_f64) {
        None => true,
        Some(want) => got.get(*k).and_then(Value::as_f64).is_some_and(|v| (v - want).abs() <= 1e-9 * want.abs().max(1.0)),
    })
}

/// What rebot-vision holds after a PUT whose reply was an error (timeout, 500 after the rename).
enum VisionAfterError {
    /// Our values are in place at this revision (the PUT was applied).
    Written(u64),
    /// Still the values before the PUT.
    Unchanged,
    /// The read-back failed too.
    Unknown(String),
}

/// Read rebot-vision back after a failed PUT of `values` over `before`.
async fn vision_after_error(deps: &CalibrationDeps, before: &Value, values: &Map<String, Value>) -> VisionAfterError {
    match vision_get(deps).await {
        Err(e) => VisionAfterError::Unknown(e.message),
        Ok(got) => {
            let moved = revision(&got) > revision(before);
            let ours = intrinsics_match(&got, values) && !intrinsics_match(before, values);
            if moved || ours { VisionAfterError::Written(revision(&got)) } else { VisionAfterError::Unchanged }
        }
    }
}

// ---- files ----

async fn blocking<T: Send + 'static>(f: impl FnOnce() -> std::io::Result<T> + Send + 'static) -> Result<T, String> {
    match tokio::task::spawn_blocking(f).await {
        Ok(r) => r.map_err(|e| e.to_string()),
        Err(e) => Err(format!("task failed: {e}")),
    }
}

/// Write (or with None remove) a file durably.
async fn put_file(path: PathBuf, text: Option<String>) -> Result<(), String> {
    blocking(move || match text {
        Some(t) => crate::persist::write_atomic(&path, &t),
        None => match std::fs::remove_file(&path) {
            Err(e) if e.kind() != std::io::ErrorKind::NotFound => Err(e),
            _ => Ok(()),
        },
    }).await
}

fn read_if_file(path: &Path) -> std::io::Result<Option<String>> {
    if !path.is_file() { return Ok(None); }
    std::fs::read_to_string(path).map(Some)
}

/// Backup directory name: UTC time, unique within `root`.
fn backup_name(root: &Path) -> String {
    let base = chrono::Utc::now().format("%Y%m%dT%H%M%SZ").to_string();
    let mut name = base.clone();
    let mut k = 1;
    while root.join(&name).exists() { name = format!("{base}-{k}"); k += 1; }
    name
}

/// Write a backup: `vision_intrinsics.json` (rebot-vision's GET body), copies of
/// `calibration.json` and `grasp_setup.json` when they exist, and `manifest.json` (the live
/// armd state: camera mount, calibration, grasp setup). Returns its directory.
async fn write_backup(deps: &CalibrationDeps, reason: &str, vision: &Value, state: &ArmdState) -> Result<PathBuf, String> {
    let root = deps.backup_dir.clone();
    let (calib, setup) = (deps.path.clone(), deps.setup_path.clone());
    let manifest = json!({"created_at": chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Secs, true), "reason": reason,
        "camera_mount": state.camera_mount, "calibration": state.calibration, "grasp_setup": state.grasp_setup});
    let vision = vision.clone();
    blocking(move || {
        std::fs::create_dir_all(&root)?;
        let dir = root.join(backup_name(&root));
        std::fs::create_dir(&dir)?;
        let mut files = vec!["vision_intrinsics.json", "manifest.json"];
        crate::persist::write_atomic(&dir.join("vision_intrinsics.json"), &serde_json::to_string_pretty(&vision).unwrap_or_default())?;
        if let Some(t) = read_if_file(&calib)? { crate::persist::write_atomic(&dir.join("calibration.json"), &t)?; files.push("calibration.json"); }
        if let Some(t) = setup.as_deref().map(read_if_file).transpose()?.flatten() {
            crate::persist::write_atomic(&dir.join("grasp_setup.json"), &t)?;
            files.push("grasp_setup.json");
        }
        let mut manifest = manifest;
        manifest["files"] = json!(files);
        crate::persist::write_atomic(&dir.join("manifest.json"), &serde_json::to_string_pretty(&manifest).unwrap_or_default())?;
        Ok(dir)
    }).await
}

/// Backups under the backup root, newest first: `{name, created_at, reason, files}`.
pub fn list_backups(deps: &CalibrationDeps) -> Vec<Value> {
    let Ok(rd) = std::fs::read_dir(&deps.backup_dir) else { return Vec::new(); };
    let mut out: Vec<Value> = rd.flatten().filter(|e| e.path().is_dir()).filter_map(|e| {
        let name = e.file_name().to_string_lossy().into_owned();
        let m: Value = serde_json::from_str(&std::fs::read_to_string(e.path().join("manifest.json")).ok()?).ok()?;
        Some(json!({"name": name, "created_at": m.get("created_at"), "reason": m.get("reason"), "files": m.get("files")}))
    }).collect();
    out.sort_by(|a, b| b["name"].as_str().cmp(&a["name"].as_str()));
    out
}

// ---- transaction ----

/// Re-checks the caller's lease and epoch before each write.
pub type LeaseCheck = Box<dyn Fn(&mut SupervisorInner) -> Result<(), CalibError> + Send + Sync>;

/// Under the lock, before a transaction: no other transaction, the arm idle.
pub fn begin_checks(inner: &SupervisorInner) -> Result<(), CalibError> {
    if inner.calibration_busy { return Err(CalibError::new(409, "CONFIG_BUSY", "a calibration apply or restore is running")); }
    if let Some(reason) = inner.workspace_change_blocker() {
        return Err(CalibError::new(409, "MOTION_BUSY", format!("the camera calibration cannot change while {reason}")));
    }
    if inner.config.grasp.is_none() || inner.grasp_setup().is_none() {
        return Err(CalibError::new(400, "UNSUPPORTED", "This profile has no grasp setup"));
    }
    Ok(())
}

/// Holds `calibration_busy` for a running apply / restore and clears it when released or
/// dropped, so a panic or an early return in the transaction task does not leave it set.
pub struct BusyGuard { sup: Option<Arc<ArmSupervisor>> }

impl BusyGuard {
    /// Takes over a `calibration_busy` set under the lock by the caller.
    pub fn new(sup: Arc<ArmSupervisor>) -> Self { Self { sup: Some(sup) } }

    /// Clear the flag (normal end of the transaction).
    pub async fn release(mut self) {
        if let Some(sup) = self.sup.take() { sup.inner().write().await.calibration_busy = false; }
    }
}

impl Drop for BusyGuard {
    fn drop(&mut self) {
        let Some(sup) = self.sup.take() else { return; };
        if let Ok(mut inner) = sup.inner().try_write() { inner.calibration_busy = false; return; }
        // the lock is held (for example by the code that panicked): clear it once it is free
        match tokio::runtime::Handle::try_current() {
            Ok(h) => { h.spawn(async move { sup.inner().write().await.calibration_busy = false; }); }
            Err(_) => sup.inner().blocking_write().calibration_busy = false,
        }
        tracing::warn!("calibration transaction ended without release (panic or early return); busy flag cleared");
    }
}

/// What the transaction writes and how far it got (for the revert).
struct Txn<'a> {
    sup: &'a ArmSupervisor,
    deps: &'a CalibrationDeps,
    check: &'a LeaseCheck,
    before: ArmdState,
    vision_before: Value,
    /// rebot-vision revision after our PUT (None: not written).
    vision_written: Option<u64>,
    /// Our PUT failed and the read-back failed too: rebot-vision may hold either values.
    vision_unknown: bool,
    file_written: bool,
    live_written: bool,
    backup: Option<PathBuf>,
}

impl Txn<'_> {
    async fn recheck(&self, step: &'static str) -> Result<(), CalibError> {
        let mut inner = self.sup.inner().write().await;
        (self.check)(&mut inner).map_err(|e| e.at(step))
    }

    async fn write_vision(&mut self, values: &Map<String, Value>) -> Result<(), CalibError> {
        self.recheck("vision").await?;
        match vision_put(self.deps, values, revision(&self.vision_before)).await {
            Ok(reply) => {
                self.vision_written = Some(reply.get("revision").and_then(Value::as_u64).unwrap_or(revision(&self.vision_before) + 1));
                Ok(())
            }
            Err(e) => {
                // the PUT may have been applied although its reply failed: read back
                match vision_after_error(self.deps, &self.vision_before, values).await {
                    VisionAfterError::Written(rev) => {
                        tracing::warn!("rebot-vision PUT failed ({}) but the new intrinsics are in place (revision {rev}); reverting", e.message);
                        self.vision_written = Some(rev);
                    }
                    VisionAfterError::Unchanged => {}
                    VisionAfterError::Unknown(m) => {
                        tracing::warn!("rebot-vision PUT failed ({}) and the read-back failed ({m}); vision state unknown", e.message);
                        self.vision_unknown = true;
                    }
                }
                Err(e.at("vision"))
            }
        }
    }

    /// calibration.json, then the live mount and calibration.
    async fn write_calibration(&mut self, after: &ArmdState) -> Result<(), CalibError> {
        self.recheck("calibration_file").await?;
        let text = after.calibration.as_ref().map(|c| serde_json::to_string_pretty(c).unwrap_or_default());
        put_file(self.deps.path.clone(), text).await
            .map_err(|e| CalibError::new(500, "PERSIST_FAILED", format!("calibration file {}: {e}", self.deps.path.display())).at("calibration_file"))?;
        self.file_written = true;
        let mut inner = self.sup.inner().write().await;
        inner.set_camera_mount(after.camera_mount.clone());
        inner.calibration = after.calibration.clone();
        self.live_written = true;
        Ok(())
    }

    /// The grasp setup, live and persisted (through the persist writer, as the setup API).
    async fn write_grasp(&mut self, after: &ArmdState) -> Result<(), CalibError> {
        self.recheck("grasp_setup").await?;
        let Some(setup) = after.grasp_setup.clone() else { return Ok(()); };
        let write = {
            let mut inner = self.sup.inner().write().await;
            let mut setup = setup;
            setup.camera_tilt_deg = f64::from(inner.config.camera_mount.tilt_deg);
            inner.set_grasp_setup(setup.clone());
            self.deps.setup_path.clone().map(|p| crate::persist::PendingWrite::new(p, serde_json::to_string_pretty(&setup).unwrap_or_default()))
        };
        if let Some(w) = write {
            if let Err(e) = w.run().await {
                // the file is unchanged (atomic write): put the live setup back
                let mut inner = self.sup.inner().write().await;
                if let Some(g) = self.before.grasp_setup.clone() { inner.set_grasp_setup(g); }
                return Err(CalibError::new(500, "PERSIST_FAILED", format!("grasp setup file: {e}")).at("grasp_setup"));
            }
        }
        Ok(())
    }

    /// Undo the completed steps in reverse order. Err: the state of each service.
    async fn revert(&mut self) -> Result<(), Map<String, Value>> {
        let mut state = Map::new();
        let mut ok = true;
        if self.live_written {
            let mut inner = self.sup.inner().write().await;
            inner.set_camera_mount(self.before.camera_mount.clone());
            inner.calibration = self.before.calibration.clone();
            state.insert("armd_live".into(), json!("reverted"));
        }
        if self.file_written {
            let backup_text = self.backup.as_ref().and_then(|b| read_if_file(&b.join("calibration.json")).ok().flatten());
            match put_file(self.deps.path.clone(), backup_text).await {
                Ok(()) => { state.insert("calibration_file".into(), json!("reverted")); }
                Err(e) => { ok = false; state.insert("calibration_file".into(), json!(format!("revert failed: {e}"))); }
            }
        }
        if self.vision_unknown {
            ok = false;
            state.insert("vision".into(), json!("unknown"));
        } else if let Some(rev) = self.vision_written {
            let old = self.vision_before.as_object().cloned().unwrap_or_default();
            match vision_put(self.deps, &old, rev).await {
                Ok(_) => { state.insert("vision".into(), json!("reverted")); }
                // the revert PUT may have been applied although its reply failed: read back
                Err(e) => match vision_get(self.deps).await {
                    Ok(got) if intrinsics_match(&got, &old) => { state.insert("vision".into(), json!("reverted")); }
                    Ok(_) => { ok = false; state.insert("vision".into(), json!(format!("revert failed: {}", e.message))); }
                    Err(g) => { ok = false; state.insert("vision".into(), json!(format!("unknown (revert failed: {}; read-back failed: {})", e.message, g.message))); }
                },
            }
        }
        if ok { Ok(()) } else { Err(state) }
    }

    /// A step failed: revert and return the error (CALIBRATION_INCONSISTENT when the revert failed).
    async fn fail(&mut self, mut e: CalibError) -> CalibError {
        if let Some(b) = &self.backup { e.extra.insert("backup".into(), json!(b.display().to_string())); }
        match self.revert().await {
            Ok(()) => { e.extra.insert("reverted".into(), json!(true)); e }
            Err(state) => {
                let mut c = CalibError::new(500, "CALIBRATION_INCONSISTENT",
                    format!("{} failed ({}) and the revert failed; restore the backup", e.step.unwrap_or("a step"), e.message));
                c.step = e.step;
                c.extra = e.extra;
                c.extra.insert("state".into(), Value::Object(state));
                c
            }
        }
    }

    /// Run the three writes; reply with the values read back from both services.
    async fn run(&mut self, reason: &str, vision_values: &Map<String, Value>, after: &ArmdState) -> Result<Value, CalibError> {
        let backup = write_backup(self.deps, reason, &self.vision_before, &self.before).await
            .map_err(|e| CalibError::new(500, "PERSIST_FAILED", format!("backup: {e}")).at("backup"))?;
        self.backup = Some(backup.clone());
        if let Err(e) = self.write_vision(vision_values).await { return Err(self.fail(e).await); }
        if let Err(e) = self.write_calibration(after).await { return Err(self.fail(e).await); }
        if let Err(e) = self.write_grasp(after).await { return Err(self.fail(e).await); }
        let vision = vision_get(self.deps).await.unwrap_or_else(|e| json!({"error": e.message}));
        let inner = self.sup.inner().read().await;
        Ok(json!({"backup": backup.display().to_string(), "vision_intrinsics": vision,
            "camera_mount": mount_summary(&inner.config.camera_mount), "calibration": inner.calibration,
            "grasp": inner.grasp_setup().map(|s| grasp_fields(&s))}))
    }
}

fn diff_view(vision: &Value, state: &ArmdState) -> Value {
    let mut v = json!({"vision_intrinsics": vision, "camera_mount": mount_summary(&state.camera_mount),
        "grasp": state.grasp_setup.as_ref().map(grasp_fields)});
    if let Some(p) = state.grip_pixel_model { v["grip_pixel_model"] = json!(p); }
    v
}

/// `/api/v1/calibration/apply` after [`begin_checks`] (the caller set `calibration_busy` for a
/// real apply and clears it afterwards). `dry_run` returns `{before, after}` and writes nothing.
pub async fn apply(sup: &Arc<ArmSupervisor>, deps: &CalibrationDeps, req: ApplyRequest, check: &LeaseCheck) -> Result<Value, CalibError> {
    let (before, after) = {
        let inner = sup.inner().read().await;
        (armd_state(&inner), after_state(&inner, &req)?)
    };
    let vision_before = vision_get(deps).await.map_err(|e| e.at("vision"))?;
    let mut vision_after = vision_before.as_object().cloned().unwrap_or_default();
    for (k, v) in &req.intrinsics { vision_after.insert(k.clone(), v.clone()); }
    if req.dry_run {
        let mut va = vision_after.clone();
        va.remove("revision");
        return Ok(json!({"dry_run": true, "before": diff_view(&vision_before, &before), "after": diff_view(&Value::Object(va), &after)}));
    }
    let mut txn = Txn { sup, deps, check, before, vision_before, vision_written: None, vision_unknown: false, file_written: false, live_written: false, backup: None };
    let mut out = txn.run("apply", &req.intrinsics, &after).await?;
    out["grip_pixel_model"] = json!(after.grip_pixel_model);
    tracing::info!("Camera calibration applied (backup {})", out["backup"].as_str().unwrap_or(""));
    Ok(out)
}

/// `/api/v1/calibration/restore {backup}`: put a backup back (rebot-vision intrinsics, armd's
/// camera mount / calibration.json, grasp setup), with a new backup first and the same revert.
pub async fn restore(sup: &Arc<ArmSupervisor>, deps: &CalibrationDeps, body: &Value, check: &LeaseCheck) -> Result<Value, CalibError> {
    let name = body.get("backup").and_then(Value::as_str)
        .filter(|n| !n.is_empty() && n.len() <= 64 && n.chars().all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_'))
        .ok_or_else(|| CalibError::invalid("backup must be the name of a backup (GET /api/v1/calibration)"))?.to_string();
    let dir = deps.backup_dir.join(&name);
    let read = |f: &str| -> Result<Value, CalibError> {
        let t = std::fs::read_to_string(dir.join(f)).map_err(|e| CalibError::new(404, "BACKUP_NOT_FOUND", format!("backup {name}: {f}: {e}")))?;
        serde_json::from_str(&t).map_err(|e| CalibError::invalid(format!("backup {name}: {f}: {e}")))
    };
    let manifest = read("manifest.json")?;
    let vision_target = read("vision_intrinsics.json")?;
    let target: ArmdState = serde_json::from_value(manifest.clone()).map_err(|e| CalibError::invalid(format!("backup {name}: manifest.json: {e}")))?;
    let vision_values = validate_intrinsics(&vision_target).map_err(|e| CalibError::invalid(format!("backup {name}: vision_intrinsics.json: {e}")))?;
    if let Some(c) = &target.calibration { c.validate().map_err(CalibError::invalid)?; }
    if let Some(g) = &target.grasp_setup { g.validate().map_err(|e| CalibError::invalid(format!("backup {name}: grasp setup: {e}")))?; }
    let before = armd_state(&*sup.inner().read().await);
    let vision_before = vision_get(deps).await.map_err(|e| e.at("vision"))?;
    let mut txn = Txn { sup, deps, check, before, vision_before, vision_written: None, vision_unknown: false, file_written: false, live_written: false, backup: None };
    let mut out = txn.run("restore", &vision_values, &target).await?;
    out["restored"] = json!(name);
    tracing::info!("Camera calibration restored from backup {name}");
    Ok(out)
}

/// `GET /api/v1/calibration`.
pub async fn status(sup: &ArmSupervisor, deps: &CalibrationDeps) -> Value {
    let inner = sup.inner().read().await;
    json!({"calibrated": inner.camera_calibrated(), "busy": inner.calibration_busy,
        "path": deps.path.display().to_string(), "backup_dir": deps.backup_dir.display().to_string(),
        "calibration": inner.calibration, "camera_mount": mount_summary(&inner.config.camera_mount),
        "grasp": inner.grasp_setup().map(|s| grasp_fields(&s)), "backups": list_backups(deps)})
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kinematics as k;

    #[test]
    fn grip_pixel_matches_a_direct_projection() {
        let setup = GraspSetup { table_z_m: 0.01, approach_height_m: 0.15, grasp_height_m: 0.015, fx_px: 820.0, fy_px: 818.0,
            cx_px: Some(318.0), cy_px: Some(183.0), ..Default::default() };
        let mount = CameraMount { offset_m: Some([-0.0731, 0.0005, 0.066]), q_end_cam: Some(k::r_to_quat(&k::cam_mount(false, 0.0, -38.0))), ..Default::default() };
        let cam = CamModel::assumed(&crate::chain::ArmModel::REBOT_DEV_ARM, &setup, &mount);
        let got = grip_pixel_norm(&cam, &setup).unwrap();
        // end_link frame: the grasp-plane point is (approach - grasp) along end_link x
        let d = setup.approach_height_m - setup.grasp_height_m;
        let pc = k::mat_vec(&k::transpose(&cam.rec), &k::sub(&[d, 0.0, 0.0], &cam.offset));
        let want = [(820.0 * pc[0] / pc[2] + 318.0) / 640.0, (818.0 * pc[1] / pc[2] + 183.0) / 360.0];
        assert!((got[0] - want[0]).abs() < 1e-4 && (got[1] - want[1]).abs() < 1e-4, "{got:?} vs {want:?}");
        // the yaw of the look-down pose does not matter
        let yawed = k::Pose::new(k::mat_mul(&k::axis_r(&[0.0, 0.0, 1.0], 0.7), &crate::grasp::look_down(&k::IDENTITY)), [0.2, -0.1, 0.16]);
        let p = cam.project(&yawed, &[0.2, -0.1, 0.025]).unwrap();
        assert!((p[0] / 640.0 - want[0]).abs() < 1e-6 && (p[1] / 360.0 - want[1]).abs() < 1e-6);
    }

    #[test]
    fn intrinsics_validation() {
        let ok = json!({"fx": 824.0, "fy": 824.0, "cx": 320.0, "cy": 180.0, "k1": 0.0, "k2": 0.0, "p1": 0.0, "p2": 0.0, "k3": 0.0, "width": 640, "height": 360});
        assert!(validate_intrinsics(&ok).is_ok());
        for (k, v) in [("fx", json!(50.0)), ("cx", json!(640.0)), ("k1", json!(11.0)), ("p2", json!(1.5)), ("width", json!(1280)), ("fy", json!(null))] {
            let mut b = ok.clone();
            b[k] = v;
            assert!(validate_intrinsics(&b).unwrap_err().contains(k) || k == "width", "{k}");
        }
    }
}
