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
//!
//! `calibration.json` may also hold `kinematics`, the kinematic fine calibration of the
//! built-in chain ([`crate::kinematics::JointCorrection`] per joint). It is written in the same
//! transaction and installed live in `config.arm_model` ([`SupervisorInner::set_kinematics_correction`]);
//! changing it needs torque off and no action running (KINEMATICS_ARMED).
//!
//! Self-service verification (T20): `tool` holds the fingertip measured with the 4-dot sticker
//! (`tip_cam_m`, camera frame) bound to the intrinsics and X it was measured with
//! ([`x_fingerprint`]); `acceptance` holds the automatic hover acceptance bound to
//! [`fingerprint`] (intrinsics, X, kinematics, joint zero, tool); `table_z_source` says whether
//! `table_z_m` came from the board solve (`camera`) or the hovers (`hover`). A dry-run apply
//! reports the candidate's fingerprints; an apply with a `tool` / `acceptance` whose binding does
//! not match the candidate is refused (TOOL_STALE / ACCEPTANCE_STALE). A zero marks the
//! acceptance invalidated; validity is always rechecked against the live state, so a restored
//! backup measured with another joint zero is not valid. The effective floor
//! ([`effective_floor`]) follows a valid calibrated table height.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use axum::http::StatusCode;
use serde::{Deserialize, Serialize};
use serde_json::{json, Map, Value};

use crate::api::proxy::DetectorUpstream;
use crate::grasp::{CamModel, GraspSetup};
use crate::kinematics::Correction;
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
    /// Kinematic fine calibration of the built-in chain; None = the URDF chain.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub kinematics: Option<KinematicsCalibration>,
    /// Set by a motor zero: the camera calibration was measured with the old zero. Grasp keeps
    /// using it; clients warn. A new apply clears it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stale: Option<CalibrationStale>,
    /// Joint zero offsets against the profile's `zero_reference` (src/joint_zero.rs); None =
    /// offsets 0. A file may hold only this (no camera calibration).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub joint_zero: Option<crate::joint_zero::JointZero>,
    /// Where `table_z_m` comes from: `camera` (the board solve) or `hover` (the automatic
    /// acceptance measured the fingertip above the board); None = a file written before T20
    /// (treated as `camera`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub table_z_source: Option<String>,
    /// Fingertip measured with the sticker, bound to the camera calibration it was measured with.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tool: Option<ToolCalibration>,
    /// Automatic hover acceptance of this calibration; None = unknown (legacy or not run).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub acceptance: Option<AcceptanceRecord>,
    /// Device qualification of MIT joints (`src/mit.rs`), bound to the profile, joint zero and
    /// motor identity; rechecked against a fresh readback on every arm. A file may hold only
    /// this. Not part of a camera apply or restore (the live record is kept). Kept unparsed:
    /// an invalid record never invalidates the rest of the file; it is flagged at load and
    /// refuses MIT ([`CalibrationFile::mit_record`]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mit_qualification: Option<Value>,
}

/// `calibration.json` `tool`: the fingertip measured with the 4-dot sticker. `tip_end = X · tip_cam`
/// is derived on demand; the record is valid only while `x_fingerprint` matches the camera
/// calibration in effect ([`x_fingerprint`]).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ToolCalibration {
    /// The profile's `calibration_tool.tool_id` the sticker sat on (or the client's id).
    pub tool_id: String,
    /// Sticker origin (bottom-edge centre, flush with the fingertip) in the camera frame (m).
    pub tip_cam_m: [f64; 3],
    /// Sticker edge above the tool's lowest point (m), from the profile's `calibration_tool`
    /// when the ids match; None = unknown tool.
    #[serde(default)]
    pub edge_to_low_m: Option<f64>,
    /// Gripper opening (fraction 0 closed .. 1 open) during the measurement.
    #[serde(default)]
    pub gripper_opening: Option<f64>,
    /// UTC time of the measurement (RFC 3339).
    pub measured_at: String,
    /// [`x_fingerprint`] of the intrinsics and camera mount the tip was measured with.
    pub x_fingerprint: String,
    /// Measurement report (frames, reprojection, ambiguity gap, std), passed through.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fit: Option<Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tool_config_digest: Option<String>,
}

/// `calibration.json` `acceptance`: the automatic hover acceptance (tool-down hovers over board
/// corners, camera-measured fingertip height above the paper).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AcceptanceRecord {
    /// [`fingerprint`] of the calibration (intrinsics, X, kinematics, joint zero, tool) it accepted.
    pub fingerprint: String,
    /// Version of the client's gates.
    pub gate_version: u32,
    pub passed: bool,
    /// UTC time (RFC 3339).
    pub measured_at: String,
    /// Board identity (`cols`, `rows`, `marker_side_m`, `gap_m`), passed through.
    pub board: Value,
    /// Per-point results and observations (at most [`MAX_ACCEPTANCE_POINTS`]), passed through.
    pub points: Vec<Value>,
    /// Summary (height spread, max XY error, `table_z_m`, `optical_table_z_m`, gates), passed through.
    pub summary: Value,
    /// The device-side hover job the observations came from.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub job_id: Option<String>,
    /// Set by a zero: measured with the old zero.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub invalidated: Option<CalibrationStale>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tool_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tool_config_digest: Option<String>,
}

/// Most points in an acceptance record.
pub const MAX_ACCEPTANCE_POINTS: usize = 16;
/// Largest serialized acceptance record (bytes).
const MAX_ACCEPTANCE_BYTES: usize = 64 * 1024;
/// The effective floor sits this far below a valid calibrated table height (m).
pub const FLOOR_BELOW_TABLE_M: f64 = 0.003;

// ---- fingerprints ----

/// FNV-1a 64 of `s`, 16 hex digits (change detection, not security).
fn fnv64(s: &str) -> String {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in s.bytes() { h ^= u64::from(b); h = h.wrapping_mul(0x0000_0100_0000_01b3); }
    format!("{h:016x}")
}

/// Fingerprint of a camera calibration: the intrinsics (fx..k3) and X = T_end_cam
/// (`q_end_cam`, `offset_m`). A fingertip measurement is bound to it.
pub fn x_fingerprint(intrinsics: Option<&Value>, mount: &CameraMount) -> String {
    let intr: Vec<Value> = INTRINSIC_KEYS.iter().map(|k| intrinsics.and_then(|i| i.get(*k)).and_then(Value::as_f64).map_or(Value::Null, |v| json!(v))).collect();
    fnv64(&format!("x1|{}|{}|{}", Value::Array(intr), json!(mount.q_end_cam), json!(mount.offset_m)))
}

/// Fingerprint of everything an acceptance depends on: [`x_fingerprint`], the kinematic
/// correction, the joint zero (offsets, method, time), the measured tool record, and the active
/// tool configuration digest.
pub fn fingerprint(
    x_fp: &str,
    kin: Option<&Correction>,
    offsets: &[f32],
    zero: Option<&crate::joint_zero::JointZero>,
    tool: Option<&ToolCalibration>,
    active_tool_digest: Option<&str>,
) -> String {
    let z = zero.map(|z| json!([z.method, z.set_at]));
    let t = tool.map(|t| json!([t.tool_id, t.tip_cam_m, t.gripper_opening, t.edge_to_low_m, t.tool_config_digest]));
    fnv64(&format!("f1|{x_fp}|{}|{}|{}|{}|{}", json!(kin), json!(offsets), json!(z), json!(t), json!(active_tool_digest)))
}

/// Legacy pre-tool-pack fingerprint (omitting `tool_config_digest` and `active_tool_digest`),
/// accepted only for legacy unkeyed stock `AcceptanceRecord` entries.
pub fn legacy_fingerprint(
    x_fp: &str,
    kin: Option<&Correction>,
    offsets: &[f32],
    zero: Option<&crate::joint_zero::JointZero>,
    tool: Option<&ToolCalibration>,
) -> String {
    let z = zero.map(|z| json!([z.method, z.set_at]));
    let t = tool.map(|t| json!([t.tool_id, t.tip_cam_m, t.gripper_opening, t.edge_to_low_m]));
    fnv64(&format!("f1|{x_fp}|{}|{}|{}|{}", json!(kin), json!(offsets), json!(z), json!(t)))
}

/// (x_fingerprint, fingerprint) of a calibration file with the camera mount `mount`, the live
/// joint zero offsets, and the active tool digest.
pub fn file_fingerprints(
    c: Option<&CalibrationFile>,
    mount: &CameraMount,
    offsets: &[f32],
    active_tool_digest: Option<&str>,
) -> (String, String) {
    let x = x_fingerprint(c.and_then(|c| c.intrinsics.as_ref()), mount);
    let f = fingerprint(&x, correction_of(c), offsets, c.and_then(|c| c.joint_zero.as_ref()), c.and_then(|c| c.tool.as_ref()), active_tool_digest);
    (x, f)
}

/// Legacy `(x_fingerprint, legacy_fingerprint)` for stock upgrade compatibility.
pub fn legacy_file_fingerprint(c: Option<&CalibrationFile>, mount: &CameraMount, offsets: &[f32]) -> String {
    let x = x_fingerprint(c.and_then(|c| c.intrinsics.as_ref()), mount);
    legacy_fingerprint(&x, correction_of(c), offsets, c.and_then(|c| c.joint_zero.as_ref()), c.and_then(|c| c.tool.as_ref()))
}

/// (x_fingerprint, fingerprint) of the calibration in effect.
pub fn live_fingerprints(inner: &SupervisorInner) -> (String, String) {
    file_fingerprints(
        inner.calibration.as_ref(),
        &inner.config.camera_mount,
        &inner.joint_zero_offsets,
        Some(&inner.config.active_tool.config_digest),
    )
}

/// The stored fingertip is valid: bound to the camera calibration whose x fingerprint is `x_fp`.
pub fn tool_valid(c: &CalibrationFile, x_fp: &str) -> bool { c.has_camera() && c.tool.as_ref().is_some_and(|t| t.x_fingerprint == x_fp) }

/// [`tool_valid`] plus active-tool `(tool_id, tool_config_digest)` match.
pub fn tool_valid_for(c: &CalibrationFile, x_fp: &str, active_tool: &crate::tool::ActiveToolInfo) -> bool {
    c.has_camera()
        && c.tool
            .as_ref()
            .is_some_and(|t| t.x_fingerprint == x_fp && active_tool.matches_tool_calib(&t.tool_id, t.tool_config_digest.as_deref()))
}

/// The stored acceptance is valid: passed, not invalidated, the calibration not stale and its
/// fingerprint `fp` unchanged.
pub fn acceptance_valid(c: &CalibrationFile, fp: &str) -> bool {
    c.has_camera() && c.stale.is_none() && c.acceptance.as_ref().is_some_and(|a| a.passed && a.invalidated.is_none() && a.fingerprint == fp)
}

/// [`acceptance_valid`] plus active-tool `(tool_id, tool_config_digest)` match.
pub fn acceptance_valid_for(c: &CalibrationFile, fp: &str, active_tool: &crate::tool::ActiveToolInfo) -> bool {
    acceptance_valid_for_with_legacy(c, fp, None, active_tool)
}

/// [`acceptance_valid_for`] additionally accepting `legacy_fp` when `active_tool.is_stock`
/// and the stored `AcceptanceRecord` has no tool metadata (`tool_id == None && tool_config_digest == None`).
pub fn acceptance_valid_for_with_legacy(
    c: &CalibrationFile,
    fp: &str,
    legacy_fp: Option<&str>,
    active_tool: &crate::tool::ActiveToolInfo,
) -> bool {
    c.has_camera()
        && c.stale.is_none()
        && c.acceptance.as_ref().is_some_and(|a| {
            let fp_ok = a.fingerprint == fp
                || (active_tool.is_stock
                    && a.tool_id.is_none()
                    && a.tool_config_digest.is_none()
                    && legacy_fp.is_some_and(|lfp| a.fingerprint == lfp));
            a.passed
                && a.invalidated.is_none()
                && fp_ok
                && active_tool.matches_record(a.tool_id.as_deref(), a.tool_config_digest.as_deref())
        })
}

/// Effective floor (design T20 D): `max(profile floor, valid calibrated table_z_m - 3 mm)`.
/// A table height is valid when the camera calibration is in effect and not stale, and, for
/// `table_z_source: hover`, its acceptance is valid. Returns (floor, source, table_z used).
pub fn effective_floor(profile_floor: f32, c: Option<&CalibrationFile>, fp: &str) -> (f32, &'static str, Option<f64>) {
    effective_floor_inner(profile_floor, c, fp, None, None)
}

/// [`effective_floor`] enforcing active-tool matching on hover acceptance.
pub fn effective_floor_for(
    profile_floor: f32,
    c: Option<&CalibrationFile>,
    fp: &str,
    active_tool: &crate::tool::ActiveToolInfo,
) -> (f32, &'static str, Option<f64>) {
    effective_floor_inner(profile_floor, c, fp, None, Some(active_tool))
}

/// [`effective_floor_for`] with optional legacy pre-tool-pack fingerprint for stock upgrade compatibility.
pub fn effective_floor_for_with_legacy(
    profile_floor: f32,
    c: Option<&CalibrationFile>,
    fp: &str,
    legacy_fp: Option<&str>,
    active_tool: &crate::tool::ActiveToolInfo,
) -> (f32, &'static str, Option<f64>) {
    effective_floor_inner(profile_floor, c, fp, legacy_fp, Some(active_tool))
}

fn effective_floor_inner(
    profile_floor: f32,
    c: Option<&CalibrationFile>,
    fp: &str,
    legacy_fp: Option<&str>,
    active_tool: Option<&crate::tool::ActiveToolInfo>,
) -> (f32, &'static str, Option<f64>) {
    let table = c.filter(|c| c.has_camera() && c.stale.is_none()).and_then(|c| {
        let z = c.table_z_m.filter(|z| z.is_finite())?;
        match c.table_z_source.as_deref() {
            Some("hover") => {
                let ok = match active_tool {
                    Some(t) => acceptance_valid_for_with_legacy(c, fp, legacy_fp, t),
                    None => acceptance_valid(c, fp),
                };
                ok.then_some((z, "calibration_hover"))
            }
            _ => Some((z, "calibration_camera")),
        }
    });
    match table {
        Some((z, src)) if (z - FLOOR_BELOW_TABLE_M) as f32 > profile_floor => ((z - FLOOR_BELOW_TABLE_M) as f32, src, Some(z)),
        Some((z, _)) => (profile_floor, "profile", Some(z)),
        None => (profile_floor, "profile", None),
    }
}

/// Strip tool-dependent fingertip and hover acceptance records that belong to another tool
/// while preserving `camera_mount`, `intrinsics`, `kinematics`, and `joint_zero`.
pub fn strip_mismatched_tool_records(file: &mut CalibrationFile, active_tool: &crate::tool::ActiveToolInfo) {
    if let Some(ref t) = file.tool {
        if !active_tool.matches_tool_calib(&t.tool_id, t.tool_config_digest.as_deref()) {
            tracing::warn!(
                "Dropping calibration fingertip record for tool '{}' ({:?}); active tool is '{}' ({})",
                t.tool_id, t.tool_config_digest, active_tool.id, active_tool.config_digest
            );
            file.tool = None;
        }
    }
    if let Some(ref a) = file.acceptance {
        if !active_tool.matches_record(a.tool_id.as_deref(), a.tool_config_digest.as_deref()) {
            tracing::warn!(
                "Dropping calibration hover acceptance for tool {:?}/{:?}; active tool is '{}' ({})",
                a.tool_id, a.tool_config_digest, active_tool.id, active_tool.config_digest
            );
            file.acceptance = None;
            if file.table_z_source.as_deref() == Some("hover") {
                file.table_z_source = None;
                file.table_z_m = None;
            }
        }
    }
}

/// `calibration.json` `stale`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CalibrationStale {
    pub reason: String,
    /// UTC time (RFC 3339).
    pub at: String,
}

/// `calibration.json` `kinematics` (and the apply field of the same name).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct KinematicsCalibration {
    /// J1..J6 (design "Correction model"); joints the solver does not identify are zeros.
    pub joints: Correction,
    /// Fit report of the solver (`heldout_mm`, `train_mm`, `views`, `solved_at`), passed through.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fit: Option<Value>,
}

impl KinematicsCalibration {
    pub fn validate(&self) -> Result<(), String> {
        crate::kinematics::validate_correction(&self.joints)?;
        if self.fit.as_ref().is_some_and(|f| !f.is_object()) { return Err("kinematics.fit must be an object".into()); }
        Ok(())
    }
}

/// The correction a calibration installs (None without one).
fn correction_of(c: Option<&CalibrationFile>) -> Option<&Correction> { c.and_then(|c| c.kinematics.as_ref()).map(|k| &k.joints) }

impl CalibrationFile {
    /// The file holds a camera calibration (`camera_mount.q_end_cam`).
    pub fn has_camera(&self) -> bool { self.camera_mount.q_end_cam.is_some() }

    /// The MIT qualification record, parsed and validated.
    pub fn mit_record(&self) -> Result<Option<crate::mit::MitQualificationFile>, String> {
        let Some(v) = &self.mit_qualification else { return Ok(None) };
        let rec: crate::mit::MitQualificationFile = serde_json::from_value(v.clone()).map_err(|e| format!("mit_qualification: {e}"))?;
        rec.validate()?;
        Ok(Some(rec))
    }

    /// 1-based joints listed by the MIT record, also when it does not validate (None when the
    /// list cannot be read).
    pub fn mit_record_raw_joints(&self) -> Option<Vec<usize>> {
        self.mit_qualification.as_ref()?.get("joints")?.as_array()?.iter()
            .map(|j| j.get("joint").and_then(Value::as_u64).map(|n| n as usize)).collect()
    }

    /// Store `rec` as the MIT qualification record.
    pub fn set_mit_record(&mut self, rec: &crate::mit::MitQualificationFile) {
        self.mit_qualification = serde_json::to_value(rec).ok();
    }

    pub fn validate(&self) -> Result<(), String> {
        if let Some(z) = &self.joint_zero { z.validate(None).map_err(|e| format!("calibration.json: {e}"))?; }
        let m = &self.camera_mount;
        if m.q_end_cam.is_none() && m.offset_m.is_none() && self.intrinsics.is_none()
            && (self.joint_zero.is_some() || self.mit_qualification.is_some()) {
            // joint zero and / or MIT qualification only
            if let Some(k) = &self.kinematics { k.validate().map_err(|e| format!("calibration.json: {e}"))?; }
            return Ok(());
        }
        let Some(q) = &m.q_end_cam else { return Err("calibration.json: camera_mount.q_end_cam is required".into()); };
        crate::model::unit_quaternion(q)?;
        if !m.offset_m.is_some_and(|o| o.iter().all(|v| v.is_finite() && v.abs() <= 0.3)) {
            return Err("calibration.json: camera_mount.offset_m must be three values within 0.3 m".into());
        }
        if self.table_z_m.is_some_and(|z| !z.is_finite() || z.abs() > 0.5) {
            return Err("calibration.json: table_z_m must be in [-0.5, 0.5] m".into());
        }
        if let Some(k) = &self.kinematics { k.validate().map_err(|e| format!("calibration.json: {e}"))?; }
        if let Some(s) = &self.table_z_source { if !["camera", "hover"].contains(&s.as_str()) { return Err("calibration.json: table_z_source must be \"camera\" or \"hover\"".into()); } }
        if let Some(t) = &self.tool { validate_tool(t).map_err(|e| format!("calibration.json: {e}"))?; }
        if let Some(a) = &self.acceptance { validate_acceptance(a).map_err(|e| format!("calibration.json: {e}"))?; }
        Ok(())
    }
}

fn validate_tool(t: &ToolCalibration) -> Result<(), String> {
    if !crate::model::valid_tool_id(&t.tool_id) { return Err("tool.tool_id must be 1..64 of A-Z a-z 0-9 _ - .".into()); }
    if !t.tip_cam_m.iter().all(|v| v.is_finite() && v.abs() <= 0.3) || t.tip_cam_m[2] <= 0.0 {
        return Err("tool.tip_cam_m must be three values within 0.3 m, in front of the camera (z > 0)".into());
    }
    if t.edge_to_low_m.is_some_and(|e| !e.is_finite() || !(-0.10..=0.10).contains(&e)) { return Err("tool.edge_to_low_m must be in [-0.10, 0.10] m".into()); }
    if t.gripper_opening.is_some_and(|g| !g.is_finite() || !(0.0..=1.0).contains(&g)) { return Err("tool.gripper_opening must be in [0, 1]".into()); }
    if t.x_fingerprint.is_empty() || t.x_fingerprint.len() > 64 || t.measured_at.len() > 64 { return Err("tool.x_fingerprint / measured_at: at most 64 bytes".into()); }
    if t.fit.as_ref().is_some_and(|f| !f.is_object()) { return Err("tool.fit must be an object".into()); }
    Ok(())
}

fn validate_acceptance(a: &AcceptanceRecord) -> Result<(), String> {
    if a.fingerprint.is_empty() || a.fingerprint.len() > 64 || a.measured_at.len() > 64 { return Err("acceptance.fingerprint / measured_at: 1..64 bytes".into()); }
    if a.points.len() > MAX_ACCEPTANCE_POINTS { return Err(format!("acceptance.points: at most {MAX_ACCEPTANCE_POINTS}")); }
    if !a.board.is_object() || !a.summary.is_object() || a.points.iter().any(|p| !p.is_object()) { return Err("acceptance.board, summary and points must be objects".into()); }
    if serde_json::to_string(a).map_or(usize::MAX, |t| t.len()) > MAX_ACCEPTANCE_BYTES { return Err(format!("acceptance: at most {MAX_ACCEPTANCE_BYTES} bytes")); }
    Ok(())
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
    /// Stored calibration sessions (`ARMD_CALIBRATION_SESSIONS`, default `calibration-sessions/`
    /// next to `path`; [`crate::calibration_sessions`]).
    pub sessions_dir: PathBuf,
}

/// `ARMD_CALIBRATION` or [`DEFAULT_PATH`].
pub fn path_from_env() -> PathBuf { crate::persist::path_from_env("ARMD_CALIBRATION", DEFAULT_PATH) }

impl CalibrationDeps {
    pub fn new(path: PathBuf, setup_path: Option<PathBuf>, upstream: DetectorUpstream) -> Self {
        let parent = path.parent().unwrap_or(Path::new("."));
        let (backup_dir, sessions_dir) = (parent.join("calibration-backups"), parent.join("calibration-sessions"));
        Self { path, backup_dir, setup_path, upstream, vision_timeout: VISION_TIMEOUT, sessions_dir }
    }

    /// Paths from `ARMD_CALIBRATION` / `ARMD_CALIBRATION_BACKUPS` / `ARMD_CALIBRATION_SESSIONS`.
    pub fn from_env(setup_path: Option<PathBuf>, upstream: DetectorUpstream) -> Self {
        let mut d = Self::new(path_from_env(), setup_path, upstream);
        if let Some(b) = std::env::var_os("ARMD_CALIBRATION_BACKUPS") { d.backup_dir = PathBuf::from(b); }
        if let Some(b) = std::env::var_os("ARMD_CALIBRATION_SESSIONS") { d.sessions_dir = PathBuf::from(b); }
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

fn apply_pending_joint_zero(
    inner: &mut SupervisorInner,
    path: &Path,
    file: &mut CalibrationFile,
    offsets: Option<&[f32]>,
) {
    let Some(offsets) = offsets else { return; };
    if let Err(e) = inner.set_joint_zero_offsets(offsets) {
        tracing::warn!("Ignoring the joint zero in {}: {e}; offsets 0", path.display());
        inner.calibration_load_error = Some(format!("calibration.json: joint_zero: {e}; offsets 0 are used"));
        file.joint_zero = None;
    } else if let Some(z) = &file.joint_zero {
        tracing::info!(
            "Joint zero loaded from {} (method {}, set {}): offsets {:?} rad",
            path.display(),
            z.method,
            z.set_at.as_deref().unwrap_or("?"),
            z.offsets()
        );
    }
}

/// Read `calibration.json` at startup (after the grasp setup) and install its camera mount.
/// A malformed or invalid file is moved aside and the profile mount stays; the grasp setup's
/// camera model (fx_px, fy_px, cx_px, cy_px, image_size_px, camera_tilt_deg, grip_pixel_norm),
/// which an apply wrote for the calibrated mount, is reset to the profile values (live and in
/// `setup_path`, the old file kept as `<name>.pre-reset-<ts>`) so no grasp mixes a calibrated
/// camera model with the profile mount. The error is reported in `GET /api/v1/calibration`
/// (`load_error`) until an apply or restore succeeds.
pub async fn load_into(supervisor: &ArmSupervisor, path: &Path, setup_path: Option<&Path>) -> Result<(), String> {
    let loaded = crate::persist::load::<CalibrationFile>(path, CalibrationFile::validate);
    let mut inner = supervisor.inner().write().await;
    match loaded {
        crate::persist::Loaded::Loaded(mut file) => {
            strip_mismatched_tool_records(&mut file, &inner.config.active_tool);
            if let Err(e) = file.mit_record() {
                // the record alone is refused (MIT stays refused); the rest of the file is used
                tracing::warn!("Ignoring the MIT qualification in {}: {e}; MIT tracking is refused", path.display());
                inner.calibration_load_error = Some(format!("calibration.json: mit_qualification ignored ({e}); MIT tracking is refused"));
            }
            let mut pending_jz_err: Option<String> = None;
            let pending_joint_zero = if let Some(z) = &file.joint_zero {
                match z.validate(Some(inner.config.dof)) {
                    Ok(()) if !inner.is_torqued && !inner.disable_pending => Some(z.offsets()),
                    Ok(()) => {
                        tracing::warn!("Ignoring the joint zero in {}: torque is on; offsets 0", path.display());
                        pending_jz_err = Some("calibration.json: joint_zero: torque is on; offsets 0 are used".into());
                        file.joint_zero = None;
                        None
                    }
                    Err(e) => {
                        tracing::warn!("Ignoring the joint zero in {}: {e}; offsets 0", path.display());
                        pending_jz_err = Some(format!("calibration.json: joint_zero: {e}; offsets 0 are used"));
                        file.joint_zero = None;
                        None
                    }
                }
            } else {
                None
            };
            if !file.has_camera() {
                if let Some(err) = pending_jz_err.take() {
                    inner.calibration_load_error = Some(err);
                }
                apply_pending_joint_zero(&mut inner, path, &mut file, pending_joint_zero.as_deref());
                if let Some(k) = &file.kinematics {
                    if let Err(e) = inner.set_kinematics_correction(Some(&k.joints)) {
                        tracing::warn!("Ignoring the kinematic correction in {}: {e}", path.display());
                        file.kinematics = None;
                    }
                }
                inner.set_calibration(Some(file));
                return Ok(());
            }
            let q = crate::model::unit_quaternion(&file.camera_mount.q_end_cam.unwrap_or([1.0, 0.0, 0.0, 0.0])).unwrap_or([1.0, 0.0, 0.0, 0.0]);
            let mount = calibrated_mount(&inner, file.camera_mount.offset_m.unwrap_or([0.0; 3]), q);
            file.camera_mount = mount.clone();

            // Pre-validate candidate kinematics and synced grasp_setup BEFORE committing any live state.
            let mut pending_kin_err: Option<String> = None;
            let candidate_arm_model = if let Some(k) = &file.kinematics {
                match inner.config.arm_model.with_correction(Some(&k.joints)) {
                    Ok(m) => Some(m),
                    Err(e) => {
                        tracing::warn!("Ignoring the kinematic correction in {}: {e}; using the uncorrected model", path.display());
                        pending_kin_err = Some(format!("calibration.json: kinematics: {e}; the uncorrected model is used"));
                        file.kinematics = None;
                        None
                    }
                }
            } else {
                None
            };
            let candidate_grasp = if inner.grasp_setup_needs_camera_sync {
                if let Some(mut setup) = inner.config.grasp.clone() {
                    let arm_ref = candidate_arm_model.as_ref().unwrap_or(&inner.config.arm_model);
                    let tcp_offset = inner.config.tcp_offset;
                    let base = setup.clone();
                    refresh_grasp_setup_camera_fields(
                        &mut setup,
                        file.intrinsics.as_ref(),
                        &mount,
                        file.table_z_m,
                        arm_ref,
                        &tcp_offset,
                    );
                    // persisted intrinsics an older armd accepted (cx 0, ...) must not stop the
                    // start: sync without them, else keep the setup as it is
                    match setup.validate() {
                        Ok(()) => Some(setup),
                        Err(e) => {
                            tracing::warn!("calibration.json: grasp setup synced with its intrinsics is invalid ({e}); syncing without the intrinsics");
                            let mut setup = base;
                            refresh_grasp_setup_camera_fields(&mut setup, None, &mount, file.table_z_m, arm_ref, &tcp_offset);
                            match setup.validate() {
                                Ok(()) => Some(setup),
                                Err(e) => {
                                    tracing::warn!("calibration.json: grasp setup camera sync skipped ({e})");
                                    None
                                }
                            }
                        }
                    }
                } else {
                    None
                }
            } else {
                None
            };

            if let Some(err) = pending_jz_err.or(pending_kin_err) {
                inner.calibration_load_error = Some(err);
            }
            apply_pending_joint_zero(&mut inner, path, &mut file, pending_joint_zero.as_deref());

            tracing::info!("Camera calibration loaded from {}: q_end_cam {:?}, offset_m {:?}, derived tilt {:.2} deg (overrides the profile mount and the grasp setup's camera_tilt_deg)",
                path.display(), mount.q_end_cam, mount.offset_m, mount.tilt_deg);
            inner.set_camera_mount(mount);
            if let Some(ref k) = file.kinematics {
                if let Ok(()) = inner.set_kinematics_correction(Some(&k.joints)) {
                    let fit = k.fit.as_ref().map_or_else(|| "none".to_string(), |f| f.to_string());
                    tracing::info!("Kinematic correction loaded from {} (fit {fit})", path.display());
                }
            }
            if inner.grasp_setup_needs_camera_sync {
                if let Some(setup) = candidate_grasp {
                    inner.set_grasp_setup(setup);
                }
                inner.grasp_setup_needs_camera_sync = false;
            }
            inner.set_calibration(Some(file));
            Ok(())
        }
        crate::persist::Loaded::Missing => {
            tracing::info!("No camera calibration at {}; using the profile camera mount", path.display());
            Ok(())
        }
        crate::persist::Loaded::Invalid(e) => {
            tracing::warn!("Ignoring camera calibration file: {e}; using the profile camera mount");
            let mut msg = e;
            if let Some(setup) = compute_reset_camera_model(&inner) {
                let dir_sync = match setup_path {
                    Some(p) => match persist_reset_setup_rename(p, &setup, &inner.config.active_tool) {
                        Ok(ds) => Some(ds),
                        Err(persist_err) => {
                            tracing::error!("Aborting startup grasp setup reset (persist failed: {persist_err})");
                            msg.push_str(&format!("; grasp setup reset aborted (not saved: {persist_err})"));
                            inner.calibration_load_error = Some(msg.clone());
                            return Err(msg);
                        }
                    },
                    None => None,
                };
                let prof_mount = inner.profile_camera_mount.clone();
                inner.set_camera_mount(prof_mount);
                inner.set_grasp_setup(setup);
                inner.grasp_setup_revision += 1;
                msg.push_str("; grasp setup camera model reset to the profile values");
                tracing::warn!("Grasp setup camera model reset to the profile values (calibration.json rejected)");
                if let Some(ds) = dir_sync {
                    inner.calibration_load_error = Some(msg);
                    drop(inner);
                    if let crate::persist::PublishOutcome::PublishedDurabilityUnconfirmed(w) = ds.sync_parent_dir() {
                        let mut inner = supervisor.inner().write().await;
                        if let Some(ref mut cur_err) = inner.calibration_load_error {
                            cur_err.push_str(&format!(" (durability unconfirmed: {w})"));
                        }
                    }
                    return Ok(());
                }
            }
            inner.calibration_load_error = Some(msg);
            Ok(())
        }
    }
}

/// Compute the grasp setup with the camera-model fields of the profile without mutating `inner`;
/// None when nothing changes (or the profile has no grasp setup).
fn compute_reset_camera_model(inner: &SupervisorInner) -> Option<GraspSetup> {
    let (cur, prof) = (inner.grasp_setup()?, inner.profile_grasp.clone()?);
    let setup = GraspSetup { fx_px: prof.fx_px, fy_px: prof.fy_px, cx_px: prof.cx_px, cy_px: prof.cy_px, image_size_px: prof.image_size_px,
        grip_pixel_norm: prof.grip_pixel_norm, camera_tilt_deg: f64::from(inner.profile_camera_mount.tilt_deg), ..cur.clone() };
    if setup == cur { return None; }
    Some(setup)
}

/// Keep a copy of `grasp_setup.json`, then stage and atomically rename the reset setup
/// stamped with active-tool provenance, returning `PendingDirSync` so directory `sync_all()` runs
/// outside the supervisor lock.
fn persist_reset_setup_rename(
    path: &Path,
    setup: &GraspSetup,
    active_tool: &crate::tool::ActiveToolInfo,
) -> Result<crate::persist::PendingDirSync, String> {
    if path.is_file() {
        let name = path.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
        std::fs::copy(path, path.with_file_name(format!("{name}.pre-reset-{}", chrono::Utc::now().format("%Y%m%dT%H%M%SZ"))))
            .map_err(|e| e.to_string())?;
    }
    let staged = crate::persist::StagedWrite::stage(
        path.to_path_buf(),
        crate::grasp::serialize_setup_for_tool(setup, active_tool),
    )?;
    staged.rename_only()
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
    /// `kinematics`: None = absent (a stored correction stays), Some(None) = null (removes it),
    /// Some(Some(k)) = install `k`.
    pub kinematics: Option<Option<KinematicsCalibration>>,
    /// `table_z_source`: `camera` (default) or `hover` (needs a passed `acceptance` and a tool
    /// with a known `edge_to_low_m`).
    pub table_z_source: String,
    /// `tool`: None = absent (a stored tool stays while its x fingerprint matches), Some(None) =
    /// null (removes it), Some(Some(t)) = store `t` (its `x_fingerprint` must match the new calibration).
    pub tool: Option<Option<ToolInput>>,
    /// `acceptance`: stored when given (its `fingerprint` must match the new calibration); an
    /// apply without it drops the stored one.
    pub acceptance: Option<AcceptanceInput>,
}

/// Apply field `tool` (armd adds `edge_to_low_m` from the profile's `calibration_tool`).
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ToolInput {
    pub tool_id: String,
    #[serde(default)]
    pub tool_config_digest: Option<String>,
    pub tip_cam_m: [f64; 3],
    #[serde(default)]
    pub gripper_opening: Option<f64>,
    /// `x_fingerprint` of the candidate the tip was measured with (from a dry run of this apply).
    pub x_fingerprint: String,
    #[serde(default)]
    pub measured_at: Option<String>,
    #[serde(default)]
    pub fit: Option<Value>,
}

/// Apply field `acceptance`.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AcceptanceInput {
    /// `fingerprint` of the candidate the hovers ran with (from a dry run of this apply).
    pub fingerprint: String,
    pub gate_version: u32,
    pub passed: bool,
    #[serde(default)]
    pub measured_at: Option<String>,
    pub board: Value,
    pub points: Vec<Value>,
    pub summary: Value,
    #[serde(default)]
    pub job_id: Option<String>,
}

pub fn parse_apply(body: &Value) -> Result<ApplyRequest, CalibError> {
    let o = body.as_object().ok_or_else(|| CalibError::invalid("Command body must be an object"))?;
    for k in o.keys() {
        if !["intrinsics", "camera_mount", "table_z_m", "rms", "solved_at", "dry_run", "kinematics", "table_z_source", "tool", "acceptance"].contains(&k.as_str()) {
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
    let kinematics = match o.get("kinematics") {
        None => None,
        Some(Value::Null) => Some(None),
        Some(v) => {
            let k: KinematicsCalibration = serde_json::from_value(v.clone())
                .map_err(|e| CalibError::invalid(format!("kinematics must be {{joints: [6 x {{offset_rad, scale, tilt_rad, origin_m}}], fit?}}: {e}")))?;
            k.validate().map_err(CalibError::invalid)?;
            Some(Some(k))
        }
    };
    let table_z_source = match o.get("table_z_source") {
        None | Some(Value::Null) => "camera".to_string(),
        Some(Value::String(s)) if s == "camera" || s == "hover" => s.clone(),
        Some(_) => return Err(CalibError::invalid("table_z_source must be \"camera\" or \"hover\"")),
    };
    let tool = match o.get("tool") {
        None => None,
        Some(Value::Null) => Some(None),
        Some(v) => Some(Some(serde_json::from_value::<ToolInput>(v.clone())
            .map_err(|e| CalibError::invalid(format!("tool must be {{tool_id, tool_config_digest?, tip_cam_m, x_fingerprint, gripper_opening?, measured_at?, fit?}}: {e}")))?)),
    };
    let acceptance = match o.get("acceptance") {
        None | Some(Value::Null) => None,
        Some(v) => Some(serde_json::from_value::<AcceptanceInput>(v.clone())
            .map_err(|e| CalibError::invalid(format!("acceptance must be {{fingerprint, gate_version, passed, board, points, summary, measured_at?, job_id?}}: {e}")))?),
    };
    if acceptance.as_ref().is_some_and(|a| a.job_id.as_ref().is_some_and(|j| j.len() > 64)) { return Err(CalibError::invalid("acceptance.job_id: at most 64 bytes")); }
    Ok(ApplyRequest {
        table_z_source,
        tool,
        acceptance,
        intrinsics,
        offset_m: [off[0] as f32, off[1] as f32, off[2] as f32],
        q_end_cam,
        table_z_m,
        rms: o.get("rms").filter(|v| !v.is_null()).cloned(),
        solved_at,
        dry_run,
        kinematics,
    })
}

// ---- derived values ----

/// X3: the grip pixel (normalised) of a camera model: the projection of the point under the
/// TCP on the grasp plane (`table_z_m + grasp_height_m`) seen from the look-down approach pose
/// (tool axis end_link x vertical, TCP at `table_z_m + approach_height_m`). Independent of the
/// yaw about the vertical. None when the point is behind the camera.
pub fn grip_pixel_norm(cam: &CamModel, setup: &GraspSetup) -> Option<[f64; 2]> {
    grip_pixel_norm_for_tcp(cam, setup, &[0.0; 3])
}

/// [`grip_pixel_norm`] accounting for the active tool's `tcp_offset` (in `end_link` frame).
pub fn grip_pixel_norm_for_tcp(
    cam: &CamModel,
    setup: &GraspSetup,
    tcp_offset: &[f64; 3],
) -> Option<[f64; 2]> {
    let tcp_pose = crate::kinematics::Pose::new(
        crate::grasp::look_down(&crate::kinematics::IDENTITY),
        [0.0, 0.0, setup.table_z_m + setup.approach_height_m],
    );
    let end_pose = crate::motion::tcp_to_end(&tcp_pose, tcp_offset);
    let px = cam.project(&end_pose, &[0.0, 0.0, setup.table_z_m + setup.grasp_height_m])?;
    let n = cam.to_norm(px);
    Some([(n[0] * 1e4).round() / 1e4, (n[1] * 1e4).round() / 1e4])
}

/// Sync camera-derived fields of `GraspSetup` from camera intrinsics, mount, and optional `table_z_m`,
/// recomputing `grip_pixel_norm` via [`grip_pixel_norm_for_tcp`].
fn refresh_grasp_setup_camera_fields(
    setup: &mut GraspSetup,
    intrinsics: Option<&Value>,
    mount: &CameraMount,
    table_z_m: Option<f64>,
    arm_model: &crate::chain::ArmModel,
    tcp_offset: &[f64; 3],
) {
    if let Some(intr) = intrinsics {
        if let Some(fx) = intr.get("fx").and_then(Value::as_f64).filter(|v| v.is_finite()) {
            setup.fx_px = fx;
        }
        if let Some(fy) = intr.get("fy").and_then(Value::as_f64).filter(|v| v.is_finite()) {
            setup.fy_px = fy;
        }
        if let Some(cx) = intr.get("cx").and_then(Value::as_f64).filter(|v| v.is_finite()) {
            setup.cx_px = Some(cx);
        }
        if let Some(cy) = intr.get("cy").and_then(Value::as_f64).filter(|v| v.is_finite()) {
            setup.cy_px = Some(cy);
        }
        setup.image_size_px = IMAGE_SIZE_PX;
    }
    setup.camera_tilt_deg = f64::from(mount.tilt_deg);
    if let Some(z) = table_z_m.filter(|z| z.is_finite()) {
        setup.table_z_m = z;
    }
    let cam = CamModel::assumed(arm_model, setup, mount);
    if let Some(p) = grip_pixel_norm_for_tcp(&cam, setup, tcp_offset)
        .filter(|p| p.iter().all(|v| (0.0..=1.0).contains(v)))
    {
        setup.grip_pixel_norm = p;
    }
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
    #[serde(default, skip_serializing_if = "Option::is_none")]
    tool_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    tool_config_digest: Option<String>,
    /// Apply: the grip pixel of the new camera model ([`grip_pixel_norm`]), also when it is
    /// outside the image (then `grip_pixel_norm` is left as it was).
    #[serde(skip)]
    grip_pixel_model: Option<[f64; 2]>,
}

fn armd_state(inner: &SupervisorInner) -> ArmdState {
    ArmdState {
        camera_mount: inner.config.camera_mount.clone(),
        calibration: inner.calibration.clone(),
        grasp_setup: inner.config.grasp.clone(),
        tool_id: Some(inner.config.active_tool.id.clone()),
        tool_config_digest: Some(inner.config.active_tool.config_digest.clone()),
        grip_pixel_model: None,
    }
}

pub(crate) fn mount_summary(m: &CameraMount) -> Value {
    json!({"flip": m.flip, "rot_deg": m.rot_deg, "tilt_deg": m.tilt_deg, "offset_m": m.offset_m, "q_end_cam": m.q_end_cam})
}

/// The state an apply produces, computed under the lock.
fn after_state(inner: &SupervisorInner, req: &ApplyRequest) -> Result<ArmdState, CalibError> {
    let Some(base) = inner.config.grasp.clone() else {
        return Err(CalibError::new(400, "UNSUPPORTED", "This profile has no grasp setup"));
    };
    let kinematics = match &req.kinematics {
        None => inner.calibration.as_ref().and_then(|c| c.kinematics.clone()),
        Some(k) => k.clone(),
    };
    if let Some(k) = &kinematics {
        inner.config.arm_model.with_correction(Some(&k.joints)).map_err(|e| CalibError::new(400, "UNSUPPORTED", e))?;
    }
    let mount = calibrated_mount(inner, req.offset_m, req.q_end_cam);
    let g = |k: &str| req.intrinsics[k].as_f64().unwrap_or(f64::NAN);
    let mut setup = GraspSetup {
        fx_px: g("fx"), fy_px: g("fy"), cx_px: Some(g("cx")), cy_px: Some(g("cy")), image_size_px: IMAGE_SIZE_PX,
        table_z_m: req.table_z_m, camera_tilt_deg: f64::from(mount.tilt_deg), ..base
    };
    let cam = CamModel::assumed(&inner.config.arm_model, &setup, &mount);
    // outside the image (a mount tilted toward the jaws, e.g. v8 at approach height): the grip
    // pixel cannot be stored; camera-down acquisition does not use it, so it stays as it was
    let grip_pixel_model = grip_pixel_norm_for_tcp(&cam, &setup, &inner.config.tcp_offset);
    if let Some(p) = grip_pixel_model.filter(|p| p.iter().all(|v| (0.0..=1.0).contains(v))) { setup.grip_pixel_norm = p; }
    setup.validate().map_err(|e| CalibError::invalid(format!("grasp setup after the calibration: {e}")))?;
    let now = || chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Secs, true);
    let intrinsics = Some(Value::Object(req.intrinsics.clone()));
    let x_fp = x_fingerprint(intrinsics.as_ref(), &mount);
    let tool = match &req.tool {
        // a stored fingertip stays only while it is bound to the same camera calibration and active tool
        None => inner
            .calibration
            .as_ref()
            .and_then(|c| c.tool.clone())
            .filter(|t| {
                t.x_fingerprint == x_fp
                    && inner
                        .config
                        .active_tool
                        .matches_tool_calib(&t.tool_id, t.tool_config_digest.as_deref())
            }),
        Some(None) => None,
        Some(Some(t)) => {
            if t.x_fingerprint != x_fp {
                return Err(CalibError::new(409, "TOOL_STALE", "tool.x_fingerprint does not match this calibration (intrinsics or camera mount changed since the fingertip was measured); measure the fingertip again"));
            }
            if !inner.config.active_tool.matches_tool_calib(&t.tool_id, t.tool_config_digest.as_deref()) {
                return Err(CalibError::new(
                    409,
                    "TOOL_STALE",
                    format!(
                        "tool ({:?} / {:?}) does not match active tool '{}' ({}); measure the fingertip again",
                        t.tool_id, t.tool_config_digest, inner.config.active_tool.id, inner.config.active_tool.config_digest
                    ),
                ));
            }
            let spec = inner.config.calibration_tool.as_ref().filter(|s| s.tool_id == t.tool_id);
            let tool = ToolCalibration {
                tool_id: t.tool_id.clone(),
                tip_cam_m: t.tip_cam_m,
                edge_to_low_m: spec.and_then(|s| s.edge_to_low_m),
                gripper_opening: t.gripper_opening,
                measured_at: t.measured_at.clone().filter(|m| !m.is_empty()).unwrap_or_else(now),
                x_fingerprint: t.x_fingerprint.clone(),
                fit: t.fit.clone(),
                tool_config_digest: Some(inner.config.active_tool.config_digest.clone()),
            };
            validate_tool(&tool).map_err(CalibError::invalid)?;
            Some(tool)
        }
    };
    let mut file = CalibrationFile {
        camera_mount: mount.clone(),
        intrinsics,
        table_z_m: Some(req.table_z_m),
        solved_at: Some(req.solved_at.clone().unwrap_or_else(now)),
        rms: req.rms.clone(),
        kinematics,
        stale: None,
        joint_zero: inner.calibration.as_ref().and_then(|c| c.joint_zero.clone()),
        table_z_source: Some(req.table_z_source.clone()),
        tool,
        acceptance: None,
        mit_qualification: inner.calibration.as_ref().and_then(|c| c.mit_qualification.clone()),
    };
    let (_, fp) = file_fingerprints(
        Some(&file),
        &mount,
        &inner.joint_zero_offsets,
        Some(&inner.config.active_tool.config_digest),
    );
    if let Some(a) = &req.acceptance {
        if a.fingerprint != fp {
            return Err(CalibError::new(409, "ACCEPTANCE_STALE", "acceptance.fingerprint does not match this calibration (camera, kinematics, joint zero or tool changed since the hovers); run the acceptance again"));
        }
        let rec = AcceptanceRecord {
            fingerprint: a.fingerprint.clone(),
            gate_version: a.gate_version,
            passed: a.passed,
            measured_at: a.measured_at.clone().filter(|m| !m.is_empty()).unwrap_or_else(now),
            board: a.board.clone(),
            points: a.points.clone(),
            summary: a.summary.clone(),
            job_id: a.job_id.clone(),
            invalidated: None,
            tool_id: Some(inner.config.active_tool.id.clone()),
            tool_config_digest: Some(inner.config.active_tool.config_digest.clone()),
        };
        validate_acceptance(&rec).map_err(CalibError::invalid)?;
        file.acceptance = Some(rec);
    }
    if req.table_z_source == "hover" {
        let ok = file.acceptance.as_ref().is_some_and(|a| a.passed) && file.tool.as_ref().is_some_and(|t| t.edge_to_low_m.is_some());
        if !ok {
            return Err(CalibError::new(400, "TABLE_Z_UNVERIFIED", "table_z_source hover needs a passed acceptance and a fingertip of a known tool (profile calibration_tool with edge_to_low_m)"));
        }
        let summary_z = file.acceptance.as_ref().and_then(|a| a.summary.get("table_z_m")).and_then(Value::as_f64);
        if summary_z.is_none_or(|z| (z - req.table_z_m).abs() > 1e-6) {
            return Err(CalibError::invalid("table_z_source hover: table_z_m must equal acceptance.summary.table_z_m"));
        }
    }
    Ok(ArmdState {
        camera_mount: mount,
        calibration: Some(file),
        grasp_setup: Some(setup),
        tool_id: Some(inner.config.active_tool.id.clone()),
        tool_config_digest: Some(inner.config.active_tool.config_digest.clone()),
        grip_pixel_model,
    })
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

pub(crate) async fn blocking<T: Send + 'static>(f: impl FnOnce() -> std::io::Result<T> + Send + 'static) -> Result<T, String> {
    match tokio::task::spawn_blocking(f).await {
        Ok(r) => r.map_err(|e| e.to_string()),
        Err(e) => Err(format!("task failed: {e}")),
    }
}

/// Write (or with None remove) a file durably.
pub(crate) async fn put_file(path: PathBuf, text: Option<String>) -> Result<(), String> {
    blocking(move || match text {
        Some(t) => crate::persist::write_atomic(&path, &t),
        None => match std::fs::remove_file(&path) {
            Err(e) if e.kind() != std::io::ErrorKind::NotFound => Err(e),
            _ => Ok(()),
        },
    }).await
}

pub(crate) fn read_if_file(path: &Path) -> std::io::Result<Option<String>> {
    if !path.is_file() { return Ok(None); }
    std::fs::read_to_string(path).map(Some)
}

fn read_backup_file(path: &Path) -> std::io::Result<Option<String>> {
    match std::fs::read_to_string(path) {
        Ok(s) => Ok(Some(s)),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(e) => Err(e),
    }
}

/// Backup directory name: UTC time, unique within `root`.
pub(crate) fn backup_name(root: &Path) -> String {
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
        "camera_mount": state.camera_mount, "calibration": state.calibration, "grasp_setup": state.grasp_setup,
        "tool_id": state.tool_id, "tool_config_digest": state.tool_config_digest});
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

/// A change of the kinematic correction needs torque off and no action running (the arm model
/// of a running action or of a held pose would change under it).
pub fn kinematics_armed(inner: &SupervisorInner) -> Result<(), CalibError> {
    let running = inner.last_action.as_ref().is_some_and(|a| a.is_running()) || inner.macro_run.is_some();
    if inner.is_torqued || running {
        let why = if running { "an action is running" } else { "torque is on" };
        return Err(CalibError::new(409, "KINEMATICS_ARMED",
            format!("the kinematic correction can change only with torque off and no action running ({why}); park or disable torque first")));
    }
    Ok(())
}

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
        if let Some(sup) = self.sup.take() { clear_busy(&mut *sup.inner().write().await); }
    }
}

impl Drop for BusyGuard {
    fn drop(&mut self) {
        let Some(sup) = self.sup.take() else { return; };
        if let Ok(mut inner) = sup.inner().try_write() { clear_busy(&mut inner); return; }
        // the lock is held (for example by the code that panicked): clear it once it is free
        match tokio::runtime::Handle::try_current() {
            Ok(h) => { h.spawn(async move { clear_busy(&mut *sup.inner().write().await); }); }
            Err(_) => clear_busy(&mut sup.inner().blocking_write()),
        }
        tracing::warn!("calibration transaction ended without release (panic or early return); busy flag cleared");
    }
}

fn clear_busy(inner: &mut SupervisorInner) {
    inner.calibration_busy = false;
    inner.motor_zero_busy = false;
}

/// After a zero (motor or software) of `joints` (1-based) whose readings moved by `changes`
/// (rad, same order): remove the kinematic correction (live, and from `calibration.json`), mark
/// the camera calibration stale, write `joint_zero` (`new_zero`: None = unchanged, Some(None) =
/// removed), and, for joints that moved by more than [`crate::joint_zero::STALE_THRESHOLD_RAD`],
/// mark the saved poses stale and, for the gripper joint, the gripper calibration (nothing is
/// deleted). The caller holds `calibration_busy` and torque is off. Returns `{kinematics_removed,
/// camera_stale, poses_stale, gripper_calibration_stale, removed_kinematics, at, error?}`; a
/// failed file write leaves the live state changed and reports `error`.
pub async fn invalidate_after_zero(sup: &ArmSupervisor, deps: &CalibrationDeps, joints: &[usize], changes: &[f32],
    method: &str, new_zero: Option<Option<crate::joint_zero::JointZero>>) -> Value {
    let at = chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Secs, true);
    let moved: Vec<usize> = joints.iter().zip(changes).filter(|(_, d)| f64::from(d.abs()) > crate::joint_zero::STALE_THRESHOLD_RAD)
        .map(|(j, _)| *j).collect();
    let (file, removed, had_live, camera_stale, file_changes) = {
        let inner = sup.inner().read().await;
        let mut file = inner.calibration.clone();
        let removed = file.as_mut().and_then(|c| c.kinematics.take());
        let camera_stale = file.as_ref().is_some_and(CalibrationFile::has_camera);
        if let Some(c) = file.as_mut().filter(|c| c.has_camera()) {
            c.stale = Some(CalibrationStale { reason: format!("{method} zero of joints {joints:?}; the camera calibration was measured with the old zero"), at: at.clone() });
            // the hovers (and a hover table height) were measured with the old zero
            if let Some(a) = c.acceptance.as_mut() {
                a.invalidated = Some(CalibrationStale { reason: format!("{method} zero of joints {joints:?}; the acceptance was measured with the old zero"), at: at.clone() });
            }
        }
        let mut file_changes = removed.is_some() || camera_stale;
        // an MIT qualification of a zeroed joint was measured with the old zero
        if let Some(c) = file.as_mut() {
            if crate::mit::invalidate_record(c, joints, &format!("{method} zero of joints {joints:?}; the MIT qualification was measured with the old zero"), &at) {
                file_changes = true;
            }
        }
        if let Some(z) = new_zero {
            if file.as_ref().and_then(|c| c.joint_zero.as_ref()) != z.as_ref() { file_changes = true; }
            match file.as_mut() {
                Some(c) => c.joint_zero = z,
                None if z.is_some() => file = Some(CalibrationFile { joint_zero: z, ..Default::default() }),
                None => {}
            }
        }
        // nothing left in the file (no camera, no joint zero): remove it
        if file.as_ref().is_some_and(|c| !c.has_camera() && c.joint_zero.is_none() && c.kinematics.is_none() && c.mit_qualification.is_none()) { file = None; }
        (file, removed, inner.config.arm_model.correction().is_some(), camera_stale, file_changes)
    };
    let mut error = None;
    if file_changes {
        if let Err(e) = put_file(deps.path.clone(), file.as_ref().map(|c| serde_json::to_string_pretty(c).unwrap_or_default())).await {
            error = Some(format!("calibration file {}: {e}", deps.path.display()));
        }
    }
    let (poses_write, gripper_write, poses_stale, gripper_stale) = {
        let mut inner = sup.inner().write().await;
        if let Err(e) = inner.set_kinematics_correction(None) { error.get_or_insert(format!("kinematic correction not removed: {e}")); }
        if file_changes { inner.set_calibration(file); }
        let reason = format!("{method} zero of joints {moved:?}; taught with the old zero");
        let mut poses_stale = Vec::new();
        if !moved.is_empty() {
            for p in inner.poses.list.poses.iter_mut() {
                p.stale = Some(CalibrationStale { reason: reason.clone(), at: at.clone() });
                poses_stale.push(p.name.clone());
            }
        }
        if !poses_stale.is_empty() { inner.poses.revision += 1; }
        let poses_write = (!poses_stale.is_empty()).then(|| inner.poses_write()).flatten();
        let grip = inner.config.gripper_index().map(|g| g + 1);
        let has_grip_calib = inner.gripper_calib_offset.is_some() || inner.gripper_calib_jaw_opening.is_some();
        let gripper_stale = grip.is_some_and(|g| moved.contains(&g)) && has_grip_calib;
        // the free-motion baseline is indexed by jaw position (crate::gripper_baseline)
        if grip.is_some_and(|g| moved.contains(&g)) && inner.gripper_baseline.is_some() { inner.reset_gripper_baseline(); }
        let gripper_write = gripper_stale.then(|| {
            inner.gripper_calib_stale = Some(CalibrationStale { reason: reason.clone(), at: at.clone() });
            inner.gripper_calib_revision += 1;
            let f = crate::gripper_calib::GripperCalibFile {
                offset_rad: inner.gripper_calib_offset,
                jaw_opening: inner.gripper_calib_jaw_opening.clone(),
                ts: Some(std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs())),
                stale: inner.gripper_calib_stale.clone(),
                tool_id: Some(inner.config.active_tool.id.clone()),
                tool_config_digest: Some(inner.config.active_tool.config_digest.clone()),
            };
            let path = inner.gripper_calib_path.clone().unwrap_or_else(crate::gripper_calib::path_from_env);
            crate::persist::PendingWrite::new(path, serde_json::to_string_pretty(&f).unwrap_or_default())
        });
        (poses_write, gripper_write, poses_stale, gripper_stale)
    };
    for (what, w) in [("poses file", poses_write), ("gripper calibration file", gripper_write)] {
        if let Some(w) = w {
            if let Err(e) = w.run().await { error.get_or_insert(format!("{what}: {e}")); }
        }
    }
    if let Some(e) = &error { tracing::error!("zero invalidation: {e}"); }
    tracing::warn!("{method} zero of joints {joints:?}: kinematic correction {}, camera calibration {}, {} poses and gripper calibration {} marked stale",
        if removed.is_some() || had_live { "removed" } else { "none" }, if camera_stale { "marked stale" } else { "none" },
        poses_stale.len(), if gripper_stale { "" } else { "not" });
    json!({"kinematics_removed": removed.is_some() || had_live, "camera_stale": camera_stale, "poses_stale": poses_stale,
        "gripper_calibration_stale": gripper_stale, "removed_kinematics": removed, "at": at, "error": error})
}

/// Everything marked stale by a zero: `[{item: camera | poses | gripper_calibration, reason, at, names?}]`.
pub fn stale_list(inner: &SupervisorInner) -> Vec<Value> {
    let mut out = Vec::new();
    if let Some(st) = inner.calibration.as_ref().filter(|c| c.has_camera()).and_then(|c| c.stale.as_ref()) {
        out.push(json!({"item": "camera", "reason": st.reason, "at": st.at}));
    }
    let stale: Vec<_> = inner.poses.list.poses.iter().filter(|p| p.stale.is_some()).collect();
    if let Some(first) = stale.first().and_then(|p| p.stale.as_ref()) {
        out.push(json!({"item": "poses", "reason": first.reason, "at": first.at, "names": stale.iter().map(|p| &p.name).collect::<Vec<_>>()}));
    }
    if let Some(st) = &inner.gripper_calib_stale {
        out.push(json!({"item": "gripper_calibration", "reason": st.reason, "at": st.at}));
    }
    out
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
    durability_warnings: Vec<String>,
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

    fn commit_live_calibration(
        &mut self,
        inner: &mut SupervisorInner,
        after: &ArmdState,
        kin: Option<&Correction>,
    ) -> Result<(), CalibError> {
        self.file_written = true;
        inner
            .set_kinematics_correction(kin)
            .map_err(|e| CalibError::new(400, "UNSUPPORTED", e).at("calibration_file"))?;
        inner.set_camera_mount(after.camera_mount.clone());
        inner.set_calibration(after.calibration.clone());
        self.live_written = true;
        Ok(())
    }

    /// calibration.json, then the live mount and calibration.
    /// Stages outside the supervisor lock, re-checks auth/lease/epoch + renames inside the lock,
    /// and syncs the parent directory outside the lock (recording `file_written = true` as soon
    /// as rename/remove succeeds so revert remains consistent even if dir sync fails).
    async fn write_calibration(&mut self, after: &ArmdState) -> Result<(), CalibError> {
        self.recheck("calibration_file").await?;
        let kin = correction_of(after.calibration.as_ref()).copied();
        let kin_changes = kin.as_ref() != correction_of(self.before.calibration.as_ref());
        if kin_changes { kinematics_armed(&*self.sup.inner().read().await).map_err(|e| e.at("calibration_file"))?; }
        let text = after.calibration.as_ref().map(|c| serde_json::to_string_pretty(c).unwrap_or_default());
        let dir_sync_opt = match text {
            Some(t) => {
                let path = self.deps.path.clone();
                let staged = tokio::task::spawn_blocking(move || crate::persist::StagedWrite::stage(path, t))
                    .await
                    .map_err(|e| CalibError::new(500, "PERSIST_FAILED", format!("calibration stage task: {e}")).at("calibration_file"))?
                    .map_err(|e| CalibError::new(500, "PERSIST_FAILED", format!("calibration file {}: {e}", self.deps.path.display())).at("calibration_file"))?;
                let mut inner = self.sup.inner().write().await;
                // checked again under the write lock: torque may have come on during the stage
                if let Err(e) = (self.check)(&mut inner).and_then(|()| if kin_changes { kinematics_armed(&inner) } else { Ok(()) }) {
                    staged.abort();
                    return Err(e.at("calibration_file"));
                }
                let dir_sync = staged.rename_only()
                    .map_err(|e| CalibError::new(500, "PERSIST_FAILED", format!("calibration file {}: {e}", self.deps.path.display())).at("calibration_file"))?;
                self.commit_live_calibration(&mut inner, after, kin.as_ref())?;
                Some(dir_sync)
            }
            None => {
                let mut inner = self.sup.inner().write().await;
                (self.check)(&mut inner).map_err(|e| e.at("calibration_file"))?;
                if kin_changes { kinematics_armed(&inner).map_err(|e| e.at("calibration_file"))?; }
                let mut removed = false;
                match std::fs::remove_file(&self.deps.path) {
                    Ok(()) => { removed = true; }
                    Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
                    Err(e) => {
                        return Err(CalibError::new(500, "PERSIST_FAILED", format!("calibration file {}: {e}", self.deps.path.display())).at("calibration_file"));
                    }
                }
                self.commit_live_calibration(&mut inner, after, kin.as_ref())?;
                removed.then(|| crate::persist::PendingDirSync::for_path(&self.deps.path))
            }
        };
        if let Some(dir_sync) = dir_sync_opt {
            let outcome = tokio::task::spawn_blocking(move || dir_sync.sync_parent_dir())
                .await
                .unwrap_or_else(|e| crate::persist::PublishOutcome::PublishedDurabilityUnconfirmed(e.to_string()));
            if let crate::persist::PublishOutcome::PublishedDurabilityUnconfirmed(w) = outcome {
                self.durability_warnings.push(format!("calibration_file: {w}"));
            }
        }
        Ok(())
    }

    fn precheck_grasp_commit(&self, inner: &mut SupervisorInner, base_rev: u64) -> Result<(), CalibError> {
        (self.check)(inner).map_err(|e| e.at("grasp_setup"))?;
        if inner.grasp_setup_revision != base_rev {
            let mut err = CalibError::new(
                409,
                "POLICY_REVISION_MISMATCH",
                format!("Policy revision mismatch: expected {base_rev}, current {}", inner.grasp_setup_revision),
            ).at("grasp_setup");
            err.extra.insert("current_revision".into(), json!(inner.grasp_setup_revision));
            return Err(err);
        }
        Ok(())
    }

    /// The grasp setup, persisted via two-phase atomic write (`StagedWrite`) and committed with
    /// monotonic `grasp_setup_revision` increment. Parent-dir fsync runs outside supervisor lock.
    async fn write_grasp(&mut self, after: &ArmdState) -> Result<(), CalibError> {
        self.recheck("grasp_setup").await?;
        let Some(setup) = after.grasp_setup.clone() else { return Ok(()); };
        let cw_lock = self.sup.config_write_lock();
        let _cw = cw_lock.lock().await;
        let (setup, base_rev, active_tool) = {
            let inner = self.sup.inner().read().await;
            let mut setup = setup;
            setup.camera_tilt_deg = f64::from(inner.config.camera_mount.tilt_deg);
            (setup, inner.grasp_setup_revision, inner.config.active_tool.clone())
        };
        if let Some(path) = self.deps.setup_path.clone() {
            let text = crate::grasp::serialize_setup_for_tool(&setup, &active_tool);
            let staged = tokio::task::spawn_blocking(move || crate::persist::StagedWrite::stage(path, text))
                .await
                .map_err(|e| CalibError::new(500, "PERSIST_FAILED", format!("grasp setup stage task: {e}")).at("grasp_setup"))?
                .map_err(|e| CalibError::new(500, "PERSIST_FAILED", format!("grasp setup file: {e}")).at("grasp_setup"))?;
            let dir_sync = {
                let mut inner = self.sup.inner().write().await;
                if let Err(err) = self.precheck_grasp_commit(&mut inner, base_rev) {
                    staged.abort();
                    return Err(err);
                }
                let dir_sync = staged.rename_only()
                    .map_err(|e| CalibError::new(500, "PERSIST_FAILED", format!("grasp setup file: {e}")).at("grasp_setup"))?;
                inner.set_grasp_setup(setup);
                inner.grasp_setup_revision += 1;
                dir_sync
            };
            let outcome = tokio::task::spawn_blocking(move || dir_sync.sync_parent_dir())
                .await
                .unwrap_or_else(|e| crate::persist::PublishOutcome::PublishedDurabilityUnconfirmed(e.to_string()));
            if let crate::persist::PublishOutcome::PublishedDurabilityUnconfirmed(w) = outcome {
                self.durability_warnings.push(format!("grasp_setup: {w}"));
            }
        } else {
            let mut inner = self.sup.inner().write().await;
            self.precheck_grasp_commit(&mut inner, base_rev)?;
            inner.set_grasp_setup(setup);
            inner.grasp_setup_revision += 1;
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
            inner.set_calibration(self.before.calibration.clone());
            if let Err(e) = inner.set_kinematics_correction(correction_of(self.before.calibration.as_ref())) {
                tracing::warn!("revert: kinematic correction not put back: {e}");
            }
            state.insert("armd_live".into(), json!("reverted"));
        }
        if self.file_written {
            let backup_res = match self.backup.as_ref() {
                Some(b) => read_backup_file(&b.join("calibration.json"))
                    .map_err(|e| format!("read backup calibration.json: {e}")),
                None => Err("no backup directory recorded".into()),
            };
            match backup_res {
                Ok(backup_text) => match put_file(self.deps.path.clone(), backup_text).await {
                    Ok(()) => { state.insert("calibration_file".into(), json!("reverted")); }
                    Err(e) => { ok = false; state.insert("calibration_file".into(), json!(format!("revert failed: {e}"))); }
                },
                Err(e) => {
                    ok = false;
                    state.insert("calibration_file".into(), json!(format!("revert failed: {e}")));
                }
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
        let mut inner = self.sup.inner().write().await;
        inner.calibration_load_error = None;
        let mut out = json!({"backup": backup.display().to_string(), "vision_intrinsics": vision,
            "camera_mount": mount_summary(&inner.config.camera_mount), "calibration": inner.calibration,
            "kinematics_correction": inner.config.arm_model.correction(),
            "grasp": inner.grasp_setup().map(|s| grasp_fields(&s)), "grasp_revision": inner.grasp_setup_revision,
            "durability_confirmed": self.durability_warnings.is_empty()});
        if !self.durability_warnings.is_empty() {
            out["durability_warning"] = json!(self.durability_warnings.join("; "));
        }
        Ok(out)
    }
}

fn diff_view(vision: &Value, state: &ArmdState, inner: &SupervisorInner) -> Value {
    let c = state.calibration.as_ref();
    let (x_fp, fp) = file_fingerprints(
        c,
        &state.camera_mount,
        &inner.joint_zero_offsets,
        Some(&inner.config.active_tool.config_digest),
    );
    let legacy_fp = legacy_file_fingerprint(c, &state.camera_mount, &inner.joint_zero_offsets);
    let mut v = json!({"vision_intrinsics": vision, "camera_mount": mount_summary(&state.camera_mount),
        "kinematics": c.and_then(|c| c.kinematics.as_ref()),
        "grasp": state.grasp_setup.as_ref().map(grasp_fields),
        "x_fingerprint": x_fp, "fingerprint": fp,
        "table_z_m": c.and_then(|c| c.table_z_m), "table_z_source": c.and_then(|c| c.table_z_source.as_deref()),
        "tool": c.and_then(|c| c.tool.as_ref()), "acceptance": c.and_then(|c| c.acceptance.as_ref()).map(|a| json!({"passed": a.passed, "fingerprint": a.fingerprint, "job_id": a.job_id})),
        "floor": inner.profile_floor_z_m.map(|p| floor_view_for_with_legacy(p, c, &fp, Some(&legacy_fp), &inner.config.active_tool))});
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
        let inner = sup.inner().read().await;
        return Ok(json!({"dry_run": true, "before": diff_view(&vision_before, &before, &inner), "after": diff_view(&Value::Object(va), &after, &inner)}));
    }
    let mut txn = Txn { sup, deps, check, before, vision_before, vision_written: None, vision_unknown: false, file_written: false, live_written: false, backup: None, durability_warnings: Vec::new() };
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
    let mut target: ArmdState = serde_json::from_value(manifest.clone()).map_err(|e| CalibError::invalid(format!("backup {name}: manifest.json: {e}")))?;
    let vision_values = validate_intrinsics(&vision_target).map_err(|e| CalibError::invalid(format!("backup {name}: vision_intrinsics.json: {e}")))?;
    if let Some(c) = &target.calibration { c.validate().map_err(CalibError::invalid)?; }
    if let Some(g) = &target.grasp_setup { g.validate().map_err(|e| CalibError::invalid(format!("backup {name}: grasp setup: {e}")))?; }
    let (before, active_tool, arm_model, tcp_offset) = {
        let inner = sup.inner().read().await;
        if let Some(k) = target.calibration.as_ref().and_then(|c| c.kinematics.as_ref()) {
            inner.config.arm_model.with_correction(Some(&k.joints)).map_err(|e| CalibError::new(400, "UNSUPPORTED", format!("backup {name}: {e}")))?;
        }
        if correction_of(target.calibration.as_ref()) != correction_of(inner.calibration.as_ref()) { kinematics_armed(&inner)?; }
        (
            armd_state(&inner),
            inner.config.active_tool.clone(),
            inner.config.arm_model.clone(),
            inner.config.tcp_offset,
        )
    };
    let (backup_tool_id, backup_tool_digest) = if target.tool_id.is_some() || target.tool_config_digest.is_some() {
        (target.tool_id.clone(), target.tool_config_digest.clone())
    } else if let Some(gs_val) = std::fs::read_to_string(dir.join("grasp_setup.json"))
        .ok()
        .and_then(|t| serde_json::from_str::<Value>(&t).ok())
    {
        (
            gs_val.get("tool_id").and_then(Value::as_str).map(str::to_string),
            gs_val.get("tool_config_digest").and_then(Value::as_str).map(str::to_string),
        )
    } else {
        (None, None)
    };
    // Tool-dependent fingertip / hover acceptance records from another tool do not carry over across tool swaps.
    if let Some(c) = target.calibration.as_mut() {
        strip_mismatched_tool_records(c, &active_tool);
    }
    let tool_matches = active_tool.matches_record(backup_tool_id.as_deref(), backup_tool_digest.as_deref());
    if !tool_matches {
        if target.grasp_setup.is_some() {
            tracing::warn!(
                "Calibration restore {name}: stripping tool-scoped grasp_setup for {:?}/{:?} (active tool is '{}' / '{}')",
                backup_tool_id, backup_tool_digest, active_tool.id, active_tool.config_digest
            );
            target.grasp_setup = None;
        }
        if let Some(mut live_b) = before.grasp_setup.clone() {
            let intrinsics_val = Value::Object(vision_values.clone());
            refresh_grasp_setup_camera_fields(
                &mut live_b,
                Some(&intrinsics_val),
                &target.camera_mount,
                target.calibration.as_ref().and_then(|c| c.table_z_m),
                &arm_model,
                &tcp_offset,
            );
            live_b
                .validate()
                .map_err(|e| CalibError::invalid(format!("grasp setup after cross-tool restore: {e}")))?;
            target.grasp_setup = Some(live_b);
            target.tool_id = Some(active_tool.id.clone());
            target.tool_config_digest = Some(active_tool.config_digest.clone());
        }
    } else if let Some(g) = &target.grasp_setup {
        // a missing or narrower jaw curve does not block the restore: the width is unverified or
        // clamped to the curve (`grasp_width_check`)
        crate::grasp::log_grasp_width_check(&*sup.inner().read().await, g.max_grasp_width_m);
    }
    // the joint zero is not part of a camera restore: the live one stays (it has its own restore,
    // `/api/v1/calibration/joint_zero/restore`)
    let live_zero = before.calibration.as_ref().and_then(|c| c.joint_zero.clone());
    // nor is the MIT qualification: the live record stays
    let live_mit = before.calibration.as_ref().and_then(|c| c.mit_qualification.clone());
    match target.calibration.as_mut() {
        Some(c) => { c.joint_zero = live_zero; c.mit_qualification = live_mit; }
        None if live_zero.is_some() || live_mit.is_some() =>
            target.calibration = Some(CalibrationFile { joint_zero: live_zero, mit_qualification: live_mit, ..Default::default() }),
        None => {}
    }
    let vision_before = vision_get(deps).await.map_err(|e| e.at("vision"))?;
    let mut txn = Txn { sup, deps, check, before, vision_before, vision_written: None, vision_unknown: false, file_written: false, live_written: false, backup: None, durability_warnings: Vec::new() };
    let mut out = txn.run("restore", &vision_values, &target).await?;
    out["restored"] = json!(name);
    tracing::info!("Camera calibration restored from backup {name}");
    Ok(out)
}

fn floor_json(profile: f32, (eff, src, z): (f32, &'static str, Option<f64>)) -> Value {
    json!({"profile_floor_z_m": profile, "effective_floor_z_m": eff, "source": src, "table_z_m": z, "below_table_m": FLOOR_BELOW_TABLE_M})
}

/// `{profile_floor_z_m, effective_floor_z_m, source, table_z_m}` of the floor rule.
pub fn floor_view(profile: f32, c: Option<&CalibrationFile>, fp: &str) -> Value {
    floor_json(profile, effective_floor(profile, c, fp))
}

pub fn floor_view_for(
    profile: f32,
    c: Option<&CalibrationFile>,
    fp: &str,
    active_tool: &crate::tool::ActiveToolInfo,
) -> Value {
    floor_view_for_with_legacy(profile, c, fp, None, active_tool)
}

pub fn floor_view_for_with_legacy(
    profile: f32,
    c: Option<&CalibrationFile>,
    fp: &str,
    legacy_fp: Option<&str>,
    active_tool: &crate::tool::ActiveToolInfo,
) -> Value {
    floor_json(profile, effective_floor_for_with_legacy(profile, c, fp, legacy_fp, active_tool))
}

/// `{x_fingerprint, fingerprint, tool_valid, acceptance_valid, tip_end_m?}` of the calibration in effect.
pub fn verification_view(inner: &SupervisorInner) -> Value {
    let (x_fp, fp) = live_fingerprints(inner);
    let legacy_fp = legacy_file_fingerprint(
        inner.calibration.as_ref(),
        &inner.config.camera_mount,
        &inner.joint_zero_offsets,
    );
    let c = inner.calibration.as_ref();
    let tool_ok = c.is_some_and(|c| tool_valid_for(c, &x_fp, &inner.config.active_tool));
    // tip_end = X · tip_cam with the X in effect
    let tip_end = c.and_then(|c| c.tool.as_ref()).filter(|_| tool_ok).and_then(|t| {
        let m = &inner.config.camera_mount;
        let (q, o) = (m.q_end_cam?, m.offset_m?);
        let r = crate::kinematics::quat_to_r(&q);
        let p = crate::kinematics::mat_vec(&r, &t.tip_cam_m);
        Some([p[0] + f64::from(o[0]), p[1] + f64::from(o[1]), p[2] + f64::from(o[2])])
    });
    json!({"x_fingerprint": x_fp, "fingerprint": fp, "tool_valid": tool_ok, "tip_end_m": tip_end,
        "acceptance_valid": c.is_some_and(|c| acceptance_valid_for_with_legacy(c, &fp, Some(&legacy_fp), &inner.config.active_tool)),
        "table_z_source": c.and_then(|c| c.table_z_source.as_deref()).unwrap_or(if c.is_some_and(CalibrationFile::has_camera) { "camera" } else { "none" })})
}

/// `GET /api/v1/calibration`.
pub async fn status(sup: &ArmSupervisor, deps: &CalibrationDeps) -> Value {
    let inner = sup.inner().read().await;
    let (_, fp) = live_fingerprints(&inner);
    let legacy_fp = legacy_file_fingerprint(
        inner.calibration.as_ref(),
        &inner.config.camera_mount,
        &inner.joint_zero_offsets,
    );
    json!({"calibrated": inner.camera_calibrated(), "busy": inner.calibration_busy, "load_error": inner.calibration_load_error,
        "path": deps.path.display().to_string(), "backup_dir": deps.backup_dir.display().to_string(),
        "calibration": inner.calibration, "camera_mount": mount_summary(&inner.config.camera_mount),
        "kinematics_correction": inner.config.arm_model.correction(),
        "stale": inner.calibration.as_ref().and_then(|c| c.stale.as_ref()),
        "stale_list": stale_list(&inner),
        "joint_zero": inner.calibration.as_ref().and_then(|c| c.joint_zero.as_ref()),
        "joint_zero_offsets": inner.joint_zero_offsets,
        "zero_reference": inner.config.zero_reference,
        "zero_convention": inner.config.zero_reference.as_ref().map(|_| &inner.zero_convention),
        "calibration_tools": inner.calibration_tools(),
        "calibration_tool": inner.config.calibration_tool,
        "active_tool": inner.config.active_tool,
        "verification": verification_view(&inner),
        "floor": inner.profile_floor_z_m.map(|p| floor_view_for_with_legacy(p, inner.calibration.as_ref(), &fp, Some(&legacy_fp), &inner.config.active_tool)),
        "grasp": inner.grasp_setup().map(|s| grasp_fields(&s)), "grasp_revision": inner.grasp_setup_revision,
        "backups": list_backups(deps)})
}

/// Before a zero of `joints` (1-based) is written: mark an MIT qualification record that holds
/// one of them invalidated and persist that first (atomic write), then publish it live. A failed
/// write refuses the zero (nothing changed), so a crash or write error never leaves a valid
/// record next to a changed physical zero. Nothing to do without such a record.
pub async fn invalidate_mit_before_zero(sup: &ArmSupervisor, deps: &CalibrationDeps, joints: &[usize], method: &str) -> Result<(), CalibError> {
    let at = chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Secs, true);
    let file = {
        let inner = sup.inner().read().await;
        let Some(mut c) = inner.calibration.clone() else { return Ok(()) };
        let reason = format!("{method} zero of joints {joints:?}; the MIT qualification was measured with the old zero");
        if !crate::mit::invalidate_record(&mut c, joints, &reason, &at) { return Ok(()); }
        c
    };
    let text = serde_json::to_string_pretty(&file).map_err(|e| CalibError::new(500, "INTERNAL", e.to_string()))?;
    put_file(deps.path.clone(), Some(text)).await.map_err(|e| CalibError::new(500, "PERSIST_FAILED",
        format!("the MIT qualification could not be invalidated in {} ({e}); the zero was refused, nothing was changed", deps.path.display())))?;
    sup.inner().write().await.set_calibration(Some(file));
    Ok(())
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

