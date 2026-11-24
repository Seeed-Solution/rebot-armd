//! Calibration hover job (T20 automatic acceptance): a bounded, device-side motion sequence that
//! brings the fingertip (tool down) above board corners and records the board markers the camera
//! sees there. armd owns the motion and the observation capture; the browser plans the targets
//! with the candidate calibration and evaluates the observations (src/api/calib_wizard.js).
//!
//! WS action `/api/v1/action/calib_hover` (lease, epoch) starts one run of a job:
//! `{candidate_fingerprint, points: [{id, approach_m, hover_m, rpy_rad?}], rpy_rad, approach_speed_mps?,
//! descent_speed_mps?, settle_s?, frames?, discard?}` or `{resume: <job id>}` (the points without
//! observations of a stored job). Per point: Cartesian move to `approach_m` (above the corner),
//! straight descent to `hover_m` at <= [`MAX_DESCENT_MPS`] ending with the profile's precise final
//! approach, settle >= [`MIN_SETTLE_S`], discard [`MIN_DISCARD`] distinct frames, then `frames` fresh
//! stationary ArUco triggers (frame ids strictly increasing, joints within [`STILL_RAD`] of the
//! settled position), back up to `approach_m`. A planning refusal before a point's motion (IK,
//! workspace) skips the point; a failure after a motion started, a stop, collision, fault, stale
//! telemetry (no fresh feedback within [`FEEDBACK_MAX_AGE`]), a foreign action, lease loss or an
//! epoch change end the run. Lease loss (dropped browser) leaves the arm to the lease watchdog,
//! which parks it; completed observations stay in the job record.
//!
//! `GET /api/v1/calibration/hover` lists the jobs, `GET /api/v1/calibration/hover/<id>` returns one
//! with the live arm state (torque, safety state, fault, watchdog park) so a reconnecting client
//! sees how the run ended. Jobs are kept in memory ([`KEEP_JOBS`]) and written to
//! `calibration-hover/<id>.json` next to `calibration.json`.
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

use axum::extract::Path as UrlPath;
use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

use crate::macro_action::{fail, FinalApproach, MacroCtx, Stop, POLL};
use crate::motion::{GoalSpec, Speed};
use crate::supervisor::{ArmError, ArmSupervisor, SupervisorInner};

pub const PATH: &str = "/api/v1/action/calib_hover";
pub const MAX_POINTS: usize = 8;
pub const KEEP_JOBS: usize = 4;
/// Descent above the paper (m/s).
pub const MAX_DESCENT_MPS: f64 = 0.015;
pub const DEFAULT_DESCENT_MPS: f64 = 0.01;
pub const MAX_APPROACH_MPS: f64 = 0.05;
pub const DEFAULT_APPROACH_MPS: f64 = 0.04;
/// Commissioning linear/angular caps matching WS Cartesian moves (`0.2 * MAX_*`).
pub const COMMISSIONING_MAX_LINEAR_MPS: f64 = 0.2 * crate::motion::MAX_LINEAR_MPS;
pub const COMMISSIONING_MAX_ANGULAR_RPS: f64 = 0.2 * crate::motion::MAX_ANGULAR_RPS;
/// Wait after the descent before the first frame (s).
pub const MIN_SETTLE_S: f64 = 2.5;
pub const MAX_SETTLE_S: f64 = 10.0;
/// Distinct frames thrown away after the settle (the snapshot lags 1-2 frames after motion).
pub const MIN_DISCARD: usize = 2;
pub const DEFAULT_FRAMES: usize = 3;
pub const MAX_FRAMES: usize = 5;
/// Hover target: at least this far above the effective floor (m).
pub const MIN_HOVER_ABOVE_FLOOR_M: f64 = 0.005;
/// Approach height above the hover target (m).
pub const APPROACH_RISE_M: [f64; 2] = [0.02, 0.15];
/// Approach and hover share x, y within this (m): the descent is vertical.
pub const MAX_DESCENT_XY_M: f64 = 0.002;
/// A capture is stationary when every arm joint is within this of the settled position (rad).
pub const STILL_RAD: f32 = 0.003;
pub const FEEDBACK_MAX_AGE: Duration = Duration::from_millis(250);
/// One ArUco trigger waits this long for a fresh frame (ms).
const TRIGGER_WAIT_MS: u64 = 2500;
/// Triggers per point before the capture gives up.
const MAX_TRIGGERS: usize = 12;

fn invalid(m: impl Into<String>) -> ArmError { ArmError::detailed(400, "INVALID_COMMAND", m.into(), Value::Null) }

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Observation {
    pub frame_id: u64,
    /// Board markers (ids 0..11) with 4 corners each in rebot-vision's order and pixels.
    pub markers: Value,
    /// Measured joint positions at the capture (armd joint angles, rad).
    pub joints: Vec<f32>,
    pub image_size: [f64; 2],
    /// Seconds since the job started.
    pub t_s: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HoverPoint {
    pub id: String,
    pub approach_m: [f64; 3],
    pub hover_m: [f64; 3],
    pub rpy_rad: [f64; 3],
    /// `pending`, `done`, `skipped` (refused before moving), `failed` (the run ended here).
    pub state: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub code: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub message: Option<String>,
    #[serde(default)]
    pub observations: Vec<Observation>,
    /// Frame ids discarded after the settle.
    #[serde(default)]
    pub discarded: Vec<u64>,
    /// Settled joint positions (measured) and their live FK end_link position.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub settled_joints: Option<Vec<f32>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub settled_end_m: Option<[f64; 3]>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HoverParams {
    pub approach_speed_mps: f64,
    pub descent_speed_mps: f64,
    pub settle_s: f64,
    pub frames: usize,
    pub discard: usize,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HoverJob {
    pub id: String,
    pub candidate_fingerprint: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tool_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tool_config_digest: Option<String>,
    pub created_at: String,
    pub updated_at: String,
    pub params: HoverParams,
    pub points: Vec<HoverPoint>,
    /// `running`, `done`, `failed`, `cancelled`.
    pub state: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub code: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub message: Option<String>,
    /// Action id of the current / last run.
    pub action_id: Option<u64>,
    pub runs: u32,
    /// Arm state when the run ended (`is_torqued`, `safety_state`, `fault`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub end: Option<Value>,
}

/// A checked run, ready to begin.
#[derive(Debug, Clone)]
pub struct HoverPlan {
    pub job: HoverJob,
    pub epoch: u64,
    pub workspace_revision: u64,
    pub calibration_revision: u64,
    pub angular_speed_rps: f64,
    pub deadline_s: f64,
}

fn now() -> String { chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Millis, true) }

fn vec3(v: Option<&Value>, what: &str, max: f64) -> Result<[f64; 3], ArmError> {
    let a = v.and_then(Value::as_array).filter(|a| a.len() == 3).ok_or_else(|| invalid(format!("{what} must be three numbers")))?;
    let mut out = [0.0; 3];
    for (o, x) in out.iter_mut().zip(a) {
        *o = x.as_f64().filter(|x| x.is_finite() && x.abs() <= max).ok_or_else(|| invalid(format!("{what} must be finite numbers within {max}")))?;
    }
    Ok(out)
}

fn num(body: &Value, key: &str, default: f64, lo: f64, hi: f64) -> Result<f64, ArmError> {
    match body.get(key) {
        None | Some(Value::Null) => Ok(default),
        Some(v) => v.as_f64().filter(|x| x.is_finite() && (lo..=hi).contains(x)).ok_or_else(|| invalid(format!("{key} must be in [{lo}, {hi}]"))),
    }
}

fn valid_job_id(id: &str) -> bool { !id.is_empty() && id.len() <= 40 && id.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'-') }

fn validate_hover_point(i: usize, approach: [f64; 3], hover: [f64; 3], floor: f64) -> Result<(), ArmError> {
    let rise = approach[2] - hover[2];
    if !(APPROACH_RISE_M[0]..=APPROACH_RISE_M[1]).contains(&rise) {
        return Err(invalid(format!("points[{i}]: approach_m must be {}..{} m above hover_m (is {rise:.3})", APPROACH_RISE_M[0], APPROACH_RISE_M[1])));
    }
    if (approach[0] - hover[0]).hypot(approach[1] - hover[1]) > MAX_DESCENT_XY_M {
        return Err(invalid(format!("points[{i}]: the descent must be vertical (approach and hover x, y within {MAX_DESCENT_XY_M} m)")));
    }
    if hover[2] < floor + MIN_HOVER_ABOVE_FLOOR_M {
        return Err(ArmError::detailed(422, "WORKSPACE_VIOLATION", format!("points[{i}]: hover_m z {:.4} m is less than {MIN_HOVER_ABOVE_FLOOR_M} m above the effective floor {floor:.4} m", hover[2]), json!({"point": i})));
    }
    Ok(())
}

/// Check a run under the lock (fresh job or `resume`).
pub fn prepare(inner: &SupervisorInner, body: &Value, epoch: u64) -> Result<HoverPlan, ArmError> {
    if epoch != inner.safety.epoch_id() {
        return Err(crate::safety::SafetyError::EpochMismatch { expected: inner.safety.epoch_id(), received: epoch }.into());
    }
    if inner.workspace_policy().is_none() { return Err(ArmError::detailed(400, "UNSUPPORTED", "calib_hover needs the Cartesian workspace (profile workspace + kinematics)", Value::Null)); }
    let o = body.as_object().ok_or_else(|| invalid("Command body must be an object"))?;
    let floor = inner.workspace_policy().map_or(0.0, |(ws, _)| f64::from(ws.floor_z_m));
    let mut job = if let Some(r) = o.get("resume") {
        for k in o.keys() {
            if k != "resume" && k != "candidate_fingerprint" {
                return Err(invalid(format!("resume takes no other field ({k})")));
            }
        }
        let id = r.as_str().filter(|i| valid_job_id(i)).ok_or_else(|| invalid("resume must be a job id"))?;
        let mut job = inner.hover_jobs.iter().find(|j| j.id == id).cloned()
            .ok_or_else(|| ArmError::detailed(404, "JOB_NOT_FOUND", format!("no hover job {id} (GET /api/v1/calibration/hover)"), Value::Null))?;
        if job.state == "running" { return Err(ArmError::detailed(409, "JOB_RUNNING", format!("hover job {id} is running"), Value::Null)); }
        if !inner.config.active_tool.matches_record(job.tool_id.as_deref(), job.tool_config_digest.as_deref()) {
            return Err(ArmError::detailed(
                409,
                "TOOL_MISMATCH",
                format!(
                    "hover job {id} belongs to tool {:?}/{:?}, not active tool {}/{}",
                    job.tool_id, job.tool_config_digest, inner.config.active_tool.id, inner.config.active_tool.config_digest
                ),
                Value::Null,
            ));
        }
        let fp = o.get("candidate_fingerprint")
            .and_then(Value::as_str)
            .filter(|f| !f.is_empty() && f.len() <= 64)
            .ok_or_else(|| invalid("resume requires candidate_fingerprint (1..64 bytes)"))?;
        if fp != job.candidate_fingerprint {
            return Err(ArmError::detailed(
                409,
                "FINGERPRINT_MISMATCH",
                format!("hover job {id} belongs to candidate {}, not {}", job.candidate_fingerprint, fp),
                Value::Null,
            ));
        }
        for p in job.points.iter_mut().filter(|p| p.state != "done") {
            p.state = "pending".into(); p.code = None; p.message = None; p.observations.clear(); p.discarded.clear(); p.settled_joints = None; p.settled_end_m = None;
        }
        if job.points.iter().all(|p| p.state == "done") { return Err(invalid(format!("hover job {id} has no point left"))); }
        for (i, p) in job.points.iter().enumerate().filter(|(_, p)| p.state != "done") {
            validate_hover_point(i, p.approach_m, p.hover_m, floor)?;
        }
        job
    } else {
        for k in o.keys() {
            if !["candidate_fingerprint", "points", "rpy_rad", "approach_speed_mps", "descent_speed_mps", "settle_s", "frames", "discard"].contains(&k.as_str()) {
                return Err(invalid(format!("unknown field: {k}")));
            }
        }
        let fp = o.get("candidate_fingerprint").and_then(Value::as_str).filter(|f| !f.is_empty() && f.len() <= 64)
            .ok_or_else(|| invalid("candidate_fingerprint must be a string of 1..64 bytes (dry-run apply after.fingerprint)"))?;
        let rpy = vec3(o.get("rpy_rad"), "rpy_rad", 7.0)?;
        let pts = o.get("points").and_then(Value::as_array).filter(|p| (1..=MAX_POINTS).contains(&p.len()))
            .ok_or_else(|| invalid(format!("points must be 1..{MAX_POINTS} objects")))?;
        let params = HoverParams {
            approach_speed_mps: num(body, "approach_speed_mps", DEFAULT_APPROACH_MPS, 0.005, MAX_APPROACH_MPS)?,
            descent_speed_mps: num(body, "descent_speed_mps", DEFAULT_DESCENT_MPS, 0.002, MAX_DESCENT_MPS)?,
            settle_s: num(body, "settle_s", MIN_SETTLE_S, MIN_SETTLE_S, MAX_SETTLE_S)?,
            frames: num(body, "frames", DEFAULT_FRAMES as f64, 1.0, MAX_FRAMES as f64)? as usize,
            discard: num(body, "discard", MIN_DISCARD as f64, MIN_DISCARD as f64, 4.0)? as usize,
        };
        let mut points = Vec::with_capacity(pts.len());
        for (i, p) in pts.iter().enumerate() {
            let id = p.get("id").and_then(Value::as_str).filter(|s| !s.is_empty() && s.len() <= 32).ok_or_else(|| invalid(format!("points[{i}].id must be a string of 1..32 bytes")))?;
            let approach = vec3(p.get("approach_m"), &format!("points[{i}].approach_m"), 1.0)?;
            let hover = vec3(p.get("hover_m"), &format!("points[{i}].hover_m"), 1.0)?;
            let prpy = match p.get("rpy_rad") { None | Some(Value::Null) => rpy, v => vec3(v, &format!("points[{i}].rpy_rad"), 7.0)? };
            validate_hover_point(i, approach, hover, floor)?;
            points.push(HoverPoint { id: id.into(), approach_m: approach, hover_m: hover, rpy_rad: prpy, state: "pending".into(), code: None, message: None,
                observations: Vec::new(), discarded: Vec::new(), settled_joints: None, settled_end_m: None });
        }
        let t = now();
        let nanos = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_nanos());
        HoverJob {
            id: format!("hv-{nanos:x}"),
            candidate_fingerprint: fp.into(),
            tool_id: Some(inner.config.active_tool.id.clone()),
            tool_config_digest: Some(inner.config.active_tool.config_digest.clone()),
            created_at: t.clone(),
            updated_at: t,
            params,
            points,
            state: "pending".into(),
            code: None,
            message: None,
            action_id: None,
            runs: 0,
            end: None,
        }
    };
    inner.action_preconditions()?;
    inner.gravity_comp_motion_busy()?;
    if !inner.is_torqued { return Err(crate::safety::SafetyError::NotArmed(inner.safety.state().clone()).into()); }
    let angular_speed_rps = if inner.config.active_tool.commissioning_required {
        job.params.approach_speed_mps = job.params.approach_speed_mps.min(COMMISSIONING_MAX_LINEAR_MPS);
        job.params.descent_speed_mps = job.params.descent_speed_mps.min(COMMISSIONING_MAX_LINEAR_MPS);
        crate::motion::DEFAULT_ANGULAR_RPS.min(COMMISSIONING_MAX_ANGULAR_RPS)
    } else {
        crate::motion::DEFAULT_ANGULAR_RPS
    };
    // bound: per pending point approach (from up to 0.6 m away) + descent + settle + capture + retreat
    let p = &job.params;
    let per_point = 0.6 / p.approach_speed_mps + 2.0 * APPROACH_RISE_M[1] / p.descent_speed_mps + p.settle_s
        + (p.discard + p.frames) as f64 * (TRIGGER_WAIT_MS as f64 / 1000.0 + 0.5) + 3.0 * crate::motion::SETTLE_MAX_S;
    let pending = job.points.iter().filter(|q| q.state == "pending").count() as f64;
    Ok(HoverPlan {
        job,
        epoch,
        workspace_revision: inner.workspace_revision,
        calibration_revision: inner.calibration_revision,
        angular_speed_rps,
        deadline_s: per_point * pending + 30.0,
    })
}

/// Prepare a `calib_hover` plan from a published [`ActionSnapshot`] + existing hover jobs
/// (0 `SupervisorLock` on `OWNER_EXECUTOR`).
pub fn prepare_from_snapshot(
    snap: &crate::rt_core::requests::actions::ActionSnapshot,
    existing_jobs: &[HoverJob],
    body: &Value,
    epoch: u64,
) -> Result<HoverPlan, ArmError> {
    if epoch != snap.owner.authority.epoch {
        return Err(crate::safety::SafetyError::EpochMismatch {
            expected: snap.owner.authority.epoch,
            received: epoch,
        }
        .into());
    }
    let Some(ws) = snap.env.workspace_policy.as_ref() else {
        return Err(ArmError::detailed(
            400,
            "UNSUPPORTED",
            "calib_hover needs the Cartesian workspace (profile workspace + kinematics)",
            Value::Null,
        ));
    };
    let o = body.as_object().ok_or_else(|| invalid("Command body must be an object"))?;
    let floor = f64::from(ws.floor_z_m);
    let mut job = if let Some(r) = o.get("resume") {
        for k in o.keys() {
            if k != "resume" && k != "candidate_fingerprint" {
                return Err(invalid(format!("resume takes no other field ({k})")));
            }
        }
        let id = r.as_str().filter(|i| valid_job_id(i)).ok_or_else(|| invalid("resume must be a job id"))?;
        let mut job = existing_jobs
            .iter()
            .find(|j| j.id == id)
            .cloned()
            .ok_or_else(|| ArmError::detailed(404, "JOB_NOT_FOUND", format!("no hover job {id} (GET /api/v1/calibration/hover)"), Value::Null))?;
        if job.state == "running" {
            return Err(ArmError::detailed(409, "JOB_RUNNING", format!("hover job {id} is running"), Value::Null));
        }
        if !snap.env.config.active_tool.matches_record(job.tool_id.as_deref(), job.tool_config_digest.as_deref()) {
            return Err(ArmError::detailed(
                409,
                "TOOL_MISMATCH",
                format!(
                    "hover job {id} belongs to tool {:?}/{:?}, not active tool {}/{}",
                    job.tool_id, job.tool_config_digest, snap.env.config.active_tool.id, snap.env.config.active_tool.config_digest
                ),
                Value::Null,
            ));
        }
        let fp = o
            .get("candidate_fingerprint")
            .and_then(Value::as_str)
            .filter(|f| !f.is_empty() && f.len() <= 64)
            .ok_or_else(|| invalid("resume requires candidate_fingerprint (1..64 bytes)"))?;
        if fp != job.candidate_fingerprint {
            return Err(ArmError::detailed(
                409,
                "FINGERPRINT_MISMATCH",
                format!("hover job {id} belongs to candidate {}, not {}", job.candidate_fingerprint, fp),
                Value::Null,
            ));
        }
        for p in job.points.iter_mut().filter(|p| p.state != "done") {
            p.state = "pending".into();
            p.code = None;
            p.message = None;
            p.observations.clear();
            p.discarded.clear();
            p.settled_joints = None;
            p.settled_end_m = None;
        }
        if job.points.iter().all(|p| p.state == "done") {
            return Err(invalid(format!("hover job {id} has no point left")));
        }
        for (i, p) in job.points.iter().enumerate().filter(|(_, p)| p.state != "done") {
            validate_hover_point(i, p.approach_m, p.hover_m, floor)?;
        }
        job
    } else {
        for k in o.keys() {
            if !["candidate_fingerprint", "points", "rpy_rad", "approach_speed_mps", "descent_speed_mps", "settle_s", "frames", "discard"].contains(&k.as_str()) {
                return Err(invalid(format!("unknown field: {k}")));
            }
        }
        let fp = o
            .get("candidate_fingerprint")
            .and_then(Value::as_str)
            .filter(|f| !f.is_empty() && f.len() <= 64)
            .ok_or_else(|| invalid("candidate_fingerprint must be a string of 1..64 bytes (dry-run apply after.fingerprint)"))?;
        let rpy = vec3(o.get("rpy_rad"), "rpy_rad", 7.0)?;
        let pts = o
            .get("points")
            .and_then(Value::as_array)
            .filter(|p| (1..=MAX_POINTS).contains(&p.len()))
            .ok_or_else(|| invalid(format!("points must be 1..{MAX_POINTS} objects")))?;
        let params = HoverParams {
            approach_speed_mps: num(body, "approach_speed_mps", DEFAULT_APPROACH_MPS, 0.005, MAX_APPROACH_MPS)?,
            descent_speed_mps: num(body, "descent_speed_mps", DEFAULT_DESCENT_MPS, 0.002, MAX_DESCENT_MPS)?,
            settle_s: num(body, "settle_s", MIN_SETTLE_S, MIN_SETTLE_S, MAX_SETTLE_S)?,
            frames: num(body, "frames", DEFAULT_FRAMES as f64, 1.0, MAX_FRAMES as f64)? as usize,
            discard: num(body, "discard", MIN_DISCARD as f64, MIN_DISCARD as f64, 4.0)? as usize,
        };
        let mut points = Vec::with_capacity(pts.len());
        for (i, p) in pts.iter().enumerate() {
            let id = p.get("id").and_then(Value::as_str).filter(|s| !s.is_empty() && s.len() <= 32).ok_or_else(|| invalid(format!("points[{i}].id must be a string of 1..32 bytes")))?;
            let approach = vec3(p.get("approach_m"), &format!("points[{i}].approach_m"), 1.0)?;
            let hover = vec3(p.get("hover_m"), &format!("points[{i}].hover_m"), 1.0)?;
            let prpy = match p.get("rpy_rad") { None | Some(Value::Null) => rpy, v => vec3(v, &format!("points[{i}].rpy_rad"), 7.0)? };
            validate_hover_point(i, approach, hover, floor)?;
            points.push(HoverPoint {
                id: id.into(),
                approach_m: approach,
                hover_m: hover,
                rpy_rad: prpy,
                state: "pending".into(),
                code: None,
                message: None,
                observations: Vec::new(),
                discarded: Vec::new(),
                settled_joints: None,
                settled_end_m: None,
            });
        }
        let t = now();
        let nanos = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_nanos());
        HoverJob {
            id: format!("hv-{nanos:x}"),
            candidate_fingerprint: fp.into(),
            tool_id: Some(snap.env.config.active_tool.id.clone()),
            tool_config_digest: Some(snap.env.config.active_tool.config_digest.clone()),
            created_at: t.clone(),
            updated_at: t,
            params,
            points,
            state: "pending".into(),
            code: None,
            message: None,
            action_id: None,
            runs: 0,
            end: None,
        }
    };
    snap.check_preconditions()?;
    if !snap.owner.is_torqued {
        return Err(crate::safety::SafetyError::NotArmed(snap.safety_state.clone()).into());
    }
    let angular_speed_rps = if snap.env.config.active_tool.commissioning_required {
        job.params.approach_speed_mps = job.params.approach_speed_mps.min(COMMISSIONING_MAX_LINEAR_MPS);
        job.params.descent_speed_mps = job.params.descent_speed_mps.min(COMMISSIONING_MAX_LINEAR_MPS);
        crate::motion::DEFAULT_ANGULAR_RPS.min(COMMISSIONING_MAX_ANGULAR_RPS)
    } else {
        crate::motion::DEFAULT_ANGULAR_RPS
    };
    let p = &job.params;
    let per_point = 0.6 / p.approach_speed_mps + 2.0 * APPROACH_RISE_M[1] / p.descent_speed_mps + p.settle_s
        + (p.discard + p.frames) as f64 * (TRIGGER_WAIT_MS as f64 / 1000.0 + 0.5) + 3.0 * crate::motion::SETTLE_MAX_S;
    let pending = job.points.iter().filter(|q| q.state == "pending").count() as f64;
    Ok(HoverPlan {
        job,
        epoch,
        workspace_revision: snap.env.workspace_revision,
        calibration_revision: snap.env.calibration_revision,
        angular_speed_rps,
        deadline_s: per_point * pending + 30.0,
    })
}

/// Store `job` (replacing one with the same id; at most [`KEEP_JOBS`]).
fn put_job(inner: &mut SupervisorInner, job: HoverJob) {
    inner.hover_jobs.retain(|j| j.id != job.id);
    inner.hover_jobs.push(job);
    let n = inner.hover_jobs.len();
    if n > KEEP_JOBS { inner.hover_jobs.drain(..n - KEEP_JOBS); }
}

/// `calibration-hover/` next to `calibration.json`.
pub fn jobs_dir(calibration_path: &Path) -> PathBuf { calibration_path.with_file_name("calibration-hover") }

/// Build the A7 [`BeginMacroSpec`] for a prepared calibration hover job (shared by Legacy and RT Executor).
pub fn begin_macro_spec(
    plan: &HoverPlan,
    prepared_boot: crate::rt_core::broker::BootId,
    deadline: Instant,
) -> crate::rt_core::requests::actions::BeginMacroSpec {
    use crate::rt_core::requests::actions::{BeginMacroSpec, MacroKind};
    let mut spec = BeginMacroSpec::new(MacroKind::CalibHover, prepared_boot);
    spec.check_workspace_revision = true;
    spec.calibration_revision = Some(plan.calibration_revision);
    spec.macro_deadline = Some(deadline);
    spec.timeout_code = crate::safety::BoundedText::new("HOVER_TIMEOUT");
    spec.tag = crate::model::HolderLabel::new(&plan.job.id);
    spec
}

fn finish_begin_hover(
    inner: &mut SupervisorInner,
    sup: &Arc<ArmSupervisor>,
    id: u64,
    mut plan: HoverPlan,
    upstream: crate::api::proxy::DetectorUpstream,
    dir: PathBuf,
    stamp: crate::supervisor::AdmissionStamp,
    deadline_at: Instant,
) -> (u64, f64, String) {
    plan.job.state = "running".into();
    plan.job.code = None; plan.job.message = None; plan.job.end = None;
    plan.job.action_id = Some(id);
    plan.job.runs += 1;
    plan.job.updated_at = now();
    let job_id = plan.job.id.clone();
    put_job(inner, plan.job.clone());
    let deadline_s = plan.deadline_s;
    tracing::info!("Calibration hover job {job_id} run {} started (action {id}, {} points pending, bound {deadline_s:.0} s)",
        plan.job.runs, plan.job.points.iter().filter(|p| p.state == "pending").count());
    let sup = sup.clone();
    tokio::spawn(async move {
        let worker = { let sup = sup.clone(); let dir = dir.clone(); tokio::spawn(async move { run(sup, id, plan, upstream, dir, stamp, deadline_at).await }) };
        if let Err(e) = worker.await {
            tracing::error!("Calibration hover {id} orchestrator failed: {e}");
            if sup.owner_mode() == crate::supervisor::OWNER_EXECUTOR {
                let ctx = MacroCtx {
                    sup: sup.clone(),
                    id,
                    stamp,
                    deadline: deadline_at,
                    phase_timeout_s: deadline_s,
                    timeout_code: "HOVER_TIMEOUT",
                    timeout_message: String::new(),
                };
                ctx.finish(
                    crate::rt_core::requests::actions::FinishState::Failed,
                    Some("INTERNAL"),
                    Some(format!("hover orchestrator failed: {e}")),
                    None,
                    None,
                    crate::rt_core::requests::actions::HeldUpdate::Unchanged,
                )
                .await;
            } else {
                let mut inner = sup.inner().write().await;
                inner.finish_macro(id, "failed", Some("INTERNAL"), Some(format!("hover orchestrator failed: {e}")), None, None);
                if let Some(j) = inner.hover_jobs.iter_mut().find(|j| j.action_id == Some(id) && j.state == "running") {
                    j.state = "failed".into(); j.code = Some("INTERNAL".into()); j.message = Some(e.to_string());
                }
            }
        }
    });
    (id, deadline_s, job_id)
}

/// Begin the run under a short write lock through the single owner-side
/// [`crate::rt_core::requests::actions::install_begin_macro`] function and spawn its orchestrator.
pub fn begin_with_stamp(
    inner: &mut SupervisorInner,
    stamp: crate::supervisor::AdmissionStamp,
    sup: &Arc<ArmSupervisor>,
    plan: HoverPlan,
    upstream: crate::api::proxy::DetectorUpstream,
    dir: PathBuf,
) -> Result<(u64, f64, String), ArmError> {
    use crate::rt_core::requests::actions::{
        install_begin_macro, ActionStamp, OwnerCheckCtx, DEFAULT_ACTION_DEADLINE,
    };
    let deadline_at = Instant::now() + Duration::from_secs_f64(plan.deadline_s);
    let spec = begin_macro_spec(&plan, inner.boot_id, deadline_at);
    let action_stamp = ActionStamp {
        arm: inner.core.id(),
        boot: inner.boot_id,
        admission: stamp,
        config_revision: plan.workspace_revision,
        calibration_revision: Some(plan.calibration_revision),
        poses_revision: None,
        sequences_revision: None,
        grasp_setup_revision: None,
        gripper_calib_revision: None,
        power_budget_revision: None,
        parent_generation: 0,
        deadline: Instant::now() + DEFAULT_ACTION_DEADLINE,
    };
    let owner_ctx = OwnerCheckCtx::for_legacy(inner);
    let id = install_begin_macro(
        inner,
        &action_stamp,
        &owner_ctx,
        &spec,
        Some(plan.job.id.clone()),
        None,
    )
    .map_err(|r| r.into_arm_error(inner))?;
    Ok(finish_begin_hover(inner, sup, id, plan, upstream, dir, stamp, deadline_at))
}

/// Begin the run (parent action `calib_hover`, tag = job id) and spawn its orchestrator.
/// Returns (action id, deadline bound s, job id).
pub fn begin(admitted: crate::supervisor::Admitted<'_>, sup: &Arc<ArmSupervisor>, plan: HoverPlan, upstream: crate::api::proxy::DetectorUpstream, dir: PathBuf) -> Result<(u64, f64, String), ArmError> {
    let deadline_at = Instant::now() + Duration::from_secs_f64(plan.deadline_s);
    let (inner, proof) = admitted.consume()?;
    let stamp = proof.stamp();
    if inner.safety.epoch_id() != plan.epoch {
        return Err(crate::safety::SafetyError::EpochMismatch {
            expected: inner.safety.epoch_id(),
            received: plan.epoch,
        }
        .into());
    }
    let spec = begin_macro_spec(&plan, inner.boot_id, deadline_at);
    let ws_rev = plan.workspace_revision;
    let id = crate::rt_core::requests::actions::install_begin_macro_with_proof(
        inner,
        &proof,
        &spec,
        Some(ws_rev),
        Some(plan.job.id.clone()),
        None,
    )?;
    Ok(finish_begin_hover(inner, sup, id, plan, upstream, dir, stamp, deadline_at))
}

/// Production broker-to-worker handoff for `calib_hover` on `OWNER_EXECUTOR` (A7 / Major 11 + Minor 2):
/// submits `BeginMacro` carrying the caller's original `AdmissionStamp` and exact absolute
/// `deadline_at`, records live job state via the owner-aware store, and spawns the worker with
/// the accepted parent `ActionId`, original `stamp`, and exact `deadline_at` (0 `SupervisorLock`).
pub async fn begin_executor(
    sup: &Arc<ArmSupervisor>,
    stamp: crate::supervisor::AdmissionStamp,
    mut plan: HoverPlan,
    upstream: crate::api::proxy::DetectorUpstream,
    dir: PathBuf,
) -> Result<(u64, f64, String), ArmError> {
    use crate::rt_core::requests::actions::{ActionStamp, DEFAULT_ACTION_DEADLINE};
    let handle = sup.executor_handle().ok_or_else(|| ArmError::Rejected {
        status: 503,
        code: "EXECUTOR_UNAVAILABLE".into(),
        message: "RT executor is not active".into(),
    })?;
    let boot = handle.boot();
    let arm = sup
        .hub()
        .latest()
        .map_or(crate::rt_core::ids::ArmId::LOCAL, |s| s.rt.owner.arm);
    let now_inst = Instant::now();
    let deadline_at = now_inst + Duration::from_secs_f64(plan.deadline_s);
    let mut spec = begin_macro_spec(&plan, boot, deadline_at);
    spec.admission_request_id = stamp.request_id;
    let action_stamp = ActionStamp {
        arm,
        boot,
        admission: stamp,
        config_revision: plan.workspace_revision,
        calibration_revision: Some(plan.calibration_revision),
        poses_revision: None,
        sequences_revision: None,
        grasp_setup_revision: None,
        gripper_calib_revision: None,
        power_budget_revision: None,
        parent_generation: 0,
        deadline: now_inst + DEFAULT_ACTION_DEADLINE,
    };
    let (id, _req_id) =
        crate::macro_action::begin_macro_executor(sup, &action_stamp, spec, action_stamp.deadline)
            .await?;
    plan.job.state = "running".into();
    plan.job.code = None;
    plan.job.message = None;
    plan.job.end = None;
    plan.job.action_id = Some(id);
    plan.job.runs += 1;
    plan.job.updated_at = now();
    let job_id = plan.job.id.clone();
    let deadline_s = plan.deadline_s;
    sup.put_hover_job(plan.job.clone()).await;
    tracing::info!(
        "Calibration hover job {} started on RT executor (action {}, run {}, {} points, bound {:.0} s)",
        job_id,
        id,
        plan.job.runs,
        plan.job.points.len(),
        deadline_s
    );
    let sup = sup.clone();
    tokio::spawn(async move {
        let worker = {
            let sup = sup.clone();
            tokio::spawn(async move { run(sup, id, plan, upstream, dir, stamp, deadline_at).await })
        };
        if let Err(e) = worker.await {
            tracing::error!("Calibration hover {id} orchestrator failed: {e}");
            let ctx = MacroCtx {
                sup: sup.clone(),
                id,
                stamp,
                deadline: deadline_at,
                phase_timeout_s: deadline_s,
                timeout_code: "HOVER_TIMEOUT",
                timeout_message: String::new(),
            };
            ctx.finish(
                crate::rt_core::requests::actions::FinishState::Failed,
                Some("INTERNAL"),
                Some(format!("hover orchestrator failed: {e}")),
                None,
                None,
                crate::rt_core::requests::actions::HeldUpdate::Unchanged,
            )
            .await;
            let jobs = sup.hover_jobs_snapshot().await;
            if let Some(mut j) = jobs.into_iter().find(|j| j.action_id == Some(id) && j.state == "running") {
                j.state = "failed".into();
                j.code = Some("INTERNAL".into());
                j.message = Some(e.to_string());
                j.updated_at = now();
                sup.put_hover_job(j).await;
            }
        }
    });
    Ok((id, deadline_s, job_id))
}

/// Update the stored job of this run (owner-aware: uses sidecar store on `OWNER_EXECUTOR`).
async fn update(sup: &ArmSupervisor, job: &HoverJob) {
    sup.put_hover_job(job.clone()).await;
}

async fn persist(dir: &Path, job: &HoverJob) {
    let (d, text, id) = (dir.to_path_buf(), serde_json::to_string_pretty(job).unwrap_or_default(), job.id.clone());
    let res = tokio::task::spawn_blocking(move || -> std::io::Result<()> {
        let dir = d;
        std::fs::create_dir_all(&dir)?;
        // only the newest KEEP_JOBS files stay
        let own = format!("{id}.json");
        let mut files: Vec<_> = std::fs::read_dir(&dir)?.flatten()
            .filter(|e| { let n = e.file_name().to_string_lossy().into_owned(); n.starts_with("hv-") && n != own }).collect();
        files.sort_by_key(|e| e.file_name());
        while files.len() >= KEEP_JOBS { let _ = std::fs::remove_file(files.remove(0).path()); }
        let tmp = dir.join(format!(".{id}.tmp"));
        std::fs::write(&tmp, text)?;
        std::fs::rename(tmp, dir.join(format!("{id}.json")))
    }).await;
    match res {
        Ok(Ok(())) => {}
        Ok(Err(e)) => tracing::warn!("hover job {}: not written to {}: {e}", job.id, dir.display()),
        Err(e) => tracing::warn!("hover job {}: write task failed: {e}", job.id),
    }
}

/// Alive, nothing it was checked against changed, telemetry fresh, torque on.
async fn recheck(ctx: &MacroCtx, plan: &HoverPlan) -> Result<(), Stop> {
    if ctx.is_executor() {
        let snap = ctx.snapshot(ctx.deadline).await?;
        let m = snap.owner.macro_state;
        if m.active_id != ctx.id || !m.alive { return Err(Stop::Cancelled); }
        if snap.owner.authority.epoch != plan.epoch { return Err(fail("PLAN_STALE", "the epoch changed during the hovers (stop or e-stop)")); }
        if snap.owner.authority.config_revision != plan.workspace_revision { return Err(fail("PLAN_STALE", "the workspace, floor or camera mount changed during the hovers")); }
        if snap.owner.calibration_revision != plan.calibration_revision { return Err(fail("CALIBRATION_CHANGED", "the joint zero or the kinematic correction changed during the hovers")); }
        if let Some(pub_snap) = ctx.sup.hub().latest() {
            if !pub_snap.obs.is_torqued { return Err(fail("NOT_ARMED", "torque went off during the hovers")); }
        }
        if snap.env.driver_present {
            let now = Instant::now();
            let arm = snap.env.config.arm_joint_indices();
            let fresh = arm.iter().all(|&i| {
                snap.feedback_instants
                    .get(i)
                    .copied()
                    .flatten()
                    .is_some_and(|t| now.saturating_duration_since(t) <= FEEDBACK_MAX_AGE)
            });
            if !fresh { return Err(fail("STALE_TELEMETRY", "joint feedback is stale")); }
        }
        if Instant::now() > ctx.deadline { return Err(fail(ctx.timeout_code, ctx.timeout_message.clone())); }
        return Ok(());
    }
    let mut inner = ctx.sup.inner().write().await;
    if !inner.macro_alive(ctx.id) { return Err(Stop::Cancelled); }
    if inner.safety.epoch_id() != plan.epoch { return Err(fail("PLAN_STALE", "the epoch changed during the hovers (stop or e-stop)")); }
    if inner.workspace_revision != plan.workspace_revision { return Err(fail("PLAN_STALE", "the workspace, floor or camera mount changed during the hovers")); }
    if inner.calibration_revision != plan.calibration_revision { return Err(fail("CALIBRATION_CHANGED", "the joint zero or the kinematic correction changed during the hovers")); }
    if inner.active_lease.as_ref().map(|l| l.generation) != inner.action_lease_generation {
        let id = ctx.id;
        inner.finish_macro(id, "cancelled", Some("LEASE_LOST"), Some("Control lease lost or released".into()), None, None);
        return Err(Stop::Cancelled);
    }
    if !inner.is_torqued { return Err(fail("NOT_ARMED", "torque went off during the hovers")); }
    if !inner.hardware_feedback_fresh(FEEDBACK_MAX_AGE) { return Err(fail("STALE_TELEMETRY", "joint feedback is stale")); }
    if Instant::now() > ctx.deadline { return Err(fail(ctx.timeout_code, ctx.timeout_message.clone())); }
    Ok(())
}

/// Measured arm joints, still (all arm joint speeds small) or None.
async fn measured(ctx: &MacroCtx) -> (Vec<f32>, bool) {
    let (pos, vel, arm) = ctx.measured_with_vel().await;
    let still = arm.iter().all(|&i| vel.get(i).is_none_or(|v| v.abs() <= 0.02));
    (pos, still)
}

/// One ArUco trigger: (frame_id, board markers, image size).
async fn trigger(up: &crate::api::proxy::DetectorUpstream) -> Result<Option<(u64, Value, [f64; 2])>, String> {
    let body = json!({"pipelines": ["aruco"], "wait_ms": TRIGGER_WAIT_MS}).to_string();
    let (status, v) = crate::api::proxy::request_status_json(up, "POST", "/api/trigger", Some(body.as_bytes()), Duration::from_millis(TRIGGER_WAIT_MS + 1500)).await?;
    if !status.is_success() { return Err(format!("rebot-vision trigger returned {status}")); }
    if v.get("timeout").and_then(Value::as_bool) == Some(true) || v.get("success").and_then(Value::as_bool) == Some(false) { return Ok(None); }
    let Some(frame_id) = v.get("frame_id").and_then(Value::as_u64) else { return Ok(None); };
    let size = [v.get("width").and_then(Value::as_f64).unwrap_or(640.0), v.get("height").and_then(Value::as_f64).unwrap_or(360.0)];
    let markers: Vec<Value> = v.get("markers").and_then(Value::as_array).map(|a| a.iter().filter_map(|m| {
        let id = m.get("id").and_then(Value::as_i64).filter(|i| (0..12).contains(i))?;
        let c = m.get("corners").and_then(Value::as_array).filter(|c| c.len() == 4)?;
        let corners: Option<Vec<[f64; 2]>> = c.iter().map(|p| {
            let p = p.as_array()?;
            Some([p.first()?.as_f64().filter(|x| x.is_finite())?, p.get(1)?.as_f64().filter(|x| x.is_finite())?])
        }).collect();
        Some(json!({"id": id, "corners": corners?}))
    }).collect()).unwrap_or_default();
    Ok(Some((frame_id, Value::Array(markers), size)))
}

/// Settle, discard frames, collect fresh stationary observations.
async fn capture(ctx: &MacroCtx, plan: &HoverPlan, point: &mut HoverPoint, up: &crate::api::proxy::DetectorUpstream, t0: Instant) -> Result<(), Stop> {
    let p = &plan.job.params;
    let until = Instant::now() + Duration::from_secs_f64(p.settle_s);
    while Instant::now() < until {
        ctx.check().await?;
        tokio::time::sleep(POLL).await;
    }
    recheck(ctx, plan).await?;
    let (settled, still) = measured(ctx).await;
    if !still { return Err(fail("NOT_SETTLED", "the arm is still moving after the settle time")); }
    let arm = if ctx.is_executor() {
        let env = ctx.sup.hub().prep_env();
        point.settled_end_m = env
            .as_ref()
            .and_then(|e| e.config.arm_model.fk_f32(&settled).map(|f| f.p));
        env.map_or_else(Vec::new, |e| e.config.arm_joint_indices())
    } else {
        let inner = ctx.sup.inner().read().await;
        point.settled_end_m = inner.config.arm_model.fk_f32(&settled).map(|f| f.p);
        inner.config.arm_joint_indices()
    };
    point.settled_joints = Some(settled.clone());
    let mut last: Option<u64> = None;
    for _ in 0..MAX_TRIGGERS {
        if point.observations.len() >= p.frames { break; }
        recheck(ctx, plan).await?;
        let got = trigger(up).await.map_err(|e| fail("VISION_UNREACHABLE", e))?;
        let Some((frame_id, markers, image_size)) = got else { continue; };
        if last.is_some_and(|l| frame_id <= l) { continue; }
        last = Some(frame_id);
        if point.discarded.len() < p.discard { point.discarded.push(frame_id); continue; }
        let (joints, still) = measured(ctx).await;
        let moved = arm.iter().any(|&i| (joints[i] - settled[i]).abs() > STILL_RAD);
        if moved || !still { return Err(fail("MOVED", "the arm moved during the capture")); }
        point.observations.push(Observation { frame_id, markers, joints, image_size, t_s: t0.elapsed().as_secs_f64() });
    }
    if point.observations.len() < p.frames {
        return Err(fail("VISION_TIMEOUT", format!("{} of {} fresh frames in {MAX_TRIGGERS} triggers", point.observations.len(), p.frames)));
    }
    Ok(())
}

fn goal(p: [f64; 3], rpy: [f64; 3]) -> GoalSpec { GoalSpec::Absolute { p, r: Some(crate::kinematics::rpy_to_r(rpy[0], rpy[1], rpy[2])) } }

fn precise() -> FinalApproach {
    FinalApproach { use_: "move_to_precise", request: crate::backlash::Request::Profile, rules: crate::backlash::DescentRules::tcp() }
}

/// Planning refusals before a point's motion: the point is skipped.
pub const PLAN_REFUSALS: [&str; 3] = ["IK_UNREACHABLE", "IK_BRANCH_FLIP", "WORKSPACE_VIOLATION"];

/// What a point ended with: skip (refused before moving) or end the run.
enum PointEnd { Skip(String, String), Abort(Stop) }

fn classify(e: (Stop, bool)) -> PointEnd {
    match e {
        (Stop::Fail(code, msg), false) if PLAN_REFUSALS.contains(&code.as_str()) => PointEnd::Skip(code, msg),
        (stop, _) => PointEnd::Abort(stop),
    }
}

async fn run_point(ctx: &MacroCtx, plan: &HoverPlan, point: &mut HoverPoint, up: &crate::api::proxy::DetectorUpstream, t0: Instant) -> Result<(), PointEnd> {
    let p = plan.job.params.clone();
    let approach = Speed { linear: p.approach_speed_mps, angular: plan.angular_speed_rps };
    let descent = Speed { linear: p.descent_speed_mps, angular: plan.angular_speed_rps };
    recheck(ctx, plan).await.map_err(PointEnd::Abort)?;
    if !ctx.is_executor() {
        ctx.sup.inner().write().await.set_action_phase(ctx.id, &format!("approach:{}", point.id), 0.0, None, None);
    }
    ctx.move_arm_staged("approach", goal(point.approach_m, point.rpy_rad), approach, None).await.map_err(classify)?;
    recheck(ctx, plan).await.map_err(PointEnd::Abort)?;
    if !ctx.is_executor() {
        ctx.sup.inner().write().await.set_action_phase(ctx.id, &format!("descend:{}", point.id), 0.0, None, None);
    }
    ctx.move_arm_staged("descend", goal(point.hover_m, point.rpy_rad), descent, Some(precise())).await.map_err(classify)?;
    if !ctx.is_executor() {
        ctx.sup.inner().write().await.set_action_phase(ctx.id, &format!("capture:{}", point.id), 0.0, None, None);
    }
    capture(ctx, plan, point, up, t0).await.map_err(PointEnd::Abort)?;
    recheck(ctx, plan).await.map_err(PointEnd::Abort)?;
    if !ctx.is_executor() {
        ctx.sup.inner().write().await.set_action_phase(ctx.id, &format!("retreat:{}", point.id), 0.0, None, None);
    }
    // the way back up: a refusal here leaves the tip near the paper, so it ends the run
    ctx.move_arm_staged("retreat", goal(point.approach_m, point.rpy_rad), descent, None).await.map_err(|(s, _)| PointEnd::Abort(s))?;
    Ok(())
}

async fn run(
    sup: Arc<ArmSupervisor>,
    id: u64,
    mut plan: HoverPlan,
    up: crate::api::proxy::DetectorUpstream,
    dir: PathBuf,
    stamp: crate::supervisor::AdmissionStamp,
    deadline: Instant,
) {
    use crate::rt_core::requests::actions::{FinishState, HeldUpdate};
    let ctx = MacroCtx {
        sup: sup.clone(), id, stamp,
        deadline,
        phase_timeout_s: plan.deadline_s,
        timeout_code: "HOVER_TIMEOUT",
        timeout_message: "the hovers did not finish within their time bound".into(),
    };
    let t0 = Instant::now();
    let mut job = plan.job.clone();
    let total = job.points.iter().filter(|p| p.state == "pending").count().max(1) as f32;
    let mut k = 0.0f32;
    let mut result: Result<(), Stop> = Ok(());
    for i in 0..job.points.len() {
        if job.points[i].state != "pending" { continue; }
        if !ctx.is_executor() {
            sup.inner().write().await.set_action_phase(id, &format!("point:{}", job.points[i].id), k / total, None, None);
        }
        let mut point = job.points[i].clone();
        let r = run_point(&ctx, &plan, &mut point, &up, t0).await;
        match r {
            Ok(()) => point.state = "done".into(),
            Err(PointEnd::Skip(code, msg)) => {
                tracing::info!("hover job {}: point {} skipped ({code}: {msg})", job.id, point.id);
                point.state = "skipped".into(); point.code = Some(code); point.message = Some(msg);
            }
            Err(PointEnd::Abort(stop)) => {
                point.state = "failed".into();
                if let Stop::Fail(code, msg) = &stop { point.code = Some(code.clone()); point.message = Some(msg.clone()); }
                job.points[i] = point;
                result = Err(stop);
                break;
            }
        }
        job.points[i] = point;
        k += 1.0;
        job.updated_at = now();
        plan.job = job.clone();
        update(&sup, &job).await;
        persist(&dir, &job).await;
    }
    // end the action and record how the run ended
    match &result {
        Ok(()) => {
            ctx.finish(FinishState::Done, None, None, None, None, HeldUpdate::Unchanged).await;
        }
        Err(Stop::Fail(code, message)) => {
            ctx.finish(FinishState::Failed, Some(code), Some(message.clone()), None, None, HeldUpdate::Unchanged).await;
        }
        Err(Stop::Cancelled) => {}
    }
    if !ctx.is_executor() {
        let mut inner = sup.inner().write().await;
        let action = inner.last_action.as_ref().filter(|a| a.id == id).cloned();
        job.state = match &result { Ok(()) => "done".into(), Err(Stop::Fail(..)) => "failed".into(), Err(Stop::Cancelled) => "cancelled".into() };
        job.code = action.as_ref().and_then(|a| a.code.as_deref().map(str::to_owned));
        job.message = action.as_ref().and_then(|a| a.message.clone());
        if let Some(p) = job.points.iter_mut().find(|p| p.state == "pending" && matches!(result, Err(Stop::Cancelled))) {
            // the run was cancelled while this point ran
            p.code = job.code.clone();
        }
        job.end = Some(arm_state(&inner));
        job.updated_at = now();
        put_job(&mut inner, job.clone());
    } else {
        let pub_snap = sup.hub().latest();
        let action = pub_snap
            .as_ref()
            .and_then(|s| s.obs.action.as_ref())
            .filter(|a| a.id == id);
        job.state = match &result { Ok(()) => "done".into(), Err(Stop::Fail(..)) => "failed".into(), Err(Stop::Cancelled) => "cancelled".into() };
        if let Err(Stop::Fail(code, msg)) = &result {
            job.code = Some(code.clone());
            job.message = Some(msg.clone());
        } else {
            job.code = action.and_then(|a| a.code.as_deref().map(str::to_owned));
            job.message = action.and_then(|a| a.message.clone());
        }
        if let Some(p) = job.points.iter_mut().find(|p| p.state == "pending" && matches!(result, Err(Stop::Cancelled))) {
            p.code = job.code.clone();
        }
        job.end = pub_snap.as_deref().map(arm_state_from_snapshot);
        job.updated_at = now();
        sup.put_hover_job(job.clone()).await;
    }
    tracing::info!("Calibration hover job {} run ended: {} {:?} ({} done, {} skipped)", job.id, job.state, job.code,
        job.points.iter().filter(|p| p.state == "done").count(), job.points.iter().filter(|p| p.state == "skipped").count());
    persist(&dir, &job).await;
}

/// Live arm state for a reconnecting client (from published snapshot on `OWNER_EXECUTOR`).
fn arm_state_from_snapshot(snap: &crate::rt_core::publish::Snapshot) -> Value {
    json!({
        "is_torqued": snap.obs.is_torqued,
        "safety_state": snap.obs.safety_state,
        "fault": snap.obs.fault,
        "watchdog_park": snap.obs.action.as_ref().is_some_and(|a| a.kind == "park" && a.is_running()),
        "pending_normal_unload": snap.rt.owner.pending_normal_unload,
        "lease_held": snap.rt.owner.authority.lease_generation != 0,
        "at": now(),
    })
}

/// Live arm state for a reconnecting client.
fn arm_state(inner: &SupervisorInner) -> Value {
    json!({"is_torqued": inner.is_torqued, "safety_state": inner.safety.state(), "fault": inner.safety.fault(),
        "watchdog_park": inner.watchdog_park.is_some(), "pending_normal_unload": inner.pending_normal_unload,
        "lease_held": inner.active_lease.is_some(), "at": now()})
}

fn err(status: StatusCode, code: &str, message: impl Into<String>) -> Response {
    (status, axum::Json(json!({"error": code, "code": code, "message": message.into()}))).into_response()
}

/// `GET /api/v1/calibration/hover`: `{jobs: [{id, state, code, created_at, updated_at, points, done}], arm}`.
pub async fn get_jobs(axum::extract::State(sup): axum::extract::State<Arc<ArmSupervisor>>) -> Response {
    let (all_jobs, arm) = if sup.owner_mode() == crate::supervisor::OWNER_EXECUTOR {
        let jobs = sup.hover_jobs_snapshot().await;
        let arm = sup
            .hub()
            .latest()
            .map_or(Value::Null, |s| arm_state_from_snapshot(&s));
        (jobs, arm)
    } else {
        let inner = sup.inner().read().await;
        (inner.hover_jobs.clone(), arm_state(&inner))
    };
    let jobs: Vec<Value> = all_jobs.iter().rev().map(|j| json!({"id": j.id, "state": j.state, "code": j.code, "created_at": j.created_at,
        "updated_at": j.updated_at, "candidate_fingerprint": j.candidate_fingerprint, "points": j.points.len(),
        "done": j.points.iter().filter(|p| p.state == "done").count()})).collect();
    axum::Json(json!({"jobs": jobs, "arm": arm})).into_response()
}

/// `GET /api/v1/calibration/hover/<id>`: the job (memory, else its file) with the live `arm` state.
pub async fn get_job(axum::extract::State(sup): axum::extract::State<Arc<ArmSupervisor>>,
    axum::Extension(deps): axum::Extension<crate::calibration::CalibrationDeps>, UrlPath(id): UrlPath<String>) -> Response {
    if !valid_job_id(&id) { return err(StatusCode::BAD_REQUEST, "INVALID", "invalid job id"); }
    let (job, arm) = if sup.owner_mode() == crate::supervisor::OWNER_EXECUTOR {
        let jobs = sup.hover_jobs_snapshot().await;
        let arm = sup
            .hub()
            .latest()
            .map_or(Value::Null, |s| arm_state_from_snapshot(&s));
        (jobs.into_iter().find(|j| j.id == id), arm)
    } else {
        let inner = sup.inner().read().await;
        (inner.hover_jobs.iter().find(|j| j.id == id).cloned(), arm_state(&inner))
    };
    let job = match job {
        Some(j) => serde_json::to_value(j).unwrap_or_default(),
        None => {
            let path = jobs_dir(&deps.path).join(format!("{id}.json"));
            match tokio::task::spawn_blocking(move || std::fs::read_to_string(path)).await {
                Ok(Ok(t)) => serde_json::from_str(&t).unwrap_or(Value::Null),
                _ => return err(StatusCode::NOT_FOUND, "NOT_FOUND", format!("no hover job {id}")),
            }
        }
    };
    let mut v = job;
    v["arm"] = arm;
    axum::Json(v).into_response()
}

/// Startup: put the stored jobs back (a job that was running when armd stopped is `failed`, code `ARMD_RESTART`).
pub async fn load_into(sup: &ArmSupervisor, calibration_path: &Path) {
    let dir = jobs_dir(calibration_path);
    let jobs = tokio::task::spawn_blocking(move || -> Vec<HoverJob> {
        let Ok(rd) = std::fs::read_dir(&dir) else { return Vec::new(); };
        let mut v: Vec<HoverJob> = rd.flatten().filter(|e| e.file_name().to_string_lossy().ends_with(".json"))
            .filter_map(|e| serde_json::from_str(&std::fs::read_to_string(e.path()).ok()?).ok()).collect();
        v.sort_by(|a, b| a.id.cmp(&b.id));
        v
    }).await.unwrap_or_default();
    let mut inner = sup.inner().write().await;
    for mut j in jobs {
        if j.state == "running" { j.state = "failed".into(); j.code = Some("ARMD_RESTART".into()); j.message = Some("armd restarted during the run".into()); }
        put_job(&mut inner, j);
    }
}
