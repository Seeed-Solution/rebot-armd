//! Runtime gripper calibration (`/api/v1/gripper/calibration`), persisted to `ARMD_GRIPPER_CALIB`.

use std::path::{Path, PathBuf};
use serde::{Deserialize, Serialize};

use crate::model::{GripperPolicy, JawOpening, JointLimit};
use crate::motion::MotionError;
use crate::persist::{self, Loaded, PendingWrite};
use crate::supervisor::{ArmError, ArmSupervisor, SupervisorInner};

pub const DEFAULT_GRIPPER_CALIB_PATH: &str = "/userdata/rebot-teleop/gripper_calib.json";

/// `ARMD_GRIPPER_CALIB` or [`DEFAULT_GRIPPER_CALIB_PATH`].
pub fn path_from_env() -> PathBuf {
    persist::path_from_env("ARMD_GRIPPER_CALIB", DEFAULT_GRIPPER_CALIB_PATH)
}

/// Persisted file format.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct GripperCalibFile {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub offset_rad: Option<f32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub jaw_opening: Option<JawOpening>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ts: Option<u64>,
    /// Set by a zero of the gripper joint: the calibration was taught with the old zero.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stale: Option<crate::calibration::CalibrationStale>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tool_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tool_config_digest: Option<String>,
}

impl GripperCalibFile {
    pub fn validate(&self, j7_limit: Option<&JointLimit>) -> Result<(), String> {
        if let Some(off) = self.offset_rad {
            if !off.is_finite() || off.abs() > 0.5 {
                return Err(format!("offset_rad {off} exceeds limit of 0.5 rad"));
            }
        }
        if let Some(ref jo) = self.jaw_opening {
            jo.validate(j7_limit)?;
        }
        Ok(())
    }
}

/// REST response for `GET /api/v1/gripper/calibration`.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct GripperCalibrationResponse {
    pub offset_rad: Option<f32>,
    pub jaw_opening: Option<JawOpening>,
    pub calibrated: bool,
    pub policy: Option<GripperPolicy>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ts: Option<u64>,
    /// The calibration was taught with an older gripper zero (null when current).
    #[serde(default)]
    pub stale: Option<crate::calibration::CalibrationStale>,
    pub revision: u64,
}

/// Body for WS `/api/v1/gripper/calibration`.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CalibBody {
    pub jaw_opening: JawOpening,
    #[serde(default)]
    pub offset_rad: Option<f32>,
    #[serde(default)]
    pub expected_revision: Option<u64>,
}

/// Validated gripper calibration update prepared without mutating `SupervisorInner`.
#[derive(Debug, Clone)]
pub struct PreparedGripperCalib {
    pub base_revision: u64,
    pub offset_rad: Option<f32>,
    pub jaw_opening: JawOpening,
    pub candidate: Option<crate::supervisor::GripperCalibrationCandidate>,
    pub effective_policy: GripperPolicy,
    pub write: Option<PendingWrite>,
}

/// Step 1 of atomic persist-before-commit: validate payload, expected_revision, and
/// driver limits without leaving `SupervisorInner` mutated.
pub fn prepare_update(inner: &mut SupervisorInner, body: &serde_json::Value) -> Result<PreparedGripperCalib, ArmError> {
    prepare_update_with(inner, body, false)
}

/// Same as [`prepare_update`], optionally requiring `expected_revision` (enforced on real hardware).
pub fn prepare_update_with(
    inner: &mut SupervisorInner,
    body: &serde_json::Value,
    require_expected_revision: bool,
) -> Result<PreparedGripperCalib, ArmError> {
    let b: CalibBody = serde_json::from_value(body.clone()).map_err(|e| {
        MotionError::InvalidCommand(format!(
            "gripper calibration body must have jaw_opening and optional offset_rad: {e}"
        ))
    })?;

    if require_expected_revision && b.expected_revision.is_none() {
        return Err(MotionError::InvalidCommand("expected_revision is required when operator auth is enforced".into()).into());
    }
    if let Some(exp) = b.expected_revision {
        if exp != inner.gripper_calib_revision {
            return Err(ArmError::PolicyRevisionMismatch { expected: exp, current: inner.gripper_calib_revision });
        }
    }

    if let Some(off) = b.offset_rad {
        if !off.is_finite() || off.abs() > 0.5 {
            return Err(MotionError::InvalidCommand(format!(
                "offset_rad {off} must be in [-0.5, 0.5] rad"
            )).into());
        }
    }

    let j7_limit = inner.config.gripper_index().and_then(|idx| inner.config.limits.get(idx));
    b.jaw_opening.validate(j7_limit).map_err(MotionError::InvalidCommand)?;

    if inner.config.gripper_policy.is_none() {
        return Err(MotionError::Unsupported("Gripper calibration requires a gripper policy".into()).into());
    }

    let offset = b.offset_rad.or(inner.gripper_calib_offset);
    let (candidate, effective_policy) = inner
        .build_gripper_calibration_candidate(offset, Some(b.jaw_opening.clone()))
        .map_err(MotionError::InvalidCommand)?;

    let file = GripperCalibFile {
        offset_rad: offset,
        jaw_opening: Some(b.jaw_opening.clone()),
        ts: Some(std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs())),
        stale: None,
        tool_id: Some(inner.config.active_tool.id.clone()),
        tool_config_digest: Some(inner.config.active_tool.config_digest.clone()),
    };

    let write = inner.gripper_calib_path.clone().or_else(|| Some(path_from_env())).map(|path| {
        PendingWrite::new(path, serde_json::to_string_pretty(&file).unwrap_or_default())
    });

    Ok(PreparedGripperCalib {
        base_revision: inner.gripper_calib_revision,
        offset_rad: offset,
        jaw_opening: b.jaw_opening,
        candidate: Some(candidate),
        effective_policy,
        write,
    })
}

/// Step 2.5 of atomic persist-before-commit (under supervisor lock, before rename):
/// re-verify revision, busy flags, and hardware driver limits and refresh the candidate
/// so post-rename commit is infallible.
pub fn precheck_update(inner: &mut SupervisorInner, prepared: &mut PreparedGripperCalib) -> Result<(), ArmError> {
    if inner.gripper_calib_revision != prepared.base_revision {
        return Err(ArmError::PolicyRevisionMismatch {
            expected: prepared.base_revision,
            current: inner.gripper_calib_revision,
        });
    }
    if inner.calibration_busy || inner.zero_check_persisting.is_some() {
        return Err(ArmError::Rejected {
            status: 409,
            code: "CONFIG_BUSY".into(),
            message: "another calibration operation is running".into(),
        });
    }
    if inner.config.gripper_policy.is_none() {
        return Err(MotionError::Unsupported("Gripper calibration requires a gripper policy".into()).into());
    }
    let (candidate, effective_policy) = inner
        .build_gripper_calibration_candidate(prepared.offset_rad, Some(prepared.jaw_opening.clone()))
        .map_err(MotionError::InvalidCommand)?;
    prepared.candidate = Some(candidate);
    prepared.effective_policy = effective_policy;
    Ok(())
}

/// Step 3 of atomic persist-before-commit: re-verify revision and infallibly commit
/// the pre-validated candidate built before rename.
pub fn commit_update(inner: &mut SupervisorInner, mut prepared: PreparedGripperCalib) -> Result<(GripperPolicy, u64), ArmError> {
    if inner.gripper_calib_revision != prepared.base_revision {
        return Err(ArmError::PolicyRevisionMismatch { expected: prepared.base_revision, current: inner.gripper_calib_revision });
    }
    let cand = match prepared.candidate.take() {
        Some(c) => c,
        None => {
            let (c, p) = inner
                .build_gripper_calibration_candidate(prepared.offset_rad, Some(prepared.jaw_opening.clone()))
                .map_err(MotionError::InvalidCommand)?;
            prepared.effective_policy = p;
            c
        }
    };
    inner.commit_gripper_calibration_candidate(cand);
    // the new calibration is taught with the current gripper zero
    inner.gripper_calib_stale = None;
    inner.gripper_calib_revision += 1;
    Ok((prepared.effective_policy, inner.gripper_calib_revision))
}

/// WS command `/api/v1/gripper/calibration` `{"jaw_opening": JawOpening, "offset_rad": f32?}`.
/// Validates jaw_opening (monotonic, >= 2 points, inside J7 limits, openings in [0, 0.2]) and |offset_rad| <= 0.5.
/// Returns the effective policy and the file write to run after the lock is released.
pub fn update(inner: &mut SupervisorInner, body: &serde_json::Value) -> Result<(GripperPolicy, Option<PendingWrite>), ArmError> {
    let mut prepared = prepare_update(inner, body)?;
    let write = prepared.write.take();
    let (effective, _) = commit_update(inner, prepared)?;
    Ok((effective, write))
}

/// Learned free-motion torque baseline (`crate::gripper_baseline`): `ARMD_GRIPPER_BASELINE`, else
/// `gripper_baseline.<follower>.json` next to the gripper calibration file, so a sim or mock run
/// never writes the real arm's file. A separate file: it changes after gripper moves, the
/// calibration only on a calibration write. None without a follower (not persisted).
pub fn baseline_path_for(calib_path: &Path, follower: Option<&str>) -> Option<PathBuf> {
    let follower = follower.filter(|f| !f.is_empty() && f.chars().all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_'))?;
    Some(std::env::var_os("ARMD_GRIPPER_BASELINE").map(PathBuf::from)
        .unwrap_or_else(|| calib_path.with_file_name(format!("gripper_baseline.{follower}.json"))))
}

/// How often [`spawn_baseline_persister`] learns queued moves and writes a changed baseline.
pub const BASELINE_PERSIST_INTERVAL: std::time::Duration = std::time::Duration::from_secs(1);

/// Lowest first value of a bin's baseline: the free-motion torque the absolute contact rule
/// leaves room for (`contact_torque_nm` − margin floor); 0.45 N·m without contact_torque_nm.
fn initial_max_nm(inner: &SupervisorInner) -> f32 {
    inner.config.gripper_policy.as_ref().and_then(|p| p.contact_torque_nm)
        .map_or(0.45, |c| c - crate::gripper_baseline::MARGIN_FLOOR_NM)
}

/// Queued moves and the baseline they apply to, taken under the lock: (moves, baseline,
/// generation, initial_max_nm).
type LearnJob = (Vec<crate::gripper_baseline::MoveRecord>, crate::gripper_baseline::FreeTorqueBaseline, u64, f32);

fn take_learn_job(inner: &mut SupervisorInner) -> Option<LearnJob> {
    if inner.gripper_baseline_pending.is_empty() { return None; }
    let moves = std::mem::replace(&mut inner.gripper_baseline_pending,
        Vec::with_capacity(crate::supervisor::GRIPPER_BASELINE_PENDING_MAX));
    let base = inner.gripper_baseline_for_motion()?;
    Some((moves, base, inner.gripper_baseline_generation, initial_max_nm(inner)))
}

/// Learn the moves into the baseline (no lock held). True when anything changed.
fn run_learn_job(job: &mut LearnJob) -> bool {
    let (moves, base, _, initial) = job;
    let mut changed = false;
    for rec in moves.iter() {
        let out = base.learn(rec, *initial);
        if out.bins_updated > 0 || out.bins_rejected > 0 {
            changed = true;
            tracing::debug!("Gripper baseline: {:?} move {:.2} -> {:.2} rad: {} bins updated, {} rejected ({})",
                rec.direction, rec.start_rad, rec.end_rad, out.bins_updated, out.bins_rejected, base.state());
        }
    }
    if changed { base.ts = Some(std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs())); }
    changed
}

/// Store a learned baseline unless a reset (or another store) came in between.
fn store_learn_job(inner: &mut SupervisorInner, job: LearnJob) {
    let (_, base, generation, _) = job;
    if inner.gripper_baseline_generation != generation { return; }
    inner.gripper_baseline = Some(base);
    inner.gripper_baseline_generation += 1;
    inner.gripper_baseline_dirty = true;
}

/// Learn the queued moves now, under the caller's lock (tests and tools; armd itself learns
/// in [`spawn_baseline_persister`], off the control tick).
pub fn learn_pending_now(inner: &mut SupervisorInner) {
    if let Some(mut job) = take_learn_job(inner) {
        if run_learn_job(&mut job) { store_learn_job(inner, job); }
    }
}

/// The baseline to write when it changed since the last write (clears the flag): (path,
/// snapshot). Serialize the snapshot after releasing the lock.
pub fn take_baseline_snapshot(inner: &mut SupervisorInner) -> Option<(PathBuf, crate::gripper_baseline::FreeTorqueBaseline)> {
    if !inner.gripper_baseline_dirty { return None; }
    let path = inner.gripper_baseline_path.clone()?;
    let b = inner.gripper_baseline.clone()?;
    inner.gripper_baseline_dirty = false;
    Some((path, b))
}

/// Every [`BASELINE_PERSIST_INTERVAL`]: learn queued gripper moves and write a changed baseline.
/// The lock is held only to take the moves and a snapshot and to store the result; learning and
/// JSON serialization run without it.
pub fn spawn_baseline_persister(supervisor: std::sync::Arc<ArmSupervisor>) {
    tokio::spawn(async move {
        let mut tick = tokio::time::interval(BASELINE_PERSIST_INTERVAL);
        loop {
            tick.tick().await;
            let job = take_learn_job(&mut *supervisor.inner().write().await);
            if let Some(mut job) = job {
                if run_learn_job(&mut job) { store_learn_job(&mut *supervisor.inner().write().await, job); }
            }
            let snapshot = take_baseline_snapshot(&mut *supervisor.inner().write().await);
            let Some((path, b)) = snapshot else { continue };
            let text = match serde_json::to_string_pretty(&b) { Ok(t) => t, Err(_) => continue };
            if let Err(e) = PendingWrite::new(path.clone(), text).run().await {
                tracing::warn!("Failed to write gripper baseline {}: {e}", path.display());
                supervisor.inner().write().await.gripper_baseline_dirty = true;
            }
        }
    });
}

/// Load the persisted baseline. It applies only to the tool (id and config digest) and the
/// follower it was learned with; another one's or an invalid file is ignored (unlearned, and the
/// next write replaces the file).
fn load_baseline(inner: &mut SupervisorInner, path: &Path) {
    inner.gripper_baseline_path = Some(path.to_path_buf());
    match persist::load::<crate::gripper_baseline::FreeTorqueBaseline>(path, |b| b.validate()) {
        Loaded::Loaded(b) => {
            let tool = &inner.config.active_tool;
            if !b.belongs_to(&tool.id, &tool.config_digest, inner.follower.as_deref()) {
                tracing::warn!("Ignoring gripper baseline from {} (tool {:?}/{:?}, follower {:?}; active {}/{}, {:?})",
                    path.display(), b.tool_id, b.tool_config_digest, b.follower, tool.id, tool.config_digest, inner.follower);
                return;
            }
            tracing::info!("Gripper baseline loaded from {} ({}, {} close / {} open moves)", path.display(), b.state(), b.close_moves, b.open_moves);
            inner.gripper_baseline = Some(b);
        }
        Loaded::Missing => {}
        Loaded::Invalid(e) => tracing::warn!("Ignoring gripper baseline file: {e}; starting unlearned"),
    }
}

/// Load the persisted gripper calibration at startup (logs outcome).
/// Invalid/corrupt file -> log a warning, ignore it, report observation.gripper_zero.state "calib_invalid".
pub async fn load_into(supervisor: &ArmSupervisor, path: &Path) {
    let mut inner = supervisor.inner().write().await;
    inner.gripper_calib_path = Some(path.to_path_buf());
    if let Some(bp) = baseline_path_for(path, inner.follower.as_deref()) { load_baseline(&mut inner, &bp); }
    let j7_lim = inner.config.gripper_index().and_then(|idx| inner.config.limits.get(idx)).cloned();
    match persist::load::<GripperCalibFile>(path, |f| f.validate(j7_lim.as_ref())) {
        Loaded::Loaded(calib) => {
            if !inner.config.active_tool.matches_record(calib.tool_id.as_deref(), calib.tool_config_digest.as_deref()) {
                tracing::warn!(
                    "Ignoring gripper calibration from {} (record tool {:?}/{:?} does not match active tool {}/{})",
                    path.display(),
                    calib.tool_id,
                    calib.tool_config_digest,
                    inner.config.active_tool.id,
                    inner.config.active_tool.config_digest
                );
                return;
            }
            tracing::info!("Gripper calibration loaded from {}", path.display());
            if let Err(e) = inner.apply_gripper_calibration(calib.offset_rad, calib.jaw_opening) {
                tracing::warn!("Ignoring invalid gripper calibration: {e}");
                inner.gripper_calib_invalid = true;
            } else if let Some(st) = calib.stale {
                tracing::warn!("Gripper calibration is stale: {}", st.reason);
                inner.gripper_calib_stale = Some(st);
            }
        }
        Loaded::Missing => {}
        Loaded::Invalid(e) => {
            tracing::warn!("Ignoring gripper calibration file: {e}; using uncalibrated default");
            inner.gripper_calib_invalid = true;
        }
    }
}
