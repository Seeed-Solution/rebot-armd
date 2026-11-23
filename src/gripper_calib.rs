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

/// First-value limit of a calibration stroke: CALIBRATION_MAX_TORQUE_FRACTION of the cap.
fn calibration_initial_max_nm(inner: &SupervisorInner) -> f32 {
    inner.config.gripper_policy.as_ref().and_then(|p| p.max_close_torque_nm)
        .map_or(CALIBRATION_MAX_TORQUE_NM, |c| CALIBRATION_MAX_TORQUE_FRACTION * c)
}

/// Queued moves and the baseline they apply to, taken under the lock: (moves, baseline,
/// generation, (initial_max_nm, calibration initial_max_nm)).
type LearnJob = (Vec<crate::gripper_baseline::MoveRecord>, crate::gripper_baseline::FreeTorqueBaseline, u64, (f32, f32));

fn take_learn_job(inner: &mut SupervisorInner) -> Option<LearnJob> {
    if inner.gripper_baseline_pending.is_empty() { return None; }
    let moves = std::mem::replace(&mut inner.gripper_baseline_pending,
        Vec::with_capacity(crate::supervisor::GRIPPER_BASELINE_PENDING_MAX));
    let base = inner.gripper_baseline_for_motion()?;
    Some((moves, base, inner.gripper_baseline_generation, (initial_max_nm(inner), calibration_initial_max_nm(inner))))
}

/// Learn the moves into the baseline (no lock held). True when anything changed.
fn run_learn_job(job: &mut LearnJob) -> bool {
    let (moves, base, _, initial) = job;
    let mut changed = false;
    for rec in moves.iter() {
        let out = base.learn(rec, if rec.calibration { initial.1 } else { initial.0 });
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

// ---- gripper_calibrate: one action, strokes with nothing in the jaws ----

/// WS command path of the empty-gripper calibration action.
pub const CALIBRATE_PATH: &str = "/api/v1/action/gripper_calibrate";
/// A stroke must end within this distance of its target (rad). The strokes stay off both
/// mechanical stops (5 % and 95 % open; the real gripper opens to about -5.61 against a -5.70
/// command), so a free jaw reaches them.
pub const CALIBRATION_TARGET_TOL_RAD: f32 = crate::motion::CALIBRATION_END_TOL_RAD;
/// Calibration strokes run between these opening fractions.
pub const CALIBRATION_CLOSED_FRACTION: f64 = 0.05;
pub const CALIBRATION_OPEN_FRACTION: f64 = 0.95;
/// Free motion: away from the start and the end (LEARN_*_GUARD_RAD), at most this share of a
/// stroke's samples may move slower than CALIBRATION_MIN_SPEED_FRACTION of the stroke speed.
pub const CALIBRATION_SLOW_SHARE_MAX: f32 = 0.1;
pub const CALIBRATION_MIN_SPEED_FRACTION: f32 = 0.5;
/// |τ| p95 of a stroke above this fraction of `max_close_torque_nm` is not free motion.
pub const CALIBRATION_MAX_TORQUE_FRACTION: f32 = 0.85;
/// Without a torque cap: |τ| p95 limit of a stroke (N·m).
pub const CALIBRATION_MAX_TORQUE_NM: f32 = 0.85;

/// One calibration stroke: move to opening `fraction` at `speed` (rad/s).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stroke { pub fraction: f64, pub speed: f64 }

/// The strokes: close to 5 %, then 4 x (open to 95 % at 1.0 rad/s, close to 5 %), 2 x (open at
/// 0.3 rad/s, close to 5 %), and a final open at 1.0 rad/s. Every close runs at `close_speed`
/// (`gripper_policy.close_speed_rad_s`, 0.3 rad/s by default), the first one included: the fast
/// closing band is not learned (closes at speed fall back on the slow band).
pub fn strokes(close_speed: f64) -> Vec<Stroke> {
    let close = Stroke { fraction: CALIBRATION_CLOSED_FRACTION, speed: close_speed };
    let open = |speed| Stroke { fraction: CALIBRATION_OPEN_FRACTION, speed };
    let mut v = vec![close];
    for _ in 0..4 { v.push(open(1.0)); v.push(close); }
    for _ in 0..2 { v.push(open(0.3)); v.push(close); }
    v.push(open(1.0));
    v
}

/// Start `gripper_calibrate`: one action whose orchestrator runs the [`strokes`] as gripper
/// sub-motions; stop, E-stop and lease loss end it like any macro action. Each stroke must reach
/// its target with no contact, stall or torque cap trip, |τ| p95 below
/// CALIBRATION_MAX_TORQUE_FRACTION x cap, and every bin within the profile of the earlier
/// strokes; otherwise the action fails CALIBRATION_OBSTRUCTED and the baseline stays as it was.
/// When all strokes pass, the baseline is replaced by the one learned from them.
pub fn begin_calibration(inner: &mut SupervisorInner, sup: &std::sync::Arc<ArmSupervisor>, epoch: u64, tag: Option<String>) -> Result<(u64, f64), ArmError> {
    let (Some(_), Some(policy)) = (inner.config.gripper_index(), inner.config.gripper_policy.clone()) else {
        return Err(MotionError::Unsupported("Gripper calibration needs a profile with a gripper joint and policy".into()).into());
    };
    let torque_feedback = inner.driver.as_ref().and_then(|d| d.lock().ok().map(|d| d.capabilities().torque_feedback)).unwrap_or(true);
    if !torque_feedback {
        return Err(MotionError::Unsupported("Gripper calibration needs torque feedback".into()).into());
    }
    let id = inner.begin_orchestrated_action("gripper_calibrate", tag, epoch)?;
    let list = strokes(policy.close_speed());
    inner.gripper_calibration = Some(crate::supervisor::GripperCalibrationRun { id, records: Vec::with_capacity(list.len() + 2) });
    let span = f64::from((policy.open_rad - policy.closed_rad).abs());
    let expected: f64 = list.iter().map(|s| span * 0.95 / s.speed + 1.5).sum();
    let sup2 = sup.clone();
    tokio::spawn(async move {
        let worker = { let sup = sup2.clone(); tokio::spawn(async move { run_calibration(sup, id).await }) };
        if let Err(e) = worker.await {
            let mut inner = sup2.inner().write().await;
            inner.gripper_calibration = None;
            inner.finish_macro(id, "failed", Some("INTERNAL"), Some(format!("calibration orchestrator failed: {e}")), None, None);
        }
    });
    Ok((id, expected))
}

/// Start `gripper_calibrate` without a WS lease check (tests, in-process agents).
pub async fn start_calibration(sup: &std::sync::Arc<ArmSupervisor>) -> Result<(u64, f64), ArmError> {
    let mut inner = sup.inner().write().await;
    let epoch = inner.safety.epoch_id();
    begin_calibration(&mut inner, sup, epoch, None)
}

async fn run_calibration(sup: std::sync::Arc<ArmSupervisor>, id: u64) {
    use crate::macro_action::{MacroCtx, Stop};
    let ctx = MacroCtx {
        sup: sup.clone(), id,
        deadline: std::time::Instant::now() + std::time::Duration::from_secs(400),
        phase_timeout_s: 40.0,
        timeout_code: "CALIBRATION_TIMEOUT",
        timeout_message: "the gripper calibration did not finish within its time bound".into(),
    };
    let result = calibration_strokes(&ctx).await;
    let mut inner = sup.inner().write().await;
    inner.gripper_calibration = None;
    match result {
        Ok(base) => {
            if !inner.macro_alive(id) { return; }
            inner.gripper_baseline = Some(base);
            inner.gripper_baseline_pending.clear();
            inner.gripper_baseline_generation += 1;
            inner.gripper_baseline_dirty = true;
            inner.gripper_last_contact = None;
            tracing::info!("Gripper calibration {id}: baseline learned");
            inner.finish_macro(id, "done", None, None, None, None);
        }
        Err(Stop::Fail(code, message)) => {
            tracing::warn!("Gripper calibration {id} failed: {code}: {message}");
            inner.finish_macro(id, "failed", Some(&code), Some(message), None, None);
        }
        Err(Stop::Cancelled) => {}
    }
}

/// |τ| quantile `q` of a stroke's samples (0 without samples).
fn torque_quantile(rec: &crate::gripper_baseline::MoveRecord, q: f32) -> f32 {
    let mut v: Vec<f32> = rec.samples.iter().map(|s| s.1).collect();
    if v.is_empty() { return 0.0; }
    v.sort_by(|a, b| a.total_cmp(b));
    v[((v.len() - 1) as f32 * q).round() as usize]
}

async fn calibration_strokes(ctx: &crate::macro_action::MacroCtx) -> Result<crate::gripper_baseline::FreeTorqueBaseline, crate::macro_action::Stop> {
    use crate::macro_action::{arm_fail, fail};
    let obstructed = |m: String| fail("CALIBRATION_OBSTRUCTED", m);
    let (policy, g, mut scratch, limit_nm) = {
        let inner = ctx.sup.inner().read().await;
        let policy = inner.config.gripper_policy.clone().ok_or_else(|| fail("UNSUPPORTED", "no gripper policy"))?;
        let g = inner.config.gripper_index().ok_or_else(|| fail("UNSUPPORTED", "no gripper joint"))?;
        let tool = &inner.config.active_tool;
        let scratch = crate::gripper_baseline::FreeTorqueBaseline::new(policy.closed_rad, policy.open_rad,
            Some(tool.id.clone()), Some(tool.config_digest.clone()), inner.follower.clone());
        let limit = policy.max_close_torque_nm.map_or(CALIBRATION_MAX_TORQUE_NM, |c| CALIBRATION_MAX_TORQUE_FRACTION * c);
        (policy, g, scratch, limit)
    };
    let list = strokes(policy.close_speed());
    for (i, st) in list.iter().enumerate() {
        let n = i + 1;
        let phase = format!("stroke:{n}/{}", list.len());
        ctx.phase(&phase, i as f32 / list.len() as f32).await?;
        let request = crate::motion::GripperRequest { command: crate::motion::GripperCommand::Move, fraction: Some(st.fraction), grasp: false, speed: st.speed, timeout_s: None, tag: None };
        let target = policy.closed_rad + st.fraction as f32 * (policy.open_rad - policy.closed_rad);
        let (limit_s, before) = {
            let mut inner = ctx.sup.inner().write().await;
            if !inner.macro_alive(ctx.id) { return Err(crate::macro_action::Stop::Cancelled); }
            let before = inner.gripper_calibration.as_ref().map_or(0, |c| c.records.len());
            let l = inner.start_macro_gripper_learning(ctx.id, request, crate::motion::LearnMode::Calibration).map_err(arm_fail)?;
            (l, before)
        };
        ctx.wait_sub(&phase, limit_s + 10.0).await?;
        let (at, rec) = {
            let inner = ctx.sup.inner().read().await;
            let at = inner.measured_positions.get(g).copied().unwrap_or(f32::NAN);
            (at, inner.gripper_calibration.as_ref().and_then(|c| c.records.get(before).cloned()))
        };
        if !(at - target).abs().le(&CALIBRATION_TARGET_TOL_RAD) {
            return Err(obstructed(format!("stroke {n}: the jaw stopped at {at:.3} rad, {:.3} rad short of {target:.3} rad; check that nothing is in or against the jaws", (at - target).abs())));
        }
        let Some(rec) = rec else {
            return Err(obstructed(format!("stroke {n}: a stall or torque cap trip on the way; check that nothing is in or against the jaws")));
        };
        // free motion: the jaw kept up with the stroke speed away from its ends
        let sign = if rec.end_rad >= rec.start_rad { 1.0 } else { -1.0 };
        let (mut mid, mut slow) = (0usize, 0usize);
        for &(q, _, v) in rec.samples.iter() {
            let (from_start, to_end) = ((q - rec.start_rad) * sign, (rec.end_rad - q) * sign);
            if from_start < crate::gripper_baseline::LEARN_START_GUARD_RAD || to_end < crate::gripper_baseline::LEARN_END_GUARD_RAD { continue; }
            mid += 1;
            if v < CALIBRATION_MIN_SPEED_FRACTION * st.speed as f32 { slow += 1; }
        }
        if mid > 0 && slow as f32 > CALIBRATION_SLOW_SHARE_MAX * mid as f32 {
            return Err(obstructed(format!("stroke {n}: the jaw moved slower than {:.2} rad/s for {slow} of {mid} samples", CALIBRATION_MIN_SPEED_FRACTION * st.speed as f32)));
        }
        let p95 = torque_quantile(&rec, 0.95);
        if p95 > limit_nm {
            return Err(obstructed(format!("stroke {n}: |torque| p95 {p95:.2} N·m above {limit_nm:.2} N·m")));
        }
        let out = scratch.learn(&rec, limit_nm);
        if out.bins_rejected > 0 {
            return Err(obstructed(format!("stroke {n}: torque above the free-motion profile of the earlier strokes in {} position bins", out.bins_rejected)));
        }
    }
    scratch.ts = Some(std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs()));
    Ok(scratch)
}
