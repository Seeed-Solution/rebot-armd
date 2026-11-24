//! Sequences (teach v1): ordered steps over stored poses and episodes, `sequences.json`.
//! References are by id (poses: [`crate::poses::NamedPose::id`], episodes: the episode id);
//! a referenced pose or episode cannot be deleted (IN_USE).
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

use crate::macro_action::{arm_fail, fail, MacroCtx, Stop};
use crate::persist::{self, Loaded, PendingWrite};
use crate::supervisor::{ArmError, ArmSupervisor, SupervisorInner};

pub const DEFAULT_SEQUENCES_PATH: &str = "/userdata/rebot-teleop/sequences.json";
/// Most steps in one sequence.
pub const MAX_STEPS: usize = 200;
/// Most sequences in the store.
pub const MAX_SEQUENCES: usize = 200;
/// Longest wait step (s).
pub const MAX_WAIT_S: f64 = 60.0;
/// Most repeats of one run.
pub const MAX_REPEAT: u32 = 100;

/// `ARMD_SEQUENCES` or [`DEFAULT_SEQUENCES_PATH`].
pub fn path_from_env() -> PathBuf { persist::path_from_env("ARMD_SEQUENCES", DEFAULT_SEQUENCES_PATH) }

/// Gripper during a pose step: `keep` its command (default, as `move_to_pose`) or move it to
/// the pose's value.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "lowercase")]
pub enum PoseGripper { #[default] Keep, Pose }

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum GripperAction { Open, Close, Move }

fn one() -> f64 { 1.0 }
fn half() -> f64 { crate::replay::DEFAULT_REPLAY_SPEED }
fn yes() -> bool { true }

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum Step {
    Pose {
        pose_id: String,
        #[serde(default = "one")]
        speed_scale: f64,
        #[serde(default)]
        gripper: PoseGripper,
    },
    Gripper {
        action: GripperAction,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        fraction: Option<f64>,
    },
    Wait { s: f64 },
    Replay {
        episode_id: String,
        #[serde(default = "half")]
        speed: f64,
        #[serde(default = "yes")]
        gripper: bool,
    },
}

impl Step {
    pub fn kind(&self) -> &'static str {
        match self { Self::Pose { .. } => "pose", Self::Gripper { .. } => "gripper", Self::Wait { .. } => "wait", Self::Replay { .. } => "replay" }
    }

    /// Steps that move the arm (rechecks before each one).
    pub fn is_motion(&self) -> bool { !matches!(self, Self::Wait { .. }) }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Sequence {
    pub id: String,
    pub name: String,
    #[serde(default)]
    pub description: String,
    pub steps: Vec<Step>,
    pub created_at: String,
    pub updated_at: String,
    /// Incremented by every save; a save naming another `rev` is SEQUENCE_CONFLICT.
    pub rev: u64,
}

impl Sequence {
    pub fn pose_ids(&self) -> impl Iterator<Item = &str> {
        self.steps.iter().filter_map(|s| match s { Step::Pose { pose_id, .. } => Some(pose_id.as_str()), _ => None })
    }

    pub fn episode_ids(&self) -> impl Iterator<Item = &str> {
        self.steps.iter().filter_map(|s| match s { Step::Replay { episode_id, .. } => Some(episode_id.as_str()), _ => None })
    }
}

/// `sequences.json` and `GET /api/v1/sequences` body.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct SequenceFile {
    #[serde(default)]
    pub sequences: Vec<Sequence>,
}

/// Pose and episode ids pinned by the running sequence (they count as referenced).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct RunPins {
    pub sequence_id: String,
    pub action_id: u64,
    pub pose_ids: Vec<String>,
    pub episode_ids: Vec<String>,
}

#[derive(Debug, Clone, Default)]
pub struct SequenceStore {
    pub file: SequenceFile,
    /// File the store is written to; None = not persisted.
    pub path: Option<PathBuf>,
    /// The running sequence's references.
    pub pins: Option<RunPins>,
    /// Store revision: incremented by every committed save / delete (`expected_revision`).
    pub revision: u64,
}

impl SequenceStore {
    pub fn get(&self, id: &str) -> Option<&Sequence> { self.file.sequences.iter().find(|s| s.id == id) }

    /// Ids of the sequences that reference pose `id` (and the running one when it pinned it).
    pub fn using_pose(&self, id: &str) -> Vec<String> {
        let mut out: Vec<String> = self.file.sequences.iter().filter(|s| s.pose_ids().any(|p| p == id)).map(|s| s.id.clone()).collect();
        if let Some(p) = self.pins.as_ref().filter(|p| p.pose_ids.iter().any(|x| x == id)) {
            if !out.contains(&p.sequence_id) { out.push(p.sequence_id.clone()); }
        }
        out
    }

    /// Ids of the sequences that reference episode `id` (and the running one when it pinned it).
    pub fn using_episode(&self, id: &str) -> Vec<String> {
        let mut out: Vec<String> = self.file.sequences.iter().filter(|s| s.episode_ids().any(|e| e == id)).map(|s| s.id.clone()).collect();
        if let Some(p) = self.pins.as_ref().filter(|p| p.episode_ids.iter().any(|x| x == id)) {
            if !out.contains(&p.sequence_id) { out.push(p.sequence_id.clone()); }
        }
        out
    }
}

/// Structural checks of a loaded file (ids, names, counts); references are checked on save
/// and run, so a deleted file of poses does not drop the sequences.
pub fn validate_file(file: &SequenceFile) -> Result<(), String> {
    if file.sequences.len() > MAX_SEQUENCES { return Err(format!("more than {MAX_SEQUENCES} sequences")); }
    for (i, s) in file.sequences.iter().enumerate() {
        if !crate::recording::is_ulid(&s.id) { return Err(format!("sequence id '{}' is not a ULID", s.id)); }
        if file.sequences[..i].iter().any(|o| o.id == s.id || o.name == s.name) { return Err(format!("duplicate sequence '{}'", s.name)); }
        if !crate::recording::store::valid_display_name(&s.name) { return Err(format!("invalid sequence name '{}'", s.name)); }
        if s.steps.len() > MAX_STEPS { return Err(format!("sequence '{}' has more than {MAX_STEPS} steps", s.name)); }
    }
    Ok(())
}

impl SupervisorInner {
    /// Sequences that reference pose `id` (IN_USE on delete).
    pub fn sequences_using_pose(&self, id: &str) -> Vec<String> {
        if id.is_empty() { return Vec::new(); }
        self.sequences.using_pose(id)
    }

    /// Sequences that reference episode `id` (IN_USE on delete).
    pub fn sequences_using_episode(&self, id: &str) -> Vec<String> { self.sequences.using_episode(id) }

    pub fn sequences_write(&self) -> Option<PendingWrite> {
        self.sequences.path.clone().map(|path| PendingWrite::new(path, serde_json::to_string_pretty(&self.sequences.file).unwrap_or_default()))
    }
}

/// Load the persisted sequences at startup. An invalid file is moved aside and the store starts empty.
pub async fn load_into(supervisor: &ArmSupervisor, path: &Path) {
    let mut inner = supervisor.inner().write().await;
    inner.sequences.path = Some(path.to_path_buf());
    match persist::load::<SequenceFile>(path, validate_file) {
        Loaded::Loaded(file) => {
            tracing::info!("{} sequences loaded from {}", file.sequences.len(), path.display());
            inner.sequences.file = file;
        }
        Loaded::Missing => {}
        Loaded::Invalid(e) => tracing::warn!("Ignoring sequences file: {e}; starting with no sequences"),
    }
}

// ---- save / delete (WS, lease required) ----

fn rejected(status: u16, code: &str, message: impl Into<String>) -> ArmError {
    ArmError::Rejected { status, code: code.into(), message: message.into() }
}

fn invalid(message: impl Into<String>) -> ArmError { rejected(400, "INVALID_COMMAND", message) }

fn now_rfc3339() -> String { chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Millis, true) }

/// Parse and check the steps of a save: shape, ranges, and that every referenced pose and
/// episode exists (SEQUENCE_REF_MISSING `{step}`).
pub fn check_steps(inner: &SupervisorInner, steps: &Value) -> Result<Vec<Step>, ArmError> {
    let arr = steps.as_array().ok_or_else(|| invalid("steps must be an array"))?;
    if arr.len() > MAX_STEPS { return Err(invalid(format!("at most {MAX_STEPS} steps"))); }
    let mut out = Vec::with_capacity(arr.len());
    for (i, v) in arr.iter().enumerate() {
        let step: Step = serde_json::from_value(v.clone()).map_err(|e| ArmError::detailed(400, "INVALID_COMMAND", format!("step {i}: {e}"), json!({"step": i})))?;
        let bad = |m: String| ArmError::detailed(400, "INVALID_COMMAND", format!("step {i}: {m}"), json!({"step": i}));
        let missing = |m: String| ArmError::detailed(422, "SEQUENCE_REF_MISSING", format!("step {i}: {m}"), json!({"step": i}));
        match &step {
            Step::Pose { pose_id, speed_scale, .. } => {
                if !(crate::poses::MIN_SPEED_SCALE..=1.0).contains(speed_scale) {
                    return Err(bad(format!("speed_scale must be in [{}, 1.0]", crate::poses::MIN_SPEED_SCALE)));
                }
                if inner.poses.by_id(pose_id).is_none() { return Err(missing(format!("no pose with id '{pose_id}'"))); }
            }
            Step::Gripper { action, fraction } => {
                if inner.config.gripper_index().is_none() || inner.config.gripper_policy.is_none() {
                    return Err(bad("this arm has no gripper policy".into()));
                }
                match (action, fraction) {
                    (GripperAction::Move, Some(f)) if (0.0..=1.0).contains(f) => {}
                    (GripperAction::Move, _) => return Err(bad("move needs fraction in [0, 1]".into())),
                    (_, Some(_)) => return Err(bad("fraction is only for move".into())),
                    _ => {}
                }
            }
            Step::Wait { s } => {
                if !s.is_finite() || !(0.0..=MAX_WAIT_S).contains(s) { return Err(bad(format!("s must be in [0, {MAX_WAIT_S}]"))); }
            }
            Step::Replay { episode_id, speed, .. } => {
                if !crate::replay::REPLAY_SPEEDS.iter().any(|x| (x - speed).abs() < 1e-9) {
                    return Err(bad(format!("speed must be one of {:?}", crate::replay::REPLAY_SPEEDS)));
                }
                if !inner.episodes.contains(episode_id) { return Err(missing(format!("no episode '{episode_id}'"))); }
            }
        }
        out.push(step);
    }
    Ok(out)
}

/// A sequences.json change checked under the lock and committed after its file is staged
/// (main's two-phase write): `next` replaces the store when the revision is unchanged.
#[derive(Debug, Clone)]
pub struct PreparedSequences {
    pub base_revision: u64,
    pub next: SequenceFile,
    /// The saved sequence (None for a delete).
    pub sequence: Option<Sequence>,
    pub write: Option<PendingWrite>,
}

fn check_store_revision(inner: &SupervisorInner, body: &Value) -> Result<u64, ArmError> {
    if !body.is_object() { return Err(invalid("Command body must be an object")); }
    // sequences use their own `rev` (SEQUENCE_CONFLICT); a store-wide expected_revision is optional
    if let Some(exp) = persist::parse_expected_revision(body, false).map_err(invalid)? {
        if exp != inner.sequences.revision {
            return Err(ArmError::PolicyRevisionMismatch { expected: exp, current: inner.sequences.revision });
        }
    }
    Ok(inner.sequences.revision)
}

impl SupervisorInner {
    fn prepared_sequences(&self, base_revision: u64, next: SequenceFile, sequence: Option<Sequence>) -> PreparedSequences {
        let write = self.sequences.path.clone().map(|path| PendingWrite::new(path, serde_json::to_string_pretty(&next).unwrap_or_default()));
        PreparedSequences { base_revision, next, sequence, write }
    }
}

/// Under the lock before the rename: revision unchanged, no calibration running, and every
/// pose / episode the saved sequence references still exists (a delete since the prepare).
pub fn precheck(inner: &SupervisorInner, prepared: &PreparedSequences) -> Result<(), ArmError> {
    if inner.sequences.revision != prepared.base_revision {
        return Err(ArmError::PolicyRevisionMismatch { expected: prepared.base_revision, current: inner.sequences.revision });
    }
    if inner.calibration_busy {
        return Err(rejected(409, "CONFIG_BUSY", "a camera calibration apply or restore is running"));
    }
    if let Some(seq) = &prepared.sequence {
        let steps = serde_json::to_value(&seq.steps).unwrap_or(Value::Null);
        check_steps(inner, &steps)?;
    }
    if let Some(p) = inner.sequences.pins.as_ref().filter(|p| !prepared.next.sequences.iter().any(|s| s.id == p.sequence_id)) {
        if inner.sequences.get(&p.sequence_id).is_some() {
            return Err(rejected(409, "SEQUENCE_RUNNING", "the sequence is running; stop it first"));
        }
    }
    Ok(())
}

/// Install a prepared change (revision checked again). Returns the store revision.
pub fn commit(inner: &mut SupervisorInner, prepared: PreparedSequences) -> Result<(Option<Sequence>, u64), ArmError> {
    if inner.sequences.revision != prepared.base_revision {
        return Err(ArmError::PolicyRevisionMismatch { expected: prepared.base_revision, current: inner.sequences.revision });
    }
    inner.sequences.file = prepared.next;
    inner.sequences.revision += 1;
    Ok((prepared.sequence, inner.sequences.revision))
}

/// WS `/api/v1/sequences/save {id?, name, description?, steps, rev?}` (in-process callers):
/// prepare and commit. Returns the sequence and the file write.
pub fn save(inner: &mut SupervisorInner, body: &Value) -> Result<(Sequence, Option<PendingWrite>), ArmError> {
    let mut prepared = prepare_save(inner, body)?;
    let write = prepared.write.take();
    let (seq, _) = commit(inner, prepared)?;
    Ok((seq.expect("a save has its sequence"), write))
}

/// Step 1 of `/api/v1/sequences/save {id?, name, description?, steps, rev?}`.
pub fn prepare_save(inner: &SupervisorInner, body: &Value) -> Result<PreparedSequences, ArmError> {
    let base_revision = check_store_revision(inner, body)?;
    let name = body.get("name").and_then(Value::as_str).ok_or_else(|| invalid("name must be a string"))?.to_string();
    if !crate::recording::store::valid_display_name(&name) {
        return Err(invalid("name must be 1-64 characters from letters, digits, _ - space and Han characters"));
    }
    let description = match body.get("description") {
        None | Some(Value::Null) => String::new(),
        Some(Value::String(d)) if d.chars().count() <= 1024 => d.clone(),
        Some(_) => return Err(invalid("description must be a string of at most 1024 characters")),
    };
    let id = match body.get("id") {
        None | Some(Value::Null) => None,
        Some(Value::String(id)) => Some(id.clone()),
        Some(_) => return Err(invalid("id must be a string")),
    };
    let rev = match body.get("rev") {
        None | Some(Value::Null) => None,
        Some(v) => Some(v.as_u64().ok_or_else(|| invalid("rev must be a non-negative integer"))?),
    };
    let steps = check_steps(inner, body.get("steps").unwrap_or(&Value::Null))?;
    if let Some(id) = &id {
        if inner.sequences.pins.as_ref().is_some_and(|p| &p.sequence_id == id) {
            // the run pinned the old steps; saving is allowed (the run does not change)
        }
        let current = inner.sequences.get(id).ok_or_else(|| rejected(404, "SEQUENCE_NOT_FOUND", format!("No sequence with id '{id}'")))?;
        if rev.is_some_and(|r| r != current.rev) {
            return Err(ArmError::detailed(409, "SEQUENCE_CONFLICT", format!("sequence '{}' is at rev {}, not {}", current.name, current.rev, rev.unwrap_or(0)),
                json!({"rev": current.rev})));
        }
    }
    if inner.sequences.file.sequences.iter().any(|s| s.name == name && Some(&s.id) != id.as_ref()) {
        return Err(rejected(409, "SEQUENCE_EXISTS", format!("Another sequence is named '{name}'")));
    }
    let now = now_rfc3339();
    let mut next = inner.sequences.file.clone();
    let seq = match id {
        Some(id) => {
            let s = next.sequences.iter_mut().find(|s| s.id == id).expect("checked above");
            s.name = name;
            s.description = description;
            s.steps = steps;
            s.updated_at = now;
            s.rev += 1;
            s.clone()
        }
        None => {
            if next.sequences.len() >= MAX_SEQUENCES { return Err(invalid(format!("at most {MAX_SEQUENCES} sequences; delete one first"))); }
            let s = Sequence { id: crate::recording::new_ulid(), name, description, steps, created_at: now.clone(), updated_at: now, rev: 1 };
            next.sequences.push(s.clone());
            s
        }
    };
    Ok(inner.prepared_sequences(base_revision, next, Some(seq)))
}

/// WS `/api/v1/sequences/delete {id}` (in-process callers): prepare and commit.
pub fn delete(inner: &mut SupervisorInner, body: &Value) -> Result<Option<PendingWrite>, ArmError> {
    let mut prepared = prepare_delete(inner, body)?;
    let write = prepared.write.take();
    commit(inner, prepared)?;
    Ok(write)
}

/// Step 1 of `/api/v1/sequences/delete {id}`; SEQUENCE_RUNNING while it runs.
pub fn prepare_delete(inner: &SupervisorInner, body: &Value) -> Result<PreparedSequences, ArmError> {
    let base_revision = check_store_revision(inner, body)?;
    let id = body.get("id").and_then(Value::as_str).ok_or_else(|| invalid("id must be a string"))?;
    if inner.sequences.pins.as_ref().is_some_and(|p| p.sequence_id == id) {
        return Err(rejected(409, "SEQUENCE_RUNNING", "the sequence is running; stop it first"));
    }
    let mut next = inner.sequences.file.clone();
    let i = next.sequences.iter().position(|s| s.id == id)
        .ok_or_else(|| rejected(404, "SEQUENCE_NOT_FOUND", format!("No sequence with id '{id}'")))?;
    next.sequences.remove(i);
    Ok(inner.prepared_sequences(base_revision, next, None))
}

/// A sequence in `GET /api/v1/sequences`: the stored fields plus the steps whose pose /
/// episode is stale or missing.
pub fn view(inner: &SupervisorInner, s: &Sequence) -> Value {
    let mut stale = Vec::new();
    let mut missing = Vec::new();
    for (i, step) in s.steps.iter().enumerate() {
        match step {
            Step::Pose { pose_id, .. } => match inner.poses.by_id(pose_id) {
                None => missing.push(i),
                Some(p) if p.stale.is_some() => stale.push(i),
                _ => {}
            },
            Step::Replay { episode_id, .. } => match inner.episodes.get(episode_id) {
                None => missing.push(i),
                Some(e) if e.joint_zero.stale_against(&inner.joint_zero_offsets) => stale.push(i),
                _ => {}
            },
            _ => {}
        }
    }
    let mut v = serde_json::to_value(s).unwrap_or_default();
    v["stale_steps"] = json!(stale);
    v["missing_steps"] = json!(missing);
    v["running"] = json!(inner.sequences.pins.as_ref().is_some_and(|p| p.sequence_id == s.id));
    v
}

// ---- run_sequence (action kind `sequence`) ----

/// `/api/v1/action/run_sequence` body.
#[derive(Debug, Clone, PartialEq)]
pub struct RunRequest {
    pub id: String,
    pub from_step: usize,
    pub single_step: bool,
    pub repeat: u32,
    pub record: Option<(String, Option<String>)>,
    pub allow_stale: bool,
    pub tag: Option<String>,
}

pub fn parse_run(body: &Value) -> Result<RunRequest, ArmError> {
    if !body.is_object() { return Err(invalid("Command body must be an object")); }
    let id = body.get("id").and_then(Value::as_str).ok_or_else(|| invalid("id must be a sequence id"))?.to_string();
    let from_step = match body.get("from_step") {
        None | Some(Value::Null) => 0,
        Some(v) => v.as_u64().ok_or_else(|| invalid("from_step must be a non-negative integer"))? as usize,
    };
    let repeat = match body.get("repeat") {
        None | Some(Value::Null) => 1,
        Some(v) => v.as_u64().filter(|r| (1..=u64::from(MAX_REPEAT)).contains(r)).ok_or_else(|| invalid(format!("repeat must be in 1..{MAX_REPEAT}")))? as u32,
    };
    let record = match body.get("record") {
        None | Some(Value::Null) => None,
        Some(r) if r.is_object() => {
            let dataset = r.get("dataset").and_then(Value::as_str).ok_or_else(|| invalid("record.dataset must be a string"))?.to_string();
            let task = match r.get("task") {
                None | Some(Value::Null) => None,
                Some(Value::String(t)) => Some(t.clone()),
                Some(_) => return Err(invalid("record.task must be a string")),
            };
            Some((dataset, task))
        }
        Some(_) => return Err(invalid("record must be an object {dataset, task?}")),
    };
    Ok(RunRequest {
        id, from_step, repeat, record,
        single_step: crate::motion::flag(body, "single_step", false).map_err(ArmError::from)?,
        allow_stale: crate::motion::flag(body, "allow_stale", false).map_err(ArmError::from)?,
        tag: crate::motion::tag(body).map_err(ArmError::from)?,
    })
}

/// A step with what it references copied at the start of the run (pinned revisions).
#[derive(Debug, Clone)]
pub enum PinnedStep {
    Pose { positions: Vec<f32>, speed_scale: f64, gripper: PoseGripper, pose_id: String },
    Gripper { action: GripperAction, fraction: Option<f64> },
    Wait { s: f64 },
    Replay { episode_id: String, speed: f64, gripper: bool },
}

impl PinnedStep {
    fn kind(&self) -> &'static str {
        match self { Self::Pose { .. } => "pose", Self::Gripper { .. } => "gripper", Self::Wait { .. } => "wait", Self::Replay { .. } => "replay" }
    }
}

/// A checked run, ready to begin.
#[derive(Debug, Clone)]
pub struct RunPlan {
    pub request: RunRequest,
    pub sequence: Sequence,
    /// (index in the sequence, step) of the steps to run.
    pub steps: Vec<(usize, PinnedStep)>,
    pub repeats: u32,
    pub epoch: u64,
    pub workspace_revision: u64,
    pub calibration_revision: u64,
    pub expected_duration_s: f64,
}

/// Check a run under the lock: sequence, steps, references (pinned), stale data, recorder,
/// motion preconditions; computes the duration bound.
pub fn prepare_run(inner: &SupervisorInner, body: &Value, epoch: u64) -> Result<RunPlan, ArmError> {
    inner.check_commissioning_for_ordinary_motion("run_sequence")?;
    let request = parse_run(body)?;
    if epoch != inner.safety.epoch_id() {
        return Err(crate::safety::SafetyError::EpochMismatch { expected: inner.safety.epoch_id(), received: epoch }.into());
    }
    let sequence = inner.sequences.get(&request.id).cloned()
        .ok_or_else(|| rejected(404, "SEQUENCE_NOT_FOUND", format!("No sequence with id '{}'", request.id)))?;
    if sequence.steps.is_empty() { return Err(invalid("the sequence has no steps")); }
    if request.from_step >= sequence.steps.len() {
        return Err(invalid(format!("from_step must be below {}", sequence.steps.len())));
    }
    let end = if request.single_step { request.from_step + 1 } else { sequence.steps.len() };
    let repeats = if request.single_step { 1 } else { request.repeat };
    let mut steps = Vec::new();
    let mut stale = Vec::new();
    for i in request.from_step..end {
        let missing = |m: String| ArmError::detailed(422, "SEQUENCE_REF_MISSING", format!("step {i}: {m}"), json!({"step": i}));
        let pinned = match &sequence.steps[i] {
            Step::Pose { pose_id, speed_scale, gripper } => {
                let p = inner.poses.by_id(pose_id).ok_or_else(|| missing(format!("no pose with id '{pose_id}'")))?;
                if p.stale.is_some() { stale.push(i); }
                if p.positions.len() != inner.config.dof { return Err(invalid(format!("step {i}: pose '{}' has {} joints", p.name, p.positions.len()))); }
                if *gripper == PoseGripper::Pose
                    && inner.config.gripper_index().is_some()
                    && !inner
                        .config
                        .active_tool
                        .matches_record(p.tool_id.as_deref(), p.tool_config_digest.as_deref())
                {
                    return Err(rejected(409, "TOOL_MISMATCH", format!(
                        "step {i}: pose '{}' gripper position was saved with tool {:?} ({:?}), which does not match active tool '{}' ({})",
                        p.name, p.tool_id, p.tool_config_digest, inner.config.active_tool.id, inner.config.active_tool.config_digest
                    )));
                }
                PinnedStep::Pose { positions: p.positions.clone(), speed_scale: *speed_scale, gripper: *gripper, pose_id: pose_id.clone() }
            }
            Step::Gripper { action, fraction } => PinnedStep::Gripper { action: *action, fraction: *fraction },
            Step::Wait { s } => PinnedStep::Wait { s: *s },
            Step::Replay { episode_id, speed, gripper } => {
                let e = inner.episodes.get(episode_id).ok_or_else(|| missing(format!("no episode '{episode_id}'")))?;
                if e.joint_zero.stale_against(&inner.joint_zero_offsets) { stale.push(i); }
                if *gripper {
                    let gripper_involved = e.has_gripper_motion == Some(true)
                        || inner.config.gripper_index().is_some_and(|g| {
                            e.first_state
                                .as_ref()
                                .and_then(|fs| fs.get(g).copied())
                                .zip(inner.commanded_positions.get(g).copied())
                                .is_some_and(|(fg, cg)| (fg - cg).abs() > 1e-3)
                        });
                    if gripper_involved
                        && !inner
                            .config
                            .active_tool
                            .matches_record(e.tool_id.as_deref(), e.tool_config_digest.as_deref())
                    {
                        return Err(rejected(409, "TOOL_MISMATCH", format!(
                            "step {i}: episode '{episode_id}' contains gripper motion from tool {:?} ({:?}), which does not match active tool '{}' ({})",
                            e.tool_id, e.tool_config_digest, inner.config.active_tool.id, inner.config.active_tool.config_digest
                        )));
                    }
                }
                PinnedStep::Replay { episode_id: episode_id.clone(), speed: *speed, gripper: *gripper }
            }
        };
        steps.push((i, pinned));
    }
    if !stale.is_empty() && !request.allow_stale {
        return Err(ArmError::detailed(409, "DATA_STALE", format!("steps {stale:?} use poses / episodes taught with another joint zero; check them, then run with allow_stale: true"),
            json!({"steps": stale})));
    }
    if let Some((dataset, _)) = &request.record {
        if !inner.episodes.enabled() { return Err(crate::recording::StoreError::Disabled.into()); }
        inner.recorder_idle()?;
        if !inner.episodes.has_dataset(dataset) { return Err(crate::recording::StoreError::DatasetNotFound(dataset.clone()).into()); }
    }
    inner.action_preconditions()?;
    inner.gravity_comp_motion_busy()?;
    let expected_duration_s = expected_duration(inner, &steps, repeats, request.record.is_some());
    Ok(RunPlan {
        request, sequence, steps, repeats, epoch,
        workspace_revision: inner.workspace_revision, calibration_revision: inner.calibration_revision, expected_duration_s,
    })
}

/// Upper bound of a run (s): per pose step the quintic from the previous target (from the
/// joint-limit extremes when unknown, after a replay) plus the settle limit; gripper steps
/// their expected duration + 1 s; waits; replays a worst-case approach + recorded duration /
/// speed + 0.5 s; 1 s per repeat for saving an episode. Power-budget slowdowns are not included.
pub fn expected_duration(inner: &SupervisorInner, steps: &[(usize, PinnedStep)], repeats: u32, record: bool) -> f64 {
    let dof = inner.config.dof;
    let limits = inner.planning_limits();
    let lo: Vec<f32> = inner.config.limits.iter().map(|l| l.min_position).collect();
    let hi: Vec<f32> = inner.config.limits.iter().map(|l| l.max_position).collect();
    let quintic = |a: &[f32], b: &[f32], lim: &[crate::model::JointLimit]| -> f64 {
        crate::trajectory::TrajectoryPlanner::plan(a, &vec![0.0; dof], b, &vec![0.0; dof], None, lim).map_or(10.0, |t| t.duration)
    };
    let worst = quintic(&lo, &hi, &limits);
    let gripper_idx = inner.config.gripper_index();
    // one pass from `q`: (duration, end state)
    let pass = |mut q: Option<Vec<f32>>| -> (f64, Option<Vec<f32>>) {
        let mut once = 0.0;
        for (_, step) in steps {
            match step {
                PinnedStep::Pose { positions, speed_scale, gripper, .. } => {
                    let mut lim = limits.clone();
                    for (i, l) in lim.iter_mut().enumerate() { if Some(i) != gripper_idx { l.max_velocity *= *speed_scale as f32; } }
                    let mut target = q.clone().unwrap_or_else(|| positions.clone());
                    for i in (0..dof).filter(|i| Some(*i) != gripper_idx) { target[i] = positions[i]; }
                    if let (PoseGripper::Pose, Some(g)) = (gripper, gripper_idx) { target[g] = positions[g]; }
                    once += match &q { Some(a) => quintic(a, &target, &lim), None => quintic(&lo, &hi, &lim) } + crate::motion::SETTLE_MAX_S;
                    q = Some(target);
                }
                PinnedStep::Gripper { action, fraction } => {
                    once += 1.0 + match (gripper_idx, inner.config.gripper_policy.clone()) {
                        (Some(g), Some(policy)) => {
                            let request = gripper_request(*action, *fraction, policy.close_speed());
                            let start = q.as_ref().map_or(inner.commanded_positions[g], |q| q[g]);
                            match crate::motion::GripperMotion::try_with_close_speed_limit(
                                request, policy, inner.config.limits[g].clone(), start, g, inner.gripper_close_speed_limit(),
                            ) {
                                Ok(m) => {
                                    if let Some(q) = q.as_mut() { q[g] = m.target; }
                                    m.expected_duration()
                                }
                                Err(_) => 0.0,
                            }
                        }
                        _ => 0.0,
                    };
                }
                PinnedStep::Wait { s } => once += s,
                PinnedStep::Replay { episode_id, speed, gripper } => {
                    let e = inner.episodes.get(episode_id);
                    let recorded = e.as_ref().map_or(crate::recording::store::MAX_EPISODE_S, |e| e.duration_s);
                    let (first, last) = e.map_or((None, None), |e| (e.first_state, e.last_state));
                    let hold = |mut f: Vec<f32>, q: &[f32]| { if let (false, Some(g)) = (gripper, gripper_idx) { f[g] = q[g]; } f };
                    let approach = match (&q, first.filter(|f| f.len() == dof)) {
                        (Some(a), Some(f)) => quintic(a, &hold(f, a), &limits),
                        _ => worst,
                    };
                    once += approach + recorded / speed + 0.5;
                    q = match (q, last.filter(|l| l.len() == dof)) { (Some(a), Some(l)) => Some(hold(l, &a)), _ => None };
                }
            }
        }
        (once + if record { 1.0 } else { 0.0 }, q)
    };
    // the first repeat starts at the commanded position, later ones where the previous ended
    let (first, end) = pass(Some(inner.commanded_positions.clone()));
    let next = if repeats > 1 { pass(end).0 } else { 0.0 };
    first + next * f64::from(repeats.saturating_sub(1)) + 1.0
}

/// The request a sequence gripper step runs (as `macro_action::MacroCtx::gripper` /
/// `gripper_move` build it): closes and moves at `close_speed` with the derived deadline.
fn gripper_request(action: GripperAction, fraction: Option<f64>, close_speed: f64) -> crate::motion::GripperRequest {
    use crate::motion::GripperCommand as C;
    let command = match action { GripperAction::Open => C::Open, GripperAction::Close => C::Close, GripperAction::Move => C::Move };
    let (speed, timeout_s) = if command == C::Open { (1.0, Some(8.0)) } else { (close_speed, None) };
    crate::motion::GripperRequest { command, fraction, grasp: command == C::Close, speed, timeout_s, tag: None }
}

/// Build the A7 [`BeginMacroSpec`] for a prepared sequence run (shared by Legacy and RT Executor).
pub fn begin_macro_spec(
    plan: &RunPlan,
    prepared_boot: crate::rt_core::broker::BootId,
    deadline: Instant,
) -> crate::rt_core::requests::actions::BeginMacroSpec {
    use crate::rt_core::requests::actions::{BeginMacroSpec, MacroKind};
    let mut spec = BeginMacroSpec::new(MacroKind::Sequence, prepared_boot);
    spec.check_workspace_revision = true;
    spec.calibration_revision = Some(plan.calibration_revision);
    spec.macro_deadline = Some(deadline);
    spec.timeout_code = crate::safety::BoundedText::new("SEQUENCE_TIMEOUT");
    if let Some(tag) = plan.request.tag.as_deref() {
        spec.tag = crate::model::HolderLabel::new(tag);
    }
    spec
}

fn finish_begin_setup(
    inner: &mut SupervisorInner,
    sup: &Arc<ArmSupervisor>,
    id: u64,
    plan: RunPlan,
    stamp: crate::supervisor::AdmissionStamp,
    deadline: Instant,
) -> (u64, f64) {
    let pose_ids = plan.steps.iter().filter_map(|(_, s)| match s { PinnedStep::Pose { pose_id, .. } => Some(pose_id.clone()), _ => None }).collect();
    let episode_ids = plan.steps.iter().filter_map(|(_, s)| match s { PinnedStep::Replay { episode_id, .. } => Some(episode_id.clone()), _ => None }).collect();
    inner.sequences.pins = Some(RunPins { sequence_id: plan.sequence.id.clone(), action_id: id, pose_ids, episode_ids });
    let first = plan.steps.first().map(|(i, s)| (*i, s.kind()));
    inner.update_action(id, |a| {
        a.sequence_id = Some(plan.sequence.id.clone());
        a.repeats = Some(plan.repeats);
        a.repeat_index = Some(0);
        a.step_index = first.map(|f| f.0);
        a.step_type = first.map(|f| f.1.to_string());
    });
    let duration = plan.expected_duration_s;
    tracing::info!("Sequence '{}' started (action {id}, {} steps x {}, record {}, bound {duration:.1} s)",
        plan.sequence.name, plan.steps.len(), plan.repeats, plan.request.record.is_some());
    let sup = sup.clone();
    tokio::spawn(async move {
        let worker = { let sup = sup.clone(); tokio::spawn(async move { run(sup, id, plan, stamp, deadline).await }) };
        let outcome = worker.await;
        cleanup(&sup, id, outcome.err().map(|e| e.to_string())).await;
    });
    (id, duration)
}

/// Begin the parent action under a short write lock through the single owner-side
/// [`crate::rt_core::requests::actions::install_begin_macro`] function and spawn its orchestrator.
pub fn begin_with_stamp(
    inner: &mut SupervisorInner,
    stamp: crate::supervisor::AdmissionStamp,
    sup: &Arc<ArmSupervisor>,
    plan: RunPlan,
) -> Result<(u64, f64), ArmError> {
    use crate::rt_core::requests::actions::{
        install_begin_macro, ActionStamp, OwnerCheckCtx, DEFAULT_ACTION_DEADLINE,
    };
    let deadline = Instant::now() + Duration::from_secs_f64(plan.expected_duration_s * 2.0 + 30.0);
    let spec = begin_macro_spec(&plan, inner.boot_id, deadline);
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
        plan.request.tag.clone(),
        None,
    )
    .map_err(|r| r.into_arm_error(inner))?;
    Ok(finish_begin_setup(inner, sup, id, plan, stamp, deadline))
}

/// Begin the parent action and spawn its orchestrator. Returns (action id, expected duration).
pub fn begin(admitted: crate::supervisor::Admitted<'_>, sup: &Arc<ArmSupervisor>, plan: RunPlan) -> Result<(u64, f64), ArmError> {
    let deadline = Instant::now() + Duration::from_secs_f64(plan.expected_duration_s * 2.0 + 30.0);
    let (inner, proof) = admitted.consume()?;
    let stamp = proof.stamp();
    if inner.safety.epoch_id() != plan.epoch {
        return Err(crate::safety::SafetyError::EpochMismatch {
            expected: inner.safety.epoch_id(),
            received: plan.epoch,
        }
        .into());
    }
    let spec = begin_macro_spec(&plan, inner.boot_id, deadline);
    let ws_rev = plan.workspace_revision;
    let id = crate::rt_core::requests::actions::install_begin_macro_with_proof(
        inner,
        &proof,
        &spec,
        Some(ws_rev),
        plan.request.tag.clone(),
        None,
    )?;
    Ok(finish_begin_setup(inner, sup, id, plan, stamp, deadline))
}

/// After the orchestrator: discard an unfinished repeat's recording, release the pins, end
/// the action when the orchestrator died without ending it.
async fn cleanup(sup: &Arc<ArmSupervisor>, id: u64, panic: Option<String>) {
    let owned = sup.inner().read().await.recording.as_ref()
        .is_some_and(|r| r.owner == crate::teach::RecorderOwner::Sequence { action_id: id });
    if owned {
        if let Err(e) = sup.stop_recording_as(crate::teach::Stopper::Owner(crate::teach::RecorderOwner::Sequence { action_id: id }), false, None, None).await {
            tracing::warn!("Sequence {id}: discarding its recording failed: {e}");
        } else {
            tracing::info!("Sequence {id}: unfinished repeat's recording discarded");
        }
    }
    let mut inner = sup.inner().write().await;
    if inner.sequences.pins.as_ref().is_some_and(|p| p.action_id == id) { inner.sequences.pins = None; }
    if let Some(e) = panic {
        tracing::error!("Sequence {id} orchestrator failed: {e}");
        inner.finish_macro(id, "failed", Some("INTERNAL"), Some(format!("sequence orchestrator failed: {e}")), None, None);
    }
}

/// The orchestrator: repeats x steps, rechecks before each motion step, one episode per repeat.
async fn run(
    sup: Arc<ArmSupervisor>,
    id: u64,
    plan: RunPlan,
    stamp: crate::supervisor::AdmissionStamp,
    deadline: Instant,
) {
    let ctx = MacroCtx {
        sup: sup.clone(), id, stamp,
        deadline,
        phase_timeout_s: plan.expected_duration_s * 2.0 + 30.0,
        timeout_code: "SEQUENCE_TIMEOUT",
        timeout_message: "the sequence did not finish within twice its expected duration".into(),
    };
    let total = (plan.steps.len() as f32 * plan.repeats as f32).max(1.0);
    let owner = crate::teach::RecorderOwner::Sequence { action_id: id };
    // the step that ran last (failed_step when the run ends between steps)
    let mut current: Option<usize> = None;
    for r in 0..plan.repeats {
        if let Some((dataset, task)) = &plan.request.record {
            let options = crate::teach::RecordOptions {
                dataset: dataset.clone(), task: task.clone().unwrap_or_else(|| plan.sequence.name.clone()), source: "sequence".into(),
                sequence_id: Some(plan.sequence.id.clone()), repeat_index: Some(r),
            };
            if !ctx.alive().await { return end(&ctx, Err(Stop::Cancelled), current).await; }
            if let Err(e) = sup.start_recording_as(owner.clone(), options, None, None).await {
                return end(&ctx, Err(arm_fail(e)), plan.steps.first().map(|s| s.0)).await;
            }
        }
        for (k, (index, step)) in plan.steps.iter().enumerate() {
            let progress = (r as f32 * plan.steps.len() as f32 + k as f32) / total;
            if !ctx.is_executor() {
                let mut inner = sup.inner().write().await;
                if !inner.macro_alive(id) { drop(inner); return end(&ctx, Err(Stop::Cancelled), current).await; }
                inner.update_action(id, |a| {
                    a.step_index = Some(*index);
                    a.step_type = Some(step.kind().into());
                    a.repeat_index = Some(r);
                });
                inner.set_action_phase(id, step.kind(), progress, None, None);
            } else if !ctx.alive().await {
                return end(&ctx, Err(Stop::Cancelled), current).await;
            }
            current = Some(*index);
            if let Err(stop) = run_step(&ctx, &plan, step).await { return end(&ctx, Err(stop), current).await; }
        }
        if plan.request.record.is_some() {
            if !ctx.alive().await { return end(&ctx, Err(Stop::Cancelled), current).await; }
            match sup.stop_recording_as(crate::teach::Stopper::Owner(owner.clone()), true, Some("ok"), None).await {
                Ok(crate::teach::Stopped::Saved { sidecar, .. }) => {
                    if !ctx.is_executor() {
                        sup.inner().write().await.update_action(id, |a| a.episodes.push(sidecar.id.clone()));
                    }
                }
                Ok(_) => {}
                Err(e) => return end(&ctx, Err(arm_fail(e)), current).await,
            }
        }
    }
    end(&ctx, Ok(()), None).await
}

/// End the run; a run that did not finish records `failed_step` (also when a stop, lease loss
/// or fault ended it while the step ran).
async fn end(ctx: &MacroCtx, result: Result<(), Stop>, step: Option<usize>) {
    use crate::rt_core::requests::actions::{FinishState, HeldUpdate};
    match result {
        Ok(()) => {
            ctx.finish(FinishState::Done, None, None, None, None, HeldUpdate::Unchanged)
                .await;
            return;
        }
        Err(Stop::Fail(code, message)) => {
            if !ctx.is_executor() {
                let mut inner = ctx.sup.inner().write().await;
                if let Some(a) = inner.last_action.as_mut().filter(|a| a.id == ctx.id && a.is_running()) {
                    a.failed_step = step;
                }
            }
            ctx.finish(
                FinishState::Failed,
                Some(&code),
                Some(message),
                None,
                None,
                HeldUpdate::Unchanged,
            )
            .await;
        }
        Err(Stop::Cancelled) => {}
    }
    if !ctx.is_executor() {
        let mut inner = ctx.sup.inner().write().await;
        if let Some(a) = inner.last_action.as_mut().filter(|a| a.id == ctx.id && !a.is_running()) {
            a.failed_step = step;
        }
        inner.update_recorded_terminal_failed_step(ctx.id, step);
    }
}

/// Before a motion step: the run is alive and nothing it was checked against changed.
async fn recheck(ctx: &MacroCtx, plan: &RunPlan) -> Result<(), Stop> {
    if ctx.is_executor() {
        let snap = ctx.snapshot(ctx.deadline).await?;
        let m = snap.owner.macro_state;
        if m.active_id != ctx.id || !m.alive { return Err(Stop::Cancelled); }
        if snap.owner.authority.epoch != plan.epoch { return Err(fail("PLAN_STALE", "the epoch changed during the sequence (stop or e-stop)")); }
        if snap.owner.authority.config_revision != plan.workspace_revision { return Err(fail("PLAN_STALE", "the workspace or keep-out boxes changed during the sequence")); }
        if snap.owner.calibration_revision != plan.calibration_revision {
            return Err(fail("CALIBRATION_CHANGED", "the joint zero or the kinematic correction changed during the sequence"));
        }
        if Instant::now() > ctx.deadline { return Err(fail(ctx.timeout_code, ctx.timeout_message.clone())); }
        return Ok(());
    }
    let mut inner = ctx.sup.inner().write().await;
    if !inner.macro_alive(ctx.id) { return Err(Stop::Cancelled); }
    if inner.safety.epoch_id() != plan.epoch { return Err(fail("PLAN_STALE", "the epoch changed during the sequence (stop or e-stop)")); }
    if inner.workspace_revision != plan.workspace_revision { return Err(fail("PLAN_STALE", "the workspace or keep-out boxes changed during the sequence")); }
    if inner.calibration_revision != plan.calibration_revision {
        return Err(fail("CALIBRATION_CHANGED", "the joint zero or the kinematic correction changed during the sequence"));
    }
    if inner.active_lease.as_ref().map(|l| l.generation) != inner.action_lease_generation {
        let id = ctx.id;
        inner.finish_macro(id, "cancelled", Some("LEASE_LOST"), Some("Control lease lost or released".into()), None, None);
        return Err(Stop::Cancelled);
    }
    if Instant::now() > ctx.deadline { return Err(fail(ctx.timeout_code, ctx.timeout_message.clone())); }
    Ok(())
}

async fn run_step(ctx: &MacroCtx, plan: &RunPlan, step: &PinnedStep) -> Result<(), Stop> {
    match step {
        PinnedStep::Wait { s } => {
            let until = Instant::now() + Duration::from_secs_f64(*s);
            // stop, lease loss, fault and e-stop end the action (reconcile); checked every poll
            while Instant::now() < until {
                ctx.check().await?;
                tokio::time::sleep(crate::macro_action::POLL.min(until.saturating_duration_since(Instant::now()))).await;
            }
            ctx.check().await
        }
        PinnedStep::Pose { positions, speed_scale, gripper, .. } => {
            recheck(ctx, plan).await?;
            let snap = ctx.snapshot(ctx.deadline).await?;
            let gripper_idx = snap.env.config.gripper_index();
            let mut target = snap.commanded_positions.clone();
            for i in (0..snap.env.config.dof).filter(|i| Some(*i) != gripper_idx) { target[i] = positions[i]; }
            if let (PoseGripper::Pose, Some(g)) = (gripper, gripper_idx) { target[g] = positions[g]; }
            let limits = snap.env.planning_limits(Some(*speed_scale));
            ctx.install_joint_child(
                "pose",
                target,
                limits,
                true,
                Some(plan.workspace_revision),
                Some(plan.calibration_revision),
            )
            .await
            .map(|_| ())
            .map_err(|(s, _)| s)
        }
        PinnedStep::Gripper { action, fraction } => {
            recheck(ctx, plan).await?;
            match action {
                GripperAction::Open => ctx.gripper("gripper", crate::motion::GripperCommand::Open).await.map(|_| ()),
                GripperAction::Close => ctx.gripper("gripper", crate::motion::GripperCommand::Close).await.map(|_| ()),
                GripperAction::Move => ctx.gripper_move("gripper", fraction.unwrap_or(0.0)).await.map(|_| ()),
            }
        }
        PinnedStep::Replay { episode_id, speed, gripper } => {
            recheck(ctx, plan).await?;
            let snap = ctx.snapshot(ctx.deadline).await?;
            let request = crate::replay::ReplayRequest {
                episode: episode_id.clone(),
                speed: *speed,
                gripper: *gripper,
                tag: None,
                allow_stale: plan.request.allow_stale,
            };
            let input = snap.replay_input(request, plan.epoch).map_err(arm_fail)?;
            let planned = tokio::task::spawn_blocking(move || crate::replay::plan(input)).await
                .map_err(|e| fail("INTERNAL", format!("replay check task failed: {e}")))?
                .map_err(arm_fail)?;
            recheck(ctx, plan).await?;
            ctx.install_replay_child(
                "replay",
                planned,
                Some(plan.workspace_revision),
                Some(plan.calibration_revision),
            )
            .await
            .map(|_| ())
        }
    }
}
