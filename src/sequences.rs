//! Sequences (teach v1): ordered steps over stored poses and episodes, `sequences.json`.
//! References are by id (poses: [`crate::poses::NamedPose::id`], episodes: the episode id);
//! a referenced pose or episode cannot be deleted (IN_USE).
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::persist::{self, Loaded, PendingWrite};
use crate::supervisor::{ArmSupervisor, SupervisorInner};

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
