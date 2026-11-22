//! Teach v1, armd side of the recording session and the lease-free episode commands
//! (contract: rebot-companion `_scratch/teach/CONTRACT.md`, docs/agent_action_api.md "Teach").
//!
//! - One recording at a time. It has an owner ([`RecorderOwner`]) that is separate from the
//!   motion lease: the legacy REST routes (`legacy-rest`), a WS connection (its `client_id`), or
//!   a sequence run. Only the owner stops it.
//! - The control tick only appends a frame to the recorder's buffer (as before teach v1); the
//!   parquet and sidecar are written on a blocking thread after the frames left the lock.
//! - A monitor task per recording (outside the control loop) stops it at [`MAX_EPISODE_S`]
//!   (saved, `truncated`), below [`STOP_MIN_FREE_BYTES`] free space (saved) and discards it 5 s
//!   after its owner's WS connection closed.
//!
//! [`MAX_EPISODE_S`]: crate::recording::store::MAX_EPISODE_S
//! [`STOP_MIN_FREE_BYTES`]: crate::recording::store::STOP_MIN_FREE_BYTES
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

use tokio::sync::RwLock;

use serde_json::{json, Value};

use crate::recording::store::{self, EpisodeSidecar, JointZeroStamp, StoreError};
use crate::recording::{EpisodeState, ParquetRecordingError, RecordFrame};
use crate::supervisor::{ArmError, ArmSupervisor, SupervisorInner};

/// A recording whose owner connection closed is discarded after this.
pub const OWNER_LOST_DISCARD: Duration = Duration::from_secs(5);
/// Period of the recording monitor.
pub const MONITOR_PERIOD: Duration = Duration::from_millis(200);
/// Free space is checked by the monitor at most this often.
pub const SPACE_CHECK_PERIOD: Duration = Duration::from_secs(1);

/// Who owns the running recording.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RecorderOwner {
    /// `/api/v1/dataset/record/*` (no identity).
    LegacyRest,
    /// A WS connection; `client_id` is what the client calls itself (reported as `owner`).
    Client { connection: String, client_id: String },
    /// The sequence run (action id) that records itself.
    Sequence { action_id: u64 },
}

impl RecorderOwner {
    /// `owner` in replies: `legacy-rest`, the client id, `sequence:<action id>`.
    pub fn label(&self) -> String {
        match self {
            Self::LegacyRest => "legacy-rest".into(),
            Self::Client { client_id, .. } => client_id.clone(),
            Self::Sequence { action_id } => format!("sequence:{action_id}"),
        }
    }
}

/// What a new recording is stored as.
#[derive(Debug, Clone, PartialEq)]
pub struct RecordOptions {
    pub dataset: String,
    pub task: String,
    pub source: String,
    pub sequence_id: Option<String>,
    pub repeat_index: Option<u32>,
}

impl Default for RecordOptions {
    fn default() -> Self {
        Self { dataset: store::DEFAULT_DATASET.into(), task: String::new(), source: "sdk".into(), sequence_id: None, repeat_index: None }
    }
}

/// The running recording (supervisor state; the frames are in `SupervisorInner::recorder`).
#[derive(Debug, Clone)]
pub struct ActiveRecording {
    pub id: String,
    pub owner: RecorderOwner,
    pub options: RecordOptions,
    pub recorded_at: String,
    pub started: Instant,
    pub joint_zero: JointZeroStamp,
    /// The tick appends at most this many frames (the length cap at the control rate).
    pub max_frames: usize,
    /// When the owner's connection closed.
    pub owner_lost_at: Option<Instant>,
}

/// Who asks to end the recording.
#[derive(Debug, Clone, PartialEq)]
pub enum Stopper {
    Owner(RecorderOwner),
    /// The monitor (length cap, low space: `reason`; owner lost: discard).
    Monitor { reason: &'static str },
}

/// Result of ending a recording.
#[derive(Debug, Clone, PartialEq)]
pub enum Stopped {
    Saved { sidecar: Box<EpisodeSidecar>, path: PathBuf },
    Discarded,
}

/// Custom parquet writer (tests): writes the frames to the path it gets.
pub type ParquetWriter = Box<dyn FnOnce(&Path, &[RecordFrame]) -> Result<(), ParquetRecordingError> + Send>;

fn rejected(status: u16, code: &str, message: impl Into<String>) -> ArmError {
    ArmError::Rejected { status, code: code.into(), message: message.into() }
}

impl SupervisorInner {
    /// Joint names of the profile (sidecars, import checks).
    pub fn joint_names(&self) -> Vec<String> { self.config.limits.iter().map(|l| l.name.clone()).collect() }

    /// RECORDER_BUSY while a recording runs or is being written.
    pub fn recorder_idle(&self) -> Result<(), ArmError> {
        if let Some(r) = &self.recording {
            return Err(ArmError::detailed(409, "RECORDER_BUSY", format!("a recording owned by '{}' is running", r.owner.label()),
                json!({"owner": r.owner.label(), "recording_id": r.id})));
        }
        if matches!(self.recorder.state(), EpisodeState::Recording | EpisodeState::Finalizing) {
            return Err(ArmError::detailed(409, "RECORDER_BUSY", "an episode is being saved", json!({"owner": Value::Null})));
        }
        Ok(())
    }

    /// The tick appends frames while this holds.
    pub(crate) fn recording_frame_room(&self) -> bool {
        self.recorder.state() == &EpisodeState::Recording
            && self.recording.as_ref().is_none_or(|r| self.recorder.frame_count() < r.max_frames)
    }

    /// `recording` field of `GET /api/v1/episodes` and the recorder status.
    pub fn recording_status(&self) -> Value {
        match &self.recording {
            None => Value::Null,
            Some(r) => json!({
                "recording_id": r.id, "owner": r.owner.label(), "dataset": r.options.dataset, "task": r.options.task, "started_at": r.recorded_at,
                "source": r.options.source, "frames": self.recorder.frame_count(), "elapsed_s": r.started.elapsed().as_secs_f64(),
                "owner_connected": r.owner_lost_at.is_none(),
            }),
        }
    }
}

impl ArmSupervisor {
    /// Load the episode catalog (boot recovery) once; later calls do nothing.
    pub async fn ensure_episodes_loaded(&self) -> Result<Option<store::RecoveryReport>, ArmError> {
        let (st, offsets, names, profile, fps) = {
            let inner = self.inner().read().await;
            (inner.episodes.clone(), inner.joint_zero_offsets.clone(), inner.joint_names(), inner.config.profile_id.clone(), f64::from(inner.config.control_frequency_hz))
        };
        if !st.enabled() || st.is_loaded() { return Ok(None); }
        let report = tokio::task::spawn_blocking(move || st.load(&offsets, &names, profile.as_deref(), fps)).await
            .map_err(|e| ArmError::Internal(format!("episode load failed: {e}")))??;
        Ok(Some(report))
    }

    /// Boot: run the episode recovery (after the joint zero was loaded) and log it.
    pub async fn load_episodes(&self) {
        match self.ensure_episodes_loaded().await {
            Ok(Some(r)) => tracing::info!("Episodes: {} listed; recovery: {} temp files removed, {} sidecars rebuilt, {} legacy episodes adopted, {} orphans removed, {} unreadable kept, {} lone sidecars removed",
                r.episodes, r.temps_removed.len(), r.sidecars_rebuilt.len(), r.legacy_adopted.len(), r.orphans_removed.len(), r.orphans_kept.len(), r.sidecars_removed.len()),
            Ok(None) => {}
            Err(e) => tracing::warn!("Episode store not loaded: {e}"),
        }
    }

    /// Start a recording owned by `owner`. `legacy_id` / `legacy_path`: the legacy REST
    /// `episode_id` / `output_path` (else a new ULID). Returns (recording id, owner label).
    pub async fn start_recording_as(&self, owner: RecorderOwner, options: RecordOptions,
                                    legacy_id: Option<String>, legacy_path: Option<PathBuf>) -> Result<(String, String), ArmError> {
        if !store::EPISODE_SOURCES.contains(&options.source.as_str()) {
            return Err(rejected(400, "INVALID_COMMAND", format!("source must be one of {:?}", store::EPISODE_SOURCES)));
        }
        if options.task.chars().count() > 256 { return Err(rejected(400, "INVALID_COMMAND", "task is at most 256 characters")); }
        self.ensure_episodes_loaded().await?;
        let st = {
            let inner = self.inner().read().await;
            inner.recorder_idle()?;
            inner.episodes.clone()
        };
        let dataset = options.dataset.clone();
        // file system checks without the lock (the control tick needs it)
        let (id, path) = tokio::task::spawn_blocking(move || {
            let episode = st.new_episode(legacy_id.as_deref(), legacy_path.as_deref())?;
            if !st.has_dataset(&dataset) { return Err(StoreError::DatasetNotFound(dataset)); }
            st.check_capacity()?;
            st.check_free_for_start()?;
            Ok::<_, StoreError>(episode)
        }).await.map_err(|e| ArmError::Internal(format!("episode check failed: {e}")))??;
        let label = owner.label();
        {
            let mut inner = self.inner().write().await;
            inner.recorder_idle()?;
            inner.recorder.start(Some(id.clone()), Some(path))?;
            let max_frames = (inner.max_episode_s * f64::from(inner.config.control_frequency_hz.max(1))).ceil() as usize + 1;
            inner.recording = Some(ActiveRecording {
                id: id.clone(), owner, options,
                recorded_at: chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Millis, true),
                started: Instant::now(), joint_zero: JointZeroStamp::of(&inner.joint_zero_offsets), max_frames, owner_lost_at: None,
            });
        }
        tracing::info!("Recording {id} started (owner {label})");
        let lock = self.inner().clone();
        let mid = id.clone();
        tokio::spawn(async move { monitor(lock, mid).await });
        Ok((id, label))
    }

    /// End the recording: discard, or save with `outcome` (`ok` | `aborted`). Only its owner
    /// (or the monitor) may; others get RECORDER_NOT_OWNER. `writer`: custom parquet writer.
    pub async fn stop_recording_as(&self, by: Stopper, save: bool, outcome: Option<&str>, writer: Option<ParquetWriter>) -> Result<Stopped, ArmError> {
        stop_recording(self.inner(), by, save, outcome, writer).await
    }
    /// The owner `caller` stands for: its own connection's, or (taking the recording back) that
    /// of a lost owner with the same client id. Anything else is not the owner.
    pub async fn claim_recording(&self, caller: &Caller) -> RecorderOwner {
        let me = RecorderOwner::Client { connection: caller.connection.clone(), client_id: caller.client_id.clone().unwrap_or_default() };
        let mut inner = self.inner().write().await;
        let Some(r) = inner.recording.as_mut() else { return me; };
        if let RecorderOwner::Client { connection, client_id } = &mut r.owner {
            if *connection == caller.connection { return r.owner.clone(); }
            if r.owner_lost_at.is_some() && caller.client_id.as_deref() == Some(client_id.as_str()) {
                *connection = caller.connection.clone();
                r.owner_lost_at = None;
                tracing::info!("Recording {} taken back by '{client_id}' on a new connection", r.id);
                return r.owner.clone();
            }
        }
        me
    }

    /// A WS connection closed: a recording it owns is discarded after [`OWNER_LOST_DISCARD`]
    /// unless a connection with the same client id takes it back.
    pub async fn recorder_connection_closed(&self, connection: &str) {
        let mut inner = self.inner().write().await;
        if let Some(r) = inner.recording.as_mut() {
            if matches!(&r.owner, RecorderOwner::Client { connection: c, .. } if c == connection) && r.owner_lost_at.is_none() {
                r.owner_lost_at = Some(Instant::now());
                tracing::warn!("Owner of recording {} disconnected; discarding in {} s unless it reconnects", r.id, OWNER_LOST_DISCARD.as_secs());
            }
        }
    }

    // ---- legacy REST (`/api/v1/dataset/record/*`, owner `legacy-rest`) ----

    pub async fn start_recording(&self, episode_id: Option<String>, output_path: Option<PathBuf>) -> Result<(), ArmError> {
        self.start_recording_as(RecorderOwner::LegacyRest, RecordOptions::default(), episode_id, output_path).await.map(|_| ())
    }

    pub async fn save_recording(&self) -> Result<PathBuf, ArmError> {
        match self.stop_recording_as(Stopper::Owner(RecorderOwner::LegacyRest), true, None, None).await? {
            Stopped::Saved { path, .. } => Ok(path),
            Stopped::Discarded => Err(ArmError::Internal("recording was discarded".into())),
        }
    }

    /// Save with a caller-supplied parquet writer (tests). The lock is held only to move the
    /// frames out and to record the outcome; the write runs on a blocking thread.
    #[doc(hidden)]
    pub async fn save_recording_with<W>(&self, write: W) -> Result<PathBuf, ArmError>
    where
        W: FnOnce(&Path, &[RecordFrame]) -> Result<(), ParquetRecordingError> + Send + 'static,
    {
        match self.stop_recording_as(Stopper::Owner(RecorderOwner::LegacyRest), true, None, Some(Box::new(write))).await? {
            Stopped::Saved { path, .. } => Ok(path),
            Stopped::Discarded => Err(ArmError::Internal("recording was discarded".into())),
        }
    }

    pub async fn discard_recording(&self) -> Result<(), ArmError> {
        self.stop_recording_as(Stopper::Owner(RecorderOwner::LegacyRest), false, None, None).await.map(|_| ())
    }

    pub async fn get_recording_state(&self) -> (EpisodeState, usize) {
        let inner = self.inner().read().await;
        (inner.recorder.state().clone(), inner.recorder.frame_count())
    }

    /// Set the episode directory (`None`: storage off); the catalog loads on first use.
    pub async fn set_episodes_dir(&self, dir: Option<PathBuf>) {
        self.inner().write().await.episodes = crate::recording::EpisodeStore::new(dir);
    }

    pub async fn episode_store(&self) -> crate::recording::EpisodeStore {
        self.inner().read().await.episodes.clone()
    }
}

async fn stop_recording(lock: &Arc<RwLock<SupervisorInner>>, by: Stopper, save: bool, outcome: Option<&str>, writer: Option<ParquetWriter>) -> Result<Stopped, ArmError> {
        let outcome = outcome.unwrap_or("ok");
        if !["ok", "aborted"].contains(&outcome) { return Err(rejected(400, "INVALID_COMMAND", "outcome must be \"ok\" or \"aborted\"")); }
        let monitor = matches!(by, Stopper::Monitor { .. });
        let st = {
            let inner = lock.read().await;
            let Some(r) = inner.recording.as_ref() else {
                return Err(crate::recording::RecordingError::InvalidState { action: if save { "save".into() } else { "discard".into() }, state: inner.recorder.state().clone() }.into());
            };
            if let Stopper::Owner(o) = &by {
                if o != &r.owner {
                    return Err(ArmError::detailed(409, "RECORDER_NOT_OWNER", format!("the recording is owned by '{}'", r.owner.label()), json!({"owner": r.owner.label()})));
                }
            }
            inner.episodes.clone()
        };
        if save && !monitor {
            // refused before the frames leave the recorder, so the recording can still be
            // saved after episodes were deleted, or discarded
            let s2 = st.clone();
            tokio::task::spawn_blocking(move || s2.check_capacity()).await
                .map_err(|e| ArmError::Internal(format!("episode check failed: {e}")))??;
        }
        let (rec, _path, frames, sidecar_base) = {
            let mut inner = lock.write().await;
            let Some(r) = inner.recording.clone() else {
                return Err(crate::recording::RecordingError::InvalidState { action: "save".into(), state: inner.recorder.state().clone() }.into());
            };
            if let Stopper::Owner(o) = &by { if o != &r.owner { return Err(rejected(409, "RECORDER_NOT_OWNER", "the recording changed owner")); } }
            if !save {
                inner.recorder.discard()?;
                inner.recording = None;
                tracing::info!("Recording {} discarded", r.id);
                return Ok(Stopped::Discarded);
            }
            let (path, frames) = inner.recorder.begin_save()?;
            inner.recording = None;
            let base = (
                inner.joint_names(),
                inner.config.profile_id.clone(),
                inner.config.arm_dof(),
                inner.config.gripper_index(),
                f64::from(inner.config.control_frequency_hz),
                inner.config.active_tool.id.clone(),
                inner.config.active_tool.config_digest.clone(),
            );
            (r, path, frames, base)
        };
        let (joint_names, profile_id, arm_dof, gripper_joint, fps, tool_id, tool_config_digest) = sidecar_base;
        let has_gripper_motion = store::episode_has_gripper_motion(&frames, gripper_joint);
        let duration = match (frames.first(), frames.last()) { (Some(a), Some(b)) => (b.timestamp - a.timestamp).max(0.0), _ => 0.0 };
        let (truncated, stopped_by) = match by { Stopper::Monitor { reason } => (reason == "max_duration", Some(reason.to_string())), _ => (false, None) };
        let sidecar = EpisodeSidecar {
            id: rec.id.clone(), dataset: rec.options.dataset.clone(), task: rec.options.task.clone(), source: rec.options.source.clone(),
            sequence_id: rec.options.sequence_id.clone(), repeat_index: rec.options.repeat_index, outcome: outcome.into(),
            recorded_at: rec.recorded_at.clone(), duration_s: duration, frames: frames.len(), joint_names, units: "rad".into(),
            joint_zero: rec.joint_zero.clone(), profile_id, armd_version: env!("CARGO_PKG_VERSION").into(), fps_nominal: fps,
            truncated, stopped_by, arm_dof: Some(arm_dof), gripper_joint, recovered: None,
            first_state: frames.first().map(|f| f.observation_state.clone()), last_state: frames.last().map(|f| f.observation_state.clone()),
            tool_id: Some(tool_id), tool_config_digest: Some(tool_config_digest), has_gripper_motion: Some(has_gripper_motion),
        };
        let sc = sidecar.clone();
        let (frames, result) = tokio::task::spawn_blocking(move || {
            let result = st.write_episode(&sc, &frames, writer);
            (frames, result)
        }).await.unwrap_or_else(|e| (Vec::new(), Err(StoreError::Io(std::io::Error::other(format!("recording writer panicked: {e}"))))));
        let parquet_result = match &result {
            Ok(_) => Ok(()),
            Err(StoreError::Parquet(e)) => Err(ParquetRecordingError::Corrupted(e.to_string())),
            Err(e) => Err(ParquetRecordingError::Io(std::io::Error::other(e.to_string()))),
        };
        lock.write().await.recorder.finish_save(frames, parquet_result)?;
        let path = result?;
        tracing::info!("Recording {} saved ({} frames, {:.1} s{})", rec.id, sidecar.frames, sidecar.duration_s, if truncated { ", truncated" } else { "" });
        Ok(Stopped::Saved { sidecar: Box::new(sidecar), path })
    }

/// Stops the recording `id` at the length cap or on low space, discards it after its owner
/// left. Ends when the recording ends.
async fn monitor(lock: Arc<RwLock<SupervisorInner>>, id: String) {
    let mut last_space: Option<Instant> = None;
    loop {
        tokio::time::sleep(MONITOR_PERIOD).await;
        let (full, lost, st) = {
            let inner = lock.read().await;
            let Some(r) = inner.recording.as_ref().filter(|r| r.id == id) else { return; };
            let full = inner.recorder.frame_count() >= r.max_frames || r.started.elapsed().as_secs_f64() >= inner.max_episode_s + 1.0;
            (full, r.owner_lost_at.is_some_and(|t| t.elapsed() >= OWNER_LOST_DISCARD), inner.episodes.clone())
        };
        let action = if lost {
            Some((false, "owner_lost"))
        } else if full {
            Some((true, "max_duration"))
        } else if last_space.is_none_or(|t| t.elapsed() >= SPACE_CHECK_PERIOD) {
            last_space = Some(Instant::now());
            let free = tokio::task::spawn_blocking(move || st.free_bytes()).await.ok().flatten();
            free.filter(|f| *f < store::STOP_MIN_FREE_BYTES).map(|_| (true, "low_space"))
        } else { None };
        let Some((save, reason)) = action else { continue; };
        // only this recording: a newer one is not touched
        if !lock.read().await.recording.as_ref().is_some_and(|r| r.id == id) { return; }
        match stop_recording(&lock, Stopper::Monitor { reason }, save, None, None).await {
            Ok(_) => tracing::warn!("Recording {id} {} by armd ({reason})", if save { "saved" } else { "discarded" }),
            Err(e) => tracing::warn!("Recording {id}: stop ({reason}) failed: {e}"),
        }
        return;
    }
}

// ---- lease-free WS commands ----

/// Teach commands that need no motion lease (recorder ownership or nothing).
pub fn is_lease_free(path: &str) -> bool {
    matches!(path, "/api/v1/record/start" | "/api/v1/record/stop" | "/api/v1/episodes/update" | "/api/v1/episodes/delete"
        | "/api/v1/datasets/create" | "/api/v1/datasets/delete")
}

fn str_field<'a>(body: &'a Value, key: &str) -> Result<Option<&'a str>, ArmError> {
    match body.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::String(s)) => Ok(Some(s.as_str())),
        Some(_) => Err(rejected(400, "INVALID_COMMAND", format!("{key} must be a string"))),
    }
}

fn required<'a>(body: &'a Value, key: &str) -> Result<&'a str, ArmError> {
    str_field(body, key)?.ok_or_else(|| rejected(400, "INVALID_COMMAND", format!("{key} is required")))
}

/// Who sent a lease-free command.
#[derive(Debug, Clone)]
pub struct Caller {
    /// Internal connection id (never disclosed).
    pub connection: String,
    /// `client_id` of the message, else the label of the lease this connection holds.
    pub client_id: Option<String>,
}

/// Run a lease-free command; Ok = fields of the ack.
pub async fn lease_free(sup: &Arc<ArmSupervisor>, caller: &Caller, path: &str, body: &Value) -> Result<Value, ArmError> {
    if !body.is_object() { return Err(rejected(400, "INVALID_COMMAND", "Command body must be an object")); }
    sup.ensure_episodes_loaded().await?;
    let st = sup.episode_store().await;
    if !st.enabled() { return Err(StoreError::Disabled.into()); }
    match path {
        "/api/v1/record/start" => {
            let client_id = caller.client_id.clone().filter(|c| !c.is_empty() && c.len() <= 128)
                .ok_or_else(|| rejected(400, "INVALID_COMMAND", "client_id (1-128 characters) is required: in the message, or from acquire_lease on this connection"))?;
            let options = RecordOptions {
                dataset: str_field(body, "dataset")?.unwrap_or(store::DEFAULT_DATASET).to_string(),
                task: str_field(body, "task")?.unwrap_or("").to_string(),
                source: str_field(body, "source")?.unwrap_or("sdk").to_string(),
                sequence_id: None, repeat_index: None,
            };
            let owner = RecorderOwner::Client { connection: caller.connection.clone(), client_id };
            let (id, owner) = sup.start_recording_as(owner, options, None, None).await?;
            Ok(json!({"recording_id": id, "owner": owner}))
        }
        "/api/v1/record/stop" => {
            let save = match body.get("save") {
                Some(Value::Bool(b)) => *b,
                _ => return Err(rejected(400, "INVALID_COMMAND", "save must be a boolean")),
            };
            let outcome = str_field(body, "outcome")?.map(str::to_string);
            let owner = sup.claim_recording(caller).await;
            match sup.stop_recording_as(Stopper::Owner(owner), save, outcome.as_deref(), None).await? {
                Stopped::Discarded => Ok(json!({"discarded": true})),
                Stopped::Saved { sidecar, .. } => {
                    let inner = sup.inner().read().await;
                    Ok(json!({"episode": inner.episodes.view(&sidecar.id, &inner.joint_zero_offsets)}))
                }
            }
        }
        "/api/v1/episodes/update" => {
            let id = required(body, "id")?.to_string();
            let task = str_field(body, "task")?.map(str::to_string);
            let dataset = str_field(body, "dataset")?.map(str::to_string);
            let offsets = sup.inner().read().await.joint_zero_offsets.clone();
            let s2 = st.clone();
            let sc = blocking(move || s2.update(&id, task.as_deref(), dataset.as_deref())).await?;
            let stale = sc.joint_zero.stale_against(&offsets);
            let mut v = serde_json::to_value(&sc).unwrap_or_default();
            v["stale"] = json!(stale);
            Ok(json!({"episode": v}))
        }
        "/api/v1/episodes/delete" => {
            let id = required(body, "id")?.to_string();
            {
                let inner = sup.inner().read().await;
                inner.episode_delete_allowed(&id)?;
                if !inner.episodes.take_for_delete(&id) { return Err(StoreError::NotFound(id).into()); }
            }
            let s2 = st.clone();
            let id2 = id.clone();
            blocking(move || s2.delete_files(&id2)).await?;
            Ok(json!({"deleted": id}))
        }
        "/api/v1/datasets/create" => {
            let name = required(body, "name")?.to_string();
            let description = str_field(body, "description")?.unwrap_or("").to_string();
            let s2 = st.clone();
            let d = blocking(move || s2.create_dataset(&name, &description)).await?;
            Ok(json!({"dataset": d}))
        }
        "/api/v1/datasets/delete" => {
            let name = required(body, "name")?.to_string();
            let force = match body.get("force") {
                None | Some(Value::Null) => false,
                Some(Value::Bool(b)) => *b,
                Some(_) => return Err(rejected(400, "INVALID_COMMAND", "force must be a boolean")),
            };
            let ids = st.episodes_of(&name);
            if force {
                let inner = sup.inner().read().await;
                for id in &ids { inner.episode_delete_allowed(id)?; }
            }
            if inner_recording_dataset(sup, &name).await {
                return Err(ArmError::detailed(409, "RECORDER_BUSY", "a recording into this dataset is running", json!({})));
            }
            let s2 = st.clone();
            let deleted = blocking(move || s2.delete_dataset(&name, force)).await?;
            Ok(json!({"deleted_episodes": deleted}))
        }
        _ => Err(rejected(400, "INVALID_COMMAND", "Unknown command")),
    }
}

async fn inner_recording_dataset(sup: &ArmSupervisor, name: &str) -> bool {
    sup.inner().read().await.recording.as_ref().is_some_and(|r| r.options.dataset == name)
}

async fn blocking<T: Send + 'static>(f: impl FnOnce() -> Result<T, StoreError> + Send + 'static) -> Result<T, ArmError> {
    tokio::task::spawn_blocking(f).await.map_err(|e| ArmError::Internal(format!("episode task failed: {e}")))?.map_err(ArmError::from)
}

impl SupervisorInner {
    /// Whether the episode `id` may be deleted now: IN_USE `{sequences}` when a sequence
    /// references it or the running sequence pinned it.
    pub fn episode_delete_allowed(&self, id: &str) -> Result<(), ArmError> {
        let users = self.sequences_using_episode(id);
        if users.is_empty() { return Ok(()); }
        Err(ArmError::detailed(409, "IN_USE", format!("Episode '{id}' is used by {} sequence(s)", users.len()), json!({"sequences": users})))
    }
}

/// `command_ack` / `command_reject` of a lease-free command.
pub fn reply(request_id: &Value, result: Result<Value, ArmError>) -> Value {
    match result {
        Ok(fields) => {
            let mut ack = json!({"type":"command_ack", "request_id":request_id, "status":"accepted"});
            if let Value::Object(m) = fields { for (k, v) in m { ack[k] = v; } }
            ack
        }
        Err(e) => {
            let api = crate::api::rest::ApiError::from(e);
            let mut r = json!({"type":"command_reject", "request_id":request_id, "status":api.status.as_u16(),
                "code":api.code, "error":api.error, "message":api.message});
            if let Some(Value::Object(m)) = api.details.map(|d| *d) { for (k, v) in m { r[k] = v; } }
            r
        }
    }
}
