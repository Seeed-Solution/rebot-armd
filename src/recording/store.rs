//! Episode store (teach v1): immutable episodes, one `<id>.parquet` + `<id>.json` sidecar
//! each, grouped into datasets by `datasets.json`.
//!
//! - New episodes get a ULID id (26 characters, Crockford base32). Episodes recorded before the
//!   store (legacy REST ids `^[A-Za-z0-9_-]{1,64}$`) keep their id.
//! - Write order: `<id>.parquet.tmp` -> fsync -> rename to `<id>.parquet`, then the sidecar
//!   (temp + rename). The parquet footer also carries the sidecar (key `rebot.episode`).
//! - Boot recovery ([`EpisodeStore::load`]): see [`RecoveryReport`] for the exact rule.
//! - The store keeps an in-memory catalog of every sidecar, so listing, reference checks and
//!   stale flags need no file I/O; file I/O runs on blocking threads, never under the
//!   supervisor lock.
use std::collections::{BTreeMap, HashMap};
use std::path::{Component, Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime};

use serde::{Deserialize, Serialize};
use thiserror::Error;

use super::episode::RecordFrame;
use super::parquet_writer::{read_frames_from_parquet, read_metadata_from_parquet, read_parquet_kv, write_episode_parquet, EpisodeMetadata, ParquetRecordingError};

/// Default episode directory (used when `/userdata` exists).
pub const DEFAULT_EPISODES_DIR: &str = "/userdata/rebot-teleop/episodes";
/// Episode directory when `/userdata` does not exist (relative to armd's working directory).
pub const FALLBACK_EPISODES_DIR: &str = "./data/episodes";
/// Environment variable read when `--episodes-dir` is not given.
pub const EPISODES_DIR_ENV: &str = "ARMD_EPISODES_DIR";
/// Starting a recording is refused while the directory holds this many bytes.
pub const MAX_EPISODES_BYTES: u64 = 200 * 1024 * 1024;
/// Starting a recording is refused while the directory holds this many episodes.
pub const MAX_EPISODE_FILES: usize = 500;
pub const EPISODE_EXT: &str = "parquet";
pub const SIDECAR_EXT: &str = "json";
pub const DATASETS_FILE: &str = "datasets.json";
pub const DEFAULT_DATASET: &str = "default";
pub const MAX_EPISODE_ID_LEN: usize = 64;
/// Longest episode (s): a recording stops (saved, `truncated`) at this length; imports above it
/// are refused.
pub const MAX_EPISODE_S: f64 = 600.0;
/// A recording does not start with less free space on the episode file system (bytes).
pub const START_MIN_FREE_BYTES: u64 = 200 * 1024 * 1024;
/// A running recording stops (saved) below this free space (bytes).
pub const STOP_MIN_FREE_BYTES: u64 = 100 * 1024 * 1024;
/// Boot recovery deletes an unreadable ULID episode without sidecar only when older than this.
pub const ORPHAN_AGE: Duration = Duration::from_secs(3600);
/// Parquet key-value entry holding the sidecar JSON (sidecar recovery).
pub const PARQUET_SIDECAR_KEY: &str = "rebot.episode";
/// `source` values of a sidecar.
pub const EPISODE_SOURCES: [&str; 7] = ["hand_guide", "jog", "servo", "replay", "sequence", "import", "sdk"];

const CROCKFORD: &[u8; 32] = b"0123456789ABCDEFGHJKMNPQRSTVWXYZ";

/// `^[A-Za-z0-9_-]{1,64}$`: every episode id (ULIDs and legacy ids).
pub fn valid_episode_id(id: &str) -> bool {
    !id.is_empty() && id.len() <= MAX_EPISODE_ID_LEN && id.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-')
}

/// 26 Crockford base32 characters (upper case), first one 0-7.
pub fn is_ulid(id: &str) -> bool {
    id.len() == 26 && id.bytes().all(|b| CROCKFORD.contains(&b)) && id.as_bytes()[0] <= b'7'
}

/// A new ULID: 48-bit Unix ms + 80 random bits; strictly increasing within this process.
pub fn new_ulid() -> String {
    use std::hash::{BuildHasher, Hasher};
    static LAST: Mutex<u128> = Mutex::new(0);
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    let ms = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).map_or(0, |d| d.as_millis() as u64) & ((1 << 48) - 1);
    let random = || {
        let mut h = std::collections::hash_map::RandomState::new().build_hasher();
        h.write_u64(COUNTER.fetch_add(1, Ordering::Relaxed));
        h.write_u128(SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).map_or(0, |d| d.as_nanos()));
        h.finish()
    };
    let rand80 = ((u128::from(random()) << 64) | u128::from(random())) & ((1u128 << 80) - 1);
    let mut value = (u128::from(ms) << 80) | rand80;
    let mut last = LAST.lock().unwrap_or_else(|e| e.into_inner());
    if value <= *last { value = *last + 1; }
    *last = value;
    (0..26).map(|i| CROCKFORD[((value >> (125 - 5 * i)) & 0x1f) as usize] as char).collect()
}

/// Fingerprint of joint zero offsets (FNV-1a 64 over the f32 bits, 16 hex digits): equal offsets,
/// equal revision, across restarts.
pub fn zero_rev(offsets: &[f32]) -> String {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for o in offsets {
        for b in o.to_bits().to_le_bytes() { h ^= u64::from(b); h = h.wrapping_mul(0x0100_0000_01b3); }
    }
    format!("{h:016x}")
}

/// Display names of datasets and sequences: `^[A-Za-z0-9_\- \p{Han}]{1,64}$` (64 characters).
pub fn valid_display_name(name: &str) -> bool {
    let n = name.chars().count();
    (1..=64).contains(&n) && name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-' || c == ' ' || is_han(c))
}

fn is_han(c: char) -> bool {
    matches!(c as u32, 0x3400..=0x4DBF | 0x4E00..=0x9FFF | 0xF900..=0xFAFF | 0x20000..=0x2A6DF | 0x2A700..=0x2EBEF | 0x30000..=0x3134F | 0x2F800..=0x2FA1F | 0x3005 | 0x3007 | 0x3021..=0x3029)
}

/// Return true when recorded frames command or observe movement on `gripper_joint`.
pub fn episode_has_gripper_motion(frames: &[RecordFrame], gripper_joint: Option<usize>) -> bool {
    let Some(g) = gripper_joint else { return false; };
    let mut obs_min = f32::INFINITY;
    let mut obs_max = f32::NEG_INFINITY;
    let mut act_min = f32::INFINITY;
    let mut act_max = f32::NEG_INFINITY;
    for f in frames {
        let obs = f.observation_state.get(g).copied().filter(|v| v.is_finite());
        let act = f.action.get(g).copied().filter(|v| v.is_finite());
        if let Some(v) = obs {
            obs_min = obs_min.min(v);
            obs_max = obs_max.max(v);
        }
        if let Some(v) = act {
            act_min = act_min.min(v);
            act_max = act_max.max(v);
        }
        if let (Some(o), Some(a)) = (obs, act) {
            if (a - o).abs() > 1e-3 {
                return true;
            }
        }
    }
    (obs_max - obs_min) > 1e-3 || (act_max - act_min) > 1e-3
}

/// `sidecar.joint_zero`: the joint zero in effect at capture.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct JointZeroStamp {
    pub offsets_rad: Vec<f64>,
    pub rev: String,
}

impl JointZeroStamp {
    pub fn of(offsets: &[f32]) -> Self {
        Self { offsets_rad: offsets.iter().map(|o| f64::from(*o)).collect(), rev: zero_rev(offsets) }
    }

    /// A joint zero moved by more than [`crate::joint_zero::STALE_THRESHOLD_RAD`] since capture.
    pub fn stale_against(&self, current: &[f32]) -> bool {
        self.offsets_rad.len() != current.len()
            || self.offsets_rad.iter().zip(current).any(|(a, b)| (a - f64::from(*b)).abs() > crate::joint_zero::STALE_THRESHOLD_RAD)
    }
}

/// `<id>.json`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EpisodeSidecar {
    pub id: String,
    pub dataset: String,
    #[serde(default)]
    pub task: String,
    pub source: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sequence_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub repeat_index: Option<u32>,
    /// `ok` | `aborted`.
    pub outcome: String,
    /// RFC 3339 UTC time of the start of the recording.
    pub recorded_at: String,
    pub duration_s: f64,
    pub frames: usize,
    pub joint_names: Vec<String>,
    /// Always `rad`.
    pub units: String,
    pub joint_zero: JointZeroStamp,
    #[serde(default)]
    pub profile_id: Option<String>,
    pub armd_version: String,
    /// Control rate of the capture (Hz); timestamps are the measured samples, not resampled.
    pub fps_nominal: f64,
    /// Stopped at [`MAX_EPISODE_S`].
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub truncated: bool,
    /// Why the recording stopped when armd stopped it: `max_duration` | `low_space`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stopped_by: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub arm_dof: Option<usize>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gripper_joint: Option<usize>,
    /// Set by boot recovery: `sidecar` (rebuilt from the parquet footer) or `legacy` (an episode
    /// recorded before the store, adopted with the joint zero of the boot).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub recovered: Option<String>,
    /// First and last `observation.state` (rad): the approach of a replay step and where it
    /// ends, without reading the parquet (sequence duration bound).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub first_state: Option<Vec<f32>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub last_state: Option<Vec<f32>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tool_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tool_config_digest: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub has_gripper_motion: Option<bool>,
    /// Samples the control tick dropped on a full recording ring (the episode is incomplete);
    /// absent when none was dropped.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub frames_dropped: Option<u64>,
}

impl EpisodeSidecar {
    pub fn metadata(&self) -> EpisodeMetadata {
        EpisodeMetadata {
            profile_id: self.profile_id.clone(),
            arm_dof: self.arm_dof,
            gripper_joint: self.gripper_joint,
            joint_names: Some(self.joint_names.clone()),
            tool_id: self.tool_id.clone(),
            tool_config_digest: self.tool_config_digest.clone(),
            has_gripper_motion: self.has_gripper_motion,
        }
    }
}

/// An episode in the API: the sidecar, `stale` against the current joint zero, file size.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct EpisodeView {
    #[serde(flatten)]
    pub sidecar: EpisodeSidecar,
    pub stale: bool,
    pub bytes: u64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DatasetInfo {
    pub created_at: String,
    #[serde(default)]
    pub description: String,
}

/// `datasets.json`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct DatasetsManifest {
    #[serde(default)]
    pub datasets: BTreeMap<String, DatasetInfo>,
}

/// `GET /api/v1/datasets` entry.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct DatasetSummary {
    pub name: String,
    pub episodes: usize,
    pub duration_s: f64,
    pub bytes: u64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub created_at: Option<String>,
    pub description: String,
}

/// What [`EpisodeStore::load`] did. Rule, per file in the episode directory:
/// 1. a name containing `.tmp` (unfinished parquet / sidecar / manifest write) is deleted;
/// 2. `<id>.parquet` with a readable `<id>.json` is listed;
/// 3. `<id>.parquet` without a (readable) sidecar whose footer carries the sidecar
///    ([`PARQUET_SIDECAR_KEY`], a crash between the parquet rename and the sidecar write)
///    gets the sidecar back (`recovered: "sidecar"`);
/// 4. otherwise, a readable parquet with a legacy id is adopted (`recovered: "legacy"`,
///    dataset `default`, joint zero of the boot);
/// 5. otherwise (unreadable): a ULID-named file older than [`ORPHAN_AGE`] is deleted, any
///    other file is left in place and not listed;
/// 6. `<id>.json` without `<id>.parquet` is deleted.
#[derive(Debug, Clone, Default, PartialEq, Serialize)]
pub struct RecoveryReport {
    pub episodes: usize,
    pub temps_removed: Vec<String>,
    pub sidecars_rebuilt: Vec<String>,
    pub legacy_adopted: Vec<String>,
    pub orphans_removed: Vec<String>,
    pub orphans_kept: Vec<String>,
    pub sidecars_removed: Vec<String>,
}

#[derive(Debug, Error)]
pub enum StoreError {
    #[error("Episode storage is off (--episodes-dir off)")]
    Disabled,
    #[error("Invalid episode id '{0}': use 1-64 characters from A-Z a-z 0-9 _ -")]
    InvalidId(String),
    #[error("Invalid output_path '{0}': it must name <id>.parquet directly inside the episode directory {1}")]
    PathOutside(String, String),
    #[error("Episode '{0}' not found")]
    NotFound(String),
    #[error("Episode '{0}' already exists")]
    Exists(String),
    #[error("Episode directory is full: {files} files, {bytes} bytes (limits {max_files} files, {max_bytes} bytes); delete episodes first")]
    Full { files: usize, bytes: u64, max_files: usize, max_bytes: u64 },
    #[error("Only {free} bytes free on the episode file system; a recording needs {need}")]
    NoSpace { free: u64, need: u64 },
    #[error("Dataset '{0}' not found")]
    DatasetNotFound(String),
    #[error("Dataset '{0}' already exists")]
    DatasetExists(String),
    #[error("Dataset '{0}' holds {1} episode(s); delete them first or pass force")]
    DatasetNotEmpty(String, usize),
    #[error("a recording into this dataset is running")]
    DatasetRecordingBusy(String),
    #[error("Invalid dataset name '{0}': 1-64 characters from letters, digits, _ - space and Han characters")]
    InvalidDataset(String),
    #[error("{0}")]
    Invalid(String),
    #[error("Episode directory I/O error: {0}")]
    Io(#[from] std::io::Error),
    #[error("Episode file error: {0}")]
    Parquet(#[from] ParquetRecordingError),
}

impl StoreError {
    /// (HTTP status, code) of the rejection.
    pub fn status_code(&self) -> (u16, &'static str) {
        match self {
            Self::Disabled => (409, "EPISODES_DISABLED"),
            Self::InvalidId(_) => (400, "INVALID_EPISODE_ID"),
            Self::PathOutside(..) => (400, "INVALID_OUTPUT_PATH"),
            Self::NotFound(_) => (404, "EPISODE_NOT_FOUND"),
            Self::Exists(_) => (409, "EPISODE_EXISTS"),
            Self::Full { .. } => (507, "EPISODES_FULL"),
            Self::NoSpace { .. } => (507, "RECORD_NO_SPACE"),
            Self::DatasetNotFound(_) => (404, "DATASET_NOT_FOUND"),
            Self::DatasetExists(_) => (409, "DATASET_EXISTS"),
            Self::DatasetNotEmpty(..) => (409, "DATASET_NOT_EMPTY"),
            Self::DatasetRecordingBusy(_) => (409, "RECORDER_BUSY"),
            Self::InvalidDataset(_) | Self::Invalid(_) => (400, "INVALID_COMMAND"),
            Self::Io(_) | Self::Parquet(_) => (500, "EPISODES_IO_ERROR"),
        }
    }
}

/// One saved episode in the legacy `GET /api/v1/dataset/episodes`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EpisodeInfo {
    pub id: String,
    pub frames: usize,
    /// Last minus first timestamp (s).
    pub duration_s: f64,
    pub size_bytes: u64,
    /// Start of the recording (Unix ms).
    pub created_ms: u64,
    /// Joints per frame (`observation.state` length).
    pub dof: usize,
    /// Per joint: minimum and maximum of `observation.state` over the episode (rad).
    pub joint_min: Vec<f32>,
    pub joint_max: Vec<f32>,
    /// Set when the file could not be read; the other fields are then zero / empty.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    /// teach v1 fields (dataset, task, source, stale) for clients that read the legacy list.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dataset: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub task: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source: Option<String>,
    #[serde(default)]
    pub stale: bool,
}

/// Legacy `GET /api/v1/dataset/episodes` body.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EpisodeList {
    pub enabled: bool,
    pub dir: Option<String>,
    pub episodes: Vec<EpisodeInfo>,
    pub total_bytes: u64,
    pub max_bytes: u64,
    pub max_files: usize,
}

#[derive(Debug, Clone)]
struct Entry { sidecar: EpisodeSidecar, bytes: u64 }

#[derive(Debug, Clone)]
struct InProgressClaim {
    token: Option<u64>,
    sidecar: EpisodeSidecar,
}

#[derive(Debug, Clone)]
struct CompletedClaim {
    token: u64,
    sidecar: EpisodeSidecar,
}

fn next_delete_claim() -> u64 {
    static NEXT_DELETE_CLAIM: AtomicU64 = AtomicU64::new(1);
    NEXT_DELETE_CLAIM.fetch_add(1, Ordering::Relaxed)
}

#[derive(Debug, Default)]
struct Catalog {
    loaded: bool,
    episodes: HashMap<String, Entry>,
    datasets: DatasetsManifest,
    /// Reference count of episodes currently pinned by an in-flight `ConfigDomain::Sequences`
    /// reservation (or post-rename `Reconciling` state). Prevents concurrent episode deletion
    /// without acquiring any lock on `armd-control`.
    pinned_episodes: HashMap<String, usize>,
    /// Reference count of datasets currently targeted by a starting, active, or finalizing
    /// recording or import. Blocks concurrent dataset deletion (`409 RECORDER_BUSY`).
    pinned_datasets: HashMap<String, usize>,
    /// Datasets currently undergoing atomic force-deletion (blocks concurrent recording/import
    /// pins, episode moves, and overlapping force-deletes).
    deleting_datasets: std::collections::HashSet<String>,
    /// Remaining un-deleted `(episode_id, delete_claim)` pairs from a force-delete that
    /// encountered an unlink error, preserved across retries so a retry validates exact claim
    /// ownership and still removes stranded parquet/sidecar files.
    pending_delete_files: HashMap<String, Vec<(String, u64)>>,
    /// Per-episode deletion claims (`episode_id -> delete_claim_token`) active from catalog
    /// removal until all on-disk files are unlinked (or through failed-unlink retry). Blocks
    /// concurrent `get_or_adopt`, `new_episode`, `publish_complete_episode`, and stale unlinks.
    deleting_episodes: HashMap<String, u64>,
    /// Reserved capacity slots held by in-flight manual saves and imports between precheck and
    /// publication Phase 1 claim so concurrent writers cannot race past `MAX_EPISODE_FILES`.
    reserved_slots: usize,
    /// Operation commit tokens + immutable sidecar of completed episode publications.
    commit_tokens: HashMap<String, CompletedClaim>,
    /// In-flight episode publications (`id -> InProgressClaim`): prevents concurrent writers,
    /// raw deletion, and replay adoption from racing with an uncommitted publication, and binds
    /// post-parquet-rename recovery strictly to the original `(commit_token, sidecar)`.
    in_progress_tokens: HashMap<String, InProgressClaim>,
}

/// RAII pin guard holding a dataset in [`Catalog::pinned_datasets`] across a recording's
/// start-reservation, active, and finalizing lifecycle (or an episode import).
#[derive(Debug)]
pub struct DatasetPinGuard {
    catalog: Arc<Mutex<Catalog>>,
    dataset: String,
}

impl Drop for DatasetPinGuard {
    fn drop(&mut self) {
        if self.dataset.is_empty() {
            return;
        }
        let mut cat = self.catalog.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(count) = cat.pinned_datasets.get_mut(&self.dataset) {
            if *count <= 1 {
                cat.pinned_datasets.remove(&self.dataset);
            } else {
                *count -= 1;
            }
        }
    }
}

/// RAII guard marking a dataset as mid-deletion so new recordings/imports cannot acquire a pin
/// between catalog removal and on-disk manifest/file cleanup.
#[derive(Debug)]
pub struct DatasetDeleteGuard {
    catalog: Arc<Mutex<Catalog>>,
    dataset: String,
}

impl Drop for DatasetDeleteGuard {
    fn drop(&mut self) {
        if self.dataset.is_empty() {
            return;
        }
        let mut cat = self.catalog.lock().unwrap_or_else(|e| e.into_inner());
        cat.deleting_datasets.remove(&self.dataset);
    }
}

/// RAII capacity slot reservation held by an in-flight manual save or import so concurrent
/// writers cannot steal the last slot or race past `MAX_EPISODE_FILES`.
#[derive(Debug)]
pub struct CapacitySlotGuard {
    catalog: Arc<Mutex<Catalog>>,
    active: bool,
}

impl CapacitySlotGuard {
    /// Atomically releases this reserved slot under an already-held `Catalog` lock when the
    /// caller inserts its corresponding `in_progress_tokens` claim (preventing double-counting).
    fn release_locked(&mut self, cat: &mut Catalog) {
        if self.active {
            self.active = false;
            cat.reserved_slots = cat.reserved_slots.saturating_sub(1);
        }
    }
}

impl Drop for CapacitySlotGuard {
    fn drop(&mut self) {
        if !self.active {
            return;
        }
        let mut cat = self.catalog.lock().unwrap_or_else(|e| e.into_inner());
        cat.reserved_slots = cat.reserved_slots.saturating_sub(1);
    }
}

/// RAII pin guard holding episode IDs in [`Catalog::pinned_episodes`] across a `Sequences` commit
/// reservation. Drops cleanly on `Cleared` or post-activation; call [`Self::keep_pinned_for_reconcile`]
/// on post-rename `Reconciling` so on-disk sequence references stay protected.
pub struct EpisodePinGuard {
    catalog: Arc<Mutex<Catalog>>,
    ids: Vec<String>,
}

impl EpisodePinGuard {
    /// Retains the episode pins in the catalog when the commit enters post-rename `Reconciling`.
    pub fn keep_pinned_for_reconcile(mut self) {
        self.ids.clear();
    }
}

impl Drop for EpisodePinGuard {
    fn drop(&mut self) {
        if self.ids.is_empty() {
            return;
        }
        let mut cat = self.catalog.lock().unwrap_or_else(|e| e.into_inner());
        for id in &self.ids {
            if let Some(count) = cat.pinned_episodes.get_mut(id) {
                if *count <= 1 {
                    cat.pinned_episodes.remove(id);
                } else {
                    *count -= 1;
                }
            }
        }
    }
}

/// Atomic outcome of [`EpisodeStore::take_if_unpinned_and_rev_matches`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TakeEpisodeOutcome {
    Removed(u64),
    NotFound,
    Pinned,
    SequenceRevisionChanged,
}

/// Atomic outcome of [`EpisodeStore::take_dataset_if_unpinned_and_rev_matches`].
#[derive(Debug)]
pub enum TakeDatasetOutcome {
    Removed(Vec<(String, u64)>, DatasetDeleteGuard),
    Pinned(String),
    DatasetRecordingBusy,
    SequenceRevisionChanged,
}

type SummaryCache = Arc<Mutex<HashMap<String, (SystemTime, u64, EpisodeInfo)>>>;

type PublishPauseHook = (std::sync::mpsc::SyncSender<String>, std::sync::mpsc::Receiver<()>);

/// Episode directory and its catalog; `dir == None` means storage is off. Clones share the
/// catalog.
#[derive(Debug, Clone, Default)]
pub struct EpisodeStore {
    dir: Option<PathBuf>,
    cache: SummaryCache,
    catalog: Arc<Mutex<Catalog>>,
    /// Test hook: free bytes reported instead of the file system's.
    free_override: Arc<Mutex<Option<u64>>>,
    /// Test hook: optional one-shot pause inside Phase 2 of `publish_complete_episode`.
    publish_pause: Arc<Mutex<Option<PublishPauseHook>>>,
    /// Serialises sidecar / manifest rewrites.
    write_lock: Arc<Mutex<()>>,
}

fn now_rfc3339() -> String { chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Millis, true) }

fn recorded_ms(s: &str) -> u64 {
    chrono::DateTime::parse_from_rfc3339(s).map_or(0, |t| t.timestamp_millis().max(0) as u64)
}

fn is_temp_name(name: &str) -> bool { name.contains(".tmp") }

impl EpisodeStore {
    pub fn new(dir: Option<PathBuf>) -> Self { Self { dir, ..Default::default() } }

    /// `--episodes-dir` value, else `ARMD_EPISODES_DIR`, else [`DEFAULT_EPISODES_DIR`] when
    /// `/userdata` exists, else [`FALLBACK_EPISODES_DIR`]. `off`, `none` or empty: storage off.
    pub fn resolve(cli: Option<&str>) -> Self {
        let value = cli.map(str::to_string).or_else(|| std::env::var(EPISODES_DIR_ENV).ok());
        let dir = match value.as_deref().map(str::trim) {
            Some("" | "off" | "none") => None,
            Some(v) => Some(PathBuf::from(v)),
            None if Path::new("/userdata").is_dir() => Some(PathBuf::from(DEFAULT_EPISODES_DIR)),
            None => Some(PathBuf::from(FALLBACK_EPISODES_DIR)),
        };
        Self::new(dir)
    }

    pub fn dir(&self) -> Option<&Path> { self.dir.as_deref() }

    pub fn enabled(&self) -> bool { self.dir.is_some() }

    fn require_dir(&self) -> Result<&Path, StoreError> { self.dir.as_deref().ok_or(StoreError::Disabled) }

    fn cat(&self) -> std::sync::MutexGuard<'_, Catalog> { self.catalog.lock().unwrap_or_else(|e| e.into_inner()) }

    /// `<dir>/<id>.parquet` for a valid `id`.
    pub fn path_of(&self, id: &str) -> Result<PathBuf, StoreError> {
        let dir = self.require_dir()?;
        if !valid_episode_id(id) { return Err(StoreError::InvalidId(id.into())); }
        Ok(dir.join(format!("{id}.{EPISODE_EXT}")))
    }

    fn sidecar_path(dir: &Path, id: &str) -> PathBuf { dir.join(format!("{id}.{SIDECAR_EXT}")) }

    // ---- boot recovery and catalog ----

    /// Boot recovery and catalog load (blocking). `offsets` = the joint zero at boot (legacy
    /// adoption). Rule: [`RecoveryReport`].
    pub fn load(&self, offsets: &[f32], joint_names: &[String], profile_id: Option<&str>, fps: f64) -> Result<RecoveryReport, StoreError> {
        let mut report = RecoveryReport::default();
        let Some(dir) = self.dir.as_deref() else { return Ok(report); };
        let mut episodes = HashMap::new();
        let entries = match std::fs::read_dir(dir) {
            Ok(e) => e.collect::<Result<Vec<_>, _>>()?,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Vec::new(),
            Err(e) => return Err(e.into()),
        };
        let mut parquet_ids = Vec::new();
        let mut sidecar_ids = Vec::new();
        for entry in &entries {
            let path = entry.path();
            let Some(name) = path.file_name().and_then(|n| n.to_str()).map(str::to_string) else { continue; };
            if !entry.file_type().map(|t| t.is_file()).unwrap_or(false) { continue; }
            if is_temp_name(&name) {
                match std::fs::remove_file(&path) {
                    Ok(()) => report.temps_removed.push(name),
                    Err(e) => tracing::warn!("Cannot remove {}: {e}", path.display()),
                }
                continue;
            }
            let ext = path.extension().and_then(|e| e.to_str());
            let Some(stem) = path.file_stem().and_then(|s| s.to_str()).filter(|s| valid_episode_id(s)) else { continue; };
            match ext {
                Some(EPISODE_EXT) => parquet_ids.push(stem.to_string()),
                Some(SIDECAR_EXT) if name != DATASETS_FILE => sidecar_ids.push(stem.to_string()),
                _ => {}
            }
        }
        for id in &parquet_ids {
            let ppath = dir.join(format!("{id}.{EPISODE_EXT}"));
            let bytes = std::fs::metadata(&ppath).map_or(0, |m| m.len());
            let spath = Self::sidecar_path(dir, id);
            if let Some(sc) = std::fs::read_to_string(&spath).ok().and_then(|t| serde_json::from_str::<EpisodeSidecar>(&t).ok()).filter(|s| &s.id == id) {
                episodes.insert(id.clone(), Entry { sidecar: sc, bytes });
                continue;
            }
            // no (readable) sidecar
            let embedded = read_parquet_kv(&ppath).ok()
                .and_then(|kv| kv.get(PARQUET_SIDECAR_KEY).cloned())
                .and_then(|t| serde_json::from_str::<EpisodeSidecar>(&t).ok())
                .filter(|s| &s.id == id);
            if let Some(mut sc) = embedded {
                sc.recovered = Some("sidecar".into());
                Self::write_sidecar_file(dir, &sc)?;
                report.sidecars_rebuilt.push(id.clone());
                episodes.insert(id.clone(), Entry { sidecar: sc, bytes });
                continue;
            }
            if !is_ulid(id) {
                if let Some(sc) = adopt_legacy(&ppath, id, offsets, joint_names, profile_id, fps) {
                    Self::write_sidecar_file(dir, &sc)?;
                    report.legacy_adopted.push(id.clone());
                    episodes.insert(id.clone(), Entry { sidecar: sc, bytes });
                    continue;
                }
                report.orphans_kept.push(id.clone());
                continue;
            }
            let age = std::fs::metadata(&ppath).and_then(|m| m.modified()).ok()
                .and_then(|t| SystemTime::now().duration_since(t).ok()).unwrap_or_default();
            if age > ORPHAN_AGE {
                std::fs::remove_file(&ppath)?;
                report.orphans_removed.push(id.clone());
            } else {
                report.orphans_kept.push(id.clone());
            }
        }
        for id in sidecar_ids.iter().filter(|id| !parquet_ids.contains(id)) {
            let spath = Self::sidecar_path(dir, id);
            // only files that parse as a sidecar: other JSON files are left alone
            if std::fs::read_to_string(&spath).ok().and_then(|t| serde_json::from_str::<EpisodeSidecar>(&t).ok()).is_some() {
                std::fs::remove_file(&spath)?;
                report.sidecars_removed.push(id.clone());
            }
        }
        let datasets = match std::fs::read_to_string(dir.join(DATASETS_FILE)) {
            Ok(t) => serde_json::from_str::<DatasetsManifest>(&t).unwrap_or_else(|e| {
                tracing::warn!("Ignoring {}: {e}", dir.join(DATASETS_FILE).display());
                DatasetsManifest::default()
            }),
            Err(_) => DatasetsManifest::default(),
        };
        report.episodes = episodes.len();
        let mut cat = self.cat();
        // entries written while the directory was scanned (a concurrent first use) are kept
        for (id, e) in episodes { cat.episodes.entry(id).or_insert(e); }
        if !cat.loaded { cat.datasets = datasets; }
        cat.loaded = true;
        Ok(report)
    }

    pub fn is_loaded(&self) -> bool { self.cat().loaded }

    fn write_sidecar_file(dir: &Path, sc: &EpisodeSidecar) -> Result<(), StoreError> {
        let text = serde_json::to_string_pretty(sc).map_err(|e| StoreError::Invalid(e.to_string()))?;
        crate::persist::write_atomic(&Self::sidecar_path(dir, &sc.id), &text)?;
        Ok(())
    }

    fn write_manifest(&self, dir: &Path, manifest: &DatasetsManifest) -> Result<(), StoreError> {
        let text = serde_json::to_string_pretty(manifest).map_err(|e| StoreError::Invalid(e.to_string()))?;
        crate::persist::write_atomic(&dir.join(DATASETS_FILE), &text)?;
        Ok(())
    }

    /// The sidecar of `id` (catalog only, no I/O).
    pub fn get(&self, id: &str) -> Option<EpisodeSidecar> { self.cat().episodes.get(id).map(|e| e.sidecar.clone()) }

    pub fn contains(&self, id: &str) -> bool { self.cat().episodes.contains_key(id) }

    /// [`get`](Self::get), else (episode files written next to the store, tests and tools) adopt
    /// `<id>.parquet` like boot recovery does (blocking). Never adopts an in-progress publication
    /// or an episode undergoing deletion.
    pub fn get_or_adopt(&self, id: &str, offsets: &[f32], joint_names: &[String], profile_id: Option<&str>, fps: f64) -> Result<EpisodeSidecar, StoreError> {
        {
            let cat = self.cat();
            if let Some(e) = cat.episodes.get(id) {
                return Ok(e.sidecar.clone());
            }
            if cat.in_progress_tokens.contains_key(id) || cat.deleting_episodes.contains_key(id) {
                return Err(StoreError::NotFound(id.into()));
            }
        }
        let path = self.path_of(id)?;
        let dir = self.require_dir()?;
        if !path.is_file() { return Err(StoreError::NotFound(id.into())); }
        let sc = read_parquet_kv(&path).ok().and_then(|kv| kv.get(PARQUET_SIDECAR_KEY).cloned())
            .and_then(|t| serde_json::from_str::<EpisodeSidecar>(&t).ok()).filter(|s| s.id == id)
            .or_else(|| adopt_legacy(&path, id, offsets, joint_names, profile_id, fps))
            .ok_or_else(|| StoreError::Invalid(format!("episode '{id}' cannot be read")))?;
        let _w = self.write_lock.lock().unwrap_or_else(|e| e.into_inner());
        {
            let cat = self.cat();
            if let Some(e) = cat.episodes.get(id) {
                return Ok(e.sidecar.clone());
            }
            if cat.in_progress_tokens.contains_key(id) || cat.deleting_episodes.contains_key(id) {
                return Err(StoreError::NotFound(id.into()));
            }
        }
        if !path.is_file() {
            return Err(StoreError::NotFound(id.into()));
        }
        let bytes = std::fs::metadata(&path)?.len();
        Self::write_sidecar_file(dir, &sc)?;
        self.cat().episodes.insert(id.into(), Entry { sidecar: sc.clone(), bytes });
        Ok(sc)
    }

    /// Episodes, newest first (`recorded_at`, then id). `before`: only those after the episode
    /// with this id in that order. `limit`: at most this many.
    pub fn list_episodes(&self, dataset: Option<&str>, limit: Option<usize>, before: Option<&str>, offsets: &[f32]) -> Vec<EpisodeView> {
        let cat = self.cat();
        let mut all: Vec<&Entry> = cat.episodes.values().filter(|e| dataset.is_none_or(|d| e.sidecar.dataset == d)).collect();
        all.sort_by(|a, b| b.sidecar.recorded_at.cmp(&a.sidecar.recorded_at).then_with(|| b.sidecar.id.cmp(&a.sidecar.id)));
        let start = before.and_then(|b| all.iter().position(|e| e.sidecar.id == b).map(|i| i + 1)).unwrap_or(if before.is_some() { all.len() } else { 0 });
        all.into_iter().skip(start).take(limit.unwrap_or(usize::MAX))
            .map(|e| EpisodeView { stale: e.sidecar.joint_zero.stale_against(offsets), sidecar: e.sidecar.clone(), bytes: e.bytes })
            .collect()
    }

    pub fn view(&self, id: &str, offsets: &[f32]) -> Option<EpisodeView> {
        self.cat().episodes.get(id).map(|e| EpisodeView { stale: e.sidecar.joint_zero.stale_against(offsets), sidecar: e.sidecar.clone(), bytes: e.bytes })
    }

    /// Every dataset of the manifest plus `default` and any name an episode carries.
    pub fn datasets(&self) -> Vec<DatasetSummary> {
        let cat = self.cat();
        let mut out: BTreeMap<String, DatasetSummary> = BTreeMap::new();
        let mut add = |name: &str, info: Option<&DatasetInfo>| {
            out.entry(name.to_string()).or_insert_with(|| DatasetSummary {
                name: name.into(), episodes: 0, duration_s: 0.0, bytes: 0,
                created_at: info.map(|i| i.created_at.clone()), description: info.map(|i| i.description.clone()).unwrap_or_default(),
            });
        };
        add(DEFAULT_DATASET, cat.datasets.datasets.get(DEFAULT_DATASET));
        for (name, info) in &cat.datasets.datasets { add(name, Some(info)); }
        for e in cat.episodes.values() { add(&e.sidecar.dataset, None); }
        for e in cat.episodes.values() {
            if let Some(d) = out.get_mut(&e.sidecar.dataset) {
                d.episodes += 1;
                d.duration_s += e.sidecar.duration_s;
                d.bytes += e.bytes;
            }
        }
        out.into_values().collect()
    }

    /// The dataset exists (`default` always does).
    pub fn has_dataset(&self, name: &str) -> bool {
        name == DEFAULT_DATASET || self.cat().datasets.datasets.contains_key(name)
    }

    /// Episode ids of `dataset` (catalog).
    pub fn episodes_of(&self, dataset: &str) -> Vec<String> {
        self.cat().episodes.values().filter(|e| e.sidecar.dataset == dataset).map(|e| e.sidecar.id.clone()).collect()
    }

    /// Add a dataset to the manifest (blocking).
    pub fn create_dataset(&self, name: &str, description: &str) -> Result<DatasetSummary, StoreError> {
        let dir = self.require_dir()?;
        if !valid_display_name(name) { return Err(StoreError::InvalidDataset(name.into())); }
        let _w = self.write_lock.lock().unwrap_or_else(|e| e.into_inner());
        let mut manifest = self.cat().datasets.clone();
        if manifest.datasets.contains_key(name) || (name == DEFAULT_DATASET && !self.episodes_of(name).is_empty()) {
            return Err(StoreError::DatasetExists(name.into()));
        }
        let info = DatasetInfo { created_at: now_rfc3339(), description: description.into() };
        manifest.datasets.insert(name.into(), info.clone());
        std::fs::create_dir_all(dir)?;
        self.write_manifest(dir, &manifest)?;
        self.cat().datasets = manifest;
        Ok(DatasetSummary { name: name.into(), episodes: 0, duration_s: 0.0, bytes: 0, created_at: Some(info.created_at), description: info.description })
    }

    /// Atomically verifies `dataset` exists (and is not mid-deletion) and pins it against
    /// concurrent deletion for the lifetime of a recording (start preflight -> active -> finalizing).
    pub fn pin_dataset_for_recording(&self, dataset: &str) -> Result<DatasetPinGuard, StoreError> {
        let _w = self.write_lock.lock().unwrap_or_else(|e| e.into_inner());
        let mut cat = self.cat();
        if cat.deleting_datasets.contains(dataset) {
            return Err(StoreError::DatasetNotFound(dataset.into()));
        }
        let exists = dataset == DEFAULT_DATASET || cat.datasets.datasets.contains_key(dataset);
        if !exists {
            return Err(StoreError::DatasetNotFound(dataset.into()));
        }
        *cat.pinned_datasets.entry(dataset.to_string()).or_insert(0) += 1;
        Ok(DatasetPinGuard {
            catalog: Arc::clone(&self.catalog),
            dataset: dataset.to_string(),
        })
    }

    /// Atomically ensures `dataset` exists (creating it if absent) and pins it against concurrent
    /// deletion for the lifetime of an episode import.
    pub fn ensure_and_pin_dataset(&self, dataset: &str) -> Result<DatasetPinGuard, StoreError> {
        let dir = self.require_dir()?;
        if dataset != DEFAULT_DATASET && !valid_display_name(dataset) {
            return Err(StoreError::InvalidDataset(dataset.into()));
        }
        let _w = self.write_lock.lock().unwrap_or_else(|e| e.into_inner());
        {
            let cat = self.cat();
            if cat.deleting_datasets.contains(dataset) {
                return Err(StoreError::DatasetRecordingBusy(dataset.into()));
            }
        }
        if dataset != DEFAULT_DATASET && !self.cat().datasets.datasets.contains_key(dataset) {
            let mut manifest = self.cat().datasets.clone();
            let info = DatasetInfo { created_at: now_rfc3339(), description: String::new() };
            manifest.datasets.insert(dataset.into(), info);
            std::fs::create_dir_all(dir)?;
            self.write_manifest(dir, &manifest)?;
            self.cat().datasets = manifest;
        }
        let mut cat = self.cat();
        if cat.deleting_datasets.contains(dataset) {
            return Err(StoreError::DatasetRecordingBusy(dataset.into()));
        }
        *cat.pinned_datasets.entry(dataset.to_string()).or_insert(0) += 1;
        Ok(DatasetPinGuard {
            catalog: Arc::clone(&self.catalog),
            dataset: dataset.to_string(),
        })
    }

    /// True if `dataset` currently has a starting, active, or finalizing recording or import.
    pub fn is_dataset_pinned(&self, dataset: &str) -> bool {
        self.cat()
            .pinned_datasets
            .get(dataset)
            .copied()
            .unwrap_or(0)
            > 0
    }

    /// True if a prior force-delete of `dataset` has un-deleted files waiting for retry.
    pub fn has_pending_delete_files(&self, dataset: &str) -> bool {
        self.cat()
            .pending_delete_files
            .get(dataset)
            .is_some_and(|v| !v.is_empty())
    }

    /// Remove a dataset from the manifest and, with `force`, its episodes (blocking). The
    /// caller checks references first ([`take_for_delete`](Self::take_for_delete)).
    pub fn delete_dataset(&self, name: &str, force: bool) -> Result<Vec<String>, StoreError> {
        let dir = self.require_dir()?;
        if self.is_dataset_pinned(name) {
            return Err(StoreError::DatasetRecordingBusy(name.into()));
        }
        let _w = self.write_lock.lock().unwrap_or_else(|e| e.into_inner());
        {
            let cat = self.cat();
            if cat.pinned_datasets.get(name).copied().unwrap_or(0) > 0
                || cat.deleting_datasets.contains(name)
            {
                return Err(StoreError::DatasetRecordingBusy(name.into()));
            }
        }
        let ids = self.episodes_of(name);
        let known = self.has_dataset(name) || !ids.is_empty();
        if !known { return Err(StoreError::DatasetNotFound(name.into())); }
        if !ids.is_empty() && !force { return Err(StoreError::DatasetNotEmpty(name.into(), ids.len())); }
        for id in &ids {
            if let Some(claim) = self.take_for_delete(id) {
                match self.delete_files_with_claim_locked(id, claim) {
                    Err(StoreError::NotFound(_)) => {}
                    other => other?,
                }
            }
        }
        let mut manifest = self.cat().datasets.clone();
        if manifest.datasets.remove(name).is_some() {
            self.write_manifest(dir, &manifest)?;
            self.cat().datasets = manifest;
        }
        Ok(ids)
    }

    /// Change `task` and / or `dataset` of an episode (sidecar rewrite, blocking).
    /// Atomically pins both source and destination datasets before writing the sidecar so a
    /// concurrent force-delete cannot race between the check and sidecar write, while holding
    /// `write_lock` to serialize against single-episode deletion.
    pub fn update(&self, id: &str, task: Option<&str>, dataset: Option<&str>) -> Result<EpisodeSidecar, StoreError> {
        let dir = self.require_dir()?;
        if task.is_some_and(|t| t.chars().count() > 256) { return Err(StoreError::Invalid("task is at most 256 characters".into())); }
        let _w = self.write_lock.lock().unwrap_or_else(|e| e.into_inner());
        let (mut sc, _src_pin, _dst_pin) = {
            let mut cat = self.cat();
            if let Some(d) = dataset {
                if cat.deleting_datasets.contains(d) {
                    return Err(StoreError::DatasetRecordingBusy(d.into()));
                }
                if d != DEFAULT_DATASET && !cat.datasets.datasets.contains_key(d) {
                    return Err(StoreError::DatasetNotFound(d.into()));
                }
            }
            if cat.deleting_episodes.contains_key(id) {
                return Err(StoreError::NotFound(id.into()));
            }
            let sc = cat
                .episodes
                .get(id)
                .map(|e| e.sidecar.clone())
                .ok_or_else(|| StoreError::NotFound(id.into()))?;
            if cat.deleting_datasets.contains(&sc.dataset) {
                return Err(StoreError::DatasetRecordingBusy(sc.dataset.clone()));
            }
            *cat.pinned_datasets.entry(sc.dataset.clone()).or_insert(0) += 1;
            let src_pin = DatasetPinGuard {
                catalog: Arc::clone(&self.catalog),
                dataset: sc.dataset.clone(),
            };
            let dst_pin = dataset.filter(|d| *d != sc.dataset).map(|d| {
                *cat.pinned_datasets.entry(d.to_string()).or_insert(0) += 1;
                DatasetPinGuard {
                    catalog: Arc::clone(&self.catalog),
                    dataset: d.to_string(),
                }
            });
            (sc, src_pin, dst_pin)
        };
        if let Some(t) = task { sc.task = t.into(); }
        if let Some(d) = dataset { sc.dataset = d.into(); }
        Self::write_sidecar_file(dir, &sc)?;
        let mut cat = self.cat();
        if let Some(e) = cat.episodes.get_mut(id) {
            e.sidecar = sc.clone();
        } else {
            return Err(StoreError::NotFound(id.into()));
        }
        Ok(sc)
    }

    /// Remove `id` from the catalog and install a per-episode deletion claim token until
    /// [`delete_files_with_claim`](Self::delete_files_with_claim) finishes unlinking on-disk files.
    pub fn take_for_delete(&self, id: &str) -> Option<u64> {
        let mut cat = self.cat();
        if cat.episodes.remove(id).is_some() {
            cat.commit_tokens.remove(id);
            let claim = next_delete_claim();
            cat.deleting_episodes.insert(id.to_string(), claim);
            Some(claim)
        } else {
            None
        }
    }

    /// Atomically verifies that every `(step_idx, episode_id)` exists in the catalog and pins
    /// them against concurrent deletion while a `Sequences` reservation is held.
    /// Returns `Err((step_idx, episode_id))` on the first missing episode without pinning any.
    pub fn pin_episodes_for_sequence(
        &self,
        step_episodes: &[(usize, String)],
    ) -> Result<EpisodePinGuard, (usize, String)> {
        let mut cat = self.cat();
        for (step, id) in step_episodes {
            if !cat.episodes.contains_key(id) {
                return Err((*step, id.clone()));
            }
        }
        let mut ids = Vec::with_capacity(step_episodes.len());
        for (_, id) in step_episodes {
            *cat.pinned_episodes.entry(id.clone()).or_insert(0) += 1;
            ids.push(id.clone());
        }
        Ok(EpisodePinGuard {
            catalog: Arc::clone(&self.catalog),
            ids,
        })
    }

    /// True if `id` is currently pinned by an active `Sequences` save reservation or reconciling state.
    pub fn is_pinned(&self, id: &str) -> bool {
        self.cat()
            .pinned_episodes
            .get(id)
            .copied()
            .unwrap_or(0)
            > 0
    }

    /// Atomically checks that `id` is not pinned and `seq_rev` still equals `expected_rev`,
    /// then removes `id` from the catalog and installs its unique deletion claim token under
    /// `write_lock` (serialized against `EpisodeStore::update` sidecar writes).
    pub fn take_if_unpinned_and_rev_matches(
        &self,
        id: &str,
        seq_rev: &AtomicU64,
        expected_rev: u64,
    ) -> TakeEpisodeOutcome {
        let _w = self.write_lock.lock().unwrap_or_else(|e| e.into_inner());
        let mut cat = self.cat();
        if cat.pinned_episodes.get(id).copied().unwrap_or(0) > 0 {
            return TakeEpisodeOutcome::Pinned;
        }
        if seq_rev.load(Ordering::Acquire) != expected_rev {
            return TakeEpisodeOutcome::SequenceRevisionChanged;
        }
        if cat.episodes.remove(id).is_some() {
            cat.commit_tokens.remove(id);
            let claim = next_delete_claim();
            cat.deleting_episodes.insert(id.to_string(), claim);
            TakeEpisodeOutcome::Removed(claim)
        } else {
            TakeEpisodeOutcome::NotFound
        }
    }

    /// Atomically verifies `dataset` is not pinned or already deleting, revalidates that current
    /// catalog membership of `dataset` still exactly equals `ids`, verifies none of `ids` are
    /// pinned by a sequence save and `seq_rev` still equals `expected_rev`, then marks `dataset`
    /// as mid-deletion (single owner via [`DatasetDeleteGuard`]), installs per-episode deletion
    /// claim tokens, retains any prior un-deleted retry claims, and removes `ids` from the catalog.
    pub fn take_dataset_if_unpinned_and_rev_matches(
        &self,
        dataset: &str,
        ids: &[String],
        seq_rev: &AtomicU64,
        expected_rev: u64,
    ) -> TakeDatasetOutcome {
        let mut cat = self.cat();
        if cat.pinned_datasets.get(dataset).copied().unwrap_or(0) > 0
            || cat.deleting_datasets.contains(dataset)
        {
            return TakeDatasetOutcome::DatasetRecordingBusy;
        }
        let current_ids: std::collections::HashSet<&str> = cat
            .episodes
            .iter()
            .filter(|(_, e)| e.sidecar.dataset == dataset)
            .map(|(id, _)| id.as_str())
            .collect();
        let expected_set: std::collections::HashSet<&str> =
            ids.iter().map(String::as_str).collect();
        if current_ids != expected_set {
            return TakeDatasetOutcome::SequenceRevisionChanged;
        }
        for id in ids {
            if cat.pinned_episodes.get(id).copied().unwrap_or(0) > 0 {
                return TakeDatasetOutcome::Pinned(id.clone());
            }
        }
        if seq_rev.load(Ordering::Acquire) != expected_rev {
            return TakeDatasetOutcome::SequenceRevisionChanged;
        }
        cat.deleting_datasets.insert(dataset.to_string());
        let mut all_claims: Vec<(String, u64)> = cat
            .pending_delete_files
            .remove(dataset)
            .unwrap_or_default()
            .into_iter()
            .filter(|(id, tok)| cat.deleting_episodes.get(id).copied() == Some(*tok))
            .collect();
        for id in ids {
            if cat.episodes.remove(id).is_some() {
                cat.commit_tokens.remove(id);
                let claim = next_delete_claim();
                cat.deleting_episodes.insert(id.clone(), claim);
                all_claims.push((id.clone(), claim));
            }
        }
        if !all_claims.is_empty() {
            cat.pending_delete_files
                .insert(dataset.to_string(), all_claims.clone());
        }
        TakeDatasetOutcome::Removed(
            all_claims,
            DatasetDeleteGuard {
                catalog: Arc::clone(&self.catalog),
                dataset: dataset.to_string(),
            },
        )
    }

    /// Delete dataset manifest entry plus already-catalog-removed episode files (blocking).
    /// Takes ownership of `_guard` so async cancellation never releases the dataset deletion
    /// guard before disk cleanup finishes. Validates each `(id, claim_token)` before unlinking so
    /// a stale delete can never unlink a replacement episode.
    pub fn delete_dataset_taken_files(
        &self,
        name: &str,
        taken_claims: &[(String, u64)],
        _guard: DatasetDeleteGuard,
    ) -> Result<Vec<String>, StoreError> {
        let dir = self.require_dir()?;
        if self.is_dataset_pinned(name) {
            return Err(StoreError::DatasetRecordingBusy(name.into()));
        }
        let _w = self.write_lock.lock().unwrap_or_else(|e| e.into_inner());
        if self.is_dataset_pinned(name) {
            return Err(StoreError::DatasetRecordingBusy(name.into()));
        }
        let mut deleted = Vec::with_capacity(taken_claims.len());
        for (id, claim) in taken_claims {
            match self.delete_files_with_claim_locked(id, *claim) {
                Ok(()) | Err(StoreError::NotFound(_)) => {
                    deleted.push(id.clone());
                }
                Err(e) => return Err(e),
            }
        }
        let mut manifest = self.cat().datasets.clone();
        if manifest.datasets.remove(name).is_some() {
            self.write_manifest(dir, &manifest)?;
            self.cat().datasets = manifest;
        }
        self.cat().pending_delete_files.remove(name);
        Ok(deleted)
    }

    fn unlink_episode_files_raw(&self, id: &str) -> Result<bool, StoreError> {
        let path = self.path_of(id)?;
        let dir = self.require_dir()?;
        self.cache.lock().unwrap_or_else(|e| e.into_inner()).remove(id);
        let mut found = false;
        for p in [path.clone(), Self::sidecar_path(dir, id), path.with_extension("meta.json")] {
            match std::fs::remove_file(&p) {
                Ok(()) => found = true,
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
                Err(e) => return Err(e.into()),
            }
        }
        Ok(found)
    }

    fn delete_files_with_claim_locked(&self, id: &str, claim: u64) -> Result<(), StoreError> {
        {
            let cat = self.cat();
            if cat.in_progress_tokens.contains_key(id) {
                return Err(StoreError::NotFound(id.into()));
            }
            if cat.deleting_episodes.get(id).copied() != Some(claim) {
                return Ok(());
            }
        }
        let found = self.unlink_episode_files_raw(id)?;
        let mut cat = self.cat();
        if cat.deleting_episodes.get(id).copied() == Some(claim) {
            cat.deleting_episodes.remove(id);
        }
        cat.pending_delete_files.retain(|_, list| {
            list.retain(|(eid, tok)| !(eid == id && *tok == claim));
            !list.is_empty()
        });
        if found { Ok(()) } else { Err(StoreError::NotFound(id.into())) }
    }

    /// Remove on-disk files for an episode taken via `take_for_delete` /
    /// `take_if_unpinned_and_rev_matches`, validating exact `claim` ownership under `write_lock`.
    pub fn delete_files_with_claim(&self, id: &str, claim: u64) -> Result<(), StoreError> {
        let _w = self.write_lock.lock().unwrap_or_else(|e| e.into_inner());
        self.delete_files_with_claim_locked(id, claim)
    }

    /// Remove uncataloged legacy `<id>.parquet` / sidecar files (blocking).
    /// Serialized with `write_lock` and refuses to touch any ID currently owned by the catalog,
    /// an in-flight publication, or an active deletion claim.
    pub fn delete_files(&self, id: &str) -> Result<(), StoreError> {
        let _w = self.write_lock.lock().unwrap_or_else(|e| e.into_inner());
        {
            let cat = self.cat();
            if cat.episodes.contains_key(id)
                || cat.in_progress_tokens.contains_key(id)
                || cat.deleting_episodes.contains_key(id)
            {
                return Err(StoreError::NotFound(id.into()));
            }
        }
        let found = self.unlink_episode_files_raw(id)?;
        if found { Ok(()) } else { Err(StoreError::NotFound(id.into())) }
    }

    /// Catalog removal + file removal (blocking).
    pub fn delete(&self, id: &str) -> Result<(), StoreError> {
        let _w = self.write_lock.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(claim) = self.take_for_delete(id) {
            match self.delete_files_with_claim_locked(id, claim) {
                Err(StoreError::NotFound(_)) => Ok(()),
                other => other,
            }
        } else {
            let cat = self.cat();
            if cat.in_progress_tokens.contains_key(id) || cat.deleting_episodes.contains_key(id) {
                return Err(StoreError::NotFound(id.into()));
            }
            drop(cat);
            let found = self.unlink_episode_files_raw(id)?;
            if found { Ok(()) } else { Err(StoreError::NotFound(id.into())) }
        }
    }

    fn confirm_parent_dir_sync(path: &Path) -> Result<(), StoreError> {
        match crate::persist::PendingDirSync::for_path(path).sync_parent_dir() {
            crate::persist::PublishOutcome::Published => Ok(()),
            crate::persist::PublishOutcome::PublishedDurabilityUnconfirmed(err) => {
                Err(StoreError::Io(std::io::Error::other(format!(
                    "parent directory sync failed for {}: {err}",
                    path.display()
                ))))
            }
        }
    }

    /// Single serialized episode publication path (blocking): rejects empty episodes, writes
    /// parquet (temp + fsync + atomic rename, sidecar in footer), writes atomic sidecar JSON,
    /// syncs the parent directory (failing closed on durability failure), validates the parquet
    /// file, and inserts into the catalog only once complete.
    pub fn publish_complete_episode(
        &self,
        sidecar: &EpisodeSidecar,
        frames: &[RecordFrame],
        writer: Option<Box<dyn FnOnce(&Path, &[RecordFrame]) -> Result<(), ParquetRecordingError> + Send>>,
        commit_token: Option<u64>,
        enforce_capacity: bool,
    ) -> Result<PathBuf, StoreError> {
        self.publish_complete_episode_with_slot(
            sidecar,
            frames,
            writer,
            commit_token,
            enforce_capacity,
            None,
        )
    }

    /// Same as [`Self::publish_complete_episode`], atomically consuming `reserved_slot` into the
    /// `in_progress_tokens` claim under the same catalog lock so 1 episode occupies strictly 1
    /// capacity slot across reservation -> publication -> file rename.
    pub fn publish_complete_episode_with_slot(
        &self,
        sidecar: &EpisodeSidecar,
        frames: &[RecordFrame],
        writer: Option<Box<dyn FnOnce(&Path, &[RecordFrame]) -> Result<(), ParquetRecordingError> + Send>>,
        commit_token: Option<u64>,
        enforce_capacity: bool,
        reserved_slot: Option<&mut CapacitySlotGuard>,
    ) -> Result<PathBuf, StoreError> {
        if frames.is_empty() {
            return Err(StoreError::Parquet(ParquetRecordingError::EmptyFrames));
        }
        let dir = self.require_dir()?;
        let path = self.path_of(&sidecar.id)?;
        // Phase 1 (under write_lock): idempotency/duplicate check, capacity check, and id claim.
        {
            let _w = self.write_lock.lock().unwrap_or_else(|e| e.into_inner());
            let cat = self.cat();
            if cat.deleting_episodes.contains_key(&sidecar.id) {
                return Err(StoreError::Exists(sidecar.id.clone()));
            }
            if cat.episodes.contains_key(&sidecar.id) {
                if let Some(tok) = commit_token {
                    if let Some(done) = cat.commit_tokens.get(&sidecar.id) {
                        if done.token == tok && done.sidecar == *sidecar {
                            drop(cat);
                            if !path.is_file() {
                                return Err(StoreError::NotFound(sidecar.id.clone()));
                            }
                            return Ok(path);
                        }
                    }
                }
                return Err(StoreError::Exists(sidecar.id.clone()));
            }
            if path.exists() {
                if commit_token.is_some()
                    && cat
                        .in_progress_tokens
                        .get(&sidecar.id)
                        .is_some_and(|c| c.token == commit_token && c.sidecar == *sidecar)
                {
                    drop(cat);
                    // Post-parquet-rename recovery for the exact same (token, sidecar) retry.
                    if !path.is_file() {
                        return Err(StoreError::NotFound(sidecar.id.clone()));
                    }
                    let bytes = std::fs::metadata(&path)?.len();
                    Self::write_sidecar_file(dir, sidecar)?;
                    Self::confirm_parent_dir_sync(&path)?;
                    let mut cat = self.cat();
                    if let Some(slot) = reserved_slot {
                        slot.release_locked(&mut cat);
                    }
                    cat.episodes
                        .insert(sidecar.id.clone(), Entry { sidecar: sidecar.clone(), bytes });
                    if let Some(tok) = commit_token {
                        cat.commit_tokens.insert(
                            sidecar.id.clone(),
                            CompletedClaim {
                                token: tok,
                                sidecar: sidecar.clone(),
                            },
                        );
                    }
                    cat.in_progress_tokens.remove(&sidecar.id);
                    return Ok(path);
                }
                return Err(StoreError::Exists(sidecar.id.clone()));
            }
            if cat.in_progress_tokens.contains_key(&sidecar.id) {
                return Err(StoreError::Exists(sidecar.id.clone()));
            }
            drop(cat);
            if enforce_capacity {
                self.check_capacity()?;
            }
            std::fs::create_dir_all(dir)?;
            let mut cat = self.cat();
            if let Some(slot) = reserved_slot {
                slot.release_locked(&mut cat);
            }
            cat.in_progress_tokens.insert(
                sidecar.id.clone(),
                InProgressClaim {
                    token: commit_token,
                    sidecar: sidecar.clone(),
                },
            );
        }
        // Phase 2 (off write_lock): parquet write (temp + fsync + atomic rename) so unrelated
        // dataset/sidecar queries never block behind a large episode write.
        if let Some((entered_tx, release_rx)) = self
            .publish_pause
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .take()
        {
            let _ = entered_tx.send(sidecar.id.clone());
            let _ = release_rx.recv();
        }
        let write_res = match writer {
            Some(w) => w(&path, frames).map_err(StoreError::from),
            None => {
                let embedded =
                    serde_json::to_string(sidecar).map_err(|e| StoreError::Invalid(e.to_string()))?;
                write_episode_parquet(
                    &path,
                    frames,
                    Some(&sidecar.metadata()),
                    &[(PARQUET_SIDECAR_KEY, embedded)],
                )
                .map_err(StoreError::from)
            }
        };
        if let Err(e) = write_res {
            if !path.is_file() {
                self.cat().in_progress_tokens.remove(&sidecar.id);
            }
            return Err(e);
        }
        if !path.is_file() {
            self.cat().in_progress_tokens.remove(&sidecar.id);
            return Err(StoreError::Io(std::io::Error::new(
                std::io::ErrorKind::NotFound,
                format!("parquet writer reported success without creating {}", path.display()),
            )));
        }
        // Phase 3 (under write_lock): verify parquet file, write atomic sidecar, sync parent dir,
        // and publish to catalog only after every step succeeds.
        let _w = self.write_lock.lock().unwrap_or_else(|e| e.into_inner());
        let bytes = std::fs::metadata(&path)?.len();
        Self::write_sidecar_file(dir, sidecar)?;
        Self::confirm_parent_dir_sync(&path)?;
        let mut cat = self.cat();
        cat.episodes
            .insert(sidecar.id.clone(), Entry { sidecar: sidecar.clone(), bytes });
        if let Some(tok) = commit_token {
            cat.commit_tokens.insert(
                sidecar.id.clone(),
                CompletedClaim {
                    token: tok,
                    sidecar: sidecar.clone(),
                },
            );
        }
        cat.in_progress_tokens.remove(&sidecar.id);
        Ok(path)
    }

    /// Write a new episode (blocking): parquet (temp + fsync + rename, sidecar in the footer),
    /// then the sidecar, then the catalog entry. `writer` writes the parquet file at the path
    /// it gets; None = the store's writer.
    pub fn write_episode(
        &self,
        sidecar: &EpisodeSidecar,
        frames: &[RecordFrame],
        writer: Option<Box<dyn FnOnce(&Path, &[RecordFrame]) -> Result<(), ParquetRecordingError> + Send>>,
    ) -> Result<PathBuf, StoreError> {
        self.publish_complete_episode(sidecar, frames, writer, None, false)
    }

    // ---- recording checks ----

    /// Set the free bytes reported by [`free_bytes`](Self::free_bytes) (tests). None: the file system.
    pub fn set_free_bytes_override(&self, bytes: Option<u64>) {
        *self.free_override.lock().unwrap_or_else(|e| e.into_inner()) = bytes;
    }

    /// Test hook: arms a one-shot pause inside Phase 2 of [`Self::publish_complete_episode`].
    /// Returns `(entered_rx, release_tx)` where `entered_rx` receives the episode id once paused.
    #[doc(hidden)]
    pub fn arm_test_publish_pause(
        &self,
    ) -> (std::sync::mpsc::Receiver<String>, std::sync::mpsc::SyncSender<()>) {
        let (entered_tx, entered_rx) = std::sync::mpsc::sync_channel(1);
        let (release_tx, release_rx) = std::sync::mpsc::sync_channel(1);
        *self.publish_pause.lock().unwrap_or_else(|e| e.into_inner()) = Some((entered_tx, release_rx));
        (entered_rx, release_tx)
    }

    /// Free bytes on the file system of the episode directory (its nearest existing ancestor).
    pub fn free_bytes(&self) -> Option<u64> {
        if let Some(b) = *self.free_override.lock().unwrap_or_else(|e| e.into_inner()) { return Some(b); }
        let dir = self.dir.as_deref()?;
        let mut p = std::path::absolute(dir).ok()?;
        while !p.exists() { if !p.pop() { return None; } }
        statvfs_free(&p)
    }

    /// RECORD_NO_SPACE below [`START_MIN_FREE_BYTES`] (unknown free space passes).
    pub fn check_free_for_start(&self) -> Result<(), StoreError> {
        match self.free_bytes() {
            Some(free) if free < START_MIN_FREE_BYTES => Err(StoreError::NoSpace { free, need: START_MIN_FREE_BYTES }),
            _ => Ok(()),
        }
    }

    /// Legacy REST start: id and file for a new recording. `client_path` (absolute, or
    /// relative to the episode directory) must name `<id>.parquet` directly inside the
    /// directory; its stem is the id and must equal `episode_id` when both are given. Without
    /// either, the id is a new ULID. An existing episode is [`StoreError::Exists`].
    pub fn new_episode(&self, episode_id: Option<&str>, client_path: Option<&Path>) -> Result<(String, PathBuf), StoreError> {
        let dir = self.require_dir()?;
        if let Some(id) = episode_id {
            if !valid_episode_id(id) { return Err(StoreError::InvalidId(id.into())); }
        }
        let id = match client_path {
            Some(p) => {
                let stem = self.stem_inside(dir, p)?;
                if episode_id.is_some_and(|id| id != stem) {
                    return Err(StoreError::PathOutside(p.display().to_string(), format!("{} (its file name must match episode_id)", dir.display())));
                }
                stem
            }
            None => match episode_id {
                Some(id) => id.to_string(),
                None => new_ulid(),
            },
        };
        let path = dir.join(format!("{id}.{EPISODE_EXT}"));
        let claimed = {
            let cat = self.cat();
            cat.episodes.contains_key(&id)
                || cat.in_progress_tokens.contains_key(&id)
                || cat.deleting_episodes.contains_key(&id)
        };
        if path.exists() || claimed { return Err(StoreError::Exists(id)); }
        Ok((id, path))
    }

    /// File stem of `p` when it is `<valid id>.parquet` directly inside `dir`.
    fn stem_inside(&self, dir: &Path, p: &Path) -> Result<String, StoreError> {
        let outside = || StoreError::PathOutside(p.display().to_string(), dir.display().to_string());
        if p.components().any(|c| matches!(c, Component::ParentDir)) { return Err(outside()); }
        let full = if p.is_absolute() { p.to_path_buf() } else { dir.join(p) };
        let parent = full.parent().ok_or_else(outside)?;
        let same_dir = match (std::path::absolute(parent), std::path::absolute(dir)) {
            (Ok(a), Ok(b)) => normalize(&a) == normalize(&b),
            _ => false,
        };
        if !same_dir { return Err(outside()); }
        if full.extension().and_then(|e| e.to_str()) != Some(EPISODE_EXT) { return Err(outside()); }
        let stem = full.file_stem().and_then(|s| s.to_str()).ok_or_else(outside)?;
        if !valid_episode_id(stem) { return Err(StoreError::InvalidId(stem.into())); }
        Ok(stem.to_string())
    }

    /// (bytes, episode files) in the directory; a missing directory is empty.
    pub fn usage(&self) -> Result<(u64, usize), StoreError> {
        let dir = self.require_dir()?;
        let mut bytes = 0;
        let mut files = 0;
        for (_, meta) in episode_files(dir)? {
            bytes += meta.len();
            files += 1;
        }
        Ok((bytes, files))
    }

    /// [`StoreError::Full`] when the directory (including the union of on-disk files and
    /// in-flight publication claims, plus reserved capacity slots) is at either cap.
    pub fn check_capacity(&self) -> Result<(), StoreError> {
        self.check_capacity_with(MAX_EPISODES_BYTES, MAX_EPISODE_FILES)
    }

    pub fn check_capacity_with(&self, max_bytes: u64, max_files: usize) -> Result<(), StoreError> {
        let dir = self.require_dir()?;
        let mut bytes = 0u64;
        let mut disk_stems = std::collections::HashSet::new();
        for (stem, meta) in episode_files(dir)? {
            bytes = bytes.saturating_add(meta.len());
            disk_stems.insert(stem);
        }
        let (in_progress_extra, reserved_slots) = {
            let cat = self.cat();
            let extra = cat
                .in_progress_tokens
                .keys()
                .filter(|id| !disk_stems.contains(id.as_str()))
                .count();
            (extra, cat.reserved_slots)
        };
        let effective_files = disk_stems
            .len()
            .saturating_add(in_progress_extra)
            .saturating_add(reserved_slots);
        if bytes >= max_bytes || effective_files >= max_files {
            return Err(StoreError::Full {
                files: effective_files,
                bytes,
                max_files,
                max_bytes,
            });
        }
        Ok(())
    }

    /// Atomically verifies capacity and reserves one episode file slot for the lifetime of the
    /// returned [`CapacitySlotGuard`], preventing concurrent manual saves and imports from
    /// stealing the last slot or racing past `MAX_EPISODE_FILES`.
    pub fn reserve_capacity_slot(&self) -> Result<CapacitySlotGuard, StoreError> {
        self.reserve_capacity_slot_with(MAX_EPISODES_BYTES, MAX_EPISODE_FILES)
    }

    pub fn reserve_capacity_slot_with(
        &self,
        max_bytes: u64,
        max_files: usize,
    ) -> Result<CapacitySlotGuard, StoreError> {
        let _w = self.write_lock.lock().unwrap_or_else(|e| e.into_inner());
        self.check_capacity_with(max_bytes, max_files)?;
        let mut cat = self.cat();
        cat.reserved_slots = cat.reserved_slots.saturating_add(1);
        Ok(CapacitySlotGuard {
            catalog: Arc::clone(&self.catalog),
            active: true,
        })
    }

    /// Legacy listing: catalog episodes with per-joint ranges, newest first. Ranges are read
    /// from the parquet and cached by (mtime, size).
    pub fn list(&self) -> Result<EpisodeList, StoreError> {
        self.list_with(&[])
    }

    pub fn list_with(&self, offsets: &[f32]) -> Result<EpisodeList, StoreError> {
        let Some(dir) = self.dir.as_deref() else {
            return Ok(EpisodeList { enabled: false, dir: None, episodes: Vec::new(), total_bytes: 0, max_bytes: MAX_EPISODES_BYTES, max_files: MAX_EPISODE_FILES });
        };
        let entries: Vec<Entry> = self.cat().episodes.values().cloned().collect();
        let mut episodes = Vec::new();
        let mut total_bytes = 0;
        let mut cache = self.cache.lock().unwrap_or_else(|e| e.into_inner());
        for e in &entries {
            let id = &e.sidecar.id;
            let path = dir.join(format!("{id}.{EPISODE_EXT}"));
            let Ok(meta) = std::fs::metadata(&path) else { continue; };
            total_bytes += meta.len();
            let mtime = meta.modified().unwrap_or(SystemTime::UNIX_EPOCH);
            let mut info = match cache.get(id) {
                Some((t, size, info)) if *t == mtime && *size == meta.len() => info.clone(),
                _ => {
                    let info = summarize(id, &path, meta.len());
                    cache.insert(id.clone(), (mtime, meta.len(), info.clone()));
                    info
                }
            };
            info.created_ms = recorded_ms(&e.sidecar.recorded_at);
            info.dataset = Some(e.sidecar.dataset.clone());
            info.task = Some(e.sidecar.task.clone());
            info.source = Some(e.sidecar.source.clone());
            info.stale = !offsets.is_empty() && e.sidecar.joint_zero.stale_against(offsets);
            episodes.push(info);
        }
        cache.retain(|id, _| entries.iter().any(|e| &e.sidecar.id == id));
        episodes.sort_by(|a, b| b.created_ms.cmp(&a.created_ms).then_with(|| b.id.cmp(&a.id)));
        Ok(EpisodeList { enabled: true, dir: Some(dir.display().to_string()), episodes, total_bytes, max_bytes: MAX_EPISODES_BYTES, max_files: MAX_EPISODE_FILES })
    }
}

#[cfg(unix)]
fn statvfs_free(p: &Path) -> Option<u64> {
    use std::os::unix::ffi::OsStrExt;
    let c = std::ffi::CString::new(p.as_os_str().as_bytes()).ok()?;
    let mut st: libc::statvfs = unsafe { std::mem::zeroed() };
    // SAFETY: `c` is a NUL-terminated path and `st` a valid out pointer.
    if unsafe { libc::statvfs(c.as_ptr(), &mut st) } != 0 { return None; }
    Some((st.f_bavail as u64).saturating_mul(st.f_frsize as u64))
}

#[cfg(not(unix))]
fn statvfs_free(_p: &Path) -> Option<u64> { None }

/// Sidecar for a parquet recorded before the store (legacy REST): frames and the legacy
/// metadata (parquet footer / `.meta.json`); joint zero = `offsets`. None when unreadable.
fn adopt_legacy(path: &Path, id: &str, offsets: &[f32], joint_names: &[String], profile_id: Option<&str>, fps: f64) -> Option<EpisodeSidecar> {
    let frames = read_frames_from_parquet(path).ok()?;
    let meta = read_metadata_from_parquet(path).ok().flatten().unwrap_or_default();
    let (first, last) = (frames.first()?, frames.last()?);
    let mtime = std::fs::metadata(path).and_then(|m| m.modified()).unwrap_or(SystemTime::UNIX_EPOCH);
    let duration = (last.timestamp - first.timestamp).max(0.0);
    let started = chrono::DateTime::<chrono::Utc>::from(mtime) - chrono::Duration::milliseconds((duration * 1000.0) as i64);
    Some(EpisodeSidecar {
        id: id.into(), dataset: DEFAULT_DATASET.into(), task: String::new(), source: "import".into(),
        sequence_id: None, repeat_index: None, outcome: "ok".into(),
        recorded_at: started.to_rfc3339_opts(chrono::SecondsFormat::Millis, true),
        duration_s: duration, frames: frames.len(),
        joint_names: meta.joint_names.clone().unwrap_or_else(|| joint_names.to_vec()),
        units: "rad".into(), joint_zero: JointZeroStamp::of(offsets),
        profile_id: meta.profile_id.clone().or_else(|| profile_id.map(str::to_string)),
        armd_version: "legacy".into(), fps_nominal: fps, truncated: false, stopped_by: None,
        arm_dof: meta.arm_dof, gripper_joint: meta.gripper_joint, recovered: Some("legacy".into()),
        first_state: Some(first.observation_state.clone()), last_state: Some(last.observation_state.clone()),
        tool_id: meta.tool_id,
        tool_config_digest: meta.tool_config_digest,
        has_gripper_motion: Some(meta.has_gripper_motion.unwrap_or_else(|| episode_has_gripper_motion(&frames, meta.gripper_joint))),
        frames_dropped: None,
    })
}

/// Lexical normalization: drops `.` components (`..` is rejected before this is used).
fn normalize(p: &Path) -> PathBuf {
    p.components().filter(|c| !matches!(c, Component::CurDir)).collect()
}

/// `(id, metadata)` of every regular `<valid id>.parquet` file in `dir`.
fn episode_files(dir: &Path) -> Result<Vec<(String, std::fs::Metadata)>, StoreError> {
    let entries = match std::fs::read_dir(dir) {
        Ok(e) => e,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(Vec::new()),
        Err(e) => return Err(e.into()),
    };
    let mut out = Vec::new();
    for entry in entries {
        let entry = entry?;
        let path = entry.path();
        if path.extension().and_then(|e| e.to_str()) != Some(EPISODE_EXT) { continue; }
        let Some(stem) = path.file_stem().and_then(|s| s.to_str()).filter(|s| valid_episode_id(s)) else { continue; };
        let meta = entry.metadata()?;
        if meta.is_file() { out.push((stem.to_string(), meta)); }
    }
    Ok(out)
}

fn summarize(id: &str, path: &Path, size_bytes: u64) -> EpisodeInfo {
    let mut info = EpisodeInfo {
        id: id.into(), frames: 0, duration_s: 0.0, size_bytes, created_ms: 0, dof: 0,
        joint_min: Vec::new(), joint_max: Vec::new(), error: None, dataset: None, task: None, source: None, stale: false,
    };
    match read_frames_from_parquet(path) {
        Ok(frames) => {
            info.frames = frames.len();
            if let (Some(first), Some(last)) = (frames.first(), frames.last()) {
                info.duration_s = last.timestamp - first.timestamp;
                info.dof = first.observation_state.len();
                info.joint_min = vec![f32::INFINITY; info.dof];
                info.joint_max = vec![f32::NEG_INFINITY; info.dof];
                for f in &frames {
                    for (j, q) in f.observation_state.iter().take(info.dof).enumerate() {
                        info.joint_min[j] = info.joint_min[j].min(*q);
                        info.joint_max[j] = info.joint_max[j].max(*q);
                    }
                }
            }
        }
        Err(e) => info.error = Some(e.to_string()),
    }
    info
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::recording::{write_frames_to_parquet, RecordFrame};

    fn frames(n: usize) -> Vec<RecordFrame> {
        (0..n).map(|k| RecordFrame { timestamp: 10.0 + k as f64 * 0.02, observation_state: vec![k as f32 * 0.01, -0.5], action: vec![0.0, 0.0] }).collect()
    }

    fn sidecar(id: &str, dataset: &str) -> EpisodeSidecar {
        EpisodeSidecar {
            id: id.into(), dataset: dataset.into(), task: "t".into(), source: "jog".into(), sequence_id: None, repeat_index: None,
            outcome: "ok".into(), recorded_at: now_rfc3339(), duration_s: 1.0, frames: 51, joint_names: vec!["a".into(), "b".into()],
            units: "rad".into(), joint_zero: JointZeroStamp::of(&[0.0, 0.0]), profile_id: None, armd_version: "x".into(),
            fps_nominal: 50.0, truncated: false, stopped_by: None, arm_dof: None, gripper_joint: None, recovered: None,
            first_state: None, last_state: None,
            tool_id: None, tool_config_digest: None, has_gripper_motion: None, frames_dropped: None,
        }
    }

    #[test]
    fn ids() {
        for ok in ["a", "ep_1", "Episode-20261112_101010", &"x".repeat(64)] { assert!(valid_episode_id(ok), "{ok}"); }
        for bad in ["", "a b", "../x", "a/b", "a.parquet", "é", &"x".repeat(65), "a\0", "."] { assert!(!valid_episode_id(bad), "{bad:?}"); }
        let a = new_ulid();
        let b = new_ulid();
        assert!(is_ulid(&a) && is_ulid(&b) && a < b && valid_episode_id(&a), "{a} {b}");
        assert!(!is_ulid("episode_1") && !is_ulid(&"8".repeat(26)) && !is_ulid(&"0".repeat(25)) && !is_ulid(&"0I".repeat(13)));
        assert_eq!(zero_rev(&[0.0, 0.1]), zero_rev(&[0.0, 0.1]));
        assert_ne!(zero_rev(&[0.0, 0.1]), zero_rev(&[0.0, 0.2]));
        for ok in ["default", "pick place", "抓取-1", "a_b"] { assert!(valid_display_name(ok), "{ok}"); }
        for bad in ["", "a/b", "a.b", &"x".repeat(65), "été"] { assert!(!valid_display_name(bad), "{bad}"); }
        let st = JointZeroStamp::of(&[0.0, 0.0]);
        assert!(!st.stale_against(&[0.005, 0.0]) && st.stale_against(&[0.01, 0.0]) && st.stale_against(&[0.0]));
    }

    #[test]
    fn output_path_must_stay_inside_the_directory() {
        let tmp = tempfile::tempdir().unwrap();
        let dir = tmp.path().join("episodes");
        let store = EpisodeStore::new(Some(dir.clone()));
        assert_eq!(store.new_episode(Some("a"), None).unwrap(), ("a".into(), dir.join("a.parquet")));
        assert_eq!(store.new_episode(None, Some(Path::new("b.parquet"))).unwrap().0, "b");
        assert_eq!(store.new_episode(None, Some(&dir.join("c.parquet"))).unwrap().0, "c");
        assert_eq!(store.new_episode(None, Some(&dir.join(".").join("c2.parquet"))).unwrap().0, "c2");
        let code = |r: Result<(String, PathBuf), StoreError>| r.unwrap_err().status_code().1;
        assert_eq!(code(store.new_episode(None, Some(Path::new("../x.parquet")))), "INVALID_OUTPUT_PATH");
        assert_eq!(code(store.new_episode(None, Some(&tmp.path().join("x.parquet")))), "INVALID_OUTPUT_PATH");
        assert_eq!(code(store.new_episode(None, Some(Path::new("sub/x.parquet")))), "INVALID_OUTPUT_PATH");
        assert_eq!(code(store.new_episode(None, Some(Path::new("a b.parquet")))), "INVALID_EPISODE_ID");
        assert_eq!(code(store.new_episode(Some("e"), Some(Path::new("f.parquet")))), "INVALID_OUTPUT_PATH");
        write_frames_to_parquet(&dir.join("a.parquet"), &frames(3)).unwrap();
        assert_eq!(code(store.new_episode(Some("a"), None)), "EPISODE_EXISTS");
        // generated ids are ULIDs
        let (g, _) = store.new_episode(None, None).unwrap();
        assert!(is_ulid(&g), "{g}");
    }

    #[test]
    fn resolve_and_disabled() {
        assert_eq!(EpisodeStore::resolve(Some("/x/ep")).dir(), Some(Path::new("/x/ep")));
        assert_eq!(EpisodeStore::resolve(Some("off")).dir(), None);
        let off = EpisodeStore::new(None);
        assert_eq!(off.path_of("a").unwrap_err().status_code().1, "EPISODES_DISABLED");
        assert_eq!(off.new_episode(None, None).unwrap_err().status_code().1, "EPISODES_DISABLED");
        let list = off.list().unwrap();
        assert!(!list.enabled && list.episodes.is_empty());
    }

    #[test]
    fn write_list_update_delete_and_datasets() {
        let tmp = tempfile::tempdir().unwrap();
        let store = EpisodeStore::new(Some(tmp.path().to_path_buf()));
        store.load(&[0.0, 0.0], &[], None, 50.0).unwrap();
        let a = new_ulid();
        store.write_episode(&sidecar(&a, "default"), &frames(51), None).unwrap();
        assert!(tmp.path().join(format!("{a}.parquet")).exists() && tmp.path().join(format!("{a}.json")).exists());
        assert!(!tmp.path().join(format!("{a}.meta.json")).exists());
        assert_eq!(store.write_episode(&sidecar(&a, "default"), &frames(2), None).unwrap_err().status_code().1, "EPISODE_EXISTS");
        assert_eq!(store.create_dataset("pick", "").unwrap().name, "pick");
        assert_eq!(store.create_dataset("pick", "").unwrap_err().status_code().1, "DATASET_EXISTS");
        let b = new_ulid();
        store.write_episode(&sidecar(&b, "pick"), &frames(5), None).unwrap();
        let all = store.list_episodes(None, None, None, &[0.0, 0.0]);
        assert_eq!(all.iter().map(|e| e.sidecar.id.as_str()).collect::<Vec<_>>(), vec![b.as_str(), a.as_str()]);
        assert_eq!(store.list_episodes(None, Some(1), Some(&b), &[0.0, 0.0])[0].sidecar.id, a);
        assert_eq!(store.list_episodes(Some("pick"), None, None, &[0.0, 0.0]).len(), 1);
        assert!(store.list_episodes(None, None, None, &[0.1, 0.0]).iter().all(|e| e.stale));
        let ds = store.datasets();
        assert_eq!(ds.iter().map(|d| (d.name.as_str(), d.episodes)).collect::<Vec<_>>(), vec![("default", 1), ("pick", 1)]);
        assert_eq!(store.delete_dataset("pick", false).unwrap_err().status_code().1, "DATASET_NOT_EMPTY");
        assert_eq!(store.update(&a, Some("new task"), Some("nope")).unwrap_err().status_code().1, "DATASET_NOT_FOUND");
        assert_eq!(store.update(&a, Some("new task"), Some("pick")).unwrap().dataset, "pick");
        // reload from disk: the same catalog
        let again = EpisodeStore::new(Some(tmp.path().to_path_buf()));
        let r = again.load(&[0.0, 0.0], &[], None, 50.0).unwrap();
        assert_eq!(r.episodes, 2);
        assert_eq!(again.get(&a).unwrap().task, "new task");
        assert!(again.has_dataset("pick"));
        assert_eq!(store.delete_dataset("pick", true).unwrap().len(), 2);
        assert!(!store.has_dataset("pick") && store.list_episodes(None, None, None, &[]).is_empty());
        assert!(std::fs::read_dir(tmp.path()).unwrap().all(|e| e.unwrap().file_name() == DATASETS_FILE));
    }

    #[test]
    fn legacy_list_and_delete() {
        let tmp = tempfile::tempdir().unwrap();
        let store = EpisodeStore::new(Some(tmp.path().to_path_buf()));
        write_frames_to_parquet(&tmp.path().join("one.parquet"), &frames(51)).unwrap();
        write_frames_to_parquet(&tmp.path().join("two.parquet"), &frames(2)).unwrap();
        std::fs::write(tmp.path().join("two.meta.json"), b"{\"unit_mode\":\"so101_norm\"}").unwrap();
        std::fs::write(tmp.path().join("bad.parquet"), b"not parquet").unwrap();
        std::fs::write(tmp.path().join("notes.txt"), b"x").unwrap();
        let r = store.load(&[0.0, 0.0], &[], None, 50.0).unwrap();
        assert_eq!((r.legacy_adopted.len(), r.orphans_kept.clone()), (2, vec!["bad".to_string()]));
        let list = store.list().unwrap();
        assert_eq!(list.episodes.len(), 2);
        let one = list.episodes.iter().find(|e| e.id == "one").unwrap();
        assert_eq!((one.frames, one.dof), (51, 2));
        assert!((one.joint_min[0] - 0.0).abs() < 1e-6 && (one.joint_max[0] - 0.5).abs() < 1e-6);
        assert!(store.check_capacity().is_ok());
        assert_eq!(store.check_capacity_with(u64::MAX, 3).unwrap_err().status_code(), (507, "EPISODES_FULL"));
        store.delete("two").unwrap();
        assert!(!tmp.path().join("two.meta.json").exists() && !tmp.path().join("two.json").exists());
        assert_eq!(store.delete("two").unwrap_err().status_code(), (404, "EPISODE_NOT_FOUND"));
        assert_eq!(store.delete("../one").unwrap_err().status_code().1, "INVALID_EPISODE_ID");
    }

    #[test]
    fn boot_recovery_after_a_crash() {
        let tmp = tempfile::tempdir().unwrap();
        let dir = tmp.path();
        let store = EpisodeStore::new(Some(dir.to_path_buf()));
        store.load(&[0.0, 0.0], &[], None, 50.0).unwrap();
        // a complete episode, then its sidecar lost (crash after the parquet rename)
        let a = new_ulid();
        store.write_episode(&sidecar(&a, "default"), &frames(10), None).unwrap();
        std::fs::remove_file(dir.join(format!("{a}.json"))).unwrap();
        // a write that never finished (temp files left)
        let t = new_ulid();
        std::fs::write(dir.join(format!("{t}.parquet.tmp")), b"partial").unwrap();
        std::fs::write(dir.join(format!(".{t}.json.tmp-1-2")), b"{").unwrap();
        // a ULID parquet without sidecar that cannot be read: old -> deleted, new -> kept
        let old = new_ulid();
        let young = new_ulid();
        std::fs::write(dir.join(format!("{old}.parquet")), b"junk").unwrap();
        std::fs::write(dir.join(format!("{young}.parquet")), b"junk").unwrap();
        let f = std::fs::File::options().write(true).open(dir.join(format!("{old}.parquet"))).unwrap();
        f.set_modified(SystemTime::now() - Duration::from_secs(7200)).unwrap();
        // a sidecar without parquet
        let s = new_ulid();
        std::fs::write(dir.join(format!("{s}.json")), serde_json::to_string(&sidecar(&s, "default")).unwrap()).unwrap();

        let fresh = EpisodeStore::new(Some(dir.to_path_buf()));
        let r = fresh.load(&[0.0, 0.0], &[], None, 50.0).unwrap();
        println!("recovery: {r:?}");
        assert_eq!(r.sidecars_rebuilt, vec![a.clone()]);
        assert_eq!(r.temps_removed.len(), 2);
        assert_eq!(r.orphans_removed, vec![old.clone()]);
        assert_eq!(r.orphans_kept, vec![young.clone()]);
        assert_eq!(r.sidecars_removed, vec![s.clone()]);
        assert_eq!(r.episodes, 1);
        let back = fresh.get(&a).unwrap();
        assert_eq!((back.recovered.as_deref(), back.frames), (Some("sidecar"), 51));
        assert!(dir.join(format!("{a}.json")).exists());
        assert!(!dir.join(format!("{old}.parquet")).exists() && dir.join(format!("{young}.parquet")).exists());
        assert!(!dir.join(format!("{t}.parquet.tmp")).exists() && !dir.join(format!("{s}.json")).exists());
    }

    #[test]
    fn free_space_guard() {
        let tmp = tempfile::tempdir().unwrap();
        let store = EpisodeStore::new(Some(tmp.path().join("not/yet")));
        assert!(store.free_bytes().is_some_and(|b| b > 0));
        store.set_free_bytes_override(Some(START_MIN_FREE_BYTES - 1));
        assert_eq!(store.check_free_for_start().unwrap_err().status_code(), (507, "RECORD_NO_SPACE"));
        store.set_free_bytes_override(Some(START_MIN_FREE_BYTES));
        assert!(store.check_free_for_start().is_ok());
    }
}
