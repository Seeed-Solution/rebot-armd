//! Episode directory: file names, path checks, size cap, listing and deletion.
//!
//! Every episode is `<dir>/<id>.parquet` with `id` matching `^[A-Za-z0-9_-]{1,64}$`.
//! A client-supplied `output_path` must resolve to such a file directly inside `dir`.
use std::collections::HashMap;
use std::path::{Component, Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::SystemTime;

use serde::{Deserialize, Serialize};
use thiserror::Error;

use super::parquet_writer::read_frames_from_parquet;

/// Default episode directory (used when `/userdata` exists).
pub const DEFAULT_EPISODES_DIR: &str = "/userdata/rebot-teleop/episodes";
/// Episode directory when `/userdata` does not exist (relative to armd's working directory).
pub const FALLBACK_EPISODES_DIR: &str = "./data/episodes";
/// Environment variable read when `--episodes-dir` is not given.
pub const EPISODES_DIR_ENV: &str = "ARMD_EPISODES_DIR";
/// Saving (and starting) a recording is refused while the directory holds this many bytes.
pub const MAX_EPISODES_BYTES: u64 = 200 * 1024 * 1024;
/// Saving (and starting) a recording is refused while the directory holds this many episodes.
pub const MAX_EPISODE_FILES: usize = 500;
pub const EPISODE_EXT: &str = "parquet";
pub const MAX_EPISODE_ID_LEN: usize = 64;

/// `^[A-Za-z0-9_-]{1,64}$`
pub fn valid_episode_id(id: &str) -> bool {
    !id.is_empty() && id.len() <= MAX_EPISODE_ID_LEN && id.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-')
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
    #[error("Episode directory I/O error: {0}")]
    Io(#[from] std::io::Error),
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
            Self::Io(_) => (500, "EPISODES_IO_ERROR"),
        }
    }
}

/// One saved episode in `GET /api/v1/dataset/episodes`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EpisodeInfo {
    pub id: String,
    pub frames: usize,
    /// Last minus first timestamp (s).
    pub duration_s: f64,
    pub size_bytes: u64,
    /// File modification time (Unix ms); armd writes the file once, at save.
    pub created_ms: u64,
    /// Joints per frame (`observation.state` length).
    pub dof: usize,
    /// Per joint: minimum and maximum of `observation.state` over the episode (rad).
    pub joint_min: Vec<f32>,
    pub joint_max: Vec<f32>,
    /// Set when the file could not be read; the other fields are then zero / empty.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

/// `GET /api/v1/dataset/episodes` body.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EpisodeList {
    pub enabled: bool,
    pub dir: Option<String>,
    pub episodes: Vec<EpisodeInfo>,
    pub total_bytes: u64,
    pub max_bytes: u64,
    pub max_files: usize,
}

type SummaryCache = Arc<Mutex<HashMap<String, (SystemTime, u64, EpisodeInfo)>>>;

/// Episode directory; `dir == None` means storage is off.
#[derive(Debug, Clone, Default)]
pub struct EpisodeStore {
    dir: Option<PathBuf>,
    cache: SummaryCache,
}

impl EpisodeStore {
    pub fn new(dir: Option<PathBuf>) -> Self { Self { dir, cache: SummaryCache::default() } }

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

    fn require_dir(&self) -> Result<&Path, StoreError> { self.dir.as_deref().ok_or(StoreError::Disabled) }

    /// `<dir>/<id>.parquet` for a valid `id`.
    pub fn path_of(&self, id: &str) -> Result<PathBuf, StoreError> {
        let dir = self.require_dir()?;
        if !valid_episode_id(id) { return Err(StoreError::InvalidId(id.into())); }
        Ok(dir.join(format!("{id}.{EPISODE_EXT}")))
    }

    /// Episode id and file for a new recording. `client_path` (absolute, or relative to the
    /// episode directory) must name `<id>.parquet` directly inside the directory; its stem
    /// is the id and must equal `episode_id` when both are given. Without either, the id is
    /// `episode_<UTC date>_<time>` (with `_2`, `_3`... when taken). An existing file is
    /// [`StoreError::Exists`].
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
                None => {
                    let base = format!("episode_{}", chrono::Utc::now().format("%Y%m%d_%H%M%S"));
                    let mut id = base.clone();
                    let mut n = 2;
                    while dir.join(format!("{id}.{EPISODE_EXT}")).exists() { id = format!("{base}_{n}"); n += 1; }
                    id
                }
            },
        };
        let path = dir.join(format!("{id}.{EPISODE_EXT}"));
        if path.exists() { return Err(StoreError::Exists(id)); }
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

    /// [`StoreError::Full`] when the directory is at either cap.
    pub fn check_capacity(&self) -> Result<(), StoreError> {
        self.check_capacity_with(MAX_EPISODES_BYTES, MAX_EPISODE_FILES)
    }

    pub fn check_capacity_with(&self, max_bytes: u64, max_files: usize) -> Result<(), StoreError> {
        let (bytes, files) = self.usage()?;
        if bytes >= max_bytes || files >= max_files {
            return Err(StoreError::Full { files, bytes, max_files, max_bytes });
        }
        Ok(())
    }

    /// Saved episodes, newest first. Summaries are cached by (mtime, size).
    pub fn list(&self) -> Result<EpisodeList, StoreError> {
        let Some(dir) = self.dir.as_deref() else {
            return Ok(EpisodeList { enabled: false, dir: None, episodes: Vec::new(), total_bytes: 0, max_bytes: MAX_EPISODES_BYTES, max_files: MAX_EPISODE_FILES });
        };
        let mut episodes = Vec::new();
        let mut total_bytes = 0;
        let mut cache = self.cache.lock().unwrap_or_else(|e| e.into_inner());
        let mut seen = Vec::new();
        for (id, meta) in episode_files(dir)? {
            total_bytes += meta.len();
            let mtime = meta.modified().unwrap_or(SystemTime::UNIX_EPOCH);
            let info = match cache.get(&id) {
                Some((t, size, info)) if *t == mtime && *size == meta.len() => info.clone(),
                _ => {
                    let info = summarize(&id, &dir.join(format!("{id}.{EPISODE_EXT}")), meta.len(), mtime);
                    cache.insert(id.clone(), (mtime, meta.len(), info.clone()));
                    info
                }
            };
            seen.push(id);
            episodes.push(info);
        }
        cache.retain(|id, _| seen.contains(id));
        episodes.sort_by(|a, b| b.created_ms.cmp(&a.created_ms).then_with(|| a.id.cmp(&b.id)));
        Ok(EpisodeList { enabled: true, dir: Some(dir.display().to_string()), episodes, total_bytes, max_bytes: MAX_EPISODES_BYTES, max_files: MAX_EPISODE_FILES })
    }

    /// Remove `<dir>/<id>.parquet` and any sidecar `<dir>/<id>.meta.json`.
    pub fn delete(&self, id: &str) -> Result<(), StoreError> {
        let path = self.path_of(id)?;
        match std::fs::remove_file(&path) {
            Ok(()) => {
                self.cache.lock().unwrap_or_else(|e| e.into_inner()).remove(id);
                let meta_path = path.with_extension("meta.json");
                if let Err(e) = std::fs::remove_file(&meta_path) {
                    if e.kind() != std::io::ErrorKind::NotFound {
                        tracing::warn!("Failed to remove episode sidecar {}: {}", meta_path.display(), e);
                    }
                }
                Ok(())
            }
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Err(StoreError::NotFound(id.into())),
            Err(e) => Err(e.into()),
        }
    }
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

fn summarize(id: &str, path: &Path, size_bytes: u64, mtime: SystemTime) -> EpisodeInfo {
    let created_ms = mtime.duration_since(SystemTime::UNIX_EPOCH).map_or(0, |d| d.as_millis() as u64);
    let mut info = EpisodeInfo {
        id: id.into(), frames: 0, duration_s: 0.0, size_bytes, created_ms, dof: 0,
        joint_min: Vec::new(), joint_max: Vec::new(), error: None,
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

    #[test]
    fn episode_id_rule() {
        for ok in ["a", "ep_1", "Episode-20261112_101010", &"x".repeat(64)] { assert!(valid_episode_id(ok), "{ok}"); }
        for bad in ["", "a b", "../x", "a/b", "a.parquet", "é", &"x".repeat(65), "a\0", "."] { assert!(!valid_episode_id(bad), "{bad:?}"); }
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
        assert_eq!(store.new_episode(Some("d"), Some(&dir.join("d.parquet"))).unwrap().0, "d");
        let code = |r: Result<(String, PathBuf), StoreError>| r.unwrap_err().status_code().1;
        assert_eq!(code(store.new_episode(None, Some(Path::new("../x.parquet")))), "INVALID_OUTPUT_PATH");
        assert_eq!(code(store.new_episode(None, Some(&dir.join("..").join("episodes").join("x.parquet")))), "INVALID_OUTPUT_PATH");
        assert_eq!(code(store.new_episode(None, Some(&tmp.path().join("x.parquet")))), "INVALID_OUTPUT_PATH");
        assert_eq!(code(store.new_episode(None, Some(Path::new("/etc/x.parquet")))), "INVALID_OUTPUT_PATH");
        assert_eq!(code(store.new_episode(None, Some(Path::new("sub/x.parquet")))), "INVALID_OUTPUT_PATH");
        assert_eq!(code(store.new_episode(None, Some(Path::new("x.txt")))), "INVALID_OUTPUT_PATH");
        assert_eq!(code(store.new_episode(None, Some(Path::new("a b.parquet")))), "INVALID_EPISODE_ID");
        assert_eq!(code(store.new_episode(Some("e"), Some(Path::new("f.parquet")))), "INVALID_OUTPUT_PATH");
        assert_eq!(code(store.new_episode(Some("../e"), None)), "INVALID_EPISODE_ID");
        // an existing file is not overwritten
        write_frames_to_parquet(&dir.join("a.parquet"), &frames(3)).unwrap();
        assert_eq!(code(store.new_episode(Some("a"), None)), "EPISODE_EXISTS");
        // generated ids are unique
        let (g1, p1) = store.new_episode(None, None).unwrap();
        write_frames_to_parquet(&p1, &frames(3)).unwrap();
        let (g2, _) = store.new_episode(None, None).unwrap();
        assert!(g1.starts_with("episode_") && g1 != g2 && valid_episode_id(&g2), "{g1} {g2}");
        // relative directory (the fallback) works with relative and absolute paths
        let rel = EpisodeStore::new(Some(PathBuf::from("./data/episodes")));
        assert_eq!(rel.new_episode(None, Some(Path::new("r.parquet"))).unwrap().0, "r");
        let abs = std::path::absolute("data/episodes/r2.parquet").unwrap();
        assert_eq!(rel.new_episode(None, Some(&abs)).unwrap().0, "r2");
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
    fn list_capacity_and_delete() {
        let tmp = tempfile::tempdir().unwrap();
        let store = EpisodeStore::new(Some(tmp.path().to_path_buf()));
        assert!(store.list().unwrap().episodes.is_empty());
        write_frames_to_parquet(&tmp.path().join("one.parquet"), &frames(51)).unwrap();
        write_frames_to_parquet(&tmp.path().join("two.parquet"), &frames(2)).unwrap();
        std::fs::write(tmp.path().join("two.meta.json"), b"{\"unit_mode\":\"so101_norm\"}").unwrap();
        std::fs::write(tmp.path().join("bad.parquet"), b"not parquet").unwrap();
        std::fs::write(tmp.path().join("notes.txt"), b"x").unwrap();
        std::fs::write(tmp.path().join("a b.parquet"), b"x").unwrap();
        let list = store.list().unwrap();
        let ids: Vec<_> = list.episodes.iter().map(|e| e.id.as_str()).collect();
        assert_eq!(ids.len(), 3, "{ids:?}");
        let one = list.episodes.iter().find(|e| e.id == "one").unwrap();
        assert_eq!((one.frames, one.dof), (51, 2));
        assert!((one.duration_s - 1.0).abs() < 1e-9);
        assert!((one.joint_min[0] - 0.0).abs() < 1e-6 && (one.joint_max[0] - 0.5).abs() < 1e-6 && one.joint_min[1] == -0.5);
        assert!(one.size_bytes > 0 && one.created_ms > 0 && one.error.is_none());
        assert!(list.episodes.iter().find(|e| e.id == "bad").unwrap().error.is_some());
        assert_eq!(list.total_bytes, store.usage().unwrap().0);
        assert_eq!(store.usage().unwrap().1, 3);

        assert!(store.check_capacity().is_ok());
        let full = store.check_capacity_with(u64::MAX, 3).unwrap_err();
        assert_eq!(full.status_code(), (507, "EPISODES_FULL"));
        assert!(store.check_capacity_with(10, 100).is_err());

        assert!(tmp.path().join("two.meta.json").exists());
        store.delete("two").unwrap();
        assert!(!tmp.path().join("two.meta.json").exists());
        assert_eq!(store.delete("two").unwrap_err().status_code(), (404, "EPISODE_NOT_FOUND"));
        assert_eq!(store.delete("../one").unwrap_err().status_code().1, "INVALID_EPISODE_ID");
        assert!(tmp.path().join("one.parquet").exists());
        assert_eq!(store.list().unwrap().episodes.len(), 2);
    }
}
