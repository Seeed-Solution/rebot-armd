//! Small JSON state files under `/userdata` (keep-out boxes, named poses, collision
//! setting, grasp setup).
//!
//! Writes are prepared under the supervisor lock ([`PendingWrite::new`] takes a revision
//! from one global counter, so revisions follow the order of the state changes) and carried
//! out by a single writer thread in the order they are queued. A write whose revision is
//! older than the last one written to the same file is skipped, so an older state never
//! overwrites a newer one whatever order the callers queue them in.

use std::collections::HashMap;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{mpsc, OnceLock};

/// `env_var` when set, else `default`.
pub fn path_from_env(env_var: &str, default: &str) -> PathBuf {
    std::env::var_os(env_var).map(PathBuf::from).unwrap_or_else(|| PathBuf::from(default))
}

/// Parse optional `expected_revision` from a JSON command body.
/// When `require` is true, missing or null `expected_revision` is rejected.
pub fn parse_expected_revision(
    body: &serde_json::Value,
    require: bool,
) -> Result<Option<u64>, String> {
    match body.get("expected_revision") {
        None | Some(serde_json::Value::Null) => {
            if require {
                Err("expected_revision is required when operator auth is enforced".into())
            } else {
                Ok(None)
            }
        }
        Some(v) => v
            .as_u64()
            .map(Some)
            .ok_or_else(|| "expected_revision must be a non-negative integer".into()),
    }
}

static TMP_COUNTER: AtomicU64 = AtomicU64::new(0);

/// Outcome of publishing a [`StagedWrite`] to its target path.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PublishOutcome {
    Published,
    /// `rename` succeeded (official file now holds the new content), but `sync_dir` failed.
    PublishedDurabilityUnconfirmed(String),
}

/// Parent directory handle returned by [`StagedWrite::rename_only`] so directory
/// `sync_all()` can run after releasing the supervisor write lock.
#[derive(Debug, Clone)]
pub struct PendingDirSync {
    target: PathBuf,
    dir: PathBuf,
}

impl PendingDirSync {
    pub fn for_path(target: &Path) -> Self {
        let dir = target
            .parent()
            .filter(|d| !d.as_os_str().is_empty())
            .unwrap_or(Path::new("."))
            .to_path_buf();
        Self {
            target: target.to_path_buf(),
            dir,
        }
    }

    pub fn sync_parent_dir(self) -> PublishOutcome {
        self.sync_with(|dir| std::fs::File::open(dir)?.sync_all())
    }

    pub(crate) fn sync_with(self, sync_dir: impl FnOnce(&Path) -> std::io::Result<()>) -> PublishOutcome {
        match sync_dir(&self.dir) {
            Ok(()) => PublishOutcome::Published,
            Err(e) => {
                let err = e.to_string();
                tracing::warn!(
                    "Published {} but directory sync on {} failed: {err}",
                    self.target.display(),
                    self.dir.display()
                );
                PublishOutcome::PublishedDurabilityUnconfirmed(err)
            }
        }
    }
}

/// Two-phase atomic file write:
/// 1. [`StagedWrite::stage`] writes and `sync_all()`s a unique temp file in the target's parent
///    directory, leaving the official file untouched.
/// 2. [`StagedWrite::rename_only`] (under supervisor lock) atomically renames the temp file over
///    the target, returning [`PendingDirSync`] so [`PendingDirSync::sync_parent_dir`] runs outside
///    the lock; or [`StagedWrite::abort`] removes the temp file without touching the official file.
#[derive(Debug)]
pub struct StagedWrite {
    target: PathBuf,
    dir: PathBuf,
    tmp: PathBuf,
    active: bool,
}

impl StagedWrite {
    /// Phase 1 (outside supervisor lock): create a unique temp file in `path`'s parent directory,
    /// write `content`, and `sync_all()`. Official file is untouched.
    pub fn stage(path: PathBuf, content: String) -> Result<StagedWrite, String> {
        let delay = TEST_WRITE_DELAY_MS.load(Ordering::Relaxed);
        if delay > 0 {
            std::thread::sleep(std::time::Duration::from_millis(delay));
        }
        let dir = path
            .parent()
            .filter(|d| !d.as_os_str().is_empty())
            .unwrap_or(Path::new("."))
            .to_path_buf();
        std::fs::create_dir_all(&dir).map_err(|e| format!("create_dir_all {}: {e}", dir.display()))?;
        let name = path
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_else(|| "state".into());
        let tmp = dir.join(format!(
            ".{name}.tmp-{}-{}",
            std::process::id(),
            TMP_COUNTER.fetch_add(1, Ordering::Relaxed)
        ));
        let res = (|| -> std::io::Result<()> {
            let mut file = std::fs::File::create(&tmp)?;
            file.write_all(content.as_bytes())?;
            file.sync_all()?;
            drop(file);
            Ok(())
        })();
        if let Err(e) = res {
            let _ = std::fs::remove_file(&tmp);
            return Err(format!("stage {}: {e}", tmp.display()));
        }
        Ok(Self {
            target: path,
            dir,
            tmp,
            active: true,
        })
    }

    pub fn target_path(&self) -> &Path {
        &self.target
    }

    pub fn tmp_path(&self) -> &Path {
        &self.tmp
    }

    /// Phase 2a (under supervisor lock after prechecks pass): atomically rename `tmp` over
    /// `target` without blocking the lock on parent-directory `sync_all()`.
    pub fn rename_only(mut self) -> Result<PendingDirSync, String> {
        self.active = false;
        if let Err(e) = std::fs::rename(&self.tmp, &self.target) {
            let _ = std::fs::remove_file(&self.tmp);
            return Err(format!(
                "rename {} -> {}: {e}",
                self.tmp.display(),
                self.target.display()
            ));
        }
        Ok(PendingDirSync {
            target: self.target.clone(),
            dir: self.dir.clone(),
        })
    }

    /// Phase 2 (convenience for non-realtime paths): atomically rename `tmp` over `target`
    /// and sync the parent directory.
    pub fn publish(self) -> Result<PublishOutcome, String> {
        self.publish_with_dir_sync(|dir| std::fs::File::open(dir)?.sync_all())
    }

    pub(crate) fn publish_with_dir_sync(
        self,
        sync_dir: impl FnOnce(&Path) -> std::io::Result<()>,
    ) -> Result<PublishOutcome, String> {
        let dir_sync = self.rename_only()?;
        Ok(dir_sync.sync_with(sync_dir))
    }

    /// Discard the staged temp file without touching the official target file.
    pub fn abort(mut self) {
        if self.active {
            self.active = false;
            let _ = std::fs::remove_file(&self.tmp);
        }
    }
}

impl Drop for StagedWrite {
    fn drop(&mut self) {
        if self.active {
            self.active = false;
            let _ = std::fs::remove_file(&self.tmp);
        }
    }
}

/// Write `text` atomically and return [`PublishOutcome`], distinguishing pre-rename failures
/// (`Err`) from post-rename parent-directory fsync failures (`Ok(PublishedDurabilityUnconfirmed)`).
pub fn write_atomic_outcome(path: &Path, text: &str) -> Result<PublishOutcome, String> {
    StagedWrite::stage(path.to_path_buf(), text.to_string())?.publish()
}

/// Write `text` atomically and durably: unique temp file in the same directory, fsync,
/// rename over `path`, fsync the directory. Creates the directory.
pub fn write_atomic(path: &Path, text: &str) -> std::io::Result<()> {
    let dir = path.parent().filter(|d| !d.as_os_str().is_empty()).unwrap_or(Path::new("."));
    std::fs::create_dir_all(dir)?;
    let name = path.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_else(|| "state".into());
    let tmp = dir.join(format!(".{name}.tmp-{}-{}", std::process::id(), TMP_COUNTER.fetch_add(1, Ordering::Relaxed)));
    let result = (|| {
        let mut file = std::fs::File::create(&tmp)?;
        file.write_all(text.as_bytes())?;
        file.sync_all()?;
        drop(file);
        std::fs::rename(&tmp, path)?;
        std::fs::File::open(dir)?.sync_all()
    })();
    if result.is_err() { let _ = std::fs::remove_file(&tmp); }
    result
}

/// Result of reading a state file.
#[derive(Debug, Clone, PartialEq)]
pub enum Loaded<T> {
    Loaded(T),
    Missing,
    /// Unreadable, malformed or invalid. A malformed or invalid file has been renamed to
    /// `<name>.bad-<unix seconds>` so that no later write overwrites it.
    Invalid(String),
}

/// Move an invalid state file aside (`<name>.bad-<unix seconds>`). Returns the new path.
pub fn quarantine(path: &Path) -> std::io::Result<PathBuf> {
    let ts = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs());
    let name = path.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
    let mut bad = path.with_file_name(format!("{name}.bad-{ts}"));
    let mut k = 1;
    while bad.exists() {
        bad = path.with_file_name(format!("{name}.bad-{ts}-{k}"));
        k += 1;
    }
    std::fs::rename(path, &bad)?;
    Ok(bad)
}

/// Read and parse `path`, then check it with `validate`. A file that does not parse or
/// validate is moved aside with [`quarantine`].
pub fn load<T: serde::de::DeserializeOwned>(path: &Path, validate: impl FnOnce(&T) -> Result<(), String>) -> Loaded<T> {
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Loaded::Missing,
        Err(e) => return Loaded::Invalid(format!("read {}: {e}", path.display())),
    };
    let problem = match serde_json::from_str::<T>(&text) {
        Ok(v) => match validate(&v) {
            Ok(()) => return Loaded::Loaded(v),
            Err(e) => format!("{}: {e}", path.display()),
        },
        Err(e) => format!("parse {}: {e}", path.display()),
    };
    match quarantine(path) {
        Ok(bad) => Loaded::Invalid(format!("{problem}; moved to {}", bad.display())),
        Err(e) => Loaded::Invalid(format!("{problem}; could not move it aside: {e}")),
    }
}

static REVISION: AtomicU64 = AtomicU64::new(0);

/// Extra delay (ms) before each write on the writer thread; 0 in normal use. Tests set it to
/// make "the ack follows later replies" deterministic.
pub static TEST_WRITE_DELAY_MS: AtomicU64 = AtomicU64::new(0);

/// A write prepared under the supervisor lock and carried out after it is released.
#[derive(Debug, Clone)]
pub struct PendingWrite {
    pub path: PathBuf,
    pub text: String,
    /// Global, increasing in creation order.
    pub revision: u64,
}

struct Job {
    write: PendingWrite,
    done: tokio::sync::oneshot::Sender<Result<(), String>>,
}

fn writer() -> &'static std::sync::Mutex<mpsc::Sender<Job>> {
    static WRITER: OnceLock<std::sync::Mutex<mpsc::Sender<Job>>> = OnceLock::new();
    WRITER.get_or_init(|| {
        let (tx, rx) = mpsc::channel::<Job>();
        std::thread::Builder::new().name("armd-persist".into()).spawn(move || {
            let mut written: HashMap<PathBuf, u64> = HashMap::new();
            for Job { write, done } in rx {
                let delay = TEST_WRITE_DELAY_MS.load(Ordering::Relaxed);
                if delay > 0 { std::thread::sleep(std::time::Duration::from_millis(delay)); }
                let last = written.get(&write.path).copied();
                let result = if last.is_some_and(|r| r > write.revision) {
                    Ok(()) // a newer state of this file is already on disk
                } else {
                    let r = write_atomic(&write.path, &write.text).map_err(|e| e.to_string());
                    if r.is_ok() { written.insert(write.path.clone(), write.revision); }
                    r
                };
                let _ = done.send(result);
            }
        }).expect("spawn persist writer");
        std::sync::Mutex::new(tx)
    })
}

impl PendingWrite {
    pub fn new(path: PathBuf, text: String) -> Self {
        PendingWrite { path, text, revision: REVISION.fetch_add(1, Ordering::SeqCst) + 1 }
    }

    /// Queue on the writer thread and wait for it; Ok(()) or the error text.
    pub async fn run(self) -> Result<(), String> {
        let (done, wait) = tokio::sync::oneshot::channel();
        writer().lock().unwrap_or_else(|e| e.into_inner()).send(Job { write: self, done })
            .map_err(|_| "persist writer stopped".to_string())?;
        wait.await.map_err(|_| "persist writer stopped".to_string())?
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn staged_write_abort_and_dir_sync_failure_behavior() {
        let dir = tempfile::tempdir().unwrap();
        let target = dir.path().join("cfg.json");
        std::fs::write(&target, "{\"old\":true}").unwrap();

        // Stage leaves official file untouched; abort removes temp file
        let staged = StagedWrite::stage(target.clone(), "{\"new\":true}".into()).unwrap();
        assert!(staged.tmp_path().exists());
        assert_eq!(std::fs::read_to_string(&target).unwrap(), "{\"old\":true}");
        let tmp_path = staged.tmp_path().to_path_buf();
        staged.abort();
        assert!(!tmp_path.exists());
        assert_eq!(std::fs::read_to_string(&target).unwrap(), "{\"old\":true}");

        // Publish with directory sync failure still returns Ok(PublishedDurabilityUnconfirmed)
        let staged2 = StagedWrite::stage(target.clone(), "{\"new\":2}".into()).unwrap();
        let tmp2 = staged2.tmp_path().to_path_buf();
        let outcome = staged2
            .publish_with_dir_sync(|_| Err(std::io::Error::other("simulated sync_dir EIO")))
            .unwrap();
        assert!(matches!(
            outcome,
            PublishOutcome::PublishedDurabilityUnconfirmed(ref msg) if msg.contains("simulated sync_dir EIO")
        ));
        assert!(!tmp2.exists());
        assert_eq!(std::fs::read_to_string(&target).unwrap(), "{\"new\":2}");

        // Split rename_only() + sync_parent_dir() allows releasing supervisor locks before dir fsync
        let staged3 = StagedWrite::stage(target.clone(), "{\"new\":3}".into()).unwrap();
        let dir_sync3 = staged3.rename_only().unwrap();
        assert_eq!(std::fs::read_to_string(&target).unwrap(), "{\"new\":3}");
        assert_eq!(dir_sync3.sync_parent_dir(), PublishOutcome::Published);
    }
}
