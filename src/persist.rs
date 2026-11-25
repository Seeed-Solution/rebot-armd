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
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
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

/// Durably create `dir` and any missing ancestor directories, syncing each newly created
/// directory link AND syncing every ancestor directory link up to `durable_root` (or the same
/// filesystem mount root when `durable_root` is `None`). Fails closed on every fsync error
/// (including `PermissionDenied`) so retries never report durability for an unsynced link.
pub(crate) fn create_dir_all_durable(dir: &Path) -> std::io::Result<()> {
    create_dir_all_durable_under(dir, None)
}

pub(crate) fn create_dir_all_durable_under(
    dir: &Path,
    durable_root: Option<&Path>,
) -> std::io::Result<()> {
    create_dir_all_durable_with(dir, durable_root, |d| std::fs::File::open(d)?.sync_all())
}

pub(crate) fn create_dir_all_durable_with(
    dir: &Path,
    durable_root: Option<&Path>,
    mut sync_dir: impl FnMut(&Path) -> std::io::Result<()>,
) -> std::io::Result<()> {
    if dir.as_os_str().is_empty() || dir == Path::new(".") {
        return Ok(());
    }
    let mut missing: Vec<PathBuf> = Vec::new();
    let mut cur = dir.to_path_buf();
    while !cur.as_os_str().is_empty() && cur != Path::new(".") && !cur.exists() {
        missing.push(cur.clone());
        match cur.parent() {
            Some(p) if !p.as_os_str().is_empty() && p != cur => cur = p.to_path_buf(),
            _ => break,
        }
    }
    for path in missing.into_iter().rev() {
        match std::fs::create_dir(&path) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {
                if !path.is_dir() {
                    return Err(e);
                }
            }
            Err(e) => return Err(e),
        }
        let parent = path
            .parent()
            .filter(|d| !d.as_os_str().is_empty())
            .unwrap_or(Path::new("."));
        sync_dir(parent).map_err(|e| {
            std::io::Error::new(
                e.kind(),
                format!("parent dir fsync after creating {}: {e}", path.display()),
            )
        })?;
    }
    if !dir.is_dir() {
        return Err(std::io::Error::new(
            std::io::ErrorKind::AlreadyExists,
            format!("{} exists but is not a directory", dir.display()),
        ));
    }
    // Walk up from the canonicalized target directory syncing every ancestor directory link up
    // to the explicit known-durable root (or the filesystem mount root) so a retry after an
    // earlier `mkdir` + parent-fsync failure on a higher ancestor still syncs or fails closed.
    let canonical = std::fs::canonicalize(dir)?;
    let canon_stop = match durable_root {
        Some(r) if r.is_dir() => {
            let cr = std::fs::canonicalize(r)?;
            if canonical.starts_with(&cr) {
                Some(cr)
            } else {
                None
            }
        }
        _ => None,
    };
    #[cfg(unix)]
    let target_dev = {
        use std::os::unix::fs::MetadataExt;
        std::fs::metadata(&canonical)?.dev()
    };
    let mut child = canonical.as_path();
    let mut depth = 0usize;
    while let Some(parent) = child.parent() {
        if parent.as_os_str().is_empty() || parent == child {
            break;
        }
        if canon_stop.is_none() {
            #[cfg(unix)]
            if depth > 0 {
                use std::os::unix::fs::MetadataExt;
                if std::fs::metadata(parent)?.dev() != target_dev {
                    break;
                }
            }
        }
        sync_dir(parent).map_err(|e| {
            std::io::Error::new(
                e.kind(),
                format!("parent dir fsync for {}: {e}", child.display()),
            )
        })?;
        if canon_stop.as_deref() == Some(parent) {
            break;
        }
        child = parent;
        depth += 1;
    }
    Ok(())
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

    /// The file that was renamed into place.
    pub fn target(&self) -> &Path {
        &self.target
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
        if TEST_STAGE_PAUSE_ARMED.load(Ordering::Acquire) {
            let hook = TEST_STAGE_PAUSE.get().and_then(|m| {
                let mut g = m.lock().unwrap_or_else(|e| e.into_inner());
                if g.as_ref().is_some_and(|(target, _, _)| *target == path) {
                    TEST_STAGE_PAUSE_ARMED.store(false, Ordering::Release);
                    g.take()
                } else {
                    None
                }
            });
            if let Some((_, entered_tx, release_rx)) = hook {
                let _ = entered_tx.send(());
                let _ = release_rx.recv();
            }
        }
        let delay = TEST_WRITE_DELAY_MS.load(Ordering::Relaxed);
        if delay > 0 {
            std::thread::sleep(std::time::Duration::from_millis(delay));
        }
        let dir = path
            .parent()
            .filter(|d| !d.as_os_str().is_empty())
            .unwrap_or(Path::new("."))
            .to_path_buf();
        if !dir.is_dir() {
            create_dir_all_durable(&dir)
                .map_err(|e| format!("create_dir_all_durable {}: {e}", dir.display()))?;
        }
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

/// Removes the temp files a [`StagedWrite`] of `path` left behind when the process ended
/// between staging and the rename (`.<name>.tmp-<pid>-<n>` next to `path`). Such a candidate
/// was never renamed into place, so it was never active nor durable: the file at `path` is
/// both. Call it at startup, before the first write to `path`. Returns the removed files.
pub fn remove_staged_leftovers(path: &Path) -> Vec<PathBuf> {
    let dir = path.parent().filter(|d| !d.as_os_str().is_empty()).unwrap_or(Path::new("."));
    let Some(name) = path.file_name().map(|n| n.to_string_lossy().into_owned()) else { return Vec::new() };
    let prefix = format!(".{name}.tmp-");
    let Ok(entries) = std::fs::read_dir(dir) else { return Vec::new() };
    let mut removed = Vec::new();
    for entry in entries.flatten() {
        let file = entry.file_name();
        if !file.to_string_lossy().starts_with(&prefix) { continue; }
        let p = entry.path();
        match std::fs::remove_file(&p) {
            Ok(()) => {
                tracing::warn!("Removed {} (a configuration candidate staged but never published)", p.display());
                removed.push(p);
            }
            Err(e) => tracing::warn!("Could not remove {}: {e}", p.display()),
        }
    }
    removed
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

type StagePauseHook = (PathBuf, mpsc::SyncSender<()>, mpsc::Receiver<()>);
static TEST_STAGE_PAUSE_ARMED: AtomicBool = AtomicBool::new(false);
static TEST_STAGE_PAUSE: OnceLock<std::sync::Mutex<Option<StagePauseHook>>> = OnceLock::new();

/// Test hook: pauses the next [`StagedWrite::stage`] call targeting `target_path` until the
/// returned release sender is signalled (or dropped), signalling `entered_rx` once staging starts.
#[doc(hidden)]
pub fn arm_test_stage_pause(target_path: PathBuf) -> (mpsc::Receiver<()>, mpsc::SyncSender<()>) {
    let (entered_tx, entered_rx) = mpsc::sync_channel(1);
    let (release_tx, release_rx) = mpsc::sync_channel(1);
    let slot = TEST_STAGE_PAUSE.get_or_init(|| std::sync::Mutex::new(None));
    *slot.lock().unwrap_or_else(|e| e.into_inner()) = Some((target_path, entered_tx, release_rx));
    TEST_STAGE_PAUSE_ARMED.store(true, Ordering::Release);
    (entered_rx, release_tx)
}

/// Clears any armed [`arm_test_stage_pause`] hook (for RAII test cleanup).
#[doc(hidden)]
pub fn clear_test_stage_pause() {
    TEST_STAGE_PAUSE_ARMED.store(false, Ordering::Release);
    if let Some(slot) = TEST_STAGE_PAUSE.get() {
        *slot.lock().unwrap_or_else(|e| e.into_inner()) = None;
    }
}

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

    #[test]
    fn create_dir_all_durable_retry_syncs_previously_created_ancestor_parent() {
        let root = tempfile::tempdir().unwrap();
        let state = root.path().join("state");
        std::fs::create_dir(&state).unwrap();
        let nested = state.join("new").join("nested");

        // Attempt 1: `mkdir state/new` succeeds, then `fsync(state)` fails before `nested` exists.
        let mut fail_once = true;
        let err = create_dir_all_durable_with(&nested, Some(&state), |d| {
            if fail_once && d == state {
                fail_once = false;
                return Err(std::io::Error::other("simulated parent dir fsync EIO"));
            }
            Ok(())
        })
        .expect_err("first attempt must fail on state fsync");
        assert!(err.to_string().contains("simulated parent dir fsync EIO"));
        assert!(state.join("new").is_dir());
        assert!(!nested.exists());

        // Attempt 2 (retry) with persistent PermissionDenied on `state`: MUST still fail closed
        // (never ignore PermissionDenied on a higher ancestor below durable_root).
        let canon_state = std::fs::canonicalize(&state).unwrap();
        let err_perm = create_dir_all_durable_with(&nested, Some(&state), |d| {
            if d == state || d == canon_state {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::PermissionDenied,
                    "simulated unreadable ancestor",
                ));
            }
            Ok(())
        })
        .expect_err("retry must fail closed on PermissionDenied for unsynced ancestor");
        assert_eq!(err_perm.kind(), std::io::ErrorKind::PermissionDenied);
        assert!(nested.is_dir());

        // Attempt 3 (retry after permissions restored): MUST sync both `state/new` AND `state`,
        // and stop cleanly at `state` (known-durable root).
        let mut synced: Vec<PathBuf> = Vec::new();
        create_dir_all_durable_with(&nested, Some(&state), |d| {
            synced.push(d.to_path_buf());
            Ok(())
        })
        .expect("retry should succeed once state is syncable");
        let canon_new = std::fs::canonicalize(state.join("new")).unwrap();
        assert_eq!(synced, vec![canon_new, canon_state]);
    }
}
