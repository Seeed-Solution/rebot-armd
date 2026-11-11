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

static TMP_COUNTER: AtomicU64 = AtomicU64::new(0);

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
