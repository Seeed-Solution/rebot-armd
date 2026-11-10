//! Small JSON state files under `/userdata` (keep-out boxes, named poses).

use std::path::{Path, PathBuf};

/// `env_var` when set, else `default`.
pub fn path_from_env(env_var: &str, default: &str) -> PathBuf {
    std::env::var_os(env_var).map(PathBuf::from).unwrap_or_else(|| PathBuf::from(default))
}

/// Write `text` atomically (temp file + rename), creating the directory.
pub fn write_atomic(path: &Path, text: &str) -> std::io::Result<()> {
    if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty()) { std::fs::create_dir_all(dir)?; }
    let tmp = path.with_extension("json.tmp");
    std::fs::write(&tmp, text)?;
    std::fs::rename(&tmp, path)
}

/// Result of reading a state file.
#[derive(Debug, Clone, PartialEq)]
pub enum Loaded<T> {
    Loaded(T),
    Missing,
    /// Unreadable, malformed or invalid; the file is left untouched.
    Invalid(String),
}

/// Read and parse `path`, then check it with `validate`.
pub fn load<T: serde::de::DeserializeOwned>(path: &Path, validate: impl FnOnce(&T) -> Result<(), String>) -> Loaded<T> {
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Loaded::Missing,
        Err(e) => return Loaded::Invalid(format!("read {}: {e}", path.display())),
    };
    let value: T = match serde_json::from_str(&text) {
        Ok(v) => v,
        Err(e) => return Loaded::Invalid(format!("parse {}: {e}", path.display())),
    };
    match validate(&value) {
        Ok(()) => Loaded::Loaded(value),
        Err(e) => Loaded::Invalid(format!("{}: {e}", path.display())),
    }
}

/// A write prepared under the supervisor lock and carried out after it is released.
#[derive(Debug, Clone)]
pub struct PendingWrite {
    pub path: PathBuf,
    pub text: String,
}

impl PendingWrite {
    /// Write on a blocking thread; Ok(()) or the error text.
    pub async fn run(self) -> Result<(), String> {
        match tokio::task::spawn_blocking(move || write_atomic(&self.path, &self.text)).await {
            Ok(Ok(())) => Ok(()),
            Ok(Err(e)) => Err(e.to_string()),
            Err(e) => Err(e.to_string()),
        }
    }
}
