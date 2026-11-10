//! Persistent diagnostics: the rotating armd log, the black box (state recorder) and the
//! boot record, all under `/userdata` so they survive a reboot or a power cut.

pub mod blackbox;
pub mod boot;
pub mod rotating;

use std::path::{Path, PathBuf};

use rotating::{files_oldest_first, RotatingFile};

/// Default log directory (used when `/userdata` exists).
pub const DEFAULT_LOG_DIR: &str = "/userdata/rebot-teleop/logs";
/// Default black-box directory (used when `/userdata` exists).
pub const DEFAULT_BLACKBOX_DIR: &str = "/userdata/rebot-teleop/blackbox";
pub const LOG_FILE: &str = "armd.log";
pub const LOG_MAX_BYTES: u64 = 1024 * 1024;
/// `armd.log.1`, `armd.log.2`.
pub const LOG_KEEP: usize = 2;
/// Upper bound of a diagnostics response body.
pub const MAX_RESPONSE_BYTES: usize = 2 * 1024 * 1024;

/// A directory option: the CLI value, else the environment variable, else `default` when
/// `/userdata` exists. `off`, `none` or an empty value disables it.
pub fn resolve_dir(cli: Option<&str>, env_var: &str, default: &str) -> Option<PathBuf> {
    let value = cli.map(str::to_string).or_else(|| std::env::var(env_var).ok());
    match value.as_deref().map(str::trim) {
        Some("" | "off" | "none") => None,
        Some(v) => Some(PathBuf::from(v)),
        None => Path::new("/userdata").is_dir().then(|| PathBuf::from(default)),
    }
}

/// Creates `dir` and checks it is writable.
pub fn prepare_dir(dir: &Path) -> std::io::Result<()> {
    std::fs::create_dir_all(dir)?;
    let probe = dir.join(".write_test");
    std::fs::write(&probe, b"")?;
    std::fs::remove_file(probe)
}

/// Opens `dir/armd.log` for the tracing file layer.
pub fn open_log(dir: &Path) -> std::io::Result<RotatingFile> {
    prepare_dir(dir)?;
    RotatingFile::open(dir.join(LOG_FILE), LOG_MAX_BYTES, LOG_KEEP)
}

/// What the diagnostics routes read. `blackbox_dir` None: disabled.
#[derive(Debug, Clone, Default)]
pub struct DiagnosticsState {
    pub blackbox_dir: Option<PathBuf>,
    /// This armd start's counter (0 when the boot record could not be written).
    pub boot: u64,
}

static GLOBAL: std::sync::OnceLock<DiagnosticsState> = std::sync::OnceLock::new();

impl DiagnosticsState {
    /// Set once by `main` so [`crate::create_router`] serves this armd's files.
    pub fn set_global(state: DiagnosticsState) {
        let _ = GLOBAL.set(state);
    }

    pub fn global() -> DiagnosticsState {
        GLOBAL.get().cloned().unwrap_or_default()
    }
}

/// Lines of the files `path`, `path.1` … `path.<keep>`, oldest first; lines that are not
/// a JSON object (a line cut by a power loss) are skipped.
fn json_lines(path: &Path, keep: usize) -> Vec<String> {
    let mut out = Vec::new();
    for p in files_oldest_first(path, keep) {
        let Ok(text) = std::fs::read_to_string(&p) else { continue };
        out.extend(text.lines().filter(|l| l.starts_with('{') && serde_json::from_str::<serde::de::IgnoredAny>(l).is_ok()).map(str::to_string));
    }
    out
}

/// Keeps the newest lines whose total size fits `MAX_RESPONSE_BYTES`; returns whether
/// older ones were cut.
fn cap_newest(lines: &mut Vec<String>) -> bool {
    let mut total = 0usize;
    let mut first = lines.len();
    for (i, l) in lines.iter().enumerate().rev() {
        if total + l.len() + 1 > MAX_RESPONSE_BYTES - 256 {
            break;
        }
        total += l.len() + 1;
        first = i;
    }
    let cut = first > 0;
    lines.drain(..first);
    cut
}

fn body(head: serde_json::Value, key: &str, lines: &[String]) -> String {
    // `head` is an object; splice the raw lines in as an array without re-encoding them.
    let mut s = head.to_string();
    s.pop();
    s.push_str(&format!(",\"{key}\":["));
    s.push_str(&lines.join(","));
    s.push_str("]}");
    s
}

/// JSON body of `GET /api/v1/diagnostics/boots`.
pub fn boots_body(state: &DiagnosticsState) -> String {
    let Some(dir) = &state.blackbox_dir else {
        return body(serde_json::json!({"enabled": false, "boot": state.boot}), "boots", &[]);
    };
    let mut lines = json_lines(&dir.join(boot::BOOTS_FILE), boot::BOOTS_KEEP);
    let truncated = cap_newest(&mut lines);
    body(serde_json::json!({"enabled": true, "boot": state.boot, "truncated": truncated}), "boots", &lines)
}

#[derive(serde::Deserialize)]
struct Stamp {
    boot: u64,
    t_ms: u64,
}

/// JSON body of `GET /api/v1/diagnostics/blackbox`: the last `seconds` of records of this
/// start (`prev` false) or of the previous one (`prev` true), oldest first.
pub fn blackbox_body(state: &DiagnosticsState, seconds: u64, prev: bool) -> String {
    let target = if prev { state.boot.saturating_sub(1) } else { state.boot };
    let head = |enabled: bool, truncated: bool| serde_json::json!({"enabled": enabled, "boot": target, "current_boot": state.boot, "seconds": seconds, "truncated": truncated});
    let Some(dir) = &state.blackbox_dir else {
        return body(head(false, false), "records", &[]);
    };
    let stamped: Vec<(Stamp, String)> = json_lines(&dir.join(blackbox::BB_FILE), blackbox::BB_KEEP)
        .into_iter()
        .filter_map(|l| serde_json::from_str::<Stamp>(&l).ok().map(|s| (s, l)))
        .filter(|(s, _)| s.boot == target)
        .collect();
    let last = stamped.last().map(|(s, _)| s.t_ms).unwrap_or(0);
    let from = last.saturating_sub(seconds.saturating_mul(1000));
    let mut lines: Vec<String> = stamped.into_iter().filter(|(s, _)| s.t_ms >= from).map(|(_, l)| l).collect();
    let truncated = cap_newest(&mut lines);
    body(head(true, truncated), "records", &lines)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dir_resolution() {
        assert_eq!(resolve_dir(Some("/x/logs"), "ARMD_TEST_UNSET_VAR", DEFAULT_LOG_DIR), Some(PathBuf::from("/x/logs")));
        assert_eq!(resolve_dir(Some("off"), "ARMD_TEST_UNSET_VAR", DEFAULT_LOG_DIR), None);
        let none = resolve_dir(None, "ARMD_TEST_UNSET_VAR", DEFAULT_LOG_DIR);
        assert_eq!(none.is_some(), Path::new("/userdata").is_dir());
    }

    #[test]
    fn unwritable_dir_is_reported() {
        let dir = tempfile::tempdir().unwrap();
        let file = dir.path().join("file");
        std::fs::write(&file, "").unwrap();
        assert!(open_log(&file.join("logs")).is_err());
        assert!(open_log(&dir.path().join("logs")).is_ok());
    }

    #[test]
    fn blackbox_query_selects_boot_and_window() {
        let dir = tempfile::tempdir().unwrap();
        let bb = dir.path().join(blackbox::BB_FILE);
        let mut old = String::new();
        for t in (0..60_000).step_by(5_000) {
            old.push_str(&format!("{{\"boot\":4,\"t_ms\":{t}}}\n"));
        }
        std::fs::write(RotatingFile::numbered(&bb, 1), old).unwrap();
        std::fs::write(&bb, "{\"boot\":5,\"t_ms\":0}\n{\"boot\":5,\"t_ms\":200}\n{\"boot\":5,\"t_ms\":4").unwrap();
        let state = DiagnosticsState { blackbox_dir: Some(dir.path().to_path_buf()), boot: 5 };
        let cur: serde_json::Value = serde_json::from_str(&blackbox_body(&state, 60, false)).unwrap();
        assert_eq!(cur["records"].as_array().unwrap().len(), 2, "partial line skipped");
        let prev: serde_json::Value = serde_json::from_str(&blackbox_body(&state, 10, true)).unwrap();
        assert_eq!(prev["boot"], 4);
        let t: Vec<u64> = prev["records"].as_array().unwrap().iter().map(|r| r["t_ms"].as_u64().unwrap()).collect();
        assert_eq!(t, vec![45_000, 50_000, 55_000], "last 10 s of the previous start");
        let off: serde_json::Value = serde_json::from_str(&blackbox_body(&DiagnosticsState::default(), 10, false)).unwrap();
        assert_eq!(off["enabled"], false);
        assert!(off["records"].as_array().unwrap().is_empty());
    }

    #[test]
    fn responses_are_capped() {
        let mut lines: Vec<String> = (0..5000).map(|i| format!("{{\"i\":{i},\"pad\":\"{}\"}}", "x".repeat(1000))).collect();
        assert!(cap_newest(&mut lines));
        let total: usize = lines.iter().map(|l| l.len() + 1).sum();
        assert!(total <= MAX_RESPONSE_BYTES);
        assert!(lines.last().unwrap().starts_with("{\"i\":4999,"), "the newest lines are kept");
    }
}
