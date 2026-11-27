//! Central source-path indirection for integration tests that inspect first-party source text
//! or the cockpit HTML page.
//!
//! All logical module keys live in `.config/source_paths.tsv`. When a file moves or splits into a
//! directory (e.g. `src/grasp.rs` -> `src/grasp`, or `src/api/index.html` -> an assembled path),
//! updating that single manifest keeps every source-reading test working without per-test edits.
#![allow(dead_code)]

use std::fs;
use std::path::{Path, PathBuf};

const REGISTRY_REL: &str = ".config/source_paths.tsv";

/// Repository root (`CARGO_MANIFEST_DIR`, or current directory when invoked via `rustc --test`).
pub fn repo_root() -> PathBuf {
    if let Some(dir) = option_env!("CARGO_MANIFEST_DIR") {
        return PathBuf::from(dir);
    }
    if let Ok(dir) = std::env::var("CARGO_MANIFEST_DIR") {
        if !dir.is_empty() {
            return PathBuf::from(dir);
        }
    }
    std::env::current_dir().expect("resolve current working directory")
}

/// Parses `<root>/.config/source_paths.tsv` and returns all repo-relative POSIX entries for `key`.
pub fn configured_entries_in(root: &Path, key: &str) -> Vec<String> {
    let reg_path = root.join(REGISTRY_REL);
    let raw = fs::read_to_string(&reg_path)
        .unwrap_or_else(|e| panic!("failed to read {}: {e}", reg_path.display()));
    let mut entries = Vec::new();
    for (idx, line) in raw.lines().enumerate() {
        let line = line.strip_suffix('\r').unwrap_or(line);
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }
        let mut fields = line.split('\t');
        let Some(k) = fields.next() else {
            panic!("invalid row {}:{} in {}", REGISTRY_REL, idx + 1, reg_path.display());
        };
        let Some(v) = fields.next() else {
            panic!(
                "invalid row {}:{} in {}: expected `<key>\\t<path>`",
                REGISTRY_REL,
                idx + 1,
                reg_path.display()
            );
        };
        assert!(
            fields.next().is_none(),
            "invalid row {}:{} in {}: expected exactly 2 tab-separated fields",
            REGISTRY_REL,
            idx + 1,
            reg_path.display()
        );
        let k = k.trim();
        assert!(
            !k.is_empty(),
            "empty key on row {}:{} in {}",
            REGISTRY_REL,
            idx + 1,
            reg_path.display()
        );
        let rel = v.trim().replace('\\', "/");
        assert!(
            !rel.is_empty() && !rel.starts_with('/') && !rel.split('/').any(|s| s == ".."),
            "invalid repo-relative path `{rel}` for key `{k}` in {REGISTRY_REL}"
        );
        if k == key {
            entries.push(rel);
        }
    }
    assert!(
        !entries.is_empty(),
        "unknown source_paths key `{key}` in {}",
        reg_path.display()
    );
    entries
}

/// Returns all configured repo-relative POSIX entries for `key`.
pub fn configured_entries(key: &str) -> Vec<String> {
    configured_entries_in(&repo_root(), key)
}

/// Resolves the cockpit HTML path. Honors `ARMD_COCKPIT_HTML` when set and non-empty;
/// otherwise reads the `cockpit_html` key from `.config/source_paths.tsv`.
pub fn cockpit_html_path_in(root: &Path) -> PathBuf {
    if let Ok(override_path) = std::env::var("ARMD_COCKPIT_HTML") {
        if !override_path.is_empty() {
            return PathBuf::from(override_path);
        }
    }
    let entries = configured_entries_in(root, "cockpit_html");
    root.join(&entries[0])
}

/// Resolves the cockpit HTML path relative to the repository root.
pub fn cockpit_html_path() -> PathBuf {
    cockpit_html_path_in(&repo_root())
}

/// Reads the cockpit HTML page as UTF-8 text.
pub fn read_cockpit_html() -> String {
    let path = cockpit_html_path();
    fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("failed to read cockpit HTML at {}: {e}", path.display()))
}

fn collect_rs_recursive(dir: &Path, out: &mut Vec<PathBuf>) {
    let mut entries: Vec<PathBuf> = fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("failed to read directory {}: {e}", dir.display()))
        .map(|entry| entry.expect("read_dir entry").path())
        .collect();
    entries.sort();
    for path in entries {
        if path.is_dir() {
            collect_rs_recursive(&path, out);
        } else if path.extension().and_then(|e| e.to_str()) == Some("rs") {
            out.push(path);
        }
    }
}

/// Expands all configured entries for `key` into sorted absolute file paths.
/// Directory entries are expanded recursively to `.rs` files.
pub fn resolve_files_in(root: &Path, key: &str) -> Vec<PathBuf> {
    let mut out = Vec::new();
    for rel in configured_entries_in(root, key) {
        let full = root.join(&rel);
        if full.is_dir() {
            collect_rs_recursive(&full, &mut out);
        } else if full.is_file() {
            out.push(full);
        } else {
            panic!(
                "source_paths entry `{rel}` for key `{key}` does not exist at {}",
                full.display()
            );
        }
    }
    out.sort();
    out.dedup();
    assert!(
        !out.is_empty(),
        "source_paths key `{key}` resolved to 0 files under {}",
        root.display()
    );
    out
}

/// Expands all configured entries for `key` relative to the repository root.
pub fn resolve_files(key: &str) -> Vec<PathBuf> {
    resolve_files_in(&repo_root(), key)
}

/// Reads every file for `key` as `(repo_relative_posix_path, contents)`.
pub fn read_sources_in(root: &Path, key: &str) -> Vec<(String, String)> {
    resolve_files_in(root, key)
        .into_iter()
        .map(|path| {
            let rel = path
                .strip_prefix(root)
                .unwrap_or(&path)
                .to_string_lossy()
                .replace('\\', "/");
            let content = fs::read_to_string(&path)
                .unwrap_or_else(|e| panic!("failed to read {}: {e}", path.display()));
            (rel, content)
        })
        .collect()
}

/// Reads every file for `key` relative to the repository root.
pub fn read_sources(key: &str) -> Vec<(String, String)> {
    read_sources_in(&repo_root(), key)
}

/// Concatenates the contents of all files for `key` in deterministic path order.
pub fn read_concatenated(key: &str) -> String {
    let mut buf = String::new();
    for (_, text) in read_sources(key) {
        buf.push_str(&text);
        if !buf.ends_with('\n') {
            buf.push('\n');
        }
    }
    buf
}

/// Finds the first file under `key` whose text contains `needle`, returning `(rel_path, contents)`.
pub fn find_in_key(key: &str, needle: &str) -> Option<(String, String)> {
    read_sources(key)
        .into_iter()
        .find(|(_, content)| content.contains(needle))
}

/// Returns `true` if `rel_path` matches a file entry or lies inside a directory entry of `key`.
pub fn matches_key_in(root: &Path, key: &str, rel_path: &str) -> bool {
    let norm = rel_path.replace('\\', "/");
    let norm = norm.trim_start_matches("./");
    for entry in configured_entries_in(root, key) {
        let entry_trimmed = entry.trim_end_matches('/');
        if norm == entry_trimmed {
            return true;
        }
        if norm.starts_with(entry_trimmed)
            && norm.as_bytes().get(entry_trimmed.len()) == Some(&b'/')
        {
            return true;
        }
    }
    false
}

/// Returns `true` if `rel_path` matches `key` in the repository manifest.
pub fn matches_key(key: &str, rel_path: &str) -> bool {
    matches_key_in(&repo_root(), key, rel_path)
}

/// Strips trailing `#[cfg(test)]` module text using the standard source-audit boundary.
pub fn strip_cfg_test(src: &str) -> &str {
    src.split("#[cfg(test)]").next().unwrap_or(src)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    static ENV_LOCK: Mutex<()> = Mutex::new(());

    #[test]
    fn manifest_entries_all_resolve_in_repo() {
        let root = repo_root();
        let raw = fs::read_to_string(root.join(REGISTRY_REL)).expect("read manifest");
        let mut keys = Vec::new();
        for line in raw.lines() {
            let t = line.trim();
            if t.is_empty() || t.starts_with('#') {
                continue;
            }
            let (k, _) = t.split_once('\t').expect("tab-separated row");
            keys.push(k.trim().to_string());
        }
        keys.sort();
        keys.dedup();
        assert!(!keys.is_empty());
        for key in keys {
            if key == "cockpit_assets_dir" {
                let entries = configured_entries_in(&root, &key);
                assert!(root.join(&entries[0]).is_dir(), "missing dir for {key}");
                continue;
            }
            let files = resolve_files_in(&root, &key);
            assert!(!files.is_empty(), "key {key} resolved to empty");
        }
    }

    #[test]
    fn directory_expansion_and_matches_key_work_after_split() {
        let tmp = std::env::temp_dir().join(format!(
            "armd_source_paths_test_{}",
            std::process::id()
        ));
        let _ = fs::remove_dir_all(&tmp);
        fs::create_dir_all(tmp.join(".config")).unwrap();
        fs::create_dir_all(tmp.join("src/grasp/perception")).unwrap();
        fs::write(tmp.join("src/grasp/mod.rs"), "pub mod setup;\n").unwrap();
        fs::write(tmp.join("src/grasp/setup.rs"), "pub fn setup() {}\n").unwrap();
        fs::write(
            tmp.join("src/grasp/perception/frame.rs"),
            "pub struct Frame;\n",
        )
        .unwrap();
        fs::write(tmp.join("src/grasp/README.md"), "ignored non-rs\n").unwrap();
        fs::write(
            tmp.join(".config/source_paths.tsv"),
            "grasp\tsrc/grasp\ncockpit_html\tsrc/api/index.html\n",
        )
        .unwrap();

        let sources = read_sources_in(&tmp, "grasp");
        let rels: Vec<&str> = sources.iter().map(|(p, _)| p.as_str()).collect();
        assert_eq!(
            rels,
            vec![
                "src/grasp/mod.rs",
                "src/grasp/perception/frame.rs",
                "src/grasp/setup.rs",
            ]
        );
        assert!(matches_key_in(&tmp, "grasp", "src/grasp/setup.rs"));
        assert!(matches_key_in(&tmp, "grasp", "src/grasp/perception/frame.rs"));
        assert!(!matches_key_in(&tmp, "grasp", "src/grasp_other.rs"));
        assert!(!matches_key_in(&tmp, "grasp", "src/driver.rs"));

        let _ = fs::remove_dir_all(&tmp);
    }

    #[test]
    fn cockpit_html_env_override_takes_precedence() {
        let _guard = ENV_LOCK.lock().unwrap();
        let tmp = std::env::temp_dir().join(format!(
            "armd_cockpit_override_test_{}",
            std::process::id()
        ));
        let _ = fs::remove_dir_all(&tmp);
        fs::create_dir_all(tmp.join(".config")).unwrap();
        fs::write(
            tmp.join(".config/source_paths.tsv"),
            "cockpit_html\tsrc/api/assembled.html\r\n",
        )
        .unwrap();
        let prev = std::env::var_os("ARMD_COCKPIT_HTML");

        std::env::remove_var("ARMD_COCKPIT_HTML");
        assert_eq!(
            cockpit_html_path_in(&tmp),
            tmp.join("src/api/assembled.html")
        );

        std::env::set_var("ARMD_COCKPIT_HTML", "/tmp/custom_cockpit.html");
        assert_eq!(
            cockpit_html_path_in(&tmp),
            PathBuf::from("/tmp/custom_cockpit.html")
        );

        match prev {
            Some(v) => std::env::set_var("ARMD_COCKPIT_HTML", v),
            None => std::env::remove_var("ARMD_COCKPIT_HTML"),
        }
        let _ = fs::remove_dir_all(&tmp);
    }

    #[test]
    #[should_panic(expected = "unknown source_paths key")]
    fn unknown_key_panics_loudly() {
        let _ = configured_entries("nonexistent_key_xyz");
    }

    #[test]
    fn strip_cfg_test_splits_at_attribute() {
        assert_eq!(
            strip_cfg_test("fn prod() {}\n#[cfg(test)]\nmod tests {}"),
            "fn prod() {}\n"
        );
    }
}
