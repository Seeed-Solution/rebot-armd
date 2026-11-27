//! Cockpit page assembler, shared by `build.rs` and `tests/test_cockpit_assembly.rs` (`#[path]`).
//!
//! The cockpit is one inline HTML page (`src/api/index.html`, the shell) whose CSS and scripts
//! live in fragments under `src/api/cockpit/`. A splice point is a shell line that is exactly
//! `<!-- @include cockpit/<path> -->`; the assembler replaces that line (and its newline) with
//! the fragment's bytes. Rules (the Node assembler next to this file is a port with the same checks and error texts;
//! both run tests/fixtures/cockpit_assembly/cases.json):
//! - paths are relative to `src/api/`, start with `cockpit/`, use `[A-Za-z0-9_.-]` segments;
//! - a fragment is non-empty UTF-8, ends with `\n` and contains no splice line (one level);
//! - each fragment is included once, and every `.css` / `.js` / `.html` file under
//!   `cockpit/` (outside `cockpit/assembler/`) is included (no orphan fragments).
//!
//! Standard library only: `build.rs` has no build-dependencies.

use std::collections::BTreeSet;
use std::fs;
use std::path::{Path, PathBuf};

pub const SHELL: &str = "index.html";
pub const FRAGMENT_ROOT: &str = "cockpit";
const ASSEMBLER_DIR: &str = "cockpit/assembler";
const PREFIX: &str = "<!-- @include ";
const SUFFIX: &str = " -->";
const FRAGMENT_EXTS: [&str; 3] = ["css", "js", "html"];

/// The assembled page and the fragment paths (relative to the api dir) in splice order.
pub struct Assembled {
    pub page: String,
    pub fragments: Vec<String>,
}

/// Returns the fragment path of a splice line, `None` for any other line.
pub fn splice_target(line: &str) -> Option<&str> {
    line.strip_prefix(PREFIX)?.strip_suffix(SUFFIX)
}

fn check_rel(rel: &str) -> Result<(), String> {
    let ok_segment =
        |s: &str| !s.is_empty() && s != "." && s != ".." && s.bytes().all(|b| b.is_ascii_alphanumeric() || b"_.-".contains(&b));
    let segments: Vec<&str> = rel.split('/').collect();
    if segments.len() < 2 || segments[0] != FRAGMENT_ROOT || !segments[1..].iter().all(|s| ok_segment(s)) {
        return Err(format!("invalid fragment path `{rel}` (want cockpit/<segments>)"));
    }
    Ok(())
}

fn read_fragment(api_dir: &Path, rel: &str) -> Result<String, String> {
    check_rel(rel)?;
    let path = api_dir.join(rel);
    let text = fs::read_to_string(&path).map_err(|e| format!("cannot read fragment {}: {e}", path.display()))?;
    if text.is_empty() || !text.ends_with('\n') {
        return Err(format!("fragment {rel} must be non-empty and end with a newline"));
    }
    if let Some(line) = text.lines().find(|l| splice_target(l).is_some()) {
        return Err(format!("fragment {rel} contains a splice line `{line}` (no nested includes)"));
    }
    Ok(text)
}

fn collect_files(dir: &Path, api_dir: &Path, out: &mut BTreeSet<String>) -> Result<(), String> {
    let entries = fs::read_dir(dir).map_err(|e| format!("cannot list {}: {e}", dir.display()))?;
    for entry in entries {
        let path = entry.map_err(|e| e.to_string())?.path();
        let rel = path.strip_prefix(api_dir).map_err(|e| e.to_string())?.to_string_lossy().replace('\\', "/");
        if path.is_dir() {
            if rel != ASSEMBLER_DIR {
                collect_files(&path, api_dir, out)?;
            }
        } else if path.extension().and_then(|e| e.to_str()).is_some_and(|e| FRAGMENT_EXTS.contains(&e)) {
            out.insert(rel);
        }
    }
    Ok(())
}

/// Every fragment-like file under `cockpit/` (outside the assembler dir), sorted.
pub fn fragment_files(api_dir: &Path) -> Result<BTreeSet<String>, String> {
    let mut out = BTreeSet::new();
    let root = api_dir.join(FRAGMENT_ROOT);
    if root.is_dir() {
        collect_files(&root, api_dir, &mut out)?;
    }
    Ok(out)
}

/// Assembles `<api_dir>/index.html` from its fragments.
pub fn assemble(api_dir: &Path) -> Result<Assembled, String> {
    let shell_path: PathBuf = api_dir.join(SHELL);
    let shell = fs::read_to_string(&shell_path).map_err(|e| format!("cannot read {}: {e}", shell_path.display()))?;
    let mut page = String::with_capacity(shell.len() * 20);
    let mut fragments: Vec<String> = Vec::new();
    for line in shell.split_inclusive('\n') {
        let bare = line.strip_suffix('\n').unwrap_or(line);
        let Some(rel) = splice_target(bare) else {
            page.push_str(line);
            continue;
        };
        if !line.ends_with('\n') {
            return Err(format!("splice line for {rel} must end with a newline"));
        }
        if fragments.iter().any(|f| f == rel) {
            return Err(format!("fragment {rel} is included twice"));
        }
        page.push_str(&read_fragment(api_dir, rel)?);
        fragments.push(rel.to_string());
    }
    let included: BTreeSet<String> = fragments.iter().cloned().collect();
    let orphans: Vec<String> = fragment_files(api_dir)?.difference(&included).cloned().collect();
    if !orphans.is_empty() {
        return Err(format!("fragments not included by {SHELL}: {}", orphans.join(", ")));
    }
    Ok(Assembled { page, fragments })
}
