//! Shared lexical cleaner, `#[cfg(test)]`-aware production corpus builder, and item extractors
//! for the source-level invariant scanners in `test_rt_tx_budget` and
//! `test_torque_off_when_stopped`.
//!
//! Threat model: these scanners guard against accidental regressions in ordinary code, not
//! adversarial source; no new parser dependency.

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

/// Recursively collect all `.rs` files under `dir` in sorted order.
pub fn collect_rs_files(dir: &Path, out: &mut Vec<PathBuf>) {
    let mut entries: Vec<_> = std::fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("read_dir {}: {e}", dir.display()))
        .map(|e| e.expect("dir entry").path())
        .collect();
    entries.sort();
    for path in entries {
        if path.is_dir() {
            collect_rs_files(&path, out);
        } else if path.extension().and_then(|s| s.to_str()) == Some("rs") {
            out.push(path);
        }
    }
}

/// Blank Rust `//` line comments, nested `/* ... */` block comments, regular/byte/raw string
/// literal interiors, and char/byte literal interiors with ASCII spaces while preserving exact
/// byte offsets, quote delimiters, and `\n` newlines (`out.len() == src.len()`).
pub fn blank_rust_comments_and_literals(src: &str) -> String {
    let bytes = src.as_bytes();
    let len = bytes.len();
    let mut out = String::with_capacity(len);
    let mut i = 0usize;
    while i < len {
        // Line comment: // ... (preserve trailing \n)
        if bytes[i] == b'/' && bytes.get(i + 1) == Some(&b'/') {
            while i < len && bytes[i] != b'\n' {
                out.push(' ');
                i += 1;
            }
            continue;
        }
        // Nested block comment: /* ... */
        if bytes[i] == b'/' && bytes.get(i + 1) == Some(&b'*') {
            let mut depth = 1usize;
            out.push_str("  ");
            i += 2;
            while i < len && depth > 0 {
                if bytes[i] == b'/' && bytes.get(i + 1) == Some(&b'*') {
                    depth += 1;
                    out.push_str("  ");
                    i += 2;
                } else if bytes[i] == b'*' && bytes.get(i + 1) == Some(&b'/') {
                    depth -= 1;
                    out.push_str("  ");
                    i += 2;
                } else {
                    out.push(if bytes[i] == b'\n' { '\n' } else { ' ' });
                    i += 1;
                }
            }
            continue;
        }
        // Raw string literal: r"...", r#"..."#, br#"..."#, cr#"..."#
        let raw_prefix = if matches!(bytes[i], b'b' | b'c')
            && bytes.get(i + 1) == Some(&b'r')
            && !i
                .checked_sub(1)
                .is_some_and(|p| bytes[p].is_ascii_alphanumeric() || bytes[p] == b'_')
        {
            Some(i + 2)
        } else if bytes[i] == b'r'
            && !i
                .checked_sub(1)
                .is_some_and(|p| bytes[p].is_ascii_alphanumeric() || bytes[p] == b'_')
        {
            Some(i + 1)
        } else {
            None
        };
        if let Some(mut j) = raw_prefix {
            let mut hashes = 0usize;
            while j < len && bytes[j] == b'#' {
                hashes += 1;
                j += 1;
            }
            if j < len && bytes[j] == b'"' {
                out.push_str(&src[i..=j]);
                j += 1;
                while j < len {
                    if bytes[j] == b'"'
                        && j + hashes < len
                        && bytes[j + 1..j + 1 + hashes].iter().all(|&b| b == b'#')
                    {
                        out.push_str(&src[j..j + 1 + hashes]);
                        j += 1 + hashes;
                        break;
                    }
                    out.push(if bytes[j] == b'\n' { '\n' } else { ' ' });
                    j += 1;
                }
                i = j;
                continue;
            }
        }
        // Regular string literal: "..." (including b"..." / c"...")
        if bytes[i] == b'"' {
            out.push('"');
            i += 1;
            while i < len {
                if bytes[i] == b'\\' && i + 1 < len {
                    out.push(' ');
                    out.push(if bytes[i + 1] == b'\n' { '\n' } else { ' ' });
                    i += 2;
                    continue;
                }
                if bytes[i] == b'"' {
                    out.push('"');
                    i += 1;
                    break;
                }
                out.push(if bytes[i] == b'\n' { '\n' } else { ' ' });
                i += 1;
            }
            continue;
        }
        // Char / byte literal `'...'` (distinguished from lifetime `'a`)
        if bytes[i] == b'\'' {
            if bytes.get(i + 1) == Some(&b'\\') {
                out.push('\'');
                i += 1;
                while i < len {
                    if bytes[i] == b'\\' && i + 1 < len {
                        out.push(' ');
                        out.push(if bytes[i + 1] == b'\n' { '\n' } else { ' ' });
                        i += 2;
                        continue;
                    }
                    if bytes[i] == b'\'' {
                        out.push('\'');
                        i += 1;
                        break;
                    }
                    out.push(if bytes[i] == b'\n' { '\n' } else { ' ' });
                    i += 1;
                }
                continue;
            }
            let mut chars = src[i + 1..].char_indices();
            if let (Some((_, _ch)), Some((end_idx, '\''))) = (chars.next(), chars.next()) {
                out.push('\'');
                for _ in 0..end_idx {
                    out.push(' ');
                }
                out.push('\'');
                i = i + 1 + end_idx + 1;
                continue;
            }
        }
        let ch = src[i..].chars().next().unwrap();
        out.push(ch);
        i += ch.len_utf8();
    }
    debug_assert_eq!(out.len(), src.len());
    out
}

fn skip_ws(bytes: &[u8], mut i: usize) -> usize {
    while i < bytes.len() && bytes[i].is_ascii_whitespace() {
        i += 1;
    }
    i
}

fn is_ident_continue(b: u8) -> bool {
    b.is_ascii_alphanumeric() || b == b'_'
}

/// If `cleaned[i..]` starts with an outer attribute `#[ ... ]`, return `Some((inner_start, end_bracket))`.
/// Operates on lexically cleaned text so brackets in comments/strings/chars cannot interfere.
fn parse_outer_attr(cleaned: &str, i: usize) -> Option<(usize, usize)> {
    let bytes = cleaned.as_bytes();
    if bytes.get(i) != Some(&b'#') || bytes.get(i + 1) != Some(&b'[') {
        return None;
    }
    let mut depth = 0usize;
    let mut j = i + 1;
    while j < bytes.len() {
        match bytes[j] {
            b'[' => depth += 1,
            b']' => {
                depth -= 1;
                if depth == 0 {
                    return Some((i + 2, j));
                }
            }
            _ => {}
        }
        j += 1;
    }
    None
}

/// Exact `#[cfg(test)]` check (whitespace/newline-insensitive). Anything else such as
/// `#[cfg_attr(any(), cfg(test))]` or `#[cfg(any(test, ...))]` returns `false` (conservative).
fn is_exact_cfg_test(cleaned_inner: &str) -> bool {
    let compact: String = cleaned_inner.chars().filter(|c| !c.is_ascii_whitespace()).collect();
    compact == "cfg(test)"
}

/// Extract the string value from a `#[path = "..."]` attribute (whitespace/newline-insensitive),
/// reading the unblanked literal slice from `raw` via the exact preserved byte offsets.
fn extract_path_attr(raw: &str, cleaned: &str, inner_start: usize, inner_end: usize) -> Option<String> {
    let bytes = cleaned.as_bytes();
    let mut k = skip_ws(bytes, inner_start);
    if !cleaned[k..inner_end].starts_with("path") {
        return None;
    }
    k += 4;
    if k < inner_end && is_ident_continue(bytes[k]) {
        return None;
    }
    k = skip_ws(bytes, k);
    if k >= inner_end || bytes[k] != b'=' {
        return None;
    }
    k = skip_ws(bytes, k + 1);
    if k >= inner_end {
        return None;
    }
    let (val_start, val_end, after) = if bytes[k] == b'"' {
        let close = cleaned[k + 1..inner_end].find('"').map(|rel| k + 1 + rel)?;
        (k + 1, close, close + 1)
    } else if bytes[k] == b'r' {
        let mut h = 0usize;
        let mut j = k + 1;
        while j < inner_end && bytes[j] == b'#' {
            h += 1;
            j += 1;
        }
        if j >= inner_end || bytes[j] != b'"' {
            return None;
        }
        let open = j;
        j += 1;
        let mut close = None;
        while j < inner_end {
            if bytes[j] == b'"'
                && j + h < inner_end
                && bytes[j + 1..j + 1 + h].iter().all(|&b| b == b'#')
            {
                close = Some(j);
                break;
            }
            j += 1;
        }
        let close = close?;
        (open + 1, close, close + 1 + h)
    } else {
        return None;
    };
    if skip_ws(bytes, after) != inner_end {
        return None;
    }
    Some(raw[val_start..val_end].to_string())
}

/// Remove every item annotated with exact `#[cfg(test)]` from lexically cleaned source `cleaned`,
/// blanking stripped bytes to spaces while preserving `\n` so 1-based line numbers remain exact.
pub fn strip_cfg_test_items_cleaned(cleaned: &str) -> String {
    let bytes = cleaned.as_bytes();
    let len = bytes.len();
    let mut out = String::with_capacity(len);
    let mut i = 0usize;
    while i < len {
        if bytes[i] == b'#' && bytes.get(i + 1) == Some(&b'[') {
            let group_start = i;
            let mut j = i;
            let mut has_cfg_test = false;
            while let Some((inner_start, end_bracket)) = parse_outer_attr(cleaned, j) {
                if is_exact_cfg_test(&cleaned[inner_start..end_bracket]) {
                    has_cfg_test = true;
                }
                j = skip_ws(bytes, end_bracket + 1);
                if j >= len || bytes[j] != b'#' || bytes.get(j + 1) != Some(&b'[') {
                    break;
                }
            }
            if !has_cfg_test {
                out.push_str(&cleaned[group_start..j]);
                i = j;
                continue;
            }
            // Skip the entire annotated item (`...;` or `... { ... }` with optional trailing `;`).
            let mut pb_depth = 0usize;
            while j < len {
                match bytes[j] {
                    b'(' | b'[' => pb_depth += 1,
                    b')' | b']' => pb_depth = pb_depth.saturating_sub(1),
                    b';' if pb_depth == 0 => {
                        j += 1;
                        break;
                    }
                    b'{' if pb_depth == 0 => {
                        let mut bd = 0usize;
                        while j < len {
                            match bytes[j] {
                                b'{' => bd += 1,
                                b'}' => {
                                    bd -= 1;
                                    j += 1;
                                    if bd == 0 {
                                        let next = skip_ws(bytes, j);
                                        if next < len && bytes[next] == b';' {
                                            j = next + 1;
                                        }
                                        break;
                                    }
                                    continue;
                                }
                                _ => {}
                            }
                            j += 1;
                        }
                        break;
                    }
                    _ => {}
                }
                j += 1;
            }
            for &b in &bytes[group_start..j] {
                out.push(if b == b'\n' { '\n' } else { ' ' });
            }
            i = j;
            continue;
        }
        let ch = cleaned[i..].chars().next().unwrap();
        out.push(ch);
        i += ch.len_utf8();
    }
    debug_assert_eq!(out.len(), cleaned.len());
    out
}

fn normalize_rel(base: &str, rel: &str) -> String {
    let mut parts: Vec<&str> = if base.is_empty() { Vec::new() } else { base.split('/').collect() };
    for seg in rel.split('/') {
        match seg {
            "" | "." => {}
            ".." => {
                parts.pop();
            }
            s => parts.push(s),
        }
    }
    parts.join("/")
}

/// If `cleaned[start..]` starts an external module declaration `(pub(...)? )? mod <ident>;`
/// (whitespace/newline-insensitive), return `Some((ident, next_offset))`.
fn try_parse_external_mod(cleaned: &str, start: usize) -> Option<(String, usize)> {
    let bytes = cleaned.as_bytes();
    let len = bytes.len();
    let mut j = skip_ws(bytes, start);
    if cleaned[j..].starts_with("pub")
        && j + 3 < len
        && !is_ident_continue(bytes[j + 3])
    {
        j = skip_ws(bytes, j + 3);
        if j < len && bytes[j] == b'(' {
            let mut pd = 0usize;
            while j < len {
                match bytes[j] {
                    b'(' => pd += 1,
                    b')' => {
                        pd -= 1;
                        j += 1;
                        if pd == 0 {
                            break;
                        }
                        continue;
                    }
                    _ => {}
                }
                j += 1;
            }
            j = skip_ws(bytes, j);
        }
    }
    if !cleaned[j..].starts_with("mod") || j + 3 >= len || !bytes[j + 3].is_ascii_whitespace() {
        return None;
    }
    j = skip_ws(bytes, j + 3);
    if j + 2 <= len && &bytes[j..j + 2] == b"r#" {
        j += 2;
    }
    let id_start = j;
    while j < len && is_ident_continue(bytes[j]) {
        j += 1;
    }
    if j == id_start {
        return None;
    }
    let name = cleaned[id_start..j].to_string();
    j = skip_ws(bytes, j);
    if j < len && bytes[j] == b';' {
        Some((name, j + 1))
    } else {
        None
    }
}

/// Classify test-only files using `blank_rust_comments_and_literals` + exact `#[cfg(test)]`
/// attribute matching (whitespace/newline-insensitive). A file is test-only iff at least one
/// `mod` declaration (including split-line `#[path = "..."]`) resolves to it AND EVERY such
/// declaration carries exact `#[cfg(test)]` (or lives inside an exact `#[cfg(test)]` block or
/// test-only parent file). Anything else (`#[cfg_attr(...)]`, `#[cfg(any(test, ...))]`) counts
/// as PRODUCTION.
///
/// Returns `(production_corpus, test_only_files)` where each corpus entry is `(rel, prod_cleaned)`
/// with comments, string/char literals, and exact `#[cfg(test)]` items blanked while preserving
/// exact line numbers.
pub fn build_production_corpus(files: &[(String, String)]) -> (Vec<(String, String)>, BTreeSet<String>) {
    let cleaned_files: Vec<(&str, &str, String)> = files
        .iter()
        .map(|(rel, raw)| (rel.as_str(), raw.as_str(), blank_rust_comments_and_literals(raw)))
        .collect();
    let file_set: BTreeSet<&str> = files.iter().map(|(r, _)| r.as_str()).collect();
    let mut incoming: BTreeMap<String, Vec<(String, bool)>> = BTreeMap::new();

    for &(file_rel, raw, ref cleaned) in &cleaned_files {
        let bytes = cleaned.as_bytes();
        let len = bytes.len();
        let parent = file_rel.rsplit_once('/').map_or("", |(p, _)| p);
        let stem = file_rel
            .rsplit_once('/')
            .map_or(file_rel, |(_, f)| f)
            .strip_suffix(".rs")
            .unwrap_or("");
        let is_dir_mod = matches!(file_rel, "lib.rs" | "main.rs" | "src/lib.rs" | "src/main.rs")
            || file_rel.ends_with("/mod.rs")
            || file_rel.starts_with("bin/")
            || file_rel.starts_with("src/bin/");
        let sub_dir = if is_dir_mod {
            parent.to_string()
        } else if parent.is_empty() {
            stem.to_string()
        } else {
            format!("{parent}/{stem}")
        };

        let mut i = 0usize;
        let mut cfg_block_depth = 0usize;
        let mut attr_cfg_test = false;
        let mut attr_path: Option<String> = None;

        while i < len {
            i = skip_ws(bytes, i);
            if i >= len {
                break;
            }
            // Skip inner attributes `#![...]`
            if bytes[i] == b'#' && bytes.get(i + 1) == Some(&b'!') && bytes.get(i + 2) == Some(&b'[') {
                let mut d = 0usize;
                let mut j = i + 2;
                while j < len {
                    match bytes[j] {
                        b'[' => d += 1,
                        b']' => {
                            d -= 1;
                            j += 1;
                            if d == 0 {
                                break;
                            }
                            continue;
                        }
                        _ => {}
                    }
                    j += 1;
                }
                i = j;
                continue;
            }
            // Outer attribute `#[...]`
            if let Some((inner_start, end_bracket)) = parse_outer_attr(cleaned, i) {
                if is_exact_cfg_test(&cleaned[inner_start..end_bracket]) {
                    attr_cfg_test = true;
                }
                if let Some(p) = extract_path_attr(raw, cleaned, inner_start, end_bracket) {
                    attr_path = Some(p);
                }
                i = end_bracket + 1;
                continue;
            }
            // External `mod <ident>;` declaration
            if let Some((name, next_i)) = try_parse_external_mod(cleaned, i) {
                let gated = attr_cfg_test || cfg_block_depth > 0;
                let mut cands = if let Some(p) = &attr_path {
                    vec![normalize_rel(parent, p), normalize_rel(&sub_dir, p)]
                } else {
                    vec![
                        normalize_rel(&sub_dir, &format!("{name}.rs")),
                        normalize_rel(&sub_dir, &format!("{name}/mod.rs")),
                    ]
                };
                cands.sort();
                cands.dedup();
                for cand in cands {
                    if file_set.contains(cand.as_str()) {
                        incoming.entry(cand).or_default().push((file_rel.to_string(), gated));
                    }
                }
                attr_cfg_test = false;
                attr_path = None;
                i = next_i;
                continue;
            }
            // Non-external item annotated with exact `#[cfg(test)]`: enter its brace block if any.
            if attr_cfg_test && cfg_block_depth == 0 {
                let mut j = i;
                let mut pb_depth = 0usize;
                while j < len {
                    match bytes[j] {
                        b'(' | b'[' => pb_depth += 1,
                        b')' | b']' => pb_depth = pb_depth.saturating_sub(1),
                        b';' if pb_depth == 0 => {
                            j += 1;
                            break;
                        }
                        b'{' if pb_depth == 0 => {
                            cfg_block_depth = 1;
                            j += 1;
                            break;
                        }
                        _ => {}
                    }
                    j += 1;
                }
                attr_cfg_test = false;
                attr_path = None;
                i = j;
                continue;
            }
            attr_cfg_test = false;
            attr_path = None;

            if cfg_block_depth > 0 {
                if bytes[i] == b'{' {
                    cfg_block_depth += 1;
                    i += 1;
                    continue;
                }
                if bytes[i] == b'}' {
                    cfg_block_depth -= 1;
                    i += 1;
                    continue;
                }
            }
            if is_ident_continue(bytes[i]) {
                while i < len && is_ident_continue(bytes[i]) {
                    i += 1;
                }
            } else {
                let ch = cleaned[i..].chars().next().unwrap();
                i += ch.len_utf8();
            }
        }
    }

    let mut test_only = BTreeSet::new();
    let mut changed = true;
    while changed {
        changed = false;
        for (rel, _) in files {
            if !test_only.contains(rel)
                && incoming
                    .get(rel)
                    .is_some_and(|d| !d.is_empty() && d.iter().all(|(s, g)| *g || test_only.contains(s)))
            {
                test_only.insert(rel.clone());
                changed = true;
            }
        }
    }

    let corpus = cleaned_files
        .into_iter()
        .filter(|(rel, _, _)| !test_only.contains(*rel))
        .map(|(rel, _, cleaned)| (rel.to_string(), strip_cfg_test_items_cleaned(&cleaned)))
        .collect();
    (corpus, test_only)
}

/// Extract all `impl ... <type_name>` blocks from lexically cleaned, `#[cfg(test)]`-stripped `prod`.
/// Returns `(extracted_impls, remainder)`.
pub fn extract_impl_blocks_for_type(prod: &str, type_name: &str) -> (String, String) {
    let bytes = prod.as_bytes();
    let len = prod.len();
    let mut extracted = String::new();
    let mut remainder = String::with_capacity(len);
    let mut cursor = 0usize;
    let mut i = 0usize;
    while i + 4 <= len {
        let is_impl = &bytes[i..i + 4] == b"impl"
            && (i == 0 || !is_ident_continue(bytes[i - 1]))
            && (i + 4 < len && (bytes[i + 4].is_ascii_whitespace() || bytes[i + 4] == b'<'));
        if is_impl {
            let mut j = i + 4;
            let mut brace = None;
            while j < len {
                if bytes[j] == b';' {
                    break;
                }
                if bytes[j] == b'{' {
                    brace = Some(j);
                    break;
                }
                j += 1;
            }
            if let Some(bopen) = brace {
                let hdr = &prod[i..bopen];
                let hdr_no_where = hdr
                    .match_indices("where")
                    .find(|&(pos, _)| {
                        (pos == 0 || !is_ident_continue(hdr.as_bytes()[pos - 1]))
                            && hdr.as_bytes().get(pos + 5).is_some_and(|b| b.is_ascii_whitespace())
                    })
                    .map_or(hdr, |(pos, _)| &hdr[..pos]);
                let target = hdr_no_where.rsplit_once(" for ").map_or(hdr_no_where, |(_, rhs)| rhs);
                let targets_type = target.match_indices(type_name).any(|(pos, _)| {
                    (pos == 0 || !is_ident_continue(target.as_bytes()[pos - 1]))
                        && (pos + type_name.len() == target.len()
                            || !is_ident_continue(target.as_bytes()[pos + type_name.len()]))
                });
                if targets_type {
                    let mut depth = 0usize;
                    let mut bend = None;
                    for (rel, ch) in prod[bopen..].char_indices() {
                        if ch == '{' {
                            depth += 1;
                        } else if ch == '}' {
                            depth -= 1;
                            if depth == 0 {
                                bend = Some(bopen + rel);
                                break;
                            }
                        }
                    }
                    if let Some(end) = bend {
                        remainder.push_str(&prod[cursor..i]);
                        extracted.push_str(&prod[i..=end]);
                        extracted.push('\n');
                        cursor = end + 1;
                        i = cursor;
                        continue;
                    }
                }
            }
        }
        i += 1;
    }
    remainder.push_str(&prod[cursor..]);
    (extracted, remainder)
}

/// Expected `#[cfg(test)]`-only split files under `src/driver/**` and `src/supervisor/**`.
pub const SPLIT_TEST_ONLY_FILES: &[&str] = &[
    "driver/tx_chokepoint_tests.rs",
    "supervisor/observation_snapshot_tests.rs",
    "supervisor/driver_transitions/panic_disable_tests.rs",
    "supervisor/terminal_capture/tests.rs",
    "supervisor/streams/target_tests.rs",
];

/// Assert every expected split test-only module file (with optional `prefix`, e.g. `""` or `"src/"`)
/// is classified as test-only by [`build_production_corpus`].
pub fn assert_split_test_only_files(test_only: &BTreeSet<String>, prefix: &str) -> usize {
    for rel in SPLIT_TEST_ONLY_FILES {
        let full = format!("{prefix}{rel}");
        assert!(
            test_only.contains(&full),
            "{full} must be included solely via #[cfg(test)] mod declarations"
        );
    }
    SPLIT_TEST_ONLY_FILES.len()
}

/// Remove the single function definition matching `sig_prefix` (from `sig_prefix` through its
/// matching closing `}`) from lexically cleaned production text `prod`, panicking if `sig_prefix`
/// does not appear or appears more than once.
fn strip_unique_fn_by_signature(prod: &str, sig_prefix: &str, rel: &str) -> String {
    let mut matches = prod.match_indices(sig_prefix);
    let Some((start, _)) = matches.next() else {
        panic!("missing expected definition `{sig_prefix}` in {rel}");
    };
    assert!(
        matches.next().is_none(),
        "duplicate definition `{sig_prefix}` in {rel}"
    );
    let bopen = prod[start..]
        .find('{')
        .map(|rel_idx| start + rel_idx)
        .unwrap_or_else(|| panic!("missing `{{` for `{sig_prefix}` in {rel}"));
    let mut depth = 0usize;
    let mut bend = None;
    for (rel_idx, ch) in prod[bopen..].char_indices() {
        if ch == '{' {
            depth += 1;
        } else if ch == '}' {
            depth -= 1;
            if depth == 0 {
                bend = Some(bopen + rel_idx);
                break;
            }
        }
    }
    let end = bend.unwrap_or_else(|| panic!("unbalanced braces for `{sig_prefix}` in {rel}"));
    let mut out = String::with_capacity(prod.len() - (end + 1 - start));
    out.push_str(&prod[..start]);
    out.push_str(&prod[end + 1..]);
    out
}

/// Audit production supervisor motion-start modules using the shared lexical cleaner +
/// `#[cfg(test)]`-aware corpus: verify no production file re-samples `current_admission_stamp`
/// or `capture_admission_stamp` (narrowing the two definition sites to their exact signatures),
/// verify negative fixtures are rejected, and verify every `(rel, sig_prefix)` in
/// `required_signatures` requires `Admitted` by value.
/// Returns `(audited_file_count, checked_sig_count, test_only_checked)`.
pub fn audit_supervisor_admission_stamps(
    manifest_dir: &Path,
    required_signatures: &[(&str, &str)],
) -> (usize, usize, usize) {
    let src_dir = manifest_dir.join("src");
    let mut rs_paths = Vec::new();
    collect_rs_files(&src_dir, &mut rs_paths);
    let src_files: Vec<(String, String)> = rs_paths
        .iter()
        .map(|p| {
            let rel = p.strip_prefix(manifest_dir).unwrap().to_string_lossy().replace('\\', "/");
            (rel, std::fs::read_to_string(p).unwrap())
        })
        .collect();

    let run_audit = |files: &[(String, String)]| {
        let (corpus, test_only) = build_production_corpus(files);
        let test_only_n = assert_split_test_only_files(&test_only, "src/");
        let audited_exact = [
            "src/supervisor/motion.rs",
            "src/gravity_comp.rs",
            "src/poses.rs",
            "src/backlash.rs",
            "src/grasp.rs",
            "src/place.rs",
            "src/gripper_calib.rs",
            "src/gripper_baseline.rs",
        ];
        let audited_prefixes = [
            "src/supervisor/actions/",
            "src/supervisor/streams/",
            "src/supervisor/driver_transitions/",
        ];
        let mut audited_count = 0usize;
        for (rel, prod) in &corpus {
            let in_audited = audited_exact.contains(&rel.as_str())
                || audited_prefixes.iter().any(|pfx| rel.starts_with(pfx));
            if in_audited {
                audited_count += 1;
                assert!(
                    !prod.contains("current_admission_stamp") && !prod.contains("capture_admission_stamp"),
                    "{rel} must never re-sample an admission stamp in production code"
                );
            }
            if rel == "src/supervisor.rs" || rel.starts_with("src/supervisor/") {
                let remainder = match rel.as_str() {
                    "src/supervisor/state_init.rs" => strip_unique_fn_by_signature(
                        prod,
                        "pub fn current_admission_stamp(&self) -> AdmissionStamp",
                        rel,
                    ),
                    "src/supervisor/ownership.rs" => strip_unique_fn_by_signature(
                        prod,
                        "pub fn capture_admission_stamp(&self) -> AdmissionStamp",
                        rel,
                    ),
                    _ => prod.clone(),
                };
                assert!(
                    !remainder.contains("current_admission_stamp")
                        && !remainder.contains("capture_admission_stamp"),
                    "{rel} must never re-sample or define an admission stamp outside its single definition signature"
                );
            }
        }
        (corpus, audited_count, test_only_n)
    };

    let (corpus, audited_count, test_only_n) = run_audit(&src_files);
    // Negative fixture 1: re-sampling inside split `src/supervisor/actions/**`
    let mut bad_actions = src_files.clone();
    bad_actions.push((
        "src/supervisor/actions/injected_resample.rs".into(),
        "impl SupervisorInner { fn sneaky(&self) { let _ = self.current_admission_stamp(); } }".into(),
    ));
    assert!(
        std::panic::catch_unwind(|| { let _ = run_audit(&bad_actions); }).is_err(),
        "must reject current_admission_stamp re-sampling inside split src/supervisor/actions/**"
    );

    // Negative fixture 2: re-sampling inside `src/supervisor/state_init.rs` outside its single definition
    let mut bad_state_init = src_files.clone();
    if let Some((_, content)) = bad_state_init.iter_mut().find(|(r, _)| r == "src/supervisor/state_init.rs") {
        content.push_str("\nimpl SupervisorInner { fn sneaky_resample(&self) { let _ = self.current_admission_stamp(); } }\n");
    }
    assert!(
        std::panic::catch_unwind(|| { let _ = run_audit(&bad_state_init); }).is_err(),
        "must reject current_admission_stamp re-sampling inside src/supervisor/state_init.rs"
    );

    // Negative fixture 3: re-sampling inside `src/supervisor/ownership.rs` outside its single definition
    let mut bad_ownership = src_files;
    if let Some((_, content)) = bad_ownership.iter_mut().find(|(r, _)| r == "src/supervisor/ownership.rs") {
        content.push_str("\nimpl ArmSupervisor { fn sneaky_capture(&self) { let _ = self.capture_admission_stamp(); } }\n");
    }
    assert!(
        std::panic::catch_unwind(|| { let _ = run_audit(&bad_ownership); }).is_err(),
        "must reject capture_admission_stamp re-sampling inside src/supervisor/ownership.rs"
    );

    let corpus_map: BTreeMap<&str, &str> =
        corpus.iter().map(|(k, v)| (k.as_str(), v.as_str())).collect();
    for &(rel, sig_prefix) in required_signatures {
        let prod = corpus_map
            .get(rel)
            .unwrap_or_else(|| panic!("missing production corpus entry for {rel}"));
        let idx = prod.find(sig_prefix).unwrap_or_else(|| panic!("missing {sig_prefix} in {rel}"));
        let sig_slice = &prod[idx..prod[idx..].find('{').map_or(prod.len(), |o| idx + o)];
        assert!(
            sig_slice.contains("Admitted"),
            "{rel}::{sig_prefix} lacks Admitted parameter: {sig_slice}"
        );
    }
    (audited_count, required_signatures.len(), test_only_n)
}
