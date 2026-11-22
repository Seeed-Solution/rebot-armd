//! Sandbox REST API and simulation pack management for armd.
//!
//! Provides scene, tool, and policy management, runtime MJCF composition, and
//! state/telemetry bridging for MuJoCo WASM and rebot-sim.

use std::collections::{BTreeMap, HashMap, HashSet};
use std::path::{Path, PathBuf};
pub use crate::mjcf_compose::*;
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use axum::{
    body::{Body, Bytes},
    extract::{Extension, Path as UrlPath, Query, State},
    http::{header, Method, StatusCode},
    response::{IntoResponse, Response},
    Json,
};
use serde::{Deserialize, Serialize};
use tokio::sync::{OwnedSemaphorePermit, RwLock, Semaphore};

use super::assets;
use super::proxy::{self, DetectorUpstream};

static STAGING_COUNTER: AtomicU64 = AtomicU64::new(0);
static NEXT_BUNDLE_CACHE_ID: AtomicU64 = AtomicU64::new(1);
static TARGET_REPLACE_MUTEX: std::sync::Mutex<()> = std::sync::Mutex::new(());

pub const MAX_MANIFEST_BYTES: u64 = 1024 * 1024; // 1 MiB
pub const MAX_PACK_FILE_BYTES: u64 = 16 * 1024 * 1024; // 16 MiB
pub const MAX_POLICY_MODEL_BYTES: u64 = 64 * 1024 * 1024; // 64 MiB
pub const MAX_POLICY_PACK_BYTES: u64 = 96 * 1024 * 1024; // 96 MiB total pack size for policy uploads
/// Default HTTP body limit of the scene / tool / policy uploads. Base64 decoding holds the body,
/// the decoded bytes and the files at once (about 3x the body), so 16 MiB stays far below the
/// 256 MB of a reCamera. `ARMD_SANDBOX_MAX_BODY_MB` overrides it.
pub const DEFAULT_MAX_UPLOAD_BODY_BYTES: usize = 16 * 1024 * 1024;

/// The upload body limit: `ARMD_SANDBOX_MAX_BODY_MB` (MiB, 1..=1024) or
/// [`DEFAULT_MAX_UPLOAD_BODY_BYTES`].
pub fn max_upload_body_bytes() -> usize {
    upload_body_limit_from(std::env::var("ARMD_SANDBOX_MAX_BODY_MB").ok().as_deref())
}

fn upload_body_limit_from(value: Option<&str>) -> usize {
    let Some(value) = value.map(str::trim).filter(|v| !v.is_empty()) else { return DEFAULT_MAX_UPLOAD_BODY_BYTES; };
    match value.parse::<usize>() {
        Ok(mb @ 1..=1024) => mb * 1024 * 1024,
        _ => {
            tracing::warn!("ARMD_SANDBOX_MAX_BODY_MB={value:?} is not a whole number of MiB in 1..=1024; using {} MiB", DEFAULT_MAX_UPLOAD_BODY_BYTES >> 20);
            DEFAULT_MAX_UPLOAD_BODY_BYTES
        }
    }
}
pub const MAX_CUSTOM_PACKS_PER_KIND: usize = 64;
pub const MAX_SANDBOX_STORAGE_BYTES: u64 = 512 * 1024 * 1024; // 512 MiB

static UPLOAD_SEMAPHORE: std::sync::OnceLock<Arc<Semaphore>> = std::sync::OnceLock::new();

pub fn upload_semaphore() -> Arc<Semaphore> {
    UPLOAD_SEMAPHORE.get_or_init(|| Arc::new(Semaphore::new(1))).clone()
}

#[derive(Clone)]
pub struct UploadPermitSlot(pub Arc<std::sync::Mutex<Option<OwnedSemaphorePermit>>>);

/// Axum middleware ensuring only one sandbox upload runs at a time.
///
/// Gating happens BEFORE the request body (`Json(payload)`) is buffered by Axum,
/// preventing concurrent buffering of upload payloads when an upload is already active.
/// A `Content-Length` over [`max_upload_body_bytes`] is answered 413 here, before the permit
/// and before any of the body is read (`DefaultBodyLimit` only stops reading at the limit).
///
/// The acquired permit is attached to request extensions via [`UploadPermitSlot`].
/// Upload handlers transfer this permit into `spawn_blocking` so that if the client
/// disconnects prematurely, the permit remains held until the background blocking task
/// finishes writing to disk. If the request fails early (e.g. payload too large or invalid
/// JSON), the permit slot drops along with the request, immediately releasing the permit.
pub async fn upload_concurrency_middleware(
    mut req: axum::extract::Request,
    next: axum::middleware::Next,
) -> axum::response::Response {
    let limit = max_upload_body_bytes();
    let declared = req.headers().get(axum::http::header::CONTENT_LENGTH)
        .and_then(|v| v.to_str().ok()).and_then(|v| v.parse::<u64>().ok());
    if declared.is_some_and(|n| n > limit as u64) {
        return (
            StatusCode::PAYLOAD_TOO_LARGE,
            Json(serde_json::json!({
                "error": format!("Upload body is larger than the {} MiB limit (ARMD_SANDBOX_MAX_BODY_MB)", limit >> 20),
                "limit_bytes": limit,
            })),
        )
            .into_response();
    }
    let permit = match upload_semaphore().try_acquire_owned() {
        Ok(permit) => permit,
        Err(_) => {
            return (
                StatusCode::TOO_MANY_REQUESTS,
                Json(serde_json::json!({
                    "error": "Another sandbox upload is already in progress"
                })),
            )
                .into_response();
        }
    };
    req.extensions_mut()
        .insert(UploadPermitSlot(Arc::new(std::sync::Mutex::new(Some(permit)))));
    next.run(req).await
}

// ---------------------------------------------------------------------------
// Pure-Rust cryptographic & encoding helpers (zero extra crates)
// ---------------------------------------------------------------------------

pub use crate::gravity::Sha256;

/// Computes the SHA-256 hex digest of the given bytes according to FIPS 180-4.
pub fn sha256_hex(data: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(data);
    hasher.finalize_hex()
}

pub fn sha256_bytes(data: &[u8]) -> [u8; 32] {
    let mut hasher = Sha256::new();
    hasher.update(data);
    hasher.finalize()
}

/// Decodes standard or URL-safe Base64 strings, with optional `data:...;base64,` or `base64:` prefix.
pub fn base64_decode(input: &str) -> Result<Vec<u8>, String> {
    let clean = if let Some(idx) = input.find(";base64,") {
        &input[idx + 8..]
    } else if let Some(stripped) = input.strip_prefix("base64:") {
        stripped
    } else {
        input
    };

    let mut buf = Vec::with_capacity(clean.len() * 3 / 4);
    let mut quad = [0u8; 4];
    let mut q_len = 0;

    let decode_char = |c: u8| -> Result<u8, String> {
        match c {
            b'A'..=b'Z' => Ok(c - b'A'),
            b'a'..=b'z' => Ok(c - b'a' + 26),
            b'0'..=b'9' => Ok(c - b'0' + 52),
            b'+' | b'-' => Ok(62),
            b'/' | b'_' => Ok(63),
            b'=' => Ok(64), // padding indicator
            _ => Err(format!("invalid base64 byte: {}", c as char)),
        }
    };

    for &b in clean.as_bytes() {
        if b.is_ascii_whitespace() {
            continue;
        }
        let val = decode_char(b)?;
        quad[q_len] = val;
        q_len += 1;
        if q_len == 4 {
            let (b0, b1, b2, b3) = (quad[0], quad[1], quad[2], quad[3]);
            if b0 == 64 || b1 == 64 {
                return Err("invalid padding in base64 header".into());
            }
            let byte0 = (b0 << 2) | (b1 >> 4);
            buf.push(byte0);
            if b2 != 64 {
                let byte1 = ((b1 & 0x0f) << 4) | (b2 >> 2);
                buf.push(byte1);
                if b3 != 64 {
                    let byte2 = ((b2 & 0x03) << 6) | b3;
                    buf.push(byte2);
                }
            } else if b3 != 64 {
                return Err("invalid padding in base64 trailer".into());
            }
            q_len = 0;
        }
    }
    if q_len != 0 {
        return Err("truncated base64 sequence".into());
    }
    Ok(buf)
}


pub const SO_ARM101_BASE_XML: &str = include_str!("../../sim/mujoco/models/so_arm101_scene.xml");

struct BuiltinSceneSpec {
    id: &'static str,
    name: &'static str,
    description: &'static str,
    robots: &'static [&'static str],
    default_tool: &'static str,
    recommended_tools: &'static [&'static str],
    is_virtual: bool,
}

const BUILTIN_SCENE_SPECS: &[BuiltinSceneSpec] = &[
    BuiltinSceneSpec {
        id: "default",
        name: "Default Worktable Scene",
        description: "Standard worktable with interactive red cube.",
        robots: &["b601dm", "so-arm101"],
        default_tool: "default",
        recommended_tools: &["default"],
        is_virtual: true,
    },
    BuiltinSceneSpec {
        id: "coffee_kiosk",
        name: "Mobile Coffee Kiosk",
        description: "4-stage coffee workflow: detect/grasp cup, place under spout avoiding espresso column, press spring brew button, serve at pickup counter.",
        robots: &["b601dm"],
        default_tool: "default",
        recommended_tools: &["default", "wide_cup_gripper"],
        is_virtual: false,
    },
    BuiltinSceneSpec {
        id: "pcb_suction",
        name: "PCB Pick-and-Place & Test Cell",
        description: "3-stage PCB workflow with suction end-effector: vacuum-pick PCB from infeed tray over divider wall, place on test fixture, then transfer to outfeed bin.",
        robots: &["b601dm"],
        default_tool: "suction",
        recommended_tools: &["suction", "default"],
        is_virtual: false,
    },
    BuiltinSceneSpec {
        id: "pcb_inspection",
        name: "PCB AOI & Sorting Cell",
        description: "4-stage AOI continuity test fixture and Pass/NG sorting cell with dual bins and divider keepouts.",
        robots: &["b601dm"],
        default_tool: "suction",
        recommended_tools: &["suction", "screwdriver", "default"],
        is_virtual: false,
    },
    BuiltinSceneSpec {
        id: "screw_fastening",
        name: "Aluminum Assembly & Screw Fastening Cell",
        description: "2-point automated M3 screw fastening workflow with electric screwdriver: align bit with M3 screw targets on aluminum assembly carrier and fasten with torque feedback.",
        robots: &["b601dm"],
        default_tool: "screwdriver",
        recommended_tools: &["screwdriver", "default"],
        is_virtual: false,
    },
    BuiltinSceneSpec {
        id: "lab_desktop",
        name: "Laboratory Desktop Pick & Place",
        description: "SO-ARM101 table sorting cell: pick cube from feeder tray and place into sorting bin with obstacle avoidance.",
        robots: &["so-arm101"],
        default_tool: "default",
        recommended_tools: &["default"],
        is_virtual: false,
    },
];

fn builtin_scene_spec(id: &str) -> Option<&'static BuiltinSceneSpec> {
    BUILTIN_SCENE_SPECS.iter().find(|s| s.id == id)
}

/// Normalize a robot/profile identifier to its canonical sandbox robot ID
/// (`"b601dm"`, `"so-arm101"`, etc.).
pub fn normalize_robot_id(raw: &str) -> Option<String> {
    let trimmed = raw.trim();
    if trimmed.is_empty() || !trimmed.is_ascii() {
        return None;
    }
    let norm = trimmed.to_ascii_lowercase();
    let collapsed = norm.replace('_', "-");
    match collapsed.as_str() {
        "b601dm" | "b601-dm" | "rebot-dm" | "rebot-b601dm" | "rebot-b601-dm" => {
            Some("b601dm".to_string())
        }
        "so-arm101" | "soarm101" | "so101" | "so-101" => Some("so-arm101".to_string()),
        _ => Some(collapsed),
    }
}

pub fn is_reserved_builtin_id(kind: &str, id: &str) -> bool {
    if id == "default" {
        return true;
    }
    match kind {
        "scene" => BUILTIN_SCENE_SPECS.iter().any(|s| s.id == id),
        "tool" => BUILTIN_TOOL_SPECS.iter().any(|t| t.id == id),
        _ => false,
    }
}

struct BuiltinToolSpec {
    id: &'static str,
    name: &'static str,
    kind: &'static str,
    description: &'static str,
    robots: &'static [&'static str],
    is_virtual: bool,
}

const BUILTIN_TOOL_SPECS: &[BuiltinToolSpec] = &[
    BuiltinToolSpec {
        id: "default",
        name: "Stock Parallel Jaw Gripper",
        kind: "gripper",
        description: "Factory parallel gripper shipped with B601-DM or SO-ARM101.",
        robots: &["b601dm", "so-arm101"],
        is_virtual: true,
    },
    BuiltinToolSpec {
        id: "wide_cup_gripper",
        name: "Wide Silicone Cup Gripper",
        kind: "gripper",
        description: "Tall high-friction silicone parallel jaws tuned for 48 mm paper coffee cups.",
        robots: &["b601dm"],
        is_virtual: false,
    },
    BuiltinToolSpec {
        id: "suction",
        name: "Vacuum Suction End-Effector",
        kind: "suction",
        description: "Flat vacuum cup with runtime mjEQ_WELD suction controlled via /sim/io {vacuum: true|false}.",
        robots: &["b601dm"],
        is_virtual: false,
    },
    BuiltinToolSpec {
        id: "screwdriver",
        name: "Spindle Electric Screwdriver",
        kind: "screwdriver",
        description: "Rotating electric screwdriver spindle controlled via /sim/io {spindle_rpm: ...}.",
        robots: &["b601dm"],
        is_virtual: false,
    },
];

fn builtin_tool_spec(id: &str) -> Option<&'static BuiltinToolSpec> {
    BUILTIN_TOOL_SPECS.iter().find(|t| t.id == id)
}

#[derive(Debug)]
enum PackFileError {
    UnsafePath(String),
    SymlinkEscape(String),
    Missing(String),
    InvalidFormat(String),
    UnsupportedFormat(String),
    InvalidPack(String),
}

impl From<PackValidationError> for PackFileError {
    fn from(err: PackValidationError) -> Self {
        match err {
            PackValidationError::UnsupportedFormat(msg) => PackFileError::UnsupportedFormat(msg),
            PackValidationError::InvalidPack(msg) => PackFileError::InvalidPack(msg),
        }
    }
}

impl PackFileError {
    fn into_http_error(self, missing_code: &str, context: &str) -> (StatusCode, Json<serde_json::Value>) {
        match self {
            PackFileError::UnsafePath(msg) => (
                StatusCode::BAD_REQUEST,
                Json(serde_json::json!({
                    "error": format!("{context}: {msg}"),
                    "code": "INVALID_PATH",
                })),
            ),
            PackFileError::SymlinkEscape(msg) => (
                StatusCode::BAD_REQUEST,
                Json(serde_json::json!({
                    "error": format!("{context}: {msg}"),
                    "code": "PATH_ESCAPE",
                })),
            ),
            PackFileError::InvalidFormat(msg) => (
                StatusCode::BAD_REQUEST,
                Json(serde_json::json!({
                    "error": format!("{context}: {msg}"),
                    "code": "INVALID_MANIFEST",
                })),
            ),
            PackFileError::UnsupportedFormat(msg) => (
                StatusCode::BAD_REQUEST,
                Json(serde_json::json!({
                    "error": format!("{context}: {msg}"),
                    "code": "UNSUPPORTED_FORMAT",
                })),
            ),
            PackFileError::InvalidPack(msg) => (
                StatusCode::BAD_REQUEST,
                Json(serde_json::json!({
                    "error": format!("{context}: {msg}"),
                    "code": "INVALID_PACK",
                })),
            ),
            PackFileError::Missing(msg) => (
                StatusCode::NOT_FOUND,
                Json(serde_json::json!({
                    "error": format!("{context}: {msg}"),
                    "code": missing_code,
                })),
            ),
        }
    }
}

fn check_safe_pack_dir(allowed_root: &Path, pack_dir: &Path) -> Result<PathBuf, PackFileError> {
    let canon_allowed = allowed_root
        .canonicalize()
        .map_err(|_| PackFileError::Missing(format!("root '{}' not found", allowed_root.display())))?;
    if std::fs::symlink_metadata(pack_dir).is_err() {
        return Err(PackFileError::Missing(format!(
            "pack directory '{}' not found",
            pack_dir.display()
        )));
    }
    let canon_dir = pack_dir.canonicalize().map_err(|_| {
        PackFileError::SymlinkEscape(format!(
            "pack directory '{}' cannot be resolved safely",
            pack_dir.display()
        ))
    })?;
    if !canon_dir.starts_with(&canon_allowed) {
        return Err(PackFileError::SymlinkEscape(format!(
            "pack directory '{}' escapes allowed root '{}'",
            pack_dir.display(),
            allowed_root.display()
        )));
    }
    if !canon_dir.is_dir() {
        return Err(PackFileError::Missing(format!(
            "pack path '{}' is not a directory",
            pack_dir.display()
        )));
    }
    Ok(canon_dir)
}

fn check_safe_pack_file(canon_pack_dir: &Path, rel: &str) -> Result<PathBuf, PackFileError> {
    if !is_safe_relative_path(rel) {
        return Err(PackFileError::UnsafePath(format!(
            "unsafe relative path '{rel}'"
        )));
    }
    let candidate = canon_pack_dir.join(rel);
    if std::fs::symlink_metadata(&candidate).is_err() {
        return Err(PackFileError::Missing(format!("missing file '{rel}'")));
    }
    let canon_file = candidate.canonicalize().map_err(|_| {
        PackFileError::SymlinkEscape(format!("file '{rel}' resolves to an invalid or broken target"))
    })?;
    if !canon_file.starts_with(canon_pack_dir) {
        return Err(PackFileError::SymlinkEscape(format!(
            "file '{rel}' escapes pack directory via symlink"
        )));
    }
    if !canon_file.is_file() {
        return Err(PackFileError::Missing(format!("'{rel}' is not a regular file")));
    }
    Ok(canon_file)
}

fn read_safe_pack_file_with_limit(
    canon_pack_dir: &Path,
    rel: &str,
    max_bytes: u64,
) -> Result<String, PackFileError> {
    let path = check_safe_pack_file(canon_pack_dir, rel)?;
    let meta = std::fs::metadata(&path)
        .map_err(|e| PackFileError::Missing(format!("cannot stat '{rel}': {e}")))?;
    if meta.len() > max_bytes {
        return Err(PackFileError::InvalidPack(format!(
            "file '{rel}' size {} exceeds limit of {max_bytes} bytes",
            meta.len()
        )));
    }
    std::fs::read_to_string(&path)
        .map_err(|e| PackFileError::Missing(format!("cannot read '{rel}': {e}")))
}

fn read_safe_pack_file(canon_pack_dir: &Path, rel: &str) -> Result<String, PackFileError> {
    read_safe_pack_file_with_limit(canon_pack_dir, rel, MAX_MANIFEST_BYTES)
}

fn asset_entry_rel_path(entry: &serde_json::Value) -> Result<&str, PackFileError> {
    match entry {
        serde_json::Value::String(s) => {
            let trimmed = s.trim();
            if trimmed.is_empty() {
                Err(PackFileError::InvalidFormat("empty asset path string".into()))
            } else {
                Ok(trimmed)
            }
        }
        serde_json::Value::Object(map) => {
            for key in ["path", "target", "filename"] {
                if let Some(s) = map.get(key).and_then(|v| v.as_str()) {
                    let trimmed = s.trim();
                    if !trimmed.is_empty() {
                        return Ok(trimmed);
                    }
                }
            }
            Err(PackFileError::InvalidFormat(
                "asset entry object missing non-empty 'path', 'target', or 'filename'".into(),
            ))
        }
        _ => Err(PackFileError::InvalidFormat(
            "asset entry must be a relative path string or object".into(),
        )),
    }
}

fn is_optional_asset(entry: &serde_json::Value) -> bool {
    entry.get("required").and_then(|v| v.as_bool()) == Some(false)
}

fn verify_pack_assets_from_map(
    files: &BTreeMap<String, Vec<u8>>,
    assets: &[serde_json::Value],
) -> Result<(), PackFileError> {
    for entry in assets {
        let rel = asset_entry_rel_path(entry)?;
        if !is_safe_relative_path(rel) {
            return Err(PackFileError::UnsafePath(format!(
                "unsafe relative path '{rel}'"
            )));
        }
        if is_urdf_path(rel) {
            return Err(PackFileError::UnsupportedFormat(format!(
                "URDF assets are not supported: '{rel}'"
            )));
        }
        if !files.contains_key(rel) && !is_optional_asset(entry) {
            return Err(PackFileError::Missing(format!("missing file '{rel}'")));
        }
    }
    Ok(())
}

fn verify_pack_assets_in_map(
    files: &BTreeMap<String, Vec<u8>>,
    assets: &[serde_json::Value],
) -> Result<(), PackValidationError> {
    for entry in assets {
        let rel = asset_entry_rel_path(entry).map_err(|e| match e {
            PackFileError::InvalidFormat(m)
            | PackFileError::UnsafePath(m)
            | PackFileError::SymlinkEscape(m)
            | PackFileError::InvalidPack(m)
            | PackFileError::Missing(m) => PackValidationError::InvalidPack(m),
            PackFileError::UnsupportedFormat(m) => PackValidationError::UnsupportedFormat(m),
        })?;
        if !is_safe_relative_path(rel) {
            return Err(PackValidationError::InvalidPack(format!(
                "unsafe asset relative path '{rel}'"
            )));
        }
        if is_urdf_path(rel) {
            return Err(PackValidationError::UnsupportedFormat(format!(
                "URDF assets are not supported: '{rel}'"
            )));
        }
        if !files.contains_key(rel) && !is_optional_asset(entry) {
            return Err(PackValidationError::InvalidPack(format!(
                "missing declared required asset '{rel}'"
            )));
        }
    }
    Ok(())
}

fn collect_and_validate_pack_dir(
    canon_pack_dir: &Path,
    primary_xml_rel: &str,
    required_elem: Option<&str>,
) -> Result<(BTreeMap<String, Vec<u8>>, String), PackFileError> {
    if !is_safe_relative_path(primary_xml_rel) {
        return Err(PackFileError::UnsafePath(format!(
            "unsafe relative path '{primary_xml_rel}'"
        )));
    }
    if is_urdf_path(primary_xml_rel) {
        return Err(PackFileError::UnsupportedFormat(format!(
            "URDF files are not supported: '{primary_xml_rel}'"
        )));
    }
    // Verify the primary XML path directly first so dangling symlinks/escapes keep exact error type.
    let _ = check_safe_pack_file(canon_pack_dir, primary_xml_rel)?;

    fn walk_dir(
        root: &Path,
        cur: &Path,
        rel_prefix: &str,
        primary_xml_rel: &str,
        visited: &mut HashSet<PathBuf>,
        total_bytes: &mut usize,
        out: &mut BTreeMap<String, Vec<u8>>,
    ) -> Result<(), PackFileError> {
        let canon_cur = cur.canonicalize().map_err(|_| {
            PackFileError::SymlinkEscape(format!("directory '{}' cannot be resolved safely", cur.display()))
        })?;
        if !canon_cur.starts_with(root) {
            return Err(PackFileError::SymlinkEscape(format!(
                "directory '{}' escapes pack root",
                cur.display()
            )));
        }
        if !visited.insert(canon_cur) {
            return Ok(());
        }
        let entries = std::fs::read_dir(cur).map_err(|e| {
            PackFileError::Missing(format!("cannot read pack directory '{}': {e}", cur.display()))
        })?;
        for entry in entries.flatten() {
            let name = entry.file_name();
            let name_str = name.to_string_lossy();
            if name_str.starts_with('.') {
                continue;
            }
            if is_urdf_path(&name_str) {
                return Err(PackFileError::UnsupportedFormat(format!(
                    "URDF files are not supported in pack: '{name_str}'"
                )));
            }
            let rel = if rel_prefix.is_empty() {
                name_str.to_string()
            } else {
                format!("{rel_prefix}/{name_str}")
            };
            let path = entry.path();
            let sym_meta = std::fs::symlink_metadata(&path).map_err(|e| {
                PackFileError::Missing(format!("cannot stat '{rel}': {e}"))
            })?;
            let canon = path.canonicalize().map_err(|_| {
                PackFileError::SymlinkEscape(format!(
                    "file '{rel}' resolves to an invalid or broken target"
                ))
            })?;
            if !canon.starts_with(root) {
                return Err(PackFileError::SymlinkEscape(format!(
                    "file '{rel}' escapes pack directory via symlink"
                )));
            }
            if sym_meta.file_type().is_symlink() {
                // Consistent with Python os.walk(followlinks=False): do NOT follow directory symlinks.
                if canon.is_dir() {
                    continue;
                }
            }
            if canon.is_dir() {
                walk_dir(root, &path, &rel, primary_xml_rel, visited, total_bytes, out)?;
            } else if canon.is_file() {
                if !is_safe_relative_path(&rel) {
                    return Err(PackFileError::UnsafePath(format!(
                        "unsafe relative path '{rel}'"
                    )));
                }
                let meta = std::fs::metadata(&canon).map_err(|e| {
                    PackFileError::Missing(format!("cannot stat '{rel}': {e}"))
                })?;
                let raw_len = meta.len();
                let lower = rel.to_ascii_lowercase();
                if rel == primary_xml_rel || lower.ends_with(".xml") {
                    if raw_len > MAX_XML_BYTES as u64 {
                        return Err(PackFileError::InvalidPack(format!(
                            "XML '{rel}' size {raw_len} exceeds limit of {MAX_XML_BYTES} bytes"
                        )));
                    }
                } else if rel == "scene.json" || rel == "tool.json" {
                    if raw_len > MAX_MANIFEST_BYTES {
                        return Err(PackFileError::InvalidPack(format!(
                            "Manifest '{rel}' size {raw_len} exceeds limit of {MAX_MANIFEST_BYTES} bytes"
                        )));
                    }
                } else if (lower.ends_with(".stl") || lower.ends_with(".obj"))
                    && raw_len > MAX_MESH_FILE_BYTES as u64
                {
                    return Err(PackFileError::InvalidPack(format!(
                        "Mesh '{rel}' size {raw_len} exceeds limit of {MAX_MESH_FILE_BYTES} bytes"
                    )));
                }
                let file_len = usize::try_from(raw_len).unwrap_or(usize::MAX);
                *total_bytes = total_bytes.saturating_add(file_len);
                if *total_bytes > MAX_PACK_TOTAL_BYTES {
                    return Err(PackFileError::InvalidPack(format!(
                        "Total pack size exceeds {MAX_PACK_TOTAL_BYTES} byte limit"
                    )));
                }
                let bytes = std::fs::read(&canon).map_err(|e| {
                    PackFileError::Missing(format!("cannot read '{rel}': {e}"))
                })?;
                out.insert(rel, bytes);
            }
        }
        Ok(())
    }

    let mut files = BTreeMap::new();
    let mut visited = HashSet::new();
    let mut total_bytes = 0usize;
    walk_dir(
        canon_pack_dir,
        canon_pack_dir,
        "",
        primary_xml_rel,
        &mut visited,
        &mut total_bytes,
        &mut files,
    )?;

    if !files.contains_key(primary_xml_rel) {
        return Err(PackFileError::Missing(format!(
            "missing file '{primary_xml_rel}'"
        )));
    }

    let digest = validate_pack_map(primary_xml_rel, required_elem, &files)?;
    Ok((files, digest))
}

fn normalize_manifest_assets(
    files: Option<&BTreeMap<String, Vec<u8>>>,
    primary_xml_rel: Option<&str>,
    assets: Vec<serde_json::Value>,
) -> Vec<serde_json::Value> {
    let mut out = Vec::with_capacity(assets.len());
    let mut seen_paths = HashSet::new();

    for entry in assets {
        let Ok(rel) = asset_entry_rel_path(&entry).map(|s| s.to_string()) else {
            continue;
        };
        let file_meta = if let Some(file_map) = files {
            let Some(bytes) = file_map.get(&rel) else {
                // Skip absent optional assets (and anything missing from the validated pack map).
                continue;
            };
            Some((sha256_hex(bytes), bytes.len() as u64))
        } else {
            None
        };
        let mut map = match entry {
            serde_json::Value::Object(map) => map,
            serde_json::Value::String(_) => serde_json::Map::new(),
            _ => continue,
        };
        seen_paths.insert(rel.clone());
        map.insert("path".to_string(), serde_json::Value::String(rel));
        if let Some((digest, size_bytes)) = file_meta {
            map.insert("sha256".to_string(), serde_json::Value::String(digest));
            map.insert("size_bytes".to_string(), serde_json::json!(size_bytes));
        }
        out.push(serde_json::Value::Object(map));
    }

    if let Some(file_map) = files {
        for (xml_rel, xml_bytes) in file_map {
            let is_xml = primary_xml_rel == Some(xml_rel.as_str())
                || xml_rel.to_ascii_lowercase().ends_with(".xml");
            if !is_xml {
                continue;
            }
            let Ok(xml_str) = std::str::from_utf8(xml_bytes) else {
                continue;
            };
            let Ok(summary) = validate_xml_and_collect_summary_for_path(xml_rel, xml_str, None) else {
                continue;
            };
            for asset_ref in summary.refs {
                if !seen_paths.insert(asset_ref.path.clone()) {
                    continue;
                }
                if let Some(bytes) = file_map.get(&asset_ref.path) {
                    out.push(serde_json::json!({
                        "path": asset_ref.path,
                        "sha256": sha256_hex(bytes),
                        "size_bytes": bytes.len() as u64,
                    }));
                }
            }
        }
    }

    out
}

fn clamp_builtin_robots(manifest_robots: &[String], authoritative: &[&str]) -> Vec<String> {
    let allowed: Vec<String> = authoritative.iter().map(|r| (*r).to_string()).collect();
    if manifest_robots.is_empty() {
        return allowed;
    }
    let mut filtered = Vec::new();
    for r in manifest_robots {
        if let Some(norm) = normalize_robot_id(r) {
            if allowed.iter().any(|a| a == &norm) && !filtered.contains(&norm) {
                filtered.push(norm);
            }
        }
    }
    filtered
}

fn collect_pack_files(canon_pack_dir: &Path) -> Vec<String> {
    fn walk(
        base: &Path,
        cur: &Path,
        visited: &mut std::collections::HashSet<PathBuf>,
        out: &mut Vec<String>,
    ) {
        if !visited.insert(cur.to_path_buf()) {
            return;
        }
        if let Ok(entries) = std::fs::read_dir(cur) {
            for e in entries.flatten() {
                let path = e.path();
                let name = e.file_name();
                let name_str = name.to_string_lossy();
                if name_str.starts_with('.') {
                    continue;
                }
                if let Ok(sym_meta) = std::fs::symlink_metadata(&path) {
                    if let Ok(canon) = path.canonicalize() {
                        if !canon.starts_with(base) {
                            continue;
                        }
                        if sym_meta.file_type().is_symlink() && canon.is_dir() {
                            continue;
                        }
                        if canon.is_dir() {
                            walk(base, &canon, visited, out);
                        } else if canon.is_file() {
                            if let Ok(rel) = canon.strip_prefix(base) {
                                let rel_str = rel.to_string_lossy().replace('\\', "/");
                                if is_safe_relative_path(&rel_str) {
                                    out.push(rel_str);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    let mut files = Vec::new();
    let mut visited = std::collections::HashSet::new();
    walk(canon_pack_dir, canon_pack_dir, &mut visited, &mut files);
    files.sort();
    files.dedup();
    files
}

pub fn search_builtin_roots() -> Vec<PathBuf> {
    let mut roots = Vec::new();
    if let Ok(p) = std::env::var("ARMD_BUILTIN_SCENES_DIR") {
        if !p.trim().is_empty() {
            roots.push(PathBuf::from(p));
        }
    }
    roots.push(PathBuf::from("../rebot-companion/rebot_companion/scenes/builtin"));
    roots.push(PathBuf::from("rebot_companion/scenes/builtin"));
    roots.push(PathBuf::from("/usr/share/rebot-teleop/scenes/builtin"));
    roots
}

struct ValidBuiltinPack<M> {
    root: PathBuf,
    canon_dir: PathBuf,
    manifest: M,
    xml: String,
    digest: String,
    files: BTreeMap<String, Vec<u8>>,
}

struct ValidUserPack<M> {
    canon_dir: PathBuf,
    manifest: M,
    xml: String,
    digest: String,
    files: BTreeMap<String, Vec<u8>>,
}

struct ResolvedPack<M> {
    canon_dir: Option<PathBuf>,
    manifest: M,
    xml: String,
    digest: Option<String>,
    files: Option<BTreeMap<String, Vec<u8>>>,
}

fn find_valid_builtin_scene_dir(
    id: &str,
    roots: &[PathBuf],
) -> (bool, Option<ValidBuiltinPack<SceneManifest>>) {
    let spec = builtin_scene_spec(id);
    let mut saw_disk_manifest = false;

    for root in roots {
        let scene_dir = root.join("scenes").join(id);
        if scene_dir.join("scene.json").exists() {
            saw_disk_manifest = true;
        }
        let Ok(canon_dir) = check_safe_pack_dir(root, &scene_dir) else {
            continue;
        };
        let Ok(raw_json) = read_safe_pack_file(&canon_dir, "scene.json") else {
            continue;
        };
        let Ok(mut manifest) = serde_json::from_str::<SceneManifest>(&raw_json) else {
            continue;
        };
        let Ok((files, digest)) = collect_and_validate_pack_dir(&canon_dir, &manifest.world_xml, Some("worldbody")) else {
            continue;
        };
        let Some(world_xml) = files.get(&manifest.world_xml).and_then(|b| std::str::from_utf8(b).ok()).map(|s| s.to_string()) else {
            continue;
        };
        if verify_pack_assets_from_map(&files, &manifest.assets).is_err() {
            continue;
        }
        if let Some(s) = spec {
            manifest.id = s.id.to_string();
            manifest.robots = clamp_builtin_robots(&manifest.robots, s.robots);
        } else if manifest.id.is_empty() {
            manifest.id = id.to_string();
        }
        return (
            saw_disk_manifest,
            Some(ValidBuiltinPack {
                root: root.clone(),
                canon_dir,
                manifest,
                xml: world_xml,
                digest,
                files,
            }),
        );
    }
    (saw_disk_manifest, None)
}

pub fn load_builtin_scene_in_roots(id: &str, roots: &[PathBuf]) -> Option<(SceneManifest, String)> {
    if !validate_pack_id(id) {
        return None;
    }
    if id == "default" {
        let manifest = SceneManifest {
            id: "default".into(),
            name: "Default Worktable Scene".into(),
            version: "1.0.0".into(),
            description: "Standard worktable with interactive red cube.".into(),
            world_xml: "world.xml".into(),
            robots: vec!["b601dm".into(), "so-arm101".into()],
            default_tool: "default".into(),
            recommended_tools: vec!["default".into()],
            objects: vec![serde_json::json!({
                "name": "cube",
                "body": "cube",
                "geom": "cube",
                "class_name": "cube",
                "default_pos": [0.30, 0.0, 0.02],
                "jitter_xy": 0.02
            })],
            zones: serde_json::json!({}),
            io: vec![],
            task_stages: vec![],
            poses: serde_json::json!({}),
            keepout_boxes: vec![],
            training: serde_json::json!({}),
            assets: vec![],
        };
        return Some((manifest, "".into()));
    }

    // Check on-disk builtin scenes directory first so full scene manifests stay authoritative.
    let (saw_disk_manifest, found) = find_valid_builtin_scene_dir(id, roots);
    if let Some(pack) = found {
        return Some((pack.manifest, pack.xml));
    }

    if id == "pcb_suction" && !saw_disk_manifest {
        let manifest = SceneManifest {
            id: "pcb_suction".into(),
            name: "PCB Pick-and-Place & Test Cell".into(),
            version: "1.0.0".into(),
            description: "3-stage PCB workflow with suction end-effector: vacuum-pick PCB from infeed tray over divider wall, place on test fixture, then transfer to outfeed bin.".into(),
            world_xml: "world.xml".into(),
            robots: vec!["b601dm".into()],
            default_tool: "suction".into(),
            recommended_tools: vec!["suction".into(), "default".into()],
            objects: vec![serde_json::json!({
                "name": "pcb",
                "body": "pcb",
                "geom": "pcb",
                "class_name": "pcb",
                "default_pos": [0.23, -0.14, 0.011],
                "jitter_xy": 0.015
            })],
            zones: serde_json::json!({
                "infeed_tray": { "center": [0.23, -0.14, 0.018], "half_extents": [0.05, 0.05, 0.025] },
                "test_fixture": { "center": [0.27, 0.01, 0.024], "half_extents": [0.05, 0.045, 0.03] },
                "outfeed_bin": { "center": [0.23, 0.15, 0.022], "half_extents": [0.055, 0.055, 0.03] }
            }),
            io: vec![
                serde_json::json!({"name": "fixture_test_pass", "kind": "bool", "direction": "input", "default": false}),
                serde_json::json!({"name": "fixture_led", "kind": "bool", "direction": "output", "default": false}),
            ],
            task_stages: vec![
                serde_json::json!({"id": "pcb_picked", "type": "object_grasped", "object": "pcb", "min_z": 0.055, "latch": true}),
                serde_json::json!({
                    "id": "pcb_in_fixture",
                    "type": "object_in_zone",
                    "object": "pcb",
                    "zone": "test_fixture",
                    "requires": ["pcb_picked"],
                    "indicator_geom": "test_pass_led",
                    "indicator_rgba": [0.15, 0.9, 0.35, 1.0],
                    "latch": true
                }),
                serde_json::json!({
                    "id": "pcb_in_outfeed",
                    "type": "object_in_zone",
                    "object": "pcb",
                    "zone": "outfeed_bin",
                    "requires": ["pcb_in_fixture"],
                    "latch": true
                }),
            ],
            poses: serde_json::json!({}),
            keepout_boxes: vec![serde_json::json!({
                "name": "divider_wall",
                "min": [0.175, -0.070, 0.0],
                "max": [0.325, -0.060, 0.055]
            })],
            training: serde_json::json!({}),
            assets: vec![],
        };
        return Some((manifest, PRESET_PCB_SUCTION_XML.into()));
    }

    None
}

pub fn load_builtin_scene(id: &str) -> Option<(SceneManifest, String)> {
    load_builtin_scene_in_roots(id, &search_builtin_roots())
}

fn find_valid_builtin_tool_dir(
    id: &str,
    roots: &[PathBuf],
) -> (bool, Option<ValidBuiltinPack<ToolManifest>>) {
    let spec = builtin_tool_spec(id);
    let mut saw_disk_manifest = false;

    for root in roots {
        let tool_dir = root.join("tools").join(id);
        if tool_dir.join("tool.json").exists() {
            saw_disk_manifest = true;
        }
        let Ok(canon_dir) = check_safe_pack_dir(root, &tool_dir) else {
            continue;
        };
        let Ok(raw_json) = read_safe_pack_file(&canon_dir, "tool.json") else {
            continue;
        };
        let Ok(mut manifest) = serde_json::from_str::<ToolManifest>(&raw_json) else {
            continue;
        };
        let Ok((files, digest)) = collect_and_validate_pack_dir(&canon_dir, &manifest.tool_xml, None) else {
            continue;
        };
        let Some(tool_xml) = files.get(&manifest.tool_xml).and_then(|b| std::str::from_utf8(b).ok()).map(|s| s.to_string()) else {
            continue;
        };
        if verify_pack_assets_from_map(&files, &manifest.assets).is_err() {
            continue;
        }
        if let Some(s) = spec {
            manifest.id = s.id.to_string();
            manifest.robots = clamp_builtin_robots(&manifest.robots, s.robots);
        } else if manifest.id.is_empty() {
            manifest.id = id.to_string();
        }
        return (
            saw_disk_manifest,
            Some(ValidBuiltinPack {
                root: root.clone(),
                canon_dir,
                manifest,
                xml: tool_xml,
                digest,
                files,
            }),
        );
    }
    (saw_disk_manifest, None)
}

pub fn load_builtin_tool_in_roots(id: &str, roots: &[PathBuf]) -> Option<(ToolManifest, String)> {
    if !validate_pack_id(id) || !is_reserved_builtin_id("tool", id) {
        return None;
    }
    if id == "default" {
        return builtin_tool_preset("default").map(|(m, xml)| (m, xml.into()));
    }

    let (saw_disk_manifest, found) = find_valid_builtin_tool_dir(id, roots);
    if let Some(pack) = found {
        return Some((pack.manifest, pack.xml));
    }

    if !saw_disk_manifest {
        if let Some((manifest, xml)) = builtin_tool_preset(id) {
            return Some((manifest, xml.into()));
        }
    }

    None
}

pub fn load_builtin_tool(id: &str) -> Option<(ToolManifest, String)> {
    load_builtin_tool_in_roots(id, &search_builtin_roots())
}

struct ResolvedBaseRobotXml {
    xml: String,
    watch_paths: Vec<PathBuf>,
}

fn push_watch_path_with_canonical(
    paths: &mut Vec<PathBuf>,
    seen: &mut HashSet<PathBuf>,
    candidate: PathBuf,
) {
    if seen.insert(candidate.clone()) {
        paths.push(candidate.clone());
    }
    if let Ok(canon) = std::fs::canonicalize(&candidate) {
        if seen.insert(canon.clone()) {
            paths.push(canon);
        }
    }
}

fn resolve_base_robot_xml_from_candidates(
    env_var: &str,
    default_rel_path: &str,
    fallback: &str,
) -> ResolvedBaseRobotXml {
    let mut watch_paths = Vec::new();
    let mut seen = HashSet::new();

    if let Ok(path) = std::env::var(env_var) {
        let env_path = PathBuf::from(&path);
        push_watch_path_with_canonical(&mut watch_paths, &mut seen, env_path.clone());
        if let Ok(content) = std::fs::read_to_string(&env_path) {
            return ResolvedBaseRobotXml {
                xml: content,
                watch_paths,
            };
        }
    }

    let rel_path = PathBuf::from(default_rel_path);
    push_watch_path_with_canonical(&mut watch_paths, &mut seen, rel_path.clone());
    if let Ok(content) = std::fs::read_to_string(&rel_path) {
        return ResolvedBaseRobotXml {
            xml: content,
            watch_paths,
        };
    }

    ResolvedBaseRobotXml {
        xml: fallback.to_string(),
        watch_paths,
    }
}

fn resolve_base_robot_xml_b601dm() -> ResolvedBaseRobotXml {
    resolve_base_robot_xml_from_candidates(
        "ARMD_SANDBOX_BASE_XML",
        "./sim/mujoco/models/rebot_b601dm_scene.xml",
        FALLBACK_BASE_XML,
    )
}

fn resolve_base_robot_xml_for_robot(
    robot: &str,
) -> Result<ResolvedBaseRobotXml, (StatusCode, Json<serde_json::Value>)> {
    match normalize_robot_id(robot).as_deref() {
        Some("b601dm") => Ok(resolve_base_robot_xml_b601dm()),
        Some("so-arm101") => Ok(resolve_base_robot_xml_from_candidates(
            "ARMD_SANDBOX_SO_ARM101_BASE_XML",
            "./sim/mujoco/models/so_arm101_scene.xml",
            SO_ARM101_BASE_XML,
        )),
        _ => Err((
            StatusCode::BAD_REQUEST,
            Json(serde_json::json!({
                "error": format!("No base robot MJCF model for robot '{robot}'; refusing to fall back to B601"),
                "code": "ROBOT_INCOMPATIBLE",
                "robot": robot,
            })),
        )),
    }
}

pub fn get_base_robot_xml() -> String {
    resolve_base_robot_xml_b601dm().xml
}

/// Select the base MuJoCo robot XML for the specified canonical robot ID.
/// Refuses to silently fall back to B601 XML for any non-B601 robot.
pub fn get_base_robot_xml_for_robot(robot: &str) -> Result<String, (StatusCode, Json<serde_json::Value>)> {
    resolve_base_robot_xml_for_robot(robot).map(|r| r.xml)
}

// ---------------------------------------------------------------------------
// Sandbox State and Runtime Management
// ---------------------------------------------------------------------------

pub fn default_storage_dir() -> PathBuf {
    if let Ok(p) = std::env::var("ARMD_SANDBOX_DIR") {
        return PathBuf::from(p);
    }
    let userdata = Path::new("/userdata/rebot-teleop/sandbox");
    if Path::new("/userdata").is_dir() && std::fs::create_dir_all(userdata).is_ok() {
        return userdata.to_path_buf();
    }
    std::env::temp_dir().join("rebot-armd-sandbox")
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActiveState {
    pub mode: String, // "none" | "wasm" | "sim"
    pub active_scene: String,
    pub active_tool: String,
    pub active_policy: Option<String>,
    pub auto_start_policy: bool,
    pub seed: u64,
    pub reset_seq: u64,
    pub io_seq: u64,
    pub desired_io: serde_json::Value,
    #[serde(default)]
    pub telemetry: Option<serde_json::Value>,
    pub updated_at_ms: u64,
    #[serde(skip)]
    pub cached_mjcf: String,
    #[serde(skip)]
    pub committed_scene_digest: Option<String>,
    #[serde(skip)]
    pub committed_tool_digest: Option<String>,
}

impl Default for ActiveState {
    fn default() -> Self {
        Self {
            mode: "none".into(),
            active_scene: "default".into(),
            active_tool: "default".into(),
            active_policy: None,
            auto_start_policy: false,
            seed: 42,
            reset_seq: 0,
            io_seq: 0,
            desired_io: serde_json::json!({}),
            telemetry: None,
            updated_at_ms: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map(|d| d.as_millis() as u64)
                .unwrap_or(0),
            cached_mjcf: String::new(),
            committed_scene_digest: None,
            committed_tool_digest: None,
        }
    }
}

static TX_COUNTER: AtomicU64 = AtomicU64::new(1);

fn generate_tx_id() -> String {
    let seq = TX_COUNTER.fetch_add(1, Ordering::Relaxed);
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0);
    let mut hasher = Sha256::new();
    hasher.update(&std::process::id().to_le_bytes());
    hasher.update(&seq.to_le_bytes());
    hasher.update(&nanos.to_le_bytes());
    let hex = hasher.finalize_hex();
    format!("tx-{}", &hex[..24])
}

#[derive(Debug, Clone, Copy)]
pub struct SimTxTimeouts {
    pub prepare: Duration,
    pub commit: Duration,
    pub abort: Duration,
    pub reconcile: Duration,
}

impl Default for SimTxTimeouts {
    fn default() -> Self {
        Self {
            prepare: Duration::from_secs(15),
            commit: Duration::from_secs(10),
            abort: Duration::from_secs(5),
            reconcile: Duration::from_secs(5),
        }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(tag = "state", rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ActivationTxState {
    #[default]
    Idle,
    Preparing { tx_id: String },
    Prepared { tx_id: String },
    Committing { tx_id: String },
    Committed {
        tx_id: String,
        scene_digest: Option<String>,
        tool_digest: Option<String>,
    },
    Aborted { tx_id: String },
    DegradedSplit {
        tx_id: String,
        prev_tx_id: Option<String>,
        prev_scene: String,
        prev_tool: String,
        prev_policy: Option<String>,
        prev_mode: String,
        prev_seed: u64,
        prev_auto_start_policy: bool,
        prev_cached_mjcf: String,
        prev_scene_digest: Option<String>,
        prev_tool_digest: Option<String>,
        target_scene: String,
        target_tool: String,
        target_policy: Option<String>,
        target_seed: u64,
        target_auto_start_policy: bool,
        target_composed_mjcf: String,
        target_scene_digest: Option<String>,
        target_tool_digest: Option<String>,
        target_matches_tx: bool,
        detail: String,
    },
    Unknown {
        tx_id: String,
        prev_tx_id: Option<String>,
        prev_scene: String,
        prev_tool: String,
        prev_policy: Option<String>,
        prev_mode: String,
        prev_seed: u64,
        prev_auto_start_policy: bool,
        prev_cached_mjcf: String,
        prev_scene_digest: Option<String>,
        prev_tool_digest: Option<String>,
        target_scene: String,
        target_tool: String,
        target_policy: Option<String>,
        target_seed: u64,
        target_auto_start_policy: bool,
        target_composed_mjcf: String,
        target_scene_digest: Option<String>,
        target_tool_digest: Option<String>,
    },
}

impl ActivationTxState {
    pub fn label(&self) -> &'static str {
        match self {
            ActivationTxState::Idle => "IDLE",
            ActivationTxState::Preparing { .. } => "PREPARING",
            ActivationTxState::Prepared { .. } => "PREPARED",
            ActivationTxState::Committing { .. } => "COMMITTING",
            ActivationTxState::Committed { .. } => "COMMITTED",
            ActivationTxState::Aborted { .. } => "ABORTED",
            ActivationTxState::DegradedSplit { .. } => "DEGRADED_SPLIT",
            ActivationTxState::Unknown { .. } => "UNKNOWN",
        }
    }
}

const MAX_CACHED_BUNDLES: usize = 16;

#[derive(Debug, Clone, PartialEq, Eq)]
struct StatEntry {
    path: PathBuf,
    is_dir: bool,
    len: u64,
    mtime_nanos: i128,
    ctime_nanos: i128,
    ino: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct PackStatFingerprint {
    roots: Vec<PathBuf>,
    absent_watch_dirs: Vec<PathBuf>,
    extra_files: Vec<(PathBuf, Option<StatEntry>)>,
    entries: Vec<StatEntry>,
    dir_listing_count: usize,
}

#[cfg(unix)]
fn metadata_ino(meta: &std::fs::Metadata) -> u64 {
    use std::os::unix::fs::MetadataExt;
    meta.ino()
}

#[cfg(not(unix))]
fn metadata_ino(_meta: &std::fs::Metadata) -> u64 {
    0
}

#[cfg(unix)]
fn metadata_ctime_nanos(meta: &std::fs::Metadata) -> i128 {
    use std::os::unix::fs::MetadataExt;
    i128::from(meta.ctime())
        .saturating_mul(1_000_000_000)
        .saturating_add(i128::from(meta.ctime_nsec()))
}

#[cfg(not(unix))]
fn metadata_ctime_nanos(_meta: &std::fs::Metadata) -> i128 {
    0
}

fn metadata_mtime_nanos(meta: &std::fs::Metadata) -> i128 {
    meta.modified()
        .ok()
        .and_then(|t| match t.duration_since(UNIX_EPOCH) {
            Ok(d) => i128::try_from(d.as_nanos()).ok(),
            Err(e) => i128::try_from(e.duration().as_nanos()).ok().map(|v| -v),
        })
        .unwrap_or(0)
}

fn stat_entry_for_path(path: &Path) -> Option<StatEntry> {
    let meta = std::fs::symlink_metadata(path).ok()?;
    Some(StatEntry {
        path: path.to_path_buf(),
        is_dir: meta.is_dir(),
        len: meta.len(),
        mtime_nanos: metadata_mtime_nanos(&meta),
        ctime_nanos: metadata_ctime_nanos(&meta),
        ino: metadata_ino(&meta),
    })
}

impl PackStatFingerprint {
    fn collect_entries(roots: &[PathBuf]) -> Option<(Vec<StatEntry>, usize)> {
        fn walk(
            dir: &Path,
            visited: &mut HashSet<PathBuf>,
            entries: &mut Vec<StatEntry>,
            listing_count: &mut usize,
        ) -> Option<()> {
            let dir_meta = std::fs::symlink_metadata(dir).ok()?;
            if !dir_meta.is_dir() {
                return None;
            }
            let canon = dir.canonicalize().ok()?;
            if !visited.insert(canon) {
                return Some(());
            }
            entries.push(StatEntry {
                path: dir.to_path_buf(),
                is_dir: true,
                len: dir_meta.len(),
                mtime_nanos: metadata_mtime_nanos(&dir_meta),
                ctime_nanos: metadata_ctime_nanos(&dir_meta),
                ino: metadata_ino(&dir_meta),
            });
            let rd = std::fs::read_dir(dir).ok()?;
            for entry_res in rd {
                let entry = entry_res.ok()?;
                *listing_count = listing_count.saturating_add(1);
                let path = entry.path();
                let meta = std::fs::symlink_metadata(&path).ok()?;
                let is_dir = meta.is_dir();
                entries.push(StatEntry {
                    path: path.clone(),
                    is_dir,
                    len: meta.len(),
                    mtime_nanos: metadata_mtime_nanos(&meta),
                    ctime_nanos: metadata_ctime_nanos(&meta),
                    ino: metadata_ino(&meta),
                });
                if is_dir {
                    walk(&path, visited, entries, listing_count)?;
                }
            }
            Some(())
        }

        let mut visited = HashSet::new();
        let mut entries = Vec::new();
        let mut listing_count = 0usize;
        for root in roots {
            walk(root, &mut visited, &mut entries, &mut listing_count)?;
        }
        entries.sort_by(|a, b| a.path.cmp(&b.path));
        Some((entries, listing_count))
    }

    fn collect(
        roots: Vec<PathBuf>,
        absent_watch_dirs: Vec<PathBuf>,
        extra_watch_paths: Vec<PathBuf>,
    ) -> Self {
        let (entries, dir_listing_count) = Self::collect_entries(&roots).unwrap_or_default();
        let extra_files = extra_watch_paths
            .into_iter()
            .map(|p| {
                let st = stat_entry_for_path(&p);
                (p, st)
            })
            .collect();
        Self {
            roots,
            absent_watch_dirs,
            extra_files,
            entries,
            dir_listing_count,
        }
    }

    fn matches_disk(&self) -> bool {
        for p in &self.absent_watch_dirs {
            if std::fs::symlink_metadata(p).is_ok() {
                return false;
            }
        }
        for (p, expected) in &self.extra_files {
            if stat_entry_for_path(p) != *expected {
                return false;
            }
        }
        let Some((entries, dir_listing_count)) = Self::collect_entries(&self.roots) else {
            return false;
        };
        entries == self.entries && dir_listing_count == self.dir_listing_count
    }
}

struct InflightStatCheck {
    cache_id: u64,
    rx: tokio::sync::watch::Receiver<Option<bool>>,
}

#[derive(Debug, Clone)]
pub struct CachedBundle {
    cache_id: u64,
    pub robot: String,
    pub active_scene: String,
    pub active_tool: String,
    pub active_policy: Option<String>,
    pub scene_digest: Option<String>,
    pub tool_digest: Option<String>,
    pub composed_xml: String,
    pub composed_xml_sha256: String,
    pub scene_manifest: SceneManifest,
    pub tool_manifest: ToolManifest,
    pub policy_manifest: Option<PolicyManifest>,
    pub policy_model_url: Option<String>,
    pub base_meshdir: Option<String>,
    pub base_assets: Vec<serde_json::Value>,
    pub missing_base_assets: Vec<String>,
    pub base_assets_note: Option<String>,
    stat_fingerprint: PackStatFingerprint,
}

impl CachedBundle {
    fn matches_key(&self, robot: &str, active: &ActiveState) -> bool {
        self.robot == robot
            && self.active_scene == active.active_scene
            && self.active_tool == active.active_tool
            && self.active_policy == active.active_policy
            && match &active.committed_scene_digest {
                Some(d) => self.scene_digest.as_deref() == Some(d.as_str()),
                None => true,
            }
            && match &active.committed_tool_digest {
                Some(d) => self.tool_digest.as_deref() == Some(d.as_str()),
                None => true,
            }
    }
}

pub struct SandboxState {
    pub root: PathBuf,
    pub active: RwLock<ActiveState>,
    pub tx_state: RwLock<ActivationTxState>,
    pub activate_lock: tokio::sync::Mutex<()>,
    pub sim_timeouts: SimTxTimeouts,
    bundle_cache: RwLock<Vec<Arc<CachedBundle>>>,
    bundle_rebuild_sem: Arc<Semaphore>,
    bundle_rebuild_count: Arc<AtomicU64>,
    stat_check_inflight: Arc<std::sync::Mutex<Option<InflightStatCheck>>>,
    bundle_stat_walk_count: Arc<AtomicU64>,
    stat_walk_delay_ms: AtomicU64,
    in_flight_rebuilds: Arc<AtomicUsize>,
    peak_concurrent_rebuilds: Arc<AtomicUsize>,
    bundle_rebuild_delay_ms: AtomicU64,
    builtin_roots: Option<Vec<PathBuf>>,
    #[cfg(feature = "policy-runtime")]
    policy_load_delay_ms: AtomicU64,
    #[cfg(feature = "policy-runtime")]
    policy_infer_delay_ms: AtomicU64,
}

impl SandboxState {
    pub fn new(root: PathBuf) -> Self {
        let _ = std::fs::create_dir_all(root.join("scenes"));
        let _ = std::fs::create_dir_all(root.join("tools"));
        let _ = std::fs::create_dir_all(root.join("policies"));

        let state_path = root.join("state.json");
        let active = if let Ok(bytes) = std::fs::read(&state_path) {
            serde_json::from_slice(&bytes).unwrap_or_default()
        } else {
            ActiveState::default()
        };

        Self {
            root,
            active: RwLock::new(active),
            tx_state: RwLock::new(ActivationTxState::Idle),
            activate_lock: tokio::sync::Mutex::new(()),
            sim_timeouts: SimTxTimeouts::default(),
            bundle_cache: RwLock::new(Vec::new()),
            bundle_rebuild_sem: Arc::new(Semaphore::new(1)),
            bundle_rebuild_count: Arc::new(AtomicU64::new(0)),
            stat_check_inflight: Arc::new(std::sync::Mutex::new(None)),
            bundle_stat_walk_count: Arc::new(AtomicU64::new(0)),
            stat_walk_delay_ms: AtomicU64::new(0),
            in_flight_rebuilds: Arc::new(AtomicUsize::new(0)),
            peak_concurrent_rebuilds: Arc::new(AtomicUsize::new(0)),
            bundle_rebuild_delay_ms: AtomicU64::new(0),
            builtin_roots: None,
            #[cfg(feature = "policy-runtime")]
            policy_load_delay_ms: AtomicU64::new(0),
            #[cfg(feature = "policy-runtime")]
            policy_infer_delay_ms: AtomicU64::new(0),
        }
    }

    /// Number of full disk reread/validate/hash/compose bundle rebuilds performed by `get_bundle`.
    pub fn bundle_rebuild_count(&self) -> u64 {
        self.bundle_rebuild_count.load(Ordering::Relaxed)
    }

    /// Number of `PackStatFingerprint::matches_disk` walks executed on warm cache hits.
    pub fn bundle_stat_walk_count(&self) -> u64 {
        self.bundle_stat_walk_count.load(Ordering::Relaxed)
    }

    /// Test hook: artificial delay inside each warm bundle stat walk.
    pub fn set_stat_walk_delay_ms(&self, delay_ms: u64) {
        self.stat_walk_delay_ms.store(delay_ms, Ordering::Relaxed);
    }

    /// Peak number of concurrent `build_bundle_from_disk` executions observed on this state.
    pub fn peak_concurrent_rebuilds(&self) -> usize {
        self.peak_concurrent_rebuilds.load(Ordering::SeqCst)
    }

    /// Test hook: artificial delay inside `build_bundle_from_disk`.
    pub fn set_bundle_rebuild_delay_ms(&self, delay_ms: u64) {
        self.bundle_rebuild_delay_ms.store(delay_ms, Ordering::Relaxed);
    }

    /// Single-flight coalescing for warm `PackStatFingerprint::matches_disk` checks keyed by `cache_id`.
    async fn check_bundle_fresh(self: &Arc<Self>, cached: &CachedBundle) -> bool {
        let cache_id = cached.cache_id;
        loop {
            enum Action {
                Wait(tokio::sync::watch::Receiver<Option<bool>>),
                WaitThenRetry(tokio::sync::watch::Receiver<Option<bool>>),
                Run(tokio::sync::watch::Sender<Option<bool>>),
            }

            let action = {
                let mut guard = self
                    .stat_check_inflight
                    .lock()
                    .unwrap_or_else(|e| e.into_inner());
                match guard.as_ref() {
                    Some(inflight) if inflight.cache_id == cache_id => {
                        Action::Wait(inflight.rx.clone())
                    }
                    Some(inflight) => Action::WaitThenRetry(inflight.rx.clone()),
                    None => {
                        let (tx, rx) = tokio::sync::watch::channel(None);
                        *guard = Some(InflightStatCheck { cache_id, rx });
                        Action::Run(tx)
                    }
                }
            };

            match action {
                Action::Wait(mut rx) => {
                    loop {
                        if let Some(res) = *rx.borrow_and_update() {
                            return res;
                        }
                        if rx.changed().await.is_err() {
                            return false;
                        }
                    }
                }
                Action::WaitThenRetry(mut rx) => {
                    loop {
                        if rx.borrow_and_update().is_some() {
                            break;
                        }
                        if rx.changed().await.is_err() {
                            break;
                        }
                    }
                }
                Action::Run(tx) => {
                    let state_c = Arc::clone(self);
                    let fp = cached.stat_fingerprint.clone();
                    return tokio::task::spawn_blocking(move || {
                        struct StatSlotGuard {
                            slot: Arc<std::sync::Mutex<Option<InflightStatCheck>>>,
                            tx: Option<tokio::sync::watch::Sender<Option<bool>>>,
                            result: bool,
                        }
                        impl Drop for StatSlotGuard {
                            fn drop(&mut self) {
                                {
                                    let mut guard =
                                        self.slot.lock().unwrap_or_else(|e| e.into_inner());
                                    *guard = None;
                                }
                                if let Some(tx) = self.tx.take() {
                                    let _ = tx.send(Some(self.result));
                                }
                            }
                        }
                        let mut guard = StatSlotGuard {
                            slot: Arc::clone(&state_c.stat_check_inflight),
                            tx: Some(tx),
                            result: false,
                        };
                        state_c.bundle_stat_walk_count.fetch_add(1, Ordering::Relaxed);
                        let delay_ms = state_c.stat_walk_delay_ms.load(Ordering::Relaxed);
                        if delay_ms > 0 {
                            std::thread::sleep(Duration::from_millis(delay_ms));
                        }
                        let fresh = fp.matches_disk();
                        guard.result = fresh;
                        fresh
                    })
                    .await
                    .unwrap_or(false);
                }
            }
        }
    }

    async fn lookup_cached_bundle(&self, robot: &str, active: &ActiveState) -> Option<Arc<CachedBundle>> {
        let cache = self.bundle_cache.read().await;
        cache
            .iter()
            .rev()
            .find(|b| b.matches_key(robot, active))
            .cloned()
    }

    async fn insert_cached_bundle(&self, bundle: Arc<CachedBundle>) {
        let mut cache = self.bundle_cache.write().await;
        Self::insert_cached_bundle_into(&mut cache, bundle);
    }

    fn insert_cached_bundle_blocking(&self, bundle: Arc<CachedBundle>) {
        let mut cache = self.bundle_cache.blocking_write();
        Self::insert_cached_bundle_into(&mut cache, bundle);
    }

    fn insert_cached_bundle_into(cache: &mut Vec<Arc<CachedBundle>>, bundle: Arc<CachedBundle>) {
        cache.retain(|b| {
            !(b.robot == bundle.robot
                && b.active_scene == bundle.active_scene
                && b.active_tool == bundle.active_tool
                && b.active_policy == bundle.active_policy
                && b.scene_digest == bundle.scene_digest
                && b.tool_digest == bundle.tool_digest)
        });
        while cache.len() >= MAX_CACHED_BUNDLES {
            cache.remove(0);
        }
        cache.push(bundle);
    }

    async fn invalidate_bundle_cache_all(&self) {
        self.bundle_cache.write().await.clear();
    }

    async fn invalidate_bundle_cache_for_scene(&self, scene_id: &str) {
        self.bundle_cache
            .write()
            .await
            .retain(|b| b.active_scene != scene_id);
    }

    async fn invalidate_bundle_cache_for_tool(&self, tool_id: &str) {
        self.bundle_cache
            .write()
            .await
            .retain(|b| b.active_tool != tool_id);
    }

    async fn invalidate_bundle_cache_for_policy(&self, policy_id: &str) {
        self.bundle_cache
            .write()
            .await
            .retain(|b| b.active_policy.as_deref() != Some(policy_id));
    }

    /// Override builtin scene/tool search roots (useful in tests without mutating env vars).
    pub fn with_builtin_roots(mut self, roots: Vec<PathBuf>) -> Self {
        self.builtin_roots = Some(roots);
        self
    }

    /// Override simulation transaction timeouts (useful in unit/integration tests).
    pub fn with_sim_timeouts(mut self, timeouts: SimTxTimeouts) -> Self {
        self.sim_timeouts = timeouts;
        self
    }

    fn effective_builtin_roots(&self) -> Vec<PathBuf> {
        self.builtin_roots.clone().unwrap_or_else(search_builtin_roots)
    }

    /// Returns the user scene directory path if a directory on disk collides with a reserved builtin scene ID.
    pub fn user_scene_conflict(&self, id: &str) -> Option<PathBuf> {
        if !is_reserved_builtin_id("scene", id) {
            return None;
        }
        let dir = self.root.join("scenes").join(id);
        std::fs::symlink_metadata(&dir).is_ok().then_some(dir)
    }

    /// Returns the user tool directory path if a directory on disk collides with a reserved builtin tool ID.
    pub fn user_tool_conflict(&self, id: &str) -> Option<PathBuf> {
        if !is_reserved_builtin_id("tool", id) {
            return None;
        }
        let dir = self.root.join("tools").join(id);
        std::fs::symlink_metadata(&dir).is_ok().then_some(dir)
    }

    #[cfg(feature = "policy-runtime")]
    pub fn set_policy_load_delay_ms(&self, ms: u64) {
        self.policy_load_delay_ms.store(ms, Ordering::Relaxed);
    }

    #[cfg(feature = "policy-runtime")]
    pub fn policy_load_delay_ms(&self) -> u64 {
        self.policy_load_delay_ms.load(Ordering::Relaxed)
    }

    /// Test hook: policies loaded for a run sleep this long before every inference
    /// (warm-up included). 0 = off.
    #[cfg(feature = "policy-runtime")]
    pub fn set_policy_infer_delay_ms(&self, ms: u64) {
        self.policy_infer_delay_ms.store(ms, Ordering::Relaxed);
    }

    #[cfg(feature = "policy-runtime")]
    pub fn policy_infer_delay_ms(&self) -> u64 {
        self.policy_infer_delay_ms.load(Ordering::Relaxed)
    }

    /// Return the active `(scene_id, tool_id)`, defaulting empty strings to `"default"`.
    pub async fn active_scene_and_tool(&self) -> (String, String) {
        let st = self.active.read().await;
        let scene = st.active_scene.trim();
        let tool = st.active_tool.trim();
        (
            if scene.is_empty() {
                "default".to_string()
            } else {
                scene.to_string()
            },
            if tool.is_empty() {
                "default".to_string()
            } else {
                tool.to_string()
            },
        )
    }

    pub fn new_shared() -> Arc<Self> {
        Arc::new(Self::new(default_storage_dir()))
    }

    pub async fn persist(&self) {
        let st = self.active.read().await;
        let path = self.root.join("state.json");
        if let Ok(data) = serde_json::to_vec_pretty(&*st) {
            let _ = tokio::fs::write(path, data).await;
        }
    }

    fn load_user_scene_checked(
        &self,
        id: &str,
        check_assets: bool,
    ) -> Result<ValidUserPack<SceneManifest>, PackFileError> {
        let scenes_root = self.root.join("scenes");
        let dir = scenes_root.join(id);
        let canon_dir = check_safe_pack_dir(&scenes_root, &dir)?;
        let manifest_str = read_safe_pack_file(&canon_dir, "scene.json")?;
        let mut manifest: SceneManifest = serde_json::from_str(&manifest_str)
            .map_err(|e| PackFileError::InvalidFormat(format!("invalid scene.json: {e}")))?;
        manifest.id = id.to_string();
        let (files, digest) =
            collect_and_validate_pack_dir(&canon_dir, &manifest.world_xml, Some("worldbody"))?;
        let world_xml = files
            .get(&manifest.world_xml)
            .and_then(|b| std::str::from_utf8(b).ok())
            .ok_or_else(|| PackFileError::InvalidPack("world XML is not valid UTF-8".into()))?
            .to_string();
        if check_assets {
            verify_pack_assets_from_map(&files, &manifest.assets)?;
        }
        Ok(ValidUserPack {
            canon_dir,
            manifest,
            xml: world_xml,
            digest,
            files,
        })
    }

    fn load_user_tool_checked(
        &self,
        id: &str,
        check_assets: bool,
    ) -> Result<ValidUserPack<ToolManifest>, PackFileError> {
        let tools_root = self.root.join("tools");
        let dir = tools_root.join(id);
        let canon_dir = check_safe_pack_dir(&tools_root, &dir)?;
        let manifest_str = read_safe_pack_file(&canon_dir, "tool.json")?;
        let mut manifest: ToolManifest = serde_json::from_str(&manifest_str)
            .map_err(|e| PackFileError::InvalidFormat(format!("invalid tool.json: {e}")))?;
        manifest.id = id.to_string();
        let (files, digest) = collect_and_validate_pack_dir(&canon_dir, &manifest.tool_xml, None)?;
        let tool_xml = files
            .get(&manifest.tool_xml)
            .and_then(|b| std::str::from_utf8(b).ok())
            .ok_or_else(|| PackFileError::InvalidPack("tool XML is not valid UTF-8".into()))?
            .to_string();
        if check_assets {
            verify_pack_assets_from_map(&files, &manifest.assets)?;
        }
        Ok(ValidUserPack {
            canon_dir,
            manifest,
            xml: tool_xml,
            digest,
            files,
        })
    }

    pub fn resolve_scene(&self, id: &str) -> Option<(SceneManifest, String)> {
        if !validate_pack_id(id) {
            return None;
        }
        if let Some(conflict_dir) = self.user_scene_conflict(id) {
            tracing::error!(
                "User scene directory '{}' conflicts with reserved builtin scene '{}'; refusing to load or override",
                conflict_dir.display(),
                id
            );
            return None;
        }
        if is_reserved_builtin_id("scene", id) {
            return load_builtin_scene_in_roots(id, &self.effective_builtin_roots());
        }
        let pack = self.load_user_scene_checked(id, true).ok()?;
        Some((pack.manifest, pack.xml))
    }

    fn resolve_scene_checked_full(
        &self,
        id: &str,
    ) -> Result<ResolvedPack<SceneManifest>, (StatusCode, Json<serde_json::Value>)> {
        if !validate_pack_id(id) {
            return Err((
                StatusCode::BAD_REQUEST,
                Json(serde_json::json!({
                    "error": format!("Invalid scene pack id '{id}'"),
                    "code": "INVALID_ID",
                })),
            ));
        }
        if let Some(conflict_dir) = self.user_scene_conflict(id) {
            return Err((
                StatusCode::CONFLICT,
                Json(serde_json::json!({
                    "error": format!(
                        "User scene directory '{}' conflicts with reserved builtin scene '{}'; remove or rename the user directory",
                        conflict_dir.display(),
                        id
                    ),
                    "code": "BUILTIN_ID_CONFLICT",
                    "scene_id": id,
                    "conflict_path": conflict_dir.to_string_lossy(),
                })),
            ));
        }
        if is_reserved_builtin_id("scene", id) {
            let roots = self.effective_builtin_roots();
            if id != "default" {
                if let (_, Some(pack)) = find_valid_builtin_scene_dir(id, &roots) {
                    return Ok(ResolvedPack {
                        canon_dir: Some(pack.canon_dir),
                        manifest: pack.manifest,
                        xml: pack.xml,
                        digest: Some(pack.digest),
                        files: Some(pack.files),
                    });
                }
            }
            return load_builtin_scene_in_roots(id, &roots)
                .map(|(m, w)| ResolvedPack {
                    canon_dir: None,
                    manifest: m,
                    xml: w,
                    digest: None,
                    files: None,
                })
                .ok_or_else(|| {
                    (
                        StatusCode::NOT_FOUND,
                        Json(serde_json::json!({
                            "error": format!("Builtin scene '{id}' has no installed valid assets"),
                            "code": "SCENE_NOT_INSTALLED",
                            "scene_id": id,
                        })),
                    )
                });
        }
        let pack = self
            .load_user_scene_checked(id, true)
            .map_err(|e| e.into_http_error("SCENE_NOT_FOUND", &format!("Scene pack '{id}'")))?;
        Ok(ResolvedPack {
            canon_dir: Some(pack.canon_dir),
            manifest: pack.manifest,
            xml: pack.xml,
            digest: Some(pack.digest),
            files: Some(pack.files),
        })
    }

    pub fn resolve_scene_checked(
        &self,
        id: &str,
    ) -> Result<(SceneManifest, String), (StatusCode, Json<serde_json::Value>)> {
        let pack = self.resolve_scene_checked_full(id)?;
        Ok((pack.manifest, pack.xml))
    }

    pub fn resolve_tool(&self, id: &str) -> Option<(ToolManifest, String)> {
        if !validate_pack_id(id) {
            return None;
        }
        if let Some(conflict_dir) = self.user_tool_conflict(id) {
            tracing::error!(
                "User tool directory '{}' conflicts with reserved builtin tool '{}'; refusing to load or override",
                conflict_dir.display(),
                id
            );
            return None;
        }
        if is_reserved_builtin_id("tool", id) {
            return load_builtin_tool_in_roots(id, &self.effective_builtin_roots());
        }
        let pack = self.load_user_tool_checked(id, true).ok()?;
        Some((pack.manifest, pack.xml))
    }

    fn resolve_tool_checked_full(
        &self,
        id: &str,
    ) -> Result<ResolvedPack<ToolManifest>, (StatusCode, Json<serde_json::Value>)> {
        if !validate_pack_id(id) {
            return Err((
                StatusCode::BAD_REQUEST,
                Json(serde_json::json!({
                    "error": format!("Invalid tool pack id '{id}'"),
                    "code": "INVALID_ID",
                })),
            ));
        }
        if let Some(conflict_dir) = self.user_tool_conflict(id) {
            return Err((
                StatusCode::CONFLICT,
                Json(serde_json::json!({
                    "error": format!(
                        "User tool directory '{}' conflicts with reserved builtin tool '{}'; remove or rename the user directory",
                        conflict_dir.display(),
                        id
                    ),
                    "code": "BUILTIN_ID_CONFLICT",
                    "tool_id": id,
                    "conflict_path": conflict_dir.to_string_lossy(),
                })),
            ));
        }
        if is_reserved_builtin_id("tool", id) {
            let roots = self.effective_builtin_roots();
            if id != "default" {
                if let (_, Some(pack)) = find_valid_builtin_tool_dir(id, &roots) {
                    return Ok(ResolvedPack {
                        canon_dir: Some(pack.canon_dir),
                        manifest: pack.manifest,
                        xml: pack.xml,
                        digest: Some(pack.digest),
                        files: Some(pack.files),
                    });
                }
            }
            return load_builtin_tool_in_roots(id, &roots)
                .map(|(m, w)| ResolvedPack {
                    canon_dir: None,
                    manifest: m,
                    xml: w,
                    digest: None,
                    files: None,
                })
                .ok_or_else(|| {
                    (
                        StatusCode::NOT_FOUND,
                        Json(serde_json::json!({
                            "error": format!("Builtin tool '{id}' has no installed valid assets"),
                            "code": "TOOL_NOT_INSTALLED",
                            "tool_id": id,
                        })),
                    )
                });
        }
        let pack = self
            .load_user_tool_checked(id, true)
            .map_err(|e| e.into_http_error("TOOL_NOT_FOUND", &format!("Tool pack '{id}'")))?;
        Ok(ResolvedPack {
            canon_dir: Some(pack.canon_dir),
            manifest: pack.manifest,
            xml: pack.xml,
            digest: Some(pack.digest),
            files: Some(pack.files),
        })
    }

    pub fn resolve_tool_checked(
        &self,
        id: &str,
    ) -> Result<(ToolManifest, String), (StatusCode, Json<serde_json::Value>)> {
        let pack = self.resolve_tool_checked_full(id)?;
        Ok((pack.manifest, pack.xml))
    }

    fn resolve_policy_checked(
        &self,
        id: &str,
        missing_status: StatusCode,
        missing_code: &str,
    ) -> Result<PolicyManifest, (StatusCode, Json<serde_json::Value>)> {
        if !validate_pack_id(id) {
            return Err((
                StatusCode::BAD_REQUEST,
                Json(serde_json::json!({
                    "error": format!("Invalid policy pack id '{id}'"),
                    "code": "INVALID_ID",
                })),
            ));
        }
        let policies_root = self.root.join("policies");
        let dir = policies_root.join(id);
        let canon_dir = check_safe_pack_dir(&policies_root, &dir).map_err(|e| match e {
            PackFileError::Missing(msg) => (
                missing_status,
                Json(serde_json::json!({
                    "error": format!("Policy pack '{id}': {msg}"),
                    "code": missing_code,
                    "policy_id": id,
                })),
            ),
            other => other.into_http_error(missing_code, &format!("Policy pack '{id}'")),
        })?;
        let manifest_str = read_safe_pack_file(&canon_dir, "policy.json").map_err(|e| match e {
            PackFileError::Missing(msg) => (
                missing_status,
                Json(serde_json::json!({
                    "error": format!("Policy pack '{id}': {msg}"),
                    "code": missing_code,
                    "policy_id": id,
                })),
            ),
            other => other.into_http_error(missing_code, &format!("Policy pack '{id}'")),
        })?;
        let mut m: PolicyManifest = serde_json::from_str(&manifest_str).map_err(|e| {
            (
                StatusCode::BAD_REQUEST,
                Json(serde_json::json!({
                    "error": format!("Policy pack '{id}' has invalid policy.json: {e}"),
                    "code": "INVALID_POLICY_MANIFEST",
                    "policy_id": id,
                })),
            )
        })?;
        if m.id.is_empty() {
            m.id = id.to_string();
        }
        if m.name.is_empty() {
            m.name = id.to_string();
        }
        check_safe_pack_file(&canon_dir, &m.model_file).map_err(|e| match e {
            PackFileError::Missing(msg) => (
                missing_status,
                Json(serde_json::json!({
                    "error": format!("Policy pack '{id}' missing model file: {msg}"),
                    "code": missing_code,
                    "policy_id": id,
                })),
            ),
            other => other.into_http_error(missing_code, &format!("Policy pack '{id}'")),
        })?;
        Ok(m)
    }

    pub fn resolve_policy(&self, id: &str) -> Option<PolicyManifest> {
        self.resolve_policy_checked(id, StatusCode::NOT_FOUND, "POLICY_NOT_FOUND")
            .ok()
    }

    pub fn policy_dir(&self, id: &str) -> Option<PathBuf> {
        if !validate_pack_id(id) {
            return None;
        }
        let dir = self.root.join("policies").join(id);
        dir.is_dir().then_some(dir)
    }

    fn collect_pack_pre_fingerprint(
        &self,
        scene_id: &str,
        tool_id: &str,
        policy_id: Option<&str>,
        base_xml: &str,
        base_xml_watch_paths: &[PathBuf],
    ) -> PackStatFingerprint {
        let mut roots = Vec::new();
        let mut absent_watch_dirs = Vec::new();

        for (kind_singular, kind_dir, id) in [
            ("scene", "scenes", scene_id),
            ("tool", "tools", tool_id),
        ] {
            if is_reserved_builtin_id(kind_singular, id) {
                absent_watch_dirs.push(self.root.join(kind_dir).join(id));
                if id != "default" {
                    for root in self.effective_builtin_roots() {
                        let candidate = root.join(kind_dir).join(id);
                        if let Ok(canon) = check_safe_pack_dir(&root, &candidate) {
                            roots.push(canon);
                        } else if std::fs::symlink_metadata(&candidate).is_err() {
                            absent_watch_dirs.push(candidate);
                        }
                    }
                }
            } else {
                let pack_root = self.root.join(kind_dir);
                let dir = pack_root.join(id);
                if let Ok(canon) = check_safe_pack_dir(&pack_root, &dir) {
                    roots.push(canon);
                } else if std::fs::symlink_metadata(&dir).is_err() {
                    absent_watch_dirs.push(dir);
                }
            }
        }

        if let Some(pid) = policy_id {
            let policies_root = self.root.join("policies");
            let dir = policies_root.join(pid);
            if let Ok(canon) = check_safe_pack_dir(&policies_root, &dir) {
                roots.push(canon);
            } else if std::fs::symlink_metadata(&dir).is_err() {
                absent_watch_dirs.push(dir);
            }
        }

        let mut extra_watch_paths = Vec::new();
        let mut seen_extra = HashSet::new();
        for p in base_xml_watch_paths
            .iter()
            .cloned()
            .chain(collect_base_mesh_watch_paths(base_xml))
        {
            if seen_extra.insert(p.clone()) {
                extra_watch_paths.push(p);
            }
        }
        PackStatFingerprint::collect(roots, absent_watch_dirs, extra_watch_paths)
    }
}

/// The rebot-sim server (`ARMD_SIM_URL`). There is no fallback: the detector URL is
/// rebot-vision on the device, which does not serve `/sim/*`. Unset or empty is a 503.
pub fn sim_upstream_url() -> Result<String, (StatusCode, Json<serde_json::Value>)> {
    std::env::var("ARMD_SIM_URL").ok().filter(|u| !u.trim().is_empty()).ok_or_else(|| {
        (
            StatusCode::SERVICE_UNAVAILABLE,
            Json(serde_json::json!({
                "error": "Sim mode needs ARMD_SIM_URL (the rebot-sim server URL); it is not set",
                "code": "SIM_NOT_CONFIGURED",
            })),
        )
    })
}

// ---------------------------------------------------------------------------
// HTTP Handlers
// ---------------------------------------------------------------------------

struct ActiveCatalogSnapshot {
    mode: String,
    active_scene: String,
    active_tool: String,
    active_policy: Option<String>,
}

fn build_catalog_blocking(
    state: &SandboxState,
    active: &ActiveCatalogSnapshot,
    running_robot: &str,
) -> serde_json::Value {
    let roots = state.effective_builtin_roots();

    let mut scenes_out = Vec::new();
    let mut conflicts_out = Vec::new();
    for spec in BUILTIN_SCENE_SPECS {
        let conflict_dir = state.user_scene_conflict(spec.id);
        let has_conflict = conflict_dir.is_some();
        let resolved = if !has_conflict {
            load_builtin_scene_in_roots(spec.id, &roots)
        } else {
            None
        };
        let installed = if spec.is_virtual {
            !has_conflict
        } else {
            resolved.is_some()
        };
        let mut entry = serde_json::json!({
            "id": spec.id,
            "name": spec.name,
            "description": spec.description,
            "builtin": true,
            "virtual": spec.is_virtual,
            "installed": installed,
            "robots": spec.robots,
            "default_tool": spec.default_tool,
            "recommended_tools": spec.recommended_tools,
        });
        if let Some(ref cdir) = conflict_dir {
            let reason = format!(
                "User scene directory '{}' conflicts with reserved builtin scene '{}'",
                cdir.display(),
                spec.id
            );
            tracing::warn!("{reason}");
            entry["conflict"] = serde_json::json!(true);
            entry["conflict_reason"] = serde_json::json!(reason);
            conflicts_out.push(serde_json::json!({
                "kind": "scene",
                "id": spec.id,
                "path": cdir.to_string_lossy(),
                "reason": reason,
            }));
        }
        scenes_out.push(entry);
    }

    if let Ok(entries) = std::fs::read_dir(state.root.join("scenes")) {
        for e in entries.flatten() {
            if let Ok(ft) = e.file_type() {
                if ft.is_dir() {
                    let id = e.file_name().to_string_lossy().to_string();
                    if !validate_pack_id(&id) || is_reserved_builtin_id("scene", &id) {
                        continue;
                    }
                    if !scenes_out.iter().any(|s| s["id"] == id) {
                        if let Ok(pack) = state.load_user_scene_checked(&id, false) {
                            let assets_ok = verify_pack_assets_from_map(&pack.files, &pack.manifest.assets).is_ok();
                            scenes_out.push(serde_json::json!({
                                "id": id,
                                "name": pack.manifest.name,
                                "version": pack.manifest.version,
                                "builtin": false,
                                "virtual": false,
                                "installed": assets_ok,
                                "robots": pack.manifest.robots,
                                "default_tool": pack.manifest.default_tool,
                            }));
                        }
                    }
                }
            }
        }
    }

    let mut tools_out = Vec::new();
    for spec in BUILTIN_TOOL_SPECS {
        let conflict_dir = state.user_tool_conflict(spec.id);
        let has_conflict = conflict_dir.is_some();
        let resolved = if !has_conflict {
            load_builtin_tool_in_roots(spec.id, &roots)
        } else {
            None
        };
        let installed = if spec.is_virtual {
            !has_conflict
        } else {
            resolved.is_some()
        };
        let mut entry = serde_json::json!({
            "id": spec.id,
            "name": spec.name,
            "kind": spec.kind,
            "description": spec.description,
            "builtin": true,
            "virtual": spec.is_virtual,
            "installed": installed,
            "robots": spec.robots,
        });
        if let Some(ref cdir) = conflict_dir {
            let reason = format!(
                "User tool directory '{}' conflicts with reserved builtin tool '{}'",
                cdir.display(),
                spec.id
            );
            tracing::warn!("{reason}");
            entry["conflict"] = serde_json::json!(true);
            entry["conflict_reason"] = serde_json::json!(reason);
            conflicts_out.push(serde_json::json!({
                "kind": "tool",
                "id": spec.id,
                "path": cdir.to_string_lossy(),
                "reason": reason,
            }));
        }
        tools_out.push(entry);
    }

    if let Ok(entries) = std::fs::read_dir(state.root.join("tools")) {
        for e in entries.flatten() {
            if let Ok(ft) = e.file_type() {
                if ft.is_dir() {
                    let id = e.file_name().to_string_lossy().to_string();
                    if !validate_pack_id(&id) || is_reserved_builtin_id("tool", &id) {
                        continue;
                    }
                    if !tools_out.iter().any(|t| t["id"] == id) {
                        if let Ok(pack) = state.load_user_tool_checked(&id, false) {
                            let assets_ok = verify_pack_assets_from_map(&pack.files, &pack.manifest.assets).is_ok();
                            tools_out.push(serde_json::json!({
                                "id": id,
                                "name": pack.manifest.name,
                                "kind": pack.manifest.kind,
                                "version": pack.manifest.version,
                                "builtin": false,
                                "virtual": false,
                                "installed": assets_ok,
                                "robots": pack.manifest.robots,
                            }));
                        }
                    }
                }
            }
        }
    }

    let mut policies_out = Vec::new();
    if let Ok(entries) = std::fs::read_dir(state.root.join("policies")) {
        for e in entries.flatten() {
            if let Ok(ft) = e.file_type() {
                if ft.is_dir() {
                    let id = e.file_name().to_string_lossy().to_string();
                    if let Some(m) = state.resolve_policy(&id) {
                        policies_out.push(serde_json::json!({
                            "id": m.id,
                            "name": m.name,
                            "format": m.format,
                            "robot": m.robot,
                            "action_dim": m.action_dim,
                        }));
                    }
                }
            }
        }
    }

    let builtin_scene_ids: Vec<&'static str> = BUILTIN_SCENE_SPECS.iter().map(|s| s.id).collect();
    let builtin_tool_ids: Vec<&'static str> = BUILTIN_TOOL_SPECS.iter().map(|t| t.id).collect();
    serde_json::json!({
        "mode": active.mode,
        "robot": running_robot,
        "active_scene": active.active_scene,
        "active_tool": active.active_tool,
        "active_policy": active.active_policy,
        "wasm_runtime_ready": assets::wasm_runtime_ready(),
        "builtin_scenes": builtin_scene_ids,
        "builtin_tools": builtin_tool_ids,
        "scenes": scenes_out,
        "tools": tools_out,
        "policies": policies_out,
        "conflicts": conflicts_out,
    })
}

/// GET /api/v1/sandbox/catalog
pub async fn get_catalog(
    State(supervisor): State<Arc<crate::supervisor::ArmSupervisor>>,
    Extension(state): Extension<Arc<SandboxState>>,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<serde_json::Value>)> {
    let active_snapshot = {
        let active = state.active.read().await;
        ActiveCatalogSnapshot {
            mode: active.mode.clone(),
            active_scene: active.active_scene.clone(),
            active_tool: active.active_tool.clone(),
            active_policy: active.active_policy.clone(),
        }
    };
    let running_robot = running_robot_id(&supervisor).await;
    let state_scan = state.clone();
    let catalog = tokio::task::spawn_blocking(move || {
        build_catalog_blocking(&state_scan, &active_snapshot, &running_robot)
    })
    .await
    .map_err(|e| {
        (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(serde_json::json!({"error": format!("Catalog scan failed: {e}")})),
        )
    })?;
    Ok(Json(catalog))
}

// ---------------------------------------------------------------------------
// Pack uploads
// ---------------------------------------------------------------------------

enum FileValueError {
    Base64(String),
    Unsupported,
}

/// Decodes an uploaded file value: a string, or `{"encoding": "base64"|"utf8", "data": ...}`.
/// With `marked_base64_only`, a string is decoded only when it carries a `base64:` or
/// `;base64,` marker and a bad decode is an error; otherwise a string is decoded when it is
/// valid Base64 and kept as text when it is not.
fn decode_file_value(val: serde_json::Value, marked_base64_only: bool) -> Result<Vec<u8>, FileValueError> {
    match val {
        serde_json::Value::String(s) => {
            if !marked_base64_only {
                Ok(base64_decode(&s).unwrap_or_else(|_| s.into_bytes()))
            } else if s.starts_with("base64:") || s.contains(";base64,") {
                base64_decode(&s).map_err(FileValueError::Base64)
            } else {
                Ok(s.into_bytes())
            }
        }
        serde_json::Value::Object(map) => {
            let enc = map.get("encoding").and_then(|v| v.as_str()).unwrap_or("utf8");
            let data = map.get("data").and_then(|v| v.as_str()).unwrap_or("");
            if enc.eq_ignore_ascii_case("base64") {
                base64_decode(data).map_err(FileValueError::Base64)
            } else {
                Ok(data.as_bytes().to_vec())
            }
        }
        _ => Err(FileValueError::Unsupported),
    }
}

fn extract_files_map(
    files: Option<HashMap<String, serde_json::Value>>,
    primary_json: Option<(&str, &serde_json::Value)>,
    primary_xml: Option<(&str, &str)>,
    assets: Option<HashMap<String, serde_json::Value>>,
) -> Result<HashMap<String, Vec<u8>>, (StatusCode, String)> {
    let mut out: HashMap<String, Vec<u8>> = HashMap::new();

    if let Some(fmap) = files {
        for (rel, val) in fmap {
            if !is_safe_relative_path(&rel) {
                return Err((StatusCode::BAD_REQUEST, format!("Invalid or escaping relative path: {rel}")));
            }
            let bytes = decode_file_value(val, true).map_err(|e| match e {
                FileValueError::Base64(e) => (StatusCode::BAD_REQUEST, format!("Base64 decode error on {rel}: {e}")),
                FileValueError::Unsupported => (StatusCode::BAD_REQUEST, format!("Unsupported file value format for {rel}")),
            })?;
            out.insert(rel, bytes);
        }
    }

    if let Some((name, json_val)) = primary_json {
        let bytes = serde_json::to_vec_pretty(json_val)
            .map_err(|e| (StatusCode::BAD_REQUEST, format!("Failed to serialize {name}: {e}")))?;
        out.insert(name.to_string(), bytes);
    }

    if let Some((name, xml_str)) = primary_xml {
        out.insert(name.to_string(), xml_str.as_bytes().to_vec());
    }

    if let Some(ast) = assets {
        for (rel, val) in ast {
            if !is_safe_relative_path(&rel) {
                return Err((StatusCode::BAD_REQUEST, format!("Invalid asset relative path: {rel}")));
            }
            let bytes = decode_file_value(val, false).map_err(|e| match e {
                FileValueError::Base64(e) => (StatusCode::BAD_REQUEST, format!("Base64 decode error on {rel}: {e}")),
                FileValueError::Unsupported => (StatusCode::BAD_REQUEST, format!("Unsupported asset value format for {rel}")),
            })?;
            out.insert(rel, bytes);
        }
    }

    Ok(out)
}

pub fn count_custom_packs(kind_dir: &Path) -> usize {
    let mut count = 0;
    if let Ok(entries) = std::fs::read_dir(kind_dir) {
        for entry in entries.flatten() {
            let path = entry.path();
            if let Some(name) = path.file_name().and_then(|n| n.to_str()) {
                if name.starts_with('.') {
                    continue;
                }
            }
            if let Ok(ft) = entry.file_type() {
                if ft.is_dir() {
                    count += 1;
                }
            }
        }
    }
    count
}

pub fn dir_size_excluding(dir: &Path, exclude: Option<&Path>) -> u64 {
    let mut total = 0u64;
    if !dir.exists() {
        return 0;
    }
    if let Ok(entries) = std::fs::read_dir(dir) {
        for entry in entries.flatten() {
            let path = entry.path();
            if let Some(ex) = exclude {
                if path == ex {
                    continue;
                }
            }
            if let Some(name) = path.file_name().and_then(|n| n.to_str()) {
                if name.starts_with('.') {
                    continue;
                }
            }
            if let Ok(ft) = entry.file_type() {
                if ft.is_dir() {
                    total = total.saturating_add(dir_size_excluding(&path, exclude));
                } else if ft.is_file() {
                    if let Ok(meta) = entry.metadata() {
                        total = total.saturating_add(meta.len());
                    }
                }
            }
        }
    }
    total
}

pub fn enforce_sandbox_quota(
    root: &Path,
    kind: &str,
    target_dir: &Path,
    new_pack_bytes: u64,
    max_packs: usize,
    max_total_bytes: u64,
) -> Result<(), (StatusCode, String)> {
    let kind_dir = root.join(kind);
    if !target_dir.exists() {
        let count = count_custom_packs(&kind_dir);
        if count >= max_packs {
            return Err((
                StatusCode::INSUFFICIENT_STORAGE,
                format!("Maximum number of custom {kind} packs ({max_packs}) reached"),
            ));
        }
    }

    let existing_size = dir_size_excluding(root, Some(target_dir));
    let projected_size = existing_size.saturating_add(new_pack_bytes);
    if projected_size > max_total_bytes {
        return Err((
            StatusCode::INSUFFICIENT_STORAGE,
            format!(
                "Sandbox storage quota exceeded: total size would be {projected_size} bytes (limit: {max_total_bytes} bytes)"
            ),
        ));
    }

    Ok(())
}

fn write_pack_atomically(target_dir: &Path, files: &HashMap<String, Vec<u8>>, max_bytes: u64) -> Result<String, String> {
    let total_bytes: usize = files.values().map(|v| v.len()).sum();
    if total_bytes > max_bytes as usize {
        return Err(format!("Total pack size {total_bytes} exceeds limit of {max_bytes} bytes"));
    }

    let parent = target_dir.parent().unwrap_or(target_dir);
    let counter = STAGING_COUNTER.fetch_add(1, Ordering::Relaxed);
    let ts = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0);
    let target_name = target_dir.file_name().unwrap_or_default().to_string_lossy();
    let tmp_dir = parent.join(format!(".staging.{}.{}.{}.{}", target_name, std::process::id(), ts, counter));

    let _ = std::fs::remove_dir_all(&tmp_dir);
    std::fs::create_dir_all(&tmp_dir).map_err(|e| format!("Failed to create temp dir: {e}"))?;

    let mut sorted_keys: Vec<&String> = files.keys().collect();
    sorted_keys.sort();

    let mut hasher = Sha256::new();

    for key in sorted_keys {
        let bytes = &files[key];
        hasher.update(key.as_bytes());
        hasher.update(&[0]);
        hasher.update(&(bytes.len() as u64).to_le_bytes());
        hasher.update(bytes);

        let dest = tmp_dir.join(key);
        if let Some(p) = dest.parent() {
            if let Err(e) = std::fs::create_dir_all(p) {
                let _ = std::fs::remove_dir_all(&tmp_dir);
                return Err(format!("Failed to create parent dir: {e}"));
            }
        }
        if let Err(e) = std::fs::write(&dest, bytes) {
            let _ = std::fs::remove_dir_all(&tmp_dir);
            return Err(format!("Failed to write file {key}: {e}"));
        }
    }

    let digest = hasher.finalize_hex();

    let _lock = TARGET_REPLACE_MUTEX.lock().map_err(|_| "Target replacement lock poisoned".to_string())?;

    let backup_dir = parent.join(format!(".old.{}.{}.{}.{}", target_name, std::process::id(), ts, counter));
    let had_target = target_dir.exists();
    if had_target {
        std::fs::rename(target_dir, &backup_dir)
            .map_err(|e| {
                let _ = std::fs::remove_dir_all(&tmp_dir);
                format!("Failed to move old pack to backup: {e}")
            })?;
    }

    if let Err(e) = std::fs::rename(&tmp_dir, target_dir) {
        if had_target {
            let _ = std::fs::rename(&backup_dir, target_dir);
        }
        let _ = std::fs::remove_dir_all(&tmp_dir);
        return Err(format!("Failed to activate new pack: {e}"));
    }

    if had_target {
        let _ = std::fs::remove_dir_all(&backup_dir);
    }

    Ok(digest)
}

#[derive(Debug, Deserialize)]
pub struct UploadSceneRequest {
    pub id: Option<String>,
    pub files: Option<HashMap<String, serde_json::Value>>,
    pub scene_json: Option<serde_json::Value>,
    pub world_xml: Option<String>,
    pub assets: Option<HashMap<String, serde_json::Value>>,
}

#[allow(clippy::too_many_arguments)]
fn validate_and_write_pack<M: serde::de::DeserializeOwned + serde::Serialize>(
    root: &Path,
    kind: &str,
    dir_name: &str,
    json_name: &str,
    primary_json: Option<(&str, &serde_json::Value)>,
    primary_xml: Option<(&str, &str)>,
    files: Option<HashMap<String, serde_json::Value>>,
    assets: Option<HashMap<String, serde_json::Value>>,
    override_id: Option<String>,
    required_elem: Option<&str>,
    parse_meta: fn(&M) -> (&str, &str, &[serde_json::Value]),
    set_id: fn(&mut M, String),
) -> Result<(String, serde_json::Value), (StatusCode, &'static str, String)> {
    let mut file_map = extract_files_map(files, primary_json, primary_xml, assets)
        .map_err(|(sc, msg)| (sc, "INVALID_PACK", msg))?;

    for key in file_map.keys() {
        if is_urdf_path(key) {
            return Err((
                StatusCode::BAD_REQUEST,
                "UNSUPPORTED_FORMAT",
                format!("URDF files are not supported: '{key}'"),
            ));
        }
    }

    let json_bytes = file_map
        .get(json_name)
        .ok_or_else(|| (StatusCode::BAD_REQUEST, "INVALID_PACK", format!("Missing {json_name}")))?;

    let mut manifest_val: serde_json::Value = serde_json::from_slice(json_bytes)
        .map_err(|e| (StatusCode::BAD_REQUEST, "INVALID_PACK", format!("Invalid {json_name}: {e}")))?;

    let mut manifest: M = serde_json::from_value(manifest_val.clone())
        .map_err(|e| (StatusCode::BAD_REQUEST, "INVALID_PACK", format!("Invalid {json_name}: {e}")))?;

    let pack_id = override_id.unwrap_or_else(|| parse_meta(&manifest).0.to_string());
    if !validate_pack_id(&pack_id) {
        return Err((
            StatusCode::BAD_REQUEST,
            "INVALID_ID",
            format!("Invalid {kind} pack id '{pack_id}'"),
        ));
    }
    if is_reserved_builtin_id(kind, &pack_id) {
        return Err((
            StatusCode::BAD_REQUEST,
            "RESERVED_ID",
            format!("Cannot overwrite reserved builtin {kind} '{pack_id}'"),
        ));
    }
    if parse_meta(&manifest).0 != pack_id {
        set_id(&mut manifest, pack_id.clone());
        if let Some(obj) = manifest_val.as_object_mut() {
            obj.insert("id".into(), serde_json::Value::String(pack_id.clone()));
        }
        let rewritten = serde_json::to_vec_pretty(&manifest_val).map_err(|e| {
            (StatusCode::BAD_REQUEST, "INVALID_PACK", format!("Failed to serialize {json_name}: {e}"))
        })?;
        file_map.insert(json_name.into(), rewritten);
    }

    let btree: BTreeMap<String, Vec<u8>> =
        file_map.iter().map(|(k, v)| (k.clone(), v.clone())).collect();
    let (_, primary_xml_rel, manifest_assets) = parse_meta(&manifest);
    verify_pack_assets_in_map(&btree, manifest_assets)
        .map_err(|e| (StatusCode::BAD_REQUEST, e.code(), e.message().to_string()))?;
    let digest = validate_pack_map(primary_xml_rel, required_elem, &btree)
        .map_err(|e| (StatusCode::BAD_REQUEST, e.code(), e.message().to_string()))?;

    let target_dir = root.join(dir_name).join(&pack_id);
    let total_pack_bytes: u64 = file_map.values().map(|v| v.len() as u64).sum();
    enforce_sandbox_quota(
        root,
        dir_name,
        &target_dir,
        total_pack_bytes,
        MAX_CUSTOM_PACKS_PER_KIND,
        MAX_SANDBOX_STORAGE_BYTES,
    )
    .map_err(|(sc, msg)| (sc, "QUOTA_EXCEEDED", msg))?;

    let written_digest = write_pack_atomically(&target_dir, &file_map, MAX_PACK_TOTAL_BYTES as u64)
        .map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, "WRITE_FAILED", e))?;
    debug_assert_eq!(digest, written_digest);

    let mut file_names: Vec<String> = file_map.keys().cloned().collect();
    file_names.sort();

    Ok((
        pack_id.clone(),
        serde_json::json!({
            "id": pack_id,
            "sha256": digest,
            "pack_digest": digest,
            "path": target_dir.to_string_lossy(),
            "manifest": manifest,
            "files": file_names,
        }),
    ))
}

/// POST /api/v1/sandbox/scenes
pub async fn upload_scene(
    Extension(state): Extension<Arc<SandboxState>>,
    permit_slot: Option<Extension<UploadPermitSlot>>,
    Json(payload): Json<UploadSceneRequest>,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<serde_json::Value>)> {
    let _act_guard = state.activate_lock.lock().await;
    {
        let mut st = state.active.write().await;
        if payload.id.as_ref().is_none_or(|id| id == &st.active_scene) {
            st.cached_mjcf.clear();
            st.committed_scene_digest = None;
        }
    }
    if let Some(ref id) = payload.id {
        state.invalidate_bundle_cache_for_scene(id).await;
    } else {
        state.invalidate_bundle_cache_all().await;
    }
    let root = state.root.clone();
    let permit = permit_slot.and_then(|Extension(slot)| slot.0.lock().ok().and_then(|mut g| g.take()));
    let res = tokio::task::spawn_blocking(move || {
        let _permit = permit;
        let primary_json = payload.scene_json.as_ref().map(|v| ("scene.json", v));
        let primary_xml = payload.world_xml.as_deref().map(|v| ("world.xml", v));
        validate_and_write_pack::<SceneManifest>(
            &root,
            "scene",
            "scenes",
            "scene.json",
            primary_json,
            primary_xml,
            payload.files,
            payload.assets,
            payload.id,
            Some("worldbody"),
            |m| (&m.id, &m.world_xml, &m.assets),
            |m, id| m.id = id,
        )
    })
    .await
    .map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, Json(serde_json::json!({"error": format!("Blocking task failed: {e}")}))))?;

    match res {
        Ok((pack_id, val)) => {
            {
                let mut st = state.active.write().await;
                if st.active_scene == pack_id {
                    st.cached_mjcf.clear();
                    st.committed_scene_digest = None;
                }
            }
            state.invalidate_bundle_cache_for_scene(&pack_id).await;
            Ok(Json(val))
        }
        Err((sc, code, msg)) => Err((sc, Json(serde_json::json!({"error": msg, "code": code})))),
    }
}

#[derive(Debug, Deserialize)]
pub struct UploadToolRequest {
    pub id: Option<String>,
    pub files: Option<HashMap<String, serde_json::Value>>,
    pub tool_json: Option<serde_json::Value>,
    pub tool_xml: Option<String>,
    pub assets: Option<HashMap<String, serde_json::Value>>,
}

/// POST /api/v1/sandbox/tools
pub async fn upload_tool(
    Extension(state): Extension<Arc<SandboxState>>,
    permit_slot: Option<Extension<UploadPermitSlot>>,
    Json(payload): Json<UploadToolRequest>,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<serde_json::Value>)> {
    let _act_guard = state.activate_lock.lock().await;
    {
        let mut st = state.active.write().await;
        if payload.id.as_ref().is_none_or(|id| id == &st.active_tool) {
            st.cached_mjcf.clear();
            st.committed_tool_digest = None;
        }
    }
    if let Some(ref id) = payload.id {
        state.invalidate_bundle_cache_for_tool(id).await;
    } else {
        state.invalidate_bundle_cache_all().await;
    }
    let root = state.root.clone();
    let permit = permit_slot.and_then(|Extension(slot)| slot.0.lock().ok().and_then(|mut g| g.take()));
    let res = tokio::task::spawn_blocking(move || {
        let _permit = permit;
        let primary_json = payload.tool_json.as_ref().map(|v| ("tool.json", v));
        let primary_xml = payload.tool_xml.as_deref().map(|v| ("tool.xml", v));
        validate_and_write_pack::<ToolManifest>(
            &root,
            "tool",
            "tools",
            "tool.json",
            primary_json,
            primary_xml,
            payload.files,
            payload.assets,
            payload.id,
            None,
            |m| (&m.id, &m.tool_xml, &m.assets),
            |m, id| m.id = id,
        )
    })
    .await
    .map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, Json(serde_json::json!({"error": format!("Blocking task failed: {e}")}))))?;

    match res {
        Ok((pack_id, val)) => {
            {
                let mut st = state.active.write().await;
                if st.active_tool == pack_id {
                    st.cached_mjcf.clear();
                    st.committed_tool_digest = None;
                }
            }
            state.invalidate_bundle_cache_for_tool(&pack_id).await;
            Ok(Json(val))
        }
        Err((sc, code, msg)) => Err((sc, Json(serde_json::json!({"error": msg, "code": code})))),
    }
}

#[derive(Debug, Deserialize)]
pub struct UploadPolicyRequest {
    pub id: Option<String>,
    pub files: Option<HashMap<String, serde_json::Value>>,
    pub policy_json: Option<serde_json::Value>,
    pub model_onnx_base64: Option<String>,
    pub model_onnx: Option<String>,
}

/// POST /api/v1/sandbox/policies
pub async fn upload_policy(
    Extension(state): Extension<Arc<SandboxState>>,
    permit_slot: Option<Extension<UploadPermitSlot>>,
    Json(payload): Json<UploadPolicyRequest>,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<serde_json::Value>)> {
    let _act_guard = state.activate_lock.lock().await;
    let root = state.root.clone();
    let permit = permit_slot.and_then(|Extension(slot)| slot.0.lock().ok().and_then(|mut g| g.take()));
    let res = tokio::task::spawn_blocking(move || -> Result<serde_json::Value, (StatusCode, String)> {
        let _permit = permit;
        let mut file_map: HashMap<String, Vec<u8>> = HashMap::new();

        if let Some(fmap) = payload.files {
            for (rel, val) in fmap {
                if !is_safe_relative_path(&rel) {
                    return Err((StatusCode::BAD_REQUEST, format!("Invalid path {rel}")));
                }
                let bytes = decode_file_value(val, false).map_err(|e| match e {
                    FileValueError::Base64(e) => (StatusCode::BAD_REQUEST, e),
                    FileValueError::Unsupported => (StatusCode::BAD_REQUEST, "Invalid file format".to_string()),
                })?;
                file_map.insert(rel, bytes);
            }
        }

        if let Some(pj) = payload.policy_json {
            let b = serde_json::to_vec_pretty(&pj).map_err(|e| (StatusCode::BAD_REQUEST, e.to_string()))?;
            file_map.insert("policy.json".into(), b);
        }

        if let Some(b64) = payload.model_onnx_base64 {
            let b = base64_decode(&b64).map_err(|e| (StatusCode::BAD_REQUEST, e))?;
            file_map.insert("model.onnx".into(), b);
        } else if let Some(raw) = payload.model_onnx {
            let b = base64_decode(&raw).unwrap_or_else(|_| raw.into_bytes());
            file_map.insert("model.onnx".into(), b);
        }

        let policy_json_bytes = file_map
            .get("policy.json")
            .ok_or_else(|| (StatusCode::BAD_REQUEST, "Missing policy.json".to_string()))?;

        let mut manifest: PolicyManifest = serde_json::from_slice(policy_json_bytes)
            .map_err(|e| (StatusCode::BAD_REQUEST, format!("Invalid policy.json: {e}")))?;

        if manifest.format != "rebot-onnx-policy/v1" {
            return Err((StatusCode::BAD_REQUEST, "policy format must be 'rebot-onnx-policy/v1'".to_string()));
        }

        let pack_id = payload
            .id
            .or_else(|| (!manifest.id.is_empty()).then(|| manifest.id.clone()))
            .unwrap_or_else(|| manifest.name.clone());
        if !validate_pack_id(&pack_id) {
            return Err((StatusCode::BAD_REQUEST, format!("Invalid policy pack id '{pack_id}'")));
        }
        manifest.id = pack_id.clone();
        if manifest.name.is_empty() {
            manifest.name = pack_id.clone();
        }

        let model_bytes = file_map
            .get(&manifest.model_file)
            .ok_or_else(|| (StatusCode::BAD_REQUEST, format!("Missing model file '{}'", manifest.model_file)))?;

        if model_bytes.is_empty() {
            return Err((StatusCode::BAD_REQUEST, "Model ONNX file cannot be empty".to_string()));
        }
        if model_bytes.len() > MAX_POLICY_MODEL_BYTES as usize {
            return Err((StatusCode::BAD_REQUEST, "Model ONNX file exceeds 64 MiB limit".to_string()));
        }

        let target_dir = root.join("policies").join(&pack_id);
        let total_pack_bytes: u64 = file_map.values().map(|v| v.len() as u64).sum();
        enforce_sandbox_quota(
            &root,
            "policies",
            &target_dir,
            total_pack_bytes,
            MAX_CUSTOM_PACKS_PER_KIND,
            MAX_SANDBOX_STORAGE_BYTES,
        )?;

        let digest = write_pack_atomically(&target_dir, &file_map, MAX_POLICY_PACK_BYTES)
            .map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, e))?;

        let mut file_names: Vec<String> = file_map.keys().cloned().collect();
        file_names.sort();

        let model_url = format!("/api/v1/sandbox/policies/{pack_id}/files/{}", manifest.model_file);

        Ok(serde_json::json!({
            "id": pack_id,
            "sha256": digest,
            "path": target_dir.to_string_lossy(),
            "manifest": manifest,
            "files": file_names,
            "model_url": model_url,
        }))
    })
    .await
    .map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, Json(serde_json::json!({"error": format!("Blocking task failed: {e}")}))))?;

    match res {
        Ok(val) => {
            if let Some(pack_id) = val.get("id").and_then(|v| v.as_str()) {
                state.invalidate_bundle_cache_for_policy(pack_id).await;
            } else {
                state.invalidate_bundle_cache_all().await;
            }
            Ok(Json(val))
        }
        Err((sc, msg)) => Err((sc, Json(serde_json::json!({"error": msg})))),
    }
}

fn policy_delete_error(status: StatusCode, code: &str, message: String) -> (StatusCode, Json<serde_json::Value>) {
    (status, Json(serde_json::json!({"error": message, "code": code, "message": message})))
}

/// Move `policies/<id>` to a hidden `.deleted.*` name (one rename: the catalog lists either the
/// whole pack or nothing), then remove it. Leftovers of an interrupted delete are removed too.
fn remove_policy_pack(root: &Path, id: &str) -> std::io::Result<()> {
    let kind_dir = root.join("policies");
    let target_dir = kind_dir.join(id);
    let counter = STAGING_COUNTER.fetch_add(1, Ordering::Relaxed);
    let ts = SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_nanos()).unwrap_or(0);
    let trash = kind_dir.join(format!(".deleted.{id}.{}.{ts}.{counter}", std::process::id()));
    {
        let _lock = TARGET_REPLACE_MUTEX
            .lock()
            .map_err(|_| std::io::Error::other("target replacement lock poisoned"))?;
        std::fs::rename(&target_dir, &trash)?;
    }
    if let Ok(entries) = std::fs::read_dir(&kind_dir) {
        for e in entries.flatten() {
            if e.file_name().to_string_lossy().starts_with(".deleted.") {
                let _ = std::fs::remove_dir_all(e.path());
            }
        }
    }
    Ok(())
}

/// DELETE /api/v1/sandbox/policies/{id} (operator-only). 409 `POLICY_IN_USE` while the pack is
/// the active sandbox policy, a policy run uses it, or a policy is being loaded for a run.
pub async fn delete_policy(
    axum::extract::State(supervisor): axum::extract::State<Arc<crate::supervisor::ArmSupervisor>>,
    Extension(state): Extension<Arc<SandboxState>>,
    UrlPath(id): UrlPath<String>,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<serde_json::Value>)> {
    if !validate_pack_id(&id) {
        return Err(policy_delete_error(StatusCode::BAD_REQUEST, "INVALID_ID", format!("Invalid policy pack id '{id}'")));
    }
    // Global lock order: activate_lock -> supervisor -> state.active.
    // Holding activate_lock across the check + rename ensures concurrent activate() calls
    // (including mode="sim" upstream round-trips) cannot select a policy being deleted.
    let _act_guard = state.activate_lock.lock().await;
    let inner = supervisor.inner().read().await;
    #[cfg(feature = "policy-runtime")]
    {
        if let Some(run) = inner.policy_run.as_ref().filter(|r| r.policy_id == id) {
            return Err(policy_delete_error(
                StatusCode::CONFLICT,
                "POLICY_IN_USE",
                format!("Policy pack '{id}' is used by the running policy action {}", run.action_id),
            ));
        }
        if inner.policy_loading {
            return Err(policy_delete_error(
                StatusCode::CONFLICT,
                "POLICY_IN_USE",
                "A policy pack is being loaded for a run; delete it after the start finishes".into(),
            ));
        }
    }
    #[cfg(not(feature = "policy-runtime"))]
    let _ = &inner;
    let active = state.active.read().await;
    if active.active_policy.as_deref() == Some(id.as_str()) {
        return Err(policy_delete_error(
            StatusCode::CONFLICT,
            "POLICY_IN_USE",
            format!("Policy pack '{id}' is the active sandbox policy; activate another policy or none first"),
        ));
    }
    if state.policy_dir(&id).is_none() {
        return Err(policy_delete_error(StatusCode::NOT_FOUND, "POLICY_NOT_FOUND", format!("Policy pack '{id}' not found")));
    }
    remove_policy_pack(&state.root, &id).map_err(|e| {
        policy_delete_error(StatusCode::INTERNAL_SERVER_ERROR, "DELETE_FAILED", format!("Cannot delete policy pack '{id}': {e}"))
    })?;
    drop(active);
    drop(inner);
    state.invalidate_bundle_cache_for_policy(&id).await;
    Ok(Json(serde_json::json!({"id": id, "deleted": true})))
}

// ---------------------------------------------------------------------------
// Pack metadata & raw file serving
// ---------------------------------------------------------------------------

/// GET /api/v1/sandbox/scenes/{id}
pub async fn get_scene(
    Extension(state): Extension<Arc<SandboxState>>,
    UrlPath(id): UrlPath<String>,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<serde_json::Value>)> {
    let (manifest, _) = state.resolve_scene_checked(&id)?;
    let is_virtual = id == "default";
    let disk_dir = if !is_reserved_builtin_id("scene", &id) {
        Some(state.root.join("scenes").join(&id))
    } else if !is_virtual {
        find_valid_builtin_scene_dir(&id, &state.effective_builtin_roots())
            .1
            .map(|p| p.canon_dir)
    } else {
        None
    };

    let (files, path_val, compiled_preset) = if let Some(ref dir) = disk_dir {
        let canon = dir.canonicalize().unwrap_or_else(|_| dir.clone());
        (
            collect_pack_files(&canon),
            serde_json::json!(dir.to_string_lossy()),
            false,
        )
    } else {
        (Vec::new(), serde_json::Value::Null, !is_virtual)
    };

    Ok(Json(serde_json::json!({
        "id": manifest.id,
        "manifest": manifest,
        "path": path_val,
        "files": files,
        "virtual": is_virtual,
        "compiled_preset": compiled_preset,
    })))
}

/// GET /api/v1/sandbox/tools/{id}
pub async fn get_tool(
    Extension(state): Extension<Arc<SandboxState>>,
    UrlPath(id): UrlPath<String>,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<serde_json::Value>)> {
    let (manifest, _) = state.resolve_tool_checked(&id)?;
    let is_virtual = id == "default";
    let disk_dir = if !is_reserved_builtin_id("tool", &id) {
        Some(state.root.join("tools").join(&id))
    } else if !is_virtual {
        find_valid_builtin_tool_dir(&id, &state.effective_builtin_roots())
            .1
            .map(|p| p.canon_dir)
    } else {
        None
    };

    let (files, path_val, compiled_preset) = if let Some(ref dir) = disk_dir {
        let canon = dir.canonicalize().unwrap_or_else(|_| dir.clone());
        (
            collect_pack_files(&canon),
            serde_json::json!(dir.to_string_lossy()),
            false,
        )
    } else {
        (Vec::new(), serde_json::Value::Null, !is_virtual)
    };

    Ok(Json(serde_json::json!({
        "id": manifest.id,
        "manifest": manifest,
        "path": path_val,
        "files": files,
        "virtual": is_virtual,
        "compiled_preset": compiled_preset,
    })))
}

/// GET /api/v1/sandbox/policies/{id}
pub async fn get_policy(
    Extension(state): Extension<Arc<SandboxState>>,
    UrlPath(id): UrlPath<String>,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<serde_json::Value>)> {
    if !validate_pack_id(&id) {
        return Err((StatusCode::BAD_REQUEST, Json(serde_json::json!({"error": format!("Invalid policy pack id '{id}'")}))));
    }
    if let Some(manifest) = state.resolve_policy(&id) {
        let dir = state.root.join("policies").join(&id);
        let mut files = Vec::new();
        if dir.is_dir() {
            if let Ok(entries) = std::fs::read_dir(&dir) {
                for e in entries.flatten() {
                    files.push(e.file_name().to_string_lossy().to_string());
                }
            }
        }
        files.sort();
        Ok(Json(serde_json::json!({
            "id": manifest.id,
            "manifest": manifest,
            "path": dir.to_string_lossy(),
            "files": files,
        })))
    } else {
        Err((StatusCode::NOT_FOUND, Json(serde_json::json!({"error": format!("Policy pack '{id}' not found")}))))
    }
}

fn pack_content_type(path: &str) -> &'static str {
    let ext = path.rsplit_once('.').map(|x| x.1).unwrap_or("").to_ascii_lowercase();
    match ext.as_str() {
        "json" => "application/json",
        "xml" => "application/xml",
        "onnx" | "bin" | "stl" => "application/octet-stream",
        "png" => "image/png",
        "jpg" | "jpeg" => "image/jpeg",
        "txt" | "md" => "text/plain; charset=utf-8",
        _ => "application/octet-stream",
    }
}

async fn serve_file_from_dir(allowed_root: &Path, dir: &Path, rel: &str) -> Response {
    let not_found = || (StatusCode::NOT_FOUND, "not found").into_response();
    if !is_safe_relative_path(rel) {
        return not_found();
    }
    let canon_allowed = match allowed_root.canonicalize() {
        Ok(r) => r,
        Err(_) => return not_found(),
    };
    let root = match dir.canonicalize() {
        Ok(r) => r,
        Err(_) => return not_found(),
    };
    if !root.starts_with(&canon_allowed) {
        return not_found();
    }
    let file = match root.join(rel).canonicalize() {
        Ok(f) => f,
        Err(_) => return not_found(),
    };
    if !file.starts_with(&root) || !file.is_file() {
        return not_found();
    }
    let ct = pack_content_type(rel);
    match tokio::fs::read(&file).await {
        Ok(bytes) => (
            [(header::CONTENT_TYPE, ct), (header::CACHE_CONTROL, "no-cache")],
            Body::from(bytes),
        ).into_response(),
        Err(_) => not_found(),
    }
}

/// GET /api/v1/sandbox/scenes/{id}/files/{*path}
pub async fn serve_scene_file(
    Extension(state): Extension<Arc<SandboxState>>,
    UrlPath((id, rel)): UrlPath<(String, String)>,
) -> Response {
    if !validate_pack_id(&id) {
        return (StatusCode::NOT_FOUND, "scene pack or file not found").into_response();
    }
    if let Some(conflict_dir) = state.user_scene_conflict(&id) {
        return (
            StatusCode::CONFLICT,
            Json(serde_json::json!({
                "error": format!("User scene directory '{}' conflicts with reserved builtin scene '{}'", conflict_dir.display(), id),
                "code": "BUILTIN_ID_CONFLICT",
            })),
        )
            .into_response();
    }
    if !is_reserved_builtin_id("scene", &id) {
        let scenes_root = state.root.join("scenes");
        let dir = scenes_root.join(&id);
        if dir.is_dir() {
            return serve_file_from_dir(&scenes_root, &dir, &rel).await;
        }
        return (StatusCode::NOT_FOUND, "scene pack or file not found").into_response();
    }
    if id != "default" {
        if let (_, Some(p)) =
            find_valid_builtin_scene_dir(&id, &state.effective_builtin_roots())
        {
            return serve_file_from_dir(&p.root, &p.canon_dir, &rel).await;
        }
    }
    (StatusCode::NOT_FOUND, "scene pack or file not found").into_response()
}

/// GET /api/v1/sandbox/tools/{id}/files/{*path}
pub async fn serve_tool_file(
    Extension(state): Extension<Arc<SandboxState>>,
    UrlPath((id, rel)): UrlPath<(String, String)>,
) -> Response {
    if !validate_pack_id(&id) {
        return (StatusCode::NOT_FOUND, "tool pack or file not found").into_response();
    }
    if let Some(conflict_dir) = state.user_tool_conflict(&id) {
        return (
            StatusCode::CONFLICT,
            Json(serde_json::json!({
                "error": format!("User tool directory '{}' conflicts with reserved builtin tool '{}'", conflict_dir.display(), id),
                "code": "BUILTIN_ID_CONFLICT",
            })),
        )
            .into_response();
    }
    if !is_reserved_builtin_id("tool", &id) {
        let tools_root = state.root.join("tools");
        let dir = tools_root.join(&id);
        if dir.is_dir() {
            return serve_file_from_dir(&tools_root, &dir, &rel).await;
        }
        return (StatusCode::NOT_FOUND, "tool pack or file not found").into_response();
    }
    if id != "default" {
        if let (_, Some(p)) =
            find_valid_builtin_tool_dir(&id, &state.effective_builtin_roots())
        {
            return serve_file_from_dir(&p.root, &p.canon_dir, &rel).await;
        }
    }
    (StatusCode::NOT_FOUND, "tool pack or file not found").into_response()
}

/// GET /api/v1/sandbox/policies/{id}/files/{*path}
pub async fn serve_policy_file(
    Extension(state): Extension<Arc<SandboxState>>,
    UrlPath((id, rel)): UrlPath<(String, String)>,
) -> Response {
    if !validate_pack_id(&id) {
        return (StatusCode::NOT_FOUND, "policy pack or file not found").into_response();
    }
    let policies_root = state.root.join("policies");
    let dir = policies_root.join(&id);
    serve_file_from_dir(&policies_root, &dir, &rel).await
}

fn base_assets_dir() -> Option<PathBuf> {
    assets::asset_dir()
        .or_else(|| Path::new("assets").is_dir().then(|| PathBuf::from("assets")))
        .or_else(|| {
            Path::new("/usr/share/rebot-teleop/assets")
                .is_dir()
                .then(|| PathBuf::from("/usr/share/rebot-teleop/assets"))
        })
}

fn is_safe_base_asset_ext(rel: &str) -> bool {
    let ext = rel.rsplit_once('.').map(|x| x.1.to_ascii_lowercase());
    matches!(
        ext.as_deref(),
        Some("stl" | "obj" | "ply" | "dae" | "png" | "jpg" | "jpeg" | "bin")
    )
}

/// GET /api/v1/sandbox/base-assets/{*path}
pub async fn serve_base_asset(UrlPath(rel): UrlPath<String>) -> Response {
    let not_found = || (StatusCode::NOT_FOUND, "not found").into_response();
    if !is_safe_relative_path(&rel) || !is_safe_base_asset_ext(&rel) {
        return not_found();
    }
    let Some(dir) = base_assets_dir() else {
        return not_found();
    };
    let Ok(canon_root) = dir.canonicalize() else {
        return not_found();
    };
    let Ok(canon_file) = canon_root.join(&rel).canonicalize() else {
        return not_found();
    };
    if !canon_file.starts_with(&canon_root) || !canon_file.is_file() {
        return not_found();
    }
    match tokio::fs::metadata(&canon_file).await {
        Ok(m) if m.len() <= MAX_PACK_FILE_BYTES => {}
        _ => return not_found(),
    }
    let ct = pack_content_type(&rel);
    match tokio::fs::read(&canon_file).await {
        Ok(bytes) => (
            [(header::CONTENT_TYPE, ct), (header::CACHE_CONTROL, "no-cache")],
            Body::from(bytes),
        )
            .into_response(),
        Err(_) => not_found(),
    }
}

// ---------------------------------------------------------------------------
// Activation, State, Reset, IO, Telemetry
// ---------------------------------------------------------------------------

async fn running_robot_id(supervisor: &crate::supervisor::ArmSupervisor) -> String {
    let inner = supervisor.inner().read().await;
    if let Some(ref pid) = inner.config.profile_id {
        if let Some(norm) = normalize_robot_id(pid) {
            return norm;
        }
    }
    if let Some(ref follower) = inner.follower {
        if !matches!(follower.as_str(), "mock" | "sim") {
            if let Some(norm) = normalize_robot_id(follower) {
                return norm;
            }
        }
    }
    "b601dm".to_string()
}

fn robot_incompatible_error(
    message: String,
    running_robot: &str,
    requested_robot: Option<&str>,
    allowed_robots: &[String],
) -> (StatusCode, Json<serde_json::Value>) {
    (
        StatusCode::BAD_REQUEST,
        Json(serde_json::json!({
            "error": message,
            "code": "ROBOT_INCOMPATIBLE",
            "running_robot": running_robot,
            "requested_robot": requested_robot,
            "allowed_robots": allowed_robots,
        })),
    )
}

fn resolve_requested_robot(
    robot: Option<&str>,
    profile: Option<&str>,
    profile_id: Option<&str>,
) -> Result<Option<String>, (StatusCode, Json<serde_json::Value>)> {
    let mut resolved: Option<String> = None;
    for raw in [robot, profile, profile_id].into_iter().flatten() {
        let norm = normalize_robot_id(raw).ok_or_else(|| {
            (
                StatusCode::BAD_REQUEST,
                Json(serde_json::json!({
                    "error": format!("Invalid or empty robot/profile '{raw}'"),
                    "code": "ROBOT_INCOMPATIBLE",
                })),
            )
        })?;
        if let Some(ref prev) = resolved {
            if prev != &norm {
                return Err((
                    StatusCode::BAD_REQUEST,
                    Json(serde_json::json!({
                        "error": format!("Conflicting robot/profile parameters '{prev}' and '{norm}'"),
                        "code": "ROBOT_INCOMPATIBLE",
                    })),
                ));
            }
        } else {
            resolved = Some(norm);
        }
    }
    Ok(resolved)
}

fn ensure_requested_robot_matches(
    running_robot: &str,
    requested_robot: Option<&str>,
) -> Result<(), (StatusCode, Json<serde_json::Value>)> {
    if let Some(req) = requested_robot {
        if req != running_robot {
            return Err(robot_incompatible_error(
                format!(
                    "Requested robot/profile '{req}' does not match running armd robot '{running_robot}'"
                ),
                running_robot,
                Some(req),
                &[],
            ));
        }
    }
    Ok(())
}

fn validate_policy_robot_compatibility(
    running_robot: &str,
    policy_manifest: &PolicyManifest,
) -> Result<(), (StatusCode, Json<serde_json::Value>)> {
    let raw = policy_manifest.robot.trim();
    if raw.is_empty() {
        return Ok(());
    }
    let pol_robot = normalize_robot_id(raw);
    if pol_robot.as_deref() != Some(running_robot) {
        return Err(robot_incompatible_error(
            format!(
                "Policy '{}' is for robot '{}', which is incompatible with running robot '{}'",
                policy_manifest.id, policy_manifest.robot, running_robot
            ),
            running_robot,
            None,
            std::slice::from_ref(&policy_manifest.robot),
        ));
    }
    Ok(())
}

#[derive(Debug, Clone)]
struct BaseMeshInspection {
    meshdir: Option<String>,
    base_assets: Vec<serde_json::Value>,
    missing_base_assets: Vec<String>,
    note: Option<String>,
}

fn is_safe_base_mesh_file(file: &str) -> bool {
    if !is_safe_relative_path(file) {
        return false;
    }
    Path::new(file)
        .components()
        .all(|c| matches!(c, std::path::Component::Normal(_)))
}

struct BaseMeshCandidate {
    file: String,
    mjcf_path: String,
    candidate_rel: Option<String>,
    candidate_path: Option<PathBuf>,
    canon_file: Option<PathBuf>,
}

fn resolve_base_mesh_candidates(base_mjcf_xml: &str) -> (Option<String>, Vec<BaseMeshCandidate>) {
    let Ok(doc) = roxmltree::Document::parse(base_mjcf_xml) else {
        return (None, Vec::new());
    };
    let meshdir = doc
        .descendants()
        .find(|n| n.is_element() && n.tag_name().name() == "compiler")
        .and_then(|n| n.attribute("meshdir"))
        .map(|s| s.to_string());
    let canon_asset_root = base_assets_dir().and_then(|d| d.canonicalize().ok());
    let meshdir_leaf = meshdir
        .as_deref()
        .and_then(|m| Path::new(m).file_name())
        .and_then(|n| n.to_str())
        .filter(|leaf| is_safe_relative_path(leaf))
        .map(|s| s.to_string());

    let mut candidates = Vec::new();
    for node in doc.descendants() {
        if node.is_element() && node.tag_name().name() == "mesh" {
            if let Some(file) = node.attribute("file").map(str::trim).filter(|s| !s.is_empty()) {
                let mjcf_path = match meshdir.as_deref() {
                    Some(md) if !md.is_empty() => {
                        format!("{}/{}", md.trim_end_matches('/'), file)
                    }
                    _ => file.to_string(),
                };
                let (candidate_rel, candidate_path, canon_file) = if is_safe_base_mesh_file(file) {
                    let rel = match meshdir_leaf.as_deref() {
                        Some(leaf) => format!("{leaf}/{file}"),
                        None => file.to_string(),
                    };
                    if is_safe_relative_path(&rel) && is_safe_base_asset_ext(&rel) {
                        if let Some(ref root) = canon_asset_root {
                            let cand = root.join(&rel);
                            let canon = cand
                                .canonicalize()
                                .ok()
                                .filter(|f| f.starts_with(root) && f.is_file());
                            (Some(rel), Some(cand), canon)
                        } else {
                            (Some(rel), None, None)
                        }
                    } else {
                        (Some(rel), None, None)
                    }
                } else {
                    (None, None, None)
                };
                candidates.push(BaseMeshCandidate {
                    file: file.to_string(),
                    mjcf_path,
                    candidate_rel,
                    candidate_path,
                    canon_file,
                });
            }
        }
    }
    (meshdir, candidates)
}

fn collect_base_mesh_watch_paths(base_mjcf_xml: &str) -> Vec<PathBuf> {
    let (_, candidates) = resolve_base_mesh_candidates(base_mjcf_xml);
    let mut seen = HashSet::new();
    let mut paths = Vec::new();
    for c in candidates {
        if let Some(cand) = c.candidate_path {
            if seen.insert(cand.clone()) {
                paths.push(cand);
            }
        }
        if let Some(canon) = c.canon_file {
            if seen.insert(canon.clone()) {
                paths.push(canon);
            }
        }
    }
    paths
}

fn inspect_base_mesh_assets(base_mjcf_xml: &str) -> BaseMeshInspection {
    let (meshdir, candidates) = resolve_base_mesh_candidates(base_mjcf_xml);
    if meshdir.is_none() && candidates.is_empty() && roxmltree::Document::parse(base_mjcf_xml).is_err() {
        return BaseMeshInspection {
            meshdir: None,
            base_assets: Vec::new(),
            missing_base_assets: Vec::new(),
            note: None,
        };
    }

    let mut base_assets = Vec::new();
    let mut missing_base_assets = Vec::new();

    for cand in candidates {
        let file_meta = cand
            .canon_file
            .as_deref()
            .and_then(|f| std::fs::read(f).ok())
            .map(|bytes| (sha256_hex(&bytes), bytes.len() as u64));
        let (local_exists, sha256, size_bytes) = match file_meta {
            Some((digest, len)) => (true, Some(digest), Some(len)),
            None => (false, None, None),
        };

        let resolved_url = if local_exists {
            cand.candidate_rel
                .as_ref()
                .map(|rel| format!("/api/v1/sandbox/base-assets/{rel}"))
        } else {
            None
        };

        let http_accessible = resolved_url.is_some();
        if !http_accessible || !local_exists {
            missing_base_assets.push(cand.mjcf_path.clone());
        }

        base_assets.push(serde_json::json!({
            "file": cand.file,
            "meshdir": meshdir,
            "mjcf_path": cand.mjcf_path,
            "memfs_path": cand.mjcf_path,
            "path": cand.candidate_rel,
            "local_exists": local_exists,
            "http_accessible": http_accessible,
            "url": resolved_url,
            "sha256": sha256,
            "size_bytes": size_bytes,
        }));
    }

    let note = if !missing_base_assets.is_empty() {
        Some(
            "Base MJCF references mesh files that are missing or unsafe inside the base assets directory; browser MuJoCo WASM cannot fetch missing base meshes."
                .to_string(),
        )
    } else {
        None
    };

    BaseMeshInspection {
        meshdir,
        base_assets,
        missing_base_assets,
        note,
    }
}

fn validate_robot_compatibility(
    running_robot: &str,
    requested_robot: Option<&str>,
    scene_manifest: &SceneManifest,
    tool_manifest: &ToolManifest,
) -> Result<(), (StatusCode, Json<serde_json::Value>)> {
    ensure_requested_robot_matches(running_robot, requested_robot)?;
    if !scene_manifest.robots.is_empty() || builtin_scene_spec(&scene_manifest.id).is_some() {
        let scene_ok = scene_manifest
            .robots
            .iter()
            .filter_map(|r| normalize_robot_id(r))
            .any(|r| r == running_robot);
        if !scene_ok {
            return Err(robot_incompatible_error(
                format!(
                    "Scene '{}' only supports robots {:?}, which is incompatible with running robot '{}'",
                    scene_manifest.id, scene_manifest.robots, running_robot
                ),
                running_robot,
                requested_robot,
                &scene_manifest.robots,
            ));
        }
    }
    if !tool_manifest.robots.is_empty() || builtin_tool_spec(&tool_manifest.id).is_some() {
        let tool_ok = tool_manifest
            .robots
            .iter()
            .filter_map(|r| normalize_robot_id(r))
            .any(|r| r == running_robot);
        if !tool_ok {
            return Err(robot_incompatible_error(
                format!(
                    "Tool '{}' only supports robots {:?}, which is incompatible with running robot '{}'",
                    tool_manifest.id, tool_manifest.robots, running_robot
                ),
                running_robot,
                requested_robot,
                &tool_manifest.robots,
            ));
        }
    }
    Ok(())
}

fn deserialize_double_option<'de, T, D>(deserializer: D) -> Result<Option<Option<T>>, D::Error>
where
    T: Deserialize<'de>,
    D: serde::Deserializer<'de>,
{
    Deserialize::deserialize(deserializer).map(Some)
}

#[derive(Debug, Deserialize)]
pub struct ActivateRequest {
    pub mode: String, // "none" | "wasm" | "sim"
    #[serde(default)]
    pub scene: Option<String>,
    #[serde(default)]
    pub tool: Option<String>,
    #[serde(default, deserialize_with = "deserialize_double_option")]
    pub policy: Option<Option<String>>,
    #[serde(default)]
    pub seed: Option<u64>,
    #[serde(default)]
    pub auto_start_policy: Option<bool>,
    #[serde(default)]
    pub robot: Option<String>,
    #[serde(default)]
    pub profile: Option<String>,
    #[serde(default)]
    pub profile_id: Option<String>,
}

#[derive(Debug, Deserialize)]
pub struct RecoverSplitRequest {
    pub action: String, // "replay_camera" | "rollback_physics"
}

enum SimTransportError {
    ConnectOrIo(String),
    Timeout,
}

fn tx_json_err(
    status: StatusCode,
    code: &'static str,
    tx_id: &str,
    msg: impl Into<String>,
) -> (StatusCode, Json<serde_json::Value>) {
    (
        status,
        Json(serde_json::json!({
            "error": msg.into(),
            "code": code,
            "tx_id": tx_id,
        })),
    )
}

fn code_json_err(
    status: StatusCode,
    code: &'static str,
    msg: impl Into<String>,
) -> (StatusCode, Json<serde_json::Value>) {
    (
        status,
        Json(serde_json::json!({
            "error": msg.into(),
            "code": code,
        })),
    )
}

fn extract_remote_tx_and_state(val: &serde_json::Value) -> (Option<&str>, &str) {
    let remote_tx = val
        .get("tx_id")
        .and_then(|v| v.as_str())
        .map(str::trim)
        .filter(|s| !s.is_empty());
    let remote_state = val
        .get("activation_state")
        .or_else(|| val.get("state"))
        .and_then(|v| v.as_str())
        .unwrap_or("");
    (remote_tx, remote_state)
}

async fn abort_local_tx(state: &SandboxState, tx_id: &str) {
    let mut tx_w = state.tx_state.write().await;
    *tx_w = ActivationTxState::Aborted {
        tx_id: tx_id.to_string(),
    };
}

async fn sim_tx_call(
    upstream: &DetectorUpstream,
    method: Method,
    path: &str,
    body: Option<&[u8]>,
    timeout: Duration,
) -> Result<proxy::UpstreamResponse, SimTransportError> {
    let req_fut = proxy::upstream_request(upstream, method, path, body);
    match tokio::time::timeout(timeout, req_fut).await {
        Ok(Ok(resp)) => Ok(resp),
        Ok(Err(e)) => Err(SimTransportError::ConnectOrIo(e)),
        Err(_elapsed) => Err(SimTransportError::Timeout),
    }
}

async fn sim_best_effort_abort(upstream: &DetectorUpstream, tx_id: &str, timeout: Duration) {
    let abort_body = serde_json::to_vec(&serde_json::json!({ "tx_id": tx_id })).unwrap_or_default();
    let _ = sim_tx_call(upstream, Method::POST, "/sim/scene/abort", Some(&abort_body), timeout).await;
}

fn check_robot_field_matches(
    robot_val: &serde_json::Value,
    running_robot: &str,
) -> Result<Option<String>, (StatusCode, Json<serde_json::Value>)> {
    if robot_val.is_null() {
        return Ok(None);
    }
    let upstream_raw = robot_val.as_str().ok_or_else(|| {
        robot_incompatible_error(
            format!("rebot-sim returned non-string robot field {robot_val}"),
            running_robot,
            None,
            std::slice::from_ref(&running_robot.to_string()),
        )
    })?;
    if !upstream_raw.trim().is_empty()
        && normalize_robot_id(upstream_raw).as_deref() != Some(running_robot)
    {
        return Err(robot_incompatible_error(
            format!(
                "rebot-sim upstream reported robot '{upstream_raw}', incompatible with running armd robot '{running_robot}'"
            ),
            running_robot,
            Some(upstream_raw),
            std::slice::from_ref(&running_robot.to_string()),
        ));
    }
    Ok(Some(upstream_raw.to_string()))
}

fn check_required_upstream_robot_match(
    val: &serde_json::Value,
    running_robot: &str,
    missing_code: &'static str,
    tx_id: &str,
    context_label: &str,
) -> Result<(), (StatusCode, Json<serde_json::Value>)> {
    let Some(robot_val) = val.get("robot").filter(|v| !v.is_null()) else {
        return Err(tx_json_err(
            StatusCode::BAD_GATEWAY,
            missing_code,
            tx_id,
            format!("rebot-sim {context_label} ACK missing non-empty 'robot' field"),
        ));
    };
    match check_robot_field_matches(robot_val, running_robot)? {
        Some(raw) if !raw.trim().is_empty() => Ok(()),
        _ => Err(tx_json_err(
            StatusCode::BAD_GATEWAY,
            missing_code,
            tx_id,
            format!("rebot-sim {context_label} ACK returned empty 'robot' field"),
        )),
    }
}

fn parse_and_check_tx_ack(
    body: &[u8],
    expected_state: &str,
    code: &'static str,
    expected_tx_id: &str,
    context_label: &str,
) -> Result<serde_json::Value, (StatusCode, Json<serde_json::Value>)> {
    let val = serde_json::from_slice::<serde_json::Value>(body).map_err(|e| {
        tx_json_err(
            StatusCode::BAD_GATEWAY,
            code,
            expected_tx_id,
            format!("rebot-sim {context_label} returned invalid JSON: {e}"),
        )
    })?;
    let state_str = val
        .get("state")
        .or_else(|| val.get("activation_state"))
        .and_then(|v| v.as_str())
        .unwrap_or("");
    if !state_str.eq_ignore_ascii_case(expected_state) {
        return Err(tx_json_err(
            StatusCode::BAD_GATEWAY,
            code,
            expected_tx_id,
            format!("rebot-sim {context_label} ACK missing state={expected_state} (got {state_str:?})"),
        ));
    }
    let resp_tx = val.get("tx_id").and_then(|v| v.as_str()).unwrap_or("");
    if resp_tx != expected_tx_id {
        return Err(tx_json_err(
            StatusCode::BAD_GATEWAY,
            code,
            expected_tx_id,
            format!("rebot-sim {context_label} ACK tx_id mismatch: expected '{expected_tx_id}', got '{resp_tx}'"),
        ));
    }
    Ok(val)
}

fn verify_prepare_ack(
    body: &[u8],
    expected_tx_id: &str,
    running_robot: &str,
) -> Result<(), (StatusCode, Json<serde_json::Value>)> {
    let val = parse_and_check_tx_ack(
        body,
        "PREPARED",
        "SIM_INVALID_PREPARE_ACK",
        expected_tx_id,
        "prepare",
    )?;
    check_required_upstream_robot_match(
        &val,
        running_robot,
        "SIM_INVALID_PREPARE_ACK",
        expected_tx_id,
        "prepare",
    )
}

fn check_upstream_robot_match(
    resp_bytes: &[u8],
    running_robot: &str,
) -> Result<(), (StatusCode, Json<serde_json::Value>)> {
    let Ok(resp_val) = serde_json::from_slice::<serde_json::Value>(resp_bytes) else {
        return Ok(());
    };
    if let Some(robot_val) = resp_val.get("robot") {
        check_robot_field_matches(robot_val, running_robot)?;
    }
    Ok(())
}

fn is_degraded_response(body: &[u8]) -> bool {
    let Ok(val) = serde_json::from_slice::<serde_json::Value>(body) else {
        return false;
    };
    let code = val.get("code").and_then(|v| v.as_str()).unwrap_or("");
    let (_, state) = extract_remote_tx_and_state(&val);
    code == "SIM_ACTIVATION_DEGRADED" || state.eq_ignore_ascii_case("DEGRADED_SPLIT")
}

fn extract_remote_tx_id(body: &[u8]) -> Option<String> {
    serde_json::from_slice::<serde_json::Value>(body)
        .ok()
        .and_then(|v| {
            v.get("tx_id")
                .or_else(|| v.get("degraded_tx_id"))
                .and_then(|t| t.as_str())
                .map(str::trim)
                .filter(|s| !s.is_empty())
                .map(str::to_string)
        })
}

fn verify_ack_digest_field(
    val: &serde_json::Value,
    field: &str,
    expected: Option<&str>,
    expected_tx_id: &str,
) -> Result<(), (StatusCode, Json<serde_json::Value>)> {
    match expected {
        Some(exp) => {
            let actual = val
                .get(field)
                .filter(|v| !v.is_null())
                .and_then(|v| v.as_str())
                .map(str::trim)
                .filter(|s| !s.is_empty());
            match actual {
                Some(got) if got == exp => Ok(()),
                Some(got) => Err(tx_json_err(
                    StatusCode::BAD_GATEWAY,
                    "SIM_INVALID_COMMIT_ACK",
                    expected_tx_id,
                    format!("rebot-sim commit ACK {field} mismatch: expected '{exp}', got '{got}'"),
                )),
                None => Err(tx_json_err(
                    StatusCode::BAD_GATEWAY,
                    "SIM_INVALID_COMMIT_ACK",
                    expected_tx_id,
                    format!("rebot-sim commit ACK missing required {field} '{exp}'"),
                )),
            }
        }
        None => {
            if let Some(d_val) = val.get(field).filter(|v| !v.is_null()) {
                return Err(tx_json_err(
                    StatusCode::BAD_GATEWAY,
                    "SIM_INVALID_COMMIT_ACK",
                    expected_tx_id,
                    format!("rebot-sim commit ACK unexpected non-null {field}: {d_val}"),
                ));
            }
            Ok(())
        }
    }
}

fn verify_commit_ack(
    body: &[u8],
    expected_tx_id: &str,
    running_robot: Option<&str>,
    expected_scene_digest: Option<&str>,
    expected_tool_digest: Option<&str>,
) -> Result<(), (StatusCode, Json<serde_json::Value>)> {
    let val = parse_and_check_tx_ack(
        body,
        "COMMITTED",
        "SIM_INVALID_COMMIT_ACK",
        expected_tx_id,
        "commit",
    )?;
    if let Some(robot) = running_robot {
        check_required_upstream_robot_match(
            &val,
            robot,
            "SIM_INVALID_COMMIT_ACK",
            expected_tx_id,
            "commit",
        )?;
    }
    verify_ack_digest_field(&val, "scene_digest", expected_scene_digest, expected_tx_id)?;
    verify_ack_digest_field(&val, "tool_digest", expected_tool_digest, expected_tx_id)?;
    Ok(())
}

fn verify_rollback_ack(
    body: &[u8],
    expected_tx_id: &str,
    expected_prev_tx_id: Option<&str>,
) -> Result<Option<String>, String> {
    let val = serde_json::from_slice::<serde_json::Value>(body)
        .map_err(|e| format!("rollback_physics returned invalid JSON: {e}"))?;
    let state_str = val
        .get("state")
        .or_else(|| val.get("activation_state"))
        .and_then(|v| v.as_str())
        .unwrap_or("");
    if !state_str.eq_ignore_ascii_case("ABORTED") {
        return Err(format!(
            "rollback_physics expected state=ABORTED, got {state_str:?}"
        ));
    }
    let resp_tx = val.get("tx_id").and_then(|v| v.as_str()).unwrap_or("");
    if resp_tx != expected_tx_id {
        return Err(format!(
            "rollback_physics tx_id mismatch: expected '{expected_tx_id}', got '{resp_tx}'"
        ));
    }
    let restored_tx = val
        .get("restored_tx_id")
        .and_then(|v| v.as_str())
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .map(str::to_string);
    if let Some(expected_prev) = expected_prev_tx_id {
        if restored_tx.as_deref() != Some(expected_prev) {
            return Err(format!(
                "rollback_physics restored_tx_id mismatch: expected {expected_prev:?}, got {restored_tx:?}"
            ));
        }
    }
    Ok(restored_tx)
}

fn degraded_split_http_error(tx_id: &str, detail: &str) -> (StatusCode, Json<serde_json::Value>) {
    (
        StatusCode::SERVICE_UNAVAILABLE,
        Json(serde_json::json!({
            "error": detail,
            "code": "SIM_ACTIVATION_DEGRADED",
            "state": "DEGRADED_SPLIT",
            "tx_id": tx_id,
        })),
    )
}

#[derive(Clone)]
struct ActivationBundleSnapshot {
    robot: String,
    scene_id: String,
    scene_manifest: SceneManifest,
    scene_digest: Option<String>,
    scene_files: Option<BTreeMap<String, Vec<u8>>>,
    tool_id: String,
    tool_manifest: ToolManifest,
    tool_digest: Option<String>,
    tool_files: Option<BTreeMap<String, Vec<u8>>>,
    policy_id: Option<String>,
    policy_manifest: Option<PolicyManifest>,
    base_xml: String,
    composed_xml: String,
    pre_fp: PackStatFingerprint,
}

fn build_cached_bundle_from_snapshot(
    mut snap: ActivationBundleSnapshot,
) -> Result<Arc<CachedBundle>, (StatusCode, Json<serde_json::Value>)> {
    snap.scene_manifest.assets = normalize_manifest_assets(
        snap.scene_files.as_ref(),
        Some(&snap.scene_manifest.world_xml),
        snap.scene_manifest.assets,
    );
    snap.tool_manifest.assets = normalize_manifest_assets(
        snap.tool_files.as_ref(),
        Some(&snap.tool_manifest.tool_xml),
        snap.tool_manifest.assets,
    );
    let composed_xml_sha256 = sha256_hex(snap.composed_xml.as_bytes());
    let mesh_info = inspect_base_mesh_assets(&snap.base_xml);
    if !snap.pre_fp.matches_disk() {
        return Err((
            StatusCode::CONFLICT,
            Json(serde_json::json!({
                "error": "Pack files modified on disk during bundle build",
                "code": "INVALID_PACK",
            })),
        ));
    }
    let policy_model_url = snap
        .policy_manifest
        .as_ref()
        .map(|pm| format!("/api/v1/sandbox/policies/{}/files/{}", pm.id, pm.model_file));

    Ok(Arc::new(CachedBundle {
        cache_id: NEXT_BUNDLE_CACHE_ID.fetch_add(1, Ordering::Relaxed),
        robot: snap.robot,
        active_scene: snap.scene_id,
        active_tool: snap.tool_id,
        active_policy: snap.policy_id,
        scene_digest: snap.scene_digest,
        tool_digest: snap.tool_digest,
        composed_xml: snap.composed_xml,
        composed_xml_sha256,
        scene_manifest: snap.scene_manifest,
        tool_manifest: snap.tool_manifest,
        policy_manifest: snap.policy_manifest,
        policy_model_url,
        base_meshdir: mesh_info.meshdir,
        base_assets: mesh_info.base_assets,
        missing_base_assets: mesh_info.missing_base_assets,
        base_assets_note: mesh_info.note,
        stat_fingerprint: snap.pre_fp,
    }))
}

fn build_bundle_from_disk(
    state: &SandboxState,
    running_robot: &str,
    requested_robot: Option<&str>,
    current: &ActiveState,
) -> Result<Arc<CachedBundle>, (StatusCode, Json<serde_json::Value>)> {
    struct RebuildConcurrencyGuard<'a> {
        in_flight: &'a AtomicUsize,
    }
    impl Drop for RebuildConcurrencyGuard<'_> {
        fn drop(&mut self) {
            self.in_flight.fetch_sub(1, Ordering::SeqCst);
        }
    }
    let active_now = state.in_flight_rebuilds.fetch_add(1, Ordering::SeqCst) + 1;
    let _concurrency_guard = RebuildConcurrencyGuard {
        in_flight: &state.in_flight_rebuilds,
    };
    state
        .peak_concurrent_rebuilds
        .fetch_max(active_now, Ordering::SeqCst);
    let resolved_base = resolve_base_robot_xml_for_robot(running_robot)?;
    let base_xml = resolved_base.xml;
    let pre_fp = state.collect_pack_pre_fingerprint(
        &current.active_scene,
        &current.active_tool,
        current.active_policy.as_deref(),
        &base_xml,
        &resolved_base.watch_paths,
    );
    let delay_ms = state.bundle_rebuild_delay_ms.load(Ordering::Relaxed);
    if delay_ms > 0 {
        std::thread::sleep(Duration::from_millis(delay_ms));
    }

    let ResolvedPack {
        canon_dir: _scene_dir,
        manifest: scene_manifest,
        xml: sw,
        digest: scene_digest,
        files: scene_files,
    } = state.resolve_scene_checked_full(&current.active_scene)?;
    let ResolvedPack {
        canon_dir: _tool_dir,
        manifest: tool_manifest,
        xml: tw,
        digest: tool_digest,
        files: tool_files,
    } = state.resolve_tool_checked_full(&current.active_tool)?;

    validate_robot_compatibility(running_robot, requested_robot, &scene_manifest, &tool_manifest)?;

    if let Some(ref expected) = current.committed_scene_digest {
        if scene_digest.as_deref() != Some(expected.as_str()) {
            return Err((
                StatusCode::BAD_REQUEST,
                Json(serde_json::json!({
                    "error": "Active scene pack digest changed on disk after activation",
                    "code": "INVALID_PACK",
                })),
            ));
        }
    }
    if let Some(ref expected) = current.committed_tool_digest {
        if tool_digest.as_deref() != Some(expected.as_str()) {
            return Err((
                StatusCode::BAD_REQUEST,
                Json(serde_json::json!({
                    "error": "Active tool pack digest changed on disk after activation",
                    "code": "INVALID_PACK",
                })),
            ));
        }
    }

    let policy_manifest = if let Some(ref p) = current.active_policy {
        let pm = state.resolve_policy_checked(p, StatusCode::CONFLICT, "ACTIVE_POLICY_MISSING")?;
        validate_policy_robot_compatibility(running_robot, &pm)?;
        Some(pm)
    } else {
        None
    };

    let composed_xml = compose_mjcf_checked(&base_xml, Some(&sw), Some(&tool_manifest), Some(&tw))
        .map_err(|e| PackFileError::from(e).into_http_error("INVALID_PACK", "MJCF composition failed"))?;

    build_cached_bundle_from_snapshot(ActivationBundleSnapshot {
        robot: running_robot.to_string(),
        scene_id: current.active_scene.clone(),
        scene_manifest,
        scene_digest,
        scene_files,
        tool_id: current.active_tool.clone(),
        tool_manifest,
        tool_digest,
        tool_files,
        policy_id: current.active_policy.clone(),
        policy_manifest,
        base_xml,
        composed_xml,
        pre_fp,
    })
}

fn render_bundle_json(cached: &CachedBundle, current: &ActiveState) -> serde_json::Value {
    serde_json::json!({
        "mode": current.mode,
        "robot": cached.robot,
        "active_scene": current.active_scene,
        "active_tool": current.active_tool,
        "active_policy": current.active_policy,
        "auto_start_policy": current.auto_start_policy,
        "seed": current.seed,
        "reset_seq": current.reset_seq,
        "io_seq": current.io_seq,
        "scene_digest": cached.scene_digest,
        "tool_digest": cached.tool_digest,
        "composed_xml": cached.composed_xml,
        "composed_xml_sha256": cached.composed_xml_sha256,
        "base_meshdir": cached.base_meshdir,
        "base_assets": cached.base_assets,
        "missing_base_assets": cached.missing_base_assets,
        "base_assets_note": cached.base_assets_note,
        "scene_manifest": cached.scene_manifest,
        "tool_manifest": cached.tool_manifest,
        "policy_manifest": cached.policy_manifest,
        "policy_model_url": cached.policy_model_url,
        "desired_io": current.desired_io,
    })
}

#[derive(Clone)]
struct TxSnapshot {
    tx_id: String,
    prev_tx_id: Option<String>,
    prev_scene: String,
    prev_tool: String,
    prev_policy: Option<String>,
    prev_mode: String,
    prev_seed: u64,
    prev_auto_start_policy: bool,
    prev_cached_mjcf: String,
    prev_scene_digest: Option<String>,
    prev_tool_digest: Option<String>,
    target_scene: String,
    target_tool: String,
    target_policy: Option<String>,
    target_seed: u64,
    target_auto_start_policy: bool,
    target_composed_mjcf: String,
    target_scene_digest: Option<String>,
    target_tool_digest: Option<String>,
    bundle_snapshot: Option<ActivationBundleSnapshot>,
}

impl TxSnapshot {
    fn into_degraded(self, owner_tx_id: String, target_matches_tx: bool, detail: String) -> ActivationTxState {
        ActivationTxState::DegradedSplit {
            tx_id: owner_tx_id,
            prev_tx_id: self.prev_tx_id,
            prev_scene: self.prev_scene,
            prev_tool: self.prev_tool,
            prev_policy: self.prev_policy,
            prev_mode: self.prev_mode,
            prev_seed: self.prev_seed,
            prev_auto_start_policy: self.prev_auto_start_policy,
            prev_cached_mjcf: self.prev_cached_mjcf,
            prev_scene_digest: self.prev_scene_digest,
            prev_tool_digest: self.prev_tool_digest,
            target_scene: self.target_scene,
            target_tool: self.target_tool,
            target_policy: self.target_policy,
            target_seed: self.target_seed,
            target_auto_start_policy: self.target_auto_start_policy,
            target_composed_mjcf: self.target_composed_mjcf,
            target_scene_digest: self.target_scene_digest,
            target_tool_digest: self.target_tool_digest,
            target_matches_tx,
            detail,
        }
    }

    fn into_unknown(self) -> ActivationTxState {
        ActivationTxState::Unknown {
            tx_id: self.tx_id,
            prev_tx_id: self.prev_tx_id,
            prev_scene: self.prev_scene,
            prev_tool: self.prev_tool,
            prev_policy: self.prev_policy,
            prev_mode: self.prev_mode,
            prev_seed: self.prev_seed,
            prev_auto_start_policy: self.prev_auto_start_policy,
            prev_cached_mjcf: self.prev_cached_mjcf,
            prev_scene_digest: self.prev_scene_digest,
            prev_tool_digest: self.prev_tool_digest,
            target_scene: self.target_scene,
            target_tool: self.target_tool,
            target_policy: self.target_policy,
            target_seed: self.target_seed,
            target_auto_start_policy: self.target_auto_start_policy,
            target_composed_mjcf: self.target_composed_mjcf,
            target_scene_digest: self.target_scene_digest,
            target_tool_digest: self.target_tool_digest,
        }
    }
}

async fn restore_tx_state_after_rollback(
    state: &SandboxState,
    restored_tx_id: Option<String>,
    prev_tx_id: Option<String>,
    prev_scene_digest: Option<String>,
    prev_tool_digest: Option<String>,
    failed_tx_id: &str,
) {
    let mut tx_w = state.tx_state.write().await;
    if let Some(rtx) = restored_tx_id.or(prev_tx_id) {
        *tx_w = ActivationTxState::Committed {
            tx_id: rtx,
            scene_digest: prev_scene_digest,
            tool_digest: prev_tool_digest,
        };
    } else {
        *tx_w = ActivationTxState::Aborted {
            tx_id: failed_tx_id.to_string(),
        };
    }
}

#[allow(clippy::too_many_arguments)]
async fn commit_local_active_state(
    state: &SandboxState,
    mode: &str,
    scene_id: &str,
    tool_id: &str,
    policy_id: Option<String>,
    seed: Option<u64>,
    auto_start_policy: Option<bool>,
    composed: String,
    tx_id: &str,
    scene_digest: Option<String>,
    tool_digest: Option<String>,
    bundle_snapshot: Option<ActivationBundleSnapshot>,
) -> (bool, u64, u64, u64) {
    let (final_auto_start, final_seed, final_reset_seq, final_io_seq) = {
        let mut st_write = state.active.write().await;
        st_write.mode = mode.to_string();
        st_write.active_scene = scene_id.to_string();
        st_write.active_tool = tool_id.to_string();
        st_write.active_policy = policy_id;
        if let Some(s) = seed {
            st_write.seed = s;
        }
        if let Some(asp) = auto_start_policy {
            st_write.auto_start_policy = asp;
        }
        st_write.updated_at_ms = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_millis() as u64)
            .unwrap_or(0);
        st_write.cached_mjcf = composed;
        st_write.committed_scene_digest = scene_digest.clone();
        st_write.committed_tool_digest = tool_digest.clone();

        (
            st_write.auto_start_policy,
            st_write.seed,
            st_write.reset_seq,
            st_write.io_seq,
        )
    };
    {
        let mut tx_write = state.tx_state.write().await;
        *tx_write = ActivationTxState::Committed {
            tx_id: tx_id.to_string(),
            scene_digest,
            tool_digest,
        };
    }
    state.persist().await;
    if let Some(snap) = bundle_snapshot {
        if let Ok(Ok(cached)) =
            tokio::task::spawn_blocking(move || build_cached_bundle_from_snapshot(snap)).await
        {
            state.insert_cached_bundle(cached).await;
        } else {
            state.invalidate_bundle_cache_all().await;
        }
    } else {
        state.invalidate_bundle_cache_all().await;
    }
    (final_auto_start, final_seed, final_reset_seq, final_io_seq)
}

fn resolve_remote_target_locally(
    state: &SandboxState,
    running_robot: &str,
    r_scene: &str,
    r_tool: &str,
    r_sd: Option<String>,
    r_td: Option<String>,
) -> (bool, String, Option<String>, Option<String>) {
    if let (Ok(scene_pack), Ok(tool_pack), Ok(base_xml)) = (
        state.resolve_scene_checked_full(r_scene),
        state.resolve_tool_checked_full(r_tool),
        get_base_robot_xml_for_robot(running_robot),
    ) {
        if scene_pack.digest == r_sd && tool_pack.digest == r_td {
            if let Ok(composed) = compose_mjcf(
                &base_xml,
                Some(&scene_pack.xml),
                Some(&tool_pack.manifest),
                Some(&tool_pack.xml),
            ) {
                if !composed.trim().is_empty() {
                    return (true, composed, scene_pack.digest, tool_pack.digest);
                }
            }
        }
    }
    (false, String::new(), r_sd, r_td)
}

fn foreign_degraded_from_sim_val(
    snap: &TxSnapshot,
    owner_tx: String,
    sim_val: &serde_json::Value,
    detail: String,
) -> ActivationTxState {
    let r_scene = sim_val
        .get("scene")
        .and_then(|v| v.as_str())
        .unwrap_or("")
        .to_string();
    let r_tool = sim_val
        .get("tool")
        .and_then(|v| v.as_str())
        .unwrap_or("")
        .to_string();
    let r_sd = sim_val
        .get("scene_digest")
        .and_then(|v| v.as_str())
        .map(str::to_string);
    let r_td = sim_val
        .get("tool_digest")
        .and_then(|v| v.as_str())
        .map(str::to_string);
    ActivationTxState::DegradedSplit {
        tx_id: owner_tx,
        prev_tx_id: snap.prev_tx_id.clone(),
        prev_scene: snap.prev_scene.clone(),
        prev_tool: snap.prev_tool.clone(),
        prev_policy: snap.prev_policy.clone(),
        prev_mode: snap.prev_mode.clone(),
        prev_seed: snap.prev_seed,
        prev_auto_start_policy: snap.prev_auto_start_policy,
        prev_cached_mjcf: snap.prev_cached_mjcf.clone(),
        prev_scene_digest: snap.prev_scene_digest.clone(),
        prev_tool_digest: snap.prev_tool_digest.clone(),
        target_scene: r_scene,
        target_tool: r_tool,
        target_policy: None,
        target_seed: snap.prev_seed,
        target_auto_start_policy: snap.prev_auto_start_policy,
        target_composed_mjcf: String::new(),
        target_scene_digest: r_sd,
        target_tool_digest: r_td,
        target_matches_tx: false,
        detail,
    }
}

async fn converge_degraded_split(
    state: &SandboxState,
    upstream: &DetectorUpstream,
    running_robot: &str,
    snap: TxSnapshot,
    detail: &str,
) -> Result<(bool, u64, u64, u64), (StatusCode, Json<serde_json::Value>)> {
    if let Some(remote_tx) = extract_remote_tx_id(detail.as_bytes()) {
        if remote_tx != snap.tx_id {
            let sim_val = serde_json::from_str::<serde_json::Value>(detail)
                .unwrap_or(serde_json::Value::Null);
            let msg = format!(
                "Sim activation is in DEGRADED_SPLIT owned by foreign transaction '{remote_tx}' (not '{}'); call /api/v1/sandbox/recover-split first",
                snap.tx_id
            );
            {
                let mut tx_w = state.tx_state.write().await;
                *tx_w = foreign_degraded_from_sim_val(&snap, remote_tx.clone(), &sim_val, msg.clone());
            }
            return Err(degraded_split_http_error(&remote_tx, &msg));
        }
    }
    let split_tx_id = snap.tx_id.clone();
    {
        let mut tx_w = state.tx_state.write().await;
        *tx_w = snap.clone().into_degraded(split_tx_id.clone(), true, detail.to_string());
    }

    let camera_body = serde_json::to_vec(&serde_json::json!({
        "tx_id": split_tx_id,
        "phase": "camera_only",
    }))
    .unwrap_or_default();

    if let Ok(resp) = sim_tx_call(
        upstream,
        Method::POST,
        "/sim/scene/commit",
        Some(&camera_body),
        state.sim_timeouts.commit,
    )
    .await
    {
        if resp.status.is_success()
            && verify_commit_ack(
                &resp.body,
                &split_tx_id,
                Some(running_robot),
                snap.target_scene_digest.as_deref(),
                snap.target_tool_digest.as_deref(),
            )
            .is_ok()
        {
            let committed = commit_local_active_state(
                state,
                "sim",
                &snap.target_scene,
                &snap.target_tool,
                snap.target_policy.clone(),
                Some(snap.target_seed),
                Some(snap.target_auto_start_policy),
                snap.target_composed_mjcf.clone(),
                &split_tx_id,
                snap.target_scene_digest.clone(),
                snap.target_tool_digest.clone(),
                snap.bundle_snapshot.clone(),
            )
            .await;
            return Ok(committed);
        }
    }

    let rollback_body = serde_json::to_vec(&serde_json::json!({
        "tx_id": split_tx_id,
        "phase": "rollback_physics",
    }))
    .unwrap_or_default();

    if let Ok(resp) = sim_tx_call(
        upstream,
        Method::POST,
        "/sim/scene/commit",
        Some(&rollback_body),
        state.sim_timeouts.commit,
    )
    .await
    {
        if resp.status.is_success() {
            if let Ok(restored_tx_id) =
                verify_rollback_ack(&resp.body, &split_tx_id, snap.prev_tx_id.as_deref())
            {
                restore_tx_state_after_rollback(
                    state,
                    restored_tx_id,
                    snap.prev_tx_id,
                    snap.prev_scene_digest,
                    snap.prev_tool_digest,
                    &split_tx_id,
                )
                .await;
                return Err((
                    StatusCode::BAD_GATEWAY,
                    Json(serde_json::json!({
                        "error": "Camera commit failed; physics rolled back cleanly",
                        "code": "SIM_CAMERA_COMMIT_FAILED",
                        "tx_id": split_tx_id,
                    })),
                ));
            }
        }
    }

    Err(degraded_split_http_error(&split_tx_id, detail))
}

#[allow(clippy::too_many_arguments)]
fn activate_committed_response(
    mode: &str,
    running_robot: &str,
    scene_id: &str,
    tool_id: &str,
    policy_id: &Option<String>,
    final_auto_start: bool,
    final_seed: u64,
    final_reset_seq: u64,
    final_io_seq: u64,
    tx_id: &str,
    scene_digest: &Option<String>,
    tool_digest: &Option<String>,
    scene_manifest: &SceneManifest,
    tool_manifest: &ToolManifest,
) -> Json<serde_json::Value> {
    Json(serde_json::json!({
        "mode": mode,
        "robot": running_robot,
        "active_scene": scene_id,
        "active_tool": tool_id,
        "active_policy": policy_id,
        "auto_start_policy": final_auto_start,
        "seed": final_seed,
        "reset_seq": final_reset_seq,
        "io_seq": final_io_seq,
        "tx_id": tx_id,
        "activation_state": "COMMITTED",
        "scene_digest": scene_digest,
        "tool_digest": tool_digest,
        "scene_manifest": scene_manifest,
        "tool_manifest": tool_manifest,
    }))
}

async fn reconcile_unknown_tx(state: &SandboxState) -> Result<(), (StatusCode, Json<serde_json::Value>)> {
    let existing_tx = state.tx_state.read().await.clone();
    match existing_tx {
        ActivationTxState::DegradedSplit { ref tx_id, ref detail, .. } => {
            Err(degraded_split_http_error(tx_id, detail))
        }
        ActivationTxState::Unknown {
            ref tx_id,
            ref prev_tx_id,
            ref prev_scene,
            ref prev_tool,
            ref prev_policy,
            ref prev_mode,
            prev_seed,
            prev_auto_start_policy,
            ref prev_cached_mjcf,
            ref prev_scene_digest,
            ref prev_tool_digest,
            ref target_scene,
            ref target_tool,
            ref target_policy,
            target_seed,
            target_auto_start_policy,
            ref target_composed_mjcf,
            ref target_scene_digest,
            ref target_tool_digest,
        } => {
            let unknown_err = || {
                tx_json_err(
                    StatusCode::SERVICE_UNAVAILABLE,
                    "SIM_ACTIVATION_UNKNOWN",
                    tx_id,
                    "Previous sim activation is in UNKNOWN state",
                )
            };
            let Ok(sim_url) = sim_upstream_url() else {
                return Err(unknown_err());
            };
            let Ok(upstream) = DetectorUpstream::parse(&sim_url) else {
                return Err(unknown_err());
            };
            let Ok(resp) = sim_tx_call(&upstream, Method::GET, "/sim/state", None, state.sim_timeouts.reconcile).await else {
                return Err(unknown_err());
            };
            if !resp.status.is_success() {
                return Err(unknown_err());
            }
            let Ok(sim_val) = serde_json::from_slice::<serde_json::Value>(&resp.body) else {
                return Err(unknown_err());
            };
            let (remote_tx, remote_state) = extract_remote_tx_and_state(&sim_val);
            if remote_tx == Some(tx_id.as_str()) && remote_state.eq_ignore_ascii_case("COMMITTED") {
                commit_local_active_state(
                    state,
                    "sim",
                    target_scene,
                    target_tool,
                    target_policy.clone(),
                    Some(target_seed),
                    Some(target_auto_start_policy),
                    target_composed_mjcf.clone(),
                    tx_id,
                    target_scene_digest.clone(),
                    target_tool_digest.clone(),
                    None,
                )
                .await;
                return Ok(());
            }
            if remote_state.eq_ignore_ascii_case("DEGRADED_SPLIT") {
                let own_snap = TxSnapshot {
                    tx_id: tx_id.clone(),
                    prev_tx_id: prev_tx_id.clone(),
                    prev_scene: prev_scene.clone(),
                    prev_tool: prev_tool.clone(),
                    prev_policy: prev_policy.clone(),
                    prev_mode: prev_mode.clone(),
                    prev_seed,
                    prev_auto_start_policy,
                    prev_cached_mjcf: prev_cached_mjcf.clone(),
                    prev_scene_digest: prev_scene_digest.clone(),
                    prev_tool_digest: prev_tool_digest.clone(),
                    target_scene: target_scene.clone(),
                    target_tool: target_tool.clone(),
                    target_policy: target_policy.clone(),
                    target_seed,
                    target_auto_start_policy,
                    target_composed_mjcf: target_composed_mjcf.clone(),
                    target_scene_digest: target_scene_digest.clone(),
                    target_tool_digest: target_tool_digest.clone(),
                    bundle_snapshot: None,
                };
                if remote_tx == Some(tx_id.as_str()) {
                    let detail = String::from_utf8_lossy(&resp.body).to_string();
                    let mut tx_w = state.tx_state.write().await;
                    *tx_w = own_snap.into_degraded(tx_id.clone(), true, detail);
                    return Err(degraded_split_http_error(
                        tx_id,
                        "Previous sim activation is in DEGRADED_SPLIT; call /api/v1/sandbox/recover-split first",
                    ));
                }
                let owner_tx = remote_tx
                    .map(str::to_string)
                    .unwrap_or_else(|| format!("foreign-split-{tx_id}"));
                let foreign_msg = format!(
                    "Remote sim is in DEGRADED_SPLIT owned by foreign transaction '{owner_tx}' (not '{tx_id}'); call /api/v1/sandbox/recover-split first"
                );
                let mut tx_w = state.tx_state.write().await;
                *tx_w = foreign_degraded_from_sim_val(&own_snap, owner_tx.clone(), &sim_val, foreign_msg.clone());
                return Err(degraded_split_http_error(&owner_tx, &foreign_msg));
            }
            if remote_state.eq_ignore_ascii_case("COMMITTING") {
                return Err(tx_json_err(
                    StatusCode::SERVICE_UNAVAILABLE,
                    "SIM_ACTIVATION_UNKNOWN",
                    tx_id,
                    "Previous sim activation is still COMMITTING",
                ));
            }
            if remote_state.eq_ignore_ascii_case("ABORTED")
                || (remote_tx.is_some() && remote_tx != Some(tx_id.as_str()))
            {
                restore_tx_state_after_rollback(
                    state,
                    remote_tx.map(str::to_string),
                    prev_tx_id.clone(),
                    prev_scene_digest.clone(),
                    prev_tool_digest.clone(),
                    tx_id,
                )
                .await;
                return Ok(());
            }
            Err(unknown_err())
        }
        _ => Ok(()),
    }
}

#[allow(clippy::too_many_arguments)]
async fn run_legacy_sim_activate(
    state: &SandboxState,
    upstream: &DetectorUpstream,
    sim_url: &str,
    tx_id: &str,
    running_robot: &str,
    scene_id: &str,
    tool_id: &str,
    target_seed: u64,
) -> Result<(), (StatusCode, Json<serde_json::Value>)> {
    // B2 + M02 + m01: Legacy fallback: GET /sim/state MUST return exact 200 OK,
    // valid JSON, and matching non-empty robot; any error sets Aborted.
    let state_resp = match sim_tx_call(upstream, Method::GET, "/sim/state", None, state.sim_timeouts.reconcile).await {
        Ok(r) => r,
        Err(SimTransportError::Timeout) => {
            abort_local_tx(state, tx_id).await;
            return Err((StatusCode::GATEWAY_TIMEOUT, Json(serde_json::json!({
                "error": format!("Timeout querying legacy rebot-sim /sim/state at {sim_url}"),
                "sim_url": sim_url,
            }))));
        }
        Err(SimTransportError::ConnectOrIo(e)) => {
            abort_local_tx(state, tx_id).await;
            return Err((StatusCode::BAD_GATEWAY, Json(serde_json::json!({
                "error": format!("Cannot query legacy rebot-sim /sim/state at {sim_url}: {e}"),
                "sim_url": sim_url,
            }))));
        }
    };
    if state_resp.status != StatusCode::OK {
        abort_local_tx(state, tx_id).await;
        let status = if state_resp.status.is_server_error() {
            state_resp.status
        } else {
            StatusCode::BAD_GATEWAY
        };
        let err_msg = String::from_utf8_lossy(&state_resp.body).to_string();
        return Err((status, Json(serde_json::json!({
            "error": format!("Legacy rebot-sim /sim/state failed ({}): {err_msg}", state_resp.status),
            "code": "SIM_STATE_UNAVAILABLE",
            "sim_url": sim_url,
        }))));
    }
    let state_val: serde_json::Value = match serde_json::from_slice(&state_resp.body) {
        Ok(v) => v,
        Err(e) => {
            abort_local_tx(state, tx_id).await;
            return Err((StatusCode::BAD_GATEWAY, Json(serde_json::json!({
                "error": format!("Legacy rebot-sim /sim/state returned invalid JSON: {e}"),
                "code": "SIM_STATE_INVALID",
                "sim_url": sim_url,
            }))));
        }
    };
    let Some(remote_robot_raw) = state_val
        .get("robot")
        .and_then(|v| v.as_str())
        .map(str::trim)
        .filter(|s| !s.is_empty())
    else {
        abort_local_tx(state, tx_id).await;
        return Err(robot_incompatible_error(
            "Legacy rebot-sim /sim/state missing non-empty 'robot' field".into(),
            running_robot,
            None,
            std::slice::from_ref(&running_robot.to_string()),
        ));
    };
    if normalize_robot_id(remote_robot_raw).as_deref() != Some(running_robot) {
        abort_local_tx(state, tx_id).await;
        return Err(robot_incompatible_error(
            format!(
                "rebot-sim upstream /sim/state reported robot '{remote_robot_raw}', incompatible with running armd robot '{running_robot}'"
            ),
            running_robot,
            Some(remote_robot_raw),
            std::slice::from_ref(&running_robot.to_string()),
        ));
    }

    let legacy_bytes = serde_json::to_vec(&serde_json::json!({
        "scene": scene_id,
        "tool": tool_id,
        "seed": target_seed,
        "robot": running_robot,
    }))
    .unwrap_or_default();

    let resp_bytes = match sim_post_with_timeout(
        upstream,
        sim_url,
        "/sim/scene",
        &legacy_bytes,
        state.sim_timeouts.commit,
    )
    .await
    {
        Ok(b) => b,
        Err(e) => {
            abort_local_tx(state, tx_id).await;
            return Err(e);
        }
    };
    if let Err(e) = check_upstream_robot_match(&resp_bytes, running_robot) {
        abort_local_tx(state, tx_id).await;
        return Err(e);
    }
    Ok(())
}

/// Reconciles an ambiguous commit ACK via `GET /sim/state`.
/// Returns `Ok(Some(committed_tuple))` if converged via `DEGRADED_SPLIT`, `Ok(None)` if committed cleanly.
async fn reconcile_lost_commit_ack(
    state: &SandboxState,
    upstream: &DetectorUpstream,
    running_robot: &str,
    snap: TxSnapshot,
) -> Result<Option<(bool, u64, u64, u64)>, (StatusCode, Json<serde_json::Value>)> {
    let tx_id = snap.tx_id.clone();
    {
        let mut tx_w = state.tx_state.write().await;
        *tx_w = snap.clone().into_unknown();
    }
    let Ok(sresp) = sim_tx_call(upstream, Method::GET, "/sim/state", None, state.sim_timeouts.reconcile).await else {
        return Err(tx_json_err(
            StatusCode::SERVICE_UNAVAILABLE,
            "SIM_ACTIVATION_UNKNOWN",
            &tx_id,
            "Sim commit ACK lost and /sim/state reconciliation failed",
        ));
    };
    if !sresp.status.is_success() {
        return Err(tx_json_err(
            StatusCode::SERVICE_UNAVAILABLE,
            "SIM_ACTIVATION_UNKNOWN",
            &tx_id,
            "Sim commit ACK lost and /sim/state reconciliation failed",
        ));
    }
    let sim_val = serde_json::from_slice::<serde_json::Value>(&sresp.body).unwrap_or(serde_json::Value::Null);
    let (remote_tx, remote_state) = extract_remote_tx_and_state(&sim_val);
    if remote_tx == Some(tx_id.as_str()) && remote_state.eq_ignore_ascii_case("COMMITTED") {
        return Ok(None);
    }
    if remote_state.eq_ignore_ascii_case("DEGRADED_SPLIT") {
        let detail = String::from_utf8_lossy(&sresp.body).to_string();
        if remote_tx == Some(tx_id.as_str()) {
            let committed = converge_degraded_split(state, upstream, running_robot, snap, &detail).await?;
            return Ok(Some(committed));
        }
        // B01: Foreign split owner -> adopt with target_matches_tx = false,
        // never graft foreign tx_id onto our target config.
        let owner_tx = remote_tx
            .map(str::to_string)
            .unwrap_or_else(|| format!("foreign-split-{tx_id}"));
        let foreign_msg = format!(
            "Sim activation is in DEGRADED_SPLIT owned by foreign transaction '{owner_tx}' (not '{tx_id}'); call /api/v1/sandbox/recover-split first"
        );
        {
            let mut tx_w = state.tx_state.write().await;
            *tx_w = foreign_degraded_from_sim_val(&snap, owner_tx.clone(), &sim_val, foreign_msg.clone());
        }
        return Err(degraded_split_http_error(&owner_tx, &foreign_msg));
    }
    if remote_state.eq_ignore_ascii_case("COMMITTING") {
        return Err(tx_json_err(
            StatusCode::SERVICE_UNAVAILABLE,
            "SIM_ACTIVATION_UNKNOWN",
            &tx_id,
            "Sim commit still COMMITTING during /sim/state reconciliation",
        ));
    }
    if remote_state.eq_ignore_ascii_case("ABORTED")
        || (remote_tx.is_some() && remote_tx != Some(tx_id.as_str()))
    {
        restore_tx_state_after_rollback(
            state,
            remote_tx.map(str::to_string),
            snap.prev_tx_id,
            snap.prev_scene_digest,
            snap.prev_tool_digest,
            &tx_id,
        )
        .await;
        return Err(tx_json_err(
            StatusCode::BAD_GATEWAY,
            "SIM_COMMIT_ABORTED",
            &tx_id,
            "Sim commit aborted according to /sim/state reconciliation",
        ));
    }
    Err(tx_json_err(
        StatusCode::SERVICE_UNAVAILABLE,
        "SIM_ACTIVATION_UNKNOWN",
        &tx_id,
        "Sim commit ACK lost and /sim/state is ambiguous",
    ))
}

/// POST /api/v1/sandbox/activate
pub async fn activate(
    State(supervisor): State<Arc<crate::supervisor::ArmSupervisor>>,
    Extension(state): Extension<Arc<SandboxState>>,
    Json(payload): Json<ActivateRequest>,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<serde_json::Value>)> {
    let mode = payload.mode.to_ascii_lowercase();
    if !matches!(mode.as_str(), "none" | "wasm" | "sim") {
        return Err((StatusCode::BAD_REQUEST, Json(serde_json::json!({"error": format!("Invalid mode '{mode}', must be none, wasm, or sim")}))));
    }

    let _act_guard = state.activate_lock.lock().await;

    // 1. Refuse any new activation while SandboxState is DEGRADED_SPLIT (or reconcile Unknown first).
    reconcile_unknown_tx(&state).await?;

    // Validate requested robot/profile against running robot BEFORE resolving scene/tool/policy.
    let running_robot = running_robot_id(&supervisor).await;
    let requested_robot = resolve_requested_robot(
        payload.robot.as_deref(),
        payload.profile.as_deref(),
        payload.profile_id.as_deref(),
    )?;
    ensure_requested_robot_matches(&running_robot, requested_robot.as_deref())?;

    let prev_active = state.active.read().await.clone();
    let (prev_tx_id, prev_scene_digest, prev_tool_digest) = match &*state.tx_state.read().await {
        ActivationTxState::Committed { tx_id, scene_digest, tool_digest } => {
            (Some(tx_id.clone()), scene_digest.clone(), tool_digest.clone())
        }
        _ => (None, None, None),
    };

    let scene_id = payload.scene.unwrap_or_else(|| prev_active.active_scene.clone());
    let tool_id = payload.tool.unwrap_or_else(|| prev_active.active_tool.clone());

    if !validate_pack_id(&scene_id) {
        return Err((StatusCode::BAD_REQUEST, Json(serde_json::json!({"error": format!("Invalid scene pack id '{scene_id}'")}))));
    }
    if !validate_pack_id(&tool_id) {
        return Err((StatusCode::BAD_REQUEST, Json(serde_json::json!({"error": format!("Invalid tool pack id '{tool_id}'")}))));
    }

    // M21: Determine target policy ID and validate ID format before collecting pre-read fingerprint.
    let (policy_id, policy_missing_status, policy_missing_code) = match payload.policy {
        Some(Some(ref pid)) => {
            if !validate_pack_id(pid) {
                return Err((StatusCode::BAD_REQUEST, Json(serde_json::json!({"error": format!("Invalid policy pack id '{pid}'")}))));
            }
            (Some(pid.clone()), StatusCode::NOT_FOUND, "POLICY_NOT_FOUND")
        }
        Some(None) => (None, StatusCode::NOT_FOUND, "POLICY_NOT_FOUND"),
        None => (
            prev_active.active_policy.clone(),
            StatusCode::CONFLICT,
            "ACTIVE_POLICY_MISSING",
        ),
    };

    let resolved_base = resolve_base_robot_xml_for_robot(&running_robot)?;
    let base_xml = resolved_base.xml;
    let pre_fp = state.collect_pack_pre_fingerprint(
        &scene_id,
        &tool_id,
        policy_id.as_deref(),
        &base_xml,
        &resolved_base.watch_paths,
    );

    let ResolvedPack {
        canon_dir: scene_dir,
        manifest: scene_manifest,
        xml: scene_world_xml,
        digest: scene_digest,
        files: scene_files,
    } = state.resolve_scene_checked_full(&scene_id)?;
    let ResolvedPack {
        canon_dir: tool_dir,
        manifest: tool_manifest,
        xml: tool_xml,
        digest: tool_digest,
        files: tool_files,
    } = state.resolve_tool_checked_full(&tool_id)?;

    validate_robot_compatibility(
        &running_robot,
        requested_robot.as_deref(),
        &scene_manifest,
        &tool_manifest,
    )?;

    // M21: Validate policy completely BEFORE prepare (never run fallible disk checks after remote commit).
    let policy_manifest = if let Some(ref pid) = policy_id {
        let pm = state.resolve_policy_checked(pid, policy_missing_status, policy_missing_code)?;
        validate_policy_robot_compatibility(&running_robot, &pm)?;
        Some(pm)
    } else {
        None
    };

    let composed = compose_mjcf_checked(
        &base_xml,
        Some(&scene_world_xml),
        Some(&tool_manifest),
        Some(&tool_xml),
    )
    .map_err(|e| PackFileError::from(e).into_http_error("INVALID_PACK", "MJCF composition failed"))?;

    if !pre_fp.matches_disk() {
        return Err((
            StatusCode::CONFLICT,
            Json(serde_json::json!({
                "error": "Pack files modified on disk during bundle build",
                "code": "INVALID_PACK",
            })),
        ));
    }

    let bundle_snapshot = ActivationBundleSnapshot {
        robot: running_robot.clone(),
        scene_id: scene_id.clone(),
        scene_manifest: scene_manifest.clone(),
        scene_digest: scene_digest.clone(),
        scene_files,
        tool_id: tool_id.clone(),
        tool_manifest: tool_manifest.clone(),
        tool_digest: tool_digest.clone(),
        tool_files,
        policy_id: policy_id.clone(),
        policy_manifest,
        base_xml,
        composed_xml: composed.clone(),
        pre_fp,
    };

    let target_seed = payload.seed.unwrap_or(prev_active.seed);
    let target_auto_start = payload.auto_start_policy.unwrap_or(prev_active.auto_start_policy);
    let tx_id = generate_tx_id();

    if mode == "sim" {
        let sim_url = sim_upstream_url()?;
        let upstream = DetectorUpstream::parse(&sim_url)
            .map_err(|e| (StatusCode::BAD_GATEWAY, Json(serde_json::json!({"error": e, "sim_url": sim_url}))))?;

        let snap = TxSnapshot {
            tx_id: tx_id.clone(),
            prev_tx_id: prev_tx_id.clone(),
            prev_scene: prev_active.active_scene.clone(),
            prev_tool: prev_active.active_tool.clone(),
            prev_policy: prev_active.active_policy.clone(),
            prev_mode: prev_active.mode.clone(),
            prev_seed: prev_active.seed,
            prev_auto_start_policy: prev_active.auto_start_policy,
            prev_cached_mjcf: prev_active.cached_mjcf.clone(),
            prev_scene_digest: prev_scene_digest.clone(),
            prev_tool_digest: prev_tool_digest.clone(),
            target_scene: scene_id.clone(),
            target_tool: tool_id.clone(),
            target_policy: policy_id.clone(),
            target_seed,
            target_auto_start_policy: target_auto_start,
            target_composed_mjcf: composed.clone(),
            target_scene_digest: scene_digest.clone(),
            target_tool_digest: tool_digest.clone(),
            bundle_snapshot: Some(bundle_snapshot.clone()),
        };

        {
            let mut tx_w = state.tx_state.write().await;
            *tx_w = ActivationTxState::Preparing { tx_id: tx_id.clone() };
        }

        let mut prepare_payload = serde_json::json!({
            "tx_id": tx_id,
            "scene": scene_id,
            "tool": tool_id,
            "seed": target_seed,
            "robot": running_robot,
            "scene_digest": scene_digest,
            "tool_digest": tool_digest,
        });
        if let Some(ref d) = scene_dir {
            prepare_payload["scene_dir"] = serde_json::json!(d.to_string_lossy());
        }
        if let Some(ref d) = tool_dir {
            prepare_payload["tool_dir"] = serde_json::json!(d.to_string_lossy());
        }
        let prepare_bytes = serde_json::to_vec(&prepare_payload).unwrap_or_default();

        match sim_tx_call(
            &upstream,
            Method::POST,
            "/sim/scene/prepare",
            Some(&prepare_bytes),
            state.sim_timeouts.prepare,
        )
        .await
        {
            Ok(resp) if resp.status == StatusCode::NOT_FOUND => {
                run_legacy_sim_activate(
                    &state,
                    &upstream,
                    &sim_url,
                    &tx_id,
                    &running_robot,
                    &scene_id,
                    &tool_id,
                    target_seed,
                )
                .await?;
            }
            Ok(resp) if resp.status == StatusCode::OK => {
                if let Err(err) = verify_prepare_ack(&resp.body, &tx_id, &running_robot) {
                    sim_best_effort_abort(&upstream, &tx_id, state.sim_timeouts.abort).await;
                    abort_local_tx(&state, &tx_id).await;
                    return Err(err);
                }
                {
                    let mut tx_w = state.tx_state.write().await;
                    *tx_w = ActivationTxState::Prepared { tx_id: tx_id.clone() };
                }
                {
                    let mut tx_w = state.tx_state.write().await;
                    *tx_w = ActivationTxState::Committing { tx_id: tx_id.clone() };
                }

                let commit_bytes = serde_json::to_vec(&serde_json::json!({
                    "tx_id": tx_id,
                    "phase": "all",
                }))
                .unwrap_or_default();

                enum CommitOutcome {
                    CleanCommit,
                    VerifyRejected((StatusCode, Json<serde_json::Value>)),
                    Degraded(String),
                    FailedHttp(StatusCode, String),
                    NeedReconcile,
                }

                let outcome = match sim_tx_call(
                    &upstream,
                    Method::POST,
                    "/sim/scene/commit",
                    Some(&commit_bytes),
                    state.sim_timeouts.commit,
                )
                .await
                {
                    Ok(cresp) if cresp.status.is_success() => {
                        if is_degraded_response(&cresp.body) {
                            CommitOutcome::Degraded(String::from_utf8_lossy(&cresp.body).to_string())
                        } else if serde_json::from_slice::<serde_json::Value>(&cresp.body).is_err() {
                            // M06: malformed/damaged 2xx body -> commit may have succeeded remotely; reconcile via /sim/state
                            CommitOutcome::NeedReconcile
                        } else if let Err(err) = verify_commit_ack(
                            &cresp.body,
                            &tx_id,
                            Some(&running_robot),
                            scene_digest.as_deref(),
                            tool_digest.as_deref(),
                        ) {
                            CommitOutcome::VerifyRejected(err)
                        } else {
                            CommitOutcome::CleanCommit
                        }
                    }
                    Ok(cresp)
                        if cresp.status == StatusCode::SERVICE_UNAVAILABLE
                            && is_degraded_response(&cresp.body) =>
                    {
                        CommitOutcome::Degraded(String::from_utf8_lossy(&cresp.body).to_string())
                    }
                    Ok(cresp)
                        if matches!(
                            cresp.status,
                            StatusCode::BAD_GATEWAY | StatusCode::GATEWAY_TIMEOUT
                        ) =>
                    {
                        // M06: 502/504 -> reconcile via GET /sim/state
                        CommitOutcome::NeedReconcile
                    }
                    Ok(cresp) => CommitOutcome::FailedHttp(
                        cresp.status,
                        String::from_utf8_lossy(&cresp.body).to_string(),
                    ),
                    Err(_) => CommitOutcome::NeedReconcile,
                };

                match outcome {
                    CommitOutcome::CleanCommit => {}
                    CommitOutcome::Degraded(detail) => {
                        let (final_auto_start, final_seed, final_reset_seq, final_io_seq) =
                            converge_degraded_split(&state, &upstream, &running_robot, snap, &detail)
                                .await?;
                        return Ok(activate_committed_response(
                            &mode,
                            &running_robot,
                            &scene_id,
                            &tool_id,
                            &policy_id,
                            final_auto_start,
                            final_seed,
                            final_reset_seq,
                            final_io_seq,
                            &tx_id,
                            &scene_digest,
                            &tool_digest,
                            &scene_manifest,
                            &tool_manifest,
                        ));
                    }
                    CommitOutcome::VerifyRejected(err) => {
                        sim_best_effort_abort(&upstream, &tx_id, state.sim_timeouts.abort).await;
                        abort_local_tx(&state, &tx_id).await;
                        return Err(err);
                    }
                    CommitOutcome::FailedHttp(status, err_msg) => {
                        sim_best_effort_abort(&upstream, &tx_id, state.sim_timeouts.abort).await;
                        abort_local_tx(&state, &tx_id).await;
                        return Err((
                            status,
                            Json(serde_json::json!({
                                "error": format!("rebot-sim commit failed: {err_msg}"),
                                "sim_url": sim_url,
                                "tx_id": tx_id,
                            })),
                        ));
                    }
                    CommitOutcome::NeedReconcile => {
                        if let Some((final_auto_start, final_seed, final_reset_seq, final_io_seq)) =
                            reconcile_lost_commit_ack(&state, &upstream, &running_robot, snap).await?
                        {
                            return Ok(activate_committed_response(
                                &mode,
                                &running_robot,
                                &scene_id,
                                &tool_id,
                                &policy_id,
                                final_auto_start,
                                final_seed,
                                final_reset_seq,
                                final_io_seq,
                                &tx_id,
                                &scene_digest,
                                &tool_digest,
                                &scene_manifest,
                                &tool_manifest,
                            ));
                        }
                    }
                }
            }
            Ok(resp) if resp.status.is_success() => {
                // M09: non-200 2xx (e.g. 202 Accepted) is NOT a valid prepare ACK
                sim_best_effort_abort(&upstream, &tx_id, state.sim_timeouts.abort).await;
                abort_local_tx(&state, &tx_id).await;
                return Err(tx_json_err(
                    StatusCode::BAD_GATEWAY,
                    "SIM_INVALID_PREPARE_ACK",
                    &tx_id,
                    format!("rebot-sim prepare returned unexpected 2xx status {}", resp.status),
                ));
            }
            Ok(resp) if resp.status == StatusCode::SERVICE_UNAVAILABLE && is_degraded_response(&resp.body) => {
                // M3: store the remote split owner tx_id (from response or GET /sim/state), NOT the newly requested tx_id
                let detail = String::from_utf8_lossy(&resp.body).to_string();
                let mut owner_tx = extract_remote_tx_id(&resp.body);
                if owner_tx.is_none() {
                    if let Ok(sresp) = sim_tx_call(
                        &upstream,
                        Method::GET,
                        "/sim/state",
                        None,
                        state.sim_timeouts.reconcile,
                    )
                    .await
                    {
                        if sresp.status.is_success() {
                            owner_tx = extract_remote_tx_id(&sresp.body);
                        }
                    }
                }
                let owner_tx_id = owner_tx.unwrap_or_else(|| tx_id.clone());
                let target_matches_tx = owner_tx_id == tx_id;
                let mut tx_w = state.tx_state.write().await;
                *tx_w = snap.into_degraded(owner_tx_id.clone(), target_matches_tx, detail.clone());
                return Err(degraded_split_http_error(&owner_tx_id, &detail));
            }
            Ok(resp)
                if resp.status == StatusCode::BAD_REQUEST
                    || resp.status == StatusCode::UNPROCESSABLE_ENTITY =>
            {
                sim_best_effort_abort(&upstream, &tx_id, state.sim_timeouts.abort).await;
                abort_local_tx(&state, &tx_id).await;
                let parsed = serde_json::from_slice::<serde_json::Value>(&resp.body).ok();
                let upstream_code = parsed
                    .as_ref()
                    .and_then(|v| v.get("code").and_then(|c| c.as_str()))
                    .unwrap_or("");
                let code = match upstream_code {
                    "ROBOT_INCOMPATIBLE" => "ROBOT_INCOMPATIBLE",
                    "UNSUPPORTED_FORMAT" => "UNSUPPORTED_FORMAT",
                    _ => "INVALID_PACK",
                };
                let msg = parsed
                    .as_ref()
                    .and_then(|v| {
                        v.get("error")
                            .or_else(|| v.get("message"))
                            .and_then(|m| m.as_str())
                            .map(str::to_string)
                    })
                    .unwrap_or_else(|| String::from_utf8_lossy(&resp.body).to_string());
                return Err(tx_json_err(StatusCode::BAD_REQUEST, code, &tx_id, msg));
            }
            Ok(resp) => {
                sim_best_effort_abort(&upstream, &tx_id, state.sim_timeouts.abort).await;
                abort_local_tx(&state, &tx_id).await;
                let err_msg = String::from_utf8_lossy(&resp.body).to_string();
                return Err((
                    resp.status,
                    Json(serde_json::json!({
                        "error": format!("rebot-sim prepare error: {err_msg}"),
                        "sim_url": sim_url,
                        "tx_id": tx_id,
                    })),
                ));
            }
            Err(err) => {
                sim_best_effort_abort(&upstream, &tx_id, state.sim_timeouts.abort).await;
                abort_local_tx(&state, &tx_id).await;
                let (status, msg) = match err {
                    SimTransportError::ConnectOrIo(e) => (
                        StatusCode::BAD_GATEWAY,
                        format!("Cannot connect to rebot-sim at {sim_url}: {e}"),
                    ),
                    SimTransportError::Timeout => (
                        StatusCode::GATEWAY_TIMEOUT,
                        format!("Timeout during rebot-sim prepare at {sim_url}"),
                    ),
                };
                return Err((
                    status,
                    Json(serde_json::json!({
                        "error": msg,
                        "sim_url": sim_url,
                        "tx_id": tx_id,
                    })),
                ));
            }
        }
    }

    let (final_auto_start, final_seed, final_reset_seq, final_io_seq) = commit_local_active_state(
        &state,
        &mode,
        &scene_id,
        &tool_id,
        policy_id.clone(),
        payload.seed,
        payload.auto_start_policy,
        composed,
        &tx_id,
        scene_digest.clone(),
        tool_digest.clone(),
        Some(bundle_snapshot),
    )
    .await;

    Ok(activate_committed_response(
        &mode,
        &running_robot,
        &scene_id,
        &tool_id,
        &policy_id,
        final_auto_start,
        final_seed,
        final_reset_seq,
        final_io_seq,
        &tx_id,
        &scene_digest,
        &tool_digest,
        &scene_manifest,
        &tool_manifest,
    ))
}

/// POST /api/v1/sandbox/recover-split
pub async fn recover_split(
    State(supervisor): State<Arc<crate::supervisor::ArmSupervisor>>,
    Extension(state): Extension<Arc<SandboxState>>,
    Json(payload): Json<RecoverSplitRequest>,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<serde_json::Value>)> {
    let phase = match payload.action.as_str() {
        "replay_camera" => "camera_only",
        "rollback_physics" => "rollback_physics",
        other => {
            return Err(code_json_err(
                StatusCode::BAD_REQUEST,
                "INVALID_ACTION",
                format!("Invalid recovery action '{other}'; expected 'replay_camera' or 'rollback_physics'"),
            ));
        }
    };

    let _act_guard = state.activate_lock.lock().await;
    let running_robot = running_robot_id(&supervisor).await;
    let sim_url = sim_upstream_url()?;
    let upstream = DetectorUpstream::parse(&sim_url)
        .map_err(|e| (StatusCode::BAD_GATEWAY, Json(serde_json::json!({"error": e, "sim_url": sim_url}))))?;

    let cur_tx = state.tx_state.read().await.clone();
    let (
        tx_id,
        prev_tx_id,
        prev_scene,
        prev_tool,
        prev_scene_digest,
        prev_tool_digest,
        target_scene,
        target_tool,
        target_policy,
        target_seed,
        target_auto_start_policy,
        target_composed_mjcf,
        target_scene_digest,
        target_tool_digest,
        target_matches_tx,
        detail,
    ) = match cur_tx {
        ActivationTxState::DegradedSplit {
            tx_id,
            prev_tx_id,
            prev_scene,
            prev_tool,
            prev_scene_digest,
            prev_tool_digest,
            target_scene,
            target_tool,
            target_policy,
            target_seed,
            target_auto_start_policy,
            target_composed_mjcf,
            target_scene_digest,
            target_tool_digest,
            target_matches_tx,
            detail,
            ..
        } => (
            tx_id,
            prev_tx_id,
            prev_scene,
            prev_tool,
            prev_scene_digest,
            prev_tool_digest,
            target_scene,
            target_tool,
            target_policy,
            target_seed,
            target_auto_start_policy,
            target_composed_mjcf,
            target_scene_digest,
            target_tool_digest,
            target_matches_tx,
            detail,
        ),
        // M5: Restart recovery (or Unknown): query GET /sim/state and adopt remote DEGRADED_SPLIT tx_id
        ActivationTxState::Idle | ActivationTxState::Unknown { .. } => {
            let sresp = sim_tx_call(
                &upstream,
                Method::GET,
                "/sim/state",
                None,
                state.sim_timeouts.reconcile,
            )
            .await
            .map_err(|_| {
                code_json_err(
                    StatusCode::BAD_GATEWAY,
                    "SIM_STATE_UNAVAILABLE",
                    format!("Cannot query rebot-sim /sim/state at {sim_url}"),
                )
            })?;
            if !sresp.status.is_success() {
                return Err(code_json_err(
                    StatusCode::CONFLICT,
                    "NOT_DEGRADED",
                    "Sandbox is not in DEGRADED_SPLIT state",
                ));
            }
            let sim_val = serde_json::from_slice::<serde_json::Value>(&sresp.body)
                .unwrap_or(serde_json::Value::Null);
            let (remote_tx, remote_state) = extract_remote_tx_and_state(&sim_val);
            if !remote_state.eq_ignore_ascii_case("DEGRADED_SPLIT") {
                return Err(code_json_err(
                    StatusCode::CONFLICT,
                    "NOT_DEGRADED",
                    "Sandbox is not in DEGRADED_SPLIT state",
                ));
            }
            let adopted_tx = remote_tx.ok_or_else(|| {
                code_json_err(
                    StatusCode::BAD_GATEWAY,
                    "SIM_STATE_INVALID",
                    "Remote /sim/state reports DEGRADED_SPLIT without tx_id",
                )
            })?;
            let prev_active = state.active.read().await.clone();
            let r_scene = sim_val
                .get("scene")
                .and_then(|v| v.as_str())
                .map(str::to_string)
                .unwrap_or_else(|| prev_active.active_scene.clone());
            let r_tool = sim_val
                .get("tool")
                .and_then(|v| v.as_str())
                .map(str::to_string)
                .unwrap_or_else(|| prev_active.active_tool.clone());
            let r_sd = sim_val
                .get("scene_digest")
                .and_then(|v| v.as_str())
                .map(str::to_string);
            let r_td = sim_val
                .get("tool_digest")
                .and_then(|v| v.as_str())
                .map(str::to_string);

            let (resolved_target_matches, target_mjcf, target_sd, target_td) = match cur_tx {
                ActivationTxState::Unknown {
                    ref tx_id,
                    ref target_composed_mjcf,
                    ref target_scene_digest,
                    ref target_tool_digest,
                    ..
                } if tx_id == adopted_tx => (
                    true,
                    target_composed_mjcf.clone(),
                    target_scene_digest.clone(),
                    target_tool_digest.clone(),
                ),
                _ => resolve_remote_target_locally(
                    &state,
                    &running_robot,
                    &r_scene,
                    &r_tool,
                    r_sd,
                    r_td,
                ),
            };

            let detail = String::from_utf8_lossy(&sresp.body).to_string();
            (
                adopted_tx.to_string(),
                None,
                prev_active.active_scene,
                prev_active.active_tool,
                None,
                None,
                r_scene,
                r_tool,
                prev_active.active_policy,
                prev_active.seed,
                prev_active.auto_start_policy,
                target_mjcf,
                target_sd,
                target_td,
                resolved_target_matches,
                detail,
            )
        }
        _ => {
            return Err(code_json_err(
                StatusCode::CONFLICT,
                "NOT_DEGRADED",
                "Sandbox is not in DEGRADED_SPLIT state",
            ));
        }
    };

    let commit_bytes = serde_json::to_vec(&serde_json::json!({
        "tx_id": tx_id,
        "phase": phase,
    }))
    .unwrap_or_default();

    let resp = sim_tx_call(
        &upstream,
        Method::POST,
        "/sim/scene/commit",
        Some(&commit_bytes),
        state.sim_timeouts.commit,
    )
    .await
    .map_err(|_| degraded_split_http_error(&tx_id, &detail))?;

    if !resp.status.is_success() {
        let msg = String::from_utf8_lossy(&resp.body).to_string();
        return Err(degraded_split_http_error(&tx_id, &msg));
    }

    if payload.action == "replay_camera" {
        verify_commit_ack(
            &resp.body,
            &tx_id,
            Some(&running_robot),
            target_scene_digest.as_deref(),
            target_tool_digest.as_deref(),
        )?;
        if target_matches_tx {
            let (final_auto_start, final_seed, final_reset_seq, final_io_seq) =
                commit_local_active_state(
                    &state,
                    "sim",
                    &target_scene,
                    &target_tool,
                    target_policy.clone(),
                    Some(target_seed),
                    Some(target_auto_start_policy),
                    target_composed_mjcf,
                    &tx_id,
                    target_scene_digest,
                    target_tool_digest,
                    None,
                )
                .await;
            let permit = state
                .bundle_rebuild_sem
                .clone()
                .acquire_owned()
                .await
                .map_err(|_| {
                    (
                        StatusCode::INTERNAL_SERVER_ERROR,
                        Json(serde_json::json!({"error": "Bundle rebuild semaphore closed"})),
                    )
                })?;
            let state_c = state.clone();
            let robot_c = running_robot.clone();
            let active_snap = state.active.read().await.clone();
            if let Ok(Ok(cached)) = tokio::task::spawn_blocking(move || {
                let _permit = permit;
                let cached = build_bundle_from_disk(&state_c, &robot_c, None, &active_snap)?;
                state_c.insert_cached_bundle_blocking(cached.clone());
                Ok::<_, (StatusCode, Json<serde_json::Value>)>(cached)
            })
            .await
            {
                let _ = cached;
            }
            Ok(Json(serde_json::json!({
                "ok": true,
                "state": "COMMITTED",
                "tx_id": tx_id,
                "active_scene": target_scene,
                "active_tool": target_tool,
                "active_policy": target_policy,
                "auto_start_policy": final_auto_start,
                "seed": final_seed,
                "reset_seq": final_reset_seq,
                "io_seq": final_io_seq,
            })))
        } else {
            {
                let mut tx_w = state.tx_state.write().await;
                *tx_w = ActivationTxState::Committed {
                    tx_id: tx_id.clone(),
                    scene_digest: target_scene_digest,
                    tool_digest: target_tool_digest,
                };
            }
            Ok(Json(serde_json::json!({
                "ok": true,
                "state": "COMMITTED",
                "tx_id": tx_id,
            })))
        }
    } else {
        let restored_tx_id = verify_rollback_ack(&resp.body, &tx_id, prev_tx_id.as_deref())
            .map_err(|msg| degraded_split_http_error(&tx_id, &msg))?;
        restore_tx_state_after_rollback(
            &state,
            restored_tx_id.clone(),
            prev_tx_id,
            prev_scene_digest,
            prev_tool_digest,
            &tx_id,
        )
        .await;
        Ok(Json(serde_json::json!({
            "ok": true,
            "state": "ROLLED_BACK",
            "tx_id": tx_id,
            "restored_tx_id": restored_tx_id,
            "active_scene": prev_scene,
            "active_tool": prev_tool,
        })))
    }
}

/// POSTs `body` to rebot-sim; a non-2xx reply, connect failure or timeout becomes the handler error.
async fn sim_post_with_timeout(
    upstream: &DetectorUpstream,
    sim_url: &str,
    path: &str,
    body: &[u8],
    timeout: Duration,
) -> Result<Vec<u8>, (StatusCode, Json<serde_json::Value>)> {
    let req_fut = proxy::upstream_request(upstream, Method::POST, path, Some(body));
    match tokio::time::timeout(timeout, req_fut).await {
        Ok(Ok(resp)) if !resp.status.is_success() => {
            let err_msg = String::from_utf8_lossy(&resp.body).to_string();
            Err((resp.status, Json(serde_json::json!({"error": format!("rebot-sim error: {err_msg}"), "sim_url": sim_url}))))
        }
        Ok(Ok(resp)) => Ok(resp.body),
        Ok(Err(e)) => Err((StatusCode::BAD_GATEWAY, Json(serde_json::json!({
            "error": format!("Cannot connect to rebot-sim at {sim_url}: {e}"),
            "sim_url": sim_url,
        })))),
        Err(_elapsed) => Err((StatusCode::GATEWAY_TIMEOUT, Json(serde_json::json!({
            "error": format!("Timeout connecting to rebot-sim at {sim_url}"),
            "sim_url": sim_url,
        })))),
    }
}

async fn sim_post(
    upstream: &DetectorUpstream,
    sim_url: &str,
    path: &str,
    body: &[u8],
) -> Result<Vec<u8>, (StatusCode, Json<serde_json::Value>)> {
    sim_post_with_timeout(upstream, sim_url, path, body, Duration::from_secs(10)).await
}

#[derive(Debug, Default, Deserialize)]
pub struct BundleQuery {
    #[serde(default)]
    pub robot: Option<String>,
    #[serde(default)]
    pub profile: Option<String>,
    #[serde(default)]
    pub profile_id: Option<String>,
}

/// GET /api/v1/sandbox/bundle
pub async fn get_bundle(
    State(supervisor): State<Arc<crate::supervisor::ArmSupervisor>>,
    Extension(state): Extension<Arc<SandboxState>>,
    Query(query): Query<BundleQuery>,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<serde_json::Value>)> {
    // Validate requested robot/profile against running robot BEFORE resolving scene/tool/policy.
    let running_robot = running_robot_id(&supervisor).await;
    let requested_robot = resolve_requested_robot(
        query.robot.as_deref(),
        query.profile.as_deref(),
        query.profile_id.as_deref(),
    )?;
    ensure_requested_robot_matches(&running_robot, requested_robot.as_deref())?;

    let current = state.active.read().await.clone();
    if let Some(cached) = state.lookup_cached_bundle(&running_robot, &current).await {
        if state.check_bundle_fresh(&cached).await {
            return Ok(Json(render_bundle_json(&cached, &current)));
        }
    }

    let permit = state
        .bundle_rebuild_sem
        .clone()
        .acquire_owned()
        .await
        .map_err(|_| {
            (
                StatusCode::INTERNAL_SERVER_ERROR,
                Json(serde_json::json!({"error": "Bundle rebuild semaphore closed"})),
            )
        })?;

    let current = state.active.read().await.clone();
    if let Some(cached) = state.lookup_cached_bundle(&running_robot, &current).await {
        if state.check_bundle_fresh(&cached).await {
            return Ok(Json(render_bundle_json(&cached, &current)));
        }
    }

    let state_c = state.clone();
    let robot_c = running_robot.clone();
    let req_robot_c = requested_robot.clone();
    let current_c = current.clone();
    let (new_bundle, render_state) = tokio::task::spawn_blocking(move || {
        let _permit = permit;
        state_c.bundle_rebuild_count.fetch_add(1, Ordering::Relaxed);
        let new_bundle =
            build_bundle_from_disk(&state_c, &robot_c, req_robot_c.as_deref(), &current_c)?;
        let latest = state_c.active.blocking_read().clone();
        let same_identity = latest.active_scene == current_c.active_scene
            && latest.active_tool == current_c.active_tool
            && latest.active_policy == current_c.active_policy
            && latest.committed_scene_digest == current_c.committed_scene_digest
            && latest.committed_tool_digest == current_c.committed_tool_digest;
        if same_identity && latest.updated_at_ms == current_c.updated_at_ms {
            state_c.insert_cached_bundle_blocking(new_bundle.clone());
        }
        let render_state = if same_identity { latest } else { current_c };
        Ok::<_, (StatusCode, Json<serde_json::Value>)>((new_bundle, render_state))
    })
    .await
    .map_err(|e| {
        (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(serde_json::json!({"error": format!("Bundle rebuild task failed: {e}")})),
        )
    })??;

    Ok(Json(render_bundle_json(&new_bundle, &render_state)))
}

fn inject_tx_state_fields(obj: &mut serde_json::Value, tx_state: &ActivationTxState) {
    obj["activation_state"] = serde_json::json!(tx_state.label());
    match tx_state {
        ActivationTxState::DegradedSplit { tx_id, detail, .. } => {
            obj["degraded"] = serde_json::json!(true);
            obj["degraded_tx_id"] = serde_json::json!(tx_id);
            obj["degraded_detail"] = serde_json::json!(detail);
        }
        ActivationTxState::Unknown { tx_id, .. } => {
            obj["degraded"] = serde_json::json!(false);
            obj["unknown_tx_id"] = serde_json::json!(tx_id);
        }
        _ => {
            obj["degraded"] = serde_json::json!(false);
        }
    }
}

fn build_state_response(
    current: &ActiveState,
    tx_state: &ActivationTxState,
    sim_reachable: Option<bool>,
    sim_state: Option<serde_json::Value>,
) -> Json<serde_json::Value> {
    let mut out = serde_json::json!({
        "mode": current.mode,
        "active_scene": current.active_scene,
        "active_tool": current.active_tool,
        "active_policy": current.active_policy,
        "seed": current.seed,
        "reset_seq": current.reset_seq,
        "io_seq": current.io_seq,
        "desired_io": current.desired_io,
        "telemetry": current.telemetry,
        "updated_at_ms": current.updated_at_ms,
    });
    if let Some(reachable) = sim_reachable {
        out["sim_reachable"] = serde_json::json!(reachable);
    }
    if let Some(sv) = sim_state {
        out["sim_state"] = sv;
    }
    inject_tx_state_fields(&mut out, tx_state);
    Json(out)
}

/// GET /api/v1/sandbox/state
pub async fn get_state(
    Extension(state): Extension<Arc<SandboxState>>,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<serde_json::Value>)> {
    let current = state.active.read().await.clone();
    let tx_state = state.tx_state.read().await.clone();

    if current.mode != "sim" {
        return Ok(build_state_response(&current, &tx_state, None, None));
    }

    let sim_url = sim_upstream_url().unwrap_or_default();
    if let Ok(upstream) = DetectorUpstream::parse(&sim_url) {
        let req_fut = proxy::upstream_request(&upstream, Method::GET, "/sim/state", None);
        match tokio::time::timeout(std::time::Duration::from_secs(10), req_fut).await {
            Ok(Ok(resp)) if resp.status.is_success() => {
                if let Ok(sim_val) = serde_json::from_slice::<serde_json::Value>(&resp.body) {
                    return Ok(build_state_response(&current, &tx_state, Some(true), Some(sim_val)));
                }
            }
            Ok(_) => {}
            Err(_elapsed) => {
                return Err((StatusCode::GATEWAY_TIMEOUT, Json(serde_json::json!({
                    "error": format!("Timeout connecting to rebot-sim at {sim_url}"),
                    "sim_url": sim_url,
                }))));
            }
        }
    }
    Ok(build_state_response(&current, &tx_state, Some(false), None))
}

#[derive(Debug, Deserialize)]
pub struct ResetRequest {
    pub seed: Option<u64>,
}

/// POST /api/v1/sandbox/reset
pub async fn reset(
    Extension(state): Extension<Arc<SandboxState>>,
    payload: Option<Json<ResetRequest>>,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<serde_json::Value>)> {
    let (seed, mode) = {
        let cur = state.active.read().await;
        let s = if let Some(Json(ref req)) = payload {
            req.seed.unwrap_or(cur.seed)
        } else {
            cur.seed
        };
        (s, cur.mode.clone())
    };

    if mode == "sim" {
        let sim_url = sim_upstream_url()?;
        let upstream = DetectorUpstream::parse(&sim_url)
            .map_err(|e| (StatusCode::BAD_GATEWAY, Json(serde_json::json!({"error": e, "sim_url": sim_url}))))?;
        let body = serde_json::to_vec(&serde_json::json!({"seed": seed})).unwrap();
        sim_post(&upstream, &sim_url, "/sim/reset", &body).await?;
    }

    let (seq, final_seed) = {
        let mut current = state.active.write().await;
        current.seed = seed;
        current.reset_seq = current.reset_seq.wrapping_add(1);
        current.telemetry = None;
        current.updated_at_ms = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_millis() as u64)
            .unwrap_or(0);
        (current.reset_seq, current.seed)
    };

    state.persist().await;

    Ok(Json(serde_json::json!({
        "ok": true,
        "reset_seq": seq,
        "seed": final_seed,
    })))
}

/// Most keys `desired_io` holds (it is persisted to state.json on every IO write).
pub const MAX_IO_KEYS: usize = 64;
/// Longest serialized JSON value of one IO key.
pub const MAX_IO_VALUE_BYTES: usize = 4096;

/// IO keys: `^[A-Za-z0-9_.-]{1,64}$`; values at most [`MAX_IO_VALUE_BYTES`] as JSON.
fn check_io_updates(updates: &serde_json::Map<String, serde_json::Value>) -> Result<(), String> {
    if updates.len() > MAX_IO_KEYS {
        return Err(format!("At most {MAX_IO_KEYS} IO keys per request, got {}", updates.len()));
    }
    for (key, value) in updates {
        if key.is_empty() || key.len() > 64 || !key.bytes().all(|b| b.is_ascii_alphanumeric() || matches!(b, b'_' | b'.' | b'-')) {
            return Err(format!("Invalid IO key {key:?}: must match ^[A-Za-z0-9_.-]{{1,64}}$"));
        }
        let size = serde_json::to_vec(value).map_or(usize::MAX, |v| v.len());
        if size > MAX_IO_VALUE_BYTES {
            return Err(format!("IO value of {key:?} is {size} bytes as JSON, at most {MAX_IO_VALUE_BYTES}"));
        }
    }
    Ok(())
}

/// `desired_io` after merging `updates` holds at most [`MAX_IO_KEYS`] keys.
fn check_io_key_count(desired: &serde_json::Value, updates: &serde_json::Map<String, serde_json::Value>) -> Result<(), String> {
    let existing = desired.as_object();
    let added = updates.keys().filter(|k| !existing.is_some_and(|m| m.contains_key(*k))).count();
    let total = existing.map_or(0, |m| m.len()) + added;
    if total > MAX_IO_KEYS {
        return Err(format!("desired_io would hold {total} keys, at most {MAX_IO_KEYS}"));
    }
    Ok(())
}

/// POST /api/v1/sandbox/io
pub async fn set_io(
    Extension(state): Extension<Arc<SandboxState>>,
    body: Bytes,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<serde_json::Value>)> {
    let parsed: serde_json::Value = serde_json::from_slice(&body)
        .map_err(|e| (StatusCode::BAD_REQUEST, Json(serde_json::json!({"error": format!("Invalid JSON: {e}")}))))?;

    let updates = if let Some(up) = parsed.get("updates").and_then(|v| v.as_object()) {
        up.clone()
    } else if let Some(obj) = parsed.as_object() {
        obj.clone()
    } else {
        return Err((StatusCode::BAD_REQUEST, Json(serde_json::json!({"error": "Expected object or {updates: ...}"}))));
    };
    let bad_request = |error: String| (StatusCode::BAD_REQUEST, Json(serde_json::json!({"error": error})));
    check_io_updates(&updates).map_err(bad_request)?;

    let mode = {
        let current = state.active.read().await;
        check_io_key_count(&current.desired_io, &updates).map_err(bad_request)?;
        current.mode.clone()
    };

    if mode == "sim" {
        let sim_url = sim_upstream_url()?;
        let upstream = DetectorUpstream::parse(&sim_url)
            .map_err(|e| (StatusCode::BAD_GATEWAY, Json(serde_json::json!({"error": e, "sim_url": sim_url}))))?;
        let req_body = serde_json::to_vec(&updates).unwrap();
        sim_post(&upstream, &sim_url, "/sim/io", &req_body).await?;
    }

    let (io_seq, desired) = {
        let mut current = state.active.write().await;
        if !current.desired_io.is_object() {
            current.desired_io = serde_json::json!({});
        }
        // re-checked under the write lock: another request may have added keys meanwhile
        check_io_key_count(&current.desired_io, &updates).map_err(bad_request)?;
        if let Some(map) = current.desired_io.as_object_mut() {
            for (k, v) in updates.iter() {
                map.insert(k.clone(), v.clone());
            }
        }
        current.io_seq = current.io_seq.wrapping_add(1);
        current.updated_at_ms = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_millis() as u64)
            .unwrap_or(0);
        (current.io_seq, current.desired_io.clone())
    };

    state.persist().await;

    Ok(Json(serde_json::json!({
        "ok": true,
        "io_seq": io_seq,
        "desired_io": desired,
    })))
}

/// POST /api/v1/sandbox/telemetry
pub async fn post_telemetry(
    Extension(state): Extension<Arc<SandboxState>>,
    Json(payload): Json<serde_json::Value>,
) -> Json<serde_json::Value> {
    let mut current = state.active.write().await;
    current.telemetry = Some(payload);
    current.updated_at_ms = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0);

    Json(serde_json::json!({"ok": true}))
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use tower::ServiceExt;

    #[test]
    fn sha256_known_test_vectors() {
        assert_eq!(
            sha256_hex(b""),
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
        );
        assert_eq!(
            sha256_hex(b"abc"),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
        assert_eq!(
            sha256_hex(b"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"),
            "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1"
        );
        // Standard NIST multi-block (> 64 bytes) test vector (112 bytes)
        assert_eq!(
            sha256_hex(b"abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmnhijklmnoijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu"),
            "cf5b16a778af8380036ce59e7b0492370b249b11e8f07a51afac45037afee9d1"
        );
    }

    #[test]
    fn sha256_streaming_across_chunk_boundaries() {
        // Empty input streaming
        let hasher = Sha256::new();
        assert_eq!(
            hasher.finalize_hex(),
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
        );

        // "abc" fed in 1-byte chunks
        let mut hasher = Sha256::new();
        hasher.update(b"a");
        hasher.update(b"b");
        hasher.update(b"c");
        assert_eq!(
            hasher.finalize_hex(),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );

        // 1000 bytes input (e.g. repeated 'a')
        let data = vec![b'a'; 1000];
        let expected = "41edece42d63e8d9bf515a9ba6932e1c20cbc9f5a5d134645adb5db1b9737ea3";
        assert_eq!(sha256_hex(&data), expected);

        // Feed across irregular chunk boundaries crossing 64-byte block boundaries
        let mut hasher = Sha256::new();
        let chunk_sizes = [1, 2, 3, 5, 8, 13, 21, 34, 55, 63, 64, 65, 128, 200, 338];
        let mut offset = 0;
        for &sz in &chunk_sizes {
            let end = (offset + sz).min(data.len());
            if offset < end {
                hasher.update(&data[offset..end]);
                offset = end;
            }
        }
        assert_eq!(offset, 1000);
        assert_eq!(hasher.finalize_hex(), expected);

        // Verify finalize() produces identical bytes as finalize_hex()
        let mut hasher = Sha256::new();
        hasher.update(&data);
        let raw_digest = hasher.finalize();
        let hex_from_raw = raw_digest.iter().map(|b| format!("{b:02x}")).collect::<String>();
        assert_eq!(hex_from_raw, expected);
    }

    #[test]
    fn quota_enforcement_pack_count_and_storage_limits() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        let scenes_dir = root.join("scenes");
        std::fs::create_dir_all(&scenes_dir).unwrap();

        // 1. Pack count limit enforcement
        let s1 = scenes_dir.join("s1");
        let s2 = scenes_dir.join("s2");
        std::fs::create_dir_all(&s1).unwrap();
        std::fs::create_dir_all(&s2).unwrap();
        std::fs::write(s1.join("file.txt"), b"hello").unwrap();

        assert_eq!(count_custom_packs(&scenes_dir), 2);

        // Adding 3rd pack with max_packs = 2 must fail with 507 INSUFFICIENT_STORAGE
        let s3 = scenes_dir.join("s3");
        let err = enforce_sandbox_quota(root, "scenes", &s3, 10, 2, 100_000).unwrap_err();
        assert_eq!(err.0, StatusCode::INSUFFICIENT_STORAGE);
        assert!(err.1.contains("Maximum number of custom scenes packs (2) reached"));

        // Updating existing pack s1 with max_packs = 2 must SUCCEED
        assert!(enforce_sandbox_quota(root, "scenes", &s1, 10, 2, 100_000).is_ok());

        // 2. Storage quota enforcement
        // Existing s1 has 5 bytes. Write 500 bytes to s2.
        std::fs::write(s2.join("data.bin"), vec![0u8; 500]).unwrap();
        let s1_size = dir_size_excluding(&s1, None);
        assert_eq!(s1_size, 5);
        let total_existing = dir_size_excluding(root, None);
        assert_eq!(total_existing, 505);

        // Target s4 is new (100 bytes). Max limit is 600 bytes.
        // Projected: 505 + 100 = 605 > 600 -> must fail with 507
        let s4 = scenes_dir.join("s4");
        let err = enforce_sandbox_quota(root, "scenes", &s4, 100, 10, 600).unwrap_err();
        assert_eq!(err.0, StatusCode::INSUFFICIENT_STORAGE);
        assert!(err.1.contains("Sandbox storage quota exceeded"));

        // Target s2 is existing (500 bytes). If updating s2 with 80 bytes:
        // Existing excluding s2 is 5 bytes. Projected: 5 + 80 = 85 <= 600 -> must succeed!
        assert!(enforce_sandbox_quota(root, "scenes", &s2, 80, 10, 600).is_ok());
    }

    static TEST_UPLOAD_MUTEX: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());

    #[tokio::test]
    async fn upload_endpoint_enforces_quota_507() {
        let _guard = TEST_UPLOAD_MUTEX.lock().await;
        let tmp = tempfile::tempdir().unwrap();
        let state = Arc::new(SandboxState::new(tmp.path().to_path_buf()));
        let app = test_router(state.clone());

        // Fill up to MAX_CUSTOM_PACKS_PER_KIND (64) mock scene folders under state.root/scenes
        let scenes_dir = state.root.join("scenes");
        std::fs::create_dir_all(&scenes_dir).unwrap();
        for i in 0..MAX_CUSTOM_PACKS_PER_KIND {
            std::fs::create_dir_all(scenes_dir.join(format!("mock_scene_{i}"))).unwrap();
        }

        // Now attempt to upload a new scene (65th pack) via HTTP POST
        let payload = serde_json::json!({
            "id": "new_scene_overflow",
            "scene_json": {
                "id": "new_scene_overflow",
                "name": "Overflow Scene",
                "world_xml": "world.xml"
            },
            "world_xml": r#"<mujoco model="overflow"><worldbody/></mujoco>"#
        });
        let req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/scenes")
            .header("Content-Type", "application/json")
            .body(Body::from(serde_json::to_vec(&payload).unwrap()))
            .unwrap();
        let res = app.oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::INSUFFICIENT_STORAGE);
    }

    #[test]
    fn base64_decodes_and_rejects_invalid() {
        assert_eq!(base64_decode("").unwrap(), b"");
        assert_eq!(base64_decode("YWJj").unwrap(), b"abc");
        assert_eq!(base64_decode("data:application/octet-stream;base64,YWJj").unwrap(), b"abc");
        assert_eq!(base64_decode("base64:YWJj").unwrap(), b"abc");
        assert_eq!(base64_decode("aGVsbG8gd29ybGQ=").unwrap(), b"hello world");
        assert_eq!(base64_decode("  aGVs\nbG8g\r\nd29ybGQ=\n").unwrap(), b"hello world");

        assert!(base64_decode("invalid!").is_err());
        assert!(base64_decode("YWJ").is_err()); // truncated
    }

    #[test]
    fn path_traversal_and_id_validation() {
        assert!(validate_pack_id("scene1"));
        assert!(validate_pack_id("my_custom-tool-123"));
        assert!(!validate_pack_id(""));
        assert!(!validate_pack_id("_start"));
        assert!(!validate_pack_id("bad/id"));
        assert!(!validate_pack_id("bad.id"));

        assert!(is_safe_relative_path("world.xml"));
        assert!(is_safe_relative_path("meshes/part.stl"));
        assert!(!is_safe_relative_path("../world.xml"));
        assert!(!is_safe_relative_path("/etc/passwd"));
        assert!(!is_safe_relative_path("a/../../b.xml"));
        assert!(!is_safe_relative_path(".hidden.xml"));
    }

    #[test]
    fn xml_security_rejects_xxe_and_requires_elements() {
        assert!(validate_xml_security(r#"<!DOCTYPE foo [ <!ENTITY xxe "evil"> ]><mujoco/>"#, None).is_err());
        assert!(validate_xml_security(r#"<mujoco><worldbody/></mujoco>"#, Some("worldbody")).is_ok());
        assert!(validate_xml_security(r#"<mujoco><actuator/></mujoco>"#, Some("worldbody")).is_err());
    }

    #[test]
    fn mjcf_composer_merges_pcb_suction_and_suction_tool() {
        let base_xml = get_base_robot_xml();
        let (_scene_manifest, scene_world_xml) = load_builtin_scene("pcb_suction").unwrap();
        let (tool_manifest, tool_xml) = load_builtin_tool("suction").unwrap();

        let composed = compose_mjcf(&base_xml, Some(&scene_world_xml), Some(&tool_manifest), Some(&tool_xml)).unwrap();

        // Valid roxmltree Document
        let doc = roxmltree::Document::parse(&composed).expect("Composed MJCF must be valid XML");

        // Verify suction_weld_0_pcb exists
        assert!(
            composed.contains("suction_weld_0_pcb"),
            "Composed MJCF must inject suction_weld_0_pcb"
        );

        // Verify cube is removed
        assert!(
            !composed.contains("<body name=\"cube\""),
            "Composed MJCF must remove base cube body"
        );

        // Verify finger_left is removed
        assert!(
            !composed.contains("<body name=\"finger_left\""),
            "Composed MJCF must remove replaced finger_left body"
        );

        // Verify suction manifold is present
        assert!(
            composed.contains("suction_manifold"),
            "Composed MJCF must contain suction manifold"
        );

        assert!(doc.root_element().has_tag_name("mujoco"));
    }

    fn test_router(state: Arc<SandboxState>) -> axum::Router {
        test_router_with(state, Arc::new(crate::supervisor::ArmSupervisor::new(crate::model::ArmConfig::default())))
    }

    fn test_router_with(state: Arc<SandboxState>, supervisor: Arc<crate::supervisor::ArmSupervisor>) -> axum::Router {
        use axum::extract::DefaultBodyLimit;
        use axum::routing::{get, post};
        axum::Router::new()
            .route("/api/v1/sandbox/catalog", get(get_catalog))
            .route(
                "/api/v1/sandbox/scenes",
                post(upload_scene)
                    .layer(DefaultBodyLimit::max(max_upload_body_bytes()))
                    .layer(axum::middleware::from_fn(upload_concurrency_middleware)),
            )
            .route("/api/v1/sandbox/scenes/{id}", get(get_scene))
            .route("/api/v1/sandbox/scenes/{id}/files/{*path}", get(serve_scene_file))
            .route(
                "/api/v1/sandbox/tools",
                post(upload_tool)
                    .layer(DefaultBodyLimit::max(max_upload_body_bytes()))
                    .layer(axum::middleware::from_fn(upload_concurrency_middleware)),
            )
            .route("/api/v1/sandbox/tools/{id}", get(get_tool))
            .route("/api/v1/sandbox/tools/{id}/files/{*path}", get(serve_tool_file))
            .route(
                "/api/v1/sandbox/policies",
                post(upload_policy)
                    .layer(DefaultBodyLimit::max(max_upload_body_bytes()))
                    .layer(axum::middleware::from_fn(upload_concurrency_middleware)),
            )
            .route("/api/v1/sandbox/policies/{id}", get(get_policy).delete(delete_policy))
            .route("/api/v1/sandbox/policies/{id}/files/{*path}", get(serve_policy_file))
            .route("/api/v1/sandbox/base-assets/{*path}", get(serve_base_asset))
            .route("/api/v1/sandbox/activate", post(activate))
            .route("/api/v1/sandbox/recover-split", post(recover_split))
            .route("/api/v1/sandbox/bundle", get(get_bundle))
            .route("/api/v1/sandbox/state", get(get_state))
            .route("/api/v1/sandbox/reset", post(reset))
            .route("/api/v1/sandbox/io", post(set_io))
            .route("/api/v1/sandbox/telemetry", post(post_telemetry))
            .layer(Extension(state))
            .with_state(supervisor)
    }

    #[tokio::test]
    async fn axum_oneshot_sandbox_lifecycle() {
        let _guard = TEST_UPLOAD_MUTEX.lock().await;
        let tmp = tempfile::tempdir().unwrap();
        let state = Arc::new(SandboxState::new(tmp.path().to_path_buf()));
        let app = test_router(state);

        // 1. Catalog
        let req = axum::http::Request::builder().uri("/api/v1/sandbox/catalog").body(Body::empty()).unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::OK);
        let body = axum::body::to_bytes(res.into_body(), usize::MAX).await.unwrap();
        let catalog: serde_json::Value = serde_json::from_slice(&body).unwrap();
        assert!(catalog["builtin_scenes"].is_array());
        assert!(catalog["builtin_tools"].is_array());

        // 2. Upload Scene
        let scene_payload = serde_json::json!({
            "id": "test_station",
            "scene_json": {
                "id": "test_station",
                "name": "Test Workstation",
                "world_xml": "world.xml"
            },
            "world_xml": "<mujoco model=\"test_world\"><worldbody><body name=\"test_obj\" pos=\"0.2 0 0\"><freejoint/><geom type=\"box\" size=\"0.01 0.01 0.01\"/></body></worldbody></mujoco>"
        });
        let req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/scenes")
            .header("Content-Type", "application/json")
            .body(Body::from(serde_json::to_vec(&scene_payload).unwrap()))
            .unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::OK);

        // 3. Upload Tool
        let tool_payload = serde_json::json!({
            "id": "test_gripper",
            "tool_json": {
                "id": "test_gripper",
                "name": "Custom Test Gripper",
                "kind": "gripper",
                "tool_xml": "tool.xml"
            },
            "tool_xml": "<mujoco model=\"test_tool\"><worldbody><body name=\"test_finger\" pos=\"0 0 0\"><geom type=\"box\" size=\"0.01 0.01 0.01\"/></body></worldbody></mujoco>"
        });
        let req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/tools")
            .header("Content-Type", "application/json")
            .body(Body::from(serde_json::to_vec(&tool_payload).unwrap()))
            .unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::OK);

        // 4. Upload Policy
        let policy_payload = serde_json::json!({
            "id": "test_policy",
            "policy_json": {
                "id": "test_policy",
                "format": "rebot-onnx-policy/v1",
                "action_dim": 7,
                "inputs": {},
                "outputs": {},
                "model_file": "model.onnx"
            },
            "model_onnx_base64": "base64:T05OWFRFU1Q=" // ONNXTEST in base64
        });
        let req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/policies")
            .header("Content-Type", "application/json")
            .body(Body::from(serde_json::to_vec(&policy_payload).unwrap()))
            .unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::OK);

        // 5. Activate in WASM mode
        let activate_payload = serde_json::json!({
            "mode": "wasm",
            "scene": "test_station",
            "tool": "test_gripper",
            "policy": "test_policy"
        });
        let req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/activate")
            .header("Content-Type", "application/json")
            .body(Body::from(serde_json::to_vec(&activate_payload).unwrap()))
            .unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::OK);

        // 6. Bundle
        let req = axum::http::Request::builder().uri("/api/v1/sandbox/bundle").body(Body::empty()).unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::OK);
        let body = axum::body::to_bytes(res.into_body(), usize::MAX).await.unwrap();
        let bundle: serde_json::Value = serde_json::from_slice(&body).unwrap();
        assert_eq!(bundle["mode"], "wasm");
        assert_eq!(bundle["active_scene"], "test_station");
        assert!(bundle["composed_xml"].as_str().unwrap().contains("test_obj"));

        // 7. Reset
        let req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/reset")
            .header("Content-Type", "application/json")
            .body(Body::from(r#"{"seed": 999}"#))
            .unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::OK);

        // 8. IO
        let req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/io")
            .header("Content-Type", "application/json")
            .body(Body::from(r#"{"vacuum": true}"#))
            .unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::OK);

        // 9. Telemetry
        let req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/telemetry")
            .header("Content-Type", "application/json")
            .body(Body::from(r#"{"seq": 1, "sim_time_s": 0.05}"#))
            .unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::OK);

        // 10. State check
        let req = axum::http::Request::builder().uri("/api/v1/sandbox/state").body(Body::empty()).unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::OK);
        let body = axum::body::to_bytes(res.into_body(), usize::MAX).await.unwrap();
        let st: serde_json::Value = serde_json::from_slice(&body).unwrap();
        assert_eq!(st["mode"], "wasm");
        assert_eq!(st["seed"], 999);
        assert_eq!(st["desired_io"]["vacuum"], true);
        assert_eq!(st["telemetry"]["seq"], 1);

        // 11. File traversal protection
        let req = axum::http::Request::builder().uri("/api/v1/sandbox/scenes/test_station/files/../secret.json").body(Body::empty()).unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::NOT_FOUND);
    }

    #[tokio::test]
    async fn upload_policy_without_id_preserves_normalization_in_bundle() {
        let dir = std::env::temp_dir().join(format!("armd-sandbox-norm-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let state = Arc::new(SandboxState::new(dir));
        let supervisor = Arc::new(crate::supervisor::ArmSupervisor::new(crate::model::ArmConfig::default()));
        let app = axum::Router::new()
            .route("/api/v1/sandbox/policies", axum::routing::post(upload_policy))
            .route("/api/v1/sandbox/activate", axum::routing::post(activate))
            .route("/api/v1/sandbox/bundle", axum::routing::get(get_bundle))
            .layer(axum::Extension(state))
            .with_state(supervisor);

        let pol_payload = serde_json::json!({
            "policy_json": {
                "format": "rebot-onnx-policy/v1",
                "name": "norm_pol",
                "robot": "b601dm",
                "fps": 30,
                "action_dim": 7,
                "unit_mode": "rad",
                "model_file": "model.onnx",
                "inputs": { "state": { "name": "observation.state", "shape": [1, 7] } },
                "outputs": { "action": { "name": "action", "shape": [1, 7] } },
                "normalization": {
                    "state_mean": [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    "state_std": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
                }
            },
            "model_onnx_base64": "AQID"
        });
        let req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/policies")
            .header("Content-Type", "application/json")
            .body(Body::from(serde_json::to_vec(&pol_payload).unwrap()))
            .unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::OK);
        let up_body = axum::body::to_bytes(res.into_body(), usize::MAX).await.unwrap();
        let up_json: serde_json::Value = serde_json::from_slice(&up_body).unwrap();
        assert_eq!(up_json["id"], "norm_pol");

        let req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/activate")
            .header("Content-Type", "application/json")
            .body(Body::from(serde_json::to_vec(&serde_json::json!({
                "mode": "wasm",
                "scene": "pcb_suction",
                "tool": "suction",
                "policy": "norm_pol"
            })).unwrap()))
            .unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::OK);

        let req = axum::http::Request::builder().uri("/api/v1/sandbox/bundle").body(Body::empty()).unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::OK);
        let body = axum::body::to_bytes(res.into_body(), usize::MAX).await.unwrap();
        let bundle: serde_json::Value = serde_json::from_slice(&body).unwrap();
        assert_eq!(bundle["policy_manifest"]["id"], "norm_pol");
        assert_eq!(bundle["policy_manifest"]["unit_mode"], "rad");
        assert_eq!(bundle["policy_manifest"]["normalization"]["state_mean"][0], 0.1);
    }

    #[tokio::test]
    async fn delete_policy_removes_pack_and_refuses_the_active_policy() {
        let _guard = TEST_UPLOAD_MUTEX.lock().await;
        let tmp = tempfile::tempdir().unwrap();
        let state = Arc::new(SandboxState::new(tmp.path().to_path_buf()));
        let app = test_router(state.clone());
        let send = |method: &str, uri: &str, body: Option<serde_json::Value>| {
            let mut b = axum::http::Request::builder().method(method).uri(uri);
            if body.is_some() {
                b = b.header("Content-Type", "application/json");
            }
            b.body(body.map_or_else(Body::empty, |v| Body::from(serde_json::to_vec(&v).unwrap()))).unwrap()
        };
        async fn json_of(res: Response) -> (StatusCode, serde_json::Value) {
            let status = res.status();
            let body = axum::body::to_bytes(res.into_body(), usize::MAX).await.unwrap();
            (status, serde_json::from_slice(&body).unwrap_or(serde_json::Value::Null))
        }
        let upload = serde_json::json!({
            "id": "del_pol",
            "policy_json": {
                "format": "rebot-onnx-policy/v1", "name": "del_pol", "robot": "b601dm", "fps": 30, "action_dim": 7,
                "unit_mode": "rad", "model_file": "model.onnx",
                "inputs": {"state": {"name": "observation.state", "shape": [1, 7]}},
                "outputs": {"action": {"name": "action", "shape": [1, 7]}}
            },
            "model_onnx_base64": "AQID"
        });
        let (st, v) = json_of(app.clone().oneshot(send("POST", "/api/v1/sandbox/policies", Some(upload.clone()))).await.unwrap()).await;
        assert_eq!(st, StatusCode::OK, "{v}");

        // active sandbox policy: 409 and the pack stays
        state.active.write().await.active_policy = Some("del_pol".into());
        let (st, v) = json_of(app.clone().oneshot(send("DELETE", "/api/v1/sandbox/policies/del_pol", None)).await.unwrap()).await;
        assert_eq!(st, StatusCode::CONFLICT, "{v}");
        assert_eq!(v["code"], "POLICY_IN_USE");
        assert!(state.policy_dir("del_pol").is_some());

        state.active.write().await.active_policy = None;
        let (st, v) = json_of(app.clone().oneshot(send("DELETE", "/api/v1/sandbox/policies/del_pol", None)).await.unwrap()).await;
        assert_eq!(st, StatusCode::OK, "{v}");
        assert_eq!(v, serde_json::json!({"id": "del_pol", "deleted": true}));
        assert!(state.policy_dir("del_pol").is_none());
        let leftovers: Vec<_> = std::fs::read_dir(tmp.path().join("policies")).unwrap().flatten().map(|e| e.file_name()).collect();
        assert!(leftovers.is_empty(), "{leftovers:?}");
        let (_, cat) = json_of(app.clone().oneshot(send("GET", "/api/v1/sandbox/catalog", None)).await.unwrap()).await;
        assert_eq!(cat["policies"], serde_json::json!([]));
        let (st, _) = json_of(app.clone().oneshot(send("GET", "/api/v1/sandbox/policies/del_pol", None)).await.unwrap()).await;
        assert_eq!(st, StatusCode::NOT_FOUND);

        // again: 404; a bad id: 400
        let (st, v) = json_of(app.clone().oneshot(send("DELETE", "/api/v1/sandbox/policies/del_pol", None)).await.unwrap()).await;
        assert_eq!((st, v["code"].as_str()), (StatusCode::NOT_FOUND, Some("POLICY_NOT_FOUND")));
        let (st, _) = json_of(app.clone().oneshot(send("DELETE", "/api/v1/sandbox/policies/.hidden", None)).await.unwrap()).await;
        assert_eq!(st, StatusCode::BAD_REQUEST);

        // an interrupted delete (hidden trash dir) is not listed and is swept by the next delete
        let (st, _) = json_of(app.clone().oneshot(send("POST", "/api/v1/sandbox/policies", Some(upload))).await.unwrap()).await;
        assert_eq!(st, StatusCode::OK);
        std::fs::create_dir_all(tmp.path().join("policies/.deleted.old_pol.1.2.3")).unwrap();
        let (_, cat) = json_of(app.clone().oneshot(send("GET", "/api/v1/sandbox/catalog", None)).await.unwrap()).await;
        assert_eq!(cat["policies"].as_array().unwrap().len(), 1, "{cat}");
        let (st, _) = json_of(app.clone().oneshot(send("DELETE", "/api/v1/sandbox/policies/del_pol", None)).await.unwrap()).await;
        assert_eq!(st, StatusCode::OK);
        assert_eq!(std::fs::read_dir(tmp.path().join("policies")).unwrap().count(), 0);
    }

    #[tokio::test]
    async fn percent_encoded_path_traversal_is_rejected() {
        let tmp = tempfile::tempdir().unwrap();
        let state = Arc::new(SandboxState::new(tmp.path().to_path_buf()));
        let app = test_router(state);

        let attack_ids = ["..%2F..%2Fetc", "%2Fetc", "..%2Ffoo"];

        // 1. serve_*_file endpoints must return 404
        for bad_id in attack_ids {
            let uri = format!("/api/v1/sandbox/scenes/{bad_id}/files/world.xml");
            let req = axum::http::Request::builder().uri(&uri).body(Body::empty()).unwrap();
            let res = app.clone().oneshot(req).await.unwrap();
            assert_eq!(res.status(), StatusCode::NOT_FOUND, "serve_scene_file should 404 for {bad_id}");

            let uri = format!("/api/v1/sandbox/tools/{bad_id}/files/tool.xml");
            let req = axum::http::Request::builder().uri(&uri).body(Body::empty()).unwrap();
            let res = app.clone().oneshot(req).await.unwrap();
            assert_eq!(res.status(), StatusCode::NOT_FOUND, "serve_tool_file should 404 for {bad_id}");

            let uri = format!("/api/v1/sandbox/policies/{bad_id}/files/model.onnx");
            let req = axum::http::Request::builder().uri(&uri).body(Body::empty()).unwrap();
            let res = app.clone().oneshot(req).await.unwrap();
            assert_eq!(res.status(), StatusCode::NOT_FOUND, "serve_policy_file should 404 for {bad_id}");
        }

        // 2. get_* endpoints must reject traversal with 400 Bad Request
        for bad_id in attack_ids {
            let uri = format!("/api/v1/sandbox/scenes/{bad_id}");
            let req = axum::http::Request::builder().uri(&uri).body(Body::empty()).unwrap();
            let res = app.clone().oneshot(req).await.unwrap();
            assert_eq!(res.status(), StatusCode::BAD_REQUEST, "get_scene should 400 for {bad_id}");

            let uri = format!("/api/v1/sandbox/tools/{bad_id}");
            let req = axum::http::Request::builder().uri(&uri).body(Body::empty()).unwrap();
            let res = app.clone().oneshot(req).await.unwrap();
            assert_eq!(res.status(), StatusCode::BAD_REQUEST, "get_tool should 400 for {bad_id}");

            let uri = format!("/api/v1/sandbox/policies/{bad_id}");
            let req = axum::http::Request::builder().uri(&uri).body(Body::empty()).unwrap();
            let res = app.clone().oneshot(req).await.unwrap();
            assert_eq!(res.status(), StatusCode::BAD_REQUEST, "get_policy should 400 for {bad_id}");
        }

        // 3. activate endpoint must reject traversal with 400 Bad Request
        for bad_id in attack_ids {
            let req = axum::http::Request::builder()
                .method("POST")
                .uri("/api/v1/sandbox/activate")
                .header("Content-Type", "application/json")
                .body(Body::from(format!(r#"{{"mode":"wasm","scene":"{bad_id}"}}"#)))
                .unwrap();
            let res = app.clone().oneshot(req).await.unwrap();
            assert_eq!(res.status(), StatusCode::BAD_REQUEST, "activate should 400 for scene {bad_id}");

            let req = axum::http::Request::builder()
                .method("POST")
                .uri("/api/v1/sandbox/activate")
                .header("Content-Type", "application/json")
                .body(Body::from(format!(r#"{{"mode":"wasm","tool":"{bad_id}"}}"#)))
                .unwrap();
            let res = app.clone().oneshot(req).await.unwrap();
            assert_eq!(res.status(), StatusCode::BAD_REQUEST, "activate should 400 for tool {bad_id}");

            let req = axum::http::Request::builder()
                .method("POST")
                .uri("/api/v1/sandbox/activate")
                .header("Content-Type", "application/json")
                .body(Body::from(format!(r#"{{"mode":"wasm","policy":"{bad_id}"}}"#)))
                .unwrap();
            let res = app.clone().oneshot(req).await.unwrap();
            assert_eq!(res.status(), StatusCode::BAD_REQUEST, "activate should 400 for policy {bad_id}");
        }
    }

    #[tokio::test]
    async fn concurrent_upload_staging_race() {
        let tmp = tempfile::tempdir().unwrap();
        let state = Arc::new(SandboxState::new(tmp.path().to_path_buf()));

        let mut handles = Vec::new();
        for i in 0..10 {
            let target_dir = state.root.join("tools").join("race_tool");
            let handle = tokio::task::spawn_blocking(move || {
                let mut files = HashMap::new();
                let tool_json = serde_json::json!({
                    "id": "race_tool",
                    "name": format!("Race Tool Variant {i}"),
                    "kind": "gripper",
                    "tool_xml": "tool.xml"
                });
                files.insert("tool.json".to_string(), serde_json::to_vec(&tool_json).unwrap());
                files.insert(
                    "tool.xml".to_string(),
                    format!(r#"<mujoco model="race_tool_{i}"><worldbody><body name="finger_{i}"><geom type="box" size="0.01 0.01 0.01"/></body></worldbody></mujoco>"#).into_bytes(),
                );
                write_pack_atomically(&target_dir, &files, MAX_PACK_FILE_BYTES)
            });
            handles.push(handle);
        }

        for h in handles {
            let res = h.await.unwrap();
            assert!(res.is_ok(), "write_pack_atomically failed: {:?}", res);
        }

        // Pack must exist and resolve cleanly without 404 gap or corrupted files
        let resolved = state.resolve_tool("race_tool");
        assert!(resolved.is_some());
        let (m, xml) = resolved.unwrap();
        assert_eq!(m.id, "race_tool");
        assert!(xml.contains("race_tool_"));
    }

    #[tokio::test]
    async fn upload_concurrency_middleware_returns_429_when_busy() {
        let _guard = TEST_UPLOAD_MUTEX.lock().await;
        let tmp = tempfile::tempdir().unwrap();
        let state = Arc::new(SandboxState::new(tmp.path().to_path_buf()));
        let app = test_router(state);

        // Manually hold the upload permit to simulate an active in-flight upload
        let permit = upload_semaphore().try_acquire_owned().expect("Should acquire permit");

        let scene_payload = serde_json::json!({
            "id": "busy_scene",
            "scene_json": {
                "id": "busy_scene",
                "name": "Busy Scene",
                "world_xml": "world.xml"
            },
            "world_xml": r#"<mujoco model="busy"><worldbody/></mujoco>"#
        });
        let req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/scenes")
            .header("Content-Type", "application/json")
            .body(Body::from(serde_json::to_vec(&scene_payload).unwrap()))
            .unwrap();

        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::TOO_MANY_REQUESTS);

        let body = axum::body::to_bytes(res.into_body(), usize::MAX).await.unwrap();
        let err_json: serde_json::Value = serde_json::from_slice(&body).unwrap();
        assert_eq!(
            err_json["error"],
            "Another sandbox upload is already in progress"
        );

        // Verify tools upload route also yields 429 when permit is held
        let tool_payload = serde_json::json!({
            "id": "busy_tool",
            "tool_json": {
                "id": "busy_tool",
                "name": "Busy Tool",
                "kind": "gripper",
                "tool_xml": "tool.xml"
            },
            "tool_xml": r#"<mujoco model="busy_tool"><worldbody/></mujoco>"#
        });
        let tool_req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/tools")
            .header("Content-Type", "application/json")
            .body(Body::from(serde_json::to_vec(&tool_payload).unwrap()))
            .unwrap();
        let tool_res = app.clone().oneshot(tool_req).await.unwrap();
        assert_eq!(tool_res.status(), StatusCode::TOO_MANY_REQUESTS);

        // Verify policies upload route also yields 429 when permit is held
        let pol_payload = serde_json::json!({
            "id": "busy_policy",
            "policy_json": {
                "id": "busy_policy",
                "format": "rebot-onnx-policy/v1",
                "action_dim": 7,
                "model_file": "model.onnx"
            },
            "model_onnx_base64": "ABCD"
        });
        let pol_req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/policies")
            .header("Content-Type", "application/json")
            .body(Body::from(serde_json::to_vec(&pol_payload).unwrap()))
            .unwrap();
        let pol_res = app.clone().oneshot(pol_req).await.unwrap();
        assert_eq!(pol_res.status(), StatusCode::TOO_MANY_REQUESTS);

        // Release the permit
        drop(permit);

        // Subsequent upload must now succeed
        let req2 = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/scenes")
            .header("Content-Type", "application/json")
            .body(Body::from(serde_json::to_vec(&scene_payload).unwrap()))
            .unwrap();
        let res2 = app.oneshot(req2).await.unwrap();
        assert_eq!(res2.status(), StatusCode::OK);
    }

    #[tokio::test]
    async fn upload_permit_held_in_spawn_blocking_survives_client_disconnect() {
        let _guard = TEST_UPLOAD_MUTEX.lock().await;
        let tmp = tempfile::tempdir().unwrap();
        let state = Arc::new(SandboxState::new(tmp.path().to_path_buf()));

        let (started_tx, started_rx) = std::sync::mpsc::channel();
        let (unblock_tx, unblock_rx) = std::sync::mpsc::channel();
        let (finished_tx, finished_rx) = std::sync::mpsc::channel();

        let unblock_rx = Arc::new(std::sync::Mutex::new(unblock_rx));

        let slow_handler = {
            let started_tx = started_tx.clone();
            let unblock_rx = unblock_rx.clone();
            let finished_tx = finished_tx.clone();
            move |permit_slot: Option<Extension<UploadPermitSlot>>| {
                let started_tx = started_tx.clone();
                let unblock_rx = unblock_rx.clone();
                let finished_tx = finished_tx.clone();
                async move {
                    let permit = permit_slot
                        .and_then(|Extension(slot)| slot.0.lock().ok().and_then(|mut g| g.take()));
                    tokio::task::spawn_blocking(move || {
                        let _permit = permit;
                        let _ = started_tx.send(());
                        if let Ok(rx) = unblock_rx.lock() {
                            let _ = rx.recv();
                        }
                        let _ = finished_tx.send(());
                        StatusCode::OK
                    })
                    .await
                    .unwrap()
                }
            }
        };

        let app = test_router(state).route(
            "/api/v1/sandbox/test_slow",
            axum::routing::post(slow_handler)
                .layer(axum::middleware::from_fn(upload_concurrency_middleware)),
        );

        // 1. Issue the slow request wrapped in a spawned task (simulating an in-flight HTTP request)
        let slow_req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/test_slow")
            .body(Body::empty())
            .unwrap();

        let slow_app = app.clone();
        let slow_task = tokio::spawn(async move {
            slow_app.oneshot(slow_req).await
        });

        // 2. Wait until spawn_blocking has actually acquired the permit and started running
        tokio::task::spawn_blocking(move || {
            started_rx.recv().expect("blocking task should signal start");
        })
        .await
        .unwrap();

        // 3. Simulate client disconnect: abort and drop the request future
        slow_task.abort();
        let _ = slow_task.await;

        // 4. While spawn_blocking is still holding _permit in the background, send a second request
        // to a real production endpoint (/api/v1/sandbox/scenes). It MUST receive 429 TOO_MANY_REQUESTS!
        let scene_payload = serde_json::json!({
            "id": "scene_after_abort",
            "scene_json": {
                "id": "scene_after_abort",
                "name": "Scene After Abort",
                "world_xml": "world.xml"
            },
            "world_xml": r#"<mujoco model="scene_after_abort"><worldbody/></mujoco>"#
        });
        let req2 = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/scenes")
            .header("Content-Type", "application/json")
            .body(Body::from(serde_json::to_vec(&scene_payload).unwrap()))
            .unwrap();

        let res2 = app.clone().oneshot(req2).await.unwrap();
        assert_eq!(
            res2.status(),
            StatusCode::TOO_MANY_REQUESTS,
            "Second request must be rejected with 429 while disconnected request's spawn_blocking still holds permit"
        );

        // 5. Allow the background spawn_blocking task to finish
        unblock_tx.send(()).expect("should unblock spawn_blocking");
        tokio::task::spawn_blocking(move || {
            finished_rx.recv().expect("blocking task should finish");
        })
        .await
        .unwrap();

        // 6. Now that spawn_blocking has completed and dropped _permit, the next upload must succeed (200 OK)!
        let req3 = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/scenes")
            .header("Content-Type", "application/json")
            .body(Body::from(serde_json::to_vec(&scene_payload).unwrap()))
            .unwrap();

        let res3 = app.oneshot(req3).await.unwrap();
        assert_eq!(
            res3.status(),
            StatusCode::OK,
            "Upload request must succeed once spawn_blocking has dropped the permit"
        );
    }

    #[tokio::test]
    async fn upload_permit_released_immediately_on_early_rejection() {
        let _guard = TEST_UPLOAD_MUTEX.lock().await;
        let tmp = tempfile::tempdir().unwrap();
        let state = Arc::new(SandboxState::new(tmp.path().to_path_buf()));
        let app = test_router(state);

        // Send invalid JSON body -> rejected by axum Json extractor before upload handler runs
        let bad_req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/scenes")
            .header("Content-Type", "application/json")
            .body(Body::from(b"not-json-payload".to_vec()))
            .unwrap();

        let bad_res = app.clone().oneshot(bad_req).await.unwrap();
        assert!(
            bad_res.status() == StatusCode::BAD_REQUEST || bad_res.status() == StatusCode::UNPROCESSABLE_ENTITY,
            "Malformed JSON should be rejected before handler"
        );

        // Permit must have been dropped immediately with the rejected request, so next valid request succeeds
        let scene_payload = serde_json::json!({
            "id": "scene_after_bad_json",
            "scene_json": {
                "id": "scene_after_bad_json",
                "name": "Scene After Bad JSON",
                "world_xml": "world.xml"
            },
            "world_xml": r#"<mujoco model="scene_after_bad_json"><worldbody/></mujoco>"#
        });
        let good_req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/scenes")
            .header("Content-Type", "application/json")
            .body(Body::from(serde_json::to_vec(&scene_payload).unwrap()))
            .unwrap();

        let good_res = app.oneshot(good_req).await.unwrap();
        assert_eq!(good_res.status(), StatusCode::OK);
    }

    #[tokio::test]
    async fn policy_upload_over_2mb_is_not_blocked() {
        let _guard = TEST_UPLOAD_MUTEX.lock().await;
        let tmp = tempfile::tempdir().unwrap();
        let state = Arc::new(SandboxState::new(tmp.path().to_path_buf()));
        let app = test_router(state);

        // 2.5 MiB dummy model binary (exceeds default Axum 2 MiB body limit)
        let model_bytes = vec![0xABu8; 2_500_000];
        let mut b64 = String::with_capacity(model_bytes.len() * 4 / 3 + 4);
        const B64_CHARS: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        for chunk in model_bytes.chunks(3) {
            let b0 = chunk[0];
            let b1 = if chunk.len() > 1 { chunk[1] } else { 0 };
            let b2 = if chunk.len() > 2 { chunk[2] } else { 0 };
            b64.push(B64_CHARS[(b0 >> 2) as usize] as char);
            b64.push(B64_CHARS[(((b0 & 0x03) << 4) | (b1 >> 4)) as usize] as char);
            if chunk.len() > 1 {
                b64.push(B64_CHARS[(((b1 & 0x0F) << 2) | (b2 >> 6)) as usize] as char);
            } else {
                b64.push('=');
            }
            if chunk.len() > 2 {
                b64.push(B64_CHARS[(b2 & 0x3F) as usize] as char);
            } else {
                b64.push('=');
            }
        }

        let policy_payload = serde_json::json!({
            "id": "large_policy",
            "policy_json": {
                "id": "large_policy",
                "format": "rebot-onnx-policy/v1",
                "action_dim": 7,
                "inputs": {},
                "outputs": {},
                "model_file": "model.onnx"
            },
            "model_onnx_base64": b64
        });

        let req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/policies")
            .header("Content-Type", "application/json")
            .body(Body::from(serde_json::to_vec(&policy_payload).unwrap()))
            .unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::OK, "Upload >2 MiB policy should not be blocked by 413");

        // Verify file is served and size matches
        let req = axum::http::Request::builder()
            .uri("/api/v1/sandbox/policies/large_policy/files/model.onnx")
            .body(Body::empty())
            .unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::OK);
        let body = axum::body::to_bytes(res.into_body(), usize::MAX).await.unwrap();
        assert_eq!(body.len(), 2_500_000);
    }

    #[tokio::test]
    async fn activate_preserves_concurrent_reset_and_io_seq() {
        let tmp = tempfile::tempdir().unwrap();
        let state = Arc::new(SandboxState::new(tmp.path().to_path_buf()));
        let app = test_router(state.clone());

        // Increment reset_seq and set desired_io / io_seq
        {
            let mut st = state.active.write().await;
            st.reset_seq = 42;
            st.io_seq = 99;
            st.desired_io = serde_json::json!({"vacuum": true, "gripper_speed": 100});
            st.telemetry = Some(serde_json::json!({"seq": 5}));
        }

        // Activate in wasm mode
        let req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/activate")
            .header("Content-Type", "application/json")
            .body(Body::from(r#"{"mode":"wasm","scene":"pcb_suction","tool":"suction"}"#))
            .unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::OK);

        // Verify active state preserved concurrent reset_seq, io_seq, desired_io, and telemetry
        let st = state.active.read().await;
        assert_eq!(st.mode, "wasm");
        assert_eq!(st.active_scene, "pcb_suction");
        assert_eq!(st.active_tool, "suction");
        assert_eq!(st.reset_seq, 42, "reset_seq must not be clobbered by activate");
        assert_eq!(st.io_seq, 99, "io_seq must not be clobbered by activate");
        assert_eq!(st.desired_io["vacuum"], true, "desired_io must not be clobbered by activate");
        assert_eq!(st.telemetry.as_ref().unwrap()["seq"], 5, "telemetry must not be clobbered by activate");
    }

    #[tokio::test]
    async fn sim_upstream_error_propagation_and_timeout() {
        use axum::routing::post;
        use tokio::net::TcpListener;

        let mock_app = axum::Router::new()
            .route("/sim/scene", post(|| async {
                (StatusCode::INTERNAL_SERVER_ERROR, "sim scene failed")
            }))
            .route("/sim/reset", post(|| async {
                (StatusCode::SERVICE_UNAVAILABLE, "sim reset failed")
            }))
            .route("/sim/io", post(|| async {
                (StatusCode::BAD_REQUEST, "sim io failed")
            }))
            .route("/sim/state", axum::routing::get(|| async {
                (StatusCode::INTERNAL_SERVER_ERROR, "sim state failed")
            }));

        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        tokio::spawn(async move {
            axum::serve(listener, mock_app).await.unwrap();
        });

        // Point ARMD_SIM_URL to the mock server
        std::env::set_var("ARMD_SIM_URL", format!("http://127.0.0.1:{port}"));

        let tmp = tempfile::tempdir().unwrap();
        let state = Arc::new(SandboxState::new(tmp.path().to_path_buf()));
        let app = test_router(state.clone());

        // 1. Activate mode="sim" should propagate upstream non-success status
        let req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/activate")
            .header("Content-Type", "application/json")
            .body(Body::from(r#"{"mode":"sim","scene":"pcb_suction","tool":"suction"}"#))
            .unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::INTERNAL_SERVER_ERROR);

        // 2. Set state mode to sim for testing reset, set_io, get_state
        {
            let mut st = state.active.write().await;
            st.mode = "sim".to_string();
        }

        // 3. Reset in sim mode propagates upstream 503 error
        let req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/reset")
            .header("Content-Type", "application/json")
            .body(Body::from(r#"{"seed":123}"#))
            .unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::SERVICE_UNAVAILABLE);

        // 4. Set IO in sim mode propagates upstream 400 error
        let req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/io")
            .header("Content-Type", "application/json")
            .body(Body::from(r#"{"vacuum":true}"#))
            .unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::BAD_REQUEST);

        // 5. Unreachable upstream returns Bad Gateway
        std::env::set_var("ARMD_SIM_URL", "http://127.0.0.1:1"); // port 1 is unreachable
        let req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/reset")
            .header("Content-Type", "application/json")
            .body(Body::from(r#"{"seed":123}"#))
            .unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::BAD_GATEWAY);

        // Clean up env
        std::env::remove_var("ARMD_SIM_URL");

        // 6. Without ARMD_SIM_URL sim mode is refused (no fallback to the detector URL)
        let req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/activate")
            .header("Content-Type", "application/json")
            .body(Body::from(r#"{"mode":"sim","scene":"pcb_suction","tool":"suction"}"#))
            .unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::SERVICE_UNAVAILABLE);
        let body: serde_json::Value = serde_json::from_slice(&axum::body::to_bytes(res.into_body(), usize::MAX).await.unwrap()).unwrap();
        assert_eq!(body["code"], "SIM_NOT_CONFIGURED", "{body}");
        assert!(body["error"].as_str().unwrap().contains("ARMD_SIM_URL"), "{body}");
    }

    #[test]
    fn upload_body_limit_defaults_to_16_mib_and_reads_the_override() {
        assert_eq!(upload_body_limit_from(None), 16 * 1024 * 1024);
        assert_eq!(upload_body_limit_from(Some("")), 16 * 1024 * 1024);
        assert_eq!(upload_body_limit_from(Some("48")), 48 * 1024 * 1024);
        for bad in ["0", "-3", "1.5", "lots", "4096"] {
            assert_eq!(upload_body_limit_from(Some(bad)), DEFAULT_MAX_UPLOAD_BODY_BYTES, "{bad}");
        }
    }

    #[tokio::test]
    async fn upload_over_the_body_limit_is_413_from_content_length() {
        let _guard = TEST_UPLOAD_MUTEX.lock().await;
        let tmp = tempfile::tempdir().unwrap();
        let app = test_router(Arc::new(SandboxState::new(tmp.path().to_path_buf())));
        for path in ["/api/v1/sandbox/scenes", "/api/v1/sandbox/tools", "/api/v1/sandbox/policies"] {
            // declared length over the limit: refused before any of the body is read
            let req = axum::http::Request::builder()
                .method("POST")
                .uri(path)
                .header("Content-Type", "application/json")
                .header("Content-Length", (DEFAULT_MAX_UPLOAD_BODY_BYTES + 1).to_string())
                .body(Body::from("{}"))
                .unwrap();
            let res = app.clone().oneshot(req).await.unwrap();
            assert_eq!(res.status(), StatusCode::PAYLOAD_TOO_LARGE, "{path}");
            // no Content-Length (chunked): DefaultBodyLimit stops reading at the limit
            let big = vec![b' '; DEFAULT_MAX_UPLOAD_BODY_BYTES + 1];
            let stream = futures_util::stream::once(async move { Ok::<_, std::io::Error>(axum::body::Bytes::from(big)) });
            let req = axum::http::Request::builder()
                .method("POST")
                .uri(path)
                .header("Content-Type", "application/json")
                .body(Body::from_stream(stream))
                .unwrap();
            let res = app.clone().oneshot(req).await.unwrap();
            assert_eq!(res.status(), StatusCode::PAYLOAD_TOO_LARGE, "{path}");
        }
    }

    #[tokio::test]
    async fn set_io_limits_keys_and_values() {
        let tmp = tempfile::tempdir().unwrap();
        let state = Arc::new(SandboxState::new(tmp.path().to_path_buf()));
        let app = test_router(state.clone());
        let post = |body: serde_json::Value| {
            axum::http::Request::builder()
                .method("POST")
                .uri("/api/v1/sandbox/io")
                .header("Content-Type", "application/json")
                .body(Body::from(serde_json::to_vec(&body).unwrap()))
                .unwrap()
        };
        let long_key = "k".repeat(65);
        let big_value = "x".repeat(MAX_IO_VALUE_BYTES);
        for bad in [
            serde_json::json!({"": 1}),
            serde_json::json!({long_key: 1}),
            serde_json::json!({"has space": 1}),
            serde_json::json!({"slash/key": 1}),
            serde_json::json!({"ünicode": 1}),
            serde_json::json!({"big": big_value}),
        ] {
            let res = app.clone().oneshot(post(bad.clone())).await.unwrap();
            assert_eq!(res.status(), StatusCode::BAD_REQUEST, "{bad}");
        }
        // 4 KiB as JSON (with the quotes) is accepted; a 64-char key too
        let ok = serde_json::json!({"k".repeat(64): "x".repeat(MAX_IO_VALUE_BYTES - 2), "a.b-c_D9": true});
        assert_eq!(app.clone().oneshot(post(ok)).await.unwrap().status(), StatusCode::OK);
        // 64 keys in total: 62 more fit, the 65th is refused, and rewriting a key still works
        let more: serde_json::Map<String, serde_json::Value> = (0..62).map(|i| (format!("io{i}"), serde_json::json!(i))).collect();
        assert_eq!(app.clone().oneshot(post(serde_json::Value::Object(more))).await.unwrap().status(), StatusCode::OK);
        let res = app.clone().oneshot(post(serde_json::json!({"one_too_many": 1}))).await.unwrap();
        assert_eq!(res.status(), StatusCode::BAD_REQUEST);
        assert_eq!(app.clone().oneshot(post(serde_json::json!({"io0": 7}))).await.unwrap().status(), StatusCode::OK);
        let st = state.active.read().await;
        assert_eq!(st.desired_io.as_object().unwrap().len(), MAX_IO_KEYS);
        assert_eq!(st.desired_io["io0"], 7);
        assert!(st.desired_io.get("one_too_many").is_none());
    }

    #[tokio::test]
    async fn bundle_includes_pack_digests_and_asset_sha256() {
        let _guard = TEST_UPLOAD_MUTEX.lock().await;
        let tmp = tempfile::tempdir().unwrap();
        let empty_builtins = tempfile::tempdir().unwrap();
        let state = Arc::new(
            SandboxState::new(tmp.path().to_path_buf())
                .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
        );
        let app = test_router(state.clone());

        // 1. Stock virtual presets have null scene_digest and tool_digest
        let req = axum::http::Request::builder()
            .uri("/api/v1/sandbox/bundle")
            .body(Body::empty())
            .unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::OK);
        let body = axum::body::to_bytes(res.into_body(), usize::MAX).await.unwrap();
        let default_bundle: serde_json::Value = serde_json::from_slice(&body).unwrap();
        assert!(default_bundle["scene_digest"].is_null(), "{default_bundle}");
        assert!(default_bundle["tool_digest"].is_null(), "{default_bundle}");
        assert_eq!(
            default_bundle["composed_xml_sha256"].as_str(),
            Some(sha256_hex(default_bundle["composed_xml"].as_str().unwrap().as_bytes()).as_str()),
        );

        // 2. Base assets with local_exists == true include 64-char hex sha256 and size_bytes matching disk bytes
        let mut so101_cfg: crate::model::ArmConfig =
            serde_json::from_str(include_str!("../../profiles/so_arm101.json")).unwrap();
        so101_cfg
            .init_arm_model_in_memory(Some(include_str!("../../assets/so101_new_calib.urdf")))
            .unwrap();
        let so101_state = Arc::new(
            SandboxState::new(tmp.path().to_path_buf())
                .with_builtin_roots(vec![empty_builtins.path().to_path_buf()]),
        );
        let so101_app = test_router_with(
            so101_state,
            Arc::new(crate::supervisor::ArmSupervisor::new(so101_cfg)),
        );
        let req = axum::http::Request::builder()
            .uri("/api/v1/sandbox/bundle?robot=so-arm101")
            .body(Body::empty())
            .unwrap();
        let res = so101_app.oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::OK);
        let body = axum::body::to_bytes(res.into_body(), usize::MAX).await.unwrap();
        let so101_bundle: serde_json::Value = serde_json::from_slice(&body).unwrap();
        assert!(so101_bundle["scene_digest"].is_null());
        assert!(so101_bundle["tool_digest"].is_null());
        if let Some(asset_root) = base_assets_dir() {
            let base_assets = so101_bundle["base_assets"].as_array().unwrap();
            assert!(!base_assets.is_empty());
            for entry in base_assets {
                if entry["local_exists"] == true {
                    let rel = entry["path"].as_str().unwrap();
                    let disk_bytes = std::fs::read(asset_root.join(rel)).unwrap();
                    let expected = sha256_hex(&disk_bytes);
                    assert_eq!(entry["sha256"].as_str(), Some(expected.as_str()), "{entry}");
                    assert_eq!(entry["size_bytes"], disk_bytes.len() as u64, "{entry}");
                }
            }
        }

        // 3. Custom scene + tool packs return 64-char hex digests, composed_xml_sha256,
        // and include both declared and XML-referenced pack assets with sha256 + size_bytes
        let scene_stl = b"solid scene_prop\n  facet normal 0 0 1\n    outer loop\n      vertex 0 0 0\n      vertex 0.02 0 0\n      vertex 0 0.03 0\n    endloop\n  endfacet\nendsolid scene_prop\n";
        let scene_tex = b"\x89PNG\r\n\x1a\nfake_texture_payload_1234";
        let scene_tex_xml_only = b"\x89PNG\r\n\x1a\nxml_only_texture_payload_5678";
        let tool_stl = b"solid tool_jaw\n  facet normal 0 0 1\n    outer loop\n      vertex 0 0 0\n      vertex 0.01 0 0\n      vertex 0 0.04 0\n    endloop\n  endfacet\nendsolid tool_jaw\n";

        let encode_b64_obj = |bytes: &[u8]| -> serde_json::Value {
            const B64: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
            let mut s = String::with_capacity(bytes.len() * 4 / 3 + 4);
            for chunk in bytes.chunks(3) {
                let b0 = chunk[0];
                let b1 = if chunk.len() > 1 { chunk[1] } else { 0 };
                let b2 = if chunk.len() > 2 { chunk[2] } else { 0 };
                s.push(B64[(b0 >> 2) as usize] as char);
                s.push(B64[(((b0 & 0x03) << 4) | (b1 >> 4)) as usize] as char);
                s.push(if chunk.len() > 1 { B64[(((b1 & 0x0F) << 2) | (b2 >> 6)) as usize] as char } else { '=' });
                s.push(if chunk.len() > 2 { B64[(b2 & 0x3F) as usize] as char } else { '=' });
            }
            serde_json::json!({ "encoding": "base64", "data": s })
        };

        let scene_payload = serde_json::json!({
            "id": "digest_scene",
            "scene_json": {
                "id": "digest_scene",
                "name": "Digest Scene",
                "world_xml": "world.xml",
                "robots": ["b601dm"],
                "assets": [
                    "meshes/prop.stl",
                    {"target": "textures/tex.png"},
                    {"path": "meshes/absent_opt.stl", "required": false}
                ]
            },
            "world_xml": r#"<mujoco model="digest_world"><compiler meshdir="meshes"/><asset><mesh name="prop" file="prop.stl"/><texture name="tex2" type="2d" file="textures/tex_xml_only.png"/></asset><worldbody><body name="part" pos="0.25 0 0.02"><freejoint/><geom type="mesh" mesh="prop"/></body></worldbody></mujoco>"#,
            "assets": {
                "meshes/prop.stl": encode_b64_obj(scene_stl),
                "textures/tex.png": encode_b64_obj(scene_tex),
                "textures/tex_xml_only.png": encode_b64_obj(scene_tex_xml_only)
            }
        });
        let req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/scenes")
            .header("Content-Type", "application/json")
            .body(Body::from(serde_json::to_vec(&scene_payload).unwrap()))
            .unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::OK);
        let scene_up: serde_json::Value = serde_json::from_slice(
            &axum::body::to_bytes(res.into_body(), usize::MAX).await.unwrap(),
        )
        .unwrap();
        let expected_scene_digest = scene_up["pack_digest"].as_str().unwrap().to_string();
        assert_eq!(expected_scene_digest.len(), 64);

        let tool_payload = serde_json::json!({
            "id": "digest_tool",
            "tool_json": {
                "id": "digest_tool",
                "name": "Digest Tool",
                "kind": "gripper",
                "tool_xml": "tool.xml",
                "robots": ["b601dm"],
                "assets": []
            },
            "tool_xml": r#"<mujoco model="digest_tool"><compiler meshdir="meshes"/><asset><mesh name="jaw" file="jaw.stl"/></asset><worldbody><geom name="jaw_geom" type="mesh" mesh="jaw"/></worldbody></mujoco>"#,
            "assets": {
                "meshes/jaw.stl": encode_b64_obj(tool_stl)
            }
        });
        let req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/tools")
            .header("Content-Type", "application/json")
            .body(Body::from(serde_json::to_vec(&tool_payload).unwrap()))
            .unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::OK);
        let tool_up: serde_json::Value = serde_json::from_slice(
            &axum::body::to_bytes(res.into_body(), usize::MAX).await.unwrap(),
        )
        .unwrap();
        let expected_tool_digest = tool_up["pack_digest"].as_str().unwrap().to_string();
        assert_eq!(expected_tool_digest.len(), 64);

        let req = axum::http::Request::builder()
            .method("POST")
            .uri("/api/v1/sandbox/activate")
            .header("Content-Type", "application/json")
            .body(Body::from(
                serde_json::to_vec(&serde_json::json!({
                    "mode": "wasm",
                    "scene": "digest_scene",
                    "tool": "digest_tool"
                }))
                .unwrap(),
            ))
            .unwrap();
        let res = app.clone().oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::OK);

        let req = axum::http::Request::builder()
            .uri("/api/v1/sandbox/bundle")
            .body(Body::empty())
            .unwrap();
        let res = app.oneshot(req).await.unwrap();
        assert_eq!(res.status(), StatusCode::OK);
        let bundle: serde_json::Value = serde_json::from_slice(
            &axum::body::to_bytes(res.into_body(), usize::MAX).await.unwrap(),
        )
        .unwrap();

        assert_eq!(bundle["scene_digest"].as_str(), Some(expected_scene_digest.as_str()));
        assert_eq!(bundle["tool_digest"].as_str(), Some(expected_tool_digest.as_str()));
        assert_eq!(
            bundle["composed_xml_sha256"].as_str(),
            Some(sha256_hex(bundle["composed_xml"].as_str().unwrap().as_bytes()).as_str()),
        );

        let scene_assets = bundle["scene_manifest"]["assets"].as_array().unwrap();
        assert_eq!(scene_assets.len(), 3, "{scene_assets:?}");
        assert_eq!(scene_assets[0]["path"], "meshes/prop.stl");
        assert_eq!(scene_assets[0]["sha256"].as_str(), Some(sha256_hex(scene_stl).as_str()));
        assert_eq!(scene_assets[0]["size_bytes"], scene_stl.len() as u64);
        assert_eq!(scene_assets[1]["path"], "textures/tex.png");
        assert_eq!(scene_assets[1]["sha256"].as_str(), Some(sha256_hex(scene_tex).as_str()));
        assert_eq!(scene_assets[1]["size_bytes"], scene_tex.len() as u64);
        assert_eq!(scene_assets[2]["path"], "textures/tex_xml_only.png");
        assert_eq!(
            scene_assets[2]["sha256"].as_str(),
            Some(sha256_hex(scene_tex_xml_only).as_str()),
        );
        assert_eq!(scene_assets[2]["size_bytes"], scene_tex_xml_only.len() as u64);

        let tool_assets = bundle["tool_manifest"]["assets"].as_array().unwrap();
        assert_eq!(tool_assets.len(), 1, "{tool_assets:?}");
        assert_eq!(tool_assets[0]["path"], "meshes/jaw.stl");
        assert_eq!(tool_assets[0]["sha256"].as_str(), Some(sha256_hex(tool_stl).as_str()));
        assert_eq!(tool_assets[0]["size_bytes"], tool_stl.len() as u64);
    }

    #[test]
    fn pack_stat_fingerprint_tracks_extra_files_and_ctime_with_preserved_mtime() {
        let tmp = tempfile::tempdir().unwrap();
        let pack_dir = tmp.path().join("pack");
        std::fs::create_dir_all(&pack_dir).unwrap();
        let present_mesh = tmp.path().join("present_mesh.stl");
        let missing_mesh = tmp.path().join("missing_mesh.stl");
        std::fs::write(&present_mesh, b"AAAA").unwrap();
        let orig_mtime = std::fs::metadata(&present_mesh).unwrap().modified().unwrap();

        let fp = PackStatFingerprint::collect(
            vec![pack_dir.clone()],
            Vec::new(),
            vec![present_mesh.clone(), missing_mesh.clone()],
        );
        assert!(fp.matches_disk(), "fresh fingerprint must match disk");

        // In-place overwrite preserving byte length and mtime still changes ctime.
        std::thread::sleep(Duration::from_millis(5));
        {
            use std::io::Write;
            let mut f = std::fs::OpenOptions::new()
                .write(true)
                .truncate(true)
                .open(&present_mesh)
                .unwrap();
            f.write_all(b"BBBB").unwrap();
            f.set_modified(orig_mtime).unwrap();
            f.sync_all().unwrap();
        }
        assert_eq!(
            std::fs::metadata(&present_mesh).unwrap().modified().unwrap(),
            orig_mtime
        );
        assert!(
            !fp.matches_disk(),
            "in-place same-len mtime-preserving update on extra_files must invalidate fingerprint via ctime"
        );

        // Creating a previously missing extra_file also invalidates the fingerprint.
        let fp2 = PackStatFingerprint::collect(
            vec![pack_dir],
            Vec::new(),
            vec![present_mesh, missing_mesh.clone()],
        );
        assert!(fp2.matches_disk());
        std::fs::write(&missing_mesh, b"NEW").unwrap();
        assert!(
            !fp2.matches_disk(),
            "creating a previously missing extra_file must invalidate fingerprint"
        );
    }

    #[cfg(unix)]
    #[test]
    fn pack_stat_fingerprint_watches_symlink_and_canonical_target_for_base_xml() {
        let tmp = tempfile::tempdir().unwrap();
        let pack_dir = tmp.path().join("pack");
        std::fs::create_dir_all(&pack_dir).unwrap();
        let target_xml = tmp.path().join("target_base.xml");
        let link_xml = tmp.path().join("link_base.xml");
        std::fs::write(&target_xml, b"<mujoco><worldbody/></mujoco>").unwrap();
        std::os::unix::fs::symlink(&target_xml, &link_xml).unwrap();

        let mut watch_paths = Vec::new();
        let mut seen = HashSet::new();
        push_watch_path_with_canonical(&mut watch_paths, &mut seen, link_xml.clone());
        assert_eq!(watch_paths.len(), 2);
        assert!(watch_paths.contains(&link_xml));
        assert!(watch_paths.contains(&target_xml.canonicalize().unwrap()));

        let fp = PackStatFingerprint::collect(vec![pack_dir], Vec::new(), watch_paths);
        assert!(fp.matches_disk());

        std::thread::sleep(Duration::from_millis(5));
        std::fs::write(
            &target_xml,
            b"<mujoco><worldbody><site name=\"edited\"/></worldbody></mujoco>",
        )
        .unwrap();
        assert!(
            !fp.matches_disk(),
            "editing canonical symlink target must invalidate fingerprint"
        );
    }
}
