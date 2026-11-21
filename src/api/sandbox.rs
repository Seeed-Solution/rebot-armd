//! Sandbox REST API and simulation pack management for armd.
//!
//! Provides scene, tool, and policy management, runtime MJCF composition, and
//! state/telemetry bridging for MuJoCo WASM and rebot-sim.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
pub use crate::mjcf_compose::*;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

use axum::{
    body::{Body, Bytes},
    extract::{Extension, Path as UrlPath},
    http::{header, Method, StatusCode},
    response::{IntoResponse, Response},
    Json,
};
use serde::{Deserialize, Serialize};
use tokio::sync::{OwnedSemaphorePermit, RwLock, Semaphore};

use super::assets;
use super::proxy::{self, DetectorUpstream};

static STAGING_COUNTER: AtomicU64 = AtomicU64::new(0);
static TARGET_REPLACE_MUTEX: std::sync::Mutex<()> = std::sync::Mutex::new(());

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


pub fn is_reserved_builtin_id(kind: &str, id: &str) -> bool {
    if id == "default" {
        return true;
    }
    match kind {
        "scene" => matches!(id, "coffee_kiosk" | "pcb_suction"),
        "tool" => matches!(id, "wide_cup_gripper" | "suction" | "screwdriver"),
        _ => false,
    }
}

pub fn search_builtin_roots() -> Vec<PathBuf> {
    let mut roots = Vec::new();
    if let Ok(p) = std::env::var("ARMD_BUILTIN_SCENES_DIR") {
        roots.push(PathBuf::from(p));
    }
    roots.push(PathBuf::from("../rebot-companion/rebot_companion/scenes/builtin"));
    roots.push(PathBuf::from("rebot_companion/scenes/builtin"));
    roots.push(PathBuf::from("/usr/share/rebot-teleop/scenes/builtin"));
    roots
}

pub fn load_builtin_scene(id: &str) -> Option<(SceneManifest, String)> {
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

    // Check on-disk builtin scenes directory first so full scene manifests (e.g., coffee_kiosk, pcb_suction) stay authoritative.
    for root in search_builtin_roots() {
        let scene_dir = root.join("scenes").join(id);
        let scene_json_path = scene_dir.join("scene.json");
        if scene_json_path.is_file() {
            if let Ok(raw_json) = std::fs::read_to_string(&scene_json_path) {
                if let Ok(manifest) = serde_json::from_str::<SceneManifest>(&raw_json) {
                    let world_path = scene_dir.join(&manifest.world_xml);
                    if let Ok(world_xml) = std::fs::read_to_string(&world_path) {
                        return Some((manifest, world_xml));
                    }
                }
            }
        }
    }

    if id == "pcb_suction" {
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

pub fn load_builtin_tool(id: &str) -> Option<(ToolManifest, String)> {
    if !validate_pack_id(id) { return None; }
    if let Some((manifest, xml)) = builtin_tool_preset(id) {
        return Some((manifest, xml.into()));
    }

    // Try finding directory under search roots
    for root in search_builtin_roots() {
        let tool_dir = root.join("tools").join(id);
        let tool_json_path = tool_dir.join("tool.json");
        if tool_json_path.is_file() {
            if let Ok(raw_json) = std::fs::read_to_string(&tool_json_path) {
                if let Ok(manifest) = serde_json::from_str::<ToolManifest>(&raw_json) {
                    let tool_path = tool_dir.join(&manifest.tool_xml);
                    if let Ok(tool_xml) = std::fs::read_to_string(&tool_path) {
                        return Some((manifest, tool_xml));
                    }
                }
            }
        }
    }

    None
}

pub fn get_base_robot_xml() -> String {
    if let Ok(path) = std::env::var("ARMD_SANDBOX_BASE_XML") {
        if let Ok(content) = std::fs::read_to_string(&path) {
            return content;
        }
    }
    if let Ok(content) = std::fs::read_to_string("./sim/mujoco/models/rebot_b601dm_scene.xml") {
        return content;
    }
    FALLBACK_BASE_XML.to_string()
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
        }
    }
}

pub struct SandboxState {
    pub root: PathBuf,
    pub active: RwLock<ActiveState>,
    pub cached_mjcf: RwLock<String>,
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
            cached_mjcf: RwLock::new(String::new()),
            #[cfg(feature = "policy-runtime")]
            policy_load_delay_ms: AtomicU64::new(0),
            #[cfg(feature = "policy-runtime")]
            policy_infer_delay_ms: AtomicU64::new(0),
        }
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

    pub fn resolve_scene(&self, id: &str) -> Option<(SceneManifest, String)> {
        if !validate_pack_id(id) {
            return None;
        }
        if let Some(builtin) = load_builtin_scene(id) {
            return Some(builtin);
        }
        let dir = self.root.join("scenes").join(id);
        let manifest_path = dir.join("scene.json");
        let manifest_str = std::fs::read_to_string(&manifest_path).ok()?;
        let manifest: SceneManifest = serde_json::from_str(&manifest_str).ok()?;
        let world_path = dir.join(&manifest.world_xml);
        let world_xml = std::fs::read_to_string(&world_path).ok()?;
        Some((manifest, world_xml))
    }

    pub fn resolve_tool(&self, id: &str) -> Option<(ToolManifest, String)> {
        if !validate_pack_id(id) {
            return None;
        }
        if let Some(builtin) = load_builtin_tool(id) {
            return Some(builtin);
        }
        let dir = self.root.join("tools").join(id);
        let manifest_path = dir.join("tool.json");
        let manifest_str = std::fs::read_to_string(&manifest_path).ok()?;
        let manifest: ToolManifest = serde_json::from_str(&manifest_str).ok()?;
        let tool_path = dir.join(&manifest.tool_xml);
        let tool_xml = std::fs::read_to_string(&tool_path).ok()?;
        Some((manifest, tool_xml))
    }

    pub fn resolve_policy(&self, id: &str) -> Option<PolicyManifest> {
        if !validate_pack_id(id) {
            return None;
        }
        let dir = self.root.join("policies").join(id);
        let manifest_path = dir.join("policy.json");
        let manifest_str = std::fs::read_to_string(&manifest_path).ok()?;
        let mut m: PolicyManifest = serde_json::from_str(&manifest_str).ok()?;
        if m.id.is_empty() {
            m.id = id.to_string();
        }
        if m.name.is_empty() {
            m.name = id.to_string();
        }
        Some(m)
    }

    pub fn policy_dir(&self, id: &str) -> Option<PathBuf> {
        if !validate_pack_id(id) {
            return None;
        }
        let dir = self.root.join("policies").join(id);
        dir.is_dir().then_some(dir)
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

/// GET /api/v1/sandbox/catalog
pub async fn get_catalog(Extension(state): Extension<Arc<SandboxState>>) -> Json<serde_json::Value> {
    let active = state.active.read().await;

    let mut scenes_out = vec![
        serde_json::json!({"id": "default", "name": "Default Worktable Scene", "builtin": true}),
        serde_json::json!({"id": "coffee_kiosk", "name": "Mobile Coffee Kiosk", "builtin": true}),
        serde_json::json!({"id": "pcb_suction", "name": "PCB Suction Handling", "builtin": true}),
    ];

    if let Ok(entries) = std::fs::read_dir(state.root.join("scenes")) {
        for e in entries.flatten() {
            if let Ok(ft) = e.file_type() {
                if ft.is_dir() {
                    let id = e.file_name().to_string_lossy().to_string();
                    if !scenes_out.iter().any(|s| s["id"] == id) {
                        if let Some((m, _)) = state.resolve_scene(&id) {
                            scenes_out.push(serde_json::json!({
                                "id": m.id,
                                "name": m.name,
                                "version": m.version,
                                "builtin": false,
                            }));
                        }
                    }
                }
            }
        }
    }

    let mut tools_out = vec![
        serde_json::json!({"id": "default", "name": "Stock Parallel Jaw Gripper", "kind": "gripper", "builtin": true}),
        serde_json::json!({"id": "wide_cup_gripper", "name": "Wide Silicone Cup Gripper", "kind": "gripper", "builtin": true}),
        serde_json::json!({"id": "suction", "name": "Vacuum Suction End-Effector", "kind": "suction", "builtin": true}),
        serde_json::json!({"id": "screwdriver", "name": "Spindle Electric Screwdriver", "kind": "screwdriver", "builtin": true}),
    ];

    if let Ok(entries) = std::fs::read_dir(state.root.join("tools")) {
        for e in entries.flatten() {
            if let Ok(ft) = e.file_type() {
                if ft.is_dir() {
                    let id = e.file_name().to_string_lossy().to_string();
                    if !tools_out.iter().any(|t| t["id"] == id) {
                        if let Some((m, _)) = state.resolve_tool(&id) {
                            tools_out.push(serde_json::json!({
                                "id": m.id,
                                "name": m.name,
                                "kind": m.kind,
                                "version": m.version,
                                "builtin": false,
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
                            "action_dim": m.action_dim,
                        }));
                    }
                }
            }
        }
    }

    Json(serde_json::json!({
        "mode": active.mode,
        "active_scene": active.active_scene,
        "active_tool": active.active_tool,
        "active_policy": active.active_policy,
        "wasm_runtime_ready": assets::wasm_runtime_ready(),
        "builtin_scenes": ["default", "coffee_kiosk", "pcb_suction"],
        "builtin_tools": ["default", "wide_cup_gripper", "suction", "screwdriver"],
        "scenes": scenes_out,
        "tools": tools_out,
        "policies": policies_out,
    }))
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

/// POST /api/v1/sandbox/scenes
pub async fn upload_scene(
    Extension(state): Extension<Arc<SandboxState>>,
    permit_slot: Option<Extension<UploadPermitSlot>>,
    Json(payload): Json<UploadSceneRequest>,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<serde_json::Value>)> {
    let root = state.root.clone();
    let permit = permit_slot.and_then(|Extension(slot)| slot.0.lock().ok().and_then(|mut g| g.take()));
    let res = tokio::task::spawn_blocking(move || -> Result<serde_json::Value, (StatusCode, String)> {
        let _permit = permit;
        let primary_json = payload.scene_json.as_ref().map(|v| ("scene.json", v));
        let primary_xml = payload.world_xml.as_deref().map(|v| ("world.xml", v));

        let file_map = extract_files_map(payload.files, primary_json, primary_xml, payload.assets)?;

        let scene_json_bytes = file_map
            .get("scene.json")
            .ok_or_else(|| (StatusCode::BAD_REQUEST, "Missing scene.json".to_string()))?;

        let manifest: SceneManifest = serde_json::from_slice(scene_json_bytes)
            .map_err(|e| (StatusCode::BAD_REQUEST, format!("Invalid scene.json: {e}")))?;

        let pack_id = payload.id.unwrap_or(manifest.id.clone());
        if !validate_pack_id(&pack_id) {
            return Err((StatusCode::BAD_REQUEST, format!("Invalid scene pack id '{pack_id}'")));
        }
        if is_reserved_builtin_id("scene", &pack_id) {
            return Err((StatusCode::BAD_REQUEST, format!("Cannot overwrite reserved builtin scene '{pack_id}'")));
        }

        let world_xml_bytes = file_map
            .get(&manifest.world_xml)
            .ok_or_else(|| (StatusCode::BAD_REQUEST, format!("Missing world XML file '{}'", manifest.world_xml)))?;

        let world_xml_str = std::str::from_utf8(world_xml_bytes)
            .map_err(|_| (StatusCode::BAD_REQUEST, "World XML is not valid UTF-8".to_string()))?;

        validate_xml_security(world_xml_str, Some("worldbody"))
            .map_err(|e| (StatusCode::BAD_REQUEST, e))?;

        let target_dir = root.join("scenes").join(&pack_id);
        let total_pack_bytes: u64 = file_map.values().map(|v| v.len() as u64).sum();
        enforce_sandbox_quota(
            &root,
            "scenes",
            &target_dir,
            total_pack_bytes,
            MAX_CUSTOM_PACKS_PER_KIND,
            MAX_SANDBOX_STORAGE_BYTES,
        )?;

        let digest = write_pack_atomically(&target_dir, &file_map, MAX_PACK_FILE_BYTES)
            .map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, e))?;

        let mut file_names: Vec<String> = file_map.keys().cloned().collect();
        file_names.sort();

        Ok(serde_json::json!({
            "id": pack_id,
            "sha256": digest,
            "path": target_dir.to_string_lossy(),
            "manifest": manifest,
            "files": file_names,
        }))
    })
    .await
    .map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, Json(serde_json::json!({"error": format!("Blocking task failed: {e}")}))))?;

    match res {
        Ok(val) => Ok(Json(val)),
        Err((sc, msg)) => Err((sc, Json(serde_json::json!({"error": msg})))),
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
    let root = state.root.clone();
    let permit = permit_slot.and_then(|Extension(slot)| slot.0.lock().ok().and_then(|mut g| g.take()));
    let res = tokio::task::spawn_blocking(move || -> Result<serde_json::Value, (StatusCode, String)> {
        let _permit = permit;
        let primary_json = payload.tool_json.as_ref().map(|v| ("tool.json", v));
        let primary_xml = payload.tool_xml.as_deref().map(|v| ("tool.xml", v));

        let file_map = extract_files_map(payload.files, primary_json, primary_xml, payload.assets)?;

        let tool_json_bytes = file_map
            .get("tool.json")
            .ok_or_else(|| (StatusCode::BAD_REQUEST, "Missing tool.json".to_string()))?;

        let manifest: ToolManifest = serde_json::from_slice(tool_json_bytes)
            .map_err(|e| (StatusCode::BAD_REQUEST, format!("Invalid tool.json: {e}")))?;

        let pack_id = payload.id.unwrap_or(manifest.id.clone());
        if !validate_pack_id(&pack_id) {
            return Err((StatusCode::BAD_REQUEST, format!("Invalid tool pack id '{pack_id}'")));
        }
        if is_reserved_builtin_id("tool", &pack_id) {
            return Err((StatusCode::BAD_REQUEST, format!("Cannot overwrite reserved builtin tool '{pack_id}'")));
        }

        let tool_xml_bytes = file_map
            .get(&manifest.tool_xml)
            .ok_or_else(|| (StatusCode::BAD_REQUEST, format!("Missing tool XML file '{}'", manifest.tool_xml)))?;

        let tool_xml_str = std::str::from_utf8(tool_xml_bytes)
            .map_err(|_| (StatusCode::BAD_REQUEST, "Tool XML is not valid UTF-8".to_string()))?;

        validate_xml_security(tool_xml_str, None)
            .map_err(|e| (StatusCode::BAD_REQUEST, e))?;

        let target_dir = root.join("tools").join(&pack_id);
        let total_pack_bytes: u64 = file_map.values().map(|v| v.len() as u64).sum();
        enforce_sandbox_quota(
            &root,
            "tools",
            &target_dir,
            total_pack_bytes,
            MAX_CUSTOM_PACKS_PER_KIND,
            MAX_SANDBOX_STORAGE_BYTES,
        )?;

        let digest = write_pack_atomically(&target_dir, &file_map, MAX_PACK_FILE_BYTES)
            .map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, e))?;

        let mut file_names: Vec<String> = file_map.keys().cloned().collect();
        file_names.sort();

        Ok(serde_json::json!({
            "id": pack_id,
            "sha256": digest,
            "path": target_dir.to_string_lossy(),
            "manifest": manifest,
            "files": file_names,
        }))
    })
    .await
    .map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, Json(serde_json::json!({"error": format!("Blocking task failed: {e}")}))))?;

    match res {
        Ok(val) => Ok(Json(val)),
        Err((sc, msg)) => Err((sc, Json(serde_json::json!({"error": msg})))),
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
        Ok(val) => Ok(Json(val)),
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
    // Lock order as in the policy runner: supervisor, then the sandbox state. Both are held
    // across the rename, so no run can start on the pack and no activation can select it.
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
    if !validate_pack_id(&id) {
        return Err((StatusCode::BAD_REQUEST, Json(serde_json::json!({"error": format!("Invalid scene pack id '{id}'")}))));
    }
    if let Some((manifest, _)) = state.resolve_scene(&id) {
        let dir = state.root.join("scenes").join(&id);
        let mut files = Vec::new();
        if dir.is_dir() {
            if let Ok(entries) = std::fs::read_dir(&dir) {
                for e in entries.flatten() {
                    files.push(e.file_name().to_string_lossy().to_string());
                }
            }
        } else {
            files.push("scene.json".into());
            files.push(manifest.world_xml.clone());
        }
        files.sort();
        Ok(Json(serde_json::json!({
            "id": manifest.id,
            "manifest": manifest,
            "path": dir.to_string_lossy(),
            "files": files,
        })))
    } else {
        Err((StatusCode::NOT_FOUND, Json(serde_json::json!({"error": format!("Scene pack '{id}' not found")}))))
    }
}

/// GET /api/v1/sandbox/tools/{id}
pub async fn get_tool(
    Extension(state): Extension<Arc<SandboxState>>,
    UrlPath(id): UrlPath<String>,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<serde_json::Value>)> {
    if !validate_pack_id(&id) {
        return Err((StatusCode::BAD_REQUEST, Json(serde_json::json!({"error": format!("Invalid tool pack id '{id}'")}))));
    }
    if let Some((manifest, _)) = state.resolve_tool(&id) {
        let dir = state.root.join("tools").join(&id);
        let mut files = Vec::new();
        if dir.is_dir() {
            if let Ok(entries) = std::fs::read_dir(&dir) {
                for e in entries.flatten() {
                    files.push(e.file_name().to_string_lossy().to_string());
                }
            }
        } else {
            files.push("tool.json".into());
            files.push(manifest.tool_xml.clone());
        }
        files.sort();
        Ok(Json(serde_json::json!({
            "id": manifest.id,
            "manifest": manifest,
            "path": dir.to_string_lossy(),
            "files": files,
        })))
    } else {
        Err((StatusCode::NOT_FOUND, Json(serde_json::json!({"error": format!("Tool pack '{id}' not found")}))))
    }
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
    let dir = state.root.join("scenes").join(&id);
    if dir.is_dir() {
        return serve_file_from_dir(&state.root, &dir, &rel).await;
    }
    // Search built-in roots if not found in custom storage
    for root in search_builtin_roots() {
        let bdir = root.join("scenes").join(&id);
        if bdir.is_dir() {
            return serve_file_from_dir(&root, &bdir, &rel).await;
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
    let dir = state.root.join("tools").join(&id);
    if dir.is_dir() {
        return serve_file_from_dir(&state.root, &dir, &rel).await;
    }
    for root in search_builtin_roots() {
        let bdir = root.join("tools").join(&id);
        if bdir.is_dir() {
            return serve_file_from_dir(&root, &bdir, &rel).await;
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
    let dir = state.root.join("policies").join(&id);
    serve_file_from_dir(&state.root, &dir, &rel).await
}

// ---------------------------------------------------------------------------
// Activation, State, Reset, IO, Telemetry
// ---------------------------------------------------------------------------

#[derive(Debug, Deserialize)]
pub struct ActivateRequest {
    pub mode: String, // "none" | "wasm" | "sim"
    #[serde(default)]
    pub scene: Option<String>,
    #[serde(default)]
    pub tool: Option<String>,
    #[serde(default)]
    pub policy: Option<Option<String>>,
    #[serde(default)]
    pub seed: Option<u64>,
    #[serde(default)]
    pub auto_start_policy: Option<bool>,
}

/// POST /api/v1/sandbox/activate
pub async fn activate(
    Extension(state): Extension<Arc<SandboxState>>,
    Json(payload): Json<ActivateRequest>,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<serde_json::Value>)> {
    let mode = payload.mode.to_ascii_lowercase();
    if !matches!(mode.as_str(), "none" | "wasm" | "sim") {
        return Err((StatusCode::BAD_REQUEST, Json(serde_json::json!({"error": format!("Invalid mode '{mode}', must be none, wasm, or sim")}))));
    }

    let (cur_scene, cur_tool, cur_policy, cur_seed) = {
        let cur = state.active.read().await;
        (
            cur.active_scene.clone(),
            cur.active_tool.clone(),
            cur.active_policy.clone(),
            cur.seed,
        )
    };

    let scene_id = payload.scene.unwrap_or(cur_scene);
    let tool_id = payload.tool.unwrap_or(cur_tool);

    if !validate_pack_id(&scene_id) {
        return Err((StatusCode::BAD_REQUEST, Json(serde_json::json!({"error": format!("Invalid scene pack id '{scene_id}'")}))));
    }
    if !validate_pack_id(&tool_id) {
        return Err((StatusCode::BAD_REQUEST, Json(serde_json::json!({"error": format!("Invalid tool pack id '{tool_id}'")}))));
    }

    let (scene_manifest, scene_world_xml) = state.resolve_scene(&scene_id)
        .ok_or_else(|| (StatusCode::NOT_FOUND, Json(serde_json::json!({"error": format!("Scene pack '{scene_id}' not found")}))))?;

    let (tool_manifest, tool_xml) = state.resolve_tool(&tool_id)
        .ok_or_else(|| (StatusCode::NOT_FOUND, Json(serde_json::json!({"error": format!("Tool pack '{tool_id}' not found")}))))?;

    let policy_id = match payload.policy {
        Some(Some(ref pid)) => {
            if !validate_pack_id(pid) {
                return Err((StatusCode::BAD_REQUEST, Json(serde_json::json!({"error": format!("Invalid policy pack id '{pid}'")}))));
            }
            if state.resolve_policy(pid).is_none() {
                return Err((StatusCode::NOT_FOUND, Json(serde_json::json!({"error": format!("Policy pack '{pid}' not found")}))));
            }
            Some(pid.clone())
        }
        Some(None) => None,
        None => cur_policy,
    };

    let base_xml = get_base_robot_xml();
    let composed = compose_mjcf(
        &base_xml,
        Some(&scene_world_xml),
        Some(&tool_manifest),
        Some(&tool_xml),
    ).map_err(|e| (StatusCode::BAD_REQUEST, Json(serde_json::json!({"error": format!("MJCF composition failed: {e}")}))))?;

    // If mode == "sim", send POST /sim/scene to rebot-sim upstream
    if mode == "sim" {
        let sim_url = sim_upstream_url()?;
        let upstream = DetectorUpstream::parse(&sim_url)
            .map_err(|e| (StatusCode::BAD_GATEWAY, Json(serde_json::json!({"error": e, "sim_url": sim_url}))))?;

        let body_bytes = serde_json::to_vec(&serde_json::json!({
            "scene": scene_id,
            "tool": tool_id,
            "seed": payload.seed.unwrap_or(cur_seed),
        })).unwrap();

        sim_post(&upstream, &sim_url, "/sim/scene", &body_bytes).await?;
    }

    // Success: mutate ONLY mode, active packs, seed, auto_start_policy and updated_at_ms
    let (final_auto_start, final_seed, final_reset_seq, final_io_seq) = {
        let mut st_write = state.active.write().await;
        st_write.mode = mode.clone();
        st_write.active_scene = scene_id.clone();
        st_write.active_tool = tool_id.clone();
        st_write.active_policy = policy_id.clone();
        if let Some(s) = payload.seed {
            st_write.seed = s;
        }
        if let Some(asp) = payload.auto_start_policy {
            st_write.auto_start_policy = asp;
        }
        st_write.updated_at_ms = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_millis() as u64)
            .unwrap_or(0);

        let mut mjcf_write = state.cached_mjcf.write().await;
        *mjcf_write = composed;

        (
            st_write.auto_start_policy,
            st_write.seed,
            st_write.reset_seq,
            st_write.io_seq,
        )
    };
    state.persist().await;

    Ok(Json(serde_json::json!({
        "mode": mode,
        "active_scene": scene_id,
        "active_tool": tool_id,
        "active_policy": policy_id,
        "auto_start_policy": final_auto_start,
        "seed": final_seed,
        "reset_seq": final_reset_seq,
        "io_seq": final_io_seq,
        "scene_manifest": scene_manifest,
        "tool_manifest": tool_manifest,
    })))
}

/// POSTs `body` to rebot-sim; a non-2xx reply, connect failure or 10 s timeout becomes the handler error.
async fn sim_post(
    upstream: &DetectorUpstream,
    sim_url: &str,
    path: &str,
    body: &[u8],
) -> Result<(), (StatusCode, Json<serde_json::Value>)> {
    let req_fut = proxy::upstream_request(upstream, Method::POST, path, Some(body));
    match tokio::time::timeout(std::time::Duration::from_secs(10), req_fut).await {
        Ok(Ok(resp)) if !resp.status.is_success() => {
            let err_msg = String::from_utf8_lossy(&resp.body).to_string();
            Err((resp.status, Json(serde_json::json!({"error": format!("rebot-sim error: {err_msg}"), "sim_url": sim_url}))))
        }
        Ok(Ok(_)) => Ok(()),
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

/// GET /api/v1/sandbox/bundle
pub async fn get_bundle(
    Extension(state): Extension<Arc<SandboxState>>,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<serde_json::Value>)> {
    let current = state.active.read().await.clone();
    let cached_xml = state.cached_mjcf.read().await.clone();

    let composed_xml = if !cached_xml.is_empty() {
        cached_xml
    } else {
        let (_sm, sw) = state.resolve_scene(&current.active_scene)
            .ok_or_else(|| (StatusCode::INTERNAL_SERVER_ERROR, Json(serde_json::json!({"error": "Failed to resolve active scene"}))))?;
        let (tm, tw) = state.resolve_tool(&current.active_tool)
            .ok_or_else(|| (StatusCode::INTERNAL_SERVER_ERROR, Json(serde_json::json!({"error": "Failed to resolve active tool"}))))?;
        compose_mjcf(&get_base_robot_xml(), Some(&sw), Some(&tm), Some(&tw))
            .map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, Json(serde_json::json!({"error": e}))))?
    };

    let scene_manifest = state.resolve_scene(&current.active_scene).map(|x| x.0);
    let tool_manifest = state.resolve_tool(&current.active_tool).map(|x| x.0);
    let policy_manifest = current.active_policy.as_ref().and_then(|p| state.resolve_policy(p));
    let policy_model_url = policy_manifest.as_ref().map(|pm| format!("/api/v1/sandbox/policies/{}/files/{}", pm.id, pm.model_file));

    Ok(Json(serde_json::json!({
        "mode": current.mode,
        "active_scene": current.active_scene,
        "active_tool": current.active_tool,
        "active_policy": current.active_policy,
        "auto_start_policy": current.auto_start_policy,
        "seed": current.seed,
        "reset_seq": current.reset_seq,
        "io_seq": current.io_seq,
        "composed_xml": composed_xml,
        "scene_manifest": scene_manifest,
        "tool_manifest": tool_manifest,
        "policy_manifest": policy_manifest,
        "policy_model_url": policy_model_url,
        "desired_io": current.desired_io,
    })))
}

/// GET /api/v1/sandbox/state
pub async fn get_state(
    Extension(state): Extension<Arc<SandboxState>>,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<serde_json::Value>)> {
    let current = state.active.read().await.clone();

    if current.mode == "sim" {
        let sim_url = sim_upstream_url().unwrap_or_default();
        if let Ok(upstream) = DetectorUpstream::parse(&sim_url) {
            let req_fut = proxy::upstream_request(&upstream, Method::GET, "/sim/state", None);
            match tokio::time::timeout(std::time::Duration::from_secs(10), req_fut).await {
                Ok(Ok(resp)) => {
                    if resp.status.is_success() {
                        if let Ok(sim_val) = serde_json::from_slice::<serde_json::Value>(&resp.body) {
                            return Ok(Json(serde_json::json!({
                                "mode": current.mode,
                                "active_scene": current.active_scene,
                                "active_tool": current.active_tool,
                                "active_policy": current.active_policy,
                                "seed": current.seed,
                                "reset_seq": current.reset_seq,
                                "io_seq": current.io_seq,
                                "desired_io": current.desired_io,
                                "sim_reachable": true,
                                "sim_state": sim_val,
                                "telemetry": current.telemetry,
                                "updated_at_ms": current.updated_at_ms,
                            })));
                        }
                    }
                }
                Ok(Err(_)) => {
                    // Fallback to unreachable below
                }
                Err(_elapsed) => {
                    return Err((StatusCode::GATEWAY_TIMEOUT, Json(serde_json::json!({
                        "error": format!("Timeout connecting to rebot-sim at {sim_url}"),
                        "sim_url": sim_url,
                    }))));
                }
            }
        }
        return Ok(Json(serde_json::json!({
            "mode": current.mode,
            "active_scene": current.active_scene,
            "active_tool": current.active_tool,
            "active_policy": current.active_policy,
            "seed": current.seed,
            "reset_seq": current.reset_seq,
            "io_seq": current.io_seq,
            "desired_io": current.desired_io,
            "sim_reachable": false,
            "telemetry": current.telemetry,
            "updated_at_ms": current.updated_at_ms,
        })));
    }

    Ok(Json(serde_json::json!({
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
    })))
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
            .route("/api/v1/sandbox/policies/{id}", get(get_policy).delete(delete_policy).with_state(supervisor))
            .route("/api/v1/sandbox/policies/{id}/files/{*path}", get(serve_policy_file))
            .route("/api/v1/sandbox/activate", post(activate))
            .route("/api/v1/sandbox/bundle", get(get_bundle))
            .route("/api/v1/sandbox/state", get(get_state))
            .route("/api/v1/sandbox/reset", post(reset))
            .route("/api/v1/sandbox/io", post(set_io))
            .route("/api/v1/sandbox/telemetry", post(post_telemetry))
            .layer(Extension(state))
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
        let app = axum::Router::new()
            .route("/api/v1/sandbox/policies", axum::routing::post(upload_policy))
            .route("/api/v1/sandbox/activate", axum::routing::post(activate))
            .route("/api/v1/sandbox/bundle", axum::routing::get(get_bundle))
            .layer(axum::Extension(state));

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
}
