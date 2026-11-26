//! Detection kind switch (`POST /api/v1/vision/detector`, operator only): the cockpit's simple
//! mode chooses between boxes (`detect`) and outlines (`segment`), each served by one default
//! detector model of this armd. Any other model is chosen elsewhere (grasp task `models`).
//!
//! The kind maps to the first name of [`DetectorDefaults`] (profile `vision_detectors`, else the
//! built-in names of the rebot-vision catalogue) that the provider's catalogue (`GET /api/mode`
//! `models.catalogue`) lists; armd then loads it the way a grasp task does (`POST /api/models`,
//! then `/api/mode` polled until the swap is done). Grasps, measurements and other switches use
//! the same detector, so a switch is refused while one of them runs.
use std::collections::HashMap;
use std::fs::{self, File};
use std::io::Read;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant, UNIX_EPOCH};

use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

use crate::grasp::DetectionSource;
use crate::supervisor::{ArmError, ArmSupervisor};

/// Compiled-in copy of the single-source-of-truth vision model manifest.
pub const EMBEDDED_MODELS_TSV: &str = include_str!("../kits/recamera-b601dm/vision/models/models.tsv");

/// Default on-device path to `models.tsv` installed by the `rebot-teleop` package.
pub const DEFAULT_MANIFEST_REL_PATH: &str = "usr/share/rebot-teleop/models/models.tsv";

/// One parsed row of `kits/recamera-b601dm/vision/models/models.tsv`.
#[derive(Debug, Clone, PartialEq)]
pub struct ManifestEntry {
    pub role: String,
    pub file: String,
    pub md5: String,
    pub size: u64,
    pub ion_mb: f64,
    pub url: String,
    pub ship: String,
}

/// Model configuration health exposed on `GET /api/v1/vision/mode` as `model_config`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ModelConfigStatus {
    pub optimal: bool,
    pub missing: Vec<String>,
    pub mismatched: Vec<String>,
    pub source: String,
}

/// File stem up to the first `.` (matches `rebot-vision`'s `rv::file_stem`).
pub fn model_file_stem(file: &str) -> &str {
    let base = file.rsplit('/').next().unwrap_or(file);
    base.split('.').next().unwrap_or(base)
}

/// Parse tab-separated `models.tsv` text (comments `#` and blank lines skipped).
pub fn parse_manifest(tsv: &str) -> Vec<ManifestEntry> {
    let mut out = Vec::new();
    for raw in tsv.lines() {
        let line = raw.trim_end_matches('\r');
        let trimmed = line.trim_start();
        if trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }
        let cols: Vec<&str> = line.split('\t').collect();
        if cols.len() < 7 || cols[0].is_empty() || cols[1].is_empty() {
            continue;
        }
        out.push(ManifestEntry {
            role: cols[0].to_string(),
            file: cols[1].to_string(),
            md5: cols[2].to_string(),
            size: cols[3].parse::<u64>().unwrap_or(0),
            ion_mb: cols[4].parse::<f64>().unwrap_or(0.0),
            url: cols[5].to_string(),
            ship: cols[6].to_string(),
        });
    }
    out
}

/// Load `models.tsv` from `REBOT_MODELS_TSV` or `<root>/usr/share/rebot-teleop/models/models.tsv`,
/// falling back to [`EMBEDDED_MODELS_TSV`].
pub fn load_manifest_for_root(root: &Path) -> Vec<ManifestEntry> {
    if let Ok(p) = std::env::var("REBOT_MODELS_TSV") {
        if !p.is_empty() {
            if let Ok(text) = fs::read_to_string(&p) {
                let parsed = parse_manifest(&text);
                if !parsed.is_empty() {
                    return parsed;
                }
            }
        }
    }
    let on_disk = root.join(DEFAULT_MANIFEST_REL_PATH);
    if let Ok(text) = fs::read_to_string(&on_disk) {
        let parsed = parse_manifest(&text);
        if !parsed.is_empty() {
            return parsed;
        }
    }
    parse_manifest(EMBEDDED_MODELS_TSV)
}

/// Pure-Rust streaming MD5 (RFC 1321) to avoid extra crate dependencies and stay offline-safe.
pub fn md5_bytes(data: &[u8]) -> String {
    let mut h = Md5::new();
    h.update(data);
    h.finish_hex()
}

struct Md5 {
    state: [u32; 4],
    len: u64,
    buf: [u8; 64],
    buf_len: usize,
}

impl Md5 {
    fn new() -> Self {
        Self {
            state: [0x6745_2301, 0xefcd_ab89, 0x98ba_dcfe, 0x1032_5476],
            len: 0,
            buf: [0u8; 64],
            buf_len: 0,
        }
    }

    fn update(&mut self, mut input: &[u8]) {
        self.len = self.len.wrapping_add(input.len() as u64);
        if self.buf_len > 0 {
            let need = 64 - self.buf_len;
            if input.len() < need {
                self.buf[self.buf_len..self.buf_len + input.len()].copy_from_slice(input);
                self.buf_len += input.len();
                return;
            }
            self.buf[self.buf_len..64].copy_from_slice(&input[..need]);
            let block = self.buf;
            self.compress(&block);
            self.buf_len = 0;
            input = &input[need..];
        }
        while input.len() >= 64 {
            let mut block = [0u8; 64];
            block.copy_from_slice(&input[..64]);
            self.compress(&block);
            input = &input[64..];
        }
        if !input.is_empty() {
            self.buf[..input.len()].copy_from_slice(input);
            self.buf_len = input.len();
        }
    }

    fn finish_hex(mut self) -> String {
        let bit_len = self.len.wrapping_mul(8);
        self.update(&[0x80]);
        let pad_len = if self.buf_len <= 56 { 56 - self.buf_len } else { 120 - self.buf_len };
        self.update(&[0u8; 64][..pad_len.min(64)]);
        if pad_len > 64 {
            self.update(&[0u8; 64][..pad_len - 64]);
        }
        self.update(&bit_len.to_le_bytes());
        let mut out = String::with_capacity(32);
        for w in self.state {
            for b in w.to_le_bytes() {
                use std::fmt::Write;
                let _ = write!(&mut out, "{b:02x}");
            }
        }
        out
    }

    fn compress(&mut self, block: &[u8; 64]) {
        const S: [u32; 64] = [
            7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
            5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20,
            4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
            6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21,
        ];
        const K: [u32; 64] = [
            0xd76a_a478, 0xe8c7_b756, 0x2420_70db, 0xc1bd_ceee, 0xf57c_0faf, 0x4787_c62a, 0xa830_4613, 0xfd46_9501,
            0x6980_98d8, 0x8b44_f7af, 0xffff_5bb1, 0x895c_d7be, 0x6b90_1122, 0xfd98_7193, 0xa679_438e, 0x49b4_0821,
            0xf61e_2562, 0xc040_b340, 0x265e_5a51, 0xe9b6_c7aa, 0xd62f_105d, 0x0244_1453, 0xd8a1_e681, 0xe7d3_fbc8,
            0x21e1_cde6, 0xc337_07d6, 0xf4d5_0d87, 0x455a_14ed, 0xa9e3_e905, 0xfcef_a3f8, 0x676f_02d9, 0x8d2a_4c8a,
            0xfffa_3942, 0x8771_f681, 0x6d9d_6122, 0xfde5_380c, 0xa4be_ea44, 0x4bde_cfa9, 0xf6bb_4b60, 0xbebf_bc70,
            0x289b_7ec6, 0xeaa1_27fa, 0xd4ef_3085, 0x0488_1d05, 0xd9d4_d039, 0xe6db_99e5, 0x1fa2_7cf8, 0xc4ac_5665,
            0xf429_2244, 0x432a_ff97, 0xab94_23a7, 0xfc93_a039, 0x655b_59c3, 0x8f0c_cc92, 0xffef_f47d, 0x8584_5dd1,
            0x6fa8_7e4f, 0xfe2c_e6e0, 0xa301_4314, 0x4e08_11a1, 0xf753_7e82, 0xbd3a_f235, 0x2ad7_d2bb, 0xeb86_d391,
        ];
        let mut m = [0u32; 16];
        for i in 0..16 {
            m[i] = u32::from_le_bytes([block[4 * i], block[4 * i + 1], block[4 * i + 2], block[4 * i + 3]]);
        }
        let [mut a, mut b, mut c, mut d] = self.state;
        for i in 0..64 {
            let (f, g) = match i {
                0..=15 => ((b & c) | ((!b) & d), i),
                16..=31 => ((d & b) | ((!d) & c), (5 * i + 1) % 16),
                32..=47 => (b ^ c ^ d, (3 * i + 5) % 16),
                _ => (c ^ (b | (!d)), (7 * i) % 16),
            };
            let f = f.wrapping_add(a).wrapping_add(K[i]).wrapping_add(m[g]);
            a = d;
            d = c;
            c = b;
            b = b.wrapping_add(f.rotate_left(S[i]));
        }
        self.state[0] = self.state[0].wrapping_add(a);
        self.state[1] = self.state[1].wrapping_add(b);
        self.state[2] = self.state[2].wrapping_add(c);
        self.state[3] = self.state[3].wrapping_add(d);
    }
}

fn cached_file_md5(path: &Path) -> Option<String> {
    let meta = fs::metadata(path).ok()?;
    if !meta.is_file() {
        return None;
    }
    let len = meta.len();
    let mtime = meta
        .modified()
        .ok()
        .and_then(|t| t.duration_since(UNIX_EPOCH).ok())
        .map(|d| d.as_nanos())
        .unwrap_or(0);
    let canon = fs::canonicalize(path).unwrap_or_else(|_| path.to_path_buf());
    static CACHE: OnceLock<Mutex<HashMap<PathBuf, (u64, u128, String)>>> = OnceLock::new();
    let cache = CACHE.get_or_init(|| Mutex::new(HashMap::new()));
    if let Ok(g) = cache.lock() {
        if let Some((clen, cmtime, digest)) = g.get(&canon) {
            if *clen == len && *cmtime == mtime {
                return Some(digest.clone());
            }
        }
    }
    let mut f = File::open(&canon).ok()?;
    let mut hasher = Md5::new();
    let mut buf = [0u8; 64 * 1024];
    loop {
        let n = f.read(&mut buf).ok()?;
        if n == 0 {
            break;
        }
        hasher.update(&buf[..n]);
    }
    let digest = hasher.finish_hex();
    if let Ok(mut g) = cache.lock() {
        g.insert(canon, (len, mtime, digest.clone()));
    }
    Some(digest)
}

fn resolve_under_root(root: &Path, device_path: &str) -> PathBuf {
    if root == Path::new("/") {
        PathBuf::from(device_path)
    } else {
        root.join(device_path.trim_start_matches('/'))
    }
}

fn read_recorded_sources(root: &Path) -> HashMap<String, String> {
    let mut out = HashMap::new();
    let path = root.join("userdata/local/models/.rebot-model-sources");
    if let Ok(text) = fs::read_to_string(path) {
        for line in text.lines() {
            if let Some((file, src)) = line.split_once('\t') {
                if !file.is_empty() && !src.is_empty() {
                    out.insert(file.to_string(), src.to_string());
                }
            }
        }
    }
    out
}

fn classify_model_source(path: &Path, file: &str, root: &Path, recorded: &HashMap<String, String>) -> &'static str {
    let bundled_dir = root.join("usr/share/rebot-teleop/models");
    let bundled_canon = fs::canonicalize(&bundled_dir).unwrap_or(bundled_dir);
    if path.starts_with(&bundled_canon) {
        return "bundled";
    }
    if let Ok(link) = fs::read_link(path) {
        let target = if link.is_absolute() {
            resolve_under_root(root, &link.to_string_lossy())
        } else {
            path.parent().map(|p| p.join(&link)).unwrap_or(link)
        };
        let target_canon = fs::canonicalize(&target).unwrap_or(target);
        if target_canon.starts_with(&bundled_canon) {
            return "bundled";
        }
    }
    if let Ok(canon) = fs::canonicalize(path) {
        if canon.starts_with(&bundled_canon) {
            return "bundled";
        }
    }
    if recorded.get(file).map(String::as_str) == Some("cdn") {
        return "cdn";
    }
    "user"
}

/// Evaluate `model_config` against `manifest`, the upstream `/api/mode` JSON, and filesystem `root`.
pub fn evaluate_model_config(mode_json: &Value, manifest: &[ManifestEntry], root: &Path) -> ModelConfigStatus {
    let mut reported_paths: HashMap<String, String> = HashMap::new();
    let mut has_catalogue = false;
    if let Some(cat) = mode_json
        .get("models")
        .and_then(|m| m.get("catalogue"))
        .and_then(Value::as_array)
    {
        has_catalogue = true;
        for entry in cat {
            if let (Some(name), Some(path)) = (
                entry.get("name").and_then(Value::as_str),
                entry.get("path").and_then(Value::as_str),
            ) {
                reported_paths.insert(name.to_string(), path.to_string());
            }
        }
    }
    for slot in ["model", "depth_model"] {
        if let Some(obj) = mode_json.get(slot).and_then(Value::as_object) {
            if let (Some(name), Some(path)) = (
                obj.get("name").and_then(Value::as_str),
                obj.get("path").and_then(Value::as_str),
            ) {
                reported_paths.insert(name.to_string(), path.to_string());
            }
        }
    }

    let recorded = read_recorded_sources(root);
    let mut missing = Vec::new();
    let mut mismatched = Vec::new();
    let mut saw_bundled = false;
    let mut saw_cdn = false;
    let mut saw_user = false;

    for entry in manifest {
        let required = matches!(entry.role.as_str(), "detector-segment" | "detector-detect" | "depth");
        if !required {
            continue;
        }
        let stem = model_file_stem(&entry.file);
        let candidate: Option<PathBuf> = if let Some(dev_path) = reported_paths.get(stem) {
            Some(resolve_under_root(root, dev_path))
        } else if has_catalogue {
            None
        } else {
            let dirs: &[&str] = if entry.ship == "os" {
                &["usr/share/supervisor/models", "userdata/local/models"]
            } else {
                &[
                    "userdata/local/models",
                    "userdata/rebot-teleop/models",
                    "usr/share/rebot-teleop/models",
                ]
            };
            dirs.iter()
                .map(|d| root.join(d).join(&entry.file))
                .find(|p| fs::metadata(p).map(|m| m.is_file()).unwrap_or(false))
        };

        let Some(path) = candidate else {
            missing.push(entry.file.clone());
            continue;
        };
        let Some(got_md5) = cached_file_md5(&path) else {
            missing.push(entry.file.clone());
            continue;
        };
        if got_md5 != entry.md5 {
            mismatched.push(entry.file.clone());
        }
        if entry.ship == "bundled" {
            match classify_model_source(&path, &entry.file, root, &recorded) {
                "bundled" => saw_bundled = true,
                "cdn" => saw_cdn = true,
                _ => saw_user = true,
            }
        }
    }

    // Check if an active model slot is pinned to a non-manifest user file.
    let manifest_stems: Vec<&str> = manifest.iter().map(|e| model_file_stem(&e.file)).collect();
    for slot in ["model", "depth_model"] {
        if let Some(name) = mode_json
            .get(slot)
            .and_then(|v| v.get("name"))
            .and_then(Value::as_str)
        {
            if !manifest_stems.contains(&name) {
                saw_user = true;
            }
        }
    }

    let source = if saw_user {
        "user"
    } else if saw_cdn {
        "cdn"
    } else if saw_bundled {
        "bundled"
    } else {
        "user"
    };

    ModelConfigStatus {
        optimal: missing.is_empty() && mismatched.is_empty(),
        missing,
        mismatched,
        source: source.to_string(),
    }
}

/// Evaluate `model_config` on the active host filesystem (`REBOT_MODELS_ROOT`, default `/`).
pub fn model_config_status(mode_json: &Value) -> ModelConfigStatus {
    let root = std::env::var("REBOT_MODELS_ROOT")
        .ok()
        .filter(|s| !s.is_empty())
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("/"));
    let manifest = load_manifest_for_root(&root);
    evaluate_model_config(mode_json, &manifest, &root)
}

/// Path of the simple-mode detection-kind switch REST route.
pub const PATH: &str = "/api/v1/vision/detector";
/// Path of the expert-mode model variant / depth slot switch REST route.
pub const MODELS_PATH: &str = "/api/v1/vision/models";
/// How long the switch waits for the provider's swap (`/api/mode` polled).
const SWITCH_TIMEOUT: Duration = Duration::from_secs(5);

/// Default detector model names per kind (profile `vision_detectors`). The first name the
/// provider's catalogue lists is loaded; an empty list disables that kind.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct DetectorDefaults {
    pub detect: Vec<String>,
    pub segment: Vec<String>,
}

impl Default for DetectorDefaults {
    /// The rebot-vision catalogue on the reCamera: the YOLO11n detector, and the 384x640
    /// segmentation model (fits next to the depth model) before the 640x640 one.
    fn default() -> Self {
        Self {
            detect: vec!["yolo11n_detection_cv181x_int8".into()],
            segment: vec!["yolo11n_seg_384x640_cv181x_int8_fuse".into(), "yolo11n_segment_cv181x_int8".into()],
        }
    }
}

impl DetectorDefaults {
    pub fn validate(&self) -> Result<(), String> {
        for (kind, names) in [("detect", &self.detect), ("segment", &self.segment)] {
            if names.iter().any(|n| n.is_empty() || n.len() > 64) {
                return Err(format!("vision_detectors.{kind}: names must be 1..64 bytes"));
            }
        }
        Ok(())
    }

    fn names(&self, kind: &str) -> &[String] {
        if kind == "detect" { &self.detect } else { &self.segment }
    }
}

/// Single-view depth measurements (`POST /api/v1/grasp/measure`), REST model switches
/// (`POST /api/v1/vision/detector`, `POST /api/v1/vision/models`), and grasp preflight model
/// switches share the vision provider and mutually exclude each other through one non-blocking
/// state gate (never held across `.await`).
#[derive(Default)]
struct VisionUseState {
    measures: usize,
    switching: bool,
    /// Set when the provider accepts an asynchronous model switch (`POST /api/models` 2xx with
    /// `generation`) and cleared only once `GET /api/mode` confirms `generation >= gen` with
    /// `swap_state` in `idle | failed`. Survives polling errors/timeouts so a subsequent
    /// `depth_measure` cannot overlap an unsettled provider swap.
    pending_swap_generation: Option<u64>,
}

static VISION_USE: Mutex<VisionUseState> = Mutex::new(VisionUseState {
    measures: 0,
    switching: false,
    pending_swap_generation: None,
});

static VISION_NOTIFY: tokio::sync::Notify = tokio::sync::Notify::const_new();

/// Held by a running depth measurement ([`begin_measure`] / [`try_begin_measure`]).
pub struct MeasureGuard(());

impl Drop for MeasureGuard {
    fn drop(&mut self) {
        {
            let mut st = VISION_USE.lock().unwrap_or_else(|e| e.into_inner());
            st.measures = st.measures.saturating_sub(1);
        }
        VISION_NOTIFY.notify_waiters();
    }
}

struct SwitchGuard(());

impl Drop for SwitchGuard {
    fn drop(&mut self) {
        {
            let mut st = VISION_USE.lock().unwrap_or_else(|e| e.into_inner());
            st.switching = false;
        }
        VISION_NOTIFY.notify_waiters();
    }
}

/// Held by a grasp preflight model switch ([`begin_grasp_model_switch`]): serializes against
/// REST detector switches and waits for / fences against concurrent depth measurements.
pub(crate) struct GraspModelSwitchGuard {
    _serial: tokio::sync::MutexGuard<'static, ()>,
    _switch: SwitchGuard,
}

/// Serializes a grasp preflight model switch behind any active model switch and any in-flight
/// depth measurements, then fences new depth measurements until preflight drops the guard.
pub(crate) async fn begin_grasp_model_switch() -> GraspModelSwitchGuard {
    let serial = crate::grasp::MODEL_SWITCH.lock().await;
    loop {
        let notified = VISION_NOTIFY.notified();
        {
            let mut st = VISION_USE.lock().unwrap_or_else(|e| e.into_inner());
            if st.measures == 0 && !st.switching {
                st.switching = true;
                return GraspModelSwitchGuard {
                    _serial: serial,
                    _switch: SwitchGuard(()),
                };
            }
        }
        notified.await;
    }
}

/// Admits a depth measurement if no model switch holds the vision lock; returns `409 VISION_BUSY`
/// when a model swap is in progress so swaps and measurements cannot overlap in either order.
pub fn try_measure_in_flight() -> Result<MeasureGuard, ArmError> {
    try_begin_measure()
}

/// Marks a depth measurement as running until the guard is dropped.
/// Kept for compatibility with tests; production callers use [`begin_measure`].
pub fn measure_in_flight() -> MeasureGuard {
    let mut st = VISION_USE.lock().unwrap_or_else(|e| e.into_inner());
    st.measures = st.measures.saturating_add(1);
    MeasureGuard(())
}

/// Atomically registers an in-flight depth measurement if no model switch (active or unsettled on
/// the provider) holds the vision provider, refusing with `409 VISION_BUSY` otherwise.
pub fn try_begin_measure() -> Result<MeasureGuard, ArmError> {
    let _probe = crate::grasp::MODEL_SWITCH
        .try_lock()
        .map_err(|_| busy("a model switch is in progress"))?;
    let mut st = VISION_USE.lock().unwrap_or_else(|e| e.into_inner());
    if st.switching || st.pending_swap_generation.is_some() {
        return Err(busy("a model switch is in progress"));
    }
    st.measures = st.measures.saturating_add(1);
    Ok(MeasureGuard(()))
}

/// Production admission gate for single-view depth measurements: if a prior REST model switch left
/// an unsettled provider-accepted `pending_swap_generation` (e.g. poll timed out or failed), probes
/// `source.mode_info()` once to confirm the swap settled before admitting the measurement.
pub async fn begin_measure(source: &dyn DetectionSource) -> Result<MeasureGuard, ArmError> {
    let pending = VISION_USE
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .pending_swap_generation;
    if let Some(gen) = pending {
        let mode = source
            .mode_info()
            .await
            .map_err(|_| busy("the vision service is switching models"))?;
        let settled = mode.models.as_ref().is_some_and(|pm| {
            pm.generation >= gen && matches!(pm.swap_state.as_str(), "idle" | "failed")
        });
        if !settled {
            return Err(busy("the vision service is switching models"));
        }
        let mut st = VISION_USE.lock().unwrap_or_else(|e| e.into_inner());
        if st.pending_swap_generation == Some(gen) {
            st.pending_swap_generation = None;
        }
    }
    try_begin_measure()
}

/// Why an action of `kind` in `state` blocks a switch (None: it does not).
pub fn busy_action(kind: &str, state: &str) -> Option<&'static str> {
    if state != "running" { return None; }
    match kind {
        "grasp" => Some("a grasp is running"),
        "measure_multiview" => Some("a size measurement is running"),
        _ => None,
    }
}

fn rejected(status: u16, code: &str, message: impl Into<String>) -> ArmError {
    ArmError::Rejected { status, code: code.into(), message: message.into() }
}

fn busy(why: &str) -> ArmError {
    rejected(409, "VISION_BUSY", format!("Cannot switch the detection kind: {why}. Wait until it ends (or press Stop), then switch again."))
}

/// Body `{"kind": "detect" | "segment"}`.
pub fn parse_kind(body: &Value) -> Result<&'static str, ArmError> {
    let invalid = |m: String| ArmError::Rejected { status: 400, code: "INVALID_COMMAND".into(), message: m };
    let obj = body.as_object().ok_or_else(|| invalid("Body must be {\"kind\": \"detect\" | \"segment\"}".into()))?;
    if let Some(k) = obj.keys().find(|k| k.as_str() != "kind") {
        return Err(invalid(format!("Unknown field {k}: the body is {{\"kind\": \"detect\" | \"segment\"}}")));
    }
    match obj.get("kind").and_then(Value::as_str) {
        Some("detect") => Ok("detect"),
        Some("segment") => Ok("segment"),
        Some(other) => Err(invalid(format!(
            "Unknown detection kind {other:?}: use \"detect\" (boxes) or \"segment\" (outlines); other models are loaded by grasp tasks"
        ))),
        None => Err(invalid("kind is missing: use \"detect\" (boxes) or \"segment\" (outlines)".into())),
    }
}

/// Parsed `POST /api/v1/vision/models` body: at least one of `detector` or `depth`.
/// `None` = keep current slot, `Some(None)` = unload slot, `Some(Some(name))` = load catalogue model.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ModelsPatch {
    pub detector: Option<Option<String>>,
    pub depth: Option<Option<String>>,
}

/// Parse `{"detector"?: name|null, "depth"?: name|null}` (at least one slot, no unknown fields).
pub fn parse_models_patch(body: &Value) -> Result<ModelsPatch, ArmError> {
    let invalid = |m: String| ArmError::Rejected { status: 400, code: "INVALID_COMMAND".into(), message: m };
    let obj = body.as_object().ok_or_else(|| {
        invalid("Body must be a JSON object with \"detector\" and/or \"depth\" (model name or null to unload).".into())
    })?;
    if let Some(k) = obj.keys().find(|k| !matches!(k.as_str(), "detector" | "depth")) {
        return Err(invalid(format!(
            "Unknown field {k:?}: send \"detector\" and/or \"depth\" (model name from the catalogue, or null to unload)."
        )));
    }
    let parse_slot = |slot: &str| -> Result<Option<Option<String>>, ArmError> {
        match obj.get(slot) {
            None => Ok(None),
            Some(Value::Null) => Ok(Some(None)),
            Some(Value::String(s)) if !s.is_empty() && s.len() <= 64 => Ok(Some(Some(s.clone()))),
            Some(Value::String(_)) => Err(invalid(format!(
                "{slot} model name must be 1..64 bytes. Choose a model from the catalogue or send null to unload."
            ))),
            Some(_) => Err(invalid(format!(
                "{slot} must be a catalogue model name string or null to unload."
            ))),
        }
    };
    let detector = parse_slot("detector")?;
    let depth = parse_slot("depth")?;
    if detector.is_none() && depth.is_none() {
        return Err(invalid("No model slot given: include \"detector\" and/or \"depth\" (name or null).".into()));
    }
    Ok(ModelsPatch { detector, depth })
}

fn format_ion_budget_error(target_desc: &str, v: &Value, depth_loaded: bool, loading_depth: bool) -> ArmError {
    let mb = |k: &str| v.get(k).and_then(Value::as_f64);
    let need = match (mb("need_mb"), mb("limit_mb")) {
        (Some(n), Some(l)) => format!(" (needs {n:.1} MB, limit {l:.1} MB)"),
        _ => String::new(),
    };
    let fix = if loading_depth {
        "Choose a smaller detector variant (or unload the detector) so the depth model fits in ION memory, then try again."
    } else if depth_loaded {
        "Disable/unload the depth model first or choose a smaller detector variant, then switch again."
    } else {
        "Keep the current model or choose a smaller variant from the catalogue, then switch again."
    };
    rejected(409, "ION_BUDGET", format!(
        "The vision service refused {target_desc}: not enough camera ION memory for the selected models (ION_BUDGET{need}). {fix}"
    ))
}

async fn execute_swap(
    source: &dyn DetectionSource,
    req_body: &Value,
    target_desc: &str,
    expected_det: Option<Option<&str>>,
    expected_dep: Option<Option<&str>>,
    depth_loaded_after: bool,
    loading_depth: bool,
) -> Result<crate::grasp::ProviderModels, ArmError> {
    let unreachable = |e: String| rejected(503, "DETECTION_UNAVAILABLE", format!(
        "Cannot reach the vision service ({e}). Check that the camera app (rebot-vision) is running, then try again."
    ));
    let (status, v) = source.set_models(req_body).await.map_err(unreachable)?;
    let error = v.get("error").and_then(Value::as_str).unwrap_or("refused");
    match status {
        200..=299 => {}
        404 => return Err(rejected(422, "UNSUPPORTED", "The vision service cannot switch models (POST /api/models answered 404). Update rebot-teleop on the device.")),
        409 if error == "SWAP_BUSY" => return Err(busy("the vision service is switching models")),
        409 if error == "ION_BUDGET" => return Err(format_ion_budget_error(target_desc, &v, depth_loaded_after, loading_depth)),
        400..=499 => return Err(rejected(422, "MODEL_SWITCH_REFUSED", format!(
            "The vision service refused to load {target_desc}: {error}. Check the model file and slot compatibility on the device."
        ))),
        _ => return Err(rejected(502, "DETECTION_UNAVAILABLE", format!(
            "The vision service answered {status} to the model switch. Check rebot-vision on the device, then try again."
        ))),
    }
    let Some(generation) = v.get("generation").and_then(Value::as_u64) else {
        return Err(rejected(502, "DETECTION_UNAVAILABLE", "The vision service's model switch reply has no generation. Update rebot-teleop on the device."));
    };
    {
        let mut st = VISION_USE.lock().unwrap_or_else(|e| e.into_inner());
        st.pending_swap_generation = Some(generation);
    }
    let deadline = Instant::now() + SWITCH_TIMEOUT;
    loop {
        let m = source.mode_info().await.map_err(unreachable)?;
        if let Some(pm) = m.models.filter(|pm| pm.generation >= generation && matches!(pm.swap_state.as_str(), "idle" | "failed")) {
            {
                let mut st = VISION_USE.lock().unwrap_or_else(|e| e.into_inner());
                if st.pending_swap_generation == Some(generation) {
                    st.pending_swap_generation = None;
                }
            }
            if pm.swap_state == "failed" {
                let why = pm.swap_error.as_deref().unwrap_or("unknown error");
                let code = if why.starts_with("ION_BUDGET") { "ION_BUDGET" } else { "MODEL_SWITCH_FAILED" };
                let fix = if code == "ION_BUDGET" {
                    if loading_depth {
                        "Choose a smaller detector variant (or unload the detector) so the depth model fits in ION memory, then try again."
                    } else {
                        "Disable/unload the depth model or pick a smaller detector variant, then try again."
                    }
                } else {
                    "Keep the current model, or check the model file and free camera memory on the device, then try again."
                };
                return Err(rejected(409, code, format!("The vision service could not load {target_desc} ({why}). {fix}")));
            }
            let det_ok = expected_det.is_none_or(|exp| pm.resident.detector.as_deref() == exp);
            let dep_ok = expected_dep.is_none_or(|exp| pm.resident.depth.as_deref() == exp);
            if !det_ok || !dep_ok {
                return Err(rejected(409, "MODEL_SWITCH_FAILED", format!(
                    "The vision service finished at generation {} with detector={:?} depth={:?} instead of {target_desc}. Refresh the model list and retry if needed.",
                    pm.generation, pm.resident.detector, pm.resident.depth
                )));
            }
            return Ok(pm);
        }
        if Instant::now() >= deadline {
            return Err(rejected(504, "DETECTION_UNAVAILABLE", format!(
                "The vision service did not finish loading {target_desc} within {} s. Check rebot-vision on the device, then try again.", SWITCH_TIMEOUT.as_secs()
            )));
        }
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
}

/// Acquire the non-blocking vision switch guard after taking `MODEL_SWITCH`.
fn acquire_switch_guard() -> Result<SwitchGuard, ArmError> {
    let mut st = VISION_USE.lock().unwrap_or_else(|e| e.into_inner());
    if st.measures > 0 {
        return Err(busy("a depth measurement is running"));
    }
    if st.switching {
        return Err(busy("another model switch is in progress"));
    }
    st.switching = true;
    Ok(SwitchGuard(()))
}

/// Switch the provider's detector to the default model of the requested kind. Reply
/// `{kind, model, changed, generation}`.
pub async fn switch(supervisor: &Arc<ArmSupervisor>, source: &dyn DetectionSource, body: &Value) -> Result<Value, ArmError> {
    let kind = parse_kind(body)?;
    let svc = supervisor
        .read_services(crate::rt_core::publish::ReadRequirements {
            require_prep_env: true,
            ..crate::rt_core::publish::ReadRequirements::default()
        })
        .await?;
    let defaults = svc
        .prep_env
        .as_ref()
        .and_then(|env| env.config.vision_detectors.clone())
        .unwrap_or_default();
    if let Some(a) = svc
        .snapshot
        .obs
        .action
        .as_ref()
        .and_then(|a| busy_action(&a.kind, &a.state))
    {
        return Err(busy(a));
    }
    // Acquire MODEL_SWITCH first, then check VISION_USE so a concurrent `depth_measure` cannot
    // race past admission into provider I/O.
    let Ok(_serial) = crate::grasp::MODEL_SWITCH.try_lock() else {
        return Err(busy("another model switch is in progress"));
    };
    let _switch_guard = acquire_switch_guard()?;
    let unreachable = |e: String| rejected(503, "DETECTION_UNAVAILABLE", format!("Cannot reach the vision service ({e}). Check that the camera app (rebot-vision) is running, then try again."));
    let mode = source.mode_info().await.map_err(unreachable)?;
    let Some(models) = mode.models.as_ref() else {
        return Err(rejected(422, "UNSUPPORTED", "The vision service cannot switch models (no models in GET /api/mode). Update rebot-teleop on the device."));
    };
    if !matches!(models.swap_state.as_str(), "idle" | "failed") {
        return Err(busy("the vision service is switching models"));
    }
    let Some(name) = defaults.names(kind).iter().find(|n| models.catalogue.contains_key(*n)).cloned() else {
        let have: Vec<&str> = models.catalogue.keys().map(String::as_str).collect();
        return Err(rejected(422, "MODEL_NOT_FOUND", format!(
            "No default {kind} model of this armd ({}) is in the vision service's catalogue ({}). Install the model on the device or set vision_detectors.{kind} in the arm profile.",
            defaults.names(kind).join(", "), if have.is_empty() { "empty".to_string() } else { have.join(", ") }
        )));
    };
    let reply = |changed: bool, generation: u64| json!({"kind": kind, "model": name, "changed": changed, "generation": generation});
    if models.resident.detector.as_deref() == Some(name.as_str()) && models.swap_state == "idle" {
        return Ok(reply(false, models.generation));
    }
    let pm = execute_swap(
        source,
        &json!({"detector": name}),
        &name,
        Some(Some(name.as_str())),
        None,
        models.resident.depth.is_some(),
        false,
    ).await?;
    Ok(reply(true, pm.generation))
}

/// Switch the provider's detector variant and/or depth slot by catalogue name (or `null` to unload).
/// Reply `{detector, depth, changed, generation}`.
pub async fn switch_models(supervisor: &Arc<ArmSupervisor>, source: &dyn DetectionSource, body: &Value) -> Result<Value, ArmError> {
    let patch = parse_models_patch(body)?;
    let svc = supervisor
        .read_services(crate::rt_core::publish::ReadRequirements::default())
        .await?;
    if let Some(a) = svc
        .snapshot
        .obs
        .action
        .as_ref()
        .and_then(|a| busy_action(&a.kind, &a.state))
    {
        return Err(busy(a));
    }
    // Acquire MODEL_SWITCH first, then check VISION_USE so a concurrent `depth_measure` cannot
    // race past admission into provider I/O.
    let Ok(_serial) = crate::grasp::MODEL_SWITCH.try_lock() else {
        return Err(busy("another model switch is in progress"));
    };
    let _switch_guard = acquire_switch_guard()?;
    let unreachable = |e: String| rejected(503, "DETECTION_UNAVAILABLE", format!(
        "Cannot reach the vision service ({e}). Check that the camera app (rebot-vision) is running, then try again."
    ));
    let mode = source.mode_info().await.map_err(unreachable)?;
    let Some(models) = mode.models.as_ref() else {
        return Err(rejected(422, "UNSUPPORTED", "The vision service cannot switch models (no models in GET /api/mode). Update rebot-teleop on the device."));
    };
    if !matches!(models.swap_state.as_str(), "idle" | "failed") {
        return Err(busy("the vision service is switching models"));
    }

    let mut req = serde_json::Map::new();
    let mut parts: Vec<String> = Vec::new();
    if let Some(det) = &patch.detector {
        if let Some(name) = det {
            if !models.catalogue.contains_key(name) {
                let have: Vec<&str> = models.catalogue.keys().map(String::as_str).collect();
                return Err(rejected(422, "MODEL_NOT_FOUND", format!(
                    "Detector model {name:?} is not in the vision service's catalogue ({}). Pick an installed model or refresh the model list.",
                    if have.is_empty() { "empty".to_string() } else { have.join(", ") }
                )));
            }
            if models.catalogue_kinds.get(name).map(String::as_str) == Some("depth") {
                return Err(rejected(400, "INVALID_COMMAND", format!(
                    "Model {name:?} is a depth model and cannot be loaded into the detector slot. Pass it under \"depth\" instead."
                )));
            }
            parts.push(name.clone());
        } else {
            parts.push("detector=off".into());
        }
        req.insert("detector".into(), match det { Some(n) => json!(n), None => Value::Null });
    }
    if let Some(dep) = &patch.depth {
        if let Some(name) = dep {
            if !models.catalogue.contains_key(name) {
                let have: Vec<&str> = models.catalogue.keys().map(String::as_str).collect();
                return Err(rejected(422, "MODEL_NOT_FOUND", format!(
                    "Depth model {name:?} is not in the vision service's catalogue ({}). Pick an installed depth model or refresh the model list.",
                    if have.is_empty() { "empty".to_string() } else { have.join(", ") }
                )));
            }
            if let Some(k) = models.catalogue_kinds.get(name) {
                if k != "depth" {
                    return Err(rejected(400, "INVALID_COMMAND", format!(
                        "Model {name:?} has kind {k:?} and cannot be loaded into the depth slot. Pass it under \"detector\" instead."
                    )));
                }
            }
            parts.push(name.clone());
        } else {
            parts.push("depth=off".into());
        }
        req.insert("depth".into(), match dep { Some(n) => json!(n), None => Value::Null });
    }

    let want_det = patch.detector.as_ref().unwrap_or(&models.resident.detector);
    let want_dep = patch.depth.as_ref().unwrap_or(&models.resident.depth);
    if *want_det == models.resident.detector && *want_dep == models.resident.depth && models.swap_state == "idle" {
        return Ok(json!({
            "detector": models.resident.detector,
            "depth": models.resident.depth,
            "changed": false,
            "generation": models.generation,
        }));
    }

    let loading_depth = matches!(patch.depth, Some(Some(_))) && patch.detector.is_none();
    // Pre-check ION budget from catalogue when the provider reports both ion and model sizes.
    if let Some([base, total, reserve]) = models.ion {
        let det_mb = want_det.as_ref().and_then(|n| models.catalogue.get(n).copied().flatten());
        let dep_mb = want_dep.as_ref().and_then(|n| models.catalogue.get(n).copied().flatten());
        let known = (want_det.is_none() || det_mb.is_some()) && (want_dep.is_none() || dep_mb.is_some());
        if known {
            let need = base + reserve + det_mb.unwrap_or(0.0) + dep_mb.unwrap_or(0.0);
            if need > total + 1e-9 {
                return Err(format_ion_budget_error(
                    &parts.join(" + "),
                    &json!({"need_mb": need, "limit_mb": total}),
                    want_dep.is_some(),
                    loading_depth,
                ));
            }
        }
    }

    let expected_det = patch.detector.as_ref().map(|o| o.as_deref());
    let expected_dep = patch.depth.as_ref().map(|o| o.as_deref());
    let pm = execute_swap(
        source,
        &Value::Object(req),
        &parts.join(" + "),
        expected_det,
        expected_dep,
        want_dep.is_some(),
        loading_depth,
    ).await?;
    Ok(json!({
        "detector": pm.resident.detector,
        "depth": pm.resident.depth,
        "changed": true,
        "generation": pm.generation,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kinds_and_busy_actions() {
        assert_eq!(parse_kind(&json!({"kind": "detect"})).unwrap(), "detect");
        assert_eq!(parse_kind(&json!({"kind": "segment"})).unwrap(), "segment");
        for bad in [json!({"kind": "obb"}), json!({}), json!({"kind": "detect", "x": 1}), json!([1]), json!({"kind": 3})] {
            assert!(matches!(parse_kind(&bad), Err(ArmError::Rejected { status: 400, .. })), "{bad}");
        }
        assert!(busy_action("grasp", "running").is_some());
        assert!(busy_action("measure_multiview", "running").is_some());
        assert!(busy_action("grasp", "done").is_none());
        assert!(busy_action("move_to", "running").is_none());
        assert!(DetectorDefaults::default().validate().is_ok());
        assert!(DetectorDefaults { detect: vec![String::new()], segment: vec![] }.validate().is_err());
    }

    #[test]
    fn md5_rfc1321_vectors() {
        assert_eq!(md5_bytes(b""), "d41d8cd98f00b204e9800998ecf8427e");
        assert_eq!(md5_bytes(b"a"), "0cc175b9c0f1b6a831c399e269772661");
        assert_eq!(md5_bytes(b"abc"), "900150983cd24fb0d6963f7d28e17f72");
        assert_eq!(md5_bytes(b"message digest"), "f96b697d7cb7938d525a2f31aaf161d0");
        assert_eq!(md5_bytes(b"abcdefghijklmnopqrstuvwxyz"), "c3fcd3d76192e4007dfb496cca67e13b");
    }

    #[test]
    fn manifest_matches_detector_defaults() {
        let disk_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("kits/recamera-b601dm/vision/models/models.tsv");
        let disk_tsv = fs::read_to_string(&disk_path).expect("models.tsv must exist");
        assert_eq!(disk_tsv, EMBEDDED_MODELS_TSV);
        let entries = parse_manifest(&disk_tsv);

        let by_role = |role: &str| -> &ManifestEntry {
            entries
                .iter()
                .find(|e| e.role == role)
                .unwrap_or_else(|| panic!("missing role {role} in models.tsv"))
        };
        let det = by_role("detector-detect");
        let seg = by_role("detector-segment");
        let seg_fb = by_role("detector-segment-fallback");
        let depth = by_role("depth");

        let defaults = DetectorDefaults::default();
        assert_eq!(defaults.detect, vec![model_file_stem(&det.file).to_string()]);
        assert_eq!(
            defaults.segment,
            vec![
                model_file_stem(&seg.file).to_string(),
                model_file_stem(&seg_fb.file).to_string(),
            ]
        );

        // Verify required manifest fields and tuned tabletop/seg384 hashes.
        for e in [det, seg, seg_fb, depth] {
            assert_eq!(e.md5.len(), 32, "bad md5 for {}", e.role);
            assert!(e.md5.chars().all(|c| c.is_ascii_hexdigit()), "non-hex md5 for {}", e.role);
            assert!(e.size > 0, "zero size for {}", e.role);
            assert!(e.ion_mb > 0.0, "zero ion_mb for {}", e.role);
        }
        assert_eq!(seg.ship, "bundled");
        assert_eq!(depth.ship, "bundled");
        assert!(seg.url.starts_with("https://"));
        assert!(depth.url.starts_with("https://"));
        assert_eq!(seg.md5, "5ad01b51727c5259abc1448f2bd0cec6");
        assert_eq!(depth.md5, "d0a6ee396c12680188a1cb8d56cd4a94");
    }

    #[test]
    fn evaluate_model_config_reports_bundled_cdn_user_missing_and_mismatched() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        let bundled = root.join("usr/share/rebot-teleop/models");
        let local = root.join("userdata/local/models");
        let os_dir = root.join("usr/share/supervisor/models");
        fs::create_dir_all(&bundled).unwrap();
        fs::create_dir_all(&local).unwrap();
        fs::create_dir_all(&os_dir).unwrap();

        let seg_bytes = b"seg384-test-payload";
        let dep_bytes = b"zipdepth-tabletop-payload";
        let det_bytes = b"os-detect-payload";
        fs::write(bundled.join("seg.cvimodel"), seg_bytes).unwrap();
        fs::write(bundled.join("dep.cvimodel"), dep_bytes).unwrap();
        fs::write(os_dir.join("det.cvimodel"), det_bytes).unwrap();

        let manifest = vec![
            ManifestEntry {
                role: "detector-segment".into(),
                file: "seg.cvimodel".into(),
                md5: md5_bytes(seg_bytes),
                size: seg_bytes.len() as u64,
                ion_mb: 13.1,
                url: "https://example.invalid/seg".into(),
                ship: "bundled".into(),
            },
            ManifestEntry {
                role: "detector-detect".into(),
                file: "det.cvimodel".into(),
                md5: md5_bytes(det_bytes),
                size: det_bytes.len() as u64,
                ion_mb: 8.5,
                url: "-".into(),
                ship: "os".into(),
            },
            ManifestEntry {
                role: "depth".into(),
                file: "dep.cvimodel".into(),
                md5: md5_bytes(dep_bytes),
                size: dep_bytes.len() as u64,
                ion_mb: 20.0,
                url: "https://example.invalid/dep".into(),
                ship: "bundled".into(),
            },
        ];

        let mode_all = json!({
            "model": {"name": "seg", "path": "/userdata/local/models/seg.cvimodel"},
            "depth_model": {"name": "dep", "path": "/userdata/local/models/dep.cvimodel"},
            "models": {"catalogue": [
                {"name": "det", "path": "/usr/share/supervisor/models/det.cvimodel"},
                {"name": "seg", "path": "/userdata/local/models/seg.cvimodel"},
                {"name": "dep", "path": "/userdata/local/models/dep.cvimodel"}
            ]}
        });

        // 1. Missing when symlinks have not been created yet
        let st = evaluate_model_config(&mode_all, &manifest, root);
        assert!(!st.optimal);
        assert_eq!(st.missing, vec!["seg.cvimodel", "dep.cvimodel"]);
        assert!(st.mismatched.is_empty());

        // 2. Linked to bundled -> optimal + source="bundled"
        #[cfg(unix)]
        {
            std::os::unix::fs::symlink(bundled.join("seg.cvimodel"), local.join("seg.cvimodel")).unwrap();
            std::os::unix::fs::symlink(bundled.join("dep.cvimodel"), local.join("dep.cvimodel")).unwrap();
            let st = evaluate_model_config(&mode_all, &manifest, root);
            assert!(st.optimal, "{st:?}");
            assert!(st.missing.is_empty());
            assert!(st.mismatched.is_empty());
            assert_eq!(st.source, "bundled");
        }

        // 3. Regular file recorded as cdn -> optimal + source="cdn"
        fs::remove_file(local.join("dep.cvimodel")).unwrap();
        fs::write(local.join("dep.cvimodel"), dep_bytes).unwrap();
        fs::write(local.join(".rebot-model-sources"), "seg.cvimodel\tbundled\ndep.cvimodel\tcdn\n").unwrap();
        let st = evaluate_model_config(&mode_all, &manifest, root);
        assert!(st.optimal);
        assert_eq!(st.source, "cdn");

        // 4. Wrong md5 -> not optimal, listed in mismatched
        fs::write(local.join("dep.cvimodel"), b"old-room-scale-bytes").unwrap();
        let st = evaluate_model_config(&mode_all, &manifest, root);
        assert!(!st.optimal);
        assert!(st.missing.is_empty());
        assert_eq!(st.mismatched, vec!["dep.cvimodel"]);

        // 5. User-supplied valid regular file without cdn marker -> optimal + source="user"
        fs::write(local.join("dep.cvimodel"), dep_bytes).unwrap();
        fs::remove_file(local.join(".rebot-model-sources")).unwrap();
        let st = evaluate_model_config(&mode_all, &manifest, root);
        assert!(st.optimal);
        assert_eq!(st.source, "user");
    }
}
