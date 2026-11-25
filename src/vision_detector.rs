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
use std::sync::atomic::{AtomicUsize, Ordering};
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

/// Path of the REST route.
pub const PATH: &str = "/api/v1/vision/detector";
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

/// Single-view depth measurements (`POST /api/v1/grasp/measure`) in flight: they trigger the
/// detector, so a switch waits until they are done.
static MEASURES: AtomicUsize = AtomicUsize::new(0);

/// Held by a running depth measurement ([`measure_in_flight`]).
pub struct MeasureGuard(());

impl Drop for MeasureGuard {
    fn drop(&mut self) { MEASURES.fetch_sub(1, Ordering::SeqCst); }
}

/// Marks a depth measurement as running until the guard is dropped.
pub fn measure_in_flight() -> MeasureGuard {
    MEASURES.fetch_add(1, Ordering::SeqCst);
    MeasureGuard(())
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

/// Switch the provider's detector to the default model of the requested kind. Reply
/// `{kind, model, changed, generation}`.
pub async fn switch(supervisor: &Arc<ArmSupervisor>, source: &dyn DetectionSource, body: &Value) -> Result<Value, ArmError> {
    let kind = parse_kind(body)?;
    let defaults = supervisor.inner().read().await.config.vision_detectors.clone().unwrap_or_default();
    if let Some(a) = supervisor.hub().latest().and_then(|s| s.obs.action.as_ref().and_then(|a| busy_action(&a.kind, &a.state))) {
        return Err(busy(a));
    }
    if MEASURES.load(Ordering::SeqCst) > 0 { return Err(busy("a depth measurement is running")); }
    // one model change at a time (grasp task preflights take the same lock)
    let Ok(_serial) = crate::grasp::MODEL_SWITCH.try_lock() else {
        return Err(busy("another model switch is in progress"));
    };
    let unreachable = |e: String| rejected(503, "DETECTION_UNAVAILABLE", format!("Cannot reach the vision service ({e}). Check that the camera app (rebot-vision) is running, then try again."));
    let mode = source.mode_info().await.map_err(unreachable)?;
    let Some(models) = mode.models.as_ref() else {
        return Err(rejected(422, "UNSUPPORTED", "The vision service cannot switch models (no models in GET /api/mode). Update rebot-teleop on the device."));
    };
    let Some(name) = defaults.names(kind).iter().find(|n| models.catalogue.contains_key(*n)).cloned() else {
        let have: Vec<&str> = models.catalogue.keys().map(String::as_str).collect();
        return Err(rejected(422, "MODEL_NOT_FOUND", format!(
            "No default {kind} model of this armd ({}) is in the vision service's catalogue ({}). Install the model on the device or set vision_detectors.{kind} in the arm profile.",
            defaults.names(kind).join(", "), if have.is_empty() { "empty".to_string() } else { have.join(", ") }
        )));
    };
    let reply = |changed: bool, generation: u64| json!({"kind": kind, "model": name, "changed": changed, "generation": generation});
    if models.resident.detector.as_deref() == Some(name.as_str()) && models.swap_state != "failed" {
        return Ok(reply(false, models.generation));
    }
    let (status, v) = source.set_models(&json!({"detector": name})).await.map_err(unreachable)?;
    let error = v.get("error").and_then(Value::as_str).unwrap_or("refused");
    match status {
        200..=299 => {}
        404 => return Err(rejected(422, "UNSUPPORTED", "The vision service cannot switch models (POST /api/models answered 404). Update rebot-teleop on the device.")),
        409 if error == "SWAP_BUSY" => return Err(busy("the vision service is switching models")),
        409 if error == "ION_BUDGET" => {
            let mb = |k: &str| v.get(k).and_then(Value::as_f64);
            let need = match (mb("need_mb"), mb("limit_mb")) {
                (Some(n), Some(l)) => format!(" (needs {n:.1} MB, limit {l:.1} MB)"),
                _ => String::new(),
            };
            return Err(rejected(409, "ION_BUDGET", format!(
                "The vision service refused {name}: not enough camera memory next to the loaded models (ION_BUDGET{need}). Keep the current kind, or unload the depth model first (a grasp task with models.depth null), then switch again."
            )));
        }
        400..=499 => return Err(rejected(422, "MODEL_SWITCH_REFUSED", format!("The vision service refused to load {name}: {error}. Check the model on the device."))),
        _ => return Err(rejected(502, "DETECTION_UNAVAILABLE", format!("The vision service answered {status} to the model switch. Check rebot-vision on the device, then try again."))),
    }
    let Some(generation) = v.get("generation").and_then(Value::as_u64) else {
        return Err(rejected(502, "DETECTION_UNAVAILABLE", "The vision service's model switch reply has no generation. Update rebot-teleop on the device."));
    };
    let deadline = Instant::now() + SWITCH_TIMEOUT;
    loop {
        let m = source.mode_info().await.map_err(unreachable)?;
        if let Some(pm) = m.models.as_ref().filter(|pm| pm.generation >= generation && matches!(pm.swap_state.as_str(), "idle" | "failed")) {
            if pm.swap_state == "failed" {
                let why = pm.swap_error.as_deref().unwrap_or("unknown error");
                let code = if why.starts_with("ION_BUDGET") { "ION_BUDGET" } else { "MODEL_SWITCH_FAILED" };
                return Err(rejected(409, code, format!("The vision service could not load {name} ({why}). Keep the current kind, or check the model and the free camera memory on the device, then switch again.")));
            }
            return Ok(reply(true, pm.generation));
        }
        if Instant::now() >= deadline {
            return Err(rejected(504, "DETECTION_UNAVAILABLE", format!(
                "The vision service did not finish loading {name} within {} s. Check rebot-vision on the device, then try again.", SWITCH_TIMEOUT.as_secs()
            )));
        }
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
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
}
