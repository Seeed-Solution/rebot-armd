//! Immutable policy package snapshot, release digest, strict manifest parsing,
//! and ONNX external-tensor-data rejection.

use std::collections::HashSet;
use std::fs::{self, File};
use std::io::Read;
use std::path::{Component, Path, PathBuf};

use serde_json::Value;

use crate::gravity::Sha256;

use super::mlp::RMLP_MAGIC;
use super::PolicyError;

pub const POLICY_FORMAT_V1: &str = "rebot-onnx-policy/v1";
pub const MAX_POLICY_JSON_BYTES: usize = 4 * 1024 * 1024;
/// Largest model file in a policy package. The largest RMLP the MLP backend accepts
/// (MAX_PARAMS = 2M f32 = 8.0 MB) fits; loading needs about twice the file in memory (file
/// bytes + weights), well inside the ~130 MB the SG2002 has free next to armd and
/// rebot-vision. ONNX models share the limit and the node / parameter budget in
/// `onnx_resource_budget`.
pub const MAX_MODEL_BYTES: usize = 8 * 1024 * 1024 + 4096;
pub const MAX_VISION_STALENESS_MS: u64 = 500;
pub const DEFAULT_VISION_MIN_SCORE: f32 = 0.3;
const MAX_ONNX_PROTO_DEPTH: usize = 32;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VisionSourceKind {
    Detector,
}

impl VisionSourceKind {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Detector => "detector",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VisionLayout {
    Top1BoxNormV1,
}

impl VisionLayout {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Top1BoxNormV1 => "top1_box_norm_v1",
        }
    }

    pub fn feature_dim(self) -> usize {
        match self {
            Self::Top1BoxNormV1 => 6,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VisionTargetLostPolicy {
    ZeroFeatures,
    Hold,
    Stop,
}

impl VisionTargetLostPolicy {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::ZeroFeatures => "zero_features",
            Self::Hold => "hold",
            Self::Stop => "stop",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VisionStalePolicy {
    Hold,
    Stop,
}

impl VisionStalePolicy {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Hold => "hold",
            Self::Stop => "stop",
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct VisionFeatureSpec {
    pub source: VisionSourceKind,
    pub layout: VisionLayout,
    pub target_label: Option<String>,
    pub min_score: f32,
    pub max_staleness_ms: u64,
    pub max_sync_skew_ms: u64,
    pub on_target_lost: VisionTargetLostPolicy,
    pub on_stale: VisionStalePolicy,
    pub require_model_generation: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnitMode {
    Rad,
    So101Norm,
}

impl UnitMode {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Rad => "rad",
            Self::So101Norm => "so101_norm",
        }
    }
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct NormalizationStats {
    pub state_mean: Option<Vec<f32>>,
    pub state_std: Option<Vec<f32>>,
    pub action_mean: Option<Vec<f32>>,
    pub action_std: Option<Vec<f32>>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct StrictPolicyContract {
    pub format: String,
    pub name: String,
    pub robot: Option<String>,
    pub model_file: String,
    pub unit_mode: UnitMode,
    pub action_dim: usize,
    pub action_space: String,
    pub joint_names: Vec<String>,
    pub normalization: NormalizationStats,
    pub fps: Option<f64>,
    pub max_step_rad: Option<f32>,
    pub max_first_step_rad: Option<f32>,
    pub vision: Option<VisionFeatureSpec>,
}

#[derive(Debug, Clone)]
pub struct PolicySnapshot {
    pub dir_path: PathBuf,
    pub policy_json_bytes: Vec<u8>,
    pub model_rel_posix: String,
    pub model_bytes: Vec<u8>,
    pub release_digest: String,
}

impl PolicySnapshot {
    /// Load `policy.json` and the referenced `model_file` once into memory.
    ///
    /// Enforces safe relative paths (rejecting absolute paths, `..`, and symlink escapes),
    /// rejects empty files and files exceeding size caps, rejects ONNX external tensor data,
    /// and computes the deterministic release digest from the exact in-memory bytes.
    pub fn load_from_dir(dir: &Path) -> Result<Self, PolicyError> {
        let dir_canon = fs::canonicalize(dir).map_err(|e| {
            PolicyError::Io(format!("cannot resolve policy directory '{}': {e}", dir.display()))
        })?;
        if !dir_canon.is_dir() {
            return Err(PolicyError::InvalidPath(format!(
                "policy path is not a directory: {}",
                dir_canon.display()
            )));
        }

        let policy_json_path = dir_canon.join("policy.json");
        let policy_json_canon = fs::canonicalize(&policy_json_path).map_err(|e| {
            PolicyError::Io(format!(
                "cannot resolve policy manifest '{}': {e}",
                policy_json_path.display()
            ))
        })?;
        if !policy_json_canon.starts_with(&dir_canon) || !policy_json_canon.is_file() {
            return Err(PolicyError::InvalidPath(format!(
                "policy.json '{}' escapes directory or is not a file",
                policy_json_canon.display()
            )));
        }

        let policy_json_bytes = read_bounded_file(&policy_json_canon, MAX_POLICY_JSON_BYTES, "policy.json")?;
        let raw_manifest: Value = serde_json::from_slice(&policy_json_bytes)
            .map_err(|e| PolicyError::InvalidManifest(format!("invalid JSON in policy.json: {e}")))?;
        let obj = raw_manifest
            .as_object()
            .ok_or_else(|| PolicyError::InvalidManifest("policy.json must be a JSON object".into()))?;

        let model_file = parse_model_file(obj)?;
        let (model_canon, model_rel_posix) = resolve_safe_relative_file(&dir_canon, &model_file)?;
        let model_bytes = read_bounded_file(&model_canon, MAX_MODEL_BYTES, &model_rel_posix)?;

        if is_onnx_artifact(&model_rel_posix, &model_bytes) {
            reject_external_onnx_data(&model_bytes)?;
        }

        let release_digest = compute_release_digest(&policy_json_bytes, &model_bytes, &model_rel_posix);

        Ok(Self {
            dir_path: dir_canon,
            policy_json_bytes,
            model_rel_posix,
            model_bytes,
            release_digest,
        })
    }
}

fn is_onnx_artifact(model_rel_posix: &str, model_bytes: &[u8]) -> bool {
    let lower = model_rel_posix.to_ascii_lowercase();
    if model_bytes.starts_with(RMLP_MAGIC) || lower.ends_with(".rmlp") || lower.ends_with(".mlp.json") {
        return false;
    }
    lower.ends_with(".onnx")
}

fn read_bounded_file(path: &Path, max_bytes: usize, label: &str) -> Result<Vec<u8>, PolicyError> {
    let file = File::open(path)
        .map_err(|e| PolicyError::Io(format!("failed to open {label} ('{}'): {e}", path.display())))?;
    let meta = file
        .metadata()
        .map_err(|e| PolicyError::Io(format!("failed to stat {label} ('{}'): {e}", path.display())))?;
    if !meta.is_file() {
        return Err(PolicyError::InvalidPath(format!("{label} is not a regular file")));
    }
    if meta.len() == 0 {
        return Err(PolicyError::InvalidArtifact(format!("{label} is empty")));
    }
    if meta.len() > max_bytes as u64 {
        return Err(PolicyError::ResourceBudget(format!(
            "{label} size ({} bytes) exceeds maximum allowed ({max_bytes} bytes)",
            meta.len()
        )));
    }

    let mut buf = Vec::with_capacity((meta.len() as usize).min(max_bytes));
    let mut limited = file.take((max_bytes as u64) + 1);
    limited
        .read_to_end(&mut buf)
        .map_err(|e| PolicyError::Io(format!("failed to read {label}: {e}")))?;

    if buf.is_empty() {
        return Err(PolicyError::InvalidArtifact(format!("{label} is empty")));
    }
    if buf.len() > max_bytes {
        return Err(PolicyError::ResourceBudget(format!(
            "{label} exceeds maximum allowed size ({max_bytes} bytes)"
        )));
    }
    Ok(buf)
}

fn parse_model_file(obj: &serde_json::Map<String, Value>) -> Result<String, PolicyError> {
    match obj.get("model_file") {
        None => Ok("model.onnx".to_string()),
        Some(v) => {
            let s = v.as_str().ok_or_else(|| {
                PolicyError::InvalidManifest("model_file must be a non-empty string".into())
            })?;
            let trimmed = s.trim();
            if trimmed.is_empty() {
                return Err(PolicyError::InvalidManifest("model_file must not be empty".into()));
            }
            if trimmed != s {
                return Err(PolicyError::InvalidPath(format!(
                    "model_file must not have leading/trailing whitespace: {s:?}"
                )));
            }
            Ok(trimmed.to_string())
        }
    }
}

fn resolve_safe_relative_file(dir_canon: &Path, rel: &str) -> Result<(PathBuf, String), PolicyError> {
    if rel.contains('\0') || rel.starts_with('/') || rel.starts_with('\\') {
        return Err(PolicyError::InvalidPath(format!(
            "model_file must be a safe relative path, got {rel:?}"
        )));
    }
    // Reject Windows-style drive prefixes on all platforms for consistent cross-platform safety.
    let bytes = rel.as_bytes();
    if bytes.len() >= 2 && bytes[0].is_ascii_alphabetic() && bytes[1] == b':' {
        return Err(PolicyError::InvalidPath(format!(
            "model_file must not contain a drive prefix, got {rel:?}"
        )));
    }

    let rel_path = Path::new(rel);
    if rel_path.is_absolute() {
        return Err(PolicyError::InvalidPath(format!(
            "model_file must be relative, got {rel:?}"
        )));
    }
    for comp in rel_path.components() {
        match comp {
            Component::Normal(_) | Component::CurDir => {}
            Component::ParentDir | Component::RootDir | Component::Prefix(_) => {
                return Err(PolicyError::InvalidPath(format!(
                    "model_file contains forbidden path component: {rel:?}"
                )));
            }
        }
    }

    let joined = dir_canon.join(rel_path);
    let model_canon = fs::canonicalize(&joined).map_err(|e| {
        PolicyError::Io(format!(
            "model_file '{}' not found or cannot be resolved: {e}",
            joined.display()
        ))
    })?;
    let stripped = model_canon.strip_prefix(dir_canon).map_err(|_| {
        PolicyError::InvalidPath(format!(
            "model_file {rel:?} escapes policy directory '{}'",
            dir_canon.display()
        ))
    })?;
    if stripped.as_os_str().is_empty() || !model_canon.is_file() {
        return Err(PolicyError::InvalidPath(format!(
            "model_file {rel:?} is not a file inside policy directory"
        )));
    }

    let posix_parts: Vec<String> = stripped
        .components()
        .map(|c| match c {
            Component::Normal(os) => os.to_string_lossy().into_owned(),
            other => other.as_os_str().to_string_lossy().into_owned(),
        })
        .collect();
    Ok((model_canon, posix_parts.join("/")))
}

/// Deterministic SHA-256 digest matching `rebot_companion/release_gate.py` lines 273-293 byte-for-byte.
pub fn compute_release_digest(
    policy_json_bytes: &[u8],
    model_bytes: &[u8],
    model_rel_posix: &str,
) -> String {
    let mut hasher = Sha256::new();
    for (rel_name, content) in [("policy.json", policy_json_bytes), (model_rel_posix, model_bytes)] {
        hasher.update(rel_name.as_bytes());
        hasher.update(b"\x00");
        let len_str = content.len().to_string();
        hasher.update(len_str.as_bytes());
        hasher.update(b"\x00");
        hasher.update(content);
    }
    hasher.finalize_hex()
}

impl StrictPolicyContract {
    pub fn vision_feature_dim(&self) -> usize {
        self.vision.as_ref().map_or(0, |v| v.layout.feature_dim())
    }

    pub fn expected_input_dim(&self) -> usize {
        self.action_dim + self.vision_feature_dim()
    }

    /// Parse and strictly validate `policy.json` from raw JSON bytes without permissive defaults.
    pub fn parse(policy_json_bytes: &[u8]) -> Result<Self, PolicyError> {
        if policy_json_bytes.is_empty() {
            return Err(PolicyError::InvalidManifest("policy.json is empty".into()));
        }
        let root: Value = serde_json::from_slice(policy_json_bytes)
            .map_err(|e| PolicyError::InvalidManifest(format!("invalid JSON in policy.json: {e}")))?;
        let obj = root
            .as_object()
            .ok_or_else(|| PolicyError::InvalidManifest("policy.json must be a JSON object".into()))?;

        let format = require_non_empty_str(obj.get("format"), "format")?;
        if format != POLICY_FORMAT_V1 {
            return Err(PolicyError::InvalidManifest(format!(
                "unsupported policy format {format:?}; expected {POLICY_FORMAT_V1:?}"
            )));
        }

        let name = match obj.get("name") {
            None => String::new(),
            Some(v) => v
                .as_str()
                .ok_or_else(|| PolicyError::InvalidManifest("name must be a string".into()))?
                .to_string(),
        };

        let robot = match obj.get("robot") {
            None | Some(Value::Null) => None,
            Some(v) => Some(require_non_empty_str(Some(v), "robot")?),
        };

        let model_file = parse_model_file(obj)?;
        // Validate model_file lexical path rules even without a filesystem directory.
        validate_lexical_relative_path(&model_file)?;

        let unit_mode_raw = require_non_empty_str(obj.get("unit_mode"), "unit_mode")?;
        let unit_mode = match unit_mode_raw.as_str() {
            "rad" => UnitMode::Rad,
            "so101_norm" => UnitMode::So101Norm,
            other => {
                return Err(PolicyError::InvalidManifest(format!(
                    "unsupported unit_mode {other:?}; must be 'rad' or 'so101_norm'"
                )));
            }
        };

        let action_dim_val = obj
            .get("action_dim")
            .ok_or_else(|| PolicyError::InvalidManifest("missing required field 'action_dim'".into()))?;
        let action_dim_u64 = action_dim_val
            .as_u64()
            .ok_or_else(|| PolicyError::InvalidManifest("action_dim must be integer 6 or 7".into()))?;
        let action_dim = match action_dim_u64 {
            6 => 6usize,
            7 => 7usize,
            other => {
                return Err(PolicyError::InvalidManifest(format!(
                    "action_dim must be 6 or 7, got {other}"
                )));
            }
        };

        let action_space = parse_strict_action_space(obj)?;
        if action_space != "joint_positions" {
            return Err(PolicyError::InvalidManifest(format!(
                "unsupported action_space {action_space:?}; must be 'joint_positions'"
            )));
        }

        if let Some(inputs) = obj.get("inputs") {
            validate_inputs_shape(inputs, action_dim)?;
        }
        if let Some(outputs) = obj.get("outputs") {
            validate_outputs_shape(outputs, action_dim)?;
        }

        let joint_names = parse_strict_joint_names(obj, action_dim)?;
        let normalization = parse_normalization(obj.get("normalization"), action_dim)?;

        let fps = parse_optional_positive_f64(obj.get("fps"), "fps")?;
        let max_step_rad = parse_optional_positive_f32(obj.get("max_step_rad"), "max_step_rad")?;
        let max_first_step_rad =
            parse_optional_positive_f32(obj.get("max_first_step_rad"), "max_first_step_rad")?;
        let vision = parse_vision_spec(obj.get("vision"))?;

        Ok(Self {
            format,
            name,
            robot,
            model_file,
            unit_mode,
            action_dim,
            action_space,
            joint_names,
            normalization,
            fps,
            max_step_rad,
            max_first_step_rad,
            vision,
        })
    }
}

fn parse_vision_spec(val: Option<&Value>) -> Result<Option<VisionFeatureSpec>, PolicyError> {
    let Some(raw) = val else {
        return Ok(None);
    };
    if raw.is_null() {
        return Ok(None);
    }
    let obj = raw
        .as_object()
        .ok_or_else(|| PolicyError::InvalidManifest("vision must be a JSON object".into()))?;

    let source_raw = require_non_empty_str(obj.get("source"), "vision.source")?;
    let source = match source_raw.as_str() {
        "detector" => VisionSourceKind::Detector,
        other => {
            return Err(PolicyError::InvalidManifest(format!(
                "unsupported vision.source {other:?}; must be 'detector'"
            )));
        }
    };

    let layout_raw = require_non_empty_str(obj.get("layout"), "vision.layout")?;
    let layout = match layout_raw.as_str() {
        "top1_box_norm_v1" => VisionLayout::Top1BoxNormV1,
        other => {
            return Err(PolicyError::InvalidManifest(format!(
                "unsupported vision.layout {other:?}; must be 'top1_box_norm_v1'"
            )));
        }
    };

    let target_label = match obj.get("target_label") {
        None | Some(Value::Null) => None,
        Some(v) => {
            let s = v.as_str().ok_or_else(|| {
                PolicyError::InvalidManifest("vision.target_label must be a non-empty string".into())
            })?;
            if s.trim().is_empty() {
                return Err(PolicyError::InvalidManifest(
                    "vision.target_label must be a non-empty string".into(),
                ));
            }
            Some(s.to_string())
        }
    };

    let min_score = match obj.get("min_score") {
        None | Some(Value::Null) => DEFAULT_VISION_MIN_SCORE,
        Some(v) => {
            let f64_val = v.as_f64().ok_or_else(|| {
                PolicyError::InvalidManifest(
                    "vision.min_score must be a finite number in [0.0, 1.0]".into(),
                )
            })?;
            let f32_val = f64_val as f32;
            if !f64_val.is_finite() || !f32_val.is_finite() {
                return Err(PolicyError::NonFinite(
                    "vision.min_score is non-finite".into(),
                ));
            }
            if !(0.0..=1.0).contains(&f64_val) || !(0.0..=1.0).contains(&f32_val) {
                return Err(PolicyError::InvalidManifest(format!(
                    "vision.min_score must be in [0.0, 1.0], got {f64_val}"
                )));
            }
            f32_val
        }
    };

    let max_staleness_raw = obj.get("max_staleness_ms").ok_or_else(|| {
        PolicyError::InvalidManifest("missing required field 'vision.max_staleness_ms'".into())
    })?;
    let max_staleness_ms = max_staleness_raw.as_u64().ok_or_else(|| {
        PolicyError::InvalidManifest(format!(
            "vision.max_staleness_ms must be an integer in 1..={MAX_VISION_STALENESS_MS}"
        ))
    })?;
    if !(1..=MAX_VISION_STALENESS_MS).contains(&max_staleness_ms) {
        return Err(PolicyError::InvalidManifest(format!(
            "vision.max_staleness_ms must be in 1..={MAX_VISION_STALENESS_MS}, got {max_staleness_ms}"
        )));
    }

    let max_sync_skew_ms = match obj.get("max_sync_skew_ms") {
        None | Some(Value::Null) => max_staleness_ms,
        Some(v) => {
            let skew = v.as_u64().ok_or_else(|| {
                PolicyError::InvalidManifest(format!(
                    "vision.max_sync_skew_ms must be an integer in 1..={MAX_VISION_STALENESS_MS}"
                ))
            })?;
            if !(1..=MAX_VISION_STALENESS_MS).contains(&skew) {
                return Err(PolicyError::InvalidManifest(format!(
                    "vision.max_sync_skew_ms must be in 1..={MAX_VISION_STALENESS_MS}, got {skew}"
                )));
            }
            skew
        }
    };

    let on_target_lost = match obj.get("on_target_lost") {
        None | Some(Value::Null) => VisionTargetLostPolicy::Hold,
        Some(v) => match require_non_empty_str(Some(v), "vision.on_target_lost")?.as_str() {
            "zero_features" => VisionTargetLostPolicy::ZeroFeatures,
            "hold" => VisionTargetLostPolicy::Hold,
            "stop" => VisionTargetLostPolicy::Stop,
            other => {
                return Err(PolicyError::InvalidManifest(format!(
                    "unsupported vision.on_target_lost {other:?}; must be 'zero_features', 'hold', or 'stop'"
                )));
            }
        },
    };

    let on_stale = match obj.get("on_stale") {
        None | Some(Value::Null) => VisionStalePolicy::Stop,
        Some(v) => match require_non_empty_str(Some(v), "vision.on_stale")?.as_str() {
            "hold" => VisionStalePolicy::Hold,
            "stop" => VisionStalePolicy::Stop,
            other => {
                return Err(PolicyError::InvalidManifest(format!(
                    "unsupported vision.on_stale {other:?}; must be 'hold' or 'stop'"
                )));
            }
        },
    };

    let require_model_generation = match obj.get("require_model_generation") {
        None | Some(Value::Null) => true,
        Some(v) => v.as_bool().ok_or_else(|| {
            PolicyError::InvalidManifest(
                "vision.require_model_generation must be a boolean".into(),
            )
        })?,
    };

    Ok(Some(VisionFeatureSpec {
        source,
        layout,
        target_label,
        min_score,
        max_staleness_ms,
        max_sync_skew_ms,
        on_target_lost,
        on_stale,
        require_model_generation,
    }))
}

fn validate_lexical_relative_path(rel: &str) -> Result<(), PolicyError> {
    if rel.contains('\0') || rel.starts_with('/') || rel.starts_with('\\') {
        return Err(PolicyError::InvalidPath(format!(
            "model_file must be a safe relative path, got {rel:?}"
        )));
    }
    let bytes = rel.as_bytes();
    if bytes.len() >= 2 && bytes[0].is_ascii_alphabetic() && bytes[1] == b':' {
        return Err(PolicyError::InvalidPath(format!(
            "model_file must not contain a drive prefix, got {rel:?}"
        )));
    }
    let rel_path = Path::new(rel);
    if rel_path.is_absolute() {
        return Err(PolicyError::InvalidPath(format!(
            "model_file must be relative, got {rel:?}"
        )));
    }
    for comp in rel_path.components() {
        match comp {
            Component::Normal(_) | Component::CurDir => {}
            Component::ParentDir | Component::RootDir | Component::Prefix(_) => {
                return Err(PolicyError::InvalidPath(format!(
                    "model_file contains forbidden path component: {rel:?}"
                )));
            }
        }
    }
    Ok(())
}

fn require_non_empty_str(val: Option<&Value>, label: &str) -> Result<String, PolicyError> {
    let raw = val.ok_or_else(|| PolicyError::InvalidManifest(format!("missing required field '{label}'")))?;
    let s = raw
        .as_str()
        .ok_or_else(|| PolicyError::InvalidManifest(format!("{label} must be a non-empty string")))?;
    let trimmed = s.trim();
    if trimmed.is_empty() {
        return Err(PolicyError::InvalidManifest(format!(
            "{label} must be a non-empty string"
        )));
    }
    Ok(trimmed.to_string())
}

fn parse_strict_action_space(obj: &serde_json::Map<String, Value>) -> Result<String, PolicyError> {
    let top_space = match obj.get("action_space") {
        None => None,
        Some(Value::Null) => {
            return Err(PolicyError::InvalidManifest(
                "action_space cannot be null".into(),
            ));
        }
        Some(v) => Some(require_non_empty_str(Some(v), "action_space")?),
    };

    let output_space = match obj
        .get("outputs")
        .and_then(|o| o.as_object())
        .and_then(|o| o.get("action"))
        .and_then(|a| a.as_object())
        .and_then(|a| a.get("space"))
    {
        None => None,
        Some(Value::Null) => {
            return Err(PolicyError::InvalidManifest(
                "outputs.action.space cannot be null".into(),
            ));
        }
        Some(v) => Some(require_non_empty_str(Some(v), "outputs.action.space")?),
    };

    match (top_space, output_space) {
        (Some(top), Some(out)) => {
            if top != out {
                return Err(PolicyError::InvalidManifest(format!(
                    "conflicting action_space ({top:?}) and outputs.action.space ({out:?})"
                )));
            }
            Ok(top)
        }
        (Some(top), None) => Ok(top),
        (None, Some(out)) => Ok(out),
        (None, None) => Err(PolicyError::InvalidManifest(
            "missing explicit action_space (expected top-level 'action_space' or 'outputs.action.space' == 'joint_positions')".into(),
        )),
    }
}

fn validate_inputs_shape(inputs: &Value, action_dim: usize) -> Result<(), PolicyError> {
    let inputs_obj = inputs
        .as_object()
        .ok_or_else(|| PolicyError::InvalidManifest("inputs must be an object".into()))?;
    if let Some(state) = inputs_obj.get("state") {
        let state_obj = state
            .as_object()
            .ok_or_else(|| PolicyError::InvalidManifest("inputs.state must be an object".into()))?;
        if let Some(shape) = state_obj.get("shape") {
            let arr = shape.as_array().ok_or_else(|| {
                PolicyError::InvalidManifest("inputs.state.shape must be an array".into())
            })?;
            if let Some(last) = arr.last() {
                let d = last.as_u64().ok_or_else(|| {
                    PolicyError::InvalidManifest(
                        "inputs.state.shape elements must be positive integers".into(),
                    )
                })? as usize;
                if d != action_dim {
                    return Err(PolicyError::DimensionMismatch {
                        expected: action_dim,
                        actual: d,
                    });
                }
            }
        }
    }
    Ok(())
}

fn validate_outputs_shape(outputs: &Value, action_dim: usize) -> Result<(), PolicyError> {
    let outputs_obj = outputs
        .as_object()
        .ok_or_else(|| PolicyError::InvalidManifest("outputs must be an object".into()))?;
    if let Some(action) = outputs_obj.get("action") {
        let action_obj = action
            .as_object()
            .ok_or_else(|| PolicyError::InvalidManifest("outputs.action must be an object".into()))?;
        if let Some(shape) = action_obj.get("shape") {
            let arr = shape.as_array().ok_or_else(|| {
                PolicyError::InvalidManifest("outputs.action.shape must be an array".into())
            })?;
            if let Some(last) = arr.last() {
                let d = last.as_u64().ok_or_else(|| {
                    PolicyError::InvalidManifest(
                        "outputs.action.shape elements must be positive integers".into(),
                    )
                })? as usize;
                if d != action_dim {
                    return Err(PolicyError::DimensionMismatch {
                        expected: action_dim,
                        actual: d,
                    });
                }
            }
        }
    }
    Ok(())
}

fn parse_strict_joint_names(
    obj: &serde_json::Map<String, Value>,
    action_dim: usize,
) -> Result<Vec<String>, PolicyError> {
    let top_jn = match obj.get("joint_names") {
        None | Some(Value::Null) => None,
        Some(v) => Some(parse_joint_name_array(v, "joint_names")?),
    };
    let state_jn = match obj
        .get("inputs")
        .and_then(|i| i.as_object())
        .and_then(|i| i.get("state"))
        .and_then(|s| s.as_object())
        .and_then(|s| s.get("joint_names"))
    {
        None | Some(Value::Null) => None,
        Some(v) => Some(parse_joint_name_array(v, "inputs.state.joint_names")?),
    };

    let names = match (top_jn, state_jn) {
        (Some(top), Some(state)) => {
            if top != state {
                return Err(PolicyError::InvalidManifest(
                    "conflicting joint_names between top-level and inputs.state.joint_names".into(),
                ));
            }
            top
        }
        (Some(top), None) => top,
        (None, Some(state)) => state,
        (None, None) => {
            return Err(PolicyError::InvalidManifest(
                "missing required 'joint_names' list".into(),
            ));
        }
    };

    if names.len() != action_dim {
        return Err(PolicyError::DimensionMismatch {
            expected: action_dim,
            actual: names.len(),
        });
    }
    Ok(names)
}

fn parse_joint_name_array(val: &Value, label: &str) -> Result<Vec<String>, PolicyError> {
    let arr = val
        .as_array()
        .ok_or_else(|| PolicyError::InvalidManifest(format!("{label} must be a non-empty array")))?;
    if arr.is_empty() {
        return Err(PolicyError::InvalidManifest(format!(
            "{label} must not be empty"
        )));
    }
    let mut out = Vec::with_capacity(arr.len());
    let mut seen = HashSet::with_capacity(arr.len());
    for (idx, item) in arr.iter().enumerate() {
        let s = item.as_str().ok_or_else(|| {
            PolicyError::InvalidManifest(format!("{label}[{idx}] must be a non-empty string"))
        })?;
        if s.trim().is_empty() {
            return Err(PolicyError::InvalidManifest(format!(
                "{label}[{idx}] must be a non-empty string"
            )));
        }
        if !seen.insert(s.to_string()) {
            return Err(PolicyError::InvalidManifest(format!(
                "{label} contains duplicate joint name {s:?}"
            )));
        }
        out.push(s.to_string());
    }
    Ok(out)
}

fn parse_normalization(
    val: Option<&Value>,
    action_dim: usize,
) -> Result<NormalizationStats, PolicyError> {
    let Some(raw) = val else {
        return Ok(NormalizationStats::default());
    };
    if raw.is_null() {
        return Ok(NormalizationStats::default());
    }
    let obj = raw
        .as_object()
        .ok_or_else(|| PolicyError::InvalidManifest("normalization must be a JSON object".into()))?;

    let (state_mean, state_std) = parse_stat_pair(
        obj,
        "state_mean",
        "state_std",
        "state",
        action_dim,
    )?;
    let (action_mean, action_std) = parse_stat_pair(
        obj,
        "action_mean",
        "action_std",
        "action",
        action_dim,
    )?;

    Ok(NormalizationStats {
        state_mean,
        state_std,
        action_mean,
        action_std,
    })
}

fn parse_stat_pair(
    obj: &serde_json::Map<String, Value>,
    flat_mean_key: &str,
    flat_std_key: &str,
    nested_key: &str,
    expected_len: usize,
) -> Result<(Option<Vec<f32>>, Option<Vec<f32>>), PolicyError> {
    let flat_mean = match obj.get(flat_mean_key) {
        None | Some(Value::Null) => None,
        Some(v) => Some(parse_f32_vec(v, flat_mean_key, expected_len, false)?),
    };
    let flat_std = match obj.get(flat_std_key) {
        None | Some(Value::Null) => None,
        Some(v) => Some(parse_f32_vec(v, flat_std_key, expected_len, true)?),
    };
    if flat_mean.is_some() != flat_std.is_some() {
        return Err(PolicyError::InvalidManifest(format!(
            "both {flat_mean_key} and {flat_std_key} must be provided together"
        )));
    }

    let (nested_mean, nested_std) = match obj.get(nested_key) {
        None | Some(Value::Null) => (None, None),
        Some(v) => {
            let sub = v.as_object().ok_or_else(|| {
                PolicyError::InvalidManifest(format!("normalization.{nested_key} must be an object"))
            })?;
            let m = match sub.get("mean") {
                None | Some(Value::Null) => None,
                Some(mv) => Some(parse_f32_vec(
                    mv,
                    &format!("normalization.{nested_key}.mean"),
                    expected_len,
                    false,
                )?),
            };
            let s = match sub.get("std") {
                None | Some(Value::Null) => None,
                Some(sv) => Some(parse_f32_vec(
                    sv,
                    &format!("normalization.{nested_key}.std"),
                    expected_len,
                    true,
                )?),
            };
            if m.is_some() != s.is_some() {
                return Err(PolicyError::InvalidManifest(format!(
                    "both normalization.{nested_key}.mean and normalization.{nested_key}.std must be provided together"
                )));
            }
            (m, s)
        }
    };

    match ((flat_mean, flat_std), (nested_mean, nested_std)) {
        ((Some(fm), Some(fs)), (Some(nm), Some(ns))) => {
            if fm != nm || fs != ns {
                return Err(PolicyError::InvalidManifest(format!(
                    "conflicting flat and nested normalization stats for {nested_key}"
                )));
            }
            Ok((Some(fm), Some(fs)))
        }
        ((Some(fm), Some(fs)), (None, None)) => Ok((Some(fm), Some(fs))),
        ((None, None), (Some(nm), Some(ns))) => Ok((Some(nm), Some(ns))),
        ((None, None), (None, None)) => Ok((None, None)),
        _ => unreachable!(),
    }
}

fn parse_f32_vec(
    val: &Value,
    label: &str,
    expected_len: usize,
    require_positive: bool,
) -> Result<Vec<f32>, PolicyError> {
    let arr = val
        .as_array()
        .ok_or_else(|| PolicyError::InvalidManifest(format!("{label} must be an array")))?;
    if arr.len() != expected_len {
        return Err(PolicyError::DimensionMismatch {
            expected: expected_len,
            actual: arr.len(),
        });
    }
    let mut out = Vec::with_capacity(arr.len());
    for (idx, item) in arr.iter().enumerate() {
        let f64_val = item.as_f64().ok_or_else(|| {
            PolicyError::InvalidManifest(format!("{label}[{idx}] must be a finite number"))
        })?;
        let f32_val = f64_val as f32;
        if !f64_val.is_finite() || !f32_val.is_finite() {
            return Err(PolicyError::NonFinite(format!("{label}[{idx}] is non-finite")));
        }
        if require_positive && f32_val <= 0.0 {
            return Err(PolicyError::InvalidManifest(format!(
                "{label}[{idx}] must be strictly > 0.0, got {f32_val}"
            )));
        }
        out.push(f32_val);
    }
    Ok(out)
}

fn parse_optional_positive_f32(val: Option<&Value>, label: &str) -> Result<Option<f32>, PolicyError> {
    let Some(raw) = val else {
        return Ok(None);
    };
    if raw.is_null() {
        return Ok(None);
    }
    let f64_val = raw
        .as_f64()
        .ok_or_else(|| PolicyError::InvalidManifest(format!("{label} must be a finite number")))?;
    let f32_val = f64_val as f32;
    if !f64_val.is_finite() || !f32_val.is_finite() {
        return Err(PolicyError::NonFinite(format!("{label} is non-finite")));
    }
    if f32_val <= 0.0 {
        return Err(PolicyError::InvalidManifest(format!(
            "{label} must be strictly > 0.0, got {f32_val}"
        )));
    }
    Ok(Some(f32_val))
}

fn parse_optional_positive_f64(val: Option<&Value>, label: &str) -> Result<Option<f64>, PolicyError> {
    let Some(raw) = val else {
        return Ok(None);
    };
    if raw.is_null() {
        return Ok(None);
    }
    let f64_val = raw
        .as_f64()
        .ok_or_else(|| PolicyError::InvalidManifest(format!("{label} must be a finite number")))?;
    if !f64_val.is_finite() {
        return Err(PolicyError::NonFinite(format!("{label} is non-finite")));
    }
    if f64_val <= 0.0 {
        return Err(PolicyError::InvalidManifest(format!(
            "{label} must be strictly > 0.0, got {f64_val}"
        )));
    }
    Ok(Some(f64_val))
}

// ---------------------------------------------------------------------------
// Pure-Rust ONNX Protobuf External-Tensor-Data Scanner
// ---------------------------------------------------------------------------

enum PbValue<'a> {
    Varint(u64),
    LenDelimited(&'a [u8]),
    Fixed,
}

fn read_varint(buf: &[u8], pos: &mut usize) -> Result<u64, PolicyError> {
    let mut result: u64 = 0;
    let mut shift: u32 = 0;
    loop {
        if *pos >= buf.len() {
            return Err(PolicyError::InvalidArtifact(
                "truncated protobuf varint in ONNX model".into(),
            ));
        }
        let b = buf[*pos];
        *pos += 1;
        result |= ((b & 0x7F) as u64) << shift;
        if (b & 0x80) == 0 {
            return Ok(result);
        }
        shift += 7;
        if shift >= 70 {
            return Err(PolicyError::InvalidArtifact(
                "malformed protobuf varint in ONNX model".into(),
            ));
        }
    }
}

fn next_pb_field<'a>(
    buf: &'a [u8],
    pos: &mut usize,
) -> Result<Option<(u64, u8, PbValue<'a>)>, PolicyError> {
    if *pos >= buf.len() {
        return Ok(None);
    }
    let tag = read_varint(buf, pos)?;
    let field_num = tag >> 3;
    let wire_type = (tag & 0x07) as u8;
    if field_num == 0 {
        return Err(PolicyError::InvalidArtifact(
            "invalid protobuf field number 0 in ONNX model".into(),
        ));
    }
    let val = match wire_type {
        0 => PbValue::Varint(read_varint(buf, pos)?),
        1 => {
            if *pos + 8 > buf.len() {
                return Err(PolicyError::InvalidArtifact(
                    "truncated 64-bit protobuf field in ONNX model".into(),
                ));
            }
            *pos += 8;
            PbValue::Fixed
        }
        2 => {
            let length_u64 = read_varint(buf, pos)?;
            let length = usize::try_from(length_u64).map_err(|_| {
                PolicyError::InvalidArtifact("protobuf field length overflows usize".into())
            })?;
            if *pos + length > buf.len() {
                return Err(PolicyError::InvalidArtifact(
                    "truncated length-delimited protobuf field in ONNX model".into(),
                ));
            }
            let sub = &buf[*pos..*pos + length];
            *pos += length;
            PbValue::LenDelimited(sub)
        }
        5 => {
            if *pos + 4 > buf.len() {
                return Err(PolicyError::InvalidArtifact(
                    "truncated 32-bit protobuf field in ONNX model".into(),
                ));
            }
            *pos += 4;
            PbValue::Fixed
        }
        other => {
            return Err(PolicyError::InvalidArtifact(format!(
                "unsupported protobuf wire type {other} in ONNX model"
            )));
        }
    };
    Ok(Some((field_num, wire_type, val)))
}

fn check_tensor_proto(buf: &[u8]) -> Result<(), PolicyError> {
    let mut tensor_name: Option<String> = None;
    let mut has_external_data = false;
    let mut data_location: Option<u64> = None;

    let mut pos = 0;
    while let Some((field_num, wire_type, val)) = next_pb_field(buf, &mut pos)? {
        match (field_num, wire_type, val) {
            (8, 2, PbValue::LenDelimited(sub)) => {
                tensor_name = Some(String::from_utf8_lossy(sub).into_owned());
            }
            (13, 2, PbValue::LenDelimited(_)) => {
                has_external_data = true;
            }
            (14, 0, PbValue::Varint(loc)) => {
                data_location = Some(loc);
            }
            _ => {}
        }
    }

    let label = tensor_name
        .as_deref()
        .map(|n| format!(" '{n}'"))
        .unwrap_or_default();

    if let Some(loc) = data_location {
        if loc != 0 {
            return Err(PolicyError::ExternalOnnxData(format!(
                "ONNX tensor{label} declares data_location={loc} (EXTERNAL)"
            )));
        }
    }
    if has_external_data {
        return Err(PolicyError::ExternalOnnxData(format!(
            "ONNX tensor{label} declares external_data"
        )));
    }
    Ok(())
}

fn check_sparse_tensor_proto(buf: &[u8]) -> Result<(), PolicyError> {
    let mut pos = 0;
    while let Some((field_num, wire_type, val)) = next_pb_field(buf, &mut pos)? {
        if (field_num == 1 || field_num == 2) && wire_type == 2 {
            if let PbValue::LenDelimited(sub) = val {
                check_tensor_proto(sub)?;
            }
        }
    }
    Ok(())
}

fn check_attribute_proto(buf: &[u8], depth: usize) -> Result<(), PolicyError> {
    let mut pos = 0;
    while let Some((field_num, wire_type, val)) = next_pb_field(buf, &mut pos)? {
        if wire_type != 2 {
            continue;
        }
        if let PbValue::LenDelimited(sub) = val {
            match field_num {
                5 | 10 => check_tensor_proto(sub)?,
                6 | 11 => check_graph_proto(sub, depth + 1)?,
                22 | 23 => check_sparse_tensor_proto(sub)?,
                _ => {}
            }
        }
    }
    Ok(())
}

fn check_node_proto(buf: &[u8], depth: usize) -> Result<(), PolicyError> {
    let mut pos = 0;
    while let Some((field_num, wire_type, val)) = next_pb_field(buf, &mut pos)? {
        if field_num == 5 && wire_type == 2 {
            if let PbValue::LenDelimited(sub) = val {
                check_attribute_proto(sub, depth)?;
            }
        }
    }
    Ok(())
}

fn check_graph_proto(buf: &[u8], depth: usize) -> Result<(), PolicyError> {
    if depth > MAX_ONNX_PROTO_DEPTH {
        return Err(PolicyError::InvalidArtifact(
            "ONNX graph nesting exceeds maximum supported depth".into(),
        ));
    }
    let mut pos = 0;
    while let Some((field_num, wire_type, val)) = next_pb_field(buf, &mut pos)? {
        if wire_type != 2 {
            continue;
        }
        if let PbValue::LenDelimited(sub) = val {
            match field_num {
                1 => check_node_proto(sub, depth)?,
                5 => check_tensor_proto(sub)?,
                15 => check_sparse_tensor_proto(sub)?,
                _ => {}
            }
        }
    }
    Ok(())
}

fn check_function_proto(buf: &[u8], depth: usize) -> Result<(), PolicyError> {
    let mut pos = 0;
    while let Some((field_num, wire_type, val)) = next_pb_field(buf, &mut pos)? {
        if wire_type != 2 {
            continue;
        }
        if let PbValue::LenDelimited(sub) = val {
            match field_num {
                7 => check_node_proto(sub, depth)?,
                11 => check_attribute_proto(sub, depth)?,
                _ => {}
            }
        }
    }
    Ok(())
}

/// Fail closed if any `TensorProto` in an ONNX `ModelProto` declares external tensor data.
pub fn reject_external_onnx_data(model_bytes: &[u8]) -> Result<(), PolicyError> {
    if model_bytes.is_empty() {
        return Err(PolicyError::InvalidArtifact("ONNX model bytes must be non-empty".into()));
    }
    let mut pos = 0;
    while let Some((field_num, wire_type, val)) = next_pb_field(model_bytes, &mut pos)? {
        if wire_type != 2 {
            continue;
        }
        if let PbValue::LenDelimited(sub) = val {
            match field_num {
                7 => check_graph_proto(sub, 0)?,
                25 => check_function_proto(sub, 0)?,
                _ => {}
            }
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// ONNX resource budget (checked before tract parses or optimizes the model)
// ---------------------------------------------------------------------------

/// Most nodes in an ONNX graph accepted for the tract backend.
pub const MAX_ONNX_NODES: usize = 512;
/// Most initializer / constant elements (all tensors together) for the tract backend; the
/// same parameter budget as the MLP backend ([`super::mlp::MAX_PARAMS`]).
pub const MAX_ONNX_PARAM_ELEMENTS: u64 = super::mlp::MAX_PARAMS as u64;
/// Most elements in all intermediate tensors of the typed graph (one inference).
pub const MAX_ONNX_ACTIVATION_ELEMENTS: u64 = 1_000_000;
const MAX_ONNX_TENSOR_RANK: usize = 8;

/// Counts from [`onnx_resource_budget`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct OnnxBudget {
    pub nodes: usize,
    pub param_elements: u64,
}

fn budget_err(msg: String) -> PolicyError {
    PolicyError::ResourceBudget(msg)
}

/// Element count of one `TensorProto` from its `dims` (field 1, packed or not).
fn tensor_elements(buf: &[u8]) -> Result<u64, PolicyError> {
    let mut dims: Vec<u64> = Vec::new();
    let mut pos = 0;
    while let Some((field_num, wire_type, val)) = next_pb_field(buf, &mut pos)? {
        match (field_num, wire_type, val) {
            (1, 0, PbValue::Varint(d)) => dims.push(d),
            (1, 2, PbValue::LenDelimited(packed)) => {
                let mut p = 0;
                while p < packed.len() {
                    dims.push(read_varint(packed, &mut p)?);
                }
            }
            _ => {}
        }
        if dims.len() > MAX_ONNX_TENSOR_RANK {
            return Err(budget_err(format!("ONNX tensor rank exceeds {MAX_ONNX_TENSOR_RANK}")));
        }
    }
    dims.iter().try_fold(1u64, |acc, &d| acc.checked_mul(d)).ok_or_else(|| budget_err("ONNX tensor element count overflows".into()))
}

fn budget_attribute(buf: &[u8], b: &mut OnnxBudget) -> Result<(), PolicyError> {
    let mut pos = 0;
    while let Some((field_num, wire_type, val)) = next_pb_field(buf, &mut pos)? {
        if let (2, PbValue::LenDelimited(sub)) = (wire_type, val) {
            match field_num {
                5 | 10 => add_params(b, tensor_elements(sub)?)?,
                6 | 11 => return Err(budget_err("ONNX subgraphs (If / Loop / Scan bodies) are not supported by the device backend".into())),
                22 | 23 => return Err(budget_err("ONNX sparse tensors are not supported by the device backend".into())),
                _ => {}
            }
        }
    }
    Ok(())
}

fn add_params(b: &mut OnnxBudget, n: u64) -> Result<(), PolicyError> {
    b.param_elements = b.param_elements.checked_add(n).ok_or_else(|| budget_err("ONNX parameter count overflows".into()))?;
    if b.param_elements > MAX_ONNX_PARAM_ELEMENTS {
        return Err(budget_err(format!(
            "ONNX initializers hold more than {MAX_ONNX_PARAM_ELEMENTS} elements"
        )));
    }
    Ok(())
}

/// Count nodes and parameter elements of an ONNX `ModelProto` without building it, and
/// refuse models over [`MAX_ONNX_NODES`] / [`MAX_ONNX_PARAM_ELEMENTS`], with subgraphs,
/// sparse tensors or model-local functions.
pub fn onnx_resource_budget(model_bytes: &[u8]) -> Result<OnnxBudget, PolicyError> {
    let mut b = OnnxBudget::default();
    let mut graphs = 0;
    let mut pos = 0;
    while let Some((field_num, wire_type, val)) = next_pb_field(model_bytes, &mut pos)? {
        let (2, PbValue::LenDelimited(sub)) = (wire_type, val) else { continue };
        match field_num {
            7 => {
                graphs += 1;
                let mut gpos = 0;
                while let Some((gf, gw, gv)) = next_pb_field(sub, &mut gpos)? {
                    let (2, PbValue::LenDelimited(gsub)) = (gw, gv) else { continue };
                    match gf {
                        1 => {
                            b.nodes += 1;
                            if b.nodes > MAX_ONNX_NODES {
                                return Err(budget_err(format!("ONNX graph has more than {MAX_ONNX_NODES} nodes")));
                            }
                            let mut npos = 0;
                            while let Some((nf, nw, nv)) = next_pb_field(gsub, &mut npos)? {
                                if let (5, 2, PbValue::LenDelimited(attr)) = (nf, nw, nv) {
                                    budget_attribute(attr, &mut b)?;
                                }
                            }
                        }
                        5 => add_params(&mut b, tensor_elements(gsub)?)?,
                        15 => return Err(budget_err("ONNX sparse initializers are not supported by the device backend".into())),
                        _ => {}
                    }
                }
            }
            25 => return Err(budget_err("ONNX model-local functions are not supported by the device backend".into())),
            _ => {}
        }
    }
    if graphs != 1 {
        return Err(PolicyError::InvalidArtifact(format!("ONNX model must contain exactly one graph, found {graphs}")));
    }
    Ok(b)
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::tempdir;

    fn base_manifest() -> serde_json::Value {
        serde_json::json!({
            "format": "rebot-onnx-policy/v1",
            "name": "test_policy",
            "robot": "so-arm101",
            "unit_mode": "so101_norm",
            "action_dim": 6,
            "action_space": "joint_positions",
            "joint_names": ["j1", "j2", "j3", "j4", "j5", "j6"],
            "model_file": "model.rmlp",
            "normalization": {
                "state_mean": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "state_std": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
                "action_mean": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "action_std": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
            }
        })
    }

    #[test]
    fn release_digest_matches_python_golden_vector() {
        // Golden output from `rebot_companion.release_gate.compute_policy_digest_from_bytes(
        //   b'{"hello":1}', b'\x08\x01', 'sub/model.onnx')`
        let d = compute_release_digest(br#"{"hello":1}"#, b"\x08\x01", "sub/model.onnx");
        assert_eq!(d, "e2c302076321dddd3806d044bc73024b31a3c852ef9fdc00d14e6ca961f14803");
    }

    #[test]
    fn strict_contract_accepts_valid_manifest_and_nested_norm_and_output_space() {
        let bytes = serde_json::to_vec(&base_manifest()).unwrap();
        let c = StrictPolicyContract::parse(&bytes).unwrap();
        assert_eq!(c.unit_mode, UnitMode::So101Norm);
        assert_eq!(c.action_dim, 6);
        assert_eq!(c.joint_names.len(), 6);
        assert!(c.normalization.state_mean.is_some());

        // Also accept outputs.action.space + nested normalization.{state,action}.{mean,std}
        let nested = serde_json::json!({
            "format": "rebot-onnx-policy/v1",
            "unit_mode": "rad",
            "action_dim": 7,
            "outputs": { "action": { "name": "action", "shape": [1, 7], "space": "joint_positions" } },
            "joint_names": ["j1", "j2", "j3", "j4", "j5", "j6", "j7"],
            "normalization": {
                "state": { "mean": vec![0.0; 7], "std": vec![2.0; 7] },
                "action": { "mean": vec![1.0; 7], "std": vec![0.5; 7] }
            },
            "max_step_rad": 0.25,
            "max_first_step_rad": 0.1
        });
        let c2 = StrictPolicyContract::parse(&serde_json::to_vec(&nested).unwrap()).unwrap();
        assert_eq!(c2.unit_mode, UnitMode::Rad);
        assert_eq!(c2.action_dim, 7);
        assert_eq!(c2.normalization.state_std.unwrap(), vec![2.0; 7]);
        assert_eq!(c2.max_step_rad, Some(0.25));
        assert_eq!(c2.max_first_step_rad, Some(0.1));
    }

    #[test]
    fn strict_contract_rejects_missing_and_invalid_fields() {
        // Missing unit_mode
        let mut m = base_manifest();
        m.as_object_mut().unwrap().remove("unit_mode");
        assert!(StrictPolicyContract::parse(&serde_json::to_vec(&m).unwrap()).is_err());

        // Unknown unit_mode
        let mut m = base_manifest();
        m["unit_mode"] = serde_json::json!("deg");
        assert!(StrictPolicyContract::parse(&serde_json::to_vec(&m).unwrap()).is_err());

        // Missing joint_names
        let mut m = base_manifest();
        m.as_object_mut().unwrap().remove("joint_names");
        assert!(StrictPolicyContract::parse(&serde_json::to_vec(&m).unwrap()).is_err());

        // Duplicate joint_names
        let mut m = base_manifest();
        m["joint_names"] = serde_json::json!(["j1", "j1", "j3", "j4", "j5", "j6"]);
        assert!(StrictPolicyContract::parse(&serde_json::to_vec(&m).unwrap()).is_err());

        // Missing action_space
        let mut m = base_manifest();
        m.as_object_mut().unwrap().remove("action_space");
        assert!(StrictPolicyContract::parse(&serde_json::to_vec(&m).unwrap()).is_err());

        // Non-joint_positions action_space
        let mut m = base_manifest();
        m["action_space"] = serde_json::json!("cartesian_xy_target");
        assert!(StrictPolicyContract::parse(&serde_json::to_vec(&m).unwrap()).is_err());

        // std <= 0
        let mut m = base_manifest();
        m["normalization"]["state_std"] = serde_json::json!([1.0, 0.0, 1.0, 1.0, 1.0, 1.0]);
        assert!(StrictPolicyContract::parse(&serde_json::to_vec(&m).unwrap()).is_err());

        // Unpaired normalization stats
        let mut m = base_manifest();
        m["normalization"].as_object_mut().unwrap().remove("action_std");
        assert!(StrictPolicyContract::parse(&serde_json::to_vec(&m).unwrap()).is_err());

        // Wrong action_dim
        let mut m = base_manifest();
        m["action_dim"] = serde_json::json!(5);
        assert!(StrictPolicyContract::parse(&serde_json::to_vec(&m).unwrap()).is_err());

        // Non-positive max_step_rad
        let mut m = base_manifest();
        m["max_step_rad"] = serde_json::json!(0.0);
        assert!(StrictPolicyContract::parse(&serde_json::to_vec(&m).unwrap()).is_err());
    }

    #[test]
    fn snapshot_rejects_path_traversal_empty_files_and_external_onnx_data() {
        let dir = tempdir().unwrap();
        let mut m = base_manifest();
        m["model_file"] = serde_json::json!("../escape.onnx");
        fs::write(dir.path().join("policy.json"), serde_json::to_vec(&m).unwrap()).unwrap();
        let err = PolicySnapshot::load_from_dir(dir.path()).unwrap_err();
        assert!(matches!(err, PolicyError::InvalidPath(_)));

        // Absolute path
        m["model_file"] = serde_json::json!("/etc/passwd");
        fs::write(dir.path().join("policy.json"), serde_json::to_vec(&m).unwrap()).unwrap();
        let err = PolicySnapshot::load_from_dir(dir.path()).unwrap_err();
        assert!(matches!(err, PolicyError::InvalidPath(_)));

        // Empty model file
        m["model_file"] = serde_json::json!("model.onnx");
        fs::write(dir.path().join("policy.json"), serde_json::to_vec(&m).unwrap()).unwrap();
        fs::write(dir.path().join("model.onnx"), b"").unwrap();
        let err = PolicySnapshot::load_from_dir(dir.path()).unwrap_err();
        assert!(matches!(err, PolicyError::InvalidArtifact(_)));

        // External data in ONNX TensorProto (data_location = 1)
        fn encode_varint(mut v: u64, out: &mut Vec<u8>) {
            loop {
                let b = (v & 0x7F) as u8;
                v >>= 7;
                if v == 0 {
                    out.push(b);
                    break;
                }
                out.push(b | 0x80);
            }
        }
        fn encode_ld(field: u64, data: &[u8], out: &mut Vec<u8>) {
            encode_varint((field << 3) | 2, out);
            encode_varint(data.len() as u64, out);
            out.extend_from_slice(data);
        }
        let mut tensor = Vec::new();
        encode_ld(8, b"ext_w", &mut tensor);
        encode_varint(14 << 3, &mut tensor); // data_location = field 14, wire 0
        encode_varint(1, &mut tensor); // EXTERNAL = 1
        let mut graph = Vec::new();
        encode_ld(5, &tensor, &mut graph); // initializer = field 5
        let mut model = Vec::new();
        encode_ld(7, &graph, &mut model); // graph = field 7

        fs::write(dir.path().join("model.onnx"), &model).unwrap();
        let err = PolicySnapshot::load_from_dir(dir.path()).unwrap_err();
        assert!(matches!(err, PolicyError::ExternalOnnxData(_)));
    }
}
