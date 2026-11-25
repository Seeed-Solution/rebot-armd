//! Core policy runtime: immutable package snapshot, strict manifest validation,
//! bidirectional unit conversion, normalization, and dual inference backends
//! (pure-Rust `MlpBackend` and optional `tract-onnx` `TractBackend`).

pub mod manifest;
pub mod mlp;
pub mod qualification;
pub mod release_gate;
pub mod runner;
#[cfg(feature = "policy-tract")]
pub mod tract_backend;
pub mod units;
pub mod vision;

use thiserror::Error;

pub use manifest::{
    compute_release_digest, reject_external_onnx_data, validate_chunking_horizon, ChunkingSpec,
    NormalizationStats, PolicySnapshot, StrictPolicyContract, UnitMode, VisionFeatureSpec,
    VisionLayout, VisionSourceKind, VisionStalePolicy, VisionTargetLostPolicy,
    DEFAULT_VISION_MIN_SCORE, MAX_CHUNK_SIZE, MAX_MODEL_BYTES, MAX_OPEN_LOOP_HORIZON_MS,
    MAX_POLICY_JSON_BYTES, MAX_TEMPORAL_ENSEMBLE_K, MAX_VISION_STALENESS_MS,
};
pub use mlp::{
    encode_rmlp_bytes, Activation, MlpBackend, MlpLayerSpec, RMLP_MAGIC, MAX_DIM, MAX_LAYERS,
    MAX_MACS, MAX_MLP_JSON_BYTES, MAX_PARAMS,
};
pub use release_gate::{
    canonical_robot_id, load_release_gate_json, ActiveDeploymentContext, AuthorizationTier,
    ReleaseGate, VerifiedReleaseGate, EMPTY_CALIBRATION_DIGEST, MIN_SIM_EVAL_SEEDS,
    RELEASE_GATE_SCHEMA_VERSION, SUPERVISED_FIRST_RUN_MAX_SPEED_SCALE,
};
pub use runner::{
    compute_policy_deadline, max_cached_chunk_source_age_ms, max_policy_input_age_ms,
    parse_policy_start_request, prepare_policy, spawn_policy_runner, validate_policy_fps,
    ChunkFreshnessLimits, ChunkRunContext, ChunkSource, CommitGateDecision, CommitGateParams,
    PolicyAuthorization, PolicyMode, PolicyRun, PolicyRunStart, PolicyStartRequest,
    PolicyStepStats, PreparedPolicy, SpeedGateReport, WarmupInferUs, DEFAULT_MAX_FIRST_STEP_RAD,
    DEFAULT_POLICY_FPS, MAX_POLICY_FPS, MAX_POLICY_SECONDS, MIN_LIVE_POLICY_FPS, MIN_POLICY_FPS,
    period_us_for_fps, POLICY_STATS_RING, SPEED_GATE_PERIOD_FRACTION, WARMUP_INFERENCES,
    WARMUP_MAX_TOTAL,
};
#[cfg(not(target_arch = "wasm32"))]
pub use runner::{spawn_executor_policy_runner, ExecutorPolicyRunParams};
#[cfg(feature = "policy-tract")]
pub use tract_backend::TractBackend;
pub use units::{
    denormalize_action, denormalize_action_into, normalize_state, normalize_state_into,
    policy_units_to_rad, rad_to_policy_units, JointPermutation,
};
pub use vision::{
    evaluate_vision_frame, extract_top1_box_norm_v1, extract_top1_target, DetectionSourceAdapter,
    DetectorTargetIdentity, PolicyVisionFrame, PolicyVisionSource, VisionFeatureCollector,
    VisionStepDecision, DETECTOR_TARGET_SAME_IOU_MIN,
};

#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum PolicyError {
    #[error("I/O error: {0}")]
    Io(String),
    #[error("invalid policy path: {0}")]
    InvalidPath(String),
    #[error("invalid policy manifest: {0}")]
    InvalidManifest(String),
    #[error("external ONNX tensor data is forbidden: {0}")]
    ExternalOnnxData(String),
    #[error("unit conversion error: {0}")]
    InvalidUnits(String),
    #[error("dimension mismatch: expected {expected}, got {actual}")]
    DimensionMismatch { expected: usize, actual: usize },
    #[error("non-finite value: {0}")]
    NonFinite(String),
    #[error("invalid model artifact: {0}")]
    InvalidArtifact(String),
    #[error("failed to load inference backend: {0}")]
    BackendLoad(String),
    #[error("unsupported inference backend: {0}")]
    UnsupportedBackend(String),
    #[error("inference execution failed: {0}")]
    InferenceFailed(String),
    #[error("release gate verification failed: {0}")]
    ReleaseGate(String),
    #[error("policy exceeds the device resource budget: {0}")]
    ResourceBudget(String),
}

/// Synchronous, single-vector inference backend for state-to-action policies.
pub trait PolicyBackend: Send {
    fn backend_name(&self) -> &'static str;
    fn input_dim(&self) -> usize;
    fn output_dim(&self) -> usize;
    fn infer(&mut self, input: &[f32], output: &mut [f32]) -> Result<(), PolicyError>;
}

/// Dispatch and construct the appropriate [`PolicyBackend`] for a loaded snapshot and contract.
///
/// Validates that the backend's input dimension matches `contract.expected_input_dim()`
/// and output dimension matches `contract.action_dim`.
pub fn load_backend(
    snapshot: &PolicySnapshot,
    contract: &StrictPolicyContract,
) -> Result<Box<dyn PolicyBackend>, PolicyError> {
    let model_bytes = &snapshot.model_bytes;
    let rel_lower = snapshot.model_rel_posix.to_ascii_lowercase();

    let backend: Box<dyn PolicyBackend> = if model_bytes.starts_with(RMLP_MAGIC) || rel_lower.ends_with(".rmlp") {
        Box::new(MlpBackend::from_rmlp_bytes(model_bytes)?)
    } else if rel_lower.ends_with(".mlp.json") || model_bytes.trim_ascii_start().starts_with(b"{") {
        Box::new(MlpBackend::from_json_bytes(model_bytes)?)
    } else if rel_lower.ends_with(".onnx") {
        load_onnx_backend(model_bytes)?
    } else {
        return Err(PolicyError::UnsupportedBackend(format!(
            "unrecognized model artifact '{}'; expected .rmlp, .mlp.json, or .onnx",
            snapshot.model_rel_posix
        )));
    };

    let expected_in = contract.expected_input_dim();
    if backend.input_dim() != expected_in {
        return Err(PolicyError::DimensionMismatch {
            expected: expected_in,
            actual: backend.input_dim(),
        });
    }
    let expected_out = contract.expected_output_dim();
    if backend.output_dim() != expected_out {
        return Err(PolicyError::DimensionMismatch {
            expected: expected_out,
            actual: backend.output_dim(),
        });
    }

    Ok(backend)
}

#[cfg(feature = "policy-tract")]
fn load_onnx_backend(model_bytes: &[u8]) -> Result<Box<dyn PolicyBackend>, PolicyError> {
    Ok(Box::new(TractBackend::from_onnx_bytes(model_bytes)?))
}

#[cfg(not(feature = "policy-tract"))]
fn load_onnx_backend(_model_bytes: &[u8]) -> Result<Box<dyn PolicyBackend>, PolicyError> {
    Err(PolicyError::UnsupportedBackend(
        "raw .onnx models require the `policy-tract` feature to be enabled; \
         rebuild with `--features policy-tract` or provide an explicit MLP artifact (.rmlp / .mlp.json)"
            .to_string(),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use tempfile::tempdir;

    #[test]
    fn load_backend_dispatches_rmlp_and_json_and_handles_onnx_feature_gate() {
        let dir = tempdir().unwrap();
        let layers = vec![MlpLayerSpec::new(
            6,
            6,
            Activation::None,
            (0..36).map(|i| if i % 7 == 0 { 1.0 } else { 0.0 }).collect(),
            vec![0.0; 6],
        )];
        let rmlp = encode_rmlp_bytes(&layers).unwrap();
        let manifest = serde_json::json!({
            "format": "rebot-onnx-policy/v1",
            "unit_mode": "rad",
            "action_dim": 6,
            "action_space": "joint_positions",
            "joint_names": ["j1", "j2", "j3", "j4", "j5", "j6"],
            "model_file": "model.rmlp"
        });
        fs::write(dir.path().join("policy.json"), serde_json::to_vec(&manifest).unwrap()).unwrap();
        fs::write(dir.path().join("model.rmlp"), &rmlp).unwrap();

        let snap = PolicySnapshot::load_from_dir(dir.path()).unwrap();
        let contract = StrictPolicyContract::parse(&snap.policy_json_bytes).unwrap();
        let mut backend = load_backend(&snap, &contract).unwrap();
        assert_eq!(backend.backend_name(), "mlp");
        let inp = [0.1f32, -0.2, 0.3, -0.4, 0.5, -0.6];
        let mut out = [0.0f32; 6];
        backend.infer(&inp, &mut out).unwrap();
        assert_eq!(out, inp);

        // JSON MLP dispatch
        let json_dir = tempdir().unwrap();
        let mlp_json = serde_json::json!({
            "format": "rebot-mlp/v1",
            "layers": [{
                "in_dim": 6,
                "out_dim": 6,
                "activation": "none",
                "weights": (0..36).map(|i| if i % 7 == 0 { 2.0 } else { 0.0 }).collect::<Vec<f32>>(),
                "bias": vec![0.0; 6]
            }]
        });
        let manifest_j = serde_json::json!({
            "format": "rebot-onnx-policy/v1",
            "unit_mode": "rad",
            "action_dim": 6,
            "action_space": "joint_positions",
            "joint_names": ["j1", "j2", "j3", "j4", "j5", "j6"],
            "model_file": "weights.mlp.json"
        });
        fs::write(json_dir.path().join("policy.json"), serde_json::to_vec(&manifest_j).unwrap()).unwrap();
        fs::write(json_dir.path().join("weights.mlp.json"), serde_json::to_vec(&mlp_json).unwrap()).unwrap();

        let snap_j = PolicySnapshot::load_from_dir(json_dir.path()).unwrap();
        let contract_j = StrictPolicyContract::parse(&snap_j.policy_json_bytes).unwrap();
        let mut backend_j = load_backend(&snap_j, &contract_j).unwrap();
        assert_eq!(backend_j.backend_name(), "mlp");
        backend_j.infer(&inp, &mut out).unwrap();
        for i in 0..6 {
            assert!((out[i] - 2.0 * inp[i]).abs() < 1e-6);
        }

        // ONNX without policy-tract gives UnsupportedBackend
        #[cfg(not(feature = "policy-tract"))]
        {
            let onnx_snap = PolicySnapshot {
                dir_path: dir.path().to_path_buf(),
                policy_json_bytes: snap.policy_json_bytes.clone(),
                model_rel_posix: "model.onnx".to_string(),
                model_bytes: vec![0x08, 0x08],
                release_digest: String::new(),
            };
            let err = load_backend(&onnx_snap, &contract).err().unwrap();
            assert!(matches!(err, PolicyError::UnsupportedBackend(_)));
        }
    }
}
