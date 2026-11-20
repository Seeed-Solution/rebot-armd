//! Async policy runner, prepared policy pipeline, and supervisor `PolicyRun` state.

use std::path::Path;
use std::sync::Arc;
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::api::sandbox::validate_pack_id;
use crate::model::{ArmConfig, JointLimit, PolicyObservation};
use crate::motion::MotionError;
use crate::supervisor::{ArmError, ArmSupervisor};

use super::manifest::{PolicySnapshot, StrictPolicyContract, VisionStalePolicy};
use super::release_gate::VerifiedReleaseGate;
use super::units::{
    denormalize_action_into, normalize_state_into, policy_units_to_rad, rad_to_policy_units,
    JointPermutation,
};
use super::vision::{
    evaluate_vision_frame, PolicyVisionSource, VisionFeatureCollector, VisionStepDecision,
};
use super::{load_backend, PolicyBackend, PolicyError};

pub const DEFAULT_POLICY_FPS: f64 = 20.0;
pub const MIN_POLICY_FPS: f64 = 1.0;
pub const MIN_LIVE_POLICY_FPS: f64 = 10.0;
pub const MAX_POLICY_FPS: f64 = 50.0;
pub const MAX_POLICY_SECONDS: f64 = 86_400.0;
pub const DEFAULT_MAX_FIRST_STEP_RAD: f32 = 0.25;
/// Bounds of [`max_policy_input_age_ms`].
pub const MIN_POLICY_INPUT_AGE_MS: u64 = 50;
pub const MAX_POLICY_INPUT_AGE_MS: u64 = 200;

/// Largest age of the joint positions a live target was computed from, measured from the
/// driver's feedback capture to the commit under the supervisor lock: two policy periods
/// (one for inference, one of slack), clamped to 50..=200 ms. 200 ms equals the servo
/// motion-frame timeout; 50 ms leaves room for feedback age and scheduling at 50 fps.
/// A target older than this is refused and the run ends with `STALE_INPUT`.
pub fn max_policy_input_age_ms(fps: f64) -> u64 {
    if !fps.is_finite() || fps <= 0.0 {
        return MAX_POLICY_INPUT_AGE_MS;
    }
    ((2000.0 / fps).round() as u64).clamp(MIN_POLICY_INPUT_AGE_MS, MAX_POLICY_INPUT_AGE_MS)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PolicyMode {
    Shadow,
    Live,
}

impl PolicyMode {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Shadow => "shadow",
            Self::Live => "live",
        }
    }
}

/// Active policy run tracked by `SupervisorInner`.
#[derive(Debug, Clone)]
pub struct PolicyRun {
    pub action_id: u64,
    pub epoch_id: u64,
    pub connection_id: String,
    pub lease_generation: u64,
    pub policy_id: String,
    pub artifact_digest: String,
    pub backend: String,
    pub mode: PolicyMode,
    pub steps: u64,
    pub last_input_age_ms: Option<u64>,
    pub last_vision_age_ms: Option<u64>,
    pub vision_frame_id: Option<u64>,
    pub last_shadow_target: Option<Vec<f32>>,
    pub last_live_target: Option<Vec<f32>>,
    pub max_first_step_rad: f32,
    pub max_step_rad: Option<f32>,
    pub i_am_at_the_arm: bool,
    pub verified_release_gate: Option<VerifiedReleaseGate>,
    pub stop_reason: Option<String>,
    /// [`max_policy_input_age_ms`] for the run's rate (set by the runner; until then the
    /// bound for [`MIN_LIVE_POLICY_FPS`]).
    pub max_input_age_ms: u64,
}

impl PolicyRun {
    pub fn to_observation(&self, active: bool) -> PolicyObservation {
        PolicyObservation {
            active,
            action_id: self.action_id,
            policy_id: self.policy_id.clone(),
            artifact_digest: self.artifact_digest.clone(),
            backend: self.backend.clone(),
            mode: self.mode.as_str().to_string(),
            steps: self.steps,
            last_input_age_ms: self.last_input_age_ms,
            last_vision_age_ms: self.last_vision_age_ms,
            vision_frame_id: self.vision_frame_id,
            last_shadow_target: self.last_shadow_target.clone(),
            stop_reason: self.stop_reason.clone(),
        }
    }
}

/// Parameters for `SupervisorInner::begin_policy_run`.
#[derive(Debug, Clone)]
pub struct PolicyRunStart {
    pub epoch_id: u64,
    pub connection_id: String,
    pub lease_generation: u64,
    pub policy_id: String,
    pub artifact_digest: String,
    pub backend: String,
    pub mode: PolicyMode,
    pub sim_only: bool,
    pub max_first_step_rad: Option<f32>,
    pub max_step_rad: Option<f32>,
    pub tag: Option<String>,
    pub i_am_at_the_arm: bool,
    pub verified_release_gate: Option<VerifiedReleaseGate>,
}

/// Parsed WS `/api/v1/action/policy/start` request.
#[derive(Debug, Clone)]
pub struct PolicyStartRequest {
    pub policy_id: String,
    pub mode: PolicyMode,
    pub sim_only: bool,
    pub max_seconds: Option<f64>,
    pub fps: Option<f64>,
    pub tag: Option<String>,
    pub i_am_at_the_arm: bool,
    pub release_gate: Option<Value>,
}

/// Validate an effective policy rate for the requested execution mode.
pub fn validate_policy_fps(mode: PolicyMode, fps: f64) -> Result<f64, MotionError> {
    if !fps.is_finite() || fps <= 0.0 {
        return Err(MotionError::InvalidCommand(
            "fps must be a positive finite number".into(),
        ));
    }
    let min_fps = match mode {
        PolicyMode::Live => MIN_LIVE_POLICY_FPS,
        PolicyMode::Shadow => MIN_POLICY_FPS,
    };
    if fps < min_fps || fps > MAX_POLICY_FPS {
        return Err(MotionError::InvalidCommand(format!(
            "fps for {} mode must be between {min_fps} and {MAX_POLICY_FPS}, got {fps}",
            mode.as_str()
        )));
    }
    Ok(fps)
}

/// Validate `max_seconds` and convert it into a monotonic `Instant` deadline without panicking.
pub fn compute_policy_deadline(max_seconds: Option<f64>) -> Result<Option<Instant>, MotionError> {
    let Some(secs) = max_seconds else {
        return Ok(None);
    };
    if !secs.is_finite() || secs <= 0.0 || secs > MAX_POLICY_SECONDS {
        return Err(MotionError::InvalidCommand(format!(
            "max_seconds must be a positive finite number <= {MAX_POLICY_SECONDS}"
        )));
    }
    let dur = Duration::try_from_secs_f64(secs).map_err(|_| {
        MotionError::InvalidCommand("max_seconds is out of representable duration range".into())
    })?;
    let deadline = Instant::now().checked_add(dur).ok_or_else(|| {
        MotionError::InvalidCommand("max_seconds overflows system clock instant".into())
    })?;
    Ok(Some(deadline))
}

pub fn parse_policy_start_request(body: &Value) -> Result<PolicyStartRequest, MotionError> {
    let policy_id = body
        .get("policy_id")
        .and_then(Value::as_str)
        .ok_or_else(|| MotionError::InvalidCommand("policy_id must be a non-empty string".into()))?;
    if !validate_pack_id(policy_id) {
        return Err(MotionError::InvalidCommand(format!(
            "policy_id '{policy_id}' is not a valid pack id"
        )));
    }

    let (mode, sim_only) = match body.get("mode").and_then(Value::as_str) {
        Some("shadow") => (PolicyMode::Shadow, false),
        Some("live_sim") => (PolicyMode::Live, true),
        Some("live") | Some("live_gated") => (PolicyMode::Live, false),
        Some(other) => {
            return Err(MotionError::InvalidCommand(format!(
                "unsupported policy mode '{other}'; expected 'shadow' or 'live'"
            )));
        }
        None => {
            return Err(MotionError::InvalidCommand(
                "mode must be 'shadow' or 'live'".into(),
            ));
        }
    };

    let max_seconds = match body.get("max_seconds") {
        None | Some(Value::Null) => None,
        Some(v) => {
            let secs = v.as_f64().ok_or_else(|| {
                MotionError::InvalidCommand("max_seconds must be a positive finite number".into())
            })?;
            if !secs.is_finite() || secs <= 0.0 || secs > MAX_POLICY_SECONDS {
                return Err(MotionError::InvalidCommand(format!(
                    "max_seconds must be a positive finite number <= {MAX_POLICY_SECONDS}"
                )));
            }
            Some(secs)
        }
    };

    let fps = match body.get("fps") {
        None | Some(Value::Null) => None,
        Some(v) => {
            let f = v.as_f64().ok_or_else(|| {
                MotionError::InvalidCommand("fps must be a positive finite number".into())
            })?;
            Some(validate_policy_fps(mode, f)?)
        }
    };

    let tag = match body.get("tag") {
        None | Some(Value::Null) => None,
        Some(v) => {
            let s = v.as_str().ok_or_else(|| {
                MotionError::InvalidCommand("tag must be a string".into())
            })?;
            Some(s.to_string())
        }
    };

    let i_am_at_the_arm = match body.get("i_am_at_the_arm") {
        None | Some(Value::Null) => false,
        Some(v) => v.as_bool().ok_or_else(|| {
            MotionError::InvalidCommand("i_am_at_the_arm must be a boolean".into())
        })?,
    };

    let release_gate = match body.get("release_gate") {
        None | Some(Value::Null) => None,
        Some(v) if v.is_object() => Some(v.clone()),
        Some(_) => {
            return Err(MotionError::InvalidCommand(
                "release_gate must be a JSON object".into(),
            ));
        }
    };

    Ok(PolicyStartRequest {
        policy_id: policy_id.to_string(),
        mode,
        sim_only,
        max_seconds,
        fps,
        tag,
        i_am_at_the_arm,
        release_gate,
    })
}

impl From<PolicyError> for ArmError {
    fn from(err: PolicyError) -> Self {
        let (status, code) = match &err {
            PolicyError::Io(_) => (404, "POLICY_IO_ERROR"),
            PolicyError::UnsupportedBackend(_) => (422, "UNSUPPORTED_BACKEND"),
            PolicyError::BackendLoad(_) | PolicyError::InferenceFailed(_) => {
                (422, "POLICY_LOAD_FAILED")
            }
            PolicyError::ReleaseGate(_) => (403, "RELEASE_GATE_INVALID"),
            PolicyError::InvalidPath(_)
            | PolicyError::InvalidManifest(_)
            | PolicyError::ExternalOnnxData(_)
            | PolicyError::InvalidUnits(_)
            | PolicyError::DimensionMismatch { .. }
            | PolicyError::NonFinite(_)
            | PolicyError::InvalidArtifact(_) => (422, "INVALID_POLICY"),
        };
        ArmError::Rejected {
            status,
            code: code.to_string(),
            message: err.to_string(),
        }
    }
}

/// Fully validated, preallocated policy inference pipeline ready to step against an arm config.
pub struct PreparedPolicy {
    snapshot: PolicySnapshot,
    contract: StrictPolicyContract,
    backend: Box<dyn PolicyBackend>,
    perm: JointPermutation,
    reordered_limits: Vec<JointLimit>,
    policy_gripper_idx: Option<usize>,
    gripper_bounds: Option<(f32, f32)>,
    policy_state_rad: Vec<f32>,
    infer_in: Vec<f32>,
    infer_out: Vec<f32>,
    policy_act_units: Vec<f32>,
    arm_target_rad: Vec<f32>,
}

impl PreparedPolicy {
    /// Construct a validated inference pipeline from an explicit backend and arm config.
    pub fn from_backend(
        snapshot: PolicySnapshot,
        contract: StrictPolicyContract,
        backend: Box<dyn PolicyBackend>,
        config: &ArmConfig,
    ) -> Result<Self, PolicyError> {
        if config.dof != contract.action_dim {
            return Err(PolicyError::DimensionMismatch {
                expected: config.dof,
                actual: contract.action_dim,
            });
        }
        if config.limits.len() != contract.action_dim {
            return Err(PolicyError::DimensionMismatch {
                expected: contract.action_dim,
                actual: config.limits.len(),
            });
        }
        let expected_in = contract.expected_input_dim();
        if backend.input_dim() != expected_in {
            return Err(PolicyError::DimensionMismatch {
                expected: expected_in,
                actual: backend.input_dim(),
            });
        }
        if backend.output_dim() != contract.action_dim {
            return Err(PolicyError::DimensionMismatch {
                expected: contract.action_dim,
                actual: backend.output_dim(),
            });
        }

        let perm = JointPermutation::from_arm_config(&contract, config)?;
        let reordered_limits = perm.reordered_limits(&config.limits)?;
        let policy_gripper_idx = config
            .gripper_index()
            .and_then(|arm_idx| perm.policy_index_for_arm_joint(arm_idx));
        let gripper_bounds = config
            .gripper_policy
            .as_ref()
            .map(|p| (p.closed_rad, p.open_rad));

        let dim = contract.action_dim;
        Ok(Self {
            snapshot,
            contract,
            backend,
            perm,
            reordered_limits,
            policy_gripper_idx,
            gripper_bounds,
            policy_state_rad: vec![0.0; dim],
            infer_in: vec![0.0; expected_in],
            infer_out: vec![0.0; dim],
            policy_act_units: vec![0.0; dim],
            arm_target_rad: vec![0.0; dim],
        })
    }

    pub fn artifact_digest(&self) -> &str {
        &self.snapshot.release_digest
    }

    pub fn backend_name(&self) -> &'static str {
        self.backend.backend_name()
    }

    pub fn contract(&self) -> &StrictPolicyContract {
        &self.contract
    }

    /// Run one full state -> units -> normalize -> infer -> denormalize -> radians -> reorder step.
    pub fn step_infer(&mut self, measured_arm_rad: &[f32]) -> Result<Vec<f32>, PolicyError> {
        self.step_infer_with_vision(measured_arm_rad, None)
    }

    /// Run one full inference step, optionally appending pre-extracted vision features after the
    /// normalized proprioception vector.
    pub fn step_infer_with_vision(
        &mut self,
        measured_arm_rad: &[f32],
        vision_features: Option<&[f32]>,
    ) -> Result<Vec<f32>, PolicyError> {
        let dim = self.contract.action_dim;
        let vision_dim = self.contract.vision_feature_dim();
        if measured_arm_rad.len() != dim {
            return Err(PolicyError::DimensionMismatch {
                expected: dim,
                actual: measured_arm_rad.len(),
            });
        }
        for (idx, &v) in measured_arm_rad.iter().enumerate() {
            if !v.is_finite() {
                return Err(PolicyError::NonFinite(format!(
                    "measured_positions[{idx}] is non-finite: {v}"
                )));
            }
        }
        match (vision_dim, vision_features) {
            (0, None) => {}
            (0, Some(vf)) => {
                if !vf.is_empty() {
                    return Err(PolicyError::DimensionMismatch {
                        expected: 0,
                        actual: vf.len(),
                    });
                }
            }
            (expected_v, None) => {
                return Err(PolicyError::DimensionMismatch {
                    expected: expected_v,
                    actual: 0,
                });
            }
            (expected_v, Some(vf)) => {
                if vf.len() != expected_v {
                    return Err(PolicyError::DimensionMismatch {
                        expected: expected_v,
                        actual: vf.len(),
                    });
                }
                for (idx, &v) in vf.iter().enumerate() {
                    if !v.is_finite() {
                        return Err(PolicyError::NonFinite(format!(
                            "vision_features[{idx}] is non-finite: {v}"
                        )));
                    }
                }
            }
        }

        self.perm
            .reorder_arm_to_policy(measured_arm_rad, &mut self.policy_state_rad)?;

        let state_units = rad_to_policy_units(
            &self.policy_state_rad,
            self.contract.unit_mode,
            &self.reordered_limits,
            self.policy_gripper_idx,
            self.gripper_bounds,
        )?;

        if let (Some(mean), Some(std)) = (
            &self.contract.normalization.state_mean,
            &self.contract.normalization.state_std,
        ) {
            normalize_state_into(&state_units, mean, std, &mut self.infer_in[..dim])?;
        } else {
            self.infer_in[..dim].copy_from_slice(&state_units);
        }
        if let Some(vf) = vision_features.filter(|_| vision_dim > 0) {
            self.infer_in[dim..dim + vision_dim].copy_from_slice(vf);
        }

        self.backend.infer(&self.infer_in, &mut self.infer_out)?;
        for (idx, &v) in self.infer_out.iter().enumerate() {
            if !v.is_finite() {
                return Err(PolicyError::NonFinite(format!(
                    "model output[{idx}] is non-finite: {v}"
                )));
            }
        }

        if let (Some(mean), Some(std)) = (
            &self.contract.normalization.action_mean,
            &self.contract.normalization.action_std,
        ) {
            denormalize_action_into(&self.infer_out, mean, std, &mut self.policy_act_units)?;
        } else {
            self.policy_act_units.copy_from_slice(&self.infer_out);
        }

        let act_rad = policy_units_to_rad(
            &self.policy_act_units,
            self.contract.unit_mode,
            &self.reordered_limits,
            self.policy_gripper_idx,
            self.gripper_bounds,
        )?;

        self.perm
            .reorder_policy_to_arm(&act_rad, &mut self.arm_target_rad)?;

        for (idx, &v) in self.arm_target_rad.iter().enumerate() {
            if !v.is_finite() {
                return Err(PolicyError::NonFinite(format!(
                    "arm target[{idx}] is non-finite: {v}"
                )));
            }
        }

        Ok(self.arm_target_rad.clone())
    }
}

/// Load and validate a policy pack from `dir` against `config` without holding any supervisor lock.
pub fn prepare_policy(dir: &Path, config: &ArmConfig) -> Result<PreparedPolicy, PolicyError> {
    let snapshot = PolicySnapshot::load_from_dir(dir)?;
    let contract = StrictPolicyContract::parse(&snapshot.policy_json_bytes)?;

    if config.dof != contract.action_dim {
        return Err(PolicyError::DimensionMismatch {
            expected: config.dof,
            actual: contract.action_dim,
        });
    }
    if config.limits.len() != contract.action_dim {
        return Err(PolicyError::DimensionMismatch {
            expected: contract.action_dim,
            actual: config.limits.len(),
        });
    }

    let backend = load_backend(&snapshot, &contract)?;
    PreparedPolicy::from_backend(snapshot, contract, backend, config)
}

/// Spawn the async policy runner task that executes inference outside the supervisor lock and
/// submits targets via `SupervisorInner::policy_step_with_vision`.
pub fn spawn_policy_runner(
    supervisor: Arc<ArmSupervisor>,
    action_id: u64,
    mut prepared: PreparedPolicy,
    fps: f64,
    deadline: Option<Instant>,
    vision_source: Option<Arc<dyn PolicyVisionSource>>,
) -> tokio::task::JoinHandle<()> {
    let vision_spec = prepared.contract().vision.clone();
    if vision_spec.is_some() && vision_source.is_none() {
        return tokio::spawn(async move {
            let mut inner = supervisor.inner().write().await;
            inner.fail_policy_run(
                action_id,
                "VISION_UNAVAILABLE",
                "Policy requires vision input, but no vision source is available",
            );
        });
    }

    let clamped_fps = if fps.is_finite() && fps > 0.0 {
        fps.clamp(MIN_POLICY_FPS, MAX_POLICY_FPS)
    } else {
        DEFAULT_POLICY_FPS
    };
    let period = Duration::from_secs_f64(1.0 / clamped_fps);
    let tokio_dl = deadline.map(tokio::time::Instant::from_std);

    tokio::spawn(async move {
        supervisor.inner().write().await.set_policy_max_input_age(action_id, max_policy_input_age_ms(clamped_fps));
        let collector = vision_spec.as_ref().zip(vision_source).map(|(spec, src)| {
            let wait_ms = (spec.max_staleness_ms / 4).clamp(10, 50);
            VisionFeatureCollector::start(src, wait_ms)
        });
        let run_started_at = Instant::now();
        let mut locked_generation: Option<u64> = None;

        let mut interval = tokio::time::interval(period);
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

        loop {
            if let Some(dl) = tokio_dl {
                tokio::select! {
                    biased;
                    _ = tokio::time::sleep_until(dl) => {
                        let mut inner = supervisor.inner().write().await;
                        inner.finish_policy_run(action_id, "MAX_SECONDS");
                        break;
                    }
                    _ = interval.tick() => {}
                }
            } else {
                interval.tick().await;
            }
            if let Some(dl) = deadline {
                if Instant::now() >= dl {
                    let mut inner = supervisor.inner().write().await;
                    inner.finish_policy_run(action_id, "MAX_SECONDS");
                    break;
                }
            }

            let (measured, prop_age_range_ms, captured_at) = {
                let inner = supervisor.inner().read().await;
                if !inner.is_policy_run_active(action_id) {
                    break;
                }
                (
                    inner.measured_positions.clone(),
                    inner.policy_input_age_range_ms(),
                    Instant::now(),
                )
            };
            let base_input_age_ms = prop_age_range_ms.map(|(_, max_prop)| max_prop);

            let (vision_features, vision_captured_at, vision_frame_id) =
                if let (Some(spec), Some(col)) = (&vision_spec, &collector) {
                    let frame = col.latest();
                    let now = Instant::now();
                    let prop_elapsed_ms = now
                        .saturating_duration_since(captured_at)
                        .as_millis()
                        .min(u64::MAX as u128) as u64;
                    let synced_prop_age_range_ms = prop_age_range_ms.map(|(lo, hi)| {
                        (
                            lo.saturating_add(prop_elapsed_ms),
                            hi.saturating_add(prop_elapsed_ms),
                        )
                    });
                    match evaluate_vision_frame(
                        frame.as_ref(),
                        spec,
                        run_started_at,
                        now,
                        synced_prop_age_range_ms,
                        &mut locked_generation,
                    ) {
                        VisionStepDecision::Ready {
                            features,
                            vision_age_ms: _,
                            frame_id,
                        } => (
                            Some(features),
                            frame.as_ref().and_then(|f| f.captured_at),
                            Some(frame_id),
                        ),
                        VisionStepDecision::Hold => {
                            supervisor.inner().write().await.hold_policy_run(action_id);
                            continue;
                        }
                        VisionStepDecision::Stop { reason, message } => {
                            let mut inner = supervisor.inner().write().await;
                            inner.fail_policy_run(action_id, reason, message);
                            break;
                        }
                    }
                } else {
                    (None, None, None)
                };

            let infer_handle = tokio::task::spawn_blocking(move || {
                let res = prepared.step_infer_with_vision(
                    &measured,
                    vision_features.as_ref().map(|f| f.as_slice()),
                );
                (prepared, res)
            });
            let infer_result = if let Some(dl) = tokio_dl {
                tokio::select! {
                    biased;
                    _ = tokio::time::sleep_until(dl) => {
                        let mut inner = supervisor.inner().write().await;
                        inner.finish_policy_run(action_id, "MAX_SECONDS");
                        break;
                    }
                    res = infer_handle => res,
                }
            } else {
                infer_handle.await
            };

            let target = match infer_result {
                Err(e) => {
                    tracing::warn!("Policy inference task failed for action {action_id}: {e}");
                    let mut inner = supervisor.inner().write().await;
                    inner.fail_policy_run(
                        action_id,
                        "INFERENCE_ERROR",
                        format!("inference task failed: {e}"),
                    );
                    break;
                }
                Ok((p, res)) => {
                    prepared = p;
                    match res {
                        Ok(t) => t,
                        Err(e) => {
                            tracing::warn!("Policy inference failed for action {action_id}: {e}");
                            let reason = match e {
                                PolicyError::NonFinite(_) => "NON_FINITE",
                                _ => "INFERENCE_ERROR",
                            };
                            let mut inner = supervisor.inner().write().await;
                            inner.fail_policy_run(action_id, reason, e.to_string());
                            break;
                        }
                    }
                }
            };

            let mut inner = supervisor.inner().write().await;
            if let Some(dl) = deadline {
                if Instant::now() >= dl {
                    inner.finish_policy_run(action_id, "MAX_SECONDS");
                    break;
                }
            }
            let post_vision_age_ms = if let (Some(spec), Some(vis_cap)) =
                (&vision_spec, vision_captured_at)
            {
                let age_ms = Instant::now()
                    .saturating_duration_since(vis_cap)
                    .as_millis()
                    .min(u64::MAX as u128) as u64;
                if age_ms > spec.max_staleness_ms {
                    match spec.on_stale {
                        VisionStalePolicy::Stop => {
                            inner.fail_policy_run(
                                action_id,
                                "VISION_STALE",
                                format!(
                                    "Vision frame became stale before step commit ({age_ms} ms > {} ms)",
                                    spec.max_staleness_ms
                                ),
                            );
                            break;
                        }
                        VisionStalePolicy::Hold => {
                            inner.hold_policy_run(action_id);
                            continue;
                        }
                    }
                }
                Some(age_ms)
            } else {
                None
            };
            if let (Some(spec), Some(col)) = (&vision_spec, &collector) {
                if spec.require_model_generation {
                    if let Some(latest_frame) = col.latest() {
                        if latest_frame.model_generation != locked_generation {
                            let (reason, message) = match latest_frame.model_generation {
                                None => (
                                    "VISION_GENERATION_MISSING",
                                    format!(
                                        "Vision frame {} missing required model_generation before step commit",
                                        latest_frame.frame_id
                                    ),
                                ),
                                Some(new_gen) => (
                                    "VISION_GENERATION_CHANGED",
                                    format!(
                                        "Vision model generation changed from {} to {new_gen} before step commit",
                                        locked_generation
                                            .map_or_else(|| "none".to_string(), |g| g.to_string())
                                    ),
                                ),
                            };
                            inner.fail_policy_run(action_id, reason, message);
                            break;
                        }
                    }
                }
            }
            let input_age_ms = base_input_age_ms.map(|base| {
                base.saturating_add(captured_at.elapsed().as_millis().min(u64::MAX as u128) as u64)
            });
            if let Err(e) = inner.policy_step_with_vision(
                action_id,
                target,
                input_age_ms,
                post_vision_age_ms,
                vision_frame_id,
            ) {
                tracing::info!("Policy runner {action_id} exited on step: {e}");
                break;
            }
        }
    })
}
