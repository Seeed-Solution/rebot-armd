//! Async policy runner, prepared policy pipeline, and supervisor `PolicyRun` state.

use std::path::Path;
use std::sync::Arc;
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::api::sandbox::validate_pack_id;
use crate::model::{ArmConfig, JointLimit, PolicyObservation};
use crate::motion::MotionError;
use crate::supervisor::{ArmError, ArmSupervisor, SupervisorInner};

use super::manifest::{PolicySnapshot, StrictPolicyContract, VideoEmbeddingSpec, VisionStalePolicy};
use super::release_gate::VerifiedReleaseGate;
use super::units::{
    denormalize_action_into, normalize_state_into, policy_units_to_rad, rad_to_policy_units,
    JointPermutation,
};
use super::vision::{
    embedding_hub_for_supervisor, evaluate_vision_frame, EmbeddingSampleDecision, HubCommitGuard,
    PolicyVisionSource, VideoEmbeddingHub, VisionFeatureCollector, VisionStepDecision,
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
/// Warm-up inferences run before a policy run starts; the first one is discarded.
pub const WARMUP_INFERENCES: usize = 10;
/// Upper bound of the whole warm-up. Inference cannot be interrupted, so the warm-up stops
/// after the inference that crosses this bound and reports the samples taken so far.
pub const WARMUP_MAX_TOTAL: Duration = Duration::from_secs(5);
/// A live run is refused when the warm-up p50 exceeds this fraction of the period.
pub const SPEED_GATE_PERIOD_FRACTION: f64 = 0.5;
/// Length of the per-step inference-time ring behind `policy_stats.infer_us`.
pub const POLICY_STATS_RING: usize = 256;

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
    /// Per-step timing behind `ActionStatus::policy_stats`.
    pub step_stats: PolicyStepStats,
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

/// Warm-up inference time, published in the start ACK as `warmup_infer_us`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct WarmupInferUs {
    pub p50: u64,
    pub max: u64,
    /// Samples behind `p50` / `max` (the first inference is discarded when more than one ran).
    pub n: u64,
}

/// Result of the start speed gate: the warm-up against the run's period.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SpeedGateReport {
    pub warmup: WarmupInferUs,
    pub period_us: u64,
    /// Highest rate the model holds by the gate rule: floor(0.5 / p50).
    pub max_fps: u64,
    /// p50 above [`SPEED_GATE_PERIOD_FRACTION`] of the period.
    pub too_slow: bool,
}

impl SpeedGateReport {
    pub fn new(warmup: WarmupInferUs, fps: f64) -> Self {
        let period_us = period_us_for_fps(fps);
        let too_slow = warmup.p50 as f64 > SPEED_GATE_PERIOD_FRACTION * period_us as f64;
        let max_fps = (SPEED_GATE_PERIOD_FRACTION * 1e6 / warmup.p50.max(1) as f64).floor() as u64;
        Self { warmup, period_us, max_fps, too_slow }
    }

    /// A live run (any mode that commands the arm, `live_sim` included) is refused with
    /// `POLICY_TOO_SLOW` when the model is too slow; a shadow run starts.
    pub fn check(&self, mode: PolicyMode) -> Result<(), ArmError> {
        if mode != PolicyMode::Live || !self.too_slow {
            return Ok(());
        }
        Err(ArmError::Rejected {
            status: 422,
            code: "POLICY_TOO_SLOW".into(),
            message: format!(
                "Policy inference is too slow for this rate: warm-up p50 {} us, max {} us (n={}) exceeds 50% of the {} us period; this model holds at most {} fps on this device",
                self.warmup.p50, self.warmup.max, self.warmup.n, self.period_us, self.max_fps
            ),
        })
    }

    /// Fields added to the start ACK (and to a `POLICY_TOO_SLOW` reject).
    pub fn ack_fields(&self) -> [(&'static str, Value); 4] {
        [
            ("warmup_infer_us", serde_json::json!(self.warmup)),
            ("too_slow", Value::Bool(self.too_slow)),
            ("period_us", serde_json::json!(self.period_us)),
            ("max_fps", serde_json::json!(self.max_fps)),
        ]
    }
}

/// Period in microseconds for a positive finite rate (0 otherwise).
pub fn period_us_for_fps(fps: f64) -> u64 {
    if !fps.is_finite() || fps <= 0.0 {
        return 0;
    }
    (1e6 / fps).round() as u64
}

/// Per-step timing of a policy run: counters plus a fixed ring of the last
/// [`POLICY_STATS_RING`] inference times. No allocation per step.
#[derive(Debug, Clone)]
pub struct PolicyStepStats {
    ring: [u32; POLICY_STATS_RING],
    len: usize,
    head: usize,
    steps: u64,
    overruns: u64,
    skipped: u64,
    period_us: u64,
}

impl Default for PolicyStepStats {
    fn default() -> Self {
        Self { ring: [0; POLICY_STATS_RING], len: 0, head: 0, steps: 0, overruns: 0, skipped: 0, period_us: 0 }
    }
}

impl PolicyStepStats {
    pub fn set_period_us(&mut self, period_us: u64) {
        self.period_us = period_us;
    }

    /// One inference done. `infer_us` is the inference alone; `step_us` runs from the period
    /// tick to the commit point (inference + commit). A step longer than the period is an
    /// overrun; the runner skips missed ticks, so floor(step_us / period_us) periods get no target.
    pub fn record(&mut self, infer_us: u64, step_us: u64) {
        self.steps += 1;
        self.ring[self.head] = infer_us.min(u32::MAX as u64) as u32;
        self.head = (self.head + 1) % POLICY_STATS_RING;
        self.len = (self.len + 1).min(POLICY_STATS_RING);
        if self.period_us > 0 && step_us > self.period_us {
            self.overruns += 1;
            self.skipped += step_us / self.period_us;
        }
    }

    pub fn report(&self) -> crate::motion::PolicyStats {
        let mut sorted = [0u32; POLICY_STATS_RING];
        let n = self.len;
        sorted[..n].copy_from_slice(&self.ring[..n]);
        sorted[..n].sort_unstable();
        let rank = |q: f64| -> u64 {
            if n == 0 {
                return 0;
            }
            let idx = ((q * n as f64).ceil() as usize).clamp(1, n) - 1;
            sorted[idx] as u64
        };
        crate::motion::PolicyStats {
            steps: self.steps,
            overruns: self.overruns,
            skipped: self.skipped,
            infer_us: crate::motion::PolicyInferUs {
                p50: rank(0.5),
                p99: rank(0.99),
                max: if n == 0 { 0 } else { sorted[n - 1] as u64 },
            },
            period_us: self.period_us,
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
            PolicyError::ResourceBudget(_) => (413, "POLICY_TOO_LARGE"),
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

    /// Test hook: every inference sleeps `delay` first (see
    /// `SandboxState::set_policy_infer_delay_ms`).
    #[doc(hidden)]
    pub fn with_infer_delay(mut self, delay: Duration) -> Self {
        struct Delayed(Box<dyn PolicyBackend>, Duration);
        impl PolicyBackend for Delayed {
            fn backend_name(&self) -> &'static str { self.0.backend_name() }
            fn input_dim(&self) -> usize { self.0.input_dim() }
            fn output_dim(&self) -> usize { self.0.output_dim() }
            fn infer(&mut self, input: &[f32], output: &mut [f32]) -> Result<(), PolicyError> {
                std::thread::sleep(self.1);
                self.0.infer(input, output)
            }
        }
        let inner = self.backend;
        self.backend = Box::new(Delayed(inner, delay));
        self
    }

    /// Run `n` inferences on the model's input shape and measure them; the first one is
    /// discarded when more than one ran. Stops early once `max_total` has elapsed.
    pub fn warmup(&mut self, n: usize, max_total: Duration) -> Result<WarmupInferUs, PolicyError> {
        let n = n.clamp(1, WARMUP_INFERENCES);
        let mut samples = [0u64; WARMUP_INFERENCES];
        let mut taken = 0usize;
        let started = Instant::now();
        for _ in 0..n {
            let t0 = Instant::now();
            self.backend.infer(&self.infer_in, &mut self.infer_out)?;
            samples[taken] = t0.elapsed().as_micros().min(u64::MAX as u128) as u64;
            taken += 1;
            if started.elapsed() >= max_total {
                break;
            }
        }
        self.infer_out.fill(0.0);
        let kept = if taken > 1 { &mut samples[1..taken] } else { &mut samples[..taken] };
        kept.sort_unstable();
        Ok(WarmupInferUs {
            p50: kept[(kept.len() - 1) / 2],
            max: kept[kept.len() - 1],
            n: kept.len() as u64,
        })
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

/// Immutable per-step inference ticket captured before inference and re-verified
/// synchronously inside the target-commit critical section before emitting any target.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InferenceTicket {
    policy_instance_id: u64,
    lease_generation: u64,
    producer_epoch: u64,
    model_generation: Option<u64>,
    invalidation_generation: u64,
    clock_handshake_generation: u64,
    clock_handshake_expires_at_ms: u64,
    window_newest_deadline_ms: u64,
    window_oldest_deadline_ms: u64,
}

impl InferenceTicket {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        policy_instance_id: u64,
        lease_generation: u64,
        producer_epoch: u64,
        model_generation: Option<u64>,
        invalidation_generation: u64,
        clock_handshake_generation: u64,
        clock_handshake_expires_at_ms: u64,
        window_newest_deadline_ms: u64,
        window_oldest_deadline_ms: u64,
    ) -> Self {
        Self {
            policy_instance_id,
            lease_generation,
            producer_epoch,
            model_generation,
            invalidation_generation,
            clock_handshake_generation,
            clock_handshake_expires_at_ms,
            window_newest_deadline_ms,
            window_oldest_deadline_ms,
        }
    }

    pub fn policy_instance_id(&self) -> u64 {
        self.policy_instance_id
    }

    pub fn lease_generation(&self) -> u64 {
        self.lease_generation
    }

    pub fn producer_epoch(&self) -> u64 {
        self.producer_epoch
    }

    pub fn model_generation(&self) -> Option<u64> {
        self.model_generation
    }

    pub fn invalidation_generation(&self) -> u64 {
        self.invalidation_generation
    }

    pub fn clock_handshake_generation(&self) -> u64 {
        self.clock_handshake_generation
    }

    pub fn clock_handshake_expires_at_ms(&self) -> u64 {
        self.clock_handshake_expires_at_ms
    }

    pub fn window_newest_deadline_ms(&self) -> u64 {
        self.window_newest_deadline_ms
    }

    pub fn window_oldest_deadline_ms(&self) -> u64 {
        self.window_oldest_deadline_ms
    }

    pub fn verify(&self, live: &TicketVerificationContext) -> Result<(), TicketViolation> {
        if live.policy_instance_id != Some(self.policy_instance_id) {
            return Err(TicketViolation::PolicyInstanceMismatch {
                expected: self.policy_instance_id,
                actual: live.policy_instance_id,
            });
        }
        if live.lease_generation != Some(self.lease_generation) {
            return Err(TicketViolation::LeaseGenerationMismatch {
                expected: self.lease_generation,
                actual: live.lease_generation,
            });
        }
        if live.producer_epoch != Some(self.producer_epoch) {
            return Err(TicketViolation::ProducerEpochMismatch {
                expected: self.producer_epoch,
                actual: live.producer_epoch,
            });
        }
        if live.model_generation != self.model_generation {
            return Err(TicketViolation::ModelGenerationMismatch {
                expected: self.model_generation,
                actual: live.model_generation,
            });
        }
        if live.invalidation_generation != self.invalidation_generation {
            return Err(TicketViolation::InvalidationGenerationMismatch {
                expected: self.invalidation_generation,
                actual: live.invalidation_generation,
            });
        }
        if live.now_armd_ms > self.clock_handshake_expires_at_ms {
            return Err(TicketViolation::ClockHandshakeExpired {
                now_ms: live.now_armd_ms,
                expires_at_ms: self.clock_handshake_expires_at_ms,
            });
        }
        if live.clock_handshake_generation != self.clock_handshake_generation {
            return Err(TicketViolation::ClockHandshakeGenerationMismatch {
                expected: self.clock_handshake_generation,
                actual: live.clock_handshake_generation,
            });
        }
        if live.now_armd_ms > self.window_newest_deadline_ms {
            return Err(TicketViolation::WindowNewestExpired {
                now_ms: live.now_armd_ms,
                deadline_ms: self.window_newest_deadline_ms,
            });
        }
        if live.now_armd_ms > self.window_oldest_deadline_ms {
            return Err(TicketViolation::WindowOldestExpired {
                now_ms: live.now_armd_ms,
                deadline_ms: self.window_oldest_deadline_ms,
            });
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TicketVerificationContext {
    pub policy_instance_id: Option<u64>,
    pub lease_generation: Option<u64>,
    pub producer_epoch: Option<u64>,
    pub model_generation: Option<u64>,
    pub invalidation_generation: u64,
    pub clock_handshake_generation: u64,
    pub now_armd_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TicketViolation {
    PolicyInstanceMismatch { expected: u64, actual: Option<u64> },
    LeaseGenerationMismatch { expected: u64, actual: Option<u64> },
    ProducerEpochMismatch { expected: u64, actual: Option<u64> },
    ModelGenerationMismatch { expected: Option<u64>, actual: Option<u64> },
    InvalidationGenerationMismatch { expected: u64, actual: u64 },
    ClockHandshakeGenerationMismatch { expected: u64, actual: u64 },
    ClockHandshakeExpired { now_ms: u64, expires_at_ms: u64 },
    WindowNewestExpired { now_ms: u64, deadline_ms: u64 },
    WindowOldestExpired { now_ms: u64, deadline_ms: u64 },
}

impl TicketViolation {
    pub fn code(&self) -> &'static str {
        match self {
            Self::PolicyInstanceMismatch { .. } => "POLICY_INSTANCE_CHANGED",
            Self::LeaseGenerationMismatch { .. } => "LEASE_LOST",
            Self::ProducerEpochMismatch { .. } => "PRODUCER_EPOCH_CHANGED",
            Self::ModelGenerationMismatch { .. } => "VISION_GENERATION_CHANGED",
            Self::InvalidationGenerationMismatch { .. } => "WINDOW_INVALIDATED",
            Self::ClockHandshakeGenerationMismatch { .. } => "HANDSHAKE_MISMATCH",
            Self::ClockHandshakeExpired { .. } => "HANDSHAKE_EXPIRED",
            Self::WindowNewestExpired { .. } | Self::WindowOldestExpired { .. } => "VISION_STALE",
        }
    }

    pub fn message(&self) -> String {
        match self {
            Self::PolicyInstanceMismatch { expected, actual } => format!(
                "InferenceTicket policy_instance_id changed mid-inference (expected {expected}, got {actual:?})"
            ),
            Self::LeaseGenerationMismatch { expected, actual } => format!(
                "InferenceTicket lease_generation changed mid-inference (expected {expected}, got {actual:?})"
            ),
            Self::ProducerEpochMismatch { expected, actual } => format!(
                "InferenceTicket producer_epoch changed mid-inference (expected {expected}, got {actual:?})"
            ),
            Self::ModelGenerationMismatch { expected, actual } => format!(
                "InferenceTicket model_generation changed mid-inference (expected {expected:?}, got {actual:?})"
            ),
            Self::InvalidationGenerationMismatch { expected, actual } => format!(
                "InferenceTicket invalidation_generation changed mid-inference (expected {expected}, got {actual})"
            ),
            Self::ClockHandshakeGenerationMismatch { expected, actual } => format!(
                "InferenceTicket clock_handshake_generation changed mid-inference (expected {expected}, got {actual})"
            ),
            Self::ClockHandshakeExpired { now_ms, expires_at_ms } => format!(
                "InferenceTicket clock handshake expired before commit ({now_ms} ms > {expires_at_ms} ms)"
            ),
            Self::WindowNewestExpired { now_ms, deadline_ms } => format!(
                "InferenceTicket window_newest_deadline_ms expired before commit ({now_ms} ms > {deadline_ms} ms)"
            ),
            Self::WindowOldestExpired { now_ms, deadline_ms } => format!(
                "InferenceTicket window_oldest_deadline_ms expired before commit ({now_ms} ms > {deadline_ms} ms)"
            ),
        }
    }
}

fn active_run_lease_generation(inner: &SupervisorInner, action_id: u64) -> Option<u64> {
    let run = inner.policy_run.as_ref().filter(|r| r.action_id == action_id)?;
    let lease_gen = inner.active_lease.as_ref().map(|l| l.generation)?;
    (run.lease_generation == lease_gen).then_some(lease_gen)
}

fn verify_ticket_in_critical_section(
    inner: &SupervisorInner,
    action_id: u64,
    ticket: &InferenceTicket,
    hub_guard: &HubCommitGuard<'_>,
) -> Result<u64, TicketViolation> {
    let live_hub = hub_guard.live_ticket_state();
    let live_lease = active_run_lease_generation(inner, action_id);
    let ctx = TicketVerificationContext {
        policy_instance_id: live_hub.policy_instance_id,
        lease_generation: live_lease,
        producer_epoch: live_hub.producer_epoch,
        model_generation: live_hub.model_generation,
        invalidation_generation: live_hub.invalidation_generation,
        clock_handshake_generation: live_hub.clock_handshake_generation,
        now_armd_ms: live_hub.now_armd_ms,
    };
    ticket.verify(&ctx)?;
    Ok(live_hub.now_armd_ms)
}

fn verify_embedding_release_gate(
    gate: &VerifiedReleaseGate,
    emb: &VideoEmbeddingSpec,
) -> Result<(), String> {
    if gate.encoder_artifact_sha256() != Some(emb.encoder_artifact_sha256.as_str()) {
        return Err(format!(
            "release gate encoder_artifact_sha256 ({:?}) does not match active policy ({})",
            gate.encoder_artifact_sha256(),
            emb.encoder_artifact_sha256
        ));
    }
    if gate.preprocess_config_sha256() != Some(emb.preprocess_config_sha256.as_str()) {
        return Err(format!(
            "release gate preprocess_config_sha256 ({:?}) does not match active policy ({})",
            gate.preprocess_config_sha256(),
            emb.preprocess_config_sha256
        ));
    }
    Ok(())
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
    sandbox: Option<Arc<crate::api::sandbox::SandboxState>>,
) -> tokio::task::JoinHandle<()> {
    let video_embedding_spec = prepared.contract().video_embedding.clone();
    let vision_spec = prepared.contract().vision.clone();
    if video_embedding_spec.is_none() && vision_spec.is_some() && vision_source.is_none() {
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
        struct UnbindGuard {
            hub: Arc<VideoEmbeddingHub>,
            action_id: u64,
        }
        impl Drop for UnbindGuard {
            fn drop(&mut self) {
                self.hub.unbind_policy(self.action_id);
            }
        }

        let embedding_hub = video_embedding_spec
            .as_ref()
            .map(|_| embedding_hub_for_supervisor(&supervisor));

        let _unbind_guard = {
            let mut inner = supervisor.inner().write().await;
            if !inner.is_policy_run_active(action_id) {
                return;
            }
            if let (Some(ref emb), Some(gate)) = (
                &video_embedding_spec,
                inner.active_policy_release_gate(action_id),
            ) {
                if let Err(msg) = verify_embedding_release_gate(gate, emb) {
                    inner.fail_policy_run(action_id, "RELEASE_GATE_INVALID", msg);
                    return;
                }
            }
            inner.set_policy_max_input_age(action_id, max_policy_input_age_ms(clamped_fps));
            inner.set_policy_period_us(action_id, period_us_for_fps(clamped_fps));

            embedding_hub
                .as_ref()
                .zip(video_embedding_spec.as_ref())
                .map(|(hub, spec)| {
                    hub.bind_policy(action_id, spec.clone(), clamped_fps);
                    UnbindGuard {
                        hub: hub.clone(),
                        action_id,
                    }
                })
        };
        let emb_run_started_ms = embedding_hub
            .as_ref()
            .map_or(0, |h| h.now_armd_ms());

        let collector = if video_embedding_spec.is_none() {
            vision_spec.as_ref().zip(vision_source).map(|(spec, src)| {
                let wait_ms = (spec.max_staleness_ms / 4).clamp(10, 50);
                VisionFeatureCollector::start(src, wait_ms)
            })
        } else {
            None
        };
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
            let step_started_at = Instant::now();
            if let Some(dl) = deadline {
                if Instant::now() >= dl {
                    let mut inner = supervisor.inner().write().await;
                    inner.finish_policy_run(action_id, "MAX_SECONDS");
                    break;
                }
            }

            let (measured, prop_age_range_ms, captured_at, snap_lease_gen) = {
                let inner = supervisor.inner().read().await;
                if !inner.is_policy_run_active(action_id) {
                    break;
                }
                (
                    inner.measured_positions.clone(),
                    inner.policy_input_age_range_ms(),
                    Instant::now(),
                    active_run_lease_generation(&inner, action_id),
                )
            };
            let base_input_age_ms = prop_age_range_ms.map(|(_, max_prop)| max_prop);

            let (vision_features, vision_captured_at, vision_frame_id, emb_ticket, emb_mapped_and_unc_ms) =
                if let (Some(emb_spec), Some(hub)) = (&video_embedding_spec, &embedding_hub) {
                    let Some(lease_gen) = snap_lease_gen else {
                        let mut inner = supervisor.inner().write().await;
                        inner.fail_policy_run(
                            action_id,
                            "LEASE_LOST",
                            "Control lease lost or expired before embedding step",
                        );
                        break;
                    };
                    match hub.sample_window(emb_spec, emb_run_started_ms) {
                        EmbeddingSampleDecision::Ready(win) => {
                            let ticket = InferenceTicket::new(
                                action_id,
                                lease_gen,
                                win.producer_epoch,
                                win.model_generation,
                                win.invalidation_generation,
                                win.clock_handshake_generation,
                                win.clock_handshake_expires_at_ms,
                                win.window_newest_deadline_ms,
                                win.window_oldest_deadline_ms,
                            );
                            (
                                Some(win.features),
                                None,
                                Some(win.newest_frame_id),
                                Some(ticket),
                                Some((win.newest_oldest_mapped_at_ms, win.effective_uncertainty_ms)),
                            )
                        }
                        EmbeddingSampleDecision::Hold => {
                            supervisor.inner().write().await.hold_policy_run(action_id);
                            continue;
                        }
                        EmbeddingSampleDecision::Stop { reason, message } => {
                            let mut inner = supervisor.inner().write().await;
                            inner.fail_policy_run(action_id, reason, message);
                            break;
                        }
                    }
                } else if let (Some(spec), Some(col)) = (&vision_spec, &collector) {
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
                            Some(features.to_vec()),
                            frame.as_ref().and_then(|f| f.captured_at),
                            Some(frame_id),
                            None,
                            None,
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
                    (None, None, None, None, None)
                };

            let infer_handle = tokio::task::spawn_blocking(move || {
                let t0 = Instant::now();
                let res = prepared.step_infer_with_vision(
                    &measured,
                    vision_features.as_deref(),
                );
                let infer_us = t0.elapsed().as_micros().min(u64::MAX as u128) as u64;
                (prepared, res.map(|t| (t, infer_us)))
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

            let (target, infer_us) = match infer_result {
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
            // Read active deployment context under the supervisor write lock before
            // acquiring the synchronous hub commit lock (consistent lock order, no `.await`
            // while holding `HubCommitGuard`).
            let active_gate = inner.active_policy_release_gate(action_id).cloned();
            let sandbox_scene_tool = match (active_gate.is_some(), sandbox.as_ref()) {
                (true, Some(s)) => Some(s.active_scene_and_tool().await),
                _ => None,
            };

            // Synchronous target-commit checks under the supervisor write lock.
            let step_us = step_started_at.elapsed().as_micros().min(u64::MAX as u128) as u64;
            inner.record_policy_step_timing(action_id, infer_us, step_us);

            let detector_post_vision_age_ms = if video_embedding_spec.is_none() {
                let post_age = if let (Some(spec), Some(vis_cap)) = (&vision_spec, vision_captured_at) {
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
                post_age
            } else {
                None
            };
            if let Some(gate) = active_gate {
                if let Some(ref emb) = video_embedding_spec {
                    if let Err(msg) = verify_embedding_release_gate(&gate, emb) {
                        inner.fail_policy_run(action_id, "RELEASE_GATE_INVALID", msg);
                        break;
                    }
                }
                let (scene_id, tool_id) = match sandbox_scene_tool {
                    Some((s, t)) => (s, t),
                    None => (gate.scene_id().to_string(), gate.tool_id().to_string()),
                };
                let calibration_digest = inner.active_calibration_digest();
                let cur_ctx = super::release_gate::ActiveDeploymentContext {
                    scene_id,
                    tool_id,
                    calibration_digest,
                };
                if let Err(e) = gate.verify_deployment(&cur_ctx) {
                    inner.fail_policy_run(action_id, "DEPLOYMENT_CHANGED", e.to_string());
                    break;
                }
            }

            // Final critical-section check (A-R2-1): hold `HubCommitGuard` together with
            // `inner`, re-verify ALL `InferenceTicket` fields + monotonic deadlines AFTER
            // calibration hashing / release-gate / deployment checks, and immediately commit
            // `policy_step_with_vision` with zero intervening work.
            let _hub_guard = embedding_hub.as_ref().map(|hub| hub.lock_for_commit());
            let mut hold_step = false;
            let hold_on_stale = video_embedding_spec
                .as_ref()
                .is_some_and(|s| s.on_stale == VisionStalePolicy::Hold);
            let emb_commit_now_ms = if let (Some(ref ticket), Some(ref guard)) = (&emb_ticket, &_hub_guard) {
                match verify_ticket_in_critical_section(&inner, action_id, ticket, guard) {
                    Ok(now_ms) => Some(now_ms),
                    Err(TicketViolation::ClockHandshakeGenerationMismatch { expected, actual })
                        if actual != 0 && guard.is_unbroken_active_renewal(expected) =>
                    {
                        hold_step = true;
                        None
                    }
                    Err(
                        TicketViolation::WindowNewestExpired { .. }
                        | TicketViolation::WindowOldestExpired { .. },
                    ) if hold_on_stale => {
                        hold_step = true;
                        None
                    }
                    Err(violation) => {
                        inner.fail_policy_run(action_id, violation.code(), violation.message());
                        break;
                    }
                }
            } else {
                None
            };
            if hold_step {
                inner.hold_policy_run(action_id);
                continue;
            }
            let post_vision_age_ms = if let (Some(now_ms), Some((mapped_ms, unc_ms))) =
                (emb_commit_now_ms, emb_mapped_and_unc_ms)
            {
                Some(now_ms.saturating_sub(mapped_ms).saturating_add(unc_ms))
            } else {
                detector_post_vision_age_ms
            };
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
